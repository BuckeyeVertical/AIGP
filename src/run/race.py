"""Autonomous full-course race runner.

Flies the whole VQ1 course as waypoint corridor flight on LOCAL_POSITION_NED,
sequenced by the sim's official race-status feed (see race_data.py):

  * The course map (gate NED positions) comes from the sim's track-data
    broadcast when available, else from the baked-in fallback table below
    (the environment is deterministic and identical for all participants).
  * The drone holds yaw ~= pi (spawn heading; the whole course runs along -X)
    and flies a constant forward speed with P-corrections on cross-track
    y/z toward each gate's opening center, exactly like the successful
    manual run (109.6 s), which never yawed.
  * Gate advancement is driven by ``active_gate_index`` from race status
    (authoritative -- the sim decides what counts as a pass), with a
    position fallback when the feed is absent. Passing the gate plane
    without the index advancing triggers a go-around.
  * ``race_finish_time_ns > 0`` ends the run: brake to hover, report the
    official time, exit.

Run (on the sim host):

    python src\\run\\race.py --log-dir auto
    python src\\run\\race.py --reset --max-seconds 300

No camera stream is needed; the loop paces itself at 30 Hz (< 100 Hz spec).
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
import os
from pathlib import Path
import sys
import time

_SRC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_SRC, "control"))
sys.path.insert(0, os.path.join(_SRC, "mavlink_client"))

from control import (  # noqa: E402
    TAKEOFF_ALT,
    TAKEOFF_THRUST,
    TAKEOFF_TIMEOUT_S,
    Telemetry,
    VelocityCommand,
    VelocityToRates,
    clamp,
)
from mavlink_client import MAVLinkClient  # noqa: E402
from race_data import Gate, RaceDataTracker  # noqa: E402

CONTROL_HZ = 30.0
TELEMETRY_TIMEOUT_S = 5.0

# Course map fallback, decoded from the sim's own track-data broadcast during
# the 2026-06-19 manual run (spawn-relative NED, z = gate bottom edge). Used
# when the live broadcast is missing or arrives in a stale pre-reset frame.
FALLBACK_GATES = [
    Gate(0, -23.30, -0.40, -0.03),
    Gate(1, -46.89, -2.50, 5.07),
    Gate(2, -74.59, 1.20, 13.67),
    Gate(3, -111.49, -5.10, 24.57),
    Gate(4, -135.49, -0.80, 25.36),
    Gate(5, -159.19, -4.40, 25.97),
]
# Live track data is adopted only if its gate 0 sits near the fallback's:
# broadcasts sent around a sim reset can be in the previous session's origin.
TRACK_ACCEPT_TOL_M = 5.0

# --- corridor flight tuning -------------------------------------------------------
YAW_REF = math.pi  # spawn heading; the course runs straight down -X
K_YAW_HOLD = 0.8  # rad/s per rad of heading error
MAX_YAW_RATE = 0.5
V_CRUISE = 2.5  # forward speed (the manual run averaged ~2.3 and passed all)
V_SLOW = 1.0  # forward speed when close to a gate but not yet lined up
SLOW_DIST_M = 8.0  # "close to a gate" for the slowdown rule
CROSS_TOL_M = 0.5  # lined up = cross-track error under this
KP_POS = 0.8  # m/s of correction per m of cross-track error
MAX_V_CROSS = 1.2  # cap on lateral/vertical correction speed
# Never descend more than this below the current target's opening center:
# the terrain follows the course down, and gate z is all the ground truth
# we have for it.
DEPTH_MARGIN_M = 2.0

# --- gate advancement / go-around -------------------------------------------------
PLANE_GRACE_M = 1.5  # past the plane this far with no index advance = missed
GOAROUND_BACKOFF_M = 6.0  # retry from this far in front of the missed gate
FINISH_BRAKE_S = 2.0  # hover-brake duration after the finish signal

# Teleport detection: LOCAL_POSITION_NED jumping this far in one 30 Hz tick
# means the sim respawned us (out-of-bounds / reset), not flight.
TELEPORT_M = 15.0

LOG_COLUMNS = (
    "t", "phase", "gate", "x", "y", "z", "yaw",
    "tx", "ty", "tz", "dist_plane", "cross_err",
    "cmd_vx", "cmd_vy", "cmd_vz", "cmd_yaw_rate", "thrust",
    "race_idx", "race_started", "race_finished",
)


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def adopt_track(tracker, gates):
    """Adopt live track data when it is plausibly in the current frame."""
    live = tracker.gates
    if len(live) < len(FALLBACK_GATES):
        return gates
    ref = FALLBACK_GATES[0]
    g0 = live[0]
    if (
        abs(g0.x - ref.x) <= TRACK_ACCEPT_TOL_M
        and abs(g0.y - ref.y) <= TRACK_ACCEPT_TOL_M
        and abs(g0.z - ref.z) <= TRACK_ACCEPT_TOL_M
    ):
        return live
    return gates


def main():
    parser = argparse.ArgumentParser(description="Autonomous full-course race run.")
    parser.add_argument("--mav-ip", default="127.0.0.1")
    parser.add_argument("--mav-port", type=int, default=14550)
    parser.add_argument("--speed", type=float, default=V_CRUISE, help="cruise m/s")
    parser.add_argument(
        "--max-seconds", type=float, default=300.0,
        help="abort the run after this long (spec cap is 480)",
    )
    parser.add_argument(
        "--reset", action="store_true",
        help="reset the simulator before flying (respawns at the start gate)",
    )
    parser.add_argument(
        "--log", default="auto",
        help="per-tick CSV path; 'auto' = logs/race/<stamp>.csv, '' disables",
    )
    args = parser.parse_args()

    log_path = args.log
    if log_path == "auto":
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = str(Path(_SRC).parent / "logs" / "race" / f"{stamp}.csv")
    log_file = None
    log_writer = None
    if log_path:
        Path(log_path).parent.mkdir(parents=True, exist_ok=True)
        log_file = open(log_path, "w", newline="", buffering=1)
        log_writer = csv.writer(log_file)
        log_writer.writerow(LOG_COLUMNS)
        print(f"Logging to {log_path}", flush=True)

    client = MAVLinkClient.connect(args.mav_ip, args.mav_port)
    client.start_heartbeat()  # spec: client must maintain >= 2 Hz heartbeat
    telem = Telemetry()
    inner = VelocityToRates()
    tracker = RaceDataTracker()
    gates = list(FALLBACK_GATES)

    if args.reset:
        print("Resetting simulator...", flush=True)
        client.send_sim_reset()
        time.sleep(2.0)

    def drain():
        latest = {}
        for msg in client.recv_all():
            mtype = msg.get_type()
            tracker.update(msg)
            if mtype == "COLLISION":
                print(
                    f"COLLISION id={msg.id} threat={msg.threat_level}", flush=True
                )
            latest[mtype] = msg
        telem.update_from(latest)

    # Wait for a position fix before arming.
    deadline = time.monotonic() + TELEMETRY_TIMEOUT_S
    while time.monotonic() < deadline:
        drain()
        if telem.have_attitude and telem.have_position:
            break
        time.sleep(0.02)
    else:
        raise SystemExit("Timed out waiting for ATTITUDE and LOCAL_POSITION_NED")

    print("Arming...", flush=True)
    client.arm()

    period = 1.0 / CONTROL_HZ
    start_t = time.monotonic()
    last_t = start_t
    next_status = 0.0
    phase = "takeoff"
    takeoff_deadline = start_t + TAKEOFF_TIMEOUT_S
    gate_i = 0
    goaround_until_x = None
    finish_brake_until = None
    prev_xy = None
    finish_s = -1.0

    while True:
        tick = time.monotonic()
        t = tick - start_t
        dt_s = tick - last_t
        last_t = tick
        if t > args.max_seconds:
            print("max-seconds reached, stopping.", flush=True)
            break

        drain()
        gates = adopt_track(tracker, gates)
        status = tracker.status

        # Respawn/teleport detection: restart the flight state machine.
        if telem.have_position:
            if prev_xy is not None:
                jump = math.hypot(telem.x - prev_xy[0], telem.y - prev_xy[1])
                if jump > TELEPORT_M:
                    print(
                        f"TELEPORT detected ({jump:.0f} m): respawn assumed; "
                        "re-arming and restarting from takeoff",
                        flush=True,
                    )
                    client.arm()
                    phase = "takeoff"
                    takeoff_deadline = tick + TAKEOFF_TIMEOUT_S
                    gate_i = 0
                    goaround_until_x = None
            prev_xy = (telem.x, telem.y)

        # Authoritative gate sequencing from race status.
        if tracker.have_status:
            if status.finished and phase not in ("brake", "done"):
                finish_s = status.finish_time_s
                print(f"RACE FINISHED: official time {finish_s:.2f} s", flush=True)
                phase = "brake"
                finish_brake_until = tick + FINISH_BRAKE_S
            elif status.active_gate_index > gate_i and phase == "fly":
                print(
                    f"gate {gate_i} passed at t={t:.1f}s "
                    f"(race idx -> {status.active_gate_index})",
                    flush=True,
                )
                gate_i = status.active_gate_index
                goaround_until_x = None

        if gate_i >= len(gates) and phase == "fly":
            # No finish signal but no gates left (status feed absent).
            phase = "brake"
            finish_brake_until = tick + FINISH_BRAKE_S

        gate = gates[min(gate_i, len(gates) - 1)]
        tx, ty, tz = gate.x, gate.y, gate.center_z
        dist_plane = telem.x - gate.x  # positive while in front of the gate
        cross_err = math.hypot(ty - telem.y, tz - telem.z)
        cmd = VelocityCommand()

        if phase == "takeoff":
            if telem.z <= TAKEOFF_ALT or tick > takeoff_deadline:
                print(f"takeoff done (z={telem.z:+.2f}); flying the course", flush=True)
                phase = "fly"
            else:
                cmd = VelocityCommand(0.0, 0.0, -0.8, 0.0)

        if phase == "fly":
            if goaround_until_x is not None:
                # Missed the opening: back off in front of the gate and retry.
                if telem.x > goaround_until_x:
                    print(f"go-around done, re-approaching gate {gate_i}", flush=True)
                    goaround_until_x = None
                else:
                    vn = clamp(KP_POS * (goaround_until_x + 1.0 - telem.x), 0.0, V_SLOW)
                    ve = clamp(KP_POS * (ty - telem.y), -MAX_V_CROSS, MAX_V_CROSS)
                    vd = clamp(KP_POS * (tz - telem.z), -MAX_V_CROSS, MAX_V_CROSS)
                    cmd = _world_to_cmd(vn, ve, vd, telem)
            if goaround_until_x is None:
                if dist_plane < -PLANE_GRACE_M and tracker.have_status and (
                    status.active_gate_index <= gate_i
                ):
                    print(
                        f"passed gate {gate_i} plane without credit; going around",
                        flush=True,
                    )
                    goaround_until_x = gate.x + GOAROUND_BACKOFF_M
                else:
                    speed = args.speed
                    if dist_plane < SLOW_DIST_M and cross_err > CROSS_TOL_M:
                        speed = min(speed, V_SLOW)
                    vn = -speed  # course runs along -X
                    ve = clamp(KP_POS * (ty - telem.y), -MAX_V_CROSS, MAX_V_CROSS)
                    vd = clamp(KP_POS * (tz - telem.z), -MAX_V_CROSS, MAX_V_CROSS)
                    # Terrain guard: don't sink below the opening center.
                    if telem.z > tz + DEPTH_MARGIN_M:
                        vd = min(vd, 0.0)
                    cmd = _world_to_cmd(vn, ve, vd, telem)

            # Position fallback when the race-status feed is absent.
            if not tracker.have_status and dist_plane < -0.5:
                print(f"gate {gate_i} plane crossed (no race feed)", flush=True)
                gate_i += 1

        if phase == "brake":
            cmd = VelocityCommand(0.0, 0.0, 0.0, 0.0)
            if tick >= finish_brake_until:
                phase = "done"

        if phase == "done":
            break

        cmd.yaw_rate = clamp(
            K_YAW_HOLD * wrap_pi(YAW_REF - telem.yaw), -MAX_YAW_RATE, MAX_YAW_RATE
        )
        rates = inner.update(cmd, telem, dt_s)
        if phase == "takeoff":
            rates.thrust = max(rates.thrust, TAKEOFF_THRUST)
        client.send_attitude_target(
            rates.roll_q, rates.pitch_q, rates.yaw_q, 0.0, rates.thrust
        )

        if log_writer is not None:
            log_writer.writerow([
                f"{t:.3f}", phase, gate_i,
                f"{telem.x:.2f}", f"{telem.y:.2f}", f"{telem.z:.2f}",
                f"{telem.yaw:.3f}",
                f"{tx:.2f}", f"{ty:.2f}", f"{tz:.2f}",
                f"{dist_plane:.2f}", f"{cross_err:.2f}",
                f"{cmd.vx:.2f}", f"{cmd.vy:.2f}", f"{cmd.vz:.2f}",
                f"{cmd.yaw_rate:.2f}", f"{rates.thrust:.3f}",
                status.active_gate_index, int(status.started), int(status.finished),
            ])

        if tick >= next_status:
            print(
                f"[{phase} g{gate_i}] xyz=({telem.x:+7.2f},{telem.y:+6.2f},"
                f"{telem.z:+6.2f}) tgt=({tx:+7.2f},{ty:+6.2f},{tz:+6.2f}) "
                f"plane={dist_plane:+6.1f} cross={cross_err:4.2f} "
                f"race_idx={status.active_gate_index} thr={inner.hover_thrust:.2f}",
                flush=True,
            )
            next_status = tick + 1.0

        time.sleep(max(0.0, period - (time.monotonic() - tick)))

    if log_file is not None:
        log_file.close()
    if finish_s > 0:
        print(f"Done. Official race time: {finish_s:.2f} s", flush=True)
    else:
        print("Done (no finish signal received).", flush=True)


def _world_to_cmd(vn, ve, vd, telem) -> VelocityCommand:
    """World-NED desired velocity -> body-frame VelocityCommand (the
    calibrated cascade takes body vx/vy and world vertical vd)."""
    cy, sy = math.cos(telem.yaw), math.sin(telem.yaw)
    return VelocityCommand(
        vx=cy * vn + sy * ve,
        vy=-sy * vn + cy * ve,
        vz=vd,
        yaw_rate=0.0,
    )


if __name__ == "__main__":
    main()
