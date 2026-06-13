"""Yaw-mirror probe: is the sim's yaw error sign inverted?

Evidence so far fits an inverted yaw gain in attitude-target mode:
  - target == measured yaw  -> bang-bang oscillation (positive feedback)
  - fixed target 1.57 from yaw pi -> constant spin AWAY from the target
  - roll/pitch track correctly

If the sim applies torque ~ K*(measured - target) instead of K*(target -
measured), then sending target = 2*measured - desired makes its correction
push toward `desired`. Phases:

  1. hold spawn yaw via mirrored target      -> should be STABLE (no spin)
  2. mirrored target toward spawn_yaw - 1.0  -> should converge to it
  3. mirrored yaw hold + pitch +0.2          -> check pitch tracking & sign

Thrust 0.15 constant (will climb slowly; run is short).
"""

from __future__ import annotations

import argparse
import math
import time

from pymavlink import mavutil

from mavlink_client import MAVLinkClient

THRUST = 0.15
TELEM_TYPES = ["ATTITUDE", "LOCAL_POSITION_NED"]


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_wxyz(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--ip", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=14550)
    args = parser.parse_args()

    client = MAVLinkClient.connect(args.ip, args.port)
    client.start_heartbeat()
    print("Arming...", flush=True)
    client.arm()
    time.sleep(0.3)

    mask = (
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
        | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
    )

    latest = client.recv_telemetry(TELEM_TYPES)
    yaw0 = latest["ATTITUDE"].yaw if "ATTITUDE" in latest else 0.0
    print(f"spawn yaw={yaw0:+.2f}")

    state = {"yaw": yaw0}

    phases = [
        ("hold spawn yaw (mirrored)", 0.0, yaw0, 4.0),
        ("turn to spawn-1.0 (mirrored)", 0.0, wrap_pi(yaw0 - 1.0), 4.0),
        ("pitch +0.2, hold yaw (mirrored)", 0.2, yaw0, 4.0),
    ]

    for label, pitch_t, yaw_desired, duration in phases:
        print(f"\n--- {label}  desired_yaw={yaw_desired:+.2f} ---")
        t_end = time.monotonic() + duration
        next_print = 0.0
        while time.monotonic() < t_end:
            latest = client.recv_telemetry(TELEM_TYPES)
            a = latest.get("ATTITUDE")
            if a is not None:
                state["yaw"] = a.yaw
            yaw_target = wrap_pi(2.0 * state["yaw"] - yaw_desired)
            q = quat_wxyz(0.0, pitch_t, yaw_target)
            client.sim_conn.mav.set_attitude_target_send(
                client._now_ms(),
                client.sim_conn.target_system,
                client.sim_conn.target_component,
                mask,
                q,
                0.0, 0.0, 0.0,
                THRUST,
            )
            now = time.monotonic()
            if now >= next_print:
                line = []
                if a is not None:
                    line.append(
                        f"rpy=({a.roll:+5.2f},{a.pitch:+5.2f},{a.yaw:+5.2f}) "
                        f"yawspeed={a.yawspeed:+5.2f}"
                    )
                pos = latest.get("LOCAL_POSITION_NED")
                if pos is not None:
                    line.append(f"z={pos.z:+7.2f} vz={pos.vz:+5.2f}")
                print("   " + "  ".join(line), flush=True)
                next_print = now + 0.5
            time.sleep(0.02)

    print("\nDone.")


if __name__ == "__main__":
    main()
