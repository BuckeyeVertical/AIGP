"""Entry point for the gate-centering visual-servo control loop.

Run with:

    python src\\run\\center.py --max-seconds 60 --log flight.csv

All flight logic lives in ``src/control/control.py``; this module only wires up
the camera/perception/MAVLink pipeline and runs the control loop. Importing
``control`` sets up the sibling-module sys.path (camera, gate_perception,
vision_receiver, mavlink_client), so those imports work below.
"""

from __future__ import annotations

import argparse
import dataclasses
import os
import sys
import time

# This file lives in src/run; control.py lives in src/control and configures
# sys.path for the sibling src modules at import time, so make it importable
# first (go up to src/, then into control/).
_SRC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_SRC, "control"))

from control import (  # noqa: E402
    ALT_FLOOR_Z,
    BLIND_MAX_EX,
    BLIND_MIN_SIZE_PX,
    BLIND_RECENT_S,
    COMMIT_MAX_EX,
    COMMIT_SIZE_PX,
    DASH_S,
    FLOOR_CLIMB_VZ,
    MIN_CONFIDENCE,
    TAKEOFF_ALT,
    TAKEOFF_THRUST,
    TAKEOFF_TIMEOUT_S,
    V_DASH,
    Telemetry,
    VelocityCommand,
    VelocityToRates,
    VisualServoController,
    gate_error_angles,
)
from control import _NO_GATE  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description="Visual-servo control loop.")
    parser.add_argument("--mav-ip", default="127.0.0.1")
    parser.add_argument("--mav-port", type=int, default=14550)
    parser.add_argument("--cam-ip", default="0.0.0.0")
    parser.add_argument("--cam-port", type=int, default=5600)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="compute and print commands without connecting/sending MAVLink",
    )
    parser.add_argument(
        "--max-seconds",
        type=float,
        default=0.0,
        help="stop the loop after this many seconds (0 = run forever)",
    )
    parser.add_argument(
        "--log",
        default="",
        help="write a per-tick CSV debug log to this path",
    )
    args = parser.parse_args()

    from gate_perception import GateDetector
    from vision_receiver import frames

    detector = GateDetector()
    # search_yaw_rate=0: when the gate is lost (e.g. after passing through it)
    # just hold heading instead of yaw-searching, so the drone doesn't veer
    # after the pass. Re-enable a search once multi-gate acquisition is wanted.
    servo = VisualServoController(search_yaw_rate=0.0)
    inner = VelocityToRates()
    telem = Telemetry()

    client = None
    if not args.dry_run:
        from mavlink_client import MAVLinkClient

        client = MAVLinkClient.connect(args.mav_ip, args.mav_port)
        client.start_heartbeat()  # spec: client must maintain >=2 Hz heartbeat
        print("Arming...", flush=True)
        client.arm()

    log = None
    if args.log:
        log = open(args.log, "w")
        log.write(
            "t,phase,found,cx,cy,wpx,hpx,conf,cmd_vx,cmd_vy,cmd_vz,cmd_yawrate,"
            "vbx,vby,vd,roll_des,pitch_des,roll,pitch,yaw,"
            "roll_q,pitch_q,yaw_q,thrust,x,y,z\n"
        )

    print(f"Control loop running on camera udp://{args.cam_ip}:{args.cam_port}")
    start_t = time.monotonic()
    last_t = start_t
    next_status = 0.0
    det = None
    phase = "takeoff"
    dash_until = 0.0
    # (t, max bbox px, ex, center_x, center_y) of the last fully-in-frame,
    # centered detection. Latched so that once the gate starts leaving the
    # frame (det.partial) we steer/dash on this clean center instead of the
    # clipped bbox center, which slides sideways into the posts.
    last_good = None
    for item in frames(args.cam_ip, args.cam_port, timeout_s=2.0):
        now = time.monotonic()
        if args.max_seconds > 0 and now - start_t > args.max_seconds:
            print("max-seconds reached, stopping.")
            break
        dt = now - last_t
        last_t = now

        if client is not None:
            telem.update_from(
                client.recv_telemetry(["ATTITUDE", "LOCAL_POSITION_NED", "ODOMETRY"])
            )

        if item is not None:
            _frame_id, image, _sim_time_ns = item
            det = detector.detect(image)

        if client is None:
            cmd = servo.compute(det if det is not None else _NO_GATE)
            print(
                f"vx={cmd.vx:+.2f} vy={cmd.vy:+.2f} vz={cmd.vz:+.2f} "
                f"yaw_rate={cmd.yaw_rate:+.2f}"
            )
            continue

        cmd = None
        rc = None

        # --- takeoff: punch off the ground, level, until at altitude ---------
        if phase == "takeoff":
            if telem.z <= TAKEOFF_ALT or now - start_t > TAKEOFF_TIMEOUT_S:
                print(f"takeoff done (z={telem.z:+.2f}), switching to servo")
                phase = "servo"
            else:
                # level-hold via the attitude loop, fixed climb thrust
                cmd = VelocityCommand(0.0, 0.0, -0.8, 0.0)
                rc = inner.update(cmd, telem, dt)
                rc.thrust = max(rc.thrust, TAKEOFF_THRUST)
                client.send_attitude_target(
                    rc.roll_q, rc.pitch_q, rc.yaw_q, 0.0, rc.thrust
                )

        # --- gate-pass dash: fly straight through, ignore the detection -------
        if phase == "dash":
            if now >= dash_until:
                phase = "servo"
            else:
                cmd = VelocityCommand(V_DASH, 0.0, 0.0, 0.0)

        # --- visual servo -----------------------------------------------------
        if phase == "servo":
            good = (
                det is not None
                and det.found
                and det.confidence >= MIN_CONFIDENCE
            )
            # The veer is a HORIZONTAL slide: as the ring fills the view its bbox
            # gets clipped on the left/right border and the center jumps toward
            # the still-visible side. Only clipped_x makes the horizontal center
            # untrustworthy. clipped_y fires harmlessly early because the gate is
            # framed low (20 deg camera up-tilt), so it must NOT gate steering.
            ex = 0.0
            size = 0
            if good:
                ex, _ = gate_error_angles(
                    det.center_x, det.center_y, telem.pitch
                )
                size = max(det.width_px, det.height_px)
            h_trust = good and not det.clipped_x  # horizontal center reliable
            centered = abs(ex) <= COMMIT_MAX_EX
            if h_trust and size >= BLIND_MIN_SIZE_PX and abs(ex) <= BLIND_MAX_EX:
                last_good = (now, size, ex, det.center_x, det.center_y)

            have_lock = (
                last_good is not None and now - last_good[0] <= BLIND_RECENT_S
            )
            centered_lock = have_lock and abs(last_good[2]) <= COMMIT_MAX_EX
            blind_pass = not good and have_lock

            if h_trust and size >= COMMIT_SIZE_PX and centered:
                print(
                    f"gate close (w={det.width_px} h={det.height_px}px, "
                    f"ex={ex:+.2f}), committing to dash"
                )
                phase = "dash"
                dash_until = now + DASH_S
                cmd = VelocityCommand(V_DASH, 0.0, 0.0, 0.0)
            elif good and det.clipped_x and centered_lock:
                # Horizontal edge is clipping the bbox, so its center is sliding.
                # We had a recent centered lock, so finish straight instead of
                # chasing the slide into a post.
                print(
                    f"gate leaving frame (w={det.width_px} h={det.height_px}px, "
                    f"last centered ex={last_good[2]:+.2f}), committing to dash"
                )
                phase = "dash"
                dash_until = now + DASH_S
                cmd = VelocityCommand(V_DASH, 0.0, 0.0, 0.0)
            elif blind_pass:
                print(
                    f"lost gate at point-blank (last size {last_good[1]}px, "
                    f"ex {last_good[2]:+.2f}), blind dash through"
                )
                phase = "dash"
                dash_until = now + DASH_S
                last_good = None
                cmd = VelocityCommand(V_DASH, 0.0, 0.0, 0.0)
            else:
                # If the horizontal center is being clipped but we have a fresh
                # lock, servo on the latched center instead of the sliding one;
                # otherwise servo on the current detection.
                servo_det = det if det is not None else _NO_GATE
                if good and det.clipped_x and have_lock:
                    servo_det = dataclasses.replace(
                        det, center_x=last_good[3], center_y=last_good[4]
                    )
                cmd = servo.compute(servo_det, reported_pitch=telem.pitch)

        if phase in ("servo", "dash") and cmd is not None:
            if telem.have_position and telem.z > ALT_FLOOR_Z:
                cmd.vz = min(cmd.vz, FLOOR_CLIMB_VZ)
            rc = inner.update(cmd, telem, dt)
            client.send_attitude_target(
                rc.roll_q, rc.pitch_q, rc.yaw_q, 0.0, rc.thrust
            )

        if log is not None and rc is not None:
            found = det is not None and det.found
            log.write(
                f"{now - start_t:.3f},{phase},{int(found)},"
                f"{det.center_x if found else 0:.0f},"
                f"{det.center_y if found else 0:.0f},"
                f"{det.width_px if found else 0},"
                f"{det.height_px if found else 0},"
                f"{det.confidence if found else 0:.2f},"
                f"{cmd.vx:.2f},{cmd.vy:.2f},{cmd.vz:.2f},{cmd.yaw_rate:.2f},"
                f"{telem.vbx:.2f},{telem.vby:.2f},{telem.vd:.2f},"
                f"{rc.roll_des:.3f},{rc.pitch_des:.3f},"
                f"{telem.roll:.3f},{telem.pitch:.3f},{telem.yaw:.3f},"
                f"{rc.roll_q:.3f},{rc.pitch_q:.3f},{rc.yaw_q:.3f},{rc.thrust:.3f},"
                f"{telem.x:.2f},{telem.y:.2f},{telem.z:.2f}\n"
            )

        if now >= next_status:
            gate = (
                f"gate@({det.center_x:.0f},{det.center_y:.0f}) "
                f"{det.width_px}x{det.height_px} conf={det.confidence:.2f}"
                if det is not None and det.found
                else "no gate"
            )
            print(
                f"[{phase}] {gate} | xyz=({telem.x:+6.2f},{telem.y:+6.2f},{telem.z:+6.2f}) "
                f"rpy=({telem.roll:+.2f},{telem.pitch:+.2f},{telem.yaw:+.2f}) "
                f"v=({telem.vn:+.2f},{telem.ve:+.2f},{telem.vd:+.2f}) "
                f"thr={inner.hover_thrust:.2f}",
                flush=True,
            )
            next_status = now + 1.0

    if log is not None:
        log.close()


if __name__ == "__main__":
    main()
