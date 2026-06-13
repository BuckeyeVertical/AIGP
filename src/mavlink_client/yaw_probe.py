"""Yaw-channel probe.

Probe 2 showed roll/pitch quaternion tracking works ([w,x,y,z]) but yaw goes
bang-bang (+/-9 rad/s) around yaw ~ +/-pi -- the quaternion double-cover
boundary, and exactly where the drone spawns. This probe checks whether yaw
behaves away from the boundary:

  1. fixed yaw target +1.57 (90 deg away from the bad zone): does it settle?
  2. fixed yaw target 0.0: stable convergence?
  3. incremental target (measured + 0.3, recomputed live): smooth slow turn?

Level attitude, constant thrust 0.13 throughout.
"""

from __future__ import annotations

import argparse
import math
import time

from pymavlink import mavutil

from mavlink_client import MAVLinkClient

THRUST = 0.13
TELEM_TYPES = ["ATTITUDE", "LOCAL_POSITION_NED"]


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

    yaw_meas = [0.0]

    phases = [
        ("fixed yaw +1.57", lambda: 1.57, 5.0),
        ("fixed yaw 0.0", lambda: 0.0, 4.0),
        ("incremental yaw (meas + 0.3)", lambda: yaw_meas[0] + 0.3, 4.0),
    ]

    for label, target_fn, duration in phases:
        print(f"\n--- {label} ---")
        t_end = time.monotonic() + duration
        next_print = 0.0
        while time.monotonic() < t_end:
            latest = client.recv_telemetry(TELEM_TYPES)
            a = latest.get("ATTITUDE")
            if a is not None:
                yaw_meas[0] = a.yaw
            q = quat_wxyz(0.0, 0.0, target_fn())
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
