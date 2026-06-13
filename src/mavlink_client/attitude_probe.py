"""Attitude-convention probe (stays on the ground).

The hover probe showed the sim does NOT track our attitude commands: a constant
level command produced a spinning ~0.31 rad tilt. This probe runs at thrust
0.12 -- below the ~0.20 liftoff point -- and tries different quaternion
conventions, watching which one the reported ATTITUDE actually follows:

  1. level, [w,x,y,z] at measured yaw   (our current convention)
  2. pitch +0.3, [w,x,y,z]
  3. pitch +0.3, [x,y,z,w]              (reordered)
  4. level + yaw_rate 0.5               (does the body yaw-rate field work?)
  5. level, yaw = measured + 1.0 rad    (does quaternion yaw steer?)
"""

from __future__ import annotations

import argparse
import math
import time

from pymavlink import mavutil

from mavlink_client import MAVLinkClient

THRUST = 0.12  # below liftoff (~0.20): attitude loop active, wheels down
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

    yaw0 = 0.0
    latest = client.recv_telemetry(TELEM_TYPES)
    if "ATTITUDE" in latest:
        yaw0 = latest["ATTITUDE"].yaw
    print(f"initial yaw={yaw0:+.2f}")

    mask = (
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
        | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
    )

    q_level = quat_wxyz(0.0, 0.0, yaw0)
    q_pitch = quat_wxyz(0.0, 0.3, yaw0)
    q_pitch_xyzw = q_pitch[1:] + q_pitch[:1]  # reorder to [x,y,z,w]
    q_yawplus = quat_wxyz(0.0, 0.0, yaw0 + 1.0)

    phases = [
        ("level [w,x,y,z]", q_level, 0.0),
        ("pitch +0.3 [w,x,y,z]", q_pitch, 0.0),
        ("pitch +0.3 [x,y,z,w]", q_pitch_xyzw, 0.0),
        ("level + yaw_rate 0.5", q_level, 0.5),
        ("level, yaw+1.0 rad", q_yawplus, 0.0),
    ]

    for label, q, yaw_rate in phases:
        print(f"\n--- {label}  q={[round(v, 3) for v in q]} ---")
        t_end = time.monotonic() + 3.0
        next_print = 0.0
        while time.monotonic() < t_end:
            client.sim_conn.mav.set_attitude_target_send(
                client._now_ms(),
                client.sim_conn.target_system,
                client.sim_conn.target_component,
                mask,
                q,
                0.0, 0.0,
                float(yaw_rate),
                THRUST,
            )
            latest = client.recv_telemetry(TELEM_TYPES)
            now = time.monotonic()
            if latest and now >= next_print:
                a = latest.get("ATTITUDE")
                pos = latest.get("LOCAL_POSITION_NED")
                line = []
                if a is not None:
                    line.append(f"rpy=({a.roll:+5.2f},{a.pitch:+5.2f},{a.yaw:+5.2f})")
                if pos is not None:
                    line.append(f"z={pos.z:+6.2f}")
                print("   " + "  ".join(line), flush=True)
                next_print = now + 0.5
            time.sleep(0.02)

    print("\nDone.")


if __name__ == "__main__":
    main()
