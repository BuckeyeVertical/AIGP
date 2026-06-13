"""Verify the yaw-as-rate model of SET_ATTITUDE_TARGET.

Hypothesis from all previous probes: the sim reads the quaternion's roll and
pitch as ANGLE targets (they track), but reads the yaw component as a yaw
RATE command, scaled by ~2.57 rad/s per rad (yaw 1.57 spun at a constant
4.03 rad/s; yaw 0.0 froze yaw entirely).

Predictions tested here (level attitude, thrust 0.15):
  1. q yaw +0.5 -> steady yawspeed ~ +1.28
  2. q yaw -0.5 -> steady yawspeed ~ -1.28
  3. q yaw 0, pitch +0.2 -> yaw frozen, measured pitch settles (sign check)
  4. q yaw 0, roll  +0.2 -> yaw frozen, measured roll settles (sign check)
"""

from __future__ import annotations

import argparse
import math
import time

from pymavlink import mavutil

from mavlink_client import MAVLinkClient

THRUST = 0.15
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

    phases = [
        ("yaw +0.5 (expect yawspeed ~ +1.28)", 0.0, 0.0, 0.5, 3.0),
        ("yaw -0.5 (expect yawspeed ~ -1.28)", 0.0, 0.0, -0.5, 3.0),
        ("pitch +0.2, yaw 0 (sign check)", 0.0, 0.2, 0.0, 3.0),
        ("roll +0.2, yaw 0 (sign check)", 0.2, 0.0, 0.0, 3.0),
    ]

    for label, roll_t, pitch_t, yaw_t, duration in phases:
        print(f"\n--- {label} ---")
        q = quat_wxyz(roll_t, pitch_t, yaw_t)
        t_end = time.monotonic() + duration
        next_print = 0.0
        while time.monotonic() < t_end:
            client.sim_conn.mav.set_attitude_target_send(
                client._now_ms(),
                client.sim_conn.target_system,
                client.sim_conn.target_component,
                mask,
                q,
                0.0, 0.0, 0.0,
                THRUST,
            )
            latest = client.recv_telemetry(TELEM_TYPES)
            a = latest.get("ATTITUDE")
            now = time.monotonic()
            if latest and now >= next_print:
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
