"""Hybrid control probe: velocity setpoints for attitude + thrust-only
SET_ATTITUDE_TARGET for lift.

Findings so far:
  - SET_POSITION_TARGET velocity setpoints: sim tracks attitude cleanly
    (tilts toward the commanded velocity) but generates ZERO thrust.
  - SET_ATTITUDE_TARGET: thrust field works, but the sim misinterprets the
    quaternion and spins at ~3 rad/s regardless of what we command.

This probe streams BOTH: a body-frame velocity setpoint (attitude guidance)
plus an attitude target with ATTITUDE_IGNORE | rate-ignore bits so only its
thrust field is active. If the sim composes them, we have a flyable stack.
"""

from __future__ import annotations

import argparse
import time

from pymavlink import mavutil

from mavlink_client import MAVLinkClient

# (label, vx, vy, vz, yaw_rate, thrust, seconds)
PHASES = [
    ("hover-ish thrust, zero vel", 0.0, 0.0, 0.0, 0.0, 0.18, 4.0),
    ("forward 1 m/s", 1.0, 0.0, 0.0, 0.0, 0.18, 4.0),
    ("yaw rate 0.5", 0.0, 0.0, 0.0, 0.5, 0.18, 4.0),
    ("reduce thrust (descend?)", 0.0, 0.0, 0.0, 0.0, 0.10, 4.0),
]

# Thrust-only attitude target: ignore attitude and all body rates.
_THRUST_ONLY_MASK = (
    mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE
    | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
    | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
    | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
)

TELEM_TYPES = ["ATTITUDE", "LOCAL_POSITION_NED", "ODOMETRY"]


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

    for label, vx, vy, vz, yaw_rate, thrust, duration in PHASES:
        print(f"\n--- {label}  vel=({vx:+.1f},{vy:+.1f},{vz:+.1f}) "
              f"yaw_rate={yaw_rate:+.1f} thrust={thrust:.2f} ---")
        t_end = time.monotonic() + duration
        next_print = 0.0
        while time.monotonic() < t_end:
            client.send_body_velocity(vx, vy, vz, yaw_rate)
            client.sim_conn.mav.set_attitude_target_send(
                client._now_ms(),
                client.sim_conn.target_system,
                client.sim_conn.target_component,
                _THRUST_ONLY_MASK,
                [1.0, 0.0, 0.0, 0.0],  # attitude (ignored)
                0.0, 0.0, 0.0,         # body rates (ignored)
                float(thrust),
            )
            latest = client.recv_telemetry(TELEM_TYPES)
            now = time.monotonic()
            if latest and now >= next_print:
                a = latest.get("ATTITUDE")
                pos = latest.get("LOCAL_POSITION_NED") or latest.get("ODOMETRY")
                line = []
                if pos is not None:
                    line.append(
                        f"pos=({pos.x:+6.2f},{pos.y:+6.2f},{pos.z:+6.2f}) "
                        f"vel=({pos.vx:+5.2f},{pos.vy:+5.2f},{pos.vz:+5.2f})"
                    )
                if a is not None:
                    line.append(f"rpy=({a.roll:+5.2f},{a.pitch:+5.2f},{a.yaw:+5.2f})")
                print("   " + "  ".join(line), flush=True)
                next_print = now + 0.5
            time.sleep(0.02)

    print("\nDone.")


if __name__ == "__main__":
    main()
