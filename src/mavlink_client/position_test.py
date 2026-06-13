"""Position-setpoint diagnostic.

The open-loop velocity test showed the sim holds position (0,0,0) and only
tilts attitude when velocity setpoints are streamed -- i.e. it appears to read
the POSITION fields of SET_POSITION_TARGET_LOCAL_NED regardless of typemask.
This test streams actual position targets to confirm: climb to 2 m, fly to
(2,0,-2), return. If the drone tracks these, control must command positions
(or position+velocity), not pure velocity.
"""

from __future__ import annotations

import argparse
import time

from pymavlink import mavutil

from mavlink_client import MAVLinkClient

# (label, x, y, z, seconds) in MAV_FRAME_LOCAL_NED; z=-2 is 2 m up.
PHASES = [
    ("climb to z=-2", 0.0, 0.0, -2.0, 5.0),
    ("forward to x=+2", 2.0, 0.0, -2.0, 5.0),
    ("return to origin, stay up", 0.0, 0.0, -2.0, 5.0),
]

# Position-only: ignore velocity, acceleration, yaw, yaw rate.
_POSITION_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

TELEM_TYPES = ["LOCAL_POSITION_NED", "ODOMETRY", "ATTITUDE"]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--ip", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=14550)
    parser.add_argument("--rate", type=float, default=50.0)
    args = parser.parse_args()

    client = MAVLinkClient.connect(args.ip, args.port)
    client.start_heartbeat()
    print("Arming...", flush=True)
    client.arm()

    period = 1.0 / args.rate
    for label, x, y, z, duration in PHASES:
        print(f"\n--- {label}  target=({x:+.1f},{y:+.1f},{z:+.1f}) for {duration:.0f}s ---")
        t_end = time.monotonic() + duration
        next_print = 0.0
        while time.monotonic() < t_end:
            client.sim_conn.mav.set_position_target_local_ned_send(
                client._now_ms(),
                client.sim_conn.target_system,
                client.sim_conn.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                _POSITION_MASK,
                float(x), float(y), float(z),
                0.0, 0.0, 0.0,  # velocity (ignored)
                0.0, 0.0, 0.0,  # acceleration (ignored)
                0.0, 0.0,       # yaw, yaw_rate (ignored)
            )
            latest = client.recv_telemetry(TELEM_TYPES)
            now = time.monotonic()
            if latest and now >= next_print:
                pos = latest.get("LOCAL_POSITION_NED") or latest.get("ODOMETRY")
                att = latest.get("ATTITUDE")
                line = []
                if pos is not None:
                    line.append(
                        f"pos=({pos.x:+6.2f},{pos.y:+6.2f},{pos.z:+6.2f}) "
                        f"vel=({pos.vx:+5.2f},{pos.vy:+5.2f},{pos.vz:+5.2f})"
                    )
                if att is not None:
                    line.append(f"rpy=({att.roll:+5.2f},{att.pitch:+5.2f},{att.yaw:+5.2f})")
                print("   " + "  ".join(line), flush=True)
                next_print = now + 0.5
            time.sleep(period)

    print("\nDone.")


if __name__ == "__main__":
    main()
