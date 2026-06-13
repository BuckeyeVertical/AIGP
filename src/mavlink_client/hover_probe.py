"""Thrust system-identification probe.

Holds a level attitude and steps the SET_ATTITUDE_TARGET thrust field through
fixed values while logging z / vz, to establish:
  - what thrust value actually hovers
  - whether thrust 0.0 means motors-off (falls) or hold
  - the sign/shape of the response in flight (the visual-servo run suggested
    thrust may behave relative to hover, not absolutely)

Steps stay short and start from the ground so the drone stays low.
"""

from __future__ import annotations

import argparse
import time

from pymavlink import mavutil

from mavlink_client import MAVLinkClient

# (thrust, seconds)
STEPS = [
    (0.00, 2.0),   # on ground, motors-off baseline
    (0.10, 3.0),   # below presumed hover: should stay on ground
    (0.20, 3.0),
    (0.25, 3.0),   # presumed hover region
    (0.28, 3.0),
    (0.00, 4.0),   # airborne by now? thrust 0: fall or hold?
    (0.25, 4.0),   # catch it again if it fell
]

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
    time.sleep(0.3)

    # Hold whatever yaw the drone starts with.
    yaw = 0.0
    latest = client.recv_telemetry(TELEM_TYPES)
    att = latest.get("ATTITUDE")
    if att is not None:
        yaw = att.yaw
    print(f"holding level attitude at yaw={yaw:+.2f}")

    period = 1.0 / args.rate
    for thrust, duration in STEPS:
        print(f"\n--- thrust={thrust:.2f} for {duration:.0f}s ---")
        t_end = time.monotonic() + duration
        next_print = 0.0
        while time.monotonic() < t_end:
            client.send_attitude_target(0.0, 0.0, yaw, 0.0, thrust)
            latest = client.recv_telemetry(TELEM_TYPES)
            now = time.monotonic()
            if latest and now >= next_print:
                pos = latest.get("LOCAL_POSITION_NED") or latest.get("ODOMETRY")
                a = latest.get("ATTITUDE")
                line = []
                if pos is not None:
                    line.append(f"z={pos.z:+7.2f} vz={pos.vz:+6.2f}")
                if a is not None:
                    line.append(f"rp=({a.roll:+5.2f},{a.pitch:+5.2f})")
                print("   " + "  ".join(line), flush=True)
                next_print = now + 0.5
            time.sleep(period)

    print("\nDone.")


if __name__ == "__main__":
    main()
