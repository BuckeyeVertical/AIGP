"""Velocity-frame probe: LOCAL_POSITION_NED vs ODOMETRY.

The control run showed velocity feedback flip-flopping sign sample-to-sample
while actual position moved steadily -- suspicion: LOCAL_POSITION_NED and
ODOMETRY report velocity in different frames/signs and the controller merges
them blindly. This probe applies a gentle constant forward tilt and prints
each message stream separately, so dx/dt can be compared against both.
"""

from __future__ import annotations

import argparse
import time

from mavlink_client import MAVLinkClient


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--ip", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=14550)
    args = parser.parse_args()

    client = MAVLinkClient.connect(args.ip, args.port)
    client.start_heartbeat()
    print("Arming...", flush=True)
    client.arm()

    # Climb briefly, then drift forward with a small nose-down rate command.
    print("\nphase: climb 2 s, then gentle forward drift 6 s")
    t0 = time.monotonic()
    next_print = {"LOCAL_POSITION_NED": 0.0, "ODOMETRY": 0.0}
    while True:
        t = time.monotonic() - t0
        if t > 8.0:
            break
        if t < 2.0:
            client.send_attitude_target(0.0, 0.0, 0.0, 0.0, 0.30)
        else:
            # encoded pitch -0.04 -> pitch rate +0.1 (nose down a touch)
            client.send_attitude_target(0.0, -0.04, 0.0, 0.0, 0.25)

        msg = client.sim_conn.recv_match(
            type=["LOCAL_POSITION_NED", "ODOMETRY"], blocking=False
        )
        now = time.monotonic()
        if msg is not None:
            mtype = msg.get_type()
            if now >= next_print[mtype]:
                print(
                    f"  t={t:4.1f} {mtype:<18} "
                    f"pos=({msg.x:+7.2f},{msg.y:+7.2f},{msg.z:+7.2f}) "
                    f"vel=({msg.vx:+6.2f},{msg.vy:+6.2f},{msg.vz:+6.2f})",
                    flush=True,
                )
                next_print[mtype] = now + 0.4
        time.sleep(0.01)

    print("\nDone.")


if __name__ == "__main__":
    main()
