"""Open-loop MAVLink command diagnostic.

Answers one question: does the simulator actually fly our velocity setpoints?
No camera, no perception -- it arms, then commands a fixed sequence of motions
(climb, forward, yaw, strafe) while printing position/attitude telemetry back
from the sim. If the printed position doesn't follow the commanded phase, the
problem is the MAVLink command path (frame, typemask, arming), not the
visual-servo controller.

Usage (simulator running):
    python command_test.py                 # body-frame velocity phases
    python command_test.py --frame local   # same phases in MAV_FRAME_LOCAL_NED
    python command_test.py --rate 50       # command rate (spec: <100 Hz)
"""

from __future__ import annotations

import argparse
import time

from pymavlink import mavutil

from mavlink_client import MAVLinkClient

# (label, vx, vy, vz, yaw_rate, seconds). NED: vz=-0.8 climbs.
PHASES = [
    ("hold     (expect: no motion)", 0.0, 0.0, 0.0, 0.0, 2.0),
    ("climb    (expect: z decreases)", 0.0, 0.0, -0.8, 0.0, 3.0),
    ("hold     (expect: hover)", 0.0, 0.0, 0.0, 0.0, 2.0),
    ("forward  (expect: x increases)", 1.0, 0.0, 0.0, 0.0, 3.0),
    ("yaw CW   (expect: heading changes, position holds)", 0.0, 0.0, 0.0, 0.5, 3.0),
    ("strafe R (expect: sideways motion)", 0.0, 0.8, 0.0, 0.0, 3.0),
    ("stop     (expect: hover)", 0.0, 0.0, 0.0, 0.0, 2.0),
]

TELEM_TYPES = ["LOCAL_POSITION_NED", "ODOMETRY", "ATTITUDE", "COMMAND_ACK"]


def _fmt_telemetry(latest):
    parts = []
    pos = latest.get("LOCAL_POSITION_NED")
    odom = latest.get("ODOMETRY")
    att = latest.get("ATTITUDE")
    if pos is not None:
        parts.append(
            f"pos=({pos.x:+6.2f},{pos.y:+6.2f},{pos.z:+6.2f}) "
            f"vel=({pos.vx:+5.2f},{pos.vy:+5.2f},{pos.vz:+5.2f})"
        )
    elif odom is not None:
        parts.append(
            f"odom pos=({odom.x:+6.2f},{odom.y:+6.2f},{odom.z:+6.2f}) "
            f"vel=({odom.vx:+5.2f},{odom.vy:+5.2f},{odom.vz:+5.2f})"
        )
    if att is not None:
        parts.append(
            f"rpy=({att.roll:+5.2f},{att.pitch:+5.2f},{att.yaw:+5.2f})"
        )
    return "  ".join(parts) if parts else "(no position/attitude telemetry seen)"


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--ip", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=14550)
    parser.add_argument("--frame", choices=["body", "local"], default="body")
    parser.add_argument("--rate", type=float, default=50.0, help="command Hz (<100)")
    args = parser.parse_args()

    frame = (
        mavutil.mavlink.MAV_FRAME_BODY_NED
        if args.frame == "body"
        else mavutil.mavlink.MAV_FRAME_LOCAL_NED
    )

    client = MAVLinkClient.connect(args.ip, args.port)
    client.start_heartbeat()

    print("Arming...", flush=True)
    client.arm()
    ack = client.sim_conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    if ack is None:
        print("!! no COMMAND_ACK for arm within 3 s (sim may not ack, or arm ignored)")
    else:
        result = mavutil.mavlink.enums["MAV_RESULT"][ack.result].name
        print(f"arm ack: command={ack.command} result={result}")

    period = 1.0 / args.rate
    print(f"\nframe={args.frame}  command rate={args.rate:.0f} Hz")
    for label, vx, vy, vz, yaw_rate, duration in PHASES:
        print(f"\n--- {label}  cmd vx={vx:+.1f} vy={vy:+.1f} vz={vz:+.1f} "
              f"yaw_rate={yaw_rate:+.1f} for {duration:.0f}s ---")
        t_end = time.monotonic() + duration
        next_print = 0.0
        while time.monotonic() < t_end:
            client.send_body_velocity(vx, vy, vz, yaw_rate, frame=frame)
            latest = client.recv_telemetry(TELEM_TYPES)
            now = time.monotonic()
            if latest and now >= next_print:
                print("   " + _fmt_telemetry(latest), flush=True)
                next_print = now + 0.5
            time.sleep(period)

    print("\nDone. If position never changed: the sim is not honoring these "
          "setpoints (try --frame local, or the arm was rejected).")


if __name__ == "__main__":
    main()
