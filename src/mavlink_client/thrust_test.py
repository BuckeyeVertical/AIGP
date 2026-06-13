"""Thrust-path diagnostic.

Velocity and position setpoints both tilt the drone but never lift it: the sim
generates no thrust from SET_POSITION_TARGET_LOCAL_NED. This test probes the
throttle channel two ways:

  phase 1: MAV_CMD_NAV_TAKEOFF (alt 2 m), watch z for 4 s
  phase 2: SET_ATTITUDE_TARGET, level attitude, thrust ramp 0.3 -> 0.9,
           watching for liftoff (z decreasing)

Whichever moves z tells us how control must command altitude.
"""

from __future__ import annotations

import argparse
import time

from pymavlink import mavutil

from mavlink_client import MAVLinkClient

TELEM_TYPES = ["LOCAL_POSITION_NED", "ODOMETRY", "ATTITUDE"]

# Ignore body roll/pitch/yaw rates; attitude (quaternion) + thrust active.
_ATT_MASK = (
    mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
    | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
    | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
)

LEVEL_Q = [1.0, 0.0, 0.0, 0.0]  # w, x, y, z


def _watch(client, seconds, label):
    t_end = time.monotonic() + seconds
    next_print = 0.0
    while time.monotonic() < t_end:
        latest = client.recv_telemetry(TELEM_TYPES)
        now = time.monotonic()
        if latest and now >= next_print:
            pos = latest.get("LOCAL_POSITION_NED") or latest.get("ODOMETRY")
            if pos is not None:
                print(
                    f"   [{label}] pos=({pos.x:+6.2f},{pos.y:+6.2f},{pos.z:+6.2f}) "
                    f"vel=({pos.vx:+5.2f},{pos.vy:+5.2f},{pos.vz:+5.2f})",
                    flush=True,
                )
            next_print = now + 0.5
        time.sleep(0.02)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--ip", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=14550)
    args = parser.parse_args()

    client = MAVLinkClient.connect(args.ip, args.port)
    client.start_heartbeat()
    print("Arming...", flush=True)
    client.arm()
    time.sleep(0.5)

    print("\n--- phase 1: MAV_CMD_NAV_TAKEOFF alt=2m ---")
    client.sim_conn.mav.command_long_send(
        client.sim_conn.target_system,
        client.sim_conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0,
        2.0,  # altitude
    )
    ack = client.sim_conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=2)
    if ack is not None:
        result = mavutil.mavlink.enums["MAV_RESULT"][ack.result].name
        print(f"   takeoff ack: command={ack.command} result={result}")
    else:
        print("   no ack for takeoff")
    _watch(client, 4.0, "takeoff")

    print("\n--- phase 2: SET_ATTITUDE_TARGET level, thrust ramp ---")
    for thrust in (0.3, 0.5, 0.6, 0.7, 0.8, 0.9):
        print(f"   thrust={thrust:.1f}")
        t_end = time.monotonic() + 2.0
        next_print = 0.0
        while time.monotonic() < t_end:
            client.sim_conn.mav.set_attitude_target_send(
                client._now_ms(),
                client.sim_conn.target_system,
                client.sim_conn.target_component,
                _ATT_MASK,
                LEVEL_Q,
                0.0, 0.0, 0.0,  # body rates (ignored)
                float(thrust),
            )
            latest = client.recv_telemetry(TELEM_TYPES)
            now = time.monotonic()
            if latest and now >= next_print:
                pos = latest.get("LOCAL_POSITION_NED") or latest.get("ODOMETRY")
                if pos is not None:
                    print(
                        f"      pos z={pos.z:+6.2f} vz={pos.vz:+5.2f}", flush=True
                    )
                next_print = now + 0.5
            time.sleep(0.02)

    print("\nDone. Disarming is not implemented; reset the sim if airborne.")


if __name__ == "__main__":
    main()
