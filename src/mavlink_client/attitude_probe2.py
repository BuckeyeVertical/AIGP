"""Attitude-convention probe 2: feed back the measured attitude as the target.

If the quaternion convention we send matches what the sim expects, then
target == measured attitude -> zero error -> zero torque -> the drone should
hold still (no spin, no tilt). Whichever phase below is stable reveals the
sim's convention. Constant low thrust (0.15) throughout.

  A: q = wxyz(measured roll, pitch, yaw)   (our current convention)
  B: q = xyzw(measured roll, pitch, yaw)   (reordered)
  C: q = wxyz(measured), w forced >= 0     (double-cover canonicalized)
  D: q = wxyz(0, 0, measured yaw)          (level hold, live yaw)
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

    def phase_a(r, p, y):
        return quat_wxyz(r, p, y)

    def phase_b(r, p, y):
        q = quat_wxyz(r, p, y)
        return q[1:] + q[:1]

    def phase_c(r, p, y):
        q = quat_wxyz(r, p, y)
        return [-v for v in q] if q[0] < 0 else q

    def phase_d(r, p, y):
        return quat_wxyz(0.0, 0.0, y)

    rpy = [0.0, 0.0, 0.0]
    for label, make_q in [
        ("A wxyz(measured)", phase_a),
        ("B xyzw(measured)", phase_b),
        ("C wxyz(measured), w>=0", phase_c),
        ("D wxyz(level, live yaw)", phase_d),
    ]:
        print(f"\n--- {label} ---")
        t_end = time.monotonic() + 4.0
        next_print = 0.0
        while time.monotonic() < t_end:
            latest = client.recv_telemetry(TELEM_TYPES)
            a = latest.get("ATTITUDE")
            if a is not None:
                rpy = [a.roll, a.pitch, a.yaw]
            q = make_q(*rpy)
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
                pos = latest.get("LOCAL_POSITION_NED")
                line = [f"rpy=({rpy[0]:+5.2f},{rpy[1]:+5.2f},{rpy[2]:+5.2f})"]
                if a is not None:
                    line.append(f"yawspeed={a.yawspeed:+5.2f}")
                if pos is not None:
                    line.append(f"z={pos.z:+7.2f} vz={pos.vz:+5.2f}")
                print("   " + "  ".join(line), flush=True)
                next_print = now + 0.5
            time.sleep(0.02)

    print("\nDone.")


if __name__ == "__main__":
    main()
