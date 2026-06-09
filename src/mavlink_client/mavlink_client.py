"""Barebones MAVLink client for sending control setpoints.

Thin wrapper over a pymavlink connection. Just enough to arm and stream
body-frame velocity + yaw-rate setpoints for first-attempt visual servoing;
telemetry receive, timesync, and clean shutdown live elsewhere.
"""

from __future__ import annotations

import time

from pymavlink import mavutil

DEFAULT_IP = "127.0.0.1"
DEFAULT_PORT = 14550

# Body-frame velocity + yaw-rate setpoint: ignore position, acceleration, and the
# yaw *angle*, leaving vx/vy/vz and yaw_rate active.
_BODY_VELOCITY_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
)


class MAVLinkClient:
    def __init__(self, sim_conn, system_boot_ms=None):
        self.sim_conn = sim_conn
        self.system_boot_ms = (
            system_boot_ms if system_boot_ms is not None else int(time.time() * 1000)
        )

    @classmethod
    def connect(cls, ip=DEFAULT_IP, port=DEFAULT_PORT, system_boot_ms=None):
        conn = mavutil.mavlink_connection("udpin:%s:%s" % (ip, port))
        print("Waiting for heartbeat...", flush=True)
        conn.wait_heartbeat()
        print(f"Connected to system: {conn.target_system}", flush=True)
        return cls(conn, system_boot_ms)

    def _now_ms(self):
        return int(time.time() * 1000) - self.system_boot_ms

    def arm(self):
        self.sim_conn.mav.command_long_send(
            self.sim_conn.target_system,
            self.sim_conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # arm
            0, 0, 0, 0, 0, 0,
        )

    def send_body_velocity(self, vx, vy, vz, yaw_rate):
        """Stream a velocity setpoint in MAV_FRAME_BODY_NED (X fwd, Y right, Z down)."""
        self.sim_conn.mav.set_position_target_local_ned_send(
            self._now_ms(),
            self.sim_conn.target_system,
            self.sim_conn.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            _BODY_VELOCITY_MASK,
            0.0, 0.0, 0.0,            # position (ignored)
            float(vx), float(vy), float(vz),
            0.0, 0.0, 0.0,            # acceleration (ignored)
            0.0,                      # yaw angle (ignored)
            float(yaw_rate),
        )
