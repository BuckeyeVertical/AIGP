"""Barebones MAVLink client for sending control setpoints.

Thin wrapper over a pymavlink connection. Just enough to arm and stream
body-frame velocity + yaw-rate setpoints for first-attempt visual servoing;
telemetry receive, timesync, and clean shutdown live elsewhere.
"""

from __future__ import annotations

import math
import threading
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

    def start_heartbeat(self, rate_hz=2.0):
        """Stream client heartbeats in a daemon thread (spec minimum: 2 Hz)."""

        def _loop():
            period = 1.0 / rate_hz
            while True:
                self.sim_conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0,
                )
                time.sleep(period)

        thread = threading.Thread(target=_loop, name="heartbeat", daemon=True)
        thread.start()
        return thread

    def arm(self):
        self.sim_conn.mav.command_long_send(
            self.sim_conn.target_system,
            self.sim_conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # arm
            0, 0, 0, 0, 0, 0,
        )

    def send_body_velocity(self, vx, vy, vz, yaw_rate, frame=None):
        """Stream a velocity setpoint in MAV_FRAME_BODY_NED (X fwd, Y right, Z down).

        frame can be overridden (e.g. MAV_FRAME_LOCAL_NED) to test which frames
        the simulator actually honors.
        """
        if frame is None:
            frame = mavutil.mavlink.MAV_FRAME_BODY_NED
        self.sim_conn.mav.set_position_target_local_ned_send(
            self._now_ms(),
            self.sim_conn.target_system,
            self.sim_conn.target_component,
            frame,
            _BODY_VELOCITY_MASK,
            0.0, 0.0, 0.0,            # position (ignored)
            float(vx), float(vy), float(vz),
            0.0, 0.0, 0.0,            # acceleration (ignored)
            0.0,                      # yaw angle (ignored)
            float(yaw_rate),
        )

    def send_attitude_target(self, roll, pitch, yaw, yaw_rate, thrust):
        """Stream an attitude + thrust setpoint (the control path this sim
        actually flies on -- position/velocity setpoints only tilt it).

        roll/pitch/yaw in radians (NED body: +pitch nose up, +roll right wing
        down), yaw_rate in rad/s via the body-rate field, thrust 0..1.
        """
        # Euler ZYX -> quaternion [w, x, y, z]
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        q = [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
        mask = (
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
            | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
        )
        self.sim_conn.mav.set_attitude_target_send(
            self._now_ms(),
            self.sim_conn.target_system,
            self.sim_conn.target_component,
            mask,
            q,
            0.0, 0.0,                 # body roll/pitch rates (ignored)
            float(yaw_rate),
            float(thrust),
        )

    def recv_telemetry(self, types=None):
        """Drain pending MAVLink messages, return the latest one per type."""
        latest = {}
        while True:
            msg = self.sim_conn.recv_match(type=types, blocking=False)
            if msg is None:
                return latest
            latest[msg.get_type()] = msg
