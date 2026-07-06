"""Fast manual flight plus synchronized telemetry and FPV recording.

Controls:
  Arrow Up/Down     forward/back
  Arrow Left/Right  strafe left/right
  W/S               climb/descend
  A/D               yaw left/right
  Esc               stop sending commands and exit

Each run creates a directory under ``logs/manual`` containing controller CSV,
raw MAVLink JSONL, events, metadata, a camera-frame index, and every decoded
FPV frame. Camera recording runs on a separate thread so disk I/O does not
pace the 30 Hz flight-control loop.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

_SRC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_SRC, "control"))
sys.path.insert(0, os.path.join(_SRC, "mavlink_client"))

from control import (  # noqa: E402
    TAKEOFF_ALT,
    TAKEOFF_THRUST,
    Telemetry,
    VelocityCommand,
    VelocityToRates,
)
from mavlink_client import MAVLinkClient  # noqa: E402

CONTROL_HZ = 30.0
TELEMETRY_TIMEOUT_S = 5.0
DEFAULT_SPEED = 2.5
DEFAULT_VERTICAL_SPEED = 1.2
DEFAULT_YAW_RATE = 1.0
KEYS = ("up", "down", "left", "right", "w", "s", "a", "d", "esc")

CONTROL_COLUMNS = (
    "elapsed_s", "utc_ns", "phase", "dt_s", "loop_overrun_s", "mav_messages",
    "key_up", "key_down", "key_left", "key_right", "key_w", "key_s", "key_a", "key_d", "key_esc",
    "requested_vx", "requested_vy", "requested_vz", "requested_yaw_rate",
    "cmd_vx", "cmd_vy", "cmd_vz", "cmd_yaw_rate",
    "have_attitude", "have_position", "have_odometry",
    "roll", "pitch", "yaw", "yaw_speed",
    "vn", "ve", "vd", "body_vx_from_ned", "body_vy_from_ned",
    "odom_vbx", "odom_vby", "x", "y", "z",
    "roll_des", "pitch_des", "roll_q", "pitch_q", "yaw_q", "thrust", "hover_thrust",
    "camera_frame_id", "camera_sim_time_ns", "camera_age_s", "camera_frames_saved",
)


class FlightLogger:
    """Own all artifacts for one manual flight."""

    def __init__(self, output_dir: Path, args):
        self.output_dir = output_dir.resolve()
        self.frames_dir = self.output_dir / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=False)
        self.start_mono = time.monotonic()
        self.start_utc_ns = time.time_ns()
        self._lock = threading.Lock()
        self._file_lock = threading.Lock()
        self._latest_frame = None
        self._frames_saved = 0
        self._camera_errors = 0

        self._control_file = (self.output_dir / "control_ticks.csv").open(
            "w", newline="", encoding="utf-8", buffering=1
        )
        self._control = csv.DictWriter(self._control_file, fieldnames=CONTROL_COLUMNS)
        self._control.writeheader()
        self._mavlink_file = (self.output_dir / "mavlink.jsonl").open(
            "w", encoding="utf-8", buffering=1024 * 1024
        )
        self._events_file = (self.output_dir / "events.jsonl").open(
            "w", encoding="utf-8", buffering=1
        )
        self._camera_file = (self.output_dir / "camera_frames.csv").open(
            "w", newline="", encoding="utf-8", buffering=1
        )
        self._camera = csv.DictWriter(
            self._camera_file,
            fieldnames=(
                "elapsed_s", "utc_ns", "frame_id", "sim_time_ns", "width", "height",
                "filename", "jpeg_bytes", "write_ms",
            ),
        )
        self._camera.writeheader()
        self.metadata = {
            "format_version": 3,
            "started_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
            "started_utc_ns": self.start_utc_ns,
            "control_hz": CONTROL_HZ,
            "camera_format": "decoded BGR frames re-encoded as JPEG",
            "arguments": vars(args),
            "controls": {
                "arrows": "forward/back/strafe",
                "w_s": "climb/descend",
                "a_d": "yaw left/right",
                "escape": "exit",
            },
        }
        self._write_metadata()

    def elapsed(self):
        return time.monotonic() - self.start_mono

    def _write_metadata(self):
        with (self.output_dir / "metadata.json").open("w", encoding="utf-8") as f:
            json.dump(self.metadata, f, indent=2, sort_keys=True)
            f.write("\n")

    def event(self, name, **details):
        record = {
            "elapsed_s": self.elapsed(),
            "utc_ns": time.time_ns(),
            "event": name,
            **details,
        }
        with self._file_lock:
            self._events_file.write(json.dumps(record, default=str) + "\n")

    def mavlink(self, message):
        record = {
            "elapsed_s": self.elapsed(),
            "utc_ns": time.time_ns(),
            "type": message.get_type(),
            "src_system": message.get_srcSystem(),
            "src_component": message.get_srcComponent(),
            "message": message.to_dict(),
        }
        with self._file_lock:
            self._mavlink_file.write(json.dumps(record, default=str) + "\n")

    def control_tick(self, phase, tick_start, dt_s, overrun_s, message_count,
                     keys, requested, cmd, telem, rates, hover_thrust):
        latest = self.latest_frame()
        frame_age = ""
        frame_id = ""
        sim_time_ns = ""
        frames_saved = 0
        if latest is not None:
            frame_id, sim_time_ns, received_mono, frames_saved = latest
            frame_age = max(0.0, tick_start - received_mono)
        cy, sy = math.cos(telem.yaw), math.sin(telem.yaw)
        body_vx = cy * telem.vn + sy * telem.ve
        body_vy = -sy * telem.vn + cy * telem.ve
        row = {
            "elapsed_s": tick_start - self.start_mono,
            "utc_ns": time.time_ns(),
            "phase": phase,
            "dt_s": dt_s,
            "loop_overrun_s": overrun_s,
            "mav_messages": message_count,
            **{f"key_{key}": int(keys.get(key, False)) for key in KEYS},
            "requested_vx": requested.vx, "requested_vy": requested.vy,
            "requested_vz": requested.vz,
            "requested_yaw_rate": requested.yaw_rate,
            "cmd_vx": cmd.vx, "cmd_vy": cmd.vy, "cmd_vz": cmd.vz,
            "cmd_yaw_rate": cmd.yaw_rate,
            "have_attitude": int(telem.have_attitude),
            "have_position": int(telem.have_position),
            "have_odometry": int(telem.have_odometry),
            "roll": telem.roll, "pitch": telem.pitch, "yaw": telem.yaw,
            "yaw_speed": telem.yaw_speed,
            "vn": telem.vn, "ve": telem.ve, "vd": telem.vd,
            "body_vx_from_ned": body_vx, "body_vy_from_ned": body_vy,
            "odom_vbx": telem.vbx, "odom_vby": telem.vby,
            "x": telem.x, "y": telem.y, "z": telem.z,
            "roll_des": rates.roll_des, "pitch_des": rates.pitch_des,
            "roll_q": rates.roll_q, "pitch_q": rates.pitch_q, "yaw_q": rates.yaw_q,
            "thrust": rates.thrust, "hover_thrust": hover_thrust,
            "camera_frame_id": frame_id, "camera_sim_time_ns": sim_time_ns,
            "camera_age_s": frame_age, "camera_frames_saved": frames_saved,
        }
        self._control.writerow(row)

    def save_frame(self, frame_id, image, sim_time_ns, jpeg_quality):
        import cv2

        received_mono = time.monotonic()
        filename = f"frame_{frame_id:010d}_{sim_time_ns}.jpg"
        path = self.frames_dir / filename
        before = time.monotonic()
        ok = cv2.imwrite(
            str(path), image, [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_quality)]
        )
        write_ms = (time.monotonic() - before) * 1000.0
        if not ok:
            raise OSError(f"OpenCV failed to write {path}")
        size = path.stat().st_size
        height, width = image.shape[:2]
        with self._lock:
            self._frames_saved += 1
            frames_saved = self._frames_saved
            self._latest_frame = (frame_id, sim_time_ns, received_mono, frames_saved)
            self._camera.writerow({
                "elapsed_s": received_mono - self.start_mono,
                "utc_ns": time.time_ns(), "frame_id": frame_id,
                "sim_time_ns": sim_time_ns, "width": width, "height": height,
                "filename": f"frames/{filename}", "jpeg_bytes": size,
                "write_ms": write_ms,
            })

    def camera_error(self, exc):
        with self._lock:
            self._camera_errors += 1
        self.event("camera_error", error=repr(exc))

    def latest_frame(self):
        with self._lock:
            return self._latest_frame

    def close(self):
        with self._lock:
            frames_saved = self._frames_saved
            camera_errors = self._camera_errors
        self.metadata.update({
            "ended_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
            "duration_s": self.elapsed(),
            "frames_saved": frames_saved,
            "camera_errors": camera_errors,
        })
        self._write_metadata()
        self._control_file.close()
        self._mavlink_file.close()
        self._events_file.close()
        self._camera_file.close()


def _default_log_dir():
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    return Path(_SRC).parent / "logs" / "manual" / stamp


def _camera_worker(logger, stop_event, ip, port, jpeg_quality):
    from vision_receiver import frames

    try:
        for item in frames(ip, port, timeout_s=1.0):
            if stop_event.is_set():
                return
            if item is None:
                continue
            frame_id, image, sim_time_ns = item
            try:
                logger.save_frame(frame_id, image, sim_time_ns, jpeg_quality)
            except Exception as exc:  # keep receiving after a single disk error
                logger.camera_error(exc)
    except Exception as exc:
        logger.camera_error(exc)


def _pressed(keyboard, key: str) -> bool:
    return bool(keyboard.is_pressed(key))


def read_keys(keyboard):
    return {key: _pressed(keyboard, key) for key in KEYS}


def command_from_keys(keys, speed: float, vertical_speed: float, yaw_rate: float):
    forward = float(keys["up"]) - float(keys["down"])
    right = float(keys["right"]) - float(keys["left"])
    magnitude = math.hypot(forward, right)
    if magnitude > 1.0:
        forward /= magnitude
        right /= magnitude
    climb = float(keys["w"]) - float(keys["s"])
    yaw = float(keys["d"]) - float(keys["a"])
    return VelocityCommand(
        vx=speed * forward,
        vy=speed * right,
        vz=-vertical_speed * climb,
        yaw_rate=yaw_rate * yaw,
    )


def read_command(keyboard, speed: float, vertical_speed: float, yaw_rate: float):
    """Compatibility/test seam: sample keys and produce one command."""
    return command_from_keys(read_keys(keyboard), speed, vertical_speed, yaw_rate)


def _drain_telemetry(client, logger):
    latest = {}
    count = 0
    while True:
        message = client.sim_conn.recv_match(blocking=False)
        if message is None:
            return latest, count
        logger.mavlink(message)
        latest[message.get_type()] = message
        count += 1


def _wait_for_telemetry(client, telem, logger):
    deadline = time.monotonic() + TELEMETRY_TIMEOUT_S
    while time.monotonic() < deadline:
        latest, _ = _drain_telemetry(client, logger)
        telem.update_from(latest)
        if telem.have_attitude and telem.have_position:
            return
        time.sleep(0.02)
    raise RuntimeError("Timed out waiting for ATTITUDE and LOCAL_POSITION_NED")


def _send(client, inner, telem, cmd, dt_s):
    applied = VelocityCommand(cmd.vx, cmd.vy, cmd.vz, cmd.yaw_rate)
    rates = inner.update(applied, telem, dt_s)
    client.send_attitude_target(
        rates.roll_q, rates.pitch_q, rates.yaw_q, 0.0, rates.thrust
    )
    return rates, applied


def main():
    parser = argparse.ArgumentParser(description="Fast manual flight with full logging.")
    parser.add_argument("--mav-ip", default="127.0.0.1")
    parser.add_argument("--mav-port", type=int, default=14550)
    parser.add_argument("--cam-ip", default="0.0.0.0")
    parser.add_argument("--cam-port", type=int, default=5600)
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED, help="horizontal m/s")
    parser.add_argument("--vertical-speed", type=float, default=DEFAULT_VERTICAL_SPEED)
    parser.add_argument("--yaw-rate", type=float, default=DEFAULT_YAW_RATE, help="rad/s")
    parser.add_argument("--takeoff-altitude", type=float, default=-TAKEOFF_ALT)
    parser.add_argument("--log-dir", default="", help="run output directory")
    parser.add_argument("--jpeg-quality", type=int, default=95, help="saved frame quality 1-100")
    parser.add_argument("--no-camera-log", action="store_true")
    args = parser.parse_args()

    if args.speed <= 0 or args.vertical_speed <= 0 or args.yaw_rate <= 0:
        parser.error("speed, vertical-speed, and yaw-rate must be positive")
    if args.takeoff_altitude <= 0:
        parser.error("takeoff-altitude must be positive")
    if not 1 <= args.jpeg_quality <= 100:
        parser.error("jpeg-quality must be in 1..100")

    try:
        import keyboard
    except ImportError as exc:
        raise SystemExit("Missing dependency: run 'pip install -r requirements.txt'") from exc

    log_dir = Path(args.log_dir) if args.log_dir else _default_log_dir()
    logger = FlightLogger(log_dir, args)
    stop_camera = threading.Event()
    camera_thread = None
    client = None
    logger.event("run_started")
    print(f"Logging everything to {logger.output_dir}", flush=True)

    try:
        if not args.no_camera_log:
            camera_thread = threading.Thread(
                target=_camera_worker,
                args=(logger, stop_camera, args.cam_ip, args.cam_port, args.jpeg_quality),
                name="camera-logger",
                daemon=True,
            )
            camera_thread.start()
            logger.event("camera_logger_started", ip=args.cam_ip, port=args.cam_port)

        client = MAVLinkClient.connect(args.mav_ip, args.mav_port)
        client.start_heartbeat()
        telem = Telemetry()
        inner = VelocityToRates()
        _wait_for_telemetry(client, telem, logger)

        print("Arming and taking off...", flush=True)
        logger.event("arming")
        client.arm()
        target_z = -args.takeoff_altitude
        period = 1.0 / CONTROL_HZ
        last_t = time.monotonic()

        while telem.z > target_z:
            tick = time.monotonic()
            dt_s = tick - last_t
            last_t = tick
            latest, message_count = _drain_telemetry(client, logger)
            telem.update_from(latest)
            keys = read_keys(keyboard)
            cmd = VelocityCommand(vz=-0.8)
            rates = inner.update(cmd, telem, dt_s)
            rates.thrust = max(rates.thrust, TAKEOFF_THRUST)
            client.send_attitude_target(
                rates.roll_q, rates.pitch_q, rates.yaw_q, 0.0, rates.thrust
            )
            elapsed = time.monotonic() - tick
            logger.control_tick(
                "takeoff", tick, dt_s, max(0.0, elapsed - period), message_count,
                keys, cmd, cmd, telem, rates, inner.hover_thrust,
            )
            if keys["esc"]:
                logger.event("takeoff_cancelled")
                return
            time.sleep(max(0.0, period - elapsed))

        logger.event("takeoff_complete", z=telem.z)
        print(
            f"Ready at {args.speed:.1f} m/s: arrows move, W/S climb/descend, "
            "A/D yaw, Esc exits.", flush=True,
        )
        next_status = 0.0
        try:
            while not _pressed(keyboard, "esc"):
                tick = time.monotonic()
                dt_s = tick - last_t
                last_t = tick
                latest, message_count = _drain_telemetry(client, logger)
                telem.update_from(latest)
                keys = read_keys(keyboard)
                requested = command_from_keys(
                    keys, args.speed, args.vertical_speed, args.yaw_rate
                )
                rates, cmd = _send(client, inner, telem, requested, dt_s)
                elapsed = time.monotonic() - tick
                logger.control_tick(
                    "manual", tick, dt_s, max(0.0, elapsed - period), message_count,
                    keys, requested, cmd, telem, rates, inner.hover_thrust,
                )
                if tick >= next_status:
                    print(
                        f"xyz=({telem.x:+.1f},{telem.y:+.1f},{telem.z:+.1f}) "
                        f"cmd=({cmd.vx:+.1f},{cmd.vy:+.1f},{cmd.vz:+.1f}) "
                        f"yaw={cmd.yaw_rate:+.1f} thrust={rates.thrust:.2f}",
                        flush=True,
                    )
                    next_status = tick + 1.0
                time.sleep(max(0.0, period - elapsed))
        except KeyboardInterrupt:
            logger.event("keyboard_interrupt")
        logger.event("manual_exit")
    except Exception as exc:
        logger.event("fatal_error", error=repr(exc))
        raise
    finally:
        stop_camera.set()
        if camera_thread is not None:
            camera_thread.join()
        logger.close()
        print(f"Logs saved to {logger.output_dir}", flush=True)

    print("Stopped. The drone is still armed; reset the simulator before another run.")


if __name__ == "__main__":
    main()
