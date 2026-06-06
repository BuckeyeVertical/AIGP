import argparse
import socket
import struct
import time

import cv2
import numpy as np
from pymavlink import mavutil


VISION_HEADER_FORMAT = "<IHHIIQ"
VISION_HEADER_SIZE = struct.calcsize(VISION_HEADER_FORMAT)


def test_mavlink(ip, port, timeout_s):
    print(f"[mavlink] listening on udp://{ip}:{port}")
    conn = mavutil.mavlink_connection(f"udpin:{ip}:{port}")

    heartbeat = conn.wait_heartbeat(timeout=timeout_s)
    if heartbeat is None:
        raise TimeoutError(f"no MAVLink HEARTBEAT received within {timeout_s:.1f}s")

    print(
        "[mavlink] heartbeat ok "
        f"system={conn.target_system} component={conn.target_component} "
        f"type={heartbeat.type} autopilot={heartbeat.autopilot}"
    )

    deadline = time.monotonic() + timeout_s
    wanted = {"ATTITUDE", "HIGHRES_IMU", "LOCAL_POSITION_NED", "ODOMETRY"}
    seen = {}

    while time.monotonic() < deadline and not (wanted & seen.keys()):
        msg = conn.recv_match(blocking=True, timeout=0.25)
        if msg is None or msg.get_type() == "BAD_DATA":
            continue
        seen[msg.get_type()] = msg

    if wanted & seen.keys():
        print(f"[mavlink] telemetry ok messages={', '.join(sorted(seen))}")
    else:
        print("[mavlink] heartbeat received, but no telemetry message was seen before timeout")

    conn.close()


def test_vision(ip, port, timeout_s):
    print(f"[vision] listening on udp://{ip}:{port}")
    frames = {}
    deadline = time.monotonic() + timeout_s

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((ip, port))
    sock.settimeout(0.25)

    try:
        while time.monotonic() < deadline:
            try:
                packet, _ = sock.recvfrom(65536)
            except socket.timeout:
                continue

            if len(packet) < VISION_HEADER_SIZE:
                continue

            header = packet[:VISION_HEADER_SIZE]
            payload = packet[VISION_HEADER_SIZE:]
            (
                frame_id,
                chunk_id,
                total_chunks,
                jpeg_size,
                payload_size,
                sim_time_ns,
            ) = struct.unpack(VISION_HEADER_FORMAT, header)

            if payload_size != len(payload):
                print(
                    "[vision] warning "
                    f"frame={frame_id} chunk={chunk_id} header_payload_size={payload_size} "
                    f"actual_payload_size={len(payload)}"
                )

            frame = frames.setdefault(
                frame_id,
                {
                    "chunks": {},
                    "total": total_chunks,
                    "jpeg_size": jpeg_size,
                    "sim_time_ns": sim_time_ns,
                },
            )
            frame["chunks"][chunk_id] = payload

            if len(frame["chunks"]) != frame["total"]:
                continue

            jpeg_bytes = bytearray()
            for i in range(frame["total"]):
                if i not in frame["chunks"]:
                    break
                jpeg_bytes.extend(frame["chunks"][i])
            else:
                image = cv2.imdecode(np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise RuntimeError(f"received complete frame {frame_id}, but JPEG decode failed")

                height, width = image.shape[:2]
                print(
                    "[vision] frame ok "
                    f"frame={frame_id} chunks={frame['total']} jpeg_bytes={len(jpeg_bytes)}/"
                    f"{frame['jpeg_size']} image={width}x{height} sim_time_ns={frame['sim_time_ns']}"
                )
                return

            del frames[frame_id]

    finally:
        sock.close()

    raise TimeoutError(f"no complete vision frame received within {timeout_s:.1f}s")


def main():
    parser = argparse.ArgumentParser(description="Smoke test the AI GP simulator connection.")
    parser.add_argument("--mavlink-ip", default="127.0.0.1")
    parser.add_argument("--mavlink-port", type=int, default=14550)
    parser.add_argument("--vision-ip", default="0.0.0.0")
    parser.add_argument("--vision-port", type=int, default=5600)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument(
        "--skip-mavlink",
        action="store_true",
        help="Only test the UDP vision stream.",
    )
    parser.add_argument(
        "--skip-vision",
        action="store_true",
        help="Only test MAVLink heartbeat/telemetry.",
    )
    args = parser.parse_args()

    if not args.skip_mavlink:
        test_mavlink(args.mavlink_ip, args.mavlink_port, args.timeout)
    if not args.skip_vision:
        test_vision(args.vision_ip, args.vision_port, args.timeout)

    print("[ok] simulator connection smoke test passed")


if __name__ == "__main__":
    main()
