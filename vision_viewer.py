import argparse
import socket
import struct
import time

import cv2
import numpy as np


HEADER_FORMAT = "<IHHIIQ"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)


def receive_frames(ip, port, timeout_s):
    frames = {}

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((ip, port))
    sock.settimeout(0.25)

    try:
        while True:
            try:
                packet, _ = sock.recvfrom(65536)
            except socket.timeout:
                yield None
                continue

            if len(packet) < HEADER_SIZE:
                continue

            header = packet[:HEADER_SIZE]
            payload = packet[HEADER_SIZE:]
            frame_id, chunk_id, total_chunks, jpeg_size, payload_size, sim_time_ns = struct.unpack(
                HEADER_FORMAT,
                header,
            )

            if payload_size != len(payload):
                continue

            frame = frames.setdefault(
                frame_id,
                {
                    "chunks": {},
                    "total": total_chunks,
                    "jpeg_size": jpeg_size,
                    "sim_time_ns": sim_time_ns,
                    "created_at": time.monotonic(),
                },
            )
            frame["chunks"][chunk_id] = payload

            if len(frame["chunks"]) == frame["total"]:
                jpeg_bytes = bytearray()
                complete = True
                for i in range(frame["total"]):
                    chunk = frame["chunks"].get(i)
                    if chunk is None:
                        complete = False
                        break
                    jpeg_bytes.extend(chunk)

                del frames[frame_id]

                if complete and len(jpeg_bytes) == frame["jpeg_size"]:
                    image = cv2.imdecode(np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if image is not None:
                        yield frame_id, image, frame["sim_time_ns"]

            now = time.monotonic()
            stale_frame_ids = [
                stale_id
                for stale_id, stale_frame in frames.items()
                if now - stale_frame["created_at"] > timeout_s
            ]
            for stale_id in stale_frame_ids:
                del frames[stale_id]
    finally:
        sock.close()


def main():
    parser = argparse.ArgumentParser(description="Display the AI GP simulator camera stream.")
    parser.add_argument("--ip", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5600)
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args()

    print(f"Listening for camera frames on udp://{args.ip}:{args.port}")
    print("Press q or Esc in the OpenCV window to exit.")

    window_name = "AI GP Camera"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    last_frame_time = time.monotonic()
    for frame in receive_frames(args.ip, args.port, args.timeout):
        if frame is None:
            if time.monotonic() - last_frame_time > args.timeout:
                placeholder = np.zeros((360, 640, 3), dtype=np.uint8)
                cv2.putText(
                    placeholder,
                    "Waiting for camera frames...",
                    (120, 180),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow(window_name, placeholder)
        else:
            frame_id, image, sim_time_ns = frame
            last_frame_time = time.monotonic()
            cv2.putText(
                image,
                f"frame {frame_id} sim_time_ns {sim_time_ns}",
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )
            cv2.imshow(window_name, image)

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break

        if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
