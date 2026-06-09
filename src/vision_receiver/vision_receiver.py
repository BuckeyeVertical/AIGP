"""Barebones FPV camera receiver.

Thin helper that reassembles the simulator's chunked-JPEG UDP stream into BGR
frames. Reuses the reassembly logic proven in examples/vision_viewer.py. This is
intentionally minimal so the gate detector can be exercised on live video; the
production receiver (threading, shared_data handoff) is a separate effort.
"""

import socket
import struct
import time

import cv2
import numpy as np

HEADER_FORMAT = "<IHHIIQ"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

DEFAULT_IP = "0.0.0.0"
DEFAULT_PORT = 5600


def frames(ip=DEFAULT_IP, port=DEFAULT_PORT, timeout_s=2.0):
    """Yield (frame_id, image_bgr, sim_time_ns) for each complete frame.

    Yields None whenever the socket is idle so callers can keep their UI
    responsive (mirrors examples/vision_viewer.py).
    """
    pending = {}

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
                HEADER_FORMAT, header
            )

            if payload_size != len(payload):
                continue

            frame = pending.setdefault(
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

                del pending[frame_id]

                if complete and len(jpeg_bytes) == frame["jpeg_size"]:
                    image = cv2.imdecode(
                        np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR
                    )
                    if image is not None:
                        yield frame_id, image, frame["sim_time_ns"]

            now = time.monotonic()
            stale = [fid for fid, f in pending.items() if now - f["created_at"] > timeout_s]
            for fid in stale:
                del pending[fid]
    finally:
        sock.close()
