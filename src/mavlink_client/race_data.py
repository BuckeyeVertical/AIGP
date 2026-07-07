"""Decoders for the simulator's ENCAPSULATED_DATA race feeds.

The sim broadcasts two official data feeds the starter code parses but
discards (format from examples/mavlink_rx.py, verified against the
2026-06-19 manual run log):

* Race status (payload type 1, ~4 Hz, streams continuously):
  ``active_gate_index`` is the authoritative "which gate is next" signal and
  ``race_finish_time_ns > 0`` the authoritative finish signal. Values are -1
  ("not yet") before the race starts / finishes.
* Track data (payload type 2): the full course map -- every gate's NED
  position, orientation quaternion, and size. It arrives as chunks announced
  by a DATA_TRANSMISSION_HANDSHAKE (``width`` = transfer id, ``packets`` =
  chunk count) and is broadcast at race (re)start only, so a client that
  attaches mid-session may never see it (keep a fallback table).

``RaceDataTracker`` is the stateful consumer: feed it every received MAVLink
message and read ``.status`` / ``.gates``. Run this module directly against a
``mavlink.jsonl`` from a logged flight to verify decoding offline.

Gate geometry note (measured from the successful manual run): the drone
crossed every gate at z ~= gate.z - 1.2..1.5, so ``position_ned_z`` is the
gate's BOTTOM edge; aim for ``gate.z - height/2``.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

RACE_STATUS_MSG_ID = 1
TRACK_INFO_MSG_ID = 2

_RACE_STATUS_FMT = "<BQqqIq"
_TRACK_HEADER_FMT = "<BH"
_GATE_FMT = "<Hfffffffff"
_GATE_SIZE = struct.calcsize(_GATE_FMT)  # 38 bytes


@dataclass
class RaceStatus:
    sim_boot_time_ms: int = 0
    race_start_boot_time_ms: int = -1
    race_finish_time_ns: int = -1
    active_gate_index: int = 0
    last_gate_race_time_ns: int = -1

    @property
    def started(self) -> bool:
        return self.race_start_boot_time_ms >= 0

    @property
    def finished(self) -> bool:
        return self.race_finish_time_ns > 0

    @property
    def finish_time_s(self) -> float:
        return self.race_finish_time_ns / 1e9 if self.finished else -1.0


@dataclass
class Gate:
    gate_id: int
    x: float  # NED, position of the gate's BOTTOM edge center
    y: float
    z: float
    qw: float = 1.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    width: float = 2.72
    height: float = 2.72

    @property
    def center_z(self) -> float:
        """NED z of the gate opening's center (bottom edge minus half height)."""
        return self.z - self.height / 2.0


def decode_race_status(payload: bytes) -> RaceStatus:
    (_dtype, boot_ms, start_ms, finish_ns, gate_idx, last_gate_ns) = struct.unpack_from(
        _RACE_STATUS_FMT, payload
    )
    return RaceStatus(boot_ms, start_ms, finish_ns, gate_idx, last_gate_ns)


def decode_track_data(payload: bytes) -> list[Gate]:
    """Decode a fully reassembled track-data payload into gates, id order."""
    (num_gates,) = struct.unpack_from("<H", payload)
    gates = []
    offset = 2
    for _ in range(num_gates):
        vals = struct.unpack_from(_GATE_FMT, payload, offset)
        offset += _GATE_SIZE
        gates.append(Gate(*vals))
    gates.sort(key=lambda g: g.gate_id)
    return gates


@dataclass
class RaceDataTracker:
    """Feed every received MAVLink message; exposes race status + track map."""

    status: RaceStatus = field(default_factory=RaceStatus)
    have_status: bool = False
    gates: list[Gate] = field(default_factory=list)
    _chunks: dict = field(default_factory=dict)  # transfer_id -> {seqnr: bytes}
    _expected: dict = field(default_factory=dict)  # transfer_id -> chunk count

    def update(self, msg) -> None:
        mtype = msg.get_type()
        if mtype == "DATA_TRANSMISSION_HANDSHAKE":
            # Repurposed as the track-data announcement.
            self._chunks[msg.width] = {}
            self._expected[msg.width] = msg.packets
        elif mtype == "ENCAPSULATED_DATA":
            self._on_encapsulated(msg)

    def _on_encapsulated(self, msg) -> None:
        payload = bytes(msg.data)
        if not payload:
            return
        dtype = payload[0]
        if dtype == RACE_STATUS_MSG_ID:
            self.status = decode_race_status(payload)
            self.have_status = True
        elif dtype == TRACK_INFO_MSG_ID:
            _dtype, transfer_id = struct.unpack_from(_TRACK_HEADER_FMT, payload)
            if transfer_id not in self._expected:
                return
            self._chunks[transfer_id][msg.seqnr] = payload[3:]
            if len(self._chunks[transfer_id]) == self._expected[transfer_id]:
                chunks = self._chunks.pop(transfer_id)
                del self._expected[transfer_id]
                full = b"".join(chunks[i] for i in sorted(chunks))
                self.gates = decode_track_data(full)


# --------------------------------------------------------------------------
# Offline verification: replay a logged mavlink.jsonl through the tracker
# --------------------------------------------------------------------------

class _JsonMsg:
    """Adapts a logged mavlink.jsonl record to the pymavlink message API the
    tracker uses (get_type() plus attribute access to fields)."""

    def __init__(self, record):
        self._type = record["type"]
        for key, value in record["message"].items():
            setattr(self, key, value)

    def get_type(self):
        return self._type


def _replay(path):
    import json

    tracker = RaceDataTracker()
    prev_idx, prev_finished = None, None
    with open(path, encoding="utf-8") as f:
        for line in f:
            record = json.loads(line)
            if record["type"] not in ("ENCAPSULATED_DATA", "DATA_TRANSMISSION_HANDSHAKE"):
                continue
            tracker.update(_JsonMsg(record))
            s = tracker.status
            if (s.active_gate_index, s.finished) != (prev_idx, prev_finished):
                prev_idx, prev_finished = s.active_gate_index, s.finished
                print(
                    f"t={record['elapsed_s']:8.2f}s gate_idx={s.active_gate_index} "
                    f"started={s.started} finished={s.finished} "
                    f"finish_s={s.finish_time_s:.2f}"
                )
    print(f"\ngates decoded: {len(tracker.gates)}")
    for g in tracker.gates:
        print(
            f"  gate {g.gate_id}: ({g.x:+8.2f},{g.y:+6.2f},{g.z:+6.2f}) "
            f"center_z={g.center_z:+6.2f} {g.width:.2f}x{g.height:.2f}m"
        )


if __name__ == "__main__":
    import sys

    if len(sys.argv) != 2:
        raise SystemExit("usage: python race_data.py <mavlink.jsonl>")
    _replay(sys.argv[1])
