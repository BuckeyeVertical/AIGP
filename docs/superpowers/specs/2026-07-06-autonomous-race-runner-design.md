# Autonomous Race Runner — Design

Date: 2026-07-06
Goal: complete the full VQ1 course (6 gates) autonomously, replacing the
vision-only pipeline that reliably passed gate 1 but could not reacquire
gate 2.

## Key discovery (from logs/manual/20260619_141409_342248)

The simulator broadcasts two official data feeds over `ENCAPSULATED_DATA`
that the starter code (`examples/mavlink_rx.py`) parses but discards:

1. **Race status** (payload type 1, ~4 Hz, streams continuously):
   `<BQqqIq` = data_type, sim_boot_time_ms, race_start_boot_time_ms,
   race_finish_time_ns, active_gate_index, last_gate_race_time.
   `active_gate_index` is the authoritative "which gate is next" signal;
   `race_finish_time_ns > 0` is the authoritative finish signal.
2. **Track data** (payload type 2, chunked, announced by
   `DATA_TRANSMISSION_HANDSHAKE` where `width`=transfer_id,
   `packets`=chunk count): gate count then per gate
   `<Hfffffffff>` = id, NED position x/y/z, orientation quaternion wxyz,
   width, height. Broadcast at race (re)start, not continuously.

Decoded course (deterministic environment, same for all participants):

| gate | x | y | z | size |
|---|---|---|---|---|
| 0 | -23.30 | -0.40 | -0.03 | 2.72×2.72 |
| 1 | -46.89 | -2.50 | +5.07 | 2.72×2.72 |
| 2 | -74.59 | +1.20 | +13.67 | 2.72×2.72 |
| 3 | -111.49 | -5.10 | +24.57 | 2.72×2.72 |
| 4 | -135.49 | -0.80 | +25.36 | 2.72×2.72 |
| 5 | -159.19 | -4.40 | +25.97 | 2.72×2.72 |

All gates share orientation q=(0.707, 0, 0, 0.707) (facing ±X). The course
runs along −X and DESCENDS 26 m (NED +z). The successful manual run crossed
every gate at drone z ≈ gate z − 1.2…−1.5, i.e. `position_ned_z` is the
gate's BOTTOM edge; the crossing target is `gate_z − half_height` (−1.36 m).

The manual pilot never yawed: the whole course was flown at yaw ≈ π with
strafe/climb corrections at ~2.3 m/s, finishing in 109.6 s.

## Approach

**Waypoint corridor flight on `LOCAL_POSITION_NED`, sequenced by race
status.** Vision is not needed for course completion (kept for the existing
debug runners). Alternatives considered:

- *Vision-primary servoing* (current center.py/two_gate.py): proven fragile —
  bbox slide near gates, no detection at point-blank, gate-2 reacquisition
  unsolved. Rejected as primary; superseded.
- *Replay of manual velocity commands*: open-loop, drifts with any timing
  difference. Rejected.
- *Waypoints + visual trim*: deferred (YAGNI) — the manual run shows position
  telemetry alone tracks gate centers to well within the ±1.36 m half-width.

## Components

- `src/mavlink_client/race_data.py` — pure decoders + `RaceDataTracker`
  (feed every MAVLink message; exposes latest `RaceStatus` and `Gate` list
  after chunk reassembly). `__main__` replays a `mavlink.jsonl` for offline
  verification.
- `src/mavlink_client/mavlink_client.py` — add `recv_all()` (drain every
  pending message; latest-per-type loses track chunks), `disarm()`,
  `send_sim_reset()` (COMMAND_LONG 31000).
- `src/run/race.py` — the autonomous entry point:
  1. connect, heartbeat, wait for telemetry; collect track data if broadcast,
     else use the baked-in fallback table above (env is deterministic).
  2. arm, takeoff to z = −1.5 (existing proven takeoff).
  3. per gate: target point = (gate_x, gate_y, gate_z − 1.36). Desired world
     velocity: forward −X at cruise speed (slowing when cross-track error is
     large or the gate plane is near), plus P-terms on y and z errors.
     Rotate into body frame by current yaw, feed the existing calibrated
     `VelocityToRates` cascade. Gentle yaw-rate P-term holds yaw = π.
  4. advance to the next gate when `active_gate_index` increments (fallback:
     drone x is past the gate plane).
  5. finish when `race_finish_time_ns > 0` (fallback: past the last gate):
     brake to hover, report the race time, exit cleanly.
- Safety: `--max-seconds` (default 300; spec cap is 480), corridor sanity
  bounds on y/z, and hover-brake on any unexpected state.
- Logging: per-tick CSV under `logs/race/<stamp>.csv` (position, targets,
  commands, race status), same spirit as manual.py.

## Error handling

- Race status missing → position-based gate advancement still completes the
  course.
- Track data missing → fallback table.
- DISARM/respawn (out-of-bounds) → detected via position snapping to spawn /
  z discontinuity is out of scope for v1; max-seconds bounds the failure.

## Testing

- Offline: replay the manual run's `mavlink.jsonl` through `race_data.py`;
  expect 6 gates, the exact positions above, gate index transitions at the
  logged times, finish 109.6 s.
- Live: run on the Windows sim host over SSH; reset sim via COMMAND_LONG
  31000 between attempts; iterate until the full course completes with a
  valid finish time.
