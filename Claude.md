# Claude.md

This repository is a starter client for the AI Grand Prix Virtual Qualifier. Coding agents should treat the task as building an autonomous drone racer, not a generic simulator demo.

## Keeping This File Current

This file is the shared source of truth for agents working in this repo. Keep
it in sync with reality:

- Whenever you learn something non-obvious about the simulator, the control
  stack, or the codebase (a new sign convention, a quirk, a fixed bug, a
  working strategy), record it here in the relevant section.
- Whenever something changes — files move or are renamed, the run command
  changes, interfaces or constants change, a "known gap" is closed — update
  every place in this file that referenced the old behavior (file map, run
  commands, caveats, current state).
- Update CLAUDE.md as part of the same change that caused it, not as a
  follow-up. A change that alters documented behavior is not done until this
  file reflects it.
- Prefer correcting/replacing stale lines over appending duplicates.
- Keep it DURABLE: document how the sim behaves and how the code is meant to
  work (conventions, frames/directions, gotchas, design intent). Do NOT paste
  specific run logs, telemetry dumps, one-off measured positions, dated "ran it
  and it worked" notes, or other transient state — that is noise for the next
  agent. Provenance like "established by probing" is fine; a play-by-play of a
  run is not.

## Competition Objective

Virtual Qualifier 1 is an autonomous gate-navigation time trial. The drone must complete a course made of a start gate, intermediate gates, and a finish gate. The practical goal is to pass every required gate quickly and reliably using onboard telemetry and the first-person camera stream.

Key constraints from `Technical_Spec.pdf`:

- Round One verifies that contestant software can successfully navigate the racecourse.
- The maximum run duration is 8 minutes.
- Human interaction during a submitted timed flight is grounds for disqualification.
- The environment is deterministic and identical for all participants.
- No GPS or geographic coordinates are available.
- The simulator uses a local Cartesian frame internally, and MAVLink coordinate convention is NED.
- The simulator exposes a forward-facing FPV camera and telemetry; LiDAR is not part of this starter interface.
- Gates are visually distinctive from the environment for Virtual Qualifier 1.

In plain engineering terms: build a Python control stack that detects gates from camera frames, uses telemetry for stabilization/state estimation, sends MAVLink control commands, and completes the gate sequence with a fast valid time.

## Simulator Interfaces

### MAVLink

The simulator communicates over UDP using MAVLink 2-compatible messages.

- Default endpoint used by this repo: `udpin:127.0.0.1:14550`
- Physics update rate in the spec: 120 Hz
- Spec command rate limit: less than 100 Hz
- Minimum heartbeat rate in the spec: 2 Hz

Simulator-to-client messages called out in the spec include:

- `HEARTBEAT`
- `ATTITUDE`
- `HIGHRES_IMU`
- `TIMESYNC`

The starter receiver also handles additional useful messages observed or anticipated by the sample code:

- `LOCAL_POSITION_NED`
- `ODOMETRY`
- `ACTUATOR_OUTPUT_STATUS`
- `COLLISION`
- `DATA_TRANSMISSION_HANDSHAKE`
- `ENCAPSULATED_DATA`

Client-to-simulator control messages supported by the starter code:

- `SET_ATTITUDE_TARGET`
- `SET_POSITION_TARGET_LOCAL_NED`
- `SET_ACTUATOR_CONTROL_TARGET`
- `COMMAND_LONG` for arming and simulator reset

### Vision Stream

The FPV camera arrives on a separate UDP stream.

- Default bind address in this repo: `0.0.0.0:5600`
- Spec frequency: 30 Hz
- Resolution: 640 x 360
- Header format: little-endian `"<IHHIIQ"`
- Header size: 24 bytes
- Payload: chunked JPEG bytes

Header fields:

- `frame_id`: uint32, unique image frame sequence id
- `chunk_id`: uint16, packet index inside the frame
- `total_chunks`: uint16, number of chunks required for the frame
- `jpeg_size`: uint32, final reconstructed JPEG byte length
- `payload_size`: uint32, bytes in this packet payload
- `sim_time_ns`: uint64, simulation timestamp in nanoseconds

Camera model from the spec:

- Pinhole camera, no lens distortion
- Resolution: 640 x 360
- Principal point: `[cx, cy] = [320, 180]`
- Focal lengths: `[fx, fy] = [320, 320]`
- Vertical field of view listed as 90 degrees
- Camera and body frame share the same origin
- Camera is tilted upward by 20 degrees relative to body

## Coordinate Frames

The spec uses MAVLink NED conventions:

- `MAV_FRAME_LOCAL_NED`: fixed local origin, usually where the drone armed
- `MAV_FRAME_BODY_NED`: origin at the vehicle, X forward, Y right, Z down
- Body-to-IMU transform is identity

Agents working on perception should be careful when converting image coordinates into body or camera-frame directions. OpenCV image coordinates are pixel-space with Y down; MAVLink/body NED has X forward, Y right, Z down; the camera has a 20 degree upward tilt.

## Repository File Map

- `main.py`: sample application entry point. Creates shared state, wires components with `setup_components`, arms the drone, then calls `controller.update()` forever.
- `setup.py`: builds the MAVLink connection, starts `MAVLinkRX`, creates `TimeSync`, starts `VisionRX`, and returns the `Controller`.
- `controller.py`: sample command sender. Contains examples for direct motor control, attitude-rate control, position/velocity target control, arming, and simulator reset.
- `mavlink_rx.py`: background MAVLink receive loop. Parses telemetry, collision, race status, and track-data packets, but currently does not store most parsed values into `shared_data`.
- `vision_rx.py`: background UDP vision receiver. Reassembles chunked JPEG frames and calls `process_frame(frame_id, image)`. `process_frame` is currently a stub.
- `timesync.py`: intended 10 Hz MAVLink `TIMESYNC` request loop.
- `connection_smoke_test.py`: CLI smoke test for simulator connectivity. It verifies MAVLink heartbeat/telemetry and one complete decodable vision frame.
- `vision_viewer.py`: visual smoke/debug tool for the FPV camera stream. It displays reconstructed frames in an OpenCV window.
- `requirements.txt`: Python dependencies: `pymavlink`, `opencv-python`, `numpy`, `matplotlib`, `keyboard`.
- `Technical_Spec.pdf`: official technical interface/specification for the virtual qualifier.

## Runtime Flow

Expected startup flow:

1. Start the simulator.
2. Run the Python client or a smoke test.
3. MAVLink client listens on UDP port `14550` and waits for `HEARTBEAT`.
4. Vision receiver binds UDP port `5600` and reconstructs JPEG frames.
5. Controller arms the drone.
6. Control loop repeatedly sends commands to the simulator.
7. MAVLink and vision threads update shared state for perception/control.

Current `main.py` loop never exits on its own. Any production controller should add an explicit shutdown condition, race-finished detection, exception handling, and clean thread shutdown.

## Smoke Tests

Use `connection_smoke_test.py` to validate that the simulator is reachable before debugging autonomy:

```powershell
python connection_smoke_test.py
```

Useful options:

```powershell
python connection_smoke_test.py --skip-vision
python connection_smoke_test.py --skip-mavlink
python connection_smoke_test.py --timeout 15
```

Expected successful behavior:

- MAVLink heartbeat is received from `127.0.0.1:14550`.
- At least one telemetry message such as `ATTITUDE`, `HIGHRES_IMU`, `LOCAL_POSITION_NED`, or `ODOMETRY` is seen.
- One complete camera frame is reconstructed and decoded from `0.0.0.0:5600`.

Use `vision_viewer.py` as the camera-stream smoke/debug tool:

```powershell
python vision_viewer.py
```

It opens an OpenCV window and overlays `frame_id` and `sim_time_ns`.

## Current Starter-Code Caveats

These are important when modifying the repo:

- `controller.py` currently sends `SET_ACTUATOR_CONTROL_TARGET` through `update_motor_control()`, but the sample motor constants are not a real flight policy. Three motor constants are zero, and `MOTOR_BACK_LEFT` / `MOTOR_BACK_RIGHT` are both initialized to `0`.
- `controller.py` sets `CONTROL_HZ = 250`, while the spec says command rate must be less than 100 Hz. A competition-ready controller should lower the command rate or otherwise respect the official limit.
- `setup.py` constructs `TimeSync(sim_conn, shared_data)` directly. That does not start the timesync thread because `TimeSync.__init__` only initializes the object. Use `TimeSync.create_timesync(...)` if active timesync requests are needed.
- `mavlink_rx.py` parses telemetry into local variables but does not persist most values into `shared_data`. A real controller will need shared state, queues, callbacks, or another thread-safe handoff.
- `vision_rx.py` decodes frames but leaves `process_frame()` empty. Gate detection belongs here or in a perception module called from here.
- `main.py` has an infinite loop and no race-finished stop condition.
- `main.py` calls `ts_loop.get_thread_for_join().join(...)` after the loop, but if `TimeSync` was not started then `get_thread_for_join()` returns `None`.
- Track-map knowledge is a sanctioned data source: the simulator itself broadcasts every gate's NED pose over `ENCAPSULATED_DATA`, and the official starter code (`examples/mavlink_rx.py`) parses it. See "Race status and track data" below.

## Recommended Agent Work Plan

When implementing autonomy, keep changes staged around this pipeline:

1. Connection health: keep `connection_smoke_test.py` passing.
2. Telemetry state: make `mavlink_rx.py` store attitude, rates, IMU, velocity, race status, collision, and actuator feedback into `shared_data` or a typed state object.
3. Perception: implement gate detection from `vision_rx.py` frames. For VQ1, start with color/contrast thresholding because gates are visually distinctive and the environment is simplified.
4. Target selection: estimate the gate center and apparent size in image space. Keep the target centered and use apparent size/vertical alignment as a rough distance/progress proxy.
5. Control: begin with conservative attitude-rate or velocity commands before attempting low-level motor control.
6. Race logic: use `active_gate_index`, `last_gate_race_time`, finish status, and collision messages when available.
7. Logging/tuning: log frame timestamps, detected gate center/area, commands, telemetry, active gate, collisions, and finish time.

For an initial valid run, reliability matters more than peak speed. Once gates are passed consistently, tune command gains, speed schedule, and gate traversal behavior to reduce time.

## Control Strategy Notes

Prefer the higher-level MAVLink controls first:

- `SET_ATTITUDE_TARGET` can command body roll/pitch/yaw rates and thrust.
- `SET_POSITION_TARGET_LOCAL_NED` can command local-frame velocity if allowed and stable in the simulator.
- Direct motor control should be treated as advanced because it bypasses higher-level stabilization.

A simple VQ1 baseline can be:

- Detect highlighted gate mask in the FPV image.
- Compute gate center error from image center.
- Use horizontal error to command yaw or roll.
- Use vertical error to command pitch/thrust correction.
- Increase forward speed when centered and the gate appears stable.
- Slow down or widen search behavior when detection confidence is low.
- After a gate pass, reacquire the next visible gate using the same visual policy.

## Definition of Done for Coding Agents

A change is not competition-useful unless it preserves or improves the following:

- Simulator connection succeeds.
- Camera frames decode consistently.
- The control loop sends commands at a spec-compliant rate.
- Autonomous operation does not require keyboard, mouse, or human intervention during a submitted run.
- The drone can recover or fail safely when a gate is not detected.
- Relevant telemetry and perception outputs are inspectable through logs or debug tools.

## Simulator Reality vs Spec (empirically established, 2026-06-09)

Everything in this section was reverse-engineered by flying probe scripts
against the actual DCL simulator, because the sim does NOT implement the
MAVLink control messages the way the spec implies. The probe scripts live in
`src/mavlink_client/*_probe.py` (plus `command_test.py`, `position_test.py`,
`thrust_test.py`) and the model fitting in `fit_tilt_model.py`. Do not "fix"
the sign conventions in `src/control/control.py` back to textbook NED without
re-running those probes — every non-obvious sign in there is load-bearing.

### Control interface

- `SET_POSITION_TARGET_LOCAL_NED` (any typemask, velocity or position, BODY or
  LOCAL frame) tilts the body but produces ZERO thrust. The drone never leaves
  the ground. It cannot be used to fly.
- `MAV_CMD_NAV_TAKEOFF` is ACK'd but does nothing.
- `SET_ATTITUDE_TARGET` is the only interface that flies, but the sim decodes
  the quaternion to euler angles and treats them as BODY-RATE commands, not an
  attitude target:
  - `d(roll_reported)/dt  ~= -2.5 * q_roll`
  - `d(pitch_reported)/dt ~= -2.4 * q_pitch`
  - `yawspeed             ~= +2.28 * q_yaw`
  - the thrust field works normally (0..1)
- Sending a constant "level attitude" quaternion therefore spins/tilts the
  drone instead of leveling it. Setting all rate-ignore typemask bits plus
  ATTITUDE_IGNORE removes all stabilization and the drone flips.
- Client code must close every loop itself: velocity -> desired tilt ->
  tilt-rate -> quaternion encoding (see the cascade in `src/control/control.py`).

### Telemetry frames (mixed conventions — trust nothing by default)

- `ATTITUDE.yaw` is TRUE NED yaw (verified against course over ground).
- `ATTITUDE.pitch` is sign-INVERTED vs NED (FLU-style export): positive
  reported pitch corresponds to body-forward acceleration.
- Tilt-to-acceleration mapping (fit across flight logs, corr 0.86):
  `a_world = g * Rz(-yaw) @ (pitch_rep, roll_rep)` — the (fwd, right) tilt
  vector lives in a LEFT-HANDED yaw frame. To realize a desired true-body
  acceleration, pre-rotate the tilt target by +2*yaw before sending.
- `LOCAL_POSITION_NED` is trustworthy true NED for both position and velocity.
- `ODOMETRY` velocity is FLU body frame (`vy` sign-flipped vs body-right).
  Never merge `LOCAL_POSITION_NED` and `ODOMETRY` velocities interchangeably —
  doing so makes the feedback sign alternate sample-to-sample.

### Flight characteristics

- Hover thrust ~= 0.25-0.27 (the adaptive hover integrator in control.py
  converges there). Liftoff from ground needs ~0.28 to break contact.
- Thrust 0 while airborne does NOT mean motors off (the drone keeps climbing
  for a while); thrust 0.9 reaches ~30 m/s climb. Keep a hard cap (~0.45).
- The drone can sink/fall through the floor when tumbling; out-of-bounds
  causes a respawn at the start gate plus DISARM (all subsequent commands are
  silently ignored — re-arm or restart).
- Arming sometimes respawns the drone at the start (reliably after
  out-of-bounds, not reliably otherwise). Reset the sim manually between runs.
- At spawn the drone faces yaw ~= pi and rests with reported pitch +0.31; the
  first gate plane is ~23.5 m ahead (x ~= -23.5, straight down y ~= 0).

### Perception behavior near gates

- The HSV gate detector's bounding box never reaches the geometric gate size;
  it peaks around 150 px during approach (partial ring coverage).
- Within ~10 m of a gate the detected center SLIDES sideways in the image (a
  bbox artifact as the ring fills the view) even when the drone flies
  perfectly straight. Chasing that slide steers the drone into the gate posts.
  Root cause: once a ring edge leaves the frame the bbox is clipped by the
  image border, so the bbox center jumps toward the still-visible side.
- The detector flags this: `GateDetection.clipped_x/clipped_y/partial` are set
  when the bbox comes within `CLIP_MARGIN_PX` (6 px) of a frame border
  (`gate_perception.py`). IMPORTANT: only `clipped_x` (horizontal) matters for
  the slide/veer. `clipped_y` fires harmlessly early because the gate is framed
  LOW (cy ~= 300 due to the 20 deg camera up-tilt), so its bbox bottom hits the
  frame edge while still several metres out and perfectly centered. An earlier
  attempt gated steering on `partial` (= clipped_x OR clipped_y); the spurious
  clipped_y blocked the commit and the drone veered. Do NOT gate centering or
  commit on clipped_y / partial.
- At point-blank range (gate mouth) detection collapses entirely ("no gate").
- Pass strategy (`src/run/center.py`):
  - `last_good` latches the last horizontally-clean (`not clipped_x`), centered
    detection including its bbox center.
  - Commit to the dash while still centered (`not clipped_x`, max(w,h) >= 100 px
    and |ex| <= 0.15 rad), OR as soon as `clipped_x` goes true (horizontal edge
    leaving) after a recent centered lock -> finish straight, never chase the
    slide. These chain into multiple commits across the final metres.
  - While `clipped_x` but not yet committing, servo on the LATCHED center, not
    the sliding one.
  - If the gate is lost right after a centered lock, blind-dash through.
  - `VisualServoController(search_yaw_rate=0.0)` makes a lost gate HOLD heading
    instead of yaw-searching, so there is no post-pass veer. Re-enable a search
    when multi-gate acquisition is wanted.

### Race status and track data (official, decoded from the sim's broadcasts)

The sim streams two feeds over `ENCAPSULATED_DATA` that the official starter
parses (`examples/mavlink_rx.py`); `src/mavlink_client/race_data.py` is our
decoder (run it on a logged `mavlink.jsonl` to verify offline):

- Race status (payload byte 0 == 1, ~4 Hz, streams continuously):
  `<BQqqIq` -> data_type, sim_boot_time_ms, race_start_boot_time_ms,
  race_finish_time_ns, active_gate_index, last_gate_race_time (ns). Values
  are -1 for "not yet". `active_gate_index` is the authoritative
  which-gate-is-next signal (the sim decides what counts as a pass);
  `race_finish_time_ns > 0` is the authoritative finish. The race clock is
  already running at spawn (starts at sim reset, not at the first gate).
- Track data (payload byte 0 == 2): the full course map. Chunked transfer
  announced by `DATA_TRANSMISSION_HANDSHAKE` (`width` = transfer id,
  `packets` = chunk count); reassembled payload is `<H` gate count then per
  gate `<Hfffffffff` = id, NED xyz, quaternion wxyz, width, height.
  CAVEATS: it is broadcast only at race (re)start — a client attaching
  mid-session never sees it (keep a fallback table; the environment is
  deterministic), and broadcasts sent around a reset can be in the PREVIOUS
  session's NED origin (sanity-check gate 0 against the known spawn-relative
  position before adopting).
- Gate `position_ned_z` is the gate's BOTTOM edge: the successful manual run
  crossed every gate at z ~= gate_z - 1.2..1.5, i.e. aim for
  `gate_z - height/2` (`Gate.center_z`).
- VQ1 course (spawn-relative NED, from the broadcast): 6 gates, all
  2.72 x 2.72 m, same orientation, running straight down -X while DESCENDING
  26 m: (-23.3,-0.4,-0.0) (-46.9,-2.5,+5.1) (-74.6,+1.2,+13.7)
  (-111.5,-5.1,+24.6) (-135.5,-0.8,+25.4) (-159.2,-4.4,+26.0). The manual
  run flew the whole course at yaw ~= pi (never yawed) at ~2.3 m/s,
  finishing in 109.6 s.

### Client requirements that are easy to miss

- The client must stream its own HEARTBEAT at >= 2 Hz
  (`MAVLinkClient.start_heartbeat()`); the starter code never sent one.
- Command rate must stay < 100 Hz; vision-driven loops are paced by the
  30 Hz camera stream (the frames generator yields `None` every 0.25 s when
  idle so the command stream never stalls); `src/run/race.py` paces itself
  at 30 Hz with sleeps (no camera needed).
- Chunked track data needs every pending MAVLink message:
  `MAVLinkClient.recv_all()` drains in arrival order; the latest-per-type
  `recv_telemetry()` drops chunks.
- The sim world can be reset remotely with COMMAND_LONG id 31000
  (`MAVLinkClient.send_sim_reset()`); it respawns the drone at the start and
  the race clock restarts.

### Code layout and known gaps

- `src/run/race.py` is the PRIMARY autonomous entry point: full-course
  waypoint corridor flight on `LOCAL_POSITION_NED`, holding yaw ~= pi and
  P-correcting cross-track y/z toward each gate's opening center
  (`Gate.center_z`), sequenced by `active_gate_index` and stopped by the
  finish signal. Live track data is adopted only when its gate 0 matches the
  fallback table (stale-frame guard); a missed plane crossing without race
  credit triggers a go-around; a position teleport (respawn) re-arms and
  restarts. No camera needed. Run it with
  `python src\run\race.py --reset --max-seconds 300` (design doc:
  `docs/superpowers/specs/2026-07-06-autonomous-race-runner-design.md`).
- `src/control/control.py` holds the flight logic (cascade controller,
  perception/control classes, tuning constants). `src/run/center.py` is the
  vision-only single-gate entry point (takeoff -> visual servo -> chained
  dash -> straight through gate -> hold heading); `src/run/two_gate.py`
  extends it to chase gate 2 by vision. Both are superseded by `race.py` for
  course completion but remain the visual-servo testbeds. Run with
  `python src\run\center.py --max-seconds 60 --log flight.csv`.
- `src/run/manual.py` is a manual flight/debug entry point using the same
  calibrated control cascade. It auto-takes off, then maps arrow keys to
  forward/back/strafe, W/S to climb/descend, and A/D to yaw. Its defaults are
  2.5 m/s horizontal, 1.2 m/s vertical, and 1.0 rad/s yaw. Every run writes a
  timestamped bundle under `logs/manual/` with every decoded camera frame, a
  camera index, every raw MAVLink message, per-control-tick state/commands, and
  event/metadata files. Run it with `python src\run\manual.py`. This is for
  debugging only; manual input is not allowed during a submitted timed flight.
  Manual commands have no altitude/ground safety override: held S remains a
  positive NED-Z (downward) velocity request at every reported altitude.
- The vision-only pipeline remains single-gate in practice (gate 2 sits far
  BELOW gate 1 and outside the up-tilted camera's view during the approach);
  multi-gate completion is `race.py`'s job. Vision's future role is optional
  fine-trim near gates, not primary navigation.
- `race.py` cruises at 2.5 m/s (the manual run's proven speed). Raise speed
  only after full-course finishes are reliable.

