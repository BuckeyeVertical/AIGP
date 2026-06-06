# Agents.md

This repository is a starter client for the AI Grand Prix Virtual Qualifier. Coding agents should treat the task as building an autonomous drone racer, not a generic simulator demo.

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
- Direct use of global position or track-map knowledge may be contrary to the intended VQ1 approach. Prefer camera-driven gate detection and telemetry-driven stabilization unless the official competition package explicitly allows a data source.

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

