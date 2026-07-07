"""Two-gate runner: pass gate 1, then reacquire and pass gate 2.

Same single-gate policy as ``center.py`` (takeoff -> visual servo -> commit ->
dash through), extended to chain a second gate:

  * While approaching gate 1 the detector usually also sees gate 2 as the
    SECOND-largest gate (it is nearer than gates 3+, so it images bigger than
    them). We continuously record gate 2's world bearing (drone yaw + the gate's
    image angle) so we know roughly which way to turn for it.
  * After dashing through gate 1, the gate-1 blob is behind us and gate 2 is now
    the largest gate ahead. We yaw toward the remembered gate-2 bearing while
    creeping forward until a solid gate comes into view, then servo to it with
    the same policy and dash through.
  * If gate 2 was never seen during the gate-1 approach, fall back to a slow
    yaw spin-search to find it.

All flight logic/constants live in ``src/control/control.py``; this only wires
up the pipeline and sequences the phases. Run with:

    python src\\run\\two_gate.py --max-seconds 90 --log flight.csv
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import os
import sys
import time

# This file lives in src/run; control.py lives in src/control and configures
# sys.path for the sibling src modules at import time, so make it importable
# first (go up to src/, then into control/).
_SRC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_SRC, "control"))

from control import (  # noqa: E402
    ALT_FLOOR_Z,
    BLIND_MAX_EX,
    BLIND_MIN_SIZE_PX,
    BLIND_RECENT_S,
    COMMIT_MAX_EX,
    COMMIT_SIZE_PX,
    DASH_S,
    FLOOR_CLIMB_VZ,
    MAX_VY,
    MAX_VZ,
    MIN_CONFIDENCE,
    TAKEOFF_ALT,
    TAKEOFF_THRUST,
    TAKEOFF_TIMEOUT_S,
    V_DASH,
    Telemetry,
    VelocityCommand,
    VelocityToRates,
    VisualServoController,
    clamp,
    gate_error_angles,
)
from control import _NO_GATE  # noqa: E402

# --- gate-2 bearing memory -----------------------------------------------------
# Record gate 2's bearing from a candidate that is a plausible SEPARATE gate, not
# a fragment of gate 1 (which is large and sits right next to / on top of gate 1,
# so it would otherwise be picked as the 2nd-largest blob and give a bearing of
# "straight ahead"). Require: confident enough, a minimum size, and its center
# far enough (in px) from the primary gate's center to be a distinct gate.
GATE2_MIN_CONF = MIN_CONFIDENCE
GATE2_CAPTURE_MIN_PX = 18
GATE2_SEPARATION_PX = 70


def _pick_gate2(candidates, primary):
    """Largest candidate that looks like a distinct second gate (separated from
    the primary), or None. Rejects gate-1 fragments by the separation test."""
    if not primary.found:
        return None
    for c in candidates:
        if c is primary or not c.found:
            continue
        if c.confidence < GATE2_MIN_CONF:
            continue
        if max(c.width_px, c.height_px) < GATE2_CAPTURE_MIN_PX:
            continue
        sep = math.hypot(c.center_x - primary.center_x, c.center_y - primary.center_y)
        if sep < GATE2_SEPARATION_PX:
            continue
        return c  # candidates are largest-first, so this is the biggest valid one
    return None

# --- reacquire phase: gate 2 sits UNDER gate 1 ---------------------------------
# Gate 2 is below gate 1, so it is never visible ahead during the gate-1 approach
# (the camera is tilted UP 20 deg). Strategy after crossing gate 1:
#   1. advance a little so we are clearly past the gate-1 plane,
#   2. descend in place until gate 2 is detected,
#   3. hand off to a strafe-centering servo (center left/right + down), then dash.
REACQ_ADVANCE_VX = 0.6  # forward speed for the short "get past gate 1" move
REACQ_ADVANCE_S = 1.5  # how long to advance before descending
REACQ_DESCEND_VZ = 1.0  # +Z is DOWN (NED): descend toward gate 2's level
REACQ_MIN_PX = 40  # a gate this big (max(w,h)) starts the descend-through

# --- gate 2 (below gate 1): center by translation, no yaw, then go through -----
# Gate 2 starts far below, so the camera (tilted up 20 deg) sees it near the
# bottom of the frame. We CENTER it by pure translation -- strafe for left/right,
# descend/climb for up/down (no yaw, so the drone never veers) -- and only ease
# forward once it is centered, then dash through when it fills the frame.
G2_TARGET_CY = 290  # aim the gate center at this row (~horizontal; drone level
# with the gate so it passes through the middle, not above it)
G2_SIDE_K = 0.006  # m/s of strafe per px of horizontal center error
G2_DESCEND_VZ = 1.0  # descend rate while the gate is still low in the frame
G2_CLIMB_VZ = 0.4  # climb rate if the gate ends up above the target row
G2_FWD_VX = 1.0  # forward speed, only once the gate is centered & fully framed
G2_ALIGN_PX = 55  # |cx-320| within this = horizontally centered
G2_ALIGN_CY = 30  # |cy-target| within this = vertically centered
G2_BOTTOM_CLIP_Y = 352  # gate bbox bottom at/below this row = still clipped low
# Keep flying forward until the gate fills the frame (mouth) or is lost at
# point-blank -- exactly like the gate-1 pass. Don't dash from far: at 150 px the
# gate is still ~5 m away and a short dash falls short.
G2_CLOSE_PX = 150  # gate got this big -> losing it now means we are at its mouth
G2_MOUTH_PX = 380  # gate fills the frame this much -> dash through
G2_DASH_S = 2.5  # through-dash duration (a bit longer to fully clear gate 2)
# Floor while hunting/flying gate 2. Gate 2 sits at the bottom of a shaft that
# drops well BELOW the arming origin, so this must allow descending past z=0
# (positive NED-z = below origin). Generous so "descend until detected" isn't
# cut short; detection switches to servo long before this depth. Raise/lower as
# the gate-2 depth is learned.
ALT_FLOOR_REACQ = 5.0

# A gate this big (max(w,h)) fills enough of the frame that we are at its mouth:
# commit the crossing straight away. This is the robust pass trigger -- it does
# not depend on a still-fresh centered lock, so it never chases the bbox slide.
PASS_SIZE_PX = 280


def _wrap(a):
    """Wrap an angle to [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    parser = argparse.ArgumentParser(description="Two-gate visual-servo loop.")
    parser.add_argument("--mav-ip", default="127.0.0.1")
    parser.add_argument("--mav-port", type=int, default=14550)
    parser.add_argument("--cam-ip", default="0.0.0.0")
    parser.add_argument("--cam-port", type=int, default=5600)
    parser.add_argument(
        "--max-seconds",
        type=float,
        default=0.0,
        help="stop the loop after this many seconds (0 = run forever)",
    )
    parser.add_argument(
        "--log",
        default="",
        help="write a per-tick CSV debug log to this path",
    )
    args = parser.parse_args()

    import cv2

    from gate_perception import GateDetector
    from vision_receiver import frames

    frame_dump_dir = os.path.join(_SRC, "..", "logs", "reacq_frames")
    os.makedirs(frame_dump_dir, exist_ok=True)
    frames_saved = 0
    frame_tick = 0

    # min_area lowered vs the default 400: gate 2 is first seen far away (small),
    # so we trade a few more false blobs (rejected by aspect/confidence) for the
    # range to spot it during the post-gate-1 search.
    detector = GateDetector(min_area=200)
    # search_yaw_rate=0: the servo itself just holds when it loses the gate; the
    # multi-gate reacquire below does the searching deliberately.
    servo = VisualServoController(search_yaw_rate=0.0)
    inner = VelocityToRates()
    telem = Telemetry()

    from mavlink_client import MAVLinkClient

    client = MAVLinkClient.connect(args.mav_ip, args.mav_port)
    client.start_heartbeat()  # spec: client must maintain >=2 Hz heartbeat
    print("Arming...", flush=True)
    client.arm()

    log = None
    if args.log:
        log = open(args.log, "w")
        log.write(
            "t,phase,target_gate,ncand,found,cx,cy,wpx,hpx,conf,"
            "g2found,g2cx,g2w,g2h,g2sep,g2bearing,yaw,"
            "cmd_vx,cmd_vy,cmd_vz,cmd_yawrate,thrust,x,y,z\n"
        )

    print(f"Two-gate loop running on camera udp://{args.cam_ip}:{args.cam_port}")
    start_t = time.monotonic()
    last_t = start_t
    next_status = 0.0

    candidates = [_NO_GATE]
    phase = "takeoff"
    target_gate = 1
    dash_until = 0.0
    # Most dashes just chain across the final metres (gate still ahead); only a
    # dash entered because the gate was LOST at point-blank actually crosses it.
    dash_is_pass = False
    reacq_sub = "advance"  # sub-phase of reacquire: "advance" then "descend"
    advance_until = 0.0
    g2_close = False  # gate 2 reached pass size -> losing it now means passed
    gate2_bearing = None  # world yaw (rad) toward gate 2, captured before gate 1
    gate2_sighting = None  # the GateDetection we last identified as gate 2
    last_good = None  # (t, size, ex, cx, cy) latched centered detection

    for item in frames(args.cam_ip, args.cam_port, timeout_s=2.0):
        now = time.monotonic()
        if args.max_seconds > 0 and now - start_t > args.max_seconds:
            print("max-seconds reached, stopping.")
            break
        dt = now - last_t
        last_t = now

        telem.update_from(
            client.recv_telemetry(["ATTITUDE", "LOCAL_POSITION_NED", "ODOMETRY"])
        )

        if item is not None:
            _frame_id, image, _sim_time_ns = item
            candidates = detector.detect_all(image, max_candidates=3)
            # Debug: save what the camera sees while hunting/flying gate 2 (every
            # 4th frame so the capture spans the whole descend + servo).
            g2_phase = phase in ("reacquire", "g2") or (
                phase == "servo" and target_gate == 2
            )
            if g2_phase and frames_saved < 320 and frame_tick % 4 == 0:
                cv2.imwrite(
                    os.path.join(frame_dump_dir, f"g2_{frames_saved:03d}_z{telem.z:+.2f}.jpg"),
                    image,
                )
                frames_saved += 1
            frame_tick += 1

        det = candidates[0]  # largest gate = the one we are heading for
        cmd = None
        rc = None

        # --- takeoff: punch off the ground, level, until at altitude ---------
        if phase == "takeoff":
            if telem.z <= TAKEOFF_ALT or now - start_t > TAKEOFF_TIMEOUT_S:
                print(f"takeoff done (z={telem.z:+.2f}), switching to servo")
                phase = "servo"
            else:
                cmd = VelocityCommand(0.0, 0.0, -0.8, 0.0)
                rc = inner.update(cmd, telem, dt)
                rc.thrust = max(rc.thrust, TAKEOFF_THRUST)
                client.send_attitude_target(
                    rc.roll_q, rc.pitch_q, rc.yaw_q, 0.0, rc.thrust
                )

        # --- gate-pass dash: fly straight through, ignore the detection -------
        if phase == "dash":
            if now >= dash_until:
                if not dash_is_pass:
                    # Chained dash: gate is still ahead, keep servoing on it.
                    phase = "servo"
                elif target_gate == 1:
                    target_gate = 2
                    last_good = None
                    reacq_sub = "advance"
                    advance_until = now + REACQ_ADVANCE_S
                    phase = "reacquire"
                    print("through gate 1; advancing then descending to find gate 2")
                else:
                    phase = "done"
                    print("through gate 2; holding")
            else:
                cmd = VelocityCommand(V_DASH, 0.0, 0.0, 0.0)

        # --- reacquire gate 2 (below gate 1): advance, descend, then servo ----
        if phase == "reacquire":
            good = det.found and det.confidence >= MIN_CONFIDENCE
            size = max(det.width_px, det.height_px) if good else 0
            if good and size >= REACQ_MIN_PX:
                print(
                    f"gate 2 acquired (w={det.width_px} h={det.height_px}px, "
                    f"conf={det.confidence:.2f}), descending through"
                )
                phase = "g2"
            elif reacq_sub == "advance" and now < advance_until:
                # Get clearly past the gate-1 plane before dropping.
                cmd = VelocityCommand(REACQ_ADVANCE_VX, 0.0, 0.0, 0.0)
            else:
                # Descend in place until gate 2 (below us) comes into view.
                reacq_sub = "descend"
                cmd = VelocityCommand(0.0, 0.0, REACQ_DESCEND_VZ, 0.0)

        # --- gate 2 (below gate 1): translate to center, then dash through ----
        if phase == "g2":
            good = det.found and det.confidence >= MIN_CONFIDENCE
            if good:
                ex_px = det.center_x - 320.0  # +ve: gate right of image center
                ey_px = det.center_y - G2_TARGET_CY  # +ve: gate below target row
                size = max(det.width_px, det.height_px)
                # The gate's bottom is off-frame if its bbox bottom hugs row 360;
                # that means we are still too HIGH -> keep descending to lift it.
                bottom = det.center_y + det.height_px / 2.0
                bottom_clipped = bottom >= G2_BOTTOM_CLIP_Y
                if size >= G2_CLOSE_PX:
                    g2_close = True
                centered = (
                    abs(ex_px) <= G2_ALIGN_PX
                    and abs(ey_px) <= G2_ALIGN_CY
                    and not bottom_clipped
                )
                # Commit when close + horizontally lined up: at the mouth the gate
                # fills the frame so the vertical row is unreliable; chasing it is
                # what made the drone bounce in place instead of going through.
                if size >= G2_MOUTH_PX and abs(ex_px) <= G2_ALIGN_PX:
                    print(
                        f"gate 2 close & lined up (w={det.width_px} h={det.height_px}px), "
                        f"dashing THROUGH"
                    )
                    phase = "dash"
                    dash_until = now + DASH_S
                    dash_is_pass = True
                    cmd = VelocityCommand(V_DASH, 0.0, 0.0, 0.0)
                else:
                    # Strafe for left/right; descend while the gate is low or its
                    # bottom is clipped (we are above it); forward only when the
                    # whole gate is framed and centered.
                    vy = clamp(G2_SIDE_K * ex_px, -MAX_VY, MAX_VY)
                    if bottom_clipped or ey_px > G2_ALIGN_CY:
                        vz = G2_DESCEND_VZ  # descend lifts the gate up the frame
                    elif ey_px < -G2_ALIGN_CY:
                        vz = -G2_CLIMB_VZ
                    else:
                        vz = 0.0
                    vx = G2_FWD_VX if centered else 0.0
                    cmd = VelocityCommand(vx, vy, clamp(vz, -MAX_VZ, MAX_VZ), 0.0)
            elif g2_close:
                # was right at the gate and it dropped from view -> passed
                phase = "done"
                print("through gate 2; holding")
            else:
                # lost it before getting close: keep descending to re-find
                cmd = VelocityCommand(0.0, 0.0, G2_DESCEND_VZ, 0.0)

        # --- visual servo (gate 1, or gate 2 after reacquire) ----------------
        if phase == "servo":
            # While heading to gate 1, remember gate 2's bearing from a distinct
            # second gate (separated from gate 1) so we know which way to turn.
            if target_gate == 1:
                g2 = _pick_gate2(candidates, det)
                if g2 is not None:
                    gate2_bearing = _wrap(telem.yaw + g2.angle_x)
                    gate2_sighting = g2

            good = det.found and det.confidence >= MIN_CONFIDENCE
            ex = 0.0
            size = 0
            if good:
                ex, _ = gate_error_angles(det.center_x, det.center_y, telem.pitch)
                size = max(det.width_px, det.height_px)
            h_trust = good and not det.clipped_x  # horizontal center reliable
            centered = abs(ex) <= COMMIT_MAX_EX
            if h_trust and size >= BLIND_MIN_SIZE_PX and abs(ex) <= BLIND_MAX_EX:
                last_good = (now, size, ex, det.center_x, det.center_y)

            have_lock = (
                last_good is not None and now - last_good[0] <= BLIND_RECENT_S
            )
            centered_lock = have_lock and abs(last_good[2]) <= COMMIT_MAX_EX
            blind_pass = not good and have_lock

            if good and size >= PASS_SIZE_PX:
                # At the gate mouth: cross it now. Robust pass trigger that does
                # not wait for the gate to be lost, so it never chases the slide.
                print(
                    f"gate {target_gate} at mouth (w={det.width_px} h={det.height_px}px), "
                    f"dashing THROUGH"
                )
                phase = "dash"
                dash_until = now + DASH_S
                dash_is_pass = True
                last_good = None
                cmd = VelocityCommand(V_DASH, 0.0, 0.0, 0.0)
            elif h_trust and size >= COMMIT_SIZE_PX and centered:
                print(
                    f"gate {target_gate} close (w={det.width_px} h={det.height_px}px, "
                    f"ex={ex:+.2f}), committing to dash"
                )
                phase = "dash"
                dash_until = now + DASH_S
                dash_is_pass = False  # chained dash, gate still ahead
                cmd = VelocityCommand(V_DASH, 0.0, 0.0, 0.0)
            elif good and det.clipped_x and centered_lock:
                print(
                    f"gate {target_gate} leaving frame (last centered "
                    f"ex={last_good[2]:+.2f}), committing to dash"
                )
                phase = "dash"
                dash_until = now + DASH_S
                dash_is_pass = False  # chained dash, gate still ahead
                cmd = VelocityCommand(V_DASH, 0.0, 0.0, 0.0)
            elif blind_pass:
                print(
                    f"gate {target_gate} lost at point-blank (last size "
                    f"{last_good[1]}px), blind dash THROUGH"
                )
                phase = "dash"
                dash_until = now + DASH_S
                dash_is_pass = True  # gate was big then vanished -> real crossing
                last_good = None
                cmd = VelocityCommand(V_DASH, 0.0, 0.0, 0.0)
            else:
                servo_det = det
                if good and det.clipped_x and have_lock:
                    servo_det = dataclasses.replace(
                        det, center_x=last_good[3], center_y=last_good[4]
                    )
                cmd = servo.compute(servo_det, reported_pitch=telem.pitch)

        # --- done: both gates passed, hold heading ---------------------------
        if phase == "done":
            cmd = VelocityCommand(0.0, 0.0, 0.0, 0.0)

        if phase != "takeoff" and cmd is not None:
            # Finding gate 2 needs to descend low; allow a lower floor while
            # reacquiring/servoing gate 2 than the default gate-1 cruise floor.
            floor_z = ALT_FLOOR_REACQ if target_gate == 2 else ALT_FLOOR_Z
            if telem.have_position and telem.z > floor_z:
                cmd.vz = min(cmd.vz, FLOOR_CLIMB_VZ)
            rc = inner.update(cmd, telem, dt)
            client.send_attitude_target(
                rc.roll_q, rc.pitch_q, rc.yaw_q, 0.0, rc.thrust
            )

        if log is not None and rc is not None:
            found = det.found
            # gate-2 candidate visible THIS frame (for offline analysis)
            g2_now = _pick_gate2(candidates, det) if found else None
            g2sep = (
                math.hypot(g2_now.center_x - det.center_x, g2_now.center_y - det.center_y)
                if g2_now is not None
                else 0.0
            )
            ncand = sum(1 for c in candidates if c.found)
            log.write(
                f"{now - start_t:.3f},{phase},{target_gate},{ncand},{int(found)},"
                f"{det.center_x if found else 0:.0f},"
                f"{det.center_y if found else 0:.0f},"
                f"{det.width_px if found else 0},"
                f"{det.height_px if found else 0},"
                f"{det.confidence if found else 0:.2f},"
                f"{int(g2_now is not None)},"
                f"{g2_now.center_x if g2_now else 0:.0f},"
                f"{g2_now.width_px if g2_now else 0},"
                f"{g2_now.height_px if g2_now else 0},"
                f"{g2sep:.0f},"
                f"{gate2_bearing if gate2_bearing is not None else 0:.3f},"
                f"{telem.yaw:.3f},"
                f"{cmd.vx:.2f},{cmd.vy:.2f},{cmd.vz:.2f},{cmd.yaw_rate:.2f},"
                f"{rc.thrust:.3f},{telem.x:.2f},{telem.y:.2f},{telem.z:.2f}\n"
            )

        if now >= next_status:
            gate = (
                f"gate@({det.center_x:.0f},{det.center_y:.0f}) "
                f"{det.width_px}x{det.height_px} conf={det.confidence:.2f}"
                if det.found
                else "no gate"
            )
            ncand = sum(1 for c in candidates if c.found)
            bearing = (
                f"{math.degrees(gate2_bearing):+.0f}" if gate2_bearing is not None else "?"
            )
            g2_now = _pick_gate2(candidates, det)
            g2str = (
                f"G2@({g2_now.center_x:.0f}) {g2_now.width_px}x{g2_now.height_px}"
                if g2_now is not None
                else "no-G2"
            )
            print(
                f"[{phase} g{target_gate}] {gate} (cand={ncand}, {g2str}, g2brg={bearing}) | "
                f"xyz=({telem.x:+6.2f},{telem.y:+6.2f},{telem.z:+6.2f}) "
                f"yaw={telem.yaw:+.2f} thr={inner.hover_thrust:.2f}",
                flush=True,
            )
            next_status = now + 1.0

    if log is not None:
        log.close()


if __name__ == "__main__":
    main()
