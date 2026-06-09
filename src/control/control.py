"""Visual-servoing controller (first attempt).

Converts a GateDetection into a body-frame velocity + yaw-rate command that
centers the drone on the gate and flies through it. Body NED convention
(Technical_Spec.pdf): X forward, Y right, Z down.

Control law per axis (gate centered in the image -> drone centered on the gate):
  error_x, error_y = normalized image-center error (see camera.normalize_image_error)
  yaw_rate = K_YAW * error_x - D_YAW * yaw_rate_meas   (turn toward gate, damped)
  vy       = K_SIDE * error_x                          (strafe toward gate)
  vz       = K_VERT * error_y                          (+error_y => below => move down)
  vx       = speed schedule on alignment (creep when off-center, fast when centered)

Outputs are clamped to safety limits and low-pass smoothed to avoid jerks.
Camera-intrinsic math lives in camera.py, not here.

Out of scope (future work): attitude-target experiments, integral term, telemetry
wiring for the yaw-rate damping (currently passed in, defaults to 0), planner
state machine, and active gate-reacquisition search.
"""

from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass

# --- sibling src module imports (each module lives in its own dir) -------------
_SRC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _m in ("camera", "gate_perception", "vision_receiver", "mavlink_client"):
    _p = os.path.join(_SRC, _m)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from camera import normalize_image_error  # noqa: E402

# --- gains (proportional + yaw damping) ---------------------------------------
K_YAW = 1.2
D_YAW = 0.2
K_SIDE = 1.0
K_VERT = 1.0

# --- safety limits ------------------------------------------------------------
MAX_VX = 2.5
MAX_VY = 1.5
MAX_VZ = 1.5
MAX_YAW_RATE = 1.5  # rad/s

# --- speed schedule (forward speed by how centered the gate is) ---------------
V_CREEP = 0.2  # badly off-center, low confidence, or aligning
V_MID = 0.8
V_FAST = 2.0
ALIGN_CREEP = 0.7  # |ex|+|ey| above this -> creep
ALIGN_MID = 0.3  # above this -> mid speed, else fast
MIN_CONFIDENCE = 0.35  # below this, treat as a weak detection and creep

# --- behavior tuning ----------------------------------------------------------
SMOOTHING_ALPHA = 0.5  # command = alpha*new + (1-alpha)*previous
SEARCH_YAW_RATE = 0.0  # yaw-rate to apply when no gate is seen (0 = hover/hold)


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


@dataclass
class VelocityCommand:
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0


class VisualServoController:
    def __init__(
        self,
        k_yaw=K_YAW,
        d_yaw=D_YAW,
        k_side=K_SIDE,
        k_vert=K_VERT,
        alpha=SMOOTHING_ALPHA,
        search_yaw_rate=SEARCH_YAW_RATE,
    ):
        self.k_yaw = k_yaw
        self.d_yaw = d_yaw
        self.k_side = k_side
        self.k_vert = k_vert
        self.alpha = alpha
        self.search_yaw_rate = search_yaw_rate
        self.prev = VelocityCommand()

    def compute(self, detection, yaw_rate_meas=0.0) -> VelocityCommand:
        """Map a GateDetection (+ measured yaw rate) to a smoothed command."""
        if not detection.found:
            # Lost-gate: stop forward motion and hold (optionally slow-search).
            return self._smooth(VelocityCommand(0.0, 0.0, 0.0, self.search_yaw_rate))

        error_x, error_y = normalize_image_error(
            detection.center_x, detection.center_y
        )

        yaw_rate = self.k_yaw * error_x - self.d_yaw * yaw_rate_meas
        vy = self.k_side * error_x
        vz = self.k_vert * error_y

        alignment_error = abs(error_x) + abs(error_y)
        if detection.confidence < MIN_CONFIDENCE or alignment_error > ALIGN_CREEP:
            vx = V_CREEP
        elif alignment_error > ALIGN_MID:
            vx = V_MID
        else:
            vx = V_FAST

        cmd = VelocityCommand(
            clamp(vx, 0.0, MAX_VX),
            clamp(vy, -MAX_VY, MAX_VY),
            clamp(vz, -MAX_VZ, MAX_VZ),
            clamp(yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE),
        )
        return self._smooth(cmd)

    def _smooth(self, cmd) -> VelocityCommand:
        a, p = self.alpha, self.prev
        out = VelocityCommand(
            a * cmd.vx + (1 - a) * p.vx,
            a * cmd.vy + (1 - a) * p.vy,
            a * cmd.vz + (1 - a) * p.vz,
            a * cmd.yaw_rate + (1 - a) * p.yaw_rate,
        )
        self.prev = out
        return out


# --------------------------------------------------------------------------
# First-attempt end-to-end demo loop: camera -> perception -> control -> MAVLink
# --------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Visual-servo control demo loop.")
    parser.add_argument("--mav-ip", default="127.0.0.1")
    parser.add_argument("--mav-port", type=int, default=14550)
    parser.add_argument("--cam-ip", default="0.0.0.0")
    parser.add_argument("--cam-port", type=int, default=5600)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="compute and print commands without connecting/sending MAVLink",
    )
    args = parser.parse_args()

    from gate_perception import GateDetector
    from vision_receiver import frames

    detector = GateDetector()
    controller = VisualServoController()

    client = None
    if not args.dry_run:
        from mavlink_client import MAVLinkClient

        client = MAVLinkClient.connect(args.mav_ip, args.mav_port)
        print("Arming...", flush=True)
        client.arm()

    print(f"Control loop running on camera udp://{args.cam_ip}:{args.cam_port}")
    for item in frames(args.cam_ip, args.cam_port, timeout_s=2.0):
        if item is None:
            # No fresh frame: hold (treat as lost gate) so we don't fly blind.
            cmd = controller.compute(_NO_GATE)
        else:
            _frame_id, image, _sim_time_ns = item
            det = detector.detect(image)
            # TODO: feed measured yaw rate from telemetry once mavlink_rx is wired.
            cmd = controller.compute(det, yaw_rate_meas=0.0)

        if client is not None:
            client.send_body_velocity(cmd.vx, cmd.vy, cmd.vz, cmd.yaw_rate)
        else:
            print(
                f"vx={cmd.vx:+.2f} vy={cmd.vy:+.2f} vz={cmd.vz:+.2f} "
                f"yaw_rate={cmd.yaw_rate:+.2f}"
            )


class _NoGate:
    found = False


_NO_GATE = _NoGate()


if __name__ == "__main__":
    main()
