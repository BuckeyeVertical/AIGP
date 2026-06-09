"""Gate perception for Virtual Qualifier 1.

First-attempt OpenCV gate detector. Answers, per camera frame: is a gate
visible, where is its center, how big is it, how confident are we. The pipeline
is HSV threshold -> morphology -> contours -> best candidate -> bounding box /
center -> confidence -> pixel-to-angle.

Default color band targets magenta gates; thresholds are starting values meant
to be retuned against the real stream (run this file with --tune).

Out of scope for this first attempt (future work): partial-gate handling,
temporal tracking/smoothing, multi-candidate scoring, inner-square center
refinement from the 2700mm/1500mm gate geometry, blur/quality detection, and
orchestrator/shared_data integration.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

import cv2
import numpy as np

# Camera intrinsics from Technical_Spec.pdf (640x360 pinhole, no distortion).
# Note: the spec also lists VFoV = 90 deg, which conflicts with fy=320
# (atan(180/320)*2 ~= 58.7 deg). This first attempt trusts fx/fy for angle math.
CX, CY = 320.0, 180.0
FX, FY = 320.0, 320.0

# Magenta HSV band (OpenCV hue is 0-180; magenta ~150, no hue wrap so one band).
# Starting values -- retune with --tune against a live frame.
LOWER_MAGENTA = (140, 80, 80)
UPPER_MAGENTA = (170, 255, 255)

# A detection below this confidence is reported but flagged low_confidence.
CONFIDENCE_FLOOR = 0.35


@dataclass
class GateDetection:
    found: bool
    center_x: float = 0.0
    center_y: float = 0.0
    width_px: int = 0
    height_px: int = 0
    angle_x: float = 0.0  # radians, + is right of image center
    angle_y: float = 0.0  # radians, + is below image center
    confidence: float = 0.0
    status: str = "no_gate"  # ok | no_gate | low_confidence


def pixel_to_angle(px, py):
    """Map a pixel to camera-frame yaw/pitch angles (radians)."""
    return math.atan((px - CX) / FX), math.atan((py - CY) / FY)


class GateDetector:
    def __init__(
        self,
        lower_hsv=LOWER_MAGENTA,
        upper_hsv=UPPER_MAGENTA,
        min_area=400,
        kernel_size=5,
    ):
        self.lower_hsv = np.array(lower_hsv, dtype=np.uint8)
        self.upper_hsv = np.array(upper_hsv, dtype=np.uint8)
        self.min_area = min_area
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
        )
        self.last_mask = None  # cleaned mask from the most recent detect()

    def _mask(self, image_bgr):
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        return mask

    def detect(self, image_bgr) -> GateDetection:
        mask = self._mask(image_bgr)
        self.last_mask = mask

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        best = None
        best_area = 0.0
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if w == 0 or h == 0:
                continue
            aspect = w / h
            if aspect < 0.5 or aspect > 2.0:  # keep roughly square gate outlines
                continue
            # First attempt: pick the largest passing contour. The final detector
            # replaces blind-largest with a scored candidate selection.
            if area > best_area:
                best_area = area
                best = (c, x, y, w, h, area)

        if best is None:
            return GateDetection(found=False, status="no_gate")

        c, x, y, w, h, area = best
        center_x = x + w / 2.0
        center_y = y + h / 2.0
        angle_x, angle_y = pixel_to_angle(center_x, center_y)
        confidence = self._confidence(mask, x, y, w, h, area)
        status = "ok" if confidence >= CONFIDENCE_FLOOR else "low_confidence"

        return GateDetection(
            found=True,
            center_x=center_x,
            center_y=center_y,
            width_px=int(w),
            height_px=int(h),
            angle_x=angle_x,
            angle_y=angle_y,
            confidence=confidence,
            status=status,
        )

    def _confidence(self, mask, x, y, w, h, area):
        """Blend of apparent size, squareness, and how much of the bbox is gate
        color. A clean, large, square magenta blob scores near 1.0."""
        frame_area = mask.shape[0] * mask.shape[1]
        # A full gate fills a meaningful chunk of the frame; saturate at ~25%.
        size_score = min((w * h) / (0.25 * frame_area), 1.0)

        aspect = w / h
        square_score = 1.0 - min(abs(aspect - 1.0), 1.0)  # 1 at square, 0 at 2:1

        roi = mask[y : y + h, x : x + w]
        fill_score = float(np.count_nonzero(roi)) / (w * h)

        confidence = 0.4 * size_score + 0.3 * square_score + 0.3 * fill_score
        return float(np.clip(confidence, 0.0, 1.0))

    def draw_overlay(self, image_bgr, det: GateDetection):
        out = image_bgr.copy()
        if not det.found:
            cv2.putText(
                out,
                "no gate",
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            return out

        x = int(det.center_x - det.width_px / 2)
        y = int(det.center_y - det.height_px / 2)
        color = (0, 255, 0) if det.status == "ok" else (0, 165, 255)
        cv2.rectangle(out, (x, y), (x + det.width_px, y + det.height_px), color, 2)
        cv2.circle(out, (int(det.center_x), int(det.center_y)), 4, color, -1)
        label = (
            f"{det.status} conf={det.confidence:.2f} "
            f"ax={math.degrees(det.angle_x):+.1f} ay={math.degrees(det.angle_y):+.1f}"
        )
        cv2.putText(
            out,
            label,
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            1,
            cv2.LINE_AA,
        )
        return out


# --------------------------------------------------------------------------
# Live debug viewer
# --------------------------------------------------------------------------

def _import_frames():
    """Import the sibling vision_receiver.frames generator regardless of cwd."""
    import os
    import sys

    src_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    receiver_dir = os.path.join(src_dir, "vision_receiver")
    if receiver_dir not in sys.path:
        sys.path.insert(0, receiver_dir)
    from vision_receiver import frames  # type: ignore

    return frames


def _make_tuner(detector):
    win = "mask (tune)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    names = ["H lo", "S lo", "V lo", "H hi", "S hi", "V hi"]
    init = list(detector.lower_hsv) + list(detector.upper_hsv)
    maxes = [180, 255, 255, 180, 255, 255]
    for name, val, mx in zip(names, init, maxes):
        cv2.createTrackbar(name, win, int(val), mx, lambda _v: None)

    def apply():
        vals = [cv2.getTrackbarPos(n, win) for n in names]
        detector.lower_hsv = np.array(vals[:3], dtype=np.uint8)
        detector.upper_hsv = np.array(vals[3:], dtype=np.uint8)
        if detector.last_mask is not None:
            cv2.imshow(win, detector.last_mask)

    return apply


def main():
    parser = argparse.ArgumentParser(description="Live gate detector debug viewer.")
    parser.add_argument("--ip", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5600)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--tune", action="store_true", help="show HSV trackbars")
    args = parser.parse_args()

    frames = _import_frames()
    detector = GateDetector()

    window = "gate perception"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    tuner = _make_tuner(detector) if args.tune else None

    print(f"Listening for camera frames on udp://{args.ip}:{args.port}")
    print("Press q or Esc to exit.")

    for item in frames(args.ip, args.port, args.timeout):
        if item is None:
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
            cv2.imshow(window, placeholder)
        else:
            _frame_id, image, _sim_time_ns = item
            det = detector.detect(image)
            cv2.imshow(window, detector.draw_overlay(image, det))
            if tuner is not None:
                tuner()

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break
        if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
