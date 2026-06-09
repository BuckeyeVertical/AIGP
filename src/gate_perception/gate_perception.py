"""Gate perception for Virtual Qualifier 1.

First-attempt OpenCV gate detector. Answers, per camera frame: is a gate
visible, where is its center, how big is it, how confident are we. The pipeline
is HSV threshold -> morphology -> contours -> best candidate -> bounding box /
center -> confidence -> pixel-to-angle.

Default color band targets the red-orange gate color (~#f52b03); thresholds are
starting values meant to be retuned against the real stream (run with --tune).

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

# The course background is mostly grey (near-zero saturation), while the gate is
# always some saturated color in the purple->red range -- red-orange ~#f52b03
# (HSV ~(5,252,245)) in normal light, pale pink ~#ffc4f6 (HSV ~(155,59,255))
# when blown out. So rather than tune a tight band per color state, mask one
# broad hue band spanning purple/magenta/pink (130-179) through red/orange
# (0-15) and let the saturation floor reject the grey background.
#
# Band format is ((H_lo,S_lo,V_lo), (H_hi,S_hi,V_hi)); H_lo > H_hi marks a
# hue-wrap band (split around 0/180 in _band_mask). The S floor is the key knob:
# grey sits at S~0, while the gate -- even blown out to a near-white pink like
# #ffebfd (HSV ~(153,20,255)) -- holds a little saturation. The floor is set just
# under that (18) to catch the palest pink while still rejecting grey. Going
# lower risks grey leaking in (morphology + the shape filter absorb minor leaks).
# Retune with --tune.
GATE_BAND = ((130, 18, 50), (15, 255, 255))
GATE_BANDS = [GATE_BAND]

# A detection below this confidence is reported but flagged low_confidence.
CONFIDENCE_FLOOR = 0.35

# Inner opening as a fraction of the outer gate (1500mm inner / 2700mm outer).
# Used to sample only the colored frame ring when learning gate color.
INNER_RATIO = 1500.0 / 2700.0


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
        bands=GATE_BANDS,
        min_area=400,
        kernel_size=5,
        adaptive=False,
        learn_rate=0.2,
        backproj_thresh=60,
    ):
        # Each band kept as (lower, upper) uint8 arrays; the mask ORs them all.
        self.bands = [
            (np.array(lo, dtype=np.uint8), np.array(hi, dtype=np.uint8))
            for lo, hi in bands
        ]
        self.min_area = min_area
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
        )
        self.last_mask = None  # cleaned mask from the most recent detect()

        # Adaptive color: learn the gate's actual H-S histogram online from
        # confident detections, then OR a backprojection of it into the static
        # mask so coverage follows lighting/color drift. The static bands remain
        # the bootstrap seed and the always-on fallback for re-acquisition.
        self.adaptive = adaptive
        self.learn_rate = learn_rate  # online histogram blend weight (0..1)
        self.backproj_thresh = backproj_thresh
        self._hist = None  # learned H-S histogram (float32) or None
        self.last_backproj = None  # thresholded backprojection from last detect()
        self._H_BINS, self._S_BINS = 30, 32

    @staticmethod
    def _band_mask(hsv, lo, hi):
        if lo[0] <= hi[0]:
            return cv2.inRange(hsv, lo, hi)
        # Hue wraps past 0/180 (reds): OR a low-hue band with a high-hue band,
        # sharing the same S/V floor and ceiling.
        low_band = cv2.inRange(hsv, np.array([0, lo[1], lo[2]], np.uint8), hi)
        high_band = cv2.inRange(hsv, lo, np.array([179, hi[1], hi[2]], np.uint8))
        return cv2.bitwise_or(low_band, high_band)

    def _build_mask(self, hsv):
        mask = None
        for lo, hi in self.bands:
            band = self._band_mask(hsv, lo, hi)
            mask = band if mask is None else cv2.bitwise_or(mask, band)

        self.last_backproj = None
        if self.adaptive and self._hist is not None:
            # Smooth the histogram so each learned color also covers adjacent
            # hues/sats -- this is what lets backprojection bridge gradual color
            # drift instead of lagging a bin behind it.
            hist = cv2.GaussianBlur(self._hist, (0, 0), sigmaX=1.0, sigmaY=1.0)
            cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
            backproj = cv2.calcBackProject(
                [hsv], [0, 1], hist, [0, 180, 0, 256], scale=1
            )
            _, backproj = cv2.threshold(
                backproj, self.backproj_thresh, 255, cv2.THRESH_BINARY
            )
            self.last_backproj = backproj
            mask = cv2.bitwise_or(mask, backproj)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        return mask

    def _learn_color(self, hsv, x, y, w, h):
        """Update the learned H-S histogram from the confirmed gate region.

        Samples the gate's colored frame by its spatial ring -- the outer bbox
        minus the inner opening (1500mm of the 2700mm outer per gate geometry) --
        rather than by the current color mask. Sampling spatially is what lets
        the histogram pick up the gate's *actual* color even when it has drifted
        outside the static bands; the inner hole is excluded so background seen
        through the gate doesn't pollute it.
        """
        roi_hsv = hsv[y : y + h, x : x + w]
        ring = np.full((h, w), 255, dtype=np.uint8)
        iw, ih = int(w * INNER_RATIO), int(h * INNER_RATIO)
        ix, iy = (w - iw) // 2, (h - ih) // 2
        ring[iy : iy + ih, ix : ix + iw] = 0
        hist = cv2.calcHist(
            [roi_hsv], [0, 1], ring, [self._H_BINS, self._S_BINS], [0, 180, 0, 256]
        )
        cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
        if self._hist is None:
            self._hist = hist
        else:
            # Exponential moving average so color tracks drift but resists jumps.
            self._hist = (1 - self.learn_rate) * self._hist + self.learn_rate * hist

    def detect(self, image_bgr) -> GateDetection:
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        mask = self._build_mask(hsv)
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

        # Only learn from shape-validated, confident gates so the histogram
        # doesn't drift onto background.
        if self.adaptive and status == "ok":
            self._learn_color(hsv, x, y, w, h)

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
    """One trackbar window per band (6 sliders each) plus the combined mask."""
    names = ["H lo", "S lo", "V lo", "H hi", "S hi", "V hi"]
    maxes = [180, 255, 255, 180, 255, 255]
    band_wins = []
    for i, (lo, hi) in enumerate(detector.bands):
        win = f"band {i} (tune)"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        for name, val, mx in zip(names, list(lo) + list(hi), maxes):
            cv2.createTrackbar(name, win, int(val), mx, lambda _v: None)
        band_wins.append(win)

    mask_win = "mask (tune)"
    cv2.namedWindow(mask_win, cv2.WINDOW_NORMAL)

    def apply():
        for i, win in enumerate(band_wins):
            vals = [cv2.getTrackbarPos(n, win) for n in names]
            detector.bands[i] = (
                np.array(vals[:3], dtype=np.uint8),
                np.array(vals[3:], dtype=np.uint8),
            )
        if detector.last_mask is not None:
            cv2.imshow(mask_win, detector.last_mask)

    return apply


def main():
    parser = argparse.ArgumentParser(description="Live gate detector debug viewer.")
    parser.add_argument("--ip", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5600)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--tune", action="store_true", help="show HSV trackbars")
    parser.add_argument(
        "--adaptive",
        action="store_true",
        help="learn gate color online (histogram backprojection) to follow drift",
    )
    args = parser.parse_args()

    frames = _import_frames()
    detector = GateDetector(adaptive=args.adaptive)

    window = "gate perception"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    tuner = _make_tuner(detector) if args.tune else None
    if args.adaptive:
        cv2.namedWindow("backprojection", cv2.WINDOW_NORMAL)

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
            if args.adaptive and detector.last_backproj is not None:
                cv2.imshow("backprojection", detector.last_backproj)

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break
        if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
