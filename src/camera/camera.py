"""Camera intrinsics and image-geometry helpers.

Single home for the FPV camera model from Technical_Spec.pdf so that perception
and control don't each hard-code pixel math. The control module imports the
normalization helper from here -- camera-intrinsic calculations should not live
in control.py.

Camera model (Technical_Spec.pdf):
  Resolution        : 640 x 360, pinhole, no lens distortion
  Principal point   : cx, cy = 320, 180
  Focal lengths     : fx, fy = 320, 320
  Vertical FoV      : listed as 90 deg (conflicts with fy=320, which implies
                      ~58.7 deg; angle helpers below trust fx/fy)
  Mounting          : camera origin == body origin, tilted UP 20 deg from body
"""

from __future__ import annotations

import math

WIDTH, HEIGHT = 640, 360
CX, CY = 320.0, 180.0
FX, FY = 320.0, 320.0

# Camera is pitched up 20 deg relative to the body frame. Not applied in the
# first-attempt normalization below (visual centering is the goal); kept here so
# control/state code can compensate later when mapping image error to body-frame
# flight direction.
CAMERA_TILT_UP_DEG = 20.0


def normalize_image_error(center_x, center_y):
    """Image-center error normalized to roughly [-1, 1] on each axis.

    +error_x: gate is right of image center.
    +error_y: gate is below image center.
    Divides by the principal point (== half-resolution here), matching the
    controller's expected error scale.
    """
    error_x = (center_x - CX) / CX
    error_y = (center_y - CY) / CY
    return error_x, error_y


def pixel_to_angle(px, py):
    """Pixel -> camera-frame yaw/pitch angles (radians) via the pinhole model.

    +angle_x is right of center, +angle_y is below center.
    """
    return math.atan((px - CX) / FX), math.atan((py - CY) / FY)
