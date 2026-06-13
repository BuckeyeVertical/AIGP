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

# Camera is pitched up 20 deg relative to the body frame. A distant point that
# is straight ahead of the *body* (level flight direction) therefore projects
# BELOW the image center, at the aim point computed in body_forward_pixel().
CAMERA_TILT_UP_DEG = 20.0


def body_forward_pixel():
    """Pixel where a distant point along level body-forward projects.

    Because the camera is tilted up by CAMERA_TILT_UP_DEG, the body's level
    flight direction maps below the principal point: y = CY + FY*tan(tilt).
    Servoing the gate onto this pixel (not the image center) makes the drone
    fly level through the gate instead of approaching it from underneath.
    """
    return CX, CY + FY * math.tan(math.radians(CAMERA_TILT_UP_DEG))


def gate_aim_error(center_x, center_y):
    """Image error of the gate relative to the body-forward aim point,
    normalized by the half-resolution like normalize_image_error.

    +error_x: gate is right of where the body is pointing.
    +error_y: gate is below where the body is flying (NED: descend to fix).
    """
    aim_x, aim_y = body_forward_pixel()
    return (center_x - aim_x) / CX, (center_y - aim_y) / CY


def gate_error_angles(center_x, center_y, reported_pitch=0.0):
    """Gate direction error in radians relative to LEVEL body-forward,
    compensated for camera tilt and current body pitch.

    The DCL sim reports ATTITUDE.pitch sign-inverted (FLU-style), so the true
    nose-up pitch is -reported_pitch; gate elevation above the horizon is
    (tilt + true_pitch - angle_below_axis). Returned ey is positive when the
    gate is BELOW level-forward (NED: descend, +vz, to fix); ex is positive
    when the gate is right of the camera axis.
    """
    angle_x = math.atan((center_x - CX) / FX)
    angle_y = math.atan((center_y - CY) / FY)  # + below camera axis
    tilt = math.radians(CAMERA_TILT_UP_DEG)
    ey = angle_y - tilt + reported_pitch  # == angle_y - tilt - true_pitch
    return angle_x, ey


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
