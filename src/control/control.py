"""Visual-servoing controller for the DCL simulator.

Converts a GateDetection into flight commands that slowly center the drone on
the gate and creep through it. Body NED convention (Technical_Spec.pdf):
X forward, Y right, Z down.

== How this sim is actually controlled (established by probing, see
   src/mavlink_client/*_probe.py) ==

SET_POSITION_TARGET_LOCAL_NED tilts the body but generates NO thrust; it
cannot fly the drone. SET_ATTITUDE_TARGET is the real interface, but the sim
does not track the quaternion as an attitude: it decodes euler angles from it
and uses them as BODY-RATE commands:

    roll rate  ~= -2.5 * q_roll   (rad/s per rad, sign inverted)
    pitch rate ~= -2.4 * q_pitch  (sign inverted)
    yaw rate   ~= +2.28 * q_yaw
    thrust     -> works normally (hover somewhere near ~0.12-0.18)

So this module closes every loop itself, as a cascade:

  GateDetection --visual servo--> velocity cmd (vx,vy,vz,yaw_rate)
                --velocity loop--> desired roll/pitch angle + thrust
                --attitude loop--> desired body rates
                --rate encoding--> SET_ATTITUDE_TARGET quaternion

Detections below MIN_CONFIDENCE are treated as lost so false-positive blobs
cannot steer the drone. Lost-gate behavior: hold, then slow yaw search.
"""

from __future__ import annotations

import math
import os
import sys
import time
from dataclasses import dataclass

# --- sibling src module imports (each module lives in its own dir) -------------
_SRC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _m in ("camera", "gate_perception", "vision_receiver", "mavlink_client"):
    _p = os.path.join(_SRC, _m)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from camera import gate_error_angles  # noqa: E402

# --- visual-servo gains ---------------------------------------------------------
K_YAW = 0.6  # yaw-rate cmd per unit of horizontal image error
K_SIDE = 0.4  # lateral velocity per unit of horizontal image error
K_VERT = 0.6  # vertical velocity per unit of vertical image error

# --- velocity-command safety limits ----------------------------------------------
MAX_VX = 1.0
MAX_VY = 0.8
MAX_VZ = 0.8
MAX_YAW_RATE = 0.8  # rad/s

# --- forward speed schedule (by how centered the gate is) -------------------------
V_CREEP = 0.0  # off-center: center first, no forward motion
V_MID = 0.4
V_FAST = 0.8
ALIGN_CREEP = 0.6  # |ex|+|ey| above this -> V_CREEP
ALIGN_MID = 0.25  # above this -> V_MID, else V_FAST
MIN_CONFIDENCE = 0.35  # below this, treat the detection as lost (don't servo)

# --- visual-servo behavior --------------------------------------------------------
SMOOTHING_ALPHA = 0.35  # command = alpha*new + (1-alpha)*previous
DEADBAND = 0.04  # |error| below this commands zero on that axis
LOST_HOLD_S = 1.0  # hold (zero velocity) this long after losing the gate...
SEARCH_YAW_RATE = 0.3  # ...then yaw-search at this rate to reacquire

# --- velocity loop (velocity error -> tilt angle + thrust) -------------------------
KP_TILT = 0.12  # rad of tilt per m/s of velocity error
MAX_TILT = 0.25  # rad (~14 deg)
KP_THRUST = 0.08  # thrust per m/s of vertical-velocity error
KI_HOVER = 0.05  # hover-thrust adaptation rate
HOVER_THRUST_INIT = 0.25  # converged value observed in flight testing
HOVER_THRUST_MIN, HOVER_THRUST_MAX = 0.10, 0.40
MAX_THRUST = 0.50

# Safety floor: never let the visual servo fly into the ground. If NED z gets
# above (below altitude) this, override the vertical command with a climb.
ALT_FLOOR_Z = -0.4
FLOOR_CLIMB_VZ = -0.5

# --- attitude loop (angle error -> body rate) ---------------------------------------
K_ATT = 3.0  # rad/s of body rate per rad of attitude error
MAX_BODY_RATE = 2.0  # rad/s

# --- sim rate-command encoding (measured by yaw_rate_model_probe.py) ----------------
# body rate achieved ~= RATE_SCALE_* x euler angle encoded in the quaternion
RATE_SCALE_ROLL = -2.5
RATE_SCALE_PITCH = -2.4
RATE_SCALE_YAW = 2.28
MAX_RATE_Q = 0.6  # rad, keep encoded pseudo-angles small/sane

# --- takeoff -------------------------------------------------------------------------
TAKEOFF_THRUST = 0.28  # open-loop punch to break ground contact
TAKEOFF_ALT = -1.5  # NED z to reach before handing over to visual servo
TAKEOFF_TIMEOUT_S = 5.0

# --- gate-pass commit ------------------------------------------------------------------
# Close to the gate the detection slides around (the frame fills the view), so
# chasing it steers into the gate posts. Once the gate looks this big, commit:
# stop steering and dash straight through, then reacquire.
# Real bbox sizes peak around ~150 px (the detector only captures part of the
# gate ring), and within ~2 m the detection center slides sideways (artifact)
# before collapsing. So: commit while still ALIGNED and the gate is merely
# big-ish, and cross the last meters as straight dashes instead of chasing
# the corrupted detection.
COMMIT_SIZE_PX = 100  # max(w, h) of the gate bbox
COMMIT_MAX_EX = 0.15  # only commit while well centered (rad)
DASH_S = 2.0
V_DASH = 1.2
# Blind-pass trigger: at point-blank range the gate exceeds the camera view,
# the detection center slides off sideways (bbox artifact) and then collapses,
# never reaching COMMIT_SIZE_PX. If we had a big, CENTERED lock recently and
# the detection is now gone, we are at the gate mouth -> dash through. The
# window must span the ~3 s the slide artifact lasts.
BLIND_RECENT_S = 4.0  # centered lock must be this fresh
BLIND_MIN_SIZE_PX = 120  # ...and at least this big
BLIND_MAX_EX = 0.35  # ...and roughly centered (rad)


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def _deadband(value, width=DEADBAND):
    return 0.0 if abs(value) < width else value


@dataclass
class VelocityCommand:
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0


@dataclass
class Telemetry:
    """Latest values pulled from the MAVLink stream.

    Frame findings (frame_probe.py): LOCAL_POSITION_NED velocity is true
    local-NED; ODOMETRY velocity is BODY-frame (FRD). Never mix them.
    """
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    yaw_speed: float = 0.0
    vn: float = 0.0  # m/s north (LOCAL_POSITION_NED only)
    ve: float = 0.0  # m/s east
    vd: float = 0.0  # m/s down (world climb/sink rate)
    vbx: float = 0.0  # m/s body forward (ODOMETRY only)
    vby: float = 0.0  # m/s body right
    x: float = 0.0  # m north
    y: float = 0.0  # m east
    z: float = 0.0  # m, NED down (negative = altitude)
    have_attitude: bool = False
    have_position: bool = False
    have_odometry: bool = False

    def update_from(self, latest):
        att = latest.get("ATTITUDE")
        if att is not None:
            self.roll, self.pitch, self.yaw = att.roll, att.pitch, att.yaw
            self.yaw_speed = att.yawspeed
            self.have_attitude = True
        pos = latest.get("LOCAL_POSITION_NED")
        if pos is not None:
            self.vn, self.ve, self.vd = pos.vx, pos.vy, pos.vz
            self.x, self.y, self.z = pos.x, pos.y, pos.z
            self.have_position = True
        odom = latest.get("ODOMETRY")
        if odom is not None:
            self.vbx, self.vby = odom.vx, odom.vy
            self.have_odometry = True


class VisualServoController:
    """Outer loop: gate detection -> body-frame velocity + yaw-rate command."""

    def __init__(
        self,
        k_yaw=K_YAW,
        k_side=K_SIDE,
        k_vert=K_VERT,
        alpha=SMOOTHING_ALPHA,
        search_yaw_rate=SEARCH_YAW_RATE,
        lost_hold_s=LOST_HOLD_S,
    ):
        self.k_yaw = k_yaw
        self.k_side = k_side
        self.k_vert = k_vert
        self.alpha = alpha
        self.search_yaw_rate = search_yaw_rate
        self.lost_hold_s = lost_hold_s
        self.prev = VelocityCommand()
        self._lost_since = None
        self._last_ex = 0.0  # last horizontal gate error; search toward it

    def compute(self, detection, t=None, reported_pitch=0.0) -> VelocityCommand:
        if t is None:
            t = time.monotonic()

        usable = (
            detection.found
            and getattr(detection, "confidence", 1.0) >= MIN_CONFIDENCE
        )
        if not usable:
            return self._lost(t)

        self._lost_since = None
        ex, ey = gate_error_angles(
            detection.center_x, detection.center_y, reported_pitch
        )
        self._last_ex = ex

        yaw_rate = self.k_yaw * _deadband(ex)
        vy = self.k_side * _deadband(ex)
        vz = self.k_vert * _deadband(ey)

        alignment_error = abs(ex) + abs(ey)
        if alignment_error > ALIGN_CREEP:
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

    def _lost(self, t) -> VelocityCommand:
        """Lost-gate: hold briefly (zero velocity), then yaw-search toward
        the side the gate was last seen on (so a gate flickering at the frame
        edge pulls the search toward it instead of away)."""
        if self._lost_since is None:
            self._lost_since = t
        yaw = 0.0
        if t - self._lost_since >= self.lost_hold_s:
            direction = 1.0 if self._last_ex >= 0 else -1.0
            yaw = direction * self.search_yaw_rate
        return self._smooth(VelocityCommand(0.0, 0.0, 0.0, yaw))

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


@dataclass
class RateCommand:
    """What actually goes out on the wire (via the quaternion encoding)."""
    roll_q: float = 0.0  # pseudo-euler values for send_attitude_target
    pitch_q: float = 0.0
    yaw_q: float = 0.0
    thrust: float = HOVER_THRUST_INIT
    # for logging
    roll_des: float = 0.0
    pitch_des: float = 0.0


class VelocityToRates:
    """Middle + inner loops: velocity cmd -> desired tilt -> body rates ->
    the sim's quaternion rate encoding, plus the thrust channel."""

    def __init__(self):
        self.hover_thrust = HOVER_THRUST_INIT

    def update(self, cmd: VelocityCommand, telem: Telemetry, dt) -> RateCommand:
        """Horizontal control, calibrated to this sim's mirrored tilt frame.

        Established across flight logs 1-5 (fit_tilt_model.py, corr 0.86 on
        log5, 0.72 on log4, vs ~0 for every alternative):
          - reported yaw is true NED yaw (course over ground matches it);
          - world acceleration follows the reported tilt as
                a_world = g * Rz(-yaw) @ (pitch_rep, roll_rep)
            i.e. (pitch_rep, roll_rep) is a (fwd, right) tilt vector in a
            LEFT-HANDED yaw frame. To realize a desired true-body-frame
            acceleration the tilt target must be pre-rotated by +2*yaw;
          - ODOMETRY body velocity has vy sign-flipped (FLU), so body
            velocity is derived from LOCAL_POSITION_NED (true NED) + yaw;
          - roll_q/pitch_q command the RATES of the reported tilts with the
            measured scales; that inner loop converges at any heading.
        """
        # True body-frame velocity from NED velocity + yaw.
        cy, sy = math.cos(telem.yaw), math.sin(telem.yaw)
        vbx = cy * telem.vn + sy * telem.ve
        vby = -sy * telem.vn + cy * telem.ve

        # Velocity loop -> desired true-body tilt, then rotate by 2*yaw into
        # the sim's mirrored tilt frame.
        tilt_fwd = KP_TILT * (cmd.vx - vbx)
        tilt_right = KP_TILT * (cmd.vy - vby)
        c2, s2 = math.cos(2 * telem.yaw), math.sin(2 * telem.yaw)
        pitch_des = clamp(c2 * tilt_fwd - s2 * tilt_right, -MAX_TILT, MAX_TILT)
        roll_des = clamp(s2 * tilt_fwd + c2 * tilt_right, -MAX_TILT, MAX_TILT)

        # Tilt loop -> world tilt rates.
        roll_rate = clamp(
            K_ATT * (roll_des - telem.roll), -MAX_BODY_RATE, MAX_BODY_RATE
        )
        pitch_rate = clamp(
            K_ATT * (pitch_des - telem.pitch), -MAX_BODY_RATE, MAX_BODY_RATE
        )

        # Thrust loop: vz error (+down); wanting to climb raises thrust.
        ez = cmd.vz - telem.vd
        if 0.0 < dt < 0.5:
            self.hover_thrust = clamp(
                self.hover_thrust - KI_HOVER * ez * dt,
                HOVER_THRUST_MIN,
                HOVER_THRUST_MAX,
            )
        thrust = clamp(self.hover_thrust - KP_THRUST * ez, 0.0, MAX_THRUST)

        # Encode rates into the sim's quaternion convention.
        return RateCommand(
            roll_q=clamp(roll_rate / RATE_SCALE_ROLL, -MAX_RATE_Q, MAX_RATE_Q),
            pitch_q=clamp(pitch_rate / RATE_SCALE_PITCH, -MAX_RATE_Q, MAX_RATE_Q),
            yaw_q=clamp(cmd.yaw_rate / RATE_SCALE_YAW, -MAX_RATE_Q, MAX_RATE_Q),
            thrust=thrust,
            roll_des=roll_des,
            pitch_des=pitch_des,
        )


# --------------------------------------------------------------------------
# End-to-end loop: camera -> perception -> cascade control -> MAVLink
# --------------------------------------------------------------------------

class _NoGate:
    found = False
    confidence = 0.0


_NO_GATE = _NoGate()
