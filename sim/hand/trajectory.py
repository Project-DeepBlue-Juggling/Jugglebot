"""Hand trajectory generator — Python port of Teensy Trajectory.h.

Provides both catch and throw trajectories:

- HandCatchTrajectory / HandCatchSequence: port of makeCatch() — hand moves
  downward at CATCH_VEL_RATIO × ball_speed to reduce impact velocity.
- HandThrowTrajectory / HandThrowSequence: port of calcThrow() — hand moves
  upward to eject the ball at a desired speed.

Also provides HandSmoothMove (quintic S-curve) used by both sequences.

All public positions are in mm (matching sim/plant command_hand()).
Internally uses metres to match Trajectory.h math, converting at boundaries.

Timeline conventions:
- Catch: t=0 is the moment the ball arrives (midpoint of velocity-hold phase).
- Throw: t=0 is the moment of ball release (end of velocity-hold phase).
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Constants — from config/hardware_config.yaml  teensy_trajectory section
# ---------------------------------------------------------------------------
GRAVITY_MPS2 = 9.806
HAND_SPOOL_RADIUS_M = 0.00521
LINEAR_GAIN_FACTOR = 1.035
INERTIA_HAND_ONLY_KG = 0.281
INERTIA_RATIO = 0.747
# Total reflected inertia of the hand axis at the motor (rotor + cable-driven
# load), used ONLY to size the torque feedforward of the post-release
# DECELERATION segment of a throw.  Mirror of
# ``teensy_trajectory.throw_decel_reflected_inertia_kgm2`` →
# ``TeensyTraj::THROW_DECEL_REFLECTED_INERTIA_KGM2``.  See
# ``ros_ws/docs/hand_decel_feedforward.md`` (contract C-HAND-2) for the
# identification and for why it is deliberately 7-10 % BELOW the measured value.
THROW_DECEL_REFLECTED_INERTIA_KGM2 = 9.5e-6
# Catch velocity as a fraction of the incoming ball speed. Source of truth is
# config/hardware_config.yaml teensy_trajectory.catch_vel_ratio (0.6) →
# Teensy_code_platform/hardware_config.h — the platform hand, hardware-validated as reliable.
# (Was 0.9, a stale port value; corrected in Phase 7 — plan § Hand-catch smoothness,
# logbook 2026-07-08-mvp-phase6 Open Question 2.) At 0.6 the catch is a *designed*
# ~40% first-contact mismatch: the hand absorbs the ball over the stroke rather than
# velocity-matching at contact.
CATCH_VEL_RATIO = 0.6
CATCH_VEL_HOLD_PCT = 0.10
HAND_STROKE_M = 0.355
STROKE_MARGIN_M = 0.02
END_PROFILE_HOLD_S = 0.10
MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2 = 100.0
QUINTIC_S2_MAX = 5.7735027
QUINTIC_H_MAX = 0.19753086
QUINTIC_H2_MAX = 3.9402340
SMOOTH_MOVE_V0_DEADBAND_RPS = 6.0
SMOOTH_MOVE_EXCURSION_MARGIN_REV = 0.5
MIN_EVENT_VEL_MPS = 0.3
MAX_EVENT_VEL_MPS = 7.0
SAFETY_GAP_S = 0.02  # 20 ms safety gap (from teensy_operational.safety_gap_us)
SMOOTH_MOVE_MIN_DURATION_S = 0.05  # fmaxf(T, 0.05f) — Trajectory.h:260

# Stroke end stops, in the FIRMWARE's frame: revs from the homed physical
# bottom.  ``rev = mm/1000 * _LINEAR_GAIN`` with NO margin term — the 20 mm
# ``STROKE_MARGIN_MM`` inset below is a sim-side *placement* of the stroke inside
# the travel, not a frame offset the firmware shares (the firmware homes DOWNWARD
# into the bottom stop and sets zero 0.1 rev above it, and
# ``hand_motor_max_position_revs = 11.1`` is 351.08 mm above that zero, i.e.
# 0.76 mm below the top of the 355 mm stroke — a guard only coherent if zero is
# the physical bottom).  Carrying the inset across would put the ceiling 0.63 rev
# too high, 0.53 rev PAST the overextension guard.  See
# plans/active/hand-command-continuity.md § Phase 0 — Outcome, Confirmation 2.
HAND_MOTOR_MAX_POSITION_REVS = 11.1     # geometry.hand_motor_max_position_revs
HAND_HOME_ABS_POS_REV = -0.1            # Homing::HAND_ABS_POS_REV — the bottom stop
# The smooth-move excursion FLOOR is encoder zero (JBOp::HAND_RETRACT_REV), NOT
# HAND_HOME_ABS_POS_REV.  -0.1 rev IS the bottom stop (the axis homes downward
# into it), and it is also below the floor the host declares for this axis
# (teensy_bridge_node rejects a smooth-move target < 0; can/odrive.py clips a hand
# setpoint < 0 and warns).  Planning commanded travel down to the stop, with 0 rev
# of allowance for the position loop's measured +0.186 rev undershoot on this
# profile family, is not something a prelude may do — and it costs nothing to
# forbid, because the bound is relaxed to min(FLOOR, start, target) below, so
# every legal target stays servable and a bulge that would go under takes the
# rest-to-rest fallback.  See Trajectory.h SMOOTH_MOVE_POS_FLOOR_REV.
HAND_RETRACT_REV = 0.0                  # JBOp::HAND_RETRACT_REV — encoder zero
# Slack on the end-stop comparisons — float32/float64 rounding at the profile's
# own endpoints, not headroom.  Mirrors Trajectory.h SMOOTH_MOVE_END_STOP_EPS_REV.
SMOOTH_MOVE_END_STOP_EPS_REV = 1e-4

# Throw-specific constants
THROW_VEL_HOLD_PCT = 0.05  # 5% of effective stroke for velocity hold

# Derived
_LINEAR_GAIN = LINEAR_GAIN_FACTOR / (math.pi * HAND_SPOOL_RADIUS_M * 2.0)  # rev/m
_TOTAL_STROKE_M = HAND_STROKE_M - 2.0 * STROKE_MARGIN_M  # 0.315 m
_TOTAL_STROKE_MM = _TOTAL_STROKE_M * 1000.0  # 315 mm
STROKE_MARGIN_MM = STROKE_MARGIN_M * 1000.0  # 20 mm


def mm_to_rev(mm: float) -> float:
    """mm of cable travel -> motor revs.  No margin term — see the note above."""
    return float(mm) / 1000.0 * _LINEAR_GAIN


def rev_to_mm(rev: float) -> float:
    """Motor revs -> mm of cable travel."""
    return float(rev) / _LINEAR_GAIN * 1000.0


# ---------------------------------------------------------------------------
# Torque feedforward — the two conversions, and why there are two
# ---------------------------------------------------------------------------
# The Teensy packs a torque feedforward alongside every position/velocity
# setpoint (``Teensy_code_platform.ino`` packTrajectory -> ODrive ``input_torque``).  It
# is the ONLY term that commands braking OPEN-LOOP; everything else the drive
# does on the decel it has to earn from tracking error.
#
# ``_TORQUE_K_LEGACY`` is ``Trajectory.h``'s historical ``accelToTorque``:
# ``a_lin * INERTIA_HAND_ONLY_KG * HAND_SPOOL_RADIUS_M``.  It models the axis as
# a pure translating mass on a spool, so its IMPLIED reflected inertia is
# ``m*r/(2*pi*LINEAR_GAIN)`` = 7.3695e-6 kg m^2 — it omits the rotor entirely and
# uses the raw spool radius rather than the effective one the 1.035 gain factor
# implies.  Against a measured 1.02e-5 - 1.05e-5 that is ~70 % of the torque the
# commanded acceleration physically needs.
#
# ``_TORQUE_K_THROW_DECEL`` is the corrected conversion, applied ONLY to the
# post-release decel segment of a throw.  Everything else — the accel and
# velocity-hold segments, the whole kind-1 catch, and every ``makeSmoothMove``
# prelude — keeps the legacy conversion, deliberately: correcting the ASCENT
# feedforward would raise the achieved release velocity and re-calibrate every
# throw height the machine has flown.  See ros_ws/docs/hand_decel_feedforward.md.

#: N.m per (m/s^2) of hand-axis linear acceleration — historical conversion.
_TORQUE_K_LEGACY = INERTIA_HAND_ONLY_KG * HAND_SPOOL_RADIUS_M
#: Reflected inertia the legacy conversion implies (kg m^2).  7.3695e-6.
#: ``torque = J * alpha_rad = J * 2*pi * a_lin * LINEAR_GAIN``, so equating that
#: to ``a_lin * m * r`` gives ``J = m*r / (2*pi*LINEAR_GAIN)``.
LEGACY_IMPLIED_INERTIA_KGM2 = _TORQUE_K_LEGACY / (2.0 * math.pi * _LINEAR_GAIN)
#: N.m per (m/s^2) — post-release throw deceleration only.
_TORQUE_K_THROW_DECEL = (THROW_DECEL_REFLECTED_INERTIA_KGM2
                         * 2.0 * math.pi * _LINEAR_GAIN)


def accel_to_torque_nm(a_mps2: float) -> float:
    """``Trajectory.h::accelToTorque`` — the historical conversion."""
    return float(a_mps2) * _TORQUE_K_LEGACY


def throw_decel_to_torque_nm(a_mps2: float) -> float:
    """``Trajectory.h::throwDecelToTorque`` — post-release decel segment only."""
    return float(a_mps2) * _TORQUE_K_THROW_DECEL


# ---------------------------------------------------------------------------
# HandCatchTrajectory — port of calcCatch() + buildCatch()
# ---------------------------------------------------------------------------

class HandCatchTrajectory:
    """3-segment catch trajectory matching Teensy buildCatch().

    The trajectory moves the hand downward from ``start_pos_mm`` over the
    effective stroke (315 mm).  The hand velocity during the hold phase is
    ``-CATCH_VEL_RATIO * event_vel_mps`` (negative = downward).

    Timeline: t=0 is the midpoint of the velocity-hold phase — the instant
    the ball is expected to arrive.

    Parameters
    ----------
    event_vel_mps : float
        Ball speed at landing (m/s).  Clamped to [0.3, 7.0].
    start_pos_mm : float
        Hand position (mm from physical bottom) at the start of the catch
        trajectory.  Normally the top of effective stroke = STROKE_MARGIN_MM
        + TOTAL_STROKE_MM ≈ 335 mm.
    """

    def __init__(self, event_vel_mps: float, start_pos_mm: float | None = None):
        v_throw = max(MIN_EVENT_VEL_MPS, min(MAX_EVENT_VEL_MPS, event_vel_mps))

        # Default start = top of effective stroke
        if start_pos_mm is None:
            start_pos_mm = (STROKE_MARGIN_M + _TOTAL_STROKE_M) * 1000.0

        self._start_pos_mm = start_pos_mm

        # --- port of calcCatch() (Trajectory.h lines 117-139) ---
        vC = -CATCH_VEL_RATIO * v_throw           # m/s, negative (downward)
        irC = 1.0 / INERTIA_RATIO

        vel_hold_m = CATCH_VEL_HOLD_PCT * _TOTAL_STROKE_M     # 0.0315 m
        accel_stroke_m = _TOTAL_STROKE_M - vel_hold_m          # 0.2835 m

        # Segment durations (all positive)
        t_acc = -(2.0 / (irC + 1.0)) * accel_stroke_m / vC
        t_vel = -vel_hold_m / vC
        t_dec = t_acc * irC

        # Accelerations
        catchA = vC / t_acc     # negative (accelerate downward)
        catchD = -catchA / irC  # positive (decelerate)

        # --- Positions in "effective stroke" metres (0 = top, going negative) ---
        # Segment 1: accel from rest
        x_after_accel = 0.5 * catchA * t_acc ** 2              # negative
        # Segment 2: constant velocity hold
        x_after_hold = x_after_accel + vC * t_vel              # more negative
        # Segment 3: deceleration to rest
        # x_end = x_after_hold + vC * t_dec + 0.5 * catchD * t_dec^2

        # --- Time origin: t=0 at midpoint of velocity hold ---
        # makeCatch() shifts by -(t5-t4) where t5 is vel-hold start, t4 is
        # accel start.  In our local timeline (accel starts at local 0):
        #   vel-hold starts at t_acc
        #   midpoint of vel-hold at t_acc + 0.5*t_vel
        # So t=0 in ball-arrival coords = local (t_acc + 0.5*t_vel)
        self._t_offset = t_acc + 0.5 * t_vel  # local time of t=0

        # Store segment parameters
        self._t_acc = t_acc
        self._t_vel = t_vel
        self._t_dec = t_dec
        self._catchA = catchA
        self._catchD = catchD
        self._vC = vC

        # Segment boundary positions (metres, displacement from start, all <= 0)
        self._x1 = x_after_accel
        self._x2 = x_after_hold

        # Public timing (relative to ball arrival = t=0)
        self._t_start = -self._t_offset                               # negative
        self._t_end = (t_acc + t_vel + t_dec + END_PROFILE_HOLD_S
                       - self._t_offset)                               # positive

    @property
    def start_time(self) -> float:
        """Time before ball arrival when trajectory starts (negative)."""
        return self._t_start

    @property
    def end_time(self) -> float:
        """Time after ball arrival when trajectory ends (positive)."""
        return self._t_end

    @property
    def duration(self) -> float:
        """Total duration of the catch trajectory (seconds)."""
        return self._t_end - self._t_start

    @property
    def start_pos_mm(self) -> float:
        """Hand position at trajectory start (mm from physical bottom)."""
        return self._start_pos_mm

    @property
    def end_pos_mm(self) -> float:
        """Hand position at trajectory end (mm from physical bottom)."""
        return self._start_pos_mm + self._displacement_at_local(
            self._t_acc + self._t_vel + self._t_dec) * 1000.0

    @property
    def catch_velocity_mps(self) -> float:
        """Hand velocity during the constant-velocity hold phase (m/s, negative)."""
        return self._vC

    def sample(self, t: float) -> float:
        """Return hand position (mm) at time *t* relative to ball arrival.

        Before start_time: returns start_pos_mm.
        After end_time: returns end_pos_mm (hold at final position).
        """
        if t <= self._t_start:
            return self._start_pos_mm
        if t >= self._t_end:
            return self.end_pos_mm

        # Convert to local time (0 = accel start)
        t_local = t + self._t_offset
        disp_m = self._displacement_at_local(t_local)
        return self._start_pos_mm + disp_m * 1000.0

    def _displacement_at_local(self, t_local: float) -> float:
        """Displacement in metres from start position at local time."""
        if t_local <= 0.0:
            return 0.0

        t_acc = self._t_acc
        t_vel = self._t_vel
        t_dec = self._t_dec

        if t_local <= t_acc:
            # Segment 1: accelerate from rest
            return 0.5 * self._catchA * t_local ** 2

        if t_local <= t_acc + t_vel:
            # Segment 2: constant velocity hold
            tau = t_local - t_acc
            return self._x1 + self._vC * tau

        if t_local <= t_acc + t_vel + t_dec:
            # Segment 3: decelerate to rest
            tau = t_local - (t_acc + t_vel)
            return self._x2 + self._vC * tau + 0.5 * self._catchD * tau ** 2

        # End hold: stay at final position
        tau_end = t_dec
        return self._x2 + self._vC * tau_end + 0.5 * self._catchD * tau_end ** 2


# ---------------------------------------------------------------------------
# Smooth-move profile — rev domain.  The mirror of Trajectory.h's
# makeSmoothMove(), and the single enforcement point for the C-HAND-1
# velocity-continuity obligation.
# ---------------------------------------------------------------------------
#
# WHY THE QUINTIC IS SEEDED WITH THE LIVE VELOCITY
# ------------------------------------------------
# ``makeSmoothMove`` is prepended to EVERY hand command: a kind-3 prime/retract/
# SAFE_ABORT, and the prelude ahead of every kind-0/1/2 stroke
# (``Teensy_code_platform.ino:470`` and ``:522``).  It used to seed the quintic
# ``v = a = 0`` from ``current_hand_position`` alone, while
# ``current_hand_velocity`` sat declared ``extern volatile`` two lines above the
# function and was never read.  So any command landing while the hand moved
# commanded a VELOCITY STEP: measured 2026-07-25, a catch arm landing 8-18 ms
# after ball release froze the setpoint at the live encoder value (6.20-7.78 rev)
# with the hand travelling through it at ~120 rev/s, and the position loop then
# coasted to 10.17-10.32 rev (0.775 rev from the 11.1 rev overextension guard)
# and yanked the hand 0.34-1.75 rev = 10.7-55.3 mm BELOW the stroke end.  That is
# the operator-visible post-throw dip.
#
# THE SHAPE
# ---------
# A quintic with ``p(0)=x0, p'(0)=v0, p''(0)=0, p(T)=x1, p'(T)=p''(T)=0``
# decomposes exactly into two fixed shapes (verified symbolically and to 1e-14
# numerically, /tmp/probe_v0_quintic.py 2026-07-27):
#
#     p(tau) = x0 + delta * s(tau) + (v0 * T) * h(tau)
#     s(tau) = 10tau^3 - 15tau^4 + 6tau^5        (the historical rest-to-rest shape)
#     h(tau) = tau * (1-tau)^3 * (3tau + 1)
#
# ``h(0) = h(1) = 0``, so ``v0`` never moves the endpoints — the profile still
# starts at ``x0`` and lands on the target at rest.  ``h >= 0`` with a single
# maximum ``16/81`` at ``tau = 1/3``, so ``v0``'s entire effect is to bulge the
# profile by ``(v0*T) * 16/81`` in the direction of travel.  **That bulge is the
# overshoot**, it is correct (a hand moving away from its target must overshoot to
# come back), and it is what has to be checked against the stroke end stops.
#
# TWO CONSEQUENCES THE OLD DURATION FORMULA COULD NOT SURVIVE
# ----------------------------------------------------------
# 1. ``T = sqrt(|delta| * QUINTIC_S2_MAX / a_max)`` assumes ``v0 = 0``.  With
#    ``v0 != 0`` the acceleration is ``(delta * s'' + v0*T * h'') / T^2``, so the
#    duration must satisfy ``a_max*T^2 - |v0|*H2*T - |delta|*S2 >= 0``.  See
#    :func:`smooth_move_duration_s`.
# 2. The bulge can leave the stroke.  See :func:`plan_smooth_move`.


def quintic_s(tau: float) -> float:
    """``s(tau) = 10tau^3 - 15tau^4 + 6tau^5`` — the rest-to-rest shape."""
    t2 = tau * tau
    t3 = t2 * tau
    t4 = t2 * t2
    return 10.0 * t3 - 15.0 * t4 + 6.0 * t4 * tau


def quintic_s1(tau: float) -> float:
    """``s'(tau)``.  Peaks at 1.875 (``tau = 0.5``)."""
    t2 = tau * tau
    return 30.0 * t2 - 60.0 * t2 * tau + 30.0 * t2 * t2


def quintic_s2(tau: float) -> float:
    """``s''(tau)``.  ``max|s''| = QUINTIC_S2_MAX = 10/sqrt(3)``."""
    t2 = tau * tau
    return 60.0 * tau - 180.0 * t2 + 120.0 * t2 * tau


def quintic_h(tau: float) -> float:
    """``h(tau) = tau (1-tau)^3 (3tau + 1)`` — the start-velocity shape.

    ``h(0) = h(1) = 0`` and ``h >= 0``, with a single maximum
    ``QUINTIC_H_MAX = 16/81`` at ``tau = 1/3``.
    """
    w = 1.0 - tau
    return tau * w * w * w * (3.0 * tau + 1.0)


def quintic_h1(tau: float) -> float:
    """``h'(tau) = (1-tau)^2 (1 + 2tau - 15tau^2)``.  ``h'(0) = 1``, ``h'(1) = 0``."""
    w = 1.0 - tau
    return w * w * (1.0 + 2.0 * tau - 15.0 * tau * tau)


def quintic_h2(tau: float) -> float:
    """``h''(tau) = 12 tau (1-tau)(5tau - 3)``.  ``max|h''| = QUINTIC_H2_MAX``."""
    return 12.0 * tau * (1.0 - tau) * (5.0 * tau - 3.0)


def _quintic_coeffs(delta_rev: float, u_rev: float) -> tuple:
    """``(A, B, C)`` for ``p = x0 + u*tau + A tau^3 + B tau^4 + C tau^5``.

    ``u = v0 * T``.  At ``u = 0`` these are ``(10d, -15d, 6d)`` — the historical
    ``10tau^3 - 15tau^4 + 6tau^5``.  Used by the closed-form excursion and peak
    derivations; the SAMPLERS use the ``s``/``h`` decomposition above instead,
    which is what the firmware emits and which reduces exactly at ``u = 0``.
    """
    return (10.0 * delta_rev - 6.0 * u_rev,
            -15.0 * delta_rev + 8.0 * u_rev,
            6.0 * delta_rev - 3.0 * u_rev)


def smooth_move_accel_limited_duration_s(delta_rev: float,
                                         v0_rps: float = 0.0) -> float:
    """Shortest duration whose peak |acceleration| is bounded by the accel limit.

    ``|a(tau)| * T^2 = |delta * s''(tau) + (v0*T) * h''(tau)|
                    <= |delta| * S2 + |v0| * T * H2``  (triangle inequality)

    so requiring that bound ``<= a_max`` gives the quadratic
    ``a_max*T^2 - |v0|*H2*T - |delta|*S2 >= 0`` and its positive root.

    **At ``v0 = 0`` this is bit-identically the historical
    ``sqrt(|delta| * QUINTIC_S2_MAX / a_max)``** (verified to 9 decimals), so
    every rest-to-rest caller is unaffected.

    The bound is conservative rather than exact because the two shapes' extrema
    do not coincide — but only just: ``s''`` peaks at ``tau = 0.2113`` and
    ``h''`` at ``tau = 0.2427``, so when ``v0`` points AWAY from the target (the
    case that needs the room) it is 98.6 % tight.  Probed over 20 000 random
    ``(delta, v0)`` pairs: **0 violations** of the accel limit, peak/limit
    median 0.9963, worst-case looseness 0.218 (that corner is ``v0`` pointing
    TOWARD the target, where the terms partly cancel and a longer-than-necessary
    duration is the safe error).

    Chosen over bisecting the analytic peak: one ``sqrtf`` and six flops, no
    iteration count and no convergence failure mode, inside a CAN receive
    handler on a Teensy 4.0 — against at most a 22 % duration saving in the
    loosest corner.
    """
    c = abs(float(delta_rev)) * QUINTIC_S2_MAX
    if float(v0_rps) == 0.0:
        # The HISTORICAL expression, kept verbatim so a rest-to-rest duration
        # (and hence the emitted sample count) is unchanged to the last bit.
        return math.sqrt(c / MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2)
    b = abs(float(v0_rps)) * QUINTIC_H2_MAX
    return ((b + math.sqrt(b * b + 4.0 * MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2 * c))
            / (2.0 * MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2))


def smooth_move_max_duration_s() -> float:
    """Longest duration a velocity-continuous prelude may take (``0.8005`` s).

    The longest REST-TO-REST smooth move the stroke admits (full travel,
    ``HAND_MOTOR_MAX_POSITION_REVS``), so an honoured prelude can never take
    longer than a profile the firmware could already emit — which is what keeps
    every host-side window sized on a commanded hand move valid without moving
    it.  Mirrors ``Trajectory.h::smoothMoveMaxDuration``; see that comment for the
    ``_PRIME_INFLIGHT_S`` failure it closes.
    """
    return math.sqrt(HAND_MOTOR_MAX_POSITION_REVS * QUINTIC_S2_MAX
                     / MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2)


def smooth_move_duration_s(delta_rev: float, v0_rps: float = 0.0) -> float:
    """The ACCEL-LIMITED duration, with the dead-band and the 0.05 s floor.

    Not the whole of what ``makeSmoothMove`` reports: the firmware SUBSTITUTES the
    rest-to-rest duration whenever the honoured profile's excursion would leave
    the stroke or its duration would exceed :func:`smooth_move_max_duration_s`, so
    at large ``v0`` this function over-reports (at 119.6 rev/s it returns 4.71 s
    where the firmware emits a 0.05 s floored hold).  Use
    :func:`plan_smooth_move` when the branch matters — a caller sizing an arm
    window from this number alone would refuse or defer exactly the fast arms the
    velocity-continuous prelude exists to serve.  Same name and same semantics as
    ``jugglebot.motion.trajectory.hand_stroke.smooth_move_duration_s`` — two
    functions with one name and different behaviour is exactly the drift this
    module exists to prevent.
    """
    v0 = 0.0 if abs(float(v0_rps)) <= SMOOTH_MOVE_V0_DEADBAND_RPS else float(v0_rps)
    if abs(float(delta_rev)) < 1e-6 and v0 == 0.0:
        return 0.0
    return max(smooth_move_accel_limited_duration_s(delta_rev, v0),
               SMOOTH_MOVE_MIN_DURATION_S)


def _quadratic_roots_in_unit(a: float, b: float, c: float) -> list:
    """Real roots of ``a x^2 + b x + c`` that lie in [0, 1], clamped."""
    out = []
    if abs(a) < 1e-14:
        if abs(b) > 1e-14:
            out.append(-c / b)
    else:
        disc = b * b - 4.0 * a * c
        if disc >= 0.0:
            r = math.sqrt(disc)
            out.append((-b + r) / (2.0 * a))
            out.append((-b - r) / (2.0 * a))
    return [min(max(x, 0.0), 1.0) for x in out if -1e-12 <= x <= 1.0 + 1e-12]


def smooth_move_peak_accel_rps2(delta_rev: float, v0_rps: float,
                                duration_s: float) -> float:
    """EXACT peak |acceleration| of the profile (rev/s²), closed form.

    ``a(tau) * T^2 = 12 tau (1-tau) (P + Q tau)``, ``P = 5d - 3u``,
    ``Q = 5u - 10d`` — a cubic vanishing at both ends, so its extrema are the
    roots of a quadratic.  Matches a 400 000-sample brute force to 1.2e-9
    relative over 4 000 random cases.

    Used by the tests to prove :func:`smooth_move_duration_s`'s bound is sound;
    the firmware does not need it, which is the point of preferring a closed-form
    duration over a search.
    """
    T = float(duration_s)
    if T <= 0.0:
        return 0.0
    u = float(v0_rps) * T
    P = 5.0 * delta_rev - 3.0 * u
    Q = 5.0 * u - 10.0 * delta_rev
    best = 0.0
    for tau in [0.0, 1.0] + _quadratic_roots_in_unit(3.0 * Q, -2.0 * (Q - P), -P):
        best = max(best, abs(12.0 * tau * (1.0 - tau) * (P + Q * tau)))
    return best / (T * T)


def smooth_move_excursion_rev(delta_rev: float, v0_rps: float,
                              duration_s: float) -> tuple:
    """``(lowest, highest)`` position relative to the START, in rev.

    ``p'(tau) = (1-tau)^2 [30 d tau^2 + u (1 + 2tau - 15tau^2)]``, so the
    interior turning point is again a quadratic root.  Matches brute force to
    1.1e-7 rev.  Both endpoints (0 and ``delta``) are always included, so the
    interval spans the whole commanded excursion, not only the overshoot.
    """
    T = float(duration_s)
    if T <= 0.0:
        return (min(0.0, delta_rev), max(0.0, delta_rev))
    u = float(v0_rps) * T
    lo = min(0.0, delta_rev)
    hi = max(0.0, delta_rev)
    for tau in _quadratic_roots_in_unit(30.0 * delta_rev - 15.0 * u, 2.0 * u, u):
        if tau <= 0.0 or tau >= 1.0:
            continue
        p = delta_rev * quintic_s(tau) + u * quintic_h(tau)
        lo, hi = min(lo, p), max(hi, p)
    return (lo, hi)


class SmoothMovePlan:
    """The profile :func:`plan_smooth_move` decided on, and why.

    ``empty`` reproduces ``makeSmoothMove``'s "already there" early return.
    ``velocity_continuous`` says whether ``v0`` was honoured; ``reason`` records
    which branch was taken so a caller (and the bench) can tell a deliberate
    fallback from a bug.
    """

    __slots__ = ('start_rev', 'target_rev', 'delta_rev', 'v0_seed_rps',
                 'v0_measured_rps', 'duration_s', 'empty',
                 'velocity_continuous', 'reason', 'peak_accel_rps2',
                 'excursion_lo_rev', 'excursion_hi_rev')

    def __init__(self, **kw):
        for k in self.__slots__:
            setattr(self, k, kw[k])

    def sample_rev(self, t: float) -> float:
        """Commanded position (rev) at ``t`` seconds from the move's start."""
        if self.empty or self.duration_s <= 0.0:
            return self.target_rev
        if t <= 0.0:
            return self.start_rev
        if t >= self.duration_s:
            return self.target_rev
        tau = t / self.duration_s
        u = self.v0_seed_rps * self.duration_s
        return (self.start_rev + self.delta_rev * quintic_s(tau)
                + u * quintic_h(tau))

    def sample_vel_rps(self, t: float) -> float:
        """Commanded velocity (rev/s) at ``t`` seconds from the move's start."""
        if self.empty or self.duration_s <= 0.0:
            return 0.0
        if t <= 0.0 or t >= self.duration_s:
            return 0.0 if t >= self.duration_s else self.v0_seed_rps
        tau = t / self.duration_s
        u = self.v0_seed_rps * self.duration_s
        return ((self.delta_rev * quintic_s1(tau) + u * quintic_h1(tau))
                / self.duration_s)

    def sample_accel_rps2(self, t: float) -> float:
        """Commanded acceleration (rev/s²) at ``t`` seconds from the move's start."""
        if self.empty or self.duration_s <= 0.0:
            return 0.0
        if t < 0.0 or t > self.duration_s:
            return 0.0
        tau = t / self.duration_s
        u = self.v0_seed_rps * self.duration_s
        return ((self.delta_rev * quintic_s2(tau) + u * quintic_h2(tau))
                / (self.duration_s * self.duration_s))

    def __repr__(self) -> str:                                  # pragma: no cover
        return ('SmoothMovePlan(%.6f -> %.6f rev, v0=%.3f seed=%.3f, T=%.4f s, '
                'continuous=%s, %s)'
                % (self.start_rev, self.target_rev, self.v0_measured_rps,
                   self.v0_seed_rps, self.duration_s,
                   self.velocity_continuous, self.reason))


def plan_smooth_move(start_rev: float, target_rev: float,
                     v0_rps: float = 0.0,
                     pos_deadband_rev: float = 1e-6) -> SmoothMovePlan:
    """Decide the smooth-move profile from the LIVE hand state.

    Three branches, in the order the firmware evaluates them.

    **1. Already there AND at rest -> empty.**  Preserved exactly, and it is
    load-bearing: ``hand_catch_prime_rev`` was moved to the derived stroke top
    (9.9594 rev) and the catch arm gated to after the throw stroke precisely so
    a catch from rest opens with the smallest possible prelude.  It is also the
    ONLY branch that returns nothing, and ``Teensy_code_platform.ino:472-475`` returns
    from the kind-3 handler BEFORE ``packedMsgs.clear()`` when the move is
    empty — so widening this branch would widen a hole in the only un-arm
    mechanism the Teensy offers (a pre-release SAFE_ABORT depends on a kind-3
    retract clobbering an armed kind-0 stroke).  Adding the ``at rest``
    conjunct **narrows** it: every case that used to return empty and is now
    moving returns a braking profile instead.

    **2. Velocity-continuous, if it fits.**  ``v0`` is honoured when it is above
    :data:`SMOOTH_MOVE_V0_DEADBAND_RPS` (below that it is treated as gravity-hold
    dither on the ODrive ``Vel_Estimate``, and chasing dither would command 8.9 mm
    of motion nobody asked for on a stationary hand, rising as ``v0**2``).  TWO
    things then have to fit, not one: the excursion against the stroke end stops,
    and the duration against :func:`smooth_move_max_duration_s` — arresting ``v0``
    costs time as well as travel, and the duration grows linearly in ``|v0|``
    without bound.

    **3. Cannot fit -> fall back to the rest-to-rest profile.**  When the bulge
    would leave the travel, or the arrest would outlast the longest rest-to-rest
    move the stroke admits, the profile reverts to exactly today's rest-to-rest
    quintic.  Rejected alternatives, by the failure mode each causes:

    * *Refuse the command.*  For a kind-3 that means not clobbering, and a
      kind-3 retract clobbering an armed kind-0 is the only un-arm mechanism the
      Teensy offers.  Refusing via an empty trajectory is worse still — the
      early return at ``Teensy_code_platform.ino:472-475`` sits BEFORE the queue clear,
      so an armed stroke would survive a SAFE_ABORT.
    * *Brake to the limit* (shorten the duration until the bulge fits, whatever
      acceleration that costs).  Near a target the bulge has no room to be
      absorbed by the s-shape's own travel, and the acceleration needed to squeeze
      it in is bounded by nothing the firmware declares.  Worked example: a
      SAFE_ABORT retract dispatched with the hand essentially AT
      ``hand_retract_rev = 0.0`` and descending at -60 rev/s (measured peak
      retract command speed 60.89 rev/s) would need ``T = 8.4 ms`` and a commanded
      **28 000 rev/s²**, 280x the declared limit.  Far from the target the same
      descent is served fine — 5.0 -> 0.0 rev at -7.5 rev/s is honoured.
      Declaring a second, higher acceleration limit changes what the machine can
      physically do at the bench — an operator decision, not an implementer's.

    The fallback is therefore *strictly no worse than today* (it IS today's
    profile), introduces no commanded magnitude the firmware could not already
    produce, and is **observable**: it emits a from-rest quintic seed, which is
    exactly what ``tools/probes/hand_stroke_timeline.py``'s ``seeds`` detector
    counts and bench row 2 of § Section HAND already aborts on.

    Parameters
    ----------
    start_rev, target_rev : float
        Live hand position and the commanded target, revs from encoder zero.
    v0_rps : float
        Live hand velocity (``current_hand_velocity``; ODrive ``Vel_Estimate``).
    pos_deadband_rev : float
        "Already there" tolerance.  Defaults to the firmware's ``1e-6`` rev
        (= 3.16e-5 mm).
    """
    delta = float(target_rev) - float(start_rev)
    v0 = float(v0_rps)
    at_rest = abs(v0) <= SMOOTH_MOVE_V0_DEADBAND_RPS

    def _plan(v_seed: float, reason: str, continuous: bool) -> SmoothMovePlan:
        T = max(smooth_move_accel_limited_duration_s(delta, v_seed),
                SMOOTH_MOVE_MIN_DURATION_S)
        lo, hi = smooth_move_excursion_rev(delta, v_seed, T)
        return SmoothMovePlan(
            start_rev=float(start_rev), target_rev=float(target_rev),
            delta_rev=delta, v0_seed_rps=v_seed, v0_measured_rps=v0,
            duration_s=T, empty=False, velocity_continuous=continuous,
            reason=reason,
            peak_accel_rps2=smooth_move_peak_accel_rps2(delta, v_seed, T),
            excursion_lo_rev=lo, excursion_hi_rev=hi)

    if abs(delta) < pos_deadband_rev and at_rest:
        return SmoothMovePlan(
            start_rev=float(start_rev), target_rev=float(target_rev),
            delta_rev=delta, v0_seed_rps=0.0, v0_measured_rps=v0,
            duration_s=0.0, empty=True, velocity_continuous=True,
            reason='at-target-and-at-rest', peak_accel_rps2=0.0,
            excursion_lo_rev=0.0, excursion_hi_rev=0.0)

    if not at_rest:
        plan = _plan(v0, 'velocity-continuous', True)
        # End stops, in the firmware's frame.  Both bounds are RELAXED to admit
        # the endpoints: the live position may already be outside the band (a
        # mid-coast reading above 10.6 rev is exactly the case that most needs a
        # continuous profile) and a legal target may sit below the lower bound,
        # and neither must make the move infeasible by definition.  EPS absorbs
        # rounding at those endpoints; it is not headroom.
        hi_bound = max(HAND_MOTOR_MAX_POSITION_REVS
                       - SMOOTH_MOVE_EXCURSION_MARGIN_REV,
                       plan.start_rev, plan.target_rev)
        lo_bound = min(HAND_RETRACT_REV, plan.start_rev, plan.target_rev)
        peak = plan.start_rev + plan.excursion_hi_rev
        trough = plan.start_rev + plan.excursion_lo_rev
        if (peak > hi_bound + SMOOTH_MOVE_END_STOP_EPS_REV
                or trough < lo_bound - SMOOTH_MOVE_END_STOP_EPS_REV):
            return _plan(0.0, 'excursion-would-leave-the-stroke', False)
        # ...and arresting v0 costs TIME as well as travel.  The duration grows
        # linearly in |v0| without bound, so the excursion clamp alone does not
        # stop a prelude outlasting a move the firmware could already command —
        # which is what every host-side window is sized on.
        if plan.duration_s > smooth_move_max_duration_s():
            return _plan(0.0, 'duration-would-outlast-a-rest-to-rest-move', False)
        return plan

    return _plan(0.0, 'at-rest', True)


# ---------------------------------------------------------------------------
# HandSmoothMove — port of makeSmoothMove(), mm domain
# ---------------------------------------------------------------------------

class HandSmoothMove:
    """Quintic point-to-point move, matching Teensy ``makeSmoothMove()``.

    Positions are mm in the sim's own frame (mm from the bottom of travel,
    matching ``sim/plant/mujoco_plant.command_hand``); the accel limit, the
    duration bound and the end stops all live in revs, so this class is a thin
    unit wrapper around :func:`plan_smooth_move`.

    ``start_vel_mm_s`` defaults to 0.0, which reproduces the historical
    rest-to-rest behaviour EXACTLY for every existing caller — the sim's catch
    and throw sequences do not track hand velocity and are unchanged.  The
    velocity-continuous path is the mirror of the firmware's, exercised by
    ``tests/sim/test_hand_trajectory.py`` and pinned against the shipped
    ``Trajectory.h`` by ``tests/firmware/test_hand_smooth_move_xref.py``.

    Parameters
    ----------
    start_pos_mm : float
        Starting hand position (mm).
    end_pos_mm : float
        Target hand position (mm).
    start_vel_mm_s : float
        Live hand velocity at the move's start (mm/s, positive = upward).
    """

    def __init__(self, start_pos_mm: float, end_pos_mm: float,
                 start_vel_mm_s: float = 0.0):
        self._start = start_pos_mm
        self._end = end_pos_mm
        self._start_vel_mm_s = start_vel_mm_s
        # The sim's historical "effectively zero" band is 1e-3 mm (= 3.16e-5
        # rev) against the firmware's 1e-6 rev; both are orders of magnitude
        # below the encoder resolution, so nothing reachable distinguishes them
        # and each is left as it was rather than silently unified here.
        #
        # NOTE the delta is now differenced in the REV domain (the firmware's
        # native frame: it reads a rev encoder and receives a rev target), where
        # the historical code differenced in mm and converted afterwards.  That
        # shifts existing sim durations by up to 1 ulp (e.g. 0.6549603309481845
        # -> ...846 s for the 100 -> 300 mm move), which is 9 orders of magnitude
        # inside the xref tolerance and makes the mirror MORE faithful, not less.
        self._plan = plan_smooth_move(
            mm_to_rev(start_pos_mm), mm_to_rev(end_pos_mm),
            v0_rps=mm_to_rev(start_vel_mm_s),
            pos_deadband_rev=mm_to_rev(0.001))
        self._duration = 0.0 if self._plan.empty else self._plan.duration_s

    @property
    def duration(self) -> float:
        """Duration of the smooth move (seconds)."""
        return self._duration

    @property
    def start_pos_mm(self) -> float:
        return self._start

    @property
    def end_pos_mm(self) -> float:
        return self._end

    @property
    def start_vel_mm_s(self) -> float:
        """Live hand velocity the profile was seeded with (mm/s)."""
        return self._start_vel_mm_s

    @property
    def plan(self) -> SmoothMovePlan:
        """The rev-domain plan (branch taken, peak accel, excursion)."""
        return self._plan

    @property
    def velocity_continuous(self) -> bool:
        """True when the live velocity was honoured (no commanded velocity step)."""
        return self._plan.velocity_continuous

    @property
    def peak_accel_rps2(self) -> float:
        """Exact peak commanded |acceleration| (rev/s²)."""
        return self._plan.peak_accel_rps2

    def excursion_mm(self) -> tuple:
        """``(lowest, highest)`` commanded position over the move (mm)."""
        return (self._start + rev_to_mm(self._plan.excursion_lo_rev),
                self._start + rev_to_mm(self._plan.excursion_hi_rev))

    def sample(self, t: float) -> float:
        """Return hand position (mm) at time *t* from move start.

        Returns start position for t < 0, end position for t > duration.
        """
        if self._duration == 0.0:
            return self._end
        if t <= 0.0:
            return self._start
        if t >= self._duration:
            return self._end
        return rev_to_mm(self._plan.sample_rev(t))

    def sample_velocity(self, t: float) -> float:
        """Commanded hand velocity (mm/s) at time *t* from move start."""
        if self._duration == 0.0:
            return 0.0
        return rev_to_mm(self._plan.sample_vel_rps(t))


# ---------------------------------------------------------------------------
# HandCatchSequence — smooth-move prelude + catch trajectory + hold
# ---------------------------------------------------------------------------

@dataclass
class HandCatchResult:
    """Result of attempting to create a HandCatchSequence."""
    sequence: HandCatchSequence | None
    feasible: bool
    reason: str = ""


class HandCatchSequence:
    """Complete hand catch sequence: smooth-move to start → catch trajectory.

    The sequence is anchored to absolute sim time via ``arrival_time``.

    Parameters
    ----------
    event_vel_mps : float
        Ball speed at landing (m/s).
    arrival_time : float
        Absolute sim time when ball arrives.
    current_pos_mm : float
        Current hand position (mm from physical bottom).
    """

    def __init__(
        self,
        event_vel_mps: float,
        arrival_time: float,
        current_pos_mm: float,
    ):
        # Build catch trajectory (positions from top of effective stroke)
        self._catch = HandCatchTrajectory(event_vel_mps)
        self._arrival_time = arrival_time

        # Catch trajectory start in absolute time
        self._catch_abs_start = arrival_time + self._catch.start_time

        # Build smooth-move from current position to catch trajectory start
        self._prelude = HandSmoothMove(current_pos_mm, self._catch.start_pos_mm)

        # Smooth-move must finish before catch trajectory starts
        self._prelude_start = self._catch_abs_start - self._prelude.duration

        # Absolute end time
        self._abs_end = arrival_time + self._catch.end_time

    @property
    def prelude_start_time(self) -> float:
        """Absolute sim time when smooth-move prelude begins."""
        return self._prelude_start

    @property
    def arrival_time(self) -> float:
        """Absolute sim time of ball arrival."""
        return self._arrival_time

    @property
    def end_time(self) -> float:
        """Absolute sim time when the full sequence ends."""
        return self._abs_end

    @property
    def catch_trajectory(self) -> HandCatchTrajectory:
        """The underlying catch trajectory (for inspection/testing)."""
        return self._catch

    @property
    def prelude(self) -> HandSmoothMove:
        """The smooth-move prelude (for inspection/testing)."""
        return self._prelude

    def sample(self, sim_time: float) -> float | None:
        """Return hand position (mm) at the given absolute sim time.

        Returns ``None`` after the sequence is complete.
        Before the prelude starts, returns the prelude's start position
        (hand should already be near there from priming).
        """
        if sim_time > self._abs_end:
            return None

        if sim_time < self._catch_abs_start:
            # In prelude phase
            t_prelude = sim_time - self._prelude_start
            return self._prelude.sample(t_prelude)

        # In catch trajectory phase
        t_catch = sim_time - self._arrival_time  # relative to ball arrival
        return self._catch.sample(t_catch)

    @staticmethod
    def try_create(
        event_vel_mps: float,
        arrival_time: float,
        current_pos_mm: float,
        current_time: float,
    ) -> HandCatchResult:
        """Attempt to create a HandCatchSequence with timing budget check.

        If there isn't enough time for the smooth-move prelude to complete
        before the catch trajectory starts (plus a safety gap), the catch
        attempt is infeasible and should be aborted.

        Parameters
        ----------
        event_vel_mps : float
            Ball speed at landing (m/s).
        arrival_time : float
            Absolute sim time when ball arrives.
        current_pos_mm : float
            Current hand position (mm).
        current_time : float
            Current sim time.

        Returns
        -------
        HandCatchResult
            ``feasible=True`` with a valid ``sequence``, or
            ``feasible=False`` with ``sequence=None`` and a ``reason``.
        """
        seq = HandCatchSequence(event_vel_mps, arrival_time, current_pos_mm)

        # Check timing budget: prelude must start no earlier than now,
        # with a safety gap
        time_available = seq.prelude_start_time - current_time
        if time_available < -SAFETY_GAP_S:
            shortfall = -time_available
            return HandCatchResult(
                sequence=None,
                feasible=False,
                reason=(
                    f"Insufficient time for hand catch: need "
                    f"{seq._prelude.duration:.3f}s smooth-move + "
                    f"{-seq._catch.start_time:.3f}s catch lead, "
                    f"but only {arrival_time - current_time:.3f}s until "
                    f"arrival (shortfall {shortfall:.3f}s)"
                ),
            )

        return HandCatchResult(sequence=seq, feasible=True)


# ---------------------------------------------------------------------------
# HandThrowTrajectory — port of calcThrow() + buildThrow()
# ---------------------------------------------------------------------------

class HandThrowTrajectory:
    """3-segment throw trajectory matching Teensy buildThrow().

    Moves the hand upward: accelerate → constant-velocity hold → decelerate.
    Ball is released at the end of the velocity-hold phase (t = 0 by convention).

    Timeline: t=0 is the moment of ball release (end of velocity-hold phase).

    Parameters
    ----------
    throw_speed_mps : float
        Desired ejection speed (m/s).  Clamped to [MIN, MAX].
    start_pos_mm : float
        Hand position at trajectory start (mm from physical bottom).
        Default: bottom of effective stroke (STROKE_MARGIN_MM).
    """

    def __init__(self, throw_speed_mps: float, start_pos_mm: float | None = None):
        v_throw = max(MIN_EVENT_VEL_MPS, min(MAX_EVENT_VEL_MPS, throw_speed_mps))
        self._throw_speed_mps = v_throw

        # Default start = bottom of effective stroke
        if start_pos_mm is None:
            start_pos_mm = STROKE_MARGIN_MM
        self._start_pos_mm = start_pos_mm

        # --- port of calcThrow() ---
        accel_stroke_m = (1.0 - THROW_VEL_HOLD_PCT) * _TOTAL_STROKE_M  # 0.95 * 0.315
        vel_hold_m = THROW_VEL_HOLD_PCT * _TOTAL_STROKE_M               # 0.05 * 0.315

        # Segment durations (positive)
        t_acc = 2.0 / (INERTIA_RATIO + 1.0) * accel_stroke_m / v_throw
        t_vel = vel_hold_m / v_throw
        t_dec = t_acc * INERTIA_RATIO

        # Accelerations
        throwA = v_throw / t_acc           # positive (upward)
        throwD = -throwA / INERTIA_RATIO   # negative (deceleration)

        # Positions in metres from effective bottom (positive = upward)
        x1_m = 0.5 * throwA * t_acc ** 2           # end of accel
        x2_m = x1_m + v_throw * t_vel               # end of vel hold = release point
        # x3_m computed for end position
        x3_m = x2_m + v_throw * t_dec + 0.5 * throwD * t_dec ** 2

        # Store parameters
        self._t_acc = t_acc
        self._t_vel = t_vel
        self._t_dec = t_dec
        self._throwA = throwA
        self._throwD = throwD
        self._x1_m = x1_m
        self._x2_m = x2_m
        self._x3_m = x3_m

        # Timeline: t=0 at release (end of vel-hold)
        # Trajectory starts at -(t_acc + t_vel), ends at +t_dec
        self._t_start = -(t_acc + t_vel)
        self._t_end = t_dec

    @property
    def release_time(self) -> float:
        """Always 0.0 (by timeline convention)."""
        return 0.0

    @property
    def release_pos_mm(self) -> float:
        """Hand position at ball release (mm from physical bottom)."""
        return self._start_pos_mm + self._x2_m * 1000.0

    @property
    def release_speed_mps(self) -> float:
        """Hand speed at release = throw_speed_mps (clamped)."""
        return self._throw_speed_mps

    @property
    def start_time(self) -> float:
        """Time before release when trajectory starts (negative)."""
        return self._t_start

    @property
    def end_time(self) -> float:
        """Time after release when trajectory ends (positive)."""
        return self._t_end

    @property
    def start_pos_mm(self) -> float:
        """Hand position at trajectory start (mm from physical bottom)."""
        return self._start_pos_mm

    @property
    def end_pos_mm(self) -> float:
        """Hand position at trajectory end (mm from physical bottom)."""
        return self._start_pos_mm + self._x3_m * 1000.0

    def sample(self, t: float) -> float:
        """Hand position (mm) at time t relative to ball release.

        Before start_time: returns start_pos_mm.
        After end_time: returns end_pos_mm (hold at final position).
        """
        if t <= self._t_start:
            return self._start_pos_mm
        if t >= self._t_end:
            return self.end_pos_mm

        # Convert to local time (0 = accel start)
        t_local = t - self._t_start  # shift so accel starts at 0
        disp_m = self._displacement_at_local(t_local)
        return self._start_pos_mm + disp_m * 1000.0

    def velocity_at(self, t: float) -> float:
        """Hand velocity (m/s, positive = upward) at time t relative to release."""
        if t <= self._t_start:
            return 0.0
        if t >= self._t_end:
            return 0.0

        t_local = t - self._t_start

        if t_local <= self._t_acc:
            return self._throwA * t_local

        if t_local <= self._t_acc + self._t_vel:
            return self._throw_speed_mps

        # Deceleration phase
        tau = t_local - (self._t_acc + self._t_vel)
        return self._throw_speed_mps + self._throwD * tau

    # -- torque feedforward ------------------------------------------------
    # ``buildThrow``'s ``torA[4]`` in ``Trajectory.h``: one value per segment,
    # held (not interpolated) across the segment, exactly as the firmware packs
    # it.  The DECEL segment is the only one on the corrected conversion — see
    # the module-level note at ``_TORQUE_K_THROW_DECEL`` and contract C-HAND-2.

    @property
    def accel_torque_nm(self) -> float:
        """Commanded torque feedforward over the acceleration segment (N.m)."""
        return accel_to_torque_nm(self._throwA)

    @property
    def decel_torque_nm(self) -> float:
        """Commanded torque feedforward over the POST-RELEASE decel segment (N.m).

        Negative (braking).  Sized from the axis's total reflected inertia
        rather than from the hand mass alone: after release the ball is gone, so
        the only thing the profile has to stop is rotor + hand, and that is a
        quantity the firmware can know exactly.
        """
        return throw_decel_to_torque_nm(self._throwD)

    def torque_at(self, t: float) -> float:
        """Commanded torque feedforward (N.m) at time t relative to release."""
        if t <= self._t_start or t >= self._t_end:
            return 0.0
        t_local = t - self._t_start
        if t_local <= self._t_acc:
            return self.accel_torque_nm
        if t_local <= self._t_acc + self._t_vel:
            return 0.0
        return self.decel_torque_nm

    def peak_decel_current_a(self, torque_constant_nm_per_a: float) -> float:
        """|decel torque feedforward| expressed as motor q-axis current (A).

        The quantity that must stay inside ``jugglebot_operational
        hand_curr_limit_a``: a feedforward that saturates the drive leaves the
        loop no authority to correct anything on top of it.
        """
        return abs(self.decel_torque_nm) / float(torque_constant_nm_per_a)

    def _displacement_at_local(self, t_local: float) -> float:
        """Displacement in metres from start position at local time."""
        if t_local <= 0.0:
            return 0.0

        if t_local <= self._t_acc:
            return 0.5 * self._throwA * t_local ** 2

        if t_local <= self._t_acc + self._t_vel:
            tau = t_local - self._t_acc
            return self._x1_m + self._throw_speed_mps * tau

        if t_local <= self._t_acc + self._t_vel + self._t_dec:
            tau = t_local - (self._t_acc + self._t_vel)
            return self._x2_m + self._throw_speed_mps * tau + 0.5 * self._throwD * tau ** 2

        return self._x3_m


# ---------------------------------------------------------------------------
# HandThrowSequence — smooth-move prelude + throw trajectory
# ---------------------------------------------------------------------------

@dataclass
class HandThrowResult:
    """Result of attempting to create a HandThrowSequence."""
    sequence: HandThrowSequence | None
    feasible: bool
    reason: str = ""


class HandThrowSequence:
    """Complete throw sequence: smooth-move to start → throw trajectory.

    Anchored to absolute sim time via ``release_time``.

    Parameters
    ----------
    throw_speed_mps : float
        Desired ejection speed (m/s).
    release_time : float
        Absolute sim time when ball should be released.
    current_pos_mm : float
        Current hand position (mm from physical bottom).
    """

    def __init__(
        self,
        throw_speed_mps: float,
        release_time: float,
        current_pos_mm: float,
    ):
        self._throw = HandThrowTrajectory(throw_speed_mps)
        self._release_time = release_time

        # Throw trajectory start in absolute time
        self._throw_abs_start = release_time + self._throw.start_time

        # Build smooth-move from current position to throw trajectory start
        self._prelude = HandSmoothMove(current_pos_mm, self._throw.start_pos_mm)

        # Smooth-move must finish before throw trajectory starts
        self._prelude_start = self._throw_abs_start - self._prelude.duration

        # Absolute end time
        self._abs_end = release_time + self._throw.end_time

    @property
    def prelude_start_time(self) -> float:
        """Absolute sim time when smooth-move prelude begins."""
        return self._prelude_start

    @property
    def release_time(self) -> float:
        """Absolute sim time of ball release."""
        return self._release_time

    @property
    def end_time(self) -> float:
        """Absolute sim time when throw trajectory ends."""
        return self._abs_end

    @property
    def throw_trajectory(self) -> HandThrowTrajectory:
        """The underlying throw trajectory (for inspection/testing)."""
        return self._throw

    @property
    def prelude(self) -> HandSmoothMove:
        """The smooth-move prelude (for inspection/testing)."""
        return self._prelude

    @property
    def end_pos_mm(self) -> float:
        """Hand position after throw completes (mm from physical bottom)."""
        return self._throw.end_pos_mm

    def sample(self, sim_time: float) -> float | None:
        """Hand position (mm) at the given absolute sim time.

        Returns ``None`` after the sequence is complete.
        Before the prelude starts, returns the prelude's start position.
        """
        if sim_time > self._abs_end:
            return None

        if sim_time < self._throw_abs_start:
            # In prelude phase
            t_prelude = sim_time - self._prelude_start
            return self._prelude.sample(t_prelude)

        # In throw trajectory phase
        t_throw = sim_time - self._release_time  # relative to ball release
        return self._throw.sample(t_throw)

    @staticmethod
    def try_create(
        throw_speed_mps: float,
        release_time: float,
        current_pos_mm: float,
        current_time: float,
    ) -> HandThrowResult:
        """Create with feasibility check (enough time for smooth-move prelude).

        Parameters
        ----------
        throw_speed_mps : float
            Desired ejection speed (m/s).
        release_time : float
            Absolute sim time when ball should be released.
        current_pos_mm : float
            Current hand position (mm).
        current_time : float
            Current sim time.

        Returns
        -------
        HandThrowResult
            ``feasible=True`` with a valid ``sequence``, or
            ``feasible=False`` with ``sequence=None`` and a ``reason``.
        """
        seq = HandThrowSequence(throw_speed_mps, release_time, current_pos_mm)

        time_available = seq.prelude_start_time - current_time
        if time_available < -SAFETY_GAP_S:
            shortfall = -time_available
            return HandThrowResult(
                sequence=None,
                feasible=False,
                reason=(
                    f"Insufficient time for hand throw: need "
                    f"{seq._prelude.duration:.3f}s smooth-move + "
                    f"{-seq._throw.start_time:.3f}s throw lead, "
                    f"but only {release_time - current_time:.3f}s until "
                    f"release (shortfall {shortfall:.3f}s)"
                ),
            )

        return HandThrowResult(sequence=seq, feasible=True)


# ---------------------------------------------------------------------------
# max_throw_speed_mps — maximum achievable throw speed
# ---------------------------------------------------------------------------

def max_throw_speed_mps() -> float:
    """Maximum ejection speed achievable within the hand's stroke and accel limits.

    The throw uses accel_stroke = (1 - THROW_VEL_HOLD_PCT) * TOTAL_STROKE for
    acceleration, with the kinematic relation v² = 2 * a * d.  However, the
    acceleration itself depends on v (t_acc = 2/(IR+1) * accel_stroke / v),
    so throwA = v * (IR+1) / (2 * accel_stroke).  This means the stroke is
    always fully consumed — the limit is actually MAX_EVENT_VEL_MPS.

    Returns the clamped maximum (MAX_EVENT_VEL_MPS = 7.0 m/s).
    """
    return MAX_EVENT_VEL_MPS
