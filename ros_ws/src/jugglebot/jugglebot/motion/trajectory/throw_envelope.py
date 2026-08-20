"""C-HAND-3 — the derived throw-admission envelope.  THE one enforcement point.

Contract: ``ros_ws/docs/hand_throw_envelope.md``.  Read it before changing a
number here; every constant below is a *configured* value or an algebraic
consequence of one, and that is the property the contract exists to keep.

**What this module answers.**  Given a nominated flight time and the release
speed the hand will actually be commanded to, may this throw be dispatched at
all?  It replaces the two hand-picked literals ``FLIGHT_TIME_MIN_S = 0.55`` /
``FLIGHT_TIME_MAX_S = 1.10`` that ``toss_sequencer`` carried until 2026-08-18 —
"plan sweep floor" and "plan sweep ceiling", neither of which was sized against
the hand's mechanical end stop.  They could not have been: the end stop's
DECLARED location was 11.1 rev, **0.3 rev past metal**, until it was corrected to
10.8 rev on 2026-08-18 (``logbook/2026-08-18-hand-end-stop-corrected.md``).
Against the corrected stop, the modelled post-release coast at the shipped
1.10 s ceiling peaks at **12.17 rev — 1.37 rev (43 mm) PAST the stop**.

**The five bounds, and which configured value each is made of.**  Every one is
evaluated on every call; the FIRST to fail names itself in the verdict, so a
refusal routes the operator at a subsystem rather than at a number.

===================  ==========================================================
bound                what it is
===================  ==========================================================
``END_STOP``         the hand's *uncommanded ballistic coast* past the commanded
                     stroke top ``x3`` must leave it below the mechanical stop.
                     Inputs: ``jugglebot_geometry.hand_motor_hard_stop_revs``,
                     ``hand_throw_envelope.end_stop_margin_rev``, and the
                     ``measured_coast_rev`` ladder.  **This is the bound the old
                     constants missed, and today it is the binding one.**
``DECEL_FF_HEADROOM`` the decel FEEDFORWARD alone must stay inside
                     ``decel_ff_current_headroom_frac`` of the current limit, or
                     it saturates the drive and the loop has no authority left to
                     correct on top of it (C-HAND-2's requirement).  **This is
                     what binds.**
``DECEL_AUTHORITY``  the deceleration the profile COMMANDS must be one the drive
                     can actually produce: ``a_cmd(v) <= I_max*Kt / (J*2*pi)``.
                     Inputs: ``jugglebot_odrive_defaults.hand_curr_limit_a``,
                     ``hand_torque_constant_nm_per_a``,
                     ``measured_reflected_inertia_kgm2``, and — as a second,
                     looser fence — ``hand_torque_soft_limit_nm``.
``ACCEL_AUTHORITY``  the same question for the ASCENT, which carries the ball as
                     well and fights gravity instead of being helped by it.
                     Never binds before ``DECEL_AUTHORITY`` (see the contract's
                     proof), but it is evaluated rather than assumed.
``REGEN``            peak BURST braking power at release must fit what the drive
                     is configured to push back: ``tau*omega <= |dc_max_negative
                     _current| * V_bus`` = 360 W.  Inputs:
                     ``regen_current_limit_a``, ``dc_bus_nominal_v``.  The rail's
                     300 W STEADY-STATE capacity is a separate, documented check
                     that a ~2 % duty cycle clears by ~60x — deliberately not the
                     instantaneous fence.
``ARM_WINDOW``       the Phase-1 catch-arm window must still be open with margin
                     at this flight — otherwise the ball is thrown uncatchable.
                     This is what actually sizes the FLOOR of the band.  Inputs:
                     ``arm_window_margin_s`` plus the whole of
                     :mod:`hand_stroke`'s timing model.  It is sized against
                     ``catch_vel_scale_default``; a lowered catch knob can still
                     close the window at an admitted flight (contract § Scope).
``WIRE_BAND``        ``teensy_bridge_node`` raises outside
                     ``[min_event_vel_mps, max_event_vel_mps]`` before any CAN
                     frame exists.  Belt-and-braces here: ``toss_sequencer``
                     gates on it one line earlier so the operator gets the
                     wire-level code, and this copy exists so *any* future caller
                     of :func:`evaluate` inherits the whole envelope.
===================  ==========================================================

**Fail closed.**  A non-finite or non-positive input, or a coast ladder that is
absent/empty/malformed, refuses with bound ``INPUT`` rather than assuming a
value.  Import-time validation of the ladder is deliberate: a bad regenerate
should be loud at node start, not at the first throw.

**No firmware change, and no firmware reads any of this.**  The generated
``HandEnv::`` C++ namespace is inert.  Deploying a change here is
``python config/generate_config.py`` +
``colcon build --packages-select jugglebot`` + relaunch.

**The coast model is MEASURED ON THE FLASHED PLANT** (2026-08-20), superseding
the 2026-07-27 pre-fix ladder this module shipped with on 2026-08-18.  That
ladder was captured with the legacy decel feedforward and the hand ODrive's
-10.00 A ``torque_soft_min`` live; on it a 1.10 s toss modelled to 12.17 rev
against a 10.8 rev stop and END_STOP set the whole envelope.  The clamp was the
mechanism: bag ``2026-08-18_18-42-19`` contains a mid-session clamp change, and
the same commanded 4.436 m/s throw coasts **0.763 rev clamped vs 0.18-0.23
unclamped**.  Post-fix ``END_STOP`` no longer binds and the ceiling is
``DECEL_AUTHORITY``.
"""

from __future__ import annotations

import math

import jugglebot.hardware_config as hw
from jugglebot.motion.trajectory import ballistics_bc, hand_stroke

# ── stroke geometry: taken from the ONE host-side copy of Trajectory.h ────────
#
# ``x2`` (ball release) and ``x3`` (stroke top) are both velocity-INDEPENDENT —
# ``x2 = accelSt/(1+IR) + velHold`` and ``x3 = totalStroke`` — so one reference
# model evaluates them exactly.  ``test_stroke_landmarks_are_velocity_independent``
# pins that across the whole wire band rather than trusting this comment.
_REF_MODEL = hand_stroke.HandStrokeModel(
    0.5 * (hw.TEENSY_TRAJ_MIN_EVENT_VEL_MPS + hw.TEENSY_TRAJ_MAX_EVENT_VEL_MPS))

#: ``x2`` — the hand position at ball release (rev from the encoder zero).
RELEASE_POS_REV = _REF_MODEL.x2_rev

#: ``x3`` — the commanded stroke top.  The profile NEVER leaves this; everything
#: above it is uncommanded coast.
STROKE_TOP_REV = hand_stroke.STROKE_TOP_REV

#: ``x3 - x2`` — the travel ``calcThrow`` allocates to the decel ramp.  Famously
#: **velocity-independent at 4.046 rev**, which is why the profile budgets the
#: IDEAL stopping distance with zero allowance for tracking error (C-HAND-2).
DECEL_ALLOWANCE_REV = STROKE_TOP_REV - RELEASE_POS_REV

#: The mechanical stop, and the line the coast peak must stay under.
HARD_STOP_REV = float(hw.GEOM_HAND_MOTOR_HARD_STOP_REVS)
END_STOP_MARGIN_REV = float(hw.HAND_ENV_END_STOP_MARGIN_REV)
PEAK_LIMIT_REV = HARD_STOP_REV - END_STOP_MARGIN_REV

# ── declared plant ───────────────────────────────────────────────────────────
KT_NM_PER_A = float(hw.HAND_ENV_HAND_TORQUE_CONSTANT_NM_PER_A)
CURRENT_LIMIT_A = float(hw.ODRIVE_HAND_CURR_LIMIT_A)
TORQUE_SOFT_LIMIT_NM = float(hw.HAND_ENV_HAND_TORQUE_SOFT_LIMIT_NM)
J_MEASURED_KGM2 = float(hw.HAND_ENV_MEASURED_REFLECTED_INERTIA_KGM2)
GRAVITY_TORQUE_NM = float(hw.HAND_ENV_GRAVITY_HOLD_CURRENT_A) * KT_NM_PER_A
#: BURST regen fence (W) — what the drive's ``dc_max_negative_current`` allows at
#: the nominal bus.  Braking is a burst, not a duty: the decel ramp is 50-90 ms
#: against a 3.5 s ``MIN_TOSS_THROW_DELAY_S`` cadence floor (~2 % duty), and the
#: owner has confirmed (2026-08-20) that the rail tolerates 360 W bursts.  So the
#: rail's STEADY-STATE capacity is NOT the instantaneous fence — using it as one
#: would tighten the envelope on a duty the machine never runs.
REGEN_POWER_W = (float(hw.HAND_ENV_REGEN_CURRENT_LIMIT_A)
                 * float(hw.HAND_ENV_DC_BUS_NOMINAL_V))

#: Steady-state rail capacity (W).  Documented, not enforced — see above.
REGEN_RAIL_STEADY_W = float(hw.HAND_ENV_REGEN_RAIL_CAPACITY_W)
ARM_WINDOW_MARGIN_S = float(hw.HAND_ENV_ARM_WINDOW_MARGIN_S)

# Fail closed on a hand-edited YAML too.  Both margins LOOSEN the envelope as
# they shrink, so a zero or negative value is not a degenerate case that refuses
# everything — it is one that admits more, silently.  That asymmetry is why they
# are validated at import rather than trusted.
for _name, _val in (('end_stop_margin_rev', END_STOP_MARGIN_REV),
                    ('arm_window_margin_s', ARM_WINDOW_MARGIN_S)):
    if not (math.isfinite(_val) and _val > 0.0):
        raise ValueError(
            'hand_throw_envelope.{} must be a positive finite value; got {!r}. '
            'A non-positive margin LOOSENS the envelope, so it fails at import '
            'rather than at the throw.'.format(_name, _val))

#: Ball mass implied by ``INERTIA_RATIO = m_hand/(m_hand + m_ball)`` — the same
#: identity that makes ``throwD = -throwA/IR`` a constant-motor-torque design.
BALL_MASS_KG = (hw.TEENSY_TRAJ_INERTIA_HAND_ONLY_KG
                * (1.0 / hw.TEENSY_TRAJ_INERTIA_RATIO - 1.0))

#: Reflected inertia the ASCENT sees: the axis plus the seated ball.  Release is
#: at ``x2``, the END of the velocity hold, so the ball is in the cup for the
#: whole ascent — the withdrawn accel-phase identification in C-HAND-2 is
#: exactly the mistake of forgetting that.
J_ASCENT_KGM2 = (J_MEASURED_KGM2
                 + BALL_MASS_KG / (hand_stroke.LINEAR_GAIN_REV_PER_M
                                   * 2.0 * math.pi) ** 2)

#: Largest commanded deceleration whose FEEDFORWARD alone stays inside
#: ``decel_ff_current_headroom_frac`` of the current limit (rev/s^2).
#:
#: The feedforward is sized on the DECLARED ``J_ff`` (9.5e-6), so it draws
#: ``J_ff*2*pi*a_cmd/Kt`` amps.  C-HAND-2 requires that to leave the loop room —
#: a saturated feedforward has no authority left to correct anything on top of
#: itself.  This bound is what stops the envelope admitting a throw C-HAND-2
#: forbids; it binds at 5.636 m/s, ahead of DECEL_AUTHORITY's 5.816.
DECEL_FF_HEADROOM_FRAC = float(hw.HAND_ENV_DECEL_FF_CURRENT_HEADROOM_FRAC)
DECEL_FF_HEADROOM_RPS2 = (
    DECEL_FF_HEADROOM_FRAC * CURRENT_LIMIT_A * KT_NM_PER_A
    / (hw.TEENSY_TRAJ_THROW_DECEL_REFLECTED_INERTIA_KGM2 * 2.0 * math.pi))

#: Largest braking deceleration the drive can produce (rev/s^2).  The current
#: limit binds; ``torque_soft_limit`` is 2.5x looser and is carried so a
#: regression back to the -10.00 A asymmetry shows up here instead of silently
#: truncating the brake.  Gravity is deliberately EXCLUDED: on an upward decel it
#: brakes in the same direction, so ignoring it under-states authority.
BRAKING_TORQUE_LIMIT_NM = min(CURRENT_LIMIT_A * KT_NM_PER_A,
                              TORQUE_SOFT_LIMIT_NM)
DECEL_AUTHORITY_RPS2 = BRAKING_TORQUE_LIMIT_NM / (J_MEASURED_KGM2 * 2.0 * math.pi)

#: Gravity torque during the ASCENT, when the ball is still in the cup.  The
#: declared hold current is BALL-FREE and ``INERTIA_RATIO = m_hand/(m_hand +
#: m_ball)`` is exactly the mass ratio, so dividing by it scales the hold torque
#: to the loaded hand.
#:
#: **It is a scaling, not an identity, and it under-states.**  It assumes the
#: measured 1.50 A hold is purely gravity on the full 0.281 kg.  Back-solving,
#: 1.50 A x Kt x 2*pi*G = 1.643 N implies a gravity-loaded mass of 0.168 kg, so
#: the real loaded factor is ~1.57 rather than 1/IR = 1.339 and this value is
#: ~17 % low (0.0111 vs 0.0130 N.m).  Immaterial while ACCEL_AUTHORITY does not
#: bind — it is 0.7 % of that ceiling, and the ceiling sits above
#: DECEL_AUTHORITY's — but it is an assumption (no counterweight, no static
#: friction in the hold current), not a derivation.
ASCENT_GRAVITY_TORQUE_NM = GRAVITY_TORQUE_NM / hw.TEENSY_TRAJ_INERTIA_RATIO

#: Largest ascent acceleration the drive can produce (rev/s^2).  Gravity is
#: SUBTRACTED here — on the way up it opposes the commanded accel.
ACCEL_AUTHORITY_RPS2 = ((BRAKING_TORQUE_LIMIT_NM - ASCENT_GRAVITY_TORQUE_NM)
                        / (J_ASCENT_KGM2 * 2.0 * math.pi))

# ── the measured coast ladder ────────────────────────────────────────────────


def _load_ladder():
    """Validate + monotonise ``hand_throw_envelope.measured_coast_rev``.

    Monotonised by a RUNNING MAXIMUM, not by sorting or fitting: the 3.440 m/s
    rung measured *less* coast than the 2.742 m/s rung (0.063 vs 0.074), which
    is measurement scatter on a quantity that is physically non-decreasing in
    speed, and the conservative reading of scatter is "the worse of the two
    applies from here up".

    Raises at import on a malformed ladder — a bad regenerate must be loud at
    node start, not at the first throw.
    """
    raw = getattr(hw, 'HAND_ENV_MEASURED_COAST_REV', None)
    if not raw:
        raise ValueError(
            'hand_throw_envelope.measured_coast_rev is empty or absent — the '
            'throw envelope has no coast model and cannot admit any throw. '
            'Run: python config/generate_config.py')
    out = []
    worst = float('-inf')
    prev_v = float('-inf')
    for row in raw:
        if len(row) != 2:
            raise ValueError(
                'measured_coast_rev rows must be [release_speed_mps, coast_rev]'
                ', got {!r}'.format(row))
        v, coast = float(row[0]), float(row[1])
        if not (math.isfinite(v) and math.isfinite(coast)) or v <= 0.0:
            raise ValueError('measured_coast_rev row {!r} is not a finite, '
                             'positive-speed measurement'.format(row))
        if coast < 0.0:
            # A negative rung would put the modelled peak BELOW x3, so END_STOP
            # could never fire and the envelope would silently open to the
            # DECEL_AUTHORITY ceiling.  Coast is an overshoot; it is >= 0.
            raise ValueError('measured_coast_rev row {!r} has a NEGATIVE coast '
                             '— coast is an overshoot past x3 and cannot be '
                             'less than zero'.format(row))
        if v <= prev_v:
            raise ValueError('measured_coast_rev must be strictly increasing in '
                             'release speed; {!r} follows {!r}'.format(v, prev_v))
        prev_v = v
        worst = max(worst, coast)
        out.append((v, worst))
    return tuple(out)


COAST_LADDER = _load_ladder()

_TOP_V, _TOP_COAST = COAST_LADDER[-1]

#: Achieved post-release deceleration at the ladder's TOP rung (rev/s^2), by the
#: measurement's own definition ``a = v_rev^2 / (2 * stopping distance)``.
#: Reported, and used for the aliasing budget; no longer the extrapolation law.
TOP_RUNG_ACHIEVED_DECEL_RPS2 = (
    (_TOP_V * hand_stroke.LINEAR_GAIN_REV_PER_M) ** 2
    / (2.0 * (DECEL_ALLOWANCE_REV + _TOP_COAST)))

#: Exponent of the extrapolation above the top rung: ``coast_top*(v/v_top)^p``.
#:
#: **Why a power law and not the old "hold achieved decel constant".**  That law
#: (``coast = v_rev^2/(2*a_top) - d_dec``) is the AUTHORITY-SATURATED shape, and
#: it fitted the pre-fix plant, whose tracking fraction was collapsing
#: (eta 0.982 -> 0.799).  The post-fix plant is TRACKING-limited with eta nearly
#: flat (0.968 / 0.954 / 0.951 across the measured span) and ~1.8x the braking
#: authority it is using, so asserting that its achieved deceleration saturates
#: at the measured 2302 rev/s^2 would be physically wrong — and wildly so: it
#: predicts 3.30 rev of coast at 5.816 m/s where the fitted family predicts 0.39.
COAST_EXPONENT = float(hw.HAND_ENV_COAST_EXTRAPOLATION_EXPONENT)
if not (math.isfinite(COAST_EXPONENT) and COAST_EXPONENT > 0.0):
    raise ValueError(
        'hand_throw_envelope.coast_extrapolation_exponent must be positive and '
        'finite; got {!r}. A non-positive exponent would make coast SHRINK with '
        'speed above the ladder.'.format(COAST_EXPONENT))


def coast_rev(release_speed_mps: float) -> float:
    """Modelled uncommanded coast past ``x3`` at this release speed (rev).

    Each rung is that speed's **MAXIMUM** observed coast on the FLASHED plant
    (bags ``2026-08-20_21-51-39`` and ``2026-08-18_18-42-19``; n = 2 / 2 / 14).
    So the only scatter the margin still has to pay for is telemetry aliasing
    (~0.029 rev) — see the contract's § B1.

    Above the top rung it extrapolates ``coast_top*(v/v_top)^p``.  **That is an
    extrapolation, and the contract says so plainly**: the data spans
    3.142-4.436 m/s and the shipped ceiling is 5.816 m/s, 1.31x past the fastest
    speed ever measured here.  It is not load-bearing for the ceiling — END_STOP
    does not bind at any exponent in the fitted family — only for the claim that
    it does not.

    Piecewise-linear through :data:`COAST_LADDER`; flat at the lowest rung's
    value below it (coast shrinks with speed, so the lowest measured value is a
    ceiling down there); and above the top rung, the constant-achieved-decel law
    ``v_rev^2 / (2*a_top) - d_dec``, which joins the ladder continuously at the
    top rung by construction.
    """
    v = float(release_speed_mps)
    if v <= COAST_LADDER[0][0]:
        return COAST_LADDER[0][1]
    if v >= _TOP_V:
        return _TOP_COAST * (v / _TOP_V) ** COAST_EXPONENT
    for (v0, c0), (v1, c1) in zip(COAST_LADDER, COAST_LADDER[1:]):
        if v0 <= v <= v1:
            return c0 + (c1 - c0) * (v - v0) / (v1 - v0)
    raise AssertionError('unreachable: coast ladder is not contiguous')


def peak_rev(release_speed_mps: float) -> float:
    """Modelled PEAK hand position of a throw at this release speed (rev).

    ``x3 + coast(v)``.  What the runbook's ``peak`` column measures, and what
    ``§ CHECK HAND-7``'s per-rung bands are written against.
    """
    return STROKE_TOP_REV + coast_rev(release_speed_mps)


def commanded_decel_rps2(release_speed_mps: float) -> float:
    """The deceleration ``calcThrow`` commands after release (rev/s^2).

    Read off the shipped stroke model rather than re-derived, so the
    ``123.55*v^2`` coefficient cannot drift out of this file.
    """
    m = hand_stroke.HandStrokeModel(release_speed_mps)
    return abs(m.throwD) * hand_stroke.LINEAR_GAIN_REV_PER_M


def commanded_accel_rps2(release_speed_mps: float) -> float:
    """The ascent acceleration ``calcThrow`` commands (rev/s^2)."""
    m = hand_stroke.HandStrokeModel(release_speed_mps)
    return abs(m.throwA) * hand_stroke.LINEAR_GAIN_REV_PER_M


# ── ballistics: the Tier-8a co-located vertical projection ───────────────────
#
# The gate itself takes the release speed the caller resolved, so it is tier
# agnostic.  These helpers exist for the REPORTED band and for the arm-window
# bound, whose armed velocity is the flight's VERTICAL arrival speed and is
# therefore identical for a displaced (8b) throw of the same flight time.

GRAVITY_MMS2 = float(ballistics_bc.GRAVITY_MMS2)

#: Cup plane minus release plane (mm) — the same expression ``toss_sequencer``
#: and ``motion/trajectory/toss_release`` use.
RELEASE_TO_CUP_MM = float(hw.HAND_CATCH_OFFSET_MM) - (
    float(hw.GEOM_HAND_AXIS_BOTTOM_OFFSET_MM) + float(hw.HAND_THROW_POS_M) * 1000.0)


def vertical_release_speed_mps(flight_time_s: float) -> float:
    """Tier-8a release speed for a flight time (m/s): ``dz/T + g*T/2``."""
    t = float(flight_time_s)
    return (RELEASE_TO_CUP_MM / t + GRAVITY_MMS2 * t / 2.0) / 1000.0


def flight_time_for_vertical_speed_s(release_speed_mps: float) -> float:
    """Inverse of :func:`vertical_release_speed_mps` — the ascending (physical)
    root of ``g/2*T^2 - v*T + dz = 0``.  Returns ``nan`` when no real toss
    reaches that speed."""
    a = GRAVITY_MMS2 / 2.0
    b = -float(release_speed_mps) * 1000.0
    disc = b * b - 4.0 * a * RELEASE_TO_CUP_MM
    if disc < 0.0:
        return float('nan')
    return (-b + math.sqrt(disc)) / (2.0 * a)


def apex_height_m(flight_time_s: float) -> float:
    """Operator-facing apex height (m) — ``Toss.action``'s ``throw_height_m``.
    ``h = g*T^2/8``, the exact inverse of ``toss_release.flight_time_from_height``."""
    return GRAVITY_MMS2 * float(flight_time_s) ** 2 / 8.0 / 1000.0


def armed_catch_speed_mps(flight_time_s: float) -> float:
    """Velocity the catch stroke will be armed with (m/s).

    The ball's VERTICAL arrival speed ``|dz/T - g*T/2|`` scaled by the catch
    knob's config default.  Vertical-only, and therefore identical for a
    displaced Tier-8b throw of the same flight time: a horizontal launch
    component changes neither the vertical launch component nor the fall.
    """
    t = float(flight_time_s)
    v_arr = abs(RELEASE_TO_CUP_MM / t - GRAVITY_MMS2 * t / 2.0) / 1000.0
    return v_arr * float(hw.JB_OP_CATCH_VEL_SCALE_DEFAULT)


#: The ``max(_MIN_EVENT_DELAY_S, required_arm_lead)`` floor that
#: ``catch_coordinator_node`` applies.  Local copy for the same reason
#: ``tests/motion/test_hand_stroke.py`` keeps one — importing the node drags
#: rclpy into a pure module.  Pinned equal by
#: ``test_min_event_delay_matches_the_catch_coordinator``.
MIN_EVENT_DELAY_S = 0.3


def arm_window_s(flight_time_s: float, release_speed_mps: float) -> float:
    """Width of the Phase-1 catch-arm window at this (flight, release speed).

    ``latest - earliest`` where ``earliest = t_dec(v_throw) +
    ARM_SUPPRESS_MARGIN_S`` (the throw stroke must be clear) and ``latest =
    T - max(MIN_EVENT_DELAY_S, required_arm_lead(v_armed))`` (the Platform
    Teensy's **makeSmoothMove time-budget check** must still fit — quoted by
    NAME, not by line: it is ``Teensy_code_platform.ino:642`` today, and the
    ``:533`` several older docs cite is now a CAN state-update handler).  Both
    edges are :mod:`hand_stroke`'s, so this function adds no timing model of
    its own.

    Negative means the arm cannot be placed at all: the throw stroke is still
    decelerating when the catch already needed to be armed, so the ball flies
    uncatchable.  It narrows monotonically toward SHORT flights, which is why
    this — not the coast — is what sizes the floor of the band.
    """
    t = float(flight_time_s)
    earliest = (hand_stroke.throw_decel_s(release_speed_mps)
                + hand_stroke.ARM_SUPPRESS_MARGIN_S)
    latest = t - max(MIN_EVENT_DELAY_S,
                     hand_stroke.required_arm_lead_s(armed_catch_speed_mps(t)))
    return latest - earliest


# ── the verdict ──────────────────────────────────────────────────────────────


class ThrowEnvelopeVerdict(object):
    """Result of :func:`evaluate`.  ``ok`` plus the NAME of the binding bound.

    ``detail`` is written to be pasted into an operator's message verbatim: it
    always carries the offending quantity, the derived limit, and the units, so
    a refusal never reads as a bare "too high".
    """

    __slots__ = ('ok', 'bound', 'detail')

    def __init__(self, ok, bound='', detail=''):
        self.ok = bool(ok)
        self.bound = str(bound)
        self.detail = str(detail)

    def __repr__(self):
        return 'ThrowEnvelopeVerdict(ok={!r}, bound={!r}, detail={!r})'.format(
            self.ok, self.bound, self.detail)

    @property
    def message(self):
        """``BOUND:detail``, or ``''`` when admitted — the string the FSM appends
        to its reject code."""
        return '' if self.ok else '{}:{}'.format(self.bound, self.detail)


_OK = ThrowEnvelopeVerdict(True)


def evaluate(flight_time_s: float, release_speed_mps: float
             ) -> ThrowEnvelopeVerdict:
    """Admit or refuse a throw.  THE enforcement point of contract C-HAND-3.

    ``flight_time_s`` is the nominated release-to-catch-plane time;
    ``release_speed_mps`` is the speed the hand will actually be commanded to
    (the goal's resolved ``event_vel_mps``), NOT a value re-derived from the
    flight time — a Tier-8b displaced throw aims, so the two are different
    numbers and only the commanded one bounds the hardware.

    Bounds are checked machine-damage-first (``END_STOP`` before
    ``ARM_WINDOW``), so when several fail the operator hears about the one that
    breaks metal.
    """
    t = float(flight_time_s)
    v = float(release_speed_mps)

    # Fail closed on anything we cannot reason about.
    if not (math.isfinite(t) and t > 0.0):
        return ThrowEnvelopeVerdict(
            False, 'INPUT',
            'flight_time_s={:g} is not a positive, finite time'.format(t))
    if not (math.isfinite(v) and v > 0.0):
        return ThrowEnvelopeVerdict(
            False, 'INPUT',
            'release_speed_mps={:g} is not a positive, finite speed'.format(v))

    # 1. END_STOP — the coast peak must clear metal by the declared margin.
    peak = peak_rev(v)
    if peak > PEAK_LIMIT_REV:
        return ThrowEnvelopeVerdict(
            False, 'END_STOP',
            'modelled peak {:.3f} rev at {:.3f} m/s exceeds {:.3f} rev '
            '(hard stop {:.2f} - margin {:.2f}); envelope allows '
            'v <= {:.3f} m/s (T <= {:.3f} s, apex <= {:.3f} m)'.format(
                peak, v, PEAK_LIMIT_REV, HARD_STOP_REV, END_STOP_MARGIN_REV,
                max_release_speed_mps(), max_flight_time_s(),
                apex_height_m(max_flight_time_s())))

    # 2. DECEL_AUTHORITY — the commanded brake must be one the drive can make.
    a_dec = commanded_decel_rps2(v)
    if a_dec > DECEL_AUTHORITY_RPS2:
        return ThrowEnvelopeVerdict(
            False, 'DECEL_AUTHORITY',
            'commanded decel {:.0f} rev/s^2 at {:.3f} m/s exceeds the axis '
            'ceiling {:.0f} rev/s^2 ({:.1f} A x {:.6f} Nm/A over J={:.4g} '
            'kg m^2)'.format(a_dec, v, DECEL_AUTHORITY_RPS2, CURRENT_LIMIT_A,
                             KT_NM_PER_A, J_MEASURED_KGM2))

    # 3. DECEL_FF_HEADROOM — the feedforward must leave the loop some current.
    if a_dec > DECEL_FF_HEADROOM_RPS2:
        ff_a = (a_dec * hw.TEENSY_TRAJ_THROW_DECEL_REFLECTED_INERTIA_KGM2
                * 2.0 * math.pi / KT_NM_PER_A)
        return ThrowEnvelopeVerdict(
            False, 'DECEL_FF_HEADROOM',
            'the decel feedforward alone wants {:.1f} A at {:.3f} m/s, over the '
            '{:.0%} of {:.0f} A C-HAND-2 reserves for it ({:.1f} A) — a '
            'saturated feedforward leaves the loop no authority'.format(
                ff_a, v, DECEL_FF_HEADROOM_FRAC, CURRENT_LIMIT_A,
                DECEL_FF_HEADROOM_FRAC * CURRENT_LIMIT_A))

    # 4. ACCEL_AUTHORITY — same question for the ball-carrying ascent.
    a_acc = commanded_accel_rps2(v)
    if a_acc > ACCEL_AUTHORITY_RPS2:
        return ThrowEnvelopeVerdict(
            False, 'ACCEL_AUTHORITY',
            'commanded ascent accel {:.0f} rev/s^2 at {:.3f} m/s exceeds the '
            'ball-carrying ceiling {:.0f} rev/s^2 (J={:.4g} kg m^2 incl. a '
            '{:.4f} kg ball, less {:.5f} Nm of gravity)'.format(
                a_acc, v, ACCEL_AUTHORITY_RPS2, J_ASCENT_KGM2, BALL_MASS_KG,
                ASCENT_GRAVITY_TORQUE_NM))

    # 5. REGEN — peak braking power is at release, where omega is largest.
    #
    # Deliberately the torque LIMIT, not the commanded torque: this fences what
    # the drive COULD push back if it braked flat out, which is conservative
    # inside the DECEL_AUTHORITY-admitted set (where tau_cmd <= tau_max by
    # construction) and consistent with how DECEL_AUTHORITY itself is derived.
    omega = v * hand_stroke.LINEAR_GAIN_REV_PER_M * 2.0 * math.pi
    brake_power_w = BRAKING_TORQUE_LIMIT_NM * omega
    if brake_power_w > REGEN_POWER_W:
        return ThrowEnvelopeVerdict(
            False, 'REGEN',
            'braking at {:.3f} m/s returns up to {:.0f} W, over the {:.0f} W '
            'regen budget ({:.1f} A x {:.1f} V)'.format(
                v, brake_power_w, REGEN_POWER_W,
                hw.HAND_ENV_REGEN_CURRENT_LIMIT_A, hw.HAND_ENV_DC_BUS_NOMINAL_V))

    # 6. WIRE_BAND — the bridge refuses outside this before any CAN frame.
    if not (hw.TEENSY_TRAJ_MIN_EVENT_VEL_MPS <= v
            <= hw.TEENSY_TRAJ_MAX_EVENT_VEL_MPS):
        return ThrowEnvelopeVerdict(
            False, 'WIRE_BAND',
            'release speed {:.3f} m/s is outside the bridge band '
            '[{:g}, {:g}] m/s'.format(v, hw.TEENSY_TRAJ_MIN_EVENT_VEL_MPS,
                                      hw.TEENSY_TRAJ_MAX_EVENT_VEL_MPS))

    # 7. ARM_WINDOW — the catch must still be armable after the stroke clears.
    window = arm_window_s(t, v)
    if window < ARM_WINDOW_MARGIN_S:
        return ThrowEnvelopeVerdict(
            False, 'ARM_WINDOW',
            'catch-arm window {:.3f} s at T={:.3f} s is under the {:.3f} s '
            'margin (window closes entirely at T={:.4f} s); envelope allows '
            'T >= {:.3f} s (apex >= {:.3f} m)'.format(
                window, t, ARM_WINDOW_MARGIN_S, arm_window_closes_at_s(),
                min_flight_time_s(), apex_height_m(min_flight_time_s())))

    return _OK


# ── the reported band (derived at import, never a gate) ──────────────────────
#
# Solving each bound for its edge is a one-dimensional root find, so a bisection
# is exact to double precision and needs no closed form per bound — which
# matters because the coast ladder has no closed form at all.  The edges are
# computed ONCE at import: `evaluate` quotes them in its refusal messages, so
# computing them inside `evaluate` would recurse.  They are REPORTING values;
# `evaluate` never *gates* on them.


def _speed_ok(v):
    """The four speed bounds plus the wire band, with no message building — the
    predicate the edge search bisects.  Kept in lockstep with :func:`evaluate`
    by ``test_reported_band_edges_agree_with_evaluate``."""
    return (peak_rev(v) <= PEAK_LIMIT_REV
            and commanded_decel_rps2(v) <= DECEL_FF_HEADROOM_RPS2
            and commanded_decel_rps2(v) <= DECEL_AUTHORITY_RPS2
            and commanded_accel_rps2(v) <= ACCEL_AUTHORITY_RPS2
            and (BRAKING_TORQUE_LIMIT_NM * v
                 * hand_stroke.LINEAR_GAIN_REV_PER_M * 2.0 * math.pi)
            <= REGEN_POWER_W
            and hw.TEENSY_TRAJ_MIN_EVENT_VEL_MPS <= v
            <= hw.TEENSY_TRAJ_MAX_EVENT_VEL_MPS)


def _flight_ok(t):
    """Every bound, on the Tier-8a co-located projection of a flight time."""
    if not (math.isfinite(t) and t > 0.0):
        return False
    v = vertical_release_speed_mps(t)
    return _speed_ok(v) and arm_window_s(t, v) >= ARM_WINDOW_MARGIN_S


def _bisect(pred, lo, hi, iters=200):
    """Largest ``x`` in ``[lo, hi]`` with ``pred(x)`` true, given ``pred(lo)``
    true and exactly one sign change over the bracket.

    Returns a value that SATISFIES ``pred``.  Which side of the crossing a
    bisection returns is not cosmetic here: a band edge that lands a float
    epsilon on the refused side is an edge :func:`evaluate` rejects, and every
    consumer that samples the band at its own edge (the sim sweep does) then
    fails on a value the band advertised.  :func:`_bisect_first` is the mirror
    for a rising edge, for the same reason.
    """
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        if pred(mid):
            lo = mid
        else:
            hi = mid
    return lo


def _bisect_first(pred, lo, hi, iters=200):
    """Smallest ``x`` in ``[lo, hi]`` with ``pred(x)`` true, given ``pred(hi)``
    true and exactly one sign change.  Returns a value that SATISFIES ``pred``."""
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        if pred(mid):
            hi = mid
        else:
            lo = mid
    return hi


#: Search bracket for the flight-time edges.  Both ends are DELIBERATELY outside
#: anything flyable (0.05 s is the smooth-move duration floor itself; 2.0 s needs
#: a 9.8 m/s release, well past the 7.0 m/s wire clamp), so a band that reached
#: an end would be a configuration error rather than a physical answer — which is
#: what ``test_the_band_does_not_touch_its_own_search_bracket`` asserts.
_SEARCH_LO_S, _SEARCH_HI_S, _SEARCH_STEP_S = 0.05, 2.0, 0.005


def _find_flight_band():
    """``(min, max)`` admitted Tier-8a flight times, or ``(nan, nan)``.

    A coarse scan first, because the admitted set is an INTERVAL whose two ends
    are refused for DIFFERENT reasons (ARM_WINDOW below, END_STOP above), so no
    single monotone predicate brackets it.  The scan finds the admitted cells,
    then each edge is bisected inside the 5 ms cell it must lie in.  An empty
    scan returns ``nan``, and every flight-time query then reads as refused —
    fail closed, the same posture as a malformed ladder.
    """
    n = int(round((_SEARCH_HI_S - _SEARCH_LO_S) / _SEARCH_STEP_S))
    grid = [_SEARCH_LO_S + i * _SEARCH_STEP_S for i in range(n + 1)]
    admitted = [t for t in grid if _flight_ok(t)]
    if not admitted:
        return (float('nan'), float('nan'))
    lo_seed, hi_seed = admitted[0], admitted[-1]
    lo = _bisect_first(_flight_ok,
                       max(_SEARCH_LO_S, lo_seed - _SEARCH_STEP_S), lo_seed)
    hi = _bisect(_flight_ok, hi_seed,
                 min(_SEARCH_HI_S, hi_seed + _SEARCH_STEP_S))
    return (lo, hi)


#: Highest release speed every SPEED bound admits (m/s).
MAX_RELEASE_SPEED_MPS = (
    hw.TEENSY_TRAJ_MAX_EVENT_VEL_MPS
    if _speed_ok(hw.TEENSY_TRAJ_MAX_EVENT_VEL_MPS)
    else _bisect(_speed_ok, hw.TEENSY_TRAJ_MIN_EVENT_VEL_MPS,
                 hw.TEENSY_TRAJ_MAX_EVENT_VEL_MPS))

#: Flight time at which the catch-arm window reaches exactly zero (s).  Quoted in
#: the ``ARM_WINDOW`` refusal so the operator can see how far the goal sits from
#: *impossible*, not merely from *policy*.
ARM_WINDOW_CLOSES_AT_S = _bisect(
    lambda t: arm_window_s(t, vertical_release_speed_mps(t)) <= 0.0,
    _SEARCH_LO_S, _SEARCH_HI_S)

#: The derived replacement for the old ``FLIGHT_TIME_MIN_S`` /
#: ``FLIGHT_TIME_MAX_S`` literals.
#:
#: **A PROJECTION, not the gate.**  It is the image of the speed envelope under
#: the Tier-8a co-located vertical inverse.  A Tier-8b displaced throw of the
#: same flight time is AIMED and so releases faster, and only :func:`evaluate` —
#: which is handed the commanded speed — bounds it.
MIN_FLIGHT_TIME_S, MAX_FLIGHT_TIME_S = _find_flight_band()


def max_release_speed_mps() -> float:
    """Highest release speed every SPEED bound admits (m/s)."""
    return MAX_RELEASE_SPEED_MPS


def arm_window_closes_at_s() -> float:
    """Flight time at which the catch-arm window reaches exactly zero (s)."""
    return ARM_WINDOW_CLOSES_AT_S


def min_flight_time_s() -> float:
    """Shortest Tier-8a flight the whole envelope admits (s)."""
    return MIN_FLIGHT_TIME_S


def max_flight_time_s() -> float:
    """Longest Tier-8a flight the whole envelope admits (s)."""
    return MAX_FLIGHT_TIME_S


def flight_time_band_s():
    """``(min, max)`` Tier-8a flight times — see :data:`MIN_FLIGHT_TIME_S`."""
    return (MIN_FLIGHT_TIME_S, MAX_FLIGHT_TIME_S)


def apex_height_band_m():
    """``(min, max)`` operator-facing apex heights (m) — ``Toss.action``'s
    ``throw_height_m``, the variable an operator actually types."""
    return (apex_height_m(MIN_FLIGHT_TIME_S), apex_height_m(MAX_FLIGHT_TIME_S))
