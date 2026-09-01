"""T-U5 — ``feasibility.validate_cycle``, the unified 7-DoF cycle gate.

Plan: ``plans/active/unified-7dof-planner.md`` § 4 Phase 1 (WP3).

WHAT THESE TESTS DEFEND
-----------------------
``validate_cycle`` is the single enforcement point for a 7-channel
``cycle_plan.CyclePlan``.  Two failure classes live here, and only one of them
would ever be loud on its own:

* **The delegation trap.**  A ``CyclePlan`` carries ZERO segments, and every
  measurement loop in ``feasibility`` iterates ``plan.segments``.  Handed a cycle
  plan, ``validate`` and ``validate_follow`` collapse to a single sample at
  ``t = 0`` and return ``OK`` on a trajectory nothing measured.
  ``test_the_six_dof_gates_are_blind_to_a_cycle_plan`` pins that, because the day
  someone "simplifies" ``validate_cycle`` into a ``validate`` call is the day the
  whole 7-channel path stops being gated — silently.
* **The hand gates themselves** — stroke, velocity, acceleration, per-knot step,
  and the catch-runway re-check — each driven by a MINIMAL violating plan so the
  code that fires is the code under test and not a neighbour.

EMPIRICAL-PROBE RECIPES (CLAUDE.md's empirical-probe rule)
----------------------------------------------------------
Every violating plan below was prototyped in ``/tmp/probe_validate_codes.py``
first and confirmed to fire its code deterministically (each run twice,
identical) on the pinned stack, 2026-08-30.  The recipes, with the numbers that
make each one fire exactly one gate:

===================  =========================================================
code                 recipe (dt = 0.025 s, pose held at NEUTRAL throughout)
===================  =========================================================
``HAND_STROKE``      3 knots at rest, hand ``[5.0, 10.2, 5.0]`` rev — the middle
                     knot is past the 9.9594 rev operating top.  ``-0.5`` instead
                     of ``10.2`` goes below the homed zero.  At rest everywhere,
                     so no vel/acc/step gate can fire.  ⚠ To reach the END-STOP
                     branch the FIRST sample must already be past 10.8 rev
                     (``[11.5, 11.5]``): on a rise out of the band the gate
                     refuses at the first out-of-band sample, which on a
                     ``[5.0, 11.5, 5.0]`` ramp is 10.484 rev — inside the end
                     stop.  That cost a probe iteration and is why the recipe is
                     written out here.
``HAND_LIMIT_VEL``   2 knots, hand ``[1.0, 7.5]`` rev with BOTH knot velocities
                     260 rev/s — a constant-velocity span, so the acceleration is
                     identically zero and the stroke stays inside ``[0, 9.9594]``.
                     Only the 200 rev/s cap can fire.
``HAND_LIMIT_ACC``   3 knots, hand ``[1.0, 2.0, 3.0]`` rev with velocities
                     ``[0, 100, 0]`` rev/s.  Measured peak 6400 rev/s² against
                     the 3500 cap; |v| peaks at 100 rev/s, half the vel cap, and
                     the per-knot step is 1.0 rev against a 4.0 rev bound.
``STEP_BOUND``       2 knots, constant 180 rev/s = 0.9 × the vel cap ⇒ a 4.5 rev
                     per-knot step against the 4.0 rev bound (0.8 × 200 × 0.025)
                     while |v| stays UNDER the cap.  This is the only shape that
                     reaches the hand step bound without tripping velocity first,
                     which is itself worth knowing: the bound is a backstop.
``UNREACHABLE``      A finite ``CyclePlan`` built normally, then ``plan.pose`` or
                     ``plan.hand_rev`` mutated to NaN in place.  The constructor
                     refuses non-finite arrays, so a post-construction mutation is
                     how the gate's own defence-in-depth check is reachable.
``HAND_STROKE``      2 knots, ``catch_k = 1``, hand ``[3.5, 1.0]`` rev at a
(catch runway)       CONSTANT -100 rev/s (``p1 = p0 - 100·dt`` makes the Hermite
                     acceleration identically zero, so the runway is the only
                     thing that can answer).  The achieved catch speed of
                     100 rev/s needs ``100²/(2·3500) = 1.429`` rev to stop, plus
                     the 0.632 rev margin = 2.061 rev, against 1.000 rev of stroke
                     below the catch.  ⚠ Do NOT open ``hand_acc_limit_rps2`` to
                     isolate this gate: that limit IS the runway's deceleration
                     authority, so opening it makes the requirement vanish.
===================  =========================================================

The clean end-to-end case (through ``cup_cycle.plan_cup_cycle`` →
``cup_realize.tilt_schedule`` / ``decompose`` → ``CyclePlan.from_realized``) was
found the same way; ``_clean_cycle``'s docstring carries the sweep that produced
it and the measured peaks.

Unmarked and parallel-safe: pure computation, no filesystem, no ports.
"""

from __future__ import annotations

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.trajectory import TrajectoryLimits
from jugglebot.motion.trajectory import cup_cycle as cc
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory import feasibility as feas
from jugglebot.motion.trajectory import tilt_geometry as tg
from jugglebot.motion.trajectory.cycle_plan import CyclePlan
from jugglebot.outcome_detail import base_outcome, outcome_subcode

NEUTRAL = np.array([0.0, 0.0, float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM), 0.0, 0.0, 0.0])
DT = 0.025


@pytest.fixture
def geom():
    return StewartGeometry()


@pytest.fixture
def session_limits():
    """The SHIPPED session limits — what a launched trajectory_node enforces."""
    return TrajectoryLimits.from_config(hw)


def _limits(*, vel=1e9, acc=1e9, jerk=1e9, step=1e9,
            hand_vel=None, hand_acc=None, dt=DT):
    """Limits with the LEG ceilings effectively off and the HAND caps shipped.

    The hand pair defaults to the generated config so a hand test asserts against
    the number the machine will actually run, not a number invented here; the leg
    ceilings are opened so a hand-channel test cannot be answered by a leg code.
    """
    return TrajectoryLimits(
        leg_vel_mmps=vel, leg_acc_mmps2=acc, leg_jerk_mmps3=jerk,
        leg_vel_ceiling_mmps=1e12, leg_acc_ceiling_mmps2=1e12,
        leg_jerk_ceiling_mmps3=1e12,
        hand_vel_limit_rps=(float(hw.JB_TRAJ_HAND_VEL_LIMIT_RPS)
                            if hand_vel is None else hand_vel),
        hand_acc_limit_rps2=(float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2)
                             if hand_acc is None else hand_acc),
        hand_vel_ceiling_rps=1e12, hand_acc_ceiling_rps2=1e12,
        knot_dt_s=dt, max_step_rev=step, min_move_duration_s=0.2,
        min_timed_lead_s=0.25, max_timed_lead_s=60.0)


def _held(hand_rev, hand_vel_rps, *, dt=DT, catch_k=-1):
    """A cycle holding NEUTRAL with a prescribed hand track — the minimal driver.

    The platform is motionless at every knot, so every leg-space peak is exactly
    zero and the ONLY gate that can fire is a hand gate.  That isolation is the
    whole point of the shape.
    """
    hand_rev = np.asarray(hand_rev, dtype=float)
    n = hand_rev.shape[0]
    return CyclePlan(pose=np.tile(NEUTRAL, (n, 1)), pose_vel=np.zeros((n, 6)),
                     hand_rev=hand_rev,
                     hand_vel_rps=np.asarray(hand_vel_rps, dtype=float),
                     dt=dt, catch_k=catch_k)


# ── the clean end-to-end cycle ───────────────────────────────────────────────

#: The SLIDER-reachable cup-z window.  ``cup_cycle``'s own docstring says the
#: caller MUST override ``z_min_m`` / ``z_max_m`` to this; with the 0.45/1.10
#: defaults the realisation saturates its stroke clamp at 31 of 41 knots and the
#: gate refuses (measured 2026-08-30).  Operating band ``[0, 9.9594]`` rev maps to
#: slider 20..335 mm, i.e. cup z 679.6..994.6 mm at level; the tilt drop moves that
#: by at most ~6 mm, which the inset below covers.
CUP_Z_LO_M = 0.690
CUP_Z_HI_M = 0.985


def _clean_cycle():
    """A real cycle that passes ``validate_cycle`` clean at the SHIPPED limits.

    Found by sweep (``/tmp/probe_clean_case2.py``, 2026-08-30).  Two parameters
    are load-bearing and neither is arbitrary:

    * **flight 0.5 s, not 0.8 s.**  The cup's vertical acceleration peaks AT THE
      RELEASE and scales with take-off speed: 140.2 m/s² (= 4432 rev/s²) at
      flight 0.8 s, 43.2 m/s² (= 1367 rev/s²) at 0.5 s.  The QP bounds cup-z
      JERK (``max_jerk_z``, box active at 6000 m/s³) but nothing bounds cup-z
      ACCELERATION, so the 3500 rev/s² hand cap is what actually limits the
      admissible flight time.  Lowering ``max_jerk_z`` instead does NOT help —
      3000 and below make the QP infeasible outright.
    * **level tilts.**  Any tilt schedule at all pushes leg acceleration past the
      5000 mm/s² session limit (7943 at 0.5 rad/s with no banking; 104k–115k with
      banking at the 3.0 rad/s default), because a rate-limited tilt series
      finite-differenced into knot rates puts large rotational accelerations
      through the lever arm.  That is a real Phase-1 result for the WP4 sim
      harness, not something to tune away here.

    Measured peaks for this case (2026-08-30): leg vel 44.0 mm/s, acc 393.8
    mm/s², jerk 17005 mm/s³, step 0.016 rev; hand vel 78.6 rev/s, acc 1366.6
    rev/s², step 1.917 rev, position within [0.33, 9.66] rev.
    """
    period_s, flight_s, amp = 1.4, 0.5, 0.02
    throw_pos = np.array([amp, 0.4 * amp, 0.88])
    throw_target = np.array([-amp, -0.4 * amp, 0.88])
    v_take = cc.takeoff_velocity(throw_pos, throw_target, flight_s)
    cfg = cc.CupCycleConfig(z_min_m=CUP_Z_LO_M, z_max_m=CUP_Z_HI_M,
                            catch_runway_z_floor_m=CUP_Z_LO_M)
    cup = cc.plan_cup_cycle(
        throw_pos, v_take, cc.GRAVITY, throw_pos, throw_target, flight_s,
        period_s * 0.55, np.array([-0.9 * amp, -0.5 * amp, 0.85]),
        np.array([0.10, -0.05, -2.5]), period_s, cfg=cfg)
    rcfg = cr.RealizeConfig(banking_enabled=False)
    tilts = cr.tilt_schedule(cup, np.zeros(2), np.zeros(2), rcfg)
    realized = cr.decompose(cup, tilts, rcfg)
    return cup, realized, CyclePlan.from_realized(realized)


# ═══════════════════════════════════════════════════════════════════════════
# The reason this function exists at all
# ═══════════════════════════════════════════════════════════════════════════

def test_the_six_dof_gates_are_blind_to_a_cycle_plan(geom, session_limits):
    """``validate`` / ``validate_follow`` pass an UNGATED cycle — characterisation.

    Both iterate ``plan.segments``; a ``CyclePlan`` has none, so both fall through
    to their single-pose degenerate branch and return ``OK``.  The driver here is
    an 8-span x-sweep whose real leg acceleration is far past any session limit,
    plus a hand knot parked 1.1 rev past the physical end stop — a plan that must
    never be called feasible.

    If this ever goes RED, one of the two gates learned to sample a segmentless
    plan: good news, but re-read ``validate_cycle``'s module block before deleting
    anything, because the 7th channel is still invisible to both of them.
    """
    n = 9
    t = np.arange(n) * DT
    pose = np.tile(NEUTRAL, (n, 1))
    pose[:, 0] = 60.0 * np.sin(2 * np.pi * t / (n * DT))
    pose_vel = np.zeros((n, 6))
    pose_vel[:, 0] = (60.0 * (2 * np.pi / (n * DT))
                      * np.cos(2 * np.pi * t / (n * DT)))
    hand = np.full(n, 5.0)
    hand[4] = 12.0                       # past the 10.8 rev end stop
    cyc = CyclePlan(pose=pose, pose_vel=pose_vel, hand_rev=hand,
                    hand_vel_rps=np.zeros(n), dt=DT)

    assert cyc.segments == ()
    assert feas.validate(cyc, session_limits, geom).ok is True
    assert feas.validate_follow(cyc, session_limits, geom).ok is True
    # ...and the leg jerk both of them report for a whole sweeping cycle is zero.
    assert feas.validate(cyc, session_limits, geom).peak_leg_jerk_mmps3 == 0.0

    report = feas.validate_cycle(cyc, session_limits, geom)
    assert report.ok is False
    assert report.code == feas.HAND_STROKE


# ═══════════════════════════════════════════════════════════════════════════
# HAND_STROKE
# ═══════════════════════════════════════════════════════════════════════════

def test_hand_stroke_refuses_above_the_operating_top(geom):
    """Recipe: 3 knots at rest, hand ``[5.0, 10.2, 5.0]`` rev."""
    report = feas.validate_cycle(
        _held([5.0, 10.2, 5.0], [0.0, 0.0, 0.0]), _limits(), geom)
    assert report.code == feas.HAND_STROKE
    assert report.ok is False
    assert report.peak_hand_rev == pytest.approx(10.2)
    detail = report.reasons[0]
    # The bound_msg / range_msg discipline: the offending value, the band, and
    # the units, all inside the one string that reaches the operator.
    assert '10.200 rev' in detail
    assert '[0.000, 9.959]' in detail
    # Inside the operating band it is NOT past the end stop, so the refusal must
    # not shout about one.
    assert 'END STOP' not in detail


def test_hand_stroke_names_the_end_stop_separately(geom):
    """Recipe: BOTH knots at 11.5 rev — past the 10.8 rev physical stop.

    The first sample must already be past the end stop.  On a ramp OUT of the
    band the gate refuses at the first out-of-band sample, which is inside the end
    stop — correct, but it exercises the other branch.
    """
    report = feas.validate_cycle(
        _held([11.5, 11.5], [0.0, 0.0]), _limits(), geom)
    assert report.code == feas.HAND_STROKE
    assert 'END STOP' in report.reasons[0]
    assert '10.80 rev' in report.reasons[0]


def test_hand_stroke_refuses_below_the_homed_zero(geom):
    """Recipe: the middle knot at -0.5 rev."""
    report = feas.validate_cycle(
        _held([5.0, -0.5, 5.0], [0.0, 0.0, 0.0]), _limits(), geom)
    assert report.code == feas.HAND_STROKE
    assert 'BELOW the homed zero' in report.reasons[0]


def test_hand_stroke_sees_an_excursion_hidden_BETWEEN_knots(geom):
    """Both knots inside the band, the cubic between them outside it.

    Knots at 9.0 rev with equal and opposite 190 rev/s velocities bulge the
    Hermite to 10.19 rev even though NEITHER knot leaves the band and neither knot
    velocity reaches the 200 rev/s cap.  A per-knot check alone would pass it.
    """
    plan = _held([9.0, 9.0], [190.0, -190.0])
    # Neither knot is out of band, and neither is over the velocity cap...
    assert plan.hand_rev.max() < feas.HAND_STROKE_MAX_REV
    assert np.abs(plan.hand_vel_rps).max() < float(hw.JB_TRAJ_HAND_VEL_LIMIT_RPS)
    report = feas.validate_cycle(plan, _limits(), geom)
    assert report.code == feas.HAND_STROKE
    assert report.peak_hand_rev == pytest.approx(10.1875)


# ═══════════════════════════════════════════════════════════════════════════
# HAND_LIMIT_VEL
# ═══════════════════════════════════════════════════════════════════════════

def test_hand_limit_vel_refuses_a_span_over_the_cap(geom):
    """Recipe: 2 knots, constant 260 rev/s (``[1.0, 7.5]`` rev over 25 ms).

    Constant velocity ⇒ zero acceleration, and 7.5 rev is inside the operating
    band, so ``HAND_LIMIT_VEL`` is the only gate that can answer.
    """
    plan = _held([1.0, 1.0 + 260.0 * DT], [260.0, 260.0])
    report = feas.validate_cycle(plan, _limits(), geom)
    assert report.code == feas.HAND_LIMIT_VEL
    assert report.peak_hand_vel_rps == pytest.approx(260.0)
    assert report.peak_hand_acc_rps2 == pytest.approx(0.0, abs=1e-9)
    detail = report.reasons[0]
    assert '260.00 rev/s' in detail and '200.00 rev/s' in detail
    assert '[trajectory_op.hand_vel_limit_rps]' in detail


def test_hand_limit_vel_sees_an_extremum_hidden_BETWEEN_knots(geom):
    """Both knot velocities at 10 rev/s; the cubic peaks at 220 rev/s inside.

    This is why the hand channel's span extrema are closed-form rather than
    sampled: a knot-only reading would report 10 rev/s and pass a span that
    genuinely asks for 220.
    """
    plan = _held([1.0, 1.0 + 150.0 * DT], [10.0, 10.0])
    assert np.abs(plan.hand_vel_rps).max() == 10.0
    report = feas.validate_cycle(plan, _limits(), geom)
    assert report.code == feas.HAND_LIMIT_VEL
    assert report.peak_hand_vel_rps == pytest.approx(220.0)


# ═══════════════════════════════════════════════════════════════════════════
# HAND_LIMIT_ACC
# ═══════════════════════════════════════════════════════════════════════════

def test_hand_limit_acc_refuses_a_span_over_the_cap(geom):
    """Recipe: 3 knots, hand ``[1.0, 2.0, 3.0]`` rev, velocities ``[0, 100, 0]``.

    Measured peak 6400 rev/s² against the shipped 3500 cap.  |v| tops out at
    100 rev/s (half the velocity cap) and the per-knot step is 1.0 rev against a
    4.0 rev bound, so no neighbouring gate can claim the failure.
    """
    plan = _held([1.0, 2.0, 3.0], [0.0, 100.0, 0.0])
    report = feas.validate_cycle(plan, _limits(), geom)
    assert report.code == feas.HAND_LIMIT_ACC
    assert report.peak_hand_acc_rps2 == pytest.approx(6400.0)
    assert report.peak_hand_vel_rps <= float(hw.JB_TRAJ_HAND_VEL_LIMIT_RPS)
    detail = report.reasons[0]
    assert '6400.0 rev/s^2' in detail and '3500.0 rev/s^2' in detail
    assert '[trajectory_op.hand_acc_limit_rps2]' in detail


def test_hand_acceleration_is_the_plans_true_demand_not_a_knot_difference(geom):
    """The closed form recovers the plan's real acceleration; an FD would not.

    On the clean end-to-end cycle the analytic cup-z acceleration peaks at
    140.19 m/s² when the flight is 0.8 s.  Measured 2026-08-30: the gate's
    closed-form span extrema report exactly ``140.19 × LINEAR_GAIN`` = 4432.5
    rev/s², while a finite difference of the KNOT velocities reports 2763.5 —
    a 38 % under-measurement that would have ACCEPTED a plan asking for 4432
    against a 3500 cap.  That is the whole argument for
    :func:`feasibility._hand_span_extrema`.
    """
    period_s, flight_s, amp = 1.0, 0.8, 0.02
    throw_pos = np.array([amp, 0.4 * amp, 0.80])
    throw_target = np.array([-amp, -0.4 * amp, 0.80])
    v_take = cc.takeoff_velocity(throw_pos, throw_target, flight_s)
    cfg = cc.CupCycleConfig(z_min_m=CUP_Z_LO_M, z_max_m=CUP_Z_HI_M,
                            catch_runway_z_floor_m=CUP_Z_LO_M)
    cup = cc.plan_cup_cycle(
        throw_pos, v_take, cc.GRAVITY, throw_pos, throw_target, flight_s,
        period_s * 0.55, np.array([-0.9 * amp, -0.5 * amp, 0.78]),
        np.array([0.10, -0.05, -4.5]), period_s, cfg=cfg)
    rcfg = cr.RealizeConfig(banking_enabled=False)
    tilts = cr.tilt_schedule(cup, np.zeros(2), np.zeros(2), rcfg)
    realized = cr.decompose(cup, tilts, rcfg)
    cyc = CyclePlan.from_realized(realized)

    gain = cr.LINEAR_GAIN_REV_PER_M
    truth_rps2 = float(np.abs(cup.acc[:, 2]).max()) * gain
    report = feas.validate_cycle(cyc, _limits(), geom)
    assert report.peak_hand_acc_rps2 == pytest.approx(truth_rps2, rel=1e-9)

    fd_rps2 = float(np.abs(np.diff(cyc.hand_vel_rps) / cyc.dt).max())
    assert fd_rps2 < 0.75 * truth_rps2          # the under-measurement is real
    assert fd_rps2 < float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2) < truth_rps2


# ═══════════════════════════════════════════════════════════════════════════
# STEP_BOUND (the 7th channel reuses the existing code)
# ═══════════════════════════════════════════════════════════════════════════

def test_hand_step_bound_refuses_without_tripping_the_velocity_cap(geom):
    """Recipe: constant 180 rev/s = 0.9 × the cap ⇒ a 4.5 rev step vs 4.0 rev.

    The bound is ``STEP_BOUND_MARGIN × hand_vel_limit_rps × the plan's dt``, so a step
    can only exceed it when the MEAN velocity over the span exceeds 80 % of the
    cap.  A constant-velocity span at 0.9 × cap is therefore the only shape that
    reaches it with the velocity gate silent — which says plainly that this bound
    is a backstop, not a primary constraint.  It reuses ``STEP_BOUND`` because the
    fact is the same one the leg code reports: a per-knot wire step is too large.
    """
    plan = _held([1.0, 1.0 + 180.0 * DT], [180.0, 180.0])
    report = feas.validate_cycle(plan, _limits(), geom)
    assert report.code == feas.STEP_BOUND
    assert report.peak_hand_step_rev == pytest.approx(4.5)
    assert report.peak_hand_vel_rps <= float(hw.JB_TRAJ_HAND_VEL_LIMIT_RPS)
    assert '4.500 rev' in report.reasons[0]
    assert '4.000 rev' in report.reasons[0]


# ═══════════════════════════════════════════════════════════════════════════
# UNREACHABLE — non-finite input
# ═══════════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize('channel', ['pose', 'hand_rev', 'pose_vel',
                                     'hand_vel_rps'])
def test_a_nan_anywhere_is_unreachable(geom, channel):
    """NaN ⇒ the existing ``UNREACHABLE``, not a silent pass.

    ``CyclePlan.__init__`` refuses non-finite arrays, so the recipe mutates a
    validly-constructed plan in place — which is exactly the duck-typed /
    mutated-array case the gate's own finite check is defence-in-depth against.
    Every comparison against NaN is False, so without an explicit check a NaN
    sails through every bound below it.
    """
    plan = _held(np.full(5, 5.0), np.zeros(5))
    arr = getattr(plan, channel)
    if arr.ndim == 2:
        arr[2, 0] = np.nan
    else:
        arr[2] = np.nan
    report = feas.validate_cycle(plan, _limits(), geom)
    assert report.ok is False
    assert report.code == feas.UNREACHABLE
    assert 'non-finite' in report.reasons[0]


# ═══════════════════════════════════════════════════════════════════════════
# The terminal-hold cliff
# ═══════════════════════════════════════════════════════════════════════════

def test_the_terminal_hold_does_not_mask_the_release_velocity(geom):
    """A cycle ENDS at the throw, moving — the gate must see that.

    ``TrajectoryPlan``'s contract returns ``final_pose`` with ZERO twist at and
    past ``total_duration``, so sampling the last knot at exactly the end would
    read a hard stop and hide the release.  The gate samples the last instant
    still inside the plan instead.  Recipe: 3 knots whose FINAL velocity is
    260 rev/s, everything before it inside the caps.
    """
    plan = _held([1.0, 3.0, 5.0], [0.0, 130.0, 260.0])
    # The contract really does report a stop at the end.
    assert plan.hand_at(plan.total_duration) == (5.0, 0.0)
    report = feas.validate_cycle(plan, _limits(), geom)
    assert report.code == feas.HAND_LIMIT_VEL
    assert report.peak_hand_vel_rps == pytest.approx(260.0)


# ═══════════════════════════════════════════════════════════════════════════
# The catch runway, re-checked with the ACHIEVED velocity
# ═══════════════════════════════════════════════════════════════════════════

#: A constant -100 rev/s descent: ``p1 = p0 - 100·dt`` makes every Hermite
#: coefficient of the acceleration vanish, so the span is exactly uniform motion
#: and the runway is the only thing that can refuse it.
_RUNWAY_SPEED_RPS = 100.0
_RUNWAY_DROP_REV = _RUNWAY_SPEED_RPS * DT       # 2.5 rev per span


def _descent(p0, *, catch_k=1):
    return _held([p0, p0 - _RUNWAY_DROP_REV],
                 [-_RUNWAY_SPEED_RPS, -_RUNWAY_SPEED_RPS], catch_k=catch_k)


def test_catch_runway_refuses_with_the_achieved_catch_velocity(geom):
    """Recipe: ``catch_k = 1``, hand ``[3.5, 1.0]`` rev at a constant -100 rev/s.

    100 rev/s needs ``100²/(2·3500) = 1.429`` rev to stop, plus the 0.632 rev
    margin = 2.061 rev, against 1.000 rev of stroke below the catch.  Reported as
    ``HAND_STROKE`` because the fact IS a stroke fact: the travel below the catch
    is insufficient.  The SHIPPED ``hand_acc_limit_rps2`` is used deliberately —
    it is the deceleration authority the requirement is sized against.
    """
    report = feas.validate_cycle(_descent(3.5), _limits(), geom)
    assert report.code == feas.HAND_STROKE
    detail = report.reasons[0]
    assert 'catch runway' in detail
    assert '1.000 rev' in detail and '2.061 rev' in detail
    assert 'achieved catch speed 100.0 rev/s' in detail


def test_catch_runway_is_silent_without_a_catch(geom):
    """The identical hand track with ``catch_k = -1`` is not a runway question."""
    assert feas.validate_cycle(_descent(3.5, catch_k=-1), _limits(), geom).ok


def test_catch_runway_passes_with_stroke_to_spare(geom):
    """The same descent 4.5 rev higher clears the requirement."""
    report = feas.validate_cycle(_descent(8.0), _limits(), geom)
    assert report.ok is True, report.reasons


def test_catch_runway_tightens_as_the_deceleration_authority_drops(geom):
    """Halving ``hand_acc_limit_rps2`` quadruples the stopping distance.

    ``v²/(2a)`` — so the plan that cleared the runway at 3500 rev/s² with 5.5 rev
    of stroke below it needs 5.71 rev at 875 rev/s² and is refused.  Pinned
    because the runway is the ONE gate whose requirement moves with a limit rather
    than being compared against it.
    """
    assert feas.validate_cycle(_descent(8.0), _limits(), geom).ok is True
    report = feas.validate_cycle(_descent(8.0), _limits(hand_acc=875.0), geom)
    assert report.code == feas.HAND_STROKE
    assert 'catch runway' in report.reasons[0]


def test_the_runway_margin_matches_the_planners_own(geom):
    """The gate's rev-space margin IS ``cup_cycle``'s metre-space one.

    ``feasibility`` restates the number rather than importing ``cup_cycle``
    (importing a PLANNER from the gate it is checked by inverts the layering), so
    the two spellings are pinned against each other here — the only thing that
    stops the second copy drifting.
    """
    assert feas.CATCH_RUNWAY_MARGIN_M == pytest.approx(
        cc.CupCycleConfig().catch_runway_margin_m)
    assert feas.CATCH_RUNWAY_MARGIN_REV == pytest.approx(
        feas.CATCH_RUNWAY_MARGIN_M * cr.LINEAR_GAIN_REV_PER_M)


# ═══════════════════════════════════════════════════════════════════════════
# The refusal-string contract
# ═══════════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize('code', [feas.HAND_STROKE, feas.HAND_LIMIT_VEL,
                                  feas.HAND_LIMIT_ACC])
def test_every_new_code_round_trips_through_base_outcome(code):
    """A new code must survive ``base_outcome`` bare AND composed into an outcome.

    ``outcome_detail``'s contract: adding a parenthetical turns ``REJECTED_X``
    into ``REJECTED_X(...)`` and silently breaks every equality test against the
    bare code.  A code that itself contained a ``(`` would break the same guards
    from the other end, so the round trip is asserted in both forms.
    """
    assert base_outcome(code) == code
    assert outcome_subcode(code) == ''          # a code, not a code-with-detail
    composed = 'REJECTED_{}({})'.format(code, 'peak hand velocity 260.00 > 200.00')
    assert base_outcome(composed) == 'REJECTED_{}'.format(code)


@pytest.mark.parametrize('code,plan_args,limit_kw', [
    (feas.HAND_STROKE, ([5.0, 10.2, 5.0], [0.0, 0.0, 0.0]), {}),
    (feas.HAND_LIMIT_VEL, ([1.0, 1.0 + 260.0 * DT], [260.0, 260.0]), {}),
    (feas.HAND_LIMIT_ACC, ([1.0, 2.0, 3.0], [0.0, 100.0, 0.0]), {}),
])
def test_a_real_refusal_round_trips_too(geom, code, plan_args, limit_kw):
    """Not just the constant — the code a real refusal CARRIES round-trips."""
    report = feas.validate_cycle(_held(*plan_args), _limits(**limit_kw), geom)
    assert report.code == code
    assert base_outcome(report.code) == code
    assert report.reasons and report.reasons[0]


# ═══════════════════════════════════════════════════════════════════════════
# The clean end-to-end cycle
# ═══════════════════════════════════════════════════════════════════════════

def test_a_real_cycle_passes_clean_at_the_shipped_session_limits(geom,
                                                                 session_limits):
    """WP1 → WP2 → ``CyclePlan`` → the gate, with nothing opened up.

    The whole chain runs: ``plan_cup_cycle`` (QP) → ``tilt_schedule`` →
    ``decompose`` → ``CyclePlan.from_realized`` → ``validate_cycle``, judged
    against ``TrajectoryLimits.from_config(hw)`` — the limits a launched
    ``trajectory_node`` enforces.  See :func:`_clean_cycle` for why this
    particular cycle and what the peaks measure.
    """
    cup, realized, cyc = _clean_cycle()
    report = feas.validate_cycle(cyc, session_limits, geom)
    assert report.ok is True, report.reasons
    assert report.code == feas.OK

    # The realisation never hit its stroke clamp, so the cup went where the QP
    # planned rather than where the clamp allowed.
    assert not realized.slider_saturated.any()
    # Every peak is populated and inside its limit — a pass that measured nothing
    # would show zeros here, which is exactly the delegation trap's signature.
    assert 0.0 < report.peak_hand_vel_rps <= session_limits.hand_vel_limit_rps
    assert 0.0 < report.peak_hand_acc_rps2 <= session_limits.hand_acc_limit_rps2
    assert 0.0 < report.peak_leg_vel_mmps <= session_limits.leg_vel_mmps
    assert 0.0 < report.peak_leg_acc_mmps2 <= session_limits.leg_acc_mmps2
    assert 0.0 < report.peak_leg_jerk_mmps3 <= session_limits.leg_jerk_mmps3
    assert (feas.HAND_STROKE_MIN_REV <= report.peak_hand_rev
            <= feas.HAND_STROKE_MAX_REV)


def test_the_clean_cycle_is_gated_on_every_channel_it_uses(geom):
    """Tightening ANY one limit onto the clean cycle's own peak refuses it.

    A gate that measures a channel but never compares it is indistinguishable
    from one that does, until the day it matters.  Each limit is set to 90 % of
    the peak this plan actually reaches, and each must produce its own code.
    """
    _cup, _realized, cyc = _clean_cycle()
    base = feas.validate_cycle(cyc, _limits(), geom)
    assert base.ok is True

    tight_vel = _limits(vel=0.9 * base.peak_leg_vel_mmps)
    assert feas.validate_cycle(cyc, tight_vel, geom).code == feas.LIMIT_VEL

    tight_acc = _limits(acc=0.9 * base.peak_leg_acc_mmps2)
    assert feas.validate_cycle(cyc, tight_acc, geom).code == feas.LIMIT_ACC

    tight_jerk = _limits(jerk=0.9 * base.peak_leg_jerk_mmps3)
    assert feas.validate_cycle(cyc, tight_jerk, geom).code == feas.LIMIT_JERK

    tight_step = _limits(step=base.peak_step_rev / feas.STEP_BOUND_MARGIN * 0.9)
    assert feas.validate_cycle(cyc, tight_step, geom).code == feas.STEP_BOUND

    tight_hv = _limits(hand_vel=0.9 * base.peak_hand_vel_rps)
    assert feas.validate_cycle(cyc, tight_hv, geom).code == feas.HAND_LIMIT_VEL

    tight_ha = _limits(hand_acc=0.9 * base.peak_hand_acc_rps2)
    assert feas.validate_cycle(cyc, tight_ha, geom).code == feas.HAND_LIMIT_ACC


def test_the_default_cup_z_box_saturates_the_slider_and_is_refused(geom,
                                                                   session_limits):
    """``cup_cycle``'s default z box is NOT the slider's reach — and it shows.

    ``CupCycleConfig``'s ``z_min_m`` / ``z_max_m`` default to 0.45 / 1.10 m, which
    its own docstring flags as "the caller MUST override this to the SLIDER's
    reachable range".  Left at the defaults, ``decompose`` clamps 31 of 41 knots
    and the gate refuses with ``HAND_STROKE`` — the loud outcome ``cup_realize``'s
    docstring promises ("an unreachable z leaves here and is refused there,
    loudly, rather than being silently trimmed").
    """
    period_s, flight_s, amp = 1.0, 0.8, 0.10
    throw_pos = np.array([amp, 0.4 * amp, 0.80])
    throw_target = np.array([-amp, -0.4 * amp, 0.80])
    v_take = cc.takeoff_velocity(throw_pos, throw_target, flight_s)
    cup = cc.plan_cup_cycle(
        throw_pos, v_take, cc.GRAVITY, throw_pos, throw_target, flight_s,
        period_s * 0.55, np.array([-0.9 * amp, -0.5 * amp, 0.78]),
        np.array([0.10, -0.05, -4.5]), period_s,
        cfg=cc.CupCycleConfig(),                     # the DEFAULT z box
        detach_axis=v_take / np.linalg.norm(v_take))
    recv = np.array(tg.tilt_to_receive(np.array([0.10, -0.05, -4.5])))
    thr = np.array(tg.tilt_to_throw(v_take))
    rcfg = cr.RealizeConfig()
    realized = cr.decompose(cup, cr.tilt_schedule(cup, recv, thr, rcfg), rcfg)

    assert realized.slider_saturated.sum() > 0
    report = feas.validate_cycle(CyclePlan.from_realized(realized),
                                 session_limits, geom)
    assert report.code == feas.HAND_STROKE
