"""BUG-159: the Banking tilt schedule scores the curve the plan gate reads.

The widen loop of ``cup_realize._accel_bounded_schedule`` used to bound the tilt's
Leg jerk on Knot THIRD differences, while ``validate_cycle`` differences the
acceleration of the cubic Hermite ``decompose`` emits: span jerk 3x the third
difference (central-difference Knot rates) plus every Knot's acceleration jump
over the gate's sub-step.  At the live Leg limits (600 / 8000 / 200000) a REST
from a moving state planned 211 669 mm/s^3 against 200 000 and the gate refused
it (LIMIT_JERK).  The loop now scores ``_tilt_realised_leg_jerk_mmps3``: the
tilt-attributable pose built as ``decompose`` builds it, read through
``hermite_jerk_reads`` at the gate's sub-step, through the Leg Jacobian, times
the gate's margin.

Shown to fail without the change (2026-11-28, this file copied into a worktree
of origin/main c4a3943b): the REST test refuses LIMIT_JERK at 211 669 mm/s^3;
the other four fail on the missing functions.  Mutations on this tree
(monkeypatched, same day): the score without the Knot acceleration jumps fails
the agreement test; the score replaced by the third difference through the
static lever makes the REST test refuse LIMIT_JERK at 211 669 again; the score
handed the wrong knot-0 rate (``seed_at_rest`` inverted) fails the agreement
test.
"""
from __future__ import annotations

import types

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory import feasibility as fz
from jugglebot.motion.trajectory.cycle_plan import CyclePlan
from jugglebot.motion.trajectory.limits import TrajectoryLimits


@pytest.fixture(scope='module')
def geom():
    return StewartGeometry()


@pytest.fixture(scope='module')
def live_limits():
    """The robot's launch working point (the sim gate's default since 2026-11-28)."""
    return TrajectoryLimits.from_config(hw).with_sitting_limits(
        leg_vel_mmps=600.0, leg_acc_mmps2=8000.0, leg_jerk_mmps3=200000.0)


def _self_throw_seed0_closing_rest():
    """The closing REST of the sim gate's unbiased self-Throw run, seed 0, Attempt 1.

    Captured 2026-11-28 on origin/main c4a3943b at the live limits (the
    ``plan_cycle('settle', ...)`` call ``SkillsGate.run_self_toss_seed(0)`` made):
    0.332 s from a seed moving at 131 mm/s.  Before BUG-159 it planned
    211 669 mm/s^3 on Leg 1 at t = 0.075 s and was refused LIMIT_JERK.
    """
    goals = uc.CycleGoals(period_s=0.331880586896375, banking_enabled=True,
                          holds_ball=True,
                          settle_site_mm=np.array([-50.0, 0.0, 685.5]), ball_id=0)
    state = uc.CycleState(
        pose=np.array([-49.920822114665214, 4.605344989532546, 170.0,
                       -0.00039052450600201693, 9.36880991351021e-06, 0.0]),
        pose_vel=np.array([-2.2571836656127626, -131.26941442427062, 0.0,
                           -0.033688355836893696, 0.0004756699386706515, 0.0]),
        pose_accel=np.zeros(6),
        hand_rev=0.06141171173515307, hand_vel_rps=-0.06082380516804671,
        detach_axis=None, post_release=False,
        cup_pos_mm=np.array([-49.92140953903185, 4.580859103628966,
                             685.4999999999998]),
        cup_vel_mm_s=np.array([-2.2870267314036616, -133.38244781642595,
                               -1.9816537393157752]),
        cup_accel_mm_s2=np.array([26.72371121119597, 1562.7886219555949,
                                  271.76965547888955]))
    return goals, state


def test_a_moving_seed_rest_at_the_live_limits_plans_under_the_leg_jerk_limit(
        geom, live_limits):
    goals, state = _self_throw_seed0_closing_rest()
    assert not uc._seed_is_at_rest(state)          # the case BUG-159 is about
    plan, _ = uc.plan_cycle('settle', goals, state, live_limits, geom)
    rep = fz.validate_cycle(plan, live_limits, geom)
    assert rep.ok, rep.reasons
    # MEASURED 2026-11-28 with the fix: 49.3 % of the limit (was 105.8 %).
    assert rep.peak_leg_jerk_mmps3 <= 0.8 * live_limits.leg_jerk_mmps3


def _stationary_cup(cup_mm, n, dt):
    pos = np.tile(np.asarray(cup_mm, dtype=float) / 1000.0, (n, 1))
    zeros = np.zeros((n, 3))
    return types.SimpleNamespace(pos=pos, vel=zeros, acc=zeros,
                                 jerk=np.zeros((n - 1, 3)),
                                 t=np.arange(n) * dt, dt=dt, catch_k=-1)


@pytest.mark.parametrize('seed_at_rest', [False, True])
def test_the_schedule_score_is_the_gates_read_on_a_tilt_only_plan(
        geom, live_limits, seed_at_rest):
    """The score equals ``validate_cycle``'s peak Leg jerk on a plan whose ONLY
    motion is the tilt, within 3 %.

    The tilt series is the Banking schedule the planner produced for the REST
    above (so it carries the real sawtooth the Knot rates make), replayed on a
    stationary cup, so every bit of Leg jerk the gate reads is tilt-attributable.
    What the score leaves out (``J-dot`` terms, ``J`` evaluated untilted) is the
    tolerance.  MEASURED 2026-11-28: ratios 0.99-1.00.
    """
    goals, state = _self_throw_seed0_closing_rest()
    _, meta = uc.plan_cycle('settle', goals, state, live_limits, geom)
    tilts = np.asarray(meta.tilts, dtype=float)
    n, dt = tilts.shape[0], float(live_limits.knot_dt_s)
    cfg = cr.RealizeConfig()
    cup = _stationary_cup(meta.cup_plan.pos[0] * 1000.0, n, dt)
    realized = cr.decompose(cup, tilts, cfg, seed_at_rest=seed_at_rest)
    gate = fz.validate_cycle(CyclePlan.from_realized(realized), live_limits,
                             geom).peak_leg_jerk_mmps3
    score = cr._tilt_realised_leg_jerk_mmps3(
        tilts, dt, cr._tilt_realised_terms(cup, n, cfg, geom), seed_at_rest)
    assert gate > 1000.0                           # non-vacuous: the tilt moves
    assert score == pytest.approx(gate, rel=0.03)


def test_the_score_reads_the_gates_sub_step_and_margin_not_copies():
    """The sub-step count and the margin come FROM ``feasibility``: change the
    gate's and the score follows.  A retyped copy would not, and that drift is
    the BUG-159 mismatch again."""
    assert cr._gate_jerk_read_constants() == (fz._CYCLE_SAMPLES_PER_KNOT,
                                              fz._VALIDATE_JERK_MARGIN)
    n, dt = 24, 0.025
    k = np.arange(n, dtype=float)
    tilts = np.column_stack([0.04 * np.sin(k / 3.0), 0.03 * np.cos(k / 4.0)])
    cfg = cr.RealizeConfig()
    terms = cr._tilt_realised_terms(_stationary_cup([0.0, 0.0, 700.0], n, dt),
                                    n, cfg)
    base = cr._tilt_realised_leg_jerk_mmps3(tilts, dt, terms)
    mp = pytest.MonkeyPatch()
    try:
        mp.setattr(fz, '_VALIDATE_JERK_MARGIN', 2.0 * fz._VALIDATE_JERK_MARGIN)
        assert cr._tilt_realised_leg_jerk_mmps3(tilts, dt, terms) == pytest.approx(
            2.0 * base, rel=1e-12)
    finally:
        mp.undo()
    try:
        mp.setattr(fz, '_CYCLE_SAMPLES_PER_KNOT', 2 * fz._CYCLE_SAMPLES_PER_KNOT)
        assert cr._tilt_realised_leg_jerk_mmps3(tilts, dt, terms) != pytest.approx(
            base, rel=1e-3)
    finally:
        mp.undo()


def test_hermite_jerk_reads_is_three_third_differences_inside_and_more_at_knots():
    """The mechanism, pinned: with central-difference Knot rates an interior
    span's jerk is exactly 3 x the third difference / dt^3, and the reads at a
    Knot add its acceleration jump over the sub-step on top."""
    dt, m = 0.025, 4
    k = np.arange(12, dtype=float)
    x = (0.02 * np.sin(k / 2.0))[:, None]
    span, into, out = cr.hermite_jerk_reads(x, cr._knot_derivative(x, dt), dt, m)
    d3 = np.diff(x, n=3, axis=0) / dt ** 3                  # spans k..k+3
    np.testing.assert_allclose(span[1:-1], 3.0 * d3, rtol=1e-9, atol=1e-6)
    assert into.shape == out.shape == (10, 1)
    assert np.max(np.abs(into)) > np.max(np.abs(span))


def test_the_all_over_budget_pick_sees_the_whole_series_not_only_its_peak():
    """BUG-159: when no widen attempt is inside the budget the least-bad pick
    must weigh every read.  Here the narrowest blend has the (marginally) lowest
    worst read but 4x the summed excess of the widest: the widest is the pick.
    The old rule (smallest worst read) returned the narrowest, which on a held
    hop Throw read 134k mm/s^3 before the release against 33k on main."""
    narrow, mid, wide = object(), object(), object()
    tried = [(narrow, 1.917, 8.9), (mid, 1.940, 4.0), (wide, 1.974, 1.9)]
    assert cr._least_bad_attempt(tried) is wide
    # An exact tie in excess goes to the later, wider attempt.
    assert cr._least_bad_attempt([(narrow, 1.5, 2.0), (wide, 1.6, 2.0)]) is wide
    # A single attempt is returned as is.
    assert cr._least_bad_attempt([(narrow, 3.0, 9.0)]) is narrow
