"""planner.build_timed tests (Phase 5) — timed target states (goal 3).

Covers: arrival boundary conditions (pose + arrival twist exact at t_arrival), the
knot-quantisation bound (≤ one 25 ms knot), rest-termination safety (the plan never
ends with a nonzero velocity — that would be an unbounded-jerk terminal-hold snap),
TOO_FAST / min-lead loud rejection, hold_after (hold-at-target vs return-to-neutral),
and the load-bearing invariant that every emitted knot of a timed plan is accepted
by a real SetpointPump.
"""

from __future__ import annotations

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.trajectory import (
    KnotEmitter,
    TrajectoryInfeasible,
    TrajectoryLimits,
)
from jugglebot.motion.trajectory import feasibility as feas
from jugglebot.motion.trajectory import planner
from teensy_link.setpoint_pump import SetpointPump


NEUTRAL = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])


@pytest.fixture
def geom():
    return StewartGeometry()


@pytest.fixture
def limits():
    return TrajectoryLimits.from_config(hw)


def _rest(pose=NEUTRAL):
    return (pose, np.zeros(6), np.zeros(6))


# ── Arrival boundary conditions ───────────────────────────────

def test_arrival_pose_exact_at_t_arrival(geom, limits):
    """The reach segment END is the target — pose error at t_arrival is zero by
    construction, and the arrival is at the requested lead (not stretched)."""
    target = NEUTRAL + np.array([15., -10., 8., 0.02, 0., 0.])
    D = 3.0
    plan, report = planner.build_timed(_rest(), target, np.zeros(6), D, limits, geom)
    assert report.ok
    p_arr, v_arr, a_arr = plan.state_at(D)
    assert np.allclose(p_arr, target)
    # Zero arrival velocity ⇒ the reach alone (one segment), arrival == plan end.
    assert len(plan.segments) == 1
    assert np.isclose(plan.total_duration, D)


def test_nonzero_arrival_velocity_bc(geom, limits):
    """A nonzero arrival velocity is honoured exactly at t_arrival; the plan then
    appends a decel-to-rest so it still ends stationary (safety)."""
    target = NEUTRAL + np.array([12., 0., 6., 0., 0., 0.])
    v1 = np.array([18., -6., 0., 0., 0., 0.])
    D = 2.5
    plan, report = planner.build_timed(_rest(), target, v1, D, limits, geom,
                                       hold_after=True)
    assert report.ok
    p_arr, v_arr, a_arr = plan.state_at(D)
    assert np.allclose(p_arr, target)
    assert np.allclose(v_arr, v1)            # arrival twist matches the BC
    assert len(plan.segments) >= 2           # reach + decel-to-rest continuation


def test_c2_continuity_at_reach_decel_join(geom, limits):
    """The decel continuation joins the reach C2 (pose/twist/accel continuous)."""
    target = NEUTRAL + np.array([10., 5., 0., 0., 0., 0.])
    v1 = np.array([15., 0., 0., 0., 0., 0.])
    D = 2.5
    plan, _ = planner.build_timed(_rest(), target, v1, D, limits, geom)
    eps = 1e-4
    p_before, v_before, a_before = plan.state_at(D - eps)
    p_after, v_after, a_after = plan.state_at(D + eps)
    assert np.allclose(p_before, p_after, atol=1e-2)
    assert np.allclose(v_before, v_after, atol=1e-1)
    assert np.allclose(a_before, a_after, atol=5.0)


# ── Rest-termination (safety) ─────────────────────────────────

def test_plan_always_ends_at_rest(geom, limits):
    """Every timed plan ends at rest — an implicit terminal hold would snap a
    nonzero end velocity to zero (unbounded jerk). True for both arrival modes."""
    target = NEUTRAL + np.array([10., 0., 5., 0., 0., 0.])
    for v1 in (np.zeros(6), np.array([20., 0., 0., 0., 0., 0.])):
        for hold_after in (True, False):
            plan, _ = planner.build_timed(
                _rest(), target, v1, 3.0, limits, geom,
                hold_after=hold_after, neutral_pose=NEUTRAL)
            _, v_end, a_end = plan.state_at(plan.total_duration)
            assert np.allclose(v_end, 0.0, atol=1e-9)
            assert np.allclose(a_end, 0.0, atol=1e-9)


# ── hold_after semantics ──────────────────────────────────────

def test_hold_after_true_holds_at_target(geom, limits):
    target = NEUTRAL + np.array([14., -8., 10., 0., 0., 0.])
    plan, _ = planner.build_timed(_rest(), target, np.zeros(6), 3.0, limits, geom,
                                  hold_after=True)
    p_end, _, _ = plan.state_at(plan.total_duration)
    assert np.allclose(p_end, target)


def test_hold_after_false_returns_to_neutral(geom, limits):
    target = NEUTRAL + np.array([14., -8., 10., 0., 0., 0.])
    plan, _ = planner.build_timed(_rest(), target, np.zeros(6), 3.0, limits, geom,
                                  hold_after=False, neutral_pose=NEUTRAL)
    p_end, _, _ = plan.state_at(plan.total_duration)
    assert np.allclose(p_end, NEUTRAL, atol=1e-6)
    # The target is still hit exactly at the arrival lead (t=3.0).
    assert np.allclose(plan.state_at(3.0)[0], target, atol=1e-6)


def test_hold_after_false_requires_neutral_pose(geom, limits):
    target = NEUTRAL + np.array([10., 0., 0., 0., 0., 0.])
    with pytest.raises(ValueError):
        planner.build_timed(_rest(), target, np.zeros(6), 3.0, limits, geom,
                            hold_after=False)


# ── Loud rejection ────────────────────────────────────────────

def test_too_tight_lead_rejected_too_fast_with_min_duration(geom, limits):
    target = NEUTRAL + np.array([20., 0., 0., 0., 0., 0.])
    with pytest.raises(TrajectoryInfeasible) as ei:
        planner.build_timed(_rest(), target, np.zeros(6), 0.05, limits, geom)
    assert ei.value.code == feas.TOO_FAST
    assert ei.value.min_duration_s > 0.05
    # The advertised minimum actually gates OK.
    plan, report = planner.build_timed(
        _rest(), target, np.zeros(6), ei.value.min_duration_s, limits, geom)
    assert report.ok


def test_below_min_timed_lead_rejected(geom, limits):
    """A lead under the min_timed_lead floor is rejected even for a tiny move."""
    target = NEUTRAL + np.array([1.0, 0., 0., 0., 0., 0.])
    tight = 0.5 * limits.min_timed_lead_s
    with pytest.raises(TrajectoryInfeasible) as ei:
        planner.build_timed(_rest(), target, np.zeros(6), tight, limits, geom)
    assert ei.value.code == feas.TOO_FAST
    assert ei.value.min_duration_s >= limits.min_timed_lead_s


def test_epoch_magnitude_lead_rejected_no_crash(geom, limits):
    """A clock-domain-confused lead (an absolute epoch timestamp fed as a lead, ~1.75e9 s)
    is loudly rejected BEFORE any segment is built — never a ~7e10-element np.arange /
    MemoryError. Must return promptly (the guard short-circuits before the reach)."""
    target = NEUTRAL + np.array([10., 0., 5., 0., 0., 0.])
    with pytest.raises(TrajectoryInfeasible) as ei:
        planner.build_timed(_rest(), target, np.zeros(6), 1.75e9, limits, geom)
    assert ei.value.code == feas.TOO_FAST
    assert 'maximum timed lead' in ' '.join(ei.value.reasons)


def test_lead_just_over_max_rejected(geom, limits):
    """A finite lead a hair over max_timed_lead_s (61 s vs the 60 s ceiling) is rejected —
    an hour-scale finite lead would otherwise stall the executor ~30 s in the knot loop."""
    target = NEUTRAL + np.array([10., 0., 5., 0., 0., 0.])
    over = limits.max_timed_lead_s + 1.0
    with pytest.raises(TrajectoryInfeasible) as ei:
        planner.build_timed(_rest(), target, np.zeros(6), over, limits, geom)
    assert ei.value.code == feas.TOO_FAST
    assert ei.value.min_duration_s == pytest.approx(limits.max_timed_lead_s)


def test_lead_just_under_max_accepted(geom, limits):
    """The ceiling is not over-eager — a lead just under max_timed_lead_s still builds
    (a small in-stroke target held at the arrival)."""
    target = NEUTRAL + np.array([5., 0., 3., 0., 0., 0.])
    under = limits.max_timed_lead_s - 1.0
    plan, report = planner.build_timed(_rest(), target, np.zeros(6), under, limits, geom)
    assert report.ok
    assert np.allclose(plan.state_at(under)[0], target, atol=1e-6)


def test_min_feasible_lead_advertised_from_moving_seed_is_feasible(geom, limits):
    """A too-tight lead from a MOVING seed (the path _min_feasible_timed hardens against
    non-convergence) still advertises a min_duration_s the fast gate accepts — the
    fall-through validates + stretches the final candidate rather than returning an
    unvalidated guess."""
    moving = (NEUTRAL.copy(), np.array([8., 0., 0., 0., 0., 0.]), np.zeros(6))
    target = NEUTRAL + np.array([25., 0., 0., 0., 0., 0.])
    with pytest.raises(TrajectoryInfeasible) as ei:
        planner.build_timed(moving, target, np.zeros(6), 0.05, limits, geom)
    assert ei.value.code == feas.TOO_FAST
    # The advertised minimum, rebuilt from the same moving seed, gates OK.
    plan, report = planner.build_timed(
        moving, target, np.zeros(6), ei.value.min_duration_s, limits, geom)
    assert report.ok


def test_unreachable_target_rejected_not_stretched(geom, limits):
    """A spatially unreachable target is rejected on its gate code (WORKSPACE/
    UNREACHABLE), never TOO_FAST — a longer lead cannot reach it."""
    target = NEUTRAL + np.array([0., 0., 400., 0., 0., 0.])   # way out of stroke
    with pytest.raises(TrajectoryInfeasible) as ei:
        planner.build_timed(_rest(), target, np.zeros(6), 3.0, limits, geom)
    assert ei.value.code in (feas.WORKSPACE, feas.UNREACHABLE)


# ── Knot quantisation ─────────────────────────────────────────

def test_knot_nearest_arrival_within_one_knot(geom, limits):
    """The emitted knot nearest t_arrival is within one 25 ms knot of the target."""
    target = NEUTRAL + np.array([15., -10., 8., 0.0, 0., 0.])
    D = 2.0
    plan, _ = planner.build_timed(_rest(), target, np.zeros(6), D, limits, geom)
    kdt = limits.knot_dt_s
    # Nearest knot time to the arrival.
    k = round(D / kdt)
    p_knot = plan.state_at(k * kdt)[0]
    # The per-knot pose change bounds the quantisation error; assert the knot lands
    # within one knot's motion of the exact arrival.
    p_prev = plan.state_at((k - 1) * kdt)[0]
    one_knot_move = np.abs(p_knot - p_prev)
    assert np.all(np.abs(p_knot - target) <= one_knot_move + 1e-6)


# ── Production-in-the-loop: every timed knot is pump-accepted ──

def test_every_timed_knot_pump_accepted(geom, limits):
    """The load-bearing invariant, on a timed plan (incl. a nonzero-velocity
    arrival + decel): every emitted frame is accepted by a real SetpointPump."""
    emitter = KnotEmitter(geom, knot_dt_s=hw.JB_TRAJ_KNOT_DT_S)
    pump = SetpointPump(mm_to_rev=hw.GEOM_MM_TO_REV,
                        max_step_rev=hw.JB_OP_MAX_POSITION_STEP_REV)
    target = NEUTRAL + np.array([14., -8., 10., 0.01, 0., 0.])
    v1 = np.array([15., 0., 0., 0., 0., 0.])
    plan, _ = planner.build_timed(_rest(), target, v1, 2.5, limits, geom,
                                  hold_after=True)
    dt = hw.JB_TRAJ_KNOT_DT_S
    n = int(plan.total_duration / dt) + 4
    for seq in range(n):
        frame = emitter.frame(plan, seq * dt, seq)
        sp, reason = pump.build(frame, t_origin_us=seq * 25000)
        assert reason is None, f"pump rejected timed frame at seq={seq}: {reason}"
    assert pump.frames_rejected == 0
