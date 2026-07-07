"""Segment / plan / planner tests (Phase 1).

Covers the C2 spine of the trajectory generator: quintic boundary-condition
exactness, C2 continuity across an install-time replan, the ``HoldPlan``
degenerate case, and the planner's loud-rejection contract (infeasible requests
raise ``TrajectoryInfeasible`` with a machine code, never silently succeed).
"""

from __future__ import annotations

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.trajectory import (
    HoldPlan, QuinticSegment, TrajectoryInfeasible, TrajectoryLimits,
)
from jugglebot.motion.trajectory import feasibility as feas
from jugglebot.motion.trajectory import planner
from jugglebot.motion.trajectory.plan import TrajectoryPlan


NEUTRAL = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])


@pytest.fixture
def geom():
    return StewartGeometry()


@pytest.fixture
def limits():
    return TrajectoryLimits.from_config(hw)


# ── Segment boundary-condition exactness ──────────────────────

def test_segment_matches_boundary_conditions():
    p0 = np.array([0., 0., 170., 0., 0., 0.])
    v0 = np.array([5., -3., 2., 0., 0., 0.])
    a0 = np.array([1., 0., -1., 0., 0., 0.])
    p1 = np.array([10., 5., 175., 0.01, 0., 0.])
    v1 = np.zeros(6)
    a1 = np.zeros(6)
    seg = QuinticSegment(p0, v0, a0, p1, v1, a1, duration=1.5)

    pose_s, twist_s, accel_s = seg.eval(0.0)
    assert np.allclose(pose_s, p0)
    assert np.allclose(twist_s, v0)
    assert np.allclose(accel_s, a0)

    pose_e, twist_e, accel_e = seg.eval(1.5)
    assert np.allclose(pose_e, p1)
    assert np.allclose(twist_e, v1)
    assert np.allclose(accel_e, a1)


def test_segment_rejects_nonpositive_duration():
    z = np.zeros(6)
    with pytest.raises(ValueError):
        QuinticSegment(z, z, z, z, z, z, 0.0)


def test_segment_rejects_wrong_shape():
    with pytest.raises(ValueError):
        QuinticSegment(np.zeros(5), np.zeros(6), np.zeros(6),
                       np.zeros(6), np.zeros(6), np.zeros(6), 1.0)


# ── HoldPlan ───────────────────────────────────────────────────

def test_hold_plan_is_constant_and_at_rest():
    plan = HoldPlan(NEUTRAL)
    assert plan.kind == 'hold'
    for t in (-1.0, 0.0, 5.0, 100.0):
        pose, twist, accel = plan.state_at(t)
        assert np.allclose(pose, NEUTRAL)
        assert np.allclose(twist, 0.0)
        assert np.allclose(accel, 0.0)


def test_plan_terminal_hold_is_rest_at_final_pose():
    seg = QuinticSegment(NEUTRAL, np.zeros(6), np.zeros(6),
                         NEUTRAL + np.array([10, 0, 0, 0, 0, 0.]),
                         np.zeros(6), np.zeros(6), duration=1.0)
    plan = TrajectoryPlan((seg,), final_pose=NEUTRAL + np.array([10, 0, 0, 0, 0, 0.]))
    # Past the end: final pose, zero twist/accel, forever.
    pose, twist, accel = plan.state_at(5.0)
    assert np.allclose(pose, NEUTRAL + np.array([10, 0, 0, 0, 0, 0.]))
    assert np.allclose(twist, 0.0)
    assert np.allclose(accel, 0.0)


# ── C2 continuity across an install-time replan ───────────────

def test_c2_continuity_across_replan(geom, limits):
    """A replan seeded from the old plan's state_at(t) is C2 (pos/vel/accel match)."""
    state0 = (NEUTRAL, np.zeros(6), np.zeros(6))
    move = planner.build_return_to_neutral(
        state0, NEUTRAL + np.array([15., 0., 5., 0., 0., 0.]), 2.0, limits, geom)
    # Replan mid-flight from the sampled state.
    t_install = 0.7
    pose_before, twist_before, accel_before = move.state_at(t_install)
    seed = (pose_before, twist_before, accel_before)
    new_plan = planner.build_hold(seed, limits, geom)
    pose_after, twist_after, accel_after = new_plan.state_at(0.0)
    assert np.allclose(pose_after, pose_before, atol=1e-9)
    assert np.allclose(twist_after, twist_before, atol=1e-9)
    assert np.allclose(accel_after, accel_before, atol=1e-9)


# ── Planner: gate passes for feasible, raises for infeasible ──

def test_build_hold_at_rest_returns_holdplan(geom, limits):
    plan = planner.build_hold((NEUTRAL, np.zeros(6), np.zeros(6)), limits, geom)
    assert plan.kind == 'hold'


def test_build_return_to_neutral_feasible(geom, limits):
    plan = planner.build_return_to_neutral(
        (NEUTRAL + np.array([10., 0., 0., 0., 0., 0.]), np.zeros(6), np.zeros(6)),
        NEUTRAL, 2.0, limits, geom)
    assert plan.total_duration >= limits.min_move_duration_s
    # Ends at rest at neutral.
    pose, twist, _ = plan.state_at(plan.total_duration)
    assert np.allclose(pose, NEUTRAL)
    assert np.allclose(twist, 0.0)


def test_too_fast_move_raises_limit(geom, limits):
    with pytest.raises(TrajectoryInfeasible) as exc:
        planner.build_return_to_neutral(
            (NEUTRAL, np.zeros(6), np.zeros(6)),
            NEUTRAL + np.array([80., 0., 0., 0., 0., 0.]), 0.2, limits, geom)
    assert exc.value.code in (feas.LIMIT_VEL, feas.LIMIT_ACC)


def test_out_of_workspace_hold_raises(geom, limits):
    # A pose far outside the stroke → WORKSPACE rejection even for a hold.
    bad = np.array([0.0, 0.0, 500.0, 0.0, 0.0, 0.0])
    with pytest.raises(TrajectoryInfeasible) as exc:
        planner.build_hold((bad, np.zeros(6), np.zeros(6)), limits, geom)
    assert exc.value.code == feas.WORKSPACE
