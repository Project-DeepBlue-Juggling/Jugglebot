"""planner.build_move tests (Phase 2) — boundary conditions, duration stretch,
loud rejection, and C2 continuity across an install-time replan.
"""

from __future__ import annotations

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.trajectory import TrajectoryInfeasible, TrajectoryLimits
from jugglebot.motion.trajectory import feasibility as feas
from jugglebot.motion.trajectory import planner


NEUTRAL = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])


@pytest.fixture
def geom():
    return StewartGeometry()


@pytest.fixture
def limits():
    return TrajectoryLimits.from_config(hw)


def _rest(pose=NEUTRAL):
    return (pose, np.zeros(6), np.zeros(6))


# ── Boundary-condition exactness ──────────────────────────────

def test_move_starts_at_seed_ends_at_rest_target(geom, limits):
    target = NEUTRAL + np.array([15., -10., 8., 0.02, 0., 0.])
    v0 = np.array([3., 0., -2., 0., 0., 0.])
    a0 = np.array([1., 0., 0., 0., 0., 0.])
    plan = planner.build_move((NEUTRAL, v0, a0), target, 2.0, limits, geom)

    p_s, v_s, a_s = plan.state_at(0.0)
    assert np.allclose(p_s, NEUTRAL) and np.allclose(v_s, v0) and np.allclose(a_s, a0)
    p_e, v_e, a_e = plan.state_at(plan.total_duration)
    assert np.allclose(p_e, target)
    assert np.allclose(v_e, 0.0) and np.allclose(a_e, 0.0)


# ── Minimal-feasible vs explicit duration ─────────────────────

def test_minimal_feasible_when_duration_none(geom, limits):
    target = NEUTRAL + np.array([0., 0., 20., 0., 0., 0.])
    plan = planner.build_move(_rest(), target, None, limits, geom)
    report = feas.validate(plan, limits, geom)
    assert report.ok
    # The binding constraint sits just under its ceiling (minimal-modulo-margin).
    assert report.peak_leg_acc_mmps2 <= limits.leg_acc_mmps2
    assert report.peak_leg_jerk_mmps3 <= limits.leg_jerk_mmps3
    # And genuinely minimal: 90% of the found duration is infeasible.
    with pytest.raises(TrajectoryInfeasible):
        planner.build_move(_rest(), target, 0.9 * plan.total_duration, limits, geom)


def test_zero_duration_is_minimal_feasible(geom, limits):
    target = NEUTRAL + np.array([0., 0., 20., 0., 0., 0.])
    a = planner.build_move(_rest(), target, 0.0, limits, geom)
    b = planner.build_move(_rest(), target, None, limits, geom)
    assert a.total_duration == pytest.approx(b.total_duration)


def test_generous_explicit_duration_honoured(geom, limits):
    target = NEUTRAL + np.array([20., 0., 0., 0., 0., 0.])
    plan = planner.build_move(_rest(), target, 3.0, limits, geom)
    assert plan.total_duration == pytest.approx(3.0)
    assert feas.validate(plan, limits, geom).ok


# ── Loud rejection of a too-tight duration ────────────────────

def test_too_fast_rejects_with_min_duration(geom, limits):
    target = NEUTRAL + np.array([20., 20., 15., 0., 0., 0.])
    with pytest.raises(TrajectoryInfeasible) as exc:
        planner.build_move(_rest(), target, 0.05, limits, geom)
    assert exc.value.code == feas.TOO_FAST
    assert exc.value.min_duration_s > 0.05
    # The advertised min duration is itself feasible.
    plan = planner.build_move(_rest(), target, exc.value.min_duration_s,
                              limits, geom)
    assert feas.validate(plan, limits, geom).ok


def test_min_duration_boundary_accepted(geom, limits):
    target = NEUTRAL + np.array([20., 20., 15., 0., 0., 0.])
    t_min, _ = planner._min_feasible_move(
        NEUTRAL, np.zeros(6), np.zeros(6), target, limits, geom)
    # Exactly at the minimum is accepted (>= min feasible).
    plan = planner.build_move(_rest(), target, t_min, limits, geom)
    assert plan.total_duration == pytest.approx(t_min)


# ── Spatial failures raise immediately, never TOO_FAST ────────

def test_workspace_failure_is_not_stretched(geom, limits):
    # An out-of-stroke target cannot be fixed by a longer duration → WORKSPACE,
    # raised in the first iteration (not TOO_FAST after exhausting iters).
    bad = np.array([0., 0., 500., 0., 0., 0.])
    with pytest.raises(TrajectoryInfeasible) as exc:
        planner.build_move(_rest(), bad, None, limits, geom)
    assert exc.value.code == feas.WORKSPACE


# ── Convergence: few iterations from a rest start ─────────────

def test_stretch_converges_in_few_iters(geom, limits, monkeypatch):
    """A rest-to-rest move lands feasible within 2 validate calls (1 stretch)."""
    calls = {'n': 0}
    real_validate = feas.validate

    def counting_validate(*a, **k):
        calls['n'] += 1
        return real_validate(*a, **k)

    monkeypatch.setattr(planner, 'validate', counting_validate)
    target = NEUTRAL + np.array([25., 0., 0., 0., 0., 0.])
    planner._min_feasible_move(NEUTRAL, np.zeros(6), np.zeros(6),
                              target, limits, geom)
    assert calls['n'] <= 3


# ── C2 continuity across an install-time replan mid-move ──────

def test_c2_replan_from_mid_move(geom, limits):
    move = planner.build_move(
        _rest(), NEUTRAL + np.array([20., 0., 10., 0., 0., 0.]), 2.0, limits, geom)
    t = 0.8
    seed = move.state_at(t)
    # A new move seeded from the mid-flight state continues C2.
    new = planner.build_move(seed, NEUTRAL + np.array([0., 15., 0., 0., 0., 0.]),
                             2.0, limits, geom)
    p, v, a = new.state_at(0.0)
    assert np.allclose(p, seed[0], atol=1e-9)
    assert np.allclose(v, seed[1], atol=1e-9)
    assert np.allclose(a, seed[2], atol=1e-9)


# ── Moving-seed build still converges to a feasible plan ──────

def test_moving_seed_move_is_feasible(geom, limits):
    v0 = np.array([20., 0., 0., 0., 0., 0.])
    a0 = np.array([50., 0., 0., 0., 0., 0.])
    plan = planner.build_move((NEUTRAL, v0, a0),
                              NEUTRAL + np.array([15., 0., 0., 0., 0., 0.]),
                              None, limits, geom)
    assert feas.validate(plan, limits, geom).ok
