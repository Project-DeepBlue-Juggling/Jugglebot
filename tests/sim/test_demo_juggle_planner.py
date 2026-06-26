"""Unit tests for the online per-throw cup planner (controller/demo/juggle_planner).

The planner replaces the offline juggle_optimizer + open-loop TrajectoryPlayer
with a per-cycle task-space NLP re-solved against the observed ball (Kai-style).
These tests exercise the ballistics helpers and the NLP's hard throw / position
constraints. Plan reference: bb-led-two-ball-juggle-demo.md (Sim2Real, online
re-planning); logbook 2026-06-27-online-replanning-architecture-and-cup-bandlimit
(documents this module) + its predecessor
2026-06-27-throw-aim-band-limit-and-closed-loop-catch.
"""
import numpy as np
import pytest

from controller.demo.juggle_planner import (
    GRAVITY, PlannerConfig, ballistic_touchdown, takeoff_velocity, plan_cup_cycle,
)


def test_ballistic_touchdown_descending_root():
    """A ball launched straight up returns to its launch height symmetrically."""
    pos = np.array([0.0, 0.0, 1.0])
    vel = np.array([0.0, 0.0, 5.0])           # up at 5 m/s
    t_td, pos_td, vel_td = ballistic_touchdown(pos, vel, 1.0)
    # time to return to z=1.0 is 2*vz/g
    assert t_td == pytest.approx(2 * 5.0 / 9.806, abs=1e-6)
    np.testing.assert_allclose(pos_td[:2], [0.0, 0.0], atol=1e-9)
    assert pos_td[2] == pytest.approx(1.0, abs=1e-6)
    assert vel_td[2] == pytest.approx(-5.0, abs=1e-6)   # symmetric descent


def test_takeoff_velocity_lands_on_target():
    """Propagating the takeoff velocity ballistically lands on the target."""
    throw = np.array([0.1, 0.0, 0.75])
    target = np.array([-0.1, 0.0, 0.67])
    T = 1.03
    v = takeoff_velocity(throw, target, T)
    landing = throw + v * T + 0.5 * GRAVITY * T**2
    np.testing.assert_allclose(landing, target, atol=1e-9)


@pytest.fixture(scope="module")
def plan():
    throw_pos = np.array([0.10, 0.0, 0.80])
    throw_target = np.array([-0.10, 0.0, 0.80])
    flight, period = 1.03, 0.625
    catch_pos = np.array([-0.10, 0.0, 0.80])
    v_take = takeoff_velocity(throw_pos, throw_target, flight)
    catch_vel = v_take + GRAVITY * flight
    return plan_cup_cycle(
        pos0=throw_pos, vel0=v_take, acc0=GRAVITY.copy(),
        throw_pos=throw_pos, throw_target=throw_target, flight_s=flight,
        catch_time_s=0.405, catch_pos=catch_pos, catch_vel=catch_vel,
        period_s=period, cfg=PlannerConfig())


def test_planner_solves_and_returns_grid(plan):
    n = plan.jerk.shape[0]
    assert plan.pos.shape == (n + 1, 3)
    assert plan.vel.shape == (n + 1, 3)
    assert plan.acc.shape == (n + 1, 3)
    assert np.all(np.isfinite(plan.pos))


def test_throw_constraints_are_exact(plan):
    """End-of-cycle throw: position, takeoff velocity, and acc==gravity."""
    throw_pos = np.array([0.10, 0.0, 0.80])
    throw_target = np.array([-0.10, 0.0, 0.80])
    v_take = takeoff_velocity(throw_pos, throw_target, 1.03)
    np.testing.assert_allclose(plan.pos[-1], throw_pos, atol=1e-6)
    np.testing.assert_allclose(plan.vel[-1], v_take, atol=1e-6)
    np.testing.assert_allclose(plan.acc[-1], GRAVITY, atol=1e-6)


def test_catch_position_matched_at_interpolated_time(plan):
    """Cup is at the catch position at the (interpolated) touch-down time."""
    pc, _, _ = plan.sample(0.405)
    np.testing.assert_allclose(pc, [-0.10, 0.0, 0.80], atol=1e-4)


def test_jerk_within_per_axis_bounds(plan):
    """Lateral (platform) jerk respects its modest bound; vertical (slider) its
    high bound — the band-limit-aware split."""
    cfg = PlannerConfig()
    # +1e-2 absorbs IPOPT's constraint tolerance (it may return 300.0000003).
    assert np.abs(plan.jerk[:, :2]).max() <= cfg.max_jerk_xy + 1e-2
    assert np.abs(plan.jerk[:, 2]).max() <= cfg.max_jerk_z + 1e-2


def test_detach_no_lateral_acceleration_just_after_release(plan):
    """The first knots after the cycle start (= just after the previous throw)
    have zero lateral acceleration — clean detach up the throw axis."""
    cfg = PlannerConfig()
    for i in range(1, cfg.n_detach + 1):
        np.testing.assert_allclose(plan.acc[i, :2], [0.0, 0.0], atol=1e-5)
