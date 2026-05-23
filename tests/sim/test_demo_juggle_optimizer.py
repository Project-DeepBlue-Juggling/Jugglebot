"""Unit tests for the juggling-demo offline trajectory optimiser (Phase 2).

Covers controller/demo/juggle_optimizer.py — the periodic CasADi NLP that
produces the throw/catch-feasible platform trajectory the runtime player
loads. Each unit test exercises one constraint or invariant; the
solver-converges fixture is session-scoped so the full file runs the
optimiser only twice (default config + a custom-catch-offset variant).

Plan reference: plans/active/bb-led-two-ball-juggle-demo.md §4 Phase 2
and the test cases T-U3 (throw constraint) / T-U4 (catch constraint) of §5.
"""
import numpy as np
import pytest

from controller.demo.pattern import JugglePattern
from controller.demo.trajectory import JuggleTrajectory, build_analytic_oval
from controller.demo.juggle_optimizer import (
    OptimizerConfig, OptimizerResult,
    optimise_juggle_trajectory,
    save_optimised_trajectory,
)


# --------------------------------------------------------------------------
# Shared fixtures — solving the NLP once and sharing the result across
# tests keeps the file under a second of CasADi/IPOPT setup overhead.
# --------------------------------------------------------------------------
@pytest.fixture(scope="module")
def pattern() -> JugglePattern:
    return JugglePattern()


@pytest.fixture(scope="module")
def result_default(pattern) -> OptimizerResult:
    """Optimiser solution with default config (level-locked, 16 knots)."""
    return optimise_juggle_trajectory(pattern, OptimizerConfig(n_knots=16,
                                                               n_samples=128))


# --------------------------------------------------------------------------
# Convergence + result shape
# --------------------------------------------------------------------------
def test_optimiser_converges(result_default):
    """IPOPT reports a finite objective and a bounded iteration count.

    The lower bound on iterations is intentionally zero — a future
    warm-start that happens to land on the optimum should not flake
    this test; the upper bound is what catches runaway non-convergence.
    """
    assert np.isfinite(result_default.objective_value)
    assert result_default.objective_value > 0.0
    assert 0 <= result_default.n_iter <= 200


def test_result_arrays_have_expected_shapes(result_default):
    assert result_default.knot_poses.shape == (16, 6)
    assert result_default.knot_twists.shape == (16, 6)
    assert result_default.knot_accels.shape == (16, 6)
    assert isinstance(result_default.trajectory, JuggleTrajectory)
    assert result_default.trajectory.poses.shape == (128, 6)


def test_catch_offset_default_equals_flight_minus_period(pattern, result_default):
    """The default catch offset matches the ballistic ``throw[k] -> catch[k+1]`` rule."""
    expected = pattern.flight_time_s - pattern.platform_period_s
    assert result_default.catch_offset_s == pytest.approx(expected, abs=1e-9)


# --------------------------------------------------------------------------
# Keyframe constraints — T-U3 (throw) and T-U4 (catch)
# --------------------------------------------------------------------------
def test_throw_keyframe_pose_is_throw_pose(pattern, result_default):
    """At t=0 the optimised trajectory passes through ``pattern.throw_pose``."""
    pose, _, _ = result_default.trajectory.eval(0.0)
    np.testing.assert_allclose(pose, pattern.throw_pose, atol=1e-6)


def test_throw_keyframe_twist_matches_ballistic_launch(pattern, result_default):
    """At t=0 the platform's xy twist matches the ball's required launch velocity."""
    _, twist, _ = result_default.trajectory.eval(0.0)
    expected_vx = (pattern.catch_pose[0] - pattern.throw_pose[0]) / pattern.flight_time_s
    expected_vy = (pattern.catch_pose[1] - pattern.throw_pose[1]) / pattern.flight_time_s
    assert twist[0] == pytest.approx(expected_vx, abs=1e-6)
    assert twist[1] == pytest.approx(expected_vy, abs=1e-6)
    assert twist[2] == pytest.approx(0.0, abs=1e-6)


def test_catch_keyframe_pose_is_catch_pose(pattern, result_default):
    """At t=catch_offset the optimised trajectory passes through ``pattern.catch_pose``."""
    pose, _, _ = result_default.trajectory.eval(result_default.catch_offset_s)
    np.testing.assert_allclose(pose, pattern.catch_pose, atol=1e-3)


def test_catch_keyframe_twist_matches_arrival_velocity(pattern, result_default):
    """Matched-catch: platform xy twist at catch equals ball arrival velocity.

    No horizontal acceleration during free flight, so arrival xy velocity
    equals launch xy velocity.
    """
    _, twist, _ = result_default.trajectory.eval(result_default.catch_offset_s)
    expected_vx = (pattern.catch_pose[0] - pattern.throw_pose[0]) / pattern.flight_time_s
    expected_vy = (pattern.catch_pose[1] - pattern.throw_pose[1]) / pattern.flight_time_s
    assert twist[0] == pytest.approx(expected_vx, abs=1e-3)
    assert twist[1] == pytest.approx(expected_vy, abs=1e-3)
    assert twist[2] == pytest.approx(0.0, abs=1e-3)


# --------------------------------------------------------------------------
# Structural invariants
# --------------------------------------------------------------------------
def test_trajectory_is_periodic(pattern, result_default):
    """The wrap-around segment yields C2 continuity at the period boundary."""
    P = result_default.trajectory.period_s
    p0, v0, a0 = result_default.trajectory.eval(0.0)
    pP, vP, aP = result_default.trajectory.eval(P)
    np.testing.assert_allclose(p0, pP, atol=1e-9)
    np.testing.assert_allclose(v0, vP, atol=1e-9)
    np.testing.assert_allclose(a0, aP, atol=1e-9)


def test_level_locked_orientation_is_zero_everywhere(result_default):
    """``bank_locked_level=True`` zeros rx, ry, rz and their twist/accel components."""
    np.testing.assert_allclose(result_default.knot_poses[:, 3:6], 0.0, atol=1e-9)
    np.testing.assert_allclose(result_default.knot_twists[:, 3:6], 0.0, atol=1e-9)
    np.testing.assert_allclose(result_default.knot_accels[:, 3:6], 0.0, atol=1e-9)
    # Resampled trajectory inherits the level lock.
    np.testing.assert_allclose(result_default.trajectory.poses[:, 3:6], 0.0, atol=1e-9)


def test_workspace_bounds_respected_at_every_knot(result_default):
    """Every knot pose sits inside the configured xy / z box."""
    poses = result_default.knot_poses
    assert np.all(np.abs(poses[:, 0]) <= 200.0 + 1e-6)
    assert np.all(np.abs(poses[:, 1]) <= 200.0 + 1e-6)
    assert np.all((poses[:, 2] >= 0.0 - 1e-6) & (poses[:, 2] <= 300.0 + 1e-6))


def test_optimised_objective_is_within_expected_magnitude(result_default):
    """Regression sanity check on the integrated pose-jerk objective.

    Not a guaranteed lower-bound vs another trajectory — the optimiser
    minimises over a smaller feasible set (throw + catch + periodicity +
    box) than any arbitrary candidate lives in, so an apples-to-apples
    upper bound is hard to construct without resolving the NLP with a
    different objective. Instead, pin the result against a generous
    empirical ceiling: on 2026-05-23 the default-config solve produces
    ``objective_value ≈ 7.8e9`` (mm^2 / s^5, scale set by the analytical
    quintic-jerk integral and the ~100 mm horizontal stroke at the
    Phase 1 tempo). A regression that pushes this above 1e11 indicates
    a real degradation worth investigating.
    """
    assert 0.0 < result_default.objective_value < 1.0e11, (
        f"optimised cost {result_default.objective_value:.3e} is outside the "
        f"expected (0, 1e11) range — investigate whether the NLP setup or the "
        f"warm start has regressed.")


# --------------------------------------------------------------------------
# Custom catch_offset
# --------------------------------------------------------------------------
def test_custom_catch_offset_is_honoured(pattern):
    """A non-default catch_offset_s is solved for and reported."""
    P = pattern.platform_period_s
    custom = 0.5 * P
    res = optimise_juggle_trajectory(
        pattern, OptimizerConfig(n_knots=12, n_samples=64,
                                 catch_offset_s=custom))
    assert res.catch_offset_s == pytest.approx(custom)
    pose, _, _ = res.trajectory.eval(custom)
    np.testing.assert_allclose(pose, pattern.catch_pose, atol=1e-3)


# --------------------------------------------------------------------------
# .npz save / load
# --------------------------------------------------------------------------
def test_save_optimised_trajectory_writes_npz_loadable_by_juggle_trajectory(
        result_default, tmp_path):
    """The saved .npz round-trips through :meth:`JuggleTrajectory.load`."""
    path = tmp_path / "demo" / "juggle.npz"
    save_optimised_trajectory(result_default, path)
    assert path.exists()
    loaded = JuggleTrajectory.load(path)
    # Same period, same first/last samples.
    assert loaded.period_s == pytest.approx(result_default.trajectory.period_s)
    for t in (0.0, 0.2, 0.4, 0.5):
        for a, b in zip(loaded.eval(t), result_default.trajectory.eval(t)):
            np.testing.assert_allclose(a, b, atol=1e-12)


# --------------------------------------------------------------------------
# Input validation
# --------------------------------------------------------------------------
def test_invalid_n_knots_raises():
    p = JugglePattern()
    with pytest.raises(ValueError):
        optimise_juggle_trajectory(p, OptimizerConfig(n_knots=3))


def test_invalid_n_samples_raises():
    p = JugglePattern()
    with pytest.raises(ValueError):
        optimise_juggle_trajectory(p, OptimizerConfig(n_knots=16, n_samples=8))


def test_out_of_range_catch_offset_raises():
    p = JugglePattern()
    P = p.platform_period_s
    with pytest.raises(ValueError, match="catch_offset"):
        optimise_juggle_trajectory(p, OptimizerConfig(catch_offset_s=0.0))
    with pytest.raises(ValueError, match="catch_offset"):
        optimise_juggle_trajectory(p, OptimizerConfig(catch_offset_s=-0.1))
    with pytest.raises(ValueError, match="catch_offset"):
        optimise_juggle_trajectory(p, OptimizerConfig(catch_offset_s=P + 0.1))


# --------------------------------------------------------------------------
# Integration with TrajectoryPlayer
# --------------------------------------------------------------------------
def test_optimised_trajectory_drives_trajectory_player(result_default):
    """The output is a drop-in replacement for ``build_analytic_oval``."""
    from controller.demo.player import TrajectoryPlayer
    from jugglebot.motion.geometry import StewartGeometry
    player = TrajectoryPlayer(result_default.trajectory, geom=StewartGeometry())
    # Sample across the period; every command stays finite and the
    # extensions stay inside the leg stroke.
    period = result_default.trajectory.period_s
    geom = StewartGeometry()
    for t in np.linspace(0.0, period, 20, endpoint=False):
        cmd = player.command_at(t)
        assert np.all(np.isfinite(cmd.ext_mm))
        assert np.all(cmd.ext_mm > -1.0)
        assert np.all(cmd.ext_mm < geom.leg_stroke_mm + 1.0)
