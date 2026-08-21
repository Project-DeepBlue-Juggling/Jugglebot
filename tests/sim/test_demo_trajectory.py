"""Unit tests for the juggling-demo trajectory layer (Phase 2).

Covers sim/juggle_planner/{pattern,trajectory,player}.py — the pure-Python,
offline trajectory tooling for plans/archived/bb-led-two-ball-juggle-demo.md.
No hardware, no MuJoCo (the sim-playback check lives in
test_demo_sim_playback.py).
"""
import numpy as np
import pytest

from sim.juggle_planner.pattern import JugglePattern
from sim.juggle_planner.trajectory import JuggleTrajectory, build_analytic_oval
from sim.juggle_planner.player import TrajectoryPlayer
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import (
    leg_lengths_to_pose, pose_to_leg_lengths, rotvec_to_rot_matrix)


# --------------------------------------------------------------------------
# JugglePattern
# --------------------------------------------------------------------------
def test_pattern_derived_values_match_phase1():
    """Default pattern reproduces the Phase 1 feasibility-study numbers."""
    p = JugglePattern()  # apex 1300 mm
    assert p.throw_speed_mps == pytest.approx(5.05, abs=0.02)
    assert p.flight_time_s == pytest.approx(1.03, abs=0.01)
    assert p.throw_stroke_s == pytest.approx(0.122, abs=0.003)
    assert p.catch_stroke_s == pytest.approx(0.198, abs=0.003)
    assert p.transit_window_s == pytest.approx(0.306, abs=0.005)
    assert p.ball_period_s == pytest.approx(1.25, abs=0.02)
    assert p.platform_period_s == pytest.approx(0.625, abs=0.01)
    # The platform loop is exactly half the ball cycle.
    assert p.platform_period_s == pytest.approx(0.5 * p.ball_period_s)


def test_pattern_keyframes():
    p = JugglePattern(separation_mm=100.0, active_z_mm=170.0)
    np.testing.assert_allclose(p.throw_pose, [50, 0, 170, 0, 0, 0])
    np.testing.assert_allclose(p.catch_pose, [-50, 0, 170, 0, 0, 0])


def test_pattern_infeasible_tempo_raises():
    """A throw too low to close the two-ball tempo is rejected."""
    with pytest.raises(ValueError, match="infeasible tempo"):
        JugglePattern(apex_height_mm=200.0)


def test_pattern_invalid_params_raise():
    with pytest.raises(ValueError):
        JugglePattern(separation_mm=-1.0)
    with pytest.raises(ValueError):
        JugglePattern(apex_height_mm=0.0)


# --------------------------------------------------------------------------
# JuggleTrajectory
# --------------------------------------------------------------------------
def test_trajectory_rejects_bad_shapes():
    good = np.zeros((8, 6))
    with pytest.raises(ValueError):
        JuggleTrajectory(1.0, np.zeros((8, 5)), good, good)
    with pytest.raises(ValueError):
        JuggleTrajectory(1.0, np.zeros((1, 6)), np.zeros((1, 6)),
                         np.zeros((1, 6)))
    with pytest.raises(ValueError):
        JuggleTrajectory(-1.0, good, good, good)


def test_trajectory_is_periodic():
    """eval(t) == eval(t + period) for non-node times that exercise the wrap."""
    traj = build_analytic_oval(JugglePattern())
    T = traj.period_s
    for t in (0.07, 0.31, 0.503):
        for pa, pb in zip(traj.eval(t), traj.eval(t + T)):
            np.testing.assert_allclose(pa, pb, atol=1e-9)
        for pa, pc in zip(traj.eval(t), traj.eval(t + 2 * T)):
            np.testing.assert_allclose(pa, pc, atol=1e-9)


def test_trajectory_reproduces_sample_nodes():
    """At exact node times eval returns the stored pose/twist/accel."""
    traj = build_analytic_oval(JugglePattern(), n_samples=64)
    dt_node = traj.period_s / traj.n_samples
    for k in (0, 1, 17, 63):
        pose, twist, accel = traj.eval(k * dt_node)
        np.testing.assert_allclose(pose, traj.poses[k], atol=1e-9)
        np.testing.assert_allclose(twist, traj.twists[k], atol=1e-9)
        np.testing.assert_allclose(accel, traj.accels[k], atol=1e-9)


def test_trajectory_faithful_to_analytic_oval():
    """The quintic spline reproduces the C-infinity analytic oval between nodes.

    Faithful reproduction of a smooth closed-form function at arbitrary
    (off-node) times is the operational evidence that the interpolated
    trajectory is itself smooth — pose, twist and accel all track the
    closed-form sinusoid, including across segment boundaries.
    """
    p = JugglePattern(separation_mm=100.0, oval_width_mm=30.0)
    traj = build_analytic_oval(p, n_samples=128)
    T = traj.period_s
    w = 2.0 * np.pi / T
    x_amp, y_amp, z = 50.0, 30.0, p.active_z_mm
    for t in np.linspace(0.003, T - 0.003, 37):   # deliberately off-node
        pose, twist, accel = traj.eval(t)
        c, s = np.cos(w * t), np.sin(w * t)
        np.testing.assert_allclose(
            pose, [x_amp * c, y_amp * s, z, 0, 0, 0], atol=2e-2)
        np.testing.assert_allclose(
            twist, [-x_amp * w * s, y_amp * w * c, 0, 0, 0, 0], atol=0.2)
        np.testing.assert_allclose(
            accel, [-x_amp * w * w * c, -y_amp * w * w * s, 0, 0, 0, 0],
            atol=10.0)


def test_analytic_oval_keyframes():
    """The analytic oval passes through the THROW and CATCH keyframes, level."""
    p = JugglePattern(separation_mm=100.0, active_z_mm=170.0,
                      oval_width_mm=30.0)
    traj = build_analytic_oval(p)
    throw_pose, _, _ = traj.eval(0.0)
    catch_pose, _, _ = traj.eval(0.5 * traj.period_s)
    np.testing.assert_allclose(throw_pose[:3], [50, 0, 170], atol=1e-6)
    np.testing.assert_allclose(catch_pose[:3], [-50, 0, 170], atol=1e-6)
    # Level platform throughout: zero orientation at both keyframes.
    np.testing.assert_allclose(throw_pose[3:], 0.0, atol=1e-9)
    np.testing.assert_allclose(catch_pose[3:], 0.0, atol=1e-9)


def test_trajectory_save_load_roundtrip(tmp_path):
    traj = build_analytic_oval(JugglePattern())
    path = tmp_path / "traj.npz"
    traj.save(path)
    loaded = JuggleTrajectory.load(path)
    assert loaded.period_s == pytest.approx(traj.period_s)
    for t in (0.0, 0.13, 0.4):
        for a, b in zip(traj.eval(t), loaded.eval(t)):
            np.testing.assert_allclose(a, b, atol=1e-12)


# --------------------------------------------------------------------------
# TrajectoryPlayer
# --------------------------------------------------------------------------
def test_player_extensions_within_stroke():
    """Every commanded extension stays inside the physical leg stroke."""
    geom = StewartGeometry()
    player = TrajectoryPlayer(build_analytic_oval(JugglePattern()), geom=geom)
    traj_period = build_analytic_oval(JugglePattern()).period_s
    for t in np.linspace(0.0, traj_period, 40, endpoint=False):
        cmd = player.command_at(t)
        assert cmd.ext_mm.shape == (6,)
        assert np.all(np.isfinite(cmd.ext_mm))
        assert np.all(cmd.ext_mm > -1.0)
        assert np.all(cmd.ext_mm < geom.leg_stroke_mm + 1.0)


def test_player_lookahead_is_exact():
    """ext_next/ext_next2 equal the IK of the trajectory at t+dt / t+2dt."""
    geom = StewartGeometry()
    traj = build_analytic_oval(JugglePattern())
    dt = 0.025
    player = TrajectoryPlayer(traj, geom=geom, control_dt=dt)
    for t in (0.0, 0.11, 0.37):
        cmd = player.command_at(t)
        for ahead, expected_ext in ((dt, cmd.ext_next_mm),
                                    (2 * dt, cmd.ext_next2_mm)):
            pose_ahead = traj.eval(t + ahead)[0]
            ext_ref = pose_to_leg_lengths(pose_ahead[:3],
                                          rotvec_to_rot_matrix(pose_ahead[3:]),
                                          geom)
            np.testing.assert_allclose(expected_ext, ext_ref, atol=1e-9)


def test_player_ik_roundtrips_to_commanded_pose():
    """FK of the commanded extensions recovers the commanded platform pose."""
    geom = StewartGeometry()
    traj = build_analytic_oval(JugglePattern())
    player = TrajectoryPlayer(traj, geom=geom)
    for t in (0.05, 0.25, 0.45):
        cmd = player.command_at(t)
        pos, rot, _J = leg_lengths_to_pose(cmd.ext_mm, geom)
        np.testing.assert_allclose(pos, cmd.pose[:3], atol=1e-4)
        np.testing.assert_allclose(rot, rotvec_to_rot_matrix(cmd.pose[3:]),
                                   atol=1e-6)
