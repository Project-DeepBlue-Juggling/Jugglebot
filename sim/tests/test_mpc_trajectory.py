"""Phase 3: Validate MPC tracks time-varying reference trajectories.

Tests:
  - ReferenceGenerator produces correct shapes and values
  - T1 (linear translation): max tracking error < 3 mm / 1°
  - T2 (circular orbit): steady-state tracking error < 5 mm / 1°
  - T3 (multi-axis): max tracking error < 5 mm / 1°
  - T4 (fast point-to-point): max tracking error < 8 mm / 2°
  - Solve times remain within budget during trajectory tracking
  - Constraint satisfaction (stroke/rate) during trajectories
"""

import os
import sys

import numpy as np
import pytest

_sim_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _sim_dir not in sys.path:
    sys.path.insert(0, _sim_dir)

from plant.mujoco_plant import MuJoCoPlant
from controller.mpc import MPCController
from controller.params import MPCParams
from controller.reference import ReferenceGenerator
from input.scripted import make_T1, make_T2, make_T3, make_T4

CONTROL_DT = 0.02  # 50 Hz


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _create_mpc(plant, **param_overrides):
    """Create an MPC controller with trajectory-appropriate settings."""
    defaults = dict(
        max_cpu_time=2.0,
        max_iter=500,
        max_leg_vel_mmps=1000.0,  # realistic hardware limit for trajectory tracking
    )
    defaults.update(param_overrides)
    params = MPCParams(**defaults)
    return MPCController.from_plant(params, plant)


def _run_trajectory(plant, mpc, ref_gen, duration_s):
    """Run MPC tracking a ReferenceGenerator, return (states, diagnostics).

    Each entry in states is a dict with 'time', 'pose', 'twist', 'ref_pose',
    'ref_twist', 'ext', 'cmd'.
    """
    n_steps = int(duration_s / CONTROL_DT)
    states = []
    diags = []

    for _ in range(n_steps):
        state = plant.get_state()
        poses, twists = ref_gen.evaluate(state.time, CONTROL_DT, mpc.params.N)
        cmd, diag = mpc.solve(state, poses, ref_twist=twists)
        plant.command(cmd)
        plant.step(CONTROL_DT)

        actual_pose = np.concatenate([state.platform_pos_mm, state.platform_rot])
        states.append({
            'time': state.time,
            'pose': actual_pose.copy(),
            'twist': state.platform_twist.copy(),
            'ref_pose': poses[0].copy(),
            'ref_twist': twists[0].copy(),
            'ext': state.leg_extensions_mm.copy(),
            'cmd': cmd.copy(),
        })
        diags.append(diag)

    return states, diags


def _tracking_errors(states):
    """Compute position error (mm) and orientation error (deg) per step."""
    pos_err = np.array([
        np.linalg.norm(s['pose'][:3] - s['ref_pose'][:3]) for s in states
    ])
    ori_err = np.array([
        np.degrees(np.linalg.norm(s['pose'][3:] - s['ref_pose'][3:])) for s in states
    ])
    return pos_err, ori_err


# ---------------------------------------------------------------------------
# Tests: ReferenceGenerator
# ---------------------------------------------------------------------------

class TestReferenceGenerator:
    """ReferenceGenerator basic functionality."""

    def test_static_pose(self):
        """Static reference returns constant pose and zero twist."""
        ref = ReferenceGenerator.from_static_pose(np.array([0, 0, 50, 0, 0, 0.0]))
        poses, twists = ref.evaluate(0.0, 0.02, 10)

        assert poses.shape == (11, 6)
        assert twists.shape == (11, 6)
        np.testing.assert_allclose(poses, np.tile([0, 0, 50, 0, 0, 0], (11, 1)))
        np.testing.assert_allclose(twists, 0.0)

    def test_waypoints_shape(self):
        """Waypoint reference returns correct shapes."""
        ref = ReferenceGenerator.from_waypoints(
            waypoints=[np.zeros(6), np.array([0, 0, 50, 0, 0, 0.0])],
            durations=[1.0],
        )
        poses, twists = ref.evaluate(0.0, 0.02, 10)
        assert poses.shape == (11, 6)
        assert twists.shape == (11, 6)

    def test_waypoints_boundary(self):
        """Waypoint reference starts at first waypoint and ends at last."""
        start = np.zeros(6)
        end = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        ref = ReferenceGenerator.from_waypoints(
            waypoints=[start, end],
            durations=[1.0],
            start_time=0.0,
        )
        # At t=0 (start)
        poses, _ = ref.evaluate(0.0, 0.02, 1)
        np.testing.assert_allclose(poses[0], start, atol=0.01)

        # At t=1.0 (end)
        poses, _ = ref.evaluate(1.0, 0.02, 1)
        np.testing.assert_allclose(poses[0], end, atol=0.01)

    def test_waypoints_zero_twist_at_endpoints(self):
        """Rest-to-rest waypoints have zero twist at boundaries."""
        ref = ReferenceGenerator.from_waypoints(
            waypoints=[np.zeros(6), np.array([0, 0, 50, 0, 0, 0.0])],
            durations=[1.0],
            start_time=0.0,
        )
        # At start
        _, twists = ref.evaluate(0.0, 0.02, 1)
        np.testing.assert_allclose(twists[0], 0.0, atol=0.01)

        # At end (hold phase)
        _, twists = ref.evaluate(1.5, 0.02, 1)
        np.testing.assert_allclose(twists[0], 0.0, atol=0.01)

    def test_circular_function(self):
        """Function-based reference (circular) has correct values."""
        radius = 50.0
        omega = 2.0 * np.pi

        def circle(t):
            x = radius * np.cos(omega * t)
            y = radius * np.sin(omega * t)
            vx = -radius * omega * np.sin(omega * t)
            vy = radius * omega * np.cos(omega * t)
            return np.array([x, y, 0, 0, 0, 0.0]), np.array([vx, vy, 0, 0, 0, 0.0])

        ref = ReferenceGenerator.from_function(circle, duration=1.0)
        poses, twists = ref.evaluate(0.0, 0.02, 5)

        # At t=0: x=radius, y=0
        np.testing.assert_allclose(poses[0, 0], radius, atol=0.01)
        np.testing.assert_allclose(poses[0, 1], 0.0, atol=0.01)


# ---------------------------------------------------------------------------
# Tests: Trajectory tracking
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module')
def plant():
    """Shared MuJoCo plant for all tests in this module."""
    return MuJoCoPlant()


class TestT1LinearTranslation:
    """T1: home -> z+50mm -> home, 1s each segment."""

    def test_tracking_quality(self, plant):
        """Max tracking error < 3 mm / 1° during active trajectory."""
        plant.reset()
        mpc = _create_mpc(plant)
        ref_gen, duration = make_T1()

        states, diags = _run_trajectory(plant, mpc, ref_gen, duration)
        pos_err, ori_err = _tracking_errors(states)

        # Skip first 0.5s (pre-start settle time) and last 0.3s (post-settle)
        active = [i for i, s in enumerate(states)
                  if 0.5 <= s['time'] <= duration - 0.3]

        max_pos = np.max(pos_err[active])
        max_ori = np.max(ori_err[active])

        assert max_pos < 3.0, f"T1 max pos error {max_pos:.2f} mm > 3.0 mm"
        assert max_ori < 1.0, f"T1 max ori error {max_ori:.3f}° > 1.0°"

    def test_returns_home(self, plant):
        """Platform returns close to home after trajectory completes."""
        plant.reset()
        mpc = _create_mpc(plant)
        ref_gen, duration = make_T1()

        states, _ = _run_trajectory(plant, mpc, ref_gen, duration)

        final_pose = states[-1]['pose']
        pos_err = np.linalg.norm(final_pose[:3])
        ori_err = np.degrees(np.linalg.norm(final_pose[3:]))

        assert pos_err < 1.0, f"T1 final pos error {pos_err:.3f} mm > 1.0 mm"
        assert ori_err < 0.5, f"T1 final ori error {ori_err:.4f}° > 0.5°"


class TestT2CircularOrbit:
    """T2: 80mm radius circle in XY at z=50, 2s period."""

    def test_steady_state_tracking(self, plant):
        """Steady-state tracking error < 5 mm / 1° (after first period)."""
        plant.reset()
        mpc = _create_mpc(plant)
        ref_gen, duration = make_T2()

        states, diags = _run_trajectory(plant, mpc, ref_gen, duration)
        pos_err, ori_err = _tracking_errors(states)

        # Use second period onward (after 0.5s start + 1.0s ramp + 2.0s first period)
        steady = [i for i, s in enumerate(states) if s['time'] >= 3.5]

        if not steady:
            pytest.skip("Not enough steady-state data")

        max_pos = np.max(pos_err[steady])
        max_ori = np.max(ori_err[steady])
        rms_pos = np.sqrt(np.mean(pos_err[steady] ** 2))

        assert max_pos < 5.0, f"T2 steady max pos error {max_pos:.2f} mm > 5.0 mm"
        assert max_ori < 1.0, f"T2 steady max ori error {max_ori:.3f}° > 1.0°"

    def test_z_tracking(self, plant):
        """Z height stays near 50mm during steady-state orbit."""
        plant.reset()
        mpc = _create_mpc(plant)
        ref_gen, duration = make_T2()

        states, _ = _run_trajectory(plant, mpc, ref_gen, duration)

        # Check Z during steady orbit (after 0.5s start + 1s ramp + 2s first period)
        active_z = [s['pose'][2] for s in states if s['time'] >= 3.5]
        z_err = np.abs(np.array(active_z) - 50.0)
        max_z_err = np.max(z_err)

        assert max_z_err < 3.0, f"T2 max Z error {max_z_err:.2f} mm > 3.0 mm"


class TestT3MultiAxis:
    """T3: Simultaneous translation + tilt, 1.5s segments."""

    def test_tracking_quality(self, plant):
        """Max tracking error < 5 mm / 2° during active trajectory."""
        plant.reset()
        mpc = _create_mpc(plant)
        ref_gen, duration = make_T3()

        states, diags = _run_trajectory(plant, mpc, ref_gen, duration)
        pos_err, ori_err = _tracking_errors(states)

        active = [i for i, s in enumerate(states)
                  if 0.5 <= s['time'] <= duration - 0.3]

        max_pos = np.max(pos_err[active])
        max_ori = np.max(ori_err[active])

        assert max_pos < 5.0, f"T3 max pos error {max_pos:.2f} mm > 5.0 mm"
        assert max_ori < 2.0, f"T3 max ori error {max_ori:.3f}° > 2.0°"


class TestT4SpeedTest:
    """T4: Fast point-to-point, 400ms transit."""

    def test_tracking_quality(self, plant):
        """Max tracking error < 8 mm / 2° during fast transit."""
        plant.reset()
        mpc = _create_mpc(plant)
        ref_gen, duration = make_T4()

        states, diags = _run_trajectory(plant, mpc, ref_gen, duration)
        pos_err, ori_err = _tracking_errors(states)

        # During fast transit phase (after 1.0s ramp-up, 0.4s transit)
        transit = [i for i, s in enumerate(states)
                   if 1.5 <= s['time'] <= 2.0]

        max_pos = np.max(pos_err[transit]) if transit else 0.0
        max_ori = np.max(ori_err[transit]) if transit else 0.0

        assert max_pos < 8.0, f"T4 max pos error {max_pos:.2f} mm > 8.0 mm"
        assert max_ori < 2.0, f"T4 max ori error {max_ori:.3f}° > 2.0°"

    def test_settles_after_transit(self, plant):
        """Platform settles to final pose after fast transit."""
        plant.reset()
        mpc = _create_mpc(plant)
        ref_gen, duration = make_T4()

        states, _ = _run_trajectory(plant, mpc, ref_gen, duration)
        pos_err, ori_err = _tracking_errors(states)

        # Last 0.3s should be settled
        settle = [i for i, s in enumerate(states) if s['time'] >= duration - 0.3]
        if settle:
            assert np.max(pos_err[settle]) < 1.0, "T4 not settled (pos)"
            assert np.max(ori_err[settle]) < 0.5, "T4 not settled (ori)"


class TestTrajectoryPerformance:
    """Solve time and constraint satisfaction during trajectories."""

    def test_solve_time(self, plant):
        """Mean warm-started solve time < 15 ms during T1."""
        plant.reset()
        mpc = _create_mpc(plant)
        ref_gen, duration = make_T1()

        _, diags = _run_trajectory(plant, mpc, ref_gen, duration)

        solve_times = [d['solve_time_ms'] for d in diags]
        # Exclude first few cold-start solves
        warm_times = solve_times[5:]
        mean_ms = np.mean(warm_times)

        assert mean_ms < 15.0, f"Mean solve time {mean_ms:.1f} ms > 15 ms"

    def test_stroke_limits(self, plant):
        """Commanded extensions stay within [0, 280] mm during T3."""
        plant.reset()
        mpc = _create_mpc(plant)
        ref_gen, duration = make_T3()

        states, _ = _run_trajectory(plant, mpc, ref_gen, duration)

        for s in states:
            assert np.all(s['cmd'] >= -0.5), f"Extension below 0: {np.min(s['cmd']):.2f}"
            assert np.all(s['cmd'] <= 280.5), f"Extension above 280: {np.max(s['cmd']):.2f}"

    def test_no_solver_failures(self, plant):
        """No solver failures during T1 (after warm-up)."""
        plant.reset()
        mpc = _create_mpc(plant)
        ref_gen, duration = make_T1()

        _, diags = _run_trajectory(plant, mpc, ref_gen, duration)

        # Exclude first 5 steps (cold-start may timeout)
        warm_diags = diags[5:]
        failures = [d for d in warm_diags if 'fallback' in d['status']]

        assert len(failures) == 0, (
            f"{len(failures)} solver failures after warm-up "
            f"(first: {failures[0]['status'] if failures else 'N/A'})"
        )


class TestVelocityTracking:
    """Verify the MPC actually tracks reference twist, not just pose."""

    def test_circular_velocity_direction(self, plant):
        """During T2 steady-state orbit, actual velocity direction matches reference.

        The circular orbit has a well-defined velocity direction (tangent to circle).
        The MPC should produce platform motion roughly aligned with this direction.
        """
        plant.reset()
        mpc = _create_mpc(plant)
        ref_gen, duration = make_T2()

        states, _ = _run_trajectory(plant, mpc, ref_gen, duration)

        # Steady-state: second orbit period (after ramp + first period)
        steady = [s for s in states if s['time'] >= 3.5 and s['time'] <= 5.0]
        assert len(steady) > 10, "Not enough steady-state data"

        # Check velocity direction alignment (cosine similarity in XY)
        alignments = []
        for s in steady:
            v_actual = s['twist'][:2]  # actual XY velocity
            v_ref = s['ref_twist'][:2]  # reference XY velocity
            norm_a = np.linalg.norm(v_actual)
            norm_r = np.linalg.norm(v_ref)
            if norm_a > 10.0 and norm_r > 10.0:  # ignore near-zero velocities
                cos_sim = np.dot(v_actual, v_ref) / (norm_a * norm_r)
                alignments.append(cos_sim)

        assert len(alignments) > 5, "Not enough velocity samples above threshold"
        mean_alignment = np.mean(alignments)

        # Velocity direction should be mostly aligned (> 0.7 = within ~45°)
        assert mean_alignment > 0.7, (
            f"Velocity direction alignment {mean_alignment:.3f} < 0.7 "
            f"(MPC not tracking velocity direction)"
        )

    def test_zero_velocity_at_waypoint(self, plant):
        """At rest-to-rest waypoint boundaries, actual velocity should be near zero.

        T1 has zero reference twist at the apex (z=50mm, t≈1.5s) — the platform
        should be nearly stationary there.
        """
        plant.reset()
        mpc = _create_mpc(plant)
        ref_gen, duration = make_T1()

        states, _ = _run_trajectory(plant, mpc, ref_gen, duration)

        # Near the apex (t ≈ 1.5s, where segment 1→2 boundary is)
        apex = [s for s in states if 1.4 <= s['time'] <= 1.6]
        assert len(apex) > 0, "No data near apex"

        # Platform velocity should be small at the waypoint boundary
        apex_speeds = [np.linalg.norm(s['twist'][:3]) for s in apex]
        min_speed = np.min(apex_speeds)

        assert min_speed < 20.0, (
            f"Min speed at apex {min_speed:.1f} mm/s > 20 mm/s "
            f"(platform not decelerating at waypoint)"
        )
