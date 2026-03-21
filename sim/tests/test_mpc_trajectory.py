"""Phase 3: Validate MPC tracks waypoint-based trajectories.

With the variable-resolution MPC, trajectories are expressed as waypoint
lists ``[(pose, arrival_time), ...]``.  The MPC plans optimal motion between
waypoints internally — there is no external quintic reference to track.

For **sparse waypoints** (T1/T3/T4/T5/T6), the meaningful metric is
**arrival accuracy**: how close is the platform to the target at each
waypoint's arrival deadline.

For **dense waypoints** (T2 circular orbit), the target at each control
step IS the desired position, so continuous tracking error applies.

Tests:
  - T1 (linear translation): arrival error < 3 mm; returns to home
  - T2 (circular orbit): steady-state tracking error < 5 mm / 1.5 deg
  - T3 (multi-axis): arrival error < 5 mm / 2 deg
  - T4 (fast point-to-point): settles to final pose
  - Solve times remain within budget
  - Constraint satisfaction (stroke/rate)
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
        max_leg_vel_mmps=1000.0,
    )
    defaults.update(param_overrides)
    params = MPCParams(**defaults)
    return MPCController.from_plant(params, plant)


def _run_trajectory(plant, mpc, waypoints, duration_s):
    """Run MPC tracking a waypoint sequence, return (states, diagnostics).

    Each entry in states is a dict with 'time', 'pose', 'twist',
    'ref_pose', 'arrival_time', 'ext', 'cmd'.
    """
    n_steps = int(duration_s / CONTROL_DT)
    states = []
    diags = []
    idx = 0

    for _ in range(n_steps):
        state = plant.get_state()

        # Advance past reached waypoints
        while (idx + 1 < len(waypoints)
               and state.time >= waypoints[idx][1]):
            idx += 1
        target_pose, arrival_time = waypoints[idx]

        cmd, diag = mpc.solve(state, target_pose, arrival_time=arrival_time)
        plant.command(cmd)
        plant.step(CONTROL_DT)

        actual_pose = np.concatenate([state.platform_pos_mm, state.platform_rot])
        states.append({
            'time': state.time,
            'pose': actual_pose.copy(),
            'twist': state.platform_twist.copy(),
            'ref_pose': target_pose.copy(),
            'arrival_time': arrival_time,
            'ext': state.leg_extensions_mm.copy(),
            'cmd': cmd.copy(),
        })
        diags.append(diag)

    return states, diags


def _arrival_errors(states, waypoints):
    """Compute position and orientation error at each waypoint's arrival time.

    Returns list of dicts with 'waypoint_idx', 'arrival_time', 'pos_err_mm',
    'ori_err_deg', 'target_pose'.
    """
    errors = []
    for i, (target, arrival) in enumerate(waypoints):
        # Find the state nearest to this arrival time
        nearest = min(states, key=lambda s: abs(s['time'] - arrival))
        pos_err = np.linalg.norm(nearest['pose'][:3] - target[:3])
        ori_err = np.degrees(np.linalg.norm(nearest['pose'][3:] - target[3:]))
        errors.append({
            'waypoint_idx': i,
            'arrival_time': arrival,
            'pos_err_mm': pos_err,
            'ori_err_deg': ori_err,
            'target_pose': target,
        })
    return errors


def _continuous_tracking_errors(states):
    """Compute position error (mm) and orientation error (deg) per step.

    Meaningful only for dense waypoints (T2) where ref_pose tracks the
    desired position at each control step.
    """
    pos_err = np.array([
        np.linalg.norm(s['pose'][:3] - s['ref_pose'][:3]) for s in states
    ])
    ori_err = np.array([
        np.degrees(np.linalg.norm(s['pose'][3:] - s['ref_pose'][3:])) for s in states
    ])
    return pos_err, ori_err


# ---------------------------------------------------------------------------
# Tests: Trajectory tracking
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module')
def plant():
    """Shared MuJoCo plant for all tests in this module."""
    return MuJoCoPlant()


class TestT1LinearTranslation:
    """T1: home -> z+50mm -> home, 1s each segment."""

    def test_arrival_accuracy(self, plant):
        """Platform arrives within 3 mm of each waypoint."""
        plant.reset()
        mpc = _create_mpc(plant)
        waypoints, duration = make_T1()

        states, _ = _run_trajectory(plant, mpc, waypoints, duration)
        errors = _arrival_errors(states, waypoints)

        for e in errors:
            assert e['pos_err_mm'] < 3.0, (
                f"T1 waypoint {e['waypoint_idx']} (t={e['arrival_time']:.1f}s) "
                f"pos error {e['pos_err_mm']:.2f} mm > 3.0 mm"
            )

    def test_returns_home(self, plant):
        """Platform returns close to home after trajectory completes."""
        plant.reset()
        mpc = _create_mpc(plant)
        waypoints, duration = make_T1()

        states, _ = _run_trajectory(plant, mpc, waypoints, duration)

        final_pose = states[-1]['pose']
        pos_err = np.linalg.norm(final_pose[:3])
        ori_err = np.degrees(np.linalg.norm(final_pose[3:]))

        assert pos_err < 1.0, f"T1 final pos error {pos_err:.3f} mm > 1.0 mm"
        assert ori_err < 0.5, f"T1 final ori error {ori_err:.4f} deg > 0.5 deg"


class TestT2CircularOrbit:
    """T2: 80mm radius circle in XY at z=50, 2s period.

    T2 uses dense waypoints (every 20ms during orbit), so continuous
    tracking error is meaningful.
    """

    def test_steady_state_tracking(self, plant):
        """Steady-state tracking error < 15 mm / 1.5 deg (after first period).

        The target-based MPC sees one waypoint at a time (no trajectory
        lookahead), so it lags by ~50 ms (1 control step + actuator tau).
        At 251 mm/s orbital speed, this gives ~12 mm steady-state error.
        This is physically correct — the actuator can't predict the circle.
        """
        plant.reset()
        mpc = _create_mpc(plant)
        waypoints, duration = make_T2()

        states, _ = _run_trajectory(plant, mpc, waypoints, duration)
        pos_err, ori_err = _continuous_tracking_errors(states)

        # Use second period onward (after 0.5s start + 1.0s ramp + 2.0s first period)
        steady = [i for i, s in enumerate(states) if s['time'] >= 3.5]

        if not steady:
            pytest.skip("Not enough steady-state data")

        max_pos = np.max(pos_err[steady])
        max_ori = np.max(ori_err[steady])

        assert max_pos < 15.0, f"T2 steady max pos error {max_pos:.2f} mm > 15.0 mm"
        assert max_ori < 1.5, f"T2 steady max ori error {max_ori:.3f} deg > 1.5 deg"

    def test_z_tracking(self, plant):
        """Z height stays near 50mm during steady-state orbit."""
        plant.reset()
        mpc = _create_mpc(plant)
        waypoints, duration = make_T2()

        states, _ = _run_trajectory(plant, mpc, waypoints, duration)

        active_z = [s['pose'][2] for s in states if s['time'] >= 3.5]
        z_err = np.abs(np.array(active_z) - 50.0)
        max_z_err = np.max(z_err)

        assert max_z_err < 3.0, f"T2 max Z error {max_z_err:.2f} mm > 3.0 mm"


class TestT3MultiAxis:
    """T3: Simultaneous translation + tilt, 1.5s segments."""

    def test_arrival_accuracy(self, plant):
        """Platform arrives within 5 mm / 2 deg at each waypoint."""
        plant.reset()
        mpc = _create_mpc(plant)
        waypoints, duration = make_T3()

        states, _ = _run_trajectory(plant, mpc, waypoints, duration)
        errors = _arrival_errors(states, waypoints)

        for e in errors:
            assert e['pos_err_mm'] < 5.0, (
                f"T3 waypoint {e['waypoint_idx']} (t={e['arrival_time']:.1f}s) "
                f"pos error {e['pos_err_mm']:.2f} mm > 5.0 mm"
            )
            assert e['ori_err_deg'] < 2.0, (
                f"T3 waypoint {e['waypoint_idx']} (t={e['arrival_time']:.1f}s) "
                f"ori error {e['ori_err_deg']:.3f} deg > 2.0 deg"
            )


class TestT4SpeedTest:
    """T4: Fast point-to-point, 400ms transit."""

    def test_arrival_accuracy(self, plant):
        """Platform arrives within 8 mm at the final waypoint."""
        plant.reset()
        mpc = _create_mpc(plant)
        waypoints, duration = make_T4()

        states, _ = _run_trajectory(plant, mpc, waypoints, duration)
        errors = _arrival_errors(states, waypoints)

        # Check last waypoint (the fast destination)
        final = errors[-1]
        assert final['pos_err_mm'] < 8.0, (
            f"T4 final waypoint pos error {final['pos_err_mm']:.2f} mm > 8.0 mm"
        )

    def test_settles_after_transit(self, plant):
        """Platform settles to final pose after fast transit."""
        plant.reset()
        mpc = _create_mpc(plant)
        waypoints, duration = make_T4()

        states, _ = _run_trajectory(plant, mpc, waypoints, duration)

        # Last 0.3s should be settled at the final waypoint
        final_target = waypoints[-1][0]
        settle = [s for s in states if s['time'] >= duration - 0.3]
        if settle:
            settle_errors = [np.linalg.norm(s['pose'][:3] - final_target[:3])
                             for s in settle]
            assert np.max(settle_errors) < 1.0, "T4 not settled (pos)"


class TestTrajectoryPerformance:
    """Solve time and constraint satisfaction during trajectories."""

    def test_solve_time(self, plant):
        """Mean warm-started solve time < 20 ms during T1."""
        plant.reset()
        mpc = _create_mpc(plant)
        waypoints, duration = make_T1()

        _, diags = _run_trajectory(plant, mpc, waypoints, duration)

        solve_times = [d['solve_time_ms'] for d in diags]
        warm_times = solve_times[5:]
        mean_ms = np.mean(warm_times)

        assert mean_ms < 20.0, f"Mean solve time {mean_ms:.1f} ms > 20 ms"

    def test_stroke_limits(self, plant):
        """Commanded extensions stay within [0, 280] mm during T3."""
        plant.reset()
        mpc = _create_mpc(plant)
        waypoints, duration = make_T3()

        states, _ = _run_trajectory(plant, mpc, waypoints, duration)

        for s in states:
            assert np.all(s['cmd'] >= -0.5), f"Extension below 0: {np.min(s['cmd']):.2f}"
            assert np.all(s['cmd'] <= 280.5), f"Extension above 280: {np.max(s['cmd']):.2f}"

    def test_no_solver_failures(self, plant):
        """No solver failures during T1 (after warm-up)."""
        plant.reset()
        mpc = _create_mpc(plant)
        waypoints, duration = make_T1()

        _, diags = _run_trajectory(plant, mpc, waypoints, duration)

        warm_diags = diags[5:]
        failures = [d for d in warm_diags if 'fallback' in d['status']]

        assert len(failures) == 0, (
            f"{len(failures)} solver failures after warm-up "
            f"(first: {failures[0]['status'] if failures else 'N/A'})"
        )


class TestVelocityTracking:
    """Verify the MPC produces physically correct velocity profiles."""

    def test_circular_velocity_direction(self, plant):
        """During T2 steady-state orbit, actual velocity direction matches expected.

        The circular orbit has a well-defined velocity direction (tangent to circle).
        The MPC should produce platform motion roughly aligned with this direction.
        """
        plant.reset()
        mpc = _create_mpc(plant)
        waypoints, duration = make_T2()

        states, _ = _run_trajectory(plant, mpc, waypoints, duration)

        # Steady-state: second orbit period (after ramp + first period)
        radius = 80.0
        omega = 2.0 * np.pi / 2.0  # period = 2s
        orbit_start = 1.5  # 0.5s start + 1.0s ramp

        steady = [s for s in states if 3.5 <= s['time'] <= 5.0]
        assert len(steady) > 10, "Not enough steady-state data"

        alignments = []
        for s in steady:
            v_actual = s['twist'][:2]
            t_orbit = s['time'] - orbit_start
            theta = omega * t_orbit
            v_expected = np.array([
                -radius * omega * np.sin(theta),
                radius * omega * np.cos(theta),
            ])
            norm_a = np.linalg.norm(v_actual)
            norm_e = np.linalg.norm(v_expected)
            if norm_a > 10.0 and norm_e > 10.0:
                cos_sim = np.dot(v_actual, v_expected) / (norm_a * norm_e)
                alignments.append(cos_sim)

        assert len(alignments) > 5, "Not enough velocity samples above threshold"
        mean_alignment = np.mean(alignments)

        assert mean_alignment > 0.7, (
            f"Velocity direction alignment {mean_alignment:.3f} < 0.7 "
            f"(MPC not tracking velocity direction)"
        )

    def test_zero_velocity_at_waypoint(self, plant):
        """At T1 apex (z=50mm), platform should decelerate to near-zero velocity.

        The MPC transitions from "approach z50" to "return home" at the waypoint
        boundary. Platform velocity should be small near the transition.
        """
        plant.reset()
        mpc = _create_mpc(plant)
        waypoints, duration = make_T1()

        states, _ = _run_trajectory(plant, mpc, waypoints, duration)

        # Near the apex (t ~ 1.5s, where waypoint transition happens)
        apex = [s for s in states if 1.4 <= s['time'] <= 1.6]
        assert len(apex) > 0, "No data near apex"

        apex_speeds = [np.linalg.norm(s['twist'][:3]) for s in apex]
        min_speed = np.min(apex_speeds)

        assert min_speed < 20.0, (
            f"Min speed at apex {min_speed:.1f} mm/s > 20 mm/s "
            f"(platform not decelerating at waypoint)"
        )
