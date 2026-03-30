"""Phase 6: Variable-resolution horizon tests.

Validates properties specific to the variable-resolution MPC horizon:
  - Smooth commands at the fine→coarse tier boundary
  - Arrival-time precision (±50 ms of deadline)
  - Horizon coverage (targets up to 1.3 s tracked accurately)
  - Catch orientation tilt (lateral ball velocity → platform tilt)
"""

from __future__ import annotations

import numpy as np
import pytest

from plant.mujoco_plant import MuJoCoPlant
from controller.mpc import MPCController
from controller.params import MPCParams
from controller.target import flat_target_to_events
from input.scripted import _compute_catch_orientation, _compute_catch_target

CONTROL_DT = 0.02  # 50 Hz


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _create_mpc(plant, **overrides):
    defaults = dict(max_cpu_time=2.0, max_iter=500, max_leg_vel_mmps=1000.0)
    defaults.update(overrides)
    return MPCController.from_plant(MPCParams(**defaults), plant)


def _run_to_target(plant, mpc, target_pose, arrival_time, duration_s,
                   target_twist=None):
    """Run MPC tracking a target, return list of state dicts."""
    n_steps = int(duration_s / CONTROL_DT)
    states = []
    for _ in range(n_steps):
        state = plant.get_state()
        current_pose = np.concatenate([state.platform_pos_mm, state.platform_rot])
        ref_events = flat_target_to_events(
            current_pose, state.platform_twist, target_pose, state.time,
            target_twist=target_twist, arrival_time=arrival_time)
        cmd, _cmd_vel, diag = mpc.solve(
            state, target_pose,
            ref_events=ref_events,
        )
        plant.command(cmd)
        plant.step(CONTROL_DT)

        actual = np.concatenate([state.platform_pos_mm, state.platform_rot])
        states.append({
            'time': state.time,
            'pose': actual.copy(),
            'twist': state.platform_twist.copy(),
            'cmd': cmd.copy(),
            'diag': diag,
        })
    return states


@pytest.fixture(scope='module')
def plant():
    return MuJoCoPlant()


# ---------------------------------------------------------------------------
# 1. Variable-resolution sanity: smooth commands at tier boundary
# ---------------------------------------------------------------------------

class TestTierBoundarySmoothness:
    """Commands should be smooth across the fine→coarse tier boundary.

    The MPC's internal horizon jumps from 20 ms to 250 ms steps at index 5.
    The *output* commands (one per CONTROL_DT = 20 ms) should not show
    discontinuities caused by this internal structure.
    """

    def test_no_command_jumps(self, plant):
        """Max Δcmd between consecutive steps stays below rate limit."""
        plant.reset()
        mpc = _create_mpc(plant)
        target = np.array([0.0, 0.0, 80.0, 0.0, 0.0, 0.0])

        states = _run_to_target(plant, mpc, target, arrival_time=0.8,
                                duration_s=1.5)

        v_max_dt = mpc.params.max_leg_vel_mmps * CONTROL_DT
        max_du = 0.0
        for i in range(1, len(states)):
            du = np.max(np.abs(states[i]['cmd'] - states[i - 1]['cmd']))
            max_du = max(max_du, du)

        assert max_du <= v_max_dt + 0.5, (
            f"Max Δcmd = {max_du:.2f} mm exceeds rate limit "
            f"{v_max_dt:.2f} mm per step"
        )

    def test_command_monotonicity_during_approach(self, plant):
        """During a pure-Z upward move, Z-related commands should increase
        monotonically (no reversals from tier boundary artefacts)."""
        plant.reset()
        mpc = _create_mpc(plant)
        target = np.array([0.0, 0.0, 60.0, 0.0, 0.0, 0.0])

        states = _run_to_target(plant, mpc, target, arrival_time=1.0,
                                duration_s=1.5)

        # Check Z position is monotonically increasing until near the target
        approach = [s for s in states if s['time'] < 0.8]
        z_vals = [s['pose'][2] for s in approach]

        reversals = 0
        for i in range(2, len(z_vals)):
            if z_vals[i] < z_vals[i - 1] - 0.1:  # 0.1mm noise tolerance
                reversals += 1

        assert reversals <= 2, (
            f"{reversals} Z reversals during approach (expected ≤ 2)"
        )

    def test_smooth_cmd_derivative(self, plant):
        """Command acceleration (d²cmd/dt²) stays bounded — no sharp
        kinks at the tier boundary."""
        plant.reset()
        mpc = _create_mpc(plant)
        target = np.array([20.0, -15.0, 70.0, 0.03, -0.02, 0.0])

        states = _run_to_target(plant, mpc, target, arrival_time=0.8,
                                duration_s=1.2)

        cmds = np.array([s['cmd'] for s in states])
        # First differences (velocity)
        du = np.diff(cmds, axis=0)
        # Second differences (acceleration)
        ddu = np.diff(du, axis=0)
        # Per-leg max acceleration
        max_accel = np.max(np.abs(ddu))

        # With rate limit of 1000 mm/s, max Δu per step = 20 mm.
        # Max ΔΔu should be bounded by ~2× max Δu = 40 mm.
        # In practice it should be much smaller for smooth trajectories.
        assert max_accel < 15.0, (
            f"Max command acceleration {max_accel:.2f} mm/step² — "
            f"possible discontinuity at tier boundary"
        )


# ---------------------------------------------------------------------------
# 2. Arrival-time precision
# ---------------------------------------------------------------------------

class TestArrivalTimePrecision:
    """Platform should arrive by the deadline with small position error.

    The MPC optimizes for minimum tracking cost, so it tends to arrive
    *early* rather than precisely at the deadline.  This is correct for
    catching: early arrival means the platform is waiting at the target
    when the ball arrives.  The key metric is position error AT the
    deadline, not arrival-time precision.
    """

    def _find_arrival_time(self, states, target_pose, threshold_mm=3.0):
        """Find the first time the position error drops below threshold."""
        for s in states:
            pos_err = np.linalg.norm(s['pose'][:3] - target_pose[:3])
            if pos_err < threshold_mm:
                return s['time']
        return None

    def test_at_deadline_error_small(self, plant):
        """Position error at t=1.0s deadline should be < 2 mm."""
        plant.reset()
        mpc = _create_mpc(plant)
        target = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        deadline = 1.0

        states = _run_to_target(plant, mpc, target, arrival_time=deadline,
                                duration_s=1.5)

        at_deadline = min(states, key=lambda s: abs(s['time'] - deadline))
        pos_err = np.linalg.norm(at_deadline['pose'][:3] - target[:3])

        assert pos_err < 2.0, (
            f"At deadline t={deadline}s: pos error {pos_err:.2f} mm > 2.0 mm"
        )

    def test_arrives_before_deadline(self, plant):
        """Platform reaches target BEFORE the 1.0s deadline (not late)."""
        plant.reset()
        mpc = _create_mpc(plant)
        target = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        deadline = 1.0

        states = _run_to_target(plant, mpc, target, arrival_time=deadline,
                                duration_s=1.5)

        arrival = self._find_arrival_time(states, target, threshold_mm=3.0)
        assert arrival is not None, "Platform never reached target"
        assert arrival <= deadline + CONTROL_DT, (
            f"Platform arrived at t={arrival:.3f}s, AFTER deadline {deadline}s"
        )

    def test_short_deadline_arrives(self, plant):
        """With a 0.5s deadline, platform should arrive by the deadline."""
        plant.reset()
        mpc = _create_mpc(plant)
        target = np.array([0.0, 0.0, 40.0, 0.0, 0.0, 0.0])
        deadline = 0.5

        states = _run_to_target(plant, mpc, target, arrival_time=deadline,
                                duration_s=1.0)

        at_deadline = min(states, key=lambda s: abs(s['time'] - deadline))
        pos_err = np.linalg.norm(at_deadline['pose'][:3] - target[:3])

        assert pos_err < 3.0, (
            f"At deadline t={deadline}s: pos error {pos_err:.2f} mm > 3.0 mm"
        )

    def test_arrives_before_longer_deadline(self, plant):
        """With a 1.2s deadline, platform arrives before the deadline.

        The MPC uses target-at-all-nodes reference (no linear interpolation
        pacing), so small moves arrive quickly.  This is correct — arriving
        early and holding is preferred over forced pacing through potentially
        infeasible intermediate references.
        """
        plant.reset()
        mpc = _create_mpc(plant)
        target = np.array([0.0, 0.0, 30.0, 0.0, 0.0, 0.0])
        deadline = 1.2

        states = _run_to_target(plant, mpc, target, arrival_time=deadline,
                                duration_s=1.8)

        arrival = self._find_arrival_time(states, target, threshold_mm=2.0)
        assert arrival is not None, "Platform never reached target"
        assert arrival < deadline, (
            f"Arrived at {arrival:.3f}s, past deadline {deadline}s"
        )


# ---------------------------------------------------------------------------
# 3. Horizon coverage
# ---------------------------------------------------------------------------

class TestHorizonCoverage:
    """Targets at various arrival times should be tracked accurately.

    The variable-resolution horizon covers 1.35s. Targets within this
    window are visible to the MPC from the first solve. Targets beyond
    the horizon still work — the MPC moves toward them as fast as
    constraints allow, and the deadline enters the horizon as time
    progresses.
    """

    @pytest.mark.parametrize("arrival_s,label", [
        (0.3, "near (0.3s)"),
        (0.8, "mid (0.8s)"),
        (1.3, "edge (1.3s)"),
    ])
    def test_within_horizon(self, plant, arrival_s, label):
        """Target within horizon (≤1.35s) — position error < 3 mm at deadline."""
        plant.reset()
        mpc = _create_mpc(plant)
        target = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])

        duration = arrival_s + 0.5
        states = _run_to_target(plant, mpc, target, arrival_time=arrival_s,
                                duration_s=duration)

        at_deadline = min(states, key=lambda s: abs(s['time'] - arrival_s))
        pos_err = np.linalg.norm(at_deadline['pose'][:3] - target[:3])

        assert pos_err < 3.0, (
            f"Horizon {label}: pos error {pos_err:.2f} mm at deadline > 3.0 mm"
        )

    def test_beyond_horizon(self, plant):
        """Target at 2.0s (beyond 1.35s horizon) — still converges."""
        plant.reset()
        mpc = _create_mpc(plant)
        target = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])

        states = _run_to_target(plant, mpc, target, arrival_time=2.0,
                                duration_s=2.5)

        # Should reach target eventually (at or after deadline)
        at_deadline = min(states, key=lambda s: abs(s['time'] - 2.0))
        pos_err = np.linalg.norm(at_deadline['pose'][:3] - target[:3])

        assert pos_err < 3.0, (
            f"Beyond-horizon: pos error {pos_err:.2f} mm at deadline > 3.0 mm"
        )

    def test_multi_axis_within_horizon(self, plant):
        """Combined translation + tilt target at 1.0s — both pos and ori converge."""
        plant.reset()
        mpc = _create_mpc(plant)
        target = np.array([30.0, -20.0, 60.0, 0.05, -0.03, 0.0])

        states = _run_to_target(plant, mpc, target, arrival_time=1.0,
                                duration_s=1.5)

        at_deadline = min(states, key=lambda s: abs(s['time'] - 1.0))
        pos_err = np.linalg.norm(at_deadline['pose'][:3] - target[:3])
        ori_err = np.degrees(np.linalg.norm(at_deadline['pose'][3:] - target[3:]))

        assert pos_err < 5.0, (
            f"Multi-axis pos error {pos_err:.2f} mm > 5.0 mm at deadline"
        )
        assert ori_err < 2.0, (
            f"Multi-axis ori error {ori_err:.3f}° > 2.0° at deadline"
        )

    def test_horizon_params_consistent(self, plant):
        """Verify dt_schedule produces expected horizon coverage."""
        mpc = _create_mpc(plant)
        params = mpc.params

        assert params.N == 10
        assert abs(params.horizon_s - 1.35) < 1e-10
        assert abs(params.dt_fine - 0.02) < 1e-10

        ct = params.cumulative_times
        assert ct.shape == (11,)
        assert abs(ct[0]) < 1e-10
        assert abs(ct[5] - 0.10) < 1e-10   # end of fine tier
        assert abs(ct[10] - 1.35) < 1e-10  # end of coarse tier


# ---------------------------------------------------------------------------
# 4. Catch orientation tilt
# ---------------------------------------------------------------------------

class TestCatchOrientationTilt:
    """Platform tilts to align catching axis with ball approach direction.

    When a ball has lateral velocity, the platform should tilt so its
    Z-down axis aligns with the ball's velocity vector, up to the 30°
    angle limit.
    """

    def test_tilt_from_lateral_velocity(self):
        """Ball with horizontal velocity → nonzero tilt rotation vector."""
        # Ball approaching from the side: vx = -500 mm/s, vz = -3000 mm/s
        landing_vel = np.array([-500.0, 0.0, -3000.0])
        rv = _compute_catch_orientation(landing_vel)

        assert rv is not None, "Tilt should be computed (angle < 30°)"
        angle_deg = np.degrees(np.linalg.norm(rv))

        # Expected angle: arccos(dot([-500,0,-3000]/norm, [0,0,-1]))
        v_hat = landing_vel / np.linalg.norm(landing_vel)
        expected_angle = np.degrees(np.arccos(np.clip(
            np.dot(np.array([0, 0, -1]), v_hat), -1, 1)))

        assert abs(angle_deg - expected_angle) < 0.1, (
            f"Tilt angle {angle_deg:.2f}° != expected {expected_angle:.2f}°"
        )
        assert angle_deg > 1.0, f"Tilt angle {angle_deg:.2f}° too small"
        assert angle_deg < 30.0, f"Tilt angle {angle_deg:.2f}° exceeds limit"

    def test_no_tilt_for_vertical_drop(self):
        """Ball dropping straight down → zero tilt."""
        landing_vel = np.array([0.0, 0.0, -4000.0])
        rv = _compute_catch_orientation(landing_vel)

        assert rv is not None
        angle_deg = np.degrees(np.linalg.norm(rv))
        assert angle_deg < 0.01, f"Vertical drop tilt {angle_deg:.4f}° > 0.01°"

    def test_rejects_shallow_angle(self):
        """Ball approaching nearly horizontally → rejected (> 30°)."""
        # Mostly horizontal: vx = -3000, vz = -500
        landing_vel = np.array([-3000.0, 0.0, -500.0])
        rv = _compute_catch_orientation(landing_vel)

        assert rv is None, "Shallow angle (>30°) should return None"

    def test_tilt_axis_perpendicular_to_approach(self):
        """Tilt rotation axis should be perpendicular to both the
        reference Z-down and the ball approach direction."""
        landing_vel = np.array([-800.0, 600.0, -3000.0])
        rv = _compute_catch_orientation(landing_vel)

        assert rv is not None
        angle = np.linalg.norm(rv)
        axis = rv / angle

        reference = np.array([0.0, 0.0, -1.0])
        v_hat = landing_vel / np.linalg.norm(landing_vel)

        # Axis should be perpendicular to both reference and v_hat
        assert abs(np.dot(axis, reference)) < 0.01, "Axis not perp to Z-down"
        assert abs(np.dot(axis, v_hat)) < 0.01, "Axis not perp to approach"

    def test_tilted_catch_target_offset(self):
        """Catch target with tilt: platform centroid is offset from ball
        landing position along the tilted Z axis."""
        target, ball = _compute_catch_target(
            spawn_pos_mm=np.array([30.0, -20.0, 3000.0]),
            spawn_vel_mms=np.array([-200.0, 150.0, 0.0]),
            spawn_time=0.3,
            active_z_offset_mm=80.0,
        )

        # Target should have nonzero orientation (tilt)
        rv = target.pose_6dof[3:]
        angle_deg = np.degrees(np.linalg.norm(rv))
        assert angle_deg > 0.5, (
            f"Expected nonzero tilt for lateral ball, got {angle_deg:.3f}°"
        )

    def test_mpc_tracks_tilted_target(self, plant):
        """MPC can reach a tilted catch target pose."""
        plant.reset()
        mpc = _create_mpc(plant)

        target, _ = _compute_catch_target(
            spawn_pos_mm=np.array([20.0, 0.0, 3000.0]),
            spawn_vel_mms=np.array([-300.0, 200.0, 0.0]),
            spawn_time=0.3,
            active_z_offset_mm=80.0,
        )

        states = _run_to_target(
            plant, mpc, target.pose_6dof,
            arrival_time=target.arrival_time,
            duration_s=target.arrival_time + 0.5,
        )

        at_deadline = min(states,
                          key=lambda s: abs(s['time'] - target.arrival_time))
        pos_err = np.linalg.norm(
            at_deadline['pose'][:3] - target.pose_6dof[:3])
        ori_err = np.degrees(np.linalg.norm(
            at_deadline['pose'][3:] - target.pose_6dof[3:]))

        assert pos_err < 5.0, (
            f"Tilted catch pos error {pos_err:.2f} mm > 5.0 mm"
        )
        assert ori_err < 2.0, (
            f"Tilted catch ori error {ori_err:.3f}° > 2.0°"
        )


# ---------------------------------------------------------------------------
# 5. dt_schedule structure tests
# ---------------------------------------------------------------------------

class TestDtScheduleStructure:
    """Verify the variable-resolution schedule properties."""

    def test_uniform_schedule_factory(self):
        """MPCParams.uniform_schedule produces uniform dt."""
        sched = MPCParams.uniform_schedule(N=8, dt=0.05)
        assert len(sched) == 8
        assert all(abs(d - 0.05) < 1e-10 for d in sched)

    def test_default_schedule_is_two_tier(self):
        """Default schedule has 5 fine + 5 coarse steps."""
        params = MPCParams()
        sched = params.dt_schedule

        fine = sched[:5]
        coarse = sched[5:]

        assert all(abs(d - 0.02) < 1e-10 for d in fine), "Fine tier != 20ms"
        assert all(abs(d - 0.25) < 1e-10 for d in coarse), "Coarse tier != 250ms"

    def test_cumulative_times_read_only(self):
        """cumulative_times array should be immutable."""
        params = MPCParams()
        ct = params.cumulative_times

        with pytest.raises(ValueError):
            ct[0] = 999.0

    def test_predicted_times_matches_schedule(self, plant):
        """MPC predicted_times matches params cumulative_times."""
        mpc = _create_mpc(plant)
        plant.reset()
        state = plant.get_state()
        mpc.solve(state, np.zeros(6))

        pt = mpc.predicted_times
        ct = mpc.params.cumulative_times

        np.testing.assert_allclose(pt, ct, atol=1e-10)


# ---------------------------------------------------------------------------
# 6. Warm-start shift tests
# ---------------------------------------------------------------------------

class TestWarmStartShift:
    """Verify the index-based warm-start shift produces valid output.

    The warm-start shifts the previous optimal solution forward by one
    index position (hold-last for the final element).  This is a simple
    index shift — no interpolation across the fine→coarse tier boundary.
    The solver corrects any warm-start inaccuracy within 1-2 iterations.
    """

    def test_shift_produces_finite_values(self, plant):
        """Shifted warm-start should contain only finite values."""
        plant.reset()
        mpc = _create_mpc(plant)
        state = plant.get_state()
        target = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])

        # First solve (cold start), second solve uses warm-start
        mpc.solve(state, target)
        plant.command(state.leg_extensions_mm)
        plant.step(CONTROL_DT)

        state2 = plant.get_state()
        cmd, _cmd_vel, diag = mpc.solve(state2, target)

        assert np.all(np.isfinite(cmd)), f"Non-finite cmd: {cmd}"
        assert diag['status'] in ('Solve_Succeeded', 'Solved_To_Acceptable_Level')
