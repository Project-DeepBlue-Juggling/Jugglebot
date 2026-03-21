"""Phase 2: Tests for the target-based MPC solve() interface.

Validates that the MPC correctly tracks target poses with:
  - ASAP mode (no arrival_time): converges as fast as constraints allow
  - Timed mode (arrival_time): arrives near the deadline with urgency ramp
"""

from __future__ import annotations

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

CONTROL_DT = 0.02


def _create_mpc(plant, **overrides):
    defaults = dict(max_cpu_time=2.0, max_iter=500, max_leg_vel_mmps=1000.0)
    defaults.update(overrides)
    return MPCController.from_plant(MPCParams(**defaults), plant)


def _run_target(plant, mpc, target_pose, arrival_time, duration_s,
                target_twist=None):
    """Run MPC tracking a target pose, return list of state dicts."""
    n_steps = int(duration_s / CONTROL_DT)
    states = []
    for _ in range(n_steps):
        state = plant.get_state()
        cmd, diag = mpc.solve(
            state, target_pose,
            target_twist=target_twist,
            arrival_time=arrival_time,
        )
        plant.command(cmd)
        plant.step(CONTROL_DT)

        actual = np.concatenate([state.platform_pos_mm, state.platform_rot])
        states.append({
            'time': state.time,
            'pose': actual.copy(),
            'cmd': cmd.copy(),
            'diag': diag,
        })
    return states


@pytest.fixture(scope='module')
def plant():
    return MuJoCoPlant()


# ---------------------------------------------------------------------------
# ASAP mode (arrival_time=None)
# ---------------------------------------------------------------------------

class TestASAPMode:
    """Target pose with no arrival deadline — converge as fast as possible."""

    def test_converges_to_target(self, plant):
        """Platform reaches z+50mm target within 1.5s."""
        plant.reset()
        mpc = _create_mpc(plant)
        target = np.array([0, 0, 50, 0, 0, 0.0])

        states = _run_target(plant, mpc, target, arrival_time=None, duration_s=1.5)

        final = states[-1]['pose']
        pos_err = np.linalg.norm(final[:3] - target[:3])
        ori_err = np.degrees(np.linalg.norm(final[3:] - target[3:]))

        assert pos_err < 1.0, f"ASAP final pos error {pos_err:.2f} mm > 1.0 mm"
        assert ori_err < 0.5, f"ASAP final ori error {ori_err:.3f}° > 0.5°"

    def test_no_solver_failures(self, plant):
        """No solver failures after warm-up in ASAP mode."""
        plant.reset()
        mpc = _create_mpc(plant)
        target = np.array([0, 0, 50, 0, 0, 0.0])

        states = _run_target(plant, mpc, target, arrival_time=None, duration_s=1.5)

        failures = [s for s in states[5:] if 'fallback' in s['diag']['status']]
        assert len(failures) == 0, f"{len(failures)} solver failures"

    def test_combined_pose(self, plant):
        """ASAP mode handles combined translation + tilt."""
        plant.reset()
        mpc = _create_mpc(plant)
        target = np.array([20, -15, 60, 0.05, -0.03, 0.0])

        states = _run_target(plant, mpc, target, arrival_time=None, duration_s=2.0)

        final = states[-1]['pose']
        pos_err = np.linalg.norm(final[:3] - target[:3])
        ori_err = np.degrees(np.linalg.norm(final[3:] - target[3:]))

        assert pos_err < 1.5, f"Combined pos error {pos_err:.2f} mm > 1.5 mm"
        assert ori_err < 0.5, f"Combined ori error {ori_err:.3f}° > 0.5°"


# ---------------------------------------------------------------------------
# Timed mode (arrival_time specified)
# ---------------------------------------------------------------------------

class TestTimedArrival:
    """Target pose with arrival deadline — urgency ramp should improve timing."""

    def test_arrives_by_deadline(self, plant):
        """Platform reaches z+50mm by t=1.0s deadline."""
        plant.reset()
        mpc = _create_mpc(plant)
        target = np.array([0, 0, 50, 0, 0, 0.0])
        arrival = 1.0  # absolute time

        states = _run_target(plant, mpc, target, arrival_time=arrival, duration_s=1.5)

        # Find state nearest to deadline
        at_deadline = min(states, key=lambda s: abs(s['time'] - arrival))
        pos_err = np.linalg.norm(at_deadline['pose'][:3] - target[:3])

        # Should arrive within 3mm by deadline (allowing for actuator lag)
        assert pos_err < 3.0, (
            f"At deadline pos error {pos_err:.2f} mm > 3.0 mm "
            f"(time={at_deadline['time']:.3f}s)"
        )

    def test_holds_after_deadline(self, plant):
        """After arrival, platform holds at target pose."""
        plant.reset()
        mpc = _create_mpc(plant)
        target = np.array([0, 0, 50, 0, 0, 0.0])
        arrival = 0.8

        states = _run_target(plant, mpc, target, arrival_time=arrival, duration_s=2.0)

        # Check hold quality after deadline + settling (0.3s after deadline)
        hold_states = [s for s in states if s['time'] >= arrival + 0.3]
        assert len(hold_states) > 10

        hold_errors = [np.linalg.norm(s['pose'][:3] - target[:3]) for s in hold_states]
        max_hold_err = np.max(hold_errors)

        assert max_hold_err < 1.0, f"Hold pos error {max_hold_err:.2f} mm > 1.0 mm"

    def test_urgency_improves_timing(self, plant):
        """Urgency ramp produces tighter arrival than no urgency."""
        target = np.array([30, -20, 60, 0.04, 0.0, 0.0])
        arrival = 0.8

        # With urgency (default)
        plant.reset()
        mpc_urg = _create_mpc(plant)
        states_urg = _run_target(plant, mpc_urg, target, arrival_time=arrival,
                                 duration_s=1.2)
        at_dl_urg = min(states_urg, key=lambda s: abs(s['time'] - arrival))
        err_urg = np.linalg.norm(at_dl_urg['pose'][:3] - target[:3])

        # Without urgency (urgency_max=1.0 disables the ramp)
        plant.reset()
        mpc_no = _create_mpc(plant, urgency_max=1.0)
        states_no = _run_target(plant, mpc_no, target, arrival_time=arrival,
                                duration_s=1.2)
        at_dl_no = min(states_no, key=lambda s: abs(s['time'] - arrival))
        err_no = np.linalg.norm(at_dl_no['pose'][:3] - target[:3])

        # Urgency should produce equal or better arrival accuracy
        # (allow 10% tolerance for noise)
        assert err_urg <= err_no * 1.1 + 0.5, (
            f"Urgency didn't help: {err_urg:.2f} mm vs {err_no:.2f} mm without"
        )

    def test_long_horizon_target(self, plant):
        """Target at 1.2s (within 1.35s horizon) is tracked accurately."""
        plant.reset()
        mpc = _create_mpc(plant)
        target = np.array([0, 0, 80, 0, 0, 0.0])
        arrival = 1.2

        states = _run_target(plant, mpc, target, arrival_time=arrival, duration_s=1.8)

        at_deadline = min(states, key=lambda s: abs(s['time'] - arrival))
        pos_err = np.linalg.norm(at_deadline['pose'][:3] - target[:3])

        assert pos_err < 3.0, (
            f"Long-horizon pos error {pos_err:.2f} mm > 3.0 mm"
        )

    def test_past_deadline_still_tracks(self, plant):
        """Target beyond horizon (2.0s) still works — MPC moves toward it."""
        plant.reset()
        mpc = _create_mpc(plant)
        target = np.array([0, 0, 50, 0, 0, 0.0])
        arrival = 2.0  # beyond 1.35s horizon

        states = _run_target(plant, mpc, target, arrival_time=arrival, duration_s=2.5)

        # Should eventually reach target
        final = states[-1]['pose']
        pos_err = np.linalg.norm(final[:3] - target[:3])
        assert pos_err < 1.0, f"Beyond-horizon final err {pos_err:.2f} mm > 1.0 mm"


# ---------------------------------------------------------------------------
# Urgency computation
# ---------------------------------------------------------------------------

class TestUrgencyComputation:
    """Unit tests for the urgency multiplier calculation."""

    def test_no_arrival_gives_uniform(self, plant):
        """arrival_time=None produces all-ones urgency."""
        mpc = _create_mpc(plant)
        urgency = mpc._compute_urgency(t_now=0.0, arrival_time=None)
        np.testing.assert_array_equal(urgency, 1.0)

    def test_far_deadline_near_one(self, plant):
        """Deadline 10s away: urgency should be ~1.0 for all nodes."""
        mpc = _create_mpc(plant)
        urgency = mpc._compute_urgency(t_now=0.0, arrival_time=10.0)
        # All nodes are far from deadline
        assert np.all(urgency < 1.5), f"Far deadline urgency too high: {urgency}"

    def test_imminent_deadline_ramps(self, plant):
        """Deadline 0.1s away: first few nodes should have high urgency."""
        mpc = _create_mpc(plant)
        urgency = mpc._compute_urgency(t_now=0.0, arrival_time=0.1)
        # Node at t=0.1s (within ramp window) should have elevated urgency
        assert urgency[0] > 1.0, "Near-deadline urgency should be > 1"
        # Coarse nodes (0.35s+) are past deadline — max urgency
        assert urgency[5] >= mpc.params.urgency_max - 0.1, \
            f"Past-deadline urgency {urgency[5]} should be near {mpc.params.urgency_max}"

    def test_urgency_max_respected(self, plant):
        """Urgency never exceeds urgency_max."""
        mpc = _create_mpc(plant, urgency_max=5.0)
        urgency = mpc._compute_urgency(t_now=0.0, arrival_time=0.01)
        assert np.all(urgency <= 5.0 + 1e-10), f"Urgency exceeds max: {urgency}"


# ---------------------------------------------------------------------------
# Reference construction
# ---------------------------------------------------------------------------

class TestBuildReference:
    """Unit tests for _build_reference() internal method."""

    def test_asap_all_target(self, plant):
        """ASAP mode: all reference nodes equal the target."""
        mpc = _create_mpc(plant)
        plant.reset()
        state = plant.get_state()
        target = np.array([10, 20, 50, 0.01, 0, 0.0])

        ref, twist = mpc._build_reference(state, target, arrival_time=None,
                                           target_twist=None)

        N = mpc.params.N
        assert ref.shape == (N + 1, 6)
        for k in range(N + 1):
            np.testing.assert_allclose(ref[k], target, atol=1e-10)
        np.testing.assert_allclose(twist, 0.0, atol=1e-10)

    def test_timed_all_target(self, plant):
        """Timed mode: all reference nodes equal the target (urgency handles pacing)."""
        mpc = _create_mpc(plant)
        plant.reset()
        state = plant.get_state()
        target = np.array([0, 0, 100, 0, 0, 0.0])
        arrival = state.time + 0.5  # 0.5s from now

        ref, twist = mpc._build_reference(state, target, arrival_time=arrival,
                                           target_twist=None)

        # All nodes should equal the target
        for k in range(mpc._N + 1):
            np.testing.assert_allclose(ref[k], target, atol=1e-10)

        # Twist before deadline should be zero; at/past deadline should be zero
        # (no target_twist was provided)
        np.testing.assert_allclose(twist, 0.0, atol=1e-10)

    def test_past_deadline_all_target(self, plant):
        """When past deadline, all nodes get target pose."""
        mpc = _create_mpc(plant)
        plant.reset()
        state = plant.get_state()
        target = np.array([0, 0, 50, 0, 0, 0.0])

        ref, _ = mpc._build_reference(state, target, arrival_time=state.time - 1.0,
                                       target_twist=None)

        for k in range(mpc.params.N + 1):
            np.testing.assert_allclose(ref[k], target, atol=1e-10)

    def test_arrival_twist_propagated(self, plant):
        """Target twist is applied at/past the deadline nodes."""
        mpc = _create_mpc(plant)
        plant.reset()
        state = plant.get_state()
        target = np.array([0, 0, 50, 0, 0, 0.0])
        tw = np.array([100, 0, 0, 0, 0, 0.0])  # 100 mm/s in x at arrival

        _, twist = mpc._build_reference(state, target, arrival_time=state.time + 0.05,
                                         target_twist=tw)

        # Nodes past deadline should have the target twist
        # Node at t=0.1s (idx 5 = first coarse) is past 0.05s deadline
        np.testing.assert_allclose(twist[-1], tw, atol=1e-10)
