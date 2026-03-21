"""Tests for MPC dynamic target tracking (Phase 5C).

Validates that the MPC correctly tracks timed targets with hand coordination,
ball capture, and return-to-home sequences.  These are integration tests that
run the full MPC + plant + catch coordinator pipeline.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from plant.mujoco_plant import MuJoCoPlant
from controller import MPCController, MPCParams
from hand.coordinator import HandCoordinator, DynamicTarget, HandPhase
from hand.trajectory import HandCatchSequence
from input.scripted import get_catch_sequence, BallSpawn

# MPC control rate
CONTROL_DT = 0.02  # 50 Hz


def _run_catch_sim(
    plant: MuJoCoPlant,
    mpc: MPCController,
    coordinator: HandCoordinator,
    duration: float,
) -> dict:
    """Run a complete catch simulation and return diagnostic info.

    Returns dict with keys:
        events: list of HandEvent
        final_pose: (6,) final platform pose
        ball_state: final BallState or None
        max_pos_error_mm: max position error during approach
        captures: number of capture events
    """
    max_pos_err = 0.0
    captures = 0
    active_hand_seq = None

    n_steps = int(duration / CONTROL_DT)
    for step_idx in range(n_steps):
        state = plant.get_state()
        current_pose = np.concatenate([state.platform_pos_mm, state.platform_rot])
        hand_pos = state.hand_pos_mm if state.hand_pos_mm is not None else 0.0

        # Check ball spawn
        spawn = coordinator.should_spawn_ball(state.time)
        if spawn is not None:
            plant.spawn_ball(spawn.position_mm, spawn.velocity_mms)

        # Update coordinator
        target, hand_cmd = coordinator.update(
            state.time, current_pose, hand_pos_mm=hand_pos,
            plant_state=state)

        if isinstance(hand_cmd, HandCatchSequence):
            active_hand_seq = hand_cmd
        elif hand_cmd == 'prime':
            plant.hand_to_prime()
            active_hand_seq = None
        elif hand_cmd == 'home':
            plant.hand_to_home()
            active_hand_seq = None

        # Sample active hand catch trajectory
        if active_hand_seq is not None:
            pos = active_hand_seq.sample(state.time)
            if pos is not None:
                plant.command_hand(pos)
            else:
                active_hand_seq = None

        # MPC solve
        if target is not None:
            cmd, diag = mpc.solve(
                state, target.pose_6dof,
                arrival_time=target.arrival_time,
                target_twist=target.arrival_twist)
            # Track error to target
            pos_err = np.linalg.norm(current_pose[:3] - target.pose_6dof[:3])
            max_pos_err = max(max_pos_err, pos_err)
        else:
            cmd, diag = mpc.solve(state, np.zeros(6))

        plant.command(cmd)
        plant.step(CONTROL_DT)

        # Check capture
        if plant.has_ball and plant.check_and_capture():
            coordinator.notify_capture(state.time)
            captures += 1

    return {
        'events': coordinator.events,
        'final_pose': np.concatenate([
            plant.get_state().platform_pos_mm,
            plant.get_state().platform_rot,
        ]),
        'ball_state': plant.get_ball_state(),
        'max_pos_error_mm': max_pos_err,
        'captures': captures,
    }


@pytest.fixture
def plant():
    return MuJoCoPlant()


@pytest.fixture
def mpc(plant):
    params = MPCParams(
        max_cpu_time=2.0,
        max_iter=500,
        max_leg_vel_mmps=1000.0,
    )
    return MPCController.from_plant(params, plant)


class TestDT1SingleCatch:
    """DT1: Single catch — platform arrives at target, ball captured."""

    def test_platform_arrives(self, plant, mpc):
        coordinator = HandCoordinator()
        sequence, duration = get_catch_sequence('DT1')
        for target, ball in sequence:
            coordinator.submit_target(target, ball)

        result = _run_catch_sim(plant, mpc, coordinator, duration)

        # Check that we have at least one event
        assert len(result['events']) >= 1
        ev = result['events'][0]

        # Platform should arrive within 5 mm of target
        assert ev.arrival_error_mm < 5.0, \
            f"Arrival error {ev.arrival_error_mm:.1f} mm exceeds 5 mm threshold"

    def test_hand_is_primed(self, plant, mpc):
        coordinator = HandCoordinator()
        sequence, duration = get_catch_sequence('DT1')
        for target, ball in sequence:
            coordinator.submit_target(target, ball)

        # Run for a bit, then check hand state
        active_hand_seq = None
        for _ in range(50):  # 1 second
            state = plant.get_state()
            pose = np.concatenate([state.platform_pos_mm, state.platform_rot])
            hand_pos = state.hand_pos_mm if state.hand_pos_mm is not None else 0.0
            spawn = coordinator.should_spawn_ball(state.time)
            if spawn:
                plant.spawn_ball(spawn.position_mm, spawn.velocity_mms)
            target_out, hand_cmd = coordinator.update(
                state.time, pose, hand_pos_mm=hand_pos,
                plant_state=state)
            if isinstance(hand_cmd, HandCatchSequence):
                active_hand_seq = hand_cmd
            elif hand_cmd == 'prime':
                plant.hand_to_prime()
            if active_hand_seq is not None:
                p = active_hand_seq.sample(state.time)
                if p is not None:
                    plant.command_hand(p)
                else:
                    active_hand_seq = None
            if target_out is not None:
                cmd, _ = mpc.solve(
                    state, target_out.pose_6dof,
                    arrival_time=target_out.arrival_time,
                    target_twist=target_out.arrival_twist)
            else:
                cmd, _ = mpc.solve(state, np.zeros(6))
            plant.command(cmd)
            plant.step(CONTROL_DT)

        assert coordinator.hand_primed, "Hand should be primed for catch"
        # Hand should have moved (not still at bottom). After 1 second the
        # hand may have completed its catch trajectory and be retracting,
        # so we just verify it's not at 0 (was actuated at some point).
        hand_state = plant.get_state()
        assert hand_state.hand_pos_mm is not None
        assert hand_state.hand_pos_mm > 5, \
            f"Hand at {hand_state.hand_pos_mm:.1f} mm, expected > 5 mm (should have moved)"


class TestDT3WorkspaceBoundary:
    """DT3: Target near workspace edge — should still reach."""

    def test_reaches_edge_target(self, plant, mpc):
        coordinator = HandCoordinator()
        sequence, duration = get_catch_sequence('DT3')
        for target, ball in sequence:
            coordinator.submit_target(target, ball)

        result = _run_catch_sim(plant, mpc, coordinator, duration)

        assert len(result['events']) >= 1
        ev = result['events'][0]
        # Larger tolerance for workspace edge
        assert ev.arrival_error_mm < 10.0, \
            f"Arrival error {ev.arrival_error_mm:.1f} mm exceeds 10 mm threshold"


class TestDT5EarlyArrival:
    """DT5: Long lead time — platform should arrive early and hold."""

    def test_arrives_early_and_holds(self, plant, mpc):
        coordinator = HandCoordinator()
        sequence, duration = get_catch_sequence('DT5')
        for target, ball in sequence:
            coordinator.submit_target(target, ball)

        result = _run_catch_sim(plant, mpc, coordinator, duration)

        assert len(result['events']) >= 1
        ev = result['events'][0]
        # With 2.5s lead time, should arrive with < 3mm error
        assert ev.arrival_error_mm < 3.0, \
            f"Arrival error {ev.arrival_error_mm:.1f} mm exceeds 3 mm threshold"


class TestDT6Throw:
    """DT6: Throw mode — arrive with nonzero velocity."""

    def test_throw_completes(self, plant, mpc):
        coordinator = HandCoordinator()
        sequence, duration = get_catch_sequence('DT6')
        for target, ball in sequence:
            coordinator.submit_target(target, ball)

        result = _run_catch_sim(plant, mpc, coordinator, duration)

        # Should have a throw event
        assert len(result['events']) >= 1
        ev = result['events'][0]
        assert ev.phase == HandPhase.THROWING

        # Platform should return near home after deceleration
        final_pos_err = np.linalg.norm(result['final_pose'][:3])
        assert final_pos_err < 15.0, \
            f"Final position {final_pos_err:.1f} mm from home"


class TestDT8BallMiss:
    """DT8: Ball misses — system returns to home gracefully."""

    def test_miss_returns_home(self, plant, mpc):
        coordinator = HandCoordinator()
        sequence, duration = get_catch_sequence('DT8')
        for target, ball in sequence:
            coordinator.submit_target(target, ball)

        result = _run_catch_sim(plant, mpc, coordinator, duration)

        # Should not have captured the ball
        assert result['captures'] == 0, "Ball should not have been captured"

        # Platform should return near home
        final_pos_err = np.linalg.norm(result['final_pose'][:3])
        assert final_pos_err < 10.0, \
            f"Final position {final_pos_err:.1f} mm from home"


class TestFeasibilityChecker:
    """Coarse-horizon feasibility checker."""

    def test_accepts_feasible_target(self, plant):
        """DT1 (moderate pose, 0.9s deadline) should be feasible."""
        from hand.feasibility import FeasibilityChecker
        checker = FeasibilityChecker(plant)
        sequence, _ = get_catch_sequence('DT1')
        target, _ = sequence[0]
        state = plant.get_state()
        feasible, reason = checker.check(state, target.pose_6dof, target.arrival_time)
        assert feasible, f"DT1 should be feasible but got: {reason}"

    def test_rejects_infeasible_target(self, plant):
        """DT4 (extreme pose, 0.3s deadline) should be rejected."""
        from hand.feasibility import FeasibilityChecker
        checker = FeasibilityChecker(plant)
        sequence, _ = get_catch_sequence('DT4')
        target, _ = sequence[0]
        state = plant.get_state()
        feasible, reason = checker.check(state, target.pose_6dof, target.arrival_time)
        assert not feasible, "DT4 should be rejected (infeasible)"
        assert 'pos error' in reason or 'ori error' in reason

    def test_rejects_out_of_stroke(self, plant):
        """Target with extensions beyond stroke should be rejected at IK stage."""
        from hand.feasibility import FeasibilityChecker
        checker = FeasibilityChecker(plant)
        # Extreme Z that exceeds stroke
        extreme_pose = np.array([0, 0, 350, 0, 0, 0])
        state = plant.get_state()
        feasible, reason = checker.check(state, extreme_pose, 2.0)
        assert not feasible, "Extreme Z should be rejected"
        assert 'stroke' in reason

    def test_coordinator_rejects_with_checker(self, plant, mpc):
        """When feasibility checker is wired in, DT4 gets a rejection event."""
        from hand.feasibility import FeasibilityChecker
        checker = FeasibilityChecker(plant)
        coordinator = HandCoordinator(feasibility_checker=checker)
        sequence, duration = get_catch_sequence('DT4')
        for target, ball in sequence:
            coordinator.submit_target(target, ball)

        result = _run_catch_sim(plant, mpc, coordinator, duration)

        # Should have a rejection event (arrival_error_mm == -1 sentinel)
        assert len(result['events']) >= 1
        ev = result['events'][0]
        assert ev.arrival_error_mm == -1.0, \
            f"Expected rejection sentinel, got arrival_error={ev.arrival_error_mm}"
        assert result['captures'] == 0


class TestSolverPerformance:
    """MPC solver performance during catch sequences."""

    def test_no_consecutive_failures(self, plant, mpc):
        """MPC should not have excessive consecutive solver failures."""
        coordinator = HandCoordinator()
        sequence, duration = get_catch_sequence('DT1')
        for target, ball in sequence:
            coordinator.submit_target(target, ball)

        _run_catch_sim(plant, mpc, coordinator, duration)

        # Check that MPC didn't enter failure cascade
        assert mpc.consecutive_failures < 5, \
            f"MPC had {mpc.consecutive_failures} consecutive failures"
