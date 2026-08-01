"""Tests for MPC dynamic target tracking (Phase 5C).

Validates that the MPC correctly tracks timed targets with hand coordination,
ball capture, and return-to-active-pose sequences.  These are integration tests that
run the full MPC + plant + catch coordinator pipeline.
"""

from __future__ import annotations

import numpy as np
import pytest

from sim.plant.mujoco_plant import MuJoCoPlant
from controller import MPCController, MPCParams
from controller.target import flat_target_to_events
from sim.hand.coordinator import HandCoordinator, DynamicTarget, HandPhase
from sim.hand.trajectory import HandCatchSequence
from sim.input.scripted import get_catch_sequence, BallSpawn
from helpers import run_catch_sim, IDLE_POSE


# NIGHTLY TIER — the MPC is operationally dormant (plans/active/refactor-2026-07.md
# Phase 3: jugglebot_launch.py no longer starts motor_guard/motion_bridge_node; the
# leg path is trajectory_node -> teensy_bridge_node -> the Teensy MAX_DEVIATION
# guard). The code is parked, not deleted, so this battery is parked with it: it
# runs nightly via tools/nightly_suite.sh and on `./run_tests.sh --full`, which is
# mandatory before any hardware sitting. Promotion back to per-commit is step 4 of
# the MPC revival.
pytestmark = pytest.mark.nightly


# MPC control rate
CONTROL_DT = 0.025  # 40 Hz


def _run_catch_sim(plant, mpc, coordinator, duration, **kwargs):
    """Thin wrapper around helpers.run_catch_sim for backward compat."""
    return run_catch_sim(plant, mpc, coordinator, duration, **kwargs)


@pytest.fixture
def plant():
    return MuJoCoPlant()


@pytest.fixture
def mpc(plant):
    params = MPCParams(
        max_cpu_time=2.0,
        max_iter=500,
        max_leg_vel_mmps=1000.0,
        prime_solver=False,
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
                current_pose = np.concatenate(
                    [state.platform_pos_mm, state.platform_rot])
                ref_events = flat_target_to_events(
                    current_pose, state.platform_twist,
                    target_out.pose_6dof, state.time,
                    target_twist=target_out.arrival_twist,
                    arrival_time=target_out.arrival_time)
                cmd, _, _ = mpc.solve(
                    state, target_out.pose_6dof, ref_events=ref_events)
            else:
                cmd, _, _ = mpc.solve(state, IDLE_POSE)
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

        # Platform should return near active pose after deceleration
        final_pos_err = np.linalg.norm(result['final_pose'][:3])
        assert final_pos_err < 15.0, \
            f"Final position {final_pos_err:.1f} mm from active pose"


class TestDT8BallMiss:
    """DT8: Ball misses — system returns to active pose gracefully."""

    def test_miss_returns_to_active(self, plant, mpc):
        coordinator = HandCoordinator()
        sequence, duration = get_catch_sequence('DT8')
        for target, ball in sequence:
            coordinator.submit_target(target, ball)

        result = _run_catch_sim(plant, mpc, coordinator, duration)

        # Should not have captured the ball
        assert result['captures'] == 0, "Ball should not have been captured"

        # Platform should return near active pose
        final_pos_err = np.linalg.norm(result['final_pose'][:3])
        assert final_pos_err < 12.0, \
            f"Final position {final_pos_err:.1f} mm from active pose"


class TestFeasibilityChecker:
    """Coarse-horizon feasibility checker."""

    def test_accepts_feasible_target(self, plant):
        """DT1 (moderate pose, 0.9s deadline) should be feasible."""
        from sim.hand.feasibility import FeasibilityChecker
        checker = FeasibilityChecker(plant)
        sequence, _ = get_catch_sequence('DT1')
        target, _ = sequence[0]
        state = plant.get_state()
        feasible, reason = checker.check(state, target.pose_6dof, target.arrival_time)
        assert feasible, f"DT1 should be feasible but got: {reason}"

    def test_rejects_infeasible_target(self, plant):
        """DT4 (extreme pose, 0.3s deadline) should be rejected."""
        from sim.hand.feasibility import FeasibilityChecker
        checker = FeasibilityChecker(plant)
        sequence, _ = get_catch_sequence('DT4')
        target, _ = sequence[0]
        state = plant.get_state()
        feasible, reason = checker.check(state, target.pose_6dof, target.arrival_time)
        assert not feasible, "DT4 should be rejected (infeasible)"
        assert 'pos error' in reason or 'ori error' in reason

    def test_rejects_out_of_stroke(self, plant):
        """Target with extensions beyond stroke should be rejected at IK stage."""
        from sim.hand.feasibility import FeasibilityChecker
        checker = FeasibilityChecker(plant)
        # Extreme Z that exceeds stroke
        extreme_pose = np.array([0, 0, 350, 0, 0, 0])
        state = plant.get_state()
        feasible, reason = checker.check(state, extreme_pose, 2.0)
        assert not feasible, "Extreme Z should be rejected"
        assert 'stroke' in reason

    def test_coordinator_rejects_with_checker(self, plant, mpc):
        """When feasibility checker is wired in, DT4 gets a rejection event."""
        from sim.hand.feasibility import FeasibilityChecker
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


class TestFeasibilityRefEvents:
    """Feasibility checker with event-based references (R5)."""

    def test_catch_uses_flat_reference(self, plant):
        """Catch targets (arrival_twist=None) use the legacy flat-reference path."""
        from sim.hand.feasibility import FeasibilityChecker
        checker = FeasibilityChecker(plant)
        sequence, _ = get_catch_sequence('DT1')
        target, _ = sequence[0]
        state = plant.get_state()
        # DT1 has arrival_twist=None → flat reference (backward compat)
        feasible, reason = checker.check(
            state, target.pose_6dof, target.arrival_time)
        assert feasible, f"DT1 should be feasible: {reason}"

    def test_throw_target_with_ref_events(self, plant):
        """Throw target (nonzero twist) triggers event-based coarse MPC solve."""
        from sim.hand.feasibility import FeasibilityChecker
        checker = FeasibilityChecker(plant)
        state = plant.get_state()
        # Moderate throw target: small Z offset (within stroke), downward velocity
        target_pose = np.array([0.0, 0.0, 10.0, 0.0, 0.0, 0.0])
        target_twist = np.array([0.0, 0.0, -200.0, 0.0, 0.0, 0.0])
        feasible, reason = checker.check(
            state, target_pose, time_available=1.0,
            target_twist=target_twist)
        assert feasible, f"Moderate throw should be feasible: {reason}"

    def test_infeasible_throw_rejected(self, plant):
        """Extreme throw target with tight deadline should be rejected."""
        from sim.hand.feasibility import FeasibilityChecker
        checker = FeasibilityChecker(plant)
        state = plant.get_state()
        # Extreme pose + very short deadline
        target_pose = np.array([40.0, -30.0, -60.0, 0.15, -0.1, 0.0])
        target_twist = np.array([0.0, 0.0, -500.0, 0.0, 0.0, 0.0])
        feasible, reason = checker.check(
            state, target_pose, time_available=0.15,
            target_twist=target_twist)
        assert not feasible, "Extreme throw with 150ms deadline should be rejected"


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


class TestDT2RapidSuccession:
    """DT2: Two catches in rapid succession — tests state reset between targets."""

    def test_both_captures(self, plant, mpc):
        """Both balls should be captured."""
        coordinator = HandCoordinator()
        sequence, duration = get_catch_sequence('DT2')
        for target, ball in sequence:
            coordinator.submit_target(target, ball)

        result = _run_catch_sim(plant, mpc, coordinator, duration)

        assert result['captures'] == 2, \
            f"Expected 2 captures, got {result['captures']}"

    def test_second_arrival(self, plant, mpc):
        """Second target arrival error should be within threshold."""
        coordinator = HandCoordinator()
        sequence, duration = get_catch_sequence('DT2')
        for target, ball in sequence:
            coordinator.submit_target(target, ball)

        result = _run_catch_sim(plant, mpc, coordinator, duration)

        # Need at least 2 events for second arrival check
        assert len(result['events']) >= 2, \
            f"Expected >=2 events, got {len(result['events'])}"
        ev2 = result['events'][1]
        assert ev2.arrival_error_mm < 5.0, \
            f"Second arrival error {ev2.arrival_error_mm:.1f} mm exceeds 5 mm"


class TestDT7CatchThenThrow:
    """DT7: Catch a ball, then throw — tests two-phase dynamic target."""

    def test_catch_completes(self, plant, mpc):
        """At least one capture event should occur during the catch phase."""
        coordinator = HandCoordinator()
        sequence, duration = get_catch_sequence('DT7')
        for target, ball in sequence:
            coordinator.submit_target(target, ball)

        result = _run_catch_sim(plant, mpc, coordinator, duration)

        assert result['captures'] >= 1, \
            f"Expected at least 1 capture, got {result['captures']}"

    def test_throw_phase_activates(self, plant, mpc):
        """Throw phase should activate with nonzero twist target after catch.

        DT7 throw phase: target [0,0,60] with arrival_twist [0,0,-150,0,0,0].
        The coordinator sequences catch→hold→return→throw.

        With tuned Q_vel_lin=0.05 (R3), the MPC achieves meaningful
        velocity on the flat-reference path.  Full velocity tracking
        requires event-based references (ref_events).  This test verifies
        the throw phase activates and the MPC receives the twist target.
        """
        coordinator = HandCoordinator()
        sequence, duration = get_catch_sequence('DT7')
        for target, ball in sequence:
            coordinator.submit_target(target, ball)

        # Add headroom: at 40 Hz the MPC has fewer steps to complete the
        # catch→hold→return→throw pipeline, so the throw phase may start
        # slightly later.
        result = _run_catch_sim(plant, mpc, coordinator, duration + 1.0,
                                record_states=True)

        # Verify the throw phase was entered (target_twist with nonzero Z)
        throw_states = [
            s for s in result['states']
            if s['target_twist'] is not None
            and abs(s['target_twist'][2]) > 1.0
        ]

        assert len(throw_states) > 0, (
            "No throw-phase states found — coordinator never activated "
            "the throw target with nonzero Z twist"
        )


class TestDT6ThrowVelocity:
    """DT6: Velocity at arrival — MPC velocity tracking via flat reference.

    With tuned Q_vel_lin=0.05 (R3), the MPC achieves some velocity via
    the flat-reference path (target_twist at deadline + terminal cost).
    For full velocity tracking, use event-based references (ref_events)
    — see TestDT6ThrowWithRefEvents below.
    """

    def test_twist_target_received(self, plant, mpc):
        """MPC receives the nonzero twist target during DT6 throw phase."""
        coordinator = HandCoordinator()
        sequence, duration = get_catch_sequence('DT6')
        for target, ball in sequence:
            coordinator.submit_target(target, ball)

        result = _run_catch_sim(plant, mpc, coordinator, duration,
                                record_states=True)

        # Verify the throw target with twist was active
        throw_states = [
            s for s in result['states']
            if s['target_twist'] is not None
            and abs(s['target_twist'][2]) > 1.0
        ]
        assert len(throw_states) > 0, \
            "No states with nonzero Z twist target found"

    def test_velocity_direction(self, plant, mpc):
        """Platform Z velocity should be negative (downward) near throw deadline.

        On the flat-reference path, velocity tracking is structurally limited
        (Issue 2): twist_ref=0 before the deadline, then jumps to target_twist
        at/past deadline.  With conservative Q_vel_lin=0.001, the MPC achieves
        some directional velocity but not the full target magnitude.

        Full velocity tracking requires event-based references — see
        TestDT6ThrowWithRefEvents below for the strong assertion.
        """
        coordinator = HandCoordinator()
        sequence, duration = get_catch_sequence('DT6')
        throw_deadline = sequence[0][0].arrival_time
        for target, ball in sequence:
            coordinator.submit_target(target, ball)

        result = _run_catch_sim(plant, mpc, coordinator, duration,
                                record_states=True)

        # Look for any downward velocity within ±0.2s of deadline
        window = [
            s for s in result['states']
            if abs(s['time'] - throw_deadline) < 0.2
        ]
        assert len(window) > 0, "No states near throw deadline"

        vz_values = [s['twist'][2] for s in window]
        min_vz = min(vz_values)

        # Weak direction check: the platform should not be moving upward.
        # The flat-reference path achieves ~-10 to -15 mm/s (limited by
        # conservative velocity weights and inconsistent references).
        assert min_vz < 10.0, (
            f"Platform should not be moving upward near throw deadline, "
            f"got min vz={min_vz:.1f} mm/s"
        )


class TestDT6ThrowWithRefEvents:
    """DT6 throw velocity with event-based references (R3).

    Uses ReferenceEvent to give the MPC a kinematically consistent
    velocity reference.  This eliminates the position/velocity conflict
    inherent in the flat-reference path, enabling accurate throw velocity.
    """

    THROW_POSE = np.array([0.0, 0.0, 60.0, 0.0, 0.0, 0.0])
    THROW_TWIST = np.array([0.0, 0.0, -200.0, 0.0, 0.0, 0.0])
    THROW_TIME = 1.0  # absolute time of throw event

    def _run_throw_with_events(self, plant, duration_s=1.5):
        """Run MPC with a single throw event using ref_events."""
        from controller.target import ReferenceEvent

        params = MPCParams(
            max_cpu_time=2.0,
            max_iter=500,
            max_leg_vel_mmps=1000.0,
            prime_solver=False,
        )
        mpc = MPCController.from_plant(params, plant)

        n_steps = int(duration_s / CONTROL_DT)
        states = []
        for _ in range(n_steps):
            state = plant.get_state()
            pose = np.concatenate([state.platform_pos_mm, state.platform_rot])

            # Single throw event — backward extrapolation gives the MPC
            # a velocity-aware reference that is kinematically consistent.
            events = [ReferenceEvent(
                time=self.THROW_TIME,
                pose=self.THROW_POSE.copy(),
                twist=self.THROW_TWIST.copy(),
            )]

            cmd, cmd_vel, diag = mpc.solve(
                state, self.THROW_POSE,
                ref_events=events,
            )
            plant.command(cmd, vel_mm_s=cmd_vel)
            plant.step(CONTROL_DT)
            states.append({
                'time': state.time,
                'pose': pose.copy(),
                'twist': state.platform_twist.copy(),
                'diag': diag,
            })
        return states

    def test_velocity_magnitude(self, plant):
        """MPC achieves substantial throw velocity with ref_events.

        With event-based references, the position reference is kinematically
        consistent with the velocity reference (backward extrapolation from
        the throw event).  The MPC sees a trajectory that naturally leads
        to the target velocity, rather than fighting between position and
        velocity costs.
        """
        states = self._run_throw_with_events(plant)

        # Check velocity near the throw event time
        window = [s for s in states if abs(s['time'] - self.THROW_TIME) < 0.15]
        assert len(window) > 0, "No states near throw event time"

        vz_values = [s['twist'][2] for s in window]
        min_vz = min(vz_values)

        # With ref_events + tuned weights, the MPC achieves meaningful
        # downward velocity (target: -200 mm/s).  Actuator lag (τ=30ms)
        # and the single-event backward extrapolation limit tracking.
        # The 1.0s event time gives ~50 steps of approach; the MPC
        # balances position and velocity cost across the horizon.
        assert min_vz < -40.0, (
            f"Expected meaningful downward velocity with ref_events, "
            f"got min vz={min_vz:.1f} mm/s (target: -200 mm/s)"
        )

    def test_position_near_target(self, plant):
        """Platform should be near the throw pose at the event time."""
        states = self._run_throw_with_events(plant)

        # Find state closest to throw time
        near_throw = min(states, key=lambda s: abs(s['time'] - self.THROW_TIME))
        pos_err = np.linalg.norm(near_throw['pose'][:3] - self.THROW_POSE[:3])

        # Position tracking should still be reasonable despite velocity cost
        assert pos_err < 15.0, (
            f"Position error {pos_err:.1f} mm at throw time "
            f"(threshold 15 mm)"
        )

    def test_solver_succeeds(self, plant):
        """MPC solver should not have excessive failures during throw."""
        states = self._run_throw_with_events(plant)

        failures = sum(
            1 for s in states if 'fallback' in s['diag'].get('status', '')
        )
        assert failures < 3, (
            f"MPC had {failures} solver failures during throw sequence"
        )
