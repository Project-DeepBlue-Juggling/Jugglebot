"""Tests for post-catch coordinator state machine (CAUGHT → RETURNING → IDLE).

Verifies that the HandCoordinator issues the correct hand commands and state
transitions after a ball is captured, including the throw-catch flow.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from hand.coordinator import (
    HandCoordinator,
    HandPhase,
    DynamicTarget,
    BallSpawn,
    BallRelease,
)
from hand.trajectory import HandCatchSequence, HandThrowSequence, STROKE_MARGIN_MM


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_HOME = np.zeros(6)

_CATCH_POSE = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])


def _make_catch_target(arrival_time: float = 2.0, event_vel: float = 3.0,
                       hold_duration: float = 0.5) -> DynamicTarget:
    return DynamicTarget(
        pose_6dof=_CATCH_POSE.copy(),
        arrival_time=arrival_time,
        event_vel_mps=event_vel,
        hold_duration=hold_duration,
        mode='catch',
    )


def _advance_to_phase(coord: HandCoordinator, phase: HandPhase,
                      sim_time: float, dt: float = 0.005) -> float:
    """Step the coordinator forward until it reaches *phase* or time limit."""
    t = sim_time
    for _ in range(10000):
        target, hand_cmd = coord.update(t, _CATCH_POSE, hand_pos_mm=320.0)
        if coord.phase == phase:
            return t
        t += dt
    raise RuntimeError(f"Never reached {phase} (stuck at {coord.phase})")


# ---------------------------------------------------------------------------
# Catch-only flow: HOLDING → CAUGHT → RETURNING → IDLE
# ---------------------------------------------------------------------------

class TestCatchOnlyPostCatch:
    """Post-catch behaviour in the catch-only (legacy) flow."""

    def test_caught_issues_home_command(self):
        """After notify_capture, the CAUGHT phase must issue 'home'."""
        coord = HandCoordinator(home_pose=_HOME)
        coord.submit_target(_make_catch_target(arrival_time=1.0))

        # Advance past IDLE → PRIMING → APPROACHING → HOLDING
        t = _advance_to_phase(coord, HandPhase.HOLDING, sim_time=0.0)

        # Simulate capture while in HOLDING
        coord.notify_capture(t)
        assert coord.phase == HandPhase.CAUGHT

        # Next update must return hand_cmd = 'home'
        target, hand_cmd = coord.update(t + 0.005, _CATCH_POSE, hand_pos_mm=100.0)
        assert hand_cmd == 'home'

    def test_caught_transitions_to_returning(self):
        """CAUGHT must transition to RETURNING on the same update."""
        coord = HandCoordinator(home_pose=_HOME)
        coord.submit_target(_make_catch_target(arrival_time=1.0))

        t = _advance_to_phase(coord, HandPhase.HOLDING, sim_time=0.0)
        coord.notify_capture(t)

        # The update that processes CAUGHT should also enter RETURNING
        coord.update(t + 0.005, _CATCH_POSE, hand_pos_mm=100.0)
        assert coord.phase == HandPhase.RETURNING

    def test_returning_reaches_idle(self):
        """RETURNING must transition to IDLE after its duration."""
        coord = HandCoordinator(home_pose=_HOME)
        coord.submit_target(_make_catch_target(arrival_time=1.0))

        t = _advance_to_phase(coord, HandPhase.HOLDING, sim_time=0.0)
        coord.notify_capture(t)

        # Process CAUGHT → RETURNING
        coord.update(t + 0.005, _CATCH_POSE, hand_pos_mm=100.0)
        assert coord.phase == HandPhase.RETURNING

        # Step forward > 1.0s to complete RETURNING
        t_return = t + 0.005
        target, hand_cmd = coord.update(t_return + 1.1, _CATCH_POSE, hand_pos_mm=0.0)
        assert coord.phase == HandPhase.IDLE

    def test_no_hand_commands_during_returning(self):
        """No hand commands should be issued during RETURNING."""
        coord = HandCoordinator(home_pose=_HOME)
        coord.submit_target(_make_catch_target(arrival_time=1.0))

        t = _advance_to_phase(coord, HandPhase.HOLDING, sim_time=0.0)
        coord.notify_capture(t)

        # CAUGHT → RETURNING
        coord.update(t + 0.005, _CATCH_POSE, hand_pos_mm=100.0)

        # Step through RETURNING — all hand_cmd should be None
        for i in range(1, 50):
            t_step = t + 0.005 + i * 0.005
            if t_step > t + 0.005 + 1.0:
                break
            target, hand_cmd = coord.update(t_step, _CATCH_POSE, hand_pos_mm=0.0)
            assert hand_cmd is None, \
                f"Expected no hand command during RETURNING, got {hand_cmd} at step {i}"

    def test_missed_catch_issues_prime(self):
        """When hold expires without capture, hand goes to 'prime'.

        The timeout is deferred by one step (to give notify_capture a chance),
        so 'prime' is issued on the second update after the hold end time.
        """
        coord = HandCoordinator(home_pose=_HOME)
        coord.submit_target(_make_catch_target(arrival_time=1.0, hold_duration=0.5))

        t = _advance_to_phase(coord, HandPhase.HOLDING, sim_time=0.0)

        # Do NOT call notify_capture — let hold expire
        # First update past hold_end: sets _hold_expired flag, no command yet
        target, hand_cmd = coord.update(t + 0.6, _CATCH_POSE, hand_pos_mm=320.0)
        assert hand_cmd is None
        assert coord.phase == HandPhase.HOLDING

        # Second update: deferred timeout fires, issues 'prime'
        target, hand_cmd = coord.update(t + 0.605, _CATCH_POSE, hand_pos_mm=320.0)
        assert hand_cmd == 'prime'
        assert coord.phase == HandPhase.RETURNING

    def test_capture_on_same_step_as_timeout(self):
        """Capture on the same step as hold expiry must still succeed.

        This is the race condition: update() sets _hold_expired, then
        the main loop calls notify_capture() before the next update().
        The capture must win over the deferred timeout.
        """
        coord = HandCoordinator(home_pose=_HOME)
        coord.submit_target(_make_catch_target(arrival_time=1.0, hold_duration=0.5))

        t = _advance_to_phase(coord, HandPhase.HOLDING, sim_time=0.0)

        # Step past hold end — sets _hold_expired flag
        coord.update(t + 0.6, _CATCH_POSE, hand_pos_mm=100.0)
        assert coord.phase == HandPhase.HOLDING  # still holding (deferred)

        # Main loop detects capture between updates
        coord.notify_capture(t + 0.6)
        assert coord.phase == HandPhase.CAUGHT

        # Next update: CAUGHT issues 'home', NOT 'prime'
        target, hand_cmd = coord.update(t + 0.605, _CATCH_POSE, hand_pos_mm=100.0)
        assert hand_cmd == 'home'
        assert coord.phase == HandPhase.RETURNING

    def test_capture_event_recorded(self):
        """notify_capture should record a HandEvent with captured=True."""
        coord = HandCoordinator(home_pose=_HOME)
        coord.submit_target(_make_catch_target(arrival_time=1.0))

        t = _advance_to_phase(coord, HandPhase.HOLDING, sim_time=0.0)
        coord.notify_capture(t)

        capture_events = [e for e in coord.events if e.captured]
        assert len(capture_events) == 1
        assert capture_events[0].time == t


# ---------------------------------------------------------------------------
# Throw-catch flow: full cycle through to post-catch
# ---------------------------------------------------------------------------

class TestThrowCatchPostCatch:
    """Post-catch behaviour in the throw-catch flow."""

    def _make_tc_coordinator(self):
        """Build a coordinator with a simple vertical throw-catch plan."""
        from hand.planner import ThrowCatchPlanner

        coord = HandCoordinator(home_pose=_HOME)
        planner = ThrowCatchPlanner()
        plan = planner.plan(
            throw_pos_mm=np.array([0.0, 0.0, 800.0]),
            catch_pos_mm=np.array([0.0, 0.0, 800.0]),
            throw_time=1.5,
            catch_time=2.5,
        )
        coord.submit_throw_catch(plan)
        return coord, plan

    def test_throw_catch_reaches_holding(self):
        """The full throw-catch flow should reach HOLDING at catch_time."""
        coord, plan = self._make_tc_coordinator()
        t = _advance_to_phase(coord, HandPhase.HOLDING, sim_time=0.0)
        assert t <= 2.6, f"HOLDING reached too late: {t:.3f}s"

    def test_throw_catch_caught_issues_home(self):
        """After capture in throw-catch flow, hand_cmd must be 'home'."""
        coord, plan = self._make_tc_coordinator()
        t = _advance_to_phase(coord, HandPhase.HOLDING, sim_time=0.0)

        coord.notify_capture(t)
        assert coord.phase == HandPhase.CAUGHT

        target, hand_cmd = coord.update(t + 0.005, _CATCH_POSE, hand_pos_mm=50.0)
        assert hand_cmd == 'home'
        assert coord.phase == HandPhase.RETURNING

    def test_throw_catch_full_cycle_to_idle(self):
        """Full cycle: APPROACHING_THROW → ... → CAUGHT → RETURNING → IDLE."""
        coord, plan = self._make_tc_coordinator()

        # Walk through to HOLDING
        t = _advance_to_phase(coord, HandPhase.HOLDING, sim_time=0.0)

        # Capture and walk to IDLE
        coord.notify_capture(t)
        coord.update(t + 0.005, _CATCH_POSE, hand_pos_mm=50.0)
        assert coord.phase == HandPhase.RETURNING

        target, hand_cmd = coord.update(t + 1.5, _CATCH_POSE, hand_pos_mm=0.0)
        assert coord.phase == HandPhase.IDLE

    def test_throw_catch_no_spurious_prime_after_catch(self):
        """After capture in throw-catch, 'prime' must NOT be issued."""
        coord, plan = self._make_tc_coordinator()
        t = _advance_to_phase(coord, HandPhase.HOLDING, sim_time=0.0)

        coord.notify_capture(t)

        # Collect all hand commands from CAUGHT through RETURNING to IDLE
        hand_cmds = []
        t_step = t + 0.005
        for _ in range(500):
            target, hand_cmd = coord.update(t_step, _CATCH_POSE, hand_pos_mm=0.0)
            if hand_cmd is not None:
                hand_cmds.append(hand_cmd)
            if coord.phase == HandPhase.IDLE:
                break
            t_step += 0.005

        assert coord.phase == HandPhase.IDLE, \
            f"Did not reach IDLE, stuck at {coord.phase}"
        assert 'prime' not in hand_cmds, \
            f"Unexpected 'prime' in post-catch commands: {hand_cmds}"
        assert 'home' in hand_cmds, \
            f"Expected 'home' in post-catch commands: {hand_cmds}"

    def test_ball_release_cmd_during_approaching_throw(self):
        """BallRelease must be issued when sim_time >= ball_release_time."""
        coord, plan = self._make_tc_coordinator()

        # Advance to APPROACHING_THROW
        t = _advance_to_phase(coord, HandPhase.APPROACHING_THROW, sim_time=0.0)

        # Step until we get a BallRelease command
        ball_releases = []
        for _ in range(1000):
            target, hand_cmd = coord.update(t, _CATCH_POSE, hand_pos_mm=200.0)
            if isinstance(hand_cmd, BallRelease):
                ball_releases.append(hand_cmd)
                break
            t += 0.005

        assert len(ball_releases) == 1, "Expected exactly one BallRelease"
        assert ball_releases[0].velocity_mms is not None

    def test_throw_hand_sequence_issued(self):
        """A HandThrowSequence must be issued during APPROACHING_THROW."""
        coord, plan = self._make_tc_coordinator()

        hand_seqs = []
        t = 0.0
        for _ in range(2000):
            target, hand_cmd = coord.update(t, _CATCH_POSE, hand_pos_mm=0.0)
            if isinstance(hand_cmd, HandThrowSequence):
                hand_seqs.append(hand_cmd)
            if coord.phase in (HandPhase.THROWING, HandPhase.TRANSITIONING):
                break
            t += 0.005

        assert len(hand_seqs) >= 1, "Expected HandThrowSequence during APPROACHING_THROW"

    def test_catch_hand_sequence_issued(self):
        """A HandCatchSequence must be issued at the THROWING→TRANSITIONING transition."""
        coord, plan = self._make_tc_coordinator()

        catch_seqs = []
        t = 0.0
        for _ in range(5000):
            target, hand_cmd = coord.update(t, _CATCH_POSE, hand_pos_mm=200.0)
            if isinstance(hand_cmd, HandCatchSequence):
                catch_seqs.append(hand_cmd)
            if coord.phase == HandPhase.HOLDING:
                break
            t += 0.005

        assert len(catch_seqs) >= 1, "Expected HandCatchSequence at throw→catch transition"


# ---------------------------------------------------------------------------
# Planner input validation
# ---------------------------------------------------------------------------

class TestPlannerValidation:
    """Edge-case input validation for ThrowCatchPlanner."""

    def test_negative_throw_time_rejected(self):
        """throw_time < 0 should raise ValueError."""
        from hand.planner import ThrowCatchPlanner

        planner = ThrowCatchPlanner()
        with pytest.raises(ValueError, match="non-negative"):
            planner.plan(
                throw_pos_mm=np.array([0.0, 0.0, 800.0]),
                catch_pos_mm=np.array([0.0, 0.0, 800.0]),
                throw_time=-1.0,
                catch_time=1.0,
            )

    def test_catch_before_throw_rejected(self):
        """catch_time <= throw_time should raise ValueError."""
        from hand.planner import ThrowCatchPlanner

        planner = ThrowCatchPlanner()
        with pytest.raises(ValueError, match="after throw_time"):
            planner.plan(
                throw_pos_mm=np.array([0.0, 0.0, 800.0]),
                catch_pos_mm=np.array([0.0, 0.0, 800.0]),
                throw_time=2.0,
                catch_time=1.0,
            )
