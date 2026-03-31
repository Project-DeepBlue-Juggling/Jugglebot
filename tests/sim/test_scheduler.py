"""Tests for the EventScheduler (Phase 1: foundation).

Tests cover:
- Single event approach and arrival
- Return to active pose
- Event cancellation
- Live target refinement (update_current_event)
- Event replacement (replace_next_event)
- C2 continuity at segment boundaries
- ref_events sampling
"""

from __future__ import annotations

import numpy as np
import pytest

from controller.scheduler import (
    EventScheduler,
    EventType,
    ScheduledEvent,
    SchedulerPhase,
)
from controller.hermite import quintic_interp_with_accel

# MPC horizon times (simplified: 5 fine + 5 coarse from MPCParams defaults)
_FINE_DT = 0.02
_COARSE_DT = 0.25
_DT_SCHEDULE = [_FINE_DT] * 5 + [_COARSE_DT] * 5
_CUMULATIVE_TIMES = np.array([0.0] + list(np.cumsum(_DT_SCHEDULE)))

ACTIVE_POSE = np.zeros(6)
CONTROL_DT = 0.025  # 40 Hz


def _make_event(
    pose=None, time=1.0, twist=None, event_type=EventType.CATCH,
    event_id=None, hand_vel_mps=None,
):
    if pose is None:
        pose = np.array([50.0, 0.0, 20.0, 0.0, 0.0, 0.0])
    if event_id is None:
        from controller.scheduler import _next_event_id
        event_id = _next_event_id()
    return ScheduledEvent(
        event_id=event_id,
        event_type=event_type,
        pose=np.asarray(pose, dtype=np.float64),
        time=time,
        twist=np.asarray(twist, dtype=np.float64) if twist is not None else None,
        hand_vel_mps=hand_vel_mps,
    )


def _make_scheduler(**kwargs):
    return EventScheduler(
        active_pose=ACTIVE_POSE,
        cumulative_times=_CUMULATIVE_TIMES,
        **kwargs,
    )


class TestSchedulerBasics:
    """Basic lifecycle: submit, approach, arrive, return."""

    def test_starts_idle(self):
        sched = _make_scheduler()
        assert sched.phase == SchedulerPhase.IDLE

    def test_idle_output_holds_active_pose(self):
        sched = _make_scheduler()
        out = sched.update(0.0, ACTIVE_POSE, np.zeros(6))
        assert out.phase == SchedulerPhase.IDLE
        np.testing.assert_array_equal(out.target_command.target_pose, ACTIVE_POSE)
        assert out.target_command.arrival_time is None
        assert out.target_command.ref_events is None

    def test_submit_transitions_to_approaching(self):
        sched = _make_scheduler()
        event = _make_event(time=1.0)
        sched.submit_event(event)
        assert sched.phase == SchedulerPhase.APPROACHING
        assert sched.active_event is event

    def test_approach_produces_ref_events(self):
        sched = _make_scheduler()
        event = _make_event(time=1.0)
        sched.submit_event(event)
        out = sched.update(0.0, ACTIVE_POSE, np.zeros(6))
        assert out.phase == SchedulerPhase.APPROACHING
        assert out.target_command.ref_events is not None
        assert len(out.target_command.ref_events) == len(_CUMULATIVE_TIMES)
        assert out.target_command.arrival_time == 1.0

    def test_arrives_at_event(self):
        """Step past arrival time → HOLDING."""
        sched = _make_scheduler()
        event = _make_event(time=0.5)
        sched.submit_event(event)

        # Step until past arrival
        t = 0.0
        pose = ACTIVE_POSE.copy()
        twist = np.zeros(6)
        while t < 0.5:
            out = sched.update(t, pose, twist)
            t += CONTROL_DT

        # Now past arrival
        out = sched.update(0.51, pose, twist)
        assert out.phase == SchedulerPhase.HOLDING

    def test_holding_with_no_next_holds_event_pose(self):
        sched = _make_scheduler()
        target_pose = np.array([30.0, 10.0, 5.0, 0.01, 0.0, 0.0])
        event = _make_event(pose=target_pose, time=0.5)
        sched.submit_event(event)

        # First update builds the segment (t_start=0.0, duration=0.5)
        sched.update(0.0, ACTIVE_POSE, np.zeros(6))
        # Now step past arrival
        out = sched.update(0.6, ACTIVE_POSE, np.zeros(6))
        assert out.phase == SchedulerPhase.HOLDING
        np.testing.assert_array_almost_equal(
            out.target_command.target_pose, target_pose,
        )

    def test_approach_notification(self):
        sched = _make_scheduler()
        event = _make_event()
        sched.submit_event(event)
        # The notification is set during submit, consumed on first update
        out = sched.update(0.0, ACTIVE_POSE, np.zeros(6))
        assert out.hand_notification is not None
        assert out.hand_notification.notification_type == 'approaching'

    def test_arrival_notification(self):
        sched = _make_scheduler()
        event = _make_event(time=0.1)
        sched.submit_event(event)
        sched.update(0.0, ACTIVE_POSE, np.zeros(6))
        out = sched.update(0.15, ACTIVE_POSE, np.zeros(6))
        assert out.hand_notification is not None
        assert out.hand_notification.notification_type == 'arrived'


class TestReturnToActive:
    """Return to active pose after event completion."""

    def test_begin_return_from_holding(self):
        sched = _make_scheduler()
        event = _make_event(time=0.1)
        sched.submit_event(event)
        # Arrive
        sched.update(0.0, ACTIVE_POSE, np.zeros(6))
        sched.update(0.15, ACTIVE_POSE, np.zeros(6))
        assert sched.phase == SchedulerPhase.HOLDING

        sched.begin_return()
        assert sched.phase == SchedulerPhase.RETURNING

    def test_return_completes_to_idle(self):
        sched = _make_scheduler(return_duration=0.5)
        event = _make_event(time=0.1)
        sched.submit_event(event)

        # Arrive
        sched.update(0.0, ACTIVE_POSE, np.zeros(6))
        sched.update(0.15, ACTIVE_POSE, np.zeros(6))

        sched.begin_return()

        # Step through return
        t = 0.15
        while t < 0.8:
            out = sched.update(t, ACTIVE_POSE, np.zeros(6))
            t += CONTROL_DT

        # Should be back to IDLE
        assert sched.phase == SchedulerPhase.IDLE

    def test_return_produces_ref_events(self):
        sched = _make_scheduler(return_duration=0.5)
        event = _make_event(time=0.1)
        sched.submit_event(event)

        sched.update(0.0, ACTIVE_POSE, np.zeros(6))
        sched.update(0.15, ACTIVE_POSE, np.zeros(6))
        sched.begin_return()

        out = sched.update(0.2, ACTIVE_POSE, np.zeros(6))
        assert out.phase == SchedulerPhase.RETURNING
        assert out.target_command.ref_events is not None


class TestEventCancellation:
    """Cancelling events."""

    def test_cancel_next(self):
        sched = _make_scheduler()
        ev1 = _make_event(time=0.5)
        ev2 = _make_event(time=1.0, pose=np.array([100.0, 0, 0, 0, 0, 0]))
        sched.submit_event(ev1)
        sched.submit_event(ev2)
        assert sched.next_event is ev2

        cancelled = sched.cancel_next()
        assert cancelled is ev2
        assert sched.next_event is None

    def test_cancel_next_returns_none_when_empty(self):
        sched = _make_scheduler()
        ev1 = _make_event(time=0.5)
        sched.submit_event(ev1)
        assert sched.cancel_next() is None

    def test_clear_resets_to_idle(self):
        sched = _make_scheduler()
        ev1 = _make_event(time=0.5)
        sched.submit_event(ev1)
        sched.clear()
        assert sched.phase == SchedulerPhase.IDLE
        assert sched.active_event is None
        assert sched.next_event is None


class TestLiveRefinement:
    """Updating current event during approach (continuous ball tracking)."""

    def test_update_current_replans_segment(self):
        sched = _make_scheduler()
        ev = _make_event(time=1.0, event_id=42)
        sched.submit_event(ev)

        # Step a few times
        out1 = sched.update(0.0, ACTIVE_POSE, np.zeros(6))
        out2 = sched.update(0.1, ACTIVE_POSE, np.zeros(6))

        # Refine: shift landing position
        refined = _make_event(
            pose=np.array([60.0, 10.0, 25.0, 0.0, 0.0, 0.0]),
            time=1.05, event_id=42,
        )
        sched.update_current_event(refined)

        # Next update should produce ref_events targeting the new pose
        out3 = sched.update(0.12, ACTIVE_POSE, np.zeros(6))
        assert out3.target_command.ref_events is not None

        # The last ref_event should be near the refined pose
        last_ref = out3.target_command.ref_events[-1]
        # At horizon end (>1s out), should be heading toward refined pose
        np.testing.assert_array_almost_equal(
            last_ref.pose[:3], refined.pose[:3], decimal=0,
        )

    def test_update_current_wrong_id_ignored(self):
        sched = _make_scheduler()
        ev = _make_event(time=1.0, event_id=42)
        sched.submit_event(ev)
        sched.update(0.0, ACTIVE_POSE, np.zeros(6))

        wrong = _make_event(time=1.0, event_id=999)
        sched.update_current_event(wrong)  # should warn, not crash
        assert sched.active_event.event_id == 42


class TestEventReplacement:
    """Replacing the next event."""

    def test_replace_next_during_transitioning(self):
        """When a next event exists and we arrive at current, scheduler
        auto-advances to TRANSITIONING.  Replace the next event mid-transit."""
        sched = _make_scheduler()
        ev1 = _make_event(time=0.1)
        ev2 = _make_event(time=1.0, pose=np.array([100, 0, 0, 0, 0, 0]))
        sched.submit_event(ev1)
        sched.submit_event(ev2)

        # Arrive at ev1 → auto-advance to TRANSITIONING
        sched.update(0.0, ACTIVE_POSE, np.zeros(6))
        sched.update(0.15, ACTIVE_POSE, np.zeros(6))
        assert sched.phase == SchedulerPhase.TRANSITIONING

        # Replace next mid-transition
        ev3 = _make_event(time=0.8, pose=np.array([-50, 20, 10, 0, 0, 0]))
        sched.replace_next_event(ev3)
        assert sched.next_event is ev3

        # Subsequent update should replan toward ev3
        out = sched.update(0.2, ACTIVE_POSE, np.zeros(6))
        assert out.phase == SchedulerPhase.TRANSITIONING
        assert out.target_command.ref_events is not None


class TestTransitioning:
    """Two-event chaining: current → next."""

    def test_holding_advances_to_transitioning(self):
        sched = _make_scheduler()
        ev1 = _make_event(time=0.1)
        ev2 = _make_event(time=0.8, pose=np.array([100, 0, 0, 0, 0, 0]))
        sched.submit_event(ev1)
        sched.submit_event(ev2)

        # Arrive at ev1
        sched.update(0.0, ACTIVE_POSE, np.zeros(6))
        out = sched.update(0.15, ACTIVE_POSE, np.zeros(6))
        # Should have advanced through HOLDING → TRANSITIONING
        assert sched.phase == SchedulerPhase.TRANSITIONING

    def test_transition_arrives_at_next_event(self):
        sched = _make_scheduler()
        ev1 = _make_event(time=0.1)
        target2 = np.array([100, 0, 0, 0, 0, 0])
        ev2 = _make_event(time=0.6, pose=target2)
        sched.submit_event(ev1)
        sched.submit_event(ev2)

        # Step through
        t = 0.0
        while t < 0.7:
            out = sched.update(t, ACTIVE_POSE, np.zeros(6))
            t += CONTROL_DT

        # After ev2 arrival, should be HOLDING with ev2 as current
        assert sched.phase == SchedulerPhase.HOLDING
        assert sched.active_event is ev2

    def test_transition_produces_ref_events(self):
        sched = _make_scheduler()
        ev1 = _make_event(time=0.1)
        ev2 = _make_event(time=0.8, pose=np.array([100, 0, 0, 0, 0, 0]))
        sched.submit_event(ev1)
        sched.submit_event(ev2)

        # Arrive at ev1, transition starts
        sched.update(0.0, ACTIVE_POSE, np.zeros(6))
        sched.update(0.15, ACTIVE_POSE, np.zeros(6))

        # Step in transition
        out = sched.update(0.3, ACTIVE_POSE, np.zeros(6))
        assert out.phase == SchedulerPhase.TRANSITIONING
        assert out.target_command.ref_events is not None


class TestC2Continuity:
    """Verify C2 continuity of quintic segments."""

    def test_quintic_matches_boundary_conditions(self):
        """Quintic should exactly match p0, v0, a0 at t=0 and p1, v1, a1 at t=T."""
        p0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        v0 = np.array([10.0, 0.0, 5.0, 0.0, 0.0, 0.0])
        a0 = np.array([1.0, -1.0, 0.0, 0.0, 0.0, 0.0])
        p1 = np.array([50.0, 20.0, 30.0, 0.05, 0.0, 0.0])
        v1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        a1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        T = 1.0

        p_start, v_start, a_start = quintic_interp_with_accel(
            p0, v0, a0, p1, v1, a1, T, 0.0,
        )
        np.testing.assert_array_almost_equal(p_start, p0, decimal=10)
        np.testing.assert_array_almost_equal(v_start, v0, decimal=10)
        np.testing.assert_array_almost_equal(a_start, a0, decimal=10)

        p_end, v_end, a_end = quintic_interp_with_accel(
            p0, v0, a0, p1, v1, a1, T, T,
        )
        np.testing.assert_array_almost_equal(p_end, p1, decimal=10)
        np.testing.assert_array_almost_equal(v_end, v1, decimal=10)
        np.testing.assert_array_almost_equal(a_end, a1, decimal=10)

    def test_segment_continuity_at_boundary(self):
        """Two consecutive segments should have matching pos, vel, accel."""
        # Segment 1: rest → event1
        p0 = np.zeros(6)
        v0 = np.zeros(6)
        a0 = np.zeros(6)
        p1 = np.array([50.0, 0.0, 20.0, 0.0, 0.0, 0.0])
        v1 = np.zeros(6)
        a1 = np.zeros(6)
        T1 = 0.5

        # End of segment 1
        pe, ve, ae = quintic_interp_with_accel(p0, v0, a0, p1, v1, a1, T1, T1)

        # Segment 2: event1 → event2 (using end state of seg1 as start)
        p2 = np.array([100.0, 30.0, 10.0, 0.02, 0.0, 0.0])
        v2 = np.zeros(6)
        a2 = np.zeros(6)
        T2 = 0.5

        # Start of segment 2
        ps, vs, a_s = quintic_interp_with_accel(pe, ve, ae, p2, v2, a2, T2, 0.0)

        # C2 continuity: position, velocity, and acceleration must match
        np.testing.assert_array_almost_equal(pe, ps, decimal=10)
        np.testing.assert_array_almost_equal(ve, vs, decimal=10)
        np.testing.assert_array_almost_equal(ae, a_s, decimal=10)

    def test_ref_events_are_time_ordered(self):
        sched = _make_scheduler()
        event = _make_event(time=1.0)
        sched.submit_event(event)

        out = sched.update(0.0, ACTIVE_POSE, np.zeros(6))
        refs = out.target_command.ref_events
        assert refs is not None
        times = [r.time for r in refs]
        assert times == sorted(times)

    def test_ref_events_first_is_now(self):
        sched = _make_scheduler()
        event = _make_event(time=1.0)
        sched.submit_event(event)

        t = 0.3
        out = sched.update(t, ACTIVE_POSE, np.zeros(6))
        refs = out.target_command.ref_events
        assert abs(refs[0].time - t) < 1e-10


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
