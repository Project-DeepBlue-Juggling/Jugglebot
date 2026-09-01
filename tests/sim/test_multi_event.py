"""Tests for multi-event reference lookahead (Issue 3).

Validates:
  - TossLoopController._build_ref_events: correct event construction
    across all cycle phases (approach, flight, hold, transition)
  - ContinuousThrowCatchSource: ref_events pass-through to TargetCommand

The MPC-integration class (multi-event references consumed by
``MPCController.solve()``) was removed 2026-09-01 with the MPC chain;
final implementation at git tag ``mpc-final`` — see
``logbook/2026-09-01-mpc-chain-removed.md``.
"""

from __future__ import annotations

import numpy as np
import pytest

from controller.target import ReferenceEvent
from sim.input.toss_loop import TossLoopController, _Phase, _State

CONTROL_DT = 0.025  # 40 Hz


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_controller(**kwargs) -> TossLoopController:
    """Create a TossLoopController with sensible test defaults."""
    defaults = dict(
        cycle_time=1.2,
        hold_ratio=0.4,
        lateral_spacing_mm=0.0,
        platform_event_speed_ratio=0.0,
    )
    defaults.update(kwargs)
    return TossLoopController(**defaults)


def _advance_to_active(ctrl: TossLoopController, start_time: float = 1.0):
    """Drive the controller through STARTUP into ACTIVE state.

    Returns the sim_time after the transition.
    """
    pose = np.zeros(6)
    hand_pos = 0.0

    # Spawn ball
    ctrl.update(start_time, pose, hand_pos)

    # Wait out the 0.3s startup delay
    t = start_time + 0.35
    ctrl.update(t, pose, hand_pos)

    # The controller should now be ACTIVE
    assert ctrl._state == _State.ACTIVE, f"Expected ACTIVE, got {ctrl._state}"
    return t


# ---------------------------------------------------------------------------
# Tests: _build_ref_events
# ---------------------------------------------------------------------------

class TestBuildRefEvents:
    """Test that _build_ref_events produces correct event lists."""

    def test_no_events_before_active(self):
        """Before the controller is active, ref_events should be None."""
        ctrl = _make_controller()
        assert ctrl._build_ref_events(0.0) is None

    def test_approach_phase_has_approach_and_throw_and_catch(self):
        """During approach, events should include approach-start, throw, catch."""
        ctrl = _make_controller()
        t = _advance_to_active(ctrl)

        # Controller is in APPROACHING_THROW phase
        assert ctrl._phase == _Phase.APPROACHING_THROW

        events = ctrl._build_ref_events(t)
        assert events is not None
        assert len(events) >= 3  # approach-start + throw + catch (+ next cycle)

        # First event should be the approach start (active pose, zero twist)
        assert events[0].time == ctrl._approach_start_time
        np.testing.assert_array_equal(events[0].twist, np.zeros(6))

        # Second event should be the throw
        throw_time = ctrl._last_throw_time
        assert events[1].time == throw_time

        # Third event should be the catch
        catch_time = throw_time + ctrl._air_time
        assert events[2].time == catch_time

    def test_events_are_sorted_by_time(self):
        """All events should be in ascending time order."""
        ctrl = _make_controller()
        t = _advance_to_active(ctrl)

        events = ctrl._build_ref_events(t)
        assert events is not None
        times = [e.time for e in events]
        assert times == sorted(times), f"Events not sorted: {times}"

    def test_flight_phase_has_catch_and_next_events(self):
        """During flight, events should include throw + catch + next cycle."""
        ctrl = _make_controller()
        t = _advance_to_active(ctrl)

        # Advance past throw time into flight
        throw_time = ctrl._last_throw_time
        catch_time = throw_time + ctrl._air_time
        mid_flight = (throw_time + catch_time) / 2.0

        # Manually advance state to simulate flight
        ctrl._phase = _Phase.IN_FLIGHT
        events = ctrl._build_ref_events(mid_flight)

        assert events is not None
        # Should have catch event and possibly next cycle events
        assert any(abs(e.time - catch_time) < 0.001 for e in events), \
            "Catch event missing from flight-phase events"

        # Throw event (now in the past) is kept as interpolation boundary
        assert any(abs(e.time - throw_time) < 0.001 for e in events), \
            "Throw event should be kept as left interpolation boundary"

    def test_next_cycle_events_included(self):
        """When next_plan exists, its events should be in the list."""
        ctrl = _make_controller()
        t = _advance_to_active(ctrl)

        # The controller pre-plans the next cycle
        assert ctrl._next_plan is not None

        events = ctrl._build_ref_events(t)
        assert events is not None

        next_throw_time = ctrl._last_throw_time + ctrl._cycle_time
        next_catch_time = next_throw_time + ctrl._air_time

        # Should include next cycle's throw and catch
        event_times = [e.time for e in events]
        assert any(abs(et - next_throw_time) < 0.001 for et in event_times), \
            f"Next throw event missing. Times: {event_times}"
        assert any(abs(et - next_catch_time) < 0.001 for et in event_times), \
            f"Next catch event missing. Times: {event_times}"

    def test_past_events_kept_as_boundaries(self):
        """Past events are kept as interpolation boundaries."""
        ctrl = _make_controller()
        t = _advance_to_active(ctrl)

        # Query at a time after the throw
        throw_time = ctrl._last_throw_time
        future_t = throw_time + 0.1  # just past throw

        ctrl._phase = _Phase.THROWING
        events = ctrl._build_ref_events(future_t)
        assert events is not None

        # Throw event is in the past but kept as left boundary
        assert any(e.time < future_t for e in events), \
            "Expected at least one past event as interpolation boundary"
        # Events should be time-ordered
        times = [e.time for e in events]
        assert times == sorted(times), "Events must be sorted by time"

    def test_phase_c_events_have_nonzero_twist(self):
        """Phase C events should carry the platform twist."""
        ctrl = _make_controller(
            lateral_spacing_mm=120.0,
            platform_event_speed_ratio=0.8,
        )
        t = _advance_to_active(ctrl)

        events = ctrl._build_ref_events(t)
        assert events is not None

        # Find throw event (second after approach-start)
        throw_time = ctrl._last_throw_time
        throw_events = [e for e in events if abs(e.time - throw_time) < 0.001]
        assert len(throw_events) == 1

        twist = throw_events[0].twist
        assert twist is not None
        # Phase C: platform has lateral velocity at events
        assert np.linalg.norm(twist[:3]) > 0, \
            "Phase C throw event should have nonzero translational twist"

    def test_event_poses_match_plan_targets(self):
        """Event poses should exactly match the plan's target poses."""
        ctrl = _make_controller()
        t = _advance_to_active(ctrl)

        events = ctrl._build_ref_events(t)
        assert events is not None

        throw_time = ctrl._last_throw_time
        catch_time = throw_time + ctrl._air_time

        throw_events = [e for e in events if abs(e.time - throw_time) < 0.001]
        catch_events = [e for e in events if abs(e.time - catch_time) < 0.001]

        assert len(throw_events) == 1
        assert len(catch_events) == 1

        np.testing.assert_array_almost_equal(
            throw_events[0].pose,
            ctrl._plan.throw_target.pose_6dof,
        )
        np.testing.assert_array_almost_equal(
            catch_events[0].pose,
            ctrl._plan.catch_target.pose_6dof,
        )


# ---------------------------------------------------------------------------
# Tests: update() return signature
# ---------------------------------------------------------------------------

class TestUpdateReturnSignature:
    """Test that update() returns 4-tuples with ref_events."""

    def test_startup_returns_4_tuple(self):
        """STARTUP state returns 4-tuple with None ref_events."""
        ctrl = _make_controller()
        result = ctrl.update(0.0, np.zeros(6), 0.0)
        assert len(result) == 4
        assert result[3] is None  # ref_events

    def test_active_returns_ref_events(self):
        """ACTIVE state returns ref_events in 4th element."""
        ctrl = _make_controller()
        t = _advance_to_active(ctrl)

        # Now update in active state
        result = ctrl.update(t + CONTROL_DT, np.zeros(6), 0.0)
        assert len(result) == 4

        target, hand_cmd, ball_spawn, ref_events = result
        assert ref_events is not None
        assert len(ref_events) >= 2
        assert all(isinstance(e, ReferenceEvent) for e in ref_events)

    def test_dropped_returns_none_ref_events(self):
        """DROPPED state returns None ref_events."""
        ctrl = _make_controller()
        ctrl._state = _State.DROPPED
        ctrl._drop_time = 0.0
        result = ctrl.update(0.1, np.zeros(6), 0.0)
        assert len(result) == 4
        assert result[3] is None
