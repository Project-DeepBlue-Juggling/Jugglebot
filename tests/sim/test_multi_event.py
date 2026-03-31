"""Tests for multi-event MPC lookahead (Issue 3).

Validates:
  - TossLoopController._build_ref_events: correct event construction
    across all cycle phases (approach, flight, hold, transition)
  - ContinuousThrowCatchSource: ref_events pass-through to TargetCommand
  - MPC integration: multi-event references are consumed by solve()
"""

from __future__ import annotations

import numpy as np
import pytest

from controller.target import ReferenceEvent
from input.toss_loop import TossLoopController, _Phase, _State

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


# ---------------------------------------------------------------------------
# Tests: MPC integration
# ---------------------------------------------------------------------------

class TestMPCIntegration:
    """Test that multi-event references work end-to-end with the MPC."""

    @pytest.fixture
    def plant(self):
        from plant.mujoco_plant import MuJoCoPlant
        return MuJoCoPlant()

    @pytest.fixture
    def mpc(self, plant):
        from controller.mpc import MPCController
        from controller.params import MPCParams
        params = MPCParams(max_cpu_time=2.0, max_iter=500)
        return MPCController.from_plant(params, plant)

    def test_mpc_accepts_multi_event_list(self, plant, mpc):
        """MPC.solve() should accept a multi-event list without error."""
        state = plant.get_state()
        target_pose = np.zeros(6)

        events = [
            ReferenceEvent(time=state.time + 0.3, pose=np.zeros(6)),
            ReferenceEvent(time=state.time + 0.9, pose=np.array([10, 0, 0, 0, 0, 0.0])),
            ReferenceEvent(time=state.time + 1.5, pose=np.zeros(6)),
        ]

        cmd, cmd_vel, diag = mpc.solve(
            state, target_pose, ref_events=events)

        assert diag['status'] in ('Solve_Succeeded', 'Solved_To_Acceptable_Level')

    def test_multi_event_sees_future(self, plant, mpc):
        """With multi-event refs, the MPC should plan toward future events.

        Compare: single-event (catch only) vs multi-event (catch then throw).
        The multi-event case should produce different trajectory predictions
        because the MPC sees the subsequent throw event.
        """
        state = plant.get_state()
        t = state.time

        catch_pose = np.array([20.0, 0, -10.0, 0, 0, 0])
        throw_pose = np.array([-20.0, 0, -10.0, 0, 0, 0])

        # Single event: just catch
        single_events = [
            ReferenceEvent(time=t + 0.5, pose=catch_pose, twist=np.zeros(6)),
        ]

        cmd_single, _, diag_single = mpc.solve(
            state, catch_pose, ref_events=single_events)

        # Multi-event: catch then throw
        multi_events = [
            ReferenceEvent(time=t + 0.5, pose=catch_pose, twist=np.zeros(6)),
            ReferenceEvent(time=t + 1.2, pose=throw_pose, twist=np.zeros(6)),
        ]

        cmd_multi, _, diag_multi = mpc.solve(
            state, catch_pose, ref_events=multi_events)

        # Both should solve successfully
        assert diag_single['status'] in ('Solve_Succeeded', 'Solved_To_Acceptable_Level')
        assert diag_multi['status'] in ('Solve_Succeeded', 'Solved_To_Acceptable_Level')

        # The predicted trajectories should differ because the multi-event
        # case is aware of the throw coming after the catch.  We compare
        # the final predicted poses (which should lean toward throw_pose
        # in the multi-event case).
        poses_single = mpc.predicted_poses
        poses_multi = mpc.predicted_poses  # gets overwritten by last solve

        # Re-solve to get both predictions cleanly
        mpc.solve(state, catch_pose, ref_events=single_events)
        final_single = mpc.predicted_poses[-1].copy()

        mpc.solve(state, catch_pose, ref_events=multi_events)
        final_multi = mpc.predicted_poses[-1].copy()

        # The multi-event final pose should be closer to throw_pose (x=-20)
        # than the single-event one (which only sees catch at x=+20)
        dist_single_to_throw = np.linalg.norm(final_single[:3] - throw_pose[:3])
        dist_multi_to_throw = np.linalg.norm(final_multi[:3] - throw_pose[:3])

        assert dist_multi_to_throw < dist_single_to_throw, (
            f"Multi-event final pose should be closer to throw_pose. "
            f"Single: {dist_single_to_throw:.1f}mm, Multi: {dist_multi_to_throw:.1f}mm"
        )

    def test_multi_event_with_boosted_weights(self, plant, mpc):
        """Solve succeeds with ref_events and boosted velocity weights."""
        state = plant.get_state()
        t = state.time

        events = [
            ReferenceEvent(time=t + 0.3, pose=np.zeros(6)),
            ReferenceEvent(time=t + 1.0, pose=np.array([10, 0, 0, 0, 0, 0.0])),
        ]

        cmd, cmd_vel, diag = mpc.solve(
            state, np.zeros(6),
            ref_events=events,
            boost_vel_weights=True,
        )

        assert diag['status'] in ('Solve_Succeeded', 'Solved_To_Acceptable_Level')
        assert np.all(np.isfinite(cmd))
