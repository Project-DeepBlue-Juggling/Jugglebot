"""Tests for jugglebot.can.motor_state.MotorStateTracker."""

import threading
import time

import pytest

from jugglebot.can.motor_state import (
    ALL_AXES,
    BB_AXES,
    HAND_AXIS,
    JUGGLEBOT_AXES,
    LEG_AXES,
    NUM_AXES,
    MotorStateTracker,
)


# ════════════════════════════════════════════════════════════════
# Initialisation
# ════════════════════════════════════════════════════════════════


class TestInit:
    def test_states_length(self):
        tracker = MotorStateTracker()
        assert len(tracker._states) == NUM_AXES

    def test_last_states_length(self):
        tracker = MotorStateTracker()
        assert len(tracker.last_states) == NUM_AXES

    def test_heartbeats_only_jugglebot_axes(self):
        tracker = MotorStateTracker()
        assert set(tracker.received_heartbeats.keys()) == set(JUGGLEBOT_AXES)

    def test_heartbeats_all_false(self):
        tracker = MotorStateTracker()
        assert all(v is False for v in tracker.received_heartbeats.values())

    def test_error_flags_cleared(self):
        tracker = MotorStateTracker()
        assert tracker.fatal_error is False
        assert tracker.undervoltage_error is False
        assert tracker.fatal_can_error is False

    def test_soft_reset_attempts_zero(self):
        tracker = MotorStateTracker()
        assert tracker.soft_reset_attempts == 0

    def test_encoder_search_feedback_length(self):
        tracker = MotorStateTracker()
        assert len(tracker.encoder_search_feedback) == len(LEG_AXES)

    def test_encoder_search_feedback_all_none(self):
        tracker = MotorStateTracker()
        assert all(f is None for f in tracker.encoder_search_feedback)


# ════════════════════════════════════════════════════════════════
# update()
# ════════════════════════════════════════════════════════════════


class TestUpdate:
    def test_single_field(self):
        tracker = MotorStateTracker()
        tracker.update(0, pos_estimate=1.5)
        assert tracker.get_field(0, 'pos_estimate') == 1.5

    def test_multiple_fields(self):
        tracker = MotorStateTracker()
        tracker.update(0, pos_estimate=1.5, vel_estimate=0.1)
        assert tracker.get_field(0, 'pos_estimate') == 1.5
        assert tracker.get_field(0, 'vel_estimate') == 0.1

    def test_different_axes(self):
        tracker = MotorStateTracker()
        tracker.update(0, pos_estimate=1.0)
        tracker.update(3, pos_estimate=3.0)
        assert tracker.get_field(0, 'pos_estimate') == 1.0
        assert tracker.get_field(3, 'pos_estimate') == 3.0

    def test_out_of_range_positive_ignored(self):
        tracker = MotorStateTracker()
        tracker.update(99, pos_estimate=1.0)  # Should not raise

    def test_negative_axis_ignored(self):
        tracker = MotorStateTracker()
        tracker.update(-1, pos_estimate=1.0)  # Should not raise

    def test_boundary_axis_last_valid(self):
        tracker = MotorStateTracker()
        last = NUM_AXES - 1
        tracker.update(last, pos_estimate=2.0)
        assert tracker.get_field(last, 'pos_estimate') == 2.0

    def test_boolean_field(self):
        tracker = MotorStateTracker()
        tracker.update(0, trajectory_done=True)
        assert tracker.get_field(0, 'trajectory_done') is True

    def test_integer_field(self):
        tracker = MotorStateTracker()
        tracker.update(0, current_state=8)
        assert tracker.get_field(0, 'current_state') == 8


# ════════════════════════════════════════════════════════════════
# get_states()
# ════════════════════════════════════════════════════════════════


class TestGetStates:
    def test_returns_correct_length(self):
        tracker = MotorStateTracker()
        states = tracker.get_states()
        assert len(states) == NUM_AXES

    def test_returns_list_copy(self):
        tracker = MotorStateTracker()
        states = tracker.get_states()
        assert states is not tracker._states

    def test_reflects_updates(self):
        tracker = MotorStateTracker()
        tracker.update(2, pos_estimate=3.14)
        states = tracker.get_states()
        assert states[2].pos_estimate == 3.14

    def test_updates_last_states(self):
        tracker = MotorStateTracker()
        tracker.update(0, vel_estimate=0.5)
        tracker.get_states()
        assert tracker.last_states[0].vel_estimate == 0.5


# ════════════════════════════════════════════════════════════════
# get_field()
# ════════════════════════════════════════════════════════════════


class TestGetField:
    def test_valid_read(self):
        tracker = MotorStateTracker()
        tracker.update(1, bus_voltage=48.0)
        assert tracker.get_field(1, 'bus_voltage') == 48.0

    def test_out_of_range_returns_none(self):
        tracker = MotorStateTracker()
        assert tracker.get_field(100, 'pos_estimate') is None

    def test_negative_returns_none(self):
        tracker = MotorStateTracker()
        assert tracker.get_field(-1, 'pos_estimate') is None

    def test_default_value(self):
        tracker = MotorStateTracker()
        assert tracker.get_field(0, 'pos_estimate') == 0.0


# ════════════════════════════════════════════════════════════════
# Heartbeat tracking
# ════════════════════════════════════════════════════════════════


class TestHeartbeatTracking:
    def test_all_received_false_initially(self):
        tracker = MotorStateTracker()
        assert tracker.all_jugglebot_heartbeats_received() is False

    def test_all_received_true_when_all_set(self):
        tracker = MotorStateTracker()
        for axis_id in JUGGLEBOT_AXES:
            tracker.received_heartbeats[axis_id] = True
        assert tracker.all_jugglebot_heartbeats_received() is True

    def test_partial_not_all(self):
        tracker = MotorStateTracker()
        for axis_id in LEG_AXES:  # legs only, missing hand
            tracker.received_heartbeats[axis_id] = True
        assert tracker.all_jugglebot_heartbeats_received() is False

    def test_reset_heartbeats(self):
        tracker = MotorStateTracker()
        for axis_id in JUGGLEBOT_AXES:
            tracker.received_heartbeats[axis_id] = True
        tracker.reset_heartbeats()
        assert all(v is False for v in tracker.received_heartbeats.values())


# ════════════════════════════════════════════════════════════════
# Heartbeat watchdog
# ════════════════════════════════════════════════════════════════


class TestHeartbeatWatchdog:
    def test_first_heartbeat_false_initially(self):
        tracker = MotorStateTracker()
        assert tracker.first_heartbeat_received is False

    def test_record_heartbeat_sets_first_received(self):
        tracker = MotorStateTracker()
        tracker.record_heartbeat(0)
        assert tracker.first_heartbeat_received is True

    def test_record_heartbeat_bb_axis_does_not_set_first(self):
        tracker = MotorStateTracker()
        tracker.record_heartbeat(7)  # BB_PITCH, not a Jugglebot axis
        assert tracker.first_heartbeat_received is False

    def test_any_heartbeat_stale_false_before_first(self):
        tracker = MotorStateTracker()
        assert tracker.any_heartbeat_stale(1.0) is False

    def test_any_heartbeat_stale_false_when_fresh(self):
        tracker = MotorStateTracker()
        tracker.record_heartbeat(0)
        assert tracker.any_heartbeat_stale(10.0) is False

    def test_any_heartbeat_stale_true_after_timeout(self):
        tracker = MotorStateTracker()
        tracker.record_heartbeat(0)
        # Manually backdate the timestamp
        tracker._last_heartbeat_time[0] = time.time() - 5.0
        assert tracker.any_heartbeat_stale(1.0) is True

    def test_reset_heartbeat_tracking(self):
        tracker = MotorStateTracker()
        tracker.record_heartbeat(0)
        assert tracker.first_heartbeat_received is True
        tracker.reset_heartbeat_tracking()
        assert tracker.first_heartbeat_received is False
        assert len(tracker._last_heartbeat_time) == 0


# ════════════════════════════════════════════════════════════════
# Encoder search feedback
# ════════════════════════════════════════════════════════════════


class TestEncoderSearch:
    def test_reset_encoder_search_feedback(self):
        tracker = MotorStateTracker()
        tracker.encoder_search_feedback[0] = True
        tracker.encoder_search_feedback[2] = False
        tracker.reset_encoder_search_feedback()
        assert all(f is None for f in tracker.encoder_search_feedback)
        assert len(tracker.encoder_search_feedback) == len(LEG_AXES)


# ════════════════════════════════════════════════════════════════
# Error helpers
# ════════════════════════════════════════════════════════════════


class TestErrorHelpers:
    def test_clear_error_flags(self):
        tracker = MotorStateTracker()
        tracker.fatal_error = True
        tracker.undervoltage_error = True
        tracker.soft_reset_attempts = 5
        tracker.clear_error_flags()
        assert tracker.fatal_error is False
        assert tracker.undervoltage_error is False
        assert tracker.soft_reset_attempts == 0

    def test_last_error_log_times_creates_new(self):
        tracker = MotorStateTracker()
        times = tracker.last_error_log_times(0)
        assert isinstance(times, dict)
        assert len(times) == 0

    def test_last_error_log_times_returns_existing(self):
        tracker = MotorStateTracker()
        times1 = tracker.last_error_log_times(0)
        times1[512] = 100.0
        times2 = tracker.last_error_log_times(0)
        assert times2[512] == 100.0
        assert times1 is times2


# ════════════════════════════════════════════════════════════════
# Thread safety
# ════════════════════════════════════════════════════════════════


class TestThreadSafety:
    def test_concurrent_updates(self):
        """Many threads updating different axes should not crash."""
        tracker = MotorStateTracker()
        errors = []

        def updater(axis_id):
            try:
                for _ in range(100):
                    tracker.update(axis_id, pos_estimate=float(axis_id))
                    tracker.get_states()
                    tracker.get_field(axis_id, 'pos_estimate')
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=updater, args=(i,)) for i in range(NUM_AXES)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert errors == [], f"Thread errors: {errors}"
