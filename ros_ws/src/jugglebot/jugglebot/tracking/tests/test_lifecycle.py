"""Tests for ball lifecycle transitions managed by BallTracker."""

import numpy as np
import pytest

from jugglebot.tracking.ballistics import GRAVITY_MMPS2
from jugglebot.tracking.ball import BallStatus, TrackingConfidence
from jugglebot.tracking.matcher import BallTracker


DT = 0.005
LANDING_Z = 734.3


def _ballistic_pos(pos0, vel0, t):
    return np.array([
        pos0[0] + vel0[0] * t,
        pos0[1] + vel0[1] * t,
        pos0[2] + vel0[2] * t - 0.5 * GRAVITY_MMPS2 * t * t,
    ])


class TestToBeThrown:
    """TO_BE_THROWN → IN_FLIGHT transition."""

    def test_transitions_at_throw_time(self):
        tracker = BallTracker(dt=DT, landing_z=LANDING_Z)
        tracker.handle_announcement(
            initial_position=np.array([0.0, 0.0, 500.0]),
            initial_velocity=np.array([0.0, 0.0, 3000.0]),
            throw_time=1.0,
        )

        # Before throw_time
        tracker.process_frame([], 0.5)
        assert tracker.get_ball(1).status == BallStatus.TO_BE_THROWN

        # Exactly at throw_time
        tracker.process_frame([], 1.0)
        assert tracker.get_ball(1).status == BallStatus.IN_FLIGHT

    def test_does_not_transition_early(self):
        tracker = BallTracker(dt=DT, landing_z=LANDING_Z)
        tracker.handle_announcement(
            initial_position=np.array([0.0, 0.0, 500.0]),
            initial_velocity=np.array([0.0, 0.0, 3000.0]),
            throw_time=2.0,
        )

        tracker.process_frame([], 1.999)
        assert tracker.get_ball(1).status == BallStatus.TO_BE_THROWN


class TestCaught:
    """IN_FLIGHT → CAUGHT: marker disappears after landing_time + grace."""

    def test_caught_after_landing_grace(self):
        """Confirmed ball that disappears around landing time → CAUGHT."""
        tracker = BallTracker(
            dt=DT,
            landing_z=LANDING_Z,
            caught_grace_s=0.200,
            missed_frames_to_lose=10,
        )

        pos0 = np.array([0.0, 0.0, 900.0])
        vel0 = np.array([0.0, 0.0, -500.0])  # Descending
        throw_time = 0.0

        tracker.handle_announcement(
            initial_position=pos0,
            initial_velocity=vel0,
            throw_time=throw_time,
            destination="jugglebot",
        )

        # Feed markers at DT increments until ball approaches landing plane
        t = DT
        for i in range(200):
            pos = _ballistic_pos(pos0, vel0, t)
            if pos[2] < LANDING_Z + 20:
                # Stop feeding markers here (simulating ball landing on platform)
                break
            tracker.process_frame([pos], t)
            t += DT

        ball = tracker.get_ball(1)
        assert ball.tracking == TrackingConfidence.CONFIRMED
        landing_time = ball.landing_time

        # Continue advancing in DT increments with no markers
        # until well past landing_time + grace
        while t < landing_time + 0.5:
            tracker.process_frame([], t)
            t += DT

        ball = tracker.get_ball(1)
        assert ball.status == BallStatus.CAUGHT


class TestUnknown:
    """IN_FLIGHT → UNKNOWN: lost tracking for too long."""

    def test_unknown_after_max_missed_frames(self):
        """Ball that goes untracked for max_frames → UNKNOWN."""
        max_frames = 50  # Shorter for testing
        tracker = BallTracker(
            dt=DT,
            landing_z=LANDING_Z,
            max_frames_without_measurement=max_frames,
        )

        pos0 = np.array([0.0, 0.0, 800.0])
        vel0 = np.array([1000.0, 0.0, 2000.0])
        throw_time = 0.0

        tracker.handle_announcement(
            initial_position=pos0,
            initial_velocity=vel0,
            throw_time=throw_time,
        )

        # Get the ball confirmed
        for i in range(10):
            t = (i + 1) * DT
            pos = _ballistic_pos(pos0, vel0, t)
            tracker.process_frame([pos], t)

        # Now provide no markers for max_frames
        t = 11 * DT
        for i in range(max_frames + 5):
            tracker.process_frame([], t)
            t += DT

        ball = tracker.get_ball(1)
        assert ball.status == BallStatus.UNKNOWN


class TestTerminalCleanup:
    """Terminal balls cleaned up after retention period."""

    def test_cleanup_after_retention(self):
        """CAUGHT ball should be removed after terminal_retention_s."""
        tracker = BallTracker(
            dt=DT,
            landing_z=LANDING_Z,
            caught_grace_s=0.050,
            missed_frames_to_lose=5,
            terminal_retention_s=1.0,
            max_frames_without_measurement=10,
        )

        pos0 = np.array([0.0, 0.0, 900.0])
        vel0 = np.array([0.0, 0.0, -500.0])
        throw_time = 0.0

        tracker.handle_announcement(
            initial_position=pos0,
            initial_velocity=vel0,
            throw_time=throw_time,
        )

        # Get confirmed
        t = DT
        for i in range(10):
            pos = _ballistic_pos(pos0, vel0, t)
            if pos[2] < LANDING_Z - 100:
                break
            tracker.process_frame([pos], t)
            t += DT

        ball = tracker.get_ball(1)
        landing_time = ball.landing_time

        # Stop providing markers, advance past landing grace
        t = landing_time + 0.2
        for _ in range(20):
            tracker.process_frame([], t)
            t += DT

        # Should be terminal now
        ball = tracker.get_ball(1)
        assert ball is not None
        assert ball.status in (BallStatus.CAUGHT, BallStatus.UNKNOWN)

        # Advance past retention period
        t += 2.0
        tracker.process_frame([], t)

        # Ball should be cleaned up
        assert tracker.get_ball(1) is None

    def test_terminal_ball_visible_during_retention(self):
        """Terminal ball should still be accessible during retention window."""
        tracker = BallTracker(
            dt=DT,
            landing_z=LANDING_Z,
            max_frames_without_measurement=5,
            terminal_retention_s=2.0,
        )

        tracker.handle_announcement(
            initial_position=np.array([0.0, 0.0, 800.0]),
            initial_velocity=np.array([0.0, 0.0, 2000.0]),
            throw_time=0.0,
        )

        # Get confirmed
        for i in range(5):
            t = (i + 1) * DT
            tracker.process_frame([np.array([0.0, 0.0, 800.0 + 2000.0 * t])], t)

        # Lose tracking
        t = 6 * DT
        for _ in range(10):
            tracker.process_frame([], t)
            t += DT

        ball = tracker.get_ball(1)
        assert ball is not None
        assert ball.status in (BallStatus.UNKNOWN, BallStatus.IN_FLIGHT)

        # If terminal, should still be accessible for 2 seconds
        if ball.status == BallStatus.UNKNOWN:
            t += 1.0  # Within retention
            tracker.process_frame([], t)
            assert tracker.get_ball(1) is not None


class TestMultipleBalls:
    """Multiple simultaneous balls."""

    def test_two_announced_balls(self):
        """Two announced balls tracked independently."""
        tracker = BallTracker(dt=DT, landing_z=LANDING_Z)

        # Start above landing_z + min_height_above_landing (734.3 + 50 = 784.3)
        pos1 = np.array([-200.0, 0.0, 900.0])
        vel1 = np.array([500.0, 0.0, 2000.0])
        pos2 = np.array([200.0, 0.0, 900.0])
        vel2 = np.array([-500.0, 0.0, 2000.0])

        id1 = tracker.handle_announcement(pos1, vel1, throw_time=0.0)
        id2 = tracker.handle_announcement(pos2, vel2, throw_time=0.0)

        assert id1 != id2

        # Feed both markers
        for i in range(20):
            t = (i + 1) * DT
            m1 = _ballistic_pos(pos1, vel1, t)
            m2 = _ballistic_pos(pos2, vel2, t)
            tracker.process_frame([m1, m2], t)

        b1 = tracker.get_ball(id1)
        b2 = tracker.get_ball(id2)
        assert b1 is not None and b2 is not None
        assert b1.tracking == TrackingConfidence.CONFIRMED
        assert b2.tracking == TrackingConfidence.CONFIRMED
        # They should be at different positions
        assert np.linalg.norm(b1.position - b2.position) > 100.0
