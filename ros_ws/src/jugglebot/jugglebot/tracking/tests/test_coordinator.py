"""Tests for catch_coordinator.py — catch pose, blacklist, ball selection."""

import math

import numpy as np
import pytest

from jugglebot.tracking.ball import Ball, BallStatus, TrackingConfidence
from jugglebot.catch_coordinator import CatchCoordinator


INITIAL_HEIGHT = 574.3
LANDING_Z = INITIAL_HEIGHT + 160.0


def _make_ball(
    ball_id: int = 1,
    landing_pos=None,
    landing_vel=None,
    landing_time: float = 10.0,
    destination: str = "jugglebot",
    status: BallStatus = BallStatus.IN_FLIGHT,
) -> Ball:
    """Create a test ball with sensible defaults."""
    if landing_pos is None:
        landing_pos = np.array([0.0, 0.0, LANDING_Z])
    if landing_vel is None:
        landing_vel = np.array([0.0, 0.0, -3000.0])  # Descending at 3 m/s
    return Ball(
        id=ball_id,
        status=status,
        tracking=TrackingConfidence.CONFIRMED,
        source="ball_butler",
        destination=destination,
        position=np.array([0.0, 0.0, 900.0]),
        velocity=np.array([0.0, 0.0, -2000.0]),
        landing_position=np.array(landing_pos, dtype=np.float64),
        landing_velocity=np.array(landing_vel, dtype=np.float64),
        landing_time=landing_time,
    )


class TestCatchOrientation:
    """Catch pose quaternion from landing velocity."""

    def test_straight_down_identity(self):
        """Ball coming straight down → identity quaternion (no tilt)."""
        coord = CatchCoordinator(initial_height_mm=INITIAL_HEIGHT)
        vel = np.array([0.0, 0.0, -5000.0])

        quat = coord.compute_catch_orientation(vel)
        assert quat is not None
        # Should be identity (or very close)
        np.testing.assert_allclose(quat, [1.0, 0.0, 0.0, 0.0], atol=1e-6)

    def test_angled_approach(self):
        """Ball at 15° from vertical → platform tilts 15°."""
        coord = CatchCoordinator(initial_height_mm=INITIAL_HEIGHT)
        angle = math.radians(15)
        speed = 5000.0
        vel = np.array([speed * math.sin(angle), 0.0, -speed * math.cos(angle)])

        quat = coord.compute_catch_orientation(vel)
        assert quat is not None

        # Verify the quaternion magnitude is 1
        assert np.linalg.norm(quat) == pytest.approx(1.0, abs=1e-6)

        # Verify the rotation angle is ~15°
        # For a quaternion [w, x, y, z], angle = 2 * acos(w)
        rot_angle = 2.0 * math.acos(min(abs(quat[0]), 1.0))
        assert rot_angle == pytest.approx(angle, abs=0.01)

    def test_angle_limit_rejection(self):
        """Ball approach angle > 30° → returns None."""
        coord = CatchCoordinator(
            initial_height_mm=INITIAL_HEIGHT,
            catch_angle_limit_deg=30.0,
        )
        angle = math.radians(35)
        speed = 5000.0
        vel = np.array([speed * math.sin(angle), 0.0, -speed * math.cos(angle)])

        quat = coord.compute_catch_orientation(vel)
        assert quat is None

    def test_zero_velocity(self):
        """Zero velocity → identity quaternion."""
        coord = CatchCoordinator(initial_height_mm=INITIAL_HEIGHT)
        vel = np.array([0.0, 0.0, 0.0])

        quat = coord.compute_catch_orientation(vel)
        assert quat is not None
        np.testing.assert_allclose(quat, [1.0, 0.0, 0.0, 0.0], atol=1e-6)


class TestCatchPose:
    """Full catch command generation."""

    def test_catch_position_frame_conversion(self):
        """Landing position in base frame → target_pos in platform frame."""
        coord = CatchCoordinator(initial_height_mm=INITIAL_HEIGHT)
        ball = _make_ball(
            landing_pos=[50.0, -30.0, LANDING_Z],
            landing_vel=[0.0, 0.0, -3000.0],
        )

        cmd = coord.update([ball], current_time=5.0)
        assert cmd is not None
        # target_pos should be [landing_x, landing_y, landing_z - initial_height]
        assert cmd.target_pos[0] == pytest.approx(50.0)
        assert cmd.target_pos[1] == pytest.approx(-30.0)
        assert cmd.target_pos[2] == pytest.approx(LANDING_Z - INITIAL_HEIGHT)

    def test_stationary_catch(self):
        """Catch velocity should always be zero."""
        coord = CatchCoordinator(initial_height_mm=INITIAL_HEIGHT)
        ball = _make_ball()

        cmd = coord.update([ball], current_time=5.0)
        assert cmd is not None
        np.testing.assert_array_equal(cmd.target_vel, [0.0, 0.0, 0.0])

    def test_uncatchable_angle_no_command(self):
        """Steep approach → no command produced."""
        coord = CatchCoordinator(initial_height_mm=INITIAL_HEIGHT)
        angle = math.radians(40)
        speed = 5000.0
        ball = _make_ball(
            landing_vel=[speed * math.sin(angle), 0.0, -speed * math.cos(angle)],
        )

        cmd = coord.update([ball], current_time=5.0)
        assert cmd is None


class TestBlacklist:
    """Blacklist management."""

    def test_blacklist_after_3_rejections(self):
        """Ball blacklisted after 3 consecutive rejections."""
        coord = CatchCoordinator(
            initial_height_mm=INITIAL_HEIGHT,
            blacklist_rejection_threshold=3,
        )
        ball = _make_ball()

        # First update: produces command
        cmd = coord.update([ball], current_time=5.0)
        assert cmd is not None

        # Report 3 rejections
        for _ in range(3):
            coord.report_rejection_with_position(ball.id, ball.landing_position)

        # Next update: ball should be blacklisted, no command
        cmd = coord.update([ball], current_time=5.0)
        assert cmd is None

    def test_blacklist_reeval_on_landing_shift(self):
        """Blacklisted ball re-evaluated when landing position shifts > 50mm."""
        coord = CatchCoordinator(
            initial_height_mm=INITIAL_HEIGHT,
            blacklist_rejection_threshold=3,
            blacklist_reeval_threshold_mm=50.0,
        )
        ball = _make_ball(landing_pos=[0.0, 0.0, LANDING_Z])

        # Blacklist the ball
        for _ in range(3):
            coord.report_rejection_with_position(ball.id, ball.landing_position)

        # Ball is blacklisted
        cmd = coord.update([ball], current_time=5.0)
        assert cmd is None

        # Landing position shifts by 80mm
        ball.landing_position = np.array([80.0, 0.0, LANDING_Z])
        cmd = coord.update([ball], current_time=5.0)
        # Should now produce a command (re-evaluated)
        assert cmd is not None

    def test_acceptance_resets_rejection_count(self):
        """Acceptance resets the rejection counter."""
        coord = CatchCoordinator(
            initial_height_mm=INITIAL_HEIGHT,
            blacklist_rejection_threshold=3,
        )
        ball = _make_ball()

        # 2 rejections
        coord.report_rejection_with_position(ball.id, ball.landing_position)
        coord.report_rejection_with_position(ball.id, ball.landing_position)

        # 1 acceptance resets
        coord.report_acceptance(ball.id)

        # 2 more rejections (still under threshold)
        coord.report_rejection_with_position(ball.id, ball.landing_position)
        coord.report_rejection_with_position(ball.id, ball.landing_position)

        # Should still be able to catch (4 total rejections but not 3 consecutive)
        cmd = coord.update([ball], current_time=5.0)
        assert cmd is not None

    def test_blacklist_cleanup_on_terminal(self):
        """Blacklist entry cleaned up when ball leaves IN_FLIGHT."""
        coord = CatchCoordinator(
            initial_height_mm=INITIAL_HEIGHT,
            blacklist_rejection_threshold=3,
        )
        ball = _make_ball()

        for _ in range(3):
            coord.report_rejection_with_position(ball.id, ball.landing_position)

        # Ball transitions to CAUGHT
        ball.status = BallStatus.CAUGHT
        coord.update([ball], current_time=5.0)

        # Blacklist should be cleaned up for this ball
        ball.status = BallStatus.IN_FLIGHT  # Hypothetical: same ID back in flight
        cmd = coord.update([ball], current_time=5.0)
        # Blacklist was cleaned when ball was CAUGHT, so it should be catchable now
        assert cmd is not None


class TestBallSelection:
    """Multi-ball selection: earliest landing time, correct filtering."""

    def test_selects_earliest_landing(self):
        """With multiple balls, selects the one landing soonest."""
        coord = CatchCoordinator(initial_height_mm=INITIAL_HEIGHT)
        ball_early = _make_ball(ball_id=1, landing_time=8.0)
        ball_late = _make_ball(ball_id=2, landing_time=12.0)

        cmd = coord.update([ball_late, ball_early], current_time=5.0)
        assert cmd is not None
        assert cmd.ball_id == 1

    def test_filters_wrong_destination(self):
        """Balls with wrong destination are ignored."""
        coord = CatchCoordinator(
            initial_height_mm=INITIAL_HEIGHT,
            robot_name="jugglebot",
        )
        other_ball = _make_ball(ball_id=1, destination="other_robot")
        our_ball = _make_ball(ball_id=2, destination="jugglebot")

        cmd = coord.update([other_ball, our_ball], current_time=5.0)
        assert cmd is not None
        assert cmd.ball_id == 2

    def test_filters_non_in_flight(self):
        """Only IN_FLIGHT balls are considered."""
        coord = CatchCoordinator(initial_height_mm=INITIAL_HEIGHT)
        caught = _make_ball(ball_id=1, status=BallStatus.CAUGHT)
        flying = _make_ball(ball_id=2, status=BallStatus.IN_FLIGHT)

        cmd = coord.update([caught, flying], current_time=5.0)
        assert cmd is not None
        assert cmd.ball_id == 2

    def test_filters_insufficient_lead_time(self):
        """Balls with insufficient lead time are skipped."""
        coord = CatchCoordinator(
            initial_height_mm=INITIAL_HEIGHT,
            min_lead_time_s=0.3,
        )
        too_late = _make_ball(ball_id=1, landing_time=5.1)  # 0.1s lead
        ok = _make_ball(ball_id=2, landing_time=6.0)  # 1.0s lead

        cmd = coord.update([too_late, ok], current_time=5.0)
        assert cmd is not None
        assert cmd.ball_id == 2

    def test_filters_out_of_xy_range(self):
        """Balls landing outside XY range are skipped."""
        coord = CatchCoordinator(
            initial_height_mm=INITIAL_HEIGHT,
            xy_range_mm=400.0,
        )
        far_ball = _make_ball(
            ball_id=1,
            landing_pos=[500.0, 0.0, LANDING_Z],
        )
        close_ball = _make_ball(
            ball_id=2,
            landing_pos=[100.0, -50.0, LANDING_Z],
        )

        cmd = coord.update([far_ball, close_ball], current_time=5.0)
        assert cmd is not None
        assert cmd.ball_id == 2

    def test_no_catchable_balls(self):
        """Returns None when no balls are catchable."""
        coord = CatchCoordinator(initial_height_mm=INITIAL_HEIGHT)
        cmd = coord.update([], current_time=5.0)
        assert cmd is None


class TestHandCommand:
    """Hand control fields on CatchCommand."""

    def test_arm_hand_set_on_catch(self):
        """CatchCommand populates hand fields for a catchable ball."""
        coord = CatchCoordinator(initial_height_mm=INITIAL_HEIGHT)
        ball = _make_ball(landing_vel=[0.0, 0.0, -3000.0])  # 3 m/s down

        cmd = coord.update([ball], current_time=5.0)
        assert cmd is not None
        assert cmd.arm_hand is True
        assert cmd.event_vel_mps == pytest.approx(3.0, abs=0.01)

    def test_event_vel_clamped_low(self):
        """Very slow ball → event_vel clamped to minimum (0.3 m/s)."""
        coord = CatchCoordinator(initial_height_mm=INITIAL_HEIGHT)
        ball = _make_ball(landing_vel=[0.0, 0.0, -100.0])  # 0.1 m/s

        cmd = coord.update([ball], current_time=5.0)
        assert cmd is not None
        assert cmd.event_vel_mps == pytest.approx(0.3)

    def test_event_vel_clamped_high(self):
        """Very fast ball → event_vel clamped to maximum (7.0 m/s)."""
        coord = CatchCoordinator(initial_height_mm=INITIAL_HEIGHT)
        ball = _make_ball(landing_vel=[0.0, 0.0, -10000.0])  # 10 m/s

        cmd = coord.update([ball], current_time=5.0)
        assert cmd is not None
        assert cmd.event_vel_mps == pytest.approx(7.0)

    def test_event_vel_from_3d_speed(self):
        """event_vel uses full 3D speed, not just Z component."""
        coord = CatchCoordinator(initial_height_mm=INITIAL_HEIGHT)
        # Angled approach within 30° limit: ~14° tilt, speed = 5.099 m/s
        ball = _make_ball(landing_vel=[1250.0, 0.0, -5000.0])

        cmd = coord.update([ball], current_time=5.0)
        assert cmd is not None
        expected_speed = np.linalg.norm([1250.0, 0.0, -5000.0]) / 1000.0
        assert cmd.event_vel_mps == pytest.approx(expected_speed, abs=0.01)
