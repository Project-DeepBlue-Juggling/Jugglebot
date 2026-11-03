"""Tests for matcher.py — parabolic detection, announced matching, frame-to-frame."""

import numpy as np
import pytest

from jugglebot.tracking.ballistics import GRAVITY_MMPS2
from jugglebot.tracking.ball import BallStatus, TrackingConfidence
from jugglebot.tracking.matcher import BallTracker


DT = 0.005  # 200 Hz
LANDING_Z = 734.3


def _ballistic_pos(pos0, vel0, t):
    """Analytical position under gravity at time t."""
    return np.array([
        pos0[0] + vel0[0] * t,
        pos0[1] + vel0[1] * t,
        pos0[2] + vel0[2] * t - 0.5 * GRAVITY_MMPS2 * t * t,
    ])


class TestParabolicDetection:
    """Detect parabolic motion from raw marker streams.

    The human-throw path is OFF in production since 2026-09-15 (it spawned 158
    phantom tracks off one static floor marker at the R3 sitting, and nothing
    consumes them yet), so these tests opt in explicitly with
    `detect_human_throws=True` — they exist to keep the path working for when
    it is turned back on.
    """

    def test_detects_single_parabolic_marker(self):
        """One marker following ballistic trajectory among static markers."""
        tracker = BallTracker(
            dt=DT, landing_z=LANDING_Z,
            parabolic_min_frames=3,
            parabolic_accel_threshold_mmps2=3000.0,
            detect_human_throws=True,
        )

        pos0 = np.array([0.0, 0.0, 900.0])
        vel0 = np.array([1500.0, 0.0, 1000.0])
        static_markers = [
            np.array([500.0, 500.0, 100.0]),
            np.array([-300.0, 200.0, 50.0]),
        ]

        # Feed 5 frames of the ballistic marker + static markers
        for i in range(5):
            t = i * DT
            ball_pos = _ballistic_pos(pos0, vel0, t)
            markers = static_markers + [ball_pos]
            tracker.process_frame(markers, t)

        # Should have detected one ball
        in_flight = [b for b in tracker.active_balls if b.status == BallStatus.IN_FLIGHT]
        assert len(in_flight) == 1
        assert in_flight[0].source == "human_throw"
        assert in_flight[0].tracking == TrackingConfidence.CONFIRMED

    def test_ignores_static_markers(self):
        """Static markers should not trigger ball detection."""
        tracker = BallTracker(
            dt=DT, landing_z=LANDING_Z,
            parabolic_min_frames=3,
            detect_human_throws=True,
        )

        static_markers = [
            np.array([100.0, 200.0, 300.0]),
            np.array([-100.0, 0.0, 150.0]),
        ]

        for i in range(10):
            tracker.process_frame(static_markers, i * DT)

        in_flight = [b for b in tracker.active_balls if b.status == BallStatus.IN_FLIGHT]
        assert len(in_flight) == 0

    def test_ignores_linear_motion(self):
        """Marker moving at constant velocity (no gravity) should not match."""
        tracker = BallTracker(
            dt=DT, landing_z=LANDING_Z,
            parabolic_min_frames=3,
            parabolic_accel_threshold_mmps2=3000.0,
            detect_human_throws=True,
        )

        pos0 = np.array([0.0, 0.0, 900.0])
        vel = np.array([1000.0, 500.0, -200.0])

        for i in range(6):
            t = i * DT
            # Linear motion: no gravity term
            marker_pos = pos0 + vel * t
            tracker.process_frame([marker_pos], t)

        in_flight = [b for b in tracker.active_balls if b.status == BallStatus.IN_FLIGHT]
        assert len(in_flight) == 0


class TestAnnouncedMatching:
    """Match mocap markers to announced balls."""

    def test_announced_ball_matched(self):
        """Announce a throw, then feed markers near the predicted trajectory → matched."""
        tracker = BallTracker(dt=DT, landing_z=LANDING_Z)

        pos0 = np.array([0.0, 0.0, 800.0])
        vel0 = np.array([1000.0, 0.0, 2000.0])
        throw_time = 0.1

        tracker.handle_announcement(
            initial_position=pos0,
            initial_velocity=vel0,
            throw_time=throw_time,
            source="ball_butler",
            destination="jugglebot",
        )

        # Advance to just after throw_time and feed matching markers
        t = throw_time + DT
        for i in range(10):
            ball_pos = _ballistic_pos(pos0, vel0, t - throw_time)
            # Add small noise to simulate real mocap
            noisy_pos = ball_pos + np.random.default_rng(i).normal(0, 2.0, size=3)
            tracker.process_frame([noisy_pos], t)
            t += DT

        # Ball should be CONFIRMED
        balls = [b for b in tracker.active_balls if b.id == 1]
        assert len(balls) == 1
        assert balls[0].tracking == TrackingConfidence.CONFIRMED
        assert balls[0].status == BallStatus.IN_FLIGHT

    def test_announced_ball_not_matched_far_marker(self):
        """Markers far from predicted trajectory should not match the announced ball."""
        tracker = BallTracker(dt=DT, landing_z=LANDING_Z)

        pos0 = np.array([0.0, 0.0, 800.0])
        vel0 = np.array([1000.0, 0.0, 2000.0])
        throw_time = 0.0

        tracker.handle_announcement(
            initial_position=pos0,
            initial_velocity=vel0,
            throw_time=throw_time,
        )

        # Feed markers 2000mm away from predicted path
        for i in range(10):
            t = (i + 1) * DT
            far_marker = np.array([5000.0, 5000.0, 900.0])
            tracker.process_frame([far_marker], t)

        balls = [b for b in tracker.active_balls if b.id == 1]
        assert len(balls) == 1
        # Should still be ANNOUNCED (not matched)
        assert balls[0].tracking == TrackingConfidence.ANNOUNCED

    def test_time_based_in_flight_transition(self):
        """TO_BE_THROWN → IN_FLIGHT when now >= throw_time, no mocap needed."""
        tracker = BallTracker(dt=DT, landing_z=LANDING_Z)

        throw_time = 0.5
        tracker.handle_announcement(
            initial_position=np.array([0.0, 0.0, 800.0]),
            initial_velocity=np.array([0.0, 0.0, 2000.0]),
            throw_time=throw_time,
        )

        # Before throw_time: should be TO_BE_THROWN
        tracker.process_frame([], 0.3)
        ball = tracker.get_ball(1)
        assert ball.status == BallStatus.TO_BE_THROWN

        # After throw_time: should be IN_FLIGHT
        tracker.process_frame([], 0.6)
        ball = tracker.get_ball(1)
        assert ball.status == BallStatus.IN_FLIGHT


class TestFrameToFrameAssociation:
    """Track a confirmed ball across multiple frames."""

    def test_maintains_identity_across_frames(self):
        """A confirmed ball should keep tracking across consecutive frames."""
        tracker = BallTracker(dt=DT, landing_z=LANDING_Z)

        pos0 = np.array([0.0, 0.0, 800.0])
        vel0 = np.array([1000.0, 0.0, 2000.0])
        throw_time = 0.0

        tracker.handle_announcement(
            initial_position=pos0,
            initial_velocity=vel0,
            throw_time=throw_time,
            destination="jugglebot",
        )

        # Feed 30 frames of matching markers
        positions_logged = []
        for i in range(30):
            t = (i + 1) * DT
            true_pos = _ballistic_pos(pos0, vel0, t)
            tracker.process_frame([true_pos], t)
            ball = tracker.get_ball(1)
            positions_logged.append(ball.position.copy())

        # Ball should still exist and be tracking
        ball = tracker.get_ball(1)
        assert ball is not None
        assert ball.frames_tracked > 0
        assert ball.tracking == TrackingConfidence.CONFIRMED

    def test_survives_missed_frames(self):
        """Ball should survive a few missed frames and re-acquire."""
        tracker = BallTracker(
            dt=DT, landing_z=LANDING_Z,
            missed_frames_to_lose=10,
        )

        pos0 = np.array([0.0, 0.0, 800.0])
        vel0 = np.array([1000.0, 0.0, 2000.0])
        throw_time = 0.0

        tracker.handle_announcement(
            initial_position=pos0,
            initial_velocity=vel0,
            throw_time=throw_time,
        )

        # 10 frames with marker (to get confirmed)
        for i in range(10):
            t = (i + 1) * DT
            pos = _ballistic_pos(pos0, vel0, t)
            tracker.process_frame([pos], t)

        # 5 frames with no markers (gap)
        for i in range(5):
            t = (11 + i) * DT
            tracker.process_frame([], t)

        ball = tracker.get_ball(1)
        assert ball is not None
        assert ball.status == BallStatus.IN_FLIGHT
        assert ball.frames_without_measurement == 5

        # Marker reappears
        t = 16 * DT
        pos = _ballistic_pos(pos0, vel0, t)
        tracker.process_frame([pos], t)

        ball = tracker.get_ball(1)
        assert ball.frames_without_measurement == 0


class TestHumanThrowCarriedAcrossFlights:
    """A human-thrown ball is caught in a hand and thrown again without the
    tracker ever losing its marker, so ONE ball id serves both flights. Its
    flight fit must forget the first flight at the re-throw, or the landing
    published for the second flight is the first flight's (JB-113, the
    2026-11-02 human-throw bags: 5 of 34 such throws usable at the 0.3 s
    deadline before the fix). Plane 830, freeze floor 1080, hand at 450."""

    PLANE = 830.0
    # The production mocap rate (every QTM frame since 2026-10-10). At 200 Hz
    # this 3 g scoop pulls the free-fall Kalman prediction past the 50 mm
    # gate and the track is dropped at the re-throw (the precondition fails).
    DT = 1.0 / 300.0

    def _run(self, rise_mm):
        DT = self.DT
        tracker = BallTracker(dt=DT, landing_z=self.PLANE, detect_human_throws=True,
                              flight_fit_new_flight_rise_mm=rise_mm)
        f1_p, f1_v = np.array([0.0, 0.0, 450.0]), np.array([0.0, 0.0, 4500.0])
        f2_p, f2_v = np.array([150.0, 0.0, 450.0]), np.array([-400.0, 100.0, 4000.0])
        t1_end = 2.0 * f1_v[2] / GRAVITY_MMPS2           # back at the hand
        hand_s = 0.30
        # The scoop: a cubic Hermite from the catch (flight 1's landing state)
        # to the release (flight 2's launch state), so velocity is continuous
        # as a real hand's is; it dips ~300 mm and moves 150 mm in x.
        c_p, c_v = _ballistic_pos(f1_p, f1_v, t1_end), f1_v - [0.0, 0.0, GRAVITY_MMPS2 * t1_end]
        t2 = t1_end + hand_s
        t2_apex = t2 + f2_v[2] / GRAVITY_MMPS2
        ids, t = set(), 0.0
        while True:
            if t < t1_end:
                p = _ballistic_pos(f1_p, f1_v, t)
            elif t < t2:
                s = (t - t1_end) / hand_s
                p = ((2 * s**3 - 3 * s**2 + 1) * c_p + (s**3 - 2 * s**2 + s) * hand_s * c_v
                     + (-2 * s**3 + 3 * s**2) * f2_p + (s**3 - s**2) * hand_s * f2_v)
            else:
                p = _ballistic_pos(f2_p, f2_v, t - t2)
            balls = [b for b in tracker.process_frame([p], t) if b.status == BallStatus.IN_FLIGHT]
            ids |= {b.id for b in balls}
            if t > t2_apex and p[2] < 1150.0:             # second flight descending
                break
            t += DT
        vz, dz = f2_v[2], self.PLANE - f2_p[2]
        t_true = t2 + (vz + np.sqrt(vz * vz - 2.0 * GRAVITY_MMPS2 * dz)) / GRAVITY_MMPS2
        assert len(ids) == 1 and len(balls) == 1, 'precondition: one ball carried through the hand'
        return balls[0], t_true

    def test_second_flight_landing_comes_from_its_own_fit(self):
        ball, t_true = self._run(50.0)
        assert ball.landing_from_fit
        assert abs(ball.landing_time - t_true) < 0.005

    def test_without_segmentation_the_first_flight_poisons_the_fit(self):
        """The control: the defect the segmentation fixes, pinned so a change
        that makes it moot (e.g. a new id per human throw) is noticed."""
        ball, t_true = self._run(None)
        assert abs(ball.landing_time - t_true) > 0.030
