"""Integration: `BallTracker` wires the gravity-fixed `FlightFit` landing
estimate in alongside (not instead of) the Kalman filter.

See `ros_ws/src/jugglebot/jugglebot/tracking/flight_fit.py` for the
unit-level fit tests and its module docstring for the defect this replaces:
the KF-extrapolated `landing_time` runs 0.06-0.20 s late and grows later
through the descent. This file exercises the wiring in `matcher.py` — that
`_associate_confirmed_balls` feeds the RAW marker (not the KF state) into
each CONFIRMED ball's fit, that `_update_landing_prediction` prefers a
converged fit over the KF extrapolation, that it falls back to the KF path
before the fit has enough samples, and that a fresh throw gets a fresh fit.
"""
from __future__ import annotations

import numpy as np
import pytest

from jugglebot.tracking.ball import BallStatus, TrackingConfidence
from jugglebot.tracking.ballistics import GRAVITY_MMPS2, predict_landing_state
from jugglebot.tracking.matcher import BallTracker

DT = 1.0 / 192.0  # matches the measured mocap rate on the validation bags
LANDING_Z = 830.0
CUP = np.array([-50.0, 0.0, 860.0])
TOSS_VEL = np.array([0.0, 0.0, 3500.0])


def _ballistic(pos0, vel0, t):
    p = np.array(pos0, dtype=np.float64) + np.array(vel0, dtype=np.float64) * t
    p[2] -= 0.5 * GRAVITY_MMPS2 * t * t
    return p


def _make_tracker(**over):
    kw = dict(
        dt=DT,
        landing_z=LANDING_Z,
        announced_gate_mm=200.0,
        flight_fit_min_samples=12,
        flight_fit_residual_mm=12.0,
        flight_fit_freeze_above_plane_mm=250.0,
    )
    kw.update(over)
    return BallTracker(**kw)


def _announce_and_fly(tracker, n_frames, throw_time=0.0, pos0=CUP, vel0=TOSS_VEL):
    """Announce one ball and feed it `n_frames` noiseless ballistic frames.

    Returns (ball_id, true_crossing_abs).
    """
    bid = tracker.handle_announcement(
        initial_position=pos0.copy(),
        initial_velocity=vel0.copy(),
        throw_time=throw_time,
        source='jugglebot',
        destination='jugglebot',
    )
    _, _, ttl = predict_landing_state(pos0, vel0, LANDING_Z)
    true_crossing_abs = throw_time + ttl
    for i in range(1, n_frames + 1):
        t = throw_time + i * DT
        tracker.process_frame([_ballistic(pos0, vel0, t - throw_time)], t)
    return bid, true_crossing_abs


class TestFitPreferredOverKF:
    def test_landing_time_converges_to_the_true_crossing(self):
        """With enough free-flight samples the published landing_time should
        be accurate to a few ms — the KF-only path (see
        `late_catch_bag2.txt` / `flight_truth2.txt`, 2026-09-17) is
        routinely 60-200 ms late on real hardware, so a wide 5 ms bound here
        already discriminates fit-active from fit-inactive."""
        tracker = _make_tracker()
        bid, true_crossing_abs = _announce_and_fly(tracker, n_frames=40)
        ball = tracker.get_ball(bid)
        assert ball.tracking == TrackingConfidence.CONFIRMED
        assert ball.landing_time > 0
        assert abs(ball.landing_time - true_crossing_abs) < 0.005

    def test_flight_fit_dict_holds_a_fit_for_the_confirmed_ball(self):
        tracker = _make_tracker()
        # 40 frames: the first ~14 sit below the cup floor (860 mm release,
        # floor 1080 mm on both legs since 2026-09-17) and are not admitted.
        bid, _ = _announce_and_fly(tracker, n_frames=40)
        assert bid in tracker._flight_fits
        assert len(tracker._flight_fits[bid]) >= 12


class TestFallbackBeforeConvergence:
    def test_early_frames_still_get_a_landing_estimate_from_the_kf_path(self):
        """Before the fit has `min_samples`, the ball must still carry SOME
        landing estimate (the pre-2026-09-17 KF/announcement extrapolation),
        not None/zero — the fit only changes which path wins once it has
        converged."""
        tracker = _make_tracker(flight_fit_min_samples=12)
        bid, _ = _announce_and_fly(tracker, n_frames=3)
        ball = tracker.get_ball(bid)
        assert ball.tracking == TrackingConfidence.CONFIRMED
        assert ball.landing_time > 0
        # Fewer than min_samples frames means the fit must not have
        # converged yet.
        fit = tracker._flight_fits.get(bid)
        assert fit is not None
        assert fit.fit() is None


class TestFreshThrowGetsFreshFit:
    def test_second_throw_is_not_contaminated_by_the_first(self):
        tracker = _make_tracker()
        bid1, true1 = _announce_and_fly(tracker, n_frames=40, throw_time=0.0)
        ball1 = tracker.get_ball(bid1)
        assert abs(ball1.landing_time - true1) < 0.005

        # A different flight: different origin/velocity, later throw_time.
        pos0_2 = np.array([50.0, -20.0, 900.0])
        vel0_2 = np.array([-100.0, 40.0, 3200.0])
        bid2, true2 = _announce_and_fly(
            tracker, n_frames=40, throw_time=5.0, pos0=pos0_2, vel0=vel0_2)
        assert bid2 != bid1
        ball2 = tracker.get_ball(bid2)
        assert abs(ball2.landing_time - true2) < 0.005
        # Independent fits: ball 1's estimate is unaffected by ball 2 existing.
        assert abs(tracker.get_ball(bid1).landing_time - true1) < 0.005


class TestCleanup:
    def test_terminal_ball_past_retention_drops_its_flight_fit(self):
        tracker = _make_tracker(caught_grace_s=0.0, missed_frames_to_lose=1,
                                 terminal_retention_s=0.05)
        bid, _ = _announce_and_fly(tracker, n_frames=40)
        assert bid in tracker._flight_fits

        # Push well past landing + grace + retention with no more markers so
        # the ball goes CAUGHT then gets cleaned up. True crossing for
        # CUP/TOSS_VEL at LANDING_Z is ~0.72 s after throw (solve the
        # ballistic quadratic); go to 1.1 s to clear it with margin.
        t_last = 40 * DT
        for i in range(1, 100):
            t = t_last + i * 0.01
            tracker.process_frame([], t)

        assert tracker.get_ball(bid) is None
        assert bid not in tracker._flight_fits
