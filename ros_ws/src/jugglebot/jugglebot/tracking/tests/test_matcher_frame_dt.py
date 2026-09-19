"""The Kalman predict steps by the MEASURED frame interval (2026-09-20).

`BallTracker.process_frame` used to call `kf.predict()` with no interval, so every
filter integrated the nominal `dt` (5 ms, 200 Hz) while the bags run at ~192 Hz
(5.2 ms): the filter clock ran 4 % slow and the state drifted behind the ball
through every flight. Only the published position/velocity and the pre-fit
fallback depend on it -- the landing comes from the batch fit -- but a fallback
that is wrong by construction is still wrong.
"""
from __future__ import annotations

import numpy as np
import pytest

from jugglebot.tracking.ballistics import GRAVITY_MMPS2
from jugglebot.tracking.matcher import BallTracker

CUP = np.array([-50.0, 0.0, 860.0])
VEL = np.array([0.0, 0.0, 3500.0])


def _ballistic(t):
    p = CUP + VEL * t
    p[2] -= 0.5 * GRAVITY_MMPS2 * t * t
    return p


def _fly(dt_frames: float, nominal_dt: float, n: int = 60, fixed_dt=None):
    tracker = BallTracker(dt=nominal_dt, landing_z=830.0, announced_gate_mm=200.0)
    if fixed_dt is not None:                       # the pre-2026-09-20 behaviour
        tracker._frame_dt = lambda _t: fixed_dt
    bid = tracker.handle_announcement(initial_position=CUP.copy(),
                                      initial_velocity=VEL.copy(), throw_time=0.0,
                                      source='jugglebot', destination='jugglebot')
    for i in range(1, n + 1):
        t = i * dt_frames
        tracker.process_frame([_ballistic(t)], t)
    ball = tracker.get_ball(bid)
    return tracker, ball, _ballistic(n * dt_frames)


def test_the_filter_steps_by_the_measured_interval_not_the_nominal_one():
    """Frames 8 ms apart against a 5 ms nominal: the filter that steps by the
    measured gap tracks the true velocity (measured 2026-09-20: +20 mm/s
    against +1698 mm/s stepping by the nominal one at these frames; at the
    real 5.2 ms frames the nominal step is +119 mm/s, a 4 % lag)."""
    t_end = 60 * 0.008
    vz_true = VEL[2] - GRAVITY_MMPS2 * t_end
    tracker, ball, _truth = _fly(dt_frames=0.008, nominal_dt=0.005)
    assert tracker._frame_dt(t_end + 0.008) == pytest.approx(0.008)
    # The measurement update pins the POSITION either way; the interval shows
    # in the VELOCITY, which the pre-fit fallback landing is extrapolated from.
    assert abs(float(ball.velocity[2]) - vz_true) < 50.0
    _tr, ball_nominal, _ = _fly(dt_frames=0.008, nominal_dt=0.005, fixed_dt=0.005)
    assert abs(float(ball_nominal.velocity[2]) - vz_true) > 500.0


def test_the_measured_interval_is_clamped_and_the_first_frame_is_nominal():
    tracker = BallTracker(dt=0.005, landing_z=830.0)
    assert tracker._frame_dt(10.0) == 0.005                  # no previous frame
    tracker._last_frame_time = 10.0
    assert tracker._frame_dt(10.0) == 0.005                  # non-advancing clock
    assert tracker._frame_dt(10.0 + 0.0005) == 0.4 * 0.005   # a burst: floored
    assert tracker._frame_dt(10.0 + 0.5) == 4.0 * 0.005      # a dropout: capped
