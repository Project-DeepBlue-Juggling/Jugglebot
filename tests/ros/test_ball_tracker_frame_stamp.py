"""`ball_tracker_node` fits against the mocap frame's OWN QTM time, not the
callback clock — the 2026-09-20 mechanism.

Measured on bag `~/Desktop/rosbags/2026-09-18_16-16-17`: `_on_mocap` used to
stamp every frame with `self.get_clock().now()` at CALLBACK time. Under
sitting load that callback runs late and unevenly, so the ballistic fit
(`tracking/flight_fit.py`) saw jittered sample times — at 3 m/s a 10 ms error
is 30 mm, past the fit's 12 mm residual gate, and the fit never converged
(replaying the bag's own receipt times: 54/60 flights fit; the LIVE
callback-timestamped `/balls` stream: only 33/60). Injecting Gaussian jitter
into the replay reproduced it (sigma 3/6/10 ms -> 55/43/23 of 60,
`scratchpad/yield_replay_jitter.py`).

`MocapDataMulti.stamp` now carries the QTM frame time (via
`MocapInterface.latest_frame_ros_ns`), and `_on_mocap` fits against it
whenever it is nonzero — the callback clock is only a fallback for a stamp
of zero (pre-sync, or an old producer). This file exercises that wiring at
the node level; `tests/ros/test_ball_tracker_flight_fit.py` covers the fit's
own convergence unit-to-unit and is the harness this file mirrors.
"""
from __future__ import annotations

from unittest.mock import MagicMock

import numpy as np
import pytest

from jugglebot.ball_tracker_node import BallTrackerNode
from jugglebot.tracking.ball import TrackingConfidence
from jugglebot.tracking.ballistics import GRAVITY_MMPS2
from jugglebot_interfaces.msg import MocapDataMulti, MocapDataSingle

DT = 0.005
CUP = np.array([-50.0, 0.0, 860.0])
TOSS_VEL = np.array([0.0, 0.0, 3084.5])
N_FRAMES = 40


def _ballistic(pos0, vel0, t):
    return np.array([
        pos0[0] + vel0[0] * t,
        pos0[1] + vel0[1] * t,
        pos0[2] + vel0[2] * t - 0.5 * GRAVITY_MMPS2 * t * t,
    ])


def _mocap_msg(pos, *, stamp_s):
    msg = MocapDataMulti()
    s = MocapDataSingle()
    s.position.x, s.position.y, s.position.z = (float(pos[0]), float(pos[1]), float(pos[2]))
    s.residual = 0.3
    s.label = ''
    msg.markers.append(s)
    if stamp_s:
        ns = int(round(stamp_s * 1e9))
        msg.stamp.sec = ns // 1_000_000_000
        msg.stamp.nanosec = ns % 1_000_000_000
    return msg


class _FakeClock:
    """`get_clock()` stand-in whose `.now()` returns `self.t_s` (settable),
    however many times it is called — `_ball_to_msg`'s own header-stamp read
    must not desync a test's intended per-frame time from `_on_mocap`'s
    fallback-clock read."""

    def __init__(self, t_s=0.0):
        self.t_s = t_s

    def now(self):
        clock_time = MagicMock()
        clock_time.nanoseconds = int(round(self.t_s * 1e9))
        return clock_time


def _announce(node):
    return node._tracker.handle_announcement(
        initial_position=CUP.copy(),
        initial_velocity=TOSS_VEL.copy(),
        throw_time=0.0,
        source='jugglebot',
        destination='jugglebot',
    )


def test_exact_stamps_converge_despite_a_jittered_callback_clock():
    """The 2026-09-20 mechanism, reproduced positively: feed each frame's
    EXACT ballistic time as `msg.stamp` while the callback clock is jittered
    by +-15 ms (larger than the 10 ms sigma that dropped the live bag to
    23/60) — the fit must still converge, proving `_on_mocap` fits against
    the stamp, not `get_clock()`."""
    node = BallTrackerNode()
    bid = _announce(node)

    exact_times = [i * DT for i in range(1, N_FRAMES + 1)]
    node._clock = _FakeClock()

    for i, t in enumerate(exact_times):
        node._clock.t_s = t + (0.015 if i % 2 == 0 else -0.015)
        pos = _ballistic(CUP, TOSS_VEL, t)
        node._on_mocap(_mocap_msg(pos, stamp_s=t))

    ball = node._tracker.get_ball(bid)
    assert ball is not None
    assert ball.tracking == TrackingConfidence.CONFIRMED
    assert bid in node._tracker._flight_fits
    assert len(node._tracker._flight_fits[bid]) >= 12
    assert ball.landing_from_fit is True


def test_zero_stamp_falls_back_to_the_callback_clock():
    """A stamp of zero (pre-sync, or an old producer) must not poison the
    fit with epoch-zero samples — `_on_mocap` falls back to `get_clock()`,
    same as before this change, and the fit still converges."""
    node = BallTrackerNode()
    bid = _announce(node)

    exact_times = [i * DT for i in range(1, N_FRAMES + 1)]
    node._clock = _FakeClock()  # the callback clock IS the truth here

    for t in exact_times:
        node._clock.t_s = t
        pos = _ballistic(CUP, TOSS_VEL, t)
        node._on_mocap(_mocap_msg(pos, stamp_s=0.0))

    ball = node._tracker.get_ball(bid)
    assert ball is not None
    assert ball.tracking == TrackingConfidence.CONFIRMED
    assert len(node._tracker._flight_fits[bid]) >= 12
    assert ball.landing_from_fit is True
    # Never saw a nonzero stamp, so the "stamped at the source" transition
    # never latched.
    assert node._mocap_stamp_announced is False


def test_stamped_then_zero_logs_the_fallback_transition_once():
    """The one-shot transition logs (2026-09-20): "stamped at the source" on
    the first nonzero stamp, "fell back" on the first zero stamp AFTER
    having had stamps — each exactly once, not once per frame."""
    node = BallTrackerNode()
    node._logger = MagicMock()
    node.get_logger = lambda: node._logger
    _announce(node)
    node._clock = _FakeClock(DT)

    pos = _ballistic(CUP, TOSS_VEL, DT)
    for _ in range(3):
        node._on_mocap(_mocap_msg(pos, stamp_s=DT))
    assert node._logger.info.call_count == 1
    assert 'stamped at the source' in node._logger.info.call_args[0][0]

    for _ in range(3):
        node._on_mocap(_mocap_msg(pos, stamp_s=0.0))
    assert node._logger.warning.call_count == 1
    assert 'fell back' in node._logger.warning.call_args[0][0]
