"""The release identity crosses the ``/balls`` wire (2026-10-04).

The two-ball association contract (``ball_possession``, "The release
identity") keys every scheduled release on the track's ``(source,
throw_time)``. That key is only exact if the tracker publishes the
announcement's own ``throw_time`` verbatim and the same ``source`` mapping
the consumer's latch uses. Before this change ``BallState`` carried no
``throw_time`` at all, so no consumer could tell which announcement a track
was minted for, and on 2026-10-02 every fed-columns attempt latched the OTHER
ball's flight.

These tests drive the real ``ball_tracker_node`` announcement handler and
``_ball_to_msg`` (under the mocked ROS layer) and then the real
``match_announced_track`` on the published message, so a dropped or
mis-converted field on either side goes red.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pytest

from jugglebot import ball_possession as bp
from jugglebot.ball_tracker_node import BallTrackerNode
from jugglebot_interfaces.msg import ThrowAnnouncement
from tests.ros.conftest import MsgTime, Point, Vector3

# A real ROS-epoch instant (sitting 2026-10-02, ball 0's launch release), so
# the float/nanosecond conversion is exercised at the magnitude it runs at.
T_RELEASE = 1790929266.912


def _stamp(t_s):
    ns = int(round(float(t_s) * 1e9))
    return MsgTime(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def _announcement(thrower, t_release_s):
    return ThrowAnnouncement(
        thrower_name=thrower, target_id='jugglebot',
        initial_position=Point(x=50.0, y=0.0, z=860.0),
        initial_velocity=Vector3(x=0.0, y=0.0, z=4200.0),
        throw_time=_stamp(t_release_s),
        predicted_tof_sec=0.857,
        landing_position=Point(x=50.0, y=0.0, z=830.0),
        landing_velocity=Vector3(x=0.0, y=0.0, z=-4200.0),
        landing_time=_stamp(t_release_s + 0.857))


def _published(node, tracker_id):
    return node._ball_to_msg(node._tracker.get_ball(tracker_id))


@pytest.mark.parametrize('thrower', ['jugglebot', 'ball_butler'])
def test_the_announced_throw_time_and_source_round_trip_onto_balls(thrower):
    node = BallTrackerNode()
    node._on_announcement(_announcement(thrower, T_RELEASE))
    (tracker_id,) = [b.id for b in node._tracker.active_balls]
    msg = _published(node, tracker_id)
    assert msg.source == thrower
    assert bp.ball_throw_time_s(msg) == pytest.approx(T_RELEASE, abs=1e-6)
    # ... and the consumer's identity match finds exactly this track.
    assert bp.match_announced_track(
        [msg], thrower=bp.announced_source(thrower),
        t_release_s=T_RELEASE) == tracker_id


def test_an_empty_thrower_maps_to_the_same_source_on_both_sides():
    """The tracker stamps an empty ``thrower_name`` as ``ball_butler``; the
    consumer's latch must use the identical mapping or the key never meets."""
    node = BallTrackerNode()
    node._on_announcement(_announcement('', T_RELEASE))
    (tracker_id,) = [b.id for b in node._tracker.active_balls]
    msg = _published(node, tracker_id)
    assert msg.source == bp.announced_source('') == 'ball_butler'


def test_two_announcements_are_told_apart_by_their_own_throw_time():
    """The columns case at the wire: two of OUR announcements one beat apart
    mint two tracks with the same source; each latch finds only its own."""
    beat = 0.579
    node = BallTrackerNode()
    node._on_announcement(_announcement('jugglebot', T_RELEASE))
    node._on_announcement(_announcement('jugglebot', T_RELEASE + beat))
    ids = sorted(b.id for b in node._tracker.active_balls)
    msgs = [_published(node, i) for i in ids]
    assert bp.match_announced_track(
        msgs, thrower='jugglebot', t_release_s=T_RELEASE) == ids[0]
    assert bp.match_announced_track(
        msgs, thrower='jugglebot', t_release_s=T_RELEASE + beat) == ids[1]
    # A source mismatch never matches, whatever the time.
    assert bp.match_announced_track(
        msgs, thrower='ball_butler', t_release_s=T_RELEASE) is None


def test_an_unannounced_track_publishes_no_identity():
    node = BallTrackerNode()
    node._on_announcement(_announcement('jugglebot', T_RELEASE))
    (tracker_id,) = [b.id for b in node._tracker.active_balls]
    ball = node._tracker.get_ball(tracker_id)
    ball.throw_time = 0.0               # what a parabolic detection carries
    msg = node._ball_to_msg(ball)
    assert bp.ball_throw_time_s(msg) is None
    assert bp.match_announced_track(
        [msg], thrower='jugglebot', t_release_s=T_RELEASE) is None


@dataclass
class _StaleBallState:
    """``BallState`` as generated BEFORE 2026-10-04: no ``throw_time``."""
    id: int = 0
    source: str = ''


def test_a_stale_interface_build_refuses_to_start():
    """A node started against the old generated message must fail at start,
    loudly and with the fix named -- never run on and silently never
    correlate (or, on the tracker, die on its first publish)."""
    with pytest.raises(RuntimeError, match='colcon build --packages-select '
                                           'jugglebot_interfaces jugglebot'):
        bp.require_identity_fields(_StaleBallState)
    from jugglebot_interfaces.msg import BallState
    bp.require_identity_fields(BallState)       # the current one passes
    # And reading the key off a stale message is an error, not a None.
    with pytest.raises(AttributeError):
        bp.ball_throw_time_s(_StaleBallState())
