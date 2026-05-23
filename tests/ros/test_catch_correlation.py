"""Tests for jugglebot.catch_correlation_node.CatchCorrelationNode.

Correlation logic — matching catch events to throw announcements and
computing the timing error. ROS2 is mocked via tests/ros/conftest.py.
"""

from types import SimpleNamespace

import pytest

from jugglebot_interfaces.msg import CatchEvent, ThrowAnnouncement


def _time(sec, nanosec=0):
    """A builtin_interfaces/Time-like object."""
    return SimpleNamespace(sec=sec, nanosec=nanosec)


@pytest.fixture
def node():
    from jugglebot.catch_correlation_node import CatchCorrelationNode
    return CatchCorrelationNode()


def _throw(landing_sec, landing_nanosec=0, thrower="ball_butler"):
    ann = ThrowAnnouncement()
    ann.thrower_name = thrower
    ann.landing_time = _time(landing_sec, landing_nanosec)
    return ann


def _catch(sec, nanosec=0, sequence=1, time_synced=True):
    evt = CatchEvent()
    evt.catch_time = _time(sec, nanosec)
    evt.sequence = sequence
    evt.time_synced = time_synced
    return evt


class TestCorrelation:
    def test_matches_closest_throw(self, node):
        node._on_throw(_throw(100))                       # predicted landing at 100.000 s
        node._on_catch(_catch(100, 50_000_000))           # actual catch 50 ms later
        assert len(node.result_pub.published) == 1
        res = node.result_pub.published[0]
        assert res.matched is True
        assert res.thrower_name == "ball_butler"
        assert res.timing_error_ms == pytest.approx(50.0, abs=1e-6)

    def test_negative_error_for_early_catch(self, node):
        node._on_throw(_throw(100, 100_000_000))          # predicted landing at 100.100 s
        node._on_catch(_catch(100, 60_000_000))           # caught 40 ms early
        res = node.result_pub.published[0]
        assert res.matched is True
        assert res.timing_error_ms == pytest.approx(-40.0, abs=1e-6)

    def test_unmatched_when_no_throw(self, node):
        node._on_catch(_catch(100))
        res = node.result_pub.published[0]
        assert res.matched is False

    def test_unmatched_when_outside_window(self, node):
        node._on_throw(_throw(100))
        node._on_catch(_catch(101))                       # 1 s away — beyond the 0.5 s window
        res = node.result_pub.published[0]
        assert res.matched is False

    def test_picks_closest_of_multiple(self, node):
        node._on_throw(_throw(100, thrower="ball_butler"))
        node._on_throw(_throw(105, thrower="jugglebot"))
        node._on_catch(_catch(100, 20_000_000))           # closest to the 100 s throw
        res = node.result_pub.published[0]
        assert res.matched is True
        assert res.thrower_name == "ball_butler"
        assert res.timing_error_ms == pytest.approx(20.0, abs=1e-6)

    def test_unsynced_catch_skipped(self, node):
        node._on_throw(_throw(100))
        node._on_catch(_catch(100, 10_000_000, time_synced=False))
        res = node.result_pub.published[0]
        # Published (so the event isn't lost) but never correlated.
        assert res.matched is False

    def test_result_carries_catch_time(self, node):
        catch = _catch(100, 50_000_000)
        node._on_throw(_throw(100))
        node._on_catch(catch)
        res = node.result_pub.published[0]
        assert res.actual_catch_time is catch.catch_time
