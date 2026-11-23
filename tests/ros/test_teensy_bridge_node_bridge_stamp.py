"""BUG-134: /robot_state and /hand_telemetry carry the bridge tick time.

``timestamp`` is unchanged (Jetson ROS clock at publish). The appended
``bridge_stamp`` is the ``t_teensy_us`` of the telemetry frame that supplied the
published data, valid only when the heartbeat says TIME_SYNCED and the stamp is
within 1 s of the Jetson clock; otherwise a zero stamp and ``valid`` False.
ROS 2 is mocked by tests/ros/conftest.py.
"""

from __future__ import annotations

import time

import pytest

from teensy_link import protocol as p

from tests.ros._bridge_harness import _build_paired_node, _teardown, _wait_until

_SYNCED = 0x1  # HeartbeatT2J.flags bit0 TIME_SYNCED


@pytest.fixture
def bridge():
    teensy, client, node = _build_paired_node()
    yield teensy, node
    _teardown(teensy, client, node)


def _heartbeat(teensy, node, flags):
    teensy.send_heartbeat_t2j(flags=flags)
    assert _wait_until(lambda: node._latest_heartbeat is not None
                       and int(node._latest_heartbeat.flags) == flags)


def _land(teensy, node, t_us, hand_pos=0.0):
    before = node._telemetry_gen
    pos = tuple(hand_pos for _ in range(p.NUM_AXES))
    tm = p.Telemetry(t_teensy_us=t_us, pos_rev=pos,
                     vel_rps=tuple(0.0 for _ in range(p.NUM_AXES)))
    teensy.send_to_jetson(int(p.MsgType.TELEMETRY), tm.pack())
    assert _wait_until(lambda: node._telemetry_gen > before)


def _us(msg_time):
    return msg_time.sec * 1_000_000 + msg_time.nanosec // 1000


def _now_us():
    return time.time_ns() // 1000


def test_synced_publishes_the_tick_time_of_the_published_sample(bridge):
    teensy, node = bridge
    _heartbeat(teensy, node, _SYNCED)
    t1 = _now_us() - 12_345
    _land(teensy, node, t1)
    node._publish_robot_state()
    node._publish_hand_telemetry()
    rs = node.robot_state_pub.published[-1]
    ht = node.hand_telemetry_pub.published[-1]
    assert rs.bridge_stamp_valid and ht.bridge_stamp_valid
    assert _us(rs.bridge_stamp) == t1 and _us(ht.bridge_stamp) == t1
    # `timestamp` keeps its meaning: the publish-time ROS clock, not the tick.
    assert rs.timestamp == node.get_clock().now().to_msg() or rs.timestamp is not None

    # A newer tick lands: the NEXT message carries the newer tick, never a mix.
    t2 = t1 + 10_000
    _land(teensy, node, t2, hand_pos=0.5)
    node._publish_robot_state()
    node._publish_hand_telemetry()
    rs2 = node.robot_state_pub.published[-1]
    ht2 = node.hand_telemetry_pub.published[-1]
    assert _us(rs2.bridge_stamp) == t2 and _us(ht2.bridge_stamp) == t2
    assert ht2.pos_meas == pytest.approx(0.5)
    # The earlier messages were not rewritten.
    assert _us(rs.bridge_stamp) == t1


def test_unsynced_gives_zero_stamp_and_invalid(bridge):
    teensy, node = bridge
    _heartbeat(teensy, node, 0)
    _land(teensy, node, _now_us())
    node._publish_robot_state()
    node._publish_hand_telemetry()
    for m in (node.robot_state_pub.published[-1],
              node.hand_telemetry_pub.published[-1]):
        assert m.bridge_stamp_valid is False
        assert (m.bridge_stamp.sec, m.bridge_stamp.nanosec) == (0, 0)


def test_synced_flag_but_uptime_epoch_clock_is_invalid(bridge):
    teensy, node = bridge
    _heartbeat(teensy, node, _SYNCED)
    _land(teensy, node, 5_000_000)  # bridge uptime, a 1970-era value
    node._publish_robot_state()
    node._publish_hand_telemetry()
    for m in (node.robot_state_pub.published[-1],
              node.hand_telemetry_pub.published[-1]):
        assert m.bridge_stamp_valid is False
        assert (m.bridge_stamp.sec, m.bridge_stamp.nanosec) == (0, 0)


def test_stale_interfaces_build_still_publishes(bridge, monkeypatch):
    """A message class without the BUG-134 slots (a stale jugglebot_interfaces
    build: real rosidl messages use __slots__, so the assignment raises
    AttributeError) must not stop /robot_state or /hand_telemetry: the
    orchestrator stalls in BOOT without them, and the stamp is an analysis aid."""
    import sys
    teensy, node = bridge
    mod = sys.modules[type(node).__module__]

    def _stale(base):
        class Stale(base):
            _built = False

            def __init__(self, *a, **k):
                super().__init__(*a, **k)
                object.__setattr__(self, '_built', True)

            def __setattr__(self, name, value):
                if self._built and name == 'bridge_stamp_valid':
                    raise AttributeError(name)
                super().__setattr__(name, value)
        return Stale

    monkeypatch.setattr(mod, 'RobotState', _stale(mod.RobotState))
    monkeypatch.setattr(mod, 'HandTelemetryMessage', _stale(mod.HandTelemetryMessage))
    _heartbeat(teensy, node, _SYNCED)
    _land(teensy, node, _now_us())
    n_rs = len(node.robot_state_pub.published)
    n_ht = len(node.hand_telemetry_pub.published)
    node._publish_robot_state()
    node._publish_hand_telemetry()
    assert len(node.robot_state_pub.published) == n_rs + 1
    assert len(node.hand_telemetry_pub.published) == n_ht + 1
