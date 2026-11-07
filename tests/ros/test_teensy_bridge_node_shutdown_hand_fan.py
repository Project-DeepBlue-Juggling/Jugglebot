"""On shutdown (Ctrl-C) a forced Hand fan goes back to AUTO, keeping its thresholds.

Owner, 2026-11-08: a fan switched ON by hand must not run past the end of a Sitting
for longer than the motor needs; AUTO stops it below the off threshold.
"""
from __future__ import annotations

import struct
import time

from teensy_link import RpcMethod, RpcStatus

from jugglebot import teensy_bridge_node as tbn
from tests.ros._bridge_harness import _build_paired_node


def _status(mode_id, on_c=30.0, off_c=28.5):
    return struct.pack("<BBhhh", mode_id, 0x03, round(on_c * 100), round(off_c * 100), 3000)


def _capture(teensy):
    calls = []

    def handler(req_id, args):
        calls.append(struct.unpack("<Bff", args))
        return (int(RpcStatus.OK), b"")

    teensy.on_rpc(int(RpcMethod.HAND_FAN_SET), handler)
    return calls


def _shutdown(teensy, client, node):
    node._stow_on_shutdown = False
    node.on_shutdown()
    client.stop()
    teensy.stop()


def test_a_forced_fan_returns_to_auto_with_its_thresholds_kept():
    teensy, client, node = _build_paired_node()
    calls = _capture(teensy)
    node._hand_fan_status_frame = _status(1)
    _shutdown(teensy, client, node)
    assert calls == [(0, 30.0, 28.5)]


def test_forced_off_returns_to_auto_too():
    teensy, client, node = _build_paired_node()
    calls = _capture(teensy)
    node._hand_fan_status_frame = _status(2)
    _shutdown(teensy, client, node)
    assert [c[0] for c in calls] == [0]


def test_the_newest_frame_wins_over_the_published_status():
    """The executor has stopped by shutdown, so the 4 Hz publisher may lag the RX latch."""
    teensy, client, node = _build_paired_node()
    calls = _capture(teensy)
    node._hand_fan_status_frame = _status(0)
    node._hand_fan_status_tick()                 # published: auto
    node._hand_fan_status_frame = _status(1)     # then a forced frame arrived
    _shutdown(teensy, client, node)
    assert [c[0] for c in calls] == [0]


def test_auto_and_no_status_send_nothing():
    for frame in (_status(0), None):
        teensy, client, node = _build_paired_node()
        calls = _capture(teensy)
        node._hand_fan_status_frame = frame
        _shutdown(teensy, client, node)
        assert calls == []


def test_an_unanswered_reset_is_bounded_and_never_raises():
    teensy, client, node = _build_paired_node()
    node._hand_fan_status_frame = _status(1)     # no HAND_FAN_SET handler: no reply
    t0 = time.monotonic()
    node._shutdown_hand_fan_auto()
    assert time.monotonic() - t0 < tbn._SHUTDOWN_HAND_FAN_BUDGET_S + 0.5
    node._hand_fan_status_frame = None
    _shutdown(teensy, client, node)


def test_the_reset_runs_before_the_stow():
    teensy, client, node = _build_paired_node()
    order = []
    node._shutdown_hand_fan_auto = lambda: order.append("fan")
    node._shutdown_stow = lambda: order.append("stow")
    node._wait_wire_disarmed = lambda *a, **k: None
    node._stow_on_shutdown = True
    node.on_shutdown()
    node._stow_on_shutdown = False
    client.stop()
    teensy.stop()
    assert order == ["fan", "stow"]
