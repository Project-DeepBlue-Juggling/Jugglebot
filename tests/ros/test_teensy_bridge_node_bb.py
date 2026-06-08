"""Ball Butler bridge tests (Phase A cutover — bb/* moved from can_node).

Covers the BB surface that teensy_bridge_node inherits from can_node:

* ``bb/heartbeat`` publisher — sources state from the extended HeartbeatT2J
  (bb_state / bb_state_data / bb_flags / bb_yaw_deg / bb_pitch_deg /
  bb_hand_mm). Mirrors the legacy ``can_node._publish_bb_heartbeat`` field
  semantics so existing GUI / mocap / orchestrator consumers see no shift.
* ``bb/calibrate`` / ``bb/reload`` / ``bb/reset`` / ``bb/send_throw_command``
  services — invoke the firmware RPCs (BB_CALIBRATE_LOC / BB_RELOAD /
  BB_RESET / BB_THROW). The firmware gates each TX on bb_present(); we
  translate the resulting ERR_BUS_DOWN into a silent-success for bb/calibrate
  ONLY (to preserve can_node._svc_bb_calibrate's HOMING semantics). The
  other three propagate the error.

The harness reuses the FakeTeensy loopback pattern from
tests/teensy_link/conftest.py via tests/ros/test_teensy_bridge_node_read._
build_paired_node — same pattern as test_teensy_bridge_node_rpc.py.
"""

from __future__ import annotations

import pytest

from controller.teensy_link import (
    HeartbeatT2J, LinkState, BusHealth, FaultState, MsgType,
    RpcMethod, RpcStatus,
)
from controller.teensy_link import rpc_args

from tests.ros.test_teensy_bridge_node_read import _build_paired_node, _wait_until

# Mocked ROS service/message types (tests/ros/conftest.py).
from std_srvs.srv import Trigger
from jugglebot_interfaces.srv import SendBallButlerCommand


def _node():
    teensy, client, node = _build_paired_node()
    return teensy, client, node


def _teardown(teensy, client, node):
    node.on_shutdown()
    client.stop()
    teensy.stop()


def _send_heartbeat(teensy, *, bb_state=1, bb_state_data=0,
                    ball_in_hand=False, heartbeat_seen=True,
                    heartbeat_stale=False, bb_yaw=0.0, bb_pitch=0.0,
                    bb_hand=0.0):
    """Inject a HeartbeatT2J frame from the FakeTeensy with the requested BB
    snapshot fields. Defaults to "BB present, IDLE, ball not in hand."
    """
    flags = ((0x1 if ball_in_hand else 0)
             | (0x2 if heartbeat_seen else 0)
             | (0x4 if heartbeat_stale else 0))
    hb = HeartbeatT2J(
        t_teensy_us=1, link_state=int(LinkState.UP),
        bus1_health=int(BusHealth.OK), bus2_health=int(BusHealth.OK),
        fault_state=int(FaultState.NONE), flags=0, uptime_ms=1,
        bb_state=bb_state, bb_state_data=bb_state_data, bb_flags=flags,
        bb_yaw_deg=bb_yaw, bb_pitch_deg=bb_pitch, bb_hand_mm=bb_hand,
    )
    teensy.send_to_jetson(int(MsgType.HEARTBEAT_T2J), hb.pack())


def _capture(teensy, method, status=int(RpcStatus.OK), result=b""):
    """Register an auto-responder that records the args it received."""
    box = {}

    def handler(req_id, args):
        box['args'] = args
        box['req_id'] = req_id
        return (status, result)

    teensy.on_rpc(int(method), handler)
    return box


# ── bb/heartbeat publisher ───────────────────────────────────────────────────

def test_bb_heartbeat_suppressed_before_first_t2j():
    """No bb/heartbeat published until the bridge sees a HeartbeatT2J frame
    (mirrors _publish_robot_state's 'no phantom snapshot' rule)."""
    teensy, client, node = _node()
    try:
        node._publish_bb_heartbeat()
        assert node.bb_heartbeat_pub.published == []
    finally:
        _teardown(teensy, client, node)


def test_bb_heartbeat_reflects_t2j_fields():
    teensy, client, node = _node()
    try:
        _send_heartbeat(teensy, bb_state=1, ball_in_hand=True,
                        bb_yaw=45.0, bb_pitch=22.5, bb_hand=123.4)
        assert _wait_until(lambda: node._latest_heartbeat is not None)
        node._publish_bb_heartbeat()
        assert len(node.bb_heartbeat_pub.published) == 1
        msg = node.bb_heartbeat_pub.published[0]
        assert msg.connected is True             # heartbeat_seen && !stale
        assert msg.ball_in_hand is True
        assert msg.state == 1                    # IDLE
        assert msg.state_data == 0
        assert msg.yaw_deg == pytest.approx(45.0)
        assert msg.pitch_deg == pytest.approx(22.5)
        assert msg.hand_pos_mm == pytest.approx(123.4)
    finally:
        _teardown(teensy, client, node)


def test_bb_heartbeat_connected_false_when_stale():
    """heartbeat_stale=True ⇒ connected reports False even though the field
    snapshot is still populated. Matches the legacy can_node behaviour where
    the timeout window gates the 'connected' bit."""
    teensy, client, node = _node()
    try:
        _send_heartbeat(teensy, heartbeat_seen=True, heartbeat_stale=True)
        assert _wait_until(lambda: node._latest_heartbeat is not None)
        node._publish_bb_heartbeat()
        msg = node.bb_heartbeat_pub.published[-1]
        assert msg.connected is False
    finally:
        _teardown(teensy, client, node)


def test_bb_heartbeat_connected_false_when_unseen():
    """heartbeat_seen=False ⇒ connected is False (BB never observed)."""
    teensy, client, node = _node()
    try:
        _send_heartbeat(teensy, heartbeat_seen=False, heartbeat_stale=False)
        assert _wait_until(lambda: node._latest_heartbeat is not None)
        node._publish_bb_heartbeat()
        msg = node.bb_heartbeat_pub.published[-1]
        assert msg.connected is False
    finally:
        _teardown(teensy, client, node)


# ── bb/* services (RPC dispatch + arg encoding) ──────────────────────────────

def test_bb_reset_invokes_rpc_with_empty_args():
    """bb/reset → BB_RESET RPC with payloadless args (matches NOP shape)."""
    teensy, client, node = _node()
    try:
        _send_heartbeat(teensy)  # BB present so the firmware gate is open
        assert _wait_until(lambda: node._latest_heartbeat is not None)
        box = _capture(teensy, RpcMethod.BB_RESET)
        res = node._svc_bb_reset(Trigger.Request(), Trigger.Response())
        assert res.success is True
        assert box['args'] == b""
    finally:
        _teardown(teensy, client, node)


def test_bb_reload_invokes_rpc_with_empty_args():
    teensy, client, node = _node()
    try:
        _send_heartbeat(teensy)
        assert _wait_until(lambda: node._latest_heartbeat is not None)
        box = _capture(teensy, RpcMethod.BB_RELOAD)
        res = node._svc_bb_reload(Trigger.Request(), Trigger.Response())
        assert res.success is True
        assert box['args'] == b""
    finally:
        _teardown(teensy, client, node)


def test_bb_calibrate_invokes_rpc_with_empty_args_when_bb_present():
    """bb/calibrate forwards to BB_CALIBRATE_LOC when BB is present."""
    teensy, client, node = _node()
    try:
        _send_heartbeat(teensy, bb_state=1)
        assert _wait_until(lambda: node._latest_heartbeat is not None)
        box = _capture(teensy, RpcMethod.BB_CALIBRATE_LOC)
        res = node._svc_bb_calibrate(Trigger.Request(), Trigger.Response())
        assert res.success is True
        assert box['args'] == b""
    finally:
        _teardown(teensy, client, node)


def test_bb_calibrate_silently_succeeds_when_bb_absent():
    """bb/calibrate returns success=True without an RPC call when BB is
    silent — mirrors can_node._svc_bb_calibrate's HOMING-allowed semantics."""
    teensy, client, node = _node()
    try:
        _send_heartbeat(teensy, heartbeat_seen=False, heartbeat_stale=False)
        assert _wait_until(lambda: node._latest_heartbeat is not None)
        # Register a handler that would assert if called — proves no RPC issued.
        called = {'flag': False}

        def handler(req_id, args):
            called['flag'] = True
            return (int(RpcStatus.OK), b"")

        teensy.on_rpc(int(RpcMethod.BB_CALIBRATE_LOC), handler)
        res = node._svc_bb_calibrate(Trigger.Request(), Trigger.Response())
        assert res.success is True
        assert "skipped" in res.message.lower()
        assert called['flag'] is False
    finally:
        _teardown(teensy, client, node)


def test_bb_reset_propagates_failure_when_bb_absent():
    """bb/reset does NOT short-circuit on absent BB — the firmware's
    bb_present() gate returns ERR_BUS_DOWN and the bridge surfaces it.
    Only bb/calibrate has the silent-success behaviour."""
    teensy, client, node = _node()
    try:
        _send_heartbeat(teensy, heartbeat_seen=False)
        assert _wait_until(lambda: node._latest_heartbeat is not None)
        _capture(teensy, RpcMethod.BB_RESET, status=int(RpcStatus.ERR_BUS_DOWN))
        res = node._svc_bb_reset(Trigger.Request(), Trigger.Response())
        assert res.success is False
        assert "BUS_DOWN" in res.message or "Reset failed" in res.message
    finally:
        _teardown(teensy, client, node)


def test_bb_send_throw_command_arg_bytes():
    """bb/send_throw_command encodes args as ArgBbThrow and invokes BB_THROW."""
    teensy, client, node = _node()
    try:
        _send_heartbeat(teensy)
        assert _wait_until(lambda: node._latest_heartbeat is not None)
        box = _capture(teensy, RpcMethod.BB_THROW)
        req = SendBallButlerCommand.Request()
        req.yaw_angle_rad = 0.1
        req.pitch_angle_rad = 0.5
        req.throw_speed = 3.0
        req.throw_time = 1.0
        req.suppress_announcement = False  # field is dead protocol (D3)
        res = node._svc_bb_throw(req, SendBallButlerCommand.Response())
        assert res.success is True
        assert box['args'] == rpc_args.encode_bb_throw(0.1, 0.5, 3.0, 1.0)
    finally:
        _teardown(teensy, client, node)


def test_bb_send_throw_command_never_publishes_announcement():
    """D3: throw_director_node is the sole publisher of throw_announcements.
    The bridge MUST NOT create a throw_announcement publisher OR publish to
    that topic in response to bb/send_throw_command (regardless of the
    suppress_announcement field — it is dead protocol on this service)."""
    teensy, client, node = _node()
    try:
        _send_heartbeat(teensy)
        assert _wait_until(lambda: node._latest_heartbeat is not None)
        _capture(teensy, RpcMethod.BB_THROW)
        # Test both suppress=False and suppress=True — the bridge ignores it.
        for suppress in (False, True):
            req = SendBallButlerCommand.Request()
            req.yaw_angle_rad = 0.1
            req.pitch_angle_rad = 0.5
            req.throw_speed = 3.0
            req.throw_time = 1.0
            req.suppress_announcement = suppress
            node._svc_bb_throw(req, SendBallButlerCommand.Response())
        # No publisher named 'throw_announcements' exists on the bridge.
        assert 'throw_announcements' not in node._publishers
    finally:
        _teardown(teensy, client, node)
