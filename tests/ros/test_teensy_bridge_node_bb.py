"""Ball Butler bridge tests (cutover — bb/* moved from can_node).

Covers the BB surface that teensy_bridge_node inherits from can_node:

* ``bb/heartbeat`` publisher — sources state from the extended HeartbeatT2J
  (bb_state / bb_state_data / bb_flags / bb_yaw_deg / bb_pitch_deg /
  bb_hand_mm). Mirrors the legacy ``can_node._publish_bb_heartbeat`` field
  semantics so existing GUI / mocap / orchestrator consumers see no shift.
* ``bb/calibrate`` / ``bb/reload`` / ``bb/reset`` services — invoke the
  firmware RPCs (BB_CALIBRATE_LOC / BB_RELOAD / BB_RESET). The firmware gates
  each TX on bb_present(); we translate the resulting ERR_BUS_DOWN into a
  silent-success for bb/calibrate ONLY (to preserve can_node._svc_bb_calibrate's
  HOMING semantics). The other two propagate the error.
* ``bb/throw`` ACTION — dispatches BB_THROW and awaits the firmware's
  terminal CMD_RESULT (relayed back as a CAN frame), succeeding/aborting the
  goal with the firmware outcome. Driven here by injecting a CMD_RESULT through
  the FakeTeensy loopback.

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
from controller.teensy_link import protocol as p

from tests.ros.test_teensy_bridge_node_read import _build_paired_node, _wait_until

# Mocked ROS service/message + action types (tests/ros/conftest.py).
from std_srvs.srv import Trigger
from rclpy.action import GoalResponse
from jugglebot_interfaces.action import BallButlerThrowCmd
from jugglebot.protocol_config import BallButlerCommandType, BallButlerCommandOutcome


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


# ── bb/throw ACTION (dispatch + CMD_RESULT completion) ─────────────


def _inject_cmd_result(teensy, *, cmd_type, outcome, detail0=0, detail1=0):
    """Inject a relayed CMD_RESULT frame from the FakeTeensy (CAN1 loud channel).

    Mirrors the can-bridge relay: an 8-byte CAN payload
    [cmd_type, outcome, d0_lo, d0_hi, d1_lo, d1_hi, 0, 0] wrapped in a
    CMD_RESULT stream frame the bridge's _on_cmd_result handler decodes.
    """
    payload = bytes([
        cmd_type & 0xFF, outcome & 0xFF,
        detail0 & 0xFF, (detail0 >> 8) & 0xFF,
        detail1 & 0xFF, (detail1 >> 8) & 0xFF, 0, 0,
    ])
    cf = p.CmdResultFrame(t_bridge_us=1, can_id=0x7D5, dlc=8, data=tuple(payload))
    teensy.send_to_jetson(int(MsgType.CMD_RESULT), cf.pack())


def _throw_goal(yaw=0.1, pitch=0.5, speed=3.0, throw_time=1.0):
    g = BallButlerThrowCmd.Goal()
    g.yaw_angle_rad = yaw
    g.pitch_angle_rad = pitch
    g.throw_speed = speed
    g.throw_time = throw_time
    g.suppress_announcement = False
    return g


class _FakeGoalHandle:
    """Minimal goal_handle for driving _bb_throw_execute directly."""
    def __init__(self, goal):
        self.request = goal
        self.succeeded = False
        self.aborted = False

    def succeed(self):
        self.succeeded = True

    def abort(self):
        self.aborted = True


def test_bb_throw_goal_rejects_second_while_outstanding():
    """One throw at a time: goal_callback accepts the first, rejects a second
    while it's outstanding, then accepts again once cleared (serialized firmware,
    no correlation token)."""
    teensy, client, node = _node()
    try:
        assert node._bb_throw_goal(_throw_goal()) == GoalResponse.ACCEPT
        assert node._bb_throw_goal(_throw_goal()) == GoalResponse.REJECT
        with node._bb_throw_lock:
            node._bb_throw_active = False
        assert node._bb_throw_goal(_throw_goal()) == GoalResponse.ACCEPT
    finally:
        _teardown(teensy, client, node)


def test_bb_throw_action_aim_only_dispatches_without_awaiting():
    """A speed==0 goal is an aim/track command — dispatched and completed
    immediately (firmware requestTracking emits no CMD_RESULT). It must NOT
    consume the single-throw slot or block on a result (which would hang until
    the timeout)."""
    teensy, client, node = _node()
    try:
        _send_heartbeat(teensy)
        assert _wait_until(lambda: node._latest_heartbeat is not None)
        box = _capture(teensy, RpcMethod.BB_THROW)   # acks OK, injects no CMD_RESULT
        goal = _throw_goal(yaw=0.2, pitch=0.6, speed=0.0, throw_time=0.0)
        assert node._bb_throw_goal(goal) == GoalResponse.ACCEPT
        assert node._bb_throw_active is False        # slot not consumed
        handle = _FakeGoalHandle(goal)
        result = node._bb_throw_execute(handle)
        assert box['args'] == rpc_args.encode_bb_throw(0.2, 0.6, 0.0, 0.0)
        assert result.success is True
        assert handle.succeeded is True
        assert node._bb_throw_active is False
    finally:
        _teardown(teensy, client, node)


def test_bb_throw_action_success_via_cmd_result():
    """Full loopback: BB_THROW dispatched with the encoded args, FakeTeensy
    relays an OK CMD_RESULT, the action result reports success and succeeds."""
    teensy, client, node = _node()
    try:
        _send_heartbeat(teensy)
        assert _wait_until(lambda: node._latest_heartbeat is not None)
        box = {}

        def handler(req_id, args):
            box['args'] = args
            _inject_cmd_result(
                teensy,
                cmd_type=int(BallButlerCommandType.THROW),
                outcome=int(BallButlerCommandOutcome.OK),
                detail0=-1)
            return (int(RpcStatus.OK), b"")

        teensy.on_rpc(int(RpcMethod.BB_THROW), handler)

        goal = _throw_goal(yaw=0.1, pitch=0.5, speed=3.0, throw_time=1.0)
        assert node._bb_throw_goal(goal) == GoalResponse.ACCEPT
        handle = _FakeGoalHandle(goal)
        result = node._bb_throw_execute(handle)

        assert box['args'] == rpc_args.encode_bb_throw(0.1, 0.5, 3.0, 1.0)
        assert result.success is True
        assert result.outcome == int(BallButlerCommandOutcome.OK)
        assert handle.succeeded is True
        assert node._bb_throw_active is False  # cleared in execute's finally
    finally:
        _teardown(teensy, client, node)


def test_bb_throw_action_abort_carries_reason_and_axis():
    """A non-OK outcome aborts the goal and surfaces the reason + decoded
    details (binding axis / metric) in the result."""
    teensy, client, node = _node()
    try:
        _send_heartbeat(teensy)
        assert _wait_until(lambda: node._latest_heartbeat is not None)

        def handler(req_id, args):
            _inject_cmd_result(
                teensy,
                cmd_type=int(BallButlerCommandType.THROW),
                outcome=int(BallButlerCommandOutcome.THROW_ABORTED_NOT_SETTLED),
                detail0=1, detail1=250)  # axis=PITCH, yaw err 2.50°
            return (int(RpcStatus.OK), b"")

        teensy.on_rpc(int(RpcMethod.BB_THROW), handler)

        goal = _throw_goal()
        assert node._bb_throw_goal(goal) == GoalResponse.ACCEPT
        handle = _FakeGoalHandle(goal)
        result = node._bb_throw_execute(handle)

        assert result.success is False
        assert result.outcome == int(
            BallButlerCommandOutcome.THROW_ABORTED_NOT_SETTLED)
        assert result.detail0 == 1
        assert result.detail1 == 250
        assert 'PITCH' in result.message
        assert handle.aborted is True
    finally:
        _teardown(teensy, client, node)


def test_bb_throw_action_dispatch_failure_aborts_without_waiting():
    """If the bridge can't queue the frame (BB absent → ERR_BUS_DOWN), abort
    immediately rather than waiting for a CMD_RESULT that will never come."""
    teensy, client, node = _node()
    try:
        _send_heartbeat(teensy, heartbeat_seen=False)
        assert _wait_until(lambda: node._latest_heartbeat is not None)
        _capture(teensy, RpcMethod.BB_THROW, status=int(RpcStatus.ERR_BUS_DOWN))
        goal = _throw_goal()
        assert node._bb_throw_goal(goal) == GoalResponse.ACCEPT
        handle = _FakeGoalHandle(goal)
        result = node._bb_throw_execute(handle)
        assert result.success is False
        assert handle.aborted is True
        assert 'dispatch failed' in result.message.lower()
    finally:
        _teardown(teensy, client, node)


def test_bb_throw_action_times_out_when_firmware_silent():
    """BB acks the RPC but never emits a CMD_RESULT → the action times out
    (doesn't hang) with a TIMEOUT outcome and aborts the goal."""
    teensy, client, node = _node()
    try:
        _send_heartbeat(teensy)
        assert _wait_until(lambda: node._latest_heartbeat is not None)
        _capture(teensy, RpcMethod.BB_THROW)   # acks OK, injects no CMD_RESULT
        node._BB_THROW_RESULT_MARGIN_S = 0.2   # keep the test fast
        goal = _throw_goal(throw_time=0.0)
        assert node._bb_throw_goal(goal) == GoalResponse.ACCEPT
        handle = _FakeGoalHandle(goal)
        result = node._bb_throw_execute(handle)
        assert result.success is False
        assert result.outcome == int(BallButlerCommandOutcome.TIMEOUT)
        assert handle.aborted is True
    finally:
        _teardown(teensy, client, node)
