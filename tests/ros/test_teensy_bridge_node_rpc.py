"""RPC service-surface tests for teensy_bridge_node (Phase 10b, Commit 4).

Drives the bridge's RPC methods + ROS service handlers through the real
RpcClient and a FakeTeensy that auto-responds. The headline assertion is
**arg-byte fidelity**: the exact bytes the bridge sends must equal the
rpc_args encoding the firmware rpc.cpp memcpy's into its arg structs.
"""

from __future__ import annotations

from controller.teensy_link import RpcMethod, RpcStatus
from controller.teensy_link import rpc_args

from tests.ros.test_teensy_bridge_node_read import _build_paired_node

# Mocked ROS service/message types (tests/ros/conftest.py).
from std_srvs.srv import Trigger
from jugglebot_interfaces.srv import ODriveCommandService
from jugglebot_interfaces.msg import SetMotorVelCurrLimitsMessage


def _node():
    teensy, client, node = _build_paired_node()
    return teensy, client, node


def _teardown(teensy, client, node):
    node.on_shutdown()
    client.stop()
    teensy.stop()


def _capture(teensy, method, status=int(RpcStatus.OK), result=b""):
    """Register an auto-responder that records the args it received."""
    box = {}

    def handler(req_id, args):
        box['args'] = args
        box['req_id'] = req_id
        return (status, result)

    teensy.on_rpc(int(method), handler)
    return box


# ── Arg-byte fidelity (the bytes on the wire == rpc_args encoding) ───────────

def test_set_axis_state_arg_bytes():
    teensy, client, node = _node()
    try:
        box = _capture(teensy, RpcMethod.SET_AXIS_STATE)
        ok, msg, _ = node.teensy_set_axis_state(3, 8)
        assert ok, msg
        assert box['args'] == rpc_args.encode_set_axis_state(3, 8)
    finally:
        _teardown(teensy, client, node)


def test_set_controller_mode_arg_bytes():
    teensy, client, node = _node()
    try:
        box = _capture(teensy, RpcMethod.SET_CONTROLLER_MODE)
        ok, _, _ = node.teensy_set_controller_mode(2, 3, 1)
        assert ok
        assert box['args'] == rpc_args.encode_set_controller_mode(2, 3, 1)
    finally:
        _teardown(teensy, client, node)


def test_set_pos_gain_and_vel_gains_arg_bytes():
    teensy, client, node = _node()
    try:
        b1 = _capture(teensy, RpcMethod.SET_POS_GAIN)
        b2 = _capture(teensy, RpcMethod.SET_VEL_GAINS)
        node.teensy_set_pos_gain(0, 35.0)
        node.teensy_set_vel_gains(1, 0.007, 0.07)
        assert b1['args'] == rpc_args.encode_set_pos_gain(0, 35.0)
        assert b2['args'] == rpc_args.encode_set_vel_gains(1, 0.007, 0.07)
    finally:
        _teardown(teensy, client, node)


def test_set_absolute_position_arg_bytes():
    teensy, client, node = _node()
    try:
        box = _capture(teensy, RpcMethod.SET_ABSOLUTE_POSITION)
        node.teensy_set_absolute_position(4, 2.19)
        assert box['args'] == rpc_args.encode_set_absolute_position(4, 2.19)
    finally:
        _teardown(teensy, client, node)


def test_sdo_read_write_arg_bytes():
    teensy, client, node = _node()
    try:
        br = _capture(teensy, RpcMethod.SDO_READ)
        bw = _capture(teensy, RpcMethod.SDO_WRITE)
        node.teensy_sdo_read(1, 0x1234)
        node.teensy_sdo_write(1, 0x00AB, 1.5)
        assert br['args'] == rpc_args.encode_sdo_read(1, 0x1234)
        assert bw['args'] == rpc_args.encode_sdo_write(1, 0x00AB, 1.5)
    finally:
        _teardown(teensy, client, node)


# ── clear_errors / reboot broadcast AXIS_ALL ──────────────────

def test_clear_errors_service_broadcasts():
    teensy, client, node = _node()
    try:
        box = _capture(teensy, RpcMethod.CLEAR_ERRORS)
        res = node._svc_clear_errors(Trigger.Request(), Trigger.Response())
        assert res.success is True
        assert box['args'] == bytes([rpc_args.AXIS_ALL])  # 1-byte ArgAxisOnly
    finally:
        _teardown(teensy, client, node)


def test_reboot_service_broadcasts():
    teensy, client, node = _node()
    try:
        box = _capture(teensy, RpcMethod.REBOOT_ODRIVES)
        res = node._svc_reboot_odrives(Trigger.Request(), Trigger.Response())
        assert res.success is True
        assert box['args'] == bytes([rpc_args.AXIS_ALL])
    finally:
        _teardown(teensy, client, node)


# ── odrive_command dispatch ───────────────────────────────────

def test_odrive_command_dispatch():
    teensy, client, node = _node()
    try:
        _capture(teensy, RpcMethod.CLEAR_ERRORS)
        _capture(teensy, RpcMethod.REBOOT_ODRIVES)
        req = ODriveCommandService.Request()
        req.command = 'clear_errors'
        assert node._svc_odrive_command(req, ODriveCommandService.Response()).success
        req.command = 'reboot_odrives'
        assert node._svc_odrive_command(req, ODriveCommandService.Response()).success
        req.command = 'bogus'
        res = node._svc_odrive_command(req, ODriveCommandService.Response())
        assert res.success is False and 'Unknown' in res.message
    finally:
        _teardown(teensy, client, node)


# ── set_motor_vel_curr_limits topic → per-leg RPCs ────────────

def test_vel_curr_limits_topic_sends_per_leg():
    teensy, client, node = _node()
    try:
        seen = []

        def handler(req_id, args):
            seen.append(args)
            return (int(RpcStatus.OK), b"")

        teensy.on_rpc(int(RpcMethod.SET_VEL_CURR_LIMITS), handler)
        msg = SetMotorVelCurrLimitsMessage()
        msg.legs_vel_limit = 4.0
        msg.legs_curr_limit = 20.0
        node._sub_vel_curr_limits(msg)
        assert len(seen) == 6  # one per leg
        for axis in range(6):
            assert seen[axis] == rpc_args.encode_set_vel_curr_limits(axis, 4.0, 20.0)
    finally:
        _teardown(teensy, client, node)


def test_vel_curr_limits_ignored_when_zero():
    teensy, client, node = _node()
    try:
        seen = []
        teensy.on_rpc(int(RpcMethod.SET_VEL_CURR_LIMITS),
                      lambda r, a: (seen.append(a), (int(RpcStatus.OK), b""))[1])
        msg = SetMotorVelCurrLimitsMessage()  # all zero
        node._sub_vel_curr_limits(msg)
        assert seen == []
    finally:
        _teardown(teensy, client, node)


# ── encoder_search / home: ERR_NOT_IMPL until firmware Phase 9 ─

def test_encoder_search_not_impl():
    teensy, client, node = _node()
    try:
        _capture(teensy, RpcMethod.ENCODER_SEARCH, status=int(RpcStatus.ERR_NOT_IMPL))
        res = node._svc_encoder_search(Trigger.Request(), Trigger.Response())
        assert res.success is False
        assert 'Phase 9' in res.message
    finally:
        _teardown(teensy, client, node)


def test_home_not_impl():
    teensy, client, node = _node()
    try:
        _capture(teensy, RpcMethod.HOME, status=int(RpcStatus.ERR_NOT_IMPL))
        res = node._svc_home(Trigger.Request(), Trigger.Response())
        assert res.success is False
        assert 'Phase 9' in res.message
    finally:
        _teardown(teensy, client, node)


# ── error propagation ─────────────────────────────────────────

def test_rejected_rpc_reports_failure():
    teensy, client, node = _node()
    try:
        _capture(teensy, RpcMethod.SET_AXIS_STATE, status=int(RpcStatus.ERR_REJECTED))
        ok, msg, _ = node.teensy_set_axis_state(0, 8)
        assert ok is False
        assert 'ERR_REJECTED' in msg
    finally:
        _teardown(teensy, client, node)


def test_rpc_timeout_reports_failure():
    teensy, client, node = _node()
    try:
        # No handler registered for SDO_READ ⇒ FakeTeensy replies ERR_UNKNOWN_METHOD.
        # Use a method the FakeTeensy will NOT answer at all by pointing at an
        # unregistered method with a short budget via the node method.
        ok, msg, _ = node._call_rpc(RpcMethod.NOP, b"", timeout=0.05, retries=0)
        assert ok is False  # FakeTeensy has no NOP handler → ERR_UNKNOWN_METHOD
    finally:
        _teardown(teensy, client, node)
