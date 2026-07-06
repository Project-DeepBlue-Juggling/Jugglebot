"""RPC service-surface tests for teensy_bridge_node.

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


def test_vel_curr_limits_topic_applies_hand_axis6():
    """The can-bridge owns the hand ODrive (axis 6) on CAN3, so a legs+hand
    message applies the HAND limits too — reversing the earlier can_node-parity
    regression. Seven SET_VEL_CURR_LIMITS RPCs fire (6 legs + axis 6, in order) and
    the hand values are CACHED so a later _run_configure re-applies the operator's
    update rather than the config default."""
    from jugglebot.teensy_bridge_node import _HAND_AXIS
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
        msg.hand_vel_limit = 7.0
        msg.hand_curr_limit = 15.0
        node._sub_vel_curr_limits(msg)
        assert len(seen) == 7  # 6 legs + hand
        for axis in range(6):
            assert seen[axis] == rpc_args.encode_set_vel_curr_limits(axis, 4.0, 20.0)
        assert seen[6] == rpc_args.encode_set_vel_curr_limits(_HAND_AXIS, 7.0, 15.0)
        # Cache updated so _run_configure re-applies the operator's hand limits.
        assert node._hand_vel_limit == 7.0
        assert node._hand_curr_limit == 15.0
    finally:
        _teardown(teensy, client, node)


def test_vel_curr_limits_hand_cache_unchanged_on_failure():
    """If the hand SET_VEL_CURR_LIMITS RPC is REJECTED, the cached hand limits
    are left untouched — a later _run_configure must not push a value the hand ODrive
    refused. Legs still apply (the hand failure is isolated)."""
    from jugglebot.teensy_bridge_node import _HAND_AXIS
    teensy, client, node = _node()
    try:
        v0, c0 = node._hand_vel_limit, node._hand_curr_limit

        def handler(req_id, args):
            # args[0] is the axis byte (rpc_args.encode_set_vel_curr_limits).
            if args[0] == _HAND_AXIS:
                return (int(RpcStatus.ERR_REJECTED), b"")
            return (int(RpcStatus.OK), b"")

        teensy.on_rpc(int(RpcMethod.SET_VEL_CURR_LIMITS), handler)
        msg = SetMotorVelCurrLimitsMessage()
        msg.legs_vel_limit = 4.0
        msg.legs_curr_limit = 20.0
        msg.hand_vel_limit = 7.0
        msg.hand_curr_limit = 15.0
        node._sub_vel_curr_limits(msg)
        assert node._hand_vel_limit == v0
        assert node._hand_curr_limit == c0
    finally:
        _teardown(teensy, client, node)


# ── encoder_search: Jetson-side; home: firmware move ─

def test_encoder_search_is_jetson_side_not_firmware_stub():
    """Encoder index search was moved to a Jetson-side orchestration over
    SET_AXIS_STATE; the service must NOT route to the (stubbed) firmware
    ENCODER_SEARCH RPC. Full behaviour: test_teensy_bridge_node_encoder_search.py."""
    teensy, client, node = _node()
    try:
        called = {'n': 0}

        def stub(req_id, args):
            called['n'] += 1
            return (int(RpcStatus.ERR_NOT_IMPL), b"")

        teensy.on_rpc(int(RpcMethod.ENCODER_SEARCH), stub)
        node._params['encoder_search_axes'] = []   # empty scope -> fast return
        res = node._svc_encoder_search(Trigger.Request(), Trigger.Response())
        assert res.success is False
        assert called['n'] == 0   # the firmware ENCODER_SEARCH RPC is never used
    finally:
        _teardown(teensy, client, node)


def test_home_is_firmware_move_not_stub():
    """The HOME RPC now starts a firmware move-to-hardstop (no longer
    ERR_NOT_IMPL). The service reads home_axes and observes completion; with an
    empty scope it returns fast without touching the firmware. Full behaviour:
    test_teensy_bridge_node_home.py."""
    teensy, client, node = _node()
    try:
        called = {'n': 0}

        def stub(req_id, args):
            called['n'] += 1
            return (int(RpcStatus.OK), b"")

        teensy.on_rpc(int(RpcMethod.HOME), stub)
        node._params['home_axes'] = []   # empty scope -> fast return
        res = node._svc_home(Trigger.Request(), Trigger.Response())
        assert res.success is False
        assert 'no axes' in res.message
        assert called['n'] == 0   # no firmware HOME issued for an empty scope
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
