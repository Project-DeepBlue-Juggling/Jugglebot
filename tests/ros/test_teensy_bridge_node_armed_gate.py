"""can-bridge FW 52 (BUG-99 D1): the host side of the Armed reconfiguration gate.

The bridge refuses gain, mode, non-IDLE state, absolute-position and SDO writes while
Armed (rpc.cpp refused_while_armed, its one enforcement point send_axis_frame). The
node refuses the operator services that would send them while Armed, before any RPC,
with a message naming the reason; IDLE (a stop) is never refused. "Armed" is either
side's view: this node's ``_mpc_active`` or the bridge's T2J MPC_ACTIVE bit, which
lags a disarm by up to one heartbeat. ROS 2 is mocked by tests/ros/conftest.py.
"""
from __future__ import annotations

import types

from teensy_link import RpcMethod, RpcStatus
from teensy_link import rpc_args
from teensy_link import protocol as p
from tests.ros._bridge_harness import _build_paired_node, _teardown

import jugglebot.protocol_config as proto

_HAND = p.NUM_LEGS


def _capture(teensy, method, status=int(RpcStatus.OK)):
    box = {'calls': []}

    def handler(req_id, args):
        box['calls'].append(args)
        return (status, b"")
    teensy.on_rpc(int(method), handler)
    return box


def _res():
    return types.SimpleNamespace(success=None, message='')


def _bridge_armed(node, armed=True):
    """The bridge's own view (T2J bit 3), independent of the node's _mpc_active."""
    flags = int(p.HeartbeatT2JFlags.MPC_ACTIVE) if armed else 0
    with node._lock:
        node._latest_heartbeat = types.SimpleNamespace(flags=flags, fault_state=0)


def test_set_hand_gains_refused_while_armed_no_rpc():
    teensy, client, node = _build_paired_node()
    try:
        bp = _capture(teensy, RpcMethod.SET_POS_GAIN)
        bv = _capture(teensy, RpcMethod.SET_VEL_GAINS)
        node._mpc_active = True
        req = types.SimpleNamespace(pos_gain=40.0, vel_gain=0.01, vel_integrator_gain=0.09)
        out = node._svc_set_hand_gains(req, _res())
        assert out.success is False
        assert 'Armed' in out.message and 'Disarm first' in out.message
        assert bp['calls'] == [] and bv['calls'] == []   # nothing half-applied
    finally:
        node._mpc_active = False
        _teardown(teensy, client, node)


def test_set_hand_gains_refused_while_only_the_bridge_still_reports_armed():
    teensy, client, node = _build_paired_node()
    try:
        bp = _capture(teensy, RpcMethod.SET_POS_GAIN)
        _bridge_armed(node, True)          # the disarm has not reached the bridge yet
        req = types.SimpleNamespace(pos_gain=40.0, vel_gain=0.01, vel_integrator_gain=0.09)
        out = node._svc_set_hand_gains(req, _res())
        assert out.success is False and 'Armed' in out.message
        assert bp['calls'] == []
    finally:
        _teardown(teensy, client, node)


def test_set_hand_state_closed_loop_refused_while_armed_idle_allowed():
    teensy, client, node = _build_paired_node()
    try:
        box = _capture(teensy, RpcMethod.SET_AXIS_STATE)
        node._mpc_active = True
        out = node._svc_set_hand_state(types.SimpleNamespace(data='CLOSED_LOOP'), _res())
        assert out.success is False and 'Armed' in out.message
        assert box['calls'] == []
        out = node._svc_set_hand_state(types.SimpleNamespace(data='IDLE'), _res())
        assert out.success is True, out.message
        assert box['calls'] == [rpc_args.encode_set_axis_state(_HAND, proto.ODRIVE_STATES['IDLE'])]
    finally:
        node._mpc_active = False
        _teardown(teensy, client, node)


def test_configure_refused_while_armed_no_rpc():
    teensy, client, node = _build_paired_node()
    try:
        boxes = [_capture(teensy, m) for m in (RpcMethod.SET_POS_GAIN, RpcMethod.SET_VEL_GAINS,
                                               RpcMethod.SET_CONTROLLER_MODE,
                                               RpcMethod.SET_VEL_CURR_LIMITS)]
        node._mpc_active = True
        out = node._svc_configure(types.SimpleNamespace(), _res())
        assert out.success is False and '/configure' in out.message and 'Armed' in out.message
        assert all(b['calls'] == [] for b in boxes)
    finally:
        node._mpc_active = False
        _teardown(teensy, client, node)


def test_disarmed_services_unchanged():
    teensy, client, node = _build_paired_node()
    try:
        _capture(teensy, RpcMethod.SET_POS_GAIN)
        _capture(teensy, RpcMethod.SET_VEL_GAINS)
        _bridge_armed(node, False)
        req = types.SimpleNamespace(pos_gain=40.0, vel_gain=0.01, vel_integrator_gain=0.09)
        out = node._svc_set_hand_gains(req, _res())
        assert out.success is True, out.message
    finally:
        _teardown(teensy, client, node)


def test_err_rejected_from_the_gate_is_named_in_the_rpc_message():
    teensy, client, node = _build_paired_node()
    try:
        _capture(teensy, RpcMethod.SET_ABSOLUTE_POSITION, status=int(RpcStatus.ERR_REJECTED))
        _bridge_armed(node, True)
        ok, msg, _ = node.teensy_set_absolute_position(0, 1.0)
        assert ok is False
        assert 'ERR_REJECTED' in msg and rpc_args.ARMED_REFUSAL_HINT in msg
        # Disarmed, the same status is not attributed to the gate.
        _bridge_armed(node, False)
        ok, msg, _ = node.teensy_set_absolute_position(0, 1.0)
        assert ok is False and rpc_args.ARMED_REFUSAL_HINT not in msg
    finally:
        _teardown(teensy, client, node)


def test_reboot_odrives_refused_while_armed_before_the_rpc_and_the_cold_start_clear():
    """Orchestrator decision 2026-11-25 (the D1 intent: a reboot drops a live loop):
    /reboot_odrives and odrive_command 'reboot_odrives' share _reboot_odrives, which
    refuses while Armed with no RPC and WITHOUT the cold-start clear (is_homed must
    not drop for a reboot that never happened). Disarmed, it reboots as before."""
    teensy, client, node = _build_paired_node()
    cleared = []
    try:
        box = _capture(teensy, RpcMethod.REBOOT_ODRIVES)
        node._clear_cold_start_state_on_reboot = lambda: (cleared.append(1), (True, 'ok'))[1]
        _bridge_armed(node, True)
        ok, msg = node._reboot_odrives()
        assert ok is False and 'REFUSED' in msg and 'Armed' in msg
        assert box['calls'] == [] and cleared == []
        _bridge_armed(node, False)
        node._mpc_active = True                       # the node's own view counts too
        ok, msg = node._reboot_odrives()
        assert ok is False and box['calls'] == [] and cleared == []
        node._mpc_active = False
        ok, msg = node._reboot_odrives()
        assert ok is True, msg
        assert len(box['calls']) == 1 and cleared == [1]
    finally:
        node._mpc_active = False
        _teardown(teensy, client, node)
