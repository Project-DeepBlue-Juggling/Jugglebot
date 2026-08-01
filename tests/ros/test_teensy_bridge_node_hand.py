"""Node-level tests for the hand command surface in teensy_bridge_node.

The byte-exact 0x6D0 wire parity is covered in tests/firmware/test_hand_traj_xref.py;
the firmware conduit in tests/firmware/native/test_hand_ops.cpp. Here we test the
BRIDGE glue for the hand conduit (see logbook 2026-07-01-canbridge-phase5-hand-conduit):

  * the four hand services (set_hand_state / set_hand_gains / set_hand_traj_cmd /
    smooth_move_hand) — Jetson-side validation (reject before a byte hits CAN3) +
    the RPC args they emit;
  * HAND_CMD_ECHO → hand_telemetry pos_cmd/vel_ff_cmd/tor_ff_cmd (the sniffed hand
    Set_Input_Pos echo, byte-identical decode to can_node._handle_hand_input_pos);
  * cold-start hand handling: _apply_hand_gains refuse-flash-defaults, _run_configure
    configuring the hand, _run_home applying hand gains before HOME(6), and
    _run_deactivate idling the hand.

ROS 2 is mocked by tests/ros/conftest.py.
"""

from __future__ import annotations

import struct
import threading
import time
import types

from controller.teensy_link import (
    RpcMethod, RpcStatus, MsgType, Telemetry, Diagnostic, HandCmdEcho,
)
from controller.teensy_link import rpc_args
from controller.teensy_link import protocol as p
from controller.teensy_link.homing import HOMING_RUNNING, HOMING_OK  # firmware-reported homing outcome (see logbook 2026-07-05-canhub-hardening-18a-homing-result-uplink)

from tests.ros._bridge_harness import _build_paired_node, _poll, _teardown

import jugglebot.hardware_config as hw
import jugglebot.protocol_config as proto

_HAND = p.NUM_LEGS  # axis 6
IDLE = 1
CLOSED_LOOP = 8


def _capture(teensy, method, status=int(RpcStatus.OK), result=b""):
    """Register an auto-responder that records every (req_id, args) it receives."""
    box = {'calls': []}

    def handler(req_id, args):
        box['calls'].append(args)
        box['args'] = args
        return (status, result)

    teensy.on_rpc(int(method), handler)
    return box


# ── set_hand_state ────────────────────────────────────────────────────────────

def test_set_hand_state_valid_sends_axis6():
    teensy, client, node = _build_paired_node()
    try:
        box = _capture(teensy, RpcMethod.SET_AXIS_STATE)
        res = types.SimpleNamespace(success=None, message='')
        out = node._svc_set_hand_state(types.SimpleNamespace(data='CLOSED_LOOP'), res)
        assert out.success is True, out.message
        assert box['args'] == rpc_args.encode_set_axis_state(_HAND, proto.ODRIVE_STATES['CLOSED_LOOP'])
    finally:
        _teardown(teensy, client, node)


def test_set_hand_state_unknown_string_rejected_no_rpc():
    teensy, client, node = _build_paired_node()
    try:
        box = _capture(teensy, RpcMethod.SET_AXIS_STATE)
        res = types.SimpleNamespace(success=None, message='')
        out = node._svc_set_hand_state(types.SimpleNamespace(data='NONSENSE'), res)
        assert out.success is False and 'Unknown' in out.message
        assert box['calls'] == []          # rejected Jetson-side — nothing on CAN3
    finally:
        _teardown(teensy, client, node)


# ── set_hand_gains ────────────────────────────────────────────────────────────

def test_set_hand_gains_sends_pos_and_vel_and_remembers():
    teensy, client, node = _build_paired_node()
    try:
        bp = _capture(teensy, RpcMethod.SET_POS_GAIN)
        bv = _capture(teensy, RpcMethod.SET_VEL_GAINS)
        req = types.SimpleNamespace(pos_gain=40.0, vel_gain=0.01, vel_integrator_gain=0.09)
        res = types.SimpleNamespace(success=None, message='')
        out = node._svc_set_hand_gains(req, res)
        assert out.success is True, out.message
        assert bp['args'] == rpc_args.encode_set_pos_gain(_HAND, 40.0)
        assert bv['args'] == rpc_args.encode_set_vel_gains(_HAND, 0.01, 0.09)
        # Remembered for the cold-start hand config/home path.
        assert node._hand_gains == {'pos_gain': 40.0, 'vel_gain': 0.01, 'vel_int_gain': 0.09}
    finally:
        _teardown(teensy, client, node)


# ── set_hand_traj_cmd ─────────────────────────────────────────────────────────

def test_set_hand_traj_valid_emits_hand_traj_cmd():
    teensy, client, node = _build_paired_node()
    try:
        box = _capture(teensy, RpcMethod.HAND_TRAJ_CMD)
        before_ms = int(time.time() * 1000)
        req = types.SimpleNamespace(event_delay=0.5, event_vel=3.75, traj_type=1)
        res = types.SimpleNamespace(success=None, message='')
        out = node._svc_set_hand_traj(req, res)
        after_ms = int(time.time() * 1000)
        assert out.success is True, out.message
        args = box['args']
        assert len(args) == 8
        assert args[0] == 1                                   # traj_type discriminator
        assert (args[1] | (args[2] << 8)) == int(round(3.75 * 100))  # vel_u16
        # bytes 3-6 = the LOW 32 bits of the absolute Jetson wall-clock deadline
        # (now_ms + event_delay*1000), masked like can_node. Compare against the
        # low-32 window [before, after] (no 2^32-ms wrap in a ~ms call window).
        wall = struct.unpack('<I', args[3:7])[0]
        lo = (before_ms + 500) & 0xFFFFFFFF
        hi = (after_ms + 500) & 0xFFFFFFFF
        assert lo <= wall <= hi, (lo, wall, hi)
        assert args[7] == 0
    finally:
        _teardown(teensy, client, node)


def test_set_hand_traj_validation_rejects_no_rpc():
    teensy, client, node = _build_paired_node()
    try:
        box = _capture(teensy, RpcMethod.HAND_TRAJ_CMD)
        for req in (
            types.SimpleNamespace(event_delay=0.0, event_vel=3.0, traj_type=1),    # delay <= 0
            types.SimpleNamespace(event_delay=0.5, event_vel=0.0, traj_type=1),    # vel < min
            types.SimpleNamespace(event_delay=0.5, event_vel=99.0, traj_type=1),   # vel > max
            types.SimpleNamespace(event_delay=0.5, event_vel=3.0, traj_type=3),    # bad type (3 is smooth-move only)
        ):
            res = types.SimpleNamespace(success=None, message='')
            out = node._svc_set_hand_traj(req, res)
            assert out.success is False, req
        assert box['calls'] == []          # every invalid request rejected before CAN3
    finally:
        _teardown(teensy, client, node)


# ── smooth_move_hand ──────────────────────────────────────────────────────────

def test_smooth_move_valid_byte_exact():
    teensy, client, node = _build_paired_node()
    try:
        box = _capture(teensy, RpcMethod.HAND_TRAJ_CMD)
        res = types.SimpleNamespace(success=None, message='')
        out = node._svc_smooth_move_hand(types.SimpleNamespace(data=9.858), res)
        assert out.success is True, out.message
        assert box['args'] == rpc_args.encode_smooth_move_hand(9.858)
    finally:
        _teardown(teensy, client, node)


def test_smooth_move_out_of_range_rejected_no_rpc():
    teensy, client, node = _build_paired_node()
    try:
        box = _capture(teensy, RpcMethod.HAND_TRAJ_CMD)
        for bad in (-0.1, 1e9):
            res = types.SimpleNamespace(success=None, message='')
            out = node._svc_smooth_move_hand(types.SimpleNamespace(data=bad), res)
            assert out.success is False
        assert box['calls'] == []
    finally:
        _teardown(teensy, client, node)


# ── HAND_CMD_ECHO → hand_telemetry command fields ─────────────────────────────

def test_hand_cmd_echo_fills_hand_telemetry():
    teensy, client, node = _build_paired_node()
    try:
        # A sniffed ODrive Set_Input_Pos payload: pos=5.5 rev, vel_ff=250, tor_ff=-30.
        data = struct.pack('<fhh', 5.5, 250, -30)
        teensy.send_to_jetson(int(MsgType.HAND_CMD_ECHO),
                              HandCmdEcho(t_bridge_us=42, data=tuple(data)).pack())
        assert _poll(lambda: node._last_hand_cmd['pos'] != 0.0)
        # Decoded byte-identically to can_node: vel/tor ÷ INPUT_SCALE_HAND_*(100).
        assert abs(node._last_hand_cmd['pos'] - 5.5) < 1e-6
        assert abs(node._last_hand_cmd['vel'] - 2.5) < 1e-6
        assert abs(node._last_hand_cmd['tor'] + 0.3) < 1e-6

        # _publish_hand_telemetry now echoes the command fields (was hardcoded 0).
        with node._lock:
            node._latest_telemetry = Telemetry(
                t_teensy_us=0,
                pos_rev=tuple(0.0 for _ in range(p.NUM_AXES)),
                vel_rps=tuple(0.0 for _ in range(p.NUM_AXES)))
        node._publish_hand_telemetry()
        msg = node.hand_telemetry_pub.published[-1]
        assert abs(msg.pos_cmd - 5.5) < 1e-6
        assert abs(msg.vel_ff_cmd - 2.5) < 1e-6
        assert abs(msg.tor_ff_cmd + 0.3) < 1e-6
    finally:
        _teardown(teensy, client, node)


# ── _apply_hand_gains (refuse-flash-defaults) ─────────────────────────────────

def test_apply_hand_gains_success():
    teensy, client, node = _build_paired_node()
    try:
        bp = _capture(teensy, RpcMethod.SET_POS_GAIN)
        bv = _capture(teensy, RpcMethod.SET_VEL_GAINS)
        ok, msg = node._apply_hand_gains()
        assert ok, msg
        # Uses the config-default hand gains (odrive.DEFAULT_HAND_GAINS).
        assert bp['args'] == rpc_args.encode_set_pos_gain(_HAND, hw.ODRIVE_HAND_POS_GAIN)
        assert bv['args'] == rpc_args.encode_set_vel_gains(
            _HAND, hw.ODRIVE_HAND_VEL_GAIN, hw.ODRIVE_HAND_VEL_INT_GAIN)
    finally:
        _teardown(teensy, client, node)


def test_apply_hand_gains_refuse_flash_defaults_on_failure():
    teensy, client, node = _build_paired_node()
    try:
        # SET_POS_GAIN fails to send → _apply_hand_gains reports failure (the caller
        # must NOT proceed to home/configure the hand on flash defaults).
        _capture(teensy, RpcMethod.SET_POS_GAIN, status=int(RpcStatus.ERR_TIMEOUT))
        _capture(teensy, RpcMethod.SET_VEL_GAINS)
        ok, msg = node._apply_hand_gains()
        assert not ok and 'hand.pos_gain' in msg
    finally:
        _teardown(teensy, client, node)


# ── _run_configure configures the hand───────────────────────────────

def test_run_configure_hand_applies_gains_limits_mode():
    teensy, client, node = _build_paired_node()
    try:
        bp = _capture(teensy, RpcMethod.SET_POS_GAIN)
        bv = _capture(teensy, RpcMethod.SET_VEL_GAINS)
        bl = _capture(teensy, RpcMethod.SET_VEL_CURR_LIMITS)
        bm = _capture(teensy, RpcMethod.SET_CONTROLLER_MODE)
        ok, msg = node._run_configure([_HAND])
        assert ok, msg
        assert bp['args'] == rpc_args.encode_set_pos_gain(_HAND, hw.ODRIVE_HAND_POS_GAIN)
        assert bv['args'] == rpc_args.encode_set_vel_gains(
            _HAND, hw.ODRIVE_HAND_VEL_GAIN, hw.ODRIVE_HAND_VEL_INT_GAIN)
        assert bl['args'] == rpc_args.encode_set_vel_curr_limits(
            _HAND, node._hand_vel_limit, node._hand_curr_limit)
        ctrl = proto.ODRIVE_CONTROL_MODES['POSITION']
        inp = proto.ODRIVE_INPUT_MODES['PASSTHROUGH']
        assert bm['args'] == rpc_args.encode_set_controller_mode(_HAND, ctrl, inp)
    finally:
        _teardown(teensy, client, node)


# ── _run_home hand branch (gains before HOME(6)) ──────────────────────────────

def test_run_home_hand_gain_failure_aborts_before_home():
    teensy, client, node = _build_paired_node()
    try:
        # Gain write fails → the hand home aborts BEFORE any HOME is fired
        # (refuse-flash-defaults: don't drive the hand into a stop on bad gains).
        _capture(teensy, RpcMethod.SET_POS_GAIN, status=int(RpcStatus.ERR_TIMEOUT))
        _capture(teensy, RpcMethod.SET_VEL_GAINS)
        home_box = _capture(teensy, RpcMethod.HOME)
        ok, msg = node._run_home([_HAND], poll_dt=0.02)
        assert not ok and 'hand gain' in msg.lower()
        assert home_box['calls'] == []     # HOME never fired
    finally:
        _teardown(teensy, client, node)


def test_run_home_hand_applies_gains_then_homes():
    teensy, client, node = _build_paired_node()
    try:
        bp = _capture(teensy, RpcMethod.SET_POS_GAIN)
        bv = _capture(teensy, RpcMethod.SET_VEL_GAINS)
        home_box = _capture(teensy, RpcMethod.HOME)
        result = {}

        def run():
            ok, msg = node._run_home([_HAND], poll_dt=0.02)
            result['ok'], result['msg'] = ok, msg

        t = threading.Thread(target=run)
        t.start()
        assert _poll(lambda: len(home_box['calls']) >= 1)   # HOME(6) fired
        # Firmware ran the move-to-hardstop: CLOSED_LOOP then back to IDLE → success.
        teensy.send_telemetry(pos_rev=[0.0] * 7, vel_rps=[0.0] * 7)
        teensy.send_to_jetson(int(MsgType.DIAGNOSTIC),
                              Diagnostic(axis_id=_HAND, axis_state=CLOSED_LOOP, active_errors=0,
                                         homing_result=HOMING_RUNNING).pack())
        time.sleep(0.1)
        teensy.send_telemetry(pos_rev=[0.0] * 7, vel_rps=[0.0] * 7)
        teensy.send_to_jetson(int(MsgType.DIAGNOSTIC),
                              Diagnostic(axis_id=_HAND, axis_state=IDLE, active_errors=0,
                                         homing_result=HOMING_OK).pack())
        t.join(timeout=8.0)
        assert not t.is_alive(), "hand home did not complete"
        assert result.get('ok') is True, result.get('msg')
        # Gains were applied BEFORE the HOME fired (byte-identical to _set_hand_gains).
        assert bp['args'] == rpc_args.encode_set_pos_gain(_HAND, hw.ODRIVE_HAND_POS_GAIN)
        assert bv['args'] == rpc_args.encode_set_vel_gains(
            _HAND, hw.ODRIVE_HAND_VEL_GAIN, hw.ODRIVE_HAND_VEL_INT_GAIN)
        assert home_box['args'] == rpc_args.encode_home(_HAND)
    finally:
        _teardown(teensy, client, node)


# ── _run_deactivate idles the hand───────────────────────────────────

def test_run_deactivate_idles_the_hand():
    teensy, client, node = _build_paired_node()
    try:
        teensy.on_rpc(int(RpcMethod.DEACTIVATE),
                      lambda req_id, args: (int(RpcStatus.OK), b""))
        hand_box = _capture(teensy, RpcMethod.SET_AXIS_STATE)
        result = {}

        def run():
            ok, msg = node._run_deactivate([0], poll_dt=0.02)
            result['ok'], result['msg'] = ok, msg

        t = threading.Thread(target=run)
        t.start()
        time.sleep(0.1)   # let the thread start + DEACTIVATE fire
        # The leg descends (CLOSED_LOOP) then IDLEs at STOW → deactivate completes
        # (the monitor must see the move run before IDLE counts as arrival).
        teensy.send_telemetry(pos_rev=[1.0] + [0.0] * 6, vel_rps=[-2.0] + [0.0] * 6)
        teensy.send_to_jetson(int(MsgType.DIAGNOSTIC),
                              Diagnostic(axis_id=0, axis_state=CLOSED_LOOP, active_errors=0).pack())
        time.sleep(0.1)
        teensy.send_telemetry(pos_rev=[0.0] * 7, vel_rps=[0.0] * 7)
        teensy.send_to_jetson(int(MsgType.DIAGNOSTIC),
                              Diagnostic(axis_id=0, axis_state=IDLE, active_errors=0).pack())
        t.join(timeout=8.0)
        assert not t.is_alive(), "deactivate did not complete"
        # The hand was idled: SET_AXIS_STATE(6, IDLE) — can_node JUGGLEBOT_AXES parity.
        assert rpc_args.encode_set_axis_state(_HAND, IDLE) in hand_box['calls']
    finally:
        _teardown(teensy, client, node)
