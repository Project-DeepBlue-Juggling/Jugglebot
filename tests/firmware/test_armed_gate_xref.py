"""can-bridge FW 52 (BUG-99 D1): the host twin of the Armed reconfiguration gate.

``teensy_link.rpc_args.refused_while_armed`` must agree with the firmware's one
predicate (``rpc.cpp`` ``refused_while_armed``) and the predicate must be called at
its one enforcement point (``send_axis_frame``) and nowhere else. A firmware edit
that adds or drops a refused method without the twin (or the reverse) fails here.
"""
from __future__ import annotations

import pathlib
import re

from teensy_link import rpc_args
from teensy_link.protocol import RpcMethod

_RPC_CPP = (pathlib.Path(__file__).resolve().parents[2]
            / 'ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp')


def _predicate_body(text):
    m = re.search(r'static bool refused_while_armed\([^)]*\) \{(.*?)\n\}\n', text, re.S)
    assert m, 'refused_while_armed not found in rpc.cpp'
    return m.group(1)


def test_the_firmware_always_refused_set_matches_the_twin():
    body = _predicate_body(_RPC_CPP.read_text(encoding='utf-8'))
    head = body.split('return true;')[0]
    fw_always = {getattr(RpcMethod, n) for n in re.findall(r'case RpcMethod::(\w+):', head)}
    twin_always = {m for m in RpcMethod
                   if all(rpc_args.refused_while_armed(m, ax, 8) for ax in (0, 6))
                   and rpc_args.refused_while_armed(m, 0, 1)}
    assert fw_always == twin_always == {RpcMethod.SET_ABSOLUTE_POSITION, RpcMethod.SDO_WRITE,
                                        RpcMethod.SET_CONTROLLER_MODE, RpcMethod.SET_VEL_GAINS,
                                        RpcMethod.REBOOT_ODRIVES}
    assert 'case RpcMethod::SET_POS_GAIN:\n      return axis < NUM_LEGS;' in body
    assert 'return requested_state != ODriveState::IDLE;' in body


def test_the_twin_conditional_rows():
    assert rpc_args.refused_while_armed(RpcMethod.SET_POS_GAIN, 0)
    assert rpc_args.refused_while_armed(RpcMethod.SET_POS_GAIN, 5)
    assert not rpc_args.refused_while_armed(RpcMethod.SET_POS_GAIN, 6)       # the Hand
    assert rpc_args.refused_while_armed(RpcMethod.SET_AXIS_STATE, 6, 8)      # CLOSED_LOOP
    assert rpc_args.refused_while_armed(RpcMethod.SET_AXIS_STATE, 0, 6)      # ENCODER_INDEX_SEARCH
    assert not rpc_args.refused_while_armed(RpcMethod.SET_AXIS_STATE, 6, 1)  # IDLE, a stop
    for m in (RpcMethod.CLEAR_ERRORS, RpcMethod.SET_VEL_CURR_LIMITS, RpcMethod.SDO_READ):
        assert not rpc_args.refused_while_armed(m, 0)
    for ax in (0, 6, rpc_args.AXIS_ALL):                                     # orchestrator 2026-11-25
        assert rpc_args.refused_while_armed(RpcMethod.REBOOT_ODRIVES, ax)


def test_the_enforcement_points():
    """Two call sites and no more: axis_precheck (every per-axis frame; FW 53 split it out
    of send_axis_frame so the Stow-gated methods share it) and the REBOOT_ODRIVES
    AXIS_ALL broadcast, the one refused method with a loop that bypasses the pre-check."""
    text = _RPC_CPP.read_text(encoding='utf-8')
    calls = re.findall(r'fault_mpc_active\(\) && refused_while_armed\(', text)
    assert len(calls) == 2
    reboot = re.search(r'case RpcMethod::REBOOT_ODRIVES: \{(.*?)\n    case ', text, re.S).group(1)
    all_branch = reboot.split('if (a.axis == AXIS_ALL) {', 1)[1].split('for (uint8_t i', 1)[0]
    assert 'fault_mpc_active() && refused_while_armed(method, AXIS_ALL, 0)' in all_branch
    pre = re.search(r'static uint16_t axis_precheck\(.*?\n\}\n', text, re.S).group(0)
    assert 'fault_mpc_active() && refused_while_armed(method, axis, requested_state)' in pre
    # The ungated frames and every Stow-gated handler run the pre-check; the Armed
    # gate precedes the Stow gate (a Leg IDLE passes the one, is refused by the other).
    send = re.search(r'static uint16_t send_axis_frame\(.*?\n\}\n', text, re.S).group(0)
    assert 'axis_precheck(method, axis, requested_state)' in send
    # SET_AXIS_STATE hands its requested state to the gate.
    assert 'axis_precheck(method, a.axis, a.state)' in text


def test_a_leg_idle_passes_the_armed_gate_and_the_stow_gate_decides():
    """FW 53: SET_AXIS_STATE IDLE is still NOT refused by the Armed predicate (the twin
    and the firmware agree), but it is a Stow-gated request, so while Armed (posed) the
    bridge refuses it at the Stow gate. The Hand IDLE is neither."""
    idle = rpc_args.refused_while_armed(RpcMethod.SET_AXIS_STATE, 0, 1)
    assert idle is False
    assert rpc_args.requires_platform_at_stow(RpcMethod.SET_AXIS_STATE, 0, 1) is True
    assert rpc_args.requires_platform_at_stow(RpcMethod.SET_AXIS_STATE, 6, 1) is False
