"""can-bridge FW 53 (I-FW-19): the host twin of the Stow gate's RPC classifiers.

``teensy_link.rpc_args.requires_platform_at_stow`` / ``home_requires_all_legs_idle``
must agree with the firmware classifiers of the same name in ``rpc.cpp``; the gate's
thresholds must be the shared named constants (never literals); the two Diagnostic flag
bits must agree across the firmware header, the twin and ``telemetry.cpp``; and the RPC
layer must ask ``leg_idle_permitted`` (bench or Stow), never ``platform_at_stow`` alone.
A firmware edit that adds or drops a gated method without the twin (or the reverse)
fails here.
"""
from __future__ import annotations

import pathlib
import re

from teensy_link import activate, rpc_args
from teensy_link.protocol import AXIS_ALL, RpcMethod

_FW = (pathlib.Path(__file__).resolve().parents[2]
       / 'ros_ws/src/jugglebot/Teensy_code_canbridge')
_REPO = pathlib.Path(__file__).resolve().parents[2]


def _read(name):
    return (_FW / name).read_text(encoding='utf-8')


def _code(src):
    """Strip /* */ and // comments (keeps line structure irrelevant here)."""
    src = re.sub(r'/\*.*?\*/', '', src, flags=re.S)
    return '\n'.join(line.split('//', 1)[0] for line in src.split('\n'))


def _classifier_body(text, name):
    m = re.search(r'static bool %s\([^)]*\) \{(.*?)\n\}\n' % name, text, re.S)
    assert m, f'{name} not found in rpc.cpp'
    return m.group(1)


_LEGS = (0, 1, 5)
_STATES = (0, 1, 3, 6, 8, 9, None)


def test_the_firmware_gated_method_set_matches_the_twin():
    body = _classifier_body(_read('rpc.cpp'), 'requires_platform_at_stow')
    fw = {getattr(RpcMethod, n) for n in re.findall(r'case RpcMethod::(\w+):', body)}
    twin = {m for m in RpcMethod
            if any(rpc_args.requires_platform_at_stow(m, ax, st)
                   for ax in (0, 5, 6, AXIS_ALL) for st in _STATES)}
    assert fw == twin == {RpcMethod.SET_AXIS_STATE, RpcMethod.REBOOT_ODRIVES, RpcMethod.SDO_WRITE}
    # The per-method conditions, row by row (the twin's table below pins the same rows).
    assert 'return axis < NUM_LEGS && requested_state != ODriveState::CLOSED_LOOP;' in body
    assert 'return axis < NUM_LEGS || axis == RpcArgs::AXIS_ALL;' in body
    assert 'case RpcMethod::SDO_WRITE:\n      return axis < NUM_LEGS;' in body
    assert 'default:\n      return false;' in body


def test_the_twin_rows():
    f = rpc_args.requires_platform_at_stow
    for leg in _LEGS:
        assert f(RpcMethod.SET_AXIS_STATE, leg, 1)          # IDLE
        assert f(RpcMethod.SET_AXIS_STATE, leg, 6)          # ENCODER_INDEX_SEARCH
        assert f(RpcMethod.SET_AXIS_STATE, leg)             # unknown state: could be IDLE
        assert not f(RpcMethod.SET_AXIS_STATE, leg, 8)      # CLOSED_LOOP engages
        assert f(RpcMethod.REBOOT_ODRIVES, leg)
        assert f(RpcMethod.SDO_WRITE, leg)
        for m in (RpcMethod.SDO_READ, RpcMethod.CLEAR_ERRORS, RpcMethod.SET_VEL_CURR_LIMITS,
                  RpcMethod.SET_POS_GAIN, RpcMethod.HOME):
            assert not f(m, leg, 1)
    assert not f(RpcMethod.SET_AXIS_STATE, 6, 1)             # the Hand is exempt
    assert not f(RpcMethod.REBOOT_ODRIVES, 6)
    assert f(RpcMethod.REBOOT_ODRIVES, AXIS_ALL)
    assert not f(RpcMethod.SET_AXIS_STATE, AXIS_ALL, 1)
    assert rpc_args.home_requires_all_legs_idle(0) and rpc_args.home_requires_all_legs_idle(5)
    assert not rpc_args.home_requires_all_legs_idle(6)


def test_the_home_classifier_matches():
    text = _read('rpc.cpp')
    m = re.search(r'static bool home_requires_all_legs_idle\(uint8_t axis\) \{ return (.*?); \}', text)
    assert m and m.group(1) == 'axis < NUM_LEGS'


def test_the_constants_the_twin_hardcodes_are_the_firmware_ones():
    odrive = _read('protocol_config.h')
    assert re.search(r'constexpr uint32_t CLOSED_LOOP = (\d+)u;', odrive).group(1) == \
        str(rpc_args._ODRIVE_STATE_CLOSED_LOOP) == str(activate.AXIS_STATE_CLOSED_LOOP)
    assert re.search(r'constexpr uint8_t\s+NUM_LEGS\s*=\s*JbUdp::NUM_LEGS;', _read('canbridge_config.h'))
    assert re.search(r'constexpr uint8_t NUM_LEGS = (\d+)u;', _read('udp_protocol.h')).group(1) == \
        str(rpc_args.NUM_LEGS_FOR_ARMED_GATE)


def test_the_rpc_layer_asks_leg_idle_permitted_never_platform_at_stow_alone():
    """The one-Leg bench is exempt: every pre-check in rpc.cpp must be leg_idle_permitted
    (bench or Stow). A bare platform_at_stow() here would refuse the bench Leg's IDLE."""
    code = _code(_read('rpc.cpp'))
    assert not re.search(r'(?<![A-Za-z_])platform_at_stow\s*\(', code)
    assert 'leg_idle_permitted(micros64())' in code
    # The AXIS_ALL reboot asks the classifier and the gate BEFORE any frame or the latch.
    reboot = re.search(r'case RpcMethod::REBOOT_ODRIVES: \{(.*?)\n    case ', code, re.S).group(1)
    all_branch = reboot.split('if (a.axis == AXIS_ALL) {', 1)[1]
    gate_at = all_branch.index('requires_platform_at_stow(method, AXIS_ALL, 0)')
    assert gate_at < all_branch.index('odrive_reboot_tx(')
    assert gate_at < all_branch.index('fault_notify_reboot_started()')


def test_the_status_mapping():
    text = _code(_read('rpc.cpp'))
    m = re.search(r'static uint16_t gate_status\(.*?\n\}\n', text, re.S).group(0)
    assert re.search(r'DeResult::SENT:\s*return JbUdp::RpcStatus::OK;', m)
    assert re.search(r'DeResult::REFUSED:\s*return JbUdp::RpcStatus::ERR_REJECTED;', m)
    assert re.search(r'default:\s*return JbUdp::RpcStatus::ERR_TIMEOUT;', m)
    # No new status code: an FW 52 host's RpcStatus(status).name must not raise in skew.


def test_the_gate_uses_the_shared_named_thresholds():
    gate = _code(_read('leg_stow_gate.cpp'))
    for name in ('JBOp::TARGET_REACHED_POS_TOL_REV', 'JBOp::TARGET_REACHED_VEL_TOL_RPS',
                 'CAN_HEARTBEAT_STALE_US', 'MOTOR_FB_STALENESS_US', 'STOW_WINDOW_MIN_REV',
                 'STOW_OFF_POSE_REV', 'CL_ENGAGE_SETTLE_US'):
        assert name in gate, name


def test_the_diagnostic_flag_bits_agree():
    h = _read('leg_stow_gate.h')
    held = int(re.search(r'DIAG_FLAG_LEG_HELD\s*=\s*(0x[0-9A-Fa-f]+)u;', h).group(1), 16)
    bench = int(re.search(r'DIAG_FLAG_RIG_BENCH\s*=\s*(0x[0-9A-Fa-f]+)u;', h).group(1), 16)
    assert (held, bench) == (activate.DIAG_FLAG_LEG_HELD, activate.DIAG_FLAG_RIG_BENCH) == (0x10, 0x20)
    used = {0x1, 0x2, activate.DIAG_FLAG_ACTIVATE_STALL, activate.DIAG_FLAG_LANE_STALL}
    assert not ({held, bench} & used) and held != bench and max(held, bench) < 0x100
    telem = _code(_read('telemetry.cpp'))
    assert 'leg_held_mask()' in telem and 'rig_is_bench(now)' in telem
    assert 'fw53_gate_flags(axis, now)' in telem        # in the Diagnostic's flags byte
    # The wire is unchanged: the bits ride the existing flags byte.
    assert re.search(r'^PROTOCOL_VERSION = 14\b',
                     (_REPO / 'config/generated/udp_protocol.py').read_text(encoding='utf-8'), re.M)


def test_the_host_masks_unknown_diagnostic_bits():
    """The host reads Diagnostic flags with masks only, so the FW 53 bits cannot
    be misread by a host that does not name them (no equality compare on the byte)."""
    for path in ('teensy_link/activate.py', 'ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py'):
        text = (_REPO / path).read_text(encoding='utf-8')
        assert not re.search(r'\.flags\)?\s*(==|!=)', text), path
