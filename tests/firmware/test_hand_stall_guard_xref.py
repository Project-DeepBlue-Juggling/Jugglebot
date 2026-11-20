"""The can-bridge FW 36 hand stall guard (BUG-56): cross-language pins.

The guard lives in ``leg_activate.cpp`` (behaviour pinned by the native
``test_leg_activate`` binary); its thresholds live in ``canbridge_config.h``.
What no native test can see is the contract with the HOST side:

* the hand-jam recovery deliberately stalls a HAND_MOVE_TO on a ball at a
  RELIEF current (``hand_jam.RELIEF_CURR_MAX_A``) and answers that stall itself
  (anchor, raise, never IDLE a raised hand onto the ball). The guard must never
  act on such a move, which holds only while ``HAND_STALL_IQ_A`` sits above the
  whole relief range (the ODrive clamps iq at the caller's limit). A future
  raise of the relief ceiling past the guard would silently hand the recovery's
  stalls to the firmware's IDLE;
* the Diagnostic flags bit the firmware sets is the bit both host decoders read;
* the hold window is the owner's 300 ms.
"""
from __future__ import annotations

import os
import re

from jugglebot.motion import hand_jam
from teensy_link import activate as activate_mod

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
_FW = os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot', 'Teensy_code_canbridge')
_NODE = os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot', 'jugglebot', 'teensy_bridge_node.py')


def _read(path: str) -> str:
    with open(path, encoding='utf-8') as f:
        return f.read()


def _cfg_float(name: str) -> float:
    m = re.search(rf'^constexpr\s+\w+\s+{name}\s*=\s*([0-9.]+)(?:f|ull)?\s*;',
                  _read(os.path.join(_FW, 'canbridge_config.h')), re.M)
    assert m is not None, f'{name} missing from canbridge_config.h'
    return float(m.group(1))


#: Measured iq overshoot past a soft current limit (53.3 A against 50 A,
#: 2026-11-01, BUG-51): a relief stall can read this much above its own cap.
_IQ_OVERSHOOT_A = 3.3


def test_stall_current_sits_above_the_hand_jam_relief_range():
    assert (_cfg_float('HAND_STALL_IQ_A')
            >= hand_jam.RELIEF_CURR_MAX_A + _IQ_OVERSHOOT_A)


def test_hold_window_is_the_owners_300_ms():
    assert _cfg_float('HAND_STALL_HOLD_US') == 300000.0


def test_stall_flag_bit_matches_both_host_decoders():
    m = re.search(r'DIAG_FLAG_ACTIVATE_STALL\s*=\s*(0x[0-9A-Fa-f]+)u?;',
                  _read(os.path.join(_FW, 'leg_activate.h')))
    assert m is not None
    fw = int(m.group(1), 16)
    assert fw == activate_mod.DIAG_FLAG_ACTIVATE_STALL
    n = re.search(r'^_DIAG_FLAG_ACTIVATE_STALL\s*=\s*(0x[0-9A-Fa-f]+)', _read(_NODE), re.M)
    assert n is not None
    assert int(n.group(1), 16) == fw
    # It must not collide with the two older bits (heartbeat_stale / _seen).
    assert fw & 0x3 == 0


def test_the_diagnostic_carries_the_flag():
    telem = _read(os.path.join(_FW, 'telemetry.cpp'))
    assert re.search(r'activate_stalled\(axis\)\s*\?\s*DIAG_FLAG_ACTIVATE_STALL', telem)


# ── FW 47 (BUG-123): the Hand stall guard on the streamed lane and latched holds ──
# Owner decision 2026-11-20 (verbatim): "Let's go with 40 A / 1 rev/s / 150 ms,
# after which the hand motor should enter IDLE." Behaviour is pinned by the
# native test_leg_activate / test_fault_machine binaries; these pin the numbers,
# the wire names and the host mirrors.

def test_lane_guard_numbers_are_the_owners_decision():
    assert _cfg_float('HAND_LANE_STALL_IQ_A') == 40.0
    assert _cfg_float('HAND_LANE_STALL_VEL_RPS') == 1.0
    assert _cfg_float('HAND_LANE_STALL_HOLD_US') == 150000.0
    # The host's reason line quotes the same numbers.
    assert activate_mod.HAND_LANE_STALL_IQ_A == _cfg_float('HAND_LANE_STALL_IQ_A')
    assert activate_mod.HAND_LANE_STALL_VEL_RPS == _cfg_float('HAND_LANE_STALL_VEL_RPS')
    assert activate_mod.HAND_LANE_STALL_HOLD_S * 1e6 == _cfg_float('HAND_LANE_STALL_HOLD_US')


def test_lane_guard_leaves_the_ladder_and_the_relief_alone():
    # The ladder keeps its own tighter guard, unchanged (25 A / 0.5 rev/s / 300 ms).
    assert _cfg_float('HAND_STALL_IQ_A') == 25.0
    assert _cfg_float('HAND_STALL_VEL_RPS') == 0.5
    # The hand-jam relief stalls (<= RELIEF_CURR_MAX_A plus the measured overshoot)
    # can never read as a lane stall either.
    assert (_cfg_float('HAND_LANE_STALL_IQ_A')
            >= hand_jam.RELIEF_CURR_MAX_A + _IQ_OVERSHOOT_A)
    # The sample-hole bound restarts the window well inside it.
    assert _cfg_float('HAND_LANE_STALL_GAP_US') < _cfg_float('HAND_LANE_STALL_HOLD_US')


def test_lane_stall_flag_bit_matches_both_host_decoders():
    m = re.search(r'DIAG_FLAG_LANE_STALL\s*=\s*(0x[0-9A-Fa-f]+)u?;',
                  _read(os.path.join(_FW, 'leg_activate.h')))
    assert m is not None
    fw = int(m.group(1), 16)
    assert fw == activate_mod.DIAG_FLAG_LANE_STALL
    n = re.search(r'^_DIAG_FLAG_LANE_STALL\s*=\s*(0x[0-9A-Fa-f]+)', _read(_NODE), re.M)
    assert n is not None
    assert int(n.group(1), 16) == fw
    assert fw & 0x7 == 0          # clear of heartbeat_stale / _seen / activate_stall
    telem = _read(os.path.join(_FW, 'telemetry.cpp'))
    assert re.search(r'hand_lane_stalled\(axis\)\s*\?\s*DIAG_FLAG_LANE_STALL', telem)


def test_hand_stall_is_a_named_fault_state_on_both_sides():
    from teensy_link import FaultState
    hdr = _read(os.path.join(_FW, 'udp_protocol.h'))
    m = re.search(r'constexpr uint8_t HAND_STALL = (\d+)u;', hdr)
    assert m is not None
    assert int(FaultState.HAND_STALL) == int(m.group(1))
    # fault_machine.cpp reports it and it is released by the operator's clear.
    fm = _read(os.path.join(_FW, 'fault_machine.cpp'))
    assert 'FaultState::HAND_STALL' in fm
    clear = fm[fm.index('void fault_notify_clear_errors()'):]
    clear = clear[:clear.index('\n}\n')]
    assert 's_hand_stall_latched = false' in clear


def test_the_reason_line_names_the_guard_the_action_and_the_recovery():
    line = activate_mod.hand_lane_stall_line(-68.4, 49.6)
    assert line.startswith('HAND_STALL:')
    for needle in ('IDLE', '40 A', '1 rev/s', '150 ms', '-68.4 A', '50 C',
                   'Legs are held', '/clear_errors', 're-home'):
        assert needle in line, (needle, line)
    assert '\n' not in line
