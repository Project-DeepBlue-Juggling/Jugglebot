"""Host side of the can-bridge FW 47 Hand stall guard (BUG-123).

The firmware IDLEs the Hand and latches FaultState HAND_STALL; the host must
(1) decode and name it, (2) put the reason on every refused command so the
Attempt ends with a line that says WHICH guard, and (3) never clear the latch
on its own: the hand-jam recovery's automatic converge-first clear would
release it and its raise would re-energise the Hand with no operator in the loop.
"""
from __future__ import annotations

import types

from teensy_link import FaultState
from teensy_link import activate as activate_mod

from jugglebot.motion import hand_jam as hj
from jugglebot import teensy_bridge_node as tbn
from jugglebot import trajectory_node as tn


def test_hand_stall_decodes_by_name():
    assert int(FaultState.HAND_STALL) == 8
    assert tbn._enum_name(FaultState, int(FaultState.HAND_STALL)) == 'HAND_STALL'


def test_guard_refusal_names_the_firmware_reason():
    fake = types.SimpleNamespace(_guard_fault='HAND_STALL')
    msg = tn.TrajectoryNode._guard_frozen_msg(fake)
    assert msg.startswith('Teensy guard latched (E-STOP) [fault_state=HAND_STALL]')
    # A healthy or unknown word adds nothing; an object built without the
    # callback (no attribute) keeps the old text.
    assert '[fault_state' not in tn.TrajectoryNode._guard_frozen_msg(
        types.SimpleNamespace(_guard_fault='NONE'))
    assert '[fault_state' not in tn.TrajectoryNode._guard_frozen_msg(
        types.SimpleNamespace())


def _jam_self(fault_state: int, resume: bool):
    calls = []

    def _svc_recover(req, res, park_hand=True):   # noqa: ARG001
        calls.append(park_hand)
        res.success = True
        res.message = 'cleared'
        return res

    fake = types.SimpleNamespace(
        _latest_heartbeat=types.SimpleNamespace(fault_state=fault_state),
        _hand_jam_rec=types.SimpleNamespace(resume=resume),
        _svc_recover=_svc_recover,
    )
    return fake, calls


def test_hand_jam_never_clears_a_hand_stall_on_its_own():
    fake, calls = _jam_self(int(FaultState.HAND_STALL), resume=False)
    r = tbn.TeensyBridgeNode._hand_jam_execute(
        fake, hj.Intent(hj.IntentKind.CONVERGE_CLEAR), None)
    assert r.ok is False
    assert 'HAND_STALL' in r.msg
    assert calls == []                      # no clear went out


def test_operator_resume_and_other_latches_still_clear():
    fake, calls = _jam_self(int(FaultState.HAND_STALL), resume=True)
    r = tbn.TeensyBridgeNode._hand_jam_execute(
        fake, hj.Intent(hj.IntentKind.CONVERGE_CLEAR), None)
    assert r.ok is True and calls == [False]
    fake, calls = _jam_self(int(FaultState.MAX_DEVIATION), resume=False)
    r = tbn.TeensyBridgeNode._hand_jam_execute(
        fake, hj.Intent(hj.IntentKind.CONVERGE_CLEAR), None)
    assert r.ok is True and calls == [False]


def test_reason_line_tolerates_missing_diagnostic():
    line = activate_mod.hand_lane_stall_line(float('nan'), float('nan'))
    assert line.startswith('HAND_STALL:') and 'nan' in line
