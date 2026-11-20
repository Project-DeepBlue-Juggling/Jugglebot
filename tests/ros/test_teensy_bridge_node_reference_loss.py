"""BUG-122: the bridge clears is_homed when a drive loses its position reference.

2026-11-20: the Hand drive raised MISSING_ESTIMATE at rest and came back 5.00
turns low, and later the E-stop button power-cycled every drive (INITIALIZING);
both times is_homed stayed true, so the FAULT exit went BOOT -> IDLE and the
next Catch drove the Hand into its top stop. The pure edge/latch logic lives in
``jugglebot/reference_loss.py``; the node feeds it every Diagnostic and lands
the clear through the relay STATE_WRITE (read-modify-write, levelling + pose
preserved), retrying from the 1 Hz health check until it lands.

Error bits come from ``jugglebot.can.odrive``, never literals.
"""

from __future__ import annotations

import time

from teensy_link import RpcMethod, RpcStatus, rpc_args
from teensy_link.protocol import Diagnostic

from jugglebot.can import odrive
from jugglebot.reference_loss import ReferenceLossWatch

from tests.ros._bridge_harness import _build_paired_node, _teardown, _wait_until

HAND = 6
LEG_2 = 2
BB_HAND = 8
UV = odrive.ERR_DC_BUS_UNDER_VOLTAGE


# ── The pure watch ─────────────────────────────────────────────

def test_watch_fires_once_per_rising_edge():
    w = ReferenceLossWatch()
    ev = w.observe(HAND, 0, odrive.ERR_MISSING_ESTIMATE)
    assert ev is not None and ev.axis == HAND
    assert 'MISSING_ESTIMATE (disarm)' in ev.reason
    # Sticky disarm_reason: the same frame again is not a new event.
    for _ in range(5):
        assert w.observe(HAND, 0, odrive.ERR_MISSING_ESTIMATE) is None
    # CLEAR_ERRORS wipes it; a second loss is a second event.
    assert w.observe(HAND, 0, 0) is None
    ev2 = w.observe(HAND, odrive.ERR_MISSING_ESTIMATE, 0)
    assert ev2 is not None and ev2.seq == ev.seq + 1


def test_watch_reboot_signature_and_ignored_errors():
    w = ReferenceLossWatch()
    ev = w.observe(LEG_2, odrive.ERR_INITIALIZING | UV, 0)
    assert ev is not None and 'INITIALIZING' in ev.reason
    # SPINOUT and under-voltage alone keep the reference: no event.
    assert w.observe(HAND, 0, odrive.ERR_SPINOUT_DETECTED) is None
    assert w.observe(0, UV, UV) is None
    # Ball Butler drives are not in the homed set.
    assert w.observe(BB_HAND, odrive.ERR_INITIALIZING, 0) is None


def test_watch_home_answers_only_earlier_events():
    w = ReferenceLossWatch()
    w.arm(w.observe(HAND, 0, odrive.ERR_MISSING_ESTIMATE))
    start = w.seq
    w.home_written(start)
    assert w.pending is None
    w.arm(w.observe(LEG_2, odrive.ERR_INITIALIZING, 0))   # during the next Homing
    start2 = w.seq - 1
    w.home_written(start2)
    assert w.pending is not None


# ── The node ───────────────────────────────────────────────────

def _diag(node, axis, active=0, disarm=0):
    dg = Diagnostic(axis_id=axis, axis_state=1, active_errors=active,
                    disarm_reason=disarm)
    node._on_diagnostic(0, 0, dg.pack(), None)


def _homed(node, homed=True):
    from jugglebot.teensy_bridge_node import RelayRobotState
    with node._lock:
        node._cold_start_state = RelayRobotState(
            is_homed=homed, levelling_complete=True,
            pose_offset_tiltX=0.03, pose_offset_tiltY=-0.04)
        node._cold_start_authoritative = True


def _record_writes(teensy, status=RpcStatus.OK):
    writes = []

    def handler(req_id, args):
        writes.append(args)
        return (int(status), b"")
    teensy.on_rpc(int(RpcMethod.STATE_WRITE), handler)
    return writes


def _errors(node):
    errors = []
    node.get_logger().error = lambda m, *a, **k: errors.append(str(m))
    return errors


def _wait_idle(node):
    assert _wait_until(lambda: not node._ref_loss_inflight, timeout=3.0)


def test_hand_missing_estimate_clears_once():
    teensy, client, node = _build_paired_node()
    try:
        _homed(node)
        writes = _record_writes(teensy)
        errors = _errors(node)
        for _ in range(4):   # sticky disarm, several frames
            _diag(node, HAND, disarm=odrive.ERR_MISSING_ESTIMATE)
        assert node._cold_start_state.is_homed is False   # local view at once
        assert _wait_until(lambda: len(writes) == 1, timeout=3.0)
        _wait_idle(node)
        node._kick_reference_loss_clear()                  # a later 1 Hz tick
        time.sleep(0.1)
        assert len(writes) == 1
        # is_homed False, levelling + pose preserved (read-modify-write).
        assert writes[0] == rpc_args.encode_state_write(False, True, 0.03, -0.04)
        assert node._ref_loss.pending is None
        cleared = [e for e in errors if e.startswith('homed flag cleared')]
        assert len(cleared) == 1
        assert 'hand' in cleared[0] and 'MISSING_ESTIMATE' in cleared[0]
        assert 're-home required' in cleared[0]
    finally:
        _teardown(teensy, client, node)


def test_leg_missing_estimate_clears():
    teensy, client, node = _build_paired_node()
    try:
        _homed(node)
        writes = _record_writes(teensy)
        errors = _errors(node)
        _diag(node, LEG_2, active=odrive.ERR_MISSING_ESTIMATE)
        assert _wait_until(lambda: len(writes) == 1, timeout=3.0)
        assert node._cold_start_state.is_homed is False
        assert any('leg 2' in e for e in errors if e.startswith('homed flag cleared'))
    finally:
        _teardown(teensy, client, node)


def test_drive_reboot_clears():
    """The 2026-11-20 13:34 E-stop button signature on every axis."""
    teensy, client, node = _build_paired_node()
    try:
        _homed(node)
        writes = _record_writes(teensy)
        errors = _errors(node)
        for axis in range(HAND + 1):
            _diag(node, axis, active=odrive.ERR_INITIALIZING | UV)
        assert _wait_until(lambda: len(writes) >= 1, timeout=3.0)
        _wait_idle(node)
        assert node._cold_start_state.is_homed is False
        assert node._ref_loss.pending is None
        # Only the first axis finds the robot homed; the rest find it cleared.
        assert len([e for e in errors if e.startswith('homed flag cleared')]) == 1
    finally:
        _teardown(teensy, client, node)


def test_spinout_does_not_clear():
    teensy, client, node = _build_paired_node()
    try:
        _homed(node)
        writes = _record_writes(teensy)
        _diag(node, HAND, disarm=odrive.ERR_SPINOUT_DETECTED)
        _diag(node, LEG_2, active=UV, disarm=UV)
        node._kick_reference_loss_clear()
        time.sleep(0.2)
        assert writes == []
        assert node._cold_start_state.is_homed is True
        assert node._ref_loss.pending is None
    finally:
        _teardown(teensy, client, node)


def test_startup_initializing_before_homing_writes_nothing():
    """A fresh power-on: the Platform Teensy reads not homed and every drive
    shows INITIALIZING. Nothing is cleared, and the next Homing is not undone."""
    teensy, client, node = _build_paired_node()
    try:
        _homed(node, homed=False)
        writes = _record_writes(teensy)
        errors = _errors(node)
        for axis in range(HAND + 1):
            _diag(node, axis, active=odrive.ERR_INITIALIZING | UV)
        node._kick_reference_loss_clear()
        time.sleep(0.2)
        assert writes == []
        assert node._ref_loss.pending is None
        assert not [e for e in errors if e.startswith('homed flag cleared')]
        # Homing then succeeds; no owed clear can undo it.
        assert node._write_is_homed(True)[0]
        node._kick_reference_loss_clear()
        time.sleep(0.2)
        assert node._cold_start_state.is_homed is True
    finally:
        _teardown(teensy, client, node)


def test_event_before_the_boot_read_is_applied_when_it_lands():
    """INITIALIZING before the cold-start read: nothing is written (it would
    clobber levelling + pose with defaults); a read that then says homed is
    published as not homed and the clear lands."""
    teensy, client, node = _build_paired_node()
    try:
        assert node._cold_start_authoritative is False
        writes = _record_writes(teensy)
        _diag(node, HAND, active=odrive.ERR_INITIALIZING)
        assert node._kick_reference_loss_clear() is False
        assert writes == []
        from jugglebot.teensy_bridge_node import RelayRobotState
        node.relay_read_robot_state = lambda: (
            True, 'ok', RelayRobotState(True, True, 0.03, -0.04))
        assert node._refresh_cold_start_state('boot')
        assert node._cold_start_state.is_homed is False
        node._kick_reference_loss_clear()
        assert _wait_until(lambda: len(writes) == 1, timeout=3.0)
        assert writes[0] == rpc_args.encode_state_write(False, True, 0.03, -0.04)
    finally:
        _teardown(teensy, client, node)


def test_failed_write_is_retried_until_it_lands():
    teensy, client, node = _build_paired_node()
    try:
        _homed(node)
        replies = [RpcStatus.ERR_BUS_DOWN, RpcStatus.ERR_BUS_DOWN, RpcStatus.OK]
        writes = []

        def handler(req_id, args):
            writes.append(args)
            return (int(replies[min(len(writes), len(replies)) - 1]), b"")
        teensy.on_rpc(int(RpcMethod.STATE_WRITE), handler)
        _diag(node, HAND, disarm=odrive.ERR_MISSING_ESTIMATE)
        assert _wait_until(lambda: len(writes) == 1, timeout=3.0)
        _wait_idle(node)
        assert node._ref_loss.pending is not None          # still owed
        assert node._cold_start_state.is_homed is False    # safe local view meanwhile
        for _ in range(2):                                 # two 1 Hz ticks
            node._health_check()
            _wait_idle(node)
        assert len(writes) == 3
        assert node._ref_loss.pending is None
    finally:
        _teardown(teensy, client, node)


def test_homing_answers_an_earlier_loss():
    """A loss whose clear never landed is answered by a Homing that starts
    after it: the Homing's is_homed=True is not undone by a late retry."""
    teensy, client, node = _build_paired_node()
    try:
        _homed(node)
        _record_writes(teensy, status=RpcStatus.ERR_BUS_DOWN)
        _diag(node, HAND, disarm=odrive.ERR_MISSING_ESTIMATE)
        _wait_idle(node)
        assert node._ref_loss.pending is not None
        writes = _record_writes(teensy)                    # the bus is back
        node._run_home = lambda axes: (True, 'homed')
        node._run_configure = lambda axes, **kw: (True, 'configured')
        ok, msg = node._do_home()
        assert ok, msg
        assert node._ref_loss.pending is None
        node._kick_reference_loss_clear()
        time.sleep(0.2)
        assert writes == [rpc_args.encode_state_write(True, True, 0.03, -0.04)]
        assert node._cold_start_state.is_homed is True
    finally:
        _teardown(teensy, client, node)
