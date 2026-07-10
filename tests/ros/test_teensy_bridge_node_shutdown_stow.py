"""Shutdown-stow tests for ``teensy_bridge_node.on_shutdown()``.

Ctrl-C of ``jugglebot_launch`` must profiled-stow the platform before transport
teardown — the Teensy-side analogue of ``can_node.on_shutdown``'s
``_gently_move_to_setpoint(0.0, deactivating=True)`` (can_node.py:1693-1706). The
can_node→teensy_bridge migration dropped this; a clean Ctrl-C left the legs
holding (CAN3 healthy → firmware just enters LINK_LOST and gates output, and its
only autonomous stow is the CAN3-*loss* deferred stow, which does not apply).

These pin the ported behaviour + its guards (which mirror can_node's
``if fatal_can_error: skip / elif not stowed_due_to_error: stow`` structure):
  * healthy CAN3 + legs                → fire DEACTIVATE on the configured legs;
  * CAN3 bus BUS_OFF / no telemetry    → skip (can't drive a dead bus; the
                                          firmware deferred-stow covers reconnect);
  * a deferred stow already pending    → skip (the firmware completes it);
  * stow_on_shutdown=False (unit-test default) → never fire.

The DEACTIVATE descent itself (TRAP_TRAJ lower to STOW + IDLE) is covered by the
DeactivateMonitor tests; here we stub ``_run_deactivate`` and assert the guard
decisions so the suite never drives a real (hanging) descent against a FakeTeensy.
"""

from __future__ import annotations

from unittest.mock import MagicMock

from controller.teensy_link.protocol import HeartbeatT2J, BusHealth, FaultState
from jugglebot.teensy_bridge_node import (
    _T2J_FLAG_STOW_PENDING,
    _SHUTDOWN_DEACTIVATE_TIMEOUT_S,
)

from tests.ros.test_teensy_bridge_node_read import _build_paired_node


def _hb(bus1_health=int(BusHealth.OK), flags=0, fault_state=int(FaultState.NONE)):
    """A minimal T2J heartbeat carrying just the fields the stow guard reads."""
    return HeartbeatT2J(bus1_health=int(bus1_health), flags=int(flags),
                        fault_state=int(fault_state))


def _logged_errors(logger_mock):
    """All positional first-args passed to a MagicMock logger's .error()."""
    return [str(c.args[0]) if c.args else '' for c in logger_mock.error.call_args_list]


def _node_with_stow():
    """Paired node with the shutdown-stow re-enabled (the helper pins it False) and
    ``_run_deactivate`` stubbed so no real descent runs."""
    teensy, client, node = _build_paired_node()
    node._stow_on_shutdown = True
    node._run_deactivate = MagicMock(return_value=(True, "deactivated"))
    return teensy, client, node


def _teardown(teensy, client, node):
    node._stow_on_shutdown = False   # don't re-fire the stub during teardown
    node.on_shutdown()
    client.stop()
    teensy.stop()


def test_shutdown_stow_fires_deactivate_on_configured_legs_when_bus_healthy():
    teensy, client, node = _node_with_stow()
    try:
        node._latest_heartbeat = _hb()
        node._shutdown_stow()
        node._run_deactivate.assert_called_once()
        axes = list(node._run_deactivate.call_args[0][0])
        assert axes == [int(a) for a in node.get_parameter('deactivate_axes').value]
    finally:
        _teardown(teensy, client, node)


def test_shutdown_stow_skips_when_core_bus_off():
    teensy, client, node = _node_with_stow()
    try:
        node._latest_heartbeat = _hb(bus1_health=int(BusHealth.BUS_OFF))
        node._shutdown_stow()
        node._run_deactivate.assert_not_called()
    finally:
        _teardown(teensy, client, node)


def test_shutdown_stow_skips_when_no_telemetry():
    teensy, client, node = _node_with_stow()
    try:
        node._latest_heartbeat = None
        node._shutdown_stow()
        node._run_deactivate.assert_not_called()
    finally:
        _teardown(teensy, client, node)


def test_shutdown_stow_skips_when_deferred_stow_pending():
    teensy, client, node = _node_with_stow()
    try:
        node._latest_heartbeat = _hb(flags=_T2J_FLAG_STOW_PENDING)
        node._shutdown_stow()
        node._run_deactivate.assert_not_called()
    finally:
        _teardown(teensy, client, node)


def test_on_shutdown_honours_disabled_flag():
    # Production defaults stow_on_shutdown=True, but the unit-test helper pins it
    # False; confirm on_shutdown honours the flag and fires NO DEACTIVATE — the
    # guard that keeps the rest of the ros suite from hanging on teardown.
    teensy, client, node = _build_paired_node()   # stow_on_shutdown=False
    node._run_deactivate = MagicMock(return_value=(True, "deactivated"))
    node._latest_heartbeat = _hb()
    node.on_shutdown()
    node._run_deactivate.assert_not_called()
    client.stop()
    teensy.stop()


# ── Task 3.3: the ordered disarm → settle → stow shutdown sequence ──────────

def test_on_shutdown_disarms_before_stow():
    """on_shutdown must DISARM (stop setpoint output + mpc_active→0) BEFORE firing
    the profiled DEACTIVATE — the ordering that stops the firmware rejecting the
    stow while mpc_active=1 (and avoids the emitter-stop MPC_STALE latch)."""
    teensy, client, node = _node_with_stow()
    try:
        calls = []
        node._stop_setpoint_output = MagicMock(
            side_effect=lambda: calls.append('disarm'))
        node._run_deactivate = MagicMock(
            side_effect=lambda *a, **k: (calls.append('deactivate'), (True, "ok"))[1])
        node._latest_heartbeat = _hb()
        node.on_shutdown()
        assert calls == ['disarm', 'deactivate']
    finally:
        _teardown(teensy, client, node)


def test_on_shutdown_stows_even_if_disarm_raises():
    """Best-effort: a disarm error is logged loudly but the sequence CONTINUES to
    the profiled stow (never abandons the stow on a disarm hiccup)."""
    teensy, client, node = _node_with_stow()
    try:
        node._stop_setpoint_output = MagicMock(
            side_effect=RuntimeError("disarm boom"))
        node._latest_heartbeat = _hb()
        node.on_shutdown()   # must not raise
        node._run_deactivate.assert_called_once()
    finally:
        _teardown(teensy, client, node)


def test_shutdown_stow_uses_bounded_deactivate_budget():
    """The shutdown DEACTIVATE runs on the bounded budget (not the 20 s default) so
    a stalled descent can't blow the ~8 s teardown window."""
    teensy, client, node = _node_with_stow()
    try:
        node._latest_heartbeat = _hb()
        node._shutdown_stow()
        _, kwargs = node._run_deactivate.call_args
        assert kwargs.get('timeout_s') == _SHUTDOWN_DEACTIVATE_TIMEOUT_S
    finally:
        _teardown(teensy, client, node)


def test_shutdown_stow_guard_latched_warns_disarmed_but_standing():
    """If the profiled DEACTIVATE fails while a guard is latched (deactivate
    impossible), emit the loud 'disarmed but standing / CLEAR_ERRORS' final
    message so the operator knows the robot is not stowed."""
    teensy, client, node = _node_with_stow()
    try:
        node._run_deactivate = MagicMock(
            return_value=(False, "DEACTIVATE rejected: ERR_BUS_DOWN"))
        node._latest_heartbeat = _hb(fault_state=int(FaultState.MAX_DEVIATION))
        node._logger = MagicMock()
        node._shutdown_stow()
        errs = _logged_errors(node._logger)
        assert any('DISARMED BUT STILL STANDING' in e for e in errs), errs
        assert any('CLEAR_ERRORS' in e for e in errs), errs
    finally:
        _teardown(teensy, client, node)


def test_shutdown_stow_no_standing_warning_on_success():
    """A successful stow must NOT emit the disarmed-but-standing message."""
    teensy, client, node = _node_with_stow()
    try:
        node._run_deactivate = MagicMock(return_value=(True, "deactivated"))
        node._latest_heartbeat = _hb(fault_state=int(FaultState.MAX_DEVIATION))
        node._logger = MagicMock()
        node._shutdown_stow()
        assert not any('DISARMED BUT STILL STANDING' in e
                       for e in _logged_errors(node._logger))
    finally:
        _teardown(teensy, client, node)


def test_shutdown_stow_no_standing_warning_when_no_guard_latched():
    """A stow FAILURE with no guard latched (fault_state=NONE, e.g. a stalled leg)
    gets the generic error but NOT the guard-latched 'standing' message — that one
    is specific to the deactivate-impossible case."""
    teensy, client, node = _node_with_stow()
    try:
        node._run_deactivate = MagicMock(
            return_value=(False, "deactivate FAILED — axis 0: timed out"))
        node._latest_heartbeat = _hb(fault_state=int(FaultState.NONE))
        node._logger = MagicMock()
        node._shutdown_stow()
        errs = _logged_errors(node._logger)
        assert any('did not complete' in e for e in errs), errs
        assert not any('DISARMED BUT STILL STANDING' in e for e in errs)
    finally:
        _teardown(teensy, client, node)


def test_shutdown_stow_already_stowed_is_clean_noop():
    """When the legs are already at STOW, _run_deactivate short-circuits to success
    ('already at STOW'); the shutdown treats it as a clean stow — no standing
    warning, DEACTIVATE observed once."""
    teensy, client, node = _node_with_stow()
    try:
        node._run_deactivate = MagicMock(
            return_value=(True, "already at STOW (axes [0, 1, 2, 3, 4, 5])"))
        node._latest_heartbeat = _hb()
        node._logger = MagicMock()
        node._shutdown_stow()
        node._run_deactivate.assert_called_once()
        assert not any('DISARMED BUT STILL STANDING' in e
                       for e in _logged_errors(node._logger))
    finally:
        _teardown(teensy, client, node)
