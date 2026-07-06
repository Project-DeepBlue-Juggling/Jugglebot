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

from controller.teensy_link.protocol import HeartbeatT2J, BusHealth
from jugglebot.teensy_bridge_node import _T2J_FLAG_STOW_PENDING

from tests.ros.test_teensy_bridge_node_read import _build_paired_node


def _hb(bus1_health=int(BusHealth.OK), flags=0):
    """A minimal T2J heartbeat carrying just the fields the stow guard reads."""
    return HeartbeatT2J(bus1_health=int(bus1_health), flags=int(flags))


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
