"""BUG-47 / BUG-141: shutdown ordering pin, the homing stop check, and main()'s guard.

The order of what on_shutdown() does on a healthy link is a safety property (disarm
first so mpc_active=0 reaches the wire before the DEACTIVATE; transport closed last)
and the BUG-47 change adds one step (setting the shutdown Event) that must not
reorder or add anything on the wire. Pinned here at the method level (mocks).
"""
from __future__ import annotations

import threading
from unittest.mock import MagicMock

from teensy_link.protocol import BusHealth, FaultState, HeartbeatT2J, RpcMethod, RpcStatus
from jugglebot import teensy_bridge_node as tbn
from tests.ros._bridge_harness import _build_paired_node, _teardown
from tests.ros import conftest  # noqa: F401  (mock rclpy, shared class identity)


def _hb():
    return HeartbeatT2J(t_teensy_us=0, link_state=1, bus1_health=int(BusHealth.OK),
                        bus2_health=int(BusHealth.OK), fault_state=int(FaultState.NONE),
                        flags=0, uptime_ms=0)


def test_on_shutdown_step_order_is_unchanged_on_a_healthy_link():
    """disarm -> hand fan AUTO -> stow -> imu close -> transport close (tod, rpc
    server, rpc) -> client.stop; the shutdown Event is set before the disarm and
    never reorders them."""
    teensy, client, node = _build_paired_node()
    try:
        node._stow_on_shutdown = True
        node._latest_heartbeat = _hb()
        calls = []
        node._stop_setpoint_output = MagicMock(side_effect=lambda: calls.append(
            ('disarm', node._shutdown_event.is_set())))
        node._shutdown_hand_fan_auto = MagicMock(side_effect=lambda: calls.append('fan'))
        node._wait_wire_disarmed = MagicMock(side_effect=lambda: calls.append('wait_disarmed'))
        node._shutdown_stow = MagicMock(side_effect=lambda: calls.append('stow'))
        node._tod = MagicMock(); node._tod.close.side_effect = lambda: calls.append('tod')
        node._rpc_server = MagicMock(); node._rpc_server.close.side_effect = lambda: calls.append('rpc_server')
        node._rpc = MagicMock(); node._rpc.close.side_effect = lambda: calls.append('rpc')
        node._own_client = True
        node._client = MagicMock(); node._client.stop.side_effect = lambda: calls.append('client_stop')
        node.on_shutdown()
        assert calls == [('disarm', True), 'fan', 'wait_disarmed', 'stow',
                         'tod', 'rpc_server', 'rpc', 'client_stop']
    finally:
        node._client = client
        _teardown(teensy, client, node)


def test_run_home_aborts_and_idles_the_axis_when_shutdown_is_requested():
    """BUG-47: the homing monitor loop returns once the shutdown Event is set and
    commands the axis to IDLE (SET_AXIS_STATE axis 0 state 1) when the HOME was
    accepted; the result is a failed homing, bounded by one poll."""
    teensy, client, node = _build_paired_node()
    try:
        sent = {'home': 0, 'state': []}
        teensy.on_rpc(int(RpcMethod.HOME),
                      lambda rid, a: (sent.__setitem__('home', sent['home'] + 1), (int(RpcStatus.OK), b''))[1])
        teensy.on_rpc(int(RpcMethod.SET_AXIS_STATE),
                      lambda rid, a: (sent['state'].append(a), (int(RpcStatus.OK), b''))[1])
        res = {}
        t = threading.Thread(target=lambda: res.update(r=node._run_home([0], poll_dt=0.02)))
        t.start()
        end = __import__('time').monotonic() + 3
        while sent['home'] < 1 and __import__('time').monotonic() < end:
            __import__('time').sleep(0.01)
        assert sent['home'] >= 1
        node._shutdown_event.set()
        t.join(timeout=3)
        assert not t.is_alive(), 'homing loop did not stop'
        ok, msg = res['r']
        assert not ok and 'aborted by shutdown' in msg
        assert len(sent['state']) == 1
        assert tbn.rpc_args.encode_set_axis_state(0, 1) == sent['state'][0]
    finally:
        _teardown(teensy, client, node)


def test_run_home_abort_sends_no_idle_when_home_was_never_accepted():
    teensy, client, node = _build_paired_node()
    try:
        node._shutdown_event.set()
        node.teensy_set_axis_state = MagicMock()
        ok, msg = node._run_home([0], poll_dt=0.02)
        assert not ok and 'aborted by shutdown' in msg
        node.teensy_set_axis_state.assert_not_called()
    finally:
        _teardown(teensy, client, node)


def test_home_abort_hook_is_a_single_named_function():
    """The owner-pending decision lives behind one function: forcing it False
    restores the old (run-to-result) behaviour with no other edit."""
    teensy, client, node = _build_paired_node()
    try:
        assert node._home_abort_requested() is False
        node._shutdown_event.set()
        assert node._home_abort_requested() is True
    finally:
        _teardown(teensy, client, node)


def test_main_constructs_inside_the_guard_and_shuts_down_after_boot_interrupt(monkeypatch):
    """BUG-141: an interrupt during the boot read leaves a built node flagged
    _boot_interrupted; main() must skip spin, run on_shutdown, destroy, shutdown."""
    node = MagicMock(); node._boot_interrupted = True
    monkeypatch.setattr(tbn, 'TeensyBridgeNode', lambda: node)
    spun = MagicMock()
    monkeypatch.setattr(tbn, 'MultiThreadedExecutor', spun)
    init, shut = MagicMock(), MagicMock()
    monkeypatch.setattr(tbn.rclpy, 'init', init, raising=False)
    monkeypatch.setattr(tbn.rclpy, 'shutdown', shut, raising=False)
    tbn.main([])
    spun.assert_not_called()
    node.on_shutdown.assert_called_once()
    node.destroy_node.assert_called_once()
    shut.assert_called_once()


def test_main_keyboard_interrupt_in_constructor_still_shuts_rclpy_down(monkeypatch):
    def boom():
        raise KeyboardInterrupt
    monkeypatch.setattr(tbn, 'TeensyBridgeNode', boom)
    shut = MagicMock()
    monkeypatch.setattr(tbn.rclpy, 'init', MagicMock(), raising=False)
    monkeypatch.setattr(tbn.rclpy, 'shutdown', shut, raising=False)
    tbn.main([])
    shut.assert_called_once()
