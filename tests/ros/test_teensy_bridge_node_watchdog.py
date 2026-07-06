"""Bridge-level link-loss watchdog + deferred-stow tests.

Exercises TeensyBridgeNode._health_check end-to-end through the real
TeensyLinkClient. Link staleness is simulated *deterministically* by writing
``client.stats.last_t2j_heartbeat_us`` (no real-time sleeping → no flakiness):
the FakeTeensy never auto-sends T→J heartbeats, so that field only changes when
the test sets it.

The fault-decision-tree fidelity (FaultEvaluator / DeferredStowLatch vs the
firmware spec) is covered in tests/teensy_link/test_fault_logic_mirror.py; this
file covers the *wiring* of the link latch into the node and its surfacing on
link_status.
"""

from __future__ import annotations

import time

from diagnostic_msgs.msg import DiagnosticStatus

# Reuse the loopback pairing + helper from the read-side test module.
from tests.ros.test_teensy_bridge_node_read import _build_paired_node


def _now_us() -> int:
    # Monotonic clock: this stamps client.stats.last_t2j_heartbeat_us, which the
    # production time_since_last_t2j_heartbeat_us() now reads against time.monotonic().
    # Both endpoints must share the same clock or the simulated age is nonsense.
    return int(time.monotonic() * 1_000_000)


def _set_heartbeat_age(client, age_s: float) -> None:
    """Make the client believe the last T→J heartbeat arrived ``age_s`` ago."""
    client.stats.last_t2j_heartbeat_us = _now_us() - int(age_s * 1_000_000)


def _make_node():
    teensy, client, node = _build_paired_node()
    node._heartbeat_timeout_s = 0.5  # short, deterministic threshold for tests
    return teensy, client, node


def _teardown(teensy, client, node):
    node.on_shutdown()
    client.stop()
    teensy.stop()


def test_no_arm_before_first_heartbeat():
    """Never arm the deferred stow before the link is first established."""
    teensy, client, node = _make_node()
    try:
        # No T→J heartbeat ever seen → last_t2j_heartbeat_us == 0 → age None.
        client.stats.last_t2j_heartbeat_us = 0
        node._health_check()
        assert node._link_latch.link_lost is False
        assert node._link_latch.stow_pending is False
    finally:
        _teardown(teensy, client, node)


def test_link_loss_arms_latch_and_blocks_commands():
    teensy, client, node = _make_node()
    try:
        # Establish the link (recent heartbeat), then go stale.
        _set_heartbeat_age(client, 0.0)
        node._health_check()
        assert node._link_latch.link_lost is False

        _set_heartbeat_age(client, 5.0)   # 5 s > 0.5 s timeout
        node._health_check()
        assert node._link_latch.link_lost is True
        assert node._link_latch.stow_pending is True
        # Never command into a dead link.
        assert node._link_latch.command_allowed() is False
    finally:
        _teardown(teensy, client, node)


def test_reconnect_keeps_stow_pending_and_surfaces_it():
    teensy, client, node = _make_node()
    try:
        _set_heartbeat_age(client, 0.0)
        node._health_check()
        _set_heartbeat_age(client, 5.0)
        node._health_check()              # link lost → armed
        assert node._link_latch.stow_pending is True

        _set_heartbeat_age(client, 0.0)   # reconnect
        node._health_check()
        assert node._link_latch.link_lost is False
        assert node._link_latch.stow_pending is True   # STILL pending (surfaced)

        # link_status must surface the pending stow as an ERROR.
        node._publish_link_status()
        msg = node.link_status_pub.published[-1]
        kv = {v.key: v.value for v in msg.values}
        assert kv['bridge_stow_pending'] == '1'
        assert kv['bridge_link_lost'] == '0'
        assert kv['mpc_active'] == '0'
        assert msg.level == DiagnosticStatus.ERROR
    finally:
        _teardown(teensy, client, node)


def test_redrop_rearms_latch():
    teensy, client, node = _make_node()
    try:
        _set_heartbeat_age(client, 0.0)
        node._health_check()
        _set_heartbeat_age(client, 5.0)
        node._health_check()              # loss
        _set_heartbeat_age(client, 0.0)
        node._health_check()              # reconnect (pending)
        node._link_latch.acknowledge_stow()
        assert node._link_latch.stow_pending is False

        _set_heartbeat_age(client, 5.0)
        node._health_check()              # re-drop
        assert node._link_latch.link_lost is True
        assert node._link_latch.stow_pending is True
        assert node._link_latch.command_allowed() is False
    finally:
        _teardown(teensy, client, node)


def test_link_lost_does_not_command_teensy():
    """While the link is down, the bridge sends no RPC/setpoint frames.

    The bridge has no setpoint output (Commit 1/2) and the watchdog must not
    emit any new commands on link loss — only heartbeats (which are liveness,
    not commands) continue. We assert no RPC_REQUEST frame is ever sent.
    """
    from controller.teensy_link import MsgType
    teensy, client, node = _make_node()
    try:
        _set_heartbeat_age(client, 0.0)
        node._health_check()
        _set_heartbeat_age(client, 5.0)
        node._health_check()
        node._health_check()  # repeated ticks while down
        time.sleep(0.05)
        # The bridge issued zero RPC requests (no commanding a dead link).
        assert teensy.received(int(MsgType.RPC_REQUEST)) == []
    finally:
        _teardown(teensy, client, node)
