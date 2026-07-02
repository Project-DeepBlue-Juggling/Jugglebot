"""Catching-cone tests for teensy_bridge_node (phase-10b cone uplink).

The can-bridge relays every cone CAN2 frame to the Jetson as a CONE_FRAME
(raw 8-byte CAN payload + arbitration id + bridge RX timestamp); the bridge
node decodes with jugglebot.can.catching_cone and republishes the production
cone/catch_event + cone/heartbeat topics formerly owned by can_node. These
tests drive the full UDP path on loopback (FakeTeensy → TeensyLinkClient RX
thread → node callbacks) and then invoke the publish timers directly (ROS is
mocked by tests/ros/conftest.py — timers don't auto-fire).

Coverage migrated from tests/ros/test_can_node.py (catch-event routing, cone
heartbeat routing + connected gate) plus the CONE_FRAME-specific cases
(unknown can_id, short CAN payload, truncated UDP payload, multi-event drain).
"""

from __future__ import annotations

import struct
import time

import pytest

from controller.teensy_link import protocol as p
from controller.teensy_link import MsgType

from jugglebot.can import catching_cone

from tests.ros.test_teensy_bridge_node_read import _build_paired_node, _wait_until


@pytest.fixture
def bridge():
    """Yield (teensy, node) wired on loopback; tear everything down on exit."""
    teensy, client, node = _build_paired_node()
    yield teensy, node
    node.on_shutdown()
    client.stop()
    teensy.stop()


def _cone_frame(can_id: int, data: bytes, t_bridge_us: int = 0) -> bytes:
    """Pack a CONE_FRAME payload relaying one cone CAN frame."""
    cf = p.ConeFrame(t_bridge_us=t_bridge_us, can_id=can_id,
                     dlc=len(data), data=tuple(data.ljust(8, b'\x00')))
    return cf.pack()


def _heartbeat_data(state=2, rms=8, last_seq=5, ms_since=100, flags=0x03) -> bytes:
    """Raw CONE_HEARTBEAT CAN payload: state, rms/state_data, seq, uint24 ms, flags."""
    return (bytes([state, rms, last_seq])
            + ms_since.to_bytes(3, 'little') + bytes([flags, 0]))


# ── Cone heartbeat ─────────────────────────────────────────────

def test_cone_heartbeat_published_connected(bridge):
    """A relayed CONE_HEARTBEAT publishes cone/heartbeat with connected=True."""
    teensy, node = bridge
    teensy.send_to_jetson(
        int(MsgType.CONE_FRAME),
        _cone_frame(catching_cone.HEARTBEAT_ID, _heartbeat_data()))
    assert _wait_until(lambda: node._cone_hb_received), "heartbeat never arrived"
    node._publish_cone_heartbeat()
    assert len(node.cone_heartbeat_pub.published) == 1
    hb = node.cone_heartbeat_pub.published[0]
    assert hb.connected is True
    assert hb.state == int(catching_cone.CatchingConeStates.READY)
    assert hb.sync_rms_us == 8
    assert hb.last_catch_seq == 5
    assert hb.ms_since_last_catch == 100
    assert hb.time_synced is True
    assert hb.have_any_catch is True


def test_cone_heartbeat_defaults_disconnected(bridge):
    """Before any cone frame the topic stays alive with connected=False
    (matching can_node, whose consumers rely on it to show 'disconnected')."""
    _, node = bridge
    node._publish_cone_heartbeat()
    assert len(node.cone_heartbeat_pub.published) == 1
    hb = node.cone_heartbeat_pub.published[0]
    assert hb.connected is False
    assert hb.state == int(catching_cone.CatchingConeStates.BOOT)
    assert hb.time_synced is False


def test_cone_heartbeat_timeout_disconnects(bridge):
    """connected drops once the last heartbeat ages past CC_HEARTBEAT_TIMEOUT_MS."""
    teensy, node = bridge
    teensy.send_to_jetson(
        int(MsgType.CONE_FRAME),
        _cone_frame(catching_cone.HEARTBEAT_ID, _heartbeat_data()))
    assert _wait_until(lambda: node._cone_hb_received)
    node._cone_last_hb_mono = time.monotonic() - (node._cone_hb_timeout_s + 0.1)
    node._publish_cone_heartbeat()
    hb = node.cone_heartbeat_pub.published[0]
    assert hb.connected is False
    # State fields still reflect the last heartbeat (stale-but-known, as in can_node).
    assert hb.state == int(catching_cone.CatchingConeStates.READY)


# ── Catch events ───────────────────────────────────────────────

def test_catch_event_synced_reconstructs_wall_time(bridge):
    """A synced catch event gets its full wall-clock instant reconstructed
    from the low-32 µs against host arrival time."""
    teensy, node = bridge
    low32 = int(time.time() * 1e6) & 0xFFFFFFFF
    data = struct.pack('<IBBH', low32, 7, 0x01, 0)  # seq=7, time_synced
    teensy.send_to_jetson(
        int(MsgType.CONE_FRAME), _cone_frame(catching_cone.CATCH_EVENT_ID, data))
    assert _wait_until(lambda: len(node._cone_catch_queue) == 1)
    node._drain_cone_catch_events()
    assert len(node.catch_event_pub.published) == 1
    evt = node.catch_event_pub.published[0]
    assert evt.sequence == 7
    assert evt.time_synced is True
    # Under the conftest mock, Time.to_msg() returns raw nanoseconds, so the
    # reconstruction is checkable exactly: same low32 + same wrap window.
    expected_us = catching_cone.reconstruct_catch_time_us(
        low32, int(time.time() * 1e6))
    assert evt.catch_time == expected_us * 1000
    assert evt.header.stamp == evt.catch_time
    # The queue drained — a second tick publishes nothing new.
    node._drain_cone_catch_events()
    assert len(node.catch_event_pub.published) == 1


def test_catch_event_unsynced_falls_back_to_host_stamp(bridge):
    """An unsynced event is still published (time_synced=False) with a
    host-derived stamp, not the raw cone µs counter (which would land
    near the Unix epoch and break downstream tools)."""
    teensy, node = bridge
    data = struct.pack('<IBBH', 42, 9, 0x00, 0)  # flags=0 → not synced
    teensy.send_to_jetson(
        int(MsgType.CONE_FRAME), _cone_frame(catching_cone.CATCH_EVENT_ID, data))
    assert _wait_until(lambda: len(node._cone_catch_queue) == 1)
    node._drain_cone_catch_events()
    evt = node.catch_event_pub.published[0]
    assert evt.time_synced is False
    assert evt.header.stamp == evt.catch_time


def test_catch_events_all_drain_in_order(bridge):
    """Catch events are discrete: every queued impact publishes exactly once,
    in arrival order — none dropped to a stash-latest race."""
    teensy, node = bridge
    for seq in (1, 2, 3):
        data = struct.pack('<IBBH', 1_000_000 + seq, seq, 0x01, 0)
        teensy.send_to_jetson(
            int(MsgType.CONE_FRAME), _cone_frame(catching_cone.CATCH_EVENT_ID, data))
    assert _wait_until(lambda: len(node._cone_catch_queue) == 3)
    node._drain_cone_catch_events()
    assert [m.sequence for m in node.catch_event_pub.published] == [1, 2, 3]


def test_cone_catch_queue_bounded_drop_oldest(bridge):
    """[16]: if the 100 Hz drain stalls, the catch queue is bounded to 4000
    (drop-oldest, same as the BB-estimates sibling) so a wedged drain cannot grow
    it without limit. We pre-fill to the cap, then a fresh catch arrives via the RX
    callback — the oldest entry is dropped and the newest is retained at the tail."""
    teensy, node = bridge
    # Pre-fill to the cap with sentinels (bypass UDP — deterministic).
    with node._lock:
        node._cone_catch_queue = [('OLD', i) for i in range(4000)]
    low32 = int(time.time() * 1e6) & 0xFFFFFFFF
    data = struct.pack('<IBBH', low32, 99, 0x01, 0)   # seq=99, time_synced
    node._on_cone_frame(int(MsgType.CONE_FRAME), 0,
                        _cone_frame(catching_cone.CATCH_EVENT_ID, data), None)
    with node._lock:
        q = list(node._cone_catch_queue)
    assert len(q) == 4000                 # still capped, not 4001
    assert q[0] != ('OLD', 0)             # oldest sentinel dropped
    assert q[-1][0].sequence == 99        # newest real catch retained at the tail


# ── Robustness ─────────────────────────────────────────────────

def test_unknown_cone_can_id_ignored(bridge):
    """A relayed frame with an unrecognised arbitration id (future cone
    message) is dropped silently — nothing queued, nothing stashed."""
    teensy, node = bridge
    teensy.send_to_jetson(
        int(MsgType.CONE_FRAME), _cone_frame(0x7E5, b'\x00' * 8))
    # Prove delivery happened (RX counter moved) rather than just sleeping;
    # the counter bumps just before dispatch, so give the callback a beat.
    assert _wait_until(
        lambda: node._client.stats.rx_count_by_type.get(int(MsgType.CONE_FRAME), 0) >= 1)
    time.sleep(0.05)
    assert node._cone_catch_queue == []
    assert node._cone_hb_received is False


def test_short_can_payload_ignored(bridge):
    """A catch-event relay whose CAN payload is short (dlc < 8) fails decode
    and is dropped without killing the RX thread."""
    teensy, node = bridge
    teensy.send_to_jetson(
        int(MsgType.CONE_FRAME), _cone_frame(catching_cone.CATCH_EVENT_ID, b'\x01\x02'))
    # A valid heartbeat afterwards still lands — the path survived.
    teensy.send_to_jetson(
        int(MsgType.CONE_FRAME),
        _cone_frame(catching_cone.HEARTBEAT_ID, _heartbeat_data()))
    assert _wait_until(lambda: node._cone_hb_received)
    assert node._cone_catch_queue == []


def test_truncated_udp_payload_ignored(bridge):
    """A CONE_FRAME payload shorter than the struct fails unpack and is
    dropped without killing the RX thread."""
    teensy, node = bridge
    teensy.send_to_jetson(int(MsgType.CONE_FRAME), b'\x01\x02\x03')
    teensy.send_to_jetson(
        int(MsgType.CONE_FRAME),
        _cone_frame(catching_cone.HEARTBEAT_ID, _heartbeat_data()))
    assert _wait_until(lambda: node._cone_hb_received)
