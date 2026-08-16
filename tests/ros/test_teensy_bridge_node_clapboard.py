"""Electronic-clapboard tests for teensy_bridge_node (clapboard uplink).

The can-bridge relays every frame on the cone bus to the Jetson as a CONE_FRAME
(raw 8-byte CAN payload + arbitration id + bridge RX timestamp) with NO
arbitration-id filter, so a plugged-in clapboard's frames already arrive on the
existing relay — the whole uplink is a Jetson-side dispatch change with no
firmware work.  These tests drive the full UDP path on loopback (FakeTeensy →
TeensyLinkClient RX thread → node callbacks) and then invoke the publish timers
directly (ROS is mocked by tests/ros/conftest.py — timers don't auto-fire).

Structurally the sibling of tests/ros/test_teensy_bridge_node_cone.py, including
its robustness trio (unknown id, short CAN payload, truncated UDP payload), each
proving delivery through ``stats.rx_count_by_type`` rather than a blind sleep and
each followed by a valid frame proving the RX thread survived.

That cone file must keep passing UNMODIFIED — it is the byte-identical proof for
the cone branch, which catch_correlation_node and the analysis tooling depend on.
"""

from __future__ import annotations

import time

import pytest

from teensy_link import protocol as p
from teensy_link import MsgType

from jugglebot.can import catching_cone
from jugglebot.can import clapboard

from tests.ros._bridge_harness import _build_paired_node, _wait_until


@pytest.fixture
def bridge():
    """Yield (teensy, node) wired on loopback; tear everything down on exit."""
    teensy, client, node = _build_paired_node()
    yield teensy, node
    node.on_shutdown()
    client.stop()
    teensy.stop()


def _cone_frame(can_id: int, data: bytes, t_bridge_us: int = 0) -> bytes:
    """Pack a CONE_FRAME payload relaying one cone-bus CAN frame."""
    cf = p.ConeFrame(t_bridge_us=t_bridge_us, can_id=can_id,
                     dlc=len(data), data=tuple(data.ljust(8, b'\x00')))
    return cf.pack()


def _heartbeat_data(state=1, flags=0x1F, fires=12, rail_mv=11900,
                    template_id=3, last_error=0) -> bytes:
    """Raw CLAP_HEARTBEAT CAN payload (protocol.md §8.7)."""
    return (bytes([state, flags])
            + int(fires).to_bytes(2, 'little')
            + int(rail_mv).to_bytes(2, 'little')
            + bytes([template_id, last_error]))


def _fire_data(wall_us: int, fires: int) -> bytes:
    """Raw CLAP_FIRE_EVENT CAN payload — 48-bit µs + u16 counter (§8.8)."""
    return ((wall_us & 0x0000FFFFFFFFFFFF).to_bytes(6, 'little')
            + int(fires).to_bytes(2, 'little'))


def _ack_data(txn_id=7, outcome=0, state=1, render_ms=2500) -> bytes:
    """Raw CLAP_ACK CAN payload (§8.6)."""
    return (bytes([txn_id, outcome, state])
            + int(render_ms).to_bytes(2, 'little') + b'\x00\x00\x00')


def _rx_count(node) -> int:
    return node._client.stats.rx_count_by_type.get(int(MsgType.CONE_FRAME), 0)


# ── Clapboard heartbeat ────────────────────────────────────────

def test_clapboard_heartbeat_published_connected(bridge):
    """A relayed CLAP_HEARTBEAT publishes clapboard/heartbeat, connected=True."""
    teensy, node = bridge
    teensy.send_to_jetson(
        int(MsgType.CONE_FRAME),
        _cone_frame(clapboard.HEARTBEAT_ID, _heartbeat_data()))
    assert _wait_until(lambda: node._clap_hb_received), "heartbeat never arrived"
    node._publish_clapboard_heartbeat()
    assert len(node.clapboard_heartbeat_pub.published) == 1
    hb = node.clapboard_heartbeat_pub.published[0]
    assert hb.connected is True
    assert hb.state == int(clapboard.ClapboardStates.IDLE)
    assert hb.fire_ready is True
    assert hb.template_loaded is True
    assert hb.wifi_up is True
    assert hb.rail_ok is True
    assert hb.time_synced is True
    assert hb.fires_since_boot == 12
    assert hb.rail_mv == 11900
    assert hb.active_template_id == 3
    assert hb.last_error == 0


def test_clapboard_heartbeat_defaults_disconnected(bridge):
    """Before any clapboard frame the topic stays alive with connected=False.

    A stated consumer contract, not an accident: a consumer can only render
    'clapboard disconnected' if the topic is alive to say so.
    """
    _, node = bridge
    node._publish_clapboard_heartbeat()
    assert len(node.clapboard_heartbeat_pub.published) == 1
    hb = node.clapboard_heartbeat_pub.published[0]
    assert hb.connected is False
    assert hb.state == int(clapboard.ClapboardStates.BOOT)
    assert hb.time_synced is False
    assert hb.fires_since_boot == 0


def test_clapboard_heartbeat_timeout_disconnects(bridge):
    """connected drops once the last heartbeat ages past CLAP_HEARTBEAT_TIMEOUT_MS."""
    teensy, node = bridge
    teensy.send_to_jetson(
        int(MsgType.CONE_FRAME),
        _cone_frame(clapboard.HEARTBEAT_ID, _heartbeat_data(state=3)))
    assert _wait_until(lambda: node._clap_hb_received)
    node._clap_last_hb_mono = time.monotonic() - (node._clap_hb_timeout_s + 0.1)
    node._publish_clapboard_heartbeat()
    hb = node.clapboard_heartbeat_pub.published[0]
    assert hb.connected is False
    # State fields still reflect the last heartbeat (stale-but-known, as the
    # cone's do) — 'disconnected' is a statement about freshness, not content.
    assert hb.state == int(clapboard.ClapboardStates.SCREENSAVER)


def test_clapboard_timeout_is_the_host_side_one_not_a_firmware_flag(bridge):
    """Presence is timed on the host, so it needs no firmware change at all."""
    _, node = bridge
    import jugglebot.protocol_config as proto
    assert node._clap_hb_timeout_s == proto.CLAP_HEARTBEAT_TIMEOUT_MS / 1000.0
    assert node._clap_hb_timeout_s > 0.0


def test_unknown_heartbeat_state_still_reports_connected(bridge):
    """A state byte this host predates must not read as 'disconnected'.

    The decoder passes unknown state values through instead of raising, because
    a raise would drop the frame in the RX callback and the node would report the
    clapboard gone 500 ms later — confidently wrong, where an unrecognised
    number is merely uninformative.
    """
    teensy, node = bridge
    teensy.send_to_jetson(
        int(MsgType.CONE_FRAME),
        _cone_frame(clapboard.HEARTBEAT_ID, _heartbeat_data(state=9)))
    assert _wait_until(lambda: node._clap_hb_received)
    node._publish_clapboard_heartbeat()
    hb = node.clapboard_heartbeat_pub.published[0]
    assert hb.connected is True
    assert hb.state == 9


# ── Fire events ────────────────────────────────────────────────

def test_fire_event_reconstructs_wall_time(bridge):
    """A fire event's 48-bit µs field is reconstructed into a full wall clock.

    Taking the wire field at face value would stamp the flash ~47 years in the
    past — the field carries only the low 48 bits of a value that needs 51.
    """
    teensy, node = bridge
    host_us = int(time.time() * 1e6)
    teensy.send_to_jetson(
        int(MsgType.CONE_FRAME),
        _cone_frame(clapboard.FIRE_EVENT_ID, _fire_data(host_us, 5)))
    assert _wait_until(lambda: len(node._clap_fire_queue) == 1)
    node._drain_clapboard_fire_events()
    assert len(node.clapboard_fire_event_pub.published) == 1
    evt = node.clapboard_fire_event_pub.published[0]
    assert evt.fires_since_boot == 5
    # Under the conftest mock, Time.to_msg() returns raw nanoseconds, so the
    # reconstruction is checkable exactly.
    assert evt.fire_time_us == host_us
    assert evt.fire_time == host_us * 1000
    assert evt.header.stamp == evt.fire_time
    # The queue drained — a second tick publishes nothing new.
    node._drain_clapboard_fire_events()
    assert len(node.clapboard_fire_event_pub.published) == 1


def test_fire_event_masked_overflow_does_not_corrupt_the_counter(bridge):
    """An all-ones timestamp must not bleed into fires_since_boot."""
    teensy, node = bridge
    teensy.send_to_jetson(
        int(MsgType.CONE_FRAME),
        _cone_frame(clapboard.FIRE_EVENT_ID,
                    _fire_data(0xFFFF_FFFF_FFFF_FFFF, 9)))
    assert _wait_until(lambda: len(node._clap_fire_queue) == 1)
    fe, _arrival = node._clap_fire_queue[0]
    assert fe.wall_us_low48 == (1 << 48) - 1
    assert fe.fires_since_boot == 9


def test_fire_events_all_drain_in_order(bridge):
    """Fire events are discrete: every flash publishes exactly once, in order.

    A stash-latest would delete sync records, and a sync record is the one thing
    the device exists to produce.
    """
    teensy, node = bridge
    host_us = int(time.time() * 1e6)
    for n in (1, 2, 3):
        teensy.send_to_jetson(
            int(MsgType.CONE_FRAME),
            _cone_frame(clapboard.FIRE_EVENT_ID,
                        _fire_data(host_us + n * 1000, n)))
    assert _wait_until(lambda: len(node._clap_fire_queue) == 3)
    node._drain_clapboard_fire_events()
    assert [m.fires_since_boot
            for m in node.clapboard_fire_event_pub.published] == [1, 2, 3]


def test_fire_queue_bounded_drop_oldest(bridge):
    """If the 100 Hz drain stalls the queue is capped (drop-oldest), so a wedged
    timer cannot grow it without limit. Pre-fill to the cap, then let a real
    event arrive through the RX callback: oldest dropped, newest at the tail."""
    from jugglebot.teensy_bridge_node import _CLAP_FIRE_QUEUE_MAX
    _teensy, node = bridge
    with node._lock:
        node._clap_fire_queue = [('OLD', i) for i in range(_CLAP_FIRE_QUEUE_MAX)]
    node._on_cone_frame(
        int(MsgType.CONE_FRAME), 0,
        _cone_frame(clapboard.FIRE_EVENT_ID, _fire_data(1_234_567, 99)), None)
    with node._lock:
        q = list(node._clap_fire_queue)
    assert len(q) == _CLAP_FIRE_QUEUE_MAX      # still capped, not cap+1
    assert q[0] != ('OLD', 0)                  # oldest sentinel dropped
    assert q[-1][0].fires_since_boot == 99     # newest real flash at the tail


def test_no_fire_event_means_no_publish(bridge):
    """The clapboard emits NOTHING while unsynced — the flash still happens, only
    the record is suppressed. A gap between fires_since_boot and published events
    is therefore EXPECTED and diagnostic; the node must not invent a stamp."""
    teensy, node = bridge
    teensy.send_to_jetson(
        int(MsgType.CONE_FRAME),
        # time_synced clear (flags bit4), 4 flashes counted on the device
        _cone_frame(clapboard.HEARTBEAT_ID,
                    _heartbeat_data(flags=0x0F, fires=4)))
    assert _wait_until(lambda: node._clap_hb_received)
    node._drain_clapboard_fire_events()
    node._publish_clapboard_heartbeat()
    assert node.clapboard_fire_event_pub.published == []
    hb = node.clapboard_heartbeat_pub.published[0]
    assert hb.time_synced is False
    assert hb.fires_since_boot == 4


# ── Ack ────────────────────────────────────────────────────────

def test_clap_ack_is_decoded_and_stashed(bridge):
    """CLAP_ACK is decoded at this one point and stashed for the later SetSlate
    action; nothing is published for it yet (no downlink exists to provoke one)."""
    teensy, node = bridge
    teensy.send_to_jetson(
        int(MsgType.CONE_FRAME),
        _cone_frame(clapboard.ACK_ID,
                    _ack_data(txn_id=17, outcome=2, state=2, render_ms=2750)))
    assert _wait_until(lambda: node._latest_clap_ack is not None)
    ack = node._latest_clap_ack
    assert ack.txn_id == 17
    assert ack.outcome == int(clapboard.ClapboardAckOutcome.CRC_MISMATCH)
    assert ack.state == int(clapboard.ClapboardStates.RENDERING)
    assert ack.render_ms == 2750
    assert node._latest_clap_ack_mono > 0.0


# ── Downlink-only ids arriving on the uplink ───────────────────

@pytest.mark.parametrize('can_id_attr', ['FIELD_ID', 'COMMIT_ID', 'LINK_ID'])
def test_downlink_only_id_counted_not_decoded(bridge, can_id_attr):
    """0x7E8/0x7E9/0x7EA only ever travel Jetson→clapboard. One arriving inbound
    is counted for a one-shot report and dropped — never decoded as anything."""
    teensy, node = bridge
    can_id = getattr(clapboard, can_id_attr)
    before = _rx_count(node)
    teensy.send_to_jetson(int(MsgType.CONE_FRAME),
                          _cone_frame(can_id, b'\x01' * 8))
    assert _wait_until(lambda: _rx_count(node) > before)
    assert _wait_until(lambda: node._clap_downlink_uplink_count == 1)
    assert node._clap_hb_received is False
    assert node._clap_fire_queue == []
    assert node._latest_clap_ack is None


def test_downlink_only_report_is_one_shot(bridge):
    """The report latches after the first publish tick — a wiring fault is a
    condition, not an event, and re-announcing it at 10 Hz is how a detector
    stops being read."""
    teensy, node = bridge
    teensy.send_to_jetson(int(MsgType.CONE_FRAME),
                          _cone_frame(clapboard.LINK_ID, b'\x01' * 8))
    assert _wait_until(lambda: node._clap_downlink_uplink_count == 1)
    node._publish_clapboard_heartbeat()
    assert node._clap_downlink_uplink_warned is True
    node._publish_clapboard_heartbeat()
    assert node._clap_downlink_uplink_warned is True
    # The heartbeat topic keeps publishing regardless.
    assert len(node.clapboard_heartbeat_pub.published) == 2


# ── Cone coexistence ───────────────────────────────────────────

def test_cone_frames_still_route_after_the_clapboard_branches(bridge):
    """The clapboard branches sit AHEAD of the cone ones; the cone path must be
    untouched. (tests/ros/test_teensy_bridge_node_cone.py passing unmodified is
    the primary proof; this pins the coexistence explicitly.)"""
    teensy, node = bridge
    cone_hb = (bytes([2, 8, 5]) + (100).to_bytes(3, 'little') + bytes([0x03, 0]))
    teensy.send_to_jetson(int(MsgType.CONE_FRAME),
                          _cone_frame(catching_cone.HEARTBEAT_ID, cone_hb))
    teensy.send_to_jetson(int(MsgType.CONE_FRAME),
                          _cone_frame(clapboard.HEARTBEAT_ID, _heartbeat_data()))
    assert _wait_until(lambda: node._cone_hb_received and node._clap_hb_received)
    node._publish_cone_heartbeat()
    node._publish_clapboard_heartbeat()
    assert node.cone_heartbeat_pub.published[0].connected is True
    assert node.clapboard_heartbeat_pub.published[0].connected is True


def test_a_clapboard_heartbeat_never_decodes_as_a_catch_event(bridge):
    """Distinct ids make a mis-plug diagnosable rather than silently wrong: a
    clapboard heartbeat must never inject a phantom catch into cone/catch_event
    (which catch_correlation_node consumes)."""
    teensy, node = bridge
    teensy.send_to_jetson(
        int(MsgType.CONE_FRAME),
        _cone_frame(clapboard.HEARTBEAT_ID, _heartbeat_data()))
    assert _wait_until(lambda: node._clap_hb_received)
    node._drain_cone_catch_events()
    assert node.catch_event_pub.published == []
    assert node._cone_catch_queue == []
    assert node._cone_hb_received is False


# ── Robustness ─────────────────────────────────────────────────

def test_unknown_cone_bus_can_id_ignored(bridge):
    """A relayed frame with an id in neither block (0x7E2-0x7E7 is unallocated)
    is dropped silently — nothing queued, nothing stashed."""
    teensy, node = bridge
    before = _rx_count(node)
    teensy.send_to_jetson(
        int(MsgType.CONE_FRAME), _cone_frame(0x7E5, b'\x00' * 8))
    # Prove delivery happened (RX counter moved) rather than just sleeping;
    # the counter bumps just before dispatch, so give the callback a beat.
    assert _wait_until(lambda: _rx_count(node) > before)
    time.sleep(0.05)
    assert node._clap_fire_queue == []
    assert node._clap_hb_received is False
    assert node._latest_clap_ack is None
    assert node._clap_downlink_uplink_count == 0
    # ...and the path still works afterwards.
    teensy.send_to_jetson(
        int(MsgType.CONE_FRAME),
        _cone_frame(clapboard.HEARTBEAT_ID, _heartbeat_data()))
    assert _wait_until(lambda: node._clap_hb_received)


def test_short_can_payload_ignored(bridge):
    """A clapboard relay whose CAN payload is short (dlc < 8) fails the strict
    DLC-8 decode and is dropped without killing the RX thread."""
    teensy, node = bridge
    before = _rx_count(node)
    teensy.send_to_jetson(
        int(MsgType.CONE_FRAME),
        _cone_frame(clapboard.FIRE_EVENT_ID, b'\x01\x02'))
    assert _wait_until(lambda: _rx_count(node) > before)
    # A valid heartbeat afterwards still lands — the path survived.
    teensy.send_to_jetson(
        int(MsgType.CONE_FRAME),
        _cone_frame(clapboard.HEARTBEAT_ID, _heartbeat_data()))
    assert _wait_until(lambda: node._clap_hb_received)
    assert node._clap_fire_queue == []


def test_truncated_udp_payload_ignored(bridge):
    """A CONE_FRAME payload shorter than the struct fails unpack and is dropped
    without killing the RX thread."""
    teensy, node = bridge
    before = _rx_count(node)
    teensy.send_to_jetson(int(MsgType.CONE_FRAME), b'\x01\x02\x03')
    assert _wait_until(lambda: _rx_count(node) > before)
    teensy.send_to_jetson(
        int(MsgType.CONE_FRAME),
        _cone_frame(clapboard.HEARTBEAT_ID, _heartbeat_data()))
    assert _wait_until(lambda: node._clap_hb_received)
