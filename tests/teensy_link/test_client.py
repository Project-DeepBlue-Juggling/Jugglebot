"""Integration tests: real UDP sockets on loopback, FakeTeensy peer."""

from __future__ import annotations

import struct
import time

import pytest

from teensy_link import (
    TeensyLinkClient,
    HeartbeatJ2T,
    HeartbeatT2J,
    MsgType,
)
from teensy_link import protocol as p


# Imported for the fixture
from .conftest import FakeTeensy  # noqa: F401


def test_client_sends_heartbeat_when_called(fake_teensy_and_client):
    teensy, client = fake_teensy_and_client
    client.send_heartbeat(t_jetson_us=1234, flags=0b1)
    got = teensy.wait_for(int(MsgType.HEARTBEAT_J2T), count=1, timeout=0.5)
    assert len(got) == 1
    hb = HeartbeatJ2T.unpack(got[0].payload)
    assert hb.t_jetson_us == 1234
    assert hb.flags == 0b1


def test_client_periodic_heartbeat_thread(fake_teensy_and_client):
    teensy, client = fake_teensy_and_client
    client.start_heartbeat(hz=50.0)  # fast for quick test
    got = teensy.wait_for(int(MsgType.HEARTBEAT_J2T), count=5, timeout=1.0)
    assert len(got) >= 5
    # seqs should be monotonic
    seqs = [g.seq for g in got[:5]]
    assert seqs == sorted(seqs)


def test_client_receives_and_decodes_telemetry(fake_teensy_and_client):
    teensy, client = fake_teensy_and_client
    received = []

    def on_telem(msg_type, seq, payload, addr):
        received.append((seq, p.Telemetry.unpack(payload)))

    client.subscribe(int(MsgType.TELEMETRY), on_telem)
    teensy.send_telemetry(pos_rev=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0),
                          vel_rps=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7))
    # Wait up to 0.5s for the callback to fire
    deadline = time.time() + 0.5
    while time.time() < deadline and not received:
        time.sleep(0.01)
    assert len(received) == 1
    seq, tm = received[0]
    assert tm.pos_rev == pytest.approx((1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0))
    assert tm.vel_rps == pytest.approx((0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7))


def test_client_tracks_seq_gaps(fake_teensy_and_client):
    teensy, client = fake_teensy_and_client
    # send seq 0, then skip to seq 5 manually by directly using send_to_jetson
    # FakeTeensy's send_to_jetson auto-increments; manipulate _tx_seq_stream.
    teensy.send_telemetry()  # seq=0
    teensy._tx_seq_stream = 5  # skip ahead
    teensy.send_telemetry()  # seq=5
    time.sleep(0.1)
    # The client should have noticed a gap (seq jumped from 0 to 5, expected 1)
    assert client.stats.seq_gaps_by_type.get(int(MsgType.TELEMETRY), 0) >= 1


def test_client_counts_crc_errors(fake_teensy_and_client):
    teensy, client = fake_teensy_and_client
    # Send a manually-corrupted frame
    hb = HeartbeatT2J().pack()
    frame = bytearray(p.encode_frame(int(MsgType.HEARTBEAT_T2J), 0, hb))
    frame[-1] ^= 0xFF  # corrupt CRC
    teensy.stream_sock.sendto(bytes(frame), teensy._jetson_addr)  # type: ignore[attr-defined]
    # Actually — _jetson_addr is the stream addr only; just use the patched sender's dest
    # The conftest monkey-patches send_to_jetson; we need the actual jetson stream port.
    # Easier: send a corrupted frame via the patched helper by injecting raw.
    time.sleep(0.1)
    assert client.stats.crc_errors >= 1


def test_client_counts_decode_errors_distinct_from_crc(fake_teensy_and_client):
    teensy, client = fake_teensy_and_client
    # A structurally-malformed frame (bad magic, checked before the CRC) must
    # count as a decode_error, NOT a crc_error. Guards the typed-CrcError split
    # in the RX path against the old fragile `"CRC" in str(e)` misclassification.
    frame = bytearray(p.encode_frame(int(MsgType.HEARTBEAT_T2J), 0, HeartbeatT2J().pack()))
    frame[0] ^= 0xFF  # corrupt magic
    teensy.stream_sock.sendto(bytes(frame), teensy._jetson_addr)  # type: ignore[attr-defined]
    deadline = time.time() + 0.5
    while time.time() < deadline and client.stats.decode_errors < 1:
        time.sleep(0.01)
    assert client.stats.decode_errors >= 1
    assert client.stats.crc_errors == 0


def test_client_stats_track_rx_count_by_type(fake_teensy_and_client):
    teensy, client = fake_teensy_and_client
    for _ in range(3):
        teensy.send_telemetry()
    for _ in range(2):
        teensy.send_heartbeat_t2j()
    time.sleep(0.15)
    assert client.stats.rx_count_by_type.get(int(MsgType.TELEMETRY), 0) >= 3
    assert client.stats.rx_count_by_type.get(int(MsgType.HEARTBEAT_T2J), 0) >= 2


def test_client_stats_track_tx_count_by_type(fake_teensy_and_client):
    """TX per-type counters: the /udp_diag tx_<TYPE> rows' only source.

    Covers two of the three send paths (``send_stream`` via send_heartbeat, and
    ``send_rpc``); ``send_to`` is exercised by
    ``test_client_send_to_counts_tx_by_type`` below, which needs an explicit
    destination.
    """
    teensy, client = fake_teensy_and_client
    for _ in range(3):
        client.send_heartbeat(t_jetson_us=1)          # send_stream
    client.send_rpc(int(MsgType.RPC_REQUEST), b'')    # send_rpc
    assert client.stats.tx_count_by_type[int(MsgType.HEARTBEAT_J2T)] == 3
    assert client.stats.tx_count_by_type[int(MsgType.RPC_REQUEST)] == 1
    # A type nobody sent stays an honest 0 (pre-seeded), never absent.
    assert client.stats.tx_count_by_type[int(MsgType.SETPOINT)] == 0


def test_client_send_to_counts_tx_by_type(fake_teensy_and_client):
    """``send_to`` (the RPC-response path) counts too — it is the only send
    path that does NOT bump the tx seq, so it is the one most easily missed."""
    teensy, client = fake_teensy_and_client
    dest = ("127.0.0.1", teensy.rpc_port)
    for _ in range(2):
        client.send_to("rpc", int(MsgType.RPC_RESPONSE), 7, b'', dest)
    assert client.stats.tx_count_by_type[int(MsgType.RPC_RESPONSE)] == 2


def test_link_stats_preseed_every_msgtype():
    """Both per-type counter dicts carry EVERY MsgType key at construction.

    Two distinct properties depend on this (see LinkStats' docstring):
      * ``snapshot()`` copies these dicts unlocked from the ROS timer thread
        while the RX thread mutates them — rebinding a pre-seeded key is safe,
        INSERTING one during ``dict()`` raises RuntimeError.
      * a never-seen type renders as an honest ``0`` row on /udp_diag instead
        of vanishing.
    """
    from teensy_link.client import LinkStats

    stats = LinkStats()
    want = {int(t) for t in MsgType}
    for name in ('rx_count_by_type', 'tx_count_by_type', 'seq_gaps_by_type'):
        got = set(getattr(stats, name))
        assert got == want, f'{name} is not pre-seeded with every MsgType'
        assert set(getattr(stats, name).values()) == {0}, f'{name} seeded nonzero'

    # Each instance gets its OWN dicts (default_factory, not a shared mutable
    # default) — two clients on one box must not share counters.
    other = LinkStats()
    stats.rx_count_by_type[int(MsgType.TELEMETRY)] = 5
    assert other.rx_count_by_type[int(MsgType.TELEMETRY)] == 0

    # And the snapshot carries the full key set through.
    snap = stats.snapshot()
    assert set(snap.tx_count_by_type) == want
    assert snap.rx_count_by_type[int(MsgType.TELEMETRY)] == 5


def test_snapshot_survives_concurrent_rx_counting(fake_teensy_and_client):
    """Snapshotting under live RX traffic must not raise.

    This is the race the pre-seed closes: ``snapshot()`` does ``dict(...)`` on
    three dicts the RX thread is writing, with no lock.  With every known key
    pre-seeded the RX thread only ever REBINDS, so the copy is safe.  Loops so
    the snapshot lands mid-burst rather than in a quiet gap.
    """
    teensy, client = fake_teensy_and_client
    for _ in range(50):
        teensy.send_telemetry()
        teensy.send_heartbeat_t2j()
        snap = client.stats.snapshot()
        assert set(snap.rx_count_by_type) >= {int(t) for t in MsgType}
    time.sleep(0.15)
    assert client.stats.rx_count_by_type[int(MsgType.TELEMETRY)] >= 1


def test_client_subscribe_unsubscribe(fake_teensy_and_client):
    teensy, client = fake_teensy_and_client
    count = [0]

    def on_telem(msg_type, seq, payload, addr):
        count[0] += 1

    unsub = client.subscribe(int(MsgType.TELEMETRY), on_telem)
    teensy.send_telemetry()
    time.sleep(0.1)
    assert count[0] == 1
    unsub()
    teensy.send_telemetry()
    time.sleep(0.1)
    assert count[0] == 1  # callback no longer fires


def test_client_wildcard_subscription_sees_everything(fake_teensy_and_client):
    teensy, client = fake_teensy_and_client
    all_types = []

    def on_any(msg_type, seq, payload, addr):
        all_types.append(msg_type)

    client.subscribe_all(on_any)
    teensy.send_telemetry()
    teensy.send_heartbeat_t2j()
    time.sleep(0.15)
    assert int(MsgType.TELEMETRY) in all_types
    assert int(MsgType.HEARTBEAT_T2J) in all_types
