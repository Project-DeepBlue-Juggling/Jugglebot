"""Kernel RX timestamping and the midpoint-stamped TOD response (P2).

``plans/archived/bridge-temporal-trustworthiness.md`` § P2 (arc
closed 2026-08-15 — ``logbook/2026-08-15-fw14-validated-arc-closed.md``). The
firmware anchors
its wall clock to ``returned_stamp + rtt/2``, so with the query hitting the
Jetson kernel at ``t2``, being stamped at ``ts`` and sent back at ``t3``::

    anchor error = (df − dr)/2 + (ts − (t2 + t3)/2)

Stamping at ``t3`` (pre-P2) leaves ``+p/2``; stamping at ``t2`` alone leaves
``−p/2``. Only the midpoint cancels the server's processing delay ``p``. These
tests pin that midpoint, the exact-``t3`` fallback, and — equally load-bearing —
that the fallback is *reported* rather than silent.

Everything binds ephemeral ports (``:0``) and the file is xdist-safe: the
``fake_teensy_and_client`` fixture from ``conftest.py`` builds its own pair.

Probe recipe confirmed on this box before these tests were written (Jetson,
Ubuntu 20.04, kernel 5.10.104-tegra, Python 3.8.10 — both the system
interpreter and the project venv): ``setsockopt(SOL_SOCKET, 35, 1)`` on a
loopback UDP socket, then ``recvmsg(2048, CMSG_SPACE(16))`` yields
``(SOL_SOCKET=1, 35, <16-byte struct timespec>)`` whose value tracked
``time.time()`` to ~100 µs. ``socket.SO_TIMESTAMPNS`` / ``socket.SCM_TIMESTAMPNS``
are ABSENT from that interpreter, which is why the numeric constant + runtime
probe exist at all.
"""

from __future__ import annotations

import socket
import struct
import sys
import time

import pytest

from teensy_link import (
    MsgType,
    RpcMethod,
    RpcServer,
    RpcStatus,
    TeensyLinkClient,
    TimeOfDayServer,
)
from teensy_link import client as client_mod
from teensy_link import protocol as p
from teensy_link import tod_server as tod_mod

# Imported for the fixture
from .conftest import FakeTeensy  # noqa: F401


_LINUX_STAMPING = sys.platform.startswith("linux") and client_mod._HAS_RECVMSG
_requires_kernel_stamping = pytest.mark.skipif(
    not _LINUX_STAMPING,
    reason="SO_TIMESTAMPNS ancillary data is a Linux/POSIX facility "
           "(the Win10 sim box takes the userspace fallback path)",
)

_ADDR = ("127.0.0.1", 5006)


def _wait(pred, timeout=2.0, dt=0.005):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if pred():
            return True
        time.sleep(dt)
    return pred()


def _tod_result(tod, rx_wall_us):
    """Drive the registered handler directly and return its jetson_wall_us."""
    status, blob = tod._handle(7, b"", _ADDR, rx_wall_us)
    assert status == int(RpcStatus.OK)
    assert len(blob) == 8, "ResultTimeOfDay must stay a single little-endian uint64"
    return struct.unpack("<Q", blob)[0]


# ── The socket-level fact the whole phase rests on ────────────────────────


@_requires_kernel_stamping
def test_kernel_stamp_arrives_and_parses_as_clock_realtime():
    """A real loopback datagram carries a parseable CLOCK_REALTIME timespec.

    This is the ground truth behind the runtime probe: the numeric constant is
    right, the cmsg is delivered, and the value is on the SAME clock as
    ``time.time()`` (which is what ``TimeOfDayServer``'s t3 uses — averaging two
    different clock domains would be a category error, not a rounding one).
    """
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        rx.bind(("127.0.0.1", 0))            # ephemeral — xdist-safe
        rx.setsockopt(socket.SOL_SOCKET, client_mod.SO_TIMESTAMPNS, 1)
        before_us = int(time.time() * 1_000_000)
        tx.sendto(b"probe", rx.getsockname())
        data, ancdata, _flags, _addr = rx.recvmsg(2048, client_mod._RX_ANCBUF_SIZE)
        after_us = int(time.time() * 1_000_000)
        assert data == b"probe"
        stamp_us = client_mod._parse_rx_timestamp_us(ancdata)
        assert stamp_us is not None, f"no SCM_TIMESTAMPNS cmsg in {ancdata!r}"
        # Bracketed by the two userspace reads around the send — a much tighter
        # claim than "within seconds", and it fails loudly if the parse produced
        # nanoseconds, seconds, or a byte-swapped value.
        assert before_us - 1_000_000 <= stamp_us <= after_us + 1_000_000
    finally:
        rx.close()
        tx.close()


def test_parse_returns_none_for_unrelated_ancillary_data():
    """A cmsg that is not ours (or is truncated) must read as ABSENT, not as 0.

    Returning 0 would sail through as a "valid" t2 and drag the midpoint to
    epoch/2. None is what triggers the reported fallback.
    """
    assert client_mod._parse_rx_timestamp_us([]) is None
    assert client_mod._parse_rx_timestamp_us(
        [(socket.SOL_SOCKET, client_mod.SCM_TIMESTAMPNS, b"\x00" * 4)]) is None
    assert client_mod._parse_rx_timestamp_us(
        [(socket.SOL_SOCKET, 0xBEEF, b"\x00" * 16)]) is None


# ── Client: which socket pays, and what the mode reports ──────────────────


@_requires_kernel_stamping
def test_rpc_socket_frames_carry_a_kernel_stamp(fake_teensy_and_client):
    """An RPC-port frame reaches a stamped subscriber with a plausible t2."""
    teensy, client = fake_teensy_and_client
    seen = []
    client.subscribe(int(MsgType.RPC_REQUEST),
                     lambda mt, seq, pl, addr, rx: seen.append(rx),
                     with_rx_stamp=True)
    before_us = int(time.time() * 1_000_000)
    teensy.send_rpc_request(int(RpcMethod.NOP), req_id=1)
    assert _wait(lambda: seen), "no RPC_REQUEST dispatched"
    after_us = int(time.time() * 1_000_000)
    assert seen[0] is not None
    assert before_us - 1_000_000 <= seen[0] <= after_us + 1_000_000
    # The runtime probe has now been satisfied by a REAL packet.
    assert client.rx_stamp_mode == client_mod.RX_STAMP_KERNEL


def test_stream_socket_frames_carry_no_stamp(fake_teensy_and_client):
    """The STREAM path does not pay for timestamping — it stays on recvfrom.

    A stamped subscriber on a stream type therefore sees ``None``. Pinning this
    keeps a future 'just enable it everywhere' change from adding a per-frame
    cmsg parse to the 100 Hz+ downlink for no anchor benefit.
    """
    teensy, client = fake_teensy_and_client
    seen = []
    client.subscribe(int(MsgType.TELEMETRY),
                     lambda mt, seq, pl, addr, rx: seen.append(rx),
                     with_rx_stamp=True)
    teensy.send_telemetry()
    assert _wait(lambda: seen), "no TELEMETRY dispatched"
    assert seen[0] is None


def test_plain_subscriber_still_receives_four_arguments(fake_teensy_and_client):
    """Every pre-existing subscriber keeps its 4-argument contract.

    The callback below takes exactly four positionals, so a 5-argument dispatch
    would raise TypeError inside ``_dispatch`` — which swallows exceptions and
    logs them, hence the explicit "it ran" assertion rather than trusting a
    silent pass.
    """
    teensy, client = fake_teensy_and_client
    seen = []

    def four_arg(msg_type, seq, payload, addr):
        seen.append(msg_type)

    client.subscribe(int(MsgType.RPC_REQUEST), four_arg)   # RPC port = the stamped one
    teensy.send_rpc_request(int(RpcMethod.NOP), req_id=2)
    assert _wait(lambda: seen), "plain 4-arg subscriber never fired"
    assert seen[0] == int(MsgType.RPC_REQUEST)


def test_unsubscribe_still_removes_the_callback(fake_teensy_and_client):
    """The registry now holds (callback, wants_stamp) tuples — unsub must still work."""
    teensy, client = fake_teensy_and_client
    count = [0]

    def on_telem(msg_type, seq, payload, addr):
        count[0] += 1

    unsub = client.subscribe(int(MsgType.TELEMETRY), on_telem)
    teensy.send_telemetry()
    assert _wait(lambda: count[0] == 1)
    unsub()
    teensy.send_telemetry()
    time.sleep(0.1)
    assert count[0] == 1


def test_unavailable_platform_reports_userspace(monkeypatch):
    """No recvmsg/CMSG_SPACE (Windows) → clean fallback at setup, and it SAYS so."""
    monkeypatch.setattr(client_mod, "_HAS_RECVMSG", False)
    client = TeensyLinkClient(teensy_addr=("127.0.0.1", 1), rpc_port=1,
                              local_bind_stream=0, local_bind_rpc=0,
                              bind_host="127.0.0.1")
    client.start()
    try:
        assert client.rx_stamp_mode == client_mod.RX_STAMP_USERSPACE
    finally:
        client.stop()


def test_setsockopt_failure_reports_userspace(monkeypatch):
    """A refused socket option degrades at setup instead of raising."""
    monkeypatch.setattr(client_mod, "SO_TIMESTAMPNS", 0x7FFFFFF0)  # not a real option
    client = TeensyLinkClient(teensy_addr=("127.0.0.1", 1), rpc_port=1,
                              local_bind_stream=0, local_bind_rpc=0,
                              bind_host="127.0.0.1")
    client.start()
    try:
        assert client.rx_stamp_mode == client_mod.RX_STAMP_USERSPACE
    finally:
        client.stop()


@_requires_kernel_stamping
def test_accepted_setsockopt_without_delivery_degrades_and_reports(
        fake_teensy_and_client, monkeypatch):
    """The RUNTIME half of the probe: accepted ≠ working.

    A kernel that takes the option but never delivers the cmsg must not leave the
    mode reading 'kernel' — that is exactly the invisible ``+p/2`` the plan
    forbids. Driven by making the parse return None on a real packet.
    """
    teensy, client = fake_teensy_and_client
    assert client.rx_stamp_mode == client_mod.RX_STAMP_PROBING
    seen = []
    client.subscribe(int(MsgType.RPC_REQUEST),
                     lambda mt, seq, pl, addr, rx: seen.append(rx),
                     with_rx_stamp=True)
    monkeypatch.setattr(client_mod, "_parse_rx_timestamp_us", lambda ancdata: None)
    teensy.send_rpc_request(int(RpcMethod.NOP), req_id=3)
    assert _wait(lambda: seen), "no RPC_REQUEST dispatched"
    assert seen[0] is None
    assert client.rx_stamp_mode == client_mod.RX_STAMP_USERSPACE
    monkeypatch.undo()          # parsing works again from here
    # The degrade LATCHES: the drain is back on recvfrom, so a later packet does
    # not silently re-promote the mode even though parsing works again.
    teensy.send_rpc_request(int(RpcMethod.NOP), req_id=4)
    assert _wait(lambda: len(seen) == 2)
    assert seen[1] is None
    assert client.rx_stamp_mode == client_mod.RX_STAMP_USERSPACE


# ── TimeOfDayServer: the midpoint, the fallback, and the reported mode ────


def test_midpoint_is_returned_when_a_kernel_stamp_is_present(fake_teensy_and_client):
    """jetson_wall_us == (t2 + t3)//2, and explicitly NOT t3.

    The 'not t3' half is the point: t3 alone is what the pre-P2 code returned, so
    a regression that dropped t2 on the floor would still produce a plausible
    timestamp and pass any 'is it about now' assertion.
    """
    _teensy, client = fake_teensy_and_client
    rpc_server = RpcServer(client)
    t2, t3 = 1_700_000_000_000_000, 1_700_000_000_000_400   # 400 µs of processing
    tod = TimeOfDayServer(rpc_server, clock_fn=lambda: t3)
    try:
        got = _tod_result(tod, t2)
        assert got == (t2 + t3) // 2 == 1_700_000_000_000_200
        assert got != t3
        assert tod.stamp_mode == tod_mod.STAMP_MODE_KERNEL == "kernel-midpoint"
        assert tod.call_count == 1
    finally:
        tod.close()
        rpc_server.close()


def test_fallback_returns_t3_bit_for_bit(fake_teensy_and_client):
    """t2 absent → the exact pre-P2 value, no arithmetic applied."""
    _teensy, client = fake_teensy_and_client
    rpc_server = RpcServer(client)
    t3 = 987_654_321
    tod = TimeOfDayServer(rpc_server, clock_fn=lambda: t3)
    try:
        assert _tod_result(tod, None) == t3
        assert tod.stamp_mode == tod_mod.STAMP_MODE_USERSPACE == "userspace"
    finally:
        tod.close()
        rpc_server.close()


def test_stamp_mode_is_unknown_before_the_first_query(fake_teensy_and_client):
    """No query answered yet is its own state — not a claim of either mode."""
    _teensy, client = fake_teensy_and_client
    rpc_server = RpcServer(client)
    tod = TimeOfDayServer(rpc_server, clock_fn=lambda: 1)
    try:
        assert tod.stamp_mode == tod_mod.STAMP_MODE_UNKNOWN == "unknown"
    finally:
        tod.close()
        rpc_server.close()


def test_stamp_mode_tracks_a_mid_session_degrade_and_recovers(fake_teensy_and_client):
    """kernel-midpoint → userspace is visible the moment stamps stop arriving —
    and a good stamp AFTER a bad spell recovers to kernel-midpoint (per-query
    semantics, deliberately NOT the client-side latch: a transient NTP step must
    not condemn the rest of the session to userspace stamping)."""
    _teensy, client = fake_teensy_and_client
    rpc_server = RpcServer(client)
    t3 = 1_700_000_000_000_000
    tod = TimeOfDayServer(rpc_server, clock_fn=lambda: t3)
    try:
        _tod_result(tod, t3 - 200)
        assert tod.stamp_mode == "kernel-midpoint"
        assert _tod_result(tod, None) == t3
        assert tod.stamp_mode == "userspace"
        assert _tod_result(tod, t3 - 200) == t3 - 100
        assert tod.stamp_mode == "kernel-midpoint"
        assert tod.implausible_rx_stamps == 0
        assert tod.call_count == 3
    finally:
        tod.close()
        rpc_server.close()


@pytest.mark.parametrize("t2,label", [
    (0, "zero — a mis-parsed or absent stamp read as a number"),
    (-5, "negative"),
    (1_700_000_000_000_001, "after t3 — a backward clock step between the reads"),
    (1_700_000_000_000_000 - 2_000_000, "2 s before t3 — a stalled/queued datagram"),
])
def test_implausible_kernel_stamp_falls_back_and_is_counted(
        fake_teensy_and_client, t2, label):
    """A t2 that cannot be a coherent RX time must not be averaged in.

    Half of a garbage t2 lands directly in the firmware's anchor, so the sane
    move is the documented fallback — and to COUNT it, because a stamp that is
    quietly discarded is the same invisibility failure as a silent probe.
    """
    _teensy, client = fake_teensy_and_client
    rpc_server = RpcServer(client)
    t3 = 1_700_000_000_000_000
    tod = TimeOfDayServer(rpc_server, clock_fn=lambda: t3)
    try:
        assert _tod_result(tod, t2) == t3, label
        assert tod.stamp_mode == "userspace"
        assert tod.implausible_rx_stamps == 1
    finally:
        tod.close()
        rpc_server.close()


# ── End-to-end through the real socket + RPC path ─────────────────────────


def test_tod_round_trip_reports_a_mode_consistent_with_the_transport(
        fake_teensy_and_client):
    """A real TIME_OF_DAY_QUERY over loopback: value sane, mode consistent.

    The mode assertion is what proves the PLUMBING: ``tod.stamp_mode`` can only
    read 'kernel-midpoint' if a kernel t2 travelled client → RpcServer →
    TimeOfDayServer for this very datagram.
    """
    teensy, client = fake_teensy_and_client
    rpc_server = RpcServer(client)
    tod = TimeOfDayServer(rpc_server)          # production clock (CLOCK_REALTIME)
    try:
        before_us = int(time.time() * 1_000_000)
        teensy.send_rpc_request(int(RpcMethod.TIME_OF_DAY_QUERY), req_id=99)
        got = teensy.wait_for(int(MsgType.RPC_RESPONSE), count=1, timeout=2.0)
        after_us = int(time.time() * 1_000_000)
        assert got, "no RPC_RESPONSE to TIME_OF_DAY_QUERY"
        head = p.RpcResponse.unpack(got[0].payload[:p.RPC_RESPONSE_SIZE])
        assert head.status == int(RpcStatus.OK) and head.res_len == 8
        blob = got[0].payload[p.RPC_RESPONSE_SIZE:p.RPC_RESPONSE_SIZE + 8]
        (wall_us,) = struct.unpack("<Q", blob)
        # The midpoint of two stamps taken inside this window is inside it too.
        assert before_us <= wall_us <= after_us
        if _LINUX_STAMPING:
            assert tod.stamp_mode == "kernel-midpoint"
            assert client.rx_stamp_mode == client_mod.RX_STAMP_KERNEL
        else:
            assert tod.stamp_mode == "userspace"
        assert tod.implausible_rx_stamps == 0
    finally:
        tod.close()
        rpc_server.close()


def test_legacy_three_argument_handler_is_unaffected(fake_teensy_and_client):
    """Handlers registered without the opt-in keep the 3-argument signature.

    ``TimeOfDayServer._handle`` is the only production handler today, but the
    registration API is generic and the hardware benches register their own; a
    4-argument call into a 3-argument handler would surface as ERR_REJECTED
    (the RpcServer catches and rewrites the TypeError), so this asserts the OK.
    """
    teensy, client = fake_teensy_and_client
    rpc_server = RpcServer(client)

    def legacy(req_id, args, addr):
        return int(RpcStatus.OK), b"legacy" + bytes([req_id])

    rpc_server.register(int(RpcMethod.NOP), legacy)
    try:
        teensy.send_rpc_request(int(RpcMethod.NOP), req_id=5)
        got = teensy.wait_for(int(MsgType.RPC_RESPONSE), count=1, timeout=2.0)
        assert got, "no RPC_RESPONSE"
        head = p.RpcResponse.unpack(got[0].payload[:p.RPC_RESPONSE_SIZE])
        assert head.status == int(RpcStatus.OK), "legacy handler was called wrongly"
        assert got[0].payload[p.RPC_RESPONSE_SIZE:] == b"legacy\x05"
    finally:
        rpc_server.close()
