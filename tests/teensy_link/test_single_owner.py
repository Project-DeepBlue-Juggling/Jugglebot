"""JB-82: the can-bridge UDP link is single-owner, enforced by the kernel.

The client binds its fixed ports WITHOUT ``SO_REUSEADDR``. Probed on the Jetson
2026-11-01 (logbook 2026-11-01-jb82-single-owner-link-and-stow-window): two UDP
sockets share a port only when BOTH set the option; without it the second bind is
``EADDRINUSE`` whichever binds first, and a relaunch re-binds at once after the old
owner dies (UDP has no TIME_WAIT). Every test here binds a free FIXED loopback port
pair, never 5005/5006, so it cannot collide with a live launch.
"""
from __future__ import annotations

import socket

import pytest

from teensy_link import LinkBusyError, TeensyLinkClient


def _free_ports(n: int = 2):
    socks, ports = [], []
    for _ in range(n):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("127.0.0.1", 0))
        socks.append(s)
        ports.append(s.getsockname()[1])
    for s in socks:
        s.close()
    return ports


def _client(stream: int, rpc: int) -> TeensyLinkClient:
    return TeensyLinkClient(teensy_addr=("127.0.0.1", 1), rpc_port=1, bind_host="127.0.0.1",
                            local_bind_stream=stream, local_bind_rpc=rpc)


def test_second_client_on_the_same_ports_is_refused_and_named():
    stream, rpc = _free_ports()
    first = _client(stream, rpc)
    first.start()
    try:
        second = _client(stream, rpc)
        with pytest.raises(LinkBusyError) as exc:
            second.start()
        assert ":%d" % stream in str(exc.value) and "single-owner" in str(exc.value)
        assert isinstance(exc.value, OSError)
    finally:
        first.stop()
    # a relaunch: the port is free the moment the owner lets go
    again = _client(stream, rpc)
    again.start()
    again.stop()


def test_a_foreign_socket_that_asked_to_share_still_blocks_the_client():
    """The old hazard: a program setting SO_REUSEADDR. It wins the port; we refuse, loudly."""
    stream, rpc = _free_ports()
    foreign = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    foreign.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    foreign.bind(("127.0.0.1", stream))
    try:
        with pytest.raises(LinkBusyError):
            _client(stream, rpc).start()
    finally:
        foreign.close()


def test_the_client_holding_the_port_refuses_a_foreign_sharer():
    stream, rpc = _free_ports()
    owner = _client(stream, rpc)
    owner.start()
    try:
        foreign = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        foreign.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        with pytest.raises(OSError):
            foreign.bind(("127.0.0.1", stream))
        foreign.close()
    finally:
        owner.stop()


def test_a_busy_rpc_port_does_not_leak_the_stream_socket():
    stream, rpc = _free_ports()
    foreign = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    foreign.bind(("127.0.0.1", rpc))
    try:
        with pytest.raises(LinkBusyError) as exc:
            _client(stream, rpc).start()
        assert "RPC" in str(exc.value)
    finally:
        foreign.close()
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", stream))   # would be EADDRINUSE if the half-link leaked
    probe.close()


def test_ephemeral_binds_are_unaffected():
    a, b = _client(0, 0), _client(0, 0)
    a.start()
    b.start()
    a.stop()
    b.stop()
