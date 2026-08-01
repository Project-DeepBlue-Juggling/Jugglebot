"""Tests for the RPC layer: client (J→T) and server (T→J)."""

from __future__ import annotations

import struct
import time

import pytest

from teensy_link import (
    RpcClient,
    RpcServer,
    RpcError,
    RpcTimeout,
    RpcMethod,
    RpcStatus,
    TimeOfDayServer,
)
from teensy_link import protocol as p


# Import the fake-teensy fixture
from .conftest import FakeTeensy  # noqa: F401


def test_outgoing_rpc_round_trip(fake_teensy_and_client):
    teensy, client = fake_teensy_and_client
    # Register an auto-responder on the FakeTeensy side
    teensy.on_rpc(int(RpcMethod.NOP), lambda req_id, args: (int(RpcStatus.OK), b"pong"))
    rpc = RpcClient(client, default_timeout=0.3)
    try:
        result = rpc.call(int(RpcMethod.NOP), b"ping")
        assert result == b"pong"
    finally:
        rpc.close()


def test_rpc_error_status_raises(fake_teensy_and_client):
    teensy, client = fake_teensy_and_client
    teensy.on_rpc(int(RpcMethod.SET_AXIS_STATE),
                  lambda req_id, args: (int(RpcStatus.ERR_BAD_ARGS), b""))
    rpc = RpcClient(client, default_timeout=0.3)
    try:
        with pytest.raises(RpcError) as exc:
            rpc.call(int(RpcMethod.SET_AXIS_STATE), b"\x00")
        assert exc.value.status == int(RpcStatus.ERR_BAD_ARGS)
    finally:
        rpc.close()


def test_rpc_timeout_after_retries(fake_teensy_and_client):
    teensy, client = fake_teensy_and_client
    # No handler registered → FakeTeensy returns ERR_UNKNOWN_METHOD, NOT a timeout.
    # Force timeout by registering a handler that never responds:
    #   The FakeTeensy's auto-responder always replies, so to test timeout we
    #   point the client at a port nothing listens on.
    from teensy_link import TeensyLinkClient

    nowhere = TeensyLinkClient(
        teensy_addr=("127.0.0.1", 1),  # nothing should be on port 1
        rpc_port=1,
        local_bind_stream=0,
        local_bind_rpc=0,
        bind_host="127.0.0.1",
    )
    nowhere.start()
    try:
        rpc = RpcClient(nowhere, default_timeout=0.05, default_retries=1)
        try:
            with pytest.raises(RpcTimeout) as exc:
                rpc.call(int(RpcMethod.NOP))
            assert exc.value.retries == 1
        finally:
            rpc.close()
    finally:
        nowhere.stop()


def test_concurrent_rpcs_correlate_by_req_id(fake_teensy_and_client):
    """Two RPCs in flight at once should each get the right response."""
    teensy, client = fake_teensy_and_client

    # Handler that echoes the req_id in the result so we can verify correlation
    def handler(req_id, args):
        return int(RpcStatus.OK), struct.pack("<H", req_id) + args

    teensy.on_rpc(int(RpcMethod.NOP), handler)
    rpc = RpcClient(client, default_timeout=0.3)
    try:
        import threading
        results = {}

        def call(arg_byte):
            res = rpc.call(int(RpcMethod.NOP), bytes([arg_byte]))
            req_id = struct.unpack_from("<H", res, 0)[0]
            results[arg_byte] = (req_id, res[2:])

        threads = [threading.Thread(target=call, args=(i,)) for i in range(8)]
        for t in threads: t.start()
        for t in threads: t.join(timeout=2.0)
        # Each call should have completed and each response should match its arg
        for i in range(8):
            assert i in results, f"call {i} never returned"
            _, blob = results[i]
            assert blob == bytes([i]), f"call {i} got wrong echo: {blob!r}"
    finally:
        rpc.close()


# ── Incoming RPC (TimeOfDayServer) ────────────────────────────────────────


def test_time_of_day_server_responds_to_query(fake_teensy_and_client):
    teensy, client = fake_teensy_and_client
    rpc_server = RpcServer(client)
    tod = TimeOfDayServer(rpc_server, clock_fn=lambda: 987_654_321)
    try:
        teensy.send_rpc_request(int(RpcMethod.TIME_OF_DAY_QUERY), req_id=42)
        # Wait for the response back on the FakeTeensy
        deadline = time.time() + 0.5
        responses = []
        while time.time() < deadline:
            responses = teensy.received(int(p.MsgType.RPC_RESPONSE))
            if responses:
                break
            time.sleep(0.01)
        assert len(responses) == 1
        resp_head = p.RpcResponse.unpack(responses[0].payload[:p.RPC_RESPONSE_SIZE])
        assert resp_head.method == int(RpcMethod.TIME_OF_DAY_QUERY)
        assert resp_head.req_id == 42
        assert resp_head.status == int(RpcStatus.OK)
        assert resp_head.res_len == 8
        result_blob = responses[0].payload[p.RPC_RESPONSE_SIZE:p.RPC_RESPONSE_SIZE + 8]
        wall_us = struct.unpack("<Q", result_blob)[0]
        assert wall_us == 987_654_321
        assert tod.call_count == 1
    finally:
        tod.close()
        rpc_server.close()


def test_unregistered_inbound_rpc_returns_unknown_method(fake_teensy_and_client):
    teensy, client = fake_teensy_and_client
    rpc_server = RpcServer(client)  # no handlers registered
    try:
        teensy.send_rpc_request(int(RpcMethod.NOP), req_id=1)
        deadline = time.time() + 0.5
        responses = []
        while time.time() < deadline:
            responses = teensy.received(int(p.MsgType.RPC_RESPONSE))
            if responses:
                break
            time.sleep(0.01)
        assert len(responses) == 1
        resp_head = p.RpcResponse.unpack(responses[0].payload[:p.RPC_RESPONSE_SIZE])
        assert resp_head.status == int(RpcStatus.ERR_UNKNOWN_METHOD)
    finally:
        rpc_server.close()


def test_inbound_rpc_handler_exception_returns_rejected(fake_teensy_and_client):
    teensy, client = fake_teensy_and_client
    rpc_server = RpcServer(client)

    def bad_handler(req_id, args, addr):
        raise RuntimeError("intentional test failure")

    rpc_server.register(int(RpcMethod.NOP), bad_handler)
    try:
        teensy.send_rpc_request(int(RpcMethod.NOP), req_id=1)
        deadline = time.time() + 0.5
        responses = []
        while time.time() < deadline:
            responses = teensy.received(int(p.MsgType.RPC_RESPONSE))
            if responses:
                break
            time.sleep(0.01)
        assert len(responses) == 1
        resp_head = p.RpcResponse.unpack(responses[0].payload[:p.RPC_RESPONSE_SIZE])
        assert resp_head.status == int(RpcStatus.ERR_REJECTED)
    finally:
        rpc_server.close()


# ── [8] non-idempotent methods are never re-dispatched (firmware has no dedup) ──

def test_non_idempotent_methods_not_retried(monkeypatch):
    """A lost RESPONSE must not re-dispatch a non-idempotent op. call() forces
    retries=0 for NON_IDEMPOTENT_METHODS even if the caller asks for retries, while
    an idempotent method still honours retries. We count RPC_REQUEST sends against a
    dead peer (guaranteed timeout) — HOME sends exactly ONE, SET_POS_GAIN sends
    1 + retries."""
    from teensy_link import TeensyLinkClient
    from teensy_link import rpc as rpc_mod

    nowhere = TeensyLinkClient(
        teensy_addr=("127.0.0.1", 1), rpc_port=1,
        local_bind_stream=0, local_bind_rpc=0, bind_host="127.0.0.1")
    nowhere.start()
    try:
        rpc = RpcClient(nowhere, default_timeout=0.02, default_retries=2)
        sent = {'n': 0}
        orig_send = nowhere.send_rpc

        def counting_send(msg_type, packet):
            sent['n'] += 1
            return orig_send(msg_type, packet)
        monkeypatch.setattr(nowhere, 'send_rpc', counting_send)

        assert int(RpcMethod.HOME) in rpc_mod.NON_IDEMPOTENT_METHODS
        assert int(RpcMethod.SET_POS_GAIN) not in rpc_mod.NON_IDEMPOTENT_METHODS

        # Idempotent: retries honoured → 1 + 2 = 3 sends.
        sent['n'] = 0
        with pytest.raises(RpcTimeout):
            rpc.call(int(RpcMethod.SET_POS_GAIN), b"\x00" * 8, retries=2)
        assert sent['n'] == 3

        # Non-idempotent: retries FORCED to 0 → exactly ONE send, even asking for 2.
        sent['n'] = 0
        with pytest.raises(RpcTimeout):
            rpc.call(int(RpcMethod.HOME), b"\x00", retries=2)
        assert sent['n'] == 1
        try:
            rpc.close()
        finally:
            pass
    finally:
        nowhere.stop()
