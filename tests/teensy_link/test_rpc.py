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
from teensy_link import rpc_args
from teensy_link.rpc import PlatformFrameWaiter


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


# ── Platform-Teensy relay reply correlation (PlatformFrameWaiter) ───────────
# Same (can_id, dlc) technique teensy_bridge_node.py uses for TILT_READ/
# STATE_READ, reused here for the Platform firmware-over-CAN update flow's
# FW_UPDATE_REPLY (0x6F1) correlation.

def test_platform_frame_waiter_receives_matching_reply(fake_teensy_and_client):
    teensy, client = fake_teensy_and_client
    waiter = PlatformFrameWaiter(client)
    try:
        can_id = 0x6F1
        waiter.clear(can_id)
        pf = p.PlatformFrame(t_bridge_us=0, can_id=can_id, dlc=8,
                             data=(1, 0, 7, 0, 5, 0, 0, 0))
        teensy.send_to_jetson(int(p.MsgType.PLATFORM_FRAME), pf.pack())
        got = waiter.wait(can_id, expected_dlc=8, timeout=1.0)
        assert got == bytes([1, 0, 7, 0, 5, 0, 0, 0])
    finally:
        waiter.close()


def test_platform_frame_waiter_clear_drops_stale_reply(fake_teensy_and_client):
    teensy, client = fake_teensy_and_client
    waiter = PlatformFrameWaiter(client)
    try:
        can_id = 0x6E0
        pf = p.PlatformFrame(t_bridge_us=0, can_id=can_id, dlc=8, data=(0,) * 8)
        teensy.send_to_jetson(int(p.MsgType.PLATFORM_FRAME), pf.pack())
        assert waiter.wait(can_id, expected_dlc=8, timeout=1.0) is not None
        waiter.clear(can_id)
        # No new frame sent — a short wait must time out, not return the stale one.
        assert waiter.wait(can_id, expected_dlc=8, timeout=0.1) is None
    finally:
        waiter.close()


# ── Platform firmware-over-CAN: reply decoder + DATA-window rewind ──────────

def test_platform_fw_reply_decoder_maps_status_to_name():
    ok_reply = struct.pack('<BBHI', 2, rpc_args.PLATFORM_FW_STATUS_OK, 41, 128)
    opcode, status, seq, detail = rpc_args.decode_platform_fw_reply(ok_reply)
    assert (opcode, seq, detail) == (2, 41, 128)
    assert rpc_args.PLATFORM_FW_STATUS_NAMES[status] == "OK"

    bad_seq_reply = struct.pack('<BBHI', 2, rpc_args.PLATFORM_FW_STATUS_BAD_SEQ, 40, 0)
    _, status, seq, _ = rpc_args.decode_platform_fw_reply(bad_seq_reply)
    assert rpc_args.PLATFORM_FW_STATUS_NAMES[status] == "BAD_SEQ"
    assert seq == 40


def test_platform_fw_data_window_rewinds_on_bad_seq():
    """One send-window boundary: the host has sent frames up to 48 (a whole
    16-frame window), and a fake BAD_SEQ reply — the sector-flush-stall NAK
    the firmware's wire contract documents — reports the board only accepted
    up to frame 40. Decoding the reply and feeding it to the rewind helper
    must land the host back at exactly the frame the board is waiting for."""
    reply = struct.pack('<BBHI', 2, rpc_args.PLATFORM_FW_STATUS_BAD_SEQ, 40, 0)
    _, status, expected_seq, _ = rpc_args.decode_platform_fw_reply(reply)
    assert status == rpc_args.PLATFORM_FW_STATUS_BAD_SEQ
    new_frame_idx = rpc_args.platform_fw_rewind_frame(current_frame_idx=48,
                                                       expected_seq=expected_seq)
    assert new_frame_idx == 40


def test_platform_fw_window_end_lands_on_an_ack_point():
    """Every DATA window must end where the board will answer: seq % 16 == 15
    or the image's last frame. Aligned starts get a full window; a start the
    board named on a BAD_SEQ rewind (2026-09-09: 2407, 2439, 2455 — all 7 mod
    16) gets a PARTIAL window up to the next ACK point, never a full one."""
    end = rpc_args.platform_fw_window_end
    assert end(0, 24576) == 16
    assert end(16, 24576) == 32
    assert end(2407, 24576) == 2416          # the sitting's own rewind seq
    assert end(2455, 24576) == 2464
    assert (end(2407, 24576) - 1) % rpc_args.PLATFORM_FW_ACK_CADENCE == 15
    assert end(24570, 24576) == 24576       # the final partial window
    assert end(24575, 24576) == 24576
    assert end(65535, 70000) == 65536       # continuous across the seq wrap


def test_platform_fw_window_reply_skips_stragglers_from_the_previous_window():
    """The reply read after a window must be THAT window's: its last frame's OK
    ACK, or a BAD_SEQ naming a frame inside it. A NAK about an older frame is a
    straggler from the previous window's duplicate burst and is skipped. The
    2026-09-09 rehearsal without this: 1362 rewinds in 51 s, two per window,
    each re-sending an already-accepted window and provoking the next burst."""
    import tools.teensy_link_bridge as tlb

    class _Waiter:
        def __init__(self, replies):
            self.replies = list(replies)
            self.cleared = 0

        def clear(self, can_id):
            self.cleared += 1

        def wait(self, can_id, expected_dlc, timeout):
            return self.replies.pop(0) if self.replies else None

    def nak(expected):
        return struct.pack('<BBHI', 2, rpc_args.PLATFORM_FW_STATUS_BAD_SEQ, expected, 0)

    def ack(seq):
        return struct.pack('<BBHI', 2, rpc_args.PLATFORM_FW_STATUS_OK, seq, 0)

    # Window [832..848): two stragglers naming 823 and 832 (older than / at
    # the previous boundary), then the real ACK for frame 847.
    w = _Waiter([nak(823), nak(831), ack(847)])
    reply = tlb._wait_for_window_reply(w, 0x6F1, 832, 848)
    _, status, seq, _ = rpc_args.decode_platform_fw_reply(reply)
    assert status == rpc_args.PLATFORM_FW_STATUS_OK and seq == 847
    assert w.cleared == 2                    # each straggler was cleared, not acted on
    # A NAK INSIDE the window is fresh and is returned at once.
    w = _Waiter([nak(840)])
    _, status, seq, _ = rpc_args.decode_platform_fw_reply(
        tlb._wait_for_window_reply(w, 0x6F1, 832, 848))
    assert status == rpc_args.PLATFORM_FW_STATUS_BAD_SEQ and seq == 840
    # A NAK naming the window's END (the board took everything, the ACK was
    # lost) is fresh too: the caller rewinds to 848, i.e. moves on.
    w = _Waiter([nak(848)])
    assert tlb._wait_for_window_reply(w, 0x6F1, 832, 848) == nak(848)
    # A refusal is always for us.
    refused = struct.pack('<BBHI', 2, rpc_args.PLATFORM_FW_STATUS_BAD_STATE, 0, 0)
    assert tlb._wait_for_window_reply(_Waiter([refused]), 0x6F1, 832, 848) == refused
    # Nothing but stragglers → None (the caller's retry path).
    assert tlb._wait_for_window_reply(_Waiter([nak(823)]), 0x6F1, 832, 848) is None


def test_platform_fw_rewind_frame_across_seq_wrap():
    """seq wraps mod 65536 (65536 % 16 == 0, so the ACK cadence is continuous
    across the wrap, per the firmware's own wire-contract note); the rewind
    must resolve correctly even when the window straddled the wrap. Host sent
    frames ...65534, 65535, 0, 1, 2, 3 (current_frame_idx=65540, absolute,
    never wraps); the board wants frame 65534 (wire seq 65534) back."""
    new_frame_idx = rpc_args.platform_fw_rewind_frame(current_frame_idx=65540,
                                                       expected_seq=65534)
    assert new_frame_idx == 65534


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


def _hts_responder(teensy, replies):
    """Serve GET_HAND_TORQUE_SCALE from a list of (state, value, reply_seq)."""
    calls = []

    def handler(req_id, args):
        i = min(len(calls), len(replies) - 1)
        calls.append(args)
        st, val, seq = replies[i]
        return int(RpcStatus.OK), rpc_args.encode_hand_torque_scale_result(st, val, seq, 5)
    teensy.on_rpc(int(RpcMethod.GET_HAND_TORQUE_SCALE), handler)
    return calls


def test_read_hand_input_torque_scale_needs_a_reply_newer_than_the_trigger(fake_teensy_and_client):
    teensy, client = fake_teensy_and_client
    # cached 100 (seq 3) before the call; the trigger is in flight; then a fresh
    # reply (seq 4) carries 1000. The stale 100 must never be returned.
    calls = _hts_responder(teensy, [(2, 100, 3), (1, 100, 3), (1, 100, 3), (2, 1000, 4)])
    rpc = RpcClient(client, default_timeout=0.3)
    try:
        assert rpc.read_hand_input_torque_scale(timeout_s=1.0, poll_s=0.001) == 1000
        assert all(a == b"" for a in calls) and len(calls) == 4
    finally:
        rpc.close()


def test_read_hand_input_torque_scale_times_out_without_a_fresh_reply(fake_teensy_and_client):
    from teensy_link.rpc import HandTorqueScaleUnavailable
    teensy, client = fake_teensy_and_client
    _hts_responder(teensy, [(2, 1000, 9)])          # only ever the pre-call cache
    rpc = RpcClient(client, default_timeout=0.3)
    try:
        with pytest.raises(HandTorqueScaleUnavailable):
            rpc.read_hand_input_torque_scale(timeout_s=0.05, poll_s=0.005)
    finally:
        rpc.close()


@pytest.mark.parametrize("state", [3, 4])
def test_read_hand_input_torque_scale_refusals_raise(fake_teensy_and_client, state):
    from teensy_link.rpc import HandTorqueScaleUnavailable
    teensy, client = fake_teensy_and_client
    _hts_responder(teensy, [(state, 0, 0)])
    rpc = RpcClient(client, default_timeout=0.3)
    try:
        with pytest.raises(HandTorqueScaleUnavailable) as exc:
            rpc.read_hand_input_torque_scale(timeout_s=0.2, poll_s=0.005)
        assert exc.value.state == state
    finally:
        rpc.close()


def test_read_hand_input_torque_scale_old_firmware_is_an_rpc_error(fake_teensy_and_client):
    teensy, client = fake_teensy_and_client          # no handler: ERR_UNKNOWN_METHOD
    rpc = RpcClient(client, default_timeout=0.2, default_retries=0)
    try:
        with pytest.raises((RpcError, RpcTimeout)):
            rpc.read_hand_input_torque_scale(timeout_s=0.1)
    finally:
        rpc.close()


# ── Ball Butler firmware-over-CAN (2026-09-28): the BB-only host behaviour ───
# The windowing / rewind / straggler logic above is target-agnostic and already
# covered; what BB adds is BEGIN's PARKING poll, the BB_FW_INFO version read and
# the per-target image checks.

class _ScriptedWaiter:
    """PlatformFrameWaiter stand-in: hands back scripted replies in order."""

    def __init__(self, replies):
        self.replies = list(replies)
        self.cleared = 0

    def clear(self, can_id):
        self.cleared += 1

    def wait(self, can_id, expected_dlc, timeout):
        return self.replies.pop(0) if self.replies else None


class _RecordingRpc:
    def __init__(self):
        self.calls = []

    def call(self, method, payload=b"", **kw):
        self.calls.append((method, payload))


def _fw_reply(op, status, seq=0, detail=0):
    return struct.pack('<BBHI', op, status, seq, detail & 0xFFFFFFFF)


def test_bb_begin_re_sends_while_parking_then_proceeds(monkeypatch):
    import tools.teensy_link_bridge as tlb
    monkeypatch.setattr(tlb, "_FW_PARK_POLL_S", 0.0)
    bb = tlb._FW_TARGETS["bb"]
    parking = _fw_reply(0x01, rpc_args.PLATFORM_FW_STATUS_PARKING, detail=6512)
    waiter = _ScriptedWaiter([parking, parking, _fw_reply(0x01, rpc_args.PLATFORM_FW_STATUS_OK)])
    rpc = _RecordingRpc()
    assert tlb._begin_session(rpc, waiter, bb, 163840) is True
    assert [m for m, _ in rpc.calls] == [int(RpcMethod.BB_FW_BEGIN)] * 3
    assert all(pl == rpc_args.encode_platform_fw_begin(163840) for _, pl in rpc.calls)


def test_bb_begin_park_failed_and_refusals_stop_the_session(monkeypatch):
    import tools.teensy_link_bridge as tlb
    monkeypatch.setattr(tlb, "_FW_PARK_POLL_S", 0.0)
    bb = tlb._FW_TARGETS["bb"]
    failed = _fw_reply(0x01, rpc_args.PLATFORM_FW_STATUS_PARK_FAILED, detail=-2150)
    assert tlb._begin_session(_RecordingRpc(), _ScriptedWaiter([failed]), bb, 100) is False
    busy = _fw_reply(0x01, rpc_args.PLATFORM_FW_STATUS_BUSY)
    rpc = _RecordingRpc()
    assert tlb._begin_session(rpc, _ScriptedWaiter([busy]), bb, 100) is False
    assert len(rpc.calls) == 1                        # a refusal is never re-polled
    assert tlb._begin_session(_RecordingRpc(), _ScriptedWaiter([]), bb, 100) is False


def test_bb_begin_gives_up_when_parking_never_ends(monkeypatch):
    import tools.teensy_link_bridge as tlb
    monkeypatch.setattr(tlb, "_FW_PARK_POLL_S", 0.0)
    monkeypatch.setattr(tlb, "_FW_PARK_TIMEOUT_S", 0.0)
    parking = _fw_reply(0x01, rpc_args.PLATFORM_FW_STATUS_PARKING)
    assert tlb._begin_session(_RecordingRpc(), _ScriptedWaiter([parking] * 5),
                              tlb._FW_TARGETS["bb"], 100) is False


def test_park_detail_is_signed_centidegrees():
    import tools.teensy_link_bridge as tlb
    assert tlb._centideg(8123) == 81.23
    assert tlb._centideg((-2150) & 0xFFFFFFFF) == -21.5


def test_bb_fw_version_read_takes_the_info_reply_and_skips_stragglers():
    import tools.teensy_link_bridge as tlb
    straggler = _fw_reply(0x02, rpc_args.PLATFORM_FW_STATUS_OK, seq=15)
    info = _fw_reply(rpc_args.FW_OP_INFO, rpc_args.PLATFORM_FW_STATUS_OK, detail=7)
    rpc = _RecordingRpc()
    assert tlb._read_bb_fw_version(rpc, _ScriptedWaiter([straggler, info])) == 7
    assert rpc.calls == [(int(RpcMethod.BB_FW_INFO), b"")]
    # An image that predates the receiver never answers → None ("unknown").
    assert tlb._read_bb_fw_version(_RecordingRpc(), _ScriptedWaiter([])) is None


def _write_hex(path, base, image):
    """Minimal Intel HEX writer (type 04 + 16-byte type 00 records + EOF)."""
    def rec(rtype, addr16, data):
        raw = bytes([len(data), addr16 >> 8, addr16 & 0xFF, rtype]) + data
        return ":" + (raw + bytes([(-sum(raw)) & 0xFF])).hex().upper()
    lines, ext = [], None
    for off in range(0, len(image), 16):
        addr = base + off
        if addr >> 16 != ext:
            ext = addr >> 16
            lines.append(rec(0x04, 0, bytes([ext >> 8, ext & 0xFF])))
        lines.append(rec(0x00, addr & 0xFFFF, image[off:off + 16]))
    lines.append(rec(0x01, 0, b""))
    path.write_text("\n".join(lines) + "\n")


def test_prepare_image_checks_the_target_marker_and_refuses_the_other_boards(tmp_path):
    import binascii
    import tools.teensy_link_bridge as tlb
    bb, plat = tlb._FW_TARGETS["bb"], tlb._FW_TARGETS["platform"]
    bb_img = b"\x00" * 64 + b"ballbutler-main" + b"\xff" * 49
    h = tmp_path / "bb.hex"
    _write_hex(h, 0x60000000, bb_img)
    image, crc = tlb._prepare_image(h, bb)
    assert image == bb_img and crc == binascii.crc32(bb_img) & 0xFFFFFFFF
    with pytest.raises(ValueError, match="jugglebot-platform"):
        tlb._prepare_image(h, plat)                    # no Platform marker
    both = tmp_path / "both.hex"
    _write_hex(both, 0x60000000, bb_img + b"jugglebot-platform")
    with pytest.raises(ValueError, match="another board"):
        tlb._prepare_image(both, bb)
    with pytest.raises(ValueError, match="another board"):
        tlb._prepare_image(both, plat)
    t41 = tmp_path / "t41.hex"
    _write_hex(t41, 0x60001000, bb_img)
    with pytest.raises(ValueError, match="base address"):
        tlb._prepare_image(t41, bb)


class _SimBbReceiver:
    """A Python model of BB's FwUpdate receiver behind the bridge's BB_FW_* RPCs:
    PARKING for the first ``park_polls`` BEGINs, strictly in-order DATA with the
    ACK-every-16th cadence and BAD_SEQ NAKs, one frame silently LOST mid-window
    (what a sector flush does to the CAN FIFO), CRC VERIFY, COMMIT, and INFO.
    Replies go out as PLATFORM_FRAMEs on 0x7D7, exactly as on_bb_rx uplinks them."""

    def __init__(self, teensy, version, park_polls=2, drop_seq=37, *, refuse_seq=None,
                 refuse_status=int(RpcStatus.ERR_BUS_DOWN), swap_seq=None,
                 drop_reply_seq=None, drop_ack_seqs=(), corrupt_seq=None):
        from config.generated import protocol_config as pc
        self.teensy, self.version = teensy, version
        self.reply_id = pc.CAN_ID_BB_FW_UPDATE_REPLY
        self.parks_left, self.drop_seq, self.dropped = park_polls, drop_seq, False
        self.image_len, self.staged, self.expect = 0, bytearray(), 0
        self.begins, self.committed, self.verifies = 0, False, 0
        # 2026-09-28 pipelining faults, each fired once: the BRIDGE refuses one
        # DATA frame (non-OK sync ack, nothing reaches CAN); one frame overtakes
        # its predecessor on the wire (the same-id mailbox reorder); the board's
        # window ACK is lost; the bridge's sync ack is lost although the frame is
        # delivered; one payload byte is flipped in transit (only VERIFY sees it).
        self.refuse_seq, self.refuse_status, self.refused = refuse_seq, refuse_status, False
        self.swap_seq, self.held = swap_seq, None
        self.drop_reply_seq, self.reply_dropped = drop_reply_seq, False
        self.drop_ack_seqs, self.acks_dropped = set(drop_ack_seqs), 0
        self.corrupt_seq = corrupt_seq
        self.data_frames = 0
        ok = lambda: (int(RpcStatus.OK), b"")     # noqa: E731 — the bridge's sync ack
        for method, fn in ((RpcMethod.BB_FW_BEGIN, self._begin),
                           (RpcMethod.BB_FW_VERIFY, self._verify), (RpcMethod.BB_FW_COMMIT, self._commit),
                           (RpcMethod.BB_FW_INFO, self._info)):
            teensy.on_rpc(int(method), lambda req_id, args, fn=fn: (fn(args), ok())[1])
        teensy.on_rpc(int(RpcMethod.BB_FW_DATA), lambda req_id, args: self._data_rpc(args))
        orig = teensy._maybe_respond

        def _maybe_respond(payload, addr):
            req = p.RpcRequest.unpack(payload[: p.RPC_REQUEST_SIZE])
            args = payload[p.RPC_REQUEST_SIZE: p.RPC_REQUEST_SIZE + req.arg_len]
            if int(req.method) == int(RpcMethod.BB_FW_DATA):
                (seq,) = struct.unpack('<H', args[:2])
                if seq in self.drop_ack_seqs:
                    self.drop_ack_seqs.discard(seq)
                    self.acks_dropped += 1
                    self._data_rpc(args)              # delivered; only the ack is lost
                    return
            orig(payload, addr)
        teensy._maybe_respond = _maybe_respond

    def _data_rpc(self, args):
        (seq,) = struct.unpack('<H', args[:2])
        if seq == self.refuse_seq and not self.refused:
            self.refused = True
            return self.refuse_status, b""            # the bridge refused: no CAN frame
        if seq == self.swap_seq and self.held is None:
            self.held = args                          # overtaken: arrives after the next one
            return int(RpcStatus.OK), b""
        self._data(args)
        if self.held is not None and seq != self.swap_seq:
            held, self.held, self.swap_seq = self.held, None, None
            self._data(held)
        return int(RpcStatus.OK), b""

    def _reply(self, op, status, seq=0, detail=0):
        data = struct.pack('<BBHI', op, status, seq & 0xFFFF, detail & 0xFFFFFFFF)
        pf = p.PlatformFrame(t_bridge_us=0, can_id=self.reply_id, dlc=8, data=tuple(data))
        self.teensy.send_to_jetson(int(p.MsgType.PLATFORM_FRAME), pf.pack())

    def _begin(self, args):
        self.begins += 1
        if self.parks_left:
            self.parks_left -= 1
            self._reply(0x01, rpc_args.PLATFORM_FW_STATUS_PARKING, detail=6012)
            return
        (self.image_len,) = struct.unpack('<I', args[:4])
        self.staged, self.expect = bytearray(), 0
        self._reply(0x01, rpc_args.PLATFORM_FW_STATUS_OK)

    def _data(self, args):
        self.data_frames += 1
        seq, n = struct.unpack('<HB', args[:3])
        if seq == self.drop_seq and not self.dropped:
            self.dropped = True                       # lost on the wire: no reply at all
            return
        if seq != (self.expect & 0xFFFF):
            self._reply(0x02, rpc_args.PLATFORM_FW_STATUS_BAD_SEQ, self.expect, len(self.staged))
            return
        payload = bytearray(args[3:3 + n])
        if seq == self.corrupt_seq:
            payload[0] ^= 0x01
        self.staged += payload
        self.expect += 1
        if seq % 16 == 15 or len(self.staged) == self.image_len:
            if seq == self.drop_reply_seq and not self.reply_dropped:
                self.reply_dropped = True             # the board's ACK lost on the way back
                return
            self._reply(0x02, rpc_args.PLATFORM_FW_STATUS_OK, seq, len(self.staged))

    def _verify(self, args):
        import binascii
        self.verifies += 1
        (want,) = struct.unpack('<I', args[:4])
        got = binascii.crc32(bytes(self.staged)) & 0xFFFFFFFF
        status = rpc_args.PLATFORM_FW_STATUS_OK if got == want else rpc_args.PLATFORM_FW_STATUS_BAD_CRC
        self._reply(0x03, status, self.expect - 1, got)

    def _commit(self, args):
        self.committed = True
        self.version += 1                             # "reboots" into the new image
        self._reply(0x04, rpc_args.PLATFORM_FW_STATUS_OK)

    def _info(self, args):
        self._reply(rpc_args.FW_OP_INFO, rpc_args.PLATFORM_FW_STATUS_OK, detail=self.version)


def test_bb_fw_update_end_to_end_over_the_loopback_link(fake_teensy_and_client, monkeypatch):
    """The whole BB session through the REAL RpcClient + PlatformFrameWaiter on a
    UDP loopback: INFO before, BEGIN through two PARKING polls, DATA with a lost
    frame recovered by the BAD_SEQ rewind, VERIFY, COMMIT, INFO after."""
    import binascii
    import tools.teensy_link_bridge as tlb
    from teensy_link.rpc import RpcClient
    monkeypatch.setattr(tlb, "_FW_PARK_POLL_S", 0.0)
    monkeypatch.setattr(tlb, "_FW_SECTOR_FLUSH_PAUSE_S", 0.0)
    teensy, client = fake_teensy_and_client
    sim = _SimBbReceiver(teensy, version=1)
    image = bytes((i * 37 + 11) & 0xFF for i in range(3001))   # 601 frames, a 1-byte tail
    crc = binascii.crc32(image) & 0xFFFFFFFF
    bb = tlb._FW_TARGETS["bb"]
    rpc = RpcClient(client, default_timeout=0.5, default_retries=1)
    waiter = PlatformFrameWaiter(client)
    try:
        assert tlb._read_bb_fw_version(rpc, waiter) == 1
        assert tlb._run_fw_update_session(rpc, waiter, bb.reply_id, image, crc,
                                          commit=True, target=bb) == 0
        assert sim.begins == 3                        # two PARKING polls, then OK
        assert sim.dropped                            # the lost frame really happened ...
        assert bytes(sim.staged) == image             # ... and the rewind recovered it
        assert sim.committed
        assert tlb._read_bb_fw_version(rpc, waiter) == 2
    finally:
        waiter.close()
        rpc.close()


# ── Pipelined DATA (2026-09-28): send_nowait + _DataSender ──────────────────
# The bridge services RPCs in a 1 kHz task, so a synchronous DATA RPC per frame
# costs ~1 ms (61 s for BB's 163840 B image). A pipelined sender was built and
# FAILED on hardware: the bridge's RPC socket queues ONE packet, so requests
# sharing a tick are dropped. Can-bridge FW 25 (skill-stack) raised that queue to 8, so BB now
# runs depth 4 (gated on bridge FW >= 25 in skill-stack numbering; the Platform stays at depth 1). The
# tests below hold the pipelined mechanism correct against a sim. The board's window
# ACK / BAD_SEQ stays the only delivery proof, and every refusal still surfaces.

def test_send_nowait_collects_result_and_refusal_later(fake_teensy_and_client):
    teensy, client = fake_teensy_and_client
    teensy.on_rpc(int(RpcMethod.NOP), lambda req_id, args: (int(RpcStatus.OK), args))
    teensy.on_rpc(int(RpcMethod.SET_AXIS_STATE),
                  lambda req_id, args: (int(RpcStatus.ERR_REJECTED), b""))
    rpc = RpcClient(client, default_timeout=0.3)
    try:
        calls = [rpc.send_nowait(int(RpcMethod.NOP), bytes([i])) for i in range(6)]
        assert len({c.req_id for c in calls}) == 6
        for i, c in enumerate(calls):
            assert c.wait(1.0)
            assert c.result() == bytes([i])
        refused = rpc.send_nowait(int(RpcMethod.SET_AXIS_STATE), b"\x00")
        assert refused.wait(1.0)
        with pytest.raises(RpcError) as exc:
            refused.result()
        assert exc.value.status == int(RpcStatus.ERR_REJECTED)
        assert rpc._pending == {}                   # result() released every entry
    finally:
        rpc.close()


def test_send_nowait_sends_once_and_times_out_without_retry():
    """No response: result() raises RpcTimeout, release() clears the entry, and
    the request went out exactly ONCE — so the path is safe for non-idempotent
    methods too (it never re-dispatches)."""
    from teensy_link import TeensyLinkClient
    nowhere = TeensyLinkClient(teensy_addr=("127.0.0.1", 1), rpc_port=1,
                               local_bind_stream=0, local_bind_rpc=0, bind_host="127.0.0.1")
    nowhere.start()
    sends = []
    orig = nowhere.send_rpc
    nowhere.send_rpc = lambda mt, payload: (sends.append(payload), orig(mt, payload))[1]
    rpc = RpcClient(nowhere, default_timeout=0.05, default_retries=3)
    try:
        c = rpc.send_nowait(int(RpcMethod.BB_THROW), b"\x00" * 4)
        assert not c.wait(0.1)
        with pytest.raises(RpcTimeout):
            c.result()
        assert len(sends) == 1
        assert rpc._pending == {}
        c2 = rpc.send_nowait(int(RpcMethod.NOP))
        c2.release()
        c2.release()                                # idempotent
        assert rpc._pending == {}
    finally:
        rpc.close()
        nowhere.stop()


class _DepthProbeRpc(RpcClient):
    """RpcClient that records how many requests were outstanding at each send."""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.max_outstanding = 0
        self.sync_calls = 0

    def send_nowait(self, method, args=b""):
        with self._pending_lock:
            self.max_outstanding = max(self.max_outstanding, len(self._pending) + 1)
        return super().send_nowait(method, args)

    def call(self, method, args=b"", **kw):
        if int(method) == int(RpcMethod.BB_FW_DATA):
            self.sync_calls += 1
        return super().call(method, args, **kw)


def _bb_session(fake_teensy_and_client, monkeypatch, image, sim_kwargs=None, **target_kw):
    import binascii
    import dataclasses
    import tools.teensy_link_bridge as tlb
    monkeypatch.setattr(tlb, "_FW_PARK_POLL_S", 0.0)
    # Short enough to keep the lost-ACK cases quick, long enough that a loaded
    # xdist worker's loopback never fakes a missing ack or a window timeout.
    monkeypatch.setattr(tlb, "_FW_REPLY_TIMEOUT_S", 0.5)
    monkeypatch.setattr(tlb, "_FW_ACK_TIMEOUT_S", 0.25)
    teensy, client = fake_teensy_and_client
    sim = _SimBbReceiver(teensy, version=1, park_polls=0, **(sim_kwargs or {"drop_seq": None}))
    target_kw.setdefault("data_pipeline_depth", 4)   # the mechanism under test
    target = dataclasses.replace(tlb._FW_TARGETS["bb"], sector_pause_s=0.0, **target_kw)
    rpc = _DepthProbeRpc(client, default_timeout=0.5, default_retries=1)
    waiter = PlatformFrameWaiter(client)
    crc = binascii.crc32(image) & 0xFFFFFFFF
    try:
        try:
            rc = tlb._run_fw_update_session(rpc, waiter, target.reply_id, image, crc,
                                            commit=True, target=target)
        except RpcError as e:                        # RpcTimeout included
            rc = e
        return sim, rpc, rc
    finally:
        waiter.close()
        rpc.close()


_IMG = bytes((i * 37 + 11) & 0xFF for i in range(9001))   # 1801 frames, crosses two sectors


def test_fw_profiles_bb_pipelined_on_bridge_fw25_platform_as_flown():
    """BB flies the reduced sector pause (0.12 s) and, since can-bridge FW 25
    (skill-stack numbering; FW 22 on mvp-trajectory-bringup)
    deepened the RPC socket's receive queue to 8, a pipelined DATA window of
    depth 4. The Platform keeps exactly what it flew with: 0.5 s, depth 1."""
    import tools.teensy_link_bridge as tlb
    bb, plat = tlb._FW_TARGETS["bb"], tlb._FW_TARGETS["platform"]
    assert tlb._FW_PIPELINE_DEPTH == 4
    assert tlb._FW_PIPELINE_MIN_BRIDGE_FW == 25
    assert bb.data_pipeline_depth == 4 and plat.data_pipeline_depth == 1
    assert bb.sector_pause_s == tlb._FW_SECTOR_FLUSH_PAUSE_S
    assert 0.1 <= bb.sector_pause_s <= 0.15             # > typical ~52 ms flush, < flown 0.5 s
    assert plat.sector_pause_s == 0.5                   # the Platform stays as flown


def test_pipelining_falls_back_to_synchronous_against_an_older_or_unheard_bridge():
    """Depth > 1 against a bridge whose RPC socket holds ONE packet loses frames
    (hardware, 2026-09-28, FW 21), so the tool drops to depth 1 unless the bridge
    reports FW >= 25 — never a pipelined window at a bridge that cannot queue it.
    skill-stack's FW 22-24 predate the queue fix, so they must fall back too."""
    import tools.teensy_link_bridge as tlb
    bb, plat = tlb._FW_TARGETS["bb"], tlb._FW_TARGETS["platform"]
    assert tlb._effective_target(bb, 25).data_pipeline_depth == 4
    assert tlb._effective_target(bb, 26).data_pipeline_depth == 4
    assert tlb._effective_target(bb, 24).data_pipeline_depth == 1
    assert tlb._effective_target(bb, 22).data_pipeline_depth == 1
    assert tlb._effective_target(bb, 21).data_pipeline_depth == 1
    assert tlb._effective_target(bb, None).data_pipeline_depth == 1
    assert tlb._effective_target(bb, 21).sector_pause_s == bb.sector_pause_s
    assert tlb._effective_target(plat, 22) is plat      # depth 1 is never raised


def test_bb_pipelined_session_bounds_in_flight_and_recovers_a_lost_frame(
        fake_teensy_and_client, monkeypatch):
    sim, rpc, rc = _bb_session(fake_teensy_and_client, monkeypatch, _IMG,
                               {"drop_seq": 1234})
    assert rc == 0
    assert sim.dropped and bytes(sim.staged) == _IMG and sim.committed
    assert rpc.sync_calls == 0                          # every DATA frame pipelined ...
    assert 2 <= rpc.max_outstanding <= 4                # ... never more than depth in flight


def test_bb_pipelined_depth_one_is_the_synchronous_path(fake_teensy_and_client, monkeypatch):
    sim, rpc, rc = _bb_session(fake_teensy_and_client, monkeypatch, _IMG[:3001],
                               {"drop_seq": 37}, data_pipeline_depth=1)
    assert rc == 0 and bytes(sim.staged) == _IMG[:3001]
    assert rpc.max_outstanding == 0 and rpc.sync_calls >= 601


def test_bb_pipelined_reorder_is_a_rewind(fake_teensy_and_client, monkeypatch):
    """A frame overtaken on the wire (the same-id mailbox reorder) is NAKed
    BAD_SEQ and rewound — never staged out of order."""
    sim, rpc, rc = _bb_session(fake_teensy_and_client, monkeypatch, _IMG,
                               {"drop_seq": None, "swap_seq": 500})
    assert rc == 0 and bytes(sim.staged) == _IMG and sim.committed


def test_bb_pipelined_lost_window_ack_retries_the_window(fake_teensy_and_client, monkeypatch):
    """The board took the whole window but its ACK was lost: the window times
    out, is re-sent, and the board's BAD_SEQ naming the window END moves on."""
    sim, rpc, rc = _bb_session(fake_teensy_and_client, monkeypatch, _IMG,
                               {"drop_seq": None, "drop_reply_seq": 815})
    assert rc == 0 and sim.reply_dropped
    assert bytes(sim.staged) == _IMG and sim.committed


def test_bb_pipelined_lost_bridge_ack_is_tolerated(fake_teensy_and_client, monkeypatch):
    """A lost SYNC ack (frame delivered) is counted, not fatal: the board's
    window reply is the delivery proof."""
    sim, rpc, rc = _bb_session(fake_teensy_and_client, monkeypatch, _IMG,
                               {"drop_seq": None, "drop_ack_seqs": (100, 900)})
    assert rc == 0 and sim.acks_dropped == 2
    assert bytes(sim.staged) == _IMG and sim.committed


@pytest.mark.parametrize("status", [RpcStatus.ERR_BUS_DOWN, RpcStatus.ERR_REJECTED,
                                    RpcStatus.ERR_UNKNOWN_METHOD, RpcStatus.ERR_TIMEOUT])
def test_bb_pipelined_bridge_refusal_surfaces_and_never_commits(
        fake_teensy_and_client, monkeypatch, status):
    sim, rpc, rc = _bb_session(fake_teensy_and_client, monkeypatch, _IMG,
                               {"drop_seq": None, "refuse_seq": 700, "refuse_status": int(status)})
    assert isinstance(rc, RpcError) and rc.status == int(status)
    assert sim.refused
    assert sim.verifies == 0 and not sim.committed
    assert rpc._pending == {}                           # the window's calls were released


def test_bb_pipelined_dead_link_aborts_as_timeout(fake_teensy_and_client, monkeypatch):
    """Every ack missing (the bridge gone mid-transfer) aborts as RpcTimeout
    after _FW_MAX_MISSING_ACKS + 1, never reaching VERIFY or COMMIT."""
    sim, rpc, rc = _bb_session(fake_teensy_and_client, monkeypatch, _IMG,
                               {"drop_seq": None, "drop_ack_seqs": range(48, 1801)})
    assert isinstance(rc, RpcTimeout)
    assert sim.verifies == 0 and not sim.committed


def test_bb_pipelined_corruption_fails_verify_and_never_commits(fake_teensy_and_client, monkeypatch):
    sim, rpc, rc = _bb_session(fake_teensy_and_client, monkeypatch, _IMG,
                               {"drop_seq": None, "corrupt_seq": 999})
    assert rc == 1
    assert sim.verifies == 1 and not sim.committed
