"""HAND_MOVE_TO (can-bridge FW 26, additive RpcMethod) — the host half.

What is pinned here, and why each one would fail without the change:

* the method id is the GENERATED one (0x61, next after BB_FW_INFO 0x60) and the
  firmware header the can-bridge compiles carries the same id, so the two ends
  cannot drift (a hand-typed id on either side is the failure this prevents);
* the arg / result blobs have the exact packed layouts the firmware's
  ``static_assert``s pin (9 B and 20 B);
* the host outcome codes equal the firmware's ``HandMoveOutcome`` enum, read
  from the shipped ``leg_activate.h`` (two independently-written tables that
  silently disagree would mis-report SUPERSEDED as TIMEOUT, say);
* ``RpcClient.hand_move_to`` sends the args intact, returns the deferred reply's
  result, is NEVER retried (a re-dispatch would retarget its own move), maps an
  old board's ERR_UNKNOWN_METHOD to ``HandMoveToUnavailable`` and passes every
  other refusal through as ``RpcError``;
* the addition is wire-compatible: PROTOCOL_VERSION is still 9.
"""
from __future__ import annotations

import os
import re
import struct
import threading

import pytest

from teensy_link import protocol as p
from teensy_link import rpc_args
from teensy_link.rpc import (
    NON_IDEMPOTENT_METHODS,
    HandMoveToUnavailable,
    RpcClient,
    RpcError,
)

from .conftest import FakeTeensy  # noqa: F401

# teensy_link.protocol puts config/generated on sys.path; the size constants are
# not re-exported there, so read them from the generated module itself.
from udp_protocol import ARG_HAND_MOVE_TO_SIZE, RESULT_HAND_MOVE_TO_SIZE  # noqa: E402

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_FW = os.path.join(_REPO, "ros_ws", "src", "jugglebot", "Teensy_code_canbridge")

RpcMethod = p.RpcMethod
RpcStatus = p.RpcStatus


def _read(path: str) -> str:
    with open(path, encoding="utf-8") as fh:
        return fh.read()


def test_method_id_is_generated_and_shared_with_the_firmware():
    assert int(RpcMethod.HAND_MOVE_TO) == 0x61
    # Next free id after the highest pre-existing one (BB_FW_INFO).
    assert int(RpcMethod.HAND_MOVE_TO) == int(RpcMethod.BB_FW_INFO) + 1
    hdr = _read(os.path.join(_FW, "udp_protocol.h"))
    m = re.search(r"constexpr uint16_t HAND_MOVE_TO = (0x[0-9A-Fa-f]+)u;", hdr)
    assert m is not None, "can-bridge udp_protocol.h lacks HAND_MOVE_TO (regenerate)"
    assert int(m.group(1), 16) == int(RpcMethod.HAND_MOVE_TO)


def test_protocol_version_is_unchanged_by_the_additive_method():
    assert int(p.PROTOCOL_VERSION) == 9


def test_arg_blob_exact_bytes():
    blob = rpc_args.encode_hand_move_to(3.25, 1.5)
    assert blob == struct.pack("<Bff", 6, 3.25, 1.5)
    assert len(blob) == 9 == int(ARG_HAND_MOVE_TO_SIZE)
    # The encoder never pre-validates: the firmware is the one enforcement point.
    assert rpc_args.encode_hand_move_to(99.0, 9.0, axis=0) == struct.pack("<Bff", 0, 99.0, 9.0)


def test_result_blob_layout_and_roundtrip():
    blob = rpc_args.encode_hand_move_to_result(
        rpc_args.HAND_MOVE_SUPERSEDED, 2.5, -0.25, 4.0, 1234)
    assert blob == struct.pack("<B3xfffI", 1, 2.5, -0.25, 4.0, 1234)
    assert len(blob) == 20 == int(RESULT_HAND_MOVE_TO_SIZE)
    r = rpc_args.decode_hand_move_to_result(blob)
    assert (r.outcome, r.pos_rev, r.vel_rps, r.target_rev, r.elapsed_ms) == (1, 2.5, -0.25, 4.0, 1234)


def test_outcome_codes_match_the_firmware_enum():
    hdr = _read(os.path.join(_FW, "leg_activate.h"))
    fw = {name: int(val) for name, val in
          re.findall(r"^\s*HAND_MOVE_(\w+)\s*=\s*(\d+),", hdr, re.M)}
    host = {name: code for code, name in rpc_args.HAND_MOVE_OUTCOME_NAMES.items()}
    assert fw == host, (fw, host)
    assert host == {"ARRIVED": rpc_args.HAND_MOVE_ARRIVED,
                    "SUPERSEDED": rpc_args.HAND_MOVE_SUPERSEDED,
                    "TIMEOUT": rpc_args.HAND_MOVE_TIMEOUT,
                    "ABORTED": rpc_args.HAND_MOVE_ABORTED}


def test_method_map_and_non_idempotence():
    assert rpc_args.METHOD[RpcMethod.HAND_MOVE_TO] is rpc_args.ArgHandMoveTo
    assert int(RpcMethod.HAND_MOVE_TO) in NON_IDEMPOTENT_METHODS


def test_hand_move_to_sends_the_args_and_returns_the_deferred_result(fake_teensy_and_client):
    teensy, client = fake_teensy_and_client
    seen = []

    def handler(req_id, args):
        seen.append(args)
        return int(RpcStatus.OK), rpc_args.encode_hand_move_to_result(
            rpc_args.HAND_MOVE_ARRIVED, 4.001, 0.0, 4.0, 1600)
    teensy.on_rpc(int(RpcMethod.HAND_MOVE_TO), handler)
    rpc = RpcClient(client, default_timeout=0.3)
    try:
        r = rpc_args.decode_hand_move_to_result(rpc.hand_move_to(4.0, 2.5, timeout_s=1.0))
    finally:
        rpc.close()
    assert seen == [struct.pack("<Bff", 6, 4.0, 2.5)]
    assert r.outcome == rpc_args.HAND_MOVE_ARRIVED
    assert r.pos_rev == pytest.approx(4.001)


def test_hand_move_to_is_never_retried(fake_teensy_and_client):
    """The reply is deferred for seconds; a client retry inside that window would
    re-dispatch the same req_id and retarget the move it is waiting on."""
    teensy, client = fake_teensy_and_client
    calls = []
    release = threading.Event()

    def handler(req_id, args):
        calls.append(req_id)
        release.wait(0.5)              # longer than the client's timeout below
        return int(RpcStatus.OK), rpc_args.encode_hand_move_to_result(0, 0.0, 0.0, 0.0, 0)
    teensy.on_rpc(int(RpcMethod.HAND_MOVE_TO), handler)
    rpc = RpcClient(client, default_timeout=0.05, default_retries=3)
    try:
        with pytest.raises(RpcError):          # RpcTimeout is an RpcError
            rpc.hand_move_to(1.0, 2.5, timeout_s=0.1)
    finally:
        release.set()
        rpc.close()
    assert len(calls) == 1


def test_old_firmware_unknown_method_is_hand_move_to_unavailable(fake_teensy_and_client):
    _teensy, client = fake_teensy_and_client       # no handler: ERR_UNKNOWN_METHOD
    rpc = RpcClient(client, default_timeout=0.3)
    try:
        with pytest.raises(HandMoveToUnavailable) as exc:
            rpc.hand_move_to(1.0)
    finally:
        rpc.close()
    assert "HAND_MOVE_TO" in exc.value.reason
    assert isinstance(exc.value.__cause__, RpcError)


@pytest.mark.parametrize("status", ["ERR_BAD_ARGS", "ERR_BUS_DOWN", "ERR_REJECTED", "ERR_TIMEOUT"])
def test_refusals_and_aborts_pass_through_as_rpc_error(fake_teensy_and_client, status):
    teensy, client = fake_teensy_and_client
    teensy.on_rpc(int(RpcMethod.HAND_MOVE_TO),
                  lambda req_id, args: (int(getattr(RpcStatus, status)), b""))
    rpc = RpcClient(client, default_timeout=0.3)
    try:
        with pytest.raises(RpcError) as exc:
            rpc.hand_move_to(1.0)
    finally:
        rpc.close()
    assert not isinstance(exc.value, HandMoveToUnavailable)
    assert exc.value.status == int(getattr(RpcStatus, status))


def test_nowait_returns_an_in_flight_call(fake_teensy_and_client):
    teensy, client = fake_teensy_and_client
    teensy.on_rpc(int(RpcMethod.HAND_MOVE_TO),
                  lambda req_id, args: (int(RpcStatus.OK), rpc_args.encode_hand_move_to_result(
                      rpc_args.HAND_MOVE_SUPERSEDED, 1.5, -1.0, 0.0, 300)))
    rpc = RpcClient(client, default_timeout=0.3)
    try:
        call = rpc.hand_move_to_nowait(0.0, 2.0)
        assert call.method == int(RpcMethod.HAND_MOVE_TO)
        assert call.wait(1.0)
        r = rpc_args.decode_hand_move_to_result(call.result())
    finally:
        rpc.close()
    assert r.outcome == rpc_args.HAND_MOVE_SUPERSEDED
