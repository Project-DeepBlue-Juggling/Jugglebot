"""Host-side contract for the electronic-clapboard downlink (``CLAP_SEND``).

WHAT THIS GUARDS
----------------
``CLAP_SEND`` (RpcMethod ``0x0060``) carries a **whole slate transaction** in one
request: a leading count plus three fixed-width parallel arrays, which the
can-bridge enqueues and then drains onto the cone bus **one frame per tick**.

Three properties matter here and are checked below:

1. **Byte layout.** ``ArgClapSend`` is structure-of-arrays, not an array of frame
   structs, because the generator cannot express one (``Field.count`` multiplies
   a *single* scalar type, and there is no variable-length ``RpcArg`` support at
   all).  The firmware consumes the *same* generated struct via
   ``using JbUdp::RpcArgs::ArgClapSend``, so a mismatch here is a mismatch on the
   wire.  It must also fit inside one RPC request.
2. **Order is the payload.** Chunks may arrive in any order, but ``CLAP_COMMIT``
   must arrive **after** its chunks or the clapboard answers ``INCOMPLETE``.  The
   bridge does no ordering logic at all — one burst, drained FIFO, guarantees it
   *by construction* provided the caller puts the commit last.  So the encoder
   must preserve caller order exactly, and this file pins that.
3. **``CLAP_SEND`` is NON-IDEMPOTENT.**  Retrying after a lost *response* would
   push a second copy of every chunk into a transaction the clapboard is already
   reassembling — duplicate chunks under one ``txn_id``, answered as
   ``CRC_MISMATCH`` or a wrongly-painted slate.  ``RpcClient.call`` forces
   ``retries=0`` for members of ``NON_IDEMPOTENT_METHODS``.

Pure stdlib + the pure host package.  No ROS 2, no compile, no hardware.
"""

from __future__ import annotations

import struct

import pytest

from teensy_link import protocol as p
from teensy_link import rpc as rpc_mod
from teensy_link import rpc_args as ra

# The generated module directly, for the per-arg ``*_SIZE`` / ``*_FMT`` constants:
# teensy_link/protocol.py deliberately re-exports NO ``ARG_*_SIZE`` (it is a
# curated facade, not a mirror), so this is the same route test_rpc_dispatch_lint
# takes.  ``config/generated`` is on sys.path via tests/conftest.py.
import udp_protocol as _gen


# ── 1. Byte layout ────────────────────────────────────────────────────────────

def test_arg_clap_send_layout_and_budget():
    """``1 + 13*CLAP_MAX_FRAMES`` bytes, and it fits one RPC request.

    The 625 B figure is not arbitrary: ``CLAP_MAX_FRAMES = 48`` is sized from the
    worst-case transaction (8 fields x 5 chunks + 1 commit = 41 frames) plus
    headroom, and being FIXED-size is what lets the firmware's existing
    fixed-size ``take()`` helper consume it unmodified.
    """
    assert p.CLAP_MAX_FRAMES == 48
    expected = 1 + 4 * p.CLAP_MAX_FRAMES + 1 * p.CLAP_MAX_FRAMES + 8 * p.CLAP_MAX_FRAMES
    assert expected == 625
    assert _gen.ARG_CLAP_SEND_SIZE == expected
    assert struct.calcsize(_gen.ARG_CLAP_SEND_FMT) == expected
    # Must fit a single RPC request: the fixed head plus the arg blob.
    assert expected <= p.MAX_PAYLOAD - p.RPC_REQUEST_SIZE


def test_encoded_burst_round_trips_field_for_field():
    frames = [
        (int(p_id), payload)
        for p_id, payload in (
            (0x7E8, b"\x01scene1"),
            (0x7E8, b"\x11 take7"),
            (0x7E9, b"\x03\x07\x00\x2a\xb1\x29\x00\x00"),   # commit LAST
        )
    ]
    blob = ra.encode_clap_send(frames)
    assert len(blob) == _gen.ARG_CLAP_SEND_SIZE

    a = p.ArgClapSend.unpack(blob)
    assert a.count == 3
    for i, (can_id, payload) in enumerate(frames):
        assert a.can_id[i] == can_id
        assert a.len[i] == len(payload)
        assert bytes(a.data[i * 8:i * 8 + len(payload)]) == payload
    # Unused slots are zero, so a firmware that read past `count` would read
    # zeros rather than a stale previous transaction.
    assert set(a.can_id[3:]) == {0}
    assert set(a.len[3:]) == {0}
    assert set(a.data[24:]) == {0}


def test_caller_order_is_preserved_exactly():
    """The commit must be able to land LAST, which needs order preservation.

    The bridge drains FIFO with no ordering logic; if the encoder ever sorted,
    grouped or de-duplicated frames, an out-of-order commit would reach the
    clapboard as INCOMPLETE and the cause would be three layers away.
    """
    ids = [0x7E8] * 40 + [0x7E9]
    frames = [(ids[i], bytes([i]) + b"\x00" * 7) for i in range(41)]
    a = p.ArgClapSend.unpack(ra.encode_clap_send(frames))
    assert a.count == 41
    assert [a.data[i * 8] for i in range(41)] == list(range(41))
    assert a.can_id[40] == 0x7E9                  # the commit, last
    assert set(a.can_id[:40]) == {0x7E8}


def test_worst_case_transaction_fits():
    """8 fields x 5 chunks + 1 commit = 41 frames, the number CLAP_MAX_FRAMES was
    sized from, must encode without complaint — with headroom left over."""
    frames = [(0x7E8, b"\x00" * 8)] * 41
    assert len(ra.encode_clap_send(frames)) == _gen.ARG_CLAP_SEND_SIZE
    assert 41 < p.CLAP_MAX_FRAMES


# ── 2. Validation at the encode boundary ──────────────────────────────────────

def test_empty_burst_is_rejected():
    with pytest.raises(ValueError):
        ra.encode_clap_send([])


def test_burst_longer_than_the_struct_is_rejected():
    frames = [(0x7E8, b"\x00" * 8)] * (p.CLAP_MAX_FRAMES + 1)
    with pytest.raises(ValueError):
        ra.encode_clap_send(frames)
    # Exactly at the cap is fine.
    assert ra.encode_clap_send(frames[:p.CLAP_MAX_FRAMES])


def test_over_long_payload_is_rejected():
    with pytest.raises(ValueError):
        ra.encode_clap_send([(0x7E8, b"\x00" * 9)])


def test_a_short_payload_keeps_its_honest_dlc():
    """A short payload is zero-padded but its DLC is reported as its own length.

    The clapboard requires exactly 8 and SILENTLY DROPS anything else, so quietly
    promoting a short frame to DLC 8 here would hide the bug at the one layer
    that could still name it — and the symptom is a panel stuck in screensaver
    with no error on either side.  Emitting DLC 8 is the frame BUILDER's job.
    """
    a = p.ArgClapSend.unpack(ra.encode_clap_send([(0x7E8, b"\x01\x02\x03")]))
    assert a.len[0] == 3
    assert bytes(a.data[:8]) == b"\x01\x02\x03\x00\x00\x00\x00\x00"


# ── 3. Method wiring ──────────────────────────────────────────────────────────

def test_clap_send_is_registered_as_arg_carrying():
    """``ra.METHOD`` must map CLAP_SEND to ArgClapSend.

    ``test_rpc_args.py``'s full-partition freeze fails if a new arg-carrying
    method is missing here, so this is the positive statement of the same fact.
    """
    assert ra.METHOD[p.RpcMethod.CLAP_SEND] is p.ArgClapSend


def test_clap_send_is_non_idempotent():
    """Retries forced to 0 — a re-dispatch duplicates chunks into a transaction
    the clapboard is already reassembling."""
    assert int(p.RpcMethod.CLAP_SEND) in rpc_mod.NON_IDEMPOTENT_METHODS
    # Sanity: an ordinary config write is still retryable.
    assert int(p.RpcMethod.SET_POS_GAIN) not in rpc_mod.NON_IDEMPOTENT_METHODS


def test_clap_send_opcode_is_the_allocated_one():
    """0x0060, the next free block after HAND_TRAJ_CMD 0x0054.

    The id is a cross-repo allocation; a renumber here would leave the peer's
    handoff brief pointing at a method the bridge no longer answers.
    """
    assert int(p.RpcMethod.CLAP_SEND) == 0x0060
