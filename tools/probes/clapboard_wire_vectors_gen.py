#!/usr/bin/env python3
"""Derive the shared clapboard wire fixture from the SPEC, not from our encoders.

Emits ``tests/fixtures/clapboard_wire_vectors.json`` — the cross-repository
contract artefact this repo owns and the Electronic-Clapboard repo vendors
(``plans/active/clapboard-can3-integration.md`` Phase 4, owner decision
2026-08-16; logbook ``2026-08-16-clapboard-phase4-cross-repo-contract.md``).

WHY THIS IS A COMMITTED RECIPE
------------------------------
``tools/probes/README.md`` makes it absolute: *"a fixture nobody can re-baseline
is only half a live reference"*. That bites unusually hard here. Nobody
hand-computes a CRC-16 over eight 32-byte buffers, or 41 frames of chunked text,
so without this file the only practical way to change a vector would be *"run
``clapboard_slate.build_transaction`` and paste the output"* — which would turn
the fixture into a snapshot of our own encoder agreeing with itself and destroy
the one property that makes it worth having.

So **every byte here is built from ``Electronic-Clapboard docs/protocol.md`` §8
and that repo's ``src/can_frames.h``**. Nothing in this file imports
``jugglebot.clapboard_slate``, ``jugglebot.can.clapboard`` or ``teensy_link`` —
pure stdlib, deliberately. The CRC below is transcribed from
``can_frames.h:151-161`` and anchored on the canonical CCITT-FALSE check value;
the layout encoders are transcribed from that header's encode/decode pairs and
anchored on the literal bytes that repo's own Unity tests assert. Those anchors
are ``assert``s here, not comments: this script refuses to emit a fixture it
cannot first reproduce the peer's own goldens with.

``tests/ros/test_clapboard_wire_vectors.py`` then asserts the LIVE Jugglebot
implementations reproduce what this script derived — a genuine second opinion.
It also re-runs ``build_fixture()`` and compares, so the committed JSON is
provably the recipe's output rather than something hand-edited on top of it.

USAGE
-----
    python tools/probes/clapboard_wire_vectors_gen.py               # dry run: diff vs committed
    python tools/probes/clapboard_wire_vectors_gen.py --emit-fixture  # rewrite it

``--emit-fixture`` is required to write, per the probe conventions. After
emitting, re-publish the digest and TELL THE CLAPBOARD REPO — the artefact is
jointly owned and neither side may change a vector unilaterally::

    sha256sum tests/fixtures/clapboard_wire_vectors.json \\
      | sed 's|tests/fixtures/||' > tests/fixtures/clapboard_wire_vectors.json.sha256

No hardware, no ROS 2, no network. Pure stdlib.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_FIXTURE = _REPO_ROOT / "tests" / "fixtures" / "clapboard_wire_vectors.json"

# Source citations, written into every vector so a reader can re-derive by hand.
SPEC = "Electronic-Clapboard docs/protocol.md"
PEERH = "Electronic-Clapboard src/can_frames.h"
PEER = "Electronic-Clapboard test/test_can_frames/test_can_frames.cpp"

# ── Field model (protocol.md 8.3 / can_frames.h:43-55) ───────────────────────
MAX_FIELDS = 8
MAX_FIELD_BYTES = 32
CHUNK_PAYLOAD = 7
CHUNKS_PER_FIELD = 5
DLC = 8
MAX_TEMPLATE_ID = 15
MAX_FRAMES_PER_TRANSACTION = MAX_FIELDS * CHUNKS_PER_FIELD + 1


# ── CRC-16/CCITT-FALSE, transcribed from can_frames.h:151-161 ────────────────

def crc16_ccitt_false(data: bytes) -> int:
    """poly 0x1021, init 0xFFFF, no reflection, no final XOR."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def buf32(text: str) -> bytes:
    """One field's 32-byte NUL-padded panel buffer."""
    raw = text.encode("utf-8")
    assert len(raw) <= MAX_FIELD_BYTES, f"{text!r} exceeds MAX_FIELD_BYTES"
    return raw.ljust(MAX_FIELD_BYTES, b"\x00")


def crc_over_fields(fields: dict) -> int:
    """can_frames.h:170-184 — present fields' 32-byte buffers, ascending id.

    Fixed-width padding rather than trimmed strings is what makes the CRC
    independent of chunk arrival order; absent fields contribute nothing at all,
    which is why an empty present set returns the untouched init value.
    """
    return crc16_ccitt_false(b"".join(buf32(fields[f]) for f in sorted(fields)))


# ── Layout encoders, transcribed from can_frames.h ───────────────────────────

def enc_field_chunks(field_id: int, buffer32: bytes):
    """protocol.md 8.3: d[0] = field_id | seq<<4, d[1..7] = 7 text bytes."""
    padded = buffer32.ljust(CHUNKS_PER_FIELD * CHUNK_PAYLOAD, b"\x00")
    return [bytes([(field_id & 0x0F) | ((seq & 0x0F) << 4)])
            + padded[seq * CHUNK_PAYLOAD:(seq + 1) * CHUNK_PAYLOAD]
            for seq in range(CHUNKS_PER_FIELD)]


def enc_commit(template_id, mask, flags, txn_id, crc) -> bytes:
    """protocol.md 8.4. Bytes 6-7 reserved and explicitly zeroed."""
    return bytes([template_id, mask, flags, txn_id,
                  crc & 0xFF, (crc >> 8) & 0xFF, 0, 0])


def enc_link(ros2_up: bool) -> bytes:
    """protocol.md 8.5. Byte 0 only; bytes 1-7 reserved, must be 0."""
    return bytes([1 if ros2_up else 0]) + b"\x00" * 7


def enc_ack(txn_id, outcome, state, render_ms) -> bytes:
    """protocol.md 8.6."""
    return (bytes([txn_id, outcome, state])
            + int(render_ms).to_bytes(2, "little") + b"\x00\x00\x00")


def enc_heartbeat(state, flags, fires, rail_mv, template_id, last_error) -> bytes:
    """protocol.md 8.7."""
    return (bytes([state, flags])
            + int(fires).to_bytes(2, "little")
            + int(rail_mv).to_bytes(2, "little")
            + bytes([template_id, last_error]))


def enc_fire_event(wall_us, fires) -> bytes:
    """protocol.md 8.8 — the 48-bit MASK is the rule the spec never states."""
    return ((wall_us & 0x0000FFFFFFFFFFFF).to_bytes(6, "little")
            + int(fires).to_bytes(2, "little"))


def enc_time_sync(unix_s, usec) -> bytes:
    """The bridge's broadcast_0x7dd(): pack('<II', sec, usec)."""
    return int(unix_s).to_bytes(4, "little") + int(usec).to_bytes(4, "little")


def _hx(data: bytes) -> str:
    return data.hex()


def _mask_of(field_ids) -> int:
    mask = 0
    for fid in field_ids:
        mask |= 1 << fid
    return mask


# ── Anchors: reproduce the peer's own literal goldens, or refuse to emit ─────

def _check_peer_anchors() -> None:
    assert crc16_ccitt_false(b"123456789") == 0x29B1, (
        "CRC transcription is wrong — the canonical CCITT-FALSE check value "
        "does not come out")
    assert crc16_ccitt_false(b"") == 0xFFFF

    # test_field_chunk_round_trip: field 5 / seq 3 / "abcdefg" -> d[0] == 0x35
    buf = buf32("peer-golden-carry-ove" + "abcdefg")
    assert buf[21:28] == b"abcdefg"
    assert _hx(enc_field_chunks(5, buf)[3]) == "3561626364656667"

    # test_commit_round_trip_and_layout
    assert _hx(enc_commit(3, 0b00001011, 0x01, 200, 0xBEEF)) == "030b01c8efbe0000"

    # test_ack_layout / test_heartbeat_layout
    assert list(enc_ack(42, 0x02, 0x01, 2500)) == [42, 2, 1, 0xC4, 0x09, 0, 0, 0]
    assert list(enc_heartbeat(3, 0x14, 0x0102, 0, 2, 3)) == [3, 0x14, 2, 1, 0, 0, 2, 3]

    # test_fire_event_48_bit_round_trip / ..._truncates_above_48_bits_...
    fire = enc_fire_event(0x0000112233445566, 7)
    assert fire[0] == 0x66 and fire[5] == 0x11 and fire[6] == 7 and fire[7] == 0
    ovf = enc_fire_event(0xFFFFFFFFFFFFFFFF, 0x1234)
    assert ovf[:6] == b"\xff" * 6 and ovf[6:] == b"\x34\x12"

    # test_time_sync_decodes_little_endian_pair
    assert list(enc_time_sync(1786000000, 999999)) == [
        0x80, 0x32, 0x74, 0x6A, 0x3F, 0x42, 0x0F, 0x00]


# ── Vector sets ──────────────────────────────────────────────────────────────

_CRC_FIELD_SETS = [
    ("peer/order-independent-set",
     {0: "2026-08-16", 1: "Scene 4", 2: "Take 12", 3: "wide"},
     "Fields 0-3 of the peer's order-independence case. The peer asserts only "
     "that two fill orders agree; the literal value is derived here so both "
     "repos pin the same number.",
     "test_crc_over_fields_is_order_independent"),
    ("peer/single-field-scene",
     {0: "Scene 4"},
     "A stale buffer in an ABSENT field must not change this value - patch "
     "semantics make stale buffers the normal case, not an edge case.",
     "test_crc_over_fields_ignores_absent_fields"),
    ("peer/stale-field-now-present",
     {0: "Scene 4", 3: "stale leftover"},
     "Same buffers as peer/single-field-scene with field 3 now IN the mask. "
     "Must differ from it, or the mask is not being honoured.",
     "test_crc_over_fields_ignores_absent_fields"),
    ("peer/bit-flip-good",
     {1: "Take 12"},
     "Pairs with peer/bit-flip-corrupted: one flipped bit inside the text must "
     "change the CRC.",
     "test_crc_detects_a_single_bit_flip"),
    ("peer/bit-flip-corrupted",
     {1: "Takd 12"},   # 'e' (0x65) ^ 0x01 == 'd' (0x64) at index 3
     "peer/bit-flip-good with byte 3 of the text XORed by 0x01.",
     "test_crc_detects_a_single_bit_flip"),
]

_TRANSACTIONS = [
    ("full-slate-8-fields", 3, 200, False,
     {0: "Scene 4", 1: "Take 12", 2: "wide", 3: "", 4: "X",
      5: "0123456789abcdefghijklmnopqrstuv", 6: "unit-a", 7: "cam L/R sync check"},
     "The worst case: 8 present fields x 5 chunks + 1 commit = 41 frames, the "
     "number CLAP_MAX_FRAMES (48) is sized from. Field 3 is present and EMPTY "
     "(a clear), field 5 is exactly 32 bytes. Commit is LAST."),
    ("single-field-patch", 3, 200, False,
     {2: "Take 12"},
     "UNSTATED RULE: all five chunks go out even for a 7-byte value. Panel "
     "field buffers persist between transactions (8.4 patch semantics) and the "
     "CRC covers the full 32-byte buffer, so a prefix-only send leaves the "
     "previous value's tail in the receiver -> wrong text AND a spurious "
     "CRC_MISMATCH."),
    ("repaint-only-no-fields", 2, 7, True,
     {},
     "A commit with an empty mask: repaint what is already there. One frame, "
     "CRC 0xFFFF, force-full-refresh flag set."),
]

_README_LINES = [
    "SHARED CROSS-REPOSITORY WIRE FIXTURE - Electronic Clapboard over CAN3.",
    "",
    "OWNERSHIP: the Jugglebot repo OWNS this file, at",
    "tests/fixtures/clapboard_wire_vectors.json. The Electronic-Clapboard repo",
    "VENDORS a byte-identical copy and pins it with a checksum test. Jugglebot",
    "has the stricter gate (run_tests.sh --full runs on every commit), so it",
    "holds the artefact. Owner decision, 2026-08-16, recorded in",
    "plans/active/clapboard-can3-integration.md section 6.",
    "",
    "WHAT THE CLAPBOARD REPO MUST DO:",
    "  1. Copy this file verbatim into its own tree. Pick a path its build",
    "     system is happy with - PlatformIO treats every directory under test/",
    "     as a test suite, so a bare test/fixtures/ may not be the right home;",
    "     that choice is the clapboard repo's, not Jugglebot's.",
    "  2. Copy the published SHA-256 from clapboard_wire_vectors.json.sha256",
    "     beside it and assert, in its own test suite, that the vendored copy",
    "     still hashes to that value. That test is what makes a silent",
    "     divergence impossible.",
    "  3. Assert its can_frames.h encoders/decoders reproduce every vector",
    "     here, exactly as tests/ros/test_clapboard_wire_vectors.py does on",
    "     the Jugglebot side.",
    "  4. Re-vendor (and re-pin the hash) whenever Jugglebot publishes a new",
    "     one. NEITHER SIDE MAY CHANGE A VECTOR UNILATERALLY - a change here",
    "     is a change to docs/protocol.md section 8, which is normative and",
    "     jointly owned.",
    "",
    "WHY THE VECTORS EXIST: docs/protocol.md section 8 documents the byte",
    "layouts but is silent on four rules that only code enforces. They are",
    "listed under unstated_rules below, and each has a vector that fails if",
    "either side forgets it.",
    "",
    "DERIVATION: every byte here was built from docs/protocol.md section 8 and",
    "src/can_frames.h, NOT from the Jugglebot encoders - so the agreement test",
    "is a second opinion rather than a snapshot. Vectors whose name begins",
    "'peer/' are carried across from that repo's own test assertions verbatim,",
    "so the two suites agree by construction. The recipe is committed at",
    "tools/probes/clapboard_wire_vectors_gen.py; do not hand-edit this file.",
]


def build_fixture() -> dict:
    """Derive the whole fixture. Pure function — no I/O, no repo imports."""
    _check_peer_anchors()

    crc_sets = []
    for name, fields, note, peer_test in _CRC_FIELD_SETS:
        crc_sets.append({
            "name": name,
            "fields": {str(k): v for k, v in sorted(fields.items())},
            "crc16": "0x%04X" % crc_over_fields(fields),
            "note": note,
            "source": f"{PEER}::{peer_test}",
        })
    crc_sets.append({
        "name": "empty-present-set",
        "fields": {},
        "crc16": "0xFFFF",
        "note": "UNSTATED RULE: a repaint-only commit names no field, so the CRC "
                "is the untouched init value. protocol.md 8.4 does not say so; it "
                "follows from crc16_over_fields skipping every absent field.",
        "source": f"{PEERH}::crc16_over_fields with present_mask == 0",
    })
    # The relations the peer asserts must hold on the literals we publish.
    by_name = {c["name"]: c["crc16"] for c in crc_sets}
    assert by_name["peer/single-field-scene"] != by_name["peer/stale-field-now-present"]
    assert by_name["peer/bit-flip-good"] != by_name["peer/bit-flip-corrupted"]

    transactions = []
    for name, template_id, txn_id, full_refresh, fields, note in _TRANSACTIONS:
        frames = []
        for fid in sorted(fields):
            for chunk in enc_field_chunks(fid, buf32(fields[fid])):
                frames.append({"can_id": "0x7E8", "dlc": DLC, "data": _hx(chunk)})
        mask = _mask_of(fields)
        crc = crc_over_fields(fields)
        frames.append({"can_id": "0x7E9", "dlc": DLC,
                       "data": _hx(enc_commit(template_id, mask,
                                              0x01 if full_refresh else 0x00,
                                              txn_id, crc))})
        transactions.append({
            "name": name,
            "template_id": template_id,
            "fields": {str(k): v for k, v in sorted(fields.items())},
            "txn_id": txn_id,
            "force_full_refresh": full_refresh,
            "present_mask": "0x%02X" % mask,
            "crc16": "0x%04X" % crc,
            "frame_count": len(frames),
            "note": note,
            "frames": frames,
        })
    assert [t["frame_count"] for t in transactions] == [
        MAX_FRAMES_PER_TRANSACTION, CHUNKS_PER_FIELD + 1, 1]

    peer_chunk_buf = buf32("peer-golden-carry-ove" + "abcdefg")
    max_buf = buf32("0123456789abcdefghijklmnopqrstuv")

    frames = [
        {"name": "peer/field-chunk-f5-seq3-abcdefg", "kind": "field_chunk",
         "can_id": "0x7E8", "direction": "host_to_clapboard", "dlc": DLC,
         "data": _hx(enc_field_chunks(5, peer_chunk_buf)[3]),
         "field_id": 5, "seq": 3, "buffer32": _hx(peer_chunk_buf),
         "note": "The peer's own layout golden, carried across EXACTLY: field 5 / "
                 "seq 3 -> d[0] == 0x35, d[1] == 'a', d[7] == 'g'. buffer32 is the "
                 "32-byte field buffer whose seq-3 chunk reproduces it.",
         "source": f"{PEER}::test_field_chunk_round_trip"},
        {"name": "field-chunk-f0-seq0-empty-field", "kind": "field_chunk",
         "can_id": "0x7E8", "direction": "host_to_clapboard", "dlc": DLC,
         "data": _hx(enc_field_chunks(0, buf32(""))[0]),
         "field_id": 0, "seq": 0, "buffer32": _hx(buf32("")),
         "note": "Clearing a field is a present field with an all-NUL buffer, not "
                 "an absent one. Still five chunks, still DLC 8.",
         "source": f"{SPEC} 8.3"},
        {"name": "field-chunk-f7-seq4-boundary", "kind": "field_chunk",
         "can_id": "0x7E8", "direction": "host_to_clapboard", "dlc": DLC,
         "data": _hx(enc_field_chunks(7, max_buf)[4]),
         "field_id": 7, "seq": 4, "buffer32": _hx(max_buf),
         "note": "Both nibbles at their limit (field 7, seq 4 -> d[0] == 0x47), and "
                 "the last chunk of a MAXIMUM-length field: 4 text bytes then the 3 "
                 "trailing NULs that let the receiver find the terminator.",
         "source": f"{PEER}::test_field_chunk_rejects_out_of_range_ids"},
        {"name": "peer/commit-t3-mask0b1011-txn200-crcBEEF", "kind": "commit",
         "can_id": "0x7E9", "direction": "host_to_clapboard", "dlc": DLC,
         "data": _hx(enc_commit(3, 0b00001011, 0x01, 200, 0xBEEF)),
         "template_id": 3, "present_mask": "0x0B", "flags": "0x01", "txn_id": 200,
         "crc16": "0xBEEF",
         "note": "The peer's own layout golden, carried across EXACTLY: template 3 "
                 "/ mask 0b00001011 / flags 0x01 / txn 200 / crc 0xBEEF -> "
                 "d[4] == 0xEF, d[5] == 0xBE (little-endian), d[6..7] == 0 "
                 "(reserved, must be 0).",
         "source": f"{PEER}::test_commit_round_trip_and_layout"},
        {"name": "link-ros2-up", "kind": "link",
         "can_id": "0x7EA", "direction": "bridge_to_clapboard", "dlc": DLC,
         "data": _hx(enc_link(True)), "ros2_up": True,
         "emitted_by_this_repo": True,
         "note": "THE named first-integration-bug guard. Byte 0 is the only "
                 "meaningful byte, so a 1-byte frame is the natural implementation "
                 "- and the clapboard drops any DLC != 8 silently, leaving the "
                 "panel in screensaver forever with no error on either side.",
         "source": f"{SPEC} 8.5"},
        {"name": "link-ros2-down", "kind": "link",
         "can_id": "0x7EA", "direction": "bridge_to_clapboard", "dlc": DLC,
         "data": _hx(enc_link(False)), "ros2_up": False,
         "emitted_by_this_repo": True,
         "note": "Screensaver is the safe default, so every link state short of UP "
                 "reports DOWN. DLC 8 in this direction too.",
         "source": f"{SPEC} 8.5"},
        {"name": "peer/link-any-nonzero-is-up", "kind": "link",
         "can_id": "0x7EA", "direction": "bridge_to_clapboard", "dlc": DLC,
         "data": _hx(bytes([0x81]) + b"\x00" * 7), "ros2_up": True,
         "emitted_by_this_repo": False,
         "note": "The bridge never emits this, but the clapboard MUST read it as "
                 "UP: a strict ==1 would turn a future flag bit in byte 0 into a "
                 "silent permanent 'ROS2 down', which fails in the direction that "
                 "blanks the panel. Recorded so a future bridge-side flag bit is "
                 "safe by construction.",
         "source": f"{PEER}::test_link_treats_any_nonzero_as_up"},
        {"name": "peer/ack-txn42-crc-mismatch-2500ms", "kind": "ack",
         "can_id": "0x7EB", "direction": "clapboard_to_host", "dlc": DLC,
         "data": _hx(enc_ack(42, 0x02, 0x01, 2500)),
         "txn_id": 42, "outcome": 2, "state": 1, "render_ms": 2500,
         "note": "The peer's own layout golden, carried across EXACTLY: txn 42, "
                 "outcome CRC_MISMATCH (0x02), state IDLE (0x01), render_ms 2500 = "
                 "0x09C4 little-endian at bytes 3-4, bytes 5-7 reserved and zeroed.",
         "source": f"{PEER}::test_ack_layout"},
        {"name": "ack-txn1-ok-1800ms", "kind": "ack",
         "can_id": "0x7EB", "direction": "clapboard_to_host", "dlc": DLC,
         "data": _hx(enc_ack(1, 0x00, 0x02, 1800)),
         "txn_id": 1, "outcome": 0, "state": 2, "render_ms": 1800,
         "note": "The happy path a SetSlate goal ends on: OK, still RENDERING at "
                 "ack time, render_ms inside the 1500-3500 ms full-refresh envelope.",
         "source": f"{SPEC} 8.6"},
        {"name": "peer/heartbeat-screensaver-wifi-timesynced", "kind": "heartbeat",
         "can_id": "0x7EC", "direction": "clapboard_to_host", "dlc": DLC,
         "data": _hx(enc_heartbeat(3, 0x14, 0x0102, 0, 2, 3)),
         "state": 3, "flags": "0x14", "fire_ready": False,
         "template_loaded": False, "wifi_up": True, "rail_ok": False,
         "time_synced": True, "fires_since_boot": 258, "rail_mv": 0,
         "active_template_id": 2, "last_error": 3,
         "note": "The peer's own layout golden, carried across EXACTLY: state "
                 "SCREENSAVER (3), flags bit2|bit4 == 0x14, fires 0x0102 "
                 "little-endian at bytes 2-3, rail_mv 0 (no divider fitted), "
                 "template 2, last_error 3.",
         "source": f"{PEER}::test_heartbeat_layout"},
        {"name": "heartbeat-idle-all-flags", "kind": "heartbeat",
         "can_id": "0x7EC", "direction": "clapboard_to_host", "dlc": DLC,
         "data": _hx(enc_heartbeat(1, 0x1F, 0, 3299, 1, 0)),
         "state": 1, "flags": "0x1F", "fire_ready": True, "template_loaded": True,
         "wifi_up": True, "rail_ok": True, "time_synced": True,
         "fires_since_boot": 0, "rail_mv": 3299, "active_template_id": 1,
         "last_error": 0,
         "note": "Every flag set, a real rail reading (3299 mV), no errors yet. "
                 "time_synced is the load-bearing one: while it is false the "
                 "clapboard emits NO fire events at all, so a gap between "
                 "fires_since_boot and published events is expected, not a defect.",
         "source": f"{SPEC} 8.7"},
        {"name": "peer/fire-event-48bit", "kind": "fire_event",
         "can_id": "0x7ED", "direction": "clapboard_to_host", "dlc": DLC,
         "data": _hx(enc_fire_event(0x0000112233445566, 7)),
         "wall_us_low48": 0x112233445566, "fires_since_boot": 7,
         "note": "The peer's own layout golden, carried across EXACTLY: 48-bit "
                 "little-endian microseconds in bytes 0-5, fires_since_boot in 6-7.",
         "source": f"{PEER}::test_fire_event_48_bit_round_trip"},
        {"name": "peer/fire-event-overflow-does-not-corrupt-seq",
         "kind": "fire_event",
         "can_id": "0x7ED", "direction": "clapboard_to_host", "dlc": DLC,
         "data": _hx(enc_fire_event(0xFFFFFFFFFFFFFFFF, 0x1234)),
         "wall_us_low48": 0xFFFFFFFFFFFF, "fires_since_boot": 0x1234,
         "note": "The peer's own golden for the mask: a wall clock above 2^48 is "
                 "truncated and must NOT bleed into bytes 6-7. This is the frame "
                 "that makes the UNSTATED 48-bit rule visible - see unstated_rules.",
         "source": f"{PEER}::"
                   "test_fire_event_truncates_above_48_bits_without_corrupting_seq"},
        {"name": "peer/time-sync-1786000000s-999999us", "kind": "time_sync",
         "can_id": "0x7DD", "direction": "bridge_to_clapboard", "dlc": DLC,
         "data": _hx(enc_time_sync(1786000000, 999999)),
         "unix_s": 1786000000, "usec": 999999, "emitted_by_this_repo": True,
         "note": "Not a clapboard-block id, but the bridge emits it on this same "
                 "bus and the clapboard's decoder requires DLC 8 for it too - so it "
                 "is in scope for the DLC-8 emission invariant. pack('<II', sec, "
                 "usec), matching broadcast_0x7dd(). usec >= 1e6 is REJECTED by the "
                 "peer rather than folded in: a wrong anchor is worse than no "
                 "anchor.",
         "source": f"{PEER}::test_time_sync_decodes_little_endian_pair"},
    ]

    unstated_rules = [
        {"id": "strict-dlc8",
         "rule": "EVERY frame in this contract, both directions, is exactly DLC 8. "
                 "A receiver drops any other length silently - no error on either "
                 "side.",
         "enforced_in": f"{PEERH}:191-232 (every decoder tests len != 8)",
         "stated_in_protocol_md": False,
         "pinned_by": "tests/ros/test_clapboard_wire_vectors.py + "
                      "tests/firmware/test_clap_wire_dlc_lint.py + "
                      "tests/firmware/native/test_clap_link.cpp"},
        {"id": "fire-timestamp-low48",
         "rule": "CLAP_FIRE_EVENT bytes 0-5 are the LOW 48 BITS of a Unix-epoch "
                 "microsecond clock that needs 51. The receiver must reconstruct "
                 "the high bits from its own clock; taking the field at face value "
                 "stamps every flash in 1978.",
         "enforced_in": f"{PEERH}:258-268 "
                        "(encode_fire_event masks with 0x0000FFFFFFFFFFFF)",
         "stated_in_protocol_md": False,
         "protocol_md_says": "8.8's '~8.9 years' is the WRAP PERIOD, not the "
                             "representable range, and no reconstruction rule is "
                             "given.",
         "pinned_by": "ros_ws/src/jugglebot/jugglebot/can/clapboard.py "
                      "(reconstruct_fire_time_us) + tests/ros/test_clapboard.py"},
        {"id": "all-five-chunks",
         "rule": "All CHUNKS_PER_FIELD (5) chunks are sent for every PRESENT field, "
                 "even a 1-byte value. Follows from combining 8.4's patch semantics "
                 "(panel field buffers persist between transactions) with a CRC "
                 "over the full 32-byte padded buffer: a prefix-only send leaves "
                 "the previous value's tail in the receiver, rendering wrong text "
                 "AND producing a spurious CRC_MISMATCH.",
         "enforced_in": "ros_ws/src/jugglebot/jugglebot/clapboard_slate.py "
                        "(build_transaction; sender-side, because the clapboard "
                        "cannot detect the difference except as a CRC failure)",
         "stated_in_protocol_md": False,
         "pinned_by": "tests/ros/test_clapboard_wire_vectors.py "
                      "(the single-field-patch transaction: 6 frames, not 2)"},
        {"id": "empty-field-set-crc",
         "rule": "A commit naming NO field (a repaint) CRCs to the init value "
                 "0xFFFF. Both sides must agree or every repaint-only commit is "
                 "rejected.",
         "enforced_in": f"{PEERH}:170-184 "
                        "(the loop body is skipped for every absent field)",
         "stated_in_protocol_md": False,
         "pinned_by": "tests/ros/test_clapboard_wire_vectors.py "
                      "(the empty-present-set CRC vector + the repaint-only "
                      "transaction)"},
    ]

    return {
        "_README": _README_LINES,
        "schema_version": 1,
        "owner_repo": "Jugglebot",
        "owner_path": "tests/fixtures/clapboard_wire_vectors.json",
        "checksum_file": "tests/fixtures/clapboard_wire_vectors.json.sha256",
        "recipe": "tools/probes/clapboard_wire_vectors_gen.py",
        "vendored_by": "Electronic-Clapboard",
        "normative_spec": f"{SPEC} section 8 (jointly owned; neither side may "
                          "change it unilaterally)",
        "peer_implementation": PEERH,
        "peer_tests": PEER,
        "constants": {
            "MAX_FIELDS": MAX_FIELDS,
            "MAX_FIELD_BYTES": MAX_FIELD_BYTES,
            "CHUNK_PAYLOAD": CHUNK_PAYLOAD,
            "CHUNKS_PER_FIELD": CHUNKS_PER_FIELD,
            "DLC": DLC,
            "MAX_TEMPLATE_ID": MAX_TEMPLATE_ID,
            "MAX_FRAMES_PER_TRANSACTION": MAX_FRAMES_PER_TRANSACTION,
        },
        "can_ids": {
            "TIME_SYNC": "0x7DD",
            "CLAP_FIELD": "0x7E8",
            "CLAP_COMMIT": "0x7E9",
            "CLAP_LINK": "0x7EA",
            "CLAP_ACK": "0x7EB",
            "CLAP_HEARTBEAT": "0x7EC",
            "CLAP_FIRE_EVENT": "0x7ED",
            "CLAP_BLOCK_FIRST": "0x7E8",
            "CLAP_BLOCK_LAST": "0x7EF",
        },
        "unstated_rules": unstated_rules,
        "crc16_ccitt_false": {
            "poly": "0x1021",
            "init": "0xFFFF",
            "reflect_in": False,
            "reflect_out": False,
            "xor_out": "0x0000",
            "known_answers": [
                {"name": "canonical-check-value",
                 "input_hex": _hx(b"123456789"), "crc16": "0x29B1",
                 "note": "The canonical CCITT-FALSE check value. If this fails the "
                         "two repos have DIFFERENT CRC variants and every commit is "
                         "rejected on the wire.",
                 "source": f"{PEER}::test_crc_known_answer_vectors"},
                {"name": "empty-input", "input_hex": "", "crc16": "0xFFFF",
                 "note": "Empty input returns the init value untouched - which is "
                         "also why a repaint-only commit CRCs to 0xFFFF.",
                 "source": f"{PEER}::test_crc_known_answer_vectors"},
            ],
        },
        "crc16_over_fields": crc_sets,
        "frames": frames,
        "transactions": transactions,
    }


def render(fixture: dict) -> str:
    """The exact committed file text — one renderer, so the test can compare."""
    return json.dumps(fixture, indent=2, ensure_ascii=True) + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--emit-fixture", action="store_true",
                    help="rewrite tests/fixtures/clapboard_wire_vectors.json "
                         "(required to write; a bare run only reports drift)")
    args = ap.parse_args(argv)

    text = render(build_fixture())
    if args.emit_fixture:
        _FIXTURE.parent.mkdir(parents=True, exist_ok=True)
        _FIXTURE.write_text(text, encoding="utf-8")
        print(f"wrote {_FIXTURE.relative_to(_REPO_ROOT)} ({len(text)} bytes)")
        print("NOW: re-publish the digest and tell the Electronic-Clapboard repo.")
        print("  sha256sum tests/fixtures/clapboard_wire_vectors.json "
              "| sed 's|tests/fixtures/||' "
              "> tests/fixtures/clapboard_wire_vectors.json.sha256")
        return 0

    if not _FIXTURE.exists():
        print(f"{_FIXTURE} does not exist — run with --emit-fixture")
        return 1
    committed = _FIXTURE.read_text(encoding="utf-8")
    if committed == text:
        print(f"{_FIXTURE.relative_to(_REPO_ROOT)} is up to date with this recipe.")
        return 0
    print(f"DRIFT: {_FIXTURE.relative_to(_REPO_ROOT)} differs from this recipe.")
    print("Re-run with --emit-fixture, then re-publish the digest.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
