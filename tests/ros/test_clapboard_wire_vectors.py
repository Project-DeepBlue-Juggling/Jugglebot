"""Cross-repo contract: this repo's clapboard code vs the SHARED wire fixture.

Phase 4 of ``plans/active/clapboard-can3-integration.md``. T-U4 (CRC), T-U8
(DLC-8 both directions) and the fixture-agreement guard the phase exists for.

The problem this closes
-----------------------
Until now the Jugglebot and Electronic-Clapboard repos each had their own
clapboard test file, asserting their own encoders against their own reading of
``docs/protocol.md`` §8 — with **no shared artefact**. Two suites can both stay
green while drifting apart, and the first evidence would be a bench sitting where
the panel silently ignores every frame.

``tests/fixtures/clapboard_wire_vectors.json`` is that shared artefact. This repo
OWNS it; the clapboard repo VENDORS a byte-identical copy pinned by a checksum
test (owner decision 2026-08-16, plan §6). Every byte in it was derived from
``protocol.md`` §8 and that repo's ``src/can_frames.h`` — **not** from
``jugglebot.clapboard_slate`` — so the assertions below are a second opinion
rather than a snapshot of our own encoder agreeing with itself. Vectors named
``peer/...`` are carried across verbatim from that repo's own Unity assertions,
so the two suites agree by construction.

Why a JSON fixture rather than literals in both suites: a literal in two
codebases is two literals. A vendored file with a published checksum makes
"these two repos disagree" a **test failure on both sides** instead of a bench
surprise.

What must FAIL here
-------------------
Changing the chunking geometry, the CRC input construction (padding, ordering,
absent-field handling) or any frame layout breaks a byte-exact vector. Editing
the fixture without re-publishing its checksum breaks
:func:`test_fixture_matches_published_checksum` — which is the moment a human is
told to go tell the other repo.

No ROS 2 dependency: the modules under test are pure Python. This file lives in
``tests/ros/`` because that is where the ``jugglebot`` package's tests live, as
its siblings ``test_clapboard.py`` and ``test_clapboard_slate.py`` do.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import re
import struct
from pathlib import Path

import pytest

from teensy_link import protocol as p
from teensy_link import rpc_args as ra

import jugglebot.protocol_config as proto
from jugglebot import clapboard_slate as cs
from jugglebot.can import clapboard as cb

_REPO_ROOT = Path(__file__).resolve().parents[2]
_FIXTURE = _REPO_ROOT / "tests" / "fixtures" / "clapboard_wire_vectors.json"
_CHECKSUM = _REPO_ROOT / "tests" / "fixtures" / "clapboard_wire_vectors.json.sha256"
_RECIPE = _REPO_ROOT / "tools" / "probes" / "clapboard_wire_vectors_gen.py"

#: Loaded at import time, not in a fixture, because ``parametrize`` runs at
#: collection.
VECTORS = json.loads(_FIXTURE.read_text(encoding="utf-8"))

#: Every top-level section the assertions below know how to consume. A new one
#: fails :func:`test_every_fixture_section_is_consumed` until it is wired up —
#: which is what stops the file rotting into decoration.
_KNOWN_SECTIONS = frozenset({
    "_README", "schema_version", "owner_repo", "owner_path", "checksum_file",
    "recipe", "vendored_by", "normative_spec", "peer_implementation",
    "peer_tests", "constants", "can_ids", "unstated_rules", "crc16_ccitt_false",
    "crc16_over_fields", "frames", "transactions",
})

#: Frame kinds with a check below. An unknown kind is a hard failure, never a
#: silent skip.
_HANDLED_KINDS = frozenset({
    "field_chunk", "commit", "link", "ack", "heartbeat", "fire_event",
    "time_sync",
})


def _ids(items):
    return [i["name"] for i in items]


def _fields_of(entry):
    """``{"0": "Scene 4"}`` (JSON keys are strings) → parallel id/value lists."""
    fields = {int(k): v for k, v in entry["fields"].items()}
    ids = sorted(fields)
    return ids, [fields[i] for i in ids]


# ══════════════════════════════════════════════════════════════════════════════
#  Ownership: the rule that makes the fixture shareable
# ══════════════════════════════════════════════════════════════════════════════

def test_fixture_matches_published_checksum():
    """The published SHA-256 is the version the clapboard repo vendors against.

    This is deliberately a chore. The fixture is a **jointly owned** artefact —
    the clapboard repo holds a byte-identical copy and asserts this same hash —
    so changing a vector without re-publishing the hash is exactly the silent
    unilateral change §8's cross-repository banner forbids. Failing here is the
    prompt to (a) re-publish and (b) tell the other repo.
    """
    actual = hashlib.sha256(_FIXTURE.read_bytes()).hexdigest()
    published = _CHECKSUM.read_text(encoding="utf-8").split()[0].strip()
    assert actual == published, (
        "tests/fixtures/clapboard_wire_vectors.json changed but its published "
        f"checksum did not.\n  actual:    {actual}\n  published: {published}\n"
        "If the change is intentional: write the new digest into "
        "tests/fixtures/clapboard_wire_vectors.json.sha256 AND tell the "
        "Electronic-Clapboard repo to re-vendor — it pins this same value. "
        "Neither side may change a vector unilaterally (protocol.md 8's "
        "cross-repository banner).")


def test_fixture_is_exactly_what_the_committed_recipe_produces():
    """The fixture is a product of its recipe, never a hand-edit on top of one.

    Nobody hand-computes a CRC-16 over eight 32-byte buffers or 41 frames of
    chunked text, so without a committed recipe the only practical way to change
    a vector would be "run ``build_transaction`` and paste" — which would make
    the fixture a snapshot of our own encoder agreeing with itself and destroy
    the whole point. ``tools/probes/README.md`` states the rule generally: a
    fixture nobody can re-baseline is only half a live reference.

    The recipe imports nothing from this repo — it transcribes ``protocol.md``
    §8 and the peer's ``can_frames.h``, and refuses to emit unless it first
    reproduces that repo's own literal goldens.
    """
    spec = importlib.util.spec_from_file_location("clap_vectors_gen", _RECIPE)
    gen = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gen)
    fresh = gen.render(gen.build_fixture())
    assert fresh == _FIXTURE.read_text(encoding="utf-8"), (
        "tests/fixtures/clapboard_wire_vectors.json is not what "
        "tools/probes/clapboard_wire_vectors_gen.py produces. Regenerate it:\n"
        "  python tools/probes/clapboard_wire_vectors_gen.py --emit-fixture\n"
        "then re-publish the digest. Do not hand-edit the fixture.")


def test_fixture_states_its_ownership_rule():
    """The ownership rule travels INSIDE the vendored file, not beside it.

    The clapboard repo receives the JSON, not this test module and not the plan.
    If the rule lived only here, the vendored copy would arrive with no statement
    of who may change it.
    """
    assert VECTORS["schema_version"] == 1
    assert VECTORS["owner_repo"] == "Jugglebot"
    assert VECTORS["owner_path"] == "tests/fixtures/clapboard_wire_vectors.json"
    assert VECTORS["vendored_by"] == "Electronic-Clapboard"
    assert VECTORS["checksum_file"] == _CHECKSUM.relative_to(_REPO_ROOT).as_posix()
    assert VECTORS["recipe"] == _RECIPE.relative_to(_REPO_ROOT).as_posix()
    # The three citations that let a reader re-derive a vector by hand — the only
    # route back to ground truth once the JSON is sitting in the other repo.
    assert "protocol.md" in VECTORS["normative_spec"]
    assert "unilaterally" in VECTORS["normative_spec"]
    assert VECTORS["peer_implementation"].endswith("can_frames.h")
    assert VECTORS["peer_tests"].endswith("test_can_frames.cpp")
    readme = "\n".join(VECTORS["_README"])
    for phrase in ("OWNERSHIP", "VENDORS", "WHAT THE CLAPBOARD REPO MUST DO",
                   "UNILATERALLY"):
        assert phrase in readme, f"the fixture's _README no longer states {phrase!r}"


def test_every_repo_path_the_fixture_names_still_exists():
    """The fixture's own cross-references may not rot.

    ``unstated_rules[*].pinned_by`` and the ``_README`` point at the files that
    enforce each rule — the fixture's only route from "here is a rule" to "here
    is where it is checked". Renaming a test file would leave the vendored copy
    telling the peer repo to look somewhere that does not exist, and nothing
    else in this tree would notice.

    Peer-repo paths (``src/can_frames.h`` and friends) are deliberately NOT
    checked: that checkout is not guaranteed present on every box the suite runs
    on, and a gate that depends on a sibling repo's working tree is a gate that
    reddens for someone else's edit.
    """
    blob = json.dumps(VECTORS)
    named = sorted(set(re.findall(
        r"(?:tests|tools|ros_ws|config|controller)/[A-Za-z0-9_./-]+"
        r"\.(?:py|cpp|h|json|yaml|md|sha256)", blob)))
    assert named, "the fixture no longer cross-references any file in this repo"
    missing = [path for path in named if not (_REPO_ROOT / path).exists()]
    assert not missing, (
        f"the fixture names repo paths that do not exist: {missing}. Re-point "
        "them in tools/probes/clapboard_wire_vectors_gen.py and regenerate — the "
        "clapboard repo vendors this file and follows those references.")


def test_every_fixture_section_is_consumed():
    """A section nothing reads is a section that can drift silently.

    The whole failure mode this phase exists to close is "two artefacts that
    agree with themselves". A fixture entry with no consumer is that failure in
    miniature, so an unrecognised section fails rather than being ignored.
    """
    unknown = set(VECTORS) - _KNOWN_SECTIONS
    assert not unknown, (
        f"fixture sections with no consumer in this file: {sorted(unknown)}. "
        "Add the assertion, then add the key to _KNOWN_SECTIONS.")
    missing = _KNOWN_SECTIONS - set(VECTORS)
    assert not missing, f"fixture is missing expected sections: {sorted(missing)}"


@pytest.mark.parametrize("frame", VECTORS["frames"], ids=_ids(VECTORS["frames"]))
def test_every_frame_kind_is_handled(frame):
    """No vector may fall through to a silent no-op."""
    assert frame["kind"] in _HANDLED_KINDS, (
        f"frame vector {frame['name']!r} has kind {frame['kind']!r}, which no "
        "check below consumes. Add the check, then add the kind to "
        "_HANDLED_KINDS.")


# ══════════════════════════════════════════════════════════════════════════════
#  Constants and ids
# ══════════════════════════════════════════════════════════════════════════════

#: Every constant the fixture publishes → its live counterpart. Key-set equality
#: is asserted, so a constant added to the fixture with no live counterpart fails
#: rather than sitting unchecked.
_CONSTANTS = {
    "MAX_FIELDS": cs.MAX_FIELDS,
    "MAX_FIELD_BYTES": cs.MAX_FIELD_BYTES,
    "CHUNK_PAYLOAD": cs.CHUNK_PAYLOAD,
    "CHUNKS_PER_FIELD": cs.CHUNKS_PER_FIELD,
    "DLC": cs.DLC,
    "MAX_TEMPLATE_ID": cs.MAX_TEMPLATE_ID,
    "MAX_FRAMES_PER_TRANSACTION": cs.MAX_FRAMES_PER_TRANSACTION,
}

#: Fixture ``can_ids`` key → the live generated constant. ``TIME_SYNC`` is not a
#: clapboard-block id and comes from ``protocol_config`` directly.
_CAN_IDS = {
    "TIME_SYNC": proto.CAN_ID_SHARED_TIME_SYNC,
    "CLAP_FIELD": cb.FIELD_ID,
    "CLAP_COMMIT": cb.COMMIT_ID,
    "CLAP_LINK": cb.LINK_ID,
    "CLAP_ACK": cb.ACK_ID,
    "CLAP_HEARTBEAT": cb.HEARTBEAT_ID,
    "CLAP_FIRE_EVENT": cb.FIRE_EVENT_ID,
    "CLAP_BLOCK_FIRST": cb.BLOCK_FIRST_ID,
    "CLAP_BLOCK_LAST": cb.BLOCK_LAST_ID,
}


def test_the_constant_and_id_tables_are_fully_covered():
    """No fixture constant or id may sit here without a live counterpart.

    The same anti-rot rule as :func:`test_every_fixture_section_is_consumed`, one
    level down: an unchecked entry is exactly the "artefact that agrees with
    itself" this phase exists to prevent.
    """
    assert set(VECTORS["constants"]) == set(_CONSTANTS)
    assert set(VECTORS["can_ids"]) == set(_CAN_IDS)


@pytest.mark.parametrize("name", sorted(_CONSTANTS))
def test_constants_match_the_live_field_model(name):
    """The chunking geometry in the fixture is the geometry this repo builds to.

    Changing ``CHUNKS_PER_FIELD`` or ``MAX_FIELD_BYTES`` on one side only is the
    cheapest way to break the link, because the frames still look well-formed.
    """
    assert VECTORS["constants"][name] == _CONSTANTS[name], name


def test_a_whole_transaction_still_fits_one_rpc():
    """41 worst-case frames against ``CLAP_MAX_FRAMES``.

    Ordering is load-bearing and is guaranteed only because ONE ``CLAP_SEND``
    carries the whole transaction; a transaction that no longer fits would have
    to be split, and a split has no ordering guarantee at all.
    """
    assert VECTORS["constants"]["MAX_FRAMES_PER_TRANSACTION"] <= p.CLAP_MAX_FRAMES


@pytest.mark.parametrize("name", sorted(_CAN_IDS))
def test_can_ids_match_the_generated_constants(name):
    """Ids come from ``protocol_config.yaml`` here and from ``can_frames.h``
    there; the fixture is where the two are made to agree."""
    assert int(VECTORS["can_ids"][name], 16) == _CAN_IDS[name], name


def test_unstated_rules_are_all_recorded_as_unstated():
    """The four rules ``protocol.md`` §8 does not state but code enforces.

    Recorded in the shared artefact because that is the only document both repos
    read. If one of them ever *is* written into §8, flip the flag — do not delete
    the entry, or the next reader loses the reason the vector exists.
    """
    rules = {r["id"]: r for r in VECTORS["unstated_rules"]}
    assert set(rules) == {
        "strict-dlc8", "fire-timestamp-low48", "all-five-chunks",
        "empty-field-set-crc"}
    for rid, rule in rules.items():
        assert rule["stated_in_protocol_md"] is False, (
            f"{rid} is now marked as stated in protocol.md — good news, but "
            "update the note rather than dropping the vector that pins it")
        assert rule["enforced_in"] and rule["pinned_by"]


# ══════════════════════════════════════════════════════════════════════════════
#  T-U4: CRC-16/CCITT-FALSE, and the field-set CRC construction
# ══════════════════════════════════════════════════════════════════════════════

def test_crc_parameters_are_the_shared_variant():
    """poly 0x1021, init 0xFFFF, no reflection, no final XOR.

    The clapboard chose this variant precisely because the Jetson↔Teensy UDP
    protocol already had it (§8.4: "both sides reuse an existing
    implementation"), so a different variant on either side rejects every commit.
    """
    spec = VECTORS["crc16_ccitt_false"]
    assert spec["poly"] == "0x1021"
    assert spec["init"] == "0xFFFF"
    assert spec["reflect_in"] is False and spec["reflect_out"] is False
    assert spec["xor_out"] == "0x0000"


@pytest.mark.parametrize("case", VECTORS["crc16_ccitt_false"]["known_answers"],
                         ids=_ids(VECTORS["crc16_ccitt_false"]["known_answers"]))
def test_crc_known_answers(case):
    """The canonical check value and the empty-input case, against the one
    CRC implementation this repo has."""
    assert p.crc16_ccitt(bytes.fromhex(case["input_hex"])) == int(case["crc16"], 16)


@pytest.mark.parametrize("case", VECTORS["crc16_over_fields"],
                         ids=_ids(VECTORS["crc16_over_fields"]))
def test_crc_over_field_sets(case):
    """``crc16_over_fields`` reproduces every shared field-set vector.

    Fails on any change to the CRC's INPUT construction — trimming instead of
    NUL-padding to 32 bytes, hashing absent fields, or hashing in anything other
    than ascending ``field_id`` order — none of which a round-trip test would
    catch, because both ends of a round trip would change together.
    """
    ids, values = _fields_of(case)
    got = cs.crc16_over_fields(cs.field_buffers(ids, values))
    assert got == int(case["crc16"], 16), (
        f"{case['name']}: live CRC 0x{got:04X} != fixture {case['crc16']}")


def test_absent_fields_contribute_nothing_even_when_stale():
    """The relation the peer asserts, held here against the shared literals.

    Patch semantics mean a panel field buffer holds the PREVIOUS transaction's
    value; if an absent field contributed, every commit after the first would
    CRC differently on the two sides.
    """
    by_name = {c["name"]: c for c in VECTORS["crc16_over_fields"]}
    only = by_name["peer/single-field-scene"]
    stale = by_name["peer/stale-field-now-present"]
    assert only["crc16"] != stale["crc16"], (
        "adding field 3 to the present set did not change the CRC — the mask is "
        "not being honoured")
    good = by_name["peer/bit-flip-good"]
    bad = by_name["peer/bit-flip-corrupted"]
    assert good["crc16"] != bad["crc16"], "a single flipped bit went undetected"


# ══════════════════════════════════════════════════════════════════════════════
#  Byte-exact frame goldens
# ══════════════════════════════════════════════════════════════════════════════

def _frames_of_kind(kind):
    return [f for f in VECTORS["frames"] if f["kind"] == kind]


@pytest.mark.parametrize("frame", _frames_of_kind("field_chunk"),
                         ids=_ids(_frames_of_kind("field_chunk")))
def test_field_chunk_bytes(frame):
    """``build_field_chunks`` reproduces the shared ``CLAP_FIELD`` bytes exactly.

    Includes the peer's own layout golden (field 5 / seq 3 → ``d[0] == 0x35``),
    carried across so the nibble packing cannot drift apart.
    """
    buffer32 = bytes.fromhex(frame["buffer32"])
    got = cs.build_field_chunks(frame["field_id"], buffer32)[frame["seq"]]
    assert got == bytes.fromhex(frame["data"]), (
        f"{frame['name']}: live {got.hex()} != fixture {frame['data']}")
    assert int(frame["can_id"], 16) == cb.FIELD_ID


@pytest.mark.parametrize("frame", _frames_of_kind("commit"),
                         ids=_ids(_frames_of_kind("commit")))
def test_commit_bytes(frame):
    """``build_commit`` reproduces the shared ``CLAP_COMMIT`` bytes exactly."""
    flags = int(frame["flags"], 16)
    assert flags in (0x00, cs.COMMIT_FLAG_FULL_REFRESH), (
        f"{frame['name']} sets flag bits beyond bit 0. protocol.md 8.4 allocates "
        "only bit0 (force full refresh); a new bit needs a build_commit argument "
        "before a vector can exercise it.")
    got = cs.build_commit(frame["template_id"], int(frame["present_mask"], 16),
                          frame["txn_id"], int(frame["crc16"], 16),
                          force_full_refresh=bool(flags & cs.COMMIT_FLAG_FULL_REFRESH))
    assert got == bytes.fromhex(frame["data"]), (
        f"{frame['name']}: live {got.hex()} != fixture {frame['data']}")
    assert int(frame["can_id"], 16) == cb.COMMIT_ID


@pytest.mark.parametrize("frame", _frames_of_kind("ack"),
                         ids=_ids(_frames_of_kind("ack")))
def test_ack_decodes_to_the_shared_values(frame):
    """``ClapboardAck`` reads the peer's own emitted bytes as the peer meant them."""
    a = cb.ClapboardAck.from_can_frame(bytes.fromhex(frame["data"]))
    assert (a.txn_id, a.outcome, a.state, a.render_ms) == (
        frame["txn_id"], frame["outcome"], frame["state"], frame["render_ms"])
    assert int(frame["can_id"], 16) == cb.ACK_ID


@pytest.mark.parametrize("frame", _frames_of_kind("heartbeat"),
                         ids=_ids(_frames_of_kind("heartbeat")))
def test_heartbeat_decodes_to_the_shared_values(frame):
    """``ClapboardHeartbeat`` reads the peer's own emitted bytes correctly.

    Every flag bit is checked individually: a shifted flag mask is invisible to a
    test that only compares the raw byte.
    """
    h = cb.ClapboardHeartbeat.from_can_frame(bytes.fromhex(frame["data"]))
    assert h.state == frame["state"]
    assert h.fires_since_boot == frame["fires_since_boot"]
    assert h.rail_mv == frame["rail_mv"]
    assert h.active_template_id == frame["active_template_id"]
    assert h.last_error == frame["last_error"]
    for flag in ("fire_ready", "template_loaded", "wifi_up", "rail_ok",
                 "time_synced"):
        assert getattr(h, flag) is frame[flag], f"{frame['name']}: {flag}"
    assert int(frame["can_id"], 16) == cb.HEARTBEAT_ID


@pytest.mark.parametrize("frame", _frames_of_kind("fire_event"),
                         ids=_ids(_frames_of_kind("fire_event")))
def test_fire_event_decodes_to_the_shared_values(frame):
    """The 48-bit timestamp comes off the wire as the LOW 48 BITS, unreconstructed.

    ``wall_us_low48`` is deliberately the raw field: the overflow vector proves a
    clock above 2^48 cannot bleed into ``fires_since_boot``, which is the rule the
    peer's mask enforces and ``protocol.md`` §8.8 never states.
    """
    e = cb.ClapboardFireEvent.from_can_frame(bytes.fromhex(frame["data"]))
    assert e.wall_us_low48 == frame["wall_us_low48"]
    assert e.fires_since_boot == frame["fires_since_boot"]
    assert e.wall_us_low48 < (1 << 48)
    assert int(frame["can_id"], 16) == cb.FIRE_EVENT_ID


@pytest.mark.parametrize("frame", _frames_of_kind("link"),
                         ids=_ids(_frames_of_kind("link")))
def test_link_frame_layout(frame):
    """``CLAP_LINK`` is byte 0 plus seven reserved zeros — and DLC 8.

    The emitter is firmware (``clap_link.cpp``), so the compiled pin lives in
    ``tests/firmware/native/test_clap_link.cpp`` and the compiler-free one in
    ``tests/firmware/test_clap_wire_dlc_lint.py``. What is checked HERE is the
    shared layout: reserved bytes zero, and byte 0 non-zero exactly when the link
    is up — including the vector this repo never emits but the peer must still
    read as UP (a future flag bit in byte 0 must not blank the panel).
    """
    data = bytes.fromhex(frame["data"])
    assert data[1:] == b"\x00" * 7, "bytes 1-7 are reserved and must be 0 (8.5)"
    assert bool(data[0]) is frame["ros2_up"]
    if frame["emitted_by_this_repo"]:
        assert data[0] in (0, 1), (
            "the bridge emits only 0 or 1 in byte 0; anything else is a new "
            "allocation and needs a protocol.md 8.5 change on both sides")
    assert int(frame["can_id"], 16) == cb.LINK_ID


@pytest.mark.parametrize("frame", _frames_of_kind("time_sync"),
                         ids=_ids(_frames_of_kind("time_sync")))
def test_time_sync_frame_layout(frame):
    """``0x7DD`` is ``pack('<II', sec, usec)`` — and DLC 8 like everything else.

    Not a clapboard-block id, but the bridge emits it on this same bus and the
    peer's ``decode_time_sync`` requires DLC 8 for it too, so it is in scope for
    the emission invariant. ``usec >= 1e6`` is REJECTED by the peer rather than
    folded in: a wrong time anchor silently corrupts an edit, a missing one is
    recoverable in post.
    """
    data = bytes.fromhex(frame["data"])
    unix_s, usec = struct.unpack('<II', data)
    assert unix_s == frame["unix_s"]
    assert usec == frame["usec"]
    assert usec < 1_000_000
    assert int(frame["can_id"], 16) == proto.CAN_ID_SHARED_TIME_SYNC


# ══════════════════════════════════════════════════════════════════════════════
#  Whole-transaction goldens — the strongest drift guard
# ══════════════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize("txn", VECTORS["transactions"],
                         ids=_ids(VECTORS["transactions"]))
def test_transaction_bytes(txn):
    """``build_transaction`` reproduces a whole shared transaction, frame for frame.

    This is the single assertion that fails on a change to the chunking, the CRC
    input construction, the frame layouts, the frame ORDER, or the commit-last
    rule — every semantic this repo owns for the downlink, in one comparison.
    """
    ids, values = _fields_of(txn)
    got = [(can_id, bytes(payload)) for can_id, payload in cs.build_transaction(
        txn["template_id"], ids, values, txn["txn_id"],
        force_full_refresh=txn["force_full_refresh"])]
    expected = [(int(f["can_id"], 16), bytes.fromhex(f["data"]))
                for f in txn["frames"]]
    assert len(got) == txn["frame_count"], (
        f"{txn['name']}: built {len(got)} frames, fixture says "
        f"{txn['frame_count']}")
    assert got == expected, (
        f"{txn['name']}: transaction bytes diverged from the shared fixture.\n"
        "  first mismatch: " + next(
            (f"index {i}: live 0x{g[0]:03X}/{g[1].hex()} != fixture "
             f"0x{e[0]:03X}/{e[1].hex()}"
             for i, (g, e) in enumerate(zip(got, expected)) if g != e),
            "length only"))


@pytest.mark.parametrize("txn", VECTORS["transactions"],
                         ids=_ids(VECTORS["transactions"]))
def test_transaction_puts_the_commit_last(txn):
    """The commit is the LAST frame, always.

    Chunks may arrive in any order; a ``CLAP_COMMIT`` that arrives before its
    chunks is an ``INCOMPLETE`` rejection. One RPC drained FIFO one-per-tick
    guarantees the ordering **only** if the host puts the commit last — there is
    no ordering logic in the firmware to fall back on.
    """
    can_ids = [int(f["can_id"], 16) for f in txn["frames"]]
    assert can_ids[-1] == cb.COMMIT_ID
    assert can_ids.count(cb.COMMIT_ID) == 1
    assert all(cid == cb.FIELD_ID for cid in can_ids[:-1])


def test_a_present_but_empty_field_still_ships_all_five_chunks():
    """The ``all-five-chunks`` unstated rule, on its hardest case.

    Clearing a field is a PRESENT field with an all-NUL buffer. A sender that
    optimised away "there is nothing to send" would leave the previous value on
    the panel and CRC-mismatch against a receiver that padded — the exact failure
    the fixture's ``full-slate-8-fields`` vector (field 3 empty) pins.
    """
    txn = next(t for t in VECTORS["transactions"]
               if t["name"] == "full-slate-8-fields")
    assert txn["fields"]["3"] == ""
    field_frames = [f for f in txn["frames"]
                    if int(f["can_id"], 16) == cb.FIELD_ID]
    for_field_3 = [f for f in field_frames
                   if bytes.fromhex(f["data"])[0] & 0x0F == 3]
    assert len(for_field_3) == cs.CHUNKS_PER_FIELD


def test_a_repaint_only_commit_carries_the_init_crc():
    """The ``empty-field-set-crc`` unstated rule.

    A commit naming no field is a legitimate repaint. Its CRC is the untouched
    init value, which both sides must agree on or every repaint is rejected.
    """
    txn = next(t for t in VECTORS["transactions"]
               if t["name"] == "repaint-only-no-fields")
    assert txn["fields"] == {}
    assert txn["present_mask"] == "0x00"
    assert txn["crc16"] == "0xFFFF"
    frames = cs.build_transaction(txn["template_id"], [], [], txn["txn_id"],
                                  force_full_refresh=txn["force_full_refresh"])
    assert len(frames) == 1
    assert frames[0][1] == bytes.fromhex(txn["frames"][0]["data"])


# ══════════════════════════════════════════════════════════════════════════════
#  T-U8: the DLC-8 invariant, BOTH directions
# ══════════════════════════════════════════════════════════════════════════════
#  The rule the clapboard enforces in code and states in no document: every frame
#  in this contract is exactly 8 bytes, and a receiver drops any other length
#  SILENTLY. Nothing on either side reports it — the panel simply sits in
#  screensaver. It is the plan's named most-likely-first-integration-bug.

def test_every_fixture_frame_is_dlc8():
    """The shared artefact itself may not contain a non-8 frame."""
    for frame in VECTORS["frames"]:
        data = bytes.fromhex(frame["data"])
        assert frame["dlc"] == 8, frame["name"]
        assert len(data) == 8, f"{frame['name']} is {len(data)} bytes"
    for txn in VECTORS["transactions"]:
        for i, frame in enumerate(txn["frames"]):
            assert frame["dlc"] == 8 and len(bytes.fromhex(frame["data"])) == 8, (
                f"{txn['name']} frame {i}")


@pytest.mark.parametrize("template_id,fields", [
    (0, {}),                                      # repaint only: commit alone
    (15, {0: ""}),                                # a cleared field
    (3, {7: "X"}),                                # 1 byte — still five chunks
    (3, {2: "1234567"}),                          # exactly one chunk of text
    (3, {2: "12345678"}),                         # one byte past a chunk boundary
    (5, {0: "0123456789abcdefghijklmnopqrstuv"}),  # exactly MAX_FIELD_BYTES
    (1, {i: "f%d" % i for i in range(8)}),        # all eight fields present
])
def test_every_emitted_frame_is_dlc8(template_id, fields):
    """EMIT direction: nothing this repo builds is ever anything but 8 bytes.

    Swept across the shapes where a length bug would hide — an empty value, a
    value that exactly fills one chunk, one byte past a chunk boundary, a full
    32-byte value, and the full eight-field slate.
    """
    ids = sorted(fields)
    frames = cs.build_transaction(template_id, ids, [fields[i] for i in ids], 1)
    assert frames, "a transaction always has at least its commit"
    for can_id, payload in frames:
        assert len(payload) == cs.DLC, (
            f"can_id 0x{can_id:03X} payload is {len(payload)} bytes. The "
            "clapboard drops any DLC != 8 silently — no error on either side.")


def test_the_dlc_reaches_the_rpc_wire_as_8():
    """EMIT direction, one layer down: the RPC reports DLC 8 per slot.

    ``encode_clap_send`` deliberately reports a short payload's length HONESTLY
    rather than promoting it to 8 — quietly "fixing" it there would hide the bug
    at the one layer that could still name it. So the invariant has to hold at
    the builder, and this asserts it survives the encode.
    """
    frames = cs.build_transaction(3, list(range(8)), ["v%d" % i for i in range(8)],
                                  200)
    arg = p.ArgClapSend.unpack(ra.encode_clap_send(frames))
    assert arg.count == len(frames) == cs.MAX_FRAMES_PER_TRANSACTION
    assert all(arg.len[i] == 8 for i in range(arg.count)), (
        f"CLAP_SEND slots report DLCs {arg.len[:arg.count]}")
    # Unused slots stay zeroed — the firmware reads only the first `count`.
    assert all(arg.len[i] == 0 for i in range(arg.count, p.CLAP_MAX_FRAMES))


def test_the_nibble_bounds_the_peer_decoder_enforces_are_never_emitted():
    """EMIT direction: byte 0's two nibbles stay inside the peer's accepted range.

    ``decode_field_chunk`` rejects ``field_id >= 8`` and ``seq >= 5`` outright, so
    a chunker that overflowed either nibble would produce frames the panel drops
    with no diagnostic. Validation refuses ``field_id > 7`` at the boundary; this
    pins the other nibble too.
    """
    frames = cs.build_transaction(3, list(range(8)), ["x" * 32] * 8, 9)
    for can_id, payload in frames:
        if can_id != cb.FIELD_ID:
            continue
        assert payload[0] & 0x0F < cs.MAX_FIELDS
        assert (payload[0] >> 4) & 0x0F < cs.CHUNKS_PER_FIELD


def _decoder_classes():
    """Every decoder ``jugglebot.can.clapboard`` owns, found by introspection.

    Discovered rather than listed so a decoder added later inherits the strict
    check automatically — the rule is a property of the CONTRACT, not of the
    three classes that happen to exist today.
    """
    return sorted(
        (name for name, obj in vars(cb).items()
         if isinstance(obj, type) and hasattr(obj, "from_can_frame")
         and getattr(obj, "__module__", None) == cb.__name__),
    )


def test_the_known_decoders_are_all_discovered():
    """Guards the introspection above against a rename emptying it silently."""
    found = set(_decoder_classes())
    assert {"ClapboardHeartbeat", "ClapboardFireEvent",
            "ClapboardAck"} <= found, found


@pytest.mark.parametrize("cls_name", _decoder_classes())
@pytest.mark.parametrize("length", [0, 1, 4, 7, 9, 16])
def test_every_decoder_rejects_dlc_not_8(cls_name, length):
    """DECODE direction: a frame of any other length is refused, not tolerated.

    Stricter than ``catching_cone``'s ``len < 8``, deliberately: the peer's
    decoders require exactly 8 and drop everything else, so a 9-byte frame is
    something the other end would never have sent — i.e. corruption. Accepting it
    would decode corrupted bytes into an operator-facing topic.
    """
    cls = getattr(cb, cls_name)
    with pytest.raises(ValueError):
        cls.from_can_frame(b"\x00" * length)


#: The three uplink kinds and the decoder each maps to. Kept beside the frames
#: rather than discovered, because this mapping IS the contract being asserted.
_UPLINK_DECODERS = {
    "ack": "ClapboardAck",
    "heartbeat": "ClapboardHeartbeat",
    "fire_event": "ClapboardFireEvent",
}
_UPLINK_FRAMES = [f for f in VECTORS["frames"] if f["kind"] in _UPLINK_DECODERS]


@pytest.mark.parametrize("frame", _UPLINK_FRAMES, ids=_ids(_UPLINK_FRAMES))
def test_uplink_vectors_decode_at_exactly_8(frame):
    """DECODE direction, positive half: the shared bytes are accepted as-is.

    The rejection test above proves nothing on its own — a decoder that raised on
    *everything* would pass it. This is its complement.
    """
    cls = getattr(cb, _UPLINK_DECODERS[frame["kind"]])
    assert cls.from_can_frame(bytes.fromhex(frame["data"])) is not None
