"""Unit tests for jugglebot.clapboard_slate — the clapboard wire builder.

T-U3 (chunking), T-U4 (CRC) and T-U7 (action-boundary validation) of
``plans/active/clapboard-can3-integration.md``.

The module under test has NO ROS imports, so nothing here needs the mocked-ROS
machinery — but it lives in tests/ros/ because that is where the ``jugglebot``
package's tests live (``tests/ros/test_clapboard.py``, its decoder sibling, is
the same case).

Every expectation below is traceable to ``Electronic-Clapboard
docs/protocol.md`` sections 8.3 and 8.4, which are NORMATIVE and
cross-repository, cross-checked against that repo's ``src/can_frames.h``
(the encoders/decoders and the ``crc16_over_fields`` reference).
"""

from __future__ import annotations

import pytest

from teensy_link import protocol as p
from teensy_link import rpc_args

from jugglebot import clapboard_slate as cs
from jugglebot.can import clapboard


# ── The constants themselves ───────────────────────────────────

def test_field_model_constants_match_the_peer():
    """The chunking geometry is copied from protocol.md 8.3, not invented."""
    assert cs.MAX_FIELDS == 8
    assert cs.MAX_FIELD_BYTES == 32
    assert cs.CHUNK_PAYLOAD == 7
    assert cs.CHUNKS_PER_FIELD == 5
    assert cs.DLC == 8
    # 5 x 7 = 35 > 32, so a maximum-length field always leaves >= 3 trailing
    # NULs — which is how the receiver finds the terminator with no length field.
    assert cs.CHUNKS_PER_FIELD * cs.CHUNK_PAYLOAD - cs.MAX_FIELD_BYTES >= 3
    # 8 fields x 5 chunks + 1 commit.
    assert cs.MAX_FRAMES_PER_TRANSACTION == 41
    # ...and the RPC that carries a whole transaction has room for it.
    assert cs.MAX_FRAMES_PER_TRANSACTION <= p.CLAP_MAX_FRAMES


# ── T-U4: CRC ──────────────────────────────────────────────────

def test_crc_golden_vector():
    """CRC-16/CCITT-FALSE, verified against the canonical check value.

    poly 0x1021, init 0xFFFF, no input or output reflection, no final XOR — the
    same variant the Jetson<->Teensy UDP protocol already uses, which is exactly
    why the clapboard chose it (protocol.md 8.4: "both sides reuse an existing
    implementation").
    """
    assert p.crc16_ccitt(b'123456789') == 0x29B1


def test_crc_is_over_nul_padded_fixed_width_buffers():
    """The CRC covers each present field's 32-byte NUL-PADDED buffer.

    Padding to a fixed width rather than hashing the trimmed string is what makes
    the CRC independent of chunk arrival order — the sender cannot know what the
    receiver's trimmed view will be until reassembly completes.
    """
    buffers = cs.field_buffers([2], ['TAKE 12'])
    assert len(buffers[2]) == 32
    assert buffers[2] == b'TAKE 12'.ljust(32, b'\x00')
    assert cs.crc16_over_fields(buffers) == p.crc16_ccitt(b'TAKE 12'.ljust(32, b'\x00'))


def test_crc_is_order_independent_in_the_caller():
    """Presenting the same field set in a different caller order is the same CRC.

    Ascending field_id ordering is applied inside crc16_over_fields, so a caller
    that lists fields out of order cannot produce a different check value from
    one that does not.
    """
    a = cs.crc16_over_fields(cs.field_buffers([0, 3, 5], ['SC 4', 'T 12', 'WIDE']))
    b = cs.crc16_over_fields(cs.field_buffers([5, 0, 3], ['WIDE', 'SC 4', 'T 12']))
    assert a == b


def test_crc_excludes_absent_fields_entirely():
    """An absent field contributes NOTHING — not zeros, nothing.

    That is what makes the same patch produce the same CRC regardless of how many
    other fields the panel happens to be displaying.
    """
    only_one = cs.crc16_over_fields(cs.field_buffers([1], ['ROLL A']))
    assert only_one == p.crc16_ccitt(b'ROLL A'.ljust(32, b'\x00'))
    # Adding a second present field changes it (the concatenation grew).
    two = cs.crc16_over_fields(cs.field_buffers([1, 4], ['ROLL A', 'DAY']))
    assert two != only_one
    assert two == p.crc16_ccitt(b'ROLL A'.ljust(32, b'\x00')
                                + b'DAY'.ljust(32, b'\x00'))


def test_crc_of_an_empty_field_set_is_the_init_value():
    """A repaint-only commit hashes nothing, so the CRC is the 0xFFFF init value.

    Matches the peer's crc16_over_fields with present_mask == 0, which folds no
    bytes in and returns its initial crc.
    """
    assert cs.crc16_over_fields({}) == 0xFFFF


def test_crc_detects_a_single_bit_flip():
    """One flipped bit anywhere in the payload changes the check value."""
    good = cs.field_buffers([0], ['SCENE 4'])
    flipped = {0: bytes([good[0][0] ^ 0x01]) + good[0][1:]}
    assert cs.crc16_over_fields(good) != cs.crc16_over_fields(flipped)


def test_crc_distinguishes_which_field_carried_the_text():
    """The same text under a different field_id is a different transaction.

    Concatenation order is by field_id, so moving a value between fields moves it
    within the hashed blob whenever another field is present.
    """
    a = cs.crc16_over_fields(cs.field_buffers([0, 1], ['A', 'B']))
    b = cs.crc16_over_fields(cs.field_buffers([0, 1], ['B', 'A']))
    assert a != b


# ── T-U3: chunking ─────────────────────────────────────────────

def _max_goal():
    """8 fields x 32 characters — the worst-case transaction."""
    ids = list(range(cs.MAX_FIELDS))
    values = [f'{i}' + 'X' * (cs.MAX_FIELD_BYTES - 1) for i in ids]
    return ids, values


def test_full_transaction_is_41_frames_with_the_commit_last():
    """8 fields x 5 chunks + 1 commit, and THE COMMIT IS LAST.

    Ordering is the whole guarantee against an INCOMPLETE rejection: chunks may
    arrive in any order, but a CLAP_COMMIT that lands before its chunks is
    rejected, and there is no ordering logic in the firmware to fall back on —
    the bridge drains this list FIFO exactly as given.
    """
    ids, values = _max_goal()
    frames = cs.build_transaction(3, ids, values, txn_id=7)
    assert len(frames) == 41
    assert [can_id for can_id, _ in frames[:-1]] == [clapboard.FIELD_ID] * 40
    assert frames[-1][0] == clapboard.COMMIT_ID
    # ...and nothing but the last frame is a commit.
    assert clapboard.COMMIT_ID not in [can_id for can_id, _ in frames[:-1]]


def test_every_frame_is_dlc_8():
    """Every clapboard frame, both directions, is exactly 8 bytes.

    The panel's decoders test ``len != 8`` and drop silently — no error on either
    side. protocol.md never states this; only can_frames.h enforces it, and the
    firmware below us reports a short payload's DLC honestly rather than
    promoting it, so emitting 8 is THIS layer's job.
    """
    ids, values = _max_goal()
    for _can_id, payload in cs.build_transaction(3, ids, values, txn_id=1):
        assert len(payload) == cs.DLC
    # ...and for a short value too, where the temptation to trim is greatest.
    for _can_id, payload in cs.build_transaction(0, [0], ['A'], txn_id=1):
        assert len(payload) == cs.DLC


def test_chunk_byte0_packs_field_id_and_seq_nibbles():
    """byte 0 = (field_id & 0x0F) | ((seq & 0x0F) << 4), protocol.md 8.3."""
    chunks = cs.build_field_chunks(5, b'HELLO'.ljust(32, b'\x00'))
    assert len(chunks) == cs.CHUNKS_PER_FIELD
    for seq, payload in enumerate(chunks):
        assert payload[0] & 0x0F == 5
        assert (payload[0] >> 4) & 0x0F == seq


def test_chunks_reassemble_to_the_padded_buffer():
    """Concatenating the 5 chunks' text yields the 32-byte buffer plus 3 NULs."""
    buf = b'SCENE 4'.ljust(32, b'\x00')
    text = b''.join(payload[1:] for payload in cs.build_field_chunks(1, buf))
    assert len(text) == cs.CHUNKS_PER_FIELD * cs.CHUNK_PAYLOAD == 35
    assert text[:32] == buf
    assert text[32:] == b'\x00\x00\x00'


def test_a_maximum_length_field_still_ends_in_three_nuls():
    """32 characters is the maximum precisely so the terminator always fits."""
    buf = ('X' * 32).encode()
    text = b''.join(payload[1:] for payload in cs.build_field_chunks(0, buf))
    assert text[:32] == buf
    assert text[32:] == b'\x00\x00\x00'


def test_short_fields_still_send_all_five_chunks():
    """A two-character value is still five chunks — required, not defensive.

    Panel field buffers PERSIST between transactions (protocol.md 8.4: fields not
    named in the mask retain their previous value). Sending only the occupied
    prefix would leave the PREVIOUS value's tail in the receiver's buffer, which
    both renders wrong text and produces a spurious CRC_MISMATCH against a sender
    that hashed a NUL-padded buffer.
    """
    frames = cs.build_transaction(0, [0], ['AB'], txn_id=9)
    assert len(frames) == cs.CHUNKS_PER_FIELD + 1
    seqs = [(payload[0] >> 4) & 0x0F for _id, payload in frames[:-1]]
    assert seqs == [0, 1, 2, 3, 4]
    # The tail chunks are pure NUL padding — that is the point.
    assert frames[4][1][1:] == b'\x00' * 7


def test_commit_layout_matches_protocol_8_4():
    """template_id / mask / flags / txn_id / crc16 LE / two reserved zero bytes."""
    frames = cs.build_transaction(11, [0, 3], ['SCENE 4', 'WIDE'], txn_id=200,
                                  force_full_refresh=True)
    can_id, commit = frames[-1]
    assert can_id == clapboard.COMMIT_ID
    assert commit[0] == 11                       # template_id
    assert commit[1] == 0b0000_1001              # mask: fields 0 and 3
    assert commit[2] == cs.COMMIT_FLAG_FULL_REFRESH
    assert commit[3] == 200                      # txn_id
    crc = int.from_bytes(commit[4:6], 'little')  # LE
    assert crc == cs.crc16_over_fields(cs.field_buffers([0, 3], ['SCENE 4', 'WIDE']))
    assert commit[6:8] == b'\x00\x00'            # reserved, MUST be 0


def test_commit_flags_zero_without_force_full_refresh():
    frames = cs.build_transaction(1, [0], ['A'], txn_id=1)
    assert frames[-1][1][2] == 0x00


def test_empty_field_set_is_a_repaint_only_commit():
    """No fields is legal: one commit frame, mask 0 — 'repaint what is up there'.

    Useful with force_full_refresh to clear accumulated e-paper ghosting without
    resending any text.
    """
    frames = cs.build_transaction(2, [], [], txn_id=4, force_full_refresh=True)
    assert len(frames) == 1
    can_id, commit = frames[0]
    assert can_id == clapboard.COMMIT_ID
    assert commit[1] == 0x00
    assert int.from_bytes(commit[4:6], 'little') == 0xFFFF


def test_transaction_fits_the_clap_send_rpc_and_keeps_its_order():
    """The worst case encodes into one CLAP_SEND with the commit still last.

    encode_clap_send preserves order exactly (the bridge drains FIFO one frame
    per tick), so this is the end-to-end proof that the commit reaches the wire
    after its chunks.
    """
    ids, values = _max_goal()
    frames = cs.build_transaction(3, ids, values, txn_id=17)
    args = rpc_args.encode_clap_send(frames)
    arg = p.ArgClapSend.unpack(args)
    assert arg.count == 41
    assert list(arg.can_id[:41]) == [clapboard.FIELD_ID] * 40 + [clapboard.COMMIT_ID]
    assert list(arg.len[:41]) == [8] * 41
    # The last used slot's payload is the commit we built.
    assert bytes(arg.data[40 * 8:41 * 8]) == frames[-1][1]


# ── T-U7: action-boundary validation ───────────────────────────

def test_field_id_above_seven_rejected():
    """field_id > 7 is refused HERE, not after a bus round-trip and a 3 s render.

    The panel would answer BAD_FIELD_ID eventually; catching it at the boundary
    names the offending index instead.
    """
    with pytest.raises(cs.SlateValidationError) as exc:
        cs.validate_slate(0, [0, 9], ['A', 'B'])
    assert 'field_ids[1]' in str(exc.value)
    assert '9' in str(exc.value)


def test_negative_field_id_rejected():
    with pytest.raises(cs.SlateValidationError):
        cs.validate_slate(0, [-1], ['A'])


def test_duplicate_field_id_rejected():
    """Two values for one field cannot both survive — the mask holds it once."""
    with pytest.raises(cs.SlateValidationError) as exc:
        cs.validate_slate(0, [2, 2], ['A', 'B'])
    assert 'duplicate' in str(exc.value).lower()


def test_string_over_32_bytes_rejected():
    with pytest.raises(cs.SlateValidationError) as exc:
        cs.validate_slate(0, [0], ['X' * 33])
    assert '33 bytes' in str(exc.value)


def test_string_of_exactly_32_bytes_accepted():
    """32 is the maximum, not one past it — the boundary is inclusive."""
    cs.validate_slate(0, [0], ['X' * 32])


def test_multibyte_text_is_bounded_in_bytes_not_characters():
    """The panel buffer is 32 BYTES and the CRC is over bytes.

    protocol.md 8.3 says "32 characters" and "text is ASCII", but the clapboard
    renders bytes >= 0x80 as '?' rather than rejecting a take label, so non-ASCII
    is accepted here too — and then the binding constraint is the encoded byte
    count, which is what the buffer and the CRC actually see.
    """
    # 32 characters, 34 bytes in UTF-8 — over budget despite the character count.
    with pytest.raises(cs.SlateValidationError) as exc:
        cs.validate_slate(0, [0], ['A' * 30 + 'éé'])
    assert '34 bytes' in str(exc.value)
    assert '32 characters' in str(exc.value)
    # ...but a short non-ASCII label is fine, not rejected.
    cs.validate_slate(0, [0], ['SCÈNE 4'])


def test_embedded_nul_rejected():
    """A NUL inside the text is read by the panel as the string terminator.

    Accepting one would silently truncate the value with no error anywhere on
    either side, which is the failure mode this whole module exists to avoid.
    """
    with pytest.raises(cs.SlateValidationError) as exc:
        cs.validate_slate(0, [0], ['TAKE\x0012'])
    assert 'NUL' in str(exc.value)


def test_mismatched_parallel_array_lengths_rejected():
    with pytest.raises(cs.SlateValidationError) as exc:
        cs.validate_slate(0, [0, 1], ['A'])
    assert 'parallel' in str(exc.value)


def test_more_than_eight_fields_rejected():
    with pytest.raises(cs.SlateValidationError):
        cs.validate_slate(0, list(range(8)) + [0], ['A'] * 9)


def test_template_id_above_15_rejected():
    with pytest.raises(cs.SlateValidationError) as exc:
        cs.validate_slate(16, [0], ['A'])
    assert 'template_id' in str(exc.value)


def test_non_string_value_rejected():
    with pytest.raises(cs.SlateValidationError):
        cs.validate_slate(0, [0], [12])


def test_validation_runs_before_any_frame_is_built():
    """build_transaction validates FIRST — a malformed goal never yields bytes."""
    with pytest.raises(cs.SlateValidationError):
        cs.build_transaction(0, [9], ['A'], txn_id=1)


def test_host_outcome_tokens_never_collide_with_the_panel_enum():
    """The host-only outcome names are disjoint from every CLAP_ACK outcome.

    This is the whole reason SetSlate.Result.outcome is a STRING and not the wire
    uint8: the outcome enum belongs to the clapboard repo, so a host-minted
    TIMEOUT inside that byte would collide the moment they allocate 0x07. Keeping
    the two vocabularies disjoint means a reader never has to ask which side said
    it — and this test fails if the peer ever names an outcome the same thing.
    """
    host = {cs.OUTCOME_INVALID_GOAL, cs.OUTCOME_DISPATCH_FAILED,
            cs.OUTCOME_TIMEOUT}
    panel = {m.name for m in clapboard.ClapboardAckOutcome}
    assert host & panel == set()


def test_slate_validation_error_is_a_value_error():
    """Callers catching ValueError still catch it; the subclass only adds precision."""
    assert issubclass(cs.SlateValidationError, ValueError)


# ── txn_id allocation ──────────────────────────────────────────

def test_txn_ids_increment_and_wrap_without_ever_being_zero():
    """0 is never allocated.

    CLAP_ACK decodes txn_id from a wire byte, so an all-zero frame — a mis-seated
    transceiver, a padded relay — reads as 'txn 0, outcome OK'. If 0 were
    allocatable, such a frame could satisfy a live transaction and report a
    phantom success on a panel that never painted.
    """
    alloc = cs.TxnIdAllocator()
    seen = [alloc.allocate() for _ in range(255)]
    assert seen[0] == 1
    assert seen[-1] == 255
    assert 0 not in seen
    assert len(set(seen)) == 255          # every value used once per cycle
    assert alloc.allocate() == 1          # wraps back to 1, not 0
