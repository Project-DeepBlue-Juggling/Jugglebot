"""Electronic Clapboard slate transactions: validation, chunking, CRC, ``txn_id``.

Everything that decides **which bytes reach another repository's device** lives
here, in one pure-Python module with **no ROS imports** — so the whole wire-
building path is unit-testable without the mocked-ROS conftest, and so the
single most cross-repo-sensitive code in this feature can be read on its own.
``teensy_bridge_node`` imports this and does transport only.

The can-bridge Teensy is a **mechanical relay** for this traffic: bytes in,
frames out, no reassembly, no CRC, no field-model awareness. That is deliberate
— it is why a clapboard protocol change never needs a Teensy reflash. The
consequence is that every semantic rule below is enforced here or nowhere.

**Wire contract:** ``Electronic-Clapboard docs/protocol.md`` §8.3 (``CLAP_FIELD``)
and §8.4 (``CLAP_COMMIT``) are NORMATIVE and cross-repository; neither side may
change them unilaterally. The peer implementation is that repo's
``src/can_frames.h``.

Four rules worth reading before touching anything here
------------------------------------------------------

1. **Every frame is exactly DLC 8.** The clapboard's decoders test
   ``len != 8`` and drop silently — no error on either side. ``protocol.md``
   never states this; only ``can_frames.h`` enforces it. The firmware below us
   bounds ``len <= 8`` for framing safety and reports a short payload's DLC
   honestly rather than promoting it, so emitting exactly 8 is *this module's*
   job.

2. **All five chunks are sent for every present field, always** — even for a
   two-character value. This is required for correctness, not defensive
   padding: field buffers on the panel PERSIST between transactions (§8.4's
   patch semantics), and the CRC is computed over the full **32-byte NUL-padded**
   buffer. Sending only the occupied prefix would leave the previous value's
   tail in the receiver's buffer, which both renders wrong text and produces a
   spurious ``CRC_MISMATCH`` against a sender that padded with NULs.

3. **The commit goes LAST.** Chunks may arrive in any order, but a
   ``CLAP_COMMIT`` that arrives before its chunks is an ``INCOMPLETE``
   rejection. One ``CLAP_SEND`` RPC carrying the whole transaction, drained FIFO
   one frame per tick, guarantees the ordering by construction — *provided the
   commit is last in the list*. There is no ordering logic in the firmware to
   fall back on.

4. **The date is never a field.** The clapboard fills it from its own wall
   clock, slaved to the bridge's 0x7DD broadcast. Sending one would race the
   Jetson's clock against the one the bridge is already distributing.

And one rule that is not about bytes: **there is no fire command**, here or on
either transport, by design. ROS2 gets a complete log of every clap; it does
not get to clap.

See ``plans/active/clapboard-can3-integration.md`` Phase 3 for the arc.
"""

from __future__ import annotations

import threading
from typing import Dict, List, Tuple

# The one CRC-16/CCITT-FALSE implementation in this repo. Reused rather than
# reimplemented — that reuse is precisely why the clapboard chose this variant
# ("both sides reuse an existing implementation", protocol.md §8.4). Parameters
# verified by reading config/generated/udp_protocol.h:636-645 against the peer's
# can_frames.h:151-161: poly 0x1021, init 0xFFFF, no input or output reflection,
# no final XOR. Golden vector "123456789" -> 0x29B1.
from teensy_link import protocol as _p

from jugglebot.can import clapboard

# ── Field model (protocol.md §8.3, mirroring the peer's can_frames.h) ────────
#: Fields are numbered 0..7 — ``field_present_mask`` is one byte.
MAX_FIELDS = 8
#: Maximum field text, in BYTES of the on-panel buffer (§8.3 says "32
#: characters"; the buffer is bytes and the CRC is over bytes, so bytes is the
#: binding constraint and the one this module validates).
MAX_FIELD_BYTES = 32
#: Text bytes carried by one ``CLAP_FIELD`` frame (byte 0 is the id/seq nibbles).
CHUNK_PAYLOAD = 7
#: Chunks per field, ``seq`` 0..4.
CHUNKS_PER_FIELD = 5
#: Every clapboard frame, both directions.
DLC = 8
#: ``template_id`` occupies the low nibble semantics of §8.4 byte 0 ("0-15").
MAX_TEMPLATE_ID = 15
#: ``CLAP_COMMIT`` byte 2, bit 0 — force a full refresh (clears e-paper ghosting).
COMMIT_FLAG_FULL_REFRESH = 0x01

# 5 x 7 = 35 bytes of chunk capacity against a 32-byte buffer, so a
# maximum-length field always leaves at least 3 trailing NULs — which is how the
# receiver finds the terminator without a length field. The peer holds this as a
# static_assert; here it is an import-time assert for the same reason.
assert CHUNKS_PER_FIELD * CHUNK_PAYLOAD > MAX_FIELD_BYTES, (
    "chunk capacity must exceed MAX_FIELD_BYTES so a NUL terminator always fits")
assert MAX_FIELDS <= 8, "field_present_mask is one byte"

#: Worst-case frames in one transaction: 8 fields x 5 chunks + 1 commit.
#: ``CLAP_MAX_FRAMES`` (48) sizes the RPC arg from exactly this number.
MAX_FRAMES_PER_TRANSACTION = MAX_FIELDS * CHUNKS_PER_FIELD + 1

# ── Host-side outcome tokens (SetSlate.Result.outcome) ──────────────────────
# These name failures that never reach the wire. They are STRINGS, and the
# action's ``outcome`` field is a string, because ``ClapboardAckOutcome`` is the
# clapboard repo's enum to allocate: minting a host-only TIMEOUT inside that
# uint8 would collide the moment they allocate 0x07.
OUTCOME_INVALID_GOAL = 'INVALID_GOAL'
OUTCOME_DISPATCH_FAILED = 'DISPATCH_FAILED'
OUTCOME_TIMEOUT = 'TIMEOUT'


class SlateValidationError(ValueError):
    """A slate goal that must be refused before any frame is built.

    Subclasses ``ValueError`` so generic callers still catch it, but is a
    distinct type so the action server can tell "the operator's goal is
    malformed" from "something else in the build path raised".
    """


def validate_slate(template_id, field_ids, field_values) -> None:
    """Refuse a malformed slate before a single byte is built.

    This is the action boundary the plan and the handoff doc both name: a
    ``field_id`` of 9 or a 40-character take label must be refused here with a
    message naming the offender, not shipped 41 frames and 3 s later as a bare
    ``BAD_FIELD_ID`` from the panel.

    **This must stay a PURE function of its three arguments.** The SetSlate
    action calls it twice — once in ``goal_callback`` to decide whether to claim
    the single-flight slot, once in ``execute_callback`` to produce the abort
    message — and relies on both calls reaching the same verdict. A rule that
    consulted mutable node state (a loaded template set, say) could disagree
    between the two, and ``execute_callback`` would then release a slot it never
    took, cancelling a slate push that is still in flight.

    Raises:
        SlateValidationError: with a message naming the exact offending index
            and value.
    """
    try:
        tid = int(template_id)
    except (TypeError, ValueError):
        raise SlateValidationError(f"template_id {template_id!r} is not an integer")
    if not 0 <= tid <= MAX_TEMPLATE_ID:
        raise SlateValidationError(
            f"template_id {tid} out of range 0..{MAX_TEMPLATE_ID} (protocol.md 8.4)")

    ids = list(field_ids)
    values = list(field_values)
    if len(ids) != len(values):
        raise SlateValidationError(
            f"field_ids has {len(ids)} entries but field_values has {len(values)}; "
            "they are parallel arrays and must be the same length")
    if len(ids) > MAX_FIELDS:
        raise SlateValidationError(
            f"{len(ids)} fields requested, at most {MAX_FIELDS} exist (protocol.md 8.3)")

    seen = set()
    for i, raw_id in enumerate(ids):
        try:
            fid = int(raw_id)
        except (TypeError, ValueError):
            raise SlateValidationError(f"field_ids[{i}] = {raw_id!r} is not an integer")
        if not 0 <= fid < MAX_FIELDS:
            raise SlateValidationError(
                f"field_ids[{i}] = {fid} out of range 0..{MAX_FIELDS - 1} "
                "(protocol.md 8.3)")
        if fid in seen:
            # Two values for one field cannot both survive: the mask holds the
            # field once, and whichever chunk set landed last would silently win.
            raise SlateValidationError(
                f"field_ids[{i}] = {fid} is a duplicate; each field may appear once")
        seen.add(fid)

    for i, value in enumerate(values):
        if not isinstance(value, str):
            raise SlateValidationError(
                f"field_values[{i}] = {value!r} is not a string")
        encoded = value.encode('utf-8')
        if len(encoded) > MAX_FIELD_BYTES:
            # Reported in bytes, and the character count too when they differ,
            # because a 32-character string with an em-dash in it is over budget
            # and the operator needs to be told which count bound.
            extra = (f" ({len(value)} characters)"
                     if len(encoded) != len(value) else "")
            raise SlateValidationError(
                f"field_values[{i}] is {len(encoded)} bytes{extra}, at most "
                f"{MAX_FIELD_BYTES} fit the panel buffer (protocol.md 8.3)")
        if b'\x00' in encoded:
            # The receiver finds the end of the text by scanning for a NUL, so an
            # embedded one silently truncates the value with no error anywhere.
            raise SlateValidationError(
                f"field_values[{i}] contains a NUL byte, which the panel reads "
                "as the string terminator")


def field_buffers(field_ids, field_values) -> Dict[int, bytes]:
    """Return ``{field_id: 32-byte NUL-padded buffer}`` for the present fields.

    The fixed-width padding is the whole point: it is what makes the CRC
    independent of chunk arrival order (the sender cannot know what the
    receiver's trimmed view will be until reassembly finishes, so hashing
    trimmed text would be order-dependent and useless).

    Assumes :func:`validate_slate` has already run: a duplicate ``field_id``
    would collapse silently into one dict key here, which is precisely why the
    duplicate check lives at the boundary and not in this function.
    """
    out: Dict[int, bytes] = {}
    for raw_id, value in zip(field_ids, field_values):
        out[int(raw_id)] = value.encode('utf-8').ljust(MAX_FIELD_BYTES, b'\x00')
    return out


def present_mask(field_ids) -> int:
    """``CLAP_COMMIT`` byte 1: bit N set means field N is in this transaction."""
    mask = 0
    for raw_id in field_ids:
        mask |= 1 << int(raw_id)
    return mask


def crc16_over_fields(buffers) -> int:
    """CRC-16/CCITT-FALSE over the present fields' 32-byte buffers, ascending id.

    Absent fields contribute nothing at all — not zeros, nothing — which is why
    the same field set produces the same CRC regardless of how many other fields
    the panel is currently displaying.

    An EMPTY set hashes to the init value 0xFFFF, matching the peer's
    ``crc16_over_fields`` with ``present_mask == 0``.
    """
    blob = b''.join(buffers[fid] for fid in sorted(buffers))
    return _p.crc16_ccitt(blob)


def build_field_chunks(field_id, buffer32) -> List[bytes]:
    """Split one 32-byte field buffer into its ``CHUNKS_PER_FIELD`` CAN payloads.

    Byte 0 packs the two nibbles: ``(field_id & 0x0F) | ((seq & 0x0F) << 4)``.
    Bytes 1-7 are 7 bytes of the buffer, NUL-padded — the last chunk covers
    buffer bytes 28..31 plus the three trailing NULs that guarantee the receiver
    can always find the terminator.

    Every returned payload is exactly ``DLC`` bytes.
    """
    if len(buffer32) != MAX_FIELD_BYTES:
        raise SlateValidationError(
            f"field buffer for field {field_id} is {len(buffer32)} bytes, "
            f"expected {MAX_FIELD_BYTES}")
    padded = buffer32.ljust(CHUNKS_PER_FIELD * CHUNK_PAYLOAD, b'\x00')
    chunks: List[bytes] = []
    for seq in range(CHUNKS_PER_FIELD):
        head = (int(field_id) & 0x0F) | ((seq & 0x0F) << 4)
        text = padded[seq * CHUNK_PAYLOAD:(seq + 1) * CHUNK_PAYLOAD]
        chunks.append(bytes([head]) + text)
    return chunks


def build_commit(template_id, mask, txn_id, crc16,
                 force_full_refresh=False) -> bytes:
    """Build the 8-byte ``CLAP_COMMIT`` payload (§8.4).

    Bytes 6-7 are reserved and MUST be zero — written explicitly rather than
    left to a default, so a future allocation of those bytes is a visible change
    here.
    """
    flags = COMMIT_FLAG_FULL_REFRESH if force_full_refresh else 0x00
    return bytes([
        int(template_id) & 0xFF,
        int(mask) & 0xFF,
        flags,
        int(txn_id) & 0xFF,
        int(crc16) & 0xFF,
        (int(crc16) >> 8) & 0xFF,
        0x00,
        0x00,
    ])


def build_transaction(template_id, field_ids, field_values, txn_id,
                      force_full_refresh=False) -> List[Tuple[int, bytes]]:
    """Build a whole slate transaction as ``[(can_id, 8-byte payload), ...]``.

    The list is exactly what ``teensy_link.rpc_args.encode_clap_send`` takes, and
    that encoder preserves order exactly because the bridge drains it FIFO one
    frame per tick. **The commit is last**, which is this module's contract with
    the firmware and the only thing standing between a slate push and an
    ``INCOMPLETE`` rejection.

    Field chunks are emitted in ascending ``field_id`` order (matching the CRC's
    order) purely for reproducibility — the panel accepts chunks in any order.

    Raises:
        SlateValidationError: for any malformed goal — validation runs FIRST,
            before a single frame is built.
    """
    validate_slate(template_id, field_ids, field_values)
    buffers = field_buffers(field_ids, field_values)
    frames: List[Tuple[int, bytes]] = []
    for fid in sorted(buffers):
        for payload in build_field_chunks(fid, buffers[fid]):
            frames.append((clapboard.FIELD_ID, payload))
    crc = crc16_over_fields(buffers)
    frames.append((clapboard.COMMIT_ID,
                   build_commit(template_id, present_mask(field_ids), txn_id,
                                crc, force_full_refresh)))
    return frames


class TxnIdAllocator:
    """Hands out ``CLAP_COMMIT`` transaction ids, rolling 0-255 (§8.4 byte 3).

    Thread-safe: the action's ``execute_callback`` runs in a
    ``ReentrantCallbackGroup``, so two goals can be inside it at once even though
    only one ever holds the single-flight slot.

    **Zero is never allocated.** ``CLAP_ACK`` decodes ``txn_id`` from a wire
    byte, so an all-zero frame — a mis-seated transceiver, a padded relay — reads
    as ``txn_id 0, outcome OK``. If 0 were allocatable, such a frame could
    satisfy a live transaction and report a phantom success on a panel that never
    painted. Skipping it makes that impossible for one wasted value in 256.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._next = 1

    def allocate(self) -> int:
        """Return the next transaction id (1..255, wrapping)."""
        with self._lock:
            value = self._next
            self._next = self._next + 1 if self._next < 255 else 1
            return value
