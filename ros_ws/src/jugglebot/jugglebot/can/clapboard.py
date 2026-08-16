"""Electronic Clapboard CAN protocol: heartbeat, fire-event and ack decoding.

The electronic clapboard (a separate project, ``~/Desktop/Github/Electronic-
Clapboard``) is an ESP32-S3 film slate with an e-paper panel and a sync flash.
It attaches to the can-bridge's **cone bus** (the physical CAN3 peripheral,
which has hosted the cone role since the 2026-07-31 bus-role swap), physically
replacing the catching cone — the two are mutually exclusive by connection, so
this module and :mod:`jugglebot.can.catching_cone` coexist and are told apart by
arbitration id alone.

The host only ever *decodes* the three uplink frames here. ``CLAP_FIELD`` /
``CLAP_COMMIT`` / ``CLAP_LINK`` are downlink-only and are not encoded here —
placing an arbitrary frame on the cone bus needs firmware that does not exist
yet (there is exactly one caller of ``can_cone_send()`` in the bridge today).

**Wire contract:** ``Electronic-Clapboard docs/protocol.md`` §8.6-8.8 is
NORMATIVE and cross-repository; neither side may change it unilaterally. Ids and
enum values come from ``config/protocol_config.yaml`` (which copies that section)
via the generated constants, never from literals here.

Two rules that section states nowhere but the peer's code enforces, both
reproduced below:

* **Every frame in both directions is DLC 8.** ``can_frames.h`` rejects any
  other length outright ("accepting short frames would mean reading
  uninitialised buffer bytes"). ``protocol.md`` never says so.
* **``CLAP_FIRE_EVENT`` carries only the LOW 48 BITS of the wall clock.** See
  :func:`reconstruct_fire_time_us`.

See ``plans/active/clapboard-can3-integration.md`` for the integration arc.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

import jugglebot.protocol_config as proto

# ── CAN arbitration ids (protocol.md §8.2, via protocol_config.yaml) ─────────
# Uplink (clapboard → Jetson) — the three this module decodes.
ACK_ID = proto.CAN_ID_CLAP_ACK
HEARTBEAT_ID = proto.CAN_ID_CLAP_HEARTBEAT
FIRE_EVENT_ID = proto.CAN_ID_CLAP_FIRE_EVENT

# Downlink (Jetson/bridge → clapboard). Exported so the uplink dispatcher can
# recognise — and deliberately account for — a frame that should never have come
# back the other way.
FIELD_ID = proto.CAN_ID_CLAP_FIELD
COMMIT_ID = proto.CAN_ID_CLAP_COMMIT
LINK_ID = proto.CAN_ID_CLAP_LINK

#: Inclusive bounds of the whole 0x7E8-0x7EF allocation, reserved pair included.
#: Mirrors the firmware's ``is_clapboard_id()`` and the peer's own
#: ``ID_CLAP_BLOCK_FIRST/LAST``.
BLOCK_FIRST_ID = proto.CAN_ID_CLAP_BLOCK_FIRST
BLOCK_LAST_ID = proto.CAN_ID_CLAP_BLOCK_LAST

#: Ids that only ever travel Jetson/bridge → clapboard. One arriving on the
#: uplink means something is wrong (a second transmitter on the segment, or a
#: bridge echoing its own TX), and the handoff doc says to notice it once and
#: drop it rather than decode it as anything.
DOWNLINK_ONLY_IDS = frozenset((FIELD_ID, COMMIT_ID, LINK_ID))

# Re-export the generated enums, mirroring catching_cone's re-export of
# CatchingConeStates.
ClapboardStates = proto.ClapboardStates
ClapboardAckOutcome = proto.ClapboardAckOutcome

# CLAP_HEARTBEAT byte 1 flag bits (§8.7). Module-private, as in catching_cone —
# they are frame-layout detail, not a shared protocol vocabulary.
_HB_FLAG_FIRE_READY = 0x01
_HB_FLAG_TEMPLATE_LOADED = 0x02
_HB_FLAG_WIFI_UP = 0x04
_HB_FLAG_RAIL_OK = 0x08
_HB_FLAG_TIME_SYNCED = 0x10

# 48-bit wrap span for fire-timestamp reconstruction (see the function below).
_WRAP48 = 1 << 48
_MASK48 = _WRAP48 - 1
_HALF_WRAP48 = 1 << 47

#: Every clapboard frame is exactly 8 bytes, both directions.
_DLC = 8


def _require_dlc8(kind: str, data: bytes) -> None:
    """Reject anything that is not exactly 8 bytes.

    Stricter than ``catching_cone``'s ``len(data) < 8``, and deliberately so:
    the peer's decoders require ``len == 8`` exactly and drop everything else
    silently, so a decoder here that accepted a 7- or 9-byte frame would be
    accepting bytes the other end would never have sent — i.e. corruption. The
    two paths are equivalent for frames arriving via ``ConeFrame`` (whose
    payload is a fixed 8-byte array sliced to ``dlc``), so the strictness costs
    nothing and closes the gap for any other caller.
    """
    if len(data) != _DLC:
        raise ValueError(f"{kind}: expected {_DLC} bytes, got {len(data)}")


@dataclass
class ClapboardHeartbeat:
    """Decoded 8-byte CLAP_HEARTBEAT (0x7EC) frame, emitted at 10 Hz.

    Frame layout (little-endian, protocol.md §8.7)::

        Byte 0    state — 0=BOOT 1=IDLE 2=RENDERING 3=SCREENSAVER 4=ERROR
        Byte 1    flags — bit0 fire_ready, bit1 template_loaded,
                          bit2 wifi_up, bit3 rail_ok, bit4 time_synced
        Bytes 2-3 fires_since_boot (u16 LE)
        Bytes 4-5 rail_mv (u16 LE) — 0 when no divider is fitted
        Byte 6    active_template_id
        Byte 7    last_error — last non-OK CLAP_ACK outcome, 0 if none

    ``time_synced`` is the load-bearing flag: while it is False the clapboard
    emits **no** fire events at all (the flash still happens; only the record is
    suppressed, because ``protocol.md`` §8.8 judges a missing sync record
    recoverable in post and a wrong one silently corrupting). A gap between
    ``fires_since_boot`` and the count of published fire events is therefore
    EXPECTED and diagnostic, not a defect.

    ``state`` is kept as a plain ``int``, NOT coerced to
    :class:`ClapboardStates` — a deliberate deviation from
    ``catching_cone.ConeHeartbeat``. Coercing raises ``ValueError`` on an
    unrecognised value, which (given every RX callback drops on exception) would
    turn "the clapboard reported a state this host has not heard of" into "the
    clapboard is disconnected" 500 ms later. That is a confidently wrong
    operator-facing verdict, which is exactly the failure the whole id-
    discrimination design exists to avoid; passing the raw byte through is
    merely uninformative. Use :meth:`state_name` for display.
    """
    state: int = int(ClapboardStates.BOOT)
    fire_ready: bool = False
    template_loaded: bool = False
    wifi_up: bool = False
    rail_ok: bool = False
    time_synced: bool = False
    fires_since_boot: int = 0
    rail_mv: int = 0
    active_template_id: int = 0
    last_error: int = 0

    @classmethod
    def from_can_frame(cls, data: bytes) -> 'ClapboardHeartbeat':
        """Unpack from an 8-byte CAN frame. Raises ValueError on any other length."""
        _require_dlc8("clapboard heartbeat", data)
        state, flags, fires, rail_mv, template_id, last_error = struct.unpack(
            '<BBHHBB', data)
        return cls(
            state=state,
            fire_ready=bool(flags & _HB_FLAG_FIRE_READY),
            template_loaded=bool(flags & _HB_FLAG_TEMPLATE_LOADED),
            wifi_up=bool(flags & _HB_FLAG_WIFI_UP),
            rail_ok=bool(flags & _HB_FLAG_RAIL_OK),
            time_synced=bool(flags & _HB_FLAG_TIME_SYNCED),
            fires_since_boot=fires,
            rail_mv=rail_mv,
            active_template_id=template_id,
            last_error=last_error,
        )

    @property
    def state_name(self) -> str:
        """Human-readable state, or ``UNKNOWN(n)`` for a value this host predates."""
        try:
            return ClapboardStates(self.state).name
        except ValueError:
            return f"UNKNOWN({self.state})"


@dataclass
class ClapboardFireEvent:
    """Decoded 8-byte CLAP_FIRE_EVENT (0x7ED) frame — one accepted sync flash.

    Frame layout (little-endian, protocol.md §8.8)::

        Bytes 0-5  wall-clock microseconds of the flash (48-bit LE)
        Bytes 6-7  fires_since_boot (u16 LE)

    This is the device's whole reason for existing: the instant the LED fired,
    in the wall-clock frame the bridge distributes on 0x7DD, so footage from
    several cameras can be aligned against robot telemetry in post.

    ``wall_us_low48`` is the RAW wire field — the low 48 bits only. Call
    :func:`reconstruct_fire_time_us` to get a usable absolute timestamp; the
    masking is what keeps a wrapped high bit out of ``fires_since_boot``.
    """
    wall_us_low48: int = 0
    fires_since_boot: int = 0

    @classmethod
    def from_can_frame(cls, data: bytes) -> 'ClapboardFireEvent':
        """Unpack from an 8-byte CAN frame. Raises ValueError on any other length."""
        _require_dlc8("clapboard fire event", data)
        # The 48-bit field is not a struct primitive; take the six bytes
        # directly rather than unpacking a u64 over a 6-byte field and letting
        # fires_since_boot bleed into the timestamp.
        return cls(
            wall_us_low48=int.from_bytes(data[0:6], 'little'),
            fires_since_boot=int.from_bytes(data[6:8], 'little'),
        )


@dataclass
class ClapboardAck:
    """Decoded 8-byte CLAP_ACK (0x7EB) frame — one transaction outcome.

    Frame layout (little-endian, protocol.md §8.6)::

        Byte 0    txn_id echoed from the CLAP_COMMIT that opened the transaction
        Byte 1    outcome (ClapboardAckOutcome)
        Byte 2    clapboard state at ack time (§8.7)
        Bytes 3-4 render_ms (u16 LE) — MEASURED panel time, 0 if not rendered
        Bytes 5-7 reserved

    ``render_ms`` is why this is an ack and not a fire-and-forget: an e-paper
    full refresh is 1.5-3.5 s, so a slate push's duration is dominated by the
    panel rather than the wire, and success must not be reported until this
    frame arrives.

    ``outcome`` and ``state`` are plain ``int``s for the same forward-
    compatibility reason spelled out on :class:`ClapboardHeartbeat`.
    """
    txn_id: int = 0
    outcome: int = int(ClapboardAckOutcome.OK)
    state: int = int(ClapboardStates.BOOT)
    render_ms: int = 0

    @classmethod
    def from_can_frame(cls, data: bytes) -> 'ClapboardAck':
        """Unpack from an 8-byte CAN frame. Raises ValueError on any other length."""
        _require_dlc8("clapboard ack", data)
        txn_id, outcome, state, render_ms = struct.unpack('<BBBH', data[:5])
        return cls(txn_id=txn_id, outcome=outcome, state=state,
                   render_ms=render_ms)

    @property
    def outcome_name(self) -> str:
        """Human-readable outcome, or ``UNKNOWN(n)`` for a value this host predates."""
        try:
            return ClapboardAckOutcome(self.outcome).name
        except ValueError:
            return f"UNKNOWN({self.outcome})"


def reconstruct_fire_time_us(low48: int, host_now_us: int) -> int:
    """Reconstruct a full wall-clock µs timestamp from the frame's low 48 bits.

    **Why this is needed at all**, since ``protocol.md`` §8.8 reads as though the
    whole clock fits: the clapboard's ``wall_us()`` is Unix-epoch microseconds
    (its 0x7DD anchor is ``unix_s * 1e6 + usec``), and a 2026 Unix µs value needs
    **51 bits** — 1.79e15 against a 48-bit ceiling of 2.81e14. The peer's
    ``encode_fire_event`` therefore masks with ``0x0000FFFFFFFFFFFF`` and puts
    the low 48 bits on the wire. Taking the field at face value would stamp every
    flash somewhere in 1978 — a confidently wrong timestamp in the one record the
    device exists to produce. §8.8's "~8.9 years" is the WRAP PERIOD, not the
    representable range, and the reconstruction rule is stated nowhere; it is
    inferred from the peer's code, which is the authority (same class as the
    unstated strict-DLC-8 rule).

    The host rebuilds the high bits from its own clock, assuming the true fire
    instant is within ±2^47 µs (~4.46 years) of ``host_now_us``. That is the
    µs-domain, 48-bit sibling of :func:`catching_cone.reconstruct_catch_time_us`,
    and its window is so wide that only a grossly wrong host clock can defeat it.

    Args:
        low48: ``wall_us_low48`` from the frame (0 .. 2^48-1).
        host_now_us: host wall clock in µs at decode time, e.g.
                     ``int(time.time() * 1e6)``.

    Returns:
        Full wall-clock microsecond timestamp (Unix epoch).
    """
    candidate = (host_now_us & ~_MASK48) | (low48 & _MASK48)
    if candidate > host_now_us + _HALF_WRAP48:
        candidate -= _WRAP48
    elif candidate + _HALF_WRAP48 < host_now_us:
        candidate += _WRAP48
    return candidate
