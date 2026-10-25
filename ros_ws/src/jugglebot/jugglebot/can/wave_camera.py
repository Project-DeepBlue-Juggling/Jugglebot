"""Wave camera adapter CAN protocol: frame codecs, event-time reconstruction, ack rules.

The Wave camera adapter is an ESP32 on the cone-role CAN bus (the can-bridge's
CAN3 peripheral) that drives the Freefly Wave's GPI2 to record and stamps the
first 30 Hz camera tick of each clip against the 0x7DD wall clock. The cone and
the adapter are mutually exclusive on that drop. The host:

* sends WAVE_CMD (0x7E2) through the bridge's WAVE_CMD RPC, streamed at 5 Hz
  while the adapter heartbeat is fresh (``desired`` 0 or 1); the adapter
  auto-stops after 3 s without one;
* decodes WAVE_HEARTBEAT (0x7E3, 10 Hz), WAVE_REC_EVENT (0x7E4, on event) and
  WAVE_INFO (0x7E5, one page every 333 ms), which reach it verbatim as
  CONE_FRAME uplinks.

All frames are DLC 8, little-endian, unused bytes 0. The wire contract lives in
ros_ws/docs/wave_camera_can_adapter.md s4 and is summarised in
``docs/adr/0014-wave-camera-on-can3.md``.

Pure Python: no ROS imports (``teensy_bridge_node`` is the thin wrapper).
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import jugglebot.protocol_config as proto

# CAN arbitration IDs (generated from config/protocol_config.yaml)
CMD_ID = proto.CAN_ID_WAVE_CMD
HEARTBEAT_ID = proto.CAN_ID_WAVE_HEARTBEAT
REC_EVENT_ID = proto.CAN_ID_WAVE_REC_EVENT
INFO_ID = proto.CAN_ID_WAVE_INFO
BLOCK_FIRST = proto.CAN_ID_WAVE_BLOCK_FIRST
BLOCK_LAST = proto.CAN_ID_WAVE_BLOCK_LAST

# Enums (generated)
WaveState = proto.WaveState
WaveEventKind = proto.WaveEventKind
WaveReason = proto.WaveReason

HEARTBEAT_TIMEOUT_MS = proto.WAVE_HEARTBEAT_TIMEOUT_MS
INTENT_RATE_HZ = proto.WAVE_INTENT_RATE_HZ
INTENT_TIMEOUT_MS = proto.WAVE_INTENT_TIMEOUT_MS

FRAME_LEN = 8

# WAVE_CMD byte 2 flags
CMD_FLAG_CLEAR_FAULTS = 0x01

# WAVE_HEARTBEAT byte 1 status bits
STATUS_CAMERA_ON = 0x01
STATUS_UART_SEEN = 0x02
STATUS_TIME_SYNCED = 0x04
STATUS_INTENT_FRESH = 0x08
STATUS_DESIRED = 0x10
STATUS_CONSOLE_OWNS = 0x20
STATUS_GPI2_LOW = 0x40

# WAVE_HEARTBEAT byte 2 latched fault bits
FAULT_GPI2_REFUSED = 0x01
FAULT_TICK_LOST_MID_REC = 0x02
FAULT_CLIP_MISSING = 0x04
FAULT_INTENT_TIMEOUT_AUTOSTOP = 0x08
FAULT_BUS_OFF_SEEN = 0x10
FAULT_SYNC_STEP = 0x20
FAULT_EDGE_RING_OVERFLOW = 0x40
_FAULT_NAMES = ('GPI2_REFUSED', 'TICK_LOST_MID_REC', 'CLIP_MISSING', 'INTENT_TIMEOUT_AUTOSTOP',
                'BUS_OFF_SEEN', 'SYNC_STEP', 'EDGE_RING_OVERFLOW', 'BIT7')

# WAVE_HEARTBEAT units
ELAPSED_UNIT_MS = 100
ELAPSED_SATURATED = 0xFFFE      # 6553.4 s: the adapter stops counting here
SYNC_AGE_UNIT_MS = 100
SYNC_AGE_NEVER = 0xFF           # never synced, or >= 25.5 s
SYNC_AGE_NEVER_MS = 0xFFFFFFFF  # what SYNC_AGE_NEVER maps to on the ROS message

# WAVE_REC_EVENT byte 5 layout
_KIND_MASK = 0x07
_EVT_SYNCED = 0x08
_EVT_TICK_STAMPED = 0x10
_REASON_SHIFT = 5

# 40-bit event time
_WRAP40 = 1 << 40
_MASK40 = _WRAP40 - 1
_HALF_WRAP40 = 1 << 39

# WAVE_INFO pages: page -> (struct format of bytes 1-7, field names)
INFO_PAGE_IDENTITY = 0
INFO_PAGE_TICK = 1
INFO_PAGE_SYNC = 2
INFO_PAGE_LINK = 3
_INFO_LAYOUT: Dict[int, Tuple[str, Tuple[str, ...]]] = {
    INFO_PAGE_IDENTITY: ('<HBI', ('fw_version', 'wire_version', 'uptime_s')),
    INFO_PAGE_TICK: ('<IHB', ('tick_period_ns', 'tick_jitter_us', 'ticks_missed')),
    INFO_PAGE_SYNC: ('<HHBBB', ('offset_spread_us', 'track_err_us', 'accepted_per_s',
                                'rejected_per_s', 'steps')),
    INFO_PAGE_LINK: ('<HHBBB', ('last_tick_to_clip_us', 'can_tx_errors', 'bus_off_events',
                                'intent_rx_per_s', 'reserved')),
}


def _require_dlc8(what: str, data: bytes) -> None:
    if len(data) != FRAME_LEN:
        raise ValueError(f"wave {what}: expected exactly {FRAME_LEN} bytes, got {len(data)}")


def is_wave_id(can_id: int) -> bool:
    """True for any ID in the adapter's allocation, reserved 0x7E6-0x7E7 included."""
    return BLOCK_FIRST <= int(can_id) <= BLOCK_LAST


def _enum_name(enum_cls, value: int) -> str:
    try:
        return enum_cls(value).name
    except ValueError:
        return f"UNKNOWN({value})"


# ── WAVE_CMD (host -> adapter) ───────────────────────────────────────────────

def encode_cmd(desired: int, cmd_seq: int, flags: int = 0, session_id: int = 0) -> bytes:
    """Pack one WAVE_CMD payload (8 bytes, '<BBBBI').

    Byte-identical to ``teensy_link.rpc_args.encode_wave_cmd``: the bridge sends
    the RPC args verbatim as the CAN payload. ``desired`` must be 0 or 1; the
    bridge refuses anything else with ERR_BAD_ARGS, so this raises first.
    """
    if desired not in (0, 1):
        raise ValueError(f"desired must be 0 or 1, got {desired!r}")
    return struct.pack('<BBBBI', int(desired), int(cmd_seq) & 0xFF, int(flags) & 0xFF, 0,
                       int(session_id) & 0xFFFFFFFF)


# ── WAVE_HEARTBEAT (adapter -> host) ─────────────────────────────────────────

@dataclass
class WaveHeartbeat:
    """Decoded WAVE_HEARTBEAT (0x7E3).

    Layout ('<BBBHBBB'): state, status bits, latched faults, elapsed (100 ms
    units, saturating at 0xFFFE), sync_age (100 ms units, 255 = never or
    >= 25.5 s), recordings this session (u8, wraps), cmd_seq_echo.
    """
    state: int = 0
    status: int = 0
    faults: int = 0
    elapsed_100ms: int = 0
    sync_age_100ms: int = SYNC_AGE_NEVER
    recordings: int = 0
    cmd_seq_echo: int = 0

    @classmethod
    def from_can_frame(cls, data: bytes) -> 'WaveHeartbeat':
        """Unpack from an 8-byte CAN frame. Raises ValueError on any other length."""
        _require_dlc8("heartbeat", data)
        state, status, faults, elapsed, sync_age, recs, echo = struct.unpack('<BBBHBBB', data)
        return cls(state=state, status=status, faults=faults, elapsed_100ms=elapsed,
                   sync_age_100ms=sync_age, recordings=recs, cmd_seq_echo=echo)

    def to_can_frame(self) -> bytes:
        """Pack back to 8 bytes (the adapter-side mirror, for fakes and tests)."""
        return struct.pack('<BBBHBBB', self.state, self.status, self.faults,
                           self.elapsed_100ms, self.sync_age_100ms, self.recordings,
                           self.cmd_seq_echo)

    @property
    def camera_on(self) -> bool:
        return bool(self.status & STATUS_CAMERA_ON)

    @property
    def uart_seen(self) -> bool:
        return bool(self.status & STATUS_UART_SEEN)

    @property
    def time_synced(self) -> bool:
        return bool(self.status & STATUS_TIME_SYNCED)

    @property
    def intent_fresh(self) -> bool:
        return bool(self.status & STATUS_INTENT_FRESH)

    @property
    def desired(self) -> bool:
        return bool(self.status & STATUS_DESIRED)

    @property
    def console_owns(self) -> bool:
        return bool(self.status & STATUS_CONSOLE_OWNS)

    @property
    def gpi2_low(self) -> bool:
        return bool(self.status & STATUS_GPI2_LOW)

    @property
    def recording(self) -> bool:
        return self.state == WaveState.RECORDING

    @property
    def sync_age_ms(self) -> int:
        """Sync age in ms, or SYNC_AGE_NEVER_MS for 'never / >= 25.5 s'."""
        if self.sync_age_100ms >= SYNC_AGE_NEVER:
            return SYNC_AGE_NEVER_MS
        return self.sync_age_100ms * SYNC_AGE_UNIT_MS

    @property
    def state_name(self) -> str:
        return _enum_name(WaveState, self.state)


# ── WAVE_REC_EVENT (adapter -> host) ─────────────────────────────────────────

@dataclass
class WaveRecEvent:
    """Decoded WAVE_REC_EVENT (0x7E4).

    Bytes 0-4: t40 — if ``synced``, the low 40 bits of the event's wall time in
    µs; else the event's age at TX in µs. Byte 5: bits0-2 kind, bit3 SYNCED,
    bit4 TICK_STAMPED, bits5-7 reason. Byte 6: cmd_seq_echo. Byte 7:
    recording_seq (u8, wraps).
    """
    t40: int = 0
    kind: int = 0
    synced: bool = False
    tick_stamped: bool = False
    reason: int = 0
    cmd_seq_echo: int = 0
    recording_seq: int = 0

    @classmethod
    def from_can_frame(cls, data: bytes) -> 'WaveRecEvent':
        """Unpack from an 8-byte CAN frame. Raises ValueError on any other length."""
        _require_dlc8("rec event", data)
        t40 = int.from_bytes(bytes(data[0:5]), 'little')
        b5 = data[5]
        return cls(t40=t40, kind=b5 & _KIND_MASK, synced=bool(b5 & _EVT_SYNCED),
                   tick_stamped=bool(b5 & _EVT_TICK_STAMPED), reason=(b5 >> _REASON_SHIFT) & 0x07,
                   cmd_seq_echo=data[6], recording_seq=data[7])

    def to_can_frame(self) -> bytes:
        """Pack back to 8 bytes (the adapter-side mirror, for fakes and tests)."""
        b5 = ((self.kind & _KIND_MASK) | (_EVT_SYNCED if self.synced else 0)
              | (_EVT_TICK_STAMPED if self.tick_stamped else 0)
              | ((self.reason & 0x07) << _REASON_SHIFT))
        return ((int(self.t40) & _MASK40).to_bytes(5, 'little')
                + bytes((b5, self.cmd_seq_echo & 0xFF, self.recording_seq & 0xFF)))

    @property
    def kind_name(self) -> str:
        return _enum_name(WaveEventKind, self.kind)

    @property
    def reason_name(self) -> str:
        return _enum_name(WaveReason, self.reason)


def reconstruct_event_time_us(t40: int, host_now_us: int) -> int:
    """Rebuild a full Unix-epoch µs timestamp from its low 40 bits.

    2^40 µs is 12.7 days, so the high bits come from the host wall clock under
    the assumption that the true event lies within ±2^39 µs (±6.4 days) of
    ``host_now_us``. The 40-bit twin of the clapboard's
    ``reconstruct_fire_time_us``: pick the candidate in the host's own 2^40
    window, then step one wrap either way if that lands more than half a wrap
    away.
    """
    candidate = (int(host_now_us) & ~_MASK40) | (int(t40) & _MASK40)
    if candidate > host_now_us + _HALF_WRAP40:
        candidate -= _WRAP40
    elif candidate + _HALF_WRAP40 < host_now_us:
        candidate += _WRAP40
    return candidate


def event_wall_us(evt: WaveRecEvent, t_bridge_us: int, host_now_us: int) -> int:
    """The event's wall-clock instant in µs, by the path its SYNCED bit selects.

    Synced: ``t40`` is the low 40 bits of the wall time -> 40-bit reconstruction.
    Unsynced: ``t40`` is the event's age at TX, and the bridge stamped the frame
    with its own wall clock on RX (``ConeFrame.t_bridge_us``), so the event is
    ``t_bridge_us - age``. That is still bridge wall time (good to well under a
    millisecond while the bridge is synced); the event is flagged unsynced.
    """
    if evt.synced:
        return reconstruct_event_time_us(evt.t40, host_now_us)
    return int(t_bridge_us) - int(evt.t40)


# ── WAVE_INFO (adapter -> host) ──────────────────────────────────────────────

@dataclass
class WaveInfoPage:
    """One decoded WAVE_INFO (0x7E5) page: byte 0 is the page number.

    Known pages decode into ``fields`` by name (s4.4); an unknown page keeps its
    raw bytes 1-7 in ``raw`` and an empty ``fields``.
    """
    page: int = 0
    fields: Dict[str, int] = field(default_factory=dict)
    raw: bytes = b''

    @classmethod
    def from_can_frame(cls, data: bytes) -> 'WaveInfoPage':
        """Unpack from an 8-byte CAN frame. Raises ValueError on any other length."""
        _require_dlc8("info", data)
        page = data[0]
        raw = bytes(data[1:8])
        layout = _INFO_LAYOUT.get(page)
        if layout is None:
            return cls(page=page, fields={}, raw=raw)
        fmt, names = layout
        vals = struct.unpack(fmt, raw)
        return cls(page=page, fields=dict(zip(names, vals)), raw=raw)

    @staticmethod
    def encode(page: int, **values: int) -> bytes:
        """Pack a known page (the adapter-side mirror, for fakes and tests)."""
        fmt, names = _INFO_LAYOUT[page]
        return bytes((page,)) + struct.pack(fmt, *(int(values.get(n, 0)) for n in names))


# ── Sequence / elapsed unwrapping ────────────────────────────────────────────

def fault_names(bits: int) -> str:
    """Latched fault byte as comma-separated names ('none' for 0), the same
    vocabulary as the GUI's tooltip (camera-panel.js FAULT_NAMES)."""
    names = [n for i, n in enumerate(_FAULT_NAMES) if int(bits) & (1 << i)]
    return ', '.join(names) if names else 'none'


class SeqUnwrapper:
    """Unwrap a u8 counter into a monotonic u32.

    Forward steps (mod 256) of up to half the range count as progress across a
    wrap. A larger "forward" step is a backward step on the wire, i.e. the
    adapter reset its counter (reboot, or a new session_id): the total keeps
    counting from where it was rather than jumping by ~250.

    ``seed_from_zero`` starts the total at 0 instead of at the first raw value,
    so only forward steps seen by this unwrapper count. ``recordings`` needs it:
    the node's first heartbeat always predates its own session_id at the
    adapter (intents stream only once a heartbeat is fresh), so that first
    value is the previous session's count, and the adapter's reset to 0 one
    intent later must not leave it behind as an offset.
    """

    def __init__(self, seed_from_zero: bool = False) -> None:
        self._last: Optional[int] = None
        self._total = 0
        self._seed_from_zero = seed_from_zero

    @property
    def value(self) -> int:
        return self._total

    def update(self, raw: int) -> int:
        raw = int(raw) & 0xFF
        if self._last is None:
            self._total = 0 if self._seed_from_zero else raw
        else:
            delta = (raw - self._last) & 0xFF
            if delta <= 0x80:
                self._total += delta
            else:
                self._total += raw          # counter restarted from 0: count its new starts
        self._last = raw
        return self._total & 0xFFFFFFFF


class WaveRecordingTracker:
    """Host-side bookkeeping for the heartbeat and event counters.

    * ``recording_seq`` from WAVE_REC_EVENT is unwrapped u8 -> u32;
    * ``recordings`` from WAVE_HEARTBEAT is unwrapped likewise, counted from 0
      (the first heartbeat carries the previous session's count);
    * ``elapsed`` (100 ms units) saturates at 0xFFFE on the wire, so past that
      the tracker extrapolates from the last unsaturated value on the host
      monotonic clock;
    * the current recording's start wall time is latched from START events and
      cleared on STOP / AUTO_STOP.
    """

    def __init__(self) -> None:
        self._rec_seq = SeqUnwrapper()
        self._recordings = SeqUnwrapper(seed_from_zero=True)
        self._elapsed_anchor: Optional[Tuple[int, float]] = None   # (ms, host mono s)
        self.start_wall_us: Optional[int] = None

    def on_event(self, evt: WaveRecEvent, event_wall_us_: int) -> int:
        """Fold one event in; returns the unwrapped recording_seq."""
        seq = self._rec_seq.update(evt.recording_seq)
        if evt.kind == WaveEventKind.START:
            self.start_wall_us = int(event_wall_us_)
            self._elapsed_anchor = None
        elif evt.kind in (WaveEventKind.STOP, WaveEventKind.AUTO_STOP):
            self.start_wall_us = None
            self._elapsed_anchor = None
        return seq

    def recordings(self, hb: WaveHeartbeat) -> int:
        return self._recordings.update(hb.recordings)

    def elapsed_ms(self, hb: WaveHeartbeat, host_mono_s: float) -> int:
        """Elapsed recording time in ms, extrapolated past the wire saturation."""
        if not hb.recording:
            self._elapsed_anchor = None
            return 0
        if hb.elapsed_100ms < ELAPSED_SATURATED:
            ms = hb.elapsed_100ms * ELAPSED_UNIT_MS
            self._elapsed_anchor = (ms, float(host_mono_s))
            return ms
        if self._elapsed_anchor is None:
            # Saturated before the first sample this session saw: anchor here.
            self._elapsed_anchor = (ELAPSED_SATURATED * ELAPSED_UNIT_MS, float(host_mono_s))
        ms0, t0 = self._elapsed_anchor
        return int(ms0 + max(0.0, float(host_mono_s) - t0) * 1000.0)


# ── Service ack rules (wave_camera_can_adapter.md s4.5, owner decisions 2026-10-19) ──────────

def evaluate_ack(want_record: bool, evt: WaveRecEvent) -> Tuple[bool, str]:
    """(success, message) for the record_start / record_stop service reply.

    record_start succeeds on START (reason NONE; CLIP_MISSING also succeeds but
    the message carries a warning) or on REFUSED/NO_CHANGE (already recording).
    record_stop succeeds on STOP, or on REFUSED/NO_CHANGE (already idle).
    Anything else for the matching cmd_seq is a failure naming kind and reason.
    """
    kind, reason = evt.kind, evt.reason
    if kind == WaveEventKind.REFUSED and reason == WaveReason.NO_CHANGE:
        return True, ("already recording" if want_record else "already stopped")
    if want_record and kind == WaveEventKind.START:
        if reason == WaveReason.NONE:
            return True, "recording started"
        if reason == WaveReason.CLIP_MISSING:
            return True, ("recording started, WARNING: CLIP_MISSING (no 'Created new clip.' "
                          "line within the window; GPI2 held low, check the camera)")
        return True, f"recording started, WARNING: reason {evt.reason_name}"
    if not want_record and kind == WaveEventKind.STOP:
        return True, "recording stopped"
    return False, f"{evt.kind_name} reason {evt.reason_name}"
