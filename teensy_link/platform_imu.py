"""Platform IMU capture upload: frame codecs, the upload state machine and its worker service.

Pure Python (no ROS). Contract: ``ros_ws/docs/platform_imu_protocol.md`` (normative; section numbers
below are its). The Jetson side of the capture path, in three layers:

* codecs for IMU_REPLY / IMU_STATUS / the header / anchors / the PLATFORM_IMU_BATCH datagram;
* :class:`UploadMachine`, a single-owner state machine driven by injected time and injected send
  functions (no threads, no sockets, so a fake Platform can drive it in a test);
* :class:`PlatformImuService`, the one worker thread that owns a machine, fed by two bounded queues
  that the UDP receive callback fills with O(1) work.

Why here and not in ``jugglebot/``: the importers are the node, the tests and offline tools, and the
repo rule keeps wire/transport logic in the ROS-free top-level ``teensy_link`` package (the node
imports it from the live tree through PYTHONPATH, so an edit is live at the next relaunch).
"""

from __future__ import annotations

import logging
import math
import queue
import struct
import threading
import time
import zlib
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple

import numpy as np

from . import protocol as _protocol  # noqa: F401  (puts config/generated on sys.path)
import protocol_config as pc  # noqa: E402

from .platform_imu_capture import TimeModel, build_capture, capture_stem

logger = logging.getLogger(__name__)

OP = pc.PlatformImuOp
ST = pc.PlatformImuStatus
STATE = pc.PlatformImuState

WORD = pc.PLATFORM_IMU_WORD_BYTES                  # 6
READ_MAX = pc.PLATFORM_IMU_READ_MAX_WORDS          # 256
ANCHOR_BASE = pc.PLATFORM_IMU_ANCHOR_SEQ_BASE      # 0xFA80
HEADER_BASE = pc.PLATFORM_IMU_HEADER_SEQ_BASE      # 0xFFF0
HEADER_WORDS = pc.PLATFORM_IMU_HEADER_BYTES // WORD   # 11
ANCHOR_WORDS = pc.PLATFORM_IMU_ANCHOR_BYTES // WORD   # 3
PLATFORM_MIN_GAP_10US = pc.PLATFORM_IMU_READ_MIN_GAP_10US          # 40 (the board's own 400 us floor)
BRIDGE_ARMED_MIN_GAP_10US = pc.PLATFORM_IMU_BRIDGE_ARMED_MIN_GAP_10US  # 63 (the bridge's Armed clamp)
CAN_ID_REPLY = pc.CAN_ID_PLATFORM_IMU_REPLY
CAN_ID_STATUS = pc.CAN_ID_PLATFORM_IMU_STATUS
BATCH_MAX_FRAMES = 24
NOMINAL_ODR_HZ = pc.PLATFORM_IMU_NOMINAL_ODR_HZ        # 1000, the boot rate
FAST_ODR_HZ = pc.PLATFORM_IMU_FAST_ODR_HZ              # 2000 (FW 13, SET_ODR or the 2 kHz boot image)
RING_SAMPLES = pc.PLATFORM_IMU_RING_SAMPLES            # 32000: 32 s at 1 kHz, 16 s at 2 kHz
#: The first Platform FW_VERSION that appends a contiguous READ to the active block (READ_EXTENDED) instead
#: of replacing it. Read from the frozen capture's own header (byte 52), not from the node's 0x6E0 identity
#: poll: the header is written by the board that froze THIS capture in THIS boot (the boot byte is checked),
#: so it cannot be stale after a reflash. An older board would answer a chained READ by REPLACING the block,
#: dropping the rest of the one in flight (holes, healed by the refill, but every chain would cost them).
CHAIN_MIN_FW_VERSION = 13
#: The first Platform FW_VERSION with the chunk-CRC table (0xFF40) and the TX diag region (0xFA40), spec
#: sections 3 and 6. Read from the frozen capture's header for the reason above; an older board is never sent
#: a READ of either region (it would answer ERR_OUT_OF_RANGE), so it is read exactly as before.
VERIFY_MIN_FW_VERSION = 14
CHUNK_BASE = pc.PLATFORM_IMU_CHUNK_CRC_SEQ_BASE    # 0xFF40
CHUNK_WORDS = pc.PLATFORM_IMU_CHUNK_CRC_WORDS      # 512
CHUNK_READY = pc.PLATFORM_IMU_CHUNK_CRC_READY      # 0x8000
SAMPLE_CHUNKS = -(-2 * RING_SAMPLES // CHUNK_WORDS)                              # 125
ANCHOR_CHUNKS = -(-ANCHOR_WORDS * pc.PLATFORM_IMU_ANCHOR_RING // CHUNK_WORDS)    # 3
CHUNK_TABLE_WORDS = SAMPLE_CHUNKS + ANCHOR_CHUNKS                                # 128
DIAG_BASE = pc.PLATFORM_IMU_DIAG_SEQ_BASE          # 0xFA40
DIAG_WORDS = pc.PLATFORM_IMU_DIAG_WORDS            # 5


# ── codecs ───────────────────────────────────────────────────────────────────────────────────────

class HeaderError(ValueError):
    """The 66-byte header failed its self-check (length, CRC or format version)."""


_REPLY = struct.Struct("<BBHI")
_STATUS = struct.Struct("<BBbBBBBB")
_HEADER = struct.Struct("<BBHIIIHHQIHHHBBBBhhHHHHIII")
_ANCHOR = struct.Struct("<IIQBx")
_BATCH_HEAD = struct.Struct("<QBBHIIII")           # the 28 bytes before PlatformImuBatch.frames
_DIAG = struct.Struct("<BBIIHHHHHBBHHI")           # FW 14 diag region, 30 bytes (spec section 3, "Diag")
assert _DIAG.size == DIAG_WORDS * WORD
DIAG_FIELDS = ("diag_version", "flags", "data_placed", "data_tx_done", "data_tx_lost_mb", "data_pend_over_2ms",
               "data_pend_max_us", "tec_inc_sum", "rec_inc_sum", "tec_live", "rec_live", "tx_deferred",
               "tx_queued_anyway", "reserved")
#: Cumulative counters and their wrap: the host stores deltas (mod 2**bits) between the two reads.
_DIAG_WRAP = {"data_placed": 32, "data_tx_done": 32, "data_tx_lost_mb": 16, "data_pend_over_2ms": 16,
              "tec_inc_sum": 16, "rec_inc_sum": 16, "tx_deferred": 16, "tx_queued_anyway": 16}


def decode_diag(blob: bytes) -> Dict[str, int]:
    """The 30-byte FW 14 diag image (little-endian)."""
    return dict(zip(DIAG_FIELDS, _DIAG.unpack(bytes(blob[:_DIAG.size]))))


def diag_deltas(before: Dict[str, int], after: Dict[str, int]) -> Dict[str, int]:
    """Counter deltas over an upload (modulo each counter's width); the live and peak fields as read last."""
    out = {k: (after[k] - before[k]) % (1 << bits) for k, bits in _DIAG_WRAP.items()}
    for k in ("data_pend_max_us", "tec_live", "rec_live"):
        out[k] = after[k]
    out["tracking_valid"] = bool(after["flags"] & 0x01)
    return out
assert _HEADER.size == pc.PLATFORM_IMU_HEADER_BYTES and _ANCHOR.size == pc.PLATFORM_IMU_ANCHOR_BYTES

HEADER_FIELDS = (
    "version", "reason", "capture_id", "boot_id", "n_samples", "first_index", "n_anchors", "flags",
    "freeze_wall_us", "freeze_local_us", "odr_hz", "accel_lsb_per_g", "gyro_lsb_per_dps_x10",
    "gyro_accel_config0", "accel_config0", "gyro_config0", "i2c_clock_100k", "temp_raw_start",
    "temp_raw_freeze", "fifo_overflows", "i2c_errors", "sync_innovation_us", "fw_version",
    "payload_crc32", "reserved", "header_crc32")


@dataclass(frozen=True)
class ImuReply:
    op: int
    status: int
    capture_id: int
    detail: int

    @property
    def read_first_seq(self) -> int:       # READ OK: detail = first_seq | count << 16
        return self.detail & 0xFFFF

    @property
    def read_count(self) -> int:
        return (self.detail >> 16) & 0xFFFF


@dataclass(frozen=True)
class ImuStatus:
    state: int
    flags: int
    temp_c: int
    fifo_overflows: int
    i2c_errors: int
    sync_peak: int
    fill_s: int
    boot: int

    @property
    def sensor_ok(self) -> bool:
        return bool(self.flags & pc.PlatformImuStatusFlag.SENSOR_OK)

    @property
    def odr_hz(self) -> int:
        """The selected capture rate: ODR_2K (FW 13) set = 2 kHz, else 1 kHz (FW 12 and older never set it)."""
        return FAST_ODR_HZ if self.flags & pc.PlatformImuStatusFlag.ODR_2K else NOMINAL_ODR_HZ

    @property
    def ring_s(self) -> float:
        """Seconds of samples the ring holds at the selected rate (32 s at 1 kHz, 16 s at 2 kHz)."""
        return RING_SAMPLES / float(self.odr_hz)


@dataclass(frozen=True)
class Header:
    version: int
    reason: int
    capture_id: int
    boot_id: int
    n_samples: int
    first_index: int
    n_anchors: int
    flags: int
    freeze_wall_us: int
    freeze_local_us: int
    odr_hz: int
    accel_lsb_per_g: int
    gyro_lsb_per_dps_x10: int
    gyro_accel_config0: int
    accel_config0: int
    gyro_config0: int
    i2c_clock_100k: int
    temp_raw_start: int
    temp_raw_freeze: int
    fifo_overflows: int
    i2c_errors: int
    sync_innovation_us: int
    fw_version: int
    payload_crc32: int
    reserved: int
    header_crc32: int


@dataclass(frozen=True)
class Anchor:
    index: int
    local_us: int
    wall_us: int
    flags: int


def decode_imu_reply(data: bytes) -> ImuReply:
    if len(data) < 8:
        raise ValueError("IMU_REPLY needs 8 bytes, got %d" % len(data))
    return ImuReply(*_REPLY.unpack_from(data))


def decode_imu_status(data: bytes) -> ImuStatus:
    if len(data) < 8:
        raise ValueError("IMU_STATUS needs 8 bytes, got %d" % len(data))
    return ImuStatus(*_STATUS.unpack_from(data))


def decode_header(blob: bytes) -> Header:
    """Parse and self-check the 66-byte header (``header_crc32`` over bytes 0-61, version 1)."""
    if len(blob) != pc.PLATFORM_IMU_HEADER_BYTES:
        raise HeaderError("header is %d bytes, expected %d" % (len(blob), pc.PLATFORM_IMU_HEADER_BYTES))
    h = Header(*_HEADER.unpack(blob))
    if zlib.crc32(blob[:62]) & 0xFFFFFFFF != h.header_crc32:
        raise HeaderError("header CRC mismatch")
    if h.version != pc.PLATFORM_IMU_HEADER_FORMAT_VERSION:
        raise HeaderError("unsupported header format version %d" % h.version)
    return h


def decode_anchors(blob: bytes) -> List[Anchor]:
    n = len(blob) // pc.PLATFORM_IMU_ANCHOR_BYTES
    return [Anchor(*_ANCHOR.unpack_from(blob, i * pc.PLATFORM_IMU_ANCHOR_BYTES)) for i in range(n)]


@dataclass(frozen=True)
class BatchInfo:
    t_bridge_us: int
    count: int
    flags: int
    ring_hwm: int
    rx_frames: int
    ring_drops: int
    batches_sent: int
    send_fails: int


def parse_batch(payload: bytes) -> Tuple[BatchInfo, bytes]:
    """Split a PLATFORM_IMU_BATCH payload into its counters and ``count`` 8-byte records
    (``[seq u16 LE][6 B word]``). O(1): one fixed unpack and one slice, which is all the UDP receive
    callback does per datagram. Raises ``ValueError`` on a short payload."""
    if len(payload) < _BATCH_HEAD.size:
        raise ValueError("short PLATFORM_IMU_BATCH (%d bytes)" % len(payload))
    info = BatchInfo(*_BATCH_HEAD.unpack_from(payload))
    count = min(info.count, BATCH_MAX_FRAMES, (len(payload) - _BATCH_HEAD.size) // 8)
    return info, bytes(payload[_BATCH_HEAD.size:_BATCH_HEAD.size + 8 * count])


def read_gap_check(value: Any) -> Tuple[bool, str]:
    """Validate an operator's frame-gap request (the ``imu_read_gap_10us`` node parameter): an integer number
    of 10 us from the board's floor (40, 400 us) to the u8 wire field's top (255). ``(ok, why-not)``."""
    if isinstance(value, bool) or not isinstance(value, int):
        return False, "must be an integer (x 10 us), got %r" % (value,)
    if not PLATFORM_MIN_GAP_10US <= value <= 255:
        return False, "must be %d..255 (x 10 us; the Platform clamps below %d), got %d" % (
            PLATFORM_MIN_GAP_10US, PLATFORM_MIN_GAP_10US, value)
    return True, ""


# ── configuration, job, result ───────────────────────────────────────────────────────────────────

@dataclass
class UploadConfig:
    gap_10us: int = 80                 # requested pace: 800 us, about 1250 frames/s (spec section 6, item 3)
    cmd_timeout_s: float = 0.6         # one CTRL/READ verdict (IMU_REPLY) before a resend
    freeze_timeout_s: float = 1.5      # FREEZE also computes the payload CRC on the board
    cmd_retries: int = 3
    block_slack_s: float = 0.15        # spec says 50 ms; +100 ms for the bridge's 10 ms flush and UDP
    # After a block's LAST seq arrives with a hole left in the block, wait this long (no frame at all) for
    # stragglers before the next READ. Was 30 ms; 5 ms since IDEA-25 (2026-11-21). The 30 ms bought nothing:
    # a straggler is a frame already committed to a Platform TX mailbox, which a replacing READ cannot cancel,
    # and when it arrives it fills its word at region level whichever block is current (``_Region.put``). Armed
    # (about 0.4 % of frames omitted, 64 % of 256-word blocks hold a hole) the 30 ms (35-40 ms effective with
    # the 20 ms worker poll) cost about 5 s of a 79 s whole-ring capture (debrief 2026-11-20, capture AEDE).
    settle_s: float = 0.005
    # A hole-fill READ ends at a run of MORE than this many already-present words; a shorter present run between
    # two holes is re-read (duplicates) to save a READ turnaround (about 16 ms, about 16-20 frames at the
    # 0.8 ms pace, so 16 is break-even). Before IDEA-25 a refill READ ran from the first hole to the last hole
    # within READ_MAX words, re-sending every present word between: Armed, about 55 duplicates per hole healed,
    # 14,651 duplicates of 82,580 frames and about 13 s of a 79 s capture (AEDE); modelled with this rule:
    # about 200 duplicates and about 1.3x the READs. READ_MAX (256) restores the old behaviour. 12 since the
    # pipelined refill (below) made a READ cost about 15 ms of spacing instead of a 60 ms turnaround: break-even
    # moves down to 15-19 words at 0.8 ms, and 12 stops bridging the 4-word runs between losses 5 apart (the
    # Armed gap-80 pattern) without adding READs (simulation on the real hole sets, logbook 2026-11-23).
    refill_merge_words: int = 12
    # PIPELINED REFILL (IDEA-25; spec section 6). In a hole-refill pass (pass 2 on, anchors and samples only) the
    # next refill READ goes out ``max(refill_spacing_s, count x gap + refill_margin_s)`` after the previous one was
    # acknowledged by the can-bridge (queued on its CAN TX, in order), without waiting for that block's frames.
    # The count x gap term is the whole block on the board's fixed pace schedule, so the board has finished it
    # before the next READ arrives and never replaces it (Platform imu_proto.h READ: any non-contiguous READ
    # replaces the active block); the margin covers the bridge's CAN TX queue, one Platform loop pass and pacing
    # delay the board's 2-gap catch-up did not recover. If it is ever too short, the cut block's unsent words are
    # holes for the next pass: slower, never wrong. The spacing floor bounds the IMU_CMD/IMU_REPLY rate (one each
    # per 15 ms, against the bridge's 7-usable-slot relay ring drained 4 per 10 ms tick). A pass ends serially
    # (the last block's frames land first) and the stall guard counts whole dry passes. False: one at a time.
    refill_pipeline: bool = True
    refill_spacing_s: float = 0.015
    refill_margin_s: float = 0.004
    # READ chaining (FW 13, spec section 4 "READ chaining"; host rule in section 7). While block k is in flight
    # the next CONTIGUOUS block k+1 is sent once k is within this much time of its end, judged two ways, the
    # earlier wins: the highest seq received is within ceil(chain_lead_s / gap) words of k's end, or the clock
    # is within chain_lead_s of k's predicted end. 80 ms is about 5x the measured 16 ms READ turnaround and
    # still covers a 20 ms worker poll plus the bridge's 10 ms batch flush (the progress estimate lags the
    # board by up to that much); about 100 words at the 0.8 ms pace. 0 turns chaining off.
    chain_lead_s: float = 0.08
    # This many consecutive OK (not READ_EXTENDED) replies to chained READs turn chaining off for the rest of
    # the upload: a board that replaces instead of extending (its header claimed FW 13 wrongly) costs at most
    # this many blocks of holes, which the refill pass heals. A single OK is a chain that arrived after the
    # block had ended (a fresh block: only the turnaround lost), so one alone is not evidence.
    chain_replace_limit: int = 2
    chain_check_s: float = 0.05        # wait this long after an OK to a chained READ before judging the cut block's range
    max_stalled_blocks: int = 4        # consecutive blocks that delivered nothing new
    max_passes: int = 8                # full sweeps of a region (pass 1 + hole refills)
    max_crc_rereads: int = 1           # spec: one full re-read, then write flagged
    # FW 14 chunk verification (spec section 6): re-read the words of chunks whose CRC disagrees this many times,
    # then flag the capture crc_ok = false; wait this long between reads of a chunk table not yet READY.
    max_chunk_rereads: int = 2
    chunk_table_wait_s: float = 0.05
    chunk_table_tries: int = 40
    status_grace_s: float = 2.5        # a STATUS this soon after FREEZE/RESUME may still show the old state
    span_margin_s: float = 2.0         # added each side of an Attempt span
    max_total_s: float = 300.0         # a whole upload is abandoned after this
    status_stale_s: float = 3.0        # no STATUS this long on a booted board: no IMU (spec section 4)
    fault_upload_delay_s: float = 5.0  # a Fault / Soft E-stop freezes at once but starts the READs this late
    fault_hold_max_s: float = 30.0     # ...and, while the robot is still Armed, holds them up to this long
    heal_grace_s: float = 3.0          # a ring FROZEN with the machine idle: RESUME our own capture after this
    heal_foreign_grace_s: float = 15.0  # ...or a frozen capture that is not ours (FREEZE probe learns its id)
    heal_retry_s: float = 5.0          # between heal attempts
    heal_max_tries: int = 3            # per frozen episode; then log an error and stop until the state changes


@dataclass
class UploadJob:
    t_start_wall: Optional[float] = None      # seconds; None = upload the whole frozen capture
    t_end_wall: Optional[float] = None
    reason: int = int(pc.PlatformImuFreezeReason.ATTEMPT_END)
    attempt_id: Optional[str] = None
    label: str = "attempt_end"
    read_delay_s: float = 0.0         # FREEZE now, start the READs this much later (Fault: the robot is busy)
    heal: bool = False                # a heal job: FREEZE to learn the id (or use ``heal_cid``), then RESUME only
    heal_cid: int = 0


@dataclass
class UploadResult:
    """What the machine hands the service once, at the end of an upload."""
    header: Optional[Header]
    anchors: List[Anchor]
    sample_words: bytes
    word_present: bytes
    k0: int
    flags: Dict[str, Any]
    stats: Dict[str, Any]
    job: UploadJob
    failure: str = ""
    frozen_cid: int = 0               # the capture id we froze (0 = never learned)
    freeze_lost: bool = False         # FREEZE unanswered: the ring may be frozen under an id we never saw
    resume_lost: bool = False         # RESUME unanswered: the ring may still be frozen under ``frozen_cid``


class _Region:
    """One address region being fetched: a word buffer and a presence bitmap. ``next_block`` always
    returns the first missing word from the scan position onward, so the first sweep, the hole refills
    and late tails from a replaced block all use one loss-recovery path (spec section 3).

    A block starts AND ends on a missing word, holds at most READ_MAX words, and bridges a run of
    present words only when that run is at most ``merge`` words long (IDEA-25: a refill READ covers one
    cluster of nearby holes, not every word from the first hole to the last hole in READ_MAX). Every
    word still missing when the scan reaches it is requested once per pass: the scan only moves to the
    end of the block just returned, and the next call starts at the first missing word at or after it,
    so a hole past a block's end (a long present run, or the READ_MAX limit) is the next block's start.
    A hole never delivered is re-requested each pass until ``UploadConfig.max_passes`` or
    ``max_stalled_blocks`` ends the upload."""

    def __init__(self, name: str, base: int, w_lo: int, w_hi: int, merge: int = READ_MAX):
        self.name, self.base, self.lo, self.hi = name, base, w_lo, w_hi
        self.merge = max(int(merge), 0)
        n = max(w_hi - w_lo, 0)
        self.n = n
        self.buf = bytearray(WORD * n)
        self.have = bytearray(n)
        self.n_have = 0
        self.scan = 0
        self.passes = 1
        # (missing count, its missing seqs, at most PASS1_SEQS) when the first sweep wrapped; None until then
        self.pass1: Optional[Tuple[int, List[int]]] = None

    # Every missing seq (a region holds at most 64,000 words): was 500, which cut captures 5730 and 5731 short.
    PASS1_SEQS = 1 << 16

    def at_pass_end(self) -> bool:
        """True when the next :meth:`next_block` would wrap to a new pass (no missing word at or after the scan)."""
        return not self.complete and self.have.find(b"\x00", self.scan) < 0

    def put(self, seq: int, word: bytes) -> int:
        """1 new word, 0 duplicate, -1 not in this region."""
        i = seq - self.base - self.lo
        if i < 0 or i >= self.n:
            return -1
        if self.have[i]:
            return 0
        self.buf[WORD * i:WORD * i + WORD] = word
        self.have[i] = 1
        self.n_have += 1
        return 1

    @property
    def complete(self) -> bool:
        return self.n_have >= self.n

    def next_block(self) -> Optional[Tuple[int, int]]:
        if self.complete:
            return None
        i = self.have.find(b"\x00", self.scan)
        if i < 0:                      # swept to the end with holes left: a new pass from the start
            if self.pass1 is None:
                self.pass1 = (self.missing(), self.missing_seqs(self.PASS1_SEQS))
            self.passes += 1
            self.scan = 0
            i = self.have.find(b"\x00", 0)
        lim = min(i + READ_MAX, self.n)
        have = self.have
        end = i                        # the block is [i, end); have[i] == 0 and have[end - 1] == 0
        while True:
            p = have.find(b"\x01", end, lim)          # first present word after this missing run
            if p < 0:                                   # missing to the READ_MAX limit or the region end
                end = lim
                break
            end = p
            q = have.find(b"\x00", p, lim)             # the next hole, after a present run [p, q)
            if q < 0 or q - p > self.merge:             # no further hole in reach, or the run is too long
                break
            end = q
        self.scan = end
        return self.base + self.lo + i, end - i

    def peek_block(self) -> Optional[Tuple[int, int]]:
        """What :meth:`next_block` would return, without moving the scan or counting a pass."""
        scan, passes, pass1 = self.scan, self.passes, self.pass1
        try:
            return self.next_block()
        finally:
            self.scan, self.passes, self.pass1 = scan, passes, pass1

    def missing(self) -> int:
        return self.n - self.n_have

    def missing_seqs(self, limit: int) -> List[int]:
        out: List[int] = []
        i = self.have.find(b"\x00")
        while i >= 0 and len(out) < limit:
            out.append(self.base + self.lo + i)
            i = self.have.find(b"\x00", i + 1)
        return out


@dataclass
class _Block:
    first_seq: int
    count: int
    t_sent: float
    deadline: float
    gap_eff: int
    last_rx: float = 0.0
    n_new: int = 0
    last_seen: bool = False
    replied: bool = False
    pred_end: float = 0.0              # when the board should send the block's last word (no slack)
    chained: bool = False              # sent as an extension of the block before it (FW 13)
    n_rx: int = 0                      # frames received in this block's seq range (new or duplicate)
    hi_rx: int = -1                    # highest seq received in this block's range
    no_chain: bool = False             # a chained READ on top of this block failed to queue: send the next fresh
    prev: Optional[Tuple[int, int]] = None   # (first_seq, count) of the block this one was chained onto
    t_acked: float = 0.0               # when send_read returned (the bridge had queued it): the refill spacing origin


# Phases
IDLE, FREEZING, WAITING, HEADER, ANCHORS, SAMPLES, RESUMING, DONE = (
    "idle", "freezing", "waiting", "header", "anchors", "samples", "resuming", "done")
# FW 14: the diag counters before and after the data, and the chunk-CRC table. Their errors and timeouts never fail
# the upload (an auxiliary read that fails only leaves that check undone).
DIAG0, DIAG1, CHUNKS = "diag0", "diag1", "chunks"
_AUX = (DIAG0, DIAG1, CHUNKS)
_READING = (HEADER, ANCHORS, SAMPLES) + _AUX


class UploadMachine:
    """Single-owner upload state machine. Every entry point takes ``now`` (monotonic seconds) and
    nothing else reads a clock, so tests drive it with a fake Platform and fake time.

    ``send_ctrl(op, reason, capture_id) -> (ok, msg)`` and ``send_read(capture_id, first_seq, count,
    gap_10us) -> (ok, msg)`` queue one RPC. OK means queued on the bus, not accepted: the verdict is
    the IMU_REPLY the caller feeds back through :meth:`on_reply`.

    ``ack_clock`` (optional, the one exception to "no clock"): read once right after each ``send_read``
    returns, as the origin of the pipelined refill spacing. ``send_read`` is a blocking RPC whose OK means the
    can-bridge has queued the READ on its CAN TX, so timing from its return keeps the host's UDP latency out of
    the spacing argument. Without it (tests) the origin is ``now``, which the fake board treats identically.
    """

    def __init__(self, send_ctrl: Callable[..., Tuple[bool, str]], send_read: Callable[..., Tuple[bool, str]],
                 on_result: Callable[[UploadResult], None], *, config: Optional[UploadConfig] = None,
                 is_armed: Callable[[], bool] = lambda: False, ack_clock: Optional[Callable[[], float]] = None):
        self._ctrl, self._read, self._on_result = send_ctrl, send_read, on_result
        self.cfg = config or UploadConfig()
        self._is_armed = is_armed
        self._ack_clock = ack_clock
        self.phase = IDLE
        self._reset()

    # -- bookkeeping --
    def _reset(self) -> None:
        self.job: Optional[UploadJob] = None
        self.cid = 0
        self.header: Optional[Header] = None
        self.anchors: List[Anchor] = []
        self.region: Optional[_Region] = None
        self.anchor_region: Optional[_Region] = None
        self.sample_region: Optional[_Region] = None
        self.k0 = 0
        self.k1 = 0
        self.full_coverage = True
        self.blk: Optional[_Block] = None
        self.ahead: Optional[_Block] = None    # the chained READ sent while ``blk`` is in flight (FW 13)
        self._chain = False                    # set from the header's fw_version (CHAIN_MIN_FW_VERSION)
        self._replace_streak = 0
        self._cut: Optional[tuple] = None      # (region, lo index, count, due): an OK to a chained READ awaiting its verdict
        self._cmd: Optional[dict] = None
        self.flags: Dict[str, Any] = {}
        self.stats: Dict[str, Any] = {}
        self.failure = ""
        self.t_job0 = 0.0
        self.t_freeze_reply = 0.0
        self.boot_low: Optional[int] = None
        self.stalled = 0
        self.rereads_left = 0
        self._abort_reason: Optional[str] = None
        self._wait_until = 0.0
        self._wait_cap = 0.0
        self.expected_n: Optional[int] = None
        self._verify = False                   # FW 14: chunk table and diag regions (VERIFY_MIN_FW_VERSION)
        self._chunk_state = "todo"             # todo | done | unavailable
        self._chunk_table: Optional[Dict[int, Tuple[int, int]]] = None   # index -> (crc32, info)
        self._chunk_tries = 0
        self._chunk_retry_at: Optional[float] = None
        self._chunk_rereads_left = 0
        self._chunks_ok: Optional[bool] = None
        self._payload_ok: Optional[bool] = None
        self._refill_verify = False
        self._diag0: Optional[Dict[str, int]] = None
        self._bridge0: Optional[Tuple[int, int]] = None
        self._bridge1: Optional[Tuple[int, int]] = None
        self._jet_rx = 0                       # frames in the batches after the first noted one (bridge cross-check)
        # Pipelined refill: (first_seq, count) of recent refill READs already ended host-side, so a late reply is
        # matched to its own READ; n_have at the start of the current pass (the pass-level stall guard); when the
        # samples region's first sweep ended (refill_s).
        self._refill_out: Deque[Tuple[int, int]] = deque(maxlen=16)
        self._pass_have0: Optional[int] = None
        self._t_refill0: Optional[float] = None
        self._t_refill1: Optional[float] = None
        self.frames_total = 0
        self.frames_done = 0
        self._rate_t = 0.0
        self._rate_n = 0
        self._rate = 0.0

    @property
    def busy(self) -> bool:
        return self.phase not in (IDLE, DONE)

    def effective_gap_10us(self, requested: Optional[int] = None) -> int:
        """The gap the frames will really be paced at: the requested one, never below the board's own
        floor, and while Armed never below the bridge's clamp (63 x 10 us)."""
        g = max(int(self.cfg.gap_10us if requested is None else requested), PLATFORM_MIN_GAP_10US)
        if self._is_armed():
            g = max(g, BRIDGE_ARMED_MIN_GAP_10US)
        return min(g, 255)

    def progress(self) -> Dict[str, Any]:
        tot, done = self.frames_total, self.frames_done
        eta = ((tot - done) / self._rate) if self._rate > 1.0 and tot > done else None
        return {"phase": self.phase, "frames_done": done, "frames_total": tot, "eta_s": eta,
                "rate_fps": self._rate}

    # -- commands --
    def start(self, job: UploadJob, now: float, boot_hint: Optional[int] = None) -> None:
        if self.busy:
            raise RuntimeError("upload already in progress")
        self._reset()
        self.boot_low = boot_hint          # the boot byte of the last STATUS heard before this upload
        self.job = job
        self.t_job0 = now
        self.rereads_left = self.cfg.max_crc_rereads
        self._chunk_rereads_left = self.cfg.max_chunk_rereads
        self.stats = {"blocks": 0, "block_timeouts": 0, "frames_rx": 0, "dup_frames": 0, "stray_frames": 0,
                      "hole_passes": 0, "crc_rereads": 0, "block_gap_10us": 0, "reads_chained": 0,
                      "reads_replaced": 0, "reads_late": 0, "chain": False, "reads_pipelined": 0,
                      "refill_s": 0.0}
        self.flags = {"complete": False, "crc_checked": False, "crc_ok": None, "crc_method": None, "aborted": False,
                      "platform_rebooted": False, "already_frozen": False, "partial_span": False,
                      "span_unavailable": False, "resume_ok": None, "holes_unfilled": 0}
        if job.heal and job.heal_cid:
            self.cid = job.heal_cid                 # our own capture whose RESUME was lost: RESUME it directly
            self._resume(now)
            return
        self.phase = FREEZING
        self._send_cmd(OP.FREEZE, now, reason=job.reason)

    def abort(self, reason: str, now: float) -> None:
        """Pacifying command or operator abort: stop the transfer, write what arrived (complete =
        false), and RESUME so the ring records again."""
        if not self.busy:
            return
        self.flags["aborted"] = True
        if self.phase in (WAITING,) + _READING:
            # RESUME is legal from UPLOADING and cuts the block (spec section 4 table), so no ABORT first:
            # one IMU_CMD at a time keeps the bridge's reply ring within its sizing assumption.
            self._fail("aborted: " + reason, now)
        elif self.phase == FREEZING:
            self._abort_reason = reason             # settle the FREEZE first, then give the ring back
        # RESUMING: already giving the ring back

    # -- RPC verdicts --
    def _send_cmd(self, op: int, now: float, *, reason: int = 0) -> None:
        timeout = self.cfg.freeze_timeout_s if op == OP.FREEZE else self.cfg.cmd_timeout_s
        tries = (self._cmd["tries"] + 1) if self._cmd and self._cmd["op"] == op else 1
        self._cmd = {"op": int(op), "deadline": now + timeout, "tries": tries, "reason": reason}
        ok, msg = self._ctrl(int(op), reason, self.cid if op == OP.RESUME else 0)
        if not ok:
            logger.warning("IMU %s not queued: %s", OP(op).name, msg)

    def _start_block(self, now: float) -> bool:
        """Send the next READ for the current region. False when the region is complete."""
        r = self.region
        passes0 = r.passes if r else 0
        nb = r.next_block() if r else None
        if nb is None:
            return False
        if r.passes != passes0:                    # this block starts a new pass
            self._pass_have0 = r.n_have
            if r is self.sample_region and self._t_refill0 is None:
                self._t_refill0 = now              # the samples' first sweep is over: refill_s runs from here
        first, count = nb
        gap = self.effective_gap_10us()
        span = count * gap * 10e-6
        self.blk = _Block(first, count, now, now + span + self.cfg.block_slack_s, gap, last_rx=now,
                          pred_end=now + span)
        self.ahead = None
        self.stats["blocks"] += 1
        self.stats["block_gap_10us"] = gap
        if r.passes > 1:
            self.stats["hole_passes"] = max(self.stats["hole_passes"], r.passes - 1)
        ok, msg = self._read(self.cid, first, count, gap)
        self.blk.t_acked = self._ack_clock() if self._ack_clock is not None else now
        if not ok:
            logger.warning("IMU READ not queued: %s", msg)
        return True

    # -- pipelined refill (IDEA-25) --
    def _pipelining(self) -> bool:
        """True while the current region is in a hole-refill pass that pipelines its READs (config section)."""
        r = self.region
        return (self.cfg.refill_pipeline and r is not None and r.passes > 1 and self.phase in (ANCHORS, SAMPLES)
                and not r.complete)

    def _pipe_spacing(self, b: _Block) -> float:
        """Seconds after ``b`` was acknowledged at which the next refill READ may go out: the whole block on the
        board's pace schedule plus the margin, never under the spacing floor."""
        return max(self.cfg.refill_spacing_s, b.count * b.gap_eff * 10e-6 + self.cfg.refill_margin_s)

    def _pipe_ready(self, b: _Block) -> bool:
        """``b`` may be ended host-side without waiting for its frames: a fresh (not chained) refill block with
        nothing chained ahead of it, whose region has another block in THIS pass (a pass ends serially, so its
        last block's frames land before the next pass is planned from the presence map)."""
        return (self._pipelining() and self.ahead is None and not b.chained and not self.region.at_pass_end())

    def pipeline_due_in(self, now: float) -> Optional[float]:
        """Seconds until :meth:`tick` would send the next pipelined refill READ (0 if due), or None when none is
        pending. The worker waits at most this long for the next batch (as with :meth:`settle_due_in`)."""
        b = self.blk
        if b is None or not self._pipe_ready(b):
            return None
        return max(0.0, self._pipe_spacing(b) - (now - b.t_acked))

    def _maybe_chain(self, now: float) -> None:
        """READ chaining (FW 13): keep one READ ahead so the Platform never idles between contiguous blocks.

        The rule, in full (spec section 7, "READ chaining, host side"):

        * Only when this capture's header says FW >= CHAIN_MIN_FW_VERSION, and chaining was not turned off by
          ``chain_replace_limit`` consecutive OK replies to chained READs. Header READs are never chained (the
          header decides).
        * At most one READ ahead: ``self.ahead`` is None, and the current block shows it is on the board (its
          OK/READ_EXTENDED reply or at least one of its frames arrived) and has not ended (last word not seen).
        * When: the highest seq received is within ceil(chain_lead_s / gap) words of the block's end, or the
          clock is within ``chain_lead_s`` of its predicted end, whichever is first.
        * Only the CONTIGUOUS next block: ``first_seq`` equals the current block's end, in the same pass of the
          same region. A hole-fill READ (any block not starting at that seq) is never chained: it goes out fresh
          once the current block ends, exactly as before FW 13.
        * The chained block's deadline runs from the current block's predicted end (``count x gap`` after it,
          plus ``block_slack_s``); its gap is the effective gap NOW, so the Armed clamp still applies per READ.
        """
        b, r = self.blk, self.region
        if (not self._chain or self.ahead is not None or b is None or r is None or b.no_chain or b.last_seen
                or self.phase not in (ANCHORS, SAMPLES) or not (b.replied or b.n_rx)):
            return
        end = b.first_seq + b.count
        lead_s = self.cfg.chain_lead_s
        lead_words = int(math.ceil(lead_s / (b.gap_eff * 10e-6)))
        if b.hi_rx < end - 1 - lead_words and now < b.pred_end - lead_s:
            return
        nb = r.peek_block()
        if nb is None or nb[0] != end:
            return                         # the next block is a hole refill (not contiguous): never chained
        passes = r.passes
        r.next_block()
        assert r.passes == passes          # a contiguous block is never the start of a new pass
        first, count = nb
        gap = self.effective_gap_10us()
        start = max(now, b.pred_end)
        span = count * gap * 10e-6
        a = _Block(first, count, now, start + span + self.cfg.block_slack_s, gap, last_rx=now,
                   pred_end=start + span, chained=True, prev=(b.first_seq, b.count))
        self.ahead = a
        self.stats["blocks"] += 1
        self.stats["block_gap_10us"] = gap
        ok, msg = self._read(self.cid, first, count, gap)
        if not ok:
            # Certainly not sent: forget it, let the region offer those words again, and send the next block
            # fresh once this one ends (no second chain attempt on this block).
            logger.warning("IMU chained READ not queued: %s", msg)
            self.ahead = None
            r.scan = first - r.base - r.lo
            b.no_chain = True

    def on_reply(self, rep: ImuReply, now: float) -> None:
        if not self.busy:
            return
        if self.phase in _READING:
            if rep.op == OP.READ:
                self._on_read_reply(rep, now)
            return
        cmd = self._cmd
        if cmd is None or rep.op != cmd["op"]:
            return
        if self.phase == FREEZING:
            self._on_freeze_reply(rep, now, cmd)
        elif self.phase == RESUMING:
            if rep.status in (ST.OK, ST.ERR_NOT_FROZEN, ST.ERR_WRONG_CAPTURE):
                self.flags["resume_ok"] = rep.status == ST.OK
                self._finish(now)
            elif rep.status in (ST.ERR_DISABLED, ST.ERR_FW_UPDATE):
                self.flags["resume_ok"] = False
                self._finish(now)

    def _on_freeze_reply(self, rep: ImuReply, now: float, cmd: dict) -> None:
        st = rep.status
        if st in (ST.OK, ST.ALREADY_FROZEN):
            if st == ST.ALREADY_FROZEN and cmd["tries"] == 1:
                # An earlier capture was still waiting: this Attempt was NOT recorded. Upload the old one
                # (spec section 6) in full, since the Attempt span does not apply to it.
                self.flags["already_frozen"] = True
                self.job.t_start_wall = self.job.t_end_wall = None
            # ALREADY_FROZEN on a RETRY is our own FREEZE whose reply was lost: not flagged.
            self.cid = rep.capture_id
            self.expected_n = rep.detail if st == ST.OK else None
            self.t_freeze_reply = now
            self._cmd = None
            if self._abort_reason is not None:
                self.flags["aborted"] = True
                self._fail("aborted: " + self._abort_reason, now)
                return
            if self.job.heal:                       # a heal only wanted the id: give the ring back
                self.flags["healed"] = True
                self._resume(now)
                return
            if self.job.read_delay_s > 0:
                self.phase = WAITING
                self._wait_until = now + self.job.read_delay_s
                self._wait_cap = now + max(self.job.read_delay_s, self.cfg.fault_hold_max_s)
                return
            self._begin_region_header(now)
            return
        # Refused with nothing changed: no capture to fetch and nothing to RESUME.
        names = {ST.ERR_EMPTY: "no samples", ST.ERR_DISABLED: "IMU disabled", ST.ERR_FW_UPDATE: "firmware update"}
        self._fail("FREEZE refused: %s" % names.get(st, "status 0x%02X" % st), now,
                   no_resume="nothing frozen (FREEZE refused)")

    def _check_cut(self, now: float, force: bool = False) -> None:
        """Judge an OK to a chained READ once the cut block's frames have had ``chain_check_s`` to land: holes
        left in its range mean the board replaced the block (counts toward ``chain_replace_limit``); a range
        that is complete means the chain merely arrived late (resets the streak)."""
        c = self._cut
        if c is None or (now < c[3] and not force):
            return
        self._cut = None
        reg, lo, n, _ = c
        if all(reg.have[max(lo, 0):max(lo, 0) + n]):
            self.stats["reads_late"] += 1
            self._replace_streak = 0
            return
        self.stats["reads_replaced"] += 1
        self._replace_streak += 1
        if self._replace_streak >= self.cfg.chain_replace_limit and self._chain:
            self._chain = False
            logger.warning("IMU READ chaining off for this upload: %d chained READs in a row were answered "
                           "OK (the board replaced the block instead of extending it)", self._replace_streak)

    def _on_read_reply(self, rep: ImuReply, now: float) -> None:
        b = self.blk
        if b is None:
            return
        if self.phase in _AUX and rep.status not in (ST.OK, ST.READ_EXTENDED, ST.ERR_DISABLED, ST.ERR_FW_UPDATE,
                                                      ST.ERR_NOT_FROZEN, ST.ERR_WRONG_CAPTURE):
            # FW 14 auxiliary READ (diag or chunk table): a refusal about the region itself (ERR_OUT_OF_RANGE,
            # ERR_BAD_ARGS, anything unknown) means "unavailable", never a failed upload. The four codes that
            # mean the board LEFT the frozen capture (disabled, firmware update, not frozen, wrong capture)
            # fall through to the data-READ handling below (_fail, and no RESUME where the board is gone).
            # (a firmware-13 board answers ERR_OUT_OF_RANGE; this host only asks an FW 14 header, but a board
            # that rebooted in between is caught by the next data READ or the RESUME, as before).
            try:
                name = ST(rep.status).name
            except ValueError:
                name = "status 0x%02X" % rep.status
            self._aux_give_up(now, name)
            return
        if rep.status in (ST.OK, ST.READ_EXTENDED):
            tgt = None
            for x in (self.ahead, b):
                if x is not None and rep.read_first_seq == x.first_seq and rep.read_count == x.count:
                    tgt = x
                    break
            if tgt is None:
                key = (rep.read_first_seq, rep.read_count)
                if key in self._refill_out:            # a pipelined refill READ's own (late) reply
                    self._refill_out.remove(key)
                    return
                if self.ahead is None:
                    b.replied = True               # unmatched (a stale reply): as before chaining existed
                return
            if tgt.replied:
                return                             # a duplicate reply: counted once
            tgt.replied = True
            if not tgt.chained:
                return
            if rep.status == ST.READ_EXTENDED:
                self.stats["reads_chained"] += 1
                self._replace_streak = 0
                return
            # OK to a chained READ: the board started it as a FRESH block. Either it arrived after the block
            # before it had ended (a late chain: only the turnaround lost, nothing cut) or the board replaced
            # that block (an FW 12 board: its unsent words are now holes, healed by the refill). Which one is
            # only known once the cut block's frames have all landed: remember its range and judge it at the
            # first tick/advance after ``chain_check_s`` (_check_cut). The block before is over on the board
            # either way: end it now, without counting a stall (it was cut, not silent).
            if self._cut is not None:
                self._check_cut(now, force=True)
            if tgt.prev is not None and self.region is not None:
                lo = tgt.prev[0] - self.region.base - self.region.lo
                self._cut = (self.region, lo, tgt.prev[1], now + self.cfg.chain_check_s)
            if tgt is self.ahead:
                self._next_block(now, timed_out=False, stall=False)
        elif rep.status in (ST.ERR_NOT_FROZEN, ST.ERR_WRONG_CAPTURE):
            self.flags["platform_rebooted"] = True
            self._fail("READ refused (%s): the Platform rebooted or was resumed" % ST(rep.status).name, now,
                       no_resume="Platform rebooted or already resumed")
        elif rep.status == ST.ERR_OUT_OF_RANGE:
            self._fail("READ out of range (seq 0x%04X, region end %d)" % (rep.read_first_seq, rep.read_count),
                       now)
        elif rep.status == ST.ERR_BAD_ARGS:
            # The board is still FROZEN/UPLOADING and would stay so: give the ring back (audit B2).
            self._fail("READ refused (ERR_BAD_ARGS)", now)
        elif rep.status in (ST.ERR_DISABLED, ST.ERR_FW_UPDATE):
            self._fail("READ refused (%s)" % ST(rep.status).name, now,
                       no_resume="IMU disabled (capture discarded) or firmware update (halted)")

    def on_status(self, st: ImuStatus, now: float) -> None:
        """A 1 Hz IMU_STATUS. During a transfer it is the reboot detector."""
        if not self.busy:
            return
        if self.boot_low is None:
            self.boot_low = st.boot
        if self.phase not in (WAITING,) + _READING:
            return
        if self.header is not None and st.boot != (self.header.boot_id & 0xFF):
            self.flags["platform_rebooted"] = True
            self._fail("boot changed mid-upload (status boot %d, header boot_id low byte %d)"
                       % (st.boot, self.header.boot_id & 0xFF), now, no_resume="Platform rebooted")
        elif now - self.t_freeze_reply >= self.cfg.status_grace_s and st.state not in (STATE.FROZEN, STATE.UPLOADING):
            self.flags["platform_rebooted"] = True
            self._fail("Platform left FROZEN/UPLOADING mid-upload (state %d)" % st.state, now,
                       no_resume="Platform rebooted (left FROZEN)")

    # -- data --
    def on_records(self, records: bytes, now: float) -> None:
        """Feed ``[seq u16][6 B]`` records (8 bytes each) from batches."""
        if self.phase not in _READING:
            if records:
                self.stats["stray_frames"] = self.stats.get("stray_frames", 0) + len(records) // 8
            return
        for o in range(0, len(records) - 7, 8):
            seq = records[o] | (records[o + 1] << 8)
            word = records[o + 2:o + 8]
            self.stats["frames_rx"] += 1
            if self.region is not None:
                r = self.region.put(seq, word)     # a late tail of a finished region lands outside: stray
                # Attribute the frame: to the chained block when it is in that block's range, else to the
                # current block (a straggler from an earlier block counts as today: it shows the link alive).
                tgt = self.blk
                ah = self.ahead
                if ah is not None and ah.first_seq <= seq < ah.first_seq + ah.count:
                    tgt = ah
                if r >= 0 and tgt is not None:
                    tgt.last_rx = now
                    if tgt.first_seq <= seq < tgt.first_seq + tgt.count:
                        tgt.n_rx += 1
                        tgt.hi_rx = max(tgt.hi_rx, seq)
                if r == 1:
                    self.frames_done += 1
                    if tgt is not None:
                        tgt.n_new += 1
                        if seq == tgt.first_seq + tgt.count - 1:
                            tgt.last_seen = True
                elif r == 0:
                    self.stats["dup_frames"] += 1
                else:
                    self.stats["stray_frames"] += 1
        self._advance(now)

    # -- the clock --
    def tick(self, now: float) -> None:
        if not self.busy:
            return
        if now - self.t_job0 > self.cfg.max_total_s and self.phase in (WAITING,) + _READING:
            self._fail("upload exceeded %.0f s" % self.cfg.max_total_s, now)
            return
        if self.phase == WAITING:
            if now >= self._wait_until and (not self._is_armed() or now >= self._wait_cap):
                self._begin_region_header(now)
            return
        if self.phase in (FREEZING, RESUMING):
            cmd = self._cmd
            if cmd and now >= cmd["deadline"]:
                if cmd["tries"] >= self.cfg.cmd_retries:
                    if self.phase == FREEZING:
                        # The FREEZE may have been applied with every reply lost: the id is unknown, so the
                        # service heals from a STATUS that stays FROZEN (FREEZE probe, then RESUME).
                        self.flags["freeze_lost"] = True
                        self._fail("no reply to FREEZE", now, no_resume="FREEZE unanswered (healed from STATUS)")
                    else:
                        self.flags["resume_ok"] = False
                        self.flags["resume_lost"] = True
                        self._fail("no reply to RESUME (the ring may still be frozen)", now,
                                   no_resume="RESUME unanswered (healed from STATUS)")
                else:
                    self._send_cmd(cmd["op"], now, reason=cmd["reason"])
            return
        self._rate_update(now)
        self._check_cut(now)
        if self.phase == CHUNKS and self.blk is None and self._chunk_retry_at is not None and now >= self._chunk_retry_at:
            self._chunk_retry_at = None
            self._begin_chunks(now)
            return
        b = self.blk
        if b is None:
            return
        ah = self.ahead
        if ah is not None:
            # Two blocks outstanding. The current one ends the moment the board is seen on the chained one
            # (no settle: the next READ is already on the board, so waiting would buy nothing); otherwise it
            # times out on its own deadline, the quiet time counting frames of either block.
            if b.last_seen or ah.n_rx:
                self._next_block(now, timed_out=False)
            elif now >= b.deadline and now - max(b.t_sent, b.last_rx, ah.last_rx) >= self.cfg.block_slack_s:
                self.stats["block_timeouts"] += 1
                self._next_block(now, timed_out=True)
            return
        if self._pipe_ready(b) and now - b.t_acked >= self._pipe_spacing(b):
            # Pipelined refill: the board has finished ``b`` by now (spacing rule); its frames are still in
            # flight and fill their words at region level whichever block is current. Not a stall: the guard
            # judges whole passes in a refill pass (_next_block).
            if not b.replied:
                self._refill_out.append((b.first_seq, b.count))
            self.stats["reads_pipelined"] += 1
            self._next_block(now, timed_out=False, stall=False)
            return
        quiet = now - max(b.t_sent, b.last_rx)
        if b.last_seen and now - b.last_rx >= self.cfg.settle_s:
            self._next_block(now, timed_out=False)
        elif now >= b.deadline and quiet >= self.cfg.block_slack_s:
            self.stats["block_timeouts"] += 1
            self._next_block(now, timed_out=True)
        else:
            self._maybe_chain(now)

    def settle_due_in(self, now: float) -> Optional[float]:
        """Seconds until :meth:`tick` would end the current block on ``settle_s`` (0 if already due), or None
        when no settle is pending. The worker waits at most this long for the next batch, so the settle is
        ``settle_s`` and not the 20 ms poll (the bridge flushes batches every 10 ms; after a block's last
        batch nothing else wakes the worker)."""
        b = self.blk
        if b is None or not b.last_seen or self.ahead is not None or self.phase not in _READING:
            return None
        return max(0.0, self.cfg.settle_s - (now - b.last_rx))

    def _rate_update(self, now: float) -> None:
        if now - self._rate_t >= 1.0:
            if self._rate_t:
                self._rate = (self.frames_done - self._rate_n) / (now - self._rate_t)
            self._rate_t, self._rate_n = now, self.frames_done

    def _advance(self, now: float) -> None:
        self._check_cut(now)
        for _ in range(4):                 # a promoted chained block may itself be done already (one batch)
            b = self.blk
            if b is None or self.region is None:
                return
            if self.region.complete:
                self.blk = self.ahead = None
                self._region_done(now)
                return
            lo = (b.first_seq - self.region.base - self.region.lo)
            ah = self.ahead
            if all(self.region.have[lo:lo + b.count]) or (ah is not None and (b.last_seen or ah.n_rx)):
                promote = ah is not None
                self._next_block(now, timed_out=False)
                if promote and self.blk is ah:
                    continue
                return
            self._maybe_chain(now)
            return

    def _next_block(self, now: float, timed_out: bool, stall: bool = True) -> None:
        """End the current block. With a chained block outstanding it becomes current (no READ is sent: it
        is already on the board); otherwise the next READ goes out. ``stall`` False: the block was cut by
        the board (an OK to a chained READ), so delivering nothing is not counted toward the stall guard."""
        b = self.blk
        pipe = self._pipelining()
        if pipe:
            # Pipelined refill pass: a block is ended before its frames land, so its own n_new means nothing.
            # The guard judges the whole pass instead, once its last block has settled: a pass that filled no
            # word counts one stall (max_stalled_blocks such passes in a row end the upload; max_passes and
            # max_total_s still apply on top).
            if self.ahead is None and self.region.at_pass_end():
                have0 = self._pass_have0
                self.stalled = 0 if (have0 is None or self.region.n_have > have0) else self.stalled + 1
                self._pass_have0 = self.region.n_have
        elif b is not None and stall:
            self.stalled = 0 if b.n_new else self.stalled + 1
        if self.stalled > self.cfg.max_stalled_blocks:
            self._fail("upload stalled: %d consecutive %s delivered nothing"
                       % (self.stalled, "refill passes" if pipe else "blocks"), now)
            return
        if self.region.complete:
            self.blk = self.ahead = None
            self._region_done(now)
            return
        if self.ahead is not None:
            self.blk, self.ahead = self.ahead, None
            self._maybe_chain(now)
            return
        if self.phase in _AUX and (timed_out or self.region.passes > 2):
            # An auxiliary region is small (one READ): a timeout or a second refill pass gives up on it, and the
            # stall guard of the data regions is not touched.
            self.stalled = 0
            self._aux_give_up(now, "timed out" if timed_out else "words missing after 2 passes")
            return
        if self.region.passes > self.cfg.max_passes:
            self.flags["holes_unfilled"] = self.region.missing()
            self._fail("%d word(s) of the %s region still missing after %d passes"
                       % (self.region.missing(), self.region.name, self.cfg.max_passes), now)
            return
        self._start_block(now)

    # -- phases --
    def _begin_region_header(self, now: float) -> None:
        self.phase = HEADER
        self.region = _Region("header", HEADER_BASE, 0, HEADER_WORDS, self.cfg.refill_merge_words)
        self.frames_total = HEADER_WORDS
        self.frames_done = 0
        self._start_block(now)

    def _region_done(self, now: float) -> None:
        if self.region is self.sample_region and self._t_refill0 is not None and self._t_refill1 is None:
            # The samples' hole refill, first sweep's end to region complete (once: a CRC re-read does not move it).
            self._t_refill1 = now
            self.stats["refill_s"] = now - self._t_refill0
        if self.phase == HEADER:
            self._header_done(now)
        elif self.phase == ANCHORS and self._refill_verify:
            self.anchors = decode_anchors(bytes(self.anchor_region.buf))
            if not self.sample_region.complete:
                self.phase = SAMPLES
                self.region = self.sample_region
                self._start_block(now)
            else:
                self._verify_chunks(now)
        elif self.phase == ANCHORS:
            self._anchors_done(now)
        elif self.phase == SAMPLES and self._refill_verify:
            self._verify_chunks(now)
        elif self.phase == SAMPLES:
            self._samples_done(now)
        elif self.phase == DIAG0:
            self._diag_done(now, decode_diag(bytes(self.region.buf)))
        elif self.phase == DIAG1:
            self._diag_done(now, decode_diag(bytes(self.region.buf)))
        elif self.phase == CHUNKS:
            self._chunk_table_done(now)

    # -- FW 14: diag counters and chunk verification --
    def note_bridge(self, rx_frames: int, batches_sent: int, n_frames: int = 0) -> None:
        """The can-bridge's cumulative IMU_DATA counters from each batch of this upload (first and last kept),
        called before the batch's ``n_frames`` records are fed. The first batch's own frames are already inside
        its ``rx_frames``, so the Jetson side counts only the batches after it (``stats.bridge_rx_minus_jetson_rx``)."""
        if self.phase in _READING:
            if self._bridge0 is None:
                self._bridge0 = (int(rx_frames), int(batches_sent))
            else:
                self._jet_rx += int(n_frames)
            self._bridge1 = (int(rx_frames), int(batches_sent))

    def _begin_aux(self, phase: str, name: str, base: int, lo: int, hi: int, now: float) -> None:
        self.phase = phase
        self.region = _Region(name, base, lo, hi, self.cfg.refill_merge_words)
        self.blk = self.ahead = None
        self._start_block(now)

    def _aux_give_up(self, now: float, why: str) -> None:
        self.blk = self.ahead = None
        if self.phase in (DIAG0, DIAG1):
            self.stats["diag_unavailable"] = why
            logger.info("Platform IMU diag counters unavailable (%s); the upload continues", why)
            self._diag_done(now, None)
        elif self.phase == CHUNKS:
            self._chunk_state = "unavailable"
            self.stats["chunk_table_unavailable"] = why
            logger.warning("Platform IMU chunk-CRC table unavailable (%s): the span is not verified", why)
            self._payload_and_finish(now)

    def _diag_done(self, now: float, d: Optional[Dict[str, int]]) -> None:
        if self.phase == DIAG0:
            self._diag0 = d
            self._begin_region_anchors(now)
            return
        if d is not None and self._diag0 is not None:
            dd = diag_deltas(self._diag0, d)
            if self._bridge0 is not None and self._bridge1 is not None:
                dd["bridge_rx_frames"] = (self._bridge1[0] - self._bridge0[0]) % (1 << 32)
                dd["bridge_batches_sent"] = (self._bridge1[1] - self._bridge0[1]) % (1 << 32)
            self.stats["platform_tx"] = dd
        self._resume(now)

    def _chunk_list(self) -> List[Tuple[int, "_Region", int, int]]:
        """(table index, region, first word, words) for every chunk the assembled regions hold in full."""
        out: List[Tuple[int, _Region, int, int]] = []
        h, sr, ar = self.header, self.sample_region, self.anchor_region
        total = 2 * h.n_samples
        c = sr.lo // CHUNK_WORDS
        while c * CHUNK_WORDS < sr.hi:
            start = c * CHUNK_WORDS
            n = min(CHUNK_WORDS, total - start)
            if start >= sr.lo and start + n <= sr.hi:
                out.append((c, sr, start, n))
            c += 1
        total = ANCHOR_WORDS * h.n_anchors
        for j in range(-(-total // CHUNK_WORDS)):
            out.append((SAMPLE_CHUNKS + j, ar, j * CHUNK_WORDS, min(CHUNK_WORDS, total - j * CHUNK_WORDS)))
        return out

    def _begin_chunks(self, now: float) -> None:
        if self._chunk_table is not None:              # immutable while frozen: read once per upload
            self._verify_chunks(now)
            return
        lo = min((c for c, _, _, _ in self._chunk_list()), default=SAMPLE_CHUNKS)
        self._chunk_tries += 1
        self._begin_aux(CHUNKS, "chunks", CHUNK_BASE, lo, CHUNK_TABLE_WORDS, now)

    def _chunk_table_done(self, now: float) -> None:
        r = self.region
        table = {}
        for i in range(r.n):
            table[r.lo + i] = struct.unpack_from("<IH", r.buf, WORD * i)
        need = [c for c, _, _, _ in self._chunk_list()]
        if any(not (table[c][1] & CHUNK_READY) for c in need):
            # Computed lazily after FREEZE (under 0.5 s on the board): read the table again shortly.
            if self._chunk_tries >= self.cfg.chunk_table_tries:
                self._aux_give_up(now, "table not ready after %d reads" % self._chunk_tries)
                return
            self.blk = self.ahead = None
            self._chunk_retry_at = now + self.cfg.chunk_table_wait_s
            return
        self._chunk_table = table
        self._verify_chunks(now)

    def _verify_chunks(self, now: float) -> None:
        """Check every chunk the regions hold against the board's table; re-read the words of the chunks that
        disagree (bounded), then flag the capture."""
        self._refill_verify = False
        chunks = self._chunk_list()
        bad = []
        for c, reg, start, n in chunks:
            crc, info = self._chunk_table[c]
            o = WORD * (start - reg.lo)
            if (info & ~CHUNK_READY) != n or (zlib.crc32(bytes(reg.buf[o:o + WORD * n])) & 0xFFFFFFFF) != crc:
                bad.append((c, reg, start, n))
        self.stats["chunks_checked"] = len(chunks)
        if not bad:
            self._chunks_ok = True
            self._chunk_state = "done"
            self._payload_and_finish(now)
            return
        self.stats["chunk_mismatches"] = self.stats.get("chunk_mismatches", 0) + len(bad)
        self.stats["chunk_bad_last"] = [c for c, _, _, _ in bad]
        if self._chunk_rereads_left > 0:
            self._chunk_rereads_left -= 1
            self.stats["chunk_rereads"] = self.stats.get("chunk_rereads", 0) + 1
            logger.warning("IMU chunk CRC mismatch in %d chunk(s) %s: re-reading their words",
                           len(bad), [c for c, _, _, _ in bad])
            for _, reg, start, n in bad:
                i = start - reg.lo
                reg.have[i:i + n] = bytes(n)
                reg.n_have = sum(reg.have)
                reg.scan = 0
                reg.passes = 1
            self.stalled = 0
            self._refill_verify = True
            reg = self.anchor_region if not self.anchor_region.complete else self.sample_region
            self.phase = ANCHORS if reg is self.anchor_region else SAMPLES
            self.region = reg
            self._start_block(now)
            return
        self._chunks_ok = False
        self._chunk_state = "done"
        logger.error("IMU chunk CRC still wrong after %d re-read(s) in chunk(s) %s: the capture is flagged "
                     "crc_ok = false", self.cfg.max_chunk_rereads, [c for c, _, _, _ in bad])
        self._payload_and_finish(now)

    def _header_done(self, now: float) -> None:
        try:
            h = decode_header(bytes(self.region.buf))
        except HeaderError as e:
            self._fail("header invalid: %s" % e, now)
            return
        if h.capture_id != self.cid:
            self._fail("header capture_id 0x%04X != reply 0x%04X" % (h.capture_id, self.cid), now)
            return
        if self.expected_n is not None and h.n_samples != self.expected_n:
            self._fail("header n_samples %d != FREEZE reply %d" % (h.n_samples, self.expected_n), now)
            return
        if self.boot_low is not None and (h.boot_id & 0xFF) != self.boot_low:
            self.flags["platform_rebooted"] = True
            self._fail("boot_id low byte %d != status boot %d" % (h.boot_id & 0xFF, self.boot_low), now,
                       no_resume="Platform rebooted")
            return
        self.header = h
        self._chain = h.fw_version >= CHAIN_MIN_FW_VERSION and self.cfg.chain_lead_s > 0
        self.stats["chain"] = self._chain
        self._verify = h.fw_version >= VERIFY_MIN_FW_VERSION
        if self._verify:
            self._begin_aux(DIAG0, "diag", DIAG_BASE, 0, DIAG_WORDS, now)    # the counters' baseline
            return
        self._begin_region_anchors(now)

    def _begin_region_anchors(self, now: float) -> None:
        h = self.header
        self.phase = ANCHORS
        total_anchor_words = ANCHOR_WORDS * h.n_anchors
        self.anchor_region = self.region = _Region("anchors", ANCHOR_BASE, 0, total_anchor_words,
                                                         self.cfg.refill_merge_words)
        self.frames_total = total_anchor_words + 2 * h.n_samples
        self.frames_done = 0
        if not self._start_block(now):
            self._anchors_done(now)

    def _anchors_done(self, now: float) -> None:
        h = self.header
        self.anchors = decode_anchors(bytes(self.anchor_region.buf))
        k0, k1 = 0, h.n_samples
        job = self.job
        self.full_coverage = True
        if job.t_start_wall is not None and job.t_end_wall is not None and h.n_samples:
            model = TimeModel(h, self.anchors)
            rng = model.select_range((job.t_start_wall - self.cfg.span_margin_s) * 1e6,
                                     (job.t_end_wall + self.cfg.span_margin_s) * 1e6)
            if rng is None:
                self.flags["span_unavailable"] = True      # no wall clock in this capture: take it all
            else:
                k0, k1 = rng
                if rng == (0, 0):
                    self.flags["span_empty"] = True        # no sample inside the span: clocks disagree
                elif k0 == 0:
                    # The ring did not reach back to the span's start (it holds RING_SAMPLES / odr_hz seconds:
                    # 32 s at 1 kHz, 16 s at 2 kHz): the capture is the whole ring, the span's start is missing.
                    w_first = float(model.wall_us(np.arange(1))[0])
                    if w_first > (job.t_start_wall - self.cfg.span_margin_s) * 1e6 + 1e6 / float(h.odr_hz or 1000):
                        self.flags["span_truncated"] = True
                if self._verify and rng != (0, 0):
                    # FW 14: widen the span to whole chunks (CHUNK_WORDS / 2 samples) so that every word read is
                    # inside a chunk the board's table covers in full: an unaligned edge is verified exactly by
                    # reading the rest of its chunk (at most 255 samples each side), never left unchecked.
                    cs = CHUNK_WORDS // 2
                    k0 = (k0 // cs) * cs
                    k1 = min(-(-k1 // cs) * cs, h.n_samples)
                self.full_coverage = (k0, k1) == (0, h.n_samples)
                self.flags["partial_span"] = not self.full_coverage
        self.k0, self.k1 = k0, k1
        self.phase = SAMPLES
        self.sample_region = self.region = _Region("samples", 0, 2 * k0, 2 * k1, self.cfg.refill_merge_words)
        self.frames_total = ANCHOR_WORDS * h.n_anchors + 2 * (k1 - k0)
        self.frames_done = ANCHOR_WORDS * h.n_anchors
        if not self._start_block(now):
            self._samples_done(now)

    def _samples_done(self, now: float) -> None:
        if (self._verify and self._chunk_state == "todo" and self.sample_region.complete
                and self.anchor_region.complete):
            self._begin_chunks(now)                    # FW 14: verify by chunks first (it localises a mismatch)
            return
        self._payload_and_finish(now)

    def _payload_and_finish(self, now: float) -> None:
        h = self.header
        if self.full_coverage and self.sample_region.complete and self.anchor_region.complete:
            ok = (zlib.crc32(bytes(self.sample_region.buf), zlib.crc32(bytes(self.anchor_region.buf)))
                  & 0xFFFFFFFF) == h.payload_crc32
            if not ok and self.rereads_left > 0:
                self._stash_pass1()
                self.rereads_left -= 1
                self.stats["crc_rereads"] += 1
                logger.warning("IMU payload CRC mismatch: re-reading the capture once")
                self.phase = ANCHORS
                self.anchor_region = self.region = _Region("anchors", ANCHOR_BASE, 0,
                                                             ANCHOR_WORDS * h.n_anchors, self.cfg.refill_merge_words)
                self.frames_done = 0
                self.stalled = 0
                if self._chunk_state == "done":
                    self._chunk_state = "todo"         # re-verify the re-read words (the table is kept)
                    self._chunk_rereads_left = self.cfg.max_chunk_rereads
                if not self._start_block(now):
                    self._anchors_done(now)
                return
            self._payload_ok = ok
        checks = [(m, v) for m, v in (("chunks", self._chunks_ok), ("payload", self._payload_ok)) if v is not None]
        if checks:
            self.flags["crc_checked"] = True
            self.flags["crc_ok"] = all(v for _, v in checks)
            self.flags["crc_method"] = "+".join(m for m, _ in checks)
        self.flags["complete"] = (self.sample_region.complete and self.flags["crc_ok"] is not False
                                  and not self.flags["aborted"] and not self.flags["platform_rebooted"])
        if self._verify:
            self._begin_aux(DIAG1, "diag", DIAG_BASE, 0, DIAG_WORDS, now)    # the counters after the data
            return
        self._resume(now)

    def _stash_pass1(self) -> None:
        """Record the samples region's first-sweep losses in ``stats`` once (the first region wins: a CRC
        re-read builds a new one). ``pass1_missing`` = words still missing when the first sweep ended (0 when
        it ended complete, None when the upload ended inside it); ``pass1_missing_seqs`` = every one of
        them, so a capture shows whether lost frames cluster."""
        sr = self.sample_region
        if sr is None or "pass1_missing" in self.stats:
            return
        if sr.pass1 is not None:
            self.stats["pass1_missing"], self.stats["pass1_missing_seqs"] = sr.pass1[0], list(sr.pass1[1])
        elif sr.complete:
            self.stats["pass1_missing"], self.stats["pass1_missing_seqs"] = 0, []
        else:
            self.stats["pass1_missing"], self.stats["pass1_missing_seqs"] = None, []

    def _fail(self, why: str, now: float, *, no_resume: str = "") -> None:
        """THE early exit. Invariant: the ring is never left frozen by us. Every early end RESUMEs the
        capture we froze, except where ``no_resume`` names why it is provably unnecessary (nothing frozen,
        or the Platform rebooted / discarded it) or where the id is unknown (FREEZE unanswered: the
        service heals from STATUS). The RESUME itself is retried and, if lost, healed the same way."""
        self.failure = self.failure or why
        self.flags["complete"] = False
        logger.warning("IMU upload ended early: %s", why)
        self.blk = self.ahead = None
        if no_resume:
            self.flags["resume_skipped"] = no_resume
            self._finish(now)
        elif self.cid:
            self._resume(now)
        else:
            self.flags["resume_skipped"] = "no capture id"
            self._finish(now)

    def _resume(self, now: float) -> None:
        self.blk = self.ahead = None
        self.phase = RESUMING
        self._cmd = None
        self._send_cmd(OP.RESUME, now)

    def _finish(self, now: float) -> None:
        self.phase = DONE
        self._stash_pass1()
        if self._bridge0 is not None and self._bridge1 is not None:
            # IMU_DATA frames the can-bridge received between this upload's first and last batch, minus those the
            # Jetson received in the batches after the first: 0 when nothing is lost between the bridge's ring and
            # the machine (findings: every Armed loss is upstream of the bridge software). It can differ by the
            # frames still in the bridge ring when the first or last batch was built (normally none at the ends).
            d = (self._bridge1[0] - self._bridge0[0]) % (1 << 32)
            self.stats["bridge_rx_frames"] = d
            self.stats["bridge_rx_minus_jetson_rx"] = d - self._jet_rx
        sr = self.sample_region
        res = UploadResult(
            header=self.header, anchors=list(self.anchors),
            sample_words=bytes(sr.buf) if sr else b"", word_present=bytes(sr.have) if sr else b"",
            k0=self.k0, flags=dict(self.flags), stats=dict(self.stats, wall_s=now - self.t_job0),
            job=self.job, failure=self.failure, frozen_cid=self.cid,
            freeze_lost=bool(self.flags.get("freeze_lost")), resume_lost=bool(self.flags.get("resume_lost")))
        res.flags["holes_unfilled"] = sr.missing() if sr else 0
        if sr is not None and sr.n:
            res.flags["complete"] = bool(res.flags["complete"]) and sr.complete
        if self.flags.get("span_empty"):
            res.flags["complete"] = False              # zero samples matched the span: not a capture of it
        try:
            self._on_result(res)
        except Exception:  # noqa: BLE001
            logger.exception("IMU upload result handler failed")


def format_upload_end(summary: Dict[str, Any]) -> Tuple[bool, str]:
    """The one launch-log line for an ended upload: ``(ok, text)``. ``ok`` False (log a WARNING) when the
    upload failed, was aborted, is incomplete, failed its CRC or its capture was not handed to the writer."""
    st = summary.get("stats") or {}
    fl = summary.get("flags") or {}
    crc = fl.get("crc_ok")
    crc_s = "ok" if crc is True else ("FAILED" if crc is False else "not checked")
    if crc is not None and "chunks" in (fl.get("crc_method") or ""):
        crc_s += " (%s)" % fl["crc_method"]             # FW 14: a span verified by the chunk-CRC table
    complete = bool(fl.get("complete")) and not summary.get("write_failed")
    body = ("%s, %d samples, %.1f s, %d frames rx, %d duplicate, %d READs (%d chained, %d replaced), "
            "%d block timeouts, %d hole passes, refill %.1f s, complete %s, crc %s, file %s" % (
                summary.get("label", "?"), int(summary.get("samples") or 0), float(st.get("wall_s") or 0.0),
                int(st.get("frames_rx", 0)), int(st.get("dup_frames", 0)), int(st.get("blocks", 0)),
                int(st.get("reads_chained", 0)), int(st.get("reads_replaced", 0)), int(st.get("block_timeouts", 0)),
                int(st.get("hole_passes", 0)), float(st.get("refill_s") or 0.0), "yes" if complete else "NO",
                crc_s, summary.get("stem") or "none"))
    if summary.get("failure") or fl.get("aborted") or not complete or crc is False:
        reason = summary.get("failure") or ("aborted" if fl.get("aborted") else (
            "CRC mismatch" if crc is False else "incomplete (%d word(s) missing%s)" % (
                int(fl.get("holes_unfilled") or 0), ", empty span" if fl.get("span_empty") else "")))
        word = "aborted" if fl.get("aborted") else "failed"
        return False, "Platform IMU upload %s: %s (%s)" % (word, reason, body)
    return True, "Platform IMU upload finished (%s)" % body


# ── the worker service ───────────────────────────────────────────────────────────────────────────

#: The can-bridge's refusal of SET_ODR, in the operator's words (spec section 6). Can-bridge FW 47 relays
#: SET_ODR and never refuses it for its value (the Platform judges that), so a bridge ERR_BAD_ARGS here means
#: a board older than FW 47 is aboard: FW 46 and earlier whitelist only ENABLE/DISABLE/FREEZE/RESUME/ABORT,
#: answer ERR_BAD_ARGS and send nothing to the Platform.
SET_ODR_BRIDGE_REFUSAL = (
    "refused by the can-bridge (ERR_BAD_ARGS, nothing reached the Platform): the bridge firmware aboard is "
    "older than FW 47 and does not relay the rate command; flash can-bridge FW 47 (the launch log's "
    "BRIDGE_FW_CHECK line names the version aboard). The alternative without a bridge flash is the 2 kHz "
    "Platform firmware image: pio run -e teensy40_imu2k -t upload in Teensy_code_platform (boots at "
    "2 kHz; flash teensy40 again to return to 1 kHz)")


class OdrRequest:
    """One SET_ODR request handed to the worker: the caller waits on :meth:`wait`, the worker calls
    :meth:`finish` once (bridge refusal, the Platform's IMU_REPLY, or no reply in time)."""

    def __init__(self, hz: int):
        self.hz = int(hz)
        self.ok = False
        self.message = ""
        self.deadline = 0.0
        self._done = threading.Event()

    def finish(self, ok: bool, message: str) -> None:
        if not self._done.is_set():
            self.ok, self.message = bool(ok), message
            self._done.set()

    @property
    def done(self) -> bool:
        return self._done.is_set()

    def wait(self, timeout: float) -> bool:
        return self._done.wait(timeout)


def set_odr_reply_text(rep: ImuReply, hz: int) -> Tuple[bool, str]:
    """The Platform's verdict on SET_ODR, in plain words."""
    st = rep.status
    if st == ST.OK:
        got = int(rep.detail) or hz
        return True, ("capture rate %d Hz: the ring restarts as a fresh capture and holds %.0f s"
                      % (got, RING_SAMPLES / float(got)))
    if st == ST.ERR_FROZEN:
        return False, ("refused by the Platform (ERR_FROZEN): a capture is frozen or uploading; upload it "
                       "(platform_imu/upload) or let it be resumed first, so a rate change never discards it")
    if st == ST.ERR_BAD_ARGS:
        return False, ("refused by the Platform (ERR_BAD_ARGS): its firmware does not know SET_ODR (FW 12 or "
                       "older), or this build cannot run %d Hz (the 400 kHz I2C build is 1 kHz only)" % hz)
    if st == ST.ERR_FW_UPDATE:
        return False, "refused by the Platform (ERR_FW_UPDATE): a firmware update is in progress"
    name = ST(st).name if st in set(int(x) for x in ST) else "0x%02X" % st
    return False, "refused by the Platform (%s)" % name

class PlatformImuService:
    """One worker thread owns an :class:`UploadMachine`. Producers (the UDP receive callback, the
    ROS services) only enqueue; they never block, never raise and do bounded work. All RPCs the
    machine sends run on the worker thread, so no ROS callback group is occupied by an upload.

    Counters ``dropped_batches`` / ``dropped_events`` count queue overflows (a hole the machine
    recovers by re-reading)."""

    def __init__(self, send_ctrl, send_read, *, writer=None, clock: Callable[[], float] = time.monotonic,
                 wall: Callable[[], float] = time.time, is_armed: Callable[[], bool] = lambda: False,
                 config: Optional[UploadConfig] = None, start_thread: bool = True,
                 data_queue_size: int = 2048, event_queue_size: int = 256,
                 on_state_change: Optional[Callable[[], None]] = None,
                 on_upload_end: Optional[Callable[[Dict[str, Any]], None]] = None):
        self._clock, self._wall = clock, wall
        self._on_state_change = on_state_change        # busy flipped (accepted / ended): publish at once
        self._on_upload_end = on_upload_end            # once per upload (not a heal), with the summary dict
        self.cfg = config or UploadConfig()
        self._writer = writer
        self._send_ctrl = send_ctrl
        self._odr_pending: Optional[OdrRequest] = None   # worker thread only
        self.machine = UploadMachine(send_ctrl, send_read, self._on_result, config=self.cfg, is_armed=is_armed,
                                     ack_clock=clock)
        self._data_q: "queue.Queue" = queue.Queue(maxsize=data_queue_size)
        self._ev_q: "queue.Queue" = queue.Queue(maxsize=event_queue_size)
        self.dropped_batches = 0
        self.dropped_events = 0
        self._lock = threading.Lock()                  # guards the snapshot fields below
        self._busy = False
        self._upload_after_attempt = True
        self._last_status: Optional[ImuStatus] = None
        self._last_status_t: Optional[float] = None
        self._bridge: Dict[str, Any] = {}
        self._last_result: Dict[str, Any] = {}
        self._last_skip = ""
        self._boot_seen: Optional[int] = None
        # Jetson-side running totals of the board's u8 health counters (saturate at 255 on the board).
        self._health_prev: Optional[Tuple[int, int]] = None      # (i2c_errors, fifo_overflows) at the last STATUS
        self._health_totals = {"i2c_errors": 0, "fifo_overflows": 0}
        self._health_last_inc_t: Dict[str, Optional[float]] = {"i2c_errors": None, "fifo_overflows": None}
        # Healing a ring left frozen (worker thread only): _pending_cid None = nothing owed, 0 = a capture
        # froze under an id we never saw, >0 = our own capture whose RESUME was lost.
        self._pending_cid: Optional[int] = None
        self._pending_t = 0.0
        self._frozen_since: Optional[float] = None
        self._heal_tries = 0
        self._last_heal_t = -1e9
        self._heal_gave_up = False
        self.heals_started = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="platform_imu_worker", daemon=True)
        if start_thread:
            self._thread.start()

    # -- producers (any thread; O(1); never raise) --
    def offer_batch(self, payload: bytes, now: Optional[float] = None) -> bool:
        try:
            info, records = parse_batch(payload)
            self._data_q.put_nowait((self._clock() if now is None else now, info, records))
            return True
        except queue.Full:
            self.dropped_batches += 1
        except Exception:  # noqa: BLE001
            pass
        return False

    def offer_frame(self, can_id: int, data: bytes, now: Optional[float] = None) -> bool:
        """A relayed IMU_REPLY / IMU_STATUS (PLATFORM_FRAME) from the receive thread."""
        try:
            self._ev_q.put_nowait(("frame", int(can_id), bytes(data), self._clock() if now is None else now))
            return True
        except Exception:  # noqa: BLE001
            self.dropped_events += 1
            return False

    # -- control API (any thread) --
    def set_upload_after_attempt(self, on: bool) -> None:
        with self._lock:
            self._upload_after_attempt = bool(on)

    def upload_after_attempt(self) -> bool:
        with self._lock:
            return self._upload_after_attempt

    def upload_in_progress(self) -> bool:
        with self._lock:
            return self._busy

    def imu_available(self, now: Optional[float] = None) -> Tuple[bool, str]:
        """Is there a recording IMU worth a FREEZE? A board with no IMU (PROBING, SENSOR_OK clear), no
        STATUS (boot settle, or a no-IMU build), DISABLED or FW_HALTED is not an alarm: just no capture."""
        now = self._clock() if now is None else now
        with self._lock:
            st, t = self._last_status, self._last_status_t
        if st is None or t is None or now - t > self.cfg.status_stale_s:
            return False, "no IMU_STATUS (boot settle, or a no-IMU build)"
        if st.state in (STATE.RUNNING, STATE.FROZEN, STATE.UPLOADING):
            return True, ""
        return False, "IMU state %s%s" % (STATE(st.state).name if st.state in set(int(s) for s in STATE) else st.state,
                                          "" if st.sensor_ok else " (no sensor)")

    def odr_hz(self) -> int:
        """The selected capture rate from the last IMU_STATUS (1 kHz until one is heard)."""
        with self._lock:
            st = self._last_status
        return st.odr_hz if st is not None else NOMINAL_ODR_HZ

    def ring_s(self) -> float:
        """Seconds the ring holds at the selected rate: the cap on an Attempt span (32 s at 1 kHz, 16 s at 2)."""
        return RING_SAMPLES / float(self.odr_hz())

    def submit_set_odr(self, hz: int):
        """Queue a SET_ODR for the worker. Returns an :class:`OdrRequest` to wait on, or ``(False, why)``
        when refused here (bad rate, an upload running, a capture frozen). Never blocks."""
        try:
            hz = int(hz)
        except (TypeError, ValueError):
            return False, "refused: the rate must be %d or %d Hz" % (NOMINAL_ODR_HZ, FAST_ODR_HZ)
        if hz not in (NOMINAL_ODR_HZ, FAST_ODR_HZ):
            return False, "refused: the rate must be %d or %d Hz, not %d" % (NOMINAL_ODR_HZ, FAST_ODR_HZ, hz)
        if self.upload_in_progress():
            return False, "refused: an upload is running (a rate change would discard the ring)"
        with self._lock:
            st = self._last_status
        if st is not None and st.state in (STATE.FROZEN, STATE.UPLOADING):
            return False, ("refused: the ring is %s; upload it (platform_imu/upload) or let it be resumed first"
                           % STATE(st.state).name)
        req = OdrRequest(hz)
        try:
            self._ev_q.put_nowait(("set_odr", req, self._clock()))
        except queue.Full:
            self.dropped_events += 1
            return False, "refused: the IMU worker queue is full"
        return req

    def set_odr(self, hz: int, timeout: float = 2.0) -> Tuple[bool, str]:
        """SET_ODR and wait for the verdict (blocks the CALLER up to ``timeout``; the worker sends the RPC)."""
        req = self.submit_set_odr(hz)
        if isinstance(req, tuple):
            return req
        if not req.wait(timeout):
            return False, "no verdict within %.1f s (the IMU worker is not running?)" % timeout
        return req.ok, req.message

    def _start_set_odr(self, req: OdrRequest, now: float) -> None:
        """Worker thread: send SET_ODR (one at a time, never during an upload) and arm its reply deadline."""
        if self.machine.busy:
            req.finish(False, "refused: an upload is running (a rate change would discard the ring)")
            return
        if self._odr_pending is not None:
            req.finish(False, "refused: another rate change is still waiting for its reply")
            return
        ok, msg = self._send_ctrl(int(OP.SET_ODR), req.hz // 1000, 0)
        if not ok:
            req.finish(False, SET_ODR_BRIDGE_REFUSAL if "ERR_BAD_ARGS" in (msg or "")
                       else "rate command not sent: %s" % msg)
            return
        req.deadline = now + 2 * self.cfg.cmd_timeout_s
        self._odr_pending = req

    def _odr_check(self, now: float, rep: Optional[ImuReply] = None) -> None:
        req = self._odr_pending
        if req is None:
            return
        if rep is not None:
            if rep.status == ST.OK and int(rep.detail) not in (0, req.hz):
                return                     # a late OK to an earlier request: keep waiting for ours
            ok, text = set_odr_reply_text(rep, req.hz)
            self._odr_pending = None
            if ok:
                logger.info("Platform IMU %s", text)
            req.finish(ok, text)
        elif now >= req.deadline:
            self._odr_pending = None
            req.finish(False, "no reply from the Platform within %.1f s (the rate may or may not have changed: "
                              "platform_imu/status odr_hz shows it at the next IMU_STATUS)" % (2 * self.cfg.cmd_timeout_s))

    def request_upload(self, t_start_wall: Optional[float] = None, t_end_wall: Optional[float] = None,
                       reason: int = int(pc.PlatformImuFreezeReason.OPERATOR), attempt_id: Optional[str] = None,
                       label: str = "operator", force: bool = False, read_delay_s: float = 0.0) -> bool:
        """Freeze now and upload the span (wall seconds; None = everything). False if one is already
        running or (unless ``force``) there is no IMU to freeze. Never blocks."""
        if not force:
            ok, why = self.imu_available()
            if not ok:
                with self._lock:
                    self._last_skip = why
                logger.info("IMU upload skipped: %s", why)
                return False
        with self._lock:
            if self._busy:
                return False
            self._busy = True
        job = UploadJob(t_start_wall, t_end_wall, int(reason), attempt_id, label, read_delay_s=float(read_delay_s))
        try:
            self._ev_q.put_nowait(("start", job, self._clock()))
        except queue.Full:
            with self._lock:
                self._busy = False
            self.dropped_events += 1
            return False
        self._notify()
        return True

    def _notify(self) -> None:
        cb = self._on_state_change
        if cb is not None:
            try:
                cb()
            except Exception:  # noqa: BLE001 -- a status publisher never breaks the worker
                logger.exception("platform IMU state-change callback failed")

    def attempt_ended(self, t_start_wall: float, t_end_wall: float, reason: str = "attempt_end",
                      attempt_id: Optional[str] = None) -> bool:
        """The Attempt is over: freeze the ring and upload its span plus margin, if the toggle is ON."""
        if not self.upload_after_attempt():
            return False
        rmap = {"attempt_end": pc.PlatformImuFreezeReason.ATTEMPT_END, "fault": pc.PlatformImuFreezeReason.FAULT,
                "soft_estop": pc.PlatformImuFreezeReason.SOFT_ESTOP}
        # A Fault / Soft E-stop freezes now (that is what keeps the data) but starts the READs later: the
        # robot is still deactivating and its recovery RPCs share the bus (audit B7).
        delay = self.cfg.fault_upload_delay_s if reason in ("fault", "soft_estop") else 0.0
        return self.request_upload(t_start_wall, t_end_wall, int(rmap.get(reason, pc.PlatformImuFreezeReason.ATTEMPT_END)),
                                   attempt_id, label=reason, read_delay_s=delay)

    def abort_upload(self, reason: str = "abort") -> None:
        """Stop an upload (pacifying command, Attempt start). Returns at once; the worker sends ABORT
        first, writes what arrived (complete = false) and RESUMEs."""
        try:
            self._ev_q.put_nowait(("abort", reason, self._clock()))
        except queue.Full:
            self.dropped_events += 1

    def _track_health(self, st: ImuStatus, t: float, boot_before: Optional[int]) -> None:
        """Accumulate positive counter deltas (caller holds the lock). The first STATUS only sets the baseline
        (errors from before this launch are unknown); a boot-byte change or a decrease means the board counted
        from zero again, so the new value is all new errors, but a reboot never arms the 60 s WARN (a boot's
        PROBING errors are not an alarm). A counter at 255 is saturated: further errors are invisible."""
        cur = (st.i2c_errors, st.fifo_overflows)
        prev = self._health_prev
        self._health_prev = cur
        if prev is None:
            return
        rebooted = boot_before is not None and st.boot != boot_before
        for name, p, c in (("i2c_errors", prev[0], cur[0]), ("fifo_overflows", prev[1], cur[1])):
            if rebooted or c < p:
                self._health_totals[name] += c
            elif c > p:
                self._health_totals[name] += c - p
                self._health_last_inc_t[name] = t

    def status_snapshot(self) -> Dict[str, Any]:
        now = self._clock()
        with self._lock:
            st, t = self._last_status, self._last_status_t
            snap: Dict[str, Any] = {
                "upload_after_attempt": self._upload_after_attempt, "busy": self._busy,
                "platform_state": (STATE(st.state).name if st and st.state in set(int(s) for s in STATE) else None),
                "platform_flags": st.flags if st else None, "platform_boot": st.boot if st else None,
                "platform_temp_c": st.temp_c if st else None,
                "odr_hz": st.odr_hz if st else None,
                "health": self._health_snapshot(st, now),
                "status_age_s": (now - t) if t is not None else None,
                "bridge": dict(self._bridge), "last_result": dict(self._last_result), "last_skip": self._last_skip,
                "dropped_batches": self.dropped_batches, "dropped_events": self.dropped_events,
                "heals_started": self.heals_started,
            }
            w = self._writer
            wr = {"written": int(getattr(w, "written", 0)), "dropped": int(getattr(w, "dropped", 0)),
                  "errors": int(getattr(w, "errors", 0)), "last_error": str(getattr(w, "last_error", "") or "")}
            lr = snap["last_result"]
            if lr.get("path") == "queued" and (wr["errors"] > lr.get("writer_errors_at_submit", 0)
                                               or wr["dropped"] > lr.get("writer_dropped_at_submit", 0)):
                # the writer thread failed after the machine reported success: the operator must see it
                lr["write_failed"] = True
                lr["failure"] = (lr.get("failure") + "; " if lr.get("failure") else "") + \
                    "capture write failed: " + (wr["last_error"] or "dropped")
            snap["writer"] = wr
        snap["progress"] = self.machine.progress()
        return snap

    def _health_snapshot(self, st: Optional[ImuStatus], now: float) -> Optional[Dict[str, Any]]:
        """The last STATUS's health counters plus Jetson-side totals (caller holds the lock)."""
        if st is None:
            return None
        F = pc.PlatformImuStatusFlag
        sp = st.sync_peak
        inc = self._health_last_inc_t
        return {
            "i2c_errors": st.i2c_errors, "fifo_overflows": st.fifo_overflows,
            "i2c_errors_saturated": st.i2c_errors >= 255, "fifo_overflows_saturated": st.fifo_overflows >= 255,
            "i2c_errors_delta_since_launch": self._health_totals["i2c_errors"],
            "fifo_overflows_delta_since_launch": self._health_totals["fifo_overflows"],
            "i2c_errors_increase_age_s": None if inc["i2c_errors"] is None else now - inc["i2c_errors"],
            "fifo_overflows_increase_age_s": None if inc["fifo_overflows"] is None else now - inc["fifo_overflows"],
            "temp_c": None if st.temp_c == -128 else st.temp_c,
            "sync_peak": sp, "sync_peak_valid": sp != 255, "sync_peak_saturated": sp == 254,
            "sync_peak_ms": None if sp == 255 else sp * 0.01,        # 10 us per count; 254 is a lower bound
            "sensor_ok": bool(st.flags & F.SENSOR_OK), "time_synced": bool(st.flags & F.TIME_SYNCED),
            "ring_wrapped": bool(st.flags & F.RING_WRAPPED), "i2c_400k": bool(st.flags & F.I2C_400K),
            "discontinuity": bool(st.flags & F.DISCONTINUITY),
        }

    # -- worker --
    def pump(self, now: Optional[float] = None) -> None:
        """Process everything queued, then tick (the thread's loop body; tests call it directly)."""
        now = self._clock() if now is None else now
        while True:
            try:
                ev = self._ev_q.get_nowait()
            except queue.Empty:
                break
            self._handle_event(ev)
        for _ in range(64):
            try:
                t, info, records = self._data_q.get_nowait()
            except queue.Empty:
                break
            self._bridge = {"ring_hwm": info.ring_hwm, "ring_drops": info.ring_drops, "rx_frames": info.rx_frames,
                            "batches_sent": info.batches_sent, "send_fails": info.send_fails}
            self.machine.note_bridge(info.rx_frames, info.batches_sent, len(records) // 8)
            self.machine.on_records(records, t)
        self.machine.tick(now)
        self._odr_check(now)
        self._heal_check(now)

    def _heal_check(self, now: float) -> None:
        """Audit B3: never leave the ring frozen. A STATUS that shows FROZEN/UPLOADING while no upload is
        running means a RESUME (or the FREEZE reply) was lost, or the capture was frozen by a process that
        is gone. After a grace, RESUME it: our own capture by its id; otherwise FREEZE (a probe: the board
        answers ALREADY_FROZEN with the frozen capture's id, which STATUS and the header do not carry in
        a way the spec lets us read before a FREEZE) and RESUME that id. Bounded, logged."""
        with self._lock:
            st, t, busy = self._last_status, self._last_status_t, self._busy
        fresh = st is not None and t is not None and now - t <= self.cfg.status_stale_s
        frozen = fresh and st.state in (STATE.FROZEN, STATE.UPLOADING)
        if not fresh:
            return
        if not frozen:
            self._frozen_since, self._heal_tries, self._heal_gave_up = None, 0, False
            if self._pending_cid is not None and t > self._pending_t:
                self._pending_cid = None
            return
        if busy or self.machine.busy:
            self._frozen_since = None              # the grace restarts when the job ends
            return
        if self._frozen_since is None:
            self._frozen_since = now
            return
        known = bool(self._pending_cid)
        grace = self.cfg.heal_grace_s if known else self.cfg.heal_foreign_grace_s
        if now - self._frozen_since < grace or now - self._last_heal_t < self.cfg.heal_retry_s:
            return
        if self._heal_tries >= self.cfg.heal_max_tries:
            if not self._heal_gave_up:
                self._heal_gave_up = True
                logger.error("Platform IMU ring is still FROZEN after %d heal attempts: the recording is stopped "
                             "until the Platform is resumed or rebooted", self._heal_tries)
            return
        self._heal_tries += 1
        self._last_heal_t = now
        self.heals_started += 1
        cid = int(self._pending_cid or 0)
        logger.warning("Platform IMU ring FROZEN with no upload running for %.1f s: RESUME %s (attempt %d)",
                       now - self._frozen_since, ("0x%04X" % cid) if cid else "after a FREEZE probe", self._heal_tries)
        self._pending_cid = 0                      # a retry (if this fails) probes for the id
        self._pending_t = now
        with self._lock:
            self._busy = True
        self._notify()
        try:
            self.machine.start(UploadJob(label="heal", heal=True, heal_cid=cid, reason=int(pc.PlatformImuFreezeReason.OPERATOR)),
                               now, self._boot_seen)
        except Exception:  # noqa: BLE001
            logger.exception("platform IMU heal failed to start")
            with self._lock:
                self._busy = False

    def _handle_event(self, ev: tuple) -> None:
        kind = ev[0]
        try:
            if kind == "frame":
                _, can_id, data, t = ev
                if can_id == CAN_ID_REPLY:
                    rep = decode_imu_reply(data)
                    if rep.op == OP.SET_ODR:
                        self._odr_check(t, rep)
                    self.machine.on_reply(rep, t)
                elif can_id == CAN_ID_STATUS:
                    st = decode_imu_status(data)
                    with self._lock:
                        if self._boot_seen is not None and st.boot != self._boot_seen:
                            logger.info("Platform IMU boot byte changed %d -> %d (Platform rebooted)",
                                        self._boot_seen, st.boot)
                        self._track_health(st, t, self._boot_seen)
                        self._boot_seen = st.boot
                        self._last_status, self._last_status_t = st, t
                    self.machine.on_status(st, t)
            elif kind == "start":
                _, job, t = ev
                with self._lock:
                    boot = self._boot_seen
                self.machine.start(job, t, boot)
            elif kind == "abort":
                self.machine.abort(ev[1], ev[2])
            elif kind == "set_odr":
                self._start_set_odr(ev[1], ev[2])
        except Exception:  # noqa: BLE001 -- the worker must outlive any one bad frame
            logger.exception("platform IMU event %s failed", kind)
            if kind == "set_odr":
                ev[1].finish(False, "rate command failed on the worker (see the log)")
            if kind == "start":
                with self._lock:
                    self._busy = False

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                # Block briefly on the data queue so a batch is handled within a millisecond or two; when a
                # block's settle or the next pipelined refill READ is pending, wake when it falls due (at least
                # 1 ms, at most the 20 ms poll).
                timeout = 0.02
                now = self._clock()
                for due in (self.machine.settle_due_in(now), self.machine.pipeline_due_in(now)):
                    if due is not None:
                        timeout = min(timeout, max(due, 0.001))
                t, info, records = self._data_q.get(timeout=timeout)
                self._bridge = {"ring_hwm": info.ring_hwm, "ring_drops": info.ring_drops,
                                "rx_frames": info.rx_frames, "batches_sent": info.batches_sent,
                                "send_fails": info.send_fails}
                self.machine.note_bridge(info.rx_frames, info.batches_sent, len(records) // 8)
                self.machine.on_records(records, t)
            except queue.Empty:
                pass
            except Exception:  # noqa: BLE001
                logger.exception("platform IMU batch handling failed")
            try:
                self.pump()
            except Exception:  # noqa: BLE001
                logger.exception("platform IMU pump failed")

    def _on_result(self, res: UploadResult) -> None:
        now = self._clock()
        if res.freeze_lost:
            self._pending_cid, self._pending_t = 0, now
        elif res.resume_lost:
            self._pending_cid, self._pending_t = (res.frozen_cid or 0), now
        if res.job.heal:                            # a heal: nothing to write, never overwrite the last result
            logger.info("Platform IMU heal finished: %s", res.failure or "RESUME sent")
            with self._lock:
                self._busy = False
            self._notify()
            return
        summary: Dict[str, Any] = {"label": res.job.label, "attempt_id": res.job.attempt_id,
                                   "flags": dict(res.flags), "failure": res.failure, "stats": dict(res.stats),
                                   "capture_id": res.header.capture_id if res.header else None, "path": None,
                                   "samples": len(res.sample_words) // (2 * WORD), "stem": None}
        try:
            if res.header is not None:
                uploaded = self._wall()
                meta = {"attempt_id": res.job.attempt_id, "label": res.job.label,
                        "span_wall_s": [res.job.t_start_wall, res.job.t_end_wall],
                        "span_margin_s": self.cfg.span_margin_s, "stats": res.stats, "failure": res.failure,
                        "requested_gap_10us": self.cfg.gap_10us, "uploaded_wall_time_s": uploaded}
                flags = dict(res.flags)
                flags["discontinuities"] = sum(1 for a in res.anchors if a.flags & 0x01)
                cap = build_capture(res.header, res.anchors, res.sample_words, res.word_present, res.k0, flags, meta)
                summary["fit"] = {k: cap.fit.get(k) for k in ("rate_hz", "resid_rms_us", "n_segments", "wall_ok")}
                summary["flags"] = flags
                summary["samples"] = int(cap.raw.shape[0])
                # The writer names the file from uploaded_wall_time_s (ImuCaptureWriter), so this is its stem
                # (a same-second collision appends _1, _2 ...).
                summary["stem"] = capture_stem(cap, uploaded)
                if self._writer is not None:
                    summary["writer_errors_at_submit"] = int(getattr(self._writer, "errors", 0))
                    summary["writer_dropped_at_submit"] = int(getattr(self._writer, "dropped", 0))
                    if self._writer.submit(cap):
                        summary["path"] = "queued"
                    else:
                        summary["write_failed"] = True
                        summary["failure"] = (summary["failure"] + "; " if summary["failure"] else "") + \
                            "capture dropped (writer queue full or closed)"
            else:
                logger.warning("IMU upload produced no capture: %s", res.failure)
        except Exception:  # noqa: BLE001 -- a capture-building bug must not strand the busy flag
            logger.exception("platform IMU capture build failed")
            summary["failure"] = (summary["failure"] + "; " if summary["failure"] else "") + "capture build failed"
        finally:
            with self._lock:
                self._last_result = summary
                self._busy = False
            self._notify()
            cb = self._on_upload_end
            if cb is not None:
                try:
                    cb(dict(summary))
                except Exception:  # noqa: BLE001 -- a log line never breaks the worker
                    logger.exception("platform IMU upload-end callback failed")

    def close(self, timeout: float = 5.0) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout)
        if self._writer is not None:
            try:
                self._writer.close()
            except Exception:  # noqa: BLE001
                pass
