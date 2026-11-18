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
import queue
import struct
import threading
import time
import zlib
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple

from . import protocol as _protocol  # noqa: F401  (puts config/generated on sys.path)
import protocol_config as pc  # noqa: E402

from .platform_imu_capture import TimeModel, build_capture

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


# ── codecs ───────────────────────────────────────────────────────────────────────────────────────

class HeaderError(ValueError):
    """The 66-byte header failed its self-check (length, CRC or format version)."""


_REPLY = struct.Struct("<BBHI")
_STATUS = struct.Struct("<BBbBBBBB")
_HEADER = struct.Struct("<BBHIIIHHQIHHHBBBBhhHHHHIII")
_ANCHOR = struct.Struct("<IIQBx")
_BATCH_HEAD = struct.Struct("<QBBHIIII")           # the 28 bytes before PlatformImuBatch.frames
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


# ── configuration, job, result ───────────────────────────────────────────────────────────────────

@dataclass
class UploadConfig:
    gap_10us: int = 80                 # requested pace: 800 us, about 1250 frames/s (spec section 6, item 3)
    cmd_timeout_s: float = 0.6         # one CTRL/READ verdict (IMU_REPLY) before a resend
    freeze_timeout_s: float = 1.5      # FREEZE also computes the payload CRC on the board
    cmd_retries: int = 3
    block_slack_s: float = 0.15        # spec says 50 ms; +100 ms for the bridge's 10 ms flush and UDP
    settle_s: float = 0.03             # after a block's LAST seq arrives, wait this long for stragglers
    max_stalled_blocks: int = 4        # consecutive blocks that delivered nothing new
    max_passes: int = 8                # full sweeps of a region (pass 1 + hole refills)
    max_crc_rereads: int = 1           # spec: one full re-read, then write flagged
    status_grace_s: float = 2.5        # a STATUS this soon after FREEZE/RESUME may still show the old state
    span_margin_s: float = 2.0         # added each side of an Attempt span
    max_total_s: float = 300.0         # a whole upload is abandoned after this
    status_stale_s: float = 3.0        # no STATUS this long on a booted board: no IMU (spec section 4)


@dataclass
class UploadJob:
    t_start_wall: Optional[float] = None      # seconds; None = upload the whole frozen capture
    t_end_wall: Optional[float] = None
    reason: int = int(pc.PlatformImuFreezeReason.ATTEMPT_END)
    attempt_id: Optional[str] = None
    label: str = "attempt_end"


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


class _Region:
    """One address region being fetched: a word buffer and a presence bitmap. ``next_block`` always
    returns the first missing run from the scan position (at most READ_MAX words, trailing words
    already present trimmed), so the first sweep, the hole refills and late tails from a replaced
    block all use one loss-recovery path (spec section 3)."""

    def __init__(self, name: str, base: int, w_lo: int, w_hi: int):
        self.name, self.base, self.lo, self.hi = name, base, w_lo, w_hi
        n = max(w_hi - w_lo, 0)
        self.n = n
        self.buf = bytearray(WORD * n)
        self.have = bytearray(n)
        self.n_have = 0
        self.scan = 0
        self.passes = 1

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
            self.passes += 1
            self.scan = 0
            i = self.have.find(b"\x00", 0)
        j = min(i + READ_MAX, self.n)
        end = self.have.rfind(b"\x00", i, j) + 1
        self.scan = end
        return self.base + self.lo + i, end - i

    def missing(self) -> int:
        return self.n - self.n_have


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


# Phases
IDLE, FREEZING, HEADER, ANCHORS, SAMPLES, RESUMING, DONE = (
    "idle", "freezing", "header", "anchors", "samples", "resuming", "done")


class UploadMachine:
    """Single-owner upload state machine. Every entry point takes ``now`` (monotonic seconds) and
    nothing else reads a clock, so tests drive it with a fake Platform and fake time.

    ``send_ctrl(op, reason, capture_id) -> (ok, msg)`` and ``send_read(capture_id, first_seq, count,
    gap_10us) -> (ok, msg)`` queue one RPC. OK means queued on the bus, not accepted: the verdict is
    the IMU_REPLY the caller feeds back through :meth:`on_reply`.
    """

    def __init__(self, send_ctrl: Callable[..., Tuple[bool, str]], send_read: Callable[..., Tuple[bool, str]],
                 on_result: Callable[[UploadResult], None], *, config: Optional[UploadConfig] = None,
                 is_armed: Callable[[], bool] = lambda: False):
        self._ctrl, self._read, self._on_result = send_ctrl, send_read, on_result
        self.cfg = config or UploadConfig()
        self._is_armed = is_armed
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
        self._resume_after = True
        self.expected_n: Optional[int] = None
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
        self.stats = {"blocks": 0, "block_timeouts": 0, "frames_rx": 0, "dup_frames": 0, "stray_frames": 0,
                      "hole_passes": 0, "crc_rereads": 0, "block_gap_10us": 0}
        self.flags = {"complete": False, "crc_checked": False, "crc_ok": None, "aborted": False,
                      "platform_rebooted": False, "already_frozen": False, "partial_span": False,
                      "span_unavailable": False, "resume_ok": None, "holes_unfilled": 0}
        self.phase = FREEZING
        self._send_cmd(OP.FREEZE, now, reason=job.reason)

    def abort(self, reason: str, now: float) -> None:
        """Pacifying command or operator abort: stop the transfer, write what arrived (complete =
        false), and RESUME so the ring records again."""
        if not self.busy:
            return
        self.flags["aborted"] = True
        if self.phase in (HEADER, ANCHORS, SAMPLES):
            self._ctrl(int(OP.ABORT), 0, 0)         # UPLOADING -> FROZEN: the board stops sending
            self._fail("aborted: " + reason, now, resume=True)
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
        nb = r.next_block() if r else None
        if nb is None:
            return False
        first, count = nb
        gap = self.effective_gap_10us()
        deadline = now + count * gap * 10e-6 + self.cfg.block_slack_s
        self.blk = _Block(first, count, now, deadline, gap, last_rx=now)
        self.stats["blocks"] += 1
        self.stats["block_gap_10us"] = gap
        if r.passes > 1:
            self.stats["hole_passes"] = max(self.stats["hole_passes"], r.passes - 1)
        ok, msg = self._read(self.cid, first, count, gap)
        if not ok:
            logger.warning("IMU READ not queued: %s", msg)
        return True

    def on_reply(self, rep: ImuReply, now: float) -> None:
        if not self.busy:
            return
        if self.phase in (HEADER, ANCHORS, SAMPLES):
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
                self._fail("aborted: " + self._abort_reason, now, resume=True)
                return
            self._begin_region_header(now)
            return
        # Refused with nothing changed: no capture to fetch and nothing to RESUME.
        names = {ST.ERR_EMPTY: "no samples", ST.ERR_DISABLED: "IMU disabled", ST.ERR_FW_UPDATE: "firmware update"}
        self._fail("FREEZE refused: %s" % names.get(st, "status 0x%02X" % st), now, resume=False)

    def _on_read_reply(self, rep: ImuReply, now: float) -> None:
        b = self.blk
        if b is None:
            return
        if rep.status == ST.OK:
            b.replied = True
        elif rep.status in (ST.ERR_NOT_FROZEN, ST.ERR_WRONG_CAPTURE):
            self.flags["platform_rebooted"] = True
            self._fail("READ refused (%s): the Platform rebooted or was resumed" % ST(rep.status).name, now,
                       resume=False)
        elif rep.status == ST.ERR_OUT_OF_RANGE:
            self._fail("READ out of range (seq 0x%04X, region end %d)" % (rep.read_first_seq, rep.read_count),
                       now, resume=True)
        elif rep.status in (ST.ERR_BAD_ARGS, ST.ERR_DISABLED, ST.ERR_FW_UPDATE):
            self._fail("READ refused (%s)" % ST(rep.status).name, now, resume=False)

    def on_status(self, st: ImuStatus, now: float) -> None:
        """A 1 Hz IMU_STATUS. During a transfer it is the reboot detector."""
        if not self.busy:
            return
        if self.boot_low is None:
            self.boot_low = st.boot
        if self.phase not in (HEADER, ANCHORS, SAMPLES):
            return
        if self.header is not None and st.boot != (self.header.boot_id & 0xFF):
            self.flags["platform_rebooted"] = True
            self._fail("boot changed mid-upload (status boot %d, header boot_id low byte %d)"
                       % (st.boot, self.header.boot_id & 0xFF), now, resume=False)
        elif now - self.t_freeze_reply >= self.cfg.status_grace_s and st.state not in (STATE.FROZEN, STATE.UPLOADING):
            self.flags["platform_rebooted"] = True
            self._fail("Platform left FROZEN/UPLOADING mid-upload (state %d)" % st.state, now, resume=False)

    # -- data --
    def on_records(self, records: bytes, now: float) -> None:
        """Feed ``[seq u16][6 B]`` records (8 bytes each) from batches."""
        if self.phase not in (HEADER, ANCHORS, SAMPLES):
            if records:
                self.stats["stray_frames"] = self.stats.get("stray_frames", 0) + len(records) // 8
            return
        for o in range(0, len(records) - 7, 8):
            seq = records[o] | (records[o + 1] << 8)
            word = records[o + 2:o + 8]
            self.stats["frames_rx"] += 1
            if self.region is not None:
                r = self.region.put(seq, word)     # a late tail of a finished region lands outside: stray
                if r == 1:
                    self.frames_done += 1
                    if self.blk is not None:
                        self.blk.n_new += 1
                        self.blk.last_rx = now
                        if seq == self.blk.first_seq + self.blk.count - 1:
                            self.blk.last_seen = True
                elif r == 0:
                    self.stats["dup_frames"] += 1
                    if self.blk is not None:
                        self.blk.last_rx = now
                else:
                    self.stats["stray_frames"] += 1
        self._advance(now)

    # -- the clock --
    def tick(self, now: float) -> None:
        if not self.busy:
            return
        if now - self.t_job0 > self.cfg.max_total_s and self.phase in (HEADER, ANCHORS, SAMPLES):
            self._fail("upload exceeded %.0f s" % self.cfg.max_total_s, now, resume=True)
            return
        if self.phase in (FREEZING, RESUMING):
            cmd = self._cmd
            if cmd and now >= cmd["deadline"]:
                if cmd["tries"] >= self.cfg.cmd_retries:
                    if self.phase == FREEZING:
                        self._fail("no reply to FREEZE", now, resume=False)
                    else:
                        self.flags["resume_ok"] = False
                        self._fail("no reply to RESUME (the ring may still be frozen)", now, resume=False)
                else:
                    self._send_cmd(cmd["op"], now, reason=cmd["reason"])
            return
        self._rate_update(now)
        b = self.blk
        if b is None:
            return
        quiet = now - max(b.t_sent, b.last_rx)
        if b.last_seen and now - b.last_rx >= self.cfg.settle_s:
            self._next_block(now, timed_out=False)
        elif now >= b.deadline and quiet >= self.cfg.block_slack_s:
            self.stats["block_timeouts"] += 1
            self._next_block(now, timed_out=True)

    def _rate_update(self, now: float) -> None:
        if now - self._rate_t >= 1.0:
            if self._rate_t:
                self._rate = (self.frames_done - self._rate_n) / (now - self._rate_t)
            self._rate_t, self._rate_n = now, self.frames_done

    def _advance(self, now: float) -> None:
        b = self.blk
        if b is None or self.region is None:
            return
        if self.region.complete:
            self.blk = None
            self._region_done(now)
        else:
            lo = (b.first_seq - self.region.base - self.region.lo)
            if all(self.region.have[lo:lo + b.count]):
                self._next_block(now, timed_out=False)

    def _next_block(self, now: float, timed_out: bool) -> None:
        b = self.blk
        if b is not None:
            self.stalled = 0 if b.n_new else self.stalled + 1
        if self.stalled > self.cfg.max_stalled_blocks:
            self._fail("upload stalled: %d consecutive blocks delivered nothing" % self.stalled, now, resume=True)
            return
        if self.region.complete:
            self.blk = None
            self._region_done(now)
            return
        if self.region.passes > self.cfg.max_passes:
            self.flags["holes_unfilled"] = self.region.missing()
            self._fail("%d word(s) of the %s region still missing after %d passes"
                       % (self.region.missing(), self.region.name, self.cfg.max_passes), now, resume=True)
            return
        self._start_block(now)

    # -- phases --
    def _begin_region_header(self, now: float) -> None:
        self.phase = HEADER
        self.region = _Region("header", HEADER_BASE, 0, HEADER_WORDS)
        self.frames_total = HEADER_WORDS
        self.frames_done = 0
        self._start_block(now)

    def _region_done(self, now: float) -> None:
        if self.phase == HEADER:
            self._header_done(now)
        elif self.phase == ANCHORS:
            self._anchors_done(now)
        elif self.phase == SAMPLES:
            self._samples_done(now)

    def _header_done(self, now: float) -> None:
        try:
            h = decode_header(bytes(self.region.buf))
        except HeaderError as e:
            self._fail("header invalid: %s" % e, now, resume=True)
            return
        if h.capture_id != self.cid:
            self._fail("header capture_id 0x%04X != reply 0x%04X" % (h.capture_id, self.cid), now, resume=True)
            return
        if self.expected_n is not None and h.n_samples != self.expected_n:
            self._fail("header n_samples %d != FREEZE reply %d" % (h.n_samples, self.expected_n), now, resume=True)
            return
        if self.boot_low is not None and (h.boot_id & 0xFF) != self.boot_low:
            self.flags["platform_rebooted"] = True
            self._fail("boot_id low byte %d != status boot %d" % (h.boot_id & 0xFF, self.boot_low), now, resume=False)
            return
        self.header = h
        self._begin_region_anchors(now)

    def _begin_region_anchors(self, now: float) -> None:
        h = self.header
        self.phase = ANCHORS
        total_anchor_words = ANCHOR_WORDS * h.n_anchors
        self.anchor_region = self.region = _Region("anchors", ANCHOR_BASE, 0, total_anchor_words)
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
                self.full_coverage = (k0, k1) == (0, h.n_samples)
                self.flags["partial_span"] = not self.full_coverage
        self.k0, self.k1 = k0, k1
        self.phase = SAMPLES
        self.sample_region = self.region = _Region("samples", 0, 2 * k0, 2 * k1)
        self.frames_total = ANCHOR_WORDS * h.n_anchors + 2 * (k1 - k0)
        self.frames_done = ANCHOR_WORDS * h.n_anchors
        if not self._start_block(now):
            self._samples_done(now)

    def _samples_done(self, now: float) -> None:
        h = self.header
        if self.full_coverage and self.sample_region.complete and self.anchor_region.complete:
            self.flags["crc_checked"] = True
            ok = (zlib.crc32(bytes(self.sample_region.buf), zlib.crc32(bytes(self.anchor_region.buf)))
                  & 0xFFFFFFFF) == h.payload_crc32
            if not ok and self.rereads_left > 0:
                self.rereads_left -= 1
                self.stats["crc_rereads"] += 1
                logger.warning("IMU payload CRC mismatch: re-reading the capture once")
                self.phase = ANCHORS
                self.anchor_region = self.region = _Region("anchors", ANCHOR_BASE, 0, ANCHOR_WORDS * h.n_anchors)
                self.frames_done = 0
                self.stalled = 0
                if not self._start_block(now):
                    self._anchors_done(now)
                return
            self.flags["crc_ok"] = ok
        self.flags["complete"] = (self.sample_region.complete and self.flags["crc_ok"] is not False
                                  and not self.flags["aborted"] and not self.flags["platform_rebooted"])
        self._resume(now)

    def _fail(self, why: str, now: float, *, resume: bool) -> None:
        self.failure = self.failure or why
        self.flags["complete"] = False
        logger.warning("IMU upload ended early: %s", why)
        self.blk = None
        if resume and self.cid:
            self._resume(now)
        else:
            self._finish(now)

    def _resume(self, now: float) -> None:
        self.blk = None
        self.phase = RESUMING
        self._cmd = None
        self._send_cmd(OP.RESUME, now)

    def _finish(self, now: float) -> None:
        self.phase = DONE
        sr = self.sample_region
        res = UploadResult(
            header=self.header, anchors=list(self.anchors),
            sample_words=bytes(sr.buf) if sr else b"", word_present=bytes(sr.have) if sr else b"",
            k0=self.k0, flags=dict(self.flags), stats=dict(self.stats, wall_s=now - self.t_job0),
            job=self.job, failure=self.failure)
        res.flags["holes_unfilled"] = sr.missing() if sr else 0
        if sr is not None and sr.n:
            res.flags["complete"] = bool(res.flags["complete"]) and sr.complete
        try:
            self._on_result(res)
        except Exception:  # noqa: BLE001
            logger.exception("IMU upload result handler failed")


# ── the worker service ───────────────────────────────────────────────────────────────────────────

class PlatformImuService:
    """One worker thread owns an :class:`UploadMachine`. Producers (the UDP receive callback, the
    ROS services) only enqueue; they never block, never raise and do bounded work. All RPCs the
    machine sends run on the worker thread, so no ROS callback group is occupied by an upload.

    Counters ``dropped_batches`` / ``dropped_events`` count queue overflows (a hole the machine
    recovers by re-reading)."""

    def __init__(self, send_ctrl, send_read, *, writer=None, clock: Callable[[], float] = time.monotonic,
                 wall: Callable[[], float] = time.time, is_armed: Callable[[], bool] = lambda: False,
                 config: Optional[UploadConfig] = None, start_thread: bool = True,
                 data_queue_size: int = 2048, event_queue_size: int = 256):
        self._clock, self._wall = clock, wall
        self.cfg = config or UploadConfig()
        self._writer = writer
        self.machine = UploadMachine(send_ctrl, send_read, self._on_result, config=self.cfg, is_armed=is_armed)
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

    def request_upload(self, t_start_wall: Optional[float] = None, t_end_wall: Optional[float] = None,
                       reason: int = int(pc.PlatformImuFreezeReason.OPERATOR), attempt_id: Optional[str] = None,
                       label: str = "operator", force: bool = False) -> bool:
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
        job = UploadJob(t_start_wall, t_end_wall, int(reason), attempt_id, label)
        try:
            self._ev_q.put_nowait(("start", job, self._clock()))
        except queue.Full:
            with self._lock:
                self._busy = False
            self.dropped_events += 1
            return False
        return True

    def attempt_ended(self, t_start_wall: float, t_end_wall: float, reason: str = "attempt_end",
                      attempt_id: Optional[str] = None) -> bool:
        """The Attempt is over: freeze the ring and upload its span plus margin, if the toggle is ON."""
        if not self.upload_after_attempt():
            return False
        rmap = {"attempt_end": pc.PlatformImuFreezeReason.ATTEMPT_END, "fault": pc.PlatformImuFreezeReason.FAULT,
                "soft_estop": pc.PlatformImuFreezeReason.SOFT_ESTOP}
        return self.request_upload(t_start_wall, t_end_wall, int(rmap.get(reason, pc.PlatformImuFreezeReason.ATTEMPT_END)),
                                   attempt_id, label=reason)

    def abort_upload(self, reason: str = "abort") -> None:
        """Stop an upload (pacifying command, Attempt start). Returns at once; the worker sends ABORT
        first, writes what arrived (complete = false) and RESUMEs."""
        try:
            self._ev_q.put_nowait(("abort", reason, self._clock()))
        except queue.Full:
            self.dropped_events += 1

    def status_snapshot(self) -> Dict[str, Any]:
        now = self._clock()
        with self._lock:
            st, t = self._last_status, self._last_status_t
            snap: Dict[str, Any] = {
                "upload_after_attempt": self._upload_after_attempt, "busy": self._busy,
                "platform_state": (STATE(st.state).name if st and st.state in set(int(s) for s in STATE) else None),
                "platform_flags": st.flags if st else None, "platform_boot": st.boot if st else None,
                "platform_temp_c": st.temp_c if st else None,
                "status_age_s": (now - t) if t is not None else None,
                "bridge": dict(self._bridge), "last_result": dict(self._last_result), "last_skip": self._last_skip,
                "dropped_batches": self.dropped_batches, "dropped_events": self.dropped_events,
            }
        snap["progress"] = self.machine.progress()
        return snap

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
            self.machine.on_records(records, t)
        self.machine.tick(now)

    def _handle_event(self, ev: tuple) -> None:
        kind = ev[0]
        try:
            if kind == "frame":
                _, can_id, data, t = ev
                if can_id == CAN_ID_REPLY:
                    self.machine.on_reply(decode_imu_reply(data), t)
                elif can_id == CAN_ID_STATUS:
                    st = decode_imu_status(data)
                    with self._lock:
                        if self._boot_seen is not None and st.boot != self._boot_seen:
                            logger.info("Platform IMU boot byte changed %d -> %d (Platform rebooted)",
                                        self._boot_seen, st.boot)
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
        except Exception:  # noqa: BLE001 -- the worker must outlive any one bad frame
            logger.exception("platform IMU event %s failed", kind)
            if kind == "start":
                with self._lock:
                    self._busy = False

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                # Block briefly on the data queue so a batch is handled within a millisecond or two.
                t, info, records = self._data_q.get(timeout=0.02)
                self._bridge = {"ring_hwm": info.ring_hwm, "ring_drops": info.ring_drops,
                                "rx_frames": info.rx_frames, "batches_sent": info.batches_sent,
                                "send_fails": info.send_fails}
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
        summary: Dict[str, Any] = {"label": res.job.label, "attempt_id": res.job.attempt_id,
                                   "flags": dict(res.flags), "failure": res.failure, "stats": dict(res.stats),
                                   "capture_id": res.header.capture_id if res.header else None, "path": None}
        try:
            if res.header is not None:
                meta = {"attempt_id": res.job.attempt_id, "label": res.job.label,
                        "span_wall_s": [res.job.t_start_wall, res.job.t_end_wall],
                        "span_margin_s": self.cfg.span_margin_s, "stats": res.stats, "failure": res.failure,
                        "requested_gap_10us": self.cfg.gap_10us, "uploaded_wall_time_s": self._wall()}
                flags = dict(res.flags)
                flags["discontinuities"] = sum(1 for a in res.anchors if a.flags & 0x01)
                cap = build_capture(res.header, res.anchors, res.sample_words, res.word_present, res.k0, flags, meta)
                summary["fit"] = {k: cap.fit.get(k) for k in ("rate_hz", "resid_rms_us", "n_segments", "wall_ok")}
                summary["flags"] = flags
                if self._writer is not None:
                    self._writer.submit(cap)
                    summary["path"] = "queued"
            else:
                logger.warning("IMU upload produced no capture: %s", res.failure)
        except Exception:  # noqa: BLE001 -- a capture-building bug must not strand the busy flag
            logger.exception("platform IMU capture build failed")
            summary["failure"] = (summary["failure"] + "; " if summary["failure"] else "") + "capture build failed"
        finally:
            with self._lock:
                self._last_result = summary
                self._busy = False

    def close(self, timeout: float = 5.0) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout)
        if self._writer is not None:
            try:
                self._writer.close()
            except Exception:  # noqa: BLE001
                pass
