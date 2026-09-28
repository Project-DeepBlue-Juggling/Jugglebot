#!/usr/bin/env python3
"""Minimum-viable Jetson-side daemon for the can-bridge Teensy link.

This is the MVP: a runnable single-process Jetson that gives the
Teensy what it needs to clear its `LINK_LOST` fault and lock its wall-clock:

    * Sends J→T heartbeats at 10 Hz (so the Teensy's link watchdog clears).
    * Responds to ``RpcMethod.TIME_OF_DAY_QUERY`` with CLOCK_REALTIME µs
      (so the Teensy's time-sync master can anchor its broadcast clock —
      see ADR-0008).
    * Logs T→J telemetry / diagnostic / heartbeat / profile frames at a
      configurable cadence.

No setpoint stream (that's the heavier half of this work — pulls from MPC), no
ROS 2 plumbing (that's the bridge node rewrite). The point of this MVP is
to verify the protocol layer works end-to-end on real hardware before
investing in the rest.

Usage::

    python tools/teensy_link_bridge.py
    python tools/teensy_link_bridge.py --teensy-ip 192.168.42.2
    python tools/teensy_link_bridge.py --duration 30 --verbose

Exits cleanly on Ctrl-C.
"""

from __future__ import annotations

import argparse
import binascii
import logging
import signal
import struct
import sys
import threading
import time
import dataclasses
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

_HERE = Path(__file__).resolve().parent
_REPO = _HERE.parent
sys.path.insert(0, str(_REPO))

from teensy_link import (  # noqa: E402
    TeensyLinkClient,
    RpcServer,
    TimeOfDayServer,
    MsgType,
    HeartbeatT2J,
    Telemetry,
    Profile,
    LinkState,
    FaultState,
)
from teensy_link import protocol as p  # noqa: E402
from teensy_link import rpc_args  # noqa: E402
from teensy_link.rpc import RpcClient, RpcError, RpcTimeout, PlatformFrameWaiter  # noqa: E402
from config.generated import protocol_config as pc  # noqa: E402

#: There is deliberately NO default image: the path is whatever the caller
#: built. The normal callers are `pio run -e teensy40 -t upload` in
#: Teensy_code_platform/, whose upload_command hands the fresh firmware.hex
#: here (platformio.ini, 2026-09-09), and `pio run -e teensy40_can -t upload`
#: in BallButler/ball_butler_main/, which adds `--target bb` (2026-09-28).


def _setup_logging(verbose: bool) -> None:
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s.%(msecs)03d %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


def _on_heartbeat_t2j(msg_type: int, seq: int, payload: bytes, addr: Tuple[str, int]) -> None:
    hb = HeartbeatT2J.unpack(payload)
    try:
        link_name = LinkState(hb.link_state).name
    except ValueError:
        link_name = f"state_{hb.link_state}"
    try:
        fault_name = FaultState(hb.fault_state).name
    except ValueError:
        fault_name = f"fault_{hb.fault_state}"
    # Python's %-formatter does not support %b for binary (that's a printf-ism);
    # bin() returns the '0b...' prefix already, so pass it as %s.
    logging.info(
        "[T2J] uptime=%dms  link=%s  fault=%s  bus1=%d  bus2=%d  flags=%s",
        hb.uptime_ms, link_name, fault_name,
        hb.bus1_health, hb.bus2_health, bin(hb.flags),
    )


def _on_profile(msg_type: int, seq: int, payload: bytes, addr: Tuple[str, int]) -> None:
    prof = Profile.unpack(payload)
    logging.info(
        "[PROFILE] heap=%dB  interp_misses=%d  rtt=%dus  can1_util=%.1f%%  can2_util=%.1f%%",
        prof.free_heap_bytes,
        prof.interp_deadline_misses,
        prof.udp_rtt_us,
        prof.can1_util_x100 / 100.0,
        prof.can2_util_x100 / 100.0,
    )


class _TelemetryRateLogger:
    """Throttled telemetry summary — telemetry arrives at 100 Hz, log at ~1 Hz."""

    def __init__(self, period_s: float = 1.0) -> None:
        self._period = period_s
        self._next_log = 0.0
        self._count_since = 0

    def __call__(self, msg_type: int, seq: int, payload: bytes, addr: Tuple[str, int]) -> None:
        self._count_since += 1
        now = time.time()
        if now < self._next_log:
            return
        tm = Telemetry.unpack(payload)
        logging.info(
            "[TELEM] rate=%dHz  axis0 pos=%.4f rev  vel=%.3f rps  (showing 1-of-100 throttled)",
            self._count_since,
            tm.pos_rev[0],
            tm.vel_rps[0],
        )
        self._count_since = 0
        self._next_log = now + self._period


# ── Firmware-over-CAN update (Platform 2026-09-09, Ball Butler 2026-09-28) ──
# The Platform Teensy's USB port is damaged, so every image after FW 5 has to
# arrive over CAN through the relay seam (PLATFORM_FW_BEGIN/DATA/VERIFY/COMMIT,
# see teensy_link/rpc_args.py). This is the host half of that; the wire
# contract is Teensy_code_platform.ino's "FIRMWARE UPDATE OVER CAN" header
# comment, mirrored (not re-derived) here and in rpc_args.py.
#
# Ball Butler (--target bb, 2026-09-28) speaks the SAME contract — opcodes,
# statuses, frame layouts and arg structs — through its own BB_FW_* RPCs, on
# CAN1 id 0x7D6 with replies on 0x7D7. Two BB-only additions: BEGIN may answer
# PARKING while BB moves pitch to its stow angle and idles its axes (the host
# re-sends BEGIN until OK), and BB_FW_INFO reads the running FW_VERSION, the
# receipt that a COMMIT landed. Everything else below is target-agnostic.

_FLASH_BASE = 0x60000000    # both boards are Teensy 4.0s; a 4.1 image links elsewhere
_FW_DATA_CHUNK = 5          # payload bytes per *_FW_DATA RPC
_FW_REPLY_TIMEOUT_S = 1.0   # a sector flush can stall the board tens of ms
_FW_WINDOW_RETRIES = 1      # retry a stalled window once before aborting
_FW_SECTOR_BYTES = 4096     # the receiver flushes a 4 KB sector when its buffer fills
#: Pause after the frame that fills a sector. The flush (erase 45 ms typ /
#: 400 ms worst, 16 page writes, read-back) runs with the board's interrupts
#: OFF, and frames arriving then are lost from the CAN hardware FIFO — the
#: 2026-09-09 rehearsal lost frames 823..831 right after the first 4 KB, the
#: ACK among them. Sending nothing during the flush is cheaper than the
#: retry-and-rewind it costs; 30 sectors of pause per image.
#:
#: 0.5 s is the value both boards flew with (Platform FW 6, BB FW 2). The BB
#: profile uses _FW_SECTOR_FLUSH_PAUSE_S below (2026-09-28 speed-up); the
#: Platform keeps the flown 0.5 s until a Platform flash adopts the faster
#: profile deliberately (it is a one-way board — see _FW_TARGETS).
_FW_SECTOR_FLUSH_PAUSE_FLOWN_S = 0.5
#: The reduced pause (2026-09-28). The flush is a 4 KB erase (45 ms typical,
#: 400 ms worst for the Teensy 4.0's W25Q16JV) + 16 page writes (~0.4 ms
#: typical each) + a read-back: ~52 ms typical. 0.12 s covers the typical
#: flush with >2x margin; an erase slower than that loses the frames that
#: arrive during its tail, and the BAD_SEQ rewind / window retry — the
#: contract's designed recovery — resends them. The pause is timed from the
#: bridge's ack of the sector-completing frame (all acks drained first), the
#: same reference the 0.5 s pause had.
_FW_SECTOR_FLUSH_PAUSE_S = 0.12
#: DATA RPCs allowed in flight at once. MUST stay 1 on the current can-bridge.
#: The bridge services its RPC socket in a 1 kHz task (task_net), so one
#: synchronous DATA RPC per frame costs one tick — measured 2026-09-28: NOP
#: round trip median 0.998 ms, p90 1.03 ms. Pipelining was built to beat that
#: and FAILED on hardware the same day: the bridge's RPC socket is a QNEthernet
#: EthernetUDP with the library's default receive queue of ONE packet
#: (udp_link.cpp `static EthernetUDP s_rpc;`), so of the requests that land in
#: one tick only one survives — 4 NOPs back to back got 2 acks, and the BB
#: --verify-only run lost 3 of its first DATA frames and aborted cleanly
#: before VERIFY. Depth > 1 needs a bridge FW that raises that capacity
#: (s_rpc.setReceiveQueueCapacity(>= depth) — a firmware change, not made).
#: Even then, keep depth <= 4: the bridge's CAN TX mailboxes all carry the same
#: id (0x7D6 / 0x6F0) and FlexCAN sends same-id mailboxes LOWEST-NUMBER first,
#: so a frame loaded into a freed low mailbox while an older frame waits in a
#: higher one overtakes it (a BAD_SEQ rewind, not corruption); 4 dlc-8 frames
#: are ~0.52 ms of 1 Mbps wire, on the wire before the next tick loads more.
#: Depth 1 is the synchronous rpc.call() path, exactly as flown.
#:
#: FIXED in can-bridge FW 25 on skill-stack (FW 22 on mvp-trajectory-bringup, 2026-09-28): udp_link.cpp sets the RPC socket's
#: queue to RPC_RX_QUEUE_CAPACITY = 8 (<= its 8-frame per-tick drain budget, so
#: no added latency). Probe against FW 22: bursts of 4 and 8 back-to-back NOPs
#: answered 80/80 and 160/160 (20 trials each). BB now pipelines at depth 4;
#: against a bridge older than _FW_PIPELINE_MIN_BRIDGE_FW (or one whose
#: BRIDGE_IDENTITY is not heard) the tool falls back to depth 1, as flown.
#: The Platform stays at depth 1 until a Platform flash decides otherwise.
_FW_PIPELINE_DEPTH = 4
_FW_PIPELINE_MIN_BRIDGE_FW = 25   # skill-stack numbering: its FW 22-24 predate the queue fix (mvp-trajectory-bringup shipped it as FW 22)
#: Pipelined DATA: how long to wait for an in-flight frame's bridge ack before
#: counting it missing. A missing ack is NOT fatal (the board's window reply is
#: the delivery proof, and a frame that never reached CAN is a BAD_SEQ rewind);
#: more than _FW_MAX_MISSING_ACKS in one window attempt means the link is gone
#: and aborts as RpcTimeout, as a synchronous call's exhausted retries did.
_FW_ACK_TIMEOUT_S = 0.5
_FW_MAX_MISSING_ACKS = 2
#: BB answers BEGIN with PARKING until it is parked (pitch at or above its stow
#: angle, both ODrives IDLE, yaw e-stopped). Re-send BEGIN this often, for at
#: most this long; BB's own park timeout is shorter, so a stuck park normally
#: surfaces as its PARK_FAILED rather than as this host-side limit.
_FW_PARK_POLL_S = 0.25
_FW_PARK_TIMEOUT_S = 20.0


@dataclass(frozen=True)
class _FwTarget:
    """Everything that differs between the boards the flash verb can update."""
    key: str
    label: str
    identity: bytes                 # the receiver's FW_NAME marker; the image must carry it
    foreign_identities: tuple       # other boards' markers; an image carrying one is refused
    rpc_begin: int
    rpc_data: int
    rpc_verify: int
    rpc_commit: int
    reply_id: int                   # the board's FW_UPDATE_REPLY CAN id
    requires_disarmed: bool         # refuse while the setpoint output is armed
    rejected_hint: str              # what ERR_REJECTED from the bridge means here
    abort_note: str                 # what the board does with an unfinished session
    version_wait_s: float           # after COMMIT: keep trying the version read this long
    data_pipeline_depth: int        # DATA RPCs in flight; 1 = synchronous, as flown
    sector_pause_s: float           # host pause after the frame that fills a sector


_FW_TARGETS = {
    "platform": _FwTarget(
        key="platform",
        label="Platform Teensy",
        identity=b"jugglebot-platform",
        foreign_identities=(b"ballbutler-main",),
        rpc_begin=int(p.RpcMethod.PLATFORM_FW_BEGIN),
        rpc_data=int(p.RpcMethod.PLATFORM_FW_DATA),
        rpc_verify=int(p.RpcMethod.PLATFORM_FW_VERIFY),
        rpc_commit=int(p.RpcMethod.PLATFORM_FW_COMMIT),
        reply_id=pc.CAN_ID_PLATFORM_FW_UPDATE_REPLY,
        requires_disarmed=True,
        rejected_hint="the bridge's setpoint output is armed (ERR_REJECTED); "
                      "stow the legs / disarm before flashing",
        abort_note="the board discards the staged image after its 60 s session timeout",
        version_wait_s=2.0,
        # As flown (Platform FW 6, 2026-09-09). The Platform has no USB, so the
        # faster BB profile is not adopted here untested: it moves to the
        # Platform as its own decision, at a Platform flash (a failed transfer
        # there only ends without COMMIT, but it is the one board with no
        # fallback). Its relay also rides the busier Jugglebot bus.
        data_pipeline_depth=1,
        sector_pause_s=_FW_SECTOR_FLUSH_PAUSE_FLOWN_S,
    ),
    "bb": _FwTarget(
        key="bb",
        label="Ball Butler",
        identity=b"ballbutler-main",
        foreign_identities=(b"jugglebot-platform",),
        rpc_begin=int(p.RpcMethod.BB_FW_BEGIN),
        rpc_data=int(p.RpcMethod.BB_FW_DATA),
        rpc_verify=int(p.RpcMethod.BB_FW_VERIFY),
        rpc_commit=int(p.RpcMethod.BB_FW_COMMIT),
        reply_id=pc.CAN_ID_BB_FW_UPDATE_REPLY,
        requires_disarmed=False,
        rejected_hint="Ball Butler is not IDLE or ERROR (ERR_REJECTED); let it "
                      "finish its throw / reload / calibration first",
        abort_note="Ball Butler reboots (and re-homes) after its 60 s session timeout",
        version_wait_s=15.0,   # BB's setup() waits for Serial + its ODrives before its loop runs
        data_pipeline_depth=_FW_PIPELINE_DEPTH,   # 4, gated on bridge FW >= 25
        sector_pause_s=_FW_SECTOR_FLUSH_PAUSE_S,
    ),
}


def _load_intel_hex(path) -> Tuple[int, bytes]:
    """Parse an Intel HEX file into (base_address, image_bytes), gaps filled
    0xFF. Supports the record types a Teensy/arm-none-eabi toolchain emits:
    00 (data), 01 (EOF), 02 (extended segment address), 04 (extended linear
    address); 05 (start linear address) is read and ignored (a Teensy boots
    from its IVT, not this field). No `intelhex` package in the project venv
    (checked 2026-09-09) — a small inline parser beats a new dependency for
    one format this narrow."""
    records = []
    ext = 0
    with open(path, "r") as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            if not line.startswith(":"):
                raise ValueError(f"{path}:{lineno}: record does not start with ':'")
            raw = bytes.fromhex(line[1:])
            if len(raw) < 5:
                raise ValueError(f"{path}:{lineno}: record too short")
            byte_count, addr_hi, addr_lo, rtype = raw[0], raw[1], raw[2], raw[3]
            data = raw[4:4 + byte_count]
            checksum = raw[4 + byte_count]
            if ((-(sum(raw[:4 + byte_count]))) & 0xFF) != checksum:
                raise ValueError(f"{path}:{lineno}: checksum mismatch")
            addr16 = (addr_hi << 8) | addr_lo
            if rtype == 0x00:
                records.append((ext + addr16, data))
            elif rtype == 0x01:
                break
            elif rtype == 0x02:
                ext = ((data[0] << 8) | data[1]) << 4
            elif rtype == 0x04:
                ext = ((data[0] << 8) | data[1]) << 16
            elif rtype == 0x05:
                pass
            else:
                raise ValueError(f"{path}:{lineno}: unsupported record type 0x{rtype:02X}")
    if not records:
        raise ValueError(f"{path}: no data records")
    base = min(a for a, _ in records)
    top = max(a + len(d) for a, d in records)
    image = bytearray(b"\xff" * (top - base))
    for addr, data in records:
        image[addr - base: addr - base + len(data)] = data
    return base, bytes(image)


def _prepare_image(hex_path, target: _FwTarget = _FW_TARGETS["platform"]) -> Tuple[bytes, int]:
    """Load + validate a Teensy 4.0 Intel HEX image for ``target``. Returns
    (image_bytes, crc32). Refuses a base != FLASH_BASE (a can-bridge or
    foreign image links at a different address), a missing identity marker
    (the receiver's own identityOk check, verified host-side before a single
    byte reaches the wire) and an image carrying ANOTHER board's marker."""
    base, image = _load_intel_hex(hex_path)
    if base != _FLASH_BASE:
        raise ValueError(
            f"{hex_path}: base address 0x{base:08X} != Teensy 4.0 flash "
            f"base 0x{_FLASH_BASE:08X} — wrong board's image?"
        )
    if target.identity not in image:
        raise ValueError(
            f"{hex_path}: no {target.identity.decode()!r} marker found — "
            f"not a {target.label} build"
        )
    for foreign in target.foreign_identities:
        if foreign in image:
            raise ValueError(
                f"{hex_path}: carries another board's {foreign.decode()!r} marker — "
                f"refusing to send it to the {target.label}"
            )
    crc = binascii.crc32(image) & 0xFFFFFFFF
    return image, crc


def _offset_for_frame(frame_idx: int, total: int) -> int:
    return min(frame_idx * _FW_DATA_CHUNK, total)


def _await_heartbeat(client: TeensyLinkClient, timeout: float) -> Optional[HeartbeatT2J]:
    """Wait briefly for a T2J heartbeat so the fw-update flow can check the
    bridge's live armed-state before touching the bus, rather than firing RPCs
    blind at a link that might not even be up. Returns the latest HeartbeatT2J,
    or None if none arrived in time."""
    box = {}
    ev = threading.Event()

    def _on_hb(msg_type, seq, payload, addr):
        box["hb"] = HeartbeatT2J.unpack(payload)
        ev.set()

    unsubscribe = client.subscribe(int(MsgType.HEARTBEAT_T2J), _on_hb)
    try:
        ev.wait(timeout)
        return box.get("hb")
    finally:
        unsubscribe()


def _log_fw_refusal(step: str, status: int, detail: int, crc: Optional[int] = None) -> None:
    name = rpc_args.PLATFORM_FW_STATUS_NAMES.get(status, f"status_{status}")
    if status == rpc_args.PLATFORM_FW_STATUS_TOO_BIG:
        logging.error("fw-update: %s — TOO_BIG: staging capacity is %d B", step, detail)
    elif status == rpc_args.PLATFORM_FW_STATUS_BAD_CRC and crc is not None:
        logging.error("fw-update: %s — BAD_CRC: board computed 0x%08X, expected 0x%08X",
                      step, detail, crc)
    elif status == rpc_args.PLATFORM_FW_STATUS_PARK_FAILED:
        logging.error("fw-update: %s — PARK_FAILED: pitch never reached its stow angle "
                      "(last %.1f deg); Ball Butler reboots", step, _centideg(detail))
    else:
        logging.error("fw-update: %s — %s", step, name)


def _centideg(detail: int) -> float:
    """PARKING / PARK_FAILED carry BB's pitch in signed centidegrees."""
    v = detail & 0xFFFFFFFF
    if v & 0x80000000:
        v -= 1 << 32
    return v / 100.0


def _wait_for_window_reply(waiter: PlatformFrameWaiter, reply_id: int,
                           window_start_frame: int, window_end: int) -> Optional[bytes]:
    """Wait for the reply that belongs to THIS window: the OK ACK for its last
    frame, or a BAD_SEQ whose expected frame is inside or at the end of it. A
    reply about an OLDER frame is a straggler from the previous window's
    duplicates (the NAK burst a retry provokes arrives one frame at a time,
    after the window that provoked it has been read) and is skipped, not
    acted on — acting on it re-sent an already-accepted window, provoked
    another burst, and so on: two rewinds per window for the whole
    2026-09-09 rehearsal. Returns None on timeout."""
    deadline = time.monotonic() + _FW_REPLY_TIMEOUT_S
    last_seq = (window_end - 1) & 0xFFFF
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return None
        reply = waiter.wait(reply_id, expected_dlc=8, timeout=remaining)
        if reply is None:
            return None
        opcode, status, seq, _ = rpc_args.decode_platform_fw_reply(reply)
        if opcode == 0x02 and status == rpc_args.PLATFORM_FW_STATUS_OK and seq == last_seq:
            return reply
        if status == rpc_args.PLATFORM_FW_STATUS_BAD_SEQ:
            expected = rpc_args.platform_fw_rewind_frame(window_end, seq)
            if expected >= window_start_frame:
                return reply
        elif status != rpc_args.PLATFORM_FW_STATUS_OK:
            return reply                            # a refusal is always for us
        waiter.clear(reply_id)                      # stale — wait for the next one


class _DataSender:
    """Delivers DATA RPCs with up to ``depth`` bridge acks outstanding.

    Depth 1 is the synchronous ``rpc.call()`` (its retry included) — the path
    both boards flew. Depth > 1 sends through ``RpcClient.send_nowait`` and
    collects acks oldest-first once ``depth`` are in flight: a non-OK ack (the
    bridge refusing — ERR_REJECTED / ERR_BUS_DOWN / ERR_UNKNOWN_METHOD /
    ERR_TIMEOUT) raises RpcError exactly as ``rpc.call()`` would; a MISSING ack
    is counted and tolerated (the board's window reply decides delivery), up to
    _FW_MAX_MISSING_ACKS per window attempt, past which RpcTimeout aborts.
    Nothing here decides that a frame was delivered."""

    def __init__(self, rpc: RpcClient, data_method: int, depth: int, stats: dict):
        self._rpc, self._method, self._depth, self._stats = rpc, data_method, depth, stats
        self._inflight = []
        self._missing = 0

    def send(self, args: bytes) -> None:
        if self._depth <= 1:
            self._rpc.call(self._method, args)
            return
        while len(self._inflight) >= self._depth:
            self._collect_oldest()
        self._inflight.append(self._rpc.send_nowait(self._method, args))

    def drain(self) -> None:
        """Collect every outstanding ack (the frames are then on the bridge's CAN)."""
        while self._inflight:
            self._collect_oldest()

    def release(self) -> None:
        for c in self._inflight:
            c.release()
        self._inflight = []

    def _collect_oldest(self) -> None:
        call = self._inflight.pop(0)
        if not call.wait(_FW_ACK_TIMEOUT_S):
            call.release()
            self._missing += 1
            self._stats["missing_acks"] = self._stats.get("missing_acks", 0) + 1
            logging.warning("fw-update: DATA — no bridge ack for req_id %d within %.1f s "
                            "(the board's window reply decides delivery)",
                            call.req_id, _FW_ACK_TIMEOUT_S)
            if self._missing > _FW_MAX_MISSING_ACKS:
                raise RpcTimeout(self._method, 0, _FW_ACK_TIMEOUT_S)
            return
        call.result()                                   # raises RpcError on a refusal


def _send_data_window(rpc: RpcClient, waiter: PlatformFrameWaiter, reply_id: int,
                      image: bytes, window_start_frame: int, total: int,
                      data_method: int = int(p.RpcMethod.PLATFORM_FW_DATA),
                      depth: int = 1,
                      sector_pause_s: float = _FW_SECTOR_FLUSH_PAUSE_FLOWN_S,
                      stats: Optional[dict] = None):
    """Send one DATA window (up to 16 frames) starting at
    ``window_start_frame`` (absolute, unwrapped), retrying the WHOLE window
    once if no reply arrives (the sector-flush-stall NAK burst the firmware's
    wire contract documents as the DESIGNED recovery path). Returns
    (new_frame_idx, reply_bytes) on success, or None after exhausting
    retries — the caller aborts. ``depth`` > 1 pipelines the window's frames
    (see _DataSender); the window stays stop-and-wait on the board's reply."""
    if stats is None:
        stats = {}
    total_frames = (total + _FW_DATA_CHUNK - 1) // _FW_DATA_CHUNK
    # The window ENDS on an ACK point (seq % 16 == 15, or the final frame) —
    # after a BAD_SEQ rewind that makes the first window a partial one. And the
    # waiter is cleared ONCE per window, not per frame: the reply the board
    # sends for the window's last frame (or the NAK burst of a retry) must not
    # be thrown away by the next send. Both were the 2026-09-09 first-attempt
    # failure; see rpc_args.platform_fw_window_end.
    window_end = rpc_args.platform_fw_window_end(window_start_frame, total_frames)
    for attempt in range(_FW_WINDOW_RETRIES + 1):
        frame_idx = window_start_frame
        waiter.clear(reply_id)
        sender = _DataSender(rpc, data_method, depth, stats)
        try:
            while frame_idx < window_end:
                off = _offset_for_frame(frame_idx, total)
                chunk = image[off: off + _FW_DATA_CHUNK]
                sender.send(rpc_args.encode_platform_fw_data(frame_idx & 0xFFFF, chunk))
                frame_idx += 1
                if (off + len(chunk)) % _FW_SECTOR_BYTES < len(chunk):
                    # The board is flushing a sector. Time the pause from the
                    # bridge's ack of this frame, as the synchronous path did.
                    sender.drain()
                    time.sleep(sector_pause_s)
            sender.drain()
        finally:
            sender.release()
        reply = _wait_for_window_reply(waiter, reply_id, window_start_frame, window_end)
        if reply is not None:
            return frame_idx, reply
        stats["retries"] = stats.get("retries", 0) + 1
        if attempt < _FW_WINDOW_RETRIES:
            logging.warning("fw-update: DATA window [frame %d..%d) — no ACK, retrying once",
                            window_start_frame, frame_idx)
    logging.error("fw-update: DATA window starting frame %d — no ACK after retry, aborting",
                  window_start_frame)
    return None


def _begin_session(rpc: RpcClient, waiter: PlatformFrameWaiter, target: _FwTarget,
                   total: int) -> bool:
    """BEGIN, re-sent while the board answers PARKING (Ball Butler moving pitch
    to its stow angle and idling its axes before it accepts an image — the
    Platform never parks, so for it this is a single BEGIN). Returns True once
    the board answers OK."""
    deadline = time.monotonic() + _FW_PARK_TIMEOUT_S
    next_log = 0.0
    while True:
        waiter.clear(target.reply_id)
        rpc.call(target.rpc_begin, rpc_args.encode_platform_fw_begin(total))
        reply = waiter.wait(target.reply_id, expected_dlc=8, timeout=_FW_REPLY_TIMEOUT_S)
        if reply is None:
            logging.error("fw-update: BEGIN — no reply from the %s", target.label)
            return False
        _, status, _, detail = rpc_args.decode_platform_fw_reply(reply)
        if status == rpc_args.PLATFORM_FW_STATUS_PARKING:
            now = time.monotonic()
            if now >= deadline:
                logging.error("fw-update: BEGIN — still PARKING after %.0f s (pitch %.1f deg); "
                              "giving up", _FW_PARK_TIMEOUT_S, _centideg(detail))
                return False
            if now >= next_log:
                logging.info("fw-update: BEGIN — %s parking (pitch %.1f deg)",
                             target.label, _centideg(detail))
                next_log = now + 1.0
            time.sleep(_FW_PARK_POLL_S)
            continue
        if status != rpc_args.PLATFORM_FW_STATUS_OK:
            _log_fw_refusal("BEGIN", status, detail)
            return False
        logging.info("fw-update: BEGIN OK — %d B declared", total)
        return True


def _run_fw_update_session(rpc: RpcClient, waiter: PlatformFrameWaiter,
                           reply_id: int, image: bytes, crc: int,
                           commit: bool = True,
                           target: _FwTarget = _FW_TARGETS["platform"]) -> int:
    total = len(image)

    # BEGIN
    if not _begin_session(rpc, waiter, target, total):
        return 1

    # DATA
    frame_idx = 0
    stats = {}          # rewinds / retries / missing_acks, for the summary line
    t_start = time.monotonic()
    next_log = 0.0
    while _offset_for_frame(frame_idx, total) < total:
        outcome = _send_data_window(rpc, waiter, reply_id, image, frame_idx, total,
                                    data_method=target.rpc_data,
                                    depth=target.data_pipeline_depth,
                                    sector_pause_s=target.sector_pause_s,
                                    stats=stats)
        if outcome is None:
            return 1
        frame_idx, reply = outcome
        _, status, reply_seq, _ = rpc_args.decode_platform_fw_reply(reply)
        if status == rpc_args.PLATFORM_FW_STATUS_BAD_SEQ:
            stats["rewinds"] = stats.get("rewinds", 0) + 1
            frame_idx = rpc_args.platform_fw_rewind_frame(frame_idx, reply_seq)
            logging.warning("fw-update: BAD_SEQ — board expects seq=%d, rewinding to frame %d",
                            reply_seq, frame_idx)
            continue
        if status != rpc_args.PLATFORM_FW_STATUS_OK:
            _log_fw_refusal("DATA", status, 0)
            return 1
        now = time.monotonic()
        sent = _offset_for_frame(frame_idx, total)
        if now >= next_log or sent >= total:
            logging.info("fw-update: DATA %d/%d B (%.1f%%) elapsed=%.1fs",
                        sent, total, 100.0 * sent / total, now - t_start)
            next_log = now + 1.0
    logging.info("fw-update: DATA complete — %d B in %.1f s (pipeline depth %d, sector "
                 "pause %.2f s): %d rewinds, %d window retries, %d missing acks",
                 total, time.monotonic() - t_start, target.data_pipeline_depth,
                 target.sector_pause_s, stats.get("rewinds", 0), stats.get("retries", 0),
                 stats.get("missing_acks", 0))

    # VERIFY
    waiter.clear(reply_id)
    rpc.call(target.rpc_verify, rpc_args.encode_platform_fw_verify(crc))
    reply = waiter.wait(reply_id, expected_dlc=8, timeout=_FW_REPLY_TIMEOUT_S)
    if reply is None:
        logging.error("fw-update: VERIFY — no reply from the %s", target.label)
        return 1
    _, status, _, detail = rpc_args.decode_platform_fw_reply(reply)
    if status != rpc_args.PLATFORM_FW_STATUS_OK:
        _log_fw_refusal("VERIFY", status, detail, crc=crc)
        return 1
    logging.info("fw-update: VERIFY OK — crc32=0x%08X — COMMIT armed", crc)
    if not commit:
        # --verify-only: everything but the one-way step has now run on the real
        # board — the CAN transport, the relay, the staging erase/write/read-back
        # and the CRC + identity check. Nothing was written to the program region.
        logging.info("fw-update: --verify-only — stopping before COMMIT; %s",
                     target.abort_note)
        return 0

    # COMMIT
    waiter.clear(reply_id)
    rpc.call(target.rpc_commit, rpc_args.encode_platform_fw_commit())
    reply = waiter.wait(reply_id, expected_dlc=8, timeout=_FW_REPLY_TIMEOUT_S)
    if reply is None:
        logging.error("fw-update: COMMIT — no reply from the %s", target.label)
        return 1
    _, status, _, _ = rpc_args.decode_platform_fw_reply(reply)
    if status != rpc_args.PLATFORM_FW_STATUS_OK:
        _log_fw_refusal("COMMIT", status, 0)
        return 1
    logging.info("fw-update: COMMIT OK — board is copying flash and will reboot")
    return 0


def _read_platform_fw_version(rpc: RpcClient, waiter: PlatformFrameWaiter,
                              timeout: float = _FW_REPLY_TIMEOUT_S) -> Optional[int]:
    """STATE_READ: trigger + await the Platform-Teensy RobotState reply and
    decode its FW_VERSION field — the same relay correlation
    teensy_bridge_node.py's relay_read_robot_state uses. Returns the version,
    or None if the read failed (no reply, or the RPC itself was refused)."""
    can_id = pc.CAN_ID_PLATFORM_STATE_UPDATE
    waiter.clear(can_id)
    try:
        rpc.call(int(p.RpcMethod.STATE_READ))
    except (RpcError, RpcTimeout) as e:
        logging.warning("fw-update: STATE_READ failed: %s", e)
        return None
    data = waiter.wait(can_id, expected_dlc=8, timeout=timeout)
    if data is None:
        logging.warning("fw-update: STATE_READ — no Platform reply within timeout")
        return None
    return rpc_args.decode_platform_fw_version(data)


def _read_bb_fw_version(rpc: RpcClient, waiter: PlatformFrameWaiter,
                        timeout: float = _FW_REPLY_TIMEOUT_S) -> Optional[int]:
    """BB_FW_INFO: BB answers on 0x7D7 with its running FW_VERSION in the
    reply's detail field. None if no reply — which is also what an image that
    predates the receiver (the one USB-flashed before it) gives."""
    reply_id = pc.CAN_ID_BB_FW_UPDATE_REPLY
    waiter.clear(reply_id)
    try:
        rpc.call(int(p.RpcMethod.BB_FW_INFO))
    except (RpcError, RpcTimeout) as e:
        logging.debug("fw-update: BB_FW_INFO failed: %s", e)
        return None
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return None
        data = waiter.wait(reply_id, expected_dlc=8, timeout=remaining)
        if data is None:
            return None
        opcode, status, _, detail = rpc_args.decode_platform_fw_reply(data)
        if opcode == rpc_args.FW_OP_INFO and status == rpc_args.PLATFORM_FW_STATUS_OK:
            return detail
        waiter.clear(reply_id)                     # a straggler from the session


def _read_fw_version(target: _FwTarget, rpc: RpcClient, waiter: PlatformFrameWaiter,
                     timeout: float = _FW_REPLY_TIMEOUT_S) -> Optional[int]:
    if target.key == "bb":
        return _read_bb_fw_version(rpc, waiter, timeout)
    return _read_platform_fw_version(rpc, waiter, timeout)


def _await_bridge_fw_version(client: TeensyLinkClient, timeout: float) -> Optional[int]:
    """The can-bridge's FW_VERSION from its 1 Hz BRIDGE_IDENTITY uplink, or None
    if none arrived within ``timeout``."""
    box = {}
    ev = threading.Event()

    def _on_id(msg_type, seq, payload, addr):
        box["fw"] = int(p.BridgeIdentity.unpack(payload).fw_version)
        ev.set()

    unsubscribe = client.subscribe(int(MsgType.BRIDGE_IDENTITY), _on_id)
    try:
        ev.wait(timeout)
        return box.get("fw")
    finally:
        unsubscribe()


def _effective_target(target: _FwTarget, bridge_fw: Optional[int]) -> _FwTarget:
    """Pipelined DATA (depth > 1) needs a bridge whose RPC socket queues a burst
    (FW >= _FW_PIPELINE_MIN_BRIDGE_FW). Against an older or unheard bridge, fall
    back to the synchronous depth-1 path — slower, never less safe: VERIFY gates
    COMMIT either way."""
    if target.data_pipeline_depth <= 1:
        return target
    if bridge_fw is not None and bridge_fw >= _FW_PIPELINE_MIN_BRIDGE_FW:
        return target
    logging.warning("fw-update: can-bridge FW %s < %d — DATA runs synchronous (depth 1), "
                    "not pipelined", bridge_fw if bridge_fw is not None else "unknown",
                    _FW_PIPELINE_MIN_BRIDGE_FW)
    return dataclasses.replace(target, data_pipeline_depth=1)


def run_fw_update(teensy_ip: str, hex_path, dry_run: bool, verbose: bool,
                  bind_host: str = "0.0.0.0", verify_only: bool = False,
                  target: str = "platform") -> int:
    _setup_logging(verbose)
    tgt = _FW_TARGETS[target]
    logging.info("fw-update: target %s — loading %s", tgt.label, hex_path)
    try:
        image, crc = _prepare_image(hex_path, tgt)
    except (OSError, ValueError) as e:
        logging.error("fw-update: %s", e)
        return 1
    logging.info("fw-update: image %d bytes, crc32=0x%08X, identity OK", len(image), crc)
    if dry_run:
        logging.info("fw-update: --dry-run — stopping before touching the board")
        return 0

    client = TeensyLinkClient(teensy_addr=(teensy_ip, p.PORT_STREAM), bind_host=bind_host)
    client.start()
    client.start_heartbeat(hz=float(p.HEARTBEAT_HZ), flags=0)
    try:
        hb = _await_heartbeat(client, timeout=2.0)
        if hb is None:
            logging.error("fw-update: no HEARTBEAT_T2J from the bridge — link down?")
            return 1
        if tgt.requires_disarmed and hb.flags & int(p.HeartbeatT2JFlags.MPC_ACTIVE):
            logging.error(
                "fw-update: refused — the setpoint output is armed (MPC_ACTIVE); "
                "stow the legs / disarm before flashing the %s", tgt.label
            )
            return 1

        if tgt.data_pipeline_depth > 1:
            tgt = _effective_target(tgt, _await_bridge_fw_version(client, timeout=2.5))

        rpc = RpcClient(client, default_timeout=0.5, default_retries=1)
        waiter = PlatformFrameWaiter(client)
        try:
            old_version = _read_fw_version(tgt, rpc, waiter)
            logging.info("fw-update: %s FW version before update: %s", tgt.label,
                        old_version if old_version is not None else "unknown")

            exit_code = _run_fw_update_session(rpc, waiter, tgt.reply_id, image, crc,
                                               commit=not verify_only, target=tgt)

            if exit_code == 0 and not verify_only:
                logging.info("fw-update: COMMIT sent — waiting for the board to reboot")
                time.sleep(3.0)
                new_version = None
                deadline = time.monotonic() + tgt.version_wait_s
                while new_version is None and time.monotonic() < deadline:
                    new_version = _read_fw_version(tgt, rpc, waiter, timeout=min(
                        2.0, max(0.1, deadline - time.monotonic())))
                logging.info(
                    "fw-update: %s FW version: %s -> %s", tgt.label,
                    old_version if old_version is not None else "unknown",
                    new_version if new_version is not None else "unknown",
                )
            return exit_code
        except RpcTimeout as e:
            logging.error("fw-update: RPC timed out: %s", e)
            return 1
        except RpcError as e:
            if e.status == int(p.RpcStatus.ERR_REJECTED):
                logging.error("fw-update: refused — %s", tgt.rejected_hint)
            elif e.status == int(p.RpcStatus.ERR_BUS_DOWN):
                logging.error("fw-update: refused — the bridge sees no live %s on its bus "
                              "(ERR_BUS_DOWN)", tgt.label)
            elif e.status == int(p.RpcStatus.ERR_UNKNOWN_METHOD):
                logging.error("fw-update: the bridge does not know this RPC "
                              "(ERR_UNKNOWN_METHOD) — flash can-bridge FW >= %d first",
                              21 if tgt.key == "bb" else 19)
            else:
                logging.error("fw-update: RPC failed: %s", e)
            return 1
        finally:
            waiter.close()
            rpc.close()
    finally:
        client.stop()


def run(teensy_ip: str, duration: float, verbose: bool, bind_host: str = "0.0.0.0") -> int:
    _setup_logging(verbose)
    logging.info("teensy_link_bridge MVP — peer=%s ports stream=%d rpc=%d",
                 teensy_ip, p.PORT_STREAM, p.PORT_RPC)

    client = TeensyLinkClient(
        teensy_addr=(teensy_ip, p.PORT_STREAM),
        bind_host=bind_host,
    )
    client.start()

    rpc_server = RpcServer(client)
    tod = TimeOfDayServer(rpc_server)

    # Subscribe to interesting uplink frames. Telemetry is throttled; the rest are 1-10 Hz.
    client.subscribe(int(MsgType.HEARTBEAT_T2J), _on_heartbeat_t2j)
    client.subscribe(int(MsgType.PROFILE), _on_profile)
    client.subscribe(int(MsgType.TELEMETRY), _TelemetryRateLogger(period_s=1.0))

    # Start sending heartbeats. The HeartbeatJ2T.flags = 0 means mpc_active=0
    # — i.e. the Teensy will know the Jetson is alive but will NOT enable
    # leg-output (the setpoint stream isn't implemented in this MVP).
    client.start_heartbeat(hz=float(p.HEARTBEAT_HZ), flags=0)

    # Periodic link-status summary
    def _summary_loop_until(stop_at: float) -> int:
        while time.time() < stop_at:
            time.sleep(2.0)
            since_t2j = client.time_since_last_t2j_heartbeat_us()
            t2j_s = "never" if since_t2j is None else f"{since_t2j / 1e6:.1f}s ago"
            logging.info(
                "[summary] tx=%d  rx=%d  crc_err=%d  decode_err=%d  tod_calls=%d  last_t2j=%s",
                client.stats.tx_frames, client.stats.rx_frames,
                client.stats.crc_errors, client.stats.decode_errors,
                tod.call_count, t2j_s,
            )
        return 0

    stop_at = (time.time() + duration) if duration > 0 else float("inf")
    exit_code = 0
    try:
        exit_code = _summary_loop_until(stop_at)
    except KeyboardInterrupt:
        logging.info("interrupted — shutting down")
    finally:
        tod.close()
        rpc_server.close()
        client.stop()
        logging.info(
            "final: tx=%d rx=%d crc_err=%d tod_calls=%d",
            client.stats.tx_frames, client.stats.rx_frames,
            client.stats.crc_errors, tod.call_count,
        )
    return exit_code


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--teensy-ip", default=p.TEENSY_IP,
                        help="Teensy IP (default %(default)s)")
    parser.add_argument("--bind-host", default="0.0.0.0",
                        help="Local interface to bind on (default %(default)s)")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Seconds to run (default 0 = until Ctrl-C)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="DEBUG-level logging")
    parser.add_argument("--fw-update", metavar="HEX_PATH",
                        help="Update a board's firmware over CAN from HEX_PATH "
                             "(normally the firmware.hex `pio run -t upload` just built "
                             "and handed over) and exit; the board is --target. See "
                             "--dry-run and --verify-only. The daemon loop above does "
                             "not run in this mode.")
    parser.add_argument("--target", choices=sorted(_FW_TARGETS), default="platform",
                        help="With --fw-update: which board to flash — 'platform' "
                             "(the Platform Teensy, CAN3; the default) or 'bb' (Ball "
                             "Butler, CAN1, which parks itself first: pitch to its stow "
                             "angle, ODrives IDLE, yaw disabled)")
    parser.add_argument("--dry-run", action="store_true",
                        help="With --fw-update: parse + verify identity/CRC only, "
                             "don't touch the board")
    parser.add_argument("--verify-only", action="store_true",
                        help="With --fw-update: run BEGIN, DATA and VERIFY on the real "
                             "board but never COMMIT — proves the whole path except "
                             "the one-way copy; the board discards the staged image "
                             "after 60 s")
    args = parser.parse_args()

    # Graceful Ctrl-C without a stack trace
    signal.signal(signal.SIGINT, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))

    if args.fw_update is not None:
        return run_fw_update(args.teensy_ip, args.fw_update, args.dry_run,
                             args.verbose, args.bind_host,
                             verify_only=args.verify_only, target=args.target)

    return run(args.teensy_ip, args.duration, args.verbose, args.bind_host)


if __name__ == "__main__":
    sys.exit(main())
