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

#: There is deliberately NO default image. The only images this board has ever
#: run are Arduino-IDE builds; the PlatformIO build of the same sketch was
#: CAN-MUTE on the hardware (2026-07-31, see Teensy_code_platform/platformio.ini
#: — the suspicion is Teensy-4.1-flavoured pin-mux leakage from the patched
#: linker script), and with the USB port gone a CAN-mute image is a one-way
#: brick: it boots, it never answers 0x6F0, and nothing can reach it again.
#: So the path is explicit, and anything under a `.pio/` build directory is
#: refused unless --allow-pio-image says the image was bench-proven.


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


# ── Platform firmware-over-CAN update (2026-09-09) ─────────────────────────
# The Platform Teensy's USB port is damaged, so every image after FW 5 has to
# arrive over CAN through the relay seam (PLATFORM_FW_BEGIN/DATA/VERIFY/COMMIT,
# see teensy_link/rpc_args.py). This is the host half of that; the wire
# contract is Teensy_code_platform.ino's "FIRMWARE UPDATE OVER CAN" header
# comment, mirrored (not re-derived) here and in rpc_args.py.

_PLATFORM_FLASH_BASE = 0x60000000
_PLATFORM_FW_IDENTITY = b"jugglebot-platform"
_FW_DATA_CHUNK = 5          # payload bytes per PLATFORM_FW_DATA RPC
_FW_DATA_WINDOW = 16        # ACK cadence: firmware ACKs seq % 16 == 15
_FW_REPLY_TIMEOUT_S = 1.0   # a sector flush can stall the board tens of ms
_FW_WINDOW_RETRIES = 1      # retry a stalled window once before aborting


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


def _prepare_image(hex_path) -> Tuple[bytes, int]:
    """Load + validate a Platform-Teensy Intel HEX image. Returns
    (image_bytes, crc32). Refuses a base != FLASH_BASE (a can-bridge or
    foreign image links at a different address) and a missing identity marker
    (FwUpdate::identityOk's own check, verified host-side before a single byte
    reaches the wire)."""
    base, image = _load_intel_hex(hex_path)
    if base != _PLATFORM_FLASH_BASE:
        raise ValueError(
            f"{hex_path}: base address 0x{base:08X} != Platform Teensy flash "
            f"base 0x{_PLATFORM_FLASH_BASE:08X} — wrong board's image?"
        )
    if _PLATFORM_FW_IDENTITY not in image:
        raise ValueError(
            f"{hex_path}: no {_PLATFORM_FW_IDENTITY.decode()!r} marker found — "
            "not a Platform Teensy build"
        )
    crc = binascii.crc32(image) & 0xFFFFFFFF
    return image, crc


def _offset_for_frame(frame_idx: int, total: int) -> int:
    return min(frame_idx * _FW_DATA_CHUNK, total)


def _await_heartbeat(client: TeensyLinkClient, timeout: float) -> Optional[HeartbeatT2J]:
    """Wait briefly for a T2J heartbeat so the fw-update flow can check the
    bridge's live armed-state before touching CAN3, rather than firing RPCs
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


def _log_platform_fw_refusal(step: str, status: int, detail: int, crc: Optional[int] = None) -> None:
    name = rpc_args.PLATFORM_FW_STATUS_NAMES.get(status, f"status_{status}")
    if status == rpc_args.PLATFORM_FW_STATUS_TOO_BIG:
        logging.error("fw-update: %s — TOO_BIG: staging capacity is %d B", step, detail)
    elif status == rpc_args.PLATFORM_FW_STATUS_BAD_CRC and crc is not None:
        logging.error("fw-update: %s — BAD_CRC: board computed 0x%08X, expected 0x%08X",
                      step, detail, crc)
    else:
        logging.error("fw-update: %s — %s", step, name)


def _send_data_window(rpc: RpcClient, waiter: PlatformFrameWaiter, reply_id: int,
                      image: bytes, window_start_frame: int, total: int):
    """Send one DATA window (up to 16 frames) starting at
    ``window_start_frame`` (absolute, unwrapped), retrying the WHOLE window
    once if no reply arrives (the sector-flush-stall NAK burst the firmware's
    wire contract documents as the DESIGNED recovery path). Returns
    (new_frame_idx, reply_bytes) on success, or None after exhausting
    retries — the caller aborts."""
    for attempt in range(_FW_WINDOW_RETRIES + 1):
        frame_idx = window_start_frame
        n = 0
        while n < _FW_DATA_WINDOW and _offset_for_frame(frame_idx, total) < total:
            off = _offset_for_frame(frame_idx, total)
            chunk = image[off: off + _FW_DATA_CHUNK]
            waiter.clear(reply_id)
            rpc.call(int(p.RpcMethod.PLATFORM_FW_DATA),
                    rpc_args.encode_platform_fw_data(frame_idx & 0xFFFF, chunk))
            frame_idx += 1
            n += 1
        reply = waiter.wait(reply_id, expected_dlc=8, timeout=_FW_REPLY_TIMEOUT_S)
        if reply is not None:
            return frame_idx, reply
        if attempt < _FW_WINDOW_RETRIES:
            logging.warning("fw-update: DATA window [frame %d..%d) — no ACK, retrying once",
                            window_start_frame, frame_idx)
    logging.error("fw-update: DATA window starting frame %d — no ACK after retry, aborting",
                  window_start_frame)
    return None


def _run_fw_update_session(rpc: RpcClient, waiter: PlatformFrameWaiter,
                           reply_id: int, image: bytes, crc: int,
                           commit: bool = True) -> int:
    total = len(image)

    # BEGIN
    waiter.clear(reply_id)
    rpc.call(int(p.RpcMethod.PLATFORM_FW_BEGIN), rpc_args.encode_platform_fw_begin(total))
    reply = waiter.wait(reply_id, expected_dlc=8, timeout=_FW_REPLY_TIMEOUT_S)
    if reply is None:
        logging.error("fw-update: BEGIN — no reply from the Platform Teensy")
        return 1
    _, status, _, detail = rpc_args.decode_platform_fw_reply(reply)
    if status != rpc_args.PLATFORM_FW_STATUS_OK:
        _log_platform_fw_refusal("BEGIN", status, detail)
        return 1
    logging.info("fw-update: BEGIN OK — %d B declared", total)

    # DATA
    frame_idx = 0
    t_start = time.monotonic()
    next_log = 0.0
    while _offset_for_frame(frame_idx, total) < total:
        outcome = _send_data_window(rpc, waiter, reply_id, image, frame_idx, total)
        if outcome is None:
            return 1
        frame_idx, reply = outcome
        _, status, reply_seq, _ = rpc_args.decode_platform_fw_reply(reply)
        if status == rpc_args.PLATFORM_FW_STATUS_BAD_SEQ:
            frame_idx = rpc_args.platform_fw_rewind_frame(frame_idx, reply_seq)
            logging.warning("fw-update: BAD_SEQ — board expects seq=%d, rewinding to frame %d",
                            reply_seq, frame_idx)
            continue
        if status != rpc_args.PLATFORM_FW_STATUS_OK:
            _log_platform_fw_refusal("DATA", status, 0)
            return 1
        now = time.monotonic()
        sent = _offset_for_frame(frame_idx, total)
        if now >= next_log or sent >= total:
            logging.info("fw-update: DATA %d/%d B (%.1f%%) elapsed=%.1fs",
                        sent, total, 100.0 * sent / total, now - t_start)
            next_log = now + 1.0
    logging.info("fw-update: DATA complete")

    # VERIFY
    waiter.clear(reply_id)
    rpc.call(int(p.RpcMethod.PLATFORM_FW_VERIFY), rpc_args.encode_platform_fw_verify(crc))
    reply = waiter.wait(reply_id, expected_dlc=8, timeout=_FW_REPLY_TIMEOUT_S)
    if reply is None:
        logging.error("fw-update: VERIFY — no reply from the Platform Teensy")
        return 1
    _, status, _, detail = rpc_args.decode_platform_fw_reply(reply)
    if status != rpc_args.PLATFORM_FW_STATUS_OK:
        _log_platform_fw_refusal("VERIFY", status, detail, crc=crc)
        return 1
    logging.info("fw-update: VERIFY OK — crc32=0x%08X — COMMIT armed", crc)
    if not commit:
        # --verify-only: everything but the one-way step has now run on the real
        # board — the CAN transport, the relay, the staging erase/write/read-back
        # and the CRC + identity check. The board drops the staged image on its
        # own 60 s abandoned-session timeout; nothing was written to the program
        # region.
        logging.info("fw-update: --verify-only — stopping before COMMIT; the board "
                     "discards the staged image after its 60 s session timeout")
        return 0

    # COMMIT
    waiter.clear(reply_id)
    rpc.call(int(p.RpcMethod.PLATFORM_FW_COMMIT), rpc_args.encode_platform_fw_commit())
    reply = waiter.wait(reply_id, expected_dlc=8, timeout=_FW_REPLY_TIMEOUT_S)
    if reply is None:
        logging.error("fw-update: COMMIT — no reply from the Platform Teensy")
        return 1
    _, status, _, _ = rpc_args.decode_platform_fw_reply(reply)
    if status != rpc_args.PLATFORM_FW_STATUS_OK:
        _log_platform_fw_refusal("COMMIT", status, 0)
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


def run_fw_update(teensy_ip: str, hex_path, dry_run: bool, verbose: bool,
                  bind_host: str = "0.0.0.0", verify_only: bool = False,
                  allow_pio: bool = False) -> int:
    _setup_logging(verbose)
    if ".pio" in Path(hex_path).parts and not allow_pio:
        logging.error(
            "fw-update: refused — %s is a PlatformIO build. The pio image of this "
            "sketch was CAN-MUTE on the hardware (platformio.ini, 2026-07-31) and "
            "the USB port is gone, so a CAN-mute image cannot be recovered. Use "
            "the .hex the Arduino IDE exports, or pass --allow-pio-image for an "
            "image that has been proven on a bench board.", hex_path)
        return 1
    logging.info("fw-update: loading %s", hex_path)
    try:
        image, crc = _prepare_image(hex_path)
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
        if hb.flags & int(p.HeartbeatT2JFlags.MPC_ACTIVE):
            logging.error(
                "fw-update: refused — the setpoint output is armed (MPC_ACTIVE); "
                "stow the legs / disarm before flashing the Platform Teensy"
            )
            return 1

        rpc = RpcClient(client, default_timeout=0.5, default_retries=1)
        waiter = PlatformFrameWaiter(client)
        reply_id = pc.CAN_ID_PLATFORM_FW_UPDATE_REPLY
        try:
            old_version = _read_platform_fw_version(rpc, waiter)
            logging.info("fw-update: Platform FW version before update: %s",
                        old_version if old_version is not None else "unknown")

            exit_code = _run_fw_update_session(rpc, waiter, reply_id, image, crc,
                                               commit=not verify_only)

            if exit_code == 0 and not verify_only:
                logging.info("fw-update: COMMIT sent — waiting for the board to reboot")
                time.sleep(3.0)
                new_version = _read_platform_fw_version(rpc, waiter, timeout=2.0)
                logging.info(
                    "fw-update: Platform FW version: %s -> %s",
                    old_version if old_version is not None else "unknown",
                    new_version if new_version is not None else "unknown",
                )
            return exit_code
        except RpcTimeout as e:
            logging.error("fw-update: RPC timed out: %s", e)
            return 1
        except RpcError as e:
            if e.status == int(p.RpcStatus.ERR_REJECTED):
                logging.error(
                    "fw-update: refused — the bridge's setpoint output is armed "
                    "(ERR_REJECTED); stow the legs / disarm before flashing"
                )
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
                        help="Update the Platform Teensy firmware over CAN from HEX_PATH "
                             "(the .hex the Arduino IDE exports — never the .pio build, "
                             "see --allow-pio-image) and exit; see --dry-run and "
                             "--verify-only. The daemon loop above does not run in "
                             "this mode.")
    parser.add_argument("--dry-run", action="store_true",
                        help="With --fw-update: parse + verify identity/CRC only, "
                             "don't touch the board")
    parser.add_argument("--verify-only", action="store_true",
                        help="With --fw-update: run BEGIN, DATA and VERIFY on the real "
                             "board but never COMMIT — proves the whole path except "
                             "the one-way copy; the board discards the staged image "
                             "after 60 s")
    parser.add_argument("--allow-pio-image", action="store_true",
                        help="With --fw-update: permit a HEX_PATH under a .pio/ build "
                             "directory (refused by default: that image family was "
                             "CAN-mute on this board)")
    args = parser.parse_args()

    # Graceful Ctrl-C without a stack trace
    signal.signal(signal.SIGINT, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))

    if args.fw_update is not None:
        return run_fw_update(args.teensy_ip, args.fw_update, args.dry_run,
                             args.verbose, args.bind_host,
                             verify_only=args.verify_only,
                             allow_pio=args.allow_pio_image)

    return run(args.teensy_ip, args.duration, args.verbose, args.bind_host)


if __name__ == "__main__":
    sys.exit(main())
