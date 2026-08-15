"""Core UDP client for the Jetson↔can-bridge link.

:class:`TeensyLinkClient` owns the two UDP sockets (the STREAM port for the
high-rate push streams and the RPC port for request/response), runs a single
receive thread that polls both with ``select.select``, decodes each frame, and
dispatches typed events to registered callbacks.

It is **transport-only** — frame-type-specific behaviour (TIME_OF_DAY_QUERY
responses, heartbeat reply, telemetry → ROS topic, etc.) is the caller's
responsibility. The two upper-layer helpers in this package
(:class:`teensy_link.rpc.RpcClient` for outgoing RPCs and
:class:`teensy_link.tod_server.TimeOfDayServer` for incoming RPCs)
are built on top.

Threading model: one RX thread (``_rx_loop``) + optional one heartbeat thread
(spawned by :meth:`start_heartbeat`). UDP ``sendto`` is thread-safe so
``send_*`` methods are callable from any context.

Callback contract: subscribers are invoked **on the RX thread**. Keep them
short — do not block. For heavy work (e.g. ROS publishes), enqueue and process
on a dedicated thread.

Kernel RX timestamping (P2 of ``plans/archived/2026-08-15
bridge-temporal-trustworthiness.md``, arc closed 2026-08-15 —
``logbook/2026-08-15-fw14-validated-arc-closed.md``):
the **RPC socket only** is opened with ``SO_TIMESTAMPNS`` so each received frame
carries the kernel's CLOCK_REALTIME receive time ``t2`` in ancillary data. That
``t2`` is handed to timestamp-aware subscribers (``subscribe(..,
with_rx_stamp=True)``) as a fifth argument, and is what lets
:class:`teensy_link.tod_server.TimeOfDayServer` answer a TIME_OF_DAY_QUERY with
the midpoint ``(t2 + t3)/2`` instead of the userspace-only ``t3`` — which
cancels the server's processing delay out of the Teensy's ``stamp + rtt/2``
anchor. The STREAM socket keeps the plain ``recvfrom`` path: it pays nothing.
"""

from __future__ import annotations

import logging
import select
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

from . import protocol as p

logger = logging.getLogger(__name__)

Address = Tuple[str, int]
FrameCallback = Callable[[int, int, bytes, Address], None]
"""``(msg_type, seq, payload, from_addr)`` — invoked on RX thread."""

StampedFrameCallback = Callable[[int, int, bytes, Address, Optional[int]], None]
"""``(msg_type, seq, payload, from_addr, rx_wall_us)`` — the opt-in variant.

``rx_wall_us`` is the kernel's CLOCK_REALTIME receive time in microseconds, or
``None`` when the frame arrived without one (STREAM socket, unsupported
platform, or a degraded probe). Register with
``subscribe(msg_type, cb, with_rx_stamp=True)``.
"""


# ── Kernel RX timestamping constants ──────────────────────────────────────
# Python 3.8.10 (this Jetson's system interpreter, which runs ros_ws/) does NOT
# name SO_TIMESTAMPNS / SCM_TIMESTAMPNS in its ``socket`` module — measured
# 2026-08-11, and the 3.9 on this box is the same. So the numeric constant is
# the fallback, taken from /usr/include/asm-generic/socket.h where
# ``SO_TIMESTAMPNS`` resolves to ``SO_TIMESTAMPNS_OLD = 35`` for a 64-bit
# ``time_t``, and ``SCM_TIMESTAMPNS == SO_TIMESTAMPNS``. ``getattr`` first so a
# future interpreter that DOES name it wins over our hard-coded number.
#
# The constant being right is NOT proof the mechanism works — the setsockopt can
# succeed on a kernel that never delivers the cmsg. Hence the runtime probe in
# _drain_socket: the mode only becomes "kernel" once a real packet has actually
# carried a parseable timestamp.
SO_TIMESTAMPNS = getattr(socket, "SO_TIMESTAMPNS", 35)
SCM_TIMESTAMPNS = getattr(socket, "SCM_TIMESTAMPNS", SO_TIMESTAMPNS)

# struct timespec { time_t tv_sec; long tv_nsec; } — two native words, 16 bytes
# on aarch64/x86-64. A platform where it is not 16 bytes (32-bit ARM) simply
# fails the length check in _parse_rx_timestamp_us and degrades to userspace.
_TIMESPEC = struct.Struct("@qq")

# recvmsg/CMSG_SPACE are POSIX-only — absent on Windows (the sim box runs parts
# of this suite). Probed at import so setup can fall back without raising.
_HAS_RECVMSG = hasattr(socket.socket, "recvmsg") and hasattr(socket, "CMSG_SPACE")
_RX_ANCBUF_SIZE = socket.CMSG_SPACE(_TIMESPEC.size) if _HAS_RECVMSG else 0

# Observable-fallback vocabulary (plan § P2: "log the mode once at startup and
# expose it where a bag can see it"). PROBING = setsockopt accepted, no packet
# has confirmed delivery yet; KERNEL = confirmed on a real packet; USERSPACE =
# unavailable or degraded, i.e. today's pre-P2 behaviour.
RX_STAMP_PROBING = "probing"
RX_STAMP_KERNEL = "kernel"
RX_STAMP_USERSPACE = "userspace"


def _parse_rx_timestamp_us(ancdata) -> Optional[int]:
    """Extract the SCM_TIMESTAMPNS CLOCK_REALTIME stamp (µs) from ancillary data.

    Returns ``None`` when no usable timestamp cmsg is present — which is the
    signal the caller uses to degrade the mode rather than silently stamp in
    userspace.
    """
    for level, ctype, cdata in ancdata:
        if (level == socket.SOL_SOCKET and ctype == SCM_TIMESTAMPNS
                and len(cdata) >= _TIMESPEC.size):
            sec, nsec = _TIMESPEC.unpack_from(cdata)
            return sec * 1_000_000 + nsec // 1000
    return None


@dataclass
class LinkStats:
    """Counters surfaced for diagnostics; updated by the RX/TX paths."""

    rx_frames: int = 0
    rx_bytes: int = 0
    tx_frames: int = 0
    tx_bytes: int = 0
    crc_errors: int = 0
    decode_errors: int = 0
    drain_capped: int = 0    # times a single drain hit the per-wakeup frame cap
    seq_gaps_by_type: Dict[int, int] = field(default_factory=dict)
    rx_count_by_type: Dict[int, int] = field(default_factory=dict)
    last_rx_us: int = 0
    last_t2j_heartbeat_us: int = 0

    def snapshot(self) -> "LinkStats":
        return LinkStats(
            rx_frames=self.rx_frames,
            rx_bytes=self.rx_bytes,
            tx_frames=self.tx_frames,
            tx_bytes=self.tx_bytes,
            crc_errors=self.crc_errors,
            decode_errors=self.decode_errors,
            drain_capped=self.drain_capped,
            seq_gaps_by_type=dict(self.seq_gaps_by_type),
            rx_count_by_type=dict(self.rx_count_by_type),
            last_rx_us=self.last_rx_us,
            last_t2j_heartbeat_us=self.last_t2j_heartbeat_us,
        )


class TeensyLinkClient:
    """UDP transport for the Jetson side of the can-bridge link.

    Args:
        teensy_addr: Peer address (host, port_for_streams). RPC port is
            derived as ``port_for_streams + 1`` unless ``rpc_port`` overrides.
            Default ``(192.168.42.2, 5005)`` per ADR-0007 / the protocol.
        local_bind_stream: Local UDP port to bind for STREAM RX. Default
            ``5005`` (matches what the Teensy sends to). Pass a different port
            in tests to avoid binding the production port.
        local_bind_rpc: Local UDP port to bind for RPC RX. Default ``5006``.
            Tests can override.
        rpc_port: Override the Teensy-side RPC port. Default ``5006``.
        bind_host: Interface to bind on. Default ``"0.0.0.0"`` (all
            interfaces). Set to ``"127.0.0.1"`` in tests.
    """

    def __init__(
        self,
        teensy_addr: Address = None,  # type: ignore[assignment]
        local_bind_stream: int = p.PORT_STREAM,
        local_bind_rpc: int = p.PORT_RPC,
        rpc_port: Optional[int] = None,
        bind_host: str = "0.0.0.0",
    ):
        if teensy_addr is None:
            teensy_addr = (p.TEENSY_IP, p.PORT_STREAM)
        peer_host, peer_stream_port = teensy_addr
        self._peer_stream: Address = (peer_host, peer_stream_port)
        self._peer_rpc: Address = (peer_host, rpc_port if rpc_port is not None else p.PORT_RPC)
        self._bind_host = bind_host
        self._bind_stream = local_bind_stream
        self._bind_rpc = local_bind_rpc

        self._stream_sock: Optional[socket.socket] = None
        self._rpc_sock: Optional[socket.socket] = None

        # seq counters are per-direction. Wire wraps at 16 bits.
        self._tx_seq_stream = 0
        self._tx_seq_rpc = 0

        # Per-message-type last-rx-seq for gap detection.
        self._last_rx_seq: Dict[int, int] = {}

        # Callbacks: list of (callback, wants_rx_stamp) per msg_type. A separate
        # "wildcard" list catches all frames (used by diagnostics) and is always
        # called with the plain 4-argument contract.
        self._callbacks: Dict[int, List[Tuple[FrameCallback, bool]]] = {}
        self._wildcard_callbacks: List[FrameCallback] = []
        self._cb_lock = threading.Lock()

        # Kernel RX timestamping state for the RPC socket (see module docstring).
        # Starts USERSPACE — no socket exists yet, so nothing can be stamped.
        self._rx_stamp_mode = RX_STAMP_USERSPACE

        # Threads
        self._rx_thread: Optional[threading.Thread] = None
        self._heartbeat_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # Stats — read-only externally; mutated only on the RX/TX threads.
        self.stats = LinkStats()

    # ── Lifecycle ────────────────────────────────────────────────────────

    def start(self) -> None:
        """Open sockets and start the RX thread. Idempotent."""
        if self._rx_thread is not None and self._rx_thread.is_alive():
            return
        self._stop_event.clear()
        self._open_sockets()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name="teensy_link_rx", daemon=True
        )
        self._rx_thread.start()
        logger.info(
            "TeensyLinkClient started: peer=%s stream_bind=%s:%d rpc_bind=%s:%d",
            self._peer_stream,
            self._bind_host,
            self._bind_stream,
            self._bind_host,
            self._bind_rpc,
        )

    def stop(self, timeout: float = 1.0) -> None:
        """Stop the RX thread, close sockets. Idempotent."""
        self._stop_event.set()
        for t in (self._heartbeat_thread, self._rx_thread):
            if t and t.is_alive():
                t.join(timeout=timeout)
        self._heartbeat_thread = None
        self._rx_thread = None
        self._close_sockets()

    def __enter__(self) -> "TeensyLinkClient":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()

    def _open_sockets(self) -> None:
        self._stream_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._stream_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._stream_sock.bind((self._bind_host, self._bind_stream))
        self._stream_sock.setblocking(False)

        self._rpc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rpc_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rpc_sock.bind((self._bind_host, self._bind_rpc))
        self._rpc_sock.setblocking(False)
        # RPC socket ONLY: ask the kernel to stamp each datagram's receive time.
        # The stream socket does not pay for this (it has no anchor role).
        self._arm_rx_timestamps(self._rpc_sock)

    def _arm_rx_timestamps(self, sock: socket.socket) -> None:
        """Attempt SO_TIMESTAMPNS on ``sock``; set the startup mode and log it once.

        Success here means only that the option was ACCEPTED. Confirmation that
        a timestamp actually arrives is the first-packet probe in
        :meth:`_drain_socket`, which is what moves the mode PROBING → KERNEL (or
        degrades it to USERSPACE).
        """
        if not _HAS_RECVMSG:
            self._rx_stamp_mode = RX_STAMP_USERSPACE
            logger.info(
                "RPC RX kernel timestamping unavailable (no recvmsg/CMSG_SPACE on "
                "this platform) — TIME_OF_DAY stamps fall back to userspace time"
            )
            return
        try:
            sock.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
        except (OSError, OverflowError) as e:
            self._rx_stamp_mode = RX_STAMP_USERSPACE
            logger.warning(
                "RPC RX kernel timestamping setsockopt(SO_TIMESTAMPNS=%d) failed: %s "
                "— TIME_OF_DAY stamps fall back to userspace time",
                SO_TIMESTAMPNS, e,
            )
            return
        self._rx_stamp_mode = RX_STAMP_PROBING
        logger.info(
            "RPC RX kernel timestamping armed (SO_TIMESTAMPNS=%d); awaiting "
            "first-packet confirmation", SO_TIMESTAMPNS,
        )

    def _close_sockets(self) -> None:
        for s in (self._stream_sock, self._rpc_sock):
            if s is not None:
                try:
                    s.close()
                except OSError:
                    pass
        self._stream_sock = None
        self._rpc_sock = None

    @property
    def stream_sock(self) -> socket.socket:
        if self._stream_sock is None:
            raise RuntimeError("client not started")
        return self._stream_sock

    @property
    def rpc_sock(self) -> socket.socket:
        if self._rpc_sock is None:
            raise RuntimeError("client not started")
        return self._rpc_sock

    @property
    def rx_stamp_mode(self) -> str:
        """Kernel-RX-timestamping mode of the RPC socket — the observable fallback.

        One of :data:`RX_STAMP_PROBING` (armed, not yet confirmed by a real
        packet), :data:`RX_STAMP_KERNEL` (confirmed), :data:`RX_STAMP_USERSPACE`
        (unavailable, refused, or degraded after a packet arrived without a
        timestamp). A silent fallback would reintroduce the ``+p/2`` anchor bias
        invisibly, so this is surfaced rather than inferred.
        """
        return self._rx_stamp_mode

    # ── Sending ──────────────────────────────────────────────────────────

    def send_stream(self, msg_type: int, payload: bytes) -> None:
        """Send a frame on the STREAM port to the Teensy."""
        seq = self._tx_seq_stream
        self._tx_seq_stream = (self._tx_seq_stream + 1) & 0xFFFF
        frame = p.encode_frame(msg_type, seq, payload)
        self.stream_sock.sendto(frame, self._peer_stream)
        self.stats.tx_frames += 1
        self.stats.tx_bytes += len(frame)

    def send_rpc(self, msg_type: int, payload: bytes) -> None:
        """Send a frame on the RPC port to the Teensy."""
        seq = self._tx_seq_rpc
        self._tx_seq_rpc = (self._tx_seq_rpc + 1) & 0xFFFF
        frame = p.encode_frame(msg_type, seq, payload)
        self.rpc_sock.sendto(frame, self._peer_rpc)
        self.stats.tx_frames += 1
        self.stats.tx_bytes += len(frame)

    def send_to(self, sock_kind: str, msg_type: int, seq: int, payload: bytes, dest: Address) -> None:
        """Send a frame to an explicit address. Used mainly for RPC responses.

        ``sock_kind`` is ``"stream"`` or ``"rpc"`` to pick which local socket
        to send from (the source port affects how the peer routes the reply).
        ``seq`` is supplied by the caller (we DON'T bump the tx counter here,
        since this is used for replies that should echo the request).
        """
        sock = self.stream_sock if sock_kind == "stream" else self.rpc_sock
        frame = p.encode_frame(msg_type, seq, payload)
        sock.sendto(frame, dest)
        self.stats.tx_frames += 1
        self.stats.tx_bytes += len(frame)

    def send_heartbeat(self, t_jetson_us: Optional[int] = None, flags: int = 0) -> None:
        """Send a J→T heartbeat. Convenience wrapper around send_stream."""
        if t_jetson_us is None:
            t_jetson_us = int(time.time() * 1_000_000)
        hb = p.HeartbeatJ2T(t_jetson_us=t_jetson_us, flags=flags)
        self.send_stream(int(p.MsgType.HEARTBEAT_J2T), hb.pack())

    def start_heartbeat(self, hz: float = float(p.HEARTBEAT_HZ), flags: int = 0) -> None:
        """Spawn a background thread that sends J→T heartbeats at ``hz``.

        Idempotent — calling twice is a no-op.
        """
        if self._heartbeat_thread is not None and self._heartbeat_thread.is_alive():
            return
        period = 1.0 / hz
        self._heartbeat_flags = flags  # capture for the thread

        def _loop() -> None:
            while not self._stop_event.is_set():
                try:
                    self.send_heartbeat(flags=self._heartbeat_flags)
                except OSError as e:
                    logger.warning("heartbeat send failed: %s", e)
                self._stop_event.wait(period)

        self._heartbeat_thread = threading.Thread(
            target=_loop, name="teensy_link_hb", daemon=True
        )
        self._heartbeat_thread.start()

    def set_heartbeat_flags(self, flags: int) -> None:
        """Update the flag bits the heartbeat thread emits on its next tick."""
        self._heartbeat_flags = flags

    # ── Subscription ─────────────────────────────────────────────────────

    def subscribe(
        self,
        msg_type: int,
        callback: FrameCallback,
        *,
        with_rx_stamp: bool = False,
    ) -> Callable[[], None]:
        """Register a callback for a specific msg_type.

        Returns an unsubscribe function. The callback is invoked on the RX
        thread; keep it fast.

        ``with_rx_stamp=False`` (the default, and every pre-existing caller)
        keeps the 4-argument contract ``(msg_type, seq, payload, addr)``.
        ``with_rx_stamp=True`` opts into the 5-argument
        :data:`StampedFrameCallback` contract, whose extra ``rx_wall_us`` is the
        kernel receive time in µs (or ``None``). The opt-in is explicit rather
        than signature-sniffed so adding the argument can never silently break a
        handler that did not ask for it.
        """
        with self._cb_lock:
            self._callbacks.setdefault(int(msg_type), []).append(
                (callback, bool(with_rx_stamp))
            )

        def _unsub() -> None:
            with self._cb_lock:
                lst = self._callbacks.get(int(msg_type), [])
                for i, (cb, _wants) in enumerate(lst):
                    if cb is callback:
                        del lst[i]
                        return

        return _unsub

    def subscribe_all(self, callback: FrameCallback) -> Callable[[], None]:
        """Register a callback that receives every decoded frame.

        Useful for diagnostics, packet capture, and wildcard logging.
        """
        with self._cb_lock:
            self._wildcard_callbacks.append(callback)

        def _unsub() -> None:
            with self._cb_lock:
                try:
                    self._wildcard_callbacks.remove(callback)
                except ValueError:
                    pass

        return _unsub

    # ── RX loop ──────────────────────────────────────────────────────────

    def _rx_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                ready, _, _ = select.select(
                    [self._stream_sock, self._rpc_sock], [], [], 0.1
                )
            except (OSError, ValueError):
                # Socket(s) closed under us — exit gracefully
                return
            for s in ready:
                self._drain_socket(s)

    # Cap frames drained per socket per select-wakeup so a
    # sustained flood on one socket can't starve the other within a single wakeup
    # (the stream socket could otherwise monopolise the RX loop and delay RPC
    # responses). 256 is ~16× the legitimate downlink rate; overflow is counted
    # (stats.drain_capped) and simply picked up on the next wakeup.
    _MAX_DRAIN_PER_WAKE = 256

    def _drain_socket(self, sock: socket.socket) -> None:
        # Drain pending frames on this socket without blocking, bounded per wakeup.
        #
        # The RPC socket uses recvmsg (same syscall class as recvfrom, one call
        # per datagram) so the kernel's RX timestamp rides along in ancillary
        # data. Hoisted out of the loop: one identity check per wakeup, not per
        # frame. The stream socket stays on recvfrom.
        stamped = (sock is self._rpc_sock
                   and self._rx_stamp_mode in (RX_STAMP_PROBING, RX_STAMP_KERNEL))
        for _ in range(self._MAX_DRAIN_PER_WAKE):
            rx_wall_us: Optional[int] = None
            try:
                if stamped:
                    data, ancdata, _msg_flags, addr = sock.recvmsg(
                        p.MAX_FRAME + 64, _RX_ANCBUF_SIZE
                    )
                    rx_wall_us = _parse_rx_timestamp_us(ancdata)
                    if rx_wall_us is None:
                        # setsockopt was accepted but the kernel is not actually
                        # delivering — degrade for the life of this client. This
                        # is the RUNTIME half of the probe; without it a silent
                        # userspace fallback would look like kernel stamping.
                        stamped = False
                        self._degrade_rx_stamping()
                    elif self._rx_stamp_mode == RX_STAMP_PROBING:
                        self._confirm_rx_stamping()
                else:
                    data, addr = sock.recvfrom(p.MAX_FRAME + 64)
            except BlockingIOError:
                return
            except OSError:
                return
            self.stats.rx_frames += 1
            self.stats.rx_bytes += len(data)
            # Monotonic clock: last_rx_us is one endpoint of the
            # time_since_last_rx_us / _t2j interval reads below. time.time()
            # (CLOCK_REALTIME) is NTP-steppable — a backward step would underflow
            # the age to a huge positive (spurious link-lost) or a negative;
            # time.monotonic() cannot step. The wire-bound anchor
            # (send_heartbeat's t_jetson_us) stays on time.time().
            self.stats.last_rx_us = int(time.monotonic() * 1_000_000)
            try:
                msg_type, seq, payload = p.decode_frame(data)
            except p.CrcError as e:
                # Typed CRC-16 mismatch — a corrupted frame, counted apart from a
                # structural decode error. (CrcError subclasses ValueError, so this
                # clause must precede the ValueError catch below.) Replaces the
                # former fragile `"CRC" in str(e)` string match.
                self.stats.crc_errors += 1
                logger.debug("CRC failure: %s (%d bytes from %s)", e, len(data), addr)
                continue
            except ValueError as e:
                # Structural decode error (bad magic/version/length/short frame).
                self.stats.decode_errors += 1
                logger.debug("decode failure: %s (%d bytes from %s)", e, len(data), addr)
                continue
            self._track_seq(msg_type, seq)
            self.stats.rx_count_by_type[msg_type] = (
                self.stats.rx_count_by_type.get(msg_type, 0) + 1
            )
            if msg_type == int(p.MsgType.HEARTBEAT_T2J):
                self.stats.last_t2j_heartbeat_us = self.stats.last_rx_us
            self._dispatch(msg_type, seq, payload, addr, rx_wall_us)
        else:
            # Loop ran the full cap without a BlockingIOError return — the socket
            # still had frames. Count it (surfaced on [diag]); the remainder is
            # drained on the next select wakeup (bounded, never a hard stall).
            self.stats.drain_capped += 1

    def _track_seq(self, msg_type: int, seq: int) -> None:
        prev = self._last_rx_seq.get(msg_type)
        self._last_rx_seq[msg_type] = seq
        if prev is None:
            return
        expected = (prev + 1) & 0xFFFF
        if seq != expected:
            # Crude gap measure: count one gap event per out-of-order seq.
            self.stats.seq_gaps_by_type[msg_type] = (
                self.stats.seq_gaps_by_type.get(msg_type, 0) + 1
            )

    def _confirm_rx_stamping(self) -> None:
        """PROBING → KERNEL on the first packet that actually carried a stamp.

        Logged ONCE (a state transition, not a per-packet event) — the RX loop
        must not gain per-frame work.
        """
        self._rx_stamp_mode = RX_STAMP_KERNEL
        logger.info(
            "RPC RX kernel timestamping CONFIRMED on first packet — "
            "TIME_OF_DAY stamps use the (t2+t3)/2 midpoint"
        )

    def _degrade_rx_stamping(self) -> None:
        """→ USERSPACE after a real packet arrived with no usable timestamp.

        One-shot: the mode latches, the drain goes back to ``recvfrom``, and the
        warning is emitted once, not per frame.
        """
        if self._rx_stamp_mode == RX_STAMP_USERSPACE:
            return
        self._rx_stamp_mode = RX_STAMP_USERSPACE
        logger.warning(
            "RPC RX kernel timestamping DEGRADED: setsockopt(SO_TIMESTAMPNS=%d) "
            "was accepted but a received packet carried no timestamp — "
            "TIME_OF_DAY stamps fall back to userspace time (anchor regains the "
            "+processing/2 bias)", SO_TIMESTAMPNS,
        )

    def _dispatch(
        self,
        msg_type: int,
        seq: int,
        payload: bytes,
        addr: Address,
        rx_wall_us: Optional[int] = None,
    ) -> None:
        with self._cb_lock:
            type_cbs = list(self._callbacks.get(msg_type, ()))
            wild_cbs = list(self._wildcard_callbacks)
        for cb, wants_stamp in type_cbs:
            try:
                if wants_stamp:
                    cb(msg_type, seq, payload, addr, rx_wall_us)   # type: ignore[call-arg]
                else:
                    cb(msg_type, seq, payload, addr)
            except Exception:
                logger.exception(
                    "callback for msg_type=%d raised; continuing", msg_type
                )
        for cb in wild_cbs:
            try:
                cb(msg_type, seq, payload, addr)
            except Exception:
                logger.exception(
                    "wildcard callback for msg_type=%d raised; continuing", msg_type
                )

    # ── Convenience queries ──────────────────────────────────────────────

    def time_since_last_rx_us(self) -> Optional[int]:
        """Microseconds since any frame was received from the peer, or None."""
        if self.stats.last_rx_us == 0:
            return None
        return int(time.monotonic() * 1_000_000) - self.stats.last_rx_us   # monotonic both endpoints

    def time_since_last_t2j_heartbeat_us(self) -> Optional[int]:
        """Microseconds since the last T→J heartbeat frame, or None."""
        if self.stats.last_t2j_heartbeat_us == 0:
            return None
        return int(time.monotonic() * 1_000_000) - self.stats.last_t2j_heartbeat_us   # monotonic both endpoints
