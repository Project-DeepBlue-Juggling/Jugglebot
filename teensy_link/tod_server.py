"""Time-of-day RPC handler — the wall-clock anchor side of ADR-0008.

The can-bridge Teensy is the time-sync **master**; the Jetson is the
wall-clock **anchor**. At boot and every ~30 s thereafter the Teensy sends an
``RpcMethod.TIME_OF_DAY_QUERY`` to the Jetson over UDP; the response carries
``CLOCK_REALTIME`` in microseconds. The Teensy uses it to seed and drift-
correct its broadcast wall-clock.

The response payload layout matches the firmware's
``Rpc::ResultTimeOfDay`` in ``rpc.h``:

    struct ResultTimeOfDay { uint64_t jetson_wall_us; };

i.e. a single little-endian uint64.

**What goes in that field (P2, 'plans/archived/2026-08-15
bridge-temporal-trustworthiness.md' — arc closed 2026-08-15,
logbook/2026-08-15-fw14-validated-arc-closed.md).**
The query leaves the Teensy at ``t1``, hits the Jetson kernel at ``t2``, is
stamped at ``ts``, is sent back at ``t3``, lands at ``t4``. With
``df = t2 − t1``, ``p = t3 − t2``, ``dr = t4 − t3``, the firmware anchors to
``stamp + rtt/2`` (``time_sync_master.cpp::on_tod_response``), so

    anchor error = (df − dr)/2 + (ts − (t2 + t3)/2)

Stamping at userspace processing time (``ts ≈ t3``, the pre-P2 behaviour) leaves
``+p/2``; stamping at kernel RX alone (``ts = t2``) leaves ``−p/2`` — the
scheduling jitter does not vanish, it changes sign. Returning the **midpoint**
``(t2 + t3)/2`` makes ``ts`` equal the midpoint exactly, ``p`` cancels, and the
residual is pure path asymmetry ``(df − dr)/2`` (what min-RTT anchor gating is
for, later in that plan).

This is wire-compatible: same field, same layout, no firmware change, no
``PROTOCOL_VERSION`` bump. When no kernel timestamp is available the server
returns ``t3`` exactly as before — and says so via :attr:`~TimeOfDayServer.stamp_mode`,
because a *silent* fallback would restore the ``+p/2`` bias invisibly.
"""

from __future__ import annotations

import logging
import struct
import time
from typing import Optional, Tuple

from . import protocol as p
from .client import Address
from .rpc import RpcServer

logger = logging.getLogger(__name__)

_RESULT_TIMEOFDAY_FMT = "<Q"  # uint64_t jetson_wall_us
_RESULT_TIMEOFDAY_SIZE = 8

# Stamping-mode vocabulary — the observable half of the P2 fallback. Published
# on /link_status by teensy_bridge_node, so a bag says which stamp the anchor
# was built from.
STAMP_MODE_UNKNOWN = "unknown"          # no query handled yet
STAMP_MODE_KERNEL = "kernel-midpoint"   # (t2 + t3)/2, p cancels
STAMP_MODE_USERSPACE = "userspace"      # t3 only — the pre-P2 behaviour

# Plausibility bound on t3 − t2. A kernel RX stamp should precede the pre-send
# userspace stamp by microseconds-to-milliseconds. Anything outside (0, 1 s]
# means the pair is not a coherent (rx, send) measurement — a NTP step between
# the two reads, a datagram that sat queued through a stall, or a mis-parsed
# cmsg. Averaging those would corrupt the anchor by half the error, so the
# server falls back to t3 (today's behaviour) and REPORTS it: the mode reads
# 'userspace' and implausible_rx_stamps counts.
_MAX_RX_TO_SEND_US = 1_000_000


class TimeOfDayServer:
    """Auto-handler for :data:`RpcMethod.TIME_OF_DAY_QUERY`.

    Constructed with an :class:`RpcServer`; registers itself on init and can
    be removed with :meth:`close`.

    Example::

        link = TeensyLinkClient(); link.start()
        rpc_server = RpcServer(link)
        tod = TimeOfDayServer(rpc_server)   # registered, no further setup
        # ...
        tod.close(); rpc_server.close(); link.stop()
    """

    def __init__(self, rpc_server: RpcServer, clock_fn=None):
        self._rpc_server = rpc_server
        # Allow injecting a clock for tests. ``time.time()`` is the production
        # source; tests can pass a callable returning microseconds-since-epoch.
        # This is t3 — and it MUST stay on the same clock domain as the kernel's
        # t2, which is CLOCK_REALTIME (SO_TIMESTAMPNS delivers a struct timespec
        # on CLOCK_REALTIME). ``time.time()`` is CLOCK_REALTIME, so the two are
        # directly averageable; a monotonic clock here would be a category error.
        self._clock_fn = clock_fn if clock_fn is not None else self._default_clock
        self._call_count = 0
        self._stamp_mode = STAMP_MODE_UNKNOWN
        self._implausible_rx_stamps = 0
        # wants_rx_stamp=True: this is the one handler that needs the kernel RX
        # time of its own request datagram.
        rpc_server.register(int(p.RpcMethod.TIME_OF_DAY_QUERY), self._handle,
                            wants_rx_stamp=True)

    @staticmethod
    def _default_clock() -> int:
        return int(time.time() * 1_000_000)

    def close(self) -> None:
        self._rpc_server.unregister(int(p.RpcMethod.TIME_OF_DAY_QUERY))

    @property
    def call_count(self) -> int:
        """Number of TIME_OF_DAY_QUERY requests handled since construction."""
        return self._call_count

    @property
    def stamp_mode(self) -> str:
        """How the LAST answered query was stamped — the bag-visible fallback flag.

        :data:`STAMP_MODE_KERNEL` (``'kernel-midpoint'``),
        :data:`STAMP_MODE_USERSPACE` (``'userspace'``), or
        :data:`STAMP_MODE_UNKNOWN` (``'unknown'``) before the first query. The
        Teensy queries at boot and every ``TIMEOFDAY_RESYNC_MS`` (~30 s), so on a
        live link this resolves within one resync period.
        """
        return self._stamp_mode

    @property
    def implausible_rx_stamps(self) -> int:
        """Kernel stamps rejected by the ``(0, _MAX_RX_TO_SEND_US]`` sanity bound."""
        return self._implausible_rx_stamps

    def _handle(self, req_id: int, args: bytes, addr: Address,
                rx_wall_us: Optional[int] = None) -> Tuple[int, bytes]:
        # The guard runs against a PROVISIONAL clock read; the final t3 is
        # re-read just before packing, so mode bookkeeping and logging sit
        # INSIDE the cancelled (t2..t3) window. Only pack + encode + sendto
        # remain between the final read and the wire — that residual (~tens of
        # µs, unbounded under preemption) is NOT cancelled and appears in the
        # firmware's anchor as -w/2.
        t3_probe = int(self._clock_fn())
        t2 = None if rx_wall_us is None else int(rx_wall_us)
        if t2 is not None and not (0 < t2 <= t3_probe
                                   and (t3_probe - t2) <= _MAX_RX_TO_SEND_US):
            self._implausible_rx_stamps += 1
            logger.warning(
                "TIME_OF_DAY_QUERY: implausible kernel RX stamp (t2=%d, t3=%d, "
                "delta=%d us) — falling back to the userspace stamp",
                t2, t3_probe, t3_probe - t2,
            )
            t2 = None
        mode = STAMP_MODE_USERSPACE if t2 is None else STAMP_MODE_KERNEL
        if mode != self._stamp_mode:
            # Transition-only (first query, or a degrade mid-session) — never per
            # query, and never silent.
            logger.info("TIME_OF_DAY stamp mode: %s -> %s", self._stamp_mode, mode)
            self._stamp_mode = mode
        self._call_count += 1
        logger.debug("TIME_OF_DAY_QUERY req_id=%d from %s (%s)", req_id, addr, mode)
        # Final stamp. Integer floor of the midpoint; the field is uint64 µs and
        # the discarded half-microsecond is far below the link's noise floor.
        t3 = int(self._clock_fn())
        wall_us = t3 if t2 is None else (t2 + t3) // 2
        return int(p.RpcStatus.OK), struct.pack(_RESULT_TIMEOFDAY_FMT, wall_us)
