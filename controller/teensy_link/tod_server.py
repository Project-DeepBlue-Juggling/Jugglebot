"""Time-of-day RPC handler — the wall-clock anchor side of ADR-0008.

The leg-bridge Teensy is the time-sync **master**; the Jetson is the
wall-clock **anchor**. At boot and every ~30 s thereafter the Teensy sends an
``RpcMethod.TIME_OF_DAY_QUERY`` to the Jetson over UDP; the response carries
``CLOCK_REALTIME`` in microseconds. The Teensy uses it to seed and drift-
correct its broadcast wall-clock.

The response payload layout matches the firmware's
``Rpc::ResultTimeOfDay`` in ``rpc.h``:

    struct ResultTimeOfDay { uint64_t jetson_wall_us; };

i.e. a single little-endian uint64.
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
        self._clock_fn = clock_fn if clock_fn is not None else self._default_clock
        self._call_count = 0
        rpc_server.register(int(p.RpcMethod.TIME_OF_DAY_QUERY), self._handle)

    @staticmethod
    def _default_clock() -> int:
        return int(time.time() * 1_000_000)

    def close(self) -> None:
        self._rpc_server.unregister(int(p.RpcMethod.TIME_OF_DAY_QUERY))

    @property
    def call_count(self) -> int:
        """Number of TIME_OF_DAY_QUERY requests handled since construction."""
        return self._call_count

    def _handle(self, req_id: int, args: bytes, addr: Address) -> Tuple[int, bytes]:
        wall_us = self._clock_fn()
        self._call_count += 1
        logger.debug(
            "TIME_OF_DAY_QUERY req_id=%d from %s -> jetson_wall_us=%d",
            req_id, addr, wall_us,
        )
        return int(p.RpcStatus.OK), struct.pack(_RESULT_TIMEOFDAY_FMT, wall_us)
