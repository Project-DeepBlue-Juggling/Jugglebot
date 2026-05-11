"""ZMQ corruption-test harness — real ZMQ + real msgpack at the send boundary.

Plan 2 Phase 6 (Tier 2c) — replaces the in-process ``FakeIPC`` stub used by
``tests/sim/test_zmq_target.py`` with a real ``zmq.PUB`` socket connected to
a real ``MpcTargetIPC`` / ``HardwarePlant._sub`` consumer.  Lets us drive
byte-level corruption (truncated frames, byte flips, schema skew) through
the production serializer + transport.

Two harness classes:

* ``ZmqTargetPubHarness`` — paired with :class:`MpcTargetIPC` on :5558.
  Drives the corruption tests against the MPC target SUB consumer
  (``ZmqTargetSource.poll`` → ``MpcTargetIPC.recv_all`` → ``_unpack`` →
  ``msgpack.unpackb``).
* ``ZmqTelemetryPubHarness`` — paired with :class:`HardwarePlant._sub`
  on :5556.  Drives the connection-drop test against Phase 5's
  telemetry-stale watchdog cascade (real-ZMQ end-to-end equivalent of
  the mocked-time + drain-pump driver used by Phase 5).

Both harnesses bind to ephemeral ports (``bind('tcp://127.0.0.1:0')`` +
``getsockname()`` to read back the OS-assigned port).  Ephemeral ports
let multiple test workers run in parallel without 5556/5558 collisions
and avoid colliding with any live ``mpc_bridge_node`` / ``motor_guard``
on the development workstation.

Lifecycle discipline (Plan 2 Working Note #4):
  * Each harness owns its own ``zmq.Context``; ``close()`` calls
    ``socket.close()`` then ``ctx.term()`` deterministically.
  * The ``pub_only`` / ``pub_and_consumer`` fixtures wrap the harness
    in ``try / finally`` so cleanup runs even if the test raises.
  * ``zmq.LINGER=0`` is set on every socket so ``term()`` cannot hang.
  * A small post-bind settle delay (~50 ms) gives the SUB time to
    complete its connect handshake before the first ``send_multipart``;
    without this, PUB messages emitted before the SUB is attached are
    silently dropped by ZMQ (slow-joiner).

Boundary discipline (Plan 2 Working Note #1):
  * ``corrupt_send(mutation_fn)`` mutates the msgpack-encoded payload
    bytes *between* ``msgpack.packb()`` and ``socket.send_multipart()`` —
    the boundary the bridge would write through in production.  No
    patching of ``msgpack.unpackb`` or any consumer-side decoder.
"""

from __future__ import annotations

import time
from typing import Any, Callable

import msgpack
import zmq

from jugglebot.motion.ipc import (
    MpcTargetIPC,
    TOPIC_MPC_TARGET,
    TOPIC_TELEMETRY,
)


# Post-bind settle window.  ZMQ PUB → SUB has a slow-joiner window
# during which the SUB's subscription filter hasn't reached the PUB; any
# send_multipart() during that window is silently dropped.  50 ms is
# generous on loopback; tests still complete in well under 1 s each.
_BIND_SETTLE_S = 0.05

# Default recv timeout for the polling helper.  Loopback PUB → SUB on
# the same host usually delivers in <10 ms; 0.5 s is the upper bound
# we'll tolerate before declaring a frame lost.
_DEFAULT_RECV_TIMEOUT_S = 0.5


def _ephemeral_addr() -> str:
    """Return a ``tcp://127.0.0.1:0`` address.

    ZMQ resolves the 0 port to an OS-assigned ephemeral port at bind
    time.  The caller reads the actual port back via
    ``sock.getsockopt_string(zmq.LAST_ENDPOINT)``.
    """
    return 'tcp://127.0.0.1:0'


def _resolve_endpoint(sock: zmq.Socket) -> str:
    """Read back the actual endpoint after binding to port 0."""
    # LAST_ENDPOINT returns e.g. b'tcp://127.0.0.1:38291'
    return sock.getsockopt_string(zmq.LAST_ENDPOINT)


# ---------------------------------------------------------------------------
# Target PUB harness — drives MpcTargetIPC :5558 consumer
# ---------------------------------------------------------------------------

class ZmqTargetPubHarness:
    """Real ZMQ PUB on an ephemeral port, paired with a real ``MpcTargetIPC``.

    Mirrors what ``mpc_bridge_node`` does in production: pack a target
    dict with msgpack, send multipart ``[topic, payload]`` on a PUB
    socket.  Unlike the bridge node, this harness lets the test mutate
    the payload bytes between pack and send via ``corrupt_send``.

    Usage::

        with ZmqTargetPubHarness() as h:
            h.send_valid(make_mpc_target([0, 0, 200, 0, 0, 0]))
            msgs = h.consumer.recv_all()
            # or
            h.corrupt_send(make_mpc_target([0, 0, 200, 0, 0, 0]),
                           mutation_fn=lambda b: b[:len(b)//2])
    """

    def __init__(self):
        self._ctx = zmq.Context()
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.setsockopt(zmq.LINGER, 0)
        self._pub.bind(_ephemeral_addr())
        self.addr = _resolve_endpoint(self._pub)
        # Construct the real consumer pointing at the same ephemeral port.
        # MpcTargetIPC creates its own context + sockets — we keep both
        # alive for the lifetime of the harness.
        self.consumer = MpcTargetIPC(self.addr)
        # Post-bind settle so the SUB's subscription handshake completes
        # before the first send_multipart.
        time.sleep(_BIND_SETTLE_S)

    # ------------------------------------------------------------------
    # Send helpers
    # ------------------------------------------------------------------

    def send_valid(
        self,
        msg: dict,
        topic: bytes = TOPIC_MPC_TARGET,
    ) -> bytes:
        """Send a well-formed msgpack frame; return the encoded payload bytes."""
        payload = msgpack.packb(msg, use_bin_type=True)
        self._pub.send_multipart([topic, payload], flags=zmq.NOBLOCK)
        return payload

    def corrupt_send(
        self,
        msg: dict,
        mutation_fn: Callable[[bytes], bytes],
        topic: bytes = TOPIC_MPC_TARGET,
    ) -> bytes:
        """Pack ``msg`` to msgpack bytes, mutate, then send.

        ``mutation_fn`` receives the valid msgpack bytes and returns the
        bytes actually transmitted.  Returns the mutated bytes for
        test-side assertions.
        """
        valid = msgpack.packb(msg, use_bin_type=True)
        mutated = mutation_fn(valid)
        self._pub.send_multipart([topic, mutated], flags=zmq.NOBLOCK)
        return mutated

    def send_raw(
        self,
        payload: bytes,
        topic: bytes = TOPIC_MPC_TARGET,
    ) -> None:
        """Send arbitrary bytes as the payload frame (no packing)."""
        self._pub.send_multipart([topic, payload], flags=zmq.NOBLOCK)

    # ------------------------------------------------------------------
    # Recv helper
    # ------------------------------------------------------------------

    def recv_all_until(
        self,
        timeout_s: float = _DEFAULT_RECV_TIMEOUT_S,
    ) -> list[tuple[bytes, dict]]:
        """Poll ``consumer.recv_all()`` until something arrives or timeout.

        Returns the accumulated list.  Empty list on timeout.  Each call
        ``recv_all()`` itself is non-blocking; this helper retries on a
        short sleep until a message is delivered or the budget elapses.
        Re-raises any exception from ``recv_all()``.
        """
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < timeout_s:
            msgs = self.consumer.recv_all()
            if msgs:
                return msgs
            time.sleep(0.005)
        return []

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def close(self) -> None:
        """Close both consumer and publisher; terminate the context.

        Idempotent.  Safe to call from a ``finally`` block on a test
        that has already raised.
        """
        try:
            self.consumer.close()
        except Exception:
            pass
        try:
            self._pub.close()
        except Exception:
            pass
        try:
            self._ctx.term()
        except Exception:
            pass

    def __enter__(self) -> ZmqTargetPubHarness:
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()


# ---------------------------------------------------------------------------
# Telemetry PUB harness — drives HardwarePlant._sub :5556 consumer
# ---------------------------------------------------------------------------

class ZmqTelemetryPubHarness:
    """Real ZMQ PUB on an ephemeral port, paired with a real ``HardwarePlant``.

    Used for T-U-T2c-5 — drives Phase 5's telemetry-stale cascade through
    a real publisher-close, instead of the mocked time + drain-pump
    Phase 5 used.  The harness binds the telemetry PUB; the consumer is
    the production ``HardwarePlant`` constructed with ``telemetry_addr``
    pointing at the harness's ephemeral port.

    The harness owns the PUB only.  The consumer (``HardwarePlant``) is
    passed in by the test fixture; the harness exposes ``close_pub()``
    so tests can drive a real publisher-death without taking down the
    consumer.

    Usage::

        h = ZmqTelemetryPubHarness()
        plant = build_hardware_plant_stub(
            telemetry_addr=h.addr, mpc_command_addr=...,
        )
        h.send_valid_telemetry(make_telemetry(...))
        # Pump consumer's recv side
        state = plant.get_state()
        # Kill the publisher; cascade fires once telem_age > thresholds
        h.close_pub()
    """

    def __init__(self):
        self._ctx = zmq.Context()
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.setsockopt(zmq.LINGER, 0)
        self._pub.bind(_ephemeral_addr())
        self.addr = _resolve_endpoint(self._pub)
        self._pub_closed = False
        # No consumer constructed here — tests inject HardwarePlant via
        # the existing _hardware_plant_stub factory pointing at self.addr.
        # We still need a settle window after the consumer connects;
        # tests must call ``settle()`` after stub construction.

    def settle(self, settle_s: float = _BIND_SETTLE_S) -> None:
        """Sleep to let SUB complete connect handshake."""
        time.sleep(settle_s)

    def send_valid_telemetry(self, msg: dict) -> bytes:
        """Send a valid telemetry frame.  Returns the encoded payload."""
        if self._pub_closed:
            raise RuntimeError(
                'send_valid_telemetry called after close_pub()')
        payload = msgpack.packb(msg, use_bin_type=True)
        self._pub.send_multipart(
            [TOPIC_TELEMETRY, payload], flags=zmq.NOBLOCK)
        return payload

    def close_pub(self) -> None:
        """Close the PUB socket — simulates publisher death.

        After this call the consumer's SUB sees no more frames; its
        ``recv_multipart(flags=NOBLOCK)`` returns ``zmq.Again`` on every
        subsequent call.  The consumer's stale-watchdog should then
        fire based on ``time.perf_counter() - _last_telem_recv_time``.
        """
        if self._pub_closed:
            return
        self._pub.close()
        self._pub_closed = True

    def close(self) -> None:
        """Close PUB (if still open) + context.  Idempotent."""
        try:
            self.close_pub()
        except Exception:
            pass
        try:
            self._ctx.term()
        except Exception:
            pass

    def __enter__(self) -> ZmqTelemetryPubHarness:
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()


# ---------------------------------------------------------------------------
# Stock mutation_fn helpers
# ---------------------------------------------------------------------------

def truncate_half(payload: bytes) -> bytes:
    """Mutation: keep only the first 50% of bytes."""
    return payload[:len(payload) // 2]


def truncate_to(n: int) -> Callable[[bytes], bytes]:
    """Mutation factory: keep only the first ``n`` bytes."""
    def _truncate(payload: bytes) -> bytes:
        return payload[:n]
    return _truncate


def flip_byte_at(index: int, xor_mask: int = 0xFF) -> Callable[[bytes], bytes]:
    """Mutation factory: XOR the byte at ``index`` with ``xor_mask``.

    Use with care: an index in the bin8/str8 length-prefix region may
    not flip *content* (it may flip the length, producing a different
    structural error).  Tests should target known content offsets.
    """
    def _flip(payload: bytes) -> bytes:
        if index >= len(payload):
            raise IndexError(
                f'flip_byte_at({index}) > len(payload)={len(payload)}')
        buf = bytearray(payload)
        buf[index] ^= xor_mask
        return bytes(buf)
    return _flip


def flip_mid_byte(xor_mask: int = 0xFF) -> Callable[[bytes], bytes]:
    """Mutation factory: flip the byte at len(payload)//2."""
    def _flip(payload: bytes) -> bytes:
        buf = bytearray(payload)
        mid = len(buf) // 2
        buf[mid] ^= xor_mask
        return bytes(buf)
    return _flip


def append_garbage(n: int = 16) -> Callable[[bytes], bytes]:
    """Mutation factory: append ``n`` 0xFF bytes after a valid frame."""
    def _append(payload: bytes) -> bytes:
        return payload + (b'\xff' * n)
    return _append


def prepend_garbage(n: int = 16) -> Callable[[bytes], bytes]:
    """Mutation factory: prepend ``n`` 0xFF bytes before a valid frame."""
    def _prepend(payload: bytes) -> bytes:
        return (b'\xff' * n) + payload
    return _prepend
