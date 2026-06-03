"""RPC layer over the can-bridge link.

Per ADR-0006 the link is UDP throughout; RPC reliability is therefore the
application's responsibility. This module provides two halves of that:

* :class:`RpcClient` — used to call methods *on the Teensy* (gain writes,
  state changes, encoder-search start, SDO read/write, etc.). Synchronous
  ``call()`` blocks the caller until a response arrives or the timeout
  expires; on timeout it retries up to ``retries`` times with the same
  ``req_id``. Idempotence is the firmware's job — both sides correlate by
  ``req_id``.

* :class:`RpcServer` — used to *answer* RPCs initiated by the Teensy. The
  only one defined today is :data:`RpcMethod.TIME_OF_DAY_QUERY` (wall-clock
  anchor, see :class:`controller.teensy_link.tod_server.TimeOfDayServer`).
  The infrastructure is generic so additional inbound methods can be
  registered later without rework.

The wire envelope (8-byte header + variable-length args/result) is defined in
the generated protocol module; this file is the *behaviour* layer on top.
"""

from __future__ import annotations

import logging
import struct
import threading
import time
from typing import Callable, Dict, Optional, Tuple

from . import protocol as p
from .client import TeensyLinkClient, Address

logger = logging.getLogger(__name__)


class RpcError(Exception):
    """Raised when an RPC completes with a non-OK status."""

    def __init__(self, method: int, status: int, message: str = ""):
        self.method = method
        self.status = status
        try:
            status_name = p.RpcStatus(status).name
        except ValueError:
            status_name = f"status_{status}"
        try:
            method_name = p.RpcMethod(method).name
        except ValueError:
            method_name = f"method_{method}"
        super().__init__(f"{method_name}: {status_name}{(' ' + message) if message else ''}")


class RpcTimeout(RpcError):
    """RPC did not receive a response within timeout × retries."""

    def __init__(self, method: int, retries: int, timeout: float):
        super().__init__(method, int(p.RpcStatus.ERR_TIMEOUT), f"after {retries} retries × {timeout:.3f}s")
        self.retries = retries
        self.timeout = timeout


# ── Outgoing RPCs (Jetson → Teensy) ───────────────────────────────────────


class _PendingRpc:
    """Internal: a single in-flight RPC awaiting a response."""

    __slots__ = ("method", "req_id", "event", "response_payload", "response_status")

    def __init__(self, method: int, req_id: int):
        self.method = method
        self.req_id = req_id
        self.event = threading.Event()
        self.response_payload: bytes = b""
        self.response_status: int = 0


class RpcClient:
    """Synchronous RPC client. Thread-safe; multiple callers can ``call()``
    concurrently — each gets a unique ``req_id`` and waits on its own event.

    Construct AFTER :meth:`TeensyLinkClient.start`. The client subscribes to
    ``RPC_RESPONSE`` frames on construction; unsubscribe via :meth:`close`.
    """

    def __init__(self, link: TeensyLinkClient, default_timeout: float = 0.5, default_retries: int = 2):
        self._link = link
        self._default_timeout = default_timeout
        self._default_retries = default_retries
        self._req_id_counter = 0
        self._req_id_lock = threading.Lock()
        # Map (method, req_id) -> _PendingRpc. We key by both because the
        # firmware echoes both fields and we want defence-in-depth against
        # a response for a stale req_id matching a fresh request.
        self._pending: Dict[Tuple[int, int], _PendingRpc] = {}
        self._pending_lock = threading.Lock()
        self._unsubscribe = link.subscribe(
            int(p.MsgType.RPC_RESPONSE), self._on_response
        )

    def close(self) -> None:
        """Detach from the underlying link. Pending calls will time out."""
        if self._unsubscribe is not None:
            self._unsubscribe()
            self._unsubscribe = None

    def __enter__(self) -> "RpcClient":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def _next_req_id(self) -> int:
        with self._req_id_lock:
            self._req_id_counter = (self._req_id_counter + 1) & 0xFFFF
            # Avoid 0 to keep "no req_id" sentinel safely distinct.
            if self._req_id_counter == 0:
                self._req_id_counter = 1
            return self._req_id_counter

    def call(
        self,
        method: int,
        args: bytes = b"",
        *,
        timeout: Optional[float] = None,
        retries: Optional[int] = None,
    ) -> bytes:
        """Send an RPC, block waiting for the response, return the result blob.

        Raises:
            RpcTimeout: no response after timeout × (1 + retries) attempts.
            RpcError: response arrived with non-OK status.
        """
        if timeout is None:
            timeout = self._default_timeout
        if retries is None:
            retries = self._default_retries
        req_id = self._next_req_id()
        pending = _PendingRpc(int(method), req_id)
        key = (pending.method, req_id)
        with self._pending_lock:
            self._pending[key] = pending
        try:
            payload_head = p.RpcRequest(
                method=int(method), req_id=req_id, arg_len=len(args), pad=0
            ).pack()
            packet = payload_head + args
            for attempt in range(retries + 1):
                self._link.send_rpc(int(p.MsgType.RPC_REQUEST), packet)
                if pending.event.wait(timeout):
                    break
                logger.debug(
                    "rpc retry method=%s req_id=%d attempt=%d/%d",
                    p.RpcMethod(method).name if int(method) in {m.value for m in p.RpcMethod} else int(method),
                    req_id,
                    attempt + 1,
                    retries,
                )
            else:
                raise RpcTimeout(int(method), retries, timeout)

            if pending.response_status != int(p.RpcStatus.OK):
                raise RpcError(int(method), pending.response_status)
            return pending.response_payload
        finally:
            with self._pending_lock:
                self._pending.pop(key, None)

    def _on_response(self, msg_type: int, seq: int, payload: bytes, addr: Address) -> None:
        if len(payload) < p.RPC_RESPONSE_SIZE:
            logger.debug("RPC_RESPONSE too short (%d bytes)", len(payload))
            return
        head = p.RpcResponse.unpack(payload[: p.RPC_RESPONSE_SIZE])
        result_blob = payload[p.RPC_RESPONSE_SIZE : p.RPC_RESPONSE_SIZE + head.res_len]
        key = (int(head.method), int(head.req_id))
        with self._pending_lock:
            pending = self._pending.get(key)
        if pending is None:
            logger.debug(
                "stray RPC_RESPONSE method=%d req_id=%d (no pending)",
                head.method, head.req_id,
            )
            return
        pending.response_status = int(head.status)
        pending.response_payload = result_blob
        pending.event.set()


# ── Incoming RPCs (Teensy → Jetson) ───────────────────────────────────────


RpcHandler = Callable[[int, bytes, Address], Tuple[int, bytes]]
"""``(req_id, args, from_addr) -> (status, result_blob)``.

Status is an :class:`RpcStatus` value; the result_blob is method-specific.
The handler must not raise — return ``ERR_REJECTED`` or similar instead.
"""


class RpcServer:
    """Receives RpcRequest frames from the Teensy and dispatches to handlers.

    Used by :class:`TimeOfDayServer` for the wall-clock anchor handshake.
    Register additional handlers via :meth:`register`.

    Construct AFTER :meth:`TeensyLinkClient.start`.
    """

    def __init__(self, link: TeensyLinkClient):
        self._link = link
        self._handlers: Dict[int, RpcHandler] = {}
        self._lock = threading.Lock()
        self._unsubscribe = link.subscribe(
            int(p.MsgType.RPC_REQUEST), self._on_request
        )

    def close(self) -> None:
        if self._unsubscribe is not None:
            self._unsubscribe()
            self._unsubscribe = None

    def __enter__(self) -> "RpcServer":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def register(self, method: int, handler: RpcHandler) -> None:
        """Register a handler for a specific RPC method.

        Overwrites any prior handler for that method.
        """
        with self._lock:
            self._handlers[int(method)] = handler

    def unregister(self, method: int) -> None:
        with self._lock:
            self._handlers.pop(int(method), None)

    def _on_request(self, msg_type: int, seq: int, payload: bytes, addr: Address) -> None:
        if len(payload) < p.RPC_REQUEST_SIZE:
            logger.warning("RPC_REQUEST too short (%d bytes) from %s", len(payload), addr)
            return
        head = p.RpcRequest.unpack(payload[: p.RPC_REQUEST_SIZE])
        args = payload[p.RPC_REQUEST_SIZE : p.RPC_REQUEST_SIZE + head.arg_len]
        with self._lock:
            handler = self._handlers.get(int(head.method))
        if handler is None:
            status = int(p.RpcStatus.ERR_UNKNOWN_METHOD)
            result_blob = b""
        else:
            try:
                status, result_blob = handler(int(head.req_id), args, addr)
            except Exception:
                logger.exception(
                    "RPC handler for method=%d req_id=%d raised; returning ERR_REJECTED",
                    head.method, head.req_id,
                )
                status = int(p.RpcStatus.ERR_REJECTED)
                result_blob = b""
        self._send_response(int(head.method), int(head.req_id), status, result_blob, addr)

    def _send_response(
        self, method: int, req_id: int, status: int, result_blob: bytes, addr: Address
    ) -> None:
        head = p.RpcResponse(
            method=method, req_id=req_id, status=status, res_len=len(result_blob)
        ).pack()
        # Send back to the source addr on the same RPC socket. seq=0 is
        # conventional for responses since the req_id provides correlation.
        self._link.send_to("rpc", int(p.MsgType.RPC_RESPONSE), 0, head + result_blob, addr)
