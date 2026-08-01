"""Shared fixtures: a fake Teensy peer that runs on loopback."""

from __future__ import annotations

import select
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import pytest

from teensy_link import protocol as p
from teensy_link import TeensyLinkClient

Address = Tuple[str, int]


@dataclass
class _RxItem:
    msg_type: int
    seq: int
    payload: bytes
    from_addr: Address


class FakeTeensy:
    """Loopback-side stand-in for the can-bridge Teensy.

    Binds the *Teensy*-side ports (configurable) and offers a small API to
    inject T→J frames at the Jetson and to inspect the J→T frames it
    receives. Used by tests to exercise :class:`TeensyLinkClient` without
    real hardware.
    """

    def __init__(
        self,
        bind_host: str = "127.0.0.1",
        stream_port: int = 0,  # 0 = let OS pick — caller reads back
        rpc_port: int = 0,
        jetson_addr: Optional[Address] = None,
    ):
        self._bind_host = bind_host
        self._jetson_addr = jetson_addr  # set by caller after Jetson binds
        self.stream_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.stream_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.stream_sock.bind((bind_host, stream_port))
        self.stream_sock.setblocking(False)
        self.stream_port = self.stream_sock.getsockname()[1]

        self.rpc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rpc_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rpc_sock.bind((bind_host, rpc_port))
        self.rpc_sock.setblocking(False)
        self.rpc_port = self.rpc_sock.getsockname()[1]

        self._rx_items: List[_RxItem] = []
        self._rx_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)

        # Auto-respond hooks: method -> callback(req_id, args) -> (status, blob)
        self._rpc_handlers: Dict[int, Callable[[int, bytes], Tuple[int, bytes]]] = {}

        self._tx_seq_stream = 0
        self._tx_seq_rpc = 0

    # ── Lifecycle ─────────────────────────────────────────────────────────

    def start(self) -> None:
        self._thread.start()

    def stop(self, timeout: float = 0.5) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=timeout)
        self.stream_sock.close()
        self.rpc_sock.close()

    def __enter__(self) -> "FakeTeensy":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()

    def set_jetson_addr(self, addr: Address) -> None:
        """Where to send T→J frames. Set after starting the Jetson client."""
        self._jetson_addr = addr

    # ── Inject T→J frames ────────────────────────────────────────────────

    def send_to_jetson(self, msg_type: int, payload: bytes, port: str = "stream") -> int:
        """Send a frame from this fake Teensy to the Jetson. Returns seq used."""
        if self._jetson_addr is None:
            raise RuntimeError("jetson_addr not set")
        host, base_port = self._jetson_addr
        if port == "stream":
            seq = self._tx_seq_stream
            self._tx_seq_stream = (self._tx_seq_stream + 1) & 0xFFFF
            dest = (host, base_port)
            sock = self.stream_sock
        else:
            seq = self._tx_seq_rpc
            self._tx_seq_rpc = (self._tx_seq_rpc + 1) & 0xFFFF
            dest = (host, base_port + 1)
            sock = self.rpc_sock
        frame = p.encode_frame(msg_type, seq, payload)
        sock.sendto(frame, dest)
        return seq

    def send_heartbeat_t2j(self, link_state: int = int(p.LinkState.UP), fault_state: int = 0) -> None:
        hb = p.HeartbeatT2J(
            t_teensy_us=int(time.time() * 1_000_000),
            link_state=link_state,
            bus1_health=int(p.BusHealth.OK),
            bus2_health=int(p.BusHealth.OK),
            fault_state=fault_state,
            flags=0,
            uptime_ms=0,
        )
        self.send_to_jetson(int(p.MsgType.HEARTBEAT_T2J), hb.pack())

    def send_telemetry(self, pos_rev=None, vel_rps=None) -> None:
        if pos_rev is None:
            pos_rev = (0.0,) * p.NUM_AXES
        if vel_rps is None:
            vel_rps = (0.0,) * p.NUM_AXES
        tm = p.Telemetry(
            t_teensy_us=int(time.time() * 1_000_000),
            pos_rev=tuple(pos_rev),
            vel_rps=tuple(vel_rps),
        )
        self.send_to_jetson(int(p.MsgType.TELEMETRY), tm.pack())

    def send_rpc_request(self, method: int, req_id: int, args: bytes = b"") -> int:
        """Initiate an RPC FROM the Teensy (e.g. TIME_OF_DAY_QUERY)."""
        head = p.RpcRequest(method=int(method), req_id=req_id, arg_len=len(args), pad=0).pack()
        return self.send_to_jetson(int(p.MsgType.RPC_REQUEST), head + args, port="rpc")

    # ── Inspect J→T frames ───────────────────────────────────────────────

    def received(self, msg_type: Optional[int] = None) -> List[_RxItem]:
        """All frames the Jetson has sent us so far, optionally filtered."""
        with self._rx_lock:
            items = list(self._rx_items)
        if msg_type is not None:
            items = [i for i in items if i.msg_type == int(msg_type)]
        return items

    def wait_for(self, msg_type: int, count: int = 1, timeout: float = 1.0) -> List[_RxItem]:
        """Block until at least ``count`` frames of ``msg_type`` arrive."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            got = self.received(msg_type)
            if len(got) >= count:
                return got
            time.sleep(0.005)
        return self.received(msg_type)

    def clear_received(self) -> None:
        with self._rx_lock:
            self._rx_items.clear()

    # ── Auto-respond hooks ───────────────────────────────────────────────

    def on_rpc(self, method: int, handler: Callable[[int, bytes], Tuple[int, bytes]]) -> None:
        """Auto-respond to a Jetson-initiated RPC method."""
        self._rpc_handlers[int(method)] = handler

    # ── Internals ─────────────────────────────────────────────────────────

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                ready, _, _ = select.select(
                    [self.stream_sock, self.rpc_sock], [], [], 0.05
                )
            except (OSError, ValueError):
                return
            for s in ready:
                self._drain(s)

    def _drain(self, sock: socket.socket) -> None:
        while True:
            try:
                data, addr = sock.recvfrom(p.MAX_FRAME + 64)
            except BlockingIOError:
                return
            except OSError:
                return
            try:
                msg_type, seq, payload = p.decode_frame(data)
            except ValueError:
                continue
            with self._rx_lock:
                self._rx_items.append(_RxItem(msg_type, seq, payload, addr))
            # Inbound RPC requests from the Jetson — auto-respond if registered
            if msg_type == int(p.MsgType.RPC_REQUEST) and sock is self.rpc_sock:
                self._maybe_respond(payload, addr)

    def _maybe_respond(self, payload: bytes, addr: Address) -> None:
        if len(payload) < p.RPC_REQUEST_SIZE:
            return
        req = p.RpcRequest.unpack(payload[: p.RPC_REQUEST_SIZE])
        args = payload[p.RPC_REQUEST_SIZE : p.RPC_REQUEST_SIZE + req.arg_len]
        handler = self._rpc_handlers.get(int(req.method))
        if handler is None:
            status = int(p.RpcStatus.ERR_UNKNOWN_METHOD)
            blob = b""
        else:
            status, blob = handler(int(req.req_id), args)
        resp_head = p.RpcResponse(
            method=int(req.method), req_id=int(req.req_id),
            status=int(status), res_len=len(blob),
        ).pack()
        frame = p.encode_frame(int(p.MsgType.RPC_RESPONSE), 0, resp_head + blob)
        self.rpc_sock.sendto(frame, addr)


@pytest.fixture
def fake_teensy_and_client():
    """Build a paired (FakeTeensy, TeensyLinkClient) on loopback.

    Both bind to OS-assigned ports so multiple tests can run in parallel.
    The fixture yields ``(teensy, client)`` already started; tears down on exit.
    """
    # Build the FakeTeensy first so we know its ports, then point the client at it.
    teensy = FakeTeensy(bind_host="127.0.0.1", stream_port=0, rpc_port=0)
    teensy.start()
    # Now bring up the Jetson-side client, pointing at the FakeTeensy's ports
    # and binding ITS sockets on different ports.
    client = TeensyLinkClient(
        teensy_addr=("127.0.0.1", teensy.stream_port),
        rpc_port=teensy.rpc_port,
        local_bind_stream=0,
        local_bind_rpc=0,
        bind_host="127.0.0.1",
    )
    client.start()
    # Reveal the client's bound ports to the FakeTeensy so it can send back.
    jetson_stream_port = client._stream_sock.getsockname()[1]
    jetson_rpc_port = client._rpc_sock.getsockname()[1]
    # FakeTeensy uses (host, stream_port) and assumes RPC is at +1, but we
    # have a more general arrangement: tell it both ports explicitly via
    # set_jetson_addr; we'll route stream->stream_port and rpc->rpc_port.
    teensy.set_jetson_addr(("127.0.0.1", jetson_stream_port))
    # Patch the FakeTeensy's send_to_jetson to use the actual ports:
    teensy._jetson_rpc_port = jetson_rpc_port  # type: ignore[attr-defined]

    # Monkey-patch the send routing to use the actual ports
    _orig_send = teensy.send_to_jetson

    def _send(msg_type, payload, port="stream"):
        # bypass the assumed +1 calculation
        if teensy._jetson_addr is None:
            raise RuntimeError("jetson_addr not set")
        host, _ = teensy._jetson_addr
        if port == "stream":
            seq = teensy._tx_seq_stream
            teensy._tx_seq_stream = (teensy._tx_seq_stream + 1) & 0xFFFF
            dest = (host, jetson_stream_port)
            sock = teensy.stream_sock
        else:
            seq = teensy._tx_seq_rpc
            teensy._tx_seq_rpc = (teensy._tx_seq_rpc + 1) & 0xFFFF
            dest = (host, jetson_rpc_port)
            sock = teensy.rpc_sock
        frame = p.encode_frame(msg_type, seq, payload)
        sock.sendto(frame, dest)
        return seq

    teensy.send_to_jetson = _send  # type: ignore[assignment]

    yield teensy, client
    client.stop()
    teensy.stop()
