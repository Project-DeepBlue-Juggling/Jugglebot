"""Shared loopback harness for the teensy_bridge_node test family.

Not collected by pytest (leading underscore, no ``test_`` prefix) — this is a
support module, following the ``tests/sim/_zmq_test_harness.py`` pattern.

Until 2026-08-01 these helpers lived inside
``tests/ros/test_teensy_bridge_node_read.py`` and 21 sibling files imported
them *from that test module*, which made a read-side test file the de-facto
public API of the bridge test suite: renaming or reorganising it broke twenty
unrelated files, and pytest had to import (and collect) the whole read-side
module before any of them could run.  ``_teardown`` had additionally been
copy-pasted 19 times.

**Import this only from ``tests/ros/``.**  ``tests/ros/conftest.py`` installs
the mock ROS 2 modules; ``_build_paired_node`` constructs a real
``TeensyBridgeNode``, so importing this harness from a directory whose conftest
has not run the mock injection would either fail or (worse) bind the real
rclpy.  The ``TeensyBridgeNode`` import is therefore deliberately kept *inside*
the function, not at module scope — the mock must be installed before the node
module is first imported.
"""

from __future__ import annotations

import time

from controller.teensy_link import PlatformFrame
from controller.teensy_link import TeensyLinkClient
from controller.teensy_link import protocol as p

# Reuse the FakeTeensy loopback peer.
from tests.teensy_link.conftest import FakeTeensy


# ── Polling ────────────────────────────────────────────────────

def _wait_until(predicate, timeout=2.0, interval=0.005):
    """Poll ``predicate`` until true or timeout. Returns the final bool."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()


def _poll(cond, timeout=4.0, dt=0.01):
    """Longer-deadline sibling of :func:`_wait_until` (RPC round-trips).

    Kept distinct rather than folded into ``_wait_until``: the six call-site
    files that used this one chose 4.0 s / 10 ms deliberately for paths that
    cross the FakeTeensy RPC server, and quietly halving a timeout is how
    loopback tests become flaky under xdist load.
    """
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(dt)
    return cond()


# ── Node construction / teardown ───────────────────────────────

def _build_paired_node(*, boot_state_read=False):
    """Build a (FakeTeensy, client, node) triple wired on loopback.

    Mirrors the pairing in tests/teensy_link/conftest.py::fake_teensy_and_client
    but injects the started client into a TeensyBridgeNode.

    ``boot_state_read`` defaults False so the cold-start boot read does NOT
    fire during construction (these tests don't wire a Platform-Teensy STATE_READ
    responder before __init__, and exercise the cold-start path explicitly via
    tests/ros/test_teensy_bridge_node_coldstart.py). Pass True to exercise the boot
    read (the cache then ends conservative unless a responder is pre-wired).

    All sockets bind ephemeral ports (``0``) and the assigned port is read back,
    so the whole family is xdist-parallel-safe.
    """
    from jugglebot.teensy_bridge_node import TeensyBridgeNode

    teensy = FakeTeensy(bind_host="127.0.0.1", stream_port=0, rpc_port=0)
    teensy.start()
    client = TeensyLinkClient(
        teensy_addr=("127.0.0.1", teensy.stream_port),
        rpc_port=teensy.rpc_port,
        local_bind_stream=0,
        local_bind_rpc=0,
        bind_host="127.0.0.1",
    )
    # stow_on_shutdown=False: on_shutdown() must not fire a profiled DEACTIVATE
    # against the FakeTeensy (which never completes the descent → DeactivateMonitor
    # would hang the test). Production defaults True; the stow path is exercised
    # explicitly in test_teensy_bridge_node_shutdown_stow.py.
    node = TeensyBridgeNode(client=client, boot_state_read=boot_state_read,
                            stow_on_shutdown=False)  # __init__ starts the client

    jetson_stream_port = client._stream_sock.getsockname()[1]
    jetson_rpc_port = client._rpc_sock.getsockname()[1]
    teensy.set_jetson_addr(("127.0.0.1", jetson_stream_port))

    def _send(msg_type, payload, port="stream"):
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
    return teensy, client, node


def _teardown(teensy, client, node):
    """The one teardown order: node → client → FakeTeensy.

    Absorbs the single divergent copy (test_teensy_bridge_node_shutdown_stow's,
    which cleared ``_stow_on_shutdown`` first so its stubbed stow could not
    re-fire during teardown).  For every other caller the node was already
    built with ``stow_on_shutdown=False``, so the assignment is a no-op — which
    is exactly why one canonical teardown is safe here.
    """
    node._stow_on_shutdown = False
    node.on_shutdown()
    client.stop()
    teensy.stop()


# ── Frame / message builders ───────────────────────────────────

def _platform_frame(can_id, data: bytes) -> bytes:
    """A packed PLATFORM_FRAME payload with ``data`` zero-padded to 8 bytes."""
    pf = PlatformFrame(t_bridge_us=1234, can_id=can_id, dlc=len(data),
                       data=tuple(data) + (0,) * (8 - len(data)))
    return pf.pack()


# ── Assertion helpers ──────────────────────────────────────────

def _link_kv(node):
    """Publish a link_status and return its key→value dict."""
    node._publish_link_status()
    return {v.key: v.value for v in node.link_status_pub.published[-1].values}


def _messages(mock_method):
    """All positional first-arg strings across a MagicMock log method's calls."""
    return [c.args[0] for c in mock_method.call_args_list if c.args]
