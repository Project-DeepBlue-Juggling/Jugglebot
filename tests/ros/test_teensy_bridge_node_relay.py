"""Platform-Teensy relay request/reply tests for teensy_bridge_node (Phase 1).

The can-bridge re-establishes the Jetson↔Platform-Teensy conduit over CAN3: the
inclinometer tilt read (0x7DE) and the cold-start RobotState read/write (0x6E0).
The relay READS are async — the RPC only TRIGGERS a Platform-Teensy reply, which
the firmware forwards verbatim as a PLATFORM_FRAME the bridge correlates by
(can_id, dlc). These tests drive the bridge's relay methods through the real
RpcClient + a FakeTeensy that, on the trigger RPC, both ACKs and injects the
matching PLATFORM_FRAME — exactly the on-hardware sequence (modulo the SRX_DIS
self-echo question, which is the Phase-1 bench-probe gate).

ROS 2 is mocked by tests/ros/conftest.py; the relay methods are pure node methods
(no ROS service surface yet — that is Phase 4), fully covered here.
"""

from __future__ import annotations

import struct

from controller.teensy_link import RpcMethod, RpcStatus, MsgType, PlatformFrame
from controller.teensy_link import rpc_args
from controller.teensy_link import protocol as p

from tests.ros.test_teensy_bridge_node_read import _build_paired_node

_STATE_ID = 0x6E0   # PlatformCanId::STATE_UPDATE
_TILT_ID = 0x7DE    # PlatformCanId::TILT_READING


def _node():
    teensy, client, node = _build_paired_node()
    return teensy, client, node


def _teardown(teensy, client, node):
    node.on_shutdown()
    client.stop()
    teensy.stop()


def _platform_frame(can_id, data: bytes) -> bytes:
    pf = PlatformFrame(t_bridge_us=1234, can_id=can_id, dlc=len(data),
                       data=tuple(data) + (0,) * (8 - len(data)))
    return pf.pack()


def _encode_state_frame(is_homed, levelling, x_milli, y_milli) -> bytes:
    """The 8-byte 0x6E0 RobotState reply, exactly as Teensy_code.ino packs it."""
    flags = (1 if is_homed else 0) | (2 if levelling else 0)
    return struct.pack("<Bhh", flags, x_milli, y_milli) + b"\x00\x00\x00"


def test_relay_read_robot_state_correlates_platform_frame():
    teensy, client, node = _node()
    try:
        captured = {}

        def handler(req_id, args):
            captured["args"] = args
            # On the trigger, inject the Platform-Teensy RobotState reply.
            teensy.send_to_jetson(int(MsgType.PLATFORM_FRAME),
                                  _platform_frame(_STATE_ID,
                                                  _encode_state_frame(True, False, 12, -34)))
            return (int(RpcStatus.OK), b"")

        teensy.on_rpc(int(RpcMethod.STATE_READ), handler)

        ok, msg, state = node.relay_read_robot_state(timeout=1.0)
        assert ok, msg
        assert captured["args"] == b""          # STATE_READ is payloadless (a trigger)
        assert state.is_homed is True
        assert state.levelling_complete is False
        assert state.pose_offset_tiltX == 0.012
        assert state.pose_offset_tiltY == -0.034
    finally:
        _teardown(teensy, client, node)


def test_relay_read_tilt_correlates_platform_frame():
    teensy, client, node = _node()
    try:
        def handler(req_id, args):
            teensy.send_to_jetson(int(MsgType.PLATFORM_FRAME),
                                  _platform_frame(_TILT_ID, struct.pack("<ff", 0.05, -0.07)))
            return (int(RpcStatus.OK), b"")

        teensy.on_rpc(int(RpcMethod.TILT_READ), handler)

        ok, msg, tilt = node.relay_read_tilt(timeout=1.0)
        assert ok, msg
        assert tilt[0] == struct.unpack("<f", struct.pack("<f", 0.05))[0]
        assert tilt[1] == struct.unpack("<f", struct.pack("<f", -0.07))[0]
    finally:
        _teardown(teensy, client, node)


def test_relay_read_times_out_without_platform_frame():
    """No PLATFORM_FRAME reply → the read fails (does not hang) after the timeout."""
    teensy, client, node = _node()
    try:
        # Ack the trigger but never inject a reply.
        teensy.on_rpc(int(RpcMethod.STATE_READ), lambda rid, a: (int(RpcStatus.OK), b""))
        ok, msg, state = node.relay_read_robot_state(timeout=0.2)
        assert not ok
        assert state is None
        assert "timeout" in msg.lower()
    finally:
        _teardown(teensy, client, node)


def test_relay_read_fails_fast_on_bus_down():
    """A relay read that the firmware refuses (ERR_BUS_DOWN) returns immediately —
    it must NOT enter the async await loop (the BB fail-fast pattern)."""
    teensy, client, node = _node()
    try:
        # The firmware returns ERR_BUS_DOWN; _call_rpc maps that to (False, ...).
        teensy.on_rpc(int(RpcMethod.STATE_READ),
                      lambda rid, a: (int(RpcStatus.ERR_BUS_DOWN), b""))
        ok, msg, state = node.relay_read_robot_state(timeout=2.0)
        assert not ok
        assert state is None
    finally:
        _teardown(teensy, client, node)


def test_relay_write_robot_state_sends_arg_bytes():
    teensy, client, node = _node()
    try:
        box = {}

        def handler(req_id, args):
            box["args"] = args
            return (int(RpcStatus.OK), b"")

        teensy.on_rpc(int(RpcMethod.STATE_WRITE), handler)

        ok, msg = node.relay_write_robot_state(
            is_homed=True, levelling_complete=True,
            pose_offset_tiltX=0.01, pose_offset_tiltY=-0.02)
        assert ok, msg
        # The exact bytes the firmware memcpy's into JbUdp::RpcArgs::ArgRobotState.
        assert box["args"] == rpc_args.encode_state_write(True, True, 0.01, -0.02)
    finally:
        _teardown(teensy, client, node)


def test_relay_read_clears_stale_reply_before_trigger():
    """A reply latched from a PREVIOUS read must not satisfy a new read: the bridge
    clears the latch before sending the trigger, then awaits a fresh reply."""
    teensy, client, node = _node()
    try:
        # Pre-seed a stale reply directly into the latch (as if from an earlier read).
        with node._platform_reply_cv:
            node._platform_replies[_STATE_ID] = (
                _encode_state_frame(False, False, 0, 0), 8)

        seen = {"called": False}

        def handler(req_id, args):
            seen["called"] = True
            teensy.send_to_jetson(int(MsgType.PLATFORM_FRAME),
                                  _platform_frame(_STATE_ID,
                                                  _encode_state_frame(True, True, 5, 6)))
            return (int(RpcStatus.OK), b"")

        teensy.on_rpc(int(RpcMethod.STATE_READ), handler)

        ok, msg, state = node.relay_read_robot_state(timeout=1.0)
        assert ok, msg
        assert seen["called"]                   # the trigger actually fired
        assert state.is_homed is True           # the FRESH reply, not the stale one
        assert state.levelling_complete is True
    finally:
        _teardown(teensy, client, node)


# ── Relay serialization ([16]) ─────────────────────────────────

def test_relay_lock_is_reentrant():
    """[16]: _relay_lock is an RLock so the read-modify-write helpers can hold it
    across their whole cache-read → relay_write → cache-update sequence — which
    itself re-acquires the lock inside relay_write_robot_state — WITHOUT
    deadlocking. A plain Lock would deadlock on that nested same-thread acquire."""
    teensy, client, node = _node()
    try:
        assert node._relay_lock.acquire(blocking=False) is True
        # Re-entrant: a second acquire from the SAME thread also succeeds.
        assert node._relay_lock.acquire(blocking=False) is True
        node._relay_lock.release()
        node._relay_lock.release()
    finally:
        _teardown(teensy, client, node)


def test_relay_lock_serializes_rmw_against_concurrent_relay():
    """[16]: _write_is_homed holds _relay_lock across its ENTIRE read-modify-write,
    so a concurrent relay op cannot interleave and land a stale write last (the
    STATE_WRITE lost-update). We prove the whole RMW is under the lock by blocking
    inside the wire-write and confirming another thread cannot take the lock until
    the RMW completes."""
    import threading

    teensy, client, node = _node()
    orig = node.relay_write_robot_state
    try:
        entered = threading.Event()
        release = threading.Event()

        def slow_write(*a, **k):
            entered.set()
            release.wait(2.0)
            return True, "ok"

        node.relay_write_robot_state = slow_write
        t = threading.Thread(target=lambda: node._write_is_homed(True), daemon=True)
        t.start()
        assert entered.wait(2.0), "RMW never reached the wire write"
        # The RMW thread holds _relay_lock across the (blocked) wire write; another
        # thread must NOT be able to acquire it.
        assert node._relay_lock.acquire(blocking=False) is False
        release.set()
        t.join(2.0)
        assert not t.is_alive()
        # Freed once the RMW completes.
        assert node._relay_lock.acquire(blocking=False) is True
        node._relay_lock.release()
    finally:
        node.relay_write_robot_state = orig
        _teardown(teensy, client, node)
