"""Cold-start state tests for teensy_bridge_node (Phase 2).

Phase 2 sources the orchestrator's cold-start fields (``is_homed`` /
``levelling_complete`` / ``pose_offset``) from the Platform Teensy's ``RobotState``
through the Phase-1 relay, replacing the hardcoded conservative defaults of phase
10b. The bridge:

* READS the state at boot (bounded retries; conservative ``is_homed=False`` on
  total failure — the SAFE direction) and on each confirmed link reconnect,
  CACHES it, and surfaces it on ``robot_state`` from the cache (the 100 Hz publish
  path never does a CAN3 round-trip);
* WRITES ``is_homed=True`` after a successful home (read-modify-write THROUGH the
  cache, preserving levelling + pose);
* clears all three on ``REBOOT_ODRIVES`` (the shared ordered hook, step 2);
* DERIVES ``encoder_search_complete = is_homed OR within-session-search-done``.

These tests drive those paths through the bridge's real relay methods + a
FakeTeensy that ACKs the trigger RPC and injects the matching PLATFORM_FRAME —
exactly the on-hardware sequence. ROS 2 is mocked by tests/ros/conftest.py.
"""

from __future__ import annotations

import struct
import time

from controller.teensy_link import RpcMethod, RpcStatus, MsgType, PlatformFrame
from controller.teensy_link import (
    LinkState, BusHealth, FaultState, HeartbeatT2J, Telemetry,
)
from controller.teensy_link import rpc_args
from controller.teensy_link import protocol as p

from tests.ros.test_teensy_bridge_node_read import _build_paired_node, _wait_until

_STATE_ID = 0x6E0   # PlatformCanId::STATE_UPDATE


# ── Helpers ────────────────────────────────────────────────────

def _node(*, boot_state_read=False):
    teensy, client, node = _build_paired_node(boot_state_read=boot_state_read)
    return teensy, client, node


def _teardown(teensy, client, node):
    node.on_shutdown()
    client.stop()
    teensy.stop()


def _platform_frame(can_id, data: bytes) -> bytes:
    pf = PlatformFrame(t_bridge_us=1234, can_id=can_id, dlc=len(data),
                       data=tuple(data) + (0,) * (8 - len(data)))
    return pf.pack()


def _state_reply(is_homed, levelling, x_milli, y_milli) -> bytes:
    """The 8-byte 0x6E0 RobotState reply, exactly as Teensy_code.ino packs it."""
    flags = (1 if is_homed else 0) | (2 if levelling else 0)
    return struct.pack("<Bhh", flags, x_milli, y_milli) + b"\x00\x00\x00"


def _wire_state_read(teensy, *, is_homed, levelling, x_milli, y_milli):
    """Register a STATE_READ responder that ACKs + injects a RobotState reply."""
    def handler(req_id, args):
        teensy.send_to_jetson(
            int(MsgType.PLATFORM_FRAME),
            _platform_frame(_STATE_ID,
                            _state_reply(is_homed, levelling, x_milli, y_milli)))
        return (int(RpcStatus.OK), b"")
    teensy.on_rpc(int(RpcMethod.STATE_READ), handler)


def _telem():
    pos = tuple(float(i) for i in range(p.NUM_AXES))
    vel = tuple(0.0 for _ in range(p.NUM_AXES))
    return Telemetry(t_teensy_us=1, pos_rev=pos, vel_rps=vel)


# ── STATE_READ round-trip → cache → publish ────────────────────

def test_refresh_populates_cache_and_publishes():
    """A relay STATE_READ refreshes the cache, and the next robot_state publish
    surfaces is_homed / levelling_complete / pose_offset from that cache."""
    teensy, client, node = _node()
    try:
        _wire_state_read(teensy, is_homed=True, levelling=True,
                         x_milli=12, y_milli=-34)
        assert node._refresh_cold_start_state('test') is True
        assert node._cold_start_state.is_homed is True
        assert node._cold_start_state.levelling_complete is True
        assert node._cold_start_authoritative is True

        teensy.send_telemetry(pos_rev=_telem().pos_rev, vel_rps=_telem().vel_rps)
        assert _wait_until(lambda: node._latest_telemetry is not None)
        node._publish_robot_state()
        msg = node.robot_state_pub.published[-1]
        assert msg.is_homed is True
        assert msg.levelling_complete is True
        # encoder_search_complete is DERIVED: is_homed ⇒ True.
        assert msg.encoder_search_complete is True
        assert msg.pose_offset_rad[0] == 0.012
        assert msg.pose_offset_rad[1] == -0.034
        # pose_offset_quat is the can_node tilt→quat of (0.012, -0.034).
        assert abs(msg.pose_offset_quat.w - 1.0) < 1e-2
        assert msg.pose_offset_quat.x != 0.0  # non-identity for a non-zero tilt
    finally:
        _teardown(teensy, client, node)


def test_publish_reads_cache_only_no_round_trip():
    """The 100 Hz publish path must be NON-BLOCKING: it reads only the cache and
    never triggers a relay read. We set the cache directly with NO STATE_READ
    responder wired — if publish did a round-trip it would fail to surface the
    value (and would block); instead it surfaces the cached value immediately."""
    teensy, client, node = _node()
    try:
        from jugglebot.teensy_bridge_node import RelayRobotState
        with node._lock:
            node._cold_start_state = RelayRobotState(
                is_homed=True, levelling_complete=False,
                pose_offset_tiltX=0.0, pose_offset_tiltY=0.0)
        teensy.send_telemetry(pos_rev=_telem().pos_rev, vel_rps=_telem().vel_rps)
        assert _wait_until(lambda: node._latest_telemetry is not None)
        t0 = time.monotonic()
        node._publish_robot_state()
        assert time.monotonic() - t0 < 0.1   # no 0.5 s relay await on the publish path
        assert node.robot_state_pub.published[-1].is_homed is True
    finally:
        _teardown(teensy, client, node)


# ── Boot read: success + conservative fallback ─────────────────

def test_boot_read_populates_cache():
    """_boot_read_cold_start_state reads + caches the authoritative state."""
    teensy, client, node = _node()
    try:
        _wire_state_read(teensy, is_homed=True, levelling=False,
                         x_milli=5, y_milli=6)
        assert node._boot_read_cold_start_state() is True
        assert node._cold_start_state.is_homed is True
        assert node._cold_start_authoritative is True
    finally:
        _teardown(teensy, client, node)


def test_boot_read_conservative_fallback_on_failure():
    """When the boot read fails (no Platform responder), __init__ falls back to the
    CONSERVATIVE is_homed=False — forcing a re-home (SAFE), never a stale True."""
    # boot_state_read=True exercises the __init__ boot-read path; no STATE_READ
    # handler is registered, so every attempt fails → conservative fallback.
    teensy, client, node = _node(boot_state_read=True)
    try:
        assert node._cold_start_state.is_homed is False
        assert node._cold_start_state.levelling_complete is False
        assert node._cold_start_authoritative is False
        # And it surfaces as is_homed=False on robot_state.
        teensy.send_telemetry(pos_rev=_telem().pos_rev, vel_rps=_telem().vel_rps)
        assert _wait_until(lambda: node._latest_telemetry is not None)
        node._publish_robot_state()
        assert node.robot_state_pub.published[-1].is_homed is False
    finally:
        _teardown(teensy, client, node)


def test_refresh_failure_keeps_cached_value():
    """A FAILED refresh (not the boot read) keeps the last authoritative value —
    can_node's passive 'last-known-state until a fresh frame'; never downgrade a
    good read on a transient hiccup."""
    teensy, client, node = _node()
    try:
        _wire_state_read(teensy, is_homed=True, levelling=True, x_milli=1, y_milli=2)
        assert node._refresh_cold_start_state('first') is True
        assert node._cold_start_state.is_homed is True
        # Now make reads fail (drop the responder) and refresh again.
        teensy.on_rpc(int(RpcMethod.STATE_READ),
                      lambda rid, a: (int(RpcStatus.ERR_BUS_DOWN), b""))
        assert node._refresh_cold_start_state('hiccup') is False
        assert node._cold_start_state.is_homed is True   # kept, not downgraded
    finally:
        _teardown(teensy, client, node)


# ── Reconnect re-read ──────────────────────────────────────────

def test_reconnect_rereads_cold_start_state():
    """On a confirmed link reconnect, _health_check re-reads the Platform Teensy's
    cold-start state (the authoritative store), refreshing the cache."""
    teensy, client, node = _node()
    try:
        from jugglebot.teensy_bridge_node import RelayRobotState
        _wire_state_read(teensy, is_homed=True, levelling=True, x_milli=7, y_milli=8)
        # Warm the relay round-trip once (first-call socket warmup in a cold test
        # process can exceed the 0.5 s await; production reply latency is ~14 ms),
        # then reset to the stale value so the reconnect read is the assertion.
        assert node._refresh_cold_start_state('warmup') is True
        with node._lock:
            node._cold_start_state = RelayRobotState(False, False, 0.0, 0.0)

        hb = HeartbeatT2J(t_teensy_us=1, link_state=int(LinkState.UP),
                          bus1_health=int(BusHealth.OK), bus2_health=int(BusHealth.OK),
                          fault_state=int(FaultState.NONE), flags=0, uptime_ms=1)
        teensy.send_to_jetson(int(MsgType.HEARTBEAT_T2J), hb.pack())
        assert _wait_until(lambda: node._link_age_us() is not None)

        # Force the reconnect edge: latch was lost, last view was lost; a fresh
        # heartbeat (not stale) makes update() clear link_lost → reconnect branch.
        node._link_latch.first_heartbeat_seen = True
        node._link_latch.link_lost = True
        node._last_link_lost = True

        node._health_check()

        assert node._cold_start_state.is_homed is True
        assert node._cold_start_state.levelling_complete is True
        assert node._cold_start_authoritative is True
    finally:
        _teardown(teensy, client, node)


# ── Homing write: read-modify-write preserves levelling ────────

def test_homing_write_sets_is_homed_preserving_levelling():
    """A homing-success write sets is_homed=True via STATE_WRITE while PRESERVING
    the levelling + pose fields already in the cache (read-modify-write)."""
    teensy, client, node = _node()
    try:
        # Cache already has levelling_complete=True + a pose offset (e.g. from a
        # prior levelling result), is_homed still False.
        from jugglebot.teensy_bridge_node import RelayRobotState
        with node._lock:
            node._cold_start_state = RelayRobotState(
                is_homed=False, levelling_complete=True,
                pose_offset_tiltX=0.03, pose_offset_tiltY=-0.04)

        box = {}

        def handler(req_id, args):
            box["args"] = args
            return (int(RpcStatus.OK), b"")
        teensy.on_rpc(int(RpcMethod.STATE_WRITE), handler)

        ok, msg = node._write_is_homed(True)
        assert ok, msg
        # The write carried is_homed=True AND the preserved levelling + pose.
        assert box["args"] == rpc_args.encode_state_write(True, True, 0.03, -0.04)
        # And the cache reflects is_homed=True with levelling still set.
        assert node._cold_start_state.is_homed is True
        assert node._cold_start_state.levelling_complete is True
        assert node._cold_start_state.pose_offset_tiltX == 0.03
    finally:
        _teardown(teensy, client, node)


def test_home_service_persists_is_homed():
    """The /home service persists is_homed=True after a successful home (the home
    op + configure are stubbed to succeed)."""
    teensy, client, node = _node()
    try:
        node._run_home = lambda axes, **kw: (True, "homed")
        node._run_configure = lambda axes, **kw: (True, "configured")
        writes = {}

        def handler(req_id, args):
            writes["args"] = args
            return (int(RpcStatus.OK), b"")
        teensy.on_rpc(int(RpcMethod.STATE_WRITE), handler)

        class _Res:
            success = False
            message = ""
        res = node._svc_home(object(), _Res())
        assert res.success is True
        # is_homed was persisted (levelling defaults False here).
        assert writes["args"] == rpc_args.encode_state_write(True, False, 0.0, 0.0)
        assert node._cold_start_state.is_homed is True
    finally:
        _teardown(teensy, client, node)


def test_home_failure_persists_is_homed_false():
    """A FAILED /home persists is_homed=False — exact can_node:847 parity (writes
    the home RESULT). The safety crux: a failed re-home of an already-homed robot
    must CLEAR is_homed, else a stale True would let the orchestrator skip homing
    on an unhomed robot."""
    teensy, client, node = _node()
    try:
        from jugglebot.teensy_bridge_node import RelayRobotState
        # Cache says homed + levelled (e.g. an operator-direct re-home of an
        # already-homed robot — the path NOT gated by the orchestrator).
        with node._lock:
            node._cold_start_state = RelayRobotState(True, True, 0.01, 0.02)
        configured = {"called": False}
        node._run_home = lambda axes, **kw: (False, "home FAILED")

        def _cfg(axes, **kw):
            configured["called"] = True
            return (True, "cfg")
        node._run_configure = _cfg
        writes = {}

        def handler(req_id, args):
            writes["args"] = args
            return (int(RpcStatus.OK), b"")
        teensy.on_rpc(int(RpcMethod.STATE_WRITE), handler)

        class _Res:
            success = True
            message = ""
        res = node._svc_home(object(), _Res())
        assert res.success is False
        assert configured["called"] is False          # configure skipped on failure
        # is_homed cleared to False; levelling/pose PRESERVED (read-modify-write).
        assert writes["args"] == rpc_args.encode_state_write(False, True, 0.01, 0.02)
        assert node._cold_start_state.is_homed is False
        assert node._cold_start_state.levelling_complete is True
    finally:
        _teardown(teensy, client, node)


# ── REBOOT_ODRIVES clears all three ────────────────────────────

def test_reboot_clears_all_cold_start_fields():
    """REBOOT_ODRIVES routes through the shared hook and clears is_homed +
    levelling_complete + pose_offset together (the ODrives lose their references)."""
    teensy, client, node = _node()
    try:
        from jugglebot.teensy_bridge_node import RelayRobotState
        with node._lock:
            node._cold_start_state = RelayRobotState(
                is_homed=True, levelling_complete=True,
                pose_offset_tiltX=0.05, pose_offset_tiltY=0.06)

        writes = {}

        def state_write(req_id, args):
            writes["args"] = args
            return (int(RpcStatus.OK), b"")
        teensy.on_rpc(int(RpcMethod.STATE_WRITE), state_write)
        teensy.on_rpc(int(RpcMethod.REBOOT_ODRIVES), lambda rid, a: (int(RpcStatus.OK), b""))

        ok, msg = node._reboot_odrives()
        assert ok, msg
        # All three cleared on the wire AND in the cache.
        assert writes["args"] == rpc_args.encode_state_write(False, False, 0.0, 0.0)
        assert node._cold_start_state.is_homed is False
        assert node._cold_start_state.levelling_complete is False
        assert node._cold_start_state.pose_offset_tiltX == 0.0
        assert node._cold_start_state.pose_offset_tiltY == 0.0
    finally:
        _teardown(teensy, client, node)


def test_reboot_clears_cache_even_if_write_fails():
    """If the STATE_WRITE clear cannot land (CAN3 down), the LOCAL cache still goes
    conservative (is_homed=False) — the safe local view post-reboot."""
    teensy, client, node = _node()
    try:
        from jugglebot.teensy_bridge_node import RelayRobotState
        with node._lock:
            node._cold_start_state = RelayRobotState(True, True, 0.05, 0.06)
        teensy.on_rpc(int(RpcMethod.STATE_WRITE),
                      lambda rid, a: (int(RpcStatus.ERR_BUS_DOWN), b""))
        teensy.on_rpc(int(RpcMethod.REBOOT_ODRIVES), lambda rid, a: (int(RpcStatus.OK), b""))

        ok, msg = node._reboot_odrives()
        # The reboot RPC succeeded; the clear is best-effort and surfaced a warning.
        assert "cold-start clear failed" in msg
        assert node._cold_start_state.is_homed is False  # local view safe
    finally:
        _teardown(teensy, client, node)


# ── encoder_search_complete derivation ─────────────────────────

def _publish_and_get(node, teensy):
    teensy.send_telemetry(pos_rev=_telem().pos_rev, vel_rps=_telem().vel_rps)
    assert _wait_until(lambda: node._latest_telemetry is not None)
    node._publish_robot_state()
    return node.robot_state_pub.published[-1]


def test_encoder_search_complete_derivation():
    """encoder_search_complete = is_homed OR within-session-search-done."""
    teensy, client, node = _node()
    try:
        from jugglebot.teensy_bridge_node import RelayRobotState
        # (a) is_homed=True ⇒ encoder_search_complete=True (even with session bit off).
        with node._lock:
            node._cold_start_state = RelayRobotState(True, False, 0.0, 0.0)
            node._encoder_search_done_session = False
        assert _publish_and_get(node, teensy).encoder_search_complete is True

        # (b) is_homed=False, no in-session search ⇒ False.
        with node._lock:
            node._cold_start_state = RelayRobotState(False, False, 0.0, 0.0)
            node._encoder_search_done_session = False
        assert _publish_and_get(node, teensy).encoder_search_complete is False

        # (c) is_homed=False, in-session search done ⇒ True.
        with node._lock:
            node._encoder_search_done_session = True
        assert _publish_and_get(node, teensy).encoder_search_complete is True
    finally:
        _teardown(teensy, client, node)


def test_encoder_search_service_sets_session_bit():
    """A successful /encoder_search sets the in-session bit (the OR term)."""
    teensy, client, node = _node()
    try:
        node._run_encoder_search = lambda axes, **kw: (True, "searched")

        class _Res:
            success = False
            message = ""
        assert node._encoder_search_done_session is False
        res = node._svc_encoder_search(object(), _Res())
        assert res.success is True
        assert node._encoder_search_done_session is True
    finally:
        _teardown(teensy, client, node)


def test_encoder_search_complete_monotonic_across_reboot():
    """Published encoder_search_complete is MONOTONIC (exact can_node parity): a
    read showing is_homed latches the session bit (can_node:549-550), so a
    subsequent REBOOT_ODRIVES that clears is_homed does NOT drop
    encoder_search_complete (can_node never clears it — can_node.py:1559-1565)."""
    teensy, client, node = _node()
    try:
        # A relay read returns is_homed=True (persisted-homed; no in-session
        # /encoder_search) → the read latches the session bit.
        _wire_state_read(teensy, is_homed=True, levelling=False, x_milli=0, y_milli=0)
        assert node._refresh_cold_start_state('boot') is True
        assert node._encoder_search_done_session is True   # latched on the is_homed read
        assert _publish_and_get(node, teensy).encoder_search_complete is True

        # Now reboot: is_homed cleared, but the session bit (and hence the published
        # encoder_search_complete) stays True.
        teensy.on_rpc(int(RpcMethod.STATE_WRITE), lambda rid, a: (int(RpcStatus.OK), b""))
        teensy.on_rpc(int(RpcMethod.REBOOT_ODRIVES), lambda rid, a: (int(RpcStatus.OK), b""))
        ok, msg = node._reboot_odrives()
        assert ok, msg
        assert node._cold_start_state.is_homed is False
        node._publish_robot_state()   # telemetry already cached from _publish_and_get
        msg_out = node.robot_state_pub.published[-1]
        assert msg_out.is_homed is False
        assert msg_out.encoder_search_complete is True      # monotonic — not dropped
    finally:
        _teardown(teensy, client, node)
