"""Cold-start state tests for teensy_bridge_node.

The bridge sources the orchestrator's cold-start fields (``is_homed`` /
``levelling_complete`` / ``pose_offset``) from the Platform Teensy's ``RobotState``
through the Platform-Teensy relay, replacing the hardcoded conservative defaults the read-only bridge shipped with.
The bridge:

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

import dataclasses
import struct
import time
import types
from unittest.mock import MagicMock

from controller.teensy_link import RpcMethod, RpcStatus, MsgType, PlatformFrame
from controller.teensy_link import (
    LinkState, BusHealth, FaultState, HeartbeatT2J, Telemetry,
)
from controller.teensy_link import rpc_args
from controller.teensy_link import protocol as p

from tests.ros._bridge_harness import (
    _build_paired_node,
    _messages,
    _platform_frame,
    _teardown,
    _wait_until,
)

_STATE_ID = 0x6E0   # PlatformCanId::STATE_UPDATE


# ── Helpers ────────────────────────────────────────────────────

def _node(*, boot_state_read=False):
    teensy, client, node = _build_paired_node(boot_state_read=boot_state_read)
    return teensy, client, node


def _state_reply(is_homed, levelling, x_milli, y_milli,
                 fw_version=rpc_args.PLATFORM_FW_VERSION_EXPECTED) -> bytes:
    """The 8-byte 0x6E0 RobotState reply, exactly as Teensy_code_platform.ino packs it.

    ``fw_version`` occupies bytes 5-6 (uint16 LE) and defaults to the version this
    host tree expects — i.e. a CORRECTLY-FLASHED Platform Teensy, which is the
    premise of every cold-start test below. Pass
    ``rpc_args.PLATFORM_FW_VERSION_UNVERSIONED`` (0) to simulate a pre-2026-07-27
    board: the bytes it puts there are zeros, because every firmware built before
    the identity block zero-filled 5-7 unconditionally.
    """
    flags = (1 if is_homed else 0) | (2 if levelling else 0)
    return (struct.pack("<Bhh", flags, x_milli, y_milli)
            + int(fw_version).to_bytes(2, "little") + b"\x00")


def _wire_state_read(teensy, *, is_homed, levelling, x_milli, y_milli,
                     fw_version=rpc_args.PLATFORM_FW_VERSION_EXPECTED):
    """Register a STATE_READ responder that ACKs + injects a RobotState reply."""
    def handler(req_id, args):
        teensy.send_to_jetson(
            int(MsgType.PLATFORM_FRAME),
            _platform_frame(_STATE_ID,
                            _state_reply(is_homed, levelling, x_milli, y_milli,
                                         fw_version)))
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

        # The reconnect re-read now runs on a daemon thread (non-blocking
        # on the 1 Hz timer / 100 Hz publish); wait for it to land.
        assert _wait_until(lambda: not node._cold_start_reread_inflight
                           and node._cold_start_state.is_homed)
        assert node._cold_start_state.is_homed is True
        assert node._cold_start_state.levelling_complete is True
        assert node._cold_start_authoritative is True
    finally:
        _teardown(teensy, client, node)


# ── CAN3-bus-health conservative reconnect re-read ──────
# The UDP-watchdog reconnect (above) does NOT fire for a CAN3-only drop (Jugglebot
# disconnected while the Jetson + can-bridge stay powered → UDP link intact). The
# bridge adds a bus1_health (CAN3) "→OK from a DEGRADED state" edge
# that fires a CONSERVATIVE re-read (retry, then is_homed=False on failure — NOT
# keep-stale), because a CAN3 recovery implies the Platform Teensy may have
# power-cycled (it shares the ODrive supply). UNKNOWN→OK (the boot first-connection)
# is excluded.

def _send_hb(teensy, node, bus1):
    hb = HeartbeatT2J(t_teensy_us=1, link_state=int(LinkState.UP),
                      bus1_health=int(bus1), bus2_health=int(BusHealth.OK),
                      fault_state=int(FaultState.NONE), flags=0, uptime_ms=1)
    teensy.send_to_jetson(int(MsgType.HEARTBEAT_T2J), hb.pack())
    assert _wait_until(lambda: node._latest_heartbeat is not None
                       and int(node._latest_heartbeat.bus1_health) == int(bus1))


def test_can3_health_recovery_conservative_reread_success():
    """CAN3 bus1_health recovering WARN→OK fires a conservative re-read; with a
    working STATE_READ it refreshes the cache to the authoritative value."""
    teensy, client, node = _node()
    try:
        from jugglebot.teensy_bridge_node import RelayRobotState
        _wire_state_read(teensy, is_homed=True, levelling=True, x_milli=7, y_milli=8)
        # Warm the relay round-trip (cold-process first-call latency), then stale it.
        assert node._refresh_cold_start_state('warmup') is True
        with node._lock:
            node._cold_start_state = RelayRobotState(False, False, 0.0, 0.0)

        _send_hb(teensy, node, BusHealth.OK)
        node._last_bus1_health = int(BusHealth.WARN)   # prior DEGRADED state
        node._health_check()

        # The re-read runs on a daemon thread (audit MEDIUM fix — non-blocking on
        # the timer); wait for it to land.
        assert _wait_until(lambda: not node._cold_start_reread_inflight
                           and node._cold_start_state.is_homed)
        # The WARN→OK edge re-read the authoritative store.
        assert node._cold_start_state.is_homed is True
        assert node._cold_start_state.levelling_complete is True
        assert node._cold_start_authoritative is True
    finally:
        _teardown(teensy, client, node)


def test_can3_health_recovery_conservative_on_read_failure():
    """CAN3 WARN→OK with a FAILING re-read falls back to is_homed=False (conservative
    — NOT keep-stale). This is the precondition's safety crux: a Jugglebot
    power-cycle (CAN3 recovery) must never leave a stale is_homed=True against a
    de-referenced robot. Contrast test_refresh_failure_keeps_cached_value (the
    UDP-reconnect path KEEPS stale — a link blip is not a power loss)."""
    teensy, client, node = _node()
    try:
        from jugglebot.teensy_bridge_node import RelayRobotState
        # Cache holds a STALE is_homed=True (e.g. from before Jugglebot dropped).
        with node._lock:
            node._cold_start_state = RelayRobotState(True, True, 0.01, 0.02)
        # STATE_READ fails (Platform Teensy not yet answering after the reconnect).
        teensy.on_rpc(int(RpcMethod.STATE_READ),
                      lambda rid, a: (int(RpcStatus.ERR_BUS_DOWN), b""))

        _send_hb(teensy, node, BusHealth.OK)
        node._last_bus1_health = int(BusHealth.BUS_OFF)   # prior DEGRADED state
        node._health_check()

        # The re-read runs on a daemon thread; wait for it to land + clear.
        assert _wait_until(lambda: not node._cold_start_reread_inflight
                           and node._cold_start_state.is_homed is False)
        # Conservative: is_homed cleared to False (forces a re-home), not kept True.
        assert node._cold_start_state.is_homed is False
        assert node._cold_start_authoritative is False
    finally:
        _teardown(teensy, client, node)


def test_can3_health_unknown_to_ok_does_not_fire():
    """UNKNOWN→OK (the boot first-connection, NOT a loss) does NOT fire the
    conservative re-read — so it cannot clobber a good boot value. The cache is
    left untouched even with no STATE_READ responder wired."""
    teensy, client, node = _node()
    try:
        from jugglebot.teensy_bridge_node import RelayRobotState
        with node._lock:
            node._cold_start_state = RelayRobotState(True, True, 0.01, 0.02)
        # No STATE_READ responder: if the edge fired, the conservative fallback
        # would clobber is_homed to False. It must NOT fire.
        _send_hb(teensy, node, BusHealth.OK)
        node._last_bus1_health = int(BusHealth.UNKNOWN)   # boot "no frames yet"
        node._health_check()

        assert node._cold_start_reread_inflight is False  # no re-read thread spawned
        assert node._cold_start_state.is_homed is True   # untouched (no re-read fired)
        assert node._last_bus1_health == int(BusHealth.OK)  # edge tracker advanced
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
            node._encoder_search_done_session = True   # a search completed this session

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
        # And the encoder-search bit is cleared too (the reboot invalidates the ODrive
        # encoder index → a re-search is required before the next home).
        assert node._encoder_search_done_session is False
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


def test_reboot_clears_encoder_search_complete():
    """A REBOOT_ODRIVES drops the DERIVED encoder_search_complete to False. The ODrive
    MCUs lose their incremental-encoder INDEX on reboot (not pre-calibrated to flash —
    operator-confirmed 2026-07-02), so a re-encoder-search is required before the next
    home. If encoder_search_complete stayed True, the orchestrator's HomingHandler
    would SKIP encoder-search and home on an un-indexed encoder → ODRIVE_FATAL →
    FAULT→BOOT→HOMING loop (the hardware bug found 2026-07-02). This DELIBERATELY
    diverges from can_node (can_node.py:1552-1566 cleared is_homed/levelling/pose but
    left encoder_search_complete set — a latent bug that only bit once the orchestrator
    drove homing automatically after a reboot). See logbook
    2026-07-02-canbridge-reboot-encoder-search-clear."""
    teensy, client, node = _node()
    try:
        # A relay read returns is_homed=True (persisted-homed) → latches the session
        # bit (can_node:549-550 parity), so encoder_search_complete starts True.
        _wire_state_read(teensy, is_homed=True, levelling=False, x_milli=0, y_milli=0)
        assert node._refresh_cold_start_state('boot') is True
        assert node._encoder_search_done_session is True
        assert _publish_and_get(node, teensy).encoder_search_complete is True

        # Reboot: is_homed cleared AND the session bit cleared → encoder_search_complete
        # DROPS to False (a re-search is required post-reboot).
        teensy.on_rpc(int(RpcMethod.STATE_WRITE), lambda rid, a: (int(RpcStatus.OK), b""))
        teensy.on_rpc(int(RpcMethod.REBOOT_ODRIVES), lambda rid, a: (int(RpcStatus.OK), b""))
        ok, msg = node._reboot_odrives()
        assert ok, msg
        assert node._cold_start_state.is_homed is False
        assert node._encoder_search_done_session is False   # cleared by the reboot hook
        node._publish_robot_state()   # telemetry already cached from _publish_and_get
        msg_out = node.robot_state_pub.published[-1]
        assert msg_out.is_homed is False
        assert msg_out.encoder_search_complete is False      # re-search required post-reboot
    finally:
        _teardown(teensy, client, node)


# ── Platform-Teensy firmware identity (bytes 5-6 of the same 0x6E0 reply) ──────
#
# WHY THIS RIDES THE COLD-START READ AND IS TESTED HERE. Until 2026-07-27 the
# Platform Teensy carried no version, so an UN-FLASHED board was indistinguishable
# from a flashed one from the Jetson — the only deployment in the stack that
# failed silently. The version is now reported in bytes 5-6 of the RobotState
# reply the board ALREADY sends, so detecting a stale flash costs no new CAN frame
# on the bus the 0x6D0 hand conduit shares, and — decisively — a pre-versioning
# board ANSWERS (with 0) rather than timing out, which a new query frame would
# have made indistinguishable from a dead CAN3.
#
# The verdict is WARNED, never enforced: nothing below asserts a refusal, because
# there is none. See ros_ws/docs/platform_fw_version.md.

def _link_status_values(node):
    node._publish_link_status()
    return {kv.key: kv.value
            for kv in node.link_status_pub.published[-1].values}


def test_a_flashed_board_reports_its_version_and_logs_ok():
    """The expected version caches, surfaces on both topics, and logs one OK."""
    teensy, client, node = _node()
    node._logger = MagicMock()
    try:
        _wire_state_read(teensy, is_homed=True, levelling=False, x_milli=0,
                         y_milli=0,
                         fw_version=rpc_args.PLATFORM_FW_VERSION_EXPECTED)
        assert node._refresh_cold_start_state('test') is True
        assert node._platform_fw_version == rpc_args.PLATFORM_FW_VERSION_EXPECTED

        msg = _publish_and_get(node, teensy)
        assert msg.platform_fw_version == rpc_args.PLATFORM_FW_VERSION_EXPECTED
        assert msg.platform_fw_version_read is True
        assert (_link_status_values(node)['platform_fw_version']
                == str(rpc_args.PLATFORM_FW_VERSION_EXPECTED))

        ok_lines = [m for m in _messages(node._logger.info)
                    if 'PLATFORM_FW_CHECK: OK' in m]
        assert len(ok_lines) == 1, _messages(node._logger.info)
        assert not [m for m in _messages(node._logger.error)
                    if 'PLATFORM_FW_CHECK' in m]
    finally:
        _teardown(teensy, client, node)


def test_an_unflashed_board_is_detected_and_surfaced_everywhere():
    """THE headline case: a pre-versioning board answers with 0 and is caught.

    Three surfaces, because each covers a hole the others leave: the ERROR log is
    what a runbook greps out of launch.log after the fact; link_status is the 10 Hz
    read an operator can actually take live (`ros2 topic echo` gives false negatives
    on this Foxy box for a 100 Hz RELIABLE topic like robot_state); and the typed
    robot_state fields are the machine-readable record that lands in the rosbag.
    """
    teensy, client, node = _node()
    node._logger = MagicMock()
    try:
        _wire_state_read(teensy, is_homed=True, levelling=True, x_milli=1,
                         y_milli=2,
                         fw_version=rpc_args.PLATFORM_FW_VERSION_UNVERSIONED)
        assert node._refresh_cold_start_state('boot') is True
        assert node._platform_fw_version == 0

        errors = [m for m in _messages(node._logger.error)
                  if 'PLATFORM_FW_CHECK: FAIL' in m]
        assert len(errors) == 1, _messages(node._logger.error)
        assert 'PRE-VERSIONING' in errors[0]
        # The message must say what to do AND what is not happening, or an
        # operator reads an ERROR and assumes the stack already protected them.
        assert 'NOT refused' in errors[0]
        assert 're-flash' in errors[0]

        msg = _publish_and_get(node, teensy)
        assert msg.platform_fw_version == 0
        assert msg.platform_fw_version_read is True     # it ANSWERED — not silence
        assert (_link_status_values(node)['platform_fw_version']
                == '0 (PRE-VERSIONING)')

        # The rest of the frame still decoded correctly: the version must not have
        # displaced is_homed / levelling / pose.
        assert msg.is_homed is True and msg.levelling_complete is True
        assert msg.pose_offset_rad == [0.001, 0.002]
    finally:
        _teardown(teensy, client, node)


def test_an_unexpected_version_is_reported_as_a_skew_not_as_unversioned():
    """A board flashed with a DIFFERENT numbered release is its own verdict."""
    teensy, client, node = _node()
    node._logger = MagicMock()
    try:
        other = rpc_args.PLATFORM_FW_VERSION_EXPECTED + 7
        _wire_state_read(teensy, is_homed=False, levelling=False, x_milli=0,
                         y_milli=0, fw_version=other)
        assert node._refresh_cold_start_state('test') is True
        assert node._platform_fw_version == other

        errors = [m for m in _messages(node._logger.error)
                  if 'PLATFORM_FW_CHECK: FAIL' in m]
        assert len(errors) == 1
        assert f'v{other}' in errors[0]
        assert 'PRE-VERSIONING' not in errors[0]
        assert _link_status_values(node)['platform_fw_version'] == str(other)
        assert _publish_and_get(node, teensy).platform_fw_version == other
    finally:
        _teardown(teensy, client, node)


def test_no_read_at_all_is_reported_as_unknown_not_as_unflashed():
    """Absence must be as loud as a wrong answer — and must NOT be confused with it.

    A board that never answers is a relay/CAN3 fault; a board that answers 0 has
    not been flashed. Collapsing the two would send an operator to re-flash a
    healthy board while the real fault (which also forces a re-home) went
    undiagnosed.
    """
    teensy, client, node = _node(boot_state_read=True)   # no STATE_READ responder
    try:
        assert node._platform_fw_version is None
        assert node._cold_start_authoritative is False

        msg = _publish_and_get(node, teensy)
        assert msg.platform_fw_version_read is False
        assert msg.platform_fw_version == 0        # placeholder ONLY; read=False
        assert _link_status_values(node)['platform_fw_version'] == 'unknown'
    finally:
        _teardown(teensy, client, node)


def test_the_unknown_verdict_shares_the_grep_token():
    """One `grep PLATFORM_FW_CHECK launch.log` returns all three verdicts."""
    teensy, client, node = _node()
    node._logger = MagicMock()
    try:
        assert node._read_cold_start_state_conservative('boot') is False
        unknown = [m for m in _messages(node._logger.error)
                   if 'PLATFORM_FW_CHECK: UNKNOWN' in m]
        assert len(unknown) == 1, _messages(node._logger.error)
        # It must NOT read as evidence of a stale flash — that is a different fix.
        assert 'stale flash' in unknown[0]
    finally:
        _teardown(teensy, client, node)


def test_a_reboot_does_not_erase_a_known_version():
    """REBOOT_ODRIVES clears is_homed / levelling / pose. It must NOT clear the
    firmware version.

    The Platform Teensy stays powered through an ODrive reboot (it is the board
    that PERSISTS the cold-start state across one), so its firmware cannot have
    changed. Clearing the version here — which is what folding it into
    RelayRobotState would have done — would raise PLATFORM_FW_CHECK: UNKNOWN on a
    board whose version was read seconds earlier, training the operator to ignore
    the check.
    """
    teensy, client, node = _node()
    try:
        _wire_state_read(teensy, is_homed=True, levelling=True, x_milli=0, y_milli=0)
        assert node._refresh_cold_start_state('boot') is True
        assert node._platform_fw_version == rpc_args.PLATFORM_FW_VERSION_EXPECTED

        teensy.on_rpc(int(RpcMethod.STATE_WRITE), lambda rid, a: (int(RpcStatus.OK), b""))
        teensy.on_rpc(int(RpcMethod.REBOOT_ODRIVES), lambda rid, a: (int(RpcStatus.OK), b""))
        ok, msg = node._reboot_odrives()
        assert ok, msg

        assert node._cold_start_state.is_homed is False        # cleared, as before
        assert node._platform_fw_version == rpc_args.PLATFORM_FW_VERSION_EXPECTED
        assert _publish_and_get(node, teensy).platform_fw_version_read is True
    finally:
        _teardown(teensy, client, node)


def test_a_failed_reread_keeps_the_last_known_version():
    """A transient relay failure must not downgrade a good version to UNKNOWN.

    Same policy as the cold-start cache's keep-stale-on-failure branch, and for a
    stronger reason: a firmware version can only change by a flash, and a flash
    reboots the board. A CAN3 hiccup is no evidence at all that the firmware moved.
    """
    teensy, client, node = _node()
    try:
        _wire_state_read(teensy, is_homed=True, levelling=False, x_milli=0, y_milli=0)
        assert node._refresh_cold_start_state('boot') is True
        assert node._platform_fw_version == rpc_args.PLATFORM_FW_VERSION_EXPECTED

        # Drop the responder: the next read times out.
        teensy.on_rpc(int(RpcMethod.STATE_READ),
                      lambda rid, a: (int(RpcStatus.OK), b""))
        assert node._refresh_cold_start_state('reconnect') is False
        assert node._platform_fw_version == rpc_args.PLATFORM_FW_VERSION_EXPECTED
    finally:
        _teardown(teensy, client, node)


def test_a_skew_does_not_gate_the_hand_dispatch_path():
    """The version WARNS; it never refuses. Pinned, because the tempting "safety"
    change here is actively dangerous.

    ``_svc_set_hand_traj`` carries the kind-3 retract, and a kind-3 clobbering an
    armed kind-0 is the ONLY un-arm mechanism the Teensy offers — a pre-release
    SAFE_ABORT depends on it. A version gate in front of it would turn a skipped
    flash into "the abort path no longer works". The other refusal shapes are no
    better: refusing kind-1 after the throw has flown drops a ball the pre-fix
    stack would have caught, and the version's own input (a cached relay read) has
    a documented benign-transient failure mode of its own.
    """
    teensy, client, node = _node()
    try:
        _wire_state_read(teensy, is_homed=True, levelling=False, x_milli=0,
                         y_milli=0,
                         fw_version=rpc_args.PLATFORM_FW_VERSION_UNVERSIONED)
        assert node._refresh_cold_start_state('boot') is True
        assert node._platform_fw_version == 0

        sent = []
        teensy.on_rpc(int(RpcMethod.HAND_TRAJ_CMD),
                      lambda rid, a: (sent.append(a), (int(RpcStatus.OK), b""))[1])

        # kind 3 — the SAFE_ABORT retract, the only un-arm mechanism there is.
        ok, _msg, _ = node._call_rpc(
            RpcMethod.HAND_TRAJ_CMD, rpc_args.encode_smooth_move_hand(0.0))
        assert ok, 'the kind-3 retract must dispatch on a version-skewed board'
        # kind 1 — an armed catch, dispatched mid-sequence after a throw has flown.
        ok, _msg, _ = node._call_rpc(
            RpcMethod.HAND_TRAJ_CMD,
            rpc_args.encode_hand_traj_cmd(1, 3.13, 1_800_000_000_000))
        assert ok, 'a scheduled catch must dispatch on a version-skewed board'
        assert len(sent) == 2
        assert sent[0][0] == 3      # the kind-3 payload reached the wire verbatim

        # ── The REAL dispatch path ────────────────────────────────────────────
        # The two _call_rpc calls above prove the transport works on a skewed
        # board, but they enter BELOW the funnel a refusal would be written into.
        # Both hand services route through node.teensy_hand_traj_cmd, so drive the
        # SERVICE HANDLERS — the actual entry points an operator/orchestrator hits.
        sent.clear()
        # kind 3 via /smooth_move_hand — the SAFE_ABORT retract.
        res = types.SimpleNamespace(success=None, message='')
        out = node._svc_smooth_move_hand(types.SimpleNamespace(data=0.0), res)
        assert out.success is True, (
            'the kind-3 retract must dispatch through the SERVICE on a '
            f'version-skewed board; got: {out.message}')
        # kind 1 via /set_hand_traj_cmd — an armed catch, after a throw has flown.
        res = types.SimpleNamespace(success=None, message='')
        out = node._svc_set_hand_traj(
            types.SimpleNamespace(event_delay=0.5, event_vel=3.13, traj_type=1),
            res)
        assert out.success is True, (
            'a scheduled catch must dispatch through the SERVICE on a '
            f'version-skewed board; got: {out.message}')
        assert len(sent) == 2, 'both service dispatches must reach the wire'
        assert sent[0][0] == 3      # the kind-3 payload reached the wire verbatim
        assert sent[1][0] == 1

        # Tripwire (not a proof): no host-side gate point on the 0x6D0 conduit may
        # so much as READ the version. teensy_hand_traj_cmd is included and is the
        # one that matters — it is the SINGLE FUNNEL both services call, so it is
        # where this repo's own "one enforcement point" convention would put a
        # gate, and a gate there is invisible to a tripwire that inspects only the
        # two leaf handlers.
        import inspect
        for fn in (node._svc_set_hand_traj, node._svc_smooth_move_hand,
                   node.teensy_hand_traj_cmd):
            src = inspect.getsource(fn)
            assert '_platform_fw_version' not in src, (
                f'{fn.__name__} reads the Platform FW version — a version gate on '
                'the hand path was added. Read '
                'ros_ws/docs/platform_fw_version.md § Warn, never refuse first: '
                'this path carries the kind-3 retract, the only un-arm mechanism '
                'the Teensy offers.')
    finally:
        _teardown(teensy, client, node)


def test_a_stale_interfaces_build_names_itself_at_construction():
    """A half-rebuilt tree must SAY SO, not present as a power/CAN fault.

    ``_publish_robot_state`` catches its own exceptions, so — unlike
    ``trajectory_node``, whose uncaught timer exception exits the process ~200 ms
    after launch — a stale ``jugglebot_interfaces`` build here degrades to one
    throttled ERROR per 5 s while ``/robot_state`` silently stops at 100 Hz. The
    orchestrator then stalls in BOOT and blames power/CAN. This check turns that
    into a named, greppable error at construction.
    """
    import jugglebot.teensy_bridge_node as tbn

    @dataclasses.dataclass
    class _StaleRobotState:            # a pre-2026-07-27 generated message
        is_homed: bool = False
        levelling_complete: bool = False

    teensy, client, node = _node()
    node._logger = MagicMock()
    try:
        real = tbn.RobotState
        tbn.RobotState = _StaleRobotState
        try:
            node._warn_if_robot_state_msg_is_stale()
        finally:
            tbn.RobotState = real

        errors = [m for m in _messages(node._logger.error)
                  if 'INTERFACES_STALE' in m]
        assert len(errors) == 1, _messages(node._logger.error)
        # It must name BOTH missing fields and the exact remedy, or the operator
        # cannot tell this from the hardware fault it otherwise looks like.
        assert 'platform_fw_version' in errors[0]
        assert 'platform_fw_version_read' in errors[0]
        assert 'jugglebot_interfaces jugglebot' in errors[0]

        # And it must stay SILENT on a current build — an instrument that cries
        # wolf on the healthy tree is worse than no instrument.
        node._logger.reset_mock()
        node._warn_if_robot_state_msg_is_stale()
        assert not [m for m in _messages(node._logger.error)
                    if 'INTERFACES_STALE' in m]

        # Wiring: the check is worthless unless CONSTRUCTION runs it. Pinned by
        # source inspection because the stale-message case cannot be provoked
        # through the real constructor without a stale generated package.
        import inspect
        assert '_warn_if_robot_state_msg_is_stale()' in inspect.getsource(
            type(node).__init__), (
            'TeensyBridgeNode.__init__ no longer runs the RobotState staleness '
            'check — a half-rebuilt tree goes back to presenting as a power/CAN '
            'fault at BOOT timeout.')
        # It must also stay in step with what _publish_robot_state assigns.
        pub_src = inspect.getsource(node._publish_robot_state)
        for f in node._ROBOT_STATE_REQUIRED_FIELDS:
            assert f'msg.{f}' in pub_src, (
                f'{f} is declared required but never assigned — the check and the '
                'publish path have drifted apart.')
    finally:
        _teardown(teensy, client, node)
