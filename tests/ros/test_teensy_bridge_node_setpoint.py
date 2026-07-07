"""Setpoint-downlink tests for teensy_bridge_node.

The setpoint downlink now drains the 40 Hz MPC command
stream (:5557 ``make_mpc_command`` dict) and packs Teensy-side knots. The headline
assertions are the SAFETY gates:
  * default-disabled (no setpoint thread, mpc_active=0, no SETPOINT frames);
  * mpc_active flips the J→T heartbeat flag only on explicit enable;
  * per-step clamp + link-down gate suppress unsafe frames.

Pure packing/gate logic is covered in tests/teensy_link/test_setpoint_pump.py;
this file covers the node wiring through the real client + FakeTeensy.
"""

from __future__ import annotations

import time

import pytest

from std_srvs.srv import SetBool

from controller.teensy_link import MsgType, HeartbeatJ2T
from controller.teensy_link.protocol import Setpoint

from tests.ros.test_teensy_bridge_node_read import _build_paired_node, _wait_until


class _FakeSource:
    """Injectable setpoint source: yields queued dicts then None."""

    def __init__(self, dicts=None):
        self._q = list(dicts or [])
        self.closed = False

    def push(self, d):
        self._q.append(d)

    def recv_latest(self):
        return self._q.pop(0) if self._q else None

    def close(self):
        self.closed = True


def _cmd(motor_rev, vel=None, cmd_next=None, cmd_next2=None):
    """A :5557 mpc_cmd-style dict (Teensy-side knot source) — motor_rev is used as u0."""
    d = {
        'type': 'mpc_cmd',
        'motor_rev': list(motor_rev),
        'vel_mm_s': list(vel if vel is not None else [0.0] * 6),
    }
    if cmd_next is not None:
        d['cmd_next_mm'] = list(cmd_next)
    if cmd_next2 is not None:
        d['cmd_next2_mm'] = list(cmd_next2)
    return d


def _node():
    teensy, client, node = _build_paired_node()
    return teensy, client, node


def _teardown(teensy, client, node):
    node.on_shutdown()
    client.stop()
    teensy.stop()


# ── Default-disabled (the non-negotiable safety default) ──────

def test_default_disabled_no_thread_no_mpc_active():
    teensy, client, node = _node()
    try:
        assert node._mpc_active is False
        assert node._sp_thread is None
        assert node._sp_source is None
        # Heartbeats carry flags=0 (mpc_active clear).
        got = teensy.wait_for(int(MsgType.HEARTBEAT_J2T), count=1, timeout=2.0)
        assert got and HeartbeatJ2T.unpack(got[-1].payload).flags == 0
    finally:
        _teardown(teensy, client, node)


def test_disabled_process_setpoint_sends_nothing():
    teensy, client, node = _node()
    try:
        # Even if a tick is fed directly, a disabled bridge sends no SETPOINT.
        node._process_setpoint(_cmd([0.1] * 6))
        time.sleep(0.05)
        assert teensy.received(int(MsgType.SETPOINT)) == []
    finally:
        _teardown(teensy, client, node)


# ── mpc_active flag wiring ─────────────────────────────────────

def _latest_hb_flags(teensy):
    got = teensy.received(int(MsgType.HEARTBEAT_J2T))
    return HeartbeatJ2T.unpack(got[-1].payload).flags if got else None


def test_set_mpc_active_flips_heartbeat_flag():
    teensy, client, node = _node()
    try:
        node._set_mpc_active(True)
        assert node._mpc_active is True
        # Poll for a heartbeat carrying flags=1 (the thread may still emit one
        # in-flight flags=0 frame before the new flag takes effect).
        assert _wait_until(lambda: _latest_hb_flags(teensy) == 0x1, timeout=2.0)

        node._set_mpc_active(False)
        assert _wait_until(lambda: _latest_hb_flags(teensy) == 0, timeout=2.0)
    finally:
        _teardown(teensy, client, node)


# ── Enabled path (manual enable + injected source) ────────────

def test_enabled_sends_setpoint_frame():
    import jugglebot.hardware_config as hw
    mm = hw.GEOM_MM_TO_REV
    teensy, client, node = _node()
    try:
        motor_rev = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        vel = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        cmd_next = [10.0, 11.0, 12.0, 13.0, 14.0, 15.0]
        cmd_next2 = [20.0, 21.0, 22.0, 23.0, 24.0, 25.0]
        src = _FakeSource([_cmd(motor_rev, vel=vel,
                                cmd_next=cmd_next, cmd_next2=cmd_next2)])
        node._start_setpoint_output(src)         # starts thread + mpc_active=1
        got = teensy.wait_for(int(MsgType.SETPOINT), count=1, timeout=2.0)
        assert got, "no SETPOINT frame sent"
        sp = Setpoint.unpack(got[0].payload)
        # u0 = motor_rev verbatim (ODrive convention).
        assert sp.u0 == pytest.approx(tuple(motor_rev), abs=1e-6)
        # u1/u2/v0 = cmd_next(_2)/vel × mm_to_rev.
        assert sp.u1 == pytest.approx(
            tuple(cmd_next[i] * mm[i] for i in range(6)), abs=1e-5)
        assert sp.v0 == pytest.approx(
            tuple(vel[i] * mm[i] for i in range(6)), abs=1e-5)
        assert sp.torque_ff == pytest.approx((0.0,) * 6, abs=1e-6)  # D9 drop
        assert sp.flags == 0x3         # HAS_U1 | HAS_U2
        assert sp.t_origin_us > 0
    finally:
        _teardown(teensy, client, node)


def test_enabled_heartbeat_carries_mpc_active():
    teensy, client, node = _node()
    try:
        node._start_setpoint_output(_FakeSource())
        assert _wait_until(lambda: _latest_hb_flags(teensy) == 0x1, timeout=2.0)
    finally:
        _teardown(teensy, client, node)


def test_shutdown_clears_mpc_active_on_wire():
    """safety-1 regression: after on_shutdown on a previously-ENABLED node, the
    J→T heartbeat carries flags=0 (mpc_active clear) — the injected client's
    heartbeat thread keeps running, so the wire flag (not just the local attr)
    must be cleared. Without the fix the thread would stream stale mpc_active=1."""
    teensy, client, node = _node()
    try:
        node._set_mpc_active(True)
        assert _wait_until(lambda: _latest_hb_flags(teensy) == 0x1, timeout=2.0)
        teensy.clear_received()
        node.on_shutdown()  # injected client → heartbeat thread keeps running
        assert _wait_until(lambda: _latest_hb_flags(teensy) == 0, timeout=2.0)
        assert node._mpc_active is False
    finally:
        client.stop()
        teensy.stop()


# ── Per-step safety gate ──────────────────────────────────────

def test_step_violation_not_sent():
    teensy, client, node = _node()
    try:
        node._mpc_active = True          # enable the send path directly
        node._process_setpoint(_cmd([0.0] * 6))        # baseline
        time.sleep(0.02)
        n_before = len(teensy.received(int(MsgType.SETPOINT)))
        node._process_setpoint(_cmd([0.0, 0.0, 0.0, 0.9, 0.0, 0.0]))  # 0.9 > 0.3
        time.sleep(0.05)
        n_after = len(teensy.received(int(MsgType.SETPOINT)))
        assert n_after == n_before        # the jumping frame was NOT sent
        assert node._sp_pump.frames_rejected == 1
    finally:
        _teardown(teensy, client, node)


def test_link_down_blocks_setpoint():
    teensy, client, node = _node()
    try:
        node._mpc_active = True
        # Force the link latch into the lost state.
        node._link_latch.first_heartbeat_seen = True
        node._link_latch.update(stale=True, seen=True)
        assert node._link_latch.command_allowed() is False
        node._process_setpoint(_cmd([0.1] * 6))
        time.sleep(0.05)
        assert teensy.received(int(MsgType.SETPOINT)) == []
    finally:
        _teardown(teensy, client, node)


def test_mpc_active_reenable_edge_resets_pump():
    """The 0→1 mpc_active re-enable EDGE forgets any stale prior setpoint
    (_sp_pump.reset()), so the per-step gate can't wedge on a _prev_pos left from a
    prior session/pose. A redundant True→True does NOT reset; a fresh 0→1 does."""
    teensy, client, node = _node()
    try:
        calls = {'n': 0}
        orig_reset = node._sp_pump.reset

        def counting_reset():
            calls['n'] += 1
            return orig_reset()

        node._sp_pump.reset = counting_reset
        node._set_mpc_active(True)      # 0→1 edge → reset
        assert calls['n'] == 1
        node._set_mpc_active(True)      # redundant, no edge → no reset
        assert calls['n'] == 1
        node._set_mpc_active(False)     # disable, no reset
        node._set_mpc_active(True)      # 0→1 edge again → reset
        assert calls['n'] == 2
    finally:
        _teardown(teensy, client, node)


def test_process_setpoint_contains_build_exception():
    """An unexpected error inside build() must NOT kill the setpoint thread or
    flip mpc_active — one bad frame is contained (logged), nothing is sent, and the
    production leg path stays live (a restart-only outage would be far worse)."""
    teensy, client, node = _node()
    try:
        node._mpc_active = True

        def boom(*a, **k):
            raise RuntimeError("synthetic build failure")

        node._sp_pump.build = boom
        node._process_setpoint(_cmd([0.0] * 6))     # must not raise
        time.sleep(0.02)
        assert teensy.received(int(MsgType.SETPOINT)) == []
        assert node._mpc_active is True             # not killed
    finally:
        _teardown(teensy, client, node)


def test_process_setpoint_contains_send_oserror():
    """A transient link send error (OSError, e.g. ENETUNREACH before the peer
    is up, or a mid-run drop) drops the frame but keeps the thread alive + mpc_active
    set — the link watchdog owns the stow, not the per-frame path."""
    teensy, client, node = _node()
    orig_send = node._client.send_stream
    try:
        node._mpc_active = True

        def raise_oserror(*a, **k):
            raise OSError("ENETUNREACH")

        node._client.send_stream = raise_oserror
        node._process_setpoint(_cmd([0.0] * 6))     # accepted frame → reaches send
        assert node._mpc_active is True             # OSError contained, not killed
    finally:
        node._client.send_stream = orig_send
        _teardown(teensy, client, node)


def test_no_position_command_sends_nothing():
    """A frame with no position command (no motor_rev/ext_mm) is skipped, not a
    fault — e.g. a non-mpc_cmd message that slipped the :5557 topic filter."""
    teensy, client, node = _node()
    try:
        node._mpc_active = True
        node._process_setpoint({'type': 'other', 'vel_mm_s': [0.0] * 6})
        time.sleep(0.05)
        assert teensy.received(int(MsgType.SETPOINT)) == []
        assert node._sp_pump.frames_skipped == 1
    finally:
        _teardown(teensy, client, node)


# ── Runtime arming service (set_setpoint_output) ──────────────
# The production arming flow (fixes the arm-before-stream trap): true requires
# (a) link up + fresh heartbeat, (b) a fresh mpccmd frame on :5557 within 0.5 s,
# (c) that frame's u0 within 0.25 rev of every leg's live pos_estimate. Then
# stream-then-arm. false disarms cleanly.


def _arm(node, data=True):
    req = SetBool.Request()
    req.data = data
    return node._svc_set_setpoint_output(req, SetBool.Response())


def _bring_link_and_telem_up(teensy, node, leg_pos=0.1):
    """Send a T→J heartbeat + telemetry (legs at ``leg_pos`` rev); wait until seen."""
    teensy.send_heartbeat_t2j()
    teensy.send_telemetry(pos_rev=tuple([leg_pos] * 7), vel_rps=tuple([0.0] * 7))
    assert _wait_until(
        lambda: node._latest_telemetry is not None
        and node._link_age_us() is not None, timeout=2.0)


def test_arm_rejected_when_link_down():
    """No T→J heartbeat ⇒ link considered down ⇒ refuse to arm (never command a
    dead link). mpc_active stays 0."""
    teensy, client, node = _node()
    try:
        node._injected_setpoint_source = _FakeSource([_cmd([0.1] * 6)])
        resp = _arm(node)                       # no heartbeat sent
        assert resp.success is False
        assert 'link' in resp.message.lower()
        assert node._mpc_active is False
    finally:
        _teardown(teensy, client, node)


def test_arm_rejected_when_no_stream():
    """Link up + telemetry, but no mpccmd frame on :5557 ⇒ refuse to arm."""
    teensy, client, node = _node()
    try:
        _bring_link_and_telem_up(teensy, node)
        node._injected_setpoint_source = _FakeSource([])    # no frames
        resp = _arm(node)
        assert resp.success is False
        assert 'mpccmd' in resp.message.lower() or '0.5' in resp.message
        assert node._mpc_active is False
    finally:
        _teardown(teensy, client, node)


def test_arm_rejected_on_u0_encoder_mismatch():
    """A fresh frame whose u0 is > 0.25 rev from the live encoder ⇒ refuse to arm
    (would risk a MAX_DEVIATION E-STOP at the arm edge)."""
    teensy, client, node = _node()
    try:
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        # motor_rev = 1.0 rev per leg, encoder = 0.1 rev → 0.9 rev mismatch.
        node._injected_setpoint_source = _FakeSource([_cmd([1.0] * 6)])
        resp = _arm(node)
        assert resp.success is False
        assert '0.25' in resp.message
        assert node._mpc_active is False
    finally:
        _teardown(teensy, client, node)


def test_arm_rejected_on_nan_pos_rev():
    """A leg reporting a non-finite (NaN) encoder ⇒ refuse to arm. The u0-vs-encoder
    precondition must fail CLOSED: the old `d > tol` PASSED a NaN (NaN > tol is
    False), arming onto an unknown pose; `not (d <= tol)` rejects it."""
    from controller.teensy_link import Telemetry
    teensy, client, node = _node()
    try:
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        # Overwrite the cached telemetry so leg 0 reads NaN (as it does before the
        # encoder is ready / after an encoder-search). u0 == 0.1 rev per leg, so a
        # finite mismatch is impossible here — only the NaN must trigger the reject.
        with node._lock:
            node._latest_telemetry = Telemetry(
                t_teensy_us=123,
                pos_rev=tuple([float('nan')] + [0.1] * 6),
                vel_rps=tuple([0.0] * 7))
        node._injected_setpoint_source = _FakeSource([_cmd([0.1] * 6)])
        resp = _arm(node)
        assert resp.success is False
        assert 'non-finite' in resp.message.lower()
        assert node._mpc_active is False
    finally:
        _teardown(teensy, client, node)


def test_arm_and_disarm_paths():
    """All preconditions met ⇒ arm (mpc_active=1, thread up, heartbeat flag set);
    disarm ⇒ clean (mpc_active=0, thread down, heartbeat flag clear)."""
    teensy, client, node = _node()
    try:
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        node._injected_setpoint_source = _FakeSource([_cmd([0.1] * 6)])
        resp = _arm(node, True)
        assert resp.success is True, resp.message
        assert node._mpc_active is True
        assert node._sp_thread is not None
        assert _wait_until(lambda: _latest_hb_flags(teensy) == 0x1, timeout=2.0)

        resp2 = _arm(node, False)
        assert resp2.success is True
        assert node._mpc_active is False
        # Disarm must NOT close the injected (test-owned) source — ownership stays
        # with the fixture (same convention as the injected client).
        assert node._injected_setpoint_source.closed is False
        assert _wait_until(lambda: _latest_hb_flags(teensy) == 0, timeout=2.0)
    finally:
        _teardown(teensy, client, node)


def test_mpc_command_zmq_source_decodes_real_wire_format():
    """The real _MpcCommandSetpointSource decodes the MPC command stream's actual
    ZMQ msgpack wire format ([topic, msgpack(dict, use_bin_type=True)]) and
    filters on the 'mpccmd' topic — the one integration point the fake-source
    tests don't cover. Uses a real PUB on a NON-production port; resends in a
    loop to beat PUB/SUB slow-joiner."""
    import zmq
    import msgpack
    from jugglebot.teensy_bridge_node import _MpcCommandSetpointSource

    addr = 'tcp://127.0.0.1:5599'  # NOT the production :5557
    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.bind(addr)
    src = _MpcCommandSetpointSource(addr=addr, topic=b'mpccmd')
    try:
        payload = {'type': 'mpc_cmd',
                   'ext_mm': [10.0] * 6, 'pose_6dof': [0.0] * 6,
                   'motor_rev': [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
                   'vel_mm_s': [1.0] * 6, 'cmd_next_mm': [11.0] * 6}
        wire = [b'mpccmd', msgpack.packb(payload, use_bin_type=True)]
        # A non-mpccmd frame on the same port (e.g. fallback-enable) must NOT be
        # delivered — the topic SUBSCRIBE filters it out.
        other = [b'other', msgpack.packb({'type': 'x'}, use_bin_type=True)]
        got = None
        deadline = time.time() + 3.0
        while time.time() < deadline and got is None:
            pub.send_multipart(other)         # filtered out
            pub.send_multipart(wire)          # resend until SUB connects
            time.sleep(0.02)
            got = src.recv_latest()
        assert got is not None, "source never decoded an mpc_cmd frame"
        assert got['type'] == 'mpc_cmd'
        assert got['motor_rev'] == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        # drain-to-latest: returns None once the queue is empty.
        assert src.recv_latest() is None
    finally:
        src.close()
        pub.close()
        ctx.term()
