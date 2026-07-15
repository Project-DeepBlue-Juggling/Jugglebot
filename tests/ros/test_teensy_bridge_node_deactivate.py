"""Node-level tests for the deactivate wiring in teensy_bridge_node.

The pure completion logic is covered in ``tests/teensy_link/test_deactivate.py``.
Here we test the bridge glue:
  * ``_run_deactivate`` fires DEACTIVATE (fire-and-monitor) and observes the leg
    reach IDLE via the telemetry+diagnostic cache;
  * ``_svc_deactivate`` reads the ``deactivate_axes`` param;
  * the telemetry+diagnostic cache → ``DeactivateAxisStatus`` mapping.
"""

from __future__ import annotations

import threading
import time
import types

from controller.teensy_link import (
    RpcMethod, RpcStatus, MsgType, Telemetry, Diagnostic,
)

from tests.ros.test_teensy_bridge_node_read import _build_paired_node

IDLE = 1
CLOSED_LOOP = 8


def _poll(cond, timeout=4.0, dt=0.01):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(dt)
    return cond()


def _teardown(teensy, client, node):
    node.on_shutdown()
    client.stop()
    teensy.stop()


# ── cache → DeactivateAxisStatus mapping ─────────────────────────────────────

def test_deactivate_axis_status_mapping():
    teensy, client, node = _build_paired_node()
    try:
        with node._lock:
            node._latest_telemetry = Telemetry(
                t_teensy_us=0,
                pos_rev=(0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 0.3),
                vel_rps=(0.0, -0.7, 0.0, 0.0, 0.0, 0.0, 0.0))
            node._latest_diag = {
                0: Diagnostic(axis_id=0, axis_state=IDLE, active_errors=0),
                1: Diagnostic(axis_id=1, axis_state=CLOSED_LOOP, active_errors=0x200),
            }
        out = node._deactivate_axis_status([0, 1, 2])
        assert out[0].axis_state == IDLE and abs(out[0].pos_rev) < 1e-6
        assert out[1].axis_state == CLOSED_LOOP and out[1].active_errors == 0x200
        assert 2 not in out          # no diagnostic yet
    finally:
        _teardown(teensy, client, node)


# ── _run_deactivate ──────────────────────────────────────────────────────────

def test_run_deactivate_no_axes():
    teensy, client, node = _build_paired_node()
    try:
        ok, msg = node._run_deactivate([])
        assert not ok and "no axes" in msg
    finally:
        _teardown(teensy, client, node)


def test_run_deactivate_rejected_by_firmware():
    teensy, client, node = _build_paired_node()
    try:
        teensy.on_rpc(int(RpcMethod.DEACTIVATE),
                      lambda req_id, args: (int(RpcStatus.ERR_REJECTED), b""))
        ok, msg = node._run_deactivate([0], poll_dt=0.02)
        assert not ok and "rejected" in msg.lower()
    finally:
        _teardown(teensy, client, node)


def test_deactivate_happy_path_end_to_end():
    teensy, client, node = _build_paired_node()
    try:
        sent = []
        teensy.on_rpc(int(RpcMethod.DEACTIVATE),
                      lambda req_id, args: (sent.append(args) or (int(RpcStatus.OK), b"")))
        result = {}

        def run():
            ok, msg = node._run_deactivate([0], poll_dt=0.02)
            result['ok'], result['msg'] = ok, msg

        t = threading.Thread(target=run)
        t.start()
        assert _poll(lambda: len(sent) >= 1)   # DEACTIVATE fired

        # Mid-descent: still CLOSED_LOOP, far from stow → not done, no arrival yet.
        teensy.send_telemetry(pos_rev=[1.0] + [0.0] * 6, vel_rps=[-2.0] + [0.0] * 6)
        teensy.send_to_jetson(
            int(MsgType.DIAGNOSTIC),
            Diagnostic(axis_id=0, axis_state=CLOSED_LOOP, active_errors=0).pack())
        time.sleep(0.1)
        # Final approach: a CLOSED_LOOP sample lands near stow → arrival latched.
        teensy.send_telemetry(pos_rev=[0.02] + [0.0] * 6, vel_rps=[-0.2] + [0.0] * 6)
        teensy.send_to_jetson(
            int(MsgType.DIAGNOSTIC),
            Diagnostic(axis_id=0, axis_state=CLOSED_LOOP, active_errors=0).pack())
        time.sleep(0.1)
        # Firmware reached STOW + settled + IDLE'd → done.
        teensy.send_telemetry(pos_rev=[0.0] * 7, vel_rps=[0.0] * 7)
        teensy.send_to_jetson(
            int(MsgType.DIAGNOSTIC),
            Diagnostic(axis_id=0, axis_state=IDLE, active_errors=0).pack())

        t.join(timeout=8.0)
        assert not t.is_alive(), "deactivate did not complete"
        assert result.get('ok') is True, result.get('msg')
    finally:
        _teardown(teensy, client, node)


def test_deactivate_idle_with_errors_is_failure():
    teensy, client, node = _build_paired_node()
    try:
        teensy.on_rpc(int(RpcMethod.DEACTIVATE),
                      lambda req_id, args: (int(RpcStatus.OK), b""))
        result = {}

        def run():
            ok, msg = node._run_deactivate([0], poll_dt=0.02)
            result['ok'], result['msg'] = ok, msg

        t = threading.Thread(target=run)
        t.start()
        # A leg that faulted and dropped to IDLE with errors set → FAILURE.
        teensy.send_telemetry(pos_rev=[-0.10] + [0.0] * 6, vel_rps=[0.0] * 7)
        teensy.send_to_jetson(
            int(MsgType.DIAGNOSTIC),
            Diagnostic(axis_id=0, axis_state=IDLE, active_errors=0x1).pack())
        t.join(timeout=8.0)
        assert not t.is_alive(), "deactivate did not complete"
        assert result.get('ok') is False and "active_errors" in result.get('msg', '')
    finally:
        _teardown(teensy, client, node)


# ── _svc_deactivate ──────────────────────────────────────────────────────────

# ── already-at-STOW no-op guard (operator 2026-07-05) ────────────────────────

def _set_cache(node, pos_rev, states):
    """Seed the telemetry + per-axis diagnostic cache directly."""
    with node._lock:
        node._latest_telemetry = Telemetry(
            t_teensy_us=0, pos_rev=tuple(pos_rev), vel_rps=(0.0,) * 7)
        node._latest_diag = {a: Diagnostic(axis_id=a, axis_state=s, active_errors=0)
                             for a, s in enumerate(states)}


def test_deactivate_already_at_stow_skips_leg_move_but_idles_hand():
    """If the platform is already stowed (all target legs IDLE with |pos| <= 0.2 rev),
    _run_deactivate skips the redundant LEG move (NO DEACTIVATE RPC — no CLOSED_LOOP
    re-arm jolt) and reports success — but STILL idles the hand (SET_AXIS_STATE) to keep
    can_node's de-energise-ALL-axes parity."""
    teensy, client, node = _build_paired_node()
    try:
        deact_fired, set_state = [], []
        teensy.on_rpc(int(RpcMethod.DEACTIVATE),
                      lambda rid, a: (deact_fired.append(a), (int(RpcStatus.OK), b""))[1])
        teensy.on_rpc(int(RpcMethod.SET_AXIS_STATE),
                      lambda rid, a: (set_state.append(a), (int(RpcStatus.OK), b""))[1])
        _set_cache(node, pos_rev=(0.0, 0.1, -0.05, 0.15, 0.0, 0.0, 0.0),
                   states=[IDLE] * 7)
        ok, msg = node._run_deactivate(list(range(6)), poll_dt=0.02)
        assert ok and "already at STOW" in msg
        assert deact_fired == [], "DEACTIVATE (leg move) must NOT fire when already stowed"
        assert set_state, "the hand IDLE (SET_AXIS_STATE) must still fire (de-energise all axes)"
    finally:
        _teardown(teensy, client, node)


def test_deactivate_not_stowed_proceeds():
    """A leg still extended (|pos| > 0.2) → NOT stowed → the guard lets the deactivate
    proceed (the DEACTIVATE RPC fires)."""
    teensy, client, node = _build_paired_node()
    try:
        fired = []

        def on_deact(rid, a):
            fired.append(a)
            return (int(RpcStatus.ERR_BUS_DOWN), b"")   # reject → returns fast, past the guard

        teensy.on_rpc(int(RpcMethod.DEACTIVATE), on_deact)
        _set_cache(node, pos_rev=(2.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                   states=[IDLE] * 7)
        ok, msg = node._run_deactivate(list(range(6)), poll_dt=0.02)
        assert fired, "DEACTIVATE must fire when a leg is NOT stowed"
        assert not ok and "rejected" in msg.lower()
    finally:
        _teardown(teensy, client, node)


def test_legs_already_stowed_predicate():
    teensy, client, node = _build_paired_node()
    try:
        # All IDLE, all near stow → stowed.
        _set_cache(node, (0.1, -0.05, 0.0, 0.15, 0.0, 0.0, 0.0), [IDLE] * 7)
        assert node._legs_already_stowed(list(range(6))) is True
        # One leg CLOSED_LOOP → not stowed.
        _set_cache(node, (0.0,) * 7, [IDLE, IDLE, CLOSED_LOOP, IDLE, IDLE, IDLE, IDLE])
        assert node._legs_already_stowed(list(range(6))) is False
        # All IDLE but one leg extended (|pos| > 0.2) → not stowed.
        _set_cache(node, (0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0), [IDLE] * 7)
        assert node._legs_already_stowed(list(range(6))) is False
        # A leg with a NaN position (encoder not ready / a faulted active leg dropped to
        # IDLE reporting NaN) → NOT stowed. `abs(nan) > 0.2` is False, so without the
        # isfinite guard this would MISCLASSIFY as stowed and skip a needed lower.
        _set_cache(node, (0.0, float('nan'), 0.0, 0.0, 0.0, 0.0, 0.0), [IDLE] * 7)
        assert node._legs_already_stowed(list(range(6))) is False
        # Missing diagnostic for a target axis → fail-safe False (proceed).
        with node._lock:
            node._latest_telemetry = Telemetry(t_teensy_us=0, pos_rev=(0.0,) * 7, vel_rps=(0.0,) * 7)
            node._latest_diag = {a: Diagnostic(axis_id=a, axis_state=IDLE, active_errors=0)
                                 for a in range(5)}   # axis 5 absent
        assert node._legs_already_stowed(list(range(6))) is False
    finally:
        _teardown(teensy, client, node)


def test_svc_deactivate_reads_param():
    teensy, client, node = _build_paired_node()
    try:
        node._params['deactivate_axes'] = []
        res = types.SimpleNamespace(success=None, message='')
        out = node._svc_deactivate(None, res)
        assert out.success is False and 'no axes' in out.message
    finally:
        _teardown(teensy, client, node)


# ── ARMING_CONTRACT A3: disarm-before-stow, in-process ─────────

def test_run_deactivate_disarms_first_before_firing():
    """A3 — when armed, _run_deactivate must drop mpc_active BEFORE the
    DEACTIVATE RPC reaches the firmware. In-process ordering is the one
    airtight enforcement point (same thread, synchronous); it covers the
    orchestrator's deactivate, the direct /deactivate service, and the
    shutdown stow. The firmware's reject-DEACTIVATE-while-armed gate is the
    backstop, not the mechanism (its refusal used to strand the platform
    un-stowed — Sharp Edge #6, observed 2026-07-09)."""
    teensy, client, node = _build_paired_node()
    try:
        armed_at_rpc = []
        teensy.on_rpc(
            int(RpcMethod.DEACTIVATE),
            lambda req_id, args: (armed_at_rpc.append(node._mpc_active)
                                  or (int(RpcStatus.ERR_REJECTED), b"")))
        node._mpc_active = True                    # armed (as after A2's arm)
        node._run_deactivate([0], poll_dt=0.02)
        assert node._mpc_active is False           # disarmed by the call
        assert armed_at_rpc, "DEACTIVATE never reached the firmware"
        assert armed_at_rpc[0] is False            # ...and only AFTER the disarm
    finally:
        _teardown(teensy, client, node)


def test_run_deactivate_waits_for_wire_disarm_confirmation():
    """A3 refinement (review 2026-07-15, BLOCKING): the disarm is only STAGED
    for the next 10 Hz heartbeat tick — firing DEACTIVATE immediately races
    s_mpc_active=1 on the Teensy (rejected ~every time). _run_deactivate must
    hold the RPC until the firmware's arm-took bit (T2J bit3) drops."""
    from controller.teensy_link import HeartbeatT2J, LinkState, BusHealth

    def _hb(flags):
        return HeartbeatT2J(
            t_teensy_us=1, link_state=int(LinkState.UP),
            bus1_health=int(BusHealth.OK), bus2_health=int(BusHealth.OK),
            fault_state=0, flags=flags, uptime_ms=0)

    teensy, client, node = _build_paired_node()
    try:
        sent = []
        teensy.on_rpc(int(RpcMethod.DEACTIVATE),
                      lambda req_id, args: (sent.append(node._mpc_active)
                                            or (int(RpcStatus.ERR_REJECTED), b"")))
        # Firmware reports ARMED (bit3 set) — as it would right after the host
        # staged the disarm but before the flags=0 heartbeat landed.
        teensy.send_to_jetson(int(MsgType.HEARTBEAT_T2J), _hb(0x8).pack())
        assert _poll(lambda: node._latest_heartbeat is not None)
        node._mpc_active = True

        result = {}

        def run():
            ok, msg = node._run_deactivate([0], poll_dt=0.02)
            result['ok'], result['msg'] = ok, msg

        t = threading.Thread(target=run)
        t.start()
        time.sleep(0.15)
        assert not sent, "DEACTIVATE fired while the firmware still reported armed"
        # The firmware confirms the disarm (bit3 drops) → the RPC may now fire.
        teensy.send_to_jetson(int(MsgType.HEARTBEAT_T2J), _hb(0x0).pack())
        assert _poll(lambda: len(sent) >= 1), "DEACTIVATE never fired after confirm"
        t.join(timeout=8.0)
        assert not t.is_alive()
        assert sent[0] is False       # host disarmed before the RPC
    finally:
        _teardown(teensy, client, node)
