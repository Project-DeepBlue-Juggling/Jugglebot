"""Node-level tests for the Phase 9b homing wiring in teensy_bridge_node.

The pure completion logic is covered in ``tests/teensy_link/test_homing.py`` and
the firmware detection xref in ``tests/firmware/test_homing_xref.py``. Here we
test the bridge glue: the telemetry+diagnostic cache → ``HomingAxisStatus``
mapping, the no-axes / param paths, and one end-to-end happy path driving the
real ``RpcClient`` + a ``FakeTeensy`` (``HOME`` responder) with the cache advanced
CLOSED_LOOP → IDLE@home-ref to mimic a completed firmware homing.

Unlike encoder search, the homing *move* runs in firmware (fire-and-monitor): the
HOME RPC returns OK on acceptance and the Jetson observes completion. The
firmware homes one axis at a time, so ``_run_home`` serializes axes.
"""

from __future__ import annotations

import threading
import time
import types

import jugglebot.hardware_config as hw

from controller.teensy_link import (
    RpcMethod, RpcStatus, MsgType, Telemetry, Diagnostic,
)

from tests.ros.test_teensy_bridge_node_read import _build_paired_node

IDLE = 1
CLOSED_LOOP = 8
# set_absolute_position(+0.1, ODrive) reads back -0.1 in Jugglebot telemetry.
HOME_POS = -abs(hw.HOMING_LEG_ABS_POS_REV)


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


# ── cache → HomingAxisStatus mapping ─────────────────────────────────────────

def test_homing_axis_status_mapping():
    teensy, client, node = _build_paired_node()
    try:
        with node._lock:
            node._latest_telemetry = Telemetry(
                t_teensy_us=0,
                pos_rev=(HOME_POS, 1.5, 0.0, 0.0, 0.0, 0.0, 0.3),
                vel_rps=(0.0,) * 7)
            node._latest_diag = {
                0: Diagnostic(axis_id=0, axis_state=IDLE, active_errors=0),
                1: Diagnostic(axis_id=1, axis_state=CLOSED_LOOP, active_errors=0x200),
            }
        s0 = node._homing_axis_status(0)
        assert s0.axis_state == IDLE and abs(s0.pos_rev - HOME_POS) < 1e-6
        s1 = node._homing_axis_status(1)
        assert s1.axis_state == CLOSED_LOOP and s1.active_errors == 0x200
        # axis 2 has no diagnostic yet -> None.
        assert node._homing_axis_status(2) is None
    finally:
        _teardown(teensy, client, node)


def test_homing_axis_status_none_before_telemetry():
    teensy, client, node = _build_paired_node()
    try:
        with node._lock:
            node._latest_telemetry = None
        assert node._homing_axis_status(0) is None
    finally:
        _teardown(teensy, client, node)


# ── _run_home / service glue ─────────────────────────────────────────────────

def test_run_home_no_axes():
    teensy, client, node = _build_paired_node()
    try:
        ok, msg = node._run_home([])
        assert not ok and "no axes" in msg
    finally:
        _teardown(teensy, client, node)


def test_svc_home_reads_param():
    teensy, client, node = _build_paired_node()
    try:
        node._params['home_axes'] = []   # mock param store (conftest)
        res = types.SimpleNamespace(success=None, message='')
        out = node._svc_home(None, res)
        assert out.success is False and 'no axes' in out.message
    finally:
        _teardown(teensy, client, node)


def test_run_home_rejected_by_firmware():
    """Firmware rejects HOME (e.g. bus down / already homing) → reported failure,
    no telemetry needed."""
    teensy, client, node = _build_paired_node()
    try:
        teensy.on_rpc(int(RpcMethod.HOME),
                      lambda req_id, args: (int(RpcStatus.ERR_BUS_DOWN), b""))
        ok, msg = node._run_home([0], poll_dt=0.02)
        assert not ok and "rejected" in msg.lower()
    finally:
        _teardown(teensy, client, node)


# ── end-to-end happy path (real RpcClient + FakeTeensy) ──────────────────────

def test_home_retries_transient_reject():
    """The firmware briefly rejects HOME (ERR_REJECTED) while finishing the prior
    axis (axis_state reads IDLE during STOP_SETTLE before s_phase returns to IDLE).
    _run_home must RETRY until accepted, not fail on the first reject — the
    2026-06-26 race fix (the position-free homing observer can fire the next axis's
    HOME inside that busy window)."""
    teensy, client, node = _build_paired_node()
    try:
        calls = {'n': 0}

        def on_home(req_id, args):
            calls['n'] += 1
            if calls['n'] <= 2:                       # transient busy: reject twice
                return (int(RpcStatus.ERR_REJECTED), b"")
            return (int(RpcStatus.OK), b"")           # then accept

        teensy.on_rpc(int(RpcMethod.HOME), on_home)

        result = {}

        def run():
            ok, msg = node._run_home([0], poll_dt=0.02)
            result['ok'], result['msg'] = ok, msg

        t = threading.Thread(target=run)
        t.start()
        assert _poll(lambda: calls['n'] >= 3)         # retried past the rejections

        # Drive completion: CLOSED_LOOP → IDLE (the observer trusts the trip).
        teensy.send_telemetry(pos_rev=[2.5] + [0.0] * 6, vel_rps=[0.0] * 7)
        teensy.send_to_jetson(
            int(MsgType.DIAGNOSTIC),
            Diagnostic(axis_id=0, axis_state=CLOSED_LOOP, active_errors=0).pack())
        time.sleep(0.1)
        teensy.send_telemetry(pos_rev=[HOME_POS] + [0.0] * 6, vel_rps=[0.0] * 7)
        teensy.send_to_jetson(
            int(MsgType.DIAGNOSTIC),
            Diagnostic(axis_id=0, axis_state=IDLE, active_errors=0).pack())

        t.join(timeout=8.0)
        assert not t.is_alive(), "homing did not complete after retries"
        assert result.get('ok') is True, result.get('msg')
    finally:
        _teardown(teensy, client, node)


def test_home_non_transient_reject_fails_fast():
    """A non-transient rejection (e.g. bus down) is NOT retried — fails promptly."""
    teensy, client, node = _build_paired_node()
    try:
        teensy.on_rpc(int(RpcMethod.HOME),
                      lambda req_id, args: (int(RpcStatus.ERR_BUS_DOWN), b""))
        ok, msg = node._run_home([0], poll_dt=0.02)
        assert not ok and "rejected" in msg.lower()
    finally:
        _teardown(teensy, client, node)


def test_home_happy_path_end_to_end():
    teensy, client, node = _build_paired_node()
    try:
        sent = []

        def on_home(req_id, args):
            sent.append(args)
            return (int(RpcStatus.OK), b"")   # firmware accepts the move

        teensy.on_rpc(int(RpcMethod.HOME), on_home)

        result = {}

        def run():
            ok, msg = node._run_home([0], poll_dt=0.02)
            result['ok'], result['msg'] = ok, msg

        t = threading.Thread(target=run)
        t.start()

        # First step fires HOME — wait for it to reach the Teensy.
        assert _poll(lambda: len(sent) >= 1)

        # Mimic the firmware move: axis 0 enters CLOSED_LOOP, position arbitrary.
        teensy.send_telemetry(pos_rev=[2.5] + [0.0] * 6, vel_rps=[0.0] * 7)
        teensy.send_to_jetson(
            int(MsgType.DIAGNOSTIC),
            Diagnostic(axis_id=0, axis_state=CLOSED_LOOP, active_errors=0).pack())
        time.sleep(0.1)

        # Completion: hardstop found → IDLE with the encoder snapped to the home
        # reference (-0.1 rev, Jugglebot convention), no errors.
        teensy.send_telemetry(pos_rev=[HOME_POS] + [0.0] * 6, vel_rps=[0.0] * 7)
        teensy.send_to_jetson(
            int(MsgType.DIAGNOSTIC),
            Diagnostic(axis_id=0, axis_state=IDLE, active_errors=0).pack())

        t.join(timeout=8.0)
        assert not t.is_alive(), "homing did not complete"
        assert result.get('ok') is True, result.get('msg')
        assert len(sent) == 1   # exactly one HOME issued for the single axis
    finally:
        _teardown(teensy, client, node)
