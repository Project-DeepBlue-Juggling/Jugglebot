"""Node-level tests for the Phase 11 U5 deactivate wiring in teensy_bridge_node.

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

def test_svc_deactivate_reads_param():
    teensy, client, node = _build_paired_node()
    try:
        node._params['deactivate_axes'] = []
        res = types.SimpleNamespace(success=None, message='')
        out = node._svc_deactivate(None, res)
        assert out.success is False and 'no axes' in out.message
    finally:
        _teardown(teensy, client, node)
