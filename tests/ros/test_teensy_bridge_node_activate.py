"""Node-level tests for the configure + activate wiring in
teensy_bridge_node.

The pure completion logic is covered in ``tests/teensy_link/test_activate.py``.
Here we test the bridge glue:
  * ``_run_configure`` issues the per-leg gains/limits/PASSTHROUGH RPCs and reports
    success/failure;
  * ``_run_activate`` fires ACTIVATE (fire-and-monitor) and observes the leg reach
    the active pose via the telemetry cache;
  * ``_svc_home`` applies ``_run_configure`` after a successful homing (the
    operator's "set after every homing");
  * the telemetry+diagnostic cache → ``ActivateAxisStatus`` mapping.
"""

from __future__ import annotations

import threading
import time
import types

import jugglebot.hardware_config as hw

from controller.teensy_link import (
    RpcMethod, RpcStatus, MsgType, Telemetry, Diagnostic,
)

from tests.ros._bridge_harness import _build_paired_node, _poll, _teardown

IDLE = 1
CLOSED_LOOP = 8

# The four RPCs _run_configure issues per leg.
_CONFIG_METHODS = (
    RpcMethod.SET_POS_GAIN, RpcMethod.SET_VEL_GAINS,
    RpcMethod.SET_VEL_CURR_LIMITS, RpcMethod.SET_CONTROLLER_MODE,
)


def _ok_config_handlers(teensy, seen):
    """Register OK responders for the four configure RPCs, recording method ids."""
    for m in _CONFIG_METHODS:
        mi = int(m)
        teensy.on_rpc(mi, (lambda mid: (lambda req_id, args: (seen.append(mid)
                                                              or (int(RpcStatus.OK), b""))))(mi))


# ── cache → ActivateAxisStatus mapping ───────────────────────────────────────

def test_activate_axis_status_mapping():
    teensy, client, node = _build_paired_node()
    try:
        with node._lock:
            node._latest_telemetry = Telemetry(
                t_teensy_us=0,
                pos_rev=(2.19, 1.5, 0.0, 0.0, 0.0, 0.0, 0.3),
                vel_rps=(0.0, 0.7, 0.0, 0.0, 0.0, 0.0, 0.0))
            node._latest_diag = {
                0: Diagnostic(axis_id=0, axis_state=CLOSED_LOOP, active_errors=0),
                1: Diagnostic(axis_id=1, axis_state=CLOSED_LOOP, active_errors=0x200),
            }
        out = node._activate_axis_status([0, 1, 2])
        assert abs(out[0].pos_rev - 2.19) < 1e-6 and abs(out[0].vel_rps) < 1e-6
        assert abs(out[1].vel_rps - 0.7) < 1e-6 and out[1].active_errors == 0x200
        assert 2 not in out          # no diagnostic yet
    finally:
        _teardown(teensy, client, node)


# ── _run_configure ───────────────────────────────────────────────────────────

def test_run_configure_no_axes():
    teensy, client, node = _build_paired_node()
    try:
        ok, msg = node._run_configure([])
        assert not ok and "no axes" in msg
    finally:
        _teardown(teensy, client, node)


def test_run_configure_happy_path_issues_all_rpcs():
    teensy, client, node = _build_paired_node()
    try:
        seen = []
        _ok_config_handlers(teensy, seen)
        ok, msg = node._run_configure([0])
        assert ok, msg
        # All four config RPCs were issued for the one axis.
        assert sorted(seen) == sorted(int(m) for m in _CONFIG_METHODS)
    finally:
        _teardown(teensy, client, node)


def test_run_configure_reports_failure():
    teensy, client, node = _build_paired_node()
    try:
        # pos_gain OK, but vel_gains rejected → configure fails, names the step.
        teensy.on_rpc(int(RpcMethod.SET_POS_GAIN),
                      lambda req_id, args: (int(RpcStatus.OK), b""))
        teensy.on_rpc(int(RpcMethod.SET_VEL_GAINS),
                      lambda req_id, args: (int(RpcStatus.ERR_BUS_DOWN), b""))
        teensy.on_rpc(int(RpcMethod.SET_VEL_CURR_LIMITS),
                      lambda req_id, args: (int(RpcStatus.OK), b""))
        teensy.on_rpc(int(RpcMethod.SET_CONTROLLER_MODE),
                      lambda req_id, args: (int(RpcStatus.OK), b""))
        ok, msg = node._run_configure([0])
        assert not ok and "vel_gains" in msg and "FAILED" in msg
    finally:
        _teardown(teensy, client, node)


def test_svc_configure_reads_param():
    teensy, client, node = _build_paired_node()
    try:
        node._params['configure_axes'] = []
        res = types.SimpleNamespace(success=None, message='')
        out = node._svc_configure(None, res)
        assert out.success is False and 'no axes' in out.message
    finally:
        _teardown(teensy, client, node)


# ── _run_activate ────────────────────────────────────────────────────────────

def test_run_activate_no_axes():
    teensy, client, node = _build_paired_node()
    try:
        ok, msg = node._run_activate([])
        assert not ok and "no axes" in msg
    finally:
        _teardown(teensy, client, node)


def test_run_activate_rejected_by_firmware():
    teensy, client, node = _build_paired_node()
    try:
        teensy.on_rpc(int(RpcMethod.ACTIVATE),
                      lambda req_id, args: (int(RpcStatus.ERR_BUS_DOWN), b""))
        ok, msg = node._run_activate([0], poll_dt=0.02)
        assert not ok and "rejected" in msg.lower()
    finally:
        _teardown(teensy, client, node)


def test_activate_happy_path_end_to_end():
    teensy, client, node = _build_paired_node()
    try:
        sent = []
        teensy.on_rpc(int(RpcMethod.ACTIVATE),
                      lambda req_id, args: (sent.append(args) or (int(RpcStatus.OK), b"")))
        target = float(hw.JB_OP_ACTIVATE_POSITION_REVS[0])
        result = {}

        def run():
            ok, msg = node._run_activate([0], poll_dt=0.02)
            result['ok'], result['msg'] = ok, msg

        t = threading.Thread(target=run)
        t.start()
        assert _poll(lambda: len(sent) >= 1)   # ACTIVATE fired

        # Mid-move: below target, still moving → not done.
        teensy.send_telemetry(pos_rev=[0.5] + [0.0] * 6, vel_rps=[2.0] + [0.0] * 6)
        teensy.send_to_jetson(
            int(MsgType.DIAGNOSTIC),
            Diagnostic(axis_id=0, axis_state=CLOSED_LOOP, active_errors=0).pack())
        time.sleep(0.1)
        # Reached the active pose and settled.
        teensy.send_telemetry(pos_rev=[target] + [0.0] * 6, vel_rps=[0.0] * 7)
        teensy.send_to_jetson(
            int(MsgType.DIAGNOSTIC),
            Diagnostic(axis_id=0, axis_state=CLOSED_LOOP, active_errors=0).pack())

        t.join(timeout=8.0)
        assert not t.is_alive(), "activate did not complete"
        assert result.get('ok') is True, result.get('msg')
    finally:
        _teardown(teensy, client, node)


# ── /home applies configure (the integration glue) ───────────────────────────

def test_svc_home_applies_configure_on_success(monkeypatch):
    teensy, client, node = _build_paired_node()
    try:
        node._params['home_axes'] = [0, 1]
        calls = {}
        monkeypatch.setattr(node, "_run_home", lambda axes: (True, f"homed {axes}"))
        monkeypatch.setattr(node, "_run_configure",
                            lambda axes: (calls.setdefault('configure', axes), (True, "cfg"))[1])
        res = types.SimpleNamespace(success=None, message='')
        out = node._svc_home(None, res)
        assert out.success is True
        # configure was applied to the homed axes, and its message is appended.
        assert calls['configure'] == [0, 1]
        assert "homed" in out.message and "cfg" in out.message
    finally:
        _teardown(teensy, client, node)


def test_svc_home_skips_configure_on_homing_failure(monkeypatch):
    teensy, client, node = _build_paired_node()
    try:
        node._params['home_axes'] = [0]
        called = {'configure': False}
        monkeypatch.setattr(node, "_run_home", lambda axes: (False, "home FAILED"))

        def _cfg(axes):
            called['configure'] = True
            return (True, "cfg")
        monkeypatch.setattr(node, "_run_configure", _cfg)
        res = types.SimpleNamespace(success=None, message='')
        out = node._svc_home(None, res)
        assert out.success is False and not called['configure']
    finally:
        _teardown(teensy, client, node)
