"""Node-level tests for the Phase 9a encoder-search wiring in teensy_bridge_node.

The pure sequencing is covered exhaustively in
``tests/teensy_link/test_encoder_search.py``. Here we test the bridge glue: the
telemetry+diagnostic cache → ``AxisStatus`` mapping, the no-axes / param paths,
and one end-to-end happy path driving the real ``RpcClient`` + a ``FakeTeensy``
(``SET_AXIS_STATE`` responder) with the telemetry cache advanced NaN → finite to
mimic a completed index search.
"""

from __future__ import annotations

import threading
import time
import types

from controller.teensy_link import (
    RpcMethod, RpcStatus, MsgType, Telemetry, Diagnostic,
)
from controller.teensy_link import rpc_args

from tests.ros.test_teensy_bridge_node_read import _build_paired_node

IDLE = 1
SEARCH = 6


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


# ── cache → AxisStatus mapping ───────────────────────────────────────────────

def test_encoder_axis_status_mapping():
    teensy, client, node = _build_paired_node()
    try:
        nan = float('nan')
        with node._lock:
            node._latest_telemetry = Telemetry(
                t_teensy_us=0,
                pos_rev=(nan, 1.5, nan, 0.0, nan, nan, 0.3),
                vel_rps=(0.0,) * 7)
            node._latest_diag = {
                0: Diagnostic(axis_id=0, axis_state=SEARCH, active_errors=0),
                1: Diagnostic(axis_id=1, axis_state=IDLE, active_errors=0x200),
                # axis 2: no diagnostic -> omitted from the status dict
            }
        status = node._encoder_axis_status([0, 1, 2])
        assert set(status) == {0, 1}
        assert status[0].axis_state == SEARCH and status[0].pos_finite is False
        assert status[1].axis_state == IDLE and status[1].pos_finite is True
        assert status[1].active_errors == 0x200
    finally:
        _teardown(teensy, client, node)


def test_encoder_axis_status_empty_before_telemetry():
    teensy, client, node = _build_paired_node()
    try:
        with node._lock:
            node._latest_telemetry = None
        assert node._encoder_axis_status([0]) == {}
    finally:
        _teardown(teensy, client, node)


# ── _run_encoder_search / service glue ───────────────────────────────────────

def test_run_encoder_search_no_axes():
    teensy, client, node = _build_paired_node()
    try:
        ok, msg = node._run_encoder_search([])
        assert not ok and "no axes" in msg
    finally:
        _teardown(teensy, client, node)


def test_svc_encoder_search_reads_param():
    teensy, client, node = _build_paired_node()
    try:
        node._params['encoder_search_axes'] = []   # mock param store (conftest)
        res = types.SimpleNamespace(success=None, message='')
        out = node._svc_encoder_search(None, res)
        assert out.success is False and 'no axes' in out.message
    finally:
        _teardown(teensy, client, node)


# ── end-to-end happy path (real RpcClient + FakeTeensy) ──────────────────────

def test_encoder_search_happy_path_end_to_end():
    teensy, client, node = _build_paired_node()
    try:
        sent = []

        def on_set_state(req_id, args):
            sent.append(args)
            return (int(RpcStatus.OK), b"")

        teensy.on_rpc(int(RpcMethod.SET_AXIS_STATE), on_set_state)
        teensy.on_rpc(int(RpcMethod.CLEAR_ERRORS),
                      lambda req_id, args: (int(RpcStatus.OK), b""))

        result = {}

        def run():
            ok, msg = node._run_encoder_search([0], poll_dt=0.02)
            result['ok'], result['msg'] = ok, msg

        t = threading.Thread(target=run)
        t.start()

        nan = float('nan')
        # The first step commands the search — wait for it to reach the Teensy.
        assert _poll(lambda: len(sent) >= 1)

        # Mimic the search running: axis 0 in ENCODER_INDEX_SEARCH, encoder NaN.
        teensy.send_telemetry(pos_rev=[nan] * 7, vel_rps=[0.0] * 7)
        teensy.send_to_jetson(
            int(MsgType.DIAGNOSTIC),
            Diagnostic(axis_id=0, axis_state=SEARCH, active_errors=0).pack())
        time.sleep(0.1)

        # Completion: back to IDLE with a finite encoder position, no errors.
        teensy.send_telemetry(pos_rev=[1.234] + [nan] * 6, vel_rps=[0.0] * 7)
        teensy.send_to_jetson(
            int(MsgType.DIAGNOSTIC),
            Diagnostic(axis_id=0, axis_state=IDLE, active_errors=0).pack())

        t.join(timeout=8.0)
        assert not t.is_alive(), "encoder search did not complete"
        assert result.get('ok') is True, result.get('msg')
        # The first command was SET_AXIS_STATE(axis=0, ENCODER_INDEX_SEARCH).
        assert sent[0] == rpc_args.encode_set_axis_state(0, SEARCH)
    finally:
        _teardown(teensy, client, node)
