"""Behaviour tests for the orchestrator-facing cold-start conduit.

These exercise the four thin wrappers the orchestrator drives cold-start through,
registered directly on ``teensy_bridge_node``:

* ``home_motors`` action  → ``_do_home`` (home legs+hand → persist is_homed → configure)
* ``activate_or_deactivate`` service → ``_run_activate`` + ``_run_configure`` (the
  PASSTHROUGH fold) / ``_run_deactivate``
* ``get_platform_tilt`` service → ``relay_read_tilt`` (retry + validity bound +
  NaN-on-failure)
* ``set_level_state`` subscriber → ``_write_level_state`` (STATE_WRITE preserving
  is_homed)

The ``(name, type)`` interface contract vs the orchestrator lives in
``test_orchestrator_conduit_contract.py``; here we test the behaviour. The pure
verbs (``_run_home`` / ``_run_activate`` / ``_run_deactivate`` / ``relay_read_tilt``
/ ``_write_is_homed``) are covered by their own suites — most tests here STUB them
to isolate the conduit's wrapping (dispatch, the activate→configure fold, the
NaN shape, is_homed preservation), matching the isolation pattern in
``test_teensy_bridge_node_coldstart.py::test_home_service_persists_is_homed``. One
end-to-end action test drives the REAL ``_run_home`` (FakeTeensy HOME responder +
injected CLOSED_LOOP→IDLE frames) to prove the action truly invokes the observer.
"""

from __future__ import annotations

import math
import struct
import threading
import time
import types

import pytest

import jugglebot.hardware_config as hw

from controller.teensy_link import (
    RpcMethod, RpcStatus, MsgType, Telemetry, Diagnostic, PlatformFrame,
)
from controller.teensy_link import rpc_args
from controller.teensy_link.homing import HOMING_RUNNING, HOMING_OK  # firmware-reported homing outcome (see logbook 2026-07-05-canhub-hardening-18a-homing-result-uplink)
from rclpy.action import GoalResponse

from jugglebot_interfaces.srv import ActivateOrDeactivate, GetTiltReadingService
from std_msgs.msg import Float64MultiArray

from tests.ros.test_teensy_bridge_node_read import _build_paired_node

IDLE = 1
CLOSED_LOOP = 8
HOME_POS = -abs(hw.HOMING_LEG_ABS_POS_REV)
_TILT_ID = 0x7DE   # PlatformCanId::TILT_READING


def _teardown(teensy, client, node):
    node.on_shutdown()
    client.stop()
    teensy.stop()


def _poll(cond, timeout=4.0, dt=0.01):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(dt)
    return cond()


def _platform_frame(can_id, data: bytes) -> bytes:
    pf = PlatformFrame(t_bridge_us=1234, can_id=can_id, dlc=len(data),
                       data=tuple(data) + (0,) * (8 - len(data)))
    return pf.pack()


class _GoalHandle:
    """Minimal ActionServer goal-handle stand-in (execute reads no request fields)."""
    def __init__(self):
        self.request = types.SimpleNamespace()
        self.outcome = None

    def succeed(self):
        self.outcome = 'succeed'

    def abort(self):
        self.outcome = 'abort'


# ════════════════════════════════════════════════════════════════
# home_motors action
# ════════════════════════════════════════════════════════════════

def test_home_action_success_returns_result_and_persists_is_homed():
    """A successful home_motors goal: Result.success=True, goal succeeded, and
    is_homed persisted True via STATE_WRITE (through _do_home)."""
    teensy, client, node = _build_paired_node()
    try:
        node._run_home = lambda axes, **kw: (True, "homed")
        node._run_configure = lambda axes, **kw: (True, "configured")
        writes = {}
        teensy.on_rpc(int(RpcMethod.STATE_WRITE),
                      lambda rid, a: (writes.__setitem__('args', a), (int(RpcStatus.OK), b""))[1])

        gh = _GoalHandle()
        result = node._home_action_execute(gh)
        assert result.success is True
        assert gh.outcome == 'succeed'
        # is_homed persisted True (levelling defaults False here).
        assert writes['args'] == rpc_args.encode_state_write(True, False, 0.0, 0.0)
        assert node._cold_start_state.is_homed is True
    finally:
        _teardown(teensy, client, node)


def test_home_action_failure_aborts_and_persists_false():
    """A FAILED home_motors goal: Result.success=False, goal aborted, is_homed
    persisted False (the home RESULT — can_node:847 parity), configure NOT run."""
    teensy, client, node = _build_paired_node()
    try:
        from jugglebot.teensy_bridge_node import RelayRobotState
        with node._lock:                          # pretend already homed + levelled
            node._cold_start_state = RelayRobotState(True, True, 0.01, 0.02)
        cfg = {'called': False}
        node._run_home = lambda axes, **kw: (False, "home FAILED")
        node._run_configure = lambda axes, **kw: (cfg.__setitem__('called', True),
                                                   (True, "cfg"))[1]
        writes = {}
        teensy.on_rpc(int(RpcMethod.STATE_WRITE),
                      lambda rid, a: (writes.__setitem__('args', a), (int(RpcStatus.OK), b""))[1])

        gh = _GoalHandle()
        result = node._home_action_execute(gh)
        assert result.success is False
        assert gh.outcome == 'abort'
        assert cfg['called'] is False                       # configure skipped on failure
        # is_homed cleared; levelling/pose preserved (read-modify-write).
        assert writes['args'] == rpc_args.encode_state_write(False, True, 0.01, 0.02)
    finally:
        _teardown(teensy, client, node)


def test_home_single_flight_guard():
    """The single-flight guard lives in _do_home() (the real enforcement point,
    covering the /home Trigger AND the home_motors action + any future caller);
    _home_action_goal peeks it for a clean action REJECT."""
    teensy, client, node = _build_paired_node()
    try:
        # Idle: the goal is accepted (peek sees no in-flight home).
        assert node._home_action_goal(None) == GoalResponse.ACCEPT
        # Simulate an in-flight home: the goal is rejected AND a concurrent _do_home
        # (e.g. a /home Trigger racing the action) is rejected by the guard itself.
        with node._home_lock:
            node._home_in_progress = True
        assert node._home_action_goal(None) == GoalResponse.REJECT
        ok, msg = node._do_home()
        assert ok is False and "already in progress" in msg
        # Release the simulated in-flight; a real home runs AND clears the guard in
        # its finally, so the next goal is accepted again.
        with node._home_lock:
            node._home_in_progress = False
        node._run_home = lambda axes, **kw: (True, "homed")
        node._run_configure = lambda axes, **kw: (True, "configured")
        teensy.on_rpc(int(RpcMethod.STATE_WRITE), lambda rid, a: (int(RpcStatus.OK), b""))
        ok, msg = node._do_home()
        assert ok is True
        assert node._home_in_progress is False               # cleared in finally
        assert node._home_action_goal(None) == GoalResponse.ACCEPT
    finally:
        _teardown(teensy, client, node)


def test_home_action_end_to_end_drives_real_run_home():
    """The action truly invokes the real _run_home observer: HOME responder +
    injected CLOSED_LOOP→IDLE@home-ref frames complete a single-axis home."""
    teensy, client, node = _build_paired_node()
    try:
        node._params['home_axes'] = [0]                 # single leg (excludes hand)
        node._run_configure = lambda axes, **kw: (True, "configured")   # isolate homing
        teensy.on_rpc(int(RpcMethod.HOME), lambda rid, a: (int(RpcStatus.OK), b""))
        teensy.on_rpc(int(RpcMethod.STATE_WRITE), lambda rid, a: (int(RpcStatus.OK), b""))

        gh = _GoalHandle()
        out = {}

        def run():
            out['result'] = node._home_action_execute(gh)

        t = threading.Thread(target=run)
        t.start()
        # Drive completion: CLOSED_LOOP then IDLE at the home reference.
        teensy.send_telemetry(pos_rev=[2.5] + [0.0] * 6, vel_rps=[0.0] * 7)
        teensy.send_to_jetson(int(MsgType.DIAGNOSTIC),
                              Diagnostic(axis_id=0, axis_state=CLOSED_LOOP, active_errors=0,
                                         homing_result=HOMING_RUNNING).pack())
        time.sleep(0.1)
        teensy.send_telemetry(pos_rev=[HOME_POS] + [0.0] * 6, vel_rps=[0.0] * 7)
        teensy.send_to_jetson(int(MsgType.DIAGNOSTIC),
                              Diagnostic(axis_id=0, axis_state=IDLE, active_errors=0,
                                         homing_result=HOMING_OK).pack())

        t.join(timeout=8.0)
        assert not t.is_alive(), "home action did not complete"
        assert out['result'].success is True
        assert gh.outcome == 'succeed'
    finally:
        _teardown(teensy, client, node)


# ════════════════════════════════════════════════════════════════
# activate_or_deactivate
# ════════════════════════════════════════════════════════════════

def _req(command):
    r = ActivateOrDeactivate.Request()
    r.command = command
    return r


def test_activate_folds_configure_in_order():
    """activate → _run_activate THEN _run_configure (the TRAP_TRAJ→PASSTHROUGH fold,
    rows 27/28), in that order, success=True."""
    teensy, client, node = _build_paired_node()
    try:
        node._params['activate_axes'] = [0]
        calls = []
        node._run_activate = lambda axes, **kw: (calls.append(('activate', axes)),
                                                 (True, "activated"))[1]
        node._run_configure = lambda axes, **kw: (calls.append(('configure', axes)),
                                                  (True, "configured"))[1]
        res = node._svc_activate_or_deactivate(_req('activate'),
                                               ActivateOrDeactivate.Response())
        assert res.success is True
        assert [c[0] for c in calls] == ['activate', 'configure']   # order matters
    finally:
        _teardown(teensy, client, node)


def test_activate_failure_skips_configure():
    """If _run_activate fails, the configure fold is skipped and the result fails."""
    teensy, client, node = _build_paired_node()
    try:
        node._params['activate_axes'] = [0]
        cfg = {'called': False}
        node._run_activate = lambda axes, **kw: (False, "activate FAILED")
        node._run_configure = lambda axes, **kw: (cfg.__setitem__('called', True),
                                                  (True, "cfg"))[1]
        res = node._svc_activate_or_deactivate(_req('activate'),
                                               ActivateOrDeactivate.Response())
        assert res.success is False
        assert cfg['called'] is False
    finally:
        _teardown(teensy, client, node)


def test_activate_configure_failure_fails_result():
    """activate succeeds but the configure fold fails → overall failure (legs at
    pose but not interp-ready is a real failure for run_mpc)."""
    teensy, client, node = _build_paired_node()
    try:
        node._params['activate_axes'] = [0]
        node._run_activate = lambda axes, **kw: (True, "activated")
        node._run_configure = lambda axes, **kw: (False, "configure FAILED")
        res = node._svc_activate_or_deactivate(_req('activate'),
                                               ActivateOrDeactivate.Response())
        assert res.success is False
        assert "configure FAILED" in res.message
    finally:
        _teardown(teensy, client, node)


def test_deactivate_runs_deactivate_only():
    """deactivate → _run_deactivate; NO configure fold (deactivate ends IDLE)."""
    teensy, client, node = _build_paired_node()
    try:
        node._params['deactivate_axes'] = [0]
        calls = []
        node._run_deactivate = lambda axes, **kw: (calls.append('deactivate'),
                                                   (True, "deactivated"))[1]
        node._run_configure = lambda axes, **kw: (calls.append('configure'),
                                                  (True, "cfg"))[1]
        res = node._svc_activate_or_deactivate(_req('deactivate'),
                                               ActivateOrDeactivate.Response())
        assert res.success is True
        assert calls == ['deactivate']            # configure NOT folded on deactivate
    finally:
        _teardown(teensy, client, node)


def test_activate_invalid_command():
    teensy, client, node = _build_paired_node()
    try:
        res = node._svc_activate_or_deactivate(_req('spin'),
                                               ActivateOrDeactivate.Response())
        assert res.success is False
        assert "Invalid command" in res.message
    finally:
        _teardown(teensy, client, node)


# ════════════════════════════════════════════════════════════════
# get_platform_tilt
# ════════════════════════════════════════════════════════════════

def test_get_platform_tilt_success():
    """A valid tilt reply → tilt_xy = [tx, ty] and a finite tilt_quat."""
    teensy, client, node = _build_paired_node()
    try:
        def handler(rid, a):
            teensy.send_to_jetson(int(MsgType.PLATFORM_FRAME),
                                  _platform_frame(_TILT_ID, struct.pack("<ff", 0.05, -0.07)))
            return (int(RpcStatus.OK), b"")
        teensy.on_rpc(int(RpcMethod.TILT_READ), handler)

        res = node._svc_get_platform_tilt(object(), GetTiltReadingService.Response())
        assert res.tilt_xy[0] == pytest.approx(0.05)
        assert res.tilt_xy[1] == pytest.approx(-0.07)
        assert not math.isnan(res.tilt_quat.w)
    finally:
        _teardown(teensy, client, node)


def test_get_platform_tilt_failure_returns_nan():
    """A relay failure (bus down, fail-fast) → the NaN shape the orchestrator
    consumes (NaN tilt_xy → operation_result=False → FAULT)."""
    teensy, client, node = _build_paired_node()
    try:
        teensy.on_rpc(int(RpcMethod.TILT_READ),
                      lambda rid, a: (int(RpcStatus.ERR_BUS_DOWN), b""))
        res = node._svc_get_platform_tilt(object(), GetTiltReadingService.Response())
        assert len(res.tilt_xy) == 2
        assert math.isnan(res.tilt_xy[0]) and math.isnan(res.tilt_xy[1])
        assert math.isnan(res.tilt_quat.w)
    finally:
        _teardown(teensy, client, node)


def test_get_platform_tilt_out_of_range_returns_nan():
    """A reading beyond JB_OP_MAX_VALID_TILT_RAD is rejected (retried, then NaN) —
    the validity-bound robustness ported from can_node (per the can_node<->Teensy parity audit)."""
    teensy, client, node = _build_paired_node()
    try:
        bad = hw.JB_OP_MAX_VALID_TILT_RAD + 0.5

        def handler(rid, a):
            teensy.send_to_jetson(int(MsgType.PLATFORM_FRAME),
                                  _platform_frame(_TILT_ID, struct.pack("<ff", bad, 0.0)))
            return (int(RpcStatus.OK), b"")
        teensy.on_rpc(int(RpcMethod.TILT_READ), handler)

        res = node._svc_get_platform_tilt(object(), GetTiltReadingService.Response())
        assert math.isnan(res.tilt_xy[0]) and math.isnan(res.tilt_xy[1])
    finally:
        _teardown(teensy, client, node)


# ════════════════════════════════════════════════════════════════
# set_level_state
# ════════════════════════════════════════════════════════════════

def test_set_level_state_persists_preserving_is_homed():
    """set_level_state([1.0, tx, ty]) writes levelling_complete + pose via STATE_WRITE
    while PRESERVING is_homed already in the cache (read-modify-write)."""
    teensy, client, node = _build_paired_node()
    try:
        from jugglebot.teensy_bridge_node import RelayRobotState
        with node._lock:                          # already homed, not yet levelled
            node._cold_start_state = RelayRobotState(True, False, 0.0, 0.0)
        writes = {}
        teensy.on_rpc(int(RpcMethod.STATE_WRITE),
                      lambda rid, a: (writes.__setitem__('args', a), (int(RpcStatus.OK), b""))[1])

        node._sub_set_level_state(Float64MultiArray(data=[1.0, 0.05, -0.03]))
        # is_homed preserved True; levelling True; pose written.
        assert writes['args'] == rpc_args.encode_state_write(True, True, 0.05, -0.03)
        assert node._cold_start_state.is_homed is True
        assert node._cold_start_state.levelling_complete is True
        assert node._cold_start_state.pose_offset_tiltX == pytest.approx(0.05)
    finally:
        _teardown(teensy, client, node)


def test_set_level_state_short_message_is_ignored():
    """A malformed (<3 element) message logs + returns without a write (no crash)."""
    teensy, client, node = _build_paired_node()
    try:
        called = {'write': False}
        teensy.on_rpc(int(RpcMethod.STATE_WRITE),
                      lambda rid, a: (called.__setitem__('write', True), (int(RpcStatus.OK), b""))[1])
        node._sub_set_level_state(Float64MultiArray(data=[1.0]))
        time.sleep(0.05)
        assert called['write'] is False
    finally:
        _teardown(teensy, client, node)

