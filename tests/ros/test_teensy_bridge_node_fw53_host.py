"""can-bridge FW 53 (I-FW-19), the bridge's host side (unit fw53-u6a).

* A Leg IDLE / encoder-search / reboot / SDO write that the firmware Stow gate
  refuses (ERR_REJECTED) is named "Platform not at Stow" with
  ``rpc_args.STOW_REFUSAL_HINT``, at the one RPC choke point ``_call_rpc``.
* The activate and deactivate observers read the FW 53 Diagnostic LEG_HELD bit
  from the diag cache, and ``_run_activate`` names a returned rise and a held one.
* ``/link_status`` carries ``legs_held`` and ``rig_bench``.
* ``_run_home`` logs the expected HOME refusal at INFO, never WARN.

Each test names the mutation it catches. ROS 2 is mocked by tests/ros/conftest.py.
"""
from __future__ import annotations

import threading
import types

from teensy_link import RpcMethod, RpcStatus, Telemetry, Diagnostic
from teensy_link import rpc_args
from teensy_link import protocol as p
from teensy_link.activate import (
    AxisStatus as ActivateAxisStatus, DIAG_FLAG_LEG_HELD, DIAG_FLAG_RIG_BENCH,
)
from tests.ros._bridge_harness import _build_paired_node, _poll, _teardown

import jugglebot.hardware_config as hw
from jugglebot import teensy_bridge_node as tbn

IDLE = 1
CLOSED_LOOP = 8
ENCODER_INDEX_SEARCH = 6
_HAND = p.NUM_LEGS


def _reject(teensy, method):
    calls = []
    teensy.on_rpc(int(method), lambda req_id, args: (
        calls.append(args) or (int(RpcStatus.ERR_REJECTED), b"")))
    return calls


# ── Stow-gate refusals are named (mutation: drop stow_gate= in a wrapper) ──

def test_leg_idle_refusal_names_platform_not_at_stow():
    teensy, client, node = _build_paired_node()
    try:
        _reject(teensy, RpcMethod.SET_AXIS_STATE)
        for state in (IDLE, ENCODER_INDEX_SEARCH):
            ok, msg, _ = node.teensy_set_axis_state(2, state)
            assert ok is False
            assert 'Platform not at Stow' in msg and rpc_args.STOW_REFUSAL_HINT in msg
    finally:
        _teardown(teensy, client, node)


def test_reboot_and_sdo_write_refusals_name_platform_not_at_stow():
    teensy, client, node = _build_paired_node()
    try:
        _reject(teensy, RpcMethod.REBOOT_ODRIVES)
        _reject(teensy, RpcMethod.SDO_WRITE)
        ok, msg, _ = node.teensy_reboot()                     # AXIS_ALL
        assert ok is False and 'Platform not at Stow' in msg
        ok, msg, _ = node.teensy_sdo_write(0, 100, 1)
        assert ok is False and 'Platform not at Stow' in msg
    finally:
        _teardown(teensy, client, node)


def test_hand_idle_and_leg_closed_loop_refusals_are_not_called_stow():
    """The Hand is exempt from the Stow gate, and a Leg CLOSED_LOOP refusal is a
    Homing question: neither may be misnamed (mutation: hint on any ERR_REJECTED)."""
    teensy, client, node = _build_paired_node()
    try:
        _reject(teensy, RpcMethod.SET_AXIS_STATE)
        _, msg, _ = node.teensy_set_axis_state(_HAND, IDLE)
        assert 'Stow' not in msg
        _, msg, _ = node.teensy_set_axis_state(0, CLOSED_LOOP)
        assert 'Stow' not in msg
    finally:
        _teardown(teensy, client, node)


def test_leg_idle_refused_while_armed_is_a_stow_refusal_not_an_armed_one():
    """IDLE is never Armed-gated (FW 52), so an IDLE refused while Armed is the Stow
    gate (mutation: the old method-only Armed check names it Armed)."""
    teensy, client, node = _build_paired_node()
    try:
        _reject(teensy, RpcMethod.SET_AXIS_STATE)
        node._mpc_active = True
        _, msg, _ = node.teensy_set_axis_state(1, IDLE)
        assert 'Platform not at Stow' in msg
        assert rpc_args.ARMED_REFUSAL_HINT not in msg
    finally:
        node._mpc_active = False
        _teardown(teensy, client, node)


# ── LEG_HELD reaches the observers (mutation: drop held= in either builder) ──

def _cache(node, flags_by_axis, pos=0.5):
    with node._lock:
        node._latest_telemetry = Telemetry(
            t_teensy_us=0, pos_rev=(pos,) * 7, vel_rps=(0.0,) * 7)
        node._latest_diag = {
            a: Diagnostic(axis_id=a, axis_state=CLOSED_LOOP, active_errors=0, flags=f)
            for a, f in flags_by_axis.items()}


def test_axis_status_builders_carry_the_held_bit():
    teensy, client, node = _build_paired_node()
    try:
        _cache(node, {0: DIAG_FLAG_LEG_HELD, 1: 0})
        act = node._activate_axis_status([0, 1])
        deact = node._deactivate_axis_status([0, 1])
        assert act[0].held is True and act[1].held is False
        assert deact[0].held is True and deact[1].held is False
    finally:
        _teardown(teensy, client, node)


def test_link_status_rows_name_held_legs_and_the_bench():
    """Mutation: a Hand or BB axis in the held list, or the bench bit read off BB."""
    diag = {0: types.SimpleNamespace(flags=DIAG_FLAG_LEG_HELD | DIAG_FLAG_RIG_BENCH),
            4: types.SimpleNamespace(flags=DIAG_FLAG_LEG_HELD),
            _HAND: types.SimpleNamespace(flags=DIAG_FLAG_LEG_HELD)}
    assert tbn._legs_held_str(diag) == '0,4'
    assert tbn._legs_held_str({}) == 'none'


# ── _run_activate names the FW 53 outcome ──

def _scripted_activate(node, monkeypatch, frames):
    """Run _run_activate([0]) against a scripted status sequence (one per poll)."""
    seq = list(frames)

    def status(axes):
        if 0 not in axes:
            return {}
        return {0: seq.pop(0) if len(seq) > 1 else seq[0]}
    monkeypatch.setattr(node, '_activate_axis_status', status)
    return node._run_activate([0], poll_dt=0.001)


def test_run_activate_names_a_returned_rise(monkeypatch):
    """Mutation: the RETURNED summary removed -> the generic 'activate FAILED'."""
    teensy, client, node = _build_paired_node()
    try:
        teensy.on_rpc(int(RpcMethod.ACTIVATE), lambda r, a: (int(RpcStatus.OK), b""))
        cl = lambda pos: ActivateAxisStatus(axis_state=CLOSED_LOOP, pos_rev=pos, vel_rps=0.0)
        ok, msg = _scripted_activate(node, monkeypatch, [
            cl(0.0), cl(0.8), cl(1.4), cl(1.1), cl(0.03),
            ActivateAxisStatus(axis_state=IDLE, pos_rev=0.0, vel_rps=0.0)])
        assert ok is False
        assert msg.startswith('activate aborted, Platform returned to Stow')
    finally:
        _teardown(teensy, client, node)


def test_run_activate_names_a_held_rise(monkeypatch):
    teensy, client, node = _build_paired_node()
    try:
        teensy.on_rpc(int(RpcMethod.ACTIVATE), lambda r, a: (int(RpcStatus.OK), b""))
        ok, msg = _scripted_activate(node, monkeypatch, [
            ActivateAxisStatus(axis_state=CLOSED_LOOP, pos_rev=0.8, vel_rps=2.0),
            ActivateAxisStatus(axis_state=CLOSED_LOOP, pos_rev=0.86, vel_rps=0.0,
                               held=True)])
        assert ok is False
        assert msg.startswith('activate aborted, Legs held Closed-loop above Stow')
    finally:
        _teardown(teensy, client, node)


# ── _run_home: the expected FW 53 refusal is INFO, not WARN ──

def test_home_refusal_logs_info_waiting_for_the_previous_leg(monkeypatch):
    """Mutation: the INFO line removed or raised to warning."""
    teensy, client, node = _build_paired_node()
    try:
        calls = {'n': 0}

        def on_home(req_id, args):
            calls['n'] += 1
            return (int(RpcStatus.ERR_REJECTED if calls['n'] <= 3 else RpcStatus.OK), b"")
        teensy.on_rpc(int(RpcMethod.HOME), on_home)
        teensy.on_rpc(int(RpcMethod.SET_AXIS_STATE),       # the abort's IDLE
                      lambda r, a: (int(RpcStatus.OK), b""))
        logged = {'info': [], 'warning': []}
        logger = node.get_logger()
        monkeypatch.setattr(logger, 'info', lambda m, **k: logged['info'].append(m))
        monkeypatch.setattr(logger, 'warning', lambda m, **k: logged['warning'].append(m))
        monkeypatch.setattr(logger, 'warn', lambda m, **k: logged['warning'].append(m))
        monkeypatch.setattr(node, '_home_abort_requested', lambda: False)
        t = threading.Thread(target=node._run_home, args=([1],), kwargs={'poll_dt': 0.01})
        t.daemon = True
        t.start()
        assert _poll(lambda: calls['n'] >= 4)
        node._home_abort_requested = lambda: True            # end the run
        t.join(timeout=5.0)
        waits = [m for m in logged['info'] if 'waiting for the previous Leg' in m]
        assert len(waits) == 1                                 # once per axis
        assert not any('HOME' in m for m in logged['warning'])
    finally:
        _teardown(teensy, client, node)


def test_home_retry_budget_keeps_3x_margin_over_the_fw53_refusal():
    """fw53-u4: HOME(Leg k+1) is refused up to ~310 ms (two lost heartbeats). The
    retry budget is 20 x poll_dt of sleeps after the first refusal."""
    import inspect
    src = inspect.getsource(tbn.TeensyBridgeNode._run_home)
    assert '_MAX_REJECT_RETRIES = 20' in src
    poll_dt = inspect.signature(tbn.TeensyBridgeNode._run_home).parameters['poll_dt'].default
    assert 20 * poll_dt >= 3 * 0.310
    assert hw.HOMING_MOTOR_TIMEOUT_S > 0
