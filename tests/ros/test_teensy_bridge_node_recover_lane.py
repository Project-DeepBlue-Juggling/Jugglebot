"""/recover_lane — automatic recovery from a LATCHED streamed lane (2026-10-13).

R5 sitting 7 (``logbook/2026-10-13-skill-stack-r5-sitting-7-columns-pb.md``): a
45-57 ms Jetson stall latched the leg scheduled-playback group (HeartbeatT2J flags
bit 15) and only an operator DEACTIVATE → ACTIVATE cleared it. ``/recover_lane``
runs converge (the shared ``/recover`` core, still ARMED) → rest → disarm → dwell →
re-arm → confirm, with NO CLEAR_ERRORS and NO hand park.

The Teensy is modelled by ``_FakeWire``: a 100 Hz thread that writes the node's
heartbeat + telemetry caches directly, mirrors ``node._mpc_active`` into the T2J
MPC_ACTIVE flag, counts refused frames while latched + armed, and clears the latch
on the arm EDGE (firmware premise F2, ``leg_interp.cpp:891-902``) unless told not
to. The reseed client is the ``/recover`` tests' fake; the arm preconditions are
replaced by a recorder (they have their own tests in
``test_teensy_bridge_node_setpoint.py``).

BLIND to (mocked-ROS tests cannot see choreography): that the real arm edge clears
bit 15, that the ODrives hold still across the disarm, the hand's C2 stop, the real
live_deviation trajectory, and real heartbeat / fault-tick timing (the 0.2 s dwell).
Those are the dress rehearsal's (design § 7).
"""

from __future__ import annotations

import threading
import time
import types

import pytest

from teensy_link import FaultState
from teensy_link import protocol as p

from std_srvs.srv import SetBool, Trigger

from tests.ros._bridge_harness import _build_paired_node, _link_kv, _teardown
from tests.ros.test_teensy_bridge_node_recover import _FakeReseedClient
from tests.ros.test_teensy_bridge_node_setpoint import _bring_link_and_telem_up

_MPC = int(p.HeartbeatT2JFlags.MPC_ACTIVE)
_LATCH = int(p.HeartbeatT2JFlags.SCHED_HOLD_LATCHED)
_ENC = 0.1


class _FakeWire:
    """The Teensy side of the latch: heartbeat + telemetry at 100 Hz."""

    def __init__(self, node, *, edge_clears=True):
        self.node = node
        self.latched = True
        self.fault = int(FaultState.NONE)
        self.dev = [0.0] * 7
        self.vel = [0.0] * 7
        self.refused = 5
        self.edge_clears = edge_clears
        self._prev_armed = None
        self._tick_lock = threading.Lock()
        self._stop = threading.Event()
        self._tick()
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()

    def _tick(self):
        with self._tick_lock:
            self._tick_locked()

    def _tick_locked(self):
        node = self.node
        armed = bool(node._mpc_active)
        if armed and self._prev_armed is False and self.edge_clears:
            self.latched = False                      # F2: the arm edge clears it
        self._prev_armed = armed
        if self.latched and armed:
            self.refused += 1                         # each refused frame counts
        flags = (_MPC if armed else 0) | (_LATCH if self.latched else 0)
        hb = types.SimpleNamespace(flags=flags, fault_state=self.fault,
                                   live_deviation=tuple(self.dev),
                                   sched_refused=self.refused)
        tm = types.SimpleNamespace(pos_rev=tuple([_ENC] * 6 + [8.68]),
                                   vel_rps=tuple(self.vel))
        with node._lock:
            node._latest_heartbeat = hb
            node._heartbeat_gen += 1
            node._latest_telemetry = tm
            node._telemetry_gen += 1

    def _run(self):
        while not self._stop.wait(0.01):
            self._tick()

    def stop(self):
        self._stop.set()
        self._th.join(timeout=1.0)


class _Rig:
    """A bridge node, armed and latched, with every recovery step recorded."""

    def __init__(self, *, edge_clears=True, arm_result=(True, 'setpoint output ENABLED')):
        self.teensy, self.client, self.node = _build_paired_node()
        node = self.node
        self.calls = []
        self.hooks = {}                               # step name -> callable(before/after)
        node._recover_verify_timeout_s = 1.0
        node._lane_rest_timeout_s = 0.5
        node._lane_confirm_timeout_s = 0.3
        node._lane_disarm_timeout_s = 0.5
        node._sp_pump._prev_pos = [_ENC] * 6          # the descent has converged
        node._reseed_client = _FakeReseedClient(success=True)
        node._mpc_active = True
        self.wire = _FakeWire(node, edge_clears=edge_clears)

        reseed = node._reseed_client.call_async

        def _reseed(req):
            self._step('reseed')
            return reseed(req)
        node._reseed_client.call_async = _reseed

        self._wrap('_verify_streamed_u0_converged', 'converge')
        self._wrap('_lane_wait_rest', 'rest')
        self._wrap('_stop_setpoint_output', 'disarm')
        self._wrap('_lane_sleep', 'dwell')
        self._wrap('_lane_confirm', 'confirm')

        def _arm():
            self._step('arm')
            if arm_result[0]:
                node._set_mpc_active(True)
            return arm_result
        node._arm_setpoint_output_locked = _arm

        def _clear(*a, **k):
            self.calls.append('CLEAR_ERRORS')
            return True, 'cleared', None
        node.teensy_clear_errors = _clear

        def _park(*a, **k):
            self.calls.append('PARK')
            return True, 'parked'
        node._park_hand_disarmed = _park

    def _step(self, name):
        self.calls.append(name)
        hook = self.hooks.get(name)
        if hook is not None:
            hook()

    def _wrap(self, attr, name):
        orig = getattr(self.node, attr)

        def _w(*a, **k):
            self._step(name)
            out = orig(*a, **k)
            after = self.hooks.get(name + ':after')
            if after is not None:
                after()
            return out
        setattr(self.node, attr, _w)

    def recover(self):
        return self.node._svc_recover_lane(Trigger.Request(), Trigger.Response())

    def close(self):
        self.wire.stop()
        _teardown(self.teensy, self.client, self.node)


@pytest.fixture
def rig():
    made = []

    def _make(**kw):
        r = _Rig(**kw)
        made.append(r)
        return r
    yield _make
    for r in made:
        r.close()


def test_happy_path_call_order_no_clear_no_park(rig):
    r = rig()
    res = r.recover()
    assert res.success is True, res.message
    assert r.calls == ['reseed', 'converge', 'rest', 'disarm', 'dwell', 'arm', 'confirm']
    assert 'CLEAR_ERRORS' not in r.calls and 'PARK' not in r.calls
    assert r.node._mpc_active is True
    assert r.node._lane_recovery == 'done'
    assert 'hand held at 8.68 rev' in res.message
    assert 'arm edge' in res.message


def test_short_circuit_when_latch_clears_during_converge(rig):
    """A frame continuous with the held curve was accepted: no edge needed."""
    r = rig()
    r.hooks['reseed'] = lambda: setattr(r.wire, 'latched', False)
    res = r.recover()
    assert res.success is True, res.message
    assert 'disarm' not in r.calls and 'arm' not in r.calls
    assert 'without an arm edge' in res.message
    assert r.node._mpc_active is True
    assert r.node._lane_recovery == 'done'


def _set_fault(r):
    r.wire.fault = int(FaultState.MAX_DEVIATION)
    r.wire._tick()                                    # visible before the next check


# stage, hook point that injects the fault, wire left ARMED?, re-arm attempted?
_FAULT_CASES = [
    ('converge', 'reseed', True, False),
    ('rest', 'rest', True, False),
    ('disarm', 'disarm:after', False, False),
    ('arm', 'dwell:after', False, False),
    ('confirm', 'arm', True, True),
]


@pytest.mark.parametrize('stage,hook,armed,arm_called', _FAULT_CASES,
                         ids=[c[0] for c in _FAULT_CASES])
def test_real_fault_aborts_at_each_stage_without_arming(rig, stage, hook, armed,
                                                        arm_called):
    r = rig()
    r.hooks[hook] = lambda: _set_fault(r)
    res = r.recover()
    assert res.success is False
    assert r.node._lane_recovery == 'failed'
    assert r.node._lane_recovery_stage == stage
    assert 'MAX_DEVIATION' in res.message and 'real fault wins' in res.message
    assert ('arm' in r.calls) is arm_called
    assert r.node._mpc_active is armed
    assert 'CLEAR_ERRORS' not in r.calls
    # A real fault after the arm is the existing guard path's: no second disarm.
    assert r.calls.count('disarm') == (0 if stage in ('converge', 'rest') else 1)


def test_external_disarm_between_disarm_and_arm_cancels(rig):
    r = rig()
    fired = []

    def _operator_disarm():
        if fired:
            return
        fired.append(1)
        req = SetBool.Request()
        req.data = False
        r.node._svc_set_setpoint_output(req, SetBool.Response())
    r.hooks['disarm:after'] = _operator_disarm
    res = r.recover()
    assert res.success is False
    assert 'cancelled by set_setpoint_output' in res.message
    assert 'arm' not in r.calls
    assert r.node._mpc_active is False
    assert r.node._lane_recovery_stage in ('disarm', 'arm')


def test_recover_cancels_a_running_lane_recovery(rig):
    """/recover (here via its hand-jam-resume early exit) raises the cancel."""
    r = rig()
    node = r.node

    def _operator_recover():
        node._hand_jam_hold = True
        node._hand_jam_resume = lambda res: res
        try:
            node._svc_recover(Trigger.Request(), Trigger.Response())
        finally:
            node._hand_jam_hold = False
    r.hooks['dwell:after'] = _operator_recover
    res = r.recover()
    assert res.success is False
    assert 'cancelled by recover' in res.message
    assert 'arm' not in r.calls


def test_deviation_escape_skips_rest_and_disarms(rig):
    r = rig()
    r.node._sp_pump._prev_pos = [1.0] * 6             # still descending
    r.wire.dev = [0.6] + [0.0] * 6                    # leg 0 past half MAX_DEVIATION
    res = r.recover()
    assert res.success is True, res.message
    assert 'rest' not in r.calls
    assert r.calls[-4:] == ['disarm', 'dwell', 'arm', 'confirm']


def test_converge_timeout_fails_with_wire_armed(rig):
    r = rig()
    r.node._recover_verify_timeout_s = 0.1
    r.node._sp_pump._prev_pos = [1.0] * 6             # never converges
    res = r.recover()
    assert res.success is False
    assert r.node._lane_recovery_stage == 'converge'
    assert 'did not converge' in res.message
    assert 'wire left ARMED' in res.message
    assert 'DEACTIVATE → ACTIVATE' in res.message
    assert 'disarm' not in r.calls and r.node._mpc_active is True


def test_reseed_unavailable_fails_armed_never_clears(rig):
    """Unlike /recover there is no direct-clear escape hatch here."""
    r = rig()
    r.node._reseed_client._ready = False
    res = r.recover()
    assert res.success is False
    assert 'unavailable' in res.message
    assert 'CLEAR_ERRORS' not in r.calls and r.node._mpc_active is True


def test_arm_refused_fails_disarmed_and_names_fallback(rig):
    r = rig(arm_result=(False, 'no mpccmd frame on :5557 within 0.5 s — is '
                               'trajectory_node running and streaming?'))
    res = r.recover()
    assert res.success is False
    assert r.node._lane_recovery_stage == 'arm'
    assert 're-arm refused: no mpccmd frame' in res.message
    assert 'wire left DISARMED' in res.message
    assert 'DEACTIVATE → ACTIVATE (IDLE then TRAJECTORY in the GUI)' in res.message
    assert r.node._mpc_active is False


def test_latch_still_set_after_arm_disarms_again_and_fails(rig):
    r = rig(edge_clears=False)
    res = r.recover()
    assert res.success is False
    assert r.node._lane_recovery_stage == 'confirm'
    assert 'bit 15' in res.message
    assert r.calls.count('disarm') == 2
    assert r.node._mpc_active is False


def test_second_call_refused_while_one_runs(rig):
    r = rig()
    r.node._lane_recovery_stage = 'rest'
    assert r.node._lane_recovery_lock.acquire(blocking=False)
    try:
        res = r.recover()
    finally:
        r.node._lane_recovery_lock.release()
    assert res.success is False
    assert 'lane recovery in progress (stage rest)' in res.message
    assert r.calls == []


def test_not_latched_and_not_armed_are_no_ops(rig):
    r = rig()
    r.wire.latched = False
    r.wire._tick()
    res = r.recover()
    assert res.success is True and 'not latched' in res.message
    r.wire.latched = True
    r.node._mpc_active = False
    r.wire._tick()
    res = r.recover()
    assert res.success is True and 'not armed' in res.message
    assert r.calls == [] and r.node._lane_recovery == 'idle'


def test_entry_refused_under_fault_or_operator_ops(rig):
    r = rig()
    r.node._deactivate_in_progress = True
    res = r.recover()
    assert res.success is False and 'deactivate in progress' in res.message
    r.node._deactivate_in_progress = False
    _set_fault(r)
    r.wire._tick()
    res = r.recover()
    assert res.success is False and 'MAX_DEVIATION' in res.message
    assert r.calls == [] and r.node._lane_recovery == 'idle'


def test_link_status_carries_lane_recovery_keys():
    teensy, client, node = _build_paired_node()
    try:
        _bring_link_and_telem_up(teensy, node)
        kv = _link_kv(node)
        assert kv['lane_recovery'] == 'idle'
        assert kv['lane_recovery_stage'] == 'none'
        node._lane_recovery, node._lane_recovery_stage = 'in_progress', 'disarm'
        kv = _link_kv(node)
        assert kv['lane_recovery'] == 'in_progress'
        assert kv['lane_recovery_stage'] == 'disarm'
    finally:
        _teardown(teensy, client, node)


def test_recover_lane_service_registered_in_the_recover_group():
    """Trigger type, Reentrant group shared with /recover (it blocks for seconds)."""
    teensy, client, node = _build_paired_node()
    try:
        svc = node._services['recover_lane']
        assert svc.srv_type is Trigger
        assert svc.callback_group is node._services['recover'].callback_group
        assert svc.callback_group is not None
    finally:
        _teardown(teensy, client, node)


def test_wire_model_sanity():
    """Guard the fake: the latch counter climbs only while latched + armed."""
    node = types.SimpleNamespace(_mpc_active=True, _lock=threading.Lock(),
                                 _latest_heartbeat=None, _heartbeat_gen=0,
                                 _latest_telemetry=None, _telemetry_gen=0)
    w = _FakeWire(node)
    try:
        time.sleep(0.05)
        assert node._latest_heartbeat.flags & _LATCH
    finally:
        w.stop()


@pytest.mark.parametrize('entry,who', [('_run_deactivate', 'deactivate'),
                                       ('_run_activate', 'activate')])
def test_activate_and_deactivate_raise_the_cancel(entry, who):
    """Called with no axes they return at once — after raising the cancel."""
    teensy, client, node = _build_paired_node()
    try:
        assert not node._lane_recovery_cancel.is_set()
        ok, _msg = getattr(node, entry)([])
        assert ok is False
        assert node._lane_recovery_cancel.is_set()
        assert node._lane_recovery_cancel_why == who
    finally:
        _teardown(teensy, client, node)
