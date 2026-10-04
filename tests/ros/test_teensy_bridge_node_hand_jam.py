"""teensy_bridge_node: the hand-jam detector feed and recovery executor.

The ORDER of the recovery is pinned in tests/motion/test_hand_jam.py against
the pure step machine; these tests pin the bridge's half: the detector is fed
from the RX telemetry path and the sequence runs on its own worker thread,
the intents reach the existing primitives in the machine's order with
fault_state=RECOVERING held throughout, HAND_MOVE_TO's ERR_UNKNOWN_METHOD is a
relief-only UNRECOVERED, /recover resumes a held jam at the lower instead of
running the park, and the arm/park/clear entry points respect the hold.

Mocked ROS (tests/ros/conftest.py) + the FakeTeensy loopback harness; the
hand is a fake plant writing the bridge's telemetry caches, as the RX thread
would.
"""
from __future__ import annotations

import csv
import pathlib
import threading
import time
import types

from teensy_link import FaultState
from teensy_link import protocol as p
from teensy_link import rpc_args
from teensy_link.rpc import HandMoveToUnavailable, RpcError, RpcTimeout

from jugglebot.motion import hand_jam as hj

from std_srvs.srv import SetBool, Trigger

from tests.ros._bridge_harness import _build_paired_node, _link_kv, _teardown
from tests.ros.test_teensy_bridge_node_setpoint import _bring_link_and_telem_up

FIXTURE = (pathlib.Path(__file__).resolve().parents[1] / 'motion' / 'fixtures'
           / 'hand_jam_20261002.csv')
_HAND = 6


def _set_hand(node, pos, vel=0.0, iq=-2.0, cmd=None, state=8, err=0, age=0.0):
    tm = types.SimpleNamespace(pos_rev=[0.0] * 6 + [pos], vel_rps=[0.0] * 6 + [vel])
    with node._lock:
        node._latest_telemetry = tm
        node._latest_diag[_HAND] = types.SimpleNamespace(
            axis_id=_HAND, iq_measured=iq, axis_state=state, active_errors=err,
            disarm_reason=0)
        node._latest_diag_mono[_HAND] = time.monotonic() - age
        if cmd is not None:
            node._last_hand_cmd = {'pos': cmd[0], 'vel': cmd[1], 'tor': 0.0}
    node._hand_jam_telem_mono = time.monotonic()
    return tm


def _hb(node, *, fault=FaultState.NONE, armed=False):
    node._latest_heartbeat = types.SimpleNamespace(
        fault_state=int(fault), flags=0x8 if armed else 0)


def _blob(outcome, pos, target):
    return rpc_args.encode_hand_move_to_result(outcome, pos, 0.0, target, 100)


class _FakeCall:
    """An in-flight HAND_MOVE_TO as ``RpcClient.hand_move_to_nowait`` returns
    it (the RpcCall surface: done / result / release)."""

    def __init__(self, target):
        self.target, self.final, self.released = target, None, False

    def done(self):
        return self.final is not None

    def result(self):
        self.released = True
        if self.final is None:
            raise RpcTimeout(0x61, 0, 0.0)
        if isinstance(self.final, Exception):
            raise self.final
        return self.final

    def release(self):
        self.released = True


class _FakeHand:
    """Axis 6 under HAND_MOVE_TO (FW 26 semantics): a 100 Hz thread writes the
    caches and TRAP_TRAJs toward the latest target, stopping on a ball that
    falls once the hand is ``clear`` above it. A newer move RETARGETS: the
    running call ends OK/SUPERSEDED; the latest ends ARRIVED on arrival."""

    def __init__(self, node, pos, *, ball=None, clear=0.5):
        self.node, self.pos, self.vel = node, pos, 0.0
        self.ball, self.clear = ball, clear
        self.call, self.calls = None, []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def nowait(self, target, vel_rps=2.5):
        with self._lock:
            if self.call is not None and self.call.final is None:
                self.call.final = _blob(rpc_args.HAND_MOVE_SUPERSEDED, self.pos, self.call.target)
            self.call, self.v = _FakeCall(target), vel_rps
            self.calls.append(self.call)
            return self.call

    def _loop(self):
        while not self._stop.is_set():
            with self._lock:
                c = self.call
                if c is not None and c.final is None:
                    d = c.target - self.pos
                    step = max(-self.v * 0.01, min(self.v * 0.01, d))
                    if self.ball is not None and step < 0 and self.pos + step < self.ball:
                        step = min(0.0, self.ball - self.pos)
                    self.pos += step
                    self.vel = step / 0.01
                    if self.ball is not None and self.pos >= self.ball + self.clear:
                        self.ball = None
                    if abs(c.target - self.pos) < 1e-6:
                        c.final = _blob(rpc_args.HAND_MOVE_ARRIVED, self.pos, c.target)
                        self.vel = 0.0
            _set_hand(self.node, self.pos, self.vel)
            time.sleep(0.01)

    def stop(self):
        self._stop.set()
        self._th.join(timeout=1.0)


def _instrument(node, hand=None):
    """Replace the primitives with recorders. Each entry records whether the
    RECOVERING hold was up when the primitive ran."""
    calls = []

    def setlim(axis, vel, curr, **kw):
        calls.append(('SET_VEL_CURR', float(curr), node._hand_jam_hold))
        return True, 'OK', b''

    def recover(req, res, *, park_hand=True):
        calls.append(('RECOVER', park_hand, node._hand_jam_hold))
        _hb(node, fault=FaultState.NONE, armed=bool(node._mpc_active))
        res.success, res.message = True, 'cleared'
        return res

    def stop_sp():
        calls.append(('STOP_SP', None, node._hand_jam_hold))
        node._mpc_active = False
        _hb(node, fault=FaultState(node._latest_heartbeat.fault_state), armed=False)

    def wait_disarmed(timeout_s=1.0):
        calls.append(('WAIT_DISARMED', None, node._hand_jam_hold))
        return True

    def move(target, vel_rps=2.5):
        calls.append(('MOVE', round(float(target), 4), node._hand_jam_hold))
        return hand.nowait(target, vel_rps)

    node.teensy_set_vel_curr_limits = setlim
    node._svc_recover_orig = node._svc_recover
    node._svc_recover = recover
    node._stop_setpoint_output = stop_sp
    node._wait_wire_disarmed = wait_disarmed
    if hand is not None:
        node._rpc.hand_move_to_nowait = move     # the client call site (FW 26)
    return calls


def _fast_cfg(node, **kw):
    node._hand_jam_cfg = hj.JamConfig(dwell_s=0.2, **kw)
    node._hand_jam_detector = hj.JamDetector(node._hand_jam_cfg)


def _wait_done(node, timeout=15.0):
    t = node._hand_jam_thread
    assert t is not None
    t.join(timeout=timeout)
    assert not t.is_alive(), 'jam worker did not finish'


def _fixture_rows():
    with FIXTURE.open() as f:
        rows = [r for r in csv.reader(f) if r and not r[0].startswith('#')]
    return [tuple(map(float, r)) for r in rows[1:]]


def test_fixture_fires_on_the_rx_feed_and_the_worker_relieves_first():
    """Replay the 2026-10-02 rows through `_hand_jam_feed` at their real
    cadence. It must fire >= 80 ms before the latch, from the feed's own call
    (no executor), hand the sequence to a 'hand_jam' worker thread without
    blocking the feed, and the worker's first RPC must be the 10 A relief.
    Against this checkout's firmware the raise is ERR_UNKNOWN_METHOD: the
    outcome is UNRECOVERED with that reason, the relief stays (no restore),
    and fault_state reads RECOVERING."""
    teensy, client, node = _build_paired_node()
    try:
        _fast_cfg(node, max_gap_s=1.0)          # pacing jitter is not a gap here
        calls = _instrument(node)
        node._mpc_active = True
        _hb(node, fault=FaultState.NONE, armed=True)

        def unknown(target, vel_rps=2.5):
            # An FW <= 25 board: the deferred reply is ERR_UNKNOWN_METHOD
            # (hand_move_to_nowait leaves the mapping to the caller).
            calls.append(('MOVE', round(float(target), 4), node._hand_jam_hold))
            c = _FakeCall(target)
            c.final = RpcError(0x61, int(p.RpcStatus.ERR_UNKNOWN_METHOD))
            return c
        node._rpc.hand_move_to_nowait = unknown  # the client call site

        rows = _fixture_rows()
        t0, w0 = rows[0][0], time.monotonic()
        fired_at, feed_s = None, None
        for (t, pc, vf, pm, vm, iq, st, er, age) in rows:
            while time.monotonic() < w0 + (t - t0):
                time.sleep(0.001)
            tm = _set_hand(node, pm, vm, iq, cmd=(pc, vf), state=int(st), err=int(er),
                           age=age)
            was = node._hand_jam_busy or node._hand_jam_hold
            c0 = time.monotonic()
            node._hand_jam_feed(tm)
            if not was and node._hand_jam_hold and fired_at is None:
                fired_at, feed_s = t, time.monotonic() - c0
        assert fired_at is not None and fired_at <= -0.080, fired_at
        assert feed_s < 0.05, f'the feed blocked {feed_s:.3f} s at the fire'
        assert node._hand_jam_thread.name == 'hand_jam'
        assert node._hand_jam_thread is not threading.current_thread()
        while node._hand_jam_busy:              # keep the telemetry fresh
            _set_hand(node, rows[-1][3])
            time.sleep(0.01)
        _wait_done(node)
        assert calls[0] == ('SET_VEL_CURR', 10.0, True)
        assert [c for c in calls if c[0] == 'SET_VEL_CURR'] == [('SET_VEL_CURR', 10.0, True)]
        assert all(c[2] for c in calls), calls
        assert node._hand_jam_hold is True and node._hand_jam_busy is False
        assert node._hand_jam_status == 'HAND_JAM_UNRECOVERED: firmware has no HAND_MOVE_TO'
        assert node._published_fault_state(node._latest_heartbeat) == 'RECOVERING'
        assert node._hand_curr_limit == 50.0    # the cached shipped limit is untouched
    finally:
        node._hand_jam_hold = False
        _teardown(teensy, client, node)


def test_recovered_sequence_order_restores_last_and_releases_recovering():
    teensy, client, node = _build_paired_node()
    hand = _FakeHand(node, 2.85, ball=2.85, clear=0.5)
    try:
        _fast_cfg(node)
        calls = _instrument(node, hand)
        node._mpc_active = True
        _hb(node, fault=FaultState.MAX_DEVIATION, armed=True)
        time.sleep(0.05)
        sample = node._hand_jam_sample()
        assert node._hand_jam_start(hj.Verdict.HAND_JAM, sample) is not None
        _wait_done(node)
        assert [c[:2] for c in calls] == [
            ('SET_VEL_CURR', 10.0), ('RECOVER', False), ('STOP_SP', None),
            ('WAIT_DISARMED', None), ('MOVE', 3.85), ('MOVE', 0.0),
            ('SET_VEL_CURR', 50.0)]
        assert all(c[2] for c in calls), 'RECOVERING dropped mid-sequence'
        assert node._hand_jam_hold is False
        assert node._hand_jam_status.startswith('HAND_JAM_RECOVERED')
        assert node._hand_jam_detector.fired is hj.Verdict.NONE   # re-armed
    finally:
        hand.stop()
        node._hand_jam_hold = False
        _teardown(teensy, client, node)


def test_an_exception_in_a_primitive_reports_unrecovered_and_keeps_the_hold():
    teensy, client, node = _build_paired_node()
    hand = _FakeHand(node, 2.85)
    try:
        _fast_cfg(node)
        calls = _instrument(node, hand)
        _hb(node)

        def boom(timeout_s=1.0):
            raise RuntimeError('wire gone')
        node._wait_wire_disarmed = boom
        time.sleep(0.05)
        node._hand_jam_start(hj.Verdict.HAND_JAM, node._hand_jam_sample())
        _wait_done(node)
        assert ('SET_VEL_CURR', 50.0) not in [c[:2] for c in calls]
        assert not any(c[0] == 'MOVE' for c in calls)
        assert node._hand_jam_hold is True and node._hand_jam_busy is False
        assert 'exception' in node._hand_jam_status
    finally:
        hand.stop()
        node._hand_jam_hold = False
        _teardown(teensy, client, node)


def test_recover_after_unrecovered_resumes_at_the_lower_not_the_park():
    teensy, client, node = _build_paired_node()
    hand = _FakeHand(node, 5.35)
    try:
        _fast_cfg(node)
        calls = _instrument(node, hand)
        node._svc_recover = node._svc_recover_orig      # the REAL entry point
        _hb(node)

        def no_park(*a, **k):
            raise AssertionError('the park must not run under a held jam')
        node._park_hand_disarmed = no_park
        held = hj.JamRecovery(node._hand_jam_cfg, kind=hj.Verdict.HAND_JAM,
                              stall_pos=2.85, restore_curr_a=50.0)
        held.abort('lower stalled twice')
        node._hand_jam_rec, node._hand_jam_hold = held, True
        time.sleep(0.05)
        res = node._svc_recover(Trigger.Request(), Trigger.Response())
        assert res.success is True, res.message
        assert [c[:2] for c in calls] == [
            ('SET_VEL_CURR', 10.0), ('WAIT_DISARMED', None), ('MOVE', 0.0),
            ('SET_VEL_CURR', 50.0)]
        assert node._hand_jam_hold is False
    finally:
        hand.stop()
        node._hand_jam_hold = False
        _teardown(teensy, client, node)


def test_held_jam_refuses_park_and_arm_and_routes_clear_errors():
    teensy, client, node = _build_paired_node()
    try:
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        node._hand_jam_hold = True
        node._hand_jam_status = 'HAND_JAM_UNRECOVERED: x'
        res = node._svc_park_hand(Trigger.Request(), Trigger.Response())
        assert res.success is False and 'hand-jam' in res.message
        req = SetBool.Request()
        req.data = True
        resp = node._svc_set_setpoint_output(req, SetBool.Response())
        assert resp.success is False and 'hand-jam' in resp.message
        routed = []
        node._hand_jam_resume = lambda r: (routed.append(1), r)[1]
        node._svc_clear_errors(Trigger.Request(), Trigger.Response())
        assert routed == [1]
        assert _link_kv(node)['hand_jam'] == 'HAND_JAM_UNRECOVERED: x'
        assert node._published_fault_state(node._latest_heartbeat) == 'RECOVERING'
    finally:
        node._hand_jam_hold = False
        _teardown(teensy, client, node)


def test_converge_clear_without_park_skips_the_hand_park():
    """`_svc_recover(park_hand=False)` (the jam's step 2) clears but never
    parks: here via the reseed-unavailable escape hatch."""
    teensy, client, node = _build_paired_node()
    try:
        node._reseed_client = types.SimpleNamespace(service_is_ready=lambda: False)
        node.teensy_clear_errors = lambda *a, **k: (True, 'OK', b'')

        def no_park(*a, **k):
            raise AssertionError('parked')
        node._park_hand_disarmed = no_park
        res = node._svc_recover(Trigger.Request(), Trigger.Response(), park_hand=False)
        assert res.success is True and 'left to the caller' in res.message
    finally:
        _teardown(teensy, client, node)


def test_teensy_hand_move_to_maps_the_client_outcomes():
    teensy, client, node = _build_paired_node()
    try:
        seen = []

        def arrived(t, vel_rps, timeout_s):
            seen.append((t, vel_rps, timeout_s))
            return _blob(rpc_args.HAND_MOVE_ARRIVED, t, t)
        node._rpc.hand_move_to = arrived
        ok, status, _ = node.teensy_hand_move_to(3.85, 2.5)
        assert (ok, status) == (True, 'ARRIVED') and seen == [(3.85, 2.5, 12.0)]

        def unavailable(*a, **k):
            raise HandMoveToUnavailable()
        node._rpc.hand_move_to = unavailable
        assert node.teensy_hand_move_to(3.85)[:2] == (False, 'ERR_UNKNOWN_METHOD')
        node._rpc = types.SimpleNamespace()      # a client predating the method
        assert node.teensy_hand_move_to(3.85)[:2] == (False, hj.STATUS_NO_CLIENT_METHOD)
        assert node.teensy_hand_move_to_nowait(3.85)[:2] == (False, hj.STATUS_NO_CLIENT_METHOD)
    finally:
        _teardown(teensy, client, node)


def _raise(exc):
    def f():
        raise exc
    return f


def test_reply_classifier_reads_the_outcome_and_orders_rpc_timeout_first():
    """OK covers SUPERSEDED too, so only outcome ARRIVED is success; and
    RpcTimeout (a SUBCLASS of RpcError whose status is ERR_TIMEOUT) must be
    caught first: "no reply, move state unknown" is not the firmware's own
    ERR_TIMEOUT ("the move timed out, the hand was commanded IDLE")."""
    teensy, client, node = _build_paired_node()
    try:
        r = node._hand_move_reply
        assert r(lambda: _blob(rpc_args.HAND_MOVE_ARRIVED, 3.85, 3.85))[:2] == (True, 'ARRIVED')
        assert r(lambda: _blob(rpc_args.HAND_MOVE_SUPERSEDED, 1.2, 0.0))[:2] == (
            False, 'SUPERSEDED')
        assert issubclass(RpcTimeout, RpcError)
        assert r(_raise(RpcTimeout(0x61, 0, 12.0)))[:2] == (False, 'RPC_TIMEOUT')
        assert r(_raise(RpcError(0x61, int(p.RpcStatus.ERR_TIMEOUT))))[:2] == (
            False, 'ERR_TIMEOUT')
        assert r(_raise(RpcError(0x61, int(p.RpcStatus.ERR_REJECTED))))[:2] == (
            False, 'ERR_REJECTED')
        assert r(_raise(RpcError(0x61, int(p.RpcStatus.ERR_UNKNOWN_METHOD))))[:2] == (
            False, 'ERR_UNKNOWN_METHOD')
        assert r(_raise(HandMoveToUnavailable()))[:2] == (False, 'ERR_UNKNOWN_METHOD')
    finally:
        _teardown(teensy, client, node)


def test_a_retarget_supersedes_and_the_old_reply_is_never_read():
    from jugglebot.teensy_bridge_node import _HandMoveSlot
    teensy, client, node = _build_paired_node()
    try:
        calls = []
        node._rpc.hand_move_to_nowait = lambda t, vel_rps=2.5: calls.append(_FakeCall(t)) or calls[-1]
        slot = _HandMoveSlot(node)
        slot.start(0.0, 2.5)
        slot.start(5.35, 2.5)                  # the raise-again, mid-lower
        assert calls[0].released is True
        calls[0].final = _blob(rpc_args.HAND_MOVE_SUPERSEDED, 3.0, 0.0)
        assert slot.snapshot()[0] is False     # still the NEW move, in flight
        calls[1].final = _blob(rpc_args.HAND_MOVE_ARRIVED, 5.35, 5.35)
        done, ok, status, _ = slot.snapshot()
        assert (done, ok, status) == (True, True, 'ARRIVED')
    finally:
        _teardown(teensy, client, node)


def test_a_wedged_ball_retargets_the_lower_up_twice_and_never_restores():
    teensy, client, node = _build_paired_node()
    hand = _FakeHand(node, 2.85, ball=2.85, clear=float('inf'))
    try:
        _fast_cfg(node)
        calls = _instrument(node, hand)
        _hb(node)
        time.sleep(0.05)
        node._hand_jam_start(hj.Verdict.HAND_JAM, node._hand_jam_sample())
        _wait_done(node, timeout=20.0)
        moves = [c[1] for c in calls if c[0] == 'MOVE']
        assert moves[:3] == [3.85, 0.0, 5.35] and moves[3] == 0.0
        assert moves[4] == 5.35 and len(moves) == 5          # final raise, stays up
        assert ('SET_VEL_CURR', 50.0) not in [c[:2] for c in calls]
        # each stalled lower was RETARGETED (superseded), not left to time out
        outcomes = [rpc_args.decode_hand_move_to_result(c.final).outcome for c in hand.calls]
        assert outcomes[1] == outcomes[3] == rpc_args.HAND_MOVE_SUPERSEDED
        assert hand.calls[1].released and hand.calls[3].released
        assert hand.pos > 5.3 and node._hand_jam_hold is True
        assert node._hand_jam_status.startswith('HAND_JAM_UNRECOVERED')
    finally:
        hand.stop()
        node._hand_jam_hold = False
        _teardown(teensy, client, node)


def test_dry_run_reports_predicates_and_plan_and_changes_nothing():
    teensy, client, node = _build_paired_node()
    try:
        _set_hand(node, 0.30, cmd=(0.307, 0.0))
        res = node._svc_hand_jam_dry_run(Trigger.Request(), Trigger.Response())
        assert res.success is True
        assert 'P1=f' in res.message and 'stall=False' in res.message
        assert 'relief first' in res.message and 'restore LAST' in res.message
        assert node._hand_jam_hold is False and node._hand_jam_thread is None
        assert node._hand_jam_detector.last_sample is None      # not stepped
    finally:
        _teardown(teensy, client, node)


def test_invalid_parameter_is_an_error_and_falls_back_to_the_defaults():
    teensy, client, node = _build_paired_node()
    try:
        node._params['hand_jam.band_rev'] = [0.1, 3.6]
        errors = []
        node.get_logger().error = lambda m, **kw: errors.append(m)
        cfg = node._hand_jam_load_config()
        assert cfg == hj.JamConfig()
        msg = errors[-1]
        assert 'hand_jam parameter REJECTED' in msg and 'band_rev' in msg
        node._params['hand_jam.band_rev'] = [1.2, 3.4]
        assert node._hand_jam_load_config().band_rev == (1.2, 3.4)
        assert _link_kv(node)['hand_jam'] == 'IDLE'
    finally:
        _teardown(teensy, client, node)
