"""Platform IMU upload gate and Attempt span (IDEA-20 P3c): the pure functions, the orchestrator and
skill_node entry points (mocked ROS), and the bridge node's fire-and-forget trigger.

Command table (verified against orchestrator_node._COMMANDS_ACCEPTED, 2026-11-19): motion-starting =
standby, trajectory, spacemouse, gui, level, home and the Juggle request; pacifying = deactivate (and the
Soft E-stop paths, handled in the bridge node); everything else passes."""
from __future__ import annotations

import types
from unittest.mock import MagicMock

import pytest

from jugglebot import platform_imu_gate as g
from tests.ros.conftest import MockString
from jugglebot_interfaces.srv import SetString

BUSY = {'busy': '1', 'upload_after_attempt': '1', 'phase': 'samples', 'eta_s': '12.4'}
MOTION = ['standby', 'trajectory', 'spacemouse', 'gui', 'level', 'home', 'juggle']
OTHER = ['activate', 'clear_errors', 'bogus']


def _gate(values=None, at=100.0):
    gate = g.ImuGate()
    if values is not None:
        gate.observe(values, at)
    return gate


# ---- the pure decision table ------------------------------------------------------------------------------

@pytest.mark.parametrize('cmd', MOTION)
def test_motion_starting_refused_while_busy_with_reason_and_eta(cmd):
    d = _gate(BUSY).check(cmd, 100.5)
    assert d.allow is False and d.abort is False
    assert 'Platform IMU upload is running' in d.reason and 'about 12 s remain' in d.reason and cmd in d.reason
    assert 'deactivate' in d.reason


@pytest.mark.parametrize('cmd', MOTION + OTHER + ['deactivate'])
def test_everything_passes_when_idle(cmd):
    d = _gate(dict(BUSY, busy='0')).check(cmd, 100.5)
    assert d.allow is True and d.abort is False


@pytest.mark.parametrize('cmd', MOTION)
def test_toggle_off_refuses_nothing(cmd):
    assert _gate(dict(BUSY, upload_after_attempt='0')).check(cmd, 100.5).allow is True


@pytest.mark.parametrize('cmd', OTHER)
def test_commands_off_the_motion_list_pass_while_busy(cmd):
    assert _gate(BUSY).check(cmd, 100.5) == g.GateDecision(True, '', False)


@pytest.mark.parametrize('cmd', ['deactivate', 'soft_estop', 'estop'])
def test_pacifying_always_passes_and_aborts_a_running_upload(cmd):
    assert _gate(BUSY).check(cmd, 100.5) == g.GateDecision(True, '', True)
    assert _gate(None).check(cmd, 100.5).allow is True          # no status at all
    assert _gate(BUSY).check(cmd, 100.0 + 999).allow is True    # stale, ceiling passed


@pytest.mark.parametrize('cmd', MOTION)
def test_a_missing_status_fails_open(cmd):
    assert _gate(None).check(cmd, 100.0).allow is True


@pytest.mark.parametrize('cmd', MOTION)
def test_a_stale_status_fails_open(cmd):
    gate = _gate(BUSY, 100.0)
    assert gate.check(cmd, 101.9).allow is False
    assert gate.check(cmd, 102.1).allow is True                 # older than 2 s: a dead bridge cannot lock us out


def test_the_ceiling_treats_a_hung_upload_as_idle_and_asks_for_one_abort():
    gate = _gate(BUSY, 100.0)
    t = 100.0
    while t < 100.0 + 181:                                       # status keeps arriving, busy the whole time
        t += 0.5
        gate.observe(BUSY, t)
    assert gate.view.busy_since == 100.0
    d = gate.check('standby', t)
    assert d.allow is True and d.abort is True
    assert gate.check('standby', t + 1).abort is False           # rate-limited, not an abort per command
    gate.observe(BUSY, t + 11)                                   # status still arriving
    assert gate.check('standby', t + 11).abort is True


def test_busy_since_restarts_after_an_idle_status():
    gate = _gate(BUSY, 100.0)
    gate.observe(dict(BUSY, busy='0'), 101.0)
    gate.observe(BUSY, 102.0)
    assert gate.view.busy_since == 102.0


def test_eta_falls_back_to_a_few_seconds():
    d = _gate(dict(BUSY, eta_s='')).check('home', 100.5)
    assert 'a few seconds remain' in d.reason


def test_the_gate_never_raises_on_a_malformed_status():
    gate = _gate({'busy': '1'})
    assert gate.check('home', 100.1).allow is False
    gate._view = 'garbage'
    assert gate.check('home', 100.1).allow is True               # a gate bug fails open


# ---- the span ---------------------------------------------------------------------------------------------

def test_short_attempt_span_has_both_margins():
    t = g.AttemptSpanTracker()
    t.started(1000.0)
    s = t.ended(1006.0)
    assert (s.t_start, s.t_end, s.reason) == (998.0, 1009.0, 'attempt_end')
    assert t.open is False


def test_a_long_attempt_uploads_the_last_ring():
    t = g.AttemptSpanTracker()
    t.started(1000.0)
    s = t.ended(1100.0)
    assert s.t_end == 1103.0 and s.t_end - s.t_start == g.RING_S


def test_exactly_at_the_cap_is_not_trimmed():
    t0, t1 = g.compute_span(100.0, 100.0 + 27.0)                 # 2 + 27 + 3 = 32
    assert t1 - t0 == 32.0 and t0 == 98.0


def test_fault_mid_attempt_closes_it_once():
    t = g.AttemptSpanTracker()
    t.started(1000.0)
    s = t.fault(1004.0)
    assert (s.t_start, s.t_end, s.reason) == (998.0, 1007.0, 'fault')
    assert t.fault(1005.0) is None                               # no open Attempt
    assert t.ended(1005.5) is None                               # the same Attempt's juggle_end: no second upload
    t.started(2000.0)
    assert t.ended(2002.0).reason == 'attempt_end'               # the next Attempt is unaffected


def test_a_fault_with_no_attempt_returns_none_and_an_end_without_start_uses_the_ring():
    t = g.AttemptSpanTracker()
    assert t.fault(5.0) is None
    s = t.ended(100.0)
    assert s.t_end - s.t_start == g.RING_S


# ---- orchestrator entry points (mocked ROS) ----------------------------------------------------------------

@pytest.fixture
def orch():
    from jugglebot.orchestrator_node import OrchestratorNode
    node = OrchestratorNode()
    node.aborts = []
    node._fire_forget_service_call = lambda client, req, label: node.aborts.append(label)
    return node


def _status(orch, values=BUSY):
    orch._on_platform_imu_status(types.SimpleNamespace(
        values=[types.SimpleNamespace(key=k, value=v) for k, v in values.items()]))


@pytest.mark.parametrize('cmd', ['standby', 'trajectory', 'spacemouse', 'gui', 'level', 'home'])
def test_orchestrator_command_topic_refuses_motion_starting_and_does_not_enqueue(orch, cmd):
    _status(orch)
    orch._on_command(MockString(data=cmd))
    assert orch.ctx.consume_command() is None
    assert orch.aborts == []


@pytest.mark.parametrize('cmd', ['activate', 'clear_errors'])
def test_orchestrator_command_topic_passes_non_motion_commands(orch, cmd):
    _status(orch)
    orch._on_command(MockString(data=cmd))
    assert orch.ctx.consume_command() == cmd


def test_deactivate_is_enqueued_and_aborts_the_upload(orch):
    _status(orch)
    orch._on_command(MockString(data='deactivate'))
    assert orch.ctx.consume_command() == 'deactivate'
    assert orch.aborts == ['platform_imu/abort']


def test_deactivate_with_no_upload_sends_no_abort(orch):
    _status(orch, dict(BUSY, busy='0'))
    orch._on_command(MockString(data='deactivate'))
    assert orch.ctx.consume_command() == 'deactivate' and orch.aborts == []


def test_orchestrator_with_no_status_enqueues_everything(orch):
    for cmd in MOTION[:-1]:
        orch._on_command(MockString(data=cmd))
        assert orch.ctx.consume_command() == cmd


def test_stale_status_does_not_refuse_at_the_orchestrator(orch):
    _status(orch)
    orch._imu_gate._view = orch._imu_gate._view._replace(received_at=orch._imu_gate._view.received_at - 5.0)
    orch._on_command(MockString(data='home'))
    assert orch.ctx.consume_command() == 'home'


def _req(data):
    r = SetString.Request()
    r.data = data
    return r


def test_juggle_request_is_refused_with_the_reason_and_no_goal_sent(orch):
    _status(orch)
    sent = []
    orch._juggle_client._server_ready = True
    orch._juggle_client.send_goal_async = lambda goal: sent.append(goal)
    res = orch._svc_juggle_request(_req('hop'), SetString.Response())
    assert res.success is False and 'Platform IMU upload is running' in res.message and 'remain' in res.message
    assert sent == []


def test_juggle_request_passes_when_the_upload_is_done(orch):
    _status(orch, dict(BUSY, busy='0'))
    orch._juggle_client._server_ready = True
    res = orch._svc_juggle_request(_req('hop'), SetString.Response())
    assert res.success is True


def test_juggle_stop_is_never_gated(orch):
    _status(orch)
    res = orch._svc_juggle_stop(types.SimpleNamespace(), types.SimpleNamespace())
    assert 'running' in res.message                              # the normal 'no goal' answer, not a refusal
    assert 'IMU' not in res.message


def test_a_dead_abort_service_cannot_break_the_command(orch):
    _status(orch)
    del orch._fire_forget_service_call
    orch._imu_abort_client = MagicMock()
    orch._imu_abort_client.service_is_ready.return_value = False
    orch._on_command(MockString(data='deactivate'))
    assert orch.ctx.consume_command() == 'deactivate'


# ---- skill_node: the Attempt start from any source --------------------------------------------------------

def test_skill_node_rejects_a_goal_while_an_upload_runs_and_accepts_otherwise(tmp_path):
    from unittest.mock import patch
    from jugglebot import skill_node as sn
    from tests.ros.test_skill_node import _node_with_client, _columns_goal
    node, _client = _node_with_client()
    node._on_platform_imu_status(types.SimpleNamespace(
        values=[types.SimpleNamespace(key=k, value=v) for k, v in BUSY.items()]))
    assert node._juggle_goal(_columns_goal(separation_mm=100.0)) == sn.GoalResponse.REJECT
    assert node._executor is None
    node._on_platform_imu_status(types.SimpleNamespace(
        values=[types.SimpleNamespace(key=k, value=v) for k, v in dict(BUSY, busy='0').items()]))
    with patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        assert node._juggle_goal(_columns_goal(separation_mm=100.0)) == sn.GoalResponse.ACCEPT


def test_span_cap_follows_the_ring_length_at_2_khz():
    """Platform FW 13 at 2 kHz: the ring holds 16 s, so the span is capped at the last 16 s, not 32."""
    assert g.compute_span(100.0, 127.0, ring_s=16.0) == (130.0 - 16.0, 130.0)
    ring = {"s": 16.0}
    t = g.AttemptSpanTracker(ring_s=lambda: ring["s"])
    t.started(100.0)
    assert t.ended(140.0) == g.Span(143.0 - 16.0, 143.0, "attempt_end")
    assert t.ended(200.0).t_start == 203.0 - 16.0                      # an end with no start: the last ring
    ring["s"] = 32.0
    t.started(100.0)
    assert t.fault(140.0) == g.Span(143.0 - 32.0, 143.0, "fault")
    bad = g.AttemptSpanTracker(ring_s=lambda: 1 / 0)                     # a failing lookup falls back to RING_S
    bad.started(100.0)
    assert bad.ended(140.0).t_start == 143.0 - g.RING_S
