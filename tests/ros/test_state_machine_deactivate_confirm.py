"""Confirmed ACTIVE -> IDLE: the orchestrator state machine follows the robot.

2026-10-23 Sitting: the operator requested IDLE, the bridge answered
``DEACTIVATE rejected: DEACTIVATE: ERR_REJECTED`` (the wire-disarm staging race,
see test_teensy_bridge_node_deactivate.py), and the machine entered IDLE anyway
-- it left ACTIVE on the command, before the reply -- while the legs stayed up
in CLOSED_LOOP. Owner rule: a genuine refusal keeps the machine ACTIVE; a
transient one is retried; the reported state always reflects the robot.
Owner hardening (audit, 2026-10-23): ERR_TIMEOUT is observed by the bridge and
never retried here; a reconcile window before the ACTIVE/IDLE verdict; an
ACTIVE-wide legs-out rule; a reply deadline; only mode switches discarded while
a deactivate is in flight; mode switches refused while the wire is disarmed.

Every test drives the real ``StateMachine.tick`` (``build_default_machine``)
on a fake monotonic clock.
"""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

import jugglebot.state_machine as smod
from jugglebot.state_machine import (
    ActiveHandler, Context, RobotState, build_default_machine,
    deactivate_failure_is_retryable,
)

# The exact reply logged at 17:40:21.841 on 2026-10-23.
REJECT_2026_10_23 = 'DEACTIVATE rejected: DEACTIVATE: ERR_REJECTED'
BUS_DOWN = 'DEACTIVATE rejected: DEACTIVATE: ERR_BUS_DOWN'


class _Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture
def clock(monkeypatch):
    c = _Clock()
    monkeypatch.setattr(smod.time, 'monotonic', c)
    return c


def _armed_machine():
    """A real machine ticked into ACTIVE (activated + armed), legs closed-loop,
    wire armed, /link_status seen."""
    sm = build_default_machine(log_fn=lambda m: None)
    sm.force_transition(RobotState.IDLE, Context())
    ctx = Context()
    ctx.legs_closed_loop = True
    ctx.link_status_seen = True
    ctx.wire_armed = True
    sm.tick(ctx)
    ctx.enqueue_command('activate')
    sm.tick(ctx)                      # IDLE -> ACTIVE
    sm.tick(ctx)                      # enter ACTIVE: 'activate' requested
    assert ctx.drain_requests() == ['activate']
    ctx.operation_result = True
    sm.tick(ctx)                      # activated -> arm requested
    assert ctx.drain_requests() == ['arm_setpoints']
    ctx.operation_result = True
    sm.tick(ctx)                      # armed
    assert sm.state == RobotState.ACTIVE
    return sm, ctx


def _deactivate(sm, ctx):
    ctx.enqueue_command('deactivate')
    sm.tick(ctx)
    assert sm.state == RobotState.ACTIVE
    assert ctx.drain_requests() == ['deactivate']
    ctx.operation_pending = True


def _reply(ctx, ok, message):
    ctx.operation_pending = False
    ctx.operation_result = ok
    ctx.operation_message = message


def _notices(ctx):
    return ' | '.join(ctx.operator_notices)


@pytest.mark.parametrize('message,retry', [
    (REJECT_2026_10_23, True),
    ('service /activate_or_deactivate not ready', True),
    # A lost ack may have latched the lower: never a blind re-send.
    ('DEACTIVATE rejected: DEACTIVATE: ERR_TIMEOUT', False),
    (BUS_DOWN, False),
    ('DEACTIVATE rejected: DEACTIVATE: ERR_BAD_ARGS', False),
    ('deactivate FAILED — axis 2: TIMEOUT (succeeded: [0, 1])', False),
    ('deactivate FAILED — axis 0: timed out (DEACTIVATE ack lost; observed, '
     'not re-sent)', False),
    ('', False),
    (None, False),
])
def test_retry_classification(message, retry):
    assert deactivate_failure_is_retryable(message) is retry


def test_reply_deadline_covers_the_bridge_budget():
    """The no-reply deadline must outlast the bridge's own worst case (monitor
    timeout + 5 s hard deadline + 1 s wire wait), or a live bridge watching a
    stalled descent would be declared silent."""
    from teensy_link.deactivate import DEFAULT_TIMEOUT_S
    assert ActiveHandler.DEACTIVATE_REPLY_DEADLINE_S > DEFAULT_TIMEOUT_S + 5.0 + 1.0


def test_deactivate_stays_active_until_confirmed(clock):
    sm, ctx = _armed_machine()
    _deactivate(sm, ctx)
    sm.tick(ctx)
    assert sm.state == RobotState.ACTIVE      # in flight
    assert ctx.control_mode == 'STANDBY'      # A4: mode still published
    _reply(ctx, True, 'deactivate complete on axes [0, 1, 2, 3, 4, 5]')
    sm.tick(ctx)
    assert sm.state == RobotState.IDLE
    assert ctx.drain_requests() == []         # no redundant second deactivate


def test_transient_rejection_is_retried_and_converges(clock):
    """(b) The 2026-10-23 reply is retried after the spacing and converges."""
    sm, ctx = _armed_machine()
    _deactivate(sm, ctx)
    _reply(ctx, False, REJECT_2026_10_23)
    sm.tick(ctx)
    assert sm.state == RobotState.ACTIVE
    assert 'attempt 1/3 refused' in _notices(ctx)
    clock.t += 0.5
    sm.tick(ctx)
    assert ctx.drain_requests() == []         # spacing not yet elapsed
    clock.t += 0.6
    sm.tick(ctx)
    assert ctx.drain_requests() == ['deactivate']
    _reply(ctx, True, 'deactivate complete')
    sm.tick(ctx)
    assert sm.state == RobotState.IDLE


def test_genuine_rejection_stays_active_after_reconcile_window(clock):
    """(a) ERR_BUS_DOWN is not retried; after the reconcile window, with the legs
    still closed-loop, the machine stays ACTIVE and says why."""
    sm, ctx = _armed_machine()
    _deactivate(sm, ctx)
    _reply(ctx, False, BUS_DOWN)
    sm.tick(ctx)
    assert sm.state == RobotState.ACTIVE
    assert ctx.drain_requests() == []         # no retry
    assert 'REFUSED' not in _notices(ctx)     # no verdict before the window
    for _ in range(9):
        clock.t += 0.1
        sm.tick(ctx)
    clock.t += 0.2
    sm.tick(ctx)
    assert sm.state == RobotState.ACTIVE
    assert 'REFUSED after 1 attempt(s)' in _notices(ctx)
    assert 'ERR_BUS_DOWN' in _notices(ctx)
    # Normal ACTIVE again: IDLE may be requested again.
    _deactivate(sm, ctx)


def test_err_timeout_reply_is_not_resent(clock):
    sm, ctx = _armed_machine()
    _deactivate(sm, ctx)
    _reply(ctx, False, 'DEACTIVATE rejected: DEACTIVATE: ERR_TIMEOUT')
    for _ in range(15):
        clock.t += 0.1
        sm.tick(ctx)
    assert ctx.drain_requests() == []
    assert sm.state == RobotState.ACTIVE


def test_spent_retry_budget_stays_active(clock):
    sm, ctx = _armed_machine()
    _deactivate(sm, ctx)
    for attempt in range(1, ActiveHandler.DEACTIVATE_MAX_ATTEMPTS + 1):
        _reply(ctx, False, REJECT_2026_10_23)
        sm.tick(ctx)
        if attempt < ActiveHandler.DEACTIVATE_MAX_ATTEMPTS:
            clock.t += 1.1
            sm.tick(ctx)
            assert ctx.drain_requests() == ['deactivate'], attempt
            ctx.operation_pending = True
    clock.t += 1.1
    sm.tick(ctx)
    assert ctx.drain_requests() == []
    assert sm.state == RobotState.ACTIVE
    assert 'REFUSED after 3 attempt(s)' in _notices(ctx)


def test_failure_then_legs_leave_closed_loop_inside_window_follows_robot(clock):
    """(c / item 2) The legs go IDLE during the reconcile window: the machine
    follows the robot to IDLE after LEGS_OUT_TICKS consecutive ticks."""
    sm, ctx = _armed_machine()
    _deactivate(sm, ctx)
    _reply(ctx, False, 'deactivate FAILED — axis 3: timed out')
    sm.tick(ctx)
    assert sm.state == RobotState.ACTIVE
    ctx.legs_closed_loop = False
    for _ in range(ActiveHandler.LEGS_OUT_TICKS - 1):
        clock.t += 0.05
        sm.tick(ctx)
        assert sm.state == RobotState.ACTIVE
    clock.t += 0.05
    sm.tick(ctx)
    assert sm.state == RobotState.IDLE
    assert 'following the robot to IDLE' in _notices(ctx)
    # Second audit: disarm + Hand IDLE only, never a second leg DEACTIVATE.
    assert ctx.drain_requests() == ['disarm_and_idle_hand']


def test_reply_deadline_expiry_is_a_genuine_failure(clock):
    """Item 3: no reply by the deadline -> 'no reply from the bridge', no
    re-send, through the same reconcile window (legs closed-loop -> ACTIVE)."""
    sm, ctx = _armed_machine()
    _deactivate(sm, ctx)
    clock.t += ActiveHandler.DEACTIVATE_REPLY_DEADLINE_S - 0.1
    sm.tick(ctx)
    assert 'no reply' not in _notices(ctx)
    clock.t += 0.2
    sm.tick(ctx)                              # deadline passed -> reconcile
    clock.t += ActiveHandler.DEACTIVATE_RECONCILE_S + 0.1
    sm.tick(ctx)
    assert sm.state == RobotState.ACTIVE
    assert 'no reply from the bridge' in _notices(ctx)
    assert ctx.drain_requests() == []
    assert ctx.operation_pending is False     # no wedge behind a dead call


def test_only_mode_switches_discarded_during_deactivate(clock):
    """Item 3: while waiting, mode switches are discarded with one notice each;
    a duplicate 'deactivate' is absorbed; any other command stays queued."""
    sm, ctx = _armed_machine()
    _deactivate(sm, ctx)
    ctx.enqueue_command('trajectory')
    ctx.enqueue_command('deactivate')
    ctx.enqueue_command('clear_errors')
    ctx.enqueue_command('gui')
    sm.tick(ctx)
    assert ctx.drain_requests() == []         # no duplicate dispatch
    assert ctx.control_mode == 'STANDBY'      # mode NOT switched mid-stow (A4)
    assert len([n for n in ctx.operator_notices if 'discarded' in n]) == 2
    assert ctx.consume_command() == 'clear_errors'   # not discarded
    assert ctx.consume_command() is None


def test_active_wide_legs_out_rule(clock):
    """Item 2: armed ACTIVE, legs out of closed loop for LEGS_OUT_TICKS
    consecutive ticks -> IDLE; one glitch tick does not trip it."""
    sm, ctx = _armed_machine()
    ctx.legs_closed_loop = False
    sm.tick(ctx)
    ctx.legs_closed_loop = True               # single-sample glitch
    sm.tick(ctx)
    ctx.legs_closed_loop = False
    for _ in range(ActiveHandler.LEGS_OUT_TICKS - 1):
        sm.tick(ctx)
        assert sm.state == RobotState.ACTIVE
    sm.tick(ctx)
    assert sm.state == RobotState.IDLE
    # Second audit: the wire and the Hand are made safe, the legs untouched.
    assert ctx.drain_requests() == ['disarm_and_idle_hand']
    assert 'legs out of closed loop' in _notices(ctx)


def test_legs_out_rule_needs_closed_loop_evidence(clock):
    """No /robot_state evidence of closed loop in this visit -> no claim."""
    sm, ctx = _armed_machine()
    ctx2 = ctx
    sm.force_transition(RobotState.IDLE, ctx2)
    ctx2.legs_closed_loop = False
    sm.tick(ctx2)
    ctx2.enqueue_command('activate')
    sm.tick(ctx2)
    sm.tick(ctx2)
    ctx2.drain_requests()
    ctx2.operation_result = True
    sm.tick(ctx2)
    ctx2.drain_requests()
    ctx2.operation_result = True
    sm.tick(ctx2)
    for _ in range(ActiveHandler.LEGS_OUT_TICKS + 3):
        sm.tick(ctx2)
    assert sm.state == RobotState.ACTIVE


def test_mode_switch_refused_while_wire_disarmed(clock):
    """Owner policy: after a genuine refusal the robot is ACTIVE but disarmed;
    streaming mode switches are refused with a reason until an explicit re-arm."""
    sm, ctx = _armed_machine()
    _deactivate(sm, ctx)
    ctx.wire_armed = False                    # the bridge disarmed (A3)
    _reply(ctx, False, BUS_DOWN)
    sm.tick(ctx)
    clock.t += ActiveHandler.DEACTIVATE_RECONCILE_S + 0.1
    sm.tick(ctx)
    assert sm.state == RobotState.ACTIVE
    ctx.operator_notices.clear()
    ctx.enqueue_command('trajectory')
    sm.tick(ctx)
    assert ctx.control_mode == 'STANDBY'
    assert "'trajectory' refused: setpoint output is DISARMED" in _notices(ctx)
    ctx.wire_armed = True                     # operator re-armed explicitly
    ctx.enqueue_command('trajectory')
    sm.tick(ctx)
    assert ctx.control_mode == 'TRAJECTORY'


def test_manual_arm_session_may_switch_mode_before_arming(clock):
    """auto_arm False: runbook Sharp Edge #5 switches mode BEFORE the operator
    arms, so the disarmed-wire refusal does not apply there."""
    sm, ctx = _armed_machine()
    ctx.auto_arm = False
    ctx.wire_armed = False
    ctx.enqueue_command('gui')
    sm.tick(ctx)
    assert ctx.control_mode == 'GUI'


def test_orchestrator_node_keeps_active_on_refusal_and_logs_why(clock):
    """Node level (mocked ROS): the bridge's reject message reaches the handler
    via ctx.operation_message, the state stays ACTIVE, the reason is logged,
    and the retry goes out as a fresh service call."""
    from tests.ros.test_orchestrator_node import _Capture, _cmd, _to_idle
    from jugglebot.orchestrator_node import OrchestratorNode
    orch = OrchestratorNode()

    def resolve(ok, message=''):
        fut = orch._pending_future
        assert fut is not None
        fut.set_result(MagicMock(success=ok, message=message))
        orch._tick()

    _to_idle(orch)
    _cmd(orch, 'activate')
    orch._tick()
    orch._tick()
    resolve(True)                            # activate
    resolve(True)                            # arm
    orch._tick()
    cap = _Capture(orch)
    _cmd(orch, 'deactivate')
    orch._tick()
    assert orch._pending_label == 'deactivate'
    resolve(False, REJECT_2026_10_23)
    assert orch.sm.state == RobotState.ACTIVE
    assert any('attempt 1/3 refused' in m for m in cap.at('warning')), cap.lines
    clock.t += 1.1
    orch._tick()                             # retry dispatched
    assert orch._pending_label == 'deactivate'
    resolve(True, 'deactivate complete')
    orch._tick()
    assert orch.sm.state == RobotState.IDLE
    assert 'deactivate: ACTIVE -> IDLE' in cap.at('info')


def test_orchestrator_dispatches_disarm_and_hand_idle_without_a_leg_command():
    """Second audit, item 1: 'disarm_and_idle_hand' fires /set_setpoint_output
    false and /set_hand_state IDLE, and NO /activate_or_deactivate call."""
    from jugglebot.orchestrator_node import OrchestratorNode
    orch = OrchestratorNode()
    calls = []
    for name, client in (('arm', orch._setpoint_output_client),
                         ('hand', orch._hand_state_client),
                         ('legs', orch._activate_client)):
        client.call_async = (lambda req, _n=name:
                             (calls.append((_n, req)), MagicMock())[1])
    orch._current_req = 'disarm_and_idle_hand'
    orch._dispatch_request('disarm_and_idle_hand')
    names = [n for n, _ in calls]
    assert names == ['arm', 'hand'], names
    assert calls[0][1].data is False
    assert calls[1][1].data == 'IDLE'
    assert orch._pending_future is None       # fire-and-forget, not tracked
