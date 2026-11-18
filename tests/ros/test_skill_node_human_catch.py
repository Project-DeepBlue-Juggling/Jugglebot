"""The ``human_catch`` Attempt lifecycle in skill_node (PWAY-11 P2, U5a).

The policy's rules are ``tests/motion/test_skills_human_catch_policy.py``;
the per-Throw compile is ``tests/motion/test_skills_human_catch.py``. What
this file pins is the NODE: accept, the opening REST to the wait Site,
events -> one level CATCH, the caught / missed / refused per-Throw ends, the
hold and the return, Stop from every phase (the leak test), and that Ball
Butler is never asked and no Memory row is written.

The per-Throw executor's own catch evidence is pinned in the executor's
tests (``tests/motion/test_skills_executor_human_catch.py``); most tests here
FORCE a per-Throw end (``_retire``) with the code the executor would end
with, so the node's reaction to each code is what is under test. The
real-verdict tests and the pending-Catch rows of the leak test do not force
anything: the CATCH stays live to its own verdict, read from the cup-sensor
history (``_fill_cup``), as on the robot.
"""
from __future__ import annotations

import math
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

from geometry_msgs.msg import Point, Vector3
from jugglebot_interfaces.msg import (BallButlerHeartbeat, HumanThrowEvent,
                                      ThrowAnnouncement)
from jugglebot_interfaces.srv import InstallSegment
from std_msgs.msg import String

from jugglebot import ball_possession
from jugglebot import skill_node as sn
from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import human_catch as hcp
from jugglebot.motion.skills.schedule import HUMAN_CATCH
from tests.ros.test_skill_node import (_freshen, _goal, _node_with_client,
                                       _prelevel_ready, _reload_ready,
                                       _response)

_T0 = 1000.0
_EMPTY = ball_possession.EVIDENCE_EMPTY
_SEATED = ball_possession.EVIDENCE_SEATED


def _hc_node(*, response=None, seated=False, in_traj=True, levelled=True):
    node, client = _node_with_client(response=response or _response())
    node._params['plant_id'] = 'test_hc_%d' % id(node)
    _freshen(node, seated=seated, pos_meas=sn.REST_HAND_REV,
             pos_cmd=sn.REST_HAND_REV, in_traj=in_traj, levelled=levelled)
    _prelevel_ready(node)
    clock = SimpleNamespace(t=_T0)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=int(round(clock.t * 1e9))))
    cup = [_EMPTY]
    node._cup_state = lambda: (cup[0], 0.0, '')
    return node, client, clock, cup


def _catches(client):
    return [c for c in client.calls
            if int(c.kind) == InstallSegment.Request.KIND_CATCH]


def _ctx(node):
    return node._reload_ctx


def _phase(node):
    return _ctx(node).policy.phase


def _tick(node, clock, t):
    clock.t = t
    node._on_tick()


def _rest_to_rest(node, clock):
    """Dispatch the live REST and let it come to rest: WAITING after."""
    rest = node._executor.schedule.skills[0]
    assert rest.kind == 'REST'
    _tick(node, clock, rest.dispatch_s())
    assert node._executor is None
    _tick(node, clock, rest.t_abs_s + 1e-3)


def _start(node, clock, n=0):
    resp = node._start_pattern(_goal('human_catch', num_cycles=n))
    assert resp.success is True, resp.message
    _rest_to_rest(node, clock)
    assert _phase(node) == hcp.WAITING
    return resp


def _event(node, clock, *, lead, x=-50.0, y=0.0, track=7, seq=0, rev=0,
           fit_age=0.0):
    m = HumanThrowEvent()
    m.track_id, m.flight_seq, m.revision = track, seq, rev
    m.landing_position = Point(x=float(x), y=float(y), z=830.0)
    m.landing_velocity = Vector3(x=0.0, y=0.0, z=-4580.0)
    t = clock.t + lead
    m.landing_time.sec = int(math.floor(t))
    m.landing_time.nanosec = int(round((t - math.floor(t)) * 1e9))
    m.fit_age_s = float(fit_age)
    node._on_human_throw(m)


def _retire(node, clock, code):
    """Force the live per-Throw executor to end with ``code`` ('' = caught)
    and let the tick retire it."""
    e = node._executor
    e.end_code = code
    e.attempt_ended = bool(code)
    e._pending_catches = []
    e._pending_outcomes = []
    e._fault_settle_until_s = None
    _tick(node, clock, clock.t + 0.025)
    assert node._executor is not e


def _fill_cup(node, t0, t1, seated_from=None):
    """Replace the cup-sensor history with LIVE 100 Hz samples over
    ``[t0, t1]`` (raw bit SEATED from ``seated_from`` on, else EMPTY): the
    real input of the executor's catch verdict (`SkillNode._seat_window`)."""
    node._hand_telemetry_hist.clear()
    n = int(round((t1 - t0) * 100.0))
    for k in range(n + 1):
        t = t0 + k * 0.01
        seated = seated_from is not None and t >= seated_from
        node._hand_telemetry_hist.append((t, True, seated, seated, 0.0, 0.0))


def _catch_land(node):
    return float(node._executor.schedule.skills[0].landing_prior.t_land_abs_s)


def _tick_until(node, clock, t_end, dt=0.025):
    while clock.t < t_end:
        _tick(node, clock, min(t_end, clock.t + dt))


def _plan_one(node, clock, client, lead=0.60, **kw):
    n0 = len(_catches(client))
    _event(node, clock, lead=lead, **kw)
    _tick(node, clock, clock.t + 0.005)
    assert _phase(node) == hcp.CATCHING
    assert len(_catches(client)) == n0 + 1


def _lines(node):
    log = MagicMock()
    node.get_logger = lambda: log
    return log


def _texts(log):
    return [str(c.args[0]) for m in (log.info, log.warning, log.error)
            for c in m.call_args_list]


# -- accept ------------------------------------------------------------------

def test_accepts_with_an_empty_cup_in_trajectory_and_opens_on_a_rest_only():
    node, client, clock, _cup = _hc_node()
    resp = node._start_pattern(_goal('human_catch'))
    assert resp.success is True, resp.message
    assert 'human_catch' in resp.message and 'waiting at' in resp.message
    assert node._executor.schedule.skills[0].kind == 'REST'
    assert len(node._executor.schedule.skills) == 1
    assert node._attempt_fed is False
    _rest_to_rest(node, clock)
    for k in range(80):                      # 2 s, no event
        _tick(node, clock, clock.t + 0.025)
    assert len(client.calls) == 1            # the opening REST only
    assert _phase(node) == hcp.WAITING and node._executor is None
    assert not node._goal_done_event.is_set()


def test_refused_outside_trajectory():
    node, client, _clock, _cup = _hc_node(in_traj=False)
    resp = node._start_pattern(_goal('human_catch'))
    assert resp.success is False
    assert ex.REJECTED_NOT_TRAJECTORY in resp.message
    assert client.calls == [] and node._executor is None
    assert node._reload_ctx is None


def test_refused_when_not_levelled():
    node, client, _clock, _cup = _hc_node(levelled=False)
    resp = node._start_pattern(_goal('human_catch'))
    assert resp.success is False
    assert ex.REJECTED_NOT_LEVELLED in resp.message
    assert client.calls == [] and node._reload_ctx is None


def test_refused_with_a_ball_in_the_cup():
    node, client, _clock, _cup = _hc_node(seated=True)
    node._cup_state = sn.SkillNode._cup_state.__get__(node)
    resp = node._start_pattern(_goal('human_catch'))
    assert resp.success is False
    assert ex.REJECTED_BALL_IN_HAND in resp.message
    assert client.calls == []


def test_parameters_default_from_the_policy_and_a_bad_one_refuses():
    node, _client, _clock, _cup = _hc_node()
    pol = node._hc_policy(0)
    ref = hcp.HumanCatchPolicy()
    for name in ('first_lead_s', 'plan_lead_s', 'max_fit_age_s',
                 'assoc_radius_mm', 'reaim_authority_mm', 'resend_max',
                 'mean_n', 'wait_radius_mm', 'wait_deadband_mm', 'clear_s',
                 'return_s'):
        assert getattr(pol, name) == getattr(ref, name), name
        assert ('human_catch_' + name) in node._params
    assert tuple(pol.home_xy_mm) == tuple(ref.home_xy_mm)
    node._params['human_catch_plan_lead_s'] = 0.50     # < first_lead_s
    resp = node._start_pattern(_goal('human_catch'))
    assert resp.success is False and 'plan_lead_s' in resp.message
    assert node._executor is None and node._reload_ctx is None


# -- events -> one CATCH ---------------------------------------------------

def test_an_in_window_event_installs_one_level_catch():
    node, client, clock, _cup = _hc_node()
    _start(node, clock)
    _plan_one(node, clock, client, lead=0.60, x=-20.0, y=30.0)
    sched = node._executor.schedule
    assert sched.pattern == HUMAN_CATCH and len(sched.skills) == 1
    assert sched.skills[0].kind == 'CATCH'
    assert tuple(sched.skills[0].site.cup_mm[:2]) == pytest.approx((-20.0, 30.0))
    assert node._executor.learner is None
    assert node._executor.on_experience is None


def test_an_early_event_plans_once_from_its_latest_revision_at_the_window():
    node, client, clock, _cup = _hc_node()
    _start(node, clock)
    t_land = clock.t + 0.80
    _event(node, clock, lead=0.80, x=-50.0)
    _tick(node, clock, clock.t + 0.025)
    assert _phase(node) == hcp.WAITING and _catches(client) == []
    clock.t += 0.05
    _event(node, clock, lead=t_land - clock.t, x=-20.0, rev=1)
    _tick(node, clock, t_land - 0.65)            # still above plan_lead_s
    assert _catches(client) == []
    _tick(node, clock, t_land - 0.615)           # inside the window
    assert len(_catches(client)) == 1
    assert node._executor.schedule.skills[0].site.cup_mm[0] == pytest.approx(-20.0)
    for _ in range(4):
        _tick(node, clock, clock.t + 0.025)
    assert len(_catches(client)) == 1


def test_a_refused_catch_moves_nothing_logs_once_and_keeps_waiting():
    node, client, clock, _cup = _hc_node()
    _start(node, clock)
    client._response = _response(accepted=False, code='SETTLE_SITE',
                                 message='settle site outside')
    log = _lines(node)
    _event(node, clock, lead=0.60, x=100.0)
    _tick(node, clock, clock.t + 0.005)
    assert len(_catches(client)) == 1
    _tick(node, clock, clock.t + 0.025)          # the refused executor retires
    assert node._executor is None
    assert _ctx(node) is not None and _phase(node) == hcp.WAITING
    assert node._goal_end_code == ''
    refused = [t for t in _texts(log) if 'HUMAN-CATCH-REFUSED' in t]
    assert len(refused) == 1 and 'SETTLE_SITE' in refused[0]
    assert not node._goal_done_event.is_set()
    for _ in range(20):
        _tick(node, clock, clock.t + 0.025)
    assert len(client.calls) == 2                # REST + the one refused CATCH
    assert sum('HUMAN-CATCH-REFUSED' in t for t in _texts(log)) == 1


def test_a_too_short_window_moves_nothing_and_keeps_waiting():
    node, client, clock, _cup = _hc_node()
    _start(node, clock)
    _event(node, clock, lead=0.56)
    _tick(node, clock, clock.t + 0.30)            # the tick came far too late
    assert _catches(client) == []
    assert _phase(node) == hcp.WAITING and _ctx(node) is not None


def test_a_refinement_re_aims_and_a_late_one_adds_nothing():
    node, client, clock, _cup = _hc_node()
    _start(node, clock)
    t_land = clock.t + 0.60
    _plan_one(node, clock, client, lead=0.60, x=-50.0)
    clock.t += 0.03
    _event(node, clock, lead=t_land - clock.t, x=-25.0, rev=1)
    _tick(node, clock, clock.t + 0.03)
    assert len(_catches(client)) == 2             # the re-aim
    log = _lines(node)
    _tick(node, clock, t_land - 0.10)
    _event(node, clock, lead=t_land - clock.t, x=-10.0, rev=2)
    _event(node, clock, lead=t_land - clock.t - 0.01, x=-10.0, rev=3)
    _tick(node, clock, clock.t + 0.02)
    assert len(_catches(client)) == 2             # frozen: no late re-aim
    assert sum('a later revision of the claimed Throw' in t
               for t in _texts(log)) <= 1


# -- per-Throw ends ----------------------------------------------------------

def test_caught_holds_until_empty_returns_and_serves_a_second_throw():
    node, client, clock, cup = _hc_node()
    _start(node, clock)
    _plan_one(node, clock, client, lead=0.60, x=-20.0)
    cup[0] = _SEATED
    _retire(node, clock, '')                      # caught
    assert _phase(node) == hcp.HOLDING and node._goal_end_code == ''
    for _ in range(40):                           # 1 s seated: holds
        _tick(node, clock, clock.t + 0.025)
    assert _phase(node) == hcp.HOLDING and node._executor is None
    cup[0] = _EMPTY
    _tick(node, clock, clock.t + 0.025)
    _tick(node, clock, clock.t + 0.25)
    assert _phase(node) == hcp.HOLDING            # EMPTY not yet 0.5 s
    _tick(node, clock, clock.t + 0.30)
    assert _phase(node) == hcp.RETURNING
    assert node._executor.schedule.skills[0].kind == 'REST'
    _rest_to_rest(node, clock)
    assert _phase(node) == hcp.WAITING
    _plan_one(node, clock, client, lead=0.60, x=-40.0, track=8)
    assert not node._goal_done_event.is_set()


def test_a_dead_cup_sensor_keeps_holding():
    node, client, clock, cup = _hc_node()
    _start(node, clock)
    _plan_one(node, clock, client)
    _retire(node, clock, '')
    cup[0] = ball_possession.EVIDENCE_UNKNOWN
    for _ in range(80):
        _tick(node, clock, clock.t + 0.025)
    assert _phase(node) == hcp.HOLDING and node._executor is None


def test_a_phantom_missed_catch_returns_then_waits():
    node, client, clock, _cup = _hc_node()
    _start(node, clock)
    _plan_one(node, clock, client)
    _retire(node, clock, ex.MISSED_CATCH)
    assert node._goal_end_code == ''
    assert _phase(node) == hcp.RETURNING
    assert node._executor.schedule.skills[0].kind == 'REST'
    _rest_to_rest(node, clock)
    assert _phase(node) == hcp.WAITING and _ctx(node) is not None


def test_the_catch_stays_live_to_its_real_verdict_then_holds():
    """No forced end: the per-Throw executor arms a pending Catch (the
    CATCH's own window, ``Skill.catch_open_abs_s``), stays live through
    touch-down, and the cup history's SEATED samples are the verdict."""
    node, client, clock, _cup = _hc_node()
    _start(node, clock)
    t_plan = clock.t
    _plan_one(node, clock, client)
    e = node._executor
    L = _catch_land(node)
    assert len(e._pending_catches) == 1
    assert e._pending_catches[0].t_open_s == pytest.approx(t_plan + 0.005)
    _fill_cup(node, t_plan - 0.05, L + 1.5, seated_from=L + 0.02)
    _tick_until(node, clock, L + 0.05)
    assert node._executor is e and _phase(node) == hcp.CATCHING
    _tick_until(node, clock, L + ex.MISSED_CATCH_CLOSE_AFTER_LANDING_S
                + ex.MISSED_CATCH_DECIDE_LAG_S + 0.03)
    assert node._executor is None
    assert _phase(node) == hcp.HOLDING and node._goal_end_code == ''
    assert not node._goal_done_event.is_set()


def test_a_real_miss_verdict_returns_to_the_wait_site():
    node, client, clock, _cup = _hc_node()
    _start(node, clock)
    t_plan = clock.t
    _plan_one(node, clock, client)
    L = _catch_land(node)
    _fill_cup(node, t_plan - 0.05, L + 2.0)             # never SEATED
    t_decide = (L + ex.MISSED_CATCH_CLOSE_AFTER_LANDING_S
                + ex.MISSED_CATCH_DECIDE_LAG_S)
    _tick_until(node, clock, t_decide + 0.03)
    assert node._executor is not None and node._executor.end_code == \
        ex.MISSED_CATCH                                  # BUG-68 settle
    _tick_until(node, clock, t_decide + ex.MISSED_CATCH_FAULT_SETTLE_S + 0.06)
    assert _phase(node) == hcp.RETURNING
    # the return REST was installed (its executor retires at its dispatch)
    assert int(client.calls[-1].kind) == InstallSegment.Request.KIND_REST
    assert node._goal_end_code == '' and not node._goal_done_event.is_set()


def test_a_lane_refusal_while_a_catch_is_pending_ends_once(tmp_path):
    node, client, clock, cup = _hc_node()
    _start(node, clock)
    t_plan = clock.t
    _plan_one(node, clock, client)
    L = _catch_land(node)
    assert node._executor._pending_catches
    _fill_cup(node, t_plan - 0.05, L + 2.0, seated_from=L + 0.02)
    with patch.object(sn.rp, 'end_line', wraps=sn.rp.end_line) as end_line:
        node._sched_refused_at_start = 3
        node._sched_refused = 4
        node._sched_refused_mono = sn.time.perf_counter()
        _tick_until(node, clock, L + 1.2)
    assert end_line.call_count == 1
    assert end_line.call_args.args[1] == ex.LANE_REFUSED
    assert node._goal_end_code == ex.LANE_REFUSED
    assert node._reload_ctx is None and node._executor is None
    assert node._goal_done_event.is_set()
    _assert_a_new_goal_is_accepted(node, cup, tmp_path)


def test_an_ending_code_ends_the_attempt_with_one_line():
    node, client, clock, _cup = _hc_node()
    _start(node, clock)
    _plan_one(node, clock, client)
    with patch.object(sn.rp, 'end_line', wraps=sn.rp.end_line) as end_line:
        _retire(node, clock, ex.LANE_REFUSED)
        _tick(node, clock, clock.t + 0.025)
    assert end_line.call_count == 1
    assert end_line.call_args.args[1] == ex.LANE_REFUSED
    assert node._reload_ctx is None and node._goal_done_event.is_set()
    assert node._goal_end_code == ex.LANE_REFUSED


def test_n_throws_ends_the_attempt_between_throws():
    node, client, clock, _cup = _hc_node()
    _start(node, clock, n=1)
    _plan_one(node, clock, client)
    with patch.object(sn.rp, 'end_line', wraps=sn.rp.end_line) as end_line:
        _retire(node, clock, ex.MISSED_CATCH)
        assert not node._goal_done_event.is_set()
        _rest_to_rest(node, clock)
    assert end_line.call_count == 1
    assert node._reload_ctx is None and node._goal_done_event.is_set()
    assert node._goal_end_code == ''


# -- Stop from every phase: the leak test -------------------------------------

def _assert_a_new_goal_is_accepted(node, cup, tmp_path):
    assert node._refuse_if_running(SimpleNamespace(success=None,
                                                   message=None)) is None
    # a NEW goal of another Pattern is accepted
    cup[0] = _EMPTY
    node._cup_state = sn.SkillNode._cup_state.__get__(node)
    _freshen(node, seated=False, pos_meas=sn.REST_HAND_REV,
             pos_cmd=sn.REST_HAND_REV)
    _reload_ready(node)
    with patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_goal('reload'))
    assert resp.success is True, resp.message


def _to_phase(node, clock, client, cup, phase):
    _start(node, clock)
    if phase == 'waiting':
        return
    t_plan = clock.t
    _plan_one(node, clock, client)
    if phase.startswith('catch_pending'):
        # Stop lands while the Catch is pending; its verdict arrives AFTER
        # the Stop (caught or missed), read from the real cup history.
        L = _catch_land(node)
        assert node._executor._pending_catches
        _fill_cup(node, t_plan - 0.05, L + 2.0,
                  seated_from=(L + 0.02 if phase.endswith('caught') else None))
        _tick_until(node, clock, L - 0.2)
        assert node._executor._pending_catches
        return
    cup[0] = _SEATED
    _retire(node, clock, '')
    if phase == 'holding':
        return
    cup[0] = _EMPTY
    _tick(node, clock, clock.t + 0.025)
    _tick(node, clock, clock.t + 0.6)
    assert _phase(node) == hcp.RETURNING and node._executor is not None


@pytest.mark.parametrize('phase', ['waiting', 'catch_pending_caught',
                                   'catch_pending_missed', 'holding',
                                   'returning'])
def test_stop_from_every_phase_ends_once_and_frees_the_node(tmp_path, phase):
    node, client, clock, cup = _hc_node()
    _to_phase(node, clock, client, cup, phase)
    with patch.object(sn.rp, 'end_line', wraps=sn.rp.end_line) as end_line:
        node._stop_attempt()
        if phase.startswith('catch_pending'):
            # the stopped executor waits for its verdict, then retires
            assert node._executor is not None
            assert not node._goal_done_event.is_set()
        # real time through the verdict and the BUG-68 settle; nothing forced
        _tick_until(node, clock, clock.t + 1.5)
    assert end_line.call_count == 1
    assert end_line.call_args.args[1] == 'STOPPED'
    assert node._goal_end_code == 'STOPPED'
    assert node._reload_ctx is None and node._executor is None
    assert node._goal_done_event.is_set()
    _assert_a_new_goal_is_accepted(node, cup, tmp_path)


# -- Ball Butler and Memory --------------------------------------------------

def test_ball_butler_is_never_asked_and_its_announcement_is_ignored_once():
    node, client, clock, _cup = _hc_node()
    reload_client, throw_client = _reload_ready(node)
    _start(node, clock)
    log = _lines(node)
    node._on_bb_heartbeat(BallButlerHeartbeat(
        state=0, ball_in_hand=True, connected=True))
    for _ in range(10):
        _tick(node, clock, clock.t + 0.025)
    for _ in range(3):
        node._on_announcement(ThrowAnnouncement(
            thrower_name='ball_butler', target_id=node._robot_name))
    node._on_bb_throw_outcome(String(data='THROW_REJECTED_BAD_STATE'))
    _tick(node, clock, clock.t + 0.025)
    assert reload_client.calls == [] and throw_client.calls == []
    assert _ctx(node) is not None and _phase(node) == hcp.WAITING
    assert node._goal_end_code == ''
    ignored = [t for t in _texts(log) if 'Ball Butler announcement' in t]
    assert len(ignored) == 1
    assert len(client.calls) == 1


def test_no_learner_and_no_memory_row_across_an_attempt():
    node, client, clock, cup = _hc_node()
    with patch.object(sn.SkillNode, '_make_learner') as make_learner, \
            patch.object(sn.SkillNode, '_open_memory') as open_memory:
        _start(node, clock)
        _plan_one(node, clock, client)
        assert node._executor.learner is None
        _retire(node, clock, '')
        cup[0] = _EMPTY
        _tick(node, clock, clock.t + 0.025)
        _tick(node, clock, clock.t + 0.6)
        _rest_to_rest(node, clock)
        node._stop_attempt()
        _tick(node, clock, clock.t + 0.025)
    make_learner.assert_not_called()
    open_memory.assert_not_called()
