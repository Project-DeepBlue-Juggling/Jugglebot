"""P3b unit I2: goal acceptance by NOMINATION, ``skills/nominate``, ``skills/check``.

A Juggle goal is accepted or refused by a real offline plan of the Pattern's
nominal Segments (``motion/skills/nominate.py``) at the LIVE Sitting limits,
before anything moves (the offline admissible box was retired 2026-11-06). These
tests use the REAL planner (no mocked nomination) except
``test_the_nomination_runs_once_per_goal_and_never_for_magazine_or_reload``,
the one plumbing test, which stubs ``nominate`` to count calls.

Recipes for the pinned numbers (all at the launch constants unless a test
sets a status):

* viable at defaults (600 / 8000 / 200000 / Hand 4500, pose planner):
  hop 250 mm apex 0.9 dwell 0.27, columns 100 mm apex 0.9, high-low 100 mm
  1.00/0.90, oval 75 mm apex 0.9 (``python -c`` over ``nominate.nominate``,
  2026-11-06).
* refused at defaults: self_toss apex 1.4 (Hand 4951 > 4500 rev/s^2,
  HAND_LIMIT_ACC), oval 100 and 150 mm at dwell 0.27, hop 270 mm.
* the PHASE GATE (logbook/2026-10-31-pattern-progression-p4-high-low-columns):
  high-low 1.00/0.90 at separation 125, dwell 0.27, pose, at 350 / 5000 /
  200000 / 4500 is refused and the nearest viable value is separation 100.
"""
from __future__ import annotations

import dataclasses
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from std_srvs.srv import Trigger

import jugglebot.hardware_config as hw
from jugglebot import ball_possession
from jugglebot import juggle_request as jr
from jugglebot import skill_node as sn
from jugglebot.motion.skills import nominate as nom
from jugglebot.motion.trajectory.limits import TrajectoryLimits

import threading

import numpy as np

from tests.ros.conftest import SetString
from tests.ros.test_skill_node import (
    _ACCEPTED_CUP_CELLS, _assert_no_bb_and_no_motion, _columns_goal, _freshen,
    _goal, _hop_goal, _rec_logger, _node_with_client, _prelevel_ready, _reload_ready,
    _response, _self_toss_goal, _status)


def _ready_node(tag, *, seated=True):
    node, client = _node_with_client(response=_response())
    node._params['plant_id'] = 'test_nominate_%s_%s' % (tag, id(node))
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV,
             seated=seated)
    _prelevel_ready(node)
    return node, client


def _nominate_req(node, data):
    return node._svc_nominate(SimpleNamespace(data=data), SimpleNamespace())


# ── acceptance by nomination ─────────────────────────────────────────────────

def test_a_viable_goal_is_accepted_by_nomination_alone(tmp_path):
    """THE POINT OF THE PHASE: nothing on disk gates a goal; a viable hop
    is accepted on its nomination."""
    node, _ = _ready_node('nobox')
    with patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_hop_goal(separation_mm=250.0))
    assert resp.success is True, resp.message
    assert node._last_nomination_lines == []


def test_the_phase_gate_high_low_at_125_is_refused_before_any_motion(tmp_path):
    """High-low 1.00/0.90 at separation 125 at 350 / 5000 / 200000 / 4500 is
    refused BEFORE any motion (no Ball Butler call, no platform move) and the
    refusal names separation 100 as the viable value; at 100 it is accepted."""
    node, _client = _node_with_client(response=_response())
    node._params['plant_id'] = 'test_nominate_gate_%s' % id(node)
    node._params['apex_m_ball2'] = 0.9
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV,
             seated=True)
    # AFTER `_freshen` (it re-sends the default status)
    node._on_traj_status(_status(leg_vel_limit_mmps=350.0,
                                 leg_acc_limit_mmps2=5000.0,
                                 leg_jerk_limit_mmps3=200000.0))
    reload_client, throw_client = _reload_ready(node)
    pre = _prelevel_ready(node)
    with patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_goal('columns_highlow', apex_m=1.0,
                                         separation_mm=125.0))
        assert resp.success is False
        assert resp.message.startswith('nomination REFUSED: columns_highlow')
        assert 'separation 125 mm' in resp.message
        assert 'nearest viable: separation 100 mm' in resp.message
        assert len(node._last_nomination_lines) >= 2
        _assert_no_bb_and_no_motion(node, reload_client, throw_client)
        assert pre.calls == []
        ok = node._start_pattern(_goal('columns_highlow', apex_m=1.0,
                                       separation_mm=100.0))
    assert ok.success is True, ok.message


def test_a_columns_dwell_at_or_above_the_flight_is_a_compile_refusal(tmp_path):
    node, _ = _ready_node('compile')
    reload_client, throw_client = _reload_ready(node)
    with patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_goal(separation_mm=100.0,
                                                 dwell_s=2.0))
    assert resp.success is False
    assert 'nomination REFUSED' in resp.message and 'dwell 2.00 s' in resp.message
    assert any(l.startswith('COMPILE') for l in node._last_nomination_lines)
    _assert_no_bb_and_no_motion(node, reload_client, throw_client)


def test_a_refusal_is_one_message_line_and_the_lines_are_kept(tmp_path):
    node, _ = _ready_node('refuse')
    node._params['apex_m'] = 1.4
    resp = node._start_pattern(_self_toss_goal())
    assert resp.success is False
    assert '\n' not in resp.message
    assert node._last_nomination_lines
    assert all('HAND_LIMIT_ACC' in l or 'skill' in l
               for l in node._last_nomination_lines)


# ── the limits the nomination judges ─────────────────────────────────────────

def test_the_nomination_receives_the_live_limits_as_the_planner_builds_them():
    """`_nomination_limits()` is `TrajectoryLimits.from_config(hw)
    .with_sitting_limits(leg vel/acc/jerk)` -- the chain `trajectory_node.
    _svc_set_limits` applies to `self._limits` (seeded by `from_config(hw)`),
    compared FIELD FOR FIELD here after a `set_limits` status update; the Hand
    acceleration is the YAML value (the status message carries none)."""
    node, _ = _node_with_client()
    node._on_traj_status(_status(leg_vel_limit_mmps=420.0,
                                 leg_acc_limit_mmps2=6100.0,
                                 leg_jerk_limit_mmps3=170000.0))
    got = node._nomination_limits()
    want = TrajectoryLimits.from_config(hw).with_sitting_limits(
        leg_vel_mmps=420.0, leg_acc_mmps2=6100.0, leg_jerk_mmps3=170000.0)
    assert dataclasses.asdict(got) == dataclasses.asdict(want)
    assert got.leg_vel_mmps == 420.0 and got.hand_acc_limit_rps2 == float(
        hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2)
    # ... and the nomination really gets them
    seen = []
    real = nom.nominate

    def spy(config, limits=None, *a, **kw):
        seen.append(limits)
        return real(config, limits, *a, **kw)
    node._on_traj_status(_status(leg_vel_limit_mmps=333.0))
    with patch.object(sn.nom, 'nominate', spy):
        node._nominate(node._nomination_config('self_toss', 0.9, 250.0, 0.27))
    assert seen[0].leg_vel_mmps == 333.0


def test_the_nomination_runs_once_per_goal_and_never_for_magazine_or_reload(
        tmp_path):
    """PLUMBING (stubbed nominate): one call per accepted/refused goal of a
    nominated pattern, none for `magazine` / `reload`."""
    calls = []

    def stub(config, limits=None, *a, **kw):
        calls.append(config)
        return SimpleNamespace(ok=True, summary=lambda: 'nomination OK: stub',
                               lines=lambda: [], solves=3)
    node, _ = _ready_node('plumb')
    with patch.object(sn.nom, 'nominate', stub), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        assert node._start_pattern(_hop_goal(separation_mm=250.0)).success
        assert [c.pattern for c in calls] == ['hop']
        assert calls[0].separation_mm == 250.0 and calls[0].dwell_s == pytest.approx(
            sn._pattern_default_dwell_s('hop'))
        node2, _ = _ready_node('plumb2', seated=False)
        _reload_ready(node2)
        node2._start_pattern(_goal('reload'))
        node2._start_pattern(_goal('magazine'))
    assert [c.pattern for c in calls] == ['hop']


# ── the node and the nomination build the identical Pattern ──────────────────

def _site_sig(sites):
    return [(s.name, tuple(float(x) for x in s.cup_mm)) for s in sites]


def _pattern_sig(p):
    return (type(p).__name__, _site_sig(p.sites), p.apex_m,
            getattr(p, 'apex_m_ball_b', None), p.dwell_s,
            tuple(getattr(p, 'phantom_balls', ())), p.n_throws,
            getattr(p, 'roles', None))


_PATTERN_CELLS = [(p, ev) for p, ev in _ACCEPTED_CUP_CELLS
                  if p not in ('reload',)]


@pytest.mark.parametrize('pattern,evidence', _PATTERN_CELLS)
def test_the_node_and_the_nomination_build_the_identical_pattern(
        tmp_path, pattern, evidence):
    node, _client = _node_with_client(response=_response())
    node._params['plant_id'] = 'test_nom_pat_%s_%s' % (pattern, id(node))
    _freshen(node, pos_meas=0.0, pos_cmd=0.0,
             seated=(evidence == ball_possession.EVIDENCE_SEATED))
    _prelevel_ready(node)
    _reload_ready(node)
    separation = (250.0 if pattern == 'hop'
                  else 75.0 if pattern.startswith('oval_1ball') else 100.0)
    goal_kw = {}
    if pattern == 'columns_highlow':
        node._params['apex_m_ball2'] = 0.85
        goal_kw = dict(apex_m=0.9)
    built, configs = [], []
    real_build, real_nom = nom.build_pattern, nom.nominate

    def spy_build(config, n_throws=None):
        out = real_build(config, n_throws)
        built.append((config, n_throws, out))
        return out

    def spy_nom(config, *a, **kw):
        configs.append(config)
        return real_nom(config, *a, **kw)
    with patch.object(sn.nom, 'build_pattern', spy_build), \
         patch.object(sn.nom, 'nominate', spy_nom), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_goal(pattern, separation_mm=separation,
                                         **goal_kw))
    assert resp.success is True, resp.message
    assert len(configs) == 1
    config, n_throws, node_pattern = built[-1]
    # the node's builder call carries the SAME configuration the nomination
    # judged (everything but the operator's throw count) ...
    assert config == configs[0]
    assert config.entry == ('hop' if pattern in nom.COLUMNS_FED else 't0')
    # ... and the pattern the nomination plans (short count) differs from the
    # node's only in n_throws
    nominated = real_build(configs[0])
    a, b = _pattern_sig(node_pattern), _pattern_sig(nominated)
    assert a[:6] == b[:6] and a[7:] == b[7:], (a, b)


# ── skills/nominate ──────────────────────────────────────────────────────────

def test_the_parser_is_shared_and_the_relay_grammar_is_strict():
    node, _ = _node_with_client()
    assert jr.parse_juggle_request('hop,apex_m=0.8') == ('hop', {'apex_m': 0.8})
    with pytest.raises(ValueError):
        jr.parse_juggle_request('columns_highlow,apex_m_ball2=0.9')
    assert jr.parse_juggle_request(
        'columns_highlow,apex_m_ball2=0.9,columns_angle_deg=45',
        extra_keys=jr.NOMINATE_EXTRA_FIELDS)[1] == {
            'apex_m_ball2': 0.9, 'columns_angle_deg': 45.0}


def test_nominate_reply_for_a_viable_request():
    node, _ = _node_with_client()
    resp = _nominate_req(node, 'hop,separation_mm=250')
    assert resp.success is True
    assert resp.message.startswith('nomination OK: hop apex 0.90 m, separation 250 mm')
    assert '\n' not in resp.message


def test_nominate_reply_for_a_refused_request_lists_every_refusal():
    node, _ = _node_with_client()
    resp = _nominate_req(node, 'self_toss,apex_m=1.4')
    assert resp.success is False
    first, *rest = resp.message.split('\n')
    assert first.startswith('nomination REFUSED: self_toss apex 1.40 m')
    assert rest and all('HAND_LIMIT_ACC' in l for l in rest)


def test_nominate_blank_fields_fall_back_to_the_node_parameters():
    node, _ = _node_with_client()
    node._params['apex_m'] = 0.8
    node._params['separation_mm'] = 200.0
    resp = _nominate_req(node, 'hop')
    assert 'apex 0.80 m, separation 200 mm' in resp.message


def test_nominate_extra_keys_override_the_parameters_for_that_call_only():
    node, _ = _node_with_client()
    node._params['apex_m_ball2'] = 0.9
    resp = _nominate_req(node, 'columns_highlow,apex_m=1,separation_mm=100,'
                               'apex_m_ball2=0.85')
    assert 'apex 1.00/0.85 m' in resp.message, resp.message
    assert node._params['apex_m_ball2'] == 0.9          # nothing written
    from jugglebot.robot_frame import RobotFrame
    node._robot_frame = RobotFrame.IDENTITY
    node._bb_base_pose = (np.array([0.0, -1500.0, 0.0]), None, 0.0)
    turned = _nominate_req(node, 'columns_turned,separation_mm=125,'
                                 'columns_angle_deg=45')
    assert 'angle 45 deg' in turned.message, turned.message
    assert node._params['columns_angle_deg'] != 45.0     # nothing written


def test_nominate_refuses_a_bad_request_by_name():
    node, _ = _node_with_client()
    bad = _nominate_req(node, 'hop,apex=1.2')
    assert bad.success is False and 'unknown token' in bad.message
    unk = _nominate_req(node, 'juggle')
    assert unk.success is False and 'not one of' in unk.message
    mag = _nominate_req(node, 'magazine')
    assert mag.success is True and 'not nominated' in mag.message


def test_nominate_never_moves_anything():
    node, client = _node_with_client()
    pre = _prelevel_ready(node)
    resp = _nominate_req(node, 'hop')
    assert resp.success is True
    assert node._executor is None
    assert client.calls == [] and pre.calls == []


def test_nominate_is_refused_at_once_while_an_attempt_runs():
    """A solve would contend for the GIL with the tick: no solve, no cache
    read, executor untouched, nothing above DEBUG logged."""
    node, client = _node_with_client()
    _nominate_req(node, 'hop')                   # warm the cache
    sentinel = object()
    node._executor = sentinel
    rec = _rec_logger(node)
    calls = []
    with patch.object(sn.nom, 'nominate',
                      lambda *a, **k: calls.append(a) or None):
        resp = _nominate_req(node, 'hop')
    assert resp.success is False
    assert resp.message == ('nomination unavailable: an Attempt is running '
                            '(ask again when it has ended)')
    assert calls == []
    assert node._executor is sentinel and client.calls == []
    assert all(sev == 'DEBUG' for sev, _m in rec.lines)


def test_skills_nominate_is_created_on_its_own_callback_group():
    node, _ = _node_with_client()
    assert node._nominate_cbgroup is not node._juggle_cbgroup
    assert node._nominate_cbgroup is not node._cbgroup
    assert node._nominate_cbgroup is not node._sub_cbgroup
    svc = node._services['skills/nominate']
    assert svc.srv_type is SetString
    assert svc.callback_group is node._nominate_cbgroup


# ── skills/check ─────────────────────────────────────────────────────────────

def test_check_prints_one_nomination_line_per_configuration():
    node, _ = _node_with_client()
    _freshen(node)
    resp = node._svc_check(Trigger.Request(), Trigger.Response())
    parts = resp.message.split('; ')
    # defaults (apex 0.9, separation 250, dwell 0.27): self_toss and hop are
    # viable, the oval is refused and reported as informational
    assert [p for p in parts if p.startswith('nomination self_toss: OK')]
    assert [p for p in parts if p.startswith('nomination hop: OK')]
    assert 'nomination oval_1ball: REFUSED' in resp.message
    assert 'informational' in resp.message
    assert resp.success is True
    assert 'acceptance is decided by nomination' in resp.message


def test_check_does_not_solve_while_an_attempt_runs():
    node, _ = _node_with_client()
    _freshen(node)
    node._executor = object()
    calls = []
    with patch.object(sn.nom, 'nominate', lambda *a, **k: calls.append(a)):
        resp = node._svc_check(Trigger.Request(), Trigger.Response())
    assert calls == []
    assert resp.message.count('nomination: not evaluated (an Attempt is running)') == 1
    assert 'nomination self_toss' not in resp.message


# ── the double-accept window, and a nomination that raises ───────────────────

def test_a_second_goal_and_nominate_are_refused_during_the_first_nomination(
        tmp_path):
    node, _ = _ready_node('window')
    inside, release = threading.Event(), threading.Event()

    def blocking(config, *a, **kw):
        inside.set()
        assert release.wait(10.0)
        return SimpleNamespace(ok=True, summary=lambda: 'nomination OK: stub',
                               lines=lambda: [], solves=0)
    out = {}
    with patch.object(sn.nom, 'nominate', blocking), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        t = threading.Thread(target=lambda: out.update(
            first=node._start_pattern(_hop_goal(separation_mm=250.0))))
        t.start()
        try:
            assert inside.wait(10.0)
            second = node._start_pattern(_hop_goal(separation_mm=250.0))
            assert second.success is False
            assert 'already running' in second.message
            svc = _nominate_req(node, 'hop')
            assert svc.success is False
            assert svc.message.startswith('nomination unavailable')
        finally:
            release.set()
            t.join(10.0)
    assert out['first'].success is True, out['first'].message
    assert node._goal_starting is False


def test_the_starting_flag_is_cleared_when_the_nomination_raises(tmp_path):
    node, _ = _ready_node('flag')

    def boom(*a, **k):
        raise RuntimeError('planner exploded')
    with patch.object(sn.nom, 'nominate', boom), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_hop_goal(separation_mm=250.0))
    assert resp.success is False
    assert node._goal_starting is False
    # and a later goal is not refused as running
    assert 'already running' not in node._start_pattern(
        _hop_goal(separation_mm=250.0)).message


def test_an_unexpected_nomination_exception_rejects_the_goal_with_one_error():
    node, _ = _ready_node('boom')
    rec = _rec_logger(node)

    def boom(*a, **k):
        raise RuntimeError('planner exploded')
    with patch.object(sn.nom, 'nominate', boom):
        resp = node._start_pattern(_hop_goal(separation_mm=250.0))
    assert resp.success is False
    assert resp.message == 'nomination failed: RuntimeError: planner exploded'
    errs = [m for sev, m in rec.lines if sev == 'ERROR']
    assert errs == [resp.message]
    assert node._executor is None


def test_an_unexpected_nomination_exception_answers_the_service():
    node, _ = _node_with_client()

    def boom(*a, **k):
        raise RuntimeError('planner exploded')
    with patch.object(sn.nom, 'nominate', boom):
        resp = _nominate_req(node, 'hop')
    assert resp.success is False
    assert resp.message.split('\n')[0] == (
        'nomination failed: RuntimeError: planner exploded')


# ── the nominated Schedule is the flown one (D) ──────────────────────────────

@pytest.mark.parametrize('pattern,entry', [
    ('self_toss', 't0'), ('hop', 't0'), ('oval_1ball', 't0'),
    ('oval_1ball_fed', 't0'), ('columns_1ball', 't0'),
    ('columns', 'hop'), ('columns_turned', 'hop'), ('columns_highlow', 'hop'),
    ('columns_1ball_fed', 'hop')])
def test_the_nomination_entry_matches_the_flown_compile(pattern, entry):
    node, _ = _node_with_client()
    assert node._nomination_config(pattern, 0.9, 125.0, 0.27).entry == entry
    # the one flag both the compile (`_install_columns_schedule`) and the
    # config read
    assert sn._FED_COLUMNS_HOP_ENTRY is True


def test_nominate_request_dwell_is_used_and_zero_is_the_pattern_default():
    """P4 B3a: `skills/nominate` takes `dwell_s=` in the request; absent = the
    Pattern default (the node parameter is retired)."""
    node, _ = _ready_node('nomdwell')
    with_dwell = _nominate_req(node, 'oval_1ball,separation_mm=150,dwell_s=0.65')
    without = _nominate_req(node, 'oval_1ball,separation_mm=150')
    assert 'dwell 0.65 s' in with_dwell.message
    assert 'dwell 0.27 s' in without.message


def test_the_service_walks_the_aim_figure_and_the_accept_only_prints_it():
    """BUG-73 (owner option C), cost-neutral accept: `_nominate` hands the
    nomination the LIVE authorities only when asked (`with_aim`, the
    `skills/nominate` service); the goal accept's nominate never walks the
    second figure (zero aim solves) but prints both when the service already
    primed the cache for the CURRENT authorities."""
    nom.clear_cache()
    node, _client = _ready_node('aim')
    node._params['catch_outward_authority_mm'] = 6.0
    node._params['catch_inward_authority_mm'] = -3.0
    assert node._nomination_aim() == sn.nom.AimAuthority(30.0, 6.0, 0.0)
    node._params['catch_outward_authority_mm'] = 10.0
    node._params['catch_inward_authority_mm'] = 35.0
    seen = []
    real = nom.nominate

    def spy(config, limits=None, *a, **kw):
        seen.append(kw.get('aim'))
        return real(config, limits, *a, **kw)
    cfg = node._nomination_config('hop', 0.9, 250.0, 0.27)
    with patch.object(sn.nom, 'nominate', spy):
        plain = node._nominate(cfg)                    # the accept's call
        assert seen == [None] and plain.report.aim_solves == 0
        assert ' at the nominal Sites' not in plain.summary
        both = node._nominate(cfg, with_aim=True)      # the service's call
        assert seen[-1] == sn.nom.AimAuthority(30.0, 10.0, 35.0)
        assert both.ok and ' at the nominal Sites' in both.summary
        assert 'with the Catch re-aim at its limit' in both.summary
        # accept again: no walk, both figures printed from the primed cache
        solves = both.report.aim_solves
        again = node._nominate(cfg)
        assert seen[-1] is None and again.report.aim_solves == solves
        assert 'with the Catch re-aim at its limit' in again.summary
        # a changed live authority: the stale figure is NOT printed
        node._params['catch_outward_authority_mm'] = 5.0
        stale = node._nominate(cfg)
        assert ' at the nominal Sites' not in stale.summary


def test_accept_never_walks_the_aim_figure_and_the_service_does_not_while_busy():
    nom.clear_cache()
    node, _client = _ready_node('aim_accept')
    walked = []
    with patch.object(sn.nom, 'add_aim_figure',
                      lambda *a, **k: walked.append(1)):
        sn.SkillNode._nominate_at_accept(node, 'hop', 0.9, 250.0, 0.27)
    assert walked == []
    node._goal_starting = True       # a goal is starting: the service skips it
    with patch.object(sn.nom, 'add_aim_figure',
                      lambda *a, **k: walked.append(1)):
        res = _nominate_req(node, 'hop,apex_m=0.9,separation_mm=250')
    assert walked == [] and 'unavailable' in res.message
