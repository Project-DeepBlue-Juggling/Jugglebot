"""JP-11 ``columns_rotating`` through the node (build unit U2): its label and
cup rule, the ``columns_step_deg`` parameter, the refusals by name, the
``skills/nominate`` key, the IDENTICAL Pattern pin and the goal-start line.
The compile is ``tests/motion/test_skills_columns_rotating.py``; the builder is
``tests/motion/test_skills_columns_rotating_wiring.py``."""
from __future__ import annotations

from unittest.mock import patch

import pytest

from jugglebot import records as rec
from jugglebot import skill_node as sn
from jugglebot.motion.skills import nominate as nom
from jugglebot.motion.skills import sites as st
from tests.ros.test_skill_node import _MockParameter, _goal
from tests.ros.test_skill_node_nominate import _nominate_req, _ready_node


def _rot_goal(**kw):
    return _goal('columns_rotating', separation_mm=125.0, apex_m=0.95, **kw)


def _fingerprint(pat):
    return ([(s.name, tuple(float(v) for v in s.cup_mm)) for s in pat.sites],
            pat.apex_m, pat.dwell_s, pat.n_throws, pat.step_deg,
            pat.apex_m_ball_b, tuple(pat.phantom_balls), pat.orbit_sites)


def test_it_is_a_goal_pattern_with_columns_cup_rule_and_a_record_row():
    assert 'columns_rotating' in sn.SkillNode._PATTERNS
    assert sn._CUP_RULE['columns_rotating'] == sn._CUP_RULE['columns']
    assert ('columns_rotating', 'Columns — rotating', 2) in rec.PATTERNS


def test_step_is_a_parameter_default_zero_checked_and_not_changed_mid_goal():
    node, _ = _ready_node('rot_param')
    assert node.get_parameter('columns_step_deg').value == 0.0
    for bad in (-1.0, 20.5, float('nan')):
        res = node.set_parameters([_MockParameter(bad, name='columns_step_deg')])[0]
        assert res.successful is False and st.ROTATING_STEP_OUT_OF_RANGE in res.reason
    assert node.set_parameters(
        [_MockParameter(10.0, name='columns_step_deg')])[0].successful is True
    node._executor = object()          # `_attempt_running` reads this
    res = node.set_parameters([_MockParameter(5.0, name='columns_step_deg')])[0]
    assert res.successful is False and 'goal is active' in res.reason
    node._executor = None
    node._reload_ctx = None


def test_zero_step_the_level_planner_a_turned_pair_and_unequal_apex_refuse_by_name():
    node, _ = _ready_node('rot_refuse')
    node._params['planner_mode'] = 'pose'
    res = node._start_pattern(_rot_goal())
    assert res.success is False and st.ROTATING_STEP_OUT_OF_RANGE in res.message
    node._params['columns_step_deg'] = 10.0
    node._params['planner_mode'] = 'level'
    res = node._start_pattern(_rot_goal())
    assert res.success is False and 'pose-planner only' in res.message
    node._params['planner_mode'] = 'pose'
    node._params['columns_angle_deg'] = 30.0
    res = node._start_pattern(_rot_goal())
    assert res.success is False and 'ROTATING_WITH_ANGLE' in res.message
    node._params['columns_angle_deg'] = 0.0
    node._params['apex_m_ball2'] = 0.9
    res = node._start_pattern(_rot_goal())
    assert res.success is False and st.ROTATING_APEX_PAIR_UNEQUAL in res.message
    assert node._executor is None


def test_nominate_service_reads_step_deg_and_refuses_zero_by_name():
    node, _ = _ready_node('rot_svc')
    node._params['planner_mode'] = 'pose'
    refused = _nominate_req(node, 'columns_rotating,apex_m=0.95,'
                                  'separation_mm=125,dwell_s=0.27')
    assert refused.success is False
    assert st.ROTATING_STEP_OUT_OF_RANGE in refused.message
    refused = _nominate_req(node, 'columns_rotating,apex_m=0.95,'
                                  'separation_mm=125,step_deg=25')
    assert refused.success is False
    assert st.ROTATING_STEP_OUT_OF_RANGE in refused.message


def test_the_node_and_the_nomination_build_the_identical_pattern():
    """The accept's nomination config and the run path's config build the
    same Sites / Apex / Dwell / step; only the Throw count may differ
    (nomination: min(count, one full turn + 2); Attempt: the count)."""
    node, _ = _ready_node('rot_same')
    node._params['columns_step_deg'] = 10.0
    cfg_nom = node._nomination_config('columns_rotating', 0.95, 125.0, 0.27,
                                      step_deg=10.0, n_throws=4,
                                      feed_site='P2')
    assert cfg_nom.entry == 'hop' and cfg_nom.step_deg == 10.0
    pn = nom.build_pattern(cfg_nom)
    cfg_run = node._nomination_config('columns_rotating', 0.95, 125.0, 0.27,
                                      step_deg=10.0, feed_site='P1')
    pr = nom.build_pattern(cfg_run, 4)
    assert _fingerprint(pn) == _fingerprint(pr)
    assert pn.step_deg == 10.0 and pn.n_throws == 4


def test_a_goal_installs_the_rotating_schedule_and_the_start_line_names_the_step(
        tmp_path):
    node, _ = _ready_node('rot_goal', seated=True)
    node._params['planner_mode'] = 'pose'
    node._params['columns_step_deg'] = 10.0
    built = []
    real = nom.build_pattern

    def spy(config, n_throws=None):
        pat = real(config, n_throws)
        built.append((config, n_throws, pat))
        return pat
    with patch.object(sn, '_REPO_ROOT', str(tmp_path)), \
            patch.object(nom, 'build_pattern', side_effect=spy):
        res = node._start_pattern(_rot_goal(num_cycles=6))
    assert res.success is True, res.message
    assert res.attempt['step_deg'] == 10.0 and res.attempt['pattern'] == (
        'columns_rotating')
    assert ('step_deg', '10') in node._goal_record_params
    # the run path's Pattern: the operator's 6 Throws, the Feed Site P1
    cfg, n_arg, pat = built[-1]
    assert (cfg.pattern, n_arg, pat.n_throws, pat.step_deg) == (
        'columns_rotating', 6, 6, 10.0)
    assert [s_.name for s_ in pat.sites] == ['P2', 'P1']
    assert node._executor.schedule.skills[0].kind == 'REST'
