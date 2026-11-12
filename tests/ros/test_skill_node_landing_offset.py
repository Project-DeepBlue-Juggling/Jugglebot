"""BUG-80: ``skill_node``'s ``landing_offset_x_mm`` / ``landing_offset_y_mm``
parameters, the thin wrapper over ``nominate.landing_offset_refusal`` and
``OneBallPattern.landing_offset_m`` (those are pinned in
``tests/motion/test_skills_landing_offset.py``).  Real planner, mocked ROS."""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from jugglebot import skill_node as sn
from jugglebot.motion.skills import schedule as sc

from tests.ros.test_skill_node import (
    _columns_goal, _freshen, _node_with_client, _prelevel_ready, _response,
    _self_toss_goal)


def _ready_node(tag):
    node, client = _node_with_client(response=_response())
    node._params['plant_id'] = 'test_landing_offset_%s_%s' % (tag, id(node))
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV,
             seated=True)
    _prelevel_ready(node)
    return node, client


def _set(node, name, value):
    return node._on_set_parameters([SimpleNamespace(name=name, value=value)])


def _throw_y_ds(node):
    out = []
    for sk in node._executor.schedule.skills:
        if sk.kind == sc.THROW:
            out.append(sk.y_d[0])
        elif sk.then_throw is not None:
            out.append(sk.then_throw.y_d[0])
    return out


def test_the_parameters_default_to_zero_and_leave_the_goal_untouched(tmp_path):
    node, _ = _ready_node('default')
    assert node._landing_offset_mm() == (0.0, 0.0)
    assert node._nomination_config('self_toss', 1.0, 100.0, 0.27
                                   ).landing_offset_mm == (0.0, 0.0)
    with patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_self_toss_goal(apex_m=1.0, num_cycles=3))
    assert resp.success is True, resp.message
    for dy in _throw_y_ds(node):
        assert dy.tobytes() == np.zeros(2).tobytes()
    assert not any(key.startswith('landing_offset')
                   for key, _ in node._goal_record_params)


def test_a_set_offset_moves_every_throw_and_is_named_in_the_record(tmp_path):
    node, _ = _ready_node('moved')
    assert _set(node, 'landing_offset_x_mm', 20.0).successful
    node._params['landing_offset_x_mm'] = 20.0
    with patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_self_toss_goal(apex_m=1.0, num_cycles=2))
    assert resp.success is True, resp.message
    dys = _throw_y_ds(node)
    assert len(dys) == 2
    for dy in dys:
        assert dy.tolist() == [0.02, 0.0]
    assert ('landing_offset_x_mm', '20') in node._goal_record_params


def test_a_single_throw_goal_flies_one_throw_with_no_fold(tmp_path):
    """The from-rest check's goal: ``num_cycles: 1`` through the real goal
    path compiles REST, THROW, CATCH (nothing carried), REST at the offset."""
    node, _ = _ready_node('single')
    node._params['landing_offset_x_mm'] = 20.0
    with patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_self_toss_goal(apex_m=1.0, num_cycles=1))
    assert resp.success is True, resp.message
    skills = node._executor.schedule.skills
    assert [s.kind for s in skills if s.kind in (sc.THROW, sc.CATCH)] == [sc.THROW, sc.CATCH]
    assert all(s.then_throw is None for s in skills)
    assert _throw_y_ds(node)[0].tolist() == [0.02, 0.0]


def test_a_columns_goal_refuses_a_non_zero_offset_by_name(tmp_path):
    node, _client = _ready_node('columns')
    node._params['landing_offset_y_mm'] = -15.0
    with patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_goal(separation_mm=100.0))
    assert resp.success is False
    assert 'landing offset' in resp.message and 'columns' in resp.message
    assert node._executor is None


def test_nominate_refuses_a_non_zero_offset_on_a_columns_request():
    node, _ = _node_with_client()
    node._params['landing_offset_x_mm'] = 20.0
    resp = node._svc_nominate(SimpleNamespace(data='columns,separation_mm=100'),
                              SimpleNamespace())
    assert resp.success is False
    assert 'landing offset' in resp.message


def test_nominate_shows_a_set_offset_on_a_self_toss_request():
    node, _ = _node_with_client()
    node._params['landing_offset_x_mm'] = 20.0
    resp = node._svc_nominate(SimpleNamespace(data='self_toss,apex_m=1'),
                              SimpleNamespace())
    assert 'landing offset (+20.0, +0.0) mm' in resp.message, resp.message


def test_the_offset_cannot_change_while_a_goal_runs():
    node, _ = _node_with_client()
    node._attempt_running = lambda: True
    assert _set(node, 'landing_offset_x_mm', 20.0).successful is False
    assert _set(node, 'landing_offset_y_mm', -10.0).successful is False
    assert _set(node, 'landing_offset_x_mm', 0.0).successful is True   # no change
    node._attempt_running = lambda: False
    assert _set(node, 'landing_offset_x_mm', 20.0).successful is True


def test_the_offset_cannot_change_while_a_goal_is_being_accepted():
    """Goal accept reads the parameter for the range check and the record, then
    again for the Pattern: a set between the two would fly one value and record
    another. No executor exists yet in that window, so _attempt_running is False."""
    node, _ = _node_with_client()
    node._attempt_running = lambda: False
    node._goal_starting = True
    assert _set(node, 'landing_offset_x_mm', 20.0).successful is False
    node._goal_starting = False
    assert _set(node, 'landing_offset_x_mm', 20.0).successful is True


def test_a_value_past_the_lateral_authority_is_refused():
    node, _ = _node_with_client()
    node._params['learner_lateral_authority_mm'] = 30.0
    assert _set(node, 'landing_offset_x_mm', 30.0).successful is True
    res = _set(node, 'landing_offset_x_mm', 30.5)
    assert res.successful is False
    assert 'learner_lateral_authority_mm' in res.reason
    assert _set(node, 'landing_offset_y_mm', float('nan')).successful is False
    assert _set(node, 'landing_offset_y_mm', 'twenty').successful is False


def test_the_goal_re_checks_the_authority_at_accept(tmp_path):
    """The authority may have been lowered since the offset was set."""
    node, _ = _ready_node('recheck')
    node._params['landing_offset_x_mm'] = 20.0
    node._params['learner_lateral_authority_mm'] = 10.0
    with patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_self_toss_goal(apex_m=1.0, num_cycles=1))
    assert resp.success is False
    assert 'learner_lateral_authority_mm' in resp.message
    assert node._executor is None
