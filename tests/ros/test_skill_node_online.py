"""P3a: the node hands its online admissibility check to every executor.

`SkillNode._checker` plans a segment with `check_only` and installs
nothing; this phase WIRES it: every `SkillExecutor(...)` the node builds gets
`checker=self._checker` and `early_command=<admissibility_early_command>`
through `_online_kwargs`. (Goal ACCEPTANCE is decided by nomination;
see test_skill_node_nominate.py.)
"""
from __future__ import annotations

import os
import re

from unittest.mock import patch

from std_srvs.srv import Trigger

from jugglebot import skill_node as sn

from tests.ros.test_skill_node import (
    _columns_goal, _freshen, _node_with_client,
    _prelevel_ready, _self_toss_goal, _status)


def _start_columns(node, tmp_path):
    with patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_goal(separation_mm=100.0))
    assert resp.success is True, resp.message
    return node._executor


def test_the_early_command_parameter_defaults_on():
    node, _client = _node_with_client()
    assert node.get_parameter('admissibility_early_command').value is True


def test_a_columns_executor_gets_the_checker_and_the_early_switch(tmp_path):
    node, _client = _node_with_client()
    executor = _start_columns(node, tmp_path)
    assert executor.checker == node._checker
    assert executor.early_command is True


def test_the_early_switch_follows_the_parameter_at_goal_start(tmp_path):
    node, _client = _node_with_client()
    node._params['admissibility_early_command'] = False
    executor = _start_columns(node, tmp_path)
    assert executor.checker == node._checker
    assert executor.early_command is False


def test_a_self_toss_executor_gets_the_checker_and_the_early_switch():
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    _prelevel_ready(node)
    node._params['plant_id'] = 'test_online_self_toss'
    resp = node._start_pattern(_self_toss_goal())
    assert resp.success is True, resp.message
    assert node._executor.checker == node._checker
    assert node._executor.early_command is True


def test_every_executor_construction_site_hands_the_online_kwargs():
    """Source pin for the construction sites no goal fixture reaches (bridge
    / reload / chain / oval / hop executors): each `SkillExecutor(` call that
    takes `self._installer` also splats `**self._online_kwargs()`."""
    path = os.path.join(os.path.dirname(sn.__file__), 'skill_node.py')
    with open(path) as fh:
        src = fh.read()
    sites = [m.start() for m in re.finditer(r'SkillExecutor\(\s*\n?\s*\w+, '
                                            r'self\._installer', src)]
    assert len(sites) == 7, len(sites)
    for pos in sites:
        call = src[pos:pos + 400]
        assert '**self._online_kwargs()' in call.split('catch_aim_source')[0], call


def test_online_kwargs_carry_the_checker_and_the_parameter():
    node, _client = _node_with_client()
    kw = node._online_kwargs()
    assert kw == dict(checker=node._checker, early_command=True)
    node._params['admissibility_early_command'] = False
    assert node._online_kwargs()['early_command'] is False


def test_check_states_the_command_path_line():
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    _freshen(node)
    resp = node._svc_check(Trigger.Request(), Trigger.Response())
    assert ('admissibility: acceptance is decided by nomination (a real plan '
            'of the Pattern at the live limits); online check on the command '
            'path (margin 0.9, early_command=true)' in resp.message)
    node._params['admissibility_early_command'] = False
    resp = node._svc_check(Trigger.Request(), Trigger.Response())
    assert 'early_command=false)' in resp.message
