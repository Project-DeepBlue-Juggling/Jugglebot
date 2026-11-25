"""The launch's bag record command records the hidden topics it lists (BUG-144).

A topic whose name holds a `_`-prefixed token (an action's `_action/feedback` and
`_action/status`) is hidden, and the record verb drops hidden topics, even ones named on
its command line, unless `--include-hidden-topics` is passed: 0 of 960 bags under the
rosbags directory held the Juggle action's feedback or status (2026-11-25). Source scan
of the launch file: the launch itself needs real ROS 2 and is not run here.
"""
from __future__ import annotations

import ast
from pathlib import Path

LAUNCH = (Path(__file__).resolve().parents[2]
          / 'ros_ws' / 'src' / 'jugglebot' / 'launch' / 'jugglebot_launch.py')


def _record_cmd():
    tree = ast.parse(LAUNCH.read_text())
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call) and getattr(node.func, 'id', None) == 'ExecuteProcess'):
            continue
        kw = {k.arg: k.value for k in node.keywords}
        name = kw.get('name')
        if isinstance(name, ast.Constant) and name.value == 'rosbag_record':
            return [e.value for e in kw['cmd'].elts if isinstance(e, ast.Constant)]
    raise AssertionError('no rosbag_record ExecuteProcess in %s' % LAUNCH)


def _hidden(topic: str) -> bool:
    return any(tok.startswith('_') for tok in topic.split('/') if tok)


def test_the_record_command_lists_hidden_topics():
    hidden = [a for a in _record_cmd() if a.startswith('/') and _hidden(a)]
    assert '/jugglebot/juggle/_action/feedback' in hidden
    assert '/jugglebot/juggle/_action/status' in hidden


def test_a_record_command_with_hidden_topics_includes_them():
    cmd = _record_cmd()
    assert cmd[:3] == ['ros2', 'bag', 'record']
    if any(a.startswith('/') and _hidden(a) for a in cmd):
        assert '--include-hidden-topics' in cmd, cmd
