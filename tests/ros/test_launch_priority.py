"""The launch's scheduling-priority contract (JB-76, 2026-10-28).

The Jetson runs this launch at 5.5 of 6 cores while juggling and the planner's
solver worker was measured getting ~45 % of a core (65 ms of CPU in 150 ms of
wall). The launch answers with ``prefix='nice -n 10'`` on the processes that
are NOT on the control chain, and NOTHING else: a nice on a control-chain node
would starve the setpoint stream, which is worse than a late CATCH. Pinned here
by reading the launch file's AST (``launch_ros`` is not importable in the
mocked-ROS gate), so the contract holds on every box the gate runs on.

Entry: logbook/2026-10-28-jb76-solve-tail-is-environment-starvation-instrumented.md
"""
from __future__ import annotations

import ast
import pathlib

LAUNCH = (pathlib.Path(__file__).resolve().parents[2]
          / 'ros_ws' / 'src' / 'jugglebot' / 'launch' / 'jugglebot_launch.py')

#: Off the control chain: yield under contention.
LOW_PRIORITY = {'rosbridge_websocket_lean', 'spacemouse_handler',
                'ball_butler_node', 'rosbag_record'}
#: On the control chain (mocap -> tracker -> skill -> trajectory + worker ->
#: teensy_bridge, and the orchestrator that dispatches the skills): never niced.
CONTROL_CHAIN = {'mocap_node', 'ball_tracker_node', 'skill_node',
                 'trajectory_node', 'teensy_bridge_node', 'orchestrator_node'}


def _str(node):
    return node.value if isinstance(node, ast.Constant) and isinstance(node.value, str) else None


def _process_prefixes():
    """``{executable-or-name: prefix-or-None}`` for every Node / ExecuteProcess call."""
    tree = ast.parse(LAUNCH.read_text(encoding='utf-8'))
    consts = {t.id: _str(n.value) for n in ast.walk(tree) if isinstance(n, ast.Assign)
              for t in n.targets if isinstance(t, ast.Name)}
    out = {}
    for call in ast.walk(tree):
        if not (isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
                and call.func.id in ('Node', 'ExecuteProcess')):
            continue
        kw = {k.arg: k.value for k in call.keywords}
        key = _str(kw.get('executable')) or _str(kw.get('name'))
        if key is None:
            continue
        pre = kw.get('prefix')
        # None = no prefix; anything the AST cannot resolve to a plain string
        # is '<unresolved>' so it still fails the control-chain assertion.
        if pre is None:
            out[key] = None
        elif isinstance(pre, ast.Name):
            out[key] = consts.get(pre.id) or '<unresolved>'
        else:
            out[key] = _str(pre) or '<unresolved>'
    return out


def test_the_off_chain_processes_are_niced_and_nothing_else_is():
    pre = _process_prefixes()
    missing = (LOW_PRIORITY | CONTROL_CHAIN) - set(pre)
    assert not missing, 'launch no longer defines %s' % sorted(missing)
    for name in sorted(LOW_PRIORITY):
        assert pre[name] == 'nice -n 10', (name, pre[name])
    for name in sorted(CONTROL_CHAIN):
        assert pre[name] is None, 'control-chain %s carries prefix %r' % (name, pre[name])
    others = {k: v for k, v in pre.items() if v and k not in LOW_PRIORITY}
    assert not others, 'unreviewed prefixes: %s' % others
