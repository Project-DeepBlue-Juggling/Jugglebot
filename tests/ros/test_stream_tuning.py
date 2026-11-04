"""JB-149 opt-in cures: gc_freeze and cpu_partition (both default OFF).

ROS 2 is mocked by tests/ros/conftest.py; the mock Node's declare_parameter
returns the default, so a test turns a flag on by wrapping it with overrides.
No wall-clock asserts (the Jetson is loaded)."""
from __future__ import annotations

import ast
import gc
import os
import pathlib

import pytest

from jugglebot import stream_tuning as st
from jugglebot import trajectory_node as tn
from jugglebot.motion.skills import solver_worker as sw

from tests.ros._bridge_harness import _build_paired_node, _teardown

LAUNCH = (pathlib.Path(__file__).resolve().parents[2] / 'ros_ws' / 'src'
          / 'jugglebot' / 'launch' / 'jugglebot_launch.py')


class _Spy:
    def __init__(self, monkeypatch):
        self.calls = []
        monkeypatch.setattr(os, 'sched_setaffinity',
                            lambda pid, cpus: self.calls.append(
                                ('aff', pid, set(cpus))))
        monkeypatch.setattr(gc, 'collect',
                            lambda *a: self.calls.append(('collect',)) or 0)
        monkeypatch.setattr(gc, 'freeze',
                            lambda: self.calls.append(('freeze',)))
        monkeypatch.setattr(gc, 'set_threshold',
                            lambda *t: self.calls.append(('thr',) + t))


def _override(monkeypatch, cls, **over):
    orig = cls.declare_parameter

    def declare(self, name, default_value=None, *a, **k):
        return orig(self, name, over.get(name, default_value), *a, **k)
    monkeypatch.setattr(cls, 'declare_parameter', declare)


def _logs(node):
    return getattr(node.get_logger(), 'msgs', None)


# ── pure helpers ───────────────────────────────────────────────

def test_parse_cpus_validates_against_cpu_count():
    assert st.parse_cpus('3, 1,1', ncpu=6) == [1, 3]
    assert st.parse_cpus([4, 5], ncpu=6) == [4, 5]
    for bad in ('', '6', '-1', 'x'):
        with pytest.raises(ValueError):
            st.parse_cpus(bad, ncpu=6)


def test_pin_process_fails_open():
    def boom(pid, cpus):
        raise OSError('nope')
    ok, txt = st.pin_process([0], setaffinity=boom, ncpu=6)
    assert not ok and 'nope' in txt
    ok, txt = st.pin_process([99], setaffinity=boom, ncpu=6)
    assert not ok and 'outside' in txt


def test_freeze_gc_order_and_thresholds(monkeypatch):
    spy = _Spy(monkeypatch)
    n, th = st.freeze_gc()
    assert [c[0] for c in spy.calls] == ['collect', 'freeze', 'thr']
    assert spy.calls[2] == ('thr',) + st.GC_THRESHOLDS
    assert isinstance(n, int)


def test_freeze_gc_real_does_not_disable():
    saved_th, was = gc.get_threshold(), gc.isenabled()
    try:
        st.freeze_gc()
        assert gc.isenabled() == was and gc.get_freeze_count() >= 0
        assert gc.get_threshold() == st.GC_THRESHOLDS
    finally:
        gc.unfreeze()
        gc.set_threshold(*saved_th)


# ── trajectory_node ────────────────────────────────────────────

def test_trajectory_flags_default_off_touch_nothing(monkeypatch):
    spy = _Spy(monkeypatch)
    node = tn.TrajectoryNode(start_emitter=False)
    try:
        assert spy.calls == []
        assert node._solver_cpus is None
    finally:
        node.destroy_node() if hasattr(node, 'destroy_node') else None


def test_trajectory_flags_on_pin_freeze_and_partition_worker(monkeypatch):
    spy = _Spy(monkeypatch)
    _override(monkeypatch, tn.Node, gc_freeze=True, cpu_partition=True)
    seen = {}

    class FakeSolver:
        def __init__(self, **kw):
            seen.update(kw)
            self.pid = None
    monkeypatch.setattr(tn, '_default_segment_solver_factory', FakeSolver)
    order = []
    monkeypatch.setattr(tn.TrajectoryNode, '_start_emitter',
                        lambda self: order.append(('emitter', len(spy.calls))))
    node = tn.TrajectoryNode(start_emitter=True)
    assert spy.calls[0] == ('aff', 0, {0, 1, 2, 3}), 'pin is the first act'
    assert [c[0] for c in spy.calls[1:]] == ['collect', 'freeze', 'thr']
    assert spy.calls[-1] == ('thr',) + st.GC_THRESHOLDS
    assert order == [('emitter', len(spy.calls))], 'emitter starts after both'
    assert seen['cpus'] == [4, 5]
    assert node._solver_cpus == [4, 5]


def test_trajectory_bad_cpus_fail_open(monkeypatch):
    spy = _Spy(monkeypatch)
    _override(monkeypatch, tn.Node, cpu_partition=True, stream_cpus=[0, 99])
    node = tn.TrajectoryNode(start_emitter=False)
    assert spy.calls == [] and node._solver_cpus is None


# ── bridge ─────────────────────────────────────────────────────

def test_bridge_flags_default_off_touch_nothing(monkeypatch):
    spy = _Spy(monkeypatch)
    teensy, client, node = _build_paired_node(boot_state_read=False)
    try:
        assert spy.calls == []
    finally:
        _teardown(teensy, client, node)


def test_bridge_flags_on_pin_first_then_freeze(monkeypatch):
    spy = _Spy(monkeypatch)
    from jugglebot import teensy_bridge_node as tbn
    _override(monkeypatch, tbn.Node, gc_freeze=True, cpu_partition=True)
    teensy, client, node = _build_paired_node(boot_state_read=False)
    try:
        assert spy.calls[0] == ('aff', 0, {0, 1, 2, 3})
        assert [c[0] for c in spy.calls[1:]] == ['collect', 'freeze', 'thr']
        assert getattr(node, '_sp_thread', None) is None, \
            'streaming thread not started before the freeze'
    finally:
        _teardown(teensy, client, node)


# ── worker ─────────────────────────────────────────────────────

def test_worker_argv_carries_cpus_and_main_applies_them(monkeypatch):
    spy = _Spy(monkeypatch)
    seen = {}
    monkeypatch.setattr(sw, 'apply_rt_policy',
                        lambda prio, cpus=sw.WORKER_RT_CPUS:
                        seen.update(prio=prio, cpus=tuple(cpus)) or
                        sw.SchedInfo('CFS', 0, 'unchanged', 'disabled'))
    monkeypatch.setattr(sw, 'worker_main', lambda fd, sched=None: 0)
    assert sw._main(['--fd', '3', '--cpus', '4,5']) == 0
    assert seen['cpus'] == (4, 5)
    assert ('aff', 0, {4, 5}) in spy.calls
    spy.calls.clear()
    assert sw._main(['--fd', '3']) == 0
    assert spy.calls == [] and seen['cpus'] == tuple(sw.WORKER_RT_CPUS)


def test_worker_class_stores_cpus():
    w = sw.WorkerSegmentSolver.__new__(sw.WorkerSegmentSolver)
    assert 'cpus' in sw.WorkerSegmentSolver.__init__.__code__.co_varnames


# ── launch ─────────────────────────────────────────────────────

def test_launch_exposes_both_args_default_false_and_passes_through():
    src = LAUNCH.read_text(encoding='utf-8')
    tree = ast.parse(src)
    defaults = {}
    for c in ast.walk(tree):
        if (isinstance(c, ast.Call) and getattr(c.func, 'id', '')
                == 'DeclareLaunchArgument' and c.args):
            for k in c.keywords:
                if k.arg == 'default_value' and isinstance(k.value, ast.Constant):
                    defaults[c.args[0].value] = k.value.value
    assert defaults['gc_freeze'] == 'false'
    assert defaults['cpu_partition'] == 'false'
    # the probe is ON at launch since 2026-11-04 (owner; JB-149 waits for a
    # LANE_REFUSED caught with it running)
    assert defaults['gc_probe'] == 'true'
    for flag in ('gc_freeze', 'cpu_partition'):
        # passed as a real bool, the gc_probe idiom (PythonExpression == 'true')
        assert src.count('\'%s\': PythonExpression' % flag) == 2, \
            'both trajectory_node and teensy_bridge_node get %s' % flag
        assert '%s_arg,' % flag in src
