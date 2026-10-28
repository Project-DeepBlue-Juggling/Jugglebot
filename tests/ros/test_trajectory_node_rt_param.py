"""JB-95: ``trajectory_node``'s ``solver_rt_priority`` parameter reaches the
solver factory, and the solver's READY line (with its ``sched=`` receipt)
comes out through ``_solver_log`` at INFO.  ROS is mocked (tests/ros/conftest)."""

from __future__ import annotations

import os

from jugglebot import trajectory_node as tn
from jugglebot.motion.skills import solver_worker as sw


class _Rec:
    def __init__(self):
        self.lines = []

    def info(self, msg, **_kw):
        self.lines.append(('info', msg))

    def warning(self, msg, **_kw):
        self.lines.append(('warning', msg))

    def error(self, msg, **_kw):
        self.lines.append(('error', msg))

    def debug(self, msg, **_kw):
        pass


def _capture_factory(monkeypatch):
    seen = {}

    def factory(**kw):
        seen.update(kw)
        return sw.InProcessSegmentSolver()
    monkeypatch.setattr(tn, '_default_segment_solver_factory', factory)
    return seen


def test_default_param_is_ten_and_reaches_the_factory(monkeypatch):
    seen = _capture_factory(monkeypatch)
    tn.TrajectoryNode(start_emitter=False)
    assert seen['rt_priority'] == sw.WORKER_RT_PRIORITY == 10


def test_param_zero_is_the_kill_switch_and_reaches_the_factory(monkeypatch):
    seen = _capture_factory(monkeypatch)
    orig = tn.TrajectoryNode.declare_parameter

    def declare(self, name, default_value=None, *a, **kw):
        if name == 'solver_rt_priority':
            default_value = 0
        return orig(self, name, default_value, *a, **kw)
    monkeypatch.setattr(tn.TrajectoryNode, 'declare_parameter', declare)
    tn.TrajectoryNode(start_emitter=False)
    assert seen['rt_priority'] == 0


def test_ready_info_line_carries_sched_through_solver_log(monkeypatch):
    node = tn.TrajectoryNode(start_emitter=False,
                             segment_solver=sw.InProcessSegmentSolver())
    rec = _Rec()
    node._logger = rec
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    solver = sw.WorkerSegmentSolver(env=env, log=node._solver_log,
                                    rt_priority=10)
    try:
        assert solver.wait_ready(20.0), solver.state
    finally:
        solver.close()
    ready = [m for lvl, m in rec.lines if lvl == 'info' and 'READY' in m]
    assert len(ready) == 1 and 'sched=' in ready[0], rec.lines
    if solver.sched_info.policy == 'FIFO':
        assert 'sched=FIFO/10 cpus=1-5' in ready[0]
        assert not [1 for lvl, _ in rec.lines if lvl == 'warning']
    else:
        assert 'sched=CFS (' in ready[0]
        warns = [m for lvl, m in rec.lines if lvl == 'warning']
        assert len(warns) == 1 and 'stayed on CFS' in warns[0]
