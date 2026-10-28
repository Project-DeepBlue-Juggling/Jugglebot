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


def test_default_param_is_off_and_reaches_the_factory(monkeypatch):
    # Off by default since the 2026-10-28 Sitting (FIFO worker stalled the
    # setpoint stream past the firmware's 70 ms cover).
    seen = _capture_factory(monkeypatch)
    tn.TrajectoryNode(start_emitter=False)
    assert seen['rt_priority'] == 0
    assert sw.WORKER_RT_PRIORITY == 10   # the value to use WHEN enabled


def test_explicit_ten_reaches_the_factory(monkeypatch):
    seen = _capture_factory(monkeypatch)
    orig = tn.TrajectoryNode.declare_parameter

    def declare(self, name, default_value=None, *a, **kw):
        if name == 'solver_rt_priority':
            default_value = 10
        return orig(self, name, default_value, *a, **kw)
    monkeypatch.setattr(tn.TrajectoryNode, 'declare_parameter', declare)
    tn.TrajectoryNode(start_emitter=False)
    assert seen['rt_priority'] == 10


def test_emit_gap_warn_fires_from_status_timer_once_per_window(monkeypatch):
    _capture_factory(monkeypatch)
    node = tn.TrajectoryNode(start_emitter=False)
    lines = []
    node.get_logger().warning = lambda m, *a, **k: lines.append(str(m))
    assert tn.EMIT_GAP_WARN_MS == 45.0
    node._max_emit_gap_s = 0.040
    node._publish_status()
    assert not lines
    node._max_emit_gap_s = 0.090
    node._publish_status()
    assert len(lines) == 1 and 'emitter wake gap 90 ms' in lines[0]
    node._publish_status()          # window reset -> no repeat
    assert len(lines) == 1


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
