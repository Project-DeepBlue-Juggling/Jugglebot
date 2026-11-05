"""The wall-clock budget of the command-time CHECK request (learner-generalisation
P3a, ``plans/active`` learner-generalisation § 4): ONE columns catch-with-throw
plan + gate in a real ``WorkerSegmentSolver`` child, sent with
``SegmentSolveRequest(check=True)`` (``executor.check_segment``: plan + gate, no
install).

``serial`` (it measures wall clock; a baseline taken beside xdist workers is not a
baseline) and in the DEFAULT gate, deliberately NOT ``nightly``, for the argument
``test_validate_cycle_budget.py`` makes: a command-time check that has quietly grown
costlier is a check a future session will be tempted to skip on the command path, so
the commit that grows it must fail.

Measured on this Jetson 2026-11-06 with ``tools/probes/admissible_obviation/
loaded_budget.py --kind install`` (n = 300 each; one plan + gate through the worker;
wall p50 / p99 / max ms, worker cpu p50 / p99 ms):

=====  =====================================================  =====================  ================
label  condition                                              wall p50 / p99 / max   cpu p50 / p99
=====  =====================================================  =====================  ================
L0     idle                                                   46.2 / 49.1 / 51.1     43.2 / 45.8
L1n    launch + bag, no browser client                        48.6 / 51.6 / 73.8     45.0 / 46.8
L1h    launch + bag + headless Chromium page (software GL)    100.6 / 190.2 / 265.0  52.1 / 59.4
L2h    L1h + a niced parallel test gate                       122.1 / 208.6 / 229.5  52.6 / 59.8
L3h    L1h with the launch's cpu_partition (worker on 4,5)    81.0 / 143.9 / 202.9   50.6 / 55.3
=====  =====================================================  =====================  ================

Thresholds (this test runs on whatever load the gate has, so they are bracketed by
the table):

* wall p50 <= 75 ms: 1.5 x the L1n p50 (48.6), the light Sitting bracket.
* cpu p50 (``reply.cpu_s``) <= 45 ms: 1.33 x the idle CHECK's cpu p50 (33.7 ms,
  the three idle runs of this test on 2026-11-06: 33.4 / 33.7 / 33.8).  CPU time is
  nearly load-insensitive (an install's rose 1.22 x from L0 to L2h, 43.2 -> 52.6 ms),
  so THIS is the bound that catches a costlier check, loaded box or not; the wall
  bound catches a starved one.
* wall max <= 265 ms: the L1h max.

The wall thresholds are derived from the ``--kind install`` measurement, an upper
bound for a CHECK (the same plan + gate plus the install bookkeeping; a CHECK is
about 0.8 of it idle).  OPEN ITEM: re-derive them from a ``--kind check`` run under
a Sitting-loaded launch when one is next available.

Probed 2026-11-06 (idle box, ``loaded_budget.py --kind check --n 20``): wall p50
35.9 ms, cpu p50 34.2 ms, margin p50 0.38.
"""

from __future__ import annotations

import importlib.util
import os
import types

# The launch's BLAS pin must be in place before numpy is imported anywhere.
for _v in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS',
           'NUMEXPR_NUM_THREADS'):
    os.environ[_v] = '1'

import time                                                           # noqa: E402

import numpy as np                                                    # noqa: E402
import pytest                                                         # noqa: E402

import jugglebot.hardware_config as hw                                # noqa: E402,F401
from jugglebot.motion.geometry import StewartGeometry                 # noqa: E402
from jugglebot.motion.skills import segments as sg                    # noqa: E402
from jugglebot.motion.skills import solver_worker as sw               # noqa: E402

pytestmark = pytest.mark.serial

N = 40
PERIOD_S = 0.1
WALL_P50_MS = 75.0
CPU_P50_MS = 45.0
WALL_MAX_MS = 265.0

_PROBE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'tools',
                      'probes', 'admissible_obviation', 'loaded_budget.py')


def _probe():
    # ONE builder, the probe's ``Chain`` (not tests/motion/test_solver_worker.py's):
    # the table above was measured with exactly its launch THROW and its columns
    # catch-with-throw terminals at the P4 working point, so the test and the
    # measurement cannot drift apart.
    spec = importlib.util.spec_from_file_location('loaded_budget_probe', _PROBE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_a_check_request_fits_the_wall_clock_and_cpu_budget():
    lb = _probe()
    geom = StewartGeometry()
    limits = lb.asw.sitting_limits(600.0, 8000.0, 200000.0, 4500.0)
    cfg = sg.SegmentConfig(planner_mode='pose')
    solver = sw.WorkerSegmentSolver(env=dict(os.environ))
    try:
        assert solver.wait_ready(30.0)
        chain = lb.Chain(solver, types.SimpleNamespace(kind='check', apex=0.95),
                         limits, geom, cfg)      # installs the launch THROW
        offs = lb.OFFSETS_MM
        for off in offs[:3]:                      # warm the catch path
            _ms, rep = chain.solve(off)
            assert rep.result.accepted, rep.result.message
        wall, cpu, margin, bad = [], [], [], []
        for i in range(N):
            t_next = time.perf_counter() + PERIOD_S
            ms, rep = chain.solve(offs[i % len(offs)])
            wall.append(ms)
            cpu.append(rep.cpu_s * 1e3)
            margin.append(rep.margin)
            if not rep.result.accepted or not rep.margin >= 0.0:
                bad.append((i, rep.result.code, rep.margin))
            time.sleep(max(0.0, t_next - time.perf_counter()))
    finally:
        solver.close()

    wall, cpu = np.array(wall), np.array(cpu)
    print('CHECK x%d: wall p50 %.1f p99 %.1f max %.1f ms, cpu p50 %.1f ms, '
          'margin min %.3f' % (N, np.median(wall), np.percentile(wall, 99), wall.max(),
                               np.median(cpu), min(margin)))
    assert not bad, bad[:3]
    assert np.median(wall) <= WALL_P50_MS, np.median(wall)
    assert np.median(cpu) <= CPU_P50_MS, np.median(cpu)
    assert wall.max() <= WALL_MAX_MS, wall.max()
