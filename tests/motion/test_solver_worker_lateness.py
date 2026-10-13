"""The one contract ``solver_worker`` exists for: a segment solve in the worker
does NOT starve a 40 Hz absolute-grid thread in the caller's interpreter
(unit I1, 2026-10-13; design ``w3_unitI_design.md`` § 6c).

``serial`` and in the DEFAULT gate on purpose (it measures wall clock, and a
commit that puts the solve back on the emitter's interpreter must fail).

Method (after ``scratch/uE/gilprobe.py`` / ``uI/gilworker.py``): a thread keeps
the 25 ms grid ``nd += 25 ms`` and does a little numpy work each tick.
Lateness is ``end_of_tick_work - nd`` (NOT wake - nd: GIL blocking after the
wake is the failure, and a wake-time metric under-reads it); the first two
ticks are dropped.

* Control: 20 synthetic ~150 ms numpy-dense solves IN-PROCESS.  Asserted
  ``max lateness > 50 ms`` so the test can SEE starvation at all (probe:
  149-277 ms).
* Subject: >= 40 real CATCH solves through :class:`WorkerSegmentSolver`.
  Bars: p99 < 5 ms, max < 10 ms (design probe, 600 solves: p99 <= 1.36,
  max <= 1.69 ms).

Probed 2026-10-13, before the bars were fixed, 40 real CATCH solves per run
(worker launched with the launch's BLAS pin, see the test body):

* idle (load 1-2), 4 runs:   p99 0.31-0.75 ms, max 0.32-0.92 ms.
* loaded (load 2.5-8.7), 3 runs, with ``pytest -n 4 tests/motion/
  test_skills_admissible.py tests/motion/test_learner.py`` running beside it:
  p99 0.24-0.73 ms, max 0.27-1.26 ms.
* control (150 ms GIL-holding call, in-process): max 128-149 ms idle,
  174-196 ms loaded.

Without the BLAS pin the worker's multithreaded BLAS competes for the
emitter's core and ticks reach 4-7 ms (p99 up to 3.7 ms); GC was ruled out
(``gc.callbacks``: zero collections during the runs).
"""

from __future__ import annotations

import os
import sys
import threading
import time

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import sites as si
from jugglebot.motion.skills import solver_worker as sw
from jugglebot.motion.trajectory.limits import TrajectoryLimits

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_solver_worker as tsw   # noqa: E402  request builders

PERIOD = 0.025
N_CATCH = 40
N_CONTROL = 20

pytestmark = pytest.mark.serial


def _calibrated_gil_hold(ms: float):
    """A callable that holds the GIL for ~``ms`` in ONE C call (``sum`` over a
    ``range`` never releases it or reaches the eval loop's switch check).

    The real QP starves the emitter the same way: small numpy calls under
    ``OPENBLAS_NUM_THREADS=1`` (the launch pins it) never release the GIL.  A
    numpy-based synthetic is only a control when that env is set before numpy
    is imported -- measured 2026-10-13: unpinned, 12x12 solves release the GIL
    in BLAS and the control reads 8-12 ms -- so this control does not depend
    on the environment.
    """
    n = 1_000_000
    for _ in range(8):
        t = time.perf_counter()
        sum(range(n))
        dt = time.perf_counter() - t
        n = max(1000, int(n * (ms / 1e3) / max(dt, 1e-6)))
    return lambda: sum(range(n))


def _measure(body) -> np.ndarray:
    """Run ``body()`` beside a 25 ms grid thread; end-of-work lateness in ms."""
    stop = threading.Event()
    ticks = []

    def emitter():
        nd = (time.perf_counter() // PERIOD + 1) * PERIOD
        while not stop.is_set():
            np.polyval(np.arange(6.0), 0.3)
            ticks.append(time.perf_counter() - nd)
            nd += PERIOD
            dt = nd - time.perf_counter()
            if dt > 0:
                stop.wait(dt)
            else:
                nd = (time.perf_counter() // PERIOD + 1) * PERIOD

    th = threading.Thread(target=emitter, daemon=True)
    th.start()
    time.sleep(0.2)
    body()
    time.sleep(0.1)
    stop.set()
    th.join()
    return np.array(ticks[2:]) * 1e3


def test_the_worker_keeps_the_40hz_grid_while_the_in_process_solve_starves_it():
    geom = StewartGeometry()
    limits = TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=300.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=200000.0,
        hand_acc_rps2=3500.0)
    sites = si.columns_sites(100.0)
    # The worker gets the launch's BLAS pin (jugglebot_launch._planner_blas_env).
    # Measured 2026-10-13: with it unset the worker's multithreaded BLAS takes
    # the emitter's core and ticks reach 4-7 ms late (R6's "unattributed" wakes;
    # GC was ruled out with gc.callbacks: zero collections in the run).
    pinned = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
                  MKL_NUM_THREADS='1', NUMEXPR_NUM_THREADS='1')
    solver = sw.WorkerSegmentSolver(env=pinned)
    try:
        assert solver.wait_ready(30.0)
        t0 = tsw._t0()
        first = solver.solve(tsw._throw_req(1, t0, limits, geom, sites))
        assert first.result.accepted, first.result.message
        # warm the worker's catch path so cold imports are not the measurement
        warm = solver.solve(tsw._catch_req(2, t0, first.new_record,
                                           first.warm_start, limits, geom, sites))
        assert warm.result.accepted, warm.result.message

        hold = _calibrated_gil_hold(150.0)
        control = _measure(lambda: [hold() for _ in range(N_CONTROL)])

        solves = []

        def via_worker():
            for i in range(N_CATCH):
                rep = solver.solve(tsw._catch_req(
                    10 + i, t0, first.new_record, first.warm_start, limits,
                    geom, sites))
                solves.append(rep)

        sub = _measure(via_worker)
    finally:
        solver.close()

    assert all(r.result.accepted for r in solves), \
        [r.result.message for r in solves if not r.result.accepted][:2]
    assert len(solves) >= 40
    print('control (in-process 150 ms x%d): max %.1f ms' % (N_CONTROL, control.max()))
    print('worker (%d real CATCH): p50 %.2f p99 %.2f max %.2f ms'
          % (N_CATCH, np.median(sub), np.percentile(sub, 99), sub.max()))
    assert control.max() > 50.0, 'control cannot see starvation: %.1f ms' % control.max()
    assert np.percentile(sub, 99) < 5.0, np.percentile(sub, 99)
    assert sub.max() < 10.0, sub.max()
