"""Run the BB calibration solve OFF mocap_node's executor thread.

Pure Python (no ROS imports). ``mocap_node`` spins the default single-threaded
executor, so a callback that computes for seconds stops the 5 ms frame-drain
timer: the BB calibration solve (``run_calibration`` + ``estimate_base_pose``,
~3 s on one thread, 4.3-7.2 s in the node) overflowed the 150-frame queue
0.5 s in and dropped every QTM frame until it returned
(logbook 2026-10-16-mocap-frame-queue-starvation). This module is the one
place the solve may run.

INVARIANT (the contract; ``tests/ros/test_mocap_calibration_solve_worker.py``):
no executor callback of ``mocap_node`` runs the calibration solve. The
heartbeat callback SNAPSHOTS the sweep (``submit``, copies, O(frames) list
copies only) and returns; one worker thread runs ``solve_fn(job)`` on that
snapshot and nothing else -- it never touches node state, publishers, the
logger or files; the node's executor-thread timer ``drain()``s the outcomes
and applies them (state, file writes, publishes, log lines).

Rules:

1. ONE solve at a time, FIFO. A sweep that ends while an earlier solve is in
   flight queues behind it and never runs concurrently; each outcome is
   applied once, in submit order. A sweep that STARTS while a solve is in
   flight is unaffected: the snapshot owns copies of the earlier sweep's
   inputs, the new window collects into freshly bound lists, and the
   in-flight result still applies (it belongs to the earlier sweep).
2. Shutdown with a solve in flight: ``stop(timeout)`` joins with a bounded
   timeout and every pending and late outcome is dropped (the thread is a
   daemon, so a solve still running cannot hold the process up).
3. A solve that RAISES is caught here and delivered as an outcome with
   ``error`` set; the node logs it once (as a failed attempt) and changes no
   calibration state. The worker thread survives it and the node keeps
   streaming -- fail-closed to "no calibration update", never a dead node.
4. Apply latency is bounded by the node's apply-timer period
   (``mocap_node.CALIBRATION_APPLY_PERIOD_S``). It does not matter to the
   calibration: the BB is static between sweeps and every value the apply
   step uses comes from the snapshot or the outcome.

``inline=True`` (tests) runs ``solve_fn`` synchronously inside ``submit`` and
queues the outcome, so a test that patches the solver around the sweep's exit
edge and then calls the apply step sees the same behaviour as before the move.

Why a thread and not a process: the probe
(``tools/probes/mocap_starvation/solve_vs_drain.py``, 2026-10-16, loaded box)
measured a 5 ms drain loop beside the solve in a worker thread at a 37-74 ms
max tick gap, a queue peak of 11-22 of 150 and 0 drops; the solve slows from
3.1 to 3.7-4.0 s. A ``multiprocessing`` worker would remove that GIL
contention, but it pickles the ~2000-frame snapshot and the result across a
pipe, forks (or spawns) a process carrying the node's imports, and turns a
worker crash into a child-process lifecycle problem; the thread's worst gap is
inside the frame queue's 500 ms horizon by 7x and inside the ball tracker's
100 ms gap bridge (logbook 2026-10-16-tracker-flight-fit-late-since-matcher-fix),
and it only occurs in the ~4 s after a sweep, when no ball is in flight.
"""

from __future__ import annotations

import collections
import queue
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Deque, Optional


@dataclass
class SolveOutcome:
    """What the worker hands back: the job, ``solve_fn``'s return value
    (``value``) or the exception text (``error``), and the solve's wall time."""
    job: Any
    value: Any = None
    error: Optional[str] = None
    solve_s: float = 0.0


_STOP = object()


class CalibrationSolveWorker:
    """One daemon thread that runs ``solve_fn(job)`` per submitted job, FIFO,
    one at a time, and queues a ``SolveOutcome`` for the owner to ``drain``.

    ``submit`` and ``drain`` are called from the owner's (executor) thread and
    never block. The thread starts lazily on the first ``submit``.
    """

    def __init__(self, solve_fn: Callable[[Any], Any], *, inline: bool = False,
                 name: str = 'bb_calibration_solve'):
        self._solve_fn = solve_fn
        self._inline = inline
        self._name = name
        self._jobs: 'queue.Queue[Any]' = queue.Queue()
        self._outcomes: Deque[SolveOutcome] = collections.deque()
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._stopped = False
        self._busy = False

    @property
    def inline(self) -> bool:
        return self._inline

    def pending(self) -> int:
        """Jobs submitted whose outcome has not been drained yet."""
        with self._lock:
            return self._jobs.qsize() + int(self._busy) + len(self._outcomes)

    def submit(self, job: Any) -> bool:
        """Queue ``job`` (a snapshot the caller no longer mutates). False once
        stopped (the job is dropped)."""
        with self._lock:
            if self._stopped:
                return False
        if self._inline:
            outcome = self._run(job)
            with self._lock:
                self._outcomes.append(outcome)
            return True
        with self._lock:
            if self._thread is None:
                self._thread = threading.Thread(
                    target=self._loop, name=self._name, daemon=True)
                self._thread.start()
            self._jobs.put(job)
        return True

    def drain(self) -> list:
        """Every outcome ready now, oldest first (empty list if none)."""
        with self._lock:
            if self._stopped or not self._outcomes:
                return []
            out = list(self._outcomes)
            self._outcomes.clear()
            return out

    def stop(self, timeout: float = 1.0) -> bool:
        """Stop accepting jobs, join the thread for at most ``timeout`` s and
        drop every pending and late outcome. True if the thread is gone."""
        with self._lock:
            self._stopped = True
            self._outcomes.clear()
            thread = self._thread
        if thread is None:
            return True
        self._jobs.put(_STOP)
        thread.join(timeout)
        return not thread.is_alive()

    def _run(self, job: Any) -> SolveOutcome:
        t0 = time.monotonic()
        try:
            value = self._solve_fn(job)
        except Exception as e:      # rule 3: never let a solve kill the worker
            return SolveOutcome(job=job, error=f'{type(e).__name__}: {e}',
                                solve_s=time.monotonic() - t0)
        return SolveOutcome(job=job, value=value, solve_s=time.monotonic() - t0)

    def _loop(self) -> None:
        while True:
            job = self._jobs.get()
            if job is _STOP:
                return
            with self._lock:
                if self._stopped:
                    return
                self._busy = True
            outcome = self._run(job)
            with self._lock:
                self._busy = False
                if self._stopped:
                    return
                self._outcomes.append(outcome)
