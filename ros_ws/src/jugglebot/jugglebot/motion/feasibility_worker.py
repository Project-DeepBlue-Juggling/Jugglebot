"""Feasibility worker process for the async trajectory pipeline.

Runs ``check_feasibility()`` and ``make_deceleration_trajectory()`` in a
dedicated child process, bypassing the GIL so the 500 Hz control loop is
never blocked by CPU-bound numpy work.

Communication uses ``multiprocessing.Pipe`` (pickle-based, ~20 μs latency).
The parent side is wrapped by :class:`FeasibilityWorkerProxy`, which
presents the same non-blocking submit/poll interface that
:class:`TrajectoryManager` previously used with a background thread.
"""

from __future__ import annotations

import logging
import time as _time
from multiprocessing import Process
from multiprocessing.connection import Connection

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Custom exception
# ---------------------------------------------------------------------------

class WorkerCrashed(Exception):
    """Raised when the feasibility worker process dies unexpectedly."""


# ---------------------------------------------------------------------------
# Worker process entry point
# ---------------------------------------------------------------------------

def _worker_main(pipe: Connection, geom, dynamics_params) -> None:
    """Entry point for the feasibility worker process.

    Blocks on *pipe* waiting for request dicts.  Each request has a
    ``type`` field that selects the operation:

    - ``'feasibility'``: run ``check_feasibility()`` on the provided
      trajectory and send back the result.
    - ``'decel'``: create a deceleration trajectory via
      ``make_deceleration_trajectory()``, run a feasibility check, and
      send back the trajectory (or ``None`` on failure).
    - ``None``: shutdown sentinel — exit cleanly.
    """
    # Lazy imports so the parent process doesn't pay for them at fork time
    # (forkserver re-imports anyway).
    from jugglebot.motion.trajectory import (
        check_feasibility,
        make_deceleration_trajectory,
    )

    _log = logging.getLogger(f'{__name__}.worker')
    _log.debug('Feasibility worker process started (pid=%d)', __import__('os').getpid())

    while True:
        try:
            request = pipe.recv()
        except (EOFError, OSError):
            # Parent closed the pipe — exit
            break

        if request is None:
            # Shutdown sentinel
            break

        req_type = request.get('type')

        if req_type == 'feasibility':
            _handle_feasibility(pipe, request, geom, dynamics_params,
                                check_feasibility)

        elif req_type == 'decel':
            _handle_deceleration(pipe, request, geom, dynamics_params,
                                 check_feasibility,
                                 make_deceleration_trajectory)

        else:
            _log.warning('Unknown request type: %s', req_type)

    pipe.close()
    _log.debug('Feasibility worker process exiting')


def _handle_feasibility(pipe, request, geom, dynamics_params,
                        check_feasibility) -> None:
    """Run a feasibility check and send the result back."""
    traj = request['traj']
    gen = request['generation']

    t_before = _time.perf_counter()
    result = check_feasibility(
        traj, geom, dynamics_params,
        n_samples=50, early_exit=True, yield_interval=0)
    check_elapsed = _time.perf_counter() - t_before

    pipe.send({
        'type': 'feasibility',
        'accepted': result.feasible,
        'traj': traj,
        'needs_decel': request['needs_decel'],
        'arrival_time': request['arrival_time'],
        'generation': gen,
        't_request': request['t_request'],
        'check_elapsed': check_elapsed,
        'violations': result.violations if not result.feasible else [],
    })


def _handle_deceleration(pipe, request, geom, dynamics_params,
                         check_feasibility,
                         make_deceleration_trajectory) -> None:
    """Create a deceleration trajectory and verify feasibility."""
    gen = request['generation']
    start_pose = request['start_pose']
    start_twist = request['start_twist']
    start_accel = request['start_accel']

    _log = logging.getLogger(f'{__name__}.worker')

    # Create deceleration trajectory
    traj = make_deceleration_trajectory(
        start_pose=start_pose,
        start_twist=start_twist,
        start_accel=start_accel,
        geom=geom,
    )

    if traj is None:
        _log.warning('Deceleration precompute: velocity too low, skipping')
        pipe.send({'type': 'decel', 'generation': gen, 'traj': None})
        return

    # Single feasibility check (no yield_interval — own process, own GIL)
    result = check_feasibility(
        traj, geom, dynamics_params,
        n_samples=50, early_exit=True, yield_interval=0)

    if not result.feasible:
        # Retry with 2x duration (gentler deceleration)
        _log.debug(
            'Deceleration at %.3fs infeasible, retrying at %.3fs',
            traj.duration, traj.duration * 2)
        traj = make_deceleration_trajectory(
            start_pose=start_pose,
            start_twist=start_twist,
            start_accel=start_accel,
            geom=geom,
            duration_margin=3.0,  # 1.5 base × 2
        )
        if traj is None:
            pipe.send({'type': 'decel', 'generation': gen, 'traj': None})
            return
        result = check_feasibility(
            traj, geom, dynamics_params,
            n_samples=50, early_exit=True, yield_interval=0)
        if not result.feasible:
            _log.warning(
                'Deceleration infeasible even at 2x duration: %s',
                '; '.join(result.violations))
            pipe.send({'type': 'decel', 'generation': gen, 'traj': None})
            return

    _log.info('Deceleration pre-computed: duration=%.3fs', traj.duration)
    pipe.send({'type': 'decel', 'generation': gen, 'traj': traj})


# ---------------------------------------------------------------------------
# Main-thread proxy
# ---------------------------------------------------------------------------

class FeasibilityWorkerProxy:
    """Main-thread facade for the feasibility worker process.

    Manages the child process lifecycle and provides non-blocking
    submit/poll methods that mirror the old threading-based interface.

    Parameters
    ----------
    geom : StewartGeometry
        Platform geometry (pickled to the child process once at startup).
    dynamics_params : DynamicsParams
        Dynamics parameters (pickled to the child process once at startup).
    max_restarts : int
        Maximum number of restart attempts before giving up.
    """

    def __init__(self, geom, dynamics_params, max_restarts: int = 3):
        self._geom = geom
        self._dynamics_params = dynamics_params
        self._max_restarts = max_restarts
        self._restart_count = 0
        self._conn: Connection | None = None
        self._proc: Process | None = None
        self._alive = False
        self._start_worker()

    def _start_worker(self) -> None:
        from multiprocessing import Pipe
        parent_conn, child_conn = Pipe()
        self._conn = parent_conn
        self._proc = Process(
            target=_worker_main,
            args=(child_conn, self._geom, self._dynamics_params),
            daemon=True,
            name='feas-worker',
        )
        self._proc.start()
        child_conn.close()  # parent doesn't use child end
        self._alive = True
        logger.info('Feasibility worker process started (pid=%d)',
                     self._proc.pid)

    def submit(self, request: dict) -> bool:
        """Send a request to the worker.  Non-blocking.

        Returns False if the worker is dead (caller should handle).
        """
        if not self._alive:
            return False
        try:
            self._conn.send(request)
            return True
        except (BrokenPipeError, OSError):
            self._alive = False
            return False

    def poll(self) -> dict | None:
        """Non-blocking check for a completed result.

        Returns
        -------
        dict or None
            The result dict if ready, else ``None``.

        Raises
        ------
        WorkerCrashed
            If the worker process has died.
        """
        if not self._alive:
            return None
        try:
            if self._conn.poll():
                return self._conn.recv()
        except (BrokenPipeError, EOFError, OSError):
            self._alive = False
            raise WorkerCrashed('Feasibility worker process died')
        return None

    def try_restart(self) -> bool:
        """Attempt to restart the worker process.

        Returns True if the restart succeeded, False if the maximum
        restart count has been reached or the restart failed.
        """
        if self._restart_count >= self._max_restarts:
            logger.error(
                'Feasibility worker restart limit reached (%d/%d)',
                self._restart_count, self._max_restarts)
            return False
        self._cleanup()
        try:
            self._start_worker()
            self._restart_count += 1
            logger.warning(
                'Feasibility worker restarted (%d/%d)',
                self._restart_count, self._max_restarts)
            return True
        except Exception:
            logger.exception('Failed to restart feasibility worker')
            return False

    def shutdown(self) -> None:
        """Gracefully shut down the worker process."""
        if self._alive:
            try:
                self._conn.send(None)  # shutdown sentinel
            except (BrokenPipeError, OSError):
                pass
        self._cleanup()

    def _cleanup(self) -> None:
        if self._proc is not None and self._proc.is_alive():
            self._proc.join(timeout=2.0)
            if self._proc.is_alive():
                self._proc.kill()
                self._proc.join(timeout=1.0)
        if self._conn is not None:
            try:
                self._conn.close()
            except OSError:
                pass
        self._alive = False

    @property
    def is_alive(self) -> bool:
        """True if the worker process is running."""
        return self._alive
