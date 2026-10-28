"""The segment solve in its OWN process — so a 60-206 ms QP never starves the
40 Hz emitter that shares ``trajectory_node``'s interpreter.

Pure Python, no ROS (``motion/`` rule).  Design: scratchpad ``w3_unitI_design.md``
(unit I, owner decision 2026-10-13: "move the planner into its own process so
the stream is never starved").  The failure class it closes: ``install_segment``
is numpy-dense Python that holds the GIL for most of its 40-200 ms, so the
emitter thread woke 45-57 ms late and the can-bridge latched a lane (sitting 7).
Measured on 600 real CATCH solves through a worker: emitter lateness max 1.69 ms.

**Contract**

* :class:`WorkerSegmentSolver` is the production solver: a long-lived child
  started with ``os.posix_spawn`` (NOT ``fork``: the parent is multithreaded;
  NOT ``multiprocessing`` spawn: it re-runs the colcon console-script wrapper,
  0.7 s vs 0.33 s) over an AF_UNIX socketpair, one request in flight.
  Every input is shipped per call, so the worker holds no state and there is no
  staleness class (``set_limits`` / ``pre_release_hold_s`` reach the next solve
  by construction).  ``CLOCK_MONOTONIC`` is system-wide, so ``t_now_s`` and
  ``t0_s`` cross the boundary unchanged.
* :meth:`~WorkerSegmentSolver.solve` NEVER raises.  A dead, hung, starting or
  wrong-tree worker answers :data:`SOLVER_UNAVAILABLE`; a request whose
  deadline passed answers :data:`SKIPPED_LATE`.  There is **no in-process
  fallback, ever**: it would put the solve back on the emitter's interpreter
  exactly when the system is degraded, silently.
* The worker evaluates ``t_install_s`` as ``perf_counter() +``
  :data:`REPLY_TRANSIT_ALLOW_S` (so its own in-solve lateness check runs
  3 ms EARLY) and reports it as ``reply.t_inst_s``, which is informational:
  the node applies its own splice fence at its own clock after the reply.
* Hang = an outstanding request unanswered :data:`SOLVER_HANG_S` after its send:
  SIGKILL, reap, ONE restart per episode (an episode ends on any solve reply),
  then DOWN ("worker failed twice; relaunch trajectory_node").

:class:`InProcessSegmentSolver` is a TEST DOUBLE ONLY.
"""

from __future__ import annotations

import argparse
import dataclasses
import errno
import math
import os
import pickle
import signal
import socket
import sys
import threading
import time
from multiprocessing.connection import Connection
from typing import Any, Callable, Iterable, List, NamedTuple, Optional, Tuple

#: Refusal code: the worker is dead, starting, hung, on the wrong tree, or down
#: after its restart was spent.  Not aim-dependent: the executor must not
#: re-dispatch a learned-prior fallback on it.
SOLVER_UNAVAILABLE = 'SOLVER_UNAVAILABLE'
#: Refusal code: the request's deadline had passed (at dequeue in the worker, or
#: the client gave up waiting).  The node maps it onto the matching timing code
#: (``ORIGIN_TOO_LATE`` / ``SPLICE_TOO_LATE``) by its own ``fresh`` predicate; a
#: REST that misses its deadline is reported as :data:`SOLVER_UNAVAILABLE`.
SKIPPED_LATE = 'SKIPPED_LATE'
#: How much EARLIER the worker's in-solve lateness check runs than the clock
#: (measured return path 0.8-1.0 ms + 0.5 ms unpickle, design § 4).  The node's
#: own fence is separate.
REPLY_TRANSIT_ALLOW_S = 0.003
#: A REST carries no absolute event; the worst loaded REST on record is 381 ms.
SOLVER_REST_DEADLINE_S = 1.0
SOLVER_HANG_S = 2.0
#: A worker alive but silent this long after spawn never reaches READY.
READY_TIMEOUT_S = 10.0

#: Default SCHED_FIFO priority the node asks for (JB-95): below every kernel IRQ
#: thread (FIFO 50) and migration (99).  0 = leave the worker on CFS.
WORKER_RT_PRIORITY = 10
#: CPUs the worker is pinned to once FIFO succeeded: CPU0 carries both NIC IRQs
#: and a FIFO task there would starve ksoftirqd/0 (UDP RX for the can-bridge
#: link and mocap).
WORKER_RT_CPUS = (1, 2, 3, 4, 5)

STARTING = 'STARTING'
READY = 'READY'
DOWN = 'DOWN'

_WORKER_FD = 3
_MODULE = 'jugglebot.motion.skills.solver_worker'


@dataclasses.dataclass(frozen=True)
class SegmentSolveRequest:
    """One ``executor.install_segment`` call, with its inputs by value."""

    seq: int
    record: Any            # Optional[executor.PlanRecord]
    seed_rest: Any         # Optional[unified_cycle.CycleState]
    kind: str
    terminal: Any
    t_now_s: float         # perf clock (CLOCK_MONOTONIC), grid-snapped by the node
    lead_s: float
    cfg: Any               # Optional[SegmentConfig]
    limits: Any
    geom: Any
    warm_start: Any        # Optional[SolverState]
    reserve_fresh_lead: bool
    deadline_s: float      # ABSOLUTE perf time after which a result is useless


@dataclasses.dataclass(frozen=True)
class SegmentSolveReply:
    """What the node needs back; the 12 kB ``Segment`` is NOT shipped."""

    seq: int
    new_record: Any        # Optional[executor.PlanRecord]
    result: Any            # executor.InstallResult
    warm_start: Any        # Optional[SolverState] (seg.warm_start)
    release_t_s: Optional[float]   # seg.release_t_s
    solve_s: float
    #: The instant the executor's lateness check read, plus the transit
    #: allowance.  Informational: the node applies its own splice fence at its
    #: own clock.  ``inf`` for the in-process double (no transit).
    t_inst_s: float
    #: JB-76 starvation probe, measured over the same ``t_rx`` .. solve-end
    #: window as ``solve_s``: the worker's CPU seconds (``process_time``) and
    #: its voluntary / involuntary context switches (``getrusage``). Both are
    #: process-wide, so an un-capped BLAS pool's threads count too (the
    #: launch caps it to 1 thread).
    #: ``cpu_s`` near ``solve_s`` = the solve itself is slow; ``cpu_s`` well
    #: under it with many involuntary switches = the box preempted it.
    #: ``nan`` / ``-1`` when not measured (refusals, the in-process double).
    cpu_s: float = math.nan
    vcsw: int = -1
    ivcsw: int = -1


def _refusal(req_seq: int, code: str, message: str, solve_s: float = 0.0,
             t_inst_s: float = math.inf) -> SegmentSolveReply:
    from jugglebot.motion.skills import executor as ex
    res = ex.InstallResult(accepted=False, code=code, message=message,
                           plan_wall_s=0.0)
    return SegmentSolveReply(seq=req_seq, new_record=None, result=res,
                             warm_start=None, release_t_s=None,
                             solve_s=solve_s, t_inst_s=t_inst_s)


def _solve_now(req: SegmentSolveRequest, t_install_s) -> Tuple[Any, Any, Any]:
    from jugglebot.motion.skills import executor as ex
    return ex.install_segment(
        req.record, req.seed_rest, req.kind, req.terminal, req.t_now_s,
        lead_s=req.lead_s, cfg=req.cfg, limits=req.limits, geom=req.geom,
        warm_start=req.warm_start, t_install_s=t_install_s,
        reserve_fresh_lead=req.reserve_fresh_lead)


def _reply_from(req: SegmentSolveRequest, out, solve_s: float,
                t_inst_s: float, cpu_s: float = math.nan, vcsw: int = -1,
                ivcsw: int = -1) -> SegmentSolveReply:
    new_record, res, seg = out
    return SegmentSolveReply(
        seq=req.seq, new_record=new_record, result=res,
        warm_start=None if seg is None else seg.warm_start,
        release_t_s=None if seg is None else seg.release_t_s,
        solve_s=solve_s, t_inst_s=t_inst_s, cpu_s=cpu_s, vcsw=vcsw,
        ivcsw=ivcsw)


# ---------------------------------------------------------------------------
# Test double
# ---------------------------------------------------------------------------

class InProcessSegmentSolver:
    """TEST DOUBLE ONLY — solves in the CALLER'S interpreter.

    It REINSTATES THE STARVATION this module exists to remove (the solve holds
    the GIL and the 40 Hz emitter wakes up to 200 ms late).  Production uses
    :class:`WorkerSegmentSolver`; a test pins that.  It exists so the mocked
    ``tests/ros`` suite keeps its meaning: it looks up
    ``executor.install_segment`` and ``time.perf_counter`` AT CALL TIME and
    passes ``t_install_s=time.perf_counter`` with NO transit allowance, so
    existing monkeypatches of either behave exactly as before and the path is
    bit-identical to the pre-worker node.
    """

    state = READY
    pid = None

    def solve(self, req: SegmentSolveRequest) -> SegmentSolveReply:
        t0 = time.perf_counter()
        out = _solve_now(req, time.perf_counter)
        return _reply_from(req, out, time.perf_counter() - t0, math.inf)

    def poll_health(self) -> None:
        return None

    def close(self, timeout_s: float = 1.0) -> None:
        return None


# ---------------------------------------------------------------------------
# Real-time policy (JB-95)
# ---------------------------------------------------------------------------

class SchedInfo(NamedTuple):
    """What the worker's scheduling class ended up as.  ``policy`` is ``'FIFO'``
    or ``'CFS'``; ``cpus`` is a range string (``'1-5'``) or ``'unchanged'``;
    ``reason`` is ``'ok'`` / ``'disabled'`` / why FIFO was refused."""

    policy: str
    priority: int
    cpus: str
    reason: str


def _read_text(path: str) -> str:
    with open(path) as f:
        return f.read()


def _fmt_cpus(cpus: Iterable[int]) -> str:
    xs = sorted(set(int(c) for c in cpus))
    out: List[str] = []
    i = 0
    while i < len(xs):
        j = i
        while j + 1 < len(xs) and xs[j + 1] == xs[j] + 1:
            j += 1
        out.append(str(xs[i]) if i == j else '%d-%d' % (xs[i], xs[j]))
        i = j + 1
    return ','.join(out)


def _cgroup_rt_reason(read_text: Callable[[str], str]) -> str:
    """Why EPERM when RLIMIT_RTPRIO is high enough: on a CONFIG_RT_GROUP_SCHED
    kernel a task in a cpu cgroup with ``cpu.rt_runtime_us == 0`` cannot enter
    an RT class (kernel source; the live cgroup value is 0 for /user.slice)."""
    try:
        for line in read_text('/proc/self/cgroup').splitlines():
            _id, ctrls, path = line.split(':', 2)
            if 'cpu' in ctrls.split(','):
                fn = ('/sys/fs/cgroup/cpu,cpuacct%s/cpu.rt_runtime_us'
                      % path.rstrip('/'))
                val = int(read_text(fn).strip())
                return 'EPERM: cpu cgroup %s rt_runtime_us=%d' % (path, val)
    except (OSError, ValueError):
        pass
    return 'EPERM (reason unknown)'


def apply_rt_policy(prio: int, cpus: Iterable[int] = WORKER_RT_CPUS, *,
                    setscheduler: Optional[Callable] = None,
                    setaffinity: Optional[Callable] = None,
                    getaffinity: Optional[Callable] = None,
                    getrlimit: Optional[Callable] = None,
                    read_text: Callable[[str], str] = _read_text
                    ) -> SchedInfo:
    """Put the CALLING THREAD on SCHED_FIFO ``prio``, then pin it to ``cpus``.

    INVARIANT (JB-95): fail-open and worker-only.  Any failure leaves the
    process on CFS exactly as before, with the reason in the result; the
    affinity is touched only AFTER FIFO succeeded, and if the pin then fails
    the policy is dropped back to SCHED_OTHER: a FIFO worker with an
    unrestricted mask could sit on CPU0, where both NIC IRQs land, which is
    the one placement the pin exists to prevent.  Call it from the worker
    PROCESS's ``_main`` only: ``sched_setscheduler(0, ...)`` acts on the calling
    thread, so inside ``worker_main`` a pytest thread would be made FIFO.
    The rlimit-EPERM branch is probed on the Jetson (``ulimit -r 0``); the
    cgroup branch is injection-tested only.  ``prio <= 0`` calls nothing.
    The ``os.sched_*`` defaults are resolved here, not in the signature, so
    the module still imports on a box without them (the Windows sim box).
    """
    if prio <= 0:
        return SchedInfo('CFS', 0, 'unchanged', 'disabled')
    if setscheduler is None:
        setscheduler = os.sched_setscheduler
    if setaffinity is None:
        setaffinity = os.sched_setaffinity
    if getaffinity is None:
        getaffinity = os.sched_getaffinity
    if getrlimit is None:
        import resource
        getrlimit = lambda: resource.getrlimit(resource.RLIMIT_RTPRIO)  # noqa: E731
    try:
        setscheduler(0, os.SCHED_FIFO | getattr(os, 'SCHED_RESET_ON_FORK', 0),
                     os.sched_param(prio))
    except OSError as e:
        if e.errno == errno.EPERM:
            try:
                soft = getrlimit()[0]
            except Exception:       # noqa: BLE001
                soft = None
            if soft is not None and soft < prio:
                why = 'EPERM: RLIMIT_RTPRIO=%d < %d' % (soft, prio)
            else:
                why = _cgroup_rt_reason(read_text)
        else:
            why = '%s: %s' % (errno.errorcode.get(e.errno, e.errno), e)
        return SchedInfo('CFS', 0, 'unchanged', why)
    want = set(int(c) for c in cpus)
    try:
        setaffinity(0, want)
        return SchedInfo('FIFO', prio, _fmt_cpus(want), 'ok')
    except OSError as e:
        # Fail-open: back to CFS rather than FIFO-anywhere. The RESET_ON_FORK
        # bit must be KEPT: an unprivileged task may not clear it, so a plain
        # SCHED_OTHER here is refused EPERM (seen on the Jetson 2026-10-28,
        # first run after the RT recipe) and would leave the worker FIFO on
        # any core.
        try:
            setscheduler(0, os.SCHED_OTHER | getattr(os, 'SCHED_RESET_ON_FORK', 0),
                         os.sched_param(0))
        except OSError:
            pass
        return SchedInfo('CFS', 0, 'unchanged', 'affinity failed: %s' % e)


# ---------------------------------------------------------------------------
# The worker process
# ---------------------------------------------------------------------------

def worker_main(fd: int, sched: Optional[tuple] = None) -> int:
    """Serve requests on socket ``fd`` until STOP or EOF; returns the exit code.

    ``sched`` is the :class:`SchedInfo` ``_main`` obtained (reported in READY).
    The policy is never applied here (see :func:`apply_rt_policy`)."""
    if sched is None:
        sched = tuple(SchedInfo('CFS', 0, 'unchanged', 'disabled'))
    try:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # Ctrl-C hits the group
    except ValueError:      # not the main thread (in-process test harness)
        pass
    conn = Connection(fd)
    try:
        import resource     # Unix-only; the worker only ever runs on Linux
        from jugglebot.motion.skills import executor as ex
        from jugglebot.motion import blas_threads
        blas = blas_threads.read_blas_threads()[0]
        conn.send_bytes(pickle.dumps(('READY', os.getpid(), ex.__file__, blas,
                                      tuple(sched)),
                                     protocol=pickle.HIGHEST_PROTOCOL))
        while True:
            try:
                msg = pickle.loads(conn.recv_bytes())
            except (EOFError, OSError):
                return 0
            if msg[0] == 'STOP':
                return 0
            req = msg[1]
            t_rx = time.perf_counter()
            cpu0 = time.process_time()
            ru0 = resource.getrusage(resource.RUSAGE_SELF)
            if t_rx > req.deadline_s:
                rep = _refusal(req.seq, SKIPPED_LATE,
                               'deadline passed %.1f ms before the worker '
                               'dequeued the request; not solved'
                               % ((t_rx - req.deadline_s) * 1e3))
            else:
                seen: List[float] = []

                def t_install_s() -> float:
                    seen.append(time.perf_counter() + REPLY_TRANSIT_ALLOW_S)
                    return seen[-1]

                try:
                    out = _solve_now(req, t_install_s)
                    solve_s = time.perf_counter() - t_rx
                    cpu_s = time.process_time() - cpu0
                    ru1 = resource.getrusage(resource.RUSAGE_SELF)
                    t_inst = seen[-1] if seen else (
                        time.perf_counter() + REPLY_TRANSIT_ALLOW_S)
                    rep = _reply_from(req, out, solve_s, t_inst, cpu_s,
                                      ru1.ru_nvcsw - ru0.ru_nvcsw,
                                      ru1.ru_nivcsw - ru0.ru_nivcsw)
                except Exception as exc:    # noqa: BLE001 - reply, don't die
                    import traceback
                    traceback.print_exc()
                    rep = _refusal(req.seq, SOLVER_UNAVAILABLE,
                                   'worker solve raised %s: %s'
                                   % (type(exc).__name__, exc))
            try:
                conn.send_bytes(pickle.dumps(rep,
                                             protocol=pickle.HIGHEST_PROTOCOL))
            except (BrokenPipeError, OSError):
                return 0
    finally:
        try:
            conn.close()
        except OSError:
            pass


# ---------------------------------------------------------------------------
# The client
# ---------------------------------------------------------------------------

class WorkerSegmentSolver:
    """Production solver: a long-lived worker process (see the module docstring).

    ``log(level, msg)`` takes ``'info' | 'warning' | 'error'``; ``None`` is
    silent.  ``expected_executor_file`` is the executor module the PARENT
    imported; a worker reporting another file (wrong tree on ``PYTHONPATH``) is
    marked DOWN with one error log.
    """

    def __init__(self, *, python: str = sys.executable,
                 sys_path: Optional[List[str]] = None,
                 env: Optional[dict] = None,
                 log: Optional[Callable[[str, str], None]] = None,
                 expected_executor_file: Optional[str] = None,
                 rt_priority: int = 0):
        from jugglebot.motion.skills import executor as ex
        self._python = python
        self._sys_path = list(sys.path if sys_path is None else sys_path)
        self._env_override = env
        self._log = log
        self._rt_priority = int(rt_priority)
        self._sched_info: Optional[SchedInfo] = None
        self._expected = os.path.realpath(
            ex.__file__ if expected_executor_file is None
            else expected_executor_file)
        self._lock = threading.Lock()
        self._state = STARTING
        self._pid: Optional[int] = None
        self._conn: Optional[Connection] = None
        self._inflight = {}                 # seq -> perf time of the send
        self._restart_used = False
        self._ready_info: Optional[tuple] = None
        self._down_reason = ''
        self._spawn()

    # -- properties ---------------------------------------------------------
    @property
    def state(self) -> str:
        return self._state

    @property
    def pid(self) -> Optional[int]:
        return self._pid

    @property
    def ready_info(self) -> Optional[tuple]:
        """``(pid, executor.__file__, blas_threads)`` from the last READY."""
        return self._ready_info

    @property
    def sched_info(self) -> Optional[SchedInfo]:
        """The worker's :class:`SchedInfo` from the last READY (None before)."""
        return self._sched_info

    # -- logging ------------------------------------------------------------
    def _emit(self, level: str, msg: str) -> None:
        if self._log is not None:
            try:
                self._log(level, msg)
            except Exception:       # noqa: BLE001 - logging never breaks a solve
                pass

    # -- process management ---------------------------------------------------
    def _spawn(self) -> None:
        parent, child = socket.socketpair()
        child_fd = child.fileno()
        if child_fd == _WORKER_FD:      # dup2 onto itself would keep CLOEXEC
            child_fd = os.dup(child_fd)
        env = dict(os.environ if self._env_override is None
                   else self._env_override)
        env['PYTHONPATH'] = os.pathsep.join(p for p in self._sys_path if p)
        argv = [self._python, '-m', _MODULE, '--fd', str(_WORKER_FD)]
        if self._rt_priority > 0:
            argv += ['--rt-prio', str(self._rt_priority)]
        try:
            pid = os.posix_spawn(
                self._python, argv, env,
                file_actions=[(os.POSIX_SPAWN_DUP2, child_fd, _WORKER_FD)],
                setsigmask=())
        finally:
            child.close()
            if child_fd != _WORKER_FD and child_fd != -1:
                try:
                    os.close(child_fd)
                except OSError:
                    pass
        self._conn = Connection(parent.detach())
        self._pid = pid
        self._spawned_at = time.perf_counter()
        self._state = STARTING
        self._inflight = {}
        self._ready_info = None
        self._sched_info = None

    def _reap(self, kill: bool) -> None:
        pid = self._pid
        if pid is not None:
            if kill:
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            try:
                os.waitpid(pid, 0 if kill else os.WNOHANG)
            except ChildProcessError:
                pass
            self._pid = None
        if self._conn is not None:
            try:
                self._conn.close()
            except OSError:
                pass
            self._conn = None

    def _fail(self, reason: str) -> None:
        """The worker died or hung: kill, reap, restart once per episode."""
        self._reap(kill=True)
        if not self._restart_used:
            self._restart_used = True
            self._spawn()
            self._emit('warning', 'segment solver worker restarted (%s)' % reason)
        else:
            self._state = DOWN
            self._down_reason = ('worker failed twice; relaunch '
                                 'trajectory_node (%s)' % reason)
            self._emit('error', 'segment solver ' + self._down_reason)

    def _dead(self) -> bool:
        if self._pid is None:
            return True
        try:
            pid, _status = os.waitpid(self._pid, os.WNOHANG)
        except ChildProcessError:
            self._pid = None
            return True
        if pid == 0:
            return False
        self._pid = None
        return True

    def _check_alive(self) -> bool:
        """False (and the failure handled) when the worker is dead/hung."""
        if self._state == DOWN:
            return False
        if self._dead():
            self._fail('worker process exited')
            return False
        now = time.perf_counter()
        if (self._state == STARTING
                and now - self._spawned_at > READY_TIMEOUT_S):
            self._pump_ready(0)         # READY may be sitting unread
            if self._state == STARTING:
                self._fail('worker did not report READY in %.0f s'
                           % READY_TIMEOUT_S)
                return False
            return self._state != DOWN
        if any(now - t > SOLVER_HANG_S for t in self._inflight.values()):
            self._fail('no reply within %.1f s' % SOLVER_HANG_S)
            return False
        return True

    def _pump_ready(self, timeout: float) -> None:
        """Consume the READY handshake if it has arrived."""
        if self._state != STARTING or self._conn is None:
            return
        try:
            if not self._conn.poll(timeout):
                return
            msg = pickle.loads(self._conn.recv_bytes())
        except (EOFError, OSError):
            self._fail('worker closed its socket before READY')
            return
        if not (isinstance(msg, tuple) and msg and msg[0] == 'READY'):
            self._fail('unexpected first message from worker')
            return
        _tag, pid, exfile, blas = msg[:4]
        try:
            sched = SchedInfo(*msg[4]) if len(msg) > 4 else SchedInfo(
                'CFS', 0, 'unchanged', 'unreported')
        except TypeError:
            sched = SchedInfo('CFS', 0, 'unchanged', 'unreported')
        self._ready_info = (pid, exfile, blas)
        self._sched_info = sched
        if os.path.realpath(exfile) != self._expected:
            self._reap(kill=True)
            self._state = DOWN
            self._down_reason = ('worker imported a different executor (%s, '
                                 'expected %s)' % (exfile, self._expected))
            self._emit('error', 'segment solver ' + self._down_reason)
            return
        self._state = READY
        if blas != 1:
            self._emit('warning', 'segment solver worker BLAS threads = %r '
                       '(expected 1)' % (blas,))
        if sched.policy == 'FIFO':
            sched_txt = 'FIFO/%d cpus=%s' % (sched.priority, sched.cpus)
        else:
            sched_txt = 'CFS (%s)' % sched.reason
        self._emit('info', 'segment solver worker READY pid=%d executor=%s '
                   'blas_threads=%r sched=%s' % (pid, exfile, blas, sched_txt))
        if self._rt_priority > 0 and sched.policy != 'FIFO':
            self._emit('warning', 'segment solver worker stayed on CFS '
                       '(requested FIFO %d): %s'
                       % (self._rt_priority, sched.reason))

    # -- public ---------------------------------------------------------------
    def wait_ready(self, timeout_s: float = 10.0) -> bool:
        """Block until READY (True) or DOWN/timeout (False).  Tests/startup."""
        end = time.perf_counter() + timeout_s
        while time.perf_counter() < end:
            with self._lock:
                if self._state == READY:
                    return True
                if self._state == DOWN:
                    return False
                if self._check_alive():
                    self._pump_ready(0.02)
            time.sleep(0.005)
        return self._state == READY

    def poll_health(self) -> None:
        """Non-blocking: reap a dead worker, kill a hung one, take READY."""
        if not self._lock.acquire(blocking=False):
            return                      # a solve is running; it checks itself
        try:
            if self._check_alive():
                self._pump_ready(0)
        finally:
            self._lock.release()

    def solve(self, req: SegmentSolveRequest) -> SegmentSolveReply:
        """Solve ``req`` in the worker.  NEVER raises; refusals are in ``result``."""
        try:
            return self._solve(req)
        except Exception as exc:        # noqa: BLE001 - the contract
            return self._unavailable(req, 'solver client error %s: %s'
                                     % (type(exc).__name__, exc))

    def _unavailable(self, req, message: str) -> SegmentSolveReply:
        return _refusal(req.seq, SOLVER_UNAVAILABLE, message)

    def _solve(self, req: SegmentSolveRequest) -> SegmentSolveReply:
        # A status-timer poll_health holds the lock for microseconds; wait it out.
        if not self._lock.acquire(timeout=0.005):
            return self._unavailable(req, 'solver busy (one request in flight)')
        try:
            if self._state == DOWN:
                return self._unavailable(req, self._down_reason)
            if not self._check_alive():
                return self._unavailable(
                    req, self._down_reason or 'worker died; restarting')
            self._pump_ready(0)
            if self._state == DOWN:
                return self._unavailable(req, self._down_reason)
            if self._state != READY:
                return self._unavailable(req, 'worker starting')
            if time.perf_counter() > req.deadline_s:
                return self._late(req, 'deadline already passed at send')
            try:
                self._drain_stale(req.seq)
                self._conn.send_bytes(pickle.dumps(
                    ('SOLVE', req), protocol=pickle.HIGHEST_PROTOCOL))
            except (EOFError, OSError):
                self._fail('send to worker failed')
                return self._unavailable(req, 'worker connection lost')
            self._inflight[req.seq] = time.perf_counter()
            return self._await(req)
        finally:
            self._lock.release()

    def _late(self, req, why: str) -> SegmentSolveReply:
        if req.kind == 'REST':
            return self._unavailable(req, 'solver did not answer a REST by its '
                                     'deadline (%s)' % why)
        return _refusal(req.seq, SKIPPED_LATE,
                        'solver did not answer by the wire deadline (%s)' % why)

    def _drain_stale(self, current_seq: int) -> None:
        while self._conn.poll(0):
            rep = pickle.loads(self._conn.recv_bytes())
            self._on_reply(rep)

    def _on_reply(self, rep) -> Optional[SegmentSolveReply]:
        if not isinstance(rep, SegmentSolveReply):
            return None
        self._inflight.pop(rep.seq, None)
        self._restart_used = False          # an answer ends the episode
        return rep

    def _await(self, req: SegmentSolveRequest) -> SegmentSolveReply:
        while True:
            remaining = req.deadline_s - time.perf_counter()
            if remaining <= 0:
                return self._late(req, 'no reply in time')
            try:
                if not self._conn.poll(min(remaining, 0.05)):
                    if self._dead():
                        self._fail('worker process exited mid-solve')
                        return self._unavailable(req, 'worker died mid-solve')
                    continue
                rep = self._on_reply(pickle.loads(self._conn.recv_bytes()))
            except (EOFError, OSError):
                self._fail('worker closed its socket mid-solve')
                return self._unavailable(req, 'worker died mid-solve')
            if rep is not None and rep.seq == req.seq:
                if (req.kind == 'REST' and not rep.result.accepted
                        and rep.result.code == SKIPPED_LATE):
                    return self._late(req, 'worker skipped it')
                return rep
            # a stale reply (an earlier, abandoned seq): dropped by seq

    def close(self, timeout_s: float = 1.0) -> None:
        """STOP, join, SIGTERM, join, SIGKILL (half of ``timeout_s`` per stage)."""
        with self._lock:
            pid = self._pid
            if pid is not None and self._conn is not None:
                try:
                    self._conn.send_bytes(pickle.dumps(
                        ('STOP',), protocol=pickle.HIGHEST_PROTOCOL))
                except (EOFError, OSError):
                    pass
                stage = max(0.0, float(timeout_s)) / 2.0
                if not self._join(pid, stage):
                    try:
                        os.kill(pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    if not self._join(pid, stage):
                        try:
                            os.kill(pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                        self._join(pid, 1.0)
                self._pid = None
            if self._conn is not None:
                try:
                    self._conn.close()
                except OSError:
                    pass
                self._conn = None
            self._state = DOWN
            self._down_reason = 'solver closed'

    @staticmethod
    def _join(pid: int, timeout_s: float) -> bool:
        end = time.perf_counter() + timeout_s
        while True:
            try:
                done, _s = os.waitpid(pid, os.WNOHANG)
            except ChildProcessError:
                return True
            if done:
                return True
            if time.perf_counter() >= end:
                return False
            time.sleep(0.005)


def _main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--fd', type=int, required=True,
                    help='inherited socket carrying the request stream')
    ap.add_argument('--rt-prio', type=int, default=0,
                    help='try SCHED_FIFO at this priority (fail-open; 0 = off)')
    args = ap.parse_args(argv)
    # Applied HERE, before worker_main: sched_setscheduler(0) acts on the
    # calling thread, so it must never run inside worker_main (pytest thread).
    sched = apply_rt_policy(args.rt_prio)
    # Serve from the CANONICAL module, not ``__main__``: the reply dataclasses
    # pickle by module path, and the parent must be able to unpickle them
    # (found by the round-trip test: "Can't get attribute ... on __main__").
    from jugglebot.motion.skills import solver_worker as canonical
    return canonical.worker_main(args.fd, sched=tuple(sched))


if __name__ == '__main__':
    sys.exit(_main())
