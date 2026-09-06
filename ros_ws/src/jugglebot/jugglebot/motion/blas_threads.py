"""blas_threads — read (and report) the BLAS thread pool a planner node inherited.

**Why this module exists (MEASURED 2026-09-06, 60+ reps, dose-response).**
The unified planner's ``plan_cycle`` is thousands of *small* numpy calls. Each
one fans out to OpenBLAS's default thread pool — **6 workers on this Jetson, one
per core** — and those workers **busy-spin** between calls waiting for the next
one. On an idle box that is free (default 194–223 ms vs 195–207 ms capped to one
thread: identical). The moment anything else wants a core it stops being free:
the spinners are descheduled, every one of those thousands of calls pays a
scheduler round trip, and the spinners **evict the node's own 40 Hz emitter
thread** — which is the thread feeding the can-bridge's 250 ms setpoint
watchdog.

    burners │ default pool          │ OPENBLAS_NUM_THREADS=1
    ────────┼───────────────────────┼───────────────────────
      0     │  194– 223 ms          │  195–207 ms
      1     │  520– 674 ms          │  200–203 ms
      2     │  506– 546 ms          │  207–210 ms
      3     │ 1350–2314 ms  ⚠LATCH  │  214–217 ms   (gap 27–31 ms)
      2×6t  │  876–4028 ms  ⚠LATCH  │  204–210 ms   (gap 126 ms)

At three busy cores of six the solve inflates ~10× and the emitter gaps
225–942 ms — past the 250 ms threshold, which latches ``MPC_STALE`` and E-STOPs
the machine. That is exactly the band the 2026-09-06 UH-3 attempt's five slow
solves (1655.1 / 2021.2 / 2158.9 / 1461.5 / 1444.7 ms) sit in. Capping the pool
to one thread removes the whole class: the solve becomes *flat* in box load.

Canonical record: ``logbook/2026-09-06-uh3-first-attempt-refusals-and-estop.md``
§ Diagnosis, "Cause pinned (2026-09-06 evening)".

**The cap is applied in the LAUNCH FILE**, not here — the environment has to be
set before numpy is imported, and by the time any node module is executing it is
far too late. ``jugglebot_launch.py`` puts ``OPENBLAS_NUM_THREADS=1`` and
``OMP_NUM_THREADS=1`` on the ``additional_env`` of the three nodes that call the
planner. This module is the **self-check**: it reads back what the process
actually got and says so, loudly, when the cap did not land.

**It WARNs, it does not refuse.** A developer running ``ros2 run jugglebot
trajectory_node`` by hand has no launch file and must still be able to work; the
cost of an uncapped pool is a slow solve under load, not an unsafe command. But
a silent uncapped pool on the robot is how a rung gets E-STOPped, so the warning
names the file and the fix.

Pure Python: no ROS2 imports, no numpy import of its own (it reads the pool
*through* whichever numpy the process already loaded, or falls back to the
environment).
"""

from __future__ import annotations

import os
from typing import Callable, Optional, Tuple

#: The environment variables the launch file sets, in the order they are read.
#: ``OPENBLAS_NUM_THREADS`` first because OpenBLAS is what numpy 1.24.4 links
#: against on this box (``libopenblas64_p-r0-cecebdce.3.21.so``, pthreads/armv8);
#: ``OMP_NUM_THREADS`` is the belt for an OpenMP-threaded build.
BLAS_ENV_VARS: Tuple[str, ...] = (
    'OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS',
    'NUMEXPR_NUM_THREADS',
)

#: The launch file the WARN tells the reader to fix.
LAUNCH_FILE = 'ros_ws/src/jugglebot/launch/jugglebot_launch.py'

#: The logbook entry that measured the dose-response above.
LOGBOOK_ENTRY = 'logbook/2026-09-06-uh3-first-attempt-refusals-and-estop.md'


def read_blas_threads() -> Tuple[Optional[int], str]:
    """The effective BLAS thread count, and where the number came from.

    Returns ``(num_threads, source)``. ``num_threads`` is ``None`` when neither
    reader can answer — an unknown pool is reported as unknown rather than
    assumed capped, because assuming capped is the failure this exists to catch.

    Two readers, in order of authority:

    * :mod:`threadpoolctl` (3.5.0 in BOTH interpreters on this box — the venv
      *and* ``/usr/bin/python3``, which is the one the launch file actually
      runs) inspects the **loaded shared objects**, so it reports the pool the
      process really has, including the case where the env var was set too late
      to take effect. This is the reader that can tell the truth.
    * The environment, as the fallback. It reports *intent* rather than
      reality — it cannot see a pool that ignored the variable — but it is
      never wrong about whether the launch file did its job, which is the
      question the WARN is really asking.
    """
    try:
        import threadpoolctl  # noqa: WPS433 (deliberately lazy + optional)
    except Exception:  # pragma: no cover - exercised via monkeypatch
        pass
    else:
        try:
            pools = threadpoolctl.threadpool_info()
        except Exception:  # pragma: no cover - defensive
            pools = []
        blas = [p for p in pools
                if str(p.get('user_api', '')).lower() in ('blas', 'openmp')]
        if blas:
            counts = [int(p.get('num_threads') or 0) for p in blas]
            worst = max(counts) if counts else 0
            names = ','.join(sorted({str(p.get('internal_api') or
                                         p.get('prefix') or '?')
                                     for p in blas}))
            return worst, 'threadpoolctl:%s' % (names,)

    for var in BLAS_ENV_VARS:
        raw = os.environ.get(var)
        if raw:
            try:
                return int(raw), 'env:%s' % (var,)
            except ValueError:
                continue
    return None, 'unknown (threadpoolctl unavailable, no *_NUM_THREADS set)'


def format_blas_line(num_threads: Optional[int], source: str) -> str:
    """The one INFO line: ``blas threads: 1 (threadpoolctl:openblas)``."""
    shown = 'unknown' if num_threads is None else str(int(num_threads))
    return 'blas threads: %s (%s)' % (shown, source)


def format_blas_warning(num_threads: Optional[int], source: str,
                        node_name: str) -> str:
    """The loud line for an uncapped pool, naming the entry and the fix."""
    return (
        '%s — %s is running with an UNCAPPED BLAS thread pool. A unified '
        'plan_cycle is thousands of small numpy calls whose spinning workers '
        'evict the 40 Hz emitter once anything else wants a core: MEASURED '
        '2026-09-06, three busy cores of six take the solve from ~200 ms to '
        '1350-2314 ms and gap the emitter 225-942 ms, past the can-bridge 250 ms '
        'MPC_STALE watchdog. Capped to one thread the same solve is 214-217 ms '
        'at any load. FIX: set OPENBLAS_NUM_THREADS=1 and OMP_NUM_THREADS=1 in '
        "this node's additional_env in %s (they must be set BEFORE numpy is "
        'imported, so a launch file is the only place that works). See %s. '
        'Continuing anyway — this is a WARNING so a hand-run node still works, '
        'NOT an assurance that it is safe to fly a rung like this.'
        % (format_blas_line(num_threads, source), node_name, LAUNCH_FILE,
           LOGBOOK_ENTRY))


def check_blas_threads(logger, node_name: str,
                       reader: Callable[[], Tuple[Optional[int], str]] = None
                       ) -> Tuple[Optional[int], str]:
    """Log the effective pool once at node start. Returns ``(n, source)``.

    ``logger`` is anything with ``.info()`` / ``.warning()`` — an rclpy logger,
    or a stand-in in a test. ``reader`` is injectable so a test can drive both
    branches without touching the process's real thread pool.

    One INFO line always; an additional WARN when the pool is larger than one
    thread **or** unknown. Unknown warns because the whole point is fail-closed
    reporting: a node that cannot prove it is capped should not read as capped.
    """
    read = reader or read_blas_threads
    num_threads, source = read()
    logger.info(format_blas_line(num_threads, source))
    if num_threads is None or int(num_threads) > 1:
        logger.warning(format_blas_warning(num_threads, source, node_name))
    return num_threads, source
