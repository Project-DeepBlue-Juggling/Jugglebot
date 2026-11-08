"""BUG-58 opt-in GC pause probe (pure Python, no ROS imports).

Question it answers: is the whole-process Python pause behind the 40 Hz
setpoint-stream stalls a garbage collection, or is it CPU preemption?  A
``gc.callbacks`` hook records every collection's generation and start/stop
``time.perf_counter()``; a low-rate timer in the owning node drains it with
:meth:`GcProbe.summary` and logs one line.

The callback runs at GC time, inside whatever thread triggered the collection:
it does one ``perf_counter()`` call, one dict-free tuple update and (on stop)
one bounded ``deque.append`` -- no logging, no I/O, no lock.  Nothing is
installed unless the node's ``gc_probe`` parameter is true.

Logbook: logbook/2026-11-03-jb149-gc-sched-probe.md
"""
from __future__ import annotations

import collections
import gc
import time
from typing import Deque, Optional, Tuple

# (generation, pause_s, stop_perf_counter, stop_wall_time)
_Record = Tuple[int, float, float, float]

DEFAULT_MAXLEN = 512
WARN_PAUSE_MS = 10.0


class GcProbe:
    def __init__(self, maxlen: int = DEFAULT_MAXLEN):
        self._records: Deque[_Record] = collections.deque(maxlen=maxlen)
        self._t_start = 0.0
        self._installed = False

    @property
    def installed(self) -> bool:
        return self._installed

    def _callback(self, phase: str, info: dict) -> None:
        if phase == 'start':
            self._t_start = time.perf_counter()
        else:
            now = time.perf_counter()
            self._records.append(
                (info.get('generation', -1), now - self._t_start, now,
                 time.time()))

    def install(self) -> None:
        if not self._installed:
            gc.callbacks.append(self._callback)
            self._installed = True

    def uninstall(self) -> None:
        if self._installed:
            try:
                gc.callbacks.remove(self._callback)
            except ValueError:
                pass
            self._installed = False

    def summary(self) -> dict:
        """Drain the records.  ``worst`` is ``(generation, pause_ms, wall_time)``
        of the longest pause since the last call, or ``None`` if none."""
        recs = []
        while True:
            try:
                recs.append(self._records.popleft())
            except IndexError:
                break
        if not recs:
            return {'count': 0, 'max_ms': 0.0, 'total_ms': 0.0, 'worst': None}
        worst = max(recs, key=lambda r: r[1])
        return {
            'count': len(recs),
            'max_ms': worst[1] * 1e3,
            'total_ms': sum(r[1] for r in recs) * 1e3,
            'worst': (worst[0], worst[1] * 1e3, worst[3]),
        }


def format_summary(tag: str, s: dict) -> str:
    """The grep-friendly ``GC_PROBE`` line (the report tool parses this)."""
    w: Optional[tuple] = s['worst']
    return ('GC_PROBE %s n=%d max_ms=%.3f total_ms=%.3f worst_gen=%d '
            'worst_wall=%.3f' % (tag, s['count'], s['max_ms'], s['total_ms'],
                                 w[0] if w else -1, w[2] if w else 0.0))
