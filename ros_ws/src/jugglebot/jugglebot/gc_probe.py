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
import os
import time
from typing import Deque, Optional, Tuple

# (generation, pause_s, stop_perf_counter, stop_wall_time, collected,
#  uncollectable)
_Record = Tuple[int, float, float, float, int, int]

DEFAULT_MAXLEN = 512
WARN_PAUSE_MS = 10.0

# Slow optional knob (default OFF): seconds between ``len(gc.get_objects())``
# samples.  That call builds a list of every tracked object (~1.4 ms at 29k
# objects, measured 2026-11-21) -- a stall of its own, so never on the 1 Hz
# path unless asked.  Env var, so no node or launch change is needed.
OBJECTS_ENV = 'JUGGLEBOT_GC_PROBE_OBJECTS_EVERY_S'


class GcProbe:
    def __init__(self, maxlen: int = DEFAULT_MAXLEN,
                 objects_every_s: Optional[float] = None):
        if objects_every_s is None:
            try:
                objects_every_s = float(os.environ.get(OBJECTS_ENV, '0'))
            except ValueError:
                objects_every_s = 0.0
        self._objects_every_s = objects_every_s
        self._objects_t = 0.0
        self.objects_at_install = -1     # tracked objects when install() ran
        self.objects_now = -1            # last knob sample, -1 = knob off
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
                 time.time(), info.get('collected', -1),
                 info.get('uncollectable', -1)))

    def install(self) -> None:
        if not self._installed:
            try:
                self.objects_at_install = len(gc.get_objects())   # once
            except Exception:  # noqa: BLE001
                self.objects_at_install = -1
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
            # appended: size of the worst collection (stop-phase info dict)
            'worst_collected': worst[4],
            'worst_uncollectable': worst[5],
        }

    def state(self) -> dict:
        """Collector state at call time (1 Hz path: a few microseconds).
        The tracked-object count is sampled only when the knob is on."""
        if self._objects_every_s > 0.0:
            now = time.perf_counter()
            if now - self._objects_t >= self._objects_every_s:
                self._objects_t = now
                self.objects_now = len(gc.get_objects())
        return {
            'count': gc.get_count(),
            'colls': tuple(g['collections'] for g in gc.get_stats()),
            'frozen': gc.get_freeze_count(),
            'objects0': self.objects_at_install,
            'objects': self.objects_now,
        }


def format_summary(tag: str, s: dict, state: Optional[dict] = None) -> str:
    """The grep-friendly ``GC_PROBE`` line (the report tool parses this).

    The original fields come first and never change; everything after
    ``worst_wall`` is appended: ``worst_collected`` / ``worst_uncollectable``
    (-1 when no collection), ``gc_count`` (``gc.get_count()``), ``gc_colls``
    (cumulative collections per generation), ``gc_frozen`` (permanent
    generation size), ``gc_objects0`` (tracked objects at install) and, only
    with the knob on, ``gc_objects`` (tracked objects now).  ``state`` is
    ``GcProbe.state()``; omitted, the original fields only are emitted.
    """
    w: Optional[tuple] = s['worst']
    line = ('GC_PROBE %s n=%d max_ms=%.3f total_ms=%.3f worst_gen=%d '
            'worst_wall=%.3f' % (tag, s['count'], s['max_ms'], s['total_ms'],
                                 w[0] if w else -1, w[2] if w else 0.0))
    if state is None:
        return line
    line += (' worst_collected=%d worst_uncollectable=%d'
             % (s.get('worst_collected', -1), s.get('worst_uncollectable', -1)))
    line += (' gc_count=%d,%d,%d gc_colls=%d,%d,%d gc_frozen=%d gc_objects0=%d'
             % (tuple(state['count']) + tuple(state['colls'])
                + (state['frozen'], state['objects0'])))
    if state['objects'] >= 0:
        line += ' gc_objects=%d' % state['objects']
    return line
