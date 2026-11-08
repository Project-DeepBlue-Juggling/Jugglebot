"""BUG-58 opt-in cures for stream-side stalls (pure Python, no ROS).

Both are launch-flag controlled and DEFAULT OFF.

* ``gc_freeze`` -- a gen2 collection walks every tracked object; in a node
  heap of this size it took 130-190 ms on the Jetson, longer than the
  firmware's 70 ms setpoint cover.  After init we ``gc.collect()`` once and
  ``gc.freeze()`` (every surviving object moves to the permanent generation
  and is skipped by ALL later collections), and raise the thresholds so
  gen0/gen1 run 70x/50x less often and gen2 is rare (the runtime allocations
  that remain are small).  ``gc.disable()`` is deliberately NOT used: a leak
  or a reference cycle created at run time stays collectable.
* ``cpu_partition`` -- keep the two stream processes (``trajectory_node``
  emitter, ``teensy_bridge_node`` forwarder) off the cores the CATCH-solve
  worker runs on, so a solve burst cannot preempt the 40 Hz wire.

Determinism: both run ONCE, in ``__init__`` (affinity first, before any
thread exists; gc.freeze last, before any streaming thread starts).
"""
from __future__ import annotations

import gc
import os
from typing import Callable, Iterable, List, Optional, Tuple

#: gen0 700 -> 50000: ~70x fewer young collections.  gen1 10 -> 50 and gen2
#: 10 -> 1000: a gen2 pass needs 50 * 1000 gen0-promotions' worth of churn.
#: Frozen objects are skipped by every pass, so even a gen2 that does fire
#: only walks what was allocated after startup.
GC_THRESHOLDS = (50000, 50, 1000)

DEFAULT_STREAM_CPUS = (0, 1, 2, 3)
DEFAULT_SOLVER_CPUS = (4, 5)


def freeze_gc(thresholds: Tuple[int, int, int] = GC_THRESHOLDS
              ) -> Tuple[int, Tuple[int, int, int]]:
    """collect, freeze, raise thresholds.  Returns (frozen count, thresholds)."""
    gc.collect()
    gc.freeze()
    gc.set_threshold(*thresholds)
    return gc.get_freeze_count(), tuple(gc.get_threshold())


def parse_cpus(value, ncpu: Optional[int] = None) -> List[int]:
    """'0,1,2,3' | [0, 1] | (0,) -> sorted unique ints, validated against
    ``ncpu`` (default ``os.cpu_count()``).  Raises ValueError."""
    if isinstance(value, str):
        items: Iterable = [s for s in value.replace(' ', '').split(',') if s]
    else:
        items = value
    cpus = sorted({int(c) for c in items})
    n = os.cpu_count() or 1 if ncpu is None else int(ncpu)
    if not cpus:
        raise ValueError('empty cpu set')
    bad = [c for c in cpus if c < 0 or c >= n]
    if bad:
        raise ValueError('cpu(s) %r outside 0..%d' % (bad, n - 1))
    return cpus


def format_cpus(cpus: Iterable[int]) -> str:
    return ','.join(str(c) for c in sorted(cpus))


def pin_process(cpus, setaffinity: Optional[Callable] = None,
                ncpu: Optional[int] = None) -> Tuple[bool, str]:
    """Pin the whole process (``sched_setaffinity(0)`` acts on the calling
    thread and is inherited by threads created AFTER it, so call this before
    any thread exists).  Fail-open: returns (ok, text), never raises."""
    try:
        want = parse_cpus(cpus, ncpu)
        (setaffinity or os.sched_setaffinity)(0, set(want))
        return True, format_cpus(want)
    except Exception as e:          # noqa: BLE001 -- fail-open by contract
        return False, '%s: %s' % (type(e).__name__, e)
