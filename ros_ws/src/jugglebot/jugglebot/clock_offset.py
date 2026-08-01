"""ROS-clock → ``perf_counter`` offset estimation — the one implementation.

``time.perf_counter`` is ``CLOCK_MONOTONIC``; a ROS node's clock is wall time.
Every deadline that crosses between the two domains (a catch arrival stamp, a
timed-target landing, an announcement's ``landing_time``) needs the offset

    offset = perf − ros            so that   t_perf = t_ros + offset

Two nodes — ``trajectory_node`` and ``catch_coordinator_node`` — carried
character-for-character identical copies of this estimator until 2026-08-01,
including the 10-sample median, the 20-deep history and the 30 s refresh
cadence.  Duplicated timing code is the worst kind to let drift: a divergence
does not fail a test, it silently biases catch deadlines on one path only.

**Why sample-then-median rather than a single read.**  A scheduler preemption
between the two clock reads inflates that one sample by the preemption length.
The median of 10 back-to-back reads shrugs that off; the median-of-20-history
does the same for a whole bad measurement round, while still tracking real
drift between the two clock sources over minutes.

**Read order is load-bearing.**  ``perf_counter`` is read FIRST, then the ROS
clock, in every sample.  The pair is not atomic, so the elapsed time between
the reads biases the offset by that amount, consistently signed.  Both call
sites always had perf-first; keep it that way or the offset shifts by a few
microseconds relative to every historical measurement.

Pure module: no ``rclpy`` import.  The clock is injected as a callable so the
estimator is testable with a scripted clock, and so each node keeps its own
exact nanoseconds→seconds expression (a ``/1e9`` and a ``* 1e-9`` can differ in
the last ulp).

Deliberately NOT unified here: ``reload_coordinator_node._announcement_landing_perf``
takes a single instantaneous ``perf − ros`` read at point of use rather than a
median-filtered running offset.  Whether that variant should adopt the filtered
offset is an open reconciliation decision, not an oversight — see the comment
at that function.
"""

from __future__ import annotations

import time
from typing import Callable, List

import numpy as np

#: Reads per measurement, median-filtered.
DEFAULT_SAMPLES = 10

#: Measurements retained; the offset in use is the median of these.
DEFAULT_HISTORY = 20

#: Seconds between refreshes (both nodes create a timer at this period).
REFRESH_PERIOD_S = 30.0


def measure_offset(
    ros_clock_s: Callable[[], float],
    perf_clock: Callable[[], float] = time.perf_counter,
    samples: int = DEFAULT_SAMPLES,
) -> float:
    """Median ``perf − ros`` offset over ``samples`` back-to-back read pairs.

    ``ros_clock_s`` must return the node's ROS clock in **seconds**.
    """
    offsets = []
    for _ in range(samples):
        t_perf = perf_clock()
        t_ros = ros_clock_s()
        offsets.append(t_perf - t_ros)
    return float(np.median(offsets))


def refresh_offset(
    history: List[float],
    ros_clock_s: Callable[[], float],
    perf_clock: Callable[[], float] = time.perf_counter,
    samples: int = DEFAULT_SAMPLES,
    max_history: int = DEFAULT_HISTORY,
) -> float:
    """Take a fresh measurement, fold it into ``history``, return the offset.

    ``history`` is mutated in place (appended, then trimmed from the front to
    ``max_history``) so callers keep holding the same list object — the nodes
    expose it as an attribute and tests read it.
    """
    history.append(measure_offset(ros_clock_s, perf_clock, samples))
    if len(history) > max_history:
        history.pop(0)
    return float(np.median(history))
