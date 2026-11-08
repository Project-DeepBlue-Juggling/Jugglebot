"""Inter-event gap tracking for the bridge's setpoint-forwarding thread.

Pure Python (no ROS, no I/O). The bridge node's ``_setpoint_loop`` records two
event streams with it:

* **forward gaps**: wall-clock spacing of consecutive successful setpoint
  sends to the can-bridge Teensy. A forward gap longer than the firmware's STOP
  cover (``SCHED_COVER_MS``) lets the lane's cover expire (BUG-34).
* **loop gaps**: spacing of consecutive iterations of the forwarding loop
  itself (it polls at ~1 kHz when idle). A long loop gap means the bridge
  process/thread was starved, regardless of what the emitter did. Comparing the
  two attributes a long forward gap: loop gap small -> the emitter stopped
  producing; loop gap large -> the bridge stalled (2026-11-01 trip 0: the
  emitter's own max_emit_gap_ms read 28.8 ms while this loop was silent 109 ms).

``record`` runs on the setpoint thread (a lock and a deque append, no I/O);
``snapshot`` runs on the 10 Hz link_status timer.
"""
from __future__ import annotations

import threading
from collections import deque
from typing import Deque, NamedTuple, Optional, Tuple

# The firmware scheduler lane's cover: 2 knots x 25 ms + 20 ms grace
# (canbridge_config.h / leg_interp.cpp; the knot period mirrors
# setpoint_pump.DEFAULT_KNOT_DT_S). The firmware source is not in this
# checkout, so this is the single Python mirror; a gap above it expires the
# lane's cover -> STOP.
SCHED_KNOT_DT_MS = 25.0
SCHED_GRACE_MS = 20.0
SCHED_COVER_MS = 2.0 * SCHED_KNOT_DT_MS + SCHED_GRACE_MS  # 70.0
# FW 37 Hand C1 stop (fast upstroke): the Hand group's grace shrinks to
# HAND_C1_STOP_GRACE_US (canbridge_config.h), so its deadline is the 50 ms
# cover + that grace. Mirror of the header constant; pinned against the header
# and the twin (teensy_interp.py) by tests/teensy_link/test_gap_monitor.py.
HAND_C1_STOP_GRACE_MS = 14.0
HAND_C1_COVER_MS = 2.0 * SCHED_KNOT_DT_MS + HAND_C1_STOP_GRACE_MS  # 64.0


class GapStats(NamedTuple):
    n: int
    max_ms: float
    p99_ms: float


EMPTY_STATS = GapStats(0, 0.0, 0.0)


class GapTracker:
    """Rolling-window max / p99 of the spacing between ``record`` calls.

    ``idle_s``: a spacing at or above this is a stream restart (between goals),
    not a gap, and is not recorded.
    """

    def __init__(self, window_s: float = 1.0, idle_s: float = 1.0,
                 maxlen: int = 4096):
        self._window_s = float(window_s)
        self._idle_s = float(idle_s)
        self._last: Optional[float] = None
        self._samples: Deque[Tuple[float, float]] = deque(maxlen=int(maxlen))
        self._lock = threading.Lock()

    def record(self, now: float) -> None:
        with self._lock:
            last = self._last
            self._last = now
            if last is None:
                return
            gap = now - last
            if 0.0 <= gap < self._idle_s:
                self._samples.append((now, gap))

    def snapshot(self, now: float, with_p99: bool = True) -> GapStats:
        with self._lock:
            cutoff = now - self._window_s
            while self._samples and self._samples[0][0] < cutoff:
                self._samples.popleft()
            gaps = [g for _, g in self._samples]
        if not gaps:
            return EMPTY_STATS
        mx = max(gaps) * 1e3
        p99 = mx
        if with_p99:
            gaps.sort()
            p99 = gaps[min(len(gaps) - 1, int(0.99 * len(gaps)))] * 1e3
        return GapStats(len(gaps), mx, p99)

    def open_gap_ms(self, now: float) -> float:
        """Time since the last ``record`` (ms), 0.0 before the first one.

        ``snapshot`` only sees a gap once it CLOSES; a stream that is dead right
        now reads as no gap at all. The STOP verdict (BUG-58) needs the open gap
        too before it may call the host stream clean.
        """
        with self._lock:
            last = self._last
        return 0.0 if last is None else max(0.0, (now - last) * 1e3)

    def reset(self) -> None:
        with self._lock:
            self._last = None
            self._samples.clear()


class EpisodeLatch:
    """True ONCE per episode: fires on the first value above ``limit`` and
    re-arms when a value is back at or below it."""

    def __init__(self, limit: float):
        self.limit = float(limit)
        self._active = False

    def update(self, value: float) -> bool:
        if value > self.limit:
            fired = not self._active
            self._active = True
            return fired
        self._active = False
        return False


class DownlinkLossTracker:
    """Setpoints the host sent that the firmware never dispatched (FEAT-41).

    ``update(sent, rx)`` takes the host's sent count, sampled when a heartbeat
    arrived, and that heartbeat's cumulative ``dn_rx_setpoints`` (FW 40).
    ``sent - rx`` is the loss, plus a constant (the two counters start at
    different times), plus the frames in flight at the sample, which only ever
    ADD. So the all-time minimum of the difference is the constant, and the
    minimum over the last ``window`` samples minus it is the loss: exact as
    soon as one sample in the window caught nothing in flight. A counter that
    goes backwards (bridge reboot, node restart) re-baselines.

    Not thread-safe: one caller (the bridge node's 10 Hz link_status timer).
    """

    def __init__(self, window: int = 10):
        self._win: Deque[int] = deque(maxlen=int(window))
        self._floor: Optional[int] = None
        self._last: Optional[Tuple[int, int]] = None

    def reset(self) -> None:
        self._win.clear()
        self._floor = None
        self._last = None

    def update(self, sent: int, rx: int) -> int:
        sent, rx = int(sent), int(rx)
        if self._last is not None and (sent < self._last[0] or rx < self._last[1]):
            self.reset()
        self._last = (sent, rx)
        d = sent - rx
        self._win.append(d)
        if self._floor is None or d < self._floor:
            self._floor = d
        return self.lost

    @property
    def lost(self) -> int:
        if not self._win or self._floor is None:
            return 0
        return min(self._win) - self._floor
