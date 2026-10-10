"""Minimum-latency QTM -> ROS clock mapping (pure Python, no ROS imports).

Every QTM packet gives one measurement of ``offset = ros_receive_ns - qtm_frame_ns``: the
true clock offset PLUS that packet's receive latency (QTM processing, network, socket
queueing, Python scheduling on the Jetson). Queueing only ever ADDS delay, so the
packets that carry the clock offset are the fastest ones: the estimator tracks the
LOWER ENVELOPE of the measurements, as NTP/PTP-style clock filters do, instead of
their mean.

WHY (logbook 2026-10-10-mocap-min-latency-clock-sync). The previous estimator was an
exponential moving average (alpha 0.01 per packet, ~0.3 s) of the measurement, i.e.
the clock offset plus the AVERAGE receive latency. On a loaded Jetson the average
latency wandered 4-59 ms within a 7 s BB calibration sweep (bag 2026-10-10_00-24-06)
against 0.6-2.3 ms unloaded, which moved every mocap frame stamp with it: the BB
yaw-offset calibration scattered 0.27 deg per sweep instead of 0.06 deg, and ball
tracking / catch timing see the same wander.

The estimate, per packet:

1. **Bins.** Measurements go into ``bin_s`` bins of ROS receive time; each bin keeps
   its minimum (and when it occurred). Bins older than ``window_s`` are dropped.
2. **Drift.** Each time a bin closes, the slope of the window's lower support line
   (Moon, Skelly & Towsley 1999, "Estimation and removal of clock skew from network
   delay measurements": the line on or below every bin minimum that is highest at
   their mean time, i.e. the lower-convex-hull edge spanning that mean) is clamped to
   ``max_drift`` and low-passed (``drift_tau_s``) into the drift term - the
   QTM-vs-Jetson clock drift. No slope until ``min_fit_bins`` bins span
   ``min_fit_span_s``.
   **Level.** The highest line with the drift term's slope that lies on or below
   every bin minimum, evaluated at the packet's receive time. With zero drift this is
   the plain window minimum; the drift term removes its drift x window lag.
   After startup a sample more than ``outlier_clip_s`` below the current
   envelope only lowers its bin to that clip (see DEFAULT_OUTLIER_CLIP_S).
3. **Output.** The level, then SLEW-LIMITED to
   ``max_slew`` (s/s): consumers never see a step. Startup is the exception: for the
   first ``window_s`` after (re)anchoring the output follows the estimate directly,
   so it is usable from the first packet (the first measurement itself) and drops
   onto the envelope as faster packets arrive.
4. **Re-anchoring** (full reset, startup again): QTM time going backwards or jumping
   forward by more than ``restart_jump_s`` (QTM measurement restart);
   ``reanchor_below_n`` consecutive packets more than ``reanchor_below_s`` BELOW the
   output (latency cannot be negative - the mapping itself moved, e.g. a ROS clock
   step); or every packet for ``reanchor_above_s_window`` seconds more than
   ``reanchor_above_s`` above it (a forward clock step; queueing never holds every
   packet that late for that long).

All times are integer nanoseconds at the interface; internally they are taken
relative to the anchoring packet so floats stay small.
"""

from __future__ import annotations

from collections import deque
from typing import Optional

#: Seconds of history the envelope is fitted over. Chosen from the bags
#: (logbook 2026-10-10-mocap-min-latency-clock-sync, replay of 4/6/10/16 s):
#: under the bag-2 load an unqueued packet reaches the floor only every few
#: seconds, and 16 s held the within-sweep offset range to <= 0.7 ms there
#: (10 s: 2.1 ms) while costing nothing unloaded. The drift term keeps the long
#: window from lagging a drifting offset.
DEFAULT_WINDOW_S = 16.0
#: Bin width for the per-bin minima the support line is fitted to.
DEFAULT_BIN_S = 0.5
#: Output slew limit (seconds per second): 1000 ppm, five times the drift clamp
#: so drift is always tracked, and a frame stamp never moves more than 1 us per
#: ms of wall time (3.3 us between 300 Hz frames).
DEFAULT_MAX_SLEW = 1.0e-3
#: Clamp on the drift term (seconds per second). Measured QTM-vs-Jetson drift on
#: the 2026-10-09/10 bags: 3-10 ppm.
DEFAULT_MAX_DRIFT = 2.0e-4
#: Time constant of the low-pass on the support line's slope (the drift term).
DEFAULT_DRIFT_TAU_S = 30.0
#: After startup a packet can lower its bin's minimum to at most this far below
#: the current envelope. The bags show isolated packets 1.2-2 ms below an
#: otherwise steady floor, about one per 15 s unloaded (a timestamp anomaly or
#: an unusually fast QTM frame - not the floor the mapping should sit on); a
#: plain minimum would follow each one for a whole window. A genuine move of the
#: floor is either slow (drift: microseconds per bin) or a clock step, which the
#: re-anchor rules handle.
DEFAULT_OUTLIER_CLIP_S = 0.5e-3


class MinLatencyClockSync:
    """Sliding-window lower-envelope estimator of the QTM -> ROS clock offset.

    ``update(qtm_us, ros_ns)`` once per packet; ``offset_ns`` is the current mapping
    (``ros_ns ~= qtm_us * 1000 + offset_ns``) or None before the first packet.
    """

    def __init__(self, *, window_s: float = DEFAULT_WINDOW_S, bin_s: float = DEFAULT_BIN_S,
                 max_slew: float = DEFAULT_MAX_SLEW, max_drift: float = DEFAULT_MAX_DRIFT,
                 min_fit_bins: int = 3, min_fit_span_s: float = 2.0,
                 outlier_clip_s: Optional[float] = DEFAULT_OUTLIER_CLIP_S,
                 drift_tau_s: float = DEFAULT_DRIFT_TAU_S,
                 restart_jump_s: float = 5.0,
                 reanchor_below_s: float = 0.020, reanchor_below_n: int = 5,
                 reanchor_above_s: float = 0.250, reanchor_above_s_window: float = 2.0):
        if bin_s <= 0 or window_s < 2 * bin_s:
            raise ValueError('need bin_s > 0 and window_s >= 2 * bin_s')
        self.window_ns = int(window_s * 1e9)
        self.bin_ns = int(bin_s * 1e9)
        self.max_slew = float(max_slew)
        self.max_drift = float(max_drift)
        self.min_fit_bins = int(min_fit_bins)
        self.min_fit_span_ns = int(min_fit_span_s * 1e9)
        self.drift_tau_ns = float(drift_tau_s) * 1e9
        self.outlier_clip_ns = None if outlier_clip_s is None else float(outlier_clip_s) * 1e9
        self.restart_jump_us = int(restart_jump_s * 1e6)
        self.reanchor_below_ns = int(reanchor_below_s * 1e9)
        self.reanchor_below_n = int(reanchor_below_n)
        self.reanchor_above_ns = int(reanchor_above_s * 1e9)
        self.reanchor_above_window_ns = int(reanchor_above_s_window * 1e9)
        # Lifetime diagnostics (survive re-anchoring).
        self.slew_clamps = 0
        self.outlier_clips = 0
        self.reanchors = 0
        self.restarts = 0
        self.last_event: Optional[str] = None
        self._reset()

    # ── state ──────────────────────────────────────────────────────────
    def _reset(self) -> None:
        self.offset_ns: Optional[int] = None
        self.count = 0                      # packets since (re)anchoring
        self._anchor_off = 0                # measured offset of the anchoring packet (ns)
        self._anchor_t = 0                  # ROS receive time of the anchoring packet (ns)
        self._out = 0.0                     # output, ns relative to _anchor_off
        self._est = 0.0                     # last unslewed envelope estimate, same frame
        self._last_t = 0                    # last receive time, ns relative to _anchor_t
        self._bins: deque = deque()         # [bin_index, t_min_rel_ns, d_min_rel_ns]
        self._below_run = 0
        self._last_qtm_us: Optional[int] = None
        self.last_measured_rel_ns = 0.0     # last measurement minus output (ns): its excess latency
        self.envelope_rel_ns = 0.0          # unclamped estimate minus output (ns): pending slew
        self.drift = 0.0                    # drift term (s/s)
        self._drift_ns_s = 0.0              # the same, ns per s

    def _anchor(self, measured_ns: int, ros_ns: int) -> None:
        self._anchor_off = measured_ns
        self._anchor_t = ros_ns
        self._out = 0.0
        self._est = 0.0
        self._last_t = 0
        self._bins.clear()
        self._add_to_bin(0, 0.0)
        self.offset_ns = measured_ns
        self.count = 1

    # ── public ─────────────────────────────────────────────────────────
    def update(self, qtm_us: int, ros_ns: int) -> Optional[str]:
        """Feed one packet. Returns None, or an event string: ``'init'`` (first packet),
        ``'restart'`` (QTM time discontinuity), ``'reanchor_below'`` / ``'reanchor_above'``
        (the mapping moved far outside the window's range)."""
        qtm_us = int(qtm_us)
        ros_ns = int(ros_ns)
        measured = ros_ns - qtm_us * 1000
        event = None
        if self._last_qtm_us is not None:
            dt_us = qtm_us - self._last_qtm_us
            if dt_us < 0 or dt_us > self.restart_jump_us:
                self.restarts += 1
                event = 'restart'
                self._reset()
        self._last_qtm_us = qtm_us
        if self.offset_ns is None:
            self._anchor(measured, ros_ns)
            self.last_measured_rel_ns = 0.0
            self.envelope_rel_ns = 0.0
            self.last_event = event or 'init'
            return self.last_event

        t = ros_ns - self._anchor_t
        d = float(measured - self._anchor_off)
        excess = d - self._out

        # Re-anchor checks against the CURRENT output, before this sample moves it.
        if excess < -self.reanchor_below_ns:
            self._below_run += 1
            if self._below_run >= self.reanchor_below_n:
                return self._reanchor(measured, ros_ns, 'reanchor_below')
        else:
            self._below_run = 0

        self.count += 1
        t = max(t, self._last_t)            # a ROS clock that steps back is caught above/below
        dt = t - self._last_t
        self._last_t = t
        d_bin = d
        if (self.outlier_clip_ns is not None and t >= self.window_ns
                and d < self._est - self.outlier_clip_ns):
            d_bin = self._est - self.outlier_clip_ns
            self.outlier_clips += 1
        new_bin = self._add_to_bin(t, d_bin)
        while self._bins and self._bins[0][0] * self.bin_ns < t - self.window_ns:
            self._bins.popleft()
        if new_bin:
            self._update_drift()

        if (t >= self.reanchor_above_window_ns
                and self._recent_min(t, self.reanchor_above_window_ns) - self._out > self.reanchor_above_ns):
            return self._reanchor(measured, ros_ns, 'reanchor_above')

        est = self._estimate(t)
        self._est = est
        if t < self.window_ns:              # startup: follow the estimate
            self._out = est
        else:
            step = est - self._out
            lim = self.max_slew * dt
            if step > lim:
                step = lim; self.slew_clamps += 1
            elif step < -lim:
                step = -lim; self.slew_clamps += 1
            self._out += step
        self.offset_ns = self._anchor_off + int(round(self._out))
        self.last_measured_rel_ns = d - self._out
        self.envelope_rel_ns = est - self._out
        return None

    def _reanchor(self, measured: int, ros_ns: int, why: str) -> str:
        self.reanchors += 1
        last_qtm = self._last_qtm_us
        self._reset()
        self._last_qtm_us = last_qtm
        self._anchor(measured, ros_ns)
        self.last_event = why
        return why

    def window_fill(self) -> float:
        """Fraction of the window's bins that hold at least one packet."""
        n_bins = self.window_ns // self.bin_ns
        return min(1.0, len(self._bins) / float(n_bins)) if n_bins else 0.0

    def diagnostics(self) -> dict:
        return {
            'sample_count': self.count,
            'last_excess_latency_ms': self.last_measured_rel_ns / 1e6,
            'envelope_minus_output_ms': self.envelope_rel_ns / 1e6,
            'window_fill': self.window_fill(),
            'drift_ppm': self.drift * 1e6,
            'slew_clamps': self.slew_clamps,
            'outlier_clips': self.outlier_clips,
            'reanchors': self.reanchors,
            'restarts': self.restarts,
        }

    # ── internals ──────────────────────────────────────────────────────
    def _add_to_bin(self, t: int, d: float) -> bool:
        """Fold a sample into its bin; True when it opened a new bin."""
        k = t // self.bin_ns
        if self._bins and self._bins[-1][0] == k:
            b = self._bins[-1]
            if d < b[2]:
                b[1] = t; b[2] = d
            return False
        self._bins.append([k, t, d])
        return True

    def _recent_min(self, t: int, span_ns: int) -> float:
        m = float('inf')
        for k, tb, db in reversed(self._bins):
            if (k + 1) * self.bin_ns <= t - span_ns:
                break
            m = min(m, db)
        return m

    def _hull_slope(self) -> Optional[float]:
        """Slope (ns/s) of the lower support line of the window's bin minima: the
        lower-convex-hull edge spanning their mean time (Moon et al. 1999). None while
        there are too few bins / too short a span to fit one."""
        pts = [(b[1] / 1e9, b[2]) for b in self._bins]      # (s, ns)
        if len(pts) < self.min_fit_bins or (pts[-1][0] - pts[0][0]) * 1e9 < self.min_fit_span_ns:
            return None
        hull: list = []
        for p in pts:                                        # monotone chain, time-ordered
            while len(hull) >= 2:
                (t1, d1), (t2, d2) = hull[-2], hull[-1]
                if (t2 - t1) * (p[1] - d1) - (d2 - d1) * (p[0] - t1) <= 0:
                    hull.pop()
                else:
                    break
            hull.append(p)
        tbar = sum(p[0] for p in pts) / len(pts)
        for (t1, d1), (t2, d2) in zip(hull, hull[1:]):
            if t1 <= tbar <= t2:
                return (d2 - d1) / (t2 - t1)
        return 0.0

    def _update_drift(self) -> None:
        """Once per closed bin: low-pass the support-line slope into the drift term.
        One window's slope is noisy (the bin minima scatter ~0.3 ms, and it is
        extrapolated to the window's end); the clocks' drift changes over minutes,
        so a ``drift_tau_s`` low-pass costs no tracking and removes most of that
        noise."""
        slope = self._hull_slope()
        if slope is None:
            return
        lim = self.max_drift * 1e9
        slope = max(-lim, min(lim, slope))
        gain = min(1.0, self.bin_ns / self.drift_tau_ns)
        self._drift_ns_s += gain * (slope - self._drift_ns_s)
        self.drift = self._drift_ns_s / 1e9

    def _estimate(self, t_now: int) -> float:
        """The support line with the drift term's slope, at t_now: the highest line of
        that slope on or below every bin minimum in the window."""
        a = self._drift_ns_s
        ts = t_now / 1e9
        return min(d - a * (tb / 1e9 - ts) for _k, tb, d in self._bins)
