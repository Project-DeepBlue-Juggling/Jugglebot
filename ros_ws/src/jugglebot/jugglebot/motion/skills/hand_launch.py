"""Hand launch-speed ratio from HAND TELEMETRY — the measured correction the
open-loop catch aim uses (owner decision 2026-09-15, "prefer the theoretical
throw state, but adjust it towards the measured one; no QTM for catch
prediction").

One number comes out of this module: ``r = v_meas / v_cmd``, the ratio of the
hand's MEASURED peak launch speed to its COMMANDED peak launch speed over the
throw stroke that ends at a planned release instant. A vertical self-toss's
flight time scales with the launch speed (``T' = r·T`` for a return to the
release height), so ``r`` is the whole correction the catch needs: at the R3
operating point (0.9 m apex, ``T`` ≈ 0.86 s) the measured ``r`` ≈ 1.086 puts
touch-down ~74 ms later than the schedule's commanded landing — the "the
plant throws ~25 % fast / the catch is not timed" symptom.

Why a RATIO of PEAKS rather than a single sample at release:

* A single ``vel_meas`` sample at the release instant is one ~100 Hz sample of
  a signal whose slope is thousands of rev/s² there; a 5 ms stamp skew is a
  several-percent error in exactly the quantity being estimated. The stroke's
  peak is a stationary point of the same signal, so it is the least
  skew-sensitive feature on it.
* The ratio is unit-free, so the two channels only have to share units
  (``HandTelemetryMessage.vel_ff_cmd`` / ``vel_meas``, both rev/s) — no
  ``hand_mm_per_rev`` gain enters, and no second copy of it can drift.
* Both peaks are taken with the SAME sign convention (the sign of the
  commanded peak), so a hand whose positive direction is reversed in some
  future wiring cannot silently invert the correction.

Pure Python + stdlib: no ROS2, no numpy, no config imports — ``skill_node``
feeds :meth:`HandLaunchMonitor.add_sample` from ``/hand_telemetry`` and hands
:meth:`HandLaunchMonitor.ratio` to the executor as its ``launch_ratio``
callable.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

#: Seconds of stroke, ending at the planned release, the peaks are taken over.
#: A throw stroke is shorter than this at every apex on the R3 ladder, and a
#: window that also contains the PREVIOUS skill's motion is harmless: a rest
#: or catch stroke's speed is far below a throw's, so it cannot own the peak.
DEFAULT_WINDOW_S = 0.30

#: Seconds PAST the planned release still admitted into the window. Two
#: things need it: the ~100 Hz telemetry cadence (the true peak can be
#: sampled a few ms late) and the release instant being the SCHEDULE's, not
#: the wire's. Kept small — the hand is braking hard after release, so a wide
#: post-window would start to compare a throw peak against a brake peak.
DEFAULT_POST_S = 0.02

#: Samples required inside the window before a ratio is offered at all.
DEFAULT_MIN_SAMPLES = 4

#: rev/s. The commanded peak must exceed this for the window to be a THROW
#: stroke at all. The slowest throw on the R3 ladder (0.5 m apex) commands
#: ~96 rev/s; rest and catch strokes are an order of magnitude below this
#: floor, and an idle hand's jitter is ~0.
DEFAULT_MIN_PEAK_CMD = 5.0

#: The ratio is rejected outside these bounds. Today's bag
#: (``~/Desktop/rosbags/2026-09-15_18-51-37``, 0.5–0.9 m apex) measured
#: r ≈ 1.05–1.09; a value outside this band is not a 9 %-class plant error
#: but a broken window (wrong stroke, a fault, a dropped channel), and the
#: safe answer to that is "no correction", i.e. keep the theoretical aim.
DEFAULT_R_MIN = 0.75
DEFAULT_R_MAX = 1.35


class HandLaunchMonitor:
    """Rolling hand-velocity history with one question: what was the launch
    speed ratio of the stroke ending at ``t_release_abs_s``?

    ``add_sample`` is called from the telemetry callback (~100 Hz);
    ``ratio`` is called from the orchestrator tick. Neither allocates beyond
    the bounded history, and ``ratio`` never raises: an un-answerable window
    returns ``None``, which the caller reads as "keep the theoretical aim".
    """

    def __init__(self, *, window_s: float = DEFAULT_WINDOW_S,
                 post_s: float = DEFAULT_POST_S,
                 min_samples: int = DEFAULT_MIN_SAMPLES,
                 min_peak_cmd: float = DEFAULT_MIN_PEAK_CMD,
                 r_min: float = DEFAULT_R_MIN, r_max: float = DEFAULT_R_MAX,
                 history_s: float = 2.0):
        self.window_s = float(window_s)
        self.post_s = float(post_s)
        self.min_samples = int(min_samples)
        self.min_peak_cmd = float(min_peak_cmd)
        self.r_min = float(r_min)
        self.r_max = float(r_max)
        self.history_s = float(history_s)
        #: ``(t_abs_s, vel_cmd, vel_meas)``, oldest first.
        self._samples: List[Tuple[float, float, float]] = []
        #: Why the last :meth:`ratio` call answered as it did — for the one
        #: log line the executor prints per catch, never for control flow.
        self.last_reason = 'no estimate yet'

    # ── ingest ──

    def add_sample(self, t_abs_s: float, vel_cmd: float,
                   vel_meas: float) -> None:
        """Record one hand telemetry sample on the SAME clock the schedule
        uses (the ROS wall clock — a monotonic stamp would not be comparable
        with a release instant)."""
        t = float(t_abs_s)
        if t != t or t <= 0.0:                      # NaN or an unstamped row
            return
        vc, vm = float(vel_cmd), float(vel_meas)
        if vc != vc or vm != vm:                    # NaN either channel
            return
        if self._samples and t < self._samples[-1][0]:
            # A clock step backwards (a ROS time jump) invalidates the whole
            # history: keep the new sample, drop what cannot be compared.
            self._samples = []
        self._samples.append((t, vc, vm))
        self._prune(t)

    def _prune(self, now_s: float) -> None:
        cutoff = now_s - self.history_s
        if self._samples and self._samples[0][0] < cutoff:
            self._samples = [s for s in self._samples if s[0] >= cutoff]

    # ── the estimate ──

    def ratio(self, t_release_abs_s: float) -> Optional[float]:
        """``r = peak measured / peak commanded`` over
        ``[t_release - window_s, t_release + post_s]``, or ``None`` when the
        window is not a trustworthy throw stroke.

        ``None`` cases, each a physical fact rather than a tuning choice:
        too few samples (telemetry not flowing, or the release is still in
        the future); a commanded peak below :data:`DEFAULT_MIN_PEAK_CMD` (the
        window holds no throw); a non-finite or out-of-band ratio (a broken
        window, not a 9 %-class plant error).
        """
        t_rel = float(t_release_abs_s)
        lo, hi = t_rel - self.window_s, t_rel + self.post_s
        window = [s for s in self._samples if lo <= s[0] <= hi]
        if len(window) < self.min_samples:
            self.last_reason = ('only %d hand samples in the stroke window '
                                '(need %d)' % (len(window), self.min_samples))
            return None
        # The commanded peak picks the stroke's direction; the measured peak
        # is taken along the SAME direction, so a sign convention cannot
        # invert the correction.
        t_peak, peak_cmd_signed, _ = max(window, key=lambda s: abs(s[1]))
        sign = 1.0 if peak_cmd_signed >= 0.0 else -1.0
        peak_cmd = sign * peak_cmd_signed
        peak_meas = max(sign * s[2] for s in window)
        if peak_cmd < self.min_peak_cmd:
            self.last_reason = ('commanded hand peak %.1f rev/s is below the '
                                '%.1f rev/s throw floor — not a throw stroke'
                                % (peak_cmd, self.min_peak_cmd))
            return None
        r = peak_meas / peak_cmd
        if r != r or r in (float('inf'), float('-inf')):
            self.last_reason = 'non-finite ratio'
            return None
        if not (self.r_min <= r <= self.r_max):
            self.last_reason = ('ratio %.3f outside [%.2f, %.2f] — the stroke '
                                'window is not trustworthy'
                                % (r, self.r_min, self.r_max))
            return None
        self.last_reason = ('r=%.3f from %d samples (cmd peak %.1f rev/s at '
                            '%+.3f s of release, meas peak %.1f rev/s)'
                            % (r, len(window), peak_cmd, t_peak - t_rel,
                               peak_meas))
        return float(r)

    def __len__(self) -> int:
        return len(self._samples)
