#!/usr/bin/env python3
"""Pure system-ID logic for the bench-leg gain-tuning harness.

This module is the CAN-free half of ``bench_leg_sysid.py`` — every function
here is a deterministic transform on numeric arrays or state, with **no**
``python-can`` import, no hardware, and no I/O. That split is deliberate: the
harness is only runnable on a real bench leg, but the parts most likely to
harbour a subtle bug — the instability-onset detector, the gain-escalation
state machine, and the stroke-cap-bounded stimulus generators — are exactly
the parts we can (and do) unit-test off-hardware in
``tests/motion/test_bench_sysid_logic.py``.

Contents map to Stage 1 of the leg-gain tuning methodology
(``plans/active/leg-gain-tuning-methodology.md``, "Fast-motion tier" §):

* Stage 1a step-response metrics — rise / overshoot / settle / damping.
* Stage 1a chirp frequency response — log sweep + lock-in gain/phase.
* Instability-onset detection — velocity-ripple RMS + non-decaying
  autocorrelation (the sustained-oscillation discriminant), plus
  braking-current-cycling and overshoot helpers.
* Stroke-cap-bounded stimulus generators — every step / chirp / velocity
  step is clamped to the 3.0-rev bench stroke before it can reach a motor.
* Stage 1b gain-escalation ladder — the escalate-until-unstable state
  machine with auto-backoff to the last stable triple.

The damping target throughout is ζ ≥ 0.7 (≤ ~5 % overshoot), per the
methodology's fast-motion tier.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np


# ===========================================================================
# Stage 1a — step-response metrics
# ===========================================================================
#
# All metrics take arrays already sliced to the step window (t[0] = step
# onset).  They are sign-agnostic: a retracting step (y_final < y0) is handled
# by normalising against the signed step height, so extension and retraction
# steps share one code path.


def _step_height(y0: float, y_final: float) -> float:
    return float(y_final - y0)


def _first_crossing_time(t: np.ndarray, level: np.ndarray,
                         threshold: float) -> Optional[float]:
    """Time at which a rising normalised ``level`` first reaches ``threshold``.

    Linear interpolation between the bracketing samples keeps the estimate
    independent of the sampling rate — a coarse 100 Hz log otherwise
    quantises rise time into 10 ms buckets.
    """
    for i in range(1, len(level)):
        if level[i] >= threshold:
            l0, l1 = level[i - 1], level[i]
            if l1 == l0:
                return float(t[i])
            frac = (threshold - l0) / (l1 - l0)
            return float(t[i - 1] + frac * (t[i] - t[i - 1]))
    return None


def rise_time(t: np.ndarray, y: np.ndarray, y0: float, y_final: float,
              lo: float = 0.1, hi: float = 0.9) -> float:
    """10 %→90 % rise time in seconds; NaN if the step never gets there."""
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    h = _step_height(y0, y_final)
    if h == 0.0 or t.size < 2:
        return float('nan')
    level = (y - y0) / h  # 0 at start, 1 at final, regardless of step sign
    t_lo = _first_crossing_time(t, level, lo)
    t_hi = _first_crossing_time(t, level, hi)
    if t_lo is None or t_hi is None:
        return float('nan')
    return float(t_hi - t_lo)


def overshoot_fraction(y: np.ndarray, y0: float, y_final: float) -> float:
    """Peak excursion past the final value, as a fraction of step height.

    0.0 for a monotone (over/critically damped) step; 0.05 = 5 % overshoot.
    """
    y = np.asarray(y, float)
    h = _step_height(y0, y_final)
    if h == 0.0 or y.size == 0:
        return 0.0
    extreme = float(np.max(y)) if h > 0 else float(np.min(y))
    os = (extreme - y_final) / h  # positive for both step directions
    return max(0.0, os)


def settling_time(t: np.ndarray, y: np.ndarray, y0: float, y_final: float,
                  tol: float = 0.02) -> float:
    """Time until ``y`` enters and stays within ``tol``·|step| of the final.

    Returns 0.0 if already settled at t[0]; NaN if it never settles inside
    the window.
    """
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    h = _step_height(y0, y_final)
    if t.size == 0:
        return float('nan')
    band = tol * abs(h)
    outside = np.abs(y - y_final) > band
    if not np.any(outside):
        return 0.0
    last_violation = int(np.max(np.nonzero(outside)[0]))
    if last_violation + 1 >= t.size:
        return float('nan')  # still outside the band at the end of the window
    return float(t[last_violation + 1] - t[0])


def damping_ratio_from_overshoot(overshoot: float) -> float:
    """ζ from percent-overshoot, standard 2nd-order relation.

    ζ = −ln(OS) / √(π² + ln²OS).  OS ≤ 0 → no overshoot → ζ clamped to 1.0;
    OS ≥ 1 (peak past 2× final) → ζ clamped to 0.0 (past the linear model).
    """
    if overshoot <= 0.0:
        return 1.0
    if overshoot >= 1.0:
        return 0.0
    ln = math.log(overshoot)
    return -ln / math.sqrt(math.pi ** 2 + ln ** 2)


@dataclass
class StepMetrics:
    rise_time_s: float
    overshoot: float
    settling_time_s: float
    zeta: float


def fit_step_response(t: np.ndarray, y: np.ndarray, y0: float, y_final: float,
                      settle_tol: float = 0.02) -> StepMetrics:
    """Bundle the four step metrics for a single step window."""
    os = overshoot_fraction(y, y0, y_final)
    return StepMetrics(
        rise_time_s=rise_time(t, y, y0, y_final),
        overshoot=os,
        settling_time_s=settling_time(t, y, y0, y_final, tol=settle_tol),
        zeta=damping_ratio_from_overshoot(os),
    )


# ===========================================================================
# Instability-onset detection
# ===========================================================================
#
# The core discriminant for "the loop just went unstable" is a sustained
# oscillation: high AC energy that does NOT decay.  Ripple RMS alone can't
# tell a one-shot transient (large, decaying) from a limit cycle (large,
# persistent); the autocorrelation late-lag peak supplies the "persistent"
# half.  Requiring BOTH keeps a noisy-but-stable hold from tripping backoff.


def _detrend_linear(x: np.ndarray) -> np.ndarray:
    """Remove the best-fit line so a slewing setpoint doesn't masquerade as
    ripple.  A pure ramp detrends to ~0; a limit cycle survives."""
    n = x.size
    if n < 2:
        return x - x.mean() if n else x
    tt = np.arange(n, dtype=float)
    A = np.vstack([tt, np.ones(n)]).T
    coef, *_ = np.linalg.lstsq(A, x, rcond=None)
    return x - A @ coef


def ripple_rms(x: Sequence[float], detrend: bool = True) -> float:
    """RMS of the AC (ripple) component of a signal.

    With ``detrend`` (default) a linear trend is removed first, so a clean
    velocity ramp reads ~0 ripple and only genuine oscillation shows up.
    """
    x = np.asarray(x, float)
    if x.size == 0:
        return 0.0
    d = _detrend_linear(x) if detrend else (x - x.mean())
    return float(np.sqrt(np.mean(d ** 2)))


def normalized_autocorrelation(x: Sequence[float],
                               detrend: bool = True) -> np.ndarray:
    """Autocorrelation normalised to 1.0 at zero lag.

    A flat / DC / empty signal has no correlation structure → returns
    ``[1, 0, 0, …]`` so the sustained-oscillation score reads 0.
    """
    x = np.asarray(x, float)
    n = x.size
    if n == 0:
        return np.array([], dtype=float)
    d = _detrend_linear(x) if detrend else (x - x.mean())
    ac = np.correlate(d, d, mode='full')[n - 1:]
    if ac[0] == 0.0:
        out = np.zeros(n)
        out[0] = 1.0
        return out
    return ac / ac[0]


def sustained_oscillation_score(x: Sequence[float], late_frac: float = 0.3,
                                hi_frac: float = 0.7) -> float:
    """Peak |autocorrelation| in the *late* lag window ∈ [0, 1].

    A decaying transient's autocorrelation envelope has collapsed by the late
    lags → low score.  A limit cycle's autocorrelation still peaks near 1 at
    period-aligned lags → high score.  This is the "non-decaying" half of the
    onset test.

    The biased estimator from ``np.correlate`` tapers by (1 − lag/N), which
    would let a genuinely non-decaying oscillation score lower simply because
    the window sits late.  We divide that triangular bias out (N/(N−lag)) so
    the score reflects *decay*, not window placement — a sustained tone reads
    ~1.0 at any frequency, while a decaying one stays low.  ``hi_frac`` caps
    the window before the bias correction blows up on the few-sample far lags.
    """
    x = np.asarray(x, float)
    n = x.size
    if n < 3:
        return 0.0
    ac = normalized_autocorrelation(x)
    lo = int(n * late_frac)
    hi = min(n, max(lo + 1, int(n * hi_frac)))
    if lo >= hi:
        return 0.0
    lags = np.arange(lo, hi)
    corrected = np.abs(ac[lo:hi]) * (n / (n - lags))
    return float(min(np.max(corrected), 1.0)) if corrected.size else 0.0


# --- Non-oscillatory divergence guard --------------------------------------
#
# The sustained-oscillation test above needs a PERIODIC signature, so it is
# structurally blind to a non-oscillatory runaway (a real-axis instability /
# integrator windup): the autocorrelation score stays low and the linear
# detrend suppresses a steady drift, so a slow monotonic divergence slips BOTH
# onset gates.  This guard is periodicity-independent: it measures whether the
# RAW |signal| envelope is GROWING across the window (late half vs early half).
# Deliberately NOT detrended — detrending is exactly what erases the monotonic
# drift the guard exists to catch.  A settled hold (flat) or a healthy
# constant-velocity step (constant) does not grow; a runaway does.


def divergence_metrics(x: Sequence[float], split_frac: float = 0.5
                       ) -> Tuple[float, float]:
    """(growth_ratio, terminal_rms) of the RAW envelope, early half → late half.

    ``growth_ratio`` = (late_rms − early_rms) / (early_rms + ε), clamped at 0 so
    only GROWTH counts (a decaying transient reads 0).  ``terminal_rms`` is the
    late-half RMS — its absolute magnitude, so a caller can require the growth to
    clear a noise floor before flagging (a near-zero settled tail must not trip
    on ratio noise alone).
    """
    arr = np.asarray(x, float)
    n = arr.size
    if n < 4:
        return 0.0, 0.0
    k = int(n * split_frac)
    if k < 1 or k >= n:
        return 0.0, 0.0
    early_rms = float(np.sqrt(np.mean(arr[:k] ** 2)))
    late_rms = float(np.sqrt(np.mean(arr[k:] ** 2)))
    ratio = max(0.0, (late_rms - early_rms) / (early_rms + 1e-9))
    return ratio, late_rms


def divergence_score(x: Sequence[float], split_frac: float = 0.5) -> float:
    """Raw-envelope growth ratio alone (convenience wrapper over
    :func:`divergence_metrics`)."""
    return divergence_metrics(x, split_frac)[0]


@dataclass
class OnsetResult:
    unstable: bool
    ripple_rms: float
    oscillation_score: float
    reasons: List[str] = field(default_factory=list)
    divergence_score: float = 0.0


def detect_instability_onset(vel: Sequence[float], *,
                             ripple_rms_threshold: float,
                             oscillation_score_threshold: float,
                             divergence_threshold: float = 0.5,
                             detrend: bool = True) -> OnsetResult:
    """Instability-onset test on a velocity trace.

    Unstable ⇔ (ripple RMS above threshold AND non-decaying autocorrelation) OR
    non-oscillatory divergence — the sustained-oscillation half catches limit
    cycles, the divergence half catches a monotonic runaway the oscillation test
    is structurally blind to.  ``vel`` in rev/s; thresholds are caller-supplied
    so the harness can set them from the measured hold noise floor.

    The divergence branch fires only when the raw envelope GROWS by more than
    ``divergence_threshold`` (fractional) AND the late-half magnitude clears the
    ripple noise floor — so a near-zero settled hold cannot trip it on ratio
    noise, while a velocity ramping past the floor does.
    """
    ripple = ripple_rms(vel, detrend=detrend)
    score = sustained_oscillation_score(vel)
    div_ratio, div_rms = divergence_metrics(vel)
    ripple_hi = ripple > ripple_rms_threshold
    osc_hi = score > oscillation_score_threshold
    div_hi = div_ratio > divergence_threshold and div_rms > ripple_rms_threshold
    reasons: List[str] = []
    if ripple_hi:
        reasons.append(
            "velocity ripple RMS %.4f > %.4f rev/s"
            % (ripple, ripple_rms_threshold))
    if osc_hi:
        reasons.append(
            "non-decaying autocorrelation (score %.2f > %.2f)"
            % (score, oscillation_score_threshold))
    if div_hi:
        reasons.append(
            "non-oscillatory divergence (envelope growth %.2f > %.2f, "
            "terminal RMS %.4f rev/s)" % (div_ratio, divergence_threshold, div_rms))
    unstable = (ripple_hi and osc_hi) or div_hi
    return OnsetResult(unstable=unstable, ripple_rms=ripple,
                       oscillation_score=score, reasons=reasons,
                       divergence_score=div_ratio)


@dataclass
class BrakingCycleResult:
    cycling: bool
    ripple_rms: float
    oscillation_score: float
    negative_fraction: float


def braking_current_cycling(iq: Sequence[float], *,
                            ripple_threshold: float,
                            oscillation_threshold: float,
                            negative_fraction_threshold: float = 0.1
                            ) -> BrakingCycleResult:
    """Detect the negative-iq surge/collapse limit cycle.

    The forensic signature of the ~6 Hz stutter is the current loop cycling
    into regenerative braking (negative iq) and back.  We flag it only when
    the current both *oscillates persistently* (ripple + non-decaying AC) AND
    spends a meaningful fraction of the window braking — steady forward draw
    with noise must not trip it.
    """
    arr = np.asarray(iq, float)
    ripple = ripple_rms(arr)
    score = sustained_oscillation_score(arr)
    neg_frac = float(np.mean(arr < 0.0)) if arr.size else 0.0
    cycling = (ripple > ripple_threshold
               and score > oscillation_threshold
               and neg_frac > negative_fraction_threshold)
    return BrakingCycleResult(cycling=cycling, ripple_rms=ripple,
                              oscillation_score=score,
                              negative_fraction=neg_frac)


# ===========================================================================
# Stage 1a — chirp / frequency response
# ===========================================================================


def chirp_instantaneous_freq(t: np.ndarray, f_start: float, f_end: float,
                             duration: float) -> np.ndarray:
    """Instantaneous frequency (Hz) of the log sweep at each time."""
    t = np.asarray(t, float)
    return f_start * (f_end / f_start) ** (t / duration)


def log_chirp(t: np.ndarray, f_start: float, f_end: float,
              duration: float) -> np.ndarray:
    """Unit-amplitude exponential (log) frequency sweep sine in [-1, 1].

    Exponential sweeps spend equal *log*-frequency time at each octave, so a
    1→30 Hz sweep resolves the low decades (where the pos_gain/(2π) loop
    resonance lives — the 6 Hz question) as well as the high end.  Callers
    scale by a stroke-bounded amplitude; see ``chirp_position_series``.
    """
    t = np.asarray(t, float)
    if f_start <= 0 or f_end <= 0:
        raise ValueError("chirp frequencies must be positive")
    if f_end == f_start:
        return np.sin(2.0 * math.pi * f_start * t)
    k = duration / math.log(f_end / f_start)
    phase = 2.0 * math.pi * f_start * k * ((f_end / f_start) ** (t / duration) - 1.0)
    return np.sin(phase)


def single_freq_response(t: np.ndarray, u: np.ndarray, y: np.ndarray,
                         freq: float) -> Tuple[float, float]:
    """Closed-loop gain (dimensionless) and phase (deg) at one frequency.

    Single-bin lock-in: project input and output onto e^{−jωt} and take the
    complex ratio.  The 2/N normalisation cancels in the ratio, so this is
    exact for a pure tone over an integer number of periods and robust to the
    absolute amplitude of the drive.
    """
    t = np.asarray(t, float)
    u = np.asarray(u, float)
    y = np.asarray(y, float)
    w = 2.0 * math.pi * freq
    ref = np.exp(-1j * w * t)
    x_u = np.mean(u * ref)
    x_y = np.mean(y * ref)
    if x_u == 0:
        return float('nan'), float('nan')
    h = x_y / x_u
    return float(abs(h)), float(np.degrees(np.angle(h)))


def estimate_frequency_response(t: np.ndarray, u: np.ndarray, y: np.ndarray,
                                freqs: Sequence[float]
                                ) -> Tuple[np.ndarray, np.ndarray]:
    """Gain and phase (deg) arrays over ``freqs`` via per-bin lock-in."""
    gains = np.empty(len(freqs))
    phases = np.empty(len(freqs))
    for i, f in enumerate(freqs):
        gains[i], phases[i] = single_freq_response(t, u, y, f)
    return gains, phases


# ===========================================================================
# Stroke-cap-bounded stimulus generators
# ===========================================================================
#
# Every stimulus is clamped to the bench leg's mechanical stroke BEFORE a
# setpoint can reach the ODrive.  The firmware STROKE_MAX_REV is sized for a
# 3.9-rev platform leg and does NOT protect the shorter 3.0-rev bench leg, so
# this driver-side clamp is the only thing standing between a mis-sized
# amplitude and an end-stop crash.


@dataclass(frozen=True)
class StrokeBounds:
    """Absolute position window (rev) the leg is allowed to occupy."""
    lo_rev: float
    hi_rev: float

    def clamp(self, pos_rev: float) -> float:
        return min(self.hi_rev, max(self.lo_rev, pos_rev))

    def contains(self, pos_rev: float, tol: float = 0.0) -> bool:
        return (self.lo_rev - tol) <= pos_rev <= (self.hi_rev + tol)

    @property
    def width(self) -> float:
        return self.hi_rev - self.lo_rev


def stroke_bounds(stroke_cap_rev: float, margin_rev: float = 0.0) -> StrokeBounds:
    """Bounds spanning [margin, stroke_cap − margin] of the leg's travel.

    ``margin_rev`` backs the usable window off both end-stops so a small PID
    overshoot near a limit can't crash the leg.
    """
    if margin_rev * 2 >= stroke_cap_rev:
        raise ValueError("margin too large for stroke cap")
    return StrokeBounds(margin_rev, stroke_cap_rev - margin_rev)


def symmetric_amplitude_limit(center_rev: float, bounds: StrokeBounds) -> float:
    """Largest zero-mean amplitude that keeps center±amp inside ``bounds``."""
    return max(0.0, min(center_rev - bounds.lo_rev, bounds.hi_rev - center_rev))


def clamp_amplitude(center_rev: float, amplitude_rev: float,
                    bounds: StrokeBounds) -> float:
    """Reduce a requested amplitude to fit the stroke window (never grows it)."""
    return min(abs(amplitude_rev), symmetric_amplitude_limit(center_rev, bounds))


@dataclass
class PositionStep:
    requested_rev: float   # center + step, before clamping
    target_rev: float      # actually commanded (clamped into bounds)
    clamped: bool          # True if the requested target hit a bound


def position_step_series(center_rev: float, step_sizes_rev: Sequence[float],
                         bounds: StrokeBounds) -> List[PositionStep]:
    """Absolute step targets = center + step, each clamped into ``bounds``.

    Reports ``clamped`` per step so the harness can warn the operator that a
    requested step was truncated rather than silently shrinking it.
    """
    out: List[PositionStep] = []
    for s in step_sizes_rev:
        requested = center_rev + s
        target = bounds.clamp(requested)
        out.append(PositionStep(requested_rev=requested, target_rev=target,
                                clamped=abs(target - requested) > 1e-9))
    return out


def chirp_position_series(t: np.ndarray, center_rev: float, amplitude_rev: float,
                          f_start: float, f_end: float, duration: float,
                          bounds: StrokeBounds) -> Tuple[np.ndarray, float]:
    """Absolute position chirp about ``center``, amplitude clamped to bounds.

    Returns ``(series, amplitude_used)``; the amplitude is first reduced to
    fit the stroke window, then every sample is clamped as a defence-in-depth
    backstop (the reduced amplitude should already fit).
    """
    amp = clamp_amplitude(center_rev, amplitude_rev, bounds)
    series = center_rev + amp * log_chirp(t, f_start, f_end, duration)
    series = np.clip(series, bounds.lo_rev, bounds.hi_rev)
    return series, amp


def max_velocity_step_duration(v_rps: float, stroke_cap_rev: float,
                               frac: float = 0.85) -> float:
    """Longest a velocity step may run before |displacement| risks the stroke.

    Uses |v|·duration as the displacement bound.  A ramped/settling velocity
    step travels *less* than |v|·duration (the acceleration phase covers less
    ground than full speed would), so this over-bounds true travel and stays
    safe — the same conservative rule ``cogging_bench_test.py`` uses.
    """
    v = max(1e-6, abs(v_rps))
    return frac * stroke_cap_rev / v


def predicted_velocity_displacement(v_rps: float, duration_s: float) -> float:
    """Upper-bound displacement (rev) of a velocity step held ``duration``."""
    return abs(v_rps) * duration_s


def directional_room(start_rev: float, bounds: StrokeBounds) -> Tuple[float, float]:
    """``(up_room, down_room)`` from ``start`` to the NEARER bound each way (rev).

    A velocity step runs one direction then returns, so its safe budget is the
    room in EACH direction from where the leg ACTUALLY sits — not the symmetric
    centre budget.  After ``--home`` the leg sits at the ~0.0 end-stop, well below
    the usable ``lo`` bound, so ``down_room`` is 0 and the +v/−v step must be sized
    to 0 (never driven back into the end-stop).  Both are clamped at 0 when the
    start is already past a bound.
    """
    up = max(0.0, bounds.hi_rev - start_rev)
    down = max(0.0, start_rev - bounds.lo_rev)
    return up, down


def velocity_step_room(start_rev: float, bounds: StrokeBounds) -> float:
    """Symmetric room for a +v/−v velocity step from ``start`` — the SMALLER of the
    up/down directional rooms, so neither the extension nor the return excursion can
    cross a bound (the finding: the return stroke from the 0.0 end-stop must not
    drive back into it)."""
    up, down = directional_room(start_rev, bounds)
    return min(up, down)


# ===========================================================================
# Chirp / position-stimulus kinematic caps
# ===========================================================================
#
# Position-domain stimuli (steps, chirp) are streamed as position setpoints, so
# their implied VELOCITY and ACCELERATION are bounded only by the ODrive
# vel_limit and current limit — NOT by the stroke clamp.  A small-amplitude,
# high-frequency chirp can imply a velocity/accel far past the caps (a 0.02 rev,
# 30 Hz chirp implies ~710 rev/s², over the 250 rev/s² brake-resistor cap).  The
# driver must bound the chirp amplitude by these BEFORE it streams.


def chirp_peak_kinematics(amplitude_rev: float, f_end_hz: float
                          ) -> Tuple[float, float]:
    """Peak implied ``(|velocity| rev/s, |accel| rev/s²)`` of a sinusoidal chirp.

    A sine of amplitude ``A`` at frequency ``f`` peaks at ``A·2πf`` velocity and
    ``A·(2πf)²`` accel.  A log chirp reaches its MAX frequency (``f_end``) at the
    sweep end, so ``f_end`` bounds both over the whole sweep.
    """
    w = 2.0 * math.pi * abs(f_end_hz)
    a = abs(amplitude_rev)
    return a * w, a * w * w


def chirp_within_caps(amplitude_rev: float, f_end_hz: float, *,
                      vel_cap_rps: float, accel_cap_rps2: float
                      ) -> Tuple[bool, float, float]:
    """``(ok, peak_vel, peak_accel)`` — whether a chirp's implied kinematics fit
    the caps."""
    pv, pa = chirp_peak_kinematics(amplitude_rev, f_end_hz)
    return (pv <= vel_cap_rps and pa <= accel_cap_rps2), pv, pa


def chirp_amplitude_cap_for_kinematics(f_end_hz: float, *, vel_cap_rps: float,
                                       accel_cap_rps2: float) -> float:
    """Largest chirp amplitude (rev) whose implied peak vel & accel BOTH fit the
    caps.

    ``peak_vel = A·2πf ≤ vel_cap  ⇒  A ≤ vel_cap/(2πf)``;
    ``peak_accel = A·(2πf)² ≤ accel_cap  ⇒  A ≤ accel_cap/(2πf)²``.
    Returns the tighter (binding) of the two — at high ``f_end`` the accel bound
    dominates.
    """
    w = 2.0 * math.pi * max(1e-9, abs(f_end_hz))
    return min(vel_cap_rps / w, accel_cap_rps2 / (w * w))


def chirp_amplitude_cap_for_lead_clamp(f_end_hz: float, *, frame_step_rev: float,
                                       seg_t_s: float) -> float:
    """Largest chirp amplitude (rev) whose peak per-40 Hz-knot position increment stays
    under the lead-clamp frame step — the Path-BRIDGE bound the vel/accel caps miss.

    A sine of amplitude ``A`` at ``f`` peaks at velocity ``A·2πf``; over one knot interval
    ``seg_t`` that implies a per-knot position step ``A·2πf·seg_t``. Bounding it by
    ``frame_step_rev`` (the 0.5×``MAX_LEAD`` step the position steps ramp at) gives the
    chirp the SAME 2× lead-clamp margin the steps have, so the firmware interp lead clamp
    never engages during the sweep and the streamed ``u0`` stays a faithful lock-in
    reference. WHY this is separate from the kinematic cap: ``MAX_LEAD/seg_t`` (0.10/0.025 =
    4.0 rev/s) equals the default ``vel_cap``, so at the vel-bound amplitude the peak
    per-knot step is ~``MAX_LEAD`` exactly — the chirp would ride the clamp with no margin,
    distorting gain/phase, and ``canbridge_config.h`` warns a lead-clamp limit cycle can
    accumulate past ``MAX_DEVIATION`` and latch the guard. Take ``min`` with the kinematic
    cap; on Path DIRECT (no interp, no lead clamp) this bound does not apply.
    """
    denom = 2.0 * math.pi * max(1e-9, abs(f_end_hz)) * max(1e-9, abs(seg_t_s))
    return abs(frame_step_rev) / denom


# ===========================================================================
# Stage 1b — gain-escalation ladder (escalate-until-unstable, auto-backoff)
# ===========================================================================
#
# pos_gain climbs an explicit ladder; at each rung the harness searches
# vel_gain for ζ ≥ target; the state machine below decides escalate vs stop
# vs back-off-to-last-good.  Kept a pure state machine (no CAN, no I/O) so
# every transition is unit-testable.

VEL_INT_RATIO = 125.0  # pos_gain : vel_integrator_gain, per the methodology


def ratio_vel_int(pos_gain: float, ratio: float = VEL_INT_RATIO) -> float:
    """vel_integrator_gain that preserves the pos_gain : vel_int ratio rule."""
    return pos_gain / ratio


@dataclass(frozen=True)
class GainTriple:
    pos_gain: float
    vel_gain: float
    vel_int_gain: float


@dataclass(frozen=True)
class LadderRung:
    pos_gain: float
    vel_int_gain: float
    vel_gain_lo: float
    vel_gain_hi: float


def default_ladder() -> Tuple[LadderRung, ...]:
    """The pos_gain ladder from the methodology's Stage-1b table.

    vel_int = pos_gain / 125 (the ratio rule); vel_gain search range widens
    with pos_gain because a stiffer position loop needs more velocity-loop
    damping to hold ζ ≥ 0.7.
    """
    return (
        LadderRung(25.0, ratio_vel_int(25.0), 0.20, 0.45),
        LadderRung(40.0, ratio_vel_int(40.0), 0.20, 0.50),
        LadderRung(55.0, ratio_vel_int(55.0), 0.30, 0.60),
        LadderRung(70.0, ratio_vel_int(70.0), 0.35, 0.75),
        LadderRung(90.0, ratio_vel_int(90.0), 0.45, 0.90),
    )


def vel_gain_candidates(rung: LadderRung, n: int = 4) -> List[float]:
    """Ascending vel_gain search points across a rung's range.

    Searched low→high so the harness settles on the *least* velocity-loop
    damping that still holds ζ ≥ target (minimises tracking lag)."""
    if n < 1:
        return []
    if n == 1:
        return [rung.vel_gain_lo]
    return list(np.linspace(rung.vel_gain_lo, rung.vel_gain_hi, n))


@dataclass
class LadderDecision:
    action: str                       # 'escalate' | 'stop_ok' | 'backoff'
    gains: Optional[GainTriple]       # winner (stop_ok) / reverted last-good (backoff) / None (escalate)
    rung_index: int
    reason: str


class GainLadder:
    """Escalate-until-unstable state machine with auto-backoff.

    Feed the result of testing the current rung to :meth:`record`; it returns
    the next :class:`LadderDecision`.  On any instability it reverts to the
    last stable triple and stops — it never keeps climbing past onset.
    """

    def __init__(self, rungs: Optional[Sequence[LadderRung]] = None,
                 zeta_target: float = 0.7,
                 bw_clear_hz: Optional[float] = None):
        self.rungs: Tuple[LadderRung, ...] = tuple(rungs) if rungs else default_ladder()
        self.zeta_target = zeta_target
        self.bw_clear_hz = bw_clear_hz
        self.index = 0
        self.last_good: Optional[GainTriple] = None
        self.stopped = False
        self.stop_reason: Optional[str] = None
        self.history: List[dict] = []

    def current_rung(self) -> LadderRung:
        return self.rungs[self.index]

    def record(self, tested: GainTriple, onset: OnsetResult,
               zeta: Optional[float], bandwidth_hz: Optional[float] = None,
               stable_vel_gain_found: bool = True) -> LadderDecision:
        """Advance the ladder given one rung's test result.

        ``tested`` is the triple actually applied (the best vel_gain the
        harness found for this rung).  ``stable_vel_gain_found`` is False when
        no vel_gain in the rung's range held ζ ≥ target — itself an onset (the
        pos_gain ceiling).
        """
        self.history.append({
            'index': self.index, 'tested': tested,
            'unstable': onset.unstable, 'zeta': zeta,
            'bandwidth_hz': bandwidth_hz,
            'stable_vel_gain_found': stable_vel_gain_found,
        })

        zeta_bad = zeta is not None and zeta < self.zeta_target
        if onset.unstable or not stable_vel_gain_found or zeta_bad:
            self.stopped = True
            if onset.unstable:
                self.stop_reason = 'instability_onset'
            elif not stable_vel_gain_found:
                self.stop_reason = 'pos_gain_ceiling_no_stable_vel_gain'
            else:
                self.stop_reason = 'zeta_below_target'
            return LadderDecision('backoff', self.last_good, self.index,
                                  self.stop_reason)

        # Stable at this rung — bank it as the new last-good.
        self.last_good = tested

        if (self.bw_clear_hz is not None and bandwidth_hz is not None
                and bandwidth_hz >= self.bw_clear_hz):
            self.stopped = True
            self.stop_reason = 'bandwidth_cleared'
            return LadderDecision('stop_ok', tested, self.index,
                                  self.stop_reason)

        if self.index + 1 < len(self.rungs):
            self.index += 1
            return LadderDecision('escalate', None, self.index,
                                  'stable, climbing to next rung')

        self.stopped = True
        self.stop_reason = 'ladder_top'
        return LadderDecision('stop_ok', tested, self.index, self.stop_reason)


# ===========================================================================
# Path BRIDGE — position-domain knot stimuli through the 500 Hz Hermite
# ===========================================================================
#
# On Path BRIDGE (the can-bridge Teensy over UDP teensy_link) the harness cannot
# issue a raw instantaneous step to ``input_pos``: every command is a 40 Hz knot
# the firmware cubic-Hermite interpolates at 500 Hz (``leg_interp.cpp`` Mode 1),
# and a per-500 Hz-tick commanded-minus-encoder LEAD over ``MAX_LEAD_REV`` is
# clamped (``lead_clamp_mask`` sets). A bridge "step" is therefore a 25 ms-
# quantised knot RAMP, sized so each knot increment stays under the lead clamp —
# the leg tracks it as a fast ramp, the honest bridge analogue of a step. These
# generators are pure (numeric only, no protocol/CAN import) so the Setpoint
# packing + streaming stays in the driver and the sizing math is unit-testable.


def lead_clamp_frame_step(max_lead_rev: float, margin_frac: float = 0.5) -> float:
    """Largest per-40 Hz-knot position increment that keeps the firmware interp
    lead clamp (``MAX_LEAD_REV``) from engaging, with a safety margin.

    The Teensy clamps the commanded interp output to encoder ± ``MAX_LEAD_REV``
    every 500 Hz tick; a knot ramp whose per-frame increment is ``margin_frac``·
    ``MAX_LEAD`` keeps the accumulated lead under the clamp while the leg tracks.
    ``margin_frac = 0.5`` (2× margin below the 0.10 rev clamp — the methodology's
    Stage-1 lead target) gives 0.05 rev/frame = **2.0 rev/s** at the 40 Hz knot
    rate, which is also Jugglebot's ~2 rev/s operating point (deliberate).
    """
    if not (0.0 < margin_frac <= 1.0):
        raise ValueError("margin_frac must be in (0, 1]")
    return margin_frac * abs(max_lead_rev)


def knot_ramp_frames(start_rev: float, target_rev: float,
                     frame_step_rev: float) -> int:
    """Number of 40 Hz knots to ramp ``start → target`` at ≤ ``frame_step_rev``
    each. ``0`` when already there.

    The ceil is fp-robust: a step that is an EXACT integer multiple of
    ``frame_step`` (e.g. 0.10 rev at a 0.05 rev step) must take exactly 2 frames,
    not 3 — without the ``1e-9`` slack, ``0.10/0.05`` floating-point-rounds to
    2.0000000000000004 and ceils to 3, silently adding a frame to every clean step.
    """
    if frame_step_rev <= 0.0:
        raise ValueError("frame_step_rev must be > 0")
    delta = abs(target_rev - start_rev)
    if delta <= 0.0:
        return 0
    return int(math.ceil(delta / frame_step_rev - 1e-9))


def knot_step_ramp(start_rev: float, target_rev: float, *,
                   frame_step_rev: float, hold_frames: int,
                   bounds: Optional[StrokeBounds] = None) -> np.ndarray:
    """40 Hz knot ``u0`` series that ramps ``start → target`` at ≤ ``frame_step_rev``
    per knot, then holds ``target`` for ``hold_frames``.

    The ramp is evenly spaced over ``ceil(|Δ|/frame_step)`` frames, so every knot
    step is ``|Δ|/n_ramp ≤ frame_step_rev`` — guaranteed under the lead clamp.
    Index 0 is ``start`` (so the first streamed knot commands the current position:
    no jump at arm). Every knot is clamped to ``bounds`` (defence-in-depth stroke
    cap) when given; callers pass a ``target`` already inside bounds.

    The defence-in-depth clip is widened to preserve an out-of-bounds ``start``:
    post ``--home`` the leg sits AT the ~0.0 end-stop, which is BELOW the usable
    ``lo`` (the margin backs off the end-stop). A blanket ``clip(lo, hi)`` would
    flatten the sub-``lo`` approach knots up to ``lo`` — turning arm into a jump
    from the true encoder pos to ``lo`` (e.g. 0.0 → 0.15 rev, 3× the 0.05 rev
    lead-clamp budget the ramp is sized for), the very jump the "index 0 = start"
    invariant exists to avoid. Clamping to ``[min(lo,start), max(hi,start)]``
    instead keeps the first knot on the true start and lets the ramp climb into
    the window one lead-clamp step at a time, while STILL capping the far
    (target) side at the stroke bound — commanding where the leg physically sits
    can never crash it, so the near-side relaxation costs no protection.
    """
    if hold_frames < 0:
        raise ValueError("hold_frames must be >= 0")
    n_ramp = knot_ramp_frames(start_rev, target_rev, frame_step_rev)
    seq: List[float] = [float(start_rev)]
    for i in range(1, n_ramp + 1):
        seq.append(float(start_rev + (target_rev - start_rev) * (i / n_ramp)))
    seq.extend([float(target_rev)] * int(hold_frames))
    arr = np.asarray(seq, float)
    if bounds is not None:
        lo = min(bounds.lo_rev, float(start_rev))
        hi = max(bounds.hi_rev, float(start_rev))
        arr = np.clip(arr, lo, hi)
    return arr


def max_frame_step(series: Sequence[float]) -> float:
    """Largest absolute knot-to-knot step in a series (for lead-clamp assertions)."""
    arr = np.asarray(series, float)
    if arr.size < 2:
        return 0.0
    return float(np.max(np.abs(np.diff(arr))))


@dataclass
class BridgeStepPlan:
    requested_rev: float      # center + step, before clamping
    target_rev: float         # actually commanded (clamped into bounds)
    clamped: bool             # True if the requested target hit a stroke bound
    ramp_frames: int          # 40 Hz knots in the ramp (excl. the start knot)
    ramp_duration_s: float    # ramp_frames · seg_t_s
    peak_frame_step_rev: float  # actual per-knot increment (≤ frame_step_rev)


def bridge_step_plan(center_rev: float, step_sizes_rev: Sequence[float],
                     bounds: StrokeBounds, *, frame_step_rev: float,
                     seg_t_s: float) -> List[BridgeStepPlan]:
    """Per-step bridge knot-ramp plan: stroke-clamped target + ramp length/duration.

    Reuses :func:`position_step_series` for the stroke clamp, then sizes the knot
    ramp for each step. A sub-``frame_step`` step is a single knot; a big step
    ramps over several. The peak per-knot increment is reported so a caller/test
    can confirm it never exceeds ``frame_step_rev`` (hence the lead clamp).
    """
    out: List[BridgeStepPlan] = []
    for ps in position_step_series(center_rev, step_sizes_rev, bounds):
        frames = knot_ramp_frames(center_rev, ps.target_rev, frame_step_rev)
        delta = abs(ps.target_rev - center_rev)
        peak = (delta / frames) if frames > 0 else 0.0
        out.append(BridgeStepPlan(
            requested_rev=ps.requested_rev, target_rev=ps.target_rev,
            clamped=ps.clamped, ramp_frames=frames,
            ramp_duration_s=frames * seg_t_s, peak_frame_step_rev=peak))
    return out


# ---------------------------------------------------------------------------
# Telemetry-rate estimator (bounds the honest chirp top frequency)
# ---------------------------------------------------------------------------
#
# On Path BRIDGE the ONLY observation of the leg is the UDP TELEMETRY stream
# (nominal 100 Hz, all 7 axes per frame — so the per-axis rate == the frame rate).
# Its measured rate and irregularity bound how high a chirp we can honestly
# resolve: a lock-in gain/phase estimate needs several samples per period, so the
# top usable chirp frequency is the (worst-case) sample rate / samples-per-period.
# The driver MEASURES this over a warmup window BEFORE any stimulus and clamps the
# chirp f1 to it — an irregular uplink honestly lowers the ceiling.


@dataclass
class TelemetryRate:
    n_intervals: int
    mean_hz: float
    median_hz: float
    jitter_rms_s: float       # RMS of (interval − mean interval)
    p95_interval_s: float     # 95th-percentile gap (the irregularity tail)
    max_interval_s: float
    effective_hz: float       # 1 / p95_interval — the conservative worst-case rate


def estimate_telemetry_rate(timestamps_s: Sequence[float]) -> TelemetryRate:
    """Frame rate + jitter from a list of arrival timestamps (seconds, monotonic).

    Non-monotone / duplicate stamps (Δt ≤ 0) are dropped rather than producing a
    spurious infinite rate. ``effective_hz`` uses the 95th-percentile gap, so a
    stream with occasional long stalls reports a rate that reflects the stalls —
    the honest number to bound a chirp against, not the optimistic mean.
    """
    ts = np.asarray(timestamps_s, float)
    if ts.size < 2:
        return TelemetryRate(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    dt = np.diff(ts)
    dt = dt[dt > 0.0]
    if dt.size == 0:
        return TelemetryRate(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    mean_dt = float(np.mean(dt))
    median_dt = float(np.median(dt))
    p95 = float(np.percentile(dt, 95))
    mx = float(np.max(dt))
    jitter = float(np.sqrt(np.mean((dt - mean_dt) ** 2)))
    return TelemetryRate(
        n_intervals=int(dt.size),
        mean_hz=(1.0 / mean_dt) if mean_dt > 0 else 0.0,
        median_hz=(1.0 / median_dt) if median_dt > 0 else 0.0,
        jitter_rms_s=jitter,
        p95_interval_s=p95,
        max_interval_s=mx,
        effective_hz=(1.0 / p95) if p95 > 0 else 0.0,
    )


def honest_chirp_top_freq(effective_hz: float, *,
                          min_samples_per_period: float = 8.0) -> float:
    """Highest chirp frequency the MEASURED telemetry can resolve for a single-bin
    lock-in gain/phase estimate: ``effective_hz / min_samples_per_period``.

    8 samples/period keeps lock-in leakage small; using the conservative
    ``effective_hz`` (from the p95 gap) means an irregular uplink honestly lowers
    the ceiling instead of overstating it.
    """
    if effective_hz <= 0.0 or min_samples_per_period <= 0.0:
        return 0.0
    return effective_hz / min_samples_per_period


def knot_stream_top_freq(seg_t_s: float, *,
                         min_samples_per_period: float = 5.0) -> float:
    """Highest chirp frequency the 40 Hz knot STREAM can honestly COMMAND:
    ``(1/seg_t) / min_samples_per_period``.

    The Teensy Hermite-interpolates between 40 Hz knots, so a chirp above knot-
    Nyquist (20 Hz at seg_t=0.025) aliases; keeping ≥5 knots/period bounds the
    command distortion. At seg_t=0.025 this is 8 Hz — so the bridge honestly
    reaches the 6 Hz pos-loop question but NOT the 30 Hz the direct path covers.
    """
    if seg_t_s <= 0.0 or min_samples_per_period <= 0.0:
        return 0.0
    return (1.0 / seg_t_s) / min_samples_per_period


def bridge_chirp_top_freq(requested_f1_hz: float, telem_effective_hz: float,
                          seg_t_s: float, *,
                          telem_samples_per_period: float = 8.0,
                          knot_samples_per_period: float = 5.0) -> float:
    """The binding chirp f1 on Path BRIDGE: the min of the operator request, the
    telemetry-resolution bound (response side), and the knot-stream bound (command
    side). This is the honest top frequency the bridge chirp can claim.
    """
    return min(
        float(requested_f1_hz),
        honest_chirp_top_freq(telem_effective_hz,
                              min_samples_per_period=telem_samples_per_period),
        knot_stream_top_freq(seg_t_s, min_samples_per_period=knot_samples_per_period),
    )


# ===========================================================================
# Guard-latch backoff state machine (Path BRIDGE — never fight the guard)
# ===========================================================================
#
# On Path BRIDGE the firmware safety machine stays armed underneath the harness:
# the per-leg MAX_DEVIATION E-STOP (0.5 rev) and the MPC_CMD staleness E-STOP are
# the BACKSTOP the harness must respect, not defeat. When a stimulus is too hot
# the guard latches (fault_state != NONE on the T→J heartbeat); the harness must
# detect it, revert to the last-good gains, and recover via a CLEAR_ERRORS RPC
# (the firmware re-enable slew makes that gentle) — NOT keep pushing. This is a
# pure classifier + bounded-recovery state machine so the transport-free logic is
# unit-testable; the driver supplies the observations and performs the RPCs.
#
# FaultState wire codes duplicated as plain ints so this module stays dependency-
# free (numpy + stdlib only, per the module docstring); they are cross-checked
# against the generated ``FaultState`` enum in
# ``tests/motion/test_bench_sysid_bridge.py``.
FAULT_NONE = 0
FAULT_MPC_STALE = 1
FAULT_LINK_LOST = 2
FAULT_MOTOR_OVERSPEED = 3
FAULT_MAX_DEVIATION = 4
FAULT_ODRIVE_FATAL = 5
FAULT_CAN_BUS_DOWN = 6
FAULT_MOTOR_FB_STALE = 7

FAULT_NAMES = {
    FAULT_NONE: 'NONE', FAULT_MPC_STALE: 'MPC_STALE', FAULT_LINK_LOST: 'LINK_LOST',
    FAULT_MOTOR_OVERSPEED: 'MOTOR_OVERSPEED', FAULT_MAX_DEVIATION: 'MAX_DEVIATION',
    FAULT_ODRIVE_FATAL: 'ODRIVE_FATAL', FAULT_CAN_BUS_DOWN: 'CAN_BUS_DOWN',
    FAULT_MOTOR_FB_STALE: 'MOTOR_FB_STALE',
}

# Recoverable: the firmware self-recovers once the stream/feedback returns — keep
# streaming (they fire during a link/feedback blip, not a too-hot stimulus). ONLY
# MOTOR_FB_STALE and LINK_LOST self-recover: fault_machine.cpp:388-421 sets
# MOTOR_FB_STALE from the live ``fb_stale`` (output re-enables when feedback returns)
# and LINK_LOST from the instantaneous ``jetson_link_up()`` — NEITHER latched.
_FAULT_RECOVERABLE = frozenset({FAULT_LINK_LOST, FAULT_MOTOR_FB_STALE})
# Latching: a guard E-STOP crossed — revert + CLEAR_ERRORS to recover. ALL THREE of
# MOTOR_OVERSPEED / MPC_STALE / MAX_DEVIATION latch (sticky ``s_estop_latched``,
# fault_machine.cpp:69-80,403-418): guard_mode holds ESTOP and the 500 Hz output stays
# gated off until an EXPLICIT ``fault_notify_clear_errors()``. MPC_STALE is reachable in
# normal operation — a blocking RPC gain-apply (two SET_*_GAIN calls, up to ~1.5 s with
# retries) between streams can straddle the 250 ms MPC_CMD_STALENESS window while armed;
# the driver disarms across gain application (``mpc_active=0`` suppresses the staleness
# check) so it does not spuriously latch, and a genuine MPC_STALE latch then runs the
# same back-off + CLEAR_ERRORS recovery as MAX_DEVIATION rather than a passive watch.
_FAULT_LATCHING = frozenset({FAULT_MPC_STALE, FAULT_MAX_DEVIATION, FAULT_MOTOR_OVERSPEED})
# Fatal: cede authority so the firmware deferred stow / fatal handling runs alone.
_FAULT_FATAL = frozenset({FAULT_ODRIVE_FATAL, FAULT_CAN_BUS_DOWN})


def fault_name(code: int) -> str:
    return FAULT_NAMES.get(int(code), "FAULT_%d" % int(code))


def classify_fault(fault_state: int) -> str:
    """'none' | 'recoverable' | 'latching' | 'fatal'. Unknown codes → 'fatal'
    (conservative: an unclassified fault is treated as un-recoverable)."""
    fs = int(fault_state)
    if fs == FAULT_NONE:
        return 'none'
    if fs in _FAULT_RECOVERABLE:
        return 'recoverable'
    if fs in _FAULT_LATCHING:
        return 'latching'
    return 'fatal'


@dataclass
class GuardAction:
    kind: str              # 'continue' | 'watch' | 'backoff_recover' | 'abort'
    revert: bool           # revert to last-good gains + disarm (mpc_active=0)
    clear_errors: bool     # issue a CLEAR_ERRORS RPC to recover the latch
    classification: str    # the classify_fault() bucket
    reason: str


class GuardLatchBackoff:
    """Turn a stream of firmware ``fault_state`` observations into harness actions,
    with a bounded recovery budget so the harness NEVER fights a guard forever.

    Call :meth:`observe` once per control tick with the latest heartbeat
    ``fault_state``. Semantics (edge-triggered on latches so one latch → one
    recovery, not one-per-tick while it stays latched):

    * ``none``        → ``continue``.
    * ``recoverable`` (LINK_LOST / MOTOR_FB_STALE) → ``watch`` (the firmware
      self-recovers when the stream/feedback returns); but if the SAME recoverable
      fault persists past ``recoverable_grace`` consecutive ticks the link itself is
      broken → ``abort``.
    * ``latching``    (MPC_STALE / MAX_DEVIATION / MOTOR_OVERSPEED) → on the rising edge,
      ``backoff_recover`` (revert to last-good, disarm, CLEAR_ERRORS). While it
      stays latched awaiting our clear → ``watch`` (do not re-fire). The
      ``(max_recoveries+1)``-th distinct latch → ``abort`` (stop climbing into a
      guard we cannot satisfy).
    * ``fatal``       (ODRIVE_FATAL / CAN_BUS_DOWN) → ``abort`` immediately, revert
      but NO clear (cede authority — the firmware's deferred stow runs uncontested).
    """

    def __init__(self, max_recoveries: int = 2, recoverable_grace: int = 20):
        self.max_recoveries = int(max_recoveries)
        self.recoverable_grace = int(recoverable_grace)
        self.recoveries_used = 0
        self._recoverable_run = 0
        self._prev_class = 'none'
        self.history: List[dict] = []

    def observe(self, fault_state: int) -> GuardAction:
        cls = classify_fault(fault_state)
        prev = self._prev_class
        self._prev_class = cls

        if cls == 'none':
            self._recoverable_run = 0
            act = GuardAction('continue', False, False, cls, 'nominal')
        elif cls == 'recoverable':
            self._recoverable_run += 1
            if self._recoverable_run > self.recoverable_grace:
                act = GuardAction(
                    'abort', True, False, cls,
                    "%s persisted > %d ticks (link broken, not stimulus)"
                    % (fault_name(fault_state), self.recoverable_grace))
            else:
                act = GuardAction(
                    'watch', False, False, cls,
                    "%s (firmware self-recovers)" % fault_name(fault_state))
        elif cls == 'latching':
            self._recoverable_run = 0
            if prev == 'latching':
                # Still latched, awaiting our CLEAR_ERRORS — don't double-count.
                act = GuardAction('watch', False, False, cls,
                                  "%s latch pending recovery" % fault_name(fault_state))
            else:
                self.recoveries_used += 1
                if self.recoveries_used > self.max_recoveries:
                    act = GuardAction(
                        'abort', True, False, cls,
                        "%s latched %dx (> %d recovery budget) — stop"
                        % (fault_name(fault_state), self.recoveries_used,
                           self.max_recoveries))
                else:
                    act = GuardAction(
                        'backoff_recover', True, True, cls,
                        "%s latch → back off to last-good + CLEAR_ERRORS "
                        "(recovery %d/%d)" % (fault_name(fault_state),
                                              self.recoveries_used,
                                              self.max_recoveries))
        else:  # fatal / unknown
            self._recoverable_run = 0
            act = GuardAction(
                'abort', True, False, cls,
                "%s fatal → cede authority (no clear)" % fault_name(fault_state))

        self.history.append({'fault_state': int(fault_state),
                             'classification': cls, 'action': act.kind})
        return act


# ===========================================================================
# Startup guard-latch clearance + stream-then-arm warmup (Path BRIDGE)
# ===========================================================================
#
# The firmware guard E-STOP latch (``s_estop_latched``) is STICKY: it survives a
# harness restart because the can-bridge Teensy runs on Jetson 5V and is never
# power-cycled between runs, and it is released ONLY by an explicit CLEAR_ERRORS
# (fault_machine.cpp:180-186). So a MAX_DEVIATION / MPC_STALE / MOTOR_OVERSPEED
# latch from ANY prior run makes every later run read fault_state != NONE from the
# first heartbeat — the approach stream observes it on iteration 0, backs off, and
# aborts "during approach: guard latch" with the leg in CLOSED_LOOP but never
# moving (output stays gated by the sticky latch while the interp base runs away).
# The harness must therefore verify-and-clear a pre-existing latch at startup,
# DISARMED, before any stage. These helpers are pure so the decision + sizing are
# unit-testable; the driver performs the socket I/O and the RPC.

BRIDGE_ARM_WARMUP_FRAMES = 16    # 40 Hz knots (0.4 s) streamed DISARMED before EVERY
                                 # arm: freshens the firmware MPC-staleness clock and
                                 # re-baselines the interp base to the encoder.
BRIDGE_CLEAR_DEV_TOL_REV = 0.10  # after the warmup re-baseline the interp base must be
                                 # within this of the encoder before a startup
                                 # CLEAR_ERRORS — the 0.10 rev lead clamp, a safe
                                 # re-enable delta well under MAX_DEVIATION (0.5 rev).


def warmup_knot_series(pos_rev: float, n_frames: int = BRIDGE_ARM_WARMUP_FRAMES
                       ) -> np.ndarray:
    """Flat 40 Hz knot series holding ``pos_rev`` for ``n_frames`` — the DISARMED
    stream-then-arm warmup.

    Streamed with mpc_active=0 it (a) freshens the firmware MPC-staleness clock
    (``interp_on_setpoint`` stamps ``s_last_setpoint_us`` on EVERY accepted frame,
    regardless of arm — leg_interp.cpp:196) so the FOLLOWING arm cannot race the
    250 ms MPC_STALE E-STOP, and (b) re-baselines the interp base to the measured
    encoder so arming causes no jump and no MAX_DEVIATION. Output is gated while
    disarmed, so these knots command NO motion — this is why streaming FIRST then
    arming (the ``teensy_setpoint_bench.py`` idiom) is safe, and arming FIRST then
    streaming is the race the operator's abort came from.
    """
    if n_frames < 1:
        raise ValueError("n_frames must be >= 1")
    return np.full(int(n_frames), float(pos_rev), dtype=float)


def deviation_within_clear_tol(live_deviation_rev: float,
                               tol_rev: float = BRIDGE_CLEAR_DEV_TOL_REV) -> bool:
    """True when the live interp-base-minus-encoder deviation is small enough that a
    DISARMED CLEAR_ERRORS + later re-arm won't lurch the leg (u0 ≈ encoder).

    A fresh MAX_DEVIATION latch has u0 far from the encoder (that IS why it
    latched); the driver first streams a disarmed warmup at the encoder to
    re-baseline u0, then calls this to CONFIRM the base tracked before clearing.
    """
    return abs(float(live_deviation_rev)) <= abs(float(tol_rev))


@dataclass
class StartupLatchPlan:
    action: str          # 'proceed' | 'clear' | 'wait_recover' | 'abort'
    classification: str  # the classify_fault() bucket
    reason: str


def plan_startup_latch(fault_state: int) -> StartupLatchPlan:
    """Decide what a startup ``fault_state`` demands, BEFORE any stage arms.

    * ``none``        → ``proceed`` (already clear to arm).
    * ``latching``    (MAX_DEVIATION / MPC_STALE / MOTOR_OVERSPEED) → ``clear``: a
      sticky guard E-STOP persists across restarts and blocks every run; the driver
      re-baselines + verifies u0≈enc (disarmed) then issues CLEAR_ERRORS.
    * ``recoverable`` (LINK_LOST / MOTOR_FB_STALE) → ``wait_recover``: the firmware
      self-recovers when the stream/feedback returns; wait briefly for NONE.
    * ``fatal``       (ODRIVE_FATAL / CAN_BUS_DOWN) → ``abort``: the ODrive is
      unpowered / has active errors, or CAN3 is down — do NOT clear a fatal; the
      operator must power + clear the ODrive first. (Unknown codes → fatal, so an
      unclassified state is never auto-cleared.)
    """
    cls = classify_fault(fault_state)
    name = fault_name(fault_state)
    if cls == 'none':
        return StartupLatchPlan('proceed', cls, 'fault_state NONE — clear to arm')
    if cls == 'latching':
        return StartupLatchPlan(
            'clear', cls,
            "%s latched at startup (sticky across restarts) — verify u0≈enc "
            "disarmed, then CLEAR_ERRORS" % name)
    if cls == 'recoverable':
        return StartupLatchPlan(
            'wait_recover', cls,
            "%s at startup — waiting for firmware self-recovery" % name)
    return StartupLatchPlan(
        'abort', cls,
        "%s at startup — the bench ODrive is unpowered / has active errors, or "
        "CAN3 is down; power and clear the ODrive, then retry (the harness will "
        "NOT clear a fatal)" % name)
