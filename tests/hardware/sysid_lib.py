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
