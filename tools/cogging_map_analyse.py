#!/usr/bin/env python3
"""Offline cogging-torque-map analyser for the bench-rig CSVs.

Consumes ``temp/logs/cogging_*.csv`` files produced by
``tests/hardware/cogging_bench_test.py`` and looks for:

  1. Electrical-angle-locked torque ripple (the cogging signature).
  2. Mechanical-angle-locked torque ripple (non-uniform magnet spacing,
     single-tooth manufacturing faults).
  3. Position-dependent bias (gravity, spring return, end-stop crowding).
  4. Velocity-dependent drift (Stribeck-style friction rise or fall).
  5. Forward-vs-reverse hysteresis (direction-dependent friction band).

The bench runner logs at 100 Hz.  At 0.05 rev/s a single cogging period
(1/84 mech rev) crosses in ~0.238 s ≈ 24 samples — enough per period for
averaging across many revs but too few to resolve fine structure within
one period.  The analyser therefore averages all samples falling into a
fixed grid of electrical-angle bins over the steady-state window.

Usage
-----
    # Analyse one CSV
    python tools/cogging_map_analyse.py temp/logs/cogging_20260424_230556_fwd_0.050rps.csv

    # Compare forward+reverse back-to-back (hysteresis check)
    python tools/cogging_map_analyse.py \\
        temp/logs/cogging_20260424_230556_fwd_0.050rps.csv \\
        temp/logs/cogging_20260424_230556_rev_0.050rps.csv

    # Skip plots / write JSON
    python tools/cogging_map_analyse.py <csv> --no-plot --json report.json

Verdict thresholds (on electrical-angle-binned iq ripple, after mean-
removal):

    peak-to-peak < 30 mA   → cogging negligible, demote
    30–100 mA              → cogging contributing
    > 100 mA               → cogging material, compensate in motor_guard
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd


_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_SCRIPT_DIR)


# ---------------------------------------------------------------------------
# Tunables
# ---------------------------------------------------------------------------

# Drop the first/last N seconds from analysis — the ramp-in and ramp-out from
# the bench runner contain non-steady-state iq.
STARTUP_TRIM_S = 2.0
SHUTDOWN_TRIM_S = 1.0

# Velocity tolerance: only include samples where |vel - target| < this fraction
# of the target velocity.  Keeps the "was the motor actually at target velocity"
# gate strict.
VEL_TOL_FRAC = 0.3

# Electrical-angle bin count.  360 gives 1-degree resolution, which is finer
# than the 84-cycle cogging fundamental (period = 360°/84 ≈ 4.3°) needs.
ELEC_BINS = 360
MECH_BINS = 360

# Harmonics to probe.  6 = LCM(7,12)/NCD redundancy check; 7 = pole pairs;
# 12 = stator slots; 14 = magnetic cycle per mech rev; 84 = cogging fundamental.
ELEC_HARMONICS = (1, 2, 3, 4, 6, 12)  # in units of "cycles per electrical rev"
MECH_HARMONICS = (1, 2, 6, 7, 12, 14, 42, 84)  # cycles per mech rev

# Cogging verdict thresholds (mA peak-to-peak on the electrical-angle map,
# after mean removal).
VERDICT_NEGLIGIBLE_MA = 30.0
VERDICT_MATERIAL_MA = 100.0


# ---------------------------------------------------------------------------
# Per-session analysis
# ---------------------------------------------------------------------------

@dataclass
class SessionStats:
    path: str
    n_samples_total: int
    n_samples_used: int
    t_window_s: Tuple[float, float]
    target_vel_rps: float
    mean_vel_rps: float
    vel_std_rps: float
    mean_iq_A: float
    iq_std_A: float
    n_mech_revs_covered: float
    # Electrical-angle-binned map
    elec_bin_centres_rad: np.ndarray = field(repr=False)
    elec_bin_iq_mean_A: np.ndarray = field(repr=False)
    elec_bin_counts: np.ndarray = field(repr=False)
    # Mechanical-angle-binned map
    mech_bin_centres_rad: np.ndarray = field(repr=False)
    mech_bin_iq_mean_A: np.ndarray = field(repr=False)
    elec_ripple_pkpk_mA: float = 0.0
    elec_harmonics: Dict[int, Dict[str, float]] = field(default_factory=dict)
    mech_ripple_pkpk_mA: float = 0.0
    mech_harmonics: Dict[int, Dict[str, float]] = field(default_factory=dict)
    # Position-dependent drift (linear fit iq vs pos_rev)
    pos_drift_slope_A_per_rev: float = 0.0
    pos_drift_r2: float = 0.0
    # Verdict
    verdict: str = ''


def _bin_by_angle(angles_rad: np.ndarray, values: np.ndarray,
                  n_bins: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Bin ``values`` by ``angles_rad`` modulo 2π into ``n_bins`` equal bins.

    Returns (bin_centres, mean_per_bin, count_per_bin).  Empty bins get NaN.
    """
    two_pi = 2.0 * math.pi
    ang = angles_rad % two_pi
    bin_width = two_pi / n_bins
    idx = np.clip((ang / bin_width).astype(int), 0, n_bins - 1)
    counts = np.bincount(idx, minlength=n_bins).astype(float)
    sums = np.bincount(idx, weights=values, minlength=n_bins)
    with np.errstate(invalid='ignore', divide='ignore'):
        means = np.where(counts > 0, sums / counts, np.nan)
    centres = (np.arange(n_bins) + 0.5) * bin_width
    return centres, means, counts


def _fit_harmonic(theta: np.ndarray, y: np.ndarray, n: int) -> Dict[str, float]:
    """Fit ``y ≈ A cos(n·θ) + B sin(n·θ) + C``.

    Returns {'r2', 'amplitude_A', 'phase_rad', 'offset_A'}.
    """
    mask = np.isfinite(y)
    if mask.sum() < 4:
        return {'r2': 0.0, 'amplitude_A': 0.0, 'phase_rad': 0.0, 'offset_A': 0.0}
    t = theta[mask]
    yv = y[mask]
    X = np.column_stack([np.cos(n * t), np.sin(n * t), np.ones_like(t)])
    coef, *_ = np.linalg.lstsq(X, yv, rcond=None)
    y_hat = X @ coef
    ss_res = float(((yv - y_hat) ** 2).sum())
    ss_tot = float(((yv - yv.mean()) ** 2).sum())
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    amp = float(math.hypot(coef[0], coef[1]))
    phase = float(math.atan2(coef[1], coef[0]))
    return {'r2': round(r2, 3),
            'amplitude_A': round(amp, 4),
            'amplitude_mA': round(amp * 1000.0, 1),
            'phase_rad': round(phase, 3),
            'offset_A': round(float(coef[2]), 4)}


def _pos_linear_fit(pos: np.ndarray, iq: np.ndarray) -> Tuple[float, float]:
    if len(pos) < 4:
        return 0.0, 0.0
    p = pos - pos.mean()
    y = iq - iq.mean()
    denom = float((p * p).sum())
    if denom == 0.0:
        return 0.0, 0.0
    slope = float((p * y).sum() / denom)
    y_hat = slope * p
    ss_res = float(((y - y_hat) ** 2).sum())
    ss_tot = float((y * y).sum())
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    return slope, r2


def _parse_target_from_filename(path: str) -> Optional[float]:
    """Parse target velocity from filenames like ``cogging_<ts>_fwd_0.500rps.csv``.

    Returns signed velocity (negative for ``rev`` runs) or None on parse fail.
    The filename is the canonical source of truth — at high velocities the run
    can be entirely ramp, so cmd_vel statistics are an unreliable proxy.
    """
    import re
    name = os.path.basename(path)
    m = re.search(r'_(fwd|rev)_(\d+(?:\.\d+)?)rps\.csv$', name)
    if not m:
        return None
    sign = 1.0 if m.group(1) == 'fwd' else -1.0
    return sign * float(m.group(2))


def analyse_session(path: str) -> SessionStats:
    df = pd.read_csv(path)

    # Target velocity: prefer filename (canonical), fall back to cmd_vel median.
    # Filename is canonical because at high target velocities the auto-duration
    # may be too short to reach steady-state, making cmd_vel statistics
    # dominated by ramp values rather than the intended target.
    target = _parse_target_from_filename(path)
    if target is None:
        target = float(df['cmd_vel_rps'].abs().median() * math.copysign(1.0,
                       df['cmd_vel_rps'].sum()))
        if abs(target) < 1e-4:
            target = float(df['vel_rps_raw'].mean())

    # Steady-state window
    t = df['t_s'].values
    t_end = t[-1]
    t_lo = STARTUP_TRIM_S
    t_hi = max(STARTUP_TRIM_S + 1.0, t_end - SHUTDOWN_TRIM_S)
    window_mask = (t >= t_lo) & (t <= t_hi)

    # Velocity gate: only use samples where |vel_raw - target| < tol
    vel_raw = df['vel_rps_raw'].values
    vel_tol = max(0.005, abs(target) * VEL_TOL_FRAC)
    vel_mask = np.abs(vel_raw - target) < vel_tol

    used_mask = window_mask & vel_mask
    n_used = int(used_mask.sum())

    # If the velocity gate rejects nearly everything, fall back to the
    # window-only mask so we don't return zero stats.  Noisy velocity traces
    # on a compliant leg are common.
    if n_used < 50:
        used_mask = window_mask
        n_used = int(used_mask.sum())

    iq = df['iq_measured_A'].values[used_mask]
    pos = df['pos_rev_raw'].values[used_mask]
    elec = df['elec_angle_rad'].values[used_mask]
    mech = df['mech_angle_rad'].values[used_mask]
    vel = vel_raw[used_mask]

    mean_iq = float(np.mean(iq)) if iq.size else 0.0
    iq_std = float(np.std(iq)) if iq.size else 0.0

    # Electrical-angle bin
    elec_centres, elec_means, elec_counts = _bin_by_angle(elec, iq, ELEC_BINS)
    elec_mean_removed = elec_means - np.nanmean(elec_means)
    elec_pkpk_mA = (float(np.nanmax(elec_mean_removed) - np.nanmin(elec_mean_removed))
                    * 1000.0 if np.any(np.isfinite(elec_mean_removed)) else 0.0)

    # Mechanical-angle bin
    mech_centres, mech_means, _ = _bin_by_angle(mech, iq, MECH_BINS)
    mech_mean_removed = mech_means - np.nanmean(mech_means)
    mech_pkpk_mA = (float(np.nanmax(mech_mean_removed) - np.nanmin(mech_mean_removed))
                    * 1000.0 if np.any(np.isfinite(mech_mean_removed)) else 0.0)

    # Harmonic fits on the binned means (less noisy than on raw samples)
    elec_h = {n: _fit_harmonic(elec_centres, elec_mean_removed, n)
              for n in ELEC_HARMONICS}
    mech_h = {n: _fit_harmonic(mech_centres, mech_mean_removed, n)
              for n in MECH_HARMONICS}

    pos_slope, pos_r2 = _pos_linear_fit(pos, iq)

    # Verdict
    if elec_pkpk_mA < VERDICT_NEGLIGIBLE_MA:
        verdict = f'NEGLIGIBLE cogging (pk-pk {elec_pkpk_mA:.1f} mA < {VERDICT_NEGLIGIBLE_MA:.0f})'
    elif elec_pkpk_mA < VERDICT_MATERIAL_MA:
        verdict = (f'CONTRIBUTING cogging (pk-pk {elec_pkpk_mA:.1f} mA, '
                   f'{VERDICT_NEGLIGIBLE_MA:.0f}–{VERDICT_MATERIAL_MA:.0f} mA band)')
    else:
        verdict = (f'MATERIAL cogging (pk-pk {elec_pkpk_mA:.1f} mA > '
                   f'{VERDICT_MATERIAL_MA:.0f}) — compensate in motor_guard')

    return SessionStats(
        path=path,
        n_samples_total=len(df),
        n_samples_used=n_used,
        t_window_s=(round(float(t[used_mask].min()), 2) if n_used else 0.0,
                    round(float(t[used_mask].max()), 2) if n_used else 0.0),
        target_vel_rps=round(target, 4),
        mean_vel_rps=round(float(vel.mean()) if vel.size else 0.0, 4),
        vel_std_rps=round(float(vel.std()) if vel.size else 0.0, 4),
        mean_iq_A=round(mean_iq, 4),
        iq_std_A=round(iq_std, 4),
        n_mech_revs_covered=round(float(abs(pos.max() - pos.min())) if pos.size else 0.0, 3),
        elec_bin_centres_rad=elec_centres,
        elec_bin_iq_mean_A=elec_means,
        elec_bin_counts=elec_counts,
        elec_ripple_pkpk_mA=round(elec_pkpk_mA, 1),
        elec_harmonics=elec_h,
        mech_bin_centres_rad=mech_centres,
        mech_bin_iq_mean_A=mech_means,
        mech_ripple_pkpk_mA=round(mech_pkpk_mA, 1),
        mech_harmonics=mech_h,
        pos_drift_slope_A_per_rev=round(pos_slope, 4),
        pos_drift_r2=round(pos_r2, 3),
        verdict=verdict,
    )


def _compare_fwd_rev(fwd: SessionStats, rev: SessionStats) -> Dict[str, float]:
    """Hysteresis / direction-dependent friction check.

    ``iq_mean`` flips sign if it's Coulomb friction (torque opposes motion).
    Cogging *repeats* with the same angular map regardless of direction.
    So the sum (fwd + rev)/2 isolates the *cogging* component, and the
    average of absolute values of the offsets gives the Coulomb band.
    """
    mean_fwd = fwd.mean_iq_A
    mean_rev = rev.mean_iq_A
    coulomb_A = (abs(mean_fwd) + abs(mean_rev)) / 2.0
    hysteresis_A = abs(mean_fwd + mean_rev) / 2.0  # residual drift not explained by friction

    # Cogging correlation: bin both maps to the same electrical-angle grid,
    # subtract per-session means, then compute Pearson R.  A positive R says
    # the two maps agree → cogging dominant.  Near-zero R says the ripple is
    # direction-specific → likely noise or a control-loop artefact, not cogging.
    a = fwd.elec_bin_iq_mean_A - np.nanmean(fwd.elec_bin_iq_mean_A)
    b = rev.elec_bin_iq_mean_A - np.nanmean(rev.elec_bin_iq_mean_A)
    mask = np.isfinite(a) & np.isfinite(b)
    if mask.sum() > 10:
        r = float(np.corrcoef(a[mask], b[mask])[0, 1])
    else:
        r = 0.0
    return {
        'coulomb_band_A': round(coulomb_A, 4),
        'hysteresis_residual_A': round(hysteresis_A, 4),
        'elec_map_fwd_vs_rev_pearson': round(r, 3),
    }


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def _fmt_harmonic_table(hdict: Dict[int, Dict[str, float]], top_n: int = 4) -> str:
    rows = sorted(hdict.items(), key=lambda kv: -kv[1]['r2'])[:top_n]
    out = []
    for n, h in rows:
        out.append(f"    n={n:3d}  R²={h['r2']:.3f}  amp={h['amplitude_mA']:6.1f} mA"
                   f"  phase={h['phase_rad']:+.2f} rad")
    return '\n'.join(out) if out else '    (no data)'


def print_report(stats: Sequence[SessionStats],
                 comparison: Optional[Dict[str, float]] = None) -> None:
    for s in stats:
        print('=' * 72)
        print(f'Session: {os.path.basename(s.path)}')
        print('=' * 72)
        print(f'  samples: {s.n_samples_used}/{s.n_samples_total} usable '
              f'(window {s.t_window_s[0]:.1f}–{s.t_window_s[1]:.1f} s)')
        print(f'  target vel: {s.target_vel_rps:+.4f} rev/s  '
              f'(measured: {s.mean_vel_rps:+.4f} ± {s.vel_std_rps:.4f})')
        print(f'  mech revs covered: {s.n_mech_revs_covered:.2f}')
        print(f'  mean iq: {s.mean_iq_A:+.4f} A   std: {s.iq_std_A:.4f} A')
        print()
        print(f'  VERDICT: {s.verdict}')
        print()
        print(f'  Electrical-angle ripple pk-pk: {s.elec_ripple_pkpk_mA:.1f} mA')
        print(f'  Top electrical harmonics (by R²):')
        print(_fmt_harmonic_table(s.elec_harmonics, top_n=4))
        print()
        print(f'  Mechanical-angle ripple pk-pk: {s.mech_ripple_pkpk_mA:.1f} mA')
        print(f'  Top mechanical harmonics (by R²):')
        print(_fmt_harmonic_table(s.mech_harmonics, top_n=6))
        print()
        print(f'  Position drift: {s.pos_drift_slope_A_per_rev*1000:+.1f} mA/rev '
              f'(R²={s.pos_drift_r2:.3f})   '
              f'[Coulomb/gravity bias — not cogging]')
        print()

    if comparison is not None:
        print('=' * 72)
        print('Forward vs Reverse comparison')
        print('=' * 72)
        print(f'  Coulomb friction band (½·(|iq_fwd| + |iq_rev|)): '
              f'{comparison["coulomb_band_A"]*1000:.1f} mA')
        print(f'  Residual (bias not explained by friction flip): '
              f'{comparison["hysteresis_residual_A"]*1000:.1f} mA')
        print(f'  Electrical-map fwd-vs-rev Pearson R: '
              f'{comparison["elec_map_fwd_vs_rev_pearson"]:+.3f}')
        if abs(comparison['elec_map_fwd_vs_rev_pearson']) > 0.5:
            print('    → ripple map is DIRECTION-INDEPENDENT — cogging signature')
        elif abs(comparison['elec_map_fwd_vs_rev_pearson']) < 0.2:
            print('    → ripple map is DIRECTION-DEPENDENT — likely control-loop / noise, not cogging')
        else:
            print('    → marginal correlation — partial cogging, partial other')


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------

def _maybe_plot(stats: Sequence[SessionStats], report_dir: str) -> None:
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception:
        print(f'(matplotlib unavailable, skipping plots)')
        return
    os.makedirs(report_dir, exist_ok=True)

    # iq vs electrical angle
    fig, ax = plt.subplots(figsize=(10, 5))
    for s in stats:
        y = s.elec_bin_iq_mean_A - np.nanmean(s.elec_bin_iq_mean_A)
        tag = os.path.basename(s.path).replace('cogging_', '').replace('.csv', '')
        ax.plot(np.degrees(s.elec_bin_centres_rad), y * 1000.0,
                label=f'{tag}  pk-pk={s.elec_ripple_pkpk_mA:.0f} mA', alpha=0.8)
    ax.set_xlabel('electrical angle [deg]')
    ax.set_ylabel('iq (mean-removed) [mA]')
    ax.set_title('iq vs electrical angle — cogging map')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(report_dir, 'iq_vs_elec_angle.png'), dpi=110)
    plt.close(fig)

    # iq vs mechanical angle
    fig, ax = plt.subplots(figsize=(10, 5))
    for s in stats:
        y = s.mech_bin_iq_mean_A - np.nanmean(s.mech_bin_iq_mean_A)
        tag = os.path.basename(s.path).replace('cogging_', '').replace('.csv', '')
        ax.plot(np.degrees(s.mech_bin_centres_rad), y * 1000.0,
                label=f'{tag}  pk-pk={s.mech_ripple_pkpk_mA:.0f} mA', alpha=0.8)
    ax.set_xlabel('mechanical angle [deg]')
    ax.set_ylabel('iq (mean-removed) [mA]')
    ax.set_title('iq vs mechanical angle')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(report_dir, 'iq_vs_mech_angle.png'), dpi=110)
    plt.close(fig)

    # Harmonic-fit R² bars
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    for ax, label, harmonics in [(axes[0], 'Electrical', ELEC_HARMONICS),
                                 (axes[1], 'Mechanical', MECH_HARMONICS)]:
        width = 0.8 / len(stats)
        for i, s in enumerate(stats):
            tag = os.path.basename(s.path).replace('cogging_', '').replace('.csv', '')
            h = s.elec_harmonics if label == 'Electrical' else s.mech_harmonics
            r2s = [h[n]['r2'] for n in harmonics]
            xs = np.arange(len(harmonics)) + (i - len(stats) / 2 + 0.5) * width
            ax.bar(xs, r2s, width=width, label=tag, alpha=0.85)
        ax.set_xticks(np.arange(len(harmonics)))
        ax.set_xticklabels([f'n={n}' for n in harmonics])
        ax.set_ylabel('sin-fit R²')
        ax.set_title(f'{label}-angle harmonic fit')
        ax.axhline(0.3, color='r', linestyle='--', alpha=0.4)
        ax.grid(True, axis='y', alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(report_dir, 'harmonic_r2.png'), dpi=110)
    plt.close(fig)

    # Raw time series — iq vs time and iq vs pos (one fig per session)
    for s in stats:
        df = pd.read_csv(s.path)
        tag = os.path.basename(s.path).replace('cogging_', '').replace('.csv', '')
        fig, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=False)
        axes[0].plot(df['t_s'], df['iq_measured_A'], lw=0.6, label='iq_measured')
        axes[0].plot(df['t_s'], df['iq_setpoint_A'], lw=0.6, alpha=0.6,
                     label='iq_setpoint')
        axes[0].axvspan(0, STARTUP_TRIM_S, color='gray', alpha=0.15, label='trimmed')
        axes[0].axvspan(df['t_s'].iloc[-1] - SHUTDOWN_TRIM_S, df['t_s'].iloc[-1],
                        color='gray', alpha=0.15)
        axes[0].set_xlabel('t [s]')
        axes[0].set_ylabel('iq [A]')
        axes[0].set_title(f'{tag} — time trace')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(fontsize=8)

        axes[1].scatter(df['pos_rev_raw'], df['iq_measured_A'], s=3, alpha=0.5)
        axes[1].set_xlabel('pos_rev_raw')
        axes[1].set_ylabel('iq [A]')
        axes[1].set_title(f'iq vs position '
                          f'(slope {s.pos_drift_slope_A_per_rev*1000:+.1f} mA/rev, '
                          f'R²={s.pos_drift_r2:.2f})')
        axes[1].grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(report_dir, f'timetrace_{tag}.png'), dpi=110)
        plt.close(fig)

    print(f'  Plots → {report_dir}')


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('csv', nargs='+', help='one or more cogging CSV files')
    parser.add_argument('--no-plot', action='store_true')
    parser.add_argument('--report-dir',
                        default=os.path.join(_REPO_ROOT, 'temp', 'reports',
                                             'cogging_map_study'))
    parser.add_argument('--json', default=None)
    args = parser.parse_args()

    stats: List[SessionStats] = []
    for p in args.csv:
        if not os.path.exists(p):
            print(f'ERROR: no such file {p}', file=sys.stderr)
            return 1
        stats.append(analyse_session(p))

    # Forward-vs-reverse pairing: if exactly 2 sessions and opposite signs,
    # compute hysteresis summary.
    comp: Optional[Dict[str, float]] = None
    if len(stats) == 2 and stats[0].target_vel_rps * stats[1].target_vel_rps < 0:
        fwd = stats[0] if stats[0].target_vel_rps > 0 else stats[1]
        rev = stats[1] if stats[0].target_vel_rps > 0 else stats[0]
        comp = _compare_fwd_rev(fwd, rev)

    print_report(stats, comp)

    if not args.no_plot:
        _maybe_plot(stats, args.report_dir)

    if args.json:
        payload: Dict = {
            'sessions': [{
                'path': s.path,
                'n_samples_used': s.n_samples_used,
                'target_vel_rps': s.target_vel_rps,
                'mean_vel_rps': s.mean_vel_rps,
                'vel_std_rps': s.vel_std_rps,
                'mean_iq_A': s.mean_iq_A,
                'iq_std_A': s.iq_std_A,
                'elec_ripple_pkpk_mA': s.elec_ripple_pkpk_mA,
                'mech_ripple_pkpk_mA': s.mech_ripple_pkpk_mA,
                'elec_harmonics': s.elec_harmonics,
                'mech_harmonics': s.mech_harmonics,
                'pos_drift_slope_A_per_rev': s.pos_drift_slope_A_per_rev,
                'pos_drift_r2': s.pos_drift_r2,
                'verdict': s.verdict,
            } for s in stats],
            'fwd_rev_comparison': comp,
        }
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, 'w') as f:
            json.dump(payload, f, indent=2)
        print(f'\nJSON → {args.json}')

    return 0


if __name__ == '__main__':
    sys.exit(main())
