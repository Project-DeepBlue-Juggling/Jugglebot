#!/usr/bin/env python3
"""Friction / Stribeck-curve study from the cogging-bench velocity sweep.

Consumes N pairs of ``cogging_*.csv`` files (one forward, one reverse per
target velocity) produced by ``tests/hardware/cogging_bench_test.py`` and
extracts:

  1. The Stribeck friction curve |iq|(ω) — mean current vs velocity across
     both directions, with fwd/rev half-difference (the pure friction
     component, gravity/offset subtracted) and half-sum (any direction-
     independent bias).
  2. Tracking quality vs velocity — vel_std / target_vel — showing where
     stick-slip gives way to smooth motion.
  3. Per-velocity forward-vs-reverse Pearson R on the electrical-angle map
     — the cogging-detection metric; should rise from the 0.0 we saw at
     0.05 rev/s IF cogging is present but previously drowned by stick-slip.
  4. Parametric fit to ``τ(ω) = τ_c + (τ_s - τ_c)·exp(-(|ω|/ω_s)²) + b·|ω|``
     (Stribeck with Coulomb floor, stiction peak, breakaway speed, viscous
     slope).  Falls back to plain Coulomb if Stribeck doesn't reduce χ².

Usage
-----
    # Auto-discover all cogging CSVs under temp/logs/
    python tools/friction_study_analyse.py --auto

    # Explicit list (order doesn't matter — pairs by target_vel and sign)
    python tools/friction_study_analyse.py temp/logs/cogging_*_0.*rps.csv

    # Restrict to a specific timestamp prefix
    python tools/friction_study_analyse.py --glob 'temp/logs/cogging_20260424_2330*.csv'
"""
from __future__ import annotations

import argparse
import glob
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
sys.path.insert(0, _SCRIPT_DIR)

# Reuse the single-session analyser
from cogging_map_analyse import (  # type: ignore  # noqa: E402
    analyse_session,
    SessionStats,
    STARTUP_TRIM_S,
    SHUTDOWN_TRIM_S,
)


# ---------------------------------------------------------------------------
# Pairing
# ---------------------------------------------------------------------------

@dataclass
class VelocityPoint:
    target_vel_rps: float
    fwd: SessionStats
    rev: SessionStats
    # Derived
    friction_iq_A: float = 0.0          # ½(|iq_fwd| + |iq_rev|) — pure friction
    offset_iq_A: float = 0.0            # ½(iq_fwd + iq_rev) — direction-independent bias
    tracking_ratio_fwd: float = 0.0     # vel_std / |target_vel|
    tracking_ratio_rev: float = 0.0
    elec_map_pearson_r: float = 0.0     # fwd-vs-rev electrical-angle-map correlation


def _pair_sessions(stats: Sequence[SessionStats]) -> List[VelocityPoint]:
    """Group sessions into (fwd, rev) pairs keyed by |target_vel|."""
    buckets: Dict[float, Dict[str, SessionStats]] = {}
    for s in stats:
        key = round(abs(s.target_vel_rps), 4)
        if key < 1e-4:
            continue
        dirn = 'fwd' if s.target_vel_rps > 0 else 'rev'
        buckets.setdefault(key, {})[dirn] = s

    points: List[VelocityPoint] = []
    for key in sorted(buckets):
        b = buckets[key]
        if 'fwd' not in b or 'rev' not in b:
            print(f'  WARN: velocity {key:+.4f} rev/s has only '
                  f'{list(b)} — skipping', file=sys.stderr)
            continue
        fwd, rev = b['fwd'], b['rev']
        point = VelocityPoint(target_vel_rps=key, fwd=fwd, rev=rev)
        point.friction_iq_A = 0.5 * (abs(fwd.mean_iq_A) + abs(rev.mean_iq_A))
        point.offset_iq_A = 0.5 * (fwd.mean_iq_A + rev.mean_iq_A)
        point.tracking_ratio_fwd = fwd.vel_std_rps / max(1e-6, abs(fwd.target_vel_rps))
        point.tracking_ratio_rev = rev.vel_std_rps / max(1e-6, abs(rev.target_vel_rps))
        # Pearson R between fwd and rev electrical maps (after per-session mean removal)
        a = fwd.elec_bin_iq_mean_A - np.nanmean(fwd.elec_bin_iq_mean_A)
        b_ = rev.elec_bin_iq_mean_A - np.nanmean(rev.elec_bin_iq_mean_A)
        mask = np.isfinite(a) & np.isfinite(b_)
        if mask.sum() > 10:
            point.elec_map_pearson_r = float(np.corrcoef(a[mask], b_[mask])[0, 1])
        points.append(point)
    return points


# ---------------------------------------------------------------------------
# Stribeck fit
# ---------------------------------------------------------------------------

def _stribeck_residual(params: np.ndarray, omega: np.ndarray,
                       friction: np.ndarray) -> np.ndarray:
    tau_c, tau_s, omega_s, b = params
    omega_s = max(omega_s, 1e-5)
    model = tau_c + (tau_s - tau_c) * np.exp(-(omega / omega_s) ** 2) + b * omega
    return model - friction


def _fit_stribeck(omega: np.ndarray, friction: np.ndarray) -> Dict[str, float]:
    """Fit τ(ω) = τ_c + (τ_s - τ_c)·exp(-(ω/ω_s)²) + b·ω to |iq|(|ω|).

    Uses a bounded Nelder-Mead search over (τ_c, τ_s, ω_s, b) without SciPy.
    Returns {} if fewer than 3 data points (under-determined).
    """
    if len(omega) < 3:
        return {}
    # Seed from simple heuristics
    tau_s_init = float(friction.max())                 # stiction = peak
    tau_c_init = float(friction.min())                 # Coulomb floor
    omega_s_init = max(1e-3, float(omega[np.argmin(friction)]))  # breakaway speed
    b_init = 0.0

    def scalar_cost(p):
        r = _stribeck_residual(p, omega, friction)
        return float(np.sum(r * r))

    # Light-weight pattern search (no SciPy dependency)
    p = np.array([tau_c_init, tau_s_init, omega_s_init, b_init], dtype=float)
    best = scalar_cost(p)
    step = np.array([0.05, 0.1, 0.05, 0.2])
    for _ in range(300):
        improved = False
        for i in range(4):
            for d in (+1, -1):
                trial = p.copy()
                trial[i] += d * step[i]
                if i in (0, 1, 2) and trial[i] < 0:
                    continue
                c = scalar_cost(trial)
                if c < best - 1e-9:
                    best = c
                    p = trial
                    improved = True
        if not improved:
            step *= 0.5
            if np.max(step) < 1e-5:
                break

    tau_c, tau_s, omega_s, b = p
    # Goodness-of-fit
    ss_res = best
    ss_tot = float(np.sum((friction - friction.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-9 else 0.0

    # Compare against plain-Coulomb (one parameter: τ = const)
    tau_coulomb = float(friction.mean())
    ss_coulomb = float(np.sum((friction - tau_coulomb) ** 2))
    r2_coulomb = 1.0 - ss_coulomb / ss_tot if ss_tot > 1e-9 else 0.0

    return {
        'tau_c_A': round(float(tau_c), 4),
        'tau_s_A': round(float(tau_s), 4),
        'omega_s_rps': round(float(omega_s), 4),
        'viscous_b_A_per_rps': round(float(b), 4),
        'r2_stribeck': round(r2, 3),
        'r2_coulomb_only': round(r2_coulomb, 3),
        'tau_coulomb_simple_A': round(tau_coulomb, 4),
    }


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------

def _plot_all(points: Sequence[VelocityPoint], fit: Dict[str, float],
              out_dir: str) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    os.makedirs(out_dir, exist_ok=True)

    omega = np.array([p.target_vel_rps for p in points])
    friction = np.array([p.friction_iq_A for p in points])
    offset = np.array([p.offset_iq_A for p in points])
    iq_fwd = np.array([abs(p.fwd.mean_iq_A) for p in points])
    iq_rev = np.array([abs(p.rev.mean_iq_A) for p in points])
    iq_std_fwd = np.array([p.fwd.iq_std_A for p in points])
    iq_std_rev = np.array([p.rev.iq_std_A for p in points])

    # 1 — Stribeck curve
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.errorbar(omega, iq_fwd, yerr=iq_std_fwd, fmt='o-', label='|mean iq| fwd',
                capsize=3, alpha=0.8)
    ax.errorbar(omega, iq_rev, yerr=iq_std_rev, fmt='s-', label='|mean iq| rev',
                capsize=3, alpha=0.8)
    ax.plot(omega, friction, 'k^--', label='½·(|fwd| + |rev|)  (pure friction)', lw=2)
    if fit:
        om_fine = np.linspace(omega.min(), omega.max() * 1.05, 200)
        model = (fit['tau_c_A']
                 + (fit['tau_s_A'] - fit['tau_c_A'])
                    * np.exp(-(om_fine / max(fit['omega_s_rps'], 1e-4)) ** 2)
                 + fit['viscous_b_A_per_rps'] * om_fine)
        ax.plot(om_fine, model, 'r:', lw=2,
                label=(f"Stribeck fit: τ_c={fit['tau_c_A']:.2f} A, "
                       f"τ_s={fit['tau_s_A']:.2f} A, "
                       f"ω_s={fit['omega_s_rps']:.3f} rev/s, "
                       f"b={fit['viscous_b_A_per_rps']:.2f} A/(rev/s)  "
                       f"R²={fit['r2_stribeck']:.2f}"))
    ax.set_xlabel('|target velocity| [rev/s]')
    ax.set_ylabel('|mean iq| [A]')
    ax.set_title('Stribeck friction curve — bench tester leg')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'stribeck_curve.png'), dpi=110)
    plt.close(fig)

    # 2 — Tracking quality
    fig, ax = plt.subplots(figsize=(9, 4))
    track_fwd = [p.tracking_ratio_fwd for p in points]
    track_rev = [p.tracking_ratio_rev for p in points]
    ax.plot(omega, track_fwd, 'o-', label='vel_std / |target|  fwd')
    ax.plot(omega, track_rev, 's-', label='vel_std / |target|  rev')
    ax.axhline(0.1, color='r', linestyle='--', alpha=0.4,
               label='smooth-tracking threshold (10 %)')
    ax.set_xlabel('|target velocity| [rev/s]')
    ax.set_ylabel('vel_std / |target_vel|  (dimensionless)')
    ax.set_title('Tracking quality — higher = more stick-slip')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'tracking_quality.png'), dpi=110)
    plt.close(fig)

    # 3 — iq vs time grid (one row per velocity, fwd+rev)
    n = len(points)
    fig, axes = plt.subplots(n, 2, figsize=(12, 2.2 * n), sharex=True)
    if n == 1:
        axes = np.array([axes])
    for row, p in enumerate(points):
        for col, s in enumerate([p.fwd, p.rev]):
            ax = axes[row, col]
            df = pd.read_csv(s.path)
            ax.plot(df['t_s'], df['iq_measured_A'], lw=0.5)
            ax.axvspan(0, STARTUP_TRIM_S, color='gray', alpha=0.15)
            ax.axvspan(df['t_s'].iloc[-1] - SHUTDOWN_TRIM_S, df['t_s'].iloc[-1],
                       color='gray', alpha=0.15)
            ax.set_ylabel(f"{'fwd' if col==0 else 'rev'} {p.target_vel_rps:.3f} rps\niq [A]")
            ax.grid(True, alpha=0.3)
            ax.set_title(f'mean={s.mean_iq_A:+.3f}  std={s.iq_std_A:.3f}',
                         fontsize=9)
    axes[-1, 0].set_xlabel('t [s]')
    axes[-1, 1].set_xlabel('t [s]')
    fig.suptitle('iq(t) across the velocity sweep — stick-slip visible as thick ripple')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'iq_vs_time_grid.png'), dpi=110)
    plt.close(fig)

    # 4 — Electrical-map Pearson R vs velocity
    rs = [p.elec_map_pearson_r for p in points]
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(omega, rs, 'o-', lw=2)
    ax.axhline(0.5, color='g', linestyle='--', alpha=0.4,
               label='strong cogging threshold (R ≥ 0.5)')
    ax.axhline(0.2, color='orange', linestyle='--', alpha=0.4,
               label='marginal (R ≥ 0.2)')
    ax.axhline(0.0, color='k', linestyle='-', alpha=0.2)
    ax.set_xlabel('|target velocity| [rev/s]')
    ax.set_ylabel('fwd-vs-rev electrical-map Pearson R')
    ax.set_title('Cogging visibility — if R rises with velocity, '
                 'cogging is present but drowned at low speed')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'elec_map_pearson.png'), dpi=110)
    plt.close(fig)

    # 5 — Overlaid electrical-angle maps (all velocities, fwd only)
    fig, ax = plt.subplots(figsize=(10, 5))
    for p in points:
        s = p.fwd
        y = s.elec_bin_iq_mean_A - np.nanmean(s.elec_bin_iq_mean_A)
        ax.plot(np.degrees(s.elec_bin_centres_rad), y * 1000.0,
                label=f'{p.target_vel_rps:.3f} rev/s  '
                      f'(pk-pk {s.elec_ripple_pkpk_mA:.0f} mA)',
                alpha=0.75)
    ax.set_xlabel('electrical angle [deg]')
    ax.set_ylabel('iq (mean-removed) [mA]')
    ax.set_title('Electrical-angle ripple at each velocity (forward direction)')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'elec_maps_all_velocities.png'), dpi=110)
    plt.close(fig)

    print(f'  plots → {out_dir}')


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def _print_summary(points: Sequence[VelocityPoint], fit: Dict[str, float]) -> None:
    print('=' * 80)
    print('Friction study — velocity sweep')
    print('=' * 80)
    print(f"  {'|v| [rev/s]':>12}  {'|iq_fwd| [A]':>12}  {'|iq_rev| [A]':>12}  "
          f"{'friction [A]':>14}  {'offset [A]':>11}  "
          f"{'track_fwd':>10}  {'track_rev':>10}  {'fwd-rev R':>10}")
    for p in points:
        print(f"  {p.target_vel_rps:>12.4f}  "
              f"{abs(p.fwd.mean_iq_A):>12.4f}  "
              f"{abs(p.rev.mean_iq_A):>12.4f}  "
              f"{p.friction_iq_A:>14.4f}  "
              f"{p.offset_iq_A:>+11.4f}  "
              f"{p.tracking_ratio_fwd:>10.3f}  "
              f"{p.tracking_ratio_rev:>10.3f}  "
              f"{p.elec_map_pearson_r:>+10.3f}")
    if fit:
        print()
        print('  Stribeck fit  τ(ω) = τ_c + (τ_s − τ_c)·exp(−(ω/ω_s)²) + b·ω')
        print(f"    τ_c (kinetic Coulomb floor) = {fit['tau_c_A']:.4f} A  "
              f"({fit['tau_c_A']*0.062*1000:.1f} mNm)")
        print(f"    τ_s (stiction peak)          = {fit['tau_s_A']:.4f} A  "
              f"({fit['tau_s_A']*0.062*1000:.1f} mNm)")
        print(f"    ω_s (breakaway speed scale)  = {fit['omega_s_rps']:.4f} rev/s  "
              f"({fit['omega_s_rps']*60:.1f} RPM)")
        print(f"    b   (viscous slope)          = {fit['viscous_b_A_per_rps']:.4f} A/(rev/s)")
        print(f"    R²_Stribeck = {fit['r2_stribeck']:.3f}  "
              f"vs R²_Coulomb-only = {fit['r2_coulomb_only']:.3f}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('csv', nargs='*', help='cogging_*.csv file paths')
    parser.add_argument('--auto', action='store_true',
                        help='discover latest sweep under temp/logs/')
    parser.add_argument('--glob', default=None,
                        help='explicit glob pattern (overrides positional)')
    parser.add_argument('--report-dir',
                        default=os.path.join(_REPO_ROOT, 'temp', 'reports',
                                             'friction_study'))
    parser.add_argument('--json', default=None)
    parser.add_argument('--no-plot', action='store_true')
    args = parser.parse_args()

    if args.glob:
        paths = sorted(glob.glob(args.glob))
    elif args.auto:
        all_cogging = sorted(glob.glob(
            os.path.join(_REPO_ROOT, 'temp', 'logs', 'cogging_*.csv')))
        if not all_cogging:
            print('No cogging_*.csv files under temp/logs/', file=sys.stderr)
            return 1
        # Find the most recent timestamp prefix, keep every file that shares it
        latest_ts = os.path.basename(all_cogging[-1])[:23]  # cogging_YYYYMMDD_HHMMSS
        paths = [p for p in all_cogging if os.path.basename(p).startswith(latest_ts)]
        # If only one pair (the latest single run), broaden to "all runs made
        # within the last 15 min of the latest one"
        if len(paths) < 4:
            import re
            from datetime import datetime, timedelta
            m = re.search(r'cogging_(\d{8}_\d{6})', latest_ts)
            if m:
                latest_dt = datetime.strptime(m.group(1), '%Y%m%d_%H%M%S')
                cutoff = latest_dt - timedelta(minutes=15)
                paths = []
                for p in all_cogging:
                    mm = re.search(r'cogging_(\d{8}_\d{6})', os.path.basename(p))
                    if mm:
                        dt = datetime.strptime(mm.group(1), '%Y%m%d_%H%M%S')
                        if dt >= cutoff:
                            paths.append(p)
                paths.sort()
    else:
        paths = sorted(args.csv)

    if not paths:
        print('No CSVs selected.', file=sys.stderr)
        return 1

    print(f'Analysing {len(paths)} CSV(s):')
    for p in paths:
        print(f'  {os.path.basename(p)}')
    print()

    stats = [analyse_session(p) for p in paths]
    points = _pair_sessions(stats)
    if not points:
        print('No fwd/rev pairs formed — check filenames.', file=sys.stderr)
        return 1

    omega = np.array([p.target_vel_rps for p in points])
    friction = np.array([p.friction_iq_A for p in points])
    fit = _fit_stribeck(omega, friction)

    _print_summary(points, fit)

    if not args.no_plot:
        _plot_all(points, fit, args.report_dir)

    if args.json:
        payload = {
            'points': [{
                'target_vel_rps': p.target_vel_rps,
                'fwd_mean_iq_A': p.fwd.mean_iq_A,
                'rev_mean_iq_A': p.rev.mean_iq_A,
                'friction_iq_A': p.friction_iq_A,
                'offset_iq_A': p.offset_iq_A,
                'tracking_ratio_fwd': p.tracking_ratio_fwd,
                'tracking_ratio_rev': p.tracking_ratio_rev,
                'elec_map_fwd_rev_pearson': p.elec_map_pearson_r,
                'fwd_iq_std_A': p.fwd.iq_std_A,
                'rev_iq_std_A': p.rev.iq_std_A,
                'fwd_elec_pkpk_mA': p.fwd.elec_ripple_pkpk_mA,
                'rev_elec_pkpk_mA': p.rev.elec_ripple_pkpk_mA,
            } for p in points],
            'stribeck_fit': fit,
        }
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, 'w') as f:
            json.dump(payload, f, indent=2)
        print(f'\nJSON → {args.json}')

    return 0


if __name__ == '__main__':
    sys.exit(main())
