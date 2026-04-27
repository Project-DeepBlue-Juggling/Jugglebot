#!/usr/bin/env python3
"""Analyse breakaway-ramp test summary CSV(s).

Decides:

  1. Whether the recorded breakaway events represent **true rotor escape**
     from a cogging detent, or just **detent compliance** (small angular
     deflection within the same detent).  The discriminator is the
     ``breakaway_pos_displacement_rev`` column relative to the cogging
     period (1/84 rev ≈ 0.012 rev for this motor).
  2. Per-direction breakaway iq mean / std / range.
  3. Angular dependence of breakaway iq — fit cos(n·θ_elec) for n in
     {1,2,4,6}, report R² and amplitude.
  4. Forward-vs-reverse symmetry.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_SCRIPT_DIR)

# Cogging-period heuristics for the leg motor (P=7, S=12 → 84 cycles per mech rev)
COGGING_PERIOD_REV = 1.0 / 84.0


def _fit_cos(theta: np.ndarray, y: np.ndarray, n: int) -> Dict[str, float]:
    if len(theta) < 4:
        return {'r2': 0.0, 'amplitude_A': 0.0, 'phase_rad': 0.0}
    X = np.column_stack([np.cos(n * theta), np.sin(n * theta), np.ones_like(theta)])
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    y_hat = X @ coef
    ss_res = float(((y - y_hat) ** 2).sum())
    ss_tot = float(((y - y.mean()) ** 2).sum())
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-9 else 0.0
    return {
        'r2': round(r2, 3),
        'amplitude_A': round(float(math.hypot(coef[0], coef[1])), 4),
        'phase_rad': round(float(math.atan2(coef[1], coef[0])), 3),
    }


def analyse(path: str) -> Dict:
    df = pd.read_csv(path)
    print(f'\n=== {os.path.basename(path)} ===')
    print(f'  trials: {len(df)}  outcomes: {dict(df["outcome"].value_counts())}')

    # The crucial sanity check: was this a TRUE breakaway, or did we trigger
    # on detent compliance (rotor moved within its detent, didn't escape)?
    disp = df['breakaway_pos_displacement_rev'].abs().values
    print(f'  displacement at "breakaway": '
          f'min={disp.min()*1000:.3f} mrev, '
          f'median={np.median(disp)*1000:.3f} mrev, '
          f'max={disp.max()*1000:.3f} mrev')
    print(f'  cogging period:              '
          f'{COGGING_PERIOD_REV*1000:.2f} mrev (1/84 mech rev)')
    n_within_detent = int((disp < COGGING_PERIOD_REV * 0.5).sum())
    n_escape = int((disp >= COGGING_PERIOD_REV * 1.5).sum())
    print(f'  → {n_within_detent}/{len(df)} trials moved less than half a '
          f'detent (likely detent-compliance, NOT escape)')
    print(f'  → {n_escape}/{len(df)} trials moved more than 1.5 detents '
          f'(likely true rotor escape)')

    out: Dict = {'path': path, 'n_trials': len(df),
                 'n_within_detent': n_within_detent, 'n_escape': n_escape,
                 'cogging_period_rev': COGGING_PERIOD_REV,
                 'displacement_stats_mrev': {
                     'min': round(float(disp.min() * 1000), 3),
                     'median': round(float(np.median(disp) * 1000), 3),
                     'max': round(float(disp.max() * 1000), 3),
                 }}

    for dirn in df['direction'].unique():
        sub = df[df['direction'] == dirn]
        ib = sub['breakaway_iq_A'].abs().values
        theta_e = sub['start_elec_angle_rad'].values
        print(f'\n  {dirn} (n={len(sub)}):')
        print(f'    |iq_breakaway|: mean={ib.mean()*1000:.1f} mA,  '
              f'std={ib.std()*1000:.1f} mA,  '
              f'min={ib.min()*1000:.1f} mA,  '
              f'max={ib.max()*1000:.1f} mA,  '
              f'σ/μ={ib.std()/max(1e-9, ib.mean())*100:.1f}%')

        # Angular-dependence fits
        if len(sub) >= 4:
            harmonics = {n: _fit_cos(theta_e, ib, n) for n in (1, 2, 4, 6)}
            print(f'    angular fits (vs start_elec_angle):')
            for n, fit in harmonics.items():
                print(f'      n={n}: R²={fit["r2"]:.3f}  amp={fit["amplitude_A"]*1000:.1f} mA')
            best_n, best_fit = max(harmonics.items(), key=lambda kv: kv[1]['r2'])
            if best_fit['r2'] > 0.5:
                verdict = (f'STRONG angular dependence at n={best_n} '
                           f'(R²={best_fit["r2"]:.2f}, amp={best_fit["amplitude_A"]*1000:.1f} mA)')
            elif best_fit['r2'] > 0.25:
                verdict = (f'MARGINAL angular dependence at n={best_n} '
                           f'(R²={best_fit["r2"]:.2f})')
            else:
                verdict = 'no angular dependence — pure stiction (or pure compliance)'
            print(f'    → {verdict}')
            out[f'{dirn}_harmonics'] = harmonics
            out[f'{dirn}_verdict'] = verdict

        out[f'{dirn}_iq_mean_mA'] = round(float(ib.mean() * 1000), 1)
        out[f'{dirn}_iq_std_mA'] = round(float(ib.std() * 1000), 1)
        out[f'{dirn}_cv_pct'] = round(float(ib.std() / max(1e-9, ib.mean()) * 100), 1)

    # Forward-vs-reverse symmetry
    if {'forward', 'reverse'}.issubset(df['direction'].unique()):
        fwd_mean = df[df['direction'] == 'forward']['breakaway_iq_A'].abs().mean()
        rev_mean = df[df['direction'] == 'reverse']['breakaway_iq_A'].abs().mean()
        ratio = fwd_mean / max(1e-9, rev_mean)
        print(f'\n  fwd/rev mean |iq_breakaway| ratio: {ratio:.2f}')
        if 0.85 < ratio < 1.15:
            print('    → symmetric, friction-dominated')
        else:
            print('    → asymmetric, possible mechanical bias or motor mounting effect')
        out['fwd_rev_ratio'] = round(ratio, 3)

    return out


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument('csv', nargs='+')
    p.add_argument('--json', default=None)
    args = p.parse_args()
    payload = [analyse(c) for c in args.csv]
    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, 'w') as f:
            json.dump(payload, f, indent=2)
        print(f'\nJSON → {args.json}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
