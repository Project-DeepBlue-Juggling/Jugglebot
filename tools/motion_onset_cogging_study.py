#!/usr/bin/env python3
"""Desk-check for a cogging-torque fingerprint in motion-onset events.

Mines every ``temp/logs/mpc_*.csv`` (or an explicit subset) for motion-onset
events detected by ``sim.analysis.diagnose.analyse_motion_onset``.  For each
event, the detector now emits the motor's rest electrical angle at the
moment just before breakaway.  This script correlates rest angle with the
first-tick leap magnitude (and latency) to decide whether cogging torque is
modulating motion-onset behaviour.

What "cogging signature" means here:

  A permanent-magnet motor with P pole pairs and S stator slots has a
  cogging-torque fundamental period of ``2π / lcm(2P, S)`` electrical
  radians per mechanical revolution.  For the Jugglebot leg motor
  (P=7, S=12) this is 84 cycles per mechanical revolution, i.e. detent
  peaks every ``2π/84 rad ≈ 4.3°`` of rotor angle.  If cogging dominates
  motion-onset, then parking the rotor near a detent produces a larger
  breakaway leap than parking it on a cogging slope.

Outputs
-------
* Console: per-leg Spearman correlation between leap magnitude and nearest-
  detent distance, plus the dominant harmonic from the sinusoidal-fit sweep.
* Figures under ``temp/reports/motion_onset_cogging_study/``:
    - scatter.png   — leap vs rest_elec_angle, one subplot per leg
    - binned.png    — median leap per cogging-period bin
    - harmonic.png  — R² of ``A·sin(n·θ+φ)+B`` across n ∈ {7,12,14,84}

Usage
-----
    python tools/motion_onset_cogging_study.py
    python tools/motion_onset_cogging_study.py --glob 'temp/logs/mpc_202604*.csv'
    python tools/motion_onset_cogging_study.py --json temp/reports/cogging.json
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)
_SIM_DIR = os.path.join(_REPO_ROOT, 'sim')
if _SIM_DIR not in sys.path:
    sys.path.insert(0, _SIM_DIR)

from analysis.compare import load_csv  # type: ignore
from analysis.diagnose import analyse_motion_onset, _load_motor_params  # type: ignore


# Harmonics to probe for cogging modulation.  7 = pole pair, 12 = slots,
# 14 = 2 × pole pairs (magnetic cycle per mech rev), 84 = lcm(14,12) fundamental.
HARMONIC_SWEEP = (7, 12, 14, 42, 84)

# Minimum events per leg before we trust the correlation number.
MIN_EVENTS_PER_LEG = 6


@dataclass
class Event:
    session: str
    leg: int
    rest_elec_angle_rad: float
    rest_mech_angle_rad: float
    leap_mm: float
    latency_ms: float
    hold_pos_mm: float


def collect_events(csv_paths: Sequence[str]) -> List[Event]:
    events: List[Event] = []
    for path in csv_paths:
        try:
            records = load_csv(path)
        except Exception as exc:
            print(f"  skip {os.path.basename(path)}: load failed ({exc})")
            continue
        if len(records) < 40:
            continue
        onset = analyse_motion_onset(records)
        if not onset.get('detected'):
            continue
        for ev in onset.get('events', []):
            for leg_str, per in ev.get('per_leg', {}).items():
                rest_elec = per.get('rest_elec_angle_rad')
                rest_mech = per.get('rest_mech_angle_rad')
                if rest_elec is None:
                    continue  # motor params missing → skip rather than guess
                events.append(Event(
                    session=os.path.basename(path),
                    leg=int(leg_str),
                    rest_elec_angle_rad=float(rest_elec),
                    rest_mech_angle_rad=float(rest_mech) if rest_mech is not None else 0.0,
                    leap_mm=float(per.get('first_tick_leap_mm', 0.0)),
                    latency_ms=float(per.get('latency_ms', 0.0)),
                    hold_pos_mm=float(per.get('hold_pos_mm', 0.0)),
                ))
    return events


def _spearman(x: np.ndarray, y: np.ndarray) -> float:
    """Plain Spearman rank correlation — no SciPy dep."""
    if len(x) < 3:
        return 0.0
    rx = np.argsort(np.argsort(x)).astype(float)
    ry = np.argsort(np.argsort(y)).astype(float)
    rx -= rx.mean()
    ry -= ry.mean()
    denom = math.sqrt(float((rx * rx).sum()) * float((ry * ry).sum()))
    if denom == 0.0:
        return 0.0
    return float((rx * ry).sum() / denom)


def _nearest_detent_distance(angles_rad: np.ndarray, period_per_rev: int,
                             rev_fraction: float = 1.0) -> np.ndarray:
    """Distance (rad) to nearest cogging detent for a given harmonic.

    ``angles_rad`` is the mechanical rest angle modulo 2π.  Detents sit at
    multiples of ``2π / period_per_rev``.  Returns ``[0, π/period_per_rev]``.
    """
    cogging_period = 2.0 * math.pi / period_per_rev
    phase = angles_rad % cogging_period
    return np.minimum(phase, cogging_period - phase)


def _sinusoidal_fit_r2(theta: np.ndarray, y: np.ndarray, n: int) -> Tuple[float, float, float]:
    """Least-squares fit ``y ≈ A cos(n·θ) + B sin(n·θ) + C``.

    Returns (R², amplitude, offset_C).
    """
    if len(theta) < 4:
        return 0.0, 0.0, 0.0
    X = np.column_stack([np.cos(n * theta), np.sin(n * theta), np.ones_like(theta)])
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    y_hat = X @ coef
    ss_res = float(((y - y_hat) ** 2).sum())
    ss_tot = float(((y - y.mean()) ** 2).sum())
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    amplitude = float(math.hypot(coef[0], coef[1]))
    return r2, amplitude, float(coef[2])


def per_leg_analysis(events: Sequence[Event]) -> dict:
    by_leg: dict = {leg: [] for leg in range(6)}
    for ev in events:
        by_leg[ev.leg].append(ev)

    result: dict = {}
    for leg, evs in by_leg.items():
        if len(evs) < MIN_EVENTS_PER_LEG:
            result[f'leg_{leg}'] = {
                'n_events': len(evs),
                'skipped': f'< {MIN_EVENTS_PER_LEG} events — insufficient data',
            }
            continue
        theta_mech = np.array([e.rest_mech_angle_rad for e in evs])
        theta_elec = np.array([e.rest_elec_angle_rad for e in evs])
        leap = np.array([e.leap_mm for e in evs])
        lat = np.array([e.latency_ms for e in evs])

        harmonic_results = {}
        for n in HARMONIC_SWEEP:
            r2_leap, amp_leap, off_leap = _sinusoidal_fit_r2(theta_mech, leap, n)
            r2_lat, amp_lat, off_lat = _sinusoidal_fit_r2(theta_mech, lat, n)
            # Spearman between leap and (negative) distance-to-detent: closer
            # to detent (smaller distance) → larger leap expected.
            dist = _nearest_detent_distance(theta_mech, n)
            rho_leap = _spearman(-dist, leap)
            rho_lat = _spearman(-dist, lat)
            harmonic_results[f'n={n}'] = {
                'leap_fit_r2': round(r2_leap, 3),
                'leap_fit_amplitude_mm': round(amp_leap, 3),
                'leap_fit_offset_mm': round(off_leap, 3),
                'leap_vs_detent_spearman': round(rho_leap, 3),
                'latency_fit_r2': round(r2_lat, 3),
                'latency_fit_amplitude_ms': round(amp_lat, 1),
                'latency_vs_detent_spearman': round(rho_lat, 3),
            }

        # Verdict heuristic: pick the harmonic with the best leap-fit R² and
        # report its Spearman magnitude too.
        best_n = max(HARMONIC_SWEEP,
                     key=lambda n: harmonic_results[f'n={n}']['leap_fit_r2'])
        best_r2 = harmonic_results[f'n={best_n}']['leap_fit_r2']
        best_rho = harmonic_results[f'n={best_n}']['leap_vs_detent_spearman']
        if best_r2 > 0.3 and abs(best_rho) > 0.3:
            verdict = f'COGGING SIGNATURE at n={best_n} (R²={best_r2:.2f}, ρ={best_rho:.2f})'
        elif best_r2 > 0.15:
            verdict = f'weak modulation at n={best_n} (R²={best_r2:.2f}) — marginal'
        else:
            verdict = 'no cogging signature (flat distribution)'

        result[f'leg_{leg}'] = {
            'n_events': len(evs),
            'leap_mm_mean': round(float(leap.mean()), 3),
            'leap_mm_std': round(float(leap.std()), 3),
            'latency_ms_mean': round(float(lat.mean()), 1),
            'latency_ms_std': round(float(lat.std()), 1),
            'harmonics': harmonic_results,
            'verdict': verdict,
        }
    return result


def _maybe_plot(events: Sequence[Event], report_dir: str) -> None:
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception:
        print(f"  matplotlib unavailable — skipping plots (would go to {report_dir})")
        return
    os.makedirs(report_dir, exist_ok=True)

    # Scatter: leap vs rest_elec_angle, per leg
    fig, axes = plt.subplots(2, 3, figsize=(15, 8), sharex=True, sharey=True)
    for leg in range(6):
        ax = axes[leg // 3, leg % 3]
        evs = [e for e in events if e.leg == leg]
        if evs:
            theta = np.array([e.rest_elec_angle_rad for e in evs])
            leap = np.array([e.leap_mm for e in evs])
            ax.scatter(theta, leap, alpha=0.7, s=20)
        ax.set_title(f'Leg {leg} (n={len(evs)})')
        ax.set_xlim(0, 2 * math.pi)
        ax.grid(True, alpha=0.3)
    for ax in axes[-1, :]:
        ax.set_xlabel('rest electrical angle [rad]')
    for ax in axes[:, 0]:
        ax.set_ylabel('first-tick leap [mm]')
    fig.suptitle('Motion-onset leap vs rest electrical angle')
    fig.tight_layout()
    fig.savefig(os.path.join(report_dir, 'scatter.png'), dpi=110)
    plt.close(fig)

    # Harmonic-sweep bar chart: R² per n, per leg
    fig, ax = plt.subplots(figsize=(10, 5))
    width = 0.8 / 6
    for leg in range(6):
        evs = [e for e in events if e.leg == leg]
        if len(evs) < MIN_EVENTS_PER_LEG:
            continue
        theta_mech = np.array([e.rest_mech_angle_rad for e in evs])
        leap = np.array([e.leap_mm for e in evs])
        r2s = [_sinusoidal_fit_r2(theta_mech, leap, n)[0] for n in HARMONIC_SWEEP]
        xs = np.arange(len(HARMONIC_SWEEP)) + (leg - 2.5) * width
        ax.bar(xs, r2s, width=width, label=f'Leg {leg}')
    ax.set_xticks(np.arange(len(HARMONIC_SWEEP)))
    ax.set_xticklabels([f'n={n}' for n in HARMONIC_SWEEP])
    ax.set_ylabel('sinusoidal-fit R²')
    ax.set_title('Cogging-harmonic fit against leap magnitude')
    ax.axhline(0.3, color='r', linestyle='--', alpha=0.4, label='COGGING threshold R²=0.3')
    ax.grid(True, axis='y', alpha=0.3)
    ax.legend(fontsize=8, ncol=2)
    fig.tight_layout()
    fig.savefig(os.path.join(report_dir, 'harmonic.png'), dpi=110)
    plt.close(fig)

    print(f"  plots written to {report_dir}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--glob', default=os.path.join(_REPO_ROOT, 'temp', 'logs', 'mpc_*.csv'),
                        help='glob pattern for session CSVs')
    parser.add_argument('--json', default=None,
                        help='optional path to write the full JSON result')
    parser.add_argument('--no-plot', action='store_true',
                        help='skip figure generation')
    parser.add_argument('--report-dir',
                        default=os.path.join(_REPO_ROOT, 'temp', 'reports',
                                             'motion_onset_cogging_study'),
                        help='directory for PNG outputs')
    args = parser.parse_args()

    # Sanity: confirm motor params loaded.  If not, analyse_motion_onset will
    # emit rest_elec_angle_rad=None on every event and we can't proceed.
    motor = _load_motor_params()
    if motor.get('pole_pairs') is None or motor.get('mm_to_rev') is None:
        print('ERROR: motor_pole_pairs / mm_to_rev not loaded from hardware_config.yaml')
        print(f'       loaded: {motor}')
        return 2
    print(f"motor params: pole_pairs={motor['pole_pairs']}, "
          f"stator_slots={motor['stator_slots']}, "
          f"mm_to_rev[0]={motor['mm_to_rev'][0]:.6f}")

    csv_paths = sorted(glob.glob(args.glob))
    if not csv_paths:
        print(f'no CSVs matched glob {args.glob!r}')
        return 1
    print(f'scanning {len(csv_paths)} CSV(s) under {args.glob}')

    events = collect_events(csv_paths)
    print(f'collected {len(events)} motion-onset events with rest-angle data')
    if not events:
        return 1

    analysis = per_leg_analysis(events)

    print('\n=== Per-leg cogging verdict ===')
    for leg in range(6):
        entry = analysis[f'leg_{leg}']
        if 'verdict' in entry:
            print(f"  leg {leg}: n={entry['n_events']:3d}  "
                  f"leap={entry['leap_mm_mean']:.2f}±{entry['leap_mm_std']:.2f} mm  "
                  f"{entry['verdict']}")
        else:
            print(f"  leg {leg}: n={entry['n_events']} — {entry.get('skipped', '?')}")

    print('\n=== Aggregate leap-fit R² per harmonic (all-legs pooled) ===')
    theta_mech_all = np.array([e.rest_mech_angle_rad for e in events])
    leap_all = np.array([e.leap_mm for e in events])
    pooled_harmonics: dict = {}
    for n in HARMONIC_SWEEP:
        r2, amp, off = _sinusoidal_fit_r2(theta_mech_all, leap_all, n)
        dist = _nearest_detent_distance(theta_mech_all, n)
        rho = _spearman(-dist, leap_all)
        pooled_harmonics[f'n={n}'] = {'r2': round(r2, 3),
                                      'amplitude_mm': round(amp, 3),
                                      'offset_mm': round(off, 3),
                                      'spearman_vs_detent': round(rho, 3)}
        print(f'  n={n:3d}: R²={r2:.3f}  amp={amp:.3f} mm  ρ_detent={rho:+.3f}')

    if not args.no_plot:
        _maybe_plot(events, args.report_dir)

    if args.json:
        payload = {
            'n_events': len(events),
            'n_sessions': len({e.session for e in events}),
            'motor_params': {
                'pole_pairs': motor['pole_pairs'],
                'stator_slots': motor['stator_slots'],
            },
            'per_leg': analysis,
            'pooled_harmonics': pooled_harmonics,
        }
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, 'w') as f:
            json.dump(payload, f, indent=2)
        print(f'\nJSON report: {args.json}')

    return 0


if __name__ == '__main__':
    sys.exit(main())
