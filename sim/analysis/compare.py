"""Sim-to-real comparison tool.

Loads two StepRecord CSVs (a sim baseline and a hardware run of the same
trajectory) and produces:
  1. Overlay plots: ref vs sim-actual vs hw-actual for each DoF
  2. Leg extension overlay: commanded vs actual for each leg
  3. Gap metrics table: tracking error ratio, peak error, timing offset
  4. Pass/fail against the 30% sim-prediction-accuracy target
  5. Optional actuator time constant (tau) estimation from hardware data

Usage:
    python -m analysis.compare baselines/T1_baseline.csv logs/T1_hardware.csv
    python -m analysis.compare baselines/T1_baseline.csv logs/T1_hardware.csv --estimate-tau
    python -m analysis.compare baselines/T1_baseline.csv logs/T1_hardware.csv --save fig.png
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Sequence

import numpy as np

# Single path bootstrap (repo root, ros_ws pkg, config/generated);
# see sim/_paths.py.  Runnable entry scripts only — library modules under
# sim/ never touch sys.path.
_repo_root = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)
from sim._paths import bootstrap_paths  # noqa: E402
bootstrap_paths()

from sim.viz.telemetry import StepRecord, load_records


# ---------------------------------------------------------------------------
# CSV loading
# ---------------------------------------------------------------------------

def load_csv(path: str) -> list[StepRecord]:
    """Load a StepRecord CSV into a list of records.

    Thin alias kept for the existing consumers (diagnose, compare_sessions,
    plot_interactive, tools/); the canonical typed loader lives in
    controller.telemetry.load_records.
    """
    return load_records(path)


# ---------------------------------------------------------------------------
# Time alignment
# ---------------------------------------------------------------------------

def _align_time(records: list[StepRecord]) -> np.ndarray:
    """Extract time array, zero-based."""
    times = np.array([r.time for r in records])
    return times - times[0]


def _interpolate_to_common_time(
    times_a: np.ndarray,
    vals_a: np.ndarray,
    times_b: np.ndarray,
    vals_b: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Interpolate both signals onto the denser time grid.

    Returns (common_times, interp_a, interp_b).
    """
    # Use whichever has more samples as the common grid
    if len(times_a) >= len(times_b):
        t_common = times_a
    else:
        t_common = times_b

    # Clip to overlapping range
    t_start = max(times_a[0], times_b[0])
    t_end = min(times_a[-1], times_b[-1])
    mask = (t_common >= t_start) & (t_common <= t_end)
    t_common = t_common[mask]

    interp_a = np.interp(t_common, times_a, vals_a)
    interp_b = np.interp(t_common, times_b, vals_b)
    return t_common, interp_a, interp_b


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------

_DOF_LABELS = ['x (mm)', 'y (mm)', 'z (mm)', 'rx (rad)', 'ry (rad)', 'rz (rad)']
_POSE_REF_KEYS = [f'ref_pose_{c}' for c in ('x', 'y', 'z', 'rx', 'ry', 'rz')]
_POSE_ACT_KEYS = [f'actual_pose_{c}' for c in ('x', 'y', 'z', 'rx', 'ry', 'rz')]


def _extract_array(records: list[StepRecord], attr: str) -> np.ndarray:
    return np.array([getattr(r, attr) for r in records])


def compute_gap_metrics(
    sim_records: list[StepRecord],
    hw_records: list[StepRecord],
) -> dict:
    """Compute quantitative sim-to-real gap metrics.

    Returns a dict with per-DoF and aggregate metrics.
    """
    sim_time = _align_time(sim_records)
    hw_time = _align_time(hw_records)

    sim_pos_err = _extract_array(sim_records, 'tracking_error_mm')
    hw_pos_err = _extract_array(hw_records, 'tracking_error_mm')
    sim_ori_err = _extract_array(sim_records, 'tracking_error_deg')
    hw_ori_err = _extract_array(hw_records, 'tracking_error_deg')

    # Interpolate to common grid for direct comparison
    t_common, sim_pos_i, hw_pos_i = _interpolate_to_common_time(
        sim_time, sim_pos_err, hw_time, hw_pos_err)
    _, sim_ori_i, hw_ori_i = _interpolate_to_common_time(
        sim_time, sim_ori_err, hw_time, hw_ori_err)

    # Aggregate metrics
    sim_rms_mm = float(np.sqrt(np.mean(sim_pos_i**2)))
    hw_rms_mm = float(np.sqrt(np.mean(hw_pos_i**2)))
    sim_peak_mm = float(np.max(sim_pos_i))
    hw_peak_mm = float(np.max(hw_pos_i))
    sim_rms_deg = float(np.sqrt(np.mean(sim_ori_i**2)))
    hw_rms_deg = float(np.sqrt(np.mean(hw_ori_i**2)))

    # Prediction accuracy: how well does sim predict hardware error?
    # Ratio > 1 means hardware is worse than sim predicted.
    rms_ratio = hw_rms_mm / sim_rms_mm if sim_rms_mm > 1e-6 else float('inf')
    peak_ratio = hw_peak_mm / sim_peak_mm if sim_peak_mm > 1e-6 else float('inf')

    # Error gap: element-wise |hw_error - sim_error| on the common grid
    gap_mm = np.abs(hw_pos_i - sim_pos_i)

    # Per-DoF tracking comparison
    per_dof = {}
    for i, label in enumerate(_DOF_LABELS):
        sim_act = _extract_array(sim_records, _POSE_ACT_KEYS[i])
        hw_act = _extract_array(hw_records, _POSE_ACT_KEYS[i])
        _, sim_v, hw_v = _interpolate_to_common_time(
            sim_time, sim_act, hw_time, hw_act)
        per_dof[label] = {
            'sim_hw_rms_diff': float(np.sqrt(np.mean((sim_v - hw_v)**2))),
            'sim_hw_peak_diff': float(np.max(np.abs(sim_v - hw_v))),
        }

    # Solve times (hardware only — sim solve times are on a different machine)
    hw_solve = _extract_array(hw_records, 'solve_time_ms')
    sim_solve = _extract_array(sim_records, 'solve_time_ms')

    return {
        'sim_rms_mm': sim_rms_mm,
        'hw_rms_mm': hw_rms_mm,
        'sim_peak_mm': sim_peak_mm,
        'hw_peak_mm': hw_peak_mm,
        'sim_rms_deg': sim_rms_deg,
        'hw_rms_deg': hw_rms_deg,
        'rms_ratio': rms_ratio,
        'peak_ratio': peak_ratio,
        'gap_rms_mm': float(np.sqrt(np.mean(gap_mm**2))),
        'gap_peak_mm': float(np.max(gap_mm)),
        'per_dof': per_dof,
        'hw_solve_p50_ms': float(np.median(hw_solve)) if len(hw_solve) > 0 else 0,
        'hw_solve_p95_ms': float(np.percentile(hw_solve, 95)) if len(hw_solve) > 0 else 0,
        'sim_solve_p50_ms': float(np.median(sim_solve)) if len(sim_solve) > 0 else 0,
        'sim_solve_p95_ms': float(np.percentile(sim_solve, 95)) if len(sim_solve) > 0 else 0,
        'n_sim_steps': len(sim_records),
        'n_hw_steps': len(hw_records),
        'common_duration_s': float(t_common[-1] - t_common[0]) if len(t_common) > 0 else 0,
    }


def print_gap_report(metrics: dict, threshold_pct: float = 30.0) -> bool:
    """Print a formatted gap report. Returns True if within threshold."""
    print("=" * 70)
    print("  SIM-TO-REAL COMPARISON REPORT")
    print("=" * 70)
    print()

    print(f"  Sim steps: {metrics['n_sim_steps']},  "
          f"HW steps: {metrics['n_hw_steps']},  "
          f"Overlap: {metrics['common_duration_s']:.2f}s")
    print()

    print("  TRACKING ERROR COMPARISON")
    print("  " + "-" * 40)
    print(f"  {'Metric':<30s} {'Sim':>8s} {'Hardware':>8s} {'Ratio':>8s}")
    print(f"  {'RMS pos (mm)':<30s} {metrics['sim_rms_mm']:>8.3f} {metrics['hw_rms_mm']:>8.3f} {metrics['rms_ratio']:>8.2f}")
    print(f"  {'Peak pos (mm)':<30s} {metrics['sim_peak_mm']:>8.3f} {metrics['hw_peak_mm']:>8.3f} {metrics['peak_ratio']:>8.2f}")
    print(f"  {'RMS ori (deg)':<30s} {metrics['sim_rms_deg']:>8.4f} {metrics['hw_rms_deg']:>8.4f}")
    print()

    print("  SIM-TO-REAL GAP")
    print("  " + "-" * 40)
    print(f"  Gap RMS:  {metrics['gap_rms_mm']:.3f} mm")
    print(f"  Gap Peak: {metrics['gap_peak_mm']:.3f} mm")
    print()

    print("  PER-DOF ACTUAL POSE DIVERGENCE (sim vs hardware)")
    print("  " + "-" * 50)
    print(f"  {'DoF':<15s} {'RMS diff':>10s} {'Peak diff':>10s}")
    for label, dof_m in metrics['per_dof'].items():
        print(f"  {label:<15s} {dof_m['sim_hw_rms_diff']:>10.4f} {dof_m['sim_hw_peak_diff']:>10.4f}")
    print()

    print("  SOLVE TIMES")
    print("  " + "-" * 40)
    print(f"  {'':>12s} {'p50 (ms)':>10s} {'p95 (ms)':>10s}")
    print(f"  {'Sim':>12s} {metrics['sim_solve_p50_ms']:>10.2f} {metrics['sim_solve_p95_ms']:>10.2f}")
    print(f"  {'Hardware':>12s} {metrics['hw_solve_p50_ms']:>10.2f} {metrics['hw_solve_p95_ms']:>10.2f}")
    print()

    # Pass/fail: sim should predict hardware tracking error within threshold_pct
    within = abs(metrics['rms_ratio'] - 1.0) * 100
    passed = within <= threshold_pct
    status = "PASS" if passed else "FAIL"
    print(f"  VERDICT: {status}")
    print(f"  Sim-to-real RMS ratio: {metrics['rms_ratio']:.2f}x "
          f"({within:.1f}% {'<=' if passed else '>'} {threshold_pct:.0f}% threshold)")
    print("=" * 70)
    return passed


# ---------------------------------------------------------------------------
# Tau estimation
# ---------------------------------------------------------------------------

def estimate_tau(hw_records: list[StepRecord]) -> dict:
    """Estimate actuator time constant from hardware tracking data.

    Fits a first-order lag model to the command-vs-actual leg extension data.
    Returns per-leg tau estimates and the aggregate.
    """
    hw_time = _align_time(hw_records)
    dt = np.median(np.diff(hw_time))

    taus = {}
    for leg in range(6):
        cmd = _extract_array(hw_records, f'cmd_ext_{leg}')
        actual = _extract_array(hw_records, f'actual_ext_{leg}')

        # First-order lag: actual[k+1] = actual[k] + (dt/tau)*(cmd[k] - actual[k])
        # Rearrange: (actual[k+1] - actual[k]) / (cmd[k] - actual[k]) = dt/tau
        # Average over all steps where denominator is non-trivial
        d_actual = np.diff(actual)
        error = cmd[:-1] - actual[:-1]
        mask = np.abs(error) > 0.5  # ignore near-zero error (noise-dominated)

        if np.sum(mask) < 10:
            taus[f'leg_{leg}'] = float('nan')
            continue

        ratios = d_actual[mask] / error[mask]
        # dt/tau = median(ratios), so tau = dt / median(ratios)
        med_ratio = float(np.median(ratios))
        if med_ratio > 1e-6:
            taus[f'leg_{leg}'] = float(dt / med_ratio)
        else:
            taus[f'leg_{leg}'] = float('nan')

    valid = [v for v in taus.values() if not np.isnan(v) and v > 0]
    taus['aggregate'] = float(np.median(valid)) if valid else float('nan')

    return taus


def print_tau_report(taus: dict) -> None:
    """Print tau estimation results."""
    print()
    print("  ACTUATOR TIME CONSTANT ESTIMATION")
    print("  " + "-" * 40)
    for key, val in taus.items():
        if key == 'aggregate':
            continue
        status = f"{val*1000:.1f} ms" if not np.isnan(val) else "insufficient data"
        print(f"  {key}: {status}")
    agg = taus['aggregate']
    if not np.isnan(agg):
        print(f"\n  Aggregate tau: {agg*1000:.1f} ms")
        print(f"  Recommended MPCParams.tau = {agg:.4f}")
    else:
        print("\n  Could not estimate tau (insufficient dynamic data)")
    print()


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_comparison(
    sim_records: list[StepRecord],
    hw_records: list[StepRecord],
    save_path: str | None = None,
    show: bool = True,
) -> None:
    """Overlay plots comparing sim and hardware runs."""
    import matplotlib.pyplot as plt

    sim_time = _align_time(sim_records)
    hw_time = _align_time(hw_records)

    # --- Figure 1: Reference vs Sim-Actual vs HW-Actual per DoF ---
    fig1, axes1 = plt.subplots(3, 2, figsize=(16, 11), sharex=True)
    fig1.suptitle('Pose Tracking: Reference vs Sim vs Hardware', fontsize=14)

    for i, (ax, label) in enumerate(zip(axes1.flat, _DOF_LABELS)):
        ref_sim = _extract_array(sim_records, _POSE_REF_KEYS[i])
        act_sim = _extract_array(sim_records, _POSE_ACT_KEYS[i])
        act_hw = _extract_array(hw_records, _POSE_ACT_KEYS[i])

        ax.plot(sim_time, ref_sim, 'k--', label='reference', linewidth=1, alpha=0.7)
        ax.plot(sim_time, act_sim, 'b-', label='sim', linewidth=1.2)
        ax.plot(hw_time, act_hw, 'r-', label='hardware', linewidth=1.2)
        ax.set_ylabel(label)
        ax.legend(loc='upper right', fontsize=7)
        ax.grid(True, alpha=0.3)

    axes1[-1, 0].set_xlabel('Time (s)')
    axes1[-1, 1].set_xlabel('Time (s)')
    fig1.tight_layout()

    # --- Figure 2: Tracking error overlay ---
    fig2, (ax2a, ax2b) = plt.subplots(2, 1, figsize=(16, 6), sharex=True)
    fig2.suptitle('Tracking Error: Sim vs Hardware', fontsize=14)

    sim_pos_err = _extract_array(sim_records, 'tracking_error_mm')
    hw_pos_err = _extract_array(hw_records, 'tracking_error_mm')
    sim_ori_err = _extract_array(sim_records, 'tracking_error_deg')
    hw_ori_err = _extract_array(hw_records, 'tracking_error_deg')

    ax2a.plot(sim_time, sim_pos_err, 'b-', label='sim', linewidth=1)
    ax2a.plot(hw_time, hw_pos_err, 'r-', label='hardware', linewidth=1)
    ax2a.set_ylabel('Position error (mm)')
    ax2a.legend(fontsize=8)
    ax2a.grid(True, alpha=0.3)

    ax2b.plot(sim_time, sim_ori_err, 'b-', label='sim', linewidth=1)
    ax2b.plot(hw_time, hw_ori_err, 'r-', label='hardware', linewidth=1)
    ax2b.set_ylabel('Orientation error (deg)')
    ax2b.set_xlabel('Time (s)')
    ax2b.legend(fontsize=8)
    ax2b.grid(True, alpha=0.3)
    fig2.tight_layout()

    # --- Figure 3: Leg extensions (commanded vs actual, sim vs hw) ---
    fig3, axes3 = plt.subplots(3, 2, figsize=(16, 11), sharex=True)
    fig3.suptitle('Leg Extensions: Sim vs Hardware', fontsize=14)

    for leg, ax in enumerate(axes3.flat):
        sim_cmd = _extract_array(sim_records, f'cmd_ext_{leg}')
        sim_act = _extract_array(sim_records, f'actual_ext_{leg}')
        hw_cmd = _extract_array(hw_records, f'cmd_ext_{leg}')
        hw_act = _extract_array(hw_records, f'actual_ext_{leg}')

        ax.plot(sim_time, sim_cmd, 'b--', label='sim cmd', linewidth=0.8, alpha=0.6)
        ax.plot(sim_time, sim_act, 'b-', label='sim actual', linewidth=1.2)
        ax.plot(hw_time, hw_cmd, 'r--', label='hw cmd', linewidth=0.8, alpha=0.6)
        ax.plot(hw_time, hw_act, 'r-', label='hw actual', linewidth=1.2)
        ax.set_ylabel(f'Leg {leg} (mm)')
        ax.legend(loc='upper right', fontsize=6)
        ax.grid(True, alpha=0.3)

    axes3[-1, 0].set_xlabel('Time (s)')
    axes3[-1, 1].set_xlabel('Time (s)')
    fig3.tight_layout()

    # --- Figure 4: Solve times ---
    fig4, (ax4a, ax4b) = plt.subplots(1, 2, figsize=(14, 4))
    fig4.suptitle('MPC Solve Times', fontsize=14)

    sim_solve = _extract_array(sim_records, 'solve_time_ms')
    hw_solve = _extract_array(hw_records, 'solve_time_ms')

    for ax, data, label, color in [(ax4a, sim_solve, 'Sim', 'steelblue'),
                                    (ax4b, hw_solve, 'Hardware', 'indianred')]:
        valid = data[data > 0]
        if len(valid) > 0:
            ax.hist(valid, bins=40, edgecolor='black', alpha=0.7, color=color)
            ax.axvline(20.0, color='red', linestyle='--', label='20 ms budget')
            ax.set_xlabel('Solve time (ms)')
            ax.set_ylabel('Count')
            ax.set_title(f'{label} (p50={np.median(valid):.1f}, p95={np.percentile(valid, 95):.1f} ms)')
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)
        else:
            ax.text(0.5, 0.5, 'No data', ha='center', va='center', transform=ax.transAxes)
            ax.set_title(label)
    fig4.tight_layout()

    if save_path:
        base, ext = os.path.splitext(save_path)
        fig1.savefig(f'{base}_pose{ext}', dpi=150, bbox_inches='tight')
        fig2.savefig(f'{base}_error{ext}', dpi=150, bbox_inches='tight')
        fig3.savefig(f'{base}_legs{ext}', dpi=150, bbox_inches='tight')
        fig4.savefig(f'{base}_solve{ext}', dpi=150, bbox_inches='tight')
        print(f"  Figures saved: {base}_{{pose,error,legs,solve}}{ext}")

    if show:
        plt.show()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Compare sim baseline vs hardware telemetry CSVs")
    parser.add_argument('sim_csv', help="Path to sim baseline CSV")
    parser.add_argument('hw_csv', help="Path to hardware run CSV")
    parser.add_argument('--threshold', type=float, default=30.0,
                        help="Pass/fail threshold for RMS ratio (%%, default: 30)")
    parser.add_argument('--estimate-tau', action='store_true',
                        help="Estimate actuator time constant from hardware data")
    parser.add_argument('--save', metavar='PATH',
                        help="Save figures (e.g. --save comparison.png)")
    parser.add_argument('--no-plot', action='store_true',
                        help="Skip plotting (metrics report only)")
    args = parser.parse_args()

    print(f"\nLoading sim baseline: {args.sim_csv}")
    sim_records = load_csv(args.sim_csv)
    print(f"Loading hardware run: {args.hw_csv}")
    hw_records = load_csv(args.hw_csv)

    metrics = compute_gap_metrics(sim_records, hw_records)
    passed = print_gap_report(metrics, args.threshold)

    if args.estimate_tau:
        taus = estimate_tau(hw_records)
        print_tau_report(taus)

    if not args.no_plot:
        plot_comparison(sim_records, hw_records,
                        save_path=args.save, show=args.save is None)

    sys.exit(0 if passed else 1)


if __name__ == '__main__':
    main()
