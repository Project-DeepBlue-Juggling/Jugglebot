# -*- coding: utf-8 -*-
"""Diagnostic plot generation for /diagnose.

Generates matplotlib plots from MPC telemetry CSVs, annotated with anomalies
detected by the diagnosis engine.  Each plot is saved as a PNG next to the
source CSV.

Usage (standalone):
    python sim/analysis/plot_diagnosis.py <csv_path> [--categories all]

Typically invoked via diagnose.py --plots auto.
"""

from __future__ import annotations

import os
import sys
from typing import Any, Dict, List, Optional, Sequence

import matplotlib
matplotlib.use('Agg')  # headless — no display server needed on Jetson
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from sim.viz.telemetry import StepRecord  # noqa: E402

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

PLOT_CATEGORIES = [
    'legs', 'pose', 'tracking', 'solver',
    'velocity', 'hand', 'workspace', 'chatter',
]

# Stroke limits (mm) — mirrored from diagnose.py
STROKE_MIN_MM = 5.0
STROKE_MAX_MM = 275.0
STROKE_SOFT_MARGIN_MM = 15.0

# MPC budget line (ms)
DEFAULT_BUDGET_MS = 24.0

# Shared styling
_GRID_ALPHA = 0.3
_REF_STYLE = dict(color='tab:blue', linestyle='--', linewidth=1, label='ref')
_ACTUAL_STYLE = dict(color='tab:red', linestyle='-', linewidth=1, label='actual')
_CMD_STYLE = dict(color='tab:blue', linestyle='--', linewidth=1, label='cmd')


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _extract(records: Sequence[StepRecord], attr: str) -> np.ndarray:
    return np.array([getattr(r, attr) for r in records])


def _times(records: Sequence[StepRecord]) -> np.ndarray:
    return _extract(records, 'time')


def _annotate_discontinuities(ax: Any, disc_list: List[Dict], leg: Optional[int] = None) -> None:
    """Add vertical lines at discontinuity timestamps."""
    for d in disc_list:
        if leg is not None and d.get('leg') != leg:
            continue
        ax.axvline(d['time_s'], color='red', alpha=0.5, linewidth=0.8, linestyle=':')


def _save_and_close(fig: Any, path: str, dpi: int) -> None:
    fig.savefig(path, dpi=dpi, bbox_inches='tight')
    plt.close(fig)


# ---------------------------------------------------------------------------
# Auto-selection
# ---------------------------------------------------------------------------

def auto_select_plots(analysis_result: Dict[str, Any]) -> List[str]:
    """Pick relevant plot categories based on analysis flags."""
    selected = ['legs', 'tracking', 'solver']  # always included

    flags = analysis_result.get('flags', [])
    flag_messages = ' '.join(f.get('message', '') for f in flags)

    osc = analysis_result.get('oscillation', {})
    if osc.get('detected'):
        selected.append('chatter')

    ws = analysis_result.get('workspace', {})
    if (ws.get('margin_to_lower_mm', 999) < STROKE_SOFT_MARGIN_MM or
            ws.get('margin_to_upper_mm', 999) < STROKE_SOFT_MARGIN_MM):
        selected.append('workspace')

    disc = analysis_result.get('discontinuities', {})
    if disc.get('cmd_jumps') or disc.get('actual_jumps'):
        selected.append('velocity')

    # Include pose if tracking error is notable
    tracking = analysis_result.get('tracking', {})
    if tracking.get('position_rms_mm', 0) > 0.3 or tracking.get('orientation_rms_deg', 0) > 0.1:
        selected.append('pose')

    # Include hand if any hand data is non-zero
    if any('hand' in m.lower() for m in [flag_messages]):
        selected.append('hand')

    # Deduplicate while preserving order
    seen = set()
    result = []
    for c in selected:
        if c not in seen:
            seen.add(c)
            result.append(c)
    return result


# ---------------------------------------------------------------------------
# Individual plot functions
# ---------------------------------------------------------------------------

def _plot_legs(records: Sequence[StepRecord], analysis: Dict, path: str, dpi: int) -> None:
    """Cmd vs actual extension per leg (3x2 grid)."""
    times = _times(records)
    disc = analysis.get('discontinuities', {})
    cmd_jumps = disc.get('cmd_jumps', [])

    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    fig.suptitle('Leg Extensions — Cmd vs Actual', fontsize=13)

    for leg, ax in enumerate(axes.flat):
        cmd = _extract(records, f'cmd_ext_{leg}')
        actual = _extract(records, f'actual_ext_{leg}')
        ax.plot(times, cmd, **_CMD_STYLE)
        ax.plot(times, actual, **_ACTUAL_STYLE)
        _annotate_discontinuities(ax, cmd_jumps, leg=leg)
        ax.set_ylabel(f'Leg {leg} (mm)')
        ax.legend(loc='upper right', fontsize=7)
        ax.grid(True, alpha=_GRID_ALPHA)

    axes[-1, 0].set_xlabel('Time (s)')
    axes[-1, 1].set_xlabel('Time (s)')
    fig.tight_layout()
    _save_and_close(fig, path, dpi)


def _plot_pose(records: Sequence[StepRecord], analysis: Dict, path: str, dpi: int) -> None:
    """Ref vs actual 6-DoF (3x2 grid)."""
    times = _times(records)
    dof_labels = ['x (mm)', 'y (mm)', 'z (mm)', 'rx (rad)', 'ry (rad)', 'rz (rad)']
    ref_keys = ['ref_pose_x', 'ref_pose_y', 'ref_pose_z',
                'ref_pose_rx', 'ref_pose_ry', 'ref_pose_rz']
    act_keys = ['actual_pose_x', 'actual_pose_y', 'actual_pose_z',
                'actual_pose_rx', 'actual_pose_ry', 'actual_pose_rz']

    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    fig.suptitle('Platform Pose — Ref vs Actual', fontsize=13)

    for idx, ax in enumerate(axes.flat):
        ref = _extract(records, ref_keys[idx])
        act = _extract(records, act_keys[idx])
        ax.plot(times, ref, **_REF_STYLE)
        ax.plot(times, act, **_ACTUAL_STYLE)
        ax.set_ylabel(dof_labels[idx])
        ax.legend(loc='upper right', fontsize=7)
        ax.grid(True, alpha=_GRID_ALPHA)

    axes[-1, 0].set_xlabel('Time (s)')
    axes[-1, 1].set_xlabel('Time (s)')
    fig.tight_layout()
    _save_and_close(fig, path, dpi)


def _plot_tracking(records: Sequence[StepRecord], analysis: Dict, path: str, dpi: int) -> None:
    """Position error (mm) + orientation error (deg) over time."""
    times = _times(records)
    pos_err = _extract(records, 'tracking_error_mm')
    ori_err = _extract(records, 'tracking_error_deg')

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 6), sharex=True)
    fig.suptitle('Tracking Error', fontsize=13)

    ax1.plot(times, pos_err, 'tab:red', linewidth=1)
    ax1.set_ylabel('Position error (mm)')
    ax1.grid(True, alpha=_GRID_ALPHA)

    # Annotate steady-state region if detected
    ss = analysis.get('steady_state', {})
    if ss.get('has_steady_state'):
        ss_start = ss['ss_start_s']
        ax1.axvline(ss_start, color='green', linestyle='--', alpha=0.6, label='SS start')
        ax1.axhspan(0, ss.get('ss_rms_mm', 0) * 2, alpha=0.08, color='green')
        ax1.legend(fontsize=7)

    ax2.plot(times, ori_err, 'tab:blue', linewidth=1)
    ax2.set_ylabel('Orientation error (deg)')
    ax2.set_xlabel('Time (s)')
    ax2.grid(True, alpha=_GRID_ALPHA)

    fig.tight_layout()
    _save_and_close(fig, path, dpi)


def _plot_solver(records: Sequence[StepRecord], analysis: Dict, path: str, dpi: int) -> None:
    """Solve time timeseries + histogram with budget line."""
    times = _times(records)
    solve_ms = _extract(records, 'solve_time_ms')
    budget = analysis.get('solve_times', {}).get('budget_ms', DEFAULT_BUDGET_MS)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle('MPC Solver Performance', fontsize=13)

    # Timeseries: color violations red
    within = solve_ms <= budget
    ax1.scatter(times[within], solve_ms[within], s=6, color='tab:blue', alpha=0.7, label='within budget')
    if np.any(~within):
        ax1.scatter(times[~within], solve_ms[~within], s=12, color='tab:red', alpha=0.9,
                    marker='x', label='over budget')
    ax1.axhline(budget, color='red', linestyle='--', alpha=0.6, label=f'budget ({budget:.0f} ms)')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Solve time (ms)')
    ax1.legend(fontsize=7)
    ax1.grid(True, alpha=_GRID_ALPHA)

    # Histogram
    valid = solve_ms[solve_ms > 0]
    if len(valid) > 0:
        ax2.hist(valid, bins=min(50, max(10, len(valid) // 5)), edgecolor='black', alpha=0.7)
        ax2.axvline(budget, color='red', linestyle='--', label=f'budget ({budget:.0f} ms)')

        # Annotate percentiles
        st = analysis.get('solve_times', {})
        if st.get('p50_ms'):
            ax2.axvline(st['p50_ms'], color='green', linestyle=':', alpha=0.7,
                        label=f'p50={st["p50_ms"]:.1f}ms')
        if st.get('p95_ms'):
            ax2.axvline(st['p95_ms'], color='orange', linestyle=':', alpha=0.7,
                        label=f'p95={st["p95_ms"]:.1f}ms')

    ax2.set_xlabel('Solve time (ms)')
    ax2.set_ylabel('Count')
    ax2.legend(fontsize=7)
    ax2.grid(True, alpha=_GRID_ALPHA)

    fig.tight_layout()
    _save_and_close(fig, path, dpi)


def _plot_velocity(records: Sequence[StepRecord], analysis: Dict, path: str, dpi: int) -> None:
    """Leg velocity per leg (3x2 grid)."""
    times = _times(records)
    disc = analysis.get('discontinuities', {})
    actual_jumps = disc.get('actual_jumps', [])

    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    fig.suptitle('Leg Velocities', fontsize=13)

    for leg, ax in enumerate(axes.flat):
        vel = _extract(records, f'leg_vel_{leg}')
        ax.plot(times, vel, 'tab:blue', linewidth=1)
        _annotate_discontinuities(ax, actual_jumps, leg=leg)
        ax.set_ylabel(f'Leg {leg} vel')
        ax.grid(True, alpha=_GRID_ALPHA)

    axes[-1, 0].set_xlabel('Time (s)')
    axes[-1, 1].set_xlabel('Time (s)')
    fig.tight_layout()
    _save_and_close(fig, path, dpi)


def _plot_hand(records: Sequence[StepRecord], analysis: Dict, path: str, dpi: int) -> None:
    """Hand cmd vs actual position + velocity."""
    times = _times(records)
    cmd = _extract(records, 'hand_cmd_mm')
    pos = _extract(records, 'hand_pos_mm')
    vel = _extract(records, 'hand_vel_mmps')

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 6), sharex=True)
    fig.suptitle('Hand Actuator', fontsize=13)

    ax1.plot(times, cmd, **_CMD_STYLE)
    ax1.plot(times, pos, **_ACTUAL_STYLE)
    ax1.set_ylabel('Position (mm)')
    ax1.legend(fontsize=7)
    ax1.grid(True, alpha=_GRID_ALPHA)

    ax2.plot(times, vel, 'tab:purple', linewidth=1, label='velocity')
    ax2.set_ylabel('Velocity (mm/s)')
    ax2.set_xlabel('Time (s)')
    ax2.legend(fontsize=7)
    ax2.grid(True, alpha=_GRID_ALPHA)

    fig.tight_layout()
    _save_and_close(fig, path, dpi)


def _plot_workspace(records: Sequence[StepRecord], analysis: Dict, path: str, dpi: int) -> None:
    """Extension range per leg as horizontal bars with stroke limits."""
    ws = analysis.get('workspace', {})
    per_leg = ws.get('per_leg', [])
    if not per_leg:
        return

    fig, ax = plt.subplots(figsize=(10, 5))
    fig.suptitle('Workspace Usage', fontsize=13)

    legs = list(range(6))
    mins = [pl['min_mm'] for pl in per_leg]
    maxs = [pl['max_mm'] for pl in per_leg]
    widths = [mx - mn for mn, mx in zip(mins, maxs)]

    bars = ax.barh(legs, widths, left=mins, height=0.5, color='tab:blue', alpha=0.7)

    # Stroke limits
    ax.axvline(STROKE_MIN_MM, color='red', linestyle='--', linewidth=1.5, label=f'lower limit ({STROKE_MIN_MM}mm)')
    ax.axvline(STROKE_MAX_MM, color='red', linestyle='--', linewidth=1.5, label=f'upper limit ({STROKE_MAX_MM}mm)')

    # Soft margins
    ax.axvspan(STROKE_MIN_MM, STROKE_MIN_MM + STROKE_SOFT_MARGIN_MM,
               alpha=0.1, color='red')
    ax.axvspan(STROKE_MAX_MM - STROKE_SOFT_MARGIN_MM, STROKE_MAX_MM,
               alpha=0.1, color='red')

    ax.set_yticks(legs)
    ax.set_yticklabels([f'Leg {i}' for i in legs])
    ax.set_xlabel('Extension (mm)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=_GRID_ALPHA, axis='x')

    fig.tight_layout()
    _save_and_close(fig, path, dpi)


def _plot_chatter(records: Sequence[StepRecord], analysis: Dict, path: str, dpi: int) -> None:
    """Command delta sign visualization for oscillation detection."""
    times = _times(records)

    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    fig.suptitle('Command Chatter (sign of cmd delta)', fontsize=13)

    osc = analysis.get('oscillation', {})
    per_leg_chatter = osc.get('per_leg_chatter', [0.0] * 6)

    for leg, ax in enumerate(axes.flat):
        cmd = _extract(records, f'cmd_ext_{leg}')
        delta = np.diff(cmd)
        signs = np.sign(delta)

        # Color: +1 blue, -1 red, 0 gray
        colors = np.where(signs > 0, 1.0, np.where(signs < 0, -1.0, 0.0))
        ax.bar(times[1:], colors, width=np.median(np.diff(times)) * 0.8,
               color=[('tab:blue' if c > 0 else 'tab:red' if c < 0 else 'lightgray') for c in colors],
               alpha=0.7)
        chatter_val = per_leg_chatter[leg] if leg < len(per_leg_chatter) else 0.0
        ax.set_ylabel(f'Leg {leg}\nchatter={chatter_val:.2f}')
        ax.set_yticks([-1, 0, 1])
        ax.set_yticklabels(['-', '0', '+'])
        ax.grid(True, alpha=_GRID_ALPHA)

    axes[-1, 0].set_xlabel('Time (s)')
    axes[-1, 1].set_xlabel('Time (s)')
    fig.tight_layout()
    _save_and_close(fig, path, dpi)


# ---------------------------------------------------------------------------
# Dispatcher
# ---------------------------------------------------------------------------

_PLOT_FUNCS = {
    'legs': _plot_legs,
    'pose': _plot_pose,
    'tracking': _plot_tracking,
    'solver': _plot_solver,
    'velocity': _plot_velocity,
    'hand': _plot_hand,
    'workspace': _plot_workspace,
    'chatter': _plot_chatter,
}


def generate_diagnostic_plots(
    records: Sequence[StepRecord],
    analysis_result: Dict[str, Any],
    output_prefix: str,
    categories: Optional[List[str]] = None,
    dpi: int = 150,
) -> Dict[str, str]:
    """Generate diagnostic plots and return {category: filepath} dict.

    Parameters
    ----------
    records : list of StepRecord
        Loaded telemetry records.
    analysis_result : dict
        Output of analyse_csv() + generate_flags().
    output_prefix : str
        Path prefix for PNGs, e.g. "temp/logs/mpc_20260401_153529".
        Each plot appends "_{category}.png".
    categories : list of str, optional
        Which plots to generate.  None = auto-select based on flags.
    dpi : int
        PNG resolution.

    Returns
    -------
    dict mapping category name to saved file path.
    """
    if categories is None:
        categories = auto_select_plots(analysis_result)

    result = {}
    for cat in categories:
        func = _PLOT_FUNCS.get(cat)
        if func is None:
            continue
        path = f'{output_prefix}_{cat}.png'
        try:
            func(records, analysis_result, path, dpi)
            result[cat] = os.path.abspath(path)
        except Exception as exc:
            # Don't let a plot failure break the diagnosis
            print(f'[plot_diagnosis] Warning: failed to generate {cat} plot: {exc}',
                  file=sys.stderr)

    return result


def parse_categories(spec: str) -> Optional[List[str]]:
    """Parse --plots argument into a category list.

    Returns None for 'auto', empty list for 'none', or validated list.
    """
    spec = spec.strip().lower()
    if spec == 'auto':
        return None
    if spec == 'none':
        return []
    if spec == 'all':
        return list(PLOT_CATEGORIES)
    cats = [c.strip() for c in spec.split(',') if c.strip()]
    valid = [c for c in cats if c in _PLOT_FUNCS]
    return valid
