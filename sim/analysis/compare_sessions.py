# -*- coding: utf-8 -*-
"""Hardware session comparison tool.

Compares two MPC telemetry sessions (before/after a fix) and produces:
  1. A structured delta table showing metric changes
  2. An optional interactive Plotly HTML report with dual-trace overlays

Distinct from compare.py which handles sim-vs-hardware comparison (prediction
accuracy). This module handles hardware-vs-hardware comparison (before/after).

Usage:
    python sim/analysis/compare_sessions.py <csv_a> <csv_b>
    python sim/analysis/compare_sessions.py <csv_a> <csv_b> --json
    python sim/analysis/compare_sessions.py <csv_a> <csv_b> --html report.html
    python sim/analysis/compare_sessions.py <csv_a> <csv_b> --labels "before fix" "after fix"
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

# Ensure sim/ and repo root are importable
_analysis_dir = os.path.dirname(os.path.abspath(__file__))
_sim_dir = os.path.dirname(_analysis_dir)
if _sim_dir not in sys.path:
    sys.path.insert(0, _sim_dir)
_repo_root = os.path.dirname(_sim_dir)
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)

from analysis.compare import load_csv, _align_time, _extract_array
from analysis.diagnose import analyse_csv, generate_flags


# ---------------------------------------------------------------------------
# Comparison metrics
# ---------------------------------------------------------------------------

def _pct_change(before: float, after: float) -> Optional[float]:
    """Compute percentage change. Returns None if before is near zero."""
    if abs(before) < 1e-9:
        return None
    return round((after - before) / abs(before) * 100, 1)


def _delta_str(before: float, after: float, unit: str = '',
               lower_is_better: bool = True) -> str:
    """Format a delta as a human-readable string."""
    pct = _pct_change(before, after)
    if pct is None:
        if after == before:
            return 'unchanged'
        return f'{after:.3f}{unit}'

    if abs(pct) < 0.5:
        return 'unchanged'

    direction = 'down' if pct < 0 else 'up'
    improved = (pct < 0) if lower_is_better else (pct > 0)
    return f'{abs(pct):.0f}% {direction}' + (' (improved)' if improved else ' (worse)')


def _count_flags(flags: List[Dict], severity: str) -> int:
    """Count flags of a given severity."""
    return sum(1 for f in flags if f.get('severity') == severity)


def _verdict(flags: List[Dict]) -> str:
    """Derive verdict from flags (same logic as /diagnose)."""
    errors = _count_flags(flags, 'error')
    if errors > 0:
        return 'FAIL'
    warnings = _count_flags(flags, 'warning')
    if warnings > 0:
        return 'NEEDS_ATTENTION'
    return 'PASS'


def compare_sessions(csv_a: str, csv_b: str,
                     labels: Optional[Tuple[str, str]] = None) -> Dict[str, Any]:
    """Compare two hardware sessions and return structured delta metrics.

    Args:
        csv_a: Path to the "before" CSV
        csv_b: Path to the "after" CSV
        labels: Optional (label_a, label_b) tuple, defaults to filenames

    Returns:
        Dict with before/after/delta for each key metric.
    """
    records_a = load_csv(csv_a)
    records_b = load_csv(csv_b)

    if not records_a or not records_b:
        return {'error': 'One or both CSVs are empty or could not be loaded'}

    result_a = analyse_csv(records_a)
    result_b = analyse_csv(records_b)
    flags_a = generate_flags(result_a)
    flags_b = generate_flags(result_b)

    label_a = labels[0] if labels else os.path.splitext(os.path.basename(csv_a))[0]
    label_b = labels[1] if labels else os.path.splitext(os.path.basename(csv_b))[0]

    # Extract key metrics from both
    tracking_a = result_a.get('tracking', {})
    tracking_b = result_b.get('tracking', {})
    solve_a = result_a.get('solve_times', {})
    solve_b = result_b.get('solve_times', {})
    osc_a = result_a.get('oscillation', {})
    osc_b = result_b.get('oscillation', {})
    disc_a = result_a.get('discontinuities', {})
    disc_b = result_b.get('discontinuities', {})
    ss_a = result_a.get('steady_state', {})
    ss_b = result_b.get('steady_state', {})

    # Build comparison rows
    rows = []

    def add_row(name: str, val_a: float, val_b: float, unit: str = '',
                fmt: str = '.3f', lower_is_better: bool = True):
        rows.append({
            'metric': name,
            'before': f'{val_a:{fmt}}{unit}',
            'after': f'{val_b:{fmt}}{unit}',
            'before_raw': val_a,
            'after_raw': val_b,
            'delta': _delta_str(val_a, val_b, unit, lower_is_better),
            'pct_change': _pct_change(val_a, val_b),
            'improved': ((val_b < val_a) if lower_is_better
                         else (val_b > val_a)) if val_a != val_b else None,
        })

    # Tracking
    add_row('Tracking RMS (mm)',
            tracking_a.get('position_rms_mm', 0),
            tracking_b.get('position_rms_mm', 0), '')
    add_row('Tracking peak (mm)',
            tracking_a.get('position_peak_mm', 0),
            tracking_b.get('position_peak_mm', 0), '')

    # Worst leg
    worst_a = tracking_a.get('worst_leg', 0)
    worst_b = tracking_b.get('worst_leg', 0)
    worst_rms_a = tracking_a.get('per_leg', [{}] * 6)[worst_a].get('rms_mm', 0) if tracking_a.get('per_leg') else 0
    worst_rms_b = tracking_b.get('per_leg', [{}] * 6)[worst_b].get('rms_mm', 0) if tracking_b.get('per_leg') else 0
    rows.append({
        'metric': 'Worst leg',
        'before': f'leg {worst_a} ({worst_rms_a:.3f}mm)',
        'after': f'leg {worst_b} ({worst_rms_b:.3f}mm)',
        'before_raw': worst_rms_a,
        'after_raw': worst_rms_b,
        'delta': _delta_str(worst_rms_a, worst_rms_b, 'mm'),
        'pct_change': _pct_change(worst_rms_a, worst_rms_b),
        'improved': worst_rms_b < worst_rms_a if worst_rms_a != worst_rms_b else None,
    })

    add_row('Worst leg ratio',
            tracking_a.get('worst_leg_ratio', 0),
            tracking_b.get('worst_leg_ratio', 0), 'x', '.1f')

    # Solve times
    add_row('Solve time p50 (ms)',
            solve_a.get('p50_ms', 0),
            solve_b.get('p50_ms', 0), '', '.1f')
    add_row('Solve time p95 (ms)',
            solve_a.get('p95_ms', 0),
            solve_b.get('p95_ms', 0), '', '.1f')
    add_row('Solve time max (ms)',
            solve_a.get('max_ms', 0),
            solve_b.get('max_ms', 0), '', '.1f')
    add_row('Budget violations (%)',
            solve_a.get('budget_violation_pct', 0),
            solve_b.get('budget_violation_pct', 0), '%', '.1f')

    # Discontinuities
    n_disc_a = len(disc_a.get('cmd_jumps', []))
    n_disc_b = len(disc_b.get('cmd_jumps', []))
    rows.append({
        'metric': 'Command discontinuities',
        'before': str(n_disc_a),
        'after': str(n_disc_b),
        'before_raw': n_disc_a,
        'after_raw': n_disc_b,
        'delta': 'resolved' if n_disc_a > 0 and n_disc_b == 0
                 else _delta_str(n_disc_a, n_disc_b, ''),
        'pct_change': _pct_change(n_disc_a, n_disc_b),
        'improved': n_disc_b < n_disc_a if n_disc_a != n_disc_b else None,
    })

    # Oscillation
    osc_detected_a = osc_a.get('detected', False)
    osc_detected_b = osc_b.get('detected', False)
    rows.append({
        'metric': 'Oscillation detected',
        'before': 'yes' if osc_detected_a else 'no',
        'after': 'yes' if osc_detected_b else 'no',
        'before_raw': int(osc_detected_a),
        'after_raw': int(osc_detected_b),
        'delta': ('resolved' if osc_detected_a and not osc_detected_b
                  else 'new' if not osc_detected_a and osc_detected_b
                  else 'unchanged'),
        'pct_change': None,
        'improved': (osc_detected_a and not osc_detected_b) if osc_detected_a != osc_detected_b else None,
    })

    # Steady-state error
    if ss_a.get('has_steady_state') or ss_b.get('has_steady_state'):
        add_row('SS error RMS (mm)',
                ss_a.get('ss_rms_mm', 0),
                ss_b.get('ss_rms_mm', 0), '')

    # Flags summary
    err_a = _count_flags(flags_a, 'error')
    err_b = _count_flags(flags_b, 'error')
    warn_a = _count_flags(flags_a, 'warning')
    warn_b = _count_flags(flags_b, 'warning')
    rows.append({
        'metric': 'Flags (error/warning)',
        'before': f'{err_a}/{warn_a}',
        'after': f'{err_b}/{warn_b}',
        'before_raw': err_a + warn_a,
        'after_raw': err_b + warn_b,
        'delta': _delta_str(err_a + warn_a, err_b + warn_b, ''),
        'pct_change': _pct_change(err_a + warn_a, err_b + warn_b),
        'improved': (err_b + warn_b) < (err_a + warn_a),
    })

    # Verdict
    verdict_a = _verdict(flags_a)
    verdict_b = _verdict(flags_b)
    _VERDICT_ORDER = {'PASS': 0, 'NEEDS_ATTENTION': 1, 'FAIL': 2}
    rows.append({
        'metric': 'Verdict',
        'before': verdict_a,
        'after': verdict_b,
        'before_raw': _VERDICT_ORDER.get(verdict_a, 3),
        'after_raw': _VERDICT_ORDER.get(verdict_b, 3),
        'delta': ('improved' if _VERDICT_ORDER.get(verdict_b, 3) < _VERDICT_ORDER.get(verdict_a, 3)
                  else 'worse' if _VERDICT_ORDER.get(verdict_b, 3) > _VERDICT_ORDER.get(verdict_a, 3)
                  else 'unchanged'),
        'pct_change': None,
        'improved': _VERDICT_ORDER.get(verdict_b, 3) < _VERDICT_ORDER.get(verdict_a, 3),
    })

    return {
        'label_a': label_a,
        'label_b': label_b,
        'file_a': os.path.basename(csv_a),
        'file_b': os.path.basename(csv_b),
        'n_samples_a': result_a.get('n_samples', 0),
        'n_samples_b': result_b.get('n_samples', 0),
        'duration_a': result_a.get('duration_s', 0),
        'duration_b': result_b.get('duration_s', 0),
        'rows': rows,
    }


# ---------------------------------------------------------------------------
# Text output
# ---------------------------------------------------------------------------

def print_comparison_table(comparison: Dict[str, Any]) -> None:
    """Print a formatted comparison table to stdout."""
    label_a = comparison['label_a']
    label_b = comparison['label_b']

    print()
    print('=' * 75)
    print(f'  SESSION COMPARISON: {comparison["file_a"]} vs {comparison["file_b"]}')
    print('=' * 75)
    print()
    print(f'  {label_a}: {comparison["n_samples_a"]} samples, '
          f'{comparison["duration_a"]:.1f}s')
    print(f'  {label_b}: {comparison["n_samples_b"]} samples, '
          f'{comparison["duration_b"]:.1f}s')
    print()

    # Column widths
    w_metric = max(len(r['metric']) for r in comparison['rows']) + 2
    w_before = max(max(len(r['before']) for r in comparison['rows']), len(label_a)) + 2
    w_after = max(max(len(r['after']) for r in comparison['rows']), len(label_b)) + 2
    w_delta = max(max(len(r['delta']) for r in comparison['rows']), 5) + 2

    # Header
    print(f'  {"Metric":<{w_metric}s} {label_a:>{w_before}s} {label_b:>{w_after}s} {"Delta":>{w_delta}s}')
    print(f'  {"-" * w_metric} {"-" * w_before} {"-" * w_after} {"-" * w_delta}')

    for row in comparison['rows']:
        print(f'  {row["metric"]:<{w_metric}s} {row["before"]:>{w_before}s} '
              f'{row["after"]:>{w_after}s} {row["delta"]:>{w_delta}s}')

    print()
    print('=' * 75)


# ---------------------------------------------------------------------------
# Plotly HTML report
# ---------------------------------------------------------------------------

def generate_comparison_html(csv_a: str, csv_b: str,
                             labels: Optional[Tuple[str, str]] = None,
                             output_path: Optional[str] = None) -> Optional[str]:
    """Generate an interactive Plotly HTML report comparing two sessions.

    Returns the output path, or None if Plotly is not available.
    """
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        print('  Plotly not available — skipping interactive comparison report',
              file=sys.stderr)
        return None

    records_a = load_csv(csv_a)
    records_b = load_csv(csv_b)

    if not records_a or not records_b:
        return None

    label_a = labels[0] if labels else os.path.splitext(os.path.basename(csv_a))[0]
    label_b = labels[1] if labels else os.path.splitext(os.path.basename(csv_b))[0]

    time_a = _align_time(records_a)
    time_b = _align_time(records_b)

    # Determine output path
    if output_path is None:
        base_a = os.path.splitext(os.path.basename(csv_a))[0]
        base_b = os.path.splitext(os.path.basename(csv_b))[0]
        output_dir = os.path.join(_repo_root, 'temp', 'reports')
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, f'compare_{base_a}_vs_{base_b}.html')

    # --- Figure 1: Leg extensions (3x2 grid) ---
    fig_legs = make_subplots(rows=3, cols=2,
                             subplot_titles=[f'Leg {i}' for i in range(6)],
                             vertical_spacing=0.06, horizontal_spacing=0.08)

    for leg in range(6):
        row = leg // 2 + 1
        col = leg % 2 + 1
        show_legend = (leg == 0)

        cmd_a = _extract_array(records_a, f'cmd_ext_{leg}')
        act_a = _extract_array(records_a, f'actual_ext_{leg}')
        cmd_b = _extract_array(records_b, f'cmd_ext_{leg}')
        act_b = _extract_array(records_b, f'actual_ext_{leg}')

        fig_legs.add_trace(go.Scatter(
            x=time_a, y=cmd_a, name=f'{label_a} cmd',
            line=dict(color='steelblue', dash='dash', width=1),
            legendgroup='a_cmd', showlegend=show_legend,
        ), row=row, col=col)
        fig_legs.add_trace(go.Scatter(
            x=time_a, y=act_a, name=f'{label_a} actual',
            line=dict(color='steelblue', width=1.5),
            legendgroup='a_act', showlegend=show_legend,
        ), row=row, col=col)
        fig_legs.add_trace(go.Scatter(
            x=time_b, y=cmd_b, name=f'{label_b} cmd',
            line=dict(color='indianred', dash='dash', width=1),
            legendgroup='b_cmd', showlegend=show_legend,
        ), row=row, col=col)
        fig_legs.add_trace(go.Scatter(
            x=time_b, y=act_b, name=f'{label_b} actual',
            line=dict(color='indianred', width=1.5),
            legendgroup='b_act', showlegend=show_legend,
        ), row=row, col=col)

        fig_legs.update_yaxes(title_text='Extension (mm)', row=row, col=col)
        if row == 3:
            fig_legs.update_xaxes(title_text='Time (s)', row=row, col=col)

    fig_legs.update_layout(
        title=f'Leg Extensions: {label_a} vs {label_b}',
        height=900, template='plotly_white',
    )

    # --- Figure 2: Tracking error ---
    fig_tracking = make_subplots(rows=2, cols=1,
                                 subplot_titles=['Position Error', 'Orientation Error'],
                                 vertical_spacing=0.12)

    pos_err_a = _extract_array(records_a, 'tracking_error_mm')
    pos_err_b = _extract_array(records_b, 'tracking_error_mm')
    ori_err_a = _extract_array(records_a, 'tracking_error_deg')
    ori_err_b = _extract_array(records_b, 'tracking_error_deg')

    fig_tracking.add_trace(go.Scatter(
        x=time_a, y=pos_err_a, name=label_a,
        line=dict(color='steelblue', width=1.5),
    ), row=1, col=1)
    fig_tracking.add_trace(go.Scatter(
        x=time_b, y=pos_err_b, name=label_b,
        line=dict(color='indianred', width=1.5),
    ), row=1, col=1)
    fig_tracking.add_trace(go.Scatter(
        x=time_a, y=ori_err_a, name=label_a,
        line=dict(color='steelblue', width=1.5),
        showlegend=False,
    ), row=2, col=1)
    fig_tracking.add_trace(go.Scatter(
        x=time_b, y=ori_err_b, name=label_b,
        line=dict(color='indianred', width=1.5),
        showlegend=False,
    ), row=2, col=1)

    fig_tracking.update_yaxes(title_text='Error (mm)', row=1, col=1)
    fig_tracking.update_yaxes(title_text='Error (deg)', row=2, col=1)
    fig_tracking.update_xaxes(title_text='Time (s)', row=2, col=1)
    fig_tracking.update_layout(
        title=f'Tracking Error: {label_a} vs {label_b}',
        height=500, template='plotly_white',
    )

    # --- Figure 3: Solve times histogram ---
    fig_solve = go.Figure()

    solve_a = _extract_array(records_a, 'solve_time_ms')
    solve_b = _extract_array(records_b, 'solve_time_ms')
    valid_a = solve_a[solve_a > 0]
    valid_b = solve_b[solve_b > 0]

    if len(valid_a) > 0:
        fig_solve.add_trace(go.Histogram(
            x=valid_a, name=label_a,
            marker_color='steelblue', opacity=0.6, nbinsx=40,
        ))
    if len(valid_b) > 0:
        fig_solve.add_trace(go.Histogram(
            x=valid_b, name=label_b,
            marker_color='indianred', opacity=0.6, nbinsx=40,
        ))

    fig_solve.add_vline(x=24.0, line_dash='dash', line_color='red',
                        annotation_text='24ms budget')
    fig_solve.update_layout(
        title=f'Solve Time Distribution: {label_a} vs {label_b}',
        xaxis_title='Solve time (ms)',
        yaxis_title='Count',
        barmode='overlay',
        height=400, template='plotly_white',
    )

    # --- Combine into single HTML ---
    html_parts = [
        '<html><head>',
        '<meta charset="utf-8">',
        f'<title>Session Comparison: {label_a} vs {label_b}</title>',
        '<style>body{font-family:sans-serif;margin:20px;background:#fafafa} '
        'h1{color:#333} .section{margin:20px 0}</style>',
        '</head><body>',
        f'<h1>Session Comparison: {label_a} vs {label_b}</h1>',
        '<div class="section">',
        fig_legs.to_html(full_html=False, include_plotlyjs='cdn'),
        '</div><div class="section">',
        fig_tracking.to_html(full_html=False, include_plotlyjs=False),
        '</div><div class="section">',
        fig_solve.to_html(full_html=False, include_plotlyjs=False),
        '</div>',
        '</body></html>',
    ]

    with open(output_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(html_parts))

    return output_path


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Compare two MPC telemetry sessions (before/after)')
    parser.add_argument('csv_a', help='Path to the "before" CSV')
    parser.add_argument('csv_b', help='Path to the "after" CSV')
    parser.add_argument('--labels', nargs=2, metavar=('BEFORE', 'AFTER'),
                        help='Labels for the two sessions (default: filenames)')
    parser.add_argument('--json', action='store_true',
                        help='Output comparison as JSON')
    parser.add_argument('--html', metavar='PATH', nargs='?', const='auto',
                        help='Generate interactive Plotly HTML report '
                             '(default path: temp/reports/)')
    args = parser.parse_args()

    labels = tuple(args.labels) if args.labels else None

    comparison = compare_sessions(args.csv_a, args.csv_b, labels=labels)

    if 'error' in comparison:
        print(f'Error: {comparison["error"]}', file=sys.stderr)
        sys.exit(1)

    if args.json:
        print(json.dumps(comparison, indent=2))
    else:
        print_comparison_table(comparison)

    if args.html is not None:
        html_path = args.html if args.html != 'auto' else None
        result = generate_comparison_html(
            args.csv_a, args.csv_b, labels=labels, output_path=html_path)
        if result:
            print(f'\n  Interactive report: {result}')


if __name__ == '__main__':
    main()
