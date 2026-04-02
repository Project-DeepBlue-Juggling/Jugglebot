# -*- coding: utf-8 -*-
"""Interactive diagnostic report generation using Plotly.

Generates a single self-contained HTML file with interactive plots
(zoom, pan, hover, legend toggle) and summary tables from MPC telemetry.

Usage (standalone):
    python sim/analysis/plot_interactive.py <csv_path> [--categories all]

Typically invoked via diagnose.py (default report mode).
"""

from __future__ import annotations

import os
import sys
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

# Ensure sim/ and repo root are importable
_analysis_dir = os.path.dirname(os.path.abspath(__file__))
_sim_dir = os.path.dirname(_analysis_dir)
if _sim_dir not in sys.path:
    sys.path.insert(0, _sim_dir)
_repo_root = os.path.dirname(_sim_dir)
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)

try:
    import plotly.graph_objects as go  # noqa: E402
    from plotly.subplots import make_subplots  # noqa: E402
    PLOTLY_AVAILABLE = True
except ImportError:
    PLOTLY_AVAILABLE = False

from viz.telemetry import StepRecord  # noqa: E402

# Re-use auto_select_plots and parse_categories from the static module
from analysis.plot_diagnosis import (  # noqa: E402
    auto_select_plots,
    parse_categories,
    PLOT_CATEGORIES,
    STROKE_MIN_MM,
    STROKE_MAX_MM,
    STROKE_SOFT_MARGIN_MM,
    DEFAULT_BUDGET_MS,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _extract(records: Sequence[StepRecord], attr: str) -> np.ndarray:
    return np.array([getattr(r, attr) for r in records])


def _times(records: Sequence[StepRecord]) -> np.ndarray:
    return _extract(records, 'time')


# Shared colors
_CMD_COLOR = '#1f77b4'    # blue
_ACTUAL_COLOR = '#d62728'  # red
_REF_COLOR = '#1f77b4'    # blue
_BUDGET_COLOR = '#d62728'  # red

# ---------------------------------------------------------------------------
# Individual plot functions — each returns a go.Figure
# ---------------------------------------------------------------------------

def _plot_legs(records: Sequence[StepRecord], analysis: Dict) -> go.Figure:
    """Cmd vs actual extension per leg (3x2 grid)."""
    times = _times(records)
    disc = analysis.get('discontinuities', {})
    cmd_jumps = disc.get('cmd_jumps', [])

    fig = make_subplots(rows=3, cols=2, shared_xaxes=True,
                        subplot_titles=[f'Leg {i}' for i in range(6)],
                        vertical_spacing=0.06)

    for leg in range(6):
        row, col = leg // 2 + 1, leg % 2 + 1
        cmd = _extract(records, f'cmd_ext_{leg}')
        actual = _extract(records, f'actual_ext_{leg}')

        fig.add_trace(go.Scatter(x=times, y=cmd, name=f'L{leg} cmd',
                                 line=dict(color=_CMD_COLOR, dash='dash', width=1),
                                 legendgroup=f'leg{leg}', showlegend=(leg == 0)),
                      row=row, col=col)
        fig.add_trace(go.Scatter(x=times, y=actual, name=f'L{leg} actual',
                                 line=dict(color=_ACTUAL_COLOR, width=1),
                                 legendgroup=f'leg{leg}', showlegend=(leg == 0)),
                      row=row, col=col)

        # Annotate discontinuities
        for d in cmd_jumps:
            if d.get('leg') == leg:
                fig.add_vline(x=d['time_s'], line_dash='dot', line_color='red',
                              opacity=0.5, row=row, col=col)

        fig.update_yaxes(title_text='mm', row=row, col=col)

    fig.update_xaxes(title_text='Time (s)', row=3, col=1)
    fig.update_xaxes(title_text='Time (s)', row=3, col=2)
    fig.update_layout(title='Leg Extensions — Cmd vs Actual', height=700)
    return fig


def _plot_pose(records: Sequence[StepRecord], analysis: Dict) -> go.Figure:
    """Ref vs actual 6-DoF (3x2 grid)."""
    times = _times(records)
    dof_labels = ['x (mm)', 'y (mm)', 'z (mm)', 'rx (rad)', 'ry (rad)', 'rz (rad)']
    ref_keys = ['ref_pose_x', 'ref_pose_y', 'ref_pose_z',
                'ref_pose_rx', 'ref_pose_ry', 'ref_pose_rz']
    act_keys = ['actual_pose_x', 'actual_pose_y', 'actual_pose_z',
                'actual_pose_rx', 'actual_pose_ry', 'actual_pose_rz']

    fig = make_subplots(rows=3, cols=2, shared_xaxes=True,
                        subplot_titles=dof_labels,
                        vertical_spacing=0.06)

    for idx in range(6):
        row, col = idx // 2 + 1, idx % 2 + 1
        ref = _extract(records, ref_keys[idx])
        act = _extract(records, act_keys[idx])

        fig.add_trace(go.Scatter(x=times, y=ref, name=f'{dof_labels[idx]} ref',
                                 line=dict(color=_REF_COLOR, dash='dash', width=1),
                                 legendgroup=f'dof{idx}', showlegend=(idx == 0)),
                      row=row, col=col)
        fig.add_trace(go.Scatter(x=times, y=act, name=f'{dof_labels[idx]} actual',
                                 line=dict(color=_ACTUAL_COLOR, width=1),
                                 legendgroup=f'dof{idx}', showlegend=(idx == 0)),
                      row=row, col=col)

        fig.update_yaxes(title_text=dof_labels[idx], row=row, col=col)

    fig.update_xaxes(title_text='Time (s)', row=3, col=1)
    fig.update_xaxes(title_text='Time (s)', row=3, col=2)
    fig.update_layout(title='Platform Pose — Ref vs Actual', height=700)
    return fig


def _plot_tracking(records: Sequence[StepRecord], analysis: Dict) -> go.Figure:
    """Position error (mm) + orientation error (deg) over time."""
    times = _times(records)
    pos_err = _extract(records, 'tracking_error_mm')
    ori_err = _extract(records, 'tracking_error_deg')

    fig = make_subplots(rows=2, cols=1, shared_xaxes=True,
                        subplot_titles=['Position Error', 'Orientation Error'],
                        vertical_spacing=0.12)

    fig.add_trace(go.Scatter(x=times, y=pos_err, name='pos error',
                             line=dict(color=_ACTUAL_COLOR, width=1)),
                  row=1, col=1)

    # Annotate steady-state region
    ss = analysis.get('steady_state', {})
    if ss.get('has_steady_state'):
        ss_start = ss['ss_start_s']
        fig.add_vline(x=ss_start, line_dash='dash', line_color='green',
                      opacity=0.6, annotation_text='SS start', row=1, col=1)

    fig.add_trace(go.Scatter(x=times, y=ori_err, name='ori error',
                             line=dict(color=_CMD_COLOR, width=1)),
                  row=2, col=1)

    fig.update_yaxes(title_text='mm', row=1, col=1)
    fig.update_yaxes(title_text='deg', row=2, col=1)
    fig.update_xaxes(title_text='Time (s)', row=2, col=1)
    fig.update_layout(title='Tracking Error', height=500)
    return fig


def _plot_solver(records: Sequence[StepRecord], analysis: Dict) -> go.Figure:
    """Solve time timeseries + histogram with budget line."""
    times = _times(records)
    solve_ms = _extract(records, 'solve_time_ms')
    budget = analysis.get('solve_times', {}).get('budget_ms', DEFAULT_BUDGET_MS)
    st = analysis.get('solve_times', {})

    fig = make_subplots(rows=1, cols=2,
                        subplot_titles=['Solve Time Timeline', 'Distribution'],
                        column_widths=[0.65, 0.35])

    # Timeline with budget violations highlighted
    within = solve_ms <= budget
    fig.add_trace(go.Scatter(
        x=times[within], y=solve_ms[within], mode='markers',
        marker=dict(size=4, color=_CMD_COLOR, opacity=0.7),
        name='within budget'), row=1, col=1)

    if np.any(~within):
        fig.add_trace(go.Scatter(
            x=times[~within], y=solve_ms[~within], mode='markers',
            marker=dict(size=6, color=_ACTUAL_COLOR, symbol='x', opacity=0.9),
            name='over budget'), row=1, col=1)

    fig.add_hline(y=budget, line_dash='dash', line_color='red', opacity=0.6,
                  annotation_text=f'budget ({budget:.0f}ms)', row=1, col=1)

    # Histogram
    valid = solve_ms[solve_ms > 0]
    if len(valid) > 0:
        fig.add_trace(go.Histogram(
            x=valid, nbinsx=min(50, max(10, len(valid) // 5)),
            marker_color=_CMD_COLOR, opacity=0.7, name='solve times',
            showlegend=False), row=1, col=2)

        fig.add_vline(x=budget, line_dash='dash', line_color='red',
                      opacity=0.6, row=1, col=2)
        if st.get('p50_ms'):
            fig.add_vline(x=st['p50_ms'], line_dash='dot', line_color='green',
                          opacity=0.7, annotation_text=f'p50={st["p50_ms"]:.1f}',
                          row=1, col=2)
        if st.get('p95_ms'):
            fig.add_vline(x=st['p95_ms'], line_dash='dot', line_color='orange',
                          opacity=0.7, annotation_text=f'p95={st["p95_ms"]:.1f}',
                          row=1, col=2)

    fig.update_xaxes(title_text='Time (s)', row=1, col=1)
    fig.update_yaxes(title_text='Solve time (ms)', row=1, col=1)
    fig.update_xaxes(title_text='Solve time (ms)', row=1, col=2)
    fig.update_yaxes(title_text='Count', row=1, col=2)
    fig.update_layout(title='MPC Solver Performance', height=400)
    return fig


def _plot_velocity(records: Sequence[StepRecord], analysis: Dict) -> go.Figure:
    """Leg velocity per leg (3x2 grid)."""
    times = _times(records)
    disc = analysis.get('discontinuities', {})
    actual_jumps = disc.get('actual_jumps', [])

    fig = make_subplots(rows=3, cols=2, shared_xaxes=True,
                        subplot_titles=[f'Leg {i}' for i in range(6)],
                        vertical_spacing=0.06)

    for leg in range(6):
        row, col = leg // 2 + 1, leg % 2 + 1
        vel = _extract(records, f'leg_vel_{leg}')

        fig.add_trace(go.Scatter(x=times, y=vel, name=f'L{leg} vel',
                                 line=dict(color=_CMD_COLOR, width=1),
                                 showlegend=False),
                      row=row, col=col)

        for d in actual_jumps:
            if d.get('leg') == leg:
                fig.add_vline(x=d['time_s'], line_dash='dot', line_color='red',
                              opacity=0.5, row=row, col=col)

        fig.update_yaxes(title_text='vel', row=row, col=col)

    fig.update_xaxes(title_text='Time (s)', row=3, col=1)
    fig.update_xaxes(title_text='Time (s)', row=3, col=2)
    fig.update_layout(title='Leg Velocities', height=700)
    return fig


def _plot_hand(records: Sequence[StepRecord], analysis: Dict) -> go.Figure:
    """Hand cmd vs actual position + velocity."""
    times = _times(records)
    cmd = _extract(records, 'hand_cmd_mm')
    pos = _extract(records, 'hand_pos_mm')
    vel = _extract(records, 'hand_vel_mmps')

    fig = make_subplots(rows=2, cols=1, shared_xaxes=True,
                        subplot_titles=['Hand Position', 'Hand Velocity'],
                        vertical_spacing=0.12)

    fig.add_trace(go.Scatter(x=times, y=cmd, name='cmd',
                             line=dict(color=_CMD_COLOR, dash='dash', width=1)),
                  row=1, col=1)
    fig.add_trace(go.Scatter(x=times, y=pos, name='actual',
                             line=dict(color=_ACTUAL_COLOR, width=1)),
                  row=1, col=1)

    fig.add_trace(go.Scatter(x=times, y=vel, name='velocity',
                             line=dict(color='purple', width=1)),
                  row=2, col=1)

    fig.update_yaxes(title_text='mm', row=1, col=1)
    fig.update_yaxes(title_text='mm/s', row=2, col=1)
    fig.update_xaxes(title_text='Time (s)', row=2, col=1)
    fig.update_layout(title='Hand Actuator', height=500)
    return fig


def _plot_workspace(records: Sequence[StepRecord], analysis: Dict) -> go.Figure:
    """Extension range per leg as horizontal bars with stroke limits."""
    ws = analysis.get('workspace', {})
    per_leg = ws.get('per_leg', [])
    if not per_leg:
        return go.Figure()

    legs = list(range(6))
    mins = [pl['min_mm'] for pl in per_leg]
    maxs = [pl['max_mm'] for pl in per_leg]
    leg_labels = [f'Leg {i}' for i in legs]

    fig = go.Figure()

    # Extension range bars
    fig.add_trace(go.Bar(
        y=leg_labels, x=[mx - mn for mn, mx in zip(mins, maxs)],
        base=mins, orientation='h', name='extension range',
        marker_color=_CMD_COLOR, opacity=0.7,
        hovertemplate='Leg %{y}<br>Range: %{base:.1f} – %{customdata:.1f} mm<extra></extra>',
        customdata=maxs))

    # Stroke limits
    fig.add_vline(x=STROKE_MIN_MM, line_dash='dash', line_color='red',
                  annotation_text=f'lower ({STROKE_MIN_MM}mm)')
    fig.add_vline(x=STROKE_MAX_MM, line_dash='dash', line_color='red',
                  annotation_text=f'upper ({STROKE_MAX_MM}mm)')

    # Soft margins
    fig.add_vrect(x0=STROKE_MIN_MM, x1=STROKE_MIN_MM + STROKE_SOFT_MARGIN_MM,
                  fillcolor='red', opacity=0.08, line_width=0)
    fig.add_vrect(x0=STROKE_MAX_MM - STROKE_SOFT_MARGIN_MM, x1=STROKE_MAX_MM,
                  fillcolor='red', opacity=0.08, line_width=0)

    fig.update_layout(title='Workspace Usage', height=350,
                      xaxis_title='Extension (mm)')
    return fig


def _plot_chatter(records: Sequence[StepRecord], analysis: Dict) -> go.Figure:
    """Command delta sign visualization for oscillation detection."""
    times = _times(records)
    osc = analysis.get('oscillation', {})
    per_leg_chatter = osc.get('per_leg_chatter', [0.0] * 6)

    fig = make_subplots(rows=3, cols=2, shared_xaxes=True,
                        subplot_titles=[
                            f'Leg {i} (chatter={per_leg_chatter[i]:.2f})'
                            if i < len(per_leg_chatter) else f'Leg {i}'
                            for i in range(6)],
                        vertical_spacing=0.06)

    for leg in range(6):
        row, col = leg // 2 + 1, leg % 2 + 1
        cmd = _extract(records, f'cmd_ext_{leg}')
        delta = np.diff(cmd)
        signs = np.sign(delta)

        # Split into positive and negative for coloring
        pos_signs = np.where(signs > 0, 1.0, 0.0)
        neg_signs = np.where(signs < 0, -1.0, 0.0)

        fig.add_trace(go.Bar(x=times[1:], y=pos_signs, name='+',
                             marker_color=_CMD_COLOR, opacity=0.7,
                             showlegend=(leg == 0)),
                      row=row, col=col)
        fig.add_trace(go.Bar(x=times[1:], y=neg_signs, name='-',
                             marker_color=_ACTUAL_COLOR, opacity=0.7,
                             showlegend=(leg == 0)),
                      row=row, col=col)

        fig.update_yaxes(tickvals=[-1, 0, 1], ticktext=['-', '0', '+'],
                         row=row, col=col)

    fig.update_xaxes(title_text='Time (s)', row=3, col=1)
    fig.update_xaxes(title_text='Time (s)', row=3, col=2)
    fig.update_layout(title='Command Chatter (sign of cmd delta)',
                      height=700, barmode='relative')
    return fig


# ---------------------------------------------------------------------------
# Summary tables as HTML
# ---------------------------------------------------------------------------

def _verdict_badge(flags: List[Dict]) -> str:
    severities = {f.get('severity', '').lower() for f in flags}
    if 'error' in severities:
        return '<span class="badge fail">FAIL</span>'
    if 'warning' in severities:
        return '<span class="badge warn">NEEDS ATTENTION</span>'
    return '<span class="badge pass">PASS</span>'


def _flags_table(flags: List[Dict]) -> str:
    if not flags:
        return '<p>No flags.</p>'
    rows = []
    for f in flags:
        sev = f.get('severity', 'info')
        cls = 'flag-error' if sev == 'error' else 'flag-warn' if sev == 'warning' else 'flag-info'
        rows.append(f'<tr class="{cls}"><td>{sev.upper()}</td>'
                    f'<td>{f.get("source", "")}</td>'
                    f'<td>{f.get("message", "")}</td></tr>')
    return (f'<table class="flags"><thead><tr><th>Severity</th><th>Source</th>'
            f'<th>Message</th></tr></thead><tbody>{"".join(rows)}</tbody></table>')


def _tracking_table(tracking: Dict) -> str:
    per_leg = tracking.get('per_leg', [])
    if not per_leg:
        return ''
    rows = []
    for pl in per_leg:
        rows.append(f'<tr><td>{pl["leg"]}</td><td>{pl["rms_mm"]:.3f}</td>'
                    f'<td>{pl["peak_mm"]:.3f}</td><td>{pl["mean_mm"]:.3f}</td></tr>')
    rows.append(f'<tr class="agg"><td><b>Aggregate</b></td>'
                f'<td>{tracking.get("position_rms_mm", 0):.3f}</td>'
                f'<td>{tracking.get("position_peak_mm", 0):.3f}</td>'
                f'<td>—</td></tr>')
    return (f'<table class="metrics"><thead><tr><th>Leg</th><th>RMS (mm)</th>'
            f'<th>Peak (mm)</th><th>Mean (mm)</th></tr></thead>'
            f'<tbody>{"".join(rows)}</tbody></table>')


def _solver_summary(solve_times: Dict) -> str:
    if not solve_times:
        return ''
    return (f'<div class="stats">'
            f'<span>p50: <b>{solve_times.get("p50_ms", 0):.1f}</b>ms</span>'
            f'<span>p95: <b>{solve_times.get("p95_ms", 0):.1f}</b>ms</span>'
            f'<span>p99: <b>{solve_times.get("p99_ms", 0):.1f}</b>ms</span>'
            f'<span>max: <b>{solve_times.get("max_ms", 0):.1f}</b>ms</span>'
            f'<span>violations: <b>{solve_times.get("budget_violations", 0)}</b>'
            f' ({solve_times.get("violation_pct", 0):.1f}%)</span>'
            f'</div>')


# ---------------------------------------------------------------------------
# Report assembly
# ---------------------------------------------------------------------------

_CSS = """
<style>
  body { font-family: -apple-system, system-ui, sans-serif; margin: 20px;
         background: #1a1a2e; color: #e0e0e0; }
  h1, h2, h3 { color: #e0e0e0; }
  .header { display: flex; align-items: center; gap: 20px; margin-bottom: 20px; }
  .badge { padding: 4px 12px; border-radius: 4px; font-weight: bold; font-size: 14px; }
  .badge.pass { background: #2d5a3d; color: #7dde8f; }
  .badge.warn { background: #5a4a2d; color: #ded07d; }
  .badge.fail { background: #5a2d2d; color: #de7d7d; }
  .meta { color: #a0a0a0; font-size: 13px; }
  table { border-collapse: collapse; margin: 10px 0; font-size: 13px; }
  th, td { padding: 4px 10px; border: 1px solid #333; text-align: right; }
  th { background: #2a2a3e; }
  tr.agg { background: #2a2a3e; }
  .flag-error td:first-child { color: #de7d7d; font-weight: bold; }
  .flag-warn td:first-child { color: #ded07d; font-weight: bold; }
  .flag-info td:first-child { color: #7dade0; }
  .flags td { text-align: left; }
  .stats { display: flex; gap: 20px; flex-wrap: wrap; margin: 8px 0;
           font-size: 13px; }
  .stats span { background: #2a2a3e; padding: 3px 10px; border-radius: 3px; }
  .section { margin: 20px 0; }
  .plot-container { margin: 15px 0; }
</style>
"""


def _build_html(result: Dict, figures: Dict[str, go.Figure]) -> str:
    """Assemble the full HTML report with embedded Plotly figures."""
    csv_name = os.path.basename(result.get('file', 'unknown'))
    duration = result.get('duration_s', 0)
    n_samples = result.get('n_samples', 0)
    flags = result.get('flags', [])
    tracking = result.get('tracking', {})
    solve_times = result.get('solve_times', {})

    # Build plot HTML divs
    plot_divs = []
    for cat, fig in figures.items():
        div_html = fig.to_html(full_html=False, include_plotlyjs=False,
                               config={'scrollZoom': True,
                                       'displayModeBar': True,
                                       'modeBarButtonsToAdd': ['toggleSpikelines']})
        plot_divs.append(f'<div class="plot-container" id="plot-{cat}">{div_html}</div>')

    plots_html = '\n'.join(plot_divs)

    # Plotly.js CDN (loaded once, shared by all plots)
    plotly_js = '<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>'

    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Diagnosis: {csv_name}</title>
{plotly_js}
{_CSS}
</head>
<body>
<div class="header">
  <h1>Diagnosis Report</h1>
  {_verdict_badge(flags)}
</div>
<p class="meta">{csv_name} &mdash; {duration:.1f}s, {n_samples} samples</p>

<div class="section">
  <h2>Flagged Issues ({len(flags)})</h2>
  {_flags_table(flags)}
</div>

<div class="section">
  <h2>Tracking Performance</h2>
  {_tracking_table(tracking)}
</div>

<div class="section">
  <h2>Solver Performance</h2>
  {_solver_summary(solve_times)}
</div>

<div class="section">
  <h2>Diagnostic Plots</h2>
  {plots_html}
</div>

</body>
</html>"""


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


def generate_interactive_report(
    records: Sequence[StepRecord],
    analysis_result: Dict[str, Any],
    output_path: str,
    categories: Optional[List[str]] = None,
) -> str:
    """Generate an interactive HTML report with Plotly plots.

    Parameters
    ----------
    records : list of StepRecord
        Loaded telemetry records.
    analysis_result : dict
        Output of analyse_csv() + generate_flags().
    output_path : str
        Path for the output HTML file.
    categories : list of str, optional
        Which plots to generate. None = auto-select based on flags.

    Returns
    -------
    str : absolute path to the generated HTML file.
    """
    if not PLOTLY_AVAILABLE:
        raise ImportError(
            'plotly is required for interactive reports. '
            'Install with: pip install plotly')

    if categories is None:
        categories = auto_select_plots(analysis_result)

    # Generate figures
    figures = {}
    for cat in categories:
        func = _PLOT_FUNCS.get(cat)
        if func is None:
            continue
        try:
            fig = func(records, analysis_result)
            # Apply dark theme to all figures
            fig.update_layout(
                template='plotly_dark',
                paper_bgcolor='#1a1a2e',
                plot_bgcolor='#16213e',
                font=dict(size=11),
                margin=dict(l=50, r=20, t=40, b=40),
                legend=dict(orientation='h', yanchor='bottom', y=1.02,
                            xanchor='right', x=1),
            )
            figures[cat] = fig
        except Exception as exc:
            print(f'[plot_interactive] Warning: failed to generate {cat}: {exc}',
                  file=sys.stderr)

    # Build and write HTML
    html = _build_html(analysis_result, figures)
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(html)

    return os.path.abspath(output_path)


# ---------------------------------------------------------------------------
# Session comparison
# ---------------------------------------------------------------------------

def generate_comparison_report(
    records_a: Sequence[StepRecord],
    result_a: Dict[str, Any],
    records_b: Sequence[StepRecord],
    result_b: Dict[str, Any],
    output_path: str,
    categories: Optional[List[str]] = None,
) -> str:
    """Generate an interactive comparison report overlaying two sessions.

    Plots from both sessions are overlaid with different opacity/dash styles
    so the user can compare before/after a fix.
    """
    if not PLOTLY_AVAILABLE:
        raise ImportError('plotly is required. Install with: pip install plotly')

    name_a = os.path.basename(result_a.get('file', 'session_A'))
    name_b = os.path.basename(result_b.get('file', 'session_B'))

    if categories is None:
        # Union of auto-selected categories from both sessions
        cats_a = set(auto_select_plots(result_a))
        cats_b = set(auto_select_plots(result_b))
        categories = list(cats_a | cats_b)

    times_a = _times(records_a)
    times_b = _times(records_b)

    figures = {}

    # Legs comparison
    if 'legs' in categories:
        fig = make_subplots(rows=3, cols=2, shared_xaxes=True,
                            subplot_titles=[f'Leg {i}' for i in range(6)],
                            vertical_spacing=0.06)
        for leg in range(6):
            row, col = leg // 2 + 1, leg % 2 + 1
            actual_a = _extract(records_a, f'actual_ext_{leg}')
            actual_b = _extract(records_b, f'actual_ext_{leg}')
            fig.add_trace(go.Scatter(
                x=times_a, y=actual_a, name=f'{name_a}',
                line=dict(color=_ACTUAL_COLOR, width=1),
                legendgroup='A', showlegend=(leg == 0)), row=row, col=col)
            fig.add_trace(go.Scatter(
                x=times_b, y=actual_b, name=f'{name_b}',
                line=dict(color='#2ca02c', width=1),
                legendgroup='B', showlegend=(leg == 0)), row=row, col=col)
            fig.update_yaxes(title_text='mm', row=row, col=col)
        fig.update_layout(title=f'Leg Extensions — {name_a} vs {name_b}', height=700)
        figures['legs'] = fig

    # Tracking comparison
    if 'tracking' in categories:
        fig = make_subplots(rows=2, cols=1, shared_xaxes=True,
                            subplot_titles=['Position Error', 'Orientation Error'],
                            vertical_spacing=0.12)
        pos_a = _extract(records_a, 'tracking_error_mm')
        pos_b = _extract(records_b, 'tracking_error_mm')
        ori_a = _extract(records_a, 'tracking_error_deg')
        ori_b = _extract(records_b, 'tracking_error_deg')

        fig.add_trace(go.Scatter(x=times_a, y=pos_a, name=f'{name_a}',
                                 line=dict(color=_ACTUAL_COLOR, width=1),
                                 legendgroup='A'), row=1, col=1)
        fig.add_trace(go.Scatter(x=times_b, y=pos_b, name=f'{name_b}',
                                 line=dict(color='#2ca02c', width=1),
                                 legendgroup='B'), row=1, col=1)
        fig.add_trace(go.Scatter(x=times_a, y=ori_a, name=f'{name_a} ori',
                                 line=dict(color=_ACTUAL_COLOR, dash='dash', width=1),
                                 legendgroup='A', showlegend=False), row=2, col=1)
        fig.add_trace(go.Scatter(x=times_b, y=ori_b, name=f'{name_b} ori',
                                 line=dict(color='#2ca02c', dash='dash', width=1),
                                 legendgroup='B', showlegend=False), row=2, col=1)
        fig.update_yaxes(title_text='mm', row=1, col=1)
        fig.update_yaxes(title_text='deg', row=2, col=1)
        fig.update_xaxes(title_text='Time (s)', row=2, col=1)
        fig.update_layout(title=f'Tracking Error — {name_a} vs {name_b}', height=500)
        figures['tracking'] = fig

    # Solver comparison
    if 'solver' in categories:
        fig = make_subplots(rows=1, cols=2,
                            subplot_titles=['Session A Timeline', 'Session B Timeline'],
                            column_widths=[0.5, 0.5])
        solve_a = _extract(records_a, 'solve_time_ms')
        solve_b = _extract(records_b, 'solve_time_ms')
        budget = result_a.get('solve_times', {}).get('budget_ms', DEFAULT_BUDGET_MS)

        fig.add_trace(go.Scatter(x=times_a, y=solve_a, mode='markers',
                                 marker=dict(size=4, color=_ACTUAL_COLOR, opacity=0.7),
                                 name=name_a), row=1, col=1)
        fig.add_trace(go.Scatter(x=times_b, y=solve_b, mode='markers',
                                 marker=dict(size=4, color='#2ca02c', opacity=0.7),
                                 name=name_b), row=1, col=2)
        fig.add_hline(y=budget, line_dash='dash', line_color='red', opacity=0.6)
        fig.update_yaxes(title_text='ms', row=1, col=1)
        fig.update_yaxes(title_text='ms', row=1, col=2)
        fig.update_layout(title=f'Solver Performance — {name_a} vs {name_b}', height=400)
        figures['solver'] = fig

    # Apply dark theme
    for fig in figures.values():
        fig.update_layout(
            template='plotly_dark',
            paper_bgcolor='#1a1a2e',
            plot_bgcolor='#16213e',
            font=dict(size=11),
            margin=dict(l=50, r=20, t=40, b=40),
            legend=dict(orientation='h', yanchor='bottom', y=1.02,
                        xanchor='right', x=1),
        )

    # Build comparison HTML (simpler than full report — just plots)
    plotly_js = '<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>'
    plot_divs = []
    for cat, fig in figures.items():
        div_html = fig.to_html(full_html=False, include_plotlyjs=False,
                               config={'scrollZoom': True, 'displayModeBar': True})
        plot_divs.append(f'<div class="plot-container">{div_html}</div>')

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Comparison: {name_a} vs {name_b}</title>
{plotly_js}
{_CSS}
</head>
<body>
<h1>Session Comparison</h1>
<p class="meta">
  <span style="color:{_ACTUAL_COLOR}">&#9632;</span> {name_a}
  &nbsp;&mdash;&nbsp;
  <span style="color:#2ca02c">&#9632;</span> {name_b}
</p>
{''.join(plot_divs)}
</body>
</html>"""

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(html)
    return os.path.abspath(output_path)


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    import argparse
    import json

    parser = argparse.ArgumentParser(description='Generate interactive diagnostic report')
    parser.add_argument('csv_path', help='Path to MPC telemetry CSV')
    parser.add_argument('--compare', default=None, metavar='CSV_B',
                        help='Compare against a second CSV (overlay both sessions)')
    parser.add_argument('--categories', default='auto',
                        help='Plot categories: auto, all, none, or comma-separated')
    parser.add_argument('-o', '--output', default=None,
                        help='Output HTML path (default: <csv>_interactive.html)')
    args = parser.parse_args()

    from analysis.diagnose import run_diagnosis
    from analysis.compare import load_csv as load_records

    # Parse categories
    cats = parse_categories(args.categories)

    if args.compare:
        # Comparison mode: overlay two sessions
        records_a = load_records(args.csv_path)
        result_a = run_diagnosis(args.csv_path, plots=None)
        records_b = load_records(args.compare)
        result_b = run_diagnosis(args.compare, plots=None)

        out = args.output
        if out is None:
            prefix = args.csv_path[:-4] if args.csv_path.lower().endswith('.csv') else args.csv_path
            out = f'{prefix}_vs_comparison.html'

        path = generate_comparison_report(
            records_a, result_a, records_b, result_b, out, categories=cats)
        print(f'Comparison report: {path}')
    else:
        # Single session report
        records = load_records(args.csv_path)
        result = run_diagnosis(args.csv_path, plots=None)

        out = args.output
        if out is None:
            prefix = args.csv_path[:-4] if args.csv_path.lower().endswith('.csv') else args.csv_path
            out = f'{prefix}_interactive.html'

        path = generate_interactive_report(records, result, out, categories=cats)
        print(f'Interactive report: {path}')
