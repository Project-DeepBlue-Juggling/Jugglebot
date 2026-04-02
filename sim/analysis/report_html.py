# -*- coding: utf-8 -*-
"""Self-contained HTML report generation for /diagnose.

Generates a single HTML file with base64-embedded diagnostic plots and
structured analysis tables.  No external dependencies beyond stdlib.

Usage (typically called from diagnose.py):
    from analysis.report_html import generate_html_report
    path = generate_html_report(result, "sim/logs/mpc_20260401_152048_report.html")
"""

from __future__ import annotations

import base64
import os
from datetime import datetime
from typing import Any, Dict, List, Optional


def _embed_plot(path: str) -> Optional[str]:
    """Read a PNG file and return a base64 data URI, or None if missing."""
    if not path or not os.path.isfile(path):
        return None
    with open(path, 'rb') as f:
        data = base64.b64encode(f.read()).decode('ascii')
    return f'data:image/png;base64,{data}'


def _severity_class(severity: str) -> str:
    s = severity.lower()
    if s == 'error':
        return 'badge-error'
    if s == 'warning':
        return 'badge-warn'
    return 'badge-info'


def _verdict_class(flags: List[Dict]) -> str:
    severities = {f.get('severity', '').lower() for f in flags}
    if 'error' in severities:
        return 'verdict-fail'
    if 'warning' in severities:
        return 'verdict-warn'
    return 'verdict-pass'


def _verdict_text(flags: List[Dict]) -> str:
    severities = {f.get('severity', '').lower() for f in flags}
    if 'error' in severities:
        return 'FAIL'
    if 'warning' in severities:
        return 'NEEDS ATTENTION'
    return 'PASS'


def _fmt(val: Any, decimals: int = 3) -> str:
    if val is None:
        return 'N/A'
    if isinstance(val, float):
        return f'{val:.{decimals}f}'
    return str(val)


def _plot_img(plots: Dict[str, str], category: str) -> str:
    """Return an <img> tag for the given plot category, or empty string."""
    path = plots.get(category)
    if not path:
        return ''
    uri = _embed_plot(path)
    if not uri:
        return f'<p class="plot-missing">Plot not available: {category}</p>'
    return f'<img src="{uri}" alt="{category} plot" class="plot">'


# ---------------------------------------------------------------------------
# CSS
# ---------------------------------------------------------------------------

_CSS = """\
* { box-sizing: border-box; margin: 0; padding: 0; }
body {
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    line-height: 1.5; color: #1a1a2e; background: #f5f5fa; padding: 24px;
    max-width: 1100px; margin: 0 auto;
}
h1 { font-size: 1.4rem; margin-bottom: 4px; }
h2 { font-size: 1.15rem; margin: 28px 0 12px; border-bottom: 2px solid #ddd; padding-bottom: 4px; }
h3 { font-size: 1rem; margin: 16px 0 8px; }
.meta { color: #555; font-size: 0.9rem; margin-bottom: 16px; }
.meta span { margin-right: 18px; }

.verdict { display: inline-block; padding: 4px 16px; border-radius: 4px;
           font-weight: 700; font-size: 1.1rem; margin: 8px 0 16px; }
.verdict-pass { background: #d4edda; color: #155724; }
.verdict-warn { background: #fff3cd; color: #856404; }
.verdict-fail { background: #f8d7da; color: #721c24; }

table { border-collapse: collapse; width: 100%; margin: 8px 0 16px; font-size: 0.9rem; }
th, td { border: 1px solid #ccc; padding: 5px 10px; text-align: right; }
th { background: #e9ecef; text-align: center; }
td:first-child, th:first-child { text-align: left; }
tr.aggregate { font-weight: 700; background: #f0f0f5; }

.badge { display: inline-block; padding: 1px 8px; border-radius: 3px;
         font-size: 0.8rem; font-weight: 600; margin-right: 6px; }
.badge-error { background: #f8d7da; color: #721c24; }
.badge-warn  { background: #fff3cd; color: #856404; }
.badge-info  { background: #d1ecf1; color: #0c5460; }

.flag-item { margin: 6px 0; padding: 6px 10px; background: #fff; border-radius: 4px;
             border-left: 3px solid #ccc; }
.flag-item.error { border-left-color: #dc3545; }
.flag-item.warning { border-left-color: #ffc107; }
.flag-item.info { border-left-color: #17a2b8; }

.plot { max-width: 100%; height: auto; margin: 12px 0; border: 1px solid #ddd; border-radius: 4px; }
.plot-missing { color: #888; font-style: italic; margin: 8px 0; }

.stat-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(180px, 1fr));
             gap: 8px; margin: 8px 0 16px; }
.stat-box { background: #fff; border: 1px solid #ddd; border-radius: 4px; padding: 8px 12px; }
.stat-box .label { font-size: 0.8rem; color: #666; }
.stat-box .value { font-size: 1.1rem; font-weight: 600; }

footer { margin-top: 32px; padding-top: 12px; border-top: 1px solid #ddd;
         font-size: 0.8rem; color: #888; }

/* Dark mode — follows OS/browser preference */
@media (prefers-color-scheme: dark) {
  body { background: #1a1a2e; color: #e0e0e8; }
  h2 { border-bottom-color: #444; }
  .meta { color: #aaa; }

  .verdict-pass { background: #1b4332; color: #95d5b2; }
  .verdict-warn { background: #4a3f00; color: #ffd166; }
  .verdict-fail { background: #4a1a1a; color: #f5a3a3; }

  table { border-color: #444; }
  th, td { border-color: #444; }
  th { background: #2a2a40; }
  tr.aggregate { background: #252540; }

  .badge-error { background: #4a1a1a; color: #f5a3a3; }
  .badge-warn  { background: #4a3f00; color: #ffd166; }
  .badge-info  { background: #0a3040; color: #7ec8e3; }

  .flag-item { background: #252540; }
  .stat-box { background: #252540; border-color: #444; }
  .stat-box .label { color: #999; }
  .plot { border-color: #444; }
  .plot-missing { color: #666; }
  footer { border-top-color: #444; color: #666; }
}
"""


# ---------------------------------------------------------------------------
# HTML generation
# ---------------------------------------------------------------------------

def generate_html_report(result: Dict[str, Any], output_path: str) -> str:
    """Generate a self-contained HTML report from a diagnosis result dict.

    Returns the absolute path of the written HTML file.
    """
    plots = result.get('plots', {})
    flags = result.get('flags', [])
    tracking = result.get('tracking', {})
    solve = result.get('solve_times', {})
    osc = result.get('oscillation', {})
    disc = result.get('discontinuities', {})
    ws = result.get('workspace', {})
    ss = result.get('steady_state', {})
    torques = result.get('torques', {})

    verdict = _verdict_text(flags)
    verdict_cls = _verdict_class(flags)

    parts: List[str] = []

    # --- Header ---
    parts.append(f"""\
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Diagnosis: {result.get('file', 'unknown')}</title>
<style>{_CSS}</style>
</head>
<body>
<h1>Hardware Diagnosis: {result.get('file', 'unknown')}</h1>
<div class="meta">
  <span>Source: {result.get('source', 'N/A')}</span>
  <span>Duration: {_fmt(result.get('duration_s'), 1)}s</span>
  <span>Samples: {result.get('n_samples', 'N/A')}</span>
  <span>dt: {_fmt(result.get('dt_median_ms'), 1)}ms</span>
</div>
<div class="verdict {verdict_cls}">{verdict}</div>
""")

    # --- Tracking Performance ---
    if tracking:
        per_leg = tracking.get('per_leg', [])
        parts.append('<h2>Tracking Performance</h2>')
        parts.append('<table><thead><tr>')
        parts.append('<th>Leg</th><th>RMS (mm)</th><th>Peak (mm)</th><th>Mean (mm)</th>')
        parts.append('</tr></thead><tbody>')
        for leg in per_leg:
            parts.append(
                f'<tr><td>Leg {leg["leg"]}</td>'
                f'<td>{_fmt(leg["rms_mm"], 4)}</td>'
                f'<td>{_fmt(leg["peak_mm"], 4)}</td>'
                f'<td>{_fmt(leg["mean_mm"], 4)}</td></tr>'
            )
        parts.append(
            f'<tr class="aggregate"><td>Aggregate</td>'
            f'<td>pos RMS: {_fmt(tracking.get("position_rms_mm"), 4)}</td>'
            f'<td>pos peak: {_fmt(tracking.get("position_peak_mm"), 4)}</td>'
            f'<td>ori RMS: {_fmt(tracking.get("orientation_rms_deg"), 4)}deg</td></tr>'
        )
        parts.append('</tbody></table>')

        worst = tracking.get('worst_leg')
        ratio = tracking.get('worst_leg_ratio', 0)
        if worst is not None and ratio > 1.5:
            parts.append(
                f'<p><strong>Worst leg:</strong> Leg {worst} '
                f'({_fmt(ratio, 1)}x median)</p>'
            )

        parts.append(_plot_img(plots, 'legs'))
        parts.append(_plot_img(plots, 'tracking'))

    # --- MPC Solver ---
    if solve and solve.get('n_valid', 0) > 0:
        parts.append('<h2>MPC Solver</h2>')
        parts.append('<div class="stat-grid">')
        for label, key, dec in [
            ('p50', 'p50_ms', 1), ('p95', 'p95_ms', 1),
            ('p99', 'p99_ms', 1), ('Max', 'max_ms', 1),
            ('Budget', 'budget_ms', 0),
            ('Violations', 'budget_violations', 0),
            ('Violation %', 'budget_violation_pct', 1),
            ('First sample', 'first_sample_ms', 1),
        ]:
            val = solve.get(key)
            suffix = 'ms' if 'ms' in key else ''
            suffix = '%' if 'pct' in key else suffix
            parts.append(
                f'<div class="stat-box">'
                f'<div class="label">{label}</div>'
                f'<div class="value">{_fmt(val, dec)}{suffix}</div>'
                f'</div>'
            )
        parts.append('</div>')

        consec = solve.get('max_consecutive_violations', 0)
        if consec >= 3:
            parts.append(
                f'<p><strong>Max consecutive violations:</strong> {consec}</p>'
            )

        parts.append(_plot_img(plots, 'solver'))

    # --- Stability ---
    parts.append('<h2>Stability</h2>')

    osc_detected = osc.get('detected', False)
    parts.append(f'<p><strong>Oscillation:</strong> '
                 f'{"DETECTED" if osc_detected else "not detected"}</p>')
    chatter = osc.get('per_leg_chatter', [])
    if chatter:
        parts.append(f'<p>Per-leg chatter ratios: '
                     f'{", ".join(_fmt(c, 3) for c in chatter)}</p>')

    cmd_jumps = disc.get('cmd_jumps', [])
    actual_jumps = disc.get('actual_jumps', [])
    parts.append(f'<p><strong>Discontinuities:</strong> '
                 f'{len(cmd_jumps)} command jumps, '
                 f'{len(actual_jumps)} actual jumps</p>')

    if torques.get('available'):
        parts.append(
            f'<p><strong>Torque:</strong> max {_fmt(torques.get("max_Nm"), 4)} Nm, '
            f'mean {_fmt(torques.get("mean_Nm"), 4)} Nm, '
            f'p95 {_fmt(torques.get("p95_Nm"), 4)} Nm</p>'
        )

    parts.append(_plot_img(plots, 'chatter'))
    parts.append(_plot_img(plots, 'velocity'))

    # --- Workspace ---
    if ws:
        parts.append('<h2>Workspace</h2>')
        parts.append(
            f'<p>Extension range: [{_fmt(ws.get("min_extension_mm"), 2)}, '
            f'{_fmt(ws.get("max_extension_mm"), 2)}] mm</p>'
        )
        parts.append(
            f'<p>Margin to lower limit: {_fmt(ws.get("margin_to_lower_mm"), 1)} mm '
            f'&nbsp;|&nbsp; Margin to upper limit: '
            f'{_fmt(ws.get("margin_to_upper_mm"), 1)} mm</p>'
        )
        parts.append(_plot_img(plots, 'workspace'))

    # --- Steady State ---
    if ss and ss.get('has_steady_state'):
        parts.append('<h2>Steady State</h2>')
        parts.append(
            f'<p>SS start: {_fmt(ss.get("ss_start_s"), 2)}s &nbsp;|&nbsp; '
            f'SS RMS: {_fmt(ss.get("ss_rms_mm"), 4)} mm &nbsp;|&nbsp; '
            f'SS peak: {_fmt(ss.get("ss_peak_mm"), 4)} mm</p>'
        )
        trans_peak = ss.get('transient_peak_mm')
        if trans_peak is not None:
            parts.append(
                f'<p>Transient peak: {_fmt(trans_peak, 4)} mm &nbsp;|&nbsp; '
                f'Settling time: {_fmt(ss.get("settling_time_s"), 2)}s</p>'
            )

    # --- Pose plot (if available) ---
    pose_img = _plot_img(plots, 'pose')
    if pose_img:
        parts.append('<h2>Platform Pose</h2>')
        parts.append(pose_img)

    # --- Flagged Issues ---
    parts.append('<h2>Flagged Issues</h2>')
    if not flags:
        parts.append('<p>No flags — clean run.</p>')
    else:
        parts.append(f'<p>{len(flags)} flag(s)</p>')
        for f in flags:
            sev = f.get('severity', 'info').lower()
            cls = _severity_class(sev)
            parts.append(
                f'<div class="flag-item {sev}">'
                f'<span class="badge {cls}">{sev.upper()}</span>'
                f'<strong>[{f.get("source", "?")}]</strong> '
                f'{f.get("message", "")}'
                f'</div>'
            )

    # --- Footer ---
    now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    parts.append(f"""\
<footer>
  Generated by <code>diagnose.py --html</code> at {now}
</footer>
</body>
</html>""")

    html = '\n'.join(parts)
    abs_path = os.path.abspath(output_path)
    with open(abs_path, 'w') as f:
        f.write(html)
    return abs_path
