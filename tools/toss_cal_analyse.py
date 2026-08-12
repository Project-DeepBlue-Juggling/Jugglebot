#!/usr/bin/env python3
"""Read a toss-record corpus and/or a toss aim map, and say what it means.

Build phase 2c of ``plans/active/toss-selftuning.md`` (§ 3.4, § 3.8). The
sibling of ``tools/tilt_cal_analyse.py``, deliberately: same location, same
``--json`` / ``--report-dir`` / ``--no-plot`` shape, same "print the table even
when matplotlib is missing" posture.

    # a corpus: heat map + quiver, per-node n/sd, anchor series, uptime scatter
    python tools/toss_cal_analyse.py --corpus temp/probes/toss_records_*.jsonl

    # two maps: the invariance check a re-capture owes (§ 3.8)
    python tools/toss_cal_analyse.py --map config/toss_calibration.yaml \\
        --diff temp/toss_calibration.prev.yaml

    # an A/B ladder, scored on the CONTINUOUS observables
    python tools/toss_cal_analyse.py --corpus c.jsonl --group A=1-30 --group B=31-60

WHAT IT REFUSES TO DO
---------------------
It never writes a calibration. ``toss_cal_fit.py`` owns the write and its gates;
this tool is the thing you read **before** deciding to run it, and after, to see
whether the map that landed did what it claimed. Keeping the two apart is what
lets the analyser be liberal (plot anything, score anything) while the fit stays
fail-closed.

THE FOUR PANELS, AND THE QUESTION EACH ANSWERS
-----------------------------------------------
* **Heat map + quiver of the aim residual** — *is there a spatial field at all?*
  The pre-registered fallback in § 6 says that if per-node residuals are not
  stable, ship the home-node offset as a single global bias and stop. This is the
  picture that decides it.
* **Per-node n / sd** — *does any node have enough tosses to believe?* N_MIN is 8
  and a node's se is ``sd/√n``; a node with a big residual and n = 3 is a rumour.
* **Anchor series** — *was it the same machine at the end as at the start?* The
  home node is re-visited through the capture; drift across the series is a plant
  change, and a map fitted across one is a weighted average of two machines.
* **Residual vs bridge uptime** — *is the corpus contaminated by the uptime
  effect?* The dispatch shift measured +118–133 ms at ~16 h against a 40 ms
  suppression margin. D16 refuses to invent a ceiling and instead makes the
  corpus police itself; this scatter is the instrument.

All of it is also emitted as ``--json`` so a runner can gate on it.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys
from datetime import datetime
from typing import Any, Dict, List, Optional, Sequence, Tuple

_HERE = os.path.dirname(os.path.abspath(__file__))              # tools/
_REPO = os.path.dirname(_HERE)
for _p in (os.path.join(_REPO, 'tests', 'hardware'), _REPO,
           os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(_REPO, 'config', 'generated')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np                                              # noqa: E402
import yaml                                                     # noqa: E402

import toss_fit_lib as fit                                      # noqa: E402
from jugglebot.motion import toss_cal                           # noqa: E402

RAD2DEG = 180.0 / math.pi

#: Default report directory. ``temp/`` is the repo-wide runtime-artifact root and
#: is gitignored, so a report never lands in a commit by accident.
DEFAULT_REPORT_DIR = os.path.join(_REPO, 'temp', 'reports', 'toss_cal')

#: § 3.8's map-vs-map invariance gate: ``max node delta ≤ max(2·se, 0.05°)``.
#: The floor exists because ``2·se`` collapses toward zero on a big capture and
#: would then flag genuine re-fit noise; 0.05° = 2.7 mm at h = 0.78 m, a third of
#: the flat-field floor.
DIFF_FLOOR_RAD = math.radians(0.05)

#: Anchor-series report/abort thresholds, transposed from ``tilt_cal_grid``'s
#: anchor gate. Reported ALWAYS; the numbers are in aim-equivalent radians here,
#: not inclinometer radians, so they are quoted alongside their mm equivalent.
ANCHOR_PP_WARN_RAD = 0.002

#: Minimum bridge-uptime span, hours, before a residual-vs-uptime SLOPE is
#: reported at all. 0.25 h = 15 min, which at the 6.0 s cadence is ~150 tosses —
#: an entire first capture. Below it the fit is an extrapolation to "per hour"
#: from a span that never saw an hour, and it prints a large number that is
#: entirely noise. The scatter itself is always plotted; only the slope is
#: withheld.
MIN_UPTIME_SPAN_H = 0.25


# ── loading ──────────────────────────────────────────────────────────────────


def expand(patterns: Sequence[str]) -> List[str]:
    paths: List[str] = []
    for pattern in patterns:
        hits = sorted(glob.glob(pattern))
        if not hits and os.path.exists(pattern):
            hits = [pattern]
        if not hits:
            raise SystemExit('ERROR: no file matches {!r}'.format(pattern))
        paths.extend(hits)
    seen, out = set(), []
    for path in paths:
        if path not in seen:
            seen.add(path)
            out.append(path)
    return out


def load_map(path: str) -> Tuple[Dict[str, Any], toss_cal.TossCal]:
    with open(path, 'r') as handle:
        doc = yaml.safe_load(handle)
    return doc, toss_cal.parse_toss_cal(doc)


def axes_from(doc: Optional[Dict[str, Any]], args) -> Tuple[List[float], List[float]]:
    """Grid axes: the map's own if a map was given, else the CLI's.

    Taking them from the map when one exists is not a convenience — it is what
    makes a corpus analysis and the map it is being compared against index the
    SAME physical nodes. Reconstructing a grid from the CLI while diffing a map
    is how ``[iy][ix]`` comes to mean two different poses in one report.
    """
    if doc:
        grid = doc.get('grid') or {}
        return ([float(v) for v in grid.get('x_mm', [])],
                [float(v) for v in grid.get('y_mm', [])])
    step = 2.0 * args.box_mm / (args.nodes - 1)
    axis = [round(-args.box_mm + i * step, 6) for i in range(args.nodes)]
    return list(axis), list(axis)


# ── corpus analysis ──────────────────────────────────────────────────────────


def analyse_corpus(records, x_mm, y_mm, *, n_min: int) -> Dict[str, Any]:
    """Everything the four panels need, as plain data."""
    census = fit.partition_census(records)
    fits, admitted, excluded = fit.fit_nodes(records, x_mm, y_mm, n_min=n_min)
    try:
        anchors = fit.anchor_visits(records, x_mm, y_mm)
        anchor = fit.anchor_estimate(anchors)
    except fit.TossFitError:
        anchors, anchor = [], ((0.0, 0.0), (float('nan'), float('nan')), 0)

    n_y, n_x = len(y_mm), len(x_mm)
    rx = np.full((n_y, n_x), np.nan)
    ry = np.full((n_y, n_x), np.nan)
    n_grid = np.zeros((n_y, n_x), dtype=int)
    sd_grid = np.full((n_y, n_x), np.nan)
    for (ix, iy), node in fits.items():
        rx[iy][ix] = node.rx - anchor[0][0]
        ry[iy][ix] = node.ry - anchor[0][1]
        n_grid[iy][ix] = node.n
        if math.isfinite(node.sd_rx) and math.isfinite(node.sd_ry):
            sd_grid[iy][ix] = math.hypot(node.sd_rx, node.sd_ry)

    # Residual vs bridge uptime — D16's within-session trend test, as data.
    scatter: List[Tuple[float, float]] = []
    for rec in records:
        ok, _reason = fit.admit_for_aim(rec)
        if not ok:
            continue
        up = rec.get('uptime_ms_at_release')
        err = rec.get('land_err_mm')
        if up is None or not err or err[0] is None:
            continue
        scatter.append((float(up) / 3.6e6,
                        float(math.hypot(float(err[0]), float(err[1])))))
    trend = None
    span_h = 0.0
    if len(scatter) >= 3:
        u = np.asarray([p[0] for p in scatter])
        e = np.asarray([p[1] for p in scatter])
        span_h = float(np.ptp(u))
        # A slope fitted over a few minutes and REPORTED PER HOUR is an
        # extrapolation of 10x or more, and it reads as a large effect when it is
        # only noise. The datum this panel exists for (+118-133 ms of dispatch
        # shift at ~16 h) is an hours-scale phenomenon, so a corpus that does not
        # span a meaningful fraction of an hour simply cannot see it and must say
        # so rather than print a number.
        if span_h >= MIN_UPTIME_SPAN_H:
            trend = float(np.polyfit(u, e, 1)[0])

    labels: Dict[str, int] = {}
    for rec in records:
        key = str(rec.get('label'))
        labels[key] = labels.get(key, 0) + 1

    return {
        'n_records': len(records),
        'n_admitted': sum(len(v) for v in admitted.values()),
        'excluded': dict(sorted(excluded.items())),
        'labels': labels,
        'partitions': [
            {'n': n, **{name: (None if v is None else str(v))
                        for name, v in zip(fit.PARTITION_KEYS, k)}}
            for k, n in sorted(census.items(), key=lambda kv: -kv[1])],
        'x_mm': list(x_mm), 'y_mm': list(y_mm),
        'rx': rx, 'ry': ry, 'n_grid': n_grid, 'sd_grid': sd_grid,
        'anchor_rad': list(anchor[0]),
        'anchor_se_rad': [None if not math.isfinite(v) else v
                          for v in anchor[1]],
        'anchor_n': anchor[2],
        'anchors': [{'t_s': v.t_s, 'rx': v.rx, 'ry': v.ry, 'n': v.n}
                    for v in anchors],
        'anchor_pp_rad': (
            max(max(v.rx for v in anchors) - min(v.rx for v in anchors),
                max(v.ry for v in anchors) - min(v.ry for v in anchors))
            if len(anchors) >= 2 else None),
        'uptime_scatter': scatter,
        'uptime_span_h': span_h,
        'uptime_trend_mm_per_h': trend,
        'sigma_L_mm': fit.sigma_land_mm(records),
        'R_eff_mm': fit.r_eff_mm(records),
        'speed': {k: v for k, v in fit.speed_fit(records).items()
                  if k != 'excluded'},
        'timing': {k: v for k, v in fit.timing_fit(records).items()
                   if k != 'excluded'},
    }


def map_field(cal: toss_cal.TossCal) -> Dict[str, Any]:
    return {'x_mm': [float(v) for v in cal.x_mm],
            'y_mm': [float(v) for v in cal.y_mm],
            'rx': np.asarray(cal.rx_rad, dtype=float),
            'ry': np.asarray(cal.ry_rad, dtype=float),
            'n_grid': np.zeros(cal.shape, dtype=int),
            'sd_grid': np.full(cal.shape, np.nan),
            'anchor_rad': list(cal.anchor_aim_rad or (0.0, 0.0)),
            'version': cal.version}


def diff_maps(a: Dict[str, Any], b: Dict[str, Any]) -> Dict[str, Any]:
    """§ 3.8's invariance check between two maps of the SAME grid.

    Different grids are refused rather than interpolated: an interpolated diff
    reports a number for a node that was never captured in one of the two maps,
    and the whole point of a re-capture diff is to compare *measurements*.
    """
    if (len(a['x_mm']) != len(b['x_mm']) or len(a['y_mm']) != len(b['y_mm'])
            or not np.allclose(a['x_mm'], b['x_mm'], atol=1e-6)
            or not np.allclose(a['y_mm'], b['y_mm'], atol=1e-6)):
        return {'same_axes': False}
    d_rx = a['rx'] - b['rx']
    d_ry = a['ry'] - b['ry']
    mag = np.hypot(d_rx, d_ry)
    iy, ix = np.unravel_index(int(np.nanargmax(mag)), mag.shape)
    return {'same_axes': True,
            'delta_rad': mag,
            'max_delta_rad': float(np.nanmax(mag)),
            'mean_delta_rad': float(np.nanmean(mag)),
            'worst_node': {'ix': int(ix), 'iy': int(iy),
                           'x_mm': float(a['x_mm'][ix]),
                           'y_mm': float(a['y_mm'][iy])}}


# ── printing ─────────────────────────────────────────────────────────────────


def print_report(data: Dict[str, Any], gain_mm_per_rad: float) -> None:
    print('\n=== CORPUS')
    print('  records {}   admitted for the aim fit {}'.format(
        data['n_records'], data['n_admitted']))
    print('  labels: {}'.format(', '.join(
        '{}={}'.format(k, v) for k, v in sorted(data['labels'].items()))))
    if data['excluded']:
        print('  exclusions:')
        for reason, n in sorted(data['excluded'].items(), key=lambda kv: -kv[1]):
            print('    {:<32} {}'.format(reason, n))
    print('  partitions ({}):'.format(len(data['partitions'])))
    for row in data['partitions']:
        print('    n={:<4d} {}'.format(row['n'], '  '.join(
            '{}={}'.format(k, v if v is not None else '?')
            for k, v in row.items() if k != 'n')))

    print('\n=== PER-NODE aim residual (shipped = home-referenced)')
    print('  {:>6} {:>6} {:>9} {:>9} {:>9} {:>5} {:>9}'.format(
        'x', 'y', 'rx[deg]', 'ry[deg]', '|L|[mm]', 'n', 'sd[deg]'))
    for iy in range(len(data['y_mm']) - 1, -1, -1):
        for ix in range(len(data['x_mm'])):
            rx = float(data['rx'][iy][ix])
            ry = float(data['ry'][iy][ix])
            print('  {:>6.0f} {:>6.0f} {:>9.4f} {:>9.4f} {:>9.1f} {:>5d} {:>9}'
                  .format(data['x_mm'][ix], data['y_mm'][iy],
                          rx * RAD2DEG, ry * RAD2DEG,
                          math.hypot(rx, ry) * gain_mm_per_rad,
                          int(data['n_grid'][iy][ix]),
                          '-' if not math.isfinite(data['sd_grid'][iy][ix])
                          else '{:.4f}'.format(
                              float(data['sd_grid'][iy][ix]) * RAD2DEG)))

    print('\n=== ANCHOR SERIES ({} visits, n={})'.format(
        len(data['anchors']), data['anchor_n']))
    for visit in data['anchors']:
        print('  t={:>10.1f}  n={:<3d} aim ({:+.4f}, {:+.4f})°'.format(
            visit['t_s'], visit['n'], visit['rx'] * RAD2DEG,
            visit['ry'] * RAD2DEG))
    pp = data['anchor_pp_rad']
    if pp is None:
        print('  (a single anchor visit cannot estimate its own repeatability)')
    else:
        verdict = 'WARN' if pp > ANCHOR_PP_WARN_RAD else 'OK'
        print('  peak-to-peak {:.4f}° = {:.1f} mm  [{}]'.format(
            pp * RAD2DEG, pp * gain_mm_per_rad, verdict))

    print('\n=== POPULATION')
    print('  sigma_L      {}'.format(
        '-' if data['sigma_L_mm'] is None
        else '{:.1f} mm'.format(data['sigma_L_mm'])))
    print('  R_eff        {}   (geometric radius {:.1f} mm)'.format(
        '-' if data['R_eff_mm'] is None else '{:.1f} mm'.format(data['R_eff_mm']),
        fit.hw.GEOM_HAND_RADIUS_MM))
    if data['uptime_trend_mm_per_h'] is None:
        print('  uptime trend -   (span {:.2f} h over {} admitted tosses; a '
              'slope needs >= {:.2f} h)'.format(
                  data.get('uptime_span_h', 0.0), len(data['uptime_scatter']),
                  MIN_UPTIME_SPAN_H))
    else:
        print('  uptime trend {:+.2f} mm/h of |land_err| over {} admitted '
              'tosses spanning {:.2f} h'.format(
                  data['uptime_trend_mm_per_h'], len(data['uptime_scatter']),
                  data['uptime_span_h']))
    print('  speed  n={} k_v={} applies={}'.format(
        data['speed']['n'],
        '-' if data['speed']['k_v'] is None
        else '{:.4f}'.format(data['speed']['k_v']), data['speed']['applies']))
    print('  timing n={} tau={} se={} applies={}'.format(
        data['timing']['n'],
        '-' if data['timing']['tau_ms'] is None
        else '{:.1f} ms'.format(data['timing']['tau_ms']),
        '-' if data['timing']['se_ms'] is None
        else '{:.1f} ms'.format(data['timing']['se_ms']),
        data['timing']['applies']))


def print_groups(rows: Sequence[Dict[str, Any]]) -> None:
    print('\n=== A/B  (scored on the CONTINUOUS observables — the label needs '
          '196 tosses per arm to move, F4)')
    print('  {:<12} {:>6} {:>9} {:>20} {:>10} {:>10} {:>10}'.format(
        'group', 'n', 'admitted', 'mean err [mm]', 'sigma[mm]', 'R_eff[mm]',
        'catch'))
    for row in rows:
        print('  {:<12} {:>6d} {:>9d} {:>20} {:>10} {:>10} {:>10}'.format(
            row['name'], row['n_records'], row['n_admitted'],
            '-' if row['mean_err_mm'] is None
            else '({:+.1f}, {:+.1f})'.format(*row['mean_err_mm']),
            '-' if row['sigma_mm'] is None else '{:.1f}'.format(row['sigma_mm']),
            '-' if row['r_eff_mm'] is None else '{:.1f}'.format(row['r_eff_mm']),
            '-' if row['catch_rate'] is None
            else '{:.0%}/{}'.format(row['catch_rate'], row['n_labelled'])))


# ── plots ────────────────────────────────────────────────────────────────────


def cell_edges(nodes: Sequence[float]) -> List[float]:
    """Cell BOUNDARIES for an axis of node coordinates — ``len(nodes) + 1``.

    Verbatim from ``tools/tilt_cal_analyse.py``, including its reason: what
    ``pcolormesh`` needs and what ``imshow(extent=...)`` cannot express. With
    ``imshow`` each pixel centre lands half a cell off its node, so on the ±150 mm
    grid the colour cell for x = −150 is centred at x = −120 while the text label
    sits at −150 — and reading an outlier off such a plot points the operator at
    the wrong node.
    """
    values = [float(v) for v in nodes]
    n = len(values)
    if n == 0:
        return []
    if n == 1:
        return [values[0] - 0.5, values[0] + 0.5]
    mids = [0.5 * (values[i] + values[i + 1]) for i in range(n - 1)]
    return ([values[0] - (mids[0] - values[0])] + mids
            + [values[-1] + (values[-1] - mids[-1])])


def make_plots(data: Dict[str, Any], report_dir: str, gain: float,
               diff: Optional[Dict[str, Any]] = None) -> List[str]:
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception as exc:                                    # noqa: BLE001
        print('  (plots skipped: matplotlib unavailable — {})'.format(exc))
        return []

    os.makedirs(report_dir, exist_ok=True)
    written: List[str] = []
    x_edges = np.asarray(cell_edges(data['x_mm']), dtype=float)
    y_edges = np.asarray(cell_edges(data['y_mm']), dtype=float)

    def heat(values, title, fname, label='deg', symmetric=False, fmt='{:.3f}'):
        fig, ax = plt.subplots(figsize=(7, 6))
        array = np.asarray(values, dtype=float)
        if symmetric:
            peak = float(np.nanmax(np.abs(array))) or 1.0
            kwargs = {'vmin': -peak, 'vmax': peak, 'cmap': 'coolwarm'}
        else:
            kwargs = {'cmap': 'viridis'}
        image = ax.pcolormesh(x_edges, y_edges, array, shading='flat', **kwargs)
        ax.set_aspect('equal')
        for iy in range(array.shape[0]):
            for ix in range(array.shape[1]):
                value = float(array[iy][ix])
                if math.isfinite(value):
                    ax.text(float(data['x_mm'][ix]), float(data['y_mm'][iy]),
                            fmt.format(value), ha='center', va='center',
                            fontsize=7, color='w')
        fig.colorbar(image, ax=ax, label=label)
        ax.set_xlabel('x [mm]')
        ax.set_ylabel('y [mm]')
        ax.set_title(title)
        fig.tight_layout()
        path = os.path.join(report_dir, fname)
        fig.savefig(path, dpi=110)
        plt.close(fig)
        written.append(path)

    mag_mm = np.hypot(data['rx'], data['ry']) * gain
    heat(mag_mm, 'commanded aim magnitude [mm of landing shift]',
         'aim_magnitude.png', label='mm', fmt='{:.1f}')
    heat(data['rx'] * RAD2DEG, 'commanded aim rx [deg]', 'aim_rx.png',
         symmetric=True)
    heat(data['ry'] * RAD2DEG, 'commanded aim ry [deg]', 'aim_ry.png',
         symmetric=True)
    if data['n_grid'].any():
        heat(data['n_grid'].astype(float), 'admitted tosses per node',
             'node_n.png', label='n', fmt='{:.0f}')

    # Quiver — in LANDING space, which is the only space the operator can check
    # against a ball on the floor. The arrow is the landing shift this node's
    # aim commands: J @ aim, with J the production Jacobian (a 90 deg rotation),
    # so Lx comes from ry and Ly from -rx. Plotting (rx, ry) directly would draw
    # every arrow at right angles to the correction it represents.
    fig, ax = plt.subplots(figsize=(7, 6))
    gx, gy = np.meshgrid(data['x_mm'], data['y_mm'])
    Lx = data['ry'] * gain
    Ly = -data['rx'] * gain
    ax.quiver(gx, gy, Lx, Ly, np.hypot(Lx, Ly), cmap='viridis', angles='xy')
    ax.set_xlabel('x [mm]')
    ax.set_ylabel('y [mm]')
    ax.set_title('commanded landing shift (arrow = J·aim, colour = |shift| mm)')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    fig.tight_layout()
    path = os.path.join(report_dir, 'aim_quiver.png')
    fig.savefig(path, dpi=110)
    plt.close(fig)
    written.append(path)

    if data['anchors']:
        fig, ax = plt.subplots(figsize=(7, 4))
        t = [v['t_s'] - data['anchors'][0]['t_s'] for v in data['anchors']]
        ax.plot(t, [v['rx'] * RAD2DEG for v in data['anchors']], 'o-', label='rx')
        ax.plot(t, [v['ry'] * RAD2DEG for v in data['anchors']], 's-', label='ry')
        ax.set_xlabel('time since first anchor [s]')
        ax.set_ylabel('anchor aim [deg]')
        ax.set_title('anchor series — drift here is a PLANT change, not noise')
        ax.grid(True, alpha=0.3)
        ax.legend()
        fig.tight_layout()
        path = os.path.join(report_dir, 'anchor_series.png')
        fig.savefig(path, dpi=110)
        plt.close(fig)
        written.append(path)

    if data['uptime_scatter']:
        fig, ax = plt.subplots(figsize=(7, 4))
        u = [p[0] for p in data['uptime_scatter']]
        e = [p[1] for p in data['uptime_scatter']]
        ax.scatter(u, e, s=18, alpha=0.75)
        if data['uptime_trend_mm_per_h'] is not None:
            xs = np.linspace(min(u), max(u), 2)
            slope = data['uptime_trend_mm_per_h']
            intercept = float(np.mean(e)) - slope * float(np.mean(u))
            ax.plot(xs, slope * xs + intercept, 'r--',
                    label='{:+.2f} mm/h'.format(slope))
            ax.legend()
        ax.set_xlabel('can-bridge uptime at release [h]')
        ax.set_ylabel('|land_err| [mm]')
        ax.set_title('residual vs bridge uptime (D16 — the corpus policing '
                     'itself)')
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        path = os.path.join(report_dir, 'uptime_scatter.png')
        fig.savefig(path, dpi=110)
        plt.close(fig)
        written.append(path)

    if diff and diff.get('same_axes'):
        heat(np.asarray(diff['delta_rad'], dtype=float) * RAD2DEG,
             'per-node delta vs the other map [deg]', 'diff_magnitude.png')
    return written


def write_html(data: Dict[str, Any], images: Sequence[str], report_dir: str,
               gain: float, title: str,
               diff: Optional[Dict[str, Any]] = None,
               groups: Sequence[Dict[str, Any]] = ()) -> str:
    os.makedirs(report_dir, exist_ok=True)
    head = ''.join('<th>{:.0f}</th>'.format(float(v)) for v in data['x_mm'])
    rows = []
    for iy in range(len(data['y_mm']) - 1, -1, -1):
        cells = ''.join(
            '<td>{:.1f}<br><small>n={}</small></td>'.format(
                math.hypot(float(data['rx'][iy][ix]),
                           float(data['ry'][iy][ix])) * gain,
                int(data['n_grid'][iy][ix]))
            for ix in range(len(data['x_mm'])))
        rows.append('<tr><th>{:.0f}</th>{}</tr>'.format(
            float(data['y_mm'][iy]), cells))
    img_html = ''.join(
        '<figure><img src="{0}" style="max-width:100%">'
        '<figcaption>{0}</figcaption></figure>'.format(os.path.basename(p))
        for p in images)
    group_html = ''
    if groups:
        group_html = (
            '<h2>A/B</h2><table><tr><th>group</th><th>n</th><th>admitted</th>'
            '<th>mean err [mm]</th><th>sigma [mm]</th><th>R_eff [mm]</th>'
            '<th>catch</th></tr>' + ''.join(
                '<tr><th>{}</th><td>{}</td><td>{}</td><td>{}</td><td>{}</td>'
                '<td>{}</td><td>{}</td></tr>'.format(
                    g['name'], g['n_records'], g['n_admitted'],
                    '-' if g['mean_err_mm'] is None
                    else '{:+.1f}, {:+.1f}'.format(*g['mean_err_mm']),
                    '-' if g['sigma_mm'] is None else '{:.1f}'.format(g['sigma_mm']),
                    '-' if g['r_eff_mm'] is None else '{:.1f}'.format(g['r_eff_mm']),
                    '-' if g['catch_rate'] is None
                    else '{:.0%}'.format(g['catch_rate']))
                for g in groups) + '</table>')
    diff_html = ''
    if diff:
        diff_html = ('<h2>Map vs map</h2><p>{}</p>'.format(
            'axes differ — no diff (a diff across grids would report a number '
            'for a node one map never captured)' if not diff.get('same_axes')
            else 'max node delta <b>{:.4f}°</b> ({:.1f} mm) at ({:+.0f}, '
                 '{:+.0f}); mean {:.4f}°. § 3.8 gate is &le; max(2&middot;se, '
                 '0.05&deg;) &mdash; this page shows the floor half; take '
                 '2&middot;se from each map&rsquo;s stats block.'
                 .format(diff['max_delta_rad'] * RAD2DEG,
                         diff['max_delta_rad'] * gain,
                         diff['worst_node']['x_mm'], diff['worst_node']['y_mm'],
                         diff['mean_delta_rad'] * RAD2DEG)))

    html = (
        '<!doctype html><meta charset="utf-8"><title>{title}</title>'
        '<style>body{{font-family:system-ui,sans-serif;margin:2rem;'
        'max-width:70rem}}table{{border-collapse:collapse}}'
        'td,th{{border:1px solid #ccc;padding:.25rem .5rem;text-align:right;'
        'font-variant-numeric:tabular-nums}}figure{{margin:1rem 0}}'
        'small{{color:#666}}</style>'
        '<h1>{title}</h1>'
        '<p><b>records</b> {n} &middot; <b>admitted</b> {adm} &middot; '
        '<b>sigma_L</b> {sigma} &middot; <b>R_eff</b> {reff} &middot; '
        '<b>anchor</b> ({arx:+.4f}, {ary:+.4f})&deg;</p>'
        '<h2>Commanded landing shift per node [mm]</h2>'
        '<table><tr><th>y \\ x</th>{head}</tr>{rows}</table>'
        '{groups}{diff}<h2>Plots</h2>{images}'
        '<p style="color:#666">Contract C-TOSS-CAL-1 &mdash; '
        'plans/active/toss-selftuning.md. The grid is HOME-REFERENCED; the '
        'absolute home residual is the session trim&rsquo;s warm-start prior, '
        'not a correction. Generated {now}.</p>'
    ).format(title=title, n=data['n_records'], adm=data['n_admitted'],
             sigma='-' if data['sigma_L_mm'] is None
                   else '{:.1f} mm'.format(data['sigma_L_mm']),
             reff='-' if data['R_eff_mm'] is None
                  else '{:.1f} mm'.format(data['R_eff_mm']),
             arx=data['anchor_rad'][0] * RAD2DEG,
             ary=data['anchor_rad'][1] * RAD2DEG,
             head=head, rows=''.join(rows), groups=group_html, diff=diff_html,
             images=img_html,
             now=datetime.now().isoformat(timespec='seconds'))
    path = os.path.join(report_dir, 'index.html')
    with open(path, 'w') as handle:
        handle.write(html)
    return path


def _jsonable(value):
    if isinstance(value, dict):
        return {k: _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return _jsonable(np.where(np.isfinite(value), value, None).tolist()
                         if value.dtype.kind == 'f' else value.tolist())
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


# ── main ─────────────────────────────────────────────────────────────────────


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--corpus', action='append', default=[],
                        help='toss_record JSONL file or glob (repeatable)')
    parser.add_argument('--map', default=None,
                        help='a toss_calibration.yaml to read INSTEAD of a '
                             'corpus (or alongside, to supply the grid)')
    parser.add_argument('--diff', default=None,
                        help='a second map, for the § 3.8 invariance check')
    parser.add_argument('--group', action='append', default=[],
                        help='A/B span NAME=lo-hi (1-based, inclusive)')
    parser.add_argument('--box-mm', type=float, default=150.0)
    parser.add_argument('--nodes', type=int, default=3)
    parser.add_argument('--n-min', type=int, default=fit.N_MIN)
    parser.add_argument('--z-mm', type=float, default=170.0)
    parser.add_argument('--apex-m', type=float, default=0.78,
                        help='height used for the rad -> mm report scale')
    parser.add_argument('--no-plot', action='store_true')
    parser.add_argument('--report-dir', default=None)
    parser.add_argument('--json', default=None)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if not args.corpus and not args.map:
        print('ERROR: give --corpus and/or --map')
        return 2

    T = math.sqrt(8.0 * float(args.apex_m) / 9.80665)
    gain = float(np.linalg.norm(
        fit.aim_landing_jacobian(T, float(args.z_mm))[:, 1]))

    doc = None
    cal = None
    if args.map:
        doc, cal = load_map(args.map)
        print('MAP  {}  version {}'.format(args.map, cal.version))
    x_mm, y_mm = axes_from(doc, args)

    groups: List[Dict[str, Any]] = []
    if args.corpus:
        paths = expand(args.corpus)
        print('CORPUS')
        for path in paths:
            print('  {}'.format(path))
        records = fit.load_corpus(paths)
        data = analyse_corpus(records, x_mm, y_mm, n_min=args.n_min)
        title = 'Toss aim calibration — corpus ({} records)'.format(len(records))
        if args.group:
            specs = []
            for spec in args.group:
                name, span = spec.split('=', 1)
                lo, hi = span.split('-', 1)
                specs.append((name, records[int(lo) - 1:int(hi)]))
            groups = fit.score_groups(records, specs)
            print_groups(groups)
    else:
        data = map_field(cal)
        data.update({'n_records': 0, 'n_admitted': 0, 'excluded': {},
                     'labels': {}, 'partitions': [], 'anchors': [],
                     'anchor_se_rad': [None, None], 'anchor_n': 0,
                     'anchor_pp_rad': None, 'uptime_scatter': [],
                     'uptime_span_h': 0.0,
                     'uptime_trend_mm_per_h': None, 'sigma_L_mm': None,
                     'R_eff_mm': None,
                     'speed': {'n': 0, 'k_v': None, 'se': None,
                               'applies': False},
                     'timing': {'n': 0, 'tau_ms': None, 'se_ms': None,
                                'sd_ms': None, 'applies': False,
                                'uptime_trend_ms_per_h': None}})
        title = 'Toss aim calibration — map {}'.format(cal.version)

    diff = None
    if args.diff:
        other_doc, other = load_map(args.diff)
        base = map_field(cal) if cal is not None else data
        diff = diff_maps(base, map_field(other))
        if diff.get('same_axes'):
            # The gate is max(2*se, DIFF_FLOOR_RAD) and the 2*se half needs the
            # per-node se of BOTH captures, which a map file does not carry. So
            # the floor is printed as what it is - the floor - and the operator
            # applies the se half from the stats block. Printing a single number
            # here would look like the whole gate and would pass a re-capture
            # that only cleared its weaker half.
            print('\n=== MAP vs MAP  max node delta {:.4f}° ({:.1f} mm) — '
                  '§ 3.8 gate is max(2·se, {:.2f}°); this is the FLOOR half, '
                  'take 2·se from stats.sd_r*_rad'.format(
                      diff['max_delta_rad'] * RAD2DEG,
                      diff['max_delta_rad'] * gain,
                      DIFF_FLOOR_RAD * RAD2DEG))
        else:
            print('\n=== MAP vs MAP  axes differ — no diff')

    print_report(data, gain)

    report_dir = args.report_dir or DEFAULT_REPORT_DIR
    images: List[str] = []
    if not args.no_plot:
        images = make_plots(data, report_dir, gain, diff)
    html = write_html(data, images, report_dir, gain, title, diff, groups)
    print('\nwrote {}'.format(html))
    for path in images:
        print('  {}'.format(path))

    if args.json:
        payload = _jsonable({k: v for k, v in data.items()
                             if k != 'uptime_scatter'})
        payload['gain_mm_per_rad'] = gain
        payload['groups'] = _jsonable(groups)
        payload['diff'] = _jsonable({k: v for k, v in (diff or {}).items()
                                     if k != 'delta_rad'})
        with open(args.json, 'w') as handle:
            json.dump(payload, handle, indent=2, sort_keys=False)
        print('  {}'.format(args.json))
    return 0


if __name__ == '__main__':
    sys.exit(main())
