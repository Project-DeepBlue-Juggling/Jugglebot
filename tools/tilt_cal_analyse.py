#!/usr/bin/env python3
"""Offline analyser for the tilt-calibration residual map (contract C-LEVEL-2).

Consumes either

  * a **map YAML** — ``config/tilt_calibration.yaml`` by default, resolved
    through the same production resolver the node uses; or
  * a **per-read CSV** from ``tests/hardware/tilt_cal_grid.py``
    (``temp/logs/tilt_cal_grid_<ts>.csv``), which works even when the capture
    refused to write a map (a failed node aborts the write, and that is exactly
    the run you most want to look at).

and reports the residual field, the per-node read spread, and any node that
looks like a measurement fault rather than a calibration.

Three things it is looking for
------------------------------

1. **The shape of the field.** The 2026-07-28 extremity table found the error
   growing 0.041° at (60, 0) to 0.604° at (150, −150) and *refuted* a 2-gain
   linear fit. A heat map plus a quiver of ``(tx, ty)`` says at a glance whether
   this capture agrees: same order of magnitude, same worst quadrant.

2. **Nodes the machine was unhappy at.** A leg sitting on its stroke clamp at an
   extremity node pins that node's pose without rejecting the move, so the
   reading is real but the *pose* is not the commanded one. It shows up as a
   local discontinuity against the node's neighbours — not as a large value, so
   a simple magnitude threshold misses it. Flagged on the neighbour residual,
   and separately on an inflated per-read sd.

3. **Whether the map is invariant under base tilt** (``--diff``, rung C2). Base
   tilt is common-mode and is absorbed by ``level``; the map captures only the
   robot's own pose-dependent error, so a recapture on a shimmed base must
   reproduce the same numbers. The diff mode reports per-node deltas and the max,
   and computes the rung's own PASS criterion — ``max(2 x noise, 0.05°)`` — from
   the sd the captures themselves recorded, rather than making the operator do it.

Usage
-----
    # the currently-installed calibration
    python tools/tilt_cal_analyse.py

    # a specific map, no plots, machine-readable
    python tools/tilt_cal_analyse.py config/tilt_calibration.yaml \\
        --no-plot --json temp/reports/tilt.json

    # a capture whose map was never written
    python tools/tilt_cal_analyse.py --csv temp/logs/tilt_cal_grid_<ts>.csv

    # rung C2 map-invariance: recapture vs baseline
    python tools/tilt_cal_analyse.py new_map.yaml --diff baseline_map.yaml

Offline analysis only: no ROS, no hardware, no motion. The ``--preview``
convention in ``tools/README.md`` binds the operator-facing harnesses that
*command the robot*; this commands nothing, so like ``cogging_map_analyse.py``
it takes plots and ``--json`` and no preview. Runs fine under the project venv or
the system python.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from datetime import datetime
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_SCRIPT_DIR)
# sys.path only — the *map* path is always resolved through tilt_map's own
# resolver below, never by walking from this file.
_JUGGLEBOT_PKG = os.path.join(_REPO_ROOT, 'ros_ws', 'src', 'jugglebot')
if _JUGGLEBOT_PKG not in sys.path:
    sys.path.insert(0, _JUGGLEBOT_PKG)

from jugglebot.motion import tilt_map  # noqa: E402

RAD2DEG = 180.0 / math.pi

#: Default C2 invariance floor, degrees — the plan's ``max(2 x noise, 0.05°)``.
DIFF_FLOOR_DEG = 0.05

#: A node whose per-read sd exceeds this multiple of the median node sd is
#: flagged. Multiplicative, not absolute, because the sensor's noise floor is
#: unmeasured until rung C0 and a hardcoded absolute would be a guess.
SD_FLAG_MULT = 3.0

#: **PROVISIONAL — rung C1 pins these.** Absolute floor and median multiple for
#: the curvature (kink) flag, degrees.
#:
#: The statistic (a second difference — see :func:`node_curvatures`) is sound;
#: the *threshold* is not yet knowable, and this file will not pretend it is.
#: The real field's curvature scale is unmeasured — the 2026-07-28 seed table has
#: a handful of points and refutes only a 2-gain *linear* fit — so on a probe
#: field with structured curvature the node curvatures cluster at several
#: distinct scales and **no multiple of the median separates a real 0.5° pin
#: from honest curvature**: 4x median missed it, 2x median flagged ten good
#: nodes. Picking a number that happens to work on one synthetic field and
#: shipping it as a validated threshold is exactly what the probe-first rule
#: forbids.
#:
#: So the flag is deliberately a **coarse net**, and the *diagnostic* is the
#: curvature ranking printed on every run: a node one place below the flag line
#: is visible to the operator either way. Rung C1 produces the first real map;
#: its curvature distribution is what should set these, and
#: ``--curvature-flag-deg`` overrides them in the meantime.
#:
#: The sharpest signal in the report needs **no** threshold at all: the
#: **top/median curvature ratio**. On a smooth field every node's second
#: difference is nearly the same, so the ratio sits near 1; a single pinned node
#: raises only its own (and mildly its neighbours'), so the ratio jumps. Probe
#: (2026-08-03, smooth quintic-ish field on a 5x5/±150 grid, 8 reads/node at
#: 2e-5 rad): **1.01 clean, 2.43 with a 0.2° pin, 3.65 at 0.3°, 6.07 at 0.5°** —
#: while the 4x-median flag line only fires at 0.5°. Read the ratio first.
CURVATURE_FLOOR_DEG = 0.10
CURVATURE_FLAG_MULT = 4.0

#: How many nodes of the curvature ranking to print. The ranking is the real
#: outlier diagnostic; the flag is only its loud subset.
CURVATURE_RANK_N = 5


class Field:
    """A residual field on a grid, however it was obtained."""

    def __init__(self, x_mm: Sequence[float], y_mm: Sequence[float],
                 tx: np.ndarray, ty: np.ndarray,
                 sd_tx: Optional[np.ndarray] = None,
                 sd_ty: Optional[np.ndarray] = None,
                 z_mm: Optional[float] = None,
                 version: str = '', source: str = '',
                 metadata: Optional[Dict[str, Any]] = None,
                 n_reads: Optional[np.ndarray] = None):
        self.x_mm = np.asarray(x_mm, dtype=float)
        self.y_mm = np.asarray(y_mm, dtype=float)
        self.tx = np.asarray(tx, dtype=float)
        self.ty = np.asarray(ty, dtype=float)
        self.sd_tx = None if sd_tx is None else np.asarray(sd_tx, dtype=float)
        self.sd_ty = None if sd_ty is None else np.asarray(sd_ty, dtype=float)
        self.z_mm = z_mm
        self.version = version
        self.source = source
        self.metadata = metadata or {}
        self.n_reads = None if n_reads is None else np.asarray(n_reads)

    @property
    def magnitude_deg(self) -> np.ndarray:
        return np.hypot(self.tx, self.ty) * RAD2DEG

    @property
    def shape(self) -> Tuple[int, int]:
        return (int(self.y_mm.size), int(self.x_mm.size))

    def median_sd_deg(self) -> float:
        """Median per-read sd across both axes and all nodes, in degrees."""
        pool = []
        for grid in (self.sd_tx, self.sd_ty):
            if grid is not None:
                finite = grid[np.isfinite(grid)]
                if finite.size:
                    pool.append(finite)
        if not pool:
            return float('nan')
        return float(np.median(np.concatenate(pool))) * RAD2DEG


def load_map_field(path: str) -> Field:
    """Read a map YAML through the **production** parser.

    Using ``tilt_map.parse_tilt_map`` rather than a local reader means the
    analyser refuses exactly what ``trajectory_node`` would refuse — so
    "the analyser was happy" is a statement about the machine, not about this
    script.
    """
    with open(path, 'r') as handle:
        doc = yaml.safe_load(handle)
    if not isinstance(doc, dict):
        raise SystemExit('ERROR: {} is not a tilt-map document'.format(path))
    parsed = tilt_map.parse_tilt_map(doc)
    stats = doc.get('stats') if isinstance(doc.get('stats'), dict) else {}

    def _grid(key):
        raw = stats.get(key)
        if raw is None:
            return None
        try:
            arr = np.asarray(raw, dtype=float)
        except (TypeError, ValueError):
            return None
        return arr if arr.shape == parsed.shape else None

    return Field(x_mm=parsed.x_mm, y_mm=parsed.y_mm,
                 tx=parsed.tx_rad, ty=parsed.ty_rad,
                 sd_tx=_grid('sd_tx_rad'), sd_ty=_grid('sd_ty_rad'),
                 z_mm=parsed.z_mm, version=parsed.version, source=path,
                 metadata=dict(parsed.metadata))


def load_csv_field(path: str) -> Field:
    """Reduce a per-read capture CSV into a field.

    Averages the ``res_*_rad`` columns, which already carry the mounting offset
    (``tilt_cal_grid.csv_row`` writes it per read), so this needs no config
    constant and cannot drift from the capture's own convention. Only
    ``phase == 'capture'`` rows with ``ok == 1`` participate; verification rows
    are reported separately.
    """
    by_node: Dict[Tuple[int, int], List[Tuple[float, float]]] = {}
    coords: Dict[Tuple[int, int], Tuple[float, float]] = {}
    checks: List[Dict[str, Any]] = []
    z_values: List[float] = []
    with open(path, 'r') as handle:
        for row in csv.DictReader(handle):
            try:
                ok = int(row['ok'])
                res = (float(row['res_tx_rad']), float(row['res_ty_rad']))
                x, y = float(row['x_mm']), float(row['y_mm'])
                z_values.append(float(row['z_mm']))
            except (KeyError, ValueError):
                continue
            if row.get('phase') == 'verify':
                if ok:
                    checks.append({'x_mm': x, 'y_mm': y,
                                   'res_tx_rad': res[0], 'res_ty_rad': res[1]})
                continue
            if not ok:
                continue
            key = (int(row['ix']), int(row['iy']))
            by_node.setdefault(key, []).append(res)
            coords[key] = (x, y)

    if not by_node:
        raise SystemExit(
            'ERROR: {} has no usable capture rows (every read failed, or the '
            'file is not a tilt_cal_grid CSV)'.format(path))

    xs = sorted({v[0] for v in coords.values()})
    ys = sorted({v[1] for v in coords.values()})
    n_y, n_x = len(ys), len(xs)
    tx = np.full((n_y, n_x), np.nan)
    ty = np.full((n_y, n_x), np.nan)
    sd_tx = np.full((n_y, n_x), np.nan)
    sd_ty = np.full((n_y, n_x), np.nan)
    counts = np.zeros((n_y, n_x), dtype=int)
    for key, reads in by_node.items():
        x, y = coords[key]
        ix, iy = xs.index(x), ys.index(y)
        arr = np.asarray(reads, dtype=float)
        tx[iy][ix] = float(np.mean(arr[:, 0]))
        ty[iy][ix] = float(np.mean(arr[:, 1]))
        if arr.shape[0] >= 2:
            sd_tx[iy][ix] = float(np.std(arr[:, 0], ddof=1))
            sd_ty[iy][ix] = float(np.std(arr[:, 1], ddof=1))
        counts[iy][ix] = arr.shape[0]

    field = Field(x_mm=xs, y_mm=ys, tx=tx, ty=ty, sd_tx=sd_tx, sd_ty=sd_ty,
                  z_mm=(float(np.median(z_values)) if z_values else None),
                  version='(from CSV, unwritten)', source=path,
                  n_reads=counts)
    field.metadata = {'check_poses': checks}
    return field


def node_curvatures(grid_deg: np.ndarray) -> Dict[Tuple[int, int], float]:
    """Max ``|v[i-1] - 2 v[i] + v[i+1]|`` per node, over both axes.

    The **second** difference, not a departure-from-neighbours: the residual
    field is genuinely curved (the 2026-07-28 table refutes a 2-gain linear
    fit), so any first-order test flags the entire boundary of a perfectly good
    capture — an edge node's neighbours all lie on its interior side, so a
    monotonic field always "departs" from their mean. A leg pinned on its stroke
    clamp does not bend the field, it **kinks** it, and a kink is exactly what a
    second difference measures. Edge nodes have no second difference along the
    axis they sit on and are tested only along the other one; a corner node
    cannot be tested at all, which is an honest limit rather than a silent pass.
    """
    curvature: Dict[Tuple[int, int], float] = {}
    n_y, n_x = grid_deg.shape

    def _record(iy, ix, value):
        if not math.isfinite(value):
            return
        key = (ix, iy)
        curvature[key] = max(curvature.get(key, 0.0), abs(float(value)))

    for iy in range(n_y):
        for ix in range(1, n_x - 1):
            _record(iy, ix, grid_deg[iy][ix - 1] - 2.0 * grid_deg[iy][ix]
                    + grid_deg[iy][ix + 1])
    for ix in range(n_x):
        for iy in range(1, n_y - 1):
            _record(iy, ix, grid_deg[iy - 1][ix] - 2.0 * grid_deg[iy][ix]
                    + grid_deg[iy + 1][ix])
    return curvature


def curvature_by_node(field: Field) -> Dict[Tuple[int, int], float]:
    """Per-node kink statistic, max over the signed ``tx``/``ty`` components.

    Signed components rather than the magnitude: a pin that shifts ``tx`` one
    way and ``ty`` the other can leave the magnitude field perfectly smooth.
    """
    curvature: Dict[Tuple[int, int], float] = {}
    for grid in (field.tx * RAD2DEG, field.ty * RAD2DEG):
        for key, value in node_curvatures(grid).items():
            curvature[key] = max(curvature.get(key, 0.0), value)
    return curvature


def curvature_limit(curvature: Dict[Tuple[int, int], float],
                    override: Optional[float] = None) -> Tuple[float, str]:
    """The flag line, and the sentence explaining where it came from."""
    if override is not None:
        return float(override), 'operator override --curvature-flag-deg'
    values = [v for v in curvature.values() if math.isfinite(v)]
    median = float(np.median(values)) if values else 0.0
    limit = max(CURVATURE_FLOOR_DEG, CURVATURE_FLAG_MULT * median)
    return limit, ('max({:.2f} deg floor, {:.0f}x median node curvature '
                   '{:.4f} deg) — PROVISIONAL, rung C1 pins it'
                   .format(CURVATURE_FLOOR_DEG, CURVATURE_FLAG_MULT, median))


def flag_outliers(field: Field, curvature_override: Optional[float] = None
                  ) -> List[Dict[str, Any]]:
    """Nodes that look like measurement faults rather than calibration.

    Two independent tests, because they catch different physics:

    * **sd inflation** — the platform was still moving when it was read, or the
      read path was unhealthy at that pose. Relative to the median node sd,
      since the sensor's absolute noise floor is unmeasured until rung C0.
    * **curvature kink** — the commanded pose was not the achieved pose (a leg
      pinned on its stroke clamp is the motivating case). The *value* can be
      perfectly ordinary; only its local discontinuity shows it. See
      :func:`node_curvatures` for why this is a second difference.
    """
    flags: List[Dict[str, Any]] = []
    n_y, n_x = field.shape
    magnitude = field.magnitude_deg

    sd_pool = []
    for grid in (field.sd_tx, field.sd_ty):
        if grid is not None:
            finite = grid[np.isfinite(grid)]
            if finite.size:
                sd_pool.append(finite)
    median_sd = (float(np.median(np.concatenate(sd_pool)))
                 if sd_pool else float('nan'))

    curvature = curvature_by_node(field)
    curv_limit, curv_basis = curvature_limit(curvature, curvature_override)

    for iy in range(n_y):
        for ix in range(n_x):
            reasons = []
            if math.isfinite(median_sd) and median_sd > 0.0:
                for name, grid in (('tx', field.sd_tx), ('ty', field.sd_ty)):
                    if grid is None:
                        continue
                    value = float(grid[iy][ix])
                    if math.isfinite(value) and value > SD_FLAG_MULT * median_sd:
                        reasons.append(
                            'sd_{} {:.6f} rad is {:.1f}x the median node sd '
                            '{:.6f}'.format(name, value, value / median_sd,
                                            median_sd))
            node_curv = curvature.get((ix, iy))
            if node_curv is not None and node_curv > curv_limit:
                reasons.append(
                    'local kink: second difference {:.4f} deg exceeds the '
                    '{:.4f} deg limit [{}]'
                    .format(node_curv, curv_limit, curv_basis))
            if reasons:
                flags.append({
                    'ix': ix, 'iy': iy,
                    'x_mm': float(field.x_mm[ix]),
                    'y_mm': float(field.y_mm[iy]),
                    'magnitude_deg': float(magnitude[iy][ix]),
                    'curvature_deg': node_curv,
                    'reasons': reasons,
                })
    return flags


def diff_fields(a: Field, b: Field) -> Dict[str, Any]:
    """Per-node ``a - b`` deltas, evaluated at ``a``'s nodes.

    When the axes match exactly this is a direct node-by-node subtraction. When
    they do not, ``b`` is **interpolated** onto ``a``'s nodes with the production
    ``tilt_map.lookup`` — still a fair comparison of the two calibrations as
    applied, but the report says which mode it used, because an interpolated
    diff carries the interpolation error of ``b`` on top of the real difference.
    """
    same_axes = (a.x_mm.shape == b.x_mm.shape and a.y_mm.shape == b.y_mm.shape
                 and bool(np.allclose(a.x_mm, b.x_mm))
                 and bool(np.allclose(a.y_mm, b.y_mm)))
    n_y, n_x = a.shape
    d_tx = np.full((n_y, n_x), np.nan)
    d_ty = np.full((n_y, n_x), np.nan)
    if same_axes:
        d_tx = a.tx - b.tx
        d_ty = a.ty - b.ty
    else:
        other = tilt_map.TiltMap(x_mm=b.x_mm, y_mm=b.y_mm, tx_rad=b.tx,
                                 ty_rad=b.ty, version=b.version)
        for iy in range(n_y):
            for ix in range(n_x):
                btx, bty = tilt_map.lookup(other, float(a.x_mm[ix]),
                                           float(a.y_mm[iy]))
                d_tx[iy][ix] = a.tx[iy][ix] - btx
                d_ty[iy][ix] = a.ty[iy][ix] - bty

    magnitude = np.hypot(d_tx, d_ty) * RAD2DEG
    finite = magnitude[np.isfinite(magnitude)]
    max_delta = float(np.max(finite)) if finite.size else float('nan')
    worst = (np.unravel_index(int(np.nanargmax(magnitude)), magnitude.shape)
             if finite.size else (0, 0))
    return {
        'same_axes': same_axes,
        'd_tx_rad': d_tx.tolist(),
        'd_ty_rad': d_ty.tolist(),
        'delta_deg': magnitude.tolist(),
        'max_delta_deg': max_delta,
        'mean_delta_deg': float(np.mean(finite)) if finite.size else float('nan'),
        'worst_node': {'iy': int(worst[0]), 'ix': int(worst[1]),
                       'x_mm': float(a.x_mm[int(worst[1])]),
                       'y_mm': float(a.y_mm[int(worst[0])])},
    }


def c2_threshold_deg(a: Field, b: Field,
                     override: Optional[float] = None) -> Tuple[float, str]:
    """Rung C2's ``max(2 x noise, 0.05°)``, computed from the captures' own sd."""
    if override is not None:
        return float(override), 'operator override --diff-threshold-deg'
    noises = [f.median_sd_deg() for f in (a, b)]
    noises = [n for n in noises if math.isfinite(n)]
    if not noises:
        return DIFF_FLOOR_DEG, ('no per-node sd recorded in either map — using '
                                'the {:.2f} deg floor alone'
                                .format(DIFF_FLOOR_DEG))
    noise = max(noises)
    value = max(2.0 * noise, DIFF_FLOOR_DEG)
    return value, ('max(2 x median per-read sd {:.4f} deg, floor {:.2f} deg)'
                   .format(noise, DIFF_FLOOR_DEG))


def print_report(field: Field, flags: Sequence[Dict[str, Any]],
                 diff: Optional[Dict[str, Any]] = None,
                 diff_other: Optional[Field] = None,
                 diff_threshold: Optional[Tuple[float, str]] = None,
                 curvature_override: Optional[float] = None) -> None:
    n_y, n_x = field.shape
    print('=' * 72)
    print('Tilt calibration — {}'.format(field.source))
    print('  version   : {}'.format(field.version))
    print('  grid      : {} x {} nodes at z = {} mm'
          .format(n_x, n_y, field.z_mm))
    print('  x_mm      : {}'.format([float(v) for v in field.x_mm]))
    print('  y_mm      : {}'.format([float(v) for v in field.y_mm]))
    for key in ('date', 'git_sha', 'base_condition', 'uptime_ms_first',
                'uptime_ms_last', 'level_offset_rad'):
        if key in field.metadata:
            print('  {:<10}: {}'.format(key, field.metadata[key]))

    magnitude = field.magnitude_deg
    finite = magnitude[np.isfinite(magnitude)]
    if finite.size:
        peak = np.unravel_index(int(np.nanargmax(magnitude)), magnitude.shape)
        print('\n  |residual| min {:.4f} deg, max {:.4f} deg at '
              '({:+.1f}, {:+.1f}), mean {:.4f} deg'
              .format(float(np.min(finite)), float(np.max(finite)),
                      float(field.x_mm[peak[1]]), float(field.y_mm[peak[0]]),
                      float(np.mean(finite))))
        print('  (2026-07-28 seed table for comparison: 0.041 deg at (60, 0) '
              'to 0.604 deg at (150, -150))')

    print('\n  residual magnitude [deg], rows = y descending:')
    header = '        ' + ''.join('{:>9.0f}'.format(float(v))
                                  for v in field.x_mm)
    print(header + '   <- x_mm')
    for iy in range(n_y - 1, -1, -1):
        cells = ''.join('{:>9.4f}'.format(float(magnitude[iy][ix]))
                        for ix in range(n_x))
        print('{:>7.0f} {}'.format(float(field.y_mm[iy]), cells))

    if field.sd_tx is not None or field.sd_ty is not None:
        print('\n  per-read sd [rad], max(sd_tx, sd_ty) per node:')
        print(header + '   <- x_mm')
        for iy in range(n_y - 1, -1, -1):
            cells = []
            for ix in range(n_x):
                values = [g[iy][ix] for g in (field.sd_tx, field.sd_ty)
                          if g is not None]
                values = [v for v in values if math.isfinite(v)]
                cells.append('{:>9.6f}'.format(max(values)) if values
                             else '{:>9}'.format('-'))
            print('{:>7.0f} {}'.format(float(field.y_mm[iy]), ''.join(cells)))
        median_sd = field.median_sd_deg()
        if math.isfinite(median_sd):
            print('  median per-read sd = {:.6f} deg ({:.6f} rad)'
                  .format(median_sd, median_sd / RAD2DEG))

    if field.n_reads is not None:
        low = [(int(field.x_mm[ix]), int(field.y_mm[iy]),
                int(field.n_reads[iy][ix]))
               for iy in range(n_y) for ix in range(n_x)
               if field.n_reads[iy][ix] < int(np.max(field.n_reads))]
        if low:
            print('\n  nodes with fewer usable reads than the best node:')
            for x, y, n in low:
                print('    ({:+5d}, {:+5d}): {} reads'.format(x, y, n))

    # The ranking is the real outlier diagnostic; the flag list below is only
    # its loud subset, and its threshold is provisional until rung C1.
    curvature = curvature_by_node(field)
    if curvature:
        limit, basis = curvature_limit(curvature, curvature_override)
        ranked = sorted(curvature.items(), key=lambda kv: -kv[1])
        print('\n  node curvature ranking (|second difference|, the kink '
              'statistic) — top {}:'.format(min(CURVATURE_RANK_N, len(ranked))))
        for (ix, iy), value in ranked[:CURVATURE_RANK_N]:
            print('    ({:+7.1f}, {:+7.1f})  {:.4f} deg{}'
                  .format(float(field.x_mm[ix]), float(field.y_mm[iy]), value,
                          '   <-- flagged' if value > limit else ''))
        print('    flag line {:.4f} deg [{}]'.format(limit, basis))
        median_curv = float(np.median([v for v in curvature.values()
                                       if math.isfinite(v)]))
        if median_curv > 0.0:
            ratio = ranked[0][1] / median_curv
            print('    top/median curvature ratio = {:.2f}  (scale-free: a '
                  'smooth field sits near 1.0 — probe 2026-08-03 measured 1.01 '
                  'clean, 2.43 with a 0.2 deg pin, 6.07 with 0.5 deg)'
                  .format(ratio))

    print('\n  outlier nodes: {}'.format(len(flags) or 'none'))
    for flag in flags:
        print('    ({:+.1f}, {:+.1f}) |r|={:.4f} deg'
              .format(flag['x_mm'], flag['y_mm'], flag['magnitude_deg']))
        for reason in flag['reasons']:
            print('        - {}'.format(reason))
    if flags:
        print('    A flagged node is ADVISORY: re-read it before trusting that '
              'corner of the map. The motivating fault is a leg pinned on its '
              'stroke clamp, which makes the achieved pose differ from the '
              'commanded one without any rejection.')

    checks = field.metadata.get('check_poses') if field.metadata else None
    if checks:
        print('\n  verification poses recorded in the CSV:')
        for chk in checks:
            mag = math.hypot(chk['res_tx_rad'], chk['res_ty_rad']) * RAD2DEG
            print('    ({:+7.1f}, {:+7.1f}) |r| = {:.4f} deg'
                  .format(chk['x_mm'], chk['y_mm'], mag))

    if diff is not None and diff_other is not None:
        print('\n' + '=' * 72)
        print('DIFF vs {}'.format(diff_other.source))
        print('  other version: {}'.format(diff_other.version))
        print('  axes {}'.format(
            'identical — direct node-by-node difference' if diff['same_axes']
            else 'DIFFER — the other map was interpolated onto these nodes, so '
                 'the delta carries its interpolation error too'))
        print('\n  per-node delta magnitude [deg], rows = y descending:')
        print(header + '   <- x_mm')
        delta = np.asarray(diff['delta_deg'], dtype=float)
        for iy in range(n_y - 1, -1, -1):
            cells = ''.join('{:>9.4f}'.format(float(delta[iy][ix]))
                            for ix in range(n_x))
            print('{:>7.0f} {}'.format(float(field.y_mm[iy]), cells))
        print('\n  max delta  {:.4f} deg at ({:+.1f}, {:+.1f})'
              .format(diff['max_delta_deg'], diff['worst_node']['x_mm'],
                      diff['worst_node']['y_mm']))
        print('  mean delta {:.4f} deg'.format(diff['mean_delta_deg']))
        if diff_threshold is not None:
            limit, why = diff_threshold
            verdict = ('PASS' if diff['max_delta_deg'] <= limit else 'FAIL')
            print('\n  C2 map-invariance threshold {:.4f} deg  [{}]'
                  .format(limit, why))
            print('  VERDICT: {}'.format(verdict))
            if verdict == 'FAIL':
                print('  A FAIL means the map is NOT invariant under the base '
                      'condition change — base tilt is not being fully absorbed '
                      'as common-mode by `level`, or one of the two captures '
                      'ran against a stale level reference.')
    print('=' * 72)


def cell_edges(nodes: Sequence[float]) -> List[float]:
    """Cell BOUNDARIES for an axis of node coordinates — ``len(nodes) + 1`` of them.

    Interior edges are the midpoints between adjacent nodes; the two outer edges
    reflect the first/last half-spacing outward. Each cell is therefore the set
    of positions **nearer to that node than to any neighbour**, and every node
    lies strictly inside its own cell — on a uniform axis *and* on a non-uniform
    one. (On a uniform axis that is also the exact cell centre; on a non-uniform
    axis a node sits off-centre within its cell, which is correct: the cell is a
    nearest-node region, not a centred box. Both properties cannot hold at once
    for unequal spacing, and "inside the right cell" is the one a reader needs.)

    This is what ``pcolormesh`` needs and what ``imshow(extent=...)`` cannot
    express. ``imshow`` spreads N pixels **evenly** across the extent, so with
    the extent set to the first and last NODE coordinates each pixel centre
    lands half a cell off its node — on the default 5×5 ±150 mm grid the colour
    cell for the node at x=−150 is centred at x=−120 while the text label sits
    at −150, a 30 mm disagreement. On a non-uniform axis (``--x-nodes`` with
    unequal spacing, which the tool accepts) the even spread is not merely
    offset: a node can land in a *different node's* pixel. Reading an outlier
    off such a plot then points the operator at the wrong node, which for a
    calibration map is the entire purpose of the plot.
    """
    values = [float(v) for v in nodes]
    n = len(values)
    if n == 0:
        return []
    if n == 1:
        # Degenerate (the grid spec forbids it) — a unit-wide cell keeps the
        # single node centred rather than producing an empty edge array.
        return [values[0] - 0.5, values[0] + 0.5]
    mids = [0.5 * (values[i] + values[i + 1]) for i in range(n - 1)]
    return ([values[0] - (mids[0] - values[0])]
            + mids
            + [values[-1] + (values[-1] - mids[-1])])


def make_plots(field: Field, report_dir: str,
               diff: Optional[Dict[str, Any]] = None) -> List[str]:
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception as exc:                             # noqa: BLE001
        print('  (plots skipped: matplotlib unavailable — {})'.format(exc))
        return []

    os.makedirs(report_dir, exist_ok=True)
    written: List[str] = []
    x_edges = np.asarray(cell_edges(field.x_mm), dtype=float)
    y_edges = np.asarray(cell_edges(field.y_mm), dtype=float)

    def heat(data_deg, title, fname, cmap='viridis', symmetric=False):
        fig, ax = plt.subplots(figsize=(7, 6))
        kwargs = {}
        if symmetric:
            peak = float(np.nanmax(np.abs(data_deg))) or 1.0
            kwargs = {'vmin': -peak, 'vmax': peak, 'cmap': 'coolwarm'}
        else:
            kwargs = {'cmap': cmap}
        # shading='flat' with (n+1)-length edges ⇒ one colour cell per node,
        # centred on it. The labels below use the NODE coordinates, so cell and
        # label are co-located by construction rather than by coincidence.
        image = ax.pcolormesh(x_edges, y_edges, np.asarray(data_deg, dtype=float),
                              shading='flat', **kwargs)
        ax.set_aspect('equal')
        for iy in range(field.shape[0]):
            for ix in range(field.shape[1]):
                value = float(data_deg[iy][ix])
                if math.isfinite(value):
                    ax.text(float(field.x_mm[ix]), float(field.y_mm[iy]),
                            '{:.3f}'.format(value), ha='center', va='center',
                            fontsize=7, color='w')
        fig.colorbar(image, ax=ax, label='deg')
        ax.set_xlabel('x [mm]')
        ax.set_ylabel('y [mm]')
        ax.set_title(title)
        fig.tight_layout()
        path = os.path.join(report_dir, fname)
        fig.savefig(path, dpi=110)
        plt.close(fig)
        written.append(path)

    heat(field.magnitude_deg, '|residual| [deg]', 'residual_magnitude.png')
    heat(field.tx * RAD2DEG, 'residual tx [deg]', 'residual_tx.png',
         symmetric=True)
    heat(field.ty * RAD2DEG, 'residual ty [deg]', 'residual_ty.png',
         symmetric=True)

    # Quiver — the field's direction, which the per-axis heat maps hide.
    fig, ax = plt.subplots(figsize=(7, 6))
    gx, gy = np.meshgrid(field.x_mm, field.y_mm)
    ax.quiver(gx, gy, field.tx * RAD2DEG, field.ty * RAD2DEG,
              field.magnitude_deg, cmap='viridis', angles='xy')
    ax.set_xlabel('x [mm]')
    ax.set_ylabel('y [mm]')
    ax.set_title('residual direction (arrow = (tx, ty), colour = |r| deg)')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    fig.tight_layout()
    path = os.path.join(report_dir, 'residual_quiver.png')
    fig.savefig(path, dpi=110)
    plt.close(fig)
    written.append(path)

    if diff is not None:
        heat(np.asarray(diff['delta_deg'], dtype=float),
             'per-node delta vs the other map [deg]', 'diff_magnitude.png')

    return written


def write_html(field: Field, flags: Sequence[Dict[str, Any]],
               images: Sequence[str], report_dir: str,
               diff: Optional[Dict[str, Any]] = None,
               diff_other: Optional[Field] = None) -> str:
    rows = []
    n_y, n_x = field.shape
    magnitude = field.magnitude_deg
    for iy in range(n_y - 1, -1, -1):
        cells = ''.join(
            '<td>{:.4f}</td>'.format(float(magnitude[iy][ix]))
            for ix in range(n_x))
        rows.append('<tr><th>{:.0f}</th>{}</tr>'.format(
            float(field.y_mm[iy]), cells))
    head = ''.join('<th>{:.0f}</th>'.format(float(v)) for v in field.x_mm)
    flag_html = ''.join(
        '<li>({:+.1f}, {:+.1f}) — {}</li>'.format(
            f['x_mm'], f['y_mm'], '; '.join(f['reasons'])) for f in flags)
    img_html = ''.join(
        '<figure><img src="{}" style="max-width:100%"><figcaption>{}'
        '</figcaption></figure>'.format(os.path.basename(p),
                                        os.path.basename(p))
        for p in images)
    diff_html = ''
    if diff is not None and diff_other is not None:
        diff_html = (
            '<h2>Diff vs {}</h2><p>max delta <b>{:.4f} deg</b> at '
            '({:+.1f}, {:+.1f}); mean {:.4f} deg; axes {}.</p>'.format(
                diff_other.source, diff['max_delta_deg'],
                diff['worst_node']['x_mm'], diff['worst_node']['y_mm'],
                diff['mean_delta_deg'],
                'identical' if diff['same_axes'] else 'interpolated'))

    html = (
        '<!doctype html><meta charset="utf-8">'
        '<title>Tilt calibration {version}</title>'
        '<style>body{{font-family:system-ui,sans-serif;margin:2rem;'
        'max-width:70rem}}table{{border-collapse:collapse}}'
        'td,th{{border:1px solid #ccc;padding:.25rem .5rem;text-align:right;'
        'font-variant-numeric:tabular-nums}}figure{{margin:1rem 0}}</style>'
        '<h1>Tilt calibration residual map</h1>'
        '<p><b>source</b> {source}<br><b>version</b> {version}<br>'
        '<b>grid</b> {nx} x {ny} at z = {z} mm</p>'
        '<h2>|residual| [deg]</h2>'
        '<table><tr><th>y \\ x</th>{head}</tr>{rows}</table>'
        '<h2>Outlier nodes ({nflags})</h2><ul>{flags}</ul>'
        '{diff}<h2>Plots</h2>{images}'
        '<p style="color:#666">Contract: ros_ws/docs/levelling_frame.md '
        '&sect; C-LEVEL-2. Generated {now}.</p>'
    ).format(source=field.source, version=field.version, nx=n_x, ny=n_y,
             z=field.z_mm, head=head, rows=''.join(rows),
             nflags=len(flags), flags=flag_html or '<li>none</li>',
             diff=diff_html, images=img_html,
             now=datetime.now().isoformat(timespec='seconds'))
    path = os.path.join(report_dir, 'index.html')
    with open(path, 'w') as handle:
        handle.write(html)
    return path


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('map', nargs='?', default=None,
                        help='tilt map YAML (default: the path the node would '
                             'resolve — $%s, then the repo source tree, then '
                             'the ament share copy)' % tilt_map.TILT_MAP_ENV)
    parser.add_argument('--csv', default=None,
                        help='analyse a per-read capture CSV instead of a map '
                             'YAML (works when the capture wrote no map).')
    parser.add_argument('--diff', default=None,
                        help='second map YAML; report per-node deltas and the '
                             'max (rung C2 map-invariance).')
    parser.add_argument('--diff-threshold-deg', type=float, default=None,
                        help='override the C2 PASS bound (default: computed as '
                             'max(2 x median per-read sd, %.2f deg)).'
                             % DIFF_FLOOR_DEG)
    parser.add_argument('--curvature-flag-deg', type=float, default=None,
                        help='override the kink-flag line (default: computed, '
                             'PROVISIONAL until rung C1 — the printed curvature '
                             'ranking is the real diagnostic).')
    parser.add_argument('--no-plot', action='store_true')
    parser.add_argument('--report-dir', default=None,
                        help='default temp/reports/tilt_cal_<ts>/')
    parser.add_argument('--json', default=None,
                        help='write a machine-readable summary here.')
    args = parser.parse_args(argv)

    if args.csv:
        field = load_csv_field(args.csv)
    else:
        path = args.map or tilt_map.resolve_tilt_map_path()
        if path is None:
            print('ERROR: no tilt map found at any candidate path ({}). Pass '
                  'one explicitly, or --csv a capture log.'
                  .format('; '.join(tilt_map.tilt_map_candidates()) or '<none>'),
                  file=sys.stderr)
            return 1
        if not os.path.exists(path):
            print('ERROR: no such file {}'.format(path), file=sys.stderr)
            return 1
        field = load_map_field(path)

    flags = flag_outliers(field, args.curvature_flag_deg)

    diff = other = threshold = None
    if args.diff:
        if not os.path.exists(args.diff):
            print('ERROR: no such file {}'.format(args.diff), file=sys.stderr)
            return 1
        other = load_map_field(args.diff)
        diff = diff_fields(field, other)
        threshold = c2_threshold_deg(field, other, args.diff_threshold_deg)

    print_report(field, flags, diff, other, threshold,
                 args.curvature_flag_deg)

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    report_dir = args.report_dir or os.path.join(
        _REPO_ROOT, 'temp', 'reports', 'tilt_cal_{}'.format(stamp))
    images: List[str] = []
    if not args.no_plot:
        images = make_plots(field, report_dir, diff)
        if images:
            html = write_html(field, flags, images, report_dir, diff, other)
            print('\nReport -> {}'.format(html))

    if args.json:
        payload = {
            'source': field.source,
            'version': field.version,
            'z_mm': field.z_mm,
            'x_mm': [float(v) for v in field.x_mm],
            'y_mm': [float(v) for v in field.y_mm],
            'residual_rad': {'tx': field.tx.tolist(), 'ty': field.ty.tolist()},
            'magnitude_deg': field.magnitude_deg.tolist(),
            'sd_tx_rad': (None if field.sd_tx is None
                          else field.sd_tx.tolist()),
            'sd_ty_rad': (None if field.sd_ty is None
                          else field.sd_ty.tolist()),
            'median_sd_deg': field.median_sd_deg(),
            'metadata': field.metadata,
            'outliers': list(flags),
            'diff': (None if diff is None else dict(
                diff, other_source=other.source, other_version=other.version,
                threshold_deg=threshold[0], threshold_basis=threshold[1],
                verdict=('PASS' if diff['max_delta_deg'] <= threshold[0]
                         else 'FAIL'))),
        }
        target = os.path.abspath(args.json)
        os.makedirs(os.path.dirname(target), exist_ok=True)
        with open(target, 'w') as handle:
            json.dump(payload, handle, indent=2, default=str)
        print('JSON   -> {}'.format(target))

    if diff is not None and threshold is not None:
        return 0 if diff['max_delta_deg'] <= threshold[0] else 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
