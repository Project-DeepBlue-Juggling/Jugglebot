#!/usr/bin/env python3
"""Run ONE real admissible-sweep family with every per-cell row captured to JSON.

The committed sweep keeps only the extracted rectangle; this probe keeps the whole
pass grid (per site pair, flight, offset: ok, refusing gate code, wall time) so
``boundary_replay.py`` can test boundary-only certification against it without
new solves. Same grid, same cell functions, same limits as ``tools/admissible_sweep.py``.

usage: python tools/probes/admissible_obviation/family_run.py <columns|hop|self_toss|oval_1ball> <level|pose> <apex> <out.json>
          [--sep MM] [--angle DEG] [--dwell S] [--limits VEL ACC JERK HAND_ACC]

2026-11-04 timings on a 4-core dev box, 3 runs in parallel: a columns family at one
apex (7 flights x 2 pairs x 289 offsets, up to 3 solves each) takes about 185 s.

Part of the admissible-box sweep obviation investigation
(``logbook/2026-11-04-admissible-box-sweep-obviation-investigation.md``).
Offline, no hardware, no ROS: imports the production planner + gate exactly as
``tools/admissible_sweep.py`` does. Outputs go under ``temp/probes/admissible_obviation/``.
BLAS is pinned to one thread (the sweep's determinism rule).
"""

import json, os, sys, time
for v in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[v] = '1'
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.insert(0, ROOT); sys.path.insert(0, os.path.join(ROOT, 'ros_ws', 'src', 'jugglebot'))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
import admissible_sweep as asw
from jugglebot.motion.skills import sites as st

import argparse
ap = argparse.ArgumentParser()
ap.add_argument('pattern'); ap.add_argument('mode'); ap.add_argument('apex', type=float); ap.add_argument('out')
ap.add_argument('--sep', type=float, default=None); ap.add_argument('--angle', type=float, default=0.0)
ap.add_argument('--dwell', type=float, default=0.27)
ap.add_argument('--limits', type=float, nargs=4, default=[600.0, 8000.0, 200000.0, 4500.0])
a = ap.parse_args()
pattern, mode, apex, out = a.pattern, a.mode, a.apex, a.out
sep = a.sep if a.sep is not None else (250.0 if pattern == 'hop' else 150.0 if pattern == 'oval_1ball' else 125.0)
rows = []
def log(r):
    rows.append(r)
t0 = time.perf_counter()
kw = dict(flight_frac=asw.SINGLE_APEX_FLIGHT_FRAC, halfwidth_m=0.025,
          offsets_mm=(asw.HOP_OFFSETS_MM if pattern == 'hop' else asw.OFFSETS_MM),
          dwell_s=a.dwell, separation_mm=sep, leg_vel=a.limits[0], leg_acc=a.limits[1],
          leg_jerk=a.limits[2], hand_acc=a.limits[3], pattern=pattern, planner_mode=mode,
          swept_at='2026-11-04', log=log, angle_deg=a.angle)
if pattern == 'self_toss':
    kw['site'] = st.columns_sites(100.0, 0.0)[0]
boxes, _ = asw._single_apex_boxes([apex], **kw)
wall = time.perf_counter() - t0
res = dict(pattern=pattern, mode=mode, apex=apex, separation_mm=sep, wall_s=wall, angle=a.angle, dwell=a.dwell, limits=a.limits,
           n_rows=len(rows), rows=rows,
           boxes=[dict(site_pair=b.site_pair, landing_xy_m=b.landing_xy_m, apex_m=b.apex_m,
                       apex_band_m=b.apex_band_m) for b in boxes])
json.dump(res, open(out, 'w'))
print('done', pattern, mode, apex, 'rows', len(rows), 'wall_s', round(wall, 1),
      'boxes', res['boxes'])
