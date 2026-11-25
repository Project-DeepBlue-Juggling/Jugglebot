#!/usr/bin/env python3
"""Ball-to-ball clearance of JP-14 Centre and orbit (``centre_orbit``), offline.

Compiles the REAL Schedule (``nominate.build_pattern`` -> ``schedule.
compile_columns``, the t0 entry) for each (radius, Sites per lap) and walks
both balls' nominal centre paths on a 1 ms grid:

* in flight: xy linear from the release Site to the aimed Site, z the
  symmetric ballistic arc above the Catch plane to the commanded Apex
  (``schedule.flight_s``'s own model: release and Catch at the Catch plane);
* in the cup (touch-down to release): at its Site, on the Catch plane.

Both balls ride the same cup-height convention, so the common seat offset
cancels.  Reports, per configuration: the chord Ball 2 travels, the
closest xy approach of its chord to the centre column (``r cos(pi / N)``,
the geometric floor), the minimum 3D centre distance between the two balls
with the instant and the Throw it falls in, and the margin over one ball
diameter (``2 * JUGGLING_BALL_RADIUS_MM``, hardware_config).  Nominal paths
only: landing scatter (sd about 20 mm per axis on the robot) eats the margin.

    python tools/probes/centre_orbit_clearance.py [--radius 95 100 105]
        [--n 6 8 12] [--apex 0.95] [--dwell 0.27] [--csv]

Writes nothing unless ``--csv`` (then ``temp/probes/centre_orbit_clearance_
<stamp>.csv``).  Logbook: ``2026-11-25-jp14-centre-and-orbit-compile.md``.
"""
from __future__ import annotations

import argparse
import csv
import datetime
import math
import os
import sys

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(
    __file__))))
sys.path.insert(0, os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot'))

import jugglebot.hardware_config as hw  # noqa: E402
from jugglebot.motion.skills import nominate as nm  # noqa: E402
from jugglebot.motion.skills import schedule as sc  # noqa: E402
from jugglebot.motion.skills.segments import CATCH, THROW  # noqa: E402

G_MM_S2 = 9810.0
BALL_DIAMETER_MM = 2.0 * float(hw.JUGGLING_BALL_RADIUS_MM)


def flights_of(sched):
    """``[(ball, t_release, xy_from, t_catch, xy_to, throw_index)]`` read off
    the Schedule's own skills (launch THROW, carried throws)."""
    f = float(sched.flight_s)
    out = []
    k = 0
    for sk in sched.skills:
        if sk.kind == THROW:
            out.append((sk.ball_id, sk.t_abs_s, sk.site.cup_mm[:2],
                        sk.t_abs_s + f, sk.target.cup_mm[:2], k))
            k += 1
        elif sk.kind == CATCH and sk.then_throw is not None:
            tt = sk.then_throw
            out.append((sk.ball_id, tt.t_release_abs_s,
                        tt.release_site_or(sk.site).cup_mm[:2],
                        tt.t_release_abs_s + f, tt.target.cup_mm[:2], k))
            k += 1
    out.sort(key=lambda r: r[1])
    return out


def ball_path(flights, ball, t, f, apex_mm, inflight_xy, t_first_catch):
    """(T, 3) nominal centre path of ``ball`` on grid ``t`` (Catch plane z=0)."""
    pos = np.full((len(t), 3), np.nan)
    mine = [r for r in flights if r[0] == ball]
    v0 = 4.0 * apex_mm / f          # symmetric arc: z(f / 2) = apex
    segs = []
    if ball == 1 and inflight_xy is not None:
        # the pre-existing flight (the Feed) lands at the first CATCH
        segs.append((t_first_catch - f, inflight_xy, t_first_catch,
                     inflight_xy))
    segs += [(r[1], r[2], r[3], r[4]) for r in mine]
    for i, (tr, a, tc, b) in enumerate(segs):
        m = (t >= tr) & (t <= tc)
        s = t[m] - tr
        lam = s / f
        pos[m, 0] = a[0] + (b[0] - a[0]) * lam
        pos[m, 1] = a[1] + (b[1] - a[1]) * lam
        pos[m, 2] = v0 * s - 0.5 * G_MM_S2 * s * s
        if i + 1 < len(segs):     # held in the cup until the next release
            m2 = (t > tc) & (t < segs[i + 1][0])
            pos[m2] = [b[0], b[1], 0.0]
    return pos


def clearance(radius_mm, n, apex_m=0.95, dwell_s=0.27, stop=False,
              columns=False, n_throws=None):
    """``stop=False``: the steady lap, up to the Stop's release (the Stop
    is aimed to land ON the held ball, so the two balls close by design, as
    in Columns).  ``stop=True``: the whole Schedule.  ``columns=True``: the
    Columns control at separation ``radius_mm`` (one Columns lap = ``2 n +
    1`` Throws), the shape every centre-orbit transit repeats.
    ``n_throws`` None = one closed lap (``2 n + 1``: the Stop is Ball 1's,
    from the centre onto Ball 2 at O1); an EVEN count ends on Ball 2's
    Throw onto Ball 1 at the centre instead."""
    cfg = nm.NominationConfig(
        pattern='columns' if columns else sc.CENTRE_ORBIT, apex_m=apex_m,
        separation_mm=radius_mm, dwell_s=dwell_s, orbit_n=n,
        n_throws=(2 * n + 1) if columns else n_throws)
    pat = nm.build_pattern(cfg)
    sched = sc.compile_columns(pat, t0_abs_s=10.0)
    f = float(sched.flight_s)
    fl = flights_of(sched)
    first_catch_b = min(s.t_abs_s for s in sched.skills
                        if s.kind == CATCH and s.ball_id == 1)
    t = np.arange(10.0, fl[-1][3] + 1e-9, 0.001)
    pa = ball_path(fl, 0, t, f, apex_m * 1000.0, None, None)
    pb = ball_path(fl, 1, t, f, apex_m * 1000.0, pat.sites[1].cup_mm[:2],
                   first_catch_b)
    ok = ~(np.isnan(pa[:, 0]) | np.isnan(pb[:, 0]))
    if not stop:
        ok &= t < fl[-1][1]
    d = np.linalg.norm(pa - pb, axis=1)
    d[~ok] = np.inf
    i = int(np.argmin(d))
    dxy = np.linalg.norm(pa[:, :2] - pb[:, :2], axis=1)
    dxy[~ok] = np.inf
    j = int(np.argmin(dxy))
    beat = (t[i] - 10.0) / float(sched.beat_s)
    throw = max((r[5] for r in fl if r[1] <= t[i]), default=0)
    return dict(kind=('columns' if columns else 'orbit')
                + ('+stop' if stop else ''), radius_mm=radius_mm, n=n, apex_m=apex_m, dwell_s=dwell_s,
                chord_mm=2.0 * radius_mm * math.sin(math.pi / n),
                chord_to_centre_mm=radius_mm * math.cos(math.pi / n),
                min_3d_mm=float(d[i]), t_min_s=float(t[i] - 10.0),
                beat_at_min=beat, throw_at_min=int(throw),
                ball_a_z_mm=float(pa[i, 2]), ball_b_z_mm=float(pb[i, 2]),
                min_xy_mm=float(dxy[j]),
                margin_3d_mm=float(d[i]) - BALL_DIAMETER_MM)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--radius', type=float, nargs='+',
                    default=[95.0, 100.0, 105.0])
    ap.add_argument('--n', type=int, nargs='+', default=[6, 8, 10, 12])
    ap.add_argument('--apex', type=float, default=0.95)
    ap.add_argument('--dwell', type=float, default=0.27)
    ap.add_argument('--n-throws', type=int, default=None,
                    help='orbit rows only (default one closed lap, 2 N + 1)')
    ap.add_argument('--csv', action='store_true')
    a = ap.parse_args(argv)
    rows = []
    for r in a.radius:
        for n in a.n:
            for stop in (False, True):
                rows.append(clearance(r, n, a.apex, a.dwell, stop=stop,
                                      n_throws=a.n_throws))
        for stop in (False, True):
            rows.append(clearance(r, a.n[0], a.apex, a.dwell, stop=stop,
                                  columns=True))
    print('ball diameter %.1f mm (hardware_config); Apex %.2f m, Dwell %.2f s'
          % (BALL_DIAMETER_MM, a.apex, a.dwell))
    print('kind          radius  N  chord  chord-to-C  min3D  margin  at t (s) beat Throw '
          'zA/zB (mm)  min xy (both tracked)')
    for r in rows:
        print('%-13s %6.1f %2d %6.1f %10.1f %6.1f %7.1f %8.3f %4.1f %5d %5.0f/%-5.0f'
              ' %6.1f' % (r['kind'], r['radius_mm'], r['n'], r['chord_mm'],
                          r['chord_to_centre_mm'], r['min_3d_mm'],
                          r['margin_3d_mm'], r['t_min_s'], r['beat_at_min'],
                          r['throw_at_min'], r['ball_a_z_mm'],
                          r['ball_b_z_mm'], r['min_xy_mm']))
    if a.csv:
        os.makedirs(os.path.join(_REPO, 'temp', 'probes'), exist_ok=True)
        path = os.path.join(_REPO, 'temp', 'probes',
                            'centre_orbit_clearance_%s.csv'
                            % datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
        with open(path, 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        print('wrote', path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
