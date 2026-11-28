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
import os
import sys

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(
    __file__))))
sys.path.insert(0, os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot'))

from jugglebot.motion.skills.orbit_clearance import (  # noqa: E402
    BALL_DIAMETER_MM, clearance)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--radius', type=float, nargs='+',
                    default=[95.0, 100.0, 105.0])
    ap.add_argument('--n', type=int, nargs='+', default=[6, 8, 10, 12])
    ap.add_argument('--apex', type=float, default=0.95)
    ap.add_argument('--dwell', type=float, default=0.27)
    ap.add_argument('--n-throws', type=int, default=None,
                    help='orbit rows only (default 2 N + 2; an odd count is refused '
                         'by the Pattern and measured here only)')
    ap.add_argument('--csv', action='store_true')
    a = ap.parse_args(argv)
    rows = []
    for r in a.radius:
        for n in a.n:
            for stop in (False, True):
                rows.append(clearance(r, n, a.apex, a.dwell, stop=stop,
                                      n_throws=a.n_throws))
            if a.n_throws is None:     # the refused odd count, for the record
                row = clearance(r, n, a.apex, a.dwell, stop=True,
                                n_throws=2 * n + 1)
                row['kind'] = 'orbit+stop ODD'
                rows.append(row)
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
