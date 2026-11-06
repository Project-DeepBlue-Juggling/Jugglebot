#!/usr/bin/env python3
"""Migrate a schema-v1 Learner memory (``memory.csv``) to schema v2
(``memory_v2.csv``): the one-off for ``plans/active/learner-generalisation.md``
section 2.3 (the residual memory is keyed on the release position, release
velocity and release kind, not on the old site xy + seat offset 4-vector).

Reads the v1 file READ-ONLY, applying the v1 loader's own row rules (13
columns, finite numbers, the apex band ``memory.APEX_RATIO_BAND``; a row that
breaks them is DROPPED and counted), and writes a NEW v2 file. Idempotent: a
second run with the same input finds a byte-identical ``--out`` and says so;
an existing ``--out`` whose content would DIFFER is refused (nothing written).
The v1 file is never opened for writing. The owner re-runs this on the live
memory at merge (``--memory temp/learn/jugglebot/memory.csv --out
temp/learn/jugglebot/memory_v2.csv``).

Per v1 row (x0, x1, x2, x3, u0, u1, u2_apex_m, y0, y1, y2_apex_m, t, ball, caught):

* site = (x0, x1); seat offset = (x2, x3); release xy = site + seat offset
  for a ``free_cup`` row (a same-Site fold releases where the ball was
  caught) and the site EXACTLY for a ``carried`` row (the cup is carried to
  the release Site; the seat offset is the catch offset at the other Site);
  release z = ``sites.RELEASE_CUP_Z_MM`` (860 mm, the throw plane the apex is
  measured from);
* release velocity = ``ballistics_bc.launch_velocity(release, target,
  schedule.flight_s(u2_apex_m))`` converted to m/s, the target by row class
  (Site signature only; a v1 row never recorded the pattern):

  - ``carried``: |x0| = 0.075 m and x1 = 0 (the oval's T Site at separation
    150 mm; all such rows are from 2026-10-23 on): release_kind 'carried',
    target = the OTHER oval Site, (-x0, 0) at the throw plane (+0.150 m in x
    from T);
  - ``hop``: |x0| = 0.125 m (the hop at separation 250 mm): 'free_cup',
    target = the mirrored Site (-x0, x1) at the throw plane;
  - ``vertical``: every other row: 'free_cup', target = the release Site
    (vertical throw, vz = +g T / 2 up);

* dwell_s = nan and migrated = 1.

NOT recoverable from a v1 row: the dwell (so nan; the Learner's dwell term is
inert), and the pattern a row was thrown in beyond its Site signature (a
hop-Site row thrown vertically would be mislabelled; the corpus has none the
signature can tell apart). A row whose commanded apex is not > 0 has no flight
time and is dropped (counted separately).

Usage:
    python tools/migrate_memory_schema_v2.py --memory temp/learn/X/memory.csv \\
        --out temp/learn/X/memory_v2.csv [--dry-run]
"""

from __future__ import annotations

import argparse
import collections
import csv
import io
import math
import os
import sys

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, 'ros_ws', 'src', 'jugglebot')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from jugglebot.motion.skills import memory as mem               # noqa: E402
from jugglebot.motion.skills import schedule as sch             # noqa: E402
from jugglebot.motion.skills import sites                       # noqa: E402
from jugglebot.motion.trajectory import ballistics_bc as bbc    # noqa: E402

#: |x0| (m) of the oval's T Site at separation 150 mm (x0 = -0.075 in the corpus).
OVAL_SITE_X_M = 0.075
#: |x0| (m) of the hop at separation 250 mm.
HOP_SITE_X_M = 0.125
#: Tolerance (m) on the Site signature match (the rows hold %.17g of 0.075 etc.).
SITE_TOL_M = 1e-6
#: Throw-plane z (m): where the apex is measured from.
RELEASE_Z_M = sites.RELEASE_CUP_Z_MM / 1000.0

CLASSES = ('carried', 'hop', 'vertical')


def classify(x0: float, x1: float) -> str:
    """Row class from the Site signature (see the module docstring)."""
    if abs(abs(x0) - OVAL_SITE_X_M) <= SITE_TOL_M and abs(x1) <= SITE_TOL_M:
        return 'carried'
    if abs(abs(x0) - HOP_SITE_X_M) <= SITE_TOL_M:
        return 'hop'
    return 'vertical'


def parse_v1_row(row):
    """The v1 loader's own row rules; returns the 13 parsed values
    (floats x11, ball_id, caught) or raises ValueError."""
    if len(row) != len(mem.HEADER_V1):
        raise ValueError('expected %d columns, got %d'
                         % (len(mem.HEADER_V1), len(row)))
    vals = [float(v) for v in row[:11]]
    if not all(math.isfinite(v) for v in vals):
        raise ValueError('non-finite numeric field')
    ball_id = int(row[11])
    caught = row[12].strip().lower() in ('true', '1')
    if not mem.apex_in_band(vals[6], vals[9]):
        raise ValueError('apex outside the admit band')
    return vals, ball_id, caught


def migrate_row(vals, ball_id, caught):
    """One v1 row -> (Experience, class). ValueError if no flight time."""
    x0, x1, x2, x3 = vals[0:4]
    u, y, t_abs_s = vals[4:7], vals[7:10], vals[10]
    if not u[2] > 0.0:
        raise ValueError('commanded apex %r has no flight time' % (u[2],))
    cls = classify(x0, x1)
    if cls == 'carried':
        release_mm = np.array([x0 * 1000.0, x1 * 1000.0, RELEASE_Z_M * 1000.0])
    else:
        release_mm = np.array([(x0 + x2) * 1000.0, (x1 + x3) * 1000.0,
                               RELEASE_Z_M * 1000.0])
    if cls == 'vertical':
        target_site = (x0, x1)
    else:
        target_site = (-x0, x1 if cls == 'hop' else 0.0)
    target_mm = np.array([target_site[0] * 1000.0, target_site[1] * 1000.0,
                          RELEASE_Z_M * 1000.0])
    vel = bbc.launch_velocity(release_mm, target_mm, sch.flight_s(u[2])) / 1000.0
    exp = mem.Experience(
        site_xy_m=[x0, x1], release_m=release_mm / 1000.0,
        release_vel_mps=vel, dwell_s=float('nan'), seat_offset_m=[x2, x3],
        u=u, y=y, t_abs_s=t_abs_s, ball_id=ball_id,
        release_kind='carried' if cls == 'carried' else 'free_cup',
        caught=caught, migrated=True)
    return exp, cls


def convert(memory_path: str):
    """Read ``memory_path`` read-only; return (v2 file text, stats dict)."""
    stats = {'read': 0, 'written': 0, 'dropped': 0, 'no_flight': 0,
             'classes': collections.Counter()}
    out = io.StringIO()
    writer = csv.writer(out)
    writer.writerow(mem.HEADER)
    with open(memory_path, 'r', newline='') as handle:
        reader = csv.reader(handle)
        header = next(reader, None)
        if header is None:
            raise ValueError('%s has no header row' % (memory_path,))
        cols = [str(c).strip() for c in header]
        if cols != mem.HEADER_V1:
            raise ValueError('%s has header %r, expected the schema-v1 header '
                             '%r' % (memory_path, cols, mem.HEADER_V1))
        for row in reader:
            stats['read'] += 1
            try:
                vals, ball_id, caught = parse_v1_row(row)
            except (ValueError, TypeError):
                stats['dropped'] += 1
                continue
            try:
                exp, cls = migrate_row(vals, ball_id, caught)
            except ValueError:
                stats['dropped'] += 1
                stats['no_flight'] += 1
                continue
            writer.writerow(mem.format_row(exp))
            stats['written'] += 1
            stats['classes'][cls] += 1
    return out.getvalue(), stats


def summary(memory_path, out_path, stats, verb):
    return ('%s: %s %d of %d rows read (%d dropped by the v1 loader rules, '
            'of which %d have no flight time) -> %s; carried %d, hop %d, '
            'vertical %d'
            % (memory_path, verb, stats['written'], stats['read'],
               stats['dropped'], stats['no_flight'], out_path,
               stats['classes']['carried'], stats['classes']['hop'],
               stats['classes']['vertical']))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--memory', required=True, help='the schema-v1 memory.csv (read only)')
    ap.add_argument('--out', required=True, help='the schema-v2 file to write (new)')
    ap.add_argument('--dry-run', action='store_true', help='report; write nothing')
    args = ap.parse_args(argv)
    if os.path.abspath(args.memory) == os.path.abspath(args.out):
        print('ERROR: --out must not be --memory', file=sys.stderr)
        return 1
    try:
        text, stats = convert(args.memory)
    except (OSError, ValueError) as exc:
        print('ERROR: %s' % (exc,), file=sys.stderr)
        return 1
    data = text.encode('utf-8')
    if os.path.exists(args.out):
        with open(args.out, 'rb') as handle:
            existing = handle.read()
        if existing != data:
            print('REFUSED: %s exists and differs from what this migration '
                  'would write; nothing written' % (args.out,), file=sys.stderr)
            return 1
        print(summary(args.memory, args.out, stats, 'already up to date,'))
        print('%s is byte-identical to the migration output; nothing to do'
              % (args.out,))
        return 0
    if args.dry_run:
        print(summary(args.memory, args.out, stats, 'would write'))
        return 0
    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.out, 'wb') as handle:
        handle.write(data)
    print(summary(args.memory, args.out, stats, 'wrote'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
