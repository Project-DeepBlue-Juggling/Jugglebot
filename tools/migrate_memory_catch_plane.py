#!/usr/bin/env python3
"""Migrate a learner's ``memory.csv`` off the old plane-unaware apex reading
(plan R5 sitting 6, 2026-10-05: CATCH HIGH, ``sites.CATCH_CUP_Z_MM`` 830 ->
930 mm).

Every ``temp/learn/<plant_id>/memory.csv`` row written before CATCH HIGH has
its ``y2_apex_m`` read off ``schedule.apex_from_vz`` on the catch-plane
crossing speed alone -- correct ONLY because release (``RELEASE_CUP_Z_MM``,
860 mm) and catch (830 mm) sat ``--rise-old-m`` (default -0.030 m) apart and
that rise was never otherwise accounted for. ``schedule.apex_from_crossing``
(added the same sitting) folds the rise in, so a fresh row written today
reads ``y2_apex_m`` relative to the RELEASE plane, not whichever catch plane
happens to be live. This tool brings the OLD rows into that same frame: it
recovers each row's catch-plane crossing speed (the exact inverse of
``apex_from_vz``) and re-derives ``y2_apex_m`` through ``apex_from_crossing``
at the OLD rise, so every row in the file means the same physical quantity
regardless of when it was written.

Only ``y2_apex_m`` changes. ``u2_apex_m`` (the commanded apex) is left alone:
it was never read off ``apex_from_vz`` -- it is the ``schedule.flight_s``
input the planner solved the throw's endpoints and duration from directly,
so it was never biased by the plane offset in the first place. ``y0``/``y1``
(the observed landing xy) are also left alone: these are near-vertical
columns throws, and re-targeting a lateral measurement for a ~100 mm vertical
plane shift moves it by under 3 mm -- not worth a second, parallel correction
for a sub-mm effect that is smaller than this learner's own measurement noise.

Only rows with ``t_abs_s < --cutoff-epoch`` are touched; rows at or after the
cutoff were already recorded under the new reading. A row's catch time is
assumed to have been read relative to the OLD plane (symmetric about the
schedule's own learner-row write order, which is append-only and
chronological; the whole FILE is from one plant, so one cutoff suffices).

Safety: writes a timestamped backup (``<memory>.bak-<UTC-ISO>``) before
touching the live file, then a sidecar marker (``<memory>.
catch_plane_migrated``) on success. A second invocation against the same
file REFUSES outright (before touching anything) if that marker exists --
the inverse step this tool runs is NOT idempotent against an
already-migrated value, so re-running it would silently read the NEW apex as
if it still meant "above the 830 mm plane" and corrupt the row a second
time.

Usage:
    python tools/migrate_memory_catch_plane.py \\
        --memory temp/learn/sim/memory.csv --cutoff-epoch 1791100000
    python tools/migrate_memory_catch_plane.py \\
        --memory temp/learn/sim/memory.csv --cutoff-epoch 1791100000 --dry-run
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import datetime as _dt
import math
import os
import shutil
import sys
from typing import List

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, 'ros_ws', 'src', 'jugglebot')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from jugglebot.motion.skills import schedule as sch                # noqa: E402
from jugglebot.motion.skills.memory import _FLOAT_FMT, _HEADER     # noqa: E402

#: Index of ``y2_apex_m`` in :data:`jugglebot.motion.skills.memory._HEADER`
#: -- the one column this tool rewrites.
_Y2_COL = _HEADER.index('y2_apex_m')
#: Index of ``t_abs_s`` -- the column the ``--cutoff-epoch`` split reads.
_T_ABS_COL = _HEADER.index('t_abs_s')

_MARKER_SUFFIX = '.catch_plane_migrated'


class AlreadyMigrated(RuntimeError):
    """Raised when the sidecar marker already exists for ``--memory``."""


def marker_path(memory_path: str) -> str:
    return memory_path + _MARKER_SUFFIX


def _backup_path(memory_path: str) -> str:
    stamp = _dt.datetime.now(_dt.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    return '%s.bak-%s' % (memory_path, stamp)


def migrated_y2(y2_old: float, rise_old_m: float) -> float:
    """The rise-aware apex (m) an OLD ``y2_apex_m`` reading implies.

    Recovers the (descending) catch-plane crossing speed -- the exact
    inverse of ``schedule.apex_from_vz``, ``v_c = -sqrt(2 g y2_old)`` -- and
    re-reads it through ``schedule.apex_from_crossing`` at ``rise_old_m``.
    """
    y2_old = float(y2_old)
    if not y2_old >= 0.0:
        raise ValueError('y2_apex_m must be >= 0, got %r' % (y2_old,))
    vz_m_s = -math.sqrt(2.0 * sch._G_SI * y2_old)
    return sch.apex_from_crossing(vz_m_s, float(rise_old_m))


@dataclasses.dataclass
class MigrationResult:
    """What a (dry-run or real) migration pass found/did."""
    n_rows: int
    n_migrated: int
    #: (old, new) pairs for the first 5 migrated rows, in file order.
    preview: List[tuple]
    #: mean of (new - old) over ALL migrated rows; 0.0 if none migrated.
    mean_shift_m: float
    dry_run: bool
    wrote_backup_path: str = None


def _read_rows(memory_path: str):
    if not os.path.exists(memory_path):
        raise FileNotFoundError('%s does not exist' % (memory_path,))
    with open(memory_path, newline='') as handle:
        reader = csv.reader(handle)
        header = next(reader, None)
        if header is None:
            raise ValueError('%s has no header row' % (memory_path,))
        cols = [str(c).strip() for c in header]
        if cols != _HEADER:
            raise ValueError(
                '%s has header %r, expected %r (migrating the pre-2026-09-18 '
                'flight-time schema, or any other schema, is not this '
                "tool's job)" % (memory_path, cols, _HEADER))
        rows = [list(row) for row in reader]
    return rows


def migrate(memory_path: str, cutoff_epoch: float, rise_old_m: float = -0.030,
            dry_run: bool = False) -> MigrationResult:
    """Migrate ``memory_path`` in place (unless ``dry_run``).

    Raises :class:`AlreadyMigrated` if the sidecar marker already exists --
    checked BEFORE any read, so a refused call touches nothing.
    """
    mpath = marker_path(memory_path)
    if os.path.exists(mpath):
        raise AlreadyMigrated(
            '%s already carries %s -- refusing to migrate a second time '
            '(the inverse step is not idempotent against an already-'
            'migrated value)' % (memory_path, mpath))

    rows = _read_rows(memory_path)
    cutoff_epoch = float(cutoff_epoch)
    rise_old_m = float(rise_old_m)

    new_rows = []
    preview = []
    shifts = []
    n_migrated = 0
    for i, row in enumerate(rows):
        row = list(row)
        t_abs_s = float(row[_T_ABS_COL])
        if t_abs_s < cutoff_epoch:
            y2_old = float(row[_Y2_COL])
            try:
                y2_new = migrated_y2(y2_old, rise_old_m)
            except ValueError as exc:
                # Fail closed (nothing written yet), but name the row.
                raise ValueError('data row %d (t_abs_s=%r, y2_apex_m=%r): %s'
                                 % (i + 1, t_abs_s, y2_old, exc)) from exc
            if len(preview) < 5:
                preview.append((y2_old, y2_new))
            shifts.append(y2_new - y2_old)
            row[_Y2_COL] = _FLOAT_FMT % y2_new
            n_migrated += 1
        new_rows.append(row)

    mean_shift_m = (sum(shifts) / len(shifts)) if shifts else 0.0
    result = MigrationResult(n_rows=len(rows), n_migrated=n_migrated,
                              preview=preview, mean_shift_m=mean_shift_m,
                              dry_run=dry_run)
    if dry_run:
        return result

    backup = _backup_path(memory_path)
    shutil.copy2(memory_path, backup)
    with open(memory_path, 'w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(_HEADER)
        writer.writerows(new_rows)
    with open(mpath, 'w') as handle:
        handle.write(
            'migrated %s UTC, cutoff_epoch=%r, rise_old_m=%r, '
            'n_rows=%d, n_migrated=%d\n'
            % (_dt.datetime.now(_dt.timezone.utc).isoformat(),
               cutoff_epoch, rise_old_m, result.n_rows, result.n_migrated))
    result.wrote_backup_path = backup
    return result


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--memory', required=True,
                         help='path to the memory.csv to migrate')
    parser.add_argument('--cutoff-epoch', required=True, type=float,
                         help='rows with t_abs_s before this ROS-epoch '
                              'seconds value are migrated; rows at or after '
                              'it are left untouched')
    parser.add_argument('--rise-old-m', type=float, default=-0.030,
                         help='the OLD (catch_z - release_z) / 1000 rise '
                              '(m) those rows were recorded under '
                              '(default: -0.030, the pre-2026-10-05 '
                              'catch-830/release-860 geometry)')
    parser.add_argument('--dry-run', action='store_true',
                         help='print what would change; write nothing')
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = _parse_args(argv)
    try:
        result = migrate(args.memory, args.cutoff_epoch,
                          rise_old_m=args.rise_old_m, dry_run=args.dry_run)
    except AlreadyMigrated as exc:
        print('REFUSED: %s' % (exc,), file=sys.stderr)
        return 1
    except (FileNotFoundError, ValueError) as exc:
        print('ERROR: %s' % (exc,), file=sys.stderr)
        return 1

    verb = 'would migrate' if args.dry_run else 'migrated'
    print('%s: %s %d of %d rows (t_abs_s < %r); mean y2_apex_m shift %.6f m'
          % (args.memory, verb, result.n_migrated, result.n_rows,
             args.cutoff_epoch, result.mean_shift_m))
    for y2_old, y2_new in result.preview:
        print('  %.6f -> %.6f' % (y2_old, y2_new))
    if not args.dry_run:
        print('backup written to %s' % (result.wrote_backup_path,))
        print('marker written to %s' % (marker_path(args.memory),))
    return 0


if __name__ == '__main__':
    sys.exit(main())
