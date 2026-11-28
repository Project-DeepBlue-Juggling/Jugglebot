#!/usr/bin/env python3
"""Copy a plant's Learner Memory to a scratch plant, so an experiment starts
from the main Memory's rows and writes only to the copy.

Usage (stdlib only, Python 3.8):

    python tools/memory_scratch.py <scratch_plant_id> [--from jugglebot] [--root DIR]
    python tools/memory_scratch.py --list [--root DIR]

The robot always uses plant ``jugglebot``. An experiment that sets the
``skill_node`` parameter ``plant_id`` to a fresh name starts from an EMPTY
Memory; this tool makes the fresh name start from the main rows instead
(plan ``learner-efficacy`` step E2(c)).

Layout it follows (read from the node, not invented):

* The node opens ``temp/runtime/learn/<plant_id>/memory_v2.csv``:
  ``skill_node._open_memory`` -> ``runtime_paths.live_memory_path(_REPO_ROOT,
  plant_id)`` -> ``runtime_memory_path`` = ``runtime_base(root)/learn/<id>/``.
  ``--root`` replaces that ``learn`` directory (for tests).
* A plant directory holds ``memory_v2.csv`` and ``memory_v2.csv.seed`` (JSON:
  rows / sha256 / tracked path at seeding). The CSV is copied byte for byte.
  The seed is NOT copied: it carries the main plant's tracked sha256, and the
  debrief merge (``tools/runtime_records.py``) resolves the tracked file from
  the plant id, finds none for a scratch plant and refuses ("the tracked file
  was deleted after seeding"). The scratch plant gets an EMPTY seed instead
  (``sha256`` null, ``rows`` 0, the form ``runtime_paths.write_seed`` writes
  for an absent tracked file), so the merge archives every row of the scratch
  copy under ``temp/learn/<scratch>/`` and never touches the main plant. Any
  other file is copied too (a future sidecar is not silently lost), except
  in-flight ``*.tmp`` leftovers.
* The copy is built in a temporary sibling directory and renamed into place,
  so an interrupted copy never leaves a half plant. The source is only read.

The node reads ``plant_id`` at goal start (``skill_node`` goal callbacks), and
nothing enforces an idle node, so set the parameter while no Attempt runs.
Run the copy itself while no Attempt runs too: the node appends rows to the
source file, and a copy taken mid-append could end on a torn row. The root is
``<repo>/temp/runtime/learn``; the ``JUGGLEBOT_RUNTIME_DIR`` override that
``runtime_paths.runtime_base`` honours is NOT read here, so pass ``--root``
if that override is ever set on the robot.
"""
from __future__ import annotations

import argparse
import csv
import datetime
import json
import os
import shutil
import sys
import tempfile
from typing import List, Optional

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# Same location as runtime_paths.runtime_base(root) / 'learn' (RUNTIME_DIR
# 'temp/runtime'); tests/sim/test_memory_scratch.py pins the agreement.
DEFAULT_ROOT = os.path.join(_REPO_ROOT, 'temp', 'runtime', 'learn')
MEMORY_FILE = 'memory_v2.csv'
SEED_SUFFIX = '.seed'             # runtime_paths.SEED_SUFFIX


def check_plant_id(plant_id: str) -> None:
    """Restates ``memory._check_plant_id`` (motion/skills/memory.py:213): one
    non-empty safe path segment, not '.' or '..'. A test scans that source."""
    if (not isinstance(plant_id, str) or not plant_id
            or plant_id in ('.', '..')
            or os.sep in plant_id or (os.altsep and os.altsep in plant_id)
            or '/' in plant_id):
        raise ValueError('plant id must be one safe path segment (non-empty, '
                         'no "/", not "." or ".."), got %r' % (plant_id,))


def count_rows(plant_dir: str) -> int:
    path = os.path.join(plant_dir, MEMORY_FILE)
    if not os.path.isfile(path):
        return 0
    with open(path, newline='') as handle:
        return max(sum(1 for _ in csv.reader(handle)) - 1, 0)


def make_scratch(root: str, src_id: str, dst_id: str) -> int:
    """Copy plant ``src_id`` to ``dst_id`` under ``root``; return row count.
    Raises ValueError (nothing written) on any refusal."""
    check_plant_id(src_id)
    check_plant_id(dst_id)
    if dst_id == src_id:
        raise ValueError('scratch plant equals the source %r' % src_id)
    src, dst = os.path.join(root, src_id), os.path.join(root, dst_id)
    if not os.path.isdir(src):
        raise ValueError('source plant %r does not exist under %s'
                         % (src_id, root))
    if os.path.lexists(dst):
        raise ValueError('scratch plant %r already exists: %s' % (dst_id, dst))
    tmp = tempfile.mkdtemp(prefix='.%s.tmp-' % dst_id, dir=root)
    try:
        for name in sorted(os.listdir(src)):
            if name.endswith(SEED_SUFFIX):  # replaced by an empty seed below (module docstring)
                continue
            if name.endswith('.tmp'):       # an interrupted atomic write
                continue
            shutil.copyfile(os.path.join(src, name), os.path.join(tmp, name))
        if os.path.exists(os.path.join(tmp, MEMORY_FILE)):
            # <root> is <repo>/temp/runtime/learn; the tracked side is <repo>/temp/learn/<plant>/
            tracked = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(root))), 'learn', dst_id, MEMORY_FILE)
            seed = {'rows': 0, 'sha256': None, 'tracked': tracked,
                    'seeded_at': datetime.datetime.now().astimezone().isoformat(timespec='seconds')}
            with open(os.path.join(tmp, MEMORY_FILE + SEED_SUFFIX), 'w') as fh:
                fh.write(json.dumps(seed, indent=2, sort_keys=True) + '\n')
        os.rename(tmp, dst)
    except BaseException:
        shutil.rmtree(tmp, ignore_errors=True)
        raise
    return count_rows(dst)


def list_plants(root: str) -> List[str]:
    if not os.path.isdir(root):
        return []
    return ['%-40s %6d rows' % (n, count_rows(os.path.join(root, n)))
            for n in sorted(os.listdir(root))
            if os.path.isdir(os.path.join(root, n)) and not n.startswith('.')]


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('scratch', nargs='?', help='new scratch plant id')
    ap.add_argument('--from', dest='src', default='jugglebot',
                    help='source plant id (default jugglebot)')
    ap.add_argument('--root', default=DEFAULT_ROOT,
                    help='learn directory (default: the node\'s)')
    ap.add_argument('--list', action='store_true', help='list plants and exit')
    args = ap.parse_args(argv)
    if args.list:
        print('\n'.join(list_plants(args.root)))
        return 0
    if not args.scratch:
        ap.error('scratch_plant_id is required (or use --list)')
    try:
        rows = make_scratch(args.root, args.src, args.scratch)
    except (ValueError, OSError) as exc:
        print('memory_scratch: refused: %s' % exc, file=sys.stderr)
        return 1
    print('source:      %s' % os.path.join(args.root, args.src))
    print('destination: %s' % os.path.join(args.root, args.scratch))
    print('rows copied: %d' % rows)
    print('Next (set plant_id only while no Attempt is running; the node reads it at goal start):')
    print('  before the experiment: ros2 param set /skill_node plant_id %s' % args.scratch)
    print('  after the experiment:  ros2 param set /skill_node plant_id %s' % args.src)
    return 0


if __name__ == '__main__':
    sys.exit(main())
