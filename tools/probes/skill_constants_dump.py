"""Dump every public scalar constant of the skill stack, or diff two dumps.

Bit-identity receipt for a change that must not move any planner number (the
emit-lead P1a/P1b work, 2026-11-25): run it before the first edit and after
the last, then diff.  Covers ``schedule``, ``executor`` and ``bump`` (every
public ``int/float/str/bool/None`` module attribute).

    python tools/probes/skill_constants_dump.py temp/probes/skill_constants/before.json
    # ... edit ...
    python tools/probes/skill_constants_dump.py temp/probes/skill_constants/after.json
    python tools/probes/skill_constants_dump.py --diff before.json after.json

``--diff`` prints values changed / removed / new names and exits 1 when any
value changed or was removed (new names are reported, not an error).
Stdlib only apart from the project package.
"""
from __future__ import annotations

import argparse
import importlib
import json
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
_MODULES = ('schedule', 'executor', 'bump')


def dump(emit_lead=None) -> dict:
    for p in (os.path.join(_ROOT, 'ros_ws', 'src', 'jugglebot'), _ROOT,
              os.path.join(_ROOT, 'config', 'generated')):
        if p not in sys.path:
            sys.path.insert(0, p)
    if emit_lead is not None:
        # Set BEFORE the first skills import (the constants are import-time).
        import hardware_config as hw_top
        import jugglebot.hardware_config as hw
        hw_top.JB_TRAJ_EMIT_LEAD_KNOTS = hw.JB_TRAJ_EMIT_LEAD_KNOTS = \
            int(emit_lead)
    out = {}
    for m in _MODULES:
        mod = importlib.import_module('jugglebot.motion.skills.' + m)
        for n in dir(mod):
            if n.startswith('_'):
                continue
            v = getattr(mod, n)
            if isinstance(v, (int, float, str, bool, type(None))):
                out['%s.%s' % (m, n)] = v
    return out


def diff(a: dict, b: dict) -> int:
    changed = {k: (a[k], b[k]) for k in a if k in b and a[k] != b[k]}
    removed = sorted(k for k in a if k not in b)
    new = sorted(k for k in b if k not in a)
    print('%d names before, %d after: %d changed, %d removed, %d new'
          % (len(a), len(b), len(changed), len(removed), len(new)))
    for k, (x, y) in sorted(changed.items()):
        print('  CHANGED %s: %r -> %r' % (k, x, y))
    for k in removed:
        print('  REMOVED %s' % k)
    for k in new:
        print('  NEW     %s = %r' % (k, b[k]))
    return 1 if (changed or removed) else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('out', nargs='?', help='JSON file to write')
    ap.add_argument('--emit-lead', type=int, default=None,
                    help='dump as if JB_TRAJ_EMIT_LEAD_KNOTS were this '
                         '(default: the configured lead)')
    ap.add_argument('--diff', nargs=2, metavar=('BEFORE', 'AFTER'))
    a = ap.parse_args(argv)
    if a.diff:
        return diff(*[json.load(open(f)) for f in a.diff])
    if not a.out:
        ap.error('give OUT.json or --diff BEFORE AFTER')
    d = dump(a.emit_lead)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, 'w') as fh:
        json.dump(d, fh, indent=1, sort_keys=True)
    print('%d constants -> %s' % (len(d), a.out))
    return 0


if __name__ == '__main__':
    sys.exit(main())
