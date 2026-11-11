#!/usr/bin/env python3
"""Write the backfilled "Drops to date" history figure into docs/records.md.

FEAT-47: the live counter counts drops from its ``since`` date on; a history
backfill counted every drop before it. This is the one way to enter that
figure (never by hand): it creates the drops section when the ledger has none
(live 0), otherwise changes only the history fields.

    python tools/set_drops_history.py --history 828 --from 2026-07-23 \
        --through 2026-11-11 [--history-caught N] [--since YYYY-MM-DD] \
        [--path docs/records.md]

``--history-caught`` is the Catches paired with the history drops (the Throws
and Feeds to catch are caught + drops); without it the Throws total and the
percentage stay hidden.

Exit 0 on success, 1 (file untouched) on a refused figure, date or marker.
"""
from __future__ import annotations

import argparse
import importlib.util
import os
import sys
from typing import Optional, Sequence

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_RECORDS = os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot', 'jugglebot',
                        'records.py')


def _load_records():
    """records.py by path (pure Python; importable without the ROS install)."""
    spec = importlib.util.spec_from_file_location('_records_by_path', _RECORDS)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod      # dataclasses needs the module registered
    spec.loader.exec_module(mod)
    return mod


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--history', type=int, required=True,
                    help='drops counted before the live counter')
    ap.add_argument('--history-caught', type=int, default=None,
                    help='catches counted in the same history (optional)')
    ap.add_argument('--from', dest='first', required=True,
                    help='first date the history counts (YYYY-MM-DD)')
    ap.add_argument('--through', required=True,
                    help='last date the history counts (YYYY-MM-DD)')
    ap.add_argument('--since', default=None,
                    help='live "since" date if the section is created '
                    '(default: --through)')
    ap.add_argument('--path', default=os.path.join(_REPO, 'docs', 'records.md'))
    args = ap.parse_args(argv)
    rec = _load_records()
    try:
        model = rec.set_drops_history(args.path, args.history, args.through,
                                      since=args.since, first=args.first,
                                      history_caught=args.history_caught)
    except ValueError as exc:
        print('refused (%s untouched): %s' % (args.path, exc), file=sys.stderr)
        return 1
    d = model.drops
    print('wrote %s: total %d = %d history + %d live (since %s)'
          % (args.path, d.total, d.history, d.live, d.since))
    return 0


if __name__ == '__main__':
    sys.exit(main())
