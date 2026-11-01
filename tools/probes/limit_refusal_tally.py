#!/usr/bin/env python3
"""Tally ``LIMIT_*`` refusals per goal from launch logs (JB-24 / JB-127).

Reads ``~/.ros/log/<launch>/launch.log`` for each launch directory given and,
per Juggle goal (``New goal accepted`` .. ``finished with state``), counts:

- the goal's end (``done`` or the ``ENDED (<CODE>)`` code) and balls caught/thrown;
- planner refusals (``trajectory_node: install_segment refused``) by row
  (``LIMIT_VEL/ACC/JERK``, ``HAND_LIMIT_*``, ``INFEASIBLE``, ``SPLICE_TOO_LATE``),
  with the BUMP nudge's own declines (``bump declined``) counted apart;
- what skill_node did with each: ``RESEND-REFUSED`` (re-aim after a tracker
  fit refused, the old plan stays installed: harmless), ``CATCH-AIM-FALLBACK``
  (the learned aim refused, the catch dispatched at the nominal aim) and a
  fatal ``END <row> at skill N`` (the attempt is over).

Then per-run totals across all launches given. It answers JB-127's option 4,
"is the LIMIT_* class the dominant miss class?": first run 2026-11-01 over the
2026-10-28, 10-30 and 10-31 Sittings (58 goals: 4 LIMIT-class ends, 3 of them
the JB-120 HAND_LIMIT_ACC cell; 31 of 37 leg LIMIT_* refusals were re-sends).
Logbook: ``logbook/2026-11-01-jb24-limit-refusal-tally.md``. The launch log does
not record the Platform hold per goal: take it from the ``pre/post-release
hold`` line (printed per launch) or the Sitting's logbook.

Usage::

    python3 tools/probes/limit_refusal_tally.py 2026-10-31-13-14-02-123456-jetson-1234 ...
    python3 tools/probes/limit_refusal_tally.py --root ~/.ros/log <dir> ...

Writes the same text to ``temp/probes/limit_refusal_tally/<timestamp>.txt``.
Stdlib only, Python 3.8; read-only on the logs.
"""
from __future__ import annotations

import argparse
import collections
import datetime
import os
import re
import sys

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

_ROW = re.compile(r'\((HAND_LIMIT_\w+|LIMIT_\w+|INFEASIBLE)[:)]')
_REFUSED = re.compile(r'REFUSED (\S+)')
_BUMP_ROW = re.compile(r'peak (leg|hand) (velocity|acceleration|jerk)')
_CHECK = re.compile(r'\[check=([^,\]\s]+)')


def _ts(line):
    m = re.match(r'(\d+\.\d+) ', line)
    return float(m.group(1)) if m else None


def _code(line):
    """The refusal's row; else its ``[check=...]`` tag (e.g. ``transit``)."""
    m = _ROW.search(line) or _REFUSED.search(line) or _CHECK.search(line)
    return m.group(1) if m else '?'


def _bump_row(line):
    m = _BUMP_ROW.search(line) or _CHECK.search(line)
    return m.group(0) if m and m.re is _BUMP_ROW else (m.group(1) if m else '?')


def _new_goal(t):
    c = collections.Counter
    return dict(t=t, label='?', end=None, caught=None, install=c(), bump=c(),
                resend=c(), aim=c(), fatal=None)


def parse_launch(lines):
    """Return ``(hold, goals)`` for one launch.log's lines."""
    hold = None
    for line in lines:
        m = re.search(r'pre/post-release hold ([\d.]+)/', line)
        if m:
            hold = m.group(1)
            break
    goals, cur = [], None
    for line in lines:
        if 'skill_node]: New goal accepted' in line:
            cur = _new_goal(_ts(line))
            goals.append(cur)
            continue
        if cur is None:
            continue
        m = re.search(r'skill_node\]: (.*?) started:', line)
        if m:
            cur['label'] = m.group(1)[:40]
        m = re.search(r'skill_node\]: (\S+(?: \(reload\))?) ENDED \((\w+)\)', line)
        if m:
            cur['label'], cur['end'] = m.group(1), m.group(2)
            c = re.search(r'(\d+)/(\d+) caught', line)
            cur['caught'] = (int(c.group(1)), int(c.group(2))) if c else None
        m = re.search(r'skill_node\]: (\S+(?: \(reload\))?) done: (\d+)/(\d+) caught', line)
        if m:
            cur['label'], cur['end'] = m.group(1), 'done'
            cur['caught'] = (int(m.group(2)), int(m.group(3)))
        if 'skill_node]' in line and 'RESEND-REFUSED' in line:
            cur['resend'][_code(line)] += 1
        if 'skill_node]' in line and 'CATCH-AIM-FALLBACK' in line:
            m = re.search(r'refused (\S+)', line)
            cur['aim'][m.group(1) if m else '?'] += 1
        m = re.search(r'skill_node\]: [\d.]+ END (\S+) at skill \d+ \((\w+)\)', line)
        if m:
            cur['fatal'] = '%s (%s)' % (m.group(1), m.group(2))
        if 'trajectory_node]: install_segment refused' in line:
            if 'bump declined' in line:
                cur['bump'][_bump_row(line)] += 1
            else:
                cur['install'][_code(line)] += 1
        if 'skill_node]' in line and 'finished with state' in line:
            cur = None
    return hold, goals


def report(root, dirs):
    out = []
    tot = dict(goals=0, caught=0, thrown=0, ends=collections.Counter(),
               install=collections.Counter(), bump=collections.Counter(),
               resend=collections.Counter(), aim=collections.Counter(),
               fatal=collections.Counter())
    for d in dirs:
        path = os.path.join(root, d, 'launch.log')
        with open(path, errors='replace') as fh:
            hold, goals = parse_launch(fh.read().split('\n'))
        out.append('## %s  (launch hold %s)' % (d, hold))
        for i, g in enumerate(goals, 1):
            when = (datetime.datetime.fromtimestamp(g['t']).strftime('%H:%M:%S')
                    if g['t'] else '?')
            out.append('G%d %s %s | end=%s caught=%s | refused=%s bump=%s | '
                       'resend=%s aim_fallback=%s fatal=%s'
                       % (i, when, g['label'], g['end'] or 'no-summary',
                          g['caught'], dict(g['install']), dict(g['bump']),
                          dict(g['resend']), dict(g['aim']), g['fatal']))
            tot['goals'] += 1
            tot['ends'][g['end'] or 'no-summary'] += 1
            if g['caught']:
                tot['caught'] += g['caught'][0]
                tot['thrown'] += g['caught'][1]
            for k in ('install', 'bump', 'resend', 'aim'):
                tot[k].update(g[k])
            if g['fatal']:
                tot['fatal'][g['fatal']] += 1
    out.append('## TOTAL over %d launch(es)' % len(dirs))
    out.append('goals %d, caught/thrown %d/%d (summary lines only)'
               % (tot['goals'], tot['caught'], tot['thrown']))
    for k, label in (('ends', 'ends'), ('install', 'planner refusals'),
                     ('resend', 're-send refusals'), ('aim', 'aim fallbacks'),
                     ('fatal', 'fatal ends'), ('bump', 'bump declines')):
        out.append('%s: %s' % (label, dict(tot[k])))
    return '\n'.join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('dirs', nargs='+', help='launch directories under --root')
    ap.add_argument('--root', default=os.path.expanduser('~/.ros/log'))
    args = ap.parse_args(argv)
    text = report(args.root, args.dirs)
    print(text)
    outdir = os.path.join(_REPO, 'temp', 'probes', 'limit_refusal_tally')
    os.makedirs(outdir, exist_ok=True)
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    with open(os.path.join(outdir, stamp + '.txt'), 'w') as fh:
        fh.write(text + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
