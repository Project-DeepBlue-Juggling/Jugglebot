"""Fresh-REST census: how many fresh REST installs sit in the 50-75 ms solve band.

At emit lead 1 ``FRESH_REST_REBASE_S`` drops 0.075 -> 0.050 s (derived from the
Hand's resume budget, design ``emit_lead_design.md`` section 5), so a fresh
(non-band) REST whose solve takes 50..75 ms is REBASED (its origin starts up to
125 ms later) where at lead 0 it is not.  The executor prints a solve time only
for a REBASE; the trajectory_node line ``install_segment REST ball n:
splice_k=0, plan P ms`` carries the node's service-entry -> accept time for every
REST install, and splice_k=0 marks a fresh origin (a splice has k > 0).  P is
therefore a PROXY for the executor's ``solve_s`` (it includes the node's queue
and reply, so it can only overstate the solve): the census counts P in
[50, 75) ms as would-be new REBASEs, and reports how many are ALREADY rebased
(``fresh origin REBASED`` skill_node lines) and how many are band RESTs (their
rebase test is the slide, not this threshold).

    python tools/probes/fresh_rest_census.py LOG [LOG ...]  [--out DIR]
    python tools/probes/fresh_rest_census.py temp/logs/launch_*.log

Lines are de-duplicated by their full text (a log copied under temp/logs and
~/.ros/log counts once).  Stdlib only.
"""
from __future__ import annotations

import argparse
import collections
import os
import re
import sys

_PLAN = re.compile(r'\[(?P<t>\d+\.\d+)\] \[trajectory_node\]: install_segment REST '
                   r'ball (?P<b>-?\d+): splice_k=(?P<k>-?\d+), plan '
                   r'(?P<p>[\d.]+) ms')
_SKILL = re.compile(r'\[skill_node\]: (?P<d>\d+\.\d+) REST skill (?P<n>\d+): '
                    r'(?P<msg>fresh origin.*)')


def census(paths):
    seen = set()
    plans, skills = [], []
    for path in paths:
        try:
            fh = open(path, errors='replace')
        except OSError:
            continue
        with fh:
            for ln in fh:
                if 'REST' not in ln:
                    continue
                if ln in seen:
                    continue
                seen.add(ln)
                m = _PLAN.search(ln)
                if m:
                    plans.append((float(m['t']), int(m['k']), float(m['p'])))
                    continue
                m = _SKILL.search(ln)
                if m:
                    skills.append((float(m['d']), m['msg']))
    fresh = [p for p in plans if p[1] == 0]
    rebased = [s for s in skills if 'REBASED' in s[1]]
    band = [s for s in skills if 'band REST' in s[1]]
    return plans, fresh, skills, rebased, band


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('logs', nargs='+')
    a = ap.parse_args(argv)
    plans, fresh, skills, rebased, band = census(a.logs)
    P = [p[2] for p in fresh]
    bins = collections.OrderedDict([('<25', 0), ('25-50', 0), ('50-75', 0),
                                    ('75-100', 0), ('>=100', 0)])
    for p in P:
        k = ('<25' if p < 25 else '25-50' if p < 50 else '50-75' if p < 75
             else '75-100' if p < 100 else '>=100')
        bins[k] += 1
    print('logs: %d  REST install lines: %d  fresh (splice_k=0): %d'
          % (len(a.logs), len(plans), len(fresh)))
    print('plan-ms histogram of fresh REST installs: %s'
          % ', '.join('%s: %d' % kv for kv in bins.items()))
    print('skill_node fresh-origin REST lines: %d (REBASED: %d, band REST: %d)'
          % (len(skills), len(rebased), len(band)))
    if P:
        s = sorted(P)
        print('fresh plan ms: min %.1f p50 %.1f p90 %.1f max %.1f'
              % (s[0], s[len(s) // 2], s[int(.9 * (len(s) - 1))], s[-1]))
    print('would-be NEW REBASEs at lead 1 (proxy, 50 <= P < 75 ms): %d of %d fresh REST'
          % (bins['50-75'], len(fresh)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
