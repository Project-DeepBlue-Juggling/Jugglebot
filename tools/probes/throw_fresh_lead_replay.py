"""Offline replay of recorded fresh-THROW solve times against candidate THROW
leads (2026-10-16, ``plans/archived/throw-fresh-origin-lead.md``).

A fresh THROW's origin sits ``lead`` after its dispatch and
``executor.install_segment`` refuses it ``ORIGIN_TOO_LATE`` once the solve
(``t_inst - t_now``) reaches ``(lead_knots - WIRE_READ_KNOTS) * dt``.  This
reads that measured solve from skill_node's executor lines in launch logs --
``fresh origin: THROW ... (solve X s of a Y s budget)`` for accepted installs
and ``END ORIGIN_TOO_LATE ... THROW ... the solve took X s`` for refusals --
and reports, per candidate lead, how many would refuse and the worst margin.
No solver is run: the recorded solves are re-used as-is.

Recipe (the 2026-10-16 sizing run, n=62)::

    python tools/probes/throw_fresh_lead_replay.py \\
        ~/.ros/log/2026-10-13-00-18-46-323618-jetson-3638285/launch.log \\
        ~/.ros/log/2026-10-13-23-12-50-197471-jetson-3905591/launch.log \\
        ~/.ros/log/2026-10-14-20-54-20-258010-jetson-4122860/launch.log \\
        ~/.ros/log/2026-10-14-21-27-21-980308-jetson-4138247/launch.log \\
        ~/.ros/log/2026-10-16-10-50-42-798495-jetson-1033523/launch.log

Result: 21-86 ms in the first four, 22-140 ms accepted + one 188 ms refusal
on 2026-10-16; lead 9 refuses 1/62 (-38 ms), 12 refuses 0 (+37 ms), 13
refuses 0 (+62 ms).  Output: stdout, plus ``temp/probes/
throw_fresh_lead_replay/summary.md`` (``--leads`` picks the candidates).
"""
import argparse
import os
import re

DT_S = 0.025          # hw.JB_TRAJ_KNOT_DT_S (40 Hz grid)
WIRE_READ_KNOTS = 3   # schedule.WIRE_READ_KNOTS
OUT_DIR = os.path.join('temp', 'probes', 'throw_fresh_lead_replay')

_OK = re.compile(r'\[skill_node\].*fresh origin: THROW .*\(solve ([0-9.]+) s of a')
_BAD = re.compile(r'\[skill_node\].*END ORIGIN_TOO_LATE.*THROW.*the solve took '
                  r'([0-9.]+) s')


def read_log(path):
    """``(accepted_ms, refused_ms)`` lists from one launch.log."""
    acc, ref = [], []
    with open(path, errors='replace') as fh:
        for line in fh:
            m = _OK.search(line)
            if m:
                acc.append(float(m.group(1)) * 1e3)
            m = _BAD.search(line)
            if m:
                ref.append(float(m.group(1)) * 1e3)
    return acc, ref


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--leads', default='9,12,13',
                    help='candidate leads in knots (comma-separated)')
    args = ap.parse_args()
    lines, pooled = [], []
    for path in args.logs:
        acc, ref = read_log(os.path.expanduser(path))
        pooled += acc + ref
        lines.append('%s: n=%d accepted %s refused %s' % (
            os.path.basename(os.path.dirname(path)), len(acc) + len(ref),
            '%.0f-%.0f ms' % (min(acc), max(acc)) if acc else '-',
            ['%.0f' % r for r in ref]))
    if not pooled:
        raise SystemExit('no fresh-THROW lines found')
    lines.append('pooled n=%d, max %.0f ms' % (len(pooled), max(pooled)))
    for lead in (int(k) for k in args.leads.split(',')):
        budget = (lead - WIRE_READ_KNOTS) * DT_S * 1e3
        over = sum(1 for x in pooled if x >= budget)
        lines.append('lead %2d knots: budget %3.0f ms, refused %d/%d, worst '
                     'margin %+.0f ms' % (lead, budget, over, len(pooled),
                                          budget - max(pooled)))
    print('\n'.join(lines))
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, 'summary.md'), 'w') as fh:
        fh.write('\n'.join('- ' + ln for ln in lines) + '\n')


if __name__ == '__main__':
    main()
