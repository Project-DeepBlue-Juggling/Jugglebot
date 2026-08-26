#!/usr/bin/env python3
"""Toss-cycle loop census reader — is ``NODE_LOOP_PERIOD_S`` still a bound?

``toss_sequencer.LoopPeriodCensus`` (landed in ``f997470``,
``logbook/2026-08-26-toss-loop-period-census.md``) measures every
``_run_toss_cycle`` iteration and reports ten ``loop_*`` fields into each toss
record.  Nothing read them back: that entry's § Outcome says *"No probe-side
reader yet either — the first sitting means reading the JSONL directly."*

This is that reader.  It is **Phase B0 / probe P1** of
``plans/active/toss-pipelined-preamble.md`` § 5.1, and it exists as a committed
probe rather than a one-off because B5 (the loop-cost trim) and B6 (the hardware
ladder) re-run it on later sittings to score the trim — ``tools/probes/README.md``
§ "Why committed probes exist".

**The three questions it answers**, in the census entry's own order:

1. *Is ``NODE_LOOP_PERIOD_S = 0.040`` still a bound?*  ``max`` and the
   distribution of ``loop_period_max_pre_s``, split by preamble class.
2. *Which cost dominates?*  ``loop_obs_max_pre_s`` vs ``loop_body_max_pre_s`` vs
   ``loop_sleep_max_pre_s``, reported both pooled and as a per-cycle argmax vote,
   because the three are disjoint by construction
   (``period = obs + body + sleep``) and a pooled median alone hides which term
   owns the *worst* tick.
3. *How low could a compensated TARGET go?*  ``loop_work_max_pre_s``, which is
   the max of the per-iteration ``obs + body`` **sum** — not the sum of the two
   maxima, which is a different and larger number.

**The split that makes this honest: chained vs moving.**  ``position_code`` is
``ALREADY_THERE`` when POSITIONING took the census-B1 no-op skip and ``OK`` when
it commanded a ``go_to_pose``.  Those are two different loops — the moving one
blocks on a planner round trip — and ``toss_session.min_throw_delay_s`` is
charged at the CHAINED one, so pooling them would report a bound for a ladder
that is not the ladder the operator's cadence is cut from.  Rows whose census is
present but whose ``position_code`` is null (a cycle that terminated before
POSITIONING answered) are reported as ``unknown`` and excluded from the
per-class sizing.

**The corpus needs a provenance filter, and that is a finding, not a nicety.**
``temp/logs/`` is the production record belt AND the directory the toss-cal
capture workflow mines, and ``tests/ros/conftest.py::_isolate_toss_record_sinks``
exists to keep the suite out of it (*"a test-written toss_records_*.jsonl is
byte-indistinguishable from a real session's"*).  It leaks: measured
2026-08-27, **286 of 659** records on disk carry a test ``goal_id``, and **270**
of the 461 records claiming ``CAUGHT``/``MISSED`` have no ``t_release_perf`` —
a cycle that flew always stamps its release.  Five of those are census-bearing,
with the give-away signature ``obs 0.002 / body 0.008 / sleep 0.020`` (the unit
tests' injected census).  :func:`live_only` drops both classes and prints the
count, because a synthetic 0.030 s period pooled into a bound argument is a
bound argument about pytest.

**INSTRUMENT ONLY, downstream of an instrument.**  Nothing here may be wired
into a gate.  ``NODE_LOOP_PERIOD_S`` stays a hand-set, reviewed number and
``tests/ros/test_toss_sequencer.py::test_the_census_never_feeds_a_budget``
inspects the budget functions' compiled identifier sets to keep it that way: *a
bound that tracks its own degradation hides the degradation.*  This probe prints
what a reviewed re-cut **would** be (§ SIZING, ceil-to-the-next-10-ms, the
discipline ``ARRIVAL_BAND_MAX_S`` uses) and stops there.

Usage::

    source ~/Desktop/PDJ_venv/venv/bin/activate
    python tools/probes/toss_loop_census.py                     # every sitting
    python tools/probes/toss_loop_census.py --date 20260826     # one day
    python tools/probes/toss_loop_census.py --csv               # + temp/probes/

Read-only: reads ``temp/logs/toss_records_*.jsonl``, commands nothing, touches
no hardware and no ROS graph.  Writes only under ``temp/probes/`` and only with
``--csv`` / ``--json`` (``tools/probes/README.md``).

Exit code 1 when the corpus says ``NODE_LOOP_PERIOD_S`` is **not** a bound on
the chained population, 0 when it is, 2 when no census-bearing record was found.
That is a report of a measurement, not a gate: the exit code exists so a
follow-up sitting can be scored from a shell script without re-reading prose.
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_HERE, '..', '..'))
# FIRST on sys.path, deliberately — the same reason cadence_rung_check.py gives:
# the shell profile exports a PYTHONPATH pointing at the colcon INSTALL tree,
# and a probe that silently measured the last `colcon build` instead of the
# working tree would be worse than no probe.
sys.path.insert(0, os.path.join(_REPO_ROOT, 'ros_ws', 'src', 'jugglebot'))

from jugglebot.toss_sequencer import (                          # noqa: E402
    CENSUS_FIELD_NAMES, NODE_LOOP_PERIOD_S, NODE_TICK_S)

DEFAULT_LOG_DIR = os.path.join(_REPO_ROOT, 'temp', 'logs')
OUT_DIR = os.path.join(_REPO_ROOT, 'temp', 'probes')

#: The three DISJOINT per-iteration terms, in the order the census docstring
#: decomposes them: ``period = obs + body + sleep``.
TERM_FIELDS = ('loop_obs_max_pre_s', 'loop_body_max_pre_s',
               'loop_sleep_max_pre_s')

#: Distribution fields worth a percentile table. ``loop_n_pre`` /
#: ``loop_n_over_pre`` / ``loop_n_post`` are counts and get their own histograms.
DIST_FIELDS = ('loop_period_max_pre_s', 'loop_period_mean_pre_s',
               'loop_work_max_pre_s') + TERM_FIELDS + (
                   'loop_period_max_post_s',)

#: ``position_code`` -> the preamble class the cycle actually ran.
PREAMBLE_CLASS = {'ALREADY_THERE': 'chained', 'OK': 'moving'}

#: The unit tests' placeholder goal ids
#: (``tests/ros/test_toss_record_publisher.py`` :44 and :507). Named rather than
#: pattern-matched: a heuristic that guessed
#: at "looks fake" would eventually drop a real sitting.
SYNTHETIC_GOAL_IDS = frozenset(('deadbeefcafef00d', '0badc0de0badc0de'))

#: Outcomes that mean a ball actually left the hand. Every one of them stamps
#: ``t_release_perf`` on the real node, so its absence is proof of a synthetic
#: row — the structural half of the provenance filter, which catches leaked
#: records the id list does not name.
FLEW_OUTCOMES = frozenset(('CAUGHT', 'MISSED'))

#: The sizing quantum. ``ARRIVAL_BAND_MAX_S`` was cut this way and the plan's
#: B5 acceptance names the same discipline: a constant is a BOUND, so it is
#: ceiled to the next 10 ms rather than set to the datum it bounds.
SIZING_QUANTUM_S = 0.010


def load_records(log_dir: str, date: str = '') -> list:
    """Every toss record on disk, newest-file-last, with its filename attached.

    The record schema is flat JSONL, one dict per line (``toss_record.FIELDS``);
    the ten ``timing`` fields land at the top level, not nested.
    """
    pattern = os.path.join(log_dir, 'toss_records_{}*.jsonl'.format(date or ''))
    rows = []
    for path in sorted(glob.glob(pattern)):
        name = os.path.basename(path)
        with open(path) as fh:
            for lineno, line in enumerate(fh, 1):
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                except ValueError:
                    print('  ! unparseable line {}:{}'.format(name, lineno),
                          file=sys.stderr)
                    continue
                rec['_file'] = name
                rows.append(rec)
    return rows


def census_bearing(rows) -> list:
    """Rows whose census actually ran.

    ``LoopPeriodCensus.summary`` returns all-``None`` when no complete
    pre-dispatch iteration was seen, and the record's null discipline keeps
    "not measured" distinct from "measured zero" — so the test is on the
    period, never on the presence of the key (every record since ``f997470``
    carries all ten keys, most of them null).
    """
    return [r for r in rows if r.get('loop_period_max_pre_s') is not None]


def is_synthetic(rec) -> bool:
    """A suite-written record, not a sitting's.

    Two independent tests, because either alone is escapable: the named test
    ``goal_id``s, and the structural "claims to have flown but never stamped a
    release instant". See the module docstring for the measured leak.
    """
    if rec.get('goal_id') in SYNTHETIC_GOAL_IDS:
        return True
    return (rec.get('outcome') in FLEW_OUTCOMES
            and rec.get('t_release_perf') is None)


def live_only(rows) -> list:
    return [r for r in rows if not is_synthetic(r)]


def preamble_class(rec) -> str:
    """``chained`` (the B1 skip), ``moving`` (a commanded go_to_pose), or
    ``unknown`` (the cycle died before POSITIONING answered)."""
    return PREAMBLE_CLASS.get(rec.get('position_code'), 'unknown')


def percentiles(values, ps=(0.0, 0.5, 0.9, 0.95, 1.0)) -> list:
    """Nearest-rank percentiles of a sorted copy. No interpolation: every value
    reported is a value the machine actually produced, which is what a bound
    argument needs."""
    ordered = sorted(values)
    n = len(ordered)
    if n == 0:
        return [float('nan')] * len(ps)
    return [ordered[min(n - 1, max(0, int(round(p * (n - 1)))))] for p in ps]


def dominant_term(rec) -> str:
    """The term owning this cycle's worst pre-dispatch iteration.

    An argmax over the three disjoint maxima. They are maxima of *different*
    iterations in general, so this is "which term ran longest anywhere in the
    preamble", not "which term owned the single worst tick" — the census does
    not record the latter and this probe does not pretend it does.
    """
    best, best_v = '', -1.0
    for field in TERM_FIELDS:
        v = rec.get(field)
        if v is not None and float(v) > best_v:
            best, best_v = field.split('_')[1], float(v)
    return best or 'none'


def size_bound(values, quantum_s: float = SIZING_QUANTUM_S) -> float:
    """The reviewed constant this population WOULD ask for: ceil-to-quantum of
    the observed max. Printed, never wired — see the module docstring."""
    if not values:
        return float('nan')
    return math.ceil(max(values) / quantum_s - 1e-9) * quantum_s


def summarise(records, threshold_s: float) -> dict:
    """The whole census read, as data. ``main`` only formats this."""
    classes = {}
    for rec in records:
        classes.setdefault(preamble_class(rec), []).append(rec)
    out = {
        'threshold_s': float(threshold_s),
        'n_records': len(records),
        'classes': {},
        'files': sorted({r['_file'] for r in records}),
    }
    for name, rows in sorted(classes.items()):
        periods = [r['loop_period_max_pre_s'] for r in rows]
        over = [int(r.get('loop_n_over_pre') or 0) for r in rows]
        succeeded = [r for r in rows if r.get('success')]
        entry = {
            'n': len(rows),
            'n_over_threshold': sum(1 for p in periods if p > threshold_s),
            'n_cycles_with_overrun': sum(1 for o in over if o > 0),
            'n_successful': len(succeeded),
            'n_successful_with_overrun': sum(
                1 for r in succeeded if int(r.get('loop_n_over_pre') or 0) > 0),
            'iterations_over': sum(over),
            'iterations_pre': sum(int(r.get('loop_n_pre') or 0) for r in rows),
            'sizing_period_s': size_bound(periods),
            'sizing_work_s': size_bound(
                [r['loop_work_max_pre_s'] for r in rows
                 if r.get('loop_work_max_pre_s') is not None]),
            'dist': {},
            'dominant_votes': {},
            'outcomes': {},
        }
        for field in DIST_FIELDS:
            vals = [r[field] for r in rows if r.get(field) is not None]
            entry['dist'][field] = {
                'n': len(vals),
                'p': percentiles(vals) if vals else [],
            }
        for rec in rows:
            term = dominant_term(rec)
            entry['dominant_votes'][term] = entry['dominant_votes'].get(term, 0) + 1
            outcome = rec.get('outcome') or '?'
            entry['outcomes'][outcome] = entry['outcomes'].get(outcome, 0) + 1
        out['classes'][name] = entry
    return out


def write_csv(records, stamp: str) -> str:
    """One row per census-bearing cycle, for a later cross-sitting comparison."""
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, 'toss_loop_census_{}.csv'.format(stamp))
    cols = (['file', 'session_id', 'cycle_index', 'action', 'outcome',
             'success', 'preamble', 'position_code', 'goal_throw_delay_s',
             'goal_dwell_time_s', 'flight_time_s']
            + list(CENSUS_FIELD_NAMES) + ['dominant_term'])
    with open(path, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for rec in records:
            row = [rec.get('_file'), rec.get('session_id'),
                   rec.get('cycle_index'), rec.get('action'),
                   rec.get('outcome'), rec.get('success'),
                   preamble_class(rec), rec.get('position_code'),
                   rec.get('goal_throw_delay_s'), rec.get('goal_dwell_time_s'),
                   rec.get('flight_time_s')]
            row += [rec.get(f) for f in CENSUS_FIELD_NAMES]
            row.append(dominant_term(rec))
            w.writerow(row)
    return path


def write_json(summary: dict, stamp: str) -> str:
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, 'toss_loop_census_{}.json'.format(stamp))
    with open(path, 'w') as fh:
        json.dump(summary, fh, indent=1, sort_keys=True)
    return path


def _print_class(name: str, entry: dict, threshold_s: float) -> None:
    print()
    print('=== {} preamble — {} cycle(s) '.format(name.upper(), entry['n'])
          + '-' * 28)
    print('  outcomes: {}'.format(', '.join(
        '{} x{}'.format(k, v) for k, v in sorted(entry['outcomes'].items()))))
    head = '  {:<24} {:>4} {:>9} {:>9} {:>9} {:>9} {:>9}'.format(
        'field', 'n', 'min', 'p50', 'p90', 'p95', 'max')
    print(head)
    print('  ' + '-' * (len(head) - 2))
    for field, block in entry['dist'].items():
        if not block['p']:
            print('  {:<24} {:>4} {}'.format(field, 0, '   (never measured)'))
            continue
        print('  {:<24} {:>4} {}'.format(
            field, block['n'],
            ''.join('{:>10.4f}'.format(v) for v in block['p'])))
    print('  dominant term (per-cycle argmax over the three disjoint maxima): '
          + ', '.join('{} x{}'.format(k, v) for k, v in
                      sorted(entry['dominant_votes'].items(),
                             key=lambda kv: -kv[1])))
    print('  cycles whose worst pre-dispatch iteration exceeded {:.3f} s: '
          '{}/{}'.format(threshold_s, entry['n_over_threshold'], entry['n']))
    print('  loop_n_over_pre > 0 on {}/{} cycles, and on {}/{} SUCCESSFUL '
          'cycles'.format(entry['n_cycles_with_overrun'], entry['n'],
                          entry['n_successful_with_overrun'],
                          entry['n_successful']))
    print('  overrunning iterations: {}/{} pre-dispatch iterations'.format(
        entry['iterations_over'], entry['iterations_pre']))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--logs', default=DEFAULT_LOG_DIR,
                    help='directory of toss_records_*.jsonl (default temp/logs)')
    ap.add_argument('--date', default='',
                    help='filename date prefix filter, e.g. 20260826')
    ap.add_argument('--loop-period', type=float, default=NODE_LOOP_PERIOD_S,
                    help='the bound to score against (default: the shipped '
                         'NODE_LOOP_PERIOD_S, imported — never restated)')
    ap.add_argument('--keep-synthetic', action='store_true',
                    help='do NOT drop suite-written records (diagnosing the '
                         'record-belt leak itself)')
    ap.add_argument('--csv', action='store_true',
                    help='write the per-cycle rows to temp/probes/')
    ap.add_argument('--json', action='store_true',
                    help='write the summary to temp/probes/')
    args = ap.parse_args(argv)

    rows = load_records(args.logs, args.date)
    n_synthetic = sum(1 for r in rows if is_synthetic(r))
    if not args.keep_synthetic:
        rows = live_only(rows)
    records = census_bearing(rows)
    print('TOSS LOOP CENSUS — probe P1 of plans/active/toss-pipelined-preamble.md')
    print('corpus {}  |  {} record(s), {} census-bearing'
          .format(os.path.join(args.logs, 'toss_records_{}*.jsonl'
                               .format(args.date or '')),
                  len(rows), len(records)))
    print('provenance: {} suite-written record(s) {}'
          .format(n_synthetic,
                  'KEPT (--keep-synthetic)' if args.keep_synthetic
                  else 'dropped'))
    print('shipped constants: NODE_LOOP_PERIOD_S {:.3f} s, NODE_TICK_S (the '
          'sleep) {:.3f} s  |  scored against {:.3f} s'
          .format(NODE_LOOP_PERIOD_S, NODE_TICK_S, args.loop_period))
    if not records:
        print()
        print('NO CENSUS-BEARING RECORD FOUND. Either the corpus predates '
              'f997470 or no cycle completed a pre-dispatch iteration.')
        return 2

    summary = summarise(records, args.loop_period)
    print('sittings: {}'.format(len(summary['files'])))
    for name in ('chained', 'moving', 'unknown'):
        if name in summary['classes']:
            _print_class(name, summary['classes'][name], args.loop_period)

    chained = summary['classes'].get('chained', {})
    everything = [r['loop_period_max_pre_s'] for r in records]
    print()
    print('=== HEADLINE ' + '-' * 55)
    print('  max(loop_period_max_pre_s), ALL classes   {:.4f} s'
          .format(max(everything)))
    if chained:
        print('  max(loop_period_max_pre_s), CHAINED only  {:.4f} s'
              .format(chained['dist']['loop_period_max_pre_s']['p'][-1]))
        print('  is {:.3f} s a bound on the CHAINED loop?  {}'
              .format(args.loop_period,
                      'YES' if chained['n_over_threshold'] == 0 else
                      'NO — {}/{} cycles exceed it'
                      .format(chained['n_over_threshold'], chained['n'])))
    print()
    print('=== SIZING — what a REVIEWED re-cut would ask for ' + '-' * 18)
    print('  (printed, never wired: the constant stays hand-set, and')
    print('   test_the_census_never_feeds_a_budget makes that structural)')
    for name in ('chained', 'moving'):
        entry = summary['classes'].get(name)
        if not entry:
            continue
        print('  {:<8} period bound {:.3f} s   compensated-TARGET floor '
              '(work) {:.3f} s'.format(name, entry['sizing_period_s'],
                                       entry['sizing_work_s']))

    if args.csv:
        print()
        print('csv  -> {}'.format(write_csv(records, _stamp())))
    if args.json:
        print('json -> {}'.format(write_json(summary, _stamp())))

    if chained and chained['n_over_threshold'] > 0:
        return 1
    return 0


def _stamp() -> str:
    """One timestamp per process, so ``--csv --json`` name the same run."""
    global _STAMP
    if _STAMP is None:
        import datetime
        _STAMP = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    return _STAMP


_STAMP = None


if __name__ == '__main__':
    raise SystemExit(main())
