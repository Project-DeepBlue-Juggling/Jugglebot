#!/usr/bin/env python3
"""Seat-edge decomposition — where does the +183.9 ms actually come from?

The possession verdict for cycle ``k`` is minted at the cup's empty->held edge,
and that edge lands **+87.6 … +554.7 ms past the SCHEDULED landing, median
+183.9 ms** (n = 33 over four post-FW-14 bags,
``logbook/2026-08-24-arrival-band-remeasure.md`` § Measurement).  Under the
pipelined preamble the earliest honest release for cycle ``k+1`` is
``seat_edge(k) + commit_budget_s``, so that bias sets the achieved cadence no
matter how much preamble is moved off the critical path
(``plans/active/toss-pipelined-preamble.md`` § 1.4).

**Phase B0 / probe P2.**  The plan asks for a three-way split — release
execution lateness, flight-time model error, sensor detection lag — because
*"only (b) and part of (a) are correctable; the residual is a hard floor on any
evidence-gated commit, and § 1.4's whole prediction rests on which is which"*
(§ 4).  This is that probe.

**The identity it decomposes**, per cycle, all four instants on the ROS clock::

    total   = t_catch_raw_ros − announce_landing_time_ros          (+183.9 ms)
            = (a) RELEASE  release_time_err_ms
                             the BALLISTIC departure (ascending mocap branch
                             back-cast to the release plane) minus the announced
                             release
            + (b) FLIGHT   flight_time_err_s
                             achieved mocap flight (arc crossing to arc crossing)
                             minus the commanded flight time
            + (c) SEATING  t_catch_raw_ros − t_arrival_fit_bag(→ros)
                             the cup's RAW rise edge minus the ball's ballistic
                             crossing of the cup plane — the detection residual,
                             which is mostly the ball riding the catch stroke
                             down to the seat before the beam breaks

The identity is **exact per row** (the announcement satisfies
``announce_landing = announce_throw + cmd_flight_time`` to <1 µs, verified here),
so the split is an algebraic decomposition of a measured number, not a fit.
Only ``median(a) + median(b) + median(c)`` versus ``median(total)`` carries any
slack, and that slack is what the plan's 10 ms acceptance is about.

**⚠ Two corrections this probe makes to the plan's premises.**

1. ``catch_event_dt_s`` is **not** the +183.9 ms measurand (plan § 4, P2's row).
   +183.9 ms is ``t_catch_raw_ros − announce_landing_time_ros``, MINED from the
   bag on the ROS clock off the RAW bit.  ``catch_event_dt_s_fsm`` is the LIVE
   node's ``perf``-clock twin, taken at the subscriber callback off the
   DEBOUNCED bit.  They differ by the ``/hand_telemetry`` delivery lag, which
   this probe measures on the rows carrying both.
2. The plan's ``"about 52.3 ms of it is the release itself running late"`` reads
   the cup's ``held->empty`` edge as a departure instant.  The mocap back-cast
   says the ball is airborne within a couple of milliseconds of the announced
   release — see § Q-2 in the output.  The ~48 ms (the cup's held->empty edge
   past the announced release; 49 ms measured against the ballistic departure)
   is the ball still occluding the beam on its way out of the cup, i.e. it
   belongs to the SENSOR term, not the RELEASE term, and shifting the commanded
   release by it would inject a real ~48 ms error rather than remove one.

**Corpus.**  ``temp/logs/toss_records_*.jsonl`` cannot answer this at all: it is
the DECLARATION stream (origin ``D``), and every field here except
``flight_time_s`` is origin ``M`` — mined offline.  The probe reads the mined
corpus in ``temp/probes/``, newest file per bag label.  **The four post-FW-14
bags must be mined WITHOUT ``--sensor-only``**, which returns before the mocap
pass; the 2026-08-24 re-measure used ``--sensor-only``, so its own artefacts
carry no arc fit and only the two-way split is reachable from them::

    for b in 2026-08-18_18-42-19 2026-08-20_21-51-39 \\
             2026-08-21_10-11-42 2026-08-23_19-14-54; do
        python tools/probes/toss_record_miner.py --bag $b --jsonl
    done

Usage::

    source ~/Desktop/PDJ_venv/venv/bin/activate
    python tools/probes/seat_edge_decomposition.py            # the four bags
    python tools/probes/seat_edge_decomposition.py --csv      # + temp/probes/
    python tools/probes/seat_edge_decomposition.py --bag 2026-08-23_19-14-54

Read-only: reads mined JSONL, commands nothing, touches no hardware and no ROS
graph.  Writes only under ``temp/probes/`` and only with ``--csv``/``--json``
(``tools/probes/README.md``).

Exit code 0 when the three-way split closes inside
:data:`SPLIT_TOLERANCE_MS`, 1 when it does not, 2 when the corpus cannot support
it (no arc fits — the honest failure the plan asks for by name rather than a
forced split).
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import statistics

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_HERE, '..', '..'))

MINED_DIR = os.path.join(_REPO_ROOT, 'temp', 'probes')
OUT_DIR = MINED_DIR

#: The four post-FW-14 bags the +183.9 ms median was measured over
#: (``logbook/2026-08-24-arrival-band-remeasure.md`` § Measurement, and the
#: pooled row of its per-bag table).
DEFAULT_BAGS = (
    '2026-08-18_18-42-19',
    '2026-08-20_21-51-39',
    '2026-08-21_10-11-42',
    '2026-08-23_19-14-54',
)

#: The published headline this probe must reproduce before it may decompose it.
#: If the pooled median moves, the corpus is not the corpus that was measured.
PUBLISHED_MEDIAN_MS = 183.9
PUBLISHED_N = 33

#: The plan's § 4 acceptance: "a three-way split summing to the measured median
#: within 10 ms".
SPLIT_TOLERANCE_MS = 10.0

#: Only CAUGHT cycles have a seat edge to decompose. A MISSED cycle has no rise
#: edge at all, and a BOUNCED one has a rise edge that is not a seat.
DECOMPOSABLE_LABELS = ('CAUGHT',)


def newest_mined(bag: str, mined_dir: str = MINED_DIR):
    """The most recent WHOLE-ARC ``toss_records_<bag>_<stamp>.jsonl``, or ``None``.

    Newest-wins rather than pooled: successive mines of one bag are re-cuts of
    the same cycles, and pooling them would count every cycle twice. Newest
    WITHIN THE FLAVOR this probe needs: a ``--sensor-only`` mine returns before
    the mocap pass and carries no arc, so if one lands later than the whole-arc
    mine, stamp order alone would silently hand this probe a corpus with
    nothing to decompose (`test_ilc_fit.py` had the mirror-image failure
    2026-08-27). The miner records the flavor in its ``_meta.json`` sidecar;
    an unreadable or absent sidecar admits the file (every pre-marker mine in
    this probe's four bags is whole-arc, and `load` still refuses arc-less rows
    downstream).
    """
    hits = sorted(glob.glob(os.path.join(
        mined_dir, 'toss_records_{}_*.jsonl'.format(bag))))
    for path in reversed(hits):
        meta_path = path[:-len('.jsonl')] + '_meta.json'
        try:
            with open(meta_path) as fh:
                if json.load(fh).get('sensor_only'):
                    continue
        except (OSError, ValueError):
            pass
        return path
    return None


def load(paths) -> list:
    rows = []
    for path in paths:
        with open(path) as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                rec = json.loads(line)
                rec['_file'] = os.path.basename(path)
                rows.append(rec)
    return rows


def bag_to_ros_offset(rec):
    """Seconds to add to a BAG-clock instant to land on the ROS clock.

    ``t_land_bag`` is the ANNOUNCED landing crossed into the bag clock
    (``toss_record.py``: *"not the same number"* as ``t_arrival_fit_bag`` — it
    is a conversion of a declared instant, which is exactly what makes it usable
    as the clock bridge).  Recovering the offset from it rather than re-deriving
    ``log_minus_stamp_s`` keeps this probe out of the miner's internals; the two
    agree to **0.0002 ms** on every row that carries ``release_time_err_ms``,
    which :func:`decompose` checks rather than assumes.
    """
    if rec.get('t_land_bag') is None or rec.get('announce_landing_time_ros') is None:
        return None
    return float(rec['announce_landing_time_ros']) - float(rec['t_land_bag'])


def total_ms(rec):
    """THE measurand: the raw cup rise edge past the ANNOUNCED landing.

    ``t_catch_raw_ros`` is the zero-debounce ``empty->held`` edge; the rise
    debounce is structurally 0 ms (``max_missing_samples`` gates
    ``HELD->EMPTY`` only), so raw and debounced agree here — but the runbook's
    recipe names the raw bit and this reproduces the recipe, not a paraphrase.
    """
    if rec.get('t_catch_raw_ros') is None or rec.get('announce_landing_time_ros') is None:
        return None
    return (float(rec['t_catch_raw_ros'])
            - float(rec['announce_landing_time_ros'])) * 1e3


def departure_edge_ms(rec):
    """The SENSOR's ``held->empty`` edge past the announced release.

    This is the quantity Phase A reported as *"departure minus commanded: median
    +52.3 ms"*, and the plan attributes it to release execution lateness.  It is
    reported here beside the ballistic release error so the two can be compared
    rather than conflated — see § Q-2 in the output.
    """
    if rec.get('t_departure_raw_ros') is None or rec.get('announce_throw_time_ros') is None:
        return None
    return (float(rec['t_departure_raw_ros'])
            - float(rec['announce_throw_time_ros'])) * 1e3


def decompose(rec):
    """The three terms for one cycle, in ms, or ``None`` when the row cannot
    carry the full chain. ``residual_ms`` is the identity's closure error."""
    off = bag_to_ros_offset(rec)
    tot = total_ms(rec)
    if (off is None or tot is None
            or rec.get('t_arrival_fit_bag') is None
            or rec.get('t_release_fit_bag') is None
            or rec.get('cmd_flight_time_s') is None
            or rec.get('announce_throw_time_ros') is None):
        return None
    rel_ros = float(rec['t_release_fit_bag']) + off
    arr_ros = float(rec['t_arrival_fit_bag']) + off
    a = (rel_ros - float(rec['announce_throw_time_ros'])) * 1e3
    b = ((arr_ros - rel_ros) - float(rec['cmd_flight_time_s'])) * 1e3
    c = (float(rec['t_catch_raw_ros']) - arr_ros) * 1e3
    return {
        'release_ms': a, 'flight_ms': b, 'seating_ms': c,
        'total_ms': tot, 'sum_ms': a + b + c, 'residual_ms': (a + b + c) - tot,
        'departure_edge_ms': departure_edge_ms(rec),
        'recorded_release_err_ms': rec.get('release_time_err_ms'),
        'recorded_flight_err_ms': (None if rec.get('flight_time_err_s') is None
                                   else float(rec['flight_time_err_s']) * 1e3),
    }


def _stats(values):
    vals = [v for v in values if v is not None]
    if not vals:
        return None
    ordered = sorted(vals)
    n = len(ordered)
    return {
        'n': n, 'min': ordered[0], 'max': ordered[-1],
        'median': statistics.median(ordered),
        'mean': statistics.mean(ordered),
        'sd': statistics.pstdev(ordered) if n > 1 else 0.0,
    }


def _row(label, s):
    if s is None:
        return '  {:<34} {:>4}   (no rows)'.format(label, 0)
    return ('  {:<34} {:>4} {:>+10.1f} {:>+10.1f} {:>9.1f} {:>+10.1f} {:>+10.1f}'
            .format(label, s['n'], s['median'], s['mean'], s['sd'],
                    s['min'], s['max']))


_HEAD = ('  {:<34} {:>4} {:>10} {:>10} {:>9} {:>10} {:>10}'
         .format('term (ms)', 'n', 'median', 'mean', 'sd', 'min', 'max'))


def write_csv(rows, stamp: str) -> str:
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, 'seat_edge_decomposition_{}.csv'.format(stamp))
    cols = ['file', 'toss_uid', 'label', 'flight_time_s', 'cmd_flight_time_s',
            'release_ms', 'flight_ms', 'seating_ms', 'sum_ms', 'total_ms',
            'residual_ms', 'departure_edge_ms', 'recorded_release_err_ms',
            'recorded_flight_err_ms', 'catch_event_dt_s_fsm']
    with open(path, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for rec, terms in rows:
            w.writerow([rec.get('_file'), rec.get('toss_uid'),
                        rec.get('label'), rec.get('flight_time_s'),
                        rec.get('cmd_flight_time_s')]
                       + [terms.get(k) for k in cols[5:14]]
                       + [rec.get('catch_event_dt_s_fsm')])
    return path


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', action='append', default=[],
                    help='bag label to read the newest mine of (repeatable; '
                         'default: the four post-FW-14 bags)')
    ap.add_argument('--corpus', action='append', default=[],
                    help='explicit mined JSONL path (repeatable; overrides '
                         '--bag)')
    ap.add_argument('--mined-dir', default=MINED_DIR)
    ap.add_argument('--csv', action='store_true',
                    help='write the per-cycle terms to temp/probes/')
    ap.add_argument('--json', action='store_true',
                    help='write the summary to temp/probes/')
    args = ap.parse_args(argv)

    print('SEAT-EDGE DECOMPOSITION — probe P2 of '
          'plans/active/toss-pipelined-preamble.md')
    paths, missing = list(args.corpus), []
    if not paths:
        for bag in (args.bag or list(DEFAULT_BAGS)):
            hit = newest_mined(bag, args.mined_dir)
            (paths if hit else missing).append(hit or bag)
    for path in paths:
        print('  corpus  {}'.format(os.path.basename(path)))
    for bag in missing:
        print('  MISSING {} — mine it: python tools/probes/toss_record_miner.py '
              '--bag {} --jsonl   (NOT --sensor-only)'.format(bag, bag))
    if not paths:
        print('no corpus. Nothing to decompose.')
        return 2

    rows = load(paths)
    caught = [r for r in rows if r.get('label') in DECOMPOSABLE_LABELS
              and total_ms(r) is not None]

    # ── 0. Reproduce the published headline before decomposing it ────────────
    head = _stats([total_ms(r) for r in caught])
    print()
    print('=== 0. THE MEASURAND, REPRODUCED ' + '-' * 43)
    print('  t_catch_raw_ros − announce_landing_time_ros, label CAUGHT')
    print(_HEAD)
    print(_row('total seat edge', head))
    if head is None:
        print('  no CAUGHT row carries a raw rise edge — corpus cannot answer.')
        return 2
    drift = head['median'] - PUBLISHED_MEDIAN_MS
    print('  published: n={} median {:+.1f} ms  |  here: n={} median {:+.1f} ms'
          '  (drift {:+.1f} ms)'.format(PUBLISHED_N, PUBLISHED_MEDIAN_MS,
                                        head['n'], head['median'], drift))

    # ── 1. The live-vs-mined measurand gap ───────────────────────────────────
    gap = _stats([r['catch_event_dt_s_fsm'] * 1e3 - total_ms(r)
                  for r in caught if r.get('catch_event_dt_s_fsm') is not None])
    print()
    print('=== 1. catch_event_dt_s_fsm IS A DIFFERENT MEASURAND ' + '-' * 24)
    print('  the LIVE node\'s perf-clock, subscriber-callback, DEBOUNCED-bit twin')
    print(_HEAD)
    print(_row('catch_event_dt_s_fsm − total', gap))
    print('  (a /hand_telemetry delivery lag. Plan § 4 names catch_event_dt_s '
          'as the\n   +183.9 ms measurand; it is not — see the module '
          'docstring.)')

    # ── 2. The two-way split — every CAUGHT row, no mocap needed ─────────────
    print()
    print('=== 2. THE TWO-WAY SPLIT — sensor only, every CAUGHT row ' + '-' * 20)
    two_a = _stats([departure_edge_ms(r) for r in caught])
    two_bc = _stats([
        (float(r['t_catch_raw_ros']) - float(r['t_departure_raw_ros'])
         - float(r['cmd_flight_time_s'])) * 1e3
        for r in caught if r.get('t_departure_raw_ros') is not None
        and r.get('cmd_flight_time_s') is not None])
    print(_HEAD)
    print(_row('(a\') cup departure edge', two_a))
    print(_row('(b+c) flight err + seating', two_bc))
    print(_row('total', head))
    if two_a and two_bc:
        print('  medians sum to {:+.1f} vs {:+.1f}  (Δ {:+.1f} ms)'
              .format(two_a['median'] + two_bc['median'], head['median'],
                      two_a['median'] + two_bc['median'] - head['median']))

    # ── 3. The three-way split — needs the mocap arc fits ───────────────────
    split = [(r, decompose(r)) for r in caught]
    split = [(r, t) for r, t in split if t is not None]
    print()
    print('=== 3. THE THREE-WAY SPLIT — mocap arc fits ' + '-' * 33)
    print('  {}/{} CAUGHT rows carry the full chain (ascending fit + descending '
          'fit\n  + clock bridge). The rest are refused, not imputed.'
          .format(len(split), len(caught)))
    if not split:
        print()
        print('  ⛔ NO ROW CARRIES BOTH ARC FITS. The three-way split is NOT')
        print('     supported by this corpus. Most likely cause: the bags were')
        print('     mined with --sensor-only, which returns before the mocap')
        print('     pass. Re-mine without it (recipe above) and re-run.')
        return 2

    # Refuse, don't impute: `max()` over an empty generator raises, and every
    # matched row CAN legitimately lack `release_time_err_ms`. Say so instead.
    deltas = [abs(t['recorded_release_err_ms'] - t['release_ms'])
              for _, t in split if t['recorded_release_err_ms'] is not None]
    print('  clock-bridge check vs the recorded release_time_err_ms: '
          + ('max |Δ| = {:.4f} ms'.format(max(deltas)) if deltas else
             'NOT AVAILABLE — no matched row carries release_time_err_ms'))
    resid = max(abs(t['residual_ms']) for _, t in split)
    print('  per-row identity closure: max |a+b+c − total| = {:.4f} ms'
          .format(resid))
    print()
    matched = _stats([t['total_ms'] for _, t in split])
    a = _stats([t['release_ms'] for _, t in split])
    b = _stats([t['flight_ms'] for _, t in split])
    c = _stats([t['seating_ms'] for _, t in split])
    print(_HEAD)
    print(_row('(a) RELEASE   ballistic − cmd', a))
    print(_row('(b) FLIGHT    mocap − cmd ToF', b))
    print(_row('(c) SEATING   raw rise − arc', c))
    print(_row('      total, matched rows', matched))
    print(_row('      (a\') cup departure edge',
               _stats([t['departure_edge_ms'] for _, t in split])))
    print()
    sum_of_medians = a['median'] + b['median'] + c['median']
    d_matched = sum_of_medians - matched['median']
    d_published = sum_of_medians - PUBLISHED_MEDIAN_MS
    print('  median(a)+median(b)+median(c) = {:+.1f} ms'.format(sum_of_medians))
    print('    vs the matched rows\' own median  {:+.1f}   (Δ {:+.1f} ms)'
          .format(matched['median'], d_matched))
    print('    vs the published +{:.1f} ms          (Δ {:+.1f} ms)'
          .format(PUBLISHED_MEDIAN_MS, d_published))
    ok = (abs(d_matched) <= SPLIT_TOLERANCE_MS
          and abs(d_published) <= SPLIT_TOLERANCE_MS)
    print('  ACCEPTANCE (both within {:.0f} ms): {}'
          .format(SPLIT_TOLERANCE_MS, 'PASS' if ok else 'FAIL'))

    # ── 4. What the split means for the plan's Q-2 ──────────────────────────
    print()
    print('=== 4. Q-2 — how much would a release-latency shift correct? ' + '-' * 16)
    print('  Q-2 asks whether populating JB_OP_TOSS_RELEASE_LATENCY_MS with the')
    print('  ~52 ms "release lateness" would take that much off the seat edge.')
    print()
    print('  (a\') the CUP\'s held->empty edge says   {:+.1f} ms late'
          .format(_stats([t['departure_edge_ms'] for _, t in split])['median']))
    print('  (a)  the BALLISTIC back-cast says      {:+.1f} ms late'
          .format(a['median']))
    print()
    print('  Those are the same event measured two ways and they disagree by')
    print('  ~{:.0f} ms. The mocap arc is the one with no cup in it: the ball is'
          .format(abs(_stats([t['departure_edge_ms']
                              for _, t in split])['median'] - a['median'])))
    print('  airborne on schedule. The cup\'s departure edge is late because the')
    print('  ball keeps occluding the beam while it accelerates out of the cup —')
    print('  that latency belongs to term (c)\'s departure-side sibling, NOT to')
    print('  the release. Shifting the commanded release by it would make the')
    print('  ball leave EARLY by that much and buy no seat-edge correction.')
    print()
    print('  The correctable term is (b), the flight-time model, at {:+.1f} ms'
          .format(b['median']))
    print('  median — {:.0f}% of the seat edge. The hard residual for an'
          .format(100.0 * b['median'] / matched['median']))
    print('  evidence-gated commit is (c) at {:+.1f} ms median, {:+.1f}…{:+.1f} '
          'spread.'.format(c['median'], c['min'], c['max']))

    stamp = _stamp()
    if args.csv:
        print()
        print('csv  -> {}'.format(write_csv(split, stamp)))
    if args.json:
        os.makedirs(OUT_DIR, exist_ok=True)
        path = os.path.join(OUT_DIR,
                            'seat_edge_decomposition_{}.json'.format(stamp))
        with open(path, 'w') as fh:
            json.dump({'corpus': [os.path.basename(p) for p in paths],
                       'n_caught': len(caught), 'n_matched': len(split),
                       'total': head, 'matched_total': matched,
                       'release': a, 'flight': b, 'seating': c,
                       'sum_of_medians_ms': sum_of_medians,
                       'acceptance_pass': ok}, fh, indent=1, sort_keys=True)
        print('json -> {}'.format(path))
    return 0 if ok else 1


def _stamp() -> str:
    global _STAMP
    if _STAMP is None:
        import datetime
        _STAMP = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    return _STAMP


_STAMP = None


if __name__ == '__main__':
    raise SystemExit(main())
