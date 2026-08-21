#!/usr/bin/env python3
"""The MESSY-CATCH score: does the hand sensor bounce after arrival? — Phase 0d.

WHAT IT DOES
------------
The owner's framing (2026-08-12, ``plans/active/critical-point-ilc.md`` § Command
vector, third catch channel): the hand ball sensor is highly reliable, so a catch
whose reading **bounces between BALL and EMPTY after arrival is a messy catch**.
This probe builds and scores that metric from bagged sittings, in the order the
plan fixes:

1. **The DISTINCT-SAMPLE CADENCE CENSUS, first.** ``/hand_telemetry`` publishes
   at 100 Hz but the sensor is *polled* far slower, so the flip count is bounded
   by the poll rate, not the publish rate. The census re-verifies the toss-build
   2a findings (poll p50 ~= 71 ms against a configured 20 ms; debounce lag ~0 ms
   on the catch edge, ~241 ms on the departure edge) **on these bags** rather
   than assuming them.
2. **The FLIP CENSUS**, swept over candidate settle windows W, so W is chosen
   from where the distribution stabilises rather than picked.
3. **The SCORE, its distribution clean vs messy, a threshold, and its
   false-clean / false-messy rates** on this corpus — plus the pre-registered
   outcome, (i) or (ii), that the evidence selects.

THE ONE TRAP, NAMED UP FRONT
-----------------------------
The corpus's ground truth for "messy" is the **quick-drop**: a held segment
shorter than ``QUICK_DROP_S`` (1.5 s), measured on the DEBOUNCED verdict. So
"a debounced transition occurred within W" is, at W >= the quick-drop bound,
*the ground-truth definition restated* — it will separate perfectly and the
separation means nothing. The non-tautological channel is the **raw** bit: a
raw flip need not end possession (the firmware debounce restores on any single
HELD reply), so ``ball_held_raw`` can register a rattle the possession label
never sees. Both are reported; the score is built on the raw count and the
debounced count rides alongside as the possession-loss indicator. See
:func:`messy_score`.

WHAT IT IMPORTS RATHER THAN RE-IMPLEMENTS (plan design constraint 1)
--------------------------------------------------------------------
The bag reader, the ledger, the edge rule and the per-toss LABEL come from
``tools/probes/toss_record_miner.py`` and the installed ``jugglebot.toss_record``
it imports. ``edges()`` is the authority for what a transition IS — the same
debounced-over-valid rule ``label_from_sensor`` and ``HandBallSensorSource``
apply — so this probe and the robot cannot drift onto different definitions of
a flip.

OUTPUTS
-------
``temp/probes/hand_sensor_settle_<stamp>.json`` with ``--json``. Bags are
processed **sequentially** (8 GB box).

USAGE
-----
    python tools/probes/hand_sensor_settle.py --bag 2026-08-10_16-30-44
    python tools/probes/hand_sensor_settle.py --all --match 2026-08-12 --json
    python tools/probes/hand_sensor_settle.py --self-check

Exit 0 = every requested bag analysed; 1 = a bag was unreadable or the
self-check failed.
"""

from __future__ import annotations

import argparse
import bisect
import glob
import json
import os
import sys
from datetime import datetime

_HERE = os.path.dirname(os.path.abspath(__file__))          # tools/probes
_REPO = os.path.dirname(os.path.dirname(_HERE))             # repo root
for _p in (_HERE, _REPO, os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import jugglebot.hardware_config as hw                          # noqa: E402
import toss_record_miner as miner                                # noqa: E402
from jugglebot.toss_record import edges                          # noqa: E402

DEFAULT_ROOT = miner.DEFAULT_ROOT
OUT_DIR = os.path.join(_REPO, 'temp', 'probes')

#: The candidate settle windows, in seconds. Swept, not chosen: the plan asks W
#: to be fixed from where the flip distribution stabilises.
W_SWEEP = (0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0)

#: The W the metric SHIPS at — derived from the reference-bag plateau + 3/3
#: quick-drop recall, 2026-08-12. Every number the plan and the logbook quote for
#: this score (recall, false-clean, false-messy) is measured HERE, so it is also
#: what :func:`messy_score` and ``--window-s`` default to.
SHIPPED_W_S = 0.75

#: What :func:`analyse` / :func:`pool` fall back to when the corpus cannot size W
#: at all (:func:`choose_window` returns ``None`` — no ground-truth messy catch,
#: which is most bags in this corpus). It is DELIBERATELY the shipped window and
#: not a second value: it was 1.0 s, so a bag with no messy catch was scored at a
#: window the docs never describe and the report said "W chosen: 1.0" underneath a
#: metric defined at 0.75. A fallback that differs from the shipped default is a
#: silent second definition of the metric.
FALLBACK_W_S = SHIPPED_W_S

#: Ground truth for "messy" on this corpus: a held segment shorter than this is
#: a quick-drop. The miner's own constant, imported rather than restated.
QUICK_DROP_S = miner.QUICK_DROP_S

#: The score threshold. A catch scoring >= this is called MESSY.
MESSY_THRESHOLD = 1


# ── The cadence census (question 1) ───────────────────────────────────────────

def poll_cadence(samples):
    """Distinct-SAMPLE cadence of the sensor itself, from ``ball_held_stamp``.

    The number that bounds everything downstream: a flip cannot be observed
    faster than the bridge polls the switch. ``/hand_telemetry`` republishes the
    cached bit at 100 Hz, so counting *messages* would report 100 Hz and be
    wrong by the whole factor this census exists to expose.

    Returns the percentile spread, not just the median the miner's ledger
    carries, because the 2a finding was specifically that the distribution is
    *spread* (p5 32 / p50 71 / p95 111 ms) rather than quantised at a different
    fixed rate — the signature of a poll being deferred, not re-rated.

    A **backwards** ``ball_held_stamp`` step is dropped and counted, never
    averaged in. The stamp is bridge-sourced and a re-anchor (or a wrap) makes it
    go down; a negative sample would drag the percentiles toward zero and — via
    ``select_outcome``'s "a W-second window holds N polls" line, which DIVIDES by
    p50 — could report a negative or absurd poll count. ``n_backwards`` is
    reported rather than swallowed, because a stream that steps backwards at all
    is a finding about the bridge clock.
    """
    steps = []
    backwards = 0
    prev = None
    for s in samples:
        if not s.valid:
            continue
        stamp = float(s.stamp)
        if prev is not None and stamp != prev:
            if stamp < prev:
                backwards += 1
            else:
                steps.append((stamp - prev) * 1e3)
        prev = stamp
    steps.sort()
    if not steps:
        return {'n': 0, 'n_backwards': backwards,
                'configured_ms': float(hw.JB_BD_CHECK_INTERVAL_MS)}

    def p(q):
        return steps[min(len(steps) - 1, int(q * len(steps)))]

    return {'n': len(steps), 'n_backwards': backwards,
            'p5_ms': p(.05), 'p50_ms': p(.5), 'p95_ms': p(.95),
            'max_ms': steps[-1],
            'configured_ms': float(hw.JB_BD_CHECK_INTERVAL_MS)}


#: A raw edge whose debounced confirmation has not arrived within this long was
#: never confirmed at all. Without the bound the matcher pairs an unconfirmed
#: flicker with the NEXT catch's debounced edge — minutes later — and reports a
#: 41-second "debounce lag", which is how a census invents a fact.
MAX_DEBOUNCE_LAG_S = 1.0


def debounce_lag(samples, max_lag_s=None):
    """Measured debounce lag on each edge direction, in ms.

    Re-verifies the 2a asymmetry (~0 ms on the catch edge, ~241 ms on the
    departure edge) rather than assuming it. Each RAW edge is matched to the
    first DEBOUNCED edge of the same direction at or after it, **within
    ``max_lag_s``**; a raw edge the debounce never confirmed is counted as
    ``n_unconfirmed`` instead of contributing a spurious lag. That population is
    not noise to be discarded — it is exactly the raw-flicker population the
    messy score is built on, so it is reported.

    ``max_lag_s`` defaults through a ``None`` sentinel for the same reason
    :func:`catches_with_truth`'s does: a default bound at ``def`` time is
    invisible to ``monkeypatch.setattr``, so a mutation guard aimed at
    :data:`MAX_DEBOUNCE_LAG_S` would pass while asserting nothing.
    """
    max_lag_s = MAX_DEBOUNCE_LAG_S if max_lag_s is None else max_lag_s
    r_rise, r_fall = edges(samples, debounced=False)
    d_rise, d_fall = edges(samples, debounced=True)
    out = {}
    for name, raw, deb in (('rise_ms', r_rise, d_rise),
                           ('fall_ms', r_fall, d_fall)):
        lags = []
        unconfirmed = 0
        deb = sorted(deb)
        for t in raw:
            i = bisect.bisect_left(deb, t)
            if i < len(deb) and (deb[i] - t) <= max_lag_s:
                lags.append((deb[i] - t) * 1e3)
            else:
                unconfirmed += 1
        lags.sort()
        out[name] = ({'n': 0, 'n_unconfirmed': unconfirmed} if not lags else
                     {'n': len(lags), 'min': lags[0],
                      'median': lags[len(lags) // 2], 'max': lags[-1],
                      'n_unconfirmed': unconfirmed})
    return out


# ── The flip census (question 2) ──────────────────────────────────────────────

def edge_times(samples, debounced):
    """-> every transition instant on one bit, ascending. The ONE definition.

    Both directions merged, straight from :func:`jugglebot.toss_record.edges`,
    which is the labelling authority: invalid samples are SKIPPED rather than
    treated as a level, because letting UNKNOWN act as EMPTY would mint a flip
    out of a telemetry hiccup and inflate exactly the score this probe defines.
    """
    rise, fall = edges(samples, debounced=debounced)
    return sorted(list(rise) + list(fall))


def flips_in(edges_asc, lo, hi):
    """Transitions inside ``(lo, hi]``, counted off a precomputed edge list.

    **AUDIT FIX 2026-08-12, and it was a FALSE-CLEAN bug, not an optimisation.**
    This used to rescan ``samples[a:b]`` and seed ``prev`` from the first sample
    *inside* the window, which makes the transition between the last pre-``lo``
    sample and the first in-window sample invisible. The window opens at the
    catch instant, so the invisible transition is the one on the **first
    post-arrival poll** — a ball that seats and reads EMPTY on the very next
    poll, i.e. the shortest possible quick-drop, scored **0** and was called
    CLEAN. The miss direction was therefore the one direction the pre-registered
    outcome rule cannot tolerate (``false_clean_rate == 0`` is half of (i)).

    Counting off the authority's own edge list removes the seeding question
    entirely — an edge either is or is not in ``(lo, hi]`` — and drops the cost
    from O(catches x windows x n) to two bisects.
    """
    return (bisect.bisect_right(edges_asc, hi)
            - bisect.bisect_right(edges_asc, lo))


def catches_with_truth(samples, quick_drop_s=None):
    """-> ``[{...}, ...]``, one row per catch, carrying the ground-truth label.

    The population is every DEBOUNCED empty->held edge — the ledger's own
    ``n_catches``, which is the count the reference bag's committed fixture pins
    at 38. A catch is ground-truth MESSY when the debounced possession that
    follows it is shorter than ``quick_drop_s``.

    ``quick_drop_s`` defaults through a ``None`` sentinel rather than binding
    :data:`QUICK_DROP_S` at definition time, so the constant is resolved on every
    call. Not a style point: a default bound at ``def`` time is invisible to
    ``monkeypatch.setattr``, which silently defeated the mutation guard in
    ``tests/ros/test_ilc_measurement_probes.py`` — the guard passed while
    asserting nothing.
    """
    quick_drop_s = QUICK_DROP_S if quick_drop_s is None else quick_drop_s
    rises, falls = edges(samples, debounced=True)
    falls = sorted(falls)
    last_t = max((s.t for s in samples if s.valid), default=None)
    out = []
    for t in rises:
        i = bisect.bisect_right(falls, t)
        if i < len(falls):
            possession = falls[i] - t
            closed = True
        else:
            possession = (last_t - t) if last_t is not None else float('inf')
            closed = False
        out.append({
            't_catch': t,
            'possession_s': possession,
            'possession_closed': closed,
            # An OPEN segment (the bag ended while the ball was still in the cup)
            # cannot be a quick-drop: "we stopped looking" is not "it fell out".
            'truth_messy': bool(closed and possession < quick_drop_s),
        })
    return out


def windows_for(w=None):
    """-> the windows to sweep, ALWAYS including an explicitly forced ``w``.

    ``--window-s 0.9`` used to raise a bare ``KeyError(0.9)`` from inside
    :func:`classify` — a value not in :data:`W_SWEEP` was never swept, so the
    lookup missed — and ``main`` catches only ``(IOError, OSError)``, so the
    traceback escaped. Forcing a free-form W is a legitimate thing to ask of a
    probe whose whole point is that W is a choice, so the forced value joins the
    sweep instead of being refused.
    """
    return W_SWEEP if w is None else tuple(sorted(set(W_SWEEP) | {float(w)}))


def sweep(samples, catches, windows=W_SWEEP):
    """-> the raw/debounced flip counts for every catch at every candidate W.

    The two edge lists are built ONCE per bag per bit and every window is two
    bisects against them — see :func:`flips_in` for why counting off the edge
    list rather than rescanning the samples is a correctness fix.
    """
    raw_edges = edge_times(samples, False)
    deb_edges = edge_times(samples, True)
    for row in catches:
        t = row['t_catch']
        row['raw_flips'] = {w: flips_in(raw_edges, t, t + w) for w in windows}
        row['deb_flips'] = {w: flips_in(deb_edges, t, t + w) for w in windows}
    return catches


def choose_window(catches, windows=W_SWEEP):
    """-> ``(W, rationale)``: the smallest W on the plateau of the MESSY
    population's raw-flip distribution.

    "Stabilises" made operational: walking W upward, the messy population's flip
    counts rise as each event is captured, then hold, then rise again as the
    window starts reaching into the NEXT cycle's activity. The plateau is the
    longest run of consecutive W with an identical messy-median AND an identical
    clean-median; the chosen W is its low end, because a shorter window sees less
    unrelated activity and settles sooner after the catch.

    With no messy catches in the corpus the plateau is undefined and the function
    says so rather than inventing one — a corpus of only clean catches cannot
    size a discriminator.
    """
    messy = [c for c in catches if c['truth_messy']]
    clean = [c for c in catches if not c['truth_messy']]
    if not messy:
        return None, ('no ground-truth messy catches in this corpus — W cannot '
                      'be sized here')

    def med(pop, w):
        xs = sorted(c['raw_flips'][w] for c in pop)
        return xs[len(xs) // 2] if xs else 0

    sig = [(med(messy, w), med(clean, w)) for w in windows]
    best_lo = best_len = 0
    i = 0
    while i < len(sig):
        j = i
        while j + 1 < len(sig) and sig[j + 1] == sig[i]:
            j += 1
        if (j - i + 1) > best_len:
            best_lo, best_len = i, j - i + 1
        i = j + 1
    w = windows[best_lo]
    rec = recall_at(catches, w)
    return w, ('plateau over W={}..{} s (messy median {} raw flips, clean median '
               '{}); the low end is chosen, and it recalls {}/{} messy'.format(
                   windows[best_lo], windows[best_lo + best_len - 1],
                   sig[best_lo][0], sig[best_lo][1], rec[0], rec[1]))


def recall_at(catches, w, threshold=None):
    """-> ``(n_recalled, n_messy)`` at window ``w``.

    Reported per candidate W in the sweep table, because it is the property that
    actually decides whether a shorter W is admissible: the plateau says the
    distribution has settled, recall says the window is long enough to have seen
    every event at all.
    """
    threshold = MESSY_THRESHOLD if threshold is None else threshold
    messy = [c for c in catches if c['truth_messy']]
    return (sum(1 for c in messy if c['raw_flips'][w] >= threshold), len(messy))


# ── The score (question 3) ────────────────────────────────────────────────────

def messy_score(row, w=None):
    """THE messy-catch score: **raw** ``ball_held_raw`` flips within W of arrival.

    Raw, not debounced, and the argument is from the data rather than taste:

    * The debounced count at W >= 1.5 s is the ground-truth *definition*
      restated (a debounced fall inside W IS "possession ended inside W"), so it
      separates perfectly and proves nothing. It is reported as
      ``possession_lost`` because it is a genuinely useful field — just not a
      discriminator that was ever tested.
    * The raw count is not implied by the label. On this corpus it finds two
      catches that the possession ground truth calls CLEAN and that flipped the
      raw bit twice before settling — a ball that rattled in the cup and stayed.
      That is precisely the event the owner's framing describes as messy and no
      possession-based label can see, which is the whole reason for a graded
      score instead of a re-derived boolean.

    A raw flip is also the *earliest* observable: the catch edge is debounce-free
    (~0 ms) but the departure edge lags ~241 ms, so a bounce shows on the raw bit
    up to a quarter second before the debounced verdict admits it.
    """
    return int(row['raw_flips'][SHIPPED_W_S if w is None else w])


def classify(catches, w=None, threshold=None):
    """-> ``(rows, confusion)``: the score, the call, and the error rates.

    Both error rates are reported against the corpus's *possession* ground truth,
    and the report says plainly what that ground truth can and cannot adjudicate:
    a raw-flicker catch that never lost possession is scored MESSY here and
    labelled clean there, and the corpus contains no independent evidence
    (operator note, video, mocap) to settle which is right.
    """
    w = SHIPPED_W_S if w is None else w
    threshold = MESSY_THRESHOLD if threshold is None else threshold
    tp = fp = tn = fn = 0
    rows = []
    for c in catches:
        score = messy_score(c, w)
        called = score >= threshold
        rows.append({'t_catch': c['t_catch'],
                     'possession_s': c['possession_s'],
                     'possession_closed': c['possession_closed'],
                     'truth_messy': c['truth_messy'],
                     'score': score,
                     'possession_lost': int(c['deb_flips'][w]),
                     'called_messy': called,
                     # The full sweep rides along so a POOLED verdict can
                     # re-score every catch at ONE window. Without it the pool
                     # mixes bags scored at different W (each bag picks its own
                     # plateau) and then labels the mixture with a single W —
                     # a confusion matrix that is not a confusion matrix.
                     'raw_flips': dict(c['raw_flips']),
                     'deb_flips': dict(c['deb_flips'])})
        if c['truth_messy'] and called:
            tp += 1
        elif c['truth_messy'] and not called:
            fn += 1
        elif called:
            fp += 1
        else:
            tn += 1
    n_messy, n_clean = tp + fn, tn + fp
    return rows, {
        'w_s': w, 'threshold': threshold,
        'n': len(catches), 'n_truth_messy': n_messy, 'n_truth_clean': n_clean,
        'true_messy': tp, 'false_clean': fn, 'false_messy': fp, 'true_clean': tn,
        'false_clean_rate': (float(fn) / n_messy) if n_messy else None,
        'false_messy_rate': (float(fp) / n_clean) if n_clean else None,
    }


# ── The pre-registered outcome ────────────────────────────────────────────────

OUTCOME_I = ('i', 'DISCRIMINATES — the score joins the mined record fields and '
                  'the Phase-3 criteria')
OUTCOME_II = ('ii', 'DOES NOT DISCRIMINATE — the score stays a diagnostic, and '
                    'a faster sensor poll (firmware change) is the follow-on')


def select_outcome(confusion, cadence):
    """-> ``(code, headline, reasons)``. Plan § Phase 0d, pre-registered.

    (i) requires the score to catch **every** ground-truth messy catch
    (``false_clean_rate == 0``) at a false-messy rate the admission gate can
    live with (<= 10 %, i.e. it may not throw away more than one clean record in
    ten). Anything else is (ii).

    Recall is the hard half deliberately: the score's plan roles are a Phase-3
    regression criterion and an admission grade, and a *missed* messy catch
    silently contaminates the corpus, while a false messy one only costs a
    record. Asymmetric consequences, asymmetric criterion.

    A corpus with no ground-truth messy catches cannot select either outcome, and
    the function refuses instead of defaulting.
    """
    reasons = []
    if not confusion['n_truth_messy']:
        return None, ('UNDECIDABLE on this corpus — no ground-truth messy '
                      'catches to discriminate'), [
            '{} catches, all ground-truth clean'.format(confusion['n'])]
    reasons.append(
        'recall {}/{} messy caught (false-clean rate {:.0%})'.format(
            confusion['true_messy'], confusion['n_truth_messy'],
            confusion['false_clean_rate']))
    reasons.append(
        'false-messy {}/{} clean (rate {:.1%})'.format(
            confusion['false_messy'], confusion['n_truth_clean'],
            confusion['false_messy_rate'] or 0.0))
    if cadence.get('n'):
        reasons.append(
            'poll cadence p50 {:.0f} ms (configured {:.0f}) — a {:.2f} s window '
            'holds ~{:.0f} polls'.format(
                cadence['p50_ms'], cadence['configured_ms'], confusion['w_s'],
                confusion['w_s'] * 1e3 / cadence['p50_ms']))
    ok = (confusion['false_clean_rate'] == 0.0
          and (confusion['false_messy_rate'] or 0.0) <= 0.10)
    code, headline = OUTCOME_I if ok else OUTCOME_II
    return code, headline, reasons


# ── Per-bag analysis ──────────────────────────────────────────────────────────

def analyse(samples, rows=None, w=None):
    """-> the evidence dict for one bag. PURE — no bag reader, no ROS.

    ``rows`` are the miner's per-toss records, used only to attach the miner's
    LABEL to each catch where an announcement exists. Most 2026-08-12 sittings
    carry no ``/throw_announcements`` at all, so the catch population is anchored
    on the sensor edges (which every bag has) and the label is an annotation, not
    the anchor.

    ``w`` is tested against ``None``, never for truthiness: ``--window-s 0`` is a
    degenerate but *stated* choice, and ``w or chosen`` would silently discard it
    and report a window the operator did not ask for.
    """
    windows = windows_for(w)
    catches = sweep(samples, catches_with_truth(samples), windows)
    chosen, why = choose_window(catches, windows)
    if w is None:
        w = chosen if chosen is not None else FALLBACK_W_S
    scored, confusion = classify(catches, w)
    if rows:
        # Sorted on the INSTANT alone: `label` is None on a row the miner could
        # not label, and a tuple sort reaches the second element the moment two
        # instants tie — TypeError, str vs None, on the whole bag.
        by_t = sorted(((r['t_catch_deb_ros'], r.get('label'))
                       for r in rows if r.get('t_catch_deb_ros')),
                      key=lambda r: r[0])
        for s in scored:
            hit = [lab for t, lab in by_t if abs(t - s['t_catch']) < 0.05]
            s['miner_label'] = hit[0] if hit else None
    cad = poll_cadence(samples)
    code, headline, reasons = select_outcome(confusion, cad)
    n = len(samples)
    n_valid = sum(1 for s in samples if s.valid)
    return {
        'sensor': {'n_samples': n, 'n_valid': n_valid,
                   'valid_frac': (float(n_valid) / n) if n else 0.0},
        'poll_cadence': cad,
        'debounce_lag': debounce_lag(samples),
        'w_sweep': {str(ws): {
            'raw_messy': sorted(c['raw_flips'][ws] for c in catches
                                if c['truth_messy']),
            'raw_clean': sorted(c['raw_flips'][ws] for c in catches
                                if not c['truth_messy']),
            'deb_messy': sorted(c['deb_flips'][ws] for c in catches
                                if c['truth_messy']),
            'deb_clean': sorted(c['deb_flips'][ws] for c in catches
                                if not c['truth_messy']),
            'recall': list(recall_at(catches, ws)),
            'false_messy': sum(1 for c in catches if not c['truth_messy']
                               and c['raw_flips'][ws] >= MESSY_THRESHOLD),
        } for ws in windows},
        'w_chosen_s': w,
        'w_rationale': why,
        'catches': scored,
        'confusion': confusion,
        'verdict': {'outcome': code, 'headline': headline, 'reasons': reasons},
    }


# ── Reporting ─────────────────────────────────────────────────────────────────

def print_report(name, res):
    print('\n=== {}'.format(name))
    s = res['sensor']
    print('  {} samples, ball_held_valid {}/{} ({:.1%})'.format(
        s['n_samples'], s['n_valid'], s['n_samples'], s['valid_frac']))

    c = res['poll_cadence']
    print('\n  -- 1. DISTINCT-SAMPLE CADENCE CENSUS --')
    if c.get('n'):
        print('  sensor poll (ball_held_stamp advances): n={} p5 {:.0f} / '
              'p50 {:.0f} / p95 {:.0f} / max {:.0f} ms  vs CONFIGURED {:.0f} ms'
              .format(c['n'], c['p5_ms'], c['p50_ms'], c['p95_ms'], c['max_ms'],
                      c['configured_ms']))
    else:
        print('  sensor poll: no ball_held_stamp advance in this bag')
    d = res['debounce_lag']
    for edge, label in (('rise_ms', 'catch edge (empty->held)'),
                        ('fall_ms', 'departure edge (held->empty)')):
        e = d[edge]
        print('  debounce lag, {:28} n={} min {} med {} max {} ms  '
              '({} raw edge(s) NEVER confirmed)'.format(
                  label, e.get('n', 0),
                  '-' if not e.get('n') else '{:.0f}'.format(e['min']),
                  '-' if not e.get('n') else '{:.0f}'.format(e['median']),
                  '-' if not e.get('n') else '{:.0f}'.format(e['max']),
                  e.get('n_unconfirmed', 0)))

    print('\n  -- 2. FLIP CENSUS, swept over W --')
    print('  {:>6} | {:>26} | {:>26} | {:>8} {:>6}'.format(
        'W [s]', 'RAW flips  messy / clean', 'DEB flips  messy / clean',
        'recall', 'false+'))
    # The keys, not W_SWEEP: a forced --window-s joins the sweep, and a table
    # that dropped it would omit the only row the operator asked for.
    for ws in sorted(res['w_sweep'], key=float):
        row = res['w_sweep'][ws]

        def f(xs):
            return '-' if not xs else 'med {} max {}'.format(
                xs[len(xs) // 2], xs[-1])

        print('  {:>6} | {:>12} / {:>11} | {:>12} / {:>11} | {:>8} {:>6}'.format(
            ws, f(row['raw_messy']), f(row['raw_clean']),
            f(row['deb_messy']), f(row['deb_clean']),
            '{}/{}'.format(*row['recall']), row['false_messy']))
    print('  W chosen: {} s — {}'.format(res['w_chosen_s'], res['w_rationale']))

    print('\n  -- 3. SCORE (raw flips within W) --')
    print('  {:>9} {:>11} {:>7} {:>7} {:>9} {:>8}  {}'.format(
        'possess[s]', 'ground truth', 'score', 'poss_lost', 'called',
        'label', 'note'))
    for row in res['catches']:
        if row['score'] == 0 and not row['truth_messy']:
            continue          # the silent majority; the totals below carry them
        print('  {:9.3f} {:>11} {:>7} {:>7} {:>9} {:>8}  {}'.format(
            row['possession_s'], 'MESSY' if row['truth_messy'] else 'clean',
            row['score'], row['possession_lost'],
            'MESSY' if row['called_messy'] else 'clean',
            str(row.get('miner_label') or '-'),
            '' if row['truth_messy'] == row['called_messy']
            else ('<-- FALSE MESSY (raw flicker, possession kept)'
                  if row['called_messy'] else '<-- FALSE CLEAN')))
    cf = res['confusion']
    quiet = sum(1 for r in res['catches']
                if r['score'] == 0 and not r['truth_messy'])
    print('  ({} further catches scored 0 and are ground-truth clean)'.format(
        quiet))
    print('  confusion at W={} s, threshold >= {}: true-messy {} / false-clean '
          '{} / false-messy {} / true-clean {}'.format(
              cf['w_s'], cf['threshold'], cf['true_messy'], cf['false_clean'],
              cf['false_messy'], cf['true_clean']))
    print('  false-clean rate {} | false-messy rate {}'.format(
        '-' if cf['false_clean_rate'] is None
        else '{:.0%}'.format(cf['false_clean_rate']),
        '-' if cf['false_messy_rate'] is None
        else '{:.1%}'.format(cf['false_messy_rate'])))

    v = res['verdict']
    print('\n  -- VERDICT --')
    print('  PRE-REGISTERED OUTCOME ({}): {}'.format(v['outcome'] or '-',
                                                     v['headline']))
    for r in v['reasons']:
        print('    - {}'.format(r))


def pool(results, w=None):
    """Pooled over every bag, on the per-CATCH rows, at ONE window.

    Per-catch, not per-bag: this corpus is one 38-catch sitting plus a tail of
    sittings carrying 0-17 catches each, so any per-bag average is dominated by
    the empty ones.

    **W is re-derived from the POOLED population and every catch is re-scored at
    it.** Each bag picks its own plateau (and a bag with no messy catch cannot
    pick one at all, falling back to the default), so simply concatenating the
    per-bag calls mixes catches scored at 0.75 s with catches scored at 1.0 s and
    then labels the mixture "W = 0.75 s". That is a confusion matrix that is not
    one, and it was the first version of this function.
    """
    catches = []
    for name, r in sorted(results.items()):
        for row in r['catches']:
            row = dict(row)
            row['bag'] = name
            catches.append(row)
    if not catches:
        return None
    # The windows the per-bag pass actually swept — which includes a forced
    # --window-s. Re-deriving them from W_SWEEP would drop it; taking only the
    # first bag's would KeyError on a corpus whose bags were swept differently.
    windows = tuple(sorted(set.intersection(
        *(set(c['raw_flips']) for c in catches))))
    if not windows:
        windows = tuple(sorted(catches[0]['raw_flips']))
    chosen, why = choose_window(catches, windows)
    if w is None:
        w = chosen if chosen is not None else FALLBACK_W_S
    scored, cf = classify(catches, w)
    for src, dst in zip(catches, scored):
        dst['bag'] = src['bag']
    polls = sorted(r['poll_cadence']['p50_ms'] for r in results.values()
                   if r['poll_cadence'].get('n'))
    cad = ({'n': len(polls), 'p50_ms': polls[len(polls) // 2],
            'configured_ms': float(hw.JB_BD_CHECK_INTERVAL_MS)}
           if polls else {'n': 0})
    code, headline, reasons = select_outcome(cf, cad)
    return {'confusion': cf, 'n_bags': len(results),
            'w_chosen_s': w, 'w_rationale': why,
            'per_bag_poll_p50_ms': polls,
            'per_bag_w_chosen_s': {n: r['w_chosen_s']
                                   for n, r in sorted(results.items())},
            'score_distribution': {
                'clean': sorted(c['score'] for c in scored
                                if not c['truth_messy']),
                'messy': sorted(c['score'] for c in scored
                                if c['truth_messy'])},
            'false_messy_rows': [
                {'bag': c['bag'], 'score': c['score'],
                 'possession_s': c['possession_s']}
                for c in scored if c['called_messy'] and not c['truth_messy']],
            'verdict': {'outcome': code, 'headline': headline,
                        'reasons': reasons}}


# ── Self-check (synthetic; the right answer is known by construction) ─────────

def _samples(events, *, dt=0.071, span=(0.0, 120.0), invalid=()):
    """A synthetic sensor stream polled at ``dt`` (the MEASURED 71 ms, not the
    configured 20 ms — a self-check driven at the configured rate would pass
    while the score was unusable at the rate the plant actually runs).

    ``events`` is ``[(t, raw, deb), ...]`` — step changes applied in order, so a
    raw flicker with no debounced response is expressible, which is the whole
    population the score is built on.
    """
    from jugglebot.toss_record import SensorSample
    out = []
    t = span[0]
    raw = deb = False
    ev = sorted(events)
    i = 0
    while t < span[1]:
        while i < len(ev) and ev[i][0] <= t:
            _tt, raw, deb = ev[i]
            i += 1
        valid = not any(lo <= t <= hi for lo, hi in invalid)
        out.append(SensorSample(t=t, held=deb, raw=raw, valid=valid,
                                stamp=t if valid else 0.0))
        t += dt
    return out


def self_check() -> int:
    """Two-sided by construction: every MESSY case has a CLEAN twin.

    The clean twins are the load-bearing half — a score that calls everything
    messy has perfect recall and is useless, and the false-messy rate is the only
    thing that catches it.
    """
    fails = []
    checks = [0]

    def check(name, got, want):
        checks[0] += 1
        if got != want:
            fails.append('{}: got {!r}, want {!r}'.format(name, got, want))

    # A clean catch: raw and debounced rise together at t=20 and stay.
    clean = _samples([(20.0, True, True)])
    rows = sweep(clean, catches_with_truth(clean))
    check('a clean catch has one ground-truth row', len(rows), 1)
    check('a clean catch is ground-truth clean', rows[0]['truth_messy'], False)
    check('a clean catch scores 0', messy_score(rows[0]), 0)

    # A quick-drop: in at 20, out at 20.6 (both bits), i.e. possession 0.6 s.
    quick = _samples([(20.0, True, True), (20.6, False, False)])
    rows = sweep(quick, catches_with_truth(quick))
    check('a 0.6 s possession is ground-truth messy', rows[0]['truth_messy'], True)
    check('a quick-drop scores >= 1 raw flip', messy_score(rows[0]) >= 1, True)

    # THE SHORTEST POSSIBLE QUICK-DROP: seated on one poll, EMPTY on the very
    # next one. Regression guard for a FALSE-CLEAN defect (audit 2026-08-12):
    # the flip census used to seed its `prev` level from the first sample INSIDE
    # the window, so the transition between the last pre-window sample and the
    # first in-window one was invisible — and the window opens at the catch, so
    # the invisible transition was exactly this one. It scored 0 and was called
    # CLEAN, in the one direction the pre-registered outcome rule forbids.
    snap = _samples([(20.0, True, True), (20.03, False, False)])
    rows = sweep(snap, catches_with_truth(snap))
    check('a drop on the FIRST post-arrival poll is ground-truth messy',
          rows[0]['truth_messy'], True)
    check('...and it SCORES messy rather than reading clean',
          messy_score(rows[0]) >= MESSY_THRESHOLD, True)
    check('...at every window in the sweep, not just the long ones',
          min(rows[0]['raw_flips'][w] for w in W_SWEEP) >= MESSY_THRESHOLD,
          True)

    # A RAW FLICKER that keeps possession: the non-tautological case, and the
    # one the debounced count is structurally blind to.
    flick = _samples([(20.0, True, True), (20.3, False, True),
                      (20.5, True, True)])
    rows = sweep(flick, catches_with_truth(flick))
    check('a raw flicker keeps possession (ground-truth clean)',
          rows[0]['truth_messy'], False)
    check('a raw flicker scores 2 raw flips', messy_score(rows[0]), 2)
    check('the DEBOUNCED count is blind to it',
          rows[0]['deb_flips'][SHIPPED_W_S], 0)

    # A flicker OUTSIDE W must not be scored: the window has to bound the score.
    late = _samples([(20.0, True, True), (22.3, False, True),
                     (22.5, True, True)])
    rows = sweep(late, catches_with_truth(late))
    check('a flicker 2.3 s later is outside W=1.0 and scores 0',
          messy_score(rows[0], 1.0), 0)
    check('the same flicker IS inside W=3.0', messy_score(rows[0], 3.0), 2)

    # UNKNOWN never mints a flip (D13 / C-POSSESS-1 § 2).
    blind = _samples([(20.0, True, True)], invalid=((20.2, 20.6),))
    rows = sweep(blind, catches_with_truth(blind))
    check('an invalid span mints no flip', messy_score(rows[0]), 0)

    # Cadence census: the stream is polled at 71 ms and must census as such —
    # NOT at the sample rate, and NOT at the configured 20 ms.
    cad = poll_cadence(clean)
    check('the poll census reports the MEASURED 71 ms, not the configured 20',
          70.0 <= cad['p50_ms'] <= 72.0, True)
    check('the census carries the configured value for comparison',
          cad['configured_ms'], float(hw.JB_BD_CHECK_INTERVAL_MS))
    # A BACKWARDS ball_held_stamp (a bridge re-anchor, or a wrap) is dropped and
    # COUNTED, never averaged in: `select_outcome` divides by p50 to say how many
    # polls a window holds, and a negative sample drags the low percentiles
    # straight through zero.
    back = [s._replace(stamp=s.stamp - 5.0) if i >= 10 else s
            for i, s in enumerate(clean)]
    cad = poll_cadence(back)
    check('a backwards stamp step is counted, not averaged in',
          cad['n_backwards'], 1)
    check('...and no percentile goes negative because of it',
          cad['p5_ms'] > 0.0, True)
    check('...and the median is still the measured poll rate',
          70.0 <= cad['p50_ms'] <= 72.0, True)

    # A miner row with NO label must not take the whole bag down. The annotation
    # join sorted (t, label) TUPLES, which reaches the label the instant two
    # catch instants tie — and `None < str` is a TypeError.
    ann = analyse(clean, rows=[{'t_catch_deb_ros': 20.022, 'label': None},
                               {'t_catch_deb_ros': 20.022, 'label': 'CAUGHT'}])
    check('an unlabelled miner row tying with a labelled one is sortable',
          ann['catches'][0]['miner_label'] in (None, 'CAUGHT'), True)

    # Classification + outcome, both directions.
    mixed = _samples([(20.0, True, True), (20.6, False, False),
                      (40.0, True, True), (80.0, False, False)])
    rows = sweep(mixed, catches_with_truth(mixed))
    _scored, cf = classify(rows, 1.0)
    check('one messy, one clean', (cf['n_truth_messy'], cf['n_truth_clean']),
          (1, 1))
    check('perfect separation selects outcome (i)',
          select_outcome(cf, poll_cadence(mixed))[0], 'i')
    # The REFUSE twin: a threshold that calls every clean catch messy must fail,
    # even though its recall is perfect.
    _scored, bad = classify(rows, 1.0, threshold=0)
    check('a threshold of 0 has perfect recall and still fails',
          select_outcome(bad, poll_cadence(mixed))[0], 'ii')
    check('...because its false-messy rate is 100 %',
          bad['false_messy_rate'], 1.0)
    # And a corpus with no messy catch refuses rather than defaulting.
    _scored, none = classify(sweep(clean, catches_with_truth(clean)), 1.0)
    check('a corpus with no messy catch is UNDECIDABLE',
          select_outcome(none, poll_cadence(clean))[0], None)

    # The POOLED verdict must re-score every catch at ONE window, re-derived on
    # the pooled population. Regression guard for a real defect: two "bags" that
    # each picked a different W were concatenated and the mixture was labelled
    # with the first bag's W.
    bag_a = analyse(_samples([(20.0, True, True), (20.6, False, False),
                              (60.0, True, True), (100.0, False, False)]))
    bag_b = analyse(_samples([(30.0, True, True)]))   # no messy => cannot size W
    check('a corpus with no messy catch falls back to the default W',
          bag_b['w_chosen_s'], FALLBACK_W_S)
    pooled = pool({'a': bag_a, 'b': bag_b})
    check('the pooled window is re-derived, not inherited from a bag',
          pooled['w_chosen_s'], choose_window(
              [c for r in (bag_a, bag_b) for c in r['catches']])[0])
    check('every pooled catch is counted exactly once',
          pooled['confusion']['n'],
          len(bag_a['catches']) + len(bag_b['catches']))

    # A FORCED W. `--window-s 0.9` used to raise a bare KeyError(0.9) out of
    # `classify` — the value was never swept — and `main` catches only
    # (IOError, OSError), so the traceback escaped rather than being reported.
    mix = [(20.0, True, True), (20.6, False, False),
           (40.0, True, True), (80.0, False, False)]
    forced = analyse(_samples(mix), w=0.9)
    check('a W outside the sweep is ANALYSED, not KeyError-ed',
          forced['w_chosen_s'], 0.9)
    check('...and it joins the sweep table rather than vanishing',
          '0.9' in forced['w_sweep'], True)
    check('...and the pooled pass carries it too',
          pool({'f': forced}, 0.9)['w_chosen_s'], 0.9)
    # W = 0 is degenerate but STATED: `w or chosen` treated it as "unset" and
    # silently reported a window nobody asked for.
    check('--window-s 0 is honoured, not read as unset',
          analyse(_samples(mix), w=0.0)['w_chosen_s'], 0.0)

    # An OPEN final segment is not a quick-drop: "we stopped looking" is not
    # "it fell out". Without this a bag ending soon after a catch reports a
    # phantom messy event.
    tail = _samples([(119.0, True, True)])
    rows = catches_with_truth(tail)
    check('a catch the bag ends on is not called a quick-drop',
          rows[0]['truth_messy'], False)
    check('...and is flagged as an unclosed segment',
          rows[0]['possession_closed'], False)

    for f in fails:
        print('FAIL  {}'.format(f))
    print('{}/{} self-check cases pass'.format(checks[0] - len(fails), checks[0]))
    return 1 if fails else 0


# ── CLI ───────────────────────────────────────────────────────────────────────

def analyse_bag(path, robot='jugglebot', w=None):
    """-> ``(result, note)``. Raises ``IOError`` on an unreadable bag."""
    data = miner.read_bag(path, sensor_only=True)
    try:
        rows = miner.mine_bag(data, robot=robot)
    except Exception:                                        # noqa: BLE001
        rows = []
    res = analyse(data.hand, rows, w)
    res['n_announcements'] = len(data.announcements)
    res['n_self_toss_rows'] = len(rows)
    note = None
    s = res['sensor']
    if s['n_valid'] < s['n_samples']:
        note = ('ball_held_valid FALSE on {}/{} samples ({:.1%}) — those spans '
                'are skipped by the edge rule, so both the catch population and '
                'the flip counts are censored there'.format(
                    s['n_samples'] - s['n_valid'], s['n_samples'],
                    1.0 - s['valid_frac']))
    return res, note


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bag', action='append', default=[],
                    help='bag directory name under --root (repeatable)')
    ap.add_argument('--all', action='store_true',
                    help='every bag under --root (sequentially — 8 GB box)')
    ap.add_argument('--match', default='',
                    help='with --all, only bags whose name contains this')
    ap.add_argument('--root', default=DEFAULT_ROOT)
    ap.add_argument('--robot', default='jugglebot')
    ap.add_argument('--window-s', type=float, default=None,
                    help='force the settle window W; a value outside the '
                         'sweep JOINS it (default: the plateau the sweep '
                         'selects, else the shipped {:g})'.format(SHIPPED_W_S))
    ap.add_argument('--json', action='store_true',
                    help='write a timestamped JSON report to temp/probes/')
    ap.add_argument('--self-check', action='store_true')
    args = ap.parse_args(argv)

    if args.self_check:
        return self_check()
    bags = list(args.bag)
    if args.all:
        bags += [os.path.basename(p)
                 for p in sorted(glob.glob(os.path.join(args.root, '*')))
                 if os.path.isdir(p) and args.match in os.path.basename(p)
                 and os.path.basename(p) not in bags]
    if not bags:
        ap.error('need --bag or --all (or --self-check)')

    results, rc = {}, 0
    for name in bags:
        try:
            res, note = analyse_bag(os.path.join(args.root, name), args.robot,
                                    args.window_s)
        except (IOError, OSError) as exc:
            print('ERROR  {}: {}'.format(name, exc))
            rc = 1
            continue
        if note:
            print('\nNOTE   {}: {}'.format(name, note))
        print_report(name, res)
        results[name] = res

    pooled = pool(results, args.window_s)
    if pooled:
        cf = pooled['confusion']
        print('\n\n=== POOLED over {} bag(s), {} catches'.format(
            pooled['n_bags'], cf['n']))
        print('  per-bag poll p50: {} ms  (configured {:.0f})'.format(
            ', '.join('{:.0f}'.format(p) for p in pooled['per_bag_poll_p50_ms']),
            float(hw.JB_BD_CHECK_INTERVAL_MS)))
        print('  W re-derived on the POOLED population: {} s — {}'.format(
            pooled['w_chosen_s'], pooled['w_rationale']))
        if pooled['false_messy_rows']:
            print('  false-messy rows: {}'.format(', '.join(
                '{} (score {}, possession {:.1f} s)'.format(
                    r['bag'], r['score'], r['possession_s'])
                for r in pooled['false_messy_rows'])))
        sd = pooled['score_distribution']
        for k in ('clean', 'messy'):
            hist = {}
            for v in sd[k]:
                hist[v] = hist.get(v, 0) + 1
            print('  score distribution, {:5}: {}'.format(
                k, ', '.join('{} flips x{}'.format(s, n)
                             for s, n in sorted(hist.items())) or '(none)'))
        print('  confusion at W={} s, threshold >= {}: true-messy {} / '
              'false-clean {} / false-messy {} / true-clean {}'.format(
                  cf['w_s'], cf['threshold'], cf['true_messy'],
                  cf['false_clean'], cf['false_messy'], cf['true_clean']))
        print('  false-clean rate {} | false-messy rate {}'.format(
            '-' if cf['false_clean_rate'] is None
            else '{:.0%}'.format(cf['false_clean_rate']),
            '-' if cf['false_messy_rate'] is None
            else '{:.1%}'.format(cf['false_messy_rate'])))
        print('\n  POOLED PRE-REGISTERED OUTCOME ({}): {}'.format(
            pooled['verdict']['outcome'] or '-', pooled['verdict']['headline']))
        for r in pooled['verdict']['reasons']:
            print('    - {}'.format(r))

    if args.json and results:
        os.makedirs(OUT_DIR, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        out = os.path.join(OUT_DIR, 'hand_sensor_settle_{}.json'.format(stamp))
        with open(out, 'w') as fh:
            json.dump({'w_sweep': list(windows_for(args.window_s)),
                       'shipped_w_s': SHIPPED_W_S,
                       'fallback_w_s': FALLBACK_W_S,
                       'quick_drop_s': QUICK_DROP_S,
                       'messy_threshold': MESSY_THRESHOLD,
                       'bags': results, 'pooled': pooled}, fh, indent=1)
        print('\nJSON -> {}'.format(out))
    return rc


if __name__ == '__main__':
    sys.exit(main())
