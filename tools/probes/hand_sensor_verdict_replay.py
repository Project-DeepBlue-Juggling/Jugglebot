#!/usr/bin/env python3
"""Replay a session's hand ball sensor through the PRODUCTION possession verdict.

WHAT IT DOES
------------
Reads ``/hand_telemetry`` and ``/throw_announcements`` straight out of a rosbag,
feeds every sample into the shipped ``jugglebot.ball_possession.HandBallSensorSource``
— the same object ``reload_coordinator_node`` constructs, with the same
``JB_BD_*`` windows — and prints, per announced throw, the sensor's verdict
resolved with FULL HINDSIGHT (see the timing caveat below):

    CAUGHT    ARRIVAL CONFIRMED and RETENTION not REJECTED
    BOUNCE    ARRIVAL CONFIRMED and RETENTION REJECTED  (it arrived, then left)
    MISSED    ARRIVAL REJECTED  (the window closed with no empty->held edge)
    UNKNOWN   ARRIVAL UNKNOWN   (the sensor could not look — the merge would then
                                 fall back to the tracker, C-POSSESS-1 § 3.2)

**A BOUNCE row is a live CAUGHT.** The probe scores each announcement once its
arrival AND retention windows have both closed; the coordinator answers on the
first confirmed tick, where retention is still ``UNKNOWN`` and § 2 consequence 3
forbids it from vetoing. So the label this probe prints is the verdict the
coordinator would have minted only for CAUGHT / MISSED / UNKNOWN — a BOUNCE is
the reporting residual C-POSSESS-1 § 7 keeps by design (the goal terminates
CAUGHT, and retention binds on the NEXT cycle's precondition and the possession
latch instead). That is exactly why BOUNCE is worth counting offline: it is the
count nobody can read off the live log.

plus the **sensor ledger** for the whole bag: sample count, ``ball_held_valid``
fraction, every debounced transition, held-segment durations, and the
seat-then-leave events shorter than ``--quick-drop-s``.

It is the verdict instrument for the 2026-08-10 window sizing: the ARRIVAL and
RETENTION defaults in ``config/hardware_config.yaml`` are the numbers this probe
measured, and re-running it is how a future change to them is scored.

WHY THIS EXISTS
---------------
``logbook/2026-08-10-sensor-truth-possession.md`` — the phase that made the hand
sensor the PRIMARY possession source and flipped ``toss_require_ball_evidence``
to ``true``. Contract: ``ros_ws/docs/ball_possession_contract.md`` (C-POSSESS-1
§§ 3.2, 3.3). Consuming test: ``tests/ros/test_hand_sensor_replay.py``, which
runs this module's pure half over the committed fixture in
``tools/probes/data/hand_sensor_replay_fixture.json``.

Sibling: ``tools/probes/possession_verdict_bag_check.py``, which does the same
job for the TRACKER half of the merged verdict.

HOW A THROW IS SCORED, AND WHAT THE PROBE DOES **NOT** KNOW
-----------------------------------------------------------
The window is anchored on the announcement's own ``landing_time`` (release + ToF)
— exactly the quantity ``TossSequencer.landing_perf`` gives the node — crossed
onto the bag's ``log_time`` clock by the median ``log_time - header stamp`` offset
over the hand stream. Announcements from BOTH throwers are scored: a BB reload
lands in the same cup, and its catches are part of the same measured band.

Two honest limits:

* **Announcement count is not throw count.** A ``TossContinuous`` session and a
  reload sequence both announce, and an announcement whose throw was aborted
  still appears. So "35 CAUGHT of 81 announcements" is not a catch rate — pair it
  with the per-bag outcome lines before quoting one.
* **The probe replays the SENSOR half only.** The shipped verdict is
  ``merge_possession(sensor, tracker)``; with the sensor UNKNOWN the merge answers
  from the tracker, which this probe does not read. An UNKNOWN row here means
  "the coordinator would have fallen back", not "the coordinator refused".
  ``--merge-tracker {refuse,confirm}`` prints that counterfactual column so the
  restored-verdict count is visible: on the 2026-07-27 reference capture every
  destination-tagged reload track was refused by the tracker, so ``refuse`` is the
  measured pessimistic case.

Strictly offline and read-only: it opens ``.mcap`` files only — no node, no
socket, no hardware — and needs no ROS 2 (``mcap_ros2`` decodes the schemas
embedded in the bag, imported lazily so the scoring half runs without it).

USAGE
-----
    python tools/probes/hand_sensor_verdict_replay.py --bag 2026-08-10_16-30-44
    python tools/probes/hand_sensor_verdict_replay.py --bag A --bag B --bag C --json
    python tools/probes/hand_sensor_verdict_replay.py --self-check
    python tools/probes/hand_sensor_verdict_replay.py \
        --bag 2026-08-10_16-30-44 --emit-fixture --t0 140 --t1 310

Exit 0 = every requested bag replayed; 1 = a bag was unreadable or the
self-check failed.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from datetime import datetime

_HERE = os.path.dirname(os.path.abspath(__file__))          # tools/probes
_REPO = os.path.dirname(os.path.dirname(_HERE))             # repo root
for _p in (_REPO, os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import jugglebot.hardware_config as hw                          # noqa: E402
from jugglebot.ball_possession import (                         # noqa: E402
    ARRIVAL_CONFIRMED,
    ARRIVAL_REJECTED,
    ARRIVAL_UNKNOWN,
    EVIDENCE_SEATED,
    RETENTION_CONFIRMED,
    RETENTION_REJECTED,
    HandBallSensorSource,
    PossessionVerdict,
    merge_possession,
)

DEFAULT_ROOT = os.path.expanduser('~/Desktop/rosbags')
FIXTURE_PATH = os.path.join(_HERE, 'data', 'hand_sensor_replay_fixture.json')
OUT_DIR = os.path.join(_REPO, 'temp', 'probes')

# ── The node's own construction, mirrored (drift-guarded by the consuming test) ─
ARRIVAL_LEAD_S = float(hw.JB_BD_ARRIVAL_LEAD_S)
ARRIVAL_WINDOW_S = float(hw.JB_BD_ARRIVAL_WINDOW_S)
RETENTION_WINDOW_S = float(hw.JB_BD_RETENTION_WINDOW_S)
# reload_coordinator_node._HAND_STATE_STALE_S — the observation-level hand
# freshness window the node passes in. Mirrored rather than imported because
# importing the node drags rclpy in, and this probe must run without ROS.
STALE_S = 0.5

LABEL_CAUGHT = 'CAUGHT'
LABEL_BOUNCE = 'BOUNCE'
LABEL_MISSED = 'MISSED'
LABEL_UNKNOWN = 'UNKNOWN'


def make_source() -> HandBallSensorSource:
    """The source exactly as ``reload_coordinator_node.__init__`` builds it."""
    return HandBallSensorSource(arrival_lead_s=ARRIVAL_LEAD_S,
                                arrival_window_s=ARRIVAL_WINDOW_S,
                                retention_window_s=RETENTION_WINDOW_S,
                                stale_s=STALE_S)


# ── Pure scoring half (no bag reader, no ROS — this is what the test drives) ───

def label_for(verdict: PossessionVerdict) -> str:
    if verdict.arrival == ARRIVAL_CONFIRMED:
        return (LABEL_BOUNCE if verdict.retention == RETENTION_REJECTED
                else LABEL_CAUGHT)
    if verdict.arrival == ARRIVAL_REJECTED:
        return LABEL_MISSED
    return LABEL_UNKNOWN


def replay(samples, announcements, quick_drop_s=1.5):
    """Feed ``samples`` [(t, held, valid)] into a fresh production source and score
    every announcement in ``announcements`` [(t_ann, thrower, t_landing)].

    Returns ``(rows, ledger)``. Every quantity is derived from the source itself,
    not recomputed here — a probe that scored throws with its own copy of the rule
    would agree with the robot only by coincidence.
    """
    src = make_source()
    samples = sorted(samples, key=lambda s: s[0])
    pending = sorted(announcements, key=lambda a: a[2])
    rows = []
    cursor = 0
    n_valid = 0
    # The instant each announcement can be scored: the arrival window has closed
    # AND the retention window that a last-instant arrival would open has too.
    # Scoring earlier reads UNKNOWN for a throw that had not finished happening.
    for t, held, valid in samples:
        n_valid += 1 if valid else 0
        src.note_sample(t, held=bool(held), valid=bool(valid))
        while cursor < len(pending):
            t_ann, thrower, land = pending[cursor]
            due = land + ARRIVAL_WINDOW_S + RETENTION_WINDOW_S
            if t < due:
                break
            v = src.observe(t, landing_t=land)
            edge = src.arrival_time(land)
            rows.append({
                'thrower': thrower,
                't_announce': t_ann,
                't_landing': land,
                'arrival': v.arrival,
                'retention': v.retention,
                'reason': v.reason,
                'label': label_for(v),
                'catch_dt_s': (edge - land) if edge == edge else float('nan'),
                'seated_at_score': src.evidence(t) == EVIDENCE_SEATED,
            })
            cursor += 1
    # Announcements whose scoring instant never arrived (the bag ended first) are
    # reported rather than dropped: a silently missing row is how a replay
    # overstates its own agreement.
    for t_ann, thrower, land in pending[cursor:]:
        rows.append({'thrower': thrower, 't_announce': t_ann, 't_landing': land,
                     'arrival': ARRIVAL_UNKNOWN, 'retention': 'UNKNOWN',
                     'reason': 'BAG_ENDED_BEFORE_WINDOW_CLOSED',
                     'label': LABEL_UNKNOWN, 'catch_dt_s': float('nan'),
                     'seated_at_score': False})
    return rows, ledger(samples, n_valid, quick_drop_s)


def ledger(samples, n_valid, quick_drop_s=1.5):
    """The bag-level sensor ledger — the independent cross-check a replay has to
    reconcile against (transition counts are what an orchestrator counts by hand).

    Transitions are taken on the DEBOUNCED verdict over VALID samples only, which
    is the same rule the source applies, so a mismatch between this table and the
    per-throw rows is a real disagreement rather than two conventions.
    """
    rises, falls, segments = [], [], []
    prev = None
    open_t = None
    for t, held, valid in samples:
        if not valid:
            continue
        if prev is None:
            prev = bool(held)
            if prev:
                open_t = t
            continue
        if bool(held) != prev:
            prev = bool(held)
            if prev:
                rises.append(t)
                open_t = t
            else:
                falls.append(t)
                if open_t is not None:
                    segments.append((open_t, t - open_t))
                    open_t = None
    if open_t is not None and samples:
        segments.append((open_t, samples[-1][0] - open_t))
    quick = [(t, d) for t, d in segments if d < quick_drop_s]
    return {
        'n_samples': len(samples),
        'n_valid': n_valid,
        'valid_frac': (n_valid / len(samples)) if samples else 0.0,
        'n_rises': len(rises),
        'n_falls': len(falls),
        'n_segments': len(segments),
        'segment_s': sorted(d for _t, d in segments),
        'quick_drops': quick,
    }


def counterfactual(rows, tracker_mode):
    """What the verdict would be, given a tracker that always ``refuse``s or
    always ``confirm``s.

    ⚠ **IT IS NOW A CONSTANT FUNCTION OF ``tracker_mode``, deliberately** (owner
    decision D1, 2026-08-26). It used to make the tracker-fallback rule's
    consequence countable — "the UNKNOWN rows are exactly the ones where the
    tracker still decides" — and there is no fallback any more: the cup is the
    sole source and the tracker cannot move a verdict in either direction. Kept,
    rather than deleted, precisely because it MEASURES that: a future edit that
    quietly re-opens a tracker path shows up here as a row whose verdict depends
    on ``tracker_mode``, which is what the two self-check cases below now pin."""
    from jugglebot.ball_possession import SOURCE_TRACKER_ARRIVAL
    ok = tracker_mode == 'confirm'
    tracker = PossessionVerdict(
        SOURCE_TRACKER_ARRIVAL,
        ARRIVAL_CONFIRMED if ok else ARRIVAL_REJECTED,
        'UNKNOWN', 0.0 if ok else 500.0, 0.0,
        'ARRIVAL_OK' if ok else 'ARRIVAL_FAR')
    out = []
    for r in rows:
        sensor = PossessionVerdict('hand_ball_sensor', r['arrival'],
                                   r['retention'], float('nan'), float('nan'),
                                   r['reason'])
        out.append(merge_possession(sensor=sensor, tracker=tracker).confirmed)
    return out


# ── Bag reading ───────────────────────────────────────────────────────────────

def _stamp_s(t):
    return float(t.sec) + float(t.nanosec) * 1e-9


def read_bag(path):
    """-> (samples, announcements, t0). Raises on an unreadable bag."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    import glob
    files = sorted(glob.glob(os.path.join(path, '*.mcap')))
    if not files:
        raise IOError('no .mcap in {}'.format(path))
    raw, anns = [], []
    for f in files:
        with open(f, 'rb') as fh:
            r = make_reader(fh, decoder_factories=[DecoderFactory()])
            for _sch, ch, msg, dec in r.iter_decoded_messages(
                    topics=['/hand_telemetry', '/throw_announcements']):
                t = msg.log_time / 1e9
                if ch.topic == '/hand_telemetry':
                    raw.append((t, bool(dec.ball_held),
                                bool(dec.ball_held_valid),
                                _stamp_s(dec.timestamp)))
                else:
                    anns.append((t, str(dec.thrower_name),
                                 _stamp_s(dec.landing_time)))
    if not raw:
        raise IOError('no /hand_telemetry in {}'.format(path))
    raw.sort(key=lambda s: s[0])
    # ROS header stamps -> the bag's log_time clock. One median offset over the
    # whole hand stream: both clocks are the same machine's, so the offset is a
    # constant plus publish jitter, and the median is what survives the jitter.
    offs = sorted(t - ts for t, _h, _v, ts in raw if ts > 0)
    off = offs[len(offs) // 2] if offs else 0.0
    t0 = raw[0][0]
    samples = [(t - t0, h, v) for t, h, v, _ts in raw]
    announcements = [(t - t0, name, land + off - t0)
                     for t, name, land in anns if land > 0]
    return samples, announcements, off


# ── Fixture (committed cut + its regeneration recipe) ─────────────────────────

def emit_fixture(samples, announcements, bag_name, t0, t1, path=FIXTURE_PATH):
    """Write a committed cut of ``[t0, t1)`` so the consuming test runs on a fresh
    clone where ``~/Desktop/rosbags`` does not exist.

    Stored as CUMULATIVE integer-millisecond stamps plus run-length-encoded
    (held, valid) flags. Every sample instant is preserved to ±0.5 ms — staleness
    and the catch-event time both depend on the instants, not just the order — and
    the flag runs shrink the file ~20x against raw triples.

    Cumulative, not per-sample deltas, and that is not a style choice: rounding
    16,444 deltas to whole milliseconds accumulates a random-walk drift of
    ~17 ms over a 165 s cut, which showed up as a 14 ms error in the replayed
    catch-event time on the first attempt. Absolute stamps cannot drift."""
    cut = [(t, h, v) for t, h, v in samples if t0 <= t < t1]
    if not cut:
        raise ValueError('empty cut')
    base = cut[0][0]
    t_ms, runs = [], []
    for t, h, v in cut:
        t_ms.append(int(round((t - base) * 1000.0)))
        flag = (1 if h else 0) | (2 if v else 0)
        if runs and runs[-1][0] == flag:
            runs[-1][1] += 1
        else:
            runs.append([flag, 1])
    doc = {
        'source_bag': bag_name,
        'cut_s': [t0, t1],
        'recipe': ('python tools/probes/hand_sensor_verdict_replay.py --bag '
                   '{} --emit-fixture --t0 {:g} --t1 {:g}'.format(
                       bag_name, t0, t1)),
        'base_t': base,
        't_ms': t_ms,
        'flag_runs': runs,
        'announcements': [[round(a - base, 6), n, round(land - base, 6)]
                          for a, n, land in announcements if t0 <= land < t1],
    }
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        json.dump(doc, f, indent=1)
    return path


def load_fixture(path=FIXTURE_PATH):
    """-> (samples, announcements, doc). The inverse of :func:`emit_fixture`."""
    with open(path) as f:
        doc = json.load(f)
    flags = []
    for flag, n in doc['flag_runs']:
        flags.extend([flag] * int(n))
    samples = [(t / 1000.0, bool(flag & 1), bool(flag & 2))
               for t, flag in zip(doc['t_ms'], flags)]
    anns = [(float(a), str(n), float(land)) for a, n, land in doc['announcements']]
    return samples, anns, doc


# ── Reporting ─────────────────────────────────────────────────────────────────

def print_report(name, rows, led, tracker_mode=None):
    print('\n=== {}'.format(name))
    print('  ledger: {} samples, ball_held_valid {}/{} ({:.1f} %), '
          '{} empty->held, {} held->empty, {} held segments'.format(
              led['n_samples'], led['n_valid'], led['n_samples'],
              100.0 * led['valid_frac'], led['n_rises'], led['n_falls'],
              led['n_segments']))
    if led['segment_s']:
        s = led['segment_s']
        print('  held durations: min {:.3f}  med {:.3f}  max {:.3f} s'.format(
            s[0], s[len(s) // 2], s[-1]))
    if led['quick_drops']:
        print('  quick-drops (seat-then-leave): {}'.format(
            ', '.join('t={:.3f}s dur={:.3f}s'.format(t, d)
                      for t, d in led['quick_drops'])))
    else:
        print('  quick-drops (seat-then-leave): none')
    cf = counterfactual(rows, tracker_mode) if tracker_mode else None
    hdr = '  {:>9} {:>11} {:>10} {:>10} {:>10}  {}'.format(
        'land[s]', 'thrower', 'arrival', 'retention', 'catch_dt', 'label')
    if cf is not None:
        hdr += '  merged({})'.format(tracker_mode)
    print(hdr)
    for i, r in enumerate(rows):
        dt = r['catch_dt_s']
        line = '  {:9.3f} {:>11} {:>10} {:>10} {:>10}  {}'.format(
            r['t_landing'], r['thrower'][:11], r['arrival'], r['retention'],
            ('{:+.3f}'.format(dt) if dt == dt else '-'), r['label'])
        if cf is not None:
            line += '  {}'.format('CONFIRMED' if cf[i] else 'refused')
        print(line)
    counts = {}
    for r in rows:
        counts[r['label']] = counts.get(r['label'], 0) + 1
    print('  labels: ' + ', '.join('{}={}'.format(k, counts[k])
                                   for k in sorted(counts)))
    caught = [r['catch_dt_s'] for r in rows
              if r['label'] in (LABEL_CAUGHT, LABEL_BOUNCE)
              and r['catch_dt_s'] == r['catch_dt_s']]
    if caught:
        c = sorted(caught)
        print('  catch_dt over {} arrivals: min {:+.0f}  med {:+.0f}  '
              'max {:+.0f} ms'.format(len(c), c[0] * 1e3,
                                      c[len(c) // 2] * 1e3, c[-1] * 1e3))


# ── Self-check (two-sided instrument acceptance; needs NO bag) ────────────────

def self_check() -> int:
    """Two-sided by construction: every ACCEPT case has a REFUSE twin one measured
    millisecond away, so a window that silently widened or a retention rule that
    stopped firing fails here rather than at the bench.

    "Two-sided" has to mean ABSOLUTE on at least one side of each window. A case
    written relative to the constant it is guarding (``RETENTION_WINDOW_S + 0.2``)
    follows that constant wherever it goes and can only catch a NARROWING — which
    is how the retention window shipped on 2026-08-10 with no widen guard at all
    (audit finding). The absolute cases are pinned to the measured session and are
    MEANT to go red when a window is re-sized: re-run the probe over the bags, and
    re-baseline them deliberately."""
    fails = []
    n_checks = 0

    def check(name, got, want):
        nonlocal n_checks
        n_checks += 1
        if got != want:
            fails.append('{}: got {!r}, want {!r}'.format(name, got, want))

    def run(edge_dt=None, hold_s=None, blind=False, land=10.0, end=None):
        """Build a synthetic session: empty cup, an edge at land+edge_dt, held for
        hold_s (None = to the end)."""
        samples = []
        t = 8.0
        end = end or (land + ARRIVAL_WINDOW_S + RETENTION_WINDOW_S + 1.0)
        edge = None if edge_dt is None else land + edge_dt
        drop = None if (edge is None or hold_s is None) else edge + hold_s
        while t < end:
            if blind and (land - ARRIVAL_LEAD_S - 0.1) <= t <= (
                    land + ARRIVAL_WINDOW_S + 0.1):
                t += 0.01
                continue
            held = edge is not None and t >= edge and (drop is None or t < drop)
            samples.append((t, held, True))
            t += 0.01
        rows, _led = replay(samples, [(land - 5.0, 'jugglebot', land)])
        return rows[0]

    # ACCEPT: the measured catch band, both ends.
    check('earliest measured catch (+137 ms)', run(0.137)['label'], LABEL_CAUGHT)
    check('latest measured catch (+798 ms)', run(0.798)['label'], LABEL_CAUGHT)
    # REFUSE: the earliest measured NON-catch edge.
    check('earliest measured re-seat (+3194 ms)',
          run(3.194)['label'], LABEL_MISSED)
    # ACCEPT/REFUSE across the retention window, at the measured bounce durations.
    check('longest measured seat-then-leave (0.999 s)',
          run(0.4, hold_s=0.999)['label'], LABEL_BOUNCE)
    check('a hold past the retention window',
          run(0.4, hold_s=RETENTION_WINDOW_S + 0.2)['label'], LABEL_CAUGHT)
    # The WIDEN twin, absolute: a 1.55 s hold is a keeper at the shipped 1.50 s
    # window and a BOUNCE at anything from ~1.56 s up. The case above cannot see a
    # widening because it is written relative to the constant.
    check('a 1.55 s hold at the SHIPPED 1.50 s window (widen guard)',
          run(0.4, hold_s=1.55)['label'], LABEL_CAUGHT)
    # Blindness beats both.
    check('blind across the window', run(0.4, blind=True)['label'], LABEL_UNKNOWN)
    # No edge at all: the ball never left the cup, so nothing arrived.
    check('no edge in the window', run(None)['label'], LABEL_MISSED)
    # D1 (2026-08-26): the tracker decides NOTHING, in either direction. These two
    # cases used to pin the fallback rule; they now pin its absence, which is the
    # property that has to stay true.
    blind_row = run(0.4, blind=True)
    check('blind + confirming tracker -> STILL refuses (no fallback)',
          counterfactual([blind_row], 'confirm')[0], False)
    check('sensor REJECTED vetoes a confirming tracker',
          counterfactual([run(3.194)], 'confirm')[0], False)
    # …and a CONFIRMING cup is not vetoed by a refusing tracker either — the same
    # rule read from the other side, and the direction that recovered 13 real
    # reload catches from corrupt split tracks.
    check('sensor CONFIRMED survives a refusing tracker',
          counterfactual([run(0.4)], 'refuse')[0], True)

    for f in fails:
        print('FAIL  {}'.format(f))
    print('{}/{} self-check cases pass'.format(n_checks - len(fails), n_checks))
    return 1 if fails else 0


# ── CLI ───────────────────────────────────────────────────────────────────────

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bag', action='append', default=[],
                    help='bag directory name under --root (repeatable)')
    ap.add_argument('--root', default=DEFAULT_ROOT)
    ap.add_argument('--quick-drop-s', type=float, default=1.5,
                    help='held segments shorter than this are listed as '
                         'seat-then-leave events (default 1.5)')
    ap.add_argument('--merge-tracker', choices=('refuse', 'confirm'),
                    help='also print the MERGED verdict against a tracker that '
                         'always refuses / always confirms')
    ap.add_argument('--json', action='store_true',
                    help='write a timestamped JSON report to temp/probes/')
    ap.add_argument('--self-check', action='store_true')
    ap.add_argument('--emit-fixture', action='store_true',
                    help='write the committed test cut (needs one --bag)')
    ap.add_argument('--t0', type=float, default=0.0)
    ap.add_argument('--t1', type=float, default=1e9)
    args = ap.parse_args(argv)

    if args.self_check:
        return self_check()
    if not args.bag:
        ap.error('need --bag (or --self-check)')

    reports, rc = {}, 0
    for name in args.bag:
        path = os.path.join(args.root, name)
        try:
            samples, anns, _off = read_bag(path)
        except Exception as exc:                          # noqa: BLE001
            print('ERROR: {}: {}'.format(name, exc))
            rc = 1
            continue
        if args.emit_fixture:
            out = emit_fixture(samples, anns, name, args.t0,
                               min(args.t1, samples[-1][0]))
            print('fixture -> {} ({} samples in [{:g}, {:g}) s)'.format(
                out, sum(1 for t, _h, _v in samples if args.t0 <= t < args.t1),
                args.t0, min(args.t1, samples[-1][0])))
            continue
        rows, led = replay(samples, anns, args.quick_drop_s)
        print_report(name, rows, led, args.merge_tracker)
        reports[name] = {'ledger': led, 'rows': rows}

    if args.json and reports:
        os.makedirs(OUT_DIR, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        out = os.path.join(OUT_DIR,
                           'hand_sensor_verdict_replay_{}.json'.format(stamp))
        with open(out, 'w') as f:
            json.dump({'windows': {'arrival_lead_s': ARRIVAL_LEAD_S,
                                   'arrival_window_s': ARRIVAL_WINDOW_S,
                                   'retention_window_s': RETENTION_WINDOW_S,
                                   'stale_s': STALE_S},
                       'bags': reports}, f, indent=1)
        print('\nJSON -> {}'.format(out))
    return rc


if __name__ == '__main__':
    sys.exit(main())
