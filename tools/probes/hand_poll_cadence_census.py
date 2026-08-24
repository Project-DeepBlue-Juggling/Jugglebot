#!/usr/bin/env python3
"""The hand-sensor POLL CADENCE census — per bag, and over a corpus.

WHAT IT DOES
------------
Measures how often the can-bridge actually got an SDO reply from the hand ball
sensor, by counting **distinct ``ball_held_stamp`` advances** in a bag's
``/hand_telemetry`` stream, and reports the spread (p5 / p50 / p95 / max) against
the configured ``JB_BD_CHECK_INTERVAL_MS``. With ``--all`` it does the same over
a corpus and prints one row per bag next to the bridge uptime that bag opened at.

This exists because the number is a **measurement of the session in front of
you**, not a property of the plant: the same configured 20 ms has produced a
measured median of 71 ms (``2026-08-10_16-30-44``) and of 20 ms (every decodable
bag since the FW 14 ring-leak fix of 2026-08-15). It is the instrument behind
``logbook/2026-08-24-hand-sensor-poll-cadence.md``, committed so that entry's
tables can be re-derived rather than taken on trust.

Reading a row: an elevated cadence means **investigate**. The bridge's poll loop
is strictly one-in-flight on a 10 ms task grid, so the achieved cadence is
``C = 10 * max(2, ceil(RTT/10) + 1)`` — i.e. C inverts to a round-trip time, and
*anything* that slows the round trip raises C. The transfer function is the
useful inheritance; a particular mechanism is not, and this probe deliberately
names none.

WHAT IT IMPORTS RATHER THAN RE-IMPLEMENTS
------------------------------------------
- **The step rule** is ``jugglebot.toss_record.poll_dt_steps_ms`` — the single
  definition of what counts as one poll interval (repeats are not steps; a
  backwards step is counted and reseeds the tracker; ``prev`` resets on any
  invalid sample; a step across the bridge's wall anchor is refused). Its
  contract is pinned by ``tests/motion/test_toss_record.py`` § 2b, not here, so
  this probe and the mined ``sensor_poll_dt_ms_median`` field on every
  ``toss_record/1`` row cannot drift onto two notions of a poll.
- **The bag reader** is ``toss_record_miner.read_bag(..., sensor_only=True)``,
  which also carries the ``/hand_telemetry`` clock rule (``hand_sample_time``)
  and the ``/link_status`` stream this uses for bridge uptime. ``sensor_only``
  skips the mocap pass, which is minutes per sitting and irrelevant here.

So the only thing this file actually defines is the **reduction**: percentiles,
the per-bag row, and the corpus table. That is what ``--self-check`` covers.

SCOPE
-----
Cadence and bridge uptime. It does **not** read ``/ring_diag``, so ring
occupancy (``leak_jb``) is out of scope — see
``logbook/2026-08-14-s3-conviction-ring-leak-measured.md`` for that measurement.

OUTPUTS
-------
``temp/probes/hand_poll_cadence_census_<stamp>.json`` with ``--json``. Bags are
processed **sequentially** (8 GB box).

USAGE
-----
    python tools/probes/hand_poll_cadence_census.py --bag 2026-08-10_16-30-44
    python tools/probes/hand_poll_cadence_census.py --bag ~/Desktop/rosbags/2026-08-12_14-55-18
    python tools/probes/hand_poll_cadence_census.py --all --match '2026-08-*' --json
    python tools/probes/hand_poll_cadence_census.py --self-check

Exit 0 = every requested bag was read; 1 = a bag was unreadable (no
``/hand_telemetry``, truncated, absent) or the self-check failed. A bag that
cannot be read is REPORTED and the corpus run continues — "four of these bags
carry no hand topic" is itself a census result.
"""

from __future__ import annotations

import argparse
import fnmatch
import glob
import json
import os
import sys
from datetime import datetime

_HERE = os.path.dirname(os.path.abspath(__file__))           # tools/probes
_REPO = os.path.dirname(os.path.dirname(_HERE))             # repo root
for _p in (_HERE, _REPO, os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import jugglebot.hardware_config as hw                              # noqa: E402
import toss_record_miner as miner                                   # noqa: E402
from jugglebot.toss_record import (SensorSample,                    # noqa: E402
                                   poll_dt_ms_median, poll_dt_steps_ms)

DEFAULT_ROOT = miner.DEFAULT_ROOT
OUT_DIR = os.path.join(_REPO, 'temp', 'probes')

#: The can-bridge fix that removed the RX-ring delay line
#: (``logbook/2026-08-15-fw14-validated-arc-closed.md``). Used ONLY to split the
#: corpus table into before/after — it is a date, not a prediction.
FW14_DATE = '2026-08-15'


def _pct(sorted_steps, q: float) -> float:
    """The percentile index rule ``tools/probes/hand_sensor_settle.poll_cadence``
    uses, kept identical so the two censuses are comparable — and so that
    ``_pct(s, 0.5)`` is exactly ``toss_record.poll_dt_ms_median`` rather than a
    second, subtly different median (``--self-check`` asserts that equality)."""
    return float(sorted_steps[min(len(sorted_steps) - 1,
                                  int(q * len(sorted_steps)))])


def cadence(samples) -> dict:
    """-> the cadence block for one ``/hand_telemetry`` stream.

    ``n`` is the number of MEASURED intervals, not the number of messages: the
    bridge republishes its cached bit at 100 Hz, so counting messages would
    report the publish rate for every plant and be wrong by the whole factor
    this measurement exists to expose.
    """
    steps = poll_dt_steps_ms(samples)
    valid = sum(1 for s in samples if s.valid)
    block = {
        'n_samples': len(samples),
        'n_valid': valid,
        'valid_frac': round(valid / float(len(samples)), 4) if samples else None,
        'n': len(steps.steps_ms),
        'n_backwards': steps.n_backwards,
        'n_domain_breaks': steps.n_domain_breaks,
        'configured_ms': float(hw.JB_BD_CHECK_INTERVAL_MS),
    }
    if not steps.steps_ms:
        return block
    srt = sorted(steps.steps_ms)
    block.update({
        'p5_ms': round(_pct(srt, .05), 3),
        'p50_ms': round(_pct(srt, .50), 3),
        'p95_ms': round(_pct(srt, .95), 3),
        'max_ms': round(srt[-1], 3),
        'mean_ms': round(sum(srt) / len(srt), 3),
    })
    return block


def _uptime_h(link, which: int):
    """Bridge uptime in hours at the first (0) or last (-1) ``/link_status``
    sample that carries a parseable ``uptime_ms``. ``None`` when the bag has
    none — an absent covariate, never a zero."""
    vals = [v['uptime_ms'] for _t, v in link
            if str(v.get('uptime_ms', '')).lstrip('-').isdigit()]
    if not vals:
        return None
    return round(int(vals[which]) / 3.6e6, 2)


def _bridge_fw(link) -> str:
    """The LAST sample's ``bridge_fw_version``. The first usually still renders
    "unknown (never seen)" because the BRIDGE_IDENTITY frame has not landed at
    the moment recording starts."""
    for _t, values in reversed(link):
        fw = str(values.get('bridge_fw_version', '') or '')
        if fw and not fw.startswith('unknown'):
            return fw
    return ''


def census_bag(path: str) -> dict:
    """-> one bag's row. Raises ``IOError`` when the bag cannot be read."""
    data = miner.read_bag(path, sensor_only=True)
    row = {'bag': miner.bag_label(path)}
    row.update(cadence(data.hand))
    if data.hand:
        row['dur_s'] = round(data.hand[-1].t - data.hand[0].t, 3)
    row['uptime_h_open'] = _uptime_h(data.link, 0)
    row['uptime_h_close'] = _uptime_h(data.link, -1)
    row['bridge_fw'] = _bridge_fw(data.link)
    return row


_HDR = ('{:<22} {:>7} {:>8} {:>8} {:>8} {:>9} {:>6} {:>5} {:>9}'
        .format('bag', 'steps', 'p5', 'p50', 'p95', 'max', 'back', 'dom',
                'uptime_h'))


def _row_line(row: dict) -> str:
    def f(key):
        v = row.get(key)
        return '-' if v is None else '{:.1f}'.format(v)
    up = row.get('uptime_h_open')
    return ('{:<22} {:>7} {:>8} {:>8} {:>8} {:>9} {:>6} {:>5} {:>9}'.format(
        row['bag'][:22], row.get('n', 0), f('p5_ms'), f('p50_ms'), f('p95_ms'),
        f('max_ms'), row.get('n_backwards', 0), row.get('n_domain_breaks', 0),
        '-' if up is None else '{:.1f}'.format(up)))


def summarise(rows) -> dict:
    """Corpus-level reduction: how many bags median AT the configured interval,
    split around the FW 14 date. A count, not a claim about a mechanism.

    The split is a string compare because these bag names are ISO-date-prefixed
    (``YYYY-MM-DD_HH-MM-SS``), so lexicographic order IS chronological order. A
    bag named otherwise lands in the ``before`` bucket; ``--match`` is what keeps
    the surveyed set to date-named sittings.
    """
    measured = [r for r in rows if r.get('n')]
    cfg = float(hw.JB_BD_CHECK_INTERVAL_MS)
    before = [r for r in measured if r['bag'] < FW14_DATE]
    after = [r for r in measured if r['bag'] >= FW14_DATE]

    def block(sub):
        if not sub:
            return {'n_bags': 0}
        p50s = sorted(r['p50_ms'] for r in sub)
        return {'n_bags': len(sub),
                'p50_min': p50s[0], 'p50_max': p50s[-1],
                'p50_median': p50s[len(p50s) // 2],
                'n_at_configured': sum(1 for p in p50s if abs(p - cfg) <= 1.0),
                'uptime_h_span': [min((r['uptime_h_open'] for r in sub
                                       if r['uptime_h_open'] is not None),
                                      default=None),
                                  max((r['uptime_h_open'] for r in sub
                                       if r['uptime_h_open'] is not None),
                                      default=None)]}
    return {'n_bags_measured': len(measured), 'configured_ms': cfg,
            'fw14_date': FW14_DATE,
            'before_fw14': block(before), 'from_fw14': block(after)}


# ── Self-check (bag-free) ─────────────────────────────────────────────────────

def _poll_stream(poll_dt=0.020, publish_dt=0.010, n_polls=40, t0=1000.0):
    """A stream that REPUBLISHES one poll several times, as the bridge does.
    Integer division builds the stamp so the repeat count is exact."""
    repeat = int(round(poll_dt / publish_dt))
    return [SensorSample(t=t0 + k * publish_dt, held=True, raw=True, valid=True,
                         stamp=t0 + (k // repeat) * poll_dt)
            for k in range(n_polls * repeat)]


def self_check() -> int:
    """Bag-free acceptance for THIS file's reduction.

    It deliberately does not re-test ``poll_dt_steps_ms`` — that contract lives
    in ``tests/motion/test_toss_record.py`` § 2b, and duplicating it here would
    create the second definition this probe exists to avoid. What is checked is
    everything this file adds: the percentile rule (including its identity with
    ``poll_dt_ms_median`` at p50), the republish dedupe surviving the reduction,
    the refusal counters reaching the row, and the corpus split.
    """
    fails = []

    def ck(name, cond, detail=''):
        if not cond:
            fails.append('{}{}'.format(name, ' — ' + detail if detail else ''))

    # 1. The poll rate is reported, not the 100 Hz republish rate.
    for poll_dt in (0.020, 0.050, 0.071):
        blk = cadence(_poll_stream(poll_dt=poll_dt))
        ck('dedupe@{:.0f}ms'.format(poll_dt * 1e3),
           abs(blk['p50_ms'] - poll_dt * 1e3) < 0.5 and blk['n'] == 39,
           'p50={} n={}'.format(blk.get('p50_ms'), blk['n']))
        ck('not-the-publish-rate@{:.0f}ms'.format(poll_dt * 1e3),
           abs(blk['p50_ms'] - 10.0) > 0.5)

    # 2. p50 by the index rule IS poll_dt_ms_median — one median, not two.
    for poll_dt in (0.020, 0.050, 0.071):
        s = _poll_stream(poll_dt=poll_dt)
        ck('p50-is-poll_dt_ms_median@{:.0f}ms'.format(poll_dt * 1e3),
           abs(cadence(s)['p50_ms'] - round(poll_dt_ms_median(s), 3)) < 1e-9)

    # 3. A backwards re-anchor is counted and mints nothing. The old tracker
    #    carried `prev` onto the LOW stamp and the next sample came back as a
    #    500 ms "poll"; a max assertion is what catches that, not a sign one.
    s = _poll_stream()
    hurt = list(s)
    hurt[20] = s[20]._replace(stamp=s[20].stamp - 0.5)
    blk = cadence(hurt)
    ck('backwards-counted', blk['n_backwards'] == 1)
    ck('backwards-mints-nothing', abs(blk['max_ms'] - 20.0) < 0.5,
       'max={}'.format(blk.get('max_ms')))
    ck('backwards-costs-one-interval', blk['n'] == 38, 'n={}'.format(blk['n']))

    # 4. An invalid span is darkness, not one long poll.
    blind = set(range(30, 50))
    hurt = [x._replace(valid=False) if i in blind else x for i, x in enumerate(s)]
    blk = cadence(hurt)
    ck('blind-span-not-spanned', abs(blk['max_ms'] - 20.0) < 0.5,
       'max={}'.format(blk.get('max_ms')))
    ck('valid_frac-reported', abs(blk['valid_frac'] - 0.75) < 1e-6,
       'valid_frac={}'.format(blk['valid_frac']))

    # 5. The wall-anchor discontinuity is refused and counted. Stamps verbatim
    #    from 2026-08-12_14-55-18, driven with valid=True (the reachable case).
    wall_t0 = 1786510521.957
    boot = (17.140004, 17.154007, 17.154007, 17.191004, 17.211004, 17.251004)
    pre = [SensorSample(t=wall_t0 + k * 0.010, held=True, raw=True, valid=True,
                        stamp=boot[k]) for k in range(len(boot))]
    blk = cadence(pre + _poll_stream(n_polls=20, t0=wall_t0 + len(boot) * 0.010))
    ck('anchor-refused', blk['n_domain_breaks'] == 1)
    ck('anchor-not-in-stats', blk['max_ms'] < 1e3,
       'max={}'.format(blk.get('max_ms')))

    # 6. A never-advancing stream measures nothing; it is not a 0 ms poll.
    frozen = [SensorSample(t=1000.0 + k * 0.01, held=True, raw=True, valid=True,
                           stamp=1000.0) for k in range(50)]
    blk = cadence(frozen)
    ck('frozen-measures-nothing', blk['n'] == 0 and 'p50_ms' not in blk)

    # 7. The corpus split counts bags, and an unmeasured bag never enters it.
    rows = [{'bag': '2026-08-10_16-30-44', 'n': 9845, 'p50_ms': 70.998,
             'uptime_h_open': 4.0},
            {'bag': '2026-08-15_00-44-59', 'n': 3440, 'p50_ms': 20.0,
             'uptime_h_open': 5.8},
            {'bag': '2026-08-23_19-14-54', 'n': 11462, 'p50_ms': 20.0,
             'uptime_h_open': 116.7},
            {'bag': '2026-08-18_22-32-30', 'n': 0}]
    summ = summarise(rows)
    ck('split-before', summ['before_fw14']['n_bags'] == 1
       and summ['before_fw14']['n_at_configured'] == 0)
    ck('split-after', summ['from_fw14']['n_bags'] == 2
       and summ['from_fw14']['n_at_configured'] == 2)
    ck('unmeasured-excluded', summ['n_bags_measured'] == 3)

    # 8. --bag takes a name OR a path, and the row is labelled the same either
    #    way (the miner's own rule, so the label cannot drift from its reports).
    ck('bag_label-path', miner.bag_label(
        '/home/jetson/Desktop/rosbags/2026-08-10_16-30-44/')
        == '2026-08-10_16-30-44')

    for f in fails:
        print('FAIL  {}'.format(f))
    total = 8
    print('\nself-check: {} — {} group(s), {} failure(s)'.format(
        'PASS' if not fails else 'FAIL', total, len(fails)))
    return 1 if fails else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bag', action='append', default=[],
                    help='bag directory NAME under --root, or a full path '
                         '(repeatable)')
    ap.add_argument('--all', action='store_true',
                    help='every bag under --root (sequentially — 8 GB box)')
    ap.add_argument('--match', default='',
                    help="with --all, a glob over the bag name (e.g. "
                         "'2026-08-*'); a pattern with no glob character is "
                         'treated as a substring')
    ap.add_argument('--root', default=DEFAULT_ROOT)
    ap.add_argument('--json', action='store_true',
                    help='write a timestamped JSON report to temp/probes/')
    ap.add_argument('--self-check', action='store_true')
    args = ap.parse_args(argv)

    if args.self_check:
        return self_check()

    paths = [b if os.sep in b else os.path.join(args.root, b)
             for b in args.bag]
    if args.all:
        pat = args.match
        if pat and not any(c in pat for c in '*?['):
            pat = '*{}*'.format(pat)
        seen = set(miner.bag_label(p) for p in paths)
        for p in sorted(glob.glob(os.path.join(args.root, '*'))):
            name = os.path.basename(p)
            if (os.path.isdir(p) and name not in seen
                    and (not pat or fnmatch.fnmatch(name, pat))):
                paths.append(p)
    if not paths:
        ap.error('need --bag or --all (or --self-check)')

    rows, errors, rc = [], [], 0
    print(_HDR)
    print('-' * len(_HDR))
    for path in paths:
        label = miner.bag_label(path)
        try:
            row = census_bag(path)
        except Exception as exc:            # noqa: BLE001 — see below
            # Deliberately broad. A corpus census exists to survey what the
            # corpus HOLDS, and "this bag is truncated" is one of its results,
            # not a reason to lose the other fifty. A half-written final chunk
            # surfaces as `struct.error` from inside the CDR reader — three
            # libraries down, no shared base class with the IOError a missing
            # bag raises — so an enumerated tuple silently means "the census
            # dies on the first bad bag it meets". The failure is REPORTED per
            # bag and sets the exit code, so nothing is swallowed.
            errors.append({'bag': label, 'error': '{}: {}'.format(
                type(exc).__name__, exc)})
            print('{:<22} UNREADABLE — {}: {}'.format(
                label[:22], type(exc).__name__, exc))
            rc = 1
            continue
        rows.append(row)
        print(_row_line(row))

    summ = summarise(rows)
    cfg = summ['configured_ms']
    print('\nconfigured JB_BD_CHECK_INTERVAL_MS = {:.0f} ms'.format(cfg))
    for key, label in (('before_fw14', 'before ' + FW14_DATE),
                       ('from_fw14', 'from ' + FW14_DATE + ' on')):
        b = summ[key]
        if not b['n_bags']:
            continue
        span = b['uptime_h_span']
        print('  {:<22} {:>2} bag(s)  p50 {:.1f}-{:.1f} ms  '
              '{}/{} within 1 ms of configured  uptime {}'.format(
                  label, b['n_bags'], b['p50_min'], b['p50_max'],
                  b['n_at_configured'], b['n_bags'],
                  '-' if span[0] is None
                  else '{:.1f}-{:.1f} h'.format(span[0], span[1])))
    if errors:
        print('  {} bag(s) unreadable: {}'.format(
            len(errors), ', '.join(e['bag'] for e in errors)))

    if args.json:
        os.makedirs(OUT_DIR, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        out = os.path.join(
            OUT_DIR, 'hand_poll_cadence_census_{}.json'.format(stamp))
        with open(out, 'w') as fh:
            json.dump({'generated': datetime.now().isoformat(timespec='seconds'),
                       'root': args.root, 'bags': rows, 'errors': errors,
                       'summary': summ}, fh, indent=1)
        print('\nJSON -> {}'.format(out))
    return rc


if __name__ == '__main__':
    sys.exit(main())
