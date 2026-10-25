#!/usr/bin/env python3
"""Hand cup census: how many throws and catches has the Hand made, from the bags?

WHAT IT DOES
------------
There is no throw counter in the code, so a cup's service life is derived from
the recorded sittings. For every bag under ``--bags`` whose directory name falls
in ``[--since, --until)`` (the ``YYYY-MM-DD_HH-MM-SS`` stamp is found anywhere
in the name — ``session_`` / ``cone_test_`` prefixes included — so a prefix
compares chronologically; an undated name is read only when neither
``--since`` nor ``--until`` is given), sequentially:

- **Throw** = one ``/throw_announcements`` message with
  ``thrower_name='jugglebot'``, de-duplicated on ``throw_time``.
- **Feed** = a ``ball_butler`` announcement with ``target_id='jugglebot'``.
- **Caught** (cup-sensor heuristic, NOT the production ``ball_possession.py``
  verdict): a valid, seated ``/hand_telemetry.ball_held`` inside
  ``[landing - 0.1 s, landing + 0.8 s]``; a self-throw must also read EMPTY
  between ``release - 0.3 s`` and the landing (the ball left the cup).
  Throws in a bag with no valid sensor sample are ``nosensor``; throws whose
  window holds no valid sample are ``unknown``.
- **Cross-checks**, independent of announcements: ``strokes`` = upward hand
  strokes (``vel_meas`` crossing ``--stroke-rev-s``, 0.4 s refractory) and
  ``sensor_throws`` = debounced HELD -> EMPTY departures with such a stroke in
  the preceding 0.5 s. They exist because some bags carry no announcements
  (e.g. eight of the ten 2026-08-12 bags); the 2026-10-21 census found every bag
  with a throw-speed stroke also had announcements, so the announcement count
  is the throw count. ``strokes`` run ~8 % above it in two-ball sessions
  (strokes that release no ball); ``sensor_throws`` below it (the sensor cannot
  tell which ball seated).
  ``strokes`` is meaningless before 2026-07 (older ``/hand_telemetry`` bags
  count tens of thousands): read the cross-checks for the current-layout era only.

Known biases of ``caught``: HIGH in two-ball sessions (the other ball's seat
scores, ``logbook/2026-10-04-skill-stack-r5-sitting-4.md``); LOW for throws
cancelled after announcement and seats later than 0.8 s. Throws are a floor:
an unrecorded sitting is invisible.

Decoding is hand-rolled CDR over ``rosbags.rosbag2.Reader`` (no typestore for
the project messages needed); the offsets follow
``jugglebot_interfaces/msg/ThrowAnnouncement.msg`` and
``HandTelemetryMessage.msg`` — re-check them if either message changes.

Motivating entry: ``logbook/2026-10-21-hand-cup-pla-broke-throw-catch-census.md``
(the PLA cup's life; an ASA cup was fitted 2026-10-21 — count it with
``--since 2026-10-21``). Consuming test: none.

OUTPUTS
-------
A per-bag table and totals on stdout; ``--json`` also writes
``temp/probes/hand_cup_census_<stamp>.json``.

USAGE
-----
    python tools/probes/hand_cup_census.py                       # every bag
    python tools/probes/hand_cup_census.py --since 2026-10-21    # the ASA cup
    python tools/probes/hand_cup_census.py --since 2026-07-29_22 --until 2026-10-21 --json

Run under the project venv (needs ``rosbags``). Exit 0 = done; 1 = no bag read.
"""

from __future__ import annotations

import argparse
import bisect
import collections
import json
import os
import re
import struct
import sys
from datetime import datetime

from rosbags.rosbag2 import Reader

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
_DEFAULT_BAGS = os.path.expanduser('~/Desktop/rosbags')

#: Catch window around the announced landing time (s).
CATCH_LO_S, CATCH_HI_S = -0.1, 0.8
#: A self-throw's ball must leave the cup after release - this (s).
LEFT_LO_S = -0.3
#: Departure-to-stroke lookback (s); covers the ~110-241 ms departure debounce.
DEPARTURE_LOOKBACK_S = 0.5
STROKE_REFRACTORY_S = 0.4
#: The bag's start stamp, wherever it sits in the name (``session_``, ``cone_test_``).
_STAMP_RE = re.compile(r'\d{4}-\d{2}-\d{2}(_\d{2}-\d{2}-\d{2})?')


def _cdr_string(b, o):
    o = (o + 3) & ~3
    n = struct.unpack_from('<I', b, o)[0]
    o += 4
    return b[o:o + n - 1].decode(errors='replace'), o + n


def _time(b, o):
    o = (o + 3) & ~3
    s, ns = struct.unpack_from('<iI', b, o)
    return s + ns * 1e-9, o + 8


def decode_announcement(raw):
    """-> (thrower_name, target_id, throw_time_s, landing_time_s)."""
    b = raw[4:]                                  # skip CDR encapsulation
    _, o = _cdr_string(b, 8)                     # header: stamp, frame_id
    thrower, o = _cdr_string(b, o)
    o = ((o + 7) & ~7) + 48                      # initial_position, initial_velocity
    target, o = _cdr_string(b, o)
    throw_t, o = _time(b, o)
    o = ((o + 7) & ~7) + 8 + 48                  # predicted_tof, landing pos + vel
    if o + 8 > len(b):                           # pre-landing-field layout (2026-02)
        return thrower, target, throw_t, None
    landing_t, _ = _time(b, o)
    return thrower, target, throw_t, landing_t


def decode_hand(raw):
    """-> (vel_meas rev/s, ball_held, ball_held_valid) or None if too short.

    Pre-sensor messages (before 2026-07-29) end at iq_meas: held/valid read
    False, i.e. UNKNOWN.
    """
    b = raw[4:]
    if len(b) < 56:
        return None
    vel = struct.unpack_from('<d', b, 40)[0]     # stamp 8 + 4 doubles
    if len(b) < 59:
        return vel, False, False
    return vel, b[56] == 1, b[58] == 1


def read_bag(path):
    t, hand, anns = [], [], []
    with Reader(path) as r:
        conns = [c for c in r.connections
                 if c.topic in ('/hand_telemetry', '/throw_announcements')]
        for c, ts, raw in r.messages(connections=conns):
            raw = bytes(raw)
            if c.topic == '/throw_announcements':
                anns.append(decode_announcement(raw))
            else:
                h = decode_hand(raw)
                if h is not None:
                    t.append(ts * 1e-9)
                    hand.append(h)
    return t, hand, anns


def analyse(t, hand, anns, stroke_rev_s):
    res = collections.Counter()
    sensor_ok = any(v for _, _, v in hand)
    seen = set()
    for thrower, target, throw_t, landing_t in anns:
        if target != 'jugglebot' or thrower not in ('jugglebot', 'ball_butler'):
            continue
        key = (thrower, round(throw_t, 2))
        if key in seen:
            continue
        seen.add(key)
        kind = 'throws' if thrower == 'jugglebot' else 'feeds'
        res[kind] += 1
        if not sensor_ok or landing_t is None:
            res[kind + '_nosensor'] += 1
            continue
        i0 = bisect.bisect_left(t, throw_t + LEFT_LO_S)
        i1 = bisect.bisect_left(t, landing_t + CATCH_LO_S)
        i2 = bisect.bisect_left(t, landing_t + CATCH_HI_S)
        before, after = hand[i0:i1], hand[i1:i2]
        if not any(v for _, _, v in before + after):
            res[kind + '_unknown'] += 1
            continue
        left = kind == 'feeds' or any(v and not h for _, h, v in before)
        seated = any(v and h for _, h, v in after)
        res[kind + ('_caught' if left and seated else '_missed')] += 1

    last = -1e9
    for i, (vel, _, _) in enumerate(hand):
        if (vel > stroke_rev_s and (i == 0 or hand[i - 1][0] <= stroke_rev_s)
                and t[i] - last > STROKE_REFRACTORY_S):
            res['strokes'] += 1
            last = t[i]
    prev = None
    for i, (_, held, valid) in enumerate(hand):
        if not valid:
            prev = None
            continue
        if prev is True and not held:
            j = bisect.bisect_left(t, t[i] - DEPARTURE_LOOKBACK_S)
            if max(h[0] for h in hand[j:i + 1]) > stroke_rev_s:
                res['sensor_throws'] += 1
        prev = held
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bags', default=_DEFAULT_BAGS)
    ap.add_argument('--since', default='', help='bag-name prefix, inclusive')
    ap.add_argument('--until', default='~', help='bag-name prefix, exclusive')
    ap.add_argument('--stroke-rev-s', type=float, default=50.0,
                    help='upward vel_meas threshold for a throw-speed stroke')
    ap.add_argument('--json', action='store_true')
    args = ap.parse_args(argv)

    total = collections.Counter()
    rows, unreadable = [], []
    for dirpath, _, files in sorted(os.walk(args.bags)):
        if 'metadata.yaml' not in files:
            continue
        name = os.path.basename(dirpath)
        m = _STAMP_RE.search(name)
        stamp = m.group(0) if m else ''
        windowed = bool(args.since) or args.until != '~'
        if (m and not args.since <= stamp < args.until) or (not m and windowed):
            continue
        try:
            res = analyse(*read_bag(dirpath), args.stroke_rev_s)
        except Exception as exc:                 # noqa: BLE001 — report, go on
            unreadable.append((name, repr(exc)[:100]))
            continue
        if res:
            rows.append((name, dict(res)))
            total.update(res)
            print(name, ' '.join('%s=%d' % kv for kv in sorted(res.items())))
    print('\nTOTAL', ' '.join('%s=%d' % kv for kv in sorted(total.items())))
    caught = total['throws_caught'] + total['feeds_caught']
    print('throws %d, catches %d (self %d + feeds %d), bags %d, unreadable %d'
          % (total['throws'], caught, total['throws_caught'],
             total['feeds_caught'], len(rows), len(unreadable)))
    for name, why in unreadable:
        print('  unreadable:', name, why)

    if args.json:
        out_dir = os.path.join(_REPO_ROOT, 'temp', 'probes')
        os.makedirs(out_dir, exist_ok=True)
        out = os.path.join(out_dir, 'hand_cup_census_%s.json'
                           % datetime.now().strftime('%Y%m%dT%H%M%S'))
        with open(out, 'w') as fh:
            json.dump({'args': vars(args), 'total': dict(total),
                       'bags': dict(rows), 'unreadable': unreadable},
                      fh, indent=1)
        print('wrote', out)
    return 0 if rows else 1


if __name__ == '__main__':
    sys.exit(main())
