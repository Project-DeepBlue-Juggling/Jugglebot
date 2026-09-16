#!/usr/bin/env python3
"""Replay THROW outcome capture from a bag: what row would the NEW rule write?

WHY
---
On 2026-09-16 the 16:22 apex-ladder sitting wrote memory rows with observed
flights of 2.2317 / 2.2310 / 1.1554 s for an 0.8569 s commanded flight — a
0.9 m toss cannot fly for longer than ~0.86 s. The executor's
``_advance_outcomes`` refreshed ``best_landing`` on EVERY tick whose estimate
cleared ``OUTCOME_GUARD_S``, so the LAST estimate before the row finalised
won; in a chained self-toss the same ball is caught and re-thrown on ONE
continuous tracker track, so after the catch that id's estimate is about the
NEXT flight (or, while the ball sits in the cup, about "now").

This probe reconstructs, from the bag's ``/balls`` stream and the memory CSVs
the sitting actually wrote, what the row WOULD have been under the new rule
(``executor._consider_landing``: the LAST admissible estimate wins, where
admissible means sampled strictly before the crossing it predicts, before the
ball has landed, before this ball's next release, and inside
``memory.FLIGHT_RATIO_BAND``) — and prints old vs new side by side per throw.

WHAT IT ASSUMES (read before trusting a number)
-----------------------------------------------
* The commanded release instants and commanded flights come from the memory
  CSVs themselves (``t_abs_s`` IS the throw's release, ``u[2]`` its commanded
  flight), so only throws that WROTE a row appear here. A throw whose row was
  dropped (no landing, no release evidence) is invisible to this probe.
* The executor samples the tracker on its own 40 Hz tick; this replay samples
  it at every ``/balls`` message instead (a superset of the tick instants, at
  the same or finer cadence), so a new-rule pick can be a few ms sharper than
  the live executor's would have been. The PICK — which flight the estimate
  belongs to — is not sensitive to that.
* The tracker id is chosen per row as the one whose stream best reproduces
  the row that was actually written (``old_repro``); when no id reproduces it
  the row is reported as ``?`` and the new value is not trusted.
* Clock: ``/balls`` messages are placed at their own ``header.stamp`` (the
  same ROS epoch the memory rows' ``t_abs_s`` is on) and the landing estimate
  is each ``BallState``'s own ``time_at_land``.

USAGE
-----
    python tools/probes/outcome_landing_replay.py \\
        --bag ~/Desktop/rosbags/2026-09-16_16-22-22 \\
        --bag ~/Desktop/rosbags/2026-09-16_14-16-38 \\
        --learn temp/learn --date 20260916
"""

from __future__ import annotations

import argparse
import csv
import glob
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))           # tools/probes
_REPO = os.path.dirname(os.path.dirname(_HERE))
_PKG_DIR = os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot')
sys.path.insert(0, _PKG_DIR)
sys.path.insert(0, os.path.join(_REPO, 'config', 'generated'))

import jugglebot                                            # noqa: E402
# A colcon-INSTALLED `jugglebot` on the path would otherwise shadow the live
# tree and this probe would read different constants than the ones under test.
if os.path.join(_PKG_DIR, 'jugglebot') not in jugglebot.__path__:
    jugglebot.__path__.insert(0, os.path.join(_PKG_DIR, 'jugglebot'))

from jugglebot.motion.skills import executor as ex        # noqa: E402
from jugglebot.motion.skills import memory as mem          # noqa: E402

#: A row is part of the same ATTEMPT as the previous one when their releases
#: are closer than this (the sitting's chained beats were 0.94–1.16 s; a new
#: attempt is tens of seconds later).
ATTEMPT_GAP_S = 5.0


def _sec(v) -> float:
    if hasattr(v, 'sec'):
        return float(v.sec) + float(v.nanosec) * 1e-9
    return float(v)


def read_balls(bag_dir):
    """``{tracker_id: [(t_sample_s, t_land_s, (x, y, z)_mm), ...]}``."""
    from mcap_ros2.reader import read_ros2_messages

    paths = [os.path.join(bag_dir, f) for f in sorted(os.listdir(bag_dir))
             if f.endswith('.mcap')]
    if not paths:
        raise SystemExit('no .mcap in %s' % bag_dir)
    out = {}
    for path in paths:
        for m in read_ros2_messages(path, topics=['/balls']):
            msg = m.ros_msg
            t = _sec(msg.header.stamp) if hasattr(msg, 'header') else \
                m.log_time_ns * 1e-9
            for b in msg.balls:
                out.setdefault(int(b.id), []).append(
                    (t, _sec(b.time_at_land),
                     (float(b.landing_position.x), float(b.landing_position.y),
                      float(b.landing_position.z))))
    for rows in out.values():
        rows.sort(key=lambda r: r[0])
    return out


def read_rows(learn_dir, date):
    """``[(plant_id, attempt, throw, t_release, u_flight, y_flight, caught)]``."""
    out = []
    for path in sorted(glob.glob(os.path.join(learn_dir, '*-%s' % date,
                                               'memory.csv'))):
        plant_id = os.path.basename(os.path.dirname(path))
        with open(path, newline='') as handle:
            reader = csv.reader(handle)
            next(reader, None)
            raw = [(float(r[10]), float(r[6]), float(r[9]),
                    r[12].strip().lower() in ('true', '1'))
                   for r in reader if len(r) == 13]
        raw.sort()
        attempt = 0
        throw = 0
        prev = None
        for t_rel, u_flight, y_flight, caught in raw:
            if prev is None or t_rel - prev > ATTEMPT_GAP_S:
                attempt += 1
                throw = 0
            throw += 1
            prev = t_rel
            out.append((plant_id, attempt, throw, t_rel, u_flight, y_flight,
                        caught))
    return out


def old_pick(stream, t_release, t_sched, finalise_at):
    """The PRE-FIX rule: the last estimate before ``finalise_at`` that
    post-dates the release and is outside ``OUTCOME_GUARD_S`` of its own
    predicted crossing (``abs()``, so post-crossing samples counted too)."""
    best = None
    for t_s, t_land, pos in stream:
        if t_s < t_release or t_s > finalise_at:
            continue
        if t_land <= t_release:
            continue
        if abs(t_s - t_land) <= ex.OUTCOME_GUARD_S:
            continue
        best = (t_land, pos)
    return best


def new_pick(stream, t_release, t_sched, finalise_at, t_next_release=None):
    """The NEW rule, through the executor's own ``_consider_landing``."""
    import numpy as np

    pend = ex._PendingOutcome(
        ball_id=0, x=np.zeros(4), u=np.array([0.0, 0.0, t_sched - t_release]),
        t_release_s=t_release, t_land_scheduled_s=t_sched,
        target_xy_mm=np.zeros(2), t_next_release_s=t_next_release)
    for t_s, t_land, pos in stream:
        if t_s < t_release or t_s > finalise_at:
            continue
        ex.SkillExecutor._consider_landing(
            pend, ex.Landing(pos_mm=np.asarray(pos, dtype=float),
                             vel_mm_s=np.zeros(3), t_land_abs_s=t_land), t_s)
    if pend.best_landing is None:
        return None
    return (float(pend.best_landing.t_land_abs_s),
            tuple(pend.best_landing.pos_mm))


def replay(balls, rows):
    """One output record per memory row."""
    out = []
    by_plant = {}
    for r in rows:
        by_plant.setdefault(r[0], []).append(r)
    for plant_id, plant_rows in by_plant.items():
        for i, (_p, attempt, throw, t_rel, u_flight, y_old, caught) \
                in enumerate(plant_rows):
            t_sched = t_rel + u_flight
            nxt = None
            if i + 1 < len(plant_rows):
                cand = plant_rows[i + 1][3]
                if cand - t_rel < ATTEMPT_GAP_S:
                    nxt = cand
            # The live finalise instants, from the executor's own arithmetic.
            close_old = t_sched + ex.CAUGHT_LAND_DEFER_CAP_S + ex.CAUGHT_WINDOW_S
            close_new = close_old if nxt is None else min(
                close_old, nxt - ex.OUTCOME_NEXT_RELEASE_EPS_S)
            # Pick the tracker id that best reproduces the row as written.
            best_id, best_err, best_old = None, None, None
            for bid, stream in balls.items():
                got = old_pick(stream, t_rel, t_sched, close_old)
                if got is None:
                    continue
                err = abs((got[0] - t_rel) - y_old)
                if best_err is None or err < best_err:
                    best_id, best_err, best_old = bid, err, got
            new = (None if best_id is None
                   else new_pick(balls[best_id], t_rel, t_sched, close_new,
                                 t_next_release=nxt))
            y_new = None if new is None else new[0] - t_rel
            admitted = (y_new is not None
                        and mem.flight_in_band(u_flight, y_new))
            out.append(dict(
                plant_id=plant_id, attempt=attempt, throw=throw,
                commanded=u_flight, y_old=y_old, y_new=y_new,
                admitted=admitted, caught=caught, tracker_id=best_id,
                old_repro=best_err,
                old_y_replayed=(None if best_old is None
                                else best_old[0] - t_rel)))
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bag', action='append', required=True)
    ap.add_argument('--learn', default=os.path.join(_REPO, 'temp', 'learn'))
    ap.add_argument('--date', required=True)
    args = ap.parse_args(argv)

    balls = {}
    for bag in args.bag:
        for bid, rows in read_balls(os.path.expanduser(bag)).items():
            balls.setdefault(bid, []).extend(rows)
    for rows in balls.values():
        rows.sort(key=lambda r: r[0])

    recs = replay(balls, read_rows(os.path.expanduser(args.learn), args.date))
    print('%-14s %3s %3s %10s %10s %10s %6s %8s %6s'
          % ('plant_id', 'att', 'thr', 'commanded', 'y_old', 'y_new',
             'admit', 'repro_ms', 'trk'))
    for r in recs:
        print('%-14s %3d %3d %10.4f %10.4f %10s %6s %8s %6s'
              % (r['plant_id'], r['attempt'], r['throw'], r['commanded'],
                 r['y_old'],
                 '--' if r['y_new'] is None else '%.4f' % r['y_new'],
                 'yes' if r['admitted'] else 'NO ROW',
                 '--' if r['old_repro'] is None
                 else '%.1f' % (r['old_repro'] * 1e3),
                 '--' if r['tracker_id'] is None else r['tracker_id']))
    n_adm = sum(1 for r in recs if r['admitted'])
    print('\n%d rows replayed; the new rule admits %d and drops %d'
          % (len(recs), n_adm, len(recs) - n_adm))
    ratios = [r['y_new'] / r['commanded'] for r in recs if r['y_new']]
    if ratios:
        print('new y/u ratios: %.3f .. %.3f' % (min(ratios), max(ratios)))
    old_ratios = [r['y_old'] / r['commanded'] for r in recs]
    print('old y/u ratios: %.3f .. %.3f' % (min(old_ratios), max(old_ratios)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
