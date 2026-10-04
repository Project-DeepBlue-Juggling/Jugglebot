#!/usr/bin/env python3
"""Ball Butler's feed-landing bias: where its fed ball actually lands
relative to the point it was asked to hit (schedule frame) -- the number
behind `skill_node.py`'s `columns_feed_bb_bias_mm` parameter.

METHOD
------
For every flown reload FEED (a `CATCH-AIM skill 1: source=schedule
landing=(...) mm t_land=...` log line from a `reload=True`, non-
`columns_1ball` attempt -- `columns`/`columns_1ball_fed`'s own feed catch;
a throw Ball Butler refuses before release, e.g. `THROW_ABORTED_
NOT_SETTLED`, never reaches skill 1 and is excluded automatically): seeks
the bag to a window around the committed landing epoch, free-fall-fits the
raw `label==''` `/mocap_data` ball track (robust to rim hits / occlusion;
the method `a2_feed_lateral.py` used in the 2026-10-04 R5 sitting 4
analysis -- a disjoint nearest-neighbour tracker seeded near the Platform
body, grown both ways with constant-velocity prediction), and compares the
fitted landing xy at the free-fall crossing of the catch plane
(`sites.CATCH_CUP_Z_MM`, restated as `PLANE_MM` to avoid a ROS2 import)
against the Platform body's own mocap xy at that same instant.

FRAME AND SIGN CONVENTION (settled from the raw L3/L4 bags, R5 sitting 4,
2026-10-04 -- report_a2.md and report_a3.md disagreed on the sign; this is
the one this probe and `columns_feed_bb_bias_mm` both use):
`skill_node._on_balls`/`_mocap_aim_point_mm` map mocap <-> schedule frame
by a PURE x/y translation (`landing - offset_mm`, `cup_mm + offset_mm` --
no rotation; grep both), so a DIFFERENCE between two points reads the same
in either frame. The Platform body's cup sits at the feed's committed
REQUEST point during the catch (`skill_node._columns_feed_aim_site`'s own
point -- the point actually sent to Ball Butler, not the un-walked feed
site), so `landing_mocap_xy - platform_body_mocap_xy` IS `bias = landing -
request`, in both frames, no further correction needed. **+x points from
the feed site TOWARD ball A's site** (the direction
`columns_feed_aim_toward_a_mm` walks Ball Butler's own aim); **+y is the
shared mocap/schedule y axis**. A positive bias means Ball Butler's ball
lands PAST the point it was asked to hit, in that direction.

PASS CRITERION: median |dx|, |dy| <= 10 mm over >= 10 feeds, AND >= 70% of
feeds SEATED (first debounced `ball_held_valid and ball_held` on
`/hand_telemetry`) before `t_land + 0.19 s` (the hand's own reversal into
the next throw -- `report_a2.md`'s own threshold).

RECOMMENDED: the bias that was LIVE during the run (read from
skill_node's own `columns feed bias: request (...) mm, bias [...] mm`
INFO line, or `--bias-in-force BX BY`; 0,0 if neither is available --
true for every sitting before this parameter existed) PLUS the newly
measured bias: the `columns_feed_bb_bias_mm` VALUE that would have
cancelled what was actually measured this run.

USAGE
-----
    python tools/probes/feed_lateral_miss.py \\
        --run L3 ~/.ros/log/<dir>/launch.log ~/Desktop/rosbags/<bag> \\
        --run L4 ~/.ros/log/<dir2>/launch.log ~/Desktop/rosbags/<bag2> \\
        [--bias-in-force BX BY] [--out temp/probes/feed_lateral_miss_<tag>.log]

One result line per `--run` tag (each run's feeds are kept separate --
`report_a2.md`'s own warning: a statistic pooled across sittings/launches
with different corrections in force is not a physical quantity). Strictly
offline and read-only (`mcap_ros2` decode only -- no ROS2 node, no
hardware). A 400+ MB bag takes a few minutes; run in the background and
read the output by path.
"""
from __future__ import annotations

import argparse
import glob
import math
import os
import re
import sys

import numpy as np

G_MM_S2 = 9810.0
PLANE_MM = 830.0  # sites.CATCH_CUP_Z_MM, restated to avoid a ROS2 import
SEAT_DEADLINE_S = 0.19

TOPICS = ['/mocap_data', '/hand_telemetry', '/rigid_body_poses']

_RE_LINE = re.compile(
    r'^(\d+\.\d+) \[skill_node-\d+\] \[(\w+)\] \[[\d.]+\] \[skill_node\]: (.*)$')
_RE_AIM = re.compile(
    r'CATCH-AIM skill (\d+): source=(\S+) landing=\((-?[\d.]+), (-?[\d.]+), '
    r'(-?[\d.]+)\) mm t_land=([\d.]+)')
_RE_BIAS = re.compile(
    r'columns feed bias: request \((-?[\d.]+), (-?[\d.]+)\) mm, bias '
    r'\[([+-][\d.]+), ([+-][\d.]+)\] mm')


def _sec(t):
    return t.sec + 1e-9 * t.nanosec


def parse_feeds(log_path):
    """Every flown FEED catch (skill 1 of a `reload=True`, non-
    `columns_1ball` attempt): dicts with `t_land`, `req_x`, `req_y`,
    `bias_in_force` (the most recent `columns feed bias` INFO line at or
    before this feed, or ``None`` if the log predates that line)."""
    feeds = []
    cur = None
    bias_seen = None
    for line in open(log_path, errors='replace'):
        m = _RE_LINE.match(line.strip())
        if not m:
            continue
        _t, lvl, txt = float(m.group(1)), m.group(2), m.group(3)
        bm = _RE_BIAS.search(txt)
        if bm:
            bias_seen = (float(bm.group(3)), float(bm.group(4)))
        if lvl == 'INFO' and txt.startswith('columns') and ' started' in txt.split(':')[0]:
            head = txt.split(':')[0]
            cur = dict(reload='(reload)' in head, oneball='columns_1ball' in head,
                      seen_skill1=False)
            continue
        if cur is None or not cur['reload'] or cur['oneball']:
            continue
        a = _RE_AIM.search(txt)
        if a and int(a.group(1)) == 1 and not cur['seen_skill1']:
            cur['seen_skill1'] = True
            feeds.append(dict(t_land=float(a.group(6)), req_x=float(a.group(3)),
                              req_y=float(a.group(4)), bias_in_force=bias_seen))
    return feeds


def read_window(bag_dir, lo, hi):
    """One seeked pass over ``[lo, hi]`` (epoch s) -- `/mocap_data` (raw
    unlabelled markers), `/hand_telemetry` (held/valid only), `/rigid_body_
    poses` (the Platform body)."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    mocap, hand, body = [], [], []
    for path in sorted(glob.glob(os.path.join(bag_dir, '*.mcap'))):
        r = make_reader(open(path, 'rb'), decoder_factories=[DecoderFactory()])
        for _s, ch, msg, ros in r.iter_decoded_messages(
                topics=TOPICS, start_time=int(lo * 1e9), end_time=int(hi * 1e9)):
            if ch.topic == '/mocap_data':
                t = _sec(ros.stamp) or msg.log_time * 1e-9
                pts = [(mk.position.x, mk.position.y, mk.position.z)
                       for mk in ros.markers if mk.label == '']
                mocap.append((t, np.array(pts).reshape(-1, 3)))
            elif ch.topic == '/hand_telemetry':
                hand.append((_sec(ros.timestamp), int(ros.ball_held),
                            int(ros.ball_held_valid)))
            else:
                for b in ros.bodies:
                    if b.name == 'Platform':
                        p = b.pose.pose.position
                        t = _sec(ros.header.stamp) or msg.log_time * 1e-9
                        body.append((t, p.x, p.y, p.z))
    mocap.sort(key=lambda row: row[0])
    hand_arr = np.array(sorted(hand)) if hand else np.zeros((0, 3))
    body_arr = np.array(sorted(body)) if body else np.zeros((0, 4))
    return mocap, hand_arr, body_arr


def track_ball(mocap, t_land, xy0, xy_gate=220.0):
    """Nearest-point ball track seeded near ``xy0`` in ``[t_land - 0.15,
    t_land + 0.05]``, grown both ways with constant-velocity prediction --
    `a2_feed_lateral.py`'s method, verbatim (R5 sitting 4 scratchpad)."""
    seed = None
    for t, pts in mocap:
        if t < t_land - 0.15 or t > t_land + 0.05 or len(pts) == 0:
            continue
        d = np.hypot(pts[:, 0] - xy0[0], pts[:, 1] - xy0[1])
        ok = (d < xy_gate) & (pts[:, 2] > 900) & (pts[:, 2] < 1350)
        if ok.any():
            j = int(np.argmin(np.where(ok, np.abs(pts[:, 2] - 1000), 1e9)))
            seed = (t, pts[j])
            break
    if seed is None:
        return None
    times = [row[0] for row in mocap]
    i0 = times.index(seed[0])
    out = {seed[0]: seed[1]}
    for direction, t_stop in ((1, t_land + 0.55), (-1, t_land - 0.45)):
        hist = [(seed[0], seed[1])]
        vel = np.array([0.0, 0.0, -2500.0 * direction])
        i = i0 + direction
        while 0 <= i < len(mocap):
            t, pts = mocap[i]
            if (t - t_stop) * direction > 0:
                break
            lt, lp = hist[-1]
            if abs(t - lt) > 0.04:
                break
            if len(pts):
                pred = lp + vel * (t - lt)
                d = np.linalg.norm(pts - pred, axis=1)
                j = int(np.argmin(d))
                if d[j] < 35.0:
                    if len(hist) >= 2:
                        vel = (pts[j] - hist[-1][1]) / (t - hist[-1][0])
                    else:
                        vel = (pts[j] - lp) / (t - lt) if t != lt else vel
                    hist.append((t, pts[j]))
                    out[t] = pts[j]
            i += direction
    ts = np.array(sorted(out))
    ps = np.array([out[t] for t in ts])
    return ts, ps


def measure_feed(feed, mocap, hand, body):
    """One feed's ``(dx, dy)`` -- ``None`` if the ball track or the free-
    fall fit never converged (unconverged fits are skipped, not zero-
    filled, same as `a2_feed_lateral.py`)."""
    tl = feed['t_land']
    if len(body) == 0:
        return None
    bt = body[:, 0]
    bx, by = np.interp(tl, bt, body[:, 1]), np.interp(tl, bt, body[:, 2])
    tr = track_ball(mocap, tl, (bx, by), 220.0)
    if tr is None or len(tr[0]) < 8:
        return None
    ts, ps = tr
    pre = (ps[:, 2] > 900) & (ts < tl + 0.1)
    if pre.sum() < 5:
        return None
    k = int(np.argmax(np.where(pre, ps[:, 2], -1e9)))
    sel = pre & (ts >= ts[k])
    if sel.sum() < 5:
        return None
    tp = ts[sel]
    t0 = tp[-1]
    z = ps[sel, 2] + 0.5 * G_MM_S2 * (tp - t0) ** 2
    cz = np.polyfit(tp - t0, z, 1)
    cx = np.polyfit(tp - t0, ps[sel, 0], 1)
    cy = np.polyfit(tp - t0, ps[sel, 1], 1)
    disc = cz[0] ** 2 + 2 * G_MM_S2 * (cz[1] - PLANE_MM)
    if disc <= 0:
        return None
    ta = t0 + (cz[0] + math.sqrt(disc)) / G_MM_S2
    xa, ya = np.polyval(cx, ta - t0), np.polyval(cy, ta - t0)
    pxa, pya = np.interp(ta, bt, body[:, 1]), np.interp(ta, bt, body[:, 2])
    dx, dy = float(xa - pxa), float(ya - pya)
    seat = None
    for row in hand:
        if row[0] >= tl - 0.1 and row[1] and row[2]:
            seat = row[0]
            break
    seated_before = (seat is not None and (seat - tl) < SEAT_DEADLINE_S)
    return dict(dx=dx, dy=dy, r=math.hypot(dx, dy), seated_before=seated_before,
               bias_in_force=feed['bias_in_force'])


def _summarize(tag, rows, bias_in_force_override):
    a = np.array([[r['dx'], r['dy'], r['r']] for r in rows])
    n = len(rows)
    med_dx, med_dy, med_r = (float(np.median(a[:, 0])), float(np.median(a[:, 1])),
                             float(np.median(a[:, 2])))
    seated_n = sum(r['seated_before'] for r in rows)
    seated_frac = seated_n / n
    passed = (abs(med_dx) <= 10.0 and abs(med_dy) <= 10.0 and n >= 10
             and seated_frac >= 0.70)
    result = ('%s n=%d  dx med %+.1f  dy med %+.1f  r med %.1f  '
             'seated-before-%.2fs %.0f%% (%d/%d)  ->  %s'
             % (tag, n, med_dx, med_dy, med_r, SEAT_DEADLINE_S,
                seated_frac * 100, seated_n, n, 'PASS' if passed else 'FAIL'))
    if bias_in_force_override is not None:
        bif = tuple(bias_in_force_override)
    else:
        seen = [r['bias_in_force'] for r in rows if r['bias_in_force'] is not None]
        bif = seen[-1] if seen else (0.0, 0.0)
    rec_x, rec_y = bif[0] + med_dx, bif[1] + med_dy
    rec = ('RECOMMENDED columns_feed_bb_bias_mm = [%+.1f, %+.1f]  (%s: bias in '
          'force [%+.1f, %+.1f] + measured [%+.1f, %+.1f])'
          % (rec_x, rec_y, tag, bif[0], bif[1], med_dx, med_dy))
    return result, rec


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--run', nargs=3, action='append', metavar=('TAG', 'LOG', 'BAG'),
                    required=True, help='one per launch; repeatable')
    ap.add_argument('--bias-in-force', nargs=2, type=float, default=None,
                    metavar=('BX', 'BY'),
                    help='override: the columns_feed_bb_bias_mm value LIVE '
                         'during every --run passed (default: read per-run '
                         'from the log\'s own "columns feed bias" INFO line, '
                         '[0.0, 0.0] if the log predates that line)')
    ap.add_argument('--out', default=None, help='also write the summary here')
    args = ap.parse_args(argv)

    out_lines = []
    for tag, log, bag in args.run:
        feeds = parse_feeds(log)
        print('%s: %d flown feed(s)' % (tag, len(feeds)), flush=True)
        rows = []
        if feeds:
            tls = [f['t_land'] for f in feeds]
            mocap, hand, body = read_window(bag, min(tls) - 0.8, max(tls) + 0.3)
            for f in feeds:
                r = measure_feed(f, mocap, hand, body)
                if r is not None:
                    rows.append(r)
        if not rows:
            line = '%s: NO feeds measured (0 flown or 0 converged fits)' % (tag,)
            print(line)
            out_lines.append(line)
            continue
        result, rec = _summarize(tag, rows, args.bias_in_force)
        print(result)
        print(rec)
        out_lines.append(result)
        out_lines.append(rec)

    if args.out:
        out_dir = os.path.dirname(args.out)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(args.out, 'w') as f:
            f.write('\n'.join(out_lines) + '\n')
        print('wrote', args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
