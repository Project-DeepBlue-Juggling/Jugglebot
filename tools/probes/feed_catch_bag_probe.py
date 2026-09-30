#!/usr/bin/env python3
"""BB-feed catch timing, ground-truthed from a rosbag + the launch log — R5.

WHAT IT DOES
------------
For every Ball Butler reload FEED in one sitting (a ``CATCH-AIM skill 1:
source=schedule ... t_land=`` line in the launch log — one per flown feed; a
``REJECTED_BB(THROW_ABORTED_NOT_SETTLED)`` announcement never reaches skill 1
and so never produces this line, and is excluded automatically, not by a
hand-maintained list):

1. Parses the launch log for that feed's committed landing epoch
   (``t_land_sched``), the reload's hold-tilt cap/result, whether the FEED's
   own catch (skill 1) succeeded (a ``REJECTED_NO_BALL`` at skill 3 between
   this feed and the next means the hand never received a ball -- see
   ``_feed_caught``), and the executor's own (explicitly UNCONVERGED)
   ``RESEND-SKIPPED NO-CONVERGED-FIT skill 1`` estimate, if one fired.
2. Seeks the bag to a window around ``t_land_sched`` and reads
   ``/throw_announcements``, ``/mocap_data``, ``/hand_telemetry`` for that
   window only (``mcap.reader.make_reader(...).iter_decoded_messages(...,
   start_time=, end_time=)`` -- measured ~1.2 s/6.4 s-window on a 427 MB bag
   this far in, versus ~90 s/window for a full non-seeking
   ``mcap_ros2.reader.read_ros2_messages`` pass, which is what the sibling
   probes in this directory use when they don't need per-feed windowing).
3. Picks the matching ``/throw_announcements`` (thrower=ball_butler,
   target=ROBOT_NAME) by nearest predicted landing to ``t_land_sched``.
4. Ground-truths the ball's PHYSICAL release and landing from the RAW
   ``/mocap_data`` marker track, not the tracker's ``/balls`` Kalman state
   (which itself lags 60-200 ms, ``tracking/flight_fit.py``'s own docstring)
   and not the log's RESEND-SKIPPED estimate (explicitly unconverged; see
   ``_decomposition`` below for why it disagrees in SIGN with the ground
   truth here). Release: first 5-sample window that fits a constant-``g``
   free-fall parabola. Landing: descending linear-interpolated crossing of
   ``sites.CATCH_CUP_Z_MM``.
5. Reads the cushioning-stroke dive start/bottom (``/hand_telemetry``
   ``pos_cmd``/``vel_ff_cmd``) and the SEATED transition (``ball_held_valid
   and ball_held``) around the observed landing.

METHOD NOTES (read before trusting a number)
---------------------------------------------
* **Ball identification.** The ball carries no QTM rigid-body label in these
  bags -- every labelled marker (``'Base - N'``, ``'Platform - N'``,
  ``'Ball Butler - N'``) is a stationary reference point. The ball is one of
  several ``label==''`` points per frame. This probe builds a disjoint
  multi-target nearest-neighbour tracker over ALL unlabelled points (a
  45 mm/frame association gate against ~180 Hz sampling) and stitches tracks
  separated by a small time/space gap (<=0.15 s, <=250 mm) -- Ball Butler's
  own throwing arm is *also* an unlabelled, moving marker for the first tens
  of ms after release and briefly steals the correspondence, splitting one
  physical flight into two raw tracks. The stitched track with the largest
  z-excursion in the window is reported as the ball. This heuristic was
  visually audited against raw samples for one feed only (2026-09-30, feed 4)
  -- see ``feed_timing_report.md``'s Method notes for the audit and its
  caveats.
* Release/flight-duration SPLIT carries real method noise (the free-fall fit
  window is 14-47 ms wide per feed on the 2026-09-30 bag) even where their
  SUM (the landing) does not -- see ``_decomposition``.
* Timestamps: ``/mocap_data`` uses ``MocapDataMulti.stamp`` (QTM-clock-synced
  capture time), not bag receipt time (measured 3-10 ms of jitter on the
  2026-09-30 bag). ``/throw_announcements`` and ``/hand_telemetry`` use their
  own header/``timestamp`` fields throughout.

REUSE, NOT REIMPLEMENTATION
----------------------------
``sites.CATCH_CUP_Z_MM`` is imported, not restated (the same convention
``throw_outcome_bag_probe.py`` uses for the same constant).

USAGE
-----
    source ~/Desktop/PDJ_venv/venv/bin/activate
    python tools/probes/feed_catch_bag_probe.py \\
        --bag ~/Desktop/rosbags/2026-09-30_16-20-05 \\
        --log ~/.ros/log/2026-09-30-16-20-04-988735-jetson-2494716/launch.log

Prints the per-feed table and the landing-vs-committed mean/stdev, and writes
``temp/probes/feed_catch_bag_probe_<bag>_<timestamp>.json`` (timestamped so
successive runs in one session accumulate rather than clobber each other).

This underpins ``ball_butler_node.py``'s ``BB_RELEASE_PUSH_LAG_S`` /
``BB_FLIGHT_BIAS_S`` constants and the fail-before/pass-after test in
``tests/ros/test_ball_butler_node.py::TestReleaseLagCorrection`` -- re-run
this probe against a fresh sitting's bag to re-measure them. Promoted from a
one-off scratchpad probe (2026-09-30); the first validation pass against the
2026-09-30 sitting is this promotion's own re-run, recorded in that day's
handoff, not yet a second, independent sitting.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import re
import statistics
import sys
from datetime import datetime

_HERE = os.path.dirname(os.path.abspath(__file__))          # tools/probes
_REPO = os.path.dirname(os.path.dirname(_HERE))
_ROS_PKG = os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot')
sys.path.insert(0, _ROS_PKG)
sys.path.insert(0, _REPO)

from jugglebot.motion.skills import sites                        # noqa: E402

G_MM_S2 = 9810.0
#: The skill stack's ONE catch plane (imported, not restated).
CATCH_PLANE_MM = sites.CATCH_CUP_Z_MM
ROBOT_NAME = 'jugglebot'
THROWER_NAME = 'ball_butler'

TOPICS = ['/throw_announcements', '/mocap_data', '/hand_telemetry']

_CATCH_AIM_RE = re.compile(
    r'CATCH-AIM skill 1: source=(?P<source>\S+) '
    r'landing=\((?P<x>-?[\d.]+), (?P<y>-?[\d.]+), (?P<z>-?[\d.]+)\) mm '
    r't_land=(?P<t_land>[\d.]+)')
_HOLD_TILT_RE = re.compile(
    r'hold tilt capped at (?P<cap>[\d.]+) deg, resulting hold '
    r'(?P<result>[\d.]+) deg')
_RESEND_RE = re.compile(
    r'RESEND-SKIPPED NO-CONVERGED-FIT skill 1: the tracker landing is '
    r'(?P<mm>[\d.]+) mm / (?P<dt>-?[\d.]+) s from the committed aim')
_REJECTED_NO_BALL_RE = re.compile(
    r'self_toss \(reload\) ENDED \(REJECTED_NO_BALL\)')

#: How far past a feed's own CATCH-AIM line to search for its outcome /
#: resend line when it is the LAST feed in the log (no next feed's line to
#: bound the window) -- generous; the events this searches for fire within a
#: few lines of skill 1 ending in every observed sitting.
_TAIL_SEARCH_LINES = 400


def _sec(v) -> float:
    if hasattr(v, 'sec'):
        return float(v.sec) + float(v.nanosec) * 1e-9
    return float(v)


# ─────────────────────────── launch-log parsing ───────────────────────────

def parse_log(log_path):
    """Return one record per flown feed, in log order.

    A "flown" feed is one whose ``CATCH-AIM skill 1: source=schedule`` line
    exists -- an announcement that never reached skill 1 (BB refused the
    throw before release, e.g. ``THROW_ABORTED_NOT_SETTLED``) never produces
    this line, so it is excluded without needing a hand-maintained list of
    line numbers.
    """
    with open(log_path, 'r') as f:
        lines = f.readlines()

    catch_aims = []
    for i, line in enumerate(lines):
        m = _CATCH_AIM_RE.search(line)
        if m and m.group('source') == 'schedule':
            catch_aims.append((i, m))

    feeds = []
    for idx, (i, m) in enumerate(catch_aims):
        line_no = i + 1  # 1-based, matches `grep -n`
        t_land_sched = float(m.group('t_land'))
        x_mm, y_mm = float(m.group('x')), float(m.group('y'))

        # Nearest PRECEDING hold-tilt line -- the reload compile that
        # installed this feed's schedule (searched backward, unbounded: nothing
        # else emits this text between two consecutive reload compiles).
        hold_cap = hold_result = None
        for j in range(i, -1, -1):
            hm = _HOLD_TILT_RE.search(lines[j])
            if hm:
                hold_cap = float(hm.group('cap'))
                hold_result = float(hm.group('result'))
                break

        next_i = catch_aims[idx + 1][0] if idx + 1 < len(catch_aims) else \
            min(i + _TAIL_SEARCH_LINES, len(lines))
        window = lines[i:next_i]
        feed_caught = not any(_REJECTED_NO_BALL_RE.search(w) for w in window)
        resend_mm = resend_dt_s = None
        for w in window:
            rm = _RESEND_RE.search(w)
            if rm:
                resend_mm = float(rm.group('mm'))
                resend_dt_s = float(rm.group('dt'))
                break

        feeds.append(dict(
            idx=idx + 1, line=line_no, t_land_sched=t_land_sched,
            x_mm=x_mm, y_mm=y_mm, hold_cap=hold_cap, hold_result=hold_result,
            feed_caught=feed_caught, resend_mm=resend_mm,
            resend_dt_s=resend_dt_s,
        ))
    return feeds


# ─────────────────────────── bag reading (seeked) ──────────────────────────

def _bag_files(bag_path):
    if os.path.isdir(bag_path):
        paths = sorted(glob.glob(os.path.join(bag_path, '*.mcap')))
        if not paths:
            raise SystemExit('no .mcap under %s' % bag_path)
        return paths
    return [bag_path]


def read_window(bag_path, lo, hi):
    """One seeked pass over ``[lo, hi]`` (epoch s) across every .mcap file
    that makes up the bag. Returns ``{topic: [(t, ros_msg), ...]}``, sorted."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    out = {t: [] for t in TOPICS}
    for path in _bag_files(bag_path):
        r = make_reader(open(path, 'rb'), decoder_factories=[DecoderFactory()])
        for schema, ch, msg, ros in r.iter_decoded_messages(
                topics=TOPICS, start_time=int(lo * 1e9), end_time=int(hi * 1e9)):
            t = msg.log_time * 1e-9
            if ch.topic == '/mocap_data':
                st = _sec(ros.stamp)
                if st > 0:
                    t = st
            out[ch.topic].append((t, ros))
    for k in out:
        out[k].sort(key=lambda row: row[0])
    return out


def pick_announcement(anns, t_land_sched):
    """Nearest ``/throw_announcements`` (thrower=ball_butler,
    target=ROBOT_NAME) whose ``throw_time + predicted_tof_sec`` is closest to
    the schedule's committed ``t_land``."""
    best, best_err = None, None
    for _t_recv, msg in anns:
        if msg.thrower_name != THROWER_NAME or msg.target_id != ROBOT_NAME:
            continue
        t_rel = _sec(msg.throw_time)
        t_land = t_rel + float(msg.predicted_tof_sec)
        err = abs(t_land - t_land_sched)
        if best_err is None or err < best_err:
            best, best_err = msg, err
    return best, best_err


def _dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def ball_track(mocap, t_lo, t_hi, gate_mm=45.0):
    """Disjoint multi-target nearest-neighbour correspondence across ALL
    ``label==''`` markers, gap-stitched, largest-z-excursion track reported
    as the ball. See the module docstring's Method notes."""
    frames = []
    for t, msg in mocap:
        pts = [(mk.position.x, mk.position.y, mk.position.z)
               for mk in msg.markers if mk.label == '']
        if pts:
            frames.append((t, pts))
    frames.sort(key=lambda r: r[0])
    if not frames:
        return []

    tracks = []
    for t, pts in frames:
        used = set()
        for tr in tracks:
            if t - tr['last_t'] > 0.05:
                continue
            best_j, best_d = None, None
            for j, p in enumerate(pts):
                if j in used:
                    continue
                d = _dist(p, tr['last_pos'])
                if best_d is None or d < best_d:
                    best_j, best_d = j, d
            if best_j is not None and best_d <= gate_mm:
                used.add(best_j)
                tr['last_t'], tr['last_pos'] = t, pts[best_j]
                tr['points'].append((t, pts[best_j]))
        for j, p in enumerate(pts):
            if j in used:
                continue
            tracks.append(dict(last_t=t, last_pos=p, points=[(t, p)]))

    if not tracks:
        return []

    def start(tr):
        return tr['points'][0]

    def end(tr):
        return tr['points'][-1]

    changed = True
    while changed:
        changed = False
        n = len(tracks)
        best = None
        for i in range(n):
            e_t, e_p = end(tracks[i])
            for j in range(n):
                if i == j:
                    continue
                s_t, s_p = start(tracks[j])
                gap = s_t - e_t
                if 0 < gap <= 0.15 and _dist(e_p, s_p) <= 250.0:
                    if best is None or gap < best[0]:
                        best = (gap, i, j)
        if best is not None:
            _, i, j = best
            tracks[i]['points'] = tracks[i]['points'] + tracks[j]['points']
            del tracks[j]
            changed = True

    best = max(tracks, key=lambda tr: (max(z for _, (_, _, z) in tr['points'])
                                        - min(z for _, (_, _, z) in tr['points'])))
    return sorted(best['points'])


def _solve3(M, b):
    import copy
    M = copy.deepcopy(M)
    b = list(b)
    n = 3
    for i in range(n):
        piv = max(range(i, n), key=lambda r: abs(M[r][i]))
        if abs(M[piv][i]) < 1e-9:
            raise ZeroDivisionError
        M[i], M[piv] = M[piv], M[i]
        b[i], b[piv] = b[piv], b[i]
        for r in range(i + 1, n):
            f = M[r][i] / M[i][i]
            for c in range(i, n):
                M[r][c] -= f * M[i][c]
            b[r] -= f * b[i]
    x = [0.0, 0.0, 0.0]
    for i in reversed(range(n)):
        s = b[i] - sum(M[i][c] * x[c] for c in range(i + 1, n))
        x[i] = s / M[i][i]
    return x


def find_release(track, t_ann_release, window_n=5, accel_tol=1500.0, resid_tol=10.0):
    """First time (>= ``t_ann_release - 0.15 s``) a sliding window of
    ``window_n`` consecutive samples fits a constant ``-g`` parabola.
    Returns ``(t_release, window_span_s)`` or ``(None, None)``."""
    ts = [t for t, _ in track]
    zs = [p[2] for _, p in track]
    n = len(ts)
    for i in range(n - window_n + 1):
        w_t = ts[i:i + window_n]
        w_z = zs[i:i + window_n]
        if w_t[0] < t_ann_release - 0.15:
            continue
        t0 = w_t[0]
        tau = [tt - t0 for tt in w_t]
        A = [[1.0, tt, 0.5 * tt * tt] for tt in tau]
        ATA = [[sum(A[k][r] * A[k][c] for k in range(window_n)) for c in range(3)]
               for r in range(3)]
        ATb = [sum(A[k][r] * w_z[k] for k in range(window_n)) for r in range(3)]
        try:
            a, b, c = _solve3(ATA, ATb)
        except ZeroDivisionError:
            continue
        resid = [a + b * tt + 0.5 * c * tt * tt - zz for tt, zz in zip(tau, w_z)]
        rms = math.sqrt(sum(r * r for r in resid) / window_n)
        if abs(c - (-G_MM_S2)) < accel_tol and rms < resid_tol:
            return w_t[0], (w_t[-1] - w_t[0])
    return None, None


def find_landing_crossing(track, t_after):
    """Descending crossing of ``CATCH_PLANE_MM`` after ``t_after``,
    linear-interpolated between straddling raw samples."""
    pts = [(t, p) for t, p in track if t >= t_after]
    for (t0, p0), (t1, p1) in zip(pts, pts[1:]):
        if p0[2] >= CATCH_PLANE_MM > p1[2]:
            frac = (p0[2] - CATCH_PLANE_MM) / (p0[2] - p1[2])
            tc = t0 + frac * (t1 - t0)
            xc = p0[0] + frac * (p1[0] - p0[0])
            yc = p0[1] + frac * (p1[1] - p0[1])
            return tc, xc, yc
    return None, None, None


def find_seat(hand, t_after, search_lead=0.05):
    """First ``/hand_telemetry`` sample with ``ball_held_valid and
    ball_held`` at or after ``t_after - search_lead``."""
    for _t, msg in hand:
        ts = _sec(msg.timestamp)
        if ts < t_after - search_lead:
            continue
        if bool(msg.ball_held_valid) and bool(msg.ball_held):
            return ts
    return None


def find_dive(hand, t_land, half_window=0.5):
    """Hand ``pos_cmd`` extremum (bottom of the cushioning stroke) nearest
    ``t_land``, and the ``vel_ff_cmd`` sign-flip before it (stroke start)."""
    win = [(_sec(msg.timestamp), msg.pos_cmd, msg.vel_ff_cmd)
           for _t, msg in hand if abs(_sec(msg.timestamp) - t_land) < half_window]
    win.sort(key=lambda r: r[0])
    if len(win) < 3:
        return None, None, None, None
    ts = [r[0] for r in win]
    ps = [r[1] for r in win]
    vs = [r[2] for r in win]
    extrema = [i for i in range(1, len(ps) - 1)
               if (ps[i] - ps[i - 1]) * (ps[i + 1] - ps[i]) < 0]
    if not extrema:
        i_bot = min(range(len(ts)), key=lambda i: abs(ts[i] - t_land))
    else:
        i_bot = min(extrema, key=lambda i: abs(ts[i] - t_land))
    t_bottom, pos_bottom = ts[i_bot], ps[i_bot]
    sign_bot = 1 if vs[i_bot] >= 0 else -1
    t_start = None
    for i in range(i_bot - 1, -1, -1):
        s = 1 if vs[i] >= 0 else -1
        if s != sign_bot:
            t_start = ts[i + 1]
            break
    return t_start, t_bottom, pos_bottom, sign_bot


# ────────────────────────────── per-feed run ───────────────────────────────

def run_feed(bag_path, feed):
    t_land_sched = feed['t_land_sched']
    lo, hi = t_land_sched - 4.8, t_land_sched + 1.6
    data = read_window(bag_path, lo, hi)

    rec = dict(feed)
    ann, ann_err = pick_announcement(data['/throw_announcements'], t_land_sched)
    if ann is None:
        rec['error'] = 'no matching /throw_announcements in window'
        return rec

    t_release_ann = _sec(ann.throw_time)
    tof_ann = float(ann.predicted_tof_sec)
    t_land_ann = t_release_ann + tof_ann

    track = ball_track(data['/mocap_data'], lo, hi)
    t_release_obs, span = find_release(track, t_release_ann)
    if t_release_obs is not None:
        t_cross, x_cross, y_cross = find_landing_crossing(track, t_release_obs + 0.05)
    elif track:
        t_cross, x_cross, y_cross = find_landing_crossing(track, t_release_ann)
    else:
        t_cross = x_cross = y_cross = None

    t_seat = find_seat(data['/hand_telemetry'], t_cross if t_cross else t_land_sched)
    t_dive_start, t_dive_bottom, pos_bottom, _sign = find_dive(
        data['/hand_telemetry'], t_cross if t_cross else t_land_sched)

    rec.update(dict(
        t_release_ann=t_release_ann, tof_ann=tof_ann, t_land_ann=t_land_ann,
        ann_match_err_s=ann_err, n_track=len(track),
        t_release_obs=t_release_obs, release_fit_window_s=span,
        t_land_obs=t_cross, x_land_obs=x_cross, y_land_obs=y_cross,
        t_seat=t_seat, t_dive_start=t_dive_start, t_dive_bottom=t_dive_bottom,
        pos_cmd_bottom=pos_bottom,
    ))
    return rec


# ─────────────────────────────── reporting ─────────────────────────────────

def _mean_stdev(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None, None, 0
    mean = statistics.mean(vals)
    stdev = statistics.pstdev(vals) if len(vals) > 1 else 0.0
    return mean, stdev, len(vals)


def print_table(results):
    hdr = ('%3s %6s %6s %3s %14s %8s %8s %8s %14s %8s %8s %8s' % (
        '#', 'line', 'hold', 'C', 't_land_sched', 'a.ann', 'b.rel', 'c.land',
        '(x,y)mm', 'd.held', 'e.strt', 'e.bot'))
    print(hdr)
    for r in results:
        if r.get('error'):
            print('%3d %6d  -- err: %s' % (r['idx'], r['line'], r['error']))
            continue
        xy = ('--' if r['x_land_obs'] is None else
              '(%+.0f,%+.0f)' % (r['x_land_obs'], r['y_land_obs']))

        def _ms(key, ref):
            v = r.get(key)
            return '--' if v is None or ref is None else '%+.1f' % ((v - ref) * 1e3)

        print('%3d %6d %6.1f %3s %14.3f %8s %8s %8s %14s %8s %8s %8s' % (
            r['idx'], r['line'], r['hold_result'] or 0.0,
            'Y' if r['feed_caught'] else 'N', r['t_land_sched'],
            _ms('t_land_ann', r['t_land_sched']),
            _ms('t_release_obs', r['t_release_ann']),
            _ms('t_land_obs', r['t_land_sched']),
            xy,
            _ms('t_seat', r['t_land_obs']),
            _ms('t_dive_start', r['t_land_obs']),
            _ms('t_dive_bottom', r['t_land_obs']),
        ))


def print_decomposition(results):
    """Reproduces ``feed_timing_report.md``'s Decomposition table: release
    timing + flight duration vs. their sum, the landing lateness."""
    release = [(r['t_release_obs'] - r['t_release_ann']) for r in results
               if r.get('t_release_obs') is not None]
    flight = [((r['t_land_obs'] - r['t_release_obs']) - r['tof_ann']) for r in results
              if r.get('t_land_obs') is not None and r.get('t_release_obs') is not None]
    landing = [(r['t_land_obs'] - r['t_land_sched']) for r in results
               if r.get('t_land_obs') is not None]

    print('\nDecomposition (ms):')
    for label, vals in (('(1) release timing', release),
                        ('(2) flight duration', flight),
                        ('(1)+(2) = landing', landing)):
        mean, stdev, n = _mean_stdev(vals)
        if mean is None:
            print('  %-22s n=0' % label)
        else:
            print('  %-22s mean=%+7.1f  stdev=%6.1f  n=%d'
                  % (label, mean * 1e3, stdev * 1e3, n))
    return _mean_stdev(landing)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', required=True,
                     help='rosbag2/mcap directory or a single .mcap file')
    ap.add_argument('--log', required=True, help='launch.log path')
    args = ap.parse_args(argv)

    bag_path = os.path.expanduser(args.bag)
    log_path = os.path.expanduser(args.log)

    feeds = parse_log(log_path)
    if not feeds:
        raise SystemExit('no "CATCH-AIM skill 1: source=schedule" lines found in %s'
                          % log_path)
    print('%d flown feed(s) found in %s' % (len(feeds), log_path))

    results = [run_feed(bag_path, feed) for feed in feeds]
    print_table(results)
    mean, stdev, n = print_decomposition(results)
    if mean is not None:
        print('\nlanding vs. committed: mean=%+.1f ms  stdev=%.1f ms  (n=%d)'
              % (mean * 1e3, stdev * 1e3, n))

    out_dir = os.path.join(_REPO, 'temp', 'probes')
    os.makedirs(out_dir, exist_ok=True)
    bag_tag = os.path.basename(bag_path.rstrip('/'))
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    out_path = os.path.join(
        out_dir, 'feed_catch_bag_probe_%s_%s.json' % (bag_tag, stamp))
    with open(out_path, 'w') as f:
        json.dump(dict(
            bag=bag_path, log=log_path, feeds=results,
            landing_vs_committed_mean_s=mean, landing_vs_committed_stdev_s=stdev,
            landing_vs_committed_n=n,
        ), f, indent=2)
    print('\nwrote %s' % out_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
