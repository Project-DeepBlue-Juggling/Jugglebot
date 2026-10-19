#!/usr/bin/env python3
"""robot_frame_bag_check.py - does the robot frame survive a QTM re-alignment? (jugglebot.robot_frame)

What it measures
    Per bag: fits the robot frame (resources/jugglebot_robot_frame.json, no override) on sampled
    /mocap_data frames and reports n frames, fit-OK fraction, median/p99 RMS, the mean origin t and
    rotation (yaw of R; the fit is yaw-only) and the QTM 'Base' 6DOF body's mean pose. Then the INVARIANCE check:
    the mocap 'Platform' body position (/rigid_body_poses; z is published SHIFTED by
    hw.GEOM_INITIAL_HEIGHT_MM, added back here) in WORLD and in the ROBOT frame (each pose is
    transformed with the fit of the nearest preceding mocap frame), over frames where the Platform is
    at rest (position sd < 0.5 mm over 1 s). Bags straddling a QTM re-definition must show the same
    robot-frame rest position while the world one moves. Also fit-origin minus QTM Base-body position
    (a constant ~2 mm offset if the QTM body definition is stable): the fallback evidence when no
    common rest pose exists.

Replay mode (--replay)
    Feeds the PRODUCTION jugglebot.robot_frame.RobotFrameTracker with the bag's /mocap_data frames at
    mocap_node's feed rule (_feed_robot_frame: one frame per ROBOT_FRAME_PERIOD_S = 0.2 s of frame
    time, i.e. 5 Hz) and prints one row per bag and cap: time to first lock, share of feeds with a
    running frame, OK/UNKNOWN transitions (with times), origin / yaw p95 scatter of the running frame
    about its per-position median (positions split at UNKNOWN gaps), share of accepted fits that
    dropped a marker (+ most-dropped label), accepted-fit RMS p50/p99. --caps sweeps max_rms_mm;
    --displace LABEL:MM synthesises one displaced marker (+x) on every frame, for the guard check.
    Defaults for --replay: the three 2026-10-19 bags plus the five earlier Base-labelled bags
    (2026-10-10_14-13-18, 2026-10-11_11-27-53, 2026-10-16_16-23-01, 2026-10-16_22-01-50,
    2026-09-27_14-31-13); --frames 0 = whole bag (the 09-27 bag takes ~20 min x caps; the whole
    default sweep at 3 caps took 35 min on 2026-10-19). The invariance number (fit origin minus QTM
    Base body) is the default mode's second table. Writes temp/probes/robot_frame/<timestamp>/replay.md.

Inputs
    --bags    bag names/dirs (default: the five below); --frames mocap frames read per bag from the
              start (default 6000); --every fit every Nth frame (default 2).

Usage (repo root, project venv)
    python -I tools/probes/robot_frame_bag_check.py [--bags B ...] [--frames 6000] [--every 2]
    python -I tools/probes/robot_frame_bag_check.py --replay [--bags B ...] [--caps 1.5 2.0 2.5]
        [--frames 0] [--displace "Base - 1:5"]
    Writes ``temp/probes/robot_frame/<timestamp>/bag_check.md``.
"""
from __future__ import annotations

import argparse
import datetime
import glob
import math
import os
import sys

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'ros_ws', 'src', 'jugglebot'))
from jugglebot import robot_frame as rf  # noqa: E402
from jugglebot.hardware_config import GEOM_INITIAL_HEIGHT_MM  # noqa: E402

BAG_ROOT = os.path.expanduser('~/Desktop/rosbags')
DEFAULT_BAGS = ['2026-10-10_19-23-29', '2026-10-10_23-32-12', '2026-10-16_16-23-01',
                '2026-10-16_22-01-50', '2026-09-27_14-31-13']
NEW_BAGS = ['2026-10-19_11-04-03', '2026-10-19_11-13-17', '2026-10-19_11-14-54']
EARLIER_BAGS = ['2026-10-10_14-13-18', '2026-10-11_11-27-53', '2026-10-16_16-23-01',
                '2026-10-16_22-01-50', '2026-09-27_14-31-13']
FEED_PERIOD_S = 0.2     # mocap_node.ROBOT_FRAME_PERIOD_S
TEMPLATE = os.path.join(REPO, 'ros_ws', 'src', 'jugglebot', 'resources', 'jugglebot_robot_frame.json')
REST_SD_MM = 0.5


def read(bag, n):
    """(fits [(ns, R, t, rms, ok)], bodies {name: [(ns_of_last_mocap, xyz)]})"""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    d = bag if os.path.isdir(bag) else os.path.join(BAG_ROOT, bag)
    path = sorted(glob.glob(os.path.join(d, '*.mcap')))[0]
    return path


def analyse(bag, tpl, nframes, every):
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    path = read(bag, nframes)
    fits, platform, base_body = [], [], []
    last = None          # (R, t) of the newest mocap frame's fit, or None when it failed
    i = 0
    with open(path, 'rb') as f:
        rd = make_reader(f, decoder_factories=[DecoderFactory()])
        for _s, ch, _m, msg in rd.iter_decoded_messages(topics=['/mocap_data', '/rigid_body_poses']):
            if ch.topic == '/mocap_data':
                if i >= nframes:
                    break
                if i % every == 0:
                    rows = [(mk.label, mk.position.x, mk.position.y, mk.position.z, mk.residual)
                            for mk in msg.markers]
                    ns = msg.stamp.sec * 10 ** 9 + msg.stamp.nanosec
                    ft = rf.fit_robot_frame(rows, tpl, stamp_ns=ns)
                    fits.append((ns, ft.rms_mm, ft.ok))
                    last = ft.frame
                    if ft.ok:
                        fits[-1] = fits[-1] + (ft.frame.R, ft.frame.t)
                i += 1
            else:
                for b in msg.bodies:
                    p = b.pose.pose.position
                    if b.name == 'Platform':
                        platform.append((msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec if hasattr(msg, 'header') else len(platform) / 100.0,
                                         np.array([p.x, p.y, p.z + GEOM_INITIAL_HEIGHT_MM]), last))
                    elif b.name == 'Base':
                        base_body.append((np.array([p.x, p.y, p.z]), last))
    return fits, platform, base_body


def rest_mask(times, pos, win_s=1.0):
    """True where the position sd over the trailing win_s is < REST_SD_MM."""
    n = len(pos)
    ok = np.zeros(n, bool)
    j = 0
    for k in range(n):
        while times[k] - times[j] > win_s:
            j += 1
        if k - j >= 20:
            ok[k] = float(np.max(np.std(pos[j:k + 1], axis=0))) < REST_SD_MM
    return ok


def yaw_deg(R):
    # The fit is yaw-only since 2026-10-19 (R is a pure rotation about world z), so yaw is the whole rotation.
    return math.degrees(math.atan2(R[1, 0], R[0, 0]))


def load_feed_frames(bag, nframes=0):
    """[(t_s, ns, rows)] at mocap_node's feed cadence (>= FEED_PERIOD_S of frame time apart), plus
    [(t_s, xyz)] of the QTM 'Base' body. nframes 0 = whole bag."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    path = read(bag, nframes)
    frames, base, last, i = [], [], None, 0
    with open(path, 'rb') as f:
        rd = make_reader(f, decoder_factories=[DecoderFactory()])
        for _s, ch, _m, msg in rd.iter_decoded_messages(topics=['/mocap_data', '/rigid_body_poses']):
            if ch.topic == '/rigid_body_poses':
                for b in msg.bodies:
                    if b.name == 'Base':
                        p = b.pose.pose.position
                        base.append((msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec,
                                     np.array([p.x, p.y, p.z])))
                continue
            if nframes and i >= nframes:
                break
            i += 1
            ns = msg.stamp.sec * 10 ** 9 + msg.stamp.nanosec
            t = ns * 1e-9
            if last is not None and t - last < FEED_PERIOD_S:
                continue
            last = t
            frames.append((t, ns, [(mk.label, mk.position.x, mk.position.y, mk.position.z, mk.residual)
                                   for mk in msg.markers]))
    return frames, base


def displace(rows, label, mm):
    out = []
    for r in rows:
        out.append((r[0], r[1] + mm, *r[2:]) if r[0] == label else r)
    return out


def replay(frames, tpl, cap, move_edges=(), disp=None):
    """Run the production tracker; returns a dict of the four headline numbers + extras."""
    tr = rf.RobotFrameTracker(tpl, max_rms_mm=cap)
    t0 = frames[0][0]
    known = 0
    trans, state = [], None
    hist = []                       # (t, origin, yaw_deg) of the running frame
    rms, drops = [], []
    n_ok = 0
    for t, ns, rows in frames:
        if disp:
            rows = displace(rows, *disp)
        fit = tr.feed(rows, ns)
        cur = tr.current()
        has = cur is not None
        known += has
        if has != state:
            trans.append((t - t0, 'OK' if has else 'UNK'))
            state = has
        if has:
            hist.append((t - t0, *cur.t, math.degrees(math.atan2(cur.R[1, 0], cur.R[0, 0]))))
        if fit.ok:
            n_ok += 1
            rms.append(fit.rms_mm)
            drops.append(tuple(fit.dropped))
    H = np.array(hist) if hist else np.zeros((0, 5))
    # Scatter about the median of each stretch between UNKNOWN gaps / given move edges.
    edges = [0.0] + list(move_edges) + [1e9]
    do, dy = [], []
    for a, b in zip(edges[:-1], edges[1:]):
        S = H[(H[:, 0] >= a) & (H[:, 0] < b)]
        if len(S) < 5:
            continue
        do += list(np.linalg.norm(S[:, 1:4] - np.median(S[:, 1:4], 0), axis=1))
        dy += list(np.abs(S[:, 4] - np.median(S[:, 4])))
    nd = [d for d in drops if d]
    cnt = {}
    for d in nd:
        for lab in d:
            cnt[lab] = cnt.get(lab, 0) + 1
    top = max(cnt, key=cnt.get) if cnt else '-'
    r = np.array(rms) if rms else np.array([float('nan')])
    return dict(n=len(frames), lock=(next((x[0] for x in trans if x[1] == 'OK'), float('nan'))),
                share=known / max(1, len(frames)), trans=trans,
                o95=float(np.percentile(do, 95)) if do else float('nan'),
                y95=float(np.percentile(dy, 95)) if dy else float('nan'),
                drop_share=len(nd) / max(1, n_ok), top=top,
                rms50=float(np.percentile(r, 50)), rms99=float(np.percentile(r, 99)),
                final=tr.current(), hist=H)


MOVE_EDGES = {'2026-10-19_11-13-17': [64.5, 69.0], '2026-10-19_11-14-54': [36.5, 40.0]}


def run_replay(a):
    tpl = rf.load_robot_frame_template(TEMPLATE)
    bags = a.bags or (NEW_BAGS + EARLIER_BAGS)
    disp = None
    if a.displace:
        lab, mm = a.displace.rsplit(':', 1)
        disp = (lab, float(mm))
    lines = ['# robot_frame_bag_check --replay', '',
             f'production RobotFrameTracker, feed {FEED_PERIOD_S} s of frame time, displace={a.displace}', '',
             '| bag | cap | feeds | lock s | frame share | transitions | origin p95 mm | yaw p95 deg | '
             'drop share (top) | RMS p50/p99 | final origin |', '|---|' * 1 + '---|' * 10]
    for bag in bags:
        name = os.path.basename(bag.rstrip('/'))
        frames, base = load_feed_frames(bag, a.frames)
        print(f'{name}: {len(frames)} feeds', flush=True)
        for cap in a.caps:
            r = replay(frames, tpl, cap, MOVE_EDGES.get(name, ()), disp)
            fo = r['final']
            lines.append(f"| {name} | {cap} | {r['n']} | {r['lock']:.1f} | {r['share']:.3f} | "
                         f"{len(r['trans'])} {[(round(t, 1), s) for t, s in r['trans']]} | {r['o95']:.2f} | "
                         f"{r['y95']:.3f} | {r['drop_share']:.3f} ({r['top']}) | {r['rms50']:.2f}/{r['rms99']:.2f} | "
                         + ('n/a' if fo is None else f'({fo.t[0]:.2f}, {fo.t[1]:.2f}, {fo.t[2]:.2f})') + ' |')
            print(lines[-1], flush=True)
    out_dir = a.out or os.path.join(REPO, 'temp', 'probes', 'robot_frame',
                                    datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
    os.makedirs(out_dir, exist_ok=True)
    p = os.path.join(out_dir, 'replay.md')
    with open(p, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print('written', p)
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--bags', nargs='*', default=None)
    ap.add_argument('--frames', type=int, default=None)
    ap.add_argument('--replay', action='store_true')
    ap.add_argument('--caps', nargs='*', type=float, default=[rf.ROBOT_FRAME_MAX_RMS_MM])
    ap.add_argument('--displace', default=None)
    ap.add_argument('--every', type=int, default=2)
    ap.add_argument('--out', default=None)
    a = ap.parse_args(argv)
    if a.replay:
        a.frames = a.frames or 0
        return run_replay(a)
    a.bags = a.bags or DEFAULT_BAGS
    a.frames = a.frames or 6000
    tpl = rf.load_robot_frame_template(TEMPLATE)
    out_dir = a.out or os.path.join(REPO, 'temp', 'probes', 'robot_frame',
                                    datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
    os.makedirs(out_dir, exist_ok=True)
    fit_rows, plat_rows, off_rows = [], [], []
    for bag in a.bags:
        name = os.path.basename(bag.rstrip('/'))
        fits, plat, body = analyse(bag, tpl, a.frames, a.every)
        ok = [f for f in fits if f[2]]
        rms = np.array([f[1] for f in fits if math.isfinite(f[1])])
        Rm = rf.RobotFrame(*[None] * 0) if False else None
        if ok:
            ts = np.array([f[4] for f in ok])
            from jugglebot.bb_base_frame import mean_rotation
            R = mean_rotation(np.array([f[3] for f in ok]))
            yaw = yaw_deg(R)
            t = np.median(ts, axis=0)
            tstr = f'({t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f})'
        else:
            yaw = float('nan'); tstr = 'n/a'
        bb = np.mean([b[0] for b in body], axis=0) if body else None
        fit_rows.append(f'| {name} | {len(fits)} | {len(ok) / max(1, len(fits)):.3f} | '
                        f'{np.median(rms) if len(rms) else float("nan"):.3f} / '
                        f'{np.percentile(rms, 99) if len(rms) else float("nan"):.3f} | {tstr} | '
                        f'{yaw:+.3f} | '
                        + ('n/a' if bb is None else f'({bb[0]:.2f}, {bb[1]:.2f}, {bb[2]:.2f})') + ' |')
        # Fit origin minus QTM Base body (world), per body message with a valid fit.
        d = [np.asarray(fr.t) - b for b, fr in body if fr is not None]
        if d:
            m = np.mean(d, axis=0)
            off_rows.append(f'| {name} | {len(d)} | ({m[0]:.2f}, {m[1]:.2f}, {m[2]:.2f}) | '
                            f'{np.max(np.std(d, axis=0)):.3f} |')
        # Platform at rest, world and robot frame.
        if plat:
            tt = np.array([p[0] for p in plat]); W = np.array([p[1] for p in plat])
            mask = rest_mask(tt, W)
            Wr, Rb = [], []
            for (tm, w, fr), m in zip(plat, mask):
                if m and fr is not None:
                    Wr.append(w); Rb.append(fr.to_robot(w))
            if Wr:
                Wr = np.array(Wr); Rb = np.array(Rb)
                key = np.round(Rb / 5.0).astype(int)      # dominant rest pose: 5 mm cells in the robot frame
                cells, cnt = np.unique(key, axis=0, return_counts=True)
                sel = np.all(key == cells[np.argmax(cnt)], axis=1)
                mw, mr = Wr[sel].mean(axis=0), Rb[sel].mean(axis=0)
                plat_rows.append(f'| {name} | {len(Wr)} | {int(sel.sum())} | '
                                 f'({mw[0]:.2f}, {mw[1]:.2f}, {mw[2]:.2f}) | ({mr[0]:.2f}, {mr[1]:.2f}, {mr[2]:.2f}) |')
            else:
                plat_rows.append(f'| {name} | 0 | 0 | no rest frames | no rest frames |')
        print(f'{name}: {len(fits)} fits, {len(ok)} ok', flush=True)
    md = ['# robot_frame_bag_check', '', f'template {os.path.relpath(TEMPLATE, REPO)}; '
          f'{a.frames} mocap frames per bag from the start, every {a.every}', '',
          '## Fit per bag (world; t = robot origin in world mm, yaw of R in deg)', '',
          '| bag | frames fitted | fit-OK | RMS median / p99 mm | t median (mm) | yaw (deg) | QTM Base body mean (mm) |',
          '|---|---|---|---|---|---|---|', *fit_rows, '',
          '## Fit origin minus QTM Base body position (world, mm)', '',
          '| bag | n | mean offset | max per-axis sd |', '|---|---|---|---|', *off_rows, '',
          '## Platform at rest (sd < 0.5 mm over 1 s), dominant 5 mm rest cell; z includes +'
          f'{GEOM_INITIAL_HEIGHT_MM} mm', '',
          '| bag | rest frames | in cell | world (mm) | robot frame (mm) |', '|---|---|---|---|---|', *plat_rows, '']
    p = os.path.join(out_dir, 'bag_check.md')
    with open(p, 'w') as f:
        f.write('\n'.join(md))
    print('\n'.join(md)); print('written', p)
    return 0


if __name__ == '__main__':
    sys.exit(main())
