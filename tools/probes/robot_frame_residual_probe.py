#!/usr/bin/env python3
"""robot_frame_residual_probe.py - evidence for ROBOT_FRAME_MAX_RMS_MM (jugglebot.robot_frame).

What it measures
    The per-frame Kabsch fit RMS of the seven labelled robot-base markers (``Base - 1..7``) against
    a template learned from the kinematic-calibration bag (whose QTM global frame IS the robot
    frame), on each bag given, plus the fitted robot frame (origin t, rotation angle). With the cap
    lifted, so the live scatter is seen in full. Also the single-marker sensitivity: how much the
    RMS grows when one template marker is displaced by 1 mm (the cap's refusal threshold in mm of
    marker error = cap / slope).

Inputs
    --bags      bag directories (default: the kincal bag + the two 2026-10-16 bags below), each
                holding one ``*.mcap`` with ``/mocap_data``.
    --template-bag  the bag the template is learned from (default: the kincal bag).
    --frames    frames read per bag (default 3000, from the start of the bag).

Usage (repo root, project venv)
    python -I tools/probes/robot_frame_residual_probe.py [--bags DIR ...] [--frames 3000]
    Writes ``temp/probes/robot_frame/<timestamp>/report.txt`` and ``fits.csv``. ~1 min.

Result, 2026-10-19: see the ROBOT_FRAME_MAX_RMS_MM comment in robot_frame.py (it quotes this probe).
"""
from __future__ import annotations

import argparse
import csv
import datetime
import glob
import os
import sys

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'ros_ws', 'src', 'jugglebot'))
from jugglebot import robot_frame as rf  # noqa: E402

BAG_ROOT = os.path.expanduser('~/Desktop/rosbags')
KINCAL = '2026-09-27_14-31-13'
DEFAULT_BAGS = [KINCAL, '2026-10-16_16-23-01', '2026-10-16_22-01-50']


def read_frames(bag: str, n: int):
    """[(stamp_ns, labelled rows)] for the first ``n`` /mocap_data messages."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    d = bag if os.path.isdir(bag) else os.path.join(BAG_ROOT, bag)
    path = sorted(glob.glob(os.path.join(d, '*.mcap')))[0]
    out = []
    with open(path, 'rb') as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for _s, _c, _m, msg in reader.iter_decoded_messages(topics=['/mocap_data']):
            rows = [(mk.label, mk.position.x, mk.position.y, mk.position.z, mk.residual)
                    for mk in msg.markers]
            out.append((msg.stamp.sec * 10 ** 9 + msg.stamp.nanosec, rows))
            if len(out) >= n:
                break
    return out


def sensitivity(template, frames):
    """RMS growth per mm when one template marker is displaced (worst direction, median frame)."""
    lines = []
    _, rows = frames[len(frames) // 2]
    base = rf.fit_robot_frame(rows, template, max_rms_mm=np.inf).rms_mm
    for i, lb in enumerate(template.labels):
        worst = 0.0
        for ax in range(3):
            P = np.array(template.points_mm)
            P[i, ax] += 1.0
            T2 = rf.RobotFrameTemplate(template.labels, P)
            worst = max(worst, rf.fit_robot_frame(rows, T2, max_rms_mm=np.inf).rms_mm - base)
        lines.append(f'  {lb}: +{worst:.3f} mm RMS per mm displaced (worst axis)')
    return lines


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bags', nargs='+', default=DEFAULT_BAGS)
    ap.add_argument('--template-bag', default=KINCAL)
    ap.add_argument('--frames', type=int, default=3000)
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    out = a.out or os.path.join(REPO, 'temp', 'probes', 'robot_frame',
                                datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
    os.makedirs(out, exist_ok=True)

    tframes = read_frames(a.template_bag, a.frames)
    template = rf.learn_robot_frame_template([r for _, r in tframes], source=a.template_bag)
    rep = [f'robot_frame_residual_probe {datetime.datetime.now().isoformat(timespec="seconds")}',
           f'template from {a.template_bag} ({len(tframes)} frames):']
    for lb, p in zip(template.labels, template.points_mm):
        st = template.provenance['per_marker'][lb]
        rep.append(f'  {lb}: ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}) mm, n {st["n"]}, '
                   f'sd {np.round(st["sd_mm"], 3).tolist()}')
    all_rms = []
    with open(os.path.join(out, 'fits.csv'), 'w', newline='') as fcsv:
        w = csv.writer(fcsv)
        w.writerow(['bag', 'stamp_ns', 'n_matched', 'rms_mm', 'reason', 'tx', 'ty', 'tz', 'rot_deg'])
        for bag in a.bags:
            frames = tframes if bag == a.template_bag else read_frames(bag, a.frames)
            rms, ts, angs, nm, reasons = [], [], [], [], {}
            tracker = rf.RobotFrameTracker(template, max_rms_mm=np.inf)
            for stamp, rows in frames:
                fit = tracker.feed(rows, stamp)
                reasons[fit.reason or 'OK'] = reasons.get(fit.reason or 'OK', 0) + 1
                if fit.ok:
                    rms.append(fit.rms_mm)
                    ts.append(fit.frame.t)
                    angs.append(rf.rotation_angle_deg(fit.frame.R))
                    nm.append(fit.n_matched)
                    w.writerow([bag, stamp, fit.n_matched, f'{fit.rms_mm:.4f}', '',
                                *[f'{v:.3f}' for v in fit.frame.t], f'{angs[-1]:.4f}'])
                else:
                    w.writerow([bag, stamp, fit.n_matched, '', fit.reason, '', '', '', ''])
            rms = np.array(rms)
            all_rms.extend(rms.tolist())
            rep.append(f'\n{bag}: {len(frames)} frames, outcomes {reasons}')
            if len(rms):
                cur = tracker.current()
                tm = np.median(np.array(ts), axis=0)
                rep.append(f'  RMS median {np.median(rms):.3f}  p99 {np.percentile(rms, 99):.3f}  '
                           f'max {rms.max():.3f} mm;  n_matched min {min(nm)} median '
                           f'{int(np.median(nm))}')
                rep.append(f'  per-frame t median ({tm[0]:.2f}, {tm[1]:.2f}, {tm[2]:.2f}) mm, sd '
                           f'{np.round(np.std(ts, axis=0), 3).tolist()} mm; rot median '
                           f'{np.median(angs):.4f} deg (sd {np.std(angs):.4f})')
                rep.append(f'  tracker current t ({cur.t[0]:.2f}, {cur.t[1]:.2f}, {cur.t[2]:.2f}) '
                           f'mm, rot {rf.rotation_angle_deg(cur.R):.4f} deg, quat '
                           f'{np.round(cur.quaternion_xyzw(), 6).tolist()}')
    all_rms = np.array(all_rms)
    rep.append(f'\nALL: {len(all_rms)} fits, RMS median {np.median(all_rms):.3f}  p99 '
               f'{np.percentile(all_rms, 99):.3f}  max {all_rms.max():.3f} mm')
    rep.append('single-marker sensitivity (kincal bag):')
    rep.extend(sensitivity(template, tframes))
    rep.append(f'current cap ROBOT_FRAME_MAX_RMS_MM = {rf.ROBOT_FRAME_MAX_RMS_MM} mm')
    text = '\n'.join(rep)
    with open(os.path.join(out, 'report.txt'), 'w') as f:
        f.write(text + '\n')
    print(text)
    print(f'\nwrote {out}')


if __name__ == '__main__':
    main()
