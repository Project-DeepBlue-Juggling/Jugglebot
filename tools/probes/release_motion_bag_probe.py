#!/usr/bin/env python3
"""Per-release platform motion and ball launch from a sitting's bag (skill-stack R4+).

For every ``jugglebot`` ThrowAnnouncement in the bag: the mocap ``Platform`` body's tilt (deg)
and world xy (mm) from -0.6 s to +0.2 s around the announced release, the announced launch
velocity, and a gravity-fixed least-squares fit of the tracked ball's free flight right after
release (launch velocity at t_rel, apex, landing on the announced landing plane, the error
against the announced landing). The "parabola is N mm above/below the announced release point at
t_rel" line is the release-lag instrument: a ball that separates from the hand LATER than the
plan's release knot extrapolates back to BELOW the release point (2026-09-27 sitting: -48 .. -172
mm on every throw, i.e. 20-40 ms late). Optionally dumps hand pos_cmd / pos_meas over a window.

First use: `logbook/2026-09-28-skill-stack-r4-sitting-1-analysis.md` (the throw after a
laterally re-aimed catch leaves with +85..+110 mm/s of unplanned lateral velocity while the
platform is level and stationary at release; the hop's platform tilt matched the plan to 0.1 deg
and the +100 mm overshoot is the late release meeting the plan's immediate return).

Offline and read-only: opens ``.mcap`` files (``mcap_ros2`` decodes the schemas embedded in the
bag) and writes one timestamped report under ``temp/probes/``.

Usage:
  python tools/probes/release_motion_bag_probe.py ~/Desktop/rosbags/<bag_dir> \
      [--hand-window T0 T1] [--out temp/probes/release_motion_<stamp>.txt]
"""
from __future__ import annotations

import argparse
import datetime as _dt
import glob
import math
import os
import sys

import numpy as np

G_MM_S2 = 9806.65
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _stamp(s):
    return float(s.sec) + 1e-9 * float(s.nanosec)


def load(bag_dir):
    from mcap_ros2.reader import read_ros2_messages
    ann, plat, balls, hand = [], [], [], []
    for p in sorted(glob.glob(os.path.join(bag_dir, '*.mcap'))):
        for m in read_ros2_messages(p, topics=['/throw_announcements', '/rigid_body_poses',
                                               '/balls', '/hand_telemetry']):
            r = m.ros_msg
            t_log = m.log_time_ns * 1e-9
            if m.channel.topic == '/throw_announcements':
                ann.append((t_log, r))
            elif m.channel.topic == '/rigid_body_poses':
                for b in r.bodies:
                    if b.name == 'Platform':
                        q = b.pose.pose.orientation
                        pos = b.pose.pose.position
                        nx = 2 * (q.x * q.z + q.w * q.y)          # body z-axis in world
                        ny = 2 * (q.y * q.z - q.w * q.x)
                        nz = 1 - 2 * (q.x * q.x + q.y * q.y)
                        plat.append((_stamp(b.pose.header.stamp), pos.x, pos.y, pos.z, nx, ny, nz))
            elif m.channel.topic == '/balls':
                for b in r.balls:
                    balls.append((_stamp(b.header.stamp), b.id, b.status,
                                  b.position.x, b.position.y, b.position.z))
            elif m.channel.topic == '/hand_telemetry':
                hand.append((t_log, r.pos_cmd, r.pos_meas, r.vel_meas, r.vel_ff_cmd, r.tor_ff_cmd, r.iq_meas))
    return ann, np.array(plat), np.array(balls), np.array(hand)


def tilt_deg(row):
    nx, ny, nz = row[4], row[5], row[6]
    return (math.degrees(math.atan2(math.hypot(nx, ny), nz)),
            math.degrees(math.asin(max(-1.0, min(1.0, nx)))),
            math.degrees(math.asin(max(-1.0, min(1.0, ny)))))


def report(bag_dir, hand_window=None, out=None):
    ann, plat, balls, hand = load(bag_dir)
    lines = ['bag %s: %d announcements, %d platform samples, %d ball samples, %d hand samples'
             % (bag_dir, len(ann), len(plat), len(balls), len(hand))]

    def plat_at(t):
        i = min(max(np.searchsorted(plat[:, 0], t), 0), len(plat) - 1)
        return plat[i]

    for t_log, a in ann:
        if a.thrower_name != 'jugglebot':
            lines.append('%.3f  %s -> %s (external announcement)' % (t_log, a.thrower_name, a.target_id))
            continue
        t_rel = _stamp(a.throw_time)
        v, p0, land = a.initial_velocity, a.initial_position, a.landing_position
        ang = math.degrees(math.atan2(math.hypot(v.x, v.y), v.z))
        lines.append('')
        lines.append('--- release t=%.3f (announced %.3f s ahead) pos=(%.1f,%.1f,%.1f) v=(%.0f,%.0f,%.0f) mm/s '
                     '|v|=%.2f m/s angle-from-vertical %.2f deg -> announced landing (%.1f,%.1f,%.1f) tof %.3f'
                     % (t_rel, t_rel - t_log, p0.x, p0.y, p0.z, v.x, v.y, v.z,
                        math.hypot(v.x, v.y, v.z) / 1000, ang, land.x, land.y, land.z, a.predicted_tof_sec))
        for dt in (-0.6, -0.4, -0.3, -0.2, -0.1, -0.05, 0.0, 0.05, 0.1, 0.2):
            r = plat_at(t_rel + dt)
            tot, tx, ty = tilt_deg(r)
            lines.append('  platform %+.2f s: tilt %.2f deg (x%+.2f y%+.2f) xy (%.0f, %.0f) z %.0f'
                         % (dt, tot, tx, ty, r[1], r[2], r[3]))
        sel = balls[(balls[:, 0] > t_rel + 0.04) & (balls[:, 0] < t_rel + 0.45) & (balls[:, 2] == 1)]
        if len(sel) < 6:
            lines.append('  ball fit: only %d in-flight samples' % len(sel))
            continue
        tt = sel[:, 0] - t_rel
        A = np.vstack([np.ones_like(tt), tt]).T
        cx = np.linalg.lstsq(A, sel[:, 3], rcond=None)[0]
        cy = np.linalg.lstsq(A, sel[:, 4], rcond=None)[0]
        cz = np.linalg.lstsq(A, sel[:, 5] + 0.5 * G_MM_S2 * tt ** 2, rcond=None)[0]
        vx, vy, vz = cx[1], cy[1], cz[1]
        angf = math.degrees(math.atan2(math.hypot(vx, vy), vz))
        disc = vz ** 2 - 2 * G_MM_S2 * (land.z - cz[0])
        tof = (vz + math.sqrt(max(disc, 0.0))) / G_MM_S2
        lx, ly = cx[0] + vx * tof, cy[0] + vy * tof
        resid = float(np.std(sel[:, 5] - (cz[0] + cz[1] * tt - 0.5 * G_MM_S2 * tt ** 2)))
        lines.append('  ball fit (%d samples %.2f..%.2f s): at t_rel pos=(%.1f,%.1f,%.1f) v=(%.0f,%.0f,%.0f) |v|=%.2f m/s '
                     'angle %.2f deg; apex z %.0f; landing on plane (%.1f,%.1f) tof %.3f; err vs announced (%+.1f,%+.1f) mm; z-resid %.1f mm'
                     % (len(sel), tt.min(), tt.max(), cx[0], cy[0], cz[0], vx, vy, vz,
                        math.hypot(vx, vy, vz) / 1000, angf, cz[0] + vz ** 2 / (2 * G_MM_S2),
                        lx, ly, tof, lx - land.x, ly - land.y, resid))
        lines.append('  free-flight parabola at t_rel is %+.1f mm relative to the announced release point '
                     '(negative = the ball separated AFTER the planned release)' % (cz[0] - p0.z))
    if hand_window is not None and len(hand):
        t0, t1 = hand_window
        lines.append('')
        lines.append('=== hand telemetry %.3f..%.3f (every 8th sample): t cmd meas vel vel_ff tor_ff iq ===' % (t0, t1))
        sel = hand[(hand[:, 0] >= t0) & (hand[:, 0] <= t1)]
        for r in sel[::8]:
            lines.append('%.3f %+.3f %+.3f %+.2f %+.2f %+.3f %+.2f' % tuple(r))
    text = '\n'.join(lines)
    if out is None:
        os.makedirs(os.path.join(ROOT, 'temp', 'probes'), exist_ok=True)
        out = os.path.join(ROOT, 'temp', 'probes',
                           'release_motion_%s.txt' % _dt.datetime.now().strftime('%Y%m%d_%H%M%S'))
    with open(out, 'w') as fh:
        fh.write(text + '\n')
    print(text)
    print('\nwritten: %s' % out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag_dir')
    ap.add_argument('--hand-window', nargs=2, type=float, default=None, metavar=('T0', 'T1'))
    ap.add_argument('--out', default=None)
    args = ap.parse_args(argv)
    if not glob.glob(os.path.join(args.bag_dir, '*.mcap')):
        sys.exit('ERROR: no *.mcap under %s' % args.bag_dir)
    report(args.bag_dir, args.hand_window, args.out)


if __name__ == '__main__':
    main()
