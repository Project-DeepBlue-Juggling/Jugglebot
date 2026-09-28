#!/usr/bin/env python3
"""Decompose every self-toss landing error in a sitting's bag + launch log (skill-stack R4).

Wayfinder ticket 01 of the R4 throw-precision map (``.scratch/r4-throw-precision``): is the
self-toss landing error a repeatable BIAS or SCATTER, and which channel does it enter through?

Per jugglebot throw (``/throw_announcements``):

* COMMANDED -- the announced landing ``A`` (schedule site + learner command ``u``) and the ``u``
  / ``y`` of the matching ``memory row appended`` log line (when a log is given).
* GROUND TRUTH -- a gravity-fixed least-squares parabola fitted to the RAW ``/mocap_data``
  markers of the ball (gated to within 40 mm of the tracker's ``/balls`` track), over
  ``t_rel + 0.08 s`` .. 60 mm above the landing plane on the way down: launch position ``p`` and
  velocity ``v`` at the announced release instant, apex, landing ``L`` on the announced plane.
* REALISED RELEASE -- the mocap ``Platform`` body at ``t_rel``: xy, the xy velocity over
  -40..0 ms, and the body-axis tilt (x, y components, deg) at -50 ms and at ``t_rel``.
* DECOMPOSITION of ``E = L - A`` (lateral): ``E_pos = p_xy - p_ann_xy`` (where the ball left
  from, vs the announcement; for a carried throw the release is the CAUGHT xy and the plan flies
  it back through a small release tilt, so this term is supposed to be cancelled by ``E_vel``),
  and ``needed = (A - p)/T`` the lateral velocity that would have landed the ball on ``A`` from
  where it actually left, ``dv = v_xy - needed`` -> ``E ~= dv * T``. ``dv`` is then set against
  what the cup could have given it: ``v_axial * tilt`` (mocap body axis) + platform xy velocity.
* RE-AIM vs TRUTH -- the first converged tracker fit (``/balls`` ``landing_from_fit``) against
  the final one and against the ground truth: is a 20 mm re-aim chasing real error or fit noise?

Offline and read-only; a raw-data cache (``.npz``) is written next to the report so a re-run is
seconds, not minutes. Outputs under ``temp/probes/``.

Usage:
  python tools/probes/selftoss_landing_decomposition.py ~/Desktop/rosbags/<bag> [--log <launch.log>]
      [--csv temp/probes/selftoss_decomp_<bag>.csv]
"""
from __future__ import annotations

import argparse
import glob
import math
import os
import re
import sys

import numpy as np

G = 9806.65
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT_DIR = os.path.join(ROOT, 'temp', 'probes')


def _stamp(s):
    return float(s.sec) + 1e-9 * float(s.nanosec)


def load(bag_dir):
    cache = os.path.join(OUT_DIR, 'selftoss_decomp_cache_%s.npz' % os.path.basename(bag_dir.rstrip('/')))
    if os.path.exists(cache):
        d = np.load(cache, allow_pickle=True)
        return {k: d[k] for k in d.files}
    from mcap_ros2.reader import read_ros2_messages
    ann, plat, balls, mk, hand = [], [], [], [], []
    for p in sorted(glob.glob(os.path.join(bag_dir, '*.mcap'))):
        for m in read_ros2_messages(p, topics=['/throw_announcements', '/rigid_body_poses', '/balls',
                                               '/mocap_data', '/hand_telemetry']):
            r, top = m.ros_msg, m.channel.topic
            tl = m.log_time_ns * 1e-9
            if top == '/throw_announcements':
                if r.thrower_name != 'jugglebot':
                    continue
                v, p0, L = r.initial_velocity, r.initial_position, r.landing_position
                ann.append((tl, _stamp(r.throw_time), p0.x, p0.y, p0.z, v.x, v.y, v.z,
                            L.x, L.y, L.z, r.predicted_tof_sec))
            elif top == '/rigid_body_poses':
                for b in r.bodies:
                    if b.name == 'Platform':
                        q, pos = b.pose.pose.orientation, b.pose.pose.position
                        plat.append((_stamp(b.pose.header.stamp), pos.x, pos.y, pos.z,
                                     2 * (q.x * q.z + q.w * q.y), 2 * (q.y * q.z - q.w * q.x)))
            elif top == '/balls':
                for b in r.balls:
                    lp = b.landing_position
                    balls.append((_stamp(b.header.stamp), tl, b.id, b.status, b.position.x, b.position.y,
                                  b.position.z, lp.x, lp.y, lp.z, _stamp(b.time_at_land),
                                  1.0 if b.landing_from_fit else 0.0))
            elif top == '/mocap_data':
                t = _stamp(r.stamp)
                for s in r.markers:
                    if s.position.z > 600.0:
                        mk.append((t, s.position.x, s.position.y, s.position.z))
            elif top == '/hand_telemetry':
                hand.append((tl, r.pos_cmd, r.pos_meas, r.vel_meas, r.vel_ff_cmd))
    d = {'ann': np.array(ann), 'plat': np.array(plat), 'balls': np.array(balls), 'mk': np.array(mk),
         'hand': np.array(hand)}
    os.makedirs(OUT_DIR, exist_ok=True)
    np.savez(cache, **d)
    return d


_ROW = re.compile(r'\[(\d+\.\d+)\] \[skill_node\]: memory row appended: x=\[([^\]]*)\] u=\[([^\]]*)\] '
                  r'y=\[([^\]]*)\] caught=(\w+)')
_RESEND = re.compile(r'\]: (\d+\.\d+) RESEND(?:-REFUSED (\w+))? (?:at )?skill (\d+): the fit moved the landing '
                     r'([\d.]+) mm / ([+-][\d.]+) s')
_CLAMP = re.compile(r'\[(\d+\.\d+)\] \[skill_node\]: AIM-LATERAL-CLAMPED skill (\d+): tracker landing '
                    r'([+-][\d.]+) mm in (\w)')
_SEAT = re.compile(r'\]: (\d+\.\d+) OUTCOME ball \d+: y=.*caught=(\w+)(?: seat=([+-][\d.]+) s)?')


def parse_log(path):
    rows, resends, clamps, seats = [], [], [], []
    if not path:
        return rows, resends, clamps, seats
    with open(path, errors='replace') as fh:
        for line in fh:
            m = _ROW.search(line)
            if m:
                f = lambda s: [float(v) for v in s.split(',')]
                rows.append((float(m.group(1)), f(m.group(2)), f(m.group(3)), f(m.group(4)), m.group(5) == 'True'))
                continue
            m = _RESEND.search(line)
            if m:
                resends.append((float(m.group(1)), m.group(2) or 'OK', float(m.group(4)), float(m.group(5))))
                continue
            m = _CLAMP.search(line)
            if m:
                clamps.append((float(m.group(1)), float(m.group(3)), m.group(4)))
                continue
            m = _SEAT.search(line)
            if m:
                seats.append((float(m.group(1)), m.group(2) == 'True', float(m.group(3)) if m.group(3) else None))
    return rows, resends, clamps, seats


def fit_flight(d, t_rel, land_z, tof_ann):
    """Gravity-fixed LSQ on raw markers gated to the tracker's track; returns dict or None."""
    b = d['balls']
    sel = b[(b[:, 0] > t_rel + 0.05) & (b[:, 0] < t_rel + tof_ann + 0.05) & (b[:, 3] == 1)]
    if len(sel) < 6:
        return None
    mk = d['mk']
    w = mk[(mk[:, 0] > t_rel + 0.08) & (mk[:, 0] < t_rel + tof_ann + 0.1)]
    if len(w) < 10:
        return None
    # gate each marker to the tracker position nearest in time
    idx = np.clip(np.searchsorted(sel[:, 0], w[:, 0]), 0, len(sel) - 1)
    near = np.linalg.norm(w[:, 1:4] - sel[idx, 4:7], axis=1) < 60.0
    w = w[near]
    coef = None
    for _ in range(3):
        if len(w) < 10:
            return None
        tt = w[:, 0] - t_rel
        A = np.vstack([np.ones_like(tt), tt]).T
        cx = np.linalg.lstsq(A, w[:, 1], rcond=None)[0]
        cy = np.linalg.lstsq(A, w[:, 2], rcond=None)[0]
        cz = np.linalg.lstsq(A, w[:, 3] + 0.5 * G * tt ** 2, rcond=None)[0]
        pred = np.vstack([cx[0] + cx[1] * tt, cy[0] + cy[1] * tt, cz[0] + cz[1] * tt - 0.5 * G * tt ** 2]).T
        res = np.linalg.norm(w[:, 1:4] - pred, axis=1)
        # keep free flight only: above land_z + 60 mm on the way down
        keep = (res < 15.0) & ((w[:, 3] > land_z + 60.0) | (tt < cz[1] / G))
        coef = (cx, cy, cz, float(np.sqrt(np.mean(res[keep] ** 2))) if keep.any() else float('nan'), int(keep.sum()))
        w = w[keep]
    cx, cy, cz, rms, n = coef
    vz = cz[1]
    disc = vz ** 2 - 2 * G * (land_z - cz[0])
    T = (vz + math.sqrt(max(disc, 0.0))) / G
    return dict(p=np.array([cx[0], cy[0], cz[0]]), v=np.array([cx[1], cy[1], vz]), T=T,
                L=np.array([cx[0] + cx[1] * T, cy[0] + cy[1] * T]), apex=cz[0] + vz ** 2 / (2 * G) - land_z,
                rms=rms, n=n)


def plat_state(d, t):
    P = d['plat']

    def at(tq):
        i = int(np.clip(np.searchsorted(P[:, 0], tq), 0, len(P) - 1))
        return P[i]
    r0, rm = at(t), at(t - 0.05)
    a, b = at(t - 0.04), at(t)
    vel = (b[1:3] - a[1:3]) / max(b[0] - a[0], 1e-3)
    return dict(xy=r0[1:3], z=r0[3], vxy=vel, tilt=np.degrees(np.arcsin(np.clip(r0[4:6], -1, 1))),
                tilt_m50=np.degrees(np.arcsin(np.clip(rm[4:6], -1, 1))))


def tracker_fits(d, t_rel, t_land):
    """The ball's /balls landing predictions between release and landing: first converged fit, last."""
    b = d['balls']
    sel = b[(b[:, 0] > t_rel) & (b[:, 0] < t_land) & (b[:, 3] == 1) & (b[:, 11] > 0.5)]
    if not len(sel):
        return None
    ids, cnt = np.unique(sel[:, 2], return_counts=True)
    sel = sel[sel[:, 2] == ids[np.argmax(cnt)]]
    return dict(first=sel[0, 7:9], t_first=sel[0, 0] - t_rel, last=sel[-1, 7:9], t_last=sel[-1, 0] - t_rel,
                n=len(sel))


def analyse(bag, log=None):
    d = load(bag)
    rows, resends, clamps, seats = parse_log(log)
    out = []
    for a in d['ann']:
        tl, t_rel = a[0], a[1]
        p_ann, v_ann, A, tof = a[2:5], a[5:8], a[8:11], a[11]
        ang = math.degrees(math.atan2(math.hypot(v_ann[0], v_ann[1]), v_ann[2]))
        kind = 'hop' if ang > 2.0 else ('rest' if t_rel - tl < 0.8 else 'carried')
        fit = fit_flight(d, t_rel, A[2], tof)
        ps = plat_state(d, t_rel)
        rec = dict(t_rel=t_rel, kind=kind, p_ann=p_ann, v_ann=v_ann, A=A, tof=tof, fit=fit, plat=ps)
        # join the log: memory row finalised ~1.0-1.6 s after the release
        rec['row'] = next((r for r in rows if 0.7 < r[0] - t_rel < 1.9), None)
        rec['seat'] = next((s for s in seats if 0.7 < s[0] - t_rel < 1.9), None)
        rec['resend'] = [r for r in resends if 0.1 < r[0] - t_rel < tof]
        rec['clamp'] = [c for c in clamps if 0.0 < c[0] - t_rel < tof + 0.1]
        rec['trk'] = tracker_fits(d, t_rel, t_rel + tof)
        if fit is not None:
            E = fit['L'] - A[:2]
            need = (A[:2] - fit['p'][:2]) / fit['T']
            dv = fit['v'][:2] - need
            vax = fit['v'][2]
            cup = vax * np.radians(ps['tilt']) + ps['vxy']
            rec.update(E=E, E_pos=fit['p'][:2] - p_ann[:2], need=need, dv=dv, cup=cup,
                       resid=dv - cup, speed_ratio=np.linalg.norm(fit['v']) / np.linalg.norm(v_ann))
        out.append(rec)
    return out


def fmt(recs, label):
    L = ['## %s' % label,
         'kind    t_rel          A_xy(ann)      u(row mm,apex)        L-A err mm     p_rel-p_ann   v_fit xy   need xy'
         '    dv xy     cup(tilt+plat) resid xy   tilt@0 (deg)  tilt@-50   plat vxy   s=|v|/|v_ann| apex  rms  trk first->last vs truth  resends']
    for r in recs:
        f = r['fit']
        row = r['row']
        u = ('(%+5.1f,%+5.1f,%.3f)' % (row[2][0] * 1e3, row[2][1] * 1e3, row[2][2])) if row else '      -      '
        if f is None:
            L.append('%-7s %.3f  no fit %s' % (r['kind'], r['t_rel'], u))
            continue
        tk = r['trk']
        trk = ('(%+5.1f,%+5.1f)@%.2f->(%+5.1f,%+5.1f)' % (tk['first'][0] - f['L'][0], tk['first'][1] - f['L'][1],
                                                        tk['t_first'], tk['last'][0] - f['L'][0],
                                                        tk['last'][1] - f['L'][1])) if tk else '-'
        rs = ';'.join('%s %.1f' % (x[1], x[2]) for x in r['resend'])
        L.append('%-7s %.3f (%+6.1f,%+5.1f) %s (%+6.1f,%+6.1f) (%+5.1f,%+5.1f) (%+4.0f,%+4.0f) (%+4.0f,%+4.0f) '
                 '(%+4.0f,%+4.0f) (%+4.0f,%+4.0f) (%+4.0f,%+4.0f) (%+.2f,%+.2f) (%+.2f,%+.2f) (%+4.0f,%+4.0f) %.3f %.3f %4.1f %s %s'
                 % (r['kind'], r['t_rel'], r['A'][0], r['A'][1], u, r['E'][0], r['E'][1], r['E_pos'][0], r['E_pos'][1],
                    f['v'][0], f['v'][1], r['need'][0], r['need'][1], r['dv'][0], r['dv'][1], r['cup'][0], r['cup'][1],
                    r['resid'][0], r['resid'][1], r['plat']['tilt'][0], r['plat']['tilt'][1],
                    r['plat']['tilt_m50'][0], r['plat']['tilt_m50'][1], r['plat']['vxy'][0], r['plat']['vxy'][1],
                    r['speed_ratio'], f['apex'] / 1000, f['rms'], trk, rs))
    return L


def stats(recs, label):
    L = ['## stats %s (self-toss only)' % label]
    for kind in ('rest', 'carried', 'all'):
        s = [r for r in recs if r['fit'] is not None and r['kind'] != 'hop' and (kind == 'all' or r['kind'] == kind)]
        if len(s) < 2:
            continue
        for key in ('E', 'dv', 'resid', 'E_pos', 'cup'):
            X = np.array([r[key] for r in s])
            L.append('  %-7s n=%2d %-6s mean (%+6.1f,%+6.1f)  1sigma (%5.1f,%5.1f)' % (
                kind, len(s), key, X[:, 0].mean(), X[:, 1].mean(), X[:, 0].std(ddof=1), X[:, 1].std(ddof=1)))
        sr = np.array([r['speed_ratio'] for r in s])
        L.append('  %-7s speed ratio %.3f +- %.3f' % (kind, sr.mean(), sr.std(ddof=1)))
    return L


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--log', action='append', default=[], help='launch log, one per bag, same order')
    args = ap.parse_args(argv)
    os.makedirs(OUT_DIR, exist_ok=True)
    lines, allr = [], []
    for i, bag in enumerate(args.bags):
        log = args.log[i] if i < len(args.log) else None
        recs = analyse(bag, log)
        allr += recs
        lab = os.path.basename(bag.rstrip('/'))
        lines += fmt(recs, lab) + stats(recs, lab) + ['']
    if len(args.bags) > 1:
        lines += stats(allr, 'ALL BAGS')
    text = '\n'.join(lines)
    name = 'selftoss_decomp_%s.txt' % '_'.join(os.path.basename(b.rstrip('/'))[:16] for b in args.bags)
    with open(os.path.join(OUT_DIR, name), 'w') as fh:
        fh.write(text + '\n')
    print(text)
    print('\nwritten: %s' % os.path.join(OUT_DIR, name))


if __name__ == '__main__':
    main()
