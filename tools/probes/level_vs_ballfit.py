#!/usr/bin/env python3
"""Ballistic level check: the platform's attitude at release against gravity's
TRUE direction (taken from the thrown balls' own free-flight fits), and the
`trajectory_node.level_trim_deg` that would cancel the residual lean.

WHY THIS EXISTS
----------------
`trajectory_node`'s levelling correction (contract C-LEVEL-1,
``ros_ws/docs/levelling_frame.md``) makes the platform level with respect to
whatever the Platform Teensy's INCLINOMETER reads during `level` — not
necessarily level with respect to true gravity. A U2 investigation (R5
sitting 3, 2026-10-02 — see
``plans/active/two-ball-skill-stack.md``) found a small, repeatable residual:
across two live sittings and the R4 gate bag, self-tosses land biased toward
+y, and the platform's own encoder-FK attitude at release leans
(+0.07, +0.30) deg / (+0.07, +0.23) deg / (-0.02, +0.13) deg against gravity's
ACTUAL direction, as measured independently from the thrown balls'
free-flight parabolas. This probe is that one measurement, promoted from a
one-off scratchpad analysis to a committed, reusable instrument, with
`level_trim_deg` (``ros_ws/src/jugglebot/jugglebot/trajectory_node.py``,
``motion/levelling.py``) as the lever it feeds.

METHOD
------
Decodes four topics from the bag once (cached under
``temp/probes/level_vs_ballfit_cache/<bag>.npz``; re-decode with --force):
``/throw_announcements``, ``/rigid_body_poses``, ``/balls``, ``/mocap_data``,
``/robot_state``.

Per jugglebot-thrown ball (``/throw_announcements`` with
``thrower_name == 'jugglebot'``):
  * **Ball free flight** — a gravity-fixed LSQ parabola on raw
    ``/mocap_data`` markers, gated to the matching ``/balls`` track (3-pass
    outlier reject), for landing/apex/rms VALIDITY; plus a free (unconstrained
    vertical accel) quadratic fit whose horizontal acceleration ``(ax, ay)``
    is the ball's DOWNWARD (free-fall) acceleration direction in the QTM
    frame — a free body's acceleration vector IS gravity's vector, expressed
    in frame coordinates. **Gravity's UP direction** (what a platform "lean
    against gravity" is conventionally measured against, and the sign
    `/gravity_offset` itself uses) is the NEGATION of that:
    ``gravity_up_tilt = degrees(-a_horizontal / 9806.65)``. Getting this
    negation backwards was this probe's own first bug (self-caught on the
    sitting-A run below: the unnegated quantity reproduced u2's own
    ``acc``-derived frame-tilt number but gave a lean 5-10x too large and the
    wrong sign) — read it twice before touching the sign anywhere in this
    file.
  * **Platform attitude at release** — forward kinematics of the MEASURED
    ``/robot_state`` leg encoders (not commanded) through the production
    ``jugglebot.motion.ik_solver.leg_lengths_to_pose`` — what the platform
    actually achieved, independent of mocap.

Valid fit: apex 850-990 mm, launch angle < 3 deg, rms < 9 mm (ball), and
>= 0.5 s of tracked span (gravity-direction estimate only — a short arc's
quadratic fit is noisier, same gate the original analysis used).

Gravity's UP direction is estimated ONCE per sitting (bag), pooled over every
valid-fit throw — it is a frame constant, and pooling is far more precise
than any one throw's fit (sem ~0.02 deg over dozens of throws). Per-throw:

    lean[throw] = platform_fk_tilt_at_release - gravity_up_tilt_this_sitting

reported as the per-sitting MEAN +/- standard error of the mean.

`level_trim_deg` SIGN CONVENTION — READ BEFORE PASTING A NUMBER
-----------------------------------------------------------------
`trajectory_node`'s ``level_trim_deg`` is ADDED to the raw
``/gravity_offset`` tilt BEFORE the sign flip in
``motion/levelling.correction_from_offset`` (``rotvec = [-tilt_x, -tilt_y,
0]``) — see ``motion/levelling.offset_with_trim``. For a nominally level
request, increasing the STORED offset by `t` deg changes the COMMANDED tilt
by `-t` deg. The achieved platform attitude tracks the commanded one closely
(confirmed independently in the U2 investigation: FK and mocap agree to
~0.05-0.1 deg, and the legs hold the commanded tilt within ~0.1-0.3 deg
through a throw — there is no compliance large enough to matter here), so
changing the commanded tilt by `-t` changes the measured lean by
approximately `-t` too. To drive the lean to zero:

    recommended level_trim_deg = +lean   (SAME sign as the measured lean,
                                           added directly — NOT negated)

This was cross-checked against the two live sittings' own numbers: sitting A's
`level` published a tiltY offset 0.11 deg lower than the R4 gate's, and the
measured lean_y moved UP by a comparable amount — offset down, lean up, the
slope of -1 this derivation predicts (not independent evidence: both sittings
came from the same two numbers this probe reproduces, but it is the right
direction and the right order of magnitude).

USAGE
-----
    python tools/probes/level_vs_ballfit.py --bag <rosbag_dir> --log
        <launch.log> [--label A] [--out temp/probes/level_vs_ballfit.log]
        [--force]

Strictly offline and read-only: opens ``.mcap`` (via ``mcap_ros2``, no
``rosbag2_py``, no live ROS2) and the launch log (text) only.
"""
from __future__ import annotations

import argparse
import glob
import math
import os
import re
import sys
import time

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'ros_ws', 'src', 'jugglebot'))

G = 9806.65  # mm/s^2
CACHE_DIR = os.path.join(REPO, 'temp', 'probes', 'level_vs_ballfit_cache')
_TOPICS = ['/throw_announcements', '/rigid_body_poses', '/balls', '/mocap_data',
           '/robot_state']


def _st(stamp) -> float:
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def decode(bag: str, force: bool = False) -> dict:
    """One-pass decode of the four topics this probe needs, cached to an
    ``.npz`` under ``temp/probes/level_vs_ballfit_cache/`` (NOT under
    ``tools/`` — see ``tools/probes/README.md``'s output-location rule).
    Ported from the u2 scratchpad decoder (R5 sitting 3, 2026-10-02), trimmed
    to this probe's five topics."""
    os.makedirs(CACHE_DIR, exist_ok=True)
    name = os.path.basename(bag.rstrip('/'))
    out = os.path.join(CACHE_DIR, name + '.npz')
    if os.path.exists(out) and not force:
        return dict(np.load(out, allow_pickle=True))

    from mcap_ros2.reader import read_ros2_messages
    ann, balls, mk, robot = [], [], [], []
    bodies: dict = {}
    mcaps = sorted(glob.glob(os.path.join(bag, '*.mcap')))
    if not mcaps:
        raise FileNotFoundError('no .mcap files under %r' % (bag,))
    for p in mcaps:
        for m in read_ros2_messages(p, topics=_TOPICS):
            r, top = m.ros_msg, m.channel.topic
            if top == '/throw_announcements':
                v, p0, L = r.initial_velocity, r.initial_position, r.landing_position
                ann.append((_st(r.throw_time), p0.x, p0.y, p0.z, v.x, v.y, v.z,
                            L.x, L.y, L.z, r.predicted_tof_sec,
                            1.0 if r.thrower_name == 'jugglebot' else 0.0))
            elif top == '/rigid_body_poses':
                for b in r.bodies:
                    q, pos = b.pose.pose.orientation, b.pose.pose.position
                    bodies.setdefault(b.name, []).append(
                        (_st(b.pose.header.stamp), pos.x, pos.y, pos.z,
                         q.w, q.x, q.y, q.z))
            elif top == '/balls':
                for b in r.balls:
                    balls.append((_st(b.header.stamp), b.id, b.status,
                                  b.position.x, b.position.y, b.position.z))
            elif top == '/mocap_data':
                t = _st(r.stamp)
                for s in r.markers:
                    if s.position.z > 300.0:
                        mk.append((t, s.position.x, s.position.y, s.position.z))
            elif top == '/robot_state':
                ms = r.motor_states
                pe = ([x.pos_estimate for x in ms] + [np.nan] * 7)[:7]
                robot.append([_st(r.timestamp)] + pe)
    d = dict(ann=np.array(ann, dtype=float), balls=np.array(balls, dtype=float),
              mk=np.array(mk, dtype=float), robot=np.array(robot, dtype=float),
              body_names=np.array(list(bodies.keys()), dtype=object))
    for k, v in bodies.items():
        d['body_' + k] = np.array(v, dtype=float)
    np.savez(out, **d)
    return d


class Plat:
    """Mocap rigid-body attitude -> body z-axis lean (deg, x/y). Ported
    verbatim (minus velocity) from the u2 scratchpad probe's ``Plat`` class."""

    def __init__(self, arr: np.ndarray):
        a = arr[np.argsort(arr[:, 0])]
        self.t = a[:, 0]
        w, x, y, z = a[:, 4], a[:, 5], a[:, 6], a[:, 7]
        zx, zy = 2 * (x * z + w * y), 2 * (y * z - w * x)
        self.tilt = np.degrees(np.arcsin(np.clip(np.vstack([zx, zy]).T, -1, 1)))

    def at(self, t: float) -> np.ndarray:
        return np.array([np.interp(t, self.t, self.tilt[:, k]) for k in (0, 1)])


def fk_tilt_at(t: float, robot: np.ndarray, geom) -> np.ndarray | None:
    """FK of the MEASURED (``/robot_state`` encoder) leg lengths at time
    ``t`` -> ``[tilt_x, tilt_y]`` deg. ``None`` if FK does not converge."""
    from jugglebot.motion.ik_solver import leg_lengths_to_pose
    legs_rev = np.array([np.interp(t, robot[:, 0], robot[:, 1 + k])
                          for k in range(6)])
    ext = legs_rev / np.asarray(geom.mm_to_rev, dtype=float)
    try:
        _pos, R, _ = leg_lengths_to_pose(ext, geom)
    except Exception:
        return None
    return np.degrees(np.arcsin(np.clip(R[:2, 2], -1, 1)))


def fit_flight(d: dict, t_rel: float, land_z: float, tof_ann: float):
    """Gravity-fixed LSQ parabola + free quadratic fit on raw ``/mocap_data``
    markers gated to the matching ``/balls`` track. Ported verbatim from the
    u2 scratchpad probe's ``fit_flight``."""
    b, mk = d['balls'], d['mk']
    sel = b[(b[:, 0] > t_rel + 0.05) & (b[:, 0] < t_rel + tof_ann + 0.05)
             & (b[:, 2] == 1)]
    if len(sel) < 6:
        return None
    w = mk[(mk[:, 0] > t_rel + 0.08) & (mk[:, 0] < t_rel + tof_ann + 0.1)]
    if len(w) < 10:
        return None
    idx = np.clip(np.searchsorted(sel[:, 0], w[:, 0]), 0, len(sel) - 1)
    w = w[np.linalg.norm(w[:, 1:4] - sel[idx, 3:6], axis=1) < 60.0]
    for _ in range(3):
        if len(w) < 10:
            return None
        tt = w[:, 0] - t_rel
        A = np.vstack([np.ones_like(tt), tt]).T
        cx = np.linalg.lstsq(A, w[:, 1], rcond=None)[0]
        cy = np.linalg.lstsq(A, w[:, 2], rcond=None)[0]
        cz = np.linalg.lstsq(A, w[:, 3] + 0.5 * G * tt ** 2, rcond=None)[0]
        pred = np.vstack([cx[0] + cx[1] * tt, cy[0] + cy[1] * tt,
                           cz[0] + cz[1] * tt - 0.5 * G * tt ** 2]).T
        res = np.linalg.norm(w[:, 1:4] - pred, axis=1)
        keep = (res < 15.0) & ((w[:, 3] > land_z + 60.0) | (tt < cz[1] / G))
        if not keep.any():
            return None
        rms = float(np.sqrt(np.mean(res[keep] ** 2)))
        w = w[keep]
    tt = w[:, 0] - t_rel
    Q = np.vstack([np.ones_like(tt), tt, tt ** 2]).T
    qx = np.linalg.lstsq(Q, w[:, 1], rcond=None)[0]
    qy = np.linalg.lstsq(Q, w[:, 2], rcond=None)[0]
    vz = cz[1]
    apex = cz[0] + vz ** 2 / (2 * G) - land_z
    launch_ang = math.degrees(math.atan2(math.hypot(cx[1], cy[1]), vz))
    return dict(rms=rms, apex=apex, launch_ang=launch_ang,
                acc=np.array([2 * qx[2], 2 * qy[2]]),
                span=float(tt.max() - tt.min()))


def analyse(bag: str, force: bool = False) -> dict:
    d = decode(bag, force=force)
    geom = _geom()
    plat = Plat(d['body_Platform']) if 'body_Platform' in d else None
    robot = d['robot']
    robot = robot[np.argsort(robot[:, 0])]
    ann = d['ann']
    rows = []
    for a in ann:
        if a[11] < 0.5:        # not thrown by jugglebot
            continue
        t_rel, land_z, tof = a[0], a[9], a[10]
        fit = fit_flight(d, t_rel, land_z, tof)
        if fit is None:
            continue
        if not (850 < fit['apex'] < 990 and fit['launch_ang'] < 3.0
                 and fit['rms'] < 9.0):
            continue
        fk = fk_tilt_at(t_rel, robot, geom)
        if fk is None:
            continue
        row = dict(t_rel=float(t_rel), fk_tilt=fk, acc=fit['acc'],
                   span=fit['span'])
        if plat is not None:
            row['mocap_tilt'] = plat.at(t_rel)
        rows.append(row)
    return dict(rows=rows)


def _geom():
    from jugglebot.motion.geometry import StewartGeometry
    return StewartGeometry()


def _mean_sem(x: np.ndarray):
    x = np.asarray(x, dtype=float)
    n = len(x)
    if n == 0:
        return float('nan'), float('nan'), 0
    if n == 1:
        return float(x[0]), float('nan'), 1
    return float(x.mean()), float(x.std(ddof=1) / math.sqrt(n)), n


def report(bag: str, label: str, force: bool = False) -> str:
    result = analyse(bag, force=force)
    rows = result['rows']
    lines = []
    lines.append('=== level_vs_ballfit  sitting %s ===' % label)
    lines.append('bag: %s' % bag)
    lines.append('generated: %s' % time.strftime('%Y-%m-%d %H:%M:%S'))
    lines.append('valid-fit jugglebot throws: %d' % len(rows))
    if not rows:
        lines.append('NO valid-fit throws -- nothing to report.')
        return '\n'.join(lines) + '\n'

    span_rows = [r for r in rows if r['span'] > 0.5]
    acc = np.array([r['acc'] for r in span_rows]) if span_rows else np.zeros((0, 2))
    if len(acc) == 0:
        lines.append('NO throw had >=0.5s of tracked span -- cannot estimate '
                      "gravity's direction.")
        return '\n'.join(lines) + '\n'
    # `acc` is the ball's own fitted horizontal acceleration during free
    # flight, i.e. the DOWN/gravity-pull direction's horizontal components
    # (a free body's acceleration vector IS gravity's vector, expressed in
    # frame coordinates). "Lean against gravity" is conventionally expressed
    # against UP (away from the pull) -- the negation -- which is also the
    # convention this probe's platform/gravity comparison must match to
    # agree with the sign of trajectory_node's /gravity_offset tilt (a +tilt
    # there means the platform leans +, not that it accelerates +). Negating
    # here, once, keeps every downstream line (gravity_up, lean,
    # level_trim_deg) in that one convention.
    down_x, down_x_sem, n_g = _mean_sem(np.degrees(acc[:, 0] / G))
    down_y, down_y_sem, _ = _mean_sem(np.degrees(acc[:, 1] / G))
    grav_x, grav_y = -down_x, -down_y
    grav_x_sem, grav_y_sem = down_x_sem, down_y_sem
    lines.append("gravity's UP direction in QTM (deg, from %d free-flight "
                  'fits with span>=0.5s):' % n_g)
    lines.append('  x = %+.4f +/- %.4f   y = %+.4f +/- %.4f'
                  % (grav_x, grav_x_sem, grav_y, grav_y_sem))

    fk = np.array([r['fk_tilt'] for r in rows])
    plat_x, plat_x_sem, n_p = _mean_sem(fk[:, 0])
    plat_y, plat_y_sem, _ = _mean_sem(fk[:, 1])
    lines.append('platform attitude at release, FK of measured /robot_state '
                  '(deg, n=%d):' % n_p)
    lines.append('  x = %+.4f +/- %.4f   y = %+.4f +/- %.4f'
                  % (plat_x, plat_x_sem, plat_y, plat_y_sem))

    lean_x = plat_x - grav_x
    lean_y = plat_y - grav_y
    lean_x_sem = math.hypot(plat_x_sem, grav_x_sem)
    lean_y_sem = math.hypot(plat_y_sem, grav_y_sem)
    lines.append('')
    lines.append("LEAN vs gravity (FK), deg = platform_fk - gravity's UP "
                  'direction:')
    lines.append('  x = %+.4f +/- %.4f   y = %+.4f +/- %.4f'
                  % (lean_x, lean_x_sem, lean_y, lean_y_sem))

    mocap_rows = [r for r in rows if 'mocap_tilt' in r]
    if mocap_rows:
        mc = np.array([r['mocap_tilt'] for r in mocap_rows])
        mplat_x, mplat_x_sem, n_m = _mean_sem(mc[:, 0])
        mplat_y, mplat_y_sem, _ = _mean_sem(mc[:, 1])
        mlean_x, mlean_y = mplat_x - grav_x, mplat_y - grav_y
        lines.append('')
        lines.append('cross-check, mocap platform attitude (deg, n=%d): '
                      'x=%+.4f y=%+.4f  -> lean x=%+.4f y=%+.4f'
                      % (n_m, mplat_x, mplat_y, mlean_x, mlean_y))

    lines.append('')
    lines.append('SIGN CONVENTION (see this file\'s header docstring): '
                  'level_trim_deg is ADDED to the raw /gravity_offset tilt, '
                  'same convention, before the sign flip -- so the trim that '
                  'cancels this lean is the SAME sign as the lean, not '
                  'negated.')
    lines.append('RECOMMENDED level_trim_deg = [%+.4f, %+.4f]  (deg, FK-based; '
                  'paste into trajectory_node\'s level_trim_deg parameter)'
                  % (lean_x, lean_y))
    return '\n'.join(lines) + '\n'


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--bag', required=True, help='rosbag directory (.mcap inside)')
    p.add_argument('--log', required=True,
                   help='launch.log path (accepted for interface symmetry with '
                        'the u2 scratchpad probes; not read by this probe -- '
                        "gravity's direction and the platform FK come from the "
                        'bag alone)')
    p.add_argument('--label', default=None, help='sitting label for the report '
                                                   '(default: bag dirname)')
    p.add_argument('--out', default=None, help='also write the report here')
    p.add_argument('--force', action='store_true', help='ignore the decode cache')
    args = p.parse_args()

    if not os.path.isfile(args.log):
        p.error('--log %r not found' % (args.log,))
    label = args.label or os.path.basename(args.bag.rstrip('/'))
    text = report(args.bag, label, force=args.force)
    print(text)
    if args.out:
        os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
        with open(args.out, 'w') as fh:
            fh.write(text)
        print('wrote %s' % args.out)


if __name__ == '__main__':
    main()
