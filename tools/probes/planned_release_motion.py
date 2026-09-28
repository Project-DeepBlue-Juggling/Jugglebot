#!/usr/bin/env python3
"""What the PLAN asks of the PLATFORM in the knots either side of a release.

Motivated by ``logbook/2026-09-28-skill-stack-r4-sitting-1-analysis.md`` fact 3:
on every throw of the 2026-09-27 22:37 R4 sitting the ball separated 20-40 ms
AFTER the planned release, and the plan re-accelerated the centroid toward the
next site the instant the release knot passed (+33 mm/s at +0.05 s, +115 at
+0.10, +196 at +0.15, +276 at +0.20, from a planned centroid velocity of 0 at
the release knot itself).  For a 250 mm hop that put ~+95 mm/s of the
platform's return into the ball and overshot the far site by 87-104 mm on 3/3
attempts.

This is the offline harness for the fix (``SegmentConfig.post_release_hold_s``,
``CycleGoals.hold_platform_knots``): it drives the REAL install chain (no
plant, no ROS) for a 250 mm hop and for a self-toss whose every catch is
re-aimed +20 mm in y, and prints, per release, the platform pose / pose
velocity / tilt / hand for the knots around it, plus the HOLD verdict — the
worst centroid speed, centroid displacement and tilt change over the knots that
follow the release.

Underpins ``tests/motion/test_skills_segments.py`` §
``test_a_tilted_throws_tail_holds_the_cup_on_the_frozen_stroke_line`` and
``test_a_post_release_landing_holds_the_platform_after_the_release`` (the numbers those tests pin are
this probe's output) and the 2026-09-28 unit-B logbook entry.

Run (repo root, venv):  python tools/probes/planned_release_motion.py
Writes a timestamped copy of its stdout to temp/probes/.
"""
from __future__ import annotations

import datetime as _dt
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))
for p in (ROOT, os.path.join(ROOT, 'ros_ws', 'src', 'jugglebot'),
          os.path.join(ROOT, 'tools')):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np                                            # noqa: E402
import jugglebot.hardware_config as hw                        # noqa: E402
from jugglebot.motion import unified_cycle as uc              # noqa: E402
from jugglebot.motion.geometry import StewartGeometry         # noqa: E402
from jugglebot.motion.trajectory import ballistics_bc as bal  # noqa: E402
from jugglebot.motion.skills import executor as ex            # noqa: E402
from jugglebot.motion.skills import schedule as sc            # noqa: E402
from jugglebot.motion.skills import segments as sg            # noqa: E402
from jugglebot.motion.skills import sites as si               # noqa: E402
import admissible_sweep as aw                                 # noqa: E402

DT = float(hw.JB_TRAJ_KNOT_DT_S)
#: How many knots after the release the HOLD verdict is measured over — the
#: knots ``SegmentConfig.post_release_hold_s`` is meant to freeze, read from the
#: ONE conversion the planner itself uses (2 knots at the 2026-09-28 sizing).
HOLD_CHECK_KNOTS = int(uc.post_release_hold_knots())

geom = StewartGeometry()
# The launch's own session limits (leg 300 mm/s, 5000 mm/s^2, 150000 mm/s^3;
# hand 3500 rev/s^2) — the operating point of the 2026-09-27 sitting.
limits = aw._limits(300.0, 5000.0, 150000.0, 3500.0)

_OUT = []


def say(msg=''):
    print(msg)
    _OUT.append(msg)


def run(sites, n_throws, landing_offset_mm, label, apex=0.9, dwell=0.30,
        t0_abs=10.0):
    pattern = sc.OneBallPattern(sites=sites, apex_m=apex, dwell_s=dwell,
                               n_throws=n_throws)
    schedule = sc.compile_one_ball(pattern, t0_abs)
    T = schedule.flight_s
    landings, releases = [], []
    prev_site = None
    for sk in schedule.skills:
        if sk.kind == sg.THROW:
            prev_site = sk.site
            releases.append(float(sk.t_abs_s))
        if sk.kind == sg.CATCH:
            v0 = bal.launch_velocity(prev_site.throw_site_mm(),
                                     sk.site.catch_site_mm(), T)
            pos = (np.asarray(sk.site.catch_site_mm(), dtype=float)
                   + np.asarray(landing_offset_mm, dtype=float))
            landings.append(ex.Landing(pos_mm=pos,
                                       vel_mm_s=bal.arrival_velocity(v0, T),
                                       t_land_abs_s=float(sk.t_abs_s),
                                       from_fit=True))
            if sk.then_throw is not None:
                prev_site = sk.site
                releases.append(float(sk.then_throw.t_release_abs_s))
    clock = {'t': t0_abs - 1.0}

    def tracker(ball_id):
        for l in landings:
            if l.t_land_abs_s > clock['t'] - 0.05:
                return l
        return None

    state = {'record': None}
    cfg = sg.SegmentConfig()
    records = []

    def installer(kind, terminal, t_now_s, ball_id=0):
        rec = state['record']
        seed = aw._rest_state(sites[0].rest_site_mm()) if rec is None else None
        new_rec, res, seg = ex.install_segment(rec, seed, kind, terminal,
                                              t_now_s, cfg=cfg, limits=limits,
                                              geom=geom)
        if res.accepted:
            state['record'] = new_rec
            records.append((kind, t_now_s, new_rec, seg))
        else:
            say('  REFUSED %s %s: %s' % (kind, res.code, res.message[:200]))
        return res

    execu = ex.SkillExecutor(schedule, installer, tracker=tracker,
                             catch_aim_source=ex.AIM_TRACKER)
    t_end = max(sk.t_abs_s for sk in schedule.skills) + 0.5
    t = clock['t']
    while t < t_end and not execu.attempt_ended:
        clock['t'] = t
        execu.tick(t)
        t += DT / 10.0
    say('\n=== %s: %d skills, ended=%s code=%s, %d installs, releases at %s ==='
        % (label, len(schedule.skills), execu.attempt_ended, execu.end_code,
           len(records), ['%.3f' % r for r in releases]))
    for kind, t_now, rec, seg in records:
        rep = seg.meta.report
        say('  install %-5s at %.3f: leg vel %7.1f mm/s  acc %8.1f mm/s^2  '
            'jerk %9.1f mm/s^3  hand acc %7.1f rev/s^2'
            % (kind, t_now, rep.peak_leg_vel_mmps, rep.peak_leg_acc_mmps2,
               rep.peak_leg_jerk_mmps3, rep.peak_hand_acc_rps2))
    for i, t_rel in enumerate(releases[:3]):
        # the LAST record installed whose span covers the release (the latest re-aim)
        cover = [r for (_k, _t, r, _s) in records if r.t0_s <= t_rel < r.end_s]
        if not cover:
            say('\n-- release %d at t=%.3f: no record covers it --' % (i, t_rel))
            continue
        final = cover[-1]
        plan = final.plan
        pose = np.asarray(plan.pose)
        vel = np.asarray(plan.pose_vel)
        hand = np.asarray(plan.hand_rev)
        hvel = np.asarray(plan.hand_vel_rps)
        k = int(round((t_rel - final.t0_s) / DT))
        say('\n-- release %d at t=%.3f -> knot %d of the covering record '
            '(t0 %.3f, %d knots, installed for %s at %.3f) --'
            % (i, t_rel, k, final.t0_s, len(pose),
               [kk for (kk, _t, r, _s) in records if r is final][0],
               [_t for (kk, _t, r, _s) in records if r is final][0]))
        say('   dt(s)   cx(mm)  cy(mm)  cz(mm)  tiltx(deg) tilty(deg)  '
            'vx(mm/s) vy(mm/s) vz(mm/s)  hand(rev) hvel(rps)')
        for kk in list(range(k - 24, k, 2)) + list(range(k, k + 9)):
            if 0 <= kk < len(pose):
                p, v = pose[kk], vel[kk]
                say('  %+.3f  %7.1f %7.1f %7.1f   %+7.3f   %+7.3f    '
                    '%+7.1f  %+7.1f  %+7.1f   %6.3f  %+6.2f'
                    % ((kk - k) * DT, p[0], p[1], p[2], math.degrees(p[3]),
                       math.degrees(p[4]), v[0], v[1], v[2], hand[kk],
                       hvel[kk]))
        if 0 <= k < len(pose):
            cv = uc.cup_velocity_from_platform(pose[k], vel[k], hand[k],
                                               hvel[k])
            say('  planned CUP velocity at the release knot: (%.0f, %.0f, %.0f)'
                ' mm/s' % tuple(np.asarray(cv).ravel()[:3]))
            hold = [kk for kk in range(k + 1, k + 1 + HOLD_CHECK_KNOTS)
                    if kk < len(pose)]
            if hold:
                worst_v = max(float(np.hypot(vel[kk][0], vel[kk][1]))
                              for kk in hold)
                worst_dp = max(float(np.linalg.norm(pose[kk][:3] - pose[k][:3]))
                               for kk in hold)
                worst_dtilt = max(float(np.hypot(*(pose[kk][3:5]
                                                   - pose[k][3:5])))
                                  for kk in hold)
                say('  HOLD verdict over knots +1..+%d: worst centroid speed '
                    '%.4f mm/s, worst centroid displacement %.4f mm, worst '
                    'tilt change %.6f deg'
                    % (len(hold), worst_v, worst_dp,
                       math.degrees(worst_dtilt)))


def main():
    say('post_release_hold_s = %r (SegmentConfig default), dt = %.4f s'
        % (getattr(sg.SegmentConfig(), 'post_release_hold_s', None), DT))
    P1, P2 = si.columns_sites(250.0)
    run((P1, P2), 1, (0.0, 0.0, 0.0),
        'HOP 250 mm, single throw P1->P2, perfect landing')
    S = si.columns_sites(100.0)[0]
    run((S,), 3, (0.0, 0.0, 0.0), 'SELF-TOSS x3, landings ON the site')
    run((S,), 3, (0.0, 20.0, 0.0),
        'SELF-TOSS x3, every landing +20 mm in y (lateral re-aim)')
    out_dir = os.path.join(ROOT, 'temp', 'probes')
    os.makedirs(out_dir, exist_ok=True)
    stamp = _dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    path = os.path.join(out_dir, 'planned_release_motion_%s.txt' % stamp)
    with open(path, 'w') as fh:
        fh.write('\n'.join(_OUT) + '\n')
    print('\nwrote %s' % path)


if __name__ == '__main__':
    main()
