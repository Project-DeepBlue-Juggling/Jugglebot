#!/usr/bin/env python3
"""Bump-seeded successor replay (BUG-42): does a BUMP on the previous CATCH-with-
throw raise the NEXT catch's peak Hand acceleration past ``hand_acc_limit_rps2``?

For each ``install_segment BUMP Ball n ... bump (dx, dy) mm`` line of a launch log
that is directly followed by a ``HAND_LIMIT_ACC`` install refusal, this rebuilds
the Columns pair (sep 125, apex 0.95, dwell 0.27, planner pose, hold 0) through
the production entry point ``skills.segments.plan_segment`` the way the LIVE
executor builds it, applies ``bump.apply_catch_bump`` with the logged (dx, dy),
seeds the next CATCH at the release knot with ``release_state_at_knot`` and solves
it (limit lifted to read the peak when the 3900 solve refuses). It prints the
replayed peak next to the logged one.

Ball 2 is the P1 catch (next catch Ball 1 at P2); Ball 1 is the P2 catch (next
catch Ball 2 at P1). The learner apex commands (``U_APEX_*``) and the aim offsets
(``U_B*``) below are the 2026-11-01 memory rows; they are the probe's assumption,
not read from the log. Replay vs live agreed within -1.9..+3.0 % on 2026-11-01
(the Sitting's six HAND_LIMIT_ACC peaks).

usage:
  python tools/probes/bump_seed_replay.py LAUNCH_LOG [--bump-index N] [--root DIR]
    --bump-index N  replay only the Nth (0-based) BUMP line of the log
                    (default: every BUMP followed by a HAND_LIMIT_ACC refusal)
Output: temp/probes/bump_seed_replay/<timestamp>/replay.csv (and stdout).
Derived from the uncommitted night-shift ``jb120_replay.py`` helpers, folded in.
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import os
import re
import sys
import time

for _v in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_v] = '1'

APEX, DWELL, SEP = 0.95, 0.27, 125.0
U_APEX_B2, U_APEX_B1 = 0.8727, 0.8816     # learner apex commands (memory rows)
U_B2, U_B1 = (-11.0, -20.0), (-7.0, -10.0)  # aim offsets of Ball 2 / Ball 1 catches
HAND_LIM = 3900.0   # the hand cap AT the 2026-11-01 Sitting this probe replays (4500 since)

_BUMP_RE = re.compile(r'install_segment BUMP Ball (\d).*?bump \(([-\d.]+), ([-\d.]+)\) mm')
_REFUSE_RE = re.compile(r'install_segment refused: .*?peak hand acceleration ([\d.]+) rev/s\^2'
                        r'.*?\[check=worker:(\w+)')


def parse_log(path):
    """[(line_no, ball, dx, dy, logged_next)] ; logged_next = (code, peak) of the next
    worker refusal after the BUMP, or None when the next worker event is not one."""
    events = []
    for n, line in enumerate(open(path, errors='replace'), 1):
        if '[trajectory_node]' not in line:
            continue
        m = _BUMP_RE.search(line)
        if m:
            events.append(('BUMP', n, int(m.group(1)), float(m.group(2)), float(m.group(3))))
            continue
        if 'install_segment ' in line and 'refused' not in line:
            events.append(('OTHER', n))
            continue
        m = _REFUSE_RE.search(line)
        if m:
            events.append(('REFUSE', n, m.group(2), float(m.group(1))))
    bumps = []
    for i, e in enumerate(events):
        if e[0] != 'BUMP':
            continue
        nxt = events[i + 1] if i + 1 < len(events) else None
        logged = (nxt[2], nxt[3]) if nxt and nxt[0] == 'REFUSE' else None
        bumps.append((e[1], e[2], e[3], e[4], logged))
    return bumps


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('launch_log')
    ap.add_argument('--bump-index', type=int, default=None)
    ap.add_argument('--root', default=os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))))
    a = ap.parse_args()
    root = os.path.abspath(a.root)
    for p in (root, os.path.join(root, 'ros_ws', 'src', 'jugglebot')):
        sys.path.insert(0, p)

    import numpy as np
    import jugglebot.hardware_config as hw
    from jugglebot.motion import unified_cycle as uc
    from jugglebot.motion.geometry import StewartGeometry
    from jugglebot.motion.trajectory import ballistics_bc as bal
    from jugglebot.motion.trajectory import cup_realize as cr
    from jugglebot.motion.trajectory.limits import TrajectoryLimits
    from jugglebot.motion.skills import schedule as sc
    from jugglebot.motion.skills import segments as sg
    from jugglebot.motion.skills import sites as st
    from jugglebot.motion.skills import bump as bp

    geom = StewartGeometry()
    p1, p2 = st.columns_sites(SEP)            # P1 x=-62.5 (Ball 2), P2 x=+62.5 (Ball 1)
    tp = sc.flight_s(APEX)
    tau = sc.transit_s(tp, DWELL)
    cfg = dataclasses.replace(sg.SegmentConfig(planner_mode='pose'), pre_release_hold_s=0.0)

    def limits(hand=HAND_LIM):
        return TrajectoryLimits.from_config(hw).with_sitting_limits(
            leg_vel_mmps=float(hw.JB_TRAJ_LEG_VEL_LIMIT_MMPS),
            leg_acc_mmps2=float(hw.JB_TRAJ_LEG_ACC_LIMIT_MMPS2),
            leg_jerk_mmps3=float(hw.JB_TRAJ_LEG_JERK_LIMIT_MMPS3),
            hand_acc_rps2=float(hand))

    def mm(v):
        return np.array([float(v[0]), float(v[1]), 0.0])

    def rest_state(cup_mm):
        rcfg = cr.RealizeConfig()
        rev = (((float(cup_mm[2]) - rcfg.cup_z_base_mm) - rcfg.slider_rev_zero_mm)
               / 1000.0) * cr.HAND_REV_PER_M
        pose = np.array([cup_mm[0], cup_mm[1], rcfg.active_z_mm, 0.0, 0.0, 0.0])
        return uc.CycleState.at_rest(pose, rev, rcfg)

    def launch_seed(site):
        term = sg.ThrowTerminal(site_mm=site.throw_site_mm(), target_mm=site.throw_site_mm(),
                                flight_s=tp, t_release_s=0.4)
        seg = sg.plan_segment(sg.THROW, rest_state(site.rest_site_mm()), term, cfg,
                              limits(), geom)
        return seg.release_state

    def catch_terminal(site, catch_off, u_mm, u_apex):
        """executor._catch_terminal's live shape (same-site fold)."""
        landing = site.catch_site_mm() + mm(catch_off)
        release = np.array([landing[0], landing[1], site.throw_site_mm()[2]])
        r = release[:2] - np.asarray(site.cup_mm, float)[:2]
        g = min(max(APEX / u_apex, 0.5), 2.0)
        target = site.catch_site_mm() + mm(u_mm) + mm(r * (1.0 - 1.0 / g))
        v_launch = bal.launch_velocity(site.throw_site_mm(), landing, tp)
        v_arr = bal.arrival_velocity(v_launch, tp)
        tt = sg.ThrowAfterCatch(t_release_s=tau + DWELL, site_mm=release,
                                target_mm=target, flight_s=sc.flight_s(u_apex))
        return sg.CatchTerminal(landing_mm=landing, landing_vel_mm_s=v_arr, t_land_s=tau,
                                rest_site_mm=site.rest_site_mm(), then_throw=tt)

    def solve_peak(seed, term):
        try:
            seg = sg.plan_segment(sg.CATCH, seed, term, cfg, limits(), geom)
            return 'OK', float(seg.meta.report.peak_hand_acc_rps2)
        except uc.CycleInfeasible as exc:
            code = exc.code
        try:
            seg = sg.plan_segment(sg.CATCH, seed, term, cfg, limits(1e9), geom)
            return code, float(seg.meta.report.peak_hand_acc_rps2)
        except uc.CycleInfeasible as exc2:
            rep = getattr(exc2, 'report', None)
            return code, (float(rep.peak_hand_acc_rps2) if rep is not None else float('nan'))

    bumps = parse_log(a.launch_log)
    if a.bump_index is not None:
        picks = [bumps[a.bump_index]]
    else:
        picks = [b for b in bumps if b[4] and b[4][0] == 'HAND_LIMIT_ACC']
    print('%d BUMP lines in %s; replaying %d' % (len(bumps), a.launch_log, len(picks)))

    rows = []
    for (line, ball, dx, dy, logged) in picks:
        if ball == 2:
            prev_site, other, prev_u, prev_apex, next_site, next_u, next_apex = (
                p1, p2, U_B2, U_APEX_B2, p2, U_B1, U_APEX_B1)
        else:
            prev_site, other, prev_u, prev_apex, next_site, next_u, next_apex = (
                p2, p1, U_B1, U_APEX_B1, p1, U_B2, U_APEX_B2)
        s1 = launch_seed(other)
        seg = sg.plan_segment(sg.CATCH, s1,
                              catch_terminal(prev_site, (0.0, 0.0), prev_u, prev_apex),
                              cfg, limits(), geom)
        dt = float(seg.plan.dt)
        k_rel = int(round(seg.release_t_s / dt))
        t_c = float(seg.event_t_s)
        r = bp.apply_catch_bump(seg.plan, seg.meta, -1, (dx, dy), t_c, limits(), geom)
        if not r.accepted:
            print('L%d Ball %d bump (%.1f, %.1f): offline bump DECLINED %s' % (
                line, ball, dx, dy, r.code))
            rows.append(dict(line=line, ball=ball, dx=dx, dy=dy, code='BUMP_DECLINED',
                             replay_peak='', logged_code='', logged_peak='', diff_pct=''))
            continue
        seed = uc.release_state_at_knot(r.plan, r.meta, k_rel)
        code, peak = solve_peak(seed, catch_terminal(next_site, (0.0, 0.0), next_u, next_apex))
        lcode, lpeak = logged if logged else ('', float('nan'))
        diff = 100.0 * (peak - lpeak) / lpeak if logged else float('nan')
        print('L%-5d Ball %d bump (%5.1f,%5.1f) | replay %-15s %7.1f (%5.1f %%) | logged %-15s %7.1f | %+5.1f %%'
              % (line, ball, dx, dy, code, peak, 100 * peak / HAND_LIM, lcode, lpeak, diff),
              flush=True)
        rows.append(dict(line=line, ball=ball, dx=dx, dy=dy, code=code,
                         replay_peak=round(peak, 1), logged_code=lcode,
                         logged_peak=lpeak, diff_pct=round(diff, 1)))

    out_dir = os.path.join(root, 'temp', 'probes', 'bump_seed_replay', time.strftime('%Y%m%d-%H%M%S'))
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, 'replay.csv')
    with open(out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else ['line'])
        w.writeheader()
        w.writerows(rows)
    print('wrote', out)


if __name__ == '__main__':
    main()
