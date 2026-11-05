#!/usr/bin/env python3
"""The feasibility MARGIN as a continuous scalar along rays of landing offset.

Answers: is the gate margin smooth and monotone in the landing offset (so a
boundary can be found by root-finding instead of a dense grid), where is the TRUE
lateral boundary of the columns cell (the committed box stops at the +-30 mm grid
edge), and which limit binds. Columns cell at separation 125, dwell 0.27, seeded
exactly as the sweep seeds it.

usage: python tools/probes/admissible_obviation/margin_rays.py <level|pose> <apex> <out.json>
          [--limits VEL ACC JERK HAND_ACC] [--rmax MM] [--step MM]

Original notes:

For the columns pose cell (sep 125, dwell 0.27) at several apexes: seed the chain
exactly as the sweep does, then along 8 rays from (0,0) at 1 mm steps out to 40 mm
record, for the CATCH-with-throw cell and the LAUNCH cell: the planner code, the
peaks (leg vel/acc/jerk, hand acc) and the margin scalar
   m = min_k (0.9*L_k - peak_k) / (0.9*L_k)   over k in {leg vel, leg acc, leg jerk}
(m >= 0 <=> the sweep's _within_margin passes). A planner refusal records m = NaN
and the code.

Part of the admissible-box sweep obviation investigation
(``logbook/2026-11-04-admissible-box-sweep-obviation-investigation.md``).
Offline, no hardware, no ROS: imports the production planner + gate exactly as
``tools/admissible_sweep.py`` does. Outputs go under ``temp/probes/admissible_obviation/``.
BLAS is pinned to one thread (the sweep's determinism rule).
"""

import json, math, os, sys, time
for v in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[v] = '1'
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.insert(0, ROOT); sys.path.insert(0, os.path.join(ROOT, 'ros_ws', 'src', 'jugglebot'))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
import numpy as np
import admissible_sweep as asw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.skills import segments as sg, sites as st, schedule as sc, admissible as ab
from jugglebot.motion.trajectory import ballistics_bc as bal

import argparse
ap = argparse.ArgumentParser()
ap.add_argument('mode'); ap.add_argument('apex', type=float); ap.add_argument('out')
ap.add_argument('--limits', type=float, nargs=4, default=[600.0, 8000.0, 200000.0, 4500.0])
ap.add_argument('--rmax', type=float, default=40.0); ap.add_argument('--step', type=float, default=1.0)
args = ap.parse_args(); mode, apex, out = args.mode, args.apex, args.out
geom = StewartGeometry()
limits = asw._limits(*args.limits)
cfg = sg.SegmentConfig(planner_mode=mode)
s0, s1 = st.columns_sites(125.0, 0.0)
T = sc.flight_s(apex)
dwell = 0.27

def peaks(report):
    if report is None:
        return None
    return dict(vel=report.peak_leg_vel_mmps, acc=report.peak_leg_acc_mmps2,
                jerk=report.peak_leg_jerk_mmps3, hand_acc=report.peak_hand_acc_rps2,
                hand_vel=report.peak_hand_vel_rps, code=report.code,
                seat_deg=report.max_cup_seating_deg)

def margin(report):
    if report is None:
        return float('nan')
    f = ab.MARGIN_FRAC
    return min((f*limits.leg_vel_mmps - report.peak_leg_vel_mmps)/(f*limits.leg_vel_mmps),
               (f*limits.leg_acc_mmps2 - report.peak_leg_acc_mmps2)/(f*limits.leg_acc_mmps2),
               (f*limits.leg_jerk_mmps3 - report.peak_leg_jerk_mmps3)/(f*limits.leg_jerk_mmps3))

def catch_throw(site, seed, takeoff, off):
    tau = sc.transit_s(T, dwell); t_release = tau + dwell
    landing = site.catch_site_mm() + np.array([off[0], off[1], 0.0])
    v_arr = bal.arrival_velocity(takeoff, T)
    then = sg.ThrowAfterCatch(t_release_s=t_release, site_mm=site.throw_site_mm(), target_mm=landing, flight_s=T)
    term = sg.CatchTerminal(landing_mm=landing, landing_vel_mm_s=v_arr, t_land_s=tau,
                            rest_site_mm=site.rest_site_mm(), then_throw=then)
    try:
        seg = sg.plan_segment(sg.CATCH, seed, term, cfg, limits, geom)
    except uc.CycleInfeasible as exc:
        return dict(code=exc.code, m=margin(exc.report), peaks=peaks(exc.report), ok=False)
    r = seg.meta.report
    return dict(code='OK' if asw._within_margin(r, limits) else 'MARGIN', m=margin(r), peaks=peaks(r),
                ok=asw._within_margin(r, limits))

def launch(site, off):
    seed = asw._rest_state(site.rest_site_mm())
    target = site.throw_site_mm() + np.array([off[0], off[1], 0.0])
    term = sg.ThrowTerminal(site_mm=site.throw_site_mm(), target_mm=target, flight_s=T, t_release_s=asw.SINGLE_SITE_LAUNCH_S)
    try:
        seg = sg.plan_segment(sg.THROW, seed, term, cfg, limits, geom)
    except uc.CycleInfeasible as exc:
        return dict(code=exc.code, m=margin(exc.report), peaks=peaks(exc.report), ok=False)
    r = seg.meta.report
    return dict(code='OK' if asw._within_margin(r, limits) else 'MARGIN', m=margin(r), peaks=peaks(r),
                ok=asw._within_margin(r, limits))

ok, code, seed, takeoff = asw._throw_cell(s0, s0, T, asw.SINGLE_SITE_LAUNCH_S, limits, geom, cfg)
res = dict(mode=mode, apex=apex, T=T, seed_ok=ok, seed_code=code, rays=[], limits=args.limits)
t0 = time.perf_counter()
if ok:
    for ang in range(0, 360, 45):
        a = math.radians(ang)
        pts = []
        for r_mm in list(np.arange(0.0, args.rmax + 1e-9, args.step)):
            off = (r_mm*math.cos(a), r_mm*math.sin(a))
            pts.append(dict(r=r_mm, off=off, catch=catch_throw(s1, seed, takeoff, off), launch=launch(s1, off)))
        res['rays'].append(dict(angle_deg=ang, points=pts))
        print('ray', ang, 'done', round(time.perf_counter()-t0, 1), 's', flush=True)
res['wall_s'] = time.perf_counter() - t0
json.dump(res, open(out, 'w'))
print('done', out, res['wall_s'])
