#!/usr/bin/env python3
"""Is the planned segment AFFINE in the landing offset, and does gating the affinely
predicted trajectory give the same verdict as gating the real solve?

Two measurements on the columns catch-with-throw cell (separation 125, dwell 0.27,
seeded exactly as ``tools/probes/planner_cells.py`` seeds it):

1. **Active-set map.** Solve every offset of the sweep's 17x17 grid and record the
   Goldfarb-Idnani final active set and iteration count (``Segment.warm_start``).
   One active set over the whole grid means the QP optimiser is EXACTLY affine in
   the offset there (multi-parametric QP: one critical region).
2. **Affine prediction.** From three solves ((0,0), (10,0), (0,10)) build the
   finite-difference map of every plan array (pose, pose_vel, hand_rev,
   hand_vel_rps), predict the plan at other offsets, run ``validate_cycle`` on the
   predicted plan and compare peaks and verdict with the real solve. The QP half is
   affine; the tilt schedule / decomposition after it is not, so the error grows
   with the offset. Reports the max array error and the gate-only cost.

usage: python tools/probes/admissible_obviation/affine_check.py [--mode level|pose]
          [--apex M] [--limits VEL ACC JERK HAND_ACC] [--out JSON]

Part of the admissible-box sweep obviation investigation
(``logbook/2026-11-04-admissible-box-sweep-obviation-investigation.md``).
Offline, no hardware, no ROS. Outputs under ``temp/probes/admissible_obviation/``.
"""
import argparse, collections, json, os, sys, time
for _v in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_v] = '1'
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.insert(0, ROOT); sys.path.insert(0, os.path.join(ROOT, 'ros_ws', 'src', 'jugglebot'))
sys.path.insert(0, os.path.join(ROOT, 'tools', 'probes'))
import numpy as np                                                    # noqa: E402
import planner_cells as asw                                        # noqa: E402
from jugglebot.motion import unified_cycle as uc                      # noqa: E402
from jugglebot.motion.geometry import StewartGeometry                 # noqa: E402
from jugglebot.motion.skills import segments as sg, sites as st, schedule as sc  # noqa: E402
from jugglebot.motion.trajectory import ballistics_bc as bal          # noqa: E402
from jugglebot.motion.trajectory import feasibility as fz             # noqa: E402
from jugglebot.motion.trajectory.cycle_plan import CyclePlan          # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', default='pose', choices=('level', 'pose'))
    ap.add_argument('--apex', type=float, default=0.95)
    ap.add_argument('--limits', type=float, nargs=4, default=[600.0, 8000.0, 200000.0, 4500.0])
    ap.add_argument('--out', default=None)
    a = ap.parse_args(argv)
    geom = StewartGeometry()
    limits = asw.sitting_limits(*a.limits)
    cfg = sg.SegmentConfig(planner_mode=a.mode)
    s0, s1 = st.columns_sites(125.0, 0.0)
    T = sc.flight_s(a.apex)
    dwell = 0.27
    ok, code, seed, takeoff = asw.throw_cell(s0, s0, T, asw.SINGLE_SITE_LAUNCH_S, limits, geom, cfg)
    if not ok:
        print('seed throw refused:', code)
        return 1

    def plan(off):
        tau = sc.transit_s(T, dwell)
        landing = s1.catch_site_mm() + np.array([off[0], off[1], 0.0])
        then = sg.ThrowAfterCatch(t_release_s=tau + dwell, site_mm=s1.throw_site_mm(),
                                  target_mm=landing, flight_s=T)
        term = sg.CatchTerminal(landing_mm=landing, landing_vel_mm_s=bal.arrival_velocity(takeoff, T),
                                t_land_s=tau, rest_site_mm=s1.rest_site_mm(), then_throw=then)
        return sg.plan_segment(sg.CATCH, seed, term, cfg, limits, geom)

    # 1. active-set map over the sweep grid
    acts = collections.Counter(); iters = collections.Counter(); refused = collections.Counter()
    for dx in asw.OFFSETS_MM:
        for dy in asw.OFFSETS_MM:
            try:
                seg = plan((dx, dy))
            except uc.CycleInfeasible as exc:
                refused[exc.code] += 1
                continue
            acts[tuple(sorted(seg.warm_start.active))] += 1
            iters[seg.warm_start.iters] += 1
    res = dict(mode=a.mode, apex=a.apex, limits=a.limits, grid_cells=len(asw.OFFSETS_MM) ** 2,
               refusals=dict(refused), distinct_active_sets=len(acts),
               active_sets={str(k): v for k, v in acts.items()}, iterations=dict(iters))
    print('active-set map:', {k: res[k] for k in ('grid_cells', 'refusals', 'distinct_active_sets', 'iterations')},
          'sets', acts.most_common(3))

    # 2. affine prediction + gate on the predicted plan
    def arrs(seg):
        p = seg.plan
        return dict(pose=np.asarray(p.pose), pose_vel=np.asarray(p.pose_vel),
                    hand_rev=np.asarray(p.hand_rev), hand_vel_rps=np.asarray(p.hand_vel_rps))
    A0 = arrs(plan((0, 0))); Ax = arrs(plan((10, 0))); Ay = arrs(plan((0, 10)))
    K = {k: ((Ax[k] - A0[k]) / 10.0, (Ay[k] - A0[k]) / 10.0) for k in A0}
    rows = []
    for off in [(4, 0), (10, 10), (20, 0), (-20, 10), (30, -30), (-30, 30), (60, 0), (-60, -60)]:
        try:
            seg = plan(off)
        except uc.CycleInfeasible as exc:
            rows.append(dict(off=off, true_code=exc.code, note='real solve refused'))
            print(off, 'real solve refused', exc.code)
            continue
        A = arrs(seg); p = seg.plan
        pred = {k: A0[k] + K[k][0] * off[0] + K[k][1] * off[1] for k in A0}
        err = max(float(np.abs(A[k] - pred[k]).max()) for k in A0)
        plan_pred = CyclePlan(pred['pose'], pred['pose_vel'], pred['hand_rev'], pred['hand_vel_rps'], p.dt,
                              catch_k=p.catch_k, contact_knots=p.contact_knots,
                              catch_evidence=p.catch_evidence, release_step_knots=p.release_step_knots)
        rt = fz.validate_cycle(p, limits, geom)
        t0 = time.perf_counter(); rp = fz.validate_cycle(plan_pred, limits, geom); t_gate = time.perf_counter() - t0
        row = dict(off=off, max_affine_err=err, gate_ms=1e3 * t_gate,
                   true=dict(code=rt.code, margin_ok=asw.within_margin(rt, limits), vel=rt.peak_leg_vel_mmps,
                             acc=rt.peak_leg_acc_mmps2, jerk=rt.peak_leg_jerk_mmps3),
                   pred=dict(code=rp.code, margin_ok=asw.within_margin(rp, limits), vel=rp.peak_leg_vel_mmps,
                             acc=rp.peak_leg_acc_mmps2, jerk=rp.peak_leg_jerk_mmps3))
        rows.append(row)
        print(off, 'max affine err %.1e' % err, '| acc true/pred %.0f/%.0f jerk %.0f/%.0f' % (
            row['true']['acc'], row['pred']['acc'], row['true']['jerk'], row['pred']['jerk']),
            '| verdict true %s/%s pred %s/%s | gate %.2f ms' % (
            rt.code, row['true']['margin_ok'], rp.code, row['pred']['margin_ok'], 1e3 * t_gate))
    res['affine_rows'] = rows
    out = a.out or os.path.join(ROOT, 'temp', 'probes', 'admissible_obviation',
                                'affine_check_%s_%.2f_%s.json' % (a.mode, a.apex, time.strftime('%Y%m%d_%H%M%S')))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    json.dump(res, open(out, 'w'), indent=1)
    print('wrote', out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
