#!/usr/bin/env python3
"""Two tests of the proposed online admissibility check (logbook 2026-11-04, Outcome 1).

A. **Corpus replay.** Every row of the Learner corpus (``temp/learn/jugglebot/memory.csv``,
   one flown throw per row: release site xy ``x0, x1``, commanded landing offset
   ``u0, u1`` and apex ``u2``) is re-planned as the columns catch-with-throw it stood
   for: the partner's release at the mirrored site seeds the chain (the sweep's own
   seeding), the pattern flight is the row's commanded apex, dwell 0.27, pose planner,
   the limits given. Counts how many flown commands the online check would have passed
   at the 10 % margin, how many it would have refused, and by which code. An
   APPROXIMATION of the live seed (the real partner release state is not in the corpus).

B. **The two-solve step.** For candidates along rays (10, 20, 30 mm, 8 directions) that
   the online check refuses, the proposed fallback: read the margin of the nominal plan
   (one real solve, shared) and of the affinely predicted plan at the candidate (gate
   only, no solve), interpolate the margin's zero along the line toward the nominal
   command, back off ``--backoff`` mm, and CONFIRM with one real solve. Reports how often
   the confirmed point passes (so the check ends at two real solves) and how far it
   had to move.

usage: python tools/probes/admissible_obviation/online_check_replay.py
          [--csv temp/learn/jugglebot/memory.csv] [--limits VEL ACC JERK HAND_ACC]
          [--mode pose] [--max-rows N] [--backoff MM]

Part of the admissible-box sweep obviation investigation
(``logbook/2026-11-04-admissible-box-sweep-obviation-investigation.md``).
Offline, no hardware, no ROS. Outputs under ``temp/probes/admissible_obviation/``.
"""
import argparse, collections, csv, json, math, os, sys, time
for _v in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_v] = '1'
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.insert(0, ROOT); sys.path.insert(0, os.path.join(ROOT, 'ros_ws', 'src', 'jugglebot'))
sys.path.insert(0, os.path.join(ROOT, 'tools', 'probes'))
import numpy as np                                                    # noqa: E402
import planner_cells as asw                                        # noqa: E402
from jugglebot.motion import unified_cycle as uc                      # noqa: E402
from jugglebot.motion.geometry import StewartGeometry                 # noqa: E402
from jugglebot.motion.skills import segments as sg, sites as st, schedule as sc, admissible as ab  # noqa: E402
from jugglebot.motion.trajectory import ballistics_bc as bal          # noqa: E402
from jugglebot.motion.trajectory import feasibility as fz             # noqa: E402
from jugglebot.motion.trajectory.cycle_plan import CyclePlan          # noqa: E402

DWELL = 0.27


def margin(report, limits):
    if report is None:
        return float('nan')
    f = ab.MARGIN_FRAC
    return min((f * limits.leg_vel_mmps - report.peak_leg_vel_mmps) / (f * limits.leg_vel_mmps),
               (f * limits.leg_acc_mmps2 - report.peak_leg_acc_mmps2) / (f * limits.leg_acc_mmps2),
               (f * limits.leg_jerk_mmps3 - report.peak_leg_jerk_mmps3) / (f * limits.leg_jerk_mmps3))


class Cell:
    """One columns catch-with-throw cell: site, mirrored partner site, pattern flight."""

    def __init__(self, site_xy_mm, apex_m, limits, geom, cfg):
        """``apex_m`` is the PATTERN apex: it sets the seed throw, the transit and the
        incoming ball; ``solve`` takes the COMMANDED flight separately (the sweep's own
        R5 split of pattern timing from the Learner's command)."""
        self.site = st.Site('S', np.array([site_xy_mm[0], site_xy_mm[1], st.CATCH_CUP_Z_MM]))
        self.partner = st.Site('Q', np.array([-site_xy_mm[0], -site_xy_mm[1], st.CATCH_CUP_Z_MM]))
        self.T = sc.flight_s(apex_m)
        self.limits, self.geom, self.cfg = limits, geom, cfg
        self.ok, self.code, self.seed, self.takeoff = asw.throw_cell(
            self.partner, self.partner, self.T, asw.SINGLE_SITE_LAUNCH_S, limits, geom, cfg)

    def solve(self, off_mm, cmd_flight_s=None):
        tau = sc.transit_s(self.T, DWELL)
        landing = self.site.catch_site_mm() + np.array([off_mm[0], off_mm[1], 0.0])
        then = sg.ThrowAfterCatch(t_release_s=tau + DWELL, site_mm=self.site.throw_site_mm(),
                                  target_mm=landing,
                                  flight_s=self.T if cmd_flight_s is None else float(cmd_flight_s))
        term = sg.CatchTerminal(landing_mm=landing, landing_vel_mm_s=bal.arrival_velocity(self.takeoff, self.T),
                                t_land_s=tau, rest_site_mm=self.site.rest_site_mm(), then_throw=then)
        try:
            seg = sg.plan_segment(sg.CATCH, self.seed, term, self.cfg, self.limits, self.geom)
        except uc.CycleInfeasible as exc:
            return None, exc.code, margin(exc.report, self.limits)
        m = margin(seg.meta.report, self.limits)
        return seg, ('OK' if m >= 0 else 'MARGIN'), m


def part_a(a, limits, geom, cfg):
    rows = list(csv.DictReader(open(a.csv)))
    if a.max_rows:
        rows = rows[:a.max_rows]
    cells = {}
    verdict = collections.Counter(); seed_fail = collections.Counter(); margins = []
    t0 = time.perf_counter()
    for r in rows:
        if abs(float(r['x0'])) >= 0.1:
            verdict['SKIPPED_HOP_SITE'] += 1      # the hop's cross-site throw is not this cell's shape
            continue
        cmd_apex = round(float(r['u2_apex_m']), 2)
        pat_apex = a.pattern_apex if a.pattern_apex else cmd_apex
        key = (round(float(r['x0']) * 1000, 1), round(float(r['x1']) * 1000, 1), pat_apex)
        if key not in cells:
            cells[key] = Cell((key[0], key[1]), key[2], limits, geom, cfg)
        c = cells[key]
        if not c.ok:
            seed_fail[c.code] += 1; verdict['SEED:' + c.code] += 1
            continue
        _seg, code, m = c.solve((float(r['u0']) * 1000, float(r['u1']) * 1000), sc.flight_s(cmd_apex))
        verdict[code] += 1
        if not math.isnan(m):
            margins.append(m)
    wall = time.perf_counter() - t0
    out = dict(rows=len(rows), pattern_apex=a.pattern_apex, distinct_cells=len(cells), verdict=dict(verdict), seed_failures=dict(seed_fail),
               margin_p01=float(np.percentile(margins, 1)) if margins else None,
               margin_p50=float(np.median(margins)) if margins else None, wall_s=wall,
               ms_per_row=1e3 * wall / max(len(rows), 1))
    print('A. corpus replay:', json.dumps(out))
    return out


def part_b(a, limits, geom, cfg):
    results = []
    for apex in (0.95, 1.05):
        c = Cell((62.5, 0.0), apex, limits, geom, cfg)
        if not c.ok:
            print('B. seed refused', apex, c.code); continue
        seg0, code0, m0 = c.solve((0.0, 0.0))
        if seg0 is None or m0 < 0:
            print('B. nominal command itself refused (%s, m=%.3f) at apex %.2f: NO_ADMISSIBLE_COMMAND path' % (code0, m0, apex))
            continue
        # affine map from three solves (the nominal plus two unit steps)
        def arrs(seg):
            p = seg.plan
            return dict(pose=np.asarray(p.pose), pose_vel=np.asarray(p.pose_vel),
                        hand_rev=np.asarray(p.hand_rev), hand_vel_rps=np.asarray(p.hand_vel_rps))
        A0 = arrs(seg0); Ax = arrs(c.solve((1.0, 0.0))[0]); Ay = arrs(c.solve((0.0, 1.0))[0])
        K = {k: (Ax[k] - A0[k], Ay[k] - A0[k]) for k in A0}
        p0 = seg0.plan

        def predicted_margin(off):
            pred = {k: A0[k] + K[k][0] * off[0] + K[k][1] * off[1] for k in A0}
            plan = CyclePlan(pred['pose'], pred['pose_vel'], pred['hand_rev'], pred['hand_vel_rps'], p0.dt,
                             catch_k=p0.catch_k, contact_knots=p0.contact_knots,
                             catch_evidence=p0.catch_evidence, release_step_knots=p0.release_step_knots)
            return margin(fz.validate_cycle(plan, limits, geom), limits)
        for r_mm in (10.0, 20.0, 30.0):
            for ang in range(0, 360, 45):
                u = (r_mm * math.cos(math.radians(ang)), r_mm * math.sin(math.radians(ang)))
                seg, code, m = c.solve(u)                      # real solve 1: the candidate
                row = dict(apex=apex, r=r_mm, ang=ang, candidate_code=code, candidate_m=m, real_solves=1)
                if seg is not None and m >= 0:
                    row['result'] = 'candidate passes'
                else:
                    mp = predicted_margin(u)                   # gate only, no solve
                    if math.isnan(mp) or mp >= 0 or m0 <= 0:
                        s = 0.5                                # no usable slope: halve
                    else:
                        s = m0 / (m0 - mp)                     # zero crossing along the line
                    s = max(0.0, min(1.0, s))
                    back = a.backoff / max(r_mm, 1e-9)
                    s2 = max(0.0, s - back)
                    u2 = (u[0] * s2, u[1] * s2)
                    seg2, code2, m2 = c.solve(u2)              # real solve 2: confirm
                    row.update(real_solves=2, predicted_margin=mp, step_frac=s2, confirm_code=code2, confirm_m=m2,
                               result=('confirmed at %.1f mm' % (r_mm * s2) if (seg2 is not None and m2 >= 0)
                                       else 'confirm FAILED'))
                results.append(row)
    summary = collections.Counter(r['result'].split(' at')[0] for r in results)
    two_solve = [r for r in results if r['real_solves'] == 2]
    print('B. two-solve step: candidates %d, passed outright %d, needed the step %d, step confirmed %d, '
          'step failed %d' % (len(results), summary.get('candidate passes', 0), len(two_solve),
                              sum(1 for r in two_solve if r['result'].startswith('confirmed')),
                              summary.get('confirm FAILED', 0)))
    for r in two_solve:
        print('   apex %.2f r %4.0f ang %3d: candidate %s m=%.3f | predicted m=%.3f -> step to %.1f mm | confirm %s m=%.3f' % (
            r['apex'], r['r'], r['ang'], r['candidate_code'], r['candidate_m'], r['predicted_margin'],
            r['r'] * r['step_frac'], r['confirm_code'], r['confirm_m']))
    return results


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', default=os.path.join(ROOT, 'temp', 'learn', 'jugglebot', 'memory.csv'))
    ap.add_argument('--limits', type=float, nargs=4, default=[600.0, 8000.0, 200000.0, 4500.0])
    ap.add_argument('--mode', default='pose', choices=('level', 'pose'))
    ap.add_argument('--max-rows', type=int, default=0)
    ap.add_argument('--backoff', type=float, default=1.0)
    ap.add_argument('--skip-a', action='store_true')
    ap.add_argument('--skip-b', action='store_true')
    ap.add_argument('--pattern-apex', type=float, default=None,
                    help='use this PATTERN apex for every row (the corpus has only the commanded apex); '
                         'default: the row\'s commanded apex, which shortens the transit and is conservative')
    a = ap.parse_args(argv)
    geom = StewartGeometry(); limits = asw.sitting_limits(*a.limits); cfg = sg.SegmentConfig(planner_mode=a.mode)
    out = dict(limits=a.limits, mode=a.mode)
    if not a.skip_a and os.path.exists(a.csv):
        out['corpus'] = part_a(a, limits, geom, cfg)
    if not a.skip_b:
        out['two_solve'] = part_b(a, limits, geom, cfg)
    path = os.path.join(ROOT, 'temp', 'probes', 'admissible_obviation',
                        'online_check_replay_%s_%s.json' % (a.mode, time.strftime('%Y%m%d_%H%M%S')))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    json.dump(out, open(path, 'w'), indent=1, default=str)
    print('wrote', path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
