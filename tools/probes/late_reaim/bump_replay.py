"""Offline prototype of the late catch re-aim BUMP (plans/archived/late-catch-reaim-bump.md).

For every (delta, time-to-landing) a live re-send carried on 2026-10-16
(``parse_resends.py`` output), apply a platform-xy offset to the plan record a
sim columns run had installed at that instant (``capture_records.py``) and run
``feasibility.validate_cycle`` on the bumped span ``[k_b-4, end]`` at the live
sitting limits.  Prints to stdout only.  The hand arrays are never touched.

Shapes (all zero, with zero velocity/acceleration, through ``k_b`` = re-send
+ ``--lead-knots``):

  * ``--retarget`` (THE DESIGN): calls the PRODUCTION enforcement point
    ``jugglebot.motion.skills.bump.apply_catch_bump`` (I-CATCH-5; one
    implementation since 2026-10-16 G1, the probe's own copy deleted) with
    k_b knot-aligned at floor(t/dt) + ``--lead-knots`` and the 50 mm/s launch
    bound (``--no-launch-bound`` lifts it).  Shape: quintic ramp to delta
    ending ``--hold-s`` before the catch; held through the carried release
    with a lateral velocity ramp over the carry to
    ``v_b = -delta/(T_f + T_a/2 + pre_s)`` (exact retarget including the drift
    the ramp causes, ``retarget_parts``); after the post-release hold a
    rest-to-rest quintic returns offset and velocity to zero by the record's
    last knot.  ``--check`` prints b(t_rel), the landing offset and b(t_end).
  * ``--hold-through``: ramp in, hold to the end, throw NOT retargeted.
  * neither flag: ramp in, ramp back out before the next release's
    pre-release hold (forbidden on hardware by the 2026-09-28 carry rule;
    kept as the rejected alternative).

    python tools/probes/late_reaim/bump_replay.py \
        --records temp/probes/late_reaim/records_columns_0.pkl \
        --rows temp/probes/late_reaim/resends_2026-10-16.json \
        --patterns columns --retarget --hold-s 0.05 [--dirs diag|axis] [--check]
"""
import argparse, json, math, os, pickle, sys
import numpy as np
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path[:0] = [ROOT, os.path.join(ROOT, 'ros_ws', 'src', 'jugglebot'), os.path.join(ROOT, 'sim')]
from jugglebot.motion import unified_cycle as uc  # noqa: E402
from jugglebot.motion.trajectory import feasibility as fz  # noqa: E402
from jugglebot.motion.trajectory.cycle_plan import CyclePlan, shift_catch_evidence  # noqa: E402
from jugglebot.motion.skills import bump  # noqa: E402


def _q5(u):
    u = np.clip(u, 0.0, 1.0)
    return (10*u**3 - 15*u**4 + 6*u**5), (30*u**2 - 60*u**3 + 30*u**4)


def bump_profile(t, t_s, t_in_end, t_out_start, t_out_end, delta):
    """(b, b_dot) at times ``t`` (plan clock), per axis of ``delta`` (2,)."""
    t = np.asarray(t, float)
    T1 = t_in_end - t_s
    s1, ds1 = _q5((t - t_s) / T1)
    b = s1[:, None] * delta[None, :]
    bd = (ds1 / T1)[:, None] * delta[None, :]
    if t_out_end is not None:
        T2 = t_out_end - t_out_start
        s2, ds2 = _q5((t - t_out_start) / T2)
        b = b - s2[:, None] * delta[None, :]
        bd = bd - (ds2 / T2)[:, None] * delta[None, :]
    return b, bd


def bumped(plan, b, bd, k0=0, k1=None):
    """The plan with the bump added, sliced to knots ``[k0, k1]`` (the bump
    is zero outside, and those knots already passed the gate, so the gate
    is run on the slice only — validate_cycle's stencils are knot-local)."""
    k1 = plan.n_knots - 1 if k1 is None else k1
    pose = plan.pose.copy(); pv = plan.pose_vel.copy()
    pose[:, 0:2] += b; pv[:, 0:2] += bd
    ck = None
    if plan.contact_knots:
        ck = tuple((max(a, k0) - k0, min(b_, k1) - k0) for a, b_ in plan.contact_knots
                   if b_ >= k0 and a <= k1) or None
    catch_k = plan.catch_k - k0 if k0 <= plan.catch_k <= k1 else -1
    sl = slice(k0, k1 + 1)
    return CyclePlan(pose[sl], pv[sl], plan.hand_rev[sl], plan.hand_vel_rps[sl], plan.dt,
                     catch_k=catch_k, contact_knots=ck,
                     catch_evidence=shift_catch_evidence(
                         plan.catch_evidence, -k0, plan.dt, k1 - k0 + 1))


HOLD_THROUGH = False
RETARGET = False
LEAD_KNOTS = bump.BUMP_LEAD_KNOTS
LAUNCH_BOUND = bump.BUMP_MAX_LAUNCH_LATERAL_MM_S
LEGACY_START = False
CHECK = []  # filled with per-case b(t_rel)/b(t_end) facts when --check
G_MM = 9810.0


def try_bump(rec, t_abs, t_c_abs, delta, limits, geom, lead_s, hold_s):
    """Returns (code, detail).  ``WINDOW`` = the ramp has no room (the
    pre-registered non-terminal decline); otherwise the gate's code."""
    p, m, t0, dt = rec.plan, rec.meta, rec.t0_s, rec.plan.dt
    t_s = t_abs - t0 + lead_s
    t_c = t_c_abs - t0
    t_in_end = t_c - hold_s
    rel_after = [float(r.t_s) for r in m.releases if float(r.t_s) > t_c + 1e-9]
    if RETARGET and rel_after:
        # THE DESIGN: one implementation, the production enforcement point
        # (motion/skills/bump.py::apply_catch_bump, I-CATCH-5).  k_b is knot-
        # aligned there (floor(t/dt) + lead knots), so the ramp may start up to
        # one knot earlier than this probe's old continuous t + lead.
        k_wire = int(math.floor((t_abs - t0) / dt)) + LEAD_KNOTS - bump.BUMP_EXTRA_KNOTS
        res = bump.apply_catch_bump(p, m, k_wire, np.asarray(delta, float), t_c,
                                    limits, geom, pre_hold_s=hold_s,
                                    max_launch_lateral_mm_s=LAUNCH_BOUND,
                                    t_ramp_start_s=(t_abs - t0 + lead_s) if LEGACY_START else None)
        if CHECK and res.accepted and len(CHECK) == 1:
            rm = [r for r in m.releases if float(r.t_s) > t_c + 1e-9][0]
            k_rel = int(round(float(rm.t_s) / dt))
            db = res.plan.pose[:, 0:2] - p.pose[:, 0:2]
            dv = res.plan.pose_vel[:, 0:2] - p.pose_vel[:, 0:2]
            CHECK.append(dict(delta=list(map(float, delta)), b_rel=db[k_rel].tolist(),
                              b_rel_closed=res.release_offset_xy_mm.tolist(),
                              v_rel=dv[k_rel].tolist(),
                              land_off=(res.release_offset_xy_mm
                                        + res.launch_bias_xy_mm_s * float(rm.flight_s)).tolist(),
                              b_end=db[-1].tolist(), v_end=dv[-1].tolist(),
                              hand_identical=bool(np.array_equal(res.plan.hand_rev, p.hand_rev)
                                                  and np.array_equal(res.plan.hand_vel_rps, p.hand_vel_rps)),
                              k_b=res.k_b, flight=float(rm.flight_s), t_c=t_c,
                              t_rel=float(rm.t_s), t_end=float(p.t[-1])))
        if res.accepted:
            return 'OK', res
        if res.code in (bump.BUMP_FREEZE, bump.BUMP_NO_CARRY):
            return 'WINDOW', res
        if res.code == bump.BUMP_LAUNCH:
            return res.code, res
        tv = res.report.peak_leg_t_s if res.report is not None else None
        k0 = max(0, res.k_b - bump.BUMP_GATE_CONTEXT_KNOTS)
        ph = '?' if tv is None else ('in' if tv + k0 * dt <= t_c else ('carry' if tv + k0 * dt <= float(min(rel_after)) + uc.post_release_hold_knots() * dt else 'settle'))
        return res.code + '@' + ph, res
    if HOLD_THROUGH:
        rel_after = []
    if rel_after:
        t_out_end = min(rel_after) - uc.pre_release_hold_knots() * dt
        t_out_start = t_c + hold_s
        if t_out_end - t_out_start < 2 * dt:
            return 'WINDOW', 'no ramp-out room'
    else:
        t_out_end = t_out_start = None
    if t_in_end - t_s < 2 * dt:
        return 'WINDOW', 'ramp-in %.3f s' % (t_in_end - t_s)
    b, bd = bump_profile(p.t, t_s, t_in_end, t_out_start, t_out_end, np.asarray(delta, float))
    k_s = int(math.floor(t_s / dt))
    assert np.allclose(b[:k_s + 1], 0.0) and np.allclose(bd[:k_s + 1], 0.0)
    k_end = int(math.ceil(((t_out_end if t_out_end is not None else p.t[-1])) / dt))
    k0, k1 = max(0, k_s - 4), min(p.n_knots - 1, k_end + 6)
    z = np.zeros_like(b)
    base = fz.validate_cycle(bumped(p, z, z, k0, k1), limits, geom)
    if base.code != 'OK':
        return 'BASE_' + base.code, base
    rep = fz.validate_cycle(bumped(p, b, bd, k0, k1), limits, geom)
    if rep.code != 'OK':
        tv = rep.peak_leg_t_s
        phase = '?' if tv is None else ('in' if (tv + k0 * dt) <= t_c else 'out')
        return rep.code + '@' + phase, rep
    return rep.code, rep


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--records', required=True)
    ap.add_argument('--rows', required=True, help='parsed live re-sends (json)')
    ap.add_argument('--patterns', default='columns,columns_1ball_fed')
    ap.add_argument('--lead-knots', type=int, default=5)
    ap.add_argument('--hold-s', type=float, default=0.05)
    ap.add_argument('--installed-only', action='store_true', help='the re-sends the live raw path DID install (sanity)')
    ap.add_argument('--max-catches', type=int, default=4)
    ap.add_argument('--dirs', default='diag', choices=('diag', 'axis'))
    ap.add_argument('--hold-through', action='store_true',
                    help='never ramp out: the following throw is shifted by delta too')
    ap.add_argument('--retarget', action='store_true', help='hold through release and retarget the carried throw by a lateral launch velocity -delta/flight')
    ap.add_argument('--check', action='store_true', help='print b(t_rel), the landing offset and b(t_end) for the first retarget case')
    ap.add_argument('--no-launch-bound', action='store_true', help='disable the 50 mm/s retarget launch bound (reproduces the pre-decision design row)')
    ap.add_argument('--legacy-start', action='store_true', help='ramp from the pre-production continuous t + lead (reproduces the design row) instead of the knot-aligned k_b')
    a = ap.parse_args(argv)
    global LEAD_KNOTS, LAUNCH_BOUND, LEGACY_START
    LEGACY_START = a.legacy_start
    LEAD_KNOTS = a.lead_knots
    LAUNCH_BOUND = float('inf') if a.no_launch_bound else bump.BUMP_MAX_LAUNCH_LATERAL_MM_S
    global HOLD_THROUGH
    HOLD_THROUGH = a.hold_through
    global RETARGET
    RETARGET = a.retarget
    if a.check:
        CHECK.append(None)
    L = pickle.load(open(a.records, 'rb'))
    rows = [r for r in json.load(open(a.rows)) if r['pat'] in a.patterns.split(',')
            and r['to_land'] and r['code'] not in ('CATCH_AXIS',)
            and (r['code'] == 'INSTALLED') == a.installed_only]
    catches = [x for x in L if x['kind'] == 'CATCH' and x['accepted']][1:-1][:a.max_catches]
    acc = [x for x in L if x['accepted']]
    dt = 0.025; lead = a.lead_knots * dt
    # baseline: every record passes the gate unbumped
    base = 'per-slice'
    tally = {}; worst = []
    dirs = ([(1, 1), (1, -1), (-1, 1), (-1, -1)] if a.dirs == 'diag' else [(1, 0), (-1, 0), (0, 1), (0, -1)])
    # moved_mm is the max COMPONENT: 'diag' (|delta| = mm*sqrt2) is the worst case, 'axis' the best
    bins = {}
    for c in catches:
        t_c = c['event_abs_s']
        for r in rows:
            t_abs = t_c - r['to_land']
            live = [x for x in acc if x['t_now_s'] <= t_abs - 0.05]
            rec = live[-1]['record'] if live else c['record']
            for d in dirs:
                delta = np.array(d, float) * r['mm']
                code, det = try_bump(rec, t_abs, t_c, delta, c['limits'], c['geom'], lead, a.hold_s)
                tally[code] = tally.get(code, 0) + 1
                bk = (r['pat'], round(min(r['to_land'], 0.6) * 20) / 20)
                ok_, n_ = bins.get(bk, (0, 0)); bins[bk] = (ok_ + (code == 'OK'), n_ + 1)
                if code != 'OK' and code != 'WINDOW' and not code.startswith('BASE_'):
                    worst.append((r['to_land'], r['mm'], code))
    n = sum(tally.values())
    print('baseline codes %s; cases %d (rows %d x catches %d x dirs %d); lead %.3f s hold %.3f s'
          % (base, n, len(rows), len(catches), len(dirs), lead, a.hold_s))
    for k, v in sorted(tally.items(), key=lambda kv: -kv[1]):
        print('  %-14s %5d  %5.1f %%' % (k, v, 100.0 * v / n))
    if worst:
        w = np.array([(x[0], x[1]) for x in worst])
        print('  refused: to_land p50 %.3f min %.3f, |delta| p50 %.1f max %.1f'
              % (np.median(w[:, 0]), w[:, 0].min(), np.median(w[:, 1]), w[:, 1].max()))
    if a.check and len(CHECK) > 1:
        print('  CHECK', json.dumps(CHECK[1]))
    for k in sorted(bins):
        print('  %-18s to_land~%.2f  OK %4d / %4d' % (k[0], k[1], bins[k][0], bins[k][1]))
    return tally


if __name__ == '__main__':
    main()
