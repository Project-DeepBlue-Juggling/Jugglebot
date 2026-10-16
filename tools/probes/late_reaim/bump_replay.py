"""Offline prototype of the late catch re-aim BUMP (plans/active/late-catch-reaim-bump.md).

For every (delta, time-to-landing) a live re-send carried on 2026-10-16
(``parse_resends.py`` output), apply a platform-xy offset to the plan record a
sim columns run had installed at that instant (``capture_records.py``) and run
``feasibility.validate_cycle`` on the bumped span ``[k_b-4, end]`` at the live
session limits.  Prints to stdout only.  The hand arrays are never touched.

Shapes (all zero, with zero velocity/acceleration, through ``k_b`` = re-send
+ ``--lead-knots``):

  * ``--retarget`` (THE DESIGN, ``retarget_profile``): quintic ramp to delta
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
from jugglebot.motion.trajectory.cycle_plan import CyclePlan  # noqa: E402


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
                     catch_k=catch_k, contact_knots=ck)


HOLD_THROUGH = False
RETARGET = False
CHECK = []  # filled with per-case b(t_rel)/b(t_end) facts when --check
G_MM = 9810.0


def _vel_ramp(t, t_a, T, v):
    """Position/velocity of a quintic VELOCITY ramp 0 -> v over [t_a, t_a+T]
    (C2 in velocity, so C3 in position), continued at constant v after it."""
    u = np.clip((np.asarray(t, float) - t_a) / T, 0.0, 1.0)
    pos = T * (2.5 * u**4 - 3 * u**5 + u**6) + np.maximum(np.asarray(t, float) - (t_a + T), 0.0)
    vel = 10 * u**3 - 15 * u**4 + 6 * u**5
    return pos[:, None] * v[None, :], vel[:, None] * v[None, :]


def _rest_quintic(t, t_d, t_e, p0, v0):
    """Per-axis quintic from (p0, v0, a=0) at ``t_d`` to (0, 0, 0) at ``t_e``,
    zero after ``t_e``: the post-release re-centre (the ball has left)."""
    T = t_e - t_d
    A = np.array([[T**3, T**4, T**5], [3*T**2, 4*T**3, 5*T**4], [6*T, 12*T**2, 20*T**3]])
    tau = np.clip(np.asarray(t, float) - t_d, 0.0, T)
    pos = np.zeros((len(tau), len(p0))); vel = np.zeros_like(pos)
    for ax in range(len(p0)):
        c3, c4, c5 = np.linalg.solve(A, [-p0[ax] - v0[ax]*T, -v0[ax], 0.0])
        pos[:, ax] = p0[ax] + v0[ax]*tau + c3*tau**3 + c4*tau**4 + c5*tau**5
        vel[:, ax] = v0[ax] + 3*c3*tau**2 + 4*c4*tau**3 + 5*c5*tau**4
    return pos, vel


def retarget_parts(delta, t_c, t_rel, pre_s, flight_s):
    """(v_b, b_rel): the launch-velocity bias and the realised offset at release.

    The velocity ramp 0 -> v_b runs over the carry ``[t_c, t_rel - pre_s]``
    (T_a) and v_b is held through the pre-release hold, so the cup drifts
    ``v_b*(T_a/2 + pre_s)`` before release.  Exact retarget (landing offset 0):
    ``b_rel + v_b*T_f = 0`` with ``b_rel = delta + v_b*(T_a/2 + pre_s)``, so
    ``v_b = -delta / (T_f + T_a/2 + pre_s)``."""
    T_a = (t_rel - pre_s) - t_c
    v_b = -np.asarray(delta, float) / (flight_s + T_a / 2.0 + pre_s)
    return v_b, np.asarray(delta, float) + v_b * (T_a / 2.0 + pre_s)


def retarget_profile(t, t_s, t_in_end, t_c, t_rel, pre_s, post_s, t_end, flight_s, delta):
    """Held-through-release bump that RETARGETS the carried throw (W1-corrected
    2026-10-16): ramp in to delta by ``t_in_end``; over the carry a quintic
    lateral VELOCITY ramp to ``v_b`` (:func:`retarget_parts`, which accounts
    for the drift it causes), constant through the pre/post-release hold (no
    added acceleration across the detach cone); after the post-release hold a
    rest-to-rest quintic brings the offset AND its velocity back to zero by the
    record's last knot, so the rest-terminal is the plan's own rest site."""
    delta = np.asarray(delta, float)
    b, bd = bump_profile(t, t_s, t_in_end, None, None, delta)
    v_b, _ = retarget_parts(delta, t_c, t_rel, pre_s, flight_s)
    T_a = (t_rel - pre_s) - t_c
    p1, v1 = _vel_ramp(t, t_c, T_a, v_b)
    b, bd = b + p1, bd + v1
    t_d = t_rel + post_s
    tt = np.asarray(t, float)
    after = tt >= t_d - 1e-12
    p_d = delta + v_b * (T_a / 2.0 + pre_s + post_s)
    q, qd = _rest_quintic(tt, t_d, t_end, p_d, v_b)
    b[after] = q[after]; bd[after] = qd[after]
    return b, bd


def try_bump(rec, t_abs, t_c_abs, delta, limits, geom, lead_s, hold_s):
    """Returns (code, detail).  ``WINDOW`` = the ramp has no room (the
    pre-registered non-terminal decline); otherwise the gate's code."""
    p, m, t0, dt = rec.plan, rec.meta, rec.t0_s, rec.plan.dt
    t_s = t_abs - t0 + lead_s
    t_c = t_c_abs - t0
    t_in_end = t_c - hold_s
    rel_after = [float(r.t_s) for r in m.releases if float(r.t_s) > t_c + 1e-9]
    if RETARGET and rel_after:
        t_rel = min(rel_after)
        rm = [r for r in m.releases if abs(float(r.t_s) - t_rel) < 1e-9][0]
        flight = float(rm.flight_s)
        pre_s = uc.pre_release_hold_knots() * dt; post_s = uc.post_release_hold_knots() * dt
        t_end = float(p.t[-1])
        if (t_rel - pre_s - t_c < 2 * dt or t_c - hold_s - t_s < 2 * dt
                or t_end - (t_rel + post_s) < 2 * dt):
            return 'WINDOW', 'retarget'
        b, bd = retarget_profile(p.t, t_s, t_c - hold_s, t_c, t_rel, pre_s, post_s,
                                 t_end, flight, np.asarray(delta, float))
        if CHECK:
            v_b, b_rel_cf = retarget_parts(delta, t_c, t_rel, pre_s, flight)
            k_rel = int(round(t_rel / dt))
            CHECK.append(dict(delta=list(map(float, delta)), b_rel=b[k_rel].tolist(),
                              b_rel_closed=b_rel_cf.tolist(), v_rel=bd[k_rel].tolist(),
                              land_off=(b[k_rel] + bd[k_rel] * flight).tolist(),
                              b_end=b[-1].tolist(), v_end=bd[-1].tolist(),
                              flight=flight, t_c=t_c, t_rel=t_rel, t_end=t_end))
        k_s = int(math.floor(t_s / dt))
        k0, k1 = max(0, k_s - 4), p.n_knots - 1
        z = np.zeros_like(b)
        base = fz.validate_cycle(bumped(p, z, z, k0, k1), limits, geom)
        if base.code != 'OK':
            return 'BASE_' + base.code, base
        rep = fz.validate_cycle(bumped(p, b, bd, k0, k1), limits, geom)
        if rep.code != 'OK':
            tv = rep.peak_leg_t_s
            ph = '?' if tv is None else ('in' if tv + k0 * dt <= t_c else ('carry' if tv + k0 * dt <= t_rel + post_s else 'settle'))
            return rep.code + '@' + ph, rep
        return 'OK', rep
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
    a = ap.parse_args(argv)
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
