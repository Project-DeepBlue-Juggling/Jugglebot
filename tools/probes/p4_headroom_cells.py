#!/usr/bin/env python3
"""Oval headroom cells: plan one- and two-ball oval folds through the PRODUCTION
segment path at a chosen planner mode, working point, hand acceleration and
release holds -- and, optionally, model what the pre-release hold does to the
ball's launch velocity under the plant's tracking delay.

Promoted 2026-10-30 from the P3 re-gate's scratch probe (``cells.py`` / ``summ.py``,
``logbook/2026-10-30-two-ball-oval-p3-regate-go-and-dwell-headroom.md``
§ Discussion "The headroom finding") ahead of the P4.0 pre-release hold rung
(map JB-102).

What one cell is
----------------
* The THROW from rest at the oval's release site (``segments.plan_segment(THROW)``,
  launch ``--launch`` s), gated at ``admissible.MARGIN_FRAC`` of the limits.
* The folded CATCH-with-throw window built from that throw's release seed:
  one ball = land at ``T``, release at ``T + dwell`` (the production fold,
  ``admissible_sweep._oval_window``); two balls = land at
  ``tau = (T - dwell) / 2``, release at ``tau + dwell`` (``T = d + 2 tau``; this
  two-ball fold is the PROBE's construction -- no production two-ball compiler
  exists yet).
* A chain check: one ball = ``admissible_sweep._catch_cell_report`` from the
  fold's release; two balls = the same fold again from the fold's release.
* ``ok`` = the planner accepted every piece AND each is inside the margin.
  ``code`` names the first refusal (``INFEASIBLE`` = the QP has no solution;
  ``LIMIT_*`` / ``HAND_LIMIT_ACC`` = ``validate_cycle``; ``MARGIN`` = accepted by
  the planner but over ``MARGIN_FRAC``; ``HOLD_WINDOW`` / ``CATCH_TOO_EARLY`` =
  ``cup_cycle``'s rank guards).  ``bind`` prints each peak against its bound in
  percent (leg v/a/j against the MARGIN bound, hand against the hand limit).

``--release-model`` (accepted fold windows only)
-----------------------------------------------
Reads the fold's own plan (``Segment.plan``, ``release_t_s``,
``takeoff_vel_mm_s``) and reports:

* the planned CUP velocity at the release knot (``unified_cycle.
  cup_velocity_from_platform``) -- it equals the pinned take-off velocity,
  because the release triple is a CUP-space equality: the planner composes the
  platform's motion into the launch, it never assumes the platform is at rest;
* the PLATFORM CARRIER velocity at the release knot (the same map with the hand
  slider rate zeroed: centroid rate + lever x tilt rate) and over the last
  ``4`` knots -- what the pre-release hold drives to ~0;
* the DELAY-MODEL velocity error: the platform tracks its plan with a pure
  delay ``--delay-ms`` (45) then a first-order lag ``--lag-ms`` (30)
  (``unified_cycle.PRE_RELEASE_HOLD_S`` docstring, R4 2026-09-27/28), while the
  ball separates ``--sep-ms`` after the planned release (R4: +10..+40 ms).
  ``dv = carrier(pose_f(t_sep), vel_f(t_sep)) - carrier(plan at t_rel)``
  -- the hand is assumed to deliver its planned stroke rate at separation
  (its own lag is common to every hold and is in the learner's prior);
* the PREDICTED LANDING BIAS: the perturbed launch flown ballistically from the
  release site to the target's z (``g`` from ``ballistics_bc``), dx / dy / |d|
  mm and the apex change, for each separation delay; plus the "uncomposed"
  bound ``carrier(t_rel) x T`` -- the bias a planner that ASSUMED a resting
  platform would have (not this planner; reported as the scale of the carrier).

Probe-only knobs (never the production path)
--------------------------------------------
* ``--hand`` other than the config's hand limit also scales
  ``catch_runway_decel_mps2`` by monkeypatching ``cup_cycle._cfg`` (the
  scratch probe's convention), so the catch runway follows the hand.
* ``--ablate noskid|nocone`` monkeypatches ``cup_cycle.PoseConfig`` to drop the
  I-PLAN-14 touch-down skid rows (``touchdown_skid_mmps=0``) and/or open the
  C-CUP-4 shaper rows (``cone_deg=89.9``, ``floor_g=0``) -- to NAME which
  constraint makes a pose-mode cell QP-infeasible.  The realised-plan checks in
  ``validate_cycle`` still run.

Recipe (repo root, venv, ``OPENBLAS_NUM_THREADS=1``)
---------------------------------------------------
  # the 10-17 "0.30 at apex 1.0 under P4" reproduction, one + two balls, three holds
  python -I tools/probes/p4_headroom_cells.py --balls 1,2 --dwells 0.30 --apexes 1.0 \\
      --point p4 --pre 0.100,0.050,0 --release-model
  # the JB-102 hold ladder at the next Sitting's cell
  python -I tools/probes/p4_headroom_cells.py --balls 1 --dwells 0.45 --apexes 0.95 \\
      --point today --pre 0.100,0.075,0.050,0 --release-model
  # summarise earlier outputs
  python -I tools/probes/p4_headroom_cells.py --summary temp/probes/p4_headroom_cells_*.json

Output: ``temp/probes/p4_headroom_cells_<stamp>.json`` (or ``--out``, which must
resolve under ``temp/probes/``) + one line per cell on stdout.  Deterministic:
the JSON holds no wall-clock field (solve time is stdout only); two runs of the
same arguments on the same tree give identical JSON.  Read-only on the repo.
"""
from __future__ import annotations

import argparse
import dataclasses
import datetime as _dt
import json
import math
import os
import sys
import time

for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_v, '1')

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
for _p in (ROOT, os.path.join(ROOT, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(ROOT, 'tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np                                    # noqa: E402
import admissible_sweep as A                          # noqa: E402
import jugglebot.hardware_config as hw                # noqa: E402
from jugglebot.motion.trajectory import cup_cycle as cc  # noqa: E402

fz, sg, uc, bal, st, sc, ab = A.fz, A.sg, A.uc, A.bal, A.st, A.sc, A.ab
GEOM = A.StewartGeometry()
OUT_DIR = os.path.join(ROOT, 'temp', 'probes')
G_MMS2 = float(-bal.G_VEC_MMS2[2])
HAND_CFG = float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2)

# ---- probe-only monkeypatches (module docstring) ---------------------------
_ORIG_CFG = cc._cfg
_ORIG_POSECFG = cc.PoseConfig
_PATCH = {'runway': None, 'ablate': ()}


def _cfg(cfg, key):
    if key == 'catch_runway_decel_mps2' and _PATCH['runway'] is not None:
        return _PATCH['runway']
    return _ORIG_CFG(cfg, key)


def _posecfg(*a, **kw):
    if 'noskid' in _PATCH['ablate']:
        kw.setdefault('touchdown_skid_mmps', 0.0)
    if 'nocone' in _PATCH['ablate']:
        kw.setdefault('cone_deg', 89.9)
        kw.setdefault('floor_g', 0.0)
    return _ORIG_POSECFG(*a, **kw)


cc._cfg = _cfg
cc.PoseConfig = _posecfg

WORKING_POINTS = {
    # (leg vel mm/s, leg acc mm/s^2, leg jerk mm/s^3, leg acc ceiling)
    'today': (float(hw.JB_TRAJ_LEG_VEL_LIMIT_MMPS), float(hw.JB_TRAJ_LEG_ACC_LIMIT_MMPS2),
              float(hw.JB_TRAJ_LEG_JERK_LIMIT_MMPS3), None),
    'p4': (600.0, 8000.0, 200000.0, 10000.0),     # plan two-ball-oval decisions 52-58
}


def limits_for(point, explicit, hand):
    v, a, j, ceil = WORKING_POINTS[point] if explicit is None else (tuple(explicit) + (None,))[:4]
    # with_sitting_limits clamps to the config CEILINGS (acc 5000 today), so a
    # P4 point is written onto the dataclass afterwards, ceiling included.
    L = A._limits(float(hw.JB_TRAJ_LEG_VEL_LIMIT_MMPS), float(hw.JB_TRAJ_LEG_ACC_LIMIT_MMPS2),
                  float(hw.JB_TRAJ_LEG_JERK_LIMIT_MMPS3), HAND_CFG)
    L = dataclasses.replace(L, leg_vel_mmps=float(v), leg_acc_mmps2=float(a), leg_jerk_mmps3=float(j))
    if ceil is not None:
        L = dataclasses.replace(L, leg_acc_ceiling_mmps2=float(ceil))
    L = dataclasses.replace(L, hand_acc_limit_rps2=float(hand),
                            hand_acc_ceiling_rps2=float(hand))
    _PATCH['runway'] = (float(hand) / cc.HAND_REV_PER_M if float(hand) != HAND_CFG else None)
    return L


def metrics(r, L):
    if r is None:
        return None
    m = dict(code=str(r.code), vel=round(r.peak_leg_vel_mmps), acc=round(r.peak_leg_acc_mmps2),
             jerk=round(r.peak_leg_jerk_mmps3), hand=round(r.peak_hand_acc_rps2),
             seat=None if r.max_cup_seating_deg is None else round(r.max_cup_seating_deg, 1),
             skid=None if r.catch_skid_mmps is None else round(r.catch_skid_mmps),
             close=None if r.catch_closing_ratio is None else round(r.catch_closing_ratio, 2),
             reasons=[str(x)[:160] for x in list(r.reasons)[:2]])
    fr = (('v', r.peak_leg_vel_mmps / (ab.MARGIN_FRAC * L.leg_vel_mmps)),
          ('a', r.peak_leg_acc_mmps2 / (ab.MARGIN_FRAC * L.leg_acc_mmps2)),
          ('j', r.peak_leg_jerk_mmps3 / (ab.MARGIN_FRAC * L.leg_jerk_mmps3)),
          ('h', r.peak_hand_acc_rps2 / L.hand_acc_limit_rps2))
    m['bind'] = ' '.join('%s%+.0f' % (k, 100.0 * (x - 1.0)) for k, x in fr)
    return m


def _refusal(e, L):
    msg = str(e).replace('\n', ' ')
    return str(getattr(e, 'code', type(e).__name__)), metrics(getattr(e, 'report', None), L), msg[:300]


def fold(seed, cs, rs, ts, t_land, v_arr, t_rel, T, L, cfg):
    then = sg.ThrowAfterCatch(t_release_s=t_rel, site_mm=rs.throw_site_mm(),
                              target_mm=ts.throw_site_mm(), flight_s=float(T))
    term = sg.CatchTerminal(landing_mm=cs.catch_site_mm(), landing_vel_mm_s=v_arr, t_land_s=t_land,
                            rest_site_mm=rs.rest_site_mm(), then_throw=then)
    try:
        seg = sg.plan_segment(sg.CATCH, seed, term, cfg, L, GEOM)
    except (uc.CycleInfeasible, cc.CupCycleInfeasible) as e:
        code, m, msg = _refusal(e, L)
        return False, code, m, None, msg
    rep = fz.validate_cycle(seg.plan, L, GEOM)
    ok = A._within_margin(seg.meta.report, L)
    return ok, ('OK' if ok else 'MARGIN'), metrics(rep, L), seg, ''


# ---- the release model -----------------------------------------------------

def _lagged(series, t_grid, t_query, delay_s, lag_s):
    """``series`` (n_fine, d) sampled on ``t_grid``: pure delay then first-order lag."""
    dt = t_grid[1] - t_grid[0]
    out = []
    for tq in t_query:
        s = np.arange(0.0, 6.0 * lag_s + dt, dt) if lag_s > 0 else np.array([0.0])
        w = np.exp(-s / lag_s) if lag_s > 0 else np.array([1.0])
        w = w / w.sum()
        ts_ = np.clip(tq - delay_s - s, t_grid[0], t_grid[-1])
        cols = [np.interp(ts_, t_grid, series[:, c]) for c in range(series.shape[1])]
        out.append(np.array([float(np.dot(w, c)) for c in cols]))
    return out


def _land(p0, v0, z_land):
    """Ballistic flight from p0 with v0 down to z_land (descending root): (xy, t, apex_z)."""
    a, b, c = -0.5 * G_MMS2, float(v0[2]), float(p0[2]) - float(z_land)
    t = (-b - math.sqrt(b * b - 4.0 * a * c)) / (2.0 * a)
    return (np.asarray(p0[:2], float) + np.asarray(v0[:2], float) * t, t,
            float(p0[2]) + b * b / (2.0 * G_MMS2))


def release_model(seg, rs, ts, T, delay_ms, lag_ms, sep_ms):
    plan = seg.plan
    dt = float(plan.dt)
    pose, vel = np.asarray(plan.pose, float), np.asarray(plan.pose_vel, float)
    hand, hvel = np.asarray(plan.hand_rev, float), np.asarray(plan.hand_vel_rps, float)
    t_rel = float(seg.release_t_s)
    k = int(round(t_rel / dt))
    cvp = uc.cup_velocity_from_platform
    v_take = np.asarray(seg.takeoff_vel_mm_s, float).reshape(3)
    cup_v = cvp(pose[k], vel[k], hand[k], hvel[k])
    carrier = lambda kk: cvp(pose[kk], vel[kk], hand[kk], 0.0)  # noqa: E731
    car_k = carrier(k)
    n_pre = int(sg._pre_hold_knots(sg.SegmentConfig()))  # the default hold's knots (4)
    hist = [dict(dt_ms=round((kk - k) * dt * 1e3), carrier=[round(float(x), 1) for x in carrier(kk)],
                 centroid_xy=round(float(np.hypot(*vel[kk][:2])), 1), hand_rps=round(float(hvel[kk]), 2))
            for kk in range(max(0, k - max(n_pre, 4) - 2), min(len(pose), k + 3))]
    # fine grid: linear in pose and pose rate between knots
    tk = np.arange(len(pose)) * dt
    tf = np.arange(0.0, tk[-1] + 1e-9, 0.001)
    P = np.column_stack([np.interp(tf, tk, pose[:, c]) for c in range(6)])
    V = np.column_stack([np.interp(tf, tk, vel[:, c]) for c in range(6)])
    p0 = np.asarray(rs.throw_site_mm(), float)
    z_land = float(np.asarray(ts.throw_site_mm(), float)[2])
    xy_nom, t_nom, apex_nom = _land(p0, v_take, z_land)
    rows = []
    tq = [t_rel + s * 1e-3 for s in sep_ms]
    Pf = _lagged(P, tf, tq, delay_ms * 1e-3, lag_ms * 1e-3)
    Vf = _lagged(V, tf, tq, delay_ms * 1e-3, lag_ms * 1e-3)
    for s, pf, vf in zip(sep_ms, Pf, Vf):
        dv = cvp(pf, vf, hand[k], 0.0) - car_k
        xy, t_f, apex = _land(p0, v_take + dv, z_land)
        d = xy - xy_nom
        rows.append(dict(sep_ms=s, dv=[round(float(x), 1) for x in dv],
                         dx=round(float(d[0]), 1), dy=round(float(d[1]), 1),
                         d=round(float(np.hypot(*d)), 1), dapex_mm=round(apex - apex_nom, 1),
                         dt_flight_ms=round((t_f - t_nom) * 1e3, 1)))
    unc = car_k[:2] * T
    return dict(t_rel=round(t_rel, 3), k_rel=k, dt=dt, takeoff=[round(float(x), 1) for x in v_take],
                cup_v_at_release=[round(float(x), 1) for x in cup_v],
                carrier_at_release=[round(float(x), 1) for x in car_k],
                hand_rps_at_release=round(float(hvel[k]), 2),
                uncomposed_bias_mm=[round(float(unc[0]), 1), round(float(unc[1]), 1)],
                history=hist, delay_ms=delay_ms, lag_ms=lag_ms, bias=rows)


def run(balls, dwell, apex, L, hand, pre, post, sep, mode, launch, args):
    cfg = sg.SegmentConfig(planner_mode=mode, pre_release_hold_s=pre, post_release_hold_s=post)
    st_, sc_ = st.columns_sites(sep)
    T = sc.flight_s(apex)
    out = dict(balls=balls, dwell=dwell, apex=apex, point=args.point_label, hand=hand, pre=pre,
               post=post, mode=mode, ablate=','.join(_PATCH['ablate']) or 'none', T=round(T, 3))
    cs, rs, ts = A._oval_window(st_, sc_, apex, dwell)
    term = sg.ThrowTerminal(site_mm=rs.throw_site_mm(), target_mm=cs.throw_site_mm(), flight_s=T,
                            t_release_s=launch)
    try:
        tseg = sg.plan_segment(sg.THROW, A._rest_state(rs.rest_site_mm()), term, cfg, L, GEOM)
    except (uc.CycleInfeasible, cc.CupCycleInfeasible) as e:
        code, m, msg = _refusal(e, L)
        out.update(ok=False, code='THROW:' + code, win=m, msg=msg)
        return out
    if not A._within_margin(tseg.meta.report, L):
        out.update(ok=False, code='THROW:MARGIN', win=metrics(tseg.meta.report, L))
        return out
    seed, tk = tseg.release_state, tseg.takeoff_vel_mm_s
    out['throw_hand'] = round(tseg.meta.report.peak_hand_acc_rps2)
    v_arr = bal.arrival_velocity(tk, T)
    if balls == 1:
        t_land, t_rel = T, T + dwell
    else:
        tau = (T - dwell) / 2.0
        out['tau'] = round(tau, 3)
        t_land, t_rel = tau, tau + dwell
    ok, code, m, seg, msg = fold(seed, cs, rs, ts, t_land, v_arr, t_rel, T, L, cfg)
    out.update(ok=ok, code=code, win=m, msg=msg)
    if seg is not None and args.release_model:
        out['release'] = release_model(seg, rs, ts, T, args.delay_ms, args.lag_ms, args.sep_ms)
    if ok:
        if balls == 1:
            ok2, code2, r2 = A._catch_cell_report(ts, seg.release_state, seg.takeoff_vel_mm_s, T, T,
                                                  (0., 0.), L, GEOM, cfg)
            out['chain'] = dict(ok=bool(ok2), code=str(code2), m=metrics(r2, L))
        else:
            ok2, code2, m2, _s, msg2 = fold(seg.release_state, cs, rs, ts, t_land,
                                            bal.arrival_velocity(seg.takeoff_vel_mm_s, T), t_rel, T,
                                            L, cfg)
            out['chain'] = dict(ok=bool(ok2), code=code2, m=m2, msg=msg2)
        out['ok'] = bool(ok and out['chain']['ok'])
        if not out['chain']['ok']:
            out['code'] = 'CHAIN:' + out['chain']['code']
    return out


def _line(r):
    w = r.get('win') or {}
    c = (r.get('chain') or {}).get('m') or {}
    s = ('%db %-5s %-4s h%-5g pre%.3f post%.3f apex%.2f d%.2f tau%-5s | %-5s %-22s | win v%s a%s j%s '
         'hand%s seat%s skid%s | bind %s | chain j%s bind %s | thr_hand %s' % (
             r['balls'], r['point'], r['mode'], r['hand'], r['pre'], r['post'], r['apex'], r['dwell'],
             r.get('tau', ''), r.get('ok'), str(r.get('code'))[:22], w.get('vel'), w.get('acc'),
             w.get('jerk'), w.get('hand'), w.get('seat'), w.get('skid'), w.get('bind'),
             c.get('jerk'), c.get('bind'), r.get('throw_hand')))
    if r.get('ablate', 'none') != 'none':
        s += ' | ablate ' + r['ablate']
    rel = r.get('release')
    if rel:
        s += '\n    release t=%.3f carrier@rel %s  cup_v %s  takeoff %s  uncomposed %s mm' % (
            rel['t_rel'], rel['carrier_at_release'], rel['cup_v_at_release'], rel['takeoff'],
            rel['uncomposed_bias_mm'])
        for b in rel['bias']:
            s += '\n    sep +%-3g ms dv %-24s -> landing dx %+6.1f dy %+6.1f |d| %5.1f mm, apex %+5.1f mm' % (
                b['sep_ms'], b['dv'], b['dx'], b['dy'], b['d'], b['dapex_mm'])
    return s


def summary(paths):
    for p in paths:
        print('== %s' % p)
        for r in json.load(open(p)):
            print(_line(r))


def _floats(s):
    return [float(x) for x in s.split(',') if x.strip()]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog='See the module docstring for the cell definition and recipes.')
    ap.add_argument('--summary', nargs='+', metavar='JSON', help='print earlier outputs and exit')
    ap.add_argument('--balls', default='1', help='comma list of 1|2 (default 1)')
    ap.add_argument('--dwells', default='0.50,0.45,0.40,0.35,0.30')
    ap.add_argument('--apexes', default='0.95,1.00')
    ap.add_argument('--point', default='today', choices=sorted(WORKING_POINTS),
                    help='today = hardware_config limits; p4 = 600/8000/200k, acc ceiling 10000')
    ap.add_argument('--limits', default=None, metavar='V,A,J[,ACC_CEIL]',
                    help='explicit leg vel/acc/jerk (mm/s, mm/s^2, mm/s^3); overrides --point')
    ap.add_argument('--hand', default=str(HAND_CFG), help='comma list of hand acc limits (rev/s^2)')
    ap.add_argument('--pre', default=str(uc.PRE_RELEASE_HOLD_S), help='comma list, pre-release hold (s)')
    ap.add_argument('--post', default=str(uc.POST_RELEASE_HOLD_S), help='comma list, post-release hold (s)')
    ap.add_argument('--mode', default='pose', choices=list(uc.PLANNER_MODES))
    ap.add_argument('--sep', type=float, default=150.0, help='oval site separation (mm)')
    ap.add_argument('--launch', type=float, default=0.4, help='first THROW lead from rest (s)')
    ap.add_argument('--ablate', default='none', choices=['none', 'noskid', 'nocone', 'noskid,nocone'])
    ap.add_argument('--release-model', action='store_true', help='add the release-state bias model')
    ap.add_argument('--delay-ms', type=float, default=45.0)
    ap.add_argument('--lag-ms', type=float, default=30.0)
    ap.add_argument('--sep-ms', default='0,10,20,40', help='separation delays after the planned release')
    ap.add_argument('--out', default=None, help='JSON path under temp/probes/ (default timestamped)')
    a = ap.parse_args(argv)
    if a.summary:
        summary(a.summary)
        return 0
    a.sep_ms = _floats(a.sep_ms)
    _PATCH['ablate'] = () if a.ablate == 'none' else tuple(a.ablate.split(','))
    explicit = _floats(a.limits) if a.limits else None
    a.point_label = 'expl' if explicit else a.point
    os.makedirs(OUT_DIR, exist_ok=True)
    out = a.out or os.path.join(OUT_DIR, 'p4_headroom_cells_%s.json'
                                % _dt.datetime.now().strftime('%Y%m%d_%H%M%S'))
    out = os.path.abspath(out)
    if not out.startswith(OUT_DIR + os.sep):
        ap.error('--out must resolve under %s' % OUT_DIR)
    rows = []
    for b in [int(x) for x in a.balls.split(',')]:
        for h in _floats(a.hand):
            L = limits_for(a.point, explicit, h)
            for pr in _floats(a.pre):
                for po in _floats(a.post):
                    for ax in _floats(a.apexes):
                        for d in _floats(a.dwells):
                            t0 = time.time()
                            try:
                                r = run(b, d, ax, L, h, pr, po, a.sep, a.mode, a.launch, a)
                            except Exception as e:  # a probe reports, it does not stop the grid
                                r = dict(balls=b, dwell=d, apex=ax, point=a.point_label, hand=h, pre=pr,
                                         post=po, mode=a.mode, ok=False, code='EXC ' + repr(e)[:200])
                            print(_line(r) + '   [%.1f s]' % (time.time() - t0), flush=True)
                            rows.append(r)
    with open(out, 'w') as fh:
        json.dump(rows, fh, indent=1, default=str, sort_keys=True)
    print('wrote %s' % out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
