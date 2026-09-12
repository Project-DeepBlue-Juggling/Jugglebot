#!/usr/bin/env python3
"""Skill-stack R2 sizing sweep — which columns schedules the REAL QP + gate accept.

Plan ``plans/active/two-ball-skill-stack.md`` § 4 R2 "Sizing probe": before any
session limit or apex/separation threshold is written down, sweep

    apex ∈ {1.0, 1.3, 1.5} m × separation ∈ {80, 100} mm × leg jerk ∈ {30k, 60k, 100k}

(plus a leg-VELOCITY axis, because the first run of this probe showed a 100 mm
transit in 0.27 s reads 299 mm/s against the sittings' 250 mm/s session limit —
jerk is not the only binding bound) and record, per cell, the DWELL band ``d``
for which a self-consistent columns chain plans and passes ``validate_cycle``.

**What a "columns schedule" is here, in the four existing window kinds.**  Two
balls, two sites P1 = (0, 0) and P2 = (sep, 0); every throw is VERTICAL (target =
its own site) so the balls stay out of each other's arc; t_f = 2·√(2h/g);
β = (t_f + d)/2; the empty-hand transit between sites is τ = (t_f − d)/2.  The
steady state is a chain of STEADY windows of period β — release at P_a, catch at
P_b at t = τ with the arrival velocity of a vertical throw at v = g·t_f/2, release
at P_b at t = β — alternating sites.  The chain is built exactly the way
``sim/unified_gate.plan_ring`` builds a ring (``release_state_from_meta`` →
``plan_steady`` → ``extend``), so every seam is exact and every window is gated by
the canonical gate.  The dwell ``d`` is the OUTPUT: § 1.2's stroke-coefficient
model (0.998/v + 0.614/v) is the legacy hand's number and the streamed hand's is
what this measures.

Determinism: planning is bit-for-bit deterministic (``tests/motion/
test_unified_cycle.py::test_planning_is_deterministic``); run this twice and
``diff`` the CSVs before quoting a row.

Usage (venv)::

    python tools/probes/skills_sizing_sweep.py                 # the plan's grid
    python tools/probes/skills_sizing_sweep.py --vel 250 400   # add a velocity axis
    python tools/probes/skills_sizing_sweep.py --out temp/probes/skills_sizing_run2.csv

Outputs ``temp/probes/skills_sizing_<stamp>.csv`` and a ``.md`` table beside it.
"""
from __future__ import annotations

import argparse
import csv
import datetime as _dt
import itertools
import os
import sys
import time

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
for _p in (_ROOT, os.path.join(_ROOT, 'ros_ws', 'src', 'jugglebot')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np                                                    # noqa: E402

import jugglebot.hardware_config as hw                                # noqa: E402
from jugglebot.motion.geometry import StewartGeometry                 # noqa: E402
from jugglebot.motion import unified_cycle as uc                      # noqa: E402
from jugglebot.motion.trajectory import TrajectoryLimits              # noqa: E402
from jugglebot.motion.trajectory import ballistics_bc as bal          # noqa: E402

G_M_S2 = 9.806
#: The sittings' cup geometry (``reload_coordinator_node._UNIFIED_*``).
THROW_CUP_Z_MM = 860.0
CATCH_CUP_Z_MM = 830.0
REST_CUP_Z_MM = 750.0
#: STEADY windows chained after the launch: two full alternations plus one, so
#: both parities (P1→P2 and P2→P1) are gated at least once with a gated
#: predecessor on each side.
N_STEADY = 3
#: The launch period is searched separately (it is not beat-bound).
LAUNCH_PERIODS_S = (0.40, 0.50, 0.60, 0.80)

APEXES_M = (1.0, 1.3, 1.5)
SEPARATIONS_MM = (80.0, 100.0)
JERKS = (30_000.0, 60_000.0, 100_000.0)
DWELLS_S = tuple(round(0.20 + 0.05 * i, 2) for i in range(11))     # 0.20 … 0.70


def flight_s(apex_m: float) -> float:
    return 2.0 * np.sqrt(2.0 * float(apex_m) / G_M_S2)


def _rest_state(xy, cup_z_mm, cfg):
    slider_mm = float(cup_z_mm) - float(cfg.cup_z_base_mm)
    rev = ((slider_mm - float(cfg.slider_rev_zero_mm)) / 1000.0 * hw.HAND_REV_PER_M)
    pose = np.array([xy[0], xy[1], float(cfg.active_z_mm), 0.0, 0.0, 0.0])
    return uc.CycleState.at_rest(pose, rev, cfg)


def _limits(vel, acc, jerk, *, unclamped=False, hand_acc=None):
    """Session limits. ``unclamped`` is a PROBE-ONLY bypass of the YAML hard
    ceilings (``with_session_limits`` clamps to them, as the live ``set_limits``
    does): the frontier question "what would this pattern NEED" has to be able to
    quote a number above today's administrative ceiling, or the table cannot say
    whether a ramp decision (plan § 1.3 decision 4) would help at all."""
    base = TrajectoryLimits.from_config(hw)
    if hand_acc is not None:
        # The hand pair rides the same clamp-to-ceiling machinery (limits.py); the
        # C-HAND-2 ceiling (3900) is physical and is NOT bypassed even unclamped.
        base = base.with_session_limits(hand_acc_rps2=float(hand_acc))
    if not unclamped:
        return base.with_session_limits(
            leg_vel_mmps=float(vel), leg_acc_mmps2=float(acc), leg_jerk_mmps3=float(jerk))
    import dataclasses as _dc
    return _dc.replace(base, leg_vel_mmps=float(vel), leg_acc_mmps2=float(acc),
                       leg_jerk_mmps3=float(jerk))


def columns_chain(apex_m, sep_mm, dwell_s, limits, geom, *, launch_s=0.6,
                  n_steady=N_STEADY, throw_z_mm=THROW_CUP_Z_MM,
                  catch_z_mm=CATCH_CUP_Z_MM):
    """Plan LAUNCH + STEADY×n + LANDING for one columns cell. Returns
    ``(ok, code, detail, peaks)``; ``peaks`` is the joined report's peak set."""
    T = flight_s(apex_m)
    beta = 0.5 * (T + dwell_s)
    tau = 0.5 * (T - dwell_s)
    if tau <= 2 * float(hw.JB_TRAJ_KNOT_DT_S):
        return False, 'NO_TRANSIT', 'transit %.3f s <= 2 knots' % tau, None
    cfg = uc.build_realize_config(limits)
    sites = [np.array([0.0, 0.0]), np.array([float(sep_mm), 0.0])]
    thr = [np.array([*s, float(throw_z_mm)]) for s in sites]
    cat = [np.array([*s, float(catch_z_mm)]) for s in sites]
    settle = [np.array([*s, uc.SETTLE_CUP_Z_MM]) for s in sites]
    try:
        state = _rest_state(sites[0], REST_CUP_Z_MM, cfg)
        plan, meta = uc.plan_launch(
            uc.CycleGoals(period_s=launch_s, throw_site_mm=thr[0],
                          throw_target_mm=thr[0], flight_s=T,
                          settle_site_mm=settle[0]), state, limits, geom)
        for i in range(n_steady):
            here = (i + 1) % 2
            cv = bal.arrival_velocity(meta.release_vel_mm_s, T)
            p2, m2 = uc.plan_steady(
                uc.CycleGoals(period_s=beta, throw_site_mm=thr[here],
                              throw_target_mm=thr[here], flight_s=T,
                              catch_site_mm=cat[here], catch_vel_mm_s=cv,
                              catch_t_s=tau),
                uc.release_state_from_meta(meta, plan), limits, geom)
            plan, meta = uc.extend(plan, meta, p2, m2, limits, geom)
        last = (n_steady + 1) % 2
        cv = bal.arrival_velocity(meta.release_vel_mm_s, T)
        p2, m2 = uc.plan_landing(
            uc.CycleGoals(period_s=beta, catch_site_mm=cat[last],
                          catch_vel_mm_s=cv, catch_t_s=tau,
                          settle_site_mm=settle[last]),
            uc.release_state_from_meta(meta, plan), limits, geom)
        plan, meta = uc.extend(plan, meta, p2, m2, limits, geom)
    except uc.CycleInfeasible as exc:
        return False, exc.code, exc.outcome(), None
    except ValueError as exc:
        return False, 'VALUE_ERROR', str(exc), None
    r = meta.report
    return True, 'OK', 'beta %.3f transit %.3f' % (beta, tau), dict(
        peak_vel=r.peak_leg_vel_mmps, peak_acc=r.peak_leg_acc_mmps2,
        peak_jerk=r.peak_leg_jerk_mmps3, hand_vel=r.peak_hand_vel_rps,
        hand_acc=r.peak_hand_acc_rps2, knots=int(plan.n_knots))


def launch_min_period(apex_m, limits, geom, throw_z_mm=THROW_CUP_Z_MM):
    """Smallest period in :data:`LAUNCH_PERIODS_S` for which a rest→release LAUNCH
    at this apex plans and gates; ``(period, code_at_smallest)``."""
    T = flight_s(apex_m)
    cfg = uc.build_realize_config(limits)
    thr = np.array([0.0, 0.0, float(throw_z_mm)])
    first_code = ''
    for L in LAUNCH_PERIODS_S:
        try:
            uc.plan_launch(uc.CycleGoals(period_s=L, throw_site_mm=thr,
                                         throw_target_mm=thr, flight_s=T,
                                         settle_site_mm=np.array([0.0, 0.0, uc.SETTLE_CUP_Z_MM])),
                           _rest_state((0.0, 0.0), REST_CUP_Z_MM, cfg), limits, geom)
            return L, first_code
        except uc.CycleInfeasible as exc:
            first_code = first_code or exc.code
    return float('nan'), first_code


def sweep(apexes=APEXES_M, seps=SEPARATIONS_MM, jerks=JERKS, vels=(250.0,),
          accs=(3000.0,), dwells=DWELLS_S, log=print, *, throw_z_by_apex=None,
          unclamped=False, hand_accs=(None,)):
    """``throw_z_by_apex`` maps apex → release cup z (mm); the catch z keeps the
    sittings' 30 mm drop below it. Default: 860 mm at every apex."""
    geom = StewartGeometry()
    rows = []
    for apex, sep, jerk, vel, acc, hacc in itertools.product(apexes, seps, jerks, vels, accs, hand_accs):
        limits = _limits(vel, acc, jerk, unclamped=unclamped, hand_acc=hacc)
        thr_z = float((throw_z_by_apex or {}).get(apex, THROW_CUP_Z_MM))
        cat_z = thr_z - (THROW_CUP_Z_MM - CATCH_CUP_Z_MM)
        L, lcode = launch_min_period(apex, limits, geom, thr_z)
        for d in dwells:
            if d >= flight_s(apex):
                continue
            t0 = time.perf_counter()
            ok, code, detail, peaks = columns_chain(
                apex, sep, d, limits, geom, launch_s=(L if L == L else 0.6),
                throw_z_mm=thr_z, catch_z_mm=cat_z)
            row = dict(apex_m=apex, sep_mm=sep, jerk=jerk, vel=vel, acc=acc,
                       hand_acc_limit=float(limits.hand_acc_limit_rps2), throw_z_mm=thr_z,
                       dwell_s=d, flight_s=round(flight_s(apex), 4),
                       beta_s=round(0.5 * (flight_s(apex) + d), 4),
                       transit_s=round(0.5 * (flight_s(apex) - d), 4),
                       ok=ok, code=code, launch_min_s=L, launch_code=lcode,
                       wall_s=round(time.perf_counter() - t0, 2),
                       **{k: (round(v, 3) if isinstance(v, float) else v)
                          for k, v in (peaks or {}).items()})
            row['detail'] = detail[:160]
            rows.append(row)
            log('%s' % ({k: row[k] for k in ('apex_m', 'sep_mm', 'jerk', 'vel', 'hand_acc_limit', 'dwell_s', 'ok', 'code', 'wall_s')},))
    return rows


def _band(rows_cell):
    oks = [r['dwell_s'] for r in rows_cell if r['ok']]
    if not oks:
        codes = sorted({r['code'] for r in rows_cell})
        return '— (%s)' % ', '.join(codes)
    return '%.2f–%.2f' % (min(oks), max(oks))


def to_markdown(rows):
    cells = {}
    for r in rows:
        cells.setdefault((r['apex_m'], r['sep_mm'], r['jerk'], r['vel'], r['acc'], r['hand_acc_limit']), []).append(r)
    out = ['| apex m | rel z | sep mm | jerk | vel | acc | hand acc lim | t_f s | feasible dwell d (s) | β at min d | transit at min d | peak vel/acc/jerk on the feasible rows | hand acc | launch min s | binding when refused |',
           '|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|']
    for key in sorted(cells):
        rc = cells[key]
        oks = [r for r in rc if r['ok']]
        band = _band(rc)
        b = ('%.3f' % min(r['beta_s'] for r in oks)) if oks else '—'
        tr = ('%.3f' % max(r['transit_s'] for r in oks)) if oks else '—'
        pk = ('%.0f / %.0f / %.0fk' % (max(r['peak_vel'] for r in oks), max(r['peak_acc'] for r in oks),
                                       max(r['peak_jerk'] for r in oks) / 1000.0)) if oks else '—'
        ha = ('%.0f' % max(r['hand_acc'] for r in oks)) if oks else '—'
        refused = sorted({r['code'] for r in rc if not r['ok']})
        out.append('| %.1f | %.0f | %.0f | %.0fk | %.0f | %.0f | %.0f | %.3f | %s | %s | %s | %s | %s | %s | %s |' % (
            key[0], rc[0]['throw_z_mm'], key[1], key[2] / 1000.0, key[3], key[4], key[5], rc[0]['flight_s'],
            band, b, tr, pk, ha, rc[0]['launch_min_s'], ', '.join(refused) or '—'))
    return '\n'.join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--vel', type=float, nargs='+', default=[250.0])
    ap.add_argument('--acc', type=float, nargs='+', default=[3000.0])
    ap.add_argument('--throw-z', type=float, nargs='*', default=None,
                    help='release cup z per --apex entry (mm); default 860 everywhere')
    ap.add_argument('--hand-acc', type=float, nargs='+', default=[None],
                    help='session hand acceleration limit(s) rev/s^2 (clamped to the C-HAND-2 ceiling)')
    ap.add_argument('--unclamped', action='store_true',
                    help='PROBE ONLY: bypass the YAML hard ceilings on the leg limits')
    ap.add_argument('--apex', type=float, nargs='+', default=list(APEXES_M))
    ap.add_argument('--sep', type=float, nargs='+', default=list(SEPARATIONS_MM))
    ap.add_argument('--jerk', type=float, nargs='+', default=list(JERKS))
    ap.add_argument('--out', default=None)
    args = ap.parse_args(argv)
    tz = (dict(zip(args.apex, args.throw_z)) if args.throw_z else None)
    rows = sweep(args.apex, args.sep, args.jerk, args.vel, args.acc,
                 throw_z_by_apex=tz, unclamped=args.unclamped,
                 hand_accs=tuple(args.hand_acc))
    stamp = _dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    out = args.out or os.path.join(_ROOT, 'temp', 'probes', 'skills_sizing_%s.csv' % stamp)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    keys = sorted({k for r in rows for k in r})
    with open(out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    md = to_markdown(rows)
    with open(os.path.splitext(out)[0] + '.md', 'w') as f:
        f.write(md + '\n')
    print(md)
    print('wrote', out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
