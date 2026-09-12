#!/usr/bin/env python3
"""Skill-stack R2 sizing, SEGMENT form — the schedule the machine will actually run.

`skills_sizing_sweep.py` sizes columns as one continuous STEADY ring.  The skill
stack runs SEGMENTS: a rest-terminal window per skill, spliced onto the live
plan.  The first pinned form — a LANDING-only CATCH with the next THROW spliced
into its tail as a LAUNCH of `dwell − lead` from a still-decelerating seed — was
infeasible at every cell of a 480-cell grid (260k mm/s³ of leg jerk at the
adopted point against 200k; run 1, 2026-09-12), so a catch now CARRIES the next
same-site throw as ONE STEADY window plus a SETTLE tail (`schedule.ThenThrow`),
and its splice lands on the previous release knot and is seeded post-release.
That is still not the continuous ring: each segment is rest-terminal and is
re-gated at its seam, and the limits also reshape the banking schedule
(`build_realize_config(limits)`), so a table produced by the ring form at one
set of limits says nothing exact about the segment form at another.  This
harness drives the REAL chain — `schedule.compile_columns` → `SkillExecutor`
→ `executor.install_segment` (splice, guards, gate) — with a perfect analytic
tracker and no plant, and records, per cell, whether the whole schedule
installs and where it refuses.  It is what `sim/skills_gate.py` runs minus the
MuJoCo plant; the plant adds the noise model and the capture physics, not the
planning.

Usage (venv)::

    python tools/probes/skills_segment_sweep.py --apex 0.9 1.0 --sep 40 60 80 100 \
        --dwell 0.30 0.35 0.40 0.45 0.50 --jerk 200000 300000 --vel-acc 300/5000 500/5000 800/8000 \
        --hand-acc 3500 3900 --out temp/probes/skills_segment_run1.csv

Deterministic (the planner is); run twice and diff before quoting a row.
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
from jugglebot.motion.trajectory import cup_realize as cr             # noqa: E402
from jugglebot.motion.skills import executor as ex                    # noqa: E402
from jugglebot.motion.skills import schedule as sc                    # noqa: E402
from jugglebot.motion.skills import segments as sg                    # noqa: E402
from jugglebot.motion.skills import sites as si                       # noqa: E402

DT = float(hw.JB_TRAJ_KNOT_DT_S)


def _limits(vel, acc, jerk, hand_acc, *, unclamped=False):
    base = TrajectoryLimits.from_config(hw).with_session_limits(hand_acc_rps2=float(hand_acc))
    if not unclamped:
        return base.with_session_limits(leg_vel_mmps=float(vel), leg_acc_mmps2=float(acc),
                                        leg_jerk_mmps3=float(jerk))
    import dataclasses as _dc
    return _dc.replace(base, leg_vel_mmps=float(vel), leg_acc_mmps2=float(acc),
                       leg_jerk_mmps3=float(jerk))


def _rest_state(cup_mm):
    cfg = cr.RealizeConfig()
    slider_mm = float(cup_mm[2]) - cfg.cup_z_base_mm
    rev = (slider_mm - cfg.slider_rev_zero_mm) / 1000.0 * cr.HAND_REV_PER_M
    pose = np.array([cup_mm[0], cup_mm[1], cfg.active_z_mm, 0.0, 0.0, 0.0])
    return uc.CycleState.at_rest(pose, rev, cfg)


def run_schedule(apex_m, sep_mm, dwell_s, limits, geom, *, n_throws=4, launch_s=0.4,
                 t0_abs=10.0, solve_ms=0.0):
    """Drive one columns schedule through the executor. Returns a row dict."""
    sites = si.columns_sites(float(sep_mm))
    pattern = sc.Pattern(sites=sites, apex_m=float(apex_m), dwell_s=float(dwell_s),
                         n_throws=int(n_throws), launch_s=float(launch_s))
    try:
        schedule = sc.compile_columns(pattern, t0_abs)
    except ValueError as exc:
        return dict(ok=False, code='SCHEDULE', detail=str(exc)[:160], installs=0)
    T = schedule.flight_s
    v_mm_s = 0.5 * bal.GRAVITY_MMS2 * T          # vertical take-off speed
    arrival = np.array([0.0, 0.0, -v_mm_s])
    # Perfect tracker: every ball lands at its site's catch point, vertically.
    landings = {}
    for sk in schedule.skills:
        if sk.kind == sg.CATCH:
            landings.setdefault(sk.ball_id, []).append(
                ex.Landing(pos_mm=sk.site.catch_site_mm(), vel_mm_s=arrival.copy(),
                           t_land_abs_s=float(sk.t_abs_s)))
    clock = {'t': t0_abs - 1.0}

    def tracker(ball_id):
        for l in landings.get(ball_id, ()):
            if l.t_land_abs_s > clock['t'] - 0.05:
                return l
        return None

    state = {'record': None}
    p1 = sites[0]
    cfg = sg.SegmentConfig()
    wall = []
    peaks = []

    def installer(kind, terminal, t_now_s, ball_id=0):
        rec = state['record']
        seed = _rest_state(p1.rest_site_mm()) if rec is None else None
        # A MODELLED solve: `install_segment` re-reads the clock after the QP
        # returns and measures the splice knot against the wire, so passing
        # `t_now + solve_ms` exercises the wire budget the real chain spends.
        # `None` (the default) models an instantaneous solve and is why run 1
        # and run 2 never saw the 1-knot budget a 4-knot lead left.
        t_install = None if solve_ms <= 0.0 else t_now_s + solve_ms / 1e3
        new_rec, res, seg = ex.install_segment(rec, seed, kind, terminal, t_now_s,
                                               cfg=cfg, limits=limits, geom=geom,
                                               t_install_s=t_install)
        if res.accepted:
            state['record'] = new_rec
            # A spliced meta's report peaks describe the SOURCE plan including
            # the tail this splice discarded, so the max over installs is an
            # UPPER bound on what the machine actually flies — never below.
            r = new_rec.meta.report
            peaks.append((r.peak_leg_vel_mmps, r.peak_leg_acc_mmps2,
                          r.peak_leg_jerk_mmps3, r.peak_hand_acc_rps2))
        wall.append(res.plan_wall_s)
        return res

    execu = ex.SkillExecutor(schedule, installer, tracker=tracker)
    t_end = max(sk.t_abs_s for sk in schedule.skills) + 0.5
    t = clock['t']
    events = []
    # A TENTH of a knot, not a knot.  A splice clears the wire by
    # `LEAD_KNOTS - WIRE_READ_KNOTS` (3 knots, or 5 for a handoff), and a
    # dispatch instant is not on the knot grid, so ticking at `DT` quantised
    # every dispatch up to a whole knot late and spent a knot of that margin on
    # the harness's own sampling (measured 2026-09-12 against the then 1-knot
    # budget: the third install refused SPLICE_TOO_LATE at every cell, and the
    # same schedule installs end to end when dispatched at its own instants).
    # The node this stands in for polls faster than it emits; the harness must
    # too, or it measures itself rather than the budget.
    step = DT / 10.0
    while t < t_end and not execu.attempt_ended:
        clock['t'] = t
        events += execu.tick(t)
        t += step
    first_refusal = next(((sk, r) for _i, sk, r in execu.results if not r.accepted), None)
    return dict(ok=not execu.attempt_ended, code=(execu.end_code if execu.attempt_ended else 'OK'),
                detail=('%s#%d: %s' % (first_refusal[0].kind, first_refusal[0].ball_id,
                                       first_refusal[1].message[:150]) if first_refusal else ''),
                installs=len(wall), plan_ms_max=round(1e3 * max(wall), 1) if wall else 0.0,
                beta_s=round(schedule.beat_s, 4), transit_s=round(schedule.transit_s, 4),
                flight_s=round(T, 4),
                peak_vel=round(max(p[0] for p in peaks)) if peaks else 0,
                peak_acc=round(max(p[1] for p in peaks)) if peaks else 0,
                peak_jerk=round(max(p[2] for p in peaks)) if peaks else 0,
                peak_hand=round(max(p[3] for p in peaks)) if peaks else 0)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--apex', type=float, nargs='+', default=[0.9, 1.0])
    ap.add_argument('--sep', type=float, nargs='+', default=[40.0, 60.0, 80.0, 100.0])
    ap.add_argument('--dwell', type=float, nargs='+', default=[0.30, 0.35, 0.40, 0.45, 0.50])
    ap.add_argument('--jerk', type=float, nargs='+', default=[200000.0, 300000.0])
    ap.add_argument('--vel-acc', nargs='+', default=['300/5000', '500/5000', '800/8000'])
    ap.add_argument('--hand-acc', type=float, nargs='+', default=[3500.0, 3900.0])
    ap.add_argument('--n-throws', type=int, default=4)
    ap.add_argument('--launch-s', type=float, default=0.4)
    ap.add_argument('--solve-ms', type=float, default=0.0,
                    help='model the QP taking this long, so the splice is '
                         'measured against the wire the way the robot path '
                         'measures it (default 0 = instantaneous)')
    ap.add_argument('--out', default=None)
    args = ap.parse_args(argv)
    geom = StewartGeometry()
    rows = []
    for apex, sep, dwell, jerk, va, hacc in itertools.product(
            args.apex, args.sep, args.dwell, args.jerk, args.vel_acc, args.hand_acc):
        vel, acc = (float(x) for x in va.split('/'))
        unclamped = vel > 5000 or acc > 5000 or jerk > 200000
        limits = _limits(vel, acc, jerk, hacc, unclamped=unclamped)
        t0 = time.perf_counter()
        row = run_schedule(apex, sep, dwell, limits, geom, n_throws=args.n_throws,
                           launch_s=args.launch_s, solve_ms=args.solve_ms)
        row.update(apex_m=apex, sep_mm=sep, dwell_s=dwell, jerk=jerk, vel=vel, acc=acc,
                   hand_acc=hacc, solve_ms=args.solve_ms,
                   wall_s=round(time.perf_counter() - t0, 2))
        rows.append(row)
        print({k: row[k] for k in ('apex_m', 'sep_mm', 'dwell_s', 'jerk', 'vel', 'acc',
                                   'hand_acc', 'ok', 'code', 'installs', 'plan_ms_max')},
              flush=True)
    stamp = _dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    out = args.out or os.path.join(_ROOT, 'temp', 'probes', 'skills_segment_%s.csv' % stamp)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    keys = sorted({k for r in rows for k in r})
    with open(out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    # Markdown: per (apex, sep, jerk, vel/acc, hand) the feasible dwell set.
    cells = {}
    for r in rows:
        cells.setdefault((r['apex_m'], r['sep_mm'], r['jerk'], r['vel'], r['acc'], r['hand_acc']), []).append(r)
    md = ['# solve-ms %g' % args.solve_ms, '',
          '| apex | sep | jerk | vel/acc | hand | feasible dwells (s) | peak vel/acc/jerk/hand | plan ms max | refusals |',
          '|---|---|---|---|---|---|---|---|---|']
    for k in sorted(cells):
        rc = cells[k]
        oks = [r['dwell_s'] for r in rc if r['ok']]
        good = [r for r in rc if r['ok']]
        codes = sorted({r['code'] for r in rc if not r['ok']})
        pk = ('%d / %d / %dk / %d' % (max(r['peak_vel'] for r in good),
                                      max(r['peak_acc'] for r in good),
                                      max(r['peak_jerk'] for r in good) / 1e3,
                                      max(r['peak_hand'] for r in good))
              if good else '—')
        md.append('| %.1f | %.0f | %.0fk | %.0f/%.0f | %.0f | %s | %s | %.1f | %s |' % (
            k[0], k[1], k[2] / 1e3, k[3], k[4], k[5],
            ', '.join('%.2f' % d for d in oks) or '—', pk,
            max(r['plan_ms_max'] for r in rc), ', '.join(codes) or '—'))
    md = '\n'.join(md)
    with open(os.path.splitext(out)[0] + '.md', 'w') as f:
        f.write(md + '\n')
    print(md)
    print('wrote', out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
