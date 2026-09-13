#!/usr/bin/env python3
"""Skill-stack R3 sizing, SINGLE-SITE cycle: THROW(P1) -> CATCH(P1) -> REST.

R3 flies one site, repeated (plan `plans/active/two-ball-skill-stack.md` R3
section). This harness drives the REAL install chain --
``segments.plan_segment`` (cup QP -> tilt -> decompose -> gate) through
``executor.install_segment`` -- with a perfect analytic tracker and no plant,
exactly like ``skills_segment_sweep.py`` does for the two-site columns
pattern. There is no single-site *compiler* (``schedule.compile_columns`` is
two-site only) so every schedule below is hand-built, one ``install_segment``
call at a time, rather than adding one.

Two forms of a self-toss at ONE site:

* **F1 discrete** -- THROW from rest -> CATCH as a standalone LANDING (no
  ``then_throw``) -> REST, and the NEXT throw starts again from rest. Nothing
  is spliced; every segment is a fresh, rest-seeded install. This is three
  repetitions of the three reference scenarios ``segments.REST_TAIL_S`` was
  already probed against (throw-from-rest, catch-landing, a rest hold),
  chained by feeding each segment's own end-of-plan rest state to the next.
* **F2 chained** -- THROW from rest, then every subsequent throw is CARRIED
  by the previous catch (``segments.ThrowAfterCatch`` / a STEADY window), the
  columns pattern's ``ThenThrow`` fold with the SAME site both ends, so the
  platform never stops between a catch and the throw it carries. The last
  catch is standalone and a REST follows it, two ``rest_tail_s`` later --
  the same dispatch ``schedule.compile_columns`` uses for its own REST.
  Each catch-with-throw is genuinely SPLICED into the still-streaming
  previous segment, dispatched ``schedule.HANDOFF_LEAD_S`` before the release
  it is carried out of, exactly as ``compile_columns`` dispatches a columns
  handoff -- so the splice lands on the release knot
  (``executor._snap_to_release``) regardless of the raw dispatch instant.

Usage (venv, run under ``nice``/single-threaded BLAS per CLAUDE.md)::

    OPENBLAS_NUM_THREADS=1 nice -n 19 python tools/probes/skills_single_site_sweep.py \\
        --study grid --apex 0.7 0.8 0.9 --jerk 150000 200000 \\
        --out temp/probes/skills_single_site_grid_run1.csv

    OPENBLAS_NUM_THREADS=1 nice -n 19 python tools/probes/skills_single_site_sweep.py \\
        --study reach --form F1 --apex 0.9 --site 0,0 --jerk 150000 \\
        --out temp/probes/skills_single_site_reach_run1.csv

    OPENBLAS_NUM_THREADS=1 nice -n 19 python tools/probes/skills_single_site_sweep.py \\
        --study seed --apex 0.9 --site -50,0 --jerk 150000 \\
        --out temp/probes/skills_single_site_seed_run1.csv

Deterministic (the planner is, given fixed floats); run a study twice with
different ``--out`` and ``diff`` the two CSVs before quoting a row.
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import datetime as _dt
import itertools
import os
import sys
import time

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (_ROOT, os.path.join(_ROOT, 'ros_ws', 'src', 'jugglebot'), _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np                                                    # noqa: E402

import jugglebot.hardware_config as hw                                # noqa: E402
from jugglebot.motion.geometry import StewartGeometry                 # noqa: E402
from jugglebot.motion import levelling as lv                          # noqa: E402
from jugglebot.motion import unified_cycle as uc                      # noqa: E402
from jugglebot.motion.trajectory import ballistics_bc as bal          # noqa: E402
from jugglebot.motion.trajectory import cup_realize as cr             # noqa: E402
from jugglebot.motion.skills import executor as ex                    # noqa: E402
from jugglebot.motion.skills import schedule as sc                    # noqa: E402
from jugglebot.motion.skills import segments as sg                    # noqa: E402
from jugglebot.motion.skills import sites as si                       # noqa: E402

# Reuse, do not copy (brief instruction) -- both are plain functions in the
# columns-sizing probe, unmodified.
import skills_segment_sweep as seg_sweep                              # noqa: E402

DT = float(hw.JB_TRAJ_KNOT_DT_S)


def _site(x_mm: float, y_mm: float, name: str = 'S') -> si.Site:
    return si.Site(name, np.array([float(x_mm), float(y_mm), si.CATCH_CUP_Z_MM]))


def _rest_seed(site: si.Site) -> uc.CycleState:
    """The post-catch / post-throw rest seed: cup at the settle floor (689.6 mm)
    -- the SAME seed ``segments.REST_TAIL_S``'s three reference scenarios use."""
    return seg_sweep._rest_state(site.rest_site_mm())


def _rest_from_record(rec: ex.PlanRecord) -> uc.CycleState:
    return uc.CycleState.at_rest(rec.plan.pose[-1], float(rec.plan.hand_rev[-1]),
                                 levelling_correction=rec.meta.levelling_correction)


def _throw_terminal(site: si.Site, t_release_abs_s: float, flight_s: float,
                    offset_mm=(0.0, 0.0)) -> sg.ThrowTerminal:
    target = site.catch_site_mm() + np.array([offset_mm[0], offset_mm[1], 0.0])
    return sg.ThrowTerminal(site_mm=site.throw_site_mm(), target_mm=target,
                            flight_s=float(flight_s), t_release_s=float(t_release_abs_s))


def _catch_terminal(site: si.Site, t_land_abs_s: float, flight_s: float, *,
                    pos_perturb_mm=(0.0, 0.0, 0.0), then_throw=None) -> sg.CatchTerminal:
    v_mm_s = 0.5 * bal.GRAVITY_MMS2 * float(flight_s)
    landing = site.catch_site_mm() + np.asarray(pos_perturb_mm, dtype=float)
    vel = np.array([0.0, 0.0, -v_mm_s])
    return sg.CatchTerminal(landing_mm=landing, landing_vel_mm_s=vel,
                            t_land_s=float(t_land_abs_s), rest_site_mm=site.rest_site_mm(),
                            then_throw=then_throw)


def _report(rec: ex.PlanRecord):
    r = rec.meta.report
    return (r.peak_leg_vel_mmps, r.peak_leg_acc_mmps2, r.peak_leg_jerk_mmps3,
            r.peak_hand_acc_rps2)


def _row(ok, code, detail, installs, peaks, wall):
    if peaks:
        pk = tuple(max(p[i] for p in peaks) for i in range(4))
    else:
        pk = (0.0, 0.0, 0.0, 0.0)
    return dict(ok=ok, code=code, detail=detail[:180], installs=installs,
               peak_vel=round(pk[0]), peak_acc=round(pk[1]), peak_jerk=round(pk[2]),
               peak_hand=round(pk[3]),
               plan_ms_max=round(1e3 * max(wall), 2) if wall else 0.0)


# ── F1: discrete, fresh-origin every step ────────────────────────────────────

def run_f1(site, apex_m, limits, geom, *, n_throws=6, launch_s=0.4,
          catch_window_s=0.3, rest_hold_s=None, offset_mm=(0.0, 0.0)):
    cfg = sg.SegmentConfig()
    rest_hold_s = 2.0 * cfg.rest_tail_s if rest_hold_s is None else rest_hold_s
    t_f = sc.flight_s(apex_m)
    seed = _rest_seed(site)
    peaks, wall = [], []
    for i in range(n_throws):
        for stage, kind, terminal in (
            ('THROW', sg.THROW, _throw_terminal(site, launch_s, t_f, offset_mm)),
            ('CATCH', sg.CATCH, _catch_terminal(site, catch_window_s, t_f)),
            ('REST', sg.REST, sg.RestTerminal(rest_site_mm=site.rest_site_mm(),
                                              t_rest_s=rest_hold_s)),
        ):
            rec, res, seg = ex.install_segment(None, seed, kind, terminal, 0.0,
                                               cfg=cfg, limits=limits, geom=geom)
            wall.append(res.plan_wall_s)
            if not res.accepted:
                return _row(False, res.code, '%s#%d: %s' % (stage, i, res.message),
                           len(wall), peaks, wall)
            peaks.append(_report(rec))
            seed = _rest_from_record(rec)
    return _row(True, 'OK', '', len(wall), peaks, wall)


# ── F2: chained, catch carries the next same-site throw ──────────────────────

def run_f2(site, apex_m, dwell_s, limits, geom, *, n_throws=6, launch_s=0.4,
          offset_mm=(0.0, 0.0)):
    cfg = sg.SegmentConfig()
    t_f = sc.flight_s(apex_m)
    tau = sc.transit_s(t_f, dwell_s)
    if not tau > 0.0:
        return _row(False, 'SCHEDULE', 'dwell %.3f s >= flight %.3f s' % (dwell_s, t_f),
                   0, [], [])
    seed = _rest_seed(site)
    peaks, wall = [], []

    # THROW 0, fresh, from rest.
    t_release = launch_s
    rec, res, seg = ex.install_segment(
        None, seed, sg.THROW, _throw_terminal(site, t_release, t_f, offset_mm), 0.0,
        cfg=cfg, limits=limits, geom=geom)
    wall.append(res.plan_wall_s)
    if not res.accepted:
        return _row(False, res.code, 'THROW#0: %s' % res.message, len(wall), peaks, wall)
    peaks.append(_report(rec))

    for i in range(1, n_throws):
        t_land = t_release + t_f
        t_next_release = t_land + dwell_s
        then_throw = sg.ThrowAfterCatch(
            t_release_s=t_next_release, site_mm=site.throw_site_mm(),
            target_mm=site.catch_site_mm() + np.array([offset_mm[0], offset_mm[1], 0.0]),
            flight_s=t_f)
        terminal = _catch_terminal(site, t_land, t_f, then_throw=then_throw)
        t_now = t_release - sc.HANDOFF_LEAD_S
        rec, res, seg = ex.install_segment(rec, None, sg.CATCH, terminal, t_now,
                                           lead_s=sc.HANDOFF_LEAD_S,
                                           cfg=cfg, limits=limits, geom=geom)
        wall.append(res.plan_wall_s)
        if not res.accepted:
            return _row(False, res.code, 'CATCH+THROW#%d: %s' % (i, res.message),
                       len(wall), peaks, wall)
        peaks.append(_report(rec))
        t_release = t_next_release

    # Last catch: standalone (no ball to carry onward).
    t_land = t_release + t_f
    terminal = _catch_terminal(site, t_land, t_f)
    t_now = t_release - sc.HANDOFF_LEAD_S
    rec, res, seg = ex.install_segment(rec, None, sg.CATCH, terminal, t_now,
                                       lead_s=sc.HANDOFF_LEAD_S,
                                       cfg=cfg, limits=limits, geom=geom)
    wall.append(res.plan_wall_s)
    if not res.accepted:
        return _row(False, res.code, 'CATCH#last: %s' % res.message, len(wall), peaks, wall)
    peaks.append(_report(rec))

    # REST, two rest_tail_s after the last touch-down (compile_columns's own
    # dispatch), spliced into the still-streaming catch tail.
    rest_t = t_land + 2.0 * cfg.rest_tail_s
    t_now = t_land + cfg.rest_tail_s - sc.LEAD_S
    rec, res, seg = ex.install_segment(
        rec, None, sg.REST, sg.RestTerminal(rest_site_mm=site.rest_site_mm(), t_rest_s=rest_t),
        t_now, lead_s=sc.LEAD_S, cfg=cfg, limits=limits, geom=geom)
    wall.append(res.plan_wall_s)
    if not res.accepted:
        return _row(False, res.code, 'REST: %s' % res.message, len(wall), peaks, wall)
    peaks.append(_report(rec))
    return _row(True, 'OK', '', len(wall), peaks, wall)


# ── study: grid (question 1) ─────────────────────────────────────────────────

def study_grid(args):
    geom = StewartGeometry()
    rows = []
    sites = [(0.0, 0.0), (-50.0, 0.0)]
    for apex, (sx, sy), jerk in itertools.product(args.apex, sites, args.jerk):
        site = _site(sx, sy)
        limits = seg_sweep._limits(args.vel, args.acc, jerk, args.hand_acc)
        t0 = time.perf_counter()
        row = run_f1(site, apex, limits, geom, n_throws=args.n_throws,
                    launch_s=args.launch_s, catch_window_s=args.catch_window_s)
        row.update(form='F1', apex_m=apex, site_x=sx, site_y=sy, jerk=jerk, dwell_s='',
                  vel=args.vel, acc=args.acc, hand_acc=args.hand_acc,
                  wall_s=round(time.perf_counter() - t0, 2))
        rows.append(row)
        print({k: row[k] for k in ('form', 'apex_m', 'site_x', 'site_y', 'jerk', 'dwell_s',
                                   'ok', 'code', 'peak_jerk', 'peak_acc')}, flush=True)
        for dwell in args.dwell:
            t0 = time.perf_counter()
            row = run_f2(site, apex, dwell, limits, geom, n_throws=args.n_throws,
                        launch_s=args.launch_s)
            row.update(form='F2', apex_m=apex, site_x=sx, site_y=sy, jerk=jerk, dwell_s=dwell,
                      vel=args.vel, acc=args.acc, hand_acc=args.hand_acc,
                      wall_s=round(time.perf_counter() - t0, 2))
            rows.append(row)
            print({k: row[k] for k in ('form', 'apex_m', 'site_x', 'site_y', 'jerk', 'dwell_s',
                                       'ok', 'code', 'peak_jerk', 'peak_acc')}, flush=True)
    return rows, ('form', 'apex_m', 'site_x', 'site_y', 'jerk', 'dwell_s', 'vel', 'acc',
                 'hand_acc', 'ok', 'code', 'peak_vel', 'peak_acc', 'peak_jerk', 'peak_hand',
                 'plan_ms_max', 'installs', 'wall_s', 'detail')


def _grid_markdown(rows):
    cells = {}
    for r in rows:
        cells.setdefault((r['form'], r['apex_m'], r['site_x'], r['site_y'], r['jerk'],
                         r['dwell_s']), r)
    md = ['# single-site grid (F1 discrete / F2 chained), legs %s/%s, hand %s'
          % (rows[0]['vel'], rows[0]['acc'], rows[0]['hand_acc']) if rows else '# (empty)', '',
          '| form | apex | site | jerk | dwell | ok | code | peak vel/acc/jerk/hand | plan ms max | <=150k? |',
          '|---|---|---|---|---|---|---|---|---|---|']
    for k in sorted(cells, key=lambda k: (k[0], k[1], k[2], k[3], k[4],
                                          k[5] if k[5] != '' else -1)):
        r = cells[k]
        flag = 'YES' if r['ok'] and r['peak_jerk'] <= 150000 else ('n/a' if not r['ok'] else 'no')
        md.append('| %s | %.1f | (%.0f,%.0f) | %.0fk | %s | %s | %s | %d / %d / %dk / %d | %.1f | %s |'
                  % (r['form'], r['apex_m'], r['site_x'], r['site_y'], r['jerk'] / 1e3,
                     ('%.2f' % r['dwell_s']) if r['dwell_s'] != '' else '-',
                     r['ok'], r['code'], r['peak_vel'], r['peak_acc'], r['peak_jerk'] / 1e3,
                     r['peak_hand'], r['plan_ms_max'], flag))
    return '\n'.join(md)


# ── study: reach band (question 2) ───────────────────────────────────────────

def study_reach(args):
    geom = StewartGeometry()
    site = _site(*args.site_xy)
    limits = seg_sweep._limits(args.vel, args.acc, args.jerk[0], args.hand_acc)
    t_f = sc.flight_s(args.apex[0])
    cfg = sg.SegmentConfig()
    rows = []

    deltas_mm = [0.0, 10.0, 20.0, 30.0, 40.0, 60.0]
    axes = [('x', (1.0, 0.0)), ('y', (0.0, 1.0)), ('diag', (1.0, 1.0))]
    seen = set()
    combos = []
    for d in deltas_mm:
        signs = (1,) if d == 0.0 else (1, -1)
        for name, (ux, uy) in axes:
            for sign in signs:
                combos.append((name, d, ux * d * sign, uy * d * sign))
    for name, d, pdx, pdy in combos:
        key = (round(pdx, 3), round(pdy, 3))
        if key in seen:
            continue
        seen.add(key)
        seed = _rest_seed(site)
        terminal = _catch_terminal(site, args.catch_window_s, t_f,
                                   pos_perturb_mm=(pdx, pdy, 0.0))
        t0 = time.perf_counter()
        rec, res, seg = ex.install_segment(None, seed, sg.CATCH, terminal, 0.0,
                                           cfg=cfg, limits=limits, geom=geom)
        pk = _report(rec) if res.accepted else (0, 0, 0, 0)
        rows.append(dict(kind='catch_xy', axis=name, delta_mm=d, dx_mm=pdx, dy_mm=pdy,
                         dt_ms=0.0, du_mm=0.0, ok=res.accepted, code=res.code,
                         detail=res.message[:180], peak_vel=round(pk[0]),
                         peak_acc=round(pk[1]), peak_jerk=round(pk[2]),
                         peak_hand=round(pk[3]),
                         plan_ms=round(1e3 * (time.perf_counter() - t0), 2)))
        print(rows[-1], flush=True)

    for dt_ms in (0.0, 20.0, -20.0, 50.0, -50.0, 100.0, -100.0):
        seed = _rest_seed(site)
        terminal = _catch_terminal(site, args.catch_window_s + dt_ms / 1e3, t_f)
        t0 = time.perf_counter()
        rec, res, seg = ex.install_segment(None, seed, sg.CATCH, terminal, 0.0,
                                           cfg=cfg, limits=limits, geom=geom)
        pk = _report(rec) if res.accepted else (0, 0, 0, 0)
        rows.append(dict(kind='catch_t', axis='t', delta_mm=0.0, dx_mm=0.0, dy_mm=0.0,
                         dt_ms=dt_ms, du_mm=0.0, ok=res.accepted, code=res.code,
                         detail=res.message[:180], peak_vel=round(pk[0]), peak_acc=round(pk[1]),
                         peak_jerk=round(pk[2]), peak_hand=round(pk[3]),
                         plan_ms=round(1e3 * (time.perf_counter() - t0), 2)))
        print(rows[-1], flush=True)

    for du_mm in (0.0, 10.0, -10.0, 20.0, -20.0):
        seed = _rest_seed(site)
        terminal = _throw_terminal(site, args.launch_s, t_f, offset_mm=(du_mm, 0.0))
        t0 = time.perf_counter()
        rec, res, seg = ex.install_segment(None, seed, sg.THROW, terminal, 0.0,
                                           cfg=cfg, limits=limits, geom=geom)
        pk = _report(rec) if res.accepted else (0, 0, 0, 0)
        rows.append(dict(kind='throw_u', axis='x', delta_mm=0.0, dx_mm=0.0, dy_mm=0.0,
                         dt_ms=0.0, du_mm=du_mm, ok=res.accepted, code=res.code,
                         detail=res.message[:180], peak_vel=round(pk[0]), peak_acc=round(pk[1]),
                         peak_jerk=round(pk[2]), peak_hand=round(pk[3]),
                         plan_ms=round(1e3 * (time.perf_counter() - t0), 2)))
        print(rows[-1], flush=True)

    return rows, ('kind', 'axis', 'delta_mm', 'dx_mm', 'dy_mm', 'dt_ms', 'du_mm', 'ok', 'code',
                 'peak_vel', 'peak_acc', 'peak_jerk', 'peak_hand', 'plan_ms', 'detail')


def _reach_markdown(rows, args):
    md = ['# reach band -- apex %.1f m, site %s, legs %s/%s/%s, hand %s'
          % (args.apex[0], args.site_xy, args.vel, args.acc, args.jerk[0], args.hand_acc), '']
    for kind, label in (('catch_xy', 'CATCH xy perturbation'),
                       ('catch_t', 'CATCH arrival-time perturbation'),
                       ('throw_u', 'THROW commanded-landing offset (u)')):
        sub = [r for r in rows if r['kind'] == kind]
        if not sub:
            continue
        md.append('## %s' % label)
        md.append('| axis | delta | ok | code | peak jerk | peak acc |')
        md.append('|---|---|---|---|---|---|')
        for r in sub:
            if kind == 'catch_xy':
                delta = '(%+.0f, %+.0f) mm' % (r['dx_mm'], r['dy_mm'])
            elif kind == 'catch_t':
                delta = '%+.0f ms' % r['dt_ms']
            else:
                delta = '%+.0f mm' % r['du_mm']
            md.append('| %s | %s | %s | %s | %d | %d |'
                      % (r['axis'], delta, r['ok'], r['code'] or '-', r['peak_jerk'],
                         r['peak_acc']))
        accepted = [r for r in sub if r['ok']]
        refused = [r for r in sub if not r['ok']]
        md.append('accepted: %d/%d; refusal codes: %s'
                  % (len(accepted), len(sub), sorted({r['code'] for r in refused}) or '-'))
        md.append('')
    return '\n'.join(md)


# ── study: session-start seed (question 3) ───────────────────────────────────

def study_seed(args):
    geom = StewartGeometry()
    site = _site(*args.site_xy)
    limits = seg_sweep._limits(args.vel, args.acc, args.jerk[0], args.hand_acc)
    t_f = sc.flight_s(args.apex[0])
    cfg = sg.SegmentConfig()
    rcfg = cr.RealizeConfig()
    rows = []

    # (1) ACTIVATE park: hand 0 rev, cup 679.6 mm -- 10 mm under the floor.
    pose_park = np.array([site.cup_mm[0], site.cup_mm[1], rcfg.active_z_mm, 0.0, 0.0, 0.0])
    seed_park = uc.CycleState.at_rest(pose_park, 0.0, rcfg)
    cup_z_park = float(seed_park.cup_pos_mm[2])
    terminal = _throw_terminal(site, args.launch_s, t_f)
    rec, res, seg = ex.install_segment(None, seed_park, sg.THROW, terminal, 0.0,
                                       cfg=cfg, limits=limits, geom=geom)
    rows.append(dict(check='park_refuses', cup_z_mm=round(cup_z_park, 2), ok=res.accepted,
                     code=res.code, detail=res.message[:200]))
    print(rows[-1], flush=True)

    # (2) The same THROW from a rest seed with the hand lifted to the floor
    #     (cup 689.6 mm, unified_cycle.SETTLE_CUP_Z_MM).
    seed_floor = _rest_seed(site)
    rec, res, seg = ex.install_segment(None, seed_floor, sg.THROW, terminal, 0.0,
                                       cfg=cfg, limits=limits, geom=geom)
    pk = _report(rec) if res.accepted else (0, 0, 0, 0)
    rows.append(dict(check='floor_accepts', cup_z_mm=round(float(seed_floor.cup_pos_mm[2]), 2),
                     ok=res.accepted, code=res.code, detail=res.message[:200],
                     peak_jerk=round(pk[2])))
    print(rows[-1], flush=True)

    # (3) Level-to-base rest with the plan's levelling_correction loaded:
    #     un-prelevelled (pose tilt 0,0) vs pre-levelled (pose tilt = the
    #     corrected commanded value) -- same correction on both.
    tilt_x, tilt_y = 0.0133, 0.0009
    correction = lv.correction_from_offset(tilt_x, tilt_y)
    base = _rest_seed(site)
    seed_unlevel = dataclasses.replace(base, levelling_correction=correction)
    pose_pre = base.pose.copy()
    pose_pre[3:5] = lv.correct_pose(np.zeros(6), correction)[3:5]
    seed_prelevel = dataclasses.replace(base, pose=pose_pre, levelling_correction=correction)

    for tag, seed in (('unprelevelled', seed_unlevel), ('prelevelled', seed_prelevel)):
        rec, res, seg = ex.install_segment(None, seed, sg.THROW, terminal, 0.0,
                                           cfg=cfg, limits=limits, geom=geom)
        pk = _report(rec) if res.accepted else (0, 0, 0, 0)
        rows.append(dict(check='tilt_%s' % tag, tilt_xy=[tilt_x, tilt_y],
                         pose_rx_ry=seed.pose[3:5].tolist(), ok=res.accepted, code=res.code,
                         detail=res.message[:200], peak_jerk=round(pk[2]),
                         peak_acc=round(pk[1]) if res.accepted else 0))
        print(rows[-1], flush=True)

    return rows, None


# ── main ──────────────────────────────────────────────────────────────────────

def _xy(s):
    x, y = s.split(',')
    return (float(x), float(y))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--study', choices=('grid', 'reach', 'seed'), default='grid')
    ap.add_argument('--apex', type=float, nargs='+', default=[0.7, 0.8, 0.9])
    ap.add_argument('--dwell', type=float, nargs='+', default=[0.30, 0.40, 0.50])
    ap.add_argument('--jerk', type=float, nargs='+', default=[150000.0, 200000.0])
    ap.add_argument('--vel', type=float, default=300.0)
    ap.add_argument('--acc', type=float, default=5000.0)
    ap.add_argument('--hand-acc', type=float, default=3500.0)
    ap.add_argument('--n-throws', type=int, default=6)
    ap.add_argument('--launch-s', type=float, default=0.4)
    ap.add_argument('--catch-window-s', type=float, default=0.3,
                    help='F1/reach standalone-CATCH window (s) -- not part of the '
                         'plan\'s dwell grid (dwell is an F2-only concept); chosen '
                         'to match the already-probed 0.30 s REST_TAIL_S scenario')
    ap.add_argument('--site-xy', type=_xy, default=(-50.0, 0.0),
                    help='"x,y" mm -- used by --study reach/seed')
    ap.add_argument('--form', choices=('F1', 'F2'), default='F1')
    ap.add_argument('--out', default=None)
    args = ap.parse_args(argv)

    if args.study == 'grid':
        rows, keys = study_grid(args)
        md = _grid_markdown(rows)
    elif args.study == 'reach':
        rows, keys = study_reach(args)
        md = _reach_markdown(rows, args)
    else:
        rows, keys = study_seed(args)
        md = '\n'.join('%s: %r' % (r['check'], r) for r in rows)

    stamp = _dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    out = args.out or os.path.join(_ROOT, 'temp', 'probes',
                                   'skills_single_site_%s_%s.csv' % (args.study, stamp))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    all_keys = keys if keys else sorted({k for r in rows for k in r})
    with open(out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(all_keys), extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)
    with open(os.path.splitext(out)[0] + '.md', 'w') as f:
        f.write(md + '\n')
    print(md)
    print('wrote', out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
