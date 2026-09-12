#!/usr/bin/env python3
"""Skill-stack R2 -- the offline admissible-box sweep (plan § 2.2 / § 2.6).

Writes ``config/generated/admissible_box.yaml``: per (site pair, apex band),
the largest axis-aligned box of THROW commands ``u = (landing_xy_m relative to
the target site, flight_s)`` for which the REAL QP + gate
(``unified_cycle.plan_launch`` / ``plan_landing``, gated by
``feasibility.validate_cycle`` -- the same call ``skills.segments.plan_segment``
makes) admits every grid point with margin. See ``motion/skills/admissible.py``
for why this replaces a per-cycle call (``catch_reach.catch_reach_verdict``)
and what it does / does not fold in from ``throw_envelope.evaluate`` (C-HAND-3).

**The grid, and why it is shaped this way (owner decisions, plan § 0,
2026-09-12).** Two columns-pattern sites ``P1``/``P2`` 100 mm apart
(``sites.columns_sites``); apex band [0.85, 0.95] m centred on the owner's
0.9 m operating point, mapped through ``schedule.flight_s`` to three flight
times (~0.83 / 0.857 / 0.88 s) -- the SAME map ``tools/probes/
skills_sizing_sweep.py`` uses, so the band's centre cell is the one the owner
already validated at this leg/hand session limit; five landing offsets per
axis (-20/-10/0/10/20 mm), the identity prior's own command (0, 0) always
among them. For each of the two site-pair orderings (``P1``'s post-catch rest
seed feeding a throw+catch at ``P2``, and the reverse) this sweep re-plans the
platform's lateral TRANSIT between the two sites as part of the THROW's own
LAUNCH window -- this is I-CATCH-3's closed-form quintic reach frontier and
the old ``REJECTED_DISPLACEMENT`` pre-throw cap, both retired to this offline
box (``motion/skills/INVARIANTS.md`` rows I-CATCH-3 / ``REJECTED_DISPLACEMENT``).

Chain-building follows ``tools/probes/skills_sizing_sweep.py``'s style (a rest
seed built the same way, real ``unified_cycle`` calls, ``uc.CycleInfeasible``
caught and reported by code) but plans through ``skills.segments.plan_segment``
so the admitted set is judged by the PRODUCTION entry point, not a bespoke
chain.

Usage (venv)::

    python tools/admissible_sweep.py
    python tools/admissible_sweep.py --out /tmp/admissible_box.yaml

Deterministic (``tests/motion/test_unified_cycle.py::
test_planning_is_deterministic``); run twice and diff the YAML before quoting
a row. Must finish in well under 5 minutes -- the script reports its own wall
time.
"""
from __future__ import annotations

import argparse
import datetime as _dt
import os
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, 'ros_ws', 'src', 'jugglebot')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np                                                     # noqa: E402

import jugglebot.hardware_config as hw                                 # noqa: E402
from jugglebot.motion import unified_cycle as uc                       # noqa: E402
from jugglebot.motion.geometry import StewartGeometry                  # noqa: E402
from jugglebot.motion.trajectory import ballistics_bc as bal           # noqa: E402
from jugglebot.motion.trajectory import cup_realize as cr              # noqa: E402
from jugglebot.motion.trajectory import throw_envelope                 # noqa: E402
from jugglebot.motion.trajectory.limits import TrajectoryLimits        # noqa: E402
from jugglebot.motion.skills import admissible as ab                   # noqa: E402
from jugglebot.motion.skills import schedule as sc                     # noqa: E402
from jugglebot.motion.skills import segments as sg                     # noqa: E402
from jugglebot.motion.skills import sites as st                        # noqa: E402

# ── the owner's R2 operating point (plan § 0, 2026-09-12) ───────────────────
SEPARATION_MM = 100.0
APEXES_M = (0.85, 0.90, 0.95)
OFFSETS_MM = (-20.0, -10.0, 0.0, 10.0, 20.0)
DWELL_S = 0.30
LEG_VEL_MMPS = 300.0
LEG_ACC_MMPS2 = 5000.0
LEG_JERK_MMPS3 = 200_000.0
HAND_ACC_RPS2 = 3500.0

_DEFAULT_OUT = os.path.join(_ROOT, 'config', 'generated', 'admissible_box.yaml')


def _rest_state(cup_mm):
    """A plan seed at rest at ``cup_mm`` -- identical construction to
    ``tests/motion/test_skills_segments.py::_rest_state`` and
    ``tools/probes/skills_sizing_sweep.py::_rest_state``."""
    rcfg = cr.RealizeConfig()
    slider_mm = float(cup_mm[2]) - rcfg.cup_z_base_mm
    rev = ((slider_mm - rcfg.slider_rev_zero_mm) / 1000.0) * cr.HAND_REV_PER_M
    pose = np.array([cup_mm[0], cup_mm[1], rcfg.active_z_mm, 0.0, 0.0, 0.0])
    return uc.CycleState.at_rest(pose, rev, rcfg)


def _limits(leg_vel, leg_acc, leg_jerk, hand_acc) -> TrajectoryLimits:
    return TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=float(leg_vel), leg_acc_mmps2=float(leg_acc),
        leg_jerk_mmps3=float(leg_jerk), hand_acc_rps2=float(hand_acc))


def _within_margin(report, limits: TrajectoryLimits) -> bool:
    return (report.peak_leg_vel_mmps <= ab.MARGIN_FRAC * limits.leg_vel_mmps
            and report.peak_leg_acc_mmps2 <= ab.MARGIN_FRAC * limits.leg_acc_mmps2
            and report.peak_leg_jerk_mmps3 <= ab.MARGIN_FRAC * limits.leg_jerk_mmps3)


def _throw_cell(from_site, to_site, flight_s: float, dwell_s: float,
                limits: TrajectoryLimits, geom, cfg):
    """Plan + gate the ONE throw a (site pair, flight) cell shares across
    every landing offset. Returns ``(ok, code, release_seed, takeoff_vel_mm_s)``
    -- the last two are ``None`` on a refusal.

    Two solves of the SAME LAUNCH, deliberately: ``sg.plan_segment`` (the
    production entry point, extended with the SETTLE tail per I-PLAN-7 --
    gated and reported exactly as the executor gates it) for the admitted /
    margin verdict, and a raw ``unified_cycle.plan_launch`` call for the
    release-terminal seed the CATCH chains from. ``plan_segment``'s returned
    ``Segment`` has already been extended past release, so its ``meta`` can no
    longer satisfy ``release_state_from_meta``'s "release is the window's
    terminal knot" check (see ``unified_cycle.release_state_from_meta``).
    Planning is bit-for-bit deterministic, so the two solves agree.
    """
    seed = _rest_state(from_site.rest_site_mm())
    terminal = sg.ThrowTerminal(site_mm=to_site.throw_site_mm(),
                                target_mm=to_site.throw_site_mm(),
                                flight_s=flight_s, t_release_s=dwell_s)
    try:
        segment = sg.plan_segment(sg.THROW, seed, terminal, cfg, limits, geom)
    except uc.CycleInfeasible as exc:
        return False, exc.code, None, None
    if not _within_margin(segment.meta.report, limits):
        return False, 'MARGIN', None, None
    takeoff = segment.takeoff_vel_mm_s
    speed_mps = float(np.linalg.norm(takeoff)) / 1000.0
    verdict = throw_envelope.evaluate(flight_s, speed_mps)
    if not verdict.ok:
        return False, 'ENVELOPE:%s' % verdict.bound, None, None
    goals = uc.CycleGoals(period_s=dwell_s, throw_site_mm=terminal.site_mm,
                          throw_target_mm=terminal.target_mm, flight_s=flight_s)
    plan_a, meta_a = uc.plan_launch(goals, seed, limits, geom)
    release_seed = uc.release_state_from_meta(meta_a, plan_a)
    return True, 'OK', release_seed, takeoff


def _catch_cell(to_site, release_seed, takeoff_vel_mm_s, flight_s: float,
                tau_s: float, offset_mm: Tuple[float, float],
                limits: TrajectoryLimits, geom, cfg):
    """Plan + gate the CATCH for one landing offset (mm, relative to
    ``to_site``). Returns ``(ok, code)``."""
    landing_mm = to_site.catch_site_mm() + np.array(
        [float(offset_mm[0]), float(offset_mm[1]), 0.0])
    v_arrival = bal.arrival_velocity(takeoff_vel_mm_s, flight_s)
    terminal = sg.CatchTerminal(landing_mm=landing_mm, landing_vel_mm_s=v_arrival,
                                t_land_s=tau_s, rest_site_mm=to_site.rest_site_mm())
    try:
        segment = sg.plan_segment(sg.CATCH, release_seed, terminal, cfg, limits, geom)
    except uc.CycleInfeasible as exc:
        return False, exc.code
    if not _within_margin(segment.meta.report, limits):
        return False, 'MARGIN'
    return True, 'OK'


def _max_rectangle(xs: Sequence[float], ys: Sequence[float], pass_fn
                   ) -> Optional[Tuple[float, float, float, float]]:
    """Largest-area axis-aligned rectangle over the grid ``xs`` x ``ys`` (both
    sorted) all of whose cells satisfy ``pass_fn(x, y)``.

    Brute force over the O(n^2) index sub-ranges per axis -- trivially cheap
    at the grid sizes this sweep ever runs (<= 5x5): no solve happens here,
    every ``pass_fn`` call is a dict lookup already computed by the sweep.
    Ties (equal area) prefer the rectangle covering more grid points, i.e. the
    coarser statement of the same admissible set. Returns ``None`` when no
    single cell passes.
    """
    n, m = len(xs), len(ys)
    best = None          # (xlo, xhi, ylo, yhi)
    best_key = (-1.0, -1)
    for xlo_i in range(n):
        for xhi_i in range(xlo_i, n):
            for ylo_i in range(m):
                for yhi_i in range(ylo_i, m):
                    ok = all(pass_fn(xs[i], ys[j])
                             for i in range(xlo_i, xhi_i + 1)
                             for j in range(ylo_i, yhi_i + 1))
                    if not ok:
                        continue
                    area = (xs[xhi_i] - xs[xlo_i]) * (ys[yhi_i] - ys[ylo_i])
                    n_points = (xhi_i - xlo_i + 1) * (yhi_i - ylo_i + 1)
                    key = (area, n_points)
                    if key > best_key:
                        best_key = key
                        best = (xs[xlo_i], xs[xhi_i], ys[ylo_i], ys[yhi_i])
    return best


def _flight_band_and_rect(flights: Sequence[float], offsets_mm: Sequence[float],
                          pass_grid: Dict, center_flight: float
                          ) -> Tuple[Optional[Tuple[float, float]],
                                    Optional[Tuple[float, float, float, float]]]:
    """The admitted flight sub-band around ``center_flight``, and the xy
    rectangle over it.

    Grows OUTWARD from the flight closest to ``center_flight`` (the owner's
    operating apex, always in ``apexes_m``) while the offset-(0, 0) cell --
    the identity-prior command every R2 columns THROW actually issues -- keeps
    passing at each newly-included flight. A flight this sweep admits at NO
    offset at all (this run: apex 0.95 m fails ``HAND_LIMIT_ACC`` on the CATCH
    even at zero offset -- the higher apex's faster arrival speed alone
    exceeds the session hand-acceleration limit, independent of any landing
    miss) is excluded from the band rather than collapsing the whole box to
    empty. Returns ``(None, None)`` only when the CENTRE flight itself fails
    at zero offset -- the owner's own operating point failing is not a band
    to narrow around, it is a sweep the caller must stop and look at.
    """
    flights_sorted = sorted(flights)
    center_i = min(range(len(flights_sorted)),
                   key=lambda i: abs(flights_sorted[i] - center_flight))
    if not pass_grid.get((flights_sorted[center_i], 0.0, 0.0), False):
        return None, None
    lo = hi = center_i
    while lo - 1 >= 0 and pass_grid.get((flights_sorted[lo - 1], 0.0, 0.0), False):
        lo -= 1
    while (hi + 1 < len(flights_sorted)
           and pass_grid.get((flights_sorted[hi + 1], 0.0, 0.0), False)):
        hi += 1
    band = flights_sorted[lo:hi + 1]

    def _pass(x, y, _band=band):
        return all(pass_grid.get((T, x, y), False) for T in _band)

    rect = _max_rectangle(list(offsets_mm), list(offsets_mm), _pass)
    return (flights_sorted[lo], flights_sorted[hi]), rect


def sweep(*, apexes_m: Sequence[float] = APEXES_M,
          offsets_mm: Sequence[float] = OFFSETS_MM, dwell_s: float = DWELL_S,
          separation_mm: float = SEPARATION_MM, leg_vel: float = LEG_VEL_MMPS,
          leg_acc: float = LEG_ACC_MMPS2, leg_jerk: float = LEG_JERK_MMPS3,
          hand_acc: float = HAND_ACC_RPS2, center_apex_m: Optional[float] = None,
          site_pairs: Optional[List[Tuple['st.Site', 'st.Site']]] = None,
          log=print) -> Tuple[List['ab.AdmissibleBox'], List[Dict]]:
    """Run the grid; returns ``(boxes, rows)`` -- ``rows`` for the table.

    ``site_pairs`` overrides the default both-directions pair built from
    ``sites.columns_sites(separation_mm)`` -- the small end-to-end test uses
    this to keep its grid to one pair. ``center_apex_m`` is the owner's
    operating apex the flight band is grown outward from (see
    :func:`_flight_band_and_rect`); defaults to the median of ``apexes_m``.
    """
    geom = StewartGeometry()
    limits = _limits(leg_vel, leg_acc, leg_jerk, hand_acc)
    cfg = sg.SegmentConfig()
    if site_pairs is None:
        site0, site1 = st.columns_sites(separation_mm)
        site_pairs = [(site0, site1), (site1, site0)]
    flights = [sc.flight_s(a) for a in apexes_m]
    apex_band = (min(float(a) for a in apexes_m), max(float(a) for a in apexes_m))
    center_apex = (float(center_apex_m) if center_apex_m is not None
                  else sorted(float(a) for a in apexes_m)[len(apexes_m) // 2])
    center_flight = sc.flight_s(center_apex)
    ghash = ab.gate_hash()
    swept_at = _dt.date.today().isoformat()
    limits_dict = dict(leg_vel_mmps=float(leg_vel), leg_acc_mmps2=float(leg_acc),
                       leg_jerk_mmps3=float(leg_jerk), hand_acc_rps2=float(hand_acc))

    boxes: List[ab.AdmissibleBox] = []
    rows: List[Dict] = []
    for from_site, to_site in site_pairs:
        pass_grid = {}   # (flight, dx, dy) -> bool
        for T in flights:
            tau = sc.transit_s(T, dwell_s)
            t0 = time.perf_counter()
            ok, code, release_seed, takeoff = _throw_cell(
                from_site, to_site, T, dwell_s, limits, geom, cfg)
            row = dict(site_pair=(from_site.name, to_site.name),
                      flight_s=round(T, 4), offset_mm=None, ok=ok,
                      code=('THROW:%s' % code) if not ok else 'THROW:OK',
                      wall_s=round(time.perf_counter() - t0, 3))
            rows.append(row)
            log(row)
            if ok and not tau > 2.0 * float(hw.JB_TRAJ_KNOT_DT_S):
                ok, code = False, 'NO_TRANSIT'
            if not ok:
                for dx in offsets_mm:
                    for dy in offsets_mm:
                        pass_grid[(T, dx, dy)] = False
                continue
            for dx in offsets_mm:
                for dy in offsets_mm:
                    t1 = time.perf_counter()
                    cok, ccode = _catch_cell(to_site, release_seed, takeoff, T,
                                             tau, (dx, dy), limits, geom, cfg)
                    pass_grid[(T, dx, dy)] = cok
                    rows.append(dict(site_pair=(from_site.name, to_site.name),
                                     flight_s=round(T, 4), offset_mm=(dx, dy),
                                     ok=cok, code=ccode,
                                     wall_s=round(time.perf_counter() - t1, 3)))
                    log(rows[-1])

        flight_band, rect = _flight_band_and_rect(flights, offsets_mm, pass_grid,
                                                  center_flight)
        if flight_band is None:
            log({'site_pair': (from_site.name, to_site.name), 'WARNING':
                'the CENTRE flight %.4f s fails at zero offset -- the owner '
                'operating point itself is refused, not merely a margin edge'
                % center_flight})
        if rect is None:
            landing_xy = ((float('nan'), float('nan')), (float('nan'), float('nan')))
            flight_bounds = (float('nan'), float('nan'))
        else:
            xlo, xhi, ylo, yhi = rect
            landing_xy = ((xlo / 1000.0, xhi / 1000.0), (ylo / 1000.0, yhi / 1000.0))
            flight_bounds = flight_band
        boxes.append(ab.AdmissibleBox(
            site_pair=(from_site.name, to_site.name), apex_band_m=apex_band,
            landing_xy_m=landing_xy, flight_s=flight_bounds, limits=limits_dict,
            gate_hash=ghash, swept_at=swept_at))
    return boxes, rows


def to_markdown(boxes: List['ab.AdmissibleBox']) -> str:
    out = ['| site pair | apex band m | landing xy box (mm) | flight band (s) |'
          ' limits (vel/acc/jerk mm, hand acc rev) |',
          '|---|---|---|---|---|']
    for box in boxes:
        if box.empty:
            xy_str, fl_str = 'EMPTY', 'EMPTY'
        else:
            (xlo, xhi), (ylo, yhi) = box.landing_xy_m
            xy_str = '[%.0f, %.0f] x [%.0f, %.0f]' % (
                xlo * 1000.0, xhi * 1000.0, ylo * 1000.0, yhi * 1000.0)
            fl_str = '%.4f-%.4f' % box.flight_s
        out.append('| %s | %.2f-%.2f | %s | %s | %.0f/%.0f/%.0fk, %.0f |' % (
            box.site_pair, box.apex_band_m[0], box.apex_band_m[1], xy_str, fl_str,
            box.limits['leg_vel_mmps'], box.limits['leg_acc_mmps2'],
            box.limits['leg_jerk_mmps3'] / 1000.0, box.limits['hand_acc_rps2']))
    return '\n'.join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', default=_DEFAULT_OUT)
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args(argv)
    t_start = time.perf_counter()
    boxes, _rows = sweep(log=(lambda r: None) if args.quiet else print)
    wall_s = time.perf_counter() - t_start
    md = to_markdown(boxes)
    print(md)
    ab.dump(args.out, boxes)
    print('wrote', args.out)
    print('wall time: %.1f s (%.2f min)' % (wall_s, wall_s / 60.0))
    if wall_s > 300.0:
        print('WARNING: sweep exceeded the 5-minute budget', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
