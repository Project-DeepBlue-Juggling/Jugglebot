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
    python tools/admissible_sweep.py --site-pairs single \
        --single-apex 0.5 0.6 0.7 0.8 0.9

Deterministic (``tests/motion/test_unified_cycle.py::
test_planning_is_deterministic``); run twice and diff the YAML before quoting
a row. Must finish in well under 5 minutes -- the script reports its own wall
time.
"""
from __future__ import annotations

import argparse
import dataclasses
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

# ── R3's single-site grid (owner decision, plan § "R3", 2026-09-13) ─────────
# THROW(P1) from rest -> CATCH(P1) carrying the next same-site throw
# (dwell 0.30 s) -> ... -> CATCH -> REST, one ball, apex 0.9 m, site
# P1 = columns_sites(100)[0]. The box must be swept WIDER than the columns
# grid: flight commands spanning at least 0.75-0.95 s (apex 0.9 m -> 0.8570 s
# at the centre) and landing offsets out to +-40 mm -- the edges are set by
# what the real gate passes with margin, not by this grid's resolution.
SINGLE_SITE_FLIGHTS_S = (0.75, 0.80, 0.8570, 0.90, 0.95)
SINGLE_SITE_OFFSETS_MM = (-40.0, -30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 40.0)
#: R3's session leg-jerk limit (plan § "R3": 150 000 mm/s^3 -- "the most the
#: machine has flown"). Leg vel/acc and hand acc are unchanged from R2.
SINGLE_SITE_LEG_JERK_MMPS3 = 150_000.0
#: The window the ONE seeding throw (from rest -> the release the chain's
#: first ``_chained_catch_cell`` catches) plans over -- ``schedule.Pattern``'s
#: own ``launch_s`` default (0.4 s, "the sweep's measured minimum feasible
#: launch period" -- plan § 0), NOT the 0.30 s dwell: a same-site chain's
#: STEADY windows carry the dwell between a catch and the throw it carries,
#: but the FIRST throw of an attempt is always from rest and gets the longer
#: launch window (``schedule.compile_columns``'s ``i == 0`` case). Measured
#: 2026-09-13: seeding at ``dwell_s`` instead refused the 0.90/0.95 s flights
#: at the THROW cell itself (``HAND_LIMIT_ACC``) before any offset was even
#: tried -- an artefact of the scaffold throw's own window, not a fact about
#: the STEADY chain this box actually certifies.
SINGLE_SITE_LAUNCH_S = 0.4

# ── R3's --single-apex ladder grid (owner decision, 2026-09-14) ─────────────
# One single-site box PER apex, instead of one box spanning a flight band: an
# apex ladder (0.5/0.6/0.7/0.8/0.9 m) needs each rung certified separately --
# a self-toss at 0.5 m must never silently reuse the 0.9 m box (the defect
# this CLI closes; see motion/skills/admissible.py's module docstring).
SINGLE_APEX_FLIGHT_FRAC = (-0.20, -0.15, -0.10, -0.05, 0.0, 0.05, 0.10)
SINGLE_APEX_HALFWIDTH_M = 0.05

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
                limits: TrajectoryLimits, geom, cfg,
                offset_mm: Tuple[float, float] = (0.0, 0.0)):
    """Plan + gate the ONE throw a (site pair, flight) cell shares across
    every landing offset. Returns ``(ok, code, release_seed, takeoff_vel_mm_s)``
    -- the last two are ``None`` on a refusal.

    ``offset_mm`` shifts the ballistic TARGET only (``site_mm``, the physical
    launch site, is unchanged) -- the same convention ``_catch_cell`` /
    ``_chained_catch_cell`` use for their ``landing_mm``. Defaults to (0, 0):
    the shared per-flight seeding call this cell was written for never aims
    off-site. R3-h2 (2026-09-13) reuses it WITH an offset to gate the LAUNCH
    THROW from rest that R3's cold-start attempt (THROW from rest -> CATCH ->
    REST) actually carries the learner's command on -- the chained STEADY
    catch this module also certifies does not cover that segment.

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
    target_mm = to_site.throw_site_mm() + np.array(
        [float(offset_mm[0]), float(offset_mm[1]), 0.0])
    terminal = sg.ThrowTerminal(site_mm=to_site.throw_site_mm(),
                                target_mm=target_mm,
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


def _chained_catch_cell(site, release_seed, takeoff_vel_mm_s, flight_s: float,
                        dwell_s: float, offset_mm: Tuple[float, float],
                        limits: TrajectoryLimits, geom, cfg):
    """Plan + gate the CATCH-with-``then_throw`` (STEADY window) R3's chained
    single-site cycle actually flies, for one landing/launch offset (mm,
    relative to ``site``, applied to BOTH the incoming catch and the throw it
    carries -- the steady-state command).  Returns ``(ok, code,
    next_release_seed, next_takeoff_vel_mm_s)`` -- the last two ``None`` on a
    refusal.

    This is the cell the existing two-site ``_catch_cell`` does NOT measure: a
    standalone LANDING (no ``then_throw``) is a different, and for this case
    infeasible, segment shape (plan § 2.2's R2 amendment / ``schedule.
    ThenThrow``'s docstring: the split LANDING-then-THROW form refuses at every
    cell of a 480-cell grid; the whole-window STEADY form is what
    ``segments.plan_segment`` actually builds for a same-site
    ``CatchTerminal.then_throw`` and what R3's schedule dispatches).

    Two solves of the SAME window, deliberately, for the same reason
    :func:`_throw_cell` gives: ``sg.plan_segment`` (the production entry
    point) for the admitted / margin verdict, and a raw
    ``unified_cycle.plan_steady`` call for the release-terminal seed the NEXT
    catch in the chain needs (``plan_segment``'s returned ``Segment`` has
    already been extended past release by the SETTLE tail, so its ``meta`` can
    no longer satisfy ``release_state_from_meta``'s "release is the window's
    terminal knot" check). Planning is bit-for-bit deterministic, so the two
    solves agree.
    """
    landing_mm = site.catch_site_mm() + np.array(
        [float(offset_mm[0]), float(offset_mm[1]), 0.0])
    v_arrival = bal.arrival_velocity(takeoff_vel_mm_s, flight_s)
    t_land = float(flight_s)
    t_release = t_land + float(dwell_s)
    then_throw = sg.ThrowAfterCatch(t_release_s=t_release,
                                    site_mm=site.throw_site_mm(),
                                    target_mm=landing_mm, flight_s=float(flight_s))
    terminal = sg.CatchTerminal(landing_mm=landing_mm, landing_vel_mm_s=v_arrival,
                                t_land_s=t_land, rest_site_mm=site.rest_site_mm(),
                                then_throw=then_throw)
    try:
        segment = sg.plan_segment(sg.CATCH, release_seed, terminal, cfg, limits, geom)
    except uc.CycleInfeasible as exc:
        return False, exc.code, None, None
    if not _within_margin(segment.meta.report, limits):
        return False, 'MARGIN', None, None
    next_takeoff = segment.takeoff_vel_mm_s
    goals_a = uc.CycleGoals(period_s=t_release, throw_site_mm=then_throw.site_mm,
                            throw_target_mm=then_throw.target_mm,
                            flight_s=then_throw.flight_s, catch_site_mm=landing_mm,
                            catch_vel_mm_s=v_arrival, catch_t_s=t_land)
    plan_a, meta_a = uc.plan_steady(goals_a, release_seed, limits, geom)
    next_release_seed = uc.release_state_from_meta(meta_a, plan_a)
    return True, 'OK', next_release_seed, next_takeoff


def _max_rectangle(xs: Sequence[float], ys: Sequence[float], pass_fn,
                   must_contain: Optional[Tuple[float, float]] = None
                   ) -> Optional[Tuple[float, float, float, float]]:
    """Largest-area axis-aligned rectangle over the grid ``xs`` x ``ys`` (both
    sorted) all of whose cells satisfy ``pass_fn(x, y)``.

    ``must_contain`` restricts the search to rectangles that include that
    point. :func:`_flight_band_and_rect` passes the identity offset (0, 0):
    a box that excludes the identity command CLIPS a cold learner's first
    throw off the cup. Found 2026-09-14 on the first ``--single-apex`` sweep:
    at flights near 0.64 s a ring of small offsets (+/-10-20 mm) fails the
    chained catch's margin while (0, 0) and the large offsets pass, and the
    unconstrained largest rectangle routed around the origin (0.5 m box y
    pinned at -30 mm, 0.6-0.8 m boxes x in [-40, -30] mm). R3's own box
    contained the origin only because its three-flight grid missed those
    flights.

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
                    if must_contain is not None and not (
                            xs[xlo_i] <= must_contain[0] <= xs[xhi_i]
                            and ys[ylo_i] <= must_contain[1] <= ys[yhi_i]):
                        continue
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

    rect = _max_rectangle(list(offsets_mm), list(offsets_mm), _pass,
                          must_contain=(0.0, 0.0))
    return (flights_sorted[lo], flights_sorted[hi]), rect


def sweep(*, apexes_m: Sequence[float] = APEXES_M,
          flights_s: Optional[Sequence[float]] = None,
          offsets_mm: Sequence[float] = OFFSETS_MM, dwell_s: float = DWELL_S,
          separation_mm: float = SEPARATION_MM, leg_vel: float = LEG_VEL_MMPS,
          leg_acc: float = LEG_ACC_MMPS2, leg_jerk: float = LEG_JERK_MMPS3,
          hand_acc: float = HAND_ACC_RPS2, center_apex_m: Optional[float] = None,
          center_flight_s: Optional[float] = None,
          launch_s: float = SINGLE_SITE_LAUNCH_S,
          site_pairs: Optional[List[Tuple['st.Site', 'st.Site']]] = None,
          log=print) -> Tuple[List['ab.AdmissibleBox'], List[Dict]]:
    """Run the grid; returns ``(boxes, rows)`` -- ``rows`` for the table.

    ``site_pairs`` overrides the default both-directions pair built from
    ``sites.columns_sites(separation_mm)`` -- the small end-to-end test uses
    this to keep its grid to one pair. ``center_apex_m`` is the owner's
    operating apex the flight band is grown outward from (see
    :func:`_flight_band_and_rect`); defaults to the median of ``apexes_m``.

    ``flights_s``, given directly in seconds, bypasses the apex map
    (``sc.flight_s``) entirely -- for R3's single-site grid, whose flight span
    (owner decision: at least 0.75-0.95 s) was not chosen by picking apexes.
    ``apex_band_m`` is still recorded on the resulting box via ``sc.apex_m``
    (the exact inverse of ``sc.flight_s``) -- the same physical relationship
    run backward, not a new one. ``center_flight_s`` is then the
    flight the admitted band grows outward from; defaults to the median of
    ``flights_s``.

    A ``site_pairs`` entry with ``from_site.name == to_site.name`` is a
    SINGLE-site pair (R3): its one seeding THROW (from rest) plans over
    ``launch_s``, not ``dwell_s`` -- the same rest-to-release window
    ``schedule.compile_columns``'s own first throw uses -- and is then judged
    by the CHAINED cell (:func:`_chained_catch_cell`, the STEADY
    catch-with-``then_throw`` R3's schedule actually dispatches, then one more
    :func:`_catch_cell` on the throw it carries) rather than the two-site
    cell's standalone LANDING catch. The cross-site TRANSIT shortcut
    (``NO_TRANSIT``) does not apply -- a self-toss at one site returns after
    the FULL flight time, not a shortened cross-site transit.
    """
    geom = StewartGeometry()
    limits = _limits(leg_vel, leg_acc, leg_jerk, hand_acc)
    cfg = sg.SegmentConfig()
    if site_pairs is None:
        site0, site1 = st.columns_sites(separation_mm)
        site_pairs = [(site0, site1), (site1, site0)]
    if flights_s is not None:
        flights = [float(t) for t in flights_s]
        apex_band = (sc.apex_m(min(flights)), sc.apex_m(max(flights)))
        center_flight = (float(center_flight_s) if center_flight_s is not None
                         else sorted(flights)[len(flights) // 2])
    else:
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
        same_site = (from_site.name == to_site.name)
        pass_grid = {}   # (flight, dx, dy) -> bool
        for T in flights:
            t0 = time.perf_counter()
            ok, code, release_seed, takeoff = _throw_cell(
                from_site, to_site, T, (launch_s if same_site else dwell_s),
                limits, geom, cfg)
            row = dict(site_pair=(from_site.name, to_site.name),
                      flight_s=round(T, 4), offset_mm=None, ok=ok,
                      code=('THROW:%s' % code) if not ok else 'THROW:OK',
                      wall_s=round(time.perf_counter() - t0, 3))
            rows.append(row)
            log(row)
            if same_site:
                tau = T
            else:
                tau = sc.transit_s(T, dwell_s)
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
                    if same_site:
                        # The box is the intersection of two gates: the
                        # chained STEADY catch (below, R3-c) and the launch
                        # THROW from rest that a cold-start attempt (THROW
                        # from rest -> CATCH -> REST) actually carries the
                        # command on (R3-h2, 2026-09-13) -- neither covers the
                        # other's segment shape.
                        lok, lcode, _seed, _takeoff = _throw_cell(
                            from_site, to_site, T, launch_s, limits, geom, cfg,
                            offset_mm=(dx, dy))
                        cok, ccode, next_seed, next_takeoff = _chained_catch_cell(
                            to_site, release_seed, takeoff, T, dwell_s, (dx, dy),
                            limits, geom, cfg)
                        if cok:
                            fok, fcode = _catch_cell(to_site, next_seed, next_takeoff,
                                                     T, T, (dx, dy), limits, geom, cfg)
                            if not fok:
                                cok, ccode = False, 'CHAIN_CATCH:%s' % fcode
                        if not lok:
                            cok, ccode = False, 'LAUNCH:%s' % lcode
                    else:
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


def _apex_bands_overlap(a: Tuple[float, float], b: Tuple[float, float]) -> bool:
    """True when ``a`` and ``b`` share more than a boundary point (1e-9 m
    tolerance) -- mirrors ``admissible._apex_bands_overlap``: two bands that
    only TOUCH are adjacent, not ambiguous."""
    tol = 1e-9
    lo1, hi1 = a
    lo2, hi2 = b
    return lo1 < hi2 - tol and lo2 < hi1 - tol


def _refuse_overlapping_single_apex_bands(
        apexes_m: Sequence[float], halfwidth_m: float,
        ap: argparse.ArgumentParser) -> None:
    """Refuse (``ap.error``, exit 2) when two ``--single-apex`` values' bands
    ``(apex - h, apex + h)`` overlap -- ambiguous at ``admissible.select``
    time, so it is refused at ARGUMENT PARSING rather than left to surface as
    a confusing ``dump``/``load`` refusal after the (slow) sweep has run."""
    h = float(halfwidth_m)
    apexes = sorted(float(a) for a in apexes_m)
    for a, b in zip(apexes, apexes[1:]):
        if _apex_bands_overlap((a - h, a + h), (b - h, b + h)):
            ap.error(
                '--single-apex bands overlap: %.3f +/- %.3f m and %.3f +/- '
                '%.3f m -- widen the apex spacing or shrink '
                '--single-apex-halfwidth' % (a, h, b, h))


def _single_apex_boxes(apexes_m: Sequence[float], *, flight_frac: Sequence[float],
                       halfwidth_m: float, offsets_mm: Sequence[float],
                       dwell_s: float, separation_mm: float, leg_vel: float,
                       leg_acc: float, leg_jerk: float, hand_acc: float,
                       site: 'st.Site', log=print
                       ) -> Tuple[List['ab.AdmissibleBox'], List[Dict]]:
    """One single-site ``(site, site)`` box PER apex in ``apexes_m`` (R3 apex-
    ladder sweep, 2026-09-14): each apex gets its OWN flight grid
    (``sc.flight_s(apex) * (1 + f)`` for ``f`` in ``flight_frac``, centred on
    ``sc.flight_s(apex)``) and its OWN ``apex_band_m = (apex - h, apex + h)``
    -- NOT the grid-derived band :func:`sweep` would otherwise compute
    (neighbouring apexes' grid-derived bands overlap; the whole reason this
    CLI records the requested apex directly rather than inferring a band from
    the grid). Reuses :func:`sweep`'s single-site cell (the STEADY
    catch-with-``then_throw`` chain plus the launch-from-rest throw, R3-c /
    R3-h2) once per apex and overrides the returned box's ``apex_band_m`` via
    ``dataclasses.replace`` -- the box's ``landing_xy_m`` / ``flight_s`` /
    provenance are exactly what that sweep measured, untouched."""
    boxes: List['ab.AdmissibleBox'] = []
    rows: List[Dict] = []
    for a in apexes_m:
        centre = sc.flight_s(float(a))
        flights = [centre * (1.0 + float(f)) for f in flight_frac]
        a_boxes, a_rows = sweep(
            flights_s=flights, offsets_mm=offsets_mm, dwell_s=dwell_s,
            separation_mm=separation_mm, leg_vel=leg_vel, leg_acc=leg_acc,
            leg_jerk=leg_jerk, hand_acc=hand_acc, center_flight_s=centre,
            site_pairs=[(site, site)], log=log)
        h = float(halfwidth_m)
        boxes.append(dataclasses.replace(
            a_boxes[0], apex_band_m=(float(a) - h, float(a) + h)))
        rows.extend(a_rows)
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


def _xy(s: str) -> Tuple[float, float]:
    x, y = s.split(',')
    return (float(x), float(y))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', default=_DEFAULT_OUT)
    ap.add_argument('--quiet', action='store_true')
    ap.add_argument('--site-pairs', choices=('columns', 'single', 'both'),
                    default='columns',
                    help='which boxes to sweep: the two columns orderings '
                         '(default, unchanged), the R3 single-site self-toss '
                         'pair, or both -- one YAML needs ONE limits set '
                         '(ab.dump enforces it), so "both" sweeps every box '
                         'under the SAME --leg-*/--hand-acc')
    ap.add_argument('--apex', type=float, nargs='+', default=list(APEXES_M),
                    help='columns apex grid (m), mapped through sc.flight_s')
    ap.add_argument('--offsets-mm', type=float, nargs='+', default=list(OFFSETS_MM),
                    help='columns landing-offset grid per axis (mm)')
    flight_group = ap.add_mutually_exclusive_group()
    flight_group.add_argument(
        '--flights-s', type=float, nargs='+', default=None,
        help='single-site flight-time grid (s), direct -- bypasses the apex '
             'map (owner decision: at least 0.75-0.95 s); default %r. '
             'Mutually exclusive with --single-apex' % (list(SINGLE_SITE_FLIGHTS_S),))
    flight_group.add_argument(
        '--single-apex', type=float, nargs='+', default=None,
        help='sweep one single-site box PER apex (m) instead of one box '
             'spanning a flight band -- an apex LADDER (e.g. 0.5 0.6 0.7 0.8 '
             '0.9), each rung its own box: flight grid = sc.flight_s(apex) * '
             '(1 + f) for f in --single-flight-frac, apex_band_m = (apex - h, '
             'apex + h) for h = --single-apex-halfwidth. Refused at parse '
             'time if two requested apexes\' bands overlap. Mutually '
             'exclusive with --flights-s')
    ap.add_argument('--single-flight-frac', type=float, nargs='+',
                    default=list(SINGLE_APEX_FLIGHT_FRAC),
                    help='--single-apex only: fractional flight-time offsets '
                         'from sc.flight_s(apex) for each apex\'s grid')
    ap.add_argument('--single-apex-halfwidth', type=float,
                    default=SINGLE_APEX_HALFWIDTH_M,
                    help='--single-apex only: +/- half-width (m) of each '
                         'resulting box\'s apex_band_m')
    ap.add_argument('--single-offsets-mm', type=float, nargs='+',
                    default=list(SINGLE_SITE_OFFSETS_MM),
                    help='single-site landing-offset grid per axis (mm)')
    ap.add_argument('--single-site-xy', type=_xy, default=None,
                    help='"x,y" mm of the single site; default P1 = '
                         'sites.columns_sites(--separation-mm)[0]')
    ap.add_argument('--dwell-s', type=float, default=DWELL_S)
    ap.add_argument('--separation-mm', type=float, default=SEPARATION_MM)
    ap.add_argument('--leg-vel', type=float, default=LEG_VEL_MMPS)
    ap.add_argument('--leg-acc', type=float, default=LEG_ACC_MMPS2)
    ap.add_argument('--leg-jerk', type=float, default=LEG_JERK_MMPS3)
    ap.add_argument('--hand-acc', type=float, default=HAND_ACC_RPS2)
    args = ap.parse_args(argv)
    if args.single_apex is not None:
        _refuse_overlapping_single_apex_bands(
            args.single_apex, args.single_apex_halfwidth, ap)
    log = (lambda r: None) if args.quiet else print

    t_start = time.perf_counter()
    boxes: List['ab.AdmissibleBox'] = []
    if args.site_pairs in ('columns', 'both'):
        cboxes, _rows = sweep(
            apexes_m=args.apex, offsets_mm=args.offsets_mm, dwell_s=args.dwell_s,
            separation_mm=args.separation_mm, leg_vel=args.leg_vel,
            leg_acc=args.leg_acc, leg_jerk=args.leg_jerk, hand_acc=args.hand_acc,
            log=log)
        boxes.extend(cboxes)
    if args.site_pairs in ('single', 'both'):
        if args.single_site_xy is not None:
            sx, sy = args.single_site_xy
            site = st.Site('P1', np.array([sx, sy, st.CATCH_CUP_Z_MM]))
        else:
            site = st.columns_sites(args.separation_mm)[0]
        if args.single_apex is not None:
            sboxes, _rows = _single_apex_boxes(
                args.single_apex, flight_frac=args.single_flight_frac,
                halfwidth_m=args.single_apex_halfwidth,
                offsets_mm=args.single_offsets_mm, dwell_s=args.dwell_s,
                separation_mm=args.separation_mm, leg_vel=args.leg_vel,
                leg_acc=args.leg_acc, leg_jerk=args.leg_jerk,
                hand_acc=args.hand_acc, site=site, log=log)
        else:
            flights_s = (args.flights_s if args.flights_s is not None
                        else list(SINGLE_SITE_FLIGHTS_S))
            sboxes, _rows = sweep(
                flights_s=flights_s, offsets_mm=args.single_offsets_mm,
                dwell_s=args.dwell_s, leg_vel=args.leg_vel, leg_acc=args.leg_acc,
                leg_jerk=args.leg_jerk, hand_acc=args.hand_acc,
                site_pairs=[(site, site)], log=log)
        boxes.extend(sboxes)
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
