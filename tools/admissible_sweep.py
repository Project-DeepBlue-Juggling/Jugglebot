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
already validated at this leg/hand session limit; landing offsets per axis
out to +-20 mm, densified around the origin 2026-09-21 (+-0.5/1/2/4/6/8/10/20
mm) after the cup-contact contract's U4 re-sweep found the 2026-09-16 grid
had never sampled the 0-10 mm band where the old banking defect was worst --
the identity prior's own command (0, 0) always among them. For each of the
two site-pair orderings (``P1`` playing the site the platform just released
AT, ``P2`` the site it transits to and catches-with-throws at, and the
reverse) this sweep gates the columns pattern's OWN segments (R5, D4,
2026-09-30 -- see :func:`sweep`'s own docstring for the fix, and its "Why"
paragraph for the OLD transit-baked cell this replaced): the platform's
lateral TRANSIT between the two sites is absorbed into the CATCH's own STEADY
window (:func:`_columns_catch_throw_cell`), not baked into a THROW's LAUNCH.
This is still I-CATCH-3's closed-form quintic reach frontier and the old
``REJECTED_DISPLACEMENT`` pre-throw cap, both retired to this offline box
(``motion/skills/INVARIANTS.md`` rows I-CATCH-3 / ``REJECTED_DISPLACEMENT``)
-- only the segment SHAPE the box is swept from changed.

Chain-building follows ``tools/probes/skills_sizing_sweep.py``'s style (a rest
seed built the same way, real ``unified_cycle`` calls, ``uc.CycleInfeasible``
caught and reported by code) but plans through ``skills.segments.plan_segment``
so the admitted set is judged by the PRODUCTION entry point, not a bespoke
chain.

**R4 (2026-09-23): the cross-site HOP.** A hop releases at one columns site
and lands at the OTHER (``sites.columns_sites``'s two orderings, default
:data:`HOP_SEPARATION_MM` = 250 mm -- the owner's R4 target, wider than the
columns pair's 100 mm) -- a genuinely different segment shape from the
columns cell's cross-site catch-with-throw (a hop's THROW releases at the
ORIGIN site, not the destination; see :func:`_hop_throw_cell` /
:func:`_hop_catch_cell`), so :func:`hop_sweep` is its own driver, not a
``site_pairs`` override of :func:`sweep`. R5 (D4, 2026-09-30) adds a 0.80 m
row to the hop's own apex grid (:data:`HOP_APEXES_M`, independent of the
columns/self_toss :data:`APEXES_M`) and per-apex hop bands via
:func:`_single_apex_boxes`'s ``pattern='hop'``, mirroring R3's single-site
ladder. Every box now also carries a
``pattern`` (``admissible.PATTERNS``) and the release/target site xy it was
swept at (``motion/skills/admissible.py``'s module docstring, "Why" #1/#2) --
the executor's lookup key for a hop is genuinely (release, target), unlike
'columns'/'self_toss', which both key (site, site) because their release AND
target site coincide.

Usage (venv)::

    python tools/admissible_sweep.py
    python tools/admissible_sweep.py --out /tmp/admissible_box.yaml
    python tools/admissible_sweep.py --site-pairs single \
        --single-apex 0.5 0.6 0.7 0.8 0.9
    python tools/admissible_sweep.py --site-pairs hop --hop-separation-mm 250

Deterministic (``tests/motion/test_unified_cycle.py::
test_planning_is_deterministic``); run twice and diff the YAML before quoting
a row. Two operational rules learned 2026-09-27 (kinematic-calibration apply):

* **Pin BLAS to one thread** (``OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1``),
  as the live nodes do. Two default-threaded sweeps side by side thrashed the
  6-core Jetson (load average 18, ~0.2 cells/s each); pinned, two run side by
  side at ~19 cells/s each, ~32 min for ``--site-pairs all --single-apex 0.5
  0.6 0.7 0.8 0.9``.
* **Do not edit ANY gated file while a sweep runs** — not even a docstring.
  Every box records ``admissible.gate_hash()`` at its own write time, and the
  final write refuses a file whose boxes carry a mix (``ValueError: all boxes
  written to one file must share swept_at/gate_hash/limits``), so a comment
  edit to ``unified_cycle.py`` 20 minutes into a run discards the run. "Must finish in well under 5 minutes" is FALSE since the grid
densified 2026-09-21 (see the offset comment above): 2 985 -> 17 169 grid
cells (5.75x) measured 1745.0 s / 1710.7 s (29.08 / 28.51 min) across the two
determinism-check runs on 2026-09-21 -- the script still reports its own wall
time, read it, don't assume the old bound. The grid was NOT thinned to fit a
runtime budget: the 0-10 mm offsets it added were the hole the 2026-09-16 box
missed the old banking defect through (see above), and closing that hole was
the point of the re-sweep.
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
# Leg/hand limits are read from the GENERATED launch constants
# (config/hardware_config.yaml's `trajectory_op`, via config/generate_config.py)
# rather than pinned as literals here (2026-09-21, U4 re-sweep): a literal
# silently drifts from the launch point the next time it changes (exactly what
# happened to LEG_JERK_MMPS3 -- this module's old literal, 200_000, was the
# S4-era point; the launch default moved to 150_000 on 2026-09-16 and this
# tool's default did not follow until now). See ``config/generated/
# hardware_config.py``'s ``JB_TRAJ_LEG_*_LIMIT_*`` / ``JB_TRAJ_HAND_ACC_LIMIT_RPS2``.
SEPARATION_MM = 100.0
#: R4's cross-site HOP separation (owner target, 2026-09-16) -- wider than
#: the columns pair's 100 mm; :func:`hop_sweep` runs BOTH at this separation.
#: MEASURED 2026-09-23 (``scratchpad/probe_r4_hop.py``, R4's own operating
#: limits 300/5000/150000/3500): at 250 mm the hop's launch THROW (jerk
#: 126k), its CATCH+then_throw (88k) and the chained CATCH are all OK with
#: margin; a +40 mm landing offset REFUSES (dx -40/+-20 mm are OK) -- so the
#: box at this separation is expected to be narrower than the columns box's
#: symmetric +-20 mm, not empty.
HOP_SEPARATION_MM = 250.0
APEXES_M = (0.85, 0.90, 0.95)
#: R5's per-apex hop grid (D4, 2026-09-30) -- the hop's OWN apex list,
#: independent of :data:`APEXES_M` (columns/self_toss stay at [0.85, 0.95] m):
#: gains a 0.80 m row the columns grid deliberately does not carry. Used as
#: :func:`hop_sweep`'s default ``apexes_m`` and by ``--hop-apex`` (the same
#: "independent of the shared flag" precedent :data:`HOP_SEPARATION_MM` set).
HOP_APEXES_M = (0.80, 0.85, 0.90, 0.95)
#: Landing offsets per axis (mm): the original +-20/10/0 grid, densified
#: around the origin 2026-09-21 with +-0.5/1/2/4/6/8 mm -- the 2026-09-16 grid
#: never sampled the 0-10 mm band, exactly where the old banking defect (fixed
#: by the cup-contact contract, C-CUP-1) was worst.
OFFSETS_MM = (-20.0, -10.0, -8.0, -6.0, -4.0, -2.0, -1.0, -0.5, 0.0,
             0.5, 1.0, 2.0, 4.0, 6.0, 8.0, 10.0, 20.0)
DWELL_S = 0.30
LEG_VEL_MMPS = float(hw.JB_TRAJ_LEG_VEL_LIMIT_MMPS)
LEG_ACC_MMPS2 = float(hw.JB_TRAJ_LEG_ACC_LIMIT_MMPS2)
LEG_JERK_MMPS3 = float(hw.JB_TRAJ_LEG_JERK_LIMIT_MMPS3)
HAND_ACC_RPS2 = float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2)

# ── R3's single-site grid (owner decision, plan § "R3", 2026-09-13) ─────────
# THROW(P1) from rest -> CATCH(P1) carrying the next same-site throw
# (dwell 0.30 s) -> ... -> CATCH -> REST, one ball, apex 0.9 m, site
# P1 = columns_sites(100)[0]. The box must be swept WIDER than the columns
# grid: flight commands spanning at least 0.75-0.95 s (apex 0.9 m -> 0.8570 s
# at the centre) and landing offsets out to +-40 mm -- the edges are set by
# what the real gate passes with margin, not by this grid's resolution.
SINGLE_SITE_FLIGHTS_S = (0.75, 0.80, 0.8570, 0.90, 0.95)
#: Same 2026-09-21 origin-densification as ``OFFSETS_MM``, applied to the
#: wider +-40 mm single-site span.
SINGLE_SITE_OFFSETS_MM = (-40.0, -30.0, -20.0, -10.0, -8.0, -6.0, -4.0, -2.0,
                          -1.0, -0.5, 0.0, 0.5, 1.0, 2.0, 4.0, 6.0, 8.0, 10.0,
                          20.0, 30.0, 40.0)
#: R3's session leg-jerk limit (plan § "R3": 150 000 mm/s^3 -- "the most the
#: machine has flown"), now IDENTICAL to ``LEG_JERK_MMPS3`` above since that
#: default itself reads the generated launch constant (2026-09-21) -- kept as
#: its own named constant for the R3 single-site call sites' documentation
#: value; leg vel/acc and hand acc are unchanged from R2.
SINGLE_SITE_LEG_JERK_MMPS3 = float(hw.JB_TRAJ_LEG_JERK_LIMIT_MMPS3)
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
# Each rung is a COMMANDED flight (R5, 2026-09-30 fix): _single_apex_boxes
# passes the apex's OWN flight_s(apex) as `pattern_flight_s` to the
# underlying sweep, so a rung's fraction moves only the carried throw, never
# the schedule's timing -- see `_single_apex_boxes`'s own docstring.
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

    ONE solve: ``sg.plan_segment`` (the production entry point, extended with
    the SETTLE tail per I-PLAN-7 -- gated and reported exactly as the executor
    gates it) gives both the admitted / margin verdict and, on
    ``Segment.release_state``, the post-release seed the CATCH chains from.
    Until 2026-09-30 this cell (and the three chained cells below) re-solved
    "the same window" raw for that seed, and the raw goals omitted the
    pre-release hold, the settle site and the hold knots -- so every chained
    cell since the hold landed (2026-09-28) was seeded from a launch the
    machine does not fly, and the hop cell crashed on a seed the production
    solve had accepted (`HAND_LIMIT_ACC` 3553 > 3500, run 1 of the R5 sweep).
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
    return True, 'OK', segment.release_state, takeoff


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
                        limits: TrajectoryLimits, geom, cfg,
                        pattern_flight_s: Optional[float] = None):
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

    ONE solve, as :func:`_throw_cell`: ``sg.plan_segment`` (the production
    entry point) gives the admitted / margin verdict and, on
    ``Segment.release_state``, the release-terminal seed the NEXT catch in the
    chain plans from (2026-09-30; the raw re-solve it replaced omitted the
    hold knots).

    **``pattern_flight_s`` vs ``flight_s`` (R5, 2026-09-30 fix).** ``flight_s``
    is the COMMANDED flight (the learner's ``u``, an apex mapped through
    ``sc.flight_s`` -- possibly a ladder rung around the pattern) and sets
    ONLY the carried throw's ballistic solve (``then_throw.flight_s``) -- the
    launch follows the command, exactly as ``executor._throw_terminal`` does
    (``sch.flight_s(u_apex)``). ``pattern_flight_s`` (default: ``flight_s``,
    so a plain non-ladder call is unchanged) is the SCHEDULE's own timing --
    ``t_land_s`` and ``then_throw.t_release_s`` -- and the incoming ball's
    arrival velocity, because ``schedule.compile_*`` places every CATCH at
    ``t_throw + flight_s(pattern.apex_m)`` regardless of what was commanded,
    and the executor predicts arrival from the DESIRED apex
    (``_predicted_landing``'s ``y_d[1]``), not the literal command -- a
    converged learner lands the ball as the pattern intends. Before this fix
    both roles used one ``flight_s``, so a ladder rung below the pattern
    apex wrongly SHORTENED the schedule's own timing along with the throw,
    and every apex band collapsed to a point at the pattern (see the module
    docstring / ``logbook/2026-09-30-skill-stack-r5-columns-bb-start.md``).
    """
    Tp = float(pattern_flight_s) if pattern_flight_s is not None else float(flight_s)
    landing_mm = site.catch_site_mm() + np.array(
        [float(offset_mm[0]), float(offset_mm[1]), 0.0])
    v_arrival = bal.arrival_velocity(takeoff_vel_mm_s, Tp)
    t_land = Tp
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
    # The next seed is the segment's OWN release state (`Segment.release_state`,
    # 2026-09-30) -- no second solve of "the same window" (see `_throw_cell`).
    return True, 'OK', segment.release_state, segment.takeoff_vel_mm_s


def _columns_catch_throw_cell(site, release_seed, takeoff_vel_mm_s,
                              flight_s: float, dwell_s: float,
                              offset_mm: Tuple[float, float],
                              limits: TrajectoryLimits, geom, cfg,
                              pattern_flight_s: Optional[float] = None):
    """The columns pattern's own CATCH-with-``then_throw`` (R5, D4,
    2026-09-30) -- the segment ``schedule.compile_columns`` /
    ``_fold_catch_throw_pairs`` actually dispatches for every catch after the
    first: the platform, coming from a release at the OTHER columns site
    (``release_seed`` / ``takeoff_vel_mm_s``, a THROW's release state, NOT a
    same-site catch's), transits ``tau = sc.transit_s(pattern_flight_s,
    dwell_s)`` and touches down HERE (``site``), then dwells and releases
    again HERE at ``tau + dwell_s`` -- one STEADY window, the same shape
    :func:`_chained_catch_cell` gates for a SAME-site chain, but seeded across
    the two physical sites rather than from a rest state or a same-site
    release.

    Replaces the old transit-baked :func:`_throw_cell` cell this module's
    columns branch used to gate (rest at ``from_site`` -> release AT ``site``
    inside one ``dwell_s`` window) -- the OLD cell was never flown: the real
    columns pattern's first skill is a launch from rest at ITS OWN site over
    ``SINGLE_SITE_LAUNCH_S``, and the platform's cross-site relocation is
    absorbed into the CATCH's own window (this function), not the THROW's
    (see the module docstring's "Why" #1, and ``scratchpad/
    probe_columns_cells.py``'s Probe A, 2026-09-30, whose
    ``columns_catch_throw_cell`` this is a generalised copy of -- offset-
    parameterised, folded into this module rather than left a one-off probe).

    **``pattern_flight_s`` vs ``flight_s`` (R5, 2026-09-30 fix, same split as
    :func:`_chained_catch_cell` -- see its docstring for the full "Why").**
    ``flight_s`` (the commanded apex's flight, a possible ladder rung) sets
    ONLY the carried throw's ballistic solve; ``pattern_flight_s`` (default:
    ``flight_s``) sets ``tau``/``t_release`` (``schedule.compile_columns``'s
    own timing, which does not move with the command) and the incoming ball's
    arrival velocity (the executor predicts from the DESIRED apex, not the
    literal command). Before this fix a ladder rung below the pattern apex
    wrongly shortened ``tau`` along with the throw, collapsing every apex
    band to a point at the pattern.

    ONE solve, as :func:`_throw_cell`: ``sg.plan_segment`` gives the verdict
    and, on ``Segment.release_state``, the seed the NEXT leg of the chain
    plans from.

    Returns ``(ok, code, next_release_seed, next_takeoff_vel_mm_s)`` -- the
    last two ``None`` on a refusal.
    """
    Tp = float(pattern_flight_s) if pattern_flight_s is not None else float(flight_s)
    tau = sc.transit_s(Tp, dwell_s)
    t_release = tau + float(dwell_s)
    landing_mm = site.catch_site_mm() + np.array(
        [float(offset_mm[0]), float(offset_mm[1]), 0.0])
    v_arrival = bal.arrival_velocity(takeoff_vel_mm_s, Tp)
    then_throw = sg.ThrowAfterCatch(t_release_s=t_release, site_mm=site.throw_site_mm(),
                                    target_mm=landing_mm, flight_s=float(flight_s))
    terminal = sg.CatchTerminal(landing_mm=landing_mm, landing_vel_mm_s=v_arrival,
                                t_land_s=tau, rest_site_mm=site.rest_site_mm(),
                                then_throw=then_throw)
    try:
        segment = sg.plan_segment(sg.CATCH, release_seed, terminal, cfg, limits, geom)
    except uc.CycleInfeasible as exc:
        return False, exc.code, None, None
    if not _within_margin(segment.meta.report, limits):
        return False, 'MARGIN', None, None
    # The next seed is the segment's OWN release state (`Segment.release_state`,
    # 2026-09-30) -- no second solve of "the same window" (see `_throw_cell`).
    return True, 'OK', segment.release_state, segment.takeoff_vel_mm_s


def _hop_throw_cell(from_site, to_site, flight_s: float, launch_s: float,
                    limits: TrajectoryLimits, geom, cfg,
                    offset_mm: Tuple[float, float] = (0.0, 0.0)):
    """R4's cross-site HOP launch: THROW from rest AT ``from_site``, released
    at ``from_site.throw_site_mm()``, targeting ``to_site.throw_site_mm() +
    offset_mm`` -- unlike the columns cell's :func:`_throw_cell` (whose
    release site IS ``to_site``, the pre-throw transit baked into the
    segment), a hop's release and landing sites are genuinely two different
    physical locations, so the ballistic terminal's OWN site is
    ``from_site``'s.  ONE solve, as :func:`_throw_cell`; the release seed is
    ``Segment.release_state``.  Returns ``(ok, code, release_seed,
    takeoff_vel_mm_s)`` -- the last two ``None`` on a refusal.
    """
    seed = _rest_state(from_site.rest_site_mm())
    target_mm = to_site.throw_site_mm() + np.array(
        [float(offset_mm[0]), float(offset_mm[1]), 0.0])
    terminal = sg.ThrowTerminal(site_mm=from_site.throw_site_mm(),
                                target_mm=target_mm, flight_s=flight_s,
                                t_release_s=launch_s)
    try:
        segment = sg.plan_segment(sg.THROW, seed, terminal, cfg, limits, geom)
    except uc.CycleInfeasible as exc:
        return False, exc.code, None, None
    if not _within_margin(segment.meta.report, limits):
        return False, 'MARGIN', None, None
    return True, 'OK', segment.release_state, segment.takeoff_vel_mm_s


def _hop_catch_cell(from_site, to_site, release_seed, takeoff_vel_mm_s,
                    flight_s: float, dwell_s: float,
                    offset_mm: Tuple[float, float],
                    limits: TrajectoryLimits, geom, cfg,
                    pattern_flight_s: Optional[float] = None):
    """R4's cross-site HOP steady state: CATCH at ``to_site`` (arriving from
    ``from_site``'s throw) carrying ``then_throw`` back TOWARD ``from_site``,
    then one more :func:`_catch_cell` at ``from_site`` for that carried
    throw -- the command ``u`` (the landing offset) applies to BOTH legs,
    exactly as :func:`_chained_catch_cell` applies it to a same-site chain's
    steady state.  Returns ``(ok, code)``.

    **``pattern_flight_s`` vs ``flight_s`` (R5, 2026-09-30 fix, same split as
    :func:`_chained_catch_cell`).** ``flight_s`` (commanded, a possible ladder
    rung) sets ONLY the two carried throws' ballistic solves (``then_throw.
    flight_s`` here, and the ``flight_s`` argument :func:`_catch_cell` uses
    below to size ITS carried throw's arrival). ``pattern_flight_s`` (default:
    ``flight_s``) sets every timing instant (``t_land_s``, ``t_release``) and
    every arrival velocity (both this catch's and the final :func:`_catch_cell`'s)
    -- the schedule's own beat, which does not move with the command.
    """
    Tp = float(pattern_flight_s) if pattern_flight_s is not None else float(flight_s)
    landing_mm = to_site.catch_site_mm() + np.array(
        [float(offset_mm[0]), float(offset_mm[1]), 0.0])
    v_arrival = bal.arrival_velocity(takeoff_vel_mm_s, Tp)
    t_release = Tp + float(dwell_s)
    target_back_mm = from_site.throw_site_mm() + np.array(
        [float(offset_mm[0]), float(offset_mm[1]), 0.0])
    then_throw = sg.ThrowAfterCatch(t_release_s=t_release,
                                    site_mm=to_site.throw_site_mm(),
                                    target_mm=target_back_mm, flight_s=float(flight_s))
    terminal = sg.CatchTerminal(landing_mm=landing_mm, landing_vel_mm_s=v_arrival,
                                t_land_s=Tp,
                                rest_site_mm=to_site.rest_site_mm(),
                                then_throw=then_throw)
    try:
        segment = sg.plan_segment(sg.CATCH, release_seed, terminal, cfg, limits, geom)
    except uc.CycleInfeasible as exc:
        return False, exc.code
    if not _within_margin(segment.meta.report, limits):
        return False, 'MARGIN'
    # `Segment.release_state` (2026-09-30): the raw re-solve this replaced
    # omitted the hold knots and crashed run 1 of the R5 sweep on a seed the
    # production solve had accepted (`HAND_LIMIT_ACC` 3553 > 3500).
    next_release_seed = segment.release_state
    next_takeoff = segment.takeoff_vel_mm_s
    ok2, code2 = _catch_cell(from_site, next_release_seed, next_takeoff,
                             Tp, Tp, offset_mm,
                             limits, geom, cfg)
    if not ok2:
        return False, 'CHAIN_CATCH:%s' % code2
    return True, 'OK'


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
          pre_release_hold_s: Optional[float] = None,
          pattern_flight_s: Optional[float] = None,
          log=print) -> Tuple[List['ab.AdmissibleBox'], List[Dict]]:
    """Run the grid; returns ``(boxes, rows)`` -- ``rows`` for the table.

    **``pattern_flight_s`` -- the schedule's own timing, separated from the
    commanded flight (R5, 2026-09-30 fix; see ``_chained_catch_cell`` /
    ``_columns_catch_throw_cell``'s docstrings for the full "Why").** Every
    flight ``T`` in ``flights_s``/``apexes_m`` is the COMMANDED flight (the
    learner's ``u``) and always sets the carried throw's ballistic solve.
    ``pattern_flight_s`` (default ``None``) sets the SCHEDULE timing (tau,
    t_land, t_release) and the incoming ball's arrival velocity for EVERY row
    of this call: ``None`` means "each row's own ``T`` is also its pattern" --
    the old, still-correct behaviour for a plain multi-apex band sweep (a
    single T IS the pattern there; ``main()``'s default columns/single-site
    calls never pass this). A caller sweeping a LADDER of commanded flights
    around one fixed pattern apex (:func:`_single_apex_boxes`) passes ONE
    constant ``pattern_flight_s`` for the whole call, so every ladder rung is
    judged against the SAME schedule timing -- a command below the pattern
    apex no longer shortens the transit/dwell window it is judged against.

    **The cross-site (``'columns'``) branch is gated on the REAL segments the
    pattern flies (R5, D4, 2026-09-30).** Until 2026-09-29 this branch gated a
    single ``_throw_cell(from_site, to_site, T, dwell_s)`` -- a THROW seeded at
    ``from_site``'s rest that releases AT ``to_site`` inside one ``dwell_s``
    window, the platform's whole cross-site relocation baked into the THROW's
    own LAUNCH -- which ``schedule.compile_columns`` never flies (see the
    module docstring's "Why" #1) and which the 100 ms pre-release hold made
    fully EMPTY (0 -> OK, 0.05 -> MARGIN, 0.1 -> INFEASIBLE at the owner
    operating point, ``.scratch/r4-throw-precision/map.md`` § Out of scope).
    The branch now gates the SAME three segment shapes the real pattern
    dispatches: a launch THROW from rest AT ``to_site`` (:func:`_throw_cell`,
    the pattern's own cold start, either columns site per R5's D1 fused
    start), the cross-site CATCH-with-``then_throw``
    (:func:`_columns_catch_throw_cell` -- the segment ``_fold_catch_throw_pairs``
    actually builds: seeded from a release AT the OTHER site, transits tau,
    touches down and releases again HERE), and one more standalone
    :func:`_catch_cell` at ``from_site`` on what that throw carries. Real
    segments pass the default hold at every setting measured (2026-09-30
    probe) but sit near the 90% sweep margin -- the box is swept from them,
    not from a defect-shaped proxy.

    ``pre_release_hold_s`` (None = the planner default,
    ``unified_cycle.PRE_RELEASE_HOLD_S``) overrides the segment config's
    pre-release platform hold -- a general test-only escape hatch, no longer
    needed to make the columns cell non-empty (that was the OLD transit-baked
    cell's defect, fixed above).

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
    cell's standalone LANDING catch. A CROSS-site pair (columns) is judged by
    :func:`_columns_catch_throw_cell` instead -- the same STEADY shape, seeded
    across the two physical sites rather than from one site's own rest/release.
    The ``NO_TRANSIT`` refusal applies only cross-site -- a same-site self-toss
    returns after the FULL flight time, not a shortened cross-site transit.
    """
    geom = StewartGeometry()
    limits = _limits(leg_vel, leg_acc, leg_jerk, hand_acc)
    cfg = sg.SegmentConfig()
    if pre_release_hold_s is not None:
        cfg = dataclasses.replace(cfg, pre_release_hold_s=float(pre_release_hold_s))
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
            # Tp is the SCHEDULE's own timing flight (R5, 2026-09-30 fix):
            # `pattern_flight_s` when the caller fixed one for the whole call
            # (a `_single_apex_boxes` ladder), else T itself (a plain sweep,
            # where each row IS its own pattern -- unchanged behaviour).
            Tp = float(pattern_flight_s) if pattern_flight_s is not None else T
            # The ONE per-flight seed every offset in this row shares: a
            # launch from rest AT from_site, zero offset, AT THE PATTERN'S
            # flight Tp (R5, D4, 2026-09-30 -- both branches now share this
            # one seeding call; it used to differ, dwell_s for cross-site,
            # which baked the platform's cross-site TRANSIT into the THROW's
            # own window -- a segment shape the real columns pattern never
            # flies, see the module docstring's "Why" #1). This is the
            # "pattern-apex launch cell seeded once per pair": its release
            # state and takeoff velocity feed the STEADY chain below as the
            # INCOMING ball a converged learner actually sees, so it is
            # planned at Tp, never at a ladder rung's commanded T. For a
            # same-site pair this IS the box's own site (unchanged from
            # before). For a cross-site (columns) pair it is the release
            # :func:`_columns_catch_throw_cell` below transits FROM --
            # ``schedule.compile_columns``'s launch THROW 0, whose folded
            # catch-with-throw is the segment the columns pattern actually
            # dispatches at every later catch.
            ok, code, release_seed, takeoff = _throw_cell(
                from_site, from_site, Tp, launch_s, limits, geom, cfg)
            row = dict(site_pair=(from_site.name, to_site.name),
                      flight_s=round(T, 4), offset_mm=None, ok=ok,
                      code=('THROW:%s' % code) if not ok else 'THROW:OK',
                      wall_s=round(time.perf_counter() - t0, 3))
            rows.append(row)
            log(row)
            tau = Tp if same_site else sc.transit_s(Tp, dwell_s)
            if not same_site and ok and not tau > 2.0 * float(hw.JB_TRAJ_KNOT_DT_S):
                ok, code = False, 'NO_TRANSIT'
            if not ok:
                for dx in offsets_mm:
                    for dy in offsets_mm:
                        pass_grid[(T, dx, dy)] = False
                continue
            for dx in offsets_mm:
                for dy in offsets_mm:
                    t1 = time.perf_counter()
                    # The box is the intersection of three gates -- the same
                    # shape for same-site and cross-site pairs alike (R5, D4,
                    # 2026-09-30): the launch THROW from rest AT to_site under
                    # THIS offset (a cold-start attempt's own command --
                    # R3-h2's reasoning, which applies at EITHER columns site
                    # since R5's D1 fused start can cold-launch from either),
                    # the catch (-with-throw) that actually carries the
                    # command through the dwell, and one more catch on what
                    # it carries -- no single gate covers all three segment
                    # shapes.
                    # The launch THROW from rest AT to_site carries the
                    # COMMANDED flight T (R5, D4, 2026-09-30, R3-h2's
                    # reasoning unchanged: a cold-start attempt's own launch
                    # IS the learner's command).
                    lok, lcode, _seed, _takeoff = _throw_cell(
                        to_site, to_site, T, launch_s, limits, geom, cfg,
                        offset_mm=(dx, dy))
                    if same_site:
                        cok, ccode, next_seed, next_takeoff = _chained_catch_cell(
                            to_site, release_seed, takeoff, T, dwell_s, (dx, dy),
                            limits, geom, cfg, pattern_flight_s=Tp)
                    else:
                        cok, ccode, next_seed, next_takeoff = _columns_catch_throw_cell(
                            to_site, release_seed, takeoff, T, dwell_s, (dx, dy),
                            limits, geom, cfg, pattern_flight_s=Tp)
                    if cok:
                        if same_site:
                            # One more STANDALONE catch on the same site (the
                            # last catch of an attempt, R3's own shape) -- at
                            # the PATTERN's timing/arrival (Tp), same as tau.
                            fok, fcode = _catch_cell(
                                to_site, next_seed, next_takeoff, Tp, tau,
                                (dx, dy), limits, geom, cfg)
                        else:
                            # The reverse leg: the carried throw is a SELF-toss
                            # at to_site (compile_columns's own target, "each
                            # throw's target is its OWN site"), so the
                            # platform's next physical event is another
                            # cross-site catch-with-throw back at from_site,
                            # not a standalone catch -- measured 2026-09-30
                            # (probe): a standalone catch at from_site with
                            # tau timing is the WRONG shape and refuses
                            # LIMIT_JERK at every offset tried, including the
                            # identity command.
                            fok, fcode, _s2, _t2 = _columns_catch_throw_cell(
                                from_site, next_seed, next_takeoff, T, dwell_s,
                                (dx, dy), limits, geom, cfg, pattern_flight_s=Tp)
                        if not fok:
                            cok, ccode = False, 'CHAIN_CATCH:%s' % fcode
                    if not lok:
                        cok, ccode = False, 'LAUNCH:%s' % lcode
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
            apex_bounds = (float('nan'), float('nan'))
        else:
            xlo, xhi, ylo, yhi = rect
            landing_xy = ((xlo / 1000.0, xhi / 1000.0), (ylo / 1000.0, yhi / 1000.0))
            # The GRID is swept in flight time (the planner's own input); the
            # BOX bounds the command, which is an apex since 2026-09-18. The
            # conversion is `schedule.flight_s`'s exact inverse, so the band
            # admits the same set of throws either way.
            apex_bounds = (sc.apex_m(flight_band[0]), sc.apex_m(flight_band[1]))
        # R4 (2026-09-23): the box's KEY, not merely its provenance. A
        # same-site pair is a 'self_toss' box, keyed (site, site) -- unchanged
        # geometry. A cross-site pair here is the COLUMNS cell, whose THROW
        # (`_throw_cell`) is seeded at `from_site`'s rest but releases AT
        # `to_site` (the pre-throw transit is baked into the segment, see the
        # module docstring's "Why" #1) -- so the box the EXECUTOR looks up by
        # (release_site.name, target_site.name) is keyed (to.name, to.name),
        # not (from.name, to.name); the xy stamps are `to_site`'s own, both
        # release and target (the two coincide for this pattern).
        pattern = 'self_toss' if same_site else 'columns'
        box_pair = (to_site.name, to_site.name)
        box_xy = (float(to_site.cup_mm[0]), float(to_site.cup_mm[1]))
        boxes.append(ab.AdmissibleBox(
            site_pair=box_pair, apex_band_m=apex_band,
            landing_xy_m=landing_xy, apex_m=apex_bounds, pattern=pattern,
            release_site_xy_mm=box_xy, target_site_xy_mm=box_xy,
            limits=limits_dict, gate_hash=ghash, swept_at=swept_at,
            dwell_s=float(dwell_s)))
    return boxes, rows


def hop_sweep(*, apexes_m: Sequence[float] = HOP_APEXES_M,
             offsets_mm: Sequence[float] = OFFSETS_MM, dwell_s: float = DWELL_S,
             separation_mm: float = HOP_SEPARATION_MM, leg_vel: float = LEG_VEL_MMPS,
             leg_acc: float = LEG_ACC_MMPS2, leg_jerk: float = LEG_JERK_MMPS3,
             hand_acc: float = HAND_ACC_RPS2, launch_s: float = SINGLE_SITE_LAUNCH_S,
             center_apex_m: Optional[float] = None,
             pattern_flight_s: Optional[float] = None, log=print
             ) -> Tuple[List['ab.AdmissibleBox'], List[Dict]]:
    """R4's cross-site HOP sweep (release at one columns site, landing at the
    other) -- plan "R4", ``brief_U0_admissible.md`` § B.

    Mirrors :func:`sweep`'s box-building (the same :func:`_flight_band_and_rect`
    grid-to-rectangle reduction, and the columns apex band [0.85, 0.95] m via
    :func:`~jugglebot.motion.skills.schedule.flight_s`) but drives the hop's
    OWN three-segment chain (:func:`_hop_throw_cell` / :func:`_hop_catch_cell`)
    rather than :func:`sweep`'s columns/self-toss cells -- a hop's steady state
    is THROW(from) -> CATCH(to)+then_throw(to -> from) -> CATCH(from), a shape
    neither existing cell plans (the columns cell's THROW releases AT the
    destination site; a hop's releases at the ORIGIN).  Both directions
    (``columns_sites``'s two orderings) are swept, each its own box: pattern
    'hop', ``site_pair=(from.name, to.name)`` (genuinely two different sites,
    unlike 'columns'/'self_toss' -- see ``admissible.AdmissibleBox``'s
    docstring), xy stamped from the two sites.

    ``pattern_flight_s`` is :func:`sweep`'s own escape hatch, mirrored here
    (R5, 2026-09-30 fix): ``None`` (default) means each row's own flight is
    its own pattern (a plain multi-apex band sweep, unchanged behaviour);
    :func:`_single_apex_boxes` passes one constant value for a commanded-flight
    ladder around a fixed pattern apex, so the schedule timing (``t_land``,
    ``t_release``) and the incoming ball's arrival velocity stay pinned to the
    pattern while only the carried throws follow the ladder rung. See
    ``_hop_catch_cell``'s docstring for the per-cell split.
    """
    geom = StewartGeometry()
    limits = _limits(leg_vel, leg_acc, leg_jerk, hand_acc)
    cfg = sg.SegmentConfig()
    site0, site1 = st.columns_sites(separation_mm)
    flights = [sc.flight_s(a) for a in apexes_m]
    apex_band = (min(float(a) for a in apexes_m), max(float(a) for a in apexes_m))
    center_apex = (float(center_apex_m) if center_apex_m is not None
                  else sorted(float(a) for a in apexes_m)[len(apexes_m) // 2])
    center_flight = sc.flight_s(center_apex)
    ghash = ab.gate_hash()
    swept_at = _dt.date.today().isoformat()
    limits_dict = dict(leg_vel_mmps=float(leg_vel), leg_acc_mmps2=float(leg_acc),
                       leg_jerk_mmps3=float(leg_jerk), hand_acc_rps2=float(hand_acc))

    boxes: List['ab.AdmissibleBox'] = []
    rows: List[Dict] = []
    for from_site, to_site in ((site0, site1), (site1, site0)):
        pass_grid = {}   # (flight, dx, dy) -> bool
        for T in flights:
            t0 = time.perf_counter()
            # Tp is the pattern's own flight (R5, 2026-09-30 fix) -- see
            # `sweep`'s matching comment; the seeding launch below is planned
            # at Tp, not at a ladder rung's commanded T.
            Tp = float(pattern_flight_s) if pattern_flight_s is not None else T
            ok, code, release_seed, takeoff = _hop_throw_cell(
                from_site, to_site, Tp, launch_s, limits, geom, cfg)
            row = dict(site_pair=(from_site.name, to_site.name), flight_s=round(T, 4),
                      offset_mm=None, ok=ok,
                      code=('THROW:%s' % code) if not ok else 'THROW:OK',
                      wall_s=round(time.perf_counter() - t0, 3))
            rows.append(row)
            log(row)
            if not ok:
                for dx in offsets_mm:
                    for dy in offsets_mm:
                        pass_grid[(T, dx, dy)] = False
                continue
            for dx in offsets_mm:
                for dy in offsets_mm:
                    t1 = time.perf_counter()
                    # The box is the intersection of two gates, same reason
                    # as `sweep`'s same-site branch: the launch THROW from
                    # rest under THIS offset carries the COMMANDED flight T
                    # (a cold-start attempt's own command), and the chained
                    # CATCH+then_throw+CATCH is judged at the PATTERN's
                    # timing/arrival (Tp) from the pattern-flight seeding
                    # launch above.
                    lok, lcode, _seed, _takeoff = _hop_throw_cell(
                        from_site, to_site, T, launch_s, limits, geom, cfg,
                        offset_mm=(dx, dy))
                    if lok:
                        cok, ccode = _hop_catch_cell(
                            from_site, to_site, release_seed, takeoff, T, dwell_s,
                            (dx, dy), limits, geom, cfg, pattern_flight_s=Tp)
                    else:
                        cok, ccode = False, 'LAUNCH:%s' % lcode
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
            apex_bounds = (float('nan'), float('nan'))
        else:
            xlo, xhi, ylo, yhi = rect
            landing_xy = ((xlo / 1000.0, xhi / 1000.0), (ylo / 1000.0, yhi / 1000.0))
            apex_bounds = (sc.apex_m(flight_band[0]), sc.apex_m(flight_band[1]))
        boxes.append(ab.AdmissibleBox(
            site_pair=(from_site.name, to_site.name), apex_band_m=apex_band,
            landing_xy_m=landing_xy, apex_m=apex_bounds, pattern='hop',
            release_site_xy_mm=(float(from_site.cup_mm[0]), float(from_site.cup_mm[1])),
            target_site_xy_mm=(float(to_site.cup_mm[0]), float(to_site.cup_mm[1])),
            limits=limits_dict, gate_hash=ghash, swept_at=swept_at,
            dwell_s=float(dwell_s)))
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
                       pattern: str = 'self_toss', site: Optional['st.Site'] = None,
                       log=print) -> Tuple[List['ab.AdmissibleBox'], List[Dict]]:
    """One box PER apex in ``apexes_m``, own flight grid and own
    ``apex_band_m = (apex - h, apex + h)`` each (R3 apex-ladder sweep,
    2026-09-14; generalised R5, D4, 2026-09-30, to the ``'columns'`` and
    ``'hop'`` patterns -- D4's per-apex columns/hop grids reuse this SAME
    machinery rather than a second copy of it).

    Each apex's flight grid is ``sc.flight_s(apex) * (1 + f)`` for ``f`` in
    ``flight_frac``, centred on ``sc.flight_s(apex)`` -- NOT the grid-derived
    band :func:`sweep` would otherwise compute (neighbouring apexes'
    grid-derived bands overlap; the whole reason this CLI records the
    requested apex directly rather than inferring a band from the grid). The
    returned box's ``apex_band_m`` is overridden via ``dataclasses.replace``
    -- its ``landing_xy_m`` / ``dwell_s`` / provenance are exactly what the
    underlying sweep measured, untouched.

    ``pattern`` picks which underlying sweep drives each apex's grid:
    ``'self_toss'`` (default, R3) -- :func:`sweep` at ``site_pairs=[(site,
    site)]`` (the STEADY catch-with-``then_throw`` chain plus the
    launch-from-rest throw, R3-c / R3-h2); ``site`` is required.
    ``'columns'`` -- :func:`sweep` at ``site_pairs=None`` (the default
    two-direction columns pairs at ``separation_mm``), one box per DIRECTION
    per apex. ``'hop'`` -- :func:`hop_sweep` at ``separation_mm`` (also one
    box per direction per apex); each ``flight_frac`` offset is converted
    back to an apex (``sc.apex_m``) since :func:`hop_sweep` grids in apex, not
    flight time -- the exact inverse map :func:`sweep` itself uses to record
    ``apex_m`` on a flight-time grid (see :func:`sweep`'s own docstring).

    **``flight_frac`` is a COMMANDED-flight ladder, not a timing ladder (R5,
    2026-09-30 fix).** ``centre`` (``sc.flight_s(apex)``) is this apex's
    PATTERN flight -- the schedule's own timing, which does not move with the
    command -- and is passed to the underlying sweep as ``pattern_flight_s``
    on every call below, so every rung of ``flights`` (the commanded ladder)
    is judged against ONE fixed schedule timing instead of shortening it along
    with the command. Before this fix each rung supplied its OWN timing too,
    so a commanded apex below the pattern's shortened the transit/dwell window
    it was judged against and every resulting band collapsed to a point at the
    pattern (``logbook/2026-09-30-skill-stack-r5-columns-bb-start.md``).
    """
    boxes: List['ab.AdmissibleBox'] = []
    rows: List[Dict] = []
    for a in apexes_m:
        centre = sc.flight_s(float(a))
        flights = [centre * (1.0 + float(f)) for f in flight_frac]
        if pattern == 'self_toss':
            if site is None:
                raise ValueError("pattern='self_toss' requires `site`")
            a_boxes, a_rows = sweep(
                flights_s=flights, offsets_mm=offsets_mm, dwell_s=dwell_s,
                separation_mm=separation_mm, leg_vel=leg_vel, leg_acc=leg_acc,
                leg_jerk=leg_jerk, hand_acc=hand_acc, center_flight_s=centre,
                site_pairs=[(site, site)], pattern_flight_s=centre, log=log)
        elif pattern == 'columns':
            a_boxes, a_rows = sweep(
                flights_s=flights, offsets_mm=offsets_mm, dwell_s=dwell_s,
                separation_mm=separation_mm, leg_vel=leg_vel, leg_acc=leg_acc,
                leg_jerk=leg_jerk, hand_acc=hand_acc, center_flight_s=centre,
                site_pairs=None, pattern_flight_s=centre, log=log)
        elif pattern == 'hop':
            hop_apexes = [sc.apex_m(t) for t in flights]
            a_boxes, a_rows = hop_sweep(
                apexes_m=hop_apexes, offsets_mm=offsets_mm, dwell_s=dwell_s,
                separation_mm=separation_mm, leg_vel=leg_vel, leg_acc=leg_acc,
                leg_jerk=leg_jerk, hand_acc=hand_acc, center_apex_m=float(a),
                pattern_flight_s=centre, log=log)
        else:
            raise ValueError(
                "pattern must be one of 'self_toss'/'columns'/'hop', got %r"
                % (pattern,))
        h = float(halfwidth_m)
        for box in a_boxes:
            boxes.append(dataclasses.replace(
                box, apex_band_m=(float(a) - h, float(a) + h)))
        rows.extend(a_rows)
    return boxes, rows


def to_markdown(boxes: List['ab.AdmissibleBox']) -> str:
    out = ['| pattern | site pair | apex band m | landing xy box (mm) |'
          ' apex box (m) | limits (vel/acc/jerk mm, hand acc rev) |',
          '|---|---|---|---|---|---|']
    for box in boxes:
        if box.empty:
            xy_str, ap_str = 'EMPTY', 'EMPTY'
        else:
            (xlo, xhi), (ylo, yhi) = box.landing_xy_m
            xy_str = '[%.0f, %.0f] x [%.0f, %.0f]' % (
                xlo * 1000.0, xhi * 1000.0, ylo * 1000.0, yhi * 1000.0)
            ap_str = '%.4f-%.4f' % box.apex_m
        out.append('| %s | %s | %.2f-%.2f | %s | %s | %.0f/%.0f/%.0fk, %.0f |' % (
            box.pattern, box.site_pair, box.apex_band_m[0], box.apex_band_m[1],
            xy_str, ap_str, box.limits['leg_vel_mmps'], box.limits['leg_acc_mmps2'],
            box.limits['leg_jerk_mmps3'] / 1000.0, box.limits['hand_acc_rps2']))
    return '\n'.join(out)


def _xy(s: str) -> Tuple[float, float]:
    x, y = s.split(',')
    return (float(x), float(y))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', default=_DEFAULT_OUT)
    ap.add_argument('--quiet', action='store_true')
    ap.add_argument('--site-pairs', choices=('columns', 'single', 'hop', 'all'),
                    default='all',
                    help='which boxes to sweep: the two columns orderings, '
                         'the R3 single-site self-toss pair, the R4 '
                         'cross-site hop (both directions), or all of them '
                         '("both" was renamed "all" 2026-09-23 when hop was '
                         'added) -- one YAML needs ONE limits set (ab.dump '
                         'enforces it), so "all" sweeps every box under the '
                         'SAME --leg-*/--hand-acc')
    ap.add_argument('--apex', type=float, nargs='+', default=list(APEXES_M),
                    help='columns apex grid (m), mapped through sc.flight_s')
    ap.add_argument('--hop-apex', type=float, nargs='+', default=list(HOP_APEXES_M),
                    help='R5 hop apex grid (m), independent of --apex -- gains '
                         'a 0.80 m row the columns grid does not carry (D4, '
                         '2026-09-30)')
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
        help='sweep one box PER apex (m) instead of one box spanning a '
             'flight band -- an apex LADDER (e.g. 0.5 0.6 0.7 0.8 0.9), each '
             'rung its own box: flight grid = sc.flight_s(apex) * (1 + f) '
             'for f in --single-flight-frac, apex_band_m = (apex - h, apex '
             '+ h) for h = --single-apex-halfwidth. Applies to WHICHEVER of '
             '--site-pairs is selected (R5, D4, 2026-09-30 widened this from '
             'single-site-only to columns and hop too, via '
             '_single_apex_boxes\'s own pattern= dispatch -- one apex ladder '
             'over --apex/--hop-apex for columns/hop respectively, not the '
             'grid-derived apex_band_m sweep()/hop_sweep() would otherwise '
             'record). Refused at parse time if two requested apexes\' bands '
             'overlap. Mutually exclusive with --flights-s')
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
    ap.add_argument('--hop-separation-mm', type=float, default=HOP_SEPARATION_MM,
                    help='R4 hop-pair separation (mm); independent of '
                         '--separation-mm, which stays the columns cell\'s')
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
    if args.site_pairs in ('columns', 'all'):
        if args.single_apex is not None:
            cboxes, _rows = _single_apex_boxes(
                args.single_apex, flight_frac=args.single_flight_frac,
                halfwidth_m=args.single_apex_halfwidth,
                offsets_mm=args.offsets_mm, dwell_s=args.dwell_s,
                separation_mm=args.separation_mm, leg_vel=args.leg_vel,
                leg_acc=args.leg_acc, leg_jerk=args.leg_jerk,
                hand_acc=args.hand_acc, pattern='columns', log=log)
        else:
            cboxes, _rows = sweep(
                apexes_m=args.apex, offsets_mm=args.offsets_mm, dwell_s=args.dwell_s,
                separation_mm=args.separation_mm, leg_vel=args.leg_vel,
                leg_acc=args.leg_acc, leg_jerk=args.leg_jerk, hand_acc=args.hand_acc,
                log=log)
        boxes.extend(cboxes)
    if args.site_pairs in ('hop', 'all'):
        if args.single_apex is not None:
            hboxes, _rows = _single_apex_boxes(
                args.single_apex, flight_frac=args.single_flight_frac,
                halfwidth_m=args.single_apex_halfwidth,
                offsets_mm=args.offsets_mm, dwell_s=args.dwell_s,
                separation_mm=args.hop_separation_mm, leg_vel=args.leg_vel,
                leg_acc=args.leg_acc, leg_jerk=args.leg_jerk,
                hand_acc=args.hand_acc, pattern='hop', log=log)
        else:
            hboxes, _rows = hop_sweep(
                apexes_m=args.hop_apex, offsets_mm=args.offsets_mm, dwell_s=args.dwell_s,
                separation_mm=args.hop_separation_mm, leg_vel=args.leg_vel,
                leg_acc=args.leg_acc, leg_jerk=args.leg_jerk, hand_acc=args.hand_acc,
                log=log)
        boxes.extend(hboxes)
    if args.site_pairs in ('single', 'all'):
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
