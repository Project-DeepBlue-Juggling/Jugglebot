"""Reference catch / throw cells on the production planner, for probes and tests.

Each cell builds one segment the way the schedule compiler does
(``segments.plan_segment`` under a :class:`TrajectoryLimits`) and answers
whether it is planned with margin (``within_margin``: every peak leg
velocity / acceleration / jerk within ``admissible.MARGIN_FRAC`` of the
limit).  ``throw_cell`` and ``catch_cell*`` are the single-site chain,
``columns_catch_throw_cell`` the two-ball columns window, ``hop_*`` the
cross-site hop and ``folded_catch_cell`` / ``oval_window`` the folded oval
window.  ``rest_state`` and ``sitting_limits`` are re-exported from
``jugglebot.motion.skills.nominate`` (never copied).

The admissible-box sweep that first owned these (``tools/admissible_sweep.py``)
was retired at learner-generalisation P3c, 2026-11-06; the bodies below are
unchanged from it except for the dropped leading underscore.  Production code
must NOT import from ``tools/``: the runtime path is the online check
(``motion/skills/admissible.py``) and the nomination
(``motion/skills/nominate.py``).
"""
from __future__ import annotations

import os
import sys
from typing import Optional, Tuple

# Pin BLAS to ONE thread before numpy loads (as the retired sweep did): cell
# results are only bit-identical run to run with single-threaded BLAS.
for _blas_var in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_blas_var] = '1'

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
for _p in (_ROOT, os.path.join(_ROOT, 'ros_ws', 'src', 'jugglebot')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np                                                     # noqa: E402,F401

import jugglebot.hardware_config as hw                                 # noqa: E402,F401
from jugglebot.motion import unified_cycle as uc                       # noqa: E402
from jugglebot.motion.geometry import StewartGeometry                  # noqa: E402,F401
from jugglebot.motion.trajectory import ballistics_bc as bal           # noqa: E402,F401
from jugglebot.motion.trajectory import cup_cycle as cc                # noqa: E402,F401
from jugglebot.motion.trajectory import feasibility as fz              # noqa: E402
from jugglebot.motion.trajectory import throw_envelope                 # noqa: E402
from jugglebot.motion.trajectory.limits import TrajectoryLimits        # noqa: E402
from jugglebot.motion.skills import admissible as ab                   # noqa: E402
from jugglebot.motion.skills import schedule as sc                     # noqa: E402
from jugglebot.motion.skills import segments as sg                     # noqa: E402
from jugglebot.motion.skills import sites as st                        # noqa: E402,F401
from jugglebot.motion.skills import nominate as nm                     # noqa: E402

#: Offset grid (mm) the affine-check probe reads; was the sweep's grid.
OFFSETS_MM = (-30.0, -25.0, -20.0, -10.0, -8.0, -6.0, -4.0, -2.0, -1.0, -0.5,
             0.0, 0.5, 1.0, 2.0, 4.0, 6.0, 8.0, 10.0, 20.0, 25.0, 30.0)
#: :data:`OFFSETS_MM` capped at +-25 mm for the 250 mm HOP (see above).
HOP_OFFSETS_MM = tuple(o for o in OFFSETS_MM if abs(o) <= 25.0)

DWELL_S = 0.30
LEG_VEL_MMPS = float(hw.JB_TRAJ_LEG_VEL_LIMIT_MMPS)
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
#: first ``chained_catch_cell`` catches) plans over -- ``schedule.Pattern``'s
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

rest_state = nm.rest_state
sitting_limits = nm.sitting_limits


def within_margin(report, limits: TrajectoryLimits) -> bool:
    return (report.peak_leg_vel_mmps <= ab.MARGIN_FRAC * limits.leg_vel_mmps
            and report.peak_leg_acc_mmps2 <= ab.MARGIN_FRAC * limits.leg_acc_mmps2
            and report.peak_leg_jerk_mmps3 <= ab.MARGIN_FRAC * limits.leg_jerk_mmps3)


def throw_cell(from_site, to_site, flight_s: float, dwell_s: float,
                limits: TrajectoryLimits, geom, cfg,
                offset_mm: Tuple[float, float] = (0.0, 0.0)):
    """Plan + gate the ONE throw a (site pair, flight) cell shares across
    every landing offset. Returns ``(ok, code, release_seed, takeoff_vel_mm_s)``
    -- the last two are ``None`` on a refusal.

    ``offset_mm`` shifts the ballistic TARGET only (``site_mm``, the physical
    launch site, is unchanged) -- the same convention ``catch_cell`` /
    ``chained_catch_cell`` use for their ``landing_mm``. Defaults to (0, 0):
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
    seed = rest_state(from_site.rest_site_mm())
    target_mm = to_site.throw_site_mm() + np.array(
        [float(offset_mm[0]), float(offset_mm[1]), 0.0])
    terminal = sg.ThrowTerminal(site_mm=to_site.throw_site_mm(),
                                target_mm=target_mm,
                                flight_s=flight_s, t_release_s=dwell_s)
    try:
        segment = sg.plan_segment(sg.THROW, seed, terminal, cfg, limits, geom)
    except uc.CycleInfeasible as exc:
        return False, exc.code, None, None
    if not within_margin(segment.meta.report, limits):
        return False, 'MARGIN', None, None
    takeoff = segment.takeoff_vel_mm_s
    speed_mps = float(np.linalg.norm(takeoff)) / 1000.0
    verdict = throw_envelope.evaluate(flight_s, speed_mps)
    if not verdict.ok:
        return False, 'ENVELOPE:%s' % verdict.bound, None, None
    return True, 'OK', segment.release_state, takeoff


def catch_cell(to_site, release_seed, takeoff_vel_mm_s, flight_s: float,
                tau_s: float, offset_mm: Tuple[float, float],
                limits: TrajectoryLimits, geom, cfg):
    """Plan + gate the CATCH for one landing offset (mm, relative to
    ``to_site``). Returns ``(ok, code)``."""
    ok, code, _report = catch_cell_report(
        to_site, release_seed, takeoff_vel_mm_s, flight_s, tau_s, offset_mm,
        limits, geom, cfg)
    return ok, code


def catch_cell_report(to_site, release_seed, takeoff_vel_mm_s, flight_s: float,
                       tau_s: float, offset_mm: Tuple[float, float],
                       limits: TrajectoryLimits, geom, cfg):
    """:func:`catch_cell`'s one body: ``(ok, code, report)``, ``report`` the
    segment's ``FeasibilityReport`` (the refusing gate's report, if it carried
    one, on a ``CycleInfeasible``)."""
    landing_mm = to_site.catch_site_mm() + np.array(
        [float(offset_mm[0]), float(offset_mm[1]), 0.0])
    v_arrival = bal.arrival_velocity(takeoff_vel_mm_s, flight_s)
    terminal = sg.CatchTerminal(landing_mm=landing_mm, landing_vel_mm_s=v_arrival,
                                t_land_s=tau_s, rest_site_mm=to_site.rest_site_mm())
    try:
        segment = sg.plan_segment(sg.CATCH, release_seed, terminal, cfg, limits, geom)
    except uc.CycleInfeasible as exc:
        return False, exc.code, exc.report
    if not within_margin(segment.meta.report, limits):
        return False, 'MARGIN', segment.meta.report
    return True, 'OK', segment.meta.report


def chained_catch_cell(site, release_seed, takeoff_vel_mm_s, flight_s: float,
                        dwell_s: float, offset_mm: Tuple[float, float],
                        limits: TrajectoryLimits, geom, cfg,
                        pattern_flight_s: Optional[float] = None):
    """Plan + gate the CATCH-with-``then_throw`` (STEADY window) R3's chained
    single-site cycle actually flies, for one landing/launch offset (mm,
    relative to ``site``, applied to BOTH the incoming catch and the throw it
    carries -- the steady-state command).  Returns ``(ok, code,
    next_release_seed, next_takeoff_vel_mm_s)`` -- the last two ``None`` on a
    refusal.

    This is the cell the existing two-site ``catch_cell`` does NOT measure: a
    standalone LANDING (no ``then_throw``) is a different, and for this case
    infeasible, segment shape (plan § 2.2's R2 amendment / ``schedule.
    ThenThrow``'s docstring: the split LANDING-then-THROW form refuses at every
    cell of a 480-cell grid; the whole-window STEADY form is what
    ``segments.plan_segment`` actually builds for a same-site
    ``CatchTerminal.then_throw`` and what R3's schedule dispatches).

    ONE solve, as :func:`throw_cell`: ``sg.plan_segment`` (the production
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
    if not within_margin(segment.meta.report, limits):
        return False, 'MARGIN', None, None
    # The next seed is the segment's OWN release state (`Segment.release_state`,
    # 2026-09-30) -- no second solve of "the same window" (see `throw_cell`).
    return True, 'OK', segment.release_state, segment.takeoff_vel_mm_s


def columns_catch_throw_cell(site, release_seed, takeoff_vel_mm_s,
                              flight_s: float, dwell_s: float,
                              offset_mm: Tuple[float, float],
                              limits: TrajectoryLimits, geom, cfg,
                              pattern_flight_s: Optional[float] = None,
                              transit_s_: Optional[float] = None,
                              incoming_takeoff_vel_mm_s=None):
    """The columns pattern's own CATCH-with-``then_throw`` (R5, D4,
    2026-09-30) -- the segment ``schedule.compile_columns`` /
    ``_fold_catch_throw_pairs`` actually dispatches for every catch after the
    first: the platform, coming from a release at the OTHER columns site
    (``release_seed`` / ``takeoff_vel_mm_s``, a THROW's release state, NOT a
    same-site catch's), transits ``tau = sc.transit_s(pattern_flight_s,
    dwell_s)`` and touches down HERE (``site``), then dwells and releases
    again HERE at ``tau + dwell_s`` -- one STEADY window, the same shape
    :func:`chained_catch_cell` gates for a SAME-site chain, but seeded across
    the two physical sites rather than from a rest state or a same-site
    release.

    Replaces the old transit-baked :func:`throw_cell` cell this module's
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
    :func:`chained_catch_cell` -- see its docstring for the full "Why").**
    ``flight_s`` (the commanded apex's flight, a possible ladder rung) sets
    ONLY the carried throw's ballistic solve; ``pattern_flight_s`` (default:
    ``flight_s``) sets ``tau``/``t_release`` (``schedule.compile_columns``'s
    own timing, which does not move with the command) and the incoming ball's
    arrival velocity (the executor predicts from the DESIRED apex, not the
    literal command). Before this fix a ladder rung below the pattern apex
    wrongly shortened ``tau`` along with the throw, collapsing every apex
    band to a point at the pattern.

    ONE solve, as :func:`throw_cell`: ``sg.plan_segment`` gives the verdict
    and, on ``Segment.release_state``, the seed the NEXT leg of the chain
    plans from.

    **High-low Columns (JP-9, :func:`highlow_sweep`).** ``transit_s_`` and
    ``incoming_takeoff_vel_mm_s`` (both default ``None`` = the plain-Columns
    cell above, bit for bit) separate what equal Apexes let one number stand
    for: the transit is THIS ball's (``schedule.columns_ball_timing``, shorter
    for the low ball), ``dwell_s`` is this ball's own Dwell, and the incoming
    ball is THIS Site's ball (its own self-throw at ``pattern_flight_s``), not
    the seed's -- the seed is the OTHER ball's release at the other Site.

    Returns ``(ok, code, next_release_seed, next_takeoff_vel_mm_s)`` -- the
    last two ``None`` on a refusal.
    """
    Tp = float(pattern_flight_s) if pattern_flight_s is not None else float(flight_s)
    tau = (sc.transit_s(Tp, dwell_s) if transit_s_ is None
           else float(transit_s_))
    t_release = tau + float(dwell_s)
    landing_mm = site.catch_site_mm() + np.array(
        [float(offset_mm[0]), float(offset_mm[1]), 0.0])
    v_arrival = bal.arrival_velocity(
        takeoff_vel_mm_s if incoming_takeoff_vel_mm_s is None
        else incoming_takeoff_vel_mm_s, Tp)
    then_throw = sg.ThrowAfterCatch(t_release_s=t_release, site_mm=site.throw_site_mm(),
                                    target_mm=landing_mm, flight_s=float(flight_s))
    terminal = sg.CatchTerminal(landing_mm=landing_mm, landing_vel_mm_s=v_arrival,
                                t_land_s=tau, rest_site_mm=site.rest_site_mm(),
                                then_throw=then_throw)
    try:
        segment = sg.plan_segment(sg.CATCH, release_seed, terminal, cfg, limits, geom)
    except uc.CycleInfeasible as exc:
        return False, exc.code, None, None
    if not within_margin(segment.meta.report, limits):
        return False, 'MARGIN', None, None
    # The next seed is the segment's OWN release state (`Segment.release_state`,
    # 2026-09-30) -- no second solve of "the same window" (see `throw_cell`).
    return True, 'OK', segment.release_state, segment.takeoff_vel_mm_s


def hop_throw_cell(from_site, to_site, flight_s: float, launch_s: float,
                    limits: TrajectoryLimits, geom, cfg,
                    offset_mm: Tuple[float, float] = (0.0, 0.0)):
    """R4's cross-site HOP launch: THROW from rest AT ``from_site``, released
    at ``from_site.throw_site_mm()``, targeting ``to_site.throw_site_mm() +
    offset_mm`` -- unlike the columns cell's :func:`throw_cell` (whose
    release site IS ``to_site``, the pre-throw transit baked into the
    segment), a hop's release and landing sites are genuinely two different
    physical locations, so the ballistic terminal's OWN site is
    ``from_site``'s.  ONE solve, as :func:`throw_cell`; the release seed is
    ``Segment.release_state``.  Returns ``(ok, code, release_seed,
    takeoff_vel_mm_s)`` -- the last two ``None`` on a refusal.
    """
    seed = rest_state(from_site.rest_site_mm())
    target_mm = to_site.throw_site_mm() + np.array(
        [float(offset_mm[0]), float(offset_mm[1]), 0.0])
    terminal = sg.ThrowTerminal(site_mm=from_site.throw_site_mm(),
                                target_mm=target_mm, flight_s=flight_s,
                                t_release_s=launch_s)
    try:
        segment = sg.plan_segment(sg.THROW, seed, terminal, cfg, limits, geom)
    except uc.CycleInfeasible as exc:
        return False, exc.code, None, None
    if not within_margin(segment.meta.report, limits):
        return False, 'MARGIN', None, None
    return True, 'OK', segment.release_state, segment.takeoff_vel_mm_s


def hop_catch_cell(from_site, to_site, release_seed, takeoff_vel_mm_s,
                    flight_s: float, dwell_s: float,
                    offset_mm: Tuple[float, float],
                    limits: TrajectoryLimits, geom, cfg,
                    pattern_flight_s: Optional[float] = None):
    """R4's cross-site HOP steady state: CATCH at ``to_site`` (arriving from
    ``from_site``'s throw) carrying ``then_throw`` back TOWARD ``from_site``,
    then one more :func:`catch_cell` at ``from_site`` for that carried
    throw -- the command ``u`` (the landing offset) applies to BOTH legs,
    exactly as :func:`chained_catch_cell` applies it to a same-site chain's
    steady state.  Returns ``(ok, code)``.  A thin wrapper over
    :func:`folded_catch_cell` (catch at ``to_site``, release at ``to_site``,
    land at ``from_site``) -- the one body the one-ball oval shares.

    **``pattern_flight_s`` vs ``flight_s`` (R5, 2026-09-30 fix, same split as
    :func:`chained_catch_cell`).** ``flight_s`` (commanded, a possible ladder
    rung) sets ONLY the two carried throws' ballistic solves (``then_throw.
    flight_s`` here, and the ``flight_s`` argument :func:`catch_cell` uses
    below to size ITS carried throw's arrival). ``pattern_flight_s`` (default:
    ``flight_s``) sets every timing instant (``t_land_s``, ``t_release``) and
    every arrival velocity (both this catch's and the final :func:`catch_cell`'s)
    -- the schedule's own beat, which does not move with the command.
    """
    ok, code, _reports = folded_catch_cell(
        to_site, to_site, from_site, release_seed, takeoff_vel_mm_s,
        flight_s, dwell_s, offset_mm, limits, geom, cfg,
        pattern_flight_s=pattern_flight_s)
    return ok, code


def folded_catch_cell(catch_site, release_site, target_site, release_seed,
                       takeoff_vel_mm_s, flight_s: float, dwell_s: float,
                       offset_mm: Tuple[float, float],
                       limits: TrajectoryLimits, geom, cfg,
                       pattern_flight_s: Optional[float] = None):
    """The folded steady window: CATCH at ``catch_site`` carrying
    ``then_throw`` released at ``release_site`` toward ``target_site``, then
    one :func:`catch_cell` at ``target_site`` for that carried throw.
    Returns ``(ok, code, reports)`` -- ``reports`` the (up to two)
    ``FeasibilityReport``s of the segments that were planned, refused ones
    included when the gate carried one, for the oval's per-cell metrics.

    Hop: ``(to, to, from)`` -- release where it caught. Oval: ``(C, T, C)`` --
    the carry C -> T is INSIDE the folded catch window and the throw lands
    back on C (``ThenThrow.release_site``, plan two-ball-oval § 2.2).
    """
    Tp = float(pattern_flight_s) if pattern_flight_s is not None else float(flight_s)
    landing_mm = catch_site.catch_site_mm() + np.array(
        [float(offset_mm[0]), float(offset_mm[1]), 0.0])
    v_arrival = bal.arrival_velocity(takeoff_vel_mm_s, Tp)
    t_release = Tp + float(dwell_s)
    target_back_mm = target_site.throw_site_mm() + np.array(
        [float(offset_mm[0]), float(offset_mm[1]), 0.0])
    then_throw = sg.ThrowAfterCatch(t_release_s=t_release,
                                    site_mm=release_site.throw_site_mm(),
                                    target_mm=target_back_mm, flight_s=float(flight_s))
    terminal = sg.CatchTerminal(landing_mm=landing_mm, landing_vel_mm_s=v_arrival,
                                t_land_s=Tp,
                                # The executor settles at the RELEASE site
                                # (`release_site_of(skill).rest_site_mm()`,
                                # executor `_catch_terminal`): for a carried
                                # fold (oval, release T != catch C) the tail
                                # runs release -> T's rest, not 150 mm back to
                                # C in rest_tail_s (that read LIMIT_VEL 448
                                # at the rung-1 cell). Same-site folds (hop,
                                # columns) have release == catch: unchanged.
                                rest_site_mm=release_site.rest_site_mm(),
                                then_throw=then_throw)
    try:
        segment = sg.plan_segment(sg.CATCH, release_seed, terminal, cfg, limits, geom)
    except uc.CycleInfeasible as exc:
        return False, exc.code, [exc.report]
    if not within_margin(segment.meta.report, limits):
        return False, 'MARGIN', [segment.meta.report]
    # `Segment.release_state` (2026-09-30): the raw re-solve this replaced
    # omitted the hold knots and crashed run 1 of the R5 sweep on a seed the
    # production solve had accepted (`HAND_LIMIT_ACC` 3553 > 3500).
    # The extended plan's own `meta.report` drops the seating fields (the
    # merge of the head window with the settle tail keeps only peaks), so the
    # JUDGED window's metrics are re-read by validating the extended plan
    # itself (it carries the catch evidence and the carry window). A cell that
    # passed above was already admitted by exactly this gate.
    window_report = fz.validate_cycle(segment.plan, limits, geom)
    next_release_seed = segment.release_state
    next_takeoff = segment.takeoff_vel_mm_s
    ok2, code2, report2 = catch_cell_report(
        target_site, next_release_seed, next_takeoff, Tp, Tp, offset_mm,
        limits, geom, cfg)
    reports = [window_report, report2]
    if not ok2:
        return False, 'CHAIN_CATCH:%s' % code2, reports
    return True, 'OK', reports


def oval_window(site_t, site_c, apex_m: float, dwell_s: float):
    """``(catch_site, release_site, target_site)`` of the folded oval window,
    READ from the production compiler (``OneBallPattern(roles='fixed')`` +
    the fold in ``compile_one_ball``) rather than assumed, so a compiler change
    to who catches / releases / lands where breaks this sweep loudly instead of
    sweeping a window the executor no longer dispatches."""
    pattern = sc.OneBallPattern(sites=(site_t, site_c), apex_m=float(apex_m),
                                dwell_s=float(dwell_s), n_throws=3, roles='fixed')
    schedule = sc.compile_one_ball(pattern, t0_abs_s=0.0)
    folded = [sk for sk in schedule.skills
              if sk.kind == sc.CATCH and sk.then_throw is not None]
    if not folded:
        raise RuntimeError('oval compile produced no folded CATCH+then_throw')
    sk = folded[0]
    tt = sk.then_throw
    release_site = tt.release_site_or(sk.site)
    if not (sk.site is site_c and release_site is site_t and tt.target is site_c
            and abs(tt.t_release_abs_s - (sk.t_abs_s + float(dwell_s))) <= 1e-9
            and schedule.pattern == 'oval_1ball'):
        raise RuntimeError(
            'oval compile folded a window this sweep does not model: catch %s '
            'release %s target %s pattern %s' % (
                sk.site.name, release_site.name, tt.target.name,
                schedule.pattern))
    return sk.site, release_site, tt.target

