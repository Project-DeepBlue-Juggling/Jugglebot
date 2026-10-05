"""When a skill happens on the shared CAN wall clock (plan § 2.2 / § 2.4).

A :class:`Schedule` is a tuple of :class:`Skill` objects, each carrying the
EVENT instant it aims at (a release, a touch-down, or an at-rest-by time) and
the window the segment plans over to get there. :func:`compile_columns` builds
the columns pattern's schedule (plan § 1.2 / § 2.4): two sites, each running
its own self-toss, half a beat out of phase. Pure Python + numpy, no ROS
imports.
"""

from __future__ import annotations

import dataclasses
import math
from typing import Optional, Tuple

import numpy as np

import jugglebot.hardware_config as hw
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills.segments import CATCH, KINDS, REST, THROW
from jugglebot.motion.skills.sites import REST_CUP_Z_MM, REST_HAND_REV, Site
from jugglebot.motion.trajectory import ballistics_bc
from jugglebot.motion.trajectory import tilt_geometry as tg

#: SI gravity magnitude (m/s²) — the same number ``cup_cycle.GRAVITY`` /
#: ``ballistics_bc.G_VEC_MMS2`` already carry (9806 mm/s²), read off
#: ``ballistics_bc`` rather than restated so this module's ballistic timing
#: can never drift from the QP's own equality.
_G_SI = ballistics_bc.GRAVITY_MMS2 / 1000.0

#: A window shorter than this is not a window a segment can plan into —
#: ``feasibility.validate_cycle``'s per-knot passes need a handful of knots to
#: measure a jerk or a hand span at all.  Four knots at the fixed 40 Hz grid,
#: and it is the SAME floor ``executor.install_segment`` measures
#: ``WINDOW_TOO_SHORT`` against: one number, two uses.
MIN_WINDOW_KNOTS = 4
MIN_WINDOW_S = MIN_WINDOW_KNOTS * float(hw.JB_TRAJ_KNOT_DT_S)

# ── The wire-read budget (plan § 2.4) ────────────────────────────────────────
#
# These numbers are ONE budget — how much of a knot grid the solve may
# spend before the can-bridge has read past the splice — so they live together,
# and they live HERE rather than in ``executor`` because a skill states its own
# dispatch rule (:attr:`Skill.lead_s`) and ``schedule`` may not import
# ``executor`` (``executor`` imports ``schedule``).  ``executor`` imports them
# back by name, so ``executor.LEAD_S`` still resolves to this object.

#: Knots of the plan the WIRE has already read past the install instant: the
#: can-bridge's 2-knot lookahead plus one knot for the install's own flight time
#: (``project_canbridge_facts``: the 500 Hz interpolator consumes the knot pair
#: bracketing its own clock, so the knot at ``floor(τ/dt)`` is spent and the two
#: after it are in hand).  A splice at or before ``floor(τ/dt) + 3`` would
#: rewrite trajectory the Teensy is already interpolating.
WIRE_READ_KNOTS = 3

#: Knots of solve budget every dispatch must leave — the ONE number the
#: measured solve sizes, and the reason both leads below are what they are.
#:
#: 6 knots = 0.150 s.  Measured on the ROBOT, under sitting load (bag
#: recording, GUI, the tracker fit), 2026-09-18 sitting
#: (``temp/logs/launch_r2gate_20260918_1325.log``; CATCH ``install_segment``
#: plan times, n=51): **min 40.1, p50 76.4, p90 111.0, p95 113.4, max
#: 134.2 ms**.  So 0.150 s = p95 + one knot of margin, and it also clears the
#: measured MAX by 16 ms.  The earlier 26–34 ms figure this budget was sized
#: on (``tools/probes/skills_segment_sweep.py``, 2026-09-12,
#: ``temp/probes/skills_segment_run2.md``) was measured OFFLINE on an idle
#: Jetson and understates the loaded solve by 3-4×: at 3 knots (75 ms) of
#: budget, **16 of 23 catch attempts in that sitting refused
#: ``SPLICE_TOO_LATE``** ("the solve took 0.093–0.125 s … budget 0.083–0.100 s
#: from dispatch").
#:
#: The cost of every knot here is paid twice: each dispatch splices 25 ms
#: further ahead, and ``executor.CATCH_FREEZE_S`` (= :data:`LEAD_S` + dt) grows
#: with it, so the window in which a catch may still be RE-AIMED from the
#: tracker shrinks by the same amount.  Do not widen it to buy comfort — widen
#: it only against a measurement, and prefer making the solve faster.
SOLVE_BUDGET_KNOTS = 6

#: Knots of lead between "now" and the earliest knot a splice may land on.
#:
#: 9 knots = 0.225 s at the fixed 40 Hz grid: :data:`WIRE_READ_KNOTS` (3, spent
#: before the solve begins) + :data:`SOLVE_BUDGET_KNOTS` (6).  Derived, so the
#: budget is stated in exactly one place — ``executor``'s ``SPLICE_TOO_LATE``
#: measures ``(k_s - WIRE_READ_KNOTS) * dt`` against the same arithmetic.
#: History: 4 knots (2026-09-12) left 1 knot of budget and refused on the first
#: handoff; 6 knots left 3 (75 ms) and refused 16 of 23 catches on the loaded
#: robot (2026-09-18) — see :data:`SOLVE_BUDGET_KNOTS` for both measurements.
LEAD_KNOTS = WIRE_READ_KNOTS + SOLVE_BUDGET_KNOTS

#: Seconds of lead, derived — never a second copy of the knot grid.
LEAD_S = LEAD_KNOTS * float(hw.JB_TRAJ_KNOT_DT_S)

#: The DISPATCH lead of a skill whose segment FOLLOWS a release (every CATCH in
#: the columns pattern, with or without ``then_throw``).
#:
#: :data:`LEAD_KNOTS` + 2 = 11 knots = 0.275 s, i.e. 8 knots = 200 ms of solve
#: budget.  Derived from :data:`LEAD_KNOTS` (it was a literal 8 against a
#: 6-knot LEAD_KNOTS until 2026-09-18) so that the two leads move TOGETHER: the
#: two extra knots are the only fact this constant adds, and holding the gap
#: fixed keeps every dispatch instant in a compiled schedule shifted by the
#: same amount, which is what leaves ``compile_columns``'s dispatch-monotonicity
#: check exactly as much margin as it had.  It can exceed :data:`LEAD_KNOTS`
#: because such a segment splices AT the release knot whatever its dispatch
#: instant
#: (``executor._snap_to_release``): the splice knot is PINNED by the head's own
#: release, so buying solve time by dispatching earlier neither moves the seam
#: nor re-solves the ball that has already left the cup.  A skill that does NOT
#: follow a release has no such pin — its splice knot tracks its dispatch — so
#: it keeps :data:`LEAD_S` and its :data:`SOLVE_BUDGET_KNOTS` budget.
HANDOFF_LEAD_EXTRA_KNOTS = 2
HANDOFF_LEAD_KNOTS = LEAD_KNOTS + HANDOFF_LEAD_EXTRA_KNOTS

#: Seconds, derived.
HANDOFF_LEAD_S = HANDOFF_LEAD_KNOTS * float(hw.JB_TRAJ_KNOT_DT_S)


def flight_s(apex_m: float) -> float:
    """No-drag vertical flight time (s) for a throw that peaks ``apex_m`` above
    its release/catch height: ``t_f = 2·sqrt(2·apex_m / g)``."""
    if not float(apex_m) > 0.0:
        raise ValueError('apex_m must be > 0, got %r' % (apex_m,))
    return 2.0 * math.sqrt(2.0 * float(apex_m) / _G_SI)


def apex_m(flight_s_: float) -> float:
    """The exact inverse of :func:`flight_s`: the apex (m) a flight time (s)
    is consistent with — ``t_f = 2·sqrt(2·apex_m / g)`` run backward, i.e.
    ``apex_m = g · (t_f / 2)² / 2``, the SAME ``g`` (:data:`_G_SI`) so a
    round trip through both functions is exact to float precision.  Lets a
    caller that only has a flight time — an admissible box swept over an
    explicit flight grid before 2026-09-18
    (:func:`~jugglebot.motion.skills.admissible.load`) — recover the apex the
    command is expressed in."""
    if not float(flight_s_) > 0.0:
        raise ValueError('flight_s must be > 0, got %r' % (flight_s_,))
    return _G_SI * (float(flight_s_) / 2.0) ** 2 / 2.0


def apex_from_vz(vz_m_s: float) -> float:
    """The apex (m) above the catch plane implied by a ball's VERTICAL SPEED
    as it crosses that plane: ``h = v_z² / (2 g)``, exact for a parabola and
    the SAME ``g`` (:data:`_G_SI`) as :func:`flight_s`.

    This is how the learner reads its outcome (2026-09-18): the tracker's
    converged gravity-fixed fit reports the crossing velocity, and an apex
    taken from it is invariant to WHEN the ball was released — the bias a
    flight-time outcome could never shed (the physical release lags the
    commanded knot by 0.02-0.14 s throw to throw).
    """
    v = abs(float(vz_m_s))
    return v * v / (2.0 * _G_SI)


def apex_from_crossing(vz_m_s: float, rise_m: float) -> float:
    """The apex (m, release-relative) implied by a crossing at a DIFFERENT
    height than release, ``rise_m`` above release (negative if the crossing
    plane sits below the release plane -- e.g. a catch plane below the
    release plane, as every throw ran until 2026-10-05).

    Inverts the planner's own model for the release-to-crossing time ``T``:
    the average vertical speed over ``T`` is ``rise_m / T``, and the crossing
    (instantaneous) speed is ``g·T`` less than the release speed, so
    ``v_c = rise_m / T - g·T / 2``. Solving for ``T > 0`` (the physical root;
    ``v_c`` must be negative, i.e. descending, or the quadratic's positive
    root is not the crossing the ball we track actually makes):

        T = (-v_c + sqrt(v_c² + 2·g·rise_m)) / g

    and the apex is then read off :func:`apex_m` at that flight time (``T``
    is exactly the flight time a SYMMETRIC throw of this apex would take),
    using the SAME ``g`` (:data:`_G_SI`) as :func:`flight_s` /
    :func:`apex_from_vz`: ``apex = g·T² / 8``.

    At ``rise_m = 0`` this reduces exactly to :func:`apex_from_vz`: ``T``
    becomes ``2·|v_c| / g`` and ``apex = g·T²/8 = v_c² / (2·g)``.

    This is the fix for the catch-high plane change (plan R5 sitting 6,
    2026-10-05): the learner's outcome used to read ``apex_from_vz`` directly
    off the catch-plane crossing speed, which is only "the apex above the
    catch plane" when release and catch are the SAME height. With
    ``RELEASE_CUP_Z_MM`` and ``CATCH_CUP_Z_MM`` at different heights, a
    perfect plant reads ``y != u`` by an amount that tracks the plane offset
    (rise -30 mm -> y = u + 14.5 mm; rise +70 mm -> y = u - 35 mm) -- a
    spurious ~50 mm learner bias from a geometry change alone, not a real
    throw error. Reading the outcome through this function instead makes a
    perfect plant read ``y == u`` at ANY plane offset.

    Check (design doc worked example): ``vz_m_s=-4.349``, ``rise_m=-0.030``
    -> ``T=0.8801``, ``apex=0.9495``.

    Raises ``ValueError`` if ``vz_m_s`` is non-negative (not descending),
    non-finite, or the discriminant ``v_c² + 2·g·rise_m`` is negative (no
    real crossing time -- the ball never reaches a plane ``rise_m`` above
    release at this crossing speed).
    """
    v_c = float(vz_m_s)
    rise = float(rise_m)
    if not math.isfinite(v_c) or not math.isfinite(rise):
        raise ValueError('vz_m_s and rise_m must be finite, got %r, %r'
                          % (vz_m_s, rise_m))
    if not v_c < 0.0:
        raise ValueError('vz_m_s must be descending (< 0), got %r' % (vz_m_s,))
    discriminant = v_c * v_c + 2.0 * _G_SI * rise
    if not discriminant >= 0.0:
        raise ValueError(
            'no real crossing time for vz_m_s=%r, rise_m=%r (discriminant '
            '%r < 0)' % (vz_m_s, rise_m, discriminant))
    flight_time_s = (-v_c + math.sqrt(discriminant)) / _G_SI
    return apex_m(flight_time_s)


def beat_s(flight_s_: float, dwell_s: float) -> float:
    """Time between successive throws from the SAME hand: ``(t_f + d) / 2``."""
    return (float(flight_s_) + float(dwell_s)) / 2.0


def transit_s(flight_s_: float, dwell_s: float) -> float:
    """Time between a throw and the OTHER hand's next catch: ``(t_f − d) / 2``."""
    return (float(flight_s_) - float(dwell_s)) / 2.0


def ball_label(ball_id: int) -> str:
    """Operator-facing name for a schedule ball id (owner convention,
    2026-10-04). Code prose keeps ``'ball A'``/``'ball B'`` and schedule ids
    0/1; every OPERATOR-FACING string (log lines, START/END lines, the
    executor's evidence lines, the sim gate's verdict lines) names a ball
    through this function instead, as **Ball 1**/**Ball 2** — numbered by
    the order Jugglebot's hand FIRST THROWS them, not by site: ball A / id 0
    is the ball already in Jugglebot's hand at rest ("Ball 1"), ball B / id
    1 is the one Ball Butler feeds or that waits at the second site ("Ball
    2"). Sites keep their own names, so in fed columns Ball 1 holds at P2
    and Ball 2 lands at P1 — a deliberate crossover, not a bug to "fix".

    Never raises, so a log formatter can always produce a string: an id
    outside 0/1 falls back to ``'Ball %d' % (id + 1)``, and anything that
    cannot be read as an int falls back to ``'Ball ?'``.
    """
    try:
        bid = int(ball_id)
    except (TypeError, ValueError):
        return 'Ball ?'
    if bid == 0:
        return 'Ball 1'
    if bid == 1:
        return 'Ball 2'
    return 'Ball %d' % (bid + 1,)


def _vec3(value, name: str) -> np.ndarray:
    """Same validation as ``segments._vec3`` (private there — this module
    needs its own three-vector guard for :class:`LandingPrior`, and a schedule
    ball's identity should not have to import a segment-planning internal for
    it)."""
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.shape != (3,):
        raise ValueError('%s must be a 3-vector, got shape %s'
                          % (name, np.shape(value)))
    if not np.all(np.isfinite(arr)):
        raise ValueError('%s must be finite, got %r' % (name, arr.tolist()))
    return arr


@dataclasses.dataclass(frozen=True)
class LandingPrior:
    """An externally-observed ball arrival to aim a CATCH at, for a ball with
    no release of its own in THIS schedule (R4's reload: the ball came from
    Ball Butler's throw, so ``executor._previous_release`` has nothing to
    predict from — see :attr:`Skill.landing_prior`).

    Position, arrival velocity, wall-clock instant — the same three physical
    facts :class:`~jugglebot.motion.skills.segments.CatchTerminal` reads off
    any landing, and nothing more: no ``from_fit`` (that is the executor's own
    tracker-bookkeeping fact about a *different* landing source, the live
    tracker fit, and restating it here would be a second copy of the same
    concept under two names).
    """

    pos_mm: np.ndarray
    vel_mm_s: np.ndarray
    t_land_abs_s: float

    def __post_init__(self):
        object.__setattr__(self, 'pos_mm', _vec3(self.pos_mm, 'pos_mm'))
        object.__setattr__(self, 'vel_mm_s', _vec3(self.vel_mm_s, 'vel_mm_s'))
        if not np.isfinite(float(self.t_land_abs_s)):
            raise ValueError('t_land_abs_s must be finite, got %r'
                              % (self.t_land_abs_s,))


@dataclasses.dataclass(frozen=True)
class ThenThrow:
    """The same-site throw a CATCH carries out of the ball it just caught.

    A CATCH and the THROW of the SAME ball from the SAME site one dwell later
    are ONE window, not two segments: solved apart, the catch's runway
    decelerates the hand toward rest and the throw has to undo that from a seed
    one knot after touch-down with ``dwell - lead`` left, which is infeasible at
    every cell of a 480-cell grid (260k mm/s³ of leg jerk at the R2 operating
    point against a 200k limit — ``tools/probes/skills_segment_sweep.py``,
    2026-09-12, ``temp/probes/skills_segment_run1.md``).  Solved together as one
    STEADY window with a SETTLE tail the same point passes with 178k.

    ``t_release_abs_s`` is on the shared wall clock, like :attr:`Skill.t_abs_s`.
    ``y_d`` is the identity-prior command (landing xy offset in m against
    ``target``, and the APEX in m above the catch plane — the learner's
    outcome parameterisation, 2026-09-18).
    """

    t_release_abs_s: float
    y_d: Tuple[np.ndarray, float]
    target: Site
    #: The columns Stop's cross-site last throw (owner decision D2, 2026-09-30,
    #: R5 rescope — E1'): True when this carried release is the pattern's
    #: LAST throw, aimed at the OTHER site to land on the ball already held
    #: there.  Copied from :attr:`Skill.shadow_landing` by
    #: :func:`_fold_catch_throw_pairs` when the throw it folds set it — the
    #: ONE explicit flag both the executor's command bypass
    #: (:meth:`~jugglebot.motion.skills.executor.SkillExecutor._command_u`)
    #: and its outcome bookkeeping read, no inference from schedule shape.
    shadow_landing: bool = False

    def __post_init__(self):
        if not np.isfinite(float(self.t_release_abs_s)):
            raise ValueError('t_release_abs_s must be finite, got %r'
                              % (self.t_release_abs_s,))
        dy = np.asarray(self.y_d[0], dtype=float).reshape(-1)
        if dy.shape != (2,) or not np.all(np.isfinite(dy)):
            raise ValueError('y_d[0] must be a finite 2-vector, got %r'
                              % (self.y_d[0],))
        if not float(self.y_d[1]) > 0.0:
            raise ValueError('y_d[1] (apex_m) must be > 0, got %r'
                              % (self.y_d[1],))
        if not isinstance(self.target, Site):
            raise ValueError('target must be a Site, got %r' % (self.target,))


@dataclasses.dataclass(frozen=True)
class Skill:
    """One scheduled THROW / CATCH / REST, and the window a segment plans it
    over.

    ``t_abs_s`` is the EVENT instant (release / landing / at-rest-by) on the
    shared wall clock, seconds. ``window_s`` is the planned window from the
    segment's splice knot to that event, so :meth:`dispatch_s` — when the
    segment must be INSTALLED by — is ``t_abs_s − window_s − lead_s``.

    **A skill states its own dispatch rule.**  ``lead_s`` rides on the skill
    rather than on the executor because it is not one number: a segment that
    follows a release splices at a knot the HEAD pins, so it can be dispatched
    :data:`HANDOFF_LEAD_S` early and spend the extra knots on the solve, while
    a segment whose splice knot tracks its own dispatch cannot (see
    :data:`HANDOFF_LEAD_KNOTS`).  An executor-wide lead would have to be the
    smaller of the two for every skill.
    """

    kind: str
    ball_id: int
    site: Site
    t_abs_s: float
    window_s: float
    #: THROW only: the identity-prior command — landing xy (m) relative to
    #: ``target``, and the APEX (m) above the catch plane. The pattern's own
    #: apex, so a columns THROW always carries ``(zeros(2), apex_m)``; the
    #: schedule's flight time is DERIVED from it (:func:`flight_s`) and is
    #: not a learnable quantity (2026-09-18).
    y_d: Optional[Tuple[np.ndarray, float]] = None
    #: THROW only: the site the ball is to land at.
    target: Optional[Site] = None
    #: CATCH only: the same-site throw this catch carries out of the ball, as
    #: ONE window (:class:`ThenThrow`).  ``None`` for a standalone CATCH — the
    #: last catch of an attempt, and R4's reload.
    then_throw: Optional[ThenThrow] = None
    #: CATCH only: an externally-observed arrival (R4 reload) to aim at when
    #: this ball has no release of its own in this schedule — see
    #: :class:`LandingPrior` / ``executor._predicted_landing``.
    landing_prior: Optional[LandingPrior] = None
    #: CATCH only: the receive attitude (rx, ry) rad held through a held-axis
    #: catch (R4 reload) — passed to ``segments.CatchTerminal.hold_tilt``
    #: unchanged (``segments.receive_hold_tilt`` is the one derivation point).
    hold_tilt: Optional[Tuple[float, float]] = None
    #: CATCH only: the touch-down receive attitude (rx, ry) rad this catch is
    #: PINNED to — passed to ``segments.CatchTerminal.receive_tilt``
    #: unchanged. Unlike ``hold_tilt`` this holds no line and slaves no
    #: channel — it only overrides the single auto-computed
    #: ``tilt_to_receive`` endpoint at the touch-down knot
    #: (``unified_cycle.CycleGoals.receive_tilt``'s docstring has the
    #: physical reason). ``compile_columns(feed=...)`` is the one caller that
    #: sets it (``(0.0, 0.0)``, on the feed catch only — the skill carrying
    #: ``landing_prior=feed``). ``None`` (every other CATCH) keeps the
    #: auto-banked receive tilt. Mutually exclusive with ``hold_tilt``.
    receive_tilt: Optional[Tuple[float, float]] = None
    #: REST only: the attitude (rx, ry) rad this REST ENDS at — passed to
    #: ``segments.RestTerminal.tilt`` unchanged.  ``None`` (every pre-R4 REST)
    #: is the level rest.
    rest_tilt: Optional[Tuple[float, float]] = None
    #: REST/CATCH: the rest point (mm) this segment settles at, when it is
    #: NOT ``site.rest_site_mm()`` — a held-axis REST/CATCH (R4 reload)
    #: settles ON THE AXIS LINE through the landing point
    #: (``segments.hold_axis_site``), which is not the site's own xy at every
    #: z (U2 handoff: 29.8 mm apart at the 12° ceiling).  ``None`` (every
    #: pre-R4 skill) keeps the existing ``site.rest_site_mm()`` derivation.
    rest_site_mm: Optional[np.ndarray] = None
    #: REST only: whether the cup is holding a ball through this REST — see
    #: ``segments.RestTerminal.holds_ball``.  Defaults True (every pre-R4
    #: REST's behaviour); R4's PRE-TILT REST is the one caller that states
    #: False (the ball is still in the air, not caught yet).
    holds_ball: bool = True
    #: THROW only: True for the columns Stop's cross-site last throw (owner
    #: decision D2, 2026-09-30, R5 rescope — E1'): this throw's ``target`` is
    #: the OTHER site (100 mm away), aimed to land ON the ball the schedule
    #: already holds there (:func:`compile_columns`'s closing REST, unchanged
    #: pre-E form).  The explicit flag the executor's command bypass and
    #: outcome bookkeeping read — no inference from "nothing scheduled after
    #: this ball" (plan § 0: one data structure, no second copy of the same
    #: fact).  ``False`` (every throw before R5, and R5's own ``n == 1``
    #: schedule — nothing is held, so the last throw is an ordinary throw at
    #: its own site).
    shadow_landing: bool = False
    #: THROW only: True for a fed columns schedule's HOP ENTRY (owner
    #: decision, 2026-10-05, R5 sitting 6 prep — ``scratchpad/s6/design.md``
    #: Decision 1): THROW 0 released from the FEED site (``sites[1]``) with
    #: ``target=sites[0]`` — a cross-site throw into the schedule's own
    #: pattern — instead of a self-toss from ``sites[0]``, so the cup is
    #: already parked at the feed site for the feed catch rather than having
    #: to transit there after the launch. Set by :func:`compile_columns`
    #: (``entry='hop'``) on THROW 0 only; every other throw is an ordinary
    #: self-toss. Mutually exclusive with ``shadow_landing`` (THROW 0 can
    #: never be the columns Stop's cross-site last throw — that needs
    #: ``n_throws >= 2`` and ``i == n - 1``, never ``i == 0``). The
    #: executor bypasses the learner AND the admissible box for this throw
    #: (``u = y_d`` exactly) and writes no memory row — the same bypass
    #: mechanism ``shadow_landing`` uses, but none of its other semantics
    #: (this throw's ``caught`` reads normally; it is not landing on a ball
    #: already at rest). ``False`` (every throw before this decision, and
    #: every throw but THROW 0 under a hop-entry schedule).
    entry_hop: bool = False
    #: Seconds between this skill's DISPATCH and its splice base
    #: (``t_abs_s - window_s``).  :func:`compile_columns` sets it per skill;
    #: the default is the general :data:`LEAD_S`.
    lead_s: float = LEAD_S

    def __post_init__(self):
        if self.kind not in KINDS:
            raise ValueError('kind must be one of %s, got %r'
                              % (KINDS, self.kind))
        if self.shadow_landing and self.kind != THROW:
            raise ValueError('only a THROW may carry shadow_landing (kind=%r)'
                              % (self.kind,))
        if self.entry_hop and self.kind != THROW:
            raise ValueError('only a THROW may carry entry_hop (kind=%r)'
                              % (self.kind,))
        if self.entry_hop and self.shadow_landing:
            raise ValueError(
                'entry_hop and shadow_landing are mutually exclusive — '
                'THROW 0 (the only throw entry_hop ever marks) can never '
                'be the columns Stop\'s cross-site last throw')
        if self.then_throw is not None and self.kind != CATCH:
            raise ValueError(
                'only a CATCH may carry then_throw (kind=%r) — a throw is '
                'carried out of the ball a catch has just seated, and no other '
                'skill has one in the cup' % (self.kind,))
        if self.landing_prior is not None and self.kind != CATCH:
            raise ValueError('only a CATCH may carry landing_prior (kind=%r)'
                              % (self.kind,))
        if self.hold_tilt is not None and self.kind != CATCH:
            raise ValueError('only a CATCH may carry hold_tilt (kind=%r)'
                              % (self.kind,))
        if self.receive_tilt is not None and self.kind != CATCH:
            raise ValueError('only a CATCH may carry receive_tilt (kind=%r)'
                              % (self.kind,))
        if self.hold_tilt is not None and self.receive_tilt is not None:
            raise ValueError(
                'give hold_tilt OR receive_tilt, not both (see '
                'unified_cycle.CycleGoals.receive_tilt for why the two are '
                'mutually exclusive)')
        if self.rest_tilt is not None and self.kind != REST:
            raise ValueError('only a REST may carry rest_tilt (kind=%r)'
                              % (self.kind,))
        if not float(self.window_s) > 0.0:
            raise ValueError('window_s must be > 0, got %r' % (self.window_s,))
        if not float(self.lead_s) >= 0.0:
            raise ValueError('lead_s must be >= 0, got %r' % (self.lead_s,))

    def dispatch_s(self) -> float:
        """The wall-clock instant the segment must be installed by."""
        return (float(self.t_abs_s) - float(self.window_s)
                - float(self.lead_s))


@dataclasses.dataclass(frozen=True)
class Pattern:
    """Columns-pattern parameters (columns only at R2 — plan § 1.2).

    Ball naming (owner, 2026-10-04): this module's own code prose says
    ``'ball A'``/``'ball B'`` and schedule ids 0/1 throughout — unchanged by
    this convention. Every OPERATOR-FACING string instead names a ball
    through :func:`ball_label` as **Ball 1**/**Ball 2** (ball A/id 0 is
    "Ball 1", ball B/id 1 is "Ball 2"); see that function's docstring for
    the full glossary, including the deliberate P1/P2 crossover in fed
    columns.
    """

    sites: Tuple[Site, Site]
    apex_m: float
    dwell_s: float
    n_throws: int
    #: The first THROW's window, from rest: the sweep's measured minimum
    #: feasible launch period (0.4 s at 0.8-1.0 m apex, 2026-09-12 — plan § 0).
    launch_s: float = 0.4
    #: The SETTLE tail every THROW/CATCH segment carries — ``segments``'s
    #: probed default, imported rather than restated.
    rest_tail_s: float = sg.REST_TAIL_S
    #: Schedule ball ids (0 and/or 1, :func:`compile_columns`'s own
    #: ``i % 2`` numbering) that produce MOTION ONLY — B2's
    #: ``columns_1ball`` (``SkillNode._run_columns_1ball``, phantom ball 1 —
    #: Jugglebot's ball A real, nothing fed) and R5 sitting 4's
    #: ``columns_1ball_fed`` (``SkillNode._run_columns(..., phantom_a=True)``,
    #: the other half: phantom ball 0 — ball A's own strokes fly empty while
    #: ball B is real and fed by Ball Butler through the SAME
    #: ``compile_columns(pattern, feed=...)`` path the ordinary ``columns``
    #: pattern uses): the cup still transits to the phantom's site and the
    #: hand still strokes empty (the legs see identical dynamics to a real
    #: two-ball columns attempt), but nothing is actually in the cup. ``()``
    #: (every pattern before B2, and ordinary columns) keeps every ball
    #: real. Nothing in :func:`compile_columns` reads WHICH id is named here
    #: — it is generic over either ball being the phantom; only
    #: :func:`phantom_feed_prior` (``columns_1ball`` only) assumes ball 1.
    #: Carried onto :attr:`Schedule.phantom_balls` unchanged by
    #: :func:`compile_columns` — the ONE place this tuple is read by a
    #: compiler; the enforcement points (no tracker latch / no
    #: OUTCOME-and-learner row / no re-aim / no ball-evidence precondition /
    #: never a D3 survivor target) live downstream, in
    #: ``SkillNode._maybe_announce`` and ``SkillExecutor._register_outcome``
    #: / ``_register_catch`` / ``_tracked_landing`` / ``_dispatch`` /
    #: ``_survivor_catch`` — see :meth:`Schedule.is_phantom`.
    phantom_balls: Tuple[int, ...] = ()

    def __post_init__(self):
        bad = tuple(b for b in self.phantom_balls if int(b) not in (0, 1))
        if bad:
            raise ValueError('phantom_balls may only name schedule ball ids '
                             '0 or 1, got %r' % (self.phantom_balls,))


@dataclasses.dataclass(frozen=True)
class Schedule:
    """An ordered, immutable timeline of skills. The executor tracks which of
    ``skills`` it has already dispatched — this object does not mutate."""

    skills: Tuple[Skill, ...]
    flight_s: float
    beat_s: float
    transit_s: float
    dwell_s: float
    t0_abs_s: float
    #: Which compiler built this schedule — ``'columns'``
    #: (:func:`compile_columns`), ``'self_toss'`` or ``'hop'``
    #: (:func:`compile_one_ball`, one or two sites — R4, owner decision D1
    #: 2026-09-23).  ``compile_reload`` (R4, owner decision D4 2026-09-23)
    #: fills this with ``'self_toss'``/``'hop'`` too, NOT a third label — its
    #: own PRE-TILT REST / held CATCH / DECAY REST never reach
    #: :meth:`SkillExecutor._command_u` (only a THROW does), and every THROW
    #: it schedules afterward is kinematically an ordinary self-toss/hop
    #: throw (same site, same apex/dwell, a fresh-origin THROW 0 from a level
    #: rest then dwell-carried throws) — so the box already swept for that
    #: pattern is the CORRECT one, and a ``'reload'`` label would only send
    #: every such throw to a box that does not exist (no sweep runs at R4;
    #: `tools/` is out of scope for this unit).  Read by the admissible-box
    #: selection at dispatch
    #: (:mod:`~jugglebot.motion.skills.admissible`), which a box is swept
    #: per-pattern for.  REQUIRED, no default (R4 audit call, 2026-09-23):
    #: a schedule with no stated pattern is a schedule the executor cannot
    #: safely look a box up for, and a defaulted label would let a hand-built
    #: schedule silently borrow another pattern's swept box -- the same class
    #: of defect as a box swept for one apex being reused at another.
    pattern: str
    #: Schedule ball ids that are motion-only — see :class:`Pattern`'s own
    #: field (the ONE place a compiler sets this to anything but ``()``:
    #: :func:`compile_columns`, when ``pattern.phantom_balls`` is non-empty).
    #: ``()`` for every schedule :func:`compile_one_ball` / :func:`compile_reload`
    #: / :func:`compile_reload_wait` build — no pattern's ball is ever a
    #: phantom except B2's ``columns_1ball`` and R5 sitting 4's
    #: ``columns_1ball_fed``.
    phantom_balls: Tuple[int, ...] = ()

    def due(self, t_abs_s: float) -> Tuple[Skill, ...]:
        """Skills whose dispatch instant has passed by ``t_abs_s``."""
        return tuple(s for s in self.skills
                     if s.dispatch_s() <= float(t_abs_s))

    def is_phantom(self, ball_id: int) -> bool:
        """True if ``ball_id`` is this schedule's motion-only ball (B2).

        THE INVARIANT this flag enforces, at its three call sites
        (``SkillNode._maybe_announce``, ``SkillExecutor._register_outcome``,
        ``SkillExecutor._tracked_landing``): **a phantom ball produces
        motion and nothing else.** Its skills install exactly like a real
        ball's — same cup transit, same hand stroke, same leg dynamics — but
        it arms no tracker latch, waits on no release evidence, writes no
        OUTCOME row, trains no learner row, and its catches never read a
        tracker fit (open-loop on the schedule, no re-aim).
        """
        return int(ball_id) in self.phantom_balls


#: Knots of margin the CLOSING REST's splice base keeps PAST the end of the
#: last catch's own rest tail, so that REST is a FRESH ORIGIN from a machine
#: already at rest (``executor.install_segment``: ``t_now + lead >= record.end_s``)
#: and never a splice into the tail's last knots.
#:
#: MEASURED 2026-09-23 (scratchpad ``probe_r4_hop_schedule.py``, the one-ball HOP
#: at 250 mm through the real install chain, limits 300/5000/150000/3500): with
#: the REST's event exactly ``2 * rest_tail`` after the last touch-down its splice
#: base lands ON the tail's terminal knot, the QP's ``round(period / dt)`` puts
#: the record's end up to half a knot later than that base, and the install
#: therefore takes the SPLICE branch — whose seam re-gate read 156 495 mm/s³ of
#: leg jerk against 150 000 (the 100 mm hop passed at 101 394 by margin, not by
#: construction).  This is R3's carried item (m) ("the closing REST after a
#: spliced catch is intermittently refused LIMIT_JERK ... 24 % of tick phases
#: at 0.9 m"), now deterministic at 250 mm.  Two knots covers the rounding
#: (≤ half a knot) plus the tick quantisation of the dispatch (one knot) with a
#: knot to spare; the attempt ends 50 ms later, which nothing measures.
REST_FRESH_MARGIN_KNOTS = 2
REST_FRESH_MARGIN_S = REST_FRESH_MARGIN_KNOTS * float(hw.JB_TRAJ_KNOT_DT_S)


def closing_rest_t_abs(last_catch_t_abs_s: float, rest_tail_s: float) -> float:
    """The event instant of a schedule's CLOSING REST: two ``rest_tail_s`` after
    the last touch-down (one for that catch's own tail, one for the REST's window
    — one tail would dispatch the REST before the touch-down and splice the catch
    away) plus :data:`REST_FRESH_MARGIN_S`, so the REST installs as a fresh origin
    from rest rather than a splice into the tail's final knots."""
    return (float(last_catch_t_abs_s) + 2.0 * float(rest_tail_s)
            + REST_FRESH_MARGIN_S)


def _check_window(name: str, window_s: float) -> None:
    if window_s < MIN_WINDOW_S - 1e-12:
        raise ValueError(
            '%s window %.4f s is below the %d-knot floor (%.4f s at %.4f s/knot)'
            ' — nothing this short leaves the gate a stencil to measure'
            % (name, window_s, MIN_WINDOW_KNOTS, MIN_WINDOW_S,
               float(hw.JB_TRAJ_KNOT_DT_S)))


def _fold_catch_throw_pairs(skills, dwell_s: float):
    """Fold every same-site (CATCH, THROW) pair of ONE ball into one CATCH.

    The event TABLE is unchanged — the ball is still caught at ``t_land`` and
    still released at ``t_land + dwell``.  What changes is how many segments
    those two events are planned as: one, with the release carried on the
    catch's :attr:`Skill.then_throw` (see :class:`ThenThrow` for the measured
    reason).  The first throw, which comes from rest and has no catch before it,
    stays a THROW skill; the chronologically last catch has no throw after it
    and stays a standalone CATCH.
    """
    out = []
    folded = set()
    for i, sk in enumerate(skills):
        if i in folded:
            continue
        if sk.kind != CATCH:
            out.append(sk)
            continue
        pair = None
        for j in range(i + 1, len(skills)):
            other = skills[j]
            if (other.kind == THROW and other.ball_id == sk.ball_id
                    and other.site is sk.site
                    and abs(other.t_abs_s - (sk.t_abs_s + dwell_s)) <= 1e-9):
                pair = (j, other)
                break
        if pair is None:
            out.append(sk)
            continue
        j, throw = pair
        folded.add(j)
        out.append(dataclasses.replace(sk, then_throw=ThenThrow(
            t_release_abs_s=throw.t_abs_s, y_d=throw.y_d, target=throw.target,
            shadow_landing=throw.shadow_landing)))
    return out


def _release_instants(sk: Skill) -> Tuple[float, ...]:
    """The wall-clock instants at which ``sk``'s segment lets a ball go.

    A THROW releases at its own event instant; a CATCH carrying a
    :class:`ThenThrow` releases at the carried instant; a standalone CATCH and a
    REST release nothing.
    """
    if sk.kind == THROW:
        return (float(sk.t_abs_s),)
    if sk.kind == CATCH and sk.then_throw is not None:
        return (float(sk.then_throw.t_release_abs_s),)
    return ()


def _assign_leads(skills):
    """Give each skill the dispatch lead its SPLICE KNOT allows.

    A segment whose splice base falls at or before the last release the head
    already carries splices AT that release knot whatever its dispatch instant
    (``executor._snap_to_release``), so its seam cannot move and the extra lead
    is pure solve budget: it gets :data:`HANDOFF_LEAD_S`
    (:data:`SOLVE_BUDGET_KNOTS` + :data:`HANDOFF_LEAD_EXTRA_KNOTS` = 8 knots of
    budget).  Every other segment's splice knot tracks its own dispatch —
    dispatch it earlier and it simply splices earlier — so it gets
    :data:`LEAD_S` and the bare :data:`SOLVE_BUDGET_KNOTS` (6 knots).

    In the columns pattern this makes every CATCH a handoff (its base IS the
    previous release — see :func:`compile_columns`) and leaves the launch THROW
    (a fresh origin from rest) and the REST (dispatched a tail after the last
    touch-down, with no release between) on the general lead.  The rule is
    stated on the SEGMENT's geometry, not on the kind, so a pattern that puts a
    standalone catch after a throw gets the handoff lead too.
    """
    out = []
    last_release_s = None
    for sk in skills:
        base = float(sk.t_abs_s) - float(sk.window_s)
        follows_release = (last_release_s is not None
                           and base <= last_release_s + 1e-9)
        out.append(dataclasses.replace(
            sk, lead_s=(HANDOFF_LEAD_S if follows_release else LEAD_S)))
        for t_rel in _release_instants(sk):
            last_release_s = (t_rel if last_release_s is None
                              else max(last_release_s, t_rel))
    return out


def schedule_has_shadow_landing(schedule: Schedule) -> bool:
    """True if any skill in ``schedule`` carries the columns Stop's
    cross-site last throw (owner decision D2, 2026-09-30 — ``Skill.
    shadow_landing`` / ``ThenThrow.shadow_landing``, set by
    :func:`compile_columns` for ``n_throws >= 2``): the schedule's clean end
    state is "one ball held, the last ball rests on the platform", not an
    ordinary rest.

    False for every non-columns pattern (neither field is ever set outside
    :func:`compile_columns`) and for a columns attempt whose last throw
    never made it into the final schedule — a D3 drop's survivor tail
    (:meth:`~jugglebot.motion.skills.executor.SkillExecutor.
    _install_survivor_tail`) strips ``then_throw`` before appending its own
    closing REST, so this reads False there too: D2 and D3 are mutually
    exclusive end states by construction, and the caller (``skill_node.
    _log_attempt_end``) only asks this when ``end_code`` is empty — a D3
    drop already has its own, more specific, code.
    """
    return any(sk.shadow_landing
              or (sk.then_throw is not None and sk.then_throw.shadow_landing)
              for sk in schedule.skills)


def compile_columns(pattern: Pattern, t0_abs_s: Optional[float] = None,
                    feed: Optional[LandingPrior] = None,
                    entry: str = 'transit') -> Schedule:
    """The columns pattern's schedule (plan § 1.2 / § 2.4).

    **Exactly one of ``t0_abs_s``/``feed`` (R5, owner decision D1,
    2026-09-30).** ``t0_abs_s`` is the pre-R5 free clock instant (still used
    by every caller that already knows when ball B lands — tests, and a
    probe compile). ``feed`` is ball B's REAL, externally-observed arrival
    (a Ball Butler ``ThrowAnnouncement`` or a converged tracker fit,
    ``SkillNode._run_columns``'s FEED WAIT): the whole reason columns needs
    a feed at all is that ``t0`` is not a free choice once a real ball is
    already falling toward site 1 — it is READ OFF that ball's own landing
    instant, ``t0 = feed.t_land_abs_s - transit_s(pattern)`` (the schedule's
    own definition: "ball B is already in flight, landing at site 1 at
    ``t0 + transit_s``", unchanged below), and the initial CATCH carries
    ``landing_prior=feed`` so the executor aims at the OBSERVED arrival
    rather than predicting one from a release this schedule never issued —
    the same mechanism :func:`compile_reload`'s CATCH already uses
    (``executor._predicted_landing`` honours ``Skill.landing_prior``
    unconditionally, no new machinery here). Giving both, or neither, is
    refused: a free ``t0`` and an observed arrival cannot both be the
    authority for when ball B lands.

    Columns is two SELF-tosses (ball A at ``sites[0]``, ball B at ``sites[1]``)
    run half a beat out of phase, not a crossing pattern — each throw's target
    is its OWN site. At ``t0_abs_s`` ball A is in the hand at site 0 (about to
    be thrown) and ball B is already in flight, landing at site 1 at
    ``t0 + transit_s``.

    Throw ``i`` (0-indexed, ``i = 0 .. n_throws-1``) is ball ``i % 2`` from
    ``sites[i % 2]`` at ``t0 + i·beat_s``; its window is ``launch_s`` for the
    very first throw (from rest) and ``dwell_s`` thereafter (the segment
    splices right after the previous catch and releases at ``t_abs_s``). Every
    throw except the LAST gets a matching CATCH at ``t_abs_s + flight_s``, same
    ball, same site, window ``transit_s`` (the segment splices right after the
    release and lands at ``t_abs_s``). Ball B's PRE-EXISTING flight (airborne
    before ``t0``) gets one extra CATCH with no matching THROW in this
    schedule.

    **The Stop (owner decision D2, 2026-09-30, R5 rescope — E1'): the last
    throw is aimed at the OTHER site, not its own.** For ``n_throws >= 2`` the
    last throw's ``target`` is ``sites2[n % 2]`` — the site the
    SECOND-TO-LAST ball is caught and held at (always the other of the two
    sites, since consecutive throws alternate) — and it carries
    ``shadow_landing=True``. Nothing meets it in the air: the schedule's
    closing REST returns to the pre-E1 form, at the chronologically LAST
    CATCH's own site (:func:`closing_rest_t_abs`, unchanged), so by the time
    this throw lands the platform is already sitting there holding the ball
    it caught, and the falling ball strikes it — the platform comes to rest
    holding both (not a resumable state — cone delivery is a later rung).

    This replaces E's first design (a shadow REST that had to physically
    TRAVEL the 100 mm between sites in the ~0.228 s left after the previous
    catch's tail): probed 2026-09-30, that REST refused ``LIMIT_VEL`` at the
    session cap of 300 mm/s (needs >= 0.333 s for 100 mm on velocity alone,
    before any accel/decel ramp). Aiming the throw that is ALREADY being
    planned costs nothing extra and needs no travel at all — the platform
    never has to move a second time; it is already at the site holding the
    other ball. This is schedule-only: no new planner terminal — the last
    throw (or, once folded, the CATCH that carries it) plans exactly like any
    other throw, just aimed cross-site. Probed 2026-09-30 at the reference
    operating point (apex 0.9 m, dwell 0.30 s, separation 100 mm, session
    limits 300/5000/150000): the real ``segments.plan_segment`` ACCEPTS the
    cross-site catch-with-throw (peak leg jerk 149 210 mm/s³ against the
    150 000 cap — no headroom, but this is a single terminal event with
    ``u = y_d`` fixed, not a box-swept command the learner re-aims, so the
    tight margin is acceptable here and nowhere else).

    ``n_throws == 1`` (a single launch, nothing caught yet) keeps the pre-R5
    behaviour: the only throw stays at its own site, ``shadow_landing`` stays
    ``False`` — there is nothing held to land on.

    The executor bypasses the learner and the admissible box entirely for a
    ``shadow_landing`` throw (``u = y_d`` exactly, no clip): the box is swept
    per pattern for SAME-site throws, this is a cross-site one, and precision
    buys nothing when the ball is landing on a ball already at rest — see
    ``executor.SkillExecutor._command_u``.

    **The HOP ENTRY (owner decision, 2026-10-05, R5 sitting-6 prep —
    ``entry='hop'``).** A fed schedule's pre-R5 entry parks
    ball A at ``sites[0]`` and throws it there (THROW 0, a self-toss) while
    ball B is still falling toward ``sites[1]`` — so the cup has to TRANSIT
    from ``sites[0]`` to ``sites[1]`` between the launch and the feed catch,
    arriving against the feed ball's own velocity rather than parked and
    waiting (measured, bag 2026-10-05_16-42-57: 13/21 ``columns_1ball_fed``
    runs never even reached the feed catch window, and the 6 that did caught
    0/6 clean). Under the hop entry, ball A is instead held at the FEED site
    (``sites[1]``) through the opening REST and THROW 0 releases it FROM
    THERE, cross-site, at ``target=sites[0]`` — same apex, same flight time,
    same ``t0`` as the self-toss it replaces (only the release ``site``
    changes; the catch that follows it, and everything else in the
    schedule, keeps landing at ``sites[0]`` exactly as before). The cup is
    therefore already parked at ``sites[1]`` and simply stays there for the
    feed catch, instead of arriving mid-transit. ``entry='hop'`` requires
    ``feed is not None`` — a free-``t0_abs_s`` schedule has no real feed
    ball to park for, so there is nothing for the hop to buy — and raises
    ``ValueError`` otherwise. The feed CATCH keeps ``receive_tilt=(0.0,
    0.0)`` unchanged (the level-pinned touch-down attitude, independent of
    which entry parked the cup there). THROW 0 carries
    :attr:`Skill.entry_hop` under the hop entry (see that attribute for the
    executor-side bypass); ``entry='transit'`` reproduces the pre-R5
    self-toss entry exactly (kept for A/B comparison and rollback) and
    never sets ``entry_hop`` on any skill, so every pre-existing caller
    (which does not pass ``entry`` at all) is byte-for-byte unchanged.

    **Every same-site (CATCH, THROW) pair of one ball is emitted as ONE CATCH
    skill carrying ``then_throw``** (:func:`_fold_catch_throw_pairs`), so the
    schedule holds ``1 + (n_throws − 1) + 1 + 1 = n_throws + 2`` skills: the
    launch THROW from rest, ``n_throws − 1`` catch-with-throws, the last
    (standalone) CATCH, and the REST. The event times are untouched.

    **Why the dispatch instant of a catch-with-throw lands ON the previous
    release.** Its ``window_s`` is the transit, so its dispatch is
    ``t_land − transit − lead``, and ``t_land − transit`` IS the previous
    release instant — that is what :func:`transit_s` measures (a throw to the
    OTHER hand's next catch), so it holds for every catch in the pattern by
    construction rather than by arithmetic coincidence. The splice knot
    ``splice_knot`` then returns is therefore the release knot itself or one of
    the ``n_detach`` knots after it, which is exactly the case
    ``executor.install_segment`` SNAPS to the release knot and seeds
    post-release — the ring's own handoff, with the ball that just left the cup
    keeping its detach cone.  That splice knot is therefore pinned by the HEAD
    and not by the dispatch, which is why every such skill carries
    :data:`HANDOFF_LEAD_S` (:func:`_assign_leads`) and spends the extra two
    knots on the solve rather than on moving the seam.
    """
    if (t0_abs_s is None) == (feed is None):
        raise ValueError(
            'compile_columns takes exactly one of t0_abs_s (a free clock '
            'instant) or feed (R5: ball B\'s observed arrival, from which '
            't0 is derived) — got t0_abs_s=%r feed=%r' % (t0_abs_s, feed))
    if entry not in ('hop', 'transit'):
        raise ValueError("entry must be 'hop' or 'transit', got %r" % (entry,))
    if entry == 'hop' and feed is None:
        raise ValueError(
            "entry='hop' requires feed (a free-t0_abs_s schedule has no "
            "real feed ball to park THROW 0 for)")
    t_f = flight_s(pattern.apex_m)
    beta = beat_s(t_f, pattern.dwell_s)
    tau = transit_s(t_f, pattern.dwell_s)
    if feed is not None:
        t0_abs_s = float(feed.t_land_abs_s) - tau
    if not tau > 0.0:
        raise ValueError(
            'dwell %.4f s >= flight %.4f s leaves no transit — the ball would '
            'have to land before the other hand finishes its dwell'
            % (pattern.dwell_s, t_f))
    n = int(pattern.n_throws)
    if n < 1:
        raise ValueError('n_throws must be >= 1, got %r' % (pattern.n_throws,))
    site0, site1 = pattern.sites
    sites2 = (site0, site1)

    # THE SCHEDULE IS BUILT ON ITS OWN CLOCK (t = 0 at the first throw) AND
    # SHIFTED TO t0_abs_s ONCE, AT THE END. Pairing a catch with its throw
    # (`_fold_catch_throw_pairs`), assigning the handoff lead (`_assign_leads`)
    # and the dispatch-order check below all compare instants to 1e-9 s. On a
    # ROS wall clock (~1.79e9 s) a double resolves only ~2.4e-7 s, so those
    # comparisons failed at random: MEASURED 2026-09-13 on the robot and offline,
    # a 20-throw schedule compiled at t0 = 1789263419.5 held 24 skills (two pairs
    # unfolded, each a THROW spliced one knot after a rest-terminal catch ->
    # LIMIT_JERK) and every other handoff on LEAD_S instead of HANDOFF_LEAD_S
    # (every SPLICE_TOO_LATE of the R2 gate sitting), against the correct 22 at
    # t0 = 10. Every test, the sim gate and the rehearsal ran near t = 0, which
    # is why none of them saw it (logbook 2026-09-13-skill-stack-r2-gate-sittings).
    t0_rel = 0.0

    skills = []
    _check_window('initial CATCH', tau)
    # `feed.t_land_abs_s` is already the ORIGINAL absolute instant (the
    # same "not the relative one `_shifted` would double-shift" note as
    # `compile_reload`'s own CATCH), so it rides through unshifted —
    # `_shifted` below only ever moves `Skill.t_abs_s` / `ThenThrow.
    # t_release_abs_s`, never `landing_prior`.
    #
    # `receive_tilt=(0.0, 0.0)` ONLY when this is a real feed (`feed is not
    # None`): the level-pinned touch-down attitude (`Skill.receive_tilt`'s
    # docstring) is what makes a BB feed's oblique arrival fit the transit +
    # dwell window at all — a `t0_abs_s` schedule's initial catch has no real
    # arrival to pin against (its own self-toss/hop landing is already
    # vertical or near it) and keeps the auto-banked default, `None`.
    skills.append(Skill(kind=CATCH, ball_id=1, site=site1,
                        t_abs_s=t0_rel + tau, window_s=tau,
                        landing_prior=feed,
                        receive_tilt=((0.0, 0.0) if feed is not None else None)))

    _check_window('THROW 0 (the launch from rest)', pattern.launch_s)
    if n >= 2:
        # Every throw after the first is carried by the catch one dwell before
        # it, so the dwell is the span from that catch's touch-down to its
        # release inside ONE window — still a span the gate must be able to
        # measure a jerk across.
        _check_window('dwell (touch-down to the throw the catch carries)',
                      pattern.dwell_s)

    for i in range(n):
        ball_i = i % 2
        site_i = sites2[ball_i]
        t_throw = t0_rel + i * beta
        window = pattern.launch_s if i == 0 else pattern.dwell_s
        # The Stop (D2, 2026-09-30, R5 rescope — E1', see the docstring): the
        # LAST throw targets the OTHER site — the one the second-to-last ball
        # is caught and held at (`sites2[n % 2]`, always the other of the two
        # since consecutive throws alternate) — instead of its own. Every
        # earlier throw is an ordinary self-toss, target = its own site.
        # `n == 1` has no second-to-last ball to aim at, so it is excluded
        # (`i == n - 1 and n >= 2`) and the only throw stays a self-toss.
        is_stop_throw = (i == n - 1 and n >= 2)
        target_i = sites2[n % 2] if is_stop_throw else site_i
        # The hop entry (owner decision, 2026-10-05 — see the docstring):
        # THROW 0 only, and only under `entry='hop'`. It is released from
        # the FEED site (`sites2[1]`) rather than its own self-toss site
        # (`site_i`, still `sites2[0]` here) -- `target_i` is untouched by
        # this (it is already `site_i` == `sites2[0]` for i == 0, since
        # `is_stop_throw` can never be True at i == 0), so the ball still
        # lands at `sites[0]` exactly as the self-toss entry would.
        is_hop_throw = (entry == 'hop' and i == 0)
        release_site_i = sites2[1] if is_hop_throw else site_i
        throw_sk = Skill(kind=THROW, ball_id=ball_i, site=release_site_i,
                         t_abs_s=t_throw, window_s=window,
                         y_d=(np.zeros(2), pattern.apex_m), target=target_i,
                         shadow_landing=is_stop_throw, entry_hop=is_hop_throw)
        skills.append(throw_sk)
        if i <= n - 2:
            _check_window('CATCH %d' % i, tau)
            # `site=target_i`, not `site_i`: the catch must sit where the
            # ball actually LANDS. Identical to `site_i` for every throw
            # except the hop entry's THROW 0, where the two differ by
            # construction (release at the feed site, landing at its own).
            skills.append(Skill(kind=CATCH, ball_id=ball_i, site=target_i,
                                t_abs_s=t_throw + t_f, window_s=tau))

    skills.sort(key=lambda s: s.t_abs_s)
    skills = _fold_catch_throw_pairs(skills, float(pattern.dwell_s))

    catches = [s for s in skills if s.kind == CATCH]
    last_catch = max(catches, key=lambda s: s.t_abs_s)
    # The closing REST (pre-E1 form, restored): two `rest_tail_s` past the
    # last CATCH's own touch-down plus the fresh-origin margin, at that
    # catch's site — unchanged by the Stop, because nothing has to travel
    # there any more (the last throw is already aimed at it).
    rest_t = closing_rest_t_abs(float(last_catch.t_abs_s), pattern.rest_tail_s)
    _check_window('closing REST', pattern.rest_tail_s)
    skills.append(Skill(kind=REST, ball_id=last_catch.ball_id,
                        site=last_catch.site, t_abs_s=rest_t,
                        window_s=pattern.rest_tail_s))

    skills = _assign_leads(skills)

    # Dispatch monotonicity, on the DISPATCH instants themselves: the leads are
    # no longer one number (`_assign_leads`), so the base order is no longer the
    # dispatch order and checking it would be checking the wrong thing.
    disp = [s.dispatch_s() for s in skills]
    for k in range(1, len(disp)):
        if disp[k] < disp[k - 1] - 1e-9:
            raise ValueError(
                'skill %d (%s at %.4f s, window %.4f s, lead %.3f s) must '
                'install before skill %d (%s at %.4f s, window %.4f s, lead '
                '%.3f s) installs, but dispatches later (%.4f s vs %.4f s) — '
                'the two windows overlap'
                % (k - 1, skills[k - 1].kind, skills[k - 1].t_abs_s,
                   skills[k - 1].window_s, skills[k - 1].lead_s,
                   k, skills[k].kind, skills[k].t_abs_s, skills[k].window_s,
                   skills[k].lead_s, disp[k - 1], disp[k]))

    t0 = float(t0_abs_s)
    skills = [_shifted(s, t0) for s in skills]
    return Schedule(skills=tuple(skills), flight_s=t_f, beat_s=beta,
                    transit_s=tau, dwell_s=float(pattern.dwell_s),
                    t0_abs_s=t0, pattern='columns',
                    phantom_balls=pattern.phantom_balls)


#: Duration (s) of the OPENING REST that lifts the hand from the ACTIVATE park
#: (cup 679.6 mm, 10 mm under the 689.6 mm planner floor — a THROW from there
#: refuses ``HAND_STROKE``, R3 probe 2026-09-13) to the site's floor, so THROW 0
#: below is a fresh origin from an actual rest rather than from the park.
#:
#: THE ONE DEFINITION (like :data:`~jugglebot.motion.skills.sites.
#: RELEASE_CUP_Z_MM`): restated from the FSM's ``_UNIFIED_FLOOR_LIFT_S``
#: (``reload_coordinator_node.py:777``, MEASURED offline, probe 2026-09-07 —
#: a 1.0 s SETTLE from -0.1087 rev lands exactly on the floor rev with a peak
#: hand rate of 0.654 rev/s and 2.55 rev/s² of acceleration, three orders below
#: the 3500 cap the un-lifted launch broke) rather than imported from it,
#: because ``motion/`` may not import a ROS node. Columns' opening REST is
#: carried to R4 rather than given this treatment here (R3 build note).
#:
#: **Widened 1.0 -> 1.5 s (R3-h1, 2026-09-13).** The FSM's 1.0 s measurement
#: above was against the FSM's own (looser) jerk ceiling; through this
#: schedule's real ACTIVATE-park -> P1 (-50, 0) mm move at the R3 session
#: limit (150 000 mm/s³) it refuses ``LIMIT_JERK`` at 177 241 mm/s³ (205 051
#: at 1.2 s, a non-monotonic solver artifact) -- CLEAN at 1.5 s and above
#: (``tests/hardware/skills_plan_bench.py --rehearse --pattern self-toss``,
#: 2026-09-13).
#: **It is now the FLOOR of a SIZED period, not the period (2026-09-18).**
#: :func:`floor_lift_s` grows it when the hand starts away from
#: :data:`~jugglebot.motion.skills.sites.REST_HAND_REV`, so this is exactly
#: what a schedule whose hand is already home gets — the common case, since
#: that is where the previous schedule's own REST left it.
FLOOR_LIFT_S = 1.5

#: Peak hand VELOCITY (rev/s) the opening REST's homing move is sized to.
#: ``JB_OP_GENTLE_MOVE_VEL_LIMIT_RPS`` — the rate the firmware's own profiled
#: park (ACTIVATE's TRAP_TRAJ on axis 6) brings the hand home at, so a homing
#: REST is never faster than the op the machine already trusts for this exact
#: move.  It is NOT the streaming lane's own ceiling (``JB_TRAJ_HAND_VEL_
#: LIMIT_RPS``, 200 rev/s): a healthy armed lane does follow at full C2 speed
#: (90 rev/s in a throw), but a homing move starts from a lane the firmware may
#: be HOLDING, and the slow regimes it can meet there are the ones below.
HOME_HAND_VEL_LIMIT_RPS = float(hw.JB_OP_GENTLE_MOVE_VEL_LIMIT_RPS)

#: Peak hand ACCELERATION (rev/s²) the same move is sized to — the firmware's
#: own recovery-slew onset ramp ``RECOVER_SLEW_ACCEL_RPS2``
#: (``canbridge_config.h:387``, 5.0 rev/s²; restated, not imported, because
#: ``motion/`` cannot read firmware headers).
#:
#: WHY AN ACCELERATION BOUND AT ALL, given the lane follows at 90 rev/s.  After
#: a hand-less hold — which EVERY attempt's end installs — the firmware hand
#: group sits in ``SCHED_HOLD`` at its last knot and promotes a resumed lane
#: only if the frame at the handover instant is within
#: ``SCHED_RESUME_TOL_POS_HAND_REV`` = 0.05 rev of the held state
#: (``leg_interp.cpp:497-520``, ``canbridge_config.h:426``); otherwise it
#: LATCHES the hold and the guard then measures the REFUSED command against the
#: encoder (``leg_interp.cpp:1226-1228``) until MAX_DEVIATION E-STOPS the
#: machine — the 2026-09-18 latch 1.  The first frame reaches the wire ~60-100
#: ms after the plan's origin (solve + one tick), so the lane must not have
#: moved 0.05 rev by then: ½·a·(0.1 s)² ≤ 0.05 rev ⇒ a ≤ 10 rev/s², and the
#: firmware's own 5 rev/s² ramp is the conservative half of that.
HOME_HAND_ACC_LIMIT_RPS2 = 5.0

#: Hand displacement (rev) inside which the homing move is no move at all:
#: :func:`floor_lift_s` already collapses to :data:`FLOOR_LIFT_S` well
#: outside it, so this constant exists for the ONE caller that has no opening
#: REST to grow (``compile_columns``, R4) and must refuse instead.  0.10 rev is
#: the bridge's own already-parked no-op band (``teensy_bridge_node.
#: _HAND_PARK_BAND_REV``) — 3.3 mm of carriage at ``hand_mm_per_rev`` 32.567.
HOME_BAND_REV = 0.10

#: The SETTLE window's hand-lane SHAPE factors: peak |v| ≈ ``_SHAPE_KV·Δ/T``
#: and peak |a| ≈ ``_SHAPE_KA·Δ/T²`` for a rest-to-rest move of Δ rev over T s.
#:
#: MEASURED off the real QP, not assumed (probe 2026-09-18, venv interpreter,
#: ``plan_segment(REST, ...)`` at the R3 session limits 300/5000/150000 mm +
#: hand 3500 rev/s², hand seeds 0.0 / 0.56 / 2.0 / 5.0 / 9.62 / 9.9 rev homing
#: to ``sites.REST_HAND_REV``): the realised factors are k_v = 1.50-1.53 and
#: k_a = 5.68-5.95, flat across Δ and T.  ``_SHAPE_KV`` = 15/8 = 1.875 is the
#: quintic rest-to-rest bound and so sizes CONSERVATIVELY (23 % slower than the
#: QP needs); ``_SHAPE_KA`` = 6.0 is the measured maximum rounded up, because
#: the quintic 10/√3 = 5.774 sits BELOW what the QP actually produces and would
#: have sized the acceleration-bound branch 3 % hot.
#:
#: Pinned by ``tests/motion/test_skills_segments.py::
#: test_the_opening_rest_homes_the_hand_inside_the_firmware_envelope``, which
#: samples the planned hand lane and asserts the peaks against the two limits
#: above — so a solver change that made the shape sharper fails a test rather
#: than an E-STOP.
_SHAPE_KV = 15.0 / 8.0
_SHAPE_KA = 6.0


def floor_lift_s(hand_rev_now: float,
                 default_s: float = FLOOR_LIFT_S) -> float:
    """The opening REST's period (s) that homes the hand from ``hand_rev_now``
    to :data:`~jugglebot.motion.skills.sites.REST_HAND_REV` inside the
    firmware's resume-and-follow envelope.

    ``max`` of three floors: the default lift (:data:`FLOOR_LIFT_S`,
    which is what a hand already at home gets), the velocity bound
    (:data:`HOME_HAND_VEL_LIMIT_RPS`) and the acceleration bound
    (:data:`HOME_HAND_ACC_LIMIT_RPS2`), through the measured shape factors
    above.  Worked example, the 2026-09-18 sitting's own hand: Δ = 9.6227 −
    0.3071 = 9.3156 rev ⇒ 1.875·9.3156/2.5 = 6.99 s (velocity-bound; the
    acceleration branch asks only 3.34 s) ⇒ a 7.0 s opening REST, whose
    realised peaks are 2.01 rev/s and 1.14 rev/s² (probe, same day).

    PURE: no clock, no ROS, no state — the node passes the measured hand
    position in and gets the period out (``SelfTossPattern.floor_lift_s``).
    """
    delta = abs(float(hand_rev_now) - float(REST_HAND_REV))
    return max(float(default_s),
               _SHAPE_KV * delta / HOME_HAND_VEL_LIMIT_RPS,
               math.sqrt(_SHAPE_KA * delta / HOME_HAND_ACC_LIMIT_RPS2))


def home_hand_bounds(hand_rev_now: float, t_rest_s: float):
    """``(peak |v| rev/s, peak |a| rev/s²)`` the homing move of
    :func:`floor_lift_s` is BOUNDED by over ``t_rest_s`` — the shape factors
    read forwards, for the node's one INFO line.  Bounds, not predictions: the
    QP's realised peaks come in ~20 % under the velocity one (see
    :data:`_SHAPE_KV`)."""
    delta = abs(float(hand_rev_now) - float(REST_HAND_REV))
    t = float(t_rest_s)
    if not t > 0.0:
        raise ValueError('t_rest_s must be > 0, got %r' % (t_rest_s,))
    return (_SHAPE_KV * delta / t, _SHAPE_KA * delta / (t * t))


@dataclasses.dataclass(frozen=True)
class OneBallPattern:
    """One-ball pattern parameters (R4 — plan § 0, owner decision D1
    2026-09-23): ``sites`` ONE long is R3's self-toss exactly (bit-identical
    schedule — see :func:`compile_one_ball`); ``sites`` TWO long is R4's
    alternating hop, one ball crossing the two sites (a THROW released at
    ``sites[i % 2]`` lands at ``sites[(i + 1) % 2]``).  Replaces
    ``SelfTossPattern`` (R3) — no second name for the same compiler input;
    every live caller now builds a 1-tuple where it used to build a bare
    ``site`` (grep swept to zero, R4 U1).

    One ball, ``len(sites)`` sites: THROW from rest, then
    CATCH-with-``then_throw`` chained ``n_throws - 1`` times, then a
    standalone CATCH, then REST. Mirrors :class:`Pattern`'s fields exactly
    (``sites`` here is 1 or 2 long rather than exactly 2).
    """

    sites: Tuple[Site, ...]
    apex_m: float
    dwell_s: float
    n_throws: int
    #: THROW 0's window, from rest — see :class:`Pattern`.
    launch_s: float = 0.4
    #: The SETTLE tail every THROW/CATCH segment carries.
    rest_tail_s: float = sg.REST_TAIL_S
    #: The OPENING REST's period (s) — the window that HOMES the hand from
    #: wherever it actually is to :data:`~jugglebot.motion.skills.sites.
    #: REST_HAND_REV`.  The node sizes it from the measured hand through
    #: :func:`floor_lift_s`; the default is the already-home case
    #: (:data:`FLOOR_LIFT_S`), which is also what every offline test and the
    #: sim gate get.
    floor_lift_s: float = FLOOR_LIFT_S

    def __post_init__(self):
        if len(self.sites) not in (1, 2):
            raise ValueError(
                'OneBallPattern takes 1 or 2 sites, got %d — no third site '
                'exists in this plan (R4, owner decision D1 2026-09-23)'
                % (len(self.sites),))
        if not float(self.floor_lift_s) >= FLOOR_LIFT_S:
            raise ValueError(
                'floor_lift_s %r is below the %.3f s floor — the opening REST '
                'both lifts the cup onto the site and homes the hand, and '
                'FLOOR_LIFT_S is the measured minimum for the lift alone'
                % (self.floor_lift_s, FLOOR_LIFT_S))


def compile_one_ball(pattern: OneBallPattern, t0_abs_s: float) -> Schedule:
    """The R4 one-ball schedule: self-toss (one site) or the alternating hop
    (two — plan § 0, owner decision D1 2026-09-23). Replaces
    ``compile_self_toss`` (R3) — one compiler, not two, for what is one
    family of schedules (site-cycling with the count of sites as the only
    difference).

    One ball, ``pattern.sites`` cycled: an OPENING REST
    (``pattern.floor_lift_s``) lifts the cup onto ``sites[0]`` AND HOMES THE
    HAND from wherever it actually is to
    :data:`~jugglebot.motion.skills.sites.REST_HAND_REV` — one window, one
    home, sized by :func:`floor_lift_s` so the hand lane stays inside the
    firmware's resume-and-follow envelope (:data:`HOME_HAND_VEL_LIMIT_RPS` /
    :data:`HOME_HAND_ACC_LIMIT_RPS2`).  Then THROW 0 is a fresh origin from
    that rest (``launch_s`` window), then every later throw is CARRIED by the
    catch one dwell before it (:func:`_fold_catch_throw_pairs` — the same
    measured reason :class:`ThenThrow` gives for columns: solved apart, the
    catch's runway decelerates the hand toward rest and the throw has to undo
    that with ``dwell - lead`` left). The last throw's catch is standalone
    (nothing scheduled after it at this rung), and the schedule ends with a
    REST at the site of that last catch, two ``rest_tail_s`` after it — the
    same reason :func:`compile_columns` gives: one tail would dispatch the
    REST before the touch-down and splice the catch away.

    **Throw ``i`` releases at** ``sites[i % len(sites)]`` **and targets**
    ``sites[(i + 1) % len(sites)]``.  With one site ``(i + 1) % 1 == 0``
    always, so the target is always the same site THROW 0 releases from —
    R3's self-toss, and every number below (the fold, the lead assignment,
    the dispatch-monotonicity check) runs the identical arithmetic it always
    did, so a one-site schedule is BIT-IDENTICAL to the pre-R4
    ``compile_self_toss`` (pinned,
    ``tests/motion/test_skills_schedule.py::
    test_one_site_one_ball_schedule_matches_the_pre_r4_compile_self_toss``).
    With two sites this is the HOP (plan owner decision D1, 2026-09-23): the
    cup coasts under the ball at the ball's lateral speed (117 mm/s at
    100 mm separation / 292 mm/s at 250 mm, both apex 0.9 m) rather than the
    platform chasing it, because the QP's release equality already pins the
    cup velocity to the ballistic launch velocity (probed 2026-09-23: release
    cup velocity (117, 0, 4201) mm/s == the ballistic launch for the 100 mm
    hop) — no separate aim compensation exists or is needed. The fold
    (:func:`_fold_catch_throw_pairs`) still pairs CATCH ``i`` with THROW
    ``i + 1`` at the SAME site (the throw out of ``sites[1]`` IS at
    ``sites[1]``, since target ``i`` == release site ``i + 1`` by
    construction): CATCH ``i``'s site is ``sites[(i + 1) % 2]`` and THROW
    ``i + 1``'s site is ``sites[(i + 1) % 2]`` too, and CATCH ``i``'s
    ``t_abs_s + dwell_s`` equals THROW ``i + 1``'s ``t_abs_s`` exactly (both
    derived from the same ``period``) — so the existing predicate (same
    ``ball_id``, same ``site``, release ``dwell_s`` after touch-down) folds a
    two-site schedule with no change.

    Throw ``i`` releases at ``floor_lift_s + i * (t_f + dwell_s)`` — ONE
    ball, so the throw-to-throw period is the WHOLE cycle (``t_f +
    dwell_s``), not ``beat_s``'s half-cycle (that halving is a property of
    two BALLS alternating out of phase, plan § 2.4's columns, and does not
    apply to a single ball crossing sites). Every CATCH's window is the full
    flight ``t_f`` (its splice base is its own previous release, exactly as
    ``tau`` is for columns, so it gets :data:`HANDOFF_LEAD_S` via
    :func:`_assign_leads` the same way). ``Schedule.beat_s`` is filled with
    this period (matching
    ``toss_session.TossSessionSequencer.beat_s = flight_time_s + dwell_time_s``,
    the FSM's own name for a single ball's throw-to-throw period) and
    ``Schedule.transit_s`` with ``t_f`` (the CATCH window, honestly — there is
    no "other hand" here for ``transit_s``'s columns meaning to describe).

    **Why THROW 0 lands at ``floor_lift_s + launch_s``, not ``floor_lift_s``.**
    The opening REST is a fresh install (no record yet) that plans a SETTLE
    ending EXACTLY at its own event instant (``floor_lift_s`` — a REST segment
    carries no extra tail, so ``record.end_s`` is exactly the terminal's event
    time regardless of dispatch jitter). THROW 0 must dispatch no earlier than
    that: with ``t_abs_s = floor_lift_s + launch_s``, ``window_s = launch_s``
    and the general :data:`LEAD_S`, ``THROW 0.dispatch_s() = floor_lift_s -
    LEAD_S``, so ``install_segment``'s fresh test (``t_now + lead >=
    record.end_s``) reads ``(floor_lift_s - LEAD_S) + LEAD_S >= floor_lift_s``
    — true by construction, not by margin. Putting THROW 0 at ``floor_lift_s``
    itself would dispatch it while the lift is still mid-settle and force a
    SPLICE onto a plan that has not reached the floor yet.
    """
    n = int(pattern.n_throws)
    if n < 1:
        raise ValueError('n_throws must be >= 1, got %r' % (pattern.n_throws,))
    n_sites = len(pattern.sites)

    # THE SCHEDULE IS BUILT ON ITS OWN CLOCK AND SHIFTED TO t0_abs_s ONCE, AT
    # THE END — see the identical note in compile_columns for why (ROS-epoch
    # doubles only resolve ~2.4e-7 s, and every fold/lead/monotonicity check
    # below compares instants to 1e-9 s).
    t0_rel = 0.0

    lift_s = float(pattern.floor_lift_s)
    _check_window('the opening REST (the floor lift + the hand homing)', lift_s)
    skills = [Skill(kind=REST, ball_id=0, site=pattern.sites[0],
                    t_abs_s=t0_rel + lift_s, window_s=lift_s)]

    # + launch_s: THROW 0's release cannot land AT the floor instant itself
    # (the opening REST's plan is still mid-settle there) — see the
    # docstring's "Why THROW 0 lands at floor_lift_s + launch_s". The shared
    # tail (:func:`_append_one_ball_pattern`) takes the REST's own end
    # instant and adds this same offset for ANY caller, reload's DECAY REST
    # included.
    skills, t_f, period = _append_one_ball_pattern(
        skills, t0_rel + lift_s, pattern.sites, pattern.apex_m,
        pattern.dwell_s, n, pattern.launch_s, pattern.rest_tail_s)

    skills = _assign_leads(skills)
    _check_dispatch_monotone(skills)

    t0 = float(t0_abs_s)
    skills = [_shifted(s, t0) for s in skills]
    return Schedule(skills=tuple(skills), flight_s=t_f, beat_s=period,
                    transit_s=t_f, dwell_s=float(pattern.dwell_s), t0_abs_s=t0,
                    pattern=('self_toss' if n_sites == 1 else 'hop'))


def _append_one_ball_pattern(skills, rest_end_t_rel: float, sites, apex_m: float,
                             dwell_s: float, n_throws: int, launch_s: float,
                             rest_tail_s: float):
    """Append the one-ball pattern's THROW/CATCH events to ``skills`` (already
    ending in a REST at ``rest_end_t_rel``, on the SAME relative clock),
    fold same-site (CATCH, THROW) pairs, and append the closing REST.

    THE SHARED TAIL of :func:`compile_one_ball` (plain self-toss/hop, whose
    ``skills`` prefix is just the opening REST) and :func:`compile_reload`
    (R4: whose prefix is PRE-TILT REST -> held CATCH -> DECAY REST — see that
    function).  Lifted out unchanged from :func:`compile_one_ball` (R3/R4
    build note) rather than duplicated (plan § 0): same arithmetic, same
    fold, same lead assignment either way, because past the last REST that
    precedes it, a one-ball pattern does not know or care how its own ball
    got into the cup.

    Returns ``(skills, t_f, period)`` — ``skills`` is still on the t0_rel=0
    clock, not yet lead-assigned, monotonicity-checked or shifted (the
    caller's job, since :func:`_assign_leads` and the dispatch check must run
    over the WHOLE schedule, prefix included).
    """
    t_f = flight_s(apex_m)
    n = int(n_throws)
    period = t_f + float(dwell_s)
    n_sites = len(sites)

    _check_window('THROW 0 (the launch from rest)', launch_s)
    if n >= 2:
        _check_window('dwell (touch-down to the throw the catch carries)',
                      dwell_s)
    _check_window('CATCH (the full flight to the target site)', t_f)

    for i in range(n):
        site_i = sites[i % n_sites]
        target_i = sites[(i + 1) % n_sites]
        t_throw = float(rest_end_t_rel) + float(launch_s) + float(i) * period
        window = launch_s if i == 0 else dwell_s
        skills.append(Skill(kind=THROW, ball_id=0, site=site_i,
                            t_abs_s=t_throw, window_s=window,
                            y_d=(np.zeros(2), apex_m), target=target_i))
        # Unlike columns (whose LAST throw's landing is deliberately left
        # unscheduled for R5's cone delivery), a one-ball schedule has
        # nowhere else for the ball to go: EVERY throw gets a catch, and the
        # fold below turns all but the very last one into a carried
        # then_throw.
        skills.append(Skill(kind=CATCH, ball_id=0, site=target_i,
                            t_abs_s=t_throw + t_f, window_s=t_f))

    skills.sort(key=lambda s: s.t_abs_s)
    skills = _fold_catch_throw_pairs(skills, float(dwell_s))

    catches = [s for s in skills if s.kind == CATCH]
    last_catch = max(catches, key=lambda s: s.t_abs_s)
    # TWO tails — same reason as compile_columns: one would dispatch the REST
    # before the touch-down and splice the still-in-flight catch away.
    rest_t = closing_rest_t_abs(last_catch.t_abs_s, rest_tail_s)
    skills.append(Skill(kind=REST, ball_id=last_catch.ball_id,
                        site=last_catch.site, t_abs_s=rest_t,
                        window_s=rest_tail_s))
    return skills, t_f, period


def _check_dispatch_monotone(skills) -> None:
    """Dispatch instants must be non-decreasing across ``skills`` — shared by
    every compiler (:func:`compile_columns`, :func:`compile_one_ball`,
    :func:`compile_reload`) rather than three copies of the same loop."""
    disp = [s.dispatch_s() for s in skills]
    for k in range(1, len(disp)):
        if disp[k] < disp[k - 1] - 1e-9:
            raise ValueError(
                'skill %d (%s at %.4f s, window %.4f s, lead %.3f s) must '
                'install before skill %d (%s at %.4f s, window %.4f s, lead '
                '%.3f s) installs, but dispatches later (%.4f s vs %.4f s) — '
                'the two windows overlap'
                % (k - 1, skills[k - 1].kind, skills[k - 1].t_abs_s,
                   skills[k - 1].window_s, skills[k - 1].lead_s,
                   k, skills[k].kind, skills[k].t_abs_s, skills[k].window_s,
                   skills[k].lead_s, disp[k - 1], disp[k]))


#: Seconds the PRE-TILT REST's own transition takes (level -> the receive
#: attitude) and the DECAY REST's takes (the receive attitude -> level) — the
#: FSM's proven choreography (plan owner decision D4, 2026-09-23), restated
#: as ONE constant each because both are a PHYSICAL fact about the
#: platform's own tilt-rate limit, not a schedule choice.
#:
#: PROBED (U2, ``scratchpad/probe_r4_bb_catch4.py``, 2026-09-23, venv, R4
#: operating point: leg 300/5000/150000 mm, hand acc 3500 rev/s²): the
#: shortest of {1.0, 1.5, 2.0} s that passes ``validate_cycle`` for a
#: level -> 12° (the receive-tilt ceiling) transition is **1.0 s** — U2's
#: handoff has the full three-period sweep (peak tilt accel 1.21 rad/s² at
#: 1.0 s).
PRETILT_S = 1.0
DECAY_S = 1.0

#: Seconds the reload CATCH's own window (its splice base — the pre-tilt
#: REST's end — to the announced touch-down) must be at least.
#:
#: This is OPERATING MARGIN for the heavier held-axis solve, not what makes
#: the catch a fresh origin (any window does that, by construction — see
#: :func:`compile_reload`'s docstring).  U2 measured the held-axis LANDING at
#: ~5x a level LANDING's per-iteration solve cost under load (108 equality
#: rows vs 12), ~0.09 s unloaded either way (handoff_U2.md, "held-axis QP
#: cost").  Sized like :class:`OneBallPattern`'s ``launch_s`` (0.4 s, THROW
#: 0's own fresh-origin window) with headroom for that factor.
RELOAD_CATCH_WINDOW_S = 0.5


def compile_reload_wait(site: Site, floor_lift_s: float,
                        t0_abs_s: float, holds_ball: bool = False) -> Schedule:
    """The R4 reload's OPENING REST bridge (owner decision D4, brief step
    1a) — installed BEFORE Ball Butler is even asked to reload, because the
    receive tilt :func:`compile_reload` needs is not known until BB's
    ``ThrowAnnouncement`` supplies the arrival velocity. ONE REST: bit-
    identical Skill construction to :func:`compile_one_ball`'s own opening
    REST (homes the hand from wherever it is to
    :data:`~jugglebot.motion.skills.sites.REST_HAND_REV`, lifts the cup onto
    ``site``), except ``holds_ball`` defaults ``False`` — unlike the
    self-toss opening REST (which assumes an operator has already placed a
    ball), nothing is in the cup while BB's throw is still pending.

    ``SkillNode._run_one_ball`` (a Juggle goal with ``reload=True``) installs this
    schedule, then calls ``bb/reload`` + ``bb/throw_at_target``; the hand
    is already homing while that round trip is in flight. On the
    announcement, ``SkillNode._on_announcement`` compiles the real
    :func:`compile_reload` schedule and SWAPS the executor — by which point
    the hand is already at rest, so that schedule's own combined home+tilt
    REST (U2's SIMPLICITY contract) has nothing left to home and only
    reaches the tilt. This schedule's only job is to get there and hold.

    ``holds_ball=True`` (R5, owner decision D1, 2026-09-30): the SAME bridge
    shape reused as ``SkillNode._run_columns``'s opening REST — columns'
    ball A is already seated in the cup (an operator placed it, exactly
    like self-toss's opening REST) while ball B's feed is still awaited, so
    the bridge must say so, or the executor's own "what does this REST
    carry" bookkeeping would read the cup as empty while a ball sits in it.
    """
    t0_rel = 0.0
    period = float(floor_lift_s)
    _check_window('the reload bridge REST (hand homing while BB is asked to '
                 'reload)', period)
    skills = [Skill(kind=REST, ball_id=0, site=site,
                    t_abs_s=t0_rel + period, window_s=period,
                    holds_ball=bool(holds_ball))]
    skills = _assign_leads(skills)
    t0 = float(t0_abs_s)
    skills = [_shifted(s, t0) for s in skills]
    return Schedule(skills=tuple(skills), flight_s=0.0, beat_s=period,
                    transit_s=0.0, dwell_s=0.0, t0_abs_s=t0,
                    pattern='self_toss')


def compile_reload(landing_mm, landing_vel_mm_s, t_land_abs_s: float,
                   pattern: OneBallPattern, t0_abs_s: float,
                   max_tilt_deg: float = tg.MAX_TILT_DEG) -> Schedule:
    """The R4 reload schedule (plan owner decision D4, 2026-09-23): the FSM's
    proven ball-butler-reload choreography, expressed as skills, anchored on
    Ball Butler's OWN ``ThrowAnnouncement`` — ``landing_mm`` /
    ``landing_vel_mm_s`` / ``t_land_abs_s`` are that announcement's physics,
    already in the schedule frame (the caller subtracts the measured
    mocap-to-schedule offset — plan § 1).  The announcement is the
    AUTHORITY: this compiler is called once BB has announced, never before
    (the brief's step 1c) — there is no earlier instant this schedule could
    be built from, because the pre-tilt attitude is DERIVED from the
    announced arrival velocity (:func:`~jugglebot.motion.skills.segments.
    receive_hold_tilt`).

    **PRE-TILT REST** (level -> the receive attitude, over
    ``max(pattern.floor_lift_s, PRETILT_S)`` — ONE REST that both homes the
    hand and reaches the tilt, U2's "SIMPLICITY" contract: no scenario in
    U2's probe needed a separate homing REST ahead of it, and a second REST
    is a second fresh-origin gate for no measured benefit) at
    ``segments.hold_axis_site(landing_mm, tilt, sites.REST_CUP_Z_MM)`` — ON
    the held axis line through the announced landing, not under
    ``pattern.sites[0]``'s own xy (U2 cross-unit contract: 29.8 mm apart at
    the 12° ceiling) —
    **-> held-tilt CATCH** (a standalone LANDING, ``hold_tilt=tilt``,
    terminal = the announced landing, carrying :class:`LandingPrior` so the
    executor has an aim before any tracker fit exists for this
    externally-thrown ball) —
    **-> DECAY REST** (the same tilt back to level, over :data:`DECAY_S`, at
    ``pattern.sites[0].rest_site_mm()`` — level, so the ordinary site rest
    point already is on the (degenerate, vertical) axis) —
    **-> the one-ball pattern's own throws** from that level rest
    (:func:`_append_one_ball_pattern` — the SAME tail :func:`compile_one_ball`
    builds; one compiler body, not two, plan § 0).

    **Why the reload CATCH is a FRESH origin, not a splice** — mirrors
    :func:`compile_one_ball`'s "Why THROW 0 lands at floor_lift_s +
    launch_s".  The PRE-TILT REST is a plain SETTLE with no extra tail, so
    its ``record.end_s`` is exactly its own event instant.  Setting the
    CATCH's ``window_s`` to exactly ``t_land - pretilt_rest.t_abs_s`` (as
    below) puts its splice base EXACTLY on that event, so
    ``install_segment``'s fresh test (``t_now + lead >= record.end_s``)
    reads true by construction at the catch's own dispatch instant, for ANY
    window size — :data:`RELOAD_CATCH_WINDOW_S` is solve-budget margin for
    the heavier held-axis QP (U2), not what makes the test pass.

    ``pattern.sites`` names the site the reload catches at and the one-ball
    pattern that follows it plays — :class:`OneBallPattern`, unchanged (R4
    plays one site after a reload).  ``t0_abs_s`` plays the SAME role it does
    in :func:`compile_one_ball` (the instant the caller — ``SkillNode.
    _on_announcement`` — received the announcement; the schedule's own clock
    is built at ``t0_rel = 0`` and shifted once at the end, per the identical
    note on :func:`compile_columns`), not a free parameter.

    ``max_tilt_deg`` (R5 owner experiment, 2026-09-30, default
    :data:`tilt_geometry.MAX_TILT_DEG`, 12°): the ceiling the PRE-TILT REST's
    ``rest_tilt`` and the CATCH's ``hold_tilt`` are BOTH clamped to — the one
    derivation point (:func:`tilt_geometry.tilt_to_receive`) both read, so a
    tighter cap can never desync the REST's approach from what the CATCH
    actually holds. Called here rather than through
    ``segments.receive_hold_tilt`` only because that wrapper hardcodes the
    12° default and has no cap argument — the underlying derivation is the
    same function either way, so this is not a second copy of it. Must be ``0 < max_tilt_deg <= tilt_geometry.
    MAX_TILT_DEG``: a cap of 0 could never hold an off-vertical arrival at
    all (nothing to catch on the line), and one past 12° would exceed the
    ceiling the QP's own held-axis contract was measured against
    (``segments.receive_hold_tilt``'s docstring).
    """
    landing_mm = _vec3(landing_mm, 'landing_mm')
    landing_vel = _vec3(landing_vel_mm_s, 'landing_vel_mm_s')
    if not np.isfinite(float(t_land_abs_s)):
        raise ValueError('t_land_abs_s must be finite, got %r'
                          % (t_land_abs_s,))
    max_tilt_deg = float(max_tilt_deg)
    if not (0.0 < max_tilt_deg <= tg.MAX_TILT_DEG):
        raise ValueError(
            'max_tilt_deg must be in (0, %.1f], got %r'
            % (tg.MAX_TILT_DEG, max_tilt_deg))
    tilt = tg.tilt_to_receive(landing_vel, max_tilt_deg=max_tilt_deg)
    site0 = pattern.sites[0]

    t0_rel = 0.0
    t_land_rel = float(t_land_abs_s) - float(t0_abs_s)

    lift_s = float(pattern.floor_lift_s)
    pretilt_period = max(lift_s, PRETILT_S)
    _check_window('the opening REST (the floor lift + the pre-tilt attitude)',
                 pretilt_period)
    # The PRE-TILT REST ends LEAD_S before the CATCH's own window opens, so the
    # CATCH (window RELOAD_CATCH_WINDOW_S, dispatch = t_abs - window - LEAD_S)
    # dispatches exactly AT the REST's end. Until 2026-09-29 the two abutted,
    # so the CATCH dispatched LEAD_S (225 ms) before the REST ended: the fresh
    # test already reads true there, but trajectory_node seeds a fresh install
    # from the ACTIVE PLAN's commanded state at t_now -- still mid-slew, off
    # the held-axis line -- and every sitting-3 reload CATCH was refused
    # CATCH_AXIS (5.681 / 1.065 mm vs 0.1 mm) with the hand parked at the
    # bottom of its stroke. The catch MOTION is no longer the 0.725 s
    # (window + lead) it flew until 2026-10-02: on the live path
    # (trajectory_node, `install_segment(..., reserve_fresh_lead=True)`) a
    # fresh CATCH's origin sits LEAD_S after its dispatch, so the machine
    # HOLDS the pre-tilt rest for that lead (the solve's budget) and the catch
    # plans over RELOAD_CATCH_WINDOW_S (0.5 s) plus at most skill_node's
    # dispatch look-ahead (_DISPATCH_LOOKAHEAD_S, 0.050 s). Offline callers
    # that omit the flag still plan window + lead.
    pretilt_end_rel = t_land_rel - RELOAD_CATCH_WINDOW_S - LEAD_S
    pretilt_dispatch_rel = pretilt_end_rel - pretilt_period - LEAD_S
    if pretilt_dispatch_rel < t0_rel - 1e-9:
        raise ValueError(
            'the announced landing (%.3f s from now) leaves no room for the '
            'opening REST (%.3f s) + the reload CATCH window (%.3f s) + '
            'dispatch lead (%.3f s) ahead of it — BB\'s throw_delay_s must '
            'clear all three before the throw is announced'
            % (t_land_rel - t0_rel, pretilt_period, RELOAD_CATCH_WINDOW_S,
               LEAD_S))

    # Both the PRE-TILT REST and the reload CATCH settle ON THE HELD AXIS
    # LINE through the announced landing (U2 cross-unit contract) — ONE
    # derivation, reused for both skills rather than recomputed per site.
    pretilt_site = sg.hold_axis_site(landing_mm, tilt, REST_CUP_Z_MM)
    skills = [Skill(kind=REST, ball_id=0, site=site0,
                    t_abs_s=pretilt_end_rel, window_s=pretilt_period,
                    rest_tilt=tilt, rest_site_mm=pretilt_site,
                    holds_ball=False)]

    catch_window = t_land_rel - pretilt_end_rel - LEAD_S
    _check_window('the reload CATCH (pre-tilt REST end + lead to touch-down)',
                 catch_window)
    skills.append(Skill(
        kind=CATCH, ball_id=0, site=site0, t_abs_s=t_land_rel,
        window_s=catch_window, hold_tilt=tilt, rest_site_mm=pretilt_site,
        # t_land_abs_s carries the ORIGINAL absolute instant, not the
        # relative one `_shifted` below would double-shift — this is the
        # only Skill field `_shifted` does not know how to move, so it is
        # built already on the wall clock it will be read on.
        landing_prior=LandingPrior(pos_mm=landing_mm, vel_mm_s=landing_vel,
                                   t_land_abs_s=float(t_land_abs_s))))

    # The DECAY REST is a FRESH ORIGIN from the tilted rest the CATCH's runway
    # ends at -- its splice base clears the catch segment's end (touch-down +
    # rest_tail) by REST_FRESH_MARGIN_S, exactly as `closing_rest_t_abs` does
    # for a closing REST.  MEASURED 2026-09-23 (`sim/skills_gate.py --reload`,
    # seeds 0-4, MuJoCo catch): with the decay's base AT the touch-down instant
    # it spliced into the runway one knot after the seat and the seam's
    # downward cup acceleration read -7049 mm/s^2 against the -0.70 g
    # CUP_CONTACT_ACC floor (-6864), refusing every seed; from rest the decay
    # is a pure attitude slew over a seated ball (U2 probe (iii): leg vel
    # 70.8 mm/s, jerk 10 450 at 1.0 s).
    decay_start_rel = t_land_rel + float(pattern.rest_tail_s) + REST_FRESH_MARGIN_S
    decay_end_rel = decay_start_rel + DECAY_S
    _check_window('the DECAY REST (the receive attitude back to level)',
                 DECAY_S)
    skills.append(Skill(kind=REST, ball_id=0, site=site0,
                        t_abs_s=decay_end_rel, window_s=DECAY_S,
                        rest_tilt=(0.0, 0.0)))

    n = int(pattern.n_throws)
    if n < 1:
        raise ValueError('n_throws must be >= 1, got %r' % (pattern.n_throws,))
    skills, t_f, period = _append_one_ball_pattern(
        skills, decay_end_rel, pattern.sites, pattern.apex_m, pattern.dwell_s,
        n, pattern.launch_s, pattern.rest_tail_s)

    skills = _assign_leads(skills)
    _check_dispatch_monotone(skills)

    t0 = float(t0_abs_s)
    skills = [_shifted(s, t0) for s in skills]
    # NOT a 'reload' label — see the field docstring on `Schedule.pattern`:
    # every THROW this schedule dispatches is kinematically an ordinary
    # self-toss/hop throw, and that is the box actually swept.
    n_sites = len(pattern.sites)
    return Schedule(skills=tuple(skills), flight_s=t_f, beat_s=period,
                    transit_s=t_f, dwell_s=float(pattern.dwell_s), t0_abs_s=t0,
                    pattern=('self_toss' if n_sites == 1 else 'hop'))


def phantom_feed_prior(pattern: Pattern, aim_site: Site,
                       t0_abs_s: float) -> LandingPrior:
    """The one landing a ``columns_1ball`` phantom's FIRST catch is aimed at
    (:attr:`Pattern.phantom_balls`, B2): a plain vertical arrival at
    ``aim_site``'s catch point, at the pattern's own flight speed, landing
    exactly one transit after ``t0_abs_s`` -- so :func:`compile_columns`'s
    ``feed=`` branch builds the phantom start with the SAME skill the fed
    start uses (the level-pinned, aim-walked first catch) rather than the
    ``t0_abs_s`` branch's auto-banked catch at the nominal site.

    Why not the nominal site: MEASURED 2026-10-04 (`sim/skills_gate.py
    --one-ball`, apex 0.95 / dwell 0.25 / separation 125): the phantom's
    first catch at the nominal site refused LIMIT_JERK on every seed (peak
    212 379 mm/s^3 against 200 000, 6 % over), while the fed start -- whose
    first catch is walked ``columns_feed_aim_toward_a_mm`` toward ball A
    and pinned level -- passed 5/5 at the same geometry, and so did two
    real balls. The one-ball pattern exists to rehearse the FED start's
    motion with ball A alone, so its first catch takes the fed start's
    aim point and attitude; a vertical arrival is the easiest ball that
    catch can meet (no lateral velocity to match), so this prior is a
    lower bound on the real feed's demand, never an over-estimate.

    ``aim_site`` is whatever the caller feeds Ball Butler in the fed start
    (``skill_node._columns_feed_aim_site``); the SITE named by the skill
    stays ``pattern.sites[1]`` -- ``compile_columns`` reads the site from
    the pattern and only the landing from this prior.
    """
    if 1 not in tuple(pattern.phantom_balls):
        raise ValueError(
            'phantom_feed_prior is for columns_1ball only -- '
            'pattern.phantom_balls must name ball 1, got %r'
            % (pattern.phantom_balls,))
    t_f = flight_s(pattern.apex_m)
    tau = transit_s(t_f, pattern.dwell_s)
    pos_mm = np.asarray(aim_site.catch_site_mm(), dtype=float)
    launch_vel = ballistics_bc.launch_velocity(
        np.asarray(aim_site.throw_site_mm(), dtype=float), pos_mm, t_f)
    vel_mm_s = ballistics_bc.arrival_velocity(launch_vel, t_f)
    return LandingPrior(pos_mm=pos_mm, vel_mm_s=np.asarray(vel_mm_s, dtype=float),
                        t_land_abs_s=float(t0_abs_s) + tau)


def compile_reload_columns(landing_mm, landing_vel_mm_s, t_land_abs_s: float,
                           lift_s: float, columns_pattern: Pattern,
                           t0_abs_s: float,
                           max_tilt_deg: float = tg.MAX_TILT_DEG) -> Schedule:
    """B2's ``columns_1ball`` reload start (``SkillNode._run_columns_1ball``,
    ``reload=True``) — owner's words: "A is fed, held, then the pattern
    starts". Ball Butler's reload choreography for ball A — PRE-TILT REST
    -> held-tilt CATCH -> DECAY REST — hands off into the COLUMNS schedule
    (:func:`compile_columns`, ball A self-tossing from ``columns_pattern.
    sites[0]``, a phantom from ``sites[1]`` — :attr:`Pattern.phantom_balls`)
    rather than :func:`compile_reload`'s own one-ball tail
    (:func:`_append_one_ball_pattern`): this is columns motion with one real
    ball, not a second self-toss reload.

    **Reuses** :func:`compile_reload` **itself** for the lead-in, via a
    throwaway ``OneBallPattern(sites=(columns_pattern.sites[0],),
    n_throws=1, ...)`` that is never surfaced, rather than duplicating its
    PRE-TILT/CATCH/DECAY timing derivation: that function is delicately
    measured (its own docstring cites probe measurements for every margin
    in that lead-in), and restating it here would be a second copy that
    could silently drift from it. ``lead_in.skills[:3]`` is exactly
    ``[PRE-TILT REST, CATCH, DECAY REST]`` — the one-ball-pattern-agnostic
    prefix every reload schedule shares, asserted below rather than assumed
    silently — and everything :func:`_append_one_ball_pattern` appends
    after it (the throwaway THROW/CATCH/closing REST) is discarded.

    The columns schedule is then compiled fresh, anchored at the DECAY
    REST's own (already-absolute) end instant: that is exactly where it
    leaves ball A at rest, level, at ``columns_pattern.sites[0]`` —
    :func:`compile_columns`'s own assumption for ``t0_abs_s`` ("ball A is in
    the hand at site 0, about to be thrown").

    MEASURED (probe, 2026-10-04, default apex/dwell/launch_s): the DECAY
    REST's dispatch lands 0.6 s before the columns schedule's own THROW 0
    dispatch (:data:`DECAY_S` 1.0 s vs. the pattern's ``launch_s`` 0.4 s) —
    comfortable margin. :func:`_check_dispatch_monotone` runs again below
    over the COMBINED list regardless (never assumed from that one
    measurement), so a non-default ``launch_s`` — or a future change to
    either constant — gets the same hard refusal every other compiler gives
    rather than a silently mis-ordered schedule.
    """
    if not columns_pattern.phantom_balls:
        raise ValueError(
            'compile_reload_columns is for columns_1ball only -- '
            'columns_pattern.phantom_balls must name a phantom ball, got %r'
            % (columns_pattern.phantom_balls,))
    lead_in_pattern = OneBallPattern(sites=(columns_pattern.sites[0],),
                                     apex_m=columns_pattern.apex_m,
                                     dwell_s=columns_pattern.dwell_s,
                                     n_throws=1, launch_s=columns_pattern.launch_s,
                                     rest_tail_s=columns_pattern.rest_tail_s,
                                     floor_lift_s=float(lift_s))
    lead_in = compile_reload(landing_mm, landing_vel_mm_s, t_land_abs_s,
                             lead_in_pattern, t0_abs_s, max_tilt_deg=max_tilt_deg)
    prefix = lead_in.skills[:3]
    if tuple(s.kind for s in prefix) != (REST, CATCH, REST):
        raise AssertionError(
            'compile_reload\'s own lead-in shape changed -- '
            'compile_reload_columns assumes its first three skills are '
            'exactly [PRE-TILT REST, CATCH, DECAY REST], got %r'
            % (tuple(s.kind for s in prefix),))
    decay_end_abs = float(prefix[-1].t_abs_s)
    # `compile_columns`'s t0 is ball A's RELEASE instant (THROW 0 sits at
    # t0 with its `launch_s` window BEFORE it), whereas `_append_one_ball_
    # pattern` -- the self-toss reload's own tail -- releases `launch_s`
    # AFTER the rest end. Anchoring columns at the decay end itself put the
    # launch window inside the decay slew: MEASURED 2026-10-04 (`sim/
    # skills_gate.py --one-ball --one-ball-reload`, every seed) THROW 0
    # spliced at 0.8 s of the 1.0 s DECAY REST and refused LIMIT_JERK at
    # 421 223 mm/s^3. The release goes one launch window after the decay
    # end, exactly where the self-toss reload puts its first release.
    columns_sched = compile_columns(
        columns_pattern,
        t0_abs_s=decay_end_abs + float(columns_pattern.launch_s))
    combined = prefix + columns_sched.skills
    _check_dispatch_monotone(combined)
    return Schedule(skills=combined, flight_s=columns_sched.flight_s,
                    beat_s=columns_sched.beat_s,
                    transit_s=columns_sched.transit_s,
                    dwell_s=columns_sched.dwell_s, t0_abs_s=float(t0_abs_s),
                    pattern='columns', phantom_balls=columns_pattern.phantom_balls)


def _shifted(sk: Skill, t0: float) -> Skill:
    """``sk`` with every absolute instant it carries moved by ``t0``.

    The one place a schedule's own clock meets the wall clock (see the note at
    the top of :func:`compile_columns`): every comparison between instants has
    already been made on the schedule's clock, where a double is exact to
    ~1e-15 s.
    """
    tt = sk.then_throw
    return dataclasses.replace(
        sk, t_abs_s=float(sk.t_abs_s) + t0,
        then_throw=(None if tt is None else dataclasses.replace(
            tt, t_release_abs_s=float(tt.t_release_abs_s) + t0)))
