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


def beat_s(flight_s_: float, dwell_s: float) -> float:
    """Time between successive throws from the SAME hand: ``(t_f + d) / 2``."""
    return (float(flight_s_) + float(dwell_s)) / 2.0


def transit_s(flight_s_: float, dwell_s: float) -> float:
    """Time between a throw and the OTHER hand's next catch: ``(t_f − d) / 2``."""
    return (float(flight_s_) - float(dwell_s)) / 2.0


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
    #: Seconds between this skill's DISPATCH and its splice base
    #: (``t_abs_s - window_s``).  :func:`compile_columns` sets it per skill;
    #: the default is the general :data:`LEAD_S`.
    lead_s: float = LEAD_S

    def __post_init__(self):
        if self.kind not in KINDS:
            raise ValueError('kind must be one of %s, got %r'
                              % (KINDS, self.kind))
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
    """Columns-pattern parameters (columns only at R2 — plan § 1.2)."""

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

    def due(self, t_abs_s: float) -> Tuple[Skill, ...]:
        """Skills whose dispatch instant has passed by ``t_abs_s``."""
        return tuple(s for s in self.skills
                     if s.dispatch_s() <= float(t_abs_s))


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
            t_release_abs_s=throw.t_abs_s, y_d=throw.y_d, target=throw.target)))
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


def compile_columns(pattern: Pattern, t0_abs_s: float) -> Schedule:
    """The columns pattern's schedule (plan § 1.2 / § 2.4).

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
    release and lands at ``t_abs_s``) — the last throw's landing is R5's cone
    delivery and is deliberately not scheduled here. Ball B's PRE-EXISTING
    flight (airborne before ``t0``) gets one extra CATCH with no matching
    THROW in this schedule. The schedule ends with a REST at the site of the
    chronologically last CATCH, TWO ``rest_tail_s`` after it (one for that
    catch's own tail, one for the REST's window — see the comment at the
    append: one tail would dispatch the REST before the touch-down and splice
    the catch away).

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
    t_f = flight_s(pattern.apex_m)
    beta = beat_s(t_f, pattern.dwell_s)
    tau = transit_s(t_f, pattern.dwell_s)
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
    skills.append(Skill(kind=CATCH, ball_id=1, site=site1,
                        t_abs_s=t0_rel + tau, window_s=tau))

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
        skills.append(Skill(kind=THROW, ball_id=ball_i, site=site_i,
                            t_abs_s=t_throw, window_s=window,
                            y_d=(np.zeros(2), pattern.apex_m), target=site_i))
        if i <= n - 2:
            _check_window('CATCH %d' % i, tau)
            skills.append(Skill(kind=CATCH, ball_id=ball_i, site=site_i,
                                t_abs_s=t_throw + t_f, window_s=tau))

    skills.sort(key=lambda s: s.t_abs_s)
    skills = _fold_catch_throw_pairs(skills, float(pattern.dwell_s))

    catches = [s for s in skills if s.kind == CATCH]
    last_catch = max(catches, key=lambda s: s.t_abs_s)
    # TWO tails, and the second one is the whole point: a REST is a SETTLE, and
    # a SETTLE dispatched before the last touch-down splices OVER the catch —
    # the head is cut at a knot before the ball arrives and the new window is
    # solved to stop at the rest site, so the machine stops reaching for a ball
    # that is still in the air.  One tail puts the REST's dispatch
    # (`t_abs - rest_tail - lead`) a lead BEFORE the touch-down; two puts it a
    # lead before the catch segment's own tail runs out, where the machine is
    # already at rest and the splice only extends the hold.
    rest_t = closing_rest_t_abs(last_catch.t_abs_s, pattern.rest_tail_s)
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
                    t0_abs_s=t0, pattern='columns')


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
                        t0_abs_s: float) -> Schedule:
    """The R4 reload's OPENING REST bridge (owner decision D4, brief step
    1a) — installed BEFORE Ball Butler is even asked to reload, because the
    receive tilt :func:`compile_reload` needs is not known until BB's
    ``ThrowAnnouncement`` supplies the arrival velocity. ONE REST: bit-
    identical Skill construction to :func:`compile_one_ball`'s own opening
    REST (homes the hand from wherever it is to
    :data:`~jugglebot.motion.skills.sites.REST_HAND_REV`, lifts the cup onto
    ``site``), except ``holds_ball=False`` — unlike the self-toss opening
    REST (which assumes an operator has already placed a ball), nothing is
    in the cup while BB's throw is still pending.

    ``SkillNode._run_one_ball`` (a Juggle goal with ``reload=True``) installs this
    schedule, then calls ``bb/reload`` + ``bb/throw_at_target``; the hand
    is already homing while that round trip is in flight. On the
    announcement, ``SkillNode._on_announcement`` compiles the real
    :func:`compile_reload` schedule and SWAPS the executor — by which point
    the hand is already at rest, so that schedule's own combined home+tilt
    REST (U2's SIMPLICITY contract) has nothing left to home and only
    reaches the tilt. This schedule's only job is to get there and hold.
    """
    t0_rel = 0.0
    period = float(floor_lift_s)
    _check_window('the reload bridge REST (hand homing while BB is asked to '
                 'reload)', period)
    skills = [Skill(kind=REST, ball_id=0, site=site,
                    t_abs_s=t0_rel + period, window_s=period,
                    holds_ball=False)]
    skills = _assign_leads(skills)
    t0 = float(t0_abs_s)
    skills = [_shifted(s, t0) for s in skills]
    return Schedule(skills=tuple(skills), flight_s=0.0, beat_s=period,
                    transit_s=0.0, dwell_s=0.0, t0_abs_s=t0,
                    pattern='self_toss')


def compile_reload(landing_mm, landing_vel_mm_s, t_land_abs_s: float,
                   pattern: OneBallPattern, t0_abs_s: float) -> Schedule:
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
    """
    landing_mm = _vec3(landing_mm, 'landing_mm')
    landing_vel = _vec3(landing_vel_mm_s, 'landing_vel_mm_s')
    if not np.isfinite(float(t_land_abs_s)):
        raise ValueError('t_land_abs_s must be finite, got %r'
                          % (t_land_abs_s,))
    tilt = sg.receive_hold_tilt(landing_vel)
    site0 = pattern.sites[0]

    t0_rel = 0.0
    t_land_rel = float(t_land_abs_s) - float(t0_abs_s)

    lift_s = float(pattern.floor_lift_s)
    pretilt_period = max(lift_s, PRETILT_S)
    _check_window('the opening REST (the floor lift + the pre-tilt attitude)',
                 pretilt_period)
    pretilt_end_rel = t_land_rel - RELOAD_CATCH_WINDOW_S
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

    catch_window = t_land_rel - pretilt_end_rel
    _check_window('the reload CATCH (pre-tilt REST end to touch-down)',
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
