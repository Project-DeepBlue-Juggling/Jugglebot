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
from jugglebot.motion.skills.sites import Site
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
# These three numbers are ONE budget — how much of a knot grid the solve may
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

#: Knots of lead between "now" and the earliest knot a splice may land on.
#:
#: 6 knots = 0.150 s at the fixed 40 Hz grid: :data:`WIRE_READ_KNOTS` (3) + 2
#: knots (50 ms) for the MEASURED solve + 1 knot of margin.  Plan § 2.4 said
#: "≈ 0.10 s: plan ≤ 20 ms plus a two-knot margin"; the measured plan is
#: 26–34 ms end to end over a six-throw columns schedule on the idle Jetson
#: (``python tools/probes/skills_segment_sweep.py --apex 0.9 --sep 40 60 80 100
#: --dwell 0.30 --jerk 200000 150000 --vel-acc 300/5000 --hand-acc 3500
#: --n-throws 6``, 2026-09-12, ``temp/probes/skills_segment_run2.md``: worst
#: ``plan ms max`` 34.4), so a 4-knot lead left ``LEAD_KNOTS - WIRE_READ_KNOTS``
#: = 1 knot = 25 ms of solve budget and the robot path refused
#: ``SPLICE_TOO_LATE`` on the first handoff.  Budget here: 3 knots = 75 ms.
LEAD_KNOTS = 6

#: Seconds of lead, derived — never a second copy of the knot grid.
LEAD_S = LEAD_KNOTS * float(hw.JB_TRAJ_KNOT_DT_S)

#: The DISPATCH lead of a skill whose segment FOLLOWS a release (every CATCH in
#: the columns pattern, with or without ``then_throw``).
#:
#: 8 knots = 0.200 s, i.e. 5 knots = 125 ms of solve budget against the same
#: measured 26–34 ms.  It can exceed :data:`LEAD_KNOTS` because such a segment
#: splices AT the release knot whatever its dispatch instant
#: (``executor._snap_to_release``): the splice knot is PINNED by the head's own
#: release, so buying solve time by dispatching earlier neither moves the seam
#: nor re-solves the ball that has already left the cup.  A skill that does NOT
#: follow a release has no such pin — its splice knot tracks its dispatch — so
#: it keeps :data:`LEAD_S` and its 3-knot budget.
HANDOFF_LEAD_KNOTS = 8

#: Seconds, derived.
HANDOFF_LEAD_S = HANDOFF_LEAD_KNOTS * float(hw.JB_TRAJ_KNOT_DT_S)


def flight_s(apex_m: float) -> float:
    """No-drag vertical flight time (s) for a throw that peaks ``apex_m`` above
    its release/catch height: ``t_f = 2·sqrt(2·apex_m / g)``."""
    if not float(apex_m) > 0.0:
        raise ValueError('apex_m must be > 0, got %r' % (apex_m,))
    return 2.0 * math.sqrt(2.0 * float(apex_m) / _G_SI)


def beat_s(flight_s_: float, dwell_s: float) -> float:
    """Time between successive throws from the SAME hand: ``(t_f + d) / 2``."""
    return (float(flight_s_) + float(dwell_s)) / 2.0


def transit_s(flight_s_: float, dwell_s: float) -> float:
    """Time between a throw and the OTHER hand's next catch: ``(t_f − d) / 2``."""
    return (float(flight_s_) - float(dwell_s)) / 2.0


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
    ``target``, and the flight time) — the learner is off at R2.
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
            raise ValueError('y_d[1] (flight_s) must be > 0, got %r'
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
    #: ``target``, and the flight time (s). The learner is off at R2, so a
    #: columns THROW always carries ``(zeros(2), flight_s)``.
    y_d: Optional[Tuple[np.ndarray, float]] = None
    #: THROW only: the site the ball is to land at.
    target: Optional[Site] = None
    #: CATCH only: the same-site throw this catch carries out of the ball, as
    #: ONE window (:class:`ThenThrow`).  ``None`` for a standalone CATCH — the
    #: last catch of an attempt, and R4's reload.
    then_throw: Optional[ThenThrow] = None
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

    def due(self, t_abs_s: float) -> Tuple[Skill, ...]:
        """Skills whose dispatch instant has passed by ``t_abs_s``."""
        return tuple(s for s in self.skills
                     if s.dispatch_s() <= float(t_abs_s))


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
    is pure solve budget: it gets :data:`HANDOFF_LEAD_S` (5 knots of budget).
    Every other segment's splice knot tracks its own dispatch — dispatch it
    earlier and it simply splices earlier — so it gets :data:`LEAD_S` (3 knots).

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

    skills = []
    _check_window('initial CATCH', tau)
    skills.append(Skill(kind=CATCH, ball_id=1, site=site1,
                        t_abs_s=float(t0_abs_s) + tau, window_s=tau))

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
        t_throw = float(t0_abs_s) + i * beta
        window = pattern.launch_s if i == 0 else pattern.dwell_s
        skills.append(Skill(kind=THROW, ball_id=ball_i, site=site_i,
                            t_abs_s=t_throw, window_s=window,
                            y_d=(np.zeros(2), t_f), target=site_i))
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
    rest_t = last_catch.t_abs_s + 2.0 * pattern.rest_tail_s
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

    return Schedule(skills=tuple(skills), flight_s=t_f, beat_s=beta,
                    transit_s=tau, dwell_s=float(pattern.dwell_s),
                    t0_abs_s=float(t0_abs_s))
