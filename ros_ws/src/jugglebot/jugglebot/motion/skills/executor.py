"""Installing one skill onto the live plan, and running a schedule (plan § 2.4).

Two layers, both pure Python (no ROS):

* :func:`install_segment` — THE one install path.  It takes whatever plan is
  currently streaming and one scheduled skill, and returns the plan that should
  be streaming instead.  Either the previous segment has ENDED (the machine is
  holding its terminal rest) and the new one starts at a FRESH ORIGIN, or it has
  not and the new one is SPLICED into it at a knot the wire has not read yet.
  Both the sim gate and ``trajectory_node``'s service call this, so there is one
  set of refusal semantics rather than two.
* :class:`SkillExecutor` — the paper's Orchestrator: it walks a
  :class:`~jugglebot.motion.skills.schedule.Schedule` against a wall clock and
  dispatches each skill through an installer callable.  It never touches a plan,
  which is what lets the same executor drive a sim record and a ROS service.

**Why an installer CALLABLE and not the plan itself.**  The record a sim gate
holds is a local object; the record the robot holds lives behind a service
boundary in another process.  The executor's job — when to dispatch, what
terminal to build, when to re-send a catch, when to end the attempt — is
identical either way, and is the part worth testing offline.
"""

from __future__ import annotations

import dataclasses
import math
from typing import Callable, List, Optional, Tuple, Union

import numpy as np

import jugglebot.hardware_config as hw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills.schedule import (HANDOFF_LEAD_KNOTS,
                                               HANDOFF_LEAD_S, LEAD_KNOTS,
                                               LEAD_S, MIN_WINDOW_KNOTS,
                                               MIN_WINDOW_S, WIRE_READ_KNOTS,
                                               Schedule, Skill)
from jugglebot.motion.skills.segments import (CATCH, REST, THROW, CatchTerminal,
                                              RestTerminal, Segment,
                                              SegmentConfig, ThrowAfterCatch,
                                              ThrowTerminal)
from jugglebot.motion.trajectory.cycle_plan import CyclePlan

# ── Refusal codes this layer owns ────────────────────────────────────────────
#: The solve finished so late that the splice knot is already behind the wire.
SPLICE_TOO_LATE = 'SPLICE_TOO_LATE'
#: The window from the splice knot to the skill's event is shorter than a plan.
WINDOW_TOO_SHORT = 'WINDOW_TOO_SHORT'
#: The tracker has no landing for the ball this CATCH is for.
NO_LANDING = 'NO_LANDING'
#: A ``ValueError`` escaped a terminal build — the same code ``_svc_plan_cycle``
#: converts an unclassified failure to, so a guard matching it keeps matching.
UNREACHABLE = 'UNREACHABLE'

#: The wire-read budget — :data:`LEAD_KNOTS`, :data:`LEAD_S`,
#: :data:`WIRE_READ_KNOTS`, :data:`HANDOFF_LEAD_KNOTS`, :data:`HANDOFF_LEAD_S`,
#: :data:`MIN_WINDOW_KNOTS` and :data:`MIN_WINDOW_S` — is DEFINED in
#: ``skills.schedule`` and imported above, because a skill states its own
#: dispatch lead (``Skill.lead_s``) and ``schedule`` may not import this module.
#: ``executor.LEAD_S`` still resolves; there is one object, not two.
#:
#: Seconds inside which a committed CATCH is no longer re-aimed.  A re-send
#: dispatched at ``t`` splices no earlier than ``t + LEAD_S``, so a re-send
#: inside ``LEAD_S + dt`` of touch-down would splice at or after the landing
#: knot: there is no window left to re-solve, only a seam past the event.  One
#: knot of window is the floor, hence the ``+ dt``.
CATCH_FREEZE_S = LEAD_S + float(hw.JB_TRAJ_KNOT_DT_S)


@dataclasses.dataclass
class Landing:
    """One tracked ball arrival, on the executor's wall clock."""

    pos_mm: np.ndarray
    vel_mm_s: np.ndarray
    t_land_abs_s: float


@dataclasses.dataclass
class PlanRecord:
    """The ACTIVE plan and the wall-clock instant its knot 0 is emitted at.

    ``t0_s`` is on whatever monotone seconds clock the caller runs (the CAN wall
    clock on the robot, ``time.monotonic`` in a test), and it is the ONLY link
    between a plan's own clock and the schedule's: knot ``k`` is emitted at
    ``t0_s + k·dt``.
    """

    plan: CyclePlan
    meta: uc.CycleMeta
    t0_s: float

    @property
    def end_s(self) -> float:
        """Wall-clock instant the plan's terminal knot is emitted at."""
        return float(self.t0_s) + float(self.plan.total_duration)


@dataclasses.dataclass(frozen=True)
class InstallResult:
    """What one install attempt did — accepted or refused, never an exception."""

    accepted: bool
    code: str
    message: str
    plan_wall_s: float
    #: The knot the segment was spliced at; ``0`` for a fresh origin, ``-1`` for
    #: a refusal (nothing was spliced).
    splice_k: int = -1
    #: The new record's origin on the wall clock.
    t0_s: float = 0.0
    #: The skill's event (release / touch-down) on the NEW record's plan clock.
    event_t_s: float = 0.0
    #: True when the splice landed on a release knot and the segment was seeded
    #: POST-RELEASE (:func:`~jugglebot.motion.unified_cycle.release_state_at_knot`)
    #: rather than mid-carry — the ring's own handoff, with the ball that just
    #: left the cup keeping its detach cone.
    seeded_post_release: bool = False


def _event_abs_s(kind: str, terminal) -> float:
    """The terminal's event instant on the WALL clock.

    Unit B's terminal types carry SEGMENT-relative times (``t_release_s``,
    ``t_land_s``, ``t_rest_s``) because that is the clock ``plan_segment`` builds
    on.  The schedule speaks absolute instants.  The conversion happens exactly
    ONCE, here: :func:`install_segment` takes terminals whose time field holds
    the ABSOLUTE instant, converts to the segment clock the moment it knows the
    segment's origin, and rebuilds the terminal with the relative value.  Doing
    it anywhere else would mean two clocks in one dataclass.
    """
    if kind == THROW:
        return float(terminal.t_release_s)
    if kind == CATCH:
        return float(terminal.t_land_s)
    if kind == REST:
        return float(terminal.t_rest_s)
    raise ValueError('unknown skill kind %r (expected one of %s)'
                     % (kind, sg.KINDS))


def _terminal_on_segment_clock(kind: str, terminal, t_origin_s: float):
    """``terminal`` with its event time re-based onto a segment starting at
    ``t_origin_s``.  Raises ``ValueError`` (from the terminal's own validation)
    when the event is at or before the origin — converted by the caller."""
    t_rel = _event_abs_s(kind, terminal) - float(t_origin_s)
    if kind == THROW:
        return dataclasses.replace(terminal, t_release_s=t_rel)
    if kind == CATCH:
        # A catch-with-throw carries a SECOND instant, and it rides the same
        # conversion — two clocks in one dataclass is the thing this function
        # exists to prevent.
        tt = terminal.then_throw
        if tt is not None:
            tt = dataclasses.replace(
                tt, t_release_s=float(tt.t_release_s) - float(t_origin_s))
        return dataclasses.replace(terminal, t_land_s=t_rel, then_throw=tt)
    return dataclasses.replace(terminal, t_rest_s=t_rel)


def _rest_seed(record: PlanRecord) -> uc.CycleState:
    """The at-rest state a plan that has ENDED leaves the machine in."""
    return uc.CycleState.at_rest(
        record.plan.pose[-1], float(record.plan.hand_rev[-1]),
        levelling_correction=record.meta.levelling_correction)


def _snap_to_release(meta: uc.CycleMeta, k_s: int,
                     dt: float, event_k: Optional[int] = None
                     ) -> Tuple[int, bool]:
    """``(k_s, seeded_post_release)`` — a splice inside a detach cone SNAPS back
    to the release knot it belongs to.

    A splice landing at ``k_rel < k_s <= k_rel + n_detach`` is refused by
    ``splice_at`` (:func:`~jugglebot.motion.unified_cycle.
    _refuse_splice_into_a_detach_cone`) for a physical reason: the new window
    would be solved WITHOUT the detach-cone equalities of a ball that has
    already left the cup, measured at 1.126 m/s² of off-axis specific force
    against an original 4.4e-16 — a lateral shove delivered off the lip that
    ``validate_cycle`` cannot see, because what is left is a perfectly smooth
    track.

    But that band is exactly where a ring's own handoff falls: the catch-with-
    throw of the ball now in flight splices at the previous release
    (``schedule.compile_columns``), so the first knot ``lead_s`` ahead of the
    solve is the release knot, one of the ``n_detach`` after it, or — when the
    skill took the handoff lead and dispatched early — a knot or two before it.
    Snapping to
    ``k_rel`` and seeding POST-RELEASE (the same floats
    :func:`~jugglebot.motion.unified_cycle.release_state_from_meta` hands a
    chain: site, take-off velocity, ``g``, detach axis, the head's levelling
    frame) is what the chain has always done at a TERMINAL release — this is
    the interior spelling of it, and the cone is then carried by the new window
    rather than solved away.  The refusal stays for every other ``k_s`` in the
    band, which is now unreachable by construction rather than by policy.

    **The rule is stated on the RELEASE, not on the dispatch.**  ``k_rel`` is
    the LAST release the head carries, and the snap fires for every
    ``k_s <= k_rel + n_detach`` — an EARLY dispatch (``k_s < k_rel``) included,
    not only the ``k_rel <= k_s`` band the cone refusal covers.  That is what
    lets a skill buy solve time by dispatching earlier
    (``schedule.HANDOFF_LEAD_S``) without moving the seam: the splice knot is
    pinned by the head's own release, so dispatch JITTER cannot shift it either,
    and the release is never re-solved.  Splicing at the earlier ``k_s`` instead
    would cut the head BEFORE the release and hand the new window a mid-carry
    seed, which is exactly the re-solve the cone refusal exists to prevent.

    Only the LAST release BEFORE the segment's own event needs checking
    (``event_k``): an earlier release's cone ends before this one's, so a
    ``k_s`` past ``k_rel + n_detach`` is past every other cone
    too.

    Snapping is not free in either direction: ``k_s`` moves at most ``n_detach``
    knots LATER (never a problem — that is away from the wire) or arbitrarily
    EARLIER, i.e. toward the wire.  ``SPLICE_TOO_LATE`` is measured on the
    SNAPPED knot for that reason, so an early dispatch whose solve overruns is
    refused against the release knot it would have rewritten.
    """
    if not meta.releases:
        return k_s, False
    n_detach = uc.detach_knots()
    # Only a release the segment's own event FOLLOWS is a handoff candidate.
    # A CATCH re-send re-aims a catch whose carried throw is already in the
    # head AFTER it; snapping to that release would pull the splice past the
    # catch (measured 2026-09-12, sim/skills_gate.py: every re-send refused
    # WINDOW_TOO_SHORT with a negative window). Such a re-send splices at its
    # raw knot and re-solves its own window, release included.
    knots = [int(round(float(m.t_s) / float(dt))) for m in meta.releases
             if event_k is None or int(round(float(m.t_s) / float(dt))) < event_k]
    if not knots:
        return k_s, False
    k_rel = max(knots)
    if k_s <= k_rel + n_detach:
        return k_rel, True
    return k_s, False


def install_segment(record: Optional[PlanRecord],
                    seed_rest: Optional[uc.CycleState],
                    kind: str, terminal, t_now_s: float, *,
                    lead_s: float = LEAD_S,
                    cfg: Optional[SegmentConfig] = None,
                    limits=None, geom=None, warm_start=None,
                    t_install_s: Union[None, float, Callable[[], float]] = None
                    ) -> Tuple[Optional[PlanRecord], InstallResult,
                               Optional[Segment]]:
    """Plan one skill and put it on the machine — the ONE install path.

    ``terminal`` carries its event time as an ABSOLUTE instant on ``t_now_s``'s
    clock (see :func:`_event_abs_s`); everything below converts it once.

    **Two cases, and the machine's physical state is what chooses.**

    *Fresh origin* — there is no record, or the record's plan has ENDED by the
    time this install could reach the wire (``t_now + lead >= record.end_s``).
    The machine is then holding a terminal REST: a stopped platform over a
    seated ball.  The segment is planned from that rest and installed with
    ``t0 = t_now_s``, i.e. the solve time is NOT skipped over.  That is sound
    for exactly one reason — the seed is at rest, so the plan's knot 0 IS where
    the machine still is when the solve finishes, however long it took.  (A
    moving seed has no such property, which is why the splice branch below
    measures its own lateness.)

    *Splice* — the previous segment is still streaming.  The new window opens at
    ``k_s = splice_knot(meta, τ, lead_s)``, the first knot at least ``lead_s``
    ahead of now, seeded by ``state_at_knot`` and joined by ``splice_at``.  The
    head is carried bit for bit, so whatever the emitter has already sent stays
    sent.

    **Two refusals this function owns, one physical fact each.**

    * :data:`WINDOW_TOO_SHORT` — the splice knot is within
      ``MIN_WINDOW_KNOTS·dt`` of the event: the QP has fewer knots to reach the
      release/touch-down than the gate needs to measure a jerk at all.
    **A splice at or before the head's last release snaps to that release.**
    ``k_s <= k_rel + n_detach`` means the ring is handing this segment the ball
    it has just thrown, so ``k_s`` becomes ``k_rel`` and the seed is
    :func:`~jugglebot.motion.unified_cycle.release_state_at_knot` rather than
    ``state_at_knot`` — see :func:`_snap_to_release`, and
    :attr:`InstallResult.seeded_post_release` for what it reports.  Both
    refusals below are measured on the SNAPPED knot.

    * :data:`SPLICE_TOO_LATE` — the solve finished after the wire had already
      read past ``k_s``.  Checked AGAINST A FRESH CLOCK READ taken AFTER the
      solve (``t_install_s`` is a clock CALLABLE — the ROS shell passes
      ``time.perf_counter`` — evaluated here once the segment exists; a float
      is accepted for tests that inject the lateness) rather than against
      ``t_now_s``, because the whole question is how long the solve took:
      ``k_s`` must still be more than :data:`WIRE_READ_KNOTS` knots ahead of the
      install instant.  Without it a slow solve silently rewrites trajectory the
      Teensy is interpolating — a step command on six legs.

    Every :class:`~jugglebot.motion.unified_cycle.CycleInfeasible` and every
    ``ValueError`` from a malformed terminal becomes a refusing
    :class:`InstallResult`; nothing propagates but a programming error, because
    a caller that must branch on an exception type is a caller that will one day
    forget to.

    Returns ``(new_record_or_None, result, segment_or_None)``.  On a refusal the
    record is the one passed in (unchanged) and the segment is ``None``.
    """
    cfg = SegmentConfig() if cfg is None else cfg
    t_now_s = float(t_now_s)
    lead_s = float(lead_s)
    fresh = record is None or (t_now_s + lead_s) >= record.end_s
    post_release = False

    try:
        if fresh:
            if seed_rest is not None:
                seed = seed_rest
            elif record is not None:
                seed = _rest_seed(record)
            else:
                raise ValueError(
                    'no record and no seed_rest — a fresh origin needs the '
                    'machine state to plan from, and nothing here can invent it')
            t0 = t_now_s
            k_s = 0
            t_origin = t0
        else:
            dt = float(record.plan.dt)
            tau = t_now_s - float(record.t0_s)
            k_s = uc.splice_knot(record.meta, tau, lead_s)
            t0 = float(record.t0_s)
            event_k = int(math.floor((_event_abs_s(kind, terminal) - t0) / dt))
            k_s, post_release = _snap_to_release(record.meta, k_s, dt, event_k)
            t_origin = t0 + k_s * dt
            window_s = _event_abs_s(kind, terminal) - t_origin
            if window_s < MIN_WINDOW_S - 1e-12:
                return record, InstallResult(
                    False, WINDOW_TOO_SHORT,
                    'a %.3f s window from the splice (knot %d) to the %s is '
                    'under the %d-knot floor (%.3f s) — the gate has no stencil '
                    'to measure a jerk in'
                    % (window_s, k_s, kind, MIN_WINDOW_KNOTS, MIN_WINDOW_S),
                    0.0), None
            seed = (uc.release_state_at_knot(record.plan, record.meta, k_s)
                    if post_release
                    else uc.state_at_knot(record.plan, record.meta, k_s))

        seg = sg.plan_segment(
            kind, seed, _terminal_on_segment_clock(kind, terminal, t_origin),
            cfg, limits, geom, warm_start=warm_start)

        if fresh:
            new_record = PlanRecord(plan=seg.plan, meta=seg.meta, t0_s=t0)
            return new_record, InstallResult(
                True, 'OK',
                'fresh origin: %s over %.3f s from rest'
                % (kind, seg.plan.total_duration),
                float(seg.meta.plan_wall_s), splice_k=0, t0_s=t0,
                event_t_s=float(seg.event_t_s or 0.0),
                seeded_post_release=False), seg

        dt = float(record.plan.dt)
        t_inst = (t_now_s if t_install_s is None
                  else float(t_install_s()) if callable(t_install_s)
                  else float(t_install_s))
        k_wire = int(math.floor((t_inst - t0) / dt)) + WIRE_READ_KNOTS
        if k_s <= k_wire:
            return record, InstallResult(
                False, SPLICE_TOO_LATE,
                'the solve took %.3f s and the wire has read to knot %d; splice '
                'knot %d is no longer ahead of it (%.3f s of plan already '
                'emitted, budget %.3f s from dispatch) — the head would be '
                'rewritten under the emitter'
                % (t_inst - t_now_s, k_wire, k_s, t_inst - t0,
                   max(0.0, (k_s - WIRE_READ_KNOTS) * dt - (t_now_s - t0))),
                float(seg.meta.plan_wall_s)), None

        plan, meta = uc.splice_at(record.plan, record.meta, k_s,
                                  seg.plan, seg.meta, limits, geom)
        new_record = PlanRecord(plan=plan, meta=meta, t0_s=t0)
        return new_record, InstallResult(
            True, 'OK',
            'spliced %s at knot %d (%.3f s on the plan clock)'
            % (kind, k_s, k_s * dt),
            float(meta.plan_wall_s), splice_k=k_s, t0_s=t0,
            event_t_s=(k_s * dt + float(seg.event_t_s or 0.0)),
            seeded_post_release=post_release), seg
    except uc.CycleInfeasible as exc:
        return record, InstallResult(False, exc.code, exc.outcome(), 0.0), None
    except ValueError as exc:
        return record, InstallResult(False, UNREACHABLE, str(exc), 0.0), None


class SkillExecutor:
    """Walk a :class:`Schedule` against a wall clock, dispatching each skill.

    The executor holds NO plan.  It decides *when* a skill is due, builds the
    terminal that describes it, hands both to ``installer`` and records what came
    back.  ``installer`` has one signature —
    ``installer(kind, terminal, t_now_s, ball_id=...) -> InstallResult`` — so the
    sim gate wraps :func:`install_segment` over a local :class:`PlanRecord` and
    the ROS node wraps a service call, with no second copy of the policy below.

    **A refusal ENDS THE ATTEMPT, and does not stop the machine.**  Every segment
    is rest-terminal (``segments``' invariant), so whatever is streaming when a
    refusal lands already ends at rest: the safe thing is to dispatch nothing
    further and let it run out.  Continuing instead would put the NEXT skill's
    segment on a plan whose predecessor never installed — a catch aimed from a
    pose the machine is not in.

    The one exception is a CATCH RE-SEND (the tracker refined a landing already
    committed): its refusal leaves the committed catch standing, which is a
    strictly better plan than none, so the attempt continues and the refusal is
    logged.

    **The executor holds no lead.**  Each :class:`~jugglebot.motion.skills.
    schedule.Skill` carries its own (``Skill.lead_s``), because a segment that
    follows a release can be dispatched earlier without moving its splice knot
    and one whose splice tracks its dispatch cannot — an executor-wide lead
    would have to be the smaller of the two for every skill.
    """

    def __init__(self, schedule: Schedule,
                 installer: Callable[..., InstallResult], *,
                 tracker: Optional[Callable[[int], Optional[Landing]]] = None,
                 catch_freeze_s: float = CATCH_FREEZE_S,
                 resend_min_interval_s: float = 0.025,
                 resend_pos_tol_mm: float = 1.0,
                 resend_t_tol_s: float = 0.002):
        self.schedule = schedule
        self.installer = installer
        self.tracker = tracker
        self.catch_freeze_s = float(catch_freeze_s)
        self.resend_min_interval_s = float(resend_min_interval_s)
        self.resend_pos_tol_mm = float(resend_pos_tol_mm)
        self.resend_t_tol_s = float(resend_t_tol_s)

        self.dispatched = set()          #: indices of ``schedule.skills``
        self.results = []                #: (index, Skill, InstallResult)
        self.attempt_ended = False
        self.end_code = ''
        #: The CATCH currently committed, as ``(index, Landing, t_install_s)``.
        self._live_catch = None

    # ── terminals ──

    @staticmethod
    def _commanded_target_mm(target_site, y_d) -> np.ndarray:
        """Where the ball is COMMANDED to land: the target site's catch point
        offset by ``y_d``.

        R2's learner is off, so ``u = y_d`` — the commanded landing IS the
        desired one.  When R3 turns the learner on, THIS is the line that moves,
        and it is one line because a throw carried by a catch reads it too.
        """
        dy, _flight = y_d
        dy = np.asarray(dy, dtype=float).reshape(2)
        target = np.asarray(target_site.catch_site_mm(), dtype=float)
        return target + np.array([dy[0] * 1000.0, dy[1] * 1000.0, 0.0])

    def _throw_terminal(self, skill: Skill) -> ThrowTerminal:
        """A THROW's terminal: release at the site, landing where the learner
        says (see :meth:`_commanded_target_mm`)."""
        _dy, flight = skill.y_d
        return ThrowTerminal(
            site_mm=skill.site.throw_site_mm(),
            target_mm=self._commanded_target_mm(skill.target, skill.y_d),
            flight_s=float(flight), t_release_s=float(skill.t_abs_s))

    def _catch_terminal(self, skill: Skill, landing: Landing) -> CatchTerminal:
        """A CATCH's terminal, plus the throw it carries when the schedule
        folded one onto it (``schedule.ThenThrow``).

        The carried release is the SCHEDULE's, not the tracker's: a re-send
        re-solves the whole remaining window against a refined landing with the
        release instant, site and target unchanged, because the pattern's beat
        is what the other hand is already flying against.
        """
        then_throw = None
        tt = skill.then_throw
        if tt is not None:
            _dy, flight = tt.y_d
            then_throw = ThrowAfterCatch(
                t_release_s=float(tt.t_release_abs_s),
                site_mm=skill.site.throw_site_mm(),
                target_mm=self._commanded_target_mm(tt.target, tt.y_d),
                flight_s=float(flight))
        return CatchTerminal(landing_mm=landing.pos_mm,
                             landing_vel_mm_s=landing.vel_mm_s,
                             t_land_s=float(landing.t_land_abs_s),
                             rest_site_mm=skill.site.rest_site_mm(),
                             then_throw=then_throw)

    def _rest_terminal(self, skill: Skill) -> RestTerminal:
        return RestTerminal(rest_site_mm=skill.site.rest_site_mm(),
                            t_rest_s=float(skill.t_abs_s))

    # ── the tick ──

    def tick(self, t_abs_s: float) -> List[str]:
        """Dispatch everything due by ``t_abs_s``; return the log lines."""
        lines = []
        if self.attempt_ended:
            return lines
        t_abs_s = float(t_abs_s)
        for idx, skill in enumerate(self.schedule.skills):
            if idx in self.dispatched:
                continue
            if skill.dispatch_s() > t_abs_s:
                continue
            lines.extend(self._dispatch(idx, skill, t_abs_s))
            if self.attempt_ended:
                return lines
        lines.extend(self._resend_live_catch(t_abs_s))
        return lines

    def _dispatch(self, idx: int, skill: Skill, t_abs_s: float) -> List[str]:
        if skill.kind == CATCH:
            landing = None if self.tracker is None else self.tracker(skill.ball_id)
            if landing is None:
                self.dispatched.add(idx)
                self.attempt_ended = True
                self.end_code = NO_LANDING
                return ['%.3f END %s: the tracker has no landing for ball %d — '
                        'a catch cannot be aimed at an unobserved ball'
                        % (t_abs_s, NO_LANDING, skill.ball_id)]
            terminal = self._catch_terminal(skill, landing)
        elif skill.kind == THROW:
            terminal = self._throw_terminal(skill)
        else:
            terminal = self._rest_terminal(skill)

        res = self.installer(skill.kind, terminal, t_abs_s,
                             ball_id=skill.ball_id)
        self.dispatched.add(idx)
        self.results.append((idx, skill, res))
        if not res.accepted:
            self.attempt_ended = True
            self.end_code = res.code
            return ['%.3f END %s at skill %d (%s): %s'
                    % (t_abs_s, res.code, idx, skill.kind, res.message)]
        if skill.kind == CATCH:
            self._live_catch = (idx, terminal, t_abs_s)
        else:
            self._live_catch = None
        return ['%.3f %s skill %d: %s' % (t_abs_s, skill.kind, idx, res.message)]

    def _resend_live_catch(self, t_abs_s: float) -> List[str]:
        """Re-aim the committed CATCH when the tracker has moved its landing.

        Three fences, one physical fact each: nothing is re-sent inside
        ``catch_freeze_s`` of touch-down (the hand is already decelerating into
        the ball and a re-solve there is a change nothing can execute); nothing
        is re-sent more often than ``resend_min_interval_s`` (a re-solve costs
        more than a knot, so a faster cadence would queue installs behind the
        emitter); and nothing is re-sent for a landing that has not MOVED beyond
        the tracker's own noise.
        """
        if self._live_catch is None or self.tracker is None:
            return []
        idx, terminal, t_last = self._live_catch
        skill = self.schedule.skills[idx]
        if t_abs_s >= float(terminal.t_land_s) - self.catch_freeze_s:
            self._live_catch = None
            return []
        if t_abs_s - t_last < self.resend_min_interval_s:
            return []
        # A re-send re-splices at now + lead; once that leaves less than the
        # window floor before touch-down there is nothing a solve could do but
        # refuse WINDOW_TOO_SHORT, so it is not attempted (measured 2026-09-12,
        # sim/skills_gate.py: 75 such refusals per 8 throws at the 0.278 s
        # transit, every one a wasted ~25 ms solve on the orchestrator thread).
        if (float(terminal.t_land_s) - (t_abs_s + float(skill.lead_s))
                < MIN_WINDOW_S - 1e-12):
            self._live_catch = None
            return []
        landing = self.tracker(skill.ball_id)
        if landing is None:
            return []
        moved_mm = float(np.max(np.abs(
            np.asarray(landing.pos_mm, dtype=float) - terminal.landing_mm)))
        moved_s = abs(float(landing.t_land_abs_s) - float(terminal.t_land_s))
        if moved_mm <= self.resend_pos_tol_mm and moved_s <= self.resend_t_tol_s:
            return []
        new_terminal = self._catch_terminal(skill, landing)
        res = self.installer(skill.kind, new_terminal, t_abs_s,
                             ball_id=skill.ball_id)
        self.results.append((idx, skill, res))
        if not res.accepted:
            # The committed catch stands — a refused RE-aim is strictly better
            # than no catch, so the attempt continues.
            self._live_catch = (idx, terminal, t_abs_s)
            return ['%.3f RESEND-REFUSED %s at skill %d: %s'
                    % (t_abs_s, res.code, idx, res.message)]
        self._live_catch = (idx, new_terminal, t_abs_s)
        return ['%.3f RESEND skill %d: landing moved %.1f mm / %.3f s — %s'
                % (t_abs_s, idx, moved_mm, moved_s, res.message)]
