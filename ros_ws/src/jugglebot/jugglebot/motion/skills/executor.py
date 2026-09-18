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
from jugglebot import ball_possession as bp
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.skills import admissible as adm
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import schedule as sch
from jugglebot.motion.skills.memory import (APEX_RATIO_BAND, Experience,
                                            apex_in_band)
from jugglebot.motion.skills.schedule import (HANDOFF_LEAD_KNOTS,
                                               HANDOFF_LEAD_S, LEAD_KNOTS,
                                               LEAD_S, MIN_WINDOW_KNOTS,
                                               MIN_WINDOW_S, WIRE_READ_KNOTS,
                                               Schedule, Skill)
from jugglebot.motion.skills.segments import (CATCH, REST, THROW, CatchTerminal,
                                              RestTerminal, Segment,
                                              SegmentConfig, ThrowAfterCatch,
                                              ThrowTerminal)
from jugglebot.motion.trajectory import ballistics_bc
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
#: No safe command exists for this throw (R3, plan § 2.5/2.6): either the
#: admissible box swept for this (site, target) pair is EMPTY — no grid point
#: passed the offline sweep with margin — or the learner's local kernel fit
#: diverged to a non-finite command (the neighbourhood weights underflowed to
#: 0, a query far outside every ``h_y``-scaled neighbour). Both are the same
#: physical fact from the skill's point of view: nothing here can hand the
#: platform a command it can stand behind, so the throw is refused before any
#: solve is attempted rather than handed a NaN or an out-of-envelope target.
NO_ADMISSIBLE_COMMAND = 'NO_ADMISSIBLE_COMMAND'

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

#: How long an UNDISPATCHED CATCH is willing to wait for its own ball to show
#: up on the tracker before the attempt ends ``NO_LANDING`` (owner decision
#: 2026-09-13, "wait for landing" -- a single-site self-toss's catch has no
#: second ball to buy it tracker settling time the way columns does, so
#: refusing at the first look, as R2 did, ended every self-toss attempt after
#: exactly one throw).  **Superseded for a catch-with-throw at R3-l (owner
#: decision 2026-09-13, "at release, then refine")**: such a catch now
#: dispatches at its own SCHEDULED instant, aimed at the predicted landing
#: (:meth:`SkillExecutor._predicted_landing`), and never reaches this wait --
#: waiting dispatched it AFTER its own ball's release, which splices into the
#: launch THROW's settle tail and refuses ``LIMIT_JERK`` (262 743 mm/s³
#: against 150 000, measured on every attempt).  Since the aim became ordered
#: (2026-09-18, :meth:`SkillExecutor._catch_aim`) the ONLY catch that can
#: still reach this deadline is one with no previous release in this schedule
#: AND no tracker landing at all — columns' very first catch, of a ball
#: thrown before ``t0``.  Measured against the
#: skill's own EVENT instant, not against ``t_now`` or the skill's
#: ``window_s``: the deadline is ``skill.t_abs_s - CATCH_DEADLINE_WINDOW_S -
#: skill.lead_s``, the R2 operating point's transit window (0.278 s), flown
#: clean in the sim gate and the R2 hardware plan gate -- the shortest a
#: catch's own flight is ever budgeted, so waiting this long never eats into a
#: window a real transit would have left for the solve.
CATCH_DEADLINE_WINDOW_S = 0.278

# ── Where a CATCH's aim comes from (owner decision 2026-09-18) ──────────────
#
# A catch is aimed at a LANDING, and two things know one: the schedule knows
# what the throw was COMMANDED to achieve, the tracker knows what the ball is
# actually doing. The release instant alone slips 0.019-0.137 s from the
# release KNOT (2026-09-17, ``flight_truth3``), throw to throw, which nothing
# can learn away because it is not repeatable — only observed. So the aim
# follows the observation whenever the observation has converged, and the
# schedule is its prior: the ordered rule is FIT > SCHEDULE > filter
# (:meth:`SkillExecutor._catch_aim`), and no step of it ever WAITS.
#
#: Aim every catch from the tracker's CONVERGED ballistic fit, with the
#: schedule's commanded landing (:meth:`_predicted_landing`) as the prior at
#: dispatch and later fits refining the committed catch
#: (:meth:`_resend_live_catch`) until the freeze. **This is the LIVE default**
#: (owner 2026-09-18) — ``skill_node``'s ``catch_aim_source`` parameter and
#: this constructor both default to it.
AIM_TRACKER = 'tracker'
#: Open loop: the landing that ball's previous release was COMMANDED to
#: achieve aims the catch and nothing refines it — no tracker call at all.
#: This is what the 2026-09-15..09-17 sittings flew; retained selectable
#: because the A/B runsheets fly it against :data:`AIM_TRACKER`.
AIM_SCHEDULE = 'schedule'
#: :data:`AIM_SCHEDULE`, plus ONE re-aim of the committed catch from the
#: MEASURED throw state: the hand's launch-speed ratio ``r`` from
#: ``/hand_telemetry`` (:mod:`jugglebot.motion.skills.hand_launch`), never QTM.
AIM_SCHEDULE_HAND = 'schedule_hand'
#: The whole vocabulary — ``skill_node`` validates its parameter against it.
AIM_SOURCES = (AIM_SCHEDULE, AIM_SCHEDULE_HAND, AIM_TRACKER)

# ── Outcome capture (R3, plan § 2.5 step 6 / § 2.7) ─────────────────────────
#
#: How close a tracker sample may be to ITS OWN predicted landing instant
#: before the estimate is trusted as a throw's outcome. 0.012 s -- roughly
#: 50 mm above the 830 mm catch plane at the ~4.2 m/s vertical arrival speed:
#: (date, command, result) 2026-09-13,
#: ``tools/probes/throw_outcome_bag_probe.py`` candidate (e) -- the last
#: tracker update above 880 mm projected to 830 mm -- median 15.7 mm / 3.6 ms
#: against a 4.7 mm-RMS mocap ground truth, bag 2026-09-07_09-32-56, n = 9, run
#: twice identical. A sample taken inside this guard of the landing it is
#: itself predicting is discarded (the prior valid sample, if any, stands);
#: the whole point is that a Kalman estimate is least trustworthy right at its
#: own crossing.
OUTCOME_GUARD_S = 0.012

#: Seconds after the LANDING before a throw's outcome finalises.  Named rather
#: than reused from :data:`CATCH_FREEZE_S` because the two answer different
#: questions (when a re-aim stops being useful vs. when the possession verdict
#: has had time to settle) and a probe may move them independently.
#:
#: **0.35 s, measured** (2026-09-16,
#: ``tools/probes/caught_window_bag_probe.py`` over bags
#: ``2026-09-16_14-16-38`` and ``2026-09-15_18-51-37``): the delay from the
#: tracker's own observed crossing of the 830 mm catch plane (``/balls``,
#: descending, linearly interpolated) to the first debounced SEATED sample on
#: ``/hand_telemetry`` was, per caught throw, +112.0, +41.3, -47.5, +191.2,
#: +122.3, -28.6, +122.3 ms on 09-16 (n = 7) and -40.2 .. -49.3 ms on 09-15
#: (n = 22, median -44.5 ms).
#:
#: **Why 0.35 and not 0.25** (owner ruling 2026-09-16): an independent pass over
#: the same bags paired one 09-16 arrival with a SEATED edge +281.9 ms after its
#: landing, where ``ball_held`` then held ~150 ms and went False again.  That
#: read as a bobble, and 0.25 s was first chosen to exclude it.  It is not a
#: bobble: the operator's note for that attempt (armA-050) is "worked" -- every
#: ball caught -- and the throw AFTER it was itself caught, so the ball never
#: hit the floor.  The False ~150 ms later is the NEXT throw's release.  **A
#: catch that settles late is a catch**, so the window covers it: 0.35 s clears
#: that +281.9 ms by ~70 ms and the largest delay this probe itself reproduces
#: (+191.2 ms) by 159 ms.  The provisional 0.15 s this replaces was too short
#: for five of the seven 09-16 throws.
#:
#: WARNING: the committed probe does NOT reproduce that +281.9 ms pairing -- see
#: its own "Known limitation".  The window covers it anyway, because widening is
#: monotone (it can only ADD a catch, never invent a landing) and the ruling
#: above does not depend on the exact number.
#:
#: **0.70 s (2026-09-17, both sittings, 37 throws,
#: ``scratchpad/late_catch_probe.py`` -> logbook
#: ``2026-09-17-late-catches-are-a-late-tracker.md``)**: when the ball arrived
#: while the cup was still accelerating downward faster than gravity (the hand
#: late against the ball by 30-90 ms), the debounced SEAT came only once the
#: hand had stopped -- +0.26 .. +0.61 s after the physical landing on half of
#: the caught throws -- and every one of those past the old 0.35 s read
#: ``caught=False`` although the operator saw the ball caught (two genuine
#: drops in the sitting against ten reported).  A catch that settles late is a
#: catch (the 09-16 ruling); 0.70 s clears the +0.61 s worst case.  It costs
#: nothing on a chained catch -- :meth:`SkillExecutor._bound_by_next_release`
#: closes the window at the next release regardless -- and on a final catch
#: there is no beat to meet.  The late seats themselves are the symptom the
#: tracker fix in that entry removes; the window is the reporting floor under
#: it, not the fix.
CAUGHT_WINDOW_S = 0.70

#: How far BEFORE the landing a SEATED reading still counts as this throw's
#: catch.  The sign of the seated delay is not fixed: on 2026-09-15 the sensor
#: seated 39-48 ms BEFORE the tracker's interpolated crossing on every one of
#: 22 throws (the crossing is an ESTIMATE of a 830 mm plane the cup rim reaches
#: first, so an early seat is physically ordinary, not a clock error).  A
#: verdict window that opened at the landing would have scored every one of
#: those as a miss.  0.10 s is twice the largest early seat observed and still
#: ~0.6 s inside the empty-cup interval that precedes every arrival (the ball
#: this row tracks left the cup a full flight earlier), so it cannot read the
#: PREVIOUS ball's possession.
CAUGHT_LEAD_S = 0.10

#: The bound on how late an OBSERVED landing may push the finalise instant.
#: The window is anchored on the observed landing so a plant that throws fast
#: or slow does not move the verdict off the ball (measured 2026-09-16: the
#: observed crossing ran -13 .. +196 ms against the scheduled one), but a
#: tracker estimate is not trusted without limit -- a diverged filter reporting
#: a landing seconds away would otherwise hold the row open forever and with it
#: the learner's feedback.  Past this the scheduled landing plus the cap is the
#: anchor, and the row finalises on whatever evidence it has.  0.35 s is ~1.8x
#: the largest late crossing measured.  It does NOT move with
#: :data:`CAUGHT_WINDOW_S` (owner question, 2026-09-16): this bounds how late
#: the OBSERVED LANDING may move the anchor -- a tracker-trust question, set by
#: the +196 ms worst late crossing -- while the window is how long the SEAT may
#: take after it, a sensor question.  Worst-case finalise latency is their sum,
#: 1.05 s since 2026-09-17 -- longer than the ~0.95 s beat, which is fine: on
#: a chained catch :meth:`SkillExecutor._bound_by_next_release` closes the
#: window at the next release whatever the sum says, and the next throw's
#: command is computed at its dispatch (~1.1 s before its release), before
#: this row could reach the memory under ANY window.  Only a final catch ever
#: waits the full sum, and nothing is waiting on it.
CAUGHT_LAND_DEFER_CAP_S = 0.35

#: How far BEFORE this ball's NEXT scheduled release a row's verdict window
#: must have closed.
#:
#: A chained self-toss catches and RE-THROWS the same physical ball, and the
#: tracker keeps one continuous track across both flights (2026-09-15: all
#: markers, gated to the announced ball), so from the next release onwards the
#: tracker's estimate for this ball id is the NEXT flight's landing. A window
#: that reaches past that instant samples the wrong flight -- the 2026-09-16
#: contamination, where three ``armB-090`` rows recorded 2.23/2.23/1.16 s
#: flights for a 0.857 s command. The freeze in :meth:`SkillExecutor.
#: _consider_landing` is the primary fix; bounding the window is the
#: structural one: the row simply cannot still be open when the ball leaves
#: again. 0.010 s is a quarter of one 40 Hz tick -- just enough that the
#: closing tick is strictly before the release, since the only thing the
#: window still needs past the landing is the SEATED latch, and after the
#: re-release the cup is empty by construction.
OUTCOME_NEXT_RELEASE_EPS_S = 0.010

#: The observer's possession-evidence value this layer treats as "caught".
#: ``ball_possession.py`` is pure Python (stdlib only -- no ROS2, no config
#: imports, verified 2026-09-13), so this is the module's own constant,
#: imported rather than restated: a second copy of this value is exactly the
#: "timing twin" class plan § 0 forbids.
CAUGHT_EVIDENCE = bp.EVIDENCE_SEATED

# ── The R3 precondition ladder (PORT@R3, INVARIANTS.md § 8) ─────────────────
#
# Each code below re-enforces one row the FSM's ``_step_checking``
# (``toss_sequencer.py``) and ``_step_throwing`` used to own, on the SAME
# physical fact, in the SAME dependency order -- see :func:`precondition_refusals`
# and :meth:`SkillExecutor._dispatch`.
REJECTED_MOCAP_STALE = 'REJECTED_MOCAP_STALE'
REJECTED_NOT_LEVELLED = 'REJECTED_NOT_LEVELLED'
REJECTED_HAND_STALE = 'REJECTED_HAND_STALE'
#: ``REJECTED_HAND_NOT_PARKED`` WAS here and is RETIRED (owner decision,
#: 2026-09-16).  It refused a fresh-origin skill whose hand was off the park
#: band, and the R3 second sitting showed the refusal is the wrong shape for
#: the fact: nine of the ten refusals it produced were FALSE (the bridge's
#: ``pos_cmd`` echo went stale at 0.5639 rev across an ACTIVATE re-park while
#: the encoder read 0.0001), and the one TRUE refusal (hand at 0.5605 rev) was
#: a hand the opening REST could simply have brought home.  The class it stood
#: proxy for -- "the plan's hand seed is not where the hand actually is" -- is
#: now enforced at the SEED, in ``trajectory_node._cycle_start_state``, which
#: reconciles a fresh-origin rest seed against the MEASURED hand instead of
#: refusing; the opening REST then carries the hand to the settle clamp.  See
#: ``logbook/2026-09-16-hand-park-refusal-retired-rest-homes-the-hand.md``.
REJECTED_BALL_UNKNOWN = 'REJECTED_BALL_UNKNOWN'
REJECTED_NO_BALL = 'REJECTED_NO_BALL'
#: Leaving the streaming mode that owns the platform mid-attempt -- the rest
#: tail already streaming is the safe end (``reload_sequencer.py:340-366``'s
#: ``obs.control_mode != RELOAD_CONTROL_MODE`` -> ``self._abort('MODE_CHANGED')``).
ABORTED_MODE_CHANGED = 'ABORTED_MODE_CHANGED'
#: No evidence the ball left by ``t_release + RELEASE_GRACE_S`` -- the throw
#: produces no learner row (``toss_sequencer.py::_step_throwing``'s
#: ``now >= self._release_deadline`` -> ``self._abort('NO_RELEASE')``).
ABORTED_NO_RELEASE = 'ABORTED_NO_RELEASE'

#: Seconds a throw's release evidence may lag ``t_release`` before the attempt
#: aborts ``ABORTED_NO_RELEASE``. Restated, not imported, from
#: ``toss_sequencer.TOSS_RELEASE_GRACE_S`` (0.5 s) -- that module dies at R4.
RELEASE_GRACE_S = 0.5


@dataclasses.dataclass(frozen=True)
class Observations:
    """Everything the R3 precondition ladder asks of the machine at one tick.

    Supplied by an observer callable the ROS shell wires up (a later unit),
    so the same ladder runs unchanged in the sim gate and on the robot --
    plan § 0's "the same orchestrator code drives the MuJoCo plant and the
    robot". Each field is one row of ``INVARIANTS.md`` § 8:

    * ``mocap_fresh`` -- ``REJECTED_MOCAP_STALE``.
    * ``hand_fresh`` -- ``REJECTED_HAND_STALE``.
    * ``levelled`` -- ``trajectory_node``'s ``gravity_correction_loaded`` on a
      fresh status (C-LEVEL-1.O) -- ``REJECTED_NOT_LEVELLED``.
    * ``ball_evidence`` -- one of :data:`bp.EVIDENCE_SEATED` /
      :data:`bp.EVIDENCE_EMPTY` / :data:`bp.EVIDENCE_UNKNOWN` -- launch-only
      ``REJECTED_BALL_UNKNOWN`` / ``REJECTED_NO_BALL``.
    * ``in_trajectory_mode`` -- checked every tick while an attempt runs, not
      only at dispatch -- ``ABORTED_MODE_CHANGED``.
    """

    mocap_fresh: bool
    hand_fresh: bool
    levelled: bool
    ball_evidence: str
    in_trajectory_mode: bool


def precondition_refusals(obs: Observations, *, launch: bool,
                          skip_mocap: bool = False) -> List[str]:
    """Every PORT@R3 row this dispatch must refuse on -- ALL of them, not just
    the first, so a rehearsal driver reports every refusal in one pass rather
    than one at a time across repeated dry runs (Workflow Rules: "make gates
    report every refusal at once").

    Order is the FSM's dependency order (``toss_sequencer.py::_step_checking``):
    mocap before levelled (an un-levelled reading off a stale graph is not a
    geometry fact yet), levelled before the hand chain, hand freshness before
    the hand-parked band. ``launch`` is True only for a fresh-origin THROW --
    the schedule's own opening self-toss, planned from rest, where a stale
    hand-parked band or an unread ball sensor would seed the segment from a
    state nobody has confirmed.

    **There is NO hand-POSITION row here (RETIRED 2026-09-16).** One used to
    fire on any fresh-origin install whose hand had left the park band; it is
    gone, along with the ``fresh_origin`` argument that gated it, because a
    refusal is the wrong answer to the fact.  A hand that is not at park is now
    simply CARRIED there: the seed a fresh-origin window is planned from is
    reconciled against the MEASURED hand in
    ``trajectory_node._cycle_start_state``, and the schedule's opening REST
    (``schedule.FLOOR_LIFT_S``, 1.5 s) moves the hand from wherever it truly is
    to the settle clamp.  MEASURED (2026-09-16, probe, session limits
    300/5000/200000 mm + hand 3500 rev/s^2): that REST plans CLEAN from every
    hand seed in the stroke -- 0.0001 rev (0.31 rev/s peak) through 9.9 rev
    (9.76 rev/s), three orders under the 200 rev/s session ceiling -- so there
    is no seed the lift cannot absorb and nothing left for a refusal to
    protect.  The hand FRESHNESS row stays: a seed reconciled against a stale
    encoder would be the same defect one level down.

    A REST still runs this ladder ONLY at a fresh origin (see
    :meth:`SkillExecutor._dispatch`): the schedule's CLOSING REST splices onto
    the live plan the same way a CATCH does and needs none of these facts.
    ``skip_mocap`` drops ``REJECTED_MOCAP_STALE`` for a REST specifically: a
    REST does not aim, so a stale mocap graph is not a fact it needs.
    """
    codes = []
    if not skip_mocap and not obs.mocap_fresh:
        codes.append(REJECTED_MOCAP_STALE)
    if not obs.levelled:
        codes.append(REJECTED_NOT_LEVELLED)
    if not obs.hand_fresh:
        codes.append(REJECTED_HAND_STALE)
    if launch:
        if obs.ball_evidence == bp.EVIDENCE_UNKNOWN:
            codes.append(REJECTED_BALL_UNKNOWN)
        elif obs.ball_evidence != bp.EVIDENCE_SEATED:
            codes.append(REJECTED_NO_BALL)
    return codes


class _NoAdmissibleCommand(Exception):
    """Raised by :meth:`SkillExecutor._command_u` when no safe command exists
    for a throw -- an empty admissible box, or the learner's fit diverging to
    a non-finite command. Caught by :meth:`SkillExecutor._dispatch` and turned
    into a :data:`NO_ADMISSIBLE_COMMAND` refusal; never escapes this module."""


def _observed_apex_m(landing: 'Landing') -> float:
    """The apex (m above the catch plane) a landing estimate implies —
    ``schedule.apex_from_vz`` on its vertical arrival speed, converted from
    the mm/s this layer's :class:`Landing` carries.

    This is the learner's OUTCOME (2026-09-18): the same physical quantity
    the command is, read off the tracker's converged parabola, so neither the
    release instant nor the filter's lag can bias it."""
    return sch.apex_from_vz(float(np.asarray(landing.vel_mm_s,
                                             dtype=float)[2]) / 1000.0)


@dataclasses.dataclass
class Landing:
    """One tracked ball arrival, on the executor's wall clock.

    ``from_fit`` says the estimate came from the tracker's CONVERGED
    gravity-fixed batch fit (``tracking/flight_fit.py``) rather than the
    Kalman extrapolation fallback. Only a fitted landing may become a learner
    row (:meth:`SkillExecutor._consider_landing`): 16 of the 22 rows written
    on 2026-09-17 had no converged fit, and the filter's crossing runs
    0.06-0.20 s late and grows later through the descent. The catch AIM path
    still uses an unfitted landing -- a rough aim beats no aim.
    """

    pos_mm: np.ndarray
    vel_mm_s: np.ndarray
    t_land_abs_s: float
    from_fit: bool = False


@dataclasses.dataclass
class _PendingOutcome:
    """One released ball's outcome, still waiting to finalise (plan § 2.5/2.7).

    Registered at the RELEASING skill's first (and only) successful dispatch
    -- a THROW, or a CATCH carrying a ``then_throw`` -- and finalised
    :data:`CAUGHT_WINDOW_S` after the landing, whether or not the attempt is
    still running by then (a refused later skill still leaves an observable
    flight in progress).

    **The verdict is taken over a window around the OBSERVED landing, not as a
    point sample after the scheduled one** (2026-09-16, the R3 apex-ladder
    sitting).  The plant throws ~8 % fast, so the observed crossing ran -13 ..
    +196 ms against the scheduled one, and the possession sensor then debounces:
    sampling the observer ONCE at ``t_land_scheduled + 0.15 s`` read the cup
    BEFORE the ball had settled and scored 4 of 5 real catches as misses.  So
    :attr:`caught_seen` LATCHES on any SEATED reading taken inside
    ``[min(t_land_scheduled, t_land_obs) - CAUGHT_LEAD_S, finalise_at]``, where
    ``finalise_at`` follows the observed landing (bounded by
    :data:`CAUGHT_LAND_DEFER_CAP_S`, so a diverged tracker estimate cannot hold
    the row open forever) -- see :meth:`SkillExecutor._outcome_window`.  The
    latch is one-way for the same reason ``best_landing`` is: one good sample
    stands even if a later tick goes blind, and the ball leaving again for its
    NEXT throw must not retract a catch that happened.

    **``best_landing`` is FROZEN at the crossing** (2026-09-16): it is the
    tracker estimate taken closest to, but before, this flight's landing, and
    nothing sampled after the ball has landed can replace it -- see
    :meth:`SkillExecutor._consider_landing`. The window itself closes before
    this ball's NEXT scheduled release (:attr:`t_next_release_s`), and a
    finalised row whose observed apex is not within
    ``memory.APEX_RATIO_BAND`` of the commanded one is DROPPED.

    ``release_confirmed`` is the R3 ladder's own bookkeeping (plan § carried
    R3 note / ``ABORTED_NO_RELEASE``): confirmed only by evidence AT OR AFTER
    :attr:`t_release_s` -- a possession EMPTY reading that follows a SEATED
    reading (:attr:`seated_seen`) seen at or after release, or a tracker
    landing whose ``t_land_abs_s`` is itself after release -- checked every
    tick from registration until :data:`RELEASE_GRACE_S` past release has
    elapsed with no evidence found -- see :meth:`SkillExecutor.
    _advance_release_evidence`. This is deliberately blind to evidence from
    BEFORE the release instant: a carried throw (a CATCH with a
    ``then_throw``) is registered at the catch's dispatch, up to a beat
    ahead of its own release, while the cup is still carrying the PREVIOUS
    ball -- an EMPTY reading or tracker landing from that earlier flight must
    never confirm THIS row's release. Unconfirmed past the deadline drops
    this row: no evidence the ball left means no learner row, whether or not
    anything else ends the attempt first.
    """

    ball_id: int
    x: np.ndarray
    u: np.ndarray
    t_release_s: float
    t_land_scheduled_s: float
    target_xy_mm: np.ndarray
    best_landing: Optional[Landing] = None
    #: ``t_land_abs_s - t_sample`` of :attr:`best_landing` -- how far ahead of
    #: its OWN predicted crossing the accepted estimate was taken. Recorded
    #: for diagnosis (a row whose landing was seen from far away is a weaker
    #: row); it does NOT gate acceptance -- see
    #: :meth:`SkillExecutor._consider_landing` on why preferring the smallest
    #: lead was tried and rejected.
    best_lead_s: Optional[float] = None
    #: The implied apex (m) of the last estimate REFUSED, and how many were
    #: refused -- carried only so a dropped row can name its reason
    #: (``no row: observed apex ...``).
    rejected_apex_m: Optional[float] = None
    #: Why that estimate was refused, as the phrase the dropped row prints.
    rejected_reason: str = ''
    n_rejected: int = 0
    #: This ball's NEXT scheduled release instant, or ``None`` when the
    #: schedule never throws it again. Filled at registration from the
    #: schedule (:func:`_next_release`) so :meth:`SkillExecutor.
    #: _outcome_window` stays a pure function of the row, and bounds
    #: ``finalise_at`` by :data:`OUTCOME_NEXT_RELEASE_EPS_S`.
    t_next_release_s: Optional[float] = None
    release_confirmed: bool = False
    seated_seen: bool = False
    #: Latched True by :meth:`SkillExecutor._advance_outcomes` on the first
    #: SEATED reading inside this row's verdict window -- the ``caught`` field
    #: of the :class:`~jugglebot.motion.skills.memory.Experience`.  Distinct
    #: from :attr:`seated_seen`, which is the RELEASE ladder's bookkeeping and
    #: is cleared by an EMPTY reading before the release.
    caught_seen: bool = False
    #: The tick on which :attr:`caught_seen` latched -- the first SEATED
    #: reading inside the window.  Reported (never learned) as the CONTACT
    #: PHASE ``seat - scheduled landing`` on the OUTCOME line: on 2026-09-17
    #: (apex 0.6 m) it separated the operator's verdicts exactly -- +0.10 s
    #: on all four smooth catches (ball met at the bottom of the dive),
    #: +0.02 s (met at the top, then the cup dived away) and +0.34 s (the
    #: bounce re-seating) on the ones that bounced twice.
    t_seat_s: Optional[float] = None


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


def _previous_release(schedule: Schedule, idx: int, ball_id: int):
    """``(release_site, t_release_abs_s, y_d, target)`` of the last release of
    ``ball_id`` strictly before ``schedule.skills[idx]`` -- a THROW or a
    CATCH's ``then_throw`` -- or ``None`` when no such release is IN THIS
    SCHEDULE (columns' very first catch, of a ball thrown before ``t0`` --
    plan owner decision 2026-09-13, "at release, then refine": that catch
    falls back to waiting for the tracker, unchanged)."""
    for j in range(idx - 1, -1, -1):
        sk = schedule.skills[j]
        if sk.ball_id != ball_id:
            continue
        if sk.kind == THROW:
            return sk.site, float(sk.t_abs_s), sk.y_d, sk.target
        if sk.kind == CATCH and sk.then_throw is not None:
            tt = sk.then_throw
            return sk.site, float(tt.t_release_abs_s), tt.y_d, tt.target
    return None


def _next_release(schedule: Schedule, idx: int,
                  ball_id: int) -> Optional[float]:
    """The absolute instant ``ball_id`` is next released AFTER
    ``schedule.skills[idx]``, or ``None`` when this schedule never throws it
    again (the last throw of an attempt, or a ball only caught from here on).

    The mirror of :func:`_previous_release`, and the bound
    :meth:`SkillExecutor._outcome_window` closes a verdict window with: from
    that instant the ball is in its NEXT flight, so every observation of it
    belongs to the next row, not this one.
    """
    for j in range(idx + 1, len(schedule.skills)):
        sk = schedule.skills[j]
        if sk.ball_id != ball_id:
            continue
        if sk.kind == THROW:
            return float(sk.t_abs_s)
        if sk.kind == CATCH and sk.then_throw is not None:
            return float(sk.then_throw.t_release_abs_s)
    return None


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

    **R3: the learner, the admissible box, and outcome capture — all optional.**
    ``learner`` (an object exposing ``command(x, y_d) -> (3,)``, e.g. a
    ``memory.Memory`` bound to its ``LearnerConfig`` via a lambda at the call
    site) and ``boxes`` (a SEQUENCE of :class:`~jugglebot.motion.skills.
    admissible.AdmissibleBox` — ``admissible.load``'s own return shape,
    selected by ``(site pair, apex band)`` via ``admissible.select`` rather
    than a ``{site_pair: box}`` dict, so a box swept for one apex can never
    be silently reused at another) are consulted ONCE per released ball, at
    the releasing skill's first dispatch (:meth:`_command_u`); a CATCH
    re-send reuses that command rather than recomputing it, because the
    command must not change late in a transit (plan § 0). No ``learner`` ⇒
    the command is the identity prior (``u = y_d`` exactly, R2's behaviour).
    ``observer`` (``(ball_id, t_abs_s) -> str``, the possession evidence at
    that instant) and ``on_experience``
    (called with one ``memory.Experience`` per released ball, in schedule
    order) drive outcome capture (:meth:`_advance_outcomes`), which keeps
    running after ``attempt_ended`` — see :attr:`done`.

    **R3: the precondition ladder — also optional, gated by ``observations``.**
    ``observations`` (``(t_abs_s) -> Observations``, a snapshot of the whole
    machine at one tick — mocap, hand, level and ball state, plus whether the
    streaming mode that owns the platform is still active) turns on every
    PORT@R3 row of ``INVARIANTS.md`` § 8 in one switch: :func:`precondition_
    refusals` before each THROW/CATCH dispatch (:meth:`_dispatch`), the
    ``ABORTED_MODE_CHANGED`` check every tick (:meth:`tick`), and the
    ``ABORTED_NO_RELEASE`` check on every pending release
    (:meth:`_advance_release_evidence`). No ``observations`` ⇒ none of the
    three run and the executor behaves exactly as R2 left it — the sim gate
    and every pre-R3 caller need not change. ``observer`` is untouched by
    this: it keeps its one job, possession evidence for outcome capture, and
    :meth:`_advance_release_evidence` reads it for the SAME reason
    (``EVIDENCE_EMPTY`` is release evidence, ``EVIDENCE_SEATED`` is caught
    evidence — one sensor, two questions, no second callable).
    """

    def __init__(self, schedule: Schedule,
                 installer: Callable[..., InstallResult], *,
                 tracker: Optional[Callable[[int], Optional[Landing]]] = None,
                 catch_aim_source: str = AIM_TRACKER,
                 launch_ratio: Optional[Callable[[int, float],
                                                 Optional[float]]] = None,
                 catch_freeze_s: float = CATCH_FREEZE_S,
                 resend_min_interval_s: float = 0.025,
                 resend_pos_tol_mm: float = 10.0,
                 resend_t_tol_s: float = 0.010,
                 resend_max_per_catch: int = 2,
                 learner=None, boxes=None,
                 lateral_authority_m: Optional[float] = None,
                 observer: Optional[Callable[[int, float], str]] = None,
                 on_experience: Optional[Callable[[Experience], None]] = None,
                 observations: Optional[Callable[[float], Observations]] = None):
        self.schedule = schedule
        self.installer = installer
        self.tracker = tracker
        if catch_aim_source not in AIM_SOURCES:
            raise ValueError('catch_aim_source must be one of %r, not %r'
                             % (list(AIM_SOURCES), catch_aim_source))
        #: :data:`AIM_SCHEDULE` / :data:`AIM_SCHEDULE_HAND` / :data:`AIM_TRACKER`.
        self.catch_aim_source = str(catch_aim_source)
        #: ``(ball_id, t_release_abs_s) -> Optional[r]`` — the MEASURED hand
        #: launch-speed ratio, only ever read in :data:`AIM_SCHEDULE_HAND`.
        self.launch_ratio = launch_ratio
        self.catch_freeze_s = float(catch_freeze_s)
        self.resend_min_interval_s = float(resend_min_interval_s)
        #: How far a later fit must move the committed landing before a
        #: re-aim is worth a solve — 10.0 mm / 0.010 s (owner 2026-09-18).
        #: The catch's timing cliff is ~20 ms wide: the ball seated
        #: +0.015 s after the scheduled landing on the two catches that
        #: bounced off the cup and +0.104 s on the four that seated smoothly
        #: (2026-09-17, ``tools/probes/late_catch_bag_probe``), so half that
        #: cliff is the smallest move worth acting on, and 10 mm is inside
        #: the cup. One re-solve costs 25-130 ms of orchestrator time on the
        #: loaded Jetson (six ``SPLICE_TOO_LATE`` at the 2026-09-17 23:49
        #: sitting), so a tolerance below the tracker's own noise — the
        #: pre-2026-09-18 1.0 mm / 0.002 s pair — spends that budget on
        #: jitter and moves the aim nowhere.
        self.resend_pos_tol_mm = float(resend_pos_tol_mm)
        self.resend_t_tol_s = float(resend_t_tol_s)
        #: The most re-aims ONE catch may pay for. Two, because the fit
        #: converges once and then only sharpens: a third solve buys less
        #: than the 25-130 ms it costs, and an estimate that jitters across
        #: the tolerance could otherwise spend the whole splice budget of
        #: the catch it is refining.
        self.resend_max_per_catch = int(resend_max_per_catch)
        self.learner = learner
        self.boxes = boxes
        #: How far (m, per axis) the learner may move the commanded landing
        #: AWAY from the schedule's desired offset ``y_d``. ``None`` = the
        #: swept box alone bounds it. ``0.0`` (the live default, owner
        #: 2026-09-16) pins the lateral command to ``y_d`` and lets the learner
        #: correct FLIGHT only: a 4 mm lateral aim command made the banking
        #: step saturate to its 12 deg clamp during the pre-catch dive (leg
        #: jerk 137-161 k against 150 k, the 0.9 m 'wobble') -- sub-cm lateral
        #: authority is the planner's open defect, not the learner's to spend.
        self.lateral_authority_m = (None if lateral_authority_m is None
                                    else float(lateral_authority_m))
        self.observer = observer
        self.on_experience = on_experience
        self.observations = observations

        self.dispatched = set()          #: indices of ``schedule.skills``
        self.results = []                #: (index, Skill, InstallResult)
        self.attempt_ended = False
        self.end_code = ''
        #: The CATCH currently committed, as ``(index, Landing, t_install_s)``.
        self._live_catch = None
        #: Per-skill-index command cache: ``idx -> (x, u_dy, u_apex)``. Keyed
        #: on the RELEASING skill's own index (a THROW, or a CATCH carrying a
        #: ``then_throw``) so a catch re-send's second, third, ... call reuses
        #: the first dispatch's command rather than recomputing it.
        self._u_cache = {}
        #: Released balls awaiting outcome finalisation (:class:`_PendingOutcome`).
        self._pending_outcomes: List[_PendingOutcome] = []
        #: CATCH indices whose ONE hand-measured re-aim is already settled —
        #: applied, refused, or given up on (:data:`AIM_SCHEDULE_HAND`).
        self._hand_corrected = set()
        #: ``idx -> re-aims already installed`` (:attr:`resend_max_per_catch`).
        self._resend_counts = {}
        #: ``(idx, reason)`` pairs already reported by :meth:`_resend_live_catch`.
        self._resend_notes = set()

    @property
    def done(self) -> bool:
        """The attempt is OVER — refused, or every skill dispatched — AND no
        outcome is still waiting to finalise.

        Callers should tick until THIS, not until ``attempt_ended`` alone:
        ``attempt_ended`` only ever flips on a REFUSAL (a fully successful run
        dispatches every skill and never sets it, plan § 2.4's rest-terminal
        contract — nothing needs to "end" a schedule that simply ran out), and
        a released ball can still be in flight (its outcome not yet due) when
        either kind of finish happens, so its row is still worth a cold memory
        (plan § 2.7, "outcomes keep finalising after the attempt ends")."""
        finished = (self.attempt_ended
                   or len(self.dispatched) >= len(self.schedule.skills))
        return finished and not self._pending_outcomes

    # ── the R3 command: learner + admissible box, computed once per throw ──

    def _command_u(self, idx: int, site, target, y_d
                   ) -> Tuple[np.ndarray, float]:
        """The commanded ``u = (landing_xy_m, apex_m)`` for the ball this
        skill releases — computed ONCE (at ``idx``'s first dispatch) and
        cached, so a CATCH re-send's later calls return the SAME command
        (plan § 0: "the command never changes late in a transit").

        ``x = (site xy in m, 0, 0)`` — the seat-offset half of the state is
        unmeasured at R3 (plan § 0). No ``learner`` ⇒ ``u = y_d`` exactly (R2
        behaviour). Raises :class:`_NoAdmissibleCommand` when the learner's
        fit is non-finite, or no swept box covers ``(site.name, target.name)``
        at the DESIRED apex — the caller converts that to a
        :data:`NO_ADMISSIBLE_COMMAND` refusal before any solve is attempted.
        """
        if idx in self._u_cache:
            _x, u_dy, u_apex = self._u_cache[idx]
            return u_dy, u_apex
        dy, apex = y_d
        dy = np.asarray(dy, dtype=float).reshape(2)
        apex = float(apex)
        x = np.array([float(site.cup_mm[0]) / 1000.0,
                     float(site.cup_mm[1]) / 1000.0, 0.0, 0.0])
        # The box is looked up BEFORE the learner runs (finding 5, R3 audit,
        # 2026-09-13): a missing box means there is no admissible-region clip
        # to apply afterward, so a learner command would reach the platform
        # UNCLIPPED -- refuse up front rather than let an unbounded command
        # through the crack. Selected by the DESIRED apex (``y_d``), never the
        # learner's own command -- the learner has not run yet here, and must
        # never be able to hop the lookup between boxes by proposing a
        # different apex (the latent defect this closes: a 0.5 m apex
        # self-toss silently reusing the 0.9 m box and being clipped UP to
        # it).
        if self.boxes is None:
            box = None
        else:
            box = adm.select(self.boxes, (site.name, target.name), apex)
            if self.learner is not None and box is None:
                bands = sorted(b.apex_band_m for b in self.boxes
                              if b.site_pair == (site.name, target.name))
                bands_str = (', '.join('%.3f-%.3f m' % (lo, hi)
                                       for lo, hi in bands)
                            if bands else 'none swept for this pair')
                raise _NoAdmissibleCommand(
                    'no admissible box covers site pair %r at apex %.3f m '
                    '(bands swept for this pair: %s) — a learner command '
                    'may not reach the platform unclipped'
                    % ((site.name, target.name), apex, bands_str))
        if self.learner is None:
            u_dy, u_apex = dy, apex
        else:
            y_d_si = np.array([dy[0], dy[1], apex])
            try:
                u = np.asarray(self.learner.command(x, y_d_si), dtype=float
                               ).reshape(3)
            except ValueError as exc:
                raise _NoAdmissibleCommand(
                    'the learner could not produce a finite command for site '
                    '%r (target %r): %s' % (site.name, target.name, exc)
                ) from exc
            u_dy, u_apex = u[:2].copy(), float(u[2])
            if self.lateral_authority_m is not None:
                a = abs(self.lateral_authority_m)
                u_dy = np.asarray(dy, dtype=float) + np.clip(
                    u_dy - np.asarray(dy, dtype=float), -a, a)
        if box is not None:
            try:
                u_dy, u_apex = adm.clip((u_dy, u_apex), box)
            except adm.AdmissibleError as exc:
                raise _NoAdmissibleCommand(str(exc)) from exc
        u_dy = np.asarray(u_dy, dtype=float).reshape(2)
        u_apex = float(u_apex)
        self._u_cache[idx] = (x, u_dy, u_apex)
        return u_dy, u_apex

    # ── terminals ──

    @staticmethod
    def _commanded_target_mm(target_site, u) -> np.ndarray:
        """Where the ball is COMMANDED to land: the target site's catch point
        offset by ``u``'s landing-xy component.

        R2's learner is off, so ``u = y_d`` — the commanded landing IS the
        desired one.  At R3 this is the line that moves: ``u`` is
        :meth:`_command_u`'s output rather than the skill's raw ``y_d``, and it
        is one line because a throw carried by a catch reads it too.
        """
        dy, _apex = u
        dy = np.asarray(dy, dtype=float).reshape(2)
        target = np.asarray(target_site.catch_site_mm(), dtype=float)
        return target + np.array([dy[0] * 1000.0, dy[1] * 1000.0, 0.0])

    def _throw_terminal(self, idx: int, skill: Skill) -> ThrowTerminal:
        """A THROW's terminal: release at the site, landing where
        :meth:`_command_u` says.

        The planner's input is still a flight TIME; it is now DERIVED from
        the commanded apex (``schedule.flight_s``, the one conversion) rather
        than commanded directly, so the learner and the planner cannot
        disagree about which of the two a number is (2026-09-18)."""
        u_dy, u_apex = self._command_u(idx, skill.site, skill.target,
                                       skill.y_d)
        return ThrowTerminal(
            site_mm=skill.site.throw_site_mm(),
            target_mm=self._commanded_target_mm(skill.target, (u_dy, u_apex)),
            flight_s=sch.flight_s(u_apex), t_release_s=float(skill.t_abs_s))

    def _catch_terminal(self, idx: int, skill: Skill,
                        landing: Landing) -> CatchTerminal:
        """A CATCH's terminal, plus the throw it carries when the schedule
        folded one onto it (``schedule.ThenThrow``).

        The carried release is the SCHEDULE's, not the tracker's: a re-send
        re-solves the whole remaining window against a refined landing with the
        release instant, site and target unchanged, because the pattern's beat
        is what the other hand is already flying against. The carried throw's
        command is :meth:`_command_u`'s, cached under THIS catch's ``idx`` — a
        re-send calls this again and gets the SAME command back.
        """
        then_throw = None
        tt = skill.then_throw
        if tt is not None:
            u_dy, u_apex = self._command_u(idx, skill.site, tt.target, tt.y_d)
            then_throw = ThrowAfterCatch(
                t_release_s=float(tt.t_release_abs_s),
                site_mm=skill.site.throw_site_mm(),
                target_mm=self._commanded_target_mm(tt.target, (u_dy, u_apex)),
                flight_s=sch.flight_s(u_apex))
        return CatchTerminal(landing_mm=landing.pos_mm,
                             landing_vel_mm_s=landing.vel_mm_s,
                             t_land_s=float(landing.t_land_abs_s),
                             rest_site_mm=skill.site.rest_site_mm(),
                             then_throw=then_throw)

    def _rest_terminal(self, skill: Skill) -> RestTerminal:
        return RestTerminal(rest_site_mm=skill.site.rest_site_mm(),
                            t_rest_s=float(skill.t_abs_s))

    def _predicted_landing(self, idx: int, skill: Skill) -> Optional[Landing]:
        """The PREDICTED landing for a catch-with-throw that must dispatch at
        its SCHEDULED instant, before the tracker has one (plan owner decision
        2026-09-13, "at release, then refine"): the landing the ball's
        previous throw was commanded to ACHIEVE, built from the schedule
        alone -- ``y_d`` (the DESIRED offsets), not whatever ``_command_u``
        actually sent, because the learner's whole job is to make its command
        land AT ``y_d``.

        Position and time follow directly from ``y_d``; the arrival velocity
        is the no-drag ballistic arrival from the previous throw's release
        site to that landing over the flight the ``y_d`` APEX implies
        (``schedule.flight_s``, then ``ballistics_bc`` -- one gravity, plan
        § 2.6, the same closed form the reach/reload path already uses).

        ``None`` when this ball's previous release is not part of this
        schedule at all (columns' very first catch, of a ball thrown before
        ``t0``) -- the caller falls back to waiting for the tracker, exactly
        as every catch did before this unit.
        """
        prev = _previous_release(self.schedule, idx, skill.ball_id)
        if prev is None:
            return None
        site, t_release_s, y_d, target = prev
        pos_mm = self._commanded_target_mm(target, y_d)
        flight_s = sch.flight_s(float(y_d[1]))
        launch_vel = ballistics_bc.launch_velocity(
            site.throw_site_mm(), pos_mm, flight_s)
        vel_mm_s = ballistics_bc.arrival_velocity(launch_vel, flight_s)
        return Landing(pos_mm=pos_mm, vel_mm_s=vel_mm_s,
                       t_land_abs_s=t_release_s + flight_s)

    def _catch_aim(self, idx: int, skill: Skill, t_abs_s: float):
        """``(landing, aim_source)`` for CATCH ``idx`` at dispatch time, per
        :attr:`catch_aim_source`. ``(None, '')`` means nothing can aim it yet
        (the caller waits, then ends :data:`NO_LANDING`).

        Under :data:`AIM_TRACKER` (the live default, owner 2026-09-18) the
        rule is ONE order, ranked by how much each source knows about THIS
        flight:

        1. the tracker's landing when it carries ``from_fit`` — the
           converged gravity-fixed parabola is the only estimate that has
           seen the ball's ACTUAL release, and the release lags its knot by
           0.019-0.137 s throw to throw (2026-09-17), which the schedule
           cannot know and no learner can remove;
        2. the schedule's commanded landing (:meth:`_predicted_landing`) —
           the prior, available the instant this ball's previous release is
           in this schedule, and what the machine is actually trying to do;
        3. an UNFITTED tracker landing (the Kalman fallback) — its crossing
           runs 0.06-0.20 s late and grows later through the descent, so it
           ranks BELOW the prior, but it is the only thing left for a ball
           thrown before ``t0``.

        No step of that order waits. A catch that waits for the tracker is a
        catch that does not happen (2026-09-15: all 13 self-tosses ended
        ``NO_LANDING`` on a mocap marker that never came), and a
        catch-with-throw that waits dispatches AFTER its own ball's release,
        splicing into the launch THROW's settle tail — 262 743 mm/s³ of leg
        jerk against a 150 000 limit on every attempt (2026-09-13), where the
        release-snap dispatch accepts at 78 326. Later fits refine the
        committed catch in :meth:`_resend_live_catch`, which is where the
        tracker earns its place; the dispatch never blocks on it.

        Under :data:`AIM_SCHEDULE` / :data:`AIM_SCHEDULE_HAND` only step 2
        runs (plus the hand-measured correction), with no tracker call at
        all — the open-loop A/B arm. Either way the one catch the schedule
        cannot aim, of a ball released before ``t0``, falls through to the
        tracker: not a carve-out, the absence of any alternative.
        """
        if self.catch_aim_source == AIM_TRACKER:
            tracked = self._tracked_landing(idx, skill)
            if tracked is not None and tracked.from_fit:
                return tracked, AIM_TRACKER
            predicted = self._predicted_landing(idx, skill)
            if predicted is not None:
                return predicted, AIM_SCHEDULE
            if tracked is not None:
                return tracked, '%s (no converged fit)' % AIM_TRACKER
            return None, ''
        predicted = self._predicted_landing(idx, skill)
        if predicted is None:
            tracked = self._tracked_landing(idx, skill)
            return (tracked, AIM_TRACKER) if tracked is not None else (None, '')
        if self.catch_aim_source == AIM_SCHEDULE:
            return predicted, AIM_SCHEDULE
        # :data:`AIM_SCHEDULE_HAND`: the measured correction is applied to the
        # DISPATCH itself whenever the stroke has already been measured by
        # then — which is the common case, because a catch dispatches after
        # its own ball's release. Re-aiming a catch that was just installed
        # would pay a second ~25 ms solve on the orchestrator thread for a
        # landing that was already knowable;
        # :meth:`_resend_hand_corrected_catch` is for the case this branch
        # cannot serve (the ratio not measurable yet at dispatch).
        corrected, r = self._hand_ratio_landing(idx, skill, t_abs_s)
        if r is None:
            return predicted, self.catch_aim_source
        self._hand_corrected.add(idx)
        if corrected is None:
            return predicted, ('%s (r=%.3f, no arrival at the catch '
                               'plane — theoretical aim)'
                               % (AIM_SCHEDULE_HAND, r))
        return corrected, '%s (r=%.3f)' % (AIM_SCHEDULE_HAND, r)

    def _tracked_landing(self, idx: int, skill: Skill) -> Optional[Landing]:
        """The tracker's landing for CATCH ``idx``, gated by
        :meth:`_valid_tracked_landing`, or ``None``. The ONE place the
        tracker is called and its answer checked, shared by the aim
        (:meth:`_catch_aim`) and the re-aim (:meth:`_resend_live_catch`) so
        the two can never disagree about what a usable landing is."""
        if self.tracker is None:
            return None
        return self._valid_tracked_landing(
            idx, skill, self.tracker(skill.ball_id))

    def _hand_ratio_landing(self, idx: int, skill: Skill, t_abs_s: float):
        """``(corrected_landing, r)`` from the MEASURED hand stroke, for the
        one aim source that uses it (:data:`AIM_SCHEDULE_HAND`).

        ``(None, None)`` = no measurement to act on (no ``launch_ratio``
        wired, no previous release in this schedule, the stroke still in the
        future, or a window the monitor will not vouch for). ``(None, r)`` =
        a measurement whose scaled flight never reaches the catch plane. Both
        mean the same thing to the caller: keep the theoretical aim; the
        second says so with a number.
        """
        if self.launch_ratio is None:
            return None, None
        prev = _previous_release(self.schedule, idx, skill.ball_id)
        if prev is None:
            return None, None
        t_release_s = prev[1]
        if t_abs_s < t_release_s:
            # The throw stroke this correction measures has not happened yet.
            return None, None
        r = self.launch_ratio(skill.ball_id, t_release_s)
        if r is None:
            return None, None
        r = float(r)
        try:
            return self._hand_corrected_landing(idx, skill, r), r
        except ValueError:
            return None, r

    def _hand_corrected_landing(self, idx: int, skill: Skill,
                                r: float) -> Optional[Landing]:
        """The predicted landing re-flown with the MEASURED launch speed:
        the same release site and commanded landing as
        :meth:`_predicted_landing`, but the ballistic launch velocity scaled
        by ``r = v_meas / v_cmd`` (:mod:`~jugglebot.motion.skills.hand_launch`).

        For a vertical self-toss this is purely a TIMING correction --
        ``T' = r·T``, i.e. an apex ``r²`` times the commanded one, since a
        flight scales with the release speed and an apex with its SQUARE (the
        ONE place that ratio is applied) -- which is the whole "the catch is
        not timed" symptom: at 0.9 m apex and the measured r = 1.086,
        touch-down is ~74 ms later than the schedule commanded. The general case is solved, not
        approximated: :func:`ballistics_bc.arrival_state_at_z` crosses the
        commanded landing PLANE, so a scaled launch that also drifts
        horizontally gets the drifted arrival position too.

        Raises ``ValueError`` (from ``arrival_state_at_z``) when the scaled
        throw never reaches the catch plane -- an r so low the ball apexes
        below the hand. The caller keeps the theoretical aim and says so.
        """
        prev = _previous_release(self.schedule, idx, skill.ball_id)
        if prev is None:
            return None
        site, t_release_s, y_d, target = prev
        pos_mm = self._commanded_target_mm(target, y_d)
        flight_s = sch.flight_s(float(y_d[1]))
        release_pos = site.throw_site_mm()
        # Isotropic scaling by r: the hand stroke along the (tilted) platform
        # normal is the sole launch DoF, so an offset produced by tilt scales
        # with the stroke exactly. An offset produced by platform MOTION would
        # not — none exists in the R3 self-toss/columns schedules (dy is
        # small against the ~4 m/s stroke); revisit if that changes.
        launch_vel = ballistics_bc.launch_velocity(
            release_pos, pos_mm, flight_s) * float(r)
        pos2, vel2, t2 = ballistics_bc.arrival_state_at_z(
            release_pos, launch_vel, float(pos_mm[2]))
        return Landing(pos_mm=pos2, vel_mm_s=vel2,
                       t_land_abs_s=t_release_s + float(t2))

    def _valid_tracked_landing(self, idx: int, skill: Skill,
                               landing: Optional[Landing]
                               ) -> Optional[Landing]:
        """``landing`` if it is trustworthy for CATCH ``idx``, else ``None``.

        A tracker landing is only valid for this catch if it lands AFTER this
        ball's own previous release (:func:`_previous_release`): a tracker
        whose estimator is not reset until the ball's next physical release
        (the sim tracker; the robot's per-ball correlation resets the same
        way) keeps returning the FROZEN landing of the flight that just
        ended, so an un-gated re-use aims the next catch at a landing already
        in the past. With no previous release in this schedule (columns' very
        first catch) there is nothing to gate against: accept as before."""
        if landing is None:
            return None
        prev = _previous_release(self.schedule, idx, skill.ball_id)
        if prev is None:
            return landing
        t_release_s = prev[1]
        if float(landing.t_land_abs_s) <= t_release_s:
            return None
        return landing

    # ── the tick ──

    def tick(self, t_abs_s: float) -> List[str]:
        """Dispatch everything due by ``t_abs_s``; return the log lines.

        Dispatch and re-aim stop once the attempt has ended; outcome
        finalisation does not (plan § 2.7) — a ball already released can still
        be in flight when a LATER skill's refusal ends the attempt, and its
        landing row is still worth a cold memory. Callers should tick until
        :attr:`done`, not until ``attempt_ended``.

        **R3, when ``observations`` is wired**: ``obs.in_trajectory_mode`` is
        read EVERY tick the attempt is still running, not only at a dispatch —
        leaving the owning mode mid-skill (``ABORTED_MODE_CHANGED``) ends the
        attempt through whatever rest tail is already streaming, exactly like
        ``reload_sequencer.py``'s universal abort. With no ``observations``
        this block is skipped entirely: R2 behaviour, unchanged.
        """
        t_abs_s = float(t_abs_s)
        lines = []
        if not self.attempt_ended:
            obs = None if self.observations is None else self.observations(t_abs_s)
            if obs is not None and not obs.in_trajectory_mode:
                self.attempt_ended = True
                self.end_code = ABORTED_MODE_CHANGED
                lines.append(
                    '%.3f END %s: left the streaming mode that owns the '
                    'platform mid-attempt — the rest tail already streaming '
                    'is the safe end' % (t_abs_s, ABORTED_MODE_CHANGED))
            else:
                for idx, skill in enumerate(self.schedule.skills):
                    if idx in self.dispatched:
                        continue
                    if skill.dispatch_s() > t_abs_s:
                        continue
                    dispatch_lines, deferred = self._dispatch(
                        idx, skill, t_abs_s, obs)
                    lines.extend(dispatch_lines)
                    if self.attempt_ended:
                        break
                    if deferred:
                        # A CATCH still waiting on its own landing (below) --
                        # keep schedule order: nothing later may dispatch
                        # ahead of it this tick.  Retried next tick.
                        break
                if not self.attempt_ended:
                    # One re-aim path per aim source, and never two: under
                    # :data:`AIM_TRACKER` later fits refine the committed
                    # catch, under :data:`AIM_SCHEDULE_HAND` the measured
                    # launch ratio does, and under :data:`AIM_SCHEDULE` the
                    # schedule's commanded landing IS the aim, so there is
                    # nothing to refine it with.
                    if self.catch_aim_source == AIM_TRACKER:
                        lines.extend(self._resend_live_catch(t_abs_s))
                    elif self.catch_aim_source == AIM_SCHEDULE_HAND:
                        lines.extend(
                            self._resend_hand_corrected_catch(t_abs_s))
                if not self.attempt_ended and self.observations is not None:
                    lines.extend(self._advance_release_evidence(t_abs_s))
        lines.extend(self._advance_outcomes(t_abs_s))
        return lines

    def _dispatch(self, idx: int, skill: Skill, t_abs_s: float,
                  obs: Optional[Observations] = None
                  ) -> Tuple[List[str], bool]:
        """Try to dispatch skill ``idx``.  Returns ``(lines, deferred)`` --
        ``deferred`` is True only for a CATCH still waiting on its own
        landing (below); every other path dispatches, refuses, or ends the
        attempt outright and reports ``deferred=False``."""
        # `idx == 0` is the only skill any attempt can be SURE is a fresh
        # origin (Unit B): both `compile_self_toss` and `compile_columns`
        # build their opening REST as "a fresh install (no record yet)", and
        # every later skill splices onto the schedule's own already-streaming
        # plan. A non-fresh (closing) REST stays fully exempt -- only the
        # opening one is ever checked. A CATCH is excluded regardless of
        # index -- its seed is ALWAYS the live plan's own knot (never a fresh
        # origin, module docstring / `precondition_refusals`), and a
        # single-skill test schedule built to isolate a CATCH's own ladder
        # rows legitimately puts one at idx 0.
        fresh_origin = idx == 0 and skill.kind != CATCH
        if obs is not None and (skill.kind != REST or fresh_origin):
            codes = precondition_refusals(
                obs, launch=(skill.kind == THROW),
                skip_mocap=(skill.kind == REST))
            if codes:
                self.dispatched.add(idx)
                self.attempt_ended = True
                self.end_code = codes[0]
                return (['%.3f END %s at skill %d (%s): the precondition '
                        'ladder refused %s'
                        % (t_abs_s, codes[0], idx, skill.kind,
                           ', '.join(codes))], False)
        try:
            if skill.kind == CATCH:
                landing, aim_source = self._catch_aim(idx, skill, t_abs_s)
                if landing is None:
                    deadline = (float(skill.t_abs_s) - CATCH_DEADLINE_WINDOW_S
                               - float(skill.lead_s))
                    if t_abs_s < deadline:
                        # Wait for landing (owner decision 2026-09-13): only a
                        # catch with NO previous release in this schedule AND
                        # no tracker landing at all reaches this (columns'
                        # very first catch, of a ball thrown before ``t0``) --
                        # under every aim source, because the ordered rule in
                        # :meth:`_catch_aim` aims from the schedule's prior
                        # the moment a previous release exists.  NOT marked
                        # dispatched, so
                        # `tick` calls this again next tick; the window this
                        # catch eventually plans is measured from whichever
                        # tick actually installs it, exactly as
                        # `install_segment` already measures every splice
                        # window against `t_now`, not a schedule's nominal
                        # one. A deferral long enough to land past the own
                        # throw's detach cone is not a special case: by then
                        # `_snap_to_release` finds `k_s > k_rel + n_detach` and
                        # this becomes an ORDINARY splice, seeded by
                        # `state_at_knot` rather than `release_state_at_knot`
                        # -- correct, because the head has already carried the
                        # ball past its cone on its own solved trajectory by
                        # the time this install reaches it.
                        return ([], True)
                    self.dispatched.add(idx)
                    self.attempt_ended = True
                    self.end_code = NO_LANDING
                    return (['%.3f END %s: no schedule prior and no '
                            'tracker landing for ball %d by the deadline '
                            '(%.3f s) — a catch cannot be aimed at a ball '
                            'nothing in this attempt has seen or thrown'
                            % (t_abs_s, NO_LANDING, skill.ball_id, deadline)],
                           False)
                terminal = self._catch_terminal(idx, skill, landing)
            elif skill.kind == THROW:
                terminal = self._throw_terminal(idx, skill)
            else:
                terminal = self._rest_terminal(skill)
        except _NoAdmissibleCommand as exc:
            self.dispatched.add(idx)
            self.attempt_ended = True
            self.end_code = NO_ADMISSIBLE_COMMAND
            return (['%.3f END %s at skill %d (%s): %s'
                    % (t_abs_s, NO_ADMISSIBLE_COMMAND, idx, skill.kind, exc)],
                   False)

        res = self.installer(skill.kind, terminal, t_abs_s,
                             ball_id=skill.ball_id)
        self.dispatched.add(idx)
        self.results.append((idx, skill, res))
        if not res.accepted:
            self.attempt_ended = True
            self.end_code = res.code
            return (['%.3f END %s at skill %d (%s): %s'
                    % (t_abs_s, res.code, idx, skill.kind, res.message)],
                   False)
        lines = ['%.3f %s skill %d: %s'
                % (t_abs_s, skill.kind, idx, res.message)]
        if skill.kind == CATCH:
            self._live_catch = (idx, terminal, t_abs_s)
            lines.append(
                '%.3f CATCH-AIM skill %d: source=%s landing=(%.1f, %.1f, '
                '%.1f) mm t_land=%.3f%s'
                % (t_abs_s, idx, aim_source, terminal.landing_mm[0],
                   terminal.landing_mm[1], terminal.landing_mm[2],
                   _event_abs_s(CATCH, terminal),
                   (' — awaiting the measured hand ratio'
                    if aim_source == AIM_SCHEDULE_HAND else '')))
        else:
            self._live_catch = None
        self._register_outcome(idx, skill)
        return (lines, False)

    # ── outcome capture (plan § 2.5 step 6 / § 2.7) ──

    def _register_outcome(self, idx: int, skill: Skill) -> None:
        """Start tracking the outcome of the ball ``idx``'s dispatch just
        released, if it released one and anyone is listening.

        Runs immediately after :meth:`_command_u` has cached ``idx``'s
        command, so the pending row's ``x``/``u`` are exactly what commanded
        the throw — never recomputed, never the tracker's or the schedule's
        own numbers.
        """
        if self.on_experience is None:
            return
        if skill.kind == THROW:
            target, t_release = skill.target, float(skill.t_abs_s)
        elif skill.kind == CATCH and skill.then_throw is not None:
            target = skill.then_throw.target
            t_release = float(skill.then_throw.t_release_abs_s)
        else:
            return
        x, u_dy, u_apex = self._u_cache[idx]
        u = np.array([u_dy[0], u_dy[1], u_apex])
        target_xy_mm = np.asarray(target.catch_site_mm(), dtype=float)[:2]
        self._pending_outcomes.append(_PendingOutcome(
            ball_id=skill.ball_id, x=x, u=u, t_release_s=t_release,
            t_land_scheduled_s=t_release + sch.flight_s(u_apex),
            target_xy_mm=target_xy_mm,
            t_next_release_s=_next_release(self.schedule, idx, skill.ball_id)))

    def _advance_release_evidence(self, t_abs_s: float) -> List[str]:
        """Confirm every accepted release actually left the hand (PORT@R3,
        INVARIANTS.md § 8 ``ABORTED_NO_RELEASE``) -- called each tick from
        :meth:`tick`, only when ``observations`` is wired.

        Evidence must belong to THIS ball's release, not a previous flight
        still in progress when a carried throw's row was registered (a CATCH
        with a ``then_throw`` registers up to a beat before its own
        release, while the cup still carries the ball from the PRECEDING
        throw):

        * possession evidence (:attr:`observer`) -- a SEATED reading latches
          :attr:`_PendingOutcome.seated_seen`; an EMPTY reading before
          :attr:`_PendingOutcome.t_release_s` clears it (that EMPTY belongs
          to the previous ball's flight, not this release); an EMPTY reading
          AT OR AFTER ``t_release_s`` confirms the release only if
          ``seated_seen`` is set -- i.e. EMPTY must follow a SEATED sample
          taken at or after the release;
        * tracker evidence (:attr:`tracker`) -- a landing confirms the
          release only when sampled at or after ``t_release_s`` AND the
          landing's own ``t_land_abs_s`` is after ``t_release_s`` (a stale
          landing from the previous flight, still returned by a tracker
          whose correlation has not yet re-latched, must not confirm this
          release).

        Confirmation is sticky (:attr:`_PendingOutcome.release_confirmed`),
        so one good sample stands even if a later tick goes blind again (the
        same "one good sample stands" shape as :meth:`_advance_outcomes`'s
        ``best_landing``). No evidence by ``t_release + RELEASE_GRACE_S``
        aborts the attempt and drops the row -- the throw produces no
        learner row, whether or not this is the first thing to end the
        attempt this tick.
        """
        if not self._pending_outcomes:
            return []
        lines = []
        remaining = []
        for pend in self._pending_outcomes:
            if not pend.release_confirmed:
                if self.observer is not None:
                    ev = self.observer(pend.ball_id, t_abs_s)
                    if ev == bp.EVIDENCE_SEATED:
                        pend.seated_seen = True
                    elif ev == bp.EVIDENCE_EMPTY:
                        if t_abs_s < pend.t_release_s:
                            pend.seated_seen = False
                        elif pend.seated_seen:
                            pend.release_confirmed = True
                if (not pend.release_confirmed and self.tracker is not None
                        and t_abs_s >= pend.t_release_s):
                    landing = self.tracker(pend.ball_id)
                    if (landing is not None
                            and float(landing.t_land_abs_s)
                            > pend.t_release_s):
                        pend.release_confirmed = True
            if (not pend.release_confirmed
                    and t_abs_s >= pend.t_release_s + RELEASE_GRACE_S):
                if not self.attempt_ended:
                    self.attempt_ended = True
                    self.end_code = ABORTED_NO_RELEASE
                    lines.append(
                        '%.3f END %s: no release evidence for ball %d by '
                        't_release + %.1f s -- no learner row'
                        % (t_abs_s, ABORTED_NO_RELEASE, pend.ball_id,
                           RELEASE_GRACE_S))
                continue                     # dropped either way
            remaining.append(pend)
        self._pending_outcomes = remaining
        return lines

    @staticmethod
    def _outcome_window(pend: _PendingOutcome) -> Tuple[float, float]:
        """``(t_open, finalise_at)`` — the instants bounding ``pend``'s verdict.

        ``finalise_at`` is :data:`CAUGHT_WINDOW_S` after the LATER of the
        scheduled landing and the observed one, with the observed one clamped
        to :data:`CAUGHT_LAND_DEFER_CAP_S` past the schedule so a diverged
        tracker estimate can defer the row by a bounded amount and no more.
        Taking the LATER of the two (rather than the observed one alone) keeps
        a tracker that under-predicts from finalising the row before the ball
        has physically arrived.

        ``t_open`` is :data:`CAUGHT_LEAD_S` before the EARLIER of the two, for
        the opposite reason: the crossing is an estimate of a plane the cup rim
        reaches first, and on 2026-09-15 the sensor seated 39-48 ms ahead of it
        on every throw.

        Both are pure functions of the row, so the window a tick is measured
        against WIDENS as the tracker sharpens its landing and never moves
        arbitrarily: every call re-derives it from ``best_landing``, which is
        itself one-way (a landing already accepted is never replaced by a
        worse one).
        """
        t_sched = float(pend.t_land_scheduled_s)
        # The lead can never reach back past the RELEASE: before it, a SEATED
        # cup is this ball still sitting in the hand, not this ball caught.
        # Slack at the operating point (the flight is >= ~0.6 s against a
        # 0.10 s lead), but a shorter flight must not turn the lead into a
        # verdict taken on the throw -- the same class of hazard
        # `_advance_release_evidence` guards for the release evidence.
        floor = float(pend.t_release_s)
        if pend.best_landing is None:
            t_open = max(floor,
                         SkillExecutor._landing_instant(pend) - CAUGHT_LEAD_S)
            return (t_open, SkillExecutor._bound_by_next_release(
                pend, t_sched + CAUGHT_WINDOW_S, t_open))
        t_obs = float(pend.best_landing.t_land_abs_s)
        anchor = max(t_sched, min(t_obs, t_sched + CAUGHT_LAND_DEFER_CAP_S))
        t_open = max(floor, SkillExecutor._landing_instant(pend) - CAUGHT_LEAD_S)
        return (t_open, SkillExecutor._bound_by_next_release(
            pend, anchor + CAUGHT_WINDOW_S, t_open))

    @staticmethod
    def _next_release_bound(pend: _PendingOutcome) -> Optional[float]:
        """The instant this row must be finished with the ball by —
        ``t_next_release_s - OUTCOME_NEXT_RELEASE_EPS_S``, or ``None`` when
        the schedule never throws it again. The ONE definition of that margin,
        shared by :meth:`_landing_instant` (the freeze),
        :meth:`_consider_landing` (test 4) and
        :meth:`_bound_by_next_release` (the verdict close)."""
        if pend.t_next_release_s is None:
            return None
        return float(pend.t_next_release_s) - OUTCOME_NEXT_RELEASE_EPS_S

    @staticmethod
    def _bound_by_next_release(pend: _PendingOutcome, finalise_at: float,
                               t_open: float) -> float:
        """``finalise_at``, pulled back so the window closes BEFORE this
        ball's next scheduled release (:data:`OUTCOME_NEXT_RELEASE_EPS_S`).

        The window's only business past the landing is the SEATED latch, and
        the ball is physically back in the cup for that whole interval -- so
        closing at the re-release costs nothing a catch needs, while a window
        that reached past it would be sampling a ball in its NEXT flight (the
        2026-09-16 contamination, see :data:`OUTCOME_NEXT_RELEASE_EPS_S`).

        Never pulled back before ``t_open``: a schedule whose re-release
        crowds the landing would otherwise invert the window and finalise a
        row before the ball had arrived. The bound is skipped entirely when
        the schedule never throws this ball again.
        """
        bound = SkillExecutor._next_release_bound(pend)
        if bound is None:
            return finalise_at
        return max(t_open, min(finalise_at, bound))

    @staticmethod
    def _landing_instant(pend: _PendingOutcome) -> float:
        """The single instant this row treats as ``pend``'s expected landing
        -- the EARLIEST of the scheduled crossing, the best observed crossing,
        and this ball's next release less
        :data:`OUTCOME_NEXT_RELEASE_EPS_S`.

        Three uses, deliberately ONE instant: it anchors ``t_open``
        (:meth:`_outcome_window`), it is the FREEZE instant
        (:meth:`_consider_landing`), and it decides which of several open rows
        a single SEATED sample belongs to (:meth:`_advance_outcomes`).

        The next-release term matters on a CROWDED schedule -- one whose
        re-release falls BEFORE the scheduled landing (a late-arriving plant,
        or a beat shorter than the commanded flight). Without it the freeze
        and the window bound would disagree: :meth:`_bound_by_next_release`
        would close the verdict at the re-release while the freeze still sat
        at the later scheduled landing, leaving a gap in which the NEXT
        flight's estimate was still admissible. Both now stop at the same
        instant, on the same margin.
        """
        cands = [float(pend.t_land_scheduled_s)]
        if pend.best_landing is not None:
            cands.append(float(pend.best_landing.t_land_abs_s))
        bound = SkillExecutor._next_release_bound(pend)
        if bound is not None:
            cands.append(bound)
        return min(cands)

    @staticmethod
    def _consider_landing(pend: _PendingOutcome, landing: Optional[Landing],
                          t_abs_s: float) -> bool:
        """Offer ``landing``, sampled at ``t_abs_s``, as ``pend``'s outcome;
        return whether it was accepted.

        **Only a CONVERGED ballistic fit may become a row** (2026-09-18,
        ``Landing.from_fit``): the Kalman fallback's crossing runs 0.06-0.20 s
        late and its apex with it, and 16 of the 22 rows written on 2026-09-17
        had no converged fit at all. The catch AIM path is deliberately NOT
        held to this -- it keeps aiming off whatever estimate exists.

        **The observation FREEZES at the crossing** (2026-09-16). Then five
        tests, every one about "is this estimate about THIS flight?":

        1. ``t_land_abs_s > t_release_s`` — a landing from the PREVIOUS flight,
           still cached by a tracker whose correlation has not re-latched, is
           not this release's outcome (:meth:`_advance_release_evidence` guards
           the same class for the release evidence).
        2. The sample is taken at least :data:`OUTCOME_GUARD_S` BEFORE the
           crossing it is itself predicting. This was ``abs(...) >
           OUTCOME_GUARD_S``, which also admitted samples taken well AFTER
           their own predicted crossing — i.e. of a ball that has already
           landed, which is where a re-thrown ball's estimate lives.
        3. The ball has not landed yet: ``t_abs_s`` is before
           :meth:`_landing_instant` (the earliest of the scheduled crossing,
           the best-so-far observed one, and this ball's next release less
           :data:`OUTCOME_NEXT_RELEASE_EPS_S`). Past that instant the row is
           FROZEN — no later tick can move it. This is the structural fix for
           the contamination: a chained self-toss catches and re-throws the
           SAME ball on ONE continuous tracker track, so from the catch
           onwards every estimate for that id is about the NEXT flight, and
           the old rule (last estimate before finalise wins) wrote those —
           2.23 s and 1.16 s flights for a 0.857 s command, ``armB-090``
           attempt 1, 2026-09-16.
        4. The estimate's own crossing is BEFORE this ball's next scheduled
           release (less :data:`OUTCOME_NEXT_RELEASE_EPS_S`). A landing at or
           after the instant the ball leaves the cup again belongs to the next
           flight no matter which tick it was sampled on — the freeze (test 3)
           only rules out samples taken after the landing, and on a CROWDED
           schedule (re-release before the scheduled landing) an in-flight
           sample can still point past it. Both tests stop at the same
           instant, on the same margin (:meth:`_landing_instant`,
           :meth:`_bound_by_next_release`).
        5. The apex the estimate's arrival speed implies
           (``schedule.apex_from_vz``) is inside ``memory.APEX_RATIO_BAND`` x
           the COMMANDED apex. An estimate outside it is not about this throw
           — 2026-09-16's rows recorded 2.23 s of flight (5.4× the commanded
           apex) for a 0.857 s command. Testing this at ADMISSION, and not
           only at finalise, is what keeps a garbage estimate from moving test
           3's freeze instant: an accepted "lands 30 ms from now" would
           otherwise close the row 30 ms after the release, which is exactly
           how three genuine rows became 0.02–0.05 s flights in the first
           replay of this fix.

        Among the estimates that survive 1–5 the LAST one wins — the most
        mature view of a flight still in progress. A "prefer the smallest
        lead" rule was tried first and REJECTED on the bag replay
        (``tools/probes/outcome_landing_replay.py``, 2026-09-16): the tracker's
        first estimates just after release predict a crossing almost
        immediately (the ball is barely above the plane and slow), so the
        smallest lead in a flight is usually that opening garbage — it turned
        three genuine rows into 0.02–0.05 s flights. What a late wild estimate
        needs is a PHYSICAL test, not a vantage-point one, and that is
        ``memory.APEX_RATIO_BAND`` at finalise.

        The row is one-way in the sense that matters: once it has a landing
        it never loses one, and after the crossing it never gains a different
        one — so the verdict window :meth:`_outcome_window` derives from it
        stops moving at the landing instead of following the ball into its
        next flight.
        """
        if landing is None:
            return False
        if not landing.from_fit:
            pend.rejected_reason = 'no converged ballistic fit'
            pend.n_rejected += 1
            return False
        t_land = float(landing.t_land_abs_s)
        if t_land <= pend.t_release_s:
            return False                       # test 1: the previous flight
        if t_land - t_abs_s < OUTCOME_GUARD_S:
            return False                       # test 2: at/after its crossing
        if t_abs_s >= SkillExecutor._landing_instant(pend):
            return False                       # test 3: FROZEN at the crossing
        apex = _observed_apex_m(landing)
        u_apex = float(pend.u[2])
        bound = SkillExecutor._next_release_bound(pend)
        if bound is not None and t_land >= bound:
            # test 4: a landing at or after this ball's NEXT release is the
            # next flight's, whatever the tick it was sampled on.
            pend.rejected_apex_m = apex
            pend.rejected_reason = ('at or after this ball\'s next release '
                                     '(%.3f s after this one)'
                                     % (bound - pend.t_release_s,))
            pend.n_rejected += 1
            return False
        if not apex_in_band(u_apex, apex):
            pend.rejected_apex_m = apex        # test 5: not a physical apex
            pend.rejected_reason = ('outside [%.3f, %.3f] of commanded %.3f m'
                                     % (APEX_RATIO_BAND[0] * u_apex,
                                        APEX_RATIO_BAND[1] * u_apex,
                                        u_apex))
            pend.n_rejected += 1
            return False
        pend.best_landing = landing
        pend.best_lead_s = t_land - t_abs_s
        return True

    def _advance_outcomes(self, t_abs_s: float) -> List[str]:
        """Sample the tracker AND the possession observer for every pending
        outcome, and finalise the ones whose verdict window has closed.

        The tracker is only sampled at or after ``t_release_s``, and which
        estimates count as THIS flight's outcome is
        :meth:`_consider_landing`'s contract -- in particular the observation
        FREEZES at the crossing, so a ball re-thrown on the same continuous
        track cannot overwrite the row it has already earned (the 2026-09-16
        contamination).

        The possession observer is read EVERY tick inside a row's verdict
        window (:meth:`_outcome_window`) and
        :attr:`_PendingOutcome.caught_seen` latches on the first SEATED
        reading, so the verdict no longer depends on the cup being seated at
        one particular instant -- the 2026-09-16 defect this replaces.

        The observer is ONE physical cup sensor blind to ``ball_id`` -- with
        two balls in flight (columns) a row's window can be up to
        ``CAUGHT_LEAD_S + CAUGHT_LAND_DEFER_CAP_S + CAUGHT_WINDOW_S`` wide, so
        two rows' windows can be open on the same tick. A SEATED sample is
        therefore attributed to at most ONE row per tick: the one whose
        :meth:`_landing_instant` is nearest this tick, i.e. whichever ball
        physically landed most recently -- never latched onto every open row.
        """
        if not self._pending_outcomes:
            return []
        lines = []
        remaining = []
        for pend in self._pending_outcomes:
            if self.tracker is not None and t_abs_s >= pend.t_release_s:
                self._consider_landing(pend, self.tracker(pend.ball_id),
                                       t_abs_s)

        # Read the cup BEFORE the finalise test, so the closing tick -- which
        # is inside the window by construction -- still counts. One read
        # serves at most one row: the nearest-landing open row.
        if self.observer is not None:
            open_rows = [
                pend for pend in self._pending_outcomes
                if not pend.caught_seen
                and self._outcome_window(pend)[0] <= t_abs_s
                <= self._outcome_window(pend)[1]
            ]
            if open_rows:
                nearest = min(
                    open_rows,
                    key=lambda p: abs(t_abs_s - self._landing_instant(p)))
                if self.observer(nearest.ball_id, t_abs_s) == CAUGHT_EVIDENCE:
                    nearest.caught_seen = True
                    nearest.t_seat_s = float(t_abs_s)

        for pend in self._pending_outcomes:
            t_open, finalise_at = self._outcome_window(pend)
            if t_abs_s < finalise_at:
                remaining.append(pend)
                continue
            lines.extend(self._finalise_outcome(pend, finalise_at))
        self._pending_outcomes = remaining
        return lines

    def _finalise_outcome(self, pend: _PendingOutcome,
                          finalise_at: float) -> List[str]:
        """Build and hand off ``pend``'s :class:`~jugglebot.motion.skills.
        memory.Experience`, or drop it — a blind flight teaches the memory
        nothing (plan § 2.5 step 6)."""
        u_apex = float(pend.u[2])
        if pend.best_landing is None:
            if pend.rejected_reason:
                seen = ('' if pend.rejected_apex_m is None
                        else 'observed apex %.3f m ' % (pend.rejected_apex_m,))
                return ['%.3f OUTCOME ball %d: no row: %s%s (%d estimate(s) '
                        'refused)' % (finalise_at, pend.ball_id, seen,
                                      pend.rejected_reason, pend.n_rejected)]
            return ['%.3f OUTCOME ball %d: no landing estimate was ever '
                    'observed — no row' % (finalise_at, pend.ball_id)]
        apex_obs_m = _observed_apex_m(pend.best_landing)
        # The PHYSICAL band (``memory.APEX_RATIO_BAND``): an observed apex
        # that is not a multiple near 1 of the commanded one is not an
        # observation of this throw at all -- it is the next flight's landing,
        # or a diverged filter. Belt and braces with the freeze in
        # `_consider_landing`, and the last line of defence before the row
        # reaches the learner's memory (which refuses it again on append).
        if not apex_in_band(u_apex, apex_obs_m):
            return ['%.3f OUTCOME ball %d: no row: observed apex %.3f m '
                    'outside [%.3f, %.3f] of commanded %.3f m'
                    % (finalise_at, pend.ball_id, apex_obs_m,
                       APEX_RATIO_BAND[0] * u_apex,
                       APEX_RATIO_BAND[1] * u_apex, u_apex)]
        landing_xy_m = ((np.asarray(pend.best_landing.pos_mm, dtype=float)[:2]
                        - pend.target_xy_mm) / 1000.0)
        y = np.array([landing_xy_m[0], landing_xy_m[1], apex_obs_m])
        # The LATCH, not a fresh sample: by ``finalise_at`` a chained catch has
        # often already re-thrown the ball, so the cup at this instant says
        # nothing about whether it was caught (see :meth:`_outcome_window`).
        caught = bool(pend.caught_seen)
        exp = Experience(x=pend.x, u=pend.u, y=y, t_abs_s=pend.t_release_s,
                         ball_id=pend.ball_id, caught=bool(caught))
        self.on_experience(exp)
        phase = ('' if pend.t_seat_s is None
                 else ' seat=%+.3f s vs scheduled landing'
                 % (pend.t_seat_s - float(pend.t_land_scheduled_s)))
        return ['%.3f OUTCOME ball %d: y=(%.4f, %.4f) m apex=%.4f m '
                'caught=%s%s'
                % (finalise_at, pend.ball_id, y[0], y[1], y[2], caught, phase)]

    def _resend_hand_corrected_catch(self, t_abs_s: float) -> List[str]:
        """:data:`AIM_SCHEDULE_HAND`: re-aim the committed CATCH ONCE, from
        the MEASURED hand launch speed of its own ball's throw.

        The catch is already committed on the theoretical aim (the schedule's
        commanded landing), so this is strictly a refinement and every
        failure path keeps that aim: no ratio yet (retried next tick), a
        ratio the monitor will not vouch for (``None``, permanently), a
        scaled flight that misses the catch plane, or a re-send that no
        longer fits before touch-down. "Once" is the point -- ``r`` is a
        property of a stroke that has already happened, so a second call
        would re-install the same landing and pay a solve for it.

        The two timing fences are :meth:`_resend_live_catch`'s, for the same
        two physical facts: nothing re-sends inside ``catch_freeze_s`` of
        touch-down (the hand is already decelerating into the ball), and
        nothing re-sends once ``now + lead_s`` leaves less than
        :data:`MIN_WINDOW_S` before the landing (a solve there can only
        refuse ``WINDOW_TOO_SHORT``). Both are checked against the CORRECTED
        landing as well as the committed one -- a correction that pulls
        touch-down EARLIER (r < 1) can land inside a freeze the committed aim
        cleared.
        """
        if self._live_catch is None or self.launch_ratio is None:
            return []
        idx, terminal, _t_last = self._live_catch
        if idx in self._hand_corrected:
            return []
        skill = self.schedule.skills[idx]
        if _previous_release(self.schedule, idx, skill.ball_id) is None:
            self._hand_corrected.add(idx)
            return []
        if self._resend_too_late(t_abs_s, skill,
                                 _event_abs_s(CATCH, terminal)):
            self._hand_corrected.add(idx)
            return ['%.3f CATCH-AIM-LATE skill %d: no hand-measured '
                    'correction arrived in time — the theoretical aim stands '
                    '(t_land %.3f)' % (t_abs_s, idx, terminal.t_land_s)]
        landing, r = self._hand_ratio_landing(idx, skill, t_abs_s)
        if r is None:
            return []
        self._hand_corrected.add(idx)
        if landing is None:
            return ['%.3f CATCH-AIM-HAND-INFEASIBLE skill %d: r=%.3f gives no '
                    'arrival at the catch plane — the theoretical aim stands'
                    % (t_abs_s, idx, r)]
        dt_s = float(landing.t_land_abs_s) - _event_abs_s(CATCH, terminal)
        if self._resend_too_late(t_abs_s, skill,
                                 float(landing.t_land_abs_s)):
            return ['%.3f CATCH-AIM-LATE skill %d: the hand-measured '
                    'correction (r=%.3f, Δt=%+.3f s) would splice too late — '
                    'the theoretical aim stands' % (t_abs_s, idx, r, dt_s)]
        new_terminal = self._catch_terminal(idx, skill, landing)
        res = self.installer(CATCH, new_terminal, t_abs_s,
                             ball_id=skill.ball_id)
        self.results.append((idx, skill, res))
        if not res.accepted:
            # The committed catch stands — a refused re-aim is strictly
            # better than no catch, so the attempt continues.
            self._live_catch = (idx, terminal, t_abs_s)
            return ['%.3f CATCH-AIM-HAND-REFUSED %s skill %d: r=%.3f, '
                    'Δt=%+.3f s — %s'
                    % (t_abs_s, res.code, idx, r, dt_s, res.message)]
        self._live_catch = (idx, new_terminal, t_abs_s)
        return ['%.3f CATCH-AIM skill %d: source=%s r=%.3f Δt=%+.3f s '
                'landing=(%.1f, %.1f, %.1f) mm t_land=%.3f'
                % (t_abs_s, idx, AIM_SCHEDULE_HAND, r, dt_s,
                   landing.pos_mm[0], landing.pos_mm[1], landing.pos_mm[2],
                   landing.t_land_abs_s)]

    def _resend_too_late(self, t_abs_s: float, skill: Skill,
                         t_land_abs_s: float) -> bool:
        """True when a re-send at ``t_abs_s`` for a touch-down at
        ``t_land_abs_s`` is past one of the two fences (freeze, window floor)
        — see :meth:`_resend_hand_corrected_catch`. Inside this layer a
        terminal's time field holds the ABSOLUTE instant (``_event_abs_s``),
        which is the clock both fences are measured on."""
        if t_abs_s >= t_land_abs_s - self.catch_freeze_s:
            return True
        return (t_land_abs_s - (t_abs_s + float(skill.lead_s))
                < MIN_WINDOW_S - 1e-12)

    def _resend_live_catch(self, t_abs_s: float) -> List[str]:
        """Re-aim the committed CATCH from a LATER converged fit — the step
        that makes :data:`AIM_TRACKER` worth having, because the prior the
        catch was dispatched on cannot know this throw's release slip
        (0.019-0.137 s, 2026-09-17) and the fit can.

        Five fences, one physical fact each. Two are TIMING and unchanged
        since 2026-09-12: nothing is re-sent inside :attr:`catch_freeze_s` of
        touch-down (the hand is already decelerating into the ball and a
        re-solve there is a change nothing can execute), and nothing is
        re-sent once ``now + lead_s`` leaves less than :data:`MIN_WINDOW_S`
        before touch-down (a solve there can only refuse
        ``WINDOW_TOO_SHORT``; measured 2026-09-12, ``sim/skills_gate.py``, 75
        such refusals per 8 throws at the 0.278 s transit, each a wasted
        ~25 ms solve on the orchestrator thread). Three are WORTH-IT: the
        estimate must come from the converged fit (an unfitted Kalman
        crossing is 0.06-0.20 s late and would re-aim the catch AWAY from the
        prior), it must have moved beyond :attr:`resend_pos_tol_mm` /
        :attr:`resend_t_tol_s`, and one catch may not pay more than
        :attr:`resend_max_per_catch` solves. Every fence that turns away a
        real candidate says so once, with the delta it turned away.
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
        if (float(terminal.t_land_s) - (t_abs_s + float(skill.lead_s))
                < MIN_WINDOW_S - 1e-12):
            self._live_catch = None
            return []
        landing = self._tracked_landing(idx, skill)
        if landing is None:
            return []
        moved_mm = float(np.max(np.abs(
            np.asarray(landing.pos_mm, dtype=float) - terminal.landing_mm)))
        moved_s = float(landing.t_land_abs_s) - float(terminal.t_land_s)
        if not landing.from_fit:
            return self._resend_declined(t_abs_s, idx, 'NO-CONVERGED-FIT',
                                         moved_mm, moved_s)
        if (moved_mm <= self.resend_pos_tol_mm
                and abs(moved_s) <= self.resend_t_tol_s):
            return self._resend_declined(t_abs_s, idx, 'WITHIN-TOLERANCE',
                                         moved_mm, moved_s)
        if self._resend_counts.get(idx, 0) >= self.resend_max_per_catch:
            return self._resend_declined(t_abs_s, idx, 'CAP-SPENT',
                                         moved_mm, moved_s)
        new_terminal = self._catch_terminal(idx, skill, landing)
        res = self.installer(skill.kind, new_terminal, t_abs_s,
                             ball_id=skill.ball_id)
        self.results.append((idx, skill, res))
        self._resend_counts[idx] = self._resend_counts.get(idx, 0) + 1
        if not res.accepted:
            # The committed catch stands — a refused RE-aim is strictly better
            # than no catch, so the attempt continues.
            self._live_catch = (idx, terminal, t_abs_s)
            return ['%.3f RESEND-REFUSED %s at skill %d: the fit moved the '
                    'landing %.1f mm / %+.3f s — %s'
                    % (t_abs_s, res.code, idx, moved_mm, moved_s, res.message)]
        self._live_catch = (idx, new_terminal, t_abs_s)
        return ['%.3f RESEND skill %d: the fit moved the landing %.1f mm / '
                '%+.3f s (re-aim %d of %d) — %s'
                % (t_abs_s, idx, moved_mm, moved_s, self._resend_counts[idx],
                   self.resend_max_per_catch, res.message)]

    def _resend_declined(self, t_abs_s: float, idx: int, reason: str,
                         moved_mm: float, moved_s: float) -> List[str]:
        """One line per ``(catch, reason)`` — the tracker answers on every
        tick, so an unconditional line would repeat the same sentence at the
        re-send cadence (~40 Hz worth, gated only by
        :attr:`resend_min_interval_s`) and bury the dispatch lines the
        operator reads. Once is enough to answer "why was the committed catch
        not refined", and the delta says how much was left on the table."""
        key = (idx, reason)
        if key in self._resend_notes:
            return []
        self._resend_notes.add(key)
        return ['%.3f RESEND-SKIPPED %s skill %d: the tracker landing is '
                '%.1f mm / %+.3f s from the committed aim'
                % (t_abs_s, reason, idx, moved_mm, moved_s)]
