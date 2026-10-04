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
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.skills import admissible as adm
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import schedule as sch
from jugglebot.motion.skills.report import ThrowReport, release_error_s
from jugglebot.motion.skills.sites import RELEASE_CUP_Z_MM
from jugglebot.motion.skills.memory import (APEX_RATIO_BAND, Experience,
                                            apex_in_band)
from jugglebot.motion.skills.schedule import (HANDOFF_LEAD_KNOTS,
                                               HANDOFF_LEAD_S, LEAD_KNOTS,
                                               LEAD_S, MIN_WINDOW_KNOTS,
                                               MIN_WINDOW_S, WIRE_READ_KNOTS,
                                               Schedule, Skill, ball_label)
from jugglebot.motion.skills.segments import (CATCH, REST, THROW, CatchTerminal,
                                              RestTerminal, Segment,
                                              SegmentConfig, ThrowAfterCatch,
                                              ThrowTerminal)
from jugglebot.motion.trajectory import ballistics_bc
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory import tilt_geometry as tg
from jugglebot.motion.trajectory.cycle_plan import CyclePlan

# ── Refusal codes this layer owns ────────────────────────────────────────────
#: The solve finished so late that the splice knot is already behind the wire.
SPLICE_TOO_LATE = 'SPLICE_TOO_LATE'
#: A FRESH origin's solve ran long enough that knot 0 would already be stale
#: by the time the block reaches the wire, AND the segment carries an
#: absolute event (a THROW's release or a CATCH's touch-down) that cannot be
#: silently slid later — see :func:`install_segment`'s fresh-origin lateness
#: check (2026-09-28, R4 sitting-1 fact 5: a REST-kind segment in the same
#: situation is REBASED instead, because a rest carries no event).
ORIGIN_TOO_LATE = 'ORIGIN_TOO_LATE'
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

#: Seconds before touch-down inside which the QP enforces ``CUP_CONTACT_ACC``
#: (a floor on axial deceleration, the cup-contact contract) — the SAME
#: generated constant ``cup_cycle.CONTACT_WINDOW_LEAD_S`` reads
#: (``hw.JB_TRAJ_CUP_CONTACT_WINDOW_LEAD_S``, τ = 0.125 s), restated here
#: rather than imported from ``cup_cycle`` because this layer's only use of
#: it is a RE-SEND timing fence, not the constraint itself — reading the one
#: generated source twice is not a second copy of the number (plan § 0: no
#: ``motion/`` module owns ``hw.*`` more than another). A re-send whose
#: splice base falls inside this window is declined BEFORE the solve
#: (:meth:`SkillExecutor._resend_live_catch` /
#: :meth:`_resend_hand_corrected_catch`): a splice that opens inside the
#: window hands the QP a re-solve with no runway left to reach the contact
#: floor from whatever state the re-aim seeds, so the solve would only
#: refuse ``CUP_CONTACT_ACC`` — paying it is a wasted 40-130 ms on the
#: orchestrator thread for a result the timing alone already predicts.
CUP_CONTACT_WINDOW_LEAD_S = float(hw.JB_TRAJ_CUP_CONTACT_WINDOW_LEAD_S)

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
#: RETIRED as an admission test on 2026-09-20 (a converged fit is the same
#: parabola at its crossing); kept as the band ``tools/probes/
#: outcome_landing_replay.py`` reproduces the 2026-09-16 rule with.
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
#: The FIRMWARE refused a streamed hand lane (its ``sched_refused`` counter
#: incremented while the attempt was running) -- fail-closed, 2026-09-18.
#:
#: After a hand-less hold (every attempt END installs one) the can-bridge's hand
#: group sits in ``SCHED_HOLD`` at its last knot and promotes a resumed lane only
#: if the frame at the handover instant is within ``SCHED_RESUME_TOL_POS_HAND_REV``
#: = 0.05 rev of the held state (``leg_interp.cpp:497-520``); otherwise it LATCHES
#: the hold, counts ``sched_refused``, and the guard measures the REFUSED command
#: against the encoder (``leg_interp.cpp:1226-1228``) so the deviation grows with
#: every knot the plan walks on -- MAX_DEVIATION E-STOPPED the machine 0.61 s in on
#: 2026-09-18.  The lane is sized to stay INSIDE that envelope
#: (``schedule.floor_lift_s``); this code is what happens when it is outside it
#: anyway.  The attempt ENDS and the node installs a hold, which is a HAND-LESS
#: plan: no further hand frame, so the refused command stops walking and the
#: deviation stops growing.  At the homing move's own ceiling (2.5 rev/s) the
#: 2.5 rev guard band is ~1 s away, and the counter reaches this code within one
#: 10 Hz ``/link_status`` publish -- so the hold lands with most of the band
#: unspent.
HAND_LANE_REFUSED = 'HAND_LANE_REFUSED'

#: No evidence the ball left by ``t_release + RELEASE_GRACE_S`` -- the throw
#: produces no learner row (``toss_sequencer.py::_step_throwing``'s
#: ``now >= self._release_deadline`` -> ``self._abort('NO_RELEASE')``).
ABORTED_NO_RELEASE = 'ABORTED_NO_RELEASE'

#: The survivor drop policy (owner decision D3, 2026-09-30, R5 rescope): one
#: ball's release was never confirmed (:data:`ABORTED_NO_RELEASE`'s own
#: deadline) while ANOTHER ball still has a CATCH left in the schedule --
#: rather than end the attempt on the dropped ball, keep catching the
#: survivor: its next CATCH is dispatched WITHOUT the throw it would have
#: carried, then a closing REST at its site
#: (:meth:`SkillExecutor._advance_release_evidence`). This code names that
#: this attempt's clean finish followed a drop, not a fault-free pattern.
DROPPED_SURVIVOR_STOPPED = 'DROPPED_SURVIVOR_STOPPED'

#: A CATCH's own evidence rule (R5 sitting 4, 2026-10-04 evening,
#: ``report_a1.md`` Q3): the ball this CATCH was aimed at never reached the
#: cup -- the possession sensor read EMPTY on every valid sample across the
#: catch's whole evidence window (:meth:`SkillExecutor._advance_catch_
#: evidence`). Distinct from :data:`ABORTED_NO_RELEASE` (a RELEASE never
#: confirmed) and from a normal miss the OUTCOME ladder would score (which
#: needs a release row to exist at all -- a feed catch with no carried
#: release registers none, so a missed feed ran to ``ABORTED_NO_RELEASE``/
#: ``DROPPED_SURVIVOR_STOPPED`` on its carried throw's timeout instead, one
#: full beat later than the cup already knew). Same D3 shape as the release
#: drop: a survivor ball still with a CATCH left keeps being caught (its own
#: undispatched schedule continues, ``attempt_ended`` stays False); no
#: survivor ends the attempt through the normal stop path
#: (``skill_node._maybe_hold_pending_event`` / ``stop_terminals``).
MISSED_CATCH = 'MISSED_CATCH'

#: Seconds a throw's release evidence may lag ``t_release`` before the attempt
#: aborts ``ABORTED_NO_RELEASE``. Restated, not imported, from
#: ``toss_sequencer.TOSS_RELEASE_GRACE_S`` (0.5 s) -- that module dies at R4.
RELEASE_GRACE_S = 0.5

#: How long AFTER a release a possession SEATED reading may still arm
#: :attr:`_PendingOutcome.seated_seen` for THAT release's own confirmation
#: (R5 sitting 4, 2026-10-04 evening, ``report_a1.md`` Q4). A SEATED sample
#: taken later than this can only be another ball's arrival or that ball's
#: own settle chatter -- never evidence this release happened. Measured: a
#: seated ball's raw bit falls 0.045-0.057 s AFTER its release (the cup
#: rim clears before the sensor's own debounce settles), while the other
#: ball's earliest arrival in columns is release + 0.34 s -- so 0.10 s clears
#: the real fall-time with margin and sits a long way inside the other
#: ball's earliest possible seat. Without this gate, a SEATED sample from the
#: NEXT ball's arrival (inside this row's :data:`RELEASE_GRACE_S` window)
#: could falsely confirm a release that never happened (R5 sitting 4,
#: attempts 1/9/10/16/R1: ball A's seat-plus-settle-flicker at the OTHER
#: site falsely confirmed ball B's empty carried throw).
RELEASE_SEAT_EPS_S = 0.10

#: A tracker landing is release evidence only when it comes from a CONVERGED
#: fit (:attr:`Landing.from_fit`) whose touch-down lies within this of the
#: throw's scheduled one (2026-09-29). At the R4 gate sitting a dropped ball
#: bounced beside the cup, the tracker's unfitted estimate of it confirmed the
#: EMPTY throw that followed, and the attempt ran on to two more empty throws.
#: Measured on that sitting's bag (``/throw_announcements`` against
#: ``/balls.landing_from_fit``, scratchpad ``probe_fit_latency.py``,
#: 2026-09-29):
#:
#: * all 106 real releases had a converged fit before landing, and all 11
#:   releases without one were empty throws after drops;
#: * the fit arrives quickly: release -> first fit p50 0.176 s, p95 0.227 s,
#:   max 0.406 s, inside :data:`RELEASE_GRACE_S`;
#: * the quantity THIS tolerance gates: over those releases' 18 219 fit
#:   samples, fitted touch-down minus scheduled is +0.027..+0.066 s (p50
#:   +0.044 s; the plant throws a little high). So 0.2 s is 3x the worst real
#:   case, and far from the next flight, a beat away.
#:
#: The time check keeps a bounce that flies a ballistic arc of its own from
#: converging into evidence.
RELEASE_FIT_TOL_S = 0.2

# ── The MISSED_CATCH evidence rule (R5 sitting 4, 2026-10-04 evening,
# ``report_a1.md`` Q3) ────────────────────────────────────────────────────
#
# A CATCH's own window: [t_open, t_close], armed only when both ends can be
# computed and leave a non-degenerate window; fires :data:`MISSED_CATCH` at
# the first tick >= ``t_close + MISSED_CATCH_DECIDE_LAG_S`` iff the cup never
# read RAW-SEATED anywhere inside the window AND the sensor was demonstrably
# live over it (see :meth:`SkillExecutor._register_catch` /
# :meth:`SkillExecutor._advance_catch_evidence`). Replayed on the 2026-10-04
# sitting (``tools/probes/missed_catch_fixture.py``,
# ``tests/motion/fixtures/missed_catch_20261004.csv``, B1c's raw-bit fix):
# fires 14/14 misses, 0/8 caught feeds over 22 attempts.

#: How long AFTER the latest scheduled release of ANY ball before this
#: catch's landing ``L`` the evidence window opens. A seated ball's raw bit
#: falls 0.045-0.057 s after its OWN release (the same measurement
#: :data:`RELEASE_SEAT_EPS_S` is sized from) -- opening any earlier would
#: read the previous ball still mid-fall-into-the-cup as "not yet seated"
#: evidence for THIS catch, which it is not. The DEBOUNCED ``held`` twin
#: falls later still (~0.14 s after release) and is why :class:`SeatWindow`
#: does not count it: a 0.10 s margin against the RAW fall clears the
#: previous ball; the same margin against the debounced fall would not,
#: and `held` adds no sensitivity a real seat's raw bit hasn't already
#: provided.
MISSED_CATCH_OPEN_AFTER_RELEASE_S = 0.10

#: How long after a carried release ``R`` (this CATCH's own ``then_throw``)
#: the window closes, when one exists. Margin: the latest observed seat on a
#: catch carrying a throw was ``R + 0.028`` s (n=11, real feed catches) --
#: 92 ms (R + 0.12) - (R + 0.028) inside this close.
MISSED_CATCH_CLOSE_AFTER_RELEASE_S = 0.12

#: How long after the scheduled landing ``L`` the window closes when this
#: CATCH carries no throw (a plain, final catch). Margin: the latest
#: observed seat on a plain catch was ``L + 0.358`` s -- 142 ms inside this
#: close.
MISSED_CATCH_CLOSE_AFTER_LANDING_S = 0.50

#: The window's close is pulled back to sit this far before the OTHER ball's
#: next scheduled landing (whichever comes first), so the window never reads
#: the neighbour's own arrival as this catch's evidence -- on columns this
#: caps the close at ``L + 0.38`` (a 0.579 s beat, so the neighbour lands at
#: ``L + 0.48``).
MISSED_CATCH_OTHER_LANDING_GUARD_S = 0.10

#: Seconds after ``t_close`` the rule actually fires -- one orchestrator
#: tick of slack so the window's own last sample has definitely arrived
#: before the verdict is taken.
MISSED_CATCH_DECIDE_LAG_S = 0.03

#: The sensor must have reported at least this fraction of the samples a
#: live 100 Hz feed would produce over the window's span, or the window's
#: evidence is BLIND (no verdict either way) rather than a miss.
MISSED_CATCH_MIN_VALID_FRAC = 0.8

#: The largest gap (s) between consecutive valid samples the window may
#: contain and still count as live. Matches the hand-telemetry staleness
#: family already used elsewhere in this plan arc (0.05 s).
MISSED_CATCH_MAX_GAP_S = 0.05

#: The stop REST's first choice (:meth:`SkillExecutor.stop_terminals`) is at
#: rest this long BEFORE the next pending release -- two knots, so the REST's
#: own event precedes the release and ``_snap_to_release`` has nothing to snap
#: the splice to: the throw never runs.
#:
#: It must also keep the splice clear of the release's PRE-RELEASE hold band
#: (``unified_cycle`` refuses a splice in the last ``PRE_RELEASE_HOLD_S`` before
#: a release). 'before' is offered only when ``LEAD_S + MIN_WINDOW_S`` plus this
#: margin fit ahead of the release, and the splice lands at most ``LEAD_KNOTS +
#: 1`` knots past now, so it clears the band when ``MIN_WINDOW_KNOTS`` plus
#: this margin in knots is at least the hold's knots + 2. That is exactly 6 = 6
#: at the defaults, pinned by
#: ``test_the_before_stop_splice_clears_the_pre_release_hold_band``. A longer
#: live ``pre_release_hold_s`` can refuse 'before' near that edge, and the stop
#: then takes the 'after' REST.
STOP_BEFORE_MARGIN_S = 0.05
#: The stop REST's second choice: at rest this long AFTER the next pending
#: release. Its splice snaps to that release and is seeded post-release (the
#: cup is empty by then), so the throw runs but nothing after it does.
#: Probe (2026-09-29, scratchpad ``probe_stop_rest.py``, the real planner at
#: the R4 limits, a 250 mm hop and a self-toss chain, 7 releases per cell):
#: this REST installed 7/7 at every instant tried, the 'before' one 7/7 when
#: asked 0.50-0.60 s after the previous release (the hop refuses it at 0.45 s,
#: still mid-traverse, and at 0.65 s, the window too short).
STOP_AFTER_S = 0.6

#: Seconds a tracker landing for an EXTERNALLY announced ball (a CATCH whose
#: ``Skill.landing_prior`` is Ball Butler's own announcement, R4's reload) may
#: disagree with the announced landing instant and still be trusted to aim or
#: re-aim the catch. MEASURED 2026-09-23 (``sim/skills_gate.py --reload``, seed
#: 0, under the live ``AIM_TRACKER`` default): the tracker's immature fit of a
#: long externally-thrown flight aimed the reload CATCH at x = 306 078 mm; the
#: lateral clamp (:meth:`SkillExecutor._clamp_lateral_to_schedule`) bounds the
#: POSITION around the prior but nothing bounded the TIME, and a fit half a
#: second off moves the whole window. Ball Butler's announced instant is the
#: fact the FSM's reload catches were timed on for two months (its `throw_time`
#: + solver ToF); a fit outside this band is the fit's error, not the
#: announcement's, and is ignored -- inside it the fit refines the catch exactly
#: as for a ball this schedule threw. 0.15 s is one hand-stroke duration: a
#: catch moved further than that is a different catch, not a refinement.
EXTERNAL_LANDING_TIME_BAND_S = 0.15

# ── The two-ball association contract, executor half (2026-10-04) ────────────
#
# INVARIANT: a tracker estimate is used for a release -- to aim or re-aim its
# catch, as its release evidence, or as its OUTCOME row -- only if it can be
# THAT release's flight; when the wanted ball has no valid estimate the catch
# keeps the schedule. The identity itself is enforced upstream, exactly, by
# the correlator (`ball_possession.advance_correlation`, keyed on the
# announcement's (source, throw_time)); this is the PHYSICAL cross-check every
# tracker read passes through, one rule (:func:`tracked_landing_refusal`) at
# the executor's two read points (:meth:`SkillExecutor._valid_tracked_landing`
# for a catch, :meth:`SkillExecutor._release_landing` for a release).
#
# Why a time band discriminates: on a two-ball pattern the OTHER ball lands
# one beat (``Schedule.beat_s``, 0.579 s on columns) from this one, and on
# 2026-10-02 every mis-associated estimate sat exactly there (skill 3 aimed at
# A's landing: WINDOW_TOO_SHORT -0.228 s; the throw-2/3 OUTCOME rows "release
# -556..-584 ms"). A genuine estimate sits within the release lag
# (0.019-0.137 s) plus apex scatter (~0.02 s) of its schedule. So the band is
# a third of the beat (:func:`tracked_landing_band_s`): under half the
# spacing to the neighbouring landing, on any pattern -- 0.193 s on columns.
#
# NO 0.2 s ceiling on the identity band, deliberately: on columns a third of
# the beat is already below it, and on a one-ball pattern (no neighbouring
# ball) a ceiling would buy no identity discrimination while refusing a
# GENUINE flight the plant mis-throws by more than 0.2 s -- the measured
# 2026-09-16 plant flew 1.238x the commanded flight (+0.204 s at 0.857 s),
# and that row is exactly what the learner exists to absorb
# (`test_a_plant_throwing_25_percent_fast_is_still_IN_band`). The release
# EVIDENCE keeps its own pre-existing :data:`RELEASE_FIT_TOL_S` cap on top:
# that one is about a bounce's ballistic arc, not identity.

#: The landing-time band (s) for a schedule that carries no beat (a REST-only
#: or hand-built schedule): :data:`RELEASE_FIT_TOL_S`'s measured provenance
#: (fitted touch-down minus scheduled +0.027..+0.066 s over 18 219 samples,
#: 2026-09-29), clearing the worst measured release lag (0.137 s).
TRACKED_LANDING_TIME_BAND_S = RELEASE_FIT_TOL_S

#: Lateral identity bound (mm, per axis) on a tracker landing against the
#: schedule's landing for the same release. Not the platform's lateral
#: AUTHORITY (:attr:`SkillExecutor.lateral_authority_m`, 20 mm live), which
#: decides how far a VALID estimate may move the catch: this decides whether
#: the estimate is about this ball at all. Our own throws' lateral bias is
#: +0.05..+0.12 m (:meth:`SkillExecutor._clamp_lateral_to_schedule`), Ball
#: Butler's feed arrived 80 mm off its aim on 2026-10-02; an estimate further
#: off than 200 mm is a different object (that day's -830.8 mm was the feed
#: ball's post-catch Kalman read for our own launch) and is refused WHOLE --
#: time included -- so the clamp never keeps the time of a position it would
#: have had to throw away.
TRACKED_LANDING_LATERAL_IDENTITY_MM = 200.0


def tracked_landing_band_s(beat_s: Optional[float]) -> float:
    """The landing-time band (s) for a schedule with release spacing
    ``beat_s`` (``Schedule.beat_s``: release to release of ANY ball, 0.579 s
    on columns): ``beat_s / 3``, which keeps the band under half the spacing
    to the neighbouring ball's landing with margin. A missing or
    non-positive beat falls back to :data:`TRACKED_LANDING_TIME_BAND_S`."""
    if beat_s is None or not math.isfinite(float(beat_s)) or float(beat_s) <= 0.0:
        return TRACKED_LANDING_TIME_BAND_S
    return float(beat_s) / 3.0


def tracked_landing_refusal(landing: 'Landing', t_land_scheduled_s: float,
                            scheduled_xy_mm, band_s: float) -> Optional[str]:
    """Why ``landing`` cannot be the flight whose scheduled landing is
    ``(scheduled_xy_mm, t_land_scheduled_s)``, or ``None`` if it can. Pure.

    THE rule of the association contract's executor half (comment block
    above): refused when its landing instant is more than ``band_s`` from the
    scheduled one, or (``scheduled_xy_mm`` given) its xy is more than
    :data:`TRACKED_LANDING_LATERAL_IDENTITY_MM` off on either axis."""
    kind = 'fitted' if bool(getattr(landing, 'from_fit', False)) else 'unfitted'
    dt = float(landing.t_land_abs_s) - float(t_land_scheduled_s)
    if not abs(dt) <= float(band_s):
        return ('%s landing %+.3f s from the scheduled (band +-%.3f s)'
                % (kind, dt, float(band_s)))
    if scheduled_xy_mm is not None:
        off = (np.asarray(landing.pos_mm, dtype=float)[:2]
               - np.asarray(scheduled_xy_mm, dtype=float)[:2])
        axis = int(np.argmax(np.abs(off)))
        if not abs(float(off[axis])) <= TRACKED_LANDING_LATERAL_IDENTITY_MM:
            return ('%s landing %+.1f mm in %s from the scheduled site '
                    '(identity bound %.0f mm)'
                    % (kind, float(off[axis]), 'xy'[axis],
                       TRACKED_LANDING_LATERAL_IDENTITY_MM))
    return None


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
    * ``hand_lane_refused`` -- likewise every tick, never at dispatch alone:
      the firmware's ``sched_refused`` counter has moved since this attempt
      started (``teensy_bridge_node`` publishes it on ``/link_status``) --
      :data:`HAND_LANE_REFUSED`.  Defaults False, so a caller that cannot
      observe the counter keeps the pre-2026-09-18 behaviour exactly.
    """

    mocap_fresh: bool
    hand_fresh: bool
    levelled: bool
    ball_evidence: str
    in_trajectory_mode: bool
    hand_lane_refused: bool = False


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

    **``best_landing`` is FROZEN at this ball's next release** (the crossing,
    2026-09-16 to 2026-09-20): it is the LAST fitted estimate served for this
    flight before the ball leaves the cup again, the fit's own parabola
    whether read before or after the landing -- see
    :meth:`SkillExecutor._consider_landing`. The window itself closes before
    this ball's NEXT scheduled release (:attr:`t_next_release_s`), and a
    finalised row whose observed apex is not within
    ``memory.APEX_RATIO_BAND`` of the commanded one is DROPPED.

    ``release_confirmed`` is the R3 ladder's own bookkeeping (plan § carried
    R3 note / ``ABORTED_NO_RELEASE``): confirmed only by evidence AT OR AFTER
    :attr:`t_release_s` -- a possession EMPTY reading that follows a SEATED
    reading (:attr:`seated_seen`, itself ARMED only by a SEATED sample taken
    at or before ``t_release_s + RELEASE_SEAT_EPS_S`` -- R5 sitting 4,
    2026-10-04 evening, ``report_a1.md`` Q4: a SEATED sample seen at or
    after release) seen at or after release, or a tracker landing whose
    ``t_land_abs_s`` is itself after release -- checked every tick from
    registration until :data:`RELEASE_GRACE_S` past release has elapsed
    with no evidence found -- see :meth:`SkillExecutor.
    _advance_release_evidence`. This is deliberately blind to evidence from
    BEFORE the release instant: a carried throw (a CATCH with a
    ``then_throw``) is registered at the catch's dispatch, up to a beat
    ahead of its own release, while the cup is still carrying the PREVIOUS
    ball -- an EMPTY reading or tracker landing from that earlier flight must
    never confirm THIS row's release. Unconfirmed past the deadline drops
    this row: no evidence the ball left means no learner row, whether or not
    anything else ends the attempt first.

    ``seated_seen``'s own epsilon (:data:`RELEASE_SEAT_EPS_S`) keeps a
    SEATED sample that first appears AFTER that window from arming this
    row at all: that sample can only be another ball's arrival at the
    OTHER site or that ball's own settle chatter, which must never stand in
    for THIS release's own seat (sitting 4 attempts 1/9/10/16/R1: ball A's
    seat-plus-flicker falsely confirmed ball B's empty carried throw,
    report_a1.md Q1/Q4).
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
    #: 1-based release order within the attempt (of
    #: :attr:`SkillExecutor.n_throws`) — the operator's ``throw 3/5``.
    throw_no: int = 0
    #: The height (mm) the release is commanded at — the site's throw
    #: height, which a carried throw's release keeps too
    #: (:meth:`SkillExecutor._release_pos_mm`) — for the ``release`` timing.
    release_z_mm: float = RELEASE_CUP_Z_MM
    #: The CATCH that meets this flight: its latest aim (wall clock), its
    #: schedule index, the re-aims it accepted, and the code of the re-aim it
    #: had refused. For the operator's throw line only (:mod:`report`) —
    #: none of them reaches the learner row.
    t_catch_aim_s: Optional[float] = None
    catch_idx: int = -1
    reaims: int = 0
    reaim_refused: str = ''
    #: The columns Stop's cross-site last throw (owner decision D2,
    #: 2026-09-30, R5 rescope, E1') — read straight off
    #: :attr:`~jugglebot.motion.skills.schedule.Skill.shadow_landing` /
    #: :attr:`~jugglebot.motion.skills.schedule.ThenThrow.shadow_landing` at
    #: registration (:meth:`SkillExecutor._register_outcome`), never
    #: inferred from schedule shape (plan § 0: one data structure, no second
    #: copy of the same fact). This ball lands on the ball ALREADY held at
    #: the other site, so the possession sensor reading SEATED throughout is
    #: evidence about THAT ball, not this one — must not verdict this row
    #: caught, and must not reach the learner's memory. See
    #: :meth:`SkillExecutor._finalise_outcome`.
    shadow_landing: bool = False


@dataclasses.dataclass(frozen=True)
class SeatWindow:
    """One ``seat_window(t0, t1)`` answer (R5 sitting 4, ``report_a1.md``
    Q3) -- a SAMPLE-COMPLETE count over ``[t0, t1]``, not a tick read:
    ``n_valid`` samples had a valid possession reading, ``n_seated`` of
    those were SEATED on the RAW bit, and ``max_gap_s`` is the largest gap
    between consecutive valid samples' arrival times. The caller
    (:meth:`SkillExecutor._register_catch` / :meth:`SkillExecutor.
    _advance_catch_evidence`) judges liveness and fires :data:`MISSED_CATCH`
    from these three numbers alone; this type carries no policy.

    ``n_seated`` counts the RAW bit only, not the DEBOUNCED ``held`` twin
    (B1c, sitting 4, fixing B1's own fixture: against the REAL raw-bit
    sequences in ``tests/motion/fixtures/missed_catch_20261004.csv`` the
    raw-or-held rule fired 0/14 on genuine misses, because ``held``'s
    debounce lingers after the PREVIOUS ball leaves -- it falls ~0.14 s
    after that release, inside THIS catch's own window, which opens only
    :data:`MISSED_CATCH_OPEN_AFTER_RELEASE_S` (0.10 s) after it -- and
    vetoed every one of them. Counting raw only loses no sensitivity to a
    real catch: ``held`` is DERIVED from ``raw`` (it only ever reads SEATED
    after raw already has), so a genuine seat always shows up on the raw
    bit first -- dropping ``held`` from the count drops only its lingering
    false-positive tail, never a true positive."""

    n_valid: int
    n_seated: int
    max_gap_s: float


@dataclasses.dataclass
class _PendingCatch:
    """One accepted CATCH, still waiting on its own :data:`MISSED_CATCH`
    verdict (R5 sitting 4, ``report_a1.md`` Q3) -- registered at the CATCH's
    dispatch (:meth:`SkillExecutor._register_catch`), resolved by
    :meth:`SkillExecutor._advance_catch_evidence`.

    ``t_open_s``/``t_close_s`` are computed ONCE, at registration, from the
    schedule as it stood then -- a pure function of
    ``(ball_id, t_land_scheduled_s)`` and the schedule's other releases/
    landings. A later schedule mutation (:meth:`SkillExecutor.
    _install_survivor_tail`) never reaches back to revise an already
    registered window."""

    ball_id: int
    catch_idx: int
    t_land_scheduled_s: float
    t_open_s: float
    t_close_s: float


def _latest_release_before(schedule: Schedule, t_before: float
                           ) -> Optional[float]:
    """The latest scheduled release (a THROW's own, or a CATCH's carried
    ``then_throw``) of ANY ball strictly before ``t_before`` -- pure, reads
    only ``schedule.skills`` (:meth:`SkillExecutor._register_catch`'s
    ``t_open`` anchor, ``report_a1.md`` Q3)."""
    releases = []
    for sk in schedule.skills:
        if sk.kind == THROW:
            releases.append(float(sk.t_abs_s))
        elif sk.kind == CATCH and sk.then_throw is not None:
            releases.append(float(sk.then_throw.t_release_abs_s))
    before = [r for r in releases if r < float(t_before)]
    return max(before) if before else None


def _next_landing_other_ball(schedule: Schedule, ball_id: int,
                             t_after: float) -> Optional[float]:
    """The earliest scheduled CATCH landing, for a ball OTHER than
    ``ball_id``, strictly after ``t_after`` -- pure (:meth:`SkillExecutor.
    _register_catch`'s ``t_close`` guard, ``report_a1.md`` Q3)."""
    landings = [float(sk.t_abs_s) for sk in schedule.skills
               if sk.kind == CATCH and sk.ball_id != ball_id
               and float(sk.t_abs_s) > float(t_after)]
    return min(landings) if landings else None


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
    """``(release_site, t_release_abs_s, y_d, target, release_idx)`` of the last
    release of ``ball_id`` strictly before ``schedule.skills[idx]`` -- a THROW
    or a CATCH's ``then_throw`` -- or ``None`` when no such release is IN THIS
    SCHEDULE (columns' very first catch, of a ball thrown before ``t0`` --
    plan owner decision 2026-09-13, "at release, then refine": that catch
    falls back to waiting for the tracker, unchanged).

    ``release_idx`` is the releasing skill's OWN index in
    ``schedule.skills``. A THROW releases at its site, but a CATCH's carried
    throw releases from where the ball was CAUGHT (2026-09-28,
    :meth:`SkillExecutor._catch_terminal`), so the release POSITION is no
    longer a property of the site alone: the index is the key the chosen
    release is recorded under (:meth:`SkillExecutor._release_pos_mm`), and
    every caller that needs the position asks there rather than assuming
    ``release_site.throw_site_mm()``.
    """
    for j in range(idx - 1, -1, -1):
        sk = schedule.skills[j]
        if sk.ball_id != ball_id:
            continue
        if sk.kind == THROW:
            return sk.site, float(sk.t_abs_s), sk.y_d, sk.target, j
        if sk.kind == CATCH and sk.then_throw is not None:
            tt = sk.then_throw
            return sk.site, float(tt.t_release_abs_s), tt.y_d, tt.target, j
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
                    t_install_s: Union[None, float, Callable[[], float]] = None,
                    reserve_fresh_lead: bool = False
                    ) -> Tuple[Optional[PlanRecord], InstallResult,
                               Optional[Segment]]:
    """Plan one skill and put it on the machine — the ONE install path.

    ``terminal`` carries its event time as an ABSOLUTE instant on ``t_now_s``'s
    clock (see :func:`_event_abs_s`); everything below converts it once.

    **Two cases, and the machine's physical state is what chooses.**

    *Fresh origin* — there is no record, or the record's plan has ENDED by the
    time this install could reach the wire (``t_now + lead >= record.end_s``).
    The machine is then holding a terminal REST: a stopped platform over a
    seated ball.  The segment is planned from that rest with ``t0 = t_now_s``
    (the solve time is not skipped over), which is sound for exactly one
    reason — the seed is at rest, so the plan's knot 0 IS where the machine
    still is when the solve finishes, however long it took.  (A moving seed
    has no such property, which is why the splice branch below measures its
    own lateness.)

    That soundness argument covers the SEED, not the ORIGIN TIMESTAMP the
    plan carries onto the wire.  A fresh origin reserves no lead the way a
    splice's ``k_s`` does, so once the solve (plus whatever transit follows)
    outruns :data:`~jugglebot.motion.skills.schedule.WIRE_READ_KNOTS` knots of
    margin, the block reaches the firmware's scheduled lane with knot 0
    already behind it — 2026-09-28 R4 sitting-1 fact 5: a reload's opening
    REST (6.6 s, hand 8.87 -> 0.31 rev) solved in 381 ms, the scheduled block
    arrived ~0.4 s into its own profile, the firmware's resume check
    (``SCHED_RESUME_TOL_POS_HAND_REV`` 0.05 rev against the HELD hand)
    refused every frame, and the deviation guard E-STOPPED on a hand that
    never received a command.  So AFTER the solve, this branch re-reads the
    clock exactly as the splice branch does (:data:`ORIGIN_TOO_LATE` below)
    and, if the origin has gone stale:

    * kind ``REST`` — REBASED, not re-solved: ``t0`` moves to
      ``t_inst + WIRE_READ_KNOTS·dt``.  A REST from rest is the same motion
      later — it carries no absolute event, so sliding when it starts costs
      nothing and the plan itself (built from the same at-rest seed) is
      unchanged.
    * kind ``THROW`` / ``CATCH`` — REFUSED :data:`ORIGIN_TOO_LATE`.  These
      carry an absolute event (the release or the landing is a real instant
      the rest of the schedule, the tracker and the ball's flight all agree
      on), so the origin cannot be silently pushed later the way a REST's can.

    **``reserve_fresh_lead=True`` — the live rule for an event-bearing fresh
    origin (2026-10-02, R5 sitting 2: 3 of 7 Ball-Butler-fed columns attempts
    refused ``ORIGIN_TOO_LATE`` at ball A's first throw on solves of 76-84
    ms).**  The default above pins ``t0 = t_now_s`` BEFORE the solve, so the
    dispatch lead the schedule reserved (``Skill.dispatch_s = t_abs - window
    - lead``) silently becomes extra WINDOW (the live THROW planned 0.600 s
    for a 0.4 s ``launch_s``) and the solve is left only the 75 ms wire
    margin — and every solve under that margin still SKIPS the plan's first
    ``solve + <=1`` knot(s), because nothing reaches the wire before the
    install: a commanded step on the hand (measured offline at the R5
    operating point, ``scratchpad/probe_fresh_origin.py``: 0.015-0.07 rev and
    0.7-1.4 rev/s for a 50-100 ms skip of a from-rest THROW, over the
    firmware's 0.005 rev / 0.5 rev/s promotion tolerance).  With the flag set
    a THROW/CATCH fresh origin is planned EXACTLY as a splice is:

    * ``t0 = t_now_s + lead_s`` — the origin sits where a splice's ``k_s``
      would (the first instant ``lead_s`` ahead of dispatch), so the planned
      window is the schedule's own ``window_s`` less the dispatch lateness,
      i.e. the window ``plan_columns_first_cycle`` and the admissible box
      already certify (``t_abs - window_s``), not ``window_s + lead``;
    * refused :data:`ORIGIN_TOO_LATE` by the SPLICE's own arithmetic with
      ``k_s = 0`` on that origin — ``0 <= floor((t_inst - t0)/dt) +
      WIRE_READ_KNOTS`` — so the solve budget is ``lead_s - WIRE_READ_KNOTS·dt``
      = :data:`~jugglebot.motion.skills.schedule.SOLVE_BUDGET_KNOTS` knots
      (150 ms at ``LEAD_S``), the same budget every splice has;
    * between the install and ``t0`` the machine HOLDS: ``CyclePlan.state_at``
      / ``hand_at`` / ``hand_accel_at`` return knot 0's boundary conditions
      for ``t <= 0`` (``cycle_plan.py`` ``_locate``), and knot 0 of a plan
      seeded at rest IS the held pose with zero velocity and zero cubic
      acceleration, so the emitter streams the rest it was already streaming
      and hands over to the plan at ``t0`` on the same cubic — no knot is
      ever skipped, nothing moves before ``t0``, and the feedforward path
      (leg torque FF, hand ``J·α``) sees ``α = 0`` until the plan's own first
      span.

    A REST ignores the flag (it keeps the rebase rule above: it carries no
    event, so sliding its origin costs nothing).  The flag is opt-in because
    offline callers (sweeps, the bench rehearsal, most tests) pass the WINDOW
    START as ``t_now_s`` and mean "plan from here"; the live node and the sim
    gate pass the DISPATCH instant and set it.

    *Splice* — the previous segment is still streaming.  The new window opens at
    ``k_s = splice_knot(meta, τ, lead_s)``, the first knot at least ``lead_s``
    ahead of now, seeded by ``state_at_knot`` and joined by ``splice_at``.  The
    head is carried bit for bit, so whatever the emitter has already sent stays
    sent.

    **Three refusals this function owns, one physical fact each.**

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

    * :data:`ORIGIN_TOO_LATE` — the FRESH-origin twin of the refusal above:
      the solve outran :data:`WIRE_READ_KNOTS` knots of margin on an
      event-bearing (``THROW``/``CATCH``) segment planned from rest, so knot 0
      would already be old at the wire and there is no later knot to move the
      event to without moving the event itself.  A ``REST`` in the same
      situation is REBASED instead (see *Fresh origin* above) rather than
      refused, because it carries no event to protect.

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
            # `reserve_fresh_lead` (see the docstring): an event-bearing
            # fresh origin sits `lead_s` after dispatch, exactly where a
            # splice's `k_s` would, so the solve has the schedule's own budget
            # and knot 0 is still ahead of the wire when it lands. A REST
            # keeps `t0 = t_now_s` and the rebase rule below.
            reserved = bool(reserve_fresh_lead) and kind != REST
            t0 = t_now_s + (lead_s if reserved else 0.0)
            k_s = 0
            t_origin = t0
            if reserved:
                window_s = _event_abs_s(kind, terminal) - t_origin
                if window_s < MIN_WINDOW_S - 1e-12:
                    return record, InstallResult(
                        False, WINDOW_TOO_SHORT,
                        'a %.3f s window from the reserved fresh origin '
                        '(dispatch + %.3f s lead) to the %s is under the '
                        '%d-knot floor (%.3f s) — the gate has no stencil to '
                        'measure a jerk in'
                        % (window_s, lead_s, kind, MIN_WINDOW_KNOTS,
                           MIN_WINDOW_S), 0.0), None
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
            # The seed is sound regardless of how long the solve took (see the
            # docstring's *Fresh origin* section) — but the ORIGIN TIMESTAMP
            # this plan carries onto the wire is not, because `t0` was fixed
            # to `t_now_s` BEFORE the solve and a fresh origin reserves no
            # lead the way a splice's `k_s` does. Re-read the clock the SAME
            # way the splice branch below does (a callable evaluated once the
            # segment exists, so a float still lets tests inject the
            # lateness) and check whether the solve (plus whatever transit
            # follows) has already spent the `WIRE_READ_KNOTS` margin that
            # protects a splice.
            t0_before_solve = t0
            dt = float(seg.plan.dt)
            t_inst = (t_now_s if t_install_s is None
                      else float(t_install_s()) if callable(t_install_s)
                      else float(t_install_s))
            wire_margin_s = WIRE_READ_KNOTS * dt
            if reserved:
                # The SPLICE's own test with `k_s = 0` on the reserved origin
                # (one rule, two branches): knot 0 must still be ahead of the
                # last knot the wire has read by the install instant. Inside
                # that, the emitter streams knot 0 (= the held rest) until
                # `t0` and no knot of this plan is ever skipped.
                solve_s = t_inst - t_now_s
                budget_s = lead_s - wire_margin_s
                k_wire = int(math.floor((t_inst - t0) / dt)) + WIRE_READ_KNOTS
                if 0 <= k_wire:
                    return record, InstallResult(
                        False, ORIGIN_TOO_LATE,
                        'the solve took %.3f s, past the %.3f s solve budget a '
                        'fresh %s reserves (origin %.3f s after dispatch, less '
                        'the %d-knot wire read): knot 0 would already be '
                        'behind the wire, so the plan\'s first knots would be '
                        'skipped — a commanded step off the held lane'
                        % (solve_s, budget_s, kind, lead_s, WIRE_READ_KNOTS),
                        float(seg.meta.plan_wall_s)), None
                new_record = PlanRecord(plan=seg.plan, meta=seg.meta, t0_s=t0)
                return new_record, InstallResult(
                    True, 'OK',
                    'fresh origin: %s over %.3f s from rest, origin +%.3f s '
                    'after dispatch (solve %.3f s of a %.3f s budget)'
                    % (kind, seg.plan.total_duration, t0 - t_now_s, solve_s,
                       budget_s),
                    float(seg.meta.plan_wall_s), splice_k=0, t0_s=t0,
                    event_t_s=float(seg.event_t_s or 0.0),
                    seeded_post_release=False), seg
            solve_s = t_inst - t0_before_solve
            if solve_s - wire_margin_s > 0.0:
                # `t_inst + wire_margin_s` is how old the origin will be once
                # the wire actually starts reading this block (the same
                # worst-case margin `k_wire` budgets for a splice) — this is
                # the number fact 5 measured as "the origin ~0.4 s in the
                # past".
                staleness_s = solve_s + wire_margin_s
                if kind == REST:
                    # REBASE, not re-solve: a REST from rest is the same
                    # motion later — it carries no absolute event, so sliding
                    # its origin forward by exactly the measured staleness
                    # costs nothing and the plan already built (from the same
                    # at-rest seed) is unchanged.
                    t0 = t_inst + wire_margin_s
                    new_record = PlanRecord(plan=seg.plan, meta=seg.meta,
                                            t0_s=t0)
                    return new_record, InstallResult(
                        True, 'OK',
                        'fresh origin REBASED +%.3f s: the solve outran the '
                        'origin (%s over %.3f s from rest; solve %.3f s vs '
                        'a %.3f s wire-read margin)'
                        % (t0 - t0_before_solve, kind, seg.plan.total_duration,
                           solve_s, wire_margin_s),
                        float(seg.meta.plan_wall_s), splice_k=0, t0_s=t0,
                        event_t_s=float(seg.event_t_s or 0.0),
                        seeded_post_release=False), seg
                return record, InstallResult(
                    False, ORIGIN_TOO_LATE,
                    'the solve took %.3f s, the first knot would be %.3f s '
                    'old at the wire, and the firmware refuses a scheduled '
                    'block discontinuous with the held lane'
                    % (solve_s, staleness_s),
                    float(seg.meta.plan_wall_s)), None
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
                 observations: Optional[Callable[[float], Observations]] = None,
                 dispatch_lookahead_s: float = 0.0,
                 seat_window: Optional[Callable[[float, float],
                                                Optional[SeatWindow]]] = None):
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
        #: How far (m, per axis) the learner's THROW command, and a tracker
        #: CATCH aim (:meth:`_clamp_lateral_to_schedule`), may move the
        #: commanded/aimed landing AWAY from the schedule's desired offset
        #: ``y_d``. ``None`` = the swept box alone bounds the learner, and the
        #: tracker aim is unclamped. ``0.0`` (the live default, owner
        #: 2026-09-16) pins the lateral command to ``y_d`` and lets the learner
        #: correct FLIGHT only: a 4 mm lateral aim command made the banking
        #: step saturate to its 12 deg clamp during the pre-catch dive (leg
        #: jerk 137-161 k against 150 k, the 0.9 m 'wobble') -- sub-cm lateral
        #: authority is the planner's open defect, not the learner's to spend.
        #: The tracker aim hit the SAME limit from the catch side 2026-09-18
        #: (see :meth:`_clamp_lateral_to_schedule`) -- one knob, two paths onto
        #: the platform, because the authority being spent is the same
        #: banking budget either way.
        self.lateral_authority_m = (None if lateral_authority_m is None
                                    else float(lateral_authority_m))
        self.observer = observer
        self.on_experience = on_experience
        self.observations = observations
        if not (math.isfinite(float(dispatch_lookahead_s))
                and float(dispatch_lookahead_s) >= 0.0):
            raise ValueError('dispatch_lookahead_s must be finite and >= 0, '
                             'got %r' % (dispatch_lookahead_s,))
        #: Seconds AHEAD of ``t_abs_s`` a skill may dispatch (see :meth:`tick`):
        #: the caller's own dispatch quantisation, so a reserved fresh origin
        #: never plans under the schedule's ``window_s``. 0.0 = R2 behaviour.
        self.dispatch_lookahead_s = float(dispatch_lookahead_s)
        #: ``(t0, t1) -> Optional[SeatWindow]`` -- the possession evidence
        #: over a wall-clock span, for the :data:`MISSED_CATCH` rule
        #: (:meth:`_register_catch` / :meth:`_advance_catch_evidence`). No
        #: ``seat_window`` ⇒ the rule is off and a CATCH behaves exactly as
        #: before this rule existed (R2/R3 callers unchanged).
        self.seat_window = seat_window

        self.dispatched = set()          #: indices of ``schedule.skills``
        self.results = []                #: (index, Skill, InstallResult)
        self.attempt_ended = False
        self.end_code = ''
        #: Why ``end_code`` was set, in the END line's words (without its
        #: clock and code), and the kind of skill it ended at (``''`` when it
        #: did not end at one) — for the operator's end line (:mod:`report`).
        #: A caller that ends the attempt itself sets them too.
        self.end_message = ''
        self.end_kind = ''
        #: Releases this schedule plans (THROWs, and CATCHes carrying a
        #: ``then_throw``) and how many have registered an outcome so far.
        self.n_throws = sum(
            1 for sk in schedule.skills
            if sk.kind == THROW
            or (sk.kind == CATCH and sk.then_throw is not None))
        self._n_registered = 0
        #: One :class:`~jugglebot.motion.skills.report.ThrowReport` per
        #: finalised release, in finalisation order — the caller reads it.
        self.reports: List[ThrowReport] = []
        #: The CATCH currently committed, as ``(index, Landing, t_install_s)``.
        self._live_catch = None
        #: Per-skill-index command cache: ``idx -> (x, u_dy, u_apex)``. Keyed
        #: on the RELEASING skill's own index (a THROW, or a CATCH carrying a
        #: ``then_throw``) so a catch re-send's second, third, ... call reuses
        #: the first dispatch's command rather than recomputing it.
        self._u_cache = {}
        #: Released balls awaiting outcome finalisation (:class:`_PendingOutcome`).
        self._pending_outcomes: List[_PendingOutcome] = []
        #: Accepted CATCHes awaiting their own :data:`MISSED_CATCH` verdict
        #: (:class:`_PendingCatch`) -- empty, and never appended to, when
        #: :attr:`seat_window` is ``None``.
        self._pending_catches: List[_PendingCatch] = []
        #: CATCH indices whose ONE hand-measured re-aim is already settled —
        #: applied, refused, or given up on (:data:`AIM_SCHEDULE_HAND`).
        self._hand_corrected = set()
        #: ``idx -> re-aims already installed`` (:attr:`resend_max_per_catch`).
        self._resend_counts = {}
        #: ``(idx, reason)`` pairs already reported by :meth:`_resend_live_catch`
        #: OR the lateral clamp (:meth:`_clamp_lateral_to_schedule`, reason
        #: ``'AIM-LATERAL-CLAMPED'``) -- one shared once-per-(catch, reason) set.
        self._resend_notes = set()
        #: Keys (``('catch', idx)`` / ``('throw', throw_no)``) whose
        #: TRACKER-IDENTITY-REFUSED line is already out -- once per skill, so
        #: a ~180 Hz stream of the wrong ball's estimate is one line, not 180.
        self._identity_refused = set()
        #: ``catch idx -> the cup position that catch's CARRIED release was
        #: planned from`` (:meth:`_catch_terminal`, latest call wins -- a
        #: re-send re-derives it from the refined landing). Read by
        #: :meth:`_release_pos_mm` so the schedule-derived priors for the NEXT
        #: catch fly from the SAME release point the plan uses. A plain THROW
        #: is never recorded here: it releases at its site.
        self._carried_release_mm = {}
        #: Lines queued by :meth:`_clamp_lateral_to_schedule` (via
        #: :meth:`_catch_terminal`), drained by whichever caller fired it
        #: (dispatch, :meth:`_resend_live_catch`,
        #: :meth:`_resend_hand_corrected_catch`) into its own return.
        self._pending_notes: List[str] = []

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
        (plan § 2.7, "outcomes keep finalising after the attempt ends").

        Also waits on any open :class:`_PendingCatch` (R5 sitting 4,
        ``report_a1.md`` Q3): every skill can already be dispatched the
        instant the schedule's LAST catch installs, a beat before that
        catch's own :data:`MISSED_CATCH` evidence window has even closed --
        without this a caller that stops at ``done`` would never see the
        verdict on a schedule's final catch."""
        finished = (self.attempt_ended
                   or len(self.dispatched) >= len(self.schedule.skills))
        return (finished and not self._pending_outcomes
               and not self._pending_catches)

    # ── the R3 command: learner + admissible box, computed once per throw ──

    def _command_u(self, idx: int, site, target, y_d,
                   release_offset_m=None,
                   shadow_landing: bool = False) -> Tuple[np.ndarray, float]:
        """The commanded ``u = (landing_xy_m, apex_m)`` for the ball this
        skill releases — computed ONCE (at ``idx``'s first dispatch) and
        cached, so a CATCH re-send's later calls return the SAME command
        (plan § 0: "the command never changes late in a transit").

        ``x = (site xy in m, release offset xy in m)`` — ``memory.Experience``
        has always defined the second half as "the seat-offset xy of the ball
        just caught", and ``release_offset_m`` is now that quantity, measured:
        a CATCH's carried throw releases from where the ball was CAUGHT
        (:meth:`_catch_terminal`, 2026-09-28), so ``release_xy - site_xy`` is
        exactly how far off its nominal site this release happens. A plain
        THROW from rest passes ``None`` and keeps ``(0, 0)`` — it releases at
        its site, so there is nothing to carry.

        WHY it must be in ``x`` and not simply ignored: the release position
        is PLANT STATE that a static ``u -> y`` map cannot represent. Measured
        2026-09-28 (``sim/skills_gate.py --learn --pattern self_toss``, seeds
        0-4): with the offset release and ``x[2:4]`` pinned at zero, ~23 % of
        every offset came back as an opposite-sign landing error (the offset
        is flown back as lateral velocity, and a launch 11 % fast over a
        flight 11 % long delivers ~1.23x of it), and the learner chased its
        own tail into a limit cycle on 2 of 5 seeds — the landing walking
        50 mm off the site and back over ~12 throws. The learner's local
        affine fit already has the term for it: its regressor is
        ``Z = [delta_x; delta_u; 1]`` (``learner`` step 3), so a ``dy/dx``
        slope is fitted from the same neighbours, and the state bandwidth
        ``h_x = 0.01 m`` is shared with the site half — no new knob, no
        ``LearnerConfig`` change, no memory-schema change (``x`` has been
        4-wide since R3).

        No ``learner`` ⇒ ``u = y_d`` exactly (R2 behaviour). Raises
        :class:`_NoAdmissibleCommand` when the learner's fit is non-finite,
        or no swept box covers ``(site.name, target.name)`` at the DESIRED
        apex — the caller converts that to a :data:`NO_ADMISSIBLE_COMMAND`
        refusal before any solve is attempted.

        ``shadow_landing`` (owner decision D2, 2026-09-30, R5 rescope, E1'):
        the columns Stop's cross-site last throw bypasses the learner AND the
        box entirely, ``u = y_d`` exactly, no clip — the ball lands on the
        ball already held, so precision buys nothing, and the admissible box
        is swept per pattern for SAME-site throws only (this one is not; a
        lookup here would either miss and refuse, or hit a box that was never
        measured for this segment shape).
        """
        if idx in self._u_cache:
            _x, u_dy, u_apex = self._u_cache[idx]
            return u_dy, u_apex
        dy, apex = y_d
        dy = np.asarray(dy, dtype=float).reshape(2)
        apex = float(apex)
        off = (np.zeros(2) if release_offset_m is None
              else np.asarray(release_offset_m, dtype=float).reshape(2))
        x = np.array([float(site.cup_mm[0]) / 1000.0,
                     float(site.cup_mm[1]) / 1000.0,
                     float(off[0]), float(off[1])])
        if shadow_landing:
            u_dy, u_apex = dy, apex
            self._u_cache[idx] = (x, u_dy, u_apex)
            return u_dy, u_apex
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
            # R4 (2026-09-23): a box now carries its own pattern + site xy
            # (motion/skills/admissible.py's module docstring, "Why" #1/#2),
            # so `select` is judged on all four -- the schedule states which
            # pattern it was compiled for (`Schedule.pattern`), and the LIVE
            # sites (not merely their names) close the defect a box swept at
            # one separation being silently applied at another.
            box = adm.select(self.boxes, self.schedule.pattern,
                             (site.name, target.name), apex,
                             release_site_xy_mm=site.cup_mm[:2],
                             target_site_xy_mm=target.cup_mm[:2])
            if self.learner is not None and box is None:
                reason = adm.describe_miss(
                    self.boxes, self.schedule.pattern,
                    (site.name, target.name), apex,
                    release_site_xy_mm=site.cup_mm[:2],
                    target_site_xy_mm=target.cup_mm[:2])
                raise _NoAdmissibleCommand(
                    'no admissible box covers pattern %r site pair %r: %s — '
                    'a learner command may not reach the platform unclipped'
                    % (self.schedule.pattern, (site.name, target.name),
                       reason))
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
                                       skill.y_d,
                                       shadow_landing=skill.shadow_landing)
        return ThrowTerminal(
            site_mm=skill.site.throw_site_mm(),
            target_mm=self._commanded_target_mm(skill.target, (u_dy, u_apex)),
            flight_s=sch.flight_s(u_apex), t_release_s=float(skill.t_abs_s))

    def _clamp_lateral_to_schedule(self, idx: int, skill: Skill,
                                   landing: Landing) -> Landing:
        """Clamp ``landing``'s lateral (x, y) to within
        :attr:`lateral_authority_m` of the schedule's commanded landing for
        this catch (:meth:`_predicted_landing`'s ``pos_mm[:2]`` — the prior a
        catch is dispatched on, reused rather than re-derived). ``z``,
        ``vel_mm_s`` and ``t_land_abs_s`` pass through untouched: a tracker
        fit may move the catch in TIME and in arrival VELOCITY, never
        laterally beyond the authority.

        **The time this keeps is always a vetted one** (2026-10-04). Every
        tracker landing reaching here came through :meth:`_tracked_landing`,
        i.e. passed :meth:`_valid_tracked_landing`: inside the landing-time
        band of THIS catch's schedule and inside
        :data:`TRACKED_LANDING_LATERAL_IDENTITY_MM` of its site. An estimate
        too far off to be this ball is refused WHOLE there and the catch
        keeps the schedule; the clamp only ever clips a valid estimate's xy
        to the platform's authority. Before this, a +101.9 mm estimate of
        the OTHER columns ball was clipped here and its time (one beat
        early) kept, and the install refused WINDOW_TOO_SHORT (2026-10-02).

        Why: the plant's lateral landing bias is real and constant-velocity
        (``y`` +0.05..+0.12 m, growing with apex), so a converged fit
        routinely asks for an 80-95 mm lateral re-aim. Two such re-aims were
        ACCEPTED and the ball was dropped both times ("RESEND skill 2: the
        fit moved the landing 84.4 mm / 95.1 mm in +y", 2026-09-18 sitting,
        ``temp/logs/launch_r2gate_20260918_1325.log``); the 52 further
        solves that instead REFUSED the move (``REJECTED_CYCLE_INFEASIBLE``,
        LIMIT_VEL/ACC/JERK) are the banking-saturation class of
        ``logbook/2026-09-16-banking-saturates-on-small-lateral-offsets.md``
        — an 80 mm lateral catch move inside the pre-catch dive saturates the
        banking step. Meanwhile every SCHEDULE-aimed catch at the same site
        caught balls that landed 50-75 mm off: the cup tolerates the miss,
        the platform does not tolerate the move. The clamp lifts with
        :attr:`lateral_authority_m` — the same knob :meth:`_command_u` already
        clamps the learner's lateral command to. Owner 2026-09-16 pinned the
        node's launch default at 0.0 mm — zero authority clamps every
        lateral landing exactly onto the schedule site, i.e. fully pinned —
        until the planner carried banking that could afford the move; owner
        2026-09-21 lifted the pin to 40.0 mm once the cup-contact contract's
        amplitude-aware banking landed
        (``plans/archived/cup-contact-contract.md`` § 6). This method's clamp
        logic and its ``0.0``/``None`` special cases are unchanged; only the
        node's default input to it moved.

        ``None`` (:attr:`lateral_authority_m` unset) or no schedule prior for
        this catch (:meth:`_predicted_landing` returns ``None`` — a ball
        thrown before ``t0``, the one catch the schedule cannot aim at all)
        both leave ``landing`` exactly as given: there is nothing to clamp
        toward in the second case, exactly as before this change.
        """
        if self.lateral_authority_m is None:
            return landing
        predicted = self._predicted_landing(idx, skill)
        if predicted is None:
            return landing
        a_mm = abs(self.lateral_authority_m) * 1000.0
        sched_xy = np.asarray(predicted.pos_mm, dtype=float)[:2]
        landing_xy = np.asarray(landing.pos_mm, dtype=float)[:2]
        offset = landing_xy - sched_xy
        clamped_xy = sched_xy + np.clip(offset, -a_mm, a_mm)
        bite_mm = np.abs(landing_xy - clamped_xy)
        key = (idx, 'AIM-LATERAL-CLAMPED')
        if float(np.max(bite_mm)) > 1.0 and key not in self._resend_notes:
            self._resend_notes.add(key)
            axis = int(np.argmax(np.abs(offset)))
            self._pending_notes.append(
                'AIM-LATERAL-CLAMPED skill %d: tracker landing %+.1f mm in '
                '%s, catch keeps the schedule\'s site (authority %.0f mm)'
                % (idx, offset[axis], 'xy'[axis], a_mm))
        pos_mm = np.array([clamped_xy[0], clamped_xy[1],
                          float(np.asarray(landing.pos_mm, dtype=float)[2])])
        return Landing(pos_mm=pos_mm, vel_mm_s=landing.vel_mm_s,
                      t_land_abs_s=landing.t_land_abs_s,
                      from_fit=landing.from_fit)

    def _catch_terminal(self, idx: int, skill: Skill,
                        landing: Landing) -> CatchTerminal:
        """A CATCH's terminal, plus the throw it carries when the schedule
        folded one onto it (``schedule.ThenThrow``).

        The carried release's INSTANT and TARGET are the SCHEDULE's, not the
        tracker's: a re-send re-solves the whole remaining window against a
        refined landing with both unchanged, because the pattern's beat is what
        the other hand is already flying against. The carried throw's command
        is :meth:`_command_u`'s, cached under THIS catch's ``idx`` — a re-send
        calls this again and gets the SAME command back.

        **The release POSITION is where the ball was CAUGHT, not the site**
        (owner 2026-09-28). The physical rule: the platform does not translate
        while it holds a ball between a catch and the release that follows it.
        Measured — at the 2026-09-27 22:37 R4 sitting every throw that
        followed a laterally re-aimed catch missed by +32..+114 mm in y and
        the chains dropped. With the release pinned to the site, the plan
        dragged the platform the 20 mm from the catch xy back to the site
        DURING the 0.35 s dwell (peak ~100 mm/s lateral, ~1.2 deg bank) while
        the ball rode the hand down, so the ball left a platform that was
        level and stationary at release yet carried +85..+110 mm/s of
        unplanned lateral velocity; the throws from a cup that had NOT moved
        (the first of each chain) landed within 20 mm of the aim. So
        ``ThrowAfterCatch.site_mm`` is the CLAMPED landing's xy at this site's
        release z. ``target_mm``, ``flight_s`` and ``t_release_s`` are
        unchanged: the pattern still names the SITE as the target, and the
        ballistics from the offset release absorb the <= 20 mm shift as a
        <= 25 mm/s lateral launch component (20 mm over a 0.86 s flight).
        Re-centring on the nominal site belongs in the flight — the SETTLE
        tail AFTER the ball has left — which is why ``rest_site_mm`` stays
        ``skill.site.rest_site_mm()``. A re-send re-derives the release xy
        from the refined landing (the same rule as ``rest_mm`` for the
        held-axis catch below), and the position chosen is recorded under this
        catch's ``idx`` (:attr:`_carried_release_mm`) so the priors built for
        the NEXT catch fly from the release the plan actually uses
        (:meth:`_release_pos_mm`).

        **Measured cost, 2026-09-28** (``sim/skills_gate.py --learn --pattern
        self_toss``, seeds 0-4): the release position is now a function of the
        PREVIOUS landing, i.e. plant state the learner's static ``u -> y`` map
        cannot see, and on 2 of the 5 seeds (0 and 3) the learner+plant pair
        limit-cycled -- the landing walking up to 50 mm off the site and back
        over ~12 throws, failing the monotone-decay rule, with band entry
        unchanged (3 throws) and 25/25 caught, 0 drops. The coupling gain is
        the plant's launch-speed bias: an offset release must fly the offset
        back as lateral velocity, and a launch 11 % fast over a flight 11 %
        long delivers ~1.23x of it. The fix is to stop hiding the offset from
        the learner rather than to stop offsetting the release: it goes into
        the state the learner is queried with and the state its row carries
        (``x[2:4]``, :meth:`_command_u` / :meth:`_register_outcome`), which is
        the slot ``memory.Experience`` always reserved for it.

        **Admissible box:** the boxes in ``config/generated/admissible_box.
        yaml`` were swept for releases AT the site, and this unit deliberately
        does not touch them. The release offset is bounded by the same lateral
        authority the landing is (:attr:`lateral_authority_m`, 20 mm at the
        R4 launch default), and the online ``validate_cycle`` gate remains the
        authority: a ``REJECTED_CYCLE_INFEASIBLE`` on a re-aimed carried throw
        is the expected surface, not a box to widen.

        ``landing`` itself is clamped laterally first
        (:meth:`_clamp_lateral_to_schedule`) — the ONE place a landing becomes
        a catch terminal, so dispatch, the hand-corrected re-send and the
        live tracker re-send all pass through the same lateral authority.

        ``rest_site_mm`` and ``hold_tilt`` pass ``skill.rest_site_mm`` /
        ``skill.hold_tilt`` straight through when the schedule set them (R4
        reload's held-axis CATCH — ``schedule.compile_reload``,
        ``segments.hold_axis_site``); ``skill.site.rest_site_mm()`` is the
        default every pre-R4 CATCH still gets. ``receive_tilt`` (R5, the
        BB-fed columns feed catch) passes ``skill.receive_tilt`` straight
        through the same way — ``None`` for every catch that does not set it.
        """
        landing = self._clamp_lateral_to_schedule(idx, skill, landing)
        then_throw = None
        tt = skill.then_throw
        if tt is not None:
            # The release rides the CAUGHT xy (docstring above): only z is the
            # site's, because the release height is the stroke's geometry, not
            # the ball's arrival point. Recorded BEFORE the command is asked
            # for, because how far this release sits off its site is half of
            # the learner's state vector (:meth:`_command_u`'s ``x[2:4]``).
            release_mm = np.array([float(landing.pos_mm[0]),
                                   float(landing.pos_mm[1]),
                                   float(skill.site.throw_site_mm()[2])])
            self._carried_release_mm[idx] = release_mm
            u_dy, u_apex = self._command_u(
                idx, skill.site, tt.target, tt.y_d,
                release_offset_m=self._release_offset_m(idx, skill.site),
                shadow_landing=tt.shadow_landing)
            then_throw = ThrowAfterCatch(
                t_release_s=float(tt.t_release_abs_s),
                site_mm=release_mm,
                target_mm=(self._commanded_target_mm(tt.target, (u_dy, u_apex))
                           + self._offset_flyback_mm(idx, skill.site,
                                                     tt.y_d, u_apex)),
                flight_s=sch.flight_s(u_apex))
        if skill.rest_site_mm is not None:
            # HELD-AXIS CATCH (R4 reload): `skill.rest_site_mm` was computed
            # ONCE at compile time, from the ANNOUNCED landing --
            # `_clamp_lateral_to_schedule` above (a live re-send, `AIM_TRACKER`)
            # can move `landing.pos_mm` off that exact point, and the pinned
            # rest site would then sit off the NEW axis line through the
            # refined landing. `segments.hold_axis_site` refuses (`CATCH_AXIS`)
            # a seed off that line rather than absorbing it (its own
            # docstring), so a re-send must RE-DERIVE the rest site from the
            # refined landing at THIS catch's own hold_tilt, keeping only the
            # z the schedule chose (U2 cross-unit contract, `uc.SETTLE_CUP_
            # Z_MM`) -- not the pinned xy. A dispatch with no resend yet
            # (`landing` still the skill's own compile-time value) recomputes
            # to the SAME point, so this is a no-op there.
            rest_mm = sg.hold_axis_site(landing.pos_mm, skill.hold_tilt,
                                        float(skill.rest_site_mm[2]))
        else:
            rest_mm = skill.site.rest_site_mm()
        return CatchTerminal(landing_mm=landing.pos_mm,
                             landing_vel_mm_s=landing.vel_mm_s,
                             t_land_s=float(landing.t_land_abs_s),
                             rest_site_mm=rest_mm, then_throw=then_throw,
                             hold_tilt=skill.hold_tilt,
                             receive_tilt=skill.receive_tilt)

    def _rest_terminal(self, skill: Skill) -> RestTerminal:
        """A REST's terminal. ``rest_site_mm`` / ``tilt`` / ``holds_ball``
        pass the skill's own fields straight through — R4's PRE-TILT and
        DECAY RESTs (``schedule.compile_reload``) are the callers that set
        them to anything other than the pre-R4 defaults (the site's own
        level rest point, no tilt, holding a ball)."""
        rest_mm = (skill.rest_site_mm if skill.rest_site_mm is not None
                  else skill.site.rest_site_mm())
        return RestTerminal(rest_site_mm=rest_mm, t_rest_s=float(skill.t_abs_s),
                            tilt=skill.rest_tilt, holds_ball=skill.holds_ball)

    def _release_pos_mm(self, release_idx: int, release_site) -> np.ndarray:
        """Where the ball released by ``schedule.skills[release_idx]`` actually
        leaves the cup — the ONE place that question is answered, so every
        ballistic prior built on a release uses the release the PLAN uses.

        A plain THROW releases at its site. A CATCH's carried throw releases
        from where the ball was caught (:meth:`_catch_terminal`, 2026-09-28),
        so its position is read back from :attr:`_carried_release_mm`.

        The recorded value is the one chosen at that catch's LATEST call —
        its dispatch, or the last re-send that refined its landing. A prior
        built between the dispatch and a later re-send therefore uses a
        release that the re-send can still move, by at most the re-aim delta
        (<= :attr:`lateral_authority_m`, 20 mm): a <= 25 mm/s launch-velocity
        difference, far inside the tracker fit that supersedes this prior the
        moment the ball is in the air (:meth:`_catch_aim`, rank 1 vs rank 2).
        The site is the fallback when nothing is recorded — a release this
        executor has not planned yet, which is what every caller assumed
        before this unit.
        """
        recorded = self._carried_release_mm.get(release_idx)
        if recorded is not None:
            return np.asarray(recorded, dtype=float)
        return np.asarray(release_site.throw_site_mm(), dtype=float)

    def _offset_flyback_mm(self, idx: int, site, y_d, u_apex: float) -> np.ndarray:
        """The aim shift (mm, z = 0) that makes a carried release's offset fly
        back EXACTLY, so the learner never sees it.

        A release ``r`` off its site must put ``-r`` of lateral displacement
        into the flight for the ball to land where a release AT the site
        would have. The planner realises that displacement as a lateral
        launch component over the PLANNED flight; the plant then scales the
        whole launch vector by some speed ratio ``s`` (the hand's stroke
        overspeed; the launch lies along the cup axis with the platform
        stationary at release, so axial == the whole vector), which
        multiplies every ballistic displacement by ``s**2`` -- the apex AND
        the lateral reach alike, because both go as v^2/g. ``s**2`` is
        therefore already in hand: it is the ratio the learner commands to
        land the apex, ``g = apex_desired / u_apex`` (1.0 while the memory
        is cold). Aiming ``r * (1 - 1/g)`` toward the release side plans a
        displacement of ``-r/g`` that the plant stretches to ``-r``: the
        landing is ``site + g * u_dy``, free of ``r``, and the learner's
        ``u -> y`` map is the same one a throw from rest teaches it.

        WHY a closed form here and not the learner: without it the release
        offset is plant state a static ``u -> y`` memory cannot represent --
        each landing error is re-aimed into the next release (the catch
        follows the ball), ~(g - 1) of it returns with the opposite sign,
        and the memory, fitting ``y`` on ``u`` from rows the offset
        contaminates, hunts: the 2026-09-28 sim gate walked the landing to
        -50 mm and back over ~12 throws on seeds 0 and 3 (0 drops, 25
        makes, but the pattern left the site) with or without the offset in
        ``x[2:4]`` -- the affine fit's ``delta_x`` slope is not identifiable
        from 25 throws at one site. The offset stays in ``x`` (honest state;
        the schema slot was reserved for it), and this shift removes the
        thing it would have had to learn.

        ``g`` is clamped to [0.5, 2.0]: outside that band the apex command
        is not a plant gain but a refusal in the making (the box clips it),
        and a wild ``1/g`` must not steer the aim."""
        r_mm = self._release_offset_m(idx, site) * 1000.0
        if float(np.max(np.abs(r_mm))) == 0.0:
            return np.zeros(3)
        apex_d = float(y_d[1])
        g = apex_d / float(u_apex) if float(u_apex) > 0.0 else 1.0
        g = min(max(g, 0.5), 2.0)
        k = 1.0 - 1.0 / g
        return np.array([r_mm[0] * k, r_mm[1] * k, 0.0])

    def _release_offset_m(self, idx: int, site) -> np.ndarray:
        """How far the release planned for skill ``idx`` sits off ``site``,
        in METRES — the learner state ``x[2:4]`` (``memory.Experience``'s
        "seat-offset xy of the ball just caught", measured since 2026-09-28).

        ``(0, 0)`` when nothing is recorded for ``idx``: a plain THROW, or a
        catch whose terminal has not been built yet. One conversion, read by
        both the learner QUERY (:meth:`_command_u`, at dispatch) and the row
        that learns from it (:meth:`_register_outcome`), so the two can never
        describe the same release differently.
        """
        recorded = self._carried_release_mm.get(idx)
        if recorded is None:
            return np.zeros(2)
        return ((np.asarray(recorded, dtype=float)[:2]
                 - np.asarray(site.cup_mm, dtype=float)[:2]) / 1000.0)

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

        **R4 reload:** ``skill.landing_prior`` -- an EXTERNALLY-observed
        arrival (Ball Butler's own announcement, ``schedule.compile_reload``)
        -- takes priority over the schedule-derived prediction below, because
        such a catch has no release of its own in this schedule for
        ``_previous_release`` to find (a schedule-internal ``None`` would
        otherwise fall this catch straight to the tracker, which is the
        RIGHT fallback for a ball thrown before ``t0`` but the WRONG one here
        -- the announcement already says exactly where BB's ball is headed).

        **B2's phantom ball (``schedule.is_phantom``):** the tracker never
        runs for it (``_tracked_landing``'s own guard), so the "fall back to
        the tracker" escape above is dead for it -- a phantom catch with no
        release of its own in this schedule yet (the SAME "already airborne
        before t0" first catch the paragraph above describes, just of a
        ball that was never really thrown) would otherwise end the attempt
        ``NO_LANDING`` on a track that can never arrive. It gets the ONE
        landing a phantom can ever have: a plain self-toss at this site,
        synthesised from the pattern's own flight time (every throw in a
        columns schedule shares one apex -- ``self.schedule.flight_s``) --
        the identical ``ballistics_bc`` closed form below, just with no
        ``prev`` to read ``site``/``y_d``/``release_idx`` off, because this
        ball was never really released.
        """
        if skill.landing_prior is not None:
            lp = skill.landing_prior
            return Landing(pos_mm=lp.pos_mm, vel_mm_s=lp.vel_mm_s,
                          t_land_abs_s=lp.t_land_abs_s)
        prev = _previous_release(self.schedule, idx, skill.ball_id)
        if prev is None:
            if self.schedule.is_phantom(skill.ball_id):
                flight_s = float(self.schedule.flight_s)
                pos_mm = skill.site.catch_site_mm()
                launch_vel = ballistics_bc.launch_velocity(
                    skill.site.throw_site_mm(), pos_mm, flight_s)
                vel_mm_s = ballistics_bc.arrival_velocity(launch_vel, flight_s)
                return Landing(pos_mm=pos_mm, vel_mm_s=vel_mm_s,
                              t_land_abs_s=float(skill.t_abs_s))
            return None
        site, t_release_s, y_d, target, release_idx = prev
        pos_mm = self._commanded_target_mm(target, y_d)
        flight_s = sch.flight_s(float(y_d[1]))
        launch_vel = ballistics_bc.launch_velocity(
            self._release_pos_mm(release_idx, site), pos_mm, flight_s)
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
        the two can never disagree about what a usable landing is.

        **B2's phantom ball never reaches the tracker at all** — the
        enforcement point for "a phantom ball's catches are open-loop on
        the schedule" (``schedule.Schedule.is_phantom``'s docstring): there
        is no real ball for ``self.tracker`` to have latched onto, so
        calling it risks aiming at whatever it happens to be tracking
        instead. Returning ``None`` here sends both callers straight to
        :meth:`_predicted_landing` (the schedule's own commanded landing) —
        the fallback every catch already takes for a ball with no tracker
        answer, not a new code path.
        """
        if self.schedule.is_phantom(skill.ball_id):
            return None
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
        :meth:`_predicted_landing`, but the ballistic launch velocity's
        HAND-STROKE component scaled by ``r = v_meas / v_cmd``
        (:mod:`~jugglebot.motion.skills.hand_launch`).

        **Scale only the axial (stroke) component, not the whole vector**
        (R4, 2026-09-23 — this method's own prior comment said "revisit if
        that changes", and a cross-site hop target is exactly the case that
        changes it). ``r`` measures the fast slider stroke, which launches
        the ball ALONG the release cup's up-axis
        (``tilt_geometry.tilt_to_throw`` — the same rotation
        ``unified_cycle.cup_state_from_platform`` realises the pose
        through); the PERPENDICULAR component of the launch velocity is
        delivered by platform motion/tilt geometry, not the stroke, so
        scaling it by the same ``r`` would attribute a hand-speed error to a
        DoF the hand never touched. For a vertical throw the perpendicular
        component is exactly zero (``cup_axis(0, 0) == (0, 0, 1)``), so this
        reduces to the old isotropic ``launch_vel * r`` bit for bit (pinned:
        ``tests/motion/test_skills_executor.py::
        test_hand_corrected_landing_reduces_to_the_old_formula_for_a_vertical_throw``).
        This is still purely a TIMING correction in that case -- ``T' =
        r·T``, i.e. an apex ``r²`` times the commanded one, since a flight
        scales with the release speed and an apex with its SQUARE -- which is
        the whole "the catch is not timed" symptom: at 0.9 m apex and the
        measured r = 1.086, touch-down is ~74 ms later than the schedule
        commanded. The general (lateral hop target) case is solved, not
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
        site, t_release_s, y_d, target, release_idx = prev
        pos_mm = self._commanded_target_mm(target, y_d)
        flight_s = sch.flight_s(float(y_d[1]))
        release_pos = self._release_pos_mm(release_idx, site)
        launch_vel = ballistics_bc.launch_velocity(
            release_pos, pos_mm, flight_s)
        rx, ry = tg.tilt_to_throw(launch_vel)
        axis = tg.cup_axis(rx, ry)
        axial = float(np.dot(launch_vel, axis))
        perp = launch_vel - axis * axial
        scaled_vel = perp + axis * (axial * float(r))
        pos2, vel2, t2 = ballistics_bc.arrival_state_at_z(
            release_pos, scaled_vel, float(pos_mm[2]))
        return Landing(pos_mm=pos2, vel_mm_s=vel2,
                       t_land_abs_s=t_release_s + float(t2))

    def _valid_tracked_landing(self, idx: int, skill: Skill,
                               landing: Optional[Landing]
                               ) -> Optional[Landing]:
        """``landing`` if it can be THIS catch's ball, else ``None`` -- the
        catch half of the two-ball association contract (module comment
        above :func:`tracked_landing_refusal`), and the ONE gate the aim
        (:meth:`_catch_aim`) and the re-aim (:meth:`_resend_live_catch`) both
        read through (:meth:`_tracked_landing`). ``None`` means the catch
        keeps the schedule.

        1. It must land AFTER this ball's own previous release
           (:func:`_previous_release`): a tracker whose estimator is not reset
           until the ball's next physical release (the sim tracker) keeps
           returning the FROZEN landing of the flight that just ended. Routine,
           so refused silently.
        2. It must pass :func:`tracked_landing_refusal` against the landing
           the schedule predicts for this catch (:meth:`_predicted_landing`):
           within :func:`tracked_landing_band_s` of its instant and
           :data:`TRACKED_LANDING_LATERAL_IDENTITY_MM` of its xy. On
           2026-10-02 skill 3 (B at P1) was handed A's converged fit, one beat
           early and 100 mm over: the lateral clamp kept its TIME and the
           install refused WINDOW_TOO_SHORT (-0.228 s) on every fed-columns
           attempt. Refused once per catch with a ``TRACKER-IDENTITY-REFUSED``
           line.

        An externally announced ball (``skill.landing_prior``) is gated on the
        announcement instead, the band also capped at
        :data:`EXTERNAL_LANDING_TIME_BAND_S` (the announcement is that ball's
        timing authority). With no previous release in this schedule and no
        prior there is nothing to gate against: accept as before."""
        if landing is None:
            return None
        band_s = tracked_landing_band_s(getattr(self.schedule, 'beat_s', None))
        if skill.landing_prior is not None:
            band_s = min(EXTERNAL_LANDING_TIME_BAND_S, band_s)
        else:
            prev = _previous_release(self.schedule, idx, skill.ball_id)
            if prev is None:
                return landing
            if float(landing.t_land_abs_s) <= prev[1]:
                return None
        predicted = self._predicted_landing(idx, skill)
        refusal = tracked_landing_refusal(
            landing, predicted.t_land_abs_s, predicted.pos_mm, band_s)
        if refusal is None:
            return landing
        self._note_identity_refused(
            ('catch', idx),
            'TRACKER-IDENTITY-REFUSED skill %d (%s): %s -- not this '
            'catch\'s ball; the catch keeps the schedule'
            % (idx, ball_label(skill.ball_id), refusal), self._pending_notes)
        return None

    def _note_identity_refused(self, key, line: str, sink: List[str]) -> None:
        """Queue ``line`` on ``sink`` the first time ``key`` is refused."""
        if key in self._identity_refused:
            return
        self._identity_refused.add(key)
        sink.append(line)

    def _release_landing(self, pend: '_PendingOutcome',
                         landing: Optional[Landing], lines: List[str], *,
                         band_cap_s: Optional[float] = None
                         ) -> Tuple[Optional[Landing], Optional[str]]:
        """The release half of the association contract: ``(landing, None)``
        if ``landing`` can be ``pend``'s own flight, ``(None, reason)`` if not
        (``reason`` ``None`` too for no landing, or the silently-refused
        previous flight). The ONE gate :meth:`_advance_release_evidence` and
        :meth:`_advance_outcomes` read the tracker through.

        Same rule as the catch (:func:`tracked_landing_refusal`), against this
        release's own scheduled landing: ``t_land_scheduled_s`` (release +
        flight of the COMMANDED apex) and the commanded landing xy
        (``target_xy_mm`` + the command's lateral offset). On 2026-10-02 the
        throw-2/3 OUTCOME rows were the OTHER ball's flight -- one beat off
        ("release -556..-584 ms"), 100 mm over -- and 33 such rows reached the
        learner's memory. Logged once per throw. ``band_cap_s`` tightens the
        band further for one caller (the release evidence's own
        :data:`RELEASE_FIT_TOL_S`)."""
        if landing is None or float(landing.t_land_abs_s) <= pend.t_release_s:
            return None, None
        sched_xy = (np.asarray(pend.target_xy_mm, dtype=float)[:2]
                    + 1000.0 * np.asarray(pend.u, dtype=float)[:2])
        band_s = tracked_landing_band_s(getattr(self.schedule, 'beat_s', None))
        if band_cap_s is not None:
            band_s = min(band_s, float(band_cap_s))
        refusal = tracked_landing_refusal(
            landing, pend.t_land_scheduled_s, sched_xy, band_s)
        if refusal is None:
            return landing, None
        self._note_identity_refused(
            ('throw', pend.throw_no),
            'TRACKER-IDENTITY-REFUSED throw %d (%s): %s -- not this '
            'release\'s flight; no evidence or row from it'
            % (pend.throw_no, ball_label(pend.ball_id), refusal), lines)
        return None, refusal

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

        **Dispatch look-ahead** (``dispatch_lookahead_s``, 2026-10-02): a skill
        dispatches on the first tick with ``dispatch_s() <= t_abs_s +
        dispatch_lookahead_s``, not ``<= t_abs_s``. Since the fresh-origin
        reservation (:func:`install_segment`'s ``reserve_fresh_lead``) a fresh
        THROW/CATCH plans over ``t_event - (t_now + lead)``, so every second
        between ``dispatch_s()`` and the instant the installer actually reads
        as ``t_now`` comes straight out of the plan's window: on the robot
        that is up to one orchestrator tick here, plus ``trajectory_node``'s
        ceil-snap of ``t_now`` onto the 40 Hz knot grid, plus the service
        transit — the live first throws of 2026-10-02 planned 0.375 s
        against the 0.4 s ``launch_s`` the admissible box and
        ``plan_columns_first_cycle`` certify. Dispatching the quantisation
        early keeps the planned window at or above ``window_s`` (by at most
        the look-ahead). A splice is unaffected except that it dispatches
        that much earlier: its ``k_s`` still follows ``t_now + lead``, so its
        window only grows. The installer is still handed the real
        ``t_abs_s``, never ``dispatch_s()``.
        """
        t_abs_s = float(t_abs_s)
        lines = []
        if not self.attempt_ended:
            obs = None if self.observations is None else self.observations(t_abs_s)
            if obs is not None and not obs.in_trajectory_mode:
                self.attempt_ended = True
                self.end_code = ABORTED_MODE_CHANGED
                self.end_message = ('left the streaming mode that owns the '
                                    'platform mid-attempt')
                lines.append(
                    '%.3f END %s: left the streaming mode that owns the '
                    'platform mid-attempt — the rest tail already streaming '
                    'is the safe end' % (t_abs_s, ABORTED_MODE_CHANGED))
            elif obs is not None and obs.hand_lane_refused:
                # FAIL CLOSED, and BEFORE the deviation can grow: the rest
                # tail is NOT the safe end here, because the firmware is not
                # following the lane at all — it is holding, and every knot
                # the plan walks on widens the gap the guard measures. The
                # caller's END path installs the hand-less hold that stops
                # the walk (see HAND_LANE_REFUSED).
                self.attempt_ended = True
                self.end_code = HAND_LANE_REFUSED
                self.end_message = ('the firmware refused the streamed hand '
                                    'lane and is holding the hand')
                lines.append(
                    '%.3f END %s: the firmware refused the streamed hand lane '
                    '(sched_refused moved) and is HOLDING the hand — hold the '
                    'plan before the refused command walks into '
                    'MAX_DEVIATION' % (t_abs_s, HAND_LANE_REFUSED))
            else:
                for idx, skill in enumerate(self.schedule.skills):
                    if idx in self.dispatched:
                        continue
                    if skill.dispatch_s() > (t_abs_s
                                             + self.dispatch_lookahead_s):
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
                    lines.extend(self._advance_catch_evidence(t_abs_s))
                if not self.attempt_ended and self.observations is not None:
                    lines.extend(self._advance_release_evidence(t_abs_s))
        if self.attempt_ended and self.observations is not None:
            # An attempt that a LATER skill ended (a sibling ball's LIMIT_*
            # refusal, a mode change, a refused hand lane) can leave an
            # EARLIER catch's :class:`_PendingCatch` with its window still
            # open; `done` waits on that row, and `attempt_ended` never
            # resets, so the row must keep being visited here or the goal
            # hangs forever (audit, 2026-10-04 -- the same treatment
            # `_advance_outcomes` below already gets: outcomes keep
            # finalising after the attempt ends). Every exit of the per-row
            # loop drops the row whatever `attempt_ended` says; the live
            # ordering ("before `_advance_release_evidence`") only matters
            # while the attempt runs, which is the branch above.
            lines.extend(self._advance_catch_evidence(t_abs_s))
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
        # origin (Unit B): both `compile_one_ball` and `compile_columns`
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
            # B2's phantom ball (`columns_1ball_fed`, `schedule.is_phantom`):
            # its cup is empty by design (facts.md / `Pattern.phantom_balls`
            # docstring -- "an empty cup is the phantom's normal state"), so
            # a THROW of it must never demand the ball-evidence rows
            # (`REJECTED_BALL_UNKNOWN`/`REJECTED_NO_BALL`) -- those exist to
            # catch a REAL throw whose ball was never confirmed seated, not
            # to refuse a deliberately-empty stroke.
            codes = precondition_refusals(
                obs,
                launch=(skill.kind == THROW
                       and not self.schedule.is_phantom(skill.ball_id)),
                skip_mocap=(skill.kind == REST))
            if codes:
                self.dispatched.add(idx)
                self.attempt_ended = True
                self.end_code = codes[0]
                self.end_message = ('the precondition ladder refused %s'
                                    % (', '.join(codes),))
                self.end_kind = skill.kind
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
                    self.end_message = ('nothing to aim the catch at: no '
                                        'schedule prior and no tracker '
                                        'landing for %s'
                                        % (ball_label(skill.ball_id),))
                    self.end_kind = skill.kind
                    return (['%.3f END %s: no schedule prior and no '
                            'tracker landing for %s by the deadline '
                            '(%.3f s) — a catch cannot be aimed at a ball '
                            'nothing in this attempt has seen or thrown'
                            % (t_abs_s, NO_LANDING, ball_label(skill.ball_id),
                               deadline)],
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
            self.end_message = str(exc)
            self.end_kind = skill.kind
            return (['%.3f END %s at skill %d (%s): %s'
                    % (t_abs_s, NO_ADMISSIBLE_COMMAND, idx, skill.kind, exc)],
                   False)

        clamp_lines = self._pending_notes
        self._pending_notes = []
        res = self.installer(skill.kind, terminal, t_abs_s,
                             ball_id=skill.ball_id)
        self.dispatched.add(idx)
        self.results.append((idx, skill, res))
        if not res.accepted:
            self.attempt_ended = True
            self.end_code = res.code
            self.end_message = str(res.message)
            self.end_kind = skill.kind
            return (clamp_lines + ['%.3f END %s at skill %d (%s): %s'
                    % (t_abs_s, res.code, idx, skill.kind, res.message)],
                   False)
        lines = clamp_lines + ['%.3f %s skill %d: %s'
                % (t_abs_s, skill.kind, idx, res.message)]
        if skill.kind == CATCH:
            self._live_catch = (idx, terminal, t_abs_s)
            self._note_catch_aim(skill.ball_id, idx,
                                 _event_abs_s(CATCH, terminal))
            lines.append(
                '%.3f CATCH-AIM skill %d: source=%s landing=(%.1f, %.1f, '
                '%.1f) mm t_land=%.3f%s'
                % (t_abs_s, idx, aim_source, terminal.landing_mm[0],
                   terminal.landing_mm[1], terminal.landing_mm[2],
                   _event_abs_s(CATCH, terminal),
                   (' — awaiting the measured hand ratio'
                    if aim_source == AIM_SCHEDULE_HAND else '')))
            self._register_catch(idx, skill, _event_abs_s(CATCH, terminal))
        else:
            self._live_catch = None
        self._register_outcome(idx, skill)
        return (lines, False)

    # ── outcome capture (plan § 2.5 step 6 / § 2.7) ──

    def _register_outcome(self, idx: int, skill: Skill) -> None:
        """Start tracking the outcome of the ball ``idx``'s dispatch just
        released, if it released one and anyone is listening.

        Runs immediately after :meth:`_command_u` has cached ``idx``'s
        command, so the pending row's ``u`` is exactly what commanded the
        throw — never recomputed, never the tracker's or the schedule's own
        numbers.

        ``x[2:4]`` (the release offset, 2026-09-28) is re-read from
        :attr:`_carried_release_mm` rather than taken from the cached query
        state, so the row carries the LATEST release this catch has planned.
        The learner's QUERY used the offset known at the first dispatch; a
        re-send re-derives the release from the refined landing and can move
        it by at most the re-aim delta (<= :attr:`lateral_authority_m`,
        20 mm at the R4 launch default), and the row should describe the
        throw that actually flew, not the one first asked for. The two agree
        exactly whenever this catch was never re-sent, and a re-send after
        this registration is not chased — the row is written once.

        **B2's phantom ball (``schedule.is_phantom``) registers nothing,
        checked first and unconditionally** — the invariant ("a phantom
        ball produces motion and nothing else",
        :meth:`~jugglebot.motion.skills.schedule.Schedule.is_phantom`) holds
        whether or not a learner is wired, so this guard must not depend on
        :attr:`on_experience` being set. No pending row means
        :meth:`_advance_release_evidence` never waits on evidence for this
        release either — the two concerns (no OUTCOME/learner row, no
        release-evidence requirement) share this one list, so skipping the
        append here is the single enforcement point for both.
        """
        if self.schedule.is_phantom(skill.ball_id):
            return
        if self.on_experience is None:
            return
        if skill.kind == THROW:
            target, t_release = skill.target, float(skill.t_abs_s)
            shadow_landing = bool(skill.shadow_landing)
        elif skill.kind == CATCH and skill.then_throw is not None:
            target = skill.then_throw.target
            t_release = float(skill.then_throw.t_release_abs_s)
            shadow_landing = bool(skill.then_throw.shadow_landing)
        else:
            return
        x, u_dy, u_apex = self._u_cache[idx]
        u = np.array([u_dy[0], u_dy[1], u_apex])
        # A COPY: the cache holds the state the learner was queried with and
        # must keep saying so.
        x = np.asarray(x, dtype=float).copy()
        x[2:4] = self._release_offset_m(idx, skill.site)
        target_xy_mm = np.asarray(target.catch_site_mm(), dtype=float)[:2]
        self._n_registered += 1
        self._pending_outcomes.append(_PendingOutcome(
            ball_id=skill.ball_id, x=x, u=u, t_release_s=t_release,
            t_land_scheduled_s=t_release + sch.flight_s(u_apex),
            target_xy_mm=target_xy_mm,
            t_next_release_s=_next_release(self.schedule, idx, skill.ball_id),
            throw_no=self._n_registered,
            release_z_mm=float(skill.site.throw_site_mm()[2]),
            shadow_landing=shadow_landing))

    def _register_catch(self, idx: int, skill: Skill,
                        t_land_scheduled_s: float) -> None:
        """Arm the :data:`MISSED_CATCH` evidence rule for CATCH ``idx``, if
        it can be armed (R5 sitting 4, ``report_a1.md`` Q3).

        Off entirely when :attr:`seat_window` is unset (R2/R3 callers, and
        the sim gate unless it wires one) — same shape as
        :meth:`_register_outcome`'s ``on_experience`` gate. Also skipped,
        unconditionally, for a phantom ball
        (:meth:`~jugglebot.motion.skills.schedule.Schedule.is_phantom`) and
        for a shadow landing (``skill.shadow_landing`` -- always False for a
        CATCH itself, kept for symmetry with
        :func:`~jugglebot.motion.skills.schedule.schedule_has_shadow_landing`'s
        own check -- or ``skill.then_throw.shadow_landing``, this catch's
        OWN carried release being the columns Stop's cross-site throw): the
        false-positive list in ``report_a1.md`` Q3 names both as "never
        register" cases, not "register then veto".

        UNARMED (no row at all, the rule simply does not apply to this
        catch) when ``t_open`` cannot be anchored (no scheduled release
        before this landing at all) or comes out past the already-armed
        :data:`CAUGHT_LEAD_S` floor the OUTCOME ladder uses, or when the
        other-ball landing guard collapses the window to empty or negative.
        """
        if self.seat_window is None:
            return
        if self.schedule.is_phantom(skill.ball_id):
            return
        if bool(skill.shadow_landing) or (
                skill.then_throw is not None
                and bool(skill.then_throw.shadow_landing)):
            return
        L = float(t_land_scheduled_s)
        latest_release = _latest_release_before(self.schedule, L)
        if latest_release is None:
            return
        t_open = latest_release + MISSED_CATCH_OPEN_AFTER_RELEASE_S
        if t_open > L - CAUGHT_LEAD_S:
            return
        if skill.then_throw is not None:
            t_close = (float(skill.then_throw.t_release_abs_s)
                      + MISSED_CATCH_CLOSE_AFTER_RELEASE_S)
        else:
            t_close = L + MISSED_CATCH_CLOSE_AFTER_LANDING_S
        other_landing = _next_landing_other_ball(self.schedule,
                                                 skill.ball_id, L)
        if other_landing is not None:
            t_close = min(t_close,
                         other_landing - MISSED_CATCH_OTHER_LANDING_GUARD_S)
        if t_close <= t_open:
            return
        self._pending_catches.append(_PendingCatch(
            ball_id=skill.ball_id, catch_idx=idx, t_land_scheduled_s=L,
            t_open_s=t_open, t_close_s=t_close))

    def _note_catch_aim(self, ball_id: int, catch_idx: int,
                        t_land_abs_s: float, reaim: bool = False) -> None:
        """Record a committed catch aim on the flight it meets — the
        released, not-yet-finalised row of ``ball_id`` whose SCHEDULED landing
        is nearest the aim (a carried throw's own new row lands a beat later,
        so it is never the nearest). For the operator's ``arrival`` only
        (:mod:`report`); no learner row reads it."""
        rows = [pend for pend in self._pending_outcomes
                if pend.ball_id == ball_id
                and pend.t_release_s < float(t_land_abs_s)]
        if not rows:
            return
        pend = min(rows, key=lambda p: abs(p.t_land_scheduled_s
                                           - float(t_land_abs_s)))
        pend.t_catch_aim_s = float(t_land_abs_s)
        pend.catch_idx = int(catch_idx)
        if reaim:
            pend.reaims += 1

    def _note_reaim_refused(self, catch_idx: int, code: str) -> None:
        """Record a refused re-aim on the flight catch ``catch_idx`` meets
        (see :meth:`_note_catch_aim`)."""
        for pend in self._pending_outcomes:
            if pend.catch_idx == int(catch_idx):
                pend.reaim_refused = str(code)

    def stop_terminals(self, t_now_s: float
                       ) -> List[Tuple[str, RestTerminal]]:
        """The REST terminals that stop the machine short of the next
        pending release, in the order to try them (owner decision
        2026-09-29: always make the first throw after a catch, then stop
        before the second empty one).

        The pending release is the earliest one still ahead of ``t_now_s``
        among this attempt's ACCEPTED installs (a THROW's release, or a
        CATCH's carried one); the REST comes to rest, level, at the site of
        the skill that carries it -- where the platform is already heading,
        with the hand at home. Returns ``[]`` when no release is pending:
        the streaming plan's own rest tail is then the stop.

        * ``'before'`` -- at rest :data:`STOP_BEFORE_MARGIN_S` before that
          release, so the throw never runs. ``holds_ball`` stays True, because
          a ball may still be in the cup (an operator Stop mid-carry). It is
          left out when its window cannot fit after the install's lead
          (``LEAD_S + MIN_WINDOW_S``).
        * ``'after'`` -- at rest :data:`STOP_AFTER_S` after it. The splice
          snaps to the release and is seeded post-release, so the cup is
          empty (``holds_ball`` False, as the post-release seed requires).

        The caller installs these through the ordinary installer and falls
        back to the legacy ``trajectory/hold`` only when both refuse. It is a
        method on the executor because only the executor knows which skills
        are on the machine and which release each carries.

        The site is the skill's own ``site.rest_site_mm()``, level. Only a
        THROW or a CATCH carrying a throw is ever chosen, and neither carries
        a ``rest_site_mm`` override today: the R4 reload's held-axis CATCH,
        the one skill that does, is a plain catch, and its pattern's first
        release is the THROW after the DECAY REST, at a level site
        (``schedule.compile_reload``).
        """
        accepted = {idx for idx, _sk, res in self.results if res.accepted}
        pending = []
        for idx in accepted:
            sk = self.schedule.skills[idx]
            if sk.kind == THROW:
                t_rel = float(sk.t_abs_s)
            elif sk.kind == CATCH and sk.then_throw is not None:
                t_rel = float(sk.then_throw.t_release_abs_s)
            else:
                continue
            if t_rel > float(t_now_s):
                pending.append((t_rel, idx))
        if not pending:
            return []
        t_rel, idx = min(pending)
        rest_mm = self.schedule.skills[idx].site.rest_site_mm()
        stops = []
        t_before = t_rel - STOP_BEFORE_MARGIN_S
        if t_before - (float(t_now_s) + LEAD_S) >= MIN_WINDOW_S:
            stops.append(('before', RestTerminal(
                rest_site_mm=rest_mm, t_rest_s=t_before, holds_ball=True)))
        stops.append(('after', RestTerminal(
            rest_site_mm=rest_mm, t_rest_s=t_rel + STOP_AFTER_S,
            holds_ball=False)))
        return stops

    def _survivor_catch(self, dropped_ball_id: int
                        ) -> Optional[Tuple[int, Skill]]:
        """The nearest undispatched CATCH for a ball OTHER than
        ``dropped_ball_id`` -- the D3 survivor policy's anchor (owner
        decision, 2026-09-30, R5 rescope). ``None`` when no other ball has
        one left (a one-ball schedule, or every other CATCH already
        dispatched), in which case the caller falls back to
        :data:`ABORTED_NO_RELEASE` unchanged.

        A phantom ball's own CATCH is never a candidate (B2,
        ``columns_1ball_fed``, ``schedule.is_phantom``): it has no ball in
        it by design, so "continuing to catch it" is not a real survivor --
        a dropped real ball with a phantom partner must fall through to the
        ordinary end exactly as a genuinely one-ball schedule does.
        ``dropped_ball_id`` itself is never a phantom ball here: both
        callers (:meth:`_advance_release_evidence`, :meth:`_advance_catch_evidence`)
        only ever see ``pend.ball_id`` from a row :meth:`_register_outcome`/
        :meth:`_register_catch` registered, and both already refuse to
        register a phantom ball's row."""
        candidates = [(j, sk) for j, sk in enumerate(self.schedule.skills)
                     if j not in self.dispatched and sk.kind == CATCH
                     and sk.ball_id != dropped_ball_id
                     and not self.schedule.is_phantom(sk.ball_id)]
        if not candidates:
            return None
        return min(candidates, key=lambda pair: float(pair[1].t_abs_s))

    def _install_survivor_tail(self, y_idx: int, y_skill: Skill,
                               dropped_ball_id: int) -> None:
        """D3: cut the schedule to ``y_skill`` (its own carried throw
        stripped -- nothing is thrown after a drop) plus one closing REST at
        its site, the same fresh-origin margin every other closing REST uses
        (:func:`~jugglebot.motion.skills.schedule.closing_rest_t_abs`).

        Indices ``0 .. y_idx - 1`` are UNCHANGED OBJECTS for whatever is
        already DISPATCHED: :attr:`dispatched` and :attr:`results` are
        indexed by position, so anything already dispatched must keep the
        exact index and plan it was installed under.
        :func:`~jugglebot.motion.skills.schedule._assign_leads` runs over
        the WHOLE new list rather than just the tail -- it is a pure
        function of the ordered sequence up to each index, so the prefix's
        leads come back identical to what they already were (this is
        provable, not merely checked: its loop state at index ``k`` depends
        only on skills ``0 .. k-1``, which the kept prefix does not change).

        ``dropped_ball_id``'s own UNDISPATCHED skills before ``y_idx`` are
        DROPPED from the prefix (R5 sitting 4, ``report_a1.md`` Q1 fault
        (b) / Q3 (4)): without this, a drop verdict reached EARLY -- before
        every one of the dropped ball's own later, still-scheduled skills
        has been dispatched -- left them in the kept prefix verbatim, so
        they still ran (sitting 4 attempts 1/17/19/R1/R2: the dropped ball's
        later rows timed out LATER and overwrote this very end_code with
        ``ABORTED_NO_RELEASE``). Dropping only indices ``>= len(dispatched)``
        preserves the index invariant above: :attr:`dispatched` is a
        CONTIGUOUS prefix ``{0, .., k-1}`` by construction (:meth:`tick`
        dispatches in schedule order and stops at the first not-yet-due or
        refused skill), so nothing below ``k`` is ever filtered and nothing
        at or above it is ever referenced by :attr:`dispatched`/
        :attr:`results`.

        ``y_skill`` is never a phantom ball's CATCH (B2, ``columns_1ball_fed``):
        :meth:`_survivor_catch` already excludes phantom candidates, so this
        method only ever installs a tail around a real ball's catch.
        """
        k = len(self.dispatched)
        assert self.dispatched == set(range(k)), (
            'dispatched must be a contiguous prefix {0, .., k-1} for the '
            'survivor-tail filter to drop indices >= k safely -- got %r'
            % (sorted(self.dispatched),))
        prefix = [sk for i, sk in enumerate(self.schedule.skills[:y_idx])
                 if i < k or sk.ball_id != dropped_ball_id]
        stripped = dataclasses.replace(y_skill, then_throw=None)
        rest_t = sch.closing_rest_t_abs(float(y_skill.t_abs_s), sg.REST_TAIL_S)
        rest = Skill(kind=REST, ball_id=y_skill.ball_id, site=y_skill.site,
                    t_abs_s=rest_t, window_s=sg.REST_TAIL_S)
        new_tail = sch._assign_leads(prefix + [stripped, rest])
        self.schedule = dataclasses.replace(self.schedule,
                                            skills=tuple(new_tail))

    def _advance_catch_evidence(self, t_abs_s: float) -> List[str]:
        """Fire :data:`MISSED_CATCH` for any armed :class:`_PendingCatch`
        whose decision instant has arrived -- called each tick from
        :meth:`tick`, BEFORE :meth:`_advance_release_evidence` (R5 sitting 4,
        2026-10-04 evening, ``report_a1.md`` Q3), only when ``observations``
        is wired -- the same gate every other PORT@R3 row runs under.

        Off entirely when :attr:`seat_window` is unset: :attr:`_pending_
        catches` is then always empty (:meth:`_register_catch` never
        appends to it without one), so this is a no-op loop over nothing --
        R2/R3 callers, and the sim gate unless it wires one, are unaffected.

        At the first tick ``t_abs_s >= t_close + MISSED_CATCH_DECIDE_LAG_S``:
        query :attr:`seat_window` over ``[t_open, t_close]``. ``None`` (the
        buffer does not cover the window) is BLIND -- no verdict, the row is
        simply dropped. Otherwise :data:`MISSED_CATCH` fires iff the window
        was LIVE (``n_valid`` at least :data:`MISSED_CATCH_MIN_VALID_FRAC`
        of a live 100 Hz feed's sample count over the span, and no gap
        wider than :data:`MISSED_CATCH_MAX_GAP_S`) AND it saw no RAW-SEATED
        sample at all (``n_seated == 0``, the raw bit only -- see
        :class:`SeatWindow`) -- a single raw-seated sample anywhere in the
        window vetoes the miss, the safe direction (a real but messy catch
        is never worth risking a false positive over).

        On fire: every one of ball ``b``'s own pending OUTCOME rows
        (:attr:`_pending_outcomes`) is dropped too -- no row: ball ``b``'s
        catch produced no ball (Q1 fault (b): the catch's own carried
        throw, if any, was already dispatched open-loop and would
        otherwise time out LATER and overwrite this verdict with
        ``ABORTED_NO_RELEASE``/``DROPPED_SURVIVOR_STOPPED``). Then the D3
        shape, exactly as :meth:`_advance_release_evidence`'s own drop:
        :meth:`_survivor_catch` found -- :meth:`_install_survivor_tail`
        cuts the schedule to it (``attempt_ended`` stays False, dispatch
        continues); not found -- ``attempt_ended`` is set True and the
        normal stop path (``skill_node._maybe_hold_pending_event`` /
        :meth:`stop_terminals`) brings the machine to rest. ``end_code`` is
        :data:`MISSED_CATCH` either way, and ``end_kind`` is set to
        :data:`CATCH` so the operator line reads "... ENDED (MISSED_CATCH)
        at the CATCH: ...".
        """
        if not self._pending_catches:
            return []
        lines = []
        remaining = []
        for pend in self._pending_catches:
            if t_abs_s < pend.t_close_s + MISSED_CATCH_DECIDE_LAG_S:
                remaining.append(pend)
                continue
            window = self.seat_window(pend.t_open_s, pend.t_close_s)
            if window is None:
                continue                 # blind: no verdict, row dropped
            span_s = pend.t_close_s - pend.t_open_s
            expected = 100.0 * span_s
            live = (expected <= 0.0
                   or (float(window.n_valid) / expected)
                      >= MISSED_CATCH_MIN_VALID_FRAC)
            live = live and float(window.max_gap_s) <= MISSED_CATCH_MAX_GAP_S
            if not live or window.n_seated > 0:
                continue                 # confirmed, or not trustworthy
            self._pending_outcomes = [
                p for p in self._pending_outcomes
                if p.ball_id != pend.ball_id]
            t_open_rel = pend.t_open_s - pend.t_land_scheduled_s
            t_close_rel = pend.t_close_s - pend.t_land_scheduled_s
            survivor = self._survivor_catch(pend.ball_id)
            if survivor is not None and not self.attempt_ended:
                self._install_survivor_tail(*survivor, pend.ball_id)
                self.end_code = MISSED_CATCH
                self.end_kind = CATCH
                self.end_message = (
                    'the CATCH of %s (skill %d) produced no ball -- '
                    '%s caught, no further throws'
                    % (ball_label(pend.ball_id), pend.catch_idx,
                       ball_label(survivor[1].ball_id)))
                lines.append(
                    '%.3f END %s: the CATCH of %s (skill %d) produced '
                    'no ball -- cup EMPTY on all %d valid samples over '
                    '[%+.2f, %+.2f] s of its scheduled landing -- %s '
                    'continues to its own catch (skill %d), then rest'
                    % (t_abs_s, MISSED_CATCH, ball_label(pend.ball_id),
                       pend.catch_idx, window.n_valid, t_open_rel, t_close_rel,
                       ball_label(survivor[1].ball_id), survivor[0]))
                continue
            if not self.attempt_ended:
                self.attempt_ended = True
                self.end_code = MISSED_CATCH
                self.end_kind = CATCH
                self.end_message = (
                    'the CATCH of %s (skill %d) produced no ball'
                    % (ball_label(pend.ball_id), pend.catch_idx))
                lines.append(
                    '%.3f END %s: the CATCH of %s (skill %d) produced '
                    'no ball -- cup EMPTY on all %d valid samples over '
                    '[%+.2f, %+.2f] s of its scheduled landing -- no ball '
                    'left to catch, stopped'
                    % (t_abs_s, MISSED_CATCH, ball_label(pend.ball_id),
                       pend.catch_idx, window.n_valid, t_open_rel,
                       t_close_rel))
            else:
                # The attempt was already ended by a later skill (a
                # refusal, a mode change): the verdict still resolves this
                # row and the ball's outcome rows -- nothing to dispatch.
                lines.append(
                    '%.3f %s (attempt already ended): the CATCH of %s '
                    '(skill %d) produced no ball -- cup EMPTY on all %d '
                    'valid samples over [%+.2f, %+.2f] s of its scheduled '
                    'landing -- its outcome rows dropped'
                    % (t_abs_s, MISSED_CATCH, ball_label(pend.ball_id),
                       pend.catch_idx, window.n_valid, t_open_rel,
                       t_close_rel))
            continue                     # dropped either way
        self._pending_catches = remaining
        return lines

    def _advance_release_evidence(self, t_abs_s: float) -> List[str]:
        """Confirm every accepted release actually left the hand (PORT@R3,
        INVARIANTS.md § 8 ``ABORTED_NO_RELEASE``) -- called each tick from
        :meth:`tick`, only when ``observations`` is wired.

        Evidence must belong to THIS ball's release, not a previous flight
        still in progress when a carried throw's row was registered (a CATCH
        with a ``then_throw`` registers up to a beat before its own
        release, while the cup still carries the ball from the PRECEDING
        throw):

        * possession evidence (:attr:`observer`) -- a SEATED reading at or
          before ``t_release_s + RELEASE_SEAT_EPS_S`` latches
          :attr:`_PendingOutcome.seated_seen` (Q4, 2026-10-04 evening: a
          SEATED sample seen only AFTER that -- another ball's arrival, or
          this one's own settle chatter past the window -- must never arm
          it); an EMPTY reading before :attr:`_PendingOutcome.t_release_s`
          clears it (that EMPTY belongs to the previous ball's flight, not
          this release); an EMPTY reading AT OR AFTER ``t_release_s``
          confirms the release only if ``seated_seen`` is set -- i.e. EMPTY
          must follow a SEATED sample taken at or before the epsilon;
        * tracker evidence (:attr:`tracker`) -- a landing confirms the
          release only when sampled at or after ``t_release_s``, the
          landing's own ``t_land_abs_s`` is after ``t_release_s`` (a stale
          landing from the previous flight, still returned by a tracker
          whose correlation has not yet re-latched, must not confirm this
          release), it comes from a CONVERGED fit, and it lands within
          :data:`RELEASE_FIT_TOL_S` of the scheduled touch-down (2026-09-29:
          a bouncing dropped ball's unfitted estimate had confirmed an empty
          throw).

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
                    if (ev == bp.EVIDENCE_SEATED
                            and t_abs_s <= pend.t_release_s
                                       + RELEASE_SEAT_EPS_S):
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
                            and bool(getattr(landing, 'from_fit', False))):
                        # The association gate subsumes the old
                        # RELEASE_FIT_TOL_S check: after release, inside
                        # the landing-time band, inside the identity bound.
                        landing, _refusal = self._release_landing(
                            pend, landing, lines,
                            band_cap_s=RELEASE_FIT_TOL_S)
                        if landing is not None:
                            pend.release_confirmed = True
            if (not pend.release_confirmed
                    and t_abs_s >= pend.t_release_s + RELEASE_GRACE_S):
                survivor = self._survivor_catch(pend.ball_id)
                if survivor is not None and not self.attempt_ended:
                    # D3 (owner decision, 2026-09-30, R5 rescope): another
                    # ball still has a CATCH coming -- keep catching it
                    # instead of ending the attempt. Cut the schedule to
                    # that catch (its own carried throw stripped -- nothing
                    # is thrown after a drop) plus a closing REST at its
                    # site. `end_code` is set here so the eventual clean
                    # finish still names the drop; `attempt_ended` stays
                    # False so `tick` keeps dispatching.
                    self._install_survivor_tail(*survivor, pend.ball_id)
                    self.end_code = DROPPED_SURVIVOR_STOPPED
                    self.end_message = (
                        'throw %d/%d never left the hand (no release '
                        'evidence by +%.1f s) -- %s caught, no '
                        'further throws'
                        % (pend.throw_no, self.n_throws, RELEASE_GRACE_S,
                           ball_label(survivor[1].ball_id)))
                    lines.append(
                        '%.3f DROP %s: no release evidence for %s by '
                        't_release + %.1f s -- %s continues to its own '
                        'catch, then rest'
                        % (t_abs_s, DROPPED_SURVIVOR_STOPPED,
                           ball_label(pend.ball_id), RELEASE_GRACE_S,
                           ball_label(survivor[1].ball_id)))
                    continue
                if not self.attempt_ended:
                    self.attempt_ended = True
                    self.end_code = ABORTED_NO_RELEASE
                    self.end_message = (
                        'throw %d/%d never left the hand (no release '
                        'evidence by +%.1f s)'
                        % (pend.throw_no, self.n_throws, RELEASE_GRACE_S))
                    lines.append(
                        '%.3f END %s: no release evidence for %s by '
                        't_release + %.1f s -- no learner row'
                        % (t_abs_s, ABORTED_NO_RELEASE, ball_label(pend.ball_id),
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

        Two uses, deliberately ONE instant: it anchors ``t_open``
        (:meth:`_outcome_window`) and it decides which of several open rows a
        single SEATED sample belongs to (:meth:`_advance_outcomes`). (It was
        also the admission freeze until 2026-09-20; the freeze now sits at the
        next release -- :meth:`_consider_landing` test 3.)

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
        2. RETIRED 2026-09-20 (kept in the numbering so the tests read the
           same): "the sample is taken at least ``OUTCOME_GUARD_S`` before the
           crossing it predicts" guarded a KALMAN estimate, least trustworthy
           at its own crossing. A converged fit is the same parabola read
           before or after its crossing, so the guard only cost rows: it
           refused every fit that converged after the landing.
        3. The sample is taken BEFORE this ball's next scheduled release (less
           :data:`OUTCOME_NEXT_RELEASE_EPS_S`): past that instant the row is
           FROZEN -- no later tick can move it. Until 2026-09-20 the freeze sat
           at the CROSSING, the structural fix for the 2026-09-16
           contamination (one continuous tracker track through catch and
           re-throw, so every post-catch estimate was the NEXT flight's --
           2.23 s and 1.16 s flights for a 0.857 s command, ``armB-090``
           attempt 1). Two things retired that need: every announcement mints
           its own tracker id and the host correlates per RELEASE
           (2026-09-16), so an estimate served for this row is this flight's
           fit whenever it is read; and the outcome is now the fit's APEX
           (2026-09-18), the same parabola after the landing as before it, so
           a fit that converges after the crossing still yields the row.
        4. The estimate's own crossing is BEFORE this ball's next scheduled
           release (less :data:`OUTCOME_NEXT_RELEASE_EPS_S`). A landing at or
           after the instant the ball leaves the cup again belongs to the next
           flight no matter which tick it was sampled on — test 3 bounds WHEN
           a sample may be taken, this one bounds WHAT it may claim, and on a
           CROWDED schedule (re-release before the scheduled landing) an
           in-flight sample can still point past the bound. Both stop at the
           same instant, on the same margin (:meth:`_next_release_bound`,
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
        apex = _observed_apex_m(landing)
        u_apex = float(pend.u[2])
        bound = SkillExecutor._next_release_bound(pend)
        if bound is not None and t_abs_s >= bound:
            return False                       # test 3: FROZEN at the next release
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
        pend.best_lead_s = t_land - t_abs_s   # may be negative since 2026-09-20
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
                landing = self.tracker(pend.ball_id)
                if landing is not None and landing.from_fit:
                    # Only a fit can become a row; an unfitted estimate is
                    # left to `_consider_landing`, which names it as such.
                    raw = landing
                    landing, refusal = self._release_landing(
                        pend, landing, lines)
                    if refusal is not None:
                        pend.rejected_apex_m = _observed_apex_m(raw)
                        pend.rejected_reason = ('not this release\'s flight: '
                                                + refusal)
                        pend.n_rejected += 1
                self._consider_landing(pend, landing, t_abs_s)

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

    def _report(self, pend: _PendingOutcome, *, row: bool,
                no_row_reason: str = '',
                apex_m: Optional[float] = None) -> None:
        """Append ``pend``'s :class:`~jugglebot.motion.skills.report.
        ThrowReport` to :attr:`reports` — every finalised release gets exactly
        one, row or no row. Reads only what the row already holds."""
        landing = pend.best_landing
        landing_err = release_err = arrival_err = None
        if landing is not None:
            d = (np.asarray(landing.pos_mm, dtype=float)[:2]
                 - np.asarray(pend.target_xy_mm, dtype=float))
            landing_err = (float(d[0]), float(d[1]))
            release_err = release_error_s(
                float(landing.t_land_abs_s),
                float(np.asarray(landing.pos_mm, dtype=float)[2]),
                float(np.asarray(landing.vel_mm_s, dtype=float)[2]),
                float(pend.release_z_mm), float(pend.t_release_s))
            if pend.t_catch_aim_s is not None:
                arrival_err = (float(landing.t_land_abs_s)
                               - float(pend.t_catch_aim_s))
        # `shadow_landing` overrides the possession latch here too, for the
        # SAME reason `_finalise_outcome` overrides it on the learner's
        # ``Experience`` -- the operator's throw line must not read CAUGHT
        # off a sensor that is reading the PREVIOUS ball's possession.
        caught = (False if pend.shadow_landing
                 else (bool(pend.caught_seen) if self.observer is not None
                       else None))
        self.reports.append(ThrowReport(
            throw_no=pend.throw_no, n_throws=self.n_throws,
            ball_id=pend.ball_id, caught=caught,
            row=bool(row), no_row_reason=no_row_reason,
            apex_m=None if apex_m is None else float(apex_m),
            landing_err_mm=landing_err, release_err_s=release_err,
            arrival_err_s=arrival_err,
            seat_s=(None if pend.t_seat_s is None
                    else float(pend.t_seat_s) - float(pend.t_land_scheduled_s)),
            reaims=pend.reaims, reaim_refused=pend.reaim_refused))

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
                self._report(pend, row=False,
                             no_row_reason='%s (%d estimate(s) refused)'
                             % (pend.rejected_reason, pend.n_rejected),
                             apex_m=pend.rejected_apex_m)
                return ['%.3f OUTCOME %s: no row: %s%s (%d estimate(s) '
                        'refused)' % (finalise_at, ball_label(pend.ball_id),
                                      seen, pend.rejected_reason,
                                      pend.n_rejected)]
            self._report(pend, row=False,
                         no_row_reason='no landing estimate was ever observed')
            return ['%.3f OUTCOME %s: no landing estimate was ever '
                    'observed — no row'
                    % (finalise_at, ball_label(pend.ball_id))]
        apex_obs_m = _observed_apex_m(pend.best_landing)
        # The PHYSICAL band (``memory.APEX_RATIO_BAND``): an observed apex
        # that is not a multiple near 1 of the commanded one is not an
        # observation of this throw at all -- it is the next flight's landing,
        # or a diverged filter. Belt and braces with the freeze in
        # `_consider_landing`, and the last line of defence before the row
        # reaches the learner's memory (which refuses it again on append).
        if not apex_in_band(u_apex, apex_obs_m):
            self._report(pend, row=False, apex_m=apex_obs_m,
                         no_row_reason='observed apex %.3f m outside '
                         '[%.3f, %.3f] of commanded %.3f m'
                         % (apex_obs_m, APEX_RATIO_BAND[0] * u_apex,
                            APEX_RATIO_BAND[1] * u_apex, u_apex))
            return ['%.3f OUTCOME %s: no row: observed apex %.3f m '
                    'outside [%.3f, %.3f] of commanded %.3f m'
                    % (finalise_at, ball_label(pend.ball_id), apex_obs_m,
                       APEX_RATIO_BAND[0] * u_apex,
                       APEX_RATIO_BAND[1] * u_apex, u_apex)]
        landing_xy_m = ((np.asarray(pend.best_landing.pos_mm, dtype=float)[:2]
                        - pend.target_xy_mm) / 1000.0)
        y = np.array([landing_xy_m[0], landing_xy_m[1], apex_obs_m])
        # The LATCH, not a fresh sample: by ``finalise_at`` a chained catch has
        # often already re-thrown the ball, so the cup at this instant says
        # nothing about whether it was caught (see :meth:`_outcome_window`).
        #
        # `shadow_landing` OVERRIDES the possession latch: this ball is the
        # columns Stop's cross-site last throw (D2, E1', 2026-09-30) -- the
        # cup already reads SEATED throughout from the PREVIOUS ball it is
        # holding, which is evidence about that ball, not this one.
        caught = False if pend.shadow_landing else bool(pend.caught_seen)
        # NO memory row for a shadow-landed throw: `u = y_d` exactly
        # (`_command_u`'s bypass) so there is nothing for the learner to fit
        # against, and the command never came off a box this segment shape
        # was swept for -- an Experience built from it would teach the
        # memory a command/outcome pair from a throw it never chose. The
        # operator's report still gets the real observed landing/apex below
        # (`row=True`): only the memory row is skipped, not the account of
        # what happened.
        if not pend.shadow_landing:
            exp = Experience(x=pend.x, u=pend.u, y=y, t_abs_s=pend.t_release_s,
                             ball_id=pend.ball_id, caught=bool(caught))
            self.on_experience(exp)
        self._report(pend, row=True, apex_m=apex_obs_m)
        phase = ('' if pend.t_seat_s is None
                 else ' seat=%+.3f s vs scheduled landing'
                 % (pend.t_seat_s - float(pend.t_land_scheduled_s)))
        shadow = ' (landed on the held ball)' if pend.shadow_landing else ''
        return ['%.3f OUTCOME %s: y=(%.4f, %.4f) m apex=%.4f m '
                'caught=%s%s%s'
                % (finalise_at, ball_label(pend.ball_id), y[0], y[1], y[2],
                   caught, phase, shadow)]

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

        The first two timing fences are :meth:`_resend_live_catch`'s, for the
        same two physical facts: nothing re-sends inside ``catch_freeze_s`` of
        touch-down (the hand is already decelerating into the ball), and
        nothing re-sends once ``now + lead_s`` leaves less than
        :data:`MIN_WINDOW_S` before the landing (a solve there can only
        refuse ``WINDOW_TOO_SHORT``). Both are checked against the CORRECTED
        landing as well as the committed one -- a correction that pulls
        touch-down EARLIER (r < 1) can land inside a freeze the committed aim
        cleared. A third fence is checked ONCE, against the corrected landing
        only, right before the solve it would otherwise pay for: nothing
        re-sends once the splice would open inside the cup-contact window (a
        solve there can only refuse ``CUP_CONTACT_ACC`` — see
        :data:`CUP_CONTACT_WINDOW_LEAD_S`). It needs no earlier stage against
        the committed landing the way the two above do -- those gate whether
        computing ``r`` is worth it at all, while this one exists only to
        skip the SOLVE, and by the time ``r`` and the corrected landing are
        known, checking the value the solve would actually target is both
        necessary and sufficient (the committed landing's own touch-down can
        differ from the corrected one in either direction, so it would not
        reliably predict this fence's answer anyway).
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
        # DECLINE BEFORE THE SOLVE — the same physical fact
        # :meth:`_resend_live_catch` declines on (see
        # :data:`CUP_CONTACT_WINDOW_LEAD_S`): a splice opening inside the
        # cup-contact window hands the QP no runway to reach
        # ``CUP_CONTACT_ACC`` from a re-aimed seed, so the solve can only
        # refuse it.
        moved_mm = float(np.max(np.abs(
            np.asarray(landing.pos_mm, dtype=float) - terminal.landing_mm)))
        if (t_abs_s + float(skill.lead_s)
                >= float(landing.t_land_abs_s) - CUP_CONTACT_WINDOW_LEAD_S):
            return self._resend_declined(
                t_abs_s, idx, 'CONTACT-WINDOW', moved_mm, dt_s)
        new_terminal = self._catch_terminal(idx, skill, landing)
        clamp_lines = self._pending_notes
        self._pending_notes = []
        res = self.installer(CATCH, new_terminal, t_abs_s,
                             ball_id=skill.ball_id)
        self.results.append((idx, skill, res))
        if not res.accepted:
            # The committed catch stands — a refused re-aim is strictly
            # better than no catch, so the attempt continues.
            self._live_catch = (idx, terminal, t_abs_s)
            self._note_reaim_refused(idx, res.code)
            return clamp_lines + [
                    '%.3f CATCH-AIM-HAND-REFUSED %s skill %d: r=%.3f, '
                    'Δt=%+.3f s — %s'
                    % (t_abs_s, res.code, idx, r, dt_s, res.message)]
        self._live_catch = (idx, new_terminal, t_abs_s)
        self._note_catch_aim(skill.ball_id, idx,
                             _event_abs_s(CATCH, new_terminal), reaim=True)
        return clamp_lines + ['%.3f CATCH-AIM skill %d: source=%s r=%.3f Δt=%+.3f s '
                'landing=(%.1f, %.1f, %.1f) mm t_land=%.3f'
                % (t_abs_s, idx, AIM_SCHEDULE_HAND, r, dt_s,
                   new_terminal.landing_mm[0], new_terminal.landing_mm[1],
                   new_terminal.landing_mm[2], new_terminal.t_land_s)]

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

        Six fences, one physical fact each (the sixth, 2026-09-20: a refused
        re-solve ends re-aiming for that catch -- see the refused branch). Two are TIMING and unchanged
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
        # The "moved" test is against the CLAMPED landing, not the tracker's
        # raw one: a fit that only asks for a lateral move the catch may not
        # take (:meth:`_clamp_lateral_to_schedule`) has not moved the catch
        # at all, and must not spend a re-send fence-checking a move that
        # never reaches the platform (2026-09-18).
        clamped = self._clamp_lateral_to_schedule(idx, skill, landing)
        clamp_lines = self._pending_notes
        self._pending_notes = []
        moved_mm = float(np.max(np.abs(
            np.asarray(clamped.pos_mm, dtype=float) - terminal.landing_mm)))
        moved_s = float(clamped.t_land_abs_s) - float(terminal.t_land_s)
        if not landing.from_fit:
            return clamp_lines + self._resend_declined(
                t_abs_s, idx, 'NO-CONVERGED-FIT', moved_mm, moved_s)
        if (moved_mm <= self.resend_pos_tol_mm
                and abs(moved_s) <= self.resend_t_tol_s):
            return clamp_lines + self._resend_declined(
                t_abs_s, idx, 'WITHIN-TOLERANCE', moved_mm, moved_s)
        if self._resend_counts.get(idx, 0) >= self.resend_max_per_catch:
            return clamp_lines + self._resend_declined(
                t_abs_s, idx, 'CAP-SPENT', moved_mm, moved_s)
        # DECLINE BEFORE THE SOLVE (plan § 0 carry-in, R4 2026-09-23): the
        # splice this re-send would open at (its dispatch, `t_abs_s`, plus
        # this skill's own lead) must land BEFORE the cup-contact window
        # starts, `clamped.t_land_abs_s - CUP_CONTACT_WINDOW_LEAD_S` — a
        # splice that opens inside the contact window hands the QP a
        # re-solve with no runway left to reach the `CUP_CONTACT_ACC` floor
        # from whatever state the re-aim seeds, so the solve can only refuse
        # it (see :data:`CUP_CONTACT_WINDOW_LEAD_S`).
        if (t_abs_s + float(skill.lead_s)
                >= float(clamped.t_land_abs_s) - CUP_CONTACT_WINDOW_LEAD_S):
            return clamp_lines + self._resend_declined(
                t_abs_s, idx, 'CONTACT-WINDOW', moved_mm, moved_s)
        new_terminal = self._catch_terminal(idx, skill, landing)
        clamp_lines += self._pending_notes
        self._pending_notes = []
        res = self.installer(skill.kind, new_terminal, t_abs_s,
                             ball_id=skill.ball_id)
        self.results.append((idx, skill, res))
        self._resend_counts[idx] = self._resend_counts.get(idx, 0) + 1
        if not res.accepted:
            # The committed catch stands — a refused RE-aim is strictly better
            # than no catch, so the attempt continues. And it is TERMINAL for
            # this catch (2026-09-20): a re-solve refused on a limit only gets
            # worse as the dive tightens toward touch-down — on 2026-09-18
            # 16:16, 18 of 21 timing-only re-sends were refused LIMIT_JERK,
            # most the SECOND refusal for the same catch, each a 40-130 ms
            # solve on the orchestrator thread. The next fitted estimate is
            # not asked again.
            self._live_catch = None
            self._note_reaim_refused(idx, res.code)
            return clamp_lines + [
                    '%.3f RESEND-REFUSED %s at skill %d: the fit moved the '
                    'landing %.1f mm / %+.3f s — %s; no further re-aim for '
                    'this catch' % (t_abs_s, res.code, idx, moved_mm, moved_s,
                                    res.message)]
        self._live_catch = (idx, new_terminal, t_abs_s)
        self._note_catch_aim(skill.ball_id, idx,
                             _event_abs_s(CATCH, new_terminal), reaim=True)
        return clamp_lines + [
                '%.3f RESEND skill %d: the fit moved the landing %.1f mm / '
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


# ─────────────────────────────────────────────────────────────────────────────
# R5: the Ball-Butler-fed columns start — pre-throw feasibility
# ─────────────────────────────────────────────────────────────────────────────

def _rest_state_at(cup_mm) -> uc.CycleState:
    """A plan seed at rest with the cup at ``cup_mm`` — identical
    construction to ``tools/admissible_sweep.py::_rest_state``,
    ``tests/motion/test_skills_executor.py::_rest_state`` and
    ``tests/motion/test_skills_segments.py::_rest_state`` (one physical
    fact, several private copies by standing convention rather than a
    shared helper module, since ``motion/`` has no "test/tool utilities"
    module to hang one off without a new dependency edge nothing else
    needs)."""
    rcfg = cr.RealizeConfig()
    cup_mm = np.asarray(cup_mm, dtype=float).reshape(3)
    slider_mm = float(cup_mm[2]) - rcfg.cup_z_base_mm
    rev = (slider_mm - rcfg.slider_rev_zero_mm) / 1000.0 * cr.HAND_REV_PER_M
    pose = np.array([cup_mm[0], cup_mm[1], rcfg.active_z_mm, 0.0, 0.0, 0.0])
    return uc.CycleState.at_rest(pose, rev, rcfg)


def _never_install(*_args, **_kwargs):
    """The installer a throwaway :class:`SkillExecutor` is built with for
    :func:`plan_columns_first_cycle` — never called, because the check only
    ever reaches into the executor's terminal builders
    (:meth:`SkillExecutor._throw_terminal` / :meth:`SkillExecutor.
    _catch_terminal`), never :meth:`SkillExecutor.tick` or :meth:`SkillExecutor.
    _dispatch`. Raises loudly rather than silently returning a fake
    ``InstallResult`` if that assumption is ever wrong."""
    raise AssertionError(
        'plan_columns_first_cycle: the throwaway SkillExecutor dispatched a '
        'real install -- it must only ever use the terminal builders')


def plan_columns_first_cycle(schedule: Schedule, limits, *,
                             geom=None,
                             cfg: Optional[SegmentConfig] = None
                             ) -> Optional[str]:
    """Pre-throw feasibility check for a Ball-Butler-fed columns start (R5,
    2026-09-30, ``plans/active/two-ball-skill-stack.md`` § R5 "carried from
    R4" / the R5 day-1 announcement gap).

    ``schedule`` is a ``schedule.compile_columns(pattern, feed=...)`` result
    (:func:`~jugglebot.motion.skills.schedule.compile_columns`'s ``feed``
    branch — its very first two skills are ball A's launch THROW from rest
    at its own site, and the CATCH-with-throw of ball B carrying
    ``landing_prior=feed``). This plans BOTH, offline and exactly as the
    real :class:`SkillExecutor` would build and dispatch them, and returns
    ``None`` when both plan or a refusal string — the limit code and the
    measured numbers, from whichever fails first — when either does not.
    The LIVE terminal additionally crosses the ``InstallSegment`` wire
    (``skill_node._installer`` -> ``trajectory_node.
    _segment_terminal_from_request``), which this offline, in-process check
    does not exercise — ``tests/ros/test_install_segment.py``'s wire-map
    contract test is what guards that every terminal field actually reaches
    that wire (2026-10-02: ``receive_tilt`` did not, and this function's own
    PASS was the reason that sitting's refusal was not caught sooner).

    **Why this check exists.** ``compile_columns`` accepting a schedule only
    means the SCHEDULE's timing is consistent (a positive transit, no
    overlapping windows, the field-level ``ValueError``s on ``Skill``/
    ``CycleGoals``) — nothing about that compile actually solves the QP the
    schedule's first two skills need, and the feed catch has a real,
    measured cliff (scratchpad ``level_pinned_feed_report.md`` § 3, run
    2026-09-30, mechanism reconfirmed against THIS function 2026-09-30):
    with the touch-down attitude pinned level (``Skill.receive_tilt`` — the
    R5 fix that makes a BB arrival 11.9° off vertical fit the transit +
    dwell window AT ALL, replacing a refusal at 122% of the leg-velocity
    cap with a plan that has ~5-30 points of margin on every channel), a
    landing displaced far enough from the nominal feed point IN THE
    DIRECTION THAT EXTENDS the platform's required transit (away from ball
    A's own release site) refuses ``LIMIT_VEL`` — every other direction
    (toward ball A's site, or lateral) stays comfortably feasible.
    **The report's own scan found the cliff at +20 mm (hand-built
    terminals, release pinned to the site); reprobed THROUGH this
    function (``probe_feed_check.py``, scratchpad, 2026-09-30) the cliff
    sits further out, +30 mm (LIMIT_VEL at 108% of cap; 0 through +20 mm
    all plan clean) — because :meth:`SkillExecutor._catch_terminal`'s
    2026-09-28 "the release rides the CAUGHT xy, not the site" rule (its
    own docstring) removes a return-to-centre move the +20 mm case no
    longer has to make within the dwell.** Either way the margin at the
    nominal site is thin enough, and Ball Butler's real aim scatter
    (+/-8 mm after its volley re-fit, up to 27 mm biased before it) is
    close enough to the (now +30 mm) cliff, that whether a real feed catch
    clears the level program depends on which side of the nominal site the
    ball actually lands — calling this BEFORE the compiled schedule's
    executor is swapped in (``skill_node.py``'s ``_install_columns_
    schedule``) turns a doomed cycle into a same-tick refusal instead of
    stranding ball A in the air under a feed catch that cannot be planned.

    **How the terminals are built.** Through a THROWAWAY
    :class:`SkillExecutor` bound to ``schedule`` with a never-called
    installer (:func:`_never_install`) and no ``learner``/``boxes``/
    ``lateral_authority_m`` — at a first dispatch with none of those
    configured, :meth:`SkillExecutor._command_u` is the identity prior
    (``u = y_d`` exactly), which is what a live dispatch ALSO uses for
    every release until a learner is wired in (``SkillExecutor.__init__``'s
    own docstring, R2's behaviour) — so this reuses
    :meth:`SkillExecutor._throw_terminal` / :meth:`SkillExecutor.
    _catch_terminal` (the caught-xy carried release, the ``receive_tilt``
    pass-through) rather than re-deriving a terminal shape here that could
    silently drift from what a real dispatch builds. Both terminal builders
    hand back the ABSOLUTE (wall-clock) event time (``Skill.t_abs_s`` /
    ``ThenThrow.t_release_abs_s`` — the schedule's own clock, ``_event_abs_s``'s
    docstring), which :func:`sg.plan_segment` may not consume directly (its
    terminals speak the SEGMENT clock, seed-to-event); :func:`install_segment`
    is the one place that gap is closed today (:func:`_terminal_on_segment_clock`
    at its real fresh-origin/splice ``t_origin``), so this offline check closes
    it the same way, at the ``t_origin`` a FRESH plan of each skill implies —
    ``skill.t_abs_s - skill.window_s`` (no dispatch jitter, no splice: a fresh
    plan of ball A's own launch or the feed catch alone, exactly what
    ``schedule.compile_columns`` sized ``window_s`` to fit; the R5 probe this
    unit was built against, ``probe_bbfed_columns.py``, hand-builds this same
    relative time via ``pattern.launch_s`` / ``TAU`` / ``TAU + DWELL``, and
    MEASURED that skipping this conversion changes the verdict: the raw
    absolute time treats a THROW as a several-SECOND launch and its release
    STATE feeds the feed catch a materially different seed — undisplaced went
    from feasible to a false ``LIMIT_VEL`` refusal 3.9% over cap). Ball A's
    THROW seeds from rest at its own site (:func:`_rest_state_at`); the feed
    CATCH seeds from the THROW segment's own :attr:`Segment.release_state` —
    the one release seed a chained plan may use (the field's own docstring) —
    and is aimed at ``feed_skill.landing_prior`` directly (exactly what
    :meth:`SkillExecutor._predicted_landing` returns for a
    ``landing_prior``-carrying catch before any tracker fit exists, i.e.
    at this announcement's first dispatch).

    ``geom``/``cfg`` default to a fresh ``StewartGeometry()`` /
    ``SegmentConfig()`` when not given — the production call site
    (``skill_node.py``) has neither lying around, since every other segment
    on this node is solved by ``trajectory_node`` over the
    ``InstallSegment`` service, not locally; a test passes its own
    module-scoped fixtures instead of rebuilding the geometry table per
    case. Measured wall time (scratchpad, 2026-09-30, this module's R5
    operating point, ``OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1``): see
    ``handoff_feed_check.md`` — well inside the ~2.7 s an announcement
    arrives ahead of ``t0``.
    """
    geom = StewartGeometry() if geom is None else geom
    cfg = SegmentConfig() if cfg is None else cfg

    throw_idx, throw_skill = next(
        ((i, s) for i, s in enumerate(schedule.skills) if s.kind == THROW),
        (None, None))
    feed_idx, feed_skill = next(
        ((i, s) for i, s in enumerate(schedule.skills)
         if s.landing_prior is not None), (None, None))
    if throw_skill is None or feed_skill is None:
        raise ValueError(
            'plan_columns_first_cycle needs a compile_columns(feed=...) '
            'schedule -- a THROW skill and a CATCH carrying landing_prior '
            'are both required, got kinds %r'
            % ([s.kind for s in schedule.skills],))

    dry = SkillExecutor(schedule, _never_install)

    seed = _rest_state_at(throw_skill.site.rest_site_mm())
    throw_terminal = dry._throw_terminal(throw_idx, throw_skill)
    throw_origin_s = float(throw_skill.t_abs_s) - float(throw_skill.window_s)
    throw_terminal = _terminal_on_segment_clock(THROW, throw_terminal,
                                                throw_origin_s)
    try:
        seg1 = sg.plan_segment(THROW, seed, throw_terminal, cfg, limits, geom)
    except uc.CycleInfeasible as exc:
        return ('%s THROW (site %s, from rest) refused: %s'
                % (ball_label(throw_skill.ball_id), throw_skill.site.name,
                   exc))

    lp = feed_skill.landing_prior
    landing = Landing(pos_mm=lp.pos_mm, vel_mm_s=lp.vel_mm_s,
                      t_land_abs_s=lp.t_land_abs_s)
    catch_terminal = dry._catch_terminal(feed_idx, feed_skill, landing)
    catch_origin_s = float(feed_skill.t_abs_s) - float(feed_skill.window_s)
    catch_terminal = _terminal_on_segment_clock(CATCH, catch_terminal,
                                                catch_origin_s)
    try:
        sg.plan_segment(CATCH, seg1.release_state, catch_terminal, cfg,
                        limits, geom)
    except uc.CycleInfeasible as exc:
        return ('feed CATCH (site %s, %s) refused: %s'
                % (feed_skill.site.name, ball_label(feed_skill.ball_id), exc))
    return None
