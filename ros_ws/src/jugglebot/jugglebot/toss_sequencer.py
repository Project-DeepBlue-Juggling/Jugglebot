"""Pure-Python FSM for the Jugglebot self-toss sequence (Toss.action, Tiers 8a/8b).

The coordinator node (``reload_coordinator_node``, the merged ball-ops node) is a
thin ROS wrapper around this FSM: it feeds observations in (control mode, streaming,
mocap/hand freshness, ball possession, tracker state) and executes the actions the
FSM asks for (``go_to_pose``, the PREPARE bundle, the self-``ThrowAnnouncement``
publish, the ``SetHandTrajCmd`` throw dispatch, RECENTER, SAFE_ABORT). The FSM
itself imports NO ROS — every transition is a pure function of ``(now,
observations, discrete events)`` so the whole sequence is unit-testable without a
running graph.

The TOSS action OWNS the platform + hand for its duration and runs from the active
streaming mode (``TRAJECTORY``, armed + streaming a hold) — leaving the mode
mid-sequence is the documented abort, exactly the reload doctrine. The ball is
sourced by a prior Reload (operator sequence Reload → Toss → Toss …); a toss with
an empty cup is ``REJECTED_NO_BALL``, and one whose ball sensor cannot answer is
``REJECTED_BALL_UNKNOWN`` (C-POSSESS-1 § 3.3 — a dead sensor refuses, it does not
pass).

Design (plan § Choreography; deliberate deviations from ``reload_sequencer``
are called out inline — each exists because the toss THROWS from its own hand
where the reload receives from BB):

1. **CHECKING** — loud precondition rejects, strictest first: the tier gate
   (config ``'8a'``/``'8b'`` — anything else ``REJECTED_TIER``), the static goal
   parameters (delay floor, flight-time VALIDITY, the DERIVED throw envelope —
   ``REJECTED_THROW_ENVELOPE`` when the commanded release speed leaves the
   feasibility set that ``motion/trajectory/throw_envelope`` computes from the
   configured end stop, torque/current, regen and timing limits; the refusal
   names the binding bound and quotes the computed envelope, and since
   2026-08-18 it — not a hand-picked ``[0.55, 1.10]`` band — is what bounds
   throw height, the Tier-8b displaced-throw gates —
   ``REJECTED_POSE_UNKNOWN`` when the platform's live commanded pose (⇒ the throw
   site A) could not be read, ``REJECTED_DISPLACEMENT`` for |B−A| past the cap or
   past the closed-form quintic reach bound over the flight,
   ``REJECTED_TILT_CLAMP`` for an aim past
   the tilt ceiling — event-vel band, workspace pre-check on B and, for 8b, on
   the throw site A), the
   control mode, then the live observations: mocap fresh, trajectory streaming,
   a gravity-levelling correction loaded in the node that applies it
   (``REJECTED_NOT_LEVELLED`` — un-levelled the launch is 0.78° off gravity,
   which is 43 mm of lateral drift over a 0.8 s flight against a ~35 mm cup:
   the catch is geometrically impossible, so refuse before the ball is in the
   air. Observed on ``trajectory/status.gravity_correction_loaded``, NOT on the
   Teensy-persisted ``RobotState.levelling_complete``, which survives the
   relaunch that empties the correction),
   hand telemetry fresh (``REJECTED_HAND_STALE`` — a dead hand link blinds
   release verification), hand at the bottom park band
   (``REJECTED_HAND_NOT_PARKED`` — a kind-0 stroke commands ABSOLUTE positions
   from 0 rev, so a throw dispatched off-band is a physical hazard), ball
   evidence from the LIVE hand ball sensor — ``REJECTED_NO_BALL`` on a
   valid-empty cup, ``REJECTED_BALL_UNKNOWN`` on a sensor that cannot answer
   (two codes because they send the operator to two different subsystems) —
   and no live tracker expectation
   already destined for us (``REJECTED_TRACK_ACTIVE`` — a phantom
   destination='jugglebot' track would correlate against OUR announcement).
2. **POSITIONING** — profiled ``go_to_pose`` to the nominated catch pose (Tier
   8a: throw site = catch site). Arrival is time-based (``planned_duration_s`` +
   settle pad); a CONFIG-KEYED mocap cross-check (node parameter naming the
   platform's QTM rigid body; DISABLED by default — no platform body name or
   frame has ever been validated live) can additionally corroborate against
   the nominated pose before any arming: the ``arm_catch`` raise C2-stops any
   in-flight move, so arming mid-move (or after a silently-refused move —
   disarmed wire, guard latch) leaves the platform SHORT OF A and the throw
   then fires from a site the aim was not solved for — a mis-aimed ball, and
   for 8b a pre-tilt that never completed. (Before contract C-REACH-1 this
   paragraph also carried the reach-envelope centre, which the raise captured
   from the commanded pose; the toss now DECLARES that centre — see
   ``ros_ws/docs/catch_reach_envelope.md`` — so the envelope is no longer the
   casualty of arming mid-move, but the aim still is.) When
   the check is enabled and never corroborates within the deadline ⇒
   ``ABORTED_POSITION_FAILED``; when disabled the node feeds
   ``platform_at_target`` True so arrival rests on the go_to_pose accept +
   timed wait (+ the ``WIRE_DISARMED`` response mapping).
3. **PREPARING** — the node raises ``catch/prime_hold`` alone on the
   ``ACTION_PREPARE_CATCH`` tick and runs the PREPARE bundle (soft catch
   gains, ``arm_catch`` latch raise + confirm, vel_scale, prime-dispatched
   stamp, ``catch/armed``, phantom-track snapshot) one tick LATER — the hold
   and ``catch/armed`` have no cross-topic ordering guarantee, and a hold
   landing in the same catch_coordinator wait-set cycle as the armed edge
   could lose the intra-cycle ordering race and let the armed-edge auto-prime
   ascend with the seated ball. Then — armed confirmed → an EXPLICIT ≥1-tick
   gap → the self-``ThrowAnnouncement``. That gap is load-bearing for the same
   cross-topic reason: ``catch_coordinator`` drops announcement pre-tilts that
   arrive unarmed. Reload never needed this — BB's announcement arrives
   seconds after arming; ours would race it.

   **The bundle NARROWED on 2026-08-27 for a chained session (S6,
   ``toss_session``'s module docstring).** In a ``TossContinuous`` run the
   holds, the ``catch/reach_center`` declaration, the soft gains, the
   ``arm_catch`` raise and the ``vel_scale`` relay are SESSION-scoped — one
   raise and one lower per contiguous run — so a chained cycle's PREPARE is
   three topic publishes and a snapshot, with **no service round trip at all**.
   The FSM is unchanged by that: it still emits ``ACTION_PREPARE_CATCH``, still
   defers one tick, still waits for ``note_prepare_result`` and still refuses to
   announce until it says yes. What moved is what the node does inside the
   action, and the ordering the FSM guarantees only got stronger — the armed
   →announce gap is now satisfied by a raise that happened seconds earlier
   rather than by one tick. The single ``Toss`` is untouched and runs the full
   per-cycle bundle. A cycle whose nominated B has drifted out of the session's
   declared reach envelope refuses here, ``REJECTED_REACH_CENTER_DRIFT``.
4. **THROWING** — entered only after a hand-parked RE-VERIFY (positioning +
   prepare took seconds; a kind-0 stroke commands ABSOLUTE positions from
   0 rev, so a hand that drifted/was moved off the bottom band since CHECKING
   is a physical dispatch hazard ⇒ ``ABORTED_HAND_NOT_PARKED`` via
   SAFE_ABORT). Then ``ACTION_DISPATCH_THROW`` exactly once, ever. The dispatch is
   tri-state (``ok`` / ``ambiguous`` / ``rejected``): a definitive bridge-side
   validation reject (no CAN frame exists) SAFE_ABORTs with the ball still
   seated; ``ok`` and ``ambiguous`` are treated identically — the hand ack lies
   ~59% both ways, so the stroke's outcome is read back from release EVIDENCE
   (hand-telemetry stroke signature OR tracker CONFIRMED — a disjunction of two
   independent channels), never from the ack. A re-dispatch is forbidden in
   every branch: it would re-arm with a new wall_time if the first frame was
   lost, REPLACE a live stroke if the ack lied, and post-release it would
   clobber the armed catch stroke on the Teensy's last-writer-wins queue.
   No evidence by ``t_release + grace`` ⇒ ``ABORTED_NO_RELEASE`` (residual
   risk: ambiguous ack ∧ blind telemetry ∧ mocap miss ⇒ a stroke that DID fire
   reads as NO_RELEASE and SAFE_ABORT retracts under an airborne ball —
   accepted for Phase 1, same class as reload's NO_ANNOUNCEMENT path).
5. **BALL_IN_FLIGHT / CATCHING** — the EXISTING catch path (correlation →
   catch_coordinator → ``catch/dynamic_target`` → ``build_catch`` + hand fire)
   runs on its own; the platform holds open-loop at the pre-positioned pose
   (Tier 8a: for the whole flight; Tier 8b: at the pre-tilt pose at A until the
   scheduled release, then the ONE deferred A→B reach below spans the flight). A
   standing post-release infeasible catch target is LATCHED but does NOT
   terminate mid-flight (retract-into-incoming-ball hazard — hardware evidence
   2026-07-23, twice); the outcome resolves at settle.

   **Tier 8b's deferred reach (``ACTION_REACH_CATCH``, Phase 4):** the FSM emits
   it exactly once on the first tick with ``now >= t_release`` — TIME-triggered,
   NOT evidence-triggered: release evidence can lag up to ``TOSS_RELEASE_GRACE_S``
   (0.5 s), which would eat most of a 0.8 s flight and push the reach lead toward
   trajectory_node's 0.25 s min_timed_lead floor. The node then publishes the ONE
   announcement-derived ``catch/dynamic_target`` (B, arrival = the announced
   landing ⇒ lead = the flight time by construction). If the stroke silently
   never fired, the platform translates A→B carrying the seated ball — the same
   benign-accel class as the SAFE_ABORT retract — and ``ABORTED_NO_RELEASE``
   still cleans up at t_release + grace. The stock catch_coordinator
   announcement pre-tilt CANNOT serve 8b: it arrives ≥1 s before release with an
   arrival clamped to ~now + 1 s, so the A→B translate (and the un-tilt to the
   receive tilt) would COMPLETE before the ball is released — aim destroyed, a
   moving platform under a seated ball mid-windup. The node suppresses it via
   ``catch/pretilt_hold`` for the goal's duration (8a is motion-free under the
   stock path and keeps it unchanged).
6. **SETTLING** — the BALL-IN-CUP SENSOR observing OUR ball arrive within the
   confirm window past the scheduled landing ⇒ ``CAUGHT`` (``ACTION_STAY`` by
   default, ``ACTION_RECENTER`` iff ``stay_at_pose_on_caught`` is False — see
   ``_terminal_action``); otherwise ``MISSED_SENSOR_BLIND`` (the cup could not
   look — a machine fault, named rather than laundered into a miss),
   ``MISSED_INFEASIBLE_<code>`` (only when NO catch target was ever accepted)
   or ``MISSED``.
   Read "tracker ``CAUGHT``" here until 2026-08-26, when owner decision D1 made
   possession the cup's alone — the tracker is no longer consulted for it on
   either FSM (``ros_ws/docs/ball_possession_contract.md``, C-POSSESS-1.D).

ORDERING PRINCIPLE (transposed from reload): every Jugglebot-side arming action
(platform positioning, catch latch, announcement) happens BEFORE the throw is
dispatched — the throw is the LAST commitment of the sequence. Once the stroke's
wall_time is armed in the Teensy there is no un-arm opcode short of replacing
the queue entry, so an arming failure must abort while there is still nothing
to abort on the hand side. The one deliberate un-arm that DOES exist: a
pre-release SAFE_ABORT's kind-3 retract replaces any armed kind-0 throw stroke
on the last-writer-wins queue — used on purpose, and the reason a pre-release
abort is safe even after an ambiguous dispatch ack.

Async events arrive via ``note_*`` methods, gated on COMMITMENT FLAGS, never on
the phase (the reload lesson): ``note_catch_feasibility`` and
``note_announcement`` gate on the PREPARE having been DISPATCHED, not on the
throw — for a toss the pre-tilt target (and its acceptance) is triggered by OUR
announcement, which precedes the throw dispatch; reload's ``_throw_sent`` gate
transposed literally would drop the pre-tilt accept deterministically and mint
false ``MISSED_INFEASIBLE`` verdicts on every real toss.

Terminal actions (executed by the node, exactly once — the finished-replay path
returns ``ACTION_NONE``):
  - ``ACTION_STAY`` — on CAUGHT, the default since 2026-07-29
    (``stay_at_pose_on_caught``): lower the latch, clear ``catch/armed``,
    release the holds, and issue **no go_home** — the emitter's terminal hold
    keeps the platform at the catch pose. That is what lets a session CHAIN:
    the next Toss reads its throw site A from this live pose, so A → B → C
    needs no operator repositioning between throws. The hand keeps the caught
    ball (no retract), so the next Toss is immediately serviceable.
  - ``ACTION_RECENTER`` — on CAUGHT with ``stay_at_pose_on_caught=False``
    (the pre-2026-07-29 behaviour): lower the latch + go_home.
  - ``ACTION_SAFE_ABORT`` — on ANY not-caught terminal once the platform moved
    (``_positioned``) or the latch raise was dispatched (``_prepare_dispatched``):
    armed-off first, retract ladder, latch-off, go_home. With a seated ball the
    retract carries the ball down at ~3.16 m/s² ≪ g — it stays seated, and
    possession survives, so an aborted toss is retryable without a Reload.
  - A reject before anything moved or armed emits ``ACTION_NONE``.

**S6 (2026-08-27) changes what the NODE does inside those three terminals in a
chained session, never which one the FSM emits.** The latch lower and the two
hold releases are session-scoped, so a chained ``ACTION_STAY`` publishes
``catch/armed`` False and stops — leaving the latch standing for the next cycle,
which is exactly what makes the chain a chain. ``ACTION_RECENTER`` and
``ACTION_SAFE_ABORT`` dispatch ``go_home``, so they DRAIN and disarm first (S7):
no ``go_home`` is ever installed under a catch the machine still thinks is live.
The single ``Toss`` runs all three verbatim as before.

Cancellation is node-level and PER-PHASE (plan § Choreography 7, a deliberate
deviation from reload's immediate-cancel): honoured immediately through
PREPARING and early THROWING (the retract clears the armed stroke; nothing is
airborne), DEFERRED to the FSM's own terminal from ``TOSS_CANCEL_CUTOFF_S``
before release onward — aborting a catch mid-flight drops a ball on the robot.

The FSM never actuates the robot itself and never bypasses the feasibility gate —
every platform motion goes through ``trajectory_node`` / ``planner`` /
``feasibility.validate``, and the throw goes through the teensy bridge's own
validation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

# Pure-python sibling module (no ROS): the ball-evidence vocabulary lives with the
# possession contract, so the FSM's reject codes and the node's live sensor read
# can never drift into two spellings of the same state.
from jugglebot.ball_possession import ARRIVAL_BAND_MAX_S, EVIDENCE_UNKNOWN

# The shared refusal-detail vocabulary (pure text formatting over numbers this
# module already holds).  Imported rather than restated so the three FSMs and
# their tests punctuate a bound the same way; the module's other export,
# `base_outcome`, is what CONSUMERS of these strings use to recover the bare
# code, and it is why enriching a refusal here cannot disarm a guard there.
from jugglebot.outcome_detail import bound_msg, range_msg

# The derived throw-admission envelope (contract C-HAND-3,
# ros_ws/docs/hand_throw_envelope.md).  This is the ONE import in this module
# that is NOT mirrored by a local literal + drift-guard pin, and that asymmetry
# is deliberate: the local-copy pattern below exists so a nominal PLANNING value
# stays readable standalone, but a SAFETY envelope's whole point is that exactly
# one expression of it exists.  A pinned copy of `10.8 rev minus the coast` here
# would be a second place for the end stop to be wrong — which is the failure
# this contract was written after (the stop read 11.1 rev, 0.3 past metal, for
# the life of the constant).  It is still ROS-free: throw_envelope imports only
# math, the generated config, and motion.trajectory (numpy, no rclpy).
from jugglebot.motion.trajectory import throw_envelope

# The firmware stroke model, for the SAME reason as throw_envelope above: since
# 2026-08-22 the event-delay floor is DERIVED from the stroke geometry rather than
# declared as a constant, and a pinned local copy of "prelude + gap + windup"
# would be a second place for the Teensy's :642 budget check to be wrong.  Also
# ROS-free (math + the generated config).
from jugglebot.motion.trajectory import hand_stroke

# ── Feedback phases (Toss.action feedback.phase — LOCKED strings) ──────────────
PHASE_CHECKING = 'CHECKING'
PHASE_POSITIONING = 'POSITIONING'
PHASE_PREPARING = 'PREPARING'
PHASE_STAGED = 'STAGED'              # B4: the preamble is complete and the cycle
                                     #   is waiting for its COMMIT instant. It
                                     #   owns NOTHING that moves — S1' restricts a
                                     #   staged cycle's emittable action set to
                                     #   {NONE, POSITION_PLATFORM(skip),
                                     #   PREPARE_CATCH}, and it reaches STAGED with
                                     #   all three already spent.
PHASE_COMMITTING = 'COMMITTING'      # B4: THE arm point — the single tick that
                                     #   re-reads the evidence and, in the same
                                     #   tick and in this order, publishes the
                                     #   announcement and dispatches the throw.
                                     #   Never entered on the serial path.
PHASE_THROWING = 'THROWING'
PHASE_BALL_IN_FLIGHT = 'BALL_IN_FLIGHT'
PHASE_CATCHING = 'CATCHING'
PHASE_SETTLING = 'SETTLING'

# ── Actions the node executes on the FSM's behalf ──────────────────────────────
ACTION_NONE = 'none'
ACTION_POSITION_PLATFORM = 'position_platform'  # go_to_pose to the catch pose. Once,
                                                #   with ONE carve-out: a BUSY refusal
                                                #   is re-emitted on the
                                                #   TOSS_POSITION_BUSY_REPOLL_S grid
                                                #   for at most
                                                #   TOSS_POSITION_BUSY_PATIENCE_S
                                                #   (_absorb_position_busy). Nothing has
                                                #   moved and nothing is armed while
                                                #   that runs
ACTION_PREPARE_CATCH = 'prepare_catch'          # node: prime-hold raise on THIS tick,
                                                #   then (one tick later) gains + latch
                                                #   raise + vel_scale + prime-dispatched
                                                #   stamp + armed + snapshot — the hold
                                                #   must land in an earlier CCN wait-set
                                                #   cycle than the armed edge
ACTION_ANNOUNCE = 'announce'                    # publish the self-ThrowAnnouncement
                                                #   (≥1 tick AFTER the armed confirm)
ACTION_DISPATCH_THROW = 'dispatch_throw'        # SetHandTrajCmd traj_type=0 (ONCE, ever)
ACTION_REACH_CATCH = 'reach_catch'              # Tier 8b only: publish the ONE deferred
                                                #   A→B catch/dynamic_target (arrival =
                                                #   the announced landing). Emitted
                                                #   exactly once on the first tick with
                                                #   now >= t_release — TIME-triggered,
                                                #   never evidence-triggered (evidence
                                                #   can lag 0.5 s and eat the reach lead)
ACTION_RECENTER = 'recenter'                    # lower latch + go_home (hand keeps ball)
ACTION_STAY = 'stay'                            # CAUGHT + stay_at_pose_on_caught:
                                                #   lower latch + catch/armed False +
                                                #   release the holds, and NO go_home —
                                                #   the emitter's terminal hold keeps the
                                                #   platform at the catch pose so the NEXT
                                                #   toss throws from it (session chaining
                                                #   A → B → C). Strictly the RECENTER
                                                #   ladder minus go_home: no new mechanism,
                                                #   no new commanded motion.
ACTION_SAFE_ABORT = 'safe_abort'                # armed-off → retract → latch-off → go_home

# Throw-dispatch classifications fed back via note_throw_dispatch. REJECTED is the
# definitive no-arm channel (service unavailable, or the bridge's own validation
# raised BEFORE any CAN frame existed); OK and AMBIGUOUS are treated identically
# downstream — the ack lies ~59% both ways, so only release EVIDENCE advances.
THROW_DISPATCH_OK = 'ok'
THROW_DISPATCH_AMBIGUOUS = 'ambiguous'
THROW_DISPATCH_REJECTED = 'rejected'

# The control mode TOSS runs within: ACTIVE + streaming a hold, in TRAJECTORY — same
# doctrine as RELOAD_CONTROL_MODE. Leaving it mid-sequence is the documented abort.
TOSS_CONTROL_MODE = 'TRAJECTORY'

TIER_8A = '8a'                       # co-located vertical toss (Phase 1). The config
                                     # key (jugglebot_operational.toss_tier →
                                     # JB_OP_TOSS_TIER) selects the tier, the goal
                                     # cannot.
TIER_8B = '8b'                       # tilt-aimed displaced throw→catch (Phase 4):
                                     # pre-tilt at the throw site A = the platform's
                                     # LIVE commanded xy (Phase E; was a config site
                                     # until 2026-07-29), launch aimed at the
                                     # displaced B, deferred A→B reach at t_release.
                                     # Any tier outside {8a, 8b} is REJECTED_TIER.
                                     # 8b SUBSUMES 8a: a goal whose B equals the live
                                     # pose has zero displacement, so the aim is
                                     # exactly level and compute_release_state_tilted
                                     # returns compute_release_state BITWISE — the
                                     # vertical toss, from wherever the platform
                                     # already is.

# ── Defaults / floors ──────────────────────────────────────────────────────────
DEFAULT_TOSS_THROW_DELAY_S = 5.0     # 0 => this. Budget: CHECK ~0.1 + POSITION ≤~2 +
                                     # PREPARE ≤~0.5 + event_delay ≥1.0 + slack. NOT
                                     # reload's 3.0: the toss spends its delay on
                                     # positioning, the reload on the BB countdown.
TOSS_DISPATCH_DEBOUNCE_S = 0.10      # GOAL-STORM debounce, NOT a readiness floor.
                                     # ────────────────────────────────────────────
                                     # This name replaced MIN_TOSS_THROW_DELAY_S =
                                     # 3.5 s on 2026-08-22 (operator decision 3 of
                                     # the ILC-primary fold-in), and the rename is
                                     # the point: the old constant was a GENERIC FIT
                                     # — "CHECK(1 tick) + POSITION(≤~2 s) +
                                     # PREPARE(4 ticks) + event_delay(≥1.0) plus
                                     # slack" — and a generic fit is a policy number
                                     # wearing a physics costume. It bounded nothing
                                     # the machine can actually observe, it made the
                                     # session's advertised 4.10 s dwell floor an
                                     # artefact of itself, and at the cadence rungs
                                     # (census A1) it was 7x the whole physical
                                     # turnaround.
                                     #
                                     # What guards the same failures NOW:
                                     #   * READINESS — the derived, state-based
                                     #     CHECKING interlocks, each of which asks
                                     #     the machine rather than the clock:
                                     #     hand_fresh + hand_parked (the kind-0
                                     #     stroke commands ABSOLUTE positions from
                                     #     0 rev), ball_seated (C-POSSESS-1),
                                     #     track_active, mocap/status freshness,
                                     #     platform_levelled;
                                     #   * the ARITHMETIC that used to hide inside
                                     #     the 3.5 s — hand_stroke.
                                     #     min_throw_event_delay_s(v_throw),
                                     #     enforced at runtime by
                                     #     ABORTED_CANT_MAKE_RELEASE against the
                                     #     REAL remaining lead (census B5,
                                     #     unchanged and still the truth), and
                                     #     checked at CHECKING (loud + early) as
                                     #     min_throw_delay_for_release_s — that
                                     #     budget PLUS the pre-dispatch sequence
                                     #     the guard has already spent by the time
                                     #     it runs (2026-08-23; charging the
                                     #     dispatch budget alone made the accept
                                     #     gate loose by 0.160-0.520 s and shipped
                                     #     three cadence rungs that abort every
                                     #     cycle);
                                     #   * the HAND GEOMETRY — toss_session's
                                     #     required_dwell_s now carries
                                     #     hand_stroke.min_turnaround_dwell_s, so a
                                     #     cadence the stroke cannot physically make
                                     #     is refused at goal-accept instead of
                                     #     clobbering a live stroke (C-HAND-1).
                                     #
                                     # What is LEFT for this constant: a dispatch
                                     # debounce. A throw_delay at or below ~0 lets a
                                     # goal (or a scripted storm of them) demand a
                                     # release in the same tick it was accepted, so
                                     # CHECKING would run POSITION/PREPARE/ANNOUNCE
                                     # against an already-expired t_release and burn
                                     # a cycle's per-goal state per goal. 0.10 s is
                                     # two FSM ticks at the post-B3 _TICK_S of 0.02
                                     # — enough for the sequence to exist at all,
                                     # deliberately NOT enough to imply readiness.
                                     # It also still catches the sign typo the FSM's
                                     # __post_init__ preserves (a negative delay).
MIN_THROW_EVENT_DELAY_S = 0.0        # 0 ⇒ DERIVE from the resolved event_vel via
                                     # hand_stroke.min_throw_event_delay_s. A
                                     # positive value is an explicit OVERRIDE (tests
                                     # and the standalone harnesses).
                                     #
                                     # It was a hand-written 1.0 s from 2026-07-25
                                     # until 2026-08-22, sized off the WORST prelude
                                     # the firmware can build — a full-stroke smooth
                                     # move from the top, 0.758 s. That prelude is
                                     # unreachable on the throw path and always was:
                                     # a kind-0 dispatch is admitted only with
                                     # hand_parked true (CHECKING, and re-checked at
                                     # THROWING entry), which caps the prelude at
                                     # smooth_move_duration_s(HAND_PARK_BAND_REV) =
                                     # 0.170 s. The old comment conceded exactly
                                     # that ("conservative twice over") and shipped
                                     # the unreachable number anyway. Derived, the
                                     # floor is 0.337 s at the C-HAND-3 band floor
                                     # and 0.281 s at the 0.80 s nominal — and it
                                     # MOVES WITH v_throw, which a constant cannot:
                                     # the windup term alone spans 0.064–0.147 s
                                     # across the admitted band, a 2.3x spread.
DEFAULT_TOSS_FLIGHT_TIME_S = 0.8     # 0 => this — the NO-CONFIG fallback only: the
                                     # coordinator resolves a zero goal field against
                                     # the generated JB_OP_TOSS_FLIGHT_TIME_DEFAULT_S
                                     # and passes the resolved value into the ctor;
                                     # this literal serves standalone/test use. The
                                     # drift-guard test pins the two equal.
# ── The flyable flight band — DERIVED, not declared (contract C-HAND-3) ───────
# Until 2026-08-18 these were two hand-picked literals: 0.55 "plan sweep floor"
# and 1.10 "plan sweep ceiling (≈ 5.4 m/s < 7.0 Teensy ceiling)".  The ceiling
# was sized from the hand's DECEL AUTHORITY alone and never against its end
# stop — it could not have been, because the stop was declared at 11.1 rev,
# 0.3 rev PAST metal, until the 2026-08-18 correction to 10.8.  Measured against
# the corrected stop, a throw at the old 1.10 s ceiling coasts to a modelled
# 12.17 rev — 1.37 rev (43 mm) past metal.
#
# These are now the REPORTED PROJECTION of the derived envelope onto Tier-8a
# co-located flight times, kept as module names because the sim sweep, the bench
# runbook and several tests read the band.  **They are not the gate**: the gate
# is throw_envelope.evaluate(flight_time, RESOLVED release speed), because a
# Tier-8b displaced throw is AIMED and releases faster than its flight time
# alone implies, so a T-only band would silently under-bound it.
FLIGHT_TIME_MIN_S = throw_envelope.MIN_FLIGHT_TIME_S
FLIGHT_TIME_MAX_S = throw_envelope.MAX_FLIGHT_TIME_S

TOSS_MIN_ANNOUNCE_LEAD_S = 2.5       # announce→landing lead below which the stock CCN
                                     # announcement pre-tilt degenerates toward
                                     # arrive-at-contact. WARN-only for BOTH tiers.
                                     # SUPERSESSION (Phase 4): this comment used to
                                     # promise "MUST harden to an abort for 8b" —
                                     # that promise was sized for CCN's announce-
                                     # driven pre-tilt traverse and is WITHDRAWN:
                                     # under the deferred-reach choreography the 8b
                                     # platform reach is published at t_release with
                                     # lead = flight time BY CONSTRUCTION, so this
                                     # constant no longer sizes any 8b platform
                                     # motion — and a literal hardening would brick
                                     # every floor-delay 8b toss (announce→landing
                                     # lead ≈ 1.8–2.3 s at the 3.5 s delay floor)
                                     # while protecting nothing. The load-bearing 8b
                                     # guards are the CHECKING displacement/clamp
                                     # gates below.

TOSS_POSITIONING_TIMEOUT_S = 6.0     # no go_to_pose response / arrival within this of
                                     # entering POSITIONING ⇒ abort. Sized: service
                                     # wait 2.0 + worst planned move ~2 s + pad.
TOSS_POSITION_SETTLE_PAD_S = 0.2     # added to planned_duration_s before consulting the
                                     # arrival cross-check (5 Hz status granularity;
                                     # terminal-hold entry jitter).
TOSS_POSITION_VERIFY_WINDOW_S = 1.0  # when the config-keyed mocap cross-check is
                                     # ENABLED, mocap must corroborate arrival within
                                     # this of the timed arrival (the positioning
                                     # deadline is extended to at least arrival +
                                     # this); silence ⇒ ABORTED_POSITION_FAILED — the
                                     # silent no-op class (disarmed wire, guard latch)
                                     # that a time-only wait would arm the catch
                                     # envelope on top of. Disabled (the default) the
                                     # node feeds platform_at_target True at every
                                     # tick, so the timed arrival stands alone.
TOSS_POSITION_MIN_MOVE_S = 0.2       # trajectory_node's `min_move_duration_s` — the
                                     # PLANNER FLOOR a real (non-skipped) go_to_pose
                                     # pays even for a zero-length move. Local copy of
                                     # hw.JB_TRAJ_MIN_MOVE_DURATION_S in this module's
                                     # established pattern (a nominal PLANNING value
                                     # stays readable standalone), pinned equal by
                                     # test_toss_sequencer.py's drift guard. Read here
                                     # by pre_dispatch_budget_s: a cycle that COMMANDS
                                     # its pre-positioning move cannot arrive sooner
                                     # than this + the settle pad, and that wait is
                                     # spent out of throw_delay.
_POSITION_BUSY_CODE = 'BUSY'         # trajectory_node's `_BUSY` — the ONE go_to_pose
                                     # refusal that clears on its own. Named rather
                                     # than inlined at the branch below because the
                                     # string is a WIRE value shared with a node this
                                     # module deliberately does not import.
TOSS_POSITION_BUSY_PATIENCE_S = 0.54 # a go_to_pose refused BUSY is RE-POLLED for this
                                     # long before it becomes REJECTED_POSITION(BUSY).
                                     # 0.50 + 0.040 = hw.JB_TRAJ_CATCH_SETTLE_HOLD_S +
                                     # NODE_LOOP_PERIOD_S, local literals in this
                                     # module's established pattern and pinned to the
                                     # generated config by test_toss_sequencer.py's
                                     # drift guard.
                                     #
                                     # THE ONE plan that can legitimately hold the
                                     # platform at the head of a chained cycle is the
                                     # PREVIOUS cycle's build_catch, whose post-arrival
                                     # tail is exactly the settle hold — so the bound is
                                     # that hold, anchored at the COMMITTED arrival (a
                                     # late reactive refinement re-anchors the arrival
                                     # and the bound moves with it; observed margin
                                     # 0.266 s in bag 2026-08-28_23-53-25). One loop
                                     # period covers the tick grid the FSM polls on.
                                     #
                                     # A BUSY that OUTLIVES this is a WEDGE — a stray
                                     # go_home, a SpaceMouse nudge, a guard latch — and
                                     # must still abort. That is why the bound is NOT
                                     # TOSS_POSITIONING_TIMEOUT_S (6.0) and NOT the
                                     # session stall watchdog: both are no-response
                                     # watchdogs an order of magnitude looser, and
                                     # waiting either out would launder a wedge into
                                     # ABORTED_POSITION_TIMEOUT (or a session stall)
                                     # instead of naming the refusal that happened.
TOSS_POSITION_BUSY_REPOLL_S = 0.10   # spacing between BUSY re-polls. go_to_pose is a
                                     # BLOCKING round trip inside the node loop (bag
                                     # tick_max 320-337 ms on these cycles), so
                                     # re-emitting it on every 20 ms tick would multiply
                                     # that cost across the wait it exists to absorb.
TOSS_RELEASE_GRACE_S = 0.5           # release evidence must appear within t_release +
                                     # this (mirrors reload's ANNOUNCEMENT_GRACE_S).
CATCH_CONFIRM_WINDOW_S = ARRIVAL_BAND_MAX_S
                                     # CAUGHT within this PAST the scheduled landing;
                                     # also the CATCHING phase-report threshold, and
                                     # (through toss_session) the MISS-cleanup floor.
                                     # DERIVED, not chosen (census D7). It must clear
                                     # the band in which a real seat edge can land,
                                     # because since 2026-08-10 the possession verdict
                                     # is sensor-PRIMARY: the deadline that mints
                                     # MISSED has to outlast the latest arrival the
                                     # sensor has ever observed. It was a hand-written
                                     # 0.7 until 2026-08-21, i.e. 98 ms UNDER the
                                     # measured +798 ms ceiling — latent today only
                                     # because the tracker's own CAUGHT lands earlier
                                     # (+202..+442 ms) and the merge falls back to it.
                                     # Deriving it put the post-FW14 band re-measure
                                     # in ONE place, and on 2026-08-24 that is what
                                     # happened: ARRIVAL_BAND_MAX_S 0.80 -> 0.56, and
                                     # this, the reload twin and
                                     # DEFAULT_SESSION_MISS_CLEANUP_S followed without
                                     # an edit of their own.
                                     # (Still absorbs the +0.115 s announced-early
                                     # bias of the fourth sitting, with more room.)
                                     #
                                     # ⚠ CORRECT BY CONSTRUCTION SINCE 2026-08-26 (D1),
                                     # where before it was correct by luck. The comment
                                     # above is right that the deadline must outlast the
                                     # SENSOR band — and until D1 the budget was
                                     # actually being spent on the mocap TRACKER's
                                     # CAUGHT latency, because `ball_caught` was minted
                                     # only on a tracker CAUGHT. Two unrelated
                                     # quantities were sharing one number. Bag
                                     # 2026-08-26_14-25-16 made it visible: three
                                     # genuine catches whose CUP edge landed at +0.222 /
                                     # +0.249 / +0.227 s died MISSED because the
                                     # TRACKER's CAUGHT did not arrive until +0.615 /
                                     # +0.830 / +0.622 s. With the cup as the sole
                                     # consumer the derivation closes on its own terms:
                                     # the LATEST cup arrival edge in that entire bag is
                                     # +0.303 s against this 0.560 s, i.e. 1.85x of
                                     # margin. The constant did NOT change and the fix
                                     # was in the CONSUMER — a shrink would have been
                                     # the wrong lever, and was the pre-registered H1a
                                     # this investigation refuted.
TOSS_CANCEL_CUTOFF_S = 0.25          # node-level (§ cancellation): cancels honoured up
                                     # to t_release − this; later ⇒ deferred to the
                                     # FSM's own terminal.

# Workspace pre-check (loud early reject; the feasibility gate remains the truth —
# these are planning-envelope values, POLICY not physics):
TOSS_XY_LIMIT_MM = 150.0             # |x|, |y| bound on the nominated catch pose —
                                     # the NO-CONFIG fallback only since 2026-08-14:
                                     # the node resolves hw.JB_OP_TOSS_WORKSPACE_XY_MM
                                     # (YAML toss_workspace_xy_mm, operator-adjustable)
                                     # and passes it into the ctor (workspace_xy_mm).
                                     # The YAML default sits ABOVE the displacement
                                     # cap so the centroid-vs-cup chain divergence
                                     # (2.07 % of displacement) stops binding at the
                                     # cap edge — see the YAML key's comment.
TOSS_ACTIVE_Z_MM = 170.0             # ACTIVE plane (JB_OP_DEFAULT_ACTIVE_Z_MM; pinned
                                     # by the config drift-guard test)
TOSS_Z_BAND_MM = 50.0                # |z − ACTIVE| bound (the sweep is ±30)

# Teensy bridge event_vel acceptance band (hw.TEENSY_TRAJ_MIN/MAX_EVENT_VEL_MPS —
# the bridge raises outside [0.3, 7.0] before any CAN frame; this is the loud+early
# copy, pinned by the config drift-guard test).
TEENSY_MIN_EVENT_VEL_MPS = 0.3
TEENSY_MAX_EVENT_VEL_MPS = 7.0

# ── Tier-8b displaced-throw CHECKING gates (Phase 4; re-based Phase E) ─────────
# |B_xy − A_xy| cap — the NO-CONFIG fallback only. The node resolves
# hw.JB_OP_TOSS_MAX_DISPLACEMENT_MM and passes it into the ctor
# (``max_displacement_mm``); this literal serves standalone/test use and the
# config drift-guard test pins the two equal. Same pattern as
# DEFAULT_TOSS_FLIGHT_TIME_S.
#
# It caps REQUESTED displacement, and nothing else. Until 2026-07-29 it was 70 mm
# = the intersection of (a) trajectory_node's 80 mm catch reach envelope — which
# back then was captured at A, so a B-reach beyond it was structurally rejected
# WORKSPACE *mid-flight, after the ball was airborne* (hardware, 4/4: bag
# 2026-07-27_16-07-30, 113-141 mm goals) — with (b) the bb Rung-2a "clean box"
# (~±70 mm). **Contract C-REACH-1 (ros_ws/docs/catch_reach_envelope.md) removed
# half (a)**: the envelope now centres on the NOMINATED catch B, because it
# exists to bound UNREQUESTED drift, not requested reach. So this cap is now the
# sole bound on |B−A| and must carry its own justification rather than inheriting
# the envelope's — see the YAML key's comment for the evidence (production
# planner 8/8 out to 225 mm; hardware validated only to 70 mm).
#
# The closed-form reach bound below remains a SECOND, flight-dependent gate; it
# is CONSERVATIVE below T ≈ 0.75 s and OPTIMISTIC above it (measured,
# tools/probes/displaced_reach_frontier.py 2026-07-29), which is precisely why
# the cap must not be relaxed to lean on it.
TOSS_MAX_DISPLACEMENT_MM = 150.0
# Closed-form peak factors of build_catch's quintic (min-jerk, zero boundary
# velocities) over displacement d and lead T: peak vel = 1.875·d/T, peak acc =
# 5.7735·d/T², peak |jerk| = 60·d/T³ (platform space; leg-space peaks are the
# planner's validate truth — direction cosines ≲ 1 for xy translation).
REACH_PEAK_VEL_FACTOR = 1.875
REACH_PEAK_ACC_FACTOR = 5.7735
REACH_PEAK_JERK_FACTOR = 60.0
# FALLBACK limits for the closed-form gate — the generated-config YAML working
# point (hw.JB_TRAJ_LEG_VEL/ACC/JERK_LIMIT, pinned by the drift-guard test;
# local copies keep this module importable standalone). Since 2026-08-14 the
# gate PREFERS the LIVE session limits published on trajectory/status
# (TossObservations.leg_*_limit_*): trajectory_node's set_limits ramp is the
# operator's primary movement-acceptance authority, so the pre-throw bound must
# judge against what the feasibility gate will ACTUALLY enforce at t_release —
# a ramp-down otherwise passes a goal the planner refuses MID-FLIGHT, and a
# ramp-up is refused pre-throw for no physical reason. These copies engage only
# when the live values are absent (0.0: pre-field publisher, stale status, bag
# replay, standalone use), degrading to exactly the pre-2026-08-14 behaviour.
# History of the bound itself: at the OLD 70 mm cap it never bound (d_max ≥
# 83.2 mm at T = 0.55 s); at the Phase-E 150 mm cap it is LIVE and binding at
# the default limits — it refuses a 150 mm goal below T ≈ 0.669 s (60·d/T³ =
# jerk), while over-conservative at T = 0.60 (bound 108 mm, real frontier
# 175 mm); the operator raises T, the live limits, or both.
REACH_VEL_LIMIT_MMPS = 1000.0
REACH_ACC_LIMIT_MMPS2 = 5000.0
REACH_JERK_LIMIT_MMPS3 = 30000.0


def reach_displacement_limit_mm(flight_time_s: float,
                                vel_mmps: 'float | None' = None,
                                acc_mmps2: 'float | None' = None,
                                jerk_mmps3: 'float | None' = None) -> float:
    """Max A→B displacement (mm) the deferred catch reach can span in
    ``flight_time_s`` — the closed-form inversion of the quintic peak factors
    (d_max = min(vel·T/1.875, acc·T²/5.7735, jerk·T³/60)).

    Each limit argument is the LIVE session value from trajectory/status when
    the caller has one; ``None`` (or 0.0 upstream, mapped to ``None`` by the
    gate) falls back to the module's YAML-default copies, preserving the
    pre-live-limits behaviour bit-for-bit (spot values at the defaults:
    T = 0.55 → 83.2 mm jerk-bound; T = 0.80 → 256 mm jerk-bound).

    **It is an approximation of the real gate in BOTH directions, measured.**
    Against ``planner.build_catch`` over the 8-direction ring
    (``tools/probes/displaced_reach_frontier.py``, 2026-07-29) the real
    all-8-directions frontier is 125 mm at T = 0.55 (closed form 83.2 —
    CONSERVATIVE, it refuses feasible throws), 175 mm at T = 0.60 (cf 108.0 —
    conservative), and ~225 mm from T = 0.70 up (cf 171.5 → 586.7 — OPTIMISTIC
    above T ≈ 0.75 s: it would pass a 250-400 mm reach the planner rejects
    2/8-4/8). It is a loud+early convenience, never the truth: the truth is
    ``trajectory_node``'s own feasibility gate, and the SAFETY margin against
    the optimistic half is ``max_displacement_mm``, not this bound."""
    return reach_displacement_bound(flight_time_s, vel_mmps, acc_mmps2,
                                    jerk_mmps3)[0]


def reach_displacement_bound(flight_time_s: float,
                             vel_mmps: 'float | None' = None,
                             acc_mmps2: 'float | None' = None,
                             jerk_mmps3: 'float | None' = None) -> tuple:
    """``(limit_mm, binding_term)`` — :func:`reach_displacement_limit_mm` plus
    WHICH of the three peaks produced it (``'vel'`` / ``'acc'`` / ``'jerk'``).

    ONE derivation, two callers: the gate takes the number, the REFUSAL takes
    the name. "100 mm exceeds the 93.2 mm reach bound" sends an operator hunting
    through three limits; "jerk-bound at T = 0.571 s" sends them to the one that
    actually binds — and which term binds decides which remedy is cheap, since
    the bound grows as T³ on jerk and only as T on velocity."""
    t = float(flight_time_s)
    vel = REACH_VEL_LIMIT_MMPS if vel_mmps is None else float(vel_mmps)
    acc = REACH_ACC_LIMIT_MMPS2 if acc_mmps2 is None else float(acc_mmps2)
    jerk = REACH_JERK_LIMIT_MMPS3 if jerk_mmps3 is None else float(jerk_mmps3)
    # An explicit min-then-match rather than min() over (value, name) pairs:
    # that would order by NAME on a tie, while this keeps the vel < acc < jerk
    # precedence the peaks are quoted in everywhere else.
    terms = ((vel * t / REACH_PEAK_VEL_FACTOR, 'vel'),
             (acc * t * t / REACH_PEAK_ACC_FACTOR, 'acc'),
             (jerk * t ** 3 / REACH_PEAK_JERK_FACTOR, 'jerk'))
    limit = min(value for value, _name in terms)
    for value, name in terms:
        if value == limit:
            return limit, name
    return limit, 'jerk'               # unreachable except for an all-NaN T

# ── Tier-8a vertical-toss ballistics (default event_vel) ───────────────────────
# Kept as module constants (not a generated-config / motion import) so this module
# stays importable standalone, mirroring reload_sequencer's BB-enum pattern; the
# config drift-guard test pins them against the generated inputs. The authoritative
# release-state math (frames, announcement fields) is motion/trajectory/toss_release —
# the node passes its event_vel_mps in; this closed form is the 0 ⇒ default fallback
# and pins the same worked example (T = 0.8 s ⇒ 3.93082 m/s).
GRAVITY_MMS2 = 9806.0                # ballistics-side gravity (NEVER the tracker's 9810)
HAND_CATCH_OFFSET_MM = 64.78         # cup plane above platform centroid (generated:
                                     # hw.HAND_CATCH_OFFSET_MM)
HAND_THROW_RELEASE_OFFSET_MM = 58.044  # release plane above platform centroid =
                                     # hw.GEOM_HAND_AXIS_BOTTOM_OFFSET_MM (−129.0) +
                                     # hw.HAND_THROW_POS_M (0.187044)·1000 — no
                                     # generated constant exists for the throw case
                                     # (derivation shared with toss_release.py).


def vertical_event_vel_mps(flight_time_s: float) -> float:
    """Tier-8a co-located vertical toss: the release speed for a flight time.

    Launch is purely vertical, so ``|launch| = vz = Δz/T + g·T/2`` with
    ``Δz = cup plane − release plane`` — the FULL ballistic inverse, matching
    ``motion/trajectory/toss_release.compute_release_state``, NOT the idealised
    ``g·T/2`` magnitude (which is the Δz → 0 limit, ≈ 8.4 mm/s adrift at
    T = 0.8 s).

    A module function since 2026-08-22 because it has a SECOND caller: the
    session's hand-geometry dwell floor (``toss_session.hand_floor_dwell_s``)
    needs a release speed at SESSION checking, before any cycle FSM exists.
    Inlined twice it would be one edit away from a session that refuses a cadence
    the cycle can make, or admits one it cannot."""
    t = float(flight_time_s)
    vz_mms = ((HAND_CATCH_OFFSET_MM - HAND_THROW_RELEASE_OFFSET_MM) / t
              + GRAVITY_MMS2 * t / 2.0)
    return vz_mms / 1000.0


# ── THE pre-dispatch budget — one derivation, two gates (2026-08-23) ───────────
# `reload_coordinator_node._TICK_S`, restated here because it is now ARITHMETIC
# and not merely latency: the pre-dispatch sequence is counted in ticks, so the
# accept-time floor moves when the tick does.  `toss_session` re-exports this
# name (it lived there until 2026-08-23) and the drift-guard test pins all three
# equal.
NODE_TICK_S = 0.02

#: THE LOOP PERIOD — one iteration of ``_run_toss_cycle``, wall-clock. **This, not
#: :data:`NODE_TICK_S`, is what the pre-dispatch ladder is counted in** (owner
#: decision D3, 2026-08-26).
#:
#: The loop WAS ``work; time.sleep(_TICK_S)``, so an iteration cost the sleep PLUS
#: the tick's own work — and the pre-dispatch ticks are the expensive ones in the
#: whole sequence: ``_build_toss_observations`` (locks, a sensor query, and a numpy
#: norm only when the mocap cross-check is CONFIGURED, which the shipped default
#: is not), the feedback publish, and then a BLOCKING dispatch — the reach-centre
#: declaration, the PREPARE bundle (soft gains + ``trajectory/arm_catch`` raise and
#: confirm + vel scale + two publishes), the announcement build and publish.
#: Charging them at the SLEEP was the error: ``pre_dispatch_budget_s`` promised the
#: lead the sequence spends before the runtime guard runs, and it named the tick
#: ladder correctly while pricing every rung at zero work.
#:
#: ⚠ SINCE 2026-08-27 THIS IS ALSO A SET-POINT, not only a bound (plan B5 lever 1).
#: Both toss loops — ``_run_toss_cycle`` and ``_execute_toss_continuous`` — now
#: pace to an ABSOLUTE grid at this value through
#: ``reload_coordinator_node._pace_to_next_tick`` instead of sleeping a fixed
#: ``NODE_TICK_S``. The number did not move and its meaning as the budget
#: denominator did not change; what changed is that the machine now RUNS at it
#: rather than near it, which is the grid ``tools/probes/cadence_rung_check.py``
#: has always modelled. Read the two together: raising this constant now also
#: slows every toss loop, and lowering it below the measured
#: ``loop_work_max_pre_s`` degenerates the pacer to no sleep at all. The
#: argument, the corpus numbers behind it and the early-fire band are written out
#: at ``reload_coordinator_node._PACE_PERIOD_S`` / ``_PACE_SLOP_S``.
#:
#: MEASURED, and the measurement is reproducible from a bag with one grep.
#: ``2026-08-26_14-25-16``, **28 cycle starts** across 12 goals: from the tick that
#: dispatches ``ACTION_POSITION_PLATFORM`` (``reload_coordinator_node`` logs
#: "POSITIONING skipped …"/"POSITIONING commanded …" there) to the tick that
#: publishes the self-announcement (``/throw_ann``) is **exactly 3 loop
#: iterations** — positioning→PREPARE, the deferred PREPARE bundle answering,
#: ANNOUNCE. Measured span **0.080 – 0.113 s**, median 0.0905 ⇒ **0.0267 – 0.0377 s
#: per iteration**, i.e. 1.33x – 1.89x the sleep. Ceiled to the next 10 ms so the
#: constant is a bound rather than a datum (the sizing discipline
#: ``ARRIVAL_BAND_MAX_S`` uses): **0.040 s, exactly 2 x NODE_TICK_S**.
#:
#: WHAT IT COST TO GET THIS WRONG. Both ``ABORTED_CANT_MAKE_RELEASE`` cycles in
#: that bag are this shortfall and nothing else:
#:
#: ==========  =====  ============  ==========  =============  ================
#: cycle       v m/s  dispatch bgt  throw_delay accept floor    lead at guard
#: ==========  =====  ============  ==========  =============  ================
#: run 2 c1     2.48       0.3344        0.440   0.4144 (old)   0.330 -> ABORT
#: run 10 c2    3.92       0.2813        0.400   0.3613 (old)   0.272 -> ABORT
#: ==========  =====  ============  ==========  =============  ================
#:
#: Both cleared their accept floor — by 26 ms and 39 ms — and both then died at the
#: runtime guard, with the catch latch raised, the announcement out and a phantom
#: tracker expectation left behind. That is exactly the failure
#: :func:`min_throw_delay_for_release_s` exists to make unreachable, and it stayed
#: reachable because the two gates agreed on an arithmetic that was 0.08 s short.
#: With the loop period charged, both goals are REFUSED AT ACCEPT instead — loud,
#: early, nothing armed.
#:
#: ⚠ NOT A JITTER ALLOWANCE, and not a licence to stop measuring. It is the
#: measured cost of the work in one iteration on THIS Jetson at THIS tick rate —
#: bounded from the three MEASURED iterations (positioning→PREPARE, the deferred
#: bundle answering, ANNOUNCE) plus the charged fourth (ANNOUNCE→DISPATCH), which
#: the bag cannot time because nothing logs the dispatch tick. A tick
#: that grows new work (another blocking service in the PREPARE bundle) moves this
#: number, and the way to find out is to MEASURE it, not to reason about it.
#:
#: SINCE 2026-08-26 THE MACHINE MEASURES IT FOR YOU. :class:`LoopPeriodCensus`
#: censuses every cycle's pre-dispatch iterations and reports
#: ``loop_period_max_pre_s`` (this bound), ``loop_work_max_pre_s`` (the iteration
#: minus its sleep) and an obs/body/sleep split on every toss record — plus a WARN
#: on any cycle where a pre-dispatch tick exceeded this constant. Read those first;
#: the grep recipe above is now the FALLBACK, for bags predating the census.
#:
#: ⚠ The census must never SET this number. A bound that re-derives itself from
#: observed slowness tracks a degradation instead of exposing it, which is the
#: failure described immediately above. It stays hand-set and reviewed; the census
#: only tells a human when to look. Pinned by
#: ``tests/ros/test_toss_sequencer.py::test_the_census_never_feeds_a_budget``.
NODE_LOOP_PERIOD_S = 0.04


# ── The loop-period census (INSTRUMENT ONLY — no control authority) ───────────
#: Exactly the phases :func:`pre_dispatch_budget_s` charges.
#:
#: The split is what makes the census honest. A cycle spends most of its ticks
#: waiting out a 0.5 s flight, and those ticks are almost all ``ACTION_NONE``;
#: the handful BEFORE the throw are the ones that run a blocking ``arm_catch``
#: raise-and-confirm and an announcement publish, and they are the only ones any
#: delay floor is denominated in. A whole-cycle mean would be dominated by the
#: cheap majority and would hide the expensive minority — which is precisely the
#: quantity :data:`NODE_LOOP_PERIOD_S` has to bound.
#:
#: ``PHASE_COMMITTING`` joins them at B4 and ``PHASE_STAGED`` deliberately does
#: not. The commit tick is exactly what :func:`commit_budget_s` charges its one
#: loop period for, so it belongs in the same census as the serial ladder's four;
#: STAGED is a WAIT — the pipelined analogue of flight-waiting — and folding a
#: long idle into the pre-dispatch statistic would dilute precisely the expensive
#: minority the census exists to expose. Neither phase is reachable on the serial
#: path, so adding one of them moves no serial number.
PRE_DISPATCH_PHASES = frozenset(
    (PHASE_CHECKING, PHASE_POSITIONING, PHASE_PREPARING, PHASE_COMMITTING))

#: The census field names, in record order.
#:
#: ``toss_record.FIELDS`` declares the same ten names and is pinned equal to this
#: tuple by ``tests/motion/test_toss_record.py``. A drift-guard test rather than
#: an import, deliberately: ``toss_record`` takes exactly ONE jugglebot import
#: (``ball_possession``, pure and leaf) so that a corpus reader never drags in the
#: FSM, and importing this module there would cost that property for a list of
#: strings. Same trade the ``_TICK_S`` / ``NODE_TICK_S`` mirror already makes.
CENSUS_FIELD_NAMES = (
    'loop_n_pre',
    'loop_period_max_pre_s',
    'loop_period_mean_pre_s',
    'loop_work_max_pre_s',
    'loop_obs_max_pre_s',
    'loop_body_max_pre_s',
    'loop_sleep_max_pre_s',
    'loop_n_over_pre',
    'loop_n_post',
    'loop_period_max_post_s',
)


class LoopPeriodCensus:
    """Wall-clock census of ONE toss cycle's own tick loop.

    **INSTRUMENT ONLY. No control authority, now or ever** — and specifically
    NOT a source for :data:`NODE_LOOP_PERIOD_S`. That constant is a reviewed
    bound on how long an iteration may take; a bound that re-derived itself from
    the last cycle would TRACK a degradation instead of exposing it, and the two
    ``ABORTED_CANT_MAKE_RELEASE`` cycles of 2026-08-26 are what a silently
    moving floor costs. This class answers "should a human move it?" and stops
    there. Nothing here may be read by a gate, a budget or an FSM transition.

    **What it measures, and why the boundaries are where they are.**

    An iteration is ``now`` to the next ``now`` — the loop's OWN top-of-loop
    ``time.perf_counter()`` read, reused rather than re-taken. That is not
    frugality: ``now`` is the clock the FSM reasons with, so the interval between
    successive ``now`` values IS the granularity at which the release-window
    guard sees time advance. Any other pair of stamps measures a nearby but
    different quantity.

    Each iteration decomposes into three terms, which between them adjudicate the
    three suspected costs without inference::

        obs   = t_obs_done  - now           the per-tick observation rebuild
        body  = t_pre_sleep - t_obs_done    step + blocking dispatch + publishes
        sleep = next_now    - t_pre_sleep   what the WAIT actually cost

    ``sleep`` is measured, never assumed — and that is what kept the field
    truthful across the change of what the wait IS. Until 2026-08-27 the wait was
    a fixed ``time.sleep(NODE_TICK_S)`` and the field answered "how far past
    0.020 s did the scheduler return?" (measured p50 +1.5 ms, max +6.8 ms over 73
    chained cycles); on a loaded Jetson folding that overshoot into ``body``
    would have charged six milliseconds to code that did not run slowly, and the
    executor/GIL-contention question would have been indistinguishable from the
    blocking-service question — the whole thing the census exists to separate.

    **Since B5 the wait is `_pace_to_next_tick`, and the field's ARITHMETIC is
    unchanged while its INTERPRETATION flips.** It is still ``next_now -
    t_pre_sleep``, still the real interval, still never inferred. But under an
    absolute grid the pacer sleeps ``period - work``, so a LARGE
    ``loop_sleep_max_pre_s`` now means HEADROOM (the tick finished early and the
    grid held it), and a value at or near ZERO is the interesting one: it means
    the iteration's own work consumed the whole period and the pacer degenerated.
    Read it beside ``loop_work_max_pre_s`` — ``period ≈ work + sleep`` still
    holds, and under pacing the left-hand side is the constant, so the two on the
    right trade against each other directly.  ``tools/probes/toss_loop_census.py``
    reads the field by this name and needs no change: the number it prints is the
    same measurement, and its own "dominant term" argmax stays meaningful (a
    ``sleep``-dominant cycle is now a cycle with headroom to spare).

    **Why a row lags by one call.** An iteration's period and its sleep are only
    knowable once the NEXT one starts, so :meth:`note_iteration_start` commits
    the previous row. The terminal iteration returns from the middle of the loop
    and never reaches :meth:`note_iteration_end`, so it is never committed — also
    deliberate: it has no trailing sleep, and counting it would report a period
    the loop never spent.

    O(1) and allocation-free per tick (ten float compares, no container touched),
    and lock-free: every method is called from the cycle thread alone, and
    ``_log_toss_outcome`` reads the summary on that same thread. Taking the
    node's lock here would add contention to the loop under measurement.
    """

    __slots__ = ('n_pre', 'sum_pre_s', 'max_pre_s', 'n_over_pre',
                 'max_work_pre_s', 'max_obs_pre_s', 'max_body_pre_s',
                 'max_sleep_pre_s', 'n_post', 'max_post_s',
                 '_open_now', '_open_pre_sleep', '_open_obs_s', '_open_body_s',
                 '_open_is_pre', 'over_threshold_s')

    def __init__(self, over_threshold_s: float = NODE_LOOP_PERIOD_S) -> None:
        self.over_threshold_s = float(over_threshold_s)
        self.n_pre = 0
        self.sum_pre_s = 0.0
        self.max_pre_s = 0.0
        self.n_over_pre = 0
        self.max_work_pre_s = 0.0
        self.max_obs_pre_s = 0.0
        self.max_body_pre_s = 0.0
        self.max_sleep_pre_s = 0.0
        self.n_post = 0
        self.max_post_s = 0.0
        self._open_now = None
        self._open_pre_sleep = 0.0
        self._open_obs_s = 0.0
        self._open_body_s = 0.0
        self._open_is_pre = False

    def note_iteration_start(self, now: float) -> None:
        """Top of the loop, with the loop's own ``now``. Commits the previous
        iteration, whose period is only measurable from here."""
        if self._open_now is not None:
            self._commit(now - self._open_now, now - self._open_pre_sleep)
            self._open_now = None

    def note_iteration_end(self, now: float, t_obs_done: float,
                           t_pre_sleep: float, phase: str) -> None:
        """Bottom of the loop, immediately before the WAIT (a fixed sleep until
        2026-08-27, ``_pace_to_next_tick`` since). ``now`` is the SAME
        value passed to :meth:`note_iteration_start` for this iteration."""
        self._open_now = now
        self._open_pre_sleep = t_pre_sleep
        self._open_obs_s = t_obs_done - now
        self._open_body_s = t_pre_sleep - t_obs_done
        self._open_is_pre = phase in PRE_DISPATCH_PHASES

    def _commit(self, period_s: float, sleep_s: float) -> None:
        if not self._open_is_pre:
            self.n_post += 1
            if period_s > self.max_post_s:
                self.max_post_s = period_s
            return
        work_s = self._open_obs_s + self._open_body_s
        self.n_pre += 1
        self.sum_pre_s += period_s
        if period_s > self.max_pre_s:
            self.max_pre_s = period_s
        if period_s > self.over_threshold_s:
            self.n_over_pre += 1
        if work_s > self.max_work_pre_s:
            self.max_work_pre_s = work_s
        if self._open_obs_s > self.max_obs_pre_s:
            self.max_obs_pre_s = self._open_obs_s
        if self._open_body_s > self.max_body_pre_s:
            self.max_body_pre_s = self._open_body_s
        if sleep_s > self.max_sleep_pre_s:
            self.max_sleep_pre_s = sleep_s

    @property
    def overran(self) -> bool:
        """At least one pre-dispatch iteration exceeded the threshold — i.e. the
        bound every delay floor is built on did not hold this cycle."""
        return self.n_over_pre > 0

    def summary(self) -> dict:
        """The record fields. All ``None`` when no complete pre-dispatch
        iteration was seen (the ``REJECTED_BAD_GOAL`` path, or a cycle that
        terminated inside its first tick) — ``None`` is "not measured", which is
        a different fact from a measured zero and the record's null discipline
        keeps them apart.

        ``loop_work_max_pre_s`` is the max of the per-iteration SUM, not the sum
        of the two maxima: those differ, and only the former answers "how close
        did the worst tick come to its budget".
        """
        if self.n_pre <= 0:
            return {name: None for name in CENSUS_FIELD_NAMES}
        return {
            'loop_n_pre': int(self.n_pre),
            'loop_period_max_pre_s': float(self.max_pre_s),
            'loop_period_mean_pre_s': float(self.sum_pre_s / self.n_pre),
            'loop_work_max_pre_s': float(self.max_work_pre_s),
            'loop_obs_max_pre_s': float(self.max_obs_pre_s),
            'loop_body_max_pre_s': float(self.max_body_pre_s),
            'loop_sleep_max_pre_s': float(self.max_sleep_pre_s),
            'loop_n_over_pre': int(self.n_over_pre),
            'loop_n_post': int(self.n_post),
            'loop_period_max_post_s': (float(self.max_post_s)
                                       if self.n_post > 0 else None),
        }


#: One microsecond of slack on the accept-time delay floor, and it is NOT a
#: jitter allowance — bounding scheduling jitter is the RUNTIME guard's job and
#: no static floor can do it (an ``ABORTED_CANT_MAKE_RELEASE`` from a late tick
#: is a real finding and the cadence runbook aborts the sitting on one).
#:
#: What it buys is that the floor is STRICTLY sufficient rather than
#: exactly-equal-and-lucky.  The runtime guard asks
#: ``t_release − now < dispatch_budget``, and at a delay sitting exactly on the
#: floor that is ``(dispatch + budget) − budget < dispatch`` — an identity in
#: real arithmetic and a coin flip in binary floating point, because
#: ``(a + b) − b`` is not ``a``.  Measured against the tree on 2026-08-23: the
#: R5 rung's exactly-at-floor delay passed and R4's aborted, for no reason but
#: the bit pattern.  A contract that holds by luck at half its grid points is not
#: a contract.
#:
#: 1e-6 s is 1/20000 of a node tick: invisible in every published cadence number
#: (the ladder's rungs clear their floors by 10-40 ms) and ~10 orders of
#: magnitude above the double-rounding error it covers.
FLOOR_REPRESENTATION_SLACK_S = 1e-6


def pre_dispatch_budget_s(positioning_move: bool,
                          loop_period_s: float = NODE_LOOP_PERIOD_S) -> float:
    """Cycle START → the LAST evaluation of the release-window guard, in seconds.

    **The quantity the accept-time delay floor was missing.**  Until 2026-08-23
    both delay gates — ``TossSequencer``'s CHECKING mirror and
    ``toss_session.min_throw_delay_s`` — charged the kind-0 dispatch budget
    alone (``hand_stroke.min_throw_event_delay_s``).  The runtime guard in
    :meth:`TossSequencer._step_preparing` applies that SAME budget to the lead
    *remaining* after CHECKING, POSITIONING and the PREPARE ladder have already
    elapsed, so a goal could clear the accept gate by construction and then abort
    ``ABORTED_CANT_MAKE_RELEASE`` every cycle — at ``cycle_start + 0.06 s``, with
    the hand retracting under a seated ball.  Three published rungs of
    ``tests/hardware/session_cadence_ladder.md`` did exactly that.

    The sequence is fully determined by the node's own tick ladder, so it is
    derived here rather than measured (``tools/probes/cadence_rung_check.py``
    drives the real FSM at the real tick and pins these numbers):

    ===============================  ==========================================
    tick 0                           CHECKING passes → ``ACTION_POSITION_PLATFORM``;
                                     the node answers SYNCHRONOUSLY inside the
                                     tick (``note_position_noop`` or
                                     ``note_position_result``)
    the first tick at/after arrival  ``_step_positioning`` → ``_enter_preparing``
                                     → ``ACTION_PREPARE_CATCH``
    +1 tick                          the node's DEFERRED PREPARE bundle answers
                                     (``note_prepare_result``)
    +1 tick                          release-window guard, then ``ACTION_ANNOUNCE``
    +1 tick                          release-window guard, then DISPATCH
    ===============================  ==========================================

    ``arrival`` is 0 when POSITIONING is a no-op (census B1) and
    ``TOSS_POSITION_MIN_MOVE_S + TOSS_POSITION_SETTLE_PAD_S`` when it commands a
    move — the planner floor even a ZERO-millimetre move pays, plus the pad
    ``note_position_result`` adds.  A longer real move costs more, but that is a
    dynamic quantity (how far the platform is from the pre-positioning pose) and
    no static gate can bound it; what this closes is the STATIC shortfall.

    ⚠ **THE UNIT IS THE LOOP PERIOD, NOT THE SLEEP** (owner decision D3,
    2026-08-26).  Until then this counted the ladder in ``NODE_TICK_S`` — the
    ``time.sleep`` at the bottom of ``_run_toss_cycle`` — which prices every rung's
    own work at zero, and the pre-dispatch rungs are the ones that DO work (a
    blocking arm_catch raise+confirm, an announcement build and publish).  The
    measured iteration is 0.0267 – 0.0377 s, bounded by
    :data:`NODE_LOOP_PERIOD_S` = 0.040 s; the 0.020 s charge was short by a factor
    of two, and it is what killed both ``ABORTED_CANT_MAKE_RELEASE`` cycles of bag
    ``2026-08-26_14-25-16`` after they had cleared this very gate.  See
    :data:`NODE_LOOP_PERIOD_S` for the measurement and the two cycles.

    So: **0.160 s with the skip, 0.520 s without it** (0.080 / 0.460 before D3).
    That ~0.36 s gap is the whole LEVEL/AIMED split the cadence ladder published —
    and since 2026-08-23 an aimed CHAIN takes the skip too (the B1 predicate is
    orientation-aware), so the discriminator is "does POSITIONING command a move",
    never "is the release tilted".

    ⚠ **S6 (2026-08-27) made two of these rungs cheaper and this number did NOT
    move.** The session-scoped catch latch hoists the ``arm_catch``
    raise-and-confirm and the soft-gains call out of every chained cycle after
    the first, and it satisfies the armed→announce gap by construction (the
    latch was raised seconds earlier), so the ANNOUNCE tick no longer needs a
    tick to buy that ordering. Both savings are DELIBERATELY unbanked here: this
    function is the SERIAL ladder's budget and the serial ladder still walks all
    four iterations. The tick the gap used to cost is returned in B4's
    ``commit_budget_s`` — a separate derivation, charged by a gate that measures
    the real lead — and never by quietly shrinking the constant a runtime guard
    is fronted by. Two branches, one derivation each.
    """
    loop = float(loop_period_s)
    arrival_s = (TOSS_POSITION_MIN_MOVE_S + TOSS_POSITION_SETTLE_PAD_S
                 if positioning_move else 0.0)
    # ceil to the iteration the FSM actually observes the arrival on, never fewer
    # than one (the noop declares arrival inside tick 0, and _step_positioning
    # still runs no earlier than tick 1).  The epsilon is not cosmetic and it is
    # not tied to any particular pair of numbers: `arrival_s / loop` is a ratio of
    # decimal literals that are not exactly representable in binary, so a quotient
    # that is a whole number in real arithmetic can land a few ULPs ABOVE it, and
    # a bare ceil() would then charge an extra iteration the machine never spends
    # — 0.040 s of phantom lead demanded at the accept gate, on a rung whose whole
    # clearance is single-digit milliseconds.  Today's operands happen to divide
    # clean (0.400/0.040 is exactly 10.0, and so was 0.400/0.020), but `arrival_s`
    # is built from two CONFIG values and `loop` is a measured constant: any of the
    # three moving can re-open it.  The guard is written for the class, not for a
    # worked example.  1e-9 is ~11 orders of magnitude above the representation
    # error and ~7 below the smallest real quantity here, so it cannot mask a
    # genuine extra iteration.
    arrival_ticks = max(1, int(math.ceil(arrival_s / loop - 1e-9)))
    return (arrival_ticks + 3) * loop


def commit_budget_s(event_vel_mps: float, min_event_delay_s: float = 0.0,
                    loop_period_s: float = NODE_LOOP_PERIOD_S) -> float:
    """Cycle COMMIT → release, in seconds. The PIPELINED sibling of
    :func:`pre_dispatch_budget_s` + the dispatch budget (plan
    ``toss-pipelined-preamble.md`` § 2.7), and it lives here, next to it, so an
    edit to either sees the other.

    **ONE loop period, not four.** Under the two-slot pipeline the announce
    tick, the PREPARE tick and the deferred-bundle tick have already run inside
    the PREVIOUS cycle's flight (:data:`PHASE_STAGED`), and the ≥1-tick
    armed→announce gap is satisfied by construction rather than by a tick: the
    session-scoped ``trajectory/arm_catch`` latch (S6) stands for the whole run,
    and ``catch/armed`` is not lowered between chained cycles while a staged slot
    is live, so ``catch_coordinator`` has been armed since before this cycle
    existed. The one period charged is the COMMIT tick itself, which is
    **polled**: the iteration that crosses ``commit_at`` may be up to one full
    loop period late, and that lateness comes straight off the lead the runtime
    release-window guard measures.

    Note what is NOT here and IS in :func:`min_throw_delay_for_release_s`: the
    ``max(TOSS_DISPATCH_DEBOUNCE_S, …)`` clamp. That constant is a goal-storm
    debounce on ``throw_delay_s``, an operator-facing field; the commit budget is
    an internal schedule offset no operator types, so clamping it would charge a
    0.10 s floor for a hazard that is not on this path. Omitted deliberately, not
    by transcription.

    ``tools/probes/cadence_rung_check.py`` MODELLED this function through Phase
    B0 and now imports it — the reconciliation its docstring demanded, pinned
    equal by ``tests/motion/test_cadence_rung_check.py``. A probe that keeps its
    own copy of a shipped floor is the 2026-08-22 audit's finding wearing a
    different hat."""
    override = float(min_event_delay_s)
    dispatch_s = (override if override > 0.0
                  else hand_stroke.min_throw_event_delay_s(event_vel_mps))
    return dispatch_s + float(loop_period_s) + FLOOR_REPRESENTATION_SLACK_S


#: The STAGED preamble, in loop periods — the pipelined twin of
#: :func:`pre_dispatch_budget_s`'s ladder, counted the same way and from the same
#: tick table::
#:
#:     tick 0   CHECKING (STATIC gates only) passes -> ACTION_POSITION_PLATFORM;
#:              the node answers SYNCHRONOUSLY inside the tick (note_position_noop
#:              — staging is SKIP-ONLY, so there is never a service round trip)
#:     tick 1   _step_positioning -> _enter_preparing -> ACTION_PREPARE_CATCH
#:     tick 2   the node's deferred PREPARE answers (note_prepare_result)
#:     tick 3   _step_preparing sees the result -> PHASE_STAGED
#:
#: THREE periods, against the serial ladder's four, and the missing one is the
#: armed→announce gap S6 makes free. It is charged at CHECKING (loud, early,
#: nothing staged) and NOWHERE else — the commit gate charges
#: :func:`commit_budget_s` against the real remaining lead.
STAGE_LADDER_TICKS = 3


def stage_budget_s(loop_period_s: float = NODE_LOOP_PERIOD_S) -> float:
    """Cycle START → :data:`PHASE_STAGED`, in seconds. See
    :data:`STAGE_LADDER_TICKS` for the tick table it counts."""
    return STAGE_LADDER_TICKS * float(loop_period_s)


def min_stage_lead_for_release_s(event_vel_mps: float,
                                 min_event_delay_s: float = 0.0,
                                 loop_period_s: float = NODE_LOOP_PERIOD_S
                                 ) -> float:
    """THE accept-time floor on a STAGED cycle's lead — ``release_at_perf − now``
    at the instant the cycle is staged.

    The pipelined analogue of :func:`min_throw_delay_for_release_s`, and it
    exists for exactly the reason that one does: the runtime guard in
    :meth:`TossSequencer._step_committing` measures ``t_release − now`` against
    the kind-0 dispatch budget, and a cycle that cleared no static floor could
    reach that guard with the announcement about to go out. Charging
    ``stage_budget_s + commit_budget_s`` at CHECKING is what keeps
    ``ABORTED_CANT_MAKE_RELEASE`` off the pipelined path for every STATIC
    reason — the arithmetic of the goal, the beat and the ladder.

    ⚠ **It was claimed here as "structurally unreachable", and the first
    pipelined sitting refuted that** (2026-08-28,
    ``logbook/2026-08-28-pipeline-first-contact-deadlock.md``). The claim was
    a statement about arithmetic in a machine whose loop period is a MEASURED
    quantity, not a guaranteed one: :func:`commit_budget_s` grants exactly one
    NOMINAL loop period of polling lateness plus
    :data:`FLOOR_REPRESENTATION_SLACK_S`, so an iteration that took
    ``NODE_LOOP_PERIOD_S + 1 µs`` or more at the commit crossing broke the
    guard's inequality no matter how much static lead had been charged — and
    the Jetson's non-RT ``time.sleep`` overshoots by milliseconds routinely
    (4 of the sitting's 15 staged slots died this way, every one of them with a
    census ``loop_period_max_pre_s`` above 0.040 s).

    **The honest statement now**: this floor makes the abort unreachable for
    every static reason, and the commit gate's own late-tick shortfall is no
    longer an abort at all — it SLIPS (:meth:`TossSequencer._slip`), and only
    an exhausted slip bound (``catch_confirm_window_s`` from the ORIGINAL
    commit instant) terminalises. Beyond that bound the abort is honest and it
    no longer deadlocks the session: the machine slipped the whole window and
    still could not make the release.

    It REPLACES ``min_throw_delay_for_cycle_s`` for a staged cycle rather than
    joining it, because the two measure different quantities: that floor is
    charged against ``throw_delay_s``, which a staged cycle does not run on (its
    release is an absolute instant it was TOLD — plan § 2.6 rule 1). Leaving the
    delay floor as the only gate would be a gate on a number the cycle ignores;
    dropping both would be the retired-gate-nothing-replaces failure the probe's
    ``pipelined_session_accepts`` recorded as an open B4 decision. This is that
    decision, taken."""
    return (stage_budget_s(loop_period_s)
            + commit_budget_s(event_vel_mps, min_event_delay_s, loop_period_s))


def min_throw_delay_for_release_s(event_vel_mps: float,
                                  positioning_move: bool,
                                  min_event_delay_s: float = 0.0,
                                  loop_period_s: float = NODE_LOOP_PERIOD_S
                                  ) -> float:
    """THE accept-time ``throw_delay_s`` floor — the ONE derivation both gates use.

    ``max(TOSS_DISPATCH_DEBOUNCE_S, dispatch budget + pre-dispatch budget)``:

    * the **dispatch budget** is ``hand_stroke.min_throw_event_delay_s(v)``, the
      Teensy's own ``:642`` check written from the throw side (prelude + gap +
      windup).  It is what the runtime guard measures the REMAINING lead against;
    * the **pre-dispatch budget** is :func:`pre_dispatch_budget_s`, the lead the
      sequence spends BEFORE that guard runs.

    Adding them is the whole fix: a ``throw_delay_s`` at or above this floor
    cannot die ``ABORTED_CANT_MAKE_RELEASE`` from static arithmetic, because the
    guard's own inequality (``t_release − now ≥ dispatch budget``) is implied by
    it at the last tick the guard runs.  ``min_event_delay_s > 0`` is the same
    explicit override ``TossSequencer.min_event_delay_s`` is (tests, standalone
    harnesses); the shipped 0.0 derives the dispatch budget from the speed.

    Both gates import THIS function rather than restating it, for the reason the
    2026-08-22 audit found the hard way: the session's mirror and the cycle's own
    gate were two expressions of one floor, and they had already drifted from the
    guard they front for."""
    override = float(min_event_delay_s)
    dispatch_s = (override if override > 0.0
                  else hand_stroke.min_throw_event_delay_s(event_vel_mps))
    return max(TOSS_DISPATCH_DEBOUNCE_S,
               dispatch_s + pre_dispatch_budget_s(positioning_move, loop_period_s)
               + FLOOR_REPRESENTATION_SLACK_S)


@dataclass
class TossObservations:
    """A snapshot of everything the FSM reasons about — pure observations, no
    node/issuance state (that lives inside the FSM)."""
    now: float
    control_mode: str = ''            # must be the active toss mode (TRAJECTORY) throughout
    streaming: bool = False           # trajectory/status.streaming
    mocap_fresh: bool = False         # rigid_body_poses stamp within the staleness window
    platform_levelled: bool = False   # trajectory_node affirms, on a FRESH
                                      # trajectory/status, that it holds a gravity
                                      # correction (gravity_correction_loaded).
                                      # NOT RobotState.levelling_complete: that is a
                                      # Teensy-persisted per-boot flag which stays
                                      # True across a relaunch that empties the ROS
                                      # node's in-memory correction, so gating on it
                                      # would PASS in exactly the state this refuses.
                                      # NOT "the correction is non-identity" either —
                                      # a genuinely level machine has a zero offset
                                      # and must not be refused. Default False =
                                      # fail-closed: an FSM that was never told is
                                      # not entitled to assume. Consulted ONLY at
                                      # CHECKING (like track_active): a status
                                      # hiccup mid-sequence can never abort a
                                      # flight or retract into an incoming ball.
    hand_fresh: bool = False          # hand_telemetry stamp within the staleness window —
                                      # a dead hand link blinds release verification
    hand_parked: bool = False         # hand telemetry position within the BOTTOM park
                                      # band (catch-rest = stroke bottom = 0 rev; reload's
                                      # park-band qualifiers + noise thresholds). Kind-0
                                      # strokes command absolute positions from 0 rev —
                                      # a throw dispatched off-band is a physical hazard,
                                      # so it gates CHECKING and is RE-VERIFIED at
                                      # THROWING entry (ABORTED_HAND_NOT_PARKED).
    ball_seated: bool = False         # THE ball-evidence precondition, as a single
                                      # boolean so there is one truth. Since 2026-08-10
                                      # the node builds it from the LIVE hand ball
                                      # sensor (C-POSSESS-1 § 3.3), not from the sticky
                                      # possession latch: a ball that bounced out during
                                      # a TossContinuous dwell used to leave the latch
                                      # standing and cycle N+1 fired an empty stroke.
                                      # The trace-only waiver still forces it True; the
                                      # waiver never covers hand_fresh / hand_parked,
                                      # which stay hard. Default False = fail-closed.
    ball_evidence: str = EVIDENCE_UNKNOWN
                                      # WHY ball_seated reads as it does — the live cup
                                      # state (ball_possession.EVIDENCE_*). It refines
                                      # only the REJECT CODE and nothing branches on it
                                      # otherwise: an operator who reads NO_BALL goes
                                      # hunting for a ball, where an UNKNOWN sensor is
                                      # a sensor fault, and sending them to the wrong
                                      # subsystem costs a sitting.
    track_active: bool = False        # any LIVE track on `balls` still destined for
                                      # 'jugglebot' (in-flight or pending expectation —
                                      # NOT the seated ball's own pruned/CAUGHT track):
                                      # a phantom would correlate against OUR
                                      # announcement, so CHECKING refuses.
    platform_at_target: bool = False  # node-computed: measured platform-body position
                                      # (config-keyed QTM body, platform_start frame —
                                      # the frame non-base bodies publish in) within
                                      # tolerance of the nominated catch pose. ALWAYS
                                      # fed True when the cross-check body is
                                      # unconfigured (the shipped default — no platform
                                      # body has been validated live), so arrival then
                                      # rests on the go_to_pose accept + timed wait.
                                      # Consulted only in POSITIONING after the timed
                                      # arrival — when enabled it catches the silent
                                      # no-op move class.
    staged_site_ok: bool = False      # node-computed, STAGED SLOT ONLY: is the LIVE
                                      # commanded platform pose still the pose this
                                      # cycle's throw site was NOMINATED for, within
                                      # the census-B1 tolerances
                                      # (_TOSS_ALREADY_THERE_TOL_MM / _RAD)?
                                      #
                                      # THE honest-cache contract (2026-08-28). On the
                                      # SERIAL path the nomination makes itself true:
                                      # POSITIONING COMMANDS the platform to the
                                      # pre-tilt pose derived from A and CHECKING waits
                                      # for that arrival, so "the platform is at A at
                                      # release" holds BY CONSTRUCTION. A STAGED cycle
                                      # is skip-only by construction (§ 2.4.1) — it
                                      # commands nothing — so it cannot make its
                                      # nomination true; it can only nominate what will
                                      # be true, while the cycle AHEAD of it moves the
                                      # platform with its deferred A->B reach in
                                      # between. So the nomination is re-validated at
                                      # the COMMIT tick, from the SAME snapshot the
                                      # evidence gates read, because that is the
                                      # instant the throw becomes irrevocable. A
                                      # mismatch is REJECTED_SITE_MOVED: the staged
                                      # slot is dropped and the cycle is rebuilt on the
                                      # SERIAL path, where the nomination is read live
                                      # and POSITIONING commands the move that makes it
                                      # true (bag 2026-08-28_14-48-38 — a displaced
                                      # chain threw from B with an aim solved for A and
                                      # landed the ball at home, x=+3.98 mm against a
                                      # +70 mm target).
                                      #
                                      # Default False = FAIL-CLOSED, the same doctrine
                                      # as platform_levelled / throw_site_known: an FSM
                                      # that was never told is not entitled to assume.
                                      # Read ONLY by _step_committing, which only a
                                      # staged cycle ever reaches, so the serial path's
                                      # decision stream is untouched by the default.
    throw_stroke_seen: bool = False   # node-latched after dispatch: hand telemetry
                                      # ascending-stroke signature (vel over threshold,
                                      # pos above seated). Sticky for the goal.
    ball_track_confirmed: bool = False  # latched announced-ball id has tracking ==
                                      # CONFIRMED on `balls` — physical airborne
                                      # evidence (status IN_FLIGHT is time-based and
                                      # proves nothing).
    ball_caught: bool = False         # THE possession verdict — the ball-in-cup
                                      # sensor's, and ONLY its, since 2026-08-26
                                      # (owner decision D1). It used to be "a tracker
                                      # CAUGHT for the latched id, plausibility-gated",
                                      # which made the mocap tracker the primary
                                      # source: a track it never confirmed produced no
                                      # verdict at all. The tracker keeps every other
                                      # role (landing announcements, aim/ILC channels,
                                      # ball tracking) — only possession changed.
    possession_blind: bool = False    # the verdict was UNKNOWN because the sensor
                                      # COULD NOT LOOK (ball_possession.arrival_blind),
                                      # not because its window was still open. Drives
                                      # MISSED_SENSOR_BLIND at the settle terminal.
    catch_error_mm: float = float('nan')  # hypot(ball xy − nominated landing xy) at the
                                      # CAUGHT tick; NaN otherwise — and NaN is now the
                                      # HONEST reading on a catch the tracker never
                                      # saw, which the 2026-08-26 census showed is a
                                      # large minority of real catches
    catch_event_dt_s: float = float('nan')
                                      # HAND-SENSOR catch-event time: the ball's observed
                                      # arrival edge in the cup MINUS the predicted
                                      # landing. NaN until an edge lands in the window.
                                      # Not the same event as a tracker CAUGHT (which
                                      # stamps when the marker VANISHED) and not the same
                                      # as achieved_flight_s (which is derived from the
                                      # tracker's landing-plane crossing). REPORTED, never
                                      # branched on — it is the Phase-2 learning loop's
                                      # per-toss timing record.
    ball_time_at_land_perf: float = float('nan')
                                      # live time_at_land of the latched ball (ROS→perf
                                      # crossed by the node), refreshed while in flight —
                                      # feeds achieved_flight_s
    leg_vel_limit_mmps: float = 0.0   # LIVE session leg limits from a FRESH
    leg_acc_limit_mmps2: float = 0.0  # trajectory/status — what the feasibility
    leg_jerk_limit_mmps3: float = 0.0
                                      # gate is enforcing RIGHT NOW (YAML working
                                      # point as ramped by trajectory/set_limits,
                                      # clamped to the YAML ceilings). Consumed by
                                      # the CHECKING displacement gate's closed-form
                                      # reach bound so the pre-throw verdict judges
                                      # against the SAME limits the planner will
                                      # enforce at t_release. 0.0 = unknown
                                      # (pre-field publisher, STALE status, bag
                                      # replay): the gate falls back to the module's
                                      # YAML-default copies — the pre-2026-08-14
                                      # behaviour, NOT a refusal, because the bound
                                      # is a loud-early convenience and the planner
                                      # remains the truth (an over-permissive
                                      # fallback surfaces as the planner's own
                                      # refusal at dispatch, never as motion).


@dataclass
class TossResult:
    success: bool
    outcome: str
    catch_error_mm: float = float('nan')
    achieved_flight_s: float = float('nan')
    catch_event_dt_s: float = float('nan')   # see TossObservations.catch_event_dt_s


@dataclass
class TossDecision:
    phase: str
    action: str = ACTION_NONE
    done: bool = False
    result: Optional[TossResult] = None
    action_then: str = ACTION_NONE
    #: A SECOND action the node must execute on THIS tick, immediately after
    #: ``action`` and in that order. Exactly one producer, ever: the COMMIT gate
    #: (:meth:`TossSequencer._step_committing`), which publishes the
    #: announcement and dispatches the throw **in one tick** because a slipped
    #: release invalidates an already-published announcement and there is no
    #: withdrawal message on the wire (plan § 2.4.2). ``ACTION_NONE`` — the
    #: default every other construction keeps — means "nothing follows", so the
    #: serial path's decision stream is bit-unchanged.
    #:
    #: A tuple would have been the general shape; a second named field is the
    #: honest one, because the general shape does not exist: no other transition
    #: emits two actions, and a container invites one to.
    slip: bool = False
    #: The COMMIT gate deferred to the next loop iteration (plan § 2.4.3). Not an
    #: action — a SLIP commands nothing; it re-arms the commit and moves
    #: ``_t_release`` with it. Reported rather than hidden so the operator scores
    #: ``commit_slip_s`` instead of inferring it, and so a slip that runs away is
    #: visible before its bound (``catch_confirm_window_s``) converts it into a
    #: refusal.


@dataclass
class TossSequencer:
    """The toss FSM. Construct with the resolved goal parameters, then
    ``start(now)`` and drive with ``step(now, obs)``; feed the discrete async
    events via ``note_position_result`` / ``note_prepare_result`` /
    ``note_announcement`` / ``note_throw_dispatch`` / ``note_catch_feasibility``."""

    catch_pose_stow_mm: tuple                   # (x, y, z) PLATFORM POSE at catch,
                                                # STOW-relative (TimedTarget.pose
                                                # convention; z 170 = ACTIVE plane)
    flight_time_s: float = 0.0                  # 0 => DEFAULT_TOSS_FLIGHT_TIME_S
    throw_delay_s: float = 0.0                  # 0 => DEFAULT_TOSS_THROW_DELAY_S
    release_at_perf: float = 0.0                # THE ABSOLUTE scheduled release on
                                                # the perf clock, and THE seam a
                                                # beat clock drops in through
                                                # (pipelined-preamble plan § 2.6).
                                                #
                                                # 0.0 ⇒ derive it at start(now) as
                                                # `now + throw_delay_s`, which is
                                                # bit-for-bit today's behaviour and
                                                # what every shipped caller passes.
                                                # A non-zero value is taken
                                                # VERBATIM: the cycle no longer
                                                # decides WHEN it releases, it is
                                                # TOLD — so a scheduler that moves
                                                # the release moves the guard, the
                                                # 8b reach, the settle deadline and
                                                # the landing with it, for free,
                                                # because all of them read
                                                # _t_release and none re-derives it.
                                                #
                                                # Sentinel doctrine, the same one
                                                # flight_time_s / throw_delay_s use:
                                                # 0.0 is the ONLY "unset" value, and
                                                # a negative (or non-finite) one is
                                                # PRESERVED so CHECKING refuses it
                                                # loudly (REJECTED_RELEASE_SCHEDULE)
                                                # rather than laundering a sign typo
                                                # into a release the operator never
                                                # asked for. The substitution cannot
                                                # happen in __post_init__ like the
                                                # other two — resolving it needs
                                                # `now`, which only start() has.
                                                #
                                                # ⚠ CHECKING's accept-time floor
                                                # (min_throw_delay_for_cycle_s) is
                                                # still charged against
                                                # throw_delay_s, so a caller that
                                                # supplies an absolute release is
                                                # itself responsible for that
                                                # release clearing the pre-dispatch
                                                # budget measured from start(now).
                                                # Phase B4's COMMIT gate is what
                                                # charges the absolute lead; until
                                                # it exists nothing in-tree passes
                                                # a non-zero value.
    staged: bool = False                        # B4: run the PIPELINED ladder —
                                                # CHECKING(STATIC gates only) →
                                                # POSITIONING(skip) → PREPARING →
                                                # STAGED → COMMITTING — instead of
                                                # the serial one. The node sets it
                                                # True only for a cycle it is
                                                # putting in the STAGED slot while
                                                # another cycle owns the hand.
                                                #
                                                # ⚠ SKIP-ONLY, and enforced here
                                                # rather than trusted from the
                                                # caller: __post_init__ forces it
                                                # back to False whenever
                                                # positioning_move_expected is
                                                # True. A staged cycle may not
                                                # command a go_to_pose — it would
                                                # move the platform during the
                                                # PREVIOUS cycle's flight, under a
                                                # ball the catch is armed for
                                                # (plan § 2.4.1) — so a cycle that
                                                # must move simply does not stage
                                                # and takes the serial path. The
                                                # node declines to stage it for the
                                                # same reason; this is the belt, so
                                                # the two cannot disagree in the
                                                # dangerous direction.
                                                #
                                                # Default False = the serial
                                                # ladder, bit-for-bit the pre-B4
                                                # FSM, which is what makes
                                                # `toss_pipeline_enabled: false`
                                                # a decision-stream identity.
    tier: str = TIER_8A                         # config-resolved (JB_OP_TOSS_TIER)
    event_vel_mps: float = 0.0                  # 0 => computed from the Tier-8a vertical
                                                # closed form; the node normally passes
                                                # motion/toss_release's value in
    positioning_timeout_s: float = TOSS_POSITIONING_TIMEOUT_S
    release_grace_s: float = TOSS_RELEASE_GRACE_S
    catch_confirm_window_s: float = CATCH_CONFIRM_WINDOW_S
    min_throw_delay_s: float = TOSS_DISPATCH_DEBOUNCE_S
    min_event_delay_s: float = MIN_THROW_EVENT_DELAY_S
                                                # 0 (the shipped default) ⇒ the
                                                # DERIVED per-speed floor; see
                                                # min_event_delay_for_throw_s.
    positioning_move_expected: bool = True      # will POSITIONING COMMAND a move,
                                                # or take the census-B1 no-op skip?
                                                # It is the single input that sets
                                                # the pre-dispatch budget the
                                                # CHECKING delay gate charges
                                                # (0.520 s vs 0.160 s), so getting
                                                # it wrong in the optimistic
                                                # direction re-opens the exact hole
                                                # the 2026-08-22 audit found.
                                                #
                                                # Default TRUE = FAIL-CLOSED, the
                                                # same doctrine as
                                                # throw_site_known: an FSM that was
                                                # never told assumes it must pay
                                                # for the move. The node evaluates
                                                # the predicate ONCE per cycle (in
                                                # _build_toss_cycle) and reuses the
                                                # SAME cached answer to drive
                                                # POSITIONING, so the budget this
                                                # gate charges and the branch the
                                                # node takes cannot disagree.
    throw_site_xy_mm: tuple = (0.0, 0.0)        # Tier 8b: throw site A (STOW xy).
                                                # The node sources it from the
                                                # platform's LIVE commanded pose
                                                # (trajectory/commanded_position),
                                                # never from config — see
                                                # throw_site_known. Ignored for 8a
                                                # (throw site == catch site by
                                                # definition).
    throw_site_known: bool = False              # Tier 8b: False ⇒ the caller could
                                                # not read a FRESH commanded
                                                # platform pose, so A is unknown
                                                # and CHECKING mints
                                                # REJECTED_POSE_UNKNOWN. 8a never
                                                # consults it.
                                                #
                                                # Default False = FAIL-CLOSED, the
                                                # same doctrine as platform_levelled
                                                # and reload's platform_centered:
                                                # an FSM that was never told is not
                                                # entitled to assume. This default
                                                # is load-bearing, not cosmetic —
                                                # it pairs with throw_site_xy_mm's
                                                # (0.0, 0.0), so a caller that
                                                # omits BOTH would otherwise site
                                                # the throw at the workspace
                                                # ORIGIN and be believed. That is
                                                # the exact phantom site this phase
                                                # retired the config key to kill,
                                                # and it is not merely a wrong
                                                # number: POSITIONING would
                                                # TRANSLATE the platform to it
                                                # before throwing, so the guess
                                                # becomes commanded motion the
                                                # operator never asked for. The
                                                # node passes the flag explicitly
                                                # on BOTH branches.
    max_displacement_mm: float = TOSS_MAX_DISPLACEMENT_MM
                                                # Tier 8b |B−A| cap; the node
                                                # passes hw.JB_OP_TOSS_MAX_
                                                # DISPLACEMENT_MM. The YAML key
                                                # is OPERATOR-ADJUSTABLE (2026-08-14):
                                                # the module literal is only the
                                                # no-config fallback, and the test
                                                # pins the MECHANISM (ctor value
                                                # is what gates), not equality.
    workspace_xy_mm: float = TOSS_XY_LIMIT_MM   # |x|,|y| planning-envelope bound
                                                # on B (and, 8b, on A). The node
                                                # passes hw.JB_OP_TOSS_WORKSPACE_
                                                # XY_MM (YAML toss_workspace_xy_mm,
                                                # operator-adjustable); the module
                                                # literal is the no-config fallback.
                                                # POLICY, not physics: the planner's
                                                # feasibility gate + the firmware
                                                # stroke clamp remain the truth.
    stay_at_pose_on_caught: bool = True         # CAUGHT terminal: True ⇒
                                                # ACTION_STAY (hold the catch
                                                # pose so the next toss can throw
                                                # from it — session chaining),
                                                # False ⇒ the pre-2026-07-29
                                                # ACTION_RECENTER (go_home).
                                                # Node-resolved from
                                                # hw.JB_OP_TOSS_STAY_AT_POSE_ON_
                                                # CAUGHT. NOT-caught terminals are
                                                # unaffected in either setting.
    tilt_clamp_exceeded: bool = False           # BOTH tiers: node-fed flag — the
                                                # authoritative clamp gate lives in
                                                # motion/toss_release (compute_
                                                # release_state_tilted raises
                                                # ThrowTiltInfeasible); the node
                                                # maps the raise onto this flag so
                                                # CHECKING mints REJECTED_TILT_CLAMP
                                                # without a drift-prone second copy
                                                # of the aim math here. Two
                                                # sources: a displaced 8b goal
                                                # near the 12 deg ceiling, and
                                                # (either tier) an aim-corrected
                                                # virtual target — hence not
                                                # gated on the tier
    tilt_required_deg: float = float('nan')     # the two numbers ThrowTiltInfeasible
    tilt_max_deg: float = float('nan')          # carries, node-fed alongside the flag
                                                # above so REJECTED_TILT_CLAMP can NAME
                                                # the aim it needed and the ceiling it
                                                # broke. NaN ⇒ the node did not say
                                                # (older caller, hand-built FSM), and
                                                # the refusal degrades to the bare code
                                                # rather than inventing a number. Kept
                                                # as floats, not the exception's prose:
                                                # the clamp math stays in motion/, and
                                                # a number is what a test can assert.

    # ── internal state ──
    _phase: str = field(default=PHASE_CHECKING, init=False)
    _t_accept: float = field(default=0.0, init=False)
    _t_release: float = field(default=0.0, init=False)   # _t_accept + throw_delay (perf)
    _position_dispatched: bool = field(default=False, init=False)
    _position_result: Optional[tuple] = field(default=None, init=False)  # (ok, dur, code)
    #: The service's own ``message`` for a REFUSED go_to_pose, kept OUT of the
    #: tuple above on purpose: `code` is machine-readable (the BUSY re-poll keys
    #: on it, and the record's `position_code` column is it), while this is the
    #: prose that tells the operator WHY — so the two never get confused for one
    #: another. Empty ⇒ REJECTED_POSITION(<code>) exactly as before.
    _position_message: str = field(default='', init=False)
    _position_arrival_time: float = field(default=0.0, init=False)
    _positioning_deadline: float = field(default=0.0, init=False)
    #: The BUSY re-poll window (:data:`TOSS_POSITION_BUSY_PATIENCE_S`), stamped at
    #: EVERY ``ACTION_POSITION_PLATFORM`` dispatch alongside
    #: ``_positioning_deadline`` so the two are always anchored to the same
    #: instant, and the instant a re-poll spacing is next satisfied at.
    _position_busy_deadline: float = field(default=0.0, init=False)
    _position_busy_next_poll: float = field(default=0.0, init=False)
    #: Forensics for the absorbed wait: when the first BUSY was SEEN (0.0 = never),
    #: how long the absorb has run, and how many re-polls it cost. The pair says
    #: the same two things ``slip_s``/``commit_slips`` say about the commit gate —
    #: HOW LONG and HOW MANY — and it is what makes a settle-hold wait
    #: distinguishable from a wedge after the fact.
    _position_busy_seen_at: float = field(default=0.0, init=False)
    _position_busy_wait_s: float = field(default=0.0, init=False)
    _position_busy_polls: int = field(default=0, init=False)
    _positioned: bool = field(default=False, init=False)  # move ACCEPTED (cleanup marker)
    _prepare_dispatched: bool = field(default=False, init=False)
    _prepare_result: Optional[bool] = field(default=None, init=False)
    #: A refusal code the node attached to a FAILED PREPARE, so the terminal
    #: NAMES the refusal (``REJECTED_<code>``) instead of laundering every
    #: bundle failure into ``ABORTED_PREPARE_FAILED``. Empty ⇒ the generic
    #: abort. Today's one producer is the S6 reach-centre drift guard.
    _prepare_reject_code: str = field(default='', init=False)
    #: The numbers behind that code, when the node has them (the drift guard's
    #: nominated B, the declared centre and the distance between them). Same
    #: split as `_position_message`: the CODE routes, the message explains.
    _prepare_reject_message: str = field(default='', init=False)
    _announce_dispatched: bool = field(default=False, init=False)
    _announced: bool = field(default=False, init=False)   # node confirmed the publish
    _announce_lead_short: bool = field(default=False, init=False)  # WARN flag (Tier 8a)
    _throw_dispatched: bool = field(default=False, init=False)
    _throw_dispatch_result: Optional[tuple] = field(default=None, init=False)  # (outcome, msg)
    _reach_dispatched: bool = field(default=False, init=False)  # Tier 8b: the ONE
                                                #   deferred A→B reach went out
                                                #   (commitment flag, never reset)
    _release_deadline: float = field(default=0.0, init=False)
    _settle_deadline: float = field(default=0.0, init=False)
    _catch_infeasible: Optional[str] = field(default=None, init=False)  # gate code
    _catch_accepted: bool = field(default=False, init=False)  # any target accepted this flight
    _track_confirmed_seen: bool = field(default=False, init=False)
    _last_time_at_land: float = field(default=float('nan'), init=False)
    _commit_slip_s: float = field(default=0.0, init=False)   # see slip_s
    #: How many times the COMMIT gate re-armed (:meth:`_slip`). ``slip_s``
    #: measures HOW LATE; this measures HOW MANY RETRIES, and the pair
    #: distinguishes one unlucky iteration from a loop that is chronically over
    #: period. Read by :meth:`_commit_forensics` and by :attr:`commit_slips`.
    _commit_slips: int = field(default=0, init=False)
    #: The previous :meth:`step` instant, and the interval since it. The FSM's
    #: own measurement of the loop period it is being ticked at — the quantity
    #: :func:`commit_budget_s` charges ONE of and the late-tick slip source is a
    #: function of. Instrument only (it appears in :meth:`_commit_forensics` and
    #: nowhere else): a budget that re-derived itself from observed slowness
    #: would TRACK a degradation instead of exposing it, which is the constraint
    #: ``test_the_census_never_feeds_a_budget`` exists to keep.
    _prev_step_now: float = field(default=float('nan'), init=False)
    _last_tick_s: float = field(default=float('nan'), init=False)
    #: B4 — the COMMIT instant, ``_t_release − commit_budget_s`` (§ 2.6 rule 2:
    #: the commit is derived FROM the release, never the reverse, so a scheduler
    #: that moves the release moves the commit for free). ``_commit_at`` is
    #: re-armed by every SLIP; ``_commit_at_sched`` keeps the ORIGINAL, which is
    #: what :attr:`slip_s` is measured against and what the slip's
    #: ``catch_confirm_window_s`` bound is counted from.
    _commit_at: float = field(default=0.0, init=False)
    _commit_at_sched: float = field(default=0.0, init=False)
    _staged_at: float = field(default=0.0, init=False)     # entered PHASE_STAGED
    _committed: bool = field(default=False, init=False)    # passed the COMMIT gate
    #: The node's answer to "has the UPSTREAM cycle terminalised?" (S1′: at most
    #: one cycle past COMMIT). Set once, by :meth:`note_upstream_terminalised`;
    #: until then the commit gate SLIPS. Meaningless — and never read — on the
    #: serial path, where nothing is upstream.
    _upstream_clear: bool = field(default=False, init=False)
    _finished: bool = field(default=False, init=False)
    _result: Optional[TossResult] = field(default=None, init=False)

    def __post_init__(self):
        # Default-substitution ONLY on exactly 0.0 (the goal's "unset" sentinel).
        # A NEGATIVE value is preserved so CHECKING rejects it loudly
        # (REJECTED_FLIGHT_TIME / REJECTED_CANT_MAKE_LEAD) — silently coercing a
        # sign error into the default would throw a physically different toss.
        # (Since 2026-08-18 REJECTED_FLIGHT_TIME means exactly this: the flight
        # time is not a positive finite number. An in-range-but-unflyable T is
        # REJECTED_THROW_ENVELOPE, which names the bound it broke.)
        if self.flight_time_s == 0.0:
            self.flight_time_s = DEFAULT_TOSS_FLIGHT_TIME_S
        if self.throw_delay_s == 0.0:
            self.throw_delay_s = DEFAULT_TOSS_THROW_DELAY_S
        if self.event_vel_mps <= 0.0:
            self.event_vel_mps = vertical_event_vel_mps(self.flight_time_s)
        # SKIP-ONLY staging (plan § 2.4.1), enforced rather than trusted. See the
        # `staged` field's comment: a cycle that must COMMAND its pre-positioning
        # move cannot stage, because the move would traverse the platform during
        # the previous cycle's flight, under a ball the catch is armed for. The
        # node takes the same decision from the same predicate one layer up; this
        # is the belt, and it fails in the SAFE direction (serial, not staged).
        if self.staged and self.positioning_move_expected:
            self.staged = False
        # `release_at_perf` is deliberately NOT resolved here: its default is
        # `now + throw_delay_s` and `now` does not exist until start(). Its
        # sentinel is consumed there, and the loud refusal for a value that is
        # not a usable instant lives with the other static goal invalids in
        # CHECKING (REJECTED_RELEASE_SCHEDULE) — same split as flight_time_s,
        # whose default lands here and whose sign typo dies at the gate.

    # ── derived floors ─────────────────────────────────────────────────────────

    @property
    def min_event_delay_for_throw_s(self) -> float:
        """The event-delay floor THIS toss's release speed actually needs.

        ``min_event_delay_s > 0`` is an explicit override (tests, standalone
        harnesses); the shipped 0.0 derives it from
        ``hand_stroke.min_throw_event_delay_s(event_vel_mps)`` — the Teensy's own
        ``:642`` budget written from the throw side, prelude(park band) +
        SAFETY_GAP + windup(v).

        ``event_vel_mps`` is resolved in ``__post_init__`` and is therefore always
        finite here, but it is not yet TRUSTWORTHY at that point: CHECKING's
        EVENT_VEL / THROW_ENVELOPE / TILT_CLAMP gates are what establish that.
        ``hand_stroke`` clamps to the bridge's accepted band internally, so an
        out-of-band value yields a bounded floor rather than a divide-by-zero —
        the goal still dies at the gate that names the real fault."""
        override = float(self.min_event_delay_s)
        if override > 0.0:
            return override
        return hand_stroke.min_throw_event_delay_s(self.event_vel_mps)

    @property
    def commit_budget_for_cycle_s(self) -> float:
        """This cycle's COMMIT → release budget (:func:`commit_budget_s` at its
        own release speed), i.e. how far ahead of ``_t_release`` the arm point
        sits. Read by :meth:`start` and by the SLIP path, which is why it is a
        property and not a call at each site: a slip re-derives the release from
        it, and two derivations of a schedule offset is how a schedule and its
        gate drift apart."""
        return commit_budget_s(self.event_vel_mps, float(self.min_event_delay_s))

    @property
    def min_stage_lead_for_cycle_s(self) -> float:
        """The ACCEPT-time lead floor for THIS cycle if it stages
        (:func:`min_stage_lead_for_release_s`): the staged preamble plus the
        commit budget. Charged at CHECKING against the REAL lead
        ``_t_release − now``, which is the quantity a staged cycle actually runs
        on — see the function for why it replaces
        :attr:`min_throw_delay_for_cycle_s` rather than joining it."""
        return min_stage_lead_for_release_s(self.event_vel_mps,
                                            float(self.min_event_delay_s))

    @property
    def min_throw_delay_for_cycle_s(self) -> float:
        """The ACCEPT-time ``throw_delay_s`` floor for this cycle, whole sequence.

        :attr:`min_event_delay_for_throw_s` is what the RUNTIME guard measures the
        remaining lead against; this is that same budget PLUS the lead the
        pre-dispatch sequence spends before the guard runs
        (:func:`pre_dispatch_budget_s`, keyed on
        :attr:`positioning_move_expected`).  Clearing this at CHECKING is what
        makes ``ABORTED_CANT_MAKE_RELEASE`` unreachable from static arithmetic —
        loud and early, with nothing moved, instead of at
        ``cycle_start + 0.06 s`` with the hand already committed."""
        return min_throw_delay_for_release_s(
            self.event_vel_mps, bool(self.positioning_move_expected),
            float(self.min_event_delay_s))

    # ── discrete async events (from the node) ──────────────────────────────────

    def note_position_result(self, now: float, accepted: bool,
                             planned_duration_s: float, code: str = '',
                             message: str = '') -> None:
        """Report the ``go_to_pose`` service outcome. ``now`` anchors the timed
        arrival (``now + planned_duration_s + settle pad``) — the service returns
        at plan-install, so the plan clock starts at the response, not at the tick
        that eventually consumes it. On accept the positioning deadline is
        extended to cover the arrival + the mocap verification window (it never
        shrinks). First result wins; ignored unless the move was dispatched.

        ``code`` and ``message`` are the service response's two halves and stay
        SEPARATE all the way to the terminal. The code is the machine-readable
        one — ``_step_positioning``'s BUSY re-poll matches on it exactly, and it
        is what the record's ``position_code`` column stores — so it must never
        pick up prose. The message is what trajectory_node said about the refusal
        ("a move is in flight …", the workspace bound it broke) and, until
        2026-08-29, the node DISCARDED it: the operator got
        ``REJECTED_POSITION(BUSY)`` and had to go find a log line for the rest.
        It now rides the outcome as ``REJECTED_POSITION(<code>: <message>)``,
        and an empty message leaves the string byte-identical to before."""
        if (not self._position_dispatched or self._finished
                or self._position_result is not None):
            return
        self._position_result = (bool(accepted), float(planned_duration_s), str(code))
        self._position_message = str(message or '')
        if accepted:
            self._positioned = True
            self._position_arrival_time = (
                now + float(planned_duration_s) + TOSS_POSITION_SETTLE_PAD_S)
            self._positioning_deadline = max(
                self._positioning_deadline,
                self._position_arrival_time + TOSS_POSITION_VERIFY_WINDOW_S)

    def note_position_noop(self, now: float) -> None:
        """The node determined the platform is ALREADY at the positioning pose
        and commanded nothing (census B1). Declare arrival immediately.

        **What this is not.** It is not "skip POSITIONING". The phase still runs,
        the reach-envelope declaration still happens at PREPARE (contract
        C-REACH-1), and the mocap arrival cross-check still gets its window — the
        FSM path below this point is byte-identical to an accepted move. What is
        skipped is a ``go_to_pose`` service round trip and the wait for a move
        that would traverse **zero millimetres**.

        **Why the settle pad goes too.** ``TOSS_POSITION_SETTLE_PAD_S`` (0.20 s)
        pads a PLANNED move's arrival for 5 Hz status granularity and
        terminal-hold ENTRY jitter. With nothing commanded there is no plan to
        grant granularity to and no terminal hold to enter — the platform is
        already IN one, holding the pose the caller just matched against the live
        commanded position. Keeping the pad here would be padding an event that
        does not occur. Together with the skipped ``min_move_duration_s`` floor
        (0.20 s) that is the whole 0.45 s census row B1 costs an 8a chain, every
        cycle, to hold still.

        **The safety of the claim lives at the CALLER**, deliberately: only the
        node can compare the requested pose against a FRESH
        ``trajectory/commanded_pose`` — position AND orientation, from one plan
        sample, in the INTENT frame the node's own go_to_pose request speaks. The
        FSM cannot check that, so it does not pretend to: it records what it was
        told, with the ``ALREADY_THERE`` code on the result so the outcome line
        and the toss record name which branch ran.

        (Until 2026-08-23 only ``trajectory/commanded_position`` existed, three
        of the six pose components, so the caller ALSO had to know the release
        was LEVEL — a tilted pre-tilt pose was unverifiable and always commanded.
        That refusal was correct and cost 0.36 s of throw delay on every cycle of
        every aimed sitting.)

        First result wins, and it is ignored unless the move was dispatched —
        the same guards as :meth:`note_position_result`, because this IS a
        position result; it just has a duration of zero and no service call."""
        if (not self._position_dispatched or self._finished
                or self._position_result is not None):
            return
        self._position_result = (True, 0.0, 'ALREADY_THERE')
        self._positioned = True
        self._position_arrival_time = float(now)
        self._positioning_deadline = max(
            self._positioning_deadline,
            self._position_arrival_time + TOSS_POSITION_VERIFY_WINDOW_S)

    def note_prepare_result(self, ok: bool, reject_code: str = '',
                            message: str = '') -> None:
        """Report the ``ACTION_PREPARE_CATCH`` bundle outcome. A failure aborts
        BEFORE the announcement and the throw — the arming-before-throw ordering.

        **The bundle's contents narrowed on 2026-08-27 (S6).** For a single
        ``Toss`` this is still the ``trajectory/arm_catch`` raise result. For a
        cycle of a chained ``TossContinuous`` session the latch was raised ONCE,
        at session scope, before cycle 1's PREPARE — so ``ok`` there reports the
        SESSION arm on the first cycle and, on every later cycle, only that the
        per-cycle remainder (``catch/prime_dispatched``, ``catch/armed``, the
        phantom-flight snapshot) was admitted. Either way the FSM's contract is
        unchanged: no announcement and no throw until this says yes.

        ``reject_code``, when non-empty, names a refusal the node wants carried
        to the terminal as ``REJECTED_<code>`` rather than as the generic
        ``ABORTED_PREPARE_FAILED``. It is only read when ``ok`` is False. Today's
        one producer is the S6 reach-centre drift guard
        (``REACH_CENTER_DRIFT``): the session's ONE ``catch/reach_center``
        declaration is consumed at its ONE raise, so a cycle nominating a B that
        has left that envelope cannot be flown, and the distinction matters
        because a REJECTED names an operator-visible refusal while an ABORTED
        reads as a plant fault.

        ``message`` carries that refusal's NUMBERS to the terminal — for the
        drift guard, the B this cycle nominated, the centre the session declared
        and the tolerance between them, which until 2026-08-29 existed only in a
        node ERROR line the action result never saw."""
        if self._prepare_dispatched and not self._finished:
            self._prepare_result = bool(ok)
            self._prepare_reject_code = str(reject_code or '')
            self._prepare_reject_message = str(message or '')

    def note_upstream_terminalised(self) -> None:
        """S1′'s other half: the node reports that the cycle AHEAD of this one
        has reached its terminal, so the committed slot is free and this cycle's
        COMMIT gate may pass.

        Until it is called the gate SLIPS — it does not refuse — because "the
        ball is not in the cup yet" is a cadence fact on a healthy machine, not a
        machine fault (plan § 9.2, *slip rather than refuse*). The slip is bounded
        by :attr:`catch_confirm_window_s`, which is when the upstream cycle
        terminalises at the LATEST, so this hook introduces no new constant and
        no new way to wedge.

        One-way and idempotent: a cycle whose upstream has terminalised cannot
        un-terminalise it. Never called on the serial path (nothing is upstream),
        where ``_upstream_clear`` is never read."""
        self._upstream_clear = True

    def note_announcement(self) -> None:
        """Report that the self-``ThrowAnnouncement`` was published (the node calls
        this right after executing ``ACTION_ANNOUNCE``; a topic publish cannot fail
        detectably, so this is sequencing confirmation, not an outcome).

        Gated on the PREPARE having been DISPATCHED, not on the phase or the throw:
        the toss's announcement precedes its throw dispatch by design, so reload's
        ``_throw_sent`` gate transposed here would deadlock the sequence."""
        if self._prepare_dispatched and not self._finished:
            self._announced = True

    def note_throw_dispatch(self, outcome: str, message: str = '') -> None:
        """Report the single throw dispatch's tri-state classification
        (``THROW_DISPATCH_OK`` / ``_AMBIGUOUS`` / ``_REJECTED``). REJECTED is the
        definitive no-arm channel (service unavailable, or a bridge-validation
        raise — no CAN frame exists); OK and AMBIGUOUS both mean "await release
        evidence". First result wins; ignored unless the throw was dispatched."""
        if (self._throw_dispatched and not self._finished
                and self._throw_dispatch_result is None):
            self._throw_dispatch_result = (str(outcome), str(message))

    def note_catch_feasibility(self, accepted: bool, code: str = '') -> None:
        """Report a ``trajectory/target_feedback`` decision for the catch target.

        ``MISSED_INFEASIBLE_<code>`` means *the platform never had a reachable
        catch pose*: reported only when NO target was accepted for this flight and
        a rejection stands. Once ANY target is accepted (the announcement pre-tilt
        or a mid-flight refinement), later rejections say nothing about
        reachability and are NOT latched — the reload third-sitting verdict-bug
        semantics, verbatim.

        Gated on the PREPARE having been DISPATCHED, not on ``_throw_sent`` (the
        deliberate deviation from reload): the pre-tilt target — and its
        acceptance — is triggered by OUR announcement, which precedes the throw
        dispatch in a toss. Reload's throw-gate transposed literally would drop
        the pre-tilt accept deterministically and mint a false MISSED_INFEASIBLE
        on every real toss. The node drops the transient FROZEN / STALE_STATE
        codes before calling."""
        if not self._prepare_dispatched or self._finished:
            return
        if accepted:
            self._catch_accepted = True
            self._catch_infeasible = None
        elif not self._catch_accepted:
            self._catch_infeasible = code or 'INFEASIBLE'

    # ── driver ─────────────────────────────────────────────────────────────────

    def start(self, now: float) -> None:
        """Stamp the accept instant and THE scheduled release, once.

        The release is an INPUT, never a re-derivation (plan § 2.6 rule 1): an
        absolute :attr:`release_at_perf` is taken verbatim and only the 0.0
        sentinel derives ``now + throw_delay_s``. This is the ONLY place
        ``_t_release`` is ever written — every consumer (:meth:`_landing_perf`,
        PREPARING's release-window guard, ``_enter_throwing``'s release
        deadline, the settle deadline, Tier 8b's deferred reach, and node-side
        the announcement, the dispatch's ``event_delay``, the cancel cutoff and
        the record) READS it, so moving the release moves all of them together
        by construction rather than by nine agreeing derivations."""
        self._phase = PHASE_CHECKING
        self._t_accept = now
        self._t_release = self.release_at_perf or (now + self.throw_delay_s)
        # B4, plan § 2.6 rule 2 — the commit instant is DERIVED FROM the release,
        # never the reverse. Stamped here, with `_t_release`, so the two can never
        # be computed from different numbers; the SLIP path re-arms `_commit_at`
        # and moves `_t_release` with it, and `_commit_at_sched` keeps the
        # original so the lateness stays measurable.
        self._commit_at = self._t_release - self.commit_budget_for_cycle_s
        self._commit_at_sched = self._commit_at

    def step(self, now: float, obs: TossObservations) -> TossDecision:
        if self._finished:
            # Terminal: replay the stored result (never None once finished) so a
            # stray extra step can't hand the consumer a None — and never re-runs
            # the terminal action.
            return TossDecision(self._phase, ACTION_NONE, True, self._result)

        # THE FSM's own view of the loop period it is ticked at (instrument
        # only — see `_prev_step_now`). Taken before any phase logic so the
        # commit gate's forensics can name the iteration that broke it, and
        # taken here rather than from `LoopPeriodCensus` so the census stays
        # strictly downstream of every decision this FSM makes.
        if math.isfinite(self._prev_step_now):
            self._last_tick_s = now - self._prev_step_now
        self._prev_step_now = now

        # Live tracker bookkeeping: the confirmation latch gates
        # achieved_flight_s; the landing-crossing estimate refreshes while finite,
        # but only once OUR throw is dispatched — a pre-dispatch (phantom) ball's
        # crossing estimate must never seed the diagnostic (the node resets its
        # announced-ball latch at PREPARE; this is the FSM-side half of that gate).
        if obs.ball_track_confirmed:
            self._track_confirmed_seen = True
        if self._throw_dispatched and math.isfinite(obs.ball_time_at_land_perf):
            self._last_time_at_land = obs.ball_time_at_land_perf

        # ── Universal checks (before the phase logic; order matters) ───────────
        # Static goal invalids first, tier strictest of all (Phase-1 gate): these
        # can only fire on the first pass — CHECKING either terminates or advances.
        if self._phase == PHASE_CHECKING:
            if self.tier not in (TIER_8A, TIER_8B):
                return self._reject('TIER')
            if self.throw_delay_s < self.min_throw_delay_s:
                # The GOAL-STORM debounce, and the sign-typo belt (a negative
                # delay lands here, which is what __post_init__ preserves it
                # for). It is NOT a readiness claim — see
                # TOSS_DISPATCH_DEBOUNCE_S. The physical lead the dispatch needs
                # is checked immediately below, once the flight time (and with
                # it the release speed) has been validated.
                #
                # The message says DEBOUNCE explicitly: this code is minted at
                # three sites and the other two are physical lead floors, so an
                # operator who reads "CANT_MAKE_LEAD" and goes looking at the
                # hand geometry would be at the wrong gate entirely.
                return self._reject('CANT_MAKE_LEAD', bound_msg(
                    'throw_delay', self.throw_delay_s, '<',
                    self.min_throw_delay_s, 's', digits=3,
                    limit_label='goal-storm debounce',
                    knob='TOSS_DISPATCH_DEBOUNCE_S',
                    tail='NOT a readiness floor — the physical lead gate is '
                         'checked further down, once the flight time is valid'))
            if self.release_at_perf != 0.0 and not (
                    math.isfinite(self.release_at_perf)
                    and self.release_at_perf > 0.0):
                # The ABSOLUTE schedule's half of the sentinel doctrine, and the
                # other end of the contract `release_at_perf`'s comment states:
                # 0.0 means "derive", so anything else must be a usable instant
                # on the perf clock. A negative (the preserved sign typo) or a
                # non-finite one would otherwise sail through CHECKING — every
                # gate above it is keyed on throw_delay_s — and then WEDGE the
                # cycle: `_t_release` NaN makes `now >= deadline` false forever,
                # so nothing releases and nothing times out until the node's own
                # ceiling SAFE_ABORTs under a hand that was armed for a throw
                # that never came. Loud and early, with nothing moved, is the
                # same trade FLIGHT_TIME takes. Gated on `!= 0.0` so the shipped
                # derived path costs one float compare and is otherwise
                # bit-identical.
                return self._reject('RELEASE_SCHEDULE')
            if not (math.isfinite(self.flight_time_s)
                    and self.flight_time_s > 0.0):
                # VALIDITY only since 2026-08-18 — the flyable BAND moved to the
                # derived envelope below (C-HAND-3).  A negative flight time is
                # the sign typo __post_init__ deliberately preserves, and a
                # non-positive T makes every downstream ballistic inverse
                # meaningless, so it dies here rather than as a confusing
                # envelope refusal.
                return self._reject('FLIGHT_TIME')
            x, y, z = self.catch_pose_stow_mm
            if self.tier == TIER_8B:
                # Displaced-throw gates (Phase 4), after the flight-time band
                # (the reach bound is meaningless for an out-of-band T) and
                # before EVENT_VEL (a clamp-rejected goal has no valid release
                # state, so its event_vel is the meaningless 8a fallback).
                #
                # STRICTEST FIRST: the throw site must be KNOWN before any
                # gate that measures FROM it. Every displaced gate below is a
                # function of |B − A|, so an unknown A makes all of them
                # meaningless — and the node cannot substitute a guess (see
                # throw_site_known: a guessed A becomes a commanded POSITIONING
                # translation). REJECTED_POSE_UNKNOWN is the honest verdict and
                # routes the operator at the trajectory link, not at the goal.
                if not self.throw_site_known:
                    return self._reject('POSE_UNKNOWN')
                # Then the cap (the primary contract — REQUESTED displacement,
                # config-keyed), then the closed-form quintic reach bound over
                # lead = T — both are loud PRE-THROW verdicts for a reach whose
                # trajectory_node verdict would otherwise arrive only after
                # release, with the ball already airborne.
                ax, ay = self.throw_site_xy_mm
                displacement = math.hypot(x - float(ax), y - float(ay))
                # The reach bound judges against the LIVE session limits when
                # the node observed them on a FRESH trajectory/status (`or
                # None` maps the 0.0 unknown-sentinel to the YAML-default
                # fallback) — so a set_limits ramp moves THIS gate in lockstep
                # with the feasibility gate it fronts for: soft limits refuse
                # here, pre-throw, instead of at t_release with the ball
                # airborne; raised limits stop refusing reaches the planner
                # would happily fly.
                live = (obs.leg_vel_limit_mmps or None,
                        obs.leg_acc_limit_mmps2 or None,
                        obs.leg_jerk_limit_mmps3 or None)
                bound, term = reach_displacement_bound(
                    self.flight_time_s, vel_mmps=live[0], acc_mmps2=live[1],
                    jerk_mmps3=live[2])
                if (displacement > self.max_displacement_mm
                        or displacement > bound):
                    return self._reject(
                        'DISPLACEMENT',
                        self._displacement_detail(displacement, bound, term,
                                                  live))
            if self.tilt_clamp_exceeded:
                # The motion-module clamp gate fired: the required aim exceeds
                # the tilt ceiling, and a silently clamped aim lands the ball
                # short of B (the Rung-2a landing bias).
                #
                # BOTH TIERS, deliberately (audit fix, 2026-08-11). This was
                # inside the 8b block, which was right while the only source of
                # a raise was a displaced goal near the ceiling. The aim
                # correction added a SECOND source that exists in 8a too: the
                # node tilts toward a VIRTUAL target for the aim in either tier
                # (`aim_site` is B for 8a, the live A for 8b), and a raise there
                # sets this flag and leaves `release_cmd` None. With the check
                # 8b-only, an 8a goal in that state fell through to the
                # EVENT_VEL band with the zero fallback and reported
                # REJECTED_EVENT_VEL — fail-closed, but naming the Teensy speed
                # limit for an aim-ceiling fault, which routes the operator to
                # the wrong subsystem entirely.
                #
                # Gate order for 8b is UNCHANGED: still POSE_UNKNOWN →
                # DISPLACEMENT → TILT_CLAMP → EVENT_VEL, because this sits
                # immediately after the block it left
                # (test_displacement_precedes_tilt_clamp pins it).
                #
                # The numbers come from the node (ThrowTiltInfeasible carries
                # them); NaN means it did not say, and the refusal stays bare
                # rather than quoting a ceiling it cannot vouch for.
                if (math.isfinite(self.tilt_required_deg)
                        and math.isfinite(self.tilt_max_deg)):
                    return self._reject('TILT_CLAMP', bound_msg(
                        'aim', self.tilt_required_deg, '>', self.tilt_max_deg,
                        'deg', digits=2, limit_label='ceiling',
                        knob='MAX_TILT_DEG',
                        tail='a clamped aim lands the ball SHORT of B — lower '
                             '|B-A| or raise throw_height_m'))
                return self._reject('TILT_CLAMP')
            if not (TEENSY_MIN_EVENT_VEL_MPS <= self.event_vel_mps
                    <= TEENSY_MAX_EVENT_VEL_MPS):
                return self._reject('EVENT_VEL', range_msg(
                    'event_vel', self.event_vel_mps, TEENSY_MIN_EVENT_VEL_MPS,
                    TEENSY_MAX_EVENT_VEL_MPS, 'm/s', digits=2,
                    knob='TEENSY_MIN/MAX_EVENT_VEL_MPS',
                    tail='the WIRE band — the goal knob is throw_height_m'))
            # THE derived feasibility gate (contract C-HAND-3). It runs HERE,
            # after EVENT_VEL, for the same reason the tilt-clamp gate runs
            # before it: this is the first point at which `event_vel_mps` is
            # TRUSTWORTHY. For 8a it is the vertical ballistic inverse of T; for
            # 8b it is the aim solution, and an aim that hit the tilt ceiling
            # leaves the meaningless 8a fallback behind — refusing THAT against
            # an end-stop model would name the hand for an aiming fault.
            # The wire band goes first so a value the BRIDGE would raise on
            # still reports REJECTED_EVENT_VEL, which routes at the wire.
            #
            # The message carries the binding bound BY NAME plus the computed
            # envelope — `REJECTED_THROW_ENVELOPE(END_STOP:modelled peak 10.660
            # rev at 4.436 m/s exceeds 10.600 rev …)` — because "too high" sends
            # an operator to the wrong knob. It is a DISTINCT code from
            # FLIGHT_TIME for the same reason: an END_STOP refusal is about the
            # hand's coast, not about the clock.
            envelope = throw_envelope.evaluate(self.flight_time_s,
                                               self.event_vel_mps)
            if not envelope.ok:
                return self._reject('THROW_ENVELOPE', envelope.message)
            # The DERIVED lead gate (census A2), placed here for the same reason
            # the envelope is: this is the first point at which `event_vel_mps`
            # is trustworthy, and the floor is a function of it.
            #
            # ⚠ THE WHOLE SEQUENCE, not just its last item (2026-08-23). Until
            # then this gate charged the Teensy's `:642` dispatch budget ALONE
            # and called the rest — the CHECKING tick, the POSITIONING wait, the
            # PREPARE tick ladder — "elapsed time the runtime guard measures for
            # real". It does measure it, and that was the bug: the guard in
            # _step_preparing applies the SAME budget to the lead REMAINING after
            # all of it, so this gate was systematically loose by the entire
            # pre-dispatch cost (0.160 s with the B1 skip, 0.520 s without it).
            # A goal could clear this line by construction and then abort
            # ABORTED_CANT_MAKE_RELEASE every cycle at cycle_start + 0.06 s, with
            # the hand retracting under a seated ball — which is what three
            # published rungs of the cadence ladder did. Both terms now come from
            # ONE derivation (min_throw_delay_for_release_s), which the session's
            # mirror imports too, so the two cannot drift again.
            #
            # This is still the loud+early copy of a runtime truth, which is what
            # the retired MIN_TOSS_THROW_DELAY_S = 3.5 s claimed to be and was
            # not: 3.5 s was a generic fit over a worst-case POSITIONING move
            # that a co-located chain does not make and a 1.0 s event-delay
            # floor sized off a prelude the hand_parked precondition forbids. The
            # difference is that every term here is derived from the constants the
            # sequence is actually made of.
            if self.staged:
                # ── THE PIPELINED lead gate (B4, plan § 2.4.1 / § 2.7) ──
                # A staged cycle does not run on `throw_delay_s`: its release is
                # an ABSOLUTE instant it was TOLD (§ 2.6 rule 1), so the delay
                # floor above would gate a number this cycle ignores. What it
                # actually needs is enough REAL lead to walk the staged preamble
                # and still reach its commit tick with the dispatch budget
                # intact — `min_stage_lead_for_release_s`, charged here against
                # `_t_release − now` with nothing staged and nothing armed.
                #
                # This is what makes ABORTED_CANT_MAKE_RELEASE structurally
                # unreachable on the pipelined path (§ 6.4 stop conditions: one
                # would be a DESIGN finding, not a tuning finding). Retiring the
                # delay gate without replacing it is how the pre-dispatch budget
                # went uncharged in the first place; this replaces it.
                stage_floor = self.min_stage_lead_for_cycle_s
                lead = self._t_release - now
                if lead < stage_floor:
                    return self._reject(
                        'CANT_MAKE_LEAD',
                        'staged lead {:.3f} s is under the {:.3f} s this cycle '
                        'needs at {:.2f} m/s: the kind-0 dispatch budget {:.3f} '
                        'plus one polled loop period {:.3f} (the commit tick) '
                        'plus the {:.3f} s staged preamble'
                        .format(lead, stage_floor, self.event_vel_mps,
                                self.min_event_delay_for_throw_s,
                                NODE_LOOP_PERIOD_S, stage_budget_s()))
            lead_floor = self.min_throw_delay_for_cycle_s
            if not self.staged and self.throw_delay_s < lead_floor:
                dispatch_s = self.min_event_delay_for_throw_s
                return self._reject(
                    'CANT_MAKE_LEAD',
                    'throw_delay {:.3f} s is under the {:.3f} s this cycle '
                    'needs at {:.2f} m/s: the kind-0 dispatch budget {:.3f} '
                    '(prelude {:.3f} + gap {:.3f} + windup {:.3f}) plus the '
                    '{:.3f} s pre-dispatch sequence ({} positioning move)'
                    .format(
                        self.throw_delay_s, lead_floor, self.event_vel_mps,
                        dispatch_s,
                        hand_stroke.smooth_move_duration_s(
                            hand_stroke.HAND_PARK_BAND_REV),
                        hand_stroke.SAFETY_GAP_S,
                        -hand_stroke.HandStrokeModel(
                            self.event_vel_mps).stroke_start_rel,
                        pre_dispatch_budget_s(
                            bool(self.positioning_move_expected)),
                        'with a' if self.positioning_move_expected
                        else 'census-B1 skip, no'))
            # ONE code, THREE messages. The gate is unchanged (same three
            # conjuncts, same x → y → z precedence — the split is the boolean
            # `or` written out so each branch can name ITS bound); what changes
            # is that "outside the workspace" now says which of the lateral box
            # and the z band refused, and quotes the offending component. They
            # move under different knobs (`toss_workspace_xy_mm` vs the
            # ACTIVE-plane band), so the undifferentiated code sent half these
            # refusals to the wrong one.
            if abs(x) > self.workspace_xy_mm:
                return self._reject('WORKSPACE', bound_msg(
                    '|B.x| =', abs(x), '>', self.workspace_xy_mm, 'mm',
                    knob='toss_workspace_xy_mm'))
            if abs(y) > self.workspace_xy_mm:
                return self._reject('WORKSPACE', bound_msg(
                    '|B.y| =', abs(y), '>', self.workspace_xy_mm, 'mm',
                    knob='toss_workspace_xy_mm'))
            if abs(z - TOSS_ACTIVE_Z_MM) > TOSS_Z_BAND_MM:
                return self._reject('WORKSPACE', bound_msg(
                    'B.z {:.1f} mm is'.format(z), abs(z - TOSS_ACTIVE_Z_MM),
                    '>', TOSS_Z_BAND_MM, 'mm', knob='TOSS_Z_BAND_MM',
                    limit_label='band',
                    tail='measured from the ACTIVE plane {:.1f} mm'.format(
                        TOSS_ACTIVE_Z_MM)))
            if self.tier == TIER_8B:
                # The throw site A shares B's z (one nominated plane); its xy
                # gets the same planning-envelope bounds as B. Since A is the
                # platform's LIVE commanded xy (Phase E), this now also refuses
                # a toss commanded while the platform is parked OUTSIDE the
                # planning envelope — a real state a SpaceMouse/GUI session can
                # leave behind, and one where the pre-tilt POSITIONING move
                # would be planned from an unvalidated pose.
                ax, ay = self.throw_site_xy_mm
                for axis, value in (('A.x', float(ax)), ('A.y', float(ay))):
                    if abs(value) > self.workspace_xy_mm:
                        # The variant an operator is most likely to misread: the
                        # goal's B may be perfectly legal and the refusal is
                        # about where the PLATFORM is parked, so the message
                        # names the live throw site and the one-command remedy.
                        return self._reject('WORKSPACE', bound_msg(
                            'live throw site |{}| ='.format(axis), abs(value),
                            '>', self.workspace_xy_mm, 'mm',
                            knob='toss_workspace_xy_mm',
                            tail='the PLATFORM is parked outside the planning '
                                 'box, not the goal — go_home and retry'))
        # TOSS runs within the active streaming mode; leaving it mid-sequence is
        # the documented abort. A mode-exit mid-flight still terminates immediately
        # (as reload): trajectory_node has force-disarmed the latch on the mode
        # change anyway — the catch is dead, and SAFE_ABORT is the honest cleanup.
        if obs.control_mode != TOSS_CONTROL_MODE:
            if self._phase == PHASE_CHECKING:
                return self._reject('WRONG_MODE')
            return self._abort('MODE_CHANGED')

        if self._phase == PHASE_CHECKING:
            return self._step_checking(now, obs)
        if self._phase == PHASE_POSITIONING:
            return self._step_positioning(now, obs)
        if self._phase == PHASE_PREPARING:
            return self._step_preparing(now, obs)
        if self._phase == PHASE_STAGED:
            return self._step_staged(now, obs)
        if self._phase == PHASE_COMMITTING:
            return self._step_committing(now, obs)
        if self._phase == PHASE_THROWING:
            return self._step_throwing(now, obs)
        if self._phase in (PHASE_BALL_IN_FLIGHT, PHASE_CATCHING):
            return self._step_in_flight(now, obs)
        if self._phase == PHASE_SETTLING:
            return self._step_settling(now, obs)
        return TossDecision(self._phase, ACTION_NONE, False, None)

    # ── phase handlers ─────────────────────────────────────────────────────────

    def _step_checking(self, now: float, obs: TossObservations) -> TossDecision:
        # Live-observation preconditions (loud rejects, plan § Choreography 1).
        # Hand-evidence chain in dependency order: freshness before the park band
        # (a stale reading makes the band unknowable), band before possession (a
        # seated ball is only throwable from catch-rest).
        if not obs.mocap_fresh:
            return self._reject('MOCAP_STALE')
        if not obs.streaming:
            return self._reject('NOT_STREAMING')
        if not obs.platform_levelled:
            # GEOMETRY, not process. Un-levelled, the launch leaves the cup
            # 0.78° off gravity (the 2026-07-25 measured offset), and a vertical
            # toss drifts v·sin(θ)·T = 3.93 m/s × sin(0.78°) × 0.8 s = 43 mm
            # against a ~35 mm cup radius (GEOM_HAND_RADIUS_MM). That drift is
            # exactly 4·h·sin(θ) — LINEAR in apex height — so 43 mm is the
            # ~0.79 m config-default apex and a 0.6 m goal gives 33 mm. The
            # gate is height-INDEPENDENT on purpose: it asks whether the machine
            # knows where gravity is, not whether one goal fits in the cup (see
            # ros_ws/docs/levelling_frame.md, C-LEVEL-1.O). The catch is
            # geometrically impossible before the ball leaves the hand, so the
            # throw refuses rather than putting a ball on the floor — the same
            # class of loud-early reject as HAND_NOT_PARKED.
            #
            # Placed AFTER mocap_fresh/streaming and BEFORE the hand chain on
            # purpose. A stale graph makes this flag UNKNOWABLE, not False, and
            # a misleading reject code sends the operator to the wrong
            # subsystem — so the two staleness conditions that ARE separately
            # observable get to speak first. Honest caveat, and the node-side
            # comment repeats it: `streaming` is a sticky last-value with no
            # freshness stamp of its own, so a trajectory_node that dies
            # outright surfaces HERE (its status stops, this goes False) rather
            # than as NOT_STREAMING. The reject-code table in
            # tests/hardware/toss_trace_recorder.py names both causes.
            return self._reject('NOT_LEVELLED')
        if not obs.hand_fresh:
            return self._reject('HAND_STALE')
        # ── THE STATIC / EVIDENCE LINE (B4, plan § 2.4.1) ──
        # Everything ABOVE is a function of the goal, the config and slowly-
        # varying link state; everything BELOW is a function of the PREVIOUS
        # cycle's outcome, and is FALSE for part of the staging window BY
        # CONSTRUCTION — the hand is inside the previous catch stroke, the cup is
        # empty for the whole flight, and the machine's own airborne ball is a
        # live track destined for it. Evaluating them at stage time would refuse
        # every healthy pipelined cycle.
        #
        # They are not dropped; they MOVE, to the single tick before the CAN
        # frame exists (:meth:`_step_committing`). That makes the pipeline's
        # empty-stroke gate STRICTER than the serial one, not weaker: the
        # evidence-to-dispatch distance falls from the serial ladder's
        # 0.160-0.520 s to zero ticks.
        #
        # The four gates above are re-read at COMMIT too (a link hiccup between
        # stage and commit must refuse) — see `_step_committing`.
        if self.staged:
            self._phase = PHASE_POSITIONING
            self._position_dispatched = True
            self._positioning_deadline = now + self.positioning_timeout_s
            self._position_busy_deadline = now + TOSS_POSITION_BUSY_PATIENCE_S
            return TossDecision(PHASE_POSITIONING, ACTION_POSITION_PLATFORM,
                                False, None)
        if not obs.hand_parked:
            return self._reject('HAND_NOT_PARKED')
        if not obs.ball_seated:
            # TWO codes, because they send the operator to different subsystems.
            # EMPTY is "the cup is empty" — load a ball. UNKNOWN is "the ball
            # sensor cannot answer" (boot before the first TxSdo reply, a stale
            # reply, an un-anchored bridge clock, a dead poller) — and it REFUSES
            # rather than passing, because a fail-open sensor gate is the exact
            # BallButler defect this project declined to copy
            # (plans/archived/hand-ball-sensor.md § "Three BallButler properties
            # deliberately not copied", row 1). C-POSSESS-1 § 3.3.
            if obs.ball_evidence == EVIDENCE_UNKNOWN:
                return self._reject('BALL_UNKNOWN')
            return self._reject('NO_BALL')
        # Last gate: no live tracker expectation may already be destined for us —
        # a phantom destination='jugglebot' track (an earlier announcement whose
        # throw never happened) would correlate against OUR announcement and feed
        # the catch pipeline someone else's kinematics.
        if obs.track_active:
            return self._reject('TRACK_ACTIVE')

        self._phase = PHASE_POSITIONING
        self._position_dispatched = True
        self._positioning_deadline = now + self.positioning_timeout_s
        self._position_busy_deadline = now + TOSS_POSITION_BUSY_PATIENCE_S
        return TossDecision(PHASE_POSITIONING, ACTION_POSITION_PLATFORM, False, None)

    def _step_positioning(self, now: float, obs: TossObservations) -> TossDecision:
        if self._position_result is None:
            if now >= self._positioning_deadline:
                return self._abort('POSITION_TIMEOUT')
            return TossDecision(PHASE_POSITIONING, ACTION_NONE, False, None)
        accepted, _planned_s, code = self._position_result
        if not accepted:
            if code == _POSITION_BUSY_CODE:
                # THE ONE re-pollable refusal (bag 2026-08-28_23-53-25): the
                # previous cycle's catch plan is still inside its settle hold, so
                # the platform is genuinely busy for a BOUNDED, KNOWN interval.
                # Every other code names a state that will not clear on its own.
                busy = self._absorb_position_busy(now)
                if busy is not None:
                    return busy
            # Nothing moved, nothing armed — a loud reject with no cleanup,
            # carrying the go_to_pose reject-ladder code (reload's REJECTED_BB
            # pattern) AND, since 2026-08-29, the service's own message after it.
            # The subcode is unchanged and still leads the parenthetical — the
            # re-poll keys on `code`, not on this string, and a consumer wanting
            # the code asks `base_outcome` + the leading subcode rather than
            # matching the whole thing. A response with no message (every FSM
            # unit test, and the synthesised NO_RESPONSE) is byte-identical to
            # the pre-enrichment outcome.
            detail = code or 'NO_RESPONSE'
            if self._position_message:
                detail = '{}: {}'.format(detail, self._position_message)
            return self._reject('POSITION', detail)
        if now < self._position_arrival_time:
            # The arrived-before-arming invariant: PREPARE strictly after the
            # timed arrival — arming mid-move C2-stops the move, leaving the
            # platform wherever it happened to be and firing the throw from a
            # site the aim was not solved for (for 8b, with an unfinished
            # pre-tilt). Pre-C-REACH-1 the same stop ALSO planted the reach
            # envelope at that wrong pose; the toss now declares the envelope
            # centre, so only the aim is at stake — which is enough.
            return TossDecision(PHASE_POSITIONING, ACTION_NONE, False, None)
        if obs.platform_at_target:
            return self._enter_preparing(now)
        if now >= self._positioning_deadline:
            # Timed arrival passed but the (config-enabled) mocap cross-check
            # never corroborated within the verification window — the silent
            # no-op class (disarmed wire, guard latch): the move was "accepted"
            # yet the platform is not there. Throwing here would launch the
            # ball from a site the aim was not solved for. Unreachable with the
            # check disabled (the node then feeds platform_at_target True every
            # tick).
            return self._abort('POSITION_FAILED')
        return TossDecision(PHASE_POSITIONING, ACTION_NONE, False, None)

    def _absorb_position_busy(self, now: float) -> Optional[TossDecision]:
        """Bounded re-poll of a ``go_to_pose`` refused ``BUSY``.

        ``None`` ⇒ the caller rejects (the wait would be paid out of the release
        window, or the patience is spent); a decision ⇒ HOLD between polls
        (``ACTION_NONE``) or RE-EMIT ``ACTION_POSITION_PLATFORM``. The node
        dispatches on the ACTION, so a re-emission re-runs
        ``_position_platform_for_toss`` verbatim against the SAME cached
        ``state.positioning_move`` — the pose is re-derived from the same stash
        and cannot drift between polls.

        The refusal being absorbed is structural, not incidental: at the head of a
        chained cycle the previous cycle's catch plan is still inside its
        ``JB_TRAJ_CATCH_SETTLE_HOLD_S`` quiescent tail, ``trajectory_node``'s
        ``_active_move_in_flight`` is True and its BUSY guard is CORRECT to refuse.
        A co-located chain never sees it only because the census-B1 skip never
        calls the service.
        """
        if self._position_busy_seen_at <= 0.0:
            self._position_busy_seen_at = float(now)
            self._position_busy_next_poll = now + TOSS_POSITION_BUSY_REPOLL_S
        self._position_busy_wait_s = max(
            0.0, float(now) - self._position_busy_seen_at)
        # ── THE WAIT IS NEVER PAID OUT OF THE RELEASE WINDOW ──
        # Checked from the FIRST BUSY, not only at the patience bound.
        # `_step_preparing`'s release guard has NO slip, so a cycle that re-polls
        # past this floor turns a REJECTED_POSITION(BUSY) — nothing moved, nothing
        # armed — into an ABORTED_CANT_MAKE_RELEASE with the hand committed and a
        # retract under a seated ball. That is a strictly worse terminal for the
        # same fault, and it is the cadence fence this conjunct holds.
        #
        # ONE derivation, the same `_build_toss_cycle` charges at accept, on the
        # MOVER branch — which is the only branch a BUSY can arise on, since a
        # census-B1 skip never reaches the service. Accept-time and runtime
        # therefore cannot disagree, and a re-cut of the floor moves both.
        if (self._t_release - now) <= min_throw_delay_for_release_s(
                self.event_vel_mps, True, float(self.min_event_delay_s)):
            return None
        if now >= self._position_busy_deadline:
            return None
        if now < self._position_busy_next_poll:
            # Holding between polls. The BUSY result deliberately STAYS in place:
            # clearing it here would drop this tick into the `_position_result is
            # None` arrival-wait branch above, which reads as "no answer yet" and
            # waits out `_positioning_deadline`. The result is cleared at exactly
            # one instant — the tick that re-emits.
            return TossDecision(PHASE_POSITIONING, ACTION_NONE, False, None)
        # `note_position_result` is first-result-wins, so clearing admits exactly
        # ONE more — and it is cleared in the same tick the node re-dispatches in
        # (`_step_toss_sequence` calls the action synchronously), never earlier.
        self._position_result = None
        self._position_message = ''      # the answer and its prose clear together
        self._position_busy_next_poll = now + TOSS_POSITION_BUSY_REPOLL_S
        self._position_busy_polls += 1
        return TossDecision(PHASE_POSITIONING, ACTION_POSITION_PLATFORM,
                            False, None)

    def _enter_preparing(self, now: float) -> TossDecision:
        """Verified arrival → arm the catch BEFORE anything is committed at the
        hand: the node declares the reach-envelope centre (the nominated catch
        B — contract C-REACH-1) one tick ahead, the PREPARE bundle raises the
        latch (which consumes that declaration), and only after the node
        confirms it does the announcement — and then the throw — go out."""
        self._phase = PHASE_PREPARING
        self._prepare_dispatched = True
        return TossDecision(PHASE_PREPARING, ACTION_PREPARE_CATCH, False, None)

    def _step_preparing(self, now: float, obs: TossObservations) -> TossDecision:
        if self._prepare_result is None:
            return TossDecision(PHASE_PREPARING, ACTION_NONE, False, None)
        if not self._prepare_result:
            # A NAMED refusal (the S6 reach-centre drift guard) terminalises as
            # REJECTED_<code>; anything else is the generic abort. Both take the
            # SAFE_ABORT cleanup — `_prepare_dispatched` is already True, so the
            # declaration and the holds are out and the machine is not pristine.
            if self._prepare_reject_code:
                return self._reject(self._prepare_reject_code,
                                    self._prepare_reject_message)
            return self._abort('PREPARE_FAILED')
        if self.staged:
            # ── B4: the preamble ends HERE, one tick's worth of work short of
            # the serial ladder's announce. The cycle parks in STAGED until its
            # commit instant; the release-window guard, the announcement and the
            # dispatch all belong to that single tick (§ 2.4.2), which is what
            # the whole pipeline is for.
            self._phase = PHASE_STAGED
            self._staged_at = now
            return TossDecision(PHASE_STAGED, ACTION_NONE, False, None)
        # Release-window guard — checked BEFORE the announcement goes out (an
        # announced-then-aborted toss leaves a phantom tracker expectation that
        # REJECTED_TRACK_ACTIVE would then refuse on until it expires), and again
        # on the post-announce tick (a stall between announce and dispatch must
        # not push event_delay under the stroke's windup floor).
        if self._t_release - now < self.min_event_delay_for_throw_s:
            return self._abort('CANT_MAKE_RELEASE')
        if not self._announce_dispatched:
            # Armed confirmed → explicit ≥1-tick gap → announce: the armed edge
            # and the announcement travel on different topics with no cross-topic
            # ordering guarantee, and catch_coordinator drops announcement
            # pre-tilts that arrive unarmed. The announce action is emitted on a
            # LATER tick than the PREPARE bundle by construction.
            #
            # ⚠ S6 (2026-08-27) DID NOT DELETE THIS GAP, and the tick it costs
            # is NOT reclaimed here. Under the session-scoped latch the gap is
            # already satisfied by construction — `trajectory/arm_catch` was
            # raised SECONDS earlier, at session scope, so catch_coordinator has
            # been armed since before this cycle existed — which is strictly
            # stronger than the one tick this deferral buys. The tick survives
            # for two reasons: the single `Toss` (and a session's own FIRST
            # cycle) still raise the latch one tick before this point and need
            # it, and reclaiming it is a CADENCE change that belongs to B4's
            # `commit_budget_s`, where the whole pre-dispatch ladder is
            # re-derived at once and a gate charges the real lead. Shortening
            # the ladder HERE would loosen the accept gate under a runtime guard
            # that had not moved — the exact accept-vs-runtime gap the
            # 2026-08-22 audit closed.
            if (self._t_release + self.flight_time_s) - now < TOSS_MIN_ANNOUNCE_LEAD_S:
                # WARN-only for BOTH tiers: 8a's pre-tilt target equals the
                # already-held pose (motion-free degeneracy), and 8b's platform
                # reach is deferred to t_release with lead = flight time by
                # construction — the announce lead sizes no 8b motion (the
                # Phase-1 "harden to an abort for 8b" promise is superseded;
                # see TOSS_MIN_ANNOUNCE_LEAD_S).
                self._announce_lead_short = True
            self._announce_dispatched = True
            return TossDecision(PHASE_PREPARING, ACTION_ANNOUNCE, False, None)
        if not self._announced:
            return TossDecision(PHASE_PREPARING, ACTION_NONE, False, None)
        if not obs.hand_parked:
            # Re-verify at THROWING entry, before the dispatch commitment:
            # CHECKING's park-band gate is seconds old by now (positioning +
            # prepare), and a kind-0 stroke commands ABSOLUTE positions from
            # 0 rev — dispatching with the hand off the bottom band is a
            # physical hazard. Prepared ⇒ SAFE_ABORT cleans up.
            return self._abort('HAND_NOT_PARKED')
        return self._enter_throwing(now)

    def _step_staged(self, now: float, obs: TossObservations) -> TossDecision:
        """PHASE_STAGED — the wait. The preamble is spent, nothing is armed, and
        this cycle owns nothing that moves; the only question left is whether the
        commit instant has arrived.

        It emits ``ACTION_NONE`` and nothing else, ever. That is S1′ stated as
        code rather than as discipline: the staged slot's whole emittable action
        set — ``{NONE, POSITION_PLATFORM(skip), PREPARE_CATCH}`` — is already
        spent by the time this handler is reached.

        The tick that CROSSES ``_commit_at`` IS the commit tick — it falls
        straight through rather than waiting for the next iteration, because the
        one loop period :func:`commit_budget_s` charges is exactly the polling
        lateness of this crossing, and spending a second period here would charge
        it twice."""
        if now < self._commit_at:
            return TossDecision(PHASE_STAGED, ACTION_NONE, False, None)
        self._phase = PHASE_COMMITTING
        return self._step_committing(now, obs)

    def _step_committing(self, now: float,
                         obs: TossObservations) -> TossDecision:
        """**THE ARM POINT** (plan § 2.4.2): the single tick at which the FSM
        evaluates the evidence and, in the same tick and in this order,
        publishes the announcement and dispatches the throw. No dispatch is ever
        issued on evidence read at an earlier tick.

        The gate, in its own order, and every line of it load-bearing:

        1. **the upstream cycle must have terminalised** (S1′) — else SLIP. The
           node answers with :meth:`note_upstream_terminalised`, from the SAME
           ``_possession_observed`` read that admits this throw (§ 2.4.4): a
           SEATED cup at the commit tick *is* the CAUGHT evidence for the cycle
           ahead, so one read serves both and the cup is never sampled at two
           instants for one decision;
        2. **the four STATIC link gates, RE-READ** — a hiccup between stage and
           commit must refuse, and these are cheap. They ABORT rather than slip:
           a machine that has left TRAJECTORY or lost mocap is not late, it is
           broken;
        3. **``staged_site_ok``** — REJECT (``REJECTED_SITE_MOVED``). THE
           honest-cache gate (2026-08-28): is the platform still where this
           cycle's throw site was NOMINATED to be? A staged cycle's POSITIONING
           is a SKIP by construction, so unlike the serial path it cannot
           COMMAND its nomination true — and the cycle ahead of it moves the
           platform with its deferred A->B reach in exactly this window. It
           REJECTS rather than slipping because waiting cannot fix it: the
           orientation the reach commanded is the one it meant to command, and
           the serial rebuild re-nominates from a fresh live read, which is
           strictly more correct than waiting for a stale nomination to come
           true. See :attr:`TossObservations.staged_site_ok`;
        4. **``hand_parked``** — SLIP. The hand is still landing; that is a
           cadence fact on a healthy machine, and ``REJECTED_HAND_NOT_PARKED``
           there would be a machine-fault verdict for it (the R5 mis-routing);
        5. **``ball_seated``** — SLIP. THE hard gate, and it is evaluated at the
           last tick before the CAN frame exists. The cup's seat edge is late by
           a measured, systematic +183.9 ms median (§ 1.4), so refusing here
           would turn every healthy cycle into ``REJECTED_NO_BALL``;
        6. **``track_active``** — REJECT. A phantom destination='jugglebot'
           track would correlate against OUR announcement, and unlike the two
           above it is not a thing that resolves by waiting;
        7. **the runtime release-window guard** — the same inequality
           ``_step_preparing`` applies, against the same budget — and since
           2026-08-28 it **SLIPS** like rungs 4 and 5 rather than aborting.
           Falling short here is a CADENCE fact on a healthy machine, not a
           machine fault: :func:`commit_budget_s` grants exactly one NOMINAL
           loop period of polling lateness, so any iteration that ran longer
           than :data:`NODE_LOOP_PERIOD_S` at the commit crossing breaks the
           inequality by the overshoot alone — velocity-independent, and the
           Jetson's non-RT sleep supplies it routinely. The announcement has
           not gone out, nothing is armed, and the slip re-arms
           ``_t_release = now + commit_budget_for_cycle_s`` so the NEXT tick
           gets a full fresh budget; the absolute tick grid
           (``_pace_to_next_tick``) makes the iteration after an overrun SHORT
           by exactly the overrun, so the retry is the likely case rather than
           the hopeful one. Past the shared slip bound it aborts by name — see
           :meth:`_slip`.

        Then ANNOUNCE and DISPATCH, in one tick, in that order — because
        ``ThrowAnnouncement`` carries ``throw_time`` and ``landing_time``, a
        slipped release invalidates an already-published announcement, and there
        is no withdrawal message on the wire. Publishing at commit is what keeps
        the announcement true.

        **No announcement is published on a refused commit**, which is the whole
        difference between this and the two ``ABORTED_CANT_MAKE_RELEASE`` cycles
        of bag ``2026-08-26_14-25-16``: those left a phantom tracker expectation
        behind that ``REJECTED_TRACK_ACTIVE`` then refused on until it expired."""
        if not self._upstream_clear:
            return self._slip(now, 'upstream cycle not terminalised')
        if not (obs.streaming and obs.mocap_fresh and obs.platform_levelled
                and obs.hand_fresh):
            # Named individually, in the SAME order CHECKING names them, so a
            # commit-time refusal routes the operator to the same subsystem a
            # stage-time one would.
            if not obs.mocap_fresh:
                return self._abort('MOCAP_STALE')
            if not obs.streaming:
                return self._abort('NOT_STREAMING')
            if not obs.platform_levelled:
                return self._abort('NOT_LEVELLED')
            return self._abort('HAND_STALE')
        if not obs.staged_site_ok:
            # THE honest-cache gate. The staged nomination is only valid if the
            # platform is where the nomination ASSUMED at the moment the throw
            # becomes irrevocable — and this is that moment. REJECT, not slip:
            # nothing about a moved platform resolves by waiting, and the serial
            # rebuild re-nominates from a live read and COMMANDS the move.
            return self._reject('SITE_MOVED')
        if not obs.hand_parked:
            return self._slip(now, 'hand not back inside the park band',
                              reject_code='HAND_NOT_PARKED')
        if not obs.ball_seated:
            return self._slip(
                now, 'cup not seated',
                reject_code=('BALL_UNKNOWN'
                             if obs.ball_evidence == EVIDENCE_UNKNOWN
                             else 'NO_BALL'))
        if obs.track_active:
            return self._reject('TRACK_ACTIVE')
        if self._t_release - now < self.min_event_delay_for_throw_s:
            return self._slip(now, 'the iteration that reached the commit ran '
                                   'longer than one nominal loop period',
                              abort_code='CANT_MAKE_RELEASE')
        # ── COMMITTED. Everything below this line is one tick. ──
        self._commit_slip_s = max(0.0, now - self._commit_at_sched)
        self._committed = True
        self._announce_dispatched = True
        if (self._t_release + self.flight_time_s) - now < TOSS_MIN_ANNOUNCE_LEAD_S:
            # WARN-only for both tiers, exactly as on the serial path — and at
            # the milestone cadences it is ALWAYS short, which is the point of
            # the constant's own comment rather than a surprise.
            self._announce_lead_short = True
        self._phase = PHASE_THROWING
        self._throw_dispatched = True
        self._release_deadline = self._t_release + self.release_grace_s
        # The FSM's own phase advances to THROWING (the dispatch is committed
        # and the cancel policy must treat it as such from this instant), but
        # the DECISION reports ``COMMITTING``, because that is what this tick
        # WAS. Two consumers depend on the distinction:
        #
        #  * the session feedback publishes the decision's phase, and the arm
        #    point is the single most operator-relevant instant of a cycle — a
        #    tick that reported THROWING would make ``COMMITTING`` a wire
        #    string that appears only on a SLIP;
        #  * :class:`LoopPeriodCensus` classifies iterations by the reported
        #    phase, and this tick is exactly what :func:`commit_budget_s`
        #    charges its one loop period for. Reporting THROWING would file the
        #    one PRE-dispatch tick the pipeline has under the post-dispatch
        #    idle majority — which is the dilution the pre/post split exists to
        #    prevent.
        return TossDecision(PHASE_COMMITTING, ACTION_ANNOUNCE, False, None,
                            action_then=ACTION_DISPATCH_THROW)

    def _slip(self, now: float, why: str, reject_code: str = '',
              abort_code: str = '') -> TossDecision:
        """SLIP the commit to the next loop iteration, or TERMINALISE once the
        slip has run past its bound (plan § 2.4.3).

        **The slip moves ``_t_release`` with it**, so the released ball's own
        schedule stays self-consistent — the announcement has not gone out, and
        every consumer (the landing, the settle deadline, the 8b reach, the
        cancel cutoff, the dispatch's ``event_delay``) reads ``_t_release`` and
        re-derives nothing. Two bounds, both DERIVED rather than chosen:

        * **the release bound**. ``now`` and ``_t_release`` advance together, so
          a slip always leaves the runtime guard's inequality
          ``_t_release − now ≥ min_event_delay_for_throw_s`` satisfied AT THE
          INSTANT OF THE SLIP, with :func:`commit_budget_s`'s one nominal loop
          period plus :data:`FLOOR_REPRESENTATION_SLACK_S` of headroom on top.

          ⚠ This read "holds by construction" until 2026-08-28, and **that was
          false whenever the ACTUAL period exceeded the nominal one**: the
          headroom a slip banks is exactly ``NODE_LOOP_PERIOD_S + 1 µs``, so an
          iteration longer than that consumes all of it and the guard is broken
          on the NEXT tick — velocity-independent, and the mechanism behind 4 of
          the 15 staged slots of the first pipelined sitting. What holds now is
          the sentence above (the inequality holds AT the slip instant, not
          across an arbitrarily long following iteration), and the guard's own
          shortfall is routed BACK here with ``abort_code`` instead of aborting
          on the spot. That closes the loop: a late tick slips, the absolute
          tick grid (``_pace_to_next_tick``) makes the iteration after an
          overrun short by exactly the overrun, and a merely-jittery machine
          commits one tick later;
        * **the upstream bound** is :attr:`catch_confirm_window_s` — the
          upstream cycle terminalises at the cup edge or at
          ``landing + catch_confirm_window_s`` at the latest, so a slip that
          outlives it is waiting for something that is not coming. No new
          constant: mutate that one and this bound moves with it. It is counted
          from ``_commit_at_sched`` (the ORIGINAL commit instant) and it governs
          EVERY slip source, the late-tick one included — ONE bound, so a cycle
          cannot launder an unbounded wait by alternating its reasons.

        Past the bound the cycle terminalises by the name of the gate that held
        it — ``REJECTED_NO_BALL`` / ``REJECTED_BALL_UNKNOWN`` /
        ``REJECTED_HAND_NOT_PARKED`` for the evidence gates, and
        ``ABORTED_CANT_MAKE_RELEASE`` for the release-window guard — with
        nothing armed at the hand and NO announcement published (§ 2.4.3's
        staged-failure table). The release-window guard keeps ``ABORTED_`` and
        keeps its historical name deliberately: that string is what the
        runbooks, the record corpus and the cadence ladder key on, and the
        terminal it names is now HONEST rather than premature — the machine
        slipped the whole window and still could not make the release. It
        carries :meth:`_commit_forensics`, so the one outcome line says by how
        much and after how many retries.

        A slip with NEITHER code (the upstream one) cannot terminalise on its
        own account: it is waiting on a cycle that has its own deadline, and
        when that deadline mints the upstream terminal the node discards this
        slot instead."""
        self._commit_slip_s = max(0.0, now - self._commit_at_sched)
        past_bound = now > self._commit_at_sched + self.catch_confirm_window_s
        if abort_code and past_bound:
            return self._abort(abort_code, self._commit_forensics(now, why))
        if reject_code and past_bound:
            return self._reject(reject_code, 'commit slipped {:.3f} s past the '
                                             '{:.3f} s bound ({})'
                                .format(self._commit_slip_s,
                                        self.catch_confirm_window_s, why))
        # Re-arm on the next iteration, and take the release with it.
        self._commit_slips += 1
        self._commit_at = now
        self._t_release = now + self.commit_budget_for_cycle_s
        return TossDecision(PHASE_COMMITTING, ACTION_NONE, False, None,
                            slip=True)

    def _commit_forensics(self, now: float, why: str) -> str:
        """F5 — everything a post-mortem of a slip-bound-exhausted commit needs,
        in ONE string, carried ON THE OUTCOME so the node's single authoritative
        outcome line IS the forensic line: ``_log_toss_outcome`` prints the
        outcome verbatim, the staged slot's discard WARN prints it as
        ``staged_discarded_reason``, and the record carries both.

        The four numbers, and why each is here rather than inferable:

        * **the shortfall** ``_t_release − now``, against the dispatch budget it
          was measured against — the actual quantity the guard refused on, and
          signed, so "missed by 1 ms" and "missed by 200 ms" are different
          findings rather than one verdict string;
        * **the lateness** ``now − _commit_at_sched`` — how far past the
          ORIGINALLY scheduled arm point the machine got before giving up;
        * **the slip count** — how many iterations it actually retried. One is a
          single unlucky tick; a bound's worth is a loop that is chronically
          over period, which is a re-cut conversation and not a bad sitting;
        * **the last iteration's length**, against :data:`NODE_LOOP_PERIOD_S` —
          the proximate cause, measured by the FSM itself between consecutive
          :meth:`step` calls. :class:`LoopPeriodCensus` carries the same
          quantity per cycle as a MAXIMUM (``loop_period_max_pre_s``); this is
          the specific iteration that broke the inequality, and taking it from
          the FSM's own clock rather than plumbing the census into the FSM keeps
          the census strictly downstream of every decision — the constraint
          ``test_the_census_never_feeds_a_budget`` pins. It is a LOG STRING, not
          a budget: nothing here feeds back into a floor."""
        tick = self._last_tick_s
        tick_s = '{:.4f}'.format(tick) if math.isfinite(tick) else 'n/a'
        return ('{}: lead {:+.4f} s against the {:.4f} s dispatch budget, '
                '{:.3f} s past the scheduled commit, {} slip(s) inside the '
                '{:.3f} s bound, last iteration {} s vs the {:.4f} s nominal'
                .format(why, self._t_release - now,
                        self.min_event_delay_for_throw_s,
                        now - self._commit_at_sched, self._commit_slips,
                        self.catch_confirm_window_s, tick_s,
                        NODE_LOOP_PERIOD_S))

    def _enter_throwing(self, now: float) -> TossDecision:
        """Everything Jugglebot-side is armed (latch up, announcement out) — the
        throw is the LAST commitment of the sequence (the reload ordering
        principle transposed)."""
        self._phase = PHASE_THROWING
        self._throw_dispatched = True
        self._release_deadline = self._t_release + self.release_grace_s
        return TossDecision(PHASE_THROWING, ACTION_DISPATCH_THROW, False, None)

    def _reach_action_if_due(self, now: float) -> str:
        """Tier 8b's ``ACTION_REACH_CATCH``, exactly once, on the first tick with
        ``now >= t_release`` — TIME-triggered, never evidence-triggered (release
        evidence can lag up to the 0.5 s grace, which would eat most of the
        flight and push the reach lead toward trajectory_node's 0.25 s
        min_timed_lead floor; tick-rate jitter only shrinks the lead by one
        coordinator tick). Emitted even with NO release evidence: if the stroke
        silently never fired, the platform translates A→B carrying the seated
        ball (the benign-accel class) and ABORTED_NO_RELEASE still cleans up at
        t_release + grace. Tier 8a NEVER emits it (byte-identical decisions)."""
        if (self.tier == TIER_8B and not self._reach_dispatched
                and now >= self._t_release):
            self._reach_dispatched = True
            return ACTION_REACH_CATCH
        return ACTION_NONE

    def _step_throwing(self, now: float, obs: TossObservations) -> TossDecision:
        # The node's dispatch runs synchronously inside the tick; a still-missing
        # classification means the call is in flight — wait (the node-level
        # sequence ceiling backstops a pathological never-noted state). The
        # throw is already dispatched, so a due 8b reach still goes out here.
        if self._throw_dispatch_result is None:
            return TossDecision(PHASE_THROWING, self._reach_action_if_due(now),
                                False, None)
        outcome, _message = self._throw_dispatch_result
        if outcome == THROW_DISPATCH_REJECTED:
            # Definitive no-arm: no CAN frame exists, the ball is still seated.
            # The SAFE_ABORT retract deliberately also clears any half-state on
            # the last-writer-wins queue. Checked BEFORE the reach-due check:
            # nothing will fly, so no reach is wanted.
            return self._abort('THROW_DISPATCH_FAILED')
        # OK or AMBIGUOUS: await release evidence — NEVER a second dispatch. An
        # ambiguous ack (ERR_TIMEOUT class) may have armed the stroke; a
        # re-dispatch re-packs a new wall_time and, post-release, would CLOBBER
        # the armed catch stroke on the last-writer-wins queue.
        if obs.throw_stroke_seen or obs.ball_track_confirmed:
            self._phase = PHASE_BALL_IN_FLIGHT
            # Confirm CAUGHT within the window PAST the scheduled landing: the
            # FSM is the announcer, so release + ToF is its own schedule — the
            # deadline must include the flight, not treat release as landing.
            self._settle_deadline = (self._t_release + self.flight_time_s
                                     + self.catch_confirm_window_s)
            return TossDecision(PHASE_BALL_IN_FLIGHT,
                                self._reach_action_if_due(now), False, None)
        if now >= self._release_deadline:
            # No evidence on either channel by t_release + grace: the stroke is
            # presumed never to have fired; the ball is presumed seated (the
            # possession latch survives). Residual risk (ambiguous ack ∧ blind
            # telemetry ∧ mocap miss ⇒ retract under a genuinely airborne ball)
            # is accepted for Phase 1 — same class as reload's NO_ANNOUNCEMENT.
            return self._abort('NO_RELEASE')
        return TossDecision(PHASE_THROWING, self._reach_action_if_due(now),
                            False, None)

    def _step_in_flight(self, now: float, obs: TossObservations) -> TossDecision:
        # A standing catch-target infeasibility does NOT terminate mid-flight:
        # the platform holds, the hand keeps its armed schedule, and the ball
        # gets its chance to land in the primed cup (reload hardware evidence
        # 2026-07-23, twice — finishing early tore the arming down and retracted
        # the hand INTO the incoming ball). `_catch_infeasible` stays latched and
        # resolves the OUTCOME at settle instead.
        if obs.ball_caught:
            self._phase = PHASE_SETTLING
            return self._finish(TossResult(
                True, 'CAUGHT', obs.catch_error_mm, self._achieved_flight_s(),
                obs.catch_event_dt_s))
        if now >= self._settle_deadline:
            self._phase = PHASE_SETTLING
            return self._step_settling(now, obs)
        # Transition the reported phase to CATCHING once near the scheduled landing.
        # Release evidence can beat t_release (the stroke telemetry rises during
        # the windup), so a due 8b reach may first fire from here.
        near = now >= (self._landing_perf() - self.catch_confirm_window_s)
        self._phase = PHASE_CATCHING if near else PHASE_BALL_IN_FLIGHT
        return TossDecision(self._phase, self._reach_action_if_due(now),
                            False, None)

    def _landing_perf(self) -> float:
        """Scheduled landing on the FSM's perf clock. The FSM is the announcer —
        the commanded release + flight IS the announced prediction (no external
        announcement to refine it, unlike reload's ``_landing_time_perf``)."""
        return self._t_release + self.flight_time_s

    @property
    def landing_perf(self) -> float:
        """Public read of the scheduled landing — the window the hand ball sensor
        looks for its arrival edge in (C-POSSESS-1 § 3.2). NaN before ``start()``
        has stamped a release, so a sensor query on a goal that has not scheduled
        anything answers ``ARRIVAL_UNKNOWN`` rather than looking around t=0."""
        if self._t_release <= 0.0:
            return float('nan')
        return self._landing_perf()

    def _step_settling(self, now: float, obs: TossObservations) -> TossDecision:
        if obs.ball_caught:
            return self._finish(TossResult(
                True, 'CAUGHT', obs.catch_error_mm, self._achieved_flight_s(),
                obs.catch_event_dt_s))
        if obs.possession_blind:
            # The cup sensor could not LOOK across the arrival window (invalid
            # samples, a stale poller, an un-anchored bridge clock). Since D1 it
            # is the sole possession source, so "no rise seen" is not evidence of
            # no arrival — and a dead sensor must never be laundered into a MISS
            # the operator would route at the throw. Same MISSED family (terminal
            # action, stop_on_miss governance and the session accounting are all
            # keyed on the prefix), a different name, checked BEFORE the
            # infeasibility branch because a blind cup cannot corroborate that
            # branch's "and the ball did not land in the cup anyway" either.
            return self._finish(TossResult(
                False, 'MISSED_SENSOR_BLIND', float('nan'),
                self._achieved_flight_s()))
        if self._catch_infeasible is not None:
            # No catch target was EVER accepted for this flight — the platform
            # never had a reachable catch pose — and the ball did not land in
            # the cup anyway.
            return self._finish(TossResult(
                False, 'MISSED_INFEASIBLE_{}'.format(self._catch_infeasible),
                float('nan'), self._achieved_flight_s()))
        return self._finish(TossResult(
            False, 'MISSED', float('nan'), self._achieved_flight_s()))

    # ── terminal helpers ───────────────────────────────────────────────────────

    def _achieved_flight_s(self) -> float:
        """Measured release → catch-plane time: the tracker's last live
        KF-refined landing-plane crossing minus the COMMANDED release. NaN unless
        the announced ball was ever tracking-CONFIRMED (physical airborne
        evidence) AND a finite crossing estimate was observed. Caveats: the
        commanded release is contaminated by the unmeasured JB command→release
        latency (the Phase-5 T0 measurand), and the tracker plane is fixed at
        the ACTIVE cup height — a nominated catch z off ACTIVE biases the
        crossing by ~8 ms per 30 mm. A Phase-1 diagnostic field, not a truth."""
        if self._track_confirmed_seen and math.isfinite(self._last_time_at_land):
            return self._last_time_at_land - self._t_release
        return float('nan')

    def _displacement_detail(self, displacement: float, bound: float,
                             term: str, live: tuple) -> str:
        """The REJECTED_DISPLACEMENT parenthetical: which of the TWO bounds
        actually refused, the other one for context, and the knob that moves the
        binding one.

        Both limits are quoted every time because "too far" is ambiguous between
        them and they move under different knobs: the cap is a config number the
        operator edits, the reach bound is a function of the flight time and the
        LIVE session limits, so raising ``throw_height_m`` or ramping
        ``set_limits`` clears one and does nothing at all to the other. Reporting
        only the binding one would send half the refusals to the wrong knob.

        BINDING = the SMALLER limit, not the first one tested: with both
        exceeded, the smaller is the one that still refuses after the other is
        raised, so it is the one the remedy has to name.

        The limits label is THREE-VALUED because ``live`` is a PER-FIELD ``or
        None`` mapping and a partial observation is a real state (a status
        message that carried, say, jerk but left vel at the 0.0
        unknown-sentinel). Labelling that whole triple "live" — as this did
        until 2026-08-29 — printed fallback ``REACH_*`` numbers under a "live"
        banner, and an operator reading it would ramp ``set_limits`` on a term
        the session never reported and watch the refusal not move. So: all three
        absent ⇒ ``default``, all three observed ⇒ ``live``, anything between ⇒
        ``mixed``, and a trailing ``*`` marks each individual term that fell back
        (legend in the tail). The label and the numbers can no longer
        disagree."""
        cap = float(self.max_displacement_mm)
        t = float(self.flight_time_s)
        # The three limits arrive as ONE trajectory/status sample, so they are
        # live together or absent together; the effective values are re-derived
        # here (not re-read from `live`) so the quoted numbers are exactly the
        # ones the bound above was computed from, fallbacks included.
        eff = (REACH_VEL_LIMIT_MMPS if live[0] is None else float(live[0]),
               REACH_ACC_LIMIT_MMPS2 if live[1] is None else float(live[1]),
               REACH_JERK_LIMIT_MMPS3 if live[2] is None else float(live[2]))
        mixed = (any(v is None for v in live)
                 and not all(v is None for v in live))
        limits = '{} limits {}'.format(
            'default' if all(v is None for v in live)
            else ('mixed' if mixed else 'live'),
            '/'.join('{:.0f}{}'.format(e, '' if l is not None else '*')
                     for e, l in zip(eff, live)))
        # The legend rides only when a star is actually present, so a fully-live
        # refusal (the common case once trajectory/status is flowing) stays as
        # short as it was.
        star_note = ('; * = default, not reported'
                     if any(v is None for v in live) else '')
        reach_text = 'reach bound {:.1f} mm ({}-bound) at T = {:.3f} s'.format(
            bound, term, t)
        cap_text = 'cap {:.1f} mm [toss_max_displacement_mm]'.format(cap)
        over_cap = displacement > cap
        over_bound = displacement > bound
        if over_cap and (not over_bound or cap <= bound):
            return bound_msg(
                '|B-A| =', displacement, '>', cap, 'mm', digits=1,
                knob='toss_max_displacement_mm', limit_label='cap',
                tail='{} {} — lower |B-A| or raise the cap'.format(
                    reach_text,
                    'also exceeded' if over_bound else 'not binding'))
        return bound_msg(
            '|B-A| =', displacement, '>', bound, 'mm', digits=1, knob=limits,
            limit_label='reach bound',
            tail='{}-bound at T = {:.3f} s, {} {} — raise throw_height_m '
                 '(longer T) or set_limits {}{}'.format(
                     term, t, cap_text,
                     'also exceeded' if over_cap else 'not binding', term,
                     star_note))

    def _reject(self, code: str, message: str = '') -> TossDecision:
        outcome = 'REJECTED_{}'.format(code)
        if message:
            outcome = '{}({})'.format(outcome, message)
        return self._finish(TossResult(
            False, outcome, float('nan'), self._achieved_flight_s()))

    def _abort(self, code: str, message: str = '') -> TossDecision:
        """``ABORTED_<code>``, optionally carrying a parenthesised forensic
        message — symmetric with :meth:`_reject`, and for the same reason: the
        outcome string is the ONE thing every consumer sees (the node's single
        authoritative outcome line, the record, the session's per-cycle list),
        so a terminal whose numbers matter carries them there rather than in a
        second log line that can be filtered away from the first."""
        outcome = 'ABORTED_{}'.format(code)
        if message:
            outcome = '{}({})'.format(outcome, message)
        return self._finish(TossResult(
            False, outcome, float('nan'),
            self._achieved_flight_s()))

    def _finish(self, result: TossResult) -> TossDecision:
        self._finished = True
        self._result = result
        return TossDecision(self._phase, self._terminal_action(result), True, result)

    def _terminal_action(self, result: TossResult) -> str:
        """The cleanup action the node runs on a terminal decision:
          - a successful catch STAYs at the catch pose by default
            (``stay_at_pose_on_caught``; lower the latch + release the holds,
            NO go_home — the emitter's terminal hold does the rest) so the next
            Toss reads its throw site A from where this one caught and a
            session chains A → B → C. ``False`` restores the pre-2026-07-29
            RE-CENTER (lower the latch + go_home). Either way the hand keeps
            the caught ball — no retract — so the next Toss is immediately
            serviceable;
          - ANY not-caught terminal once the platform moved (``_positioned``) or
            the latch raise was dispatched (``_prepare_dispatched``) SAFE_ABORTs
            (armed-off → retract → latch-off → go_home). The pre-release
            retract's kind-3 deliberately CLEARS any armed kind-0 throw stroke on
            the last-writer-wins queue — the only un-arm mechanism the Teensy
            offers. If the retract ladder exhausts with a stroke armed, the throw
            MAY still fire with the catch torn down — loud ERROR at the node,
            residual risk accepted (reload's failed-retract posture);
          - a reject BEFORE anything moved or armed → no cleanup (ACTION_NONE).
        NOT-caught terminals keep go_home in BOTH stay settings, deliberately:
        a miss leaves a loose ball and possibly a hand at the top of its
        stroke, where safing to a known pose is the honest cleanup — and
        chaining, the only thing staying buys, is exactly what a miss has
        already ended.

        The action fires exactly once (the finished-replay path in :meth:`step`
        returns ACTION_NONE)."""
        if result.success:
            return ACTION_STAY if self.stay_at_pose_on_caught else ACTION_RECENTER
        if self.staged and not self._committed:
            # ── B4, plan § 2.4.3's staged-failure table: ACTION_NONE ──
            # A staged cycle that never passed its COMMIT gate has commanded
            # NOTHING and armed NOTHING. `_positioned` is True (POSITIONING is
            # skip-only here, so `note_position_noop` declared an arrival that
            # traversed zero millimetres) and `_prepare_dispatched` is True (the
            # staged PREPARE is the reach-centre drift guard and nothing else —
            # under S6 every publish and every service call moved to the session
            # or to the commit tick), so the serial test below would read both
            # commitments as real and SAFE_ABORT.
            #
            # That is not a tidiness point. A staged cycle can refuse WHILE THE
            # UPSTREAM CYCLE IS STILL AIRBORNE, and SAFE_ABORT's ladder retracts
            # the hand — under the incoming ball, with the catch torn down. The
            # honest cleanup for a slot that touched nothing is to drop it, which
            # is exactly what the node's discard path does.
            return ACTION_NONE
        if self._positioned or self._prepare_dispatched:
            return ACTION_SAFE_ABORT
        return ACTION_NONE

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def prepared(self) -> bool:
        """True once ANY robot-side commitment ran — the positioning move was
        ACCEPTED (``_positioned``; a moved platform needs the go_home leg of
        SAFE_ABORT) or the latch raise was dispatched (``_prepare_dispatched``).
        The node uses this to safe the robot on a node-level early exit
        (cancel/timeout/shutdown) that bypasses the FSM's own terminal."""
        return self._positioned or self._prepare_dispatched

    @property
    def position_busy_wait_s(self) -> float:
        """Seconds this cycle's POSITIONING spent absorbing ``go_to_pose`` BUSY
        re-polls — 0.0 for every cycle that was never refused.

        The forensic that separates the two states a REJECTED_POSITION(BUSY)
        cannot separate on its own: a cycle that waited out the previous catch's
        settle hold and threw (nonzero here, CAUGHT/MISSED outcome) and a cycle
        that met a WEDGE (nonzero here AND the reject). Recorded on every toss
        record, so a session's absorbs are countable rather than being an absence
        in the corpus."""
        return self._position_busy_wait_s

    @property
    def position_busy_polls(self) -> int:
        """How many times POSITIONING RE-EMITTED ``go_to_pose`` after a BUSY.

        :attr:`position_busy_wait_s` says HOW LONG the absorb ran; this says HOW
        MANY service round trips it cost — the same split ``slip_s`` and
        ``commit_slips`` make for the commit gate, and the quantity a re-cut of
        :data:`TOSS_POSITION_BUSY_REPOLL_S` would be argued from. The node reads
        it to emit its one-per-cycle forensics line on the FIRST re-poll."""
        return self._position_busy_polls

    @property
    def announce_lead_short(self) -> bool:
        """True once the announce→landing lead fell below
        ``TOSS_MIN_ANNOUNCE_LEAD_S`` at announce time — WARN-only for BOTH
        tiers (the node logs it). The Phase-1 promise to harden this to an
        abort for Tier 8b is SUPERSEDED — under the deferred reach the 8b
        platform lead is the flight time by construction; see the
        ``TOSS_MIN_ANNOUNCE_LEAD_S`` comment for the full reasoning."""
        return self._announce_lead_short

    @property
    def finished(self) -> bool:
        return self._finished

    @property
    def t_release(self) -> float:
        """Scheduled release instant (perf clock) — the node derives event_delay
        (``t_release − now`` at dispatch) and the cancel cutoff from it.

        Written ONCE, by :meth:`start`, from :attr:`release_at_perf` or the
        derived default. Read everywhere; re-derived nowhere."""
        return self._t_release

    @property
    def scheduled_lead_s(self) -> float:
        """accept → scheduled release, in seconds: the lead THIS cycle actually
        has, whichever way its release was scheduled.

        ``throw_delay_s`` is that lead only on the DERIVED path. With an
        absolute :attr:`release_at_perf` the two are different numbers, and
        anything that sizes a budget or a ceiling off the delay would then be
        sizing it off a quantity the cycle no longer runs on. Hence one
        accessor, read by :func:`reload_coordinator_node._toss_deadline_s`.

        Returns ``throw_delay_s`` on the derived path EXACTLY — not
        ``_t_release − _t_accept``, which is the same number only to within a
        float ulp at a large ``perf_counter`` origin — and before
        :meth:`start` has stamped anything (the throwaway sequencer the session
        ceiling budgets from is built and never started)."""
        if not self.release_at_perf or self._t_release <= 0.0:
            return float(self.throw_delay_s)
        return float(self._t_release) - float(self._t_accept)

    @property
    def commit_at(self) -> float:
        """The COMMIT instant on the perf clock — ``_t_release −
        commit_budget_s``, re-armed by every SLIP (plan § 2.6 rule 2).

        0.0 before :meth:`start`, and meaningless on the serial path, which
        never enters :data:`PHASE_COMMITTING`. Read by the node's record builder
        and by the pipeline tick; nothing gates on it outside the FSM."""
        return self._commit_at

    @property
    def commit_at_scheduled(self) -> float:
        """The commit instant as ORIGINALLY scheduled, before any slip — the
        datum :attr:`slip_s` is measured from and the origin the slip's
        ``catch_confirm_window_s`` bound is counted from."""
        return self._commit_at_sched

    @property
    def staged_at(self) -> float:
        """When this cycle entered :data:`PHASE_STAGED` (0.0 if it never did) —
        the record's ``staged_at_s``, so a corpus can measure the staged
        preamble against :func:`stage_budget_s` instead of assuming it."""
        return self._staged_at

    @property
    def committed(self) -> bool:
        """True once the COMMIT gate passed — i.e. once the announcement and the
        dispatch went out. THE S1′ predicate: at most one cycle may read True at
        any instant, and only that cycle may emit a hand-bearing action.

        Always False on a cycle that is not :attr:`staged`; the serial path's own
        commitment flags (``_throw_dispatched``, ``prepared``) are unchanged and
        still say what they always said."""
        return self._committed

    @property
    def slip_s(self) -> float:
        """Commit lateness: ``commit-time − scheduled commit``, in seconds.

        **Populated for real since Phase B4.** The COMMITTING gate writes it on
        every SLIP and once more at the commit itself, so the number the record
        carries is the lateness of the tick the announcement actually went out
        on. Plan § 2.6 rule 3, "slip is reported, not hidden": a slip the FSM
        absorbed silently is indistinguishable from a cadence that never
        slipped, and the whole point of scheduling the release absolutely is
        that lateness becomes measurable instead of being soaked up by the next
        cycle's lead. Phase C's bounded-slip policy is then a CONSUMER of a
        measured quantity rather than a new mechanism.

        0.0 for every serial cycle — there is no commit gate to be late for.

        Never negative: an EARLY commit does not exist (the gate is polled and
        can only be crossed at or after ``commit_at``)."""
        return self._commit_slip_s

    @property
    def commit_slips(self) -> int:
        """How many times the COMMIT gate re-armed before it resolved.

        :attr:`slip_s` says HOW LATE the commit was; this says HOW MANY
        ITERATIONS it took, and the two answer different questions. A single
        slip that cost 45 ms is one late iteration on a healthy loop; fourteen
        slips that cost 560 ms is a loop that never got a period inside its
        nominal one, which is a re-cut conversation about
        :data:`NODE_LOOP_PERIOD_S` rather than a bad sitting. 0 for every serial
        cycle and for a commit that passed on its first tick."""
        return self._commit_slips
