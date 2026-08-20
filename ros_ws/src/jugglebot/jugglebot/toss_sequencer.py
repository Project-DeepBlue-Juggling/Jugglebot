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
6. **SETTLING** — tracker ``CAUGHT`` for OUR announced ball within the confirm
   window past the scheduled landing ⇒ ``CAUGHT`` (``ACTION_STAY`` by default,
   ``ACTION_RECENTER`` iff ``stay_at_pose_on_caught`` is False — see
   ``_terminal_action``); otherwise ``MISSED_INFEASIBLE_<code>`` (only when NO
   catch target was ever accepted) or ``MISSED``.

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
from jugglebot.ball_possession import EVIDENCE_UNKNOWN

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

# ── Feedback phases (Toss.action feedback.phase — LOCKED strings) ──────────────
PHASE_CHECKING = 'CHECKING'
PHASE_POSITIONING = 'POSITIONING'
PHASE_PREPARING = 'PREPARING'
PHASE_THROWING = 'THROWING'
PHASE_BALL_IN_FLIGHT = 'BALL_IN_FLIGHT'
PHASE_CATCHING = 'CATCHING'
PHASE_SETTLING = 'SETTLING'

# ── Actions the node executes on the FSM's behalf ──────────────────────────────
ACTION_NONE = 'none'
ACTION_POSITION_PLATFORM = 'position_platform'  # go_to_pose to the catch pose (once)
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
MIN_TOSS_THROW_DELAY_S = 3.5         # CHECKING floor (REJECTED_CANT_MAKE_LEAD). No BB
                                     # 2.5 s lead floor here — this exists so the
                                     # positioning+prepare budget plus the event-delay
                                     # floor generically fit inside the delay; the
                                     # runtime guard (ABORTED_CANT_MAKE_RELEASE) is
                                     # the real enforcement.
MIN_THROW_EVENT_DELAY_S = 1.0        # floor on event_delay at dispatch. DERIVED, not
                                     # measured (Phase-5 T0 hard-confirms): prelude
                                     # smooth-move ≤ full stroke 0.758 s — the QUINTIC
                                     # T = √(Δ·QUINTIC_S2_MAX/A) the firmware actually
                                     # solves (Trajectory.h:257) over 9.9594 rev, the
                                     # derived stroke top since 2026-07-26 (was 9.858
                                     # ⇒ 0.754 s); matches the 0.68–1.05 s ascents
                                     # observed. NOT the 0.63 s a triangular profile
                                     # would give — that model understates this budget
                                     # by 0.13 s and this comment carried it until
                                     # 2026-07-26. Model:
                                     # hand_stroke.smooth_move_duration_s.
                                     # + throw windup 2·0.187/v ≈ 0.10–0.14 s
                                     # ⇒ 0.86–0.90 s against the 1.0 s floor, margin
                                     # 0.102 s. Conservative twice over: the THROWING
                                     # entry re-check that hand_parked (|pos| ≤ 0.5 rev)
                                     # caps the REACHABLE prelude at ~0.17 s. The
                                     # catch-side 0.3 s floor does NOT apply — that
                                     # assumes a hand already primed at top.
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
TOSS_RELEASE_GRACE_S = 0.5           # release evidence must appear within t_release +
                                     # this (mirrors reload's ANNOUNCEMENT_GRACE_S).
CATCH_CONFIRM_WINDOW_S = 0.7         # identical value+role to reload: CAUGHT within
                                     # this PAST the scheduled landing; also the
                                     # CATCHING phase-report threshold. Absorbs the
                                     # +0.115 s announced-early bias (fourth sitting).
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
    t = float(flight_time_s)
    vel = REACH_VEL_LIMIT_MMPS if vel_mmps is None else float(vel_mmps)
    acc = REACH_ACC_LIMIT_MMPS2 if acc_mmps2 is None else float(acc_mmps2)
    jerk = REACH_JERK_LIMIT_MMPS3 if jerk_mmps3 is None else float(jerk_mmps3)
    return min(vel * t / REACH_PEAK_VEL_FACTOR,
               acc * t * t / REACH_PEAK_ACC_FACTOR,
               jerk * t ** 3 / REACH_PEAK_JERK_FACTOR)

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
    throw_stroke_seen: bool = False   # node-latched after dispatch: hand telemetry
                                      # ascending-stroke signature (vel over threshold,
                                      # pos above seated). Sticky for the goal.
    ball_track_confirmed: bool = False  # latched announced-ball id has tracking ==
                                      # CONFIRMED on `balls` — physical airborne
                                      # evidence (status IN_FLIGHT is time-based and
                                      # proves nothing).
    ball_caught: bool = False         # tracker CAUGHT for the LATCHED id only,
                                      # plausibility-gated vs the NOMINATED catch point
    catch_error_mm: float = float('nan')  # hypot(ball xy − nominated landing xy) at the
                                      # CAUGHT tick; NaN otherwise
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
    tier: str = TIER_8A                         # config-resolved (JB_OP_TOSS_TIER)
    event_vel_mps: float = 0.0                  # 0 => computed from the Tier-8a vertical
                                                # closed form; the node normally passes
                                                # motion/toss_release's value in
    positioning_timeout_s: float = TOSS_POSITIONING_TIMEOUT_S
    release_grace_s: float = TOSS_RELEASE_GRACE_S
    catch_confirm_window_s: float = CATCH_CONFIRM_WINDOW_S
    min_throw_delay_s: float = MIN_TOSS_THROW_DELAY_S
    min_event_delay_s: float = MIN_THROW_EVENT_DELAY_S
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

    # ── internal state ──
    _phase: str = field(default=PHASE_CHECKING, init=False)
    _t_accept: float = field(default=0.0, init=False)
    _t_release: float = field(default=0.0, init=False)   # _t_accept + throw_delay (perf)
    _position_dispatched: bool = field(default=False, init=False)
    _position_result: Optional[tuple] = field(default=None, init=False)  # (ok, dur, code)
    _position_arrival_time: float = field(default=0.0, init=False)
    _positioning_deadline: float = field(default=0.0, init=False)
    _positioned: bool = field(default=False, init=False)  # move ACCEPTED (cleanup marker)
    _prepare_dispatched: bool = field(default=False, init=False)
    _prepare_result: Optional[bool] = field(default=None, init=False)
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
            # Tier-8a co-located vertical toss: launch is purely vertical, so
            # |launch| = vz = Δz/T + g·T/2 with Δz = cup plane − release plane
            # (the full ballistic inverse, matching motion/toss_release — NOT the
            # idealised g·T/2 magnitude).
            t = self.flight_time_s
            vz_mms = ((HAND_CATCH_OFFSET_MM - HAND_THROW_RELEASE_OFFSET_MM) / t
                      + GRAVITY_MMS2 * t / 2.0)
            self.event_vel_mps = vz_mms / 1000.0

    # ── discrete async events (from the node) ──────────────────────────────────

    def note_position_result(self, now: float, accepted: bool,
                             planned_duration_s: float, code: str = '') -> None:
        """Report the ``go_to_pose`` service outcome. ``now`` anchors the timed
        arrival (``now + planned_duration_s + settle pad``) — the service returns
        at plan-install, so the plan clock starts at the response, not at the tick
        that eventually consumes it. On accept the positioning deadline is
        extended to cover the arrival + the mocap verification window (it never
        shrinks). First result wins; ignored unless the move was dispatched."""
        if (not self._position_dispatched or self._finished
                or self._position_result is not None):
            return
        self._position_result = (bool(accepted), float(planned_duration_s), str(code))
        if accepted:
            self._positioned = True
            self._position_arrival_time = (
                now + float(planned_duration_s) + TOSS_POSITION_SETTLE_PAD_S)
            self._positioning_deadline = max(
                self._positioning_deadline,
                self._position_arrival_time + TOSS_POSITION_VERIFY_WINDOW_S)

    def note_prepare_result(self, ok: bool) -> None:
        """Report the ``ACTION_PREPARE_CATCH`` bundle outcome (== the
        ``trajectory/arm_catch`` raise result; the bundle's topic publishes cannot
        fail detectably). A failure aborts BEFORE the announcement and the throw —
        the arming-before-throw ordering."""
        if self._prepare_dispatched and not self._finished:
            self._prepare_result = bool(ok)

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
        self._phase = PHASE_CHECKING
        self._t_accept = now
        self._t_release = now + self.throw_delay_s

    def step(self, now: float, obs: TossObservations) -> TossDecision:
        if self._finished:
            # Terminal: replay the stored result (never None once finished) so a
            # stray extra step can't hand the consumer a None — and never re-runs
            # the terminal action.
            return TossDecision(self._phase, ACTION_NONE, True, self._result)

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
                return self._reject('CANT_MAKE_LEAD')
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
                if (displacement > self.max_displacement_mm
                        or displacement > reach_displacement_limit_mm(
                            self.flight_time_s,
                            vel_mmps=obs.leg_vel_limit_mmps or None,
                            acc_mmps2=obs.leg_acc_limit_mmps2 or None,
                            jerk_mmps3=obs.leg_jerk_limit_mmps3 or None)):
                    return self._reject('DISPLACEMENT')
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
                return self._reject('TILT_CLAMP')
            if not (TEENSY_MIN_EVENT_VEL_MPS <= self.event_vel_mps
                    <= TEENSY_MAX_EVENT_VEL_MPS):
                return self._reject('EVENT_VEL')
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
            if (abs(x) > self.workspace_xy_mm or abs(y) > self.workspace_xy_mm
                    or abs(z - TOSS_ACTIVE_Z_MM) > TOSS_Z_BAND_MM):
                return self._reject('WORKSPACE')
            if self.tier == TIER_8B:
                # The throw site A shares B's z (one nominated plane); its xy
                # gets the same planning-envelope bounds as B. Since A is the
                # platform's LIVE commanded xy (Phase E), this now also refuses
                # a toss commanded while the platform is parked OUTSIDE the
                # planning envelope — a real state a SpaceMouse/GUI session can
                # leave behind, and one where the pre-tilt POSITIONING move
                # would be planned from an unvalidated pose.
                ax, ay = self.throw_site_xy_mm
                if (abs(float(ax)) > self.workspace_xy_mm
                        or abs(float(ay)) > self.workspace_xy_mm):
                    return self._reject('WORKSPACE')
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
        return TossDecision(PHASE_POSITIONING, ACTION_POSITION_PLATFORM, False, None)

    def _step_positioning(self, now: float, obs: TossObservations) -> TossDecision:
        if self._position_result is None:
            if now >= self._positioning_deadline:
                return self._abort('POSITION_TIMEOUT')
            return TossDecision(PHASE_POSITIONING, ACTION_NONE, False, None)
        accepted, _planned_s, code = self._position_result
        if not accepted:
            # Nothing moved, nothing armed — a loud reject with no cleanup,
            # carrying the go_to_pose reject-ladder code (reload's REJECTED_BB
            # pattern).
            return self._reject('POSITION', code or 'NO_RESPONSE')
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
            return self._abort('PREPARE_FAILED')
        # Release-window guard — checked BEFORE the announcement goes out (an
        # announced-then-aborted toss leaves a phantom tracker expectation that
        # REJECTED_TRACK_ACTIVE would then refuse on until it expires), and again
        # on the post-announce tick (a stall between announce and dispatch must
        # not push event_delay under the stroke's windup floor).
        if self._t_release - now < self.min_event_delay_s:
            return self._abort('CANT_MAKE_RELEASE')
        if not self._announce_dispatched:
            # Armed confirmed → explicit ≥1-tick gap → announce: the armed edge
            # and the announcement travel on different topics with no cross-topic
            # ordering guarantee, and catch_coordinator drops announcement
            # pre-tilts that arrive unarmed. The announce action is emitted on a
            # LATER tick than the PREPARE bundle by construction.
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

    def _reject(self, code: str, message: str = '') -> TossDecision:
        outcome = 'REJECTED_{}'.format(code)
        if message:
            outcome = '{}({})'.format(outcome, message)
        return self._finish(TossResult(
            False, outcome, float('nan'), self._achieved_flight_s()))

    def _abort(self, code: str) -> TossDecision:
        return self._finish(TossResult(
            False, 'ABORTED_{}'.format(code), float('nan'),
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
        (``t_release − now`` at dispatch) and the cancel cutoff from it."""
        return self._t_release
