"""ROS2 node: the ball-ops coordinator — the BB→Jugglebot reload action (MVP goal 4),
the Jugglebot self-toss action (single-ball-toss plan, Phase 1) and the continuous
self-toss session (single-ball-toss plan, Phase F).

One node hosts ALL THREE ball-op actions (the filename stays for launch/setup
continuity).
They are merged deliberately, not for convenience: the actions share the hand,
the ``catch/armed`` latch, and the Teensy's last-writer-wins trajectory queue, so
mutual exclusion must be a single in-process ``_lock``-guarded check — a cross-node
busy mechanism has publish-latency TOCTOU windows in which two live sequences
double-own the hand (one action's prime/retract erases the other's armed stroke) and
fight over the latch (one side's disarm silently kills the other's catch). The merge
also keeps exactly one copy of the bag-probed telemetry-ladder thresholds and of the
executor discipline the ladders depend on.

RELOAD is a thin wrapper around the pure-Python :class:`reload_sequencer.ReloadSequencer`
FSM; TOSS wraps :class:`toss_sequencer.TossSequencer` the same way (see the toss
section further down). The RELOAD action OWNS the platform + hand for its duration
(reload-action-catch-latch plan) — but it never plans motion itself; it drives the
existing services:

  - platform motion is planned by ``trajectory_node``: the reactive catch path
    (``catch/dynamic_target`` → ``planner.build_catch``) actuates the platform while the
    ``trajectory/arm_catch`` catch-armed latch is RAISED, and ``trajectory/go_home``
    re-centers on terminal — both run in TRAJECTORY, the mode RELOAD runs within;
  - the hand is primed to top by this node the moment the sequence's preconditions
    pass (``smooth_move_hand`` on PRIME_HAND — at COMMAND time, never waiting for a
    tracked ball) and retracted on abort; ``catch_coordinator_node`` still fires the
    reactive catch stroke, gated on the catch-armed latch;
  - the throw goes through ``ball_butler_node`` (``bb/throw_at_target`` with the
    point-target extension), which enforces BB's own limits + loud rejections. The
    throw is sent LAST — after every Jugglebot-side arming step — because BB's
    firmware countdown has no abort opcode: an arming failure must never leave an
    unabortable throw heading at an unarmed robot.

So this node subscribes to the state the FSM reasons about (BB heartbeat, mocap
freshness, trajectory streaming, the levelling correction trajectory_node reports
holding, control mode, tracked-ball status, target feedback),
and executes the actions the FSM asks for: PRIME_HAND (smooth-move hand to top),
``bb/reload``, PREPARE (raise the latch, pre-throw), ``bb/throw_at_target``,
RECENTER (lower latch + go_home), SAFE_ABORT (retract hand + lower latch + go_home).
It runs within the operator's active mode (TRAJECTORY) and does NOT switch modes;
leaving that mode mid-sequence is the documented abort.

Exposed as ``jugglebot/reload`` (Reload.action): phase feedback, a structured outcome
result, and cancellation.

TOSS (``jugglebot/toss``, Toss.action) throws the ALREADY-SEATED ball to a nominated
catch state and catches it — Reload is the loader (operator sequence Reload → Toss →
Toss …). The choreography difference that shapes everything here: the toss is its own
thrower AND its own announcer, so every Jugglebot-side arming step (profiled
``go_to_pose`` pre-positioning with a timed arrival — plus a config-keyed mocap
cross-check, disabled by default (see _TOSS_MOCAP_BODY_PARAM) — the latch raise, the
self-``ThrowAnnouncement``) precedes the one hand-stroke dispatch
(``set_hand_traj_cmd`` traj_type=0), which is the sequence's LAST commitment and is
NEVER re-dispatched — an ambiguous ack may have armed the stroke, and a re-dispatch
would replace it (or, post-release, clobber the armed catch stroke) on the Teensy's
last-writer-wins queue. The existing catch pipeline (tracker correlation →
catch_coordinator → ``catch/dynamic_target``) closes the loop unchanged; the
``catch/prime_hold`` topic suppresses catch_coordinator's auto-prime paths from
PREPARE to terminal so the ball-laden hand is never carried up mid-toss.

TIER 8B (Phase 4, config ``toss_tier='8b'``) displaces the catch: pre-tilt at the
throw site A — **the platform's LIVE commanded xy**, read from
``trajectory/commanded_position`` (Phase E, 2026-07-29; it was a config site until
then) — with ``compute_release_state_tilted``'s swing-compensated pose as the
POSITIONING target, launch
tilt-aimed at the goal's B, and the platform's A→B reach is DEFERRED to the
scheduled release: the stock catch_coordinator announcement pre-tilt would
complete the A→B translate BEFORE the ball is released (its arrival clamps to
~now + 1 s while the toss announces ≥1 s pre-release) — aim destroyed, a moving
platform under a seated ball mid-windup — so this node raises the new
``catch/pretilt_hold`` gate for the goal's duration (catch_coordinator then
latches the announcement for its hand-arm window but publishes no platform
target) and itself publishes the ONE ``catch/dynamic_target`` at t_release,
built from the same ``CatchCoordinator.predicted_catch_command`` math with
arrival = the announced landing (lead = the flight time by construction).
``catch/pretilt_hold`` is raised on EVERY toss cycle since 2026-08-22 (census
E5) — it was 8b-only, then any-commanded-tilt, and is now unconditional: at the
cadence rungs CCN's pre-tilt arrival clamps to the landing ITSELF, so even a
level 8a would command a reach arriving at contact, and the next cycle's
announcement lands inside the previous ball's settle-hold window.

The reach envelope trajectory_node applies to those targets is centred on the
DECLARED catch point B (``catch/reach_center``, published on the
ACTION_PREPARE_CATCH tick — contract C-REACH-1,
``ros_ws/docs/catch_reach_envelope.md``), not on the pose held at arming. Before
that contract the envelope was captured at A, so the deferred A→B reach was
rejected WORKSPACE mid-flight for any displacement past 80 mm — observed 4/4 on
hardware (bag ``2026-07-27_16-07-30``) with the ball already airborne.

**Session chaining (Phase E).** A CAUGHT toss now ends in ``ACTION_STAY`` rather
than ``ACTION_RECENTER`` (``jugglebot_operational.toss_stay_at_pose_on_caught``):
the latch and holds come down but no go_home is issued, so the platform holds its
catch pose and the NEXT toss reads its throw site A from there — A → B → C with no
operator repositioning. Not-caught terminals keep SAFE_ABORT's go_home unchanged.
The seam this opens on the RELOAD path (its catch is hard-fixed at the workspace
centre with no pre-positioning, so an off-centre park would reject the incoming
BB ball mid-flight) is closed by the reload FSM's ``REJECTED_NOT_CENTERED``
precondition, which refuses BEFORE BB is asked to throw.

TOSS_CONTINUOUS (``jugglebot/toss_continuous``, TossContinuous.action, Phase F) is
the third action and adds **no capability**: it runs ``num_throws`` ordinary tosses
back to back with a configurable dwell, each one built and driven through the SAME
:meth:`_build_toss_cycle` / :meth:`_run_toss_cycle` pair the single ``Toss`` uses —
so there is exactly one copy of the cycle machinery and a session cannot drift from
a single toss. Its outer FSM is the pure-Python
:class:`toss_session.TossSessionSequencer`, which owns only *when* the next cycle
starts, *whether* it starts, and the per-cycle accounting. The session commands no
motion of its own; every commanded motion belongs to a cycle. Chaining works because
of two landed facts: the firmware catch stroke ends where a kind-0 throw stroke
starts (0 rev, ``Trajectory.h``), so no hand move is needed between cycles, and a
CAUGHT toss STAYs at its catch pose, so cycle N+1's throw site A is cycle N's catch
B for free. The centroid-vs-cup frame divergence — ``trajectory/commanded_position``
publishes the CENTROID, which sits a cup-swing outside B — is caught BEFORE anything
moves by :meth:`_predicted_chain_site_mm` (``REJECTED_CHAIN_UNREACHABLE``) instead of
letting a session throw one ball and then refuse cycle 2 with the platform parked
off-box. (Phase E's cap-edge chaining refusal itself dissolved 2026-08-14: the
shipped ``toss_workspace_xy_mm`` = 160 box clears the divergence at every valid B.)
"""

from __future__ import annotations

import json
import math
import os
import queue
import subprocess
import threading
import time
from datetime import datetime

import yaml

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

import numpy as np

from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import SetBool, Trigger
from jugglebot_interfaces.msg import (
    BallButlerHeartbeat,
    BallStateArray,
    DynamicTargetCommand,
    HandTelemetryMessage,
    RigidBodyPoses,
    TargetFeedback,
    ThrowAnnouncement,
    TrajectoryStatus,
)
from jugglebot_interfaces.srv import (
    BallButlerThrow,
    GetTiltReadingService,
    GoToPose,
    SetFloat,
    SetHandGains,
    SetHandTrajCmd,
)
from jugglebot_interfaces.action import Reload, Toss, TossContinuous
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3

import jugglebot.hardware_config as hw
from jugglebot.ball_possession import (
    EVIDENCE_EMPTY,
    EVIDENCE_SEATED,
    EVIDENCE_UNKNOWN,
    HandBallSensorSource,
    TrackerArrivalSource,
    describe as describe_possession,
    lateral_miss_mm,
    merge_possession,
)
from jugglebot.reload_sequencer import (
    ACTION_CALL_RELOAD,
    ACTION_PREPARE_CATCH,
    ACTION_PRIME_HAND,
    ACTION_RECENTER,
    ACTION_SAFE_ABORT,
    ACTION_SEND_THROW,
    BB_STATE_IDLE,
    PHASE_CHECKING as RELOAD_PHASE_CHECKING,
    PHASE_PREPARING as RELOAD_PHASE_PREPARING,
    ReloadObservations,
    ReloadResult,
    ReloadSequencer,
    compute_catch_point_mm,
)
from jugglebot.toss_sequencer import (
    ACTION_ANNOUNCE as TOSS_ACTION_ANNOUNCE,
    ACTION_DISPATCH_THROW as TOSS_ACTION_DISPATCH_THROW,
    ACTION_POSITION_PLATFORM as TOSS_ACTION_POSITION_PLATFORM,
    ACTION_PREPARE_CATCH as TOSS_ACTION_PREPARE_CATCH,
    ACTION_REACH_CATCH as TOSS_ACTION_REACH_CATCH,
    ACTION_RECENTER as TOSS_ACTION_RECENTER,
    ACTION_SAFE_ABORT as TOSS_ACTION_SAFE_ABORT,
    ACTION_STAY as TOSS_ACTION_STAY,
    FLIGHT_TIME_MAX_S as TOSS_FLIGHT_TIME_MAX_S,
    PHASE_CHECKING as TOSS_PHASE_CHECKING,
    PHASE_POSITIONING as TOSS_PHASE_POSITIONING,
    PHASE_PREPARING as TOSS_PHASE_PREPARING,
    PHASE_THROWING as TOSS_PHASE_THROWING,
    THROW_DISPATCH_AMBIGUOUS,
    THROW_DISPATCH_OK,
    THROW_DISPATCH_REJECTED,
    TIER_8B,
    TOSS_CANCEL_CUTOFF_S,
    TossObservations,
    TossResult,
    TossSequencer,
    min_throw_delay_for_release_s,
    pre_dispatch_budget_s,
)
from jugglebot.toss_session import (
    DEFAULT_SESSION_MISS_CLEANUP_S,
    GO_HOME_DURATION_S,
    ON_EMPTY_CUP_RELOAD,
    OUTCOME_STOPPED_RELOAD_BUDGET,
    SESSION_ACTION_NONE,
    SESSION_ACTION_RELOAD,
    SESSION_ACTION_START_CYCLE,
    SESSION_PHASE_CHECKING,
    SESSION_PHASE_DWELL,
    SESSION_PHASE_RELOAD,
    TossSessionResult,
    TossSessionSequencer,
    resolve_on_empty_cup,
)
from jugglebot.catch_coordinator import CatchCoordinator
from jugglebot import clock_offset, toss_record, toss_trim
from jugglebot.toss_record import latch_announced_ball
from jugglebot.motion.tilt_map import find_repo_root
from jugglebot.motion import toss_cal, toss_ilc
from jugglebot.motion.trajectory import hand_stroke, throw_envelope
from jugglebot.motion.trajectory.tilt_geometry import MAX_TILT_DEG
from jugglebot.motion.ik_solver import (
    quat_to_rot_matrix,
    rot_matrix_to_quat,
    rot_matrix_to_rotvec,
    rotvec_to_rot_matrix,
)
from jugglebot.motion.trajectory.toss_release import (
    ThrowTiltInfeasible,
    aim_target_offset_mm,
    apex_height_from_flight_time,
    build_announcement_fields,
    compute_release_state,
    compute_release_state_tilted,
    flight_time_from_height,
    validate_event_vel,
)

# BallStatus enum (BallState.msg): 0 = TO_BE_THROWN, 1 = IN_FLIGHT, 2 = CAUGHT.
_BALL_STATUS_TO_BE_THROWN = 0
_BALL_STATUS_IN_FLIGHT = 1
_BALL_STATUS_CAUGHT = 2
# TrackingConfidence enum (BallState.msg): 1 = CONFIRMED (mocap-matched — physical
# airborne evidence; status IN_FLIGHT alone is time-based and proves nothing).
_BALL_TRACKING_CONFIRMED = 1

# trajectory/target_feedback codes that are NOT catch infeasibility: FROZEN fires for
# every late catch/dynamic_target inside the reach-freeze window, STALE_STATE on a
# transient plant-state race — both expected on a real flight. Only a genuine gate reject
# (e.g. WORKSPACE / TOO_FAST) that still stands at catch time should count toward
# MISSED_INFEASIBLE.
_TRANSIENT_FEEDBACK_CODES = frozenset({'FROZEN', 'STALE_STATE'})

# Freshness windows (s).
_MOCAP_STALE_S = 0.5
_HEARTBEAT_STALE_S = 0.5
_STATUS_STALE_S = 0.5
# trajectory/status freshness, used ONLY by the toss's platform_levelled
# observation (`streaming` keeps its historical sticky last-value semantics —
# the reload FSM consumes it too, and changing when a reload refuses is not this
# phase's to change). Deliberately NOT the 0.5 s above: those windows are sized
# for the 100-160 Hz mocap/hand/ball streams, and trajectory/status is 5 Hz, so
# 0.5 s is only 2.5 periods. Measured inter-arrival over 420 s of recorded
# self-toss sessions (bags 2026-07-25_15-17-48 and _15-22-50, probe run
# 2026-07-26, re-measured at finalize): median 200.0 ms, p99 210 ms, MAX
# 508.5 ms — one gap already past 0.5 s, and four past 0.3 s (two per bag:
# 384.5/508.5 ms and 334.5/426.2 ms). A 0.5 s window would therefore mint an
# occasional spurious REJECTED_NOT_LEVELLED on a healthy machine, and a gate
# that cries wolf gets ignored. 1.0 s is five periods and ~2x the worst gap,
# while still catching a trajectory_node that has genuinely stopped talking
# within one second — well inside any real restart, which is the window in
# which the cached flag would otherwise be a dead process's answer.
_TRAJ_STATUS_STALE_S = 1.0

# CAUGHT plausibility gate — contract C-POSSESS-1, ros_ws/docs/ball_possession_contract.md.
# The tracker's split-track corruption (2026-07-23 re-test) flips CAUGHT on tracks
# coasting BELOW THE FLOOR near BB — 5 of 6 reloads reported spurious CAUGHT while
# the real ball bounced out. A caught ball confirms OUR catch only if its final KF
# estimate is physically near the catch point, HORIZONTALLY.
#
# The horizontal qualifier is the whole 2026-07-28 change and it is not cosmetic.
# The bound this replaced also carried `z_err <= 150 mm`, which is unsatisfiable by a
# real catch: the tracker declares CAUGHT *because the marker vanished*, so the
# published position is a dead-reckoned free-fall extrapolation and all of its error
# lands in z. Measured across the 2026-07-27 sitting's 17 self-tosses — every one a
# catch the operator watched — z error 305-1007 mm against xy error 0.30-3.88 mm.
# `success` was therefore False by construction on every ball op ever run. Full error
# model + the measured distributions: jugglebot/ball_possession.py's module docstring.
#
# The tolerance is the catching structure's own entry aperture, single-sourced from
# the machine geometry rather than a magic number: a ball whose estimate sits further
# from the cup axis than the opening it must have passed through did not enter the
# structure. Margins against the same session: 18x above the true-catch maximum
# (3.88 mm) and 2.9x below the corrupt-track floor (204.9 mm). The 200.0 it replaces
# cleared that floor by 4.9 mm — a 1.02x margin against minting a FALSE CAUGHT, which
# is the second instance of the same "bound without an error model" defect and the
# reason C-POSSESS-1 closes the class instead of the one number.
_CAUGHT_MAX_XY_ERROR_MM = float(hw.GEOM_ARM_RADIUS_MM)

# Reject-fast service-call bounds.
_SERVICE_WAIT_S = 2.0
# Hand dispatch retry ladder (prime + SAFE_ABORT retract). The HAND_TRAJ_CMD path
# failed ~40-60% PER CALL across the 2026-07-23 third sitting (firmware-replied
# 'ERR_TIMEOUT' CAN-enqueue failures on the bridge Teensy; root cause under
# investigation — suspected CAN3 TX contention with the 500 Hz leg stream). The
# failures are fast error replies and an immediate retry succeeded 9/10 times, so
# a short ladder drops the goal-abort probability from ~16% (2 attempts) to ~3%
# (4 attempts) at the ~40% end of the observed failure band. Pre-throw, so the
# extra worst case is free.
_HAND_DISPATCH_ATTEMPTS = 4
# Telemetry-verified re-dispatch: the ack LIES (frames were observed transmitted with
# the hand moving after a failed 'ERR_TIMEOUT' ack). A blind re-dispatch on a lied ack
# rebuilds the min-jerk profile from the live mid-move position at v=0, yanking the
# moving hand backwards — the fourth sitting's "hand jump up-down then abort"
# (2026-07-24: a full 4/4 lying-ack ladder restarting a live ascent -> false
# ABORTED_PRIME_FAILED while the hand physically primed). So after a failed
# ack, settle briefly then consult hand_telemetry: only re-dispatch if the hand is
# positively stationary away from the target (frame genuinely lost) or telemetry is
# absent (fall back to today's blind ladder — a strict superset, never worse).
# Thresholds probed against the recorded 2026-07-24_09-07-53 /hand_telemetry (44041
# msgs @ 100 Hz; /tmp probe, results in the fourth-sitting logbook entry): the abort
# ladder's four lied-ack dispatches all read +9.2..+22.5 rev/s at dispatch+0.25 s
# (monitor stops the ladder at attempt 1); parked-hand |vel| noise p99 = 1.82 rev/s
# (bottom) / 5.39 rev/s (top — hence the park-band position qualifiers in
# _hand_dispatch_confirmed); parked-top pos spread [9.675, 10.044] sits inside the
# ±0.5 near-band around 9.858; telemetry gaps median 10 ms / max 39.6 ms << 0.3 s.
# 2026-07-26: the prime became the derived stroke top (9.858 -> 9.9594 rev, 3.2 mm
# up), so the near-band is now [9.4594, 10.4594]. The spread above was measured
# with the prime COMMANDED to 9.858 and translates up with the target, restoring
# the 0.317/0.314 rev margins; even read pessimistically as if the hand stayed at
# the old top it keeps 0.2156 rev = 6.8 mm of clearance on the lower edge. Pinned
# by tests/ros/test_reload_coordinator_node.py::
# test_prime_move_leaves_the_park_band_windows_open.
_HAND_ACK_SETTLE_S = 0.25          # let a lied-ack move begin before reading telemetry
_HAND_NEAR_TARGET_REV = hand_stroke.HAND_PARK_BAND_REV
                                   # |pos - target| within this ⇒ already at the
                                   # target. Sourced from hand_stroke since
                                   # 2026-08-22: the same band now sizes the kind-0
                                   # dispatch's PRELUDE BUDGET
                                   # (min_throw_event_delay_s), so the gate that
                                   # admits a throw and the arithmetic that budgets
                                   # its prelude must be one number, not two.
_HAND_MOVING_VEL_RPS = 2.0         # |vel| toward the target at/above this ⇒ move underway
_HAND_TELEMETRY_STALE_S = 0.3      # telemetry older than this ⇒ cannot verify ⇒ blind ladder
# Sequence loop tick (the FSM is time-driven; this bounds latency, not correctness).
#
# 0.05 -> 0.02 on 2026-08-22 (census B3). The reason is cadence, not latency for
# its own sake: at the tuning-phase 0.49 s dwell a 0.05 s tick is 10% of the
# whole turnaround spent in sleep(), and the PREPARE ladder alone costs four of
# them. 0.02 s is still ~2 orders of magnitude above localhost topic latency, so
# the two load-bearing TICK-COUNTED ordering gaps keep their guarantee:
#   (a) catch/prime_hold must land in an EARLIER catch_coordinator wait-set cycle
#       than the catch/armed edge, or the armed-edge auto-prime ascends with the
#       seated ball;
#   (b) armed-confirm -> >=1 tick -> announce, because catch_coordinator drops
#       pre-tilts that arrive unarmed.
# THE TICK COUNTS ARE UNCHANGED, deliberately. Collapsing them is the change that
# would break (a) and (b); shortening the tick is not.
#
# Every use is a POLL PERIOD in a goal-execution thread (verified by inspection,
# 2026-08-22: seven time.sleep(_TICK_S) sites, all inside `while rclpy.ok()`
# deadline loops). None of them is a control loop and none treats the tick as a
# DURATION, so the cost of shortening it is CPU in a thread that was sleeping.
# toss_session.NODE_TICK_S mirrors this and is pinned to it by a drift-guard test
# — it is an input to DEFAULT_SESSION_DWELL_MARGIN_S and to the MISS-cleanup
# floor, so the two cannot be allowed to drift.
_TICK_S = 0.02
# A hard ceiling on a single reload attempt so a wedged sequence always terminates.
_MAX_SEQUENCE_S = 30.0
# Slack added on top of the FSM's own budgets when deriving the per-goal ceiling.
_SEQUENCE_CEILING_MARGIN_S = 5.0

# ── Toss-only constants ────────────────────────────────────────────────────────
# Observation-level hand-telemetry freshness for the TOSS preconditions
# (REJECTED_HAND_STALE) and the parked-band read — aligned with the other 0.5 s
# precondition windows above, NOT the tighter 0.3 s in-ladder staleness bound
# (_HAND_TELEMETRY_STALE_S), which stays reserved for mid-ladder verification.
_HAND_STATE_STALE_S = 0.5
# Release-stroke signature threshold: hand vel_meas at/above this with the hand out
# of the bottom park band reads as the throw stroke underway. Sits ABOVE the kind-1
# smooth-move prelude's peak ascent — so a concurrent prime/retract ascent can never
# fake release evidence. TWO figures, deliberately, because they are not
# interchangeable (hand_stroke.smooth_move_peak_vel_rps / _bound_rps):
#   * COMMANDED peak, what the firmware's quintic actually emits over the full
#     9.9594 rev stroke: |Δ|·1.875/T = 24.63 rev/s (was 24.50 at 9.858). This is
#     the figure a BENCH capture is scored against.
#   * BOUND, √(a·Δx) = 31.56 rev/s (was 31.40), the peak a bang-bang profile over
#     the same stroke would reach. This threshold clears the BOUND, which is the
#     conservative thing for a guard to do — but the bound is NOT the peak, and
#     this comment called it "the peak ascent speed" until 2026-07-26.
# and well BELOW the ≈85 rev/s minimum
# release-plane stroke speed at the 2.7 m/s sweep floor. DERIVED, not probed
# against a recorded throw stroke yet; the tracker CONFIRMED channel is the
# independent backstop, and Phase-5 T0 re-tunes this against measured stroke
# telemetry.
_THROW_STROKE_VEL_RPS = 40.0
# Mocap arrival cross-check tolerance for POSITIONING (ABORTED_POSITION_FAILED
# beyond it). JB_TRAJ_CATCH_REACH_ENVELOPE_MM is the semantically-right existing
# constant: a platform measured within one envelope radius of its nominated
# pre-position pose still has the nominated catch point inside the armed envelope,
# so the catch can reach it; beyond it the catch cannot. (A full silent no-op from
# a corner pose fails this; a near-centre no-op passes — and degrades gracefully.)
# Under contract C-REACH-1 the envelope is centred on the declared catch B rather
# than on the commanded pose, which does NOT change this argument: the measurand
# is still "how far the platform is from where the catch was planned from", and
# for 8b the un-arrived pre-tilt independently corrupts the throw AIM, which the
# envelope never protected against anyway.
_TOSS_POSITION_TOL_MM = float(hw.JB_TRAJ_CATCH_REACH_ENVELOPE_MM)
# Per-axis bound on "the platform is ALREADY at the positioning pose, so command
# nothing" (census B1, _toss_already_positioned). HALF the cup radius.
#
# It is NOT the envelope above and NOT _RELOAD_CENTERED_TOL_MM below, and the
# difference is the point. Those two bound how far a platform may be from where a
# catch was PLANNED from and still reach; this one bounds how far it may be
# before a declared no-op becomes a lie. For a co-located Tier-8a toss a residual
# is aim-NEUTRAL (throw site == catch site: the ball comes back down into the cup
# wherever the cup is), so what it actually bounds is the error in everything
# downstream that was told the ball will be at the NOMINATED B — the declared
# reach centre, the announcement the tracker correlates against, and the mocap
# cross-check target. Half a cup radius is a shift the cup absorbs with 2x margin.
#
# Deliberately tight rather than "as loose as still works": this constant's only
# effect is to make the machine do LESS, so an over-wide value trades a silent
# aim error for 0.45 s of cadence — the wrong side of that trade for a gate whose
# failure mode is a throw fired from a site nobody solved for.
_TOSS_ALREADY_THERE_TOL_MM = float(hw.GEOM_HAND_RADIUS_MM) / 2.0
# The ORIENTATION half of the same gate (2026-08-23), per rotvec component.
#
# DERIVED from the position bound above rather than picked, so the two halves
# admit the same physical error: a residual platform tilt δθ tilts the release
# velocity by δθ and displaces the landing by 4·h·sin(δθ) — the identical
# expression the CHECKING levelling gate is sized on
# (toss_sequencer._step_checking, REJECTED_NOT_LEVELLED). Setting that
# displacement equal to _TOSS_ALREADY_THERE_TOL_MM and solving at the LARGEST
# apex the C-HAND-3 band admits (fail-closed — the drift is linear in apex, so
# the tallest throw is the strictest case) gives δθ.
#
# 2.71 mrad (0.155°) as shipped. Two sanity readings on that number:
#
#   * it is 1/6.4 of the ILC's own ±1.0° aim authority, so an ARMED aim can
#     never be mistaken for a level platform — which is the wrong "yes" that
#     would fire an ILC throw with the correction not applied;
#   * on a CHAINED cycle the residual is ~1e-15 rad: the platform is holding the
#     pose the previous cycle commanded, and this cycle recomputes the identical
#     pre-tilt from the identical (catch_pose, flight, aim). The tolerance is not
#     what makes the skip fire; it is what stops it firing on anything else.
_TOSS_ALREADY_THERE_TOL_RAD = (
    (_TOSS_ALREADY_THERE_TOL_MM / 1000.0)
    / (4.0 * apex_height_from_flight_time(float(TOSS_FLIGHT_TIME_MAX_S))))
# RELOAD centred-ness tolerance (REJECTED_NOT_CENTERED, _platform_centered).
#
# NOT the bare envelope radius, and the difference is the whole point of the
# gate. The reload declares no reach centre (contract C-REACH-1), so its arm
# raise centres the envelope on the PARK. The target that envelope then judges
# is NOT the catch point: catch_coordinator pre-tilts to receive, and
# _compute_catch_command pulls the centroid back along the tilted platform-z by
# hand_catch_offset_mm — so the commanded pre-tilt pose sits
# hand_catch_offset_mm·sin(tilt) laterally off the catch point. Real BB arrivals
# are 18-40° off vertical and compute_catch_orientation CLAMPS at
# MAX_TILT_DEG = 12°, so that shift SATURATES at 64.78·sin(12°) = 13.47 mm on
# every reload (measured across 18/25/30/40° arrivals: 13.469 mm, invariant).
#
# With the tolerance set to the envelope radius there is ZERO budget for that
# systematic shift, so parks in (80 − 13.47, 80] mm are ADMITTED by this gate
# and then REJECTED WORKSPACE by the envelope — after ACTION_SEND_THROW, with
# BB's countdown started and the ball unsavable. That is precisely the
# mid-flight rejection of a real BB throw the gate was added to make
# impossible, reached through a gate that said yes. Subtracting the shift
# closes the band by construction: an admitted park + the worst-case pre-tilt
# swing still lands inside the envelope.
#
# Costs nothing in normal operation (a go_home'd platform is at 0 mm) — it only
# refuses the band that was already doomed, and a refusal moves nothing.
_RELOAD_CENTERED_TOL_MM = (
    float(hw.JB_TRAJ_CATCH_REACH_ENVELOPE_MM)
    - float(hw.HAND_CATCH_OFFSET_MM) * math.sin(math.radians(MAX_TILT_DEG)))

# ── The auto-reload interlude (TossContinuous on_empty_cup: RELOAD, § 3.9) ────
# How long past the go_home PROFILE the interlude keeps looking for a fresh,
# centred trajectory/commanded_position before minting STOPPED_RECENTRE_FAILED.
#
# The profile itself is deterministic and does NOT stretch: planner.
# build_return_to_neutral takes `dur = max(go_home_duration_s, min_move_duration_s)`
# = max(2.0, 0.20) = 2.0 s, and an infeasible move is refused at the service call
# (so _go_home() returns False and we never get here). The only unmodelled terms
# are the ack→install latency and the publish period of the channel we read.
#
# MEASURED (2026-08-11, five bags 2026-08-10_12-06-49 / _15-16-03 / _16-04-26 /
# _16-13-48 / _16-30-44, 15,409 inter-sample gaps on /trajectory/status — the
# SAME 5 Hz trajectory_node timer that publishes /trajectory/commanded_position):
# median 0.200 s, p99 <= 0.252 s, worst 0.643 s. So 1.5 s is 2.3x the worst
# observed publish gap and 1.5x the _TRAJ_STATUS_STALE_S freshness window the
# read itself applies. Erring long is free: by then the platform is parked and
# the only cost of a longer window is time on a path that is already failing.
_RECENTRE_VERIFY_PAD_S = 1.5

# ── Layer 1.5 — the dwell inclinometer covariate (§ 3.10, operator decision 2) ─
# get_platform_tilt BLOCKS the Platform-Teensy loop that streams hand moves, so
# the two hard rules are, in priority order: reads NEVER overlap PREPARE→THROW,
# and a tight dwell degrades the READ COUNT rather than the throw's schedule.
#
# The client-side timeout deliberately ABANDONS a read that needs the bridge's
# retry ladder: teensy_bridge_node runs _TILT_READ_ATTEMPTS (3) x
# _RELAY_READ_TIMEOUT_S (0.5) = up to 1.5 s, which does not fit in a quiescent
# dwell that is itself ~0.7-3 s. A healthy read is one relay round-trip (CAN,
# milliseconds), so 0.30 s passes every healthy read and refuses to let a sick
# one eat the window. The covariate has ZERO control authority, so an abandoned
# read costs a data point and nothing else.
_DWELL_TILT_READ_TIMEOUT_S = 0.30
# Slack reserved between the last possible read and the next cycle's start.
_DWELL_TILT_GUARD_S = 0.20

# The ONE identifiable BB-side abort the interlude retries within budget: the
# firmware's BallButlerCommandOutcome.THROW_ABORTED_NOT_SETTLED (41) — BB was not
# positioned in time, so no ball ever left it (observed twice on hardware: the
# 2026-07-23 and 2026-07-24 sittings, both `axis=YAW`). bb/throw_at_target is
# FIRE-AND-FORGET — ball_butler_node publishes the announcement and returns
# success BEFORE the firmware's terminal CMD_RESULT exists — so this code reaches
# us only on the bb/throw_outcome topic, and a reload that hit it otherwise looks
# exactly like an ordinary MISSED. Retrying the whole not-caught class instead
# would swallow the BB fail-open boot bug and every real BB fault; that is why
# the retry is keyed on this string and why the log line names it.
_BB_NOT_SETTLED_CODE = 'THROW_ABORTED_NOT_SETTLED'
# How recent a bb/throw_outcome must be to describe THIS reload attempt. The
# reload FSM's own budget from throw to terminal is throw_delay (>= 2.5 s) +
# announcement grace + confirm window, so anything older than the sequence
# ceiling belongs to a previous attempt and must not license a retry.
_BB_THROW_OUTCOME_STALE_S = float(_MAX_SEQUENCE_S)
# The soft catch gains the toss sets itself at PREPARE. catch/prime_hold suppresses
# catch_coordinator's auto-prime — which is the ONLY place soft catch gains are
# normally set (_prime_hand → _set_catch_gains) — so without this the toss's catch
# would run on whatever gains the last action left. Values MIRROR
# catch_coordinator_node._catch_hand_gains (pos 20 vs default 35 for impact
# compliance; vel gains at the ODrive defaults) and are pinned equal by test.
# The Tier-8a throw therefore also runs on soft gains — a deliberate, T0-measurable
# choice; no mid-flight gain switching (a mid-flight CAN config write is a new
# failure mode with no evidence behind it).
_TOSS_SOFT_CATCH_GAINS = {
    'pos_gain': 20.0,
    'vel_gain': float(hw.ODRIVE_HAND_VEL_GAIN),
    'vel_integrator_gain': float(hw.ODRIVE_HAND_VEL_INT_GAIN),
}
# teensy_bridge_node._svc_set_hand_traj raises these ValueErrors BEFORE any CAN
# frame exists — an ack failure carrying one of them is a DEFINITIVE no-arm reject
# (everything else is ambiguous: the ERR_TIMEOUT class may have transmitted).
# String-coupled to the bridge's literals; the classification test drives the
# bridge's real validation path so drift breaks the test, never silently.
_BRIDGE_HAND_TRAJ_REJECT_PREFIXES = (
    'Invalid event delay', 'Invalid event velocity', 'Invalid trajectory type')
# Trace-only ball-evidence waiver (single-ball-toss plan, Phase-3 dry trace): a node
# PARAMETER, never a goal field. Waives ONLY the possession-latch precondition
# (REJECTED_NO_BALL) so a POWERED no-ball bench can emit the full post-CHECKING
# choreography for trace review: with power up, every CHECKING precondition passes
# naturally except possession. (An UNPOWERED bench never reaches this gate at all —
# activation faults leave control_mode non-TRAJECTORY and hand telemetry silent, so
# CHECKING dies REJECTED_WRONG_MODE / REJECTED_HAND_STALE first; see
# logbook/2026-07-25-toss-phase3-prep-trace-harness.md.) The hand-parked and
# hand-fresh preconditions stay hard even under it. The Phase-3 trace runbook is
# the ONE sanctioned setter; every ball-flying runbook leaves it unset.
_TOSS_WAIVER_PARAM = 'toss_ball_evidence_waiver_trace_only'
# QTM rigid-body name for the POSITIONING mocap arrival cross-check — a node
# PARAMETER, DEFAULT '' = cross-check DISABLED. No QTM body for the platform has
# ever been validated live (the known bodies are Base, Ball_Butler, Catching_Cone),
# so the shipped default trusts the go_to_pose accept + planned-duration wait (+
# the '[wire DISARMED' response mapping) and the node feeds platform_at_target
# True at every tick. When a body IS named, its position is compared in the
# STOW/platform_start frame (non-base bodies publish z shifted by the base→
# platform transform in mocap_interface, so a platform body at ACTIVE reads
# z ≈ 170) against the nominated pose UNCONVERTED — converting either side to
# global double-adds GEOM_INITIAL_HEIGHT_MM. Resolve the real body name + frame
# on the bench (Phase-3 dry trace / T0) before enabling.
_TOSS_MOCAP_BODY_PARAM = 'toss_mocap_body'
# Layer-2 session trim (design § 3.6, phase 2e) — a node PARAMETER, DEFAULT
# FALSE. The estimator runs regardless; this decides only whether its output is
# ADDED to the commanded aim. Rollback is `ros2 param set … false` and takes
# effect at the NEXT goal build, never mid-cycle: the trim is frozen into the
# release state at _build_toss_cycle, which is what makes "disable the trim" a
# statement about the next throw rather than a race with the one in the air.
_TOSS_TRIM_PARAM = 'toss_trim_enabled'
# Toss terminals on which the platform's true motion state is UNKNOWN: an
# ack-timed-out go_to_pose may still execute (the service returns at plan-INSTALL,
# so a missing/late ack does not mean no motion — the plan may be running or land
# later). The FSM's terminal action there is ACTION_NONE (nothing verifiably
# accepted ⇒ no full SAFE_ABORT), so the node dispatches a best-effort go_home to
# supersede any zombie move — go_home replaces the installed trajectory by design.
_TOSS_POSITION_UNKNOWN_TERMINALS = frozenset({
    'REJECTED_POSITION(NO_RESPONSE)', 'ABORTED_POSITION_TIMEOUT'})


#: Best-effort JSONL belt for the per-toss record (toss-selftuning § 3.3). The
#: bag is canonical; this exists ONLY for the ``record:=false`` bench session.
#:
#: Resolved with tilt_map's MARKER walk, never a fixed number of ``dirname``
#: hops: this module runs from the colcon install tree in production and from
#: ``ros_ws/src`` under pytest, and those are different depths. A fixed walk is
#: the tilt-cal Phase-2 finding — it produced a path that could not exist in
#: production while looking perfectly reasonable in the source tree. ``None``
#: for a deployment outside the repo, where the belt simply does not run.
_REPO_ROOT = find_repo_root(__file__)
_RECORD_BELT_DIR = (os.path.join(_REPO_ROOT, 'temp', 'logs')
                    if _REPO_ROOT else None)


def _goal_id_hex(goal_handle) -> str:
    """The action goal uuid as hex, or '' when the handle cannot supply one.

    Guarded because the record must survive a test double / a partially-built
    handle: a provenance field is never worth an exception on the terminal path.
    """
    try:
        return bytes(goal_handle.goal_id.uuid).hex()
    except Exception:                                          # noqa: BLE001
        return ''


def _repo_revision():
    """-> ``(sha, dirty)`` for the running tree, or ``(None, None)``.

    Called ONCE at node construction, never on a teardown path. Provenance the
    corpus cannot reconstruct any other way: a bag records what the machine did,
    not which code did it, and ``toss_cal_fit.py``'s partition census is only
    honest if every record names its build. Fully guarded — a node that refuses
    to start because ``git`` is missing would be a spectacular own goal for a
    field that is pure metadata.
    """
    cwd = os.path.dirname(os.path.abspath(__file__))
    try:
        sha = subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=cwd,
            stderr=subprocess.DEVNULL, timeout=5.0).decode().strip()
        dirty = bool(subprocess.check_output(
            ['git', 'status', '--porcelain'], cwd=cwd,
            stderr=subprocess.DEVNULL, timeout=5.0).decode().strip())
        return sha, dirty
    except Exception:                                          # noqa: BLE001
        return None, None


def _toss_deadline_s(seq) -> float:
    """The node-level hard ceiling for THIS toss goal — same doctrine as
    :func:`_sequence_deadline_s`: never below _MAX_SEQUENCE_S and never inside a
    legitimate sequence window (positioning + delay + flight + grace + confirm),
    because the timeout path SAFE_ABORTs (retract under an airborne ball)."""
    budget = (float(seq.positioning_timeout_s) + float(seq.throw_delay_s)
              + float(seq.flight_time_s) + float(seq.release_grace_s)
              + float(seq.catch_confirm_window_s) + _SEQUENCE_CEILING_MARGIN_S)
    return max(_MAX_SEQUENCE_S, budget)


def _reload_interlude_budget_s(session) -> float:
    """Wall time the whole auto-reload allowance can legitimately consume.

    Per attempt: the seat-edge band wait + the recentre (profile + verification
    window) + a full reload sequence ceiling + the post-reload settle floor.
    Multiplied by the session's reload budget, because every attempt is charged
    to it. Zero when the session cannot reload at all, so a STOP session's
    ceiling is bit-unchanged.

    The band wait (:meth:`ReloadCoordinatorNode._wait_out_seat_edge_band`,
    C-POSSESS-1 § 3.6) is charged PER ATTEMPT even though it is spent once per
    INTERLUDE, in the gate, before the attempt loop. That over-counts by design —
    the sibling ceiling's own docstring states the doctrine ("over-counting is the
    safe direction here"), and the direction that matters is the other one: this
    ceiling's exit path is an ABORT, and a ceiling that drifts INSIDE a legitimate
    window SAFE_ABORTs a session for doing exactly what it was asked to do."""
    if str(getattr(session, 'on_empty_cup', '')) != ON_EMPTY_CUP_RELOAD:
        return 0.0
    per_attempt = (float(hw.JB_BD_ARRIVAL_WINDOW_S)
                   + GO_HOME_DURATION_S + _RECENTRE_VERIFY_PAD_S
                   + _sequence_deadline_s(ReloadSequencer(catch_point_mm=(0.0,
                                                                         0.0,
                                                                         0.0)))
                   + DEFAULT_SESSION_MISS_CLEANUP_S)
    return max(0, int(getattr(session, 'max_reloads', 0))) * per_attempt


def _toss_session_deadline_s(session, cycle_budget_s: float,
                             reload_budget_s: float = 0.0) -> float:
    """The node-level hard ceiling for a whole TossContinuous SESSION — same
    doctrine as :func:`_toss_deadline_s`, applied one level up: never below
    ``_MAX_SEQUENCE_S`` and never inside a legitimate session window, because the
    session timeout path SAFE_ABORTs and a SAFE_ABORT under an airborne ball
    retracts the hand into it.

    Deliberately GENEROUS: it is a backstop of last resort, not the working
    guard. Each cycle already carries its own :func:`_toss_deadline_s` ceiling
    inside :meth:`_run_toss_cycle`, so a wedge INSIDE a cycle is caught in ~30 s;
    this only bounds a wedge in the outer loop, whose only wait is time-based.
    Budget per cycle = the cycle's own ceiling + a full dwell (the quiescent wait
    before it), which over-counts because the cycle ceiling already contains
    ``throw_delay`` — over-counting is the safe direction here.

    ``reload_budget_s`` adds the auto-reload interludes a session may legitimately
    spend (:func:`_reload_interlude_budget_s`). Without it an on_empty_cup RELOAD
    session that actually uses its budget would trip the session ceiling for doing
    exactly what it was asked to do — and this ceiling's exit path is an ABORT."""
    per_cycle = float(cycle_budget_s) + float(session.dwell_time_s)
    return max(_MAX_SEQUENCE_S,
               int(session.num_throws) * per_cycle + float(reload_budget_s)
               + _SEQUENCE_CEILING_MARGIN_S)


def _sequence_deadline_s(seq) -> float:
    """The node-level hard ceiling for THIS goal. Never below _MAX_SEQUENCE_S, and
    never inside a legitimate sequence window: the FSM's own budgets (empty-hand
    reload wait + throw countdown + announcement grace + catch confirm) plus margin.
    Without this, an operator-supplied throw_delay_s ≳ 16 s put the fixed 30 s
    timeout INSIDE the flight window — and the timeout path SAFE_ABORTs, retracting
    the hand under the airborne ball (the exact hazard class the settle-resolution
    fix closed)."""
    budget = (float(seq.reload_timeout_s) + float(seq.throw_delay_s)
              + float(seq.announcement_grace_s) + float(seq.catch_confirm_window_s)
              + _SEQUENCE_CEILING_MARGIN_S)
    return max(_MAX_SEQUENCE_S, budget)


class ReloadCoordinatorNode(Node):
    def __init__(self, robot_name: str = 'jugglebot'):
        super().__init__('reload_coordinator_node')
        self._robot_name = robot_name
        # The ball is caught by the hand CUP, which sits HAND_CATCH_OFFSET_MM (64.78 mm)
        # above the platform centroid at the catch position within the hand stroke — not
        # at the centroid itself. Aim BB at the cup plane so the ball is delivered where
        # the hand actually intercepts it. This matches the catch plane the rest of the
        # stack already uses (throw_ballistics._DEFAULT_CATCH_HEIGHT_MM, ball_tracker's
        # landing_z, catch_coordinator's landing_z_offset) — all GEOM_INITIAL_HEIGHT +
        # ACTIVE_Z + HAND_CATCH_OFFSET. Omitting the offset aimed 64.78 mm low, pushing
        # the ball's true-plane crossing off-centre (toward BB) and eating tilt/reach
        # margin on every catch.
        self._catch_point_mm = compute_catch_point_mm(
            hw.GEOM_INITIAL_HEIGHT_MM, hw.JB_OP_DEFAULT_ACTIVE_Z_MM,
            landing_z_offset_mm=hw.HAND_CATCH_OFFSET_MM)

        # ── Cached observations (updated by subscription callbacks) ──
        self._lock = threading.Lock()
        self._hb = None
        self._hb_mono = 0.0
        self._control_mode = ''
        self._streaming = False
        # trajectory/status arrival stamp + the levelling-correction affirmation
        # it carries (contract C-LEVEL-1, ros_ws/docs/levelling_frame.md). The
        # stamp exists because the flag is only meaningful while the publishing
        # node is alive: a cached True from a trajectory_node that has since
        # restarted is the false assurance the toss's REJECTED_NOT_LEVELLED gate
        # exists to prevent. 0.0 = never heard from it ⇒ fail closed.
        self._traj_status_mono = 0.0
        self._gravity_correction_loaded = False
        # LIVE session leg limits (vel mm/s, acc mm/s², jerk mm/s³) as last
        # published on trajectory/status. (0,0,0) = never heard / pre-field
        # publisher — consumers fall back to the YAML-default copies.
        self._leg_limits_live = (0.0, 0.0, 0.0)
        # Record-only provenance mirrors of the same message (never gated on).
        self._tilt_map_loaded = False
        self._tilt_map_version = ''
        # ── Toss AIM calibration (contract C-TOSS-CAL-1, motion/toss_cal.py) ──
        # This node OWNS the aim map (operator decision 7): the map rewrites a
        # GOAL, so it belongs where goals are built, whereas the tilt map lives
        # in trajectory_node because it rewrites POSES at ingest. Loaded once in
        # __init__ and again on toss/reload_calibration; a single reference swap
        # (values first, the observable flag last) so a concurrent goal build
        # sees either the whole old map or the whole new one.
        self._toss_cal = None
        self._toss_cal_loaded = False
        self._toss_cal_version = ''
        self._toss_cal_path = ''
        # D3 dormancy: WARN once per (map, live tilt-map version) pairing, not
        # per goal. A dormant map at 10 goals/min would otherwise bury every
        # other line in the console — the 4091-ERRORs-in-41 s failure mode.
        self._toss_cal_dormant_reason = ''
        self._toss_cal_dormant_logged = ''
        # ── Layer 2: the SESSION TRIM (phase 2e, jugglebot/toss_trim.py) ──
        # RAM only, one per GOAL, discarded when the goal ends — its only
        # persistent trace is a PROPOSAL under temp/logs/ that an operator
        # promotes through the explicit calibration routine or does not
        # (premise P1). None between goals; never reused across two.
        self._toss_trim = None
        self._toss_trim_t0 = 0.0
        self._toss_trim_belt_warned = False
        # ── Layer 3: the critical-point ILC correction (motion/toss_ilc.py) ──
        # plans/active/critical-point-ilc.md Phase 2. Persistent, per-GOAL keyed,
        # fitted offline between sessions — so it lives here beside the aim map
        # for the same reason the aim map does (it rewrites a GOAL), and NOT
        # beside the session trim, which is RAM-only and dies with its goal.
        # Loaded once in __init__ and never reloaded live: unlike the aim map
        # there is no acquisition tool rewriting it mid-session, and a
        # relaunch-only artifact cannot change underneath a running fit.
        self._toss_ilc = None
        self._toss_ilc_loaded = False
        self._toss_ilc_version = ''
        self._toss_ilc_path = ''
        # Dormancy / refusal WARNs are deduplicated by REASON, not emitted per
        # goal: a dormant artifact at 10 goals/min would otherwise bury the
        # console — the same 4091-ERRORs-in-41 s failure mode the aim map's
        # dormancy log already guards against.
        self._toss_ilc_dormant_reason = ''
        self._toss_ilc_dormant_logged = ''
        self._toss_ilc_refusal_logged = ''
        self._toss_ilc_miss_logged = set()
        # Layer 3's SESSION-LOCAL common-mode component (C1, owner decision 2 of
        # the 2026-08-21 fold-in). RAM only, one per GOAL, discarded when the
        # goal ends — the same lifecycle as the layer-2 trim above and for the
        # same reason, one layer up: it carries the quantity a re-`level` moves,
        # which is only constant WITHIN a goal. It is seeded from the persistent
        # artifact's `anchor` prior, which is where the between-session evidence
        # lives; it never writes anything back.
        self._toss_ilc_session = None
        # The platform's live COMMANDED position (STOW mm) + its arrival stamp.
        # None/0.0 = never heard ⇒ fail closed (REJECTED_POSE_UNKNOWN for a
        # displaced toss, REJECTED_NOT_CENTERED for a reload). trajectory_node
        # publishes it only while seeded+streaming, so silence genuinely means
        # "no commanded pose exists", not "the topic is quiet".
        self._commanded_pos_mm = None
        self._commanded_pos_mono = 0.0
        # The SAME sample with its orientation, (x, y, z, rx, ry, rz) in the
        # INTENT frame (trajectory/commanded_pose). Separate cache, single
        # stamp: the census-B1 skip needs all six components from ONE sample,
        # and None fails closed to "command the move".
        self._commanded_pose = None
        self._commanded_pose_mono = 0.0
        self._mocap_mono = 0.0
        self._balls = []
        self._balls_mono = 0.0
        # Latest hand telemetry (measured pos/vel) — feeds the telemetry-verified
        # prime/retract ladders, which must not re-dispatch a smooth-move that is already
        # (lied-ack) moving the hand. 100 Hz, mono-stamped for staleness. CONSTRAINT:
        # the hand_telemetry subscription must stay OUT of the action's reentrant
        # callback group and the executor must stay multi-threaded — the ladders block
        # in time.sleep(_HAND_ACK_SETTLE_S) on the action thread and need this stamp to
        # keep advancing underneath them (a starved subscription reads as stale
        # telemetry, silently degrading the ladder to blind re-dispatch).
        self._hand_pos_meas = 0.0
        self._hand_vel_meas = 0.0
        self._hand_telemetry_mono = 0.0
        # The active sequencer (announcements + catch feedback route to it while a
        # reload is running); None between runs.
        self._active_seq = None
        # The tracker-assigned id of OUR announced ball, latched once it appears in flight
        # this sequence; only that id's CAUGHT confirms (a stray caught ball never does).
        # An UNTAGGED (empty-destination fallback) latch is provisional — a
        # destination-tagged candidate displaces it (see _build_observations).
        self._announced_ball_id = None
        self._announced_id_untagged = False
        # The id the PREVIOUS cycle of THIS session latched (census D6). Rolled
        # forward at _build_toss_cycle, cleared at goal ACCEPT so it never spans
        # two goals. It is excluded from `track_active`, which the CHECKING gate
        # reads: at a short dwell the previous ball's track is still IN_FLIGHT
        # when cycle N+1 checks (the tracker only leaves IN_FLIGHT when it mints
        # CAUGHT, landing +0.202..+0.442 s), and refusing our own just-caught ball
        # as a "phantom" is a hard REJECTED_TRACK_ACTIVE on a healthy cycle.
        self._prev_announced_ball_id = None
        # The LIVE cycle's next scheduled release/landing on the perf clock, or
        # (None, None) — the cadence clamp of C-POSSESS-1 § 3.4. Set from the
        # session's own schedule at cycle start, cleared with the cycle.
        self._toss_next_release_perf = None
        self._toss_next_landing_perf = None
        # The PREVIOUS and CURRENT cycles' landings, C-POSSESS-1 § 3.4 clause
        # C.1's other boundary end. Unlike the pair above these span CYCLES and
        # are cleared per SESSION (with `_toss_record_prev_uid`), because that is
        # what they are for: the previous cycle closed its arrival window at
        # `arrival_boundary_t(prev, this)` and this one must OPEN at the identical
        # call, so what is handed to `observe` is literally the number the
        # previous cycle was given as its own `landing_t` — not a re-derivation.
        self._toss_prev_landing_perf = None
        self._toss_cycle_landing_perf = None
        # Ball ids already IN_FLIGHT when THIS goal's throw was accepted — excluded
        # from the announced-ball latch. Attempt 5 of the 2026-07-23 re-test latched
        # a phantom untagged track that predated the throw and rode it to a wrong
        # MISSED verdict.
        self._preexisting_flight_ids = set()
        # This goal's catch-speed knob (goal.catch_vel_scale; 0 => the config
        # default JB_OP_CATCH_VEL_SCALE_DEFAULT), published on catch/vel_scale at
        # PREPARE so catch_coordinator has it before any arm.
        self._catch_vel_scale = float(hw.JB_OP_CATCH_VEL_SCALE_DEFAULT)
        # THE possession sources (contract C-POSSESS-1,
        # ros_ws/docs/ball_possession_contract.md). Every "did we catch it?"
        # question in this node routes through _possession_confirmed, which is
        # the ONE merge point (§ 3.2).
        #
        # _possession_source is the tracker CORROBORATOR — ARRIVAL only, and the
        # only supplier of arrival_err_mm (the catch-accuracy number the hardware
        # runbooks score, which the sensor cannot produce). It stays the
        # duck-typed swap seam that test_possession_source_is_pluggable_at_one_seam
        # exercises.
        self._possession_source = TrackerArrivalSource(
            arrival_tol_mm=_CAUGHT_MAX_XY_ERROR_MM)
        # _ball_sensor is the PRIMARY source (2026-08-10): the ball-in-cup hand
        # sensor is the only one that can observe RETENTION, and its ARRIVAL beats
        # the tracker's whenever it can see. TICK-DRIVEN — it reads the cup, so it
        # takes no ball and no reference point, which is exactly the shape the
        # contract's § 3 said a genuinely primary source would need.
        # Staleness reuses _HAND_STATE_STALE_S rather than minting a fourth
        # window: the question ("is the Jetson still hearing the hand?") is
        # literally the one that constant already answers for hand_fresh, and two
        # constants for one question is how they drift apart.
        self._ball_sensor = HandBallSensorSource(
            arrival_lead_s=float(hw.JB_BD_ARRIVAL_LEAD_S),
            arrival_window_s=float(hw.JB_BD_ARRIVAL_WINDOW_S),
            retention_window_s=float(hw.JB_BD_RETENTION_WINDOW_S),
            stale_s=_HAND_STATE_STALE_S)
        # Once-per-(ball, verdict) log guard. Keyed by the VERDICT as well as the
        # id so the accepted and refused lines each get exactly one emission: the
        # accepted line is what the operator scores the gate against by eye at the
        # bench (runbook row POSS-1), and guarding on the id alone would suppress
        # it whenever the same ball was refused first under a different reference
        # point.
        self._possession_logged = set()
        # Cross-action goal claim: set under _lock in the goal-ACCEPT callback (both
        # actions share it), cleared in the execute callbacks' finally alongside
        # _active_seq. Closes the accept→install window in which a second goal's
        # goal_callback still saw _active_seq None — tiny with one action server,
        # a real cross-action race with two (double-owned hand on the last-writer-
        # wins queue; latch fights).
        self._goal_claimed = False
        # ── Toss state ──
        # Latest measured position of the CONFIG-NAMED platform rigid body
        # (_TOSS_MOCAP_BODY_PARAM; '' = cross-check disabled, nothing cached).
        # Non-base bodies publish in the platform_start frame (z ≈ 170 at
        # ACTIVE), directly comparable to the STOW-relative nominated pose —
        # see _TOSS_MOCAP_BODY_PARAM for the frame contract.
        self._platform_pos_mm = None
        # Sticky ball-possession latch (the toss's REJECTED_NO_BALL evidence; no
        # ball-in-cup sensor exists): SET on any plausible destination-tagged CAUGHT
        # on `balls` (a Reload's catch, or a prior Toss's), CLEARED only on OUR
        # release evidence during a toss — it deliberately SURVIVES SAFE_ABORT
        # retracts (the cup carries the ball down at ~3.16 m/s² ≪ g), so an aborted
        # toss is retryable without a Reload. Known false-positive: a post-CAUGHT
        # bounce-out leaves it set; the resulting empty-cup toss is safe (stroke
        # fires, tracker never confirms, outcome resolves honestly) — accepted
        # until the ball-held-sensor era.
        self._ball_possession = False
        # Per-toss-goal state (None/False between goals):
        self._toss_release_state = None       # motion ReleaseState (announcement physics)
        # The COMMANDED release state (C-TOSS-CAL-1). Identical OBJECT to
        # _toss_release_state whenever the commanded aim is zero — i.e. with no
        # aim map loaded it is not merely equal, it is the same object, so the
        # disabled path costs not one extra floating-point operation. When an aim
        # IS commanded the two diverge on exactly the fields the aim moves: the
        # commanded tilt, the pre-tilt pose, the release point, the launch vector
        # and event_vel. _toss_release_state stays the UNCORRECTED vertical toss
        # to B, because D4 announces the uncorrected landing — that is what keeps
        # the correlation→catch path, the receive-tilt computation and the
        # possession plausibility bound bitwise unchanged.
        self._toss_release_cmd = None
        self._toss_aim = None                 # per-goal applied-calibration block (record-only)
        self._toss_landing_global_mm = None   # nominated cup point — CAUGHT plausibility ref
        self._toss_platform_target_mm = None  # nominated PLATFORM pose, global (mocap check)
        self._toss_waiver = False             # trace-only ball-evidence waiver, read per goal
        self._toss_throw_dispatched = False   # gates the release-stroke telemetry watch,
                                              # the tracker-CONFIRMED latch AND the
                                              # possession-clear (release evidence can
                                              # only exist after OUR dispatch)
        self._toss_stroke_seen = False        # sticky release evidence: telemetry channel
        self._toss_track_confirmed = False    # sticky release evidence: tracker channel
        # PREPARE-bundle deferral (one FSM tick): the ACTION_PREPARE_CATCH tick
        # publishes catch/prime_hold ALONE; the bundle (gains → arm → vel_scale
        # → prime_dispatched → armed → snapshot) runs on the NEXT tick so the
        # hold lands in an earlier catch_coordinator wait-set cycle than the
        # armed edge (same-cycle callbacks have no intra-cycle ordering
        # guarantee — the armed edge could beat the hold and auto-prime the
        # ball-laden hand).
        self._toss_prepare_pending = False
        # catch/pretilt_hold was raised this goal — the terminal teardowns
        # release it iff raised, so a LEVEL goal's publish sequence stays
        # byte-identical (it never touches the topic). Keyed on "the commanded
        # release carries a non-zero tilt", NOT on the tier (D3): the moment an
        # 8a toss commands an aim tilt, the stock announcement pre-tilt would
        # otherwise level the platform back BEFORE release and every log line
        # would still report the aim as applied.
        self._toss_pretilt_hold_raised = False
        # THE per-cycle positioning decision (census B1), taken once in
        # _build_toss_cycle and consumed by _position_platform_for_toss.
        # Initialised and reset FAIL-CLOSED (True = command the move): with no
        # cycle built there is no evidence the platform is anywhere in
        # particular, and a stale False would skip a move the next cycle needs.
        self._toss_positioning_move = True
        # Tier 8b only: the announced landing state stashed at ANNOUNCE for the
        # deferred A->B reach to reuse — (landing_pos_global_mm, landing_vel_mms,
        # landing_time_ros_seconds). None between goals / for 8a.
        self._toss_announced_reach = None
        # ── Per-toss RECORD (instrument only, zero control authority) ──
        # toss-selftuning plan § 3.3/§ 3.4, D10. Everything here is written by
        # the record path and read by NOTHING in the FSM: if every line below
        # were deleted the machine would behave identically. That is the
        # property to preserve when this block grows in phases 2b/2d/2e.
        self._toss_record_ctx = None          # per-cycle declaration context
        self._toss_record_announce = None     # (throw_time_ros, landing_time_ros)
        self._toss_record_belt_warned = False  # one WARN per goal, then silence
        self._toss_record_prev_uid = None     # the PREVIOUS cycle's toss_uid, so
                                              #   an ABORTED_NO_RELEASE retry can
                                              #   name what it retried (guard G11)
        # The record WORKER (census B6): the belt write and the trim update run
        # here instead of on the cycle thread, so a slow SD card is a lost
        # measurement rather than a cadence fault. ONE thread, so the append log
        # keeps cycle order and the MUTATING trim estimator sees one update at a
        # time. Started lazily on the first record — a node that never runs a
        # toss never spawns it. UNBOUNDED queue deliberately: bounding it would
        # trade a lost record for a bounded memory footprint, and the queue's
        # steady state is one item (records arrive at most once per ~0.5 s and a
        # belt append is sub-millisecond).
        self._toss_records_q = queue.Queue()
        self._toss_records_thread = None
        # ── Layer 1.5 dwell inclinometer covariate (§ 3.10) ──
        # Filled ONLY from the session's quiescent-dwell loop and consumed (and
        # cleared) by the next cycle's _open_toss_record. Zero control authority:
        # nothing in any FSM reads these.
        self._dwell_tilt_reads = []           # [(t_perf, rx_rad, ry_rad), ...]
        self._dwell_tilt_next_at = 0.0
        self._dwell_tilt_degraded = False
        # ── The auto-reload interlude's two consumer-side fences ──
        # (a) BallButler heartbeats ball_in_hand = TRUE from boot, BEFORE its first
        #     GPIO read (plans/archived/hand-ball-sensor.md § the fail-open boot
        #     default), and this node gates that bit only on heartbeat freshness.
        #     A freshly-rebooted BB therefore makes the reload FSM SKIP
        #     ACTION_CALL_RELOAD: it primes the hand, raises the latch, throws at
        #     an empty BB and dies ABORTED_NO_ANNOUNCEMENT having armed everything
        #     for nothing. Inside an autonomous session nobody is watching the
        #     heartbeat, so the session latches the first FALSE it ever sees and
        #     the interlude refuses until then. The underlying defect stays owned
        #     by its own investigation — this is a CONSUMER-side fence.
        self._bb_ball_in_hand_observed_false = False
        # (b) the BB firmware's terminal throw outcome, which bb/throw_at_target
        #     cannot return (it is fire-and-forget). (text, t_perf) or None.
        self._bb_throw_outcome = None
        # Tier 8b: the pure catch-pose policy for the deferred A→B reach —
        # the SAME class + construction catch_coordinator_node uses (pinned
        # equal by a drift-guard test), so the reach target's receive-tilt/
        # swing math is single-sourced through predicted_catch_command and can
        # never drift from what the announcement path would have commanded.
        self._toss_catch_policy = CatchCoordinator(
            robot_name=robot_name,
            initial_height_mm=hw.GEOM_INITIAL_HEIGHT_MM,
            landing_z_offset_mm=(hw.JB_OP_DEFAULT_ACTIVE_Z_MM
                                 + hw.HAND_CATCH_OFFSET_MM),
            hand_catch_offset_mm=hw.HAND_CATCH_OFFSET_MM,
            catch_angle_limit_deg=30.0,
        )
        # Trace-only waiver parameter (D4d): declared here, read at each toss
        # goal-accept — see _TOSS_WAIVER_PARAM.
        self.declare_parameter(_TOSS_WAIVER_PARAM, False)
        # Layer-2 session trim (phase 2e): declared here, read ONCE per goal
        # build — see _TOSS_TRIM_PARAM. DEFAULT FALSE, deliberately: the trim
        # costs ~1 mm of aim when there is nothing to correct (measured, see
        # toss_trim.SE_GATE), it has never been validated on hardware, and its
        # measurement channel is not live yet (toss_trim's module docstring).
        # It still LEARNS and still writes its proposal while disabled, which is
        # the whole point — zero authority, full instrumentation.
        self.declare_parameter(_TOSS_TRIM_PARAM, False)
        # Mocap arrival cross-check body (D7-revised): declared here, re-read at
        # each toss goal-accept — see _TOSS_MOCAP_BODY_PARAM ('' = disabled).
        self.declare_parameter(_TOSS_MOCAP_BODY_PARAM, '')
        self._toss_mocap_body = str(
            self.get_parameter(_TOSS_MOCAP_BODY_PARAM).value or '')

        # ── Subscriptions ──
        self.create_subscription(
            BallButlerHeartbeat, 'bb/heartbeat', self._on_heartbeat, 10)
        self.create_subscription(
            String, 'control_mode_topic', self._on_control_mode, 10)
        self.create_subscription(
            TrajectoryStatus, 'trajectory/status', self._on_traj_status, 10)
        # The platform's LIVE commanded position (STOW mm, 5 Hz) — trajectory_node
        # publishes it only while seeded+streaming. It is the SOURCE OF TRUTH for
        # the displaced toss's throw site A and for the reload's centred gate;
        # silence is meaningful (⇒ REJECTED_POSE_UNKNOWN / REJECTED_NOT_CENTERED),
        # never a licence to guess.
        self.create_subscription(
            Point, 'trajectory/commanded_position',
            self._on_commanded_position, 10)
        # …and the same sample WITH its orientation, in the intent frame. Read by
        # exactly one thing — the census-B1 positioning skip — which needs the
        # full 6-DOF pose from ONE sample to prove "commanding this would move
        # nothing". Its absence fails closed to "command the move" (cadence, not
        # safety), so this subscription is additive to every other read path.
        self.create_subscription(
            Pose, 'trajectory/commanded_pose', self._on_commanded_pose, 10)
        self.create_subscription(
            RigidBodyPoses, 'rigid_body_poses', self._on_mocap, 10)
        self.create_subscription(
            BallStateArray, 'balls', self._on_balls, 10)
        self.create_subscription(
            HandTelemetryMessage, 'hand_telemetry', self._on_hand_telemetry, 10)
        self.create_subscription(
            ThrowAnnouncement, 'throw_announcements', self._on_announcement, 10)
        self.create_subscription(
            TargetFeedback, 'trajectory/target_feedback', self._on_target_feedback, 10)
        # The BB firmware's TERMINAL throw outcome, relayed by ball_butler_node.
        # bb/throw_at_target returns as soon as the goal is dispatched, so this
        # topic is the ONLY channel on which THROW_ABORTED_NOT_SETTLED — BB not
        # positioned in time, ball never left — can reach a consumer.
        self.create_subscription(
            String, 'bb/throw_outcome', self._on_bb_throw_outcome, 10)

        # ── Service clients ──
        self._reload_cli = self.create_client(Trigger, 'bb/reload')
        self._throw_cli = self.create_client(BallButlerThrow, 'bb/throw_at_target')
        # Platform + hand ownership for the reload's duration:
        #   trajectory/arm_catch — raise/lower the catch-armed latch (the reactive-catch
        #     TRIGGER: while raised, catch/dynamic_target actuates the platform);
        #   smooth_move_hand     — prime to top (PRIME_HAND, at command time) / retract
        #     to bottom (SAFE_ABORT);
        #   trajectory/go_home   — re-center to level neutral on terminal.
        self._arm_catch_cli = self.create_client(SetBool, 'trajectory/arm_catch')
        self._smooth_move_hand_cli = self.create_client(SetFloat, 'smooth_move_hand')
        self._go_home_cli = self.create_client(Trigger, 'trajectory/go_home')
        # Toss-only clients:
        #   trajectory/go_to_pose — POSITIONING's profiled move to the nominated
        #     catch pose (validated through the single feasibility gate; the
        #     response's planned_duration_s anchors the timed arrival);
        #   set_hand_traj_cmd     — THE throw dispatch (traj_type=0, once, ever);
        #   set_hand_gains        — soft catch gains at PREPARE (best-effort; see
        #     _TOSS_SOFT_CATCH_GAINS for why the toss owns them).
        self._go_to_pose_cli = self.create_client(GoToPose, 'trajectory/go_to_pose')
        self._hand_traj_cli = self.create_client(SetHandTrajCmd, 'set_hand_traj_cmd')
        self._hand_gains_cli = self.create_client(SetHandGains, 'set_hand_gains')
        # Layer-1.5 COVARIATE ONLY (§ 3.10): the same get_platform_tilt the
        # orchestrator's levelling handler drives. Called ONLY from the session's
        # quiescent dwell — never from a cycle, never between PREPARE and THROW —
        # because the read blocks the Platform-Teensy loop that streams hand
        # moves. Nothing branches on the result.
        self._tilt_cli = self.create_client(
            GetTiltReadingService, 'get_platform_tilt')

        # ── Publisher: catch-armed state ──
        # catch_coordinator_node gates its hand prime/arm on this so it only actuates the
        # hand DURING a reload (latch raised), never on a stray tracked ball. Published
        # True on PREPARE, False on RECENTER/SAFE_ABORT — the same edges that drive the
        # trajectory/arm_catch service, so the two stay in lockstep.
        self._catch_armed_pub = self.create_publisher(Bool, 'catch/armed', 10)
        # Catch-speed knob relay: the goal's catch_vel_scale, published at PREPARE
        # (before catch/armed goes True) so catch_coordinator holds it before any arm.
        self._vel_scale_pub = self.create_publisher(Float64, 'catch/vel_scale', 10)
        # Prime-dispatch announcement: published on every ACTION_PRIME_HAND
        # smooth-move dispatch so catch_coordinator (which owns its OWN edge prime
        # + retry tick and cannot otherwise see this node's dispatches) can hold
        # its anti-stutter in-flight window instead of restarting a live ascent
        # (the Teensy trajectory queue is last-writer-wins; a cross-node re-prime
        # mid-ascent was the third sitting's hand stutter).
        self._prime_dispatched_pub = self.create_publisher(
            Bool, 'catch/prime_dispatched', 10)
        # Prime-suppression gate for the toss (catch/prime_hold): True at PREPARE
        # entry — BEFORE catch/armed rises — and False at terminal (LAST, after the
        # teardown; see _toss_recenter/_toss_safe_abort for why). While True,
        # catch_coordinator dispatches NO auto-prime (edge or retry tick): the ball
        # rides the hand at the stroke bottom awaiting the throw, and a kind-3
        # ascent would launch it AND clear an armed throw stroke on the Teensy's
        # last-writer-wins queue. Stale True fails SAFE (no auto-prime; the reload
        # action primes proactively itself).
        self._prime_hold_pub = self.create_publisher(Bool, 'catch/prime_hold', 10)
        # Pre-tilt suppression gate for TIER 8B (catch/pretilt_hold): True on the
        # ACTION_PREPARE_CATCH tick (alongside prime_hold — ≥2 FSM ticks before
        # our announcement can reach catch_coordinator), False at terminal
        # (released LAST, with prime_hold). While True, catch_coordinator still
        # LATCHES our announcement (open-loop freeze + hand-arm window keep
        # working) but publishes NO platform pre-tilt: the stock pre-tilt's
        # arrival clamps to ~now + 1 s while the toss announces ≥1 s before
        # release, so the A→B translate would COMPLETE before the ball leaves —
        # aim destroyed, a moving platform under a seated ball mid-windup. This
        # node instead publishes the ONE deferred reach at t_release (see
        # _publish_toss_reach). Stale True fails DEGRADED-BUT-SAFE: reload
        # announcements lose their platform pre-tilt (the platform simply holds;
        # the hand-arm stays tracker-driven) — zero hazard, WARN-logged in CCN.
        self._pretilt_hold_pub = self.create_publisher(
            Bool, 'catch/pretilt_hold', 10)
        # Declared reach-envelope centre (contract C-REACH-1,
        # ros_ws/docs/catch_reach_envelope.md). Published by the TOSS on the
        # ACTION_PREPARE_CATCH tick — the SAME tick as prime_hold, i.e. one full
        # FSM tick (_TICK_S, 20 ms since c938c1d — still two orders of magnitude
        # above localhost topic latency, but NOT the 50 ms this comment claimed
        # until 2026-08-22) before the bundle that calls
        # trajectory/arm_catch, because the two travel on different transports
        # with no cross-topic ordering guarantee (the identical reasoning that
        # split prime_hold off the bundle). trajectory_node consumes it at the
        # arm RAISE edge.
        #
        # Losing it degrades to EXACTLY the pre-contract behaviour — envelope at
        # A, the deferred A→B reach rejected WORKSPACE mid-flight, ball on the
        # floor and a loud reject on both nodes. That is a miss, not a hazard,
        # and it is why the declaration needs no ack.
        #
        # The RELOAD path never publishes here: its catch IS the held pose, so
        # the commanded-pose fallback is already the right centre, and its
        # choreography stays byte-identical.
        self._reach_center_pub = self.create_publisher(
            Point, 'catch/reach_center', 10)
        # Tier 8b's deferred A→B reach target (the same wire catch_coordinator
        # publishes; trajectory_node consumes either publisher identically while
        # the catch-armed latch is raised).
        self._dyn_target_pub = self.create_publisher(
            DynamicTargetCommand, 'catch/dynamic_target', 10)
        # The toss's self-ThrowAnnouncement: thrower_name = target_id = this robot,
        # published in PREPARING (armed confirmed → ≥1-tick gap → announce) so the
        # existing correlation → catch path closes the loop unchanged.
        self._announce_pub = self.create_publisher(
            ThrowAnnouncement, 'throw_announcements', 10)
        # ── The per-toss RECORD declaration (toss-selftuning § 3.4, D10) ──
        # std_msgs/String carrying JSON, deliberately NOT a typed message: a
        # schema tweak on a typed message needs a two-package colcon build, which
        # is exactly the partial-build ImportError class the runbook's build gate
        # exists to prevent. The cost of JSON-in-String is paid down by
        # jugglebot/toss_record.py owning encode/decode/validate plus a FIELDS
        # drift-guard test.
        #
        # The bag is the CANONICAL sink. Publishing rather than writing keeps
        # file I/O out of the node that owns the hand, the latch and the abort
        # ladder — a full disk must not be able to stall a teardown.
        self._toss_record_pub = self.create_publisher(String, 'toss/record', 10)
        # ── Toss aim-calibration observability (C-TOSS-CAL-1) ──
        # LATCHED std_msgs/String JSON rather than typed status fields, for two
        # independent reasons. (1) TrajectoryStatus is published by
        # trajectory_node, which does not own this map and cannot know whether it
        # is applied — the design's P2 preflight grep against /trajectory/status
        # is not implementable, and cross-node field-filling would be a lie.
        # (2) A new typed field forces a jugglebot_interfaces rebuild, and a
        # partial two-package colcon build takes down every ball-op action —
        # exactly the ImportError class the runbook's build gate exists to
        # prevent (D10, the same argument that made /toss/record a String).
        # TRANSIENT_LOCAL so `ros2 topic echo /toss/calibration_status --once`
        # answers immediately: the map changes only on load and on reload, so an
        # unlatched topic would be silent exactly when the operator asks.
        self._toss_cal_status_pub = self.create_publisher(
            String, 'toss/calibration_status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # std_srvs/Trigger, mirroring trajectory/reload_tilt_map: adding it needs
        # no interfaces rebuild, and the response message carries the loaded
        # version so the phase-2c fit tool can READ BACK what the node actually
        # loaded — the hard guarantee that it loaded the file the tool wrote.
        self.create_service(Trigger, 'toss/reload_calibration',
                            self._svc_reload_toss_calibration)
        self._load_toss_cal()
        self._load_toss_ilc()
        # One node launch = one session id; also names the best-effort belt file.
        self._session_id = '{}-{}'.format(
            datetime.now().strftime('%Y%m%d-%H%M%S'), os.getpid())
        self._git_sha, self._git_dirty = _repo_revision()
        # FILTERED perf-ros offset, for the record only. The FSM keeps using its
        # single instantaneous read (_announcement_landing_perf); carrying BOTH
        # numbers is what turns that method's documented open reconciliation
        # question into a measurement instead of an argument (§ 3.3).
        self._clock_offset_history = []
        self._perf_minus_ros_s = clock_offset.refresh_offset(
            self._clock_offset_history, self._ros_clock_s)
        self.create_timer(clock_offset.REFRESH_PERIOD_S,
                          self._refresh_clock_offset)

        # ── Action servers (reload + toss) ──
        # ReentrantCallbackGroup so the multi-second execute_callback (which blocks on
        # the BB service calls + the sequence loop) never starves the subscription
        # callbacks that feed it observations / announcements.
        self._cbgroup = ReentrantCallbackGroup()
        self._reload_action = ActionServer(
            self, Reload, 'jugglebot/reload',
            execute_callback=self._execute_reload,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cbgroup)
        # The toss shares the goal/cancel callbacks: the busy gate is ONE
        # _lock-guarded claim spanning ALL THREE actions (a Toss goal while a
        # Reload runs — or a session while either runs, or either while a session
        # runs — is REJECTED_BUSY at accept), and all three defer cancel semantics
        # to their execute loops.
        self._toss_action = ActionServer(
            self, Toss, 'jugglebot/toss',
            execute_callback=self._execute_toss,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cbgroup)
        # The continuous session (Phase F) is the SAME claim and the SAME cancel
        # policy: it holds ONE busy claim for the whole session, so a Reload or a
        # Toss dispatched mid-session is REJECTED_BUSY — the one-ball-op rule
        # extends unchanged rather than gaining a session-shaped exception.
        self._toss_continuous_action = ActionServer(
            self, TossContinuous, 'jugglebot/toss_continuous',
            execute_callback=self._execute_toss_continuous,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cbgroup)

        self.get_logger().info(
            f"Ball-ops coordinator ready (jugglebot/reload + jugglebot/toss + "
            f"jugglebot/toss_continuous); "
            f"catch point {self._catch_point_mm} mm (world).")

    # ── Subscription callbacks ─────────────────────────────────────────────────

    def _on_heartbeat(self, msg):
        with self._lock:
            self._hb = msg
            self._hb_mono = time.perf_counter()
            # The consumer-side fence on BallButler's fail-open boot default (see
            # _bb_ball_in_hand_observed_false). Latched on the FIRST false ever
            # seen and never cleared: it answers "has this BB's GPIO actually
            # spoken?", which is a property of the BB process, not of a moment.
            if not bool(getattr(msg, 'ball_in_hand', False)):
                self._bb_ball_in_hand_observed_false = True

    def _on_bb_throw_outcome(self, msg):
        """``bb/throw_outcome``: the BB firmware's TERMINAL CMD_RESULT text,
        relayed by ``ball_butler_node`` (``NAME (axis=..., detail1=...)``).

        Cached with an arrival stamp and consulted by exactly one caller — the
        auto-reload interlude, deciding whether a failed reload was the known
        ``THROW_ABORTED_NOT_SETTLED`` (BB not positioned in time, so no ball ever
        left) and therefore retryable within budget. Nothing else in this node
        reads it, and no FSM branches on it."""
        try:
            text = str(msg.data)
        except Exception:                                      # noqa: BLE001
            return
        with self._lock:
            self._bb_throw_outcome = (text, time.perf_counter())

    def _bb_throw_outcome_since(self, t_perf: float):
        """The BB throw outcome CODE reported after ``t_perf``, or None.

        Returns the leading token — the ``BallButlerCommandOutcome`` member name —
        so the caller compares against a named code rather than pattern-matching a
        human-readable sentence. Anything older than ``t_perf``, or older than
        ``_BB_THROW_OUTCOME_STALE_S``, belongs to a previous attempt and is not
        reported: a stale NOT_SETTLED must never license a retry of a reload that
        failed for some other reason."""
        with self._lock:
            cached = self._bb_throw_outcome
        if not cached:
            return None
        text, stamp = cached
        if stamp < float(t_perf):
            return None
        if (time.perf_counter() - stamp) >= _BB_THROW_OUTCOME_STALE_S:
            return None
        token = str(text).split()[0] if str(text).strip() else ''
        return token or None

    def _on_control_mode(self, msg):
        with self._lock:
            self._control_mode = str(msg.data)

    def _on_traj_status(self, msg):
        with self._lock:
            self._streaming = bool(msg.streaming)
            # Read as a plain attribute, never getattr-with-a-default. A
            # field-less TrajectoryStatus can only reach here from a publisher
            # built against a different jugglebot_interfaces than this node —
            # a genuine build split, which must fail loudly rather than be
            # papered over into a permanent, unexplained NOT_LEVELLED. (The
            # same-install case does NOT reach here at all: trajectory_node
            # raises AttributeError inside its own 5 Hz status timer, rclpy
            # re-raises it out of spin, and since main() catches only
            # KeyboardInterrupt the PROCESS EXITS — see the operator runbook's
            # LVLGATE build box.)
            self._gravity_correction_loaded = bool(msg.gravity_correction_loaded)
            # Record-only provenance (toss-selftuning D3/G5): an aim residual
            # fitted under tilt map A double-counts tilt map B's delta, so the
            # map identity has to travel WITH every toss. getattr-with-default
            # here and NOT above deliberately: the levelling gate must fail loud
            # on a build split, while a missing provenance string must degrade to
            # "unknown" rather than take the coordinator down.
            self._tilt_map_loaded = bool(getattr(msg, 'tilt_map_loaded', False))
            # LIVE session limits — getattr-with-default like the tilt-map
            # provenance and NOT like the levelling gate, deliberately: the
            # toss reach bound has a designed fallback (the YAML-default module
            # copies) for exactly this absence, so an old publisher must
            # degrade to that fallback rather than take the coordinator down.
            self._leg_limits_live = (
                float(getattr(msg, 'leg_vel_limit_mmps', 0.0)),
                float(getattr(msg, 'leg_acc_limit_mmps2', 0.0)),
                float(getattr(msg, 'leg_jerk_limit_mmps3', 0.0)))
            new_tilt_version = str(getattr(msg, 'tilt_map_version', '') or '')
            tilt_version_changed = new_tilt_version != self._tilt_map_version
            self._tilt_map_version = new_tilt_version
            if tilt_version_changed:
                # The aim map's APPLIED-ness is a function of this string (D3),
                # so a tilt-map reload can flip a dormant aim map live and must
                # re-arm the once-per-pairing dormancy WARN.
                self._toss_cal_dormant_logged = ''
            self._traj_status_mono = time.perf_counter()
        if tilt_version_changed:
            # Outside the lock: publishing under it would hold the FSM's own
            # mutex through a middleware call at the status rate.
            self._publish_toss_cal_status()

    def _on_commanded_position(self, msg):
        """``trajectory/commanded_position``: the platform's live COMMANDED
        (x, y, z) in STOW mm — where the emitter is actually driving it.

        Cached with an arrival stamp; consumers gate on freshness and treat
        absence as UNKNOWN, never as centre. Non-finite components are dropped
        (they would poison the throw-site aim and the centred test alike, and a
        NaN comparison silently reads as "not centred"/"not displaced"
        depending on the operator, which is exactly the class of quiet wrong
        answer this channel exists to eliminate)."""
        p = (float(msg.x), float(msg.y), float(msg.z))
        if not all(math.isfinite(v) for v in p):
            self.get_logger().error(
                f'trajectory/commanded_position {p} is non-finite — DISCARDED')
            return
        with self._lock:
            self._commanded_pos_mm = p
            self._commanded_pos_mono = time.perf_counter()

    def _live_commanded_position(self, now: float):
        """The live commanded platform position, or None when unknown/stale.

        ``_TRAJ_STATUS_STALE_S`` is the window: this topic rides the SAME 5 Hz
        trajectory_node timer as ``trajectory/status``, so it inherits that
        window rather than inventing a second one. Caller-side ``now`` is
        ``time.perf_counter()`` (the node's one monotonic domain)."""
        with self._lock:
            pos = self._commanded_pos_mm
            mono = self._commanded_pos_mono
        if pos is None or mono <= 0.0 or (now - mono) >= _TRAJ_STATUS_STALE_S:
            return None
        return pos

    def _on_commanded_pose(self, msg):
        """``trajectory/commanded_pose``: the SAME sampled plan state as
        ``commanded_position``, with its ORIENTATION, as a rotation vector in the
        **INTENT** frame (``trajectory_node._intent_orientation`` inverts the
        C-LEVEL-1 correction before publishing, so this is directly comparable
        against a pose this node is about to REQUEST — see that method).

        Cached as one ``(x, y, z, rx, ry, rz)`` tuple with a single arrival
        stamp, deliberately: the only consumer (the census-B1 positioning skip)
        needs position and orientation from ONE sample, and a torn read across
        two caches would let a mid-move platform match a target on a stale half.

        Non-finite components are dropped whole, for the reason
        :meth:`_on_commanded_position` drops them: a NaN comparison silently
        reads as "not there", which is safe here but hides a real fault."""
        p = msg.position
        q = msg.orientation
        vals = (float(p.x), float(p.y), float(p.z),
                float(q.w), float(q.x), float(q.y), float(q.z))
        if not all(math.isfinite(v) for v in vals):
            self.get_logger().error(
                'trajectory/commanded_pose {} is non-finite — DISCARDED'
                .format(vals))
            return
        rotvec = rot_matrix_to_rotvec(
            quat_to_rot_matrix(vals[3], vals[4], vals[5], vals[6]))
        with self._lock:
            self._commanded_pose = (vals[0], vals[1], vals[2],
                                    float(rotvec[0]), float(rotvec[1]),
                                    float(rotvec[2]))
            self._commanded_pose_mono = time.perf_counter()

    def _live_commanded_pose(self, now: float):
        """The live commanded platform POSE ``(x, y, z, rx, ry, rz)``, or None.

        Same freshness window and same fail-closed doctrine as
        :meth:`_live_commanded_position` — absence is UNKNOWN, never "at the
        origin, level". A trajectory_node that predates the topic therefore
        leaves the B1 skip permanently off, which costs cadence and nothing
        else."""
        with self._lock:
            pose = self._commanded_pose
            mono = self._commanded_pose_mono
        if pose is None or mono <= 0.0 or (now - mono) >= _TRAJ_STATUS_STALE_S:
            return None
        return pose

    def _on_mocap(self, msg):
        with self._lock:
            self._mocap_mono = time.perf_counter()
            # Cache the CONFIG-NAMED platform body's measured position for the
            # toss POSITIONING arrival cross-check ('' = disabled, cache
            # nothing). Store AS PUBLISHED: non-base bodies arrive in the
            # platform_start frame (mocap_interface shifts z by the base→
            # platform transform), which is the frame the nominated STOW-
            # relative pose already lives in — converting either side to
            # global would double-add GEOM_INITIAL_HEIGHT_MM.
            body_name = self._toss_mocap_body
            if not body_name:
                return
            for body in msg.bodies:
                if body.name == body_name:
                    p = body.pose.pose.position
                    self._platform_pos_mm = (
                        float(p.x), float(p.y), float(p.z))
                    break

    def _on_balls(self, msg):
        with self._lock:
            self._balls = list(msg.balls)
            self._balls_mono = time.perf_counter()
            ref = self._toss_landing_global_mm
        # Ball-possession latch (toss REJECTED_NO_BALL evidence): any plausible
        # destination-tagged CAUGHT sets it — a Reload's catch or a prior Toss's,
        # judged against the nominated toss landing point while a toss runs and the
        # ACTIVE catch point otherwise. Sticky; cleared only on OUR release
        # evidence (_build_toss_observations).
        for b in msg.balls:
            if (b.destination == self._robot_name
                    and int(b.status) == _BALL_STATUS_CAUGHT
                    and self._possession_confirmed(b, ref_point_mm=ref)):
                with self._lock:
                    self._ball_possession = True
                break

    def _on_hand_telemetry(self, msg):
        now = time.perf_counter()
        with self._lock:
            self._hand_pos_meas = float(msg.pos_meas)
            self._hand_vel_meas = float(msg.vel_meas)
            self._hand_telemetry_mono = now
            # Feed the PRIMARY possession source. The mono clock, not
            # ball_held_stamp: staleness here asks whether the JETSON is still
            # hearing the sensor, and the bridge stamp is wall-epoch only once
            # the bridge's clock anchor lands (plans/archived/hand-ball-sensor.md
            # § Architecture, "Clock discipline"). ball_held is meaningless
            # unless ball_held_valid — the source enforces that, so both are
            # passed through untouched and nothing here decides anything.
            # getattr, not a bare read: an older node running against a bag or a
            # pre-Phase-5 message must degrade to UNKNOWN, never to "no ball".
            #
            # BOTH bits are fed since 2026-08-21 (C-POSSESS-1 § 3.5, census D3).
            # The debounced one is the VERDICT bit (arrival/retention edges); the
            # raw one answers the LIVE `evidence` query, because the debounce is
            # asymmetric and slow in the fall direction (~241 ms measured) and a
            # ball-evidence gate reading it would pass an EMPTY cup once the dwell
            # approached that number — a fail-OPEN possession gate.
            #
            # The getattr default is a belt for a NON-ROS caller, and it is
            # honest about being no more than that (audit fix, 2026-08-22):
            # ball_held_raw is a declared `bool` on HandTelemetryMessage, so this
            # returns False and never None for any real message, and a publisher
            # that never sets the bit is indistinguishable here from an empty
            # cup. What actually protects the actuating path is
            # `evidence_settled` (the interlude's read), which sees the raw and
            # debounced bits disagree and answers UNKNOWN. See
            # ball_possession.note_sample.
            raw = getattr(msg, 'ball_held_raw', None)
            self._ball_sensor.note_sample(
                now,
                held=bool(getattr(msg, 'ball_held', False)),
                valid=bool(getattr(msg, 'ball_held_valid', False)),
                raw=None if raw is None else bool(raw))

    def _on_announcement(self, msg):
        # Only OUR ball's announcement (thrown by BB, aimed at us) advances the FSM.
        if msg.target_id and msg.target_id != self._robot_name:
            return
        now = time.perf_counter()
        landing_perf = self._announcement_landing_perf(msg, now)
        with self._lock:
            seq = self._active_seq
        # A TOSS is its own announcer: its FSM is notified synchronously by the
        # ANNOUNCE executor, and this subscription then hears our OWN publish loop
        # back (target_id == this robot passes the filter above) — that echo must
        # not be double-fed into the toss FSM (whose note_announcement carries no
        # landing argument; it self-schedules the landing).
        if isinstance(seq, TossSequencer):
            return
        if seq is not None:
            seq.note_announcement(now, landing_perf)

    def _announcement_landing_perf(self, msg, now_perf: float) -> float:
        """Convert the announcement's ROS ``landing_time`` into the FSM's perf clock — the
        same crossing ``catch_coordinator_node`` does for its arrival_time (perf − ros
        offset, read at point of use). The landing stamp is release + ToF, so the settle
        deadline it feeds includes the ball's time-of-flight. Falls back to
        ``now + predicted_tof_sec`` when the stamp is absent/unusable (and to ``now`` if
        neither is present).

        NOTE: this is a *single instantaneous* ``perf − ros`` read, deliberately NOT
        the median-filtered running offset that ``jugglebot.clock_offset`` provides to
        trajectory_node and catch_coordinator_node.  Whether this path should adopt
        the filtered offset (fresher vs. hiccup-immune) is an open reconciliation
        decision, not an oversight — do not "fix" it by unifying without that call."""
        lt = getattr(msg, 'landing_time', None)
        ros_s = 0.0
        if lt is not None:
            try:
                ros_s = float(lt.sec) + float(lt.nanosec) * 1e-9
            except (AttributeError, TypeError, ValueError):
                ros_s = 0.0
        if ros_s > 0.0:
            now_ros = self.get_clock().now().nanoseconds * 1e-9
            return ros_s + (now_perf - now_ros)   # offset = perf − ros
        try:
            tof = float(getattr(msg, 'predicted_tof_sec', 0.0) or 0.0)
        except (TypeError, ValueError):
            tof = 0.0
        return now_perf + tof

    def _on_target_feedback(self, msg):
        if msg.source != 'catch':
            return
        code = str(msg.code)
        # Drop the transient (non-infeasibility) codes so a late-target FROZEN or a
        # plant-state-race STALE_STATE — both expected on a real flight — never latches
        # MISSED_INFEASIBLE. Only genuine rejections (and every acceptance) reach the FSM.
        if not bool(msg.accepted) and code in _TRANSIENT_FEEDBACK_CODES:
            return
        with self._lock:
            seq = self._active_seq
        if seq is not None:
            seq.note_catch_feasibility(bool(msg.accepted), code)

    # ── Observation assembly (testable) ────────────────────────────────────────

    def _build_observations(self, now: float) -> ReloadObservations:
        with self._lock:
            hb = self._hb
            hb_fresh = hb is not None and (now - self._hb_mono) < _HEARTBEAT_STALE_S
            mocap_fresh = (now - self._mocap_mono) < _MOCAP_STALE_S
            streaming = self._streaming
            control_mode = self._control_mode
            balls = self._balls
            balls_fresh = (now - self._balls_mono) < _STATUS_STALE_S
        ball_caught = False
        catch_error_mm = float('nan')
        if balls_fresh:
            announced_id = self._update_announced_ball_latch(balls)
            if announced_id is not None:
                for b in balls:
                    if int(b.id) == announced_id and int(b.status) == _BALL_STATUS_CAUGHT:
                        if self._possession_confirmed(b):
                            ball_caught = True
                            catch_error_mm = self._catch_error_from_ball(b)
                        break
        return ReloadObservations(
            now=now,
            control_mode=control_mode,
            bb_connected=bool(hb.connected) if hb_fresh else False,
            bb_state=int(hb.state) if hb_fresh else 0,
            ball_in_hand=bool(hb.ball_in_hand) if hb_fresh else False,
            mocap_fresh=mocap_fresh,
            streaming=streaming,
            platform_centered=self._platform_centered(now),
            ball_caught=ball_caught,
            catch_error_mm=catch_error_mm)

    def _platform_centered(self, now: float) -> bool:
        """True iff the platform's LIVE commanded xy is within the catch reach
        envelope of the reload catch point — the RELOAD precondition that a
        caught toss's stay-at-pose terminal made necessary (see the
        ``REJECTED_NOT_CENTERED`` gate in ``reload_sequencer._step_checking``).

        Reference point: the reload catch point's xy, which
        :func:`compute_catch_point_mm` places on-axis at the workspace centre —
        read from ``self._catch_point_mm`` rather than hard-coded, so the two
        cannot drift apart. FRAME: ``_catch_point_mm`` is world and the commanded
        position is STOW-relative, but the STOW→world conversion adds only z
        (``toss_release.stow_to_global_mm``), so the **xy comparison is exact in
        either frame** — and it is xy-only on purpose, because the reload's z is
        the ACTIVE plane by construction and a z difference is not what makes a
        catch unreachable here.

        Tolerance: the envelope radius MINUS the reload pre-tilt's centroid
        swing (:data:`_RELOAD_CENTERED_TOL_MM`). The envelope radius alone is
        NOT the right tolerance and shipping it would have left this gate
        admitting the exact state it exists to refuse — see that constant for
        the measured trace. Degradation inside the tightened bound stays
        graceful (the announcement pre-tilt translates the remaining offset
        during the flight, planned and gated as any other catch reach).

        UNKNOWN (no fresh commanded position) ⇒ False: fail closed."""
        pos = self._live_commanded_position(now)
        if pos is None:
            return False
        return math.hypot(pos[0] - float(self._catch_point_mm[0]),
                          pos[1] - float(self._catch_point_mm[1])) \
            <= _RELOAD_CENTERED_TOL_MM

    def _update_announced_ball_latch(self, balls):
        """Correlate the tracker-assigned id of OUR announced ball and return it.

        The tracker puts our ball IN_FLIGHT (destination == us) after correlating the
        announcement; latch that id, then confirm ONLY that id's CAUGHT. This rejects
        a stray caught ball (a different id — e.g. a leftover from a prior throw)
        that would otherwise falsely confirm the catch. Shared by the reload and toss
        observation builders — the correlation rules are identical (the toss's
        "announcement" is its own self-publish); only the pre-existing-flight
        snapshot's anchor differs (throw-accept for reload, PREPARE for toss).
        Two hardening passes (2026-07-23 re-test): (1) ids already IN_FLIGHT
        when the throw was accepted are excluded (a phantom untagged track that
        predated the throw was latched as "our" ball); (2) a destination match
        is preferred over the empty-destination fallback.

        The RULE itself lives in ``jugglebot.toss_record.latch_announced_ball``
        (extracted 2026-08-10, toss-selftuning plan D11) so the offline record
        miner correlates ``/balls`` with the same logic instead of a lookalike
        copy; this method keeps the locking and the state, which is the half that
        cannot be pure."""
        with self._lock:
            announced_id = self._announced_ball_id
            preexisting = set(self._preexisting_flight_ids)
            untagged_latch = self._announced_id_untagged
        announced_id, untagged_latch = latch_announced_ball(
            balls, robot_name=self._robot_name, announced_id=announced_id,
            preexisting_ids=preexisting, untagged_latch=untagged_latch,
            in_flight_status=_BALL_STATUS_IN_FLIGHT)
        if announced_id is not None:
            with self._lock:
                self._announced_ball_id = announced_id
                self._announced_id_untagged = untagged_latch
        return announced_id

    def _expected_landing_perf(self):
        """The ACTIVE sequence's predicted landing on the perf clock, or None.

        The hand ball sensor needs a window to look for its arrival edge in
        (C-POSSESS-1 § 3.2), and both FSMs already own that number — the toss
        because it is its own announcer, the reload from BB's announcement. It is
        read fresh on every query and never latched, so a finished goal's window
        cannot survive to veto the next ball's CAUGHT.

        None (⇒ ``ARRIVAL_UNKNOWN`` ⇒ the merge falls back to the tracker) whenever
        no sequence is running or none has scheduled a throw yet — which is the
        honest answer: with nothing in the air, "did it arrive?" has no referent."""
        with self._lock:
            seq = self._active_seq
        if seq is None:
            return None
        land = getattr(seq, 'landing_perf', None)
        if land is None:
            return None
        try:
            land = float(land)
        except (TypeError, ValueError):
            return None
        return land if math.isfinite(land) else None

    def _expected_next_cycle_perf(self):
        """-> ``(next_release_t, next_landing_t)`` on the perf clock, or
        ``(None, None)``. The cadence clamp of C-POSSESS-1 § 3.4.

        These are the machine's OWN next scheduled instants, and they bound both
        sensor windows: retention closes where the next toss's departure search
        opens, arrival closes where the next cycle's arrival search opens. Sized
        by the SESSION (it owns the dwell) and latched per cycle, cleared with the
        cycle — the same lifecycle discipline `landing_perf` has, and for the same
        reason: a schedule that outlives its cycle would clamp the NEXT ball's
        windows against a stale instant.

        ``(None, None)`` for a single ``Toss`` and for a session's last intended
        cycle — nothing is scheduled after them, so the honest horizon is the
        shipped fixed one and the behaviour is the pre-2026-08-21 behaviour."""
        with self._lock:
            return self._toss_next_release_perf, self._toss_next_landing_perf

    def _reset_toss_arrival_boundary(self) -> None:
        """Drop the cross-cycle landings that clause C.1's boundary is built
        from. Per SESSION, never per cycle: clearing them per cycle would delete
        the very number the next cycle needs, and this makes a session's FIRST
        window open at the shipped `landing - arrival_lead_s`.

        A single ``Toss`` never calls `_set_toss_next_cycle_perf` and so never
        rolls the latch, which leaves a PREVIOUS session's last landing standing
        when one follows a session in the same process. That is deliberate rather
        than tolerated, and `arrival_boundary_t` is what makes it safe: the
        boundary is a `max` against `landing - arrival_lead_s`, so a landing more
        than `ARRIVAL_BAND_MAX_S + arrival_lead_s` (0.760 s) old degenerates to
        exactly the shipped opening — and one NEWER than that is a real neighbour whose
        band really does overlap, which is precisely the case the boundary exists
        to arbitrate. Clearing it there would be less correct, not more."""
        with self._lock:
            self._toss_prev_landing_perf = None
            self._toss_cycle_landing_perf = None

    def _expected_prev_landing_perf(self):
        """-> the PREVIOUS cycle's landing on the perf clock, or ``None``.

        C-POSSESS-1 § 3.4 clause C.1's other boundary end. ``None`` on a single
        ``Toss``, on a reload, and on a session's FIRST cycle — nothing landed
        before them, and the boundary can only ever move an opening later, so its
        absence is exactly the shipped window rather than a widened one.

        It is the number `_set_toss_next_cycle_perf` recorded as the previous
        cycle's landing, NOT a re-derivation: the previous cycle closed its
        arrival window at `arrival_boundary_t(prev, this)` and this one opens at
        the same call on the same pair, which is the whole point of the
        abutment."""
        with self._lock:
            return self._toss_prev_landing_perf

    def _set_toss_next_cycle_perf(self, seq, session) -> None:
        """Latch the next cycle's schedule for the cycle `seq` is about to run.

        Only when the session INTENDS another throw after this one. On the last
        intended cycle the windows stay unclamped, which costs nothing (there is
        no next release to be confused with a bounce-out) and buys back the one
        cycle per session on which a late bounce-out would otherwise read
        UNKNOWN — see § 3.4's sized residual.

        It also ROLLS the arrival boundary's other end (clause C.1). ``landing``
        here is ``seq.t_release + seq.flight_time_s``, which is exactly what
        ``seq.landing_perf`` returns and therefore exactly what `observe` will be
        given as ``landing_t`` for this cycle — so next cycle's ``prev_landing_t``
        is the same number this cycle judged against, and the two windows abut by
        identity rather than by two agreeing derivations. Safe to read
        ``t_release`` here: `_build_toss_cycle` has already called ``seq.start``,
        which is what sets it."""
        landing = float(seq.t_release) + float(seq.flight_time_s)
        if session is None or not session.intends_another_cycle:
            rel = land = None
        else:
            rel = landing + float(session.dwell_time_s)
            land = rel + float(seq.flight_time_s)
        with self._lock:
            self._toss_next_release_perf = rel
            self._toss_next_landing_perf = land
            self._toss_prev_landing_perf = self._toss_cycle_landing_perf
            self._toss_cycle_landing_perf = landing

    def _possession_confirmed(self, ball, ref_point_mm=None) -> bool:
        """THE possession question, for every caller in this node: does this tracker
        ``CAUGHT`` confirm that WE have the ball?

        The single seam onto :attr:`_possession_source` (contract C-POSSESS-1,
        ``ros_ws/docs/ball_possession_contract.md``) — routing all callers through
        here is what lets the ball-in-cup hand sensor become the primary source
        later without touching a single call site.

        Renamed from ``_caught_is_plausible`` 2026-07-28 because the semantics
        changed underneath it: the answer is now a two-part verdict (ARRIVAL +
        RETENTION), not one spatial plausibility test, and a stale name is how a
        future reader concludes the z bound is merely missing rather than
        deliberately forbidden.

        ``ref_point_mm`` overrides the reference point (the toss judges against its
        NOMINATED landing point); None = the reload's fixed ACTIVE catch point.

        THE MERGE POINT (C-POSSESS-1 § 3.2, 2026-08-10). The tracker source is
        consulted for ARRIVAL-as-corroboration and for ``arrival_err_mm``; the hand
        sensor is consulted for the real answer. ``merge_possession`` owns the
        rules — do not re-derive them here."""
        ref = self._catch_point_mm if ref_point_mm is None else ref_point_mm
        tracker = self._possession_source.judge(
            ball_xyz_mm=(float(ball.position.x), float(ball.position.y),
                         float(ball.position.z)),
            ref_point_mm=ref)
        next_release_perf, next_landing_perf = self._expected_next_cycle_perf()
        verdict = merge_possession(
            sensor=self._ball_sensor.observe(
                time.perf_counter(), self._expected_landing_perf(),
                next_release_t=next_release_perf,
                next_landing_t=next_landing_perf,
                prev_landing_t=self._expected_prev_landing_perf()),
            tracker=tracker)
        ok = bool(verdict.confirmed)
        # Keyed on the ARRIVAL STATE, not on the boolean: UNKNOWN and REFUSED both
        # project to False but say opposite things about where the fault is, and
        # keying on the bool would emit whichever fired first and swallow the
        # other for the rest of the goal.
        key = (int(ball.id), verdict.arrival)
        if key not in self._possession_logged:
            self._possession_logged.add(key)
            # BOTH verdicts are logged, and the wording AND the severity come from
            # ball_possession so the operator-facing line cannot drift from the
            # semantics. Both are INFO today: a refusal fires on essentially every
            # reload (a corrupt split track is the expected reading, not an error
            # in a working sequence) and the acceptance is the line the bench
            # scores the gate against by eye (runbook row POSS-1) — so neither
            # belongs at WARN, and both belong in the log exactly once.
            severity, line = describe_possession(verdict, _CAUGHT_MAX_XY_ERROR_MM)
            log = self.get_logger()
            emit = log.info if severity == 'info' else log.warning
            emit(f"Ball {int(ball.id)} at "
                 f"({float(ball.position.x):.0f}, {float(ball.position.y):.0f}, "
                 f"{float(ball.position.z):.0f}) mm: {line}")
        return ok

    def _catch_error_from_ball(self, ball, ref_point_mm=None) -> float:
        """Horizontal miss distance of the caught ball from the world-frame catch point.

        This is the tracker's last KF position estimate at CAUGHT (an in-flight-derived
        estimate), NOT a settled rest position — the MVP evidence is the tracker-id
        correlation + this KF miss, with a hand-telemetry rest cross-check deferred.
        ``ref_point_mm`` overrides the reference (the toss's nominated landing point);
        None = the reload's fixed ACTIVE catch point.

        Deliberately HORIZONTAL, and that is now a contract clause rather than a
        convention: the vertical component of the same estimate is dead-reckoning
        artefact, not miss distance (C-POSSESS-1 § 1). The formula is shared with
        the possession bound through ``ball_possession.lateral_miss_mm`` so the
        number the gate decides on and the number the operator reads can never be
        two different computations."""
        ref = self._catch_point_mm if ref_point_mm is None else ref_point_mm
        return lateral_miss_mm(
            (float(ball.position.x), float(ball.position.y),
             float(ball.position.z)), ref)

    def _build_toss_observations(self, now: float) -> TossObservations:
        """Assemble the toss FSM's observation snapshot (the toss sibling of
        :meth:`_build_observations`; shares the caches and the announced-ball latch).

        Toss-only constructions:
          - ``platform_levelled`` — ``trajectory/status``'s
            ``gravity_correction_loaded``, gated on that status being FRESH
            (``_TRAJ_STATUS_STALE_S``). Deliberately NOT
            ``RobotState.levelling_complete``: that flag is persisted on the
            Teensy per BOOT, so it still reads True after a relaunch has emptied
            trajectory_node's in-memory correction — the state
            ``REJECTED_NOT_LEVELLED`` exists to refuse. Observing the node that
            APPLIES the correction is the only reading that cannot lie about it,
            and the freshness gate is what stops a cached True from outliving
            the process that said it. Not gated on the correction being
            NON-identity: a genuinely level machine measures a zero offset;
            "levelled" means a measurement was taken and pushed, not that it was
            large;
          - ``hand_fresh`` / ``hand_parked`` — the throw-side hand-evidence chain: a
            dead hand link blinds release verification, and a kind-0 stroke commands
            ABSOLUTE positions from 0 rev, so dispatch off the bottom park band
            (|pos| ≤ _HAND_NEAR_TARGET_REV of the retract target — the reload
            ladder's near-band) is a physical hazard;
          - ``ball_seated`` / ``ball_evidence`` — the ball-evidence precondition.
            ``ball_evidence`` is the LIVE tri-state hand-sensor read
            (``HandBallSensorSource.evidence``), which is C-POSSESS-1 § 3.3
            edit 1: before 2026-08-10 this was a plain read of the sticky
            ``_ball_possession`` latch, so a ball that bounced out during a
            ``TossContinuous`` dwell left the latch standing and cycle N+1 fired
            an empty stroke. ``ball_seated`` is the boolean the FSM gates on and
            ``ball_evidence`` only refines the REJECT CODE, so there is exactly
            one truth. When the gate is required (``toss_require_ball_evidence``,
            default TRUE since 2026-08-10): SEATED passes, EMPTY refuses
            ``REJECTED_NO_BALL``, and **UNKNOWN also refuses**
            (``REJECTED_BALL_UNKNOWN``) — a dead sensor must not silently allow
            throws, which is precisely the fail-open boot default this project
            refused to copy from BallButler. The trace-only waiver still forces a
            pass; ``toss_require_ball_evidence: false`` restores the
            pre-2026-08-10 unconditional pass and is the operator's escape hatch.
            The sticky latch is still maintained — SET by a plausible CAUGHT and
            now also by a valid SEATED, CLEARED on OUR release evidence and now
            also by a valid EMPTY (§ 3.3 edit 2) — but with the gate on it is no
            longer what CHECKING reads;
          - ``catch_event_dt_s`` — the sensor's arrival edge minus the predicted
            landing, i.e. WHEN the ball actually entered the cup. NaN until an
            edge lands inside the window. The first such quantity the machine has
            ever had (a tracker CAUGHT stamps when the marker VANISHED, which is
            a different event); it feeds the outcome line and the Phase-2
            learning loop, and it is a diagnostic — nothing branches on it;
          - ``track_active`` — a LIVE track (TO_BE_THROWN / IN_FLIGHT) already
            destined for us would correlate against OUR announcement (the F7
            phantom-track hole); consulted only at CHECKING, before our own
            announcement exists;
          - ``platform_at_target`` — measured platform-body position within
            _TOSS_POSITION_TOL_MM of the nominated pose, compared in the
            STOW/platform_start frame (see _TOSS_MOCAP_BODY_PARAM). Fed True
            UNCONDITIONALLY while the cross-check body is unconfigured (the
            shipped default — no platform body validated live), so arrival
            rests on the go_to_pose accept + timed wait;
          - ``throw_stroke_seen`` / ``ball_track_confirmed`` — the two independent
            release-evidence channels, both node-latched sticky for the goal and
            both GATED on _toss_throw_dispatched: release evidence cannot exist
            before OUR dispatch, so a pre-existing CONFIRMED phantom track must
            never read as "the ball left the cup" (which would clear possession
            and poison the goal with false release evidence);
          - ``ball_time_at_land_perf`` — the latched ball's live landing-plane
            crossing, ROS→perf crossed here (feeds achieved_flight_s)."""
        with self._lock:
            control_mode = self._control_mode
            streaming = self._streaming
            traj_status_mono = self._traj_status_mono
            correction_loaded = self._gravity_correction_loaded
            leg_limits_live = self._leg_limits_live
            mocap_fresh = (now - self._mocap_mono) < _MOCAP_STALE_S
            platform_pos = self._platform_pos_mm
            hand_pos = self._hand_pos_meas
            hand_mono = self._hand_telemetry_mono
            hand_vel = self._hand_vel_meas
            balls = self._balls
            balls_fresh = (now - self._balls_mono) < _STATUS_STALE_S
            possession = self._ball_possession
            landing_ref = self._toss_landing_global_mm
            platform_target = self._toss_platform_target_mm
            mocap_body = self._toss_mocap_body
            waiver = self._toss_waiver
            throw_dispatched = self._toss_throw_dispatched
        # platform_levelled — an AFFIRMATION, not an absence of evidence: it is
        # True only while trajectory_node is currently saying it holds a
        # correction. Same shape as hand_fresh (stamp > 0 ⇒ heard from at all,
        # then a window), a different window because the topic is 5 Hz — see
        # _TRAJ_STATUS_STALE_S.
        traj_status_fresh = (traj_status_mono > 0.0
                             and (now - traj_status_mono) < _TRAJ_STATUS_STALE_S)
        platform_levelled = bool(traj_status_fresh and correction_loaded)
        hand_fresh = hand_mono > 0.0 and (now - hand_mono) < _HAND_STATE_STALE_S
        hand_parked = hand_fresh and (
            abs(hand_pos - float(hw.JB_OP_HAND_RETRACT_REV))
            <= _HAND_NEAR_TARGET_REV)
        # Release-stroke telemetry watch (channel 1): only after the dispatch, and
        # only once the hand has LEFT the bottom park band (parked vel noise can
        # spike; see the ladder's park-band qualifiers).
        if (throw_dispatched and hand_fresh
                and hand_vel >= _THROW_STROKE_VEL_RPS
                and hand_pos > float(hw.JB_OP_HAND_RETRACT_REV)
                + _HAND_NEAR_TARGET_REV):
            with self._lock:
                self._toss_stroke_seen = True
        # THE live ball-evidence read (C-POSSESS-1 § 3.3 edit 1). One call, one
        # place — every downstream use below reads this variable, never the source
        # again, so the whole observation is built from ONE instant's cup state.
        ball_evidence = self._ball_sensor.evidence(now)
        landing_perf = self._expected_landing_perf()
        next_release_perf, next_landing_perf = self._expected_next_cycle_perf()
        catch_event_perf = self._ball_sensor.arrival_time(
            landing_perf, next_landing_t=next_landing_perf,
            prev_landing_t=self._expected_prev_landing_perf())
        catch_event_dt_s = (catch_event_perf - landing_perf
                            if (landing_perf is not None
                                and math.isfinite(catch_event_perf))
                            else float('nan'))
        track_active = False
        ball_caught = False
        catch_error_mm = float('nan')
        time_at_land_perf = float('nan')
        if balls_fresh:
            with self._lock:
                own_prev_id = self._prev_announced_ball_id
            # Census D6 — THE bug, and the docstring below already claimed the
            # fix. The gate exists to refuse a phantom destination='jugglebot'
            # track that would correlate against OUR announcement. The ball this
            # session just caught is not that: it is the one whose fate the
            # PREVIOUS cycle already adjudicated, and its track only leaves
            # IN_FLIGHT when the tracker mints CAUGHT (landing +0.202..+0.442 s).
            # At any dwell short enough for CHECKING to run inside that window —
            # every rung from R3 down — the unfixed gate hard-rejects
            # REJECTED_TRACK_ACTIVE on a perfectly healthy cycle, and it pins the
            # same floor the CAUGHT-verdict handoff does from a completely
            # independent direction.
            track_active = any(
                b.destination == self._robot_name
                and int(b.status) in (_BALL_STATUS_TO_BE_THROWN,
                                      _BALL_STATUS_IN_FLIGHT)
                and (own_prev_id is None or int(b.id) != int(own_prev_id))
                for b in balls)
            announced_id = self._update_announced_ball_latch(balls)
            if announced_id is not None:
                for b in balls:
                    if int(b.id) != announced_id:
                        continue
                    if (throw_dispatched
                            and int(getattr(b, 'tracking', 0))
                            == _BALL_TRACKING_CONFIRMED):
                        # Channel 2: a real mocap-marker match — sticky. Gated
                        # on OUR dispatch (mirroring the stroke watch): before
                        # it, a CONFIRMED track can only be a phantom, and
                        # latching it would mint false release evidence.
                        with self._lock:
                            self._toss_track_confirmed = True
                    time_at_land_perf = self._ball_time_at_land_perf(b, now)
                    if int(b.status) == _BALL_STATUS_CAUGHT:
                        if self._possession_confirmed(b, ref_point_mm=landing_ref):
                            ball_caught = True
                            catch_error_mm = self._catch_error_from_ball(
                                b, ref_point_mm=landing_ref)
                    break
        with self._lock:
            stroke_seen = self._toss_stroke_seen
            track_confirmed = self._toss_track_confirmed
            if (throw_dispatched and possession
                    and (stroke_seen or track_confirmed) and not ball_caught):
                # OUR release evidence: the ball left the cup. Gated on the
                # dispatch (both evidence latches already are — this is the
                # belt on the consumer side): possession must never clear off
                # a phantom's evidence before our stroke could have fired.
                # (A caught toss re-sets possession via _on_balls; a missed
                # one stays cleared. Never cleared on the tick that itself
                # observes the plausible CAUGHT — the ball is back in the cup,
                # and a track pruning right after the verdict would otherwise
                # leave a caught toss possession-less.)
                self._ball_possession = False
                possession = False
            # C-POSSESS-1 § 3.3 edit 2 — the latch can now be corrected WITHOUT
            # release evidence, because the sensor reads the cup directly. This
            # is what stops a bounce-out during a TossContinuous dwell from
            # leaving a belief the machine then acts on. UNKNOWN touches nothing:
            # blindness is not evidence in either direction.
            if ball_evidence == EVIDENCE_EMPTY:
                self._ball_possession = False
                possession = False
            elif ball_evidence == EVIDENCE_SEATED:
                self._ball_possession = True
                possession = True
        if not mocap_body:
            # Cross-check DISABLED (the shipped default — see
            # _TOSS_MOCAP_BODY_PARAM): arrival = go_to_pose accept + timed
            # wait; feeding True makes the FSM's timed arrival stand alone
            # without changing its POSITIONING semantics.
            platform_at_target = True
        else:
            platform_at_target = False
            if (mocap_fresh and platform_pos is not None
                    and platform_target is not None):
                err = np.linalg.norm(
                    np.asarray(platform_pos, dtype=float)
                    - np.asarray(platform_target, dtype=float))
                platform_at_target = bool(err <= _TOSS_POSITION_TOL_MM)
        return TossObservations(
            now=now,
            control_mode=control_mode,
            streaming=streaming,
            mocap_fresh=mocap_fresh,
            platform_levelled=platform_levelled,
            hand_fresh=hand_fresh,
            hand_parked=hand_parked,
            # NOT `or possession`: with the gate on, CHECKING reads the LIVE cup,
            # never the sticky latch (C-POSSESS-1 § 3.3 edit 1). UNKNOWN refuses
            # here and `ball_evidence` below tells the FSM which code to mint.
            ball_seated=bool(waiver
                             or not hw.JB_OP_TOSS_REQUIRE_BALL_EVIDENCE
                             or ball_evidence == EVIDENCE_SEATED),
            ball_evidence=ball_evidence,
            track_active=track_active,
            platform_at_target=platform_at_target,
            throw_stroke_seen=stroke_seen,
            ball_track_confirmed=track_confirmed,
            ball_caught=ball_caught,
            catch_error_mm=catch_error_mm,
            catch_event_dt_s=catch_event_dt_s,
            ball_time_at_land_perf=time_at_land_perf,
            # Fed ONLY off a FRESH trajectory/status (same window as
            # platform_levelled): a stale cache could carry a limits triple the
            # node has since ramped away from, and the reach bound must judge
            # against what the feasibility gate enforces NOW. Stale ⇒ 0.0 ⇒
            # the sequencer's YAML-default fallback (pre-field behaviour).
            leg_vel_limit_mmps=(leg_limits_live[0] if traj_status_fresh else 0.0),
            leg_acc_limit_mmps2=(leg_limits_live[1] if traj_status_fresh else 0.0),
            leg_jerk_limit_mmps3=(leg_limits_live[2] if traj_status_fresh else 0.0))

    def _ball_time_at_land_perf(self, ball, now_perf: float) -> float:
        """The tracker's live landing-plane crossing for ``ball``, crossed onto the
        FSM's perf clock (perf = ros + (now_perf − now_ros), the same crossing as
        :meth:`_announcement_landing_perf`). NaN when the stamp is absent/unusable."""
        tal = getattr(ball, 'time_at_land', None)
        ros_s = 0.0
        if tal is not None:
            try:
                ros_s = float(tal.sec) + float(tal.nanosec) * 1e-9
            except (AttributeError, TypeError, ValueError):
                ros_s = 0.0
        if ros_s <= 0.0:
            return float('nan')
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        return ros_s + (now_perf - now_ros)

    # ── Action callbacks ───────────────────────────────────────────────────────

    def _goal_callback(self, goal_request):
        # ONE ball-op at a time, across ALL THREE actions (shared by the reload,
        # toss and toss_continuous servers — a session holds the claim for its
        # WHOLE life, dwells included).
        # rclpy runs accepted goals concurrently, so a second goal accepted
        # while the first is live would double-own the hand on the Teensy's
        # last-writer-wins queue (one action's prime/retract erases the other's
        # armed stroke), fight over the single catch/armed latch, and scramble the
        # shared async-event routing (announcements / target feedback all fan out
        # to the single _active_seq). The claim is taken HERE, under _lock, at
        # ACCEPT — not at execute-callback entry — closing the accept→install
        # window in which a second goal's callback still saw _active_seq None;
        # it is released in the execute callbacks' finally with _active_seq.
        with self._lock:
            busy = self._goal_claimed or self._active_seq is not None
            if not busy:
                self._goal_claimed = True
                # The D6 exclusion spans CYCLES, never GOALS: a new goal's
                # CHECKING must judge every live track on its merits, and a
                # stale id from an earlier session would silently un-guard one.
                self._prev_announced_ball_id = None
        if busy:
            self.get_logger().warning(
                'Ball-op goal REJECTED_BUSY — a reload/toss/session is already '
                'in progress (one ball-op at a time, across all three '
                'actions).')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute_reload(self, goal_handle):
        throw_delay = float(getattr(goal_handle.request, 'throw_delay_s', 0.0) or 0.0)
        vel_scale = float(getattr(goal_handle.request, 'catch_vel_scale', 0.0) or 0.0)
        seq = ReloadSequencer(
            catch_point_mm=self._catch_point_mm, throw_delay_s=throw_delay)
        seq.start(time.perf_counter())
        if vel_scale < 0.0:
            # A sign typo must not silently become full speed (audit): warn and use
            # the default; the coordinator's clamp never sees the raw negative.
            self.get_logger().warning(
                f'catch_vel_scale {vel_scale:.2f} is negative — using the default '
                f'{hw.JB_OP_CATCH_VEL_SCALE_DEFAULT} (valid range 0.3-1.5)')
            vel_scale = 0.0
        with self._lock:
            self._active_seq = seq
            self._announced_ball_id = None
            self._announced_id_untagged = False
            self._preexisting_flight_ids = set()
            # 0 (field default) => JB_OP_CATCH_VEL_SCALE_DEFAULT (0.8, locked in
            # from the 2026-07-23 third sitting); coordinator clamps to its range.
            self._catch_vel_scale = (vel_scale if vel_scale > 0.0
                                     else float(hw.JB_OP_CATCH_VEL_SCALE_DEFAULT))
        self._possession_logged = set()

        result = Reload.Result()
        max_sequence_s = _sequence_deadline_s(seq)
        try:
            t_start = time.perf_counter()
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._safe_on_early_exit(seq)
                    goal_handle.canceled()
                    result.success = False
                    result.outcome = 'ABORTED_CANCELLED'
                    result.catch_error_mm = float('nan')
                    self._log_reload_outcome(result)
                    return result
                now = time.perf_counter()
                decision = self._step_sequence(seq, now, goal_handle)
                if decision.done:
                    r = decision.result
                    result.success = bool(r.success)
                    result.outcome = str(r.outcome)
                    result.catch_error_mm = float(r.catch_error_mm)
                    self._log_reload_outcome(r)
                    (goal_handle.succeed if r.success else goal_handle.abort)()
                    return result
                if now - t_start > max_sequence_s:
                    self._safe_on_early_exit(seq)
                    result.success = False
                    result.outcome = 'ABORTED_TIMEOUT'
                    result.catch_error_mm = float('nan')
                    self._log_reload_outcome(result)
                    goal_handle.abort()
                    return result
                time.sleep(_TICK_S)
            # rclpy shutting down.
            self._safe_on_early_exit(seq)
            result.success = False
            result.outcome = 'ABORTED_SHUTDOWN'
            result.catch_error_mm = float('nan')
            self._log_reload_outcome(result)
            return result
        finally:
            with self._lock:
                self._active_seq = None
                self._goal_claimed = False

    def _log_reload_outcome(self, result) -> None:
        """The single authoritative per-attempt outcome line, emitted on EVERY terminal
        (FSM-terminal and the node-level cancel/timeout/shutdown exits). Without it the
        node logged NO outcome at all (verified 0 lines, 2026-07-24) and a WORKING reload
        read as pure prime/retract-ladder failure spam — the operator could only infer
        success from the absence of errors. INFO on success, WARN otherwise. Accepts
        either the FSM's ReloadResult or the action's Reload.Result (same fields)."""
        err = float(result.catch_error_mm)
        suffix = f' (catch_err={err:.0f} mm)' if np.isfinite(err) else ''
        line = f'Reload {result.outcome}{suffix}'
        if result.success:
            self.get_logger().info(line)
        else:
            self.get_logger().warning(line)

    def _step_sequence(self, seq, now, goal_handle=None):
        """One FSM tick: build observations, step, execute the requested action, publish
        phase feedback. Returns the decision (testable in isolation)."""
        obs = self._build_observations(now)
        decision = seq.step(now, obs)
        if decision.action == ACTION_PRIME_HAND:
            # Retry ladder (see _HAND_DISPATCH_ATTEMPTS): the third sitting's goal 1
            # died to two back-to-back dispatch failures on a path failing ~40-60%
            # per call all session. Pre-throw, so the retries are free; exhausting
            # the ladder is a real fault and aborts as before.
            ok = self._prime_hand_with_retries()
            seq.note_prime_result(ok)
        elif decision.action == ACTION_CALL_RELOAD:
            self._call_reload()
        elif decision.action == ACTION_SEND_THROW:
            accepted, message = self._send_throw(seq)
            seq.note_throw_result(accepted, message)
        elif decision.action == ACTION_PREPARE_CATCH:
            seq.note_prepare_result(self._prepare_catch())
        elif decision.action == ACTION_RECENTER:
            self._recenter()
        elif decision.action == ACTION_SAFE_ABORT:
            self._safe_abort()
        if goal_handle is not None and not decision.done:
            fb = Reload.Feedback()
            fb.phase = decision.phase
            goal_handle.publish_feedback(fb)
        return decision

    # ── Toss action (jugglebot/toss) ───────────────────────────────────────────

    def _execute_toss(self, goal_handle):
        req = goal_handle.request
        catch_pose = (float(req.catch_position.x), float(req.catch_position.y),
                      float(req.catch_position.z))
        height = float(getattr(req, 'throw_height_m', 0.0) or 0.0)
        throw_delay = float(getattr(req, 'throw_delay_s', 0.0) or 0.0)
        vel_scale = float(getattr(req, 'catch_vel_scale', 0.0) or 0.0)
        # Open the record context FIRST, so a REJECTED_BAD_GOAL terminal below
        # declares its own identity instead of the previous goal's (instrument
        # only — see _publish_toss_record).
        self._open_toss_record(
            action='toss', goal_id=_goal_id_hex(goal_handle), cycle_index=1,
            catch_pose=catch_pose, throw_delay=throw_delay, vel_scale=vel_scale,
            raw_goal={'throw_height_m': height, 'throw_delay_s': throw_delay,
                      'catch_vel_scale': vel_scale})
        self._toss_trim_begin(goal_id=_goal_id_hex(goal_handle))
        # Loud goal-numerics gate BEFORE anything runs (mirrors ball_butler_node's
        # non-finite target guard): a NaN/inf would flow through the release-state
        # ballistics into a NaN announcement / event_vel command, and a NEGATIVE
        # tunable is a sign typo that must never silently coerce to a default
        # (0.0 is the only "use the default" sentinel). Rejected here nothing has
        # run: no FSM, no release state, no per-goal state install.
        bad_field = self._invalid_toss_goal_field(
            catch_pose, height, throw_delay, vel_scale)
        if bad_field is not None:
            self.get_logger().error(
                f'Toss goal REJECTED_BAD_GOAL({bad_field}): '
                f'catch_position={catch_pose}, throw_height_m={height}, '
                f'throw_delay_s={throw_delay}, catch_vel_scale={vel_scale} '
                f'— refusing before anything runs.')
            result = Toss.Result()
            result.success = False
            result.outcome = 'REJECTED_BAD_GOAL({})'.format(bad_field)
            result.catch_error_mm = float('nan')
            result.achieved_flight_s = float('nan')
            self._log_toss_outcome(result)
            goal_handle.abort()
            with self._lock:
                self._goal_claimed = False
            return result
        flight = self._resolve_toss_flight_s(height)
        seq = self._build_toss_cycle(catch_pose, flight, throw_delay, vel_scale)
        self._open_toss_record(
            action='toss', goal_id=_goal_id_hex(goal_handle), cycle_index=1,
            catch_pose=catch_pose, flight=flight,
            throw_delay=throw_delay, vel_scale=vel_scale,
            raw_goal={'throw_height_m': height, 'throw_delay_s': throw_delay,
                      'catch_vel_scale': vel_scale})

        result = Toss.Result()

        def _publish_phase(phase):
            fb = Toss.Feedback()
            fb.phase = phase
            goal_handle.publish_feedback(fb)

        try:
            r, exit_kind = self._run_toss_cycle(
                seq,
                deadline_s=_toss_deadline_s(seq),
                cancel_now_fn=lambda now: (
                    goal_handle.is_cancel_requested
                    and not self._toss_cancel_deferred(seq, now)),
                feedback_fn=_publish_phase)
            result.success = bool(r.success)
            result.outcome = str(r.outcome)
            result.catch_error_mm = float(r.catch_error_mm)
            result.achieved_flight_s = float(r.achieved_flight_s)
            if exit_kind == 'shutdown':
                # SHUTDOWN terminalises NOTHING on the handle — the behaviour
                # every release of this node has had, and it must survive the
                # extraction that gave the session the same code path. rclpy is
                # tearing down; a goal-status transition on a dying executor can
                # itself raise, and the except below would then overwrite the
                # ABORTED_SHUTDOWN line already in the log with a spurious
                # ABORTED_EXCEPTION and re-raise a fault trace out of a clean
                # shutdown. TossContinuous skips the same way, deliberately.
                return result
            if goal_handle.is_cancel_requested:
                # Covers BOTH the honoured cancel and a DEFERRED one (past the
                # cutoff / in flight), which resolves here with the FSM's real
                # outcome — the catch attempt ran to its own terminal, exactly
                # the plan's §7 semantics.
                goal_handle.canceled()
            elif r.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result
        except Exception:
            # Unexpected fault mid-sequence: _run_toss_cycle has already safed
            # the robot and emitted the outcome line. Abort the goal and
            # re-raise so the executor's own error path still sees the fault.
            result.success = False
            result.outcome = 'ABORTED_EXCEPTION'
            result.catch_error_mm = float('nan')
            result.achieved_flight_s = float('nan')
            try:
                goal_handle.abort()
            except Exception:  # noqa: BLE001
                # The handle may already be terminal (fault after succeed/abort);
                # the original exception must survive the cleanup.
                self.get_logger().error(
                    'ABORTED_EXCEPTION: goal_handle.abort() itself failed')
            raise
        finally:
            self._clear_toss_cycle_state()
            # The trim is per GOAL, so it dies here — with its proposal written.
            # In the `finally` deliberately: a cancelled or aborted goal's trim
            # is exactly the one an operator wants to read.
            self._toss_trim_end()
            with self._lock:
                self._goal_claimed = False

    @staticmethod
    def _resolve_toss_flight_s(height: float) -> float:
        """Goal ``throw_height_m`` ⇒ the internal flight time (s), in ONE place
        shared by the single Toss and every TossContinuous cycle."""
        if height == 0.0:
            # 0 ⇒ the generated config default FLIGHT TIME, resolved HERE so the
            # FSM receives the config-resolved value (kept internally as a flight
            # time — the FSM's DEFAULT_TOSS_FLIGHT_TIME_S literal is the no-config
            # fallback only, drift-guard-pinned equal; ~0.784 m apex at 0.8 s).
            return float(hw.JB_OP_TOSS_FLIGHT_TIME_DEFAULT_S)
        # Operator nominates a HEIGHT (m, apex above release); convert ONCE to
        # the internal flight time via the single tested toss_release module.
        return flight_time_from_height(height)

    # ── Toss AIM calibration (contract C-TOSS-CAL-1) ───────────────────────────

    def _load_toss_cal(self):
        """Resolve and load ``config/toss_calibration.yaml``. Returns (ok, message).

        **Never raises** — it runs in ``__init__`` and in a service callback, and
        C-TOSS-CAL-1 is non-gating: no state of the calibration file may stop
        this node coming up, take down a callback, or change a rejection code.

        Three outcomes, the ``tilt_map`` loader's shape verbatim:

        * **Loaded** — every validation in ``toss_cal.parse_toss_cal`` passed.
          Loaded is NOT applied: the provenance gate (D3) decides that per goal
          and is reported separately.
        * **No file** — the map is UNLOADED and ``ok`` is True. Reload's contract
          is "make this node's map agree with the file", and absence is a
          legitimate, non-gating state producing **exactly the pre-2b toss**.
          It is also load-bearing for the acquisition tool's
          ``--force-uninstall``, which moves the file aside and reloads precisely
          to reach ``toss_cal_loaded == false`` before a capture — keeping a
          stale in-memory map there would bake the map into its own successor.
        * **Invalid** — the previous map is **kept**, ``toss_cal_loaded`` is
          unchanged, ``ok`` is False, ERROR logged. All or nothing: a
          half-trusted aim map is indistinguishable at the machine from a
          correct one until a ball misses.
        """
        candidates = toss_cal.toss_cal_candidates()
        path = toss_cal.resolve_toss_cal_path()

        if path is None:
            with self._lock:
                had_map = self._toss_cal is not None
                previous = self._toss_cal_version
                self._toss_cal = None
                self._toss_cal_version = ''
                self._toss_cal_path = ''
                self._toss_cal_dormant_reason = ''
                self._toss_cal_dormant_logged = ''
                self._toss_cal_loaded = False
            # `candidates` holds one, two or three paths (no source tree for a
            # deployment outside the repo, no share dir for an unsourced
            # overlay), so never index it.
            message = ("no toss aim calibration found (tried: "
                       + ('; '.join(candidates) if candidates
                          else '<no candidate paths — set $'
                               + toss_cal.TOSS_CAL_ENV + '>')
                       + ") — tosses are aimed vertically, exactly as before "
                         "phase 2b (C-TOSS-CAL-1: absence is silent)")
            override = os.environ.get(toss_cal.TOSS_CAL_ENV)
            if had_map:
                message = (f"toss calibration file is gone — map {previous} "
                           f"UNLOADED. " + message)
                self.get_logger().warning(message)
            elif override:
                # A SET-but-missing override is not plain absence: the operator
                # asked for a specific calibration and is getting none. The
                # override is authoritative (it is the ONLY candidate), so a typo
                # silently un-aims the whole session while the operator believes
                # a map is applied — INFO would bury that.
                message = (f"${toss_cal.TOSS_CAL_ENV} is set to {override!r} "
                           f"but NO FILE EXISTS there — the override is "
                           f"authoritative, so nothing else was tried and this "
                           f"session throws UNAIMED. Fix the path or unset the "
                           f"variable. " + message)
                self.get_logger().warning(message)
            else:
                self.get_logger().info(message)
            self._publish_toss_cal_status()
            return True, message

        try:
            loaded = toss_cal.load_toss_cal(path)
        except toss_cal.TossCalError as exc:
            with self._lock:
                kept = (self._toss_cal_version if self._toss_cal is not None
                        else 'none')
            message = (f"toss calibration REJECTED (nothing loaded from this "
                       f"file): {exc} — still applying map [{kept}]")
            self.get_logger().error(message)
            self._publish_toss_cal_status()
            return False, message
        except Exception as exc:                                   # noqa: BLE001
            # load_toss_cal's contract is that every failure is a TossCalError,
            # so this is a defect rather than a bad file — but it arrives on a
            # callback (or in __init__) where raising would take the ball-ops
            # node down over a *refinement*. Report it as a rejection, keep
            # running, keep throwing.
            with self._lock:
                kept = (self._toss_cal_version if self._toss_cal is not None
                        else 'none')
            message = (f"toss calibration loader FAILED unexpectedly on {path}: "
                       f"{type(exc).__name__}: {exc} — still applying map "
                       f"[{kept}]")
            self.get_logger().error(message)
            self._publish_toss_cal_status()
            return False, message

        n_y, n_x = loaded.shape
        with self._lock:
            self._toss_cal = loaded
            self._toss_cal_version = str(loaded.version)
            self._toss_cal_path = str(path)
            self._toss_cal_dormant_reason = ''
            self._toss_cal_dormant_logged = ''
            self._toss_cal_loaded = True
            live_tilt = self._tilt_map_version
        message = (f"toss aim calibration loaded: version={loaded.version} "
                   f"grid={n_x}x{n_y} z_mm={loaded.z_mm} from {path}")
        self.get_logger().info(message)
        mismatch = loaded.provenance_mismatch(live_tilt)
        if mismatch:
            self.get_logger().warning(
                f"toss aim map [{loaded.version}] is LOADED but DORMANT — "
                f"{mismatch}. Nothing is applied: an aim residual fitted under "
                f"one levelling layer double-counts another's delta (D3). "
                f"Tosses aim vertically until the provenance agrees.")
        self._publish_toss_cal_status()
        return True, message

    def _svc_reload_toss_calibration(self, request, response):
        """``toss/reload_calibration`` (``std_srvs/Trigger``).

        Called by the phase-2c fit tool right after it writes the map file; the
        response message carries the loaded version and the applied/dormant
        verdict so the tool can read back what the node actually holds. The
        in-flight rule is inherited rather than restated: the aim is baked into
        a goal at ``_build_toss_cycle`` and nothing re-reads it, so a reload
        during a live cycle cannot re-aim it — the next goal picks it up.
        """
        ok, message = self._load_toss_cal()
        snapshot = self._toss_cal_status_snapshot()
        response.success = bool(ok)
        response.message = '{} | applied={} version={}'.format(
            message, snapshot['toss_cal_applied'],
            snapshot['toss_cal_version'] or 'none')
        return response

    def _toss_cal_status_snapshot(self) -> dict:
        """The observable calibration state — loaded / applied / version.

        ``loaded`` and ``applied`` are deliberately separate booleans, the same
        distinction C-LEVEL-2's review had to go back and fix in four documents:
        a map can be present, valid and *doing nothing* because its provenance
        does not match the machine it is running on.
        """
        with self._lock:
            cal = self._toss_cal
            live_tilt = self._tilt_map_version
            version = self._toss_cal_version
            path = self._toss_cal_path
        reason = cal.provenance_mismatch(live_tilt) if cal is not None else ''
        return {
            'toss_cal_loaded': bool(cal is not None),
            'toss_cal_applied': bool(cal is not None and not reason),
            'toss_cal_version': version,
            'toss_cal_path': path,
            'dormant_reason': reason or '',
            'requires_tilt_map_version': (
                str(cal.requires_tilt_map_version) if cal is not None else ''),
            'live_tilt_map_version': live_tilt,
            'estimator_version': toss_cal.ESTIMATOR_VERSION,
            # Layer 3 rides the SAME latched topic under a nested key rather than
            # claiming a second one: it is the same question ("what correction is
            # this machine applying to a toss goal?") one layer up, an operator
            # reading one and not the other would draw the wrong conclusion, and
            # a nested key is additive for every existing reader.
            'ilc': self._toss_ilc_status_snapshot(),
        }

    def _toss_ilc_status_snapshot(self) -> dict:
        """The observable layer-3 state — enabled / loaded / applied / version.

        THREE separate booleans, not two, and each answers a question the other
        two cannot: ``enabled`` is the shipped config flag (a build decision),
        ``loaded`` is "a valid artifact is present", and ``applied`` is "and the
        flag is on AND its provenance matches this machine". An operator
        debugging "why did nothing change?" needs to be able to tell a disabled
        build from a missing file from a dormant artifact, and collapsing any
        pair of them is how the aim map's own loaded-vs-applied confusion had to
        be fixed in four documents.

        ``applied`` deliberately folds the flag in, so it means exactly what the
        per-toss record means by a non-zero ``ilc_aim_rad``: *this machine is
        commanding the learned correction*. A topic that said "applied" beside a
        record of zeros would be the same confusion in a new place.

        The provenance verdict is GUARDED for the reason
        :meth:`_toss_aim_for_goal` gives — ``provenance_mismatch`` hashes six
        generated ``hardware_config`` constants and an install-tree skew raises
        there — and it must fail the SAME way in both places, or the topic would
        report a correction the apply seam has already withheld. This one also
        rides a Trigger service response (``toss/reload_calibration``), so an
        unguarded raise costs an operator their read-back too.
        """
        with self._lock:
            ilc = self._toss_ilc
            live_tilt = self._tilt_map_version
            version = self._toss_ilc_version
            path = self._toss_ilc_path
        enabled = self._toss_ilc_enabled()
        reason = ''
        if ilc is not None:
            try:
                reason = ilc.provenance_mismatch(
                    tilt_map_version=live_tilt,
                    toss_cal_version=self._applied_toss_cal_version()) or ''
            except Exception as exc:                               # noqa: BLE001
                reason = ('the provenance check itself FAILED ({}: {})'
                          .format(type(exc).__name__, exc))
        try:
            identity = toss_ilc.model_config_identity()
        except Exception as exc:                                   # noqa: BLE001
            identity = 'UNAVAILABLE ({})'.format(type(exc).__name__)
        return {
            'ilc_enabled': enabled,
            'ilc_loaded': bool(ilc is not None),
            'ilc_applied': bool(enabled and ilc is not None and not reason),
            'ilc_version': version,
            'ilc_path': path,
            'ilc_cells': int(ilc.n_cells) if ilc is not None else 0,
            'dormant_reason': reason,
            'estimator_version': toss_ilc.ESTIMATOR_VERSION,
            'model_config_identity': identity,
        }

    @staticmethod
    def _toss_ilc_enabled() -> bool:
        """The shipped layer-3 gate, read **fail-closed**.

        ``getattr(..., False)`` rather than a direct attribute read, deliberately:
        the flag is generated into ``jugglebot/hardware_config.py`` from
        ``config/hardware_config.yaml``, and an install tree that predates the
        codegen simply does not have the name. Fail-closed there means "the
        learned correction stays off until the build is complete", which is the
        only safe reading — an ``AttributeError`` on a goal-build path would take
        down the toss instead.
        """
        return bool(getattr(hw, 'JB_OP_TOSS_ILC_ENABLED', False))

    def _applied_toss_cal_version(self) -> str:
        """The aim-map version this machine is **APPLYING**, or ``''``.

        NOT ``_toss_cal_version``, which is set for a loaded-but-DORMANT map too.
        Layer 3's provenance gate has to compare against what is actually being
        commanded: a dormant layer 1 aims vertically, which is the same state as
        no map at all, and an ILC residual fitted on top of an applied aim map
        double-counts it when that map stops applying.
        """
        with self._lock:
            cal = self._toss_cal
            version = self._toss_cal_version
            live_tilt = self._tilt_map_version
        if cal is None:
            return ''
        return '' if cal.provenance_mismatch(live_tilt) else str(version)

    def _load_toss_ilc(self):
        """Resolve and load ``config/toss_ilc.yaml``. Returns ``(ok, message)``.

        Four outcomes, exactly the aim map's, and non-gating in all four:

        * **Absent** — silent. ``None`` from the resolver means no correction
          exists, which is the pre-Phase-2 machine. It is also the Phase-3 A/B's
          baseline arm (``$JUGGLEBOT_TOSS_ILC`` pointed at a path that is not
          there), so it must stay quiet and free.
        * **Loaded** — every validation in ``toss_ilc.parse_toss_ilc`` passed.
          Whether it is APPLIED is a separate, per-goal provenance question.
        * **Invalid** — nothing is loaded and the failure is an ERROR. There is
          no "keep the previous artifact" case to handle here (unlike the aim
          map's reload service) because this loads exactly once, at construction.
        * **Present but the feature is OFF** — still loaded and still reported.
          Loading costs one file read at construction, never inside a control
          loop, and an operator who has shipped an artifact needs to see that the
          node can read it *before* the config commit that arms it.
        """
        candidates = toss_ilc.toss_ilc_candidates()
        path = toss_ilc.resolve_toss_ilc_path()
        enabled = self._toss_ilc_enabled()
        if path is None:
            with self._lock:
                self._toss_ilc = None
                self._toss_ilc_loaded = False
                self._toss_ilc_version = ''
                self._toss_ilc_path = ''
                self._toss_ilc_dormant_reason = ''
                self._toss_ilc_dormant_logged = ''
            message = ('no ILC artifact found (searched: {})'
                       .format(', '.join(candidates) or
                               '<nothing — set $' + toss_ilc.ILC_ENV + '>'))
            override = os.environ.get(toss_ilc.ILC_ENV)
            if override:
                # An explicitly NAMED artifact that is not there is worth one
                # WARN per launch. It fires for the Phase-3 A/B's baseline arm
                # too (flag on, override pointing at nothing) and that is the
                # right behaviour, not noise: the operator running the baseline
                # gets confirmation the override took effect, and the operator
                # who typo'd a path gets told before a whole sitting is captured
                # against a correction that was never applied. Silent absence is
                # reserved for the case with no override at all.
                self.get_logger().warning(
                    '${} names {!r}, which does not exist — this toss session '
                    'applies NO learned ILC correction'.format(
                        toss_ilc.ILC_ENV, override))
            return False, message

        try:
            loaded = toss_ilc.load_toss_ilc(path)
        except toss_ilc.TossIlcError as exc:
            with self._lock:
                self._toss_ilc = None
                self._toss_ilc_loaded = False
                self._toss_ilc_version = ''
                self._toss_ilc_path = ''
            self.get_logger().error(
                'ILC artifact REJECTED, nothing loaded: {} — this session '
                'throws with layer 3 contributing exactly zero (the learned '
                'correction is a refinement, never a gate)'.format(exc))
            return False, str(exc)
        except Exception as exc:                                   # noqa: BLE001
            # load_toss_ilc's contract is that every failure is a TossIlcError;
            # this branch exists so a contract violation degrades to "no
            # correction" instead of killing the node's constructor.
            with self._lock:
                self._toss_ilc = None
                self._toss_ilc_loaded = False
                self._toss_ilc_version = ''
                self._toss_ilc_path = ''
            self.get_logger().error(
                'ILC artifact {} raised an UNEXPECTED {} ({}) — nothing loaded'
                .format(path, type(exc).__name__, exc))
            return False, str(exc)

        with self._lock:
            self._toss_ilc = loaded
            self._toss_ilc_version = str(loaded.version)
            self._toss_ilc_path = str(path)
            self._toss_ilc_dormant_reason = ''
            self._toss_ilc_dormant_logged = ''
            self._toss_ilc_loaded = True
        message = ('ILC artifact [{}] loaded from {} ({} goal cells)'
                   .format(loaded.version, path, loaded.n_cells))
        if enabled:
            self.get_logger().info(message)
        else:
            self.get_logger().info(
                '{} — but jugglebot_operational.toss_ilc_enabled is FALSE, so '
                'layer 3 contributes exactly zero this session. Arming it is a '
                'reviewed config commit + codegen + colcon build.'
                .format(message))
        return True, message

    def _publish_toss_cal_status(self) -> None:
        """Publish the latched calibration status. Instrument only — every
        failure is swallowed, because an observability topic must never be able
        to take down the node that owns the hand and the abort ladder."""
        try:
            self._toss_cal_status_pub.publish(
                String(data=json.dumps(self._toss_cal_status_snapshot(),
                                       sort_keys=True)))
        except Exception as exc:                                   # noqa: BLE001
            self.get_logger().warning(
                'toss/calibration_status publish failed: {}'.format(exc))

    def _toss_aim_for_goal(self, catch_pose, flight):
        """THE single per-goal aim lookup (contract C-TOSS-CAL-1, D4).

        Called from exactly one place — :meth:`_build_toss_cycle` — and nowhere
        else. Not the 40 Hz emitter, not per Hermite knot, not
        ``catch_coordinator``, and never on the reload path: a BallButler ball's
        aim belongs to BallButler. ``tests/motion/test_toss_cal.py`` enforces
        that structurally, in the shape of C-LEVEL-2's own manifest test.

        Returns the applied-calibration block for the whole goal, split into its
        three contributions so the record can carry all of them:

        * ``map_aim_rad`` / ``map_offset_mm`` — layer 1, the persistent
          home-referenced field (:mod:`jugglebot.motion.toss_cal`);
        * ``trim_aim_rad`` / ``trim_offset_mm`` — layer 2's contribution to the
          COMMANDED aim. **Structurally zero since 2026-08-21**: the layer-2 aim
          estimator is MONITOR-ONLY (``toss_trim.AIM_AUTHORITY``, owner decision
          1, contradiction C4). Its estimate is carried beside them as
          ``trim_monitor_aim_rad``, so a future reader can see what layer 2
          would have asked for and how far it diverged from layer 3 — but
          nothing composes it;
        * ``ilc_aim_rad`` / ``ilc_offset_mm`` / ``ilc_vel_trim`` — layer 3, the
          learned critical-point correction (:mod:`jugglebot.motion.toss_ilc`),
          zero unless ``jugglebot_operational.toss_ilc_enabled`` is set AND an
          artifact is loaded AND its provenance matches. A provenance verdict
          that cannot be COMPUTED counts as a mismatch, on
          ``provenance_mismatch``'s own rule: *"I cannot verify what is
          underneath me" is not "the right thing is underneath me"*.
          ``ilc_aim_rad`` is layer 3's TOTAL and is split, additively, into:

          * ``ilc_spatial_aim_rad`` — the per-cell spatial residual, zero unless
            this goal HITS one of the artifact's cells (a miss is exactly zero;
            nothing is interpolated);
          * ``ilc_session_aim_rad`` — the goal-local common mode, seeded at goal
            start from the artifact's ``anchor`` prior and applied only if that
            prior clears its evidence gate. It does **not** depend on a cell hit,
            because a common mode is not a function of the cell — see the layer-3
            block below for that argument and the tradeoff it accepts. Zero, with
            a named ``ilc_session_reason``, on every other path;
        * ``aim_rad`` / ``offset_mm`` — the TOTAL, which is what the platform is
          actually commanded to and what the virtual target is built from.

        **The TOTAL is re-clamped HERE, at apply** (D7), over ``map + ilc`` —
        not at any layer's own update. Each layer bounds itself (parse time for
        the map and the ILC) but 1.0° + 1.0° is 2.0°, past the authority every
        downstream argument is sized on, and the only place that sum exists is
        this line.
        Layer 3 is composed BEFORE that single clamp, deliberately: it makes the
        existing D7 clamp the final authority over the whole commanded aim rather
        than adding a fourth bound after it.

        **A clamp HIT REFUSES the ILC contribution; it never truncates it.** If
        ``map + ilc`` would bind the authority, layer 3 is dropped whole, a WARN
        is emitted, and the aim is re-composed from ``map`` exactly as it would
        have been without this layer — so a saturated goal is bit-identical to
        the pre-Phase-2 machine. Root cause, and it is the plan's own risk 5: a
        partially truncated correction is not the correction that was solved for,
        so applied-u and recorded-u desynchronise and the next fit learns against
        a command the machine never flew. It is also
        ``ilc_fit_lib.admit_command``'s convention — the fit refuses a step the
        clamp would truncate and shrinks its trust region instead — and the two
        halves of one loop must agree about what "refused" means. The clamp's
        TRUNCATION branch is layer 1's long-standing behaviour and is untouched,
        but nothing can reach it any more: ``parse_toss_cal`` bounds a map node
        by ``TOTAL_MAX_RAD`` on the same magnitude ``clamp_total_aim`` bounds, so
        with layer 2 at zero authority every loadable map is already inside the
        clamp (pinned by ``tests/ros/test_toss_ilc_node.py::
        test_the_D7_clamp_can_no_longer_BIND_without_layer_3``).

        ``aim_rad`` is exactly ``(0.0, 0.0)`` — and ``offset_mm`` exactly
        ``(0.0, 0.0)`` — whenever no map is loaded (or it is dormant), the trim
        commands nothing and layer 3 contributes nothing, which is what makes the
        disabled path today's machine bit for bit.

        **Layer 2 does not depend on layer 1's dormancy** — historically because
        the trim was measured *this goal* against *this* layer 0 and so stayed
        valid when a map fitted under another one did not. That argument is now
        moot for the COMMAND (layer 2 commands nothing) and survives only as the
        reason its monitor read-out is still trustworthy under a dormant map.
        **Layer 3 DOES depend on it**, and that asymmetry is not an
        inconsistency: layer 3's provenance explicitly records which aim map was
        being APPLIED underneath it, so a layer 1 that stops applying is a
        recorded premise of the fit going false.
        """
        with self._lock:
            cal = self._toss_cal
            live_tilt = self._tilt_map_version
            version = self._toss_cal_version
            already_logged = self._toss_cal_dormant_logged
            trim = self._toss_trim
            ilc = self._toss_ilc
            ilc_version = self._toss_ilc_version
            ilc_already_logged = self._toss_ilc_dormant_logged
            session = self._toss_ilc_session
        reason = cal.provenance_mismatch(live_tilt) if cal is not None else ''
        if reason:
            with self._lock:
                self._toss_cal_dormant_reason = reason
                self._toss_cal_dormant_logged = reason
            if reason != already_logged:
                self.get_logger().warning(
                    f"toss aim map [{version}] is LOADED but DORMANT — {reason}. "
                    f"This goal aims vertically. Re-fit the map against the live "
                    f"levelling layer, or reload the tilt map it was captured "
                    f"under (D3).")
        block = {
            'aim_rad': (0.0, 0.0),
            'offset_mm': (0.0, 0.0),
            'map_aim_rad': (0.0, 0.0),
            'map_offset_mm': (0.0, 0.0),
            'trim_aim_rad': (0.0, 0.0),
            'trim_offset_mm': (0.0, 0.0),
            # Layer 2's estimate, and what it would have commanded if it still
            # had authority. Separate keys from `trim_aim_rad` deliberately:
            # `trim_aim_rad` means "what layer 2 CONTRIBUTED to the commanded
            # aim", which is now structurally zero, and every consumer that
            # reconstructs the applied aim from the record (`applied_aim_rad`'s
            # map+trim fallback, the miner, the fit) must keep reading it that
            # way or a monitor value would be re-applied on paper.
            'trim_monitor_aim_rad': (0.0, 0.0),
            'trim_authority': toss_trim.AIM_AUTHORITY,
            # `ilc_aim_rad` is layer 3's TOTAL contribution to the commanded aim
            # and stays that: every consumer that subtracts what layer 3 applied
            # — `toss_trim.ilc_aim_rad`'s C4 subtraction above all — needs the
            # sum, not a part. The two components ride beside it.
            'ilc_aim_rad': (0.0, 0.0),
            'ilc_spatial_aim_rad': (0.0, 0.0),
            'ilc_session_aim_rad': (0.0, 0.0),
            'ilc_session_applied': False,
            'ilc_session_reason': toss_ilc.SESSION_NO_ARTIFACT,
            'ilc_session_n': 0,
            'ilc_offset_mm': (0.0, 0.0),
            'ilc_vel_trim': 0.0,
            'ilc_enabled': False,
            'ilc_loaded': bool(ilc is not None),
            'ilc_applied': False,
            'ilc_version': ilc_version,
            'ilc_key': None,
            'ilc_hit': False,
            'ilc_refused': '',
            'clamp_hits': [],
            'loaded': bool(cal is not None),
            'applied': bool(cal is not None and not reason),
            'trim_enabled': False,
            'version': version,
        }

        # ── layer 1 ──
        map_aim = (0.0, 0.0)
        if cal is not None and not reason:
            try:
                map_aim = toss_cal.lookup(cal, float(catch_pose[0]),
                                          float(catch_pose[1]))
            except (toss_cal.TossCalError, ValueError) as exc:
                # Degrade this goal to an unaimed toss rather than converting a
                # calibration fault into a dead action callback. Loud, and per
                # goal: unlike the tilt map's per-pose lookup this runs ~10 times
                # a minute, so it cannot bury the console.
                self.get_logger().error(
                    f"toss aim lookup FAILED for catch pose {tuple(catch_pose)}: "
                    f"{exc} — this goal is thrown UNAIMED (C-TOSS-CAL-1 is a "
                    f"refinement, never a gate)")
                block['applied'] = False
                map_aim = (0.0, 0.0)

        # ── layer 2 — THE single per-goal trim read (§ 3.6), MONITOR ONLY ──
        #
        # ZERO AUTHORITY since 2026-08-21 (``toss_trim.AIM_AUTHORITY``, owner
        # decision 1 / contradiction C4). The estimator still runs, still guards,
        # still writes its proposal — but its aim is RECORDED, not commanded, and
        # `trim_aim` below is a hard zero rather than a value that happens to be
        # zero. Root cause: two converging estimators of one quantity
        # double-count in both directions (the machine over-aims by the ILC's
        # contribution while the trim reports CONVERGED; and a converged trim
        # makes the ILC unlearn itself to zero). The trim reduces `land_err_mm`
        # while the ILC's aim update is driven by `arrival_dir`, so keeping both
        # read-outs alive makes their disagreement visible per toss — which is
        # the standing validation of C3's by-decision resolution.
        trim_aim = (0.0, 0.0)
        enabled = bool(self.get_parameter(_TOSS_TRIM_PARAM).value)
        block['trim_enabled'] = enabled
        block['trim_authority'] = toss_trim.AIM_AUTHORITY
        if trim is not None:
            monitor = trim.aim()
            block['trim_monitor_aim_rad'] = (float(monitor[0]),
                                             float(monitor[1]))

        # ── layer 3 — THE single per-goal ILC lookup (critical-point-ilc.md) ──
        #
        # Layer 3's aim has TWO components since 2026-08-21 (owner decision 2,
        # contradiction C1) and both are read here, once:
        #
        #   ilc_spatial — the per-cell SPATIAL RESIDUAL from `toss_ilc.lookup`.
        #   ilc_session — the goal-local COMMON MODE, seeded at goal start from
        #                 the artifact's anchor prior and gated on its evidence.
        #
        # Root cause for the split, not the decision by name: `level()` is one
        # int16 SCL3300 sample with 1.2-1.7 mrad/axis of session-to-session
        # scatter, and D3 says a re-`level` deliberately does NOT invalidate a
        # persisted map. Against the measured per-cell |aim| of 9.1-10.5 mrad
        # that is 11-19 % of every persisted cell being one sitting's
        # inclinometer noise, frozen forever and re-applied on every future
        # session that never took that draw. None of layer 3's four provenance
        # keys can see a re-`level`, so the fence has to be structural: the
        # common mode is persisted as a PRIOR, seeded into a RAM-only component
        # that dies with the goal, and no cell ever absorbs a `level()` draw.
        ilc_spatial = (0.0, 0.0)
        ilc_session = (0.0, 0.0)
        ilc_aim = (0.0, 0.0)
        ilc_dv = 0.0
        ilc_enabled = self._toss_ilc_enabled()
        block['ilc_enabled'] = ilc_enabled
        if ilc_enabled and ilc is not None:
            # `'' if (cal is None or reason)` IS `_applied_toss_cal_version()`,
            # inlined because this scope already holds `cal`, `reason` and
            # `version` under one lock acquisition and the helper would re-take
            # the lock and re-run the provenance verdict on the goal-build path.
            # The two must agree — a divergence would make the status topic and
            # the applied correction disagree about which aim map is underneath.
            #
            # GUARDED, and it is not defensive noise. This runs on the GOAL-BUILD
            # path, and `provenance_mismatch` is not a pure string compare: it
            # calls `model_config_identity()`, which reads six generated
            # `hardware_config` constants through `getattr` and floats them. An
            # install tree that predates a codegen — the same skew
            # `_toss_ilc_enabled`'s `getattr(..., False)` already fails closed on
            # — raises `AttributeError` there, and an unguarded raise here does
            # not cost the correction, it kills `_build_toss_cycle` and with it
            # the toss. Layer 3 is a refinement, never a gate (the same rule the
            # `toss_cal.lookup` and `toss_ilc.lookup` guards below implement), so
            # the failure is fail-CLOSED: zero correction, dormant, loud.
            try:
                ilc_reason = ilc.provenance_mismatch(
                    tilt_map_version=live_tilt,
                    toss_cal_version=('' if (cal is None or reason)
                                      else str(version))) or ''
            except Exception as exc:                               # noqa: BLE001
                ilc_reason = (
                    'the provenance check itself FAILED ({}: {}) — "I cannot '
                    'verify what is underneath me" is not "the right thing is '
                    'underneath me"'.format(type(exc).__name__, exc))
            block['ilc_applied'] = not ilc_reason
            if ilc_reason:
                with self._lock:
                    self._toss_ilc_dormant_reason = ilc_reason
                    self._toss_ilc_dormant_logged = ilc_reason
                if ilc_reason != ilc_already_logged:
                    self.get_logger().warning(
                        f"ILC artifact [{ilc_version}] is LOADED but DORMANT — "
                        f"{ilc_reason}. This goal gets ZERO learned correction. "
                        f"Re-fit against the live layers, or restore the ones it "
                        f"was fitted under (design constraint 5).")
            else:
                try:
                    hit = toss_ilc.lookup(ilc, float(catch_pose[0]),
                                          float(catch_pose[1]),
                                          float(catch_pose[2]), float(flight))
                except (toss_ilc.TossIlcError, ValueError) as exc:
                    # Same degradation the aim map takes: a lookup fault costs
                    # the refinement, never the goal.
                    self.get_logger().error(
                        f"ILC lookup FAILED for goal {tuple(catch_pose)} at "
                        f"T={flight}: {exc} — this goal gets ZERO learned "
                        f"correction (layer 3 is a refinement, never a gate)")
                    block['ilc_applied'] = False
                    hit = None
                # THE single per-goal read of the session common mode. It runs
                # inside this branch, and only here, so that layer 3's dormancy
                # verdict has exactly ONE owner: a DORMANT artifact contributes
                # no prior for the same reason it contributes no cell, and the
                # verdict is not re-derived anywhere it could drift from.
                #
                # It runs BEFORE the hit/miss branch on purpose. A MISS
                # contributes exactly zero SPATIAL residual — `toss_ilc.lookup`'s
                # no-interpolation rule is untouched — but it does NOT zero the
                # common mode, and that asymmetry is the whole point of splitting
                # them. The no-interpolation rule exists because a sparse
                # command-vector table must not invent a value BETWEEN its cells;
                # the common mode is by construction not a function of the cell
                # (measured consistent, 9.1-10.5 mrad, across all three goal
                # cells), so applying it at an unvisited goal is applying a
                # constant, not interpolating a field. It is also exactly what
                # layer 2 has always done with its own common mode: the session
                # trim applies at every goal, whether or not the map has a node
                # there. The tradeoff accepted: at a goal far outside the fitted
                # region this extrapolates a constant that was measured inside
                # it — bounded by ILC_AIM_MAX_RAD, gated on evidence, and
                # recorded per toss as `ilc_session_aim_rad` so the assumption is
                # visible in the corpus rather than implicit.
                if session is not None:
                    ilc_session = session.aim()
                    block['ilc_session_aim_rad'] = (float(ilc_session[0]),
                                                    float(ilc_session[1]))
                    block['ilc_session_applied'] = bool(session.applied)
                    block['ilc_session_reason'] = str(session.reason)
                    block['ilc_session_n'] = int(session.n)
                if hit is not None:
                    block['ilc_key'] = tuple(hit.key)
                    block['ilc_hit'] = bool(hit.hit)
                    if hit.hit:
                        ilc_spatial = hit.correction.aim_rad
                        ilc_dv = float(hit.correction.event_vel_trim)
                    else:
                        # A MISS is EXACTLY zero and is said out loud once per
                        # cell: an operator who expected a correction needs to
                        # learn that this goal quantised somewhere the fit never
                        # visited, and silence would read as "applied, and it
                        # made no difference".
                        if hit.key not in self._toss_ilc_miss_logged:
                            self._toss_ilc_miss_logged.add(hit.key)
                            self.get_logger().info(
                                'ILC artifact [{}] has no cell for goal {} '
                                '(key {}) — ZERO learned correction for this '
                                'goal. Cells: {}'.format(
                                    ilc_version, tuple(catch_pose),
                                    list(hit.key), sorted(ilc.cells)))

        # Layer 3's TOTAL aim. The two components are summed here, before the
        # single D7 clamp, for the reason layer 3 as a whole is composed before
        # it: adding a bound after the clamp would make a fourth authority, and
        # the sum is what the platform is actually commanded to.
        ilc_aim = (ilc_spatial[0] + ilc_session[0],
                   ilc_spatial[1] + ilc_session[1])

        if (map_aim == (0.0, 0.0) and trim_aim == (0.0, 0.0)
                and ilc_aim == (0.0, 0.0) and ilc_dv == 0.0):
            # Not one floating-point operation on the disabled path.
            return block

        # `base` = every layer BELOW layer 3. `trim_aim` is a hard (0, 0) since
        # C4 and is kept in the expression on purpose: it makes the composition
        # read as the layered sum it is, it keeps the layer-3 mm split a
        # difference of two commanded offsets rather than a special case, and if
        # layer 2 were ever re-armed the sum would be right by construction
        # instead of needing this line rediscovered.
        base_rx = map_aim[0] + trim_aim[0]
        base_ry = map_aim[1] + trim_aim[1]
        try:
            rx, ry, hits = toss_cal.clamp_total_aim(base_rx + ilc_aim[0],
                                                    base_ry + ilc_aim[1])
            if hits and ilc_aim != (0.0, 0.0):
                # REFUSE layer 3 whole, then re-compose exactly as layers 1+2
                # would have on their own — see the method docstring (risk 5).
                #
                # WHOLE means BOTH components. Dropping only the spatial residual
                # and keeping the common mode (or the reverse) would fly a
                # correction no fit ever solved for: the cells are referenced TO
                # the anchor, so `spatial` alone is a residual about a baseline
                # the machine is not applying, and `session` alone is a baseline
                # with its residual removed. Half a decomposition is not a
                # smaller correction, it is a different one — the same reason a
                # truncation is refused rather than applied.
                if self._toss_ilc_refusal_logged != 'total_aim':
                    self._toss_ilc_refusal_logged = 'total_aim'
                    self.get_logger().warning(
                        'ILC aim contribution REFUSED at apply: map+ilc = '
                        '{:.6f} rad exceeds the {:.6f} rad D7 authority, and a '
                        'TRUNCATED correction is not the correction that was '
                        'solved for (risk 5). This goal flies the map aim '
                        'unchanged; the recorded ilc_aim_rad is (0, 0), and '
                        'BOTH layer-3 components are dropped (spatial '
                        '{:.6f} rad + session {:.6f} rad).'
                        .format(math.hypot(base_rx + ilc_aim[0],
                                           base_ry + ilc_aim[1]),
                                toss_cal.TOTAL_MAX_RAD,
                                math.hypot(*ilc_spatial),
                                math.hypot(*ilc_session)))
                block['ilc_refused'] = 'total_aim'
                ilc_aim = (0.0, 0.0)
                ilc_spatial = (0.0, 0.0)
                ilc_session = (0.0, 0.0)
                block['ilc_session_aim_rad'] = (0.0, 0.0)
                block['ilc_session_applied'] = False
                # ... and the REASON, which is the field a corpus consumer or a
                # bench operator actually routes on. Left at its pre-refusal
                # SESSION_APPLIED until 2026-08-22, so the record read
                # reason=APPLIED next to applied=False and aim=(0, 0) and the
                # landing error got attributed to a correction that was refused
                # (audit fix). Every sibling field in this branch is corrected;
                # this one was the miss.
                block['ilc_session_reason'] = toss_ilc.SESSION_REFUSED_TOTAL_AIM
                rx, ry, hits = toss_cal.clamp_total_aim(base_rx, base_ry)
            offset = aim_target_offset_mm(rx, ry, float(flight),
                                          float(catch_pose[2]))
            map_offset = aim_target_offset_mm(map_aim[0], map_aim[1],
                                              float(flight),
                                              float(catch_pose[2]))
            if ilc_aim == (0.0, 0.0):
                # The pre-Phase-2 arithmetic, unchanged and unreached by layer 3.
                base_offset = offset
            else:
                # Reachable only when the clamp did NOT bind (a hit refuses layer
                # 3 above), so (rx, ry) IS base + ilc exactly and the split below
                # is a difference of two commanded offsets rather than an
                # approximation.
                base_offset = aim_target_offset_mm(base_rx, base_ry,
                                                   float(flight),
                                                   float(catch_pose[2]))
        except (toss_cal.TossCalError, ValueError) as exc:
            self.get_logger().error(
                f"toss aim composition FAILED for catch pose "
                f"{tuple(catch_pose)}: {exc} — this goal is thrown UNAIMED "
                f"(C-TOSS-CAL-1 is a refinement, never a gate)")
            block['applied'] = False
            block['ilc_applied'] = False
            return block
        block['aim_rad'] = (float(rx), float(ry))
        block['offset_mm'] = (float(offset[0]), float(offset[1]))
        block['map_aim_rad'] = (float(map_aim[0]), float(map_aim[1]))
        block['map_offset_mm'] = (float(map_offset[0]), float(map_offset[1]))
        block['trim_aim_rad'] = (float(trim_aim[0]), float(trim_aim[1]))
        # The trim's mm REPORT value is the DIFFERENCE of the commanded offsets,
        # not `aim_target_offset_mm(trim)` on its own: what the trim actually
        # moved is the virtual target, and after the total clamp that displacement
        # is exactly (map+trim) − map. Computing it independently would report a
        # trim the machine did not command whenever the clamp bound — and since
        # C4 that is not a "whenever" but an ALWAYS, because layer 2 commands
        # nothing, so this expression is what keeps the field a structural zero
        # rather than a number about the monitor. Layer 3 gets the same treatment
        # one step further out — total − (map+trim) — and with layer 3 inactive
        # `base_offset is offset`, so this is bit-for-bit the pre-Phase-2
        # expression.
        block['trim_offset_mm'] = (float(base_offset[0]) - float(map_offset[0]),
                                   float(base_offset[1]) - float(map_offset[1]))
        block['ilc_aim_rad'] = (float(ilc_aim[0]), float(ilc_aim[1]))
        block['ilc_spatial_aim_rad'] = (float(ilc_spatial[0]),
                                        float(ilc_spatial[1]))
        block['ilc_session_aim_rad'] = (float(ilc_session[0]),
                                        float(ilc_session[1]))
        block['ilc_offset_mm'] = (float(offset[0]) - float(base_offset[0]),
                                  float(offset[1]) - float(base_offset[1]))
        block['ilc_vel_trim'] = float(ilc_dv)
        block['clamp_hits'] = list(hits)
        return block

    @staticmethod
    def _ilc_vel_trim_refusal(nominal_mps: float, trimmed_mps: float,
                              flight_s: float) -> str:
        """``''`` to APPLY the layer-3 speed trim, or the reason to drop it.

        THE apply-seam gate for the ``event_vel_trim`` channel (contradiction
        **C2** of the 2026-08-21 ILC fold-in). Three checks, and each one is a
        different failure it closes:

        1. ``validate_event_vel`` — the bridge's ``[0.3, 7.0]`` m/s acceptance
           band. Kept, and it is NOT the important one: that band "bounds
           nothing physical" (``toss_trim.SessionTrim.speed_gain``'s ⚠ block).
        2. ``throw_envelope.evaluate(T, trimmed)`` — contract **C-HAND-3**, the
           gate that does bound the hardware, run on the speed the trim would
           actually command. Without it a trim that clears the wire band but
           breaks the envelope makes ``TossSequencer`` CHECKING mint
           ``REJECTED_THROW_ENVELOPE`` and **layer 3 becomes a gate** — the
           precise thing C-TOSS-CAL-1's "a refinement, never a gate" rule
           forbids, and the reason this seam exists at all. The admissible trim
           is strongly T-dependent (at the derived band floor T = 0.4949 s the
           negative side is exactly ``+0.000``, bounded by ``ARM_WINDOW``), so a
           trim fitted at one flight time is not automatically flyable at
           another. ``ilc_fit_lib.speed_authority_band`` is the offline half of
           the same computation; this is the online verdict.
        3. the NOMINAL must clear the envelope too. Root cause: if the untrimmed
           goal is already inadmissible, applying a trim that happens to be
           admissible would make layer 3 the difference between a rejected goal
           and a flown one — so an absent or dormant artifact would flip the
           machine's verdict on a goal, and "byte-identical with layer 3 off"
           would stop being true of the outcome. A goal the machine cannot fly
           untrimmed is refused for its own reason, with the trim dropped.

        Static and pure so it can be tested without a node.
        """
        if not validate_event_vel(trimmed_mps):
            return ('would be outside the bridge band [{:.2f}, {:.2f}] '
                    '(REJECTED_EVENT_VEL)'.format(
                        hw.TEENSY_TRAJ_MIN_EVENT_VEL_MPS,
                        hw.TEENSY_TRAJ_MAX_EVENT_VEL_MPS))
        verdict = throw_envelope.evaluate(flight_s, trimmed_mps)
        if not verdict.ok:
            return 'REJECTED_THROW_ENVELOPE({})'.format(verdict.message)
        nominal = throw_envelope.evaluate(flight_s, nominal_mps)
        if not nominal.ok:
            return ('the UNTRIMMED goal is itself outside the throw envelope '
                    '({}) — layer 3 never decides whether a goal is flyable'
                    .format(nominal.message))
        return ''

    @staticmethod
    def _release_is_tilted(release) -> bool:
        """True iff this release state carries a NON-ZERO commanded tilt.

        THE predicate the three former ``tier == TIER_8B`` branches are re-keyed
        on (design F2/D3). Tier is a proxy for the thing that actually matters —
        "is the commanded release orientation non-level?" — and the proxy breaks
        the moment an 8a toss commands an aim. Keying on the release state
        instead makes each branch say what it means, and it keeps a zero-aim
        goal on the byte-identical level path whatever its tier.
        """
        if release is None:
            return False
        return (float(getattr(release, 'tilt_rx', 0.0)) != 0.0
                or float(getattr(release, 'tilt_ry', 0.0)) != 0.0)

    def _toss_commanded_release(self):
        """The COMMANDED release state for the live cycle (aim applied), or None."""
        with self._lock:
            return self._toss_release_cmd

    def _build_toss_cycle(self, catch_pose, flight, throw_delay, vel_scale,
                          *, delay_is_cadence=False):
        """Build ONE toss cycle: resolve the tier / throw site / release state,
        construct + start the ``TossSequencer``, and install the per-goal node
        state the observation builder and the async-event routing read.

        Shared verbatim by the single ``Toss`` and by every ``TossContinuous``
        cycle — that is the point. Two copies of this would let a session drift
        from the single toss the hardware ladder validated, and the drift would
        be invisible until a sitting.

        The goal numerics are the CALLER's gate (both callers refuse
        REJECTED_BAD_GOAL before reaching here, with nothing built and nothing
        installed).

        ``delay_is_cadence`` is the ONE thing the two callers do differently, and
        it is a statement about what ``throw_delay`` MEANS to each of them:

        * a single ``Toss`` is an APPOINTMENT the operator set. Too short for
          the sequence this cycle must run ⇒ ``REJECTED_CANT_MAKE_LEAD``, loud,
          nothing moved. Silently stretching it would answer a request the
          operator did not make.
        * a ``TossContinuous`` cycle's delay is a CADENCE parameter the SESSION
          consumes — it schedules cycle N+1 as ``landing + dwell − throw_delay``
          precisely so the RELEASE lands one dwell past the landing, and the
          session's own dwell floor is sized on the chained steady state. So a
          cycle that must first COMMAND its pre-positioning move is GRANTED the
          extra lead (once, loudly) instead of killing the sitting on its first
          cycle. Only the first cycle normally pays it: a CAUGHT toss ends in
          ACTION_STAY holding the pose it threw from, so every later cycle takes
          the census-B1 skip and keeps the operator's number exactly.
        """
        # The release state (frames + ballistics) comes from the single tested
        # conversion module; the FSM gets its event_vel from here, never a second
        # derivation. An OUT-OF-BAND flight time still computes a release state
        # and is then loudly REJECTED_FLIGHT_TIME by the FSM's CHECKING pass.
        tier = str(hw.JB_OP_TOSS_TIER)
        # Tier-8b throw site A = the platform's LIVE commanded xy (Phase E). NOT a
        # config site: the ball leaves the hand where the platform IS, so any other
        # A is a phantom the aim is solved for and — because POSITIONING then
        # translates the platform to the pre-tilt pose derived from it — a
        # commanded traverse the operator never asked for. Reading it live is also
        # what makes a session CHAIN: after a CAUGHT toss the platform STAYS at B,
        # so the next goal's A is that B for free.
        #
        # Self-consistency note (why a 200 ms-old read is not a correctness bug):
        # A is NOMINATED, not observed. POSITIONING commands the platform to the
        # pre-tilt pose derived from A and CHECKING/PREPARE wait for that arrival,
        # so the platform is at A at release BY CONSTRUCTION — a stale read yields
        # a slightly different, still fully self-consistent throw site.
        #
        # UNKNOWN ⇒ throw_site_known=False and the FSM rejects POSE_UNKNOWN. There
        # is deliberately no fallback value here.
        throw_site = (0.0, 0.0)
        throw_site_known = True
        tilt_clamp_exceeded = False
        if tier == TIER_8B:
            live = self._live_commanded_position(time.perf_counter())
            if live is None:
                self.get_logger().error(
                    'Toss REJECTED_POSE_UNKNOWN: no fresh '
                    'trajectory/commanded_position — the displaced throw site A '
                    'is the live commanded pose and is never guessed '
                    '(trajectory_node publishes it only while seeded+streaming)')
                throw_site_known = False
            else:
                throw_site = (float(live[0]), float(live[1]))
        if tier == TIER_8B and throw_site_known:
            # Tier 8b: throw from A, tilt-aimed at the displaced B. The module's
            # clamp gate is the authoritative aim check — a raise maps onto the
            # FSM's tilt_clamp_exceeded flag so CHECKING mints REJECTED_TILT_CLAMP
            # in gate order (no drift-prone second copy of the aim math;
            # unreachable at the 150 mm cap — the aim needs ~320 mm of
            # displacement to hit the 12° ceiling, measured
            # tools/probes/displaced_reach_frontier.py).
            try:
                release = compute_release_state_tilted(
                    catch_pose, flight, throw_site_xy_mm=throw_site)
            except ThrowTiltInfeasible as exc:
                self.get_logger().error(f'Toss displaced aim infeasible: {exc}')
                release = None
                tilt_clamp_exceeded = True
        elif tier == TIER_8B:
            # Site unknown: there is no release state to compute (every field of
            # it is a function of A). The FSM rejects POSE_UNKNOWN on its first
            # step, before POSITIONING, so nothing downstream reads this None —
            # same shape as the tilt-clamp branch above.
            release = None
        else:
            release = compute_release_state(catch_pose, flight)
        # ── The AIM (contract C-TOSS-CAL-1, D1/D4) ──
        # ONE lookup, here, for the whole goal. `release` stays the UNCORRECTED
        # state and is what ANNOUNCE, the possession plausibility reference and
        # the record's catch point all read — D4: the announcement is the
        # prediction of where the ball goes, and after a correct aim correction
        # that is B, so keeping it uncorrected leaves the correlation→catch path,
        # the receive-tilt computation and the plausibility bound bitwise
        # unchanged. `release_cmd` is what the PLATFORM is commanded to and what
        # the hand is dispatched at.
        aim = self._toss_aim_for_goal(catch_pose, flight)
        release_cmd = release
        if release is not None and aim['aim_rad'] != (0.0, 0.0):
            # The aim rides the existing, test-pinned tilted path via a VIRTUAL
            # target displaced by the aim's own ballistic offset (D2). The throw
            # site is unchanged — 8a throws from B, 8b from the live A — so the
            # only thing the virtual target moves is the commanded aim.
            virtual_b = (float(catch_pose[0]) + aim['offset_mm'][0],
                         float(catch_pose[1]) + aim['offset_mm'][1],
                         float(catch_pose[2]))
            aim_site = (throw_site if tier == TIER_8B
                        else (float(catch_pose[0]), float(catch_pose[1])))
            try:
                release_cmd = compute_release_state_tilted(
                    virtual_b, flight, throw_site_xy_mm=aim_site)
            except ThrowTiltInfeasible as exc:
                # Only reachable when a displaced 8b goal was already near the
                # 12° ceiling and the ≤1° aim pushed it over. Same terminal as
                # an un-aimed infeasible aim — REJECTED_TILT_CLAMP, loudly —
                # never a silently clamped, mis-aimed throw.
                self.get_logger().error(
                    f'Toss aim-corrected displaced aim infeasible: {exc}')
                release = None
                release_cmd = None
                tilt_clamp_exceeded = True
        # The COMMANDED launch speed: the aim tilts the launch, so |v| grows by
        # 1/cos(aim) (0.015 % at 1°). The FSM still gates it through
        # validate_event_vel, so an aim can no more push event_vel past the
        # bridge's [0.3, 7.0] m/s than a displacement can.
        event_vel = (float(release_cmd.event_vel_mps)
                     if release_cmd is not None else 0.0)
        # ── layer 3's SECOND channel: the learned event_vel trim ──
        # `k_v - 1`, applied as the multiply `release_state_for_command` models
        # (ilc_fit_lib) — the fit and the machine must scale the same magnitude
        # in the same place or the sensitivity F describes a different command
        # than the one that flies. Guarded on `!= 0.0` so an inactive layer 3
        # costs not one floating-point operation and `event_vel` stays the exact
        # float the pre-Phase-2 expression produced.
        if event_vel and aim['ilc_vel_trim'] != 0.0:
            trimmed = event_vel * (1.0 + float(aim['ilc_vel_trim']))
            refusal = self._ilc_vel_trim_refusal(event_vel, trimmed, flight)
            if not refusal:
                event_vel = trimmed
            else:
                # REFUSE the trim, fly the nominal. The FSM would otherwise mint
                # REJECTED_EVENT_VEL / REJECTED_THROW_ENVELOPE and the whole goal
                # would die for a refinement — and layer 3, like C-TOSS-CAL-1, is
                # a refinement and never a gate. The record then carries
                # ilc_vel_trim = 0.0, which is the truth about what was commanded
                # (risk 5).
                self.get_logger().warning(
                    'ILC event_vel trim {:+.4f} REFUSED at apply: it would '
                    'command {:.4f} m/s and {}. This goal throws at the '
                    'untrimmed {:.4f} m/s.'.format(
                        float(aim['ilc_vel_trim']), trimmed, refusal, event_vel))
                aim['ilc_vel_trim'] = 0.0
                aim['ilc_refused'] = (
                    '{},event_vel'.format(aim['ilc_refused'])
                    if aim['ilc_refused'] else 'event_vel')
        # ── THE positioning decision, taken ONCE (2026-08-23) ──
        # It used to be taken inside _position_platform_for_toss, one FSM tick
        # from now. It is taken here because it is now TWO things at once:
        #
        #   * the branch POSITIONING takes (skip vs command a go_to_pose), and
        #   * the pre-dispatch budget the CHECKING delay gate charges
        #     (toss_sequencer.pre_dispatch_budget_s — 0.080 s for the skip,
        #     0.460 s for a move).
        #
        # Taking it twice would let those two disagree, and a budget charged for
        # a branch the node did not take is exactly the accept-vs-runtime gap the
        # 2026-08-22 audit found. Cached under the lock below; the POSITIONING
        # step consumes the cached answer and re-reads nothing.
        #
        # `release_cmd is None` (a tilt-clamp raise, or an unknown 8b site) means
        # the FSM rejects on its first step and POSITIONING is never reached — the
        # fail-closed True there is moot but costs nothing.
        px, py, pz = self._toss_positioning_xyz(catch_pose, release_cmd)
        positioning_move = not (
            release_cmd is not None
            and self._toss_already_positioned(px, py, pz, release_cmd))
        # ── and the lead that decision needs (SESSION cycles only) ──
        # See the `delay_is_cadence` paragraph in the docstring for why this is
        # a grant here and a refusal on the single-Toss path. In one line: a
        # cadence parameter is a MINIMUM lead the session's own scheduler
        # consumes, an appointment is not.
        #
        # ⚠ STRICTLY POSITIVE delays only. 0.0 is the goal's "unset" sentinel and
        # TossSequencer.__post_init__ substitutes the 5.0 s default for it;
        # a NEGATIVE value is the sign typo __post_init__ deliberately preserves
        # so CHECKING rejects it loudly. Raising either would launder an operator
        # error into a number the machine chose.
        cycle_delay = float(throw_delay)
        lead_floor = min_throw_delay_for_release_s(event_vel, positioning_move)
        if (delay_is_cadence and positioning_move
                and 0.0 < cycle_delay < lead_floor):
            self.get_logger().warning(
                'toss throw_delay raised {:.3f} -> {:.3f} s for this cycle: '
                'POSITIONING must COMMAND the pre-positioning move (the platform '
                'is not already at ({:.1f}, {:.1f}, {:.1f}) mm within {:.1f} mm, '
                'at the commanded orientation), which spends {:.3f} s of the '
                'lead before the release-window guard runs. A chained cycle takes '
                'the census-B1 skip and keeps the {:.3f} s you asked for.'.format(
                    cycle_delay, lead_floor, px, py, pz,
                    _TOSS_ALREADY_THERE_TOL_MM,
                    pre_dispatch_budget_s(True), cycle_delay))
            cycle_delay = lead_floor
        seq = TossSequencer(
            catch_pose_stow_mm=catch_pose, flight_time_s=flight,
            throw_delay_s=cycle_delay, tier=tier,
            event_vel_mps=event_vel,
            throw_site_xy_mm=throw_site,
            throw_site_known=throw_site_known,
            positioning_move_expected=positioning_move,
            max_displacement_mm=float(hw.JB_OP_TOSS_MAX_DISPLACEMENT_MM),
            workspace_xy_mm=float(hw.JB_OP_TOSS_WORKSPACE_XY_MM),
            stay_at_pose_on_caught=bool(hw.JB_OP_TOSS_STAY_AT_POSE_ON_CAUGHT),
            tilt_clamp_exceeded=tilt_clamp_exceeded)
        seq.start(time.perf_counter())
        waiver = bool(self.get_parameter(_TOSS_WAIVER_PARAM).value)
        if waiver:
            # One WARN per goal: the waiver is a bench/trace affordance only.
            self.get_logger().warning(
                f'{_TOSS_WAIVER_PARAM} is SET — waiving the ball-possession '
                'precondition (hand-parked/hand-fresh stay hard). Trace use '
                'only; never run powered sessions with this set.')
        with self._lock:
            self._active_seq = seq
            # Roll the outgoing latch forward BEFORE clearing it (census D6):
            # cycle N+1's CHECKING must be able to tell OUR just-caught ball —
            # whose track is still IN_FLIGHT until the tracker mints CAUGHT —
            # from a genuine phantom. Cleared at goal ACCEPT, so it spans cycles
            # and never goals.
            self._prev_announced_ball_id = self._announced_ball_id
            self._announced_ball_id = None
            self._announced_id_untagged = False
            # A new cycle inherits no schedule: the session sets it immediately
            # after this returns, and a single Toss never does.
            self._toss_next_release_perf = None
            self._toss_next_landing_perf = None
            # GOAL-START phantom snapshot: any id already IN_FLIGHT before OUR
            # throw is a phantom BY CONSTRUCTION (our ball cannot be airborne
            # before our announcement's throw_time). The PREPARE-tick snapshot
            # later only ADDS ids that appear during positioning/prepare — it
            # never clears this baseline.
            self._preexisting_flight_ids = {
                int(b.id) for b in self._balls
                if int(b.status) == _BALL_STATUS_IN_FLIGHT}
            self._catch_vel_scale = (vel_scale if vel_scale > 0.0
                                     else float(hw.JB_OP_CATCH_VEL_SCALE_DEFAULT))
            self._toss_release_state = release
            self._toss_release_cmd = release_cmd
            self._toss_aim = aim
            # THE cached positioning decision (2026-08-23). Written here, read
            # exactly once by _position_platform_for_toss on the next FSM tick.
            # It is not an optimisation: the CHECKING delay gate has ALREADY
            # charged a pre-dispatch budget keyed on this boolean, so re-deriving
            # it a tick later — against a commanded pose that may have been
            # republished in between — could take the cheap branch under an
            # expensive budget, or the reverse. One decision, one place.
            self._toss_positioning_move = positioning_move
            # release is None on a Tier-8b tilt-clamp raise (the FSM rejects
            # REJECTED_TILT_CLAMP on the first step, so the nominated landing is
            # never consulted); guard the deref. None ⇒ the possession
            # plausibility falls back to the ACTIVE catch point (the reload
            # default), which is moot for an immediately-rejected goal.
            self._toss_landing_global_mm = (
                tuple(float(v) for v in release.catch_point_global_mm)
                if release is not None else None)
            # The POSITIONING mocap arrival cross-check target, kept STOW-relative
            # and UNCONVERTED (the cross-check compares in the platform_start
            # frame — the frame non-base bodies publish in, see
            # _TOSS_MOCAP_BODY_PARAM; converting to global here double-adds
            # GEOM_INITIAL_HEIGHT_MM). It is the SAME _toss_positioning_xyz the
            # go_to_pose command uses (single source, so command and verification
            # can never diverge), fed the COMMANDED release: a level goal = the
            # nominated catch pose B, a tilted one = the swing-compensated
            # pre-tilt pose at the throw site (else an operator who configures a
            # platform body for a tilted sitting gets measured-A vs target-B → a
            # spurious ABORTED_POSITION_FAILED).
            self._toss_platform_target_mm = self._toss_positioning_xyz(
                catch_pose, release_cmd)
            self._toss_waiver = waiver
            self._toss_mocap_body = str(
                self.get_parameter(_TOSS_MOCAP_BODY_PARAM).value or '')
            self._platform_pos_mm = None   # never trust a stale/other-body cache
            self._toss_prepare_pending = False
            self._toss_throw_dispatched = False
            self._toss_stroke_seen = False
            self._toss_track_confirmed = False
            # Raised iff the COMMANDED release carries a tilt (a level goal
            # never touches catch/pretilt_hold).
            self._toss_pretilt_hold_raised = False
            self._toss_announced_reach = None
        self._possession_logged = set()
        return seq

    def _clear_toss_cycle_state(self) -> None:
        """Tear down ONE cycle's per-goal node state. Deliberately does NOT touch
        ``_goal_claimed`` (that spans a whole session) nor ``_ball_possession``
        (the latch survives across cycles — a caught ball is still in the cup)."""
        with self._lock:
            self._active_seq = None
            # Per-goal toss state down; the possession plausibility reference
            # reverts to the ACTIVE catch point between goals.
            self._toss_release_state = None
            self._toss_release_cmd = None
            self._toss_aim = None
            self._toss_landing_global_mm = None
            self._toss_platform_target_mm = None
            self._toss_prepare_pending = False
            self._toss_throw_dispatched = False
            # Fail-closed between cycles: a leftover False would let the NEXT
            # cycle's POSITIONING skip a move on evidence that belonged to the
            # previous one.
            self._toss_positioning_move = True
            # The cadence clamp is per-CYCLE (C-POSSESS-1 § 3.4): a schedule that
            # outlived its cycle would clamp the next ball's windows against a
            # stale instant, which is the arm/disarm lifecycle bug class the
            # contract deletes rather than guards.
            self._toss_next_release_perf = None
            self._toss_next_landing_perf = None

    def _run_toss_cycle(self, seq, *, deadline_s, cancel_now_fn, feedback_fn):
        """Tick ONE toss cycle to a terminal. Returns ``(TossResult, exit_kind)``
        with ``exit_kind`` in {'fsm', 'cancel', 'timeout', 'shutdown'}.

        Shared verbatim by the single ``Toss`` and by every ``TossContinuous``
        cycle. It owns the safing on every NODE-level exit and emits the one
        authoritative outcome line per cycle; it does NOT touch the goal handle's
        terminal (succeed/abort/canceled) — that belongs to the caller, because a
        session's cycle terminal is not the session's terminal.

        ``cancel_now_fn(now) -> bool`` is the caller's per-phase cancel policy
        (``_toss_cancel_deferred`` for both callers today): True means honour the
        cancel NOW; a deferred cancel simply keeps ticking and resolves at the
        FSM's own terminal, because aborting a catch mid-flight drops the ball on
        the robot."""
        t_start = time.perf_counter()
        try:
            while rclpy.ok():
                now = time.perf_counter()
                if cancel_now_fn(now):
                    self._safe_toss_on_early_exit(seq)
                    r = TossResult(False, 'ABORTED_CANCELLED')
                    self._log_toss_outcome(r)
                    return r, 'cancel'
                decision = self._step_toss_sequence(seq, now, None)
                if decision.done:
                    r = decision.result
                    self._log_toss_outcome(r)
                    return r, 'fsm'
                if feedback_fn is not None:
                    feedback_fn(decision.phase)
                if now - t_start > deadline_s:
                    self._safe_toss_on_early_exit(seq)
                    r = TossResult(False, 'ABORTED_TIMEOUT')
                    self._log_toss_outcome(r)
                    return r, 'timeout'
                time.sleep(_TICK_S)
            # rclpy shutting down.
            self._safe_toss_on_early_exit(seq)
            r = TossResult(False, 'ABORTED_SHUTDOWN')
            self._log_toss_outcome(r)
            return r, 'shutdown'
        except Exception:
            # Unexpected fault mid-sequence (the loop body raised): the robot may
            # be positioned / latched / stroke-armed, so SAFE FIRST, then emit
            # the one authoritative outcome line and re-raise so the caller (and
            # the executor's own error path) still sees the fault.
            self._safe_toss_on_early_exit(seq)
            self._log_toss_outcome(TossResult(False, 'ABORTED_EXCEPTION'))
            raise

    @staticmethod
    def _invalid_toss_goal_field(catch_pose, throw_height, throw_delay, vel_scale):
        """Return the name of the first invalid Toss goal numeric, or None.

        Non-finite ANY of the six numerics ⇒ invalid (the ball_butler_node
        guard pattern: NaN/inf flows through ballistics into NaN commands).
        NEGATIVE throw_height_m / throw_delay_s / catch_vel_scale ⇒ invalid: 0.0
        is the only "use the default" sentinel, and coercing a sign typo to a
        default would silently run a physically different toss (the
        catch_position components are legitimately signed — the workspace
        pre-check and the feasibility gate own their bounds)."""
        finite_checks = (
            ('catch_position.x', catch_pose[0]),
            ('catch_position.y', catch_pose[1]),
            ('catch_position.z', catch_pose[2]),
            ('throw_height_m', throw_height),
            ('throw_delay_s', throw_delay),
            ('catch_vel_scale', vel_scale),
        )
        for name, value in finite_checks:
            if not math.isfinite(value):
                return name
        for name, value in (('throw_height_m', throw_height),
                            ('throw_delay_s', throw_delay),
                            ('catch_vel_scale', vel_scale)):
            if value < 0.0:
                return name
        return None

    def _toss_cancel_deferred(self, seq, now: float) -> bool:
        """Per-phase cancellation (plan § Choreography 7 — a DELIBERATE deviation
        from reload's immediate-cancel, which mid-flight retracts under an airborne
        ball). True ⇒ defer: keep ticking the FSM to its own terminal and resolve
        the cancel there with the real outcome.

          - CHECKING / POSITIONING / PREPARING — honour now (nothing airborne; the
            early-exit safing retracts + lowers the latch iff anything armed);
          - THROWING before t_release − TOSS_CANCEL_CUTOFF_S — honour now (the
            SAFE_ABORT retract clears the armed stroke on the last-writer-wins
            queue; the ball is still seated);
          - THROWING past the cutoff, and every later phase — DEFER (the stroke is
            about to fire or the ball is in the air; aborting a catch mid-flight
            drops a ball on the robot)."""
        phase = seq.phase
        if phase in (TOSS_PHASE_CHECKING, TOSS_PHASE_POSITIONING,
                     TOSS_PHASE_PREPARING):
            return False
        if (phase == TOSS_PHASE_THROWING
                and now < seq.t_release - TOSS_CANCEL_CUTOFF_S):
            return False
        return True

    def _step_toss_sequence(self, seq, now, goal_handle=None):
        """One toss FSM tick: build observations, step, execute the requested
        action, publish phase feedback. Returns the decision (testable in
        isolation) — the toss sibling of :meth:`_step_sequence`.

        ACTION_PREPARE_CATCH is split across two ticks: the verified-arrival
        tick publishes catch/prime_hold ALONE, and the bundle runs on the NEXT
        tick — the hold and catch/armed travel on different topics with no
        cross-topic ordering guarantee, and callbacks landing in the SAME
        catch_coordinator wait-set cycle have no intra-cycle ordering either,
        so a same-tick hold could lose to the armed edge and let the
        armed-edge auto-prime ascend with the seated ball. One full FSM tick
        puts the hold in an earlier CCN cycle.

        **That tick is ``_TICK_S`` = 20 ms, not the 50 ms this docstring claimed
        until 2026-08-22** (audit fix; the B3 cadence work cut it in c938c1d and
        left both cross-topic ordering arguments quoting the old number). Still
        two orders of magnitude above localhost topic latency, so the argument
        holds — but a future session tightening cadence reads the margin HERE,
        and reading 50 ms would credit it with 2.5x more headroom than it has
        before collapsing this split into one tick."""
        obs = self._build_toss_observations(now)
        decision = seq.step(now, obs)
        if decision.action == TOSS_ACTION_POSITION_PLATFORM:
            self._position_platform_for_toss(seq)
        elif decision.action == TOSS_ACTION_PREPARE_CATCH:
            # Verified-arrival tick: raise the hold, defer the bundle (see the
            # docstring). The FSM idles in PREPARING until the bundle's
            # note_prepare_result arrives next tick. Tier 8b ALSO raises
            # catch/pretilt_hold here (alongside prime_hold, >=2 FSM ticks
            # before our announcement can reach catch_coordinator) so the stock
            # announcement pre-tilt never fires — this node owns the deferred
            # A->B reach at t_release. For 8a the topic is NEVER touched (the
            # publish sequence stays byte-identical).
            self._publish_prime_hold(True)
            # Declare the reach-envelope centre (contract C-REACH-1) on THIS tick,
            # so it lands in an earlier trajectory_node callback than the
            # arm_catch raise the bundle makes one full FSM tick later. The
            # declared centre is the goal's NOMINATED catch pose B, not the
            # swing-compensated target the reach will actually carry: B is the
            # operator-nominated quantity, the swing shift is a few mm, and 80 mm
            # of envelope absorbs it with room for the tracker refinements the
            # envelope exists to bound.
            #
            # Declared for BOTH tiers. For 8a B IS the held pose, so the centre
            # equals the fallback and the behaviour is unchanged — but declaring
            # it means the envelope no longer silently depends on POSITIONING
            # having actually arrived.
            self._publish_reach_center(seq.catch_pose_stow_mm)
            # UNCONDITIONAL since 2026-08-22 (census E5). The key has widened
            # twice, and the direction is the same each time — toward "the toss
            # owns the platform for the whole cycle, full stop":
            #
            #   tier == TIER_8B          (Phase 4)
            #   -> any non-zero commanded tilt, tier-independent (D3): the aim
            #      map gave 8a a second source of tilt, and without the hold
            #      catch_coordinator._on_throw_announcement's stock pre-tilt
            #      completes the un-tilt to level >= 1 s BEFORE release, so an
            #      aimed 8a toss is levelled back while every log line still
            #      reports the aim as applied;
            #   -> ALWAYS (E5).
            #
            # Why the last step. CCN's pre-tilt arrival is
            # min(landing, max(landing - 1.5, now + 1.0)). At the cadence rungs
            # `landing - 1.5` is in the PAST, so the max() picks `now + 1.0`,
            # and the min() then clamps it to the landing itself — an arrival
            # scheduled AT CONTACT, which is the exact degeneracy
            # _pretilt_arrival_perf's own docstring was written to avoid (third
            # sitting: tilt still >1 deg off until 0.24-0.49 s before landing).
            #
            # For a LEVEL 8a that target is the already-held pose, so the motion
            # is degenerate and the old key was defensible. It stopped being
            # defensible for two reasons that both arrive with cadence: the
            # target is `predicted_catch_command`'s output, not literally the
            # held pose, so any receive-tilt or landing-prediction residual
            # becomes a commanded reach arriving AT CONTACT; and at a sub-second
            # dwell the NEXT cycle's announcement lands while the previous ball
            # is still seating, i.e. inside E1/E2's settle-hold and reach-freeze
            # window, where any platform motion is a dropped catch.
            #
            # The cost of raising it when it was not needed is bounded and
            # documented: a stale pretilt_hold only DEGRADES (a later reload
            # catch loses its pre-tilt), it never commands anything, and all
            # three teardowns release it keyed on the flag below. The cost of
            # NOT raising it is a platform reach under a seated ball.
            self._publish_pretilt_hold(True)
            self._toss_pretilt_hold_raised = True
            self._toss_prepare_pending = True
        elif decision.action == TOSS_ACTION_ANNOUNCE:
            self._announce_toss(seq)
        elif decision.action == TOSS_ACTION_DISPATCH_THROW:
            outcome, message = self._dispatch_toss_throw(seq)
            seq.note_throw_dispatch(outcome, message)
        elif decision.action == TOSS_ACTION_REACH_CATCH:
            # Tier 8b's deferred A->B reach (time-triggered at t_release): the
            # ONE announcement-derived catch/dynamic_target for B (lead = the
            # flight time by construction). 8a never emits this action.
            self._publish_toss_reach()
        elif decision.action == TOSS_ACTION_STAY:
            self._toss_stay()
        elif decision.action == TOSS_ACTION_RECENTER:
            self._toss_recenter()
        elif decision.action == TOSS_ACTION_SAFE_ABORT:
            self._toss_safe_abort()
        elif self._toss_prepare_pending and not decision.done:
            # The tick after the hold went out: run the PREPARE bundle. A
            # terminal decision on this tick (mode change, cancel-driven abort)
            # takes the elif branches above instead — the pending bundle is
            # simply dropped (per-goal state, reset at goal init).
            self._toss_prepare_pending = False
            seq.note_prepare_result(self._prepare_toss_catch())
        if (decision.done and decision.result is not None
                and decision.result.outcome in _TOSS_POSITION_UNKNOWN_TERMINALS):
            # Position-unknown terminals carry ACTION_NONE from the FSM, but an
            # ack-timed-out go_to_pose may still be executing — dispatch a
            # best-effort go_home to supersede any zombie move (see
            # _TOSS_POSITION_UNKNOWN_TERMINALS).
            self._go_home()
        if goal_handle is not None and not decision.done:
            fb = Toss.Feedback()
            fb.phase = decision.phase
            goal_handle.publish_feedback(fb)
        return decision

    def _position_platform_for_toss(self, seq) -> None:
        """POSITIONING: the profiled go_to_pose to the pre-positioning pose.

        Tier 8a throws from a LEVEL platform at the nominated catch pose B
        (identity orientation). Tier 8b throws from the swing-compensated PRE-TILT
        pose at the throw site A: the POSITION is release.pretilt_pose_stow[:3]
        (A, swing-compensated so the tilted release xy sits at nominal A) and the
        ORIENTATION is the throw tilt (tilt_rx, tilt_ry, rz=0). Sourcing the level
        B pose for 8b (as the shared path once did) would pre-position LEVEL at B
        and throw straight up — orphaning the computed pre-tilt aim and firing from
        the wrong site. The reach envelope the deferred A->B target is judged
        against is the DECLARED catch centre B (contract C-REACH-1; the node
        publishes catch/reach_center one FSM tick before the arm raise), so the
        displacement is bounded by toss_max_displacement_mm rather than by the
        envelope. The 8b tilt is encoded through the SAME
        quaternion round-trip go_to_pose decodes with (rotvec -> matrix -> quat),
        single-sourcing the ik_solver helpers rather than hand-rolling a quaternion.

        The response is fed straight into the FSM: the service returns at
        plan-INSTALL, so planned_duration_s anchors the timed arrival; the
        config-keyed mocap cross-check (disabled by default — see
        _TOSS_MOCAP_BODY_PARAM) can then corroborate before any arming. The
        '[wire DISARMED' marker on an otherwise-accepted response is mapped to a
        WIRE_DISARMED reject — the plan-installed-but-not-actuating case
        (string-fragile, pinned by test).

        NO_RESPONSE (service unavailable, or the ack future timing out) leaves
        the platform's true motion state UNKNOWN: the plan may have installed
        and be executing despite the missing ack. The resulting
        REJECTED_POSITION(NO_RESPONSE) terminal (and the never-noted
        ABORTED_POSITION_TIMEOUT) therefore gets a best-effort go_home from
        _step_toss_sequence — go_home supersedes any zombie move by design
        (see _TOSS_POSITION_UNKNOWN_TERMINALS)."""
        release = self._toss_commanded_release()
        # The STOW-frame POSITION is the SINGLE source shared with the POSITIONING
        # mocap arrival cross-check target (_toss_platform_target_mm, set in
        # _build_toss_cycle from the SAME _toss_positioning_xyz), so what we command
        # and what we verify against can never diverge: a level goal = B, a tilted
        # one = the swing-compensated pre-tilt pose at the throw site.
        x, y, z = self._toss_positioning_xyz(seq.catch_pose_stow_mm, release)
        if self._release_is_tilted(release):
            # A tilted release — Tier 8b's displaced aim, or a Tier-8a aim from
            # the calibration map, or both. `release` is a TiltedReleaseState
            # post-CHECKING (a tilt-clamp raise is rejected on the first FSM step,
            # before POSITIONING is ever reached) — the unguarded read mirrors
            # _announce_toss's read of the same stash.
            orientation = self._tilt_quaternion(release.tilt_rx, release.tilt_ry)
        else:
            orientation = Quaternion()              # identity = level platform
        # THE cached decision from _build_toss_cycle, not a fresh evaluation
        # (2026-08-23). The CHECKING delay gate has already charged a
        # pre-dispatch budget keyed on this exact boolean; re-deriving it here
        # would let the branch taken and the budget charged disagree.
        with self._lock:
            already_positioned = not bool(self._toss_positioning_move)
        if already_positioned:
            # CENSUS B1 — the no-op move. For a chain with
            # stay_at_pose_on_caught the platform is ALREADY at the
            # pre-positioning pose: the catch ended in ACTION_STAY and the
            # emitter's terminal hold never let it go. Commanding it anyway costs
            # min_move_duration_s (0.20) + TOSS_POSITION_SETTLE_PAD_S (0.20) + a
            # service round trip, EVERY cycle, to traverse zero millimetres —
            # 0.45 s of a turnaround whose whole physical floor is 0.49 s. Since
            # 2026-08-23 that holds for an AIMED chain too: the skip verifies the
            # commanded ORIENTATION as well, so it no longer refuses every tilted
            # release by construction.
            #
            # Nothing else is skipped: the FSM still runs POSITIONING, the
            # reach-envelope declaration still goes out at PREPARE (C-REACH-1),
            # and the mocap arrival cross-check still gets its verification
            # window. See TossSequencer.note_position_noop.
            self.get_logger().info(
                'POSITIONING skipped — platform is already at '
                '({:.1f}, {:.1f}, {:.1f}) mm within {:.1f} mm (census B1); '
                'nothing commanded'.format(x, y, z, _TOSS_ALREADY_THERE_TOL_MM))
            seq.note_position_noop(time.perf_counter())
            return
        if not self._go_to_pose_cli.wait_for_service(timeout_sec=_SERVICE_WAIT_S):
            self.get_logger().error('trajectory/go_to_pose service unavailable')
            seq.note_position_result(time.perf_counter(), False, 0.0, 'NO_RESPONSE')
            return
        req = GoToPose.Request()
        req.pose = Pose(position=Point(x=float(x), y=float(y), z=float(z)),
                        orientation=orientation)
        req.duration_s = 0.0                        # minimal feasible duration
        # lean_gain stays the srv field default (-1.0): defer to the config gain.
        resp = self._wait_future(self._go_to_pose_cli.call_async(req))
        now = time.perf_counter()
        if resp is None:
            seq.note_position_result(now, False, 0.0, 'NO_RESPONSE')
            return
        if '[wire DISARMED' in str(resp.message):
            seq.note_position_result(now, False, 0.0, 'WIRE_DISARMED')
            return
        seq.note_position_result(now, bool(resp.accepted),
                                 float(resp.planned_duration_s), str(resp.code))

    def _toss_already_positioned(self, x, y, z, release) -> bool:
        """Is the platform ALREADY at the positioning pose, provably? (census B1)

        Two conditions, BOTH required, and each is a way this can say "no" —
        which is the safe answer, because a wrong "yes" fires the throw from a
        pose the aim was not solved for:

        1. **The live commanded POSE must be FRESH.** ``None`` from
           :meth:`_live_commanded_pose` means unknown or stale, and unknown reads
           as NOT-there. Same doctrine as ``throw_site_known``: a node that was
           never told is not entitled to assume. A trajectory_node that predates
           ``trajectory/commanded_pose`` therefore leaves the skip permanently
           off — cadence lost, nothing unsafe.
        2. **Position AND orientation must match, per component, inside their
           tolerances** — ``_TOSS_ALREADY_THERE_TOL_MM`` and
           ``_TOSS_ALREADY_THERE_TOL_RAD``, the second derived from the first
           (see the constant).

        **The third condition, retired 2026-08-23: "the release must be LEVEL".**
        It was there because ``trajectory/commanded_position`` published position
        and nothing else, so a TILTED pre-tilt pose carried an orientation this
        node could not verify and had to command. Correct, and expensive: an
        armed aim — ILC layer 3, the calibration map, Tier 8b — makes EVERY
        release tilted, so the skip was off for the whole of every ILC sitting
        and each cycle paid ``min_move_duration_s`` + ``TOSS_POSITION_SETTLE_PAD_S``
        + ticks (0.38 s of throw delay, and one-for-one the same in dwell) to
        traverse zero millimetres and re-command a tilt it was already holding.
        That was the largest single cadence item on the 2026-08-22 audit's board.

        The fix was to publish what the check needed rather than to weaken the
        check: ``trajectory/commanded_pose`` carries the same sampled plan state
        WITH its orientation, in the INTENT frame, so the comparison is against
        the pose this node is about to request rather than against a
        gravity-corrected version of it. Nothing here got looser — a tilted
        release now has to PROVE the platform is at its tilt, where before it
        could not be asked.

        **Why this fires on a chain, and only on a chain.** A CAUGHT toss ends in
        ``ACTION_STAY``: nothing is commanded, and the emitter's terminal hold
        leaves the platform exactly where the cycle threw and caught from. The
        next cycle recomputes its pre-tilt pose from the same ``(catch_pose,
        flight, aim)``, so the target is bit-identical and the residual is
        floating-point noise. Anything that MOVED the platform in between — a
        MISS's ``go_home``, a reload interlude's recentre, a SpaceMouse nudge, a
        reload catch's own pre-tilt at the same position — fails one component or
        the other and the move is commanded.

        **Why the tolerance is what it is.** For a co-located Tier-8a toss the
        throw site and the catch site are the same physical place, so a residual
        δ is aim-NEUTRAL: the ball goes up and comes back down into the cup
        wherever the cup is (the vertical ballistic inverse is translation
        invariant, and Δz is a hand-frame offset, not a platform-z one). What δ
        is NOT neutral about is everything downstream that was told the ball will
        be at the NOMINATED B: the declared reach envelope (C-REACH-1), the
        announcement the tracker correlates against, and the mocap cross-check
        target. So the bound is a fraction of the cup, not a fraction of the
        workspace: ``GEOM_HAND_RADIUS_MM / 2``, which is a shift the cup absorbs
        with 2x margin and which the catch pipeline's own B-centred expectation
        still covers. The ANGULAR bound is the tilt whose landing drift equals
        that same budget at the tallest admitted throw.

        It is DELIBERATELY not ``_RELOAD_CENTERED_TOL_MM`` (66.53 mm): that is a
        recentre-verification bound, an order of magnitude looser, and reusing it
        here would let the skip fire on a platform two cup-radii off B."""
        live = self._live_commanded_pose(time.perf_counter())
        if live is None:
            return False
        if any(abs(float(live[i]) - float(v)) > _TOSS_ALREADY_THERE_TOL_MM
               for i, v in enumerate((x, y, z))):
            return False
        # The pre-tilt ORIENTATION this cycle would command, as a rotvec — the
        # SAME (tilt_rx, tilt_ry, 0) that _position_platform_for_toss encodes
        # into the go_to_pose quaternion, compared before that encoding rather
        # than after it (quaternions double-cover: q and -q are one rotation, so
        # a component-wise compare of the encoded form can read a match as a
        # mismatch).
        if self._release_is_tilted(release):
            target = (float(release.tilt_rx), float(release.tilt_ry), 0.0)
        else:
            target = (0.0, 0.0, 0.0)
        return all(abs(float(live[3 + i]) - v) <= _TOSS_ALREADY_THERE_TOL_RAD
                   for i, v in enumerate(target))

    @classmethod
    def _toss_positioning_xyz(cls, catch_pose_stow_mm, release):
        """The STOW-frame platform POSITION the toss pre-positions to — the SINGLE
        source for BOTH the go_to_pose command (:meth:`_position_platform_for_toss`)
        and the POSITIONING mocap arrival cross-check target
        (``_toss_platform_target_mm``, set in :meth:`_build_toss_cycle`), so the
        commanded pose and the verification target can never diverge.

        Keyed on the COMMANDED release, not the tier (design F2): a LEVEL release
        pre-positions to the nominated catch pose B; a TILTED one (Tier 8b's
        displaced aim, a Tier-8a aim from the calibration map, or both)
        pre-positions to the swing-compensated pre-tilt pose at the throw site
        (``release.pretilt_pose_stow[:3]``). For 8a the throw site IS B, so that
        is a pre-tilt IN PLACE offset by the cup swing (≤ 1.13 mm at the 1° aim
        authority) — which is what puts the CUP on B at both release and catch.

        ``release`` is None only when the goal is rejected on the FSM's first step
        (tilt clamp, or POSE_UNKNOWN) and POSITIONING is never reached, so the B
        fallback there is moot. Returns a float (x, y, z)."""
        if cls._release_is_tilted(release):
            pre = np.asarray(release.pretilt_pose_stow, dtype=float)
            return (float(pre[0]), float(pre[1]), float(pre[2]))
        x, y, z = catch_pose_stow_mm
        return (float(x), float(y), float(z))

    @staticmethod
    def _tilt_quaternion(rx: float, ry: float) -> Quaternion:
        """Encode a platform tilt rotvec (rx, ry, rz=0) as the go_to_pose
        orientation quaternion. trajectory_node decodes the request orientation
        back to a rotvec via quat_to_rot_matrix -> rot_matrix_to_rotvec, so the
        tilt is encoded through the EXACT inverse round-trip
        (rotvec -> rotvec_to_rot_matrix -> rot_matrix_to_quat), single-sourcing
        the ik_solver helpers rather than hand-rolling a quaternion. Returns a
        geometry_msgs/Quaternion (w, x, y, z)."""
        w, qx, qy, qz = rot_matrix_to_quat(
            rotvec_to_rot_matrix(np.array([float(rx), float(ry), 0.0])))
        return Quaternion(w=float(w), x=float(qx), y=float(qy), z=float(qz))

    def _prepare_toss_catch(self) -> bool:
        """The PREPARE bundle (verified at the nominated pose, ball seated,
        BEFORE anything is committed at the hand). catch/prime_hold is NOT in
        this bundle: the node already raised it on the PREVIOUS FSM tick (the
        ACTION_PREPARE_CATCH tick — see _step_toss_sequence) so the hold lands
        in an earlier catch_coordinator wait-set cycle than the armed edge
        below. In-bundle order is load-bearing:

          1. soft catch gains (best-effort) — the standing prime_hold suppresses
             the auto-prime that normally sets them (see _TOSS_SOFT_CATCH_GAINS);
          2. trajectory/arm_catch raise + confirm — consumes the catch/reach_center
             declaration published on the PREVIOUS tick and captures it as the
             reach-envelope centre (contract C-REACH-1; falls back to the
             commanded pose if the declaration was lost, which degrades to a
             loud mid-flight WORKSPACE reject of the A->B reach — a miss, not a
             hazard). A failure aborts with nothing announced and no throw armed;
          3. catch/vel_scale — BEFORE the armed edge, so catch_coordinator holds
             this goal's scale before any hand-arm can fire (the reload rule);
          4. catch/prime_dispatched True, ONCE, immediately before the armed
             edge — the BELT under the prime_hold contract, NOT a heartbeat: if
             the hold publish was lost or reordered, this stamp holds
             catch_coordinator's 1.2 s prime-inflight window (the
             hardware-proven suppressor) across the armed edge — the only
             auto-prime that can fire before the hold's next chance to land.
             No periodic re-stamping;
          5. catch/armed True;
          6. refresh the phantom-flight snapshot — ADD any id that went
             IN_FLIGHT since the goal-start snapshot (our announced track
             cannot be IN_FLIGHT before throw_time, so every one of them is a
             phantom); NEVER clears the goal-start baseline — and reset the
             announced-ball latch, so nothing latched against a pre-snapshot
             phantom survives into the flight.

        The self-announcement is NOT part of this bundle either: the FSM emits
        it as a separate action on a LATER tick (the ≥1-tick gap — catch/armed
        and throw_announcements have no cross-topic ordering guarantee, and
        catch_coordinator drops announcement pre-tilts that arrive unarmed)."""
        self._set_soft_catch_gains()
        if not self._arm_catch(True):
            self.get_logger().error(
                'TOSS PREPARE failed: trajectory/arm_catch raise did not succeed '
                '— aborting before anything is announced or armed at the hand')
            return False
        with self._lock:
            scale = self._catch_vel_scale
        self._vel_scale_pub.publish(Float64(data=float(scale)))
        self._prime_dispatched_pub.publish(Bool(data=True))
        self._publish_catch_armed(True)
        with self._lock:
            self._preexisting_flight_ids |= {
                int(b.id) for b in self._balls
                if int(b.status) == _BALL_STATUS_IN_FLIGHT}
            self._announced_ball_id = None
            self._announced_id_untagged = False
        return True

    def _set_soft_catch_gains(self) -> bool:
        """Best-effort soft catch gains (step 1 of PREPARE): a failure is loud but
        NON-fatal — the toss proceeds on the ambient gains (post-reload they are
        already the soft set, making this idempotent in the normal chain)."""
        if not self._hand_gains_cli.wait_for_service(timeout_sec=_SERVICE_WAIT_S):
            self.get_logger().warning(
                'set_hand_gains unavailable — tossing on the ambient hand gains')
            return False
        req = SetHandGains.Request()
        req.pos_gain = float(_TOSS_SOFT_CATCH_GAINS['pos_gain'])
        req.vel_gain = float(_TOSS_SOFT_CATCH_GAINS['vel_gain'])
        req.vel_integrator_gain = float(
            _TOSS_SOFT_CATCH_GAINS['vel_integrator_gain'])
        resp = self._wait_future(self._hand_gains_cli.call_async(req))
        ok = bool(resp.success) if resp is not None else False
        if not ok:
            self.get_logger().warning(
                'set_hand_gains failed — tossing on the ambient hand gains')
        return ok

    def _announce_toss(self, seq) -> None:
        """ANNOUNCE (its own FSM action, ≥1 tick after the armed confirm): publish
        the self-ThrowAnnouncement that closes the existing correlation → catch
        path. All physics fields come from the single tested release-state module;
        thrower_name AND target_id are this robot — target_id is the load-bearing
        one (the tracker tags destination from it; catch_coordinator's pre-tilt
        gate requires the strict match; thrower_name is cosmetic). This is the ONE
        ROS↔perf clock crossing of the sequence: the FSM's perf-domain t_release
        becomes the absolute ROS throw_time."""
        if seq.announce_lead_short:
            # WARN-only for BOTH tiers (the Phase-1 "8b hardens this to an abort"
            # promise was SUPERSEDED in Phase 4 — see the sequencer's
            # TOSS_MIN_ANNOUNCE_LEAD_S comment): level 8a's pre-tilt target
            # equals the already-held pose (motion-free), and 8b's platform reach
            # is DEFERRED to t_release with lead = the flight time by
            # construction, so this announce lead sizes no 8b platform motion.
            self.get_logger().warning(
                'toss announce→landing lead is short (< 2.5 s) — pre-tilt '
                'degenerates toward arrive-at-contact (motion-free for level 8a; '
                '8b platform reach is deferred to release regardless)')
        with self._lock:
            release = self._toss_release_state
        now_perf = time.perf_counter()
        now_ros = self.get_clock().now()
        delta_s = seq.t_release - now_perf
        fields = build_announcement_fields(
            release, throw_time_s=now_ros.nanoseconds * 1e-9 + delta_s)
        ann = ThrowAnnouncement()
        ann.header.stamp = now_ros.to_msg()
        ann.header.frame_id = 'world'
        ann.thrower_name = self._robot_name
        ann.target_id = self._robot_name
        ip = fields['initial_position']
        iv = fields['initial_velocity']
        lp = fields['landing_position']
        lv = fields['landing_velocity']
        ann.initial_position = Point(x=float(ip[0]), y=float(ip[1]), z=float(ip[2]))
        ann.initial_velocity = Vector3(x=float(iv[0]), y=float(iv[1]), z=float(iv[2]))
        ann.landing_position = Point(x=float(lp[0]), y=float(lp[1]), z=float(lp[2]))
        ann.landing_velocity = Vector3(x=float(lv[0]), y=float(lv[1]), z=float(lv[2]))
        ann.predicted_tof_sec = float(fields['predicted_tof_sec'])
        ann.throw_time = (now_ros + rclpy.time.Duration(seconds=delta_s)).to_msg()
        ann.landing_time = (now_ros + rclpy.time.Duration(
            seconds=delta_s + float(seq.flight_time_s))).to_msg()
        # Stash the announced landing state for Tier 8b's deferred A->B reach to
        # reuse (landing_pos = B's cup point global mm, landing_vel, and the ROS
        # landing time = the same value ann.landing_time encodes). 8a never reads
        # it — _publish_toss_reach is only reached on ACTION_REACH_CATCH.
        landing_time_ros_s = (now_ros.nanoseconds * 1e-9 + delta_s
                              + float(seq.flight_time_s))
        with self._lock:
            self._toss_announced_reach = (
                np.array(lp, dtype=float), np.array(lv, dtype=float),
                landing_time_ros_s)
            # THE JOIN KEY, stashed for the per-toss record. The SAME float this
            # method wrote into ThrowAnnouncement.throw_time, so the declaration
            # and the bagged announcement match exactly rather than to a
            # tolerance (toss-selftuning § 3.4). Record-only — nothing reads it
            # back into the FSM.
            self._toss_record_announce = (
                now_ros.nanoseconds * 1e-9 + delta_s, landing_time_ros_s)
        self._announce_pub.publish(ann)
        seq.note_announcement()

    def _dispatch_toss_throw(self, seq):
        """THE throw dispatch (set_hand_traj_cmd traj_type=0) — single-shot,
        tri-state, NEVER retried (unlike the idempotent smooth-moves): an ambiguous
        ack may have armed the stroke, and a re-dispatch re-packs a new wall_time /
        replaces a live stroke / post-release clobbers the armed catch stroke on
        the last-writer-wins queue. Returns (classification, message):

          - 'rejected'  — service unavailable, or an ack failure carrying one of
            the bridge's own validation ValueErrors (raised BEFORE any CAN frame —
            a definitive no-arm);
          - 'ok'        — ack success (may still be a lie; treated identically to
            ambiguous downstream — only release EVIDENCE advances the FSM);
          - 'ambiguous' — any other failure (the ERR_TIMEOUT class: the frame may
            have transmitted) or a response timeout.

        event_delay is recomputed from the ABSOLUTE scheduled release at dispatch
        time (invariant to earlier stalls), shifted EARLY by the T0-measured
        release latency (ships 0.0) while the announced landing stays un-shifted —
        the BB pattern."""
        if not self._hand_traj_cli.wait_for_service(timeout_sec=_SERVICE_WAIT_S):
            return (THROW_DISPATCH_REJECTED,
                    'set_hand_traj_cmd service unavailable')
        req = SetHandTrajCmd.Request()
        req.traj_type = 0
        req.event_delay = float(
            seq.t_release - time.perf_counter()
            - float(hw.JB_OP_TOSS_RELEASE_LATENCY_MS) / 1000.0)
        req.event_vel = float(seq.event_vel_mps)
        with self._lock:
            # Before the call, not after: the frame may fire while we wait on the
            # ack, and the release-stroke telemetry watch must already be armed.
            self._toss_throw_dispatched = True
        resp = self._wait_future(self._hand_traj_cli.call_async(req))
        if resp is None:
            return THROW_DISPATCH_AMBIGUOUS, 'set_hand_traj_cmd ack timeout'
        message = str(resp.message)
        if bool(resp.success):
            return THROW_DISPATCH_OK, message
        return self._classify_hand_traj_reject(message), message

    @staticmethod
    def _classify_hand_traj_reject(message: str) -> str:
        """Classify a failed set_hand_traj_cmd ack: the bridge's validation
        ValueErrors mean no CAN frame ever existed ('rejected'); anything else may
        have transmitted ('ambiguous'). Pinned against the bridge's REAL validation
        path by test (no silently-drifting copies)."""
        if str(message).startswith(_BRIDGE_HAND_TRAJ_REJECT_PREFIXES):
            return THROW_DISPATCH_REJECTED
        return THROW_DISPATCH_AMBIGUOUS

    def _publish_prime_hold(self, hold: bool):
        """Publish the catch/prime_hold suppression gate (see the publisher's
        comment in __init__ for the invariant it enforces)."""
        self._prime_hold_pub.publish(Bool(data=bool(hold)))

    def _publish_pretilt_hold(self, hold: bool):
        """Publish the catch/pretilt_hold suppression gate (Tier 8b only; see the
        publisher's comment in __init__ for the invariant it enforces)."""
        self._pretilt_hold_pub.publish(Bool(data=bool(hold)))

    def _publish_reach_center(self, catch_pose_stow_mm):
        """Declare the reach-envelope centre for the catch about to be armed
        (contract C-REACH-1; see the publisher's comment in __init__ for the
        ordering requirement and the benign-degradation argument)."""
        x, y, z = (float(v) for v in catch_pose_stow_mm)
        self._reach_center_pub.publish(Point(x=x, y=y, z=z))
        self.get_logger().info(
            'toss declared catch reach centre (%.1f, %.1f, %.1f) mm STOW'
            % (x, y, z))

    def _publish_toss_reach(self):
        """Tier 8b's deferred A->B reach (ACTION_REACH_CATCH, time-triggered at
        t_release): build the ONE catch/dynamic_target for B from the stashed
        announced landing via the SAME policy catch_coordinator_node uses
        (predicted_catch_command — the receive-tilt/swing math is single-sourced
        through _toss_catch_policy, never a drift-prone copy) and publish it with
        arrival = the announced landing (lead = the flight time by construction).
        Mirrors CCN's _publish_dynamic_target field-mapping + ROS->perf crossing
        exactly. A missing stash, or a policy reject (should not happen
        post-CHECKING, but guarded), logs a warning and publishes nothing — the
        FSM's ABORTED_NO_RELEASE / MISSED path still cleans up."""
        with self._lock:
            stash = self._toss_announced_reach
        if stash is None:
            self.get_logger().warning(
                'toss deferred reach requested but no announced landing was '
                'stashed — publishing no platform target')
            return
        landing_pos, landing_vel, landing_time_ros_s = stash
        cmd = self._toss_catch_policy.predicted_catch_command(
            landing_pos, landing_vel, landing_time_ros_s)
        if cmd is None:
            self.get_logger().warning(
                'toss deferred reach: predicted_catch_command rejected the '
                'announced landing — publishing no platform target')
            return
        # ROS->perf crossing — the coordinator's own convention (offset =
        # now_perf − now_ros, the SAME crossing _ball_time_at_land_perf uses);
        # arrival_perf mirrors CCN's cmd.landing_time + _ros_to_perf_offset.
        now_perf = time.perf_counter()
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        arrival_perf = cmd.landing_time + (now_perf - now_ros)
        out = DynamicTargetCommand()
        out.target_pos = Point(
            x=float(cmd.target_pos[0]),
            y=float(cmd.target_pos[1]),
            z=float(cmd.target_pos[2]),
        )
        out.target_quat = Quaternion(
            w=float(cmd.target_quat[0]),
            x=float(cmd.target_quat[1]),
            y=float(cmd.target_quat[2]),
            z=float(cmd.target_quat[3]),
        )
        out.target_vel = Vector3(
            x=float(cmd.target_vel[0]),
            y=float(cmd.target_vel[1]),
            z=float(cmd.target_vel[2]),
        )
        out.arrival_time = arrival_perf
        self._dyn_target_pub.publish(out)
        self.get_logger().info(
            'toss deferred A->B reach published (arrival %.3f perf, ROS lead '
            '%.3f s)' % (arrival_perf, cmd.landing_time - now_ros))

    def _toss_recenter(self):
        """Toss RECENTER (CAUGHT): the reload teardown verbatim (lower latch,
        catch/armed False, go_home — the hand keeps the caught ball, no retract,
        so the next Toss is immediately serviceable), THEN prime_hold False LAST.
        The release must follow the armed-False publish — they travel on different
        topics with no ordering guarantee, and a released hold meeting a
        still-armed catch_coordinator re-opens the auto-prime exactly while the
        caught ball rests in the cup (the ascent would launch it). Releasing late
        costs nothing: a standing hold only suppresses auto-primes, and both
        ball-ops prime proactively themselves. Tier 8b's catch/pretilt_hold is
        released LAST of all (iff it was raised this goal), same
        cross-topic-ordering rationale: a stale pretilt_hold only DEGRADES
        (a reload announcement loses its platform pre-tilt) — never a hazard."""
        self._recenter()
        self._publish_prime_hold(False)
        if self._toss_pretilt_hold_raised:
            self._publish_pretilt_hold(False)

    def _toss_stay(self):
        """Toss STAY (CAUGHT, the default since 2026-07-29): the RECENTER ladder
        MINUS go_home — lower the catch latch, publish catch/armed False, then
        release prime_hold and (iff raised) pretilt_hold LAST.

        The platform is left holding the catch pose because the emitter's
        terminal hold already does that when nobody commands otherwise: this is
        *not calling go_home*, not a new mechanism, and it issues no new
        setpoint of any kind. That is what lets a session CHAIN — the next
        Toss's throw site A is read from this live commanded pose.

        Ordering is the RECENTER ordering verbatim and load-bearing for the same
        reason: catch/armed False must precede the prime_hold release, because a
        released hold meeting a still-armed catch_coordinator re-opens the
        auto-prime exactly while the caught ball rests in the cup (the ascent
        would launch it). A stale pretilt_hold only DEGRADES (a later reload
        announcement loses its platform pre-tilt) — never a hazard — so it goes
        last of all.

        Residual, deliberately accepted and instrumented rather than designed
        away: the held pose carries the catch's RECEIVE TILT (up to ~3.6° at the
        150 mm cap), so the ball rests in a slightly tilted cup until the next
        command instead of being returned to level. The catch already holds that
        tilt for the whole quiescent settle with the ball in it, so this extends
        an existing, hardware-observed state rather than creating one — but it
        extends it INDEFINITELY, which the machine has never done. The operator
        runbook scores it (§ SECTION DISP row DISP-5, a REPORT row); set
        toss_stay_at_pose_on_caught false to revert to go_home if it does not
        hold."""
        self._arm_catch(False)
        self._publish_catch_armed(False)
        self._publish_prime_hold(False)
        if self._toss_pretilt_hold_raised:
            self._publish_pretilt_hold(False)

    def _toss_safe_abort(self):
        """Toss SAFE_ABORT: the reload safing ladder verbatim (catch/armed False
        FIRST so the retry tick stands down, telemetry-verified retract — which
        deliberately also CLEARS any armed throw stroke on the last-writer-wins
        queue, the Teensy's only un-arm mechanism — then latch lower, go_home),
        THEN prime_hold False LAST: the hold must outlive the whole teardown for
        the same cross-topic-ordering reason as _toss_recenter — a pre-release
        abort has the ball still seated in the cup. Tier 8b's catch/pretilt_hold
        is released LAST of all (iff it was raised this goal), same rationale;
        _safe_toss_on_early_exit routes through here so cancel/timeout/shutdown
        are covered."""
        self._safe_abort()
        self._publish_prime_hold(False)
        if self._toss_pretilt_hold_raised:
            self._publish_pretilt_hold(False)

    def _safe_toss_on_early_exit(self, seq):
        """Safe the robot on a NODE-level toss exit (cancel honoured / timeout /
        shutdown) that bypasses the FSM's own terminal. `prepared` covers both
        commitments that need undoing: an accepted platform move (go_home leg) and
        a dispatched PREPARE (latch / prime_hold / possibly an armed stroke)."""
        if seq.prepared:
            self.get_logger().warning(
                'Toss early exit while prepared — safing (retract + lower latch '
                '+ recenter + release prime hold).')
            self._toss_safe_abort()

    def _log_toss_outcome(self, result) -> None:
        """The single authoritative per-toss outcome line (the reload discipline:
        exactly one per goal, INFO on success / WARN otherwise), carrying the
        diagnostic fields when finite.

        ``catch_dt`` is the HAND-SENSOR catch-event time (arrival edge minus the
        predicted landing) — the machine's first direct measurement of WHEN the
        ball entered the cup, and the per-toss timing record the Phase-2 learning
        loop consumes. It is read with ``getattr`` because this method is called
        with BOTH a ``TossResult`` and (on the bad-goal path) the action's own
        Result message, and only the former carries the field."""
        err = float(result.catch_error_mm)
        fl = float(result.achieved_flight_s)
        dt = float(getattr(result, 'catch_event_dt_s', float('nan')))
        parts = []
        if np.isfinite(err):
            parts.append(f'catch_err={err:.0f} mm')
        if np.isfinite(fl):
            parts.append(f'flight={fl:.3f} s')
        if np.isfinite(dt):
            parts.append(f'catch_dt={dt:+.3f} s')
        suffix = f" ({', '.join(parts)})" if parts else ''
        line = f'Toss {result.outcome}{suffix}'
        if result.success:
            self.get_logger().info(line)
        else:
            self.get_logger().warning(line)
        self._publish_toss_record(result)

    # ── The per-toss record (INSTRUMENT ONLY — no control authority) ──────────

    def _ros_clock_s(self) -> float:
        """The node's ROS clock in seconds — ``clock_offset``'s injection point."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _refresh_clock_offset(self) -> None:
        """30 s median-filtered ``perf - ros`` refresh, for the RECORD only.

        The FSM is untouched: ``_announcement_landing_perf`` keeps its single
        instantaneous read. Both numbers land in every record so the open
        reconciliation question that method documents becomes a measurement.
        """
        with self._lock:
            history = self._clock_offset_history
        offset = clock_offset.refresh_offset(history, self._ros_clock_s)
        with self._lock:
            self._perf_minus_ros_s = offset

    def _toss_uid(self, goal_id, cycle_index) -> str:
        """THE per-cycle record identity. One formula, one place — the retry
        back-reference (``retry_of``) names a uid this method minted, so a second
        copy of the format string is how a corpus ends up with dangling
        references."""
        return '{}-{}-{}'.format(self._session_id,
                                 str(goal_id or '')[:8], int(cycle_index))

    def _open_toss_record(self, *, action, goal_id, cycle_index,
                          catch_pose, throw_delay, vel_scale, raw_goal,
                          flight=None, session=None, reload_settle=False,
                          retry=False) -> None:
        """Install the declaration context for ONE cycle. Never reads back into
        the FSM; deleting this method changes no commanded motion.

        Called BEFORE the goal-numerics gate as well as after the cycle is
        built, so a ``REJECTED_BAD_GOAL`` terminal records ITS own identity
        rather than inheriting the previous goal's — a census whose rejected rows
        carry the wrong ``toss_uid`` is worse than one with no rejected rows.
        ``flight`` is therefore optional: on that path no flight time was ever
        resolved, and the record says so with a null. The live sequencer is read
        at terminal time from ``self._active_seq``, not carried here — one source
        for "which FSM is running", the one the rest of the node already uses.
        """
        uid = self._toss_uid(goal_id, cycle_index)
        ctx = {
            'action': action,
            'goal_id': goal_id,
            'cycle_index': int(cycle_index),
            'uid': uid,
            'catch_pose': tuple(float(v) for v in catch_pose),
            'flight': (float(flight) if flight is not None else None),
            'throw_delay': float(throw_delay),
            'vel_scale': float(vel_scale),
            'raw_goal': dict(raw_goal),
            'session': session,
            # Guard G10: the cycle after a reload interlude is excluded from
            # every fit. Guard G11: a retried cycle names what it retried.
            'reload_settle': bool(reload_settle),
        }
        with self._lock:
            prev_uid = self._toss_record_prev_uid
            # The Layer-1.5 reads taken during the dwell that PRECEDED this
            # cycle belong to this cycle's record — they describe the platform
            # attitude the throw was launched from. Snapshotted and cleared here
            # so a read can never be attributed to two cycles.
            ctx['dwell_tilt'] = list(self._dwell_tilt_reads)
            ctx['dwell_tilt_degraded'] = bool(self._dwell_tilt_degraded)
            self._dwell_tilt_reads = []
            self._dwell_tilt_next_at = 0.0
            self._dwell_tilt_degraded = False
            ctx['retry_of'] = prev_uid if (retry and prev_uid) else None
            self._toss_record_prev_uid = uid
            self._toss_record_ctx = ctx
            self._toss_record_announce = None
            self._toss_record_belt_warned = False

    def _toss_record_fields(self, result) -> dict:
        """Assemble THE declaration for this cycle. Pure-ish: reads cached state
        under the lock, allocates a dict, touches no hardware and no service."""
        rec = toss_record.blank_record()
        with self._lock:
            ctx = self._toss_record_ctx
            announce = self._toss_record_announce
            seq = self._active_seq
            release = self._toss_release_state
            release_cmd = self._toss_release_cmd
            aim = self._toss_aim
            cal_version = self._toss_cal_version
            cal_present = self._toss_cal is not None
            gravity_loaded = self._gravity_correction_loaded
            tilt_loaded = self._tilt_map_loaded
            tilt_version = self._tilt_map_version
            perf_minus_ros = self._perf_minus_ros_s
            vel_scale = self._catch_vel_scale
            stroke_seen = self._toss_stroke_seen
            track_confirmed = self._toss_track_confirmed
            trim = self._toss_trim
        trim_snapshot = trim.snapshot() if trim is not None else None
        ctx = ctx or {}
        # A cycle was actually built iff a flight time was resolved. On the
        # REJECTED_BAD_GOAL path nothing was built, so the node's RESOLVED
        # per-goal state (_catch_vel_scale, _active_seq, _toss_release_state) is
        # still the PREVIOUS goal's — reading it would attribute the last toss's
        # numbers to this rejection. Raw goal fields stay valid either way.
        built = ctx.get('flight') is not None
        if not built:
            seq = None
            release = None
            release_cmd = None
            aim = None
            vel_scale = None
        now_ros = self._ros_clock_s()
        cycle = int(ctx.get('cycle_index', 0) or 0)
        goal_id = str(ctx.get('goal_id') or '')
        rec.update({
            'toss_uid': (ctx.get('uid')
                         or self._toss_uid(goal_id, cycle)),
            'session_id': self._session_id,
            'goal_id': goal_id or None,
            'action': str(ctx.get('action') or 'toss'),
            'cycle_index': cycle or None,
            't_record_ros': now_ros,
            'perf_minus_ros_s': perf_minus_ros,
            # The FSM's own crossing recipe, sampled here rather than filtered —
            # see _announcement_landing_perf for why the two coexist.
            'perf_minus_ros_inst_s': time.perf_counter() - now_ros,
            'git_sha': self._git_sha,
            'git_dirty': self._git_dirty,
            'gravity_correction_loaded': gravity_loaded,
            'tilt_map_applied': bool(tilt_loaded and gravity_loaded),
            'tilt_map_version': tilt_version or None,
            'toss_tier': str(hw.JB_OP_TOSS_TIER),
            'catch_knobs': self._toss_record_catch_knobs(vel_scale),
            'outcome': str(getattr(result, 'outcome', '') or 'UNKNOWN'),
            'success': bool(getattr(result, 'success', False)),
            'throw_stroke_seen': bool(stroke_seen),
            'ball_track_confirmed': bool(track_confirmed),
            'achieved_flight_s_fsm': float(
                getattr(result, 'achieved_flight_s', float('nan'))),
            'catch_error_mm_fsm': float(
                getattr(result, 'catch_error_mm', float('nan'))),
            'catch_event_dt_s_fsm': float(
                getattr(result, 'catch_event_dt_s', float('nan'))),
        })
        if announce is not None:
            rec['announce_throw_time_ros'] = float(announce[0])
            rec['announce_landing_time_ros'] = float(announce[1])
        raw = ctx.get('raw_goal') or {}
        pose = ctx.get('catch_pose')
        rec.update({
            'goal_catch_xyz_stow_mm': list(pose) if pose else None,
            'goal_throw_height_m_raw': raw.get('throw_height_m'),
            'goal_throw_delay_s_raw': raw.get('throw_delay_s'),
            'goal_catch_vel_scale_raw': raw.get('catch_vel_scale'),
            'goal_catch_vel_scale': vel_scale,
        })
        if ctx.get('flight') is not None:
            rec['flight_time_s'] = float(ctx['flight'])
            rec['goal_throw_height_m'] = float(
                apex_height_from_flight_time(float(ctx['flight'])))
            rec['apex_height_m'] = rec['goal_throw_height_m']
        if seq is not None:
            rec.update(self._toss_record_seq_fields(seq))
        if ctx.get('throw_delay') is not None:
            rec['goal_throw_delay_s'] = float(ctx['throw_delay'])
        session = ctx.get('session')
        if session is not None:
            rec.update({
                'goal_num_throws': int(getattr(session, 'num_throws', 0)) or None,
                'goal_dwell_time_s': float(getattr(session, 'dwell_time_s', 0.0)),
                'goal_stop_on_miss': bool(getattr(session, 'stop_on_miss', True)),
                # RESOLVED, never raw: on_empty_cup has already been through
                # resolve_on_empty_cup and max_reloads through the config
                # default, so the corpus records the policy the machine ran
                # rather than the string the operator typed.
                'goal_on_empty_cup': str(getattr(session, 'on_empty_cup', '')),
                'goal_max_reloads': int(getattr(session, 'max_reloads', 0)),
            })
        # Guards G10 / G11 — the two exclusion flags. Explicit booleans on every
        # session cycle (never null), because "was this cycle after a reload?" has
        # a definite answer for every cycle a session ran, and a null would make
        # a fit silently include it.
        if ctx.get('session') is not None:
            rec['reload_settle'] = bool(ctx.get('reload_settle', False))
        if ctx.get('retry_of'):
            rec['retry_of'] = str(ctx['retry_of'])
        # ── Layer 1.5 (§ 3.10) — COVARIATE, zero control authority ──
        rec['dwell_tilt_degraded'] = bool(ctx.get('dwell_tilt_degraded', False))
        rec.update(self._dwell_tilt_fields(
            list(ctx.get('dwell_tilt') or []),
            getattr(seq, 't_release', None) if seq is not None else None))
        if release is not None:
            # catch_point_global_mm comes from the UNCORRECTED state: it is B's
            # cup point, the quantity the miner independently recomputes from
            # goal_catch_xyz_stow_mm and fails loud on mismatch (plan § 7 R1).
            # An aim-corrected catch point would make that check fire on every
            # aimed toss and would redefine land_err against a target the
            # operator never nominated.
            rec['catch_point_global_mm'] = [
                float(v) for v in release.catch_point_global_mm]
        if release_cmd is not None:
            # Everything the machine actually DID comes from the commanded
            # state: the release point, the launch vector, the dispatched
            # event_vel and the commanded tilt.
            rec.update({
                # Read off the SEQUENCER, not off `release_cmd`: the sequencer is
                # what `_dispatch_toss_throw` sends (`req.event_vel =
                # seq.event_vel_mps`), and since layer 3's velocity trim is
                # applied between the two they are no longer the same number
                # whenever it acts. With layer 3 inactive the sequencer holds the
                # exact float `release_cmd.event_vel_mps` produced, so this is
                # bit-identical to the pre-Phase-2 record.
                'event_vel_mps': float(seq.event_vel_mps) if seq is not None
                else float(release_cmd.event_vel_mps),
                'release_pos_global_mm': [float(v) for v in
                                          release_cmd.release_pos_global_mm],
                'launch_vel_mms': [float(v) for v in release_cmd.launch_vel_mms],
                'aim_tilt_rx_rad': float(getattr(release_cmd, 'tilt_rx', 0.0)),
                'aim_tilt_ry_rad': float(getattr(release_cmd, 'tilt_ry', 0.0)),
            })
        # ── Applied calibration (§ 3.3, "the most important block") ──
        # Explicit ZEROS, never nulls: a null reads as "unknown", a zero reads as
        # "nothing was applied", and the corpus has to be able to prove which one
        # its baseline was. That holds for the REJECTED_BAD_GOAL path too — a
        # goal that never built a cycle commanded exactly zero aim. `trim_aim_rad`
        # is a STRUCTURAL zero since 2026-08-21 — layer 2's aim estimator is
        # monitor-only (C4) — and its estimate rides beside it in
        # `trim_monitor_aim_rad`.
        rec.update({
            'map_aim_rad': [0.0, 0.0],
            'trim_aim_rad': [0.0, 0.0],
            'trim_monitor_aim_rad': [0.0, 0.0],
            'trim_authority': toss_trim.AIM_AUTHORITY,
            'total_aim_rad': [0.0, 0.0],
            'map_aim_mm_at_h': [0.0, 0.0],
            'trim_aim_mm_at_h': [0.0, 0.0],
            # Layer 3, POST-GATE. Explicit zeros for the same reason the two
            # above are explicit zeros, and one more: a REFUSED correction (total
            # clamp or validate_event_vel) must be recorded as the zero it became,
            # or the next fit learns against a command the machine never flew.
            'ilc_aim_rad': [0.0, 0.0],
            'ilc_spatial_aim_rad': [0.0, 0.0],
            'ilc_session_aim_rad': [0.0, 0.0],
            'ilc_session_applied': False,
            'ilc_session_reason': toss_ilc.SESSION_NO_ARTIFACT,
            'ilc_session_n': 0,
            'ilc_vel_trim': 0.0,
            'clamp_hits': [],
            'toss_cal_loaded': bool(cal_present),
            'toss_cal_version': cal_version or None,
            'toss_cal_applied': False,
            'release_latency_ms_applied': float(
                getattr(hw, 'JB_OP_TOSS_RELEASE_LATENCY_MS', 0.0)),
        })
        if aim is not None:
            rec.update({
                'map_aim_rad': [float(v) for v in aim['map_aim_rad']],
                'trim_aim_rad': [float(v) for v in aim['trim_aim_rad']],
                'trim_monitor_aim_rad': [
                    float(v) for v in aim['trim_monitor_aim_rad']],
                'trim_authority': str(aim['trim_authority']),
                'total_aim_rad': [float(v) for v in aim['aim_rad']],
                # REPORT fields: mm at THIS toss's apex, never the stored unit.
                # They are the exact virtual-target offsets that were commanded,
                # so they need no re-derivation (and cannot drift from one).
                'map_aim_mm_at_h': [float(v) for v in aim['map_offset_mm']],
                'trim_aim_mm_at_h': [float(v) for v in aim['trim_offset_mm']],
                'ilc_aim_rad': [float(v) for v in aim['ilc_aim_rad']],
                'ilc_spatial_aim_rad': [
                    float(v) for v in aim['ilc_spatial_aim_rad']],
                'ilc_session_aim_rad': [
                    float(v) for v in aim['ilc_session_aim_rad']],
                'ilc_session_applied': bool(aim['ilc_session_applied']),
                'ilc_session_reason': str(aim['ilc_session_reason']),
                'ilc_session_n': int(aim['ilc_session_n']),
                'ilc_vel_trim': float(aim['ilc_vel_trim']),
                'clamp_hits': list(aim['clamp_hits']),
                'toss_cal_applied': bool(aim['applied']),
            })
        # ── Layer 2 provenance (§ 3.6) ──
        # Written from the trim's OWN snapshot, not from the aim block: the
        # state and the sample count describe the estimator, which keeps
        # learning whether or not `toss_trim_enabled` let its output through.
        # `trim_aim_rad` above is what was COMMANDED (zero while disabled); this
        # is what was KNOWN. A corpus has to be able to tell those apart, or a
        # future reader cannot say whether a session had no bias or no
        # permission.
        if trim_snapshot is not None:
            rec.update({
                'trim_source_n': int(trim_snapshot['n']),
                'trim_state': str(trim_snapshot['state']),
                'trim_reset_reason': str(trim_snapshot['reason'] or ''),
                'speed_bias_applied': float(trim_snapshot['speed_k_v']),
                'timing_bias_applied_ms': float(trim_snapshot['tau_ms']),
            })
        return rec

    def _toss_record_seq_fields(self, seq) -> dict:
        """The FSM half of the declaration, read off the live sequencer.

        Several of these have no public accessor on ``TossSequencer`` and are
        read as private fields. That is deliberate for an INSTRUMENT: widening
        the sequencer's public surface for fields nothing branches on would
        invite something to start branching on them. Every read goes through
        ``getattr`` with a default, so a rename in the FSM degrades this record
        to a null rather than raising on a terminal path — and a null here is
        visible in the corpus, which is how the drift gets noticed.
        """
        dispatch = getattr(seq, '_throw_dispatch_result', None)
        position = getattr(seq, '_position_result', None)
        out = {
            'phase_at_terminal': str(getattr(seq, 'phase', '') or '') or None,
            'prepare_ok': getattr(seq, '_prepare_result', None),
            'catch_target_accepted': bool(getattr(seq, '_catch_accepted', False)),
            'announce_lead_short': bool(getattr(seq, 'announce_lead_short', False)),
            't_accept_perf': float(getattr(seq, '_t_accept', 0.0)) or None,
            't_release_perf': float(getattr(seq, 't_release', 0.0)) or None,
            'event_delay_s': (float(getattr(seq, 'throw_delay_s', 0.0))
                              if getattr(seq, 'throw_delay_s', 0.0) else None),
            'throw_site_xy_mm': [float(v) for v in
                                 getattr(seq, 'throw_site_xy_mm', (0.0, 0.0))],
        }
        landing = getattr(seq, 'landing_perf', None)
        if landing is not None:
            try:
                out['t_landing_sched_perf'] = float(landing)
            except (TypeError, ValueError):
                pass
        if dispatch:
            out['throw_dispatch_class'] = str(dispatch[0])
            out['throw_dispatch_message'] = str(dispatch[1]) or None
        if position:
            out['position_accepted'] = bool(position[0])
            out['position_planned_s'] = float(position[1])
            out['position_code'] = str(position[2]) or None
        return out

    @staticmethod
    def _toss_record_catch_knobs(vel_scale) -> dict:
        """The catch-side tunables in force for this toss.

        Recorded because a corpus that pools two catch tunings is a corpus of
        two machines (plan § 7 R3) — and the knobs are config, so nothing else in
        the bag witnesses them.

        ``vel_scale`` may be ``None``: on the REJECTED_BAD_GOAL path no cycle was
        built, so no scale was ever resolved, and a null is the honest value. It
        must NOT fall back to the config default — that would record a knob the
        machine did not run.
        """
        return {
            'catch_vel_scale': (float(vel_scale) if vel_scale is not None
                                else None),
            'catch_vel_ratio': float(hw.TEENSY_TRAJ_CATCH_VEL_RATIO),
            'catch_vel_hold_pct': float(hw.TEENSY_TRAJ_CATCH_VEL_HOLD_PCT),
            'catch_reach_freeze_s': float(hw.JB_TRAJ_CATCH_REACH_FREEZE_S),
            'catch_settle_hold_s': float(hw.JB_TRAJ_CATCH_SETTLE_HOLD_S),
            'catch_reach_envelope_mm': float(hw.JB_TRAJ_CATCH_REACH_ENVELOPE_MM),
            # The gains the TOSS itself installs at PREPARE — the soft-catch set,
            # not the ODrive defaults. Recording the defaults here would be a
            # record of numbers the machine was not running.
            'hand_pos_gain': float(_TOSS_SOFT_CATCH_GAINS['pos_gain']),
            'hand_vel_gain': float(_TOSS_SOFT_CATCH_GAINS['vel_gain']),
            'hand_vel_int_gain': float(
                _TOSS_SOFT_CATCH_GAINS['vel_integrator_gain']),
        }

    def _publish_toss_record(self, result) -> None:
        """Publish ONE declaration on ``toss/record`` and belt it to JSONL.

        Called from the single authoritative outcome line, so exactly one record
        exists per cycle terminal — including the REJECTED_BAD_GOAL path, where
        the census matters most and the record degrades to identity + outcome.

        **Fails silently, by design.** This is an instrument; it must never be
        able to affect a teardown. The whole body is guarded, the belt write
        happens OUTSIDE ``self._lock``, and the belt warns once per goal and then
        goes quiet — a full disk is a lost measurement, never a stalled abort
        ladder.
        """
        try:
            record = self._toss_record_fields(result)
            payload = toss_record.encode(record)
        except Exception as exc:                               # noqa: BLE001
            self.get_logger().warning(
                'toss record NOT built ({}) — instrument only, the cycle is '
                'unaffected'.format(exc))
            return
        try:
            self._toss_record_pub.publish(String(data=payload))
        except Exception as exc:                               # noqa: BLE001
            self.get_logger().warning('toss/record publish failed: {}'.format(exc))
        # CENSUS B6 — the belt write and the trim update leave the cycle thread.
        # The ROS publish stays here: it is a non-blocking hand-off to the
        # middleware, and it is the CANONICAL sink (the belt exists only so a
        # `record:=false` bench session still produces a corpus). What moves is
        # the pair that can BLOCK: an `open(..., 'a')` on the SD card and a numpy
        # estimator update, both of which ran synchronously inside
        # _run_toss_cycle before it returned — i.e. squarely in the
        # landing -> next-cycle handoff. A full disk or a slow card was a cadence
        # fault; now it is a lost measurement, which is what an instrument's
        # failure should cost.
        #
        # The context snapshot is taken HERE, on the cycle thread, and passed in.
        # _toss_trim_observe reads _toss_record_ctx / _toss_aim / _toss_trim, and
        # _build_toss_cycle overwrites all three at the NEXT cycle — so a worker
        # that read them later would attribute cycle N's toss to cycle N+1's pose
        # and aim. That is the one real hazard in moving this off-thread, and it
        # is closed by construction rather than by timing.
        self._toss_records_submit(payload, record, self._toss_trim_snapshot())

    # ── The record worker (census B6) ─────────────────────────────────────────
    # ONE serialising thread, started lazily, drained at every point that READS
    # what it produces. Not a pool: the trim estimator MUTATES, and the belt is
    # an append log, so both need the cycle order the queue's FIFO gives them.

    def _toss_trim_snapshot(self) -> dict:
        """Freeze everything the trim update reads, on the CYCLE thread.

        Taken at submit time, not at execute time, because ``_build_toss_cycle``
        overwrites ``_toss_record_ctx`` / ``_toss_aim`` / ``_toss_trim_t0`` for
        the NEXT cycle. ``_toss_trim`` itself is the live estimator object and is
        carried by reference on purpose — it is the thing being updated, and the
        single worker thread is what serialises those updates. It is read under
        the lock so a concurrent ``_toss_trim_end`` cannot swap it mid-read."""
        with self._lock:
            ctx = self._toss_record_ctx or {}
            return {
                'trim': self._toss_trim,
                'pose': ctx.get('catch_pose'),
                'flight': ctx.get('flight'),
                'aim': self._toss_aim,
                't0': self._toss_trim_t0,
            }

    def _toss_records_submit(self, payload: str, record: dict,
                             snapshot: dict) -> None:
        """Queue one cycle's belt write + trim update for the worker.

        Falls back to running them INLINE when the worker cannot be started —
        so a node whose thread creation failed still produces a corpus and still
        converges a trim, just on the cycle thread as before. Degrading to the
        old behaviour is the right failure here; dropping the measurement is
        not."""
        try:
            self._toss_records_worker_start()
            self._toss_records_q.put_nowait((payload, record, snapshot))
            return
        except Exception as exc:                               # noqa: BLE001
            self.get_logger().warning(
                'toss record worker unavailable ({}) — running the belt write '
                'and trim update INLINE on the cycle thread'.format(exc))
        self._toss_records_run_one(payload, record, snapshot)

    def _toss_records_worker_start(self) -> None:
        """Start the belt/trim worker, ONCE, and only if it really started.

        The handle is latched inside the lock so two cycles racing here cannot
        start two workers — but ``start()`` runs OUTSIDE the lock (it can block)
        and it can RAISE (thread exhaustion on a loaded Jetson). Rolling the
        handle back on that failure is what keeps the retry possible: without it
        the caller's inline fallback writes THAT record correctly, and every
        later call returns early at the ``is not None`` check and queues into a
        queue no thread is servicing — silent, permanent loss of the session's
        toss corpus, announced as one transient warning, with every subsequent
        ``_toss_trim_end`` burning the full 5 s drain timeout (audit fix,
        2026-08-22)."""
        with self._lock:
            if self._toss_records_thread is not None:
                return
            t = threading.Thread(target=self._toss_records_worker,
                                 name='toss_records', daemon=True)
            self._toss_records_thread = t
        try:
            t.start()
        except Exception:                                      # noqa: BLE001
            with self._lock:
                if self._toss_records_thread is t:
                    self._toss_records_thread = None
            raise

    def _toss_records_worker(self) -> None:
        while True:
            item = self._toss_records_q.get()
            try:
                if item is None:                      # shutdown sentinel
                    return
                self._toss_records_run_one(*item)
            except Exception as exc:                  # noqa: BLE001
                # Belt-and-braces: _toss_records_run_one's two callees are each
                # individually guarded, so reaching here means something outside
                # them raised. The worker must survive it — a dead worker would
                # silently stop the corpus for the rest of the session.
                self.get_logger().warning(
                    'toss record worker iteration failed ({}) — the worker '
                    'continues'.format(exc))
            finally:
                self._toss_records_q.task_done()

    def _toss_records_run_one(self, payload: str, record: dict,
                              snapshot: dict) -> None:
        self._belt_toss_record(payload)
        self._toss_trim_observe(record, snapshot)

    def _toss_records_drain(self, timeout_s: float = 5.0) -> bool:
        """Block until every queued record has been written and observed.

        **Required before anything READS the worker's output**, and there are
        exactly two such places: ``_toss_trim_end`` (which writes the trim's
        PROPOSAL, and would otherwise omit the last cycle or two of a session)
        and node teardown. Making the drain the contract is what keeps
        "asynchronous" from meaning "sometimes missing" — the queue is an
        ordering device, not a place work is allowed to be lost.

        Bounded, and returns False on timeout rather than hanging: a wedged
        worker must not be able to hold a session's terminal open.

        **Waits on ``unfinished_tasks``, NOT on ``empty()``** (audit fix,
        2026-08-22). ``Queue.empty()`` goes True the instant the worker ``get``s
        the last item — BEFORE ``_toss_records_run_one`` has done the belt write
        or the trim update — so an ``empty()`` poll returns while the last cycle
        is still being processed, which is the exact opposite of what this method
        promises. Two things went wrong with that: ``_toss_trim_end`` took the
        estimator and wrote its proposal with the final cycle missing ("a
        silently short session", the failure this drain exists to prevent), and
        because ``snapshot['trim']`` carries the estimator BY REFERENCE, the
        proposal could read numpy state the worker was concurrently mutating.
        The queue already calls ``task_done()`` in a ``finally``, so the counter
        that means "handed out AND finished" was there all along."""
        q = self._toss_records_q
        # `Queue.join()` is exactly this wait but has no timeout, and a wedged
        # worker must not hold the terminal open — so the same condition variable
        # is waited on directly, which is what join() does internally.
        with q.all_tasks_done:
            drained = q.all_tasks_done.wait_for(
                lambda: q.unfinished_tasks == 0, timeout=float(timeout_s))
            outstanding = int(q.unfinished_tasks)
        if not drained:
            self.get_logger().warning(
                'toss record worker did not drain within {:.1f} s — {} record(s) '
                'may be missing from the belt and the trim'.format(
                    timeout_s, outstanding))
            return False
        return True

    # ── Layer 2: the session trim (contract C-TOSS-CAL-1 § 3.6, phase 2e) ─────

    def _toss_trim_begin(self, *, goal_id: str) -> None:
        """Start a FRESH session trim for one goal.

        Warm-started from the persistent map's ``anchor.aim_rad`` — the absolute
        aim residual measured at the home anchor, which the map deliberately does
        NOT ship as a correction because it is ``level``-noise contaminated
        (§ 3.2). As a *prior with strength n₀* it is exactly the right object:
        the session overrides it after ~5 admitted tosses if it disagrees, and
        starts from the best available guess if it does not.

        **A DORMANT map contributes NO prior** (audit fix, 2026-08-11). This is
        the one place layer 2 *does* read layer 1, and it is easy to misread
        against :meth:`_toss_aim_for_goal`'s "layer 2 does not depend on layer
        1's dormancy": that statement is about the trim's own MEASUREMENT, which
        is taken this goal against this layer 0 and stays valid. The prior is a
        different object — a number carried in from a map fitted under a
        DIFFERENT levelling layer — and letting it in at n₀ = 4 is exactly the
        D3 double-count the dormancy fence exists to prevent, arriving through
        the back door with a quarter of the fence's authority and none of its
        warning. Dormant ⇒ the estimator starts neutral and learns from scratch,
        which is the same fail-closed posture layer 1 already takes.

        ``speed.k_v`` is refused on the same evidence and for the same reason:
        the dormancy verdict is about the map as a WHOLE — a map that cannot be
        trusted to aim cannot be trusted to scale either, and a partial trust is
        how a fence stops being one.

        Never reuses the previous goal's estimator. The trim is defined as
        common-mode-per-goal; carrying one across a re-``level``, a map reload or
        an operator's coffee break would make it estimate a quantity that
        changed underneath it.
        """
        with self._lock:
            cal = self._toss_cal
            session_id = self._session_id
            live_tilt = self._tilt_map_version
        reason = cal.provenance_mismatch(live_tilt) if cal is not None else ''
        if reason:
            self.get_logger().warning(
                'session trim starts with NO prior — the toss aim map is '
                'DORMANT ({}). Its anchor residual and speed gain were measured '
                'under a different levelling layer, so seeding them would be '
                'the D3 double-count in miniature.'.format(reason))
        usable = cal if (cal is not None and not reason) else None
        anchor = getattr(usable, 'anchor_aim_rad', None) \
            if usable is not None else None
        k_v = getattr(usable, 'speed_k_v', None) if usable is not None else None
        try:
            trim = toss_trim.SessionTrim(anchor_aim_rad=anchor,
                                         speed_k_v_prior=k_v,
                                         session_id=session_id,
                                         goal_id=str(goal_id or ''))
        except toss_trim.TossTrimError as exc:
            # A malformed anchor is a map-validation failure that parse_toss_cal
            # should already have caught; if one gets here the goal runs with NO
            # trim rather than with a guessed one, and says so.
            self.get_logger().error(
                'session trim NOT started ({}) — this goal runs with the '
                'persistent map alone'.format(exc))
            trim = None
        with self._lock:
            self._toss_trim = trim
            self._toss_trim_t0 = time.perf_counter()
            self._toss_trim_belt_warned = False
        self._toss_ilc_session_begin(goal_id=goal_id)

    def _toss_ilc_session_begin(self, *, goal_id: str) -> None:
        """Seed a FRESH layer-3 session common mode for one goal (C1).

        Called from :meth:`_toss_trim_begin` rather than from its two call sites,
        so the two RAM-only per-goal components cannot get out of step: a goal
        that starts a trim starts one of these, always, and there is one place to
        read rather than two to keep matched.

        **Seeded from the artifact, gated at seed, never updated.** The evidence
        the gate reads is between-SESSION evidence, which by definition cannot
        accumulate inside one goal; the fit is what moves it. See
        :class:`toss_ilc.IlcSessionCommonMode` for why there is no CUSUM and no
        freeze-never-zero here, and why that is a property of a seeded-and-held
        component rather than an omission.

        **Dormancy is NOT decided here**, deliberately. This seeds from whatever
        artifact is loaded and :meth:`_toss_aim_for_goal` applies the result only
        inside the branch that already owns the provenance verdict — so a DORMANT
        artifact contributes no prior for exactly the reason it contributes no
        cell, decided once, in one place. Duplicating the verdict here would be a
        second owner of it, which is how two answers to one question begin.

        Instrument-grade: a fault costs the component, never the goal.
        """
        with self._lock:
            ilc = self._toss_ilc
        try:
            session = toss_ilc.IlcSessionCommonMode(ilc, goal_id=str(goal_id))
        except Exception as exc:                               # noqa: BLE001
            self.get_logger().warning(
                'ILC session common mode NOT seeded ({}) — this goal runs with '
                'the per-cell spatial residual alone'.format(exc))
            session = None
        with self._lock:
            self._toss_ilc_session = session
        if session is not None and session.applied:
            self.get_logger().info(session.console_line())

    def _toss_ilc_session_end(self) -> None:
        """Discard the goal's session common mode.

        **Nothing is written.** That is the C1 fence in one line: this component
        exists so that one session's ``level()`` draw can be applied without ever
        being persisted, and a component that wrote anything back would be the
        very failure it was built to prevent.
        """
        with self._lock:
            self._toss_ilc_session = None

    def _toss_trim_observe(self, record: dict, snapshot: dict) -> None:
        """Feed ONE cycle's declaration to the session trim, then print TRIM.

        The declaration is the ingest point on purpose: it is the single object
        that already carries every field the § 3.6.2 guards read, it is minted
        exactly once per cycle terminal, and it is the same shape the offline
        replay consumes — so the live and offline estimators cannot be fed
        different things.

        **Runs on the record WORKER thread since 2026-08-22 (census B6)**, so its
        context arrives as a ``snapshot`` taken on the cycle thread rather than
        being read from ``self`` here. Reading it here would attribute cycle N's
        toss to cycle N+1's pose and aim, because ``_build_toss_cycle`` overwrites
        both between the submit and the execute.

        **Instrument-grade failure handling**, like the record itself: the whole
        body is guarded and a fault costs a measurement, never a teardown.
        """
        trim = snapshot.get('trim')
        pose = snapshot.get('pose')
        flight = snapshot.get('flight')
        aim = snapshot.get('aim')
        t0 = snapshot.get('t0')
        if trim is None:
            return
        try:
            verdict = trim.observe(record)
            lines = trim.console_lines(
                node_xy_mm=(pose[:2] if pose else None),
                map_aim_rad=(aim['map_aim_rad'] if aim else None),
                flight_time_s=flight,
                catch_z_stow_mm=(pose[2] if pose and len(pose) > 2 else None),
                uptime_ms=None,
                elapsed_s=(time.perf_counter() - t0
                           if t0 else None),
                # NEVER applied since 2026-08-21 — the aim estimator is
                # monitor-only (toss_trim.AIM_AUTHORITY, C4). The console has to
                # say so, or an operator reads a converging trim as a converging
                # CORRECTION and goes looking for it in the commanded aim.
                applied=False,
                cycle_index=(record or {}).get('cycle_index'))
        except Exception as exc:                               # noqa: BLE001
            self.get_logger().warning(
                'session trim update failed ({}) — instrument only, the cycle '
                'is unaffected'.format(exc))
            return
        for line in lines:
            self.get_logger().info(line)
        if not verdict.admitted and verdict.reason:
            self.get_logger().info(
                '     trim REFUSED this toss: {} (freeze-never-zero: the '
                'commanded trim is unchanged)'.format(verdict.reason))

    def _toss_trim_end(self) -> None:
        """Discard the goal's trim and write its PROPOSAL to ``temp/logs/``.

        **Never auto-promoted.** Premise P1: promotion into
        ``config/toss_calibration.yaml`` needs the explicit routine and its
        acceptance gates, so this writes a document the map loader structurally
        cannot read (different ``schema``, no ``grid``, no ``aim_rad``) into a
        gitignored directory. A mistaken ``cp`` into ``config/`` is then refused
        loudly by ``parse_toss_cal`` rather than silently applied.

        Best-effort, one WARN per goal: a full disk costs the proposal, never a
        teardown.

        Layer 3's session common mode dies here too, and FIRST — paired with
        :meth:`_toss_trim_begin` starting it, so the two per-goal RAM components
        have one begin site and one end site between them. First because the
        proposal write below is best-effort I/O: a full disk must not be able to
        leave a stale common mode alive into the next goal, which is the one
        thing this component must never do.

        **Drains the record worker FIRST of all** (census B6). The proposal below
        is built from the trim's accumulated state, and the last cycle or two of a
        session are still queued when the goal terminates — publishing a proposal
        that omits them would be a silently short session, which is worse than a
        slow one. The drain is bounded and its failure is a WARN, not a hang.
        """
        self._toss_records_drain()
        self._toss_ilc_session_end()
        with self._lock:
            trim = self._toss_trim
            self._toss_trim = None
        if trim is None or not _RECORD_BELT_DIR:
            return
        try:
            doc = trim.proposal(
                written_at=self._ros_clock_s(),
                git_sha=self._git_sha,
                git_dirty=self._git_dirty,
                toss_cal_version=self._toss_cal_version or None,
                tilt_map_version=self._tilt_map_version or None,
                # The arrival-offset estimator this trim's measurand comes from.
                # It is passed IN rather than read inside toss_trim because that
                # module may not import the map loader (operator decision 7).
                estimator_version=toss_cal.ESTIMATOR_VERSION,
                toss_trim_enabled=bool(
                    self.get_parameter(_TOSS_TRIM_PARAM).value))
            os.makedirs(_RECORD_BELT_DIR, exist_ok=True)
            path = os.path.join(
                _RECORD_BELT_DIR,
                '{}_trim_proposal.yaml'.format(self._session_id))
            with open(path, 'w') as fh:
                yaml.safe_dump(doc, fh, sort_keys=False, default_flow_style=False)
        except Exception as exc:                               # noqa: BLE001
            self.get_logger().warning(
                'session trim proposal NOT written ({}) — instrument only, the '
                'goal is unaffected'.format(exc))
            return
        self.get_logger().info(
            'TRIM proposal written: {} — SESSION-ONLY, promote through '
            'tests/hardware/toss_cal_fit.py, never by copying it into config/'
            .format(path))

    def _belt_toss_record(self, payload: str) -> None:
        """Append one line to ``temp/logs/toss_records_<session>.jsonl``.

        The bag is canonical; this belt exists only so a ``record:=false`` bench
        session still produces a corpus. Same bytes from the same encoder, so a
        belt line and a bag line are the same record.
        """
        if not _RECORD_BELT_DIR:
            return
        try:
            os.makedirs(_RECORD_BELT_DIR, exist_ok=True)
            path = os.path.join(
                _RECORD_BELT_DIR,
                'toss_records_{}.jsonl'.format(self._session_id))
            with open(path, 'a') as fh:
                fh.write(payload + '\n')
        except Exception as exc:                               # noqa: BLE001
            with self._lock:
                warned = self._toss_record_belt_warned
                self._toss_record_belt_warned = True
            if not warned:
                self.get_logger().warning(
                    'toss record belt write failed ({}) — the bag is the '
                    'canonical sink; silencing further belt warnings for this '
                    'goal'.format(exc))

    # ── TossContinuous action (jugglebot/toss_continuous) ──────────────────────

    def _execute_toss_continuous(self, goal_handle):
        """Run a CONTINUOUS self-toss session: ``num_throws`` ordinary tosses back
        to back with a configurable dwell, under ONE busy claim.

        The session adds no capability and commands no motion of its own. Each
        cycle is built by :meth:`_build_toss_cycle` and ticked by
        :meth:`_run_toss_cycle` — the same two methods the single ``Toss`` uses —
        so every precondition, arming order, abort ladder and terminal is the
        validated single-toss one. The outer
        :class:`toss_session.TossSessionSequencer` owns only when the next cycle
        starts, whether it starts, and the accounting."""
        req = goal_handle.request
        catch_pose = (float(req.catch_position.x), float(req.catch_position.y),
                      float(req.catch_position.z))
        height = float(getattr(req, 'throw_height_m', 0.0) or 0.0)
        throw_delay = float(getattr(req, 'throw_delay_s', 0.0) or 0.0)
        vel_scale = float(getattr(req, 'catch_vel_scale', 0.0) or 0.0)
        dwell = float(getattr(req, 'dwell_time_s', 0.0) or 0.0)
        num_throws = int(getattr(req, 'num_throws', 0) or 0)
        # stop_on_miss defaults TRUE in the .action IDL AND here: an omitted or
        # unreadable field must mean STOP, never CONTINUE (operator decision (c),
        # 2026-07-28). A miss leaves a loose ball on the floor under a machine
        # that is about to stroke again.
        stop_on_miss = bool(getattr(req, 'stop_on_miss', True))
        # on_empty_cup carries the SAME doctrine one level further: the IDL
        # default is STOP and `resolve_on_empty_cup` whitelists the single
        # dangerous value, so an omitted, empty, misspelt or older-client field
        # can never start an autonomous BB reload the operator did not ask for.
        on_empty_cup = resolve_on_empty_cup(getattr(req, 'on_empty_cup', ''))
        max_reloads_raw = int(getattr(req, 'max_reloads', 0) or 0)

        result = TossContinuous.Result()
        bad_field = self._invalid_toss_session_goal_field(
            catch_pose, height, throw_delay, vel_scale, dwell,
            max_reloads=max_reloads_raw)
        if bad_field is not None:
            self.get_logger().error(
                f'TossContinuous goal REJECTED_BAD_GOAL({bad_field}): '
                f'catch_position={catch_pose}, throw_height_m={height}, '
                f'num_throws={num_throws}, dwell_time_s={dwell}, '
                f'throw_delay_s={throw_delay}, catch_vel_scale={vel_scale}, '
                f'max_reloads={max_reloads_raw} '
                f'— refusing before anything runs.')
            self._fill_session_result(result, TossSessionResult(
                success=False,
                outcome='REJECTED_BAD_GOAL({})'.format(bad_field)))
            self._log_toss_session_outcome(result)
            goal_handle.abort()
            with self._lock:
                self._goal_claimed = False
            return result

        flight = self._resolve_toss_flight_s(height)
        # A cheap, side-effect-free read of layer 3's arming state — NOT
        # _toss_aim_for_goal, which is THE single per-goal aim lookup and belongs
        # to _build_toss_cycle (tests/motion/test_toss_cal.py pins that call site
        # structurally). All the session's floors need is whether a speed trim is
        # POSSIBLE, so they can be judged fail-closed against the slowest release
        # it could command; the exact trim is the cycle's business.
        with self._lock:
            ilc_loaded = self._toss_ilc is not None
        # Phase E's KNOWN LIMITATION, caught BEFORE a ball flies (num_throws >= 2
        # only — a one-cycle session has no chain). See _predicted_chain_site_mm.
        chain_reachable = True
        if num_throws >= 2:
            site = self._predicted_chain_site_mm(catch_pose, flight)
            if site is not None:
                # Same configured box the sequencer's WORKSPACE gate enforces
                # (YAML toss_workspace_xy_mm) — checking the chain against a
                # different constant than the gate that will refuse cycle 2 is
                # exactly the drift this predictor exists to pre-empt. With the
                # YAML default sitting ABOVE the displacement cap, the 2.07 %
                # centroid-vs-cup divergence no longer binds at the cap edge
                # (the 146.5/147.0 frontier below was measured with box = cap
                # = 150).
                workspace_xy = float(hw.JB_OP_TOSS_WORKSPACE_XY_MM)
                chain_reachable = (abs(site[0]) <= workspace_xy
                                   and abs(site[1]) <= workspace_xy)
                if not chain_reachable:
                    self.get_logger().error(
                        'TossContinuous REJECTED_CHAIN_UNREACHABLE: a CAUGHT '
                        'catch at %s parks the platform CENTROID at (%.2f, %.2f) '
                        'mm, outside the +-%.0f mm planning box, so cycle 2 '
                        'would be rejected WORKSPACE with a ball already thrown '
                        'and caught. Lower |catch_position| (at box = cap = '
                        '150 the measured frontier was <= 146.5 mm chains, '
                        '>= 147.0 does not) or run num_throws=1.'
                        % (catch_pose, site[0], site[1], workspace_xy))

        session = TossSessionSequencer(
            num_throws=num_throws,
            dwell_time_s=dwell,
            throw_delay_s=throw_delay,
            flight_time_s=flight,
            # The RESOLVED catch-speed knob (goal field, else the config
            # default), because since 2026-08-22 the session's dwell floor is
            # partly the CATCH STROKE's tail and that tail is inversely
            # proportional to the armed speed — a slower catch is a longer
            # turnaround, so the floor must see the same scale the arm will.
            catch_vel_scale=(vel_scale if vel_scale > 0.0
                             else float(hw.JB_OP_CATCH_VEL_SCALE_DEFAULT)),
            stop_on_miss=stop_on_miss,
            max_throws=int(hw.JB_OP_TOSS_SESSION_MAX_THROWS),
            dwell_default_s=float(hw.JB_OP_TOSS_SESSION_DWELL_DEFAULT_S),
            dwell_margin_s=float(hw.JB_OP_TOSS_SESSION_DWELL_MARGIN_S),
            chain_site_reachable=chain_reachable,
            # Can layer 3 command a SPEED trim on this goal? It is the only thing
            # that can make a cycle's event_vel differ from the untrimmed
            # vertical closed form the session computes its floors from, and
            # every one of those floors RISES as the speed falls — so an armed
            # ILC has to be judged against the slowest release its apply seam
            # could command (toss_session.floor_event_vel_mps). Read as
            # "enabled AND an artifact is loaded": a dormant-by-provenance
            # artifact still cannot trim, but establishing that costs the whole
            # provenance verdict, and this gate is deliberately cheap and
            # conservative. Costs nothing at the cadence rungs, where the throw
            # envelope refuses the negative side outright.
            ilc_speed_trim_possible=bool(
                self._toss_ilc_enabled() and ilc_loaded),
            on_empty_cup=on_empty_cup,
            max_reloads=(max_reloads_raw if max_reloads_raw > 0
                         else int(hw.JB_OP_TOSS_SESSION_MAX_RELOADS)),
            floor_pause_every=int(hw.JB_OP_TOSS_SESSION_FLOOR_PAUSE_EVERY))
        # Per-session reset of the Layer-1.5 accumulator. It is instrument state,
        # so a leftover read from a previous goal would attribute one session's
        # platform attitude to another's toss.
        with self._lock:
            self._dwell_tilt_reads = []
            self._dwell_tilt_next_at = 0.0
            self._dwell_tilt_degraded = False
            self._toss_record_prev_uid = None
        self._reset_toss_arrival_boundary()
        # A FRESH Layer-2 trim for this session — same reasoning, one layer up:
        # the trim estimates a common mode that is only constant WITHIN a goal.
        self._toss_trim_begin(goal_id=_goal_id_hex(goal_handle))
        session.start(time.perf_counter())
        # The ONE SESSION_CHECKING feedback. The FSM leaves CHECKING inside the
        # same step() that enters it (it either finishes with a reject — and a
        # finished step publishes no feedback — or falls straight through to
        # DWELL), so without this publish the phase string the .action documents
        # is unobservable on the wire: a GUI or an operator waiting for it would
        # wait forever. cycle_index is 0 here, which CS-1's `> 0` filter drops.
        self._publish_session_feedback(goal_handle, session,
                                       SESSION_PHASE_CHECKING)
        # The session ceiling needs the PER-CYCLE ceiling, which reads a
        # sequencer's own budgets. Build a THROWAWAY sequencer — never started,
        # never installed, never stepped — purely so _toss_deadline_s reads the
        # SAME fields it will read for the real cycles. A second copy of that
        # formula here is exactly how a ceiling drifts INSIDE a legitimate flight
        # window, and the timeout path is the one that SAFE_ABORTs.
        budget_seq = TossSequencer(
            catch_pose_stow_mm=catch_pose, flight_time_s=flight,
            throw_delay_s=session.throw_delay_s, event_vel_mps=1.0)
        max_session_s = _toss_session_deadline_s(
            session, _toss_deadline_s(budget_seq),
            reload_budget_s=_reload_interlude_budget_s(session))

        try:
            t_start = time.perf_counter()
            while rclpy.ok():
                now = time.perf_counter()
                if goal_handle.is_cancel_requested and not session.cycle_live:
                    # Honoured immediately between cycles: nothing is armed and
                    # nothing is airborne (the previous cycle's own terminal
                    # already left the machine where it belongs), so there is
                    # nothing to safe and the session issues no motion here.
                    self._finish_session(
                        result, session, 'ABORTED_CANCELLED')
                    goal_handle.canceled()
                    return result
                decision = session.step(now)
                if decision.done:
                    self._fill_session_result(result, decision.result)
                    self._log_toss_session_outcome(result)
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                    elif result.success:
                        goal_handle.succeed()
                    else:
                        goal_handle.abort()
                    return result
                if decision.action == SESSION_ACTION_RELOAD:
                    # The auto-reload interlude (§ 3.9). Entered ONLY from a
                    # REJECTED_NO_BALL terminal, i.e. from a machine that
                    # commanded nothing — every rung below is an existing,
                    # validated mechanism, and every refusal names itself.
                    self._publish_session_feedback(goal_handle, session,
                                                   SESSION_PHASE_RELOAD)
                    ok, stop_code, attempts = self._run_reload_interlude(
                        session,
                        cancel_now_fn=(lambda: goal_handle.is_cancel_requested))
                    session.note_reload_result(ok, attempts=attempts,
                                               stop_code=stop_code)
                    continue
                if decision.action == SESSION_ACTION_START_CYCLE:
                    seq = self._build_toss_cycle(
                        catch_pose, flight, session.throw_delay_s, vel_scale,
                        delay_is_cadence=True)
                    # The cadence clamp (C-POSSESS-1 § 3.4). Set HERE and not
                    # inside _build_toss_cycle because the dwell is the SESSION's
                    # number and a single Toss has none — passing the session in
                    # would give the cycle builder a parameter that is None on
                    # its other caller and mean "no clamp" by accident rather
                    # than by statement.
                    self._set_toss_next_cycle_perf(seq, session)
                    self._open_toss_record(
                        action='toss_continuous',
                        goal_id=_goal_id_hex(goal_handle),
                        cycle_index=session.cycle_index,
                        catch_pose=catch_pose, flight=flight,
                        throw_delay=session.throw_delay_s, vel_scale=vel_scale,
                        raw_goal={'throw_height_m': height,
                                  'throw_delay_s': throw_delay,
                                  'catch_vel_scale': vel_scale,
                                  'on_empty_cup': on_empty_cup,
                                  'max_reloads': max_reloads_raw},
                        session=session,
                        reload_settle=bool(session.cycle_reload_settle),
                        retry=bool(session.cycle_is_retry))
                    try:
                        cycle_result, exit_kind = self._run_toss_cycle(
                            seq,
                            deadline_s=_toss_deadline_s(seq),
                            cancel_now_fn=(
                                lambda n, _s=seq: (
                                    goal_handle.is_cancel_requested
                                    and not self._toss_cancel_deferred(_s, n))),
                            feedback_fn=(
                                lambda phase: self._publish_session_feedback(
                                    goal_handle, session, phase)))
                    finally:
                        # One cycle's node state never outlives its cycle, even
                        # on the raising path (the session's own finally is the
                        # belt).
                        self._clear_toss_cycle_state()
                    # The session schedules off the cycle's SCHEDULED instants,
                    # never an observed landing: the observed one is exactly what
                    # the tracker is least trustworthy about, and a cadence must
                    # not inherit that noise.
                    # The LIVE cup state at the cycle's terminal, read once and
                    # passed in. It licenses exactly one decision — whether an
                    # ABORTED_NO_RELEASE may be retried (operator decision 6) —
                    # and it is read HERE, after the cycle has torn down, so it
                    # describes the cup the retry would stroke over rather than
                    # some earlier moment's belief.
                    session.note_cycle_result(
                        cycle_result, seq.t_release,
                        seq.t_release + float(seq.flight_time_s),
                        ball_evidence=self._ball_sensor.evidence(
                            time.perf_counter()))
                    if exit_kind != 'fsm':
                        # NODE-level exits inside a cycle end the SESSION too.
                        # The cycle has already safed and logged; terminalise
                        # with the accounting preserved rather than returning an
                        # empty result.
                        outcome = {'cancel': 'ABORTED_CANCELLED',
                                   'timeout': 'ABORTED_TIMEOUT',
                                   'shutdown': 'ABORTED_SHUTDOWN'}[exit_kind]
                        self._finish_session(result, session, outcome)
                        if exit_kind == 'cancel':
                            goal_handle.canceled()
                        elif exit_kind == 'timeout':
                            goal_handle.abort()
                        # SHUTDOWN deliberately terminalises NOTHING on the
                        # handle — the single Toss does the same. rclpy is
                        # tearing down, and a goal-status transition on a dying
                        # executor can itself raise, which would replace a clean
                        # shutdown with a spurious ABORTED_EXCEPTION.
                        return result
                    # Let the next step() adjudicate stop_on_miss / COMPLETED.
                    continue
                self._publish_session_feedback(
                    goal_handle, session, decision.phase)
                # ── Layer 1.5, and its ONLY call site ──
                # Structurally confined to the quiescent dwell: _run_toss_cycle
                # is a BLOCKING call, so no iteration of this loop happens while
                # a cycle is live — there is no reachable path from PREPARE or
                # THROW to a tilt read. The extra `not session.cycle_live` is a
                # belt, not the mechanism (test_dwell_tilt_reads_have_one_call
                # _site_and_it_is_the_quiescent_dwell pins the structure).
                if (decision.action == SESSION_ACTION_NONE
                        and decision.phase == SESSION_PHASE_DWELL
                        and not session.cycle_live):
                    self._maybe_read_dwell_tilt(now, session)
                if now - t_start > max_session_s:
                    # Reachable only BETWEEN cycles (a cycle's own ceiling is
                    # enforced inside _run_toss_cycle), where nothing is armed
                    # and nothing is airborne — so unlike the per-cycle timeout
                    # this path needs no safing and commands nothing.
                    self._finish_session(result, session, 'ABORTED_TIMEOUT')
                    goal_handle.abort()
                    return result
                time.sleep(_TICK_S)
            # rclpy shutting down.
            self._finish_session(result, session, 'ABORTED_SHUTDOWN')
            return result
        except Exception:
            # Any fault: the cycle-level handler has already safed the robot if
            # a cycle was live. Preserve the accounting, abort the goal, re-raise
            # so the executor's own error path still sees the fault.
            self._finish_session(result, session, 'ABORTED_EXCEPTION')
            try:
                goal_handle.abort()
            except Exception:  # noqa: BLE001
                self.get_logger().error(
                    'ABORTED_EXCEPTION: goal_handle.abort() itself failed')
            raise
        finally:
            self._clear_toss_cycle_state()
            # The trim is per GOAL, so it dies here — with its proposal written.
            # In the `finally` deliberately: a cancelled or aborted goal's trim
            # is exactly the one an operator wants to read.
            self._toss_trim_end()
            with self._lock:
                self._goal_claimed = False

    @staticmethod
    def _invalid_toss_session_goal_field(catch_pose, throw_height, throw_delay,
                                         vel_scale, dwell, max_reloads=0):
        """Return the name of the first invalid TossContinuous goal numeric, or
        None. The six shared with ``Toss`` route through
        :meth:`_invalid_toss_goal_field` so the two actions cannot disagree about
        what a valid toss goal is; ``dwell_time_s`` gets the same treatment (0.0
        is the only "use the default" sentinel, so a negative is a sign typo).
        ``num_throws`` is an integer and is range-checked by the session FSM
        (REJECTED_NUM_THROWS), which is where its bound lives.

        ``max_reloads`` carries the same sign-typo doctrine: 0 is the only "use
        the config default" sentinel, so a negative is refused BY NAME rather
        than coerced. Coercing it would be the worst of both worlds — the
        operator asked for something the machine cannot represent, and silently
        substituting a 3-ball budget for it is exactly the class of quiet wrong
        answer the numerics gate exists to eliminate."""
        shared = ReloadCoordinatorNode._invalid_toss_goal_field(
            catch_pose, throw_height, throw_delay, vel_scale)
        if shared is not None:
            return shared
        if not math.isfinite(dwell) or dwell < 0.0:
            return 'dwell_time_s'
        if int(max_reloads) < 0:
            return 'max_reloads'
        return None

    # ── The auto-reload interlude (§ 3.9) ─────────────────────────────────────

    def _wait_out_seat_edge_band(self, session, *, cancel_now_fn=None) -> bool:
        """Hold until the previous cycle's seat-edge band has elapsed. Commands
        nothing — it waits. Returns False iff a cancel was honoured.

        CENSUS D4/F3, and the reason the interlude cannot trust the terminal that
        summoned it. ``REJECTED_NO_BALL`` is minted at CHECKING, and CHECKING runs
        at ``landing + (dwell - throw_delay)``. The physical seat edge lands
        **+87.6…+554.7 ms** after the announced landing (n=33, four post-FW-14
        bags; it was +137…+798 ms, n=35, until the 2026-08-24 re-measure). Those
        two numbers cross: below roughly a 1.0 s dwell, CHECKING
        reads the cup BEFORE the ball has finished arriving in it, so a **good
        catch** mints ``REJECTED_NO_BALL`` — and with ``on_empty_cup: RELOAD``
        that route asks BallButler to throw a **second ball at a full cup**.

        So the interlude's own cup read is re-taken here, after
        ``JB_BD_ARRIVAL_WINDOW_S`` past the previous cycle's SCHEDULED landing —
        the constant that already means "how long after the predicted landing a
        seat edge may still arrive".

        **This wait did NOT follow the 2026-08-24 band re-measure, and the
        sentence that used to promise it would was wrong about the mechanism.**
        ``JB_BD_ARRIVAL_WINDOW_S`` is a ``hardware_config.yaml`` knob (1.50 s)
        sized WITH margin ABOVE the band, not a derivation of
        ``ball_possession.ARRIVAL_BAND_MAX_S``; a change to the constant cannot
        reach it. What the re-measure did was widen the margin this wait already
        carried — 1.9x the old +798 ms ceiling, **2.7x** the new +554.7 ms one —
        so the wait is now conservative rather than stale. Shrinking it is a
        separate YAML decision with its own blast radius (the same knob sizes the
        live ARRIVAL search window and the reload budget), and it is deliberately
        NOT taken here.

        Nothing is armed and nothing is airborne while this runs: the interlude is
        only ever entered from ``REJECTED_NO_BALL``, the one toss terminal whose
        own terminal action was ``ACTION_NONE``."""
        landing = float(getattr(session, 'last_landing_perf', float('nan')))
        if not math.isfinite(landing):
            return True                    # nothing has landed yet — nothing to wait for
        deadline = landing + float(hw.JB_BD_ARRIVAL_WINDOW_S)
        while rclpy.ok() and time.perf_counter() < deadline:
            if cancel_now_fn is not None and cancel_now_fn():
                return False
            time.sleep(_TICK_S)
        return True

    def _reload_interlude_gate(self, session, *, cancel_now_fn=None):
        """The node's half of the interlude precondition gate — the rungs that
        are OBSERVATIONS rather than counters (the session owns budget + floor,
        and has already passed them before ``SESSION_ACTION_RELOAD`` is emitted).

        Returns a named stop code, or None. **Nothing moves on any refusal**: the
        gate runs before the recentre and before the reload FSM is even built, so
        a refused interlude leaves the machine exactly where the rejected cycle
        left it — which is quiescent, because the interlude is only ever entered
        from ``REJECTED_NO_BALL``.

        Ordered so the code names the rung the operator must actually fix."""
        # (1) The RUNTIME prerequisite. Phase 1 shipped
        # toss_require_ball_evidence TRUE, but the operator's total-bypass escape
        # hatch must not silently re-open the dry-stroke path: with the gate off,
        # CHECKING passes on an empty cup, a drop produces a silent empty stroke,
        # and the session would be "reloading" around tosses that never happened.
        if not bool(hw.JB_OP_TOSS_REQUIRE_BALL_EVIDENCE):
            return 'STOPPED_BALL_EVIDENCE_DISABLED'
        now = time.perf_counter()
        # (2) BB ready. Read through the SAME freshness-gated snapshot the reload
        # FSM's own CHECKING uses, so the interlude cannot admit a state the
        # sequence would immediately reject.
        obs = self._build_observations(now)
        if not obs.bb_connected or obs.bb_state != BB_STATE_IDLE:
            return 'STOPPED_BB_NOT_READY'
        # (3) The BB fail-open boot fence (consumer side — see
        # _bb_ball_in_hand_observed_false). A BB that has never published a FALSE
        # has never had its GPIO speak, and its `true` is a boot default.
        with self._lock:
            bb_verified = self._bb_ball_in_hand_observed_false
        if not bb_verified:
            return 'STOPPED_BB_UNVERIFIED'
        # (4) The cup — a CONFIRMED-EMPTY read, taken after the seat-edge band
        # and from the SETTLED query. Two changes on 2026-08-21, both because
        # this is the one refusal in the node that answers by COMMANDING (an
        # autonomous BB throw), not by refusing:
        #
        #   * the band (census D4/F3) — see _wait_out_seat_edge_band. Below a
        #     ~1.2 s dwell, CHECKING's REJECTED_NO_BALL fires on GOOD catches
        #     because it reads the cup before the ball has finished arriving, so
        #     the terminal that summoned the interlude is not evidence of
        #     anything and the interlude must look for itself, later;
        #   * `evidence_settled`, not `evidence` (C-POSSESS-1 § 3.5) — the live
        #     query now reads the RAW bit, which is right everywhere a wrong
        #     answer REFUSES and wrong here, where a carry-flicker over a seated
        #     ball would put a second ball into a full cup. The settled query
        #     needs both bits to agree and answers UNKNOWN when they do not,
        #     which this gate already refuses on without moving anything.
        #
        # UNKNOWN refuses — a dead (or disagreeing) sensor must not license an
        # autonomous BB throw at a cup nobody can see into. SEATED refuses too,
        # and gets its own code: after the band has elapsed a SEATED read means
        # the cup is genuinely loaded, i.e. the cycle that reported NO_BALL was
        # mislabelled. (§ 3.9 names one sensor code; splitting it is the one
        # deviation here, and it is in the fail-closed direction.)
        if not self._wait_out_seat_edge_band(session,
                                             cancel_now_fn=cancel_now_fn):
            return 'STOPPED_RELOAD_CANCELLED'
        evidence = self._ball_sensor.evidence_settled(time.perf_counter())
        if evidence == EVIDENCE_SEATED:
            return 'STOPPED_CUP_NOT_EMPTY'
        if evidence != EVIDENCE_EMPTY:
            return 'STOPPED_SENSOR_UNKNOWN'
        return None

    def _recentre_for_reload(self) -> bool:
        """``go_home`` + VERIFIED arrival — rung 2 of the interlude ladder.

        The reload catch is hard-fixed at the workspace centre and the reload
        never pre-positions, so ``reload_sequencer._step_checking`` refuses an
        off-centre park (``REJECTED_NOT_CENTERED``) — see that gate's own comment
        for why it refuses rather than auto-returning. With
        ``toss_stay_at_pose_on_caught`` true, "parked 150 mm off centre" is the
        ROUTINE state after a CAUGHT cycle, so the session must recentre itself.

        ``_go_home()`` returns on the service ACK at plan-INSTALL, not on
        arrival, which is exactly the trap the MISS-cleanup floor already
        documents. So this waits the profile out and then CONFIRMS, against a
        FRESH ``trajectory/commanded_position``, that the platform is inside
        ``_RELOAD_CENTERED_TOL_MM``. A timeout is ``STOPPED_RECENTRE_FAILED`` and
        NEVER a reload attempt: asking BB to throw at a platform we could not
        prove is centred is the mid-flight-rejection failure the centred gate
        exists to make impossible."""
        if not self._go_home():
            self.get_logger().error(
                'reload interlude: trajectory/go_home dispatch FAILED — not '
                'attempting a reload from an unproven pose')
            return False
        t0 = time.perf_counter()
        deadline = t0 + GO_HOME_DURATION_S + _RECENTRE_VERIFY_PAD_S
        while rclpy.ok():
            now = time.perf_counter()
            if now >= t0 + GO_HOME_DURATION_S and self._platform_centered(now):
                return True
            if now >= deadline:
                pos = self._live_commanded_position(now)
                self.get_logger().error(
                    'reload interlude: recentre NOT verified within %.2f s — '
                    'commanded position %s vs tolerance %.2f mm (a stale or '
                    'absent trajectory/commanded_position reads as NOT centred, '
                    'by design)'
                    % (GO_HOME_DURATION_S + _RECENTRE_VERIFY_PAD_S,
                       'unknown' if pos is None else '(%.1f, %.1f) mm'
                       % (pos[0], pos[1]), _RELOAD_CENTERED_TOL_MM))
                return False
            time.sleep(_TICK_S)
        return False

    def _run_reload_interlude(self, session, *, cancel_now_fn=None):
        """Run ONE auto-reload interlude. Returns ``(ok, stop_code, attempts)``.

        The ladder is § 3.9 verbatim, and every rung of it is an EXISTING
        validated mechanism — the interlude invents no motion primitive:

          1. precondition gate (:meth:`_reload_interlude_gate`) — nothing moves;
          2. ``go_home`` + VERIFIED arrival (:meth:`_recentre_for_reload`);
          3. the reload FSM, driven through the SAME :meth:`_step_sequence` the
             shipping ``Reload`` action drives, with the SAME abort ladder and
             the SAME terminal actions;
          4. settle to the MISS-cleanup floor past the reload's landing, exactly
             as the MISS path already does;
          5. the caller flags the next cycle ``RELOAD_SETTLE`` (guard G10).

        Rung 3 may run more than once: the BB firmware's
        ``THROW_ABORTED_NOT_SETTLED`` (BB not positioned in time, ball never
        left) is retried within the session's remaining budget, because it is a
        known BB-side defect rather than a Jugglebot fault. The retry is keyed on
        that CODE — not on "the reload did not catch" — so it cannot swallow the
        BB fail-open boot bug or any real BB fault, and the log line names it.

        The whole loop deliberately does NOT own the goal handle: a session's
        interlude terminal is not the session's terminal."""
        stop = self._reload_interlude_gate(session, cancel_now_fn=cancel_now_fn)
        if stop is not None:
            self.get_logger().warning(
                'reload interlude REFUSED %s — nothing moved (reloads %d/%d, '
                'floor %d)'
                % (stop, session.reloads_used, session.max_reloads,
                   session.floor_balls))
            return False, stop, 0
        if not self._recentre_for_reload():
            return False, 'STOPPED_RECENTRE_FAILED', 0

        attempts = 0
        # The FSM refuses to emit SESSION_ACTION_RELOAD with no budget left, so
        # this is a belt — and it fails CLOSED rather than granting a courtesy
        # attempt, because "at least one" is exactly how a fence stops being one.
        budget = int(session.reload_budget_remaining)
        if budget <= 0:
            return False, OUTCOME_STOPPED_RELOAD_BUDGET, 0
        while attempts < budget:
            attempts += 1
            t_attempt = time.perf_counter()
            outcome = self._run_one_reload_attempt(cancel_now_fn=cancel_now_fn)
            if outcome is None:
                return False, 'STOPPED_RELOAD_CANCELLED', attempts
            if outcome.success:
                self.get_logger().info(
                    'reload interlude CAUGHT on attempt %d/%d — next cycle is '
                    'flagged RELOAD_SETTLE and excluded from the fit'
                    % (attempts, budget))
                return True, None, attempts
            code = self._bb_throw_outcome_since(t_attempt)
            if code == _BB_NOT_SETTLED_CODE and attempts < budget:
                self.get_logger().warning(
                    'reload interlude attempt %d/%d ended %s with BB reporting '
                    '%s — BB was not positioned in time, so no ball ever left '
                    'it. Retrying within the reload budget (this retry is '
                    'targeted at that ONE code; every other BB failure stops '
                    'the session).'
                    % (attempts, budget, outcome.outcome, _BB_NOT_SETTLED_CODE))
                continue
            if code == _BB_NOT_SETTLED_CODE:
                self.get_logger().error(
                    'reload interlude EXHAUSTED the budget on %s (%d attempts)'
                    % (_BB_NOT_SETTLED_CODE, attempts))
                return False, OUTCOME_STOPPED_RELOAD_BUDGET, attempts
            self.get_logger().error(
                'reload interlude attempt %d/%d ended %s%s — stopping the '
                'session (only %s is retryable)'
                % (attempts, budget, outcome.outcome,
                   '' if code is None else ' (BB reported %s)' % code,
                   _BB_NOT_SETTLED_CODE))
            return (False, 'STOPPED_RELOAD_{}'.format(outcome.outcome),
                    attempts)
        return False, OUTCOME_STOPPED_RELOAD_BUDGET, attempts

    @staticmethod
    def _reload_cancel_deferred(seq, now: float) -> bool:
        """Per-phase cancellation for the auto-reload interlude. True ⇒ DEFER:
        keep ticking the FSM to its own terminal and resolve the cancel there.

        THE TRANSPOSE of :meth:`_toss_cancel_deferred`, and it closes the HIGH the
        Phase-2 audit left open. The toss path already refuses to honour a cancel
        while a ball is airborne, for a reason that is not toss-specific:

            aborting a catch mid-flight drops a ball on the robot.

        The interlude's ball is thrown by BallButler and is exactly as airborne.
        Until 2026-08-21 a session cancel during an interlude was honoured on the
        very next tick, at any phase — including with a BB ball in the air — and
        the honoured path runs ``_safe_on_early_exit`` → ``_safe_abort``, i.e. it
        RETRACTS THE HAND out from under the incoming ball. Same hazard, same
        discipline:

          - CHECKING / PREPARING — honour now. Nothing has been commanded to BB;
            the early-exit safing retracts and lowers the latch iff anything
            armed, and the cup is loaded or not but nothing is falling;
          - AIMING and every later phase — DEFER. AIMING is entered by dispatching
            ``bb/throw_at_target``, and BB's countdown cannot be aborted
            (``_enter_preparing``'s own docstring says so), so from that dispatch
            onward a ball may leave the Butler at any moment. There is no
            release instant to compare against before the announcement lands —
            unlike the toss, which owns its own ``t_release`` — so the boundary is
            the dispatch itself rather than a cutoff around it.

        Deferring is bounded: every later phase carries its own deadline
        (``_announcement_deadline``, ``_settle_deadline``) and the whole attempt
        sits under ``_sequence_deadline_s``. A BB that never throws
        (``THROW_ABORTED_NOT_SETTLED``) terminates on the announcement grace, not
        on this."""
        return seq.phase not in (RELOAD_PHASE_CHECKING, RELOAD_PHASE_PREPARING)

    def _run_one_reload_attempt(self, *, cancel_now_fn=None):
        """Drive ONE reload sequence to its terminal. Returns the
        :class:`ReloadResult`, or None if a cancel was honoured.

        Deliberately a SEPARATE loop from :meth:`_execute_reload` rather than a
        refactor of it. What must be shared is the MECHANISM — the FSM, the
        observation builder, the action dispatch and the terminal safing — and
        all of that is shared, through :meth:`_step_sequence` and
        :meth:`_safe_on_early_exit`. What is NOT shared is the ~15 lines of
        goal-handle bookkeeping, and pulling those apart would edit a
        hardware-validated shipping path (four sittings of evidence) to serve a
        caller that has no goal handle. The duplication is the loop shell only,
        and the drift that matters — a rung added to the reload ladder — lands in
        ``_step_sequence`` where BOTH callers see it."""
        seq = ReloadSequencer(catch_point_mm=self._catch_point_mm,
                              throw_delay_s=0.0)
        seq.start(time.perf_counter())
        with self._lock:
            self._active_seq = seq
            self._announced_ball_id = None
            self._announced_id_untagged = False
            self._preexisting_flight_ids = set()
            self._catch_vel_scale = float(hw.JB_OP_CATCH_VEL_SCALE_DEFAULT)
        self._possession_logged = set()
        max_sequence_s = _sequence_deadline_s(seq)
        cancel_seen = False
        deferred_logged = False
        try:
            t_start = time.perf_counter()
            while rclpy.ok():
                now = time.perf_counter()
                if cancel_now_fn is not None and cancel_now_fn():
                    cancel_seen = True
                    if not self._reload_cancel_deferred(seq, now):
                        self.get_logger().warning(
                            'reload interlude CANCEL honoured NOW in %s — '
                            'nothing was commanded to BallButler yet, so no '
                            'ball can be in the air (deferred-cancel discipline, '
                            'C-POSSESS-1 § 3.6 / _reload_cancel_deferred).'
                            % seq.phase)
                        self._safe_on_early_exit(seq)
                        self._log_reload_outcome(
                            ReloadResult(False, 'ABORTED_CANCELLED'))
                        return None
                    if not deferred_logged:
                        deferred_logged = True
                        # ONE line per attempt, and it names the phase: an
                        # operator whose stop button appears not to have worked
                        # must be able to see WHY from the log, not infer it.
                        self.get_logger().warning(
                            'reload interlude CANCEL DEFERRED in %s — the throw '
                            'is committed to BallButler and its countdown cannot '
                            'be aborted, so a ball may be in the air. Honouring '
                            'now would retract the hand from under it. The '
                            'cancel resolves at this interlude terminal (same '
                            'discipline as _toss_cancel_deferred).' % seq.phase)
                decision = self._step_sequence(seq, now, None)
                if decision.done:
                    self._log_reload_outcome(decision.result)
                    return decision.result
                if now - t_start > max_sequence_s:
                    self._safe_on_early_exit(seq)
                    r = ReloadResult(False, 'ABORTED_TIMEOUT')
                    self._log_reload_outcome(r)
                    return r
                time.sleep(_TICK_S)
            self._safe_on_early_exit(seq)
            r = ReloadResult(False, 'ABORTED_SHUTDOWN')
            self._log_reload_outcome(r)
            return r
        finally:
            with self._lock:
                self._active_seq = None
            # Rung 4 — SETTLE. The reload's own terminal action (RECENTER on a
            # catch, SAFE_ABORT otherwise) dispatches on SERVICE ACKS, so it
            # returns while the go_home profile is still traversing. The MISS
            # path already carries exactly this floor and for exactly this
            # reason; reusing the constant is what keeps the two from drifting.
            # SKIPPED on a cancel: the floor exists to protect the NEXT cycle,
            # and a cancelled session has none — waiting it out would only make
            # the operator's cancel look 2.9 s slower than it is. (The safing has
            # already run above; this is a wait, not a teardown step.)
            #
            # `cancel_seen`, not "cancel honoured": a DEFERRED cancel ends the
            # session at this same terminal, so it has no next cycle either, and
            # holding the floor for it would add the deferral's cost to the
            # operator's stop TWICE.
            if not cancel_seen:
                self._settle_after_reload()

    def _settle_after_reload(self) -> None:
        """Hold for the MISS-cleanup floor so the next cycle does not start
        inside the reload's own teardown. Commands nothing — it waits."""
        deadline = time.perf_counter() + DEFAULT_SESSION_MISS_CLEANUP_S
        while rclpy.ok() and time.perf_counter() < deadline:
            time.sleep(_TICK_S)

    # ── Layer 1.5 — the dwell inclinometer covariate (§ 3.10) ─────────────────

    def _maybe_read_dwell_tilt(self, now: float, session) -> None:
        """Take at most ONE ``get_platform_tilt`` read, if one is due and the
        quiescent dwell has room for it. COVARIATE ONLY: nothing reads the result
        except the record writer.

        Two hard rules, in priority order (§ 3.10):

        1. **Reads never overlap PREPARE→THROW.** This is structural, not a
           check: the only call site is the session loop's quiescent-dwell
           branch, and ``_run_toss_cycle`` blocks that loop for a cycle's whole
           life.
        2. **A tight dwell degrades the READ COUNT, never the throw.** The read
           is refused unless it can finish, worst case, a full
           ``_DWELL_TILT_GUARD_S`` before the next cycle is due to start.

        The arithmetic bites at the shipped defaults and is stated rather than
        discovered later: the QUIESCENT dwell is ``dwell_time_s − throw_delay_s``
        minus the CAUGHT-verdict latency, because the rest of the nominal dwell
        is the next cycle's own throw countdown. At dwell 6.0 / delay 5.0 that is
        ~0.7 s, so 8 reads at 0.15 s do NOT fit and the record honestly reports
        ``dwell_tilt_degraded`` with n of 1-2. A longer ``dwell_time_s`` (~7.5 s+)
        buys the full schedule."""
        want = int(getattr(hw, 'JB_OP_TOSS_SESSION_DWELL_TILT_READS', 0) or 0)
        if want <= 0:
            return
        gap = float(getattr(hw, 'JB_OP_TOSS_SESSION_DWELL_TILT_GAP_S', 0.15))
        with self._lock:
            taken = len(self._dwell_tilt_reads)
            next_at = self._dwell_tilt_next_at
        if taken >= want:
            return
        if now < next_at:
            return
        # Rule 2 — the throw's schedule is never a function of the covariate.
        room = float(session.next_cycle_at) - now
        if room < (_DWELL_TILT_READ_TIMEOUT_S + _DWELL_TILT_GUARD_S):
            with self._lock:
                self._dwell_tilt_degraded = True
            return
        reading = self._read_platform_tilt()
        with self._lock:
            self._dwell_tilt_next_at = now + gap
            if reading is None:
                # A failed read is a lost data point and nothing else. It still
                # advances the schedule, so a dead inclinometer cannot turn the
                # dwell into a retry storm against a service that blocks the
                # Platform-Teensy loop.
                self._dwell_tilt_degraded = True
            else:
                self._dwell_tilt_reads.append(
                    (now, float(reading[0]), float(reading[1])))

    def _read_platform_tilt(self):
        """ONE ``get_platform_tilt`` call -> ``(rx, ry)`` rad, or None.

        NaN is the service's documented failure shape (the bridge returns
        ``[nan, nan]`` when the relay read fails), so it maps to None here rather
        than into the record — a NaN covariate that reached the corpus would be
        indistinguishable from a real reading of zero in any downstream fit that
        forgot to check."""
        try:
            if not self._tilt_cli.service_is_ready():
                return None
            future = self._tilt_cli.call_async(GetTiltReadingService.Request())
            resp = self._wait_future(future,
                                     timeout_s=_DWELL_TILT_READ_TIMEOUT_S)
            if resp is None:
                return None
            tilt = list(getattr(resp, 'tilt_xy', []) or [])
            if len(tilt) < 2:
                return None
            rx, ry = float(tilt[0]), float(tilt[1])
            if not (math.isfinite(rx) and math.isfinite(ry)):
                return None
            return rx, ry
        except Exception as exc:                               # noqa: BLE001
            self.get_logger().warning(
                'dwell tilt read failed ({}) — covariate only, the session is '
                'unaffected'.format(exc))
            return None

    @staticmethod
    def _dwell_tilt_fields(reads, t_release_perf) -> dict:
        """The Layer-1.5 record block from a cycle's dwell reads. Pure."""
        n = len(reads)
        out = {'dwell_tilt_n': n}
        if n == 0:
            return out
        xs = [r[1] for r in reads]
        ys = [r[2] for r in reads]
        mean_x = sum(xs) / n
        mean_y = sum(ys) / n
        out['dwell_tilt_rad'] = [mean_x, mean_y]
        if n >= 2:
            # POPULATION sd: these are the reads that were actually taken, not a
            # sample from which a wider population is being inferred.
            out['dwell_tilt_sd_rad'] = [
                math.sqrt(sum((v - mean_x) ** 2 for v in xs) / n),
                math.sqrt(sum((v - mean_y) ** 2 for v in ys) / n)]
        out['dwell_tilt_span_s'] = float(reads[-1][0] - reads[0][0])
        if t_release_perf is not None and math.isfinite(float(t_release_perf)):
            # THE field that proves rule 1 held on this toss: positive means the
            # last read finished before the release. A corpus can therefore audit
            # the invariant instead of trusting this docstring.
            out['dwell_tilt_last_read_to_release_s'] = float(
                float(t_release_perf) - reads[-1][0])
        return out

    def _predicted_chain_site_mm(self, catch_pose, flight):
        """The platform CENTROID a CAUGHT catch at ``catch_pose`` will leave the
        machine holding — i.e. the throw site cycle 2 of a session will read off
        ``trajectory/commanded_position``. Returns ``(x, y)`` STOW mm, or None
        when it cannot be predicted.

        This makes the centroid-vs-cup divergence checkable BEFORE anything
        moves. The catch deliberately parks the CENTROID a cup-swing outside the
        nominated B so the CUP lands ON B, and the wire publishes the centroid —
        so near the planning-box edge a chained session would throw one ball,
        catch it, and then refuse cycle 2 ``REJECTED_WORKSPACE`` with the
        platform parked off-box and the ball in the cup. Measured through this
        same policy at the old box = cap = 150 (2026-07-29): B_x 146.0 -> A_x
        149.017 (chained), B_x 147.0 -> A_x 150.038 (did not). The residual then
        COLLAPSES (cycle 3: 145.938, cycle 4: 146.001), so cycle 2 is the only
        one at risk — and at the shipped ``toss_workspace_xy_mm`` = 160 box
        (2026-08-14) no valid B trips the check at all; it re-binds only if the
        box is configured below cap × ~1.03.

        Single-sourced through ``self._toss_catch_policy`` —
        ``predicted_catch_command`` is the SAME object and method the deferred
        A->B reach publishes from, so this prediction cannot drift from the pose
        the machine will actually be commanded to.

        None (⇒ the check is SKIPPED, not failed) in three cases, each because a
        reject here would mis-route the operator: Tier 8a never reads a throw
        site at all; an unknown live pose is already ``REJECTED_POSE_UNKNOWN`` on
        cycle 1; and a tilt-clamp / policy refusal is already the cycle's own
        loud verdict."""
        if str(hw.JB_OP_TOSS_TIER) != TIER_8B:
            return None
        live = self._live_commanded_position(time.perf_counter())
        if live is None:
            return None
        try:
            release = compute_release_state_tilted(
                catch_pose, flight,
                throw_site_xy_mm=(float(live[0]), float(live[1])))
        except ThrowTiltInfeasible:
            return None
        fields = build_announcement_fields(release, throw_time_s=0.0)
        cmd = self._toss_catch_policy.predicted_catch_command(
            np.asarray(fields['landing_position'], dtype=float),
            np.asarray(fields['landing_velocity'], dtype=float),
            float(flight))
        if cmd is None:
            return None
        # target_pos is GLOBAL mm; its xy IS the STOW xy (the two frames differ
        # in z only, by GEOM_INITIAL_HEIGHT_MM), which is what the +-150 mm
        # planning box and the displacement cap are both applied to.
        return (float(cmd.target_pos[0]), float(cmd.target_pos[1]))

    def _publish_session_feedback(self, goal_handle, session, phase) -> None:
        """Session feedback: the live cycle index, the phase (the CYCLE's Toss
        phase verbatim while one runs, else the session's own), and the running
        catch count — so a watched session is scoreable live rather than only at
        the terminal."""
        if goal_handle is None:
            return
        fb = TossContinuous.Feedback()
        fb.cycle_index = int(session.cycle_index)
        fb.phase = str(phase)
        fb.catches_confirmed = int(session.catches_confirmed)
        goal_handle.publish_feedback(fb)

    def _finish_session(self, result, session, outcome) -> None:
        """Terminalise a session from the NODE level, preserving the per-cycle
        accounting, and emit the one authoritative session outcome line."""
        self._fill_session_result(result, session.force_terminal(outcome))
        self._log_toss_session_outcome(result)

    @staticmethod
    def _fill_session_result(result, session_result) -> None:
        result.success = bool(session_result.success)
        result.outcome = str(session_result.outcome)
        result.throws_completed = int(session_result.throws_completed)
        result.catches_confirmed = int(session_result.catches_confirmed)
        result.per_cycle_outcomes = [str(o) for o in
                                     session_result.cycle_outcomes]
        result.per_cycle_catch_error_mm = [
            float(v) for v in session_result.cycle_catch_error_mm]
        result.per_cycle_flight_s = [float(v) for v in
                                     session_result.cycle_flight_s]
        result.per_cycle_dwell_s = [float(v) for v in
                                    session_result.cycle_dwell_s]
        result.reloads_used = int(getattr(session_result, 'reloads_used', 0))

    def _log_toss_session_outcome(self, result) -> None:
        """The single authoritative per-SESSION outcome line (the reload/toss
        discipline one level up: exactly one per goal, INFO on success / WARN
        otherwise). Each CYCLE still emits its own Toss outcome line, so the log
        reads as N cycle lines followed by one session line."""
        errs = [v for v in result.per_cycle_catch_error_mm if np.isfinite(v)]
        dwells = [v for v in result.per_cycle_dwell_s if np.isfinite(v)]
        parts = [f'{result.catches_confirmed}/{result.throws_completed} caught']
        if errs:
            parts.append(f'mean catch_err={float(np.mean(errs)):.0f} mm')
        if dwells:
            parts.append(f'dwell {min(dwells):.2f}-{max(dwells):.2f} s')
        if int(getattr(result, 'reloads_used', 0) or 0):
            parts.append(f'reloads {int(result.reloads_used)}')
        line = (f'TossContinuous {result.outcome} ({", ".join(parts)}); '
                f'cycles: {list(result.per_cycle_outcomes)}')
        if result.success:
            self.get_logger().info(line)
        else:
            self.get_logger().warning(line)

    # ── BB service calls (executed on the FSM's behalf) ────────────────────────

    def _call_reload(self) -> bool:
        if not self._reload_cli.wait_for_service(timeout_sec=_SERVICE_WAIT_S):
            self.get_logger().error('bb/reload service unavailable')
            return False
        future = self._reload_cli.call_async(Trigger.Request())
        resp = self._wait_future(future)
        return bool(resp.success) if resp is not None else False

    def _send_throw(self, seq):
        """Aim + throw at the catch point via the point-target extension. Returns
        (accepted, message)."""
        if not self._throw_cli.wait_for_service(timeout_sec=_SERVICE_WAIT_S):
            return False, 'bb/throw_at_target service unavailable'
        req = BallButlerThrow.Request()
        req.use_target_point = True
        req.aim_only = False
        # Name the target so BB's ThrowAnnouncement carries target_id == this robot (not
        # the default 'point'): the tracker tags the ball destination from target_id, and
        # CatchCoordinator / this node's own announcement filter only act on OUR ball. An
        # unnamed 'point' announcement would be dropped by the whole catch pipeline.
        req.target_name = self._robot_name
        req.target_point_global_mm = Point(
            x=float(self._catch_point_mm[0]),
            y=float(self._catch_point_mm[1]),
            z=float(self._catch_point_mm[2]))
        req.throw_delay_s = float(seq.throw_delay_s)
        future = self._throw_cli.call_async(req)
        resp = self._wait_future(future)
        if resp is None:
            return False, 'bb/throw_at_target call failed'
        if bool(resp.success):
            # Snapshot the ids already IN_FLIGHT at throw-accept: they are NOT our
            # ball and must never be latched as it (attempt 5 of the 2026-07-23
            # re-test latched a pre-existing phantom track and rode it to a wrong
            # MISSED verdict).
            with self._lock:
                self._preexisting_flight_ids = {
                    int(b.id) for b in self._balls
                    if int(b.status) == _BALL_STATUS_IN_FLIGHT}
        return bool(resp.success), str(resp.message)

    # ── Platform + hand ownership (executed on the FSM's behalf) ───────────────

    def _prepare_catch(self) -> bool:
        """PREPARE (ball in hand, BEFORE the throw is committed): raise the catch-armed
        latch and publish ``catch/armed`` so the reactive catch path (platform reach +
        hand fire) is enabled for the flight window. The hand prime already ran at
        sequence start (ACTION_PRIME_HAND); ``catch_coordinator_node`` also re-primes
        idempotently on the ``catch/armed`` rising edge. Returns the latch outcome —
        a failure aborts the sequence while there is still NO pending BB throw (BB's
        firmware countdown has no abort opcode, so arming must precede the throw)."""
        ok = self._arm_catch(True)
        if ok:
            # Publish the goal's catch-speed knob BEFORE the armed edge so
            # catch_coordinator holds it before any hand-arm can fire.
            with self._lock:
                scale = self._catch_vel_scale
            self._vel_scale_pub.publish(Float64(data=float(scale)))
            self._publish_catch_armed(True)
        else:
            self.get_logger().error(
                'PREPARE failed: trajectory/arm_catch raise did not succeed — '
                'aborting before the throw is committed')
        return ok

    def _recenter(self):
        """RECENTER (successful catch): lower the catch-armed latch and re-center to
        level neutral via go_home. The hand keeps the caught ball — no retract."""
        self._arm_catch(False)
        self._publish_catch_armed(False)
        self._go_home()

    def _safe_abort(self):
        """SAFE_ABORT (abort once anything was armed): retract the hand to the bottom
        of its stroke, lower the catch-armed latch, and re-center via go_home.

        The retract target is ``JB_OP_HAND_RETRACT_REV`` (0.0) — NOT the homing
        reference ``HOMING_HAND_ABS_POS_REV`` (−0.1), which is below the bridge's
        smooth_move_hand range [0, max] and made every abort retract a silently
        rejected no-op (2026-07-23: the hand stayed parked at top through aborts).
        Every step's failure is LOUD here: this is the safing path.

        Ordering (audit, 2026-07-23): the catch/armed disarm is published FIRST
        so catch_coordinator's prime-retry tick stands down before the retract
        ladder starts — while still armed with ``_hand_primed`` unlatched
        (common at the observed ack-failure rate), a 0.5 s tick re-prime would
        erase the in-flight retract on the Teensy's last-writer-wins queue and
        the hand would silently return to top behind a 'successful' retract log.
        Topic delivery is ~ms vs the 0.5 s tick cadence, so ordering alone
        closes the race."""
        self._publish_catch_armed(False)
        if not self._retract_hand_with_retries():
            self.get_logger().error(
                'SAFE_ABORT: hand retract dispatch FAILED (all '
                f'{_HAND_DISPATCH_ATTEMPTS} attempts) — the hand may remain at '
                'the top of its stroke')
        if not self._arm_catch(False):
            self.get_logger().error(
                'SAFE_ABORT: trajectory/arm_catch lower FAILED — trajectory_node '
                'force-disarms on any mode change as the backstop')
        if not self._go_home():
            self.get_logger().error('SAFE_ABORT: go_home dispatch FAILED')

    def _safe_on_early_exit(self, seq):
        """Safe the robot when the action exits at the NODE level (cancel / timeout /
        shutdown) — bypassing the FSM's own terminal SAFE_ABORT. If PREPARE already ran
        (latch raised / hand primed), run the same retract + lower-latch + recenter so a
        cancelled reload never leaves the latch armed or the hand parked at top."""
        if seq.prepared:
            self.get_logger().warning(
                'Reload early exit while prepared — retracting hand + lowering catch latch.')
            self._safe_abort()

    def _arm_catch(self, armed: bool) -> bool:
        """Raise (True) / lower (False) trajectory_node's catch-armed latch."""
        if not self._arm_catch_cli.wait_for_service(timeout_sec=_SERVICE_WAIT_S):
            self.get_logger().error('trajectory/arm_catch service unavailable')
            return False
        req = SetBool.Request()
        req.data = bool(armed)
        resp = self._wait_future(self._arm_catch_cli.call_async(req))
        return bool(resp.success) if resp is not None else False

    def _smooth_move_hand(self, position_rev: float) -> bool:
        """Smooth-move the hand to ``position_rev`` (top = prime, bottom = retract)."""
        if not self._smooth_move_hand_cli.wait_for_service(timeout_sec=_SERVICE_WAIT_S):
            self.get_logger().error('smooth_move_hand service unavailable')
            return False
        req = SetFloat.Request()
        req.data = float(position_rev)
        resp = self._wait_future(self._smooth_move_hand_cli.call_async(req))
        return bool(resp.success) if resp is not None else False

    def _hand_dispatch_confirmed(self, target_rev: float, ascending: bool):
        """After a failed smooth-move ack, consult hand_telemetry to decide whether the
        frame actually moved the hand (the ack lies — frames transmit anyway). Returns:

          - ``True``  — the hand is at/near the target OR visibly moving toward it (ack
            lied, move underway) → do NOT re-dispatch (a re-dispatch rebuilds the
            min-jerk profile from the live mid-move position at v=0 and yanks the hand —
            the 2026-07-23 stutter);
          - ``False`` — the hand is positively stationary away from the target (the frame
            was genuinely lost) → re-dispatch;
          - ``None``  — telemetry absent/stale → cannot verify → the caller falls back to
            a blind re-dispatch (a strict superset of the pre-telemetry ladder).

        ``ascending`` picks the toward-target velocity sign (prime moves +, retract −)."""
        with self._lock:
            pos = self._hand_pos_meas
            vel = self._hand_vel_meas
            tel_mono = self._hand_telemetry_mono
        if tel_mono <= 0.0 or (time.perf_counter() - tel_mono) > _HAND_TELEMETRY_STALE_S:
            return None
        if abs(pos - target_rev) <= _HAND_NEAR_TARGET_REV:
            return True
        # Moving checks require the hand to have LEFT its starting park band: parked-hand
        # vel_meas noise reaches 1.82 rev/s p99 at bottom and 5.39 rev/s p99 at TOP
        # (gravity-hold dither; probed against the 2026-07-24_09-07-53 bag), so velocity
        # alone can fake "moving" for a genuinely-lost frame parked at top.
        if ascending:
            # Prime: rising off the bottom toward the top.
            if vel >= _HAND_MOVING_VEL_RPS and pos > 0.3:
                return True
        else:
            # Retract: descending, and clear of the top park band (see noise note).
            if (vel <= -_HAND_MOVING_VEL_RPS
                    and pos < hw.JB_OP_HAND_CATCH_PRIME_REV - _HAND_NEAR_TARGET_REV):
                return True
        return False

    def _prime_hand_with_retries(self) -> bool:
        """Dispatch the hand prime, retrying up to ``_HAND_DISPATCH_ATTEMPTS`` times.

        Announces every dispatch on ``catch/prime_dispatched`` BEFORE issuing it:
        the ack channel is unreliable in both directions (2026-07-23: failed acks
        were observed with the kind-3 frame still transmitted and the hand moving),
        so catch_coordinator's anti-stutter window must key off dispatch, not ack.

        On a failed ack, settle then telemetry-verify before re-dispatching: a lied-ack
        move that is already ascending must NOT be restarted (that yanks the moving hand
        and, over a full 4/4 ladder, produced the false ABORTED_PRIME_FAILED with the
        hand physically moving). Runs pre-throw / pre-reload-wait, so the bounded settle
        waits never eat BB's countdown."""
        for attempt in range(_HAND_DISPATCH_ATTEMPTS):
            self._prime_dispatched_pub.publish(Bool(data=True))
            if self._smooth_move_hand(hw.JB_OP_HAND_CATCH_PRIME_REV):
                return True
            time.sleep(_HAND_ACK_SETTLE_S)
            confirmed = self._hand_dispatch_confirmed(
                hw.JB_OP_HAND_CATCH_PRIME_REV, ascending=True)
            if confirmed:
                return True   # ack lied — the hand is at/nearing the top; do not restart it
            self.get_logger().warning(
                f'hand prime dispatch failed (attempt {attempt + 1}/'
                f'{_HAND_DISPATCH_ATTEMPTS}; telemetry '
                f'{"stationary" if confirmed is False else "unavailable"})')
        return False

    def _retract_hand_with_retries(self) -> bool:
        """Dispatch the SAFE_ABORT hand retract, retrying like the prime.

        The third sitting's retract failed outright on 7 of 12 aborts (same
        ~40-60% per-call HAND_TRAJ_CMD failure epidemic) and the hand silently
        stayed at top — benign only because the next goal re-primes. This is the
        safing path, so it gets the same telemetry-verified ladder the prime does: a
        lied-ack descent already underway is accepted (no spurious exhausted-ladder
        ERROR); only a positively-stationary hand (or absent telemetry) re-dispatches."""
        for attempt in range(_HAND_DISPATCH_ATTEMPTS):
            if self._smooth_move_hand(hw.JB_OP_HAND_RETRACT_REV):
                return True
            time.sleep(_HAND_ACK_SETTLE_S)
            confirmed = self._hand_dispatch_confirmed(
                hw.JB_OP_HAND_RETRACT_REV, ascending=False)
            if confirmed:
                return True   # ack lied — the hand is at/nearing the bottom (retracted)
            self.get_logger().warning(
                f'SAFE_ABORT: hand retract dispatch failed (attempt {attempt + 1}/'
                f'{_HAND_DISPATCH_ATTEMPTS}; telemetry '
                f'{"stationary" if confirmed is False else "unavailable"})')
        return False

    def _go_home(self) -> bool:
        """Re-center the platform to level neutral (trajectory/go_home)."""
        if not self._go_home_cli.wait_for_service(timeout_sec=_SERVICE_WAIT_S):
            self.get_logger().error('trajectory/go_home service unavailable')
            return False
        resp = self._wait_future(self._go_home_cli.call_async(Trigger.Request()))
        return bool(resp.success) if resp is not None else False

    def _publish_catch_armed(self, armed: bool):
        """Publish the catch-armed state that gates catch_coordinator_node's hand-arm."""
        self._catch_armed_pub.publish(Bool(data=bool(armed)))

    def _wait_future(self, future, timeout_s: float = _SERVICE_WAIT_S):
        """Wait for a service future to complete (the MultiThreadedExecutor services
        it on another thread). Returns the result or None on timeout/error."""
        deadline = time.perf_counter() + timeout_s
        while not future.done() and time.perf_counter() < deadline and rclpy.ok():
            time.sleep(0.005)
        if not future.done():
            return None
        try:
            return future.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'service call raised: {e}')
            return None


    def destroy_node(self):
        """Drain the record worker before the node goes away (census B6).

        The belt write is the last stop for a `record:=false` bench session's
        corpus, so a shutdown that discards a queued line discards a measurement
        that has no other sink. Bounded, and a WARN rather than a hang if the
        worker is wedged — a teardown must always complete.

        The sentinel is posted after the drain so the worker exits its blocking
        `get()` rather than being left parked on a daemon thread; a daemon thread
        would die with the process anyway, but leaving it explicitly stopped is
        what makes a leak visible in a test rather than invisible in a process.
        """
        try:
            self._toss_records_drain()
            if self._toss_records_thread is not None:
                self._toss_records_q.put_nowait(None)
                self._toss_records_thread.join(timeout=1.0)
        except Exception as exc:                                # noqa: BLE001
            self.get_logger().warning(
                'toss record worker teardown failed ({}) — continuing '
                'shutdown'.format(exc))
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    from rclpy.executors import MultiThreadedExecutor
    node = ReloadCoordinatorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
