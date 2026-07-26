"""ROS2 node: the ball-ops coordinator — the BB→Jugglebot reload action (MVP goal 4)
and the Jugglebot self-toss action (single-ball-toss plan, Phase 1).

One node hosts BOTH ball-op actions (the filename stays for launch/setup continuity).
They are merged deliberately, not for convenience: the two actions share the hand,
the ``catch/armed`` latch, and the Teensy's last-writer-wins trajectory queue, so
mutual exclusion must be a single in-process ``_lock``-guarded check — a cross-node
busy mechanism has publish-latency TOCTOU windows in which two live sequences
double-own the hand (one action's prime/retract erases the other's armed stroke) and
fight over the latch (one side's disarm silently kills the other's catch). The merge
also keeps exactly one copy of the bag-probed telemetry-ladder thresholds and of the
executor discipline both actions' ladders depend on.

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
config throw site A (``compute_release_state_tilted``'s swing-compensated pose is
the POSITIONING target, so the arm-edge reach envelope is captured at A), launch
tilt-aimed at the goal's B, and the platform's A→B reach is DEFERRED to the
scheduled release: the stock catch_coordinator announcement pre-tilt would
complete the A→B translate BEFORE the ball is released (its arrival clamps to
~now + 1 s while the toss announces ≥1 s pre-release) — aim destroyed, a moving
platform under a seated ball mid-windup — so this node raises the new
``catch/pretilt_hold`` gate for the goal's duration (catch_coordinator then
latches the announcement for its hand-arm window but publishes no platform
target) and itself publishes the ONE ``catch/dynamic_target`` at t_release,
built from the same ``CatchCoordinator.predicted_catch_command`` math with
arrival = the announced landing (lead = the flight time by construction). Tier
8a never touches the new topic — its publish sequence is byte-identical.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

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
    GoToPose,
    SetFloat,
    SetHandGains,
    SetHandTrajCmd,
)
from jugglebot_interfaces.action import Reload, Toss
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3

import jugglebot.hardware_config as hw
from jugglebot.reload_sequencer import (
    ACTION_CALL_RELOAD,
    ACTION_PREPARE_CATCH,
    ACTION_PRIME_HAND,
    ACTION_RECENTER,
    ACTION_SAFE_ABORT,
    ACTION_SEND_THROW,
    ReloadObservations,
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
    TossSequencer,
)
from jugglebot.catch_coordinator import CatchCoordinator
from jugglebot.motion.ik_solver import rot_matrix_to_quat, rotvec_to_rot_matrix
from jugglebot.motion.trajectory.toss_release import (
    ThrowTiltInfeasible,
    build_announcement_fields,
    compute_release_state,
    compute_release_state_tilted,
    flight_time_from_height,
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

# CAUGHT plausibility gate. The tracker's split-track corruption (2026-07-23 re-test)
# flips CAUGHT on tracks coasting BELOW THE FLOOR near BB — 5 of 6 reloads reported
# spurious CAUGHT while the real ball bounced out. A caught ball confirms OUR catch
# only if its final KF estimate is physically near the catch point.
_CAUGHT_MAX_XY_ERROR_MM = 200.0
_CAUGHT_MAX_Z_ERROR_MM = 150.0

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
_HAND_NEAR_TARGET_REV = 0.5        # |pos - target| within this ⇒ already at the target
_HAND_MOVING_VEL_RPS = 2.0         # |vel| toward the target at/above this ⇒ move underway
_HAND_TELEMETRY_STALE_S = 0.3      # telemetry older than this ⇒ cannot verify ⇒ blind ladder
# Sequence loop tick (the FSM is time-driven; this bounds latency, not correctness).
_TICK_S = 0.05
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
# constant: the arm_catch raise captures the reach-envelope center at the commanded
# pose, so a platform measured within the envelope radius of the nominated pose
# still has the nominated catch point covered by the captured envelope; beyond it
# the catch cannot reach. (A full silent no-op from a corner pose fails this; a
# near-centre no-op passes — and degrades gracefully, because the envelope then
# genuinely covers the nominated point from where the platform sits.)
_TOSS_POSITION_TOL_MM = float(hw.JB_TRAJ_CATCH_REACH_ENVELOPE_MM)
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
# Toss terminals on which the platform's true motion state is UNKNOWN: an
# ack-timed-out go_to_pose may still execute (the service returns at plan-INSTALL,
# so a missing/late ack does not mean no motion — the plan may be running or land
# later). The FSM's terminal action there is ACTION_NONE (nothing verifiably
# accepted ⇒ no full SAFE_ABORT), so the node dispatches a best-effort go_home to
# supersede any zombie move — go_home replaces the installed trajectory by design.
_TOSS_POSITION_UNKNOWN_TERMINALS = frozenset({
    'REJECTED_POSITION(NO_RESPONSE)', 'ABORTED_POSITION_TIMEOUT'})


def _toss_deadline_s(seq) -> float:
    """The node-level hard ceiling for THIS toss goal — same doctrine as
    :func:`_sequence_deadline_s`: never below _MAX_SEQUENCE_S and never inside a
    legitimate sequence window (positioning + delay + flight + grace + confirm),
    because the timeout path SAFE_ABORTs (retract under an airborne ball)."""
    budget = (float(seq.positioning_timeout_s) + float(seq.throw_delay_s)
              + float(seq.flight_time_s) + float(seq.release_grace_s)
              + float(seq.catch_confirm_window_s) + _SEQUENCE_CEILING_MARGIN_S)
    return max(_MAX_SEQUENCE_S, budget)


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
        # Ball ids already IN_FLIGHT when THIS goal's throw was accepted — excluded
        # from the announced-ball latch. Attempt 5 of the 2026-07-23 re-test latched
        # a phantom untagged track that predated the throw and rode it to a wrong
        # MISSED verdict.
        self._preexisting_flight_ids = set()
        # This goal's catch-speed knob (goal.catch_vel_scale; 0 => the config
        # default JB_OP_CATCH_VEL_SCALE_DEFAULT), published on catch/vel_scale at
        # PREPARE so catch_coordinator has it before any arm.
        self._catch_vel_scale = float(hw.JB_OP_CATCH_VEL_SCALE_DEFAULT)
        # Once-per-ball log guard for implausible CAUGHT rejections.
        self._implausible_logged_ids = set()
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
        # Tier 8b only: catch/pretilt_hold was raised this goal — the terminal
        # teardowns release it iff raised, so an 8a goal's publish sequence
        # stays byte-identical (it never touches the topic).
        self._toss_pretilt_hold_raised = False
        # Tier 8b only: the announced landing state stashed at ANNOUNCE for the
        # deferred A->B reach to reuse — (landing_pos_global_mm, landing_vel_mms,
        # landing_time_ros_seconds). None between goals / for 8a.
        self._toss_announced_reach = None
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
        # _lock-guarded claim spanning both actions (a Toss goal while a Reload
        # runs — or vice versa — is REJECTED_BUSY at accept), and both defer
        # cancel semantics to their execute loops.
        self._toss_action = ActionServer(
            self, Toss, 'jugglebot/toss',
            execute_callback=self._execute_toss,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cbgroup)

        self.get_logger().info(
            f"Ball-ops coordinator ready (jugglebot/reload + jugglebot/toss); "
            f"catch point {self._catch_point_mm} mm (world).")

    # ── Subscription callbacks ─────────────────────────────────────────────────

    def _on_heartbeat(self, msg):
        with self._lock:
            self._hb = msg
            self._hb_mono = time.perf_counter()

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
            self._traj_status_mono = time.perf_counter()

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
                    and self._caught_is_plausible(b, ref_point_mm=ref)):
                with self._lock:
                    self._ball_possession = True
                break

    def _on_hand_telemetry(self, msg):
        with self._lock:
            self._hand_pos_meas = float(msg.pos_meas)
            self._hand_vel_meas = float(msg.vel_meas)
            self._hand_telemetry_mono = time.perf_counter()

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
        neither is present)."""
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
                        if self._caught_is_plausible(b):
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
            ball_caught=ball_caught,
            catch_error_mm=catch_error_mm)

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
        is preferred over the empty-destination fallback."""
        with self._lock:
            announced_id = self._announced_ball_id
            preexisting = set(self._preexisting_flight_ids)
            untagged_latch = self._announced_id_untagged
        candidates = [
            b for b in balls
            if int(b.status) == _BALL_STATUS_IN_FLIGHT
            and int(b.id) not in preexisting]
        dest_match = next(
            (int(b.id) for b in candidates
             if b.destination == self._robot_name), None)
        if announced_id is None:
            if dest_match is not None:
                announced_id = dest_match
                untagged_latch = False
            else:
                for b in candidates:
                    if not b.destination:
                        announced_id = int(b.id)
                        untagged_latch = True
                        break
        elif untagged_latch and dest_match is not None:
            # An UNTAGGED latch is provisional (audit, 2026-07-23): a phantom
            # spawning during the countdown could grab it first. The moment a
            # destination-tagged candidate appears, re-latch to it — the
            # tagged track is the tracker's own claim about OUR ball.
            announced_id = dest_match
            untagged_latch = False
        if announced_id is not None:
            with self._lock:
                self._announced_ball_id = announced_id
                self._announced_id_untagged = untagged_latch
        return announced_id

    def _caught_is_plausible(self, ball, ref_point_mm=None) -> bool:
        """A tracker CAUGHT confirms OUR catch only if the ball's final KF estimate is
        physically near the catch point. The tracker's split-track corruption flips
        CAUGHT on tracks coasting below the floor near BB (e.g. (−539, −323, −532) mm)
        — 5 of 6 reloads on 2026-07-23 spuriously reported SUCCESS through that.
        Rejecting the implausible CAUGHT makes the outcome honest (MISSED) until the
        tracker investigation lands.

        ``ref_point_mm`` overrides the reference point (the toss judges against its
        NOMINATED landing point); None = the reload's fixed ACTIVE catch point."""
        ref = self._catch_point_mm if ref_point_mm is None else ref_point_mm
        xy_err = self._catch_error_from_ball(ball, ref_point_mm=ref)
        z_err = abs(float(ball.position.z) - float(ref[2]))
        ok = (xy_err <= _CAUGHT_MAX_XY_ERROR_MM
              and z_err <= _CAUGHT_MAX_Z_ERROR_MM)
        if not ok and int(ball.id) not in self._implausible_logged_ids:
            self._implausible_logged_ids.add(int(ball.id))
            # INFO, not WARN: split-track corruption flips CAUGHT below the floor on
            # essentially every flight — this is expected tracker noise, once-per-ball
            # guarded, and subsumed by the imminent ball-held hand sensor. It must not
            # read as an error in a working reload.
            self.get_logger().info(
                f"Ball {int(ball.id)}: tracker CAUGHT at "
                f"({float(ball.position.x):.0f}, {float(ball.position.y):.0f}, "
                f"{float(ball.position.z):.0f}) mm is IMPLAUSIBLE for the catch point "
                f"(xy err {xy_err:.0f} > {_CAUGHT_MAX_XY_ERROR_MM:.0f} or z err "
                f"{z_err:.0f} > {_CAUGHT_MAX_Z_ERROR_MM:.0f}) — not counted "
                f"(split-track corruption; see the 2026-07-23 tracker finding)")
        return ok

    def _catch_error_from_ball(self, ball, ref_point_mm=None) -> float:
        """Horizontal miss distance of the caught ball from the world-frame catch point.

        This is the tracker's last KF position estimate at CAUGHT (an in-flight-derived
        estimate), NOT a settled rest position — the MVP evidence is the tracker-id
        correlation + this KF miss, with a hand-telemetry rest cross-check deferred.
        ``ref_point_mm`` overrides the reference (the toss's nominated landing point);
        None = the reload's fixed ACTIVE catch point."""
        ref = self._catch_point_mm if ref_point_mm is None else ref_point_mm
        dx = float(ball.position.x) - float(ref[0])
        dy = float(ball.position.y) - float(ref[1])
        return float(np.hypot(dx, dy))

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
          - ``ball_seated`` — TRUE unless the ball-evidence gate is required
            (config ``toss_require_ball_evidence``, default false — the operator
            guarantees the ball; there is no ball-in-cup sensor). When required,
            it is the sticky possession latch OR the trace-only waiver; possession
            is CLEARED here on OUR release evidence (the ball left the cup) and
            re-set by the next plausible CAUGHT (chainable Toss → Toss);
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
        track_active = False
        ball_caught = False
        catch_error_mm = float('nan')
        time_at_land_perf = float('nan')
        if balls_fresh:
            track_active = any(
                b.destination == self._robot_name
                and int(b.status) in (_BALL_STATUS_TO_BE_THROWN,
                                      _BALL_STATUS_IN_FLIGHT)
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
                        if self._caught_is_plausible(b, ref_point_mm=landing_ref):
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
            ball_seated=bool(waiver or possession
                             or not hw.JB_OP_TOSS_REQUIRE_BALL_EVIDENCE),
            track_active=track_active,
            platform_at_target=platform_at_target,
            throw_stroke_seen=stroke_seen,
            ball_track_confirmed=track_confirmed,
            ball_caught=ball_caught,
            catch_error_mm=catch_error_mm,
            ball_time_at_land_perf=time_at_land_perf)

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
        # ONE ball-op at a time, across BOTH actions (shared by the reload and toss
        # servers). rclpy runs accepted goals concurrently, so a second goal accepted
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
        if busy:
            self.get_logger().warning(
                'Ball-op goal REJECTED_BUSY — a reload/toss is already in '
                'progress (one ball-op at a time, across both actions).')
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
        self._implausible_logged_ids = set()

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
        if height == 0.0:
            # 0 ⇒ the generated config default FLIGHT TIME, resolved HERE so the
            # FSM receives the config-resolved value (kept internally as a flight
            # time — the FSM's DEFAULT_TOSS_FLIGHT_TIME_S literal is the no-config
            # fallback only, drift-guard-pinned equal; ~0.784 m apex at 0.8 s).
            flight = float(hw.JB_OP_TOSS_FLIGHT_TIME_DEFAULT_S)
        else:
            # Operator nominates a HEIGHT (m, apex above release); convert ONCE to
            # the internal flight time via the single tested toss_release module.
            flight = flight_time_from_height(height)
        # The release state (frames + ballistics) comes from the single tested
        # conversion module; the FSM gets its event_vel from here, never a second
        # derivation. An OUT-OF-BAND flight time still computes a release state
        # and is then loudly REJECTED_FLIGHT_TIME by the FSM's CHECKING pass.
        tier = str(hw.JB_OP_TOSS_TIER)
        throw_site = tuple(float(v) for v in hw.JB_OP_TOSS_THROW_SITE_MM)
        tilt_clamp_exceeded = False
        if tier == TIER_8B:
            # Tier 8b: throw from the config site A, tilt-aimed at the displaced
            # B. The module's clamp gate is the authoritative aim check — a
            # raise maps onto the FSM's tilt_clamp_exceeded flag so CHECKING
            # mints REJECTED_TILT_CLAMP in gate order (no drift-prone second
            # copy of the aim math; unreachable inside the ±150 mm workspace —
            # the aim needs ~315 mm of displacement to hit the 12° ceiling).
            try:
                release = compute_release_state_tilted(
                    catch_pose, flight, throw_site_xy_mm=throw_site)
            except ThrowTiltInfeasible as exc:
                self.get_logger().error(f'Toss displaced aim infeasible: {exc}')
                release = None
                tilt_clamp_exceeded = True
        else:
            release = compute_release_state(catch_pose, flight)
        seq = TossSequencer(
            catch_pose_stow_mm=catch_pose, flight_time_s=flight,
            throw_delay_s=throw_delay, tier=tier,
            event_vel_mps=(float(release.event_vel_mps)
                           if release is not None else 0.0),
            throw_site_xy_mm=throw_site,
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
            self._announced_ball_id = None
            self._announced_id_untagged = False
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
            # can never diverge): Tier 8a = the nominated catch pose B, Tier 8b =
            # the swing-compensated pre-tilt pose at the throw site A (else an
            # operator who configures a platform body for an 8b sitting gets
            # measured-A vs target-B → a spurious ABORTED_POSITION_FAILED).
            self._toss_platform_target_mm = self._toss_positioning_xyz(
                tier, catch_pose, release)
            self._toss_waiver = waiver
            self._toss_mocap_body = str(
                self.get_parameter(_TOSS_MOCAP_BODY_PARAM).value or '')
            self._platform_pos_mm = None   # never trust a stale/other-body cache
            self._toss_prepare_pending = False
            self._toss_throw_dispatched = False
            self._toss_stroke_seen = False
            self._toss_track_confirmed = False
            # Tier-8b per-goal state (8a never touches catch/pretilt_hold).
            self._toss_pretilt_hold_raised = False
            self._toss_announced_reach = None
        self._implausible_logged_ids = set()

        result = Toss.Result()
        max_sequence_s = _toss_deadline_s(seq)
        try:
            t_start = time.perf_counter()
            while rclpy.ok():
                now = time.perf_counter()
                if (goal_handle.is_cancel_requested
                        and not self._toss_cancel_deferred(seq, now)):
                    self._safe_toss_on_early_exit(seq)
                    goal_handle.canceled()
                    result.success = False
                    result.outcome = 'ABORTED_CANCELLED'
                    result.catch_error_mm = float('nan')
                    result.achieved_flight_s = float('nan')
                    self._log_toss_outcome(result)
                    return result
                decision = self._step_toss_sequence(seq, now, goal_handle)
                if decision.done:
                    r = decision.result
                    result.success = bool(r.success)
                    result.outcome = str(r.outcome)
                    result.catch_error_mm = float(r.catch_error_mm)
                    result.achieved_flight_s = float(r.achieved_flight_s)
                    self._log_toss_outcome(r)
                    if goal_handle.is_cancel_requested:
                        # A DEFERRED cancel (past the cutoff / in flight) resolves
                        # here with the FSM's real outcome — the catch attempt ran
                        # to its own terminal, exactly the plan's §7 semantics.
                        goal_handle.canceled()
                    elif r.success:
                        goal_handle.succeed()
                    else:
                        goal_handle.abort()
                    return result
                if now - t_start > max_sequence_s:
                    self._safe_toss_on_early_exit(seq)
                    result.success = False
                    result.outcome = 'ABORTED_TIMEOUT'
                    result.catch_error_mm = float('nan')
                    result.achieved_flight_s = float('nan')
                    self._log_toss_outcome(result)
                    goal_handle.abort()
                    return result
                time.sleep(_TICK_S)
            # rclpy shutting down.
            self._safe_toss_on_early_exit(seq)
            result.success = False
            result.outcome = 'ABORTED_SHUTDOWN'
            result.catch_error_mm = float('nan')
            result.achieved_flight_s = float('nan')
            self._log_toss_outcome(result)
            return result
        except Exception:
            # Unexpected fault mid-sequence (the loop body raised): the robot may
            # be positioned / latched / stroke-armed, so SAFE FIRST, then emit
            # the one authoritative outcome line, abort the goal, and re-raise so
            # the executor's own error path still sees the fault.
            self._safe_toss_on_early_exit(seq)
            result.success = False
            result.outcome = 'ABORTED_EXCEPTION'
            result.catch_error_mm = float('nan')
            result.achieved_flight_s = float('nan')
            self._log_toss_outcome(result)
            try:
                goal_handle.abort()
            except Exception:  # noqa: BLE001
                # The handle may already be terminal (fault after succeed/abort);
                # the original exception must survive the cleanup.
                self.get_logger().error(
                    'ABORTED_EXCEPTION: goal_handle.abort() itself failed')
            raise
        finally:
            with self._lock:
                self._active_seq = None
                self._goal_claimed = False
                # Per-goal toss state down; the possession plausibility reference
                # reverts to the ACTIVE catch point between goals.
                self._toss_release_state = None
                self._toss_landing_global_mm = None
                self._toss_platform_target_mm = None
                self._toss_prepare_pending = False
                self._toss_throw_dispatched = False

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
        (50 ms ≫ topic latency) puts the hold in an earlier CCN cycle."""
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
            if seq.tier == TIER_8B:
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
        the wrong site. The trajectory/arm_catch reach-envelope is captured at the
        COMMANDED pose (A for 8b, in _prepare_toss_catch), so the deferred A->B
        reach published at t_release stays inside the 80 mm envelope (B is <= 70 mm
        from A across the Phase-4 grid). The 8b tilt is encoded through the SAME
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
        with self._lock:
            release = self._toss_release_state
        # The STOW-frame POSITION is the SINGLE source shared with the POSITIONING
        # mocap arrival cross-check target (_toss_platform_target_mm, set in
        # _execute_toss from the SAME _toss_positioning_xyz), so what we command
        # and what we verify against can never diverge: Tier 8a = level B, Tier 8b
        # = the swing-compensated pre-tilt pose at the throw site A.
        x, y, z = self._toss_positioning_xyz(
            seq.tier, seq.catch_pose_stow_mm, release)
        if seq.tier == TIER_8B:
            # Tier 8b: tilt-aimed at A. release is a TiltedReleaseState
            # post-CHECKING (a tilt-clamp raise is rejected on the first FSM step,
            # before POSITIONING is ever reached) — the unguarded read mirrors
            # _announce_toss's read of the same stash.
            orientation = self._tilt_quaternion(release.tilt_rx, release.tilt_ry)
        else:
            orientation = Quaternion()              # identity = level platform (8a)
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

    @staticmethod
    def _toss_positioning_xyz(tier, catch_pose_stow_mm, release):
        """The STOW-frame platform POSITION the toss pre-positions to — the SINGLE
        source for BOTH the go_to_pose command (:meth:`_position_platform_for_toss`)
        and the POSITIONING mocap arrival cross-check target
        (``_toss_platform_target_mm``, set in :meth:`_execute_toss`), so the
        commanded pose and the verification target can never diverge. Tier 8a = the
        nominated catch pose B (level); Tier 8b = the swing-compensated pre-tilt
        pose at the throw site A (``release.pretilt_pose_stow[:3]``). ``release`` is
        None only on an 8b tilt-clamp raise (the goal is rejected before
        POSITIONING), so the B fallback there is moot. Returns a float (x, y, z)."""
        if tier == TIER_8B and release is not None:
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
          2. trajectory/arm_catch raise + confirm — captures the reach-envelope
             center at the (verified) nominated pose; a failure aborts with
             nothing announced and no throw armed;
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
        diagnostic fields when finite."""
        err = float(result.catch_error_mm)
        fl = float(result.achieved_flight_s)
        parts = []
        if np.isfinite(err):
            parts.append(f'catch_err={err:.0f} mm')
        if np.isfinite(fl):
            parts.append(f'flight={fl:.3f} s')
        suffix = f" ({', '.join(parts)})" if parts else ''
        line = f'Toss {result.outcome}{suffix}'
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
