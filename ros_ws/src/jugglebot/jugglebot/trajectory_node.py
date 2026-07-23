"""trajectory_node — thin ROS 2 wrapper around the MVP trajectory generator.

The Jetson-side replacement for ``run_mpc.py``'s hot path (Phase 1 of
``plans/active/mvp-trajectory-bringup.md``). A dedicated 40 Hz emitter thread
streams ``make_mpc_command`` knot frames on ZMQ :5557 — the exact seam
``HardwarePlant`` used — which ``teensy_bridge_node``'s ``_MpcCommandSetpointSource``
consumes unchanged, feeding ``SetpointPump`` → the can-hub Teensy's 500 Hz
Hermite. Phase 1 streams only a **hold**: the platform holds the pose measured from
``robot_state`` telemetry, so the first commanded ``u0`` lands inside both the
pump's 0.3 rev step gate and the firmware's 1.0 rev MAX_DEVIATION backstop.

Design (mirrors the validated 40 Hz MPC path's discipline):
  * **Dedicated emitter thread** (not an rclpy timer): absolute-deadline 40 Hz
    loop on ``perf_counter``; the ZMQ PUB is bound **inside** the thread. Being
    the sole binder of :5557 is the MPC/trajectory mutual-exclusion interlock — a
    bind failure means ``run_mpc.py`` is running, and the emitter refuses to start
    (loud error), rather than fighting over the wire.
  * **Always has a plan** — a ``HoldPlan`` once seeded — so stream gaps never
    approach the 250 ms staleness E-STOP (10× margin at 25 ms knots).
  * **Atomic plan install**: every new plan is built from the *current* sampled
    state (``state_at(now)``) so pose/twist/accel are continuous (C2) across swaps.
  * **All motion flows through ``planner`` → ``feasibility.validate``** — the
    ``trajectory/hold`` and ``trajectory/go_home`` services construct plans only
    through the planner, so nothing bypasses the gate.
  * **Defence-in-depth step bound** in the emitter: if a per-knot ``|Δu0|`` ever
    exceeds the motor step bound, the node freezes to a hold at the last good pose
    and logs ERROR (the gate should already prevent this; it is a backstop).

Phase 2 adds the profiled point-to-point move surface: ``trajectory/go_to_pose``
(``GoToPose``, TRAJECTORY mode only — else a loud ``WRONG_MODE`` reject) drives
``planner.build_move`` (full feasibility gate + duration-stretch loop);
``trajectory/set_limits`` (``SetTrajectoryLimits``) is the in-session leg-limit
ramp (each value clamped to its YAML hard ceiling). ``trajectory/status`` is now
the typed ``jugglebot_interfaces/TrajectoryStatus`` (migrated off the Phase-1
``diagnostic_msgs/DiagnosticStatus`` stand-in), and ``trajectory/diagnostics``
publishes the active plan's measured leg peaks + emitter jitter.

Phase 5 adds timed target states: ``trajectory/timed_target`` (``TimedTarget``,
TRAJECTORY mode) reaches a pose at a nominal velocity a RELATIVE ``lead_time_s``
seconds after service receipt (the node anchors the absolute arrival at handler
entry on ``perf_counter``) via ``planner.build_timed`` (a fixed-lead reach — a
too-tight lead is loudly rejected ``TOO_FAST`` with the achievable
``min_duration_s``, never silently slowed). The catch-armed latch gates ``catch/dynamic_target`` through the SAME
``build_catch`` (Phase 7: the tilt-through-seat catch trajectory — a reach to the
receive-tilted catch pose with translational arrival velocity forced to zero, a
small residual tilt rate carried through the seat, then a literal quiescent hold;
Phase 5 routed the catch through ``build_timed``, a reach-only plan that parked the
tilted rim at contact) plus a reach-freeze window that ignores late target jitter,
and ``trajectory/target_feedback`` carries the accept/reject decision to
``catch_coordinator_node`` (which drives its feasibility blacklist). Both timed
paths supersede an in-flight plan with a C2 replan: ``build_timed`` is gated by the
fast ``validate_follow``, so a supersede installs shortly after its seed sample —
single-digit ms typical, guard-bounded (install-continuity rejects ``STALE_STATE`` on
a drift > 0.06 rev), worst case tens of ms with an appended stop-stretch (no ``BUSY``
restriction — that guard exists only on the ~377 ms ``go_to_pose`` path). Every
timed surface is perf-domain: the ``timed_target`` service carries a relative
lead anchored at handler entry, and the catch path's arrival arrives already
perf-domain from ``catch_coordinator_node`` — no ROS-clock crossing remains
(the retained ``_ros_time_to_perf`` offset plumbing currently has no consumer).

**Armed-mode-exit is a sharp edge (Phase 1).** Leaving the streaming mode set
while the bridge is ARMED stops the emitter publishing (``_streaming=False``), so
the bridge stops receiving frames and latches an ``MPC_STALE`` E-STOP within
250 ms. Always **disarm** (``set_setpoint_output false``) before changing the
control mode away from a streaming mode. The structural fix — coupling mode-exit
to an auto-disarm — is deferred to Phase 2 (orchestrator wiring).
"""

from __future__ import annotations

import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from jugglebot_interfaces.msg import (
    DynamicTargetCommand,
    PlatformPoseCommand,
    RobotState,
    TargetFeedback,
    TrajectoryStatus,
)
from jugglebot_interfaces.srv import GoToPose, SetTrajectoryLimits, TimedTarget

import jugglebot.hardware_config as hw
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import (
    leg_lengths_to_pose,
    pose_to_leg_lengths,
    quat_to_rot_matrix,
    rot_matrix_to_rotvec,
    rotvec_to_rot_matrix,
)
from jugglebot.motion.ipc import MpcCommandPub
from jugglebot.motion.trajectory import (
    HoldPlan,
    KnotEmitter,
    LeanShaper,
    TargetFollower,
    TrajectoryInfeasible,
    TrajectoryLimits,
    TrajectoryPlan,
)
from jugglebot.motion.trajectory import feasibility as feas
from jugglebot.motion.trajectory import planner

# The control mode in which explicit move services (go_to_pose) are accepted. In
# any other streaming mode (STANDBY holds; SPACEMOUSE has its own command
# source) a go_to_pose is rejected WRONG_MODE — loudly, never silently — so the
# operator can never drive a scripted move from a mode that isn't expecting one.
_MOVE_MODE = 'TRAJECTORY'

# Service-level rejection codes (NOT feasibility-enum members — those describe a
# *plan*; these describe the *node's acceptance state*). They live beside
# _MOVE_MODE because, like WRONG_MODE, they are properties of the service surface,
# not of any plan the gate reasons about.
#   BUSY — a go_to_pose arrived while a move is already in flight. Phase 2 accepts
#   moves only FROM a hold; interrupting an in-flight move (supersede) lands with
#   Phase 3 (SpaceMouse follower) / Phase 5 (timed targets). This restriction is
#   temporary and is lifted by those phases.
#   FROZEN — a catch/dynamic_target arrived inside the reach-freeze window, so the
#   committed catch reach is held and the late target ignored. Reported to the
#   coordinator so it neither treats the target as accepted nor counts it as a
#   feasibility rejection (it is not a property of any plan the gate reasons about).
_BUSY = 'BUSY'
_FROZEN = 'FROZEN'
#   GUARD_LATCHED — a move/target arrived while a Teensy guard E-STOP is latched
#   (FIX 1). Command advancement is frozen at the measured hold until CLEAR_ERRORS,
#   so any advancing service is refused loudly rather than jumping u0 back toward a
#   target the suppressed legs cannot follow (which is what re-trips the guard).
_GUARD_LATCHED = 'GUARD_LATCHED'


# Control modes in which the node streams a hold. STANDBY is the armed-hold state
# the Phase 1 arming bring-up uses; the active sub-modes are included so streaming
# stays on across the modes later phases drive (TRAJECTORY lands in Phase 2). A
# mode outside this set stops streaming and forces a re-seed on the next entry.
_DEFAULT_STREAM_MODES = ('STANDBY', 'TRAJECTORY', 'SPACEMOUSE', 'GUI')

# Modes in which the SpaceMouse/GUI streaming-target follower (Phase 3) is
# active — the pose-command modes, mirroring mpc_bridge_node._POSE_MODES. In these
# modes the emitter tick drains the latest platform_pose target and replans toward
# it (fast validate_follow gate); other streaming modes hold (STANDBY) or use their
# own command source (TRAJECTORY→go_to_pose; the catch path is gated by the
# catch-armed latch, not a mode — see _svc_arm_catch).
_FOLLOWER_MODES = frozenset({'SPACEMOUSE', 'GUI'})

# Modes that actively COMMAND platform motion (as opposed to STANDBY, which holds).
# Leaving one of these for a non-motion streaming mode (STANDBY) while a move is in
# flight installs a graceful stop — the move is silenced, not left to run on to its
# target (a near-boundary seed that the stop can't yet gate is retried via the
# pending-stop path in _emit_once, not dropped). The catch reach (a build_catch
# 'move' plan) runs in TRAJECTORY under the catch-armed latch; its in-flight
# graceful stops are keyed off the latch EDGES (_svc_arm_catch), not a mode.
_MOTION_MODES = _FOLLOWER_MODES | {'TRAJECTORY'}

_NUM_LEGS = 6

# A3 — follower escalation backstop. Once the follower latches an escalation stop
# (sustained chase rejects: a corrupt / boundary-wedged seed), the post-publish
# retry keeps trying build_graceful_stop. After this many CONSECUTIVE failed stop
# builds while latched, the node installs a HoldPlan at the last emitted pose
# DIRECTLY (u0 step = 0, the emitter step-backstop precedent) so the corrupt-seed
# class has a terminal, loud action — never a silent keep-last forever.
_ESCALATE_HOLD_TICKS = 12

# A4 — cap on the post-publish graceful-stop retry (mode-exit / input-loss /
# escalation). The energy-informed start converges in ≤ ~3 validates; this bounds
# the worst-case block far under the ~117 ms firmware decay→sprint threshold, and
# the retry resumes next tick from the decayed seed if it doesn't converge here.
_STOP_RETRY_ITERS = 6


class TrajectoryNode(Node):
    """Thin ROS wrapper; the trajectory math lives in ``jugglebot.motion.trajectory``.

    Args:
        geom: injected :class:`StewartGeometry` (tests); built from config if None.
        command_pub_factory: zero-arg callable returning a PUB with ``send(dict)`` /
            ``close()`` (tests inject a capturing fake); ``MpcCommandPub`` if None.
        start_emitter: start the 40 Hz emitter thread in ``__init__`` (production).
            Tests pass ``False`` and drive ``_emit_once`` / the seeding + services
            directly.
    """

    def __init__(self, *, geom: StewartGeometry | None = None,
                 command_pub_factory=None, start_emitter: bool = True):
        super().__init__('trajectory_node')

        self._geom = geom if geom is not None else StewartGeometry()
        self._mm_to_rev = np.asarray(self._geom.mm_to_rev, dtype=float)
        self._limits = TrajectoryLimits.from_config(hw)
        self._emitter = KnotEmitter(self._geom, knot_dt_s=hw.JB_TRAJ_KNOT_DT_S)
        self._neutral_pose = np.array(
            [0.0, 0.0, float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM), 0.0, 0.0, 0.0])
        self._pub_factory = command_pub_factory

        # ── Lean shaping (Phase 4; config default 0.6 since 2026-07-17) ──
        # The lever arm + gravity are fixed; the gain is per-move (config default,
        # overridable per go_to_pose call for the hardware A/B). _make_shaper builds
        # a LeanShaper for a given effective gain.
        self._config_lean_gain = float(hw.JB_TRAJ_LEAN_GAIN)
        self._gravity_mm_s2 = float(hw.GRAVITY_MMPS2)
        # Shaped-move duration search (shaped-planning-efficiency Phase 2): the
        # retiming model proposes T and the unchanged gate verifies it; a rejection
        # falls back to the legacy stretch+bisection loop inside build_move. Config-
        # gated so a hardware A/B can force the legacy loop without a redeploy.
        self._retime_model = bool(hw.JB_TRAJ_RETIME_MODEL)

        # ── Timed targets + catch (Phase 5) ──────────────────────
        # ROS-clock → perf_counter offset plumbing (plan Architecture's single
        # conversion point). perf_counter is CLOCK_MONOTONIC on Linux (system-wide,
        # comparable across processes and with catch_coordinator_node's perf-domain
        # arrival_time), so the emitter thread and the catch path share one time
        # base. Since the lead_time_s interface change the timed_target SERVICE
        # carries a RELATIVE lead (anchored at handler entry on perf_counter), so
        # this offset currently has NO consumer — retained (mirrors
        # catch_coordinator_node) for any future ROS-clock-timed surface.
        self._ros_to_perf_offset = self._measure_clock_offset()
        self._clock_offset_history = [self._ros_to_perf_offset]
        # catch/dynamic_target pose is STOW-relative — the SAME frame as the trajectory
        # pose convention (0 = the stow plane at GEOM_INITIAL_HEIGHT, ~170 = active):
        # the coordinator subtracts GEOM_INITIAL_HEIGHT from the world-frame intercept,
        # and the wire historically forwarded VERBATIM into the (stow-relative) MPC
        # frame. Hardware-verified 2026-07-23: this node's previous "+ active_z lift"
        # (which assumed an active-relative wire per a false premise about the MPC
        # convention) double-counted the active height, commanding z ≈ 341 mm and
        # driving every catch reach out of stroke (leg hard max 275 mm) — the Phase-7
        # pre-registered open question, answered. No z conversion happens here anymore.
        self._catch_reach_freeze_s = float(hw.JB_TRAJ_CATCH_REACH_FREEZE_S)
        # Quiescent hold after the catch seat: once arrival + this window has fully
        # passed, the freeze is released so a later catch/dynamic_target supersedes as a
        # new reach (repeated catches — Phases 8/9), rather than being silently dropped
        # forever behind a latched freeze.
        self._catch_settle_hold_s = float(hw.JB_TRAJ_CATCH_SETTLE_HOLD_S)
        # Absolute (perf) arrival time of the committed catch reach, or None. Once
        # within catch_reach_freeze_s of it, later target jitter is ignored (the reach
        # is frozen into the catch). Reset on the catch-armed latch edges / a fresh seed.
        self._catch_arrival_perf = None
        # Catch-armed latch (reload-action-catch-latch plan). While True,
        # ``catch/dynamic_target`` installs a ``build_catch`` reach — the latch is the
        # catch TRIGGER the RELOAD action raises for the flight window (it is the sole
        # catch trigger). The node stays in TRAJECTORY (already streaming +
        # motion-capable), so the latch is NOT a member of any mode set; the
        # freeze-reset + graceful-stop bookkeeping is keyed off the latch EDGES (see
        # _svc_arm_catch). Read only on the executor thread (_on_dynamic_target), so a
        # plain bool needs no lock.
        self._catch_armed = False
        # Reach-envelope enforcement (build_catch documents the ≤ envelope excursion as
        # the CALLER's responsibility — this is the enforcement point). The commanded
        # position captured at the arm-latch RAISE is the center; any catch target
        # farther than the envelope (3D) is rejected WORKSPACE before planning. Bounds
        # two observed failure classes (2026-07-23): a frame-convention regression on
        # the wire (rejected loudly at ~170 mm instead of as a cryptic leg-stroke
        # graze) and the platform chasing a drifting ball-tracker landing estimate
        # hundreds of mm sideways during the flight window. None while disarmed.
        self._catch_reach_envelope_mm = float(hw.JB_TRAJ_CATCH_REACH_ENVELOPE_MM)
        self._catch_envelope_center = None

        # ── Parameters ─────────────────────────────────────────
        self.declare_parameter('control_mode_topic', 'control_mode_topic')
        self.declare_parameter('go_home_duration_s', 2.0)
        self.declare_parameter('robot_state_stale_s', 0.5)
        # Follower input-loss timeout: if no fresh platform_pose target arrives
        # within this window (the SpaceMouse node itself publishes an ACTIVE-pose
        # hold on unplug, so this is the backstop for that node dying entirely), the
        # emitter installs a graceful stop. Kept well under the 250 ms staleness
        # E-STOP — the emitter keeps streaming hold frames throughout, so input loss
        # never approaches a wire-staleness fault.
        self.declare_parameter('follower_input_loss_s', 0.4)
        self._go_home_duration_s = float(
            self.get_parameter('go_home_duration_s').value)
        self._robot_state_stale_s = float(
            self.get_parameter('robot_state_stale_s').value)
        self._follower_input_loss_s = float(
            self.get_parameter('follower_input_loss_s').value)
        self._stream_modes = frozenset(_DEFAULT_STREAM_MODES)

        # ── SpaceMouse/GUI follower (Phase 3) ───────────────────
        self._follower = TargetFollower(
            self._geom, horizon_s=float(hw.JB_TRAJ_SPACEMOUSE_HORIZON_S))
        # Latest streaming target as an atomic (pose_6dof, perf_counter) tuple —
        # written by the platform_pose callback (executor thread), drained by the
        # emitter thread (drain-to-latest: only the newest tuple is ever read).
        self._follower_target = None
        self._follower_input_lost = False
        # perf_counter at follower-mode entry — a None target within the input-loss
        # window SINCE ENTRY is the pre-first-frame grace period, NOT input loss.
        self._follower_entry_mono = None
        # A graceful stop is wanted but build_graceful_stop currently rejects the
        # seed (near-boundary outward-moving: decel overshoot > boundary margin).
        # _emit_once retries it every tick from the decaying live state until it
        # converges — instead of latching failure. Set by mode-exit AND input-loss.
        self._pending_stop = False
        # A3 escalation latch: set when the follower reports escalate=True (sustained
        # chase rejects → a corrupt / boundary-wedged seed). While set, target
        # freshness does NOT clear _pending_stop (so the stop machinery keeps running
        # through a live 40 Hz stream); cleared ONLY by a follower plan accepted +
        # installed, a reseed, or leaving the follower mode. _escalate_stop_fail_count
        # counts consecutive failed stop builds while latched → the HoldPlan backstop.
        self._escalation_stop = False
        self._escalate_stop_fail_count = 0
        # Gravity-levelling correction (identity = none), composed into follower
        # target orientations — ported from mpc_bridge_node.
        self._gravity_correction = np.eye(3)

        # ── Plan + streaming state (written on ROS + emitter threads) ──
        self._plan_lock = threading.Lock()
        self._active_plan = None          # current TrajectoryPlan (None until seeded)
        self._plan_t0 = None              # perf_counter at install (plan-time origin)
        self._seeded = False              # a real telemetry seed has been installed
        self._streaming = False           # current mode is a streaming mode
        # FIX 1 — Teensy guard-latch freeze. Set True on the RISING edge of a latched
        # guard fault (read off the bridge's /link_status) WHILE streaming: the
        # emitter keeps publishing (so the stream never trips MPC_STALE) but no
        # planning advances u0 — hold-at-measured persists — until CLEAR_ERRORS
        # releases the guard. Written on the executor thread (_on_link_status /
        # _on_control_mode), read on the emitter thread (_post_publish_planning); a
        # plain bool is GIL-atomic across the one-writer/one-reader split.
        self._guard_frozen = False
        self._current_mode = ''
        # ── Arming contract (A5, see ARMING_CONTRACT.md) ──────────
        # The wire's arming state, mirrored from /link_status 'mpc_active'.
        # This node has no authority over arming — it only makes the disarmed
        # wire LOUD: an accepted motion command while disarmed executes into a
        # dropped stream (the 2026-07-15 silent no-op battery: two moves
        # "accepted", realized peaks on /trajectory/diagnostics, zero motion,
        # setpoints_sent frozen at 0). Plain bool across threads (GIL-atomic).
        self._wire_armed = False
        # is_homed from robot_state — gates SEEDING (A5): mid-homing the legs sit
        # at arbitrary index-search positions below the workspace hard margin, so
        # every seed attempt failed the gate at the 100 Hz telemetry rate (4091
        # ERROR lines in 41 s on 2026-07-15, burying the real fault). Before
        # homing completes there is nothing valid to hold anyway.
        self._is_homed = False
        self._seq = 0
        self._last_motor_rev = None       # prior emitted u0 (step-bound defence)
        self._last_pose = self._neutral_pose.copy()
        # Latest emitted frame's gravity-FF torque vector (TRUE Nm, extension-
        # positive) — stashed by the 40 Hz emitter, republished at 5 Hz on
        # leg_torques_diagnostic by _publish_status. None until the first frame
        # ships (so nothing publishes before frame 1). One writer (the emitter
        # thread), one reader (_publish_status): a plain reference assignment is
        # GIL-atomic — no lock, no copy (the reader treats it as immutable).
        self._last_torque_Nm = None
        self._last_rejection = ''
        # Latest robot_state seed source (pos_estimate rev, Jugglebot ext convention).
        self._latest_pos_rev = None
        self._robot_state_mono = 0.0
        # Emitter jitter diagnostic (max inter-tick gap over the last window).
        self._max_emit_gap_s = 0.0
        self._last_emit_mono = None
        # Post-publish follow-block cost diagnostic (C3): max duration of the
        # pending-stop retry + follower replan block over the last window. Exposed
        # alongside _max_emit_gap_s so /diagnose can confirm the block stays far
        # under the ~117 ms decay→sprint threshold. `_last_alpha` is the follower's
        # last chase progress fraction (1.0 = reached, <1 = capped, 0 = stop).
        self._max_follow_block_s = 0.0
        self._last_alpha = 1.0
        # Measured leg peaks of the last accepted move (for trajectory/diagnostics).
        # `_last_peak_*` are the gate's PREDICTED peaks (fine-sampled, from the report
        # at plan time). `_run_peak_*` are the REALIZED peaks tracked from the actual
        # emitted knots as the active plan executes (Phase 4 ramp observability),
        # reset per install. `_lean_gain_active` is the gain baked into the active
        # plan (0.0 for a hold / lean-off move) — published so /diagnose can label
        # the A/B arm each move belongs to.
        self._last_peak_vel_mmps = 0.0
        self._last_peak_acc_mmps2 = 0.0
        self._last_peak_jerk_mmps3 = 0.0
        self._run_peak_vel_mmps = 0.0
        self._run_peak_acc_mmps2 = 0.0
        self._run_peak_jerk_mmps3 = 0.0
        self._prev_frame_leg_acc = None    # for the realized-jerk tick difference
        self._lean_gain_active = 0.0
        # Monotonic per-move install counter, published on trajectory/diagnostics so
        # /diagnose can segment consecutive go_to_pose moves. `plan.kind` stays 'move'
        # for a completed move's whole terminal-hold lifetime, so back-to-back moves
        # (the ramp battery inserts NO hold between them) carry an unbroken
        # plan_kind='move' run — move_seq is the only unambiguous per-move boundary.
        # Bumped ONLY on a discrete accepted go_to_pose (NOT per follower replan), so
        # a spacemouse stream stays one window rather than thousands.
        self._move_seq = 0

        self._pub = None
        self._emit_stop = threading.Event()
        self._emit_thread = None

        # ── Subscriptions ───────────────────────────────────────
        mode_topic = str(self.get_parameter('control_mode_topic').value)
        self.create_subscription(String, mode_topic, self._on_control_mode, 10)
        self.create_subscription(RobotState, 'robot_state',
                                 self._on_robot_state, 10)
        # SpaceMouse / GUI streaming target (Phase 3).
        self.create_subscription(PlatformPoseCommand, 'platform_pose_topic',
                                 self._on_platform_pose, 10)
        # Gravity-levelling correction (verbatim port from mpc_bridge_node).
        self.create_subscription(Float64MultiArray, 'gravity_offset',
                                 self._on_gravity_offset, 10)
        # Catch coordinator's timed catch target (latch-gated; Phase 5).
        self.create_subscription(DynamicTargetCommand, 'catch/dynamic_target',
                                 self._on_dynamic_target, 10)
        # FIX 1 — Teensy guard state. The bridge publishes /link_status (a
        # DiagnosticStatus) at 10 Hz carrying the guard 'fault_state' KeyValue; we
        # watch it so a latched E-STOP freezes command advancement instead of letting
        # u0 run away (the 2026-07-10 stuttering-stroke runaway).
        self.create_subscription(DiagnosticStatus, 'link_status',
                                 self._on_link_status, 10)

        # ── Services ────────────────────────────────────────────
        self.create_service(Trigger, 'trajectory/hold', self._svc_hold)
        self.create_service(Trigger, 'trajectory/go_home', self._svc_go_home)
        self.create_service(GoToPose, 'trajectory/go_to_pose',
                            self._svc_go_to_pose)
        self.create_service(SetTrajectoryLimits, 'trajectory/set_limits',
                            self._svc_set_limits)
        self.create_service(TimedTarget, 'trajectory/timed_target',
                            self._svc_timed_target)
        # Catch-armed latch (reload-action-catch-latch plan, Phase 1). SetBool:
        # data=True arms, data=False disarms. The RELOAD action drives it for the
        # catch flight window instead of a persistent control mode.
        self.create_service(SetBool, 'trajectory/arm_catch', self._svc_arm_catch)
        # FIX 3 — one-call guard recovery support. The bridge's /recover calls this
        # FIRST (before CLEAR_ERRORS) so the commanded u0 collapses onto the frozen
        # encoder and the subsequent clear cannot re-latch MAX_DEVIATION.
        self.create_service(Trigger, 'trajectory/reseed_from_measured',
                            self._svc_reseed_from_measured)

        # ── Status + diagnostics publication (5 Hz) ─────────────
        self.status_pub = self.create_publisher(
            TrajectoryStatus, 'trajectory/status', 10)
        self.diagnostics_pub = self.create_publisher(
            DiagnosticStatus, 'trajectory/diagnostics', 10)
        # Accept/reject feedback for timed/catch targets — catch_coordinator_node
        # consumes this in place of the dormant MPC process's ZMQ :5559 feedback
        # (blacklist semantics preserved).
        self.target_feedback_pub = self.create_publisher(
            TargetFeedback, 'trajectory/target_feedback', 10)
        # Feedforward leg torques on leg_torques_diagnostic — the SAME topic name,
        # Float64MultiArray type and semantic (6 leg values, TRUE Nm, extension-
        # positive) that motion_bridge_node publishes on the MPC path, so the
        # rosbag record list / GUI / any consumer see identical data regardless of
        # which producer is live (the two paths are mutually exclusive on :5557).
        # Two publishers on one topic is fine in ROS2 as long as the types match.
        # Published at 5 Hz by _publish_status; the wire truth is the 40 Hz emitter
        # and this is a sample of it (a diagnostic — 5 Hz is plenty).
        self.leg_torques_pub = self.create_publisher(
            Float64MultiArray, 'leg_torques_diagnostic', 10)
        self.create_timer(0.2, self._publish_status)
        # Re-measure the ROS↔perf clock offset every 30 s to track drift (mirrors
        # catch_coordinator_node). NOTE: since the lead_time_s interface change the
        # timed_target service no longer converts through it (relative lead, perf-
        # anchored at handler entry) — the offset is retained, consumer-less, for
        # any future ROS-clock-timed surface.
        self.create_timer(30.0, self._refresh_clock_offset)

        if start_emitter:
            self._start_emitter()

        self.get_logger().info(
            f"trajectory_node up — {1.0 / hw.JB_TRAJ_KNOT_DT_S:.0f} Hz hold "
            "emitter on :5557 (mpccmd); "
            f"stream modes={sorted(self._stream_modes)}, "
            f"go_home={self._go_home_duration_s}s")

    # ═══════════════════════════════════════════════════════════
    # Emitter thread (dedicated; absolute-deadline 40 Hz)
    # ═══════════════════════════════════════════════════════════

    def _start_emitter(self) -> None:
        self._emit_stop.clear()
        self._emit_thread = threading.Thread(
            target=self._emitter_loop, name='trajectory_emitter', daemon=True)
        self._emit_thread.start()

    def _emitter_loop(self) -> None:
        # Bind the PUB INSIDE the thread (sole-binder interlock). A bind failure
        # ⇒ run_mpc.py is running; refuse to start rather than fight the wire.
        try:
            self._pub = (self._pub_factory() if self._pub_factory is not None
                         else MpcCommandPub())
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(
                "trajectory_node: could not bind the :5557 command PUB "
                f"({e}). Is run_mpc.py running? The emitter will NOT start; "
                "stop the MPC process and relaunch.")
            return

        # The emitter period IS the knot spacing: the can-hub firmware pins its
        # segment time to 25 ms, so this MUST match hw.JB_TRAJ_KNOT_DT_S (the dt
        # the KnotEmitter samples the plan at). Derive it — never hardcode 40 Hz.
        period = hw.JB_TRAJ_KNOT_DT_S
        next_deadline = time.perf_counter()
        try:
            while not self._emit_stop.is_set():
                now = time.perf_counter()
                try:
                    self._emit_once(now)
                except Exception as e:  # noqa: BLE001 — one frame must not kill the loop
                    self.get_logger().error(
                        f"emitter tick error (contained): {e}",
                        throttle_duration_sec=1.0)
                next_deadline += period
                sleep_dt = next_deadline - time.perf_counter()
                if sleep_dt > 0:
                    self._emit_stop.wait(sleep_dt)
                else:
                    # Overrun: reset the schedule so we don't spin catching up.
                    next_deadline = time.perf_counter()
        finally:
            try:
                self._pub.close()
            except Exception:  # noqa: BLE001
                pass

    def _emit_once(self, now: float) -> None:
        """Emit one knot frame for wall time ``now`` (perf_counter). Public for tests.

        Publish-first ordering (C3): sample + publish this tick's knot from the plan
        installed on a PRIOR tick, THEN run the pending-stop retry and follower
        replan (which install the plan the NEXT tick samples). The knot publish never
        waits on any planning, so a slow (or raising) replan can never delay or gap
        the wire stream — the fix for the S3 44-146 ms emit gaps that fed the
        firmware decay→sprint bursts. Cost: one tick (25 ms) of stick→platform
        latency, accepted. C2 is preserved — the follower still seeds from the active
        plan sampled at its seed time (``_current_state``) and installs with
        ``t0 = seed_mono``; the reorder changes only WHEN the install lands (after
        this tick's publish), not the seeding rule.
        """
        # Jitter diagnostic (gap-diag).
        if self._last_emit_mono is not None:
            self._max_emit_gap_s = max(self._max_emit_gap_s,
                                       now - self._last_emit_mono)
        self._last_emit_mono = now

        with self._plan_lock:
            plan = self._active_plan
            t0 = self._plan_t0
            streaming = self._streaming and self._seeded
            mode = self._current_mode
        if not streaming or plan is None or t0 is None:
            return

        # ── Sample + publish FIRST (before any planning) ──
        tau = now - t0
        frame = self._emitter.frame(plan, tau, self._seq)
        motor_rev = np.asarray(frame['motor_rev'], dtype=float)

        # Defence-in-depth per-knot step bound (the gate should already prevent
        # this; this is the last line before the wire). On violation: freeze to a
        # hold at the last good pose and skip this frame. The next tick emits the
        # hold (u0 == last good ⇒ zero step) so streaming resumes within one 25 ms
        # knot — far inside the 250 ms staleness window.
        if self._last_motor_rev is not None:
            step = float(np.max(np.abs(motor_rev - self._last_motor_rev)))
            if step > self._limits.max_step_rev:
                self._last_rejection = (
                    f"emitter step {step:.3f} rev > {self._limits.max_step_rev} "
                    "bound — froze to hold")
                self.get_logger().error(
                    self._last_rejection, throttle_duration_sec=1.0)
                # Exempt from the planner gate on purpose: this backstop must
                # NEVER raise, so install the hold DIRECTLY. self._last_pose is a
                # known-good, already-gated pose (the last frame we shipped).
                with self._plan_lock:
                    self._active_plan = HoldPlan(self._last_pose)
                    self._plan_t0 = now
                return

        self._pub.send(frame)
        self._last_motor_rev = motor_rev
        self._last_pose = np.asarray(frame['pose_6dof'], dtype=float)
        # Stash (only) the FF torque vector this frame shipped, for the 5 Hz
        # leg_torques_diagnostic republish (_publish_status). This is the ONLY
        # work the 40 Hz hot path does for that diagnostic: a plain reference
        # assignment (GIL-atomic; no lock, no copy — the reader never mutates it).
        # `frame['torque_Nm']` is a fresh (6,) array per emitter frame — exact
        # zeros when the FF master flag is off, nonzero gravity FF when on.
        self._last_torque_Nm = frame['torque_Nm']
        self._seq += 1
        # Pass the plan this frame was sampled FROM: an install on the service thread
        # can swap the active plan (and reset the accumulators) between the frame
        # sample above and this call, and a stale frame must not contaminate the new
        # move's realized window (the install-straddle finding).
        self._track_realized_peaks(frame, plan)

        # ── Post-publish planning: pending-stop retry then follower replan ──
        # Bounded (A4 max_iters, contained exceptions) so the worst block stays far
        # under the ~117 ms firmware decay→sprint threshold; whatever installs here
        # is what the NEXT tick samples. Timed for the follow-block diagnostic.
        block_t0 = time.perf_counter()
        try:
            self._post_publish_planning(now, mode)
        except Exception as e:  # noqa: BLE001 — planning must never kill the stream
            self.get_logger().error(
                f"post-publish planning error (contained): {e}",
                throttle_duration_sec=1.0)
        self._max_follow_block_s = max(self._max_follow_block_s,
                                       time.perf_counter() - block_t0)

    def _post_publish_planning(self, now: float, mode: str) -> None:
        """Run this tick's planning AFTER the knot is on the wire (C3).

        Pending graceful-stop retry (any streaming mode) then the follower replan
        (follower modes only). Each installs the plan the next tick samples.
        """
        # FIX 1 — while a guard E-STOP is latched, run NO new re-planning: the emitter
        # (caller) still publishes this tick's frame, and it plays out the PROFILED
        # DESCENT the latch edge installed — walking u0 DOWN onto the frozen encoder,
        # each knot inside the pump step gate — but no follower replan or pending-stop
        # retry may SUPERSEDE that descent. Without this, the follower would keep
        # chasing the latest SpaceMouse target and re-diverge u0 the instant a clear
        # released the latch. The installed descent itself advances u0 (that IS the
        # command-space collapse); it is the re-planning, not the emitter, that freezes.
        if self._guard_frozen:
            return
        if self._pending_stop:
            self._retry_pending_stop(now)
        if mode in _FOLLOWER_MODES:
            try:
                self._follower_tick(now)
            except Exception as e:  # noqa: BLE001 — one replan must not kill streaming
                self.get_logger().error(
                    f"follower tick error (contained): {e}",
                    throttle_duration_sec=1.0)

    def _retry_pending_stop(self, now: float) -> None:
        """Retry the deferred graceful stop from the live (decaying) state.

        A stop was requested (mode-exit mid-move, follower input-loss, or an A3
        escalation) but build_graceful_stop rejected the seed then — a near-boundary
        outward-moving state whose decel overshoot (~0.2·v·T) exceeds its boundary
        margin. The platform's velocity decays under the still-running gated plan, so
        retry from the live state until it converges (a few ticks), then install and
        clear the flag. Bounded by ``_STOP_RETRY_ITERS`` (A4).

        Escalation backstop (A3): while the escalation latch is set, count the
        consecutive failed builds; after ``_ESCALATE_HOLD_TICKS`` install a HoldPlan
        at the last emitted (known-good) pose DIRECTLY — a corrupt seed that never
        becomes stoppable still gets a terminal, loud action instead of a silent
        never-ending keep-last.
        """
        try:
            stop = planner.build_graceful_stop(
                self._current_state(), self._limits, self._geom,
                max_iters=_STOP_RETRY_ITERS)
        except TrajectoryInfeasible:
            # Still overshooting / spatially infeasible — keep the gated plan.
            if self._escalation_stop:
                self._escalate_stop_fail_count += 1
                if self._escalate_stop_fail_count >= _ESCALATE_HOLD_TICKS:
                    with self._plan_lock:
                        self._active_plan = HoldPlan(self._last_pose)
                        self._plan_t0 = now
                    self._escalate_stop_fail_count = 0
                    self.get_logger().error(
                        "escalation stop did not converge — holding at the last "
                        "emitted pose (corrupt-seed backstop)",
                        throttle_duration_sec=1.0)
            return
        self._install(stop)
        self._pending_stop = False
        self._escalate_stop_fail_count = 0
        # Latch input-loss too so the follower branch below doesn't immediately
        # re-stop (a fresh target un-latches it — unless the escalation latch holds).
        self._follower_input_lost = True
        self.get_logger().info(
            "pending graceful stop converged — installed (move silenced)")

    def _track_realized_peaks(self, frame: dict, sampled_plan) -> None:
        """Update the per-move realized leg peaks from an emitted knot frame.

        The emitted ``vel_mm_s`` / ``acc_mm_s2`` are the exact leg vel/acc the wire
        carries (analytic, via the emitter's Jacobian chain); realized jerk is the
        40 Hz tick difference of the emitted leg accel — a coarse (knot-rate) proxy
        for what the Teensy Hermite reproduces between knots, distinct from the gate's
        fine-sampled PREDICTED jerk. Reset per install (see ``_install``). This is the
        Phase-4 ramp-observability signal (`/diagnose` reads it back from the rosbag).

        The accumulation runs under ``_plan_lock`` and is SKIPPED if ``sampled_plan``
        is no longer the active plan: an install between the frame sample and here has
        already reset the accumulators for the new move, so folding this old-plan
        frame in would report a stale sample as the new move's peak (and would also
        race the install's reset of ``_prev_frame_leg_acc``).
        """
        leg_acc = np.asarray(frame['acc_mm_s2'], dtype=float)
        with self._plan_lock:
            if self._active_plan is not sampled_plan:
                return
            leg_vel = np.abs(np.asarray(frame['vel_mm_s'], dtype=float))
            self._run_peak_vel_mmps = max(self._run_peak_vel_mmps,
                                          float(np.max(leg_vel)))
            self._run_peak_acc_mmps2 = max(self._run_peak_acc_mmps2,
                                           float(np.max(np.abs(leg_acc))))
            if self._prev_frame_leg_acc is not None and self._emitter._dt > 0.0:
                jerk = np.abs(leg_acc - self._prev_frame_leg_acc) / self._emitter._dt
                self._run_peak_jerk_mmps3 = max(self._run_peak_jerk_mmps3,
                                                float(np.max(jerk)))
            self._prev_frame_leg_acc = leg_acc

    # ═══════════════════════════════════════════════════════════
    # Subscriptions
    # ═══════════════════════════════════════════════════════════

    def _reset_catch_reach_freeze(self) -> None:
        """Release any committed catch-reach freeze window (``_catch_arrival_perf``
        → None). The single named enforcement point for "a fresh catch session (or an
        abort) must not inherit a stale freeze", called from the catch-armed latch
        edges and a fresh telemetry seed."""
        self._catch_arrival_perf = None

    def _install_graceful_stop(self, context: str) -> None:
        """Install a graceful decel-to-rest from the LIVE commanded state — a C2
        duration-stretched stop that decelerates IN PLACE (never runs on to the old
        target). Shared by the mode-transition mid-move path (``_on_control_mode``)
        and the catch-armed latch edges (``_svc_arm_catch``): both must silence an
        in-flight move at the transition without a command discontinuity, and both do
        it identically. On a near-boundary outward-moving seed the in-place stop may
        not yet be in-stroke; flag a pending stop so ``_emit_once`` retries from the
        decaying live state rather than latching failure (the move keeps running under
        its still-gated plan meanwhile)."""
        try:
            stop = planner.build_graceful_stop(
                self._current_state(), self._limits, self._geom,
                max_iters=_STOP_RETRY_ITERS)
            self._install(stop)
            self.get_logger().info(
                f"{context} mid-move — installed a graceful stop (move silenced)")
        except TrajectoryInfeasible as e:
            self._request_pending_stop(str(e))

    def _on_control_mode(self, msg) -> None:
        mode = str(msg.data)
        if mode == self._current_mode:
            return
        # SAFETY (reload-action-catch-latch): the catch-armed latch is owned by the RELOAD
        # action, which runs ENTIRELY within TRAJECTORY. Any mode transition therefore means
        # the reload is no longer active — an E-STOP to STANDBY, an operator abort/deactivate,
        # or the action dying without lowering the latch. Force-disarm here, restoring the
        # safety the retired CATCH mode gave for free (the old gate was `_current_mode !=
        # CATCH`, so leaving CATCH inherently disarmed the catch). Without this a stuck-armed
        # latch would let a stray `catch/dynamic_target` install a `build_catch` reach with no
        # reload in progress ("catch fires when it shouldn't"). Clear the flag + release the
        # freeze window here (mirrors the disarm edge in `_svc_arm_catch`); an in-flight catch
        # reach is silenced by the mid-move graceful-stop (leaving a motion mode) or the
        # emitter-silence (leaving the stream set) logic in the branches below.
        if self._catch_armed:
            self._catch_armed = False
            self._reset_catch_reach_freeze()
            self._catch_envelope_center = None
            self.get_logger().info(
                f"catch latch force-disarmed on mode change "
                f"{self._current_mode}→{mode}")
        # A3: the escalation + pending-stop latches are cleared on ANY mode change,
        # BEFORE the mid-move stop logic below (which independently re-requests a stop
        # if this transition needs one). A mode change is a fresh context — a stale
        # escalation/pending stop from the prior mode must not leak across it. The
        # clear is done ATOMICALLY
        # with the _current_mode write under _plan_lock (in both branches below) so a
        # follower escalation racing this exit on the emitter thread — its follow() in
        # flight — cannot re-latch the flags into the new mode: _enter_escalation_stop
        # re-checks the mode under the same lock and refuses once we have left.
        if mode in self._stream_modes:
            # Leaving a motion mode (TRAJECTORY / a follower) for a non-motion
            # streaming mode (STANDBY) while a move is in flight: STANDBY silences
            # commands but keeps streaming, so the in-flight move must be stopped
            # with a C2 decel-to-rest rather than run on to its target. Snapshot
            # this BEFORE mutating _current_mode; _active_move_in_flight locks
            # itself.
            prev_mode = self._current_mode
            move_in_flight = self._active_move_in_flight()
            leaving_motion_move = (
                prev_mode in _MOTION_MODES and mode not in _MOTION_MODES
                and move_in_flight)
            entering_follower = (mode in _FOLLOWER_MODES and mode != prev_mode)
            # Streaming-state writes go under _plan_lock: the emitter thread
            # snapshots _streaming/_seeded under it in _emit_once, so an unlocked
            # write could tear against that snapshot.
            with self._plan_lock:
                self._current_mode = mode
                self._streaming = True
                self._escalation_stop = False
                self._escalate_stop_fail_count = 0
                self._pending_stop = False
            if entering_follower:
                # Drop any stale target/last-target so the first target of this
                # session always replans (never deadbanded against a prior session).
                self._follower.reset()
                self._follower_target = None
                self._follower_input_lost = False
                # Start the input-loss grace clock: a None target is not "input
                # loss" until this window elapses since entry (before the first
                # SpaceMouse frame even arrives).
                self._follower_entry_mono = time.perf_counter()
            if leaving_motion_move:
                # Sample the live (moving) state and install a graceful stop: a
                # duration-stretched decel-to-rest that lengthens its horizon until
                # the gate passes, so a mid-move mode change no longer falls through to
                # "let the move complete". The stop decelerates in place (never runs on
                # to the old target). _current_state/_install lock themselves.
                self._install_graceful_stop(f"changed mode {prev_mode}→{mode}")
            # Seed a hold at the MEASURED pose on stream start (never nominal) —
            # this is what keeps the first u0 inside the pump/firmware gates — but
            # ONLY from FRESH telemetry. Stale/absent telemetry ⇒ defer: the next
            # _on_robot_state callback (inherently fresh) performs the seed.
            # NB: _seed_hold_from acquires _plan_lock itself — call it UNLOCKED.
            if not self._seeded:
                if not self._is_homed:
                    # A5 — no seeding before homing completes (the workspace gate
                    # would reject every attempt; see _on_robot_state).
                    self.get_logger().warning(
                        "streaming mode entered before homing completed — "
                        "seeding deferred until is_homed")
                elif self._robot_state_fresh():
                    self._seed_hold_from(self._latest_pos_rev)
                else:
                    self.get_logger().error(
                        "telemetry stale — waiting for fresh robot_state before "
                        "seeding")
            # NB "ENABLED" means "will emit once seeded" — the emitter itself is
            # gated on _streaming AND _seeded (a rejection above does not stop
            # this log; the 40 Hz frames start only after a successful seed).
            self.get_logger().info(
                f"streaming ENABLED (mode {mode})"
                + ("" if self._seeded else " — awaiting seed"))
        else:
            # Leaving the streaming set: stop publishing and require a fresh seed
            # on the next entry (the measured pose may have moved meanwhile).
            with self._plan_lock:
                self._current_mode = mode
                self._streaming = False
                self._seeded = False
                self._last_motor_rev = None
                # Drop the torque stash too (audit 2026-07-16): without this, a
                # 5 Hz status tick landing in the re-entry window (reseeded but
                # before the first new frame) would republish the PREVIOUS
                # stream's last torque vector on leg_torques_diagnostic.
                self._last_torque_Nm = None
                self._escalation_stop = False
                self._escalate_stop_fail_count = 0
                self._pending_stop = False
            # FIX 1 — leaving the streaming set is a fresh context: drop the guard
            # freeze (nothing to hold now that the emitter is silent). If the guard
            # is still latched, the next _on_link_status re-computes should_freeze as
            # False (streaming is off) and re-arms cleanly on the next stream entry.
            self._guard_frozen = False
            self.get_logger().info(f"streaming DISABLED (mode {mode})")

    def _on_robot_state(self, msg) -> None:
        states = getattr(msg, 'motor_states', None)
        if not states or len(states) < _NUM_LEGS:
            return
        pos_rev = [float(states[i].pos_estimate) for i in range(_NUM_LEGS)]
        self._latest_pos_rev = pos_rev
        self._robot_state_mono = time.perf_counter()
        self._is_homed = bool(getattr(msg, 'is_homed', False))
        # Seed on the first telemetry after streaming is enabled — but never
        # before homing completes (A5): mid-homing extensions sit below the
        # workspace hard margin, so seeding can only fail (and at 100 Hz the
        # rejection spam buried the real fault on 2026-07-15).
        if self._streaming and not self._seeded and self._is_homed:
            self._seed_hold_from(pos_rev)

    def _robot_state_fresh(self) -> bool:
        """True iff a robot_state seed exists AND is within the staleness window."""
        return (self._latest_pos_rev is not None
                and (time.perf_counter() - self._robot_state_mono)
                <= self._robot_state_stale_s)

    def _seed_hold_from(self, pos_rev) -> bool:
        """Seed a hold at the measured pose (pos_estimate rev → ext → FK → gate).

        Returns True iff a hold was installed. The guard-recovery reseed (FIX 1/3)
        needs a definitive install signal, so every early-out returns False and the
        install path returns True; the other callers ignore the value (statement use).
        """
        # Defensive freshness gate: never seed from stale/absent telemetry — a
        # stale measured pose could place the first u0 outside the pump/firmware
        # gates. The _on_robot_state seed path is inherently fresh (mono stamped
        # immediately before the call), so this only guards a stray caller.
        if not self._robot_state_fresh():
            self.get_logger().error(
                "telemetry stale — waiting for fresh robot_state before seeding")
            return False
        ext_mm = np.asarray(pos_rev, dtype=float) / self._mm_to_rev
        try:
            pos, rot, _J = leg_lengths_to_pose(ext_mm, self._geom)
        except RuntimeError as e:
            self.get_logger().error(
                f"seed FK failed ({e}) — not streaming until a valid state")
            return False
        pose = np.concatenate([pos, rot_matrix_to_rotvec(rot)])
        # Route the seed through the canonical gate: build_hold validates the pose
        # (workspace + limits + finiteness) before install, so nothing — not even
        # the telemetry seed — bypasses feasibility.validate.
        try:
            plan = planner.build_hold(
                (pose, np.zeros(6), np.zeros(6)), self._limits, self._geom)
        except TrajectoryInfeasible as e:
            self._last_rejection = str(e)
            self.get_logger().error(
                f"seed hold rejected by gate ({e}) — not streaming until a valid "
                "state", throttle_duration_sec=1.0)
            return False
        now = time.perf_counter()
        with self._plan_lock:
            self._active_plan = plan
            self._plan_t0 = now
            self._seeded = True
            self._last_motor_rev = None
            self._last_pose = pose
        # A fresh seed changes the commanded pose; the follower must not deadband
        # the first target against a stale one from a previous session.
        self._follower.reset()
        self._follower_target = None
        self._follower_input_lost = False
        self._pending_stop = False   # fresh at-rest seed supersedes any pending stop
        self._escalation_stop = False       # A3: a reseed clears the escalation latch
        self._escalate_stop_fail_count = 0
        self._reset_catch_reach_freeze()  # a fresh seed releases any frozen catch reach
        self.get_logger().info(
            f"seeded hold at pose x={pose[0]:.1f} y={pose[1]:.1f} "
            f"z={pose[2]:.1f} mm (from measured telemetry)")
        return True

    # ═══════════════════════════════════════════════════════════
    # Teensy guard-latch handling (FIX 1)
    # ═══════════════════════════════════════════════════════════

    @staticmethod
    def _kv_get(values, key: str, default: str = '') -> str:
        """Read a DiagnosticStatus KeyValue by key (the /link_status decode)."""
        for kv in values:
            if kv.key == key:
                return kv.value
        return default

    def _reseed_hold_at_measured(self) -> bool:
        """Instantly reseed a hold at the measured encoder pose (a ZERO-step reseed).

        Installs ``build_hold(measured)`` — an INSTANT collapse of the commanded u0
        onto the live encoder. This is safe ONLY when the command is already within
        the pump's per-frame step gate of measured: a >``max_step_rev`` u0 jump is
        REJECTED by the pump (it gates against the prior ACCEPTED frame), which
        deadlocks the stream — the 2026-07-10 flaw. So callers that cannot guarantee
        the command is already close (the latch edge, /recover's reseed) use
        ``_install_guard_descent`` instead, which walks u0 down THROUGH the gate. This
        instant reseed survives only on the guard-CLEAR edge, guarded by
        ``_command_within_step_of_measured``.

        Permitted in ALL streaming modes (an explicit recovery reseed, not a
        mode-entry seed). No-op (returns False) when not streaming or telemetry is
        stale. Does NOT touch ``_guard_frozen`` (the freeze is owned by
        ``_on_link_status``).
        """
        with self._plan_lock:
            streaming = self._streaming
        if not streaming:
            return False
        if self._latest_pos_rev is None or not self._robot_state_fresh():
            self.get_logger().error(
                "guard reseed skipped — no fresh robot_state to seed from")
            return False
        return self._seed_hold_from(self._latest_pos_rev)

    def _measured_pose(self):
        """FK of the live encoder → the 6-DOF pose the (frozen) legs are sitting at.

        Reads ``self._latest_pos_rev`` FRESH every call (never a cached snapshot): this
        is the descent's single re-sample point, and during a guard latch the leg keeps
        drifting ~0.1 rev as the ODrive closes its frozen lead offset (the 2026-07-11
        Event-1 −0.102 rev plateau). A descent onto a stale install-time pose would leave
        u0 off the drifted encoder → the bridge's convergence verify plateaus above tol.
        Re-sampling here is what lets each re-install (the bridge's bounded re-descend)
        chase the settling leg.

        Returns ``None`` when telemetry is absent/stale or the FK fails — the caller
        then freezes in place rather than descend toward an unknown target."""
        if self._latest_pos_rev is None or not self._robot_state_fresh():
            return None
        ext_mm = np.asarray(self._latest_pos_rev, dtype=float) / self._mm_to_rev
        try:
            pos, rot, _J = leg_lengths_to_pose(ext_mm, self._geom)
        except RuntimeError as e:
            self.get_logger().error(f"guard descent FK failed ({e})")
            return None
        return np.concatenate([pos, rot_matrix_to_rotvec(rot)])

    def _command_within_step_of_measured(self) -> bool:
        """True iff the current commanded u0 is within the pump step gate of measured.

        i.e. an instant ``build_hold(measured)`` reseed would NOT exceed the pump's
        per-frame step gate. Gates the guard-CLEAR-edge instant reseed: a converged
        descent (the /recover happy path, u0 within 0.25 rev of measured) reseeds a
        clean hold; a still-in-flight descent (a mid-descent bare /clear_errors, u0
        still up to the 1.0 rev guard band out) is left to finish rather than jump u0
        across the gate. Any sampling/FK error ⇒ treat as unsafe (do not reseed)."""
        if self._latest_pos_rev is None:
            return False
        try:
            cmd_rev = self._pose_to_motor_rev(self._current_state()[0])
        except Exception:  # noqa: BLE001 — a sampling/FK error ⇒ treat as unsafe
            return False
        meas = np.asarray(self._latest_pos_rev, dtype=float)
        return bool(np.max(np.abs(cmd_rev - meas)) <= self._limits.max_step_rev)

    def _freeze_in_place(self) -> None:
        """Stop the runaway by holding u0 at the last emitted pose (zero step).

        The last-ditch fallback when no collapsing descent can be built (stale
        telemetry, or a descent the gate rejects even after an in-place decel). u0
        stays diverged from the encoder, so /recover cannot converge and recovery
        needs the manual dance — but at least the command STOPS running away
        (``self._last_pose`` is the last already-gated frame we shipped)."""
        with self._plan_lock:
            self._active_plan = HoldPlan(self._last_pose)
            self._plan_t0 = time.perf_counter()
            self._last_motor_rev = None

    def _build_descent_plan(self, seed, target):
        """Gate-validated profiled trajectory ``seed → target`` ending at rest.

        ``seed`` is the CURRENT COMMANDED ``(pose, twist, accel)`` (C2 continuation);
        ``target`` is the measured pose. Built with ``build_follow`` (the fast
        ``validate_follow`` gate), so EVERY emitted knot's u0 step stays under the
        gate's own knot-step bound (0.8·``max_step_rev``) — and thus under the pump's
        ``max_step_rev`` — unlike the >0.5 rev instant-reseed jump the pump rejected.

        Fallback for a gate-rejected direct descent (a near-boundary, outward-moving
        commanded seed whose rest-terminating overshoot leaves the workspace): decel
        IN PLACE first (``build_graceful_stop`` — minimal outward excursion) then a
        descent FROM REST → target, concatenated into one plan. The join is C2 (both
        halves are at rest at the stop's pose, so it is a zero-step, zero-velocity
        abut → every concatenated knot still passes the pump gate). Returns the plan,
        or ``None`` if every attempt is gate-rejected."""
        try:
            plan, _report = planner.build_follow(
                seed, target, self._limits, self._geom,
                self._limits.min_move_duration_s)
            return plan
        except TrajectoryInfeasible:
            pass
        # Direct descent rejected — decel in place, then descend from rest.
        try:
            stop = planner.build_graceful_stop(seed, self._limits, self._geom)
        except TrajectoryInfeasible:
            return None
        rest_pose = np.asarray(stop.final_pose, dtype=float)
        try:
            descent, _report = planner.build_follow(
                (rest_pose, np.zeros(6), np.zeros(6)), target,
                self._limits, self._geom, self._limits.min_move_duration_s)
        except TrajectoryInfeasible:
            return None
        return TrajectoryPlan(
            segments=tuple(stop.segments) + tuple(descent.segments),
            final_pose=descent.final_pose)

    def _install_guard_descent(self) -> bool:
        """Walk the commanded stream DOWN to the measured pose through the pump gate.

        Replaces the latch-edge instant reseed. The 2026-07-10 runaway left u0 ~0.5+
        rev past the frozen encoder; an instant ``build_hold(measured)`` there is a
        >``max_step_rev`` u0 jump the pump's 0.3 rev step gate REJECTS (it compares
        against the prior ACCEPTED frame), so the collapse never reached the Teensy
        and /recover could never converge. Instead we install a gate-validated
        PROFILED descent from the CURRENT COMMANDED state to the MEASURED pose (FK of
        the live encoder), ending at rest, at the CURRENT session limits — the emitter
        plays it out and u0 converges onto the encoder over ~0.5-3 s, each knot inside
        the pump gate, then it terminally holds at measured.

        The plant CANNOT move while this runs: a latched guard SUPPRESSES leg output
        on the Teensy, so this is a COMMAND-SPACE collapse only — it realigns u0 with
        the (stationary) encoder so a later CLEAR_ERRORS resumes output with u0
        already at the encoder (no MAX_DEVIATION re-trip).

        Used on the latch RISING edge AND by ``trajectory/reseed_from_measured``
        (/recover step 1) — idempotent: re-installing from the mid-descent commanded
        state just re-plans a fresh (shorter) descent to the same target.

        Returns True iff a collapsing descent was installed; False on the
        freeze-in-place fallback (stale telemetry, or a gate-rejected descent) — after
        which /recover cannot converge and the manual recovery dance is required.
        """
        with self._plan_lock:
            streaming = self._streaming and self._seeded
        if not streaming:
            return False
        target = self._measured_pose()
        if target is None:
            self.get_logger().error(
                "guard descent skipped — no fresh robot_state to descend toward; "
                "freezing u0 in place at the last emitted pose (recovery needs the "
                "manual dance)")
            self._freeze_in_place()
            return False
        plan = self._build_descent_plan(self._current_state(), target)
        if plan is None:
            self.get_logger().error(
                "guard descent gate-rejected (commanded seed near the envelope edge, "
                "even after an in-place decel) — freezing u0 in place at the last "
                "emitted pose; /recover cannot converge until the command clears")
            self._freeze_in_place()
            return False
        self._install(plan)
        self.get_logger().info(
            f"guard descent installed — walking u0 down to measured "
            f"(x={target[0]:.1f} y={target[1]:.1f} z={target[2]:.1f} mm) through the "
            f"pump step gate over {plan.total_duration:.2f}s; terminal hold at "
            "measured")
        return True

    def _on_link_status(self, msg) -> None:
        """Track the Teensy guard latch (FIX 1) off the bridge's /link_status.

        /link_status is a DiagnosticStatus the bridge publishes at 10 Hz; its
        ``fault_state`` KeyValue carries the guard reason (``NONE`` when healthy). A
        latched guard SUPPRESSES leg output on the Teensy while trajectory_node,
        blind, keeps advancing u0 — the 2026-07-10 runaway, where u0 ran 2.4-2.8 rev
        past the frozen encoder and every /clear_errors re-latched within one fault
        tick. On the RISING edge of a latch WHILE STREAMING we install a gate-validated
        PROFILED DESCENT that walks the commanded u0 DOWN onto the frozen encoder
        (each knot inside the pump step gate — an instant reseed there is a
        >max_step_rev jump the pump rejects) and FREEZE target advancement, so a later
        /clear_errors recovers with no re-trip; the emitter keeps streaming throughout
        (stopping it would trip MPC_STALE at 250 ms), playing out the descent. On the
        CLEAR edge (fault back to NONE while still streaming) we reseed a hold at
        measured — but ONLY once the descent has collapsed u0 within the pump gate
        (the /recover happy path); a still-in-flight descent is left to finish.

        A latch OUTSIDE a streaming mode is a NO-OP for us: the emitter is not
        publishing, so there is nothing to freeze or reseed. This is also what keeps
        the benign prior-session MPC_STALE latch at BOOT (streaming not yet enabled)
        from freezing anything — coherent with the arming pre-check, which is where
        that latch is meant to be caught.
        """
        fault = self._kv_get(msg.values, 'fault_state', 'NONE')
        latched = fault not in ('NONE', 'UNKNOWN', '')
        # A5 (ARMING_CONTRACT) — mirror the wire's arming state so accepted
        # motion commands on a disarmed wire can be loud (this node otherwise
        # has no way to know its frames are being dropped: a disarmed bridge
        # holds no :5557 subscription, so the drop is invisible end-to-end).
        self._wire_armed = self._kv_get(msg.values, 'mpc_active', '0') == '1'
        with self._plan_lock:
            streaming = self._streaming and self._seeded
        should_freeze = latched and streaming
        if should_freeze and not self._guard_frozen:
            # RISING edge — log ERROR once, then walk u0 DOWN onto the frozen encoder
            # via a gate-validated profiled descent (never an instant reseed: that was
            # a >max_step_rev jump the pump rejected — the 2026-07-10 deadlock).
            # _install_guard_descent freezes u0 in place (its own loud ERROR) if it
            # cannot build the descent (stale telemetry / gate-rejected).
            self._guard_frozen = True
            self.get_logger().error(
                f"Teensy guard LATCHED ({fault}) while streaming — collapsing the "
                "command onto the measured encoder via a profiled descent; target "
                "advancement suspended until CLEAR_ERRORS (recover with: ros2 "
                "service call /recover std_srvs/srv/Trigger)")
            self._install_guard_descent()
        elif self._guard_frozen and not should_freeze:
            # CLEAR edge — drop the freeze. If the descent has CONVERGED (commanded u0
            # within the pump step gate of measured — the /recover happy path, which
            # clears only AFTER verifying convergence), an instant hold-at-measured
            # reseed is a clean zero-step refresh that also picks up any encoder drift.
            # If a descent is still IN FLIGHT (a mid-descent bare /clear_errors, u0
            # still up to the 1.0 rev guard band out), DON'T jump u0 across the pump
            # gate — leave the descent to finish (it terminally holds at measured); the
            # follower then resumes C2 from the current commanded state. If instead we
            # left the streaming set while latched, _on_control_mode already dropped
            # seed/freeze and there is nothing to reseed.
            self._guard_frozen = False
            if not latched and streaming:
                if self._command_within_step_of_measured():
                    self._reseed_hold_at_measured()
                self.get_logger().info(
                    "Teensy guard cleared — resuming target acceptance")

    def _svc_reseed_from_measured(self, request, response):
        """``trajectory/reseed_from_measured`` (Trigger, /recover step 1).

        Install (or re-install) the gate-validated PROFILED descent that walks the
        commanded u0 down onto the measured encoder — the same collapse the latch edge
        installs. The bridge's ``/recover`` calls this FIRST, then WAITS for the
        descent to converge before firing CLEAR_ERRORS, so the clear cannot re-trip
        MAX_DEVIATION. Installing an instant hold-at-measured here (the old behaviour)
        would JUMP u0 across the pump step gate mid-descent — the very deadlock this
        fix removes. Permitted in ALL streaming modes; does not touch ``_guard_frozen``
        (the guard is still latched here — the freeze releases only when /link_status
        reports ``fault_state=NONE``).
        """
        ok = self._install_guard_descent()
        response.success = bool(ok)
        response.message = (
            'installed a profiled descent onto measured'
            if ok else
            'descent unavailable — not streaming, telemetry stale, or gate-rejected '
            '(froze in place; see node log)')
        return response

    def _guard_frozen_msg(self) -> str:
        """Uniform reject text for an advancing command refused during a guard latch."""
        return ("Teensy guard latched (E-STOP) — command advancement is frozen at "
                "the measured hold; recover with /recover (or /clear_errors) before "
                "commanding motion")

    # ═══════════════════════════════════════════════════════════
    # SpaceMouse / GUI follower (Phase 3)
    # ═══════════════════════════════════════════════════════════

    def _on_platform_pose(self, msg) -> None:
        """Store the latest streaming target (SPACEMOUSE/GUI).

        Gated exactly like ``mpc_bridge_node._on_platform_pose``: only when the
        current mode is a follower (pose-accepting) mode AND the message's
        ``publisher`` field matches the active mode. The gravity-levelling
        correction is composed into the target orientation here (verbatim port), so
        the emitter thread reads a fully-corrected target. Only the newest target is
        retained (drain-to-latest); the emitter drains it each tick.
        """
        if self._current_mode not in _FOLLOWER_MODES:
            return
        if str(msg.publisher).upper() != self._current_mode:
            return
        # FIX 1 — a latched guard freezes command advancement; drop fresh SpaceMouse
        # targets (WARN throttled) so u0 does not chase them onto suppressed legs.
        if self._guard_frozen:
            self.get_logger().warning(
                "Teensy guard latched — ignoring SpaceMouse target (holding at "
                "measured); recover with /recover or /clear_errors",
                throttle_duration_sec=1.0)
            return
        p = msg.pose_stamped.pose.position
        q = msg.pose_stamped.pose.orientation
        rot = quat_to_rot_matrix(float(q.w), float(q.x), float(q.y), float(q.z))
        rotvec = self._apply_gravity_correction(rot_matrix_to_rotvec(rot))
        target = np.array([float(p.x), float(p.y), float(p.z),
                           rotvec[0], rotvec[1], rotvec[2]])
        # Atomic single-reference publish (pose, timestamp) — no lock needed; the
        # emitter reads whichever tuple is current.
        self._follower_target = (target, time.perf_counter())

    def _on_gravity_offset(self, msg) -> None:
        """Store the gravity-levelling correction (verbatim port from mpc_bridge).

        The orchestrator publishes ``[tilt_x, tilt_y]`` (rad) — the measured tilt
        error; the correction is the inverse rotation so commanding "zero tilt"
        counter-tilts to true level.
        """
        if len(msg.data) < 2:
            return
        tilt_x, tilt_y = float(msg.data[0]), float(msg.data[1])
        self._gravity_correction = rotvec_to_rot_matrix(
            np.array([-tilt_x, -tilt_y, 0.0]))
        self.get_logger().info(
            f"gravity correction set: tilt=[{tilt_x:.4f}, {tilt_y:.4f}] rad")

    def _apply_gravity_correction(self, rotvec) -> np.ndarray:
        """Compose the gravity correction into a target rotvec:
        ``R_corrected = R_gravity @ R_target`` (verbatim port from mpc_bridge)."""
        R_corrected = self._gravity_correction @ rotvec_to_rot_matrix(
            np.asarray(rotvec, dtype=float))
        return rot_matrix_to_rotvec(R_corrected)

    def _follower_tick(self, now: float) -> bool:
        """One follower replan for wall time ``now``. Returns True iff a new plan
        was installed (so the caller re-reads the active plan). Runs on the emitter
        thread; must never raise (caller contains exceptions defensively too).

        Fresh target → track it (``follower.follow`` → install on accept, keep the
        last valid plan + throttled WARN on a gate rejection or a saturation clamp).
        No fresh target within the input-loss window → install a graceful stop ONCE
        (decel-to-rest in place), so a dead SpaceMouse node leaves the platform
        stopped, not drifting on a stale plan. A None target within the grace window
        since follower-mode entry is NOT input loss — it is the pre-first-frame
        window before any SpaceMouse frame arrives.
        """
        target_tuple = self._follower_target
        fresh = (target_tuple is not None
                 and (now - target_tuple[1]) <= self._follower_input_loss_s)

        if fresh:
            self._follower_input_lost = False
            # A fresh target supersedes any pending stop — UNLESS the escalation latch
            # is set (A3): during an escalation episode the seed is corrupt/wedged, so
            # a fresh 100 Hz stick must NOT keep cancelling the stop. The latch clears
            # only when a fresh plan actually installs (below) — i.e. the follower has
            # recovered — or on a reseed / mode change.
            if not self._escalation_stop:
                self._pending_stop = False
            # Capture the seed-sample time BEFORE sampling, then install the follow
            # plan with t0 = seed_mono so the plan's time origin matches the state it
            # was seeded from (no v·Δt rewind at every replan — the per-replan-t0-skew
            # finding).
            seed_mono = time.perf_counter()
            result = self._follower.follow(
                self._current_state(), target_tuple[0], self._limits)
            self._last_alpha = float(result.alpha)
            if result.saturated:
                self.get_logger().warning(
                    "follower target outside workspace — clamped to nearest "
                    "reachable pose", throttle_duration_sec=1.0)
            if result.rejection:
                self._last_rejection = result.rejection
                self.get_logger().warning(
                    f"follower target rejected ({result.rejection}) — holding "
                    "last valid plan", throttle_duration_sec=1.0)
            if result.escalate:
                # Sustained chase rejects (A3): the seed is corrupt or wedged near a
                # boundary. Latch a graceful stop off this flag; the pending-stop
                # retry (post-publish) drives it, with the HoldPlan backstop if it
                # never converges. Level-triggered — deduped by the latch.
                self._enter_escalation_stop(result.rejection)
            if result.plan is not None:
                # Conditional install (TOCTOU guard): re-check under _plan_lock that
                # the mode is still a follower mode. A concurrent mode-exit may have
                # installed a graceful stop during this tick's ~4-7 ms gate — dropping
                # this stale follow plan lets that stop survive.
                installed = self._install(
                    result.plan, t0=seed_mono, require_follower_mode=True)
                if installed:
                    # A fresh plan accepted + installed: the follower has recovered
                    # (a chase that tracks, or an A2 graceful stop) — the escalation
                    # episode is over (A3: this is what clears the latch).
                    self._clear_escalation_stop()
                    # A fresh tracking plan supersedes any pending stop. During an
                    # escalation episode the fresh-target clear above (line ~853) is
                    # gated off _escalation_stop, so it is SKIPPED on the recovery
                    # tick; without this, _pending_stop leaks True and the next tick's
                    # retry installs a spurious stop over the just-recovered plan. The
                    # install succeeded with require_follower_mode=True, so we are
                    # provably still in follower mode (not a mode-exit stop to keep).
                    self._pending_stop = False
                    if result.report is not None:
                        self._last_peak_vel_mmps = result.report.peak_leg_vel_mmps
                        self._last_peak_acc_mmps2 = result.report.peak_leg_acc_mmps2
                        self._last_peak_jerk_mmps3 = result.report.peak_leg_jerk_mmps3
                return installed
            return False

        # Not fresh. A None target within the input-loss window SINCE FOLLOWER-MODE
        # ENTRY is the pre-first-frame grace period (no SpaceMouse frame yet), NOT
        # input loss — keep the seeded hold and wait. Only past that window (or with
        # a genuinely stale target) do we declare loss.
        if target_tuple is None:
            entry = self._follower_entry_mono
            if entry is None or (now - entry) <= self._follower_input_loss_s:
                return False

        # Input loss: stop once. A SUCCESSFUL stop latches _follower_input_lost so we
        # don't re-stop every tick (a fresh target un-latches it above); a FAILED
        # build defers to the pending-stop retry in _emit_once (does NOT latch
        # failure — retry from the decaying live state until it converges).
        if not self._follower_input_lost and not self._pending_stop:
            try:
                stop = planner.build_graceful_stop(
                    self._current_state(), self._limits, self._geom,
                    max_iters=_STOP_RETRY_ITERS)
            except TrajectoryInfeasible as e:
                self._request_pending_stop(str(e))
                return False
            self._follower_input_lost = True
            self._install(stop)
            self.get_logger().warning(
                "follower input loss — installed a graceful stop",
                throttle_duration_sec=1.0)
            return True
        return False

    def _request_pending_stop(self, reason: str) -> None:
        """Flag that a graceful stop is wanted but build_graceful_stop rejects the
        current seed (near-boundary outward-moving: decel overshoot > boundary
        margin). ``_emit_once`` retries it every tick from the decaying live state
        until it converges — a few ticks — instead of latching failure. Loud ERROR
        once (on the transition into the pending state), NOT per tick."""
        self._last_rejection = reason
        if not self._pending_stop:
            self.get_logger().error(
                f"graceful stop deferred — near-boundary outward seed ({reason}); "
                "retrying from the decaying live state until it converges")
        self._pending_stop = True

    def _enter_escalation_stop(self, reason: str) -> None:
        """Latch a follower-escalation stop (A3).

        The follower has reported ``escalate=True`` — ``CHASE_ESCALATE_TICKS``
        consecutive chase rejects, meaning the seed is corrupt or wedged near a
        boundary (a fresh target the chase can never make feasible). Drive the
        pending-stop machinery so the post-publish retry installs a graceful stop
        (with the HoldPlan backstop if it never converges). Log ERROR ONCE per
        episode (deduped by the latch), with the seed's leg extensions so the
        forensic trail names the wedged pose."""
        # Latch under _plan_lock and re-check the mode: a concurrent _on_control_mode
        # may have left the follower mode (clearing these latches atomically with its
        # own _current_mode write) DURING this tick's in-flight follow(). Latching
        # after that exit would leak _escalation_stop into the new mode — the TOCTOU
        # install guard drops the stale follow plan, so _clear_escalation_stop never
        # runs to undo it. If we have already left, refuse: the mode-exit's own stop
        # logic owns the stop now.
        with self._plan_lock:
            if self._current_mode not in _FOLLOWER_MODES:
                return
            self._pending_stop = True
            first_escalation = not self._escalation_stop
            if first_escalation:
                self._escalation_stop = True
                self._escalate_stop_fail_count = 0
        if not first_escalation:
            return
        # Log ERROR once per episode (deduped by the latch) OUTSIDE the lock —
        # _current_state acquires _plan_lock itself.
        try:
            seed_pose = self._current_state()[0]
            ext = pose_to_leg_lengths(
                np.asarray(seed_pose[:3], dtype=float),
                rotvec_to_rot_matrix(np.asarray(seed_pose[3:6], dtype=float)),
                self._geom)
            ext_s = np.array2string(ext, precision=1)
        except Exception:  # noqa: BLE001 — the log must never raise
            ext_s = 'n/a'
        self.get_logger().error(
            f"follower escalation — {self._follower.consecutive_rejects} "
            f"consecutive chase rejects ({reason}); seed leg ext {ext_s} mm — "
            "requesting a graceful stop (corrupt / boundary-wedged seed)")

    def _clear_escalation_stop(self) -> None:
        """Clear the escalation latch (A3): the follower recovered (a fresh plan
        accepted + installed), or the seed was reseeded / the mode left."""
        self._escalation_stop = False
        self._escalate_stop_fail_count = 0

    # ═══════════════════════════════════════════════════════════
    # Services (Trigger)
    # ═══════════════════════════════════════════════════════════

    def _current_state(self):
        """Sample the active plan at ``now`` → ``(pose, twist, accel)`` seed."""
        with self._plan_lock:
            plan = self._active_plan
            t0 = self._plan_t0
        if plan is None or t0 is None:
            return (self._neutral_pose.copy(), np.zeros(6), np.zeros(6))
        return plan.state_at(time.perf_counter() - t0)

    def _install(self, plan, *, t0=None, require_follower_mode=False) -> bool:
        """Install ``plan`` as the active plan. Returns True iff it was installed.

        ``t0`` overrides the plan-time origin (default: ``perf_counter()`` at
        install). The follower passes its seed-sample timestamp so the plan's origin
        matches the state it was seeded from (removes the v·Δt rewind per replan);
        all other callers keep the default.

        ``require_follower_mode`` gates the install on the current mode still being a
        follower mode, re-checked UNDER ``_plan_lock`` — this closes the TOCTOU where
        a concurrent ``_on_control_mode`` installs a graceful stop while this (now
        stale) follower plan was being built, and the stale plan would otherwise
        clobber the stop. Dropped ⇒ returns False, active plan untouched.
        """
        now = t0 if t0 is not None else time.perf_counter()
        with self._plan_lock:
            if require_follower_mode and self._current_mode not in _FOLLOWER_MODES:
                return False
            self._active_plan = plan
            self._plan_t0 = now
            # New plan ⇒ fresh realized-peak window (per-move tracking). Reset the
            # lean label to 0.0 here because holds/stops/follower installs are
            # unshaped; go_to_pose sets the real gain (config default 0.6, or the
            # per-move override) immediately after this install returns.
            self._run_peak_vel_mmps = 0.0
            self._run_peak_acc_mmps2 = 0.0
            self._run_peak_jerk_mmps3 = 0.0
            self._prev_frame_leg_acc = None
            self._lean_gain_active = 0.0
            # Per-move boundary for the /diagnose summariser: bump on every
            # NON-follower install (go_to_pose, go_home, mode-exit/input-loss
            # graceful stops, pending-stop retries) so each becomes its own row —
            # each such install RESETS the realized peaks above, and last-sample-wins
            # would otherwise report the stop's near-zero peaks as the preceding
            # move's. Follower installs (require_follower_mode) deliberately do NOT
            # bump: a SpaceMouse stream is one continuous window across its many
            # per-tick replans.
            if not require_follower_mode:
                self._move_seq += 1
        return True

    def _make_shaper(self, gain: float) -> LeanShaper:
        """Build a lean shaper for ``gain`` (config gravity + ported cup lever arm).

        A gain ≤ 0 yields an identity shaper (``shape`` returns the base plan
        unchanged) — an explicit lean-OFF request. (The srv field default −1.0
        defers to the config gain BEFORE this is called, so the shipped default
        reaches here as 0.6.)
        """
        return LeanShaper(gain, g_mm_s2=self._gravity_mm_s2)

    def _pose_to_motor_rev(self, pose) -> np.ndarray:
        """pose_6dof → motor_rev, the EXACT chain the emitter ships (emitter.py):
        pose → (pos, rot) → leg extensions (mm) → × mm_to_rev. Used by the
        install-continuity guard so the drift check is in the same units the pump
        gates."""
        pos = np.asarray(pose[:3], dtype=float)
        rot = rotvec_to_rot_matrix(np.asarray(pose[3:6], dtype=float))
        return pose_to_leg_lengths(pos, rot, self._geom) * self._mm_to_rev

    def _active_move_in_flight(self) -> bool:
        """True iff the active plan is a MOVE (not a hold) with time remaining —
        inspected the way ``trajectory/status`` reports plan_kind/time_remaining."""
        with self._plan_lock:
            plan = self._active_plan
            t0 = self._plan_t0
        if plan is None or t0 is None or plan.kind != 'move':
            return False
        return (plan.total_duration - (time.perf_counter() - t0)) > 0.0

    def _svc_hold(self, request, response):
        """``trajectory/hold``: freeze at the current pose (profiled decel-to-rest).

        Rejected in a follower mode (SPACEMOUSE/GUI): the streaming input would
        supersede this hold within one 40 Hz tick, so a hold there is a mode
        confusion — exit the follower mode (which installs a graceful stop) instead.
        Mirrors ``go_to_pose``'s WRONG_MODE rationale.
        """
        if self._current_mode in _FOLLOWER_MODES:
            response.success = False
            response.message = (
                'in a follower mode the input stream would supersede this within '
                'one tick — exit the mode (graceful stop) instead')
            return response
        if not self._seeded:
            response.success = False
            response.message = 'not streaming/seeded — cannot hold'
            return response
        try:
            plan = planner.build_hold(
                self._current_state(), self._limits, self._geom)
        except TrajectoryInfeasible as e:
            self._last_rejection = str(e)
            self.get_logger().error(f"hold rejected: {e}")
            response.success = False
            response.message = str(e)
            return response
        self._install(plan)
        response.success = True
        response.message = 'holding at current pose' + self._wire_state_suffix()
        return response

    def _svc_go_home(self, request, response):
        """``trajectory/go_home``: profiled return to the neutral active pose.

        Rejected in a follower mode (SPACEMOUSE/GUI): the streaming input would
        supersede this move within one 40 Hz tick, so a go_home there is a mode
        confusion — exit the follower mode (which installs a graceful stop) instead.
        Mirrors ``go_to_pose``'s WRONG_MODE rationale.
        """
        if self._guard_frozen:                       # FIX 1 — refuse while latched
            response.success = False
            response.message = self._guard_frozen_msg()
            return response
        if self._current_mode in _FOLLOWER_MODES:
            response.success = False
            response.message = (
                'in a follower mode the input stream would supersede this within '
                'one tick — exit the mode (graceful stop) instead')
            return response
        if not self._seeded:
            response.success = False
            response.message = 'not streaming/seeded — cannot go home'
            return response
        try:
            plan = planner.build_return_to_neutral(
                self._current_state(), self._neutral_pose,
                self._go_home_duration_s, self._limits, self._geom)
        except TrajectoryInfeasible as e:
            self._last_rejection = str(e)
            self.get_logger().error(f"go_home rejected: {e}")
            response.success = False
            response.message = str(e)
            return response
        self._install(plan)
        response.success = True
        response.message = (f'returning to neutral (0,0,{self._neutral_pose[2]:.0f}) '
                            f'over {self._go_home_duration_s:.2f}s'
                            + self._wire_state_suffix())
        return response

    # ═══════════════════════════════════════════════════════════
    # Move services (Phase 2)
    # ═══════════════════════════════════════════════════════════

    def _pose_from_msg(self, pose_msg) -> np.ndarray:
        """geometry_msgs/Pose → pose_6dof ``[x, y, z, rx, ry, rz]`` (mm, rad rotvec)."""
        p = pose_msg.position
        q = pose_msg.orientation
        rot = quat_to_rot_matrix(float(q.w), float(q.x), float(q.y), float(q.z))
        rotvec = rot_matrix_to_rotvec(rot)
        return np.array([float(p.x), float(p.y), float(p.z),
                         float(rotvec[0]), float(rotvec[1]), float(rotvec[2])])

    def _svc_go_to_pose(self, request, response):
        """``trajectory/go_to_pose``: a profiled point-to-point move (TRAJECTORY mode).

        Rejects loudly — never silently — when the node is not in TRAJECTORY mode
        (``WRONG_MODE``); not yet seeded or the telemetry is stale (``STALE_STATE``);
        a move is already in flight (``BUSY`` — Phase 2 accepts moves only from a
        hold); the commanded state drifted while the gate ran (``STALE_STATE``); or
        the move is infeasible (the gate code, with ``min_duration_s`` on ``TOO_FAST``).
        """
        response.planned_duration_s = 0.0
        response.min_duration_s = 0.0
        # FIX 1 — refuse any advancing move while a guard E-STOP is latched.
        if self._guard_frozen:
            response.accepted = False
            response.code = _GUARD_LATCHED
            response.message = self._guard_frozen_msg()
            self._last_rejection = response.message
            self.get_logger().error(response.message)
            return response
        # Gate: TRAJECTORY mode only. A move from STANDBY/SPACEMOUSE is a
        # mode confusion — reject rather than move unexpectedly.
        if self._current_mode != _MOVE_MODE:
            response.accepted = False
            response.code = feas.WRONG_MODE
            response.message = (f"go_to_pose requires {_MOVE_MODE} mode "
                                f"(current mode '{self._current_mode}')")
            self._last_rejection = response.message
            self.get_logger().error(response.message)
            return response
        if not self._seeded:
            response.accepted = False
            response.code = feas.STALE_STATE
            response.message = 'not streaming/seeded — cannot plan a move'
            self._last_rejection = response.message
            self.get_logger().error(response.message)
            return response
        # Telemetry-staleness gate: the seed that drives the plan is only as fresh
        # as robot_state. If telemetry has gone stale, the sampled seed pose may no
        # longer match the platform — refuse to plan a move on it.
        if not self._robot_state_fresh():
            response.accepted = False
            response.code = feas.STALE_STATE
            response.message = 'robot_state telemetry stale — cannot plan a move'
            self._last_rejection = response.message
            self.get_logger().error(response.message)
            return response
        # Phase-2 BUSY restriction (temporary; lifted by Phase 3/5 move-supersede):
        # accept a move only from a hold. Superseding an in-flight move needs the
        # follower's C2 chaining, which is not wired yet — reject loudly instead of
        # jumping the commanded state.
        if self._active_move_in_flight():
            response.accepted = False
            response.code = _BUSY
            response.message = ('a move is in flight — Phase 2 accepts moves only '
                                'from hold; supersede arrives with Phase 3/5')
            self._last_rejection = response.message
            self.get_logger().error(response.message)
            return response

        try:
            target = self._pose_from_msg(request.pose)
        except Exception as e:  # noqa: BLE001 — malformed request
            response.accepted = False
            response.code = feas.UNREACHABLE
            response.message = f'malformed target pose: {e}'
            self._last_rejection = response.message
            self.get_logger().error(response.message)
            return response

        duration = float(request.duration_s)
        # Reject a non-finite requested duration up front (NaN/Inf sail through the
        # `> 0.0` test and every downstream numeric comparison). Loud TOO_FAST.
        if not np.isfinite(duration):
            response.accepted = False
            response.code = feas.TOO_FAST
            response.message = 'non-finite duration_s'
            self._last_rejection = response.message
            self.get_logger().error(response.message)
            return response
        # Effective lean gain (Phase 4 A/B). A negative request.lean_gain defers to
        # the configured JB_TRAJ_LEAN_GAIN; a non-negative value overrides it,
        # clamped to [0, 1]. Shaping runs BEFORE validate inside build_move, so the
        # gate measures the shaped leg peaks (the canonical ordering invariant).
        req_gain = float(request.lean_gain)
        gain = (self._config_lean_gain if (not np.isfinite(req_gain)
                                           or req_gain < 0.0)
                else min(1.0, max(0.0, req_gain)))
        shaper = self._make_shaper(gain)
        try:
            plan, report = planner.build_move(
                self._current_state(), target,
                duration if duration > 0.0 else None,
                self._limits, self._geom, shaper=shaper,
                retime_model=self._retime_model)
        except TrajectoryInfeasible as e:
            response.accepted = False
            response.code = e.code
            response.message = str(e)
            response.min_duration_s = float(e.min_duration_s)
            self._last_rejection = str(e)
            self.get_logger().error(f"go_to_pose rejected: {e}")
            return response

        # Install-continuity guard. build_move seeds from _current_state() at entry,
        # but the full gate still takes real wall time while the emitter keeps
        # streaming the OLD plan: after the 2026-07-16/17 planning speedups
        # (component-form Jacobian cross + 80-sample unshaped gate + batched shaped
        # gate + retiming-model search) an unshaped move runs ~2 validate passes
        # (~0.1 s offline-measured) and a shaped (lean-gain>0) move runs the
        # retiming model + ONE verify pass (~90 ms measured; the legacy 6-pass
        # fallback is ~0.23 s — the drift window this guard protects
        # against). If the commanded
        # state moved during planning, installing this plan would jump u0 from the
        # drifted live position back to
        # the stale seed — a transient both the pump/firmware step gates could still
        # pass. Re-sample now and reject if the plan's t=0 commanded position has
        # drifted from the live one (compared in motor_rev, the pump's units).
        drift = float(np.max(np.abs(
            self._pose_to_motor_rev(plan.state_at(0.0)[0])
            - self._pose_to_motor_rev(self._current_state()[0]))))
        continuity_bound = (0.25 * feas.STEP_BOUND_MARGIN
                            * hw.JB_OP_MAX_POSITION_STEP_REV)
        if drift > continuity_bound:
            response.accepted = False
            response.code = feas.STALE_STATE
            response.message = 'commanded state moved during planning — retry'
            self._last_rejection = response.message
            self.get_logger().error(
                f"{response.message} (drift {drift:.3f} rev > "
                f"{continuity_bound:.3f})")
            return response

        # Record the accepted plan's measured leg peaks (from the report build_move
        # returned — no second ~350 ms validate) for trajectory/diagnostics.
        self._last_peak_vel_mmps = report.peak_leg_vel_mmps
        self._last_peak_acc_mmps2 = report.peak_leg_acc_mmps2
        self._last_peak_jerk_mmps3 = report.peak_leg_jerk_mmps3

        self._install(plan)              # bumps _move_seq (non-follower install)
        self._lean_gain_active = gain    # label this move's A/B arm (install reset it)
        response.accepted = True
        response.code = feas.OK
        response.planned_duration_s = float(plan.total_duration)
        response.message = (
            f"move accepted: target (x={target[0]:.1f} y={target[1]:.1f} "
            f"z={target[2]:.1f} mm) over {plan.total_duration:.3f}s "
            f"(lean_gain={gain:.2f})" + self._wire_state_suffix())
        return response

    def _wire_state_suffix(self) -> str:
        """A5 (ARMING_CONTRACT): '' when the wire is armed; a loud marker when
        an accepted motion command is about to execute into a dropped stream.

        Acceptance here is a PLANNING verdict — this node cannot refuse on a
        disarmed wire (streaming-while-disarmed is the legal pre-arm phase of
        the documented flow) — but it must never be SILENT about it: on
        2026-07-15 a full battery was "accepted" and fully emitted with
        mpc_active=0, zero motion, and zero warnings anywhere.
        """
        if self._wire_armed:
            return ""
        self.get_logger().warning(
            "motion command accepted while the wire is DISARMED (mpc_active=0) "
            "— the emitted setpoints are NOT reaching the legs. Arm via the "
            "orchestrator (auto-arm on ACTIVE entry) or "
            "ros2 service call /set_setpoint_output std_srvs/srv/SetBool "
            "\"{data: true}\"",
            throttle_duration_sec=5.0)
        return " [wire DISARMED — setpoints not reaching the legs]"

    def _svc_set_limits(self, request, response):
        """``trajectory/set_limits``: ramp the session leg limits (0 ⇒ keep).

        Each requested limit is clamped to its YAML hard ceiling before it takes
        effect — a runtime request can never raise a limit past the pinned
        physical envelope. Swapping the (immutable, frozen) limits reference is
        atomic; the emitter and service handlers read whichever value is current.
        """
        v = request.leg_vel_limit_mmps
        a = request.leg_acc_limit_mmps2
        j = request.leg_jerk_limit_mmps3
        new_limits = self._limits.with_session_limits(
            leg_vel_mmps=v if v > 0.0 else None,
            leg_acc_mmps2=a if a > 0.0 else None,
            leg_jerk_mmps3=j if j > 0.0 else None)
        self._limits = new_limits
        response.success = True
        response.applied_vel_limit_mmps = new_limits.leg_vel_mmps
        response.applied_acc_limit_mmps2 = new_limits.leg_acc_mmps2
        response.applied_jerk_limit_mmps3 = new_limits.leg_jerk_mmps3
        response.message = (
            f"limits: vel={new_limits.leg_vel_mmps:.1f} mm/s "
            f"acc={new_limits.leg_acc_mmps2:.1f} mm/s² "
            f"jerk={new_limits.leg_jerk_mmps3:.0f} mm/s³ "
            f"(ceilings {new_limits.leg_vel_ceiling_mmps:.0f}/"
            f"{new_limits.leg_acc_ceiling_mmps2:.0f}/"
            f"{new_limits.leg_jerk_ceiling_mmps3:.0f})")
        self.get_logger().info(response.message)
        return response

    # ═══════════════════════════════════════════════════════════
    # Timed targets + catch (Phase 5)
    # ═══════════════════════════════════════════════════════════

    def _measure_clock_offset(self) -> float:
        """Median (perf_counter − ROS-clock) offset (mirrors catch_coordinator_node).

        perf_counter is CLOCK_MONOTONIC; the ROS clock is the node's wall clock.
        A ROS-domain arrival time ``t_ros`` maps to the emitter's perf domain as
        ``t_ros + offset``. Sampled a few times and median-filtered to shrug off a
        scheduler hiccup between the two reads.
        """
        offsets = []
        for _ in range(10):
            t_perf = time.perf_counter()
            t_ros = self.get_clock().now().nanoseconds / 1e9
            offsets.append(t_perf - t_ros)
        return float(np.median(offsets))

    def _refresh_clock_offset(self) -> None:
        """Periodically re-measure the offset to track drift (median of last 20)."""
        self._clock_offset_history.append(self._measure_clock_offset())
        if len(self._clock_offset_history) > 20:
            self._clock_offset_history.pop(0)
        self._ros_to_perf_offset = float(np.median(self._clock_offset_history))

    def _ros_time_to_perf(self, ros_time) -> float:
        """THE ROS-clock → perf_counter conversion point (plan Architecture).

        ``ros_time`` is a ``builtin_interfaces/Time`` (``.sec`` / ``.nanosec``).
        NO production caller since the timed_target service moved to a relative
        ``lead_time_s`` (perf-anchored at handler entry) — the emitter and catch
        path were already perf-domain. Retained so any future ROS-clock-timed
        surface still has exactly one crossing point.
        """
        ros_sec = float(ros_time.sec) + float(ros_time.nanosec) * 1e-9
        return ros_sec + self._ros_to_perf_offset

    def _install_continuity_ok(self, plan) -> bool:
        """True iff ``plan``'s t=0 commanded pose still matches the live commanded
        state (in motor_rev, the pump's units).

        The seed is sampled at service/callback entry but the plan installs a few ms
        later while the emitter streams the OLD plan; if the commanded state drifted
        past the guard bound during planning, installing would jump u0 from the live
        position back to the stale seed. The fast ``validate_follow`` gate keeps that
        window small — single-digit ms typical, worst case tens of ms with an appended
        stop-stretch — so a normal supersede's drift stays ≪ bound and passes; this
        guard is what makes the claim safe, rejecting ``STALE_STATE`` on a drift
        > 0.06 rev rather than jumping u0. Same bound (0.06 rev) as ``go_to_pose``.
        """
        drift = float(np.max(np.abs(
            self._pose_to_motor_rev(plan.state_at(0.0)[0])
            - self._pose_to_motor_rev(self._current_state()[0]))))
        bound = 0.25 * feas.STEP_BOUND_MARGIN * hw.JB_OP_MAX_POSITION_STEP_REV
        return drift <= bound

    def _publish_target_feedback(self, accepted: bool, code: str, reason: str,
                                 arrival_perf: float, source: str) -> None:
        """Publish accept/reject feedback for a timed/catch target.

        ``arrival_perf`` is the correlation key catch_coordinator_node matches
        against its last submission; on a reject it drives the feasibility blacklist.
        """
        fb = TargetFeedback()
        fb.accepted = bool(accepted)
        fb.code = str(code)
        fb.reason = str(reason)
        fb.arrival_time = float(arrival_perf)
        fb.source = str(source)
        self.target_feedback_pub.publish(fb)

    def _plan_and_install_timed(self, target, twist, arrival_perf, *,
                                hold_after, source, neutral=None):
        """Build a timed plan to ``(target, twist)`` arriving at ``arrival_perf`` and
        install it (supersede-safe). Returns ``(accepted, code, message, planned_s,
        min_s)`` and always publishes ``trajectory/target_feedback``.

        The plan is seeded from the current sampled state and installed with
        ``t0 = seed_mono``, so the reach segment's end (the target) lands at wall
        time ``seed_mono + lead = arrival_perf`` and the plan-time origin matches the
        seed sample — a clean C2 replan even mid-move (no BUSY restriction: the fast
        ``build_timed`` gate keeps the install window to single-digit ms typical
        [guard-bounded; worst case tens of ms with an appended stop-stretch], unlike
        the ~377 ms analytic gate that forced ``go_to_pose``'s BUSY guard).
        """
        seed_mono = time.perf_counter()
        lead = arrival_perf - seed_mono
        try:
            plan, report = planner.build_timed(
                self._current_state(), target, twist, lead,
                self._limits, self._geom,
                hold_after=hold_after, neutral_pose=neutral)
        except TrajectoryInfeasible as e:
            self._last_rejection = str(e)
            self.get_logger().error(f"{source} target rejected: {e}")
            self._publish_target_feedback(False, e.code, str(e), arrival_perf, source)
            return False, e.code, str(e), 0.0, float(e.min_duration_s)
        if not self._install_continuity_ok(plan):
            msg = 'commanded state moved during planning — retry'
            self._last_rejection = msg
            self.get_logger().error(msg)
            self._publish_target_feedback(
                False, feas.STALE_STATE, msg, arrival_perf, source)
            return False, feas.STALE_STATE, msg, 0.0, 0.0
        self._install(plan, t0=seed_mono)
        # Record the accepting plan's PREDICTED leg peaks (from the report build_timed
        # returned — no second gate pass) for trajectory/diagnostics, exactly as
        # _svc_go_to_pose does; the install above resets only the REALIZED peaks.
        self._last_peak_vel_mmps = report.peak_leg_vel_mmps
        self._last_peak_acc_mmps2 = report.peak_leg_acc_mmps2
        self._last_peak_jerk_mmps3 = report.peak_leg_jerk_mmps3
        # Install-latency (perf_counter − seed_mono) for the bench p95 — DEBUG only, so
        # it never costs the hot path but is captured when a session runs at DEBUG.
        self.get_logger().debug(
            f"{source} timed install latency "
            f"{(time.perf_counter() - seed_mono) * 1e3:.1f} ms")
        self._publish_target_feedback(True, feas.OK, '', arrival_perf, source)
        return True, feas.OK, 'timed target accepted', float(lead), 0.0

    def _plan_and_install_catch(self, catch_pose, arrival_perf, *, source):
        """Build the tilt-through-seat catch plan to ``catch_pose`` arriving at
        ``arrival_perf`` and install it (supersede-safe). Returns the same
        ``(accepted, code, message, planned_s, min_s)`` tuple as
        ``_plan_and_install_timed`` and always publishes ``trajectory/target_feedback``.

        The difference from ``_plan_and_install_timed`` is the planner: ``build_catch``
        assembles reach → tilt-through-seat decay → literal quiescent hold (rather than
        a bare reach-to-rest). The receive tilt is already baked into ``catch_pose``
        (``rx, ry`` — the coordinator computed it collinear with the ball's arrival
        velocity in ``compute_catch_orientation``); ``build_catch`` reads that tilt to
        aim the small through-seat residual rate, and forces translational arrival
        velocity to zero (velocity matching is the hand's job). Same fast
        ``validate_follow`` gate and same install-continuity guard, so the C2-supersede
        and pump-acceptance guarantees carry over unchanged. ``hold_after=True``: the
        plan holds the settled catch pose (the mode-exit → STANDBY path owns the return
        to neutral, exactly as the Phase-5 catch reach did).
        """
        seed_mono = time.perf_counter()
        lead = arrival_perf - seed_mono
        try:
            plan, report = planner.build_catch(
                self._current_state(), catch_pose, lead,
                self._limits, self._geom,
                settle_hold_s=self._catch_settle_hold_s,
                hold_after=True)
        except TrajectoryInfeasible as e:
            self._last_rejection = str(e)
            self.get_logger().error(f"{source} catch target rejected: {e}")
            self._publish_target_feedback(False, e.code, str(e), arrival_perf, source)
            return False, e.code, str(e), 0.0, float(e.min_duration_s)
        if not self._install_continuity_ok(plan):
            msg = 'commanded state moved during planning — retry'
            self._last_rejection = msg
            self.get_logger().error(msg)
            self._publish_target_feedback(
                False, feas.STALE_STATE, msg, arrival_perf, source)
            return False, feas.STALE_STATE, msg, 0.0, 0.0
        self._install(plan, t0=seed_mono)
        # Predicted leg peaks for trajectory/diagnostics (same as _plan_and_install_timed;
        # the install resets only the realized peaks).
        self._last_peak_vel_mmps = report.peak_leg_vel_mmps
        self._last_peak_acc_mmps2 = report.peak_leg_acc_mmps2
        self._last_peak_jerk_mmps3 = report.peak_leg_jerk_mmps3
        self.get_logger().debug(
            f"{source} catch install latency "
            f"{(time.perf_counter() - seed_mono) * 1e3:.1f} ms")
        self._publish_target_feedback(True, feas.OK, '', arrival_perf, source)
        return True, feas.OK, 'catch target accepted', float(lead), 0.0

    def _catch_target_from_msg(self, msg):
        """DynamicTargetCommand → (target pose_6dof, arrival twist) in the trajectory
        convention. The wire pose is ALREADY STOW-relative (0 = stow, ~170 = active —
        the coordinator subtracts GEOM_INITIAL_HEIGHT from the world intercept), i.e.
        the same frame as the trajectory pose, so it maps through VERBATIM. Hardware-
        verified 2026-07-23; the former "+ active_z" lift here was a double-count that
        drove every catch reach out of stroke. The orientation carries the
        gravity-levelling correction."""
        q = msg.target_quat
        rot = quat_to_rot_matrix(float(q.w), float(q.x), float(q.y), float(q.z))
        rotvec = self._apply_gravity_correction(rot_matrix_to_rotvec(rot))
        target = np.array([
            float(msg.target_pos.x),
            float(msg.target_pos.y),
            float(msg.target_pos.z),
            rotvec[0], rotvec[1], rotvec[2]])
        # Linear arrival velocity only (angular arrival rate = 0). Velocity is
        # frame-offset-invariant, so no z adjustment.
        twist = np.array([float(msg.target_vel.x), float(msg.target_vel.y),
                          float(msg.target_vel.z), 0.0, 0.0, 0.0])
        return target, twist

    def _on_dynamic_target(self, msg) -> None:
        """``catch/dynamic_target``: a timed catch target.

        Fires only when the catch-armed latch is raised (the reload-action trigger —
        the node stays in TRAJECTORY throughout). Outside the latch, a dynamic_target
        is ignored.

        ``arrival_time`` is ALREADY perf-domain (the coordinator converted
        landing_time → perf), so no clock crossing here — the single crossing is the
        ROS-time ``timed_target`` service. Each update supersedes the prior via a C2
        replan through ``build_catch`` (fast gate) — EXCEPT inside the reach-freeze
        window (within ``catch_reach_freeze_s`` of the committed arrival), where late
        target jitter is ignored so the platform holds its committed reach into the
        catch (a parked, non-jittering rim seats the ball; the reload design). An
        operator abort lowers the latch, which clears the freeze via
        _reset_catch_reach_freeze.
        """
        if not self._catch_armed:
            return
        arrival_perf = float(msg.arrival_time)
        if self._guard_frozen:                       # FIX 1 — refuse while latched
            self._publish_target_feedback(
                False, _GUARD_LATCHED, self._guard_frozen_msg(),
                arrival_perf, 'catch')
            return
        if not (self._seeded and self._robot_state_fresh()):
            self._publish_target_feedback(
                False, feas.STALE_STATE,
                'not seeded / telemetry stale', arrival_perf, 'catch')
            return
        # Reach-freeze window management. `_catch_arrival_perf` is the committed
        # arrival; the freeze runs from `arrival − reach_freeze` and holds through the
        # quiescent settle (`arrival + settle_hold`).
        if self._catch_arrival_perf is not None:
            now = time.perf_counter()
            if now > self._catch_arrival_perf + self._catch_settle_hold_s:
                # Committed arrival + settle has fully passed — release the freeze so
                # THIS (and every later) target supersedes as a new reach. Without this
                # the freeze latched forever and every post-catch target was silently
                # dropped (breaks repeated catches).
                self._catch_arrival_perf = None
            elif now >= self._catch_arrival_perf - self._catch_reach_freeze_s:
                # Genuinely frozen: hold the committed reach and IGNORE this late
                # target — but report FROZEN (a service-level code, NOT a feasibility
                # reject) so the coordinator neither accepts nor blacklists it, rather
                # than dropping it silently.
                self._publish_target_feedback(
                    False, _FROZEN,
                    'within reach-freeze window — holding committed catch reach',
                    arrival_perf, 'catch')
                return
        target, _twist = self._catch_target_from_msg(msg)
        # Reach-envelope gate (the envelope build_catch documents as the caller's
        # responsibility): reject any target farther than the envelope (3D) from the
        # pose held at arm-latch raise. WORKSPACE is deliberate — it is a
        # position-unreachable-by-policy reject, so the coordinator's feasibility
        # blacklist counts it (a drifting landing estimate blacklists out instead of
        # dragging the platform sideways all flight).
        if self._catch_envelope_center is not None:
            excursion = float(np.linalg.norm(
                target[:3] - self._catch_envelope_center))
            if excursion > self._catch_reach_envelope_mm:
                reason = (
                    f"catch target {excursion:.0f} mm from the armed hold pose "
                    f"exceeds the {self._catch_reach_envelope_mm:.0f} mm reach "
                    f"envelope")
                self._last_rejection = reason
                self.get_logger().error(f"catch target rejected: {reason}")
                self._publish_target_feedback(
                    False, feas.WORKSPACE, reason, arrival_perf, 'catch')
                return
        # Phase 7: build the tilt-through-seat catch (reach → through-seat decay →
        # quiescent hold) rather than the Phase-5 reach-only build_timed. The receive
        # tilt is already in ``target[3:5]`` (coordinator's collinear compute_catch_
        # orientation); ``build_catch`` carries a small residual rate through the seat
        # so the rim is not parked at contact. ``_twist`` (the coordinator's
        # target_vel — always zero for a stationary catch) is unused: build_catch forces
        # translational arrival velocity to zero by design.
        accepted, _code, _msg, _pl, _mn = self._plan_and_install_catch(
            target, arrival_perf, source='catch')
        if accepted:
            self._catch_arrival_perf = arrival_perf

    def _svc_arm_catch(self, request, response):
        """``trajectory/arm_catch`` (SetBool): raise/lower the catch-armed latch.

        The latch is the reactive-catch TRIGGER (reload-action-catch-latch plan).
        While armed, ``catch/dynamic_target`` installs a ``build_catch`` reach — the
        RELOAD action raises it for the flight window (it is the sole catch trigger).
        The node stays in TRAJECTORY throughout (already streaming +
        motion-capable), so the latch is NOT added to any mode set; only the
        freeze-reset + graceful-stop bookkeeping is keyed off the arm/disarm EDGES here.

        On BOTH edges the same freeze-reset + mid-move graceful-stop bookkeeping runs,
        so the arm/disarm seams produce no command discontinuity:
          - the committed catch-reach freeze window is released (``_reset_catch_reach_
            freeze`` — a fresh catch session, or an abort, must not inherit a stale
            freeze); and
          - if a move is in flight ACROSS the edge, a graceful decel-to-rest is
            installed from the LIVE commanded state (``_install_graceful_stop``): arming
            mid-move → the catch begins from a clean, non-jittering state (the first
            catch reach then supersedes via a C2 replan); the documented abort =
            disarming mid-reach → the reach is silenced, never run on to the catch
            target. The stop is C2 off the live state, so no command discontinuity at
            the seam.

        Idempotent: a set-to-current-value call is a no-op (no edge, no bookkeeping),
        mirroring ``_on_control_mode``'s same-mode early return.
        """
        want = bool(request.data)
        if want == self._catch_armed:
            response.success = True
            response.message = (
                f"catch latch already {'armed' if want else 'disarmed'}")
            return response
        # Snapshot the in-flight state BEFORE flipping the latch (mirrors
        # _on_control_mode snapshotting move_in_flight before the _current_mode write):
        # a graceful stop must sample the state at the transition. _active_move_in_flight
        # locks itself; _install_graceful_stop's _current_state/_install lock themselves.
        move_in_flight = self._active_move_in_flight()
        # Reach-envelope center: captured at the RAISE edge (the commanded position the
        # catch session starts from — normally the active hold), cleared at the lower
        # edge. Every catch target this session is bounded to the envelope around it.
        # Captured BEFORE the latch flips so no ordering ever exposes an
        # armed-with-no-center window (in which _on_dynamic_target would skip the
        # envelope gate); under the current single-threaded spin the callbacks
        # serialize anyway — this is defence-in-depth, not a live race.
        if want:
            self._catch_envelope_center = np.asarray(
                self._current_state()[0][:3], dtype=float).copy()
        else:
            self._catch_envelope_center = None
        self._catch_armed = want
        self._reset_catch_reach_freeze()
        if move_in_flight:
            self._install_graceful_stop(
                f"catch latch {'armed' if want else 'disarmed'}")
        response.success = True
        response.message = f"catch latch {'armed' if want else 'disarmed'}"
        self.get_logger().info(response.message)
        return response

    def _svc_timed_target(self, request, response):
        """``trajectory/timed_target``: reach a pose at a velocity ``lead_time_s``
        seconds after service receipt (a RELATIVE lead — the absolute arrival is
        computed here as ``perf_counter()`` at handler entry + ``lead_time_s``).

        TRAJECTORY mode only (else ``WRONG_MODE``). Rejects loudly on a stale/absent
        seed (``STALE_STATE``), a non-finite/malformed ``lead_time_s``
        (``UNREACHABLE``), a non-positive or too-tight lead (``TOO_FAST`` with the
        achievable ``min_duration_s``), or an infeasible/out-of-workspace target. Supersedes an
        in-flight plan by design — NO ``BUSY`` restriction, because the fast
        ``build_timed`` gate installs shortly after the seed sample (single-digit ms
        typical, guard-bounded — install-continuity rejects ``STALE_STATE`` on a drift
        > 0.06 rev; worst case tens of ms with an appended stop-stretch — a clean C2
        replan), unlike the ~377 ms ``go_to_pose`` path that forced ``BUSY``.
        """
        response.planned_duration_s = 0.0
        response.min_duration_s = 0.0
        if self._guard_frozen:                       # FIX 1 — refuse while latched
            response.accepted = False
            response.code = _GUARD_LATCHED
            response.message = self._guard_frozen_msg()
            self._last_rejection = response.message
            self.get_logger().error(response.message)
            return response
        if self._current_mode != _MOVE_MODE:
            response.accepted = False
            response.code = feas.WRONG_MODE
            response.message = (f"timed_target requires {_MOVE_MODE} mode "
                                f"(current mode '{self._current_mode}')")
            self._last_rejection = response.message
            self.get_logger().error(response.message)
            return response
        if not self._seeded:
            response.accepted = False
            response.code = feas.STALE_STATE
            response.message = 'not streaming/seeded — cannot plan a timed target'
            self._last_rejection = response.message
            self.get_logger().error(response.message)
            return response
        if not self._robot_state_fresh():
            response.accepted = False
            response.code = feas.STALE_STATE
            response.message = 'robot_state telemetry stale — cannot plan a timed target'
            self._last_rejection = response.message
            self.get_logger().error(response.message)
            return response
        try:
            target = self._pose_from_msg(request.pose)
        except Exception as e:  # noqa: BLE001 — malformed request
            response.accepted = False
            response.code = feas.UNREACHABLE
            response.message = f'malformed target pose: {e}'
            self._last_rejection = response.message
            self.get_logger().error(response.message)
            return response
        v = request.velocity_mm_s
        twist = np.array([float(v.x), float(v.y), float(v.z), 0.0, 0.0, 0.0])
        try:
            lead = float(request.lead_time_s)
        except Exception as e:  # noqa: BLE001 — malformed request
            response.accepted = False
            response.code = feas.UNREACHABLE
            response.message = f'malformed lead_time_s: {e}'
            self._last_rejection = response.message
            self.get_logger().error(response.message)
            return response
        if not np.isfinite(lead):
            response.accepted = False
            response.code = feas.UNREACHABLE
            response.message = (f'non-finite lead_time_s ({lead}) — the arrival '
                                f'lead must be a finite positive number of seconds')
            self._last_rejection = response.message
            self.get_logger().error(response.message)
            return response
        # Anchor the ABSOLUTE arrival at handler entry in the emitter's perf domain
        # (the handler runs synchronously, and _plan_and_install_timed / the emitter
        # both run on time.perf_counter() — same clock, no domain crossing). A
        # non-positive or too-tight lead flows into the single feasibility gate:
        # build_timed's lead floor rejects it loudly TOO_FAST with the genuinely
        # achievable min_duration_s (never a silent late arrival). Only NaN would
        # sail through the gate's floor/ceiling comparisons, hence the finite
        # guard above.
        arrival_perf = time.perf_counter() + lead
        hold_after = bool(request.hold_after)
        accepted, code, message, planned, min_s = self._plan_and_install_timed(
            target, twist, arrival_perf, hold_after=hold_after, source='timed',
            neutral=(None if hold_after else self._neutral_pose))
        response.accepted = accepted
        response.code = code
        response.message = (message + self._wire_state_suffix()
                            if accepted else message)
        response.planned_duration_s = planned
        response.min_duration_s = min_s
        return response

    # ═══════════════════════════════════════════════════════════
    # Status + diagnostics (5 Hz)
    # ═══════════════════════════════════════════════════════════

    def _publish_status(self):
        with self._plan_lock:
            plan = self._active_plan
            t0 = self._plan_t0
            streaming = self._streaming and self._seeded
            seq = self._seq
        remaining = 0.0
        kind = 'none'
        if plan is not None and t0 is not None:
            tau = time.perf_counter() - t0
            remaining = max(0.0, plan.total_duration - tau)
            kind = plan.kind

        gap_ms = self._max_emit_gap_s * 1e3
        msg = TrajectoryStatus()
        msg.streaming = bool(streaming)
        msg.mode = self._current_mode
        msg.plan_kind = kind
        msg.plan_time_remaining_s = float(remaining)
        msg.seq = int(seq)
        msg.max_emit_gap_ms = float(gap_ms)
        msg.last_rejection = self._last_rejection
        self.status_pub.publish(msg)

        # Diagnostics: the last accepted move's measured leg peaks + emitter jitter.
        diag = DiagnosticStatus()
        diag.name = 'trajectory/diagnostics'
        diag.hardware_id = 'trajectory_node'
        diag.level = DiagnosticStatus.OK if streaming else DiagnosticStatus.WARN
        diag.message = 'streaming' if streaming else 'idle'
        lim = self._limits
        diag.values = [
            # Gate-PREDICTED peaks of the last accepted plan (fine-sampled report).
            KeyValue(key='peak_leg_vel_mmps',
                     value=f'{self._last_peak_vel_mmps:.1f}'),
            KeyValue(key='peak_leg_acc_mmps2',
                     value=f'{self._last_peak_acc_mmps2:.1f}'),
            KeyValue(key='peak_leg_jerk_mmps3',
                     value=f'{self._last_peak_jerk_mmps3:.0f}'),
            # REALIZED peaks tracked from the emitted knots of the active plan
            # (Phase 4 ramp observability; reset per move).
            KeyValue(key='realized_peak_leg_vel_mmps',
                     value=f'{self._run_peak_vel_mmps:.1f}'),
            KeyValue(key='realized_peak_leg_acc_mmps2',
                     value=f'{self._run_peak_acc_mmps2:.1f}'),
            KeyValue(key='realized_peak_leg_jerk_mmps3',
                     value=f'{self._run_peak_jerk_mmps3:.0f}'),
            # Current session limits, so /diagnose computes headroom without the YAML.
            KeyValue(key='limit_leg_vel_mmps', value=f'{lim.leg_vel_mmps:.1f}'),
            KeyValue(key='limit_leg_acc_mmps2', value=f'{lim.leg_acc_mmps2:.1f}'),
            KeyValue(key='limit_leg_jerk_mmps3', value=f'{lim.leg_jerk_mmps3:.0f}'),
            # A/B arm the active plan belongs to (0.0 = lean off / hold).
            KeyValue(key='lean_gain', value=f'{self._lean_gain_active:.2f}'),
            # Per-move install counter — the /diagnose summariser segments moves on a
            # change here, since plan_kind stays 'move' across back-to-back moves.
            KeyValue(key='move_seq', value=str(self._move_seq)),
            KeyValue(key='max_emit_gap_ms', value=f'{gap_ms:.1f}'),
            KeyValue(key='plan_kind', value=kind),
            # Chase-follower observability (chase-clamp rewrite): last chase progress
            # fraction, the consecutive-reject streak, whether the escalation stop is
            # latched, and the worst post-publish follow-block cost over this window
            # (C3 — must stay far under the ~117 ms decay→sprint threshold).
            KeyValue(key='chase_alpha', value=f'{self._last_alpha:.3f}'),
            KeyValue(key='consecutive_rejects',
                     value=str(self._follower.consecutive_rejects)),
            KeyValue(key='escalation_stop',
                     value='1' if self._escalation_stop else '0'),
            KeyValue(key='follow_block_max_ms',
                     value=f'{self._max_follow_block_s * 1e3:.1f}'),
        ]
        self.diagnostics_pub.publish(diag)

        # Feedforward leg torques on leg_torques_diagnostic (TRUE Nm, extension-
        # positive) — a 5 Hz sample of the 40 Hz wire truth stashed by the emitter.
        # Gated on `streaming` (== _streaming AND _seeded, snapshotted above) AND a
        # real emitted frame (_last_torque_Nm is not None): no stale torques leak
        # after streaming stops, and nothing publishes before the first frame. When
        # the FF master flag is off the emitter stashes exact zeros — publishing
        # those is correct and desirable (it proves the producer→wire chain is live).
        torque = self._last_torque_Nm     # GIL-atomic read of the emitter's stash
        if streaming and torque is not None:
            torque_msg = Float64MultiArray()
            torque_msg.data = [float(v) for v in torque]
            self.leg_torques_pub.publish(torque_msg)

        # Reset the per-200ms windows each publish.
        self._max_emit_gap_s = 0.0
        self._max_follow_block_s = 0.0

    # ═══════════════════════════════════════════════════════════
    # Shutdown
    # ═══════════════════════════════════════════════════════════

    def on_shutdown(self):
        self.get_logger().info("Shutting down trajectory_node...")
        self._emit_stop.set()
        if self._emit_thread is not None and self._emit_thread.is_alive():
            self._emit_thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
