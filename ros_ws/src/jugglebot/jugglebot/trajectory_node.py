"""trajectory_node — thin ROS 2 wrapper around the MVP trajectory generator.

The Jetson-side replacement for ``run_mpc.py``'s hot path (Phase 1 of
``plans/active/mvp-trajectory-bringup.md``). A dedicated 40 Hz emitter thread
streams ``make_mpc_command`` knot frames on ZMQ :5557 — the exact seam
``HardwarePlant`` used — which ``teensy_bridge_node``'s ``_MpcCommandSetpointSource``
consumes unchanged, feeding ``SetpointPump`` → the can-hub Teensy's 500 Hz
Hermite. Phase 1 streams only a **hold**: the platform holds the pose measured from
``robot_state`` telemetry, so the first commanded ``u0`` lands inside both the
pump's 0.3 rev step gate and the firmware's 0.5 rev MAX_DEVIATION backstop.

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
publishes the active plan's measured leg peaks + emitter jitter. ``timed_target``
and the CATCH path land in Phase 5.

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
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from jugglebot_interfaces.msg import (
    PlatformPoseCommand,
    RobotState,
    TrajectoryStatus,
)
from jugglebot_interfaces.srv import GoToPose, SetTrajectoryLimits

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
    TargetFollower,
    TrajectoryInfeasible,
    TrajectoryLimits,
)
from jugglebot.motion.trajectory import feasibility as feas
from jugglebot.motion.trajectory import planner

# The control mode in which explicit move services (go_to_pose) are accepted. In
# any other streaming mode (STANDBY holds; SPACEMOUSE/CATCH have their own command
# sources) a go_to_pose is rejected WRONG_MODE — loudly, never silently — so the
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
_BUSY = 'BUSY'


# Control modes in which the node streams a hold. STANDBY is the armed-hold state
# the Phase 1 arming bring-up uses; the active sub-modes are included so streaming
# stays on across the modes later phases drive (TRAJECTORY lands in Phase 2). A
# mode outside this set stops streaming and forces a re-seed on the next entry.
_DEFAULT_STREAM_MODES = ('STANDBY', 'TRAJECTORY', 'SPACEMOUSE', 'GUI', 'SHELL',
                         'CATCH')

# Modes in which the SpaceMouse/GUI/shell streaming-target follower (Phase 3) is
# active — the pose-command modes, mirroring mpc_bridge_node._POSE_MODES. In these
# modes the emitter tick drains the latest platform_pose target and replans toward
# it (fast validate_follow gate); other streaming modes hold (STANDBY) or use their
# own command source (TRAJECTORY→go_to_pose, CATCH→dynamic_target, Phase 5).
_FOLLOWER_MODES = frozenset({'SPACEMOUSE', 'SHELL', 'GUI'})

# Modes that actively COMMAND platform motion (as opposed to STANDBY, which holds).
# Leaving one of these for a non-motion streaming mode (STANDBY) while a move is in
# flight installs an always-valid graceful stop — the move is silenced, not left to
# run on to its target.
_MOTION_MODES = _FOLLOWER_MODES | {'TRAJECTORY'}

_NUM_LEGS = 6


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

        # ── Parameters ─────────────────────────────────────────
        self.declare_parameter('control_mode_topic', 'control_mode_topic')
        self.declare_parameter('go_home_duration_s', 2.0)
        self.declare_parameter('robot_state_stale_s', 0.5)
        # Follower input-loss timeout: if no fresh platform_pose target arrives
        # within this window (the SpaceMouse node itself publishes an ACTIVE-pose
        # hold on unplug, so this is the backstop for that node dying entirely), the
        # emitter installs an always-valid graceful stop. Kept well under the 250 ms
        # staleness E-STOP — the emitter keeps streaming hold frames throughout, so
        # input loss never approaches a wire-staleness fault.
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
        # Gravity-levelling correction (identity = none), composed into follower
        # target orientations — ported from mpc_bridge_node.
        self._gravity_correction = np.eye(3)

        # ── Plan + streaming state (written on ROS + emitter threads) ──
        self._plan_lock = threading.Lock()
        self._active_plan = None          # current TrajectoryPlan (None until seeded)
        self._plan_t0 = None              # perf_counter at install (plan-time origin)
        self._seeded = False              # a real telemetry seed has been installed
        self._streaming = False           # current mode is a streaming mode
        self._current_mode = ''
        self._seq = 0
        self._last_motor_rev = None       # prior emitted u0 (step-bound defence)
        self._last_pose = self._neutral_pose.copy()
        self._last_rejection = ''
        # Latest robot_state seed source (pos_estimate rev, Jugglebot ext convention).
        self._latest_pos_rev = None
        self._robot_state_mono = 0.0
        # Emitter jitter diagnostic (max inter-tick gap over the last window).
        self._max_emit_gap_s = 0.0
        self._last_emit_mono = None
        # Measured leg peaks of the last accepted move (for trajectory/diagnostics).
        self._last_peak_vel_mmps = 0.0
        self._last_peak_acc_mmps2 = 0.0
        self._last_peak_jerk_mmps3 = 0.0

        self._pub = None
        self._emit_stop = threading.Event()
        self._emit_thread = None

        # ── Subscriptions ───────────────────────────────────────
        mode_topic = str(self.get_parameter('control_mode_topic').value)
        self.create_subscription(String, mode_topic, self._on_control_mode, 10)
        self.create_subscription(RobotState, 'robot_state',
                                 self._on_robot_state, 10)
        # SpaceMouse / GUI / shell streaming target (Phase 3).
        self.create_subscription(PlatformPoseCommand, 'platform_pose_topic',
                                 self._on_platform_pose, 10)
        # Gravity-levelling correction (verbatim port from mpc_bridge_node).
        self.create_subscription(Float64MultiArray, 'gravity_offset',
                                 self._on_gravity_offset, 10)

        # ── Services ────────────────────────────────────────────
        self.create_service(Trigger, 'trajectory/hold', self._svc_hold)
        self.create_service(Trigger, 'trajectory/go_home', self._svc_go_home)
        self.create_service(GoToPose, 'trajectory/go_to_pose',
                            self._svc_go_to_pose)
        self.create_service(SetTrajectoryLimits, 'trajectory/set_limits',
                            self._svc_set_limits)

        # ── Status + diagnostics publication (5 Hz) ─────────────
        self.status_pub = self.create_publisher(
            TrajectoryStatus, 'trajectory/status', 10)
        self.diagnostics_pub = self.create_publisher(
            DiagnosticStatus, 'trajectory/diagnostics', 10)
        self.create_timer(0.2, self._publish_status)

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
        """Emit one knot frame for wall time ``now`` (perf_counter). Public for tests."""
        # Jitter diagnostic.
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

        # Follower replan (SpaceMouse/GUI/shell): drain the latest target and
        # possibly install a fresh plan BEFORE sampling this tick's frame. The fast
        # validate_follow gate (~3-4 ms) keeps this well inside the 25 ms emit
        # budget; a contained exception in here must never stop the stream.
        if mode in _FOLLOWER_MODES:
            try:
                if self._follower_tick(now):
                    with self._plan_lock:
                        plan = self._active_plan
                        t0 = self._plan_t0
            except Exception as e:  # noqa: BLE001 — one replan must not kill streaming
                self.get_logger().error(
                    f"follower tick error (contained): {e}",
                    throttle_duration_sec=1.0)

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
        self._seq += 1

    # ═══════════════════════════════════════════════════════════
    # Subscriptions
    # ═══════════════════════════════════════════════════════════

    def _on_control_mode(self, msg) -> None:
        mode = str(msg.data)
        if mode == self._current_mode:
            return
        if mode in self._stream_modes:
            # Leaving a motion mode (TRAJECTORY / a follower) for a non-motion
            # streaming mode (STANDBY) while a move is in flight: STANDBY silences
            # commands but keeps streaming, so the in-flight move must be stopped
            # with a C2 decel-to-rest rather than run on to its target. Snapshot
            # this BEFORE mutating _current_mode; _active_move_in_flight locks
            # itself.
            prev_mode = self._current_mode
            leaving_motion_move = (
                prev_mode in _MOTION_MODES and mode not in _MOTION_MODES
                and self._active_move_in_flight())
            entering_follower = (mode in _FOLLOWER_MODES and mode != prev_mode)
            # Streaming-state writes go under _plan_lock: the emitter thread
            # snapshots _streaming/_seeded under it in _emit_once, so an unlocked
            # write could tear against that snapshot.
            with self._plan_lock:
                self._current_mode = mode
                self._streaming = True
            if entering_follower:
                # Drop any stale target/last-target so the first target of this
                # session always replans (never deadbanded against a prior session).
                self._follower.reset()
                self._follower_target = None
                self._follower_input_lost = False
            if leaving_motion_move:
                # Sample the live (moving) state and install an ALWAYS-VALID
                # graceful stop: a duration-stretched decel-to-rest that lengthens
                # its horizon until the gate passes, so a high-velocity mid-move
                # exit can no longer fall through to "let the move complete". The
                # stop decelerates in place (never runs on to the old target).
                # _current_state/_install take the lock themselves.
                try:
                    stop = planner.build_graceful_stop(
                        self._current_state(), self._limits, self._geom)
                    self._install(stop)
                    self.get_logger().info(
                        f"left {prev_mode} mid-move for {mode} — "
                        "installed a graceful stop (move silenced)")
                except TrajectoryInfeasible as e:
                    # build_graceful_stop is always-valid for a finite seed, so this
                    # should be unreachable; log loudly if it ever fires.
                    self._last_rejection = str(e)
                    self.get_logger().error(
                        f"graceful stop unexpectedly rejected on mode exit ({e})")
            # Seed a hold at the MEASURED pose on stream start (never nominal) —
            # this is what keeps the first u0 inside the pump/firmware gates — but
            # ONLY from FRESH telemetry. Stale/absent telemetry ⇒ defer: the next
            # _on_robot_state callback (inherently fresh) performs the seed.
            # NB: _seed_hold_from acquires _plan_lock itself — call it UNLOCKED.
            if not self._seeded:
                if self._robot_state_fresh():
                    self._seed_hold_from(self._latest_pos_rev)
                else:
                    self.get_logger().error(
                        "telemetry stale — waiting for fresh robot_state before "
                        "seeding")
            self.get_logger().info(f"streaming ENABLED (mode {mode})")
        else:
            # Leaving the streaming set: stop publishing and require a fresh seed
            # on the next entry (the measured pose may have moved meanwhile).
            with self._plan_lock:
                self._current_mode = mode
                self._streaming = False
                self._seeded = False
                self._last_motor_rev = None
            self.get_logger().info(f"streaming DISABLED (mode {mode})")

    def _on_robot_state(self, msg) -> None:
        states = getattr(msg, 'motor_states', None)
        if not states or len(states) < _NUM_LEGS:
            return
        pos_rev = [float(states[i].pos_estimate) for i in range(_NUM_LEGS)]
        self._latest_pos_rev = pos_rev
        self._robot_state_mono = time.perf_counter()
        # Seed on the first telemetry after streaming is enabled.
        if self._streaming and not self._seeded:
            self._seed_hold_from(pos_rev)

    def _robot_state_fresh(self) -> bool:
        """True iff a robot_state seed exists AND is within the staleness window."""
        return (self._latest_pos_rev is not None
                and (time.perf_counter() - self._robot_state_mono)
                <= self._robot_state_stale_s)

    def _seed_hold_from(self, pos_rev) -> None:
        """Seed a hold at the measured pose (pos_estimate rev → ext → FK → gate)."""
        # Defensive freshness gate: never seed from stale/absent telemetry — a
        # stale measured pose could place the first u0 outside the pump/firmware
        # gates. The _on_robot_state seed path is inherently fresh (mono stamped
        # immediately before the call), so this only guards a stray caller.
        if not self._robot_state_fresh():
            self.get_logger().error(
                "telemetry stale — waiting for fresh robot_state before seeding")
            return
        ext_mm = np.asarray(pos_rev, dtype=float) / self._mm_to_rev
        try:
            pos, rot, _J = leg_lengths_to_pose(ext_mm, self._geom)
        except RuntimeError as e:
            self.get_logger().error(
                f"seed FK failed ({e}) — not streaming until a valid state")
            return
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
                "state")
            return
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
        self.get_logger().info(
            f"seeded hold at pose x={pose[0]:.1f} y={pose[1]:.1f} "
            f"z={pose[2]:.1f} mm (from measured telemetry)")

    # ═══════════════════════════════════════════════════════════
    # SpaceMouse / GUI / shell follower (Phase 3)
    # ═══════════════════════════════════════════════════════════

    def _on_platform_pose(self, msg) -> None:
        """Store the latest streaming target (SPACEMOUSE/GUI/SHELL).

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
        No fresh target within the input-loss window → install an always-valid
        graceful stop ONCE (decel-to-rest in place), so a dead SpaceMouse node
        leaves the platform stopped, not drifting on a stale plan.
        """
        target_tuple = self._follower_target
        fresh = (target_tuple is not None
                 and (now - target_tuple[1]) <= self._follower_input_loss_s)

        if fresh:
            self._follower_input_lost = False
            result = self._follower.follow(
                self._current_state(), target_tuple[0], self._limits)
            if result.saturated:
                self.get_logger().warning(
                    "follower target outside workspace — clamped to nearest "
                    "reachable pose", throttle_duration_sec=1.0)
            if result.rejection:
                self._last_rejection = result.rejection
                self.get_logger().warning(
                    f"follower target rejected ({result.rejection}) — holding "
                    "last valid plan", throttle_duration_sec=1.0)
            if result.plan is not None:
                self._install(result.plan)
                if result.report is not None:
                    self._last_peak_vel_mmps = result.report.peak_leg_vel_mmps
                    self._last_peak_acc_mmps2 = result.report.peak_leg_acc_mmps2
                    self._last_peak_jerk_mmps3 = result.report.peak_leg_jerk_mmps3
                return True
            return False

        # Input loss: stop once (a stretched, always-valid decel-to-rest).
        if not self._follower_input_lost:
            self._follower_input_lost = True
            try:
                stop = planner.build_graceful_stop(
                    self._current_state(), self._limits, self._geom)
            except TrajectoryInfeasible as e:
                self._last_rejection = str(e)
                self.get_logger().error(
                    f"follower input-loss stop rejected ({e})")
                return False
            self._install(stop)
            self.get_logger().warning(
                "follower input loss — installed a graceful stop",
                throttle_duration_sec=1.0)
            return True
        return False

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

    def _install(self, plan) -> None:
        now = time.perf_counter()
        with self._plan_lock:
            self._active_plan = plan
            self._plan_t0 = now

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
        """``trajectory/hold``: freeze at the current pose (profiled decel-to-rest)."""
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
        response.message = 'holding at current pose'
        return response

    def _svc_go_home(self, request, response):
        """``trajectory/go_home``: profiled return to the neutral active pose."""
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
                            f'over {self._go_home_duration_s:.2f}s')
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
        # Gate: TRAJECTORY mode only. A move from STANDBY/SPACEMOUSE/CATCH is a
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
        try:
            plan, report = planner.build_move(
                self._current_state(), target,
                duration if duration > 0.0 else None,
                self._limits, self._geom)
        except TrajectoryInfeasible as e:
            response.accepted = False
            response.code = e.code
            response.message = str(e)
            response.min_duration_s = float(e.min_duration_s)
            self._last_rejection = str(e)
            self.get_logger().error(f"go_to_pose rejected: {e}")
            return response

        # Install-continuity guard. build_move seeds from _current_state() at entry,
        # but the full gate takes ~1.5 s (4-5 validate passes) while the emitter
        # keeps streaming the OLD plan. If the commanded state moved during planning,
        # installing this plan would jump u0 from the drifted live position back to
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

        self._install(plan)
        response.accepted = True
        response.code = feas.OK
        response.planned_duration_s = float(plan.total_duration)
        response.message = (
            f"move accepted: target (x={target[0]:.1f} y={target[1]:.1f} "
            f"z={target[2]:.1f} mm) over {plan.total_duration:.3f}s")
        return response

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
        diag.values = [
            KeyValue(key='peak_leg_vel_mmps',
                     value=f'{self._last_peak_vel_mmps:.1f}'),
            KeyValue(key='peak_leg_acc_mmps2',
                     value=f'{self._last_peak_acc_mmps2:.1f}'),
            KeyValue(key='peak_leg_jerk_mmps3',
                     value=f'{self._last_peak_jerk_mmps3:.0f}'),
            KeyValue(key='max_emit_gap_ms', value=f'{gap_ms:.1f}'),
            KeyValue(key='plan_kind', value=kind),
        ]
        self.diagnostics_pub.publish(diag)
        # Reset the jitter window each publish so the metric is per-200ms.
        self._max_emit_gap_s = 0.0

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
