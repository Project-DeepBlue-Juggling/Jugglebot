"""ROS2 wrapper for the catch coordinator.

Subscribes to:
  - balls (BallStateArray) — tracked balls from ball_tracker_node
  - catch/armed (Bool) — the reload action's catch-armed latch state. The hand
    prime/arm is GATED on this: the hand is actuated ONLY during a reload (latch
    raised), never on a stray tracked ball. The reactive-fire TIMING itself
    (set_hand_traj_cmd) stays here; only its enablement is gated. The hand primes
    on the ARM edge (+ an off-ball-path retry timer) — NEVER from the balls path,
    where a smooth-move would race the armed catch stroke on the Teensy's single
    packed queue (3/6 strokes lost that way, 2026-07-23).
  - throw_announcements (ThrowAnnouncement) — while armed, OUR announcement (strict
    target_id match) drives a PRE-TILT: one predicted catch target per announcement
    from the announced landing state, ~3.9 s early, so the platform settles into
    the receive tilt during the countdown; the reactive path refines it mid-flight.
  - catch/vel_scale (Float64) — the operator's per-attempt catch-speed knob
    (reload goal field, or published manually); scales the armed event velocity;
    reset to the config default (JB_OP_CATCH_VEL_SCALE_DEFAULT, 0.8) on disarm.

Publishes:
  - catch/dynamic_target (DynamicTargetCommand) — consumed by trajectory_node, which
    turns it into a build_catch plan while the catch-armed latch is raised. Published
    unconditionally (trajectory_node's own latch gate drops it when disarmed);
    arrival_time is in the perf_counter domain (system-wide CLOCK_MONOTONIC on Linux).

Subscribes:
  - trajectory/target_feedback (TargetFeedback) — accept/reject decision from
    trajectory_node's feasibility gate. Replaces the dormant MPC process's ZMQ
    :5559 feedback (TargetFeedbackSub); on a rejection it drives the feasibility
    blacklist (blacklist semantics preserved unchanged).

Services called (on can_node):
  - smooth_move_hand (SetFloat) — prime hand to top of stroke
  - set_hand_traj_cmd (SetHandTrajCmd) — arm catch trajectory on Teensy
  - set_hand_gains (SetHandGains) — adjust hand PID gains for catch

Clock domain conversion: ROS2 landing_time → perf_counter arrival_time.
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Bool, Float64
from jugglebot_interfaces.msg import (
    BallStateArray,
    DynamicTargetCommand,
    TargetFeedback,
    ThrowAnnouncement,
)
from jugglebot_interfaces.srv import SetFloat, SetHandGains, SetHandTrajCmd
from geometry_msgs.msg import Point, Quaternion, Vector3

import jugglebot.hardware_config as hw
from jugglebot.tracking.ball import Ball, BallStatus, TrackingConfidence
from jugglebot.catch_coordinator import CatchCoordinator

# Hand catch trajectory type (Teensy convention)
_TRAJ_TYPE_CATCH = 1

# Minimum event_delay (seconds) — below this the Teensy may not have time
# to smooth-move to the starting position before the catch window.
_MIN_EVENT_DELAY_S = 0.3

# catch/vel_scale bounds. The scale multiplies the event velocity the hand catch is
# armed with (effective hand-vs-ball speed ratio = firmware CATCH_VEL_RATIO 0.6 ×
# scale — mathematically identical to re-tuning the flash-gated firmware ratio, but
# per-attempt from the Jetson). Lower bound: below ~0.3 the scaled event velocity can
# fall under the Teensy windup budget (t_acc = 0.404/v) and the whole stroke is
# SILENTLY dropped by the prelude time-budget check; upper bound 1.5 keeps the
# clamped event velocity within the 7.0 m/s Teensy ceiling with sane decel.
_VEL_SCALE_MIN = 0.3
_VEL_SCALE_MAX = 1.5

# Suppress the primed-retry while a catch command has been emitted within this
# window: a kind-3 smooth-move (the prime) sent while a catch sequence is live
# CLEARS the Platform Teensy's armed catch trajectory (last-writer-wins on its
# single packed queue) — the 2026-07-23 re-test lost 3 of 6 catch strokes exactly
# this way (a same-tick re-prime racing the arm).
_PRIME_RETRY_QUIET_S = 1.5

# No re-prime may be dispatched while a prime ascent could still be RUNNING: a
# kind-3 re-dispatch mid-ascent rebuilds the Teensy profile from the live hand
# position at v(0)=0, yanking the moving hand backwards — the 2026-07-23 third
# sitting's "stutter" (5/12 ascents stalled ~60-70 ms with velocity reversals to
# −4 rev/s, every stall phase-locked to the 0.5 s retry tick after a failed
# dispatch ack). Ascents measure 0.68–1.05 s; this window covers them with
# margin. The window is anchored to DISPATCH, not ack — failed acks have been
# observed with the frame still transmitted and the hand moving. A re-dispatch
# AFTER the window with the hand already at top is a silent Teensy no-op
# (delta ≈ 0), so a genuinely lost dispatch still recovers on the next tick.
_PRIME_INFLIGHT_S = 1.2

# Pre-tilt early arrival: the announcement-derived target used to schedule its
# arrival AT the predicted landing, so the whole ~10.5° receive tilt was one
# min-jerk crawl completing exactly at contact (third sitting: tilt error still
# >1° until 0.24–0.49 s before landing on all 12 attempts). The platform must be
# seated well before the ball arrives: aim for landing − _PRETILT_EARLY_S, but
# never demand arrival sooner than _PRETILT_MIN_LEAD_S from now (the min-jerk
# reach needs ~0.65 s; 1.0 s keeps a profiled, non-violent traverse even on a
# late or short-countdown announcement).
_PRETILT_EARLY_S = 1.5
_PRETILT_MIN_LEAD_S = 1.0


class CatchCoordinatorNode(Node):
    def __init__(self):
        super().__init__('catch_coordinator_node')

        # Clock offset: perf_counter - ros2_time (re-measured periodically)
        self._ros_to_perf_offset = self._measure_clock_offset()
        self._clock_offset_history: list[float] = [self._ros_to_perf_offset]

        # Coordinator (pure Python policy)
        self._coordinator = CatchCoordinator(
            robot_name="jugglebot",
            initial_height_mm=hw.GEOM_INITIAL_HEIGHT_MM,
            landing_z_offset_mm=hw.JB_OP_DEFAULT_ACTIVE_Z_MM + hw.HAND_CATCH_OFFSET_MM,
            hand_catch_offset_mm=hw.HAND_CATCH_OFFSET_MM,
            catch_angle_limit_deg=30.0,
        )

        # Publisher: dynamic target → consumed by trajectory_node (CATCH mode), which
        # turns it into a tilt-through-seat catch plan via planner.build_catch (Phase 7;
        # Phase 5 routed it through the reach-only build_timed).
        self._dyn_target_pub = self.create_publisher(
            DynamicTargetCommand, 'catch/dynamic_target', 10)

        # Subscriber: tracked balls
        self._balls_sub = self.create_subscription(
            BallStateArray, 'balls', self._on_balls, 10)

        # Accept/reject feedback from trajectory_node's feasibility gate (Phase 5).
        # Replaces the dormant MPC process's ZMQ :5559 TargetFeedbackSub; the
        # blacklist logic below is unchanged.
        self._feedback_sub = self.create_subscription(
            TargetFeedback, 'trajectory/target_feedback',
            self._on_target_feedback, 10)

        # Catch-armed latch (published by reload_coordinator_node on PREPARE / RECENTER /
        # SAFE_ABORT). The hand prime/arm is gated on this so the hand is actuated ONLY
        # during a reload — never on a stray tracked ball outside one. Without CATCH mode
        # as the implicit "operator intends to catch" signal, this latch is what scopes
        # the hand actuation to a real reload.
        self._catch_armed = False
        self._catch_armed_sub = self.create_subscription(
            Bool, 'catch/armed', self._on_catch_armed, 10)

        # Pre-tilt: the throw announcement carries a solver-consistent landing
        # prediction (position on the cup plane, velocity with vz decayed, landing
        # time) ~3.9 s before the ball lands. While armed, synthesize ONE predicted
        # catch target from it so the platform settles into the receive tilt during
        # the countdown; the reactive per-ball path refines it mid-flight via C2
        # supersede (each accepted refinement re-anchors the reach freeze).
        self._announcement_sub = self.create_subscription(
            ThrowAnnouncement, 'throw_announcements',
            self._on_throw_announcement, 10)

        # Operator catch-speed knob (catch/vel_scale, published by the reload action
        # from its goal — or manually for bench throws). Scales the event velocity
        # the hand catch is armed with; reset to the config default
        # (JB_OP_CATCH_VEL_SCALE_DEFAULT, 0.8 locked in from the 2026-07-23 third
        # sitting) on the disarm edge so one reload's tuning value never leaks
        # into the next.
        self._catch_vel_scale = float(hw.JB_OP_CATCH_VEL_SCALE_DEFAULT)
        self._vel_scale_sub = self.create_subscription(
            Float64, 'catch/vel_scale', self._on_vel_scale, 10)

        # Prime-retry plumbing: monotonic time of the last emitted catch command
        # (any catch cmd within _PRIME_RETRY_QUIET_S suppresses re-priming — see the
        # constant's comment for the Teensy last-writer-wins hazard).
        self._last_cmd_mono = 0.0
        self._prime_retry_timer = self.create_timer(0.5, self._prime_retry_tick)
        # Anti-stutter in-flight window: monotonic time of the last hand-prime
        # DISPATCH from either owner — this node's own _prime_hand, or the reload
        # coordinator's ACTION_PRIME_HAND announced on catch/prime_dispatched
        # (two nodes own priming and cannot see each other's service calls; the
        # Teensy queue is last-writer-wins, so a cross-node re-prime mid-ascent
        # stutters the hand).
        self._prime_dispatch_mono = 0.0
        self._prime_dispatched_sub = self.create_subscription(
            Bool, 'catch/prime_dispatched', self._on_prime_dispatched, 10)

        # Re-measure clock offset every 30s to track drift
        self._clock_timer = self.create_timer(30.0, self._refresh_clock_offset)

        # Track which ball we last submitted a target for
        self._last_submitted_ball_id: int | None = None
        self._last_arrival_time: float = 0.0
        self._last_landing_position: np.ndarray = np.zeros(3)

        # ── Hand control state ────────────────────────────────────
        self._hand_primed = False
        self._hand_traj_armed_for_ball: int | None = None  # ball_id of last armed traj
        self._min_delay_logged_for_ball: int | None = None  # once-per-ball floor log
        self._catch_gains_active = False

        # Catch-mode hand gains (softer than defaults for compliant catch).
        # These can be tuned — the key insight is that softer position gain
        # gives the hand more compliance during impact.
        self._catch_hand_gains = {
            'pos_gain': 20.0,
            'vel_gain': hw.ODRIVE_HAND_VEL_GAIN,
            'vel_int_gain': hw.ODRIVE_HAND_VEL_INT_GAIN,
        }

        # ── Hand service clients ──────────────────────────────────
        self._smooth_move_client = self.create_client(
            SetFloat, 'smooth_move_hand')
        self._hand_traj_client = self.create_client(
            SetHandTrajCmd, 'set_hand_traj_cmd')
        self._hand_gains_client = self.create_client(
            SetHandGains, 'set_hand_gains')

        self.get_logger().info(
            f"CatchCoordinatorNode ready: "
            f"ros_to_perf_offset={self._ros_to_perf_offset:.6f}s")

    # ==================================================================
    # Clock offset
    # ==================================================================

    def _measure_clock_offset(self) -> float:
        """Measure offset between perf_counter and ROS2 wall clock."""
        offsets = []
        for _ in range(10):
            t_perf = time.perf_counter()
            t_ros = self.get_clock().now().nanoseconds / 1e9
            offsets.append(t_perf - t_ros)
        return float(np.median(offsets))

    def _refresh_clock_offset(self):
        """Periodically re-measure clock offset to track drift."""
        new_offset = self._measure_clock_offset()
        self._clock_offset_history.append(new_offset)
        # Keep last 20 measurements (10 minutes at 30s interval)
        if len(self._clock_offset_history) > 20:
            self._clock_offset_history.pop(0)
        self._ros_to_perf_offset = float(np.median(self._clock_offset_history))

    # ==================================================================
    # Ball processing
    # ==================================================================

    def _on_catch_armed(self, msg: Bool):
        """Track the reload action's catch-armed latch — the gate for hand actuation.

        On the ARM (rising) edge, prime the hand IMMEDIATELY — do not wait for a
        tracked ball. The 2026-07-23 hardware session measured the bottom→top
        smooth-move at ~0.7 s against a 0.878 s BB flight: a prime that waits for
        the ball to appear on ``balls`` is a coin flip (lost by 0.06 s, won by
        0.09 s), and a hand still mid-prime at fire time makes the Teensy silently
        drop the whole catch stroke (its smooth-move prelude time-budget check).
        Priming at the edge covers both the reload action (which also primes at
        sequence start — this re-prime is idempotent) and the manual static-catch
        flow (publish ``catch/armed`` true, throw by hand).

        On DISARM (reload ended / aborted) reset the one-shot flags so the NEXT reload
        re-primes + re-arms the hand from a clean state; a stale ``_hand_primed`` /
        ``_hand_traj_armed_for_ball`` would otherwise suppress the next reload's prime
        and arm."""
        armed = bool(msg.data)
        if armed == self._catch_armed:
            return
        self._catch_armed = armed
        if armed:
            # Skip the edge prime while a prime ascent may already be running
            # (the reload coordinator primes at CHECKING, ~0.1 s before this
            # edge — the third sitting showed the pair restarting a just-started
            # ascent on 3/12 attempts). The retry tick re-primes after the
            # window if the ascent never actually happened.
            if (time.perf_counter() - self._prime_dispatch_mono) >= _PRIME_INFLIGHT_S:
                self._prime_hand()
        else:
            self._hand_primed = False
            self._hand_traj_armed_for_ball = None
            # One reload's catch-speed tuning value must never leak into the next.
            self._catch_vel_scale = float(hw.JB_OP_CATCH_VEL_SCALE_DEFAULT)

    def _on_prime_dispatched(self, msg: Bool):
        """catch/prime_dispatched: the reload coordinator dispatched its own hand
        prime (ACTION_PRIME_HAND). Stamp the anti-stutter window so this node's
        edge prime / retry tick never restart that ascent — the two nodes cannot
        see each other's service calls, and the Teensy trajectory queue is
        last-writer-wins."""
        if bool(msg.data):
            self._prime_dispatch_mono = time.perf_counter()

    def _on_vel_scale(self, msg: Float64):
        """catch/vel_scale: the operator's per-attempt catch-speed knob (reload goal
        field, or published manually for bench throws). Clamped to the safe range —
        below it the Teensy's windup budget silently drops the stroke, above it the
        event-velocity ceiling binds."""
        raw = float(msg.data)
        scale = max(_VEL_SCALE_MIN, min(_VEL_SCALE_MAX, raw))
        if scale != raw:
            self.get_logger().warning(
                f"catch/vel_scale {raw:.2f} outside [{_VEL_SCALE_MIN}, "
                f"{_VEL_SCALE_MAX}] — clamped to {scale:.2f}")
        self._catch_vel_scale = scale
        self.get_logger().info(f"catch vel scale = {scale:.2f}")

    def _on_throw_announcement(self, msg: ThrowAnnouncement):
        """Pre-tilt on OUR announcement (while armed): publish one predicted catch
        target from the announced landing state so the platform settles into the
        receive tilt during the ~3 s countdown. The reactive path supersedes it
        mid-flight. Deliberately does NOT touch the per-ball correlation state
        (_last_submitted_ball_id etc.) — the synthetic target has no tracker ball,
        must never feed the feasibility blacklist, and must not suppress the real
        ball's hand-arm."""
        if not self._catch_armed:
            return
        # STRICT target match (audit): the reload path always names the target
        # (bb/throw_at_target's target_name), so an untagged announcement is NOT
        # ours and must not move the platform.
        target_id = str(getattr(msg, 'target_id', '') or '')
        if target_id != self._coordinator.robot_name:
            return
        landing_pos = np.array([
            msg.landing_position.x, msg.landing_position.y, msg.landing_position.z])
        landing_vel = np.array([
            msg.landing_velocity.x, msg.landing_velocity.y, msg.landing_velocity.z])
        lt = msg.landing_time
        landing_time = float(lt.sec) + float(lt.nanosec) * 1e-9
        if landing_time <= 0.0:
            return
        cmd = self._coordinator.predicted_catch_command(
            landing_pos, landing_vel, landing_time)
        if cmd is None:
            return
        # Arrive EARLY, not just-in-time: an arrival equal to the landing time
        # makes trajectory_node span the whole announce→land window with a single
        # min-jerk reach that completes AT contact (third sitting: tilt still >1°
        # off until 0.24–0.49 s before landing). Aim for landing −
        # _PRETILT_EARLY_S so the platform is seated well before the ball
        # arrives; the max() keeps a feasible profiled traverse on a late/short
        # announcement, and the min() guarantees arrival is never scheduled
        # after landing.
        landing_perf = cmd.landing_time + self._ros_to_perf_offset
        arrival_perf = min(landing_perf,
                           max(landing_perf - _PRETILT_EARLY_S,
                               time.perf_counter() + _PRETILT_MIN_LEAD_S))
        self._publish_dynamic_target(cmd, arrival_perf)
        self.get_logger().info(
            f"pre-tilt target published from announcement (landing in "
            f"{landing_time - self.get_clock().now().nanoseconds * 1e-9:.2f} s, "
            f"arrival {landing_perf - arrival_perf:.2f} s early)")

    def _prime_retry_tick(self):
        """Retry the hand prime while armed and not yet confirmed primed — but NEVER
        while a catch sequence is live (any catch command within the quiet window):
        a kind-3 smooth-move clears the Platform Teensy's single packed trajectory
        queue, erasing an armed catch stroke (the 2026-07-23 re-test lost 3/6
        strokes to exactly this race). Off the ball path, on a slow timer, the
        retry is safe: pre-flight it is idempotent re-priming; in-flight it is
        suppressed."""
        if not self._catch_armed or self._hand_primed:
            return
        # An ARMED catch stroke on the Teensy blocks the retry outright (audit,
        # 2026-07-23): the quiet window is anchored to the last EMITTED command,
        # which stops ~0.3 s before landing — a slow disarm round-trip after the
        # catch could otherwise unblock the retry while the stroke (or its settle)
        # is still live, and a kind-3 would clobber it. The flag clears on the
        # disarm edge and on arm failure, so pre-flight retries stay allowed.
        if self._hand_traj_armed_for_ball is not None:
            return
        if (time.perf_counter() - self._last_cmd_mono) < _PRIME_RETRY_QUIET_S:
            return
        # A prime ascent may still be running (dispatched by either owner):
        # re-dispatching now would rebuild the profile mid-ascent and stutter the
        # hand — the third sitting's 5/12 stalled ascents were exactly this tick
        # firing 0.5 s into a ~0.8 s ascent whose dispatch ack had failed.
        if (time.perf_counter() - self._prime_dispatch_mono) < _PRIME_INFLIGHT_S:
            return
        self._prime_hand()

    def _on_balls(self, msg: BallStateArray):
        """Process ball state updates: select best ball and send dynamic target."""
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Convert ROS2 messages to Ball objects for the coordinator
        balls = [self._msg_to_ball(b) for b in msg.balls]

        # Run coordinator policy
        cmd = self._coordinator.update(balls, current_time)
        if cmd is None:
            return

        # A catch sequence is live: stamp the quiet window that suppresses the
        # prime-retry timer. NO PRIME is ever dispatched from this path — a kind-3
        # smooth-move here clears the Platform Teensy's armed catch stroke
        # (last-writer-wins; 3/6 strokes lost to that race, 2026-07-23). Priming
        # belongs to the catch/armed rising edge + the off-path retry timer.
        self._last_cmd_mono = time.perf_counter()

        # Convert landing_time from ROS2 clock → perf_counter clock
        arrival_time_perf = cmd.landing_time + self._ros_to_perf_offset
        self._publish_dynamic_target(cmd, arrival_time_perf)

        self._last_submitted_ball_id = cmd.ball_id
        self._last_arrival_time = arrival_time_perf
        self._last_landing_position = cmd.target_pos.copy()

        # Arm hand catch trajectory (once per ball, after first target submit) — ONLY
        # during a reload (catch armed), so a stray tracked ball never arms the reactive
        # catch stroke on the Teensy. The reactive-fire TIMING (event_delay / event_vel)
        # is computed here as before; only its enablement is gated. The event velocity
        # carries the operator's catch/vel_scale knob (re-clamped to Teensy bounds).
        # The one-shot latch is set ONLY when the arm was actually dispatched —
        # a service-not-ready early return must be retried next tick, not silently
        # latched as armed (audit of the 2026-07-23 re-test).
        if (self._catch_armed and cmd.arm_hand
                and self._hand_traj_armed_for_ball != cmd.ball_id):
            event_delay = cmd.landing_time - current_time
            if event_delay >= _MIN_EVENT_DELAY_S:
                event_vel = max(0.3, min(7.0,
                                         cmd.event_vel_mps * self._catch_vel_scale))
                if self._arm_hand_catch(event_delay, event_vel):
                    self._hand_traj_armed_for_ball = cmd.ball_id
            elif self._min_delay_logged_for_ball != cmd.ball_id:
                # Previously a SILENT drop: the delay only shrinks in flight, so a
                # ball first seen < 0.3 s out never arms — say so, once per ball.
                self._min_delay_logged_for_ball = cmd.ball_id
                self.get_logger().warning(
                    f"Ball {cmd.ball_id}: event_delay {event_delay:.2f} s below the "
                    f"{_MIN_EVENT_DELAY_S} s floor — hand catch NOT armed")

    def _publish_dynamic_target(self, cmd, arrival_time_perf: float):
        """Pack a CatchCommand into the DynamicTargetCommand wire message (verbatim
        pose — the wire is STOW-relative, consumed with no conversion). Shared by the
        reactive per-ball path and the announcement pre-tilt."""
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
        out.arrival_time = arrival_time_perf
        self._dyn_target_pub.publish(out)

    # ==================================================================
    # Feedback
    # ==================================================================

    # Service-level (non-feasibility) reject codes that must NOT count toward the
    # position blacklist: the target's reachability was never actually evaluated
    # (STALE_STATE) or a committed reach was being held (FROZEN). Only feasibility-class
    # codes (WORKSPACE/UNREACHABLE/LIMIT_*/TOO_FAST/STEP_BOUND) mean the position itself
    # is unreachable and should drive the blacklist.
    _NON_BLACKLIST_CODES = frozenset({'STALE_STATE', 'FROZEN'})

    def _on_target_feedback(self, msg: TargetFeedback):
        """Accept/reject feedback from trajectory_node (Phase 5 topic swap).

        Same correlation + blacklist semantics as the old ZMQ :5559 poll: match the
        feedback's ``arrival_time`` (perf domain — the exact value we published) to
        the last submitted target, then feed acceptance/rejection to the coordinator
        so the feasibility blacklist tracks unreachable catch targets.
        """
        # trajectory/target_feedback carries BOTH catch and timed-service decisions;
        # only the catch source is ours. A timed-target reject must never touch the
        # catch blacklist.
        if msg.source != 'catch':
            return

        ball_id = self._last_submitted_ball_id
        if ball_id is None:
            return

        # Correlate by arrival_time (approximate match, same 0.1 s window as before).
        if abs(float(msg.arrival_time) - self._last_arrival_time) > 0.1:
            return  # Stale / unrelated feedback, ignore

        if msg.accepted:
            self._coordinator.report_acceptance(ball_id)
            self.get_logger().debug(f"Ball {ball_id}: target accepted")
            return

        # A non-feasibility service code (STALE_STATE/FROZEN) is neither an acceptance
        # nor a feasibility rejection — early-return so it never counts toward the
        # blacklist (an unlucky burst of these would otherwise blacklist a perfectly
        # reachable position).
        if msg.code in self._NON_BLACKLIST_CODES:
            self.get_logger().debug(
                f"Ball {ball_id}: {msg.code} (not blacklist-counted)")
            return

        self._coordinator.report_rejection_with_position(
            ball_id, self._last_landing_position)
        self.get_logger().info(
            f"Ball {ball_id}: target rejected — {msg.code}: {msg.reason}")

    # ==================================================================
    # Hand control
    # ==================================================================

    def _prime_hand(self):
        """Move hand to top of stroke and set catch gains."""
        if not self._smooth_move_client.service_is_ready():
            self.get_logger().warning(
                "smooth_move_hand service not ready — hand priming deferred")
            return

        # Set softer catch gains
        if not self._catch_gains_active:
            self._set_catch_gains()

        # Smooth-move to prime position. Stamp the anti-stutter window on
        # DISPATCH (not ack — see _PRIME_INFLIGHT_S): from here an ascent may be
        # running regardless of what the ack later says.
        req = SetFloat.Request()
        req.data = hw.JB_OP_HAND_CATCH_PRIME_REV
        self._prime_dispatch_mono = time.perf_counter()
        future = self._smooth_move_client.call_async(req)
        future.add_done_callback(self._on_prime_done)

    def _on_prime_done(self, future):
        """Callback when hand priming completes.

        On failure, _hand_primed stays False so the next _on_balls cycle retries.
        """
        try:
            result = future.result()
            if result.success:
                self._hand_primed = True
                self.get_logger().info(
                    f"Hand primed to {hw.JB_OP_HAND_CATCH_PRIME_REV:.3f} rev")
            else:
                self.get_logger().warning(
                    f"Hand priming failed: {result.message}")
        except Exception as e:
            self.get_logger().warning(f"Hand priming service error: {e}")

    def _arm_hand_catch(self, event_delay: float, event_vel_mps: float) -> bool:
        """Arm the hand catch trajectory on the Teensy. Returns True iff the arm was
        actually DISPATCHED — the caller's one-shot latch keys off this, so a
        service-not-ready early return is retried on the next balls tick instead of
        being silently latched as armed."""
        if not self._hand_traj_client.service_is_ready():
            self.get_logger().warning(
                "set_hand_traj_cmd service not ready — hand catch not armed")
            return False

        req = SetHandTrajCmd.Request()
        req.event_delay = float(event_delay)
        req.event_vel = float(event_vel_mps)
        req.traj_type = _TRAJ_TYPE_CATCH

        future = self._hand_traj_client.call_async(req)
        future.add_done_callback(self._on_hand_traj_done)

        self.get_logger().info(
            f"Arming hand catch: delay={event_delay:.2f}s, "
            f"vel={event_vel_mps:.2f} m/s (scale {self._catch_vel_scale:.2f})")
        return True

    def _on_hand_traj_done(self, future):
        """Callback when hand trajectory arm completes."""
        try:
            result = future.result()
            if result.success:
                self.get_logger().info("Hand catch trajectory armed on Teensy")
            else:
                # Clear armed flag so next update cycle retries
                self._hand_traj_armed_for_ball = None
                self.get_logger().warning(
                    f"Hand catch arm failed: {result.message}")
        except Exception as e:
            self._hand_traj_armed_for_ball = None
            self.get_logger().warning(f"Hand catch arm service error: {e}")

    def _set_catch_gains(self):
        """Set softer hand gains for catch compliance."""
        if not self._hand_gains_client.service_is_ready():
            self.get_logger().warning(
                "set_hand_gains service not ready — using default gains")
            return

        req = SetHandGains.Request()
        req.pos_gain = self._catch_hand_gains['pos_gain']
        req.vel_gain = self._catch_hand_gains['vel_gain']
        req.vel_integrator_gain = self._catch_hand_gains['vel_int_gain']

        future = self._hand_gains_client.call_async(req)
        future.add_done_callback(self._on_catch_gains_done)

    def _on_catch_gains_done(self, future):
        """Callback when catch gains are set.

        On failure, _catch_gains_active stays False so _prime_hand retries.
        """
        try:
            result = future.result()
            if result.success:
                self._catch_gains_active = True
                self.get_logger().info(
                    f"Hand catch gains set: pos={self._catch_hand_gains['pos_gain']}")
            else:
                self.get_logger().warning(
                    f"Set catch gains failed: {result.message}")
        except Exception as e:
            self.get_logger().warning(f"Set catch gains service error: {e}")

    def _restore_default_gains(self):
        """Restore default hand gains (called on shutdown / mode exit)."""
        if not self._catch_gains_active:
            return
        if not self._hand_gains_client.service_is_ready():
            return

        req = SetHandGains.Request()
        req.pos_gain = hw.ODRIVE_HAND_POS_GAIN
        req.vel_gain = hw.ODRIVE_HAND_VEL_GAIN
        req.vel_integrator_gain = hw.ODRIVE_HAND_VEL_INT_GAIN

        future = self._hand_gains_client.call_async(req)
        future.add_done_callback(self._on_restore_gains_done)

    def _on_restore_gains_done(self, future):
        """Callback when default gains are restored."""
        try:
            result = future.result()
            if result.success:
                self._catch_gains_active = False
                self.get_logger().info("Hand gains restored to defaults")
        except Exception:
            pass  # Best-effort on shutdown

    # ==================================================================
    # Utilities
    # ==================================================================

    @staticmethod
    def _msg_to_ball(msg) -> Ball:
        """Convert a ROS2 BallState message to an internal Ball object."""
        landing_time = msg.time_at_land.sec + msg.time_at_land.nanosec * 1e-9

        return Ball(
            id=msg.id,
            status=BallStatus(msg.status),
            tracking=TrackingConfidence(msg.tracking),
            source=msg.source,
            destination=msg.destination,
            position=np.array([msg.position.x, msg.position.y, msg.position.z]),
            velocity=np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z]),
            landing_position=np.array([
                msg.landing_position.x, msg.landing_position.y, msg.landing_position.z,
            ]),
            landing_velocity=np.array([
                msg.landing_velocity.x, msg.landing_velocity.y, msg.landing_velocity.z,
            ]),
            landing_time=landing_time,
        )

    def destroy_node(self):
        self.get_logger().info("Shutting down CatchCoordinatorNode.")
        self._restore_default_gains()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CatchCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
