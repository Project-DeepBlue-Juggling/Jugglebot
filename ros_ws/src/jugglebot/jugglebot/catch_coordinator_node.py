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
    the receive tilt during the countdown. Under JB_OP_RELOAD_PLATFORM_OPEN_LOOP
    (the default) the platform then HOLDS that pose for the whole flight — only the
    hand-arm stays reactive; with the flag off, the reactive path refines the
    platform mid-flight as before.
  - catch/vel_scale (Float64) — the operator's per-attempt catch-speed knob
    (reload goal field, or published manually); scales the armed event velocity;
    reset to the config default (JB_OP_CATCH_VEL_SCALE_DEFAULT, 0.8) on disarm.
  - catch/prime_hold (Bool) — the toss coordinator's prime-suppression gate
    (True at PREPARE entry, before catch/armed rises; False at terminal). While
    True, EVERY auto-prime dispatch path here (armed-edge prime, retry-tick
    re-prime) is suppressed; the catch arm and all other behaviour are
    untouched. Absent topic = False = the reload path unchanged; stale True
    fails safe (no auto-prime — the reload action primes proactively itself).
  - catch/pretilt_hold (Bool) — the Tier-8b toss coordinator's platform
    pre-tilt-suppression gate (True at PREPARE entry, alongside prime_hold;
    False at terminal). While True, OUR announcement STILL latches the
    open-loop freeze + hand-arm window but publishes NO platform pre-tilt (and
    caches none) — the toss coordinator owns the platform reach, publishing the
    ONE deferred A->B target at release. The catch arm and all other behaviour
    are untouched. Absent topic = False = the reload path unchanged; stale True
    fails DEGRADED-BUT-SAFE (a reload announcement loses its platform pre-tilt —
    the platform simply holds, the hand-arm stays tracker-driven; zero hazard).

Publishes:
  - catch/dynamic_target (DynamicTargetCommand) — consumed by trajectory_node, which
    turns it into a build_catch plan while the catch-armed latch is raised. Outside a
    reload it is published unconditionally (trajectory_node's own latch gate drops it
    when disarmed); during an announced open-loop reload the per-ball reactive target
    is suppressed and the cached pre-tilt is re-asserted instead;
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

# Arrival-window arm guard: once OUR throw is announced, only arm the hand for a ball
# whose predicted landing is within this window of the announced landing. A corrupt
# split-track whose landing prediction is far off (the current ball's own hijacked
# track) is rejected — it must not arm the one-shot stroke off garbage timing and block
# the real ball's arm. Probed against the 2026-07-24_09-07-53 bag: destination-track
# |time_at_land − announced landing| is 0.000 s at the arm moment (early life, n=6105)
# and drifts to at most 0.644 s late-life — so 0.75 s always admits the real arm and
# only fires on timing garbage beyond anything yet observed (a latent-class guard).
_ARM_LANDING_WINDOW_S = 0.75

# Per-ball hand-arm re-dispatch cap. The HAND_TRAJ_CMD ack is unreliable (~40-60%
# ERR_TIMEOUT per call, 2026-07-23 epidemic) AND lies — frames were observed
# transmitted after a failed ack. The kind-1 stroke's catch instant is an ABSOLUTE
# wall_time invariant across retries, so a lying-ack arm still physically armed; after
# this many dispatches for one ball we KEEP the latch (assume armed) rather than
# churning the Teensy's last-writer-wins queue with near-identical repacks forever.
_MAX_ARM_DISPATCHES = 2


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
        # the countdown. Under JB_OP_RELOAD_PLATFORM_OPEN_LOOP (default) the platform
        # HOLDS that pose all flight; otherwise the reactive per-ball path refines it
        # mid-flight via C2 supersede (each accepted refinement re-anchors the freeze).
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
        # Prime-suppression gate (catch/prime_hold, published by the toss
        # coordinator: True at PREPARE entry — BEFORE catch/armed rises — and
        # False at terminal). While True, no auto-prime is dispatched from this
        # node (neither the armed-edge prime nor the retry-tick re-prime): a
        # toss holds the ball at the stroke bottom through the throw, and a
        # kind-3 prime ascent would carry the ball-laden hand up mid-toss and
        # clear an armed throw stroke on the Teensy's last-writer-wins queue.
        # The catch ARM (kind-1) is NOT gated. Absent topic → False → the
        # reload path bit-identical to today. Stale True fails SAFE (no
        # auto-prime; the reload action primes proactively itself), so the
        # flag is never reset locally — the publisher owns it.
        self._prime_hold = False
        self._prime_hold_sub = self.create_subscription(
            Bool, 'catch/prime_hold', self._on_prime_hold, 10)
        # Pre-tilt suppression gate (catch/pretilt_hold, published by the Tier-8b
        # toss coordinator: True at PREPARE entry — with prime_hold, BEFORE
        # catch/armed rises — and False at terminal). While True, OUR
        # announcement still LATCHES _announcement_seen + _announced_landing_time
        # (so the open-loop freeze and the hand-arm window keep working) but
        # publishes NO platform pre-tilt and caches _pretilt_cmd = None: the
        # stock pre-tilt's arrival clamps to ~now + 1 s while the toss announces
        # >= 1 s before release, so an un-suppressed pre-tilt would COMPLETE the
        # A->B translate (and the un-tilt to the receive tilt) BEFORE the ball is
        # released — aim destroyed, a moving platform under a seated ball
        # mid-windup. The toss coordinator owns the platform reach (the ONE
        # deferred A->B target at release). The catch ARM (kind-1) and every
        # other behaviour are untouched. Absent topic → False → the reload path
        # bit-identical to today. Stale True fails DEGRADED-BUT-SAFE (a reload
        # announcement loses its platform pre-tilt; the platform simply holds,
        # the hand-arm stays tracker-driven — zero hazard), so the flag is never
        # reset locally — the publisher owns it.
        self._pretilt_hold = False
        self._pretilt_hold_sub = self.create_subscription(
            Bool, 'catch/pretilt_hold', self._on_pretilt_hold, 10)

        # Re-measure clock offset every 30s to track drift
        self._clock_timer = self.create_timer(30.0, self._refresh_clock_offset)

        # Track which ball we last submitted a target for
        self._last_submitted_ball_id: int | None = None
        self._last_arrival_time: float = 0.0
        self._last_landing_position: np.ndarray = np.zeros(3)

        # Open-loop platform state (JB_OP_RELOAD_PLATFORM_OPEN_LOOP). Once OUR throw is
        # announced during an armed reload, the platform holds the announcement-derived
        # pre-tilt pose and ignores live per-ball reactive refinements (BB throws are
        # repeatable; a bad ball prediction must never move the platform mid-reload).
        # The hand-arm below stays reactive — only the platform reach is frozen.
        self._announcement_seen = False
        self._announced_landing_time: float | None = None   # ROS seconds, for the arm-window guard
        self._pretilt_cmd = None                             # cached CatchCommand for the refresh

        # Stale-track exclusion (defense-in-depth): ids IN_FLIGHT at the catch-armed
        # rising edge — excluded from the catch candidate set so a leftover track from a
        # PRIOR attempt never drives this reload. (Cannot catch the current ball's own
        # corrupt track — that spawns after the snapshot; the arm-window guard covers it.)
        self._latest_in_flight_ids: set = set()
        self._preexisting_flight_ids: set = set()

        # Per-ball hand-arm re-dispatch counter (bounds the re-arm churn on the lying
        # ERR_TIMEOUT epidemic — see _MAX_ARM_DISPATCHES).
        self._arm_dispatch_ball_id: int | None = None
        self._arm_dispatch_count: int = 0

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
            # Snapshot the ids already IN_FLIGHT at the arm edge: a leftover track from a
            # PRIOR attempt is excluded from this reload's catch candidates (defense-in-
            # depth for the stale-track hazard; see update(exclude_ids=...)).
            self._preexisting_flight_ids = set(self._latest_in_flight_ids)
            # Skip the edge prime while a prime ascent may already be running
            # (the reload coordinator primes at CHECKING, ~0.1 s before this
            # edge — the third sitting showed the pair restarting a just-started
            # ascent on 3/12 attempts). The retry tick re-primes after the
            # window if the ascent never actually happened. catch/prime_hold
            # suppresses the edge prime outright: during a toss the ball rides
            # the hand at the stroke bottom, and an auto-prime ascent here
            # would launch it (see _on_prime_hold).
            if (not self._prime_hold
                    and (time.perf_counter() - self._prime_dispatch_mono)
                    >= _PRIME_INFLIGHT_S):
                self._prime_hand()
        else:
            self._hand_primed = False
            self._hand_traj_armed_for_ball = None
            # One reload's catch-speed tuning value must never leak into the next.
            self._catch_vel_scale = float(hw.JB_OP_CATCH_VEL_SCALE_DEFAULT)
            # Clear the open-loop / exclusion / arm-count state so the NEXT reload starts
            # from a clean slate (a stale announcement latch would freeze the platform
            # open-loop before the next throw is even announced).
            self._announcement_seen = False
            self._announced_landing_time = None
            self._pretilt_cmd = None
            self._preexisting_flight_ids = set()
            self._arm_dispatch_ball_id = None
            self._arm_dispatch_count = 0
            # Per-ball feedback-correlation state too (audit): a post-disarm straggler
            # from a still-alive track must not correlate against the finished reload.
            self._last_submitted_ball_id = None
            self._last_arrival_time = 0.0
            self._last_landing_position = np.zeros(3)

    def _on_prime_dispatched(self, msg: Bool):
        """catch/prime_dispatched: the reload coordinator dispatched its own hand
        prime (ACTION_PRIME_HAND). Stamp the anti-stutter window so this node's
        edge prime / retry tick never restart that ascent — the two nodes cannot
        see each other's service calls, and the Teensy trajectory queue is
        last-writer-wins."""
        if bool(msg.data):
            self._prime_dispatch_mono = time.perf_counter()

    def _on_prime_hold(self, msg: Bool):
        """catch/prime_hold: the toss coordinator's prime-suppression gate,
        published True at PREPARE entry (BEFORE catch/armed rises) and False at
        terminal. While True, this node dispatches NO hand prime — neither the
        armed-edge prime nor the retry-tick re-prime: a toss holds the ball at
        the stroke bottom through the throw, so an auto-prime ascent would carry
        the ball-laden hand up mid-toss AND clear an armed throw stroke on the
        Teensy's last-writer-wins queue. The catch ARM (kind-1) and every other
        behaviour are untouched — hand-stroke catch timing stays tracker-driven.
        Stale True fails SAFE (no auto-prime; the reload action primes
        proactively itself), so the flag is never reset locally."""
        hold = bool(msg.data)
        if hold != self._prime_hold:
            self.get_logger().info(
                "catch/prime_hold raised — auto-prime suppressed" if hold
                else "catch/prime_hold released — auto-prime re-enabled")
        self._prime_hold = hold

    def _on_pretilt_hold(self, msg: Bool):
        """catch/pretilt_hold: the Tier-8b toss coordinator's platform
        pre-tilt-suppression gate, published True at PREPARE entry (with
        prime_hold, BEFORE catch/armed rises) and False at terminal. While True,
        _on_throw_announcement still LATCHES the announcement (open-loop freeze +
        hand-arm window keep working) but publishes NO platform pre-tilt and
        caches _pretilt_cmd = None — the toss coordinator owns the platform reach
        (the ONE deferred A->B target at release). The catch ARM (kind-1) and
        every other behaviour are untouched. Stale True fails DEGRADED-BUT-SAFE:
        a reload announcement loses its platform pre-tilt (the platform simply
        holds; the hand-arm stays tracker-driven) — zero hazard, so the flag is
        never reset locally (the publisher owns it)."""
        hold = bool(msg.data)
        if hold != self._pretilt_hold:
            self.get_logger().info(
                "catch/pretilt_hold raised — platform pre-tilt suppressed "
                "(toss owns the deferred reach)" if hold
                else "catch/pretilt_hold released — platform pre-tilt re-enabled")
        self._pretilt_hold = hold

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
        receive tilt during the ~3 s countdown. Under JB_OP_RELOAD_PLATFORM_OPEN_LOOP
        (default) the platform then HOLDS this pose; with the flag off, the reactive
        path supersedes it mid-flight. Deliberately does NOT touch the per-ball
        correlation state
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
        if self._pretilt_hold:
            # Tier-8b toss: LATCH the announcement for the hand-arm window +
            # open-loop freeze, but publish NO platform target and cache NONE —
            # the toss coordinator owns the platform reach (it publishes the ONE
            # deferred A->B target at release). Caching _pretilt_cmd would let
            # _republish_pretilt re-assert a pre-release B-reach; the stock
            # pre-tilt's arrival clamps to ~now+1s while the toss announces >=1s
            # before release, so an un-suppressed pre-tilt COMPLETES the A->B
            # translate (and the un-tilt to the receive tilt) BEFORE the ball is
            # released — aim destroyed, moving platform under a seated ball
            # mid-windup. hand-arm timing stays tracker-driven (unaffected).
            self._announcement_seen = True
            self._announced_landing_time = landing_time
            self._pretilt_cmd = None
            self.get_logger().info(
                "catch/pretilt_hold raised — announcement latched for hand-arm; "
                "platform pre-tilt suppressed (toss owns the deferred reach)")
            return
        cmd = self._coordinator.predicted_catch_command(
            landing_pos, landing_vel, landing_time)
        if cmd is None:
            return
        # Latch the open-loop platform state: from here (this armed reload's throw is
        # announced) the platform holds this pre-tilt pose and IGNORES the per-ball
        # reactive refinements in _on_balls when JB_OP_RELOAD_PLATFORM_OPEN_LOOP is set.
        # The announced landing gates the hand-arm window; the cached cmd feeds the
        # open-loop pre-tilt refresh.
        self._announcement_seen = True
        self._announced_landing_time = landing_time
        self._pretilt_cmd = cmd
        landing_perf = cmd.landing_time + self._ros_to_perf_offset
        arrival_perf = self._pretilt_arrival_perf(cmd)
        self._publish_dynamic_target(cmd, arrival_perf)
        self.get_logger().info(
            f"pre-tilt target published from announcement (landing in "
            f"{landing_time - self.get_clock().now().nanoseconds * 1e-9:.2f} s, "
            f"arrival {landing_perf - arrival_perf:.2f} s early)")

    def _pretilt_arrival_perf(self, cmd) -> float:
        """The pre-tilt target's perf-clock arrival. Arrive EARLY, not just-in-time: an
        arrival equal to the landing time makes trajectory_node span the whole
        announce→land window with a single min-jerk reach completing AT contact (third
        sitting: tilt still >1° off until 0.24–0.49 s before landing). Aim for landing −
        _PRETILT_EARLY_S so the platform is seated well before the ball arrives; the
        max() keeps a feasible profiled traverse on a late/short announcement, and the
        min() guarantees arrival is never scheduled after landing."""
        landing_perf = cmd.landing_time + self._ros_to_perf_offset
        return min(landing_perf,
                   max(landing_perf - _PRETILT_EARLY_S,
                       time.perf_counter() + _PRETILT_MIN_LEAD_S))

    def _republish_pretilt(self):
        """Open-loop safety net: re-assert the cached announcement pre-tilt pose (same
        pose, recomputed arrival) each balls tick until the ball has landed, so a single
        dropped pre-tilt still seats the cup. The pose is byte-identical each republish →
        zero net platform motion: pre-freeze, trajectory_node replans a reach to the SAME
        pose from the (already on-pose) live state; inside the reach-freeze window the
        republishes are FROZENed. The pre-tilt never stamps _last_submitted_ball_id, so
        there is no blacklist / feedback-correlation churn."""
        cmd = self._pretilt_cmd
        if cmd is None:
            return
        landing_perf = cmd.landing_time + self._ros_to_perf_offset
        if landing_perf <= time.perf_counter():
            return  # ball has landed — the pre-tilt is moot
        self._publish_dynamic_target(cmd, self._pretilt_arrival_perf(cmd))

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
        # The toss coordinator owns the hand from PREPARE to terminal
        # (catch/prime_hold True): no auto-prime may be dispatched while the
        # ball rides the stroke bottom awaiting the throw (see _on_prime_hold).
        if self._prime_hold:
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
        # Track the ids in flight this tick so the catch-armed rising edge can snapshot
        # them — a leftover track from a PRIOR attempt is then excluded from this
        # reload's candidates (see _on_catch_armed / update(exclude_ids=...)).
        self._latest_in_flight_ids = {
            b.id for b in balls if b.status == BallStatus.IN_FLIGHT}

        # Run coordinator policy (a prior-attempt leftover track is excluded)
        cmd = self._coordinator.update(
            balls, current_time, exclude_ids=self._preexisting_flight_ids)
        if cmd is None:
            return

        # A catch sequence is live: stamp the quiet window that suppresses the
        # prime-retry timer. STAMPED UNCONDITIONALLY (even open-loop) — it guards the
        # live kind-1 catch stroke against the 0.5 s prime-retry kind-3 on the Teensy's
        # last-writer-wins queue. NO PRIME is ever dispatched from this path — a kind-3
        # smooth-move here clears the Platform Teensy's armed catch stroke (3/6 strokes
        # lost to that race, 2026-07-23). Priming belongs to the catch/armed rising
        # edge + the off-path retry timer.
        self._last_cmd_mono = time.perf_counter()

        # Open-loop platform: once OUR throw is announced (armed reload/toss), hold
        # the pre-tilt pose and IGNORE this reactive per-ball refinement — a bad
        # ball prediction must never move the platform mid-reload (2026-07-24: a
        # corrupt track's sweep got ONE 78 mm target accepted at land−0.67 s,
        # dragging the platform 83.7 mm in the last 0.8 s and costing the catch).
        # Only the PLATFORM reach is frozen; the hand-arm below stays reactive (its
        # timing is tracker-driven). TWO triggers force this branch:
        #   - JB_OP_RELOAD_PLATFORM_OPEN_LOOP — the reload open-loop config default;
        #   - _pretilt_hold — a held Tier-8b toss. This forces the open-loop branch
        #     INDEPENDENT of the reload flag (self-contained, NOT co-dependent on
        #     it): with the flag off, a held toss's reactive per-ball path would
        #     otherwise publish tracker-derived catch/dynamic_target during flight,
        #     competing with the toss coordinator's deferred A->B reach. Under
        #     pretilt_hold _pretilt_cmd is None (see _on_throw_announcement), so
        #     _republish_pretilt no-ops (cmd None) — the platform simply holds while
        #     the toss coordinator owns the one deferred reach.
        open_loop = ((self._pretilt_hold or hw.JB_OP_RELOAD_PLATFORM_OPEN_LOOP)
                     and self._catch_armed and self._announcement_seen)
        if open_loop:
            # Re-assert the pre-tilt pose (same pose, recomputed arrival) so a dropped
            # pre-tilt still seats the cup; the reactive re-anchor + feasibility
            # blacklist stay dormant (never stamp _last_submitted_ball_id, so
            # _on_target_feedback early-returns and nothing is rejected).
            self._republish_pretilt()
        else:
            # Convert landing_time from ROS2 clock → perf_counter clock
            arrival_time_perf = cmd.landing_time + self._ros_to_perf_offset
            self._publish_dynamic_target(cmd, arrival_time_perf)
            self._last_submitted_ball_id = cmd.ball_id
            self._last_arrival_time = arrival_time_perf
            self._last_landing_position = cmd.target_pos.copy()

        # Arm hand catch trajectory (once per ball) — ONLY during a reload (catch armed),
        # so a stray tracked ball never arms the reactive catch stroke on the Teensy. The
        # reactive-fire TIMING (event_delay / event_vel) is computed here as before and
        # STAYS live under open-loop (only the platform reach is frozen). An arrival-
        # window guard rejects a corrupt track whose landing prediction is far off the
        # announced landing (it must not arm the one-shot stroke off garbage timing and
        # block the real ball's arm). The event velocity carries the operator's
        # catch/vel_scale knob (re-clamped to Teensy bounds). The one-shot latch is set
        # ONLY when the arm was actually dispatched — a service-not-ready early return
        # must be retried next tick, not silently latched as armed (audit of the
        # 2026-07-23 re-test).
        if (self._catch_armed and cmd.arm_hand
                and self._hand_traj_armed_for_ball != cmd.ball_id
                and self._arm_landing_window_ok(cmd)):
            # New ball → reset the per-ball re-dispatch counter (bounds the re-arm
            # churn on the lying ERR_TIMEOUT epidemic; see _MAX_ARM_DISPATCHES).
            if self._arm_dispatch_ball_id != cmd.ball_id:
                self._arm_dispatch_ball_id = cmd.ball_id
                self._arm_dispatch_count = 0
            event_delay = cmd.landing_time - current_time
            if event_delay >= _MIN_EVENT_DELAY_S:
                event_vel = max(0.3, min(7.0,
                                         cmd.event_vel_mps * self._catch_vel_scale))
                if self._arm_hand_catch(event_delay, event_vel):
                    self._hand_traj_armed_for_ball = cmd.ball_id
                    self._arm_dispatch_count += 1
            elif self._min_delay_logged_for_ball != cmd.ball_id:
                # Previously a SILENT drop: the delay only shrinks in flight, so a
                # ball first seen < 0.3 s out never arms — say so, once per ball.
                self._min_delay_logged_for_ball = cmd.ball_id
                self.get_logger().warning(
                    f"Ball {cmd.ball_id}: event_delay {event_delay:.2f} s below the "
                    f"{_MIN_EVENT_DELAY_S} s floor — hand catch NOT armed")

    def _arm_landing_window_ok(self, cmd) -> bool:
        """Gate the hand-arm on the ball's predicted landing being near the announced
        landing. Before any announcement (manual/bench throw) the guard is inert. A
        corrupt split-track whose landing prediction is far off is rejected so it cannot
        arm the one-shot stroke off garbage timing and block the real ball's arm."""
        if self._announced_landing_time is None:
            return True
        return abs(cmd.landing_time - self._announced_landing_time) <= _ARM_LANDING_WINDOW_S

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
        """Callback when hand trajectory arm completes.

        The HAND_TRAJ_CMD ack is unreliable (~40-60% ERR_TIMEOUT per call, 2026-07-23
        epidemic) AND lies — frames were observed transmitted after a failed ack. A
        failed ack re-opens the one-shot latch so the next balls tick re-arms, but ONLY
        up to _MAX_ARM_DISPATCHES per ball: the kind-1 stroke's catch instant is an
        ABSOLUTE wall_time invariant across retries, so a lying-ack arm still physically
        armed and further repacks just churn the Teensy's last-writer-wins queue. After
        the cap we KEEP the latch (assume armed). The expected-epidemic failure is logged
        at DEBUG, not WARN — a working reload was reading as 30/51 failure spam."""
        try:
            result = future.result()
            if result.success:
                self.get_logger().info("Hand catch trajectory armed on Teensy")
            else:
                # Re-open the latch for a retry only within the per-ball cap.
                if self._arm_dispatch_count < _MAX_ARM_DISPATCHES:
                    self._hand_traj_armed_for_ball = None
                self.get_logger().debug(
                    f"Hand catch arm ack failed "
                    f"({self._arm_dispatch_count}/{_MAX_ARM_DISPATCHES}): "
                    f"{result.message} — ack unreliable, proceeding")
        except Exception as e:
            # A genuine service error (not the ack epidemic) — retry within the cap.
            if self._arm_dispatch_count < _MAX_ARM_DISPATCHES:
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
