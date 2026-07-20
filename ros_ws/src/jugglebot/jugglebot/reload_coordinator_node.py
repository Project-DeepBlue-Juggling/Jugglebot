"""ROS2 node: the BB→Jugglebot reload action (MVP goal 4).

A thin wrapper around the pure-Python :class:`reload_sequencer.ReloadSequencer` FSM.
The RELOAD action OWNS the platform + hand for its duration (reload-action-catch-latch
plan) — but it never plans motion itself; it drives the existing services:

  - platform motion is planned by ``trajectory_node``: the reactive catch path
    (``catch/dynamic_target`` → ``planner.build_catch``) actuates the platform while the
    ``trajectory/arm_catch`` catch-armed latch is RAISED, and ``trajectory/go_home``
    re-centers on terminal — both run in TRAJECTORY, the mode RELOAD runs within;
  - the hand is primed to top proactively by this node (``smooth_move_hand`` on
    PREPARE) and retracted on abort; ``catch_coordinator_node`` still fires the reactive
    catch stroke, gated on the catch-armed latch;
  - the throw goes through ``ball_butler_node`` (``bb/throw_at_target`` with the
    point-target extension), which enforces BB's own limits + loud rejections.

So this node subscribes to the state the FSM reasons about (BB heartbeat, mocap
freshness, trajectory streaming, control mode, tracked-ball status, target feedback),
and executes the actions the FSM asks for: ``bb/reload``, ``bb/throw_at_target``,
PREPARE (prime hand + raise the latch), RECENTER (lower latch + go_home), SAFE_ABORT
(retract hand + lower latch + go_home). It runs within the operator's active mode
(TRAJECTORY) and does NOT switch modes; leaving that mode mid-sequence is the
documented abort.

Exposed as ``jugglebot/reload`` (Reload.action): phase feedback, a structured outcome
result, and cancellation.
"""

from __future__ import annotations

import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

import numpy as np

from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from jugglebot_interfaces.msg import (
    BallButlerHeartbeat,
    BallStateArray,
    RigidBodyPoses,
    TargetFeedback,
    ThrowAnnouncement,
    TrajectoryStatus,
)
from jugglebot_interfaces.srv import BallButlerThrow, SetFloat
from jugglebot_interfaces.action import Reload
from geometry_msgs.msg import Point

import jugglebot.hardware_config as hw
from jugglebot.reload_sequencer import (
    ACTION_CALL_RELOAD,
    ACTION_PREPARE_CATCH,
    ACTION_RECENTER,
    ACTION_SAFE_ABORT,
    ACTION_SEND_THROW,
    ReloadObservations,
    ReloadSequencer,
    compute_catch_point_mm,
)

# BallStatus enum (BallState.msg): 1 = IN_FLIGHT, 2 = CAUGHT.
_BALL_STATUS_IN_FLIGHT = 1
_BALL_STATUS_CAUGHT = 2

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

# Reject-fast service-call bounds.
_SERVICE_WAIT_S = 2.0
# Sequence loop tick (the FSM is time-driven; this bounds latency, not correctness).
_TICK_S = 0.05
# A hard ceiling on a single reload attempt so a wedged sequence always terminates.
_MAX_SEQUENCE_S = 30.0


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
        self._mocap_mono = 0.0
        self._balls = []
        self._balls_mono = 0.0
        # The active sequencer (announcements + catch feedback route to it while a
        # reload is running); None between runs.
        self._active_seq = None
        # The tracker-assigned id of OUR announced ball, latched once it appears in flight
        # this sequence; only that id's CAUGHT confirms (a stray caught ball never does).
        self._announced_ball_id = None

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
            ThrowAnnouncement, 'throw_announcements', self._on_announcement, 10)
        self.create_subscription(
            TargetFeedback, 'trajectory/target_feedback', self._on_target_feedback, 10)

        # ── Service clients ──
        self._reload_cli = self.create_client(Trigger, 'bb/reload')
        self._throw_cli = self.create_client(BallButlerThrow, 'bb/throw_at_target')
        # Platform + hand ownership for the reload's duration:
        #   trajectory/arm_catch — raise/lower the catch-armed latch (the reactive-catch
        #     TRIGGER: while raised, catch/dynamic_target actuates the platform);
        #   smooth_move_hand     — proactive prime to top (PREPARE) / retract to bottom
        #     (SAFE_ABORT);
        #   trajectory/go_home   — re-center to level neutral on terminal.
        self._arm_catch_cli = self.create_client(SetBool, 'trajectory/arm_catch')
        self._smooth_move_hand_cli = self.create_client(SetFloat, 'smooth_move_hand')
        self._go_home_cli = self.create_client(Trigger, 'trajectory/go_home')

        # ── Publisher: catch-armed state ──
        # catch_coordinator_node gates its hand prime/arm on this so it only actuates the
        # hand DURING a reload (latch raised), never on a stray tracked ball. Published
        # True on PREPARE, False on RECENTER/SAFE_ABORT — the same edges that drive the
        # trajectory/arm_catch service, so the two stay in lockstep.
        self._catch_armed_pub = self.create_publisher(Bool, 'catch/armed', 10)

        # ── Reload action server ──
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

        self.get_logger().info(
            f"Reload coordinator ready (jugglebot/reload); catch point "
            f"{self._catch_point_mm} mm (world).")

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

    def _on_mocap(self, msg):
        with self._lock:
            self._mocap_mono = time.perf_counter()

    def _on_balls(self, msg):
        with self._lock:
            self._balls = list(msg.balls)
            self._balls_mono = time.perf_counter()

    def _on_announcement(self, msg):
        # Only OUR ball's announcement (thrown by BB, aimed at us) advances the FSM.
        if msg.target_id and msg.target_id != self._robot_name:
            return
        now = time.perf_counter()
        landing_perf = self._announcement_landing_perf(msg, now)
        with self._lock:
            seq = self._active_seq
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
            announced_id = self._announced_ball_id
        ball_caught = False
        catch_error_mm = float('nan')
        if balls_fresh:
            # Correlate CAUGHT to the tracker-assigned id of OUR announced ball. The
            # tracker puts our ball IN_FLIGHT (destination == us) after correlating the
            # announcement; latch that id, then confirm ONLY that id's CAUGHT. This rejects
            # a stray caught ball (a different id — e.g. a leftover from a prior throw)
            # that would otherwise falsely confirm this reload's catch.
            if announced_id is None:
                for b in balls:
                    if int(b.status) == _BALL_STATUS_IN_FLIGHT and (
                            not b.destination or b.destination == self._robot_name):
                        announced_id = int(b.id)
                        break
                if announced_id is not None:
                    with self._lock:
                        self._announced_ball_id = announced_id
            if announced_id is not None:
                for b in balls:
                    if int(b.id) == announced_id and int(b.status) == _BALL_STATUS_CAUGHT:
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

    def _catch_error_from_ball(self, ball) -> float:
        """Horizontal miss distance of the caught ball from the world-frame catch point.

        This is the tracker's last KF position estimate at CAUGHT (an in-flight-derived
        estimate), NOT a settled rest position — the MVP evidence is the tracker-id
        correlation + this KF miss, with a hand-telemetry rest cross-check deferred."""
        cx, cy = self._catch_point_mm[0], self._catch_point_mm[1]
        dx = float(ball.position.x) - cx
        dy = float(ball.position.y) - cy
        return float(np.hypot(dx, dy))

    # ── Action callbacks ───────────────────────────────────────────────────────

    def _goal_callback(self, goal_request):
        # One reload at a time. rclpy runs accepted goals concurrently, so a second goal
        # accepted while the first is mid-flight would drive a SECOND bb/throw_at_target
        # (double-throw) and scramble the shared async-event routing (announcements /
        # target feedback all fan out to the single _active_seq). Reject it loudly.
        with self._lock:
            busy = self._active_seq is not None
        if busy:
            self.get_logger().warning(
                'Reload goal REJECTED — a reload is already in progress '
                '(one sequence at a time).')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute_reload(self, goal_handle):
        throw_delay = float(getattr(goal_handle.request, 'throw_delay_s', 0.0) or 0.0)
        seq = ReloadSequencer(
            catch_point_mm=self._catch_point_mm, throw_delay_s=throw_delay)
        seq.start(time.perf_counter())
        with self._lock:
            self._active_seq = seq
            self._announced_ball_id = None

        result = Reload.Result()
        try:
            t_start = time.perf_counter()
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._safe_on_early_exit(seq)
                    goal_handle.canceled()
                    result.success = False
                    result.outcome = 'ABORTED_CANCELLED'
                    result.catch_error_mm = float('nan')
                    return result
                now = time.perf_counter()
                decision = self._step_sequence(seq, now, goal_handle)
                if decision.done:
                    r = decision.result
                    result.success = bool(r.success)
                    result.outcome = str(r.outcome)
                    result.catch_error_mm = float(r.catch_error_mm)
                    (goal_handle.succeed if r.success else goal_handle.abort)()
                    return result
                if now - t_start > _MAX_SEQUENCE_S:
                    self._safe_on_early_exit(seq)
                    result.success = False
                    result.outcome = 'ABORTED_TIMEOUT'
                    result.catch_error_mm = float('nan')
                    goal_handle.abort()
                    return result
                time.sleep(_TICK_S)
            # rclpy shutting down.
            self._safe_on_early_exit(seq)
            result.success = False
            result.outcome = 'ABORTED_SHUTDOWN'
            result.catch_error_mm = float('nan')
            return result
        finally:
            with self._lock:
                self._active_seq = None

    def _step_sequence(self, seq, now, goal_handle=None):
        """One FSM tick: build observations, step, execute the requested action, publish
        phase feedback. Returns the decision (testable in isolation)."""
        obs = self._build_observations(now)
        decision = seq.step(now, obs)
        if decision.action == ACTION_CALL_RELOAD:
            self._call_reload()
        elif decision.action == ACTION_SEND_THROW:
            accepted, message = self._send_throw(seq)
            seq.note_throw_result(accepted, message)
        elif decision.action == ACTION_PREPARE_CATCH:
            self._prepare_catch()
        elif decision.action == ACTION_RECENTER:
            self._recenter()
        elif decision.action == ACTION_SAFE_ABORT:
            self._safe_abort()
        if goal_handle is not None and not decision.done:
            fb = Reload.Feedback()
            fb.phase = decision.phase
            goal_handle.publish_feedback(fb)
        return decision

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
        return bool(resp.success), str(resp.message)

    # ── Platform + hand ownership (executed on the FSM's behalf) ───────────────

    def _prepare_catch(self):
        """PREPARE (throw accepted): proactively prime the hand to the top of its stroke
        and raise the catch-armed latch. Priming ~throw_delay before the ball flies keeps
        the catch stroke from racing the ball with a high-jerk late prime; raising the
        latch lets the reactive catch path actuate the platform for the flight window."""
        self._smooth_move_hand(hw.JB_OP_HAND_CATCH_PRIME_REV)
        self._arm_catch(True)
        self._publish_catch_armed(True)

    def _recenter(self):
        """RECENTER (successful catch): lower the catch-armed latch and re-center to
        level neutral via go_home. The hand keeps the caught ball — no retract."""
        self._arm_catch(False)
        self._publish_catch_armed(False)
        self._go_home()

    def _safe_abort(self):
        """SAFE_ABORT (abort once prepared): retract the hand to the bottom of its stroke,
        lower the catch-armed latch, and re-center via go_home."""
        self._smooth_move_hand(hw.HOMING_HAND_ABS_POS_REV)
        self._arm_catch(False)
        self._publish_catch_armed(False)
        self._go_home()

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
