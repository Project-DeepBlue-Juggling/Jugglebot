"""ROS2 node: the BB→Jugglebot reload action (MVP goal 4).

A thin wrapper around the pure-Python :class:`reload_sequencer.ReloadSequencer` FSM.
The coordinator **orchestrates only** — it never actuates the robot directly:

  - platform motion is planned by ``trajectory_node`` (CATCH mode consumes
    ``catch/dynamic_target`` → ``planner.build_catch``, the existing path);
  - the hand is armed by ``catch_coordinator_node`` (existing behaviour, unchanged);
  - the throw goes through ``ball_butler_node`` (``bb/throw_at_target`` with the
    point-target extension), which enforces BB's own limits + loud rejections.

So this node subscribes to the state the FSM reasons about (BB heartbeat, mocap
freshness, trajectory streaming, control mode, tracked-ball status, target feedback),
calls two BB services on the FSM's behalf (``bb/reload``, ``bb/throw_at_target``), and
publishes phase feedback / the outcome on the ``Reload`` action. The operator sets
CATCH mode; the action does NOT switch modes (leaving CATCH mid-sequence is the
documented abort).

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

from std_msgs.msg import String
from std_srvs.srv import Trigger
from jugglebot_interfaces.msg import (
    BallButlerHeartbeat,
    BallStateArray,
    RigidBodyPoses,
    TargetFeedback,
    ThrowAnnouncement,
    TrajectoryStatus,
)
from jugglebot_interfaces.srv import BallButlerThrow
from jugglebot_interfaces.action import Reload
from geometry_msgs.msg import Point

import jugglebot.hardware_config as hw
from jugglebot.reload_sequencer import (
    ACTION_CALL_RELOAD,
    ACTION_SEND_THROW,
    ReloadObservations,
    ReloadSequencer,
    compute_catch_point_mm,
)

# BallStatus enum (BallState.msg): 2 = CAUGHT.
_BALL_STATUS_CAUGHT = 2

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
        self._catch_point_mm = compute_catch_point_mm(
            hw.GEOM_INITIAL_HEIGHT_MM, hw.JB_OP_DEFAULT_ACTIVE_Z_MM)

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
        with self._lock:
            seq = self._active_seq
        if seq is not None:
            seq.note_announcement(time.perf_counter())

    def _on_target_feedback(self, msg):
        if msg.source != 'catch':
            return
        with self._lock:
            seq = self._active_seq
        if seq is not None:
            seq.note_catch_feasibility(bool(msg.accepted), str(msg.code))

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
            for b in balls:
                if b.status == _BALL_STATUS_CAUGHT and (
                        not b.destination or b.destination == self._robot_name):
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
        """Horizontal miss distance of the caught ball from the world-frame catch point."""
        cx, cy = self._catch_point_mm[0], self._catch_point_mm[1]
        dx = float(ball.position.x) - cx
        dy = float(ball.position.y) - cy
        return float(np.hypot(dx, dy))

    # ── Action callbacks ───────────────────────────────────────────────────────

    def _goal_callback(self, goal_request):
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

        result = Reload.Result()
        try:
            t_start = time.perf_counter()
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
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
                    result.success = False
                    result.outcome = 'ABORTED_TIMEOUT'
                    result.catch_error_mm = float('nan')
                    goal_handle.abort()
                    return result
                time.sleep(_TICK_S)
            # rclpy shutting down.
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
