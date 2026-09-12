"""skill_node — the skill-stack orchestrator shell (R2, Unit D2, plan § 2.4).

Ties the pure-Python ``motion.skills`` stack (schedule / segments / executor —
no ROS imports, ``motion/`` rule) to the ROS graph: tracks ``/balls`` landings
and the hand's live possession evidence, ticks a
:class:`~jugglebot.motion.skills.executor.SkillExecutor` at 40 Hz, and installs
each dispatched skill through a synchronous ``trajectory/install_segment``
call. ALL PLANNING happens inside ``trajectory_node`` (see that service's
header for why: only it holds the live commanded state); this node's whole job
is schedule bookkeeping and dispatch — a thin wrapper over a pure-Python
policy, exactly as the other ROS nodes in this package are.

``skills/start_columns`` compiles the columns pattern (plan § 1.2) from ROS
parameters and starts an attempt; ``skills/stop`` ends the current attempt —
every segment is rest-terminal (``segments``' invariant), so ending an attempt
needs no motion command of its own: whatever is streaming already ends at rest.
"""

from __future__ import annotations

import time

import numpy as np

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from jugglebot_interfaces.msg import BallStateArray, HandTelemetryMessage
from jugglebot_interfaces.srv import InstallSegment

from jugglebot import ball_possession
from jugglebot.motion.skills.executor import InstallResult, Landing, SkillExecutor
from jugglebot.motion.skills.schedule import Pattern, compile_columns
from jugglebot.motion.skills.segments import CATCH, REST, THROW
from jugglebot.motion.skills.sites import columns_sites

# ``sk_seg`` kind string -> ``InstallSegment.Request`` wire constant. Built from
# the module's own KIND_* names below (not restated as bare ints), same
# discipline as trajectory_node's `_SEGMENT_KINDS`.
_WIRE_KIND = {THROW: InstallSegment.Request.KIND_THROW,
             CATCH: InstallSegment.Request.KIND_CATCH,
             REST: InstallSegment.Request.KIND_REST}

#: How long a synchronous `trajectory/install_segment` call waits for its
#: answer. Mirrors `reload_coordinator_node._call_plan_cycle`'s flat bound: the
#: service itself carries its own < 50 ms core / plan_wall_ms budget
#: (InstallSegment.srv), so this is a "the service died" backstop, not a
#: nominal-latency budget.
_SERVICE_WAIT_S = 2.0

#: SkillExecutor.tick cadence. The plan's dispatch-lead arithmetic
#: (`schedule._MIN_WINDOW_S`, `executor.LEAD_KNOTS`) is quantised to the 40 Hz
#: knot grid, so ticking at the same rate is the only choice that cannot itself
#: become the reason a dispatch lands late.
_TICK_HZ = 40.0

#: `skills/start_columns` parameter defaults — the owner's R2 operating point
#: (brief_common.md § 0, 2026-09-12): apex 0.9 m, separation 100 mm, dwell
#: 0.30 s.
_DEFAULT_APEX_M = 0.9
_DEFAULT_SEPARATION_MM = 100.0
_DEFAULT_DWELL_S = 0.30
_DEFAULT_N_THROWS = 4

#: The columns schedule's first skill installs this long after `start_columns`
#: is called — one second of margin for the operator's own dispatch latency,
#: not a physical constant of the pattern.
_START_LEAD_S = 1.0


class SkillNode(Node):
    """Schedule-driven skill dispatch shell (plan § 2.4)."""

    def __init__(self):
        super().__init__('skill_node')

        self._balls = {}  # ball_id (int) -> Landing, the tracker `SkillExecutor` reads
        # The hand's live possession evidence (ball_possession.EVIDENCE_*),
        # unused at R2 beyond logging — see the module docstring and
        # brief_D2_ros_shell.md § C.
        self._possession_evidence = ball_possession.EVIDENCE_UNKNOWN
        self._executor = None  # a live SkillExecutor, or None between attempts

        self.declare_parameter('apex_m', _DEFAULT_APEX_M)
        self.declare_parameter('separation_mm', _DEFAULT_SEPARATION_MM)
        self.declare_parameter('dwell_s', _DEFAULT_DWELL_S)
        self.declare_parameter('n_throws', _DEFAULT_N_THROWS)

        self._install_cli = self.create_client(
            InstallSegment, 'trajectory/install_segment')

        self.create_subscription(BallStateArray, 'balls', self._on_balls, 10)
        self.create_subscription(
            HandTelemetryMessage, 'hand_telemetry', self._on_hand_telemetry, 10)

        self.create_service(Trigger, 'skills/start_columns',
                            self._svc_start_columns)
        self.create_service(Trigger, 'skills/stop', self._svc_stop)

        self.create_timer(1.0 / _TICK_HZ, self._on_tick)

        self.get_logger().info('skill_node ready')

    # ── tracking ──────────────────────────────────────────────────────────

    def _on_balls(self, msg):
        """Cache ``/balls`` landings, keyed by ball id, for the executor's
        tracker callable. Mirrors `catch_coordinator_node._msg_to_ball`'s
        field reads."""
        for b in msg.balls:
            t_land = float(b.time_at_land.sec) + float(b.time_at_land.nanosec) * 1e-9
            self._balls[int(b.id)] = Landing(
                pos_mm=np.array([b.landing_position.x, b.landing_position.y,
                                 b.landing_position.z], dtype=float),
                vel_mm_s=np.array([b.landing_velocity.x, b.landing_velocity.y,
                                   b.landing_velocity.z], dtype=float),
                t_land_abs_s=t_land)

    def _tracker(self, ball_id: int):
        return self._balls.get(int(ball_id))

    def _on_hand_telemetry(self, msg):
        """Track the hand's live possession evidence — the same tri-state
        `reload_coordinator_node._on_hand_telemetry` feeds into
        `ball_possession.HandBallSensorSource`. Unused at R2 beyond logging: no
        skill decision reads it yet (the learner and the retention verdict
        arrive later in the plan), so this stores the LIVE bit directly rather
        than standing up the source's full arrival/retention state machine for
        a value nothing consumes."""
        valid = bool(getattr(msg, 'ball_held_valid', False))
        raw = getattr(msg, 'ball_held_raw', None)
        if not valid or raw is None:
            self._possession_evidence = ball_possession.EVIDENCE_UNKNOWN
        else:
            self._possession_evidence = (
                ball_possession.EVIDENCE_SEATED if bool(raw)
                else ball_possession.EVIDENCE_EMPTY)

    # ── the installer callable SkillExecutor dispatches through ────────────

    def _installer(self, kind, terminal, t_abs_s, *, ball_id) -> InstallResult:
        req = InstallSegment.Request()
        req.kind = _WIRE_KIND[kind]
        req.ball_id = int(ball_id)
        if kind == THROW:
            req.t_event_s = float(terminal.t_release_s)
            req.site_mm = [float(v) for v in terminal.site_mm]
            req.target_mm = [float(v) for v in terminal.target_mm]
            req.flight_s = float(terminal.flight_s)
        elif kind == CATCH:
            req.t_event_s = float(terminal.t_land_s)
            req.site_mm = [float(v) for v in terminal.landing_mm]
            req.landing_vel_mm_s = [float(v) for v in terminal.landing_vel_mm_s]
            req.rest_site_mm = [float(v) for v in terminal.rest_site_mm]
            tt = terminal.then_throw
            if tt is not None:
                req.t_release_s = float(tt.t_release_s)
                req.release_site_mm = [float(v) for v in tt.site_mm]
                req.target_mm = [float(v) for v in tt.target_mm]
                req.flight_s = float(tt.flight_s)
        else:
            req.t_event_s = float(terminal.t_rest_s)
            req.rest_site_mm = [float(v) for v in terminal.rest_site_mm]

        if not self._install_cli.wait_for_service(timeout_sec=_SERVICE_WAIT_S):
            self.get_logger().error('trajectory/install_segment unavailable')
            return InstallResult(False, 'SERVICE_UNAVAILABLE',
                                 'trajectory/install_segment unavailable', 0.0)
        resp = self._wait_future(self._install_cli.call_async(req))
        if resp is None:
            return InstallResult(
                False, 'SERVICE_TIMEOUT',
                'trajectory/install_segment did not answer in %.1f s'
                % (_SERVICE_WAIT_S,), 0.0)
        return InstallResult(bool(resp.accepted), str(resp.code),
                             str(resp.message),
                             float(resp.plan_wall_ms) / 1e3,
                             splice_k=int(resp.splice_k),
                             t0_s=float(resp.t0_mono),
                             event_t_s=float(resp.t_event_mono),
                             seeded_post_release=bool(resp.seeded_post_release))

    def _wait_future(self, future, timeout_s: float = _SERVICE_WAIT_S):
        """Poll a service future to completion. Mirrors
        `reload_coordinator_node._wait_future` (the MultiThreadedExecutor
        services it on another thread)."""
        deadline = time.perf_counter() + timeout_s
        while not future.done() and time.perf_counter() < deadline and rclpy.ok():
            time.sleep(0.005)
        if not future.done():
            return None
        try:
            return future.result()
        except Exception as exc:                                  # noqa: BLE001
            self.get_logger().error('install_segment call raised: %s' % (exc,))
            return None

    # ── the tick ────────────────────────────────────────────────────────

    def _on_tick(self):
        if self._executor is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        for line in self._executor.tick(now):
            self.get_logger().info(line)
        if self._executor.attempt_ended:
            self._executor = None

    # ── attempt lifecycle ──────────────────────────────────────────────

    def _svc_start_columns(self, request, response):
        if self._executor is not None:
            response.success = False
            response.message = ('an attempt is already running — call '
                                'skills/stop first')
            return response
        apex_m = float(self.get_parameter('apex_m').value)
        separation_mm = float(self.get_parameter('separation_mm').value)
        dwell_s = float(self.get_parameter('dwell_s').value)
        n_throws = int(self.get_parameter('n_throws').value)
        t0 = self.get_clock().now().nanoseconds / 1e9 + _START_LEAD_S
        try:
            sites = columns_sites(separation_mm)
            pattern = Pattern(sites=sites, apex_m=apex_m, dwell_s=dwell_s,
                             n_throws=n_throws)
            schedule = compile_columns(pattern, t0)
        except ValueError as exc:
            response.success = False
            response.message = 'columns schedule refused: %s' % (exc,)
            self.get_logger().error(response.message)
            return response
        self._executor = SkillExecutor(schedule, self._installer,
                                       tracker=self._tracker)
        response.success = True
        response.message = ('columns schedule compiled: %d skills, %d throws, '
                            't0=%.3f' % (len(schedule.skills), n_throws, t0))
        self.get_logger().info(response.message)
        return response

    def _svc_stop(self, request, response):
        """End the current attempt. No motion command: every segment is
        rest-terminal, so whatever is streaming already ends at rest."""
        if self._executor is not None:
            self._executor.attempt_ended = True
            self._executor.end_code = 'STOPPED'
            self._executor = None
            response.message = 'attempt stopped — the rest tail is already streaming'
        else:
            response.message = 'no attempt was running'
        response.success = True
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SkillNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
