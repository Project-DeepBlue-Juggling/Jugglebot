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
parameters and starts an attempt; ``skills/start_self_toss`` (R3) is the
single-site self-toss — it pre-levels the platform (``_prelevel``, the same
gravity-level rest ``reload_coordinator_node._unified_prelevel`` brings the
session-start floor lift to) before compiling its schedule, and wires the R3
precondition ladder (``executor.Observations`` / ``precondition_refusals``)
through ``_observations``/``_ball_evidence``; ``skills/stop`` ends the
current attempt — every segment is rest-terminal (``segments``' invariant),
so ending an attempt needs no motion command of its own: whatever is
streaming already ends at rest, and a released ball's outcome keeps
finalising after the attempt ends. ``skills/check`` reports every current
ladder refusal plus the box/limits status in one call, for a dress-rehearsal
runsheet.
"""

from __future__ import annotations

import csv
import math
import os
import threading
import time
from types import SimpleNamespace
from typing import Dict, List, Tuple

import numpy as np

import rclpy
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from std_srvs.srv import Trigger
from jugglebot_interfaces.msg import (BallStateArray, HandTelemetryMessage,
                                      RigidBodyPoses, ThrowAnnouncement,
                                      TrajectoryStatus)
from jugglebot_interfaces.srv import GoToPose, InstallSegment

import jugglebot.hardware_config as hw
from jugglebot import ball_possession
from jugglebot.motion import blas_threads
from jugglebot.motion.skills import admissible as adm
from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import learner as lr
from jugglebot.motion.skills.executor import (InstallResult, Landing,
                                              Observations, SkillExecutor,
                                              precondition_refusals)
from jugglebot.motion.skills.hand_launch import HandLaunchMonitor
from jugglebot.motion.skills.memory import Memory, memory_path
from jugglebot.motion.skills.schedule import (Pattern, SelfTossPattern,
                                              compile_columns, compile_self_toss)
from jugglebot.motion.skills.segments import CATCH, REST, THROW
from jugglebot.motion.skills.sites import (CATCH_CUP_Z_MM, REST_CUP_Z_MM,
                                           Site, columns_sites)
from jugglebot.motion.tilt_map import find_repo_root
from jugglebot.motion.trajectory import ballistics_bc
from jugglebot.ball_possession import (
    FlightLatch, advance_flight_latches, flight_in_progress)

# BallStatus enum (BallState.msg): 0 = TO_BE_THROWN, 1 = IN_FLIGHT, 2 = CAUGHT.
# Restated (not imported from reload_coordinator_node — that module is ROS/FSM
# code and this one must stay independent of it) — see brief_common.md's
# "restate, don't import from the FSM node" instruction (R3-e2 item 6, same
# discipline applied here for item 3's correlation).
_BALL_STATUS_IN_FLIGHT = 1
# TrackingConfidence enum (BallState.msg): 1 = CONFIRMED (mocap-matched —
# physical airborne evidence; IN_FLIGHT status alone is time-based and proves
# nothing).
_BALL_TRACKING_CONFIRMED = 1

# The repo root, found the same way reload_coordinator_node._REPO_ROOT is
# (tilt_map.find_repo_root's MARKER walk, never a fixed __file__ depth — see
# that function's docstring for why a fixed walk breaks under colcon install).
# None for a genuinely detached deployment, where the admissible box / memory
# paths below cannot be resolved and skills/start_self_toss refuses.
_REPO_ROOT = find_repo_root(__file__)
_ADMISSIBLE_BOX_PATH = (os.path.join(_REPO_ROOT, 'config', 'generated',
                                     'admissible_box.yaml')
                       if _REPO_ROOT else None)

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

#: `_svc_stop`'s bound on waiting for `_tick_lock` (finding 12, R3 audit,
#: 2026-09-13): a tick mid-`_dispatch` can itself be blocked inside
#: `_wait_future` on the install client, up to `_SERVICE_WAIT_S`, and
#: `_prelevel`'s own `go_to_pose` round trip carries the same bound again if
#: a tick is what triggered it -- `2 * _SERVICE_WAIT_S` covers both, plus one
#: tick period so an ordinary (non-blocked) in-progress tick has time to
#: finish on its own cadence.
_STOP_LOCK_WAIT_S = 2.0 * _SERVICE_WAIT_S + 1.0 / _TICK_HZ

#: The R3 precondition ladder's freshness windows (item 6, plan carried R3
#: note / `executor.Observations`). Restated, not imported, from
#: `reload_coordinator_node` — that module is FSM/ROS code slated for deletion
#: at R4 (module docstring), and these are the SAME physical facts (a topic's
#: publish rate / the sensor's own noise), not a second definition of them.
#: Restates `reload_coordinator_node._MOCAP_STALE_S`.
_MOCAP_STALE_S = 0.5
#: Restates `reload_coordinator_node._TRAJ_STATUS_STALE_S` (the topic
#: itself is 5 Hz; also the freshness window for `trajectory/commanded_position`,
#: same as `_live_commanded_position` there).
_TRAJ_STATUS_STALE_S = 1.0
#: Restates `reload_coordinator_node._HAND_STATE_STALE_S`.
_HAND_STATE_STALE_S = 0.5

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

#: `skills/start_self_toss` parameter defaults (R3 owner decisions,
#: brief_common.md § "Owner decisions"): one site, P1 = (-50, 0) mm — the SAME
#: point `columns_sites(100.0)`'s P1 names, so a box swept for site pair
#: ('P1', 'P1') at that xy (`config/generated/admissible_box.yaml`) matches
#: without a second geometry definition.
_DEFAULT_SITE_X_MM = -50.0
_DEFAULT_SITE_Y_MM = 0.0
_DEFAULT_SITE_NAME = 'P1'
#: A fresh id starts a cold memory (plan § 2.2) — `memory_path`'s own contract.
_DEFAULT_PLANT_ID = 'jugglebot'


class SkillNode(Node):
    """Schedule-driven skill dispatch shell (plan § 2.4)."""

    def __init__(self, robot_name: str = _DEFAULT_PLANT_ID):
        super().__init__('skill_node')

        self._robot_name = robot_name
        self._balls = {}  # ball_id (int) -> Landing, the tracker `SkillExecutor` reads
        # The latest `/balls` message's raw ball records (id/status/destination/
        # tracking), for ball-identity correlation (`_advance_correlation`) —
        # `_balls` above is the LANDING projection `latch_announced_ball` cannot
        # use (it needs `status`/`destination`/`tracking`, not just a landing).
        self._raw_balls = []
        # SCHEDULE ball_id -> a tuple of `ball_possession.FlightLatch`, ONE
        # PER ANNOUNCED RELEASE (never one per ball — see `_maybe_announce`).
        # `/balls` ids are the TRACKER's own; the schedule/executor speak in
        # schedule ball ids, and this is the one translation. Advanced by
        # `_advance_correlation`, read by `_tracker` through
        # `ball_possession.flight_in_progress`.
        self._correlation = {}
        # SCHEDULE ball_id -> the absolute wall-clock instant of the last
        # release announced for it — `_maybe_announce`'s de-duplication key
        # for a CATCH re-send (see that method's docstring).
        self._announced_release_s = {}
        # The hand's live possession evidence (ball_possession.EVIDENCE_*) —
        # the R3 ladder's `ball_evidence` field (item 6) AND outcome capture's
        # `observer` read (`_ball_evidence`), one value, one writer
        # (`_on_hand_telemetry`).
        self._possession_evidence = ball_possession.EVIDENCE_UNKNOWN
        # Debounced twin for the ladder's SEATED precondition (see _on_hand_telemetry).
        self._possession_evidence_stable = ball_possession.EVIDENCE_UNKNOWN
        self._executor = None  # a live SkillExecutor, or None between attempts
        # The last ACCEPTED install's future event (a THROW's release, or a
        # CATCH-with-throw's carried release) on the schedule's own wall
        # clock -- 0.0 when the streaming plan carries none (a plain CATCH,
        # a REST). Updated on every ACCEPTED install (`_installer`), read
        # and cleared by `_maybe_hold_pending_event` (Unit A, R3 first
        # sitting, 2026-09-13, L1): an ended attempt does not stop a
        # streaming release just because it ended -- every segment is
        # rest-terminal, not event-free.
        self._pending_event_mono = 0.0
        # Guards `_pending_event_mono`'s check-and-clear only (never the hold
        # round trip): `_svc_stop` calls `_maybe_hold_pending_event` OUTSIDE
        # `_tick_lock` while `_on_tick` may be inside it on another thread.
        self._pending_lock = threading.Lock()
        # Guards `_correlation`'s read-modify-write. `_on_balls` (the
        # subscription group) and `_maybe_announce` (the tick, via
        # `_installer`) both rebind the latch queue and run on DIFFERENT
        # callback groups, so a concurrent announce could otherwise lose its
        # own latch — and a lost latch means the next flight is never
        # correlated at all. `_tracker` only reads the tuple and needs no lock.
        self._correlation_lock = threading.Lock()
        # The latest `trajectory/status`, for `skills/start_self_toss`'s
        # limits check (`_svc_start_self_toss`) and the R3 ladder's
        # `levelled`/`in_trajectory_mode` fields (`_observations`, item 6).
        self._traj_status = TrajectoryStatus()
        # Perf-clock arrival stamps for the R3 ladder's freshness checks (item
        # 6) — one stamp per cached message, same monotonic domain
        # (`time.perf_counter()`) `reload_coordinator_node` uses for the
        # identical purpose (that module's `_*_mono` caches).
        self._traj_status_mono = 0.0
        self._mocap_mono = 0.0
        self._hand_telemetry_mono = 0.0
        self._hand_pos_meas = 0.0
        self._hand_pos_cmd = 0.0
        # The hand's launch-speed ratio source for `catch_aim_source` =
        # `schedule_hand` (`hand_launch.HandLaunchMonitor`): fed every
        # `/hand_telemetry` sample, read once per catch by the executor. Held
        # for the node's life (not per attempt) — a stroke's samples arrive
        # before the catch that asks about them, and the monitor's own
        # history bound is what keeps it small.
        self._hand_launch = HandLaunchMonitor()
        # `trajectory/commanded_position` — the pre-level move's own xy/z
        # (item 7; `_prelevel`), restated from
        # `reload_coordinator_node._on_commanded_position` /
        # `_live_commanded_position`'s cache-plus-freshness shape.
        self._commanded_pos_mm = None
        self._commanded_pos_mono = 0.0

        self.declare_parameter('apex_m', _DEFAULT_APEX_M)
        self.declare_parameter('separation_mm', _DEFAULT_SEPARATION_MM)
        self.declare_parameter('dwell_s', _DEFAULT_DWELL_S)
        self.declare_parameter('n_throws', _DEFAULT_N_THROWS)
        self.declare_parameter('site_x_mm', _DEFAULT_SITE_X_MM)
        self.declare_parameter('site_y_mm', _DEFAULT_SITE_Y_MM)
        self.declare_parameter('plant_id', _DEFAULT_PLANT_ID)
        # Where a CATCH's aim comes from (owner decision 2026-09-15). The
        # LIVE default is `schedule` — open-loop from the throw state the
        # schedule COMMANDED — because at the 2026-09-15 sitting all 13
        # self-tosses ended NO_LANDING: mocap never produced a marker for the
        # flying ball, so a tracker-dependent aim never happened at all. See
        # `executor.AIM_SOURCES` for the three values; the executor's own
        # constructor default stays `tracker` so the sim gate and the R2/R3
        # tests keep exercising that path, which is why this is passed
        # EXPLICITLY at every construction below rather than left to default.
        self.declare_parameter('catch_aim_source', ex.AIM_SCHEDULE)
        # Owner 2026-09-16: the learner corrects FLIGHT only until the
        # planner's small-lateral-offset banking defect is fixed (see
        # SkillExecutor.lateral_authority_m). mm per axis; 0 = pinned to y_d.
        self.declare_parameter('learner_lateral_authority_mm', 0.0)

        # ── the install client + tick timer share ONE reentrant group ──────
        # Fixed 2026-09-13 (found by reading, never exercised live): `main`
        # used to run a plain single-threaded `rclpy.spin`, and `_installer`
        # blocks the tick callback in `_wait_future`'s poll loop waiting for
        # the SAME node's client response — a response only the executor can
        # ever deliver. Under a single-threaded spin nothing is left to
        # process that response while the tick callback is busy-waiting for
        # it: every install times out, forever. Mirrors
        # `reload_coordinator_node`'s `_hold_cbg` / `trajectory_node`'s
        # `_hold_cbg` narrow-reentrant pattern — here the WHOLE install path
        # (client + timer) moves together, because the tick callback and the
        # client response are the two halves of the one blocking call that
        # needed unblocking, not two independent throughput concerns.
        self._cbgroup = ReentrantCallbackGroup()
        # Guards against the tick RE-ENTERING itself: a `ReentrantCallbackGroup`
        # lets the executor run the SAME timer callback again on another
        # thread while the first invocation is still blocked inside
        # `_wait_future` — that would start a second dispatch on top of one
        # already in flight. `acquire(blocking=False)` in `_on_tick` makes a
        # still-busy tick a no-op tick rather than a second install.
        self._tick_lock = threading.Lock()

        self._install_cli = self.create_client(
            InstallSegment, 'trajectory/install_segment',
            callback_group=self._cbgroup)
        # Shares the SAME reentrant group as the install client (item 7,
        # `_prelevel`): `_svc_start_self_toss` runs on the node's default
        # (non-reentrant) service group and blocks in `_wait_future` waiting
        # for THIS client's response, exactly the shape `__init__`'s comment
        # above documents for the install client — a different group is what
        # lets the MultiThreadedExecutor process that response concurrently.
        self._go_to_pose_cli = self.create_client(
            GoToPose, 'trajectory/go_to_pose', callback_group=self._cbgroup)
        # Shares the SAME reentrant group too (Unit A, R3 first sitting,
        # 2026-09-13): `_maybe_hold_pending_event` blocks in `_wait_future`
        # for THIS client's response, called from `_on_tick` (already on the
        # group) and from `_svc_stop` (the default group) -- the response
        # still needs the MultiThreadedExecutor to process it concurrently.
        self._hold_cli = self.create_client(
            Trigger, 'trajectory/hold', callback_group=self._cbgroup)

        self._announce_pub = self.create_publisher(
            ThrowAnnouncement, 'throw_announcements', 10)

        # ── every subscription gets its OWN callback group (finding 4, R3
        # audit, 2026-09-13) ────────────────────────────────────────────────
        # All subscriptions used to share the node's default MutuallyExclusive
        # group. `_svc_start_self_toss` blocks in `_prelevel` (a `go_to_pose`
        # round trip whose wait can run past a second) on the SERVICE's own
        # default group, and rclpy Foxy's executor yields ready timers before
        # subscriptions -- so while `_prelevel` blocks, no subscription
        # callback runs, and the first `_on_tick` after it can read a
        # `trajectory/status` sample older than `_TRAJ_STATUS_STALE_S`,
        # aborting `ABORTED_MODE_CHANGED` before the opening REST even
        # dispatches. A dedicated non-default group per subscription (plus the
        # `MultiThreadedExecutor(num_threads=3)` in `main` below) lets them run
        # concurrently with a blocked service call instead of queuing behind
        # it.
        self._sub_cbgroup = MutuallyExclusiveCallbackGroup()
        self.create_subscription(BallStateArray, 'balls', self._on_balls, 10,
                                 callback_group=self._sub_cbgroup)
        self.create_subscription(
            HandTelemetryMessage, 'hand_telemetry', self._on_hand_telemetry, 10,
            callback_group=self._sub_cbgroup)
        self.create_subscription(
            TrajectoryStatus, 'trajectory/status', self._on_traj_status, 10,
            callback_group=self._sub_cbgroup)
        self.create_subscription(
            RigidBodyPoses, 'rigid_body_poses', self._on_mocap, 10,
            callback_group=self._sub_cbgroup)
        self.create_subscription(
            Point, 'trajectory/commanded_position',
            self._on_commanded_position, 10, callback_group=self._sub_cbgroup)

        self.create_service(Trigger, 'skills/start_columns',
                            self._svc_start_columns)
        self.create_service(Trigger, 'skills/start_self_toss',
                            self._svc_start_self_toss)
        self.create_service(Trigger, 'skills/stop', self._svc_stop)
        self.create_service(Trigger, 'skills/check', self._svc_check)

        self.create_timer(1.0 / _TICK_HZ, self._on_tick,
                          callback_group=self._cbgroup)

        # One INFO line at start-up (`blas threads: N (source)`, WARN if the
        # pool is uncapped) — the runsheet's row 13 greps for it, and the
        # launch file's `_planner_blas_env` cap is otherwise invisible here.
        self._blas_threads, self._blas_source = blas_threads.check_blas_threads(
            self.get_logger(), 'skill_node')
        self.get_logger().info('skill_node ready')

    # ── tracking ──────────────────────────────────────────────────────────

    def _on_balls(self, msg):
        """Cache ``/balls`` landings, keyed by ball id, for the executor's
        tracker callable. Mirrors `catch_coordinator_node._msg_to_ball`'s
        field reads."""
        self._raw_balls = list(msg.balls)
        for b in msg.balls:
            t_land = float(b.time_at_land.sec) + float(b.time_at_land.nanosec) * 1e-9
            self._balls[int(b.id)] = Landing(
                pos_mm=np.array([b.landing_position.x, b.landing_position.y,
                                 b.landing_position.z], dtype=float),
                vel_mm_s=np.array([b.landing_velocity.x, b.landing_velocity.y,
                                   b.landing_velocity.z], dtype=float),
                t_land_abs_s=t_land)
        self._advance_correlation()

    def _now_s(self) -> float:
        """Now, in ROS epoch seconds — the clock the announcements' own
        ``throw_time`` is on, and therefore the only clock a "has this release
        happened?" comparison may use (`_advance_correlation`, `_tracker`)."""
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _advance_correlation(self) -> None:
        """Refine every live schedule-ball-id -> tracker-id correlation
        against the latest ``/balls`` snapshot (item 3, plan § 2.7).

        ``jugglebot.ball_possession`` owns the RULE — `latch_announced_ball`
        per announced release (shared with `reload_coordinator_node`'s FSM
        latch, plan discipline: one rule, not a lookalike copy) and
        `advance_flight_latches` over the per-release queue; this method owns
        the per-schedule-ball-id state the rule threads through."""
        now_s = self._now_s()
        with self._correlation_lock:
            for ball_id, latches in list(self._correlation.items()):
                self._correlation[ball_id] = advance_flight_latches(
                    self._raw_balls, robot_name=self._robot_name,
                    latches=latches, now_s=now_s,
                    in_flight_status=_BALL_STATUS_IN_FLIGHT)

    def _tracker(self, ball_id: int):
        """The executor's tracker callable — keyed by SCHEDULE ball id.

        `/balls` ids are the tracker's OWN (item 3): a schedule ball id must
        first be correlated to one via `_maybe_announce` / `_advance_correlation`
        before a landing can be returned, and then only for the flight
        ACTUALLY IN PROGRESS — the latest announced release that has happened,
        whose id is both IN_FLIGHT and CONFIRMED (a raw IN_FLIGHT status alone
        is time-based and proves nothing — see `_BALL_TRACKING_CONFIRMED`;
        `ball_possession.flight_in_progress` holds the whole rule and the
        reason an earlier latch is never a fallback)."""
        latches = self._correlation.get(int(ball_id))
        if not latches:
            return None
        tracker_id = flight_in_progress(
            self._raw_balls, latches=latches, now_s=self._now_s(),
            in_flight_status=_BALL_STATUS_IN_FLIGHT,
            confirmed_tracking=_BALL_TRACKING_CONFIRMED)
        if tracker_id is None:
            return None
        return self._balls.get(int(tracker_id))

    def _catch_aim_source(self) -> str:
        """The validated ``catch_aim_source`` parameter. An unknown value
        falls back to the LIVE default (:data:`executor.AIM_SCHEDULE`) with an
        error logged, rather than refusing the attempt: a typo in a launch
        override must not leave the operator with no catch at all, and the
        open-loop aim is the safe one to fall back to."""
        value = str(self.get_parameter('catch_aim_source').value)
        if value not in ex.AIM_SOURCES:
            self.get_logger().error(
                'catch_aim_source=%r is not one of %s — using %r'
                % (value, list(ex.AIM_SOURCES), ex.AIM_SCHEDULE))
            return ex.AIM_SCHEDULE
        return value

    def _launch_ratio(self, ball_id: int, t_release_abs_s: float):
        """The executor's ``launch_ratio`` callable: the MEASURED hand
        launch-speed ratio of the stroke ending at ``t_release_abs_s``, or
        ``None`` when the hand telemetry cannot vouch for one (the executor
        then keeps the theoretical aim). ``ball_id`` is unused — this hand
        throws one ball at a time, and the release INSTANT is what selects
        the stroke."""
        r = self._hand_launch.ratio(t_release_abs_s)
        self.get_logger().info(
            'hand launch ratio for the release at %.3f (ball %d): %s'
            % (t_release_abs_s, ball_id, self._hand_launch.last_reason))
        return r

    def _on_traj_status(self, msg) -> None:
        """Cache the latest `trajectory/status` for `_svc_start_self_toss`'s
        limits check and the R3 ladder's `levelled`/`in_trajectory_mode`
        fields (`_observations`, item 6). Perf-stamped like every other
        freshness-gated cache in this node."""
        self._traj_status = msg
        self._traj_status_mono = time.perf_counter()

    def _on_mocap(self, msg) -> None:
        """`rigid_body_poses` freshness only — the R3 ladder's `mocap_fresh`
        (item 6) asks whether the mocap graph is publishing AT ALL, not
        anything about a specific body's pose (the skill stack has no
        per-body cross-check at R3, unlike `reload_coordinator_node`'s toss
        positioning check)."""
        self._mocap_mono = time.perf_counter()

    def _on_commanded_position(self, msg) -> None:
        """`trajectory/commanded_position`: the platform's live commanded
        (x, y, z) in STOW mm — restated from
        `reload_coordinator_node._on_commanded_position` (same NaN-drop
        discipline: a poisoned pose must never seed a pre-level move).
        `_prelevel` (item 7) is the only reader."""
        p = (float(msg.x), float(msg.y), float(msg.z))
        if not all(math.isfinite(v) for v in p):
            self.get_logger().error(
                'trajectory/commanded_position %r is non-finite — DISCARDED'
                % (p,))
            return
        self._commanded_pos_mm = p
        self._commanded_pos_mono = time.perf_counter()

    def _live_commanded_position(self, now: float):
        """The live commanded platform (x, y, z) mm, or ``None`` when absent
        or stale — fail-closed, same shape as
        `reload_coordinator_node._live_commanded_position`."""
        pos = self._commanded_pos_mm
        mono = self._commanded_pos_mono
        if pos is None or mono <= 0.0 or (now - mono) >= _TRAJ_STATUS_STALE_S:
            return None
        return pos

    def _on_hand_telemetry(self, msg):
        """Track the hand's live position and possession evidence — the same
        tri-state `reload_coordinator_node._on_hand_telemetry` feeds into
        `ball_possession.HandBallSensorSource`. `pos_meas`/`pos_cmd` and the
        perf stamp feed the R3 ladder's `hand_fresh` (`_observations`,
        item 6); `_possession_evidence` is unused at R2
        beyond logging but IS the R3 ladder's `ball_evidence` and outcome
        capture's `observer` read (`_ball_evidence`) — one value, two
        readers, stored directly here rather than standing up the source's
        full arrival/retention state machine for a value nothing else
        consumes."""
        self._hand_pos_meas = float(getattr(msg, 'pos_meas', 0.0))
        self._hand_pos_cmd = float(getattr(msg, 'pos_cmd', 0.0))
        self._hand_telemetry_mono = time.perf_counter()
        # The launch-ratio monitor's sample, stamped on the ROS WALL clock
        # (the clock the schedule's release instants live on — a perf stamp
        # would not be comparable with one). Arrival time, not a bridge
        # stamp: the ratio compares PEAKS of the two channels inside one
        # window, so the few ms of transport lag shifts both channels of the
        # same message identically and cannot bias the ratio.
        self._hand_launch.add_sample(
            self.get_clock().now().nanoseconds / 1e9,
            float(getattr(msg, 'vel_ff_cmd', 0.0)),
            float(getattr(msg, 'vel_meas', 0.0)))
        valid = bool(getattr(msg, 'ball_held_valid', False))
        raw = getattr(msg, 'ball_held_raw', None)
        if not valid or raw is None:
            self._possession_evidence = ball_possession.EVIDENCE_UNKNOWN
        else:
            self._possession_evidence = (
                ball_possession.EVIDENCE_SEATED if bool(raw)
                else ball_possession.EVIDENCE_EMPTY)
        # The DEBOUNCED bit feeds the precondition ladder (2026-09-16): the raw
        # bit is right for release/catch EDGES (the debounce lags a departing
        # ball by ~240 ms, ball_possession.py) but it chatters while the hand
        # moves — two of the sitting's three REJECTED_NO_BALL refusals were a
        # single raw-sample carry-flicker during the post-hold park motion with
        # the debounced bit True throughout. A fresh THROW asks "is a ball
        # seated", a state question the debounced bit answers.
        held = getattr(msg, 'ball_held', None)
        if not valid or held is None:
            self._possession_evidence_stable = ball_possession.EVIDENCE_UNKNOWN
        else:
            self._possession_evidence_stable = (
                ball_possession.EVIDENCE_SEATED if bool(held)
                else ball_possession.EVIDENCE_EMPTY)

    def _observations(self, t_abs_s: float) -> Observations:
        """The R3 precondition ladder's snapshot of the whole machine at one
        instant (item 6; fields per `executor.Observations`'s docstring).

        Staleness is judged on THIS node's own `time.perf_counter()` — the
        SAME monotonic domain every cache above is stamped in — never on
        ``t_abs_s`` (the schedule's own wall clock, read off
        `self.get_clock()` in `_on_tick`/`_svc_check`, which may carry a
        CAN-offset the perf-clock caches know nothing about).

        Fed to BOTH the executor's ``observations=`` and, via
        `_ball_evidence`, its ``observer=`` — one builder, no second copy of
        the evidence read.
        """
        now = time.perf_counter()
        mocap_fresh = (self._mocap_mono > 0.0
                       and (now - self._mocap_mono) < _MOCAP_STALE_S)
        status_fresh = (self._traj_status_mono > 0.0
                        and (now - self._traj_status_mono)
                        < _TRAJ_STATUS_STALE_S)
        levelled = bool(status_fresh
                        and self._traj_status.gravity_correction_loaded)
        in_trajectory_mode = bool(status_fresh
                                  and self._traj_status.mode == 'TRAJECTORY')
        hand_fresh = (self._hand_telemetry_mono > 0.0
                     and (now - self._hand_telemetry_mono)
                     < _HAND_STATE_STALE_S)
        # NO hand-POSITION predicate here (RETIRED 2026-09-16, owner
        # decision). `REJECTED_HAND_NOT_PARKED` used to be built from
        # `pos_meas` against `pos_cmd` AND against the ACTIVATE park, and on
        # 2026-09-16 it refused nine schedules at skill 0 on a hand that was
        # genuinely parked: the bridge's `pos_cmd` echo had gone stale at
        # +0.5639 rev across an ACTIVATE re-park while `pos_meas` read
        # +0.0001. The honest enforcement point for "the plan's hand seed is
        # not where the hand is" is the SEED, and it now RECONCILES rather
        # than refuses (`trajectory_node._cycle_start_state`) — the opening
        # REST carries the hand home. `hand_fresh` stays: that seed is
        # reconciled against the encoder, so a stale encoder is still a
        # refusal, one level down.
        return Observations(
            mocap_fresh=mocap_fresh, hand_fresh=hand_fresh,
            levelled=levelled, ball_evidence=self._possession_evidence_stable,
            in_trajectory_mode=in_trajectory_mode)

    def _ball_evidence(self, ball_id: int, t_abs_s: float) -> str:
        """The executor's ``observer`` callable (item 6): the live possession
        evidence from the RAW sensor bit — one hand sensor, ``ball_id`` unused
        (there is exactly one cup). Raw, not debounced, on purpose: this read
        detects release and catch EDGES, and the debounce lags a departing
        ball by ~240 ms (ball_possession.py). The ladder's SEATED precondition
        reads the debounced twin instead (`_observations`, 2026-09-16)."""
        return self._possession_evidence

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
        result = InstallResult(bool(resp.accepted), str(resp.code),
                               str(resp.message),
                               float(resp.plan_wall_ms) / 1e3,
                               splice_k=int(resp.splice_k),
                               t0_s=float(resp.t0_mono),
                               event_t_s=float(resp.t_event_mono),
                               seeded_post_release=bool(resp.seeded_post_release))
        # The release instant ON THE WIRE: a THROW's release IS its event
        # (`t_event_mono`); only a CATCH-with-throw carries a second event,
        # which rides `t_release_mono` (0.0 for every other shape — a plain
        # THROW included, so reading it here announced NOTHING for a
        # standalone throw: five single self-tosses on 2026-09-13 ended
        # NO_LANDING with a CONFIRMED track and a valid landing in /balls).
        release_mono = (float(resp.t_event_mono) if kind == THROW
                        else float(resp.t_release_mono))
        # Unit A (R3 first sitting, 2026-09-13, L1): the SAME instant, kept
        # as the streaming plan's own future event -- a THROW's release, or
        # a CATCH-with-throw's carried one. 0.0 (a plain CATCH, a REST)
        # clears it: that install carries no future event, so whatever an
        # EARLIER accepted install left pending is now stale (this one
        # superseded it on the wire). Only an ACCEPTED install updates this
        # -- a refusal changes nothing about what is actually streaming.
        if result.accepted:
            self._pending_event_mono = release_mono if release_mono > 0.0 else 0.0
        self._maybe_announce(kind, terminal, result, release_mono,
                             ball_id=ball_id)
        return result

    @staticmethod
    def _release_physics(kind, terminal):
        """``(release_abs_s, site_mm, target_mm, flight_s)`` for the ball
        ``terminal`` releases, or ``None`` when it releases nothing (a plain
        CATCH, a REST). ``release_abs_s`` is the ABSOLUTE wall-clock instant
        the terminal itself carries (the same field `_installer` puts on the
        wire), used by `_maybe_announce` to de-duplicate a CATCH re-send."""
        if kind == THROW:
            return (float(terminal.t_release_s),
                   np.asarray(terminal.site_mm, dtype=float),
                   np.asarray(terminal.target_mm, dtype=float),
                   float(terminal.flight_s))
        if kind == CATCH and terminal.then_throw is not None:
            tt = terminal.then_throw
            return (float(tt.t_release_s), np.asarray(tt.site_mm, dtype=float),
                   np.asarray(tt.target_mm, dtype=float), float(tt.flight_s))
        return None

    def _maybe_announce(self, kind, terminal, result: InstallResult,
                        t_release_mono: float, *, ball_id) -> None:
        """Publish a ``ThrowAnnouncement`` for every ACCEPTED install that
        releases a ball — a THROW, or a CATCH carrying ``then_throw`` (item 3).

        Same six physics fields, same units/frame, same ``thrower_name ==
        target_id == robot_name`` discipline as `reload_coordinator_node.
        _announce_unified`, so every downstream consumer (tracker correlation,
        possession, suppression) is unaffected by which node announced it. The
        release VELOCITY is not on the wire (`InstallSegment.srv` carries the
        commanded site/target/flight, not a realized velocity — there is no
        trim to un-trim here, unlike the FSM path), so it is recomputed from
        exactly those three fields via `ballistics_bc.launch_velocity` — the
        same boundary condition the segment itself was planned to satisfy.

        QUEUES a FRESH, unlatched `FlightLatch` for this release — it does
        NOT reset the schedule ball's correlation. The physical ball a
        self-toss re-throws gets a NEW tracker id each flight, so this
        release needs its own latch; but the next throw is announced at its
        DISPATCH, ~1.25 s before its release, while the PREVIOUS flight of the
        same schedule ball is still in the air — so a reset would throw away
        the correlation of the flight in progress and (via `preexisting`,
        which excludes exactly the airborne id) re-latch onto the NEXT
        flight's id the moment it goes IN_FLIGHT. That is how the 2026-09-16
        sitting's outcome rows came to hold the NEXT flight's landing
        (`logbook/2026-09-16-tracker-correlation-follows-the-flight-in-progress.md`).
        The latch carries the release on the ROS clock — the announcement's
        own `throw_time`, the same instant the tracker flips the ball to
        IN_FLIGHT — because `flight_in_progress` may only read a release that
        has already happened.

        **Exactly once per throw** (never on a CATCH re-send of the same
        carried release): a re-send calls this too, with a terminal whose
        carried `then_throw` is bit-identical to the one already announced
        (the executor's own `_u_cache` caches it by skill index — see
        `SkillExecutor._catch_terminal`'s docstring), so de-duplicating on the
        release's own absolute wall-clock instant is exact, not a heuristic.
        """
        physics = self._release_physics(kind, terminal)
        if physics is None or not result.accepted or t_release_mono <= 0.0:
            return
        release_abs_s, site_mm, target_mm, flight_s_ = physics
        if self._announced_release_s.get(int(ball_id)) == release_abs_s:
            return
        self._announced_release_s[int(ball_id)] = release_abs_s
        vel = ballistics_bc.launch_velocity(site_mm, target_mm, flight_s_)
        lv = ballistics_bc.arrival_velocity(vel, flight_s_)
        now_perf = time.perf_counter()
        now_ros = self.get_clock().now()
        delta_s = float(t_release_mono) - now_perf
        ann = ThrowAnnouncement()
        ann.header.stamp = now_ros.to_msg()
        ann.header.frame_id = 'world'
        ann.thrower_name = self._robot_name
        ann.target_id = self._robot_name
        ann.initial_position = Point(x=float(site_mm[0]), y=float(site_mm[1]),
                                     z=float(site_mm[2]))
        ann.initial_velocity = Vector3(x=float(vel[0]), y=float(vel[1]),
                                       z=float(vel[2]))
        ann.landing_position = Point(x=float(target_mm[0]), y=float(target_mm[1]),
                                     z=float(target_mm[2]))
        ann.landing_velocity = Vector3(x=float(lv[0]), y=float(lv[1]),
                                       z=float(lv[2]))
        ann.predicted_tof_sec = flight_s_
        ann.throw_time = (now_ros + rclpy.time.Duration(seconds=delta_s)).to_msg()
        ann.landing_time = (now_ros + rclpy.time.Duration(
            seconds=delta_s + flight_s_)).to_msg()
        # Ids already IN_FLIGHT at THIS release are phantoms, never our new
        # ball — the same hardening pass `latch_announced_ball` documents.
        # (A previous flight of THIS schedule ball is one of them, and keeps
        # its own latch below: this set is what stops the new release from
        # stealing it, now that the latch is no longer reset.)
        preexisting = tuple(sorted(int(b.id) for b in self._raw_balls
                                   if int(b.status) == _BALL_STATUS_IN_FLIGHT))
        t_release_ros_s = (float(now_ros.nanoseconds) * 1e-9) + delta_s
        with self._correlation_lock:
            self._correlation[int(ball_id)] = advance_flight_latches(
                self._raw_balls, robot_name=self._robot_name,
                latches=tuple(self._correlation.get(int(ball_id), ()))
                        + (FlightLatch(t_release_s=t_release_ros_s,
                                       preexisting=preexisting),),
                now_s=float(now_ros.nanoseconds) * 1e-9,
                in_flight_status=_BALL_STATUS_IN_FLIGHT)
        self._announce_pub.publish(ann)
        self.get_logger().info(
            'skill announced ball %d: release in %.3f s, |v| %.3f m/s'
            % (int(ball_id), delta_s, float(np.linalg.norm(vel)) / 1000.0))

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

    def _maybe_hold_pending_event(self, end_code: str) -> None:
        """``trajectory/hold`` once when the streaming plan still carries a
        future event (Unit A, R3 first sitting, 2026-09-13, L1): at
        1789304571.365 the executor ended ``ABORTED_NO_RELEASE``, but the
        last accepted install (a CATCH-with-throw) kept streaming -- the
        hand ran two more full strokes after END, the second at the
        session's own acceleration ceiling, and the guard latched.
        ``_svc_stop``'s docstring says "whatever is streaming already ends
        at rest" -- true of every segment's TERMINAL, but a streaming
        segment can still carry a RELEASE ahead of it, and ending the
        attempt does nothing to that.

        Called from `_on_tick` (every tick while ``attempt_ended`` is True)
        and from `_svc_stop` (an operator stop is a stop of the MACHINE,
        not only of dispatch) -- idempotent either way, because
        ``_pending_event_mono`` is cleared the moment a hold is attempted,
        so a later call with nothing left pending is a no-op.
        """
        # `_pending_event_mono` is on the WIRE's clock — `time.perf_counter()`
        # (`trajectory_node`'s `t0_mono`/`t_event_mono`, the same domain
        # `_maybe_announce` differences against) — NOT the ROS clock the
        # executor ticks on; comparing it to the tick's `now` (ROS epoch,
        # ~1.79e9) would read every pending event as already past.
        with self._pending_lock:
            if (self._pending_event_mono <= 0.0
                    or self._pending_event_mono <= time.perf_counter()):
                return
            self._pending_event_mono = 0.0
        if not self._hold_cli.wait_for_service(timeout_sec=_SERVICE_WAIT_S):
            self.get_logger().error(
                'END %s: trajectory/hold unavailable -- a pending event '
                'could not be cancelled' % (end_code,))
            return
        resp = self._wait_future(self._hold_cli.call_async(Trigger.Request()))
        if resp is None or not bool(resp.success):
            self.get_logger().error(
                'END %s: trajectory/hold failed (%s)'
                % (end_code, '' if resp is None else resp.message))
            return
        self.get_logger().info(
            'END %s: hold installed — 1 pending event(s) cancelled'
            % (end_code,))

    # ── the tick ────────────────────────────────────────────────────────

    def _on_tick(self):
        """Dispatch everything due, then retire a finished executor.

        Guarded against RE-ENTRY: the install client and this timer share one
        `ReentrantCallbackGroup` (see `__init__`) so the client's response can
        be processed WHILE a tick is blocked in `_installer._wait_future` —
        the fix for the single-threaded deadlock this replaces. But that same
        reentrancy would let the executor run a SECOND tick on another thread
        while the first is still mid-install; `acquire(blocking=False)` makes
        an already-busy tick a no-op instead of a second dispatch.

        Retires on `executor.done`, not `attempt_ended` — a released ball's
        outcome can still be finalising after the attempt ends (plan § 2.7),
        and `done` is exactly `attempt_ended` when nothing is listening for
        outcomes (`on_experience is None`), so this is a strict generalisation
        of the R2 behaviour, not a change to it.
        """
        if not self._tick_lock.acquire(blocking=False):
            return
        try:
            if self._executor is None:
                return
            now = self.get_clock().now().nanoseconds / 1e9
            for line in self._executor.tick(now):
                self.get_logger().info(line)
            if self._executor.attempt_ended:
                self._maybe_hold_pending_event(self._executor.end_code)
            if self._executor.done:
                self._executor = None
        finally:
            self._tick_lock.release()

    # ── attempt lifecycle ──────────────────────────────────────────────

    def _refuse_if_running(self, response):
        """``response`` refused with the shared "already running" message, or
        ``None`` when no attempt is running (the caller may proceed)."""
        if self._executor is None:
            return None
        response.success = False
        response.message = ('an attempt is already running — call '
                            'skills/stop first')
        return response

    def _svc_start_columns(self, request, response):
        refused = self._refuse_if_running(response)
        if refused is not None:
            return refused
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
        # A new schedule reuses small ball ids: drop any flight latch an
        # earlier (possibly early-ended) attempt left for them, so a catch
        # with no in-schedule release can never read a previous attempt's
        # flight (audit, 2026-09-16).
        with self._correlation_lock:
            for _bid in {sk.ball_id for sk in schedule.skills}:
                self._correlation.pop(_bid, None)
        self._executor = SkillExecutor(
            schedule, self._installer, tracker=self._tracker,
            catch_aim_source=self._catch_aim_source(),
            launch_ratio=self._launch_ratio)
        response.success = True
        response.message = ('columns schedule compiled: %d skills, %d throws, '
                            't0=%.3f' % (len(schedule.skills), n_throws, t0))
        self.get_logger().info(response.message)
        return response

    def _live_limits(self):
        """The session limits `admissible.check_limits` judges the loaded box
        against: LEG limits off the latest `trajectory/status`
        (`leg_*_limit_*`), falling back to the YAML module default on a field
        that reads 0.0 — the message's own documented "field absent" sentinel
        (`TrajectoryStatus.msg`) — and the HAND accel cap, which the status
        message does not carry at all (no live topic publishes a session hand
        cap), so it is the static session default (`hw.
        JB_TRAJ_HAND_ACC_LIMIT_RPS2`) rather than a value read off the wire."""
        status = self._traj_status
        leg_vel = float(status.leg_vel_limit_mmps) or float(hw.JB_TRAJ_LEG_VEL_LIMIT_MMPS)
        leg_acc = float(status.leg_acc_limit_mmps2) or float(hw.JB_TRAJ_LEG_ACC_LIMIT_MMPS2)
        leg_jerk = float(status.leg_jerk_limit_mmps3) or float(hw.JB_TRAJ_LEG_JERK_LIMIT_MMPS3)
        return SimpleNamespace(leg_vel_mmps=leg_vel, leg_acc_mmps2=leg_acc,
                               leg_jerk_mmps3=leg_jerk,
                               hand_acc_limit_rps2=float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2))

    def _prelevel(self) -> str:
        """Bring the platform to gravity-level rest before compiling the R3
        self-toss schedule (item 7; plan R3 carried note, ``''`` on success
        else why not). Mirrors
        `reload_coordinator_node._unified_prelevel`: the
        session-start schedule is planned from the seed the machine is
        ALREADY holding, and an un-prelevelled seed refuses ``LIMIT_JERK`` at
        174 772 mm/s^3 against the 150 000 session limit (prelevelled: 19 773
        — probe 2026-09-13). A pure attitude move: the live commanded xy/z is
        held exactly, only the attitude comes to gravity-level (the E3
        ingest, `trajectory_node`'s `GoToPose` handler, turns an IDENTITY
        intent into the gravity-level counter-tilt — C-LEVEL-1)."""
        if not self._go_to_pose_cli.wait_for_service(timeout_sec=_SERVICE_WAIT_S):
            return 'trajectory/go_to_pose unavailable'
        live = self._live_commanded_position(time.perf_counter())
        if live is None:
            return ('trajectory/commanded_position is stale — the pre-level '
                    'move has no xy/z to hold and one that guessed a pose '
                    'would move the platform')
        req = GoToPose.Request()
        req.pose = Pose(position=Point(x=float(live[0]), y=float(live[1]),
                                       z=float(live[2])),
                        orientation=Quaternion())
        req.duration_s = 0.0
        resp = self._wait_future(self._go_to_pose_cli.call_async(req))
        if resp is None:
            return ('trajectory/go_to_pose did not answer in %.1f s'
                    % (_SERVICE_WAIT_S,))
        if not bool(resp.accepted):
            return ('the pre-level move was refused: %s (%s)'
                    % (resp.code, resp.message))
        self.get_logger().info(
            'platform pre-levelled to gravity-level over %.2f s before the '
            'self-toss schedule (identity intent, E3-corrected)'
            % (float(resp.planned_duration_s),))
        # go_to_pose returns at plan INSTALL; wait out the planned move so the
        # schedule compiles from a STOPPED, gravity-level machine.
        end_at = time.perf_counter() + float(resp.planned_duration_s)
        while rclpy.ok() and time.perf_counter() < end_at:
            time.sleep(min(1.0 / _TICK_HZ, max(0.0, end_at - time.perf_counter())))
        return ''

    def _svc_start_self_toss(self, request, response):
        """``skills/start_self_toss``: the R3 single-site self-toss (plan §
        0 / R3 build note) — THROW(P1) -> CATCH(P1) -> ... -> REST, the
        learner on, memory at ``temp/learn/<plant_id>/memory.csv``.

        Order: refuse fast on anything that does not need the platform to
        move (running / repo root / pattern / box+limits) BEFORE
        `_prelevel` (item 7) actually moves it, then compile the schedule —
        pre-levelling is the last check before compilation, per the R3
        carried note ("a healthy launch rests the cup at the gravity-level
        counter-tilt").
        """
        refused = self._refuse_if_running(response)
        if refused is not None:
            return refused
        if _ADMISSIBLE_BOX_PATH is None:
            response.success = False
            response.message = ('cannot find the repo root from %r — the '
                                'admissible box / memory paths cannot be '
                                'resolved' % (__file__,))
            self.get_logger().error(response.message)
            return response

        site_x_mm = float(self.get_parameter('site_x_mm').value)
        site_y_mm = float(self.get_parameter('site_y_mm').value)
        apex_m = float(self.get_parameter('apex_m').value)
        dwell_s = float(self.get_parameter('dwell_s').value)
        n_throws = int(self.get_parameter('n_throws').value)
        plant_id = str(self.get_parameter('plant_id').value)

        # Finding 5, R3 audit (2026-09-13): `skills/start_self_toss` always
        # names the site 'P1', but the ADMISSIBLE BOX it was swept against
        # (`_ADMISSIBLE_BOX_PATH`, keyed on `(site.name, target.name)`) carries
        # no site xy at all — a box swept at the default (-50, 0) mm site is
        # silently applied to whatever xy the caller passes. Refuse before
        # anything else (no platform motion, no box/pattern object built yet)
        # rather than let an off-box site reach `_command_u`'s clip with the
        # wrong box.
        if (abs(site_x_mm - _DEFAULT_SITE_X_MM) > 1e-9
                or abs(site_y_mm - _DEFAULT_SITE_Y_MM) > 1e-9):
            response.success = False
            response.message = (
                'self-toss refused: site (%.3f, %.3f) mm != the swept '
                'default (%.3f, %.3f) mm — the admissible box carries no '
                'site xy, so a box swept at the default site cannot be '
                'applied here' % (site_x_mm, site_y_mm, _DEFAULT_SITE_X_MM,
                                  _DEFAULT_SITE_Y_MM))
            self.get_logger().error(response.message)
            return response

        site = Site(_DEFAULT_SITE_NAME,
                   np.array([site_x_mm, site_y_mm, CATCH_CUP_Z_MM]))
        try:
            pattern = SelfTossPattern(site=site, apex_m=apex_m, dwell_s=dwell_s,
                                      n_throws=n_throws)
        except ValueError as exc:
            response.success = False
            response.message = 'self-toss pattern refused: %s' % (exc,)
            self.get_logger().error(response.message)
            return response

        try:
            boxes = adm.load(_ADMISSIBLE_BOX_PATH)
            adm.check_limits(boxes, self._live_limits())
        except adm.AdmissibleError as exc:
            response.success = False
            response.message = 'admissible box refused: %s' % (exc,)
            self.get_logger().error(response.message)
            return response
        # No box covers the requested apex -> refuse BEFORE any motion
        # (before `_prelevel`), rather than let `_command_u` discover this
        # only once the learner is already asking for a command (the latent
        # defect this closes: a box swept for one apex silently reused at
        # another).
        pair = (site.name, site.name)
        if adm.select(boxes, pair, apex_m) is None:
            bands = sorted(b.apex_band_m for b in boxes if b.site_pair == pair)
            bands_str = (', '.join('%.3f-%.3f m' % (lo, hi) for lo, hi in bands)
                        if bands else 'none swept for this pair')
            response.success = False
            response.message = (
                'self-toss refused: no admissible box covers site pair %r '
                'at apex %.3f m (bands swept for this pair: %s)'
                % (pair, apex_m, bands_str))
            self.get_logger().error(response.message)
            return response

        # Finding 11, R3 audit (2026-09-13): built here, before `_prelevel`
        # actually moves the platform, and guarded — `memory_path` raises
        # `ValueError` on a bad `plant_id`, and `Memory(...)` can raise
        # `OSError` (unreadable file) or `csv.Error` (malformed rows); left
        # uncaught in a service callback either kills the node AFTER the
        # platform has already moved.
        try:
            memory = Memory(memory_path(_REPO_ROOT, plant_id))
        except (ValueError, OSError, csv.Error) as exc:
            response.success = False
            response.message = 'memory refused: %s' % (exc,)
            self.get_logger().error(response.message)
            return response

        prelevel_err = self._prelevel()
        if prelevel_err:
            response.success = False
            response.message = 'self-toss refused: %s' % (prelevel_err,)
            self.get_logger().error(response.message)
            return response

        now = self.get_clock().now().nanoseconds / 1e9
        try:
            # Compile once to read the opening REST's own dispatch lead (the
            # floor lift + Skill.lead_s arithmetic — schedule.FLOOR_LIFT_S /
            # _assign_leads), rather than restating it as a fixed margin: a
            # PROBE compile at t0=now puts the first dispatch `lead_s` (0.15 s
            # at the current grid) before `now`, so shifting t0 by exactly
            # that deficit plus one tick period of operational margin (the
            # call/dispatch latency `_START_LEAD_S` names for columns) lands
            # the real schedule's first dispatch at or after "now".
            probe = compile_self_toss(pattern, now)
            deficit = now - probe.skills[0].dispatch_s()
            t0 = now + max(deficit, 0.0) + (1.0 / _TICK_HZ)
            schedule = compile_self_toss(pattern, t0)
        except ValueError as exc:
            response.success = False
            response.message = 'self-toss schedule refused: %s' % (exc,)
            self.get_logger().error(response.message)
            return response

        learner_cfg = lr.LearnerConfig()
        learner = SimpleNamespace(
            command=lambda x, y_d: memory.command(x, y_d, learner_cfg))

        # A new schedule reuses small ball ids: drop any flight latch an
        # earlier (possibly early-ended) attempt left for them, so a catch
        # with no in-schedule release can never read a previous attempt's
        # flight (audit, 2026-09-16).
        with self._correlation_lock:
            for _bid in {sk.ball_id for sk in schedule.skills}:
                self._correlation.pop(_bid, None)
        self._executor = SkillExecutor(
            schedule, self._installer, tracker=self._tracker,
            catch_aim_source=self._catch_aim_source(),
            launch_ratio=self._launch_ratio,
            learner=learner, boxes=boxes,
            lateral_authority_m=float(self.get_parameter(
                'learner_lateral_authority_mm').value) / 1000.0,
            observer=self._ball_evidence, observations=self._observations,
            on_experience=self._bind_on_experience(memory))
        response.success = True
        response.message = (
            'self-toss schedule compiled: %d skills, %d throws, plant_id=%r, '
            'memory rows=%d, t0=%.3f'
            % (len(schedule.skills), n_throws, plant_id, len(memory), t0))
        self.get_logger().info(response.message)
        return response

    def _bind_on_experience(self, memory: Memory):
        """``on_experience`` callable bound to ``memory`` (plan § 2.5 step 6):
        appends the row (file I/O, orchestrator thread only — never the 40 Hz
        emitter, plan § 0) and logs it, one line per throw."""
        def _on_experience(exp):
            # `Memory.append` REFUSES a row outside `FLIGHT_RATIO_BAND`
            # (2026-09-16) — the executor drops such a row first, so this is
            # the belt-and-braces path, and it must never raise into the tick
            # loop: an exception here would surface as an executor fault and
            # end an attempt over a bookkeeping refusal.
            try:
                memory.append(exp)
            except ValueError as exc:
                self.get_logger().error(
                    'memory row REFUSED (not appended): %s — u=%s y=%s'
                    % (exc, exp.u.tolist(), exp.y.tolist()))
                return
            self.get_logger().info(
                'memory row appended: x=%s u=%s y=%s caught=%s'
                % (exp.x.tolist(), exp.u.tolist(), exp.y.tolist(), exp.caught))
        return _on_experience

    def _svc_stop(self, request, response):
        """End the current attempt. Every segment is rest-terminal, so
        whatever is streaming already ends at rest — but rest-terminal is
        not event-free: a streaming segment can still carry a RELEASE ahead
        of it (a THROW, or a CATCH-with-throw), and ending the attempt does
        nothing to that on its own. `_maybe_hold_pending_event` (Unit A)
        installs one `trajectory/hold` when that is the case; otherwise this
        issues no motion command of its own.

        Does NOT discard the executor: a released ball's outcome can still
        be finalising after the attempt ends (plan § 2.7, "outcomes keep
        finalising after the attempt ends") — clearing `_executor` here
        (the earlier behaviour) silently dropped that flight's memory row.
        `_on_tick` already retires on `executor.done`, not `attempt_ended`
        alone, so setting the two flags and leaving the executor live is
        enough: a stopped attempt's observable flight still produces its
        memory row, and `_refuse_if_running`'s existing `_executor is not
        None` check refuses a new start for as long as that finalisation is
        still pending.

        Takes `_tick_lock` first (finding 12, R3 audit, 2026-09-13, bounded
        by `_STOP_LOCK_WAIT_S`): without it, this can race a tick mid-
        `_dispatch` on the timer thread — `_svc_stop` sets `attempt_ended` /
        `end_code` from the service thread while `_on_tick` may be about to
        overwrite them with a dispatch's own refusal code, or STOPPED may
        itself overwrite a genuine abort the same tick produced a moment
        earlier. A failure to acquire within the bound is logged and the stop
        proceeds anyway — a stop that never lands is worse than one that
        loses this one race."""
        got_lock = self._tick_lock.acquire(timeout=_STOP_LOCK_WAIT_S)
        if not got_lock:
            self.get_logger().warning(
                'skills/stop: could not acquire the tick lock within %.1f s '
                '— proceeding without it' % _STOP_LOCK_WAIT_S)
        ended_now = False
        try:
            if self._executor is not None:
                if not self._executor.attempt_ended:
                    self._executor.attempt_ended = True
                    self._executor.end_code = 'STOPPED'
                    ended_now = True
                    response.message = (
                        'attempt stopped — the rest tail is already streaming')
                else:
                    response.message = (
                        'attempt already ended (end_code=%r) — stop is a '
                        'no-op' % self._executor.end_code)
            else:
                response.message = 'no attempt was running'
        finally:
            if got_lock:
                self._tick_lock.release()
        if ended_now:
            # An operator stop is a stop of the MACHINE, not only of
            # dispatch (Unit A) — the same hold `_on_tick` installs on an
            # abort, called here outside the tick lock so a blocked
            # `trajectory/hold` round trip cannot stall ticking.
            self._maybe_hold_pending_event('STOPPED')
        response.success = True
        self.get_logger().info(response.message)
        return response

    def _svc_check(self, request, response):
        """``skills/check`` (item 8): every current R3 precondition-ladder
        refusal at once, plus the admissible-box/limits status — the
        dress-rehearsal runsheet calls this before any powered attempt
        (Rigor: "make gates report every refusal at once", plan Workflow
        Rules). Never moves the platform and never touches ``_executor``.
        """
        now = self.get_clock().now().nanoseconds / 1e9
        obs = self._observations(now)
        # `launch=True`: a self-toss attempt's opening THROW carries every
        # row this ladder has, so rehearsing with it reports the widest set.
        # The retired `fresh_origin=` argument is gone with the hand-position
        # row it gated (2026-09-16) — a fresh origin now RECONCILES its hand
        # seed in `trajectory_node._cycle_start_state` instead of refusing.
        codes = precondition_refusals(obs, launch=True)
        ok = True
        lines = []
        if codes:
            ok = False
            lines.append('ladder REFUSED: %s' % (', '.join(codes),))
        else:
            lines.append('ladder OK')
        # The hand's POSITION is REPORTED, never gated (2026-09-16): a hand
        # off the park is a thing the opening REST carries home, and the
        # rehearsal's job is to let the operator SEE it beforehand. `pos_cmd`
        # is the bridge's own echo and is the channel that went stale in the
        # 2026-09-16 sitting — named next to `pos_meas` so a disagreement is
        # visible rather than inferred.
        lines.append(
            'hand pos_meas %.4f rev (park %.2f, band ±%.2f), bridge echo '
            'pos_cmd %.4f rev — REPORTED, not gated: the opening REST plans '
            'from the MEASURED hand and settles the cup at %.1f mm'
            % (self._hand_pos_meas, float(hw.JB_OP_HAND_RETRACT_REV),
               float(hw.HOMING_HAND_PARK_BAND_REV), self._hand_pos_cmd,
               REST_CUP_Z_MM))
        if _ADMISSIBLE_BOX_PATH is None:
            ok = False
            lines.append('box REFUSED: cannot find the repo root from %r'
                         % (__file__,))
        else:
            try:
                boxes = adm.load(_ADMISSIBLE_BOX_PATH)
                adm.check_limits(boxes, self._live_limits())
            except adm.AdmissibleError as exc:
                ok = False
                lines.append('box REFUSED: %s' % (exc,))
            else:
                # Per (site pair, apex band) -- the same predicate `select`
                # judges a live throw by, not just "a box exists somewhere".
                by_pair: Dict[Tuple[str, str], List[Tuple[float, float]]] = {}
                for b in boxes:
                    by_pair.setdefault(b.site_pair, []).append(b.apex_band_m)
                parts = ['%s %s' % (p, ', '.join(
                            '%.2f-%.2f' % (lo, hi) for lo, hi in sorted(bands)))
                        for p, bands in sorted(by_pair.items())]
                check_apex_m = float(self.get_parameter('apex_m').value)
                pair = (_DEFAULT_SITE_NAME, _DEFAULT_SITE_NAME)
                if adm.select(boxes, pair, check_apex_m) is None:
                    ok = False
                    lines.append(
                        'box REFUSED: no box covers %r at apex %.3f m (%s)'
                        % (pair, check_apex_m, '; '.join(parts)))
                else:
                    lines.append('box OK: %s' % ('; '.join(parts),))
        site_x_mm = float(self.get_parameter('site_x_mm').value)
        site_y_mm = float(self.get_parameter('site_y_mm').value)
        if (abs(site_x_mm - _DEFAULT_SITE_X_MM) > 1e-9
                or abs(site_y_mm - _DEFAULT_SITE_Y_MM) > 1e-9):
            ok = False
            lines.append(
                'site REFUSED: (%.3f, %.3f) mm != the swept default '
                '(%.3f, %.3f) mm — the admissible box carries no site xy'
                % (site_x_mm, site_y_mm, _DEFAULT_SITE_X_MM,
                   _DEFAULT_SITE_Y_MM))
        else:
            lines.append('site OK')
        response.success = ok
        response.message = '; '.join(lines)
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SkillNode()
    # ── MultiThreadedExecutor, not plain spin ─────────────────────────────
    # `_installer` blocks the tick timer in `_wait_future`'s poll loop,
    # waiting for THIS node's own install-client response — a response only
    # the executor can deliver. Under `rclpy.spin` (single-threaded) the same
    # thread that is busy-waiting is the only thread that could ever process
    # it: every install times out, forever (found 2026-09-13 by reading, never
    # exercised live). Two threads is what unblocks it: the client and the
    # tick timer share a `ReentrantCallbackGroup` (`SkillNode.__init__`), and
    # a MultiThreadedExecutor with more than one thread lets the response be
    # processed on a thread OTHER than the one blocked in the tick. The tick
    # itself is guarded against re-entering its own dispatch (`_on_tick`'s
    # `_tick_lock`), so extra threads cannot start a second install.
    # A third thread (finding 4, R3 audit, 2026-09-13) is for the
    # subscriptions' own `MutuallyExclusiveCallbackGroup` (`SkillNode.
    # __init__`'s `_sub_cbgroup`): with only two threads, both could be
    # occupied by the reentrant install-client/tick pair while a blocked
    # service call (`_prelevel`) starves every subscription callback.
    executor = MultiThreadedExecutor(num_threads=3)
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
