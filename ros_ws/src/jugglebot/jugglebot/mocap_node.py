"""Mocap Interface Node — QTM streaming, tf2 broadcast, BB calibration.

Publishes (one message per QTM frame, ~300 Hz, since 2026-10-10: the frame
queue, logbook 2026-10-10-mocap-frame-queue):
  mocap_data             (MocapDataMulti)   — all markers (labelled + unlabelled), per frame
  rigid_body_poses       (RigidBodyPoses)   — all rigid bodies, per frame that carries any
  bb/markers             (MocapDataMulti)   — BB fiducial markers, per frame (when QTM connected)
  bb/calibration_result  (BallButlerCalibrationResult) — latched: the calibration IN
                         FORCE. Only a success replaces it; a failure lands here
                         only while this process has no success yet.
  bb/calibration_attempt (BallButlerCalibrationResult) — latched: the outcome of the
                         most recent sweep, success or failure (one short line)
  qtm_clock_offset_sec   (Float64)          — QTM↔ROS clock offset at 1 Hz
  mocap/status           (DiagnosticStatus) — QTM reception + BB fiducial visibility at 5 Hz

Subscribes:
  bb/heartbeat           (BallButlerHeartbeat) — toggles marker publishing + calibration

Static TF:
  world → platform_start  (Z offset = GEOM_INITIAL_HEIGHT_MM)
"""

from __future__ import annotations

import dataclasses
import datetime
import json
import math
import os
import time

import numpy as np
import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from jugglebot_interfaces.msg import (
    MocapDataMulti,
    MocapDataSingle,
    BallButlerHeartbeat,
    BallButlerCalibrationResult,
    RigidBodyPose,
    RigidBodyPoses,
)
import tf2_ros

from .mocap_interface import FRAME_QUEUE_MAXLEN, MocapInterface
from .calibration_solve_worker import CalibrationSolveWorker
from jugglebot.protocol_config import BallButlerStates
import jugglebot.hardware_config as hw
from jugglebot import mocap_status as mocap_st
from .bb_calibration import (
    run_calibration, CalibrationResult, MIN_ARC_DEG,
    BB_MARKER_COUNT, BB_YAW_ANCHOR_INDEX,
    load_marker_template, assess_sweep_vs_reference, format_outcome_body,
    stamped_yaw_samples_from_joint_state, sweep_suffix,
)
from .robot_frame import (
    RobotFrameTracker, load_robot_frame_template,
)
from .bb_base_frame import (
    BaseFrameMonitor, KappaRecord, append_record, assess_base_frame,
    base_candidate_points, bb_in_base, current_epoch_id,
    estimate_base_pose, kappa_sweep_sd_deg, load_base_frame, load_base_state,
    seed_base_state, write_base_state, epoch_median_at_pin,
)

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - outside a ROS install
    get_package_share_directory = None


#: Cadence of the ``mocap/status`` publisher and of the calibration health
#: check. 5 Hz: fast enough that the consumers' 1.0 s staleness window
#: (``mocap_status.MOCAP_STATUS_MAX_AGE_S``) needs five consecutive missed
#: cycles before they refuse, slow enough to be free next to the per-frame marker
#: path.
MOCAP_STATUS_PERIOD_S = 0.2

#: Period of the executor-thread timer that APPLIES finished calibration
#: solves (state, files, publishes, log lines). The solve itself runs on the
#: ``CalibrationSolveWorker`` thread (logbook
#: 2026-10-16-mocap-calibration-solve-off-executor), so this bounds the
#: result's extra latency: 0-0.1 s after a ~4 s solve, on a static BB.
CALIBRATION_APPLY_PERIOD_S = 0.1

#: Bounded join of the solve thread at node shutdown; an unfinished solve's
#: result is dropped (the thread is a daemon).
CALIBRATION_SOLVE_JOIN_TIMEOUT_S = 1.0

#: Wall-clock cap on one CALIBRATING collection window (Q5c). BB's own sweep is
#: a few seconds; anything past a minute means the heartbeat is wedged at
#: CALIBRATING (or the exit edge was lost), and the old code would have kept
#: accumulating markers into a dict nothing would ever finalize.
CALIBRATION_TIMEOUT_S = 60.0

#: BB body template the yaw offset and position are read against
#: (``bb_calibration.estimate_sweep_yaw_offset``). A basename is looked up next
#: to the source tree's ``resources/`` and in ``share/jugglebot/resources``.
DEFAULT_MARKER_TEMPLATE_FILE = 'bb_marker_template.json'

#: Where the last ACCEPTED calibration is persisted, so the consistency gate
#: survives restarts. Beside the BallButler runner's session directories.
DEFAULT_CALIBRATION_STATE_FILE = os.path.join(
    os.path.expanduser('~'), 'bb_calibration_sessions',
    'bb_calibration_last_accepted.json')

#: Fewest stamped yaw samples (``bb/axis_estimates`` joint ``bb_yaw``) in a
#: calibration window for the estimator to prefer them over the heartbeat.
#: A 0→120→0 sweep at 100 Hz gives ~900; a stream that is absent (bridge
#: without the yaw lane) gives 0.
MIN_STAMPED_YAW_SAMPLES = 100

#: ``bb_yaw_source`` parameter values: which BB yaw the sweep estimator fits.
#: ``heartbeat`` — the 10 Hz bb/heartbeat yaw at its receive time;
#: ``stamped`` — the 100 Hz bb_yaw of bb/axis_estimates at its bridge stamp
#: (refused when fewer than MIN_STAMPED_YAW_SAMPLES arrived); ``auto``
#: (DEFAULT since 2026-10-10 12:26) — the stamped stream when it has enough
#: samples, else the heartbeat (older firmware's two-joint message). The
#: heartbeat was the default while the stamped source's only bag
#: (2026-10-10_00-24-06) had a wandering QTM->ROS clock that scattered BOTH
#: sources ~0.25° (now refused as CONSTELLATION_MOCAP_CLOCK). The first clean
#: sitting with both sources, bag 2026-10-10_12-26-22 (10 sweeps, clock gate
#: 0.0 %), replays at 0.021° SD stamped against 0.038° heartbeat (the
#: heartbeat's receive-time jitter is most of its scatter), meeting the
#: pre-registered rule for the switch (stamped SD <= 0.08° on a clean sitting;
#: logbooks 2026-10-10-bb-yaw-offset-spread-stamped-source and
#: 2026-10-10-bb-base-marker-frame).
BB_YAW_SOURCES = ('heartbeat', 'stamped', 'auto')
DEFAULT_BB_YAW_SOURCE = 'auto'
#: Bound on a published calibration FAILURE text (and so on its ERROR line):
#: one line, the code plus the decisive numbers. The estimator detail goes to
#: DEBUG. ``_publish_calibration_failure`` enforces it (collapses whitespace,
#: truncates with an ellipsis and logs the full text at DEBUG) — a safety net:
#: every reason the node produces fits without truncation
#: (tests/ros/test_mocap_node_keep_last_good.py). The 2026-10-09 live failure
#: line was ~900 characters. The bound covers the whole published text: the
#: sweep-kinematics suffix (``bb_calibration.sweep_suffix``, ~45 chars) is
#: reserved first and the reason truncated to what is left, so the suffix
#: always survives (raised 240 -> 300 when the suffix landed, so no existing
#: reason loses room).
MAX_CALIBRATION_FAILURE_CHARS = 300

#: BB base-marker frame (2026-10-10, ``jugglebot.bb_base_frame``): four fixed
#: markers on BB's shelf, posed by geometry among every marker but BB's own
#: (a QTM label on a base marker does not hide it). The resource
#: holds their as-built coordinates and the seed BB-in-base records; the state
#: file holds the node's running pool (seeded from the resource when absent).
DEFAULT_BASE_FRAME_FILE = 'bb_base_frame.json'
#: The robot-base marker template (robot_frame.py), learned once from the
#: kincal bag and frozen under resources/.
DEFAULT_ROBOT_FRAME_FILE = 'jugglebot_robot_frame.json'
#: The robot-frame fit is fed this often (the shelf base frame's cadence).
ROBOT_FRAME_PERIOD_S = 0.2
#: A new ``robot_frame`` is published when the running frame moved this far
#: from the last published one (latched, so no heartbeat is needed).
ROBOT_FRAME_PUBLISH_MM = 0.2
ROBOT_FRAME_PUBLISH_DEG = 0.005
#: An unchanged running frame is re-published this often (a latched heartbeat):
#: skill_node measures the frame's age from RECEIVE time and WARNs past 10 s,
#: so the age must measure a LIVE fit, not "time since it last moved".
ROBOT_FRAME_REPUBLISH_S = 5.0
DEFAULT_BASE_FRAME_STATE_FILE = os.path.join(
    os.path.expanduser('~'), 'bb_calibration_sessions', 'bb_base_frame_state.json')
#: Cadence (s) of the slow running base-pose estimate fed from the per-frame path.
BASE_MONITOR_PERIOD_S = 0.2
#: A window that does not see the base may use the running estimate if its
#: newest pose is at most this old (s).
BASE_MONITOR_MAX_AGE_S = 30.0

#: Length (s) of the window the frames-lost WARN judges, and so the shortest
#: interval between two WARN lines. Losses are summed over the window and
#: judged once it ends; a window at or below MOCAP_LOSS_WARN_FRACTION is
#: silent (its counts stay in the 1 Hz DEBUG stream line).
MOCAP_LOSS_WARN_PERIOD_S = 10.0
#: WARN only when MORE than this fraction of the window's expected QTM frames
#: was lost (node overflow + frame numbers QTM never sent). Owner's rule
#: 2026-10-11: the healthy 2026-10-10 sitting lost 0.4-0.8 % to sporadic
#: single-frame QTM skips (logbook 2026-10-10-mocap-frame-queue).
MOCAP_LOSS_WARN_FRACTION = 0.05
#: QTM's nominal stream rate, for the WARN's "starved for over x s" figure.
QTM_STREAM_HZ = 300.0


# ── Per-frame message building ───────────────────────────────────────────────
# The publisher builds one MocapDataSingle per marker per frame (~25 at
# ~300 Hz). The generated Foxy classes type-check every field in a Python
# property setter: ~8 µs per marker with the setters, ~1 µs writing the
# generated slots directly (measured on the Jetson, logbook
# 2026-10-10-mocap-frame-queue). _marker_fast writes the slots; it is used
# only when the classes have rosidl's slot layout AND it builds a message
# equal to the checked one (checked once at import), else _marker_checked —
# the old code path — is used (it is, under the tests' message stand-ins).

def _marker_checked(x, y, z, residual, label=''):
    s = MocapDataSingle()
    pos = s.position
    pos.x = float(x)
    pos.y = float(y)
    pos.z = float(z)
    s.residual = float(residual)
    if label:
        s.label = label
    return s


_object_new = object.__new__


def _marker_fast(x, y, z, residual, label=''):
    p = _object_new(Point)
    p._x = float(x)
    p._y = float(y)
    p._z = float(z)
    s = _object_new(MocapDataSingle)
    s._position = p
    s._residual = float(residual)
    s._label = str(label)
    return s


def _marker_fast_is_safe() -> bool:
    try:
        if set(getattr(Point, '__slots__', ())) != {'_x', '_y', '_z'}:
            return False
        if set(getattr(MocapDataSingle, '__slots__', ())) != {'_position', '_residual', '_label'}:
            return False
        for args in ((1.5, -2.25, 3e3, 0.5, 'Ball Butler - 1'),
                     (float('nan'), 0.0, -1.0, 0.0, '')):
            a, b = _marker_checked(*args), _marker_fast(*args)
            if repr(a) != repr(b) or type(a.position) is not type(b.position):
                return False
        return True
    except Exception:
        return False


#: The builder the publisher uses (see above).
MARKER_FAST_PATH = _marker_fast_is_safe()
_make_marker = _marker_fast if MARKER_FAST_PATH else _marker_checked


def _rotation_to_quat(R):
    """Unit quaternion (x, y, z, w), w >= 0, of a proper rotation matrix."""
    R = np.asarray(R, dtype=float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = 2.0 * math.sqrt(tr + 1.0)
        q = ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    elif R[0, 0] >= R[1, 1] and R[0, 0] >= R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q = (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    elif R[1, 1] >= R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q = ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q = ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s)
    q = np.array(q) / np.linalg.norm(q)
    if q[3] < 0:
        q = -q
    return float(q[0]), float(q[1]), float(q[2]), float(q[3])


def _set_stamp(stamp, ns):
    """Write ROS time ``ns`` into a builtin_interfaces/Time field."""
    stamp.sec = int(ns // 1_000_000_000)
    stamp.nanosec = int(ns % 1_000_000_000)

# BB_MARKER_COUNT (``mocap_interface.ball_butler_markers`` is a fixed
# (BB_MARKER_COUNT, 4) array with NaN rows for the ones QTM cannot see this
# frame) and BB_YAW_ANCHOR_INDEX (the marker ``run_calibration`` hard-requires
# for the yaw offset) are single-sourced from bb_calibration.


@dataclasses.dataclass(frozen=True)
class _SweepSnapshot:
    """One ended sweep's solve inputs, COPIED on the executor thread at the
    exit edge (the next sweep's start edge rebinds every ``_calib_*`` list,
    and the collectors append to them). The worker reads only this."""
    calib_data: dict            # marker index -> list of (3,) arrays (lists copied)
    yaw_readings: list
    frames: list
    yaw_samples: list           # the source the estimator fits
    yaw_source: str             # 'stamped' | 'heartbeat'
    suffix_samples: list        # what _attempt_suffix would have read
    template: object            # the body template (immutable after init)
    base_points: list
    base_template_mm: object    # None when the base-frame model is off
    pitch_z_offset_mm: float


@dataclasses.dataclass(frozen=True)
class _SweepSolve:
    """The worker's product: the calibration (or the solver's refusal text)
    and, for a constellation result with a base model, this window's base
    pose (or the estimator's refusal text)."""
    result: object = None       # CalibrationResult | None
    solver_error: str = ''
    base_pose: object = None
    base_error: str = ''


def _solve_sweep(snap: _SweepSnapshot) -> _SweepSolve:
    """The expensive part of a calibration, on the WORKER thread: pure, reads
    only ``snap``. ``run_calibration`` and ``estimate_base_pose`` are looked
    up in this module so tests that patch them here still take effect."""
    try:
        result = run_calibration(
            calibration_data=snap.calib_data,
            yaw_readings_deg=snap.yaw_readings,
            pitch_z_offset_mm=snap.pitch_z_offset_mm,
            marker_frames=snap.frames,
            yaw_samples=snap.yaw_samples,
            template=snap.template,
            yaw_source=snap.yaw_source,
        )
    except ValueError as e:
        return _SweepSolve(solver_error=str(e))
    if result.yaw_method != 'constellation' or snap.base_template_mm is None:
        return _SweepSolve(result=result)
    try:
        return _SweepSolve(result=result,
                           base_pose=estimate_base_pose(snap.base_points,
                                                        snap.base_template_mm))
    except ValueError as e:
        return _SweepSolve(result=result, base_error=str(e))


class MocapNode(Node):
    def __init__(self, *, solve_inline: bool = False):
        """``solve_inline=True`` (tests only) runs the calibration solve
        synchronously at the sweep's exit edge and applies it before the
        heartbeat callback returns -- the pre-2026-10-16 timing, so tests can
        patch the solver around that one call. The live node solves on the
        ``CalibrationSolveWorker`` thread."""
        super().__init__('mocap_node')

        # ── Mocap interface (pure-Python, runs its own asyncio thread) ────
        # Calibration/startup detail is logged at DEBUG (recorded, not shown by
        # launch_console). No .debug( call in this node or MocapInterface sits on
        # a per-frame path.
        self.get_logger().set_level(LoggingSeverity.DEBUG)
        self.mocap = MocapInterface(logger=self.get_logger(), node=self)

        # Platform Z offset comes directly from hardware_config — no service needed.
        platform_z_mm = hw.GEOM_INITIAL_HEIGHT_MM
        self.mocap.set_base_to_platform_offset(platform_z_mm)
        self.mocap.ready_to_publish = True

        # ── Static TF: world → platform_start ─────────────────────────────
        # Re-broadcast with the robot frame applied whenever `robot_frame` changes.
        self._platform_z_mm = platform_z_mm
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._broadcast_platform_start_tf(platform_z_mm)

        # ── Publishers ────────────────────────────────────────────────────
        self.pub_clock_offset = self.create_publisher(Float64, 'qtm_clock_offset_sec', 10)
        self.pub_mocap = self.create_publisher(MocapDataMulti, 'mocap_data', 10)
        self.pub_bb_markers = self.create_publisher(MocapDataMulti, 'bb/markers', 10)
        self.pub_rigid_bodies = self.create_publisher(RigidBodyPoses, 'rigid_body_poses', 10)

        # Latched QoS for calibration result (last value available to late subscribers)
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_calibration = self.create_publisher(
            BallButlerCalibrationResult, 'bb/calibration_result', latched_qos
        )
        # Keep-last-good (2026-10-10): bb/calibration_result is the calibration
        # IN FORCE, so a failed sweep never replaces a previous success there —
        # a consumer that (re)subscribes after the failure gets the success.
        # Every sweep's OUTCOME (success or failure, one short line) goes here,
        # latched too so a reloaded GUI can still show why the last attempt
        # failed. Same type: no interface change, nothing to rebuild.
        self.pub_calibration_attempt = self.create_publisher(
            BallButlerCalibrationResult, 'bb/calibration_attempt', latched_qos
        )
        # Live base-marker frame pose for the magazine throw (skill_node maps the
        # elbow-frame target through it). geometry_msgs/PoseStamped, latched:
        # position = origin (marker C) in WORLD **mm** (same units as
        # rigid_body_poses), orientation = quaternion of R_base_to_world (columns =
        # base x, y, z in world), header.stamp = ROS time of the newest frame in the
        # pose window (so consumers can gate freshness), frame_id 'world'. The
        # template residual has no field in this type and is not published.
        self.pub_base_frame_pose = self.create_publisher(
            PoseStamped, 'bb/base_frame_pose', latched_qos
        )
        # The robot frame (robot_frame.py): Jugglebot's base frame measured from
        # the labelled `Base - n` markers, as a pose in WORLD mm. PoseStamped,
        # latched, header.frame_id 'world', header.stamp = the newest frame time
        # used; position = robot origin in world, orientation = R_robot_to_world.
        # Published on first fit and on change (> 0.2 mm / 0.005 deg).
        self.pub_robot_frame = self.create_publisher(
            PoseStamped, 'robot_frame', latched_qos
        )
        #: The last SUCCESSFUL result this process published (None until one),
        #: and when it was accepted (ISO-8601 UTC, seconds).
        self._last_good_calibration: BallButlerCalibrationResult | None = None
        self._last_good_at = ''

        # Q1 — the ONLY ROS-observable QTM-connection signal. Until this topic
        # existed, `MocapInterface.is_receiving()` never left this process, so
        # every other node (the bridge's bb/calibrate gate, the orchestrator's
        # HOMING step) had to infer QTM health from the *absence* of marker
        # traffic. Deliberately its own topic, not a field on bb/markers
        # (200 Hz — 40x the rosbag cost for a 5-field snapshot) and not on
        # qtm_clock_offset_sec (which goes SILENT on disconnect, i.e. exactly
        # when a consumer needs to be told).
        # NB the topic is a STRING LITERAL here, not mocap_st.MOCAP_STATUS_TOPIC:
        # tools/gen_choreography_map.py resolves endpoint names by AST and
        # deliberately refuses to guess through an attribute expression, so a
        # constant here would land in ros_ws/docs/choreography.md as
        # UNRESOLVED(...) and the cross-node wire would vanish from the map.
        # The map (pinned by tests/ros/test_choreography_map.py) is what keeps
        # the three literals honest with each other.
        self.pub_mocap_status = self.create_publisher(
            DiagnosticStatus, 'mocap/status', 10
        )

        # ── Timers ────────────────────────────────────────────────────────
        self.create_timer(1.0, self._publish_clock_offset)
        # The publisher tick DRAINS the frame queue (every frame since the
        # last tick, one message each), so its period sets latency (0-5 ms,
        # consumers fit against the frames' own QTM stamps), not loss. Kept at
        # 5 ms: a faster tick costs executor wake-ups for at most 2.5 ms less
        # mean latency (logbook 2026-10-10-mocap-frame-queue).
        self.create_timer(hw.TRACKING_MOCAP_DT_S, self._publish_mocap_data)
        # Stream-health state for the 1 Hz line and the frames-lost WARN.
        self._stream_prev: dict | None = None
        self._stream_prev_mono: float | None = None
        self._loss_pending = [0, 0, 0, 0]     # node overflow, QTM gap frames, gap events, frames received
        self._loss_pending_since: float | None = None
        self.create_timer(MOCAP_STATUS_PERIOD_S, self._publish_mocap_status)
        # Q5b/Q5c ride their OWN timer rather than piggy-backing on the status
        # publisher: the publisher must stay a pure snapshot read (determinism
        # doctrine), and this one *acts* — it publishes failures and unlatches
        # collection state.
        self.create_timer(MOCAP_STATUS_PERIOD_S, self._check_calibration_health)
        self.create_timer(1.0, self._publish_base_frame_pose)
        # The calibration solve runs OFF this executor (calibration_solve_worker:
        # the invariant and its four rules); this timer applies its outcomes.
        self._calib_solver = CalibrationSolveWorker(_solve_sweep, inline=solve_inline)
        self.create_timer(CALIBRATION_APPLY_PERIOD_S, self._apply_calibration_outcomes)

        # ── BB heartbeat subscription ─────────────────────────────────────
        self.create_subscription(
            BallButlerHeartbeat, 'bb/heartbeat', self._on_bb_heartbeat, 10
        )
        self._bb_last_state: int | None = None

        # ── Calibration state ─────────────────────────────────────────────
        self._calibrating = False
        self._calib_data: dict[int, list[np.ndarray]] = {}
        self._calib_yaw_readings: list[float] = []
        #: Non-None once the in-flight collection window has been invalidated
        #: (Q5b/Q5c). Latched: sampling stops, the solver is skipped at the
        #: state-exit edge, and it is cleared only when a NEW sweep starts.
        self._calib_invalid: str | None = None
        #: Fence that stops a wedged heartbeat from immediately restarting a
        #: window we just timed out of (Q5c). Cleared when BB reports any state
        #: other than CALIBRATING.
        self._calib_blocked = False
        self._calib_start_mono = 0.0
        #: Timestamped copies for the sweep yaw estimator, all on the ROS
        #: clock: every mocap frame's visible BB points (labels discarded) at
        #: its QTM frame stamp; every heartbeat's yaw at its RECEIVE time (it
        #: carries no stamp — the estimator fits the lag); and, when the
        #: bridge publishes one, the stamped 100 Hz yaw from bb/axis_estimates
        #: at its sample stamp (preferred: no lag to fit beyond ~0).
        self._calib_frames: list = []
        self._calib_last_frame_ns = None
        self._calib_yaw_samples: list = []
        self._calib_stamped_yaw: list = []
        #: Every distinct frame's base-candidate markers in the window (all but
        #: BB's own, labelled or not: bb_base_frame.base_candidate_points).
        self._calib_base_points: list = []
        self._calib_last_base_ns = None
        self.create_subscription(
            JointState, 'bb/axis_estimates', self._on_bb_axis_estimates, 50)

        # ── Body template + advisory history (2026-10-16: nothing here refuses) ──
        self.declare_parameter('bb_marker_template_file', DEFAULT_MARKER_TEMPLATE_FILE)
        self.declare_parameter('bb_calibration_state_file', DEFAULT_CALIBRATION_STATE_FILE)
        self.declare_parameter('bb_yaw_source', DEFAULT_BB_YAW_SOURCE)
        self.declare_parameter('bb_base_frame_file', DEFAULT_BASE_FRAME_FILE)
        self.declare_parameter('bb_base_frame_state_file', DEFAULT_BASE_FRAME_STATE_FILE)
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self._marker_template = None
        self._marker_template_error = ''
        self._load_marker_template(
            str(self.get_parameter('bb_marker_template_file').value))
        self._base_model = None
        self._base_model_error = ''
        self._base_monitor = None
        self._base_monitor_last_t = None
        self._base_reported = False
        self._load_base_frame(str(self.get_parameter('bb_base_frame_file').value))
        self.declare_parameter('robot_frame_file', DEFAULT_ROBOT_FRAME_FILE)
        self._robot_tracker = None
        self._robot_last_t = None            # newest frame time fed (s)
        self._robot_published = None         # last RobotFrame published
        self._robot_pub_t = None             # frame time of that publish (s)
        self._robot_ok_prev = None           # last fit state logged (None = never)
        self._load_robot_frame(str(self.get_parameter('robot_frame_file').value))
        self._log_startup_state()

        self.get_logger().info('Mocap node up (QTM connects in the background)')

    def _on_set_parameters(self, params):
        for p in params:
            if p.name == 'bb_yaw_source' and p.value not in BB_YAW_SOURCES:
                return SetParametersResult(
                    successful=False,
                    reason=f'bb_yaw_source must be one of {", ".join(BB_YAW_SOURCES)}')
        return SetParametersResult(successful=True)

    @staticmethod
    def _resolve_resource(name: str):
        """A resource path: ``name`` itself, the source tree's ``resources/``, or
        the installed ``share/jugglebot/resources``; None if none exists."""
        candidates = [name,
                      os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   os.pardir, 'resources', name)]
        if get_package_share_directory is not None:
            try:
                candidates.append(os.path.join(
                    get_package_share_directory('jugglebot'), 'resources', name))
            except Exception:
                pass
        return next((c for c in candidates if os.path.isfile(c)), None)

    def _load_marker_template(self, name: str):
        """Resolve and load the marker template; on failure record why (the
        calibration then FAILS with that reason — there is no anchor fallback)."""
        path = self._resolve_resource(name)
        if path is None:
            self._marker_template_error = f'BB marker template not found ({name})'
        else:
            try:
                self._marker_template = load_marker_template(path)
            except (OSError, ValueError) as e:
                self._marker_template_error = f'BB marker template unreadable ({path}): {e}'
        if self._marker_template is None:
            self.get_logger().error(
                f'{self._marker_template_error} — BB calibrations will be refused')
        else:
            t = self._marker_template
            self.get_logger().debug(
                f'BB body template: {len(t.points_mm)} markers from {path}, gauge pinned '
                f'{t.pinned_yaw_offset_deg:.3f}° (raw {t.raw_offset_at_pin_deg:.3f}°), '
                f'repeatability {t.repeatability_deg:.3f}°')

    def _load_base_frame(self, name: str):
        """Load the base-marker frame resource. Missing or unreadable: the base
        frame is off (sweep only; ``base_frame`` refuses naming why)."""
        path = self._resolve_resource(name)
        if path is None:
            self._base_model_error = f'BB base frame not found ({name})'
        else:
            try:
                self._base_model = load_base_frame(path)
            except (OSError, ValueError) as e:
                self.get_logger().debug(f'BB base frame unreadable ({path}): {e}')
                self._base_model_error = (f'BB base frame unreadable '
                                          f'({os.path.basename(path)}: {type(e).__name__})')
        if self._base_model is None:
            self.get_logger().warn(f'{self._base_model_error} — base-marker frame off '
                                   '(kappa is not pooled)')
            return
        self._base_monitor = BaseFrameMonitor(self._base_model.template_mm)
        self.get_logger().debug(
            f'BB base frame: as-built template from {path}, '
            f'{len(self._base_model.records)} seed BB-in-base records')

    # ──────────────────────────────────────────────────────────────────────
    #  Publishing
    # ──────────────────────────────────────────────────────────────────────

    def _publish_base_frame_pose(self):
        """``bb/base_frame_pose`` @ 1 Hz (latched): the running base-frame pose.
        Silent while the monitor has no pose. Pure compute on the monitor's
        buffer; no I/O."""
        if self._base_monitor is None:
            return
        try:
            pose = self._base_monitor.pose()
            if pose is None:
                return
            msg = PoseStamped()
            _set_stamp(msg.header.stamp, int(pose.t_span_s[1] * 1e9))
            msg.header.frame_id = 'world'
            msg.pose.position.x = float(pose.origin_mm[0])
            msg.pose.position.y = float(pose.origin_mm[1])
            msg.pose.position.z = float(pose.origin_mm[2])
            qx, qy, qz, qw = _rotation_to_quat(pose.R)
            msg.pose.orientation.x, msg.pose.orientation.y = qx, qy
            msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
            self.pub_base_frame_pose.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing base frame pose: {e}',
                                    throttle_duration_sec=5.0)

    def _load_robot_frame(self, name: str):
        """Load the robot-frame template. Missing or unreadable: ONE error and
        the robot frame stays unknown (nothing else in this node depends on it)."""
        path = self._resolve_resource(name)
        if path is None:
            self.get_logger().error(
                f'Robot frame template not found ({name}) — robot frame UNKNOWN; '
                'skills will refuse (REJECTED_ROBOT_FRAME_UNKNOWN)')
            return
        try:
            template = load_robot_frame_template(path)
        except ValueError as e:
            self.get_logger().error(
                f'Robot frame template unreadable ({path}): {e} — robot frame UNKNOWN')
            return
        self._robot_tracker = RobotFrameTracker(template)
        self.get_logger().debug(
            f'Robot frame: {len(template.labels)}-marker template from {path}')

    def _robot_feed_due(self, frame_ros_ns) -> bool:
        """Whether _feed_robot_frame would fit this frame: once per
        ROBOT_FRAME_PERIOD_S (the per-frame cost is this comparison)."""
        if self._robot_tracker is None or frame_ros_ns is None:
            return False
        return (self._robot_last_t is None
                or frame_ros_ns * 1e-9 - self._robot_last_t >= ROBOT_FRAME_PERIOD_S)

    def _feed_robot_frame(self, labelled, frame_ros_ns):
        """Fit the robot frame to a frame's labelled Base markers (5 Hz), set the
        interface's aligned flag from the fit, log transitions, and publish the
        running frame when it is new or moved."""
        self._robot_last_t = frame_ros_ns * 1e-9
        tr = self._robot_tracker
        tr.feed(labelled, frame_ros_ns)
        st = tr.status()
        # State = the tracker's RUNNING frame (window semantics), not the last
        # single fit: one bad feed among good ones changes nothing.
        cur = tr.current()
        ok = cur is not None
        self.mocap.set_aligned(ok)
        if ok != self._robot_ok_prev:
            first_state = self._robot_ok_prev is None
            self._robot_ok_prev = ok
            if ok:
                yaw = math.degrees(math.atan2(cur.R[1, 0], cur.R[0, 0]))
                dropped = st.get('dropped') or ()
                drop_txt = f' (dropped {", ".join(dropped)})' if dropped else ''
                self.get_logger().info(
                    f'Robot frame OK: rms {st["rms_mm"]:.2f} mm, {st["n_matched"]} markers'
                    f'{drop_txt}, '
                    f'origin ({cur.t[0]:.1f}, {cur.t[1]:.1f}, {cur.t[2]:.1f}) mm, '
                    f'yaw {yaw:.2f} deg')
            elif first_state:
                # Never fitted yet this launch: said once at INFO (a launch with
                # the Base markers out of view is a status fact, not a loss);
                # the skills/check refusal names it to the operator.
                self.get_logger().info(f'Robot frame not seen yet: {st["reason"]}')
            else:
                self.get_logger().warn(f'Robot frame UNKNOWN: {st["reason"]}')
        if cur is None:
            return
        last = self._robot_published
        if (last is None
                or cur.moved_from(last, ROBOT_FRAME_PUBLISH_MM, ROBOT_FRAME_PUBLISH_DEG)
                or self._robot_pub_t is None
                or frame_ros_ns * 1e-9 - self._robot_pub_t >= ROBOT_FRAME_REPUBLISH_S):
            self._publish_robot_frame(cur, frame_ros_ns)

    def _publish_robot_frame(self, frame, stamp_ns):
        msg = PoseStamped()
        _set_stamp(msg.header.stamp, int(frame.stamp_ns if frame.stamp_ns is not None
                                         else stamp_ns))
        msg.header.frame_id = 'world'
        msg.pose.position.x = float(frame.t[0])
        msg.pose.position.y = float(frame.t[1])
        msg.pose.position.z = float(frame.t[2])
        qx, qy, qz, qw = frame.quaternion_xyzw()
        msg.pose.orientation.x, msg.pose.orientation.y = qx, qy
        msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
        self.pub_robot_frame.publish(msg)
        self._robot_published = frame
        self._robot_pub_t = stamp_ns * 1e-9
        self._broadcast_platform_start_tf(self._platform_z_mm, frame)

    def _report_base_frame_once(self):
        """Once per process, when the running estimate first has a pose: one
        INFO line with the base pose and its change vs the stored base pose —
        a change there is QTM's frame moving (BB and the base are bolted down)."""
        if self._base_reported or self._base_monitor is None:
            return
        pose = self._base_monitor.pose()
        if pose is None:
            return
        self._base_reported = True
        state, err = self._load_state()
        ref = state['base_ref']
        note = ''
        if ref is not None:
            d_mm = float(np.linalg.norm(pose.origin_mm - np.asarray(ref['origin_mm'], float)))
            d_deg = (pose.heading_deg - float(ref['heading_deg']) + 180.0) % 360.0 - 180.0
            note = (f' · vs the stored base pose ({ref.get("accepted_at", "?")[:19]}): '
                    f'{d_mm:.2f} mm, {d_deg:+.3f}° (QTM frame shift)')
        elif err:
            note = f' · base state unreadable ({err})'
        self.get_logger().info(
            f'BB base frame seen: heading {pose.heading_deg:.3f}°, origin '
            f'({pose.origin_mm[0]:.1f}, {pose.origin_mm[1]:.1f}, {pose.origin_mm[2]:.1f}) mm, '
            f'tilt {pose.tilt_deg:.2f}°{note}')

    def _check_stream_health(self) -> str:
        """Once a second (the clock-offset timer): read the frame-queue and QTM
        counters, WARN when frames were lost — dropped by this node (queue
        overflow: starved for longer than the queue holds) or never sent by
        QTM (frame-number gaps) — and return the counters for the 1 Hz DEBUG
        line. The WARN is throttled to one per MOCAP_LOSS_WARN_PERIOD_S; losses
        in between are summed into the next one, so none goes unreported."""
        try:
            st = self.mocap.take_stream_stats()
        except Exception as e:
            self.get_logger().error(f'Error reading the mocap stream counters: {e}',
                                    throttle_duration_sec=5.0)
            return ''
        now = time.monotonic()
        prev, prev_mono = self._stream_prev, self._stream_prev_mono
        self._stream_prev, self._stream_prev_mono = st, now
        if prev is None:
            prev = dict.fromkeys(st, 0)

        def d(key):
            return st[key] - prev[key]
        ovf, gap, events = d('overflow_drops'), d('qtm_gap_frames'), d('qtm_gap_events')
        if self._loss_pending_since is None:
            self._loss_pending_since = prev_mono if prev_mono is not None else now
        p = self._loss_pending
        p[0] += ovf
        p[1] += gap
        p[2] += events
        p[3] += d('frames_queued')
        window = now - self._loss_pending_since
        if window >= MOCAP_LOSS_WARN_PERIOD_S:
            # Judge the window: frames expected = received + never sent.
            lost, expected = p[0] + p[1], p[3] + p[1]
            if lost and lost > MOCAP_LOSS_WARN_FRACTION * expected:
                sides = []
                if p[0]:
                    sides.append(f'{p[0]} dropped by mocap_node (frame queue full: '
                                 f'starved for over {FRAME_QUEUE_MAXLEN / QTM_STREAM_HZ:.1f} s)')
                if p[1]:
                    sides.append(f'{p[1]} never received from QTM ({p[2]} frame-number gaps)')
                advice = []
                if p[0]:
                    advice.append('the Jetson was starved: check its load (another job running?)')
                if p[1]:
                    advice.append("check QTM's real-time output (frame-drop / out-of-sync "
                                  "indicator, QTM PC load)")
                self.get_logger().warn(
                    f'mocap: {lost} of {expected} expected QTM frames lost '
                    f'({100.0 * lost / expected:.1f} %) in the last {window:.0f} s — '
                    + ', '.join(sides) + f'; QTM 2D drop {st["max_drop_rate"]}‰ — '
                    + '; '.join(advice))
            self._loss_pending = [0, 0, 0, 0]
            self._loss_pending_since = now
        span = (now - prev_mono) if prev_mono is not None else 0.0
        rate = d('frames_queued') / span if span > 0 else 0.0
        return ('stream {:.0f} frames/s, queue max {}/{}, lost: node overflow {} (total {}), '
                'QTM gaps {} frames (total {}), duplicates {}, QTM restarts {}, '
                'QTM 2D drop {}‰, out of sync {}‰').format(
                    rate, st['max_queue_depth'], FRAME_QUEUE_MAXLEN, ovf, st['overflow_drops'],
                    gap, st['qtm_gap_frames'], d('duplicate_frames'), st['qtm_restarts'],
                    st['max_drop_rate'], st['max_out_of_sync_rate'])

    def _publish_clock_offset(self):
        self._report_base_frame_once()
        stream = self._check_stream_health()
        status = self.mocap.get_qtm_sync_status()
        offset = status.get('offset_s')
        if offset is not None:
            msg = Float64()
            msg.data = offset
            self.pub_clock_offset.publish(msg)
            # Clock-sync and stream health at 1 Hz, DEBUG only: mocap/status's
            # key set is a pinned contract (mocap_status.py), so the
            # estimator's diagnostics and the frame counters ride the log
            # rather than new KeyValues.
            if 'window_fill' in status:
                self.get_logger().debug(
                    'qtm clock sync: excess latency {:.2f} ms, envelope-output {:+.3f} ms, '
                    'window fill {:.2f}, drift {:+.1f} ppm, slew clamps {}, outlier clips {}, '
                    're-anchors {}, restarts {}{}'.format(
                        status['last_excess_latency_ms'], status['envelope_minus_output_ms'],
                        status['window_fill'], status['drift_ppm'], status['slew_clamps'],
                        status['outlier_clips'], status['reanchors'], status['restarts'],
                        f'; {stream}' if stream else ''))

    def _bb_marker_visibility(self) -> tuple[int, bool]:
        """(count of BB fiducials QTM currently resolves, yaw-anchor visible).

        ``MocapInterface.ball_butler_markers`` is a persistent (7, 4) array
        rewritten every packet — visible markers get positions, the rest get
        NaN rows — and ``get_ball_butler_markers_base_frame()`` hands back a
        copy taken under ``data_lock``. So this is a snapshot read with no I/O.

        Caveat worth knowing at the consumer: these counts go stale if QTM
        stalls WITHOUT the disconnect callback firing (only ``_on_qtm_disconnect``
        re-NaNs the array). That is precisely why ``qtm_receiving`` — which is
        time-based — is the primary gate and the counts are the secondary one.
        """
        markers = self.mocap.get_ball_butler_markers_base_frame()
        if markers is None or markers.shape[0] == 0:
            return 0, False
        visible = ~np.isnan(markers[:, :3]).any(axis=1)
        count = int(np.count_nonzero(visible))
        marker3 = bool(visible[BB_YAW_ANCHOR_INDEX]) if visible.shape[0] > BB_YAW_ANCHOR_INDEX else False
        return count, marker3

    def _publish_mocap_status(self):
        """``mocap/status`` @ 5 Hz — the QTM view other nodes gate on (Q1).

        Pure snapshot read of caches the QTM asyncio thread already maintains:
        no network, no file, no blocking call, so this never stalls the
        executor (determinism doctrine — no blocking I/O in a periodic
        callback).

        It also GATES NOTHING. ``_publish_mocap_data``'s own ``is_receiving()``
        early-return is untouched and no existing publication depends on this
        topic, so a fault in here can only cost observability, never markers.
        """
        try:
            receiving = bool(self.mocap.is_receiving())
            count, marker3 = self._bb_marker_visibility()
            aligned = bool(self.mocap.is_aligned)
            rf = self._robot_frame_status_values()
            synced = bool(self.mocap.get_qtm_sync_status().get('synced', False))

            msg = DiagnosticStatus()
            msg.name = 'mocap/qtm'
            msg.hardware_id = 'qtm'
            msg.level = DiagnosticStatus.OK if receiving else DiagnosticStatus.ERROR
            msg.message = ('QTM streaming' if receiving
                           else 'No QTM packets within the reception window')
            # Keys come from jugglebot.mocap_status — the same module the two
            # consumers read them back with, so a rename cannot leave a gate
            # silently reading a key nobody publishes (which evaluates as
            # "not ready" forever, with no error anywhere).
            msg.values = [
                KeyValue(key=mocap_st.KEY_QTM_RECEIVING,
                         value='1' if receiving else '0'),
                KeyValue(key=mocap_st.KEY_BB_MARKERS_VISIBLE, value=str(count)),
                KeyValue(key=mocap_st.KEY_MARKER3_VISIBLE,
                         value='1' if marker3 else '0'),
                KeyValue(key=mocap_st.KEY_ALIGNED, value='1' if aligned else '0'),
                KeyValue(key=mocap_st.KEY_QTM_SYNCED, value='1' if synced else '0'),
            ] + rf
            self.pub_mocap_status.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing mocap status: {e}',
                                    throttle_duration_sec=5.0)

    def _robot_frame_status_values(self):
        """The robot-frame KeyValues of mocap/status (a snapshot of the tracker)."""
        tr = self._robot_tracker
        if tr is None:
            st = {'ok': False, 'rms_mm': None, 'n_matched': 0,
                  'reason': 'NO_TEMPLATE', 'age_s': None}
        else:
            try:
                now_ns = int(self.get_clock().now().nanoseconds)
            except Exception:
                now_ns = None
            st = tr.status(now_ns)
            st['ok'] = tr.current() is not None     # same meaning as the pill
        return [
            KeyValue(key=mocap_st.KEY_ROBOT_FRAME_OK, value='1' if st['ok'] else '0'),
            KeyValue(key=mocap_st.KEY_ROBOT_FRAME_RMS_MM,
                     value='' if st['rms_mm'] is None else f'{st["rms_mm"]:.3f}'),
            KeyValue(key=mocap_st.KEY_ROBOT_FRAME_N_MATCHED, value=str(int(st['n_matched']))),
            KeyValue(key=mocap_st.KEY_ROBOT_FRAME_AGE_S,
                     value='' if st['age_s'] is None else f'{st["age_s"]:.2f}'),
            KeyValue(key=mocap_st.KEY_ROBOT_FRAME_REASON, value=str(st['reason'])),
        ]

    def _check_calibration_health(self):
        """Invalidate a collection window that has gone bad (Q5b / Q5c).

        Two failure modes, two named codes, both fail-CLOSED — a refusal is
        always safer than a plausible-looking BB pose, because
        ``ball_butler_node`` aims every subsequent throw with whatever
        ``bb/calibration_result`` last carried (a refusal leaves the last
        good calibration in force, or none if there never was one):

        * ``QTM_DROPOUT_MID_SWEEP`` — QTM went dark part-way through. The arc
          the solver sees is a fragment; ``min_points=50`` at 200 Hz means
          0.25 s of data is enough for it to fit circles and publish a pose.
          The arc-span floor in ``bb_calibration`` catches most of these after
          the fact, but only this check can name the CAUSE.
        * ``CALIBRATION_TIMEOUT`` — the heartbeat never left CALIBRATING, so
          the state-exit edge that finalizes never arrives. Before this, the
          node accumulated markers forever with no result and no complaint.

        ORDER MATTERS, and not the way it first looks. The timeout is checked
        BEFORE the "already invalidated, nothing left to do" early return, so
        that it applies to invalidated windows too. Check the dropout first and
        a sweep that loses QTM and *then* wedges at CALIBRATING is latched
        invalid, skips the timeout branch forever, and leaves ``_calibrating``
        True for the life of the process — which blocks every later sweep,
        because the start edge requires ``not self._calibrating``. The timeout
        is the only thing that guarantees a window always closes.
        """
        if not self._calibrating:
            return

        elapsed = time.monotonic() - self._calib_start_mono
        if elapsed > CALIBRATION_TIMEOUT_S:
            if self._calib_invalid is None:
                self._invalidate_calibration(
                    'CALIBRATION_TIMEOUT',
                    f'BB never left CALIBRATING after {elapsed:.0f} s '
                    f'(cap {CALIBRATION_TIMEOUT_S:.0f} s)')
            else:
                # A failure with a MORE specific cause is already latched (and
                # already published). Close the window, but do NOT publish
                # again: bb/calibration_attempt (and, with no success yet,
                # bb/calibration_result) is latched, so a second publish would
                # overwrite 'QTM_DROPOUT_MID_SWEEP' — the thing the operator
                # actually needs to read — with the vaguer timeout.
                self.get_logger().warn(
                    f'Calibration window closed on the {CALIBRATION_TIMEOUT_S:.0f} s '
                    f'cap; already invalidated ({self._calib_invalid})')
            # There is no exit edge coming, so the window is closed here — with
            # the fence up, so a heartbeat still frozen at CALIBRATING cannot
            # immediately restart it.
            self._end_calibration(blocked=True)
            return

        if self._calib_invalid is not None:
            return

        if not self.mocap.is_receiving():
            self._invalidate_calibration(
                'QTM_DROPOUT_MID_SWEEP',
                'QTM stopped delivering frames mid-sweep — the arc has a hole')

    def _invalidate_calibration(self, code: str, detail: str):
        """Latch a collection window invalid and publish the named failure."""
        self._calib_invalid = code
        message = f'{code}: {detail}'
        self.get_logger().debug(f'Calibration invalidated — {message}')
        self._publish_calibration_failure(message)

    def _end_calibration(self, *, blocked: bool = False):
        """Close the collection window and drop its data."""
        self._calibrating = False
        self._calib_blocked = blocked
        self._calib_data = {}
        self._calib_yaw_readings = []
        self._calib_frames = []
        self._calib_last_frame_ns = None
        self._calib_yaw_samples = []
        self._calib_stamped_yaw = []
        self._calib_base_points = []
        self._calib_last_base_ns = None

    def _feed_base_frame(self, points, frame_ros_ns):
        """A frame's base-candidate markers ((n, ≥3): every marker but BB's
        own, ``base_candidate_points``) → the calibration window (every
        distinct frame) and the slow running base estimate (every
        BASE_MONITOR_PERIOD_S)."""
        if points is None or frame_ros_ns is None or points.shape[0] == 0:
            return
        t = frame_ros_ns * 1e-9
        if (self._calibrating and self._calib_invalid is None
                and frame_ros_ns != self._calib_last_base_ns):
            self._calib_last_base_ns = frame_ros_ns
            self._calib_base_points.append((t, np.array(points[:, :3], dtype=float)))
        if self._base_monitor is not None and (
                self._base_monitor_last_t is None
                or t - self._base_monitor_last_t >= BASE_MONITOR_PERIOD_S):
            self._base_monitor_last_t = t
            self._base_monitor.update(t, np.array(points[:, :3], dtype=float))

    def _base_feed_due(self, frame_ros_ns) -> bool:
        """Whether _feed_base_frame would use this frame (the same conditions),
        so the per-frame base-candidate array is built only when it will be:
        every frame of a calibration window, else once per BASE_MONITOR_PERIOD_S."""
        if frame_ros_ns is None:
            return False
        if (self._calibrating and self._calib_invalid is None
                and frame_ros_ns != self._calib_last_base_ns):
            return True
        return self._base_monitor is not None and (
            self._base_monitor_last_t is None
            or frame_ros_ns * 1e-9 - self._base_monitor_last_t >= BASE_MONITOR_PERIOD_S)

    def _publish_mocap_data(self):
        """Publisher tick: drain the frame queue and publish EVERY frame that
        arrived since the last tick, oldest first, each with its own stamp
        (logbook 2026-10-10-mocap-frame-queue). An empty tick publishes
        nothing — until 2026-10-10 it re-published the last frame with an
        empty marker list (a third of /mocap_data in the loaded bags) and a
        stamp re-converted through the moved offset, which is what made the
        stamps step backwards a few µs 164-490 times per bag."""
        # ── Stop publishing when QTM packets aren't arriving ──
        # The GUI has its own 2-second timeout that sets disconnected/unaligned
        # and clears markers, so going silent here is the correct behaviour.
        if not self.mocap.is_receiving():
            return
        for frame in self.mocap.drain_frames():
            self._publish_frame(frame)

    def _publish_frame(self, frame):
        """One QTM frame (``mocap_interface.MocapFrame``) onto mocap_data,
        rigid_body_poses and bb/markers, and into the base-frame feed and the
        calibration window."""
        frame_ros_ns = frame.stamp_ns
        labelled = frame.labelled
        unlabelled = frame.unlabelled
        make = _make_marker
        try:
            msg = MocapDataMulti()
            msg.aligned = bool(frame.aligned)
            if frame_ros_ns is not None:
                _set_stamp(msg.stamp, frame_ros_ns)

            markers = msg.markers
            # Labelled markers (with their QTM label)
            for label, x, y, z, residual in labelled:
                markers.append(make(x, y, z, residual, label))
            # Unlabelled markers (label left as empty string)
            for x, y, z, residual in unlabelled:
                markers.append(make(x, y, z, residual))

            self.pub_mocap.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing markers: {e}',
                                    throttle_duration_sec=5.0)
        try:
            # Labelled markers too: a QTM rigid body that grabs a base marker
            # (2026-10-10 14:10: Base / Catching Cone took two of the four)
            # must not hide the base. Only BB's own are left out.
            if self._base_feed_due(frame_ros_ns):
                self._feed_base_frame(base_candidate_points(unlabelled, labelled), frame_ros_ns)
        except Exception as e:
            self.get_logger().error(f'Error tracking the BB base frame: {e}',
                                    throttle_duration_sec=5.0)

        try:
            if self._robot_feed_due(frame_ros_ns):
                self._feed_robot_frame(labelled, frame_ros_ns)
        except Exception as e:
            self.get_logger().error(f'Error fitting the robot frame: {e}',
                                    throttle_duration_sec=5.0)

        # ── Rigid bodies ──────────────────────────────────────────────
        # Publishing is intentionally NOT gated on the robot frame. Gating here
        # would suppress /rigid_body_poses entirely — including the
        # Catching_Cone target the thrower needs — and silently block every
        # throw. The fit's state is surfaced via MocapDataMulti.aligned (= robot
        # frame known) and mocap/status; it is informational, not a gate here.
        # Body poses stay in QTM WORLD coordinates; consumers apply `robot_frame`.
        # Stamps as before the frame queue: the message at publish time, each
        # pose at its frame's receive time.
        try:
            if frame.bodies:
                msg = RigidBodyPoses()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'world'
                receive_ns = frame.receive_ns
                for name, frame_id, x, y, z, qx, qy, qz, qw in frame.bodies:
                    body_msg = RigidBodyPose()
                    body_msg.name = name
                    # Fill the PoseStamped the constructor already built (a
                    # second one per body per frame was ~1/5 of this path)
                    pose = body_msg.pose
                    if receive_ns is not None:
                        _set_stamp(pose.header.stamp, receive_ns)
                    pose.header.frame_id = frame_id
                    position = pose.pose.position
                    position.x = float(x)
                    position.y = float(y)
                    position.z = float(z)
                    orientation = pose.pose.orientation
                    orientation.x = float(qx)
                    orientation.y = float(qy)
                    orientation.z = float(qz)
                    orientation.w = float(qw)
                    msg.bodies.append(body_msg)
                self.pub_rigid_bodies.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing body poses: {e}',
                                    throttle_duration_sec=5.0)

        # ── BB markers (always published; accumulated during calibration) ──
        try:
            msg = MocapDataMulti()
            if frame_ros_ns is not None:
                _set_stamp(msg.stamp, frame_ros_ns)
            markers = msg.markers
            for x, y, z, residual in frame.bb_markers:
                markers.append(make(x, y, z, residual))
            self.pub_bb_markers.publish(msg)

            # Accumulate for calibration. An invalidated window (Q5b/Q5c)
            # stops sampling: more points cannot repair an arc with a hole
            # in it, and a growing dict would only make the garbage look
            # better-supported.
            if self._calibrating and self._calib_invalid is None:
                self._accumulate_calibration_markers(msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing BB markers: {e}',
                                    throttle_duration_sec=5.0)

    # ──────────────────────────────────────────────────────────────────────
    #  Static TF
    # ──────────────────────────────────────────────────────────────────────

    def _broadcast_platform_start_tf(self, z_offset_mm: float, frame=None):
        """world -> platform_start: the robot frame (R, t) applied to the
        point (0, 0, z_offset); the identity version before any fit.

        For VISUALISATION only. Poses on ``rigid_body_poses`` are published in
        WORLD xy (z shifted) under frame_id 'platform_start'; this TF
        (R @ (0, 0, z0) + t, rotation R) must NOT be applied to those poses by
        a tf2 consumer (none exists today)."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'platform_start'
        if frame is None:
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = z_offset_mm
            t.transform.rotation.w = 1.0
        else:
            p = frame.to_world([0.0, 0.0, z_offset_mm])
            t.transform.translation.x = float(p[0])
            t.transform.translation.y = float(p[1])
            t.transform.translation.z = float(p[2])
            qx, qy, qz, qw = frame.quaternion_xyzw()
            t.transform.rotation.x, t.transform.rotation.y = qx, qy
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().debug(
            f'Broadcast static tf: world -> platform_start (z_offset={z_offset_mm:.1f} mm'
            f'{"" if frame is None else ", robot frame applied"})'
        )

    # ──────────────────────────────────────────────────────────────────────
    #  Ball Butler heartbeat — calibration state tracking
    # ──────────────────────────────────────────────────────────────────────

    def _on_bb_heartbeat(self, msg: BallButlerHeartbeat):
        previous = self._bb_last_state

        if msg.state != self._bb_last_state:
            self._bb_last_state = msg.state

        # Any state other than CALIBRATING means the wedge cleared — drop the
        # post-timeout fence so the next genuine sweep is collected (Q5c).
        if msg.state != BallButlerStates.CALIBRATING:
            self._calib_blocked = False

        # Detect calibration start
        if (msg.state == BallButlerStates.CALIBRATING
                and not self._calibrating and not self._calib_blocked):
            self._calibrating = True
            self._calib_invalid = None
            self._calib_start_mono = time.monotonic()
            self._calib_data = {i: [] for i in range(BB_MARKER_COUNT)}
            self._calib_yaw_readings = []
            self._calib_frames = []
            self._calib_last_frame_ns = None
            self._calib_yaw_samples = []
            self._calib_stamped_yaw = []
            self._calib_base_points = []
            self._calib_last_base_ns = None
            self.get_logger().info('BB calibration started, collecting marker data')

        # Record yaw during calibration for offset calculation — AFTER the
        # start edge, so the first CALIBRATING heartbeat's yaw is in the
        # window (it was dropped while this ran first; bag 2026-10-09_23-49-07),
        # and before the end edge, so the first post-sweep sample still is.
        if self._calibrating and self._calib_invalid is None:
            self._calib_yaw_readings.append(msg.yaw_deg)
            self._calib_yaw_samples.append(
                (self.get_clock().now().nanoseconds * 1e-9, float(msg.yaw_deg)))

        # Detect calibration end (state transition away from CALIBRATING)
        if previous == BallButlerStates.CALIBRATING and msg.state != BallButlerStates.CALIBRATING:
            if self._calibrating:
                if self._calib_invalid is not None:
                    # The failure was already published, with its cause named,
                    # at the moment it happened — running the solver now would
                    # only risk overwriting that with a plausible pose.
                    self.get_logger().warn(
                        f'Calibration sweep ended but the window was already '
                        f'invalidated ({self._calib_invalid}) — solver skipped')
                elif msg.state == BallButlerStates.ERROR:
                    self.get_logger().debug('Calibration aborted: BB entered ERROR state')
                    self._publish_calibration_failure('BB entered ERROR state during calibration')
                else:
                    self.get_logger().debug(
                        f'Calibration sweep complete (new state: {msg.state}). Processing...'
                    )
                    self._finalize_calibration()
                self._end_calibration()

    # ──────────────────────────────────────────────────────────────────────
    #  Calibration processing
    # ──────────────────────────────────────────────────────────────────────

    def _on_bb_axis_estimates(self, msg: JointState):
        """Collect the stamped yaw (if the bridge publishes one) during a sweep."""
        if self._calibrating and self._calib_invalid is None:
            self._calib_stamped_yaw.extend(stamped_yaw_samples_from_joint_state(msg))

    def _accumulate_calibration_markers(self, msg: MocapDataMulti):
        """Store each marker position from a bb/markers message: per label for
        the sweep's arc fit, and as one unlabelled point set per QTM frame,
        at the frame's QTM stamp, for the sweep yaw estimator. Since the frame
        queue (2026-10-10) the publisher sends each QTM frame once; the
        one-point-set-per-distinct-stamp rule stays as the guard it was when a
        latest-frame snapshot could be published twice. A message without a
        stamp (QTM clock sync not yet established) feeds the arc fit only."""
        frame = []
        for i, marker in enumerate(msg.markers):
            if i >= BB_MARKER_COUNT:
                break
            pos = marker.position
            if math.isnan(pos.x) or math.isnan(pos.y) or math.isnan(pos.z):
                continue
            p = np.array([pos.x, pos.y, pos.z])
            self._calib_data[i].append(p)
            frame.append(p)
        stamp_ns = msg.stamp.sec * 1_000_000_000 + msg.stamp.nanosec
        if frame and stamp_ns > 0 and stamp_ns != self._calib_last_frame_ns:
            self._calib_last_frame_ns = stamp_ns
            self._calib_frames.append((stamp_ns * 1e-9, np.array(frame)))

    def _finalize_calibration(self):
        """Exit edge of a valid sweep, on the EXECUTOR thread: the cheap
        refusals, then a COPY of the window's inputs handed to the solve
        worker. Returns at once; ``_apply_calibration_outcomes`` publishes the
        result (logbook 2026-10-16-mocap-calibration-solve-off-executor)."""
        total_pts = sum(len(v) for v in self._calib_data.values())
        if total_pts == 0:
            self._publish_calibration_failure('No marker data collected')
            return

        for idx, pts in self._calib_data.items():
            if pts:
                self.get_logger().debug(f'Marker {idx + 1}: {len(pts)} valid samples')

        if self._marker_template is None:
            # No anchor fallback: the single-marker yaw offset is what misaimed
            # session B by 0.47° (2026-10-09). Refuse, naming the cause.
            self._publish_calibration_failure(
                f'{self._marker_template_error} — cannot estimate the yaw offset')
            return

        # Yaw source per the bb_yaw_source parameter (BB_YAW_SOURCES): the
        # heartbeat by default; the stamped 100 Hz yaw (bb/axis_estimates)
        # when asked for, or under 'auto' when the bridge provides it.
        requested = str(self.get_parameter('bb_yaw_source').value)
        have_stamped = len(self._calib_stamped_yaw) >= MIN_STAMPED_YAW_SAMPLES
        if requested not in BB_YAW_SOURCES:
            self._publish_calibration_failure(
                f'BB_YAW_SOURCE_INVALID: bb_yaw_source {requested!r} is not one of '
                f'{", ".join(BB_YAW_SOURCES)}')
            return
        if requested == 'stamped' and not have_stamped:
            self._publish_calibration_failure(
                f'BB_YAW_SOURCE_UNAVAILABLE: bb_yaw_source is stamped but only '
                f'{len(self._calib_stamped_yaw)} stamped yaw samples arrived (need '
                f'{MIN_STAMPED_YAW_SAMPLES}; BB firmware 6 + can-bridge firmware 28) — set '
                'bb_yaw_source:=heartbeat or auto')
            return
        if requested == 'stamped' or (requested == 'auto' and have_stamped):
            yaw_samples, yaw_source = self._calib_stamped_yaw, 'stamped'
        else:
            yaw_samples, yaw_source = self._calib_yaw_samples, 'heartbeat'
        suffix_samples = (self._calib_stamped_yaw
                          if requested != 'heartbeat' and have_stamped
                          else self._calib_yaw_samples)
        snap = _SweepSnapshot(
            calib_data={i: list(v) for i, v in self._calib_data.items()},
            yaw_readings=list(self._calib_yaw_readings),
            frames=list(self._calib_frames),
            yaw_samples=list(yaw_samples),
            yaw_source=yaw_source,
            suffix_samples=list(suffix_samples),
            template=self._marker_template,
            base_points=list(self._calib_base_points),
            base_template_mm=(None if self._base_model is None
                              else self._base_model.template_mm),
            pitch_z_offset_mm=hw.BB_GEOM_PITCH_Z_OFFSET_MM,
        )
        if not self._calib_solver.submit(snap):
            return                              # shutting down: dropped (rule 2)
        if self._calib_solver.inline:
            self._apply_calibration_outcomes()

    def _apply_calibration_outcomes(self):
        """Executor-thread timer: apply every finished solve, oldest first,
        exactly once (``CalibrationSolveWorker.drain`` hands each out once)."""
        for outcome in self._calib_solver.drain():
            try:
                self._apply_one_outcome(outcome)
            except Exception as exc:
                # One bad outcome must not lose the rest of the drained batch.
                self.get_logger().error(
                    f'BB calibration apply failed: {type(exc).__name__}: {exc}')

    def _apply_one_outcome(self, outcome):
        snap = outcome.job
        if outcome.error is not None:
            # Rule 3: the solve raised. One line, nothing persisted, the
            # calibration in force kept; the worker and the stream live on.
            self._publish_calibration_failure(
                f'CALIBRATION_SOLVE_ERROR: {outcome.error}',
                suffix=sweep_suffix(snap.suffix_samples, None))
            return
        self.get_logger().debug(
            f'Calibration solve took {outcome.solve_s:.2f} s off the executor')
        self._apply_sweep_solve(snap, outcome.value)

    def _apply_sweep_solve(self, snap: _SweepSnapshot, solve: _SweepSolve):
        """Everything that followed the solve when it ran inline, in the same
        order, reading the sweep's inputs from ``snap`` (never ``_calib_*``:
        a new sweep may already be collecting)."""
        if solve.result is None:
            self.get_logger().debug(
                f'Calibration solver rejected the sweep: {solve.solver_error}')
            self._publish_calibration_failure(
                solve.solver_error, suffix=sweep_suffix(snap.suffix_samples, None))
            return
        result: CalibrationResult = solve.result
        lag_s = (None if result.yaw_estimate is None
                 else float(result.yaw_estimate.lag_s))
        suffix = sweep_suffix(snap.suffix_samples, lag_s)
        if result.yaw_method != 'constellation':  # defensive: never publish the anchor value
            self._publish_calibration_failure(
                f'internal: yaw offset came from the {result.yaw_method!r} estimator',
                suffix=suffix)
            return

        # ── Base-marker frame + history: ADVISORY only (nothing below refuses) ──
        # The sweep's own yaw offset and axis point are ALWAYS the published
        # output. The base frame yields kappa for the epoch bookkeeping; the
        # last accepted calibration yields the yaw/axis deltas. A flag is a
        # line on the outcome (and WARN severity), never a refusal.
        yaw_deg = math.degrees(result.yaw_offset_rad)
        est = result.yaw_estimate
        arc = result.arc_position_mm
        arc_note = ('' if arc is None else
                    f'arc-fit cross-check ({arc[0]:.2f}, {arc[1]:.2f}, {arc[2]:.2f}) mm, '
                    f'{float(np.linalg.norm(np.asarray(arc[:2]) - result.bb_position_mm[:2])):.2f} mm '
                    'from the body-model axis point in x/y')
        view = self._base_frame_view(result, snap, solve)
        summary = f'{est.summary()}; {arc_note}'
        if view['pose'] is not None:
            summary += (f'; {view["pose"].summary()}; BB-in-base κ {view["kappa"]:+.4f}°, '
                        f'p_b ({view["p_b"][0]:.2f}, {view["p_b"][1]:.2f}, {view["p_b"][2]:.2f}) mm')
        elif view['error']:
            summary += f'; base frame: {view["error"]}'
        anchor = ('n/a' if result.anchor_yaw_offset_rad is None
                  else f'{math.degrees(result.anchor_yaw_offset_rad):+.3f}°')
        self.get_logger().debug(
            f'Yaw offset (sweep) {yaw_deg:+.4f}° — {summary}; retired anchor estimator '
            f'on the same sweep {anchor}')

        node_flags: list = []
        node_lines: list = []
        state, state_error = view['state'], view['state_error']
        if state_error:
            node_flags.append('HISTORY_UNAVAILABLE')
            node_lines.append(f'history unavailable: {state_error}')
        base_adv = None
        pooled_info = None
        pooled = False
        if view['pose'] is not None:
            base_adv = assess_base_frame(
                view['kappa'], kappa_sweep_sd_deg(est.yaw_source), view['p_b'],
                view['pose'], state, self._marker_template.pinned_yaw_offset_deg)
            pooled = base_adv.pooled
        else:
            node_flags.append('BASE_NOT_SEEN')
            node_lines.append('shelf base not seen: kappa not pooled')
        reference, _ref_label, ref_error = self._gate_reference()
        if ref_error:
            node_flags.append('HISTORY_UNAVAILABLE')
            node_lines.append(f'history unavailable: {ref_error}')
        sweep_adv = assess_sweep_vs_reference(
            yaw_deg, result.yaw_offset_std_deg, result.bb_position_mm,
            est.template_residual_mm, est.lag_s * 1e3,
            None if ref_error else reference,
            None if base_adv is None else base_adv.same_world_epoch,
            yaw_source=snap.yaw_source)
        if reference is None and not ref_error:
            self.get_logger().warn(
                'BB calibration: no reference calibration exists (no state file) — this '
                'one is accepted unchecked and becomes the reference. Validate it against '
                'landings before trusting the aim')
        if pooled:
            asg = base_adv.assignment
            new_state = append_record(state, self._kappa_record(view, result), asg,
                                    self._marker_template.pinned_yaw_offset_deg)
            ref = {'origin_mm': [float(v) for v in view['pose'].origin_mm],
                   'heading_deg': float(view['pose'].heading_deg),
                   'accepted_at': datetime.datetime.now(datetime.timezone.utc).isoformat()}
            new_state = dict(new_state, base_ref=ref)
            self._write_state(new_state)
            ep = next(e for e in new_state['epochs'] if e['id'] == asg.epoch_id)
            pooled_info = {'kappa_deg': ep['median_kappa_deg'], 'epoch_id': asg.epoch_id,
                           'delta_deg': (0.0 if base_adv.delta_kappa_deg is None
                                         else base_adv.delta_kappa_deg),
                           'n': ep['n'], 'opened': asg.opened, 'rejoined': asg.rejoined}
        else:
            if view['pose'] is not None:       # BASE_FRAME_RESIDUAL: judged nothing
                self.get_logger().debug('Base frame residual too high: kappa not pooled')
            # Nothing to pool, but a corrupt/missing file is replaced by a fresh
            # one (the library backs the old one up) so the next sweep has history.
            if state_error:
                self._write_state(state)
        advisory_flags = (list(sweep_adv.flags) + list(node_flags)
                          + (list(base_adv.flags) if base_adv is not None else []))
        sweep_adv = dataclasses.replace(
            sweep_adv, lines=list(sweep_adv.lines) + node_lines)
        body = format_outcome_body(
            yaw_deg, result.yaw_offset_std_deg, result.bb_position_mm, result.axis_tilt_deg,
            est.template_residual_mm, est.lag_s * 1e3, sweep_adv, base_adv, pooled_info)
        accepted_at = self._persist_accepted(result, body, extra={
            'pose_source': 'sweep',
            'sweep_yaw_offset_deg': yaw_deg,
            'sweep_position_mm': [float(v) for v in result.bb_position_mm],
            'base_frame': None if view['pose'] is None else view['pose'].to_dict(),
            'kappa_deg': view['kappa'] if pooled else None,
            'epoch': None if pooled_info is None else pooled_info['epoch_id'],
            'advisory_flags': advisory_flags,
        })
        outcasts = ''.join(
            f' · outcast Marker {idx + 1} ({m.distance_from_axis_mm:.2f} mm off axis)'
            for idx, m in sorted(result.marker_metrics.items())
            if m.status == 'outcast')
        result_message = (f'Calibration successful (accepted {accepted_at}) · {body}{outcasts}')
        outcome = f'BB calibrated: {body}{outcasts}{suffix}'
        # One severity per call site (Foxy rclpy raises ValueError otherwise).
        if advisory_flags:
            self.get_logger().warn(outcome)
        else:
            self.get_logger().info(outcome)
        self.get_logger().debug(
            f'Axis direction: ({result.axis_direction[0]:.4f}, '
            f'{result.axis_direction[1]:.4f}, {result.axis_direction[2]:.4f}); '
            f'yaw offset {result.yaw_offset_rad:.4f} rad; '
            f'yaw span {result.yaw_span_deg:.1f}° (floor MIN_ARC_DEG={MIN_ARC_DEG:.1f}°)'
        )

        for idx, m in result.marker_metrics.items():
            if m.status == 'ok':
                self.get_logger().debug(
                    f'Marker {idx + 1}: radius={m.radius_mm:.1f} mm, '
                    f'residual={m.fit_residual_mm:.3f} mm, '
                    f'axis_dev={m.distance_from_axis_mm:.3f} mm, '
                    f'arc_span={m.arc_span_deg:.1f}°'
                )
            else:
                self.get_logger().warn(f'Marker {idx + 1}: {m.status} — {m.reason}')

        # Publish on latched topic
        self._publish_calibration_result(result, result_message, accepted_at, suffix=suffix)

    # ──────────────────────────────────────────────────────────────────────
    #  Base-marker frame
    # ──────────────────────────────────────────────────────────────────────

    def _load_state(self):
        """(state, error) of the kappa pool: the state file, or the resource's
        seed when there is none. ``error`` names an unreadable state file (the
        state is then the seed; the caller reports it as an advisory line and
        the next write replaces the file, backing the old one up)."""
        model = self._base_model
        path = str(self.get_parameter('bb_base_frame_state_file').value)
        seed = (seed_base_state(model.records, model.base_ref) if model is not None
                else seed_base_state([], None))
        try:
            state = load_base_state(path)
        except (OSError, ValueError, KeyError, TypeError) as e:
            self.get_logger().debug(f'BB base frame state unreadable ({path}): {e!r}')
            return seed, f'{os.path.basename(path)}: {type(e).__name__}'
        if state is None:
            return seed, ''
        if state['base_ref'] is None:
            state['base_ref'] = seed['base_ref']
        return state, ''

    def _write_state(self, state):
        path = str(self.get_parameter('bb_base_frame_state_file').value)
        tpl = self._marker_template
        try:
            write_base_state(path, state, tpl.pinned_yaw_offset_deg, tpl.repeatability_deg)
        except OSError as e:
            self.get_logger().error(f'Could not persist the BB base frame pool to {path}: {e}')

    def _log_startup_state(self):
        """ONE INFO line at init: what history this process starts with."""
        tpl = self._marker_template
        tpl_txt = ('none' if tpl is None else
                   f'{os.path.basename(str(tpl.source))} ({len(tpl.points_mm)} markers)')
        state, err = self._load_state()
        eid = current_epoch_id(state)
        ep = next((e for e in state['epochs'] if e['id'] == eid), None)
        if err:
            pool = f'state unreadable ({err})'
        elif ep is None:
            pool = 'pool 0 records in 0 epochs'
        else:
            med = (ep['median_kappa_deg'] if tpl is None else
                   epoch_median_at_pin(state, ep, tpl.pinned_yaw_offset_deg))
            pool = (f'pool {len(state["records"])} records in {len(state["epochs"])} epochs '
                    f'(current K{eid} median {med:.2f}° n {ep["n"]})')
        ref, _label, ref_err = self._gate_reference()
        if ref_err:
            last = f'unreadable ({ref_err})'
        elif ref is None:
            last = 'none'
        else:
            last = f'{_label.replace("accepted ", "")} yaw {ref["yaw_offset_deg"]:+.2f}°'
        self.get_logger().info(
            f'BB calibration state: template {tpl_txt}, '
            f'base model {"on" if self._base_model is not None else "off"}, {pool}, '
            f'last accepted {last}, yaw source '
            f'{self.get_parameter("bb_yaw_source").value}')

    def _base_frame_view(self, result, snap: _SweepSnapshot, solve: _SweepSolve) -> dict:
        """This window's base pose (estimated on the worker: ``solve``) and the
        sweep's BB-in-base constants, plus the kappa pool state. Every key is
        present; ``pose`` None (with ``error``) when the base frame is off or
        not seen."""
        state, state_error = self._load_state()
        view = {'pose': None, 'error': '', 'kappa': None, 'p_b': None,
                'state': state, 'state_error': state_error, 'from_monitor': False}
        if self._base_model is None:
            view['error'] = f'BASE_FRAME_MODEL_MISSING: {self._base_model_error}'
            return view
        if solve.base_pose is not None:
            view['pose'] = solve.base_pose
        else:
            view['error'] = solve.base_error
            # A window that missed the base may use the running estimate (read
            # here, on the executor: the monitor is fed by the drain timer).
            last = self._base_monitor_last_t
            pose = self._base_monitor.pose(last) if self._base_monitor is not None else None
            if pose is not None and last is not None and \
                    snap.base_points and \
                    snap.base_points[-1][0] - pose.t_span_s[1] <= BASE_MONITOR_MAX_AGE_S:
                view['pose'], view['from_monitor'] = pose, True
        if view['pose'] is not None:
            view['kappa'], view['p_b'] = bb_in_base(
                view['pose'], math.degrees(result.yaw_offset_rad), result.bb_position_mm)
        return view

    def _kappa_record(self, view: dict, result) -> KappaRecord:
        return KappaRecord(
            kappa_deg=float(view['kappa']), sigma_deg=float(result.yaw_offset_std_deg),
            axis_point_b_mm=tuple(float(v) for v in view['p_b']),
            pin_deg=float(self._marker_template.pinned_yaw_offset_deg),
            accepted_at=datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
            yaw_offset_deg=math.degrees(result.yaw_offset_rad),
            base_heading_deg=float(view['pose'].heading_deg),
            source=f'mocap_node ({result.yaw_estimate.yaw_source})')

    def _gate_reference(self):
        """(reference, label, error) for the gate: the last accepted calibration
        from the state file, or (None, '', '') if there is no file. An
        unreadable file returns its error (an advisory line, never a refusal)."""
        path = str(self.get_parameter('bb_calibration_state_file').value)
        try:
            with open(path, 'r') as f:
                state = json.load(f)
            ref = {'yaw_offset_deg': float(state['yaw_offset_deg']),
                   # The reference's own σ: the gate's limit is
                   # 3·√(σ_new² + σ_ref²) (0 if the file predates it).
                   'yaw_offset_std_deg': float(state.get('yaw_offset_std_deg') or 0.0),
                   'position_mm': [float(v) for v in state['position_mm']]}
            # Seconds are enough to identify it; the full stamp stays in the file.
            stamp = str(state.get('accepted_at', '?'))[:19]
            return ref, f'accepted {stamp}', ''
        except FileNotFoundError:
            return None, '', ''
        except (OSError, ValueError, KeyError, TypeError) as e:
            # The refusal line names the file and the error class; the full
            # exception is DEBUG (it was a second, ~200-char ERROR line).
            self.get_logger().debug(f'BB calibration state file unreadable ({path}): {e!r}')
            return None, '', f'{os.path.basename(path)}: {type(e).__name__}'

    def _persist_accepted(self, result: CalibrationResult, verdict: str,
                          extra: dict | None = None) -> str:
        """Write the accepted calibration to the state file (atomically);
        returns its acceptance time as ISO-8601 UTC to the second
        (``2026-10-10T12:34:56Z``) — returned even if the write fails."""
        path = str(self.get_parameter('bb_calibration_state_file').value)
        est = result.yaw_estimate
        now = datetime.datetime.now(datetime.timezone.utc)
        state = {
            'accepted_at': now.isoformat(),
            'yaw_offset_deg': math.degrees(result.yaw_offset_rad),
            'yaw_offset_std_deg': result.yaw_offset_std_deg,
            'position_mm': [float(v) for v in result.bb_position_mm],
            'axis_tilt_deg': result.axis_tilt_deg,
            'arc_position_mm': (None if result.arc_position_mm is None
                                else [float(v) for v in result.arc_position_mm]),
            'method': result.yaw_method,
            'yaw_source': est.yaw_source,
            'lag_ms': est.lag_s * 1e3,
            'template_residual_mm': est.template_residual_mm,
            'n_frames': est.n_frames,
            'gate': verdict,
            'template': self._marker_template.source,
        }
        state.update(extra or {})
        try:
            os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
            tmp = path + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(state, f, indent=2)
            os.replace(tmp, path)
        except OSError as e:
            self.get_logger().error(
                f'Could not persist the accepted BB calibration to {path}: {e} — '
                'the next calibration is gated against the previous reference')
        return now.strftime('%Y-%m-%dT%H:%M:%SZ')

    def _attempt_suffix(self) -> str:
        """Sweep-kinematics suffix for the attempt text, from the LIVE
        window's yaw samples the estimator would use (stamped when selected,
        else heartbeat). For failures raised during collection or at the exit
        edge; a solved sweep's suffix comes from its snapshot."""
        requested = str(self.get_parameter('bb_yaw_source').value)
        if (requested != 'heartbeat'
                and len(self._calib_stamped_yaw) >= MIN_STAMPED_YAW_SAMPLES):
            samples = self._calib_stamped_yaw
        else:
            samples = self._calib_yaw_samples
        return sweep_suffix(samples, None)

    def _publish_calibration_result(self, result: CalibrationResult,
                                    message: str = 'Calibration successful',
                                    accepted_at: str = '', suffix: str | None = None):
        """A success replaces the calibration in force (bb/calibration_result)
        and is the latest attempt (bb/calibration_attempt)."""
        msg = BallButlerCalibrationResult()
        msg.success = True
        msg.message = message + (self._attempt_suffix() if suffix is None else suffix)
        msg.position_mm.x = float(result.bb_position_mm[0])
        msg.position_mm.y = float(result.bb_position_mm[1])
        msg.position_mm.z = float(result.bb_position_mm[2])
        msg.yaw_offset_rad = result.yaw_offset_rad
        msg.yaw_offset_std_deg = result.yaw_offset_std_deg
        msg.axis_tilt_deg = result.axis_tilt_deg
        self.pub_calibration.publish(msg)
        self.pub_calibration_attempt.publish(msg)
        self._last_good_calibration = msg
        self._last_good_at = accepted_at
        self.get_logger().debug('Published calibration result on bb/calibration_result')

    def _publish_calibration_failure(self, reason: str, suffix: str | None = None):
        """Report a failed sweep: ONE short line, and never over a success.

        The failure always goes to ``bb/calibration_attempt``. It goes to the
        latched ``bb/calibration_result`` only while this process has no
        successful calibration: once one exists the failed result is
        discarded there, so ``ball_butler_node`` (including one that restarts
        after the failure), the GUI's Calibrated indicator and the
        accuracy-testing runner all keep the last good calibration. The state
        file is written on success only (``_persist_accepted``).

        ``reason`` is collapsed to one line and bounded by
        ``MAX_CALIBRATION_FAILURE_CHARS`` (full text at DEBUG if it had to be
        cut) — the one enforcement point for the short-message contract.
        """
        reason = ' '.join(str(reason).split())
        if suffix is None:
            suffix = self._attempt_suffix()
        room = MAX_CALIBRATION_FAILURE_CHARS - len(suffix)
        if len(reason) > room:
            self.get_logger().debug(f'Calibration failure (full text): {reason}')
            reason = reason[:room - 1].rstrip() + '…'
        reason += suffix
        msg = BallButlerCalibrationResult()
        msg.success = False
        msg.message = reason
        self.pub_calibration_attempt.publish(msg)
        if self._last_good_calibration is None:
            self.pub_calibration.publish(msg)
            self.get_logger().error(f'BB calibration FAILED: {reason}')
        else:
            kept = f' from {self._last_good_at}' if self._last_good_at else ''
            self.get_logger().error(
                f'BB calibration FAILED: {reason} (kept the calibration{kept})')

    # ──────────────────────────────────────────────────────────────────────
    #  Lifecycle
    # ──────────────────────────────────────────────────────────────────────

    def on_shutdown(self):
        # Rule 2: a solve still in flight is joined (bounded) and its result dropped.
        if not self._calib_solver.stop(CALIBRATION_SOLVE_JOIN_TIMEOUT_S):
            self.get_logger().warn(
                'BB calibration solve still running at shutdown: result dropped')
        self.mocap.stop()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MocapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
