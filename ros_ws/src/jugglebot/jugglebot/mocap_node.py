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

import copy
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
from geometry_msgs.msg import Point, TransformStamped
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
from jugglebot.protocol_config import (
    BallButlerStates,
    MOCAP_ALIGNMENT_POS_THRESH_MM,
    MOCAP_ALIGNMENT_ROT_THRESH_DEG,
)
import jugglebot.hardware_config as hw
from jugglebot import mocap_status as mocap_st
from .bb_calibration import (
    run_calibration, CalibrationResult, MIN_ARC_DEG,
    BB_MARKER_COUNT, BB_YAW_ANCHOR_INDEX,
    load_marker_template, check_calibration_consistency,
    stamped_yaw_samples_from_joint_state,
)
from .bb_base_frame import (
    BaseFrameMonitor, KappaRecord, BASE_MAX_RESIDUAL_MM, DEFAULT_MIN_POOLED_SWEEPS,
    base_candidate_points, base_state_to_json, bb_in_base, check_base_frame_consistency,
    compose_bb_pose, estimate_base_pose, kappa_sweep_sd_deg, load_base_frame, load_base_state,
    pool_kappa,
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
#: line was ~900 characters.
MAX_CALIBRATION_FAILURE_CHARS = 240

#: BB base-marker frame (2026-10-10, ``jugglebot.bb_base_frame``): four fixed
#: markers on BB's shelf, posed by geometry among every marker but BB's own
#: (a QTM label on a base marker does not hide it). The resource
#: holds their as-built coordinates and the seed BB-in-base records; the state
#: file holds the node's running pool (seeded from the resource when absent).
DEFAULT_BASE_FRAME_FILE = 'bb_base_frame.json'
DEFAULT_BASE_FRAME_STATE_FILE = os.path.join(
    os.path.expanduser('~'), 'bb_calibration_sessions', 'bb_base_frame_state.json')
#: ``bb_pose_source``: what the published BB world pose comes from.
#: ``sweep`` — the sweep estimator's world pose, gated in the world
#: (unchanged); the base frame, when seen, is a diagnostic and each accepted
#: sweep consistent with it extends the BB-in-base pool. ``base_frame`` — this
#: window's base pose composed with the pooled BB-in-base constants, gated in
#: the base frame (a QTM frame shift is reported and accepted; a BB-in-base
#: change is refused unless bb_moved); refused when the base is not seen.
#: ``auto`` — base_frame once the pool has ``bb_base_min_sweeps`` sweeps and the
#: base is seen with a residual inside BASE_MAX_RESIDUAL_MM, else sweep.
#: Default ``auto`` since 2026-10-10 13:45: the second sitting with the frame
#: (8 sweeps, stamped yaw, all accepted) read κ = +179.515° ± 0.011° against
#: the build's +179.491° ± 0.016° (Δ 0.024°, 1.3 combined SE, inside the 0.05°
#: criterion for a BB-vs-base QTM warp) with the base heading shifted only
#: +0.003° and the origin 0.7 mm — BB-in-base holds across sittings, so the
#: base path (≈ 0.01° per sitting) replaces the sweep's own 0.03–0.05° value
#: whenever the base is seen (logbook 2026-10-10-bb-base-marker-frame).
BB_POSE_SOURCES = ('sweep', 'base_frame', 'auto')
DEFAULT_BB_POSE_SOURCE = 'auto'
#: Cadence (s) of the slow running base-pose estimate fed from the per-frame path.
BASE_MONITOR_PERIOD_S = 0.2
#: A window that does not see the base may use the running estimate if its
#: newest pose is at most this old (s).
BASE_MONITOR_MAX_AGE_S = 30.0

#: Shortest interval (s) between two "frames lost" WARN lines. Losses inside
#: the interval are summed into the next line, so none goes unreported; the
#: first loss after a quiet spell warns at once (on the next 1 Hz check).
MOCAP_LOSS_WARN_PERIOD_S = 10.0
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


def _set_stamp(stamp, ns):
    """Write ROS time ``ns`` into a builtin_interfaces/Time field."""
    stamp.sec = int(ns // 1_000_000_000)
    stamp.nanosec = int(ns % 1_000_000_000)

# BB_MARKER_COUNT (``mocap_interface.ball_butler_markers`` is a fixed
# (BB_MARKER_COUNT, 4) array with NaN rows for the ones QTM cannot see this
# frame) and BB_YAW_ANCHOR_INDEX (the marker ``run_calibration`` hard-requires
# for the yaw offset) are single-sourced from bb_calibration.


class MocapNode(Node):
    def __init__(self):
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
        self.mocap.set_alignment_thresholds(
            MOCAP_ALIGNMENT_POS_THRESH_MM, MOCAP_ALIGNMENT_ROT_THRESH_DEG
        )
        self.mocap.ready_to_publish = True

        # ── Static TF: world → platform_start ─────────────────────────────
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
        self._loss_pending = [0, 0, 0]        # node overflow, QTM gap frames, gap events
        self._loss_pending_since: float | None = None
        self._loss_warn_mono: float | None = None
        self.create_timer(MOCAP_STATUS_PERIOD_S, self._publish_mocap_status)
        # Q5b/Q5c ride their OWN timer rather than piggy-backing on the status
        # publisher: the publisher must stay a pure snapshot read (determinism
        # doctrine), and this one *acts* — it publishes failures and unlatches
        # collection state.
        self.create_timer(MOCAP_STATUS_PERIOD_S, self._check_calibration_health)

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

        # ── Body template + consistency gate (2026-10-09) ─────────────────
        # bb_moved: the operator's explicit statement that BB was physically
        # moved OR that QTM was recalibrated — either changes the frame, and
        # the rotating markers alone cannot tell the two apart — so a
        # calibration that disagrees with the last accepted one is real and
        # becomes the new reference. ONE-SHOT: it arms the next calibration
        # only, and is re-armed by setting it true again (a parameter left true
        # must not disable the gate for good). Either event also invalidates
        # the aim correction (throw_affine_correction.json): refit it.
        self.declare_parameter('bb_marker_template_file', DEFAULT_MARKER_TEMPLATE_FILE)
        self.declare_parameter('bb_calibration_state_file', DEFAULT_CALIBRATION_STATE_FILE)
        self.declare_parameter('bb_moved', False)
        self.declare_parameter('bb_yaw_source', DEFAULT_BB_YAW_SOURCE)
        self.declare_parameter('bb_pose_source', DEFAULT_BB_POSE_SOURCE)
        self.declare_parameter('bb_base_frame_file', DEFAULT_BASE_FRAME_FILE)
        self.declare_parameter('bb_base_frame_state_file', DEFAULT_BASE_FRAME_STATE_FILE)
        self.declare_parameter('bb_base_min_sweeps', DEFAULT_MIN_POOLED_SWEEPS)
        self._bb_moved_armed = bool(self.get_parameter('bb_moved').value)
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

        self.get_logger().info('Mocap node up (QTM connects in the background)')

    def _on_set_parameters(self, params):
        for p in params:
            if p.name == 'bb_yaw_source' and p.value not in BB_YAW_SOURCES:
                return SetParametersResult(
                    successful=False,
                    reason=f'bb_yaw_source must be one of {", ".join(BB_YAW_SOURCES)}')
            if p.name == 'bb_pose_source' and p.value not in BB_POSE_SOURCES:
                return SetParametersResult(
                    successful=False,
                    reason=f'bb_pose_source must be one of {", ".join(BB_POSE_SOURCES)}')
        for p in params:
            if p.name == 'bb_moved':
                self._bb_moved_armed = bool(p.value)
                if self._bb_moved_armed:
                    self.get_logger().warn(
                        'bb_moved armed: the next BB calibration may move the yaw '
                        'frame / axis point past the consistency gate and becomes the '
                        'reference (BB moved or QTM recalibrated; refit the aim correction)')
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
                                   '(bb_pose_source sweep only)')
            return
        self._base_monitor = BaseFrameMonitor(self._base_model.template_mm)
        self.get_logger().debug(
            f'BB base frame: as-built template from {path}, '
            f'{len(self._base_model.records)} seed BB-in-base records')

    # ──────────────────────────────────────────────────────────────────────
    #  Publishing
    # ──────────────────────────────────────────────────────────────────────

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
        _, ref, err = self._base_state()
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
        if ovf or gap:
            if self._loss_pending_since is None:
                self._loss_pending_since = prev_mono if prev_mono is not None else now
            p = self._loss_pending
            p[0] += ovf
            p[1] += gap
            p[2] += events
        p = self._loss_pending
        if (p[0] or p[1]) and (self._loss_warn_mono is None
                               or now - self._loss_warn_mono >= MOCAP_LOSS_WARN_PERIOD_S):
            self.get_logger().warn(
                f'mocap: {p[0] + p[1]} QTM frames lost in the last '
                f'{now - self._loss_pending_since:.0f} s — {p[0]} dropped by mocap_node '
                f'(frame queue full: starved for over '
                f'{FRAME_QUEUE_MAXLEN / QTM_STREAM_HZ:.1f} s), {p[1]} never received from '
                f'QTM ({p[2]} frame-number gaps); QTM 2D drop {st["max_drop_rate"]}‰ — '
                'check the Jetson load (another job running?) and QTM')
            self._loss_warn_mono = now
            self._loss_pending = [0, 0, 0]
            self._loss_pending_since = None
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
            ]
            self.pub_mocap_status.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing mocap status: {e}',
                                    throttle_duration_sec=5.0)

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

        # ── Rigid bodies ──────────────────────────────────────────────
        # Publishing is intentionally NOT gated on base alignment. is_aligned
        # only flips True while the "Base" rigid body is in view; when Base is
        # moved out of the test area (e.g. the distance-sweep campaign) or sits
        # far off the QTM origin, gating here would suppress /rigid_body_poses
        # entirely — including the Catching_Cone target the thrower needs — and
        # silently block every throw. Alignment is still computed and surfaced
        # via MocapDataMulti.aligned (plus a misalignment warning) for the GUI;
        # it is informational, not a hard gate on rigid-body publishing.
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

    def _broadcast_platform_start_tf(self, z_offset_mm: float):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'platform_start'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = z_offset_mm
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().debug(
            f'Broadcast static tf: world -> platform_start (z_offset={z_offset_mm:.1f} mm)'
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
        """Run the calibration pipeline and publish the result."""
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
        try:
            result: CalibrationResult = run_calibration(
                calibration_data=self._calib_data,
                yaw_readings_deg=self._calib_yaw_readings,
                pitch_z_offset_mm=hw.BB_GEOM_PITCH_Z_OFFSET_MM,
                marker_frames=self._calib_frames,
                yaw_samples=yaw_samples,
                template=self._marker_template,
                yaw_source=yaw_source,
            )
        except ValueError as e:
            self.get_logger().debug(f'Calibration solver rejected the sweep: {e}')
            self._publish_calibration_failure(str(e))
            return
        if result.yaw_method != 'constellation':  # defensive: never publish the anchor value
            self._publish_calibration_failure(
                f'internal: yaw offset came from the {result.yaw_method!r} estimator')
            return

        # ── Base-marker frame: this window's base pose, BB-in-base, mode ───
        bb_moved = self._bb_moved_armed       # captured before any gate consumes it
        view = self._base_frame_view(result)
        requested_pose = str(self.get_parameter('bb_pose_source').value)
        if requested_pose not in BB_POSE_SOURCES:
            self._publish_calibration_failure(
                f'BB_POSE_SOURCE_INVALID: bb_pose_source {requested_pose!r} is not one of '
                f'{", ".join(BB_POSE_SOURCES)}')
            return
        pose_source, refusal = self._resolve_pose_source(requested_pose, view, bb_moved)
        if pose_source is None:
            self.get_logger().debug(f'Refused sweep ({refusal}): {result.yaw_estimate.summary()}')
            self._publish_calibration_failure(refusal)
            return

        # ── Consistency gate ──────────────────────────────────────────────
        yaw_deg = math.degrees(result.yaw_offset_rad)
        est = result.yaw_estimate
        arc = result.arc_position_mm
        arc_note = ('' if arc is None else
                    f'arc-fit cross-check ({arc[0]:.2f}, {arc[1]:.2f}, {arc[2]:.2f}) mm, '
                    f'{float(np.linalg.norm(np.asarray(arc[:2]) - result.bb_position_mm[:2])):.2f} mm '
                    'from the body-model axis point in x/y')
        summary = f'{est.summary()}; {arc_note}'
        if view['pose'] is not None:
            summary += (f'; {view["pose"].summary()}; BB-in-base κ {view["kappa"]:+.4f}°, '
                        f'p_b ({view["p_b"][0]:.2f}, {view["p_b"][1]:.2f}, {view["p_b"][2]:.2f}) mm')
        elif view['error']:
            summary += f'; base frame: {view["error"]}'
        published = result
        base_verdict = None
        anchor = ('n/a' if result.anchor_yaw_offset_rad is None
                  else f'{math.degrees(result.anchor_yaw_offset_rad):+.3f}°')
        if pose_source == 'base_frame':
            base_verdict = check_base_frame_consistency(
                view['kappa'], kappa_sweep_sd_deg(est.yaw_source), view['p_b'], view['pose'],
                view['pooled'], view['base_ref'], bb_moved=bb_moved)
            self.get_logger().debug(
                f'Yaw offset (sweep) {yaw_deg:+.4f}° — {summary}; retired anchor estimator '
                f'on the same sweep {anchor}; {base_verdict.message}')
            if not base_verdict.accepted:
                self._publish_calibration_failure(base_verdict.message)
                return
            if base_verdict.overridden:
                self._bb_moved_armed = False   # one-shot: consumed by this calibration
            if base_verdict.qtm_shift:
                self.get_logger().warn(
                    f'BB base frame moved {base_verdict.base_shift_mm:.2f} mm, '
                    f'{base_verdict.base_shift_deg:+.3f}° in the world since '
                    f'{view["base_ref"].get("accepted_at", "?")[:19]}: QTM frame shift, '
                    'BB-in-base unchanged — accepted')
            records = self._pool_records(view, result, base_verdict.reset_pool)
            pooled = pool_kappa(records, self._marker_template.pinned_yaw_offset_deg,
                                self._marker_template.repeatability_deg)
            yaw_b, pos_b = compose_bb_pose(view['pose'], pooled.kappa_deg, pooled.axis_point_b_mm)
            sigma_b = math.hypot(pooled.se_deg, view['pose'].heading_se_deg)
            published = copy.copy(result)
            published.yaw_offset_rad = math.radians(yaw_b)
            published.yaw_offset_std_deg = sigma_b
            published.bb_position_mm = np.asarray(pos_b, dtype=float)
            published.yaw_method = 'base_frame'
            gate_message = (f'{base_verdict.message} · pose source base_frame: {yaw_b:+.4f}° '
                            f'±{sigma_b:.3f}° (κ {pooled.kappa_deg:+.4f}°, {pooled.count_note()}) vs '
                            f'sweep {yaw_deg:+.4f}° (Δ {yaw_b - yaw_deg:+.3f}°), axis point '
                            f'{float(np.linalg.norm(pos_b - result.bb_position_mm)):.2f} mm '
                            'from the sweep\'s')
            self._persist_base_state(records, view['pose'])
        else:
            reference, ref_label, ref_error = self._gate_reference()
            if ref_error and not bb_moved:
                # A corrupt state file must not silently disable the gate. The
                # estimator detail is DEBUG; the failure is one short line.
                self.get_logger().debug(f'Refused sweep: {summary}')
                self._publish_calibration_failure(
                    f'CALIBRATION_STATE_UNREADABLE: {ref_error} — fix or delete it, or '
                    'set bb_moved:=true')
                return
            if ref_error:
                self.get_logger().warn(
                    f'BB calibration state unreadable ({ref_error}); bb_moved:=true makes '
                    'this calibration the new reference')
            verdict = check_calibration_consistency(
                yaw_deg, result.yaw_offset_std_deg, result.bb_position_mm,
                est.template_residual_mm, None if ref_error else reference,
                bb_moved=bb_moved, reference_label=ref_label)
            # The base frame, when seen, judges the same sweep in its own frame:
            # a diagnostic here (the world gate decides), and the pool's input.
            if view['pose'] is not None and view['state_error']:
                self.get_logger().warn(f'BB base frame state unreadable ({view["state_error"]}) '
                                       '— this sweep is not pooled')
            elif view['pose'] is not None:
                base_verdict = check_base_frame_consistency(
                    view['kappa'], kappa_sweep_sd_deg(est.yaw_source), view['p_b'], view['pose'],
                    view['pooled'], view['base_ref'], bb_moved=bb_moved and verdict.accepted)
            self.get_logger().debug(
                f'Yaw offset {yaw_deg:+.4f}° — {summary}; retired anchor estimator '
                f'on the same sweep {anchor}; {verdict.message}'
                + ('' if base_verdict is None else f'; {base_verdict.message}'))
            if not verdict.accepted:
                message = verdict.message
                if message.startswith('CALIBRATION_INCONSISTENT') and \
                        self._base_expected(requested_pose, view):
                    # The base frame would have decided this sweep and was not
                    # seen: THAT is the cause to fix (restore the base view),
                    # not bb_moved — the world gate's numbers follow as context.
                    message = (f'{view["error"]} — check QTM sees the 4 shelf markers '
                               '(occluded? taken into the Ball Butler body?); world gate also '
                               'refused: Δyaw '
                               f'{verdict.delta_deg:+.2f}°, Δaxis {verdict.axis_shift_mm:.1f} mm')
                elif (base_verdict is not None and base_verdict.accepted
                        and not base_verdict.no_reference and base_verdict.qtm_shift
                        and message.startswith('CALIBRATION_INCONSISTENT')):
                    # Short hint (the failure line is bounded): the base frame
                    # says BB did not move — QTM's frame did.
                    message += ' · base frame: BB unmoved, QTM frame shifted'
                self._publish_calibration_failure(message)
                return
            if verdict.no_reference:
                self.get_logger().warn(
                    'BB calibration: no reference calibration exists (no state file) — this '
                    'one is accepted unchecked and becomes the reference. Validate it against '
                    'landings before trusting the aim')
            if self._bb_moved_armed and (verdict.overridden or ref_error):
                self._bb_moved_armed = False   # one-shot: consumed by this calibration
            gate_message = verdict.message
            if base_verdict is not None:
                if base_verdict.accepted:
                    self._persist_base_state(
                        self._pool_records(view, result, base_verdict.reset_pool), view['pose'])
                else:
                    self.get_logger().warn(f'BB base frame: {base_verdict.message} — this '
                                           'sweep is not pooled')
        accepted_at = self._persist_accepted(published, gate_message, extra={
            'pose_source': pose_source,
            'sweep_yaw_offset_deg': yaw_deg,
            'sweep_position_mm': [float(v) for v in result.bb_position_mm],
            'base_frame': None if view['pose'] is None else view['pose'].to_dict(),
            'kappa_deg': view['kappa'],
        })
        result_message = (f'Calibration successful (accepted {accepted_at}) · {summary} · '
                          f'{gate_message}')
        result = published

        # One operator line; the detail (axis direction, per-marker fits) is DEBUG.
        # BB's own reported yaw span is encoder-derived, so unlike the per-marker
        # arc_span it is not inflated by QTM marker noise (MIN_ARC_DEG was set
        # from it, 2026-08-25).
        # An outcast (a marker the consensus excluded, bb_calibration
        # MIN_AGREEING_MARKERS) is named on the outcome line itself: the
        # calibration stands, but a marker that keeps turning up here is a
        # physical fault to go and look at.
        pos = result.bb_position_mm
        outcasts = ''.join(
            f' · outcast Marker {idx + 1} ({m.distance_from_axis_mm:.2f} mm off axis)'
            for idx, m in sorted(result.marker_metrics.items())
            if m.status == 'outcast')
        source_note = ('' if pose_source != 'base_frame' else
                       f' · base frame (sweep {yaw_deg:+.2f}°)')
        self.get_logger().info(
            f'BB calibrated: pos ({pos[0]:.0f}, {pos[1]:.0f}, {pos[2]:.0f}) mm '
            f'· axis tilt {result.axis_tilt_deg:.2f}° '
            f'· yaw offset {math.degrees(result.yaw_offset_rad):+.2f}° '
            f'±{result.yaw_offset_std_deg:.2f}° '
            f'(lag {result.yaw_estimate.lag_s * 1e3:.0f} ms, gate ok){source_note} '
            f'· swept {result.yaw_span_deg:.0f}°{outcasts}'
        )
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
        self._publish_calibration_result(result, result_message, accepted_at)

    # ──────────────────────────────────────────────────────────────────────
    #  Base-marker frame
    # ──────────────────────────────────────────────────────────────────────

    def _base_state(self):
        """(records, base_ref, error) of the base-frame pool: the state file,
        or the resource's seed when there is none. ``error`` names an
        unreadable state file (records/base_ref are then the seed)."""
        model = self._base_model
        seed = (list(model.records), model.base_ref) if model is not None else ([], None)
        path = str(self.get_parameter('bb_base_frame_state_file').value)
        try:
            state = load_base_state(path)
        except (OSError, ValueError, KeyError, TypeError) as e:
            self.get_logger().debug(f'BB base frame state unreadable ({path}): {e!r}')
            return seed[0], seed[1], f'{os.path.basename(path)}: {type(e).__name__}'
        if state is None:
            return seed[0], seed[1], ''
        return state['records'], state['base_ref'] or seed[1], ''

    def _base_frame_view(self, result) -> dict:
        """This window's base pose and the sweep's BB-in-base constants, plus
        the pool as it stands. Every key is present; ``pose`` None (with
        ``error``) when the base frame is off or not seen."""
        view = {'pose': None, 'error': '', 'kappa': None, 'p_b': None, 'records': [],
                'pooled': None, 'base_ref': None, 'state_error': '', 'from_monitor': False}
        if self._base_model is None:
            view['error'] = f'BASE_FRAME_MODEL_MISSING: {self._base_model_error}'
            return view
        T = self._base_model.template_mm
        try:
            view['pose'] = estimate_base_pose(self._calib_base_points, T)
        except ValueError as e:
            view['error'] = str(e)
            # A window that missed the base may use the running estimate.
            last = self._base_monitor_last_t
            pose = self._base_monitor.pose(last) if self._base_monitor is not None else None
            if pose is not None and last is not None and \
                    self._calib_base_points and \
                    self._calib_base_points[-1][0] - pose.t_span_s[1] <= BASE_MONITOR_MAX_AGE_S:
                view['pose'], view['from_monitor'] = pose, True
        records, base_ref, err = self._base_state()
        view['records'], view['base_ref'], view['state_error'] = records, base_ref, err
        tpl = self._marker_template
        view['pooled'] = pool_kappa(records, tpl.pinned_yaw_offset_deg, tpl.repeatability_deg)
        if view['pose'] is not None:
            view['kappa'], view['p_b'] = bb_in_base(
                view['pose'], math.degrees(result.yaw_offset_rad), result.bb_position_mm)
        return view

    def _resolve_pose_source(self, requested: str, view: dict, bb_moved: bool):
        """(pose source, '') — or (None, refusal) when ``base_frame`` was asked
        for and cannot be had. ``auto`` falls back to the sweep, saying why at DEBUG."""
        if requested == 'sweep':
            return 'sweep', ''
        problem = ''
        if view['pose'] is None:
            problem = view['error'] or 'BASE_FRAME_NOT_SEEN'
        elif view['state_error'] and not bb_moved:
            problem = (f'BASE_FRAME_STATE_UNREADABLE: {view["state_error"]} — fix or delete it, '
                       'or set bb_moved:=true')
        if requested == 'base_frame':
            if not problem:
                return 'base_frame', ''
            if problem.startswith(('BASE_FRAME_NOT_SEEN', 'BASE_FRAME_MODEL_MISSING')):
                problem += ' — bb_pose_source is base_frame; set it to auto or sweep'
            return None, problem
        n = 0 if view['pooled'] is None else view['pooled'].n
        need = int(self.get_parameter('bb_base_min_sweeps').value)
        if not problem and view['pose'].residual_mm > BASE_MAX_RESIDUAL_MM:
            problem = f'base residual {view["pose"].residual_mm:.2f} mm'
        if not problem and n < need:
            problem = f'{n} pooled sweeps < {need}'
        if problem:
            if self._base_expected(requested, view):
                # Not silent: the pose source the operator relies on is gone.
                self.get_logger().warn(f'bb_pose_source auto: the base frame was not seen — this '
                                       f'sweep falls back to the world gate ({problem})')
            else:
                self.get_logger().debug(f'bb_pose_source auto: sweep ({problem})')
            return 'sweep', ''
        return 'base_frame', ''

    def _base_expected(self, requested: str, view: dict) -> bool:
        """True when the base frame would have posed this sweep — pose source
        auto/base_frame, the model loaded, the pool at ``bb_base_min_sweeps`` —
        but the window did not see it (BASE_FRAME_NOT_SEEN). Then the missing
        base, not a BB move, is the cause to name."""
        if requested not in ('auto', 'base_frame') or self._base_model is None:
            return False
        if view['pose'] is not None or not view['error'].startswith('BASE_FRAME_NOT_SEEN'):
            return False
        n = 0 if view['pooled'] is None else view['pooled'].n
        return n >= int(self.get_parameter('bb_base_min_sweeps').value)

    def _pool_records(self, view: dict, result, reset: bool) -> list:
        """The pool with this accepted sweep appended (a new pool on reset)."""
        rec = KappaRecord(
            kappa_deg=float(view['kappa']), sigma_deg=float(result.yaw_offset_std_deg),
            axis_point_b_mm=tuple(float(v) for v in view['p_b']),
            pin_deg=float(self._marker_template.pinned_yaw_offset_deg),
            accepted_at=datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
            yaw_offset_deg=math.degrees(result.yaw_offset_rad),
            base_heading_deg=float(view['pose'].heading_deg),
            source=f'mocap_node ({result.yaw_estimate.yaw_source})')
        return ([] if reset else list(view['records'])) + [rec]

    def _persist_base_state(self, records: list, pose):
        """Write the pool and this sitting's base pose (atomically)."""
        path = str(self.get_parameter('bb_base_frame_state_file').value)
        ref = {'origin_mm': [float(v) for v in pose.origin_mm],
               'heading_deg': float(pose.heading_deg),
               'accepted_at': datetime.datetime.now(datetime.timezone.utc).isoformat()}
        try:
            os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
            tmp = path + '.tmp'
            with open(tmp, 'w') as f:
                tpl = self._marker_template
                json.dump(base_state_to_json(records, ref, tpl.pinned_yaw_offset_deg,
                                             tpl.repeatability_deg), f, indent=2)
            os.replace(tmp, path)
        except OSError as e:
            self.get_logger().error(f'Could not persist the BB base frame pool to {path}: {e}')

    def _gate_reference(self):
        """(reference, label, error) for the gate: the last accepted calibration
        from the state file, or (None, '', '') if there is no file. An
        unreadable file returns its error (the caller refuses unless bb_moved
        is armed) — never a silent fallback."""
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

    def _publish_calibration_result(self, result: CalibrationResult,
                                    message: str = 'Calibration successful',
                                    accepted_at: str = ''):
        """A success replaces the calibration in force (bb/calibration_result)
        and is the latest attempt (bb/calibration_attempt)."""
        msg = BallButlerCalibrationResult()
        msg.success = True
        msg.message = message
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

    def _publish_calibration_failure(self, reason: str):
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
        if len(reason) > MAX_CALIBRATION_FAILURE_CHARS:
            self.get_logger().debug(f'Calibration failure (full text): {reason}')
            reason = reason[:MAX_CALIBRATION_FAILURE_CHARS - 1].rstrip() + '…'
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
