"""Mocap Interface Node — QTM streaming, tf2 broadcast, BB calibration.

Publishes:
  mocap_data             (MocapDataMulti)   — all markers (labelled + unlabelled) at 200 Hz
  rigid_body_poses       (RigidBodyPoses)   — all rigid bodies at 200 Hz
  bb/markers             (MocapDataMulti)   — BB fiducial markers (always, when QTM connected)
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
from geometry_msgs.msg import TransformStamped
from jugglebot_interfaces.msg import (
    MocapDataMulti,
    MocapDataSingle,
    BallButlerHeartbeat,
    BallButlerCalibrationResult,
    RigidBodyPose,
    RigidBodyPoses,
)
import tf2_ros

from .mocap_interface import MocapInterface
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

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - outside a ROS install
    get_package_share_directory = None


#: Cadence of the ``mocap/status`` publisher and of the calibration health
#: check. 5 Hz: fast enough that the consumers' 1.0 s staleness window
#: (``mocap_status.MOCAP_STATUS_MAX_AGE_S``) needs five consecutive missed
#: cycles before they refuse, slow enough to be free next to the 200 Hz marker
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
#: ``heartbeat`` (DEFAULT) — the 10 Hz bb/heartbeat yaw at its receive time;
#: ``stamped`` — the 100 Hz bb_yaw of bb/axis_estimates at its bridge stamp
#: (refused when fewer than MIN_STAMPED_YAW_SAMPLES arrived); ``auto`` — the
#: stamped stream when it has enough samples, else the heartbeat (the
#: 2026-10-10 behaviour). The heartbeat is the default because it is the only
#: source verified to repeat (0.062° SD over 7 sweeps, bag 2026-10-09_23-49-07);
#: the stamped source's only bag (2026-10-10_00-24-06) had a wandering
#: QTM->ROS clock that scattered BOTH sources ~0.25° and that the estimator now
#: refuses (CONSTELLATION_MOCAP_CLOCK), so it is unverified, not shown bad
#: (logbook 2026-10-10-bb-yaw-offset-spread-stamped-source).
BB_YAW_SOURCES = ('heartbeat', 'stamped', 'auto')
DEFAULT_BB_YAW_SOURCE = 'heartbeat'
#: Bound on a published calibration FAILURE text (and so on its ERROR line):
#: one line, the code plus the decisive numbers. The estimator detail goes to
#: DEBUG. ``_publish_calibration_failure`` enforces it (collapses whitespace,
#: truncates with an ellipsis and logs the full text at DEBUG) — a safety net:
#: every reason the node produces fits without truncation
#: (tests/ros/test_mocap_node_keep_last_good.py). The 2026-10-09 live failure
#: line was ~900 characters.
MAX_CALIBRATION_FAILURE_CHARS = 240

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
        self.create_timer(hw.TRACKING_MOCAP_DT_S, self._publish_mocap_data)
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
        self._bb_moved_armed = bool(self.get_parameter('bb_moved').value)
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self._marker_template = None
        self._marker_template_error = ''
        self._load_marker_template(
            str(self.get_parameter('bb_marker_template_file').value))

        self.get_logger().info('Mocap node up (QTM connects in the background)')

    def _on_set_parameters(self, params):
        for p in params:
            if p.name == 'bb_yaw_source' and p.value not in BB_YAW_SOURCES:
                return SetParametersResult(
                    successful=False,
                    reason=f'bb_yaw_source must be one of {", ".join(BB_YAW_SOURCES)}')
        for p in params:
            if p.name == 'bb_moved':
                self._bb_moved_armed = bool(p.value)
                if self._bb_moved_armed:
                    self.get_logger().warn(
                        'bb_moved armed: the next BB calibration may move the yaw '
                        'frame / axis point past the consistency gate and becomes the '
                        'reference (BB moved or QTM recalibrated; refit the aim correction)')
        return SetParametersResult(successful=True)

    def _load_marker_template(self, name: str):
        """Resolve and load the marker template; on failure record why (the
        calibration then FAILS with that reason — there is no anchor fallback)."""
        candidates = [name,
                      os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   os.pardir, 'resources', name)]
        if get_package_share_directory is not None:
            try:
                candidates.append(os.path.join(
                    get_package_share_directory('jugglebot'), 'resources', name))
            except Exception:
                pass
        path = next((c for c in candidates if os.path.isfile(c)), None)
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

    # ──────────────────────────────────────────────────────────────────────
    #  Publishing
    # ──────────────────────────────────────────────────────────────────────

    def _publish_clock_offset(self):
        status = self.mocap.get_qtm_sync_status()
        offset = status.get('offset_s')
        if offset is not None:
            msg = Float64()
            msg.data = offset
            self.pub_clock_offset.publish(msg)

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

    def _publish_mocap_data(self):
        is_aligned = self.mocap.is_aligned

        # ── Stop publishing when QTM packets aren't arriving ──
        # The GUI has its own 2-second timeout that sets disconnected/unaligned
        # and clears markers, so going silent here is the correct behaviour.
        if not self.mocap.is_receiving():
            return

        unlabelled = self.mocap.get_all_markers_base_frame()
        labelled = self.mocap.get_labelled_markers()
        frame_ros_ns = self.mocap.latest_frame_ros_ns()
        try:
            msg = MocapDataMulti()
            msg.aligned = is_aligned
            if frame_ros_ns is not None:
                msg.stamp.sec = int(frame_ros_ns // 1_000_000_000)
                msg.stamp.nanosec = int(frame_ros_ns % 1_000_000_000)

            # Add labelled markers (with their QTM label)
            for label, x, y, z, residual in labelled:
                s = MocapDataSingle()
                s.position.x = float(x)
                s.position.y = float(y)
                s.position.z = float(z)
                s.residual = float(residual)
                s.label = label
                msg.markers.append(s)

            # Add unlabelled markers (label left as empty string)
            if unlabelled is not None and unlabelled.shape[0] > 0:
                for i in range(unlabelled.shape[0]):
                    s = MocapDataSingle()
                    s.position.x = float(unlabelled[i, 0])
                    s.position.y = float(unlabelled[i, 1])
                    s.position.z = float(unlabelled[i, 2])
                    s.residual = float(unlabelled[i, 3])
                    msg.markers.append(s)

            self.pub_mocap.publish(msg)
            self.mocap.clear_markers()
        except Exception as e:
            self.get_logger().error(f'Error publishing markers: {e}')

        # ── Rigid bodies ──────────────────────────────────────────────
        # Publishing is intentionally NOT gated on base alignment. is_aligned
        # only flips True while the "Base" rigid body is in view; when Base is
        # moved out of the test area (e.g. the distance-sweep campaign) or sits
        # far off the QTM origin, gating here would suppress /rigid_body_poses
        # entirely — including the Catching_Cone target the thrower needs — and
        # silently block every throw. Alignment is still computed and surfaced
        # via MocapDataMulti.aligned (plus a misalignment warning) for the GUI;
        # it is informational, not a hard gate on rigid-body publishing.
        try:
            body_poses = self.mocap.get_body_poses()
            if body_poses:
                msg = RigidBodyPoses()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'world'
                for name, pose in body_poses.items():
                    body_msg = RigidBodyPose()
                    body_msg.name = name
                    body_msg.pose = pose
                    msg.bodies.append(body_msg)
                if msg.bodies:
                    self.pub_rigid_bodies.publish(msg)
            self.mocap.clear_body_poses()
        except Exception as e:
            self.get_logger().error(f'Error publishing body poses: {e}')

        # ── BB markers (always published; accumulated during calibration) ──
        try:
            bb_markers = self.mocap.get_ball_butler_markers_base_frame()
            if bb_markers is not None and bb_markers.shape[0] > 0:
                msg = MocapDataMulti()
                if frame_ros_ns is not None:
                    msg.stamp.sec = int(frame_ros_ns // 1_000_000_000)
                    msg.stamp.nanosec = int(frame_ros_ns % 1_000_000_000)
                for i in range(bb_markers.shape[0]):
                    s = MocapDataSingle()
                    s.position.x = float(bb_markers[i, 0])
                    s.position.y = float(bb_markers[i, 1])
                    s.position.z = float(bb_markers[i, 2])
                    s.residual = float(bb_markers[i, 3])
                    msg.markers.append(s)
                self.pub_bb_markers.publish(msg)

                # Accumulate for calibration. An invalidated window (Q5b/Q5c)
                # stops sampling: more points cannot repair an arc with a hole
                # in it, and a growing dict would only make the garbage look
                # better-supported.
                if self._calibrating and self._calib_invalid is None:
                    self._accumulate_calibration_markers(msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing BB markers: {e}')

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
        at the frame's QTM stamp, for the sweep yaw estimator. The publisher
        runs on a timer at the QTM rate and snapshots the latest frame, so a
        frame can be seen twice: one point set per distinct stamp. A message
        without a stamp (QTM clock sync not yet established) feeds the arc fit
        only."""
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

        # ── Consistency gate against the last accepted calibration ─────────
        yaw_deg = math.degrees(result.yaw_offset_rad)
        est = result.yaw_estimate
        arc = result.arc_position_mm
        arc_note = ('' if arc is None else
                    f'arc-fit cross-check ({arc[0]:.2f}, {arc[1]:.2f}, {arc[2]:.2f}) mm, '
                    f'{float(np.linalg.norm(np.asarray(arc[:2]) - result.bb_position_mm[:2])):.2f} mm '
                    'from the body-model axis point in x/y')
        summary = f'{est.summary()}; {arc_note}'
        reference, ref_label, ref_error = self._gate_reference()
        if ref_error and not self._bb_moved_armed:
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
            bb_moved=self._bb_moved_armed, reference_label=ref_label)
        anchor = ('n/a' if result.anchor_yaw_offset_rad is None
                  else f'{math.degrees(result.anchor_yaw_offset_rad):+.3f}°')
        self.get_logger().debug(
            f'Yaw offset {yaw_deg:+.4f}° — {summary}; retired anchor estimator '
            f'on the same sweep {anchor}; {verdict.message}')
        if not verdict.accepted:
            # The estimator summary is in the DEBUG line just above.
            self._publish_calibration_failure(verdict.message)
            return
        if verdict.no_reference:
            self.get_logger().warn(
                'BB calibration: no reference calibration exists (no state file) — this '
                'one is accepted unchecked and becomes the reference. Validate it against '
                'landings before trusting the aim')
        if self._bb_moved_armed and (verdict.overridden or ref_error):
            self._bb_moved_armed = False   # one-shot: consumed by this calibration
        accepted_at = self._persist_accepted(result, verdict.message)
        result_message = (f'Calibration successful (accepted {accepted_at}) · {summary} · '
                          f'{verdict.message}')

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
        self.get_logger().info(
            f'BB calibrated: pos ({pos[0]:.0f}, {pos[1]:.0f}, {pos[2]:.0f}) mm '
            f'· axis tilt {result.axis_tilt_deg:.2f}° '
            f'· yaw offset {math.degrees(result.yaw_offset_rad):+.2f}° '
            f'±{result.yaw_offset_std_deg:.2f}° '
            f'(lag {result.yaw_estimate.lag_s * 1e3:.0f} ms, gate ok) '
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

    def _persist_accepted(self, result: CalibrationResult, verdict: str) -> str:
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
