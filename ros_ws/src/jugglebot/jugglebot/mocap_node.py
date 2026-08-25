"""Mocap Interface Node — QTM streaming, tf2 broadcast, BB calibration.

Publishes:
  mocap_data             (MocapDataMulti)   — all markers (labelled + unlabelled) at 200 Hz
  rigid_body_poses       (RigidBodyPoses)   — all rigid bodies at 200 Hz
  bb/markers             (MocapDataMulti)   — BB fiducial markers (always, when QTM connected)
  bb/calibration_result  (BallButlerCalibrationResult) — latched, after each calibration
  qtm_clock_offset_sec   (Float64)          — QTM↔ROS clock offset at 1 Hz
  mocap/status           (DiagnosticStatus) — QTM reception + BB fiducial visibility at 5 Hz

Subscribes:
  bb/heartbeat           (BallButlerHeartbeat) — toggles marker publishing + calibration

Static TF:
  world → platform_start  (Z offset = GEOM_INITIAL_HEIGHT_MM)
"""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import Float64
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
from .bb_calibration import run_calibration, CalibrationResult, MIN_ARC_DEG


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

#: BB carries five fiducials; ``mocap_interface.ball_butler_markers`` is a fixed
#: (5, 4) array with NaN rows for the ones QTM cannot see this frame.
BB_MARKER_COUNT = 5

#: Index of "Marker 3" — the one ``bb_calibration.run_calibration`` hard-requires
#: for the yaw offset. Zero-based, so Marker 3 is index 2.
BB_MARKER3_INDEX = 2


class MocapNode(Node):
    def __init__(self):
        super().__init__('mocap_node')

        # ── Mocap interface (pure-Python, runs its own asyncio thread) ────
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

        self.get_logger().info('MocapNode initialised')

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
        """(count of BB fiducials QTM currently resolves, Marker-3 visible).

        ``MocapInterface.ball_butler_markers`` is a persistent (5, 4) array
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
        marker3 = bool(visible[BB_MARKER3_INDEX]) if visible.shape[0] > BB_MARKER3_INDEX else False
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
        ``bb/calibration_result`` last carried:

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
                # again: bb/calibration_result is latched, so a second publish
                # would overwrite 'QTM_DROPOUT_MID_SWEEP' — the thing the
                # operator actually needs to read — with the vaguer timeout.
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
                'QTM stopped delivering frames during the sweep — the collected '
                'arc has a hole, so the fit is not trustworthy')

    def _invalidate_calibration(self, code: str, detail: str):
        """Latch a collection window invalid and publish the named failure."""
        self._calib_invalid = code
        message = f'{code}: {detail}'
        self.get_logger().error(f'Calibration invalidated — {message}')
        self._publish_calibration_failure(message)

    def _end_calibration(self, *, blocked: bool = False):
        """Close the collection window and drop its data."""
        self._calibrating = False
        self._calib_blocked = blocked
        self._calib_data = {}
        self._calib_yaw_readings = []

    def _publish_mocap_data(self):
        is_aligned = self.mocap.is_aligned

        # ── Stop publishing when QTM packets aren't arriving ──
        # The GUI has its own 2-second timeout that sets disconnected/unaligned
        # and clears markers, so going silent here is the correct behaviour.
        if not self.mocap.is_receiving():
            return

        unlabelled = self.mocap.get_all_markers_base_frame()
        labelled = self.mocap.get_labelled_markers()
        try:
            msg = MocapDataMulti()
            msg.aligned = is_aligned

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
        self.get_logger().info(
            f'Broadcast static tf: world -> platform_start (z_offset={z_offset_mm:.1f} mm)'
        )

    # ──────────────────────────────────────────────────────────────────────
    #  Ball Butler heartbeat — calibration state tracking
    # ──────────────────────────────────────────────────────────────────────

    def _on_bb_heartbeat(self, msg: BallButlerHeartbeat):
        previous = self._bb_last_state

        # Record yaw during calibration for offset calculation
        if self._calibrating and self._calib_invalid is None:
            self._calib_yaw_readings.append(msg.yaw_deg)

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
            self.get_logger().info('BB calibration started — collecting marker data')

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
                    self.get_logger().warn('Calibration aborted: BB entered ERROR state')
                    self._publish_calibration_failure('BB entered ERROR state during calibration')
                else:
                    self.get_logger().info(
                        f'Calibration sweep complete (new state: {msg.state}). Processing...'
                    )
                    self._finalize_calibration()
                self._end_calibration()

    # ──────────────────────────────────────────────────────────────────────
    #  Calibration processing
    # ──────────────────────────────────────────────────────────────────────

    def _accumulate_calibration_markers(self, msg: MocapDataMulti):
        """Store each marker position from a bb/markers message."""
        for i, marker in enumerate(msg.markers):
            if i >= BB_MARKER_COUNT:
                break
            pos = marker.position
            if math.isnan(pos.x) or math.isnan(pos.y) or math.isnan(pos.z):
                continue
            self._calib_data[i].append(np.array([pos.x, pos.y, pos.z]))

    def _finalize_calibration(self):
        """Run the calibration pipeline and publish the result."""
        total_pts = sum(len(v) for v in self._calib_data.values())
        if total_pts == 0:
            self._publish_calibration_failure('No marker data collected')
            return

        for idx, pts in self._calib_data.items():
            if pts:
                self.get_logger().info(f'Marker {idx + 1}: {len(pts)} valid samples')

        try:
            result: CalibrationResult = run_calibration(
                calibration_data=self._calib_data,
                yaw_readings_deg=self._calib_yaw_readings,
                pitch_z_offset_mm=hw.BB_GEOM_PITCH_Z_OFFSET_MM,
            )
        except ValueError as e:
            self.get_logger().error(f'Calibration failed: {e}')
            self._publish_calibration_failure(str(e))
            return

        # Log results
        self.get_logger().info('=' * 50)
        self.get_logger().info('BB CALIBRATION RESULTS')
        self.get_logger().info('=' * 50)
        pos = result.bb_position_mm
        self.get_logger().info(
            f'Position: X={pos[0]:.2f}, Y={pos[1]:.2f}, Z={pos[2]:.2f} mm'
        )
        self.get_logger().info(
            f'Axis direction: ({result.axis_direction[0]:.4f}, '
            f'{result.axis_direction[1]:.4f}, {result.axis_direction[2]:.4f})'
        )
        self.get_logger().info(f'Axis tilt from vertical: {result.axis_tilt_deg:.2f}°')
        self.get_logger().info(
            f'Yaw offset: {math.degrees(result.yaw_offset_rad):.2f}° '
            f'({result.yaw_offset_rad:.4f} rad)'
        )
        self.get_logger().info(f'Yaw uncertainty: ±{result.yaw_offset_std_deg:.2f}°')
        # THE number MIN_ARC_DEG was set from — 118.8° on the first hardware
        # calibrate, 2026-08-25 (plans/archived/operator-observability.md § 8
        # item 4, RESOLVED). It is BB's own reported yaw span — encoder-derived,
        # so unlike the per-marker arc_span below it is not inflated by QTM
        # marker noise.
        self.get_logger().info(
            f'BB yaw span swept: {result.yaw_span_deg:.1f}° '
            f'(floor MIN_ARC_DEG={MIN_ARC_DEG:.1f}°)')

        for idx, m in result.marker_metrics.items():
            if m.status == 'ok':
                self.get_logger().info(
                    f'Marker {idx + 1}: radius={m.radius_mm:.1f} mm, '
                    f'residual={m.fit_residual_mm:.3f} mm, '
                    f'axis_dev={m.distance_from_axis_mm:.3f} mm, '
                    f'arc_span={m.arc_span_deg:.1f}°'
                )
            else:
                self.get_logger().warn(f'Marker {idx + 1}: {m.status} — {m.reason}')

        self.get_logger().info('=' * 50)

        # Publish on latched topic
        self._publish_calibration_result(result)

    def _publish_calibration_result(self, result: CalibrationResult):
        msg = BallButlerCalibrationResult()
        msg.success = True
        msg.message = 'Calibration successful'
        msg.position_mm.x = float(result.bb_position_mm[0])
        msg.position_mm.y = float(result.bb_position_mm[1])
        msg.position_mm.z = float(result.bb_position_mm[2])
        msg.yaw_offset_rad = result.yaw_offset_rad
        msg.yaw_offset_std_deg = result.yaw_offset_std_deg
        msg.axis_tilt_deg = result.axis_tilt_deg
        self.pub_calibration.publish(msg)
        self.get_logger().info('Published calibration result on bb/calibration_result')

    def _publish_calibration_failure(self, reason: str):
        msg = BallButlerCalibrationResult()
        msg.success = False
        msg.message = reason
        self.pub_calibration.publish(msg)
        self.get_logger().error(f'Published calibration failure: {reason}')

    # ──────────────────────────────────────────────────────────────────────
    #  Lifecycle
    # ──────────────────────────────────────────────────────────────────────

    def on_shutdown(self):
        self.get_logger().info('Shutting down MocapNode...')
        self.mocap.stop()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MocapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received. Shutting down.')
    finally:
        node.on_shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
