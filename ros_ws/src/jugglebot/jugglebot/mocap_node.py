"""Mocap Interface Node — QTM streaming, tf2 broadcast, BB calibration.

Publishes:
  mocap_data             (MocapDataMulti)   — all markers (labelled + unlabelled) at 200 Hz
  rigid_body_poses       (RigidBodyPoses)   — all rigid bodies at 200 Hz
  bb/markers             (MocapDataMulti)   — BB fiducial markers (always, when QTM connected)
  bb/calibration_result  (BallButlerCalibrationResult) — latched, after each calibration
  qtm_clock_offset_sec   (Float64)          — QTM↔ROS clock offset at 1 Hz

Subscribes:
  bb/heartbeat           (BallButlerHeartbeat) — toggles marker publishing + calibration

Static TF:
  world → platform_start  (Z offset = GEOM_INITIAL_HEIGHT_MM)
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import Float64
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
from .bb_calibration import run_calibration, CalibrationResult


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

        # ── Timers ────────────────────────────────────────────────────────
        self.create_timer(1.0, self._publish_clock_offset)
        self.create_timer(hw.TRACKING_MOCAP_DT_S, self._publish_mocap_data)

        # ── BB heartbeat subscription ─────────────────────────────────────
        self.create_subscription(
            BallButlerHeartbeat, 'bb/heartbeat', self._on_bb_heartbeat, 10
        )
        self._bb_last_state: int | None = None

        # ── Calibration state ─────────────────────────────────────────────
        self._calibrating = False
        self._calib_data: dict[int, list[np.ndarray]] = {}
        self._calib_yaw_readings: list[float] = []

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

                # Accumulate for calibration
                if self._calibrating:
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
        if self._calibrating:
            self._calib_yaw_readings.append(msg.yaw_deg)

        if msg.state != self._bb_last_state:
            self._bb_last_state = msg.state

        # Detect calibration start
        if msg.state == BallButlerStates.CALIBRATING and not self._calibrating:
            self._calibrating = True
            self._calib_data = {i: [] for i in range(5)}
            self._calib_yaw_readings = []
            self.get_logger().info('BB calibration started — collecting marker data')

        # Detect calibration end (state transition away from CALIBRATING)
        if previous == BallButlerStates.CALIBRATING and msg.state != BallButlerStates.CALIBRATING:
            if self._calibrating:
                if msg.state == BallButlerStates.ERROR:
                    self.get_logger().warn('Calibration aborted: BB entered ERROR state')
                    self._publish_calibration_failure('BB entered ERROR state during calibration')
                else:
                    self.get_logger().info(
                        f'Calibration sweep complete (new state: {msg.state}). Processing...'
                    )
                    self._finalize_calibration()
                self._calibrating = False
                self._calib_data = {}
                self._calib_yaw_readings = []

    # ──────────────────────────────────────────────────────────────────────
    #  Calibration processing
    # ──────────────────────────────────────────────────────────────────────

    def _accumulate_calibration_markers(self, msg: MocapDataMulti):
        """Store each marker position from a bb/markers message."""
        for i, marker in enumerate(msg.markers):
            if i >= 5:
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

        for idx, m in result.marker_metrics.items():
            if m.status == 'ok':
                self.get_logger().info(
                    f'Marker {idx + 1}: radius={m.radius_mm:.1f} mm, '
                    f'residual={m.fit_residual_mm:.3f} mm, '
                    f'axis_dev={m.distance_from_axis_mm:.3f} mm'
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
