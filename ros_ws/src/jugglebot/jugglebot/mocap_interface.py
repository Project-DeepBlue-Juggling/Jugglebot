import asyncio
import collections
import logging
import threading
import time
import qtm_rt
import numpy as np
from typing import Optional, Dict
import xml.etree.ElementTree as ET
from geometry_msgs.msg import PoseStamped # To convert the rigid body pose(s) to a ROS2 message
import jugglebot.hardware_config as hw

class MocapInterface:
    """
    Class to track mocap data using QTM.
    With the new configuration, all incoming marker data (labelled and unlabelled) is in Jugglebot's base frame.
    Only unlabelled markers and full rigid bodies are stored and processed.
    """

    def __init__(self, host: str = hw.MOCAP_QTM_HOST, port: int = hw.MOCAP_QTM_PORT, logger=None, node=None):
        """
        Initialize the tracker.

        Parameters:
        - host: IP address of the QTM server.
        - port: Port to connect to QTM.
        """
        self.host = host
        self.port = port
        self.logger = logger
        self.node = node

        # mm to move in the z direction from the base to the platform in its lowest pos
        self.base_to_platform_transformation = None

        self.ready_to_publish = False # eg. if we haven't received geometry data yet

        # Initialize data to be stored
        self.ball_butler_markers = np.full((5, 4), np.nan)    # (x, y, z, residual)
        self.all_markers = np.empty((0, 4))  # (x, y, z, residual)
        self.labelled_markers: list = []  # list of (label, x, y, z, residual)
        self.body_poses: Dict[str, PoseStamped] = {} # Pose for every rigid body discovered in QTM
        self.body_dict = {}
        self.marker_dict = {}

        # Bodies whose poses are in the world/base frame (not platform frame).
        # Uses raw QTM names — checked BEFORE space/hyphen sanitization.
        self.base_frame_bodies = {"Base", "Ball Butler", "Ball_Butler", "Catching Cone", "Catching_Cone"}

        # ── Alignment check ────────────────────────────────────────────
        self.is_aligned = False  # Updated every frame from the "Base" rigid body
        self._align_pos_thresh_mm = 2.5   # Overridden by set_alignment_thresholds()
        self._align_rot_thresh_deg = 1.0

        # Performance statistics from the incoming packet header.
        # Rolling window: ~5 seconds at 200 Hz
        self.residuals_unlabelled: collections.deque = collections.deque(maxlen=1000)
        self.drop_rate = 0
        self.out_of_sync_rate = 0

        # ── QTM ↔ ROS Clock Sync ───────────────────────────────────────
        # QTM packet.timestamp is in microseconds (monotonic from QTM start).
        # We maintain a running offset: ros_time_ns = qtm_timestamp_us * 1000 + _qtm_to_ros_offset_ns
        # Updated every frame with exponential smoothing to filter network jitter.
        self._qtm_to_ros_offset_ns: Optional[int] = None
        self._qtm_sync_alpha = 0.01  # Smoothing factor (lower = more stable, slower to adapt)
        self._qtm_sync_count = 0
        self._qtm_last_timestamp_us: Optional[int] = None  # For detecting QTM restart
        self._qtm_sync_lock = threading.Lock()

        # Packet freshness tracking — lets the publisher detect when QTM stops
        # sending data, regardless of whether _on_qtm_disconnect fires.
        self._last_packet_time: Optional[float] = None

        # The qtm_rt library logs `LOG.error(...)` on its own "qtm_rt" logger
        # every failed connect attempt (it swallows the OSError and returns
        # None — verified empirically against an unreachable host). That spam
        # is independent of `self.logger`, so we gate the library logger by
        # connection state: silenced while we're in an outage and retrying,
        # restored to its normal level once connected (when its messages —
        # e.g. "Non handled packet type" — are genuinely diagnostic). Start
        # quiet so a QTM-down-at-startup produces zero qtm_rt ERROR lines.
        self._qtm_rt_logger = logging.getLogger("qtm_rt")
        self._qtm_rt_orig_level = self._qtm_rt_logger.level
        self._set_qtm_lib_quiet(True)

        # Threading lock for data synchronization
        self.data_lock = threading.Lock()

        # Flag to request parameter re-fetch from the asyncio thread
        self._params_need_refresh = False

        # Event loop and thread for asynchronous operations
        self.loop = None
        self.thread = None

        self.start()

    #########################################################################################################
    #                                       Connection Management                                           #
    #########################################################################################################

    def start(self):
        """
        Start the tracker in a separate thread.
        """
        self.thread = threading.Thread(target=self._run_asyncio_loop)
        self.thread.start()

    def _run_asyncio_loop(self):
        """
        Run the asyncio event loop in a separate thread.
        """
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.create_task(self.connect())
            self.loop.run_forever()
        except asyncio.CancelledError:
            pass
        finally:
            self.loop.close()

    _RETRY_DELAYS = [2, 5, 10, 30]  # seconds between reconnection attempts

    # Hard ceiling on a single qtm_rt.connect() attempt. qtm_rt's own
    # `timeout=` argument covers only the QRT protocol handshake, NOT TCP
    # connection establishment: if the host resolves but the QTM port is
    # filtered/down, the kernel sits in SYN-retry for minutes and connect()
    # never reaches the retry/except path — so nothing is ever logged. We
    # wrap the call in asyncio.wait_for() to bound it.
    _CONNECT_TIMEOUT_S = 6.0

    # Cadence of the "QTM unavailable" WARNING while disconnected. The
    # message is rclpy-throttled: it logs immediately on the first failure
    # and then at most once per this interval until QTM returns — a
    # reliable, ongoing declaration of the no-connection state without
    # per-attempt spam.
    _OUTAGE_WARN_THROTTLE_S = 30.0

    def _set_qtm_lib_quiet(self, quiet: bool):
        """Silence (or restore) the third-party qtm_rt library's own logger.

        While quiet, qtm_rt's per-attempt connect-failure ERRORs (which are
        expected and handled by our retry loop) are suppressed. Restored to
        the original level once connected so genuine streaming-time errors
        still surface.
        """
        self._qtm_rt_logger.setLevel(
            logging.CRITICAL if quiet else self._qtm_rt_orig_level
        )

    def _on_qtm_disconnect(self, exc):
        """Called by qtm_rt when the connection drops."""
        reason = str(exc) if exc else "unknown"
        # Single WARNING for this mid-session drop (distinct call site, so it
        # logs immediately). The reconnect loop then keeps a throttled
        # "QTM unavailable" WARNING going until QTM returns.
        self.logger.warning(f"QTM disconnected ({reason}) — reconnecting in background")
        self._set_qtm_lib_quiet(True)
        self.connection = None
        # Reset clock sync so stale offsets aren't used during the gap
        with self._qtm_sync_lock:
            self._qtm_to_ros_offset_ns = None
            self._qtm_sync_count = 0
            self._qtm_last_timestamp_us = None
        # Clear all cached data so downstream sees empty state, not stale data
        with self.data_lock:
            self.all_markers = np.empty((0, 4))
            self.labelled_markers = []
            self.ball_butler_markers = np.full((5, 4), np.nan)
            self.body_poses = {}
        self.is_aligned = False
        self._params_need_refresh = False
        if self.loop and self.loop.is_running():
            self.loop.create_task(self.connect())

    async def connect(self):
        """
        Asynchronously connect to QTM and start streaming data.
        Retries with exponential backoff on failure.
        """
        attempt = 0
        while True:
            try:
                # asyncio.wait_for bounds the whole attempt — qtm_rt's own
                # timeout= does not cover TCP connect (see _CONNECT_TIMEOUT_S).
                self.connection = await asyncio.wait_for(
                    qtm_rt.connect(
                        self.host, port=self.port, timeout=5.0,
                        on_disconnect=self._on_qtm_disconnect,
                    ),
                    timeout=self._CONNECT_TIMEOUT_S,
                )
                if self.connection is None:
                    raise ConnectionError("QTM returned None connection")

                self.logger.info("Connected to QTM.")

                # Get 6dof settings from qtm
                xml_6d_string = await self.connection.get_parameters(parameters=["6d"])
                self.body_dict = self.create_body_dict(xml_6d_string)

                # Get 3d settings from qtm
                xml_3d_string = await self.connection.get_parameters(parameters=["3d"])
                self.marker_dict = self.create_marker_dict(xml_3d_string)

                # Start streaming frames with required components.
                await self.start_streaming()
                self._set_qtm_lib_quiet(False)    # restore qtm_rt logging
                return  # streaming started successfully
            except (asyncio.TimeoutError, ConnectionError, OSError) as e:
                delay = self._RETRY_DELAYS[min(attempt, len(self._RETRY_DELAYS) - 1)]
                # str(asyncio.TimeoutError) is "" — fall back to the type name
                # so the line stays informative (e.g. "(TimeoutError)").
                reason = str(e) or type(e).__name__
                # rclpy-throttled: logs immediately on the first failure of an
                # outage, then at most once per _OUTAGE_WARN_THROTTLE_S until
                # QTM returns. A reliable, ongoing declaration of the
                # no-connection state without per-attempt spam.
                self.logger.warning(
                    f"QTM unavailable at {self.host}:{self.port} ({reason}) — "
                    f"retrying in background; will connect automatically "
                    f"when QTM starts",
                    throttle_duration_sec=self._OUTAGE_WARN_THROTTLE_S,
                )
                await asyncio.sleep(delay)
                attempt += 1
            except Exception as e:
                self.logger.error(f"Unexpected error connecting to QTM: {e}")
                return  # Don't retry on unexpected errors

    async def _refresh_parameters(self):
        """Re-fetch 3D/6DOF parameter dictionaries from QTM.

        Called when streaming data suggests the cached dicts are stale
        (e.g. QTM measurement started after we connected).
        """
        try:
            xml_6d = await self.connection.get_parameters(parameters=["6d"])
            new_body_dict = self.create_body_dict(xml_6d)

            xml_3d = await self.connection.get_parameters(parameters=["3d"])
            new_marker_dict = self.create_marker_dict(xml_3d)

            self.body_dict = new_body_dict
            self.marker_dict = new_marker_dict
            self._params_need_refresh = False

            self.logger.info(
                f"QTM parameters refreshed: {len(new_marker_dict)} markers, "
                f"{len(new_body_dict)} bodies"
            )
        except Exception as e:
            self.logger.error(f"Error refreshing QTM parameters: {e}")

    async def start_streaming(self):
        """
        Start streaming frames from QTM.
        """
        try:
            await self.connection.stream_frames(
                components=["3dres", "3dnolabelsres", "6d"],
                on_packet=self.on_packet
            )
        except Exception as e:
            self.logger.error(f"Error starting frame streaming: {e}")

    def is_receiving(self, timeout_s: float = 1.0) -> bool:
        """True if a QTM packet arrived within the last *timeout_s* seconds."""
        return (self._last_packet_time is not None
                and (time.monotonic() - self._last_packet_time) < timeout_s)

    def on_packet(self, packet):
        """
        Callback to process incoming data packets.

        Parameters:
        - packet: The data packet received from QTM.
        """
        self._last_packet_time = time.monotonic()

        # ── Update QTM ↔ ROS clock offset every frame ──────────────────
        # Do this before the ready_to_publish check so sync warms up during startup
        self._update_qtm_clock_sync(packet)

        # ── Extract labelled markers once (reused for staleness check + processing) ──
        markers_residual = packet.get_3d_markers_residual()

        # ── Detect stale parameter dicts and schedule a re-fetch ───────
        # If QTM was started after we connected, marker/body dicts will be empty
        # but the packet will contain data.
        needs_refresh = False
        if markers_residual is not None:
            _, markers_check = markers_residual
            if len(markers_check) > 0 and len(self.marker_dict) == 0:
                needs_refresh = True
            elif len(markers_check) != len(self.marker_dict) and len(self.marker_dict) > 0:
                needs_refresh = True

        sixdof_data = packet.get_6d()
        if sixdof_data is not None:
            _, bodies_check = sixdof_data
            if len(bodies_check) > 0 and len(self.body_dict) == 0:
                needs_refresh = True
            elif len(bodies_check) != len(self.body_dict) and len(self.body_dict) > 0:
                needs_refresh = True

        if needs_refresh and not self._params_need_refresh:
            self._params_need_refresh = True
            self.logger.info("QTM parameter mismatch detected — scheduling refresh")
            self.loop.create_task(self._refresh_parameters())

        # Check if we are ready to publish data
        if not self.ready_to_publish:
            return

        # Process packet header from labelled markers to update performance stats,
        """ Sometimes QTM erroneously labels single markers as being part of a rigid body.
        Since the balls are single markers, this means they will sometimes appear in the labelled markers section.
        To ensure best ball tracking, we broadcast ALL marker data to the rest of the ROS2 network.
        """
        labelled_markers = []
        try:
            if markers_residual is not None:
                header, markers = markers_residual

                # Always store Ball Butler marker positions (negligible cost,
                # needed for calibration and useful for always-on visualisation)
                for i in range(1, 6):
                    marker_name = f"Ball Butler - {i}"
                    if marker_name in self.marker_dict:
                        marker_idx = self.marker_dict[marker_name]
                        if marker_idx < len(markers):
                            marker = markers[marker_idx]
                            with self.data_lock:
                                self.ball_butler_markers[i-1] = [marker.x, marker.y, marker.z, marker.residual]
                        else:
                            with self.data_lock:
                                self.ball_butler_markers[i-1] = [np.nan, np.nan, np.nan, np.nan]
                    else:
                        with self.data_lock:
                            self.ball_butler_markers[i-1] = [np.nan, np.nan, np.nan, np.nan]

                # Get all labelled markers (including Ball Butler)
                for i, marker in enumerate(markers):
                    marker_name = self.get_name_from_index(i, self.marker_dict)
                    if not np.isnan([marker.x, marker.y, marker.z]).any():
                        labelled_markers.append((marker_name, marker.x, marker.y, marker.z, marker.residual))

                with self.data_lock:
                    self.drop_rate = header.drop_rate
                    self.out_of_sync_rate = header.out_of_sync_rate

                    # # Log the marker positions and names
                    # for i, marker in enumerate(markers):
                    #     self.logger.info("\t\tMarker: {}".format(self.get_name_from_index(i, self.marker_dict)))
                    #     self.logger.info("\t\t\tPosition: ({}, {}, {})".format(marker.x, marker.y, marker.z))

        except Exception as e:
            self.logger.error(f"Error processing labelled markers: {e}")

        # Process 6dof data to know the body positions.
        # Build all poses in a local dict, then atomically replace under the
        # lock so readers never see a mix of old and new frame data.
        # Reuse sixdof_data extracted above for staleness check.
        new_body_poses = {}

        if sixdof_data is not None:
            info, bodies = sixdof_data
        else:
            bodies = []

        for i, body in enumerate(bodies):
            raw_name = self.get_name_from_index(i, self.body_dict)
            is_base_frame = raw_name in self.base_frame_bodies

            # Sanitise name for ROS topic/TF compatibility
            body_name = raw_name.replace(' ', '_').replace('-', '_')

            # Compose a PoseStamped for this rigid body
            pose = PoseStamped()
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.header.frame_id = "world" if is_base_frame else "platform_start"

            pose.pose.position.x = body[0].x
            pose.pose.position.y = body[0].y

            # Leave base-frame bodies' Z unchanged; shift others into the platform frame
            if is_base_frame:
                pose.pose.position.z = body[0].z
            elif self.base_to_platform_transformation is not None:
                pose.pose.position.z = body[0].z - self.base_to_platform_transformation
            else:
                pose.pose.position.z = body[0].z

            qx, qy, qz, qw = self.rotation_list_to_quaternion(body[1].matrix)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            new_body_poses[body_name] = pose

            # ── Alignment check: "Base" body should be near the global origin ──
            if body_name == "Base":
                pos_dist = np.sqrt(body[0].x**2 + body[0].y**2 + body[0].z**2)
                # Rotation angle from identity: 2 * acos(|qw|)
                rot_angle_deg = np.degrees(2.0 * np.arccos(np.clip(abs(qw), 0.0, 1.0)))
                aligned = bool(pos_dist <= self._align_pos_thresh_mm
                              and rot_angle_deg <= self._align_rot_thresh_deg)
                if self.is_aligned and not aligned:
                    self.logger.warning(
                        f"Mocap base misaligned: pos={pos_dist:.2f} mm, "
                        f"rot={rot_angle_deg:.2f}° (thresholds: "
                        f"{self._align_pos_thresh_mm} mm, "
                        f"{self._align_rot_thresh_deg}°)"
                    )
                elif not self.is_aligned and aligned:
                    self.logger.info("Mocap base aligned")
                self.is_aligned = aligned

        with self.data_lock:
            self.body_poses = new_body_poses

        # Process unlabelled markers, skipping NaN positions.
        current_unlabelled = []
        markers_no_label_residual = packet.get_3d_markers_no_label_residual()
        if markers_no_label_residual is not None:
            header, markers = markers_no_label_residual
            for marker in markers:
                if not np.isnan([marker.x, marker.y, marker.z]).any():
                    current_unlabelled.append([marker.x, marker.y, marker.z, marker.residual])

        with self.data_lock:
            all_positions = current_unlabelled
            if all_positions:
                self.all_markers = np.array(all_positions)
            else:
                self.all_markers = np.empty((0, 4))
            self.labelled_markers = labelled_markers  # list of (label, x, y, z, residual)

        # Update unlabelled residual statistics.
        with self.data_lock:
            if self.all_markers.size > 0:
                residuals = self.all_markers[:, 3]
                self.residuals_unlabelled.extend(residuals.tolist())

    def stop(self):
        """
        Stop the tracker and close the connection.
        """
        if self.loop:
            if hasattr(self, 'connection') and self.connection:
                self.loop.call_soon_threadsafe(self.connection.disconnect)
            self.loop.call_soon_threadsafe(self.loop.stop)
        if self.thread:
            self.thread.join()

    #########################################################################################################
    #                                      QTM ↔ ROS Clock Sync                                            #
    #########################################################################################################

    def _update_qtm_clock_sync(self, packet):
        """Update the QTM-to-ROS clock offset using the packet timestamp.

        QTM's packet.timestamp is in microseconds (monotonic from measurement start).
        We pair it with the current ROS clock reading and maintain an exponentially
        smoothed offset so that:
            ros_time_ns ≈ qtm_timestamp_us * 1000 + offset_ns

        The first few samples use direct assignment to converge quickly, then
        switch to exponential smoothing to filter network jitter.
        """
        if self.node is None:
            return

        try:
            qtm_us = packet.timestamp  # QTM camera time in microseconds
            ros_ns = self.node.get_clock().now().nanoseconds  # ROS time in nanoseconds

            qtm_ns = int(qtm_us) * 1000  # Convert QTM μs → ns
            measured_offset = ros_ns - qtm_ns

            with self._qtm_sync_lock:
                # Detect QTM restart: timestamp jumped backwards or by > 5 s
                if self._qtm_last_timestamp_us is not None:
                    dt_us = qtm_us - self._qtm_last_timestamp_us
                    if dt_us < 0 or dt_us > 5_000_000:
                        self.logger.warning(
                            f"QTM timestamp discontinuity ({dt_us / 1e6:.3f} s) — "
                            "resetting clock sync"
                        )
                        self._qtm_to_ros_offset_ns = None
                        self._qtm_sync_count = 0
                self._qtm_last_timestamp_us = qtm_us

                if self._qtm_to_ros_offset_ns is None:
                    # First sample (or after reset) — initialise directly
                    self._qtm_to_ros_offset_ns = measured_offset
                    self._qtm_sync_count = 1
                    self.logger.info(
                        f"QTM clock sync initialised: offset = {measured_offset / 1e9:.6f} s"
                    )
                else:
                    self._qtm_sync_count += 1

                    # Use direct averaging for the first 100 samples to converge fast,
                    # then switch to exponential smoothing
                    if self._qtm_sync_count <= 100:
                        alpha = 1.0 / self._qtm_sync_count
                    else:
                        alpha = self._qtm_sync_alpha

                    self._qtm_to_ros_offset_ns = int(
                        self._qtm_to_ros_offset_ns + alpha * (measured_offset - self._qtm_to_ros_offset_ns)
                    )

        except Exception as e:
            self.logger.error(f"QTM clock sync error: {e}", throttle_duration_sec=5.0)

    def qtm_timestamp_to_ros_ns(self, qtm_timestamp_us: int) -> Optional[int]:
        """Convert a QTM timestamp (microseconds) to ROS time (nanoseconds).

        Returns None if the clock sync has not yet been initialised.
        """
        with self._qtm_sync_lock:
            if self._qtm_to_ros_offset_ns is None:
                return None
            return int(qtm_timestamp_us) * 1000 + self._qtm_to_ros_offset_ns

    def qtm_timestamp_to_ros_sec(self, qtm_timestamp_us: int) -> Optional[float]:
        """Convert a QTM timestamp (microseconds) to ROS time (seconds as float).

        Returns None if the clock sync has not yet been initialised.
        """
        ns = self.qtm_timestamp_to_ros_ns(qtm_timestamp_us)
        return ns / 1e9 if ns is not None else None

    def get_qtm_sync_status(self) -> dict:
        """Return the current QTM clock sync status for diagnostics."""
        with self._qtm_sync_lock:
            return {
                "synced": self._qtm_to_ros_offset_ns is not None,
                "offset_s": self._qtm_to_ros_offset_ns / 1e9 if self._qtm_to_ros_offset_ns is not None else None,
                "sample_count": self._qtm_sync_count,
            }

    #########################################################################################################
    #                                  Package Data For Parent Modules                                      #
    #########################################################################################################

    def get_all_markers_base_frame(self) -> np.ndarray:
        """
        Get the current unlabelled markers (and residuals) in the base frame.

        Returns:
        - A numpy array of unlabelled marker data (Mx4).
        """
        with self.data_lock:
            return self.all_markers.copy() if self.all_markers.size > 0 else np.empty((0, 4))

    def get_labelled_markers(self) -> list:
        """
        Get the current labelled markers with their names.

        Returns:
        - A list of (label, x, y, z, residual) tuples.
        """
        with self.data_lock:
            return list(self.labelled_markers)

    def get_ball_butler_markers_base_frame(self) -> np.ndarray:
        """
        Get the current ball butler markers (and residuals) in the base frame.

        Returns:
        - A numpy array of ball butler marker data (Nx4).
        """
        with self.data_lock:
            return self.ball_butler_markers.copy() if self.ball_butler_markers.size > 0 else np.empty((0, 4))

    def get_body_poses(self) -> Dict[str, PoseStamped]:
        """Return a shallow copy of the latest poses for all rigid bodies."""
        with self.data_lock:
            return self.body_poses.copy()

    def clear_body_poses(self):
        """Clear stored poses so we don't publish duplicates."""
        with self.data_lock:
            self.body_poses.clear()

    def get_performance_statistics(self) -> Dict[str, Optional[float]]:
        """
        Get the performance statistics.

        Returns:
        - A dictionary with the latest average unlabelled residual, drop rate, and out-of-sync rate.
        """
        with self.data_lock:
            stats = {
                'average_residual_unlabelled': np.mean(self.residuals_unlabelled) if self.residuals_unlabelled else None,
                'drop_rate': self.drop_rate,
                'out_of_sync_rate': self.out_of_sync_rate
            }
            return stats

    #########################################################################################################
    #                                          Helper Methods                                               #
    #########################################################################################################

    def create_body_dict(self, xml_string):
        """ Extract a name to index dictionary from 6dof settings xml """
        xml = ET.fromstring(xml_string)
        
        body_dict = {}
        for index, body in enumerate(xml.findall("*/Body/Name")):
            body_dict[body.text.strip()] = index

        return body_dict

    def create_marker_dict(self, xml_string):
        """ Extract a name to index dictionary from 3d settings xml """
        xml = ET.fromstring(xml_string)
        
        marker_dict = {}
        for index, marker in enumerate(xml.findall("*/Label/Name")):
            marker_dict[marker.text.strip()] = index

        return marker_dict
    
    def get_name_from_index(self, index, name_dict):
        for name, i in name_dict.items():
            if i == index:
                return name
        return ""

    def rotation_list_to_quaternion(self, R_list):
        """
        Convert a 9-element list (representing a column-major 3x3 rotation matrix) 
        to a quaternion (w, x, y, z).

        R_list = [r11, r21, r31, r12, r22, r32, r13, r23, r33]
        
        Args:
            R_list (list or numpy.ndarray): 9-element list ordered column-wise.
            
        Returns:
            numpy.ndarray: Quaternion as [w, x, y, z].
        """
        R = np.array(R_list, dtype=np.float64).reshape((3, 3), order='F')
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        
        if trace > 0:
            S = 2.0 * np.sqrt(trace + 1.0)
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        else:
            i = np.argmax(np.diag(R))
            if i == 0:
                S = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                qw = (R[2, 1] - R[1, 2]) / S
                qx = 0.25 * S
                qy = (R[0, 1] + R[1, 0]) / S
                qz = (R[0, 2] + R[2, 0]) / S
            elif i == 1:
                S = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                qw = (R[0, 2] - R[2, 0]) / S
                qx = (R[0, 1] + R[1, 0]) / S
                qy = 0.25 * S
                qz = (R[1, 2] + R[2, 1]) / S
            else:
                S = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                qw = (R[1, 0] - R[0, 1]) / S
                qx = (R[0, 2] + R[2, 0]) / S
                qy = (R[1, 2] + R[2, 1]) / S
                qz = 0.25 * S

        return np.array([qx, qy, qz, qw])

    def clear_markers(self):
        """Clear both labelled and unlabelled marker data after publishing."""
        with self.data_lock:
            self.all_markers = np.empty((0, 4))
            self.labelled_markers = []

    #########################################################################################################
    #                                        Get Robot Geometry                                             #
    #########################################################################################################

    def set_base_to_platform_offset(self, offset: float):
        """
        Set the offset from the base to the platform.

        Parameters:
        - offset: The offset in mm.
        """
        self.base_to_platform_transformation = offset

    def set_alignment_thresholds(self, pos_mm: float, rot_deg: float):
        """Set thresholds for the base-body alignment check."""
        self._align_pos_thresh_mm = pos_mm
        self._align_rot_thresh_deg = rot_deg


if __name__ == "__main__":
    tracker = MocapInterface()
    if tracker.logger is None:
        logger = logging.getLogger("MocapInterface")
        handler = logging.StreamHandler()
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
        tracker.logger = logger

    try:
        while True:
            unlabelled_data = tracker.get_all_markers_base_frame()
            print("Unlabelled markers in base frame:")
            print(unlabelled_data)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Shutting down.")

    finally:
        tracker.stop()
        print("MocapInterface stopped.")