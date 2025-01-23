import asyncio
import threading
import time
import qtm_rt
import numpy as np
from scipy.optimize import linear_sum_assignment
from typing import Optional, Dict, Tuple

class MocapInterface:
    """
    Class to track a rigid body using QTM data.
    """

    def __init__(self, host: str="192.168.20.20", port: int=22223, logger=None):
        """
        Initialize the RigidBodyTracker.

        Parameters:
        - host: IP address of the QTM server. (ie. the IP address of the host PC)
        - port: Port to connect to QTM.
        """
        self.host = host
        self.port = port
        self.logger = logger

        # Known positions of the markers in the body frame (from the origin to the markers)
        self.base_marker_positions = np.array([
            [-383.49, -42.23, -77.20],
            [-125.29, 417.32, -74.50],
            [-172.27, 400.22, -73.30],
            [172.27, 400.22, -75.10],
            [151.55, 87.50, -73.80],
            [383.49, -42.23, -70.70]
        ])

        # Current transformation from body to world frame
        self.R = np.eye(3)  # Rotation matrix
        self.t = np.zeros(3)  # Translation vector

        # Data storage for markers
        self.labelled_markers = np.empty((0, 4))    # Labelled markers with residuals
        self.unlabelled_markers = np.empty((0, 4))  # Unlabelled markers with residuals

        # Performance statistics
        self.residuals_rigid_body = []
        self.residuals_unlabelled = []
        self.drop_rate = 0
        self.out_of_sync_rate = 0

        # Parameters for periodic update of the base transformation
        self.update_frequency = 10  # Hz
        self.position_threshold = 1.0  # mm (threshold for marker movement)
        self.last_update_time = 0.0

        # Storage for previous markers to check movement
        self.previous_labelled_markers = None

        # Threading lock for data synchronization
        self.data_lock = threading.Lock()

        # Event loop and thread for asynchronous operations
        self.loop = None
        self.thread = None

        self.start()

    async def connect(self):
        """
        Asynchronously connect to QTM and start streaming data.
        """
        try:
            self.connection = await qtm_rt.connect(self.host, port=self.port, timeout=5.0)
            if self.connection is None:
                self.logger.info("Failed to connect to QTM.")
                return

            self.logger.info("Connected to QTM.")

            # Start streaming frames
            await self.start_streaming()
        except asyncio.TimeoutError:
            self.logger.error("Connection to QTM timed out.")
        except Exception as e:
            self.logger.error(f"Error connecting to QTM: {e}")

    async def start_streaming(self):
        """
        Start streaming frames with the required components.
        """
        try:
            await self.connection.stream_frames(
                components=["3dres", "3dnolabelsres"],
                on_packet=self.on_packet
            )
        except Exception as e:
            self.logger.error(f"Error starting frame streaming: {e}")

    def on_packet(self, packet):
        """
        Callback function to process incoming data packets.

        Parameters:
        - packet: The data packet received from QTM.
        """
        current_time = time.time()

        # Update performance statistics from the packet header
        markers_residual = packet.get_3d_markers_residual()
        if markers_residual is not None:
            header, markers = markers_residual
            with self.data_lock:
                self.drop_rate = header.drop_rate
                self.out_of_sync_rate = header.out_of_sync_rate

                # Process labelled markers, skipping NaN positions
                current_labelled = []
                for marker in markers:
                    if not np.isnan([marker.x, marker.y, marker.z]).any():
                        marker_info = [marker.x, marker.y, marker.z, marker.residual]
                        current_labelled.append(marker_info)
                self.labelled_markers = np.array(current_labelled)
        else:
            with self.data_lock:
                self.labelled_markers = np.empty((0, 4))

        # Process unlabelled markers, skipping NaN positions
        markers_no_label_residual = packet.get_3d_markers_no_label_residual()
        if markers_no_label_residual is not None:
            header, markers = markers_no_label_residual
            current_unlabelled = []
            for marker in markers:
                if not np.isnan([marker.x, marker.y, marker.z]).any():
                    marker_info = [marker.x, marker.y, marker.z, marker.residual]
                    current_unlabelled.append(marker_info)
            with self.data_lock:
                self.unlabelled_markers = np.array(current_unlabelled)
        else:
            with self.data_lock:
                self.unlabelled_markers = np.empty((0, 4))

        # Periodically update the rigid body transformation
        if current_time - self.last_update_time >= 1.0 / self.update_frequency:
            if self.should_update_transformation():
                self.find_rigid_body_transformation()
            self.last_update_time = current_time

        # Transform unlabelled markers to body frame
        self.transform_unlabelled_markers()

    def should_update_transformation(self) -> bool:
        """
        Determine whether the rigid body transformation should be updated.

        Returns:
        - True if the transformation should be updated, False otherwise.
        """
        with self.data_lock:
            # Check if previous markers exist and have the same number of markers
            if self.previous_labelled_markers is not None and len(self.labelled_markers) > 0:
                if len(self.previous_labelled_markers) == len(self.labelled_markers):
                    # Compute displacements of markers
                    displacements = np.linalg.norm(
                        self.labelled_markers[:, :3] - self.previous_labelled_markers[:, :3], axis=1
                    )
                    # Update if all markers have moved more than the threshold
                    if np.all(displacements > self.position_threshold):
                        self.logger.info("Base has moved! Updating transformation.", throttle_duration_sec=0.5)
                        return True
                    else:
                        return False
                else:
                    # Number of markers changed; update transformation
                    self.logger.info(f"Number of base markers changed! Now have {len(self.labelled_markers)} Updating transformation.")
                    return True
            else:
                # First time or missing markers; update transformation
                return True

    def find_rigid_body_transformation(self):
        """
        Compute the rigid body transformation based on current labelled markers.
        Updates self.R and self.t.
        """
        with self.data_lock:
            if len(self.labelled_markers) >= 3:
                world_markers = self.labelled_markers[:, :3]
                # Compute transformation
                self.R, self.t, residual = self.compute_transformation(
                    self.base_marker_positions, world_markers
                )
                self.residuals_rigid_body.append(residual)
                # Store current markers for future comparison
                self.previous_labelled_markers = self.labelled_markers.copy()
            else:
                self.logger.info("Not enough labelled markers to compute transformation.", throttle_duration_sec=0.5)

    def compute_transformation(
        self, body_markers: np.ndarray, world_markers: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Compute the rotation and translation from body frame to world frame.

        Parameters:
        - body_markers: Nx3 array of markers in body frame.
        - world_markers: Mx3 array of markers in world frame.

        Returns:
        - R: Rotation matrix (3x3).
        - t: Translation vector (3,).
        - residual: Root mean square error of the fit.
        """
        # Build cost matrix based on distances
        cost_matrix = np.linalg.norm(
            body_markers[:, np.newaxis] - world_markers[np.newaxis, :], axis=2
        )
        # Solve assignment problem (Hungarian Algorithm)
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        # Matched markers
        matched_body_markers = body_markers[row_ind]
        matched_world_markers = world_markers[col_ind]

        # Compute centroids
        centroid_body = np.mean(matched_body_markers, axis=0)
        centroid_world = np.mean(matched_world_markers, axis=0)

        # Center the markers
        centered_body = matched_body_markers - centroid_body
        centered_world = matched_world_markers - centroid_world

        # Compute covariance matrix
        H = centered_body.T @ centered_world

        # Perform Singular Value Decomposition
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        # Correct for reflection if necessary
        if np.linalg.det(R) < 0:
            Vt[2, :] *= -1
            R = Vt.T @ U.T

        # Compute translation vector
        t = centroid_world - R @ centroid_body

        # Compute residuals
        transformed_body_markers = (R @ matched_body_markers.T).T + t
        errors = transformed_body_markers - matched_world_markers
        residual = np.sqrt(np.mean(np.sum(errors ** 2, axis=1)))

        return R, t, residual

    def transform_unlabelled_markers(self):
        """
        Transform unlabelled markers from world frame to body frame.
        """
        with self.data_lock:
            if len(self.unlabelled_markers) > 0:
                positions_world = self.unlabelled_markers[:, :3]
                # Apply inverse transformation
                positions_body = (self.R.T @ (positions_world - self.t).T).T

                # Store transformed positions
                self.unlabelled_markers[:, :3] = positions_body

                # Store residuals
                residuals = self.unlabelled_markers[:, 3]
                self.residuals_unlabelled.extend(residuals.tolist())

    def get_unlabelled_markers_body_frame(self) -> np.ndarray:
        """
        Asynchronously get the current unlabelled markers (and residuals) in the body frame.

        Returns:
        - A numpy array of unlabelled marker positions and residuals in the body frame (Mx4).
        """
        with self.data_lock:
            return self.unlabelled_markers.copy() if len(self.unlabelled_markers) > 0 else np.empty((0, 3))

    def get_performance_statistics(self) -> Dict[str, Optional[float]]:
        """
        Asynchronously get the performance statistics.

        Returns:
        - A dictionary with residuals, drop rate, and out-of-sync rate.
        """
        with self.data_lock:
            stats = {
                'residual_rigid_body': self.residuals_rigid_body[-1] if self.residuals_rigid_body else None,
                'average_residual_unlabelled': np.mean(self.residuals_unlabelled) if self.residuals_unlabelled else None,
                'drop_rate': self.drop_rate,
                'out_of_sync_rate': self.out_of_sync_rate
            }
            return stats

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
            # Schedule the connect coroutine
            self.loop.create_task(self.connect())
            # Run the event loop forever
            self.loop.run_forever()
        except asyncio.CancelledError:
            pass
        finally:
            self.loop.close()

    def stop(self):
        """
        Stop the tracker and close the connection.
        """
        if self.loop:
            if self.connection:
                # Disconnect the connection in the event loop thread
                self.loop.call_soon_threadsafe(self.connection.disconnect)
            # Stop the event loop
            self.loop.call_soon_threadsafe(self.loop.stop)
        if self.thread:
            self.thread.join()

if __name__ == "__main__":
    tracker = MocapInterface()

    try:
        while True:
            mocap_data = tracker.get_unlabelled_markers_body_frame()
            print("Unlabelled markers in body frame:")
            print(mocap_data)
            # stats = tracker.get_performance_statistics()
            # print("Performance statistics:")
            # print(stats)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Shutting down.")

    finally:
        tracker.stop()
        print("MocapInterface stopped.")
