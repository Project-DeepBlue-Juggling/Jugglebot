import asyncio
import logging
import threading
import time
import qtm_rt
import numpy as np
from typing import Optional, Dict
import xml.etree.ElementTree as ET
from geometry_msgs.msg import PoseStamped # To convert the rigid body pose(s) to a ROS2 message

class MocapInterface:
    """
    Class to track mocap data using QTM.
    With the new configuration, all incoming marker data (labelled and unlabelled) is in Jugglebot's base frame.
    Only unlabelled markers and full rigid bodies are stored and processed.
    """

    def __init__(self, host: str = "192.168.20.9", port: int = 22223, logger=None, node=None):
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
        self.unlabelled_markers = np.empty((0, 4))  # (x, y, z, residual)
        self.body_poses: Dict[str, PoseStamped] = {} # Pose for every rigid body discovered in QTM
        self.body_dict = {}
        self.marker_dict = {}

        # Performance statistics from the incoming packet header.
        self.residuals_unlabelled = []
        self.drop_rate = 0
        self.out_of_sync_rate = 0

        # Threading lock for data synchronization
        self.data_lock = threading.Lock()

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

            # Get 6dof settings from qtm
            xml_6d_string = await self.connection.get_parameters(parameters=["6d"])
            self.body_dict = self.create_body_dict(xml_6d_string)

            # Get 3d settings from qtm
            xml_3d_string = await self.connection.get_parameters(parameters=["3d"])
            self.marker_dict = self.create_marker_dict(xml_3d_string)

            # Start streaming frames with required components.
            await self.start_streaming()
        except asyncio.TimeoutError:
            self.logger.error("Connection to QTM timed out.")
        except Exception as e:
            self.logger.error(f"Error connecting to QTM: {e}")

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

    def on_packet(self, packet):
        """
        Callback to process incoming data packets.

        Parameters:
        - packet: The data packet received from QTM.
        """
        # Check if we are ready to publish data
        if not self.ready_to_publish:
            return

        # Process packet header from labelled markers to update performance stats,
        # but ignore the labelled marker positions.
        markers_residual = packet.get_3d_markers_residual()
        if markers_residual is not None:
            header, markers = markers_residual
            with self.data_lock:
                self.drop_rate = header.drop_rate
                self.out_of_sync_rate = header.out_of_sync_rate

                # # Log the marker positions and names
                # for i, marker in enumerate(markers):
                #     self.logger.info("\t\tMarker: {}".format(self.get_name_from_index(i, self.marker_dict)))
                #     self.logger.info("\t\t\tPosition: ({}, {}, {})".format(marker.x, marker.y, marker.z))

        # Process 6dof data to know the body positions.
        info, bodies = packet.get_6d()
            
        for i, body in enumerate(bodies):
            # Log the body positions and rotations
            # self.logger.info("Body: {}".format(self.get_name_from_index(i, self.body_dict)))
            # self.logger.info("\tPosition: ({}, {}, {})".format(body[0].x, body[0].y, body[0].z))
            # self.logger.info(f"\tRotation: {self.rotation_list_to_quaternion(body[1].matrix)}")

            body_name = self.get_name_from_index(i, self.body_dict)

            # Detect if 'body_name' has any unwanted characters. If any are present, replace them with an underscore
            if any(char in body_name for char in [' ', '-']):
                body_name = body_name.replace(' ', '_')
                body_name = body_name.replace('-', '_')
                
            # Compose a PoseStamped for this rigid body
            pose = PoseStamped()
            pose.header.stamp = self.node.get_clock().now().to_msg()
            if body_name == "Base":
                pose.header.frame_id = "world"
            else:
                pose.header.frame_id = "platform_start"

            pose.pose.position.x = body[0].x
            pose.pose.position.y = body[0].y

            # Leave the Base Z position unchanged, but apply the base to platform transformation to other bodies
            if self.base_to_platform_transformation is not None and body_name == "Base":
                pose.pose.position.z = body[0].z
            else:
                pose.pose.position.z = body[0].z - self.base_to_platform_transformation

            qx, qy, qz, qw = self.rotation_list_to_quaternion(body[1].matrix)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            # Store (or overwrite) the latest pose for this body
            with self.data_lock:
                self.body_poses[body_name] = pose

        # Process unlabelled markers, skipping NaN positions.
        markers_no_label_residual = packet.get_3d_markers_no_label_residual()
        if markers_no_label_residual is not None:
            header, markers = markers_no_label_residual
            current_unlabelled = []
            for marker in markers:
                if not np.isnan([marker.x, marker.y, marker.z]).any():
                    current_unlabelled.append([marker.x, marker.y, marker.z, marker.residual])
            with self.data_lock:
                self.unlabelled_markers = np.array(current_unlabelled)
        else:
            with self.data_lock:
                self.unlabelled_markers = np.empty((0, 4))

        # Update unlabelled residual statistics.
        with self.data_lock:
            if self.unlabelled_markers.size > 0:
                residuals = self.unlabelled_markers[:, 3]
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
    #                                  Package Data For Parent Modules                                      #
    #########################################################################################################

    def get_unlabelled_markers_base_frame(self) -> np.ndarray:
        """
        Get the current unlabelled markers (and residuals) in the base frame.

        Returns:
        - A numpy array of unlabelled marker data (Mx4).
        """
        with self.data_lock:
            return self.unlabelled_markers.copy() if self.unlabelled_markers.size > 0 else np.empty((0, 4))

    def get_body_poses(self) -> Dict[str, PoseStamped]:
        """Return a shallow copy of the latest poses for all rigid bodies."""
        with self.data_lock:
            return self.body_poses.copy()

    def clear_body_poses(self):
        """Clear stored poses so we don’t publish duplicates."""
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

    def clear_unlabelled_markers(self):
        """
        Clear the unlabelled marker data.
        """
        with self.data_lock:
            self.unlabelled_markers = np.empty((0, 4))

    def clear_platform_pose(self):
        """
        Clear the platform pose data.
        """
        with self.data_lock:
            self.platform_pose = None

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
            unlabelled_data = tracker.get_unlabelled_markers_base_frame()
            print("Unlabelled markers in base frame:")
            print(unlabelled_data)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Shutting down.")

    finally:
        tracker.stop()
        print("MocapInterface stopped.")
