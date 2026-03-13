from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from datetime import datetime


def generate_launch_description():
    # ── Launch arguments ─────────────────────────────────────────
    record = LaunchConfiguration('record')
    record_arg = DeclareLaunchArgument(
        'record',
        default_value='false',
    )

    # ── CAN node ─────────────────────────────────────────────────
    can_node = Node(
        package='jugglebot',
        executable='can_node',
    )

    # ── Core ROS2 nodes ──────────────────────────────────────────
    orchestrator_node = Node(
        package='jugglebot',
        executable='orchestrator_node',
    )

    motion_bridge_node = Node(
        package='jugglebot',
        executable='motion_bridge_node',
    )

    mocap_node = Node(
        package='jugglebot',
        executable='mocap_node',
    )

    spacemouse_handler = Node(
        package='jugglebot',
        executable='spacemouse_handler',
    )

    # ── Standalone control process (not a ROS2 node) ─────────────
    # Installed as a console_scripts entry point alongside other executables
    pkg_lib_dir = os.path.join(
        get_package_share_directory('jugglebot'), '..', '..', 'lib', 'jugglebot')
    control_loop = ExecuteProcess(
        cmd=[os.path.join(pkg_lib_dir, 'control_loop'), '--rate', '500'],
        output='screen',
    )

    # ── Rosbridge (WebSocket bridge for the GUI) ─────────────────
    rosbridge_launch_file_path = os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml',
    )
    rosbridge_include = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(rosbridge_launch_file_path),
    )

    # ── Rosbag recording (optional) ──────────────────────────────
    bags_dir = os.path.join(os.path.expanduser('~'), 'Desktop', 'rosbags')
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    bag_dir = os.path.join(bags_dir, timestamp)

    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/robot_state',
            '/leg_lengths_topic',
            '/hand_telemetry',
            '/mocap_data',
            '/platform_pose_topic',
            '/rigid_body_poses',
            '/orchestrator_state',
            '/control_mode_topic',
            '/orchestrator_command',
            '/platform_target_reached',
            '/bb/heartbeat',
            '/bb/calibration_result',
            '/qtm_clock_offset_sec',
            '/motion/tracking_error',
            '/motion/diagnostics',
            '-s', 'mcap', '-o', bag_dir,
        ],
        output='screen',
        condition=IfCondition(record),
    )

    # ── Assemble launch description ──────────────────────────────
    return LaunchDescription([
        record_arg,
        # Infrastructure (gui_server.py runs independently in the background)
        rosbridge_include,
        # Hardware nodes
        can_node,
        # Core nodes
        orchestrator_node,
        motion_bridge_node,
        mocap_node,
        spacemouse_handler,
        Node(package='jugglebot', executable='sp_ik'),
        # Standalone processes
        control_loop,
        # Recording (conditional)
        rosbag_record,
    ])
