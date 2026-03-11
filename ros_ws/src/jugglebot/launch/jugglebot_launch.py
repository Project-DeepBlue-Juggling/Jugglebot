from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from datetime import datetime


def generate_launch_description():
    # ── Launch arguments ─────────────────────────────────────────
    use_simulator = LaunchConfiguration('use_simulator')
    use_simulator_arg = DeclareLaunchArgument(
        'use_simulator',
        default_value='false',
    )

    record = LaunchConfiguration('record')
    record_arg = DeclareLaunchArgument(
        'record',
        default_value='false',
    )

    # ── Simulator (optional) ─────────────────────────────────────
    simulator_launch_file_path = os.path.join(
        get_package_share_directory('jugglebot_simulator'),
        'launch',
        'launch.py',
    )
    simulator_include_description = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(simulator_launch_file_path),
        condition=IfCondition(use_simulator),
    )

    # ── CAN node (real hardware only) ────────────────────────────
    can_node = Node(
        package='jugglebot',
        executable='can_node',
        condition=UnlessCondition(use_simulator),
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

    mocap_interface_node = Node(
        package='jugglebot',
        executable='mocap_interface_node',
    )

    spacemouse_handler = Node(
        package='jugglebot',
        executable='spacemouse_handler',
    )

    # ── Standalone control process (not a ROS2 node) ─────────────
    control_loop = ExecuteProcess(
        cmd=['python3', '-m', 'jugglebot.motion.control_loop', '--rate', '500'],
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
            '/qtm_clock_offset_sec',
            '/motion/tracking_error',
            '/motion/motor_feedback',
            '/motion/diagnostics',
            '-s', 'mcap', '-o', bag_dir,
        ],
        output='screen',
        condition=IfCondition(record),
    )

    # ── Assemble launch description ──────────────────────────────
    return LaunchDescription([
        use_simulator_arg,
        record_arg,
        # Infrastructure (gui_server.py runs independently in the background)
        rosbridge_include,
        # Simulator (conditional)
        simulator_include_description,
        # Hardware nodes
        can_node,
        # Core nodes
        orchestrator_node,
        # motion_bridge_node,
        # mocap_interface_node,
        spacemouse_handler,
        Node(package='jugglebot', executable='sp_ik'),
        # Standalone processes
        # control_loop,
        # Recording (conditional)
        rosbag_record,
    ])
