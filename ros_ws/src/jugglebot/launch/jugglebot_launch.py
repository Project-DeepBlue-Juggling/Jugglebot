from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription, 
    DeclareLaunchArgument,
    ExecuteProcess
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from datetime import datetime

def generate_launch_description():
    use_simulator = LaunchConfiguration('use_simulator')
    use_simulator_arg = DeclareLaunchArgument(
        'use_simulator',
        default_value='false'
    )

    simulator_launch_file_path = os.path.join(
        get_package_share_directory('jugglebot_simulator'),
        'launch',
        'launch.py'
    )
    simulator_include_description = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(simulator_launch_file_path),
        condition=IfCondition(use_simulator)
    )

    can_bus_node = Node(
        package='jugglebot',
        executable='can_interface_node',
        condition=UnlessCondition(use_simulator)
    )

    jugglebot_node_names = [
        # 'yasmin_state_machine',
        # 'spacemouse_handler',
        # 'sp_ik',
        'robot_geometry',
        # 'level_platform_node',
        'mocap_interface_node',
        'ball_prediction_node',
        # 'catch_thrown_ball_node',
        # 'catch_dropped_ball_node',
        # 'calibrate_platform_node',
        # 'pose_correction_node',
        # 'hoop_sinker_node',
        # 'ball_butler_node',
        'ball_butler_volley_testing_node',
        'target_tracker_node',
    ]
    jugglebot_nodes = [
        Node(
            package='jugglebot',
            # namespace='jugglebot',
            executable=node_name,
            # name=node_name,
            # arguments=['--ros-args', '--log-level', 'debug']
            # parameters=[{'use_sim_time': True}],
        ) for node_name in jugglebot_node_names
    ]


    rosbridge_launch_file_path = os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml'
    )
    rosbridge_include_description = IncludeLaunchDescription(AnyLaunchDescriptionSource(rosbridge_launch_file_path))

    # ROS2 Bag file creation
    file_path = os.path.join(os.path.expanduser('~'), 'Desktop')
    bags_dir = os.path.join(file_path, 'rosbags')

    # Generate a timestamped directory name
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    bag_dir = os.path.join(bags_dir, f"session_{timestamp}")

    record = LaunchConfiguration('record')
    record_arg = DeclareLaunchArgument(
        'record',
        default_value='false'
    )

    # rosbag2 record command
    rosbag_record_cmd = [
        'ros2', 'bag', 'record', '/robot_state', '/leg_lengths_topic', '/hand_trajectory', '/mocap_data', '/platform_pose_topic',
        '/rigid_body_poses', '/settled_leg_lengths', '/settled_platform_poses', '/hand_telemetry', '/throw_debug',
        '/balls', '/targets', '/throw_announcements', 'bb/heartbeat',
        '-s', 'mcap', '-o', bag_dir
    ]

    rosbag_record = ExecuteProcess(
        cmd=rosbag_record_cmd,
        output='screen',
        condition=IfCondition(record)
    )

    return LaunchDescription([
        use_simulator_arg,
        record_arg,
        rosbridge_include_description,
        simulator_include_description,
        can_bus_node,
        *jugglebot_nodes,
        rosbag_record
    ])
