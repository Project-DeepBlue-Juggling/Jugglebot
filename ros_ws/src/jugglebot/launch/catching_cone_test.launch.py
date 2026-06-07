"""Trimmed launch for catching-cone throw-timing tests.

Starts ONLY the nodes required to drive Ball Butler at the catching cone
and read back per-catch timing results — no platform-side processes.  Use
this when Jugglebot itself is offline (e.g. the cone is plugged into the
shared CAN cable while the rest of the platform is powered down).

What's in:
    - can_node                  CAN transport + BB throw service + cone decode
    - mocap_node                QTM → rigid_body_poses, bb/calibration_result
    - catch_correlation_node    matches catches against throw announcements
    - ball_butler_node          resolves target by name + solves inverse + throws
    - rosbridge_websocket       GUI connectivity
    - rosbag (optional)         records the topics needed to audit a session

What's out (vs jugglebot_launch.py):
    motion_bridge_node, mpc_bridge_node, motor_guard, orchestrator_node,
    catch_coordinator_node, spacemouse_handler, ball_tracker_node.

Launch:
    ros2 launch jugglebot catching_cone_test.launch.py

Optional args:
    record:=false             # don't record a bag
"""
import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    record = LaunchConfiguration('record')
    record_arg = DeclareLaunchArgument(
        'record',
        default_value='true',
        description='Record a rosbag of the cone test session.',
    )

    apply_aim_correction = LaunchConfiguration('apply_aim_correction')
    apply_aim_correction_arg = DeclareLaunchArgument(
        'apply_aim_correction',
        default_value='false',
        description='Apply the 2D affine throw correction. Default OFF because '
                    'the shipped session 1+2 matrix predates several FK + '
                    'frame fixes and tends to make throws worse. Enable only '
                    'once a fresh calibration matrix has been derived.',
    )

    invert_aim_correction = LaunchConfiguration('invert_aim_correction')
    invert_aim_correction_arg = DeclareLaunchArgument(
        'invert_aim_correction',
        default_value='false',
        description='Diagnostic: if true AND apply_aim_correction is true, '
                    'invert the loaded matrix before use. Quick way to test '
                    'whether the matrix is applied in the wrong direction.',
    )

    throw_delay_s = LaunchConfiguration('throw_delay_s')
    throw_delay_s_arg = DeclareLaunchArgument(
        'throw_delay_s',
        default_value='2.5',
        description='Default release delay (s) from service-call to ball '
                    'release. Pitch axis takes up to ~1.6 s for a 60° move; '
                    '2.5 s leaves margin. Override per-call via the service '
                    'request field (non-zero wins).',
    )

    # ── Nodes ────────────────────────────────────────────────────
    can_node = Node(package='jugglebot', executable='can_node')
    mocap_node = Node(package='jugglebot', executable='mocap_node')
    catch_correlation_node = Node(
        package='jugglebot', executable='catch_correlation_node')
    ball_butler_node = Node(
        package='jugglebot', executable='ball_butler_node',
        parameters=[{
            'apply_aim_correction': apply_aim_correction,
            'invert_aim_correction': invert_aim_correction,
            'throw_delay_s': throw_delay_s,
        }],
    )

    # ── Rosbridge (GUI websocket) ────────────────────────────────
    rosbridge_launch_file_path = os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml',
    )
    rosbridge_include = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(rosbridge_launch_file_path),
    )

    # ── Rosbag (optional) ────────────────────────────────────────
    bags_dir = os.path.join(os.path.expanduser('~'), 'Desktop', 'rosbags')
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    bag_dir = os.path.join(bags_dir, f'cone_test_{timestamp}')

    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/throw_announcements',
            '/cone/catch_event',
            '/cone/heartbeat',
            '/cone/timing_result',
            '/bb/heartbeat',
            '/bb/calibration_result',
            '/rigid_body_poses',
            '/qtm_clock_offset_sec',
            '-s', 'mcap', '-o', bag_dir,
        ],
        output='screen',
        condition=IfCondition(record),
    )

    return LaunchDescription([
        record_arg,
        apply_aim_correction_arg,
        invert_aim_correction_arg,
        throw_delay_s_arg,
        rosbridge_include,
        can_node,
        mocap_node,
        catch_correlation_node,
        ball_butler_node,
        rosbag_record,
    ])
