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
        default_value='true',
    )

    # PR 3a: per-launch override for the friction-FF enable flag.  Maps
    # straight to motor_guard's --friction-ff CLI tri-state (yaml/true/
    # false).  Default 'yaml' = use whatever hardware_config.yaml says.
    # Use this for on-platform A/B comparison without YAML edits/rebuilds:
    #   ros2 launch jugglebot jugglebot_launch.py                  # baseline (YAML default)
    #   ros2 launch jugglebot jugglebot_launch.py friction_ff_enable:=true   # FF on
    #   ros2 launch jugglebot jugglebot_launch.py friction_ff_enable:=false  # FF off explicit
    friction_ff_enable = LaunchConfiguration('friction_ff_enable')
    friction_ff_enable_arg = DeclareLaunchArgument(
        'friction_ff_enable',
        default_value='yaml',
        description="Override hardware_config.yaml friction_ff.enabled "
                    "for this launch (yaml | true | false).",
    )

    # Throw aim-correction (the deployed 2D affine). This arg was documented
    # in the accuracy-testing flow but never wired through to the node — Foxy
    # silently ignores unknown launch args, so apply_aim_correction:=true was
    # a no-op and ball_butler_node started "(disabled)".
    apply_aim_correction = LaunchConfiguration('apply_aim_correction')
    apply_aim_correction_arg = DeclareLaunchArgument(
        'apply_aim_correction',
        default_value='true',   # production spatial-accuracy path (validated 2026-06-18)
        description='Load + apply the deployed throw aim-correction affine '
                    'in ball_butler_node (resources/throw_affine_correction.json). '
                    'Default true: it is the production spatial-accuracy path; pair with '
                    'the temporal release-latency offset (BB_OP_THROW_RELEASE_LATENCY_MS).',
    )

    # can_node removed from the launch (phase-10b): its USB-CAN path is dead
    # in the three-bus topology — the bridge Teensy owns all CAN buses, and
    # teensy_bridge_node owns the UDP link. The executable remains available
    # for legacy bench use via `ros2 run jugglebot can_node`.

    # teensy_bridge_node — the can-bridge UDP link (owns the production leg/hand
    # topics + services, /bb/*, /cone/* incl. /bb/axis_estimates). Folded into
    # the main bring-up now that the cutover is complete. SAFETY:
    # enable_setpoint_output defaults FALSE
    # (the bridge sends mpc_active=0 — no leg/setpoint downlink — until an
    # operator flips it AFTER bench validation with motors powered).
    teensy_ip = LaunchConfiguration('teensy_ip')
    teensy_ip_arg = DeclareLaunchArgument(
        'teensy_ip', default_value='192.168.42.2',
        description='Static IP of the can-bridge Teensy (ADR-0007).')
    enable_setpoint_output = LaunchConfiguration('enable_setpoint_output')
    enable_setpoint_output_arg = DeclareLaunchArgument(
        'enable_setpoint_output', default_value='false',
        description='SAFETY: gate for the 40 Hz setpoint downlink + mpc_active '
                    'heartbeat flag. Keep false until bench-validated.')

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

    ball_tracker_node = Node(
        package='jugglebot',
        executable='ball_tracker_node',
    )

    catch_coordinator_node = Node(
        package='jugglebot',
        executable='catch_coordinator_node',
    )

    ball_butler_node = Node(
        package='jugglebot',
        executable='ball_butler_node',
        parameters=[{'apply_aim_correction': apply_aim_correction}],
    )

    mpc_bridge_node = Node(
        package='jugglebot',
        executable='mpc_bridge_node',
    )

    # teensy_bridge_node imports controller.teensy_link from the repo root, which
    # is OUTSIDE the ROS install tree, so prepend it to PYTHONPATH (mirrors
    # teensy_bridge_launch.py). Override host path via JUGGLEBOT_REPO.
    _jugglebot_repo = os.environ.get('JUGGLEBOT_REPO', '/home/jetson/Desktop/Jugglebot')
    _existing_pp = os.environ.get('PYTHONPATH', '')
    _bridge_pythonpath = (f"{_jugglebot_repo}:{_existing_pp}"
                          if _existing_pp else _jugglebot_repo)
    teensy_bridge_node = Node(
        package='jugglebot',
        executable='teensy_bridge_node',
        name='teensy_bridge_node',
        output='screen',
        parameters=[{
            'teensy_ip': teensy_ip,
            'enable_setpoint_output': enable_setpoint_output,
        }],
        additional_env={'PYTHONPATH': _bridge_pythonpath},
    )

    # ── Standalone motor guard process (not a ROS2 node) ────────
    # 500 Hz interpolator + safety monitor between MPC and motor hardware.
    # Installed as a console_scripts entry point alongside other executables.
    pkg_lib_dir = os.path.join(
        get_package_share_directory('jugglebot'), '..', '..', 'lib', 'jugglebot')
    motor_guard = ExecuteProcess(
        cmd=[
            os.path.join(pkg_lib_dir, 'motor_guard'),
            '--rate', '500',
            '--friction-ff', friction_ff_enable,
        ],
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
            '/catch/dynamic_target',
            '/gravity_offset',
            '/leg_torques_diagnostic',
            '/balls',
            '/throw_announcements',
            '/cone/catch_event',
            '/cone/heartbeat',
            '/cone/timing_result',
            '/bb/odrive_diag',
            '/bb/axis_estimates',
            '-s', 'mcap', '-o', bag_dir,
        ],
        output='screen',
        condition=IfCondition(record),
    )

    # ── Assemble launch description ──────────────────────────────
    return LaunchDescription([
        record_arg,
        friction_ff_enable_arg,
        apply_aim_correction_arg,
        teensy_ip_arg,
        enable_setpoint_output_arg,
        # Infrastructure (gui_server.py runs independently in the background)
        rosbridge_include,
        # Core nodes
        orchestrator_node,
        motion_bridge_node,
        mocap_node,
        spacemouse_handler,
        ball_tracker_node,
        catch_coordinator_node,
        ball_butler_node,
        mpc_bridge_node,
        teensy_bridge_node,
        # Standalone processes
        motor_guard,
        # Recording (conditional)
        rosbag_record,
    ])
