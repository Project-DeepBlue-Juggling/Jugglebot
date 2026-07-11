from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
)
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

    # can_node DELETED (SocketCAN decommission, 2026-07-06; see
    # logbook/2026-07-06-phase13-socketcan-decommission.md): its USB-CAN
    # path was dead in the three-bus topology — the bridge Teensy owns all CAN
    # buses, and teensy_bridge_node owns the UDP link. The source lives only in
    # git history (pre-deletion revision; see the parity matrix
    # ros_ws/docs/can-node-teensy-parity.md for the full capability mapping).

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

    # BB→Jugglebot reload action (Phase 7): thin orchestrator over the existing
    # catch path (trajectory_node build_catch + catch_coordinator hand-arm) and BB
    # throw. Exposes jugglebot/reload (Reload.action).
    reload_coordinator_node = Node(
        package='jugglebot',
        executable='reload_coordinator_node',
    )

    # mpc_bridge_node is DROPPED from the MVP bring-up: the MPC hot path is replaced
    # by trajectory_node (a simple Jetson-side trajectory generator streaming 40 Hz
    # knots on the same :5557 seam). The MPC return path stays dormant — source is
    # retained (setup.py entry point + jugglebot/mpc_bridge_node.py) so run_mpc.py can
    # be relaunched with trajectory_node stopped (the single-binder :5557 interlock
    # makes a conflict loud). See plans/active/mvp-trajectory-bringup.md § Deferred.
    trajectory_node = Node(
        package='jugglebot',
        executable='trajectory_node',
        name='trajectory_node',
        output='screen',
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
    # Launched as direct Nodes rather than including the stock
    # rosbridge_websocket_launch.xml, because that launch file does not expose
    # the tornado websocket keepalive as an <arg>, so it always ran with
    # websocket_ping_interval=0 — server-side pings OFF. Without pings, a
    # half-open browser↔rosbridge socket (dead TCP/NAT path) is never detected
    # server-side, no 'close' event ever reaches the browser, and the GUI's
    # reconnect-on-close path can't fire — the GUI sits 'disconnected' until a
    # manual refresh (2026-07-11 forensics). Pinging every 10 s keeps idle
    # NAT/TCP state alive AND, on a genuinely dead path, closes the socket after
    # the 30 s timeout so the browser finally gets 'close' and self-heals.
    #
    # This reproduces the XML's non-ssl node (ssl defaults false there, so its
    # ssl variant never ran) plus the rosapi node it also launched — nothing is
    # dropped. All other XML params equal the node's own defaults; only
    # retry_startup_delay was a real override (2.0 → 5.0), preserved here.
    # output='log' captures rosbridge's client connect/disconnect lines in the
    # per-process launch log — they were absent before, which is why the
    # transport drop could not be confirmed from logs.
    rosbridge_websocket_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='log',
        parameters=[{
            'port': 9090,
            'retry_startup_delay': 5.0,
            'websocket_ping_interval': 10,
            'websocket_ping_timeout': 30,
        }],
    )
    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='log',
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
            # Accepted leg setpoints (u0, motor revs) echoed by teensy_bridge_node
            # from the :5557 funnel — the commanded side of the leg tracking
            # story for bag analysis. leg_lengths_topic stays: the MPC
            # (run_mpc/motor_guard) path may still publish it.
            '/leg_setpoint_echo',
            '/hand_telemetry',
            '/mocap_data',
            '/platform_pose_topic',
            '/rigid_body_poses',
            '/orchestrator_state',
            '/control_mode_topic',
            '/orchestrator_command',
            # The fault channel. Without it a latched guard E-STOP (MPC_STALE /
            # MAX_DEVIATION) leaves NO trace in the bag — the 2026-07-09 S2 session's
            # E-STOP was only visible on the live topic, making the bag unable to
            # explain its own gap. Carries fault_state, mpc_active, setpoints_sent/
            # _rejected, link + bus health.
            '/link_status',
            '/trajectory/status',
            '/trajectory/diagnostics',
            '/trajectory/target_feedback',
            # /platform_target_reached removed (SocketCAN decommission): its only publisher
            # (can_node) is deleted; completion is reported via RPC returns
            # (disposition per the can_node<->Teensy parity audit, 2026-07-06).
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
        rosbridge_websocket_node,
        rosapi_node,
        # Core nodes
        orchestrator_node,
        motion_bridge_node,
        mocap_node,
        spacemouse_handler,
        ball_tracker_node,
        catch_coordinator_node,
        ball_butler_node,
        reload_coordinator_node,
        trajectory_node,
        teensy_bridge_node,
        # Standalone processes
        motor_guard,
        # Recording (conditional)
        rosbag_record,
    ])
