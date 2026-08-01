from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import os
from datetime import datetime


def generate_launch_description():
    # ── Launch arguments ─────────────────────────────────────────
    record = LaunchConfiguration('record')
    record_arg = DeclareLaunchArgument(
        'record',
        default_value='true',
    )

    # DORMANT since 2026-08-01 (MPC dormancy, plans/active/refactor-2026-07.md
    # Phase 3).  This was PR 3a's per-launch override for motor_guard's
    # --friction-ff CLI tri-state; motor_guard no longer launches, so nothing
    # consumes it.  Kept DECLARED rather than deleted — same treatment as
    # enable_setpoint_output below — so `ros2 launch ... --show-args` still
    # names it and says it is inert.  Deleting the declaration would not make a
    # stale `friction_ff_enable:=true` an error either (Foxy silently ignores
    # unknown launch args), it would just make it invisible.  Friction FF itself
    # is unaffected: it lives in motor_guard, which is parked with the MPC.
    friction_ff_enable_arg = DeclareLaunchArgument(
        'friction_ff_enable',
        default_value='yaml',
        description="DORMANT + INERT since the MPC dormancy (2026-08-01): its "
                    "only consumer was the motor_guard process, which this "
                    "launch no longer starts. Restored by the MPC revival.",
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
    # the main bring-up now that the cutover is complete. The bridge ALWAYS
    # starts DISARMED (mpc_active=0); arming is runtime-only via
    # /set_setpoint_output — automatic on ACTIVE entry (orchestrator auto-arm)
    # or manual with auto_arm:=false. See
    # ros_ws/src/jugglebot/jugglebot/ARMING_CONTRACT.md.
    teensy_ip = LaunchConfiguration('teensy_ip')
    teensy_ip_arg = DeclareLaunchArgument(
        'teensy_ip', default_value='192.168.42.2',
        description='Static IP of the can-bridge Teensy (ADR-0007).')
    enable_setpoint_output = LaunchConfiguration('enable_setpoint_output')
    enable_setpoint_output_arg = DeclareLaunchArgument(
        'enable_setpoint_output', default_value='false',
        description='DEPRECATED + INERT since the arming contract (2026-07-15): '
                    'boot-arming had zero preconditions and self-E-STOPd '
                    '(MPC_STALE). The bridge logs an ERROR if true and stays '
                    'disarmed. Arming is runtime-only via /set_setpoint_output.')
    auto_arm = LaunchConfiguration('auto_arm')
    auto_arm_arg = DeclareLaunchArgument(
        'auto_arm', default_value='true',
        description='ARMING_CONTRACT A2: true (default) — the orchestrator arms '
                    'the setpoint wire on ACTIVE entry (after the bridge\'s '
                    'stream-then-arm pre-check passes) and FAULTs on persistent '
                    'refusal. false — the operator arms manually via '
                    '/set_setpoint_output (probe-first bench flow).')

    # ── Core ROS2 nodes ──────────────────────────────────────────
    orchestrator_node = Node(
        package='jugglebot',
        executable='orchestrator_node',
        parameters=[{
            'auto_arm_setpoint_output': auto_arm,
        }],
    )

    # motion_bridge_node is DORMANT (2026-08-01, plans/active/refactor-2026-07.md
    # Phase 3 — "remove operationally, park the code"). It was the MPC leg path's
    # ROS side: motor_guard's :5556 interpolated stream -> leg_lengths_topic ->
    # can_node. can_node was deleted in the 2026-07-06 SocketCAN decommission and
    # the bridge does not subscribe to leg_lengths_topic, so the topic has had no
    # consumer since (ros_ws/docs/can-node-teensy-parity.md:410, :548). The node
    # source, its setup.py entry point and tests/ros/test_motion_bridge_node.py
    # all stay: revival is re-adding this Node entry.
    #
    # ONE OPERATOR-VISIBLE CONSEQUENCE, deliberate and accepted: this node was
    # also the sole publisher of `motion/diagnostics` and `motion/tracking_error`.
    # The GUI subscribes to the former (gui/js/main.js:299), and its 3 s timeout
    # (gui/js/panels.js:697) now fires permanently, so the Motion panel badge sits
    # at DISABLED with a blank trajectory label (panels.js:758). That is honest —
    # the MPC motion chain IS disabled — and it degrades to a badge, never to a
    # false ERR. The two topics stay in the rosbag record list below and record
    # empty, for the same reason leg_lengths_topic does. The 2026-07-11 GUI work
    # (logbook/2026-07-11-gui-leg-setpoint-echo-poscmd.md) moved the *leg* half
    # off this node and explicitly flagged `motion/diagnostics` as the one
    # remaining GUI dependency; this is that dependency going dark. A GUI-side
    # rewording of the panel for the MVP topology is owed and out of scope here.

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

    # Catch-timing correlation: matches each cone/catch_event against the nearest
    # predicted landing in throw_announcements and publishes cone/timing_result
    # (the per-catch predicted-vs-actual delta the GUI Catching Cone panel shows).
    # This node's ONLY launcher was catching_cone_test.launch.py, deleted in the
    # Phase 13 SocketCAN decommission (7c7f61b); its consumers stayed wired
    # (rosbag /cone/timing_result, the GUI panel), so cone/timing_result had zero
    # publishers and a piezo hit surfaced nothing even though teensy_bridge_node
    # keeps publishing cone/catch_event + cone/heartbeat. Restored here.  Matches
    # the deleted launch's declaration exactly (no params/remaps).
    catch_correlation_node = Node(
        package='jugglebot',
        executable='catch_correlation_node',
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

    # ── motor_guard: DORMANT (2026-08-01) ────────────────────────
    # plans/active/refactor-2026-07.md Phase 3 — "remove operationally, park the
    # code". The 500 Hz interpolator + safety monitor sat between the MPC and the
    # motors on the OLD topology. In the MVP topology it drives nothing: the leg
    # path is trajectory_node -> :5557 -> teensy_bridge_node -> the can-bridge
    # Teensy, which does its own 500 Hz interpolation, and the guard's :5556
    # output "simply goes unconsumed" (teensy_bridge_node.py, the
    # _MpcCommandSetpointSource docstring). The GUI migrated off it too
    # (ros_ws/gui/js/main.js:417).
    #
    # SAFETY AUTHORITY on the leg path is the Teensy-side MAX_DEVIATION guard,
    # not this process — it has been that way since the Teensy-side cutover;
    # launching motor_guard was not adding a safety layer, only a dead process.
    # The console_scripts entry point + the module + its tests all stay; revival
    # is re-adding this ExecuteProcess. Note the revival is BOTH entries, not one:
    # HardwarePlant.enable() blocks on motor-feedback telemetry from the guard's
    # :5556 (controller/hardware_plant.py ~:1033), and the guard is fed by
    # motion_bridge_node, so run_mpc.py cannot come back without both.

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
            # story for bag analysis. leg_lengths_topic stays in the record list
            # even though its only publisher (motion_bridge_node) is dormant
            # since 2026-08-01: recording a silent topic costs nothing, and it
            # keeps bag schemas comparable across the dormancy boundary and after
            # the MPC revival. Same applies to /motion/tracking_error and
            # /motion/diagnostics below — motion_bridge_node was their sole
            # publisher too, so all three record empty until the MPC revival.
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
        auto_arm_arg,
        # Infrastructure (gui_server.py runs independently in the background)
        rosbridge_websocket_node,
        rosapi_node,
        # Core nodes
        orchestrator_node,
        mocap_node,
        spacemouse_handler,
        ball_tracker_node,
        catch_coordinator_node,
        catch_correlation_node,
        ball_butler_node,
        reload_coordinator_node,
        trajectory_node,
        teensy_bridge_node,
        # Recording (conditional)
        rosbag_record,
    ])
