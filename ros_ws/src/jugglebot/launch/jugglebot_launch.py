from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import glob
import os
import subprocess
import sys
from datetime import datetime


# ── Config freshness observability (plans/active/refactor-2026-07.md Phase 5) ──
#
# Production is deliberately BUILD-FROZEN: every node imports the *installed*
# generated modules, so a YAML edit changes nothing until generate_config +
# colcon build have both run. Staler-than-expected fails safe; fresher-than-
# expected on an actuator path is the dangerous direction. The failure mode this
# leaves is not danger but CONFUSION — the operator edits a gain, relaunches,
# and the robot behaves exactly as before with no signal at all. So the fix is
# observability, not automatic freshness: name the staleness at bring-up.
#
# Two independent links in the chain, reported separately because the fixes
# differ:
#   A) YAML -> repo artifacts   (`generate_config.py --check`)  -> regenerate
#   B) repo artifacts -> INSTALLED copies (byte compare)        -> colcon build
#
# HARD CONTRACT: this NEVER blocks, delays past a bounded timeout, or fails the
# launch. Every failure path — checker missing, crash, timeout, unreadable
# install tree — degrades to one informational line. A bring-up must never be
# lost to its own diagnostics.
#
# The timeout is 30x the measured cost (0.33 s on the Jetson) and deliberately
# NOT generous: `ros2 launch` prints nothing at all until
# generate_launch_description() returns, so every second spent here is a frozen
# terminal with no explanation. 10 s is a hiccup; 30 s reads as a hang.
_DRIFT_CHECK_TIMEOUT_S = 10.0

# Only the Python artifacts are checked for install drift: they are the ones the
# ROS2 nodes import at runtime. The firmware headers are consumed by a compiler,
# not by this launch, and the codegen check (A) already covers them.
_INSTALLED_GENERATED_MODULES = ('hardware_config.py', 'protocol_config.py')

# Single severity per call site by construction (all LogInfo): Foxy's rcutils
# logger caches severity per source line and RAISES on a flip — that is what
# killed teensy_bridge_node on the first STANDBY arm (971d12c, 2026-07-31).
# Foxy's launch.actions offers LogInfo only, so "loud" is done with a banner,
# not with a severity.
_BANNER = '!' * 78


def _repo_root():
    """Locate the source repo this build came from.

    Order: explicit override -> walk up from THIS file -> the canonical path.

    The walk matters because the file executing here is the colcon-INSTALLED
    launch script under `<repo>/ros_ws/install/jugglebot/share/...`, so walking
    up until `config/generate_config.py` appears lands on the repo that produced
    this install. Parallel sessions on this project isolate with `git worktree`
    (memory: feedback_parallel_session_worktrees); with a hard-coded root, a
    launch built from a worktree would check the MAIN tree's YAML against the
    worktree's install and report drift that is purely an artifact of the
    diagnostic. Falling back to the canonical path keeps the old behaviour when
    the install tree lives outside the repo.
    """
    override = os.environ.get('JUGGLEBOT_REPO')
    if override:
        return override
    try:
        here = os.path.dirname(os.path.abspath(__file__))
        while True:
            if os.path.isfile(os.path.join(here, 'config', 'generate_config.py')):
                return here
            parent = os.path.dirname(here)
            if parent == here:
                break
            here = parent
    except Exception:  # noqa: BLE001 — diagnostics never raise
        pass
    return '/home/jetson/Desktop/Jugglebot'


def _installed_jugglebot_dir():
    """Locate the colcon-installed `jugglebot` python package, or None.

    Walks AMENT_PREFIX_PATH rather than importing the package: importing would
    execute node modules (rclpy, numpy-quaternion, ...) inside the launch
    process for a diagnostic.
    """
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        if not prefix:
            continue
        for cand in glob.glob(os.path.join(
                prefix, 'lib', 'python3.*', 'site-packages', 'jugglebot')):
            if os.path.isdir(cand):
                return cand
    return None


def _codegen_drift(repo):
    """Link A: source YAML vs the in-repo generated + delivered artifacts.

    Returns ``(drift_lines, note_lines)``; drift_lines empty == fresh. A check
    that could not RUN is a note, never drift — reporting "CONFIG DRIFT" for a
    missing checker would train the operator to ignore the banner.

    Shells out to `config/generate_config.py --check`, which renders every
    artifact in memory and diffs it — a guaranteed READ-ONLY operation
    (pinned by tests/firmware/test_config_drift.py). `sys.executable` here is
    the launch's interpreter, i.e. the SYSTEM python3.8, not the project venv;
    `--check` is deliberately dependency-light (stdlib + PyYAML) so it runs
    there.

    The checker's `SKIPPED:` / `NOT CHECKED:` lines are collected EVEN ON
    EXIT 0: they mean part of the surface was never compared, and a green line
    that silently covers half the artifacts is the exact trap this whole feature
    exists to close. `EXTERNAL DRIFT:` / `EXTERNAL SKIPPED:` are deliberately
    NOT collected — ../BallButler is a separate checkout whose firmware headers
    no node in this launch reads, and raising the banner for it would spend the
    banner's only asset, its credibility.
    """
    script = os.path.join(repo, 'config', 'generate_config.py')
    if not os.path.isfile(script):
        return [], ['codegen check skipped — %s not found' % script]
    proc = subprocess.run(
        [sys.executable, script, '--check'],
        cwd=repo, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        timeout=_DRIFT_CHECK_TIMEOUT_S)
    detail = proc.stderr.decode('utf-8', 'replace')
    stripped = [ln.strip() for ln in detail.splitlines()]
    notes = [ln for ln in stripped
             if ln.startswith('SKIPPED:') or ln.startswith('NOT CHECKED:')]
    if proc.returncode == 0:
        return [], notes
    lines = [ln for ln in stripped if ln.startswith('DRIFT:')]
    if lines:
        return lines, notes
    # Non-zero without DRIFT lines == the checker itself failed (bad YAML, bad
    # argument). Report it, but do not claim to know the config is stale.
    return [], notes + ['codegen check inconclusive — --check exited %d: %s'
                        % (proc.returncode, detail.strip().replace('\n', ' | '))]


def _install_drift(repo):
    """Link B: in-repo generated artifacts vs the colcon-INSTALLED copies.

    This is the one the codegen check cannot see, and it is the actual trap:
    `generate_config.py` was run, the repo is self-consistent, and the launch
    still runs last week's numbers because `colcon build` was not.

    A missing file on either side is a NOTE, never silence: an absent installed
    module is a packaging regression (setup.py stopped shipping it) and an
    absent source module means the wrong repo root was resolved — both would
    otherwise degrade into a green "installed copies agree".
    """
    installed = _installed_jugglebot_dir()
    if installed is None:
        return [], ['install check skipped — no jugglebot package on '
                    'AMENT_PREFIX_PATH']
    src_dir = os.path.join(repo, 'ros_ws', 'src', 'jugglebot', 'jugglebot')
    drift, notes = [], []
    for name in _INSTALLED_GENERATED_MODULES:
        src, dst = os.path.join(src_dir, name), os.path.join(installed, name)
        if not os.path.isfile(src):
            notes.append('install check skipped for %s — no repo copy at %s'
                         % (name, src))
            continue
        if not os.path.isfile(dst):
            notes.append('install check skipped for %s — not present in the '
                         'installed package (%s); colcon build?' % (name, dst))
            continue
        with open(src, 'rb') as f_src, open(dst, 'rb') as f_dst:
            if f_src.read() != f_dst.read():
                drift.append('STALE INSTALL: %s differs from the repo copy' % dst)
    return drift, notes


def _config_freshness_actions():
    """Return the launch actions that report config freshness. Never raises."""
    try:
        repo = _repo_root()
        drift_a, notes_a = _codegen_drift(repo)
        drift_b, notes_b = _install_drift(repo)
        drift, notes = drift_a + drift_b, notes_a + notes_b
        if not drift and not notes:
            # Scope named precisely: the check covers what generate_config.py
            # renders. config/generated/udp_protocol.{h,py} come from a DIFFERENT
            # generator and are NOT covered — and a stale udp_protocol header is
            # a named past incident on this branch (24608bb), so "generated
            # artifacts agree" would be an over-claim exactly where it hurts.
            return [LogInfo(msg='[config] freshness check OK — hardware/protocol/'
                                'geometry codegen and the installed copies agree.')]
        if not drift:
            # Something could not be checked. Say so rather than reporting OK:
            # a green line for a check that never ran is worse than no line.
            return [LogInfo(msg='[config] freshness check PARTIAL — no drift in '
                                'what was checked; NOT checked: '
                                + '; '.join(notes))]
        msg = [_BANNER,
               '!!  CONFIG DRIFT WARNING — this launch is NOT running the '
               'source config.']
        msg += ['!!  ' + ln for ln in drift + notes]
        msg += ['!!',
                '!!  FIX:  python config/generate_config.py',
                '!!        cd ros_ws && colcon build --packages-select jugglebot',
                '!!        then RELAUNCH — the launch runs the INSTALLED copy.',
                '!!',
                '!!  Launch CONTINUES. Build-frozen config is the safe direction; '
                'this is a warning, never a gate.',
                _BANNER]
        return [LogInfo(msg='\n'.join(msg))]
    except Exception as exc:  # noqa: BLE001 — a diagnostic must never kill bring-up
        return [LogInfo(msg='[config] freshness check unavailable (%s: %s) — '
                            'continuing.' % (type(exc).__name__, exc))]


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
    # The freshness banner goes FIRST so it is not buried under node startup
    # chatter (the whole point is that the operator sees it).
    return LaunchDescription([
        *_config_freshness_actions(),
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
