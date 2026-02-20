"""Orchestrator Node — manages robot lifecycle via state machine.

Bridges the pure-Python state machine to ROS2:
    - Subscribes to /robot_state for hardware status
    - Subscribes to /orchestrator_command for user commands
    - Calls CAN node services for homing, activation, error clearing
    - Publishes /control_mode_topic for CAN node axis management
    - Publishes /orchestrator_state for monitoring

The state machine tick runs at 10 Hz.  All CAN-node interactions are
async (non-blocking) so the executor stays responsive.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from jugglebot_interfaces.msg import RobotState as RobotStateMsg
from jugglebot_interfaces.srv import ActivateOrDeactivate, ODriveCommandService
from jugglebot_interfaces.action import HomeMotors
from std_msgs.msg import String
from std_srvs.srv import Trigger

from jugglebot.state_machine import (
    RobotState, Context, build_default_machine, BOOT_TIMEOUT_S,
)
from jugglebot.can import odrive


class OrchestratorNode(Node):
    def __init__(self):
        super().__init__('orchestrator_node')

        # ── State machine ─────────────────────────────────────────
        self.ctx = Context()
        self.sm = build_default_machine(
            log_fn=lambda msg: self.get_logger().info(f'[SM] {msg}'))

        # ── Async operation tracking ──────────────────────────────
        self._pending_future = None          # For service calls
        self._pending_goal_future = None     # Phase 1: goal acceptance
        self._pending_result_future = None   # Phase 2: action result

        # ── Service clients ───────────────────────────────────────
        self._encoder_search_client = self.create_client(
            Trigger, 'encoder_search')
        self._activate_client = self.create_client(
            ActivateOrDeactivate, 'activate_or_deactivate')
        self._odrive_cmd_client = self.create_client(
            ODriveCommandService, 'odrive_command')
        self._bb_calibrate_client = self.create_client(
            Trigger, 'bb/calibrate')

        # ── Action client ─────────────────────────────────────────
        self._home_client = ActionClient(self, HomeMotors, 'home_motors')

        # ── Subscribers ───────────────────────────────────────────
        self.create_subscription(
            RobotStateMsg, 'robot_state', self._on_robot_state, 10)
        self.create_subscription(
            String, 'orchestrator_command', self._on_command, 10)

        # ── Publishers ────────────────────────────────────────────
        self._control_mode_pub = self.create_publisher(
            String, 'control_mode_topic', 10)
        self._state_pub = self.create_publisher(
            String, 'orchestrator_state', 10)

        # Track last published values to avoid spamming
        self._last_control_mode = None
        self._last_published_state = None
        self._boot_timeout_logged = False

        # ── Tick timer ────────────────────────────────────────────
        self.create_timer(0.1, self._tick)  # 10 Hz

        self.get_logger().info('Orchestrator node started')

    # ═══════════════════════════════════════════════════════════════
    # Subscriber callbacks
    # ═══════════════════════════════════════════════════════════════

    def _on_robot_state(self, msg):
        """Update context from /robot_state."""
        # Heartbeat check: all Jugglebot axes must report non-default state
        if len(msg.motor_states) >= len(odrive.JUGGLEBOT_AXES):
            self.ctx.all_heartbeats = all(
                msg.motor_states[i].current_state != 0
                for i in odrive.JUGGLEBOT_AXES
            )

        self.ctx.encoder_search_complete = msg.encoder_search_complete
        self.ctx.is_homed = msg.is_homed
        self.ctx.errors = list(msg.error)

        # Typed error flags from the CAN node — no string parsing needed.
        self.ctx.fatal_error = msg.has_fatal_odrive_error
        self.ctx.fatal_can_error = msg.has_fatal_can_error
        self.ctx.undervoltage = msg.has_undervoltage

    def _on_command(self, msg):
        """Queue a user command for the state machine."""
        if self.ctx.pending_command is not None:
            self.get_logger().warning(
                f'Command "{self.ctx.pending_command}" dropped '
                f'(replaced by "{msg.data}")')
        self.ctx.pending_command = msg.data
        self.get_logger().info(f'Command received: {msg.data}')

    # ═══════════════════════════════════════════════════════════════
    # Main tick
    # ═══════════════════════════════════════════════════════════════

    def _tick(self):
        """State machine tick: check ops, detect errors, run tick, dispatch."""
        # 1. Check if a pending async operation completed
        self._check_pending_operations()

        # 2. Force FAULT on errors (from any non-FAULT state)
        if (self.ctx.errors
                and self.sm.state != RobotState.FAULT):
            self.sm.force_transition(RobotState.FAULT, self.ctx)
            self._cancel_pending_operations()

        # 3. Run state machine
        self.sm.tick(self.ctx)

        # 3a. Log boot timeout (once per occurrence)
        if self.ctx.boot_timed_out and not self._boot_timeout_logged:
            self.get_logger().error(
                f'Boot timeout: no ODrive heartbeats received within '
                f'{BOOT_TIMEOUT_S:.0f}s. Check power and CAN connections. '
                f'Send "clear_errors" to retry.')
            self._boot_timeout_logged = True
        if not self.ctx.boot_timed_out:
            self._boot_timeout_logged = False

        # 4. Process requests set by handlers
        self._process_requests()

        # 5. Publish control mode if changed
        if (self.ctx.control_mode is not None
                and self.ctx.control_mode != self._last_control_mode):
            self._control_mode_pub.publish(String(data=self.ctx.control_mode))
            self._last_control_mode = self.ctx.control_mode

        # 6. Publish current state for monitoring
        state_name = self.sm.state.name
        if state_name != self._last_published_state:
            self._state_pub.publish(String(data=state_name))
            self._last_published_state = state_name

    # ═══════════════════════════════════════════════════════════════
    # Async operation management
    # ═══════════════════════════════════════════════════════════════

    def _check_pending_operations(self):
        """Poll pending async service/action calls for completion."""
        # ── Service call ──────────────────────────────────────────
        if self._pending_future is not None and self._pending_future.done():
            try:
                result = self._pending_future.result()
                self.ctx.operation_result = result.success
                if not result.success:
                    detail = getattr(result, 'message', 'unknown')
                    self.get_logger().warning(f'Operation failed: {detail}')
            except Exception as e:
                self.get_logger().error(f'Service call exception: {e}')
                self.ctx.operation_result = False
            self.ctx.operation_pending = False
            self._pending_future = None

        # ── Action: waiting for goal acceptance ───────────────────
        if (self._pending_goal_future is not None
                and self._pending_goal_future.done()):
            try:
                goal_handle = self._pending_goal_future.result()
                if not goal_handle.accepted:
                    self.get_logger().error('Home action goal rejected')
                    self.ctx.operation_result = False
                    self.ctx.operation_pending = False
                    self._pending_goal_future = None
                else:
                    # Goal accepted — now wait for result
                    self._pending_result_future = (
                        goal_handle.get_result_async())
                    self._pending_goal_future = None
            except Exception as e:
                self.get_logger().error(f'Action goal exception: {e}')
                self.ctx.operation_result = False
                self.ctx.operation_pending = False
                self._pending_goal_future = None

        # ── Action: waiting for result ────────────────────────────
        if (self._pending_result_future is not None
                and self._pending_result_future.done()):
            try:
                wrapper = self._pending_result_future.result()
                self.ctx.operation_result = wrapper.result.success
                if not wrapper.result.success:
                    self.get_logger().warning('Home action failed')
            except Exception as e:
                self.get_logger().error(f'Action result exception: {e}')
                self.ctx.operation_result = False
            self.ctx.operation_pending = False
            self._pending_result_future = None

    def _process_requests(self):
        """Dispatch operation requests from state handlers."""
        req = self.ctx.request
        if req is None:
            return
        self.ctx.request = None

        if req == 'encoder_search':
            self._start_service_call(
                self._encoder_search_client, Trigger.Request())

        elif req == 'home':
            self._start_home_action()

        elif req == 'bb_calibrate':
            if self._bb_calibrate_client.service_is_ready():
                self._start_service_call(
                    self._bb_calibrate_client, Trigger.Request())
            else:
                # Ball Butler not available — skip calibration
                self.get_logger().info(
                    'Ball Butler not available, skipping calibration')
                self.ctx.operation_result = True

        elif req == 'activate':
            activate_req = ActivateOrDeactivate.Request()
            activate_req.command = 'activate'
            self._start_service_call(self._activate_client, activate_req)

        elif req == 'deactivate':
            deactivate_req = ActivateOrDeactivate.Request()
            deactivate_req.command = 'deactivate'
            self._start_service_call(self._activate_client, deactivate_req)

        elif req == 'clear_errors':
            cmd_req = ODriveCommandService.Request()
            cmd_req.command = 'clear_errors'
            self._start_service_call(self._odrive_cmd_client, cmd_req)

        else:
            self.get_logger().warning(f'Unknown request: {req}')

    # ═══════════════════════════════════════════════════════════════
    # Async call helpers
    # ═══════════════════════════════════════════════════════════════

    def _cancel_pending_operations(self):
        """Discard in-flight async operations (e.g., when entering FAULT).

        Server-side processing continues but we stop tracking results.
        This prevents stale operation_result from affecting the next state.
        Also clears any pending request from the interrupted state's on_exit
        (e.g., ActiveHandler sets ctx.request='deactivate' in on_exit, which
        is correct for ACTIVE→IDLE but wrong for ACTIVE→FAULT).
        """
        self._pending_future = None
        self._pending_goal_future = None
        self._pending_result_future = None
        self.ctx.operation_pending = False
        self.ctx.operation_result = None
        self.ctx.request = None

    def _start_service_call(self, client, request):
        """Start a non-blocking service call."""
        if not client.service_is_ready():
            self.get_logger().warning(
                f'Service {client.srv_name} not ready, failing request')
            self.ctx.operation_result = False
            return

        self.ctx.operation_pending = True
        self.ctx.operation_result = None
        self._pending_future = client.call_async(request)

    def _start_home_action(self):
        """Start the home_motors action (two-phase: goal acceptance then result)."""
        if not self._home_client.server_is_ready():
            self.get_logger().warning(
                'Home action server not ready, failing request')
            self.ctx.operation_result = False
            return

        self.ctx.operation_pending = True
        self.ctx.operation_result = None
        self._pending_goal_future = self._home_client.send_goal_async(
            HomeMotors.Goal())

    # ═══════════════════════════════════════════════════════════════
    # Shutdown
    # ═══════════════════════════════════════════════════════════════

    def on_shutdown(self):
        """Graceful shutdown: publish ERROR mode to stow the platform.

        Uses fire-and-forget rather than spin_until_future_complete to avoid
        the shutdown race condition where the node may be partially destroyed.
        The CAN node handles ERROR mode by stowing the platform and idling
        all axes, which is the same end result as explicit deactivation.
        """
        if self.sm.state == RobotState.ACTIVE:
            self.get_logger().info('Shutting down from ACTIVE — publishing ERROR mode')
            try:
                self._control_mode_pub.publish(String(data='ERROR'))
            except Exception as e:
                self.get_logger().error(f'Shutdown error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
