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

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from jugglebot_interfaces.msg import RobotState as RobotStateMsg
from jugglebot_interfaces.srv import (
    ActivateOrDeactivate, GetTiltReadingService, ODriveCommandService,
)
from jugglebot_interfaces.action import HomeMotors, Reload
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger
from diagnostic_msgs.msg import DiagnosticStatus

from jugglebot.state_machine import (
    RobotState, Context, build_default_machine, BOOT_TIMEOUT_S,
)
from jugglebot.can import odrive


# ── GUI RELOAD button → jugglebot/reload action goal ─────────────────────────
# Fixed values for the one-button MVP reload dispatch (jugglebot/reload_request):
#   throw_delay_s   3.0  — the requested lead; >= BB's ~2.5 s countdown floor.
#   catch_vel_scale 0.0  — the sentinel for "use the reload coordinator's system
#                          default"; the numeric default lives ONLY in the
#                          coordinator, never here or in the GUI.
RELOAD_THROW_DELAY_S = 3.0
RELOAD_CATCH_VEL_SCALE = 0.0


class OrchestratorNode(Node):
    def __init__(self):
        super().__init__('orchestrator_node')

        # ── State machine ─────────────────────────────────────────
        self.ctx = Context()
        self.sm = build_default_machine(
            log_fn=lambda msg: self.get_logger().info(f'[SM] {msg}'))

        # ── Arming contract (A2, see ARMING_CONTRACT.md) ──────────
        # auto_arm_setpoint_output=true (default): ActiveHandler arms the wire
        # after activation via 'arm_setpoints' (the bridge's stream-then-arm
        # pre-check remains the single safe-to-arm gate). false: the operator
        # arms manually via /set_setpoint_output — the pre-contract probe-first
        # bench flow; the disarmed wire stays loud (A5) instead of silent.
        self.declare_parameter('auto_arm_setpoint_output', True)
        self.ctx.auto_arm = bool(
            self.get_parameter('auto_arm_setpoint_output').value)

        # ── Async operation tracking ──────────────────────────────
        self._pending_future = None          # For service calls
        self._pending_goal_future = None     # Phase 1: goal acceptance
        self._pending_result_future = None   # Phase 2: action result
        # Fire-and-forget futures (A2 disarm) — retained until their
        # done-callbacks fire; never tracked in the operation slots above.
        self._untracked_futures = []

        # ── Service clients ───────────────────────────────────────
        self._encoder_search_client = self.create_client(
            Trigger, 'encoder_search')
        self._activate_client = self.create_client(
            ActivateOrDeactivate, 'activate_or_deactivate')
        self._odrive_cmd_client = self.create_client(
            ODriveCommandService, 'odrive_command')
        self._bb_calibrate_client = self.create_client(
            Trigger, 'bb/calibrate')
        self._tilt_client = self.create_client(
            GetTiltReadingService, 'get_platform_tilt')
        # A2 — the bridge's runtime arming service (SetBool: true=arm,
        # false=disarm). The orchestrator is the production owner of WHEN;
        # the bridge's _arm_setpoint_output owns SAFE-TO (the 5-precondition
        # stream-then-arm check).
        self._setpoint_output_client = self.create_client(
            SetBool, 'set_setpoint_output')

        # ── Action clients ────────────────────────────────────────
        self._home_client = ActionClient(self, HomeMotors, 'home_motors')
        # Reload relay: the browser GUI cannot call the jugglebot/reload ACTION
        # directly — rosbridge on Foxy exposes no action op, and roslib ships only
        # the ROS1 actionlib client (topic protocol a ROS2 action server never
        # advertises). So the GUI hits the jugglebot/reload_request Trigger service
        # below, which relays ONE Reload goal fire-and-forget through this client.
        self._reload_client = ActionClient(self, Reload, 'jugglebot/reload')

        # ── Service servers ───────────────────────────────────────
        self.create_service(
            Trigger, 'jugglebot/reload_request', self._svc_reload_request)

        # ── Subscribers ───────────────────────────────────────────
        self.create_subscription(
            RobotStateMsg, 'robot_state', self._on_robot_state, 10)
        self.create_subscription(
            String, 'orchestrator_command', self._on_command, 10)
        # FIX 2 — Teensy guard awareness. The bridge publishes /link_status (a
        # DiagnosticStatus) at 10 Hz with a 'fault_state' KeyValue. A latched guard
        # suppresses leg output WITHOUT disarming the ODrives, so it never reaches
        # /robot_state.error — the orchestrator was blind to the 2026-07-10
        # MAX_DEVIATION latch and kept accepting mode commands on a frozen robot.
        self.create_subscription(
            DiagnosticStatus, 'link_status', self._on_link_status, 10)

        # ── Publishers ────────────────────────────────────────────
        self._control_mode_pub = self.create_publisher(
            String, 'control_mode_topic', 10)
        self._state_pub = self.create_publisher(
            String, 'orchestrator_state', 10)
        self._gravity_offset_pub = self.create_publisher(
            Float64MultiArray, 'gravity_offset', 10)
        self._level_state_pub = self.create_publisher(
            Float64MultiArray, 'set_level_state', 10)

        self._boot_timeout_logged = False

        # ── Levelling / startup offset tracking ───────────────────
        self._last_sm_state = None
        self._startup_offset_sent = False
        self._pending_tilt_future = None  # tilt service has no success field

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

        self.ctx.firmware_validated = msg.firmware_validated
        self.ctx.encoder_search_complete = msg.encoder_search_complete
        self.ctx.is_homed = msg.is_homed
        self.ctx.errors = list(msg.error)

        # Typed error flags from the CAN node — no string parsing needed.
        self.ctx.fatal_error = msg.has_fatal_odrive_error
        self.ctx.fatal_can_error = msg.has_fatal_can_error
        self.ctx.undervoltage = msg.has_undervoltage

        # Levelling state from Teensy (persisted across reboots).
        # Skip updates while LEVELLING — the state machine is computing
        # new values and we must not overwrite them with stale CAN data.
        if self.sm.state != RobotState.LEVELLING:
            self.ctx.levelling_complete = msg.levelling_complete
            if len(msg.pose_offset_rad) >= 2:
                self.ctx.pose_offset_rad = list(msg.pose_offset_rad[:2])

    def _on_command(self, msg):
        """Queue a user command for the state machine."""
        self.ctx.enqueue_command(msg.data)
        self.get_logger().info(f'Command received: {msg.data}')

    # ═══════════════════════════════════════════════════════════════
    # Reload relay (GUI → jugglebot/reload action)
    # ═══════════════════════════════════════════════════════════════
    #
    # The browser can only reach ROS via topics + services (rosbridge 1.3.1 on
    # Foxy has no action capability). The jugglebot/reload_request Trigger service
    # is the bridge to the jugglebot/reload ACTION: it dispatches ONE goal
    # fire-and-forget and returns a dispatch ACK. It is NOT a re-implementation of
    # the reload — the reload coordinator (reload_coordinator_node) stays the sole
    # authority on preconditions, ordering, and the structured CAUGHT/MISSED
    # outcome (surfaced here only via the logger). Firing an ill-timed reload is
    # therefore safe: the coordinator rejects/aborts it with an honest code.
    #
    # The relay is deliberately fire-and-forget. This node spins single-threaded,
    # so BLOCKING the service handler on the goal-acceptance or result future would
    # deadlock (the same thread must service that future). Mirrors ball_butler_node
    # bb/throw: send_goal_async + a goal-response → result callback chain that only
    # logs the terminal outcome.

    def _svc_reload_request(self, req, res):
        """Relay a GUI reload request to jugglebot/reload (fire-and-forget ACK)."""
        if not self._reload_client.server_is_ready():
            res.success = False
            res.message = 'reload action server unavailable (jugglebot/reload)'
            self.get_logger().warning(
                'Reload request received but jugglebot/reload server is not ready.')
            return res
        goal = Reload.Goal()
        goal.throw_delay_s = RELOAD_THROW_DELAY_S
        goal.catch_vel_scale = RELOAD_CATCH_VEL_SCALE
        send_future = self._reload_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_reload_goal_response)
        res.success = True
        res.message = (
            f'Reload dispatched (throw_delay {RELOAD_THROW_DELAY_S:.1f} s, '
            f'catch_vel_scale default).')
        self.get_logger().info(
            'Reload requested via jugglebot/reload_request — goal dispatched.')
        return res

    def _on_reload_goal_response(self, future):
        """jugglebot/reload goal accepted/rejected — chain to the result (log only)."""
        try:
            goal_handle = future.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f'reload goal send failed: {e}')
            return
        if not goal_handle.accepted:
            self.get_logger().warning(
                'reload goal REJECTED (a reload is already in progress?).')
            return
        goal_handle.get_result_async().add_done_callback(self._on_reload_result)

    def _on_reload_result(self, future):
        """jugglebot/reload terminal outcome — log CAUGHT / MISSED / ABORTED_*."""
        try:
            result = future.result().result
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f'reload result error: {e}')
            return
        if result.success:
            self.get_logger().info(
                f'Reload OK: {result.outcome} '
                f'(miss {result.catch_error_mm:.0f} mm).')
        else:
            self.get_logger().warning(f'Reload not caught: {result.outcome}.')

    @staticmethod
    def _kv_get(values, key, default=''):
        """Read a DiagnosticStatus KeyValue by key (the /link_status decode)."""
        for kv in values:
            if kv.key == key:
                return kv.value
        return default

    def _on_link_status(self, msg):
        """Track the Teensy guard latch from /link_status (FIX 2).

        Sets ``ctx.guard_latched`` from the bridge's ``fault_state`` KeyValue
        (``NONE``/``UNKNOWN`` ⇒ not latched). The state machine uses it to FORCE and
        HOLD FAULT — but only from ACTIVE (see ``_tick``): a stale prior-session
        latch at BOOT must not FAULT the machine before the robot even homes —
        it is CLEARED by BootHandler's disarmed stale-latch pre-flight
        (ARMING_CONTRACT A5; the pre-contract assumption that the ACTIVATE
        arming pre-check would catch it was one half of the 2026-07-15 HOMING
        wedge — the firmware's guard-gated HOME refuses first).
        ``guard_latched`` lives on its own ctx field rather than folded into
        ``ctx.errors`` because ``errors`` is overwritten wholesale every /robot_state.
        """
        fault = self._kv_get(msg.values, 'fault_state', 'NONE')
        self.ctx.guard_latched = fault not in ('NONE', 'UNKNOWN', '')
        # ARMING_CONTRACT A5 — mirror the wire's arming state (observability;
        # handlers never gate transitions on it) and mark that the bridge has
        # reported at least once (BOOT's stale-latch pre-flight waits for this
        # so the default guard_latched=False is never mistaken for "no latch").
        self.ctx.wire_armed = self._kv_get(
            msg.values, 'mpc_active', '0') == '1'
        self.ctx.link_status_seen = True

    # ═══════════════════════════════════════════════════════════════
    # Main tick
    # ═══════════════════════════════════════════════════════════════

    def _tick(self):
        """State machine tick: check ops, detect errors, run tick, dispatch."""
        # 1. Check if a pending async operation completed
        self._check_pending_operations()

        # 2. Force FAULT on errors (from any non-FAULT state).
        #    A latched Teensy guard forces FAULT too (FIX 2), but ONLY from ACTIVE:
        #    the guard's MAX_DEVIATION/MPC_STALE only latch while armed (an ACTIVE
        #    concern), and gating on ACTIVE lets BootHandler's stale-latch
        #    pre-flight CLEAR a prior-session latch at BOOT (ARMING_CONTRACT A5)
        #    instead of the machine wedging on it. FaultHandler then holds
        #    FAULT until the guard clears, routing the existing clear_errors recovery.
        guard_forces_fault = (self.ctx.guard_latched
                              and self.sm.state == RobotState.ACTIVE)
        if ((self.ctx.errors or guard_forces_fault)
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

        # 5. Publish control mode every tick so late-joining subscribers
        #    (e.g. GUI refresh) sync immediately.
        if self.ctx.control_mode is not None:
            self._control_mode_pub.publish(String(data=self.ctx.control_mode))

        # 6. Publish current state every tick for the same reason.
        state_name = self.sm.state.name
        if self.sm.state == RobotState.ACTIVE and self.ctx.active_mode:
            state_str = f'{state_name}:{self.ctx.active_mode.value}'
        else:
            state_str = state_name
        self._state_pub.publish(String(data=state_str))

        # 7. Push persisted gravity offset on first IDLE entry after boot
        current = self.sm.state
        if (current == RobotState.IDLE
                and self._last_sm_state != RobotState.IDLE
                and self.ctx.levelling_complete
                and not self._startup_offset_sent):
            msg = Float64MultiArray(data=list(self.ctx.pose_offset_rad))
            self._gravity_offset_pub.publish(msg)
            self._startup_offset_sent = True
            self.get_logger().info(
                f'Pushed persisted gravity offset: {self.ctx.pose_offset_rad}')
        self._last_sm_state = current

    # ═══════════════════════════════════════════════════════════════
    # Async operation management
    # ═══════════════════════════════════════════════════════════════

    def _check_pending_operations(self):
        """Poll pending async service/action calls for completion."""
        # ── Tilt service call (no success field) ──────────────────
        if self._pending_tilt_future is not None and self._pending_tilt_future.done():
            try:
                result = self._pending_tilt_future.result()
                tilt = list(result.tilt_xy)
                if len(tilt) >= 2 and not (math.isnan(tilt[0]) or math.isnan(tilt[1])):
                    self.ctx.tilt_reading = tilt[:2]
                    self.ctx.operation_result = True
                    self.get_logger().info(
                        f'Tilt reading: [{tilt[0]:.4f}, {tilt[1]:.4f}] rad')
                else:
                    self.get_logger().warning('Tilt reading returned NaN (failed)')
                    self.ctx.operation_result = False
            except Exception as e:
                self.get_logger().error(f'Tilt service exception: {e}')
                self.ctx.operation_result = False
            self.ctx.operation_pending = False
            self._pending_tilt_future = None

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
        """Dispatch operation requests from state handlers.

        Drains the entire request queue so that requests from both
        on_exit() and on_enter()/execute() within the same tick are
        all processed (previously a single-slot field could lose the
        first request if a second was set in the same tick).
        """
        for req in self.ctx.drain_requests():
            self._dispatch_request(req)

    def _dispatch_request(self, req):
        """Handle a single operation request."""
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
                # Ball Butler not available — skip calibration (BB is optional)
                self.get_logger().warning(
                    'Ball Butler not available — skipping calibration. '
                    'BB will be uncalibrated this session.')
                self.ctx.bb_calibration_skipped = True
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

        # ── Arming contract (A2) ──────────────────────────────────
        elif req == 'arm_setpoints':
            arm_req = SetBool.Request()
            arm_req.data = True
            self._start_service_call(self._setpoint_output_client, arm_req)

        elif req == 'disarm_setpoints':
            # Fire-and-forget, OUTSIDE the single-slot _pending_future tracking:
            # the disarm is idempotent, safety-directional, and consumed by no
            # handler — tracking it would ORPHAN an in-flight tracked op (the
            # natural ACTIVE→FAULT path dispatches the multi-second 'deactivate'
            # one tick before FaultHandler requests this disarm, and overwriting
            # that future would open IdleHandler's wait-for-deactivate gate while
            # the platform is still physically descending; review 2026-07-15).
            disarm_req = SetBool.Request()
            disarm_req.data = False
            self._fire_forget_service_call(
                self._setpoint_output_client, disarm_req, 'disarm_setpoints')

        # ── Levelling requests ────────────────────────────────────

        elif req == 'level_activate':
            activate_req = ActivateOrDeactivate.Request()
            activate_req.command = 'activate'
            self._start_service_call(self._activate_client, activate_req)

        elif req == 'level_get_tilt':
            self._start_tilt_service_call()

        elif req == 'level_send_correction':
            msg = Float64MultiArray(
                data=list(self.ctx.pose_offset_rad))
            self._gravity_offset_pub.publish(msg)
            self.get_logger().info(
                f'Gravity offset published: {self.ctx.pose_offset_rad}')
            self.ctx.operation_result = True

        elif req == 'level_persist_state':
            msg = Float64MultiArray(
                data=[1.0] + list(self.ctx.pose_offset_rad))
            self._level_state_pub.publish(msg)
            self.ctx.levelling_complete = True
            self.get_logger().info(
                f'Level state persisted: offset={self.ctx.pose_offset_rad}')
            self.ctx.operation_result = True

        elif req == 'level_mocap_check':
            self.get_logger().warning(
                'Mocap gravity alignment check not yet implemented')
            self.ctx.operation_result = True

        elif req == 'level_deactivate':
            deactivate_req = ActivateOrDeactivate.Request()
            deactivate_req.command = 'deactivate'
            self._start_service_call(self._activate_client, deactivate_req)

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
        self._pending_tilt_future = None
        self.ctx.operation_pending = False
        self.ctx.operation_result = None
        self.ctx.clear_requests()
        self.ctx.clear_commands()

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

    def _fire_forget_service_call(self, client, request, label):
        """Dispatch a call outside the single-slot operation tracking.

        For requests no handler consumes (the A2 disarm): does not touch
        ``operation_pending``/``operation_result`` and never overwrites
        ``_pending_future``. The future is retained until its done-callback
        fires (rclpy futures must outlive the call); failures are logged, not
        routed to the state machine.
        """
        if not client.service_is_ready():
            self.get_logger().warning(
                f'{label}: service not ready (fire-and-forget — not retried)')
            return
        fut = client.call_async(request)
        self._untracked_futures.append(fut)

        def _done(f, label=label):
            try:
                result = f.result()
                if not getattr(result, 'success', True):
                    self.get_logger().warning(
                        f'{label} failed: {getattr(result, "message", "")}')
            except Exception as e:  # noqa: BLE001
                self.get_logger().warning(f'{label} exception: {e}')
            finally:
                try:
                    self._untracked_futures.remove(f)
                except ValueError:
                    pass

        fut.add_done_callback(_done)

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

    def _start_tilt_service_call(self):
        """Start a non-blocking tilt reading service call.

        Uses a separate future because GetTiltReadingService has no
        'success' field — completion is detected by non-zero tilt_xy.
        """
        if not self._tilt_client.service_is_ready():
            self.get_logger().warning('Tilt service not ready, failing request')
            self.ctx.operation_result = False
            return
        self.ctx.operation_pending = True
        self.ctx.operation_result = None
        self._pending_tilt_future = self._tilt_client.call_async(
            GetTiltReadingService.Request())

    # ═══════════════════════════════════════════════════════════════
    # Shutdown
    # ═══════════════════════════════════════════════════════════════

    def on_shutdown(self):
        """Graceful shutdown — deliberately does NOT command any mode change.

        The Ctrl-C profiled stow is owned by ``teensy_bridge_node.on_shutdown``:
        that is the one process that owns BOTH the disarm and the profiled
        DEACTIVATE as in-process methods, and whose UDP link survives its own
        teardown. So the orchestrator has nothing safe to do here.

        Historically this published ``control_mode='ERROR'`` so the (now DELETED)
        ``can_node`` would stow. In the can-bridge architecture that publish is not
        just dead but ACTIVELY HARMFUL: ``ERROR`` is outside ``trajectory_node``'s
        streaming set, so it stops the 40 Hz emitter — and if the bridge is still
        armed (mpc_active=1), the resulting stream silence latches an MPC_STALE
        E-STOP within 250 ms (trajectory_node Sharp Edge #1). That latched guard is
        exactly what makes the bridge's own profiled DEACTIVATE impossible. So the
        safest shutdown action for the orchestrator is to command NOTHING and let
        the bridge disarm-then-stow in the correct order.
        """
        self.get_logger().info(
            'Orchestrator shutdown — profiled stow is owned by teensy_bridge_node')


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
