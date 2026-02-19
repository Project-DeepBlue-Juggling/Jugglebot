"""CAN Interface Node — ROS2 wrapper for CAN bus communication.

Owns the CAN bus, ODrive protocol, Ball Butler protocol, and motor state tracker.
Provides services, publishers, and subscribers that the rest of the system uses.
"""

import math
import struct
import time

import can as python_can  # python-can (not jugglebot.can subpackage)
import quaternion
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from jugglebot_interfaces.msg import (
    BallButlerHeartbeat as BallButlerHeartbeatMsg,
    CanTrafficReportMessage,
    HandTelemetryMessage,
    LegsTargetReachedMessage,
    RobotState,
    SetMotorVelCurrLimitsMessage,
    SetTrapTrajLimitsMessage,
)
from jugglebot_interfaces.srv import (
    ActivateOrDeactivate,
    GetTiltReadingService,
    ODriveCommandService,
    SendBallButlerCommand,
    SetFloat,
    SetHandTrajCmd,
    SetString,
)
from jugglebot_interfaces.action import HomeMotors
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Quaternion

from jugglebot.can.bus import CANBus
from jugglebot.can import odrive
from jugglebot.can import ball_butler
from jugglebot.can.motor_state import MotorStateTracker
import jugglebot.protocol_config as proto
import jugglebot.hardware_config as hw

# ── CAN watchdog constants ─────────────────────────────────────
_HEARTBEAT_TIMEOUT_S = 2.0       # Trigger watchdog after 2s without any axis heartbeat
_HEARTBEAT_GATE_TIMEOUT_S = 5.0  # Max wait for heartbeats before ODrive configuration


class CanInterfaceNode(Node):
    def __init__(self):
        super().__init__('can_interface_node')

        self.shutdown_flag = False

        # ── Core components ────────────────────────────────────
        self.bus = CANBus(logger=self.get_logger())
        self.motors = MotorStateTracker()
        self.last_bb_heartbeat = ball_butler.BallButlerHeartbeat()

        # Operational limits (mutable, can be changed via topics)
        self.leg_vel_limit = odrive.DEFAULT_VEL_CURR['leg_vel']
        self.leg_curr_limit = odrive.DEFAULT_VEL_CURR['leg_curr']
        self.hand_vel_limit = odrive.DEFAULT_VEL_CURR['hand_vel']
        self.hand_curr_limit = odrive.DEFAULT_VEL_CURR['hand_curr']
        self.trap_traj = dict(odrive.DEFAULT_TRAP_TRAJ)
        self.hand_gains = dict(odrive.DEFAULT_HAND_GAINS)

        # Teensy state (persisted on Teensy across reboots)
        self.last_known_state = {
            'updated': False,
            'encoder_search_complete': False,
            'is_homed': False,
            'levelling_complete': False,
            'pose_offset_rad': [0.0, 0.0],
            'pose_offset_quat': Quaternion(w=1.0, x=0.0, y=0.0, z=0.0),
            'error': [],
        }

        # Hand input pos tracking (for telemetry)
        self._last_hand_cmd = {'pos': 0.0, 'vel': 0.0, 'tor': 0.0}

        # Platform target tracking
        self.legs_target_position = [None] * odrive.NUM_LEGS
        self.legs_target_reached = [False] * odrive.NUM_LEGS
        self.stowed_due_to_error = False
        self._encoder_data_received = [False] * odrive.NUM_LEGS  # Track whether real encoder data has arrived

        # Tilt sensor and quaternion accumulation
        self._tilt_reading = None
        self._last_tilt_offset = quaternion.quaternion(1, 0, 0, 0)

        # CAN IDs for Teensy messages
        self._tilt_reading_id = proto.CAN_ID_PLATFORM_TILT_READING
        self._traffic_report_id = proto.CAN_ID_PLATFORM_TRAFFIC_REPORT
        self._state_update_id = proto.CAN_ID_PLATFORM_STATE_UPDATE
        self._hand_traj_cmd_id = proto.CAN_ID_PLATFORM_TRAJ_CMD
        self._hand_input_pos_id = (proto.NODE_ID_JUGGLEBOT_HAND << 5) | proto.ODRIVE_COMMANDS['set_input_pos']

        # Initialize the bus
        self.bus.setup()

        # ── Services ───────────────────────────────────────────
        self.create_service(Trigger, 'encoder_search', self._svc_encoder_search)
        self.create_service(Trigger, 'end_session', self._svc_end_session)
        self.create_service(ODriveCommandService, 'odrive_command', self._svc_odrive_command)
        self.create_service(GetTiltReadingService, 'get_platform_tilt', self._svc_get_tilt)
        self.create_service(ActivateOrDeactivate, 'activate_or_deactivate', self._svc_activate_or_deactivate)
        self.create_service(SetString, 'set_hand_state', self._svc_set_hand_state)
        self.create_service(SetHandTrajCmd, 'set_hand_traj_cmd', self._svc_set_hand_traj)
        self.create_service(SetFloat, 'smooth_move_hand', self._svc_smooth_move_hand)
        self.create_service(SendBallButlerCommand, 'bb/send_throw_command', self._svc_bb_throw)
        self.create_service(Trigger, 'bb/reload', self._svc_bb_reload)
        self.create_service(Trigger, 'bb/reset', self._svc_bb_reset)
        self.create_service(Trigger, 'bb/calibrate', self._svc_bb_calibrate)

        # ── Action servers ─────────────────────────────────────
        self._home_action = ActionServer(self, HomeMotors, 'home_motors', self._action_home)

        # ── Subscribers ────────────────────────────────────────
        self.create_subscription(Float64MultiArray, 'leg_lengths_topic', self._sub_leg_lengths, 10)
        self.create_subscription(SetMotorVelCurrLimitsMessage, 'set_motor_vel_curr_limits', self._sub_vel_curr_limits, 10)
        self.create_subscription(SetTrapTrajLimitsMessage, 'set_leg_trap_traj_limits', self._sub_trap_traj_limits, 10)
        self.create_subscription(String, 'control_mode_topic', self._sub_control_mode, 10)

        # ── Publishers ─────────────────────────────────────────
        self.robot_state_pub = self.create_publisher(RobotState, 'robot_state', 10)
        self.can_traffic_pub = self.create_publisher(CanTrafficReportMessage, 'can_traffic', 10)
        self.hand_telemetry_pub = self.create_publisher(HandTelemetryMessage, 'hand_telemetry', 10)
        self.target_reached_pub = self.create_publisher(LegsTargetReachedMessage, 'platform_target_reached', 10)
        self.bb_heartbeat_pub = self.create_publisher(BallButlerHeartbeatMsg, 'bb/heartbeat', 10)

        # ── Timers ─────────────────────────────────────────────
        can_poll_period = 0.001  # 1 kHz CAN poll
        state_pub_period = 0.01  # 100 Hz robot state
        ts_period = 0.01  # 100 Hz time-sync broadcast
        hand_telemetry_period = 0.002  # 500 Hz hand telemetry
        bb_heartbeat_period = 0.1  # 10 Hz Ball Butler heartbeat
        target_reached_period = 0.1  # 10 Hz target reached check

        self.create_timer(can_poll_period, self._poll_can_bus)           # 1 kHz CAN poll
        self.create_timer(state_pub_period, self._publish_robot_state)     # 100 Hz state
        self.create_timer(ts_period, self.bus.broadcast_time)  # Time sync
        self.create_timer(hand_telemetry_period, self._publish_hand_telemetry) # 500 Hz hand
        self.create_timer(bb_heartbeat_period, self._publish_bb_heartbeat)     # 10 Hz BB
        self.create_timer(target_reached_period, self._check_target_reached)     # 10 Hz target
        self.create_timer(1.0, self._watchdog_check)                              # 1 Hz heartbeat watchdog
        self._watchdog_restore_failed = False

        # Callbacks that _run_to_completion keeps alive when the executor is
        # blocked.  Each entry is (callback, period_seconds).  During normal
        # operation the ROS2 timers above handle scheduling; during blocking
        # generator operations _pump() takes over at the same frequencies.
        self._keepalive_schedule = [
            (self.bus.broadcast_time,       ts_period),
            (self._publish_robot_state,     state_pub_period),
            (self._publish_hand_telemetry,  hand_telemetry_period),
            (self._publish_bb_heartbeat,    bb_heartbeat_period),
            (self._check_target_reached,    target_reached_period),
        ]

    # ═══════════════════════════════════════════════════════════
    # CAN message polling and dispatch
    # ═══════════════════════════════════════════════════════════

    def _poll_can_bus(self):
        """Poll the CAN bus for new messages."""
        self.bus.fetch_all(self._handle_message)

        # Clear stored errors if the error condition has been resolved
        if self.last_known_state['error'] and not (
            self.motors.fatal_error or self.motors.fatal_can_error
        ):
            self.get_logger().info(f"Error cleared: {self.last_known_state['error']}")
            self.last_known_state['error'] = []

    def _handle_message(self, msg):
        """Dispatch a single CAN message to the appropriate handler."""
        aid = msg.arbitration_id

        if aid == self._traffic_report_id:
            self._handle_traffic_report(msg)
        elif aid == self._tilt_reading_id:
            if msg.data != b'\x01':
                self._handle_tilt_reading(msg)
        elif aid == self._state_update_id:
            self._decode_teensy_state(msg)
        elif aid == self._hand_input_pos_id:
            self._handle_hand_input_pos(msg)
        elif aid == ball_butler.HEARTBEAT_ID:
            try:
                self.last_bb_heartbeat = ball_butler.BallButlerHeartbeat.from_can_frame(msg.data)
            except ValueError as e:
                self.get_logger().warning(f"Bad BB heartbeat frame: {e}", throttle_duration_sec=5.0)
        else:
            # ODrive message: extract axis_id and command_id
            axis_id = aid >> 5
            cmd_id = aid & 0x1F

            # Filter irrelevant BB motor messages
            if axis_id in odrive.BB_MOTOR_IDS and cmd_id not in odrive.BB_RELEVANT_CMD_IDS:
                return

            handler = self._odrive_handlers.get(cmd_id)
            if handler:
                try:
                    handler(self, axis_id, msg.data)
                except ValueError as e:
                    self.get_logger().warning(
                        f"Bad CAN frame for cmd 0x{cmd_id:02X} axis {axis_id}: {e}",
                        throttle_duration_sec=5.0)
            else:
                self.get_logger().warning(
                    f"No handler for cmd 0x{cmd_id:02X} on axis {axis_id} (arb_id=0x{aid:03X})"
                )

    # ── ODrive message handlers ────────────────────────────────

    def _handle_heartbeat(self, axis_id, data):
        state, proc_result, traj_done = odrive.decode_heartbeat(data)
        self.motors.update(axis_id, current_state=state,
                           procedure_result=proc_result, trajectory_done=traj_done)
        self.motors.record_heartbeat(axis_id)

    def _handle_error(self, axis_id, data):
        active_errors, disarm_reason = odrive.decode_error(data)
        self.motors.update(axis_id, active_errors=active_errors, disarm_reason=disarm_reason)

        # Refresh the snapshot so we see the update we just applied
        states = self.motors.get_states()[:len(odrive.JUGGLEBOT_AXES)]
        no_active = all(s.active_errors == 0 for s in states)
        no_disarm = all(s.disarm_reason == 0 for s in states)
        any_disarmed = any(s.disarm_reason != 0 for s in states)
        any_closed_loop = any(s.current_state == odrive.AXIS_STATES['CLOSED_LOOP'] for s in states)

        if no_active and no_disarm:
            self.motors.fatal_error = False
            self.motors.undervoltage_error = False
            self.last_known_state['error'] = []
            return

        # Disarmed but no axes in closed-loop → try clearing.
        # No delay needed — if errors persist, the next CAN error message
        # will re-trigger this handler and increment the attempt counter.
        if any_disarmed and not any_closed_loop:
            if self.motors.soft_reset_attempts < self.motors.max_soft_reset_attempts:
                self._clear_errors()
                self.motors.soft_reset_attempts += 1
            else:
                self.get_logger().error("Exceeded soft-error reset attempts", throttle_duration_sec=1.0)
                self.motors.fatal_error = True
                self._update_disarm_error_state()

        # Disarmed AND closed-loop → fatal
        if any_disarmed and any_closed_loop:
            self.motors.fatal_error = True
            self._update_disarm_error_state()
            self.get_logger().error("Axis disarmed while in CLOSED_LOOP!", throttle_duration_sec=1.0)

        if not no_active:
            self.motors.fatal_error = True
        if active_errors & odrive.ERR_DC_BUS_UNDER_VOLTAGE:
            self.motors.undervoltage_error = True
        if disarm_reason & odrive.ERR_DC_BUS_UNDER_VOLTAGE and no_active:
            self.motors.undervoltage_error = False
            self.motors.fatal_error = False
            self._clear_errors()
            self.get_logger().info("Undervoltage disarm cleared (no active errors)")

        # Throttled per-axis per-error logging
        log_times = self.motors.last_error_log_times(axis_id)
        now = time.time()
        for code, name in odrive.ERROR_CODES.items():
            if active_errors & code:
                if name not in self.last_known_state['error']:
                    self.last_known_state['error'].append(name)
                if now - log_times.get(code, 0) > self.motors.error_log_throttle_sec:
                    self.get_logger().error(f"Active error on axis {axis_id}: {name}")
                    log_times[code] = now
            elif disarm_reason & code:
                if now - log_times.get(code, 0) > self.motors.error_log_throttle_sec:
                    self.get_logger().error(f"Disarm reason on axis {axis_id}: {name}")
                    log_times[code] = now

    def _handle_encoder(self, axis_id, data):
        pos, vel = odrive.decode_encoder_estimate(data)
        if axis_id in odrive.LEG_AXES:  # Legs: invert (ODrive -ve = extension, we want +ve = extension)
            pos, vel = -pos, -vel
            self._encoder_data_received[axis_id] = True
        self.motors.update(axis_id, pos_estimate=pos, vel_estimate=vel)

    def _handle_iq(self, axis_id, data):
        sp, meas = odrive.decode_iq(data)
        self.motors.update(axis_id, iq_setpoint=sp, iq_measured=meas)

    def _handle_temps(self, axis_id, data):
        fet, motor = odrive.decode_temps(data)
        self.motors.update(axis_id, fet_temp=fet, motor_temp=motor)

    def _handle_bus_vc(self, axis_id, data):
        v, c = odrive.decode_bus_voltage_current(data)
        self.motors.update(axis_id, bus_voltage=v, bus_current=c)

    def _handle_sdo_response(self, axis_id, data):
        endpoint_id, value = odrive.decode_sdo_response(data)
        if endpoint_id == odrive.ENDPOINT_IDS['commutation_mapper.pos_abs']:
            self.motors.encoder_search_feedback[axis_id] = not math.isnan(value)

    # Handler dispatch table (maps command_id → method).
    # These are unbound functions — called as handler(self, axis_id, data).
    _odrive_handlers = {
        proto.ODRIVE_COMMANDS['heartbeat_message']:       _handle_heartbeat,
        proto.ODRIVE_COMMANDS['get_error']:               _handle_error,
        proto.ODRIVE_COMMANDS['get_encoder_estimate']:    _handle_encoder,
        proto.ODRIVE_COMMANDS['get_iq']:                  _handle_iq,
        proto.ODRIVE_COMMANDS['get_temps']:               _handle_temps,
        proto.ODRIVE_COMMANDS['get_bus_voltage_current']: _handle_bus_vc,
        proto.ODRIVE_COMMANDS['TxSdo']:                   _handle_sdo_response,
    }

    # ── Teensy message handlers ────────────────────────────────

    def _handle_traffic_report(self, msg):
        if len(msg.data) < 4:
            return
        count = msg.data[0] + (msg.data[1] << 8)
        interval = msg.data[2] + (msg.data[3] << 8)
        m = CanTrafficReportMessage()
        m.received_count = count
        m.report_interval = interval
        self.can_traffic_pub.publish(m)

    def _handle_tilt_reading(self, msg):
        try:
            self._tilt_reading = struct.unpack('<ff', msg.data)
        except struct.error as e:
            self.get_logger().warning(f"Tilt unpack error: {e}")

    def _decode_teensy_state(self, msg):
        try:
            flags, tx, ty = struct.unpack('<Bhh3x', msg.data)
        except struct.error:
            return
        self.last_known_state['updated'] = True
        self.last_known_state['is_homed'] = bool(flags & 0x01)
        self.last_known_state['levelling_complete'] = bool(flags & 0x02)
        self.last_known_state['pose_offset_rad'] = (tx / 1000.0, ty / 1000.0)
        self.last_known_state['pose_offset_quat'] = self._tilt_to_quat(tx / 1000.0, ty / 1000.0)

    def _handle_hand_input_pos(self, msg):
        try:
            pos, vel_ff, tor_ff = struct.unpack('<fhh', msg.data)
            self._last_hand_cmd = {
                'pos': pos,
                'vel': vel_ff / proto.INPUT_SCALE_VEL,
                'tor': tor_ff / proto.INPUT_SCALE_TOR,
            }
        except struct.error:
            pass

    # ═══════════════════════════════════════════════════════════
    # ODrive compound operations
    # ═══════════════════════════════════════════════════════════

    def _clear_errors(self):
        """Clear errors on all Jugglebot axes."""
        for axis_id in odrive.JUGGLEBOT_AXES:
            self.bus.send(odrive.encode_clear_errors(axis_id))
        self.motors.clear_error_flags()
        self.last_known_state['error'] = []
        for state in self.motors._states:
            state.disarm_reason = 0

    def _setup_odrives_steps(self, requested_state='IDLE'):
        """Generator: configure all ODrives after confirming they're responsive.

        Waits for heartbeats from all Jugglebot axes before sending config
        commands, ensuring ODrives are alive and listening.
        """
        deadline = time.time() + _HEARTBEAT_GATE_TIMEOUT_S
        while not self.motors.all_jugglebot_heartbeats_received():
            if time.time() > deadline:
                raise RuntimeError(
                    "ODrives not responding — cannot configure unresponsive hardware")
            yield 0.1

        self._set_vel_curr_limits()
        for axis_id in odrive.LEG_AXES:
            self.bus.send(odrive.encode_set_controller_mode(axis_id, 'POSITION', 'TRAP_TRAJ'))
            time.sleep(0.005)
        self._set_trap_traj_limits()
        self.bus.send(odrive.encode_set_controller_mode(odrive.HAND_AXIS, 'POSITION', 'PASSTHROUGH'))
        for axis_id in odrive.JUGGLEBOT_AXES:
            self.bus.send(odrive.encode_set_state(axis_id, requested_state))
            time.sleep(0.005)
        self._set_hand_gains()

    def _set_vel_curr_limits(self):
        """Apply current velocity/current limits to all Jugglebot axes."""
        for axis_id in odrive.LEG_AXES:
            self.bus.send(odrive.encode_set_vel_curr_limits(axis_id, self.leg_vel_limit, self.leg_curr_limit))
            time.sleep(0.005)
        self.bus.send(odrive.encode_set_vel_curr_limits(odrive.HAND_AXIS, self.hand_vel_limit, self.hand_curr_limit))

    def _set_trap_traj_limits(self):
        """Apply trapezoidal trajectory limits to all legs."""
        for axis_id in odrive.LEG_AXES:
            self.bus.send(odrive.encode_set_traj_vel_limit(axis_id, self.trap_traj['vel_limit']))
            self.bus.send(odrive.encode_set_traj_acc_limits(axis_id, self.trap_traj['acc_limit'], self.trap_traj['dec_limit']))
            time.sleep(0.005)

    def _set_hand_gains(self):
        """Apply hand motor PID gains."""
        self.bus.send(odrive.encode_set_pos_gain(odrive.HAND_AXIS, self.hand_gains['pos_gain']))
        self.bus.send(odrive.encode_set_vel_gains(odrive.HAND_AXIS, self.hand_gains['vel_gain'], self.hand_gains['vel_int_gain']))

    def _send_position_target(self, axis_id, setpoint, vel_ff=0, torque_ff=0):
        """Send a position command to an axis, with clipping and leg inversion."""
        setpoint = odrive.clip_position(axis_id, setpoint, self.get_logger())
        if axis_id in odrive.LEG_AXES:
            setpoint = -setpoint  # Legs: ODrive -ve = extension
        self.bus.send(odrive.encode_set_input_pos(axis_id, setpoint, vel_ff, torque_ff))

    # ═══════════════════════════════════════════════════════════
    # Service callbacks
    # ═══════════════════════════════════════════════════════════

    def _svc_encoder_search(self, req, res):
        """Run encoder index search on all leg axes."""
        try:
            success = self._run_encoder_search()
            res.success = success
            res.message = 'Encoder search complete!' if success else 'Encoder search failed.'
        except Exception as e:
            self.get_logger().error(f"Encoder search error: {e}")
            res.success = False
            res.message = str(e)
        return res

    def _svc_end_session(self, req, res):
        self.get_logger().info("End session requested")
        res.success = True
        res.message = "Shutting down."
        self.shutdown_flag = True
        return res

    def _svc_odrive_command(self, req, res):
        try:
            if req.command == 'clear_errors':
                self._clear_errors()
                res.success = True
                res.message = 'Errors cleared.'
            elif req.command == 'reboot_odrives':
                self._run_to_completion(self._reboot_odrives_steps())
                res.success = True
                res.message = 'ODrives rebooted.'
            else:
                res.success = False
                res.message = f'Unknown command: {req.command}'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _svc_get_tilt(self, req, res):
        try:
            tx, ty, quat = self._get_tilt_reading()
            res.tilt_xy = [tx, ty] if tx is not None else [0.0, 0.0]
            res.tilt_quat = quat if quat else Quaternion(w=1.0)
        except Exception as e:
            self.get_logger().error(f"Tilt reading error: {e}")
            res.tilt_xy = [0.0, 0.0]
            res.tilt_quat = Quaternion(w=1.0)
        return res

    def _svc_activate_or_deactivate(self, req, res):
        try:
            if req.command == 'activate':
                result = self._gently_move_to_setpoint(hw.JB_OP_ACTIVATE_POSITION_REV, deactivating=False)
            elif req.command == 'deactivate':
                result = self._gently_move_to_setpoint(0.0, deactivating=True)
            else:
                res.success = False
                res.message = f"Invalid command: {req.command}"
                return res
            res.success = result
            res.message = f"Platform {'activated' if req.command == 'activate' else 'deactivated'}." if result else "Failed."
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _svc_set_hand_state(self, req, res):
        try:
            self.bus.send(odrive.encode_set_state(odrive.HAND_AXIS, req.data))
            res.success = True
            res.message = f"Hand state set to {req.data}"
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _svc_set_hand_traj(self, req, res):
        try:
            self._send_hand_traj_cmd(req.event_delay, req.event_vel, req.traj_type)
            res.success = True
            res.message = "Hand trajectory set."
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _svc_smooth_move_hand(self, req, res):
        try:
            self._smooth_move_hand(req.data)
            res.success = True
            res.message = f"Smooth-move hand to {req.data:.3f} rev"
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _svc_bb_throw(self, req, res):
        try:
            msg = ball_butler.encode_throw_command(req.yaw_angle_rad, req.pitch_angle_rad,
                                                   req.throw_speed, req.throw_time)
            self.bus.send(msg)
            res.success = True
            res.message = "Throw command sent."
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _svc_bb_reload(self, req, res):
        return self._bb_state_cmd(ball_butler.RELOAD_CMD_ID, "reload", res)

    def _svc_bb_reset(self, req, res):
        return self._bb_state_cmd(ball_butler.RESET_CMD_ID, "reset", res)

    def _svc_bb_calibrate(self, req, res):
        return self._bb_state_cmd(ball_butler.CALIBRATE_CMD_ID, "calibrate", res)

    def _bb_state_cmd(self, cmd_id, name, res):
        try:
            self.bus.send(ball_butler.encode_state_command(cmd_id))
            res.success = True
            res.message = f"BB {name} command sent."
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    # ═══════════════════════════════════════════════════════════
    # Action callbacks
    # ═══════════════════════════════════════════════════════════

    def _action_home(self, goal_handle):
        """Home all Jugglebot motors (legs + hand)."""
        try:
            success = self._home_robot()
            self._update_teensy_state({'is_homed': success})
            if success:
                self.get_logger().info("Homing complete.")
                goal_handle.succeed()
            else:
                goal_handle.abort()
            result = HomeMotors.Result()
            result.success = success
            return result
        except Exception as e:
            self.get_logger().error(f"Homing error: {e}")
            goal_handle.abort()
            result = HomeMotors.Result()
            result.success = False
            return result

    # ═══════════════════════════════════════════════════════════
    # Subscriber callbacks
    # ═══════════════════════════════════════════════════════════

    def _sub_leg_lengths(self, msg):
        """Handle platform movement commands (6 leg positions)."""
        positions = msg.data
        self.legs_target_position = list(positions)
        for axis_id, setpoint in enumerate(positions):
            self._send_position_target(axis_id, setpoint)

    def _sub_vel_curr_limits(self, msg):
        if msg.legs_vel_limit > 0:
            self.leg_vel_limit = msg.legs_vel_limit
        if msg.legs_curr_limit > 0:
            self.leg_curr_limit = msg.legs_curr_limit
        if msg.hand_vel_limit > 0:
            self.hand_vel_limit = msg.hand_vel_limit
        if msg.hand_curr_limit > 0:
            self.hand_curr_limit = msg.hand_curr_limit
        self._set_vel_curr_limits()

    def _sub_trap_traj_limits(self, msg):
        if msg.trap_vel_limit > 0:
            self.trap_traj['vel_limit'] = msg.trap_vel_limit
        if msg.trap_acc_limit > 0:
            self.trap_traj['acc_limit'] = msg.trap_acc_limit
        if msg.trap_dec_limit > 0:
            self.trap_traj['dec_limit'] = msg.trap_dec_limit
        self._set_trap_traj_limits()

    def _sub_control_mode(self, msg):
        """Handle control mode changes (critical Phase 1 port from can_interface_node.py:280-325)."""
        try:
            states = self.motors.last_states
            _CL = odrive.AXIS_STATES['CLOSED_LOOP']
            legs_closed = all(s.current_state == _CL for s in states[:odrive.NUM_LEGS])
            hand_closed = states[odrive.HAND_AXIS].current_state == _CL

            if msg.data == 'ERROR':
                self.get_logger().error("Error state detected. Stowing platform.")
                self._gently_move_to_setpoint(0.0, deactivating=True)
                self.stowed_due_to_error = True

            # Modes where legs need CLOSED_LOOP but hand should be IDLE
            elif msg.data in ('STANDBY_ACTIVE', 'SPACEMOUSE', 'LEVEL_PLATFORM_NODE',
                              'CATCH_THROWN_BALL_NODE', 'SHELL', 'CALIBRATE_PLATFORM'):
                self.get_logger().info(f'Control mode: {msg.data}')
                if not legs_closed:
                    for axis_id in odrive.LEG_AXES:
                        self.bus.send(odrive.encode_set_state(axis_id, 'CLOSED_LOOP'))
                        time.sleep(0.005)
                if hand_closed:
                    self.bus.send(odrive.encode_set_state(odrive.HAND_AXIS, 'IDLE'))

            # Modes where ALL axes need CLOSED_LOOP
            elif msg.data in ('CATCH_DROPPED_BALL_NODE', 'HOOP_SINKER', 'CATCH_FROM_BALL_BUTLER'):
                if not legs_closed or not hand_closed:
                    for axis_id in odrive.JUGGLEBOT_AXES:
                        self.bus.send(odrive.encode_set_state(axis_id, 'CLOSED_LOOP'))
                        time.sleep(0.005)

            elif msg.data == '':
                pass  # No mode selected

            else:
                self.get_logger().warning(f"Unknown control mode: {msg.data}. Stowing.")
                self._gently_move_to_setpoint(0.0, deactivating=True)

        except Exception as e:
            self.get_logger().error(f"Control mode error: {e}")

    # ═══════════════════════════════════════════════════════════
    # Publishers (timer callbacks)
    # ═══════════════════════════════════════════════════════════

    def _publish_robot_state(self):
        try:
            states = self.motors.get_states()
            msg = RobotState()
            msg.timestamp = self.get_clock().now().to_msg()
            msg.motor_states = states

            if self.motors.undervoltage_error:
                msg.error.append("Undervoltage detected. Was the E-stop hit?")
            elif self.motors.fatal_error:
                msg.error.append(f"Fatal ODrive issue: {self.last_known_state['error']}")
            elif self.motors.fatal_can_error:
                msg.error.append("Fatal CAN bus issue.")

            s = self.last_known_state
            msg.encoder_search_complete = s['encoder_search_complete']
            msg.is_homed = s['is_homed']
            msg.levelling_complete = s['levelling_complete']
            msg.pose_offset_rad = list(s['pose_offset_rad'])
            msg.pose_offset_quat = s['pose_offset_quat']
            self.robot_state_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Robot state publish error: {e}")

    def _publish_hand_telemetry(self):
        try:
            cmd = self._last_hand_cmd
            hand_state = self.motors.last_states[odrive.HAND_AXIS]
            msg = HandTelemetryMessage(
                timestamp=self.get_clock().now().to_msg(),
                pos_cmd=cmd['pos'], vel_ff_cmd=cmd['vel'], tor_ff_cmd=cmd['tor'],
                pos_meas=hand_state.pos_estimate, vel_meas=hand_state.vel_estimate,
                iq_meas=hand_state.iq_measured,
            )
            self.hand_telemetry_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Hand telemetry error: {e}")

    def _publish_bb_heartbeat(self):
        try:
            hb = self.last_bb_heartbeat
            msg = BallButlerHeartbeatMsg()
            msg.ball_in_hand = hb.ball_in_hand
            msg.state = hb.state
            msg.state_data = hb.state_data
            msg.yaw_deg = hb.yaw_deg
            msg.pitch_deg = hb.pitch_deg
            msg.hand_pos_mm = hb.hand_mm
            self.bb_heartbeat_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"BB heartbeat publish error: {e}")

    def _check_target_reached(self):
        try:
            if None in self.legs_target_position:
                return
            if not all(self._encoder_data_received):
                return  # No real encoder data yet — don't report false positives
            states = self.motors.last_states
            for i in odrive.LEG_AXES:
                at_target = (abs(states[i].pos_estimate - self.legs_target_position[i]) < hw.JB_OP_TARGET_REACHED_POS_TOL_REV
                             and abs(states[i].vel_estimate) < hw.JB_OP_TARGET_REACHED_VEL_TOL_RPS)
                self.legs_target_reached[i] = at_target
            msg = LegsTargetReachedMessage()
            msg.leg0_has_arrived = self.legs_target_reached[0]
            msg.leg1_has_arrived = self.legs_target_reached[1]
            msg.leg2_has_arrived = self.legs_target_reached[2]
            msg.leg3_has_arrived = self.legs_target_reached[3]
            msg.leg4_has_arrived = self.legs_target_reached[4]
            msg.leg5_has_arrived = self.legs_target_reached[5]
            self.target_reached_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Target reached check error: {e}")

    # ═══════════════════════════════════════════════════════════
    # Generator driver
    # ═══════════════════════════════════════════════════════════

    def _run_to_completion(self, gen):
        """Drive a generator to completion, processing CAN messages between yields.

        Generator protocol:
            yield None      — process CAN and continue immediately (tight polling)
            yield <seconds> — process CAN while waiting, then continue
            return <value>  — operation complete (received via StopIteration)

        Generators can compose via ``yield from sub_gen()``.

        While the executor is blocked inside this method, ROS2 timer callbacks
        can't fire.  We replicate their work here so that CAN polling, time-sync,
        state publishing, and telemetry all continue at their normal frequencies.

        On unhandled exceptions, all axes are sent to IDLE as a safety measure.
        """
        next_fire = [0.0] * len(self._keepalive_schedule)

        def _pump():
            self._poll_can_bus()
            now = time.time()
            for i, (cb, period) in enumerate(self._keepalive_schedule):
                if now >= next_fire[i]:
                    cb()
                    next_fire[i] = now + period
            # CAN disconnect detection during generator operations
            if (self.motors.first_heartbeat_received
                    and not self.motors.fatal_can_error
                    and self.motors.any_heartbeat_stale(_HEARTBEAT_TIMEOUT_S)):
                self.motors.fatal_can_error = True
                self.get_logger().error(
                    "CAN bus disconnection detected during operation")

        try:
            while True:
                delay = next(gen)
                _pump()
                if isinstance(delay, (int, float)) and delay > 0:
                    deadline = time.time() + delay
                    while time.time() < deadline:
                        time.sleep(0.001)
                        _pump()
        except StopIteration as e:
            return e.value
        except Exception:
            self._emergency_idle()
            raise

    def _emergency_idle(self):
        """Send all Jugglebot axes to IDLE — last-resort safety on unhandled errors."""
        for axis_id in odrive.JUGGLEBOT_AXES:
            try:
                self.bus.send(odrive.encode_set_state(axis_id, 'IDLE'))
            except Exception:
                pass  # Best-effort during emergency
        self.motors.fatal_error = True
        self.get_logger().error(
            "Emergency idle: all axes sent to IDLE due to unhandled error")

    def _watchdog_check(self):
        """Periodic heartbeat watchdog — detects CAN bus disconnection.

        Activates only after the first heartbeat is received.  On timeout,
        attempts bus restoration (3 retries internally).  Sets fatal_can_error
        on failure.

        Note: attempt_restore() blocks the executor for up to ~15 seconds
        (3 retries x 5s delay).  This is acceptable since we're already in a
        degraded state (CAN disconnection), and no useful work can proceed
        until the bus is restored.
        """
        if not self.motors.first_heartbeat_received:
            return
        if not self.motors.any_heartbeat_stale(_HEARTBEAT_TIMEOUT_S):
            self._watchdog_restore_failed = False
            return
        if self._watchdog_restore_failed:
            return  # Already attempted restore and failed; don't retry

        self.get_logger().error(
            "Heartbeat watchdog: CAN bus disconnection detected")
        self.motors.fatal_can_error = True
        self.motors.reset_heartbeats()

        def poll_cb():
            self.bus.fetch_all(self._handle_message)
            return self.motors.all_jugglebot_heartbeats_received()

        if self.bus.attempt_restore(poll_callback=poll_cb):
            self.motors.fatal_can_error = False
            self._watchdog_restore_failed = False
            self.get_logger().info("CAN bus restored by watchdog")
        else:
            self._watchdog_restore_failed = True
            self.get_logger().error(
                "CAN bus restoration failed — fatal CAN error")

    # ═══════════════════════════════════════════════════════════
    # Async operations (generator-based state machines)
    # ═══════════════════════════════════════════════════════════

    def _run_encoder_search(self, attempt=0) -> bool:
        """Run encoder index search on all leg axes."""
        return self._run_to_completion(self._encoder_search_steps(attempt))

    def _encoder_search_steps(self, attempt=0):
        """Generator: encoder index search on all leg axes."""
        for axis_id in odrive.LEG_AXES:
            self.bus.send(odrive.encode_set_state(axis_id, 'ENCODER_INDEX_SEARCH'))

        # Wait for all legs to enter ENCODER_INDEX_SEARCH state
        deadline = time.time() + hw.JB_OP_ENCODER_SEARCH_TIMEOUT_S
        _EIS = odrive.AXIS_STATES['ENCODER_INDEX_SEARCH']
        in_search = [False] * odrive.NUM_LEGS
        while not all(in_search):
            if time.time() > deadline:
                self.get_logger().error("Encoder search timed out (enter phase)")
                return False
            for i in odrive.LEG_AXES:
                if self.motors.last_states[i].current_state == _EIS:
                    in_search[i] = True
            yield  # Process CAN

        # Wait for procedure_result == 0 (SUCCESS) on all legs
        deadline = time.time() + hw.JB_OP_ENCODER_SEARCH_TIMEOUT_S
        while not all(s.procedure_result == 0 for s in self.motors.last_states[:odrive.NUM_LEGS]):
            if self.motors.fatal_error or self.motors.fatal_can_error:
                self.get_logger().error("Fatal error during encoder search")
                return False
            if time.time() > deadline:
                if attempt > 0:
                    self.get_logger().error("Encoder search failed after retry")
                    return False
                self.get_logger().error("Encoder search failed, clearing errors and retrying...")
                self._clear_errors()
                return (yield from self._encoder_search_steps(attempt + 1))
            yield 0.1  # Poll every 100ms

        self.get_logger().info("Encoder search complete!")
        self.last_known_state['encoder_search_complete'] = True
        return True

    def _home_robot(self) -> bool:
        """Home all Jugglebot motors by driving to end-stops."""
        return self._run_to_completion(self._home_robot_steps())

    def _home_robot_steps(self):
        """Generator: home all Jugglebot motors."""
        yield from self._setup_odrives_steps()

        for axis_id in odrive.JUGGLEBOT_AXES:
            if axis_id in odrive.LEG_AXES:
                ok = yield from self._home_motor_steps(
                    axis_id, speed=hw.HOMING_LEG_SPEED_RPS,
                    limit=hw.HOMING_LEG_CURRENT_LIMIT_A, headroom=hw.HOMING_LEG_CURRENT_HEADROOM_A)
                if not ok:
                    return False
                self.bus.send(odrive.encode_set_absolute_position(axis_id, hw.HOMING_LEG_ABS_POS_REV))
                self.get_logger().info(f"Motor {axis_id} homed!")
            else:  # Hand
                self._set_hand_gains()
                ok = yield from self._home_motor_steps(
                    axis_id, speed=hw.HOMING_HAND_SPEED_RPS,
                    limit=hw.HOMING_HAND_CURRENT_LIMIT_A, headroom=hw.HOMING_HAND_CURRENT_HEADROOM_A)
                if not ok:
                    return False
                self.bus.send(odrive.encode_set_absolute_position(axis_id, hw.HOMING_HAND_ABS_POS_REV))
                self.get_logger().info("Hand homed!")

        yield from self._setup_odrives_steps()
        return True

    def _home_motor_steps(self, axis_id, speed, limit, headroom):
        """Generator: run a motor at velocity until current exceeds the limit (homing)."""
        self.bus.send(odrive.encode_set_state(axis_id, 'CLOSED_LOOP'))
        self.bus.send(odrive.encode_set_controller_mode(axis_id, 'VELOCITY', 'VEL_RAMP'))
        self.bus.send(odrive.encode_set_vel_curr_limits(axis_id, abs(speed * 2), limit + headroom))
        yield 0.01  # Let settings take effect
        self.bus.send(odrive.encode_set_input_vel(axis_id, speed))

        avg = 0.0
        while True:
            if self.motors.fatal_error or self.motors.fatal_can_error:
                self.get_logger().fatal("Fatal error during homing!")
                self.bus.send(odrive.encode_set_state(axis_id, 'IDLE'))
                return False
            current = self.motors.get_field(axis_id, 'iq_measured')
            if current is not None:
                avg = avg * hw.HOMING_EMA_WEIGHT + current * (1.0 - hw.HOMING_EMA_WEIGHT)
                if abs(avg) >= limit:
                    self.bus.send(odrive.encode_set_state(axis_id, 'IDLE'))
                    return True
            yield  # Process CAN (tight poll — current measurement arrives at ~100Hz)

    def _gently_move_to_setpoint(self, setpoint, deactivating=True) -> bool:
        """Slowly move all legs to a setpoint, then activate or deactivate."""
        try:
            return self._run_to_completion(self._gentle_move_steps(setpoint, deactivating))
        except Exception as e:
            self.get_logger().error(f"Gentle move error: {e}")
            return False

    def _gentle_move_steps(self, setpoint, deactivating=True):
        """Generator: slowly move all legs to a setpoint."""
        states = self.motors.last_states
        _IDLE = odrive.AXIS_STATES['IDLE']
        _CL = odrive.AXIS_STATES['CLOSED_LOOP']
        legs_idle = all(s.current_state == _IDLE for s in states[:odrive.NUM_LEGS])

        if deactivating and legs_idle:
            if all(s.pos_estimate < 0.1 for s in states[:odrive.NUM_LEGS]):
                self.get_logger().info("Legs already stowed.")
                return True

        if not all(s.current_state == _CL for s in states[:odrive.NUM_LEGS]):
            for axis_id in odrive.LEG_AXES:
                self.bus.send(odrive.encode_set_state(axis_id, 'CLOSED_LOOP'))
                time.sleep(0.005)
        yield 0.1  # Let state transitions settle

        # Lower speed for gentle movement
        for axis_id in odrive.LEG_AXES:
            self.bus.send(odrive.encode_set_vel_curr_limits(axis_id, hw.JB_OP_GENTLE_MOVE_VEL_LIMIT_RPS, self.leg_curr_limit))
            time.sleep(0.005)
        yield 0.1  # Let limits take effect

        for axis_id in odrive.LEG_AXES:
            self._send_position_target(axis_id, setpoint)
            time.sleep(0.001)

        yield 0.5  # Initial movement time

        # Wait for trajectories to complete (legs only)
        deadline = time.time() + hw.JB_OP_GENTLE_MOVE_TIMEOUT_S
        while not all(s.trajectory_done for s in self.motors.last_states[:odrive.NUM_LEGS]):
            if self.motors.fatal_error or self.motors.fatal_can_error:
                self.get_logger().error("Fatal error during gentle move")
                return False
            if time.time() > deadline:
                self.get_logger().error("Gentle move timed out waiting for trajectory_done")
                return False
            yield 0.01  # Poll every 10ms

        yield 1.0  # Settle time

        if deactivating:
            for axis_id in odrive.JUGGLEBOT_AXES:
                self.bus.send(odrive.encode_set_state(axis_id, 'IDLE'))
                time.sleep(0.005)
        else:
            yield from self._setup_odrives_steps(requested_state='CLOSED_LOOP')

        return True

    def _reboot_odrives_steps(self):
        """Generator: reboot all ODrives and wait for restart."""
        for axis_id in odrive.JUGGLEBOT_AXES:
            self.bus.send(odrive.encode_reboot(axis_id))
        self.motors.reset_heartbeat_tracking()  # Suppress watchdog during reboot
        self.motors.reset_heartbeats()           # Require fresh heartbeats for setup
        yield 10.0  # Wait for ODrives to reboot
        self.last_known_state['is_homed'] = False
        self.last_known_state['levelling_complete'] = False
        self._update_teensy_state({
            'is_homed': False,
            'levelling_complete': False,
            'pose_offset_rad': [0.0, 0.0],
        })
        return True

    # ═══════════════════════════════════════════════════════════
    # Teensy communication
    # ═══════════════════════════════════════════════════════════

    def _update_teensy_state(self, state_dict):
        """Send state flags and tilt offsets to the Teensy for persistence."""
        flags = 0
        flags |= int(state_dict.get('is_homed', self.last_known_state['is_homed'])) << 0
        flags |= int(state_dict.get('levelling_complete', self.last_known_state['levelling_complete'])) << 1

        tx, ty = state_dict.get('pose_offset_rad', self.last_known_state['pose_offset_rad'])
        data = struct.pack('<Bhh3x', flags, int(tx * 1000), int(ty * 1000))
        msg = python_can.Message(arbitration_id=self._state_update_id, dlc=8,
                          is_extended_id=False, data=data)
        self.bus.send(msg)

        # Also update local state
        for k, v in state_dict.items():
            if k in self.last_known_state:
                self.last_known_state[k] = v

    def _get_tilt_reading(self, attempt=0):
        """Request tilt sensor reading from Teensy."""
        return self._run_to_completion(self._tilt_reading_steps(attempt))

    def _tilt_reading_steps(self, attempt=0):
        """Generator: request and wait for tilt reading from Teensy."""
        self._tilt_reading = None
        call_msg = python_can.Message(arbitration_id=self._tilt_reading_id, dlc=1,
                               is_extended_id=False, data=b'\x01')
        self.bus.send(call_msg)

        max_timeouts = 5
        timeout_count = 0
        deadline = time.time() + 1.0
        while self._tilt_reading is None:
            if time.time() > deadline:
                timeout_count += 1
                if timeout_count >= max_timeouts:
                    self.get_logger().error("Tilt reading failed after max retries")
                    return (None, None, Quaternion(w=1.0))
                self.get_logger().warning("Tilt request timeout, resending...")
                self.bus.send(call_msg)
                deadline = time.time() + 1.0
            yield 0.1  # Poll every 100ms

        tx, ty = self._tilt_reading
        self._tilt_reading = None

        if abs(tx) > hw.JB_OP_MAX_VALID_TILT_RAD or abs(ty) > hw.JB_OP_MAX_VALID_TILT_RAD:
            if attempt < 3:
                self.get_logger().warning("Tilt reading invalid, retrying...")
                return (yield from self._tilt_reading_steps(attempt + 1))
            return (None, None, Quaternion(w=1.0))

        return (tx, ty, self._tilt_to_quat(tx, ty))

    def _send_hand_traj_cmd(self, event_delay, event_vel, traj_type):
        """Send a hand trajectory command to the Teensy."""
        if event_delay is None or event_delay <= 0:
            raise ValueError(f"Invalid event delay: {event_delay}")
        if event_vel is None or event_vel < hw.TEENSY_TRAJ_MIN_EVENT_VEL_MPS or event_vel > hw.TEENSY_TRAJ_MAX_EVENT_VEL_MPS:
            raise ValueError(f"Invalid event velocity: {event_vel}")
        if traj_type not in (0, 1, 2):
            raise ValueError(f"Invalid trajectory type: {traj_type}")

        # Ensure hand is ready
        self.bus.send(odrive.encode_set_state(odrive.HAND_AXIS, 'CLOSED_LOOP'))
        self.bus.send(odrive.encode_set_controller_mode(odrive.HAND_AXIS, 'POSITION', 'PASSTHROUGH'))

        wall_time_ms = int(time.time() * 1000) + int(event_delay * 1000)
        wall_time_ms_low32 = wall_time_ms & 0xFFFFFFFF
        vel_u16 = int(round(event_vel * 100))

        payload = bytes([traj_type & 0xFF, vel_u16 & 0xFF, (vel_u16 >> 8) & 0xFF])
        payload += struct.pack('<I', wall_time_ms_low32) + bytes([0])

        msg = python_can.Message(arbitration_id=self._hand_traj_cmd_id,
                          is_extended_id=False, data=payload, dlc=8)
        self.bus.send(msg, timeout=0.002)

    def _smooth_move_hand(self, target_rev):
        """Command the Teensy to smooth-move the hand to target_rev."""
        if target_rev < 0 or target_rev > odrive.HAND_MOTOR_MAX_POSITION:
            raise ValueError(f"Invalid target: {target_rev:.3f} rev")

        self.bus.send(odrive.encode_set_state(odrive.HAND_AXIS, 'CLOSED_LOOP'))
        self.bus.send(odrive.encode_set_controller_mode(odrive.HAND_AXIS, 'POSITION', 'PASSTHROUGH'))

        payload = bytes([3]) + struct.pack('<f', target_rev) + bytes(3)
        msg = python_can.Message(arbitration_id=self._hand_traj_cmd_id,
                          is_extended_id=False, data=payload, dlc=8)
        self.bus.send(msg, timeout=0.002)

    # ═══════════════════════════════════════════════════════════
    # Utility
    # ═══════════════════════════════════════════════════════════

    def _tilt_to_quat(self, tilt_x, tilt_y):
        """Convert inclinometer tilt readings to a ROS Quaternion."""
        q_roll = quaternion.from_rotation_vector([-tilt_x, 0, 0])
        q_pitch = quaternion.from_rotation_vector([0, -tilt_y, 0])
        result = q_roll * q_pitch * self._last_tilt_offset
        self._last_tilt_offset = result

        quat = Quaternion()
        quat.x = result.x
        quat.y = result.y
        quat.z = result.z
        quat.w = result.w
        return quat

    def _update_disarm_error_state(self):
        """Update error state with disarmed axis info."""
        disarmed = [a for a in odrive.JUGGLEBOT_AXES if self.motors.last_states[a].disarm_reason != 0]
        entry = f"Disarmed axes: {disarmed}"
        self.last_known_state['error'] = [
            e for e in self.last_known_state['error'] if not e.startswith("Disarmed axes:")
        ]
        self.last_known_state['error'].append(entry)

    def _check_encoder_search_status(self):
        """Check whether encoder search has been completed (read from ODrives)."""
        return self._run_to_completion(self._encoder_status_steps())

    def _encoder_status_steps(self):
        """Generator: query ODrives for encoder search completion status."""
        self.motors.reset_encoder_search_feedback()
        for axis_id in odrive.LEG_AXES:
            self.bus.send(odrive.encode_sdo_read(axis_id, 'commutation_mapper.pos_abs'))

        retries = 0
        deadline = time.time() + 1.0
        while any(f is None for f in self.motors.encoder_search_feedback) and retries < 2:
            if time.time() > deadline:
                retries += 1
                deadline = time.time() + 1.0
                for axis_id in odrive.LEG_AXES:
                    self.bus.send(odrive.encode_sdo_read(axis_id, 'commutation_mapper.pos_abs'))
            yield 0.01  # Poll every 10ms

        complete = all(f is True for f in self.motors.encoder_search_feedback)
        self.last_known_state['encoder_search_complete'] = complete
        return complete

    def on_shutdown(self):
        """Handle node shutdown."""
        self.get_logger().info("Shutting down CanInterfaceNode...")
        try:
            if not self.stowed_due_to_error:
                self._gently_move_to_setpoint(0.0, deactivating=True)
            self.bus.send(odrive.encode_set_state(odrive.HAND_AXIS, 'IDLE'))
            self.bus.close()
        except Exception as e:
            self.get_logger().error(f"Shutdown error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CanInterfaceNode()
    try:
        while rclpy.ok() and not node.shutdown_flag:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
