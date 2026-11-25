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

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.logging import LoggingSeverity

from jugglebot_interfaces.msg import RobotState as RobotStateMsg
from jugglebot_interfaces.msg import TrajectoryStatus
from jugglebot_interfaces.srv import (
    ActivateOrDeactivate, GetTiltReadingService, LevelledActivateTargets,
    ODriveCommandService, SetString,
)
from jugglebot_interfaces.action import HomeMotors, Juggle
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger
from diagnostic_msgs.msg import DiagnosticStatus

from jugglebot.state_machine import (
    RobotState, Context, build_default_machine, BOOT_TIMEOUT_S,
    LEG_STATES_MAX_AGE_S,
)
from jugglebot import mocap_status as mocap_st
from jugglebot.juggle_request import parse_juggle_request
from jugglebot.platform_imu_gate import ImuGate
from jugglebot.can import odrive


# ── GUI Start button → jugglebot/juggle action goal (R4, owner decision D3)
# ── ─────────────────────────────────────────────────────────────────────────
# The Juggle goal's numeric fields all carry the "0 => the node's own
# parameter" sentinel (Juggle.action) — the relay never encodes a launch
# default itself, mirroring the retired reload relay's own discipline (the
# numeric default lived ONLY in the coordinator, never here or in the GUI).
# A numeric field the operator DID set in the GUI's Juggle panel arrives as a
# ``key=value`` token and is relayed verbatim; skill_node stays the authority
# on whether that value is admissible (it refuses an apex/separation the nomination
# cannot plan, with an honest code before anything moves).
#
# The request grammar (goal field -> parser, strict on unknown tokens) lives in
# the pure module `jugglebot.juggle_request`, shared with skill_node's
# `skills/nominate` (which allows two extra keys; this relay does not).


# ── Operator-command feedback (log-only; the handlers own the real logic) ──
# Which commands each state's handler actually consumes (state_machine.py:
# IdleHandler / ActiveHandler / FaultHandler; BOOT, HOMING and LEVELLING clear
# their command queue every tick). Used ONLY to tell the operator, in one WARN
# line, that a command was ignored and why — the command is still enqueued
# exactly as before, so control flow is unchanged. (While a deactivate is in
# flight ActiveHandler stays ACTIVE and DISCARDS the streaming mode switches
# below, one WARN notice each; a duplicate 'deactivate' is absorbed; and with
# the wire disarmed in an auto-arm session it refuses them -- see
# state_machine.ActiveHandler._step_deactivate.)
_COMMANDS_ACCEPTED = {
    RobotState.IDLE: ('activate', 'home', 'level'),
    RobotState.ACTIVE: ('deactivate', 'standby', 'trajectory', 'spacemouse',
                        'gui'),
    RobotState.FAULT: ('clear_errors',),
}
# Commands whose success is a state transition: the transition line carries the
# command name ("activate: IDLE -> ACTIVE") instead of a separate receipt line.
_COMMAND_TARGET = {
    'activate': 'ACTIVE', 'deactivate': 'IDLE', 'home': 'HOMING',
    'level': 'LEVELLING',
}


def _fmt_xy(v):
    """'(+0.0063, +0.0084)' — a rounded rad pair for one-line operator output."""
    return f'({float(v[0]):+.4f}, {float(v[1]):+.4f})'


class OrchestratorNode(Node):
    def __init__(self):
        super().__init__('orchestrator_node')
        # The node logs its per-step detail at DEBUG (recorded to the log file,
        # hidden from the launch shell by launch_console). No .debug() call here
        # fires at a sustained rate.
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        # ── State machine ─────────────────────────────────────────
        self.ctx = Context()
        self._last_cmd = None            # last accepted transition command
        self._last_active_mode = None
        self._current_req = None         # request being dispatched (log label)
        self._pending_label = None       # label of the in-flight service call
        self.sm = build_default_machine(log_fn=self._on_sm_log)

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
        # Which request the in-flight _pending_future belongs to. Needed because
        # a service RESULT has to be interpreted in the light of what was asked:
        # a bb/calibrate refusal is a skip, an activate failure is a fault. See
        # _check_pending_operations' QTM-race branch.
        self._pending_kind = None
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
        # BUG-130 (C-LEVEL-1.A): the six Levelled Active pose Leg targets, from
        # trajectory_node (NOT the bridge), fetched in IDLE before 'activate'.
        self._levelled_targets_client = self.create_client(
            LevelledActivateTargets, 'trajectory/levelled_activate_targets')
        # A2 — the bridge's runtime arming service (SetBool: true=arm,
        # false=disarm). The orchestrator is the production owner of WHEN;
        # the bridge's _arm_setpoint_output owns SAFE-TO (the 5-precondition
        # stream-then-arm check).
        self._setpoint_output_client = self.create_client(
            SetBool, 'set_setpoint_output')
        # Hand axis state (the bridge's /set_hand_state, SET_AXIS_STATE on axis
        # 6 only — no leg command). Used by 'disarm_and_idle_hand'.
        self._hand_state_client = self.create_client(
            SetString, 'set_hand_state')

        # Platform IMU upload gate (IDEA-20 P3c). Cached status only: the command path never makes a
        # service call to decide; the abort below is fire-and-forget.
        self._imu_gate = ImuGate()
        self._imu_abort_client = self.create_client(Trigger, 'platform_imu/abort')

        # ── Action clients ────────────────────────────────────────
        self._home_client = ActionClient(self, HomeMotors, 'home_motors')
        # Juggle relay (R4, owner decision D3 — replaces the retired reload
        # relay onto jugglebot/reload / reload_coordinator_node): the browser
        # GUI cannot call the jugglebot/juggle ACTION directly — rosbridge on
        # Foxy exposes no action op, and roslib ships only the ROS1 actionlib
        # client (topic protocol a ROS2 action server never advertises). So
        # the GUI hits the jugglebot/juggle_request SetString service below,
        # which relays ONE Juggle goal fire-and-forget through this client;
        # jugglebot/juggle_stop cancels whatever goal that relay last
        # dispatched.
        self._juggle_client = ActionClient(self, Juggle, 'jugglebot/juggle')
        # The live goal handle the last jugglebot/juggle_request dispatch
        # produced (set once `_on_juggle_goal_response` sees it ACCEPTED;
        # `None` before the first dispatch, after a REJECT, or once the goal
        # reaches a terminal result) — the ONLY thing `_svc_juggle_stop` can
        # cancel (this node holds no other handle to a running attempt).
        self._juggle_goal_handle = None

        # ── Service servers ───────────────────────────────────────
        self.create_service(
            SetString, 'jugglebot/juggle_request', self._svc_juggle_request)
        self.create_service(
            Trigger, 'jugglebot/juggle_stop', self._svc_juggle_stop)

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
        self.create_subscription(
            DiagnosticStatus, 'platform_imu/status', self._on_platform_imu_status, 10)
        # BUG-130 (C-LEVEL-1.A / C-LEVEL-1.O): whether trajectory_node holds a
        # levelling correction, and how fresh that answer is. Cached here; the
        # IDLE activate refusal (state_machine.activate_levelling_refusal)
        # reads ctx.gravity_correction_loaded + ctx.traj_status_age_s.
        self._traj_status_mono = None
        self.create_subscription(
            TrajectoryStatus, 'trajectory/status', self._on_trajectory_status, 10)
        # F4/Q3 — mocap_node's QTM view. HOMING's bb_calibrate step is
        # dispatched from this node, so this node needs its OWN answer to "can
        # a sweep produce data": asking the bridge would mean calling the
        # service, and the bridge's refusal is a success=False that
        # HomingHandler turns into a FAULT (state_machine.py, 'operation_result
        # is False -> FAULT'). Reading the same topic lets HOMING SKIP instead.
        # The predicate is shared (jugglebot.mocap_status) so the two views of
        # "ready" cannot drift apart.
        self._mocap_status_kv = None
        self._mocap_status_mono = 0.0
        # String literal, not mocap_st.MOCAP_STATUS_TOPIC — the choreography
        # map resolves endpoint names by AST and will not follow an attribute.
        self.create_subscription(
            DiagnosticStatus, 'mocap/status', self._on_mocap_status, 10)

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

        # Are all six legs holding a pose that a profiled DEACTIVATE can lower?
        # ODrive CLOSED_LOOP_CONTROL (odrive.AXIS_STATES['CLOSED_LOOP'], i.e.
        # protocol_config.ODRIVE_STATES — the constant the bridge's activate path
        # uses) with no active error, on every LEG axis. FaultHandler's
        # real-fault exit stow runs exactly when this is true, whatever state
        # the FAULT visit began in (ARMING_CONTRACT choreography step 6, FW 53:
        # an aborted rise can leave the six HELD mid-air). A message too short to carry
        # every leg reads False: no evidence the legs are holding.
        states = msg.motor_states
        self.ctx.legs_closed_loop = (
            len(states) > max(odrive.LEG_AXES)
            and all(states[i].current_state == odrive.AXIS_STATES['CLOSED_LOOP']
                    and states[i].active_errors == 0
                    for i in odrive.LEG_AXES))

        # ARMING_CONTRACT A6 inputs: the six Leg axis states, and since when all
        # six have read IDLE without a break (the StateMachine's homing gate).
        # A message too short to carry every Leg is "not known": no Homing.
        now = time.monotonic()
        prev = self.ctx.leg_states_time
        if prev is not None and now - prev > LEG_STATES_MAX_AGE_S:
            # A gap in /robot_state: the Legs may have been re-energised and
            # idled again in it, so the old all-Idle run proves nothing. The
            # settle restarts from this message (A6 freshness).
            self.ctx.legs_idle_since = None
        if len(states) > max(odrive.LEG_AXES):
            self.ctx.leg_states = tuple(int(states[i].current_state)
                                        for i in odrive.LEG_AXES)
            self.ctx.leg_states_time = now
            # FW 53 U6b (R3): the per-Leg drop evidence the FAULT exit's mixed-case
            # recovery reads (jugglebot/leg_reference.py). A gap since the last
            # full message beyond the A6 freshness bound marks every open drop
            # record (it could hide a drive reboot).
            last = getattr(self, '_leg_drops_fed_t', None)
            self.ctx.leg_drops.observe(
                self.ctx.leg_states,
                [int(states[i].active_errors) for i in odrive.LEG_AXES],
                [int(states[i].disarm_reason) for i in odrive.LEG_AXES],
                [float(states[i].pos_estimate) for i in odrive.LEG_AXES],
                gap=last is not None and now - last > LEG_STATES_MAX_AGE_S)
            self._leg_drops_fed_t = now
        else:
            self.ctx.leg_states = None
            self.ctx.leg_states_time = None
        if (self.ctx.leg_states is not None
                and all(s == odrive.AXIS_STATES['IDLE']
                        for s in self.ctx.leg_states)):
            if self.ctx.legs_idle_since is None:
                self.ctx.legs_idle_since = time.monotonic()
        else:
            self.ctx.legs_idle_since = None

        self.ctx.firmware_validated = msg.firmware_validated
        self.ctx.encoder_search_complete = msg.encoder_search_complete
        self.ctx.is_homed = msg.is_homed
        self.ctx.errors = list(msg.error)

        # Typed error flags from the CAN node — no string parsing needed.
        self.ctx.fatal_error = msg.has_fatal_odrive_error
        self.ctx.fatal_can_error = msg.has_fatal_can_error
        self.ctx.undervoltage = msg.has_undervoltage

        # Levelling state, held in the PLATFORM Teensy's RAM: it survives a relaunch AND a can-bridge reset (RobotState.msg).
        # Skip updates while LEVELLING — the state machine is computing
        # new values and we must not overwrite them with stale CAN data.
        if self.sm.state != RobotState.LEVELLING:
            self.ctx.levelling_complete = msg.levelling_complete
            if len(msg.pose_offset_rad) >= 2:
                self.ctx.pose_offset_rad = list(msg.pose_offset_rad[:2])

    def _on_command(self, msg):
        """Queue a user command for the state machine.

        Operator feedback is ONE line per command: a refused command gets a WARN
        here saying why; a command that changes state is reported by the
        transition line (``activate: IDLE -> ACTIVE``, see ``_on_sm_log``);
        ``clear_errors`` gets its own INFO; sub-mode switches are reported by
        ``_tick`` when the handler applies them.
        """
        cmd = msg.data
        state = self.sm.state
        # Platform IMU upload gate (IDEA-20 P3c): the one call site for the orchestrator_command topic.
        # A refused motion-starting command is NOT enqueued; 'deactivate' always passes and aborts.
        if not self._imu_gate_passes(cmd):
            return
        self.ctx.enqueue_command(cmd)
        if cmd not in _COMMANDS_ACCEPTED.get(state, ()):
            homes = [st.name for st, cmds in _COMMANDS_ACCEPTED.items()
                     if cmd in cmds]
            if homes:
                self.get_logger().warning(
                    f"'{cmd}' ignored: robot is {state.name} "
                    f"('{cmd}' works from {' or '.join(homes)})")
            else:
                self.get_logger().warning(
                    f"'{cmd}' ignored: not a known command "
                    f"(robot is {state.name})")
            return
        if cmd in _COMMAND_TARGET:
            self._last_cmd = cmd
        elif cmd == 'clear_errors':
            self.get_logger().info(
                'clear_errors: clearing ODrive errors and the guard latch')

    def _on_trajectory_status(self, msg):
        """Cache trajectory/status's gravity_correction_loaded + its arrival
        time (C-LEVEL-1.O: the applier's own answer, valid only while fresh)."""
        self.ctx.gravity_correction_loaded = bool(
            getattr(msg, 'gravity_correction_loaded', False))
        self._traj_status_mono = time.monotonic()

    def _on_platform_imu_status(self, msg):
        try:
            self._imu_gate.observe({kv.key: kv.value for kv in msg.values}, time.monotonic())
        except Exception as e:  # noqa: BLE001 -- the cache can never break the node
            self.get_logger().debug(f'platform_imu/status ignored: {e}')

    def _imu_gate_decision(self, command):
        """The ONE gate call (never raises, never blocks, reads cached status). Sends the abort the decision
        asks for. Returns the GateDecision."""
        d = self._imu_gate.check(command, time.monotonic())
        if d.abort:
            try:
                self._fire_forget_service_call(
                    self._imu_abort_client, Trigger.Request(), 'platform_imu/abort')
            except Exception as e:  # noqa: BLE001
                self.get_logger().debug(f'platform_imu/abort not sent: {e}')
        return d

    def _imu_gate_passes(self, command):
        d = self._imu_gate_decision(command)
        if not d.allow:
            self.get_logger().warning(d.reason)
        return d.allow

    def _on_sm_log(self, msg):
        """State-machine log sink: one operator line per transition.

        'Entering X' is DEBUG (the transition line already said it). A
        transition triggered by an operator command is prefixed with that
        command; a forced one keeps its '(forced)' cause. LEVELLING -> IDLE is
        DEBUG because the 'levelled: ...' result line already ends the sequence.
        """
        if msg.startswith('No handler registered'):
            self.get_logger().warning(f'[SM] {msg}')
            return
        if ' -> ' not in msg:
            self.get_logger().debug(f'[SM] {msg}')
            return
        target = msg.split(' -> ', 1)[1].split(' ', 1)[0]
        cmd = self._last_cmd
        self._last_cmd = None
        if cmd is not None and _COMMAND_TARGET.get(cmd) == target:
            msg = f'{cmd}: {msg}'
        if msg == 'LEVELLING -> IDLE':
            self.get_logger().debug(f'[SM] {msg}')
        else:
            self.get_logger().info(msg)

    # ═══════════════════════════════════════════════════════════════
    # Juggle relay (GUI → jugglebot/juggle action)
    # ═══════════════════════════════════════════════════════════════
    #
    # The browser can only reach ROS via topics + services (rosbridge 1.3.1 on
    # Foxy has no action capability). The jugglebot/juggle_request SetString
    # service is the bridge to the jugglebot/juggle ACTION (skill_node — R4
    # owner decision D3, THE skill-stack start surface): it dispatches ONE
    # goal fire-and-forget and returns a dispatch ACK. It is NOT a
    # re-implementation of the goal-accept refusal ladder — skill_node stays
    # the sole authority on preconditions and the structured
    # COMPLETED/STOPPED/<end_code> outcome (surfaced here only via the
    # logger). Firing an ill-timed goal is therefore safe: skill_node
    # rejects/aborts it with an honest code.
    #
    # The relay is deliberately fire-and-forget. This node spins single-threaded,
    # so BLOCKING the service handler on the goal-acceptance or result future would
    # deadlock (the same thread must service that future). Mirrors ball_butler_node
    # bb/throw: send_goal_async + a goal-response → result callback chain that only
    # logs the terminal outcome.

    def _svc_juggle_request(self, req, res):
        """Relay a GUI juggle request to jugglebot/juggle (fire-and-forget
        ACK). ``req.data`` is
        ``<pattern>[,apex_m=<f>][,separation_mm=<f>][,dwell_s=<f>][,num_cycles=<n>]``
        (e.g. ``'hop,num_cycles=4'``, ``'columns,num_cycles=6,apex_m=0.8'``) — the
        GUI's own encoding (juggle-panel.js), never a second place that names
        the launch defaults: every numeric goal field the request does not
        name carries its own "0 => the node's parameter" sentinel."""
        d = self._imu_gate_decision('juggle')
        if not d.allow:
            res.success = False
            res.message = d.reason
            self.get_logger().warning(d.reason)
            return res
        if not self._juggle_client.server_is_ready():
            res.success = False
            res.message = 'juggle action server unavailable (jugglebot/juggle)'
            self.get_logger().warning(
                'Juggle request received but jugglebot/juggle server is not ready.')
            return res
        try:
            pattern, values = parse_juggle_request(req.data)
        except ValueError as e:
            res.success = False
            res.message = 'juggle request refused: %s in %r' % (e, req.data)
            self.get_logger().warning(res.message)
            return res
        goal = Juggle.Goal()
        goal.pattern = pattern
        goal.apex_m = float(values.get('apex_m', 0.0))
        goal.separation_mm = float(values.get('separation_mm', 0.0))
        goal.dwell_s = float(values.get('dwell_s', 0.0))
        goal.num_cycles = int(values.get('num_cycles', 0))
        send_future = self._juggle_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_juggle_goal_response)
        overrides = ''.join(', %s=%s' % kv for kv in sorted(values.items()))
        res.success = True
        res.message = (
            f'Juggle dispatched (pattern={pattern!r}{overrides}).')
        # skill_node (the action server) prints the attempt's start line.
        self.get_logger().debug(
            'Juggle requested via jugglebot/juggle_request — goal dispatched '
            f'(pattern={pattern!r}{overrides}).')
        return res

    def _svc_juggle_stop(self, req, res):
        """Cancel the live jugglebot/juggle goal (GUI Stop button — immediate,
        no hold-to-confirm, mirrors the retired reload relay's plain-click
        idiom). Fire-and-forget, same reason as the request relay: this node
        spins single-threaded, so blocking on the cancel response would
        deadlock."""
        handle = self._juggle_goal_handle
        if handle is None:
            res.success = False
            res.message = 'no jugglebot/juggle goal is running'
            self.get_logger().warning('Juggle stop ignored: no attempt is running.')
            return res
        handle.cancel_goal_async()
        res.success = True
        res.message = 'jugglebot/juggle cancel requested'
        # skill_node reports the STOPPED outcome; this is only the relay.
        self.get_logger().debug('Juggle stop requested via jugglebot/juggle_stop.')
        return res

    def _on_juggle_goal_response(self, future):
        """jugglebot/juggle goal accepted/rejected — chain to the result (log
        only), and latch the handle for `_svc_juggle_stop`."""
        try:
            goal_handle = future.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f'juggle goal send failed: {e}')
            return
        if not goal_handle.accepted:
            # A rejection is about the NEW goal only: it must not wipe the
            # handle of an earlier goal that is still live (Stop would then
            # answer "no attempt is running"). `_on_juggle_result` alone
            # clears the handle.
            # skill_node logs the rejection and its reason (it owns the goal).
            self.get_logger().debug(
                'juggle goal REJECTED (an attempt is already in progress?).')
            return
        self._juggle_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._on_juggle_result)

    def _on_juggle_result(self, future):
        """jugglebot/juggle terminal outcome — log COMPLETED / STOPPED /
        <end_code>, and clear the latched handle (nothing left to cancel)."""
        self._juggle_goal_handle = None
        try:
            result = future.result().result
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f'juggle result error: {e}')
            return
        if result.success:
            self.get_logger().debug(
                f'Juggle OK: {result.outcome} '
                f'({result.caught}/{result.throws} caught).')
        else:
            self.get_logger().debug(
                f'Juggle ended: {result.outcome} '
                f'({result.caught}/{result.throws} caught).')

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

    def _on_mocap_status(self, msg):
        """Cache mocap_node's ``mocap/status`` KeyValues (F4/Q3).

        Snapshot only — decode and stamp the arrival. No decision is taken
        here; ``_qtm_ready()`` reads the cache at the moment the HOMING step is
        dispatched, which is what makes STALENESS (rather than "the last thing
        we heard, whenever that was") the test.
        """
        try:
            kv = mocap_st.decode_status(msg.values)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'mocap/status decode error: {e}')
            return
        self._mocap_status_kv = kv
        self._mocap_status_mono = time.monotonic()

    def _qtm_ready(self):
        """``(ready, code, detail)`` — can a BB calibration sweep produce data?

        Same predicate and same two thresholds as the bridge's gate
        (``jugglebot.mocap_status.evaluate``); only the cached snapshot is this
        node's own. Fails closed.
        """
        age_s = time.monotonic() - self._mocap_status_mono
        return mocap_st.evaluate(self._mocap_status_kv, age_s)

    # ═══════════════════════════════════════════════════════════════
    # Main tick
    # ═══════════════════════════════════════════════════════════════

    def _tick(self):
        """State machine tick: check ops, detect errors, run tick, dispatch."""
        # 1. Check if a pending async operation completed
        self._check_pending_operations()
        # 1b. Age of the cached trajectory/status (BUG-130 activate refusal).
        self.ctx.traj_status_age_s = (
            None if self._traj_status_mono is None
            else time.monotonic() - self._traj_status_mono)

        # 2. Force FAULT on errors (from any non-FAULT state).
        #    A latched Teensy guard forces FAULT too (FIX 2), but ONLY from ACTIVE:
        #    the guard's MAX_DEVIATION/SETPOINT_STALE only latch while armed (an ACTIVE
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

        # 3-notice. A handler's operator notice (e.g. a refused deactivate that
        # keeps the robot ACTIVE), logged once.
        notices, self.ctx.operator_notices = self.ctx.operator_notices, []
        for notice in notices:
            self.get_logger().warning(notice)

        # 3-pre. Sub-mode switch (standby/trajectory/spacemouse/gui) applied by
        # ActiveHandler this tick — the one line for that operator command.
        # Not fired on ACTIVE entry (its on_enter resets to STANDBY).
        if (self.sm.state == RobotState.ACTIVE
                and self._last_sm_state == RobotState.ACTIVE
                and self._last_active_mode is not None
                and self.ctx.active_mode != self._last_active_mode):
            self.get_logger().info(
                f'mode: {self._last_active_mode.value} -> '
                f'{self.ctx.active_mode.value}')
        self._last_active_mode = self.ctx.active_mode

        # 3-pre-a. (F3/C4) Say WHY on entry to FAULT, once per visit.
        # The machine already logs '[SM] Entering FAULT', which tells an operator
        # nothing about the cause: ctx.errors — the robot_state.error[] strings
        # that forced the transition at step 2, now carrying F3's decoded
        # per-axis ODrive names — was visible only to whoever thought to echo a
        # 100 Hz topic. Edge-gated off _last_sm_state (the state at the END of
        # the previous tick), so a held FAULT logs once, not at 10 Hz; it fires
        # for BOTH the forced transition above and a handler-returned one,
        # because it reads the post-tick state rather than the force path.
        # (On the forced path the '[SM] Entering FAULT' line lands in the SAME
        # tick, just above this one; on a handler-returned transition the
        # machine logs 'X -> FAULT' now and 'Entering FAULT' next tick, so this
        # cause line sits between them — adjacent either way.)
        # WARNING level, not error: FAULT itself is already logged, and a
        # guard-only fault (no ODrive error at all) is a routing state, not a
        # failure.
        if (self.sm.state == RobotState.FAULT
                and self._last_sm_state != RobotState.FAULT):
            if self.ctx.errors:
                self.get_logger().warning(
                    'FAULT cause — robot_state.error[]: '
                    + ' | '.join(str(e) for e in self.ctx.errors))
            else:
                self.get_logger().warning(
                    'FAULT cause — no robot_state.error[] strings '
                    f'(guard_latched={self.ctx.guard_latched}, '
                    f'boot_timed_out={self.ctx.boot_timed_out})')

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
                'Restored saved level: gravity offset '
                f'{_fmt_xy(self.ctx.pose_offset_rad)} rad')
        self._last_sm_state = current

    # ═══════════════════════════════════════════════════════════════
    # Async operation management
    # ═══════════════════════════════════════════════════════════════

    @staticmethod
    def _is_qtm_refusal(message):
        """Is this bb/calibrate failure message the bridge's QTM refusal?

        The bridge formats it as ``f'{code}: {detail} — calibration refused'``
        with ``code`` one of ``jugglebot.mocap_status``' two constants, so the
        prefix test is against those imported constants and not a copied
        literal — a renamed code becomes a NameError at import instead of a
        predicate that silently stops matching and starts faulting HOMING.
        """
        text = str(message or '')
        return text.startswith((mocap_st.CODE_QTM_STALE,
                                mocap_st.CODE_BB_MARKERS_NOT_VISIBLE))

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
                    self.get_logger().debug(
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
                detail = getattr(result, 'message', 'unknown')
                if (not result.success
                        and self._pending_kind == 'bb_calibrate'
                        and self._is_qtm_refusal(detail)):
                    # THE RACE. _dispatch_request checked _qtm_ready() at
                    # dispatch; the bridge checks its OWN cache when the handler
                    # runs. Those are two instants, and the cached status only
                    # has to age past MOCAP_STATUS_MAX_AGE_S in between for a
                    # dispatch this node judged healthy to meet a bridge that
                    # judges it stale. The bridge then refuses with
                    # success=False — and HomingHandler turns that straight into
                    # FAULT, which is precisely the outcome the Q3 skip exists
                    # to prevent. Losing a race is not a reason to fault a
                    # robot, so a REFUSAL (not a failure) is treated exactly
                    # like the dispatch-time skip: WARN, mark, carry on.
                    #
                    # Matched on the shared code constants, never on prose: the
                    # bridge builds this message from the same
                    # jugglebot.mocap_status codes, so the two cannot drift into
                    # a silent non-match the way a copied string would.
                    self.get_logger().warning(
                        f'bb/calibrate refused by the bridge ({detail}) '
                        '— skipping, not faulting. BB will be uncalibrated '
                        'this session.')
                    self.ctx.bb_calibration_skipped = True
                    self.ctx.operation_result = True
                elif self._pending_kind == 'activate_targets':
                    # BUG-130: IdleHandler reads the targets and, on a refusal,
                    # logs the one WARN itself (with the remedy).
                    targets = [float(t) for t in
                               (getattr(result, 'leg_targets_rev', None) or ())]
                    self.ctx.activate_leg_targets_rev = (
                        targets if result.success else None)
                    self.ctx.operation_result = bool(result.success)
                    self.ctx.operation_message = detail
                    if result.success:
                        # DEBUG: the console keeps its one transition line.
                        self.get_logger().debug(f'activate: {detail}')
                else:
                    self.ctx.operation_result = result.success
                    self.ctx.operation_message = detail
                    if not result.success:
                        # A genuine RPC failure still faults: the CAN write did
                        # not land, which is a real machine problem, not an
                        # optional subsystem being unavailable.
                        # A failed stow in FAULT holds FAULT (state_machine
                        # FaultHandler._resolve_stow): say how to retry it.
                        hint = (' — FAULT holds (legs not stowed); '
                                'clear_errors retries the stow'
                                if (self._pending_label == 'deactivate'
                                    and self.sm.state == RobotState.FAULT)
                                else '')
                        self.get_logger().warning(
                            f'{self._pending_label or "operation"} failed: '
                            f'{detail}{hint}')
                    elif self._pending_label == 'clear_errors':
                        self.get_logger().info('clear_errors: done')
            except Exception as e:
                self.get_logger().error(f'Service call exception: {e}')
                self.ctx.operation_result = False
                self.ctx.operation_message = f'service call exception: {e}'
            self.ctx.operation_pending = False
            self._pending_future = None
            self._pending_kind = None

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
        self._current_req = req
        if req == 'encoder_search':
            self._start_service_call(
                self._encoder_search_client, Trigger.Request())

        elif req == 'home':
            self._start_home_action()

        elif req == 'bb_calibrate':
            qtm_ok, qtm_code, qtm_detail = self._qtm_ready()
            if not self._bb_calibrate_client.service_is_ready():
                # Ball Butler not available — skip calibration (BB is optional)
                self.get_logger().warning(
                    'Ball Butler not available — skipping calibration. '
                    'BB will be uncalibrated this session.')
                self.ctx.bb_calibration_skipped = True
                self.ctx.operation_result = True
            elif not qtm_ok:
                # F4/Q3 — SKIP, never fail. The bridge would refuse this same
                # call with success=False, and HomingHandler.execute turns
                # `operation_result is False` straight into FAULT
                # (state_machine.py) — so gating by *calling and catching* the
                # refusal would convert "the mocap PC is off" into a faulted
                # robot the operator has to clear before homing. Mirrors the
                # service-not-ready branch above exactly: WARN, mark skipped,
                # operation_result True so HOMING advances to IDLE.
                self.get_logger().warning(
                    f'QTM not delivering BB markers ({qtm_code}: {qtm_detail}) '
                    '— skipping calibration. BB will be uncalibrated this '
                    'session.')
                self.ctx.bb_calibration_skipped = True
                self.ctx.operation_result = True
            else:
                self._start_service_call(
                    self._bb_calibrate_client, Trigger.Request(),
                    kind='bb_calibrate')

        elif req == 'fetch_activate_targets':
            # BUG-130: IdleHandler's fetch of the Levelled Active pose, still in
            # IDLE. A refusal or an unready service resolves as
            # operation_result False and IdleHandler stays IDLE with a WARN.
            self.ctx.activate_leg_targets_rev = None
            self._start_service_call(
                self._levelled_targets_client,
                LevelledActivateTargets.Request(), kind='activate_targets')

        elif req == 'activate':
            # The operator's ACTIVATE (ActiveHandler): to the Levelled Active
            # pose IdleHandler fetched, consumed here (BUG-130, C-LEVEL-1.A).
            # Without fetched targets it is FAILED, never sent as the constant
            # pose: only IdleHandler's fetch leads here, so None is a bug.
            targets = self.ctx.activate_leg_targets_rev
            self.ctx.activate_leg_targets_rev = None
            if not targets:
                self.get_logger().error(
                    'activate: no Levelled Active pose was fetched -- not sent '
                    '(BUG-130: the constant pose is level\'s own ACTIVATE only)')
                self.ctx.operation_result = False
                self.ctx.operation_message = 'no Levelled Active pose fetched'
                return
            activate_req = ActivateOrDeactivate.Request()
            activate_req.command = 'activate'
            activate_req.leg_targets_rev = [float(t) for t in targets]
            self._start_service_call(self._activate_client, activate_req)

        elif req == 'deactivate':
            # FaultHandler's real-fault exit stow (ARMING_CONTRACT choreography
            # step 6) is the only 'deactivate' raised in a FAULT that was
            # already FAULT on the previous tick: ActiveHandler.on_exit's
            # deactivate is dispatched on the transition tick itself, when
            # _last_sm_state is still ACTIVE. Log-only; dispatch is identical.
            if (self.sm.state == RobotState.FAULT
                    and self._last_sm_state == RobotState.FAULT):
                self.get_logger().info(
                    'FAULT exit: stowing the legs (profiled deactivate) — '
                    'FAULT holds until the stow completes')
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

        elif req == 'disarm_and_idle_hand':
            # ActiveHandler's legs-out exits to IDLE (the legs already dropped
            # out of closed loop): make the wire and the Hand safe WITHOUT any
            # leg command — a leg DEACTIVATE would re-energise legs that fell
            # out of closed loop and lower them a second time. Both calls are
            # idempotent and fire-and-forget (same reasoning as above).
            disarm_req = SetBool.Request()
            disarm_req.data = False
            self._fire_forget_service_call(
                self._setpoint_output_client, disarm_req, 'disarm_setpoints')
            hand_req = SetString.Request()
            hand_req.data = 'IDLE'
            self._fire_forget_service_call(
                self._hand_state_client, hand_req, 'hand_idle')

        # ── Levelling requests ────────────────────────────────────

        elif req == 'level_activate':
            # The CONSTANT Active pose, deliberately (BUG-130): the gravity
            # offset is DEFINED as the tilt measured at the uncorrected neutral.
            # A levelled rise here would read about 0 and overwrite a good
            # offset. Never set leg_targets_rev on this request.
            activate_req = ActivateOrDeactivate.Request()
            activate_req.command = 'activate'
            self._start_service_call(self._activate_client, activate_req)

        elif req == 'level_get_tilt':
            self._start_tilt_service_call()

        elif req == 'level_send_correction':
            msg = Float64MultiArray(
                data=list(self.ctx.pose_offset_rad))
            self._gravity_offset_pub.publish(msg)
            self.get_logger().debug(
                f'Gravity offset published: {self.ctx.pose_offset_rad}')
            self.ctx.operation_result = True

        elif req == 'level_persist_state':
            msg = Float64MultiArray(
                data=[1.0] + list(self.ctx.pose_offset_rad))
            self._level_state_pub.publish(msg)
            self.ctx.levelling_complete = True
            # The one line that ends a level: raw tilt -> saved gravity offset.
            self.get_logger().info(
                f'levelled: tilt {_fmt_xy(self.ctx.tilt_reading)} rad -> '
                f'gravity offset {_fmt_xy(self.ctx.pose_offset_rad)} rad, saved')
            self.ctx.operation_result = True

        elif req == 'level_mocap_check':
            # A TODO stub, not a state: nothing to tell the operator.
            self.get_logger().debug(
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
        self._pending_kind = None
        self._pending_goal_future = None
        self._pending_result_future = None
        self._pending_tilt_future = None
        self.ctx.operation_pending = False
        self.ctx.operation_result = None
        self.ctx.clear_requests()
        self.ctx.clear_commands()

    def _start_service_call(self, client, request, kind=None):
        """Start a non-blocking service call.

        ``kind`` records WHICH request this future belongs to, so
        ``_check_pending_operations`` can interpret the result in context (only
        bb_calibrate uses it today). Left None by callers that need no such
        interpretation — a None kind matches nothing.
        """
        if not client.service_is_ready():
            self.get_logger().warning(
                f'Service {client.srv_name} not ready, failing request')
            self.ctx.operation_result = False
            self.ctx.operation_message = f'service {client.srv_name} not ready'
            return

        self.ctx.operation_pending = True
        self.ctx.operation_result = None
        self.ctx.operation_message = None
        self._pending_kind = kind
        self._pending_label = self._current_req
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
        armed (mpc_active=1), the resulting stream silence latches an SETPOINT_STALE
        E-STOP within 250 ms (trajectory_node Sharp Edge #1). That latched guard is
        exactly what makes the bridge's own profiled DEACTIVATE impossible. So the
        safest shutdown action for the orchestrator is to command NOTHING and let
        the bridge disarm-then-stow in the correct order.
        """
        self.get_logger().info(
            'Orchestrator shutdown — profiled stow is owned by teensy_bridge_node')


    def destroy_node(self):
        """Destroy the action clients BEFORE the node handle.

        Foxy destroys the node first and garbage-collects the ActionClients
        later, whose ``__del__`` then raises ``InvalidHandle`` ("Exception
        ignored in ... ActionClient.__del__") on every Ctrl-C. Explicit,
        guarded destroy makes the later ``__del__`` a no-op.
        """
        for client in (getattr(self, '_home_client', None),
                       getattr(self, '_juggle_client', None)):
            destroy = getattr(client, 'destroy', None)
            if destroy is None:
                continue
            try:
                destroy()
            except Exception:  # noqa: BLE001 — already destroyed is fine
                pass
        super().destroy_node()


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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
