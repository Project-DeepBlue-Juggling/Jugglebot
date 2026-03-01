"""Extensible state machine for Jugglebot orchestration.

Pure Python — no ROS2 dependency. Each state is a handler class implementing
on_enter/execute/on_exit. New states are added by:
    1. Adding a value to the RobotState enum
    2. Writing a handler class (subclass StateHandler)
    3. Calling state_machine.register(RobotState.NEW, handler)

The orchestrator node drives the machine by calling sm.tick(ctx) on a timer.
"""

import math
import time
from collections import deque
from enum import Enum, auto


# ── States ────────────────────────────────────────────────────────

class RobotState(Enum):
    BOOT = auto()
    HOMING = auto()
    IDLE = auto()
    LEVELLING = auto()
    ACTIVE = auto()
    FAULT = auto()


class ErrorSeverity(Enum):
    TRANSIENT = auto()   # Soft errors: may auto-clear (e.g., undervoltage after E-stop release)
    FATAL = auto()       # Hard errors: require operator intervention or reboot


class ActiveMode(Enum):
    """Sub-modes within the ACTIVE state."""
    SPACEMOUSE = 'SPACEMOUSE'
    SHELL = 'SHELL'


# ── Timeouts ─────────────────────────────────────────────────────

BOOT_TIMEOUT_S = 30.0  # Max time to wait for ODrive heartbeats before FAULT


# ── Context ───────────────────────────────────────────────────────

class Context:
    """Mutable context shared between state handlers and orchestrator node.

    The orchestrator populates 'input' fields before each tick.
    Handlers read inputs and set 'output' fields for the orchestrator to act on.
    """

    def __init__(self):
        # ── Inputs (set by orchestrator from /robot_state) ────────
        self.all_heartbeats = False          # All 7 Jugglebot axes reporting
        self.firmware_validated = False       # All ODrives report consistent firmware
        self.encoder_search_complete = False
        self.is_homed = False
        self.errors = []                     # Error strings from /robot_state
        self.fatal_error = False
        self.fatal_can_error = False
        self.undervoltage = False

        # Async operation result (set by orchestrator when a requested op completes)
        self.operation_pending = False
        self.operation_result = None         # None = not complete, True/False = result

        # User command queue (set by orchestrator from command topic, consumed by handler)
        self._command_queue = deque(maxlen=4)

        # ── Outputs (set by handlers, read by orchestrator) ───────
        self.request = None                  # Operation request: 'encoder_search', 'home', etc.
        self.control_mode = None             # Control mode string to publish (None = no change)

        # ── Shared state ──────────────────────────────────────────
        self.active_mode = ActiveMode.SPACEMOUSE
        self.error_severity = None
        self.boot_timed_out = False

        # ── Levelling state ──────────────────────────────────────
        self.levelling_complete = False          # Persisted on Teensy
        self.pose_offset_rad = [0.0, 0.0]        # Accumulated [tiltX, tiltY] correction
        self.tilt_reading = [0.0, 0.0]           # Raw tilt from inclinometer (set by orchestrator)

    @property
    def has_fatal_error(self):
        return self.fatal_error or self.fatal_can_error

    def enqueue_command(self, cmd):
        """Add a command to the queue. Oldest commands are dropped if full."""
        self._command_queue.append(cmd)

    def consume_command(self):
        """Pop and return the oldest pending command, or None."""
        if self._command_queue:
            return self._command_queue.popleft()
        return None

    def clear_commands(self):
        """Discard all pending commands."""
        self._command_queue.clear()


# ── Handler base class ────────────────────────────────────────────

class StateHandler:
    """Base class for state handlers. Override on_enter/execute/on_exit."""

    def on_enter(self, ctx):
        """Called once when entering this state."""

    def execute(self, ctx):
        """Called each tick. Return a RobotState to transition, or None to stay."""
        return None

    def on_exit(self, ctx):
        """Called once when leaving this state."""


# ── State Machine ─────────────────────────────────────────────────

class StateMachine:
    """Registry-based state machine.

    Drives registered handlers through on_enter -> execute -> on_exit lifecycle.
    Supports forced transitions (e.g., error -> FAULT) via force_transition().
    """

    def __init__(self, initial_state, log_fn=None):
        self._state = initial_state
        self._handlers = {}
        self._log = log_fn or (lambda msg: print(f'[SM] {msg}'))
        self._entered = False

    def register(self, state, handler):
        self._handlers[state] = handler

    @property
    def state(self):
        return self._state

    def force_transition(self, new_state, ctx):
        """Force immediate transition, calling on_exit for current state."""
        if new_state == self._state:
            return
        self._log(f'{self._state.name} -> {new_state.name} (forced)')
        handler = self._handlers.get(self._state)
        if handler and self._entered:
            handler.on_exit(ctx)
        self._state = new_state
        self._entered = False

    def tick(self, ctx):
        """Run one tick: enter (if new), execute, and handle transitions."""
        handler = self._handlers.get(self._state)
        if handler is None:
            self._log(f'No handler registered for {self._state.name}')
            return

        if not self._entered:
            self._log(f'Entering {self._state.name}')
            handler.on_enter(ctx)
            self._entered = True

        next_state = handler.execute(ctx)
        if next_state is not None and next_state != self._state:
            self._log(f'{self._state.name} -> {next_state.name}')
            handler.on_exit(ctx)
            self._state = next_state
            self._entered = False


# ══════════════════════════════════════════════════════════════════
# Built-in state handlers
# ══════════════════════════════════════════════════════════════════

class BootHandler(StateHandler):
    """Wait for heartbeats and firmware validation from all ODrive axes."""

    def __init__(self):
        self._entry_time = 0.0

    def on_enter(self, ctx):
        self._entry_time = time.time()
        ctx.boot_timed_out = False
        ctx.control_mode = ''

    def execute(self, ctx):
        ctx.clear_commands()  # Commands not valid during BOOT

        if not ctx.all_heartbeats:
            if time.time() - self._entry_time > BOOT_TIMEOUT_S:
                ctx.boot_timed_out = True
                return RobotState.FAULT
            return None

        # Wait for firmware version check to complete.
        # On mismatch, the orchestrator forces FAULT via the error path
        # before this timeout fires.  This timeout only covers the edge
        # case where Get_Version responses never arrive.
        if not ctx.firmware_validated:
            if time.time() - self._entry_time > BOOT_TIMEOUT_S:
                ctx.boot_timed_out = True
                return RobotState.FAULT
            return None

        # Skip ahead if already homed (e.g., recovery from transient FAULT)
        if ctx.is_homed:
            return RobotState.IDLE
        return RobotState.HOMING


class HomingHandler(StateHandler):
    """Run encoder search, then homing, then optional BB calibration."""

    def __init__(self):
        self._phase = 'encoder_search'

    def on_enter(self, ctx):
        ctx.operation_result = None
        if ctx.encoder_search_complete:
            self._phase = 'home'
        else:
            self._phase = 'encoder_search'
        ctx.request = self._phase

    def execute(self, ctx):
        ctx.clear_commands()  # Commands not valid during HOMING

        # Wait for current operation
        if ctx.operation_result is None:
            return None

        if ctx.operation_result is False:
            return RobotState.FAULT

        # Advance to next phase
        if self._phase == 'encoder_search':
            self._phase = 'home'
        elif self._phase == 'home':
            self._phase = 'bb_calibrate'
        elif self._phase == 'bb_calibrate':
            return RobotState.IDLE

        ctx.operation_result = None
        ctx.request = self._phase
        return None

    def on_exit(self, ctx):
        ctx.request = None


class IdleHandler(StateHandler):
    """Robot ready — waiting for activation command."""

    def on_enter(self, ctx):
        ctx.control_mode = ''

    def execute(self, ctx):
        # Wait for any pending operation (e.g., deactivation from ACTIVE)
        if ctx.operation_pending:
            return None

        cmd = ctx.consume_command()
        if cmd == 'activate':
            return RobotState.ACTIVE
        elif cmd == 'home':
            return RobotState.HOMING
        elif cmd == 'level':
            return RobotState.LEVELLING
        # Other commands silently discarded (already logged on receipt)

        return None


class LevellingHandler(StateHandler):
    """Platform levelling — measure tilt and compute gravity correction.

    Uses profiled position commands throughout (no control loop involvement).
    Single inclinometer read with experimentally-determined mounting offset.

    Phases: activate → settle → read_tilt → send_correction →
            persist → mocap_check → deactivate → IDLE
    """

    # Phase order — each phase sets ctx.request and waits for operation_result,
    # except 'settle' which is time-based.
    _PHASES = [
        'level_activate', 'settle', 'level_get_tilt',
        'level_send_correction', 'level_persist_state',
        'level_mocap_check', 'level_deactivate',
    ]

    def __init__(self, inclinometer_offset_deg, settle_s):
        self._offset_rad = [math.radians(d) for d in inclinometer_offset_deg]
        self._settle_s = settle_s
        self._phase_idx = 0
        self._settle_start = 0.0

    def on_enter(self, ctx):
        self._phase_idx = 0
        ctx.control_mode = 'LEVELLING'
        ctx.operation_result = None
        ctx.request = self._PHASES[0]  # 'level_activate'

    def execute(self, ctx):
        ctx.clear_commands()  # Commands not valid during LEVELLING
        phase = self._PHASES[self._phase_idx]

        # Settle phase is time-based, not operation-based
        if phase == 'settle':
            if time.time() - self._settle_start >= self._settle_s:
                return self._advance(ctx)
            return None

        # All other phases wait for operation_result
        if ctx.operation_result is None:
            return None

        if ctx.operation_result is False:
            return RobotState.FAULT

        # Phase-specific logic on success
        if phase == 'level_activate':
            # Activation succeeded — start settle timer
            self._settle_start = time.time()

        elif phase == 'level_get_tilt':
            # Apply mounting offset and store corrected tilt
            ctx.pose_offset_rad = [
                ctx.tilt_reading[0] + self._offset_rad[0],
                ctx.tilt_reading[1] + self._offset_rad[1],
            ]

        return self._advance(ctx)

    def _advance(self, ctx):
        """Move to the next phase, or finish."""
        self._phase_idx += 1
        if self._phase_idx >= len(self._PHASES):
            return RobotState.IDLE

        phase = self._PHASES[self._phase_idx]
        if phase == 'settle':
            # Settle is time-based — no request needed
            self._settle_start = time.time()
        else:
            ctx.operation_result = None
            ctx.request = phase
        return None

    def on_exit(self, ctx):
        ctx.request = None


class ActiveHandler(StateHandler):
    """Robot activated — accepting pose commands via sub-mode."""

    def __init__(self):
        self._activated = False

    def on_enter(self, ctx):
        self._activated = False
        ctx.request = 'activate'
        ctx.operation_result = None

    def execute(self, ctx):
        # Wait for activation to complete
        if not self._activated:
            if ctx.operation_result is None:
                return None
            if ctx.operation_result is False:
                return RobotState.FAULT
            self._activated = True
            ctx.control_mode = ctx.active_mode.value

        cmd = ctx.consume_command()
        if cmd == 'deactivate':
            return RobotState.IDLE
        elif cmd in ('spacemouse', 'shell'):
            ctx.active_mode = ActiveMode(cmd.upper())
            ctx.control_mode = ctx.active_mode.value
        # Other commands silently discarded (already logged on receipt)

        return None

    def on_exit(self, ctx):
        ctx.request = 'deactivate'
        ctx.control_mode = ''


class FaultHandler(StateHandler):
    """Error state — classify severity and wait for resolution.

    Recovery paths:
        - Boot timeout: ODrives never reported heartbeats. Auto-recovers if
          heartbeats arrive, or operator sends 'clear_errors' to retry.
          Exits to BOOT (needs heartbeats before anything else).
        - Transient (undervoltage): CAN node auto-clears once power returns,
          errors disappear from /robot_state, handler exits to BOOT.
        - Fatal: user sends 'clear_errors' via orchestrator_command,
          orchestrator calls odrive_command('clear_errors') on CAN node,
          errors clear, handler exits to BOOT.

    BOOT then re-validates heartbeats and either skips to IDLE (if already
    homed) or proceeds through HOMING (if ODrives rebooted).
    """

    def on_enter(self, ctx):
        ctx.control_mode = 'ERROR'

        # Classify error severity (useful for monitoring/logging).
        # Fatal takes precedence: undervoltage alongside a fatal error is still FATAL.
        if ctx.boot_timed_out:
            ctx.error_severity = ErrorSeverity.FATAL
        elif ctx.has_fatal_error:
            ctx.error_severity = ErrorSeverity.FATAL
        elif ctx.undervoltage:
            ctx.error_severity = ErrorSeverity.TRANSIENT
        else:
            ctx.error_severity = ErrorSeverity.FATAL

    def execute(self, ctx):
        # Auto-recover from boot timeout if heartbeats arrive
        if ctx.boot_timed_out and ctx.all_heartbeats:
            ctx.boot_timed_out = False

        # Exit FAULT when all error conditions clear.
        # BOOT re-validates heartbeats, then skips to IDLE if already homed.
        if not ctx.has_fatal_error and not ctx.errors and not ctx.boot_timed_out:
            return RobotState.BOOT

        cmd = ctx.consume_command()
        if cmd == 'clear_errors':
            ctx.request = 'clear_errors'
            ctx.boot_timed_out = False
        # Other commands silently discarded (already logged on receipt)

        return None

    def on_exit(self, ctx):
        ctx.error_severity = None


# ══════════════════════════════════════════════════════════════════
# Factory
# ══════════════════════════════════════════════════════════════════

def build_default_machine(log_fn=None):
    """Create a state machine with all default handlers registered."""
    from jugglebot.hardware_config import (
        JB_OP_INCLINOMETER_OFFSET_DEG, JB_OP_LEVELLING_SETTLE_S,
    )

    sm = StateMachine(RobotState.BOOT, log_fn=log_fn)
    sm.register(RobotState.BOOT, BootHandler())
    sm.register(RobotState.HOMING, HomingHandler())
    sm.register(RobotState.IDLE, IdleHandler())
    sm.register(RobotState.LEVELLING, LevellingHandler(
        inclinometer_offset_deg=JB_OP_INCLINOMETER_OFFSET_DEG,
        settle_s=JB_OP_LEVELLING_SETTLE_S,
    ))
    sm.register(RobotState.ACTIVE, ActiveHandler())
    sm.register(RobotState.FAULT, FaultHandler())
    return sm
