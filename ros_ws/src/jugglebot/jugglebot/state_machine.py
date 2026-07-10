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
    STANDBY = 'STANDBY'
    TRAJECTORY = 'TRAJECTORY'
    SPACEMOUSE = 'SPACEMOUSE'
    SHELL = 'SHELL'
    GUI = 'GUI'
    CATCH = 'CATCH'


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
        # FIX 2 — Teensy guard-latch awareness. Set by the orchestrator from the
        # bridge's /link_status (fault_state != NONE). The guard SUPPRESSES leg output
        # without disarming the ODrives, so it never appears in ``errors`` (which is
        # ODrive-level only) — the 2026-07-10 incident left the orchestrator BLIND to
        # a latched MAX_DEVIATION. The orchestrator forces FAULT on this only from
        # ACTIVE (see orchestrator_node._tick); FaultHandler then holds FAULT until it
        # clears, routing the existing clear_errors recovery.
        self.guard_latched = False
        # FIX 2 (F1) — one-shot: set by FaultHandler when a GUARD-ONLY fault clears and
        # exits back to ACTIVE. The ODrives never disarmed under a guard latch (it
        # suppresses output only) and trajectory_node kept streaming a hold throughout,
        # so ActiveHandler must RESUME already-armed — NOT re-run the ACTIVATE move (a
        # TRAP_TRAJ to the active pose would fight the live 40 Hz PASSTHROUGH stream and
        # lurch the legs up from the post-descent measured hold). Consumed (reset) by
        # ActiveHandler.on_enter, and cleared on any FAULT entry so it can never leak
        # into a real-fault→BOOT→re-activate path (which MUST re-arm).
        self.resume_active_no_rearm = False

        # Async operation result (set by orchestrator when a requested op completes)
        self.operation_pending = False
        self.operation_result = None         # None = not complete, True/False = result

        # User command queue (set by orchestrator from command topic, consumed by handler)
        self._command_queue = deque(maxlen=4)

        # ── Outputs (set by handlers, read by orchestrator) ───────
        self._request_queue = deque(maxlen=4)  # Operation requests: 'encoder_search', 'home', etc.
        self.control_mode = None             # Control mode string to publish (None = no change)

        # ── Shared state ──────────────────────────────────────────
        self.active_mode = ActiveMode.STANDBY
        self.error_severity = None
        self.boot_timed_out = False

        # ── Levelling state ──────────────────────────────────────
        self.levelling_complete = False          # Persisted on Teensy
        self.pose_offset_rad = [0.0, 0.0]        # Accumulated [tiltX, tiltY] correction
        self.tilt_reading = [0.0, 0.0]           # Raw tilt from inclinometer (set by orchestrator)
        self.bb_calibration_skipped = False       # True if BB calibration was skipped (BB unavailable)

    @property
    def has_fatal_error(self):
        return self.fatal_error or self.fatal_can_error

    # ── Request queue (replaces single ctx.request slot) ──────
    @property
    def request(self):
        """Read (peek) the oldest pending request, or None.

        Kept as a property for backward compatibility with handlers that
        check ``ctx.request``.
        """
        return self._request_queue[0] if self._request_queue else None

    @request.setter
    def request(self, value):
        """Set a request.  ``None`` is a no-op (clearing is done via drain/clear)."""
        if value is not None:
            self._request_queue.append(value)

    def drain_requests(self):
        """Pop and return all pending requests as a list, clearing the queue."""
        items = list(self._request_queue)
        self._request_queue.clear()
        return items

    def clear_requests(self):
        """Discard all pending requests."""
        self._request_queue.clear()

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
        ctx.clear_requests()


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
        ctx.clear_requests()


class ActiveHandler(StateHandler):
    """Robot activated — accepting pose commands via sub-mode."""

    def __init__(self):
        self._activated = False

    def on_enter(self, ctx):
        # Always reset to STANDBY on entry so re-activation never inherits a
        # prior sub-mode.  The orchestrator's control_mode is a target-routing
        # signal: STANDBY SILENCES move commands — trajectory_node keeps streaming
        # its current plan and holds at the pose it seeded or last reached (NOT a
        # neutral pose; return-to-neutral is the explicit trajectory/go_home
        # service). The operator then switches to TRAJECTORY / SPACEMOUSE / CATCH
        # to accept the corresponding command source.
        ctx.active_mode = ActiveMode.STANDBY

        # FIX 2 (F1) — guard-clear resume. A GUARD-ONLY fault (Teensy guard latched
        # during ACTIVE with NO ODrive error) never disarmed the legs and never left
        # the streaming set (FaultHandler preserved control_mode), and trajectory_node
        # held a profiled descent throughout. So on the way back from that fault we
        # must RESUME already-armed: re-running the ACTIVATE move here (a TRAP_TRAJ to
        # the active pose) would fight the live 40 Hz PASSTHROUGH stream and lurch the
        # legs up from the post-descent measured hold — potentially re-tripping the
        # guard. Consume the one-shot and go straight to the armed STANDBY hold.
        resume = ctx.resume_active_no_rearm
        ctx.resume_active_no_rearm = False
        if resume:
            self._activated = True
            ctx.control_mode = ctx.active_mode.value  # 'STANDBY'
            ctx.operation_result = None
            return

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
        elif cmd in ('standby', 'trajectory', 'spacemouse', 'shell', 'gui',
                     'catch'):
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
        - Guard-only (FIX 2/F1): the Teensy guard latched during ACTIVE with NO
          ODrive error. This is observability + clear_errors routing ONLY —
          control_mode is PRESERVED (streaming mode kept, never 'ERROR') so the
          40 Hz emitter keeps playing out trajectory_node's profiled guard descent.
          'clear_errors' clears the Teensy latch; once the guard drops, FAULT exits
          straight back to ACTIVE (the legs never disarmed), and ActiveHandler
          resumes WITHOUT re-arming.

    BOOT (the real-fault exit) then re-validates heartbeats and either skips to
    IDLE (if already homed) or proceeds through HOMING (if ODrives rebooted).
    """

    def __init__(self):
        # Sticky for the lifetime of this FAULT visit (recomputed on every on_enter).
        # True iff FAULT was entered PURELY because of a Teensy guard latch (from
        # ACTIVE) with NO ODrive-level error — see on_enter.
        self._guard_only = False

    def on_enter(self, ctx):
        # A GUARD-ONLY fault is observability + clear_errors routing ONLY: the Teensy
        # guard suppresses leg output WITHOUT disarming the ODrives, and trajectory_node
        # already installed a gate-validated profiled descent + froze target
        # advancement. So the streaming mode is ALREADY safe. Discriminate it from a
        # real fault: guard latched, and NONE of the ODrive-level fault signals set.
        # (A guard latch riding ALONGSIDE a real ODrive error is a real fault — the
        # ODrive error dominates and gets the full 'ERROR'/BOOT treatment below.)
        self._guard_only = (ctx.guard_latched and not ctx.errors
                            and not ctx.has_fatal_error and not ctx.boot_timed_out)

        # Entering FAULT is never a clean guard-clear resume — clear the one-shot so a
        # stale value can't skip the re-arm on a later real-fault→BOOT→ACTIVE path.
        ctx.resume_active_no_rearm = False

        if self._guard_only:
            # PRESERVE the streaming mode (ActiveHandler.on_exit blanked control_mode to
            # '' during the forced transition — restore it). Publishing 'ERROR' here — a
            # mode OUTSIDE trajectory_node's streaming set — would silence the 40 Hz
            # emitter, abandon the in-flight profiled descent, and re-latch MPC_STALE on
            # top of the guard: the self-sustaining 2026-07-10 deadlock this fix removes.
            # active_mode is untouched across the forced ACTIVE→FAULT transition, and
            # every ActiveMode value is inside trajectory_node's streaming set.
            ctx.control_mode = ctx.active_mode.value
        else:
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

        # A guard-only fault that GAINS a real ODrive error mid-fault is no longer
        # guard-only: promote to the real-fault path so a genuine fault can never be
        # masked by the preserved streaming mode (publish 'ERROR', route to BOOT on
        # clear — identical to a fault that started real).
        if self._guard_only and (ctx.errors or ctx.has_fatal_error
                                 or ctx.boot_timed_out):
            self._guard_only = False
            ctx.control_mode = 'ERROR'

        if self._guard_only:
            # Guard-only fault: the ODrives never disarmed and stayed homed, so exit
            # STRAIGHT BACK TO ACTIVE the moment the guard clears (re-running
            # BOOT/HOMING would be incoherent — the platform is still armed at pose).
            # clear_errors (routed below) clears the Teensy latch; the next /link_status
            # drops guard_latched and this fires. ActiveHandler resumes WITHOUT re-arming
            # (see resume_active_no_rearm) so the live stream is never disturbed.
            if not ctx.guard_latched:
                ctx.resume_active_no_rearm = True
                return RobotState.ACTIVE
        else:
            # Real fault: exit FAULT when all error conditions clear. BOOT re-validates
            # heartbeats, then skips to IDLE if already homed. A latched Teensy guard
            # riding on top holds FAULT too — leaving while it is still latched would
            # return to a silently-crippled robot; clear_errors clears the Teensy latch
            # and the next /link_status drops guard_latched so this exit fires.
            if (not ctx.has_fatal_error and not ctx.errors
                    and not ctx.boot_timed_out and not ctx.guard_latched):
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
