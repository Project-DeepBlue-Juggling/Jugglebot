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
    GUI = 'GUI'


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
        # True iff ALL six LEG axes report ODrive CLOSED_LOOP_CONTROL with
        # active_errors == 0 (derived by the orchestrator from
        # /robot_state.motor_states; fewer than six entries -> False). This is
        # "the legs are holding a pose and a profiled DEACTIVATE can lower them".
        # FaultHandler's real-fault exit stow (a visit that began in ACTIVE or
        # LEVELLING) runs only when it is true (ARMING_CONTRACT choreography
        # step 6).
        self.legs_closed_loop = False
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

        # ── Arming contract (A1-A5, see ARMING_CONTRACT.md) ──────
        # State the machine was in immediately before the current one. Written by
        # StateMachine on EVERY transition (tick-driven and forced). FaultHandler
        # uses it to classify guard-only faults: a guard latch can only be
        # guard-only if the fault came FROM ACTIVE (an armed concern) — a latch
        # carried into a HOMING/IDLE fault is a stale prior-session latch, and
        # exiting it "back" to ACTIVE would resume a robot that never activated
        # (the 2026-07-15 misroute).
        self.prev_state = None
        # A2 — orchestrator auto-arm. True (default): ActiveHandler arms the wire
        # after activation via the 'arm_setpoints' request. False (launch arg
        # auto_arm:=false → orchestrator param auto_arm_setpoint_output): the
        # operator arms manually via /set_setpoint_output (the pre-contract
        # probe-first bench flow); the disarmed wire stays loud via
        # trajectory_node (A5).
        self.auto_arm = True
        # A5 — wire state mirrored from /link_status 'mpc_active' (orchestrator
        # sets it). Used for observability AND to NARROW FaultHandler's guard-only
        # classification (guard-only = latched WHILE ARMED); never to authorize an
        # arm — the bridge's arm service is the single source of truth for arming,
        # and a stale/absent mirror fails toward the heavier real-fault path.
        self.wire_armed = False
        # A5 — True once at least one /link_status has been decoded. BOOT's
        # stale-latch pre-flight must not conclude "no latch" from the default
        # guard_latched=False before the bridge has reported at all.
        self.link_status_seen = False

        # Async operation result (set by orchestrator when a requested op completes)
        self.operation_pending = False
        self.operation_result = None         # None = not complete, True/False = result
        # The resolved operation's service message (the bridge's reject reason on
        # failure; None until a result lands). ActiveHandler classifies a failed
        # deactivate by it (retryable refusal vs a genuine one).
        self.operation_message = None
        # Operator-facing notices raised by handlers (e.g. "deactivate refused --
        # robot stays ACTIVE because ..."), appended via notify(). The
        # orchestrator logs each once (one WARN call site) and clears the list.
        self.operator_notices = []

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
        self.levelling_complete = False          # Persisted in the PLATFORM Teensy's RAM (RobotState.msg)
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

    def notify(self, text):
        self.operator_notices.append(text)

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
        ctx.prev_state = self._state
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
            ctx.prev_state = self._state
            self._state = next_state
            self._entered = False


# ══════════════════════════════════════════════════════════════════
# Built-in state handlers
# ══════════════════════════════════════════════════════════════════

class BootHandler(StateHandler):
    """Wait for heartbeats and firmware validation from all ODrive axes,
    then clear any stale prior-session guard latch (ARMING_CONTRACT A5)."""

    # How long to wait, after a successful clear_errors, for /link_status (10 Hz)
    # to reflect the released latch before declaring the clear ineffective.
    LATCH_CLEAR_TIMEOUT_S = 5.0

    def __init__(self):
        self._entry_time = 0.0
        self._latch_clear_requested = False
        self._latch_clear_sent_t = 0.0

    def on_enter(self, ctx):
        self._entry_time = time.time()
        self._latch_clear_requested = False
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

        # ── Stale-latch pre-flight (ARMING_CONTRACT A5) ──────────────
        # The can-bridge Teensy is Jetson-5V-powered, so a guard latch from a
        # prior session (e.g. a bench harness whose final disarm heartbeat was
        # lost) SURVIVES ROS relaunches — and the firmware refuses the guard-gated
        # verbs (HOME/ACTIVATE/DEACTIVATE) with ERR_BUS_DOWN while it is latched,
        # which wedged HOMING twice on 2026-07-15. At BOOT the bridge is disarmed
        # by construction (mpc_active pinned 0 on every startup path), no output
        # path exists and nothing streams, so clearing here cannot jolt anything.
        # Clear ONCE per BOOT visit; a latch that returns after a successful clear
        # is a live fault, not a stale one → FAULT.
        #
        # Wait for the first /link_status before concluding anything: the default
        # guard_latched=False must not be mistaken for "no latch" while the bridge
        # is still coming up (its firmware-check gate above usually guarantees
        # this, but the two signals ride different topics).
        if not ctx.link_status_seen:
            if time.time() - self._entry_time > BOOT_TIMEOUT_S:
                ctx.boot_timed_out = True
                return RobotState.FAULT
            return None

        if ctx.guard_latched:
            if not self._latch_clear_requested:
                self._latch_clear_requested = True
                self._latch_clear_sent_t = time.time()
                ctx.operation_result = None
                ctx.request = 'clear_errors'
                return None
            if ctx.operation_result is False:
                return RobotState.FAULT
            if (time.time() - self._latch_clear_sent_t
                    > self.LATCH_CLEAR_TIMEOUT_S):
                # Cleared (or clear pending) but the latch never dropped on
                # /link_status — treat as a live fault.
                return RobotState.FAULT
            return None  # clear in flight / waiting for /link_status to reflect

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

    def execute(self, ctx):
        # Wait for any pending operation (e.g., deactivation from ACTIVE)
        if ctx.operation_pending:
            return None

        # A4 (ARMING_CONTRACT) — the streaming mode stays published through the
        # descent because the machine is still ACTIVE until the deactivate is
        # confirmed (ActiveHandler._step_deactivate), so the 40 Hz emitter keeps
        # feeding the (disarmed, A3) wire to the end. A path that reaches IDLE
        # with an operation still in flight (a reconcile-to-IDLE after a lost
        # reply) is held above until it resolves; only then is the mode blanked.
        if ctx.control_mode != '':
            ctx.control_mode = ''

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


# Replies on which a failed DEACTIVATE is RETRIED -- only those that mean the
# firmware latched NOTHING: ERR_REJECTED (leg_deactivate.cpp deactivate_request:
# the wire still armed -- the disarm-staging race of 2026-10-19/23 -- a
# cold-start move or a deactivate already running, or a fault stow pending) and
# the orchestrator's own 'not ready' (nothing was sent). ERR_TIMEOUT is NOT
# here: a lost ack may mean the lower DID latch, so the bridge observes the legs
# to a verdict instead of reporting a refusal (owner, 2026-10-23); a bare
# ERR_TIMEOUT reaching this layer is therefore treated as genuine. Anything else
# -- ERR_BUS_DOWN (guard latched / bus down), ERR_BAD_ARGS, a deactivate that
# was ACCEPTED and then failed mid-descent ("deactivate FAILED -- axis ..."), no
# reply at all -- is genuine: no blind re-dispatch of a move whose outcome is
# uncertain.
_DEACTIVATE_RETRYABLE_MARKERS = ('ERR_REJECTED', 'not ready')


def deactivate_failure_is_retryable(message):
    """True iff a failed deactivate's service message is a refusal that moved
    nothing and may succeed on a later attempt (see the markers above)."""
    if not message:
        return False
    if 'deactivate FAILED' in message:
        return False   # accepted, then failed during the descent: not a refusal
    return any(m in message for m in _DEACTIVATE_RETRYABLE_MARKERS)


class ActiveHandler(StateHandler):
    """Robot activated — accepting pose commands via sub-mode.

    ARMING_CONTRACT A2: after the profiled ACTIVATE completes and the streaming
    mode is published, this handler ARMS the setpoint wire ('arm_setpoints' →
    the bridge's stream-then-arm pre-check) with bounded retries — the producer
    needs ~100-300 ms to seed its hold after the mode publish, so the first
    attempt(s) may legitimately be refused with "no setpoint frame". Persistent
    refusal is a real fault, loudly.
    """

    # Arm-phase retry budget. Each REFUSED attempt is one service round-trip
    # that includes the bridge's bounded 0.5 s :5557 frame-wait, dispatched on
    # the 10 Hz tick — so a refusal cycle is ~0.6-1.0 s wall-clock and the
    # budget covers roughly 6-10 s before FAULT (not 10× less; the tick rate is
    # not the bottleneck, the frame-wait is).
    ARM_MAX_ATTEMPTS = 10

    # Confirmed ACTIVE -> IDLE (2026-10-23, owner rule: a genuine refusal keeps
    # the machine ACTIVE, a transient one is retried, the reported state always
    # reflects the robot). The machine leaves ACTIVE only when the bridge
    # reports the deactivate DONE (legs stowed + IDLE). A refusal that latched
    # nothing (deactivate_failure_is_retryable) is retried up to
    # DEACTIVATE_MAX_ATTEMPTS, DEACTIVATE_RETRY_SPACING_S apart (> one 10 Hz J->T
    # heartbeat, the disarm-staging race). A genuine failure, a spent budget or
    # no reply by DEACTIVATE_REPLY_DEADLINE_S opens a RECONCILE window of
    # DEACTIVATE_RECONCILE_S: the machine follows the robot to IDLE if the legs
    # leave closed loop, else stays ACTIVE with the reason as an operator notice.
    DEACTIVATE_MAX_ATTEMPTS = 3
    DEACTIVATE_RETRY_SPACING_S = 1.0
    DEACTIVATE_RECONCILE_S = 1.0
    # The bridge's own budget for one deactivate is the DeactivateMonitor
    # timeout (teensy_link.deactivate.DEFAULT_TIMEOUT_S = 20 s) + its 5 s hard
    # deadline + up to 1 s wire-disarm wait; 30 s leaves ~4 s of margin so a
    # slow-but-alive bridge is never called silent. (A ~15 s deadline would
    # declare "no reply" while the bridge is still legitimately watching a
    # stalled descent.) Pinned against the constant by a test.
    DEACTIVATE_REPLY_DEADLINE_S = 30.0
    # ACTIVE-wide reconciliation: legs out of closed loop for this many
    # CONSECUTIVE 10 Hz ticks => the robot is not ACTIVE; follow it to IDLE.
    # /robot_state arrives at 100 Hz, so 5 ticks = 500 ms = ~50 consecutive
    # samples all saying "not CLOSED_LOOP" (each tick reads the latest one): a
    # single short/garbled message or one-sample state glitch cannot trip it,
    # while a real drop (legs IDLE) is followed within half a second. A leg
    # with an ODrive ERROR never gets here -- ctx.errors forces FAULT first.
    LEGS_OUT_TICKS = 5
    _STREAMING_MODE_CMDS = ('standby', 'trajectory', 'spacemouse', 'gui')

    def __init__(self):
        self._activated = False
        self._armed = False
        self._arm_attempts = 0
        self._reset_deactivate()
        self._legs_out_ticks = 0
        self._legs_seen_cl = False
        self._last_reason = None

    def _exit_disarm_only(self):
        """Mark a legs-out exit to IDLE: on_exit requests the DISARM + Hand IDLE
        only ('disarm_and_idle_hand'), never a leg DEACTIVATE — legs that fell
        out of closed loop must not be re-energised and lowered a second time
        (owner, 2026-10-23 second audit)."""
        self._deactivated = True
        self._exit_request = 'disarm_and_idle_hand'

    def _reset_deactivate(self):
        self._exit_request = None
        self._deactivating = False
        self._deact_phase = None        # 'waiting' | 'spacing' | 'reconcile'
        self._deactivate_attempts = 0
        self._deact_until = None
        self._deactivated = False

    def _begin_arm_phase(self, ctx):
        """Enter the arm phase (A2), or skip it under manual arming."""
        self._arm_attempts = 0
        ctx.operation_result = None
        if not ctx.auto_arm:
            # Manual-arm session (launch arg auto_arm:=false): the operator
            # arms via /set_setpoint_output after their pre-arm verification.
            # The disarmed wire stays loud via trajectory_node (A5).
            self._armed = True
            return
        self._armed = False
        ctx.request = 'arm_setpoints'

    def on_enter(self, ctx):
        # Always reset to STANDBY on entry so re-activation never inherits a
        # prior sub-mode.  The orchestrator's control_mode is a target-routing
        # signal: STANDBY SILENCES move commands — trajectory_node keeps streaming
        # its current plan and holds at the pose it seeded or last reached (NOT a
        # neutral pose; return-to-neutral is the explicit trajectory/go_home
        # service). The operator then switches to TRAJECTORY / SPACEMOUSE / GUI
        # to accept the corresponding command source.
        ctx.active_mode = ActiveMode.STANDBY
        self._reset_deactivate()
        self._legs_out_ticks = 0
        self._legs_seen_cl = False

        # FIX 2 (F1) — guard-clear resume. A GUARD-ONLY fault (Teensy guard latched
        # during ACTIVE with NO ODrive error) never disarmed the legs and never left
        # the streaming set (FaultHandler preserved control_mode), and trajectory_node
        # held a profiled descent throughout. So on the way back from that fault we
        # must RESUME already-armed: re-running the ACTIVATE move here (a TRAP_TRAJ to
        # the active pose) would fight the live 40 Hz PASSTHROUGH stream and lurch the
        # legs up from the post-descent measured hold — potentially re-tripping the
        # guard. Consume the one-shot and go straight to the armed STANDBY hold —
        # re-VERIFYING the arm through the same phase (idempotent: the bridge
        # returns 'already enabled' if the wire stayed armed, and re-runs the full
        # stream-then-arm pre-check if something disarmed it mid-fault).
        resume = ctx.resume_active_no_rearm
        ctx.resume_active_no_rearm = False
        if resume:
            self._activated = True
            ctx.control_mode = ctx.active_mode.value  # 'STANDBY'
            self._begin_arm_phase(ctx)
            return

        self._activated = False
        self._armed = False
        self._arm_attempts = 0
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
            # A2 — the streaming mode is published this tick (the orchestrator
            # publishes control_mode AFTER request dispatch, so the producer
            # sees the mode before/with the first arm attempt's refusal window).
            self._begin_arm_phase(ctx)
            return None

        # Arm phase (A2): retry until armed or the budget is spent.
        if not self._armed:
            if ctx.operation_result is None:
                return None
            if ctx.operation_result is False:
                self._arm_attempts += 1
                if self._arm_attempts >= self.ARM_MAX_ATTEMPTS:
                    # Persistent arm refusal — a real fault (producer never
                    # seeded, guard latched, or the link is sick). FAULT, loudly.
                    return RobotState.FAULT
                ctx.operation_result = None
                ctx.request = 'arm_setpoints'
                return None
            self._armed = True

        if self._deactivating:
            return self._step_deactivate(ctx)

        # ACTIVE-wide reconciliation (armed, past the arm phase): the robot,
        # not the last command, decides the state.
        if self._legs_out(ctx):
            ctx.notify(
                f'legs out of closed loop for {self.LEGS_OUT_TICKS} consecutive '
                f'ticks while ACTIVE -- the robot is not active; following it '
                f'to IDLE')
            self._exit_disarm_only()
            return RobotState.IDLE

        cmd = ctx.consume_command()
        if cmd == 'deactivate':
            # Stay ACTIVE until the robot confirms it (see DEACTIVATE_*). The
            # streaming mode stays published through the descent (A4) because
            # the machine is still ACTIVE; the bridge disarms in-process first (A3).
            self._reset_deactivate()
            self._deactivating = True
            self._request_deactivate(ctx)
            return None
        elif cmd in self._STREAMING_MODE_CMDS:
            # Owner policy 2026-10-23: after a refused deactivate the robot is
            # ACTIVE but DISARMED. A streaming mode switch onto a disarmed wire
            # is refused -- the operator re-arms explicitly (no auto re-arm).
            # Auto-arm sessions only: a manual-arm session (auto_arm False)
            # switches mode BEFORE its operator arm (runbook Sharp Edge #5).
            if ctx.auto_arm and ctx.link_status_seen and not ctx.wire_armed:
                ctx.notify(
                    f"'{cmd}' refused: setpoint output is DISARMED (wire not "
                    f"armed) -- re-arm via /set_setpoint_output first")
                return None
            ctx.active_mode = ActiveMode(cmd.upper())
            ctx.control_mode = ctx.active_mode.value
        # Other commands silently discarded (already logged on receipt)

        return None

    def _legs_out(self, ctx):
        """True once the legs have been reported OUT of closed loop for
        LEGS_OUT_TICKS consecutive ticks, AFTER having been seen closed-loop in
        this ACTIVE visit (no evidence of the robot reporting = no claim)."""
        if ctx.legs_closed_loop:
            self._legs_seen_cl = True
            self._legs_out_ticks = 0
            return False
        if not self._legs_seen_cl:
            return False
        self._legs_out_ticks += 1
        return self._legs_out_ticks >= self.LEGS_OUT_TICKS

    def _request_deactivate(self, ctx):
        self._deactivate_attempts += 1
        self._deact_phase = 'waiting'
        self._deact_until = time.monotonic() + self.DEACTIVATE_REPLY_DEADLINE_S
        ctx.operation_result = None
        ctx.operation_message = None
        ctx.request = 'deactivate'

    def _filter_commands_while_deactivating(self, ctx):
        """Discard ONLY streaming mode switches (one notice each) and absorb a
        duplicate 'deactivate' (already being served); every other command is
        left queued, in order, for the state that follows."""
        keep = []
        while True:
            cmd = ctx.consume_command()
            if cmd is None:
                break
            if cmd in self._STREAMING_MODE_CMDS:
                ctx.notify(f"'{cmd}' discarded: a deactivate is in progress "
                           f"(mode stays {ctx.active_mode.value})")
            elif cmd != 'deactivate':
                keep.append(cmd)
        for cmd in keep:
            ctx.enqueue_command(cmd)

    def _step_deactivate(self, ctx):
        """One tick of the confirmed deactivate. Returns IDLE or None."""
        self._filter_commands_while_deactivating(ctx)
        now = time.monotonic()
        phase = self._deact_phase
        if phase == 'spacing':
            if now >= self._deact_until:
                self._request_deactivate(ctx)
            return None
        if phase == 'reconcile':
            legs_out = self._legs_out(ctx)
            if legs_out or (now >= self._deact_until and self._legs_seen_cl
                            and not ctx.legs_closed_loop):
                self._reset_deactivate()
                self._exit_disarm_only()
                ctx.notify(
                    f'deactivate reported failure ({self._last_reason}) but the '
                    f'legs are no longer closed-loop -- following the robot '
                    f'to IDLE')
                return RobotState.IDLE
            if now >= self._deact_until:
                attempts = self._deactivate_attempts
                self._reset_deactivate()
                ctx.notify(
                    f'deactivate REFUSED after {attempts} attempt(s): '
                    f'{self._last_reason} -- robot stays ACTIVE (legs '
                    f'closed-loop, setpoint output disarmed); resolve the cause, '
                    f're-arm or request IDLE again')
            return None
        # phase == 'waiting'
        if ctx.operation_pending or ctx.operation_result is None:
            if now < self._deact_until:
                return None
            # No reply: stop treating the call as in flight, or a reconcile-to-
            # IDLE would wedge IdleHandler behind operation_pending. A late reply
            # cannot be misread as a LATER deactivate's answer: the orchestrator
            # keeps one future, and the next dispatch replaces it (the old one
            # is never read); a late reply while no deactivate is pending is
            # ignored by ActiveHandler (it reads operation_result only in its
            # arm / deactivate phases, and both clear it before dispatching).
            ctx.operation_pending = False
            return self._begin_reconcile(
                ctx, f'no reply from the bridge within '
                     f'{self.DEACTIVATE_REPLY_DEADLINE_S:.0f} s')
        if ctx.operation_result is True:
            self._reset_deactivate()
            self._deactivated = True
            return RobotState.IDLE
        reason = ctx.operation_message or 'no reason given'
        ctx.operation_result = None
        if (deactivate_failure_is_retryable(reason)
                and self._deactivate_attempts < self.DEACTIVATE_MAX_ATTEMPTS):
            ctx.notify(
                f'deactivate attempt {self._deactivate_attempts}/'
                f'{self.DEACTIVATE_MAX_ATTEMPTS} refused ({reason}) -- robot '
                f'still ACTIVE; retrying in {self.DEACTIVATE_RETRY_SPACING_S:.1f} s')
            self._deact_phase = 'spacing'
            self._deact_until = now + self.DEACTIVATE_RETRY_SPACING_S
            return None
        return self._begin_reconcile(ctx, reason)

    def _begin_reconcile(self, ctx, reason):
        """Genuine failure / spent budget / no reply: watch the robot for
        DEACTIVATE_RECONCILE_S before deciding ACTIVE vs IDLE."""
        self._last_reason = reason
        self._deact_phase = 'reconcile'
        self._deact_until = time.monotonic() + self.DEACTIVATE_RECONCILE_S
        self._legs_out_ticks = 0
        return None

    def on_exit(self, ctx):
        # A3/A4 (ARMING_CONTRACT): on an exit that did NOT come through the
        # confirmed deactivate (a handler-returned ACTIVE -> FAULT), request the
        # deactivate but DO NOT blank the control mode — the bridge disarms
        # in-process at the head of _run_deactivate (A3), and the streaming mode
        # must stay published until that operation resolves so the armed wire is
        # never starved by a mode blank (A4). After a confirmed deactivate the
        # legs are already stowed: a second request would be redundant.
        if self._deactivated:
            if self._exit_request:
                ctx.request = self._exit_request   # legs-out exit: disarm only
            return
        ctx.request = 'deactivate'


# States that hold the six legs at an active pose in CLOSED_LOOP: ACTIVE, and
# LEVELLING (level_activate runs the same profiled ACTIVATE and the pose is held
# through settle / tilt / correction / persist / mocap-check until
# level_deactivate). A real fault whose FAULT visit began in one of them stows
# the legs on exit (FaultHandler._real_fault_exit, ARMING_CONTRACT choreography
# step 6). NOT the guard-only classification, which stays ACTIVE + armed wire.
_LEGS_POSED_STATES = (RobotState.ACTIVE, RobotState.LEVELLING)


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

    Real-fault exit stow (ARMING_CONTRACT choreography step 6, 2026-10-06;
    LEVELLING added 2026-10-07): a real fault whose visit began in ACTIVE or
    LEVELLING, once everything has cleared, first requests the profiled
    DEACTIVATE if all six legs are closed-loop and the robot is homed, and
    holds FAULT until it succeeds. Without it a forced ACTIVE->FAULT (whose
    ActiveHandler.on_exit deactivate the orchestrator cancels), or any fault
    during LEVELLING (whose level_activate holds the same active pose until
    level_deactivate, and whose on_exit only clears requests), reached IDLE
    with the legs still holding an active pose.
    """

    def __init__(self):
        # Sticky for the lifetime of this FAULT visit (recomputed on every on_enter).
        # True iff FAULT was entered PURELY because of a Teensy guard latch (from
        # ACTIVE) with NO ODrive-level error — see on_enter.
        self._guard_only = False
        # Real-fault exit stow bookkeeping, all reset on every on_enter.
        # _legs_posed_at_entry: this FAULT visit began in a state that holds
        # the legs at an active pose in CLOSED_LOOP (ACTIVE, or LEVELLING
        # between level_activate and level_deactivate). Sticky, so a guard-only
        # fault promoted to real mid-FAULT (promotion does not re-enter) still
        # stows. The guard-only classification below stays ACTIVE-only.
        self._legs_posed_at_entry = False
        self._stow_requested = False   # our 'deactivate' is in flight
        self._stow_done = False        # it succeeded: the legs are stowed
        self._stow_failed = False      # it failed: hold FAULT until clear_errors

    def on_enter(self, ctx):
        self._legs_posed_at_entry = ctx.prev_state in _LEGS_POSED_STATES
        self._stow_requested = False
        self._stow_done = False
        self._stow_failed = False

        # A GUARD-ONLY fault is observability + clear_errors routing ONLY: the Teensy
        # guard suppresses leg output WITHOUT disarming the ODrives, and trajectory_node
        # already installed a gate-validated profiled descent + froze target
        # advancement. So the streaming mode is ALREADY safe. Discriminate it from a
        # real fault: guard latched, NONE of the ODrive-level fault signals set, AND
        # the fault came FROM ACTIVE (ARMING_CONTRACT: the guard's staleness/deviation
        # terms only matter while armed, an ACTIVE concern). Without the origin check,
        # a stale prior-session latch surfacing during HOMING/BOOT classified as
        # guard-only and — on clear — exited "back" to ACTIVE on a robot that never
        # activated (the 2026-07-15 misroute; the comments said "from ACTIVE" but the
        # code never enforced it).
        # (A guard latch riding ALONGSIDE a real ODrive error is a real fault — the
        # ODrive error dominates and gets the full 'ERROR'/BOOT treatment below.)
        # ctx.wire_armed narrows further: guard-only semantics are literally "the
        # guard latched while the wire was armed" — prev_state alone misclassifies
        # a latch-caused ACTIVATE refusal (FAULT raised BY ActiveHandler on an
        # unarmed robot, e.g. an overspeed latch surfacing at IDLE) as guard-only
        # and would "resume" a robot that never activated. Fail-safe direction: a
        # stale/absent wire_armed routes to the heavier-but-safe real-fault path.
        self._guard_only = (ctx.guard_latched and not ctx.errors
                            and not ctx.has_fatal_error and not ctx.boot_timed_out
                            and ctx.prev_state == RobotState.ACTIVE
                            and ctx.wire_armed)

        # A2 (ARMING_CONTRACT) — a REAL fault disarms the wire. The output is
        # (or is about to be) unusable and control_mode goes 'ERROR' (stopping the
        # producer), so an armed wire would only race the staleness watchdog.
        # Guard-only faults deliberately stay armed: the resume path depends on
        # the stream surviving, and the guard is already suppressing output.
        if not self._guard_only:
            ctx.request = 'disarm_setpoints'

        # Entering FAULT is never a clean guard-clear resume — clear the one-shot so a
        # stale value can't skip the re-arm on a later real-fault→BOOT→ACTIVE path.
        ctx.resume_active_no_rearm = False

        if self._guard_only:
            # PRESERVE the streaming mode (control_mode still holds it — A4 removed
            # ActiveHandler.on_exit's blank; re-assert from active_mode so a forced
            # transition from any prior value is safe). Publishing 'ERROR' here — a
            # mode OUTSIDE trajectory_node's streaming set — would silence the 40 Hz
            # emitter, abandon the in-flight profiled descent, and re-latch SETPOINT_STALE on
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
            # A2 — promotion to a real fault is a real-fault entry in every way
            # that matters: 'ERROR' stops the producer, so an armed wire would
            # only race the staleness watchdog (on_enter cannot re-run to issue
            # this — review 2026-07-15).
            ctx.request = 'disarm_setpoints'

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
            # and the next /link_status drops guard_latched so this exit fires. (The
            # bridge also refuses DEACTIVATE with ERR_BUS_DOWN while latched, so the
            # exit stow below must wait for the same condition.)
            self._resolve_stow(ctx)
            if (not ctx.has_fatal_error and not ctx.errors
                    and not ctx.boot_timed_out and not ctx.guard_latched):
                nxt = self._real_fault_exit(ctx)
                if nxt is not None:
                    return nxt
            if self._stow_requested:
                # The stow is in flight in the orchestrator's single tracked
                # operation slot. Leave operator commands QUEUED: a clear_errors
                # dispatched now would take that slot, orphan the deactivate,
                # and its result would be read as the stow's (an IDLE claimed
                # while the platform is still descending). They are consumed on
                # the first tick after the stow resolves.
                return None

        cmd = ctx.consume_command()
        if cmd == 'clear_errors':
            ctx.request = 'clear_errors'
            ctx.boot_timed_out = False
            # One stow retry per clear_errors: a failed stow re-arms here and
            # nowhere else, so a refusing bridge cannot make the exit spin.
            self._stow_failed = False
        # Other commands silently discarded (already logged on receipt)

        return None

    def _resolve_stow(self, ctx):
        """Read the outcome of our exit stow once the operation resolves.

        Runs every real-fault tick, whatever the exit condition, so a stow that
        resolves while a NEW error is present is still accounted for.
        """
        if not self._stow_requested:
            return
        if ctx.operation_pending or ctx.operation_result is None:
            return
        self._stow_requested = False
        if ctx.operation_result is True:
            self._stow_done = True
        else:
            # The robot is NOT stowed: BOOT -> IDLE would lie about it. Hold
            # FAULT; the operator's clear_errors re-runs the exit and retries.
            self._stow_failed = True

    def _real_fault_exit(self, ctx):
        """The real-fault exit, given every error and the guard latch have cleared.

        Returns BOOT, or None to hold FAULT. A FAULT visit that began in ACTIVE
        or LEVELLING with all six legs closed-loop and the robot homed means the
        legs are still holding an active pose: request the profiled DEACTIVATE
        first and hold FAULT until it succeeds, so the machine never reaches
        IDLE ("platform stowed, legs idle") while they hold it (ARMING_CONTRACT
        choreography step 6). Otherwise BOOT at once, as before: a leg that
        dropped out of closed loop (e.g. an ODrive reboot) is BOOT -> HOMING's
        job, and a fault from BOOT/HOMING/IDLE has no active pose.
        """
        if self._stow_done:
            return RobotState.BOOT
        if self._stow_requested or self._stow_failed:
            return None
        if not (self._legs_posed_at_entry and ctx.is_homed
                and ctx.legs_closed_loop):
            return RobotState.BOOT
        if ctx.operation_pending:
            # A tracked operation (an operator clear_errors, or the
            # ActiveHandler.on_exit deactivate on a handler-returned
            # ACTIVE->FAULT) is still in flight. Requesting now would orphan it
            # in the orchestrator's single slot; wait for it to resolve. (A
            # handler-returned LEVELLING->FAULT leaves nothing in flight: it
            # fires only on a RESOLVED level_* result, and on_exit clears any
            # queued request.)
            return None
        ctx.operation_result = None
        ctx.request = 'deactivate'
        self._stow_requested = True
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
