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


# ── Homing gate (ARMING_CONTRACT A6) ──────────────────────────────
# Homing drives the Legs one at a time to their bottom stops. Started under a
# Platform held up at the Active pose (or a levelling pose), the Legs not yet
# homed keep their lengths while the one being homed retracts, and the Platform
# can collapse (owner, 2026-11-25). The machine therefore enters HOMING only when
# all six Legs report ODrive IDLE, continuously for HOMING_LEGS_IDLE_SETTLE_S:
# six Idle (unpowered, free) Legs cannot hold the Platform up, and the settle
# lets a Platform that has just been dropped come to rest.

# ODrive AxisState values (protocol_config.ODRIVE_STATES via odrive.AXIS_STATES;
# pinned equal by tests/ros/test_state_machine.py so this module stays free of
# jugglebot imports at load).
ODRIVE_AXIS_IDLE = 1
ODRIVE_AXIS_CLOSED_LOOP = 8
NUM_LEGS = 6
# How long all six Legs must have read IDLE before Homing may start. A gravity
# drop from the Active pose ends well inside it; the cost is at most 2 s added
# to a Homing that follows a stow or a power cycle.
HOMING_LEGS_IDLE_SETTLE_S = 2.0
# Prefix of the one quiet (non-refusal) answer: the Legs are Idle but not yet
# for the settle time. It resolves by itself, so it is logged, not notified.
HOMING_SETTLING_PREFIX = 'Homing waits'
# How old the /robot_state that set the Leg states may be before they count as
# unknown (A6 freshness). The bridge republishes /robot_state at the 100 Hz timer
# rate while telemetry flows, and when the telemetry generation freezes it skips
# at most _ROBOT_STATE_MAX_CONSEC_SKIPS = 50 timer ticks (~0.5 s) before it
# publishes anyway (teensy_bridge_node._publish_robot_state; the link-lost case
# publishes at the full rate), so a live bridge never leaves a gap near 1 s; the
# worst gap on record is a 20 s recovery park that used to block the publisher
# (fixed 2026-09-23, now its own callback group). 1 s is 2x the bridge bound.
LEG_STATES_MAX_AGE_S = 1.0
# Prefix of the stale refusal. Its text carries the age, so the once-per-episode
# report keys on the prefix, not the whole string.
HOMING_STALE_PREFIX = 'Homing refused: the six Leg drive states are not known'


def _legs_list(legs):
    return ', '.join(str(i) for i in legs)


# ── The Leg IDLE rule (owner, 2026-11-25; ARMING_CONTRACT A7) ─────
# "A Leg should never be made IDLE unless the Platform is already at the Stow
# position. Once the Platform is at Stow, then any/all Legs can freely enter
# IDLE." The orchestrator's IDLE state therefore means "Platform at Stow, Legs
# Idle", and BOOT enters it only when all six Legs read Idle (A7). The bridge's
# operator services that IDLE or move a Leg (reboot, encoder search, a manual
# Home) refuse unless every other present Leg reads Idle (A7). Both ask the one
# classifier below; the Hand is exempt (it is never in leg_states).

def leg_idle_blockers(leg_states, exempt=()):
    """The Legs that stop the Leg IDLE rule from passing: ``[(leg, state)]`` for
    every Leg in ``leg_states`` that is not ODrive IDLE, in Leg order, skipping
    the Legs in ``exempt``. ``leg_states`` maps a Leg index to its ODrive axis
    state, or to None when that Leg's state is not known (stale, or never
    reported): an unknown Leg blocks, because it cannot be shown to be down. A
    sequence is read as Legs 0..n-1. A Leg absent from the mapping is not
    present on this rig and is not consulted (the caller decides presence: the
    orchestrator requires all six, the bridge skips a Leg its drive never
    heartbeated). Empty list: the rule passes."""
    items = (leg_states.items() if hasattr(leg_states, 'items')
             else enumerate(leg_states))
    return [(i, s) for i, s in sorted(items)
            if i not in exempt and s != ODRIVE_AXIS_IDLE]


# The wording that differs between the two orchestrator gates built on the one
# predicate below (A6 Homing entry, A7 BOOT's IDLE entry).
_HOMING_WORDS = {
    'refused': 'Homing refused',
    'risk': 'Homing could collapse it',
    'starts': 'Homing starts by itself from BOOT once every Leg is Idle',
    'starts_mixed': 'Homing starts by itself',
    'contract': 'ARMING_CONTRACT A6',
    'settle_s': HOMING_LEGS_IDLE_SETTLE_S,
}
_IDLE_WORDS = {
    'refused': 'IDLE refused',
    'risk': 'IDLE would claim it is at Stow',
    'starts': 'BOOT enters IDLE by itself once every Leg is Idle',
    'starts_mixed': 'BOOT enters IDLE by itself',
    'contract': 'ARMING_CONTRACT A7',
    'settle_s': 0.0,
}
# Prefix of the BOOT IDLE stale refusal (once-per-episode key, as A6's).
IDLE_STALE_PREFIX = 'IDLE refused: the six Leg drive states are not known'


def _legs_down_refusal(leg_states, idle_for_s, states_age_s, w):
    """The one "Platform shown to be down" predicate behind A6 and A7: None if
    the six Legs are known, fresh and Idle (for ``w['settle_s']`` when it is
    non-zero), else the operator message in ``w``'s wording."""
    refused, contract = w['refused'], w['contract']
    if leg_states is None or len(leg_states) != NUM_LEGS:
        return (f'{refused}: the six Leg drive states are not known, so the '
                f'Platform cannot be shown to be down ({contract}). '
                f'Waiting for a full /robot_state.')
    if states_age_s is None or states_age_s > LEG_STATES_MAX_AGE_S:
        age = ('no /robot_state has set them' if states_age_s is None else
               f'the last /robot_state that carried them is {states_age_s:.1f} s '
               f'old, stale beyond {LEG_STATES_MAX_AGE_S:.0f} s')
        settle = ('; the Idle settle restarts when it resumes'
                  if w['settle_s'] > 0 else '')
        return (f'{refused}: the six Leg drive states are not known ({age}), '
                f'so the Platform cannot be shown to be down ({contract}). '
                f'Waiting for a fresh /robot_state{settle}.')
    blockers = leg_idle_blockers(leg_states)
    closed = [i for i, s in blockers if s == ODRIVE_AXIS_CLOSED_LOOP]
    other = [(i, s) for i, s in blockers if s != ODRIVE_AXIS_CLOSED_LOOP]
    if len(closed) == NUM_LEGS:
        return (f'{refused}: all six Legs are Closed-loop, so the Platform '
                f'may be held up at a pose and {w["risk"]} '
                f'({contract}). Stow it first with the profiled '
                f'deactivate (ros2 service call /deactivate '
                f'std_srvs/srv/Trigger); {w["starts"]}.')
    if closed:
        not_closed = [i for i in range(NUM_LEGS) if i not in closed]
        return (f'{refused}: Leg(s) {_legs_list(closed)} are Closed-loop '
                f'but Leg(s) {_legs_list(not_closed)} are not, so the Platform '
                f'may be held up by some Legs; {w["risk"]}, and a '
                f'profiled stow must not drive a Leg that is not Closed-loop '
                f'({contract}). Support the Platform and press the '
                f'E-stop button to bring it down, restore power, then send '
                f'clear_errors; {w["starts_mixed"]} once all six Legs have '
                f'been Idle for {HOMING_LEGS_IDLE_SETTLE_S:.0f} s.'
                if w['settle_s'] > 0 else
                f'{refused}: Leg(s) {_legs_list(closed)} are Closed-loop '
                f'but Leg(s) {_legs_list(not_closed)} are not, so the Platform '
                f'may be held up by some Legs; {w["risk"]}, and a '
                f'profiled stow must not drive a Leg that is not Closed-loop '
                f'({contract}). Support the Platform and press the '
                f'E-stop button to bring it down (the present fallback until '
                f'FW 53), restore power, then send clear_errors; '
                f'{w["starts_mixed"]} once all six Legs are Idle.')
    if other:
        named = ', '.join(f'Leg {i} state {s}' for i, s in other)
        return (f'{refused}: {named} (neither Idle nor Closed-loop), so '
                f'the Platform cannot be shown to be down ({contract}). '
                f'Wait for the drive to settle Idle, or power-cycle the drives '
                f'with the E-stop button, then send clear_errors.')
    if w['settle_s'] > 0 and (idle_for_s is None or idle_for_s < w['settle_s']):
        return (f'{HOMING_SETTLING_PREFIX}: all six Legs are Idle, but not yet '
                f'for {HOMING_LEGS_IDLE_SETTLE_S:.0f} s (a dropped Platform '
                f'comes to rest first, ARMING_CONTRACT A6).')
    return None


def homing_refusal(leg_states, idle_for_s, states_age_s):
    """The A6 predicate: None if Homing may start, else the operator message.

    ``leg_states``: the six Leg ODrive axis states (axes 0-5) from the latest
    /robot_state, or None when not known. ``idle_for_s``: how long all six have
    read IDLE without a break, or None when they do not all read IDLE now.
    ``states_age_s``: age of the /robot_state that set ``leg_states``, or None
    when none has (older than LEG_STATES_MAX_AGE_S, the states are unknown: a
    topic that stopped cannot show a Leg that was re-energised since).
    Every path into HOMING passes through this (StateMachine guard).
    """
    return _legs_down_refusal(leg_states, idle_for_s, states_age_s,
                              _HOMING_WORDS)


def idle_refusal(leg_states, states_age_s):
    """The A7 predicate: None if the orchestrator may claim IDLE ("Platform at
    Stow, Legs Idle"), else the operator message. The A6 inputs and freshness,
    with no settle: all six Legs known, fresh and Idle. BOOT's IDLE entry and
    ACTIVE's legs-out exit consult it."""
    return _legs_down_refusal(leg_states, None, states_age_s, _IDLE_WORDS)


def ctx_idle_refusal(ctx):
    """idle_refusal on a Context's A6 inputs, aged now."""
    age = (None if ctx.leg_states_time is None
           else time.monotonic() - ctx.leg_states_time)
    return idle_refusal(ctx.leg_states, age)


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
        # FaultHandler's real-fault exit stow runs exactly when it is true,
        # whatever state the visit began in (ARMING_CONTRACT choreography
        # step 6; FW 53).
        self.legs_closed_loop = False
        # ARMING_CONTRACT A6 inputs (derived by the orchestrator from
        # /robot_state.motor_states): the six Leg ODrive axis states (None until
        # a message carries every Leg), and the time.monotonic() at which all six
        # were first seen IDLE in the current unbroken run (None while any Leg is
        # not IDLE). Read only by homing_refusal through the StateMachine guard.
        # leg_states_time: time.monotonic() of the /robot_state that set
        # leg_states (None while they are unknown); A6 freshness.
        self.leg_states = None
        self.legs_idle_since = None
        self.leg_states_time = None
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

        # ── Levelled ACTIVATE (BUG-130, C-LEVEL-1.A in levelling_frame.md) ──
        # Set by the orchestrator from trajectory/status (C-LEVEL-1.O: the
        # applier's own answer). Never from levelling_complete.
        self.gravity_correction_loaded = False   # last trajectory/status value
        self.traj_status_age_s = None            # age of that status (s); None = never received
        # The six Levelled Active pose Leg targets (rev) IdleHandler fetched
        # from trajectory/levelled_activate_targets. Consumed (reset to None)
        # by the orchestrator's 'activate' dispatch; never by 'level_activate'.
        self.activate_leg_targets_rev = None

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
        # A6: the refusal last reported, so a BOOT retrying every tick reports
        # once per episode; cleared on any tick that does not try for HOMING.
        self._homing_refusal_said = None

    def register(self, state, handler):
        self._handlers[state] = handler

    @property
    def state(self):
        return self._state

    def _homing_entry_refused(self, ctx):
        """ARMING_CONTRACT A6, the one enforcement point: True (and report it
        once) if a transition into HOMING must not happen now."""
        idle_for = (None if ctx.legs_idle_since is None
                    else time.monotonic() - ctx.legs_idle_since)
        age = (None if ctx.leg_states_time is None
               else time.monotonic() - ctx.leg_states_time)
        refusal = homing_refusal(ctx.leg_states, idle_for, age)
        if refusal is None:
            self._homing_refusal_said = None
            return False
        key = (HOMING_STALE_PREFIX if refusal.startswith(HOMING_STALE_PREFIX)
               else refusal)
        if key != self._homing_refusal_said:
            self._homing_refusal_said = key
            # No ' -> ' in this line: the orchestrator's log sink reads that as
            # a transition. It lands at DEBUG; the notice below is the WARN.
            self._log(f'HOMING held in {self._state.name}: {refusal}')
            if not refusal.startswith(HOMING_SETTLING_PREFIX):
                ctx.notify(refusal)
        return True

    def force_transition(self, new_state, ctx):
        """Force immediate transition, calling on_exit for current state."""
        if new_state == self._state:
            return
        if new_state == RobotState.HOMING and self._homing_entry_refused(ctx):
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
        if next_state == RobotState.HOMING and next_state != self._state:
            if self._homing_entry_refused(ctx):
                next_state = None   # stay (BOOT / IDLE); no on_exit has run
        else:
            self._homing_refusal_said = None
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
        # A7: the IDLE refusal last notified, so a BOOT holding every tick
        # reports once per episode (reset on entry and when it passes).
        self._idle_refusal_said = None

    def on_enter(self, ctx):
        self._entry_time = time.time()
        self._latch_clear_requested = False
        self._idle_refusal_said = None
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

        # Skip ahead if already homed (e.g., recovery from transient FAULT),
        # but only onto a Platform shown to be at Stow (ARMING_CONTRACT A7):
        # IDLE means "Platform at Stow, Legs Idle", and a relaunch under a
        # posed Platform, or one Leg dropped with five holding and the homed
        # flag kept, must not claim it. Hold BOOT and say once how to lower it.
        if ctx.is_homed:
            refusal = ctx_idle_refusal(ctx)
            if refusal is None:
                self._idle_refusal_said = None
                return RobotState.IDLE
            key = (IDLE_STALE_PREFIX if refusal.startswith(IDLE_STALE_PREFIX)
                   else refusal)
            if key != self._idle_refusal_said:
                self._idle_refusal_said = key
                ctx.notify(refusal)
            return None
        self._idle_refusal_said = None
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


# trajectory/status is published at 5 Hz; older than this, its
# gravity_correction_loaded is the answer of a node that may be dead
# (C-LEVEL-1.O "Trust a cached True"). Same value as skill_node's
# _TRAJ_STATUS_STALE_S.
TRAJ_STATUS_STALE_S = 1.0

# The six Legs of the Levelled Active pose (one target each).
_NUM_LEGS = 6


def activate_levelling_refusal(ctx):
    """THE decision on whether an operator ``activate`` may proceed to fetch the
    Levelled Active pose (BUG-130, C-LEVEL-1.A; owner ruling 2026-11-24).

    Returns None to proceed, else the refusal text (without the remedy, which
    IdleHandler appends). Refuses when no levelling correction is loaded by
    C-LEVEL-1.O's test: ``trajectory/status.gravity_correction_loaded`` false,
    that status older than ``TRAJ_STATUS_STALE_S``, or never received. Does NOT
    require a Tilt map, and never reads ``ctx.levelling_complete`` (a Platform
    Teensy RAM flag that survives a trajectory_node restart).
    Change the activate policy here and only here.
    """
    age = ctx.traj_status_age_s
    if age is None:
        return 'no trajectory/status received (is trajectory_node running?)'
    if not (age < TRAJ_STATUS_STALE_S):
        return (f'trajectory/status is stale ({age:.1f} s old, limit '
                f'{TRAJ_STATUS_STALE_S:.1f} s; is trajectory_node running?)')
    if not ctx.gravity_correction_loaded:
        return ('no levelling correction is loaded '
                '(trajectory/status gravity_correction_loaded is false)')
    return None


class IdleHandler(StateHandler):
    """Robot ready — waiting for activation command.

    ``activate`` (BUG-130, C-LEVEL-1.A) is decided HERE, in IDLE, before anything
    moves: ``activate_levelling_refusal`` first, then a fetch of the six Levelled
    Active pose Leg targets ('fetch_activate_targets'). Only a fetched target
    set leaves IDLE for ACTIVE; any refusal stays IDLE with an operator WARN that
    names the remedy (``level``) -- never ACTIVE then FAULT, since nothing moved.
    """

    REMEDY = ("run 'level' (or restore the saved correction), then 'activate'; "
              "the robot stays IDLE, nothing moved")

    def __init__(self):
        self._fetching = False

    def on_enter(self, ctx):
        self._fetching = False
        ctx.activate_leg_targets_rev = None

    def _refuse(self, ctx, reason):
        ctx.activate_leg_targets_rev = None
        ctx.notify(f'activate refused: {reason} -- {self.REMEDY}')
        return None

    def _begin_activate(self, ctx):
        reason = activate_levelling_refusal(ctx)
        if reason is not None:
            return self._refuse(ctx, reason)
        ctx.activate_leg_targets_rev = None
        ctx.operation_result = None
        ctx.operation_message = None
        ctx.request = 'fetch_activate_targets'
        self._fetching = True
        return None

    def _resolve_fetch(self, ctx):
        """The fetch has resolved (no operation pending). A None result with
        nothing pending is a lost fetch (cancelled): refused like a failure."""
        self._fetching = False
        ok = ctx.operation_result is True
        ctx.operation_result = None
        targets = ctx.activate_leg_targets_rev
        if ok and targets is not None and len(targets) == _NUM_LEGS:
            return RobotState.ACTIVE
        detail = ctx.operation_message or 'no Leg targets returned'
        return self._refuse(ctx, f'no Levelled Active pose: {detail}')

    def execute(self, ctx):
        # Wait for any pending operation (e.g., deactivation from ACTIVE, or
        # the activate's own target fetch)
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

        if self._fetching:
            return self._resolve_fetch(ctx)

        cmd = ctx.consume_command()
        if cmd == 'activate':
            return self._begin_activate(ctx)
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
            # The arm reply IS the wire's state: the bridge answers success only
            # with its mpc_active set. Seed the /link_status mirror from it, or a
            # streaming mode queued during the activation (the GUI's IDLE ->
            # ACTIVE:TRAJECTORY plan publishes it the moment control_mode reads
            # STANDBY, i.e. BEFORE the arm) is consumed below on this very tick
            # against the PRE-arm 10 Hz /link_status and refused as "DISARMED"
            # -- the GUI then times out waiting for TRAJECTORY. Whether the
            # /link_status publish lands between the arm and this tick depends
            # only on the two free-running 10 Hz timer phases, fixed per launch
            # (Sitting 2026-11-02 17:00: refused twice; 2026-10-28 09:49 once).
            # A later disarm still reaches wire_armed through /link_status.
            ctx.wire_armed = True

        if self._deactivating:
            return self._step_deactivate(ctx)

        # ACTIVE-wide reconciliation (armed, past the arm phase): the robot,
        # not the last command, decides the state.
        if self._legs_out(ctx):
            ctx.notify(
                f'legs out of closed loop for {self.LEGS_OUT_TICKS} consecutive '
                f'ticks while ACTIVE -- the robot is not active; following it '
                f'out of ACTIVE')
            self._exit_disarm_only()
            return self._legs_out_target(ctx)

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

    @staticmethod
    def _legs_out_target(ctx):
        """Where a legs-out exit goes (ARMING_CONTRACT A7). IDLE only when all
        six Legs read Idle now (the Platform came down); otherwise BOOT, whose
        IDLE gate holds and tells the operator how to lower it: the legs-out
        test is "not all six Closed-loop", which one Leg dropped with five
        holding a tilted Platform also passes, and IDLE would claim Stow."""
        if ctx_idle_refusal(ctx) is None:
            return RobotState.IDLE
        return RobotState.BOOT

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
                    f'out of ACTIVE')
                return self._legs_out_target(ctx)
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
    decided on drive state alone since FW 53, 2026-11-26): a real fault, once
    everything has cleared, first requests the profiled DEACTIVATE whenever all
    six legs are closed-loop (not gated on is_homed since BUG-122, nor on the
    state the visit began in since FW 53; see _real_fault_exit), and holds
    FAULT until it succeeds. Without it a forced ACTIVE->FAULT (whose
    ActiveHandler.on_exit deactivate the orchestrator cancels), any fault
    during LEVELLING (whose level_activate holds the same active pose until
    level_deactivate, and whose on_exit only clears requests), or an FW 53
    aborted rise that HOLDS the legs Closed-loop mid-air, reached BOOT with the
    legs still up.
    """

    def __init__(self):
        # Sticky for the lifetime of this FAULT visit (recomputed on every on_enter).
        # True iff FAULT was entered PURELY because of a Teensy guard latch (from
        # ACTIVE) with NO ODrive-level error — see on_enter.
        self._guard_only = False
        # Real-fault exit stow bookkeeping, all reset on every on_enter.
        self._stow_requested = False   # our 'deactivate' is in flight
        self._stow_done = False        # it succeeded: the legs are stowed
        self._stow_failed = False      # it failed: hold FAULT until clear_errors

    def on_enter(self, ctx):
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

        Returns BOOT, or None to hold FAULT. Decided on the Legs' drive state
        alone, whatever state the visit began in (FW 53, 2026-11-26):

        * All six legs closed-loop (any height: an Active pose, an FW 53
          aborted rise HELD mid-air, or at Stow): request the profiled
          DEACTIVATE first and hold FAULT until it succeeds, so the machine
          never reaches BOOT -> IDLE ("platform stowed, legs idle") while they
          may hold the Platform up (ARMING_CONTRACT choreography step 6). The
          orchestrator has no Leg position it trusts here, so Legs already at
          Stow are lowered too: the firmware accepts that DEACTIVATE, arrives at
          once and IDLEs them through its Stow gate.
        * Otherwise BOOT at once (all six Idle, e.g. an FW 53 rise returned to
          Stow; or a mixed set). BOOT applies the one Leg IDLE predicate
          (A7 for its IDLE claim, A6 through StateMachine._homing_entry_refused
          for HOMING): while some Leg is still Closed-loop and some is not (the
          other five holding the Platform up after one Leg dropped) BOOT holds
          and the operator is told how to lower it. A profiled stow must not
          drive a Leg that is not Closed-loop, so the exit never stows a mixed
          set.

        Until FW 53 the stow also required the visit to have begun in ACTIVE or
        LEVELLING; an FW 53 abort can leave the six HELD after any ladder, and a
        visit that began elsewhere (a boot timeout under a posed Platform) is no
        safer left up, so that condition is gone.

        Not gated on is_homed (BUG-122, 2026-11-20). The bridge now clears
        is_homed when ANY drive loses its position reference, the Hand included,
        and a Hand-only loss must still stow the posed legs: skipping the stow
        would start HOMING with the legs at an active pose. legs_closed_loop
        already proves each leg's own reference: an ODrive cannot be in
        CLOSED_LOOP with no active error after MISSING_ESTIMATE or a reboot.
        DEACTIVATE only IDLEs the Hand (leg_deactivate.cpp idle_hand_now), so a
        lost Hand frame cannot move anything during the stow.
        """
        if self._stow_done:
            return RobotState.BOOT
        if self._stow_requested or self._stow_failed:
            return None
        if not ctx.legs_closed_loop:
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
