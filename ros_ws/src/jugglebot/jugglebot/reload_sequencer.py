"""Pure-Python FSM for the BB→Jugglebot reload sequence (MVP goal 4).

`reload_coordinator_node` is a thin ROS wrapper around this FSM: it feeds
observations in (BB heartbeat, mocap freshness, streaming, control mode, ball-caught
status) and executes the actions the FSM asks for (call ``bb/reload``, send
``bb/throw_at_target``, PREPARE the catch, RECENTER, SAFE_ABORT). The FSM itself
imports NO ROS — every transition is a pure function of ``(now, observations, discrete
events)`` so the whole sequence is unit-testable without a running graph.

The RELOAD action OWNS the platform + hand for its duration (reload-action-catch-latch
plan). It runs from the active streaming mode (``TRAJECTORY``, armed + streaming a
hold) and raises the catch-armed latch for the flight window — replacing the retired
persistent CATCH mode the operator used to hold.

Design (plan § Reload sequence):

1. **CHECKING** — loud precondition rejects: the robot in the active reload control
   mode (``TRAJECTORY``), ``bb/heartbeat`` connected ∧ IDLE, mocap fresh, trajectory
   streaming. If the hand is empty, call ``bb/reload`` and wait for the heartbeat
   ``RELOADING → IDLE`` with ``ball_in_hand`` (10 s timeout → ``REJECTED_NO_BALL``).
2. **AIMING** — send ``bb/throw_at_target`` at the world-frame catch point
   (``use_target_point``); a lead below BB's floor is a ``REJECTED_CANT_MAKE_LEAD``,
   a BB solver/limit reject is ``REJECTED_BB(<message>)``.
3. **On throw-accept (AIMING → THROW_PENDING)** — emit ``ACTION_PREPARE_CATCH``: the
   node proactively primes the hand to the top of its stroke AND raises the catch-armed
   latch, so the reactive catch path can actuate the platform for the flight window and
   the catch stroke never races the ball with a high-jerk late prime. Sets ``_prepared``.
4. **THROW_PENDING** — wait for the ``ThrowAnnouncement``. No announcement within
   ``throw_delay + 0.5 s`` ⇒ ``ABORTED_NO_ANNOUNCEMENT`` (routes through SAFE_ABORT,
   since PREPARE already primed the hand + raised the latch).
5. **BALL_IN_FLIGHT / CATCHING** — the EXISTING catch path (correlation →
   catch_coordinator → ``catch/dynamic_target`` → trajectory ``build_catch`` + hand
   fire) runs on its own; the sequencer only watches. A post-release infeasible catch
   target (surfaced via ``trajectory/target_feedback``) ⇒ ``MISSED_INFEASIBLE`` (the
   platform holds its last valid pose; the hand fires per its armed schedule).
6. **SETTLING → CAUGHT** — ball status ``CAUGHT`` from the tracker within the confirm
   window ⇒ ``CAUGHT`` (FSM emits ``ACTION_RECENTER``); otherwise ``MISSED``.

Terminal actions (executed by the node):
  - ``ACTION_RECENTER`` — on a successful terminal (CAUGHT): lower the latch + go_home
    (re-center to level neutral; the hand keeps the caught ball, no retract).
  - ``ACTION_SAFE_ABORT`` — on ANY not-caught terminal once ``_prepared`` (the latch
    was raised / hand primed): retract the hand to bottom, lower the latch, go_home.
  - A reject BEFORE ``_prepared`` (a precondition reject that never armed anything)
    emits ``ACTION_NONE`` — there is nothing to safe.

Aborts at any phase: the operator switching the control mode away from the active
reload mode ⇒ ``ABORTED_MODE_CHANGED``; a BB fault (heartbeat ERROR) ⇒
``ABORTED_BB_ERROR``. Both route through SAFE_ABORT once ``_prepared``.

The FSM never actuates the robot itself and never bypasses the feasibility gate — the
throw goes through ``ball_butler_node`` and every platform motion goes through
``trajectory_node`` / ``planner`` / ``feasibility.validate``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

# ── BB heartbeat state enum (BallButlerHeartbeat.msg) ──────────────────────────
BB_STATE_BOOT = 0
BB_STATE_IDLE = 1
BB_STATE_RELOADING = 2
BB_STATE_THROWING = 3
BB_STATE_ERROR = 127

# ── Feedback phases (Reload.action feedback.phase) ─────────────────────────────
PHASE_CHECKING = 'CHECKING'
PHASE_AIMING = 'AIMING'
PHASE_THROW_PENDING = 'THROW_PENDING'
PHASE_BALL_IN_FLIGHT = 'BALL_IN_FLIGHT'
PHASE_CATCHING = 'CATCHING'
PHASE_SETTLING = 'SETTLING'

# ── Actions the node executes on the FSM's behalf ──────────────────────────────
ACTION_NONE = 'none'
ACTION_CALL_RELOAD = 'call_reload'          # invoke bb/reload once
ACTION_SEND_THROW = 'send_throw'            # invoke bb/throw_at_target once
ACTION_PREPARE_CATCH = 'prepare_catch'      # prime hand to top + raise the catch-armed latch
ACTION_RECENTER = 'recenter'                # lower latch + go_home (successful catch)
ACTION_SAFE_ABORT = 'safe_abort'            # retract hand + lower latch + go_home (abort once prepared)

# The control mode RELOAD runs within: ACTIVE + streaming a hold, in TRAJECTORY — the
# mode whose go_home/go_to_pose surface the action's recenter/pre-position use and where
# the catch-armed latch actuates the platform. Leaving it mid-sequence is the documented
# abort (ABORTED_MODE_CHANGED). Replaces the retired persistent CATCH mode.
RELOAD_CONTROL_MODE = 'TRAJECTORY'

# ── Defaults / floors (plan § Reload sequence) ─────────────────────────────────
DEFAULT_THROW_DELAY_S = 3.0                 # >= BB's ~2.5 s lead floor
MIN_THROW_LEAD_S = 2.5                      # BB CANT_MAKE_LEAD floor
RELOAD_TIMEOUT_S = 10.0                     # RELOADING → IDLE + ball_in_hand
ANNOUNCEMENT_GRACE_S = 0.5                  # announcement must land within delay + this
CATCH_CONFIRM_WINDOW_S = 0.7                # CAUGHT within this of predicted landing


@dataclass
class ReloadObservations:
    """A snapshot of everything the FSM reasons about — pure observations, no
    node/issuance state (that lives inside the FSM)."""
    now: float
    control_mode: str = ''            # must be the active reload mode (TRAJECTORY) throughout
    bb_connected: bool = False
    bb_state: int = BB_STATE_BOOT
    ball_in_hand: bool = False
    mocap_fresh: bool = False
    streaming: bool = False
    ball_caught: bool = False         # tracker CAUGHT confirmation for the thrown ball
    catch_error_mm: float = float('nan')


@dataclass
class ReloadResult:
    success: bool
    outcome: str
    catch_error_mm: float = float('nan')


@dataclass
class ReloadDecision:
    phase: str
    action: str = ACTION_NONE
    done: bool = False
    result: Optional[ReloadResult] = None


@dataclass
class ReloadSequencer:
    """The reload FSM. Construct with the resolved throw parameters + catch point, then
    ``start(now)`` and drive with ``step(now, obs)``; feed the two discrete async events
    (throw service result, announcement) via ``note_throw_result`` / ``note_announcement``.
    ``note_catch_feasibility`` reports a post-release ``target_feedback`` rejection."""

    catch_point_mm: tuple                       # (x, y, z) world frame for the throw aim
    throw_delay_s: float = DEFAULT_THROW_DELAY_S
    reload_timeout_s: float = RELOAD_TIMEOUT_S
    announcement_grace_s: float = ANNOUNCEMENT_GRACE_S
    catch_confirm_window_s: float = CATCH_CONFIRM_WINDOW_S
    min_lead_s: float = MIN_THROW_LEAD_S

    # ── internal state ──
    _phase: str = field(default=PHASE_CHECKING, init=False)
    _reloading: bool = field(default=False, init=False)
    _reload_requested: bool = field(default=False, init=False)
    _reload_deadline: float = field(default=0.0, init=False)
    _throw_sent: bool = field(default=False, init=False)
    _throw_result: Optional[tuple] = field(default=None, init=False)   # (accepted, msg)
    _announced: bool = field(default=False, init=False)
    _announcement_deadline: float = field(default=0.0, init=False)
    _landing_time_perf: Optional[float] = field(default=None, init=False)  # announced landing (perf clock)
    _catch_infeasible: Optional[str] = field(default=None, init=False)  # gate code
    _throw_time: float = field(default=0.0, init=False)
    _settle_deadline: float = field(default=0.0, init=False)
    _prepared: bool = field(default=False, init=False)  # PREPARE ran (hand primed + latch raised)
    _finished: bool = field(default=False, init=False)
    _result: Optional[ReloadResult] = field(default=None, init=False)  # cached terminal result

    def __post_init__(self):
        if self.throw_delay_s <= 0.0:
            self.throw_delay_s = DEFAULT_THROW_DELAY_S

    # ── discrete async events (from the node) ──────────────────────────────────

    def note_throw_result(self, accepted: bool, message: str = '') -> None:
        """Report the ``bb/throw_at_target`` service outcome (accept/reject)."""
        self._throw_result = (bool(accepted), str(message))

    def note_announcement(self, now: float,
                          landing_time_perf: Optional[float] = None) -> None:
        """Report that a matching ``ThrowAnnouncement`` was observed.

        Gated on the throw having been COMMANDED (``_throw_sent``), NOT on the phase:
        ``ball_butler_node`` publishes the announcement synchronously INSIDE the
        ``bb/throw_at_target`` handler — before the service call returns, i.e. while the
        FSM is still AIMING — so a phase gate (THROW_PENDING/BALL_IN_FLIGHT) would drop it
        deterministically on every real reload. ``landing_time_perf`` is the announced
        landing time already converted to the FSM's perf clock (the node does the ROS→perf
        crossing); ``None`` falls back to the release-relative estimate for the settle
        deadline (see :meth:`_landing_perf`)."""
        if self._throw_sent and not self._finished:
            self._announced = True
            if landing_time_perf is not None:
                self._landing_time_perf = float(landing_time_perf)

    def note_catch_feasibility(self, accepted: bool, code: str = '') -> None:
        """Report a ``trajectory/target_feedback`` decision for the catch target.

        A genuine post-release REJECTION (a WORKSPACE / TOO_FAST gate reject that still
        stands at catch time) means the platform can't reach the catch →
        MISSED_INFEASIBLE. A SUBSEQUENT acceptance
        clears it: only a rejection that STILL stands at catch time counts. (The node
        already drops the transient FROZEN / STALE_STATE codes — a target inside the
        reach-freeze window / a plant-state race — which fire on every real flight and are
        NOT catch infeasibility.)"""
        if self._phase not in (PHASE_BALL_IN_FLIGHT, PHASE_CATCHING):
            return
        if accepted:
            self._catch_infeasible = None
        else:
            self._catch_infeasible = code or 'INFEASIBLE'

    # ── driver ─────────────────────────────────────────────────────────────────

    def start(self, now: float) -> None:
        self._phase = PHASE_CHECKING
        self._reload_deadline = now + self.reload_timeout_s

    def step(self, now: float, obs: ReloadObservations) -> ReloadDecision:
        if self._finished:
            # Terminal: replay the stored result (never None once finished) so a stray
            # extra step can't hand the consumer a None to unpack into the action result.
            return ReloadDecision(self._phase, ACTION_NONE, True, self._result)

        # ── Universal aborts (checked every phase before the phase logic) ──────
        # A lead below BB's floor can never be made — reject up front (also caught by
        # BB's own CANT_MAKE_LEAD, surfaced as REJECTED_BB, but this is loud + early).
        if self._phase == PHASE_CHECKING and self.throw_delay_s < self.min_lead_s:
            return self._reject('CANT_MAKE_LEAD')
        # RELOAD runs within the active streaming mode (TRAJECTORY); leaving it
        # mid-sequence is the documented abort. Once _prepared (latch raised / hand
        # primed) the terminal routes through SAFE_ABORT; a pre-prepare leave rejects.
        if obs.control_mode != RELOAD_CONTROL_MODE:
            if self._phase == PHASE_CHECKING and not self._reload_requested:
                return self._reject('WRONG_MODE')
            return self._abort('MODE_CHANGED')
        # A BB fault at any point is fatal to the sequence.
        if obs.bb_state == BB_STATE_ERROR:
            if self._phase == PHASE_CHECKING and not self._reload_requested:
                return self._reject('BB_ERROR')
            return self._abort('BB_ERROR')

        if self._phase == PHASE_CHECKING:
            return self._step_checking(now, obs)
        if self._phase == PHASE_AIMING:
            return self._step_aiming(now, obs)
        if self._phase == PHASE_THROW_PENDING:
            return self._step_throw_pending(now, obs)
        if self._phase in (PHASE_BALL_IN_FLIGHT, PHASE_CATCHING):
            return self._step_in_flight(now, obs)
        if self._phase == PHASE_SETTLING:
            return self._step_settling(now, obs)
        return ReloadDecision(self._phase, ACTION_NONE, False, None)

    # ── phase handlers ─────────────────────────────────────────────────────────

    def _step_checking(self, now: float, obs: ReloadObservations) -> ReloadDecision:
        # If we already asked BB to reload, wait for RELOADING → IDLE + ball_in_hand.
        if self._reload_requested:
            if obs.ball_in_hand and obs.bb_state == BB_STATE_IDLE:
                return self._enter_aiming(now)
            if now >= self._reload_deadline:
                return self._reject('NO_BALL')
            return ReloadDecision(PHASE_CHECKING, ACTION_NONE, False, None)

        # First pass through preconditions (loud rejects, plan § Reload sequence 1).
        if not obs.bb_connected:
            return self._reject('BB_DISCONNECTED')
        if obs.bb_state != BB_STATE_IDLE:
            return self._reject('BB_BUSY')
        if not obs.mocap_fresh:
            return self._reject('MOCAP_STALE')
        if not obs.streaming:
            return self._reject('NOT_STREAMING')

        if not obs.ball_in_hand:
            # Hand empty → ask BB to reload, then wait (still PHASE_CHECKING).
            self._reload_requested = True
            self._reload_deadline = now + self.reload_timeout_s
            return ReloadDecision(PHASE_CHECKING, ACTION_CALL_RELOAD, False, None)

        return self._enter_aiming(now)

    def _enter_aiming(self, now: float) -> ReloadDecision:
        self._phase = PHASE_AIMING
        return self._step_aiming(now, None)

    def _step_aiming(self, now: float, obs) -> ReloadDecision:
        # Send the throw exactly once.
        if not self._throw_sent:
            self._throw_sent = True
            self._throw_time = now
            return ReloadDecision(PHASE_AIMING, ACTION_SEND_THROW, False, None)
        # Await the service result.
        if self._throw_result is None:
            return ReloadDecision(PHASE_AIMING, ACTION_NONE, False, None)
        accepted, message = self._throw_result
        if not accepted:
            return self._reject('BB', message)
        # Stamp the announcement deadline from the CONFIRMED throw, not from when
        # SEND_THROW was issued: the blocking bb/throw_at_target call can eat up to
        # _SERVICE_WAIT_S before the throw is even accepted, which would otherwise
        # silently burn the announcement grace window before the ball is committed.
        self._announcement_deadline = (
            now + self.throw_delay_s + self.announcement_grace_s)
        self._phase = PHASE_THROW_PENDING
        # The throw is committed: proactively PREPARE the catch (node primes the hand to
        # top + raises the catch-armed latch). Priming here — ~throw_delay before the ball
        # flies — keeps the catch stroke from racing the ball with a high-jerk late prime,
        # and raising the latch lets the reactive catch path actuate the platform. From
        # here on, any not-caught terminal must SAFE_ABORT (retract + lower latch).
        self._prepared = True
        return ReloadDecision(PHASE_THROW_PENDING, ACTION_PREPARE_CATCH, False, None)

    def _step_throw_pending(self, now: float, obs: ReloadObservations) -> ReloadDecision:
        if self._announced:
            self._phase = PHASE_BALL_IN_FLIGHT
            # Confirm CAUGHT within the window PAST the true landing. The deadline must
            # include the ball's time-of-flight (~0.6–0.7 s): the announced
            # landing_time_perf IS release + ToF, so anchoring on it (not on release =
            # throw_time + throw_delay) is the difference between a nominal catch reading
            # CAUGHT and it timing out MISSED before the ball has even landed.
            self._settle_deadline = self._landing_perf() + self.catch_confirm_window_s
            return ReloadDecision(PHASE_BALL_IN_FLIGHT, ACTION_NONE, False, None)
        if now >= self._announcement_deadline:
            # No announcement → the catch path never armed the hand; platform holds.
            return self._abort('NO_ANNOUNCEMENT')
        return ReloadDecision(PHASE_THROW_PENDING, ACTION_NONE, False, None)

    def _step_in_flight(self, now: float, obs: ReloadObservations) -> ReloadDecision:
        if self._catch_infeasible is not None:
            # Platform couldn't reach the catch; the hand fires per its armed schedule.
            return self._finish(ReloadResult(
                False, f'MISSED_INFEASIBLE_{self._catch_infeasible}',
                obs.catch_error_mm))
        if obs.ball_caught:
            self._phase = PHASE_SETTLING
            return self._finish(ReloadResult(True, 'CAUGHT', obs.catch_error_mm))
        if now >= self._settle_deadline:
            self._phase = PHASE_SETTLING
            return self._step_settling(now, obs)
        # Transition the reported phase to CATCHING once we're near the (true) arrival.
        near = now >= (self._landing_perf() - self.catch_confirm_window_s)
        self._phase = PHASE_CATCHING if near else PHASE_BALL_IN_FLIGHT
        return ReloadDecision(self._phase, ACTION_NONE, False, None)

    def _landing_perf(self) -> float:
        """Predicted landing time in the FSM's perf clock: the announced
        ``landing_time_perf`` (release + ToF) when available, else the release-relative
        fallback (``throw_time + throw_delay`` — treats release AS landing, so only used
        when no announcement landing stamp reached the FSM)."""
        if self._landing_time_perf is not None:
            return self._landing_time_perf
        return self._throw_time + self.throw_delay_s

    def _step_settling(self, now: float, obs: ReloadObservations) -> ReloadDecision:
        if obs.ball_caught:
            return self._finish(ReloadResult(True, 'CAUGHT', obs.catch_error_mm))
        return self._finish(ReloadResult(False, 'MISSED', obs.catch_error_mm))

    # ── terminal helpers ───────────────────────────────────────────────────────

    def _reject(self, code: str, message: str = '') -> ReloadDecision:
        outcome = f'REJECTED_{code}'
        if message:
            outcome = f'{outcome}({message})'
        return self._finish(ReloadResult(False, outcome, float('nan')))

    def _abort(self, code: str) -> ReloadDecision:
        return self._finish(ReloadResult(False, f'ABORTED_{code}', float('nan')))

    def _finish(self, result: ReloadResult) -> ReloadDecision:
        self._finished = True
        self._result = result
        return ReloadDecision(self._phase, self._terminal_action(result), True, result)

    def _terminal_action(self, result: ReloadResult) -> str:
        """The cleanup action the node runs on a terminal decision:
          - a successful catch RE-CENTERs (lower the latch + go_home; the hand keeps the
            caught ball, so no retract);
          - ANY not-caught terminal that got past PREPARE SAFE_ABORTs (the latch was
            raised / hand primed, so it must be safed — retract + lower latch + go_home);
          - a reject BEFORE PREPARE raised nothing → no cleanup (ACTION_NONE).
        The action fires exactly once (the finished-replay path in :meth:`step` returns
        ACTION_NONE), so the node never re-runs a terminal action."""
        if result.success:
            return ACTION_RECENTER
        if self._prepared:
            return ACTION_SAFE_ABORT
        return ACTION_NONE

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def prepared(self) -> bool:
        """True once PREPARE ran (hand primed + catch-armed latch raised). The node
        uses this to safe the robot on a node-level early exit (cancel/timeout/shutdown)
        that bypasses the FSM's own terminal SAFE_ABORT."""
        return self._prepared

    @property
    def finished(self) -> bool:
        return self._finished


def compute_catch_point_mm(initial_height_mm: float, active_z_mm: float,
                           landing_z_offset_mm: float = 0.0) -> tuple:
    """The world-frame catch point BB aims at: directly above Jugglebot's base at the
    hand-CUP catch height. ``(0, 0, initial_height + active_z + landing_z_offset)``.

    The caller (``reload_coordinator_node``) passes ``landing_z_offset_mm =
    HAND_CATCH_OFFSET_MM`` (64.78 mm) so the aim lands on the CUP plane — where the hand
    actually intercepts the ball — not the platform centroid. That matches the catch
    plane the rest of the stack uses (``throw_ballistics._DEFAULT_CATCH_HEIGHT_MM``,
    ``ball_tracker`` landing_z, ``catch_coordinator`` landing_z_offset).

    NOTE (Phase-7a verification task): the z-convention (initial_height + STOW→ACTIVE lift
    + hand-cup offset) is hardware-UNVERIFIED — 7a's aim-only (speed 0) command verifies
    the QTM-world vs jugglebot-base frame AND this z before any ball flies (plan § Reload
    sequence 2)."""
    return (0.0, 0.0,
            float(initial_height_mm) + float(active_z_mm) + float(landing_z_offset_mm))
