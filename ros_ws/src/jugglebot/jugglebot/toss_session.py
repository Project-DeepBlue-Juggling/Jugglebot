"""Pure-Python outer FSM for a CONTINUOUS self-toss session (TossContinuous.action).

``TossContinuous`` is the operator-requested bridge between the validated single
``Toss`` and 2-ball juggling: throw → catch → dwell → throw, ``num_throws`` times,
from one goal, with ``stop_on_miss`` defaulting TRUE (operator decision (c),
2026-07-28, recorded in ``logbook/2026-07-28-anomaly-fixes-validation-sitting.md``
§ Decisions taken).

**This module is a SEQUENCER, not a capability.** Every cycle is an ordinary toss
run by :class:`~jugglebot.toss_sequencer.TossSequencer` through the same
coordinator code path, so every Toss precondition, arming order, abort ladder and
terminal applies per cycle UNCHANGED. What this FSM owns is exactly three things:
*when* the next cycle starts, *whether* it starts at all, and the per-cycle
accounting the operator scores a sitting from.

The design rests on one verified firmware fact: the catch stroke ENDS at 0 rev
(``Trajectory.h`` ``buildCatch``, ``xA = {x3, x5, x6, 0.f}``) and a kind-0 throw
stroke STARTS at 0 rev (``buildThrow``, ``xA = {0.f, x1, x2, x3}``). **A caught
ball therefore needs no hand move between cycles** — the catch is its own re-park
and the throw is its own catch-prime. Measured on the 2026-07-27 sitting: hand
``pos_meas`` is within ±0.045 rev of park at the CAUGHT instant on all 17
self-tosses, worst excursion 0.069 rev over the following 3 s against a ±0.5 rev
park band (7.2×) — so the next cycle's ``REJECTED_HAND_NOT_PARKED`` gate passes
with nothing commanded in between. The platform side is free for the same reason:
a CAUGHT toss ends in ``ACTION_STAY`` and ``trajectory/commanded_position``
reports the live pose, so cycle N+1's throw site A is cycle N's catch pose.

## Session invariants (enforced here, pinned by tests/ros/test_toss_session.py)

**S1 — at most ONE cycle is live.** ``ACTION_START_CYCLE`` is emitted only when no
cycle is outstanding, and never again until :meth:`note_cycle_result` has consumed
the previous one. Two live ``TossSequencer``s would double-own the hand on the
Teensy's last-writer-wins queue and fight over the single ``catch/armed`` latch —
the exact hazard the node's cross-action busy claim exists to prevent, reproduced
INSIDE one goal.

**S2 — the session commands NO motion of its own.** There is no session-level
positioning move, no session-level ``go_home``, no session-level hand dispatch.
Every commanded motion in a session belongs to a cycle and is already covered by
the single-toss ladder. A session-level move would be new commanded motion with
zero hardware evidence behind it.

**S3 — ``stop_on_miss`` stops at the CYCLE BOUNDARY and introduces no new abort
point.** The safing on a miss is the cycle's OWN ``ACTION_SAFE_ABORT`` (armed-off
→ retract → latch-off → go_home), which has already run by the time this FSM sees
the result. "Stopping" is therefore literally *not starting cycle N+1*. That is
what makes the stop immediate and side-effect-free: a loose ball on the floor
under a machine that is about to stroke again is the scenario the flag exists to
prevent, and the way to prevent it is to not stroke.

**S4 — cancellation obeys the per-cycle phase rules verbatim.** This FSM invents
no abort points. Mid-cycle cancellation is adjudicated by the node's
``_toss_cancel_deferred`` exactly as for a single Toss (aborting a catch
mid-flight drops the ball on the robot, so a late cancel is DEFERRED to the
cycle's own terminal). A cancel during DWELL is honoured immediately and moves
nothing — the machine is already quiescent.

**S5 — the dwell is a QUIESCENT WAIT before the cycle, never a stretched
throw_delay.** Both realisations satisfy the same arithmetic, and this one was
chosen for three concrete failure modes the other has:
  1. stretching ``throw_delay`` leaves ``catch/armed`` RAISED for the whole
     dwell with a ball resting in the cup, so ``catch_coordinator``'s reactive
     catch path is live for that entire window — any tracked ball entering the
     volume (a bounced-out ball re-detected on the floor, a foreign object) can
     command platform motion under a loaded cup. Waiting keeps the armed window
     exactly the one the single-toss ladder validated;
  2. an armed dwell looks identical to an about-to-throw machine, so the
     operator's between-cycles intervention window is one in which the robot is
     armed;
  3. a stretched delay moves every cycle's internal timing off the profile the
     hardware measured (positioning + prepare + event_delay at throw_delay
     3.5–5 s). Waiting keeps each cycle bit-identical to a validated single toss;
     the only new thing is when it starts.

## Dwell — definition, and why the floor is derived rather than chosen

``dwell_time_s`` is *previous SCHEDULED LANDING → next RELEASE*. A cycle's release
is its own accept + ``throw_delay``, so

    cycle_start(N+1) = landing(N) + dwell − throw_delay

and the session simply idles until that instant. The floor follows from two
landed constants and one measurement, with nothing free in it:

    dwell_floor = throw_delay + dwell_margin_s

* ``throw_delay`` cannot go below ``MIN_TOSS_THROW_DELAY_S`` = 3.5 s — the toss
  FSM's own ``REJECTED_CANT_MAKE_LEAD`` gate, which exists so the positioning +
  prepare budget plus the 1.0 s ``event_delay`` floor generically fit inside the
  delay. Verified against the real FSM (probe, 2026-07-29): 3.49 s ⇒
  ``REJECTED_CANT_MAKE_LEAD``, 3.50 s ⇒ dispatched.
* ``dwell_margin_s`` covers the handoff the machine cannot avoid: the CAUGHT
  verdict lands at *landing + 0.202–0.442 s* (median 0.209; 17/17 self-tosses of
  the 2026-07-27 sitting, ``logbook/2026-07-28-caught-gate-xy-plausibility.md``)
  plus the tick that observes it and the tick that starts the next cycle
  (2 × 0.05 s). Worst measured: 0.442 + 0.10 = 0.542 s, so the shipped 0.6 s
  covers it with the ``ACTION_STAY`` service call inside the margin.

So the absolute floor is **4.10 s** (at the FSM's 3.5 s delay floor) and the floor
at the 5.0 s default delay is **5.60 s**. The 4.10 s figure is only a real floor
if ``throw_delay`` itself is gated, so CHECKING refuses ``throw_delay <
MIN_TOSS_THROW_DELAY_S`` as ``REJECTED_THROW_DELAY`` before it looks at the dwell
— otherwise a 2.0 s delay with a 3.0 s dwell satisfies the derived inequality,
gets ACCEPTED, and dies ``REJECTED_CANT_MAKE_LEAD`` one cycle later naming a
field the operator did not think was in play.

**The 2.0 s figure in this phase's brief is unachievable and was not adopted.**
It would need ``throw_delay ≤ 1.458 s``, i.e. 2.042 s *below* the toss FSM's own
floor. Lowering that floor is a change to the arming/release timing of a
hardware-validated path — a safety fork, deliberately not taken here. It is the
lever a future phase can pull, and pulling it needs a bench measurement of the
real positioning + prepare budget, not an argument.

A dwell under the floor is REFUSED (``REJECTED_DWELL``), never silently stretched:
a cadence the machine quietly ignores is a lie about what it did. Lateness in the
other direction is absorbed — a cycle whose handoff ran long simply reports a
longer achieved dwell in ``per_cycle_dwell_s`` and never aborts.

### The MISS path needs a bigger floor than the CAUGHT one, and gets it here

``dwell_margin_s`` sizes the CAUGHT handoff, where nothing is commanded: a verdict
lands and two ticks pass. A MISSED cycle the session CONTINUES past
(``stop_on_miss`` False) hands over through a whole ``ACTION_SAFE_ABORT`` ladder
instead, and every rung of it is dispatched on a SERVICE ACK — ``_go_home()``
returns when ``trajectory_node`` has *installed* a 2.0 s recentre profile, not
when the platform has arrived, and ``_retract_hand_with_retries()`` returns on the
first successful ack, not on motion. So ``_run_toss_cycle`` returns while the
machine is still moving, and the plain ``landing + dwell − throw_delay``
arithmetic starts cycle N+1 inside the previous cycle's teardown. At the shipped
defaults it already does: 6.0 − 5.0 puts the next cycle 1.0 s past the landing,
1.7 s before the recentre lands.

:data:`DEFAULT_SESSION_MISS_CLEANUP_S` (2.80 s) is therefore applied as a FLOOR on
``landing → next cycle start`` after any non-success cycle. It can only lengthen a
gap, never shorten one, so a session that already dwells long enough is
bit-unchanged. Neither consequence it prevents is a hazard — a mid-traverse throw
site A and a hand still descending both end in loud refusals — but one of them
(``REJECTED_HAND_NOT_PARKED``) is a machine-fault verdict for a cadence fault, and
it would route the operator to the wrong subsystem.

## Known limitation inherited from Phase E — chaining near the ±150 mm box edge

A catch parks the platform CENTROID slightly outside B so the CUP lands ON B, and
``trajectory/commanded_position`` publishes the CENTROID. For a fixed-B session the
offset is largest at cycle 2 and then collapses (measured through the production
``predicted_catch_command``, 2026-07-29, B on the +x axis):

    B_x     cycle-2 A_x     cycle-3 A_x     cycle-4 A_x
     70.0        71.448          69.970         70.001
    140.0       142.894         139.940        140.001
    146.0       149.017         145.938        146.001
    147.0       150.038   ← outside the ±150 planning box

So a fixed-B chain CONVERGES, and the only cycle at risk is **cycle 2**. The
frontier is sharp: **|B| ≤ 146.5 mm chains, |B| ≥ 147.0 mm does not.** Note the
binding gate is the ±150 planning box on A, NOT the 150 mm displacement cap — the
residual |B−A| never exceeds 3.1 mm. The coordinator pre-checks this before
anything moves (``REJECTED_CHAIN_UNREACHABLE``) rather than letting the session
throw one ball, catch it, and then refuse cycle 2 with the platform parked
off-box and a ball in the cup.

## Possession across the dwell — stated honestly, not designed away

Between catch and next throw the ball sits in the cup with the platform holding.
**Nothing re-verifies possession there.** Contract C-POSSESS-1's verdict is minted
at ARRIVAL (the tracker declares CAUGHT because the mocap marker vanished); no
shipped source can answer RETENTION, so a post-CAUGHT bounce-out during the dwell
leaves the possession latch set and cycle N+1 fires an empty stroke. That stroke
is benign — it is exactly the no-ball dry-trace case — but the verdict is wrong,
and a session multiplies the exposure by ``num_throws``. ``stop_on_miss`` does not
close it either: the bounce-out happens *after* a CAUGHT verdict.

The seam that closes it is already in place and needs no wire change: the
coordinator routes every possession question through ``_possession_confirmed`` →
``_possession_source``, and the ball-in-cup hand sensor (installed 2026-07-28)
becomes the PRIMARY source there. When it does, cycle N+1's existing
``ball_seated`` precondition becomes a real retention check with no edit to this
module, the action, or the FSM. Recorded in
``ros_ws/docs/ball_possession_contract.md`` § 7.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

from jugglebot.toss_sequencer import (
    CATCH_CONFIRM_WINDOW_S,
    DEFAULT_TOSS_THROW_DELAY_S,
    MIN_TOSS_THROW_DELAY_S,
    TossResult,
)

# ── Feedback phases (TossContinuous.action feedback.phase — LOCKED strings) ────
# While a cycle is live the node reports the CYCLE's Toss phase verbatim; these
# two are the session's own.
SESSION_PHASE_CHECKING = 'SESSION_CHECKING'
SESSION_PHASE_DWELL = 'DWELL'

# ── Actions the node executes on the session's behalf ─────────────────────────
SESSION_ACTION_NONE = 'none'
SESSION_ACTION_START_CYCLE = 'start_cycle'   # build + run ONE TossSequencer, then
                                             #   feed its result back through
                                             #   note_cycle_result. Never emitted
                                             #   while a cycle is outstanding (S1).

# ── Defaults / floors (the NO-CONFIG fallbacks only) ──────────────────────────
# The node resolves the generated JB_OP_TOSS_SESSION_* keys and passes them into
# the ctor; these literals serve standalone/test use and the config drift-guard
# test pins each pair equal — the same pattern as the toss FSM's
# DEFAULT_TOSS_FLIGHT_TIME_S.
DEFAULT_SESSION_DWELL_S = 6.0        # 0 => this. Comfortably over the 5.60 s floor
                                     # at the 5.0 s default throw delay, so the
                                     # default combination is legal without the
                                     # operator doing arithmetic.
DEFAULT_SESSION_DWELL_MARGIN_S = 0.6 # the unavoidable landing -> next-cycle-start
                                     # handoff: worst measured CAUGHT-verdict
                                     # latency 0.442 s (17/17, 2026-07-27 sitting)
                                     # + 2 x 0.05 s node ticks = 0.542 s, with the
                                     # ACTION_STAY arm_catch call inside the rest.
DEFAULT_SESSION_MAX_THROWS = 20      # upper bound on num_throws. An unbounded
                                     # session is a machine stroking unattended for
                                     # an unbounded time; 20 cycles at the 6.0 s
                                     # default dwell is ~2.3 minutes, well past any
                                     # rung of the hardware ladder.

# ── The MISS-path cleanup floor (see the module docstring's dwell section) ────
# The two node-side numbers the MISS teardown is made of. Both are read from
# their sources rather than guessed, and both are pinned by
# test_toss_session.py::test_the_miss_cleanup_floor_is_derived_from_its_sources.
GO_HOME_DURATION_S = 2.0             # trajectory_node's `go_home_duration_s`
                                     # PARAMETER default: the profile _svc_go_home
                                     # installs. _go_home() returns on the service
                                     # ACK, so this whole duration elapses AFTER
                                     # the coordinator has moved on.
NODE_TICK_S = 0.05                   # reload_coordinator_node._TICK_S.

# A MISSED cycle that the session CONTINUES past cannot hand over at
# `dwell_margin_s`: that margin sizes the CAUGHT handoff (a verdict, then two
# ticks — nothing is commanded). The MISS handoff is a whole SAFE_ABORT ladder,
# and every rung of it is dispatched on a service ACK, so `_run_toss_cycle`
# returns while the retract is still descending and the go_home profile is still
# traversing. Measured from the cycle's SCHEDULED landing:
#   CATCH_CONFIRM_WINDOW_S  the settle window before the MISSED verdict is minted
# + GO_HOME_DURATION_S      the recentre profile _safe_abort installs last
# + 2 x NODE_TICK_S         observe-the-terminal + start-the-next-cycle
DEFAULT_SESSION_MISS_CLEANUP_S = (CATCH_CONFIRM_WINDOW_S + GO_HOME_DURATION_S
                                  + 2.0 * NODE_TICK_S)      # 2.80 s

# The MISSED family — the ONLY cycle-failure class stop_on_miss governs. Every
# other non-CAUGHT outcome (REJECTED_*, ABORTED_*) ends the session regardless:
# it is a machine fault, and repeating a fault num_throws times is how one fault
# becomes N.
_MISS_PREFIX = 'MISSED'

# The ONE outcome that can carry success=True. Every other terminal is a
# REJECTED_*/ABORTED_*/STOPPED_ON_MISS, and gating success on this string is what
# stops a vacuously-satisfied count comparison from reporting a rejected goal as
# a succeeded one (see _finish).
OUTCOME_COMPLETED = 'COMPLETED'
OUTCOME_STOPPED_ON_MISS = 'STOPPED_ON_MISS'


@dataclass
class TossSessionResult:
    success: bool
    outcome: str
    throws_completed: int = 0
    catches_confirmed: int = 0
    cycle_outcomes: List[str] = field(default_factory=list)
    cycle_catch_error_mm: List[float] = field(default_factory=list)
    cycle_flight_s: List[float] = field(default_factory=list)
    cycle_dwell_s: List[float] = field(default_factory=list)


@dataclass
class TossSessionDecision:
    phase: str
    action: str = SESSION_ACTION_NONE
    cycle_index: int = 0
    done: bool = False
    result: Optional[TossSessionResult] = None


@dataclass
class TossSessionSequencer:
    """The continuous-session FSM. Construct with the resolved goal parameters,
    then ``start(now)`` and drive with ``step(now)``; after each cycle the node
    reports its outcome through :meth:`note_cycle_result`.

    It reasons about TIME and CYCLE RESULTS only — it takes no observations. Every
    observation-driven decision belongs to the cycle's own ``TossSequencer``, which
    is the whole point: the session cannot second-guess a precondition the single
    toss already owns."""

    num_throws: int
    dwell_time_s: float = 0.0                   # 0 => dwell_default_s
    throw_delay_s: float = 0.0                  # 0 => DEFAULT_TOSS_THROW_DELAY_S;
                                                #   moves the FIRST release only —
                                                #   later releases are set by dwell
    flight_time_s: float = 0.0                  # resolved by the node (height ⇒ T);
                                                #   used only for reporting/deadlines
    stop_on_miss: bool = True                   # operator decision (c). The ctor
                                                #   default matches the ACTION's IDL
                                                #   default; both are load-bearing —
                                                #   an omitted field must mean STOP.
    max_throws: int = DEFAULT_SESSION_MAX_THROWS
    dwell_default_s: float = DEFAULT_SESSION_DWELL_S
    dwell_margin_s: float = DEFAULT_SESSION_DWELL_MARGIN_S
    miss_cleanup_s: float = DEFAULT_SESSION_MISS_CLEANUP_S
                                                #   the floor on landing -> next
                                                #   cycle start for a MISSED cycle
                                                #   the session CONTINUES past.
                                                #   Never shortens a cadence; only
                                                #   lengthens one that would have
                                                #   started inside the previous
                                                #   cycle's own teardown.
    chain_site_reachable: bool = True           # node-fed: for num_throws >= 2, the
                                                #   PREDICTED cycle-2 throw site (the
                                                #   catch centroid of cycle 1, through
                                                #   the SAME predicted_catch_command
                                                #   policy the deferred reach uses)
                                                #   lies inside the ±150 planning box.
                                                #   False ⇒ REJECTED_CHAIN_UNREACHABLE.
                                                #   Default True because a single-cycle
                                                #   session has no chain to check; the
                                                #   node passes it explicitly whenever
                                                #   num_throws >= 2.

    # ── internal state ──
    _phase: str = field(default=SESSION_PHASE_CHECKING, init=False)
    _t_start: float = field(default=0.0, init=False)
    _cycle_index: int = field(default=0, init=False)      # 1-based; 0 = none started
    _cycle_live: bool = field(default=False, init=False)  # S1's guard
    _next_cycle_at: float = field(default=0.0, init=False)
    _last_landing: float = field(default=float('nan'), init=False)
    _throws: int = field(default=0, init=False)
    _catches: int = field(default=0, init=False)
    _outcomes: List[str] = field(default_factory=list, init=False)
    _errors: List[float] = field(default_factory=list, init=False)
    _flights: List[float] = field(default_factory=list, init=False)
    _dwells: List[float] = field(default_factory=list, init=False)
    _stop_outcome: Optional[str] = field(default=None, init=False)
    _finished: bool = field(default=False, init=False)
    _result: Optional[TossSessionResult] = field(default=None, init=False)

    def __post_init__(self):
        # Default-substitution ONLY on exactly 0.0/0 (the goal's "unset" sentinel).
        # NEGATIVE values are preserved so CHECKING refuses them loudly — silently
        # coercing a sign error into a default would run a physically different
        # session, the same doctrine as the toss FSM's __post_init__.
        if self.dwell_time_s == 0.0:
            self.dwell_time_s = float(self.dwell_default_s)
        if self.throw_delay_s == 0.0:
            self.throw_delay_s = DEFAULT_TOSS_THROW_DELAY_S

    # ── derived ────────────────────────────────────────────────────────────────

    @property
    def required_dwell_s(self) -> float:
        """The smallest dwell this session's own ``throw_delay_s`` can honour —
        the release is accept + throw_delay, and the session cannot start cycle
        N+1 before the previous CAUGHT verdict has landed. Derived, never chosen;
        see the module docstring for the two constants and the one measurement."""
        return float(self.throw_delay_s) + float(self.dwell_margin_s)

    @property
    def cycle_index(self) -> int:
        return self._cycle_index

    @property
    def throws_completed(self) -> int:
        """Cycles that got a ball into the air (CAUGHT or MISSED). A cycle
        rejected before release does NOT count — nothing flew."""
        return self._throws

    @property
    def catches_confirmed(self) -> int:
        """Cycles whose announced ball came back CAUGHT (contract C-POSSESS-1's
        ARRIVAL verdict). Exposed live so a watched session is scoreable from
        feedback rather than only at the terminal."""
        return self._catches

    @property
    def cycle_live(self) -> bool:
        return self._cycle_live

    @property
    def finished(self) -> bool:
        return self._finished

    @property
    def phase(self) -> str:
        return self._phase

    # ── driver ─────────────────────────────────────────────────────────────────

    def start(self, now: float) -> None:
        self._phase = SESSION_PHASE_CHECKING
        self._t_start = float(now)
        self._next_cycle_at = float(now)

    def step(self, now: float) -> TossSessionDecision:
        if self._finished:
            # Terminal: replay the stored result so a stray extra step can never
            # hand the consumer a None, and never re-emit START_CYCLE.
            return TossSessionDecision(self._phase, SESSION_ACTION_NONE,
                                       self._cycle_index, True, self._result)

        if self._phase == SESSION_PHASE_CHECKING:
            reject = self._checking_reject()
            if reject is not None:
                return self._finish(reject)
            self._phase = SESSION_PHASE_DWELL

        # A pending stop set by note_cycle_result resolves here, so the terminal
        # is minted on a step (one place) rather than inside the event hook.
        if self._stop_outcome is not None:
            return self._finish(self._stop_outcome)

        # S1: never two live cycles. The node runs a cycle to completion before
        # stepping the session again, so this can only be a caller error — it
        # fails CLOSED (no new cycle) rather than double-owning the hand.
        if self._cycle_live:
            return TossSessionDecision(self._phase, SESSION_ACTION_NONE,
                                       self._cycle_index, False, None)

        if now >= self._next_cycle_at:
            self._cycle_index += 1
            self._cycle_live = True
            return TossSessionDecision(SESSION_PHASE_DWELL,
                                       SESSION_ACTION_START_CYCLE,
                                       self._cycle_index, False, None)
        return TossSessionDecision(SESSION_PHASE_DWELL, SESSION_ACTION_NONE,
                                   self._cycle_index, False, None)

    def _checking_reject(self) -> Optional[str]:
        """Session-level CHECKING, strictest first. Every one of these is a
        pre-throw refusal that moves NOTHING — no cycle has been built, so the
        machine is exactly as the operator left it."""
        if self.num_throws < 1 or self.num_throws > self.max_throws:
            return 'REJECTED_NUM_THROWS'
        if self.throw_delay_s < MIN_TOSS_THROW_DELAY_S:
            # The absolute 4.10 s floor this module, the .action and the runbook
            # all advertise is `MIN_TOSS_THROW_DELAY_S + margin` — so it is only
            # a real floor if the delay itself is gated. Without this, a goal
            # with throw_delay 2.0 / dwell 3.0 satisfies `dwell >= delay +
            # margin` and is ACCEPTED: cycle 1 is fully built (per-goal state
            # installed, the live pose read, the release state computed) and
            # then dies REJECTED_CANT_MAKE_LEAD inside the cycle FSM, naming a
            # field the operator did not think was in play. Refused HERE the
            # verdict names throw_delay_s, which is the field that is wrong.
            return 'REJECTED_THROW_DELAY'
        if not math.isfinite(self.dwell_time_s) or self.dwell_time_s < 0.0:
            return 'REJECTED_DWELL'
        if self.dwell_time_s < self.required_dwell_s:
            # The floor is derived from THIS session's throw_delay (module
            # docstring). Refusing beats silently stretching: a cadence the
            # machine quietly ignores is a lie about what it did, and the
            # operator's remedy — raise dwell, or lower throw_delay toward the
            # 3.5 s FSM floor — is only visible if the number is named.
            return 'REJECTED_DWELL'
        if self.num_throws >= 2 and not self.chain_site_reachable:
            # Phase E's KNOWN LIMITATION, caught BEFORE a ball flies. Without
            # this, a session near the box edge throws one ball, catches it,
            # then refuses cycle 2 REJECTED_WORKSPACE with the platform parked
            # outside the planning box and the ball in the cup — actuation for
            # nothing. Measured frontier: |B| <= 146.5 mm chains, >= 147.0 does
            # not (probe, 2026-07-29).
            return 'REJECTED_CHAIN_UNREACHABLE'
        return None

    # ── discrete event (from the node) ─────────────────────────────────────────

    def note_cycle_result(self, result: TossResult, t_release: float,
                          landing_perf: float) -> None:
        """Consume the cycle the node just ran to its terminal.

        ``t_release`` and ``landing_perf`` are the cycle's own scheduled instants
        (``seq.t_release`` and ``seq.t_release + seq.flight_time_s``) — the
        session schedules the NEXT cycle off the previous SCHEDULED landing, not
        off an observed one, because the observed landing is exactly the quantity
        the tracker is least trustworthy about and a cadence must not inherit
        that noise.

        Ignored once finished, and ignored when no cycle is live (S1's other
        half: a result for a cycle nobody started must not advance the count)."""
        if self._finished or not self._cycle_live:
            return
        self._cycle_live = False
        outcome = str(result.outcome)
        self._outcomes.append(outcome)
        self._errors.append(float(result.catch_error_mm))
        self._flights.append(float(result.achieved_flight_s))
        self._dwells.append(float(t_release) - self._last_landing
                            if math.isfinite(self._last_landing)
                            else float('nan'))

        if bool(result.success):
            self._throws += 1
            self._catches += 1
        elif outcome.startswith(_MISS_PREFIX):
            # A ball flew and was not caught. It counts as a throw.
            self._throws += 1
        else:
            # REJECTED_* / ABORTED_*: nothing flew, or the sequence tore down
            # mid-way. The session ALWAYS stops, whatever stop_on_miss says —
            # and the failing cycle's own verdict is carried verbatim so the
            # operator routes it exactly as they would a single Toss.
            self._stop_outcome = 'ABORTED_CYCLE_{}'.format(outcome)
            return

        self._last_landing = float(landing_perf)
        if not bool(result.success) and self.stop_on_miss:
            # S3: "stopping" is literally not starting cycle N+1. The cycle's own
            # ACTION_SAFE_ABORT has already retracted the hand, lowered the latch
            # and gone home before this hook is reached — there is nothing left
            # to safe, and nothing new is commanded.
            self._stop_outcome = OUTCOME_STOPPED_ON_MISS
            return
        if self._cycle_index >= self.num_throws:
            self._stop_outcome = OUTCOME_COMPLETED
            return
        # Schedule the next cycle so its release lands one dwell past this
        # cycle's landing. If that instant is already past (a handoff that ran
        # long), the next cycle starts immediately and simply reports a longer
        # achieved dwell. Lateness is absorbed; it never aborts.
        next_at = (float(landing_perf) + float(self.dwell_time_s)
                   - float(self.throw_delay_s))
        if not bool(result.success):
            # A MISSED cycle the session is CONTINUING past (stop_on_miss False).
            # Its SAFE_ABORT ladder dispatches the retract and the go_home on
            # SERVICE ACKS, so the node returns here while the hand is still
            # descending and the 2.0 s recentre profile is still traversing —
            # the cleanup does NOT take "however long the node took", which is
            # the assumption the plain dwell arithmetic makes.
            #
            # Without this floor the shipped defaults are already inside the
            # ladder: dwell 6.0 - delay 5.0 puts the next cycle 1.0 s after the
            # landing, 1.7 s before the recentre lands. Cycle N+1 would then
            # read a mid-traverse trajectory/commanded_position as its throw
            # site A, and would meet a hand that the retract has not brought
            # back to the park band — REJECTED_HAND_NOT_PARKED, a machine-fault
            # verdict for a cadence fault, which routes the operator to the
            # wrong subsystem. Both are refusals, not hazards; the fix is to
            # WAIT, which commands nothing and is what the docstring already
            # claimed happened.
            next_at = max(next_at,
                          float(landing_perf) + float(self.miss_cleanup_s))
        self._next_cycle_at = next_at

    # ── terminal ───────────────────────────────────────────────────────────────

    def _finish(self, outcome: str) -> TossSessionDecision:
        # ``success`` requires the COMPLETION terminal as well as the counts.
        # Counts alone are not enough and the hole is not hypothetical: a
        # ``num_throws = 0`` goal is REJECTED_NUM_THROWS with 0 throws and 0
        # catches, so ``throws == num_throws and catches == num_throws`` is
        # VACUOUSLY TRUE and the node would have called ``goal_handle.succeed()``
        # on a rejected goal. Gating on the outcome closes the whole class rather
        # than special-casing zero: no terminal that is not a clean completion
        # can report success, however the counters happen to land.
        success = (outcome == OUTCOME_COMPLETED
                   and self._throws == self.num_throws
                   and self._catches == self.num_throws)
        self._result = TossSessionResult(
            success=success,
            outcome=outcome,
            throws_completed=self._throws,
            catches_confirmed=self._catches,
            cycle_outcomes=list(self._outcomes),
            cycle_catch_error_mm=list(self._errors),
            cycle_flight_s=list(self._flights),
            cycle_dwell_s=list(self._dwells))
        self._finished = True
        return TossSessionDecision(self._phase, SESSION_ACTION_NONE,
                                   self._cycle_index, True, self._result)

    def force_terminal(self, outcome: str) -> TossSessionResult:
        """Terminalise the session from the NODE level (cancel / timeout /
        shutdown / exception), preserving everything scored so far.

        The node owns those exits because they are ROS-level events the FSM
        cannot observe; routing them through here is what keeps the per-cycle
        accounting truthful on an interrupted session instead of returning an
        empty result. Idempotent — the first terminal wins."""
        if self._finished and self._result is not None:
            return self._result
        self._finish(str(outcome))
        return self._result
