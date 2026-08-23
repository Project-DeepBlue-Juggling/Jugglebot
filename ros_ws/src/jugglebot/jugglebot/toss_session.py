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

**S2 is AMENDED, not quietly broken, by the auto-reload interlude (2026-08-11).**
``on_empty_cup: RELOAD`` gives the session exactly one motion-bearing interlude,
and it is bounded three ways so the amendment stays as narrow as the original
invariant made it: (a) it is entered from ``REJECTED_NO_BALL`` ONLY — the one
toss terminal where the cycle FSM provably commanded nothing (minted in CHECKING,
``_terminal_action`` returns ``ACTION_NONE``), so the machine is quiescent;
(b) every rung of it is an EXISTING validated mechanism — ``trajectory/go_home``
and the shipping ``ReloadSequencer``, driven through the node's own
``_step_sequence`` — so the interlude invents no motion primitive; (c) it is
fenced by ``max_reloads`` and by the floor tally. The default is still STOP, and
a goal that omits the field gets STOP (:func:`resolve_on_empty_cup`), so the
pre-2026-08-11 session is bit-unchanged.

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

and the session simply idles until that instant. The floor is the LARGER of a
plumbing term and a physics term, and neither is chosen::

    dwell_floor = max(throw_delay + handoff_margin_s,        # the handoff
                      hand_floor_dwell_s(flight, vel_scale))  # the stroke

    handoff_margin_s = max(dwell_margin_s,                   # verdict exists
                           catch_park_reentry_s(v, scale))   # hand back at park

* ``throw_delay`` is gated by the cycle FSM at ``TOSS_DISPATCH_DEBOUNCE_S``
  (0.10 s, a goal-storm debounce) and, once the release speed is known, at
  ``toss_sequencer.min_throw_delay_for_release_s`` — the Teensy's own ``:642``
  budget for the kind-0 dispatch (``hand_stroke.min_throw_event_delay_s``,
  0.281 s at the 0.80 s nominal flight) **plus the pre-dispatch sequence that
  budget is measured after** (``pre_dispatch_budget_s``: 0.080 s when POSITIONING
  takes the census-B1 skip, 0.460 s when it commands a move). The second term
  landed 2026-08-23. Without it the accept gate was systematically looser than
  the runtime guard it fronts for, and a goal could be ACCEPTED and then abort
  ``ABORTED_CANT_MAKE_RELEASE`` on every cycle — which is what three published
  rungs of ``tests/hardware/session_cadence_ladder.md`` did. Until 2026-08-22
  the gate was ``MIN_TOSS_THROW_DELAY_S`` = 3.5 s, a generic fit over a
  worst-case POSITIONING move a co-located chain never makes; retiring it is
  operator decision 3 of the ILC-primary fold-in.
* ``dwell_margin_s`` covers ONE HALF of the landing → next-cycle-start handoff —
  the verdict half; see the ``handoff_margin_s`` bullet below for the other, and
  note that the floor consumes THAT and not this. It was 0.6 s
  from 2026-07-29 to 2026-08-22, sized on the MOCAP TRACKER's CAUGHT verdict
  (*landing + 0.202–0.442 s*, median 0.209; 17/17 self-tosses of the 2026-07-27
  sitting, ``logbook/2026-07-28-caught-gate-xy-plausibility.md``) plus two node
  ticks. Since the possession verdict became sensor-PRIMARY (C-POSSESS-1,
  2026-08-10) the handoff is the HAND SENSOR's arrival edge, which is a
  different and much faster channel — 0 ms debounce on ``empty→held``, earliest
  observed edge **+137 ms** past the announced landing. **PROVISIONAL at 0.137 s**
  (``ARRIVAL_BAND_MIN_S`` itself — the earliest instant a verdict can EXIST,
  with the old derivation's 2-tick allowance dropped because invariant S1
  already enforces that ordering structurally)
  pending the post-FW14 band re-measure (see ``ball_possession``'s
  ``ARRIVAL_BAND_MAX_S``): the +137…+798 ms band was captured against a
  can-bridge dispatch shift of +54…+133 ms that FW 14 cut to 10–20 ms, so the
  band is expected to collapse and this number to move with it. It is a LOWER
  bound on the verdict and not an upper one, deliberately: the LATEST edge is
  +798 ms, and a margin sized on THAT would re-impose a 0.94 s floor and forbid
  every rung. The late-seat case is protected by the C-POSSESS-1 § 3.4/§ 3.6
  machinery, not by this number.
* ``handoff_margin_s`` is what the floor ACTUALLY uses, and it is
  ``max(dwell_margin_s, hand_stroke.catch_park_reentry_s(v, scale))`` (audit fix,
  2026-08-22). The verdict is only half of what the next cycle's CHECKING needs;
  the other half is ``hand_parked``, and the catch stroke does not bring the hand
  back inside the park band until **+0.190 s** at the R5-prime flight. The
  retired 0.6 s margin covered that by accident and 0.137 s does not, so at every
  cadence rung the bare arrival term would schedule cycle N+1 inside cycle N's
  live catch stroke. See :attr:`TossSessionSequencer.handoff_margin_s`.
* ``hand_floor_dwell_s`` is the C-HAND-1 term, and below ~0.5 s it is the ONLY
  one that binds. Between a landing and the next release the hand must
  decelerate the caught ball to rest at 0 rev, then prelude + gap + wind up the
  next throw — in SERIES, because any kind-0/1/2 command clears the whole packed
  queue (``Teensy_code_platform.ino:648``) and overlapping the two is the
  2026-07-25 clobbered-stroke defect. See
  ``hand_stroke.min_turnaround_dwell_s``.

At the shipped defaults (delay 5.0, margin 0.137, flight 0.80) the floor is
**5.137 s** and the default dwell is 6.0 s, so an all-defaults goal is unchanged.
At the tuning-phase operating point (flight 0.4949 s, ``catch/vel_scale`` 0.9)
the hand floor is **0.4871 s**, which is what makes a 0.49 s dwell — 61
throws/min — the fastest cadence this firmware can be asked for. It clears by
**2.9 ms**; the bench runbook (``tests/hardware/session_cadence_ladder.md``)
logs the per-cycle ``dispatch → catch-stroke-end`` gap for exactly that reason.

**A 0.25 s dwell is not reachable at ANY admitted flight time.** The hand floor
bottoms at 0.2505 s at the very top of the C-HAND-3 band (T = 1.1485 s, apex
1.62 m) and rises from there. Reaching it needs a Platform Teensy flash changing
``calcCatch``'s geometry — deferred by operator decision 3, deliberately not
built here.

A dwell under the floor is REFUSED (``REJECTED_DWELL``), never silently stretched:
a cadence the machine quietly ignores is a lie about what it did. Lateness in the
other direction is absorbed — a cycle whose handoff ran long simply reports a
longer achieved dwell in ``per_cycle_dwell_s`` and never aborts.

### The MISS path needs a bigger floor than the CAUGHT one, and gets it here

``handoff_margin_s`` sizes the CAUGHT handoff, where nothing is commanded: a
verdict lands and the hand finishes its catch stroke back at the park band. A MISSED cycle the session CONTINUES past
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

**The floor belongs to the SAFE_ABORT LADDER, not to the MISS outcome**, and every
continuation past that ladder gets it — there are three, and all three reuse the
one constant so they cannot drift apart:

1. the continued MISS (above);
2. the single ``ABORTED_NO_RELEASE`` retry (audit fix, 2026-08-11). A non-release
   terminates in ``PHASE_THROWING`` with the platform positioned and the latch
   raised, so its terminal action is the identical ``ACTION_SAFE_ABORT``. It had
   been returning from :meth:`note_cycle_result` without rescheduling at all,
   leaving ``_next_cycle_at`` at the previous cycle's already-past instant — so
   the retry started on the very next tick, inside its own teardown. Beyond the
   two refusals above, that had a second cost specific to this branch: the retry
   would normally die ``REJECTED_HAND_NOT_PARKED``, which is NOT
   ``ABORTED_NO_RELEASE``, so the "two consecutive non-releases stop the session"
   gauge could never fire;
3. the reload interlude's rung 4 (:meth:`_settle_after_reload` in the node), which
   spends the same floor as a blocking wait because the interlude runs inside the
   node rather than across FSM ticks.

## Former known limitation (dissolved 2026-08-14) — chaining near the box edge

A catch parks the platform CENTROID slightly outside B so the CUP lands ON B, and
``trajectory/commanded_position`` publishes the CENTROID. For a fixed-B session the
offset is largest at cycle 2 and then collapses (measured through the production
``predicted_catch_command``, 2026-07-29, B on the +x axis, at the then-hardcoded
±150 box):

    B_x     cycle-2 A_x     cycle-3 A_x     cycle-4 A_x
     70.0        71.448          69.970         70.001
    140.0       142.894         139.940        140.001
    146.0       149.017         145.938        146.001
    147.0       150.038   ← outside a ±150 planning box

So a fixed-B chain CONVERGES, and the only cycle at risk is **cycle 2**. At box =
cap = 150 the frontier was sharp: |B| ≤ 146.5 mm chained, |B| ≥ 147.0 mm did not
(the binding gate being the box on A, NOT the displacement cap — the residual
|B−A| never exceeds 3.1 mm). **Since 2026-08-14 the box is the config key
``toss_workspace_xy_mm`` (shipped 160 > cap × 1.03), the cycle-2 centroid sits
inside it at every valid B, and cap-edge chains are admitted.** The coordinator
still pre-checks against the configured box before anything moves
(``REJECTED_CHAIN_UNREACHABLE``) rather than letting the session throw one ball,
catch it, and then refuse cycle 2 with the platform parked off-box and a ball in
the cup — the refusal re-binds only if the box is set below cap × ~1.03.

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

from jugglebot.ball_possession import ARRIVAL_BAND_MIN_S
from jugglebot.motion.trajectory import hand_stroke, throw_envelope
from jugglebot.toss_sequencer import (
    CATCH_CONFIRM_WINDOW_S,
    DEFAULT_TOSS_THROW_DELAY_S,
    FLIGHT_TIME_MIN_S,
    NODE_TICK_S,
    TOSS_DISPATCH_DEBOUNCE_S,
    TossResult,
    min_throw_delay_for_release_s,
    pre_dispatch_budget_s,
    vertical_event_vel_mps,
)

# ── Feedback phases (TossContinuous.action feedback.phase — LOCKED strings) ────
# While a cycle is live the node reports the CYCLE's Toss phase verbatim; these
# three are the session's own.
SESSION_PHASE_CHECKING = 'SESSION_CHECKING'
SESSION_PHASE_DWELL = 'DWELL'
SESSION_PHASE_RELOAD = 'RELOAD'      # the auto-reload interlude (§ 3.9). Additive:
                                     # an existing consumer that only knows the
                                     # first two sees a phase it does not
                                     # recognise, never a phase that changed
                                     # meaning.

# ── Actions the node executes on the session's behalf ─────────────────────────
SESSION_ACTION_NONE = 'none'
SESSION_ACTION_START_CYCLE = 'start_cycle'   # build + run ONE TossSequencer, then
                                             #   feed its result back through
                                             #   note_cycle_result. Never emitted
                                             #   while a cycle is outstanding (S1).
SESSION_ACTION_RELOAD = 'reload'             # run the reload interlude (§ 3.9),
                                             #   then report through
                                             #   note_reload_result. Emitted ONLY
                                             #   after a REJECTED_NO_BALL cycle,
                                             #   i.e. from a state where the toss
                                             #   FSM provably commanded NOTHING.

# ── on_empty_cup (TossContinuous.action) ──────────────────────────────────────
ON_EMPTY_CUP_STOP = 'STOP'
ON_EMPTY_CUP_RELOAD = 'RELOAD'


def resolve_on_empty_cup(raw) -> str:
    """Goal field -> the resolved policy. ANYTHING that is not exactly ``RELOAD``
    resolves to ``STOP``.

    The same doctrine ``stop_on_miss`` carries, and it is load-bearing for the
    same reason: an omitted, empty, misspelt or older-client field must never
    start an autonomous BB reload the operator did not ask for. Whitelisting the
    one dangerous value (rather than blacklisting the safe one) is what makes a
    typo fail in the safe direction."""
    try:
        text = str(raw or '').strip().upper()
    except Exception:                                          # noqa: BLE001
        return ON_EMPTY_CUP_STOP
    return ON_EMPTY_CUP_RELOAD if text == ON_EMPTY_CUP_RELOAD else ON_EMPTY_CUP_STOP

# ── Defaults / floors (the NO-CONFIG fallbacks only) ──────────────────────────
# The node resolves the generated JB_OP_TOSS_SESSION_* keys and passes them into
# the ctor; these literals serve standalone/test use and the config drift-guard
# test pins each pair equal — the same pattern as the toss FSM's
# DEFAULT_TOSS_FLIGHT_TIME_S.
DEFAULT_SESSION_DWELL_S = 6.0        # 0 => this. Comfortably over the 5.137 s
                                     # floor at the 5.0 s default throw delay, so
                                     # the default combination is legal without the
                                     # operator doing arithmetic. DELIBERATELY NOT
                                     # lowered with the floor on 2026-08-22: a
                                     # DEFAULT must never jump cadence. The ladder
                                     # (tests/hardware/session_cadence_ladder.md)
                                     # selects a faster dwell EXPLICITLY, per goal,
                                     # one rung at a time.
DEFAULT_SESSION_DWELL_MARGIN_S = ARRIVAL_BAND_MIN_S
                                     # = 0.137 s. ⚠ PROVISIONAL (census A3,
                                     # 2026-08-22) — the landing -> next-cycle-
                                     # start handoff, re-based on the HAND
                                     # SENSOR's arrival edge.
                                     #
                                     # It is the earliest instant a possession
                                     # verdict for cycle N can EXIST, and nothing
                                     # else. The old 0.6 s was 0.442 (worst MOCAP
                                     # TRACKER CAUGHT-verdict latency, 17/17,
                                     # 2026-07-27 sitting) + 2 x 0.05 s node ticks
                                     # = 0.542, rounded up. TWO things changed:
                                     #
                                     #   * the CHANNEL. Possession went
                                     #     sensor-PRIMARY on 2026-08-10
                                     #     (C-POSSESS-1); the tracker is the
                                     #     FALLBACK. The sensor's empty->held edge
                                     #     carries ZERO debounce (measured 0/0/0
                                     #     ms, against 232/241/295 ms on the
                                     #     falling edge) and its earliest observed
                                     #     edge is +137 ms (ARRIVAL_BAND_MIN_S).
                                     #   * the TICK ALLOWANCE is GONE, because it
                                     #     was double-counting a guarantee the FSM
                                     #     already makes structurally. Invariant
                                     #     S1 forbids emitting START_CYCLE until
                                     #     note_cycle_result has consumed the
                                     #     previous cycle, so "the tick that
                                     #     observes the verdict and the tick that
                                     #     starts the next cycle" cannot be
                                     #     skipped whatever this number says. And
                                     #     lateness is absorbed by design: a cycle
                                     #     whose handoff ran long reports a longer
                                     #     achieved dwell and never aborts. A
                                     #     margin that budgets for it buys nothing
                                     #     and costs 40 ms of the tightest rung.
                                     #
                                     # It is a LOWER bound on the verdict and not
                                     # an upper one, deliberately: the LATEST edge
                                     # is +798 ms, and a margin sized on THAT would
                                     # re-impose a 0.94 s floor and forbid every
                                     # cadence rung. What protects the late-seat
                                     # case is not this number but the possession
                                     # machinery landed 2026-08-21 (C-POSSESS-1
                                     # § 3.4/§ 3.6: the dwell-clamped retention and
                                     # arrival windows, the raw-bit live evidence
                                     # read, and the interlude's seat-edge band
                                     # wait).
                                     #
                                     # PROVISIONAL because ARRIVAL_BAND_MIN_S is
                                     # itself pending a re-measure: the band was
                                     # captured against a can-bridge dispatch shift
                                     # of +54..+133 ms that FW 14 cut to 10-20 ms,
                                     # so it should collapse and this number move
                                     # with it. Deriving it from the band constant
                                     # is what makes that automatic — exactly what
                                     # ARRIVAL_BAND_MAX_S's own comment promises.
                                     #
                                     # Lowering it does NOT speed any shipped goal
                                     # up: it is a FLOOR term, and the DEFAULT
                                     # dwell (DEFAULT_SESSION_DWELL_S, still 6.0)
                                     # is what sets cadence.
DEFAULT_CATCH_VEL_SCALE = 0.9        # 0 => this. The catch-speed knob's shipped
                                     # default (hw.JB_OP_CATCH_VEL_SCALE_DEFAULT,
                                     # pinned equal by the config drift-guard test).
                                     # It enters the DWELL FLOOR, not just the
                                     # catch: the tail is inversely proportional to
                                     # the armed speed, so 0.9 is 11% more tail than
                                     # a scale of 1.0 and 0.3 (the _VEL_SCALE_MIN
                                     # floor) is 3x.
DEFAULT_SESSION_MAX_THROWS = 20      # upper bound on num_throws. An unbounded
                                     # session is a machine stroking unattended for
                                     # an unbounded time; 20 cycles at the 6.0 s
                                     # default dwell is ~2.3 minutes, well past any
                                     # rung of the hardware ladder.
DEFAULT_SESSION_MAX_RELOADS = 3      # goal max_reloads 0 => this. The ONLY
                                     # machine-side fence on ball supply (there is
                                     # no ball-count or magazine field anywhere on
                                     # ball_butler_node, so supply has no machine
                                     # observability and none is invented).
DEFAULT_SESSION_FLOOR_PAUSE_EVERY = 5  # balls on the floor before the session stops
                                     # cleanly so the operator can clear them. 0
                                     # disables. At max_reloads 3 the budget binds
                                     # first — this is the fence for a long-budget
                                     # session, not a routine gate.

# ── The MISS-path cleanup floor (see the module docstring's dwell section) ────
# The two node-side numbers the MISS teardown is made of. Both are read from
# their sources rather than guessed, and both are pinned by
# test_toss_session.py::test_the_miss_cleanup_floor_is_derived_from_its_sources.
GO_HOME_DURATION_S = 2.0             # trajectory_node's `go_home_duration_s`
                                     # PARAMETER default: the profile _svc_go_home
                                     # installs. _go_home() returns on the service
                                     # ACK, so this whole duration elapses AFTER
                                     # the coordinator has moved on.
# NODE_TICK_S — reload_coordinator_node._TICK_S. Was 0.05 until 2026-08-22
# (census B3). The FSMs are time-driven, so this bounds LATENCY, not correctness
# — but at a 0.49 s dwell a 0.05 s tick is 10% of the whole turnaround, spent in
# sleep(). 0.02 s is still two orders of magnitude above localhost topic latency,
# so every TICK-COUNTED ordering gap keeps its guarantee. The tick COUNTS did NOT
# change: collapsing them is what would break the two load-bearing cross-topic
# gaps (prime_hold before the armed edge; armed-confirm before the announcement).
#
# DEFINED IN toss_sequencer SINCE 2026-08-23 and re-exported here (this module's
# importers, including tests, read `toss_session.NODE_TICK_S`). It moved because
# it stopped being a latency note and became ARITHMETIC: `pre_dispatch_budget_s`
# counts the pre-dispatch sequence in ticks, and that function is what BOTH delay
# gates now charge — so the tick and the gates have to live in one module or the
# accept floor can be re-based by an edit that never mentions it.

#: The ILC's ``event_vel_trim`` outer ceiling (``k_v − 1``), RESTATED from
#: ``motion/toss_ilc.ILC_SPEED_AUTHORITY`` and pinned equal to it by
#: ``test_toss_session.py``.  Restated rather than imported for the reason the
#: whole module is: ``toss_ilc`` pulls numpy + yaml + the artifact loader, and
#: this FSM stays a standalone-importable pure-Python file.  Read by
#: :attr:`TossSessionSequencer.floor_event_vel_mps`.
ILC_SPEED_AUTHORITY = 0.15

# A MISSED cycle that the session CONTINUES past cannot hand over at
# `handoff_margin_s`: that margin sizes the CAUGHT handoff (a verdict lands, the
# catch stroke finishes — nothing is commanded). The MISS handoff is a whole SAFE_ABORT ladder,
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

# ── The reload interlude's terminals (§ 3.9) ──────────────────────────────────
# Every one of them STOPS the session, and every one names WHICH rung refused, so
# the operator routes the failure without reading a log. The two below are the
# FSM's own (it owns the counters); the node owns the observation-driven rest
# (STOPPED_BB_NOT_READY / STOPPED_BB_UNVERIFIED / STOPPED_SENSOR_UNKNOWN /
# STOPPED_CUP_NOT_EMPTY / STOPPED_BALL_EVIDENCE_DISABLED / STOPPED_RECENTRE_FAILED)
# and hands them back through :meth:`note_reload_result`.
OUTCOME_STOPPED_RELOAD_BUDGET = 'STOPPED_RELOAD_BUDGET'
OUTCOME_STOPPED_FLOOR_CLEAR_REQUIRED = 'STOPPED_FLOOR_CLEAR_REQUIRED'
#: Fallback when the node reports a failed interlude with no code of its own —
#: never expected, and it still STOPS, because an unnamed failure is still a
#: failure and continuing would stroke over a cup nobody proved has a ball.
OUTCOME_STOPPED_RELOAD_FAILED = 'STOPPED_RELOAD_FAILED'

# The ONE cycle terminal the interlude is entered from. Chosen because it is the
# only toss terminal where the FSM provably commanded NOTHING: it is minted in
# CHECKING, before `_positioned` or `_prepare_dispatched`, so `_terminal_action`
# returns ACTION_NONE and the machine is quiescent when the interlude starts.
RELOAD_TRIGGER_OUTCOME = 'REJECTED_NO_BALL'

# The terminal the single retry is gated on (operator decision 6, 2026-08-10).
NO_RELEASE_OUTCOME = 'ABORTED_NO_RELEASE'
#: Two CONSECUTIVE ABORTED_NO_RELEASE stop the session. One is a stroke that did
#: not release with the ball demonstrably still in the cup — retryable. Two in a
#: row is a plant fault repeating, and repeating a fault num_throws times is how
#: one fault becomes N (the epidemic gauge the whole abort ladder is built on).
NO_RELEASE_MAX_CONSECUTIVE = 2
#: The sensor state that licenses the retry. Deliberately the STRING, not an
#: import of ``ball_possession.EVIDENCE_SEATED``: this module is pure and takes
#: the evidence as a caller-supplied observation, and a drift-guard test pins the
#: two equal rather than a runtime import doing it silently.
EVIDENCE_SEATED_NAME = 'SEATED'


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
    reloads_used: int = 0


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
    flight_time_s: float = 0.0                  # resolved by the node (height ⇒ T).
                                                #   Reporting + deadlines, and since
                                                #   2026-08-22 the HAND-GEOMETRY dwell
                                                #   floor: the catch tail and the throw
                                                #   windup are both f(release speed),
                                                #   and the release speed is f(T). 0.0
                                                #   ⇒ judged at the C-HAND-3 band FLOOR
                                                #   (the strictest case) — see
                                                #   hand_floor_dwell_s.
    catch_vel_scale: float = 0.0                # this session's catch/vel_scale knob;
                                                #   0 ⇒ JB_OP_CATCH_VEL_SCALE_DEFAULT
                                                #   (0.9), which the node resolves and
                                                #   passes in. It belongs to the FLOOR
                                                #   because the catch is armed at
                                                #   event_vel x scale and the catch
                                                #   tail is inversely proportional to
                                                #   it: a SLOWER catch (a smaller
                                                #   scale) makes the turnaround
                                                #   LONGER, so a session that ignored
                                                #   it would under-state its own floor
                                                #   for exactly the operator setting
                                                #   most likely to be reached for when
                                                #   catches are being missed.
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
                                                #   lies inside the CONFIGURED
                                                #   toss_workspace_xy_mm box (160
                                                #   shipped 2026-08-14; was ±150).
                                                #   False ⇒ REJECTED_CHAIN_UNREACHABLE.
                                                #   Default True because a single-cycle
                                                #   session has no chain to check; the
                                                #   node passes it explicitly whenever
                                                #   num_throws >= 2.
    on_empty_cup: str = ON_EMPTY_CUP_STOP       # STOP (default) | RELOAD. The ctor
                                                #   default matches the ACTION's IDL
                                                #   default and the node re-resolves
                                                #   the raw field through
                                                #   resolve_on_empty_cup, so a value
                                                #   that is not exactly RELOAD can
                                                #   never reach here as RELOAD.
    max_reloads: int = DEFAULT_SESSION_MAX_RELOADS
    floor_pause_every: int = DEFAULT_SESSION_FLOOR_PAUSE_EVERY
    ilc_speed_trim_possible: bool = True        # can layer 3 command a speed trim
                                                #   on this goal (ILC enabled AND an
                                                #   artifact loaded)? It is the only
                                                #   thing that can make the cycle's
                                                #   event_vel differ from the
                                                #   untrimmed vertical closed form
                                                #   this session computes its floors
                                                #   from — and the delay floor RISES
                                                #   as the speed FALLS, so a negative
                                                #   trim raised the floor AFTER the
                                                #   session had accepted the goal
                                                #   (audit finding, 2026-08-22).
                                                #   See floor_event_vel_mps.
                                                #
                                                #   Default TRUE = FAIL-CLOSED: a
                                                #   session that was not told judges
                                                #   itself against the slowest
                                                #   release the apply seam could
                                                #   command. It costs nothing at the
                                                #   cadence rungs, where the throw
                                                #   envelope refuses the negative
                                                #   side outright.

    # ── internal state ──
    _phase: str = field(default=SESSION_PHASE_CHECKING, init=False)
    _t_start: float = field(default=0.0, init=False)
    _cycle_index: int = field(default=0, init=False)      # 1-based; 0 = none started
    _cycle_live: bool = field(default=False, init=False)  # S1's guard
    _next_cycle_at: float = field(default=0.0, init=False)
    _last_landing: float = field(default=float('nan'), init=False)
    _throws: int = field(default=0, init=False)
    _catches: int = field(default=0, init=False)
    _reloads_used: int = field(default=0, init=False)
    _floor_balls: int = field(default=0, init=False)
    _reload_pending: bool = field(default=False, init=False)
    _no_release_streak: int = field(default=0, init=False)
    # Flags the NEXT cycle inherits, latched here and CONSUMED at START_CYCLE so
    # exactly one cycle wears each (guards G10 / G11 depend on that: a flag that
    # leaked to a second cycle would exclude a clean toss from every fit).
    _retry_next: bool = field(default=False, init=False)
    _reload_settle_next: bool = field(default=False, init=False)
    _cycle_is_retry: bool = field(default=False, init=False)
    _cycle_reload_settle: bool = field(default=False, init=False)
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
        if self.catch_vel_scale == 0.0:
            self.catch_vel_scale = DEFAULT_CATCH_VEL_SCALE

    # ── derived ────────────────────────────────────────────────────────────────

    @property
    def _flight_or_floor_s(self) -> float:
        """``flight_time_s``, or the C-HAND-3 band FLOOR when nothing resolved it.

        ONE fallback for all three derived floors below, because every one of
        them is monotonically DECREASING in flight time: the shortest admitted
        flight has the largest floor, so an un-resolved session is judged against
        the strictest case it could be. Fail-closed, the same doctrine as
        ``throw_site_known``. Single-sourced so the three cannot drift apart —
        a session judged against 0.4949 s by one floor and 0.80 s by another is
        a floor combination no flight time produces."""
        t = float(self.flight_time_s)
        if not (math.isfinite(t) and t > 0.0):
            return float(FLIGHT_TIME_MIN_S)
        return t

    @property
    def floor_event_vel_mps(self) -> float:
        """The SLOWEST release this goal's cycles could actually be commanded at.

        Every derived floor on this class is monotonically DECREASING in release
        speed (a slower throw has a longer windup, so it needs more lead and more
        dwell), which makes the slowest admissible speed the fail-closed one to
        judge against.

        Without layer 3 that is just ``vertical_event_vel_mps(T)`` — the same
        closed form the cycle FSM resolves ``event_vel_mps`` with. **With layer 3
        armed it is not**, and that was the second half of the 2026-08-22 audit's
        BLOCKING finding: the session computed its floors from the UNTRIMMED
        speed while the FSM received ``v · (1 + ilc_vel_trim)``, so a NEGATIVE
        trim raised the cycle's floor after the session had already accepted the
        goal — a REJECTED_CANT_MAKE_LEAD on cycle 1 of a session the operator was
        told was legal.

        The bound is derived, not assumed: the trim is applied through
        ``reload_coordinator_node._ilc_vel_trim_refusal``, which REFUSES any trim
        whose commanded speed breaks ``throw_envelope.evaluate`` (contract
        C-HAND-3) — so the slowest speed that can actually fly is the smallest
        ``v · (1 + t)``, ``t ∈ [−ILC_SPEED_AUTHORITY, 0]``, that the envelope
        admits.  Bisected here because the envelope's negative-side bound is
        strongly flight-dependent and has no closed form.

        **It costs nothing where the cadence lives.** At the ladder's flights the
        envelope's ``ARM_WINDOW`` term refuses the negative side almost entirely
        (0.0 mm/s of headroom at the 0.4949 s band floor, 0.21 m/s at 0.5029 s),
        so this returns the untrimmed speed or within a whisker of it; the charge
        only becomes visible at the long flights where the delay is seconds clear
        of every floor anyway.  The bridge's own [0.3, 7.0] m/s wire band is two
        orders of magnitude away from binding here and is not re-checked."""
        nominal = vertical_event_vel_mps(self._flight_or_floor_s)
        if not self.ilc_speed_trim_possible:
            return nominal
        flight = self._flight_or_floor_s
        lo = nominal * (1.0 - ILC_SPEED_AUTHORITY)
        if throw_envelope.evaluate(flight, lo).ok:
            return lo
        hi = nominal
        # Monotone: the ARM_WINDOW term that refuses a slow release only relaxes
        # as the release speeds up, so a bisection lands on the exact frontier.
        for _ in range(48):
            mid = 0.5 * (lo + hi)
            if throw_envelope.evaluate(flight, mid).ok:
                hi = mid
            else:
                lo = mid
        return hi

    @property
    def min_throw_delay_s(self) -> float:
        """The session's ``throw_delay_s`` floor — THE cadence this build can hold.

        One derivation, shared with the cycle FSM's own CHECKING gate
        (:func:`toss_sequencer.min_throw_delay_for_release_s`):
        ``max(TOSS_DISPATCH_DEBOUNCE_S, kind-0 dispatch budget + pre-dispatch
        sequence)``.  Until 2026-08-23 it charged the dispatch budget alone, which
        is the floor the RUNTIME guard measures the *remaining* lead against — so
        the session admitted delays whose cycles then aborted
        ``ABORTED_CANT_MAKE_RELEASE`` every time (the whole reason three published
        cadence rungs never threw a ball).

        **Which pre-dispatch budget: the STEADY STATE, deliberately.** This gate
        charges ``positioning_move=False`` — the chained cycle, where the platform
        is already holding the pose the previous cycle threw and caught from and
        POSITIONING takes the census-B1 no-op skip.  That is not optimism; it is
        what the session's number MEANS.  ``throw_delay_s`` is the operator's
        cadence parameter (``required_dwell_s`` is ``throw_delay +
        handoff_margin``), a cadence is a steady state, and charging the 0.460 s
        moving budget here would refuse every cadence above ~40 throws/min for a
        cost only the FIRST cycle of a sitting ever pays.

        The first cycle is not left to abort:

        * the CYCLE's own gate charges the REAL per-cycle predicate (the node
          feeds ``positioning_move_expected`` from the same cached B1 decision
          that drives POSITIONING), so a cycle that must move and cannot make its
          lead dies ``REJECTED_CANT_MAKE_LEAD`` at CHECKING — loud, pre-throw,
          nothing moved — and never ``ABORTED_CANT_MAKE_RELEASE`` mid-sequence;
        * and the node RAISES that first cycle's ``throw_delay`` to the moving
          floor (``_build_toss_cycle``, one WARN line), so the ordinary case is
          "cycle 1 releases ~0.38 s later than the metronome implies, then the
          landing-anchored schedule takes over" rather than a refusal.

        The speed is :attr:`floor_event_vel_mps`, not the untrimmed closed form —
        see it for the ILC negative-trim half of the same finding.  The
        flight-time fallback is the shared fail-closed one
        (:attr:`_flight_or_floor_s`): the event-delay floor RISES as the flight
        shortens (0.281 s at T = 0.80, 0.337 s at the band floor)."""
        return min_throw_delay_for_release_s(self.floor_event_vel_mps,
                                             positioning_move=False)

    @property
    def hand_floor_dwell_s(self) -> float:
        """The C-HAND-1 dwell floor: catch tail + prelude + gap + throw windup.

        THE physics term, and below ~0.5 s the only one that binds (census § 0).
        Delegates to ``hand_stroke.min_turnaround_dwell_s`` — see it for why the
        four terms are additive rather than overlappable, and for the verified
        finding that the catch profile ends at ``t7`` and not at
        ``t8 = t7 + END_PROFILE_HOLD`` (which would add 0.10 s to every number
        here and make the 0.49 s operating point infeasible).

        The release speed comes from ``toss_sequencer.vertical_event_vel_mps`` —
        the SAME closed form the cycle FSM resolves its own ``event_vel_mps``
        with, so the session cannot refuse a cadence the cycle can make.

        **UNTRIMMED, unlike the two floors either side of it** (2026-08-23), and
        the asymmetry is deliberate. This is THE named C-HAND-1 geometry number:
        the census, the cadence runbook's § 0 table and
        ``ros_ws/docs/hand_decel_feedforward.md`` all quote it per flight time,
        and re-basing it onto a speed that depends on whether an ILC artifact
        happens to be loaded would make "the hand floor at T = 0.5029 s" two
        different numbers on one machine. It is safe to leave nominal because it
        is **not the binding term at any admitted flight**: since the delay floor
        grew the pre-dispatch sequence, ``throw_delay + handoff_margin`` exceeds
        this by **0.1230 s at its worst** across the whole C-HAND-3 band (probe,
        2026-08-23), which covers this term's entire **0.0715 s** worst-case
        sensitivity to a maximal negative speed trim with 1.7x margin.
        ``test_the_hand_floor_is_dominated_by_the_plumbing_term``
        pins that dominance, so if the plumbing term ever shrinks back under it
        the argument is re-taken rather than silently relied on.

        ``flight_time_s`` is 0.0 when nothing resolved it (standalone/test
        construction). The fallback is the C-HAND-3 band FLOOR, not the nominal
        0.80 s default, because the floor is monotonically DECREASING in flight
        time: the shortest admitted flight has the largest hand floor, so an
        un-resolved session is judged against the strictest case it could be.
        Fail-closed, the same doctrine as ``throw_site_known``."""
        return hand_stroke.min_turnaround_dwell_s(
            vertical_event_vel_mps(self._flight_or_floor_s),
            float(self.catch_vel_scale))

    @property
    def handoff_margin_s(self) -> float:
        """THE landing → next-cycle-CHECKING floor: the LARGER of the possession
        verdict's earliest instant and the hand's return to the park band.

        Two independent things have to be true before cycle N+1's CHECKING can
        pass, and until 2026-08-22 only one of them was modelled:

        * a possession VERDICT for cycle N has to exist — ``dwell_margin_s``,
          re-based onto ``ARRIVAL_BAND_MIN_S`` (0.137 s) when the channel went
          sensor-PRIMARY. Correct about the quantity it models.
        * the HAND has to be back inside the park band — ``hand_parked`` is a
          hard CHECKING precondition (a kind-0 stroke commands absolute positions
          from 0 rev), and the catch stroke is still running well past +137 ms:
          ``hand_stroke.catch_park_reentry_s`` is **0.1903 s** at the R5-prime
          flight, 0.1582 at R4's, 0.1204 at R0–R3's.

        **Why this is a separate property and not a bigger literal** (audit fix,
        2026-08-22). The retired 0.6 s ``DEFAULT_SESSION_DWELL_MARGIN_S`` covered
        the park re-entry by accident — 0.6 s is past every value the geometry
        can produce — so nothing ever had to name it. Re-basing the margin onto
        the arrival band removed the accident without replacing it, and at the
        cadence rungs the gap is decisive: R5-prime schedules cycle N+1 at
        ``landing + 0.140`` against a hand that re-enters the band at +0.190,
        i.e. 50 ms INSIDE the live catch stroke. CHECKING then reads ~1.5 rev and
        mints ``REJECTED_HAND_NOT_PARKED`` on a healthy catch — a machine-fault
        verdict for a cadence fault, which ends the sitting and routes the
        operator to the wrong subsystem.

        The only thing standing between that schedule and that verdict today is
        that the cycle terminal still waits for a TRACKER ``CAUGHT`` at
        *landing + 0.202…0.442 s* — the FALLBACK channel this same fold-in
        demoted, and whose 0.442 s figure it retired. An accidental fence, 12 ms
        wide at R4/R5, that disappears the moment the sensor alone is allowed to
        terminate a cycle. Derived, so it cannot drift away again.

        The speed is :attr:`floor_event_vel_mps` (2026-08-23): the catch is armed
        for the ball the throw put up, so a layer-3 speed trim moves the park
        re-entry with it, and the fail-closed value is the slow one."""
        return max(float(self.dwell_margin_s),
                   hand_stroke.catch_park_reentry_s(
                       self.floor_event_vel_mps,
                       float(self.catch_vel_scale)))

    @property
    def required_dwell_s(self) -> float:
        """The smallest dwell this session can honour — the LARGER of the
        plumbing handoff and the hand's own geometry.

        The plumbing term is ``throw_delay + handoff_margin``: the release is
        accept + throw_delay, and the session cannot start cycle N+1 before the
        previous cycle's possession verdict has landed AND the hand has come back
        to the park band (see :attr:`handoff_margin_s` — the second half was
        unmodelled until 2026-08-22). The physics term is
        :attr:`hand_floor_dwell_s`.

        **Why the max() and not just the plumbing term.** Until 2026-08-22 the
        plumbing term alone was the floor, and it was safe only by accident: at
        ``MIN_TOSS_THROW_DELAY_S`` = 3.5 s it evaluated to 4.10 s, an order of
        magnitude above anything the hand could not make, so the physics never
        had to be consulted. Retiring that constant (operator decision 3) removes
        the accident. Without this term a goal with ``throw_delay 0.30 / dwell
        0.45`` at the 0.4949 s band floor satisfies ``dwell >= delay + margin``,
        is ACCEPTED, and then dispatches cycle N+1's kind-0 throw INSIDE cycle
        N's live catch stroke — which clears the packed queue and reseeds the
        prelude from an encoder reading taken at 41-96 rev/s. That is the
        2026-07-25 defect exactly (hand overshot to 10.17-10.33 rev, then was
        yanked 0.34-1.75 rev below x3), and it is a hardware event, not a
        refusal. Derived, never chosen; see the module docstring."""
        return max(float(self.throw_delay_s) + self.handoff_margin_s,
                   self.hand_floor_dwell_s)

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
    def reloads_used(self) -> int:
        return self._reloads_used

    @property
    def reload_budget_remaining(self) -> int:
        """Attempts the interlude may still spend. The node runs the BB
        not-positioned-in-time retry INSIDE one interlude, so it needs the
        remaining budget — the retry is not free, it is the same fence."""
        return max(0, int(self.max_reloads) - self._reloads_used)

    @property
    def floor_balls(self) -> int:
        """Balls this session has put on the floor — its own count of reload
        interludes entered. Nothing on the robot can see the floor; this is a
        tally, not a measurement, and it is named that way on purpose."""
        return self._floor_balls

    @property
    def intends_another_cycle(self) -> bool:
        """Does this session INTEND to throw again after the cycle it is about to
        start? Read by the node to decide whether the sensor's windows are
        clamped by a next scheduled release (C-POSSESS-1 § 3.4).

        "Intends", not "will": whether cycle N+1 actually runs also depends on
        the outcome (``stop_on_miss``, a reject, a cancel), and none of that is
        knowable at cycle N's start. The approximation is deliberately in the
        UNCLAMPED direction on the boundary — a session that stops early leaves
        one cycle whose retention window was clamped against a release that never
        came, which costs a late bounce-out reading UNKNOWN instead of REJECTED
        on that one cycle. Clamping one cycle too FEW is the cheap error;
        clamping one too MANY on every cycle is the census-D1 inversion.

        Keyed on THROWS for the same reason ``step``'s completion test is: a
        REJECTED_NO_BALL cycle and the single ABORTED_NO_RELEASE retry cost a
        cycle index without costing one of the ``num_throws`` data points."""
        return int(self._throws) + 1 < int(self.num_throws)

    @property
    def last_landing_perf(self) -> float:
        """The previous cycle's SCHEDULED landing on the perf clock, or NaN
        before any cycle has reported. The auto-reload interlude needs it to know
        when the seat-edge band has actually elapsed (C-POSSESS-1 § 3.6): a
        cup read taken before it is a read of a catch still in progress."""
        return float(self._last_landing)

    @property
    def next_cycle_at(self) -> float:
        """The instant the next cycle is scheduled to start. Read by the node's
        Layer-1.5 dwell reads to size their own budget — the covariate must fit
        inside the quiescent window with room, never push against it."""
        return self._next_cycle_at

    @property
    def cycle_is_retry(self) -> bool:
        """True while the LIVE cycle is the single ABORTED_NO_RELEASE retry.
        Consumed by the node to stamp ``retry_of`` on the record (guard G11)."""
        return self._cycle_is_retry

    @property
    def cycle_reload_settle(self) -> bool:
        """True while the LIVE cycle is the first one after a reload interlude.
        Stamps ``reload_settle`` on the record, which excludes the cycle from
        every fit (guard G10): the platform has just been recentred and the ball
        has just been re-seated by a BB throw, so its arrival state is not the
        steady-state one the map is being fitted from."""
        return self._cycle_reload_settle

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

        # The reload interlude, before any cycle can start. Emitted from the
        # REJECTED_NO_BALL terminal only, and the budget/floor gates have already
        # passed inside note_cycle_result — so reaching here means the SESSION's
        # own preconditions hold and only the node's observation-driven ones are
        # left. Re-emitted every step until note_reload_result answers, exactly
        # as START_CYCLE is not re-emitted while a cycle is live.
        if self._reload_pending:
            self._phase = SESSION_PHASE_RELOAD
            return TossSessionDecision(SESSION_PHASE_RELOAD,
                                       SESSION_ACTION_RELOAD,
                                       self._cycle_index, False, None)

        if now >= self._next_cycle_at:
            self._phase = SESSION_PHASE_DWELL
            self._cycle_index += 1
            self._cycle_live = True
            # CONSUME the inherited flags here — one cycle wears each.
            self._cycle_is_retry = self._retry_next
            self._cycle_reload_settle = self._reload_settle_next
            self._retry_next = False
            self._reload_settle_next = False
            return TossSessionDecision(SESSION_PHASE_DWELL,
                                       SESSION_ACTION_START_CYCLE,
                                       self._cycle_index, False, None)
        self._phase = SESSION_PHASE_DWELL
        return TossSessionDecision(SESSION_PHASE_DWELL, SESSION_ACTION_NONE,
                                   self._cycle_index, False, None)

    def _checking_reject(self) -> Optional[str]:
        """Session-level CHECKING, strictest first. Every one of these is a
        pre-throw refusal that moves NOTHING — no cycle has been built, so the
        machine is exactly as the operator left it."""
        if self.num_throws < 1 or self.num_throws > self.max_throws:
            return 'REJECTED_NUM_THROWS'
        if self.throw_delay_s < self.min_throw_delay_s:
            # The session mirrors the CYCLE's own delay gates so the verdict
            # names throw_delay_s — the field that is wrong — instead of dying
            # one layer down. Without this, a goal whose delay is illegal
            # satisfies `dwell >= delay + margin`, is ACCEPTED, builds cycle 1's
            # whole per-goal state (the live pose read, the release state
            # computed) and only then reports REJECTED_CANT_MAKE_LEAD, naming a
            # field the operator did not think was in play.
            #
            # BOTH cycle gates are mirrored (see min_throw_delay_s): the
            # goal-storm debounce AND the derived `:642` dispatch budget. Until
            # 2026-08-22 there was one constant to mirror, MIN_TOSS_THROW_DELAY_S
            # = 3.5 s; mirroring only the debounce now would re-open the exact
            # hole this gate was written to close, one order of magnitude
            # narrower.
            #
            # And since 2026-08-23 the mirrored floor carries the PRE-DISPATCH
            # SEQUENCE too, through the one derivation the cycle gate uses. A
            # mirror that models a different sequence from the guard it fronts
            # for is not a mirror; it is a second, looser gate wearing the
            # first one's name, and it is what let three cadence rungs ship.
            return 'REJECTED_THROW_DELAY'
        if not math.isfinite(self.dwell_time_s) or self.dwell_time_s < 0.0:
            return 'REJECTED_DWELL'
        if self.dwell_time_s < self.required_dwell_s:
            # The floor is derived from THIS session's throw_delay (module
            # docstring). Refusing beats silently stretching: a cadence the
            # machine quietly ignores is a lie about what it did, and the
            # operator's remedy — raise dwell, or lower throw_delay toward
            # `min_throw_delay_s` — is only visible if the number is named.
            #
            # That remedy said "toward the 3.5 s FSM floor" until 2026-08-22, and
            # MIN_TOSS_THROW_DELAY_S was retired by the same commit that rewrote
            # this hunk (audit fix). The floor is now DERIVED and speed-dependent
            # — since 2026-08-23 it is the `:642` dispatch budget PLUS the
            # pre-dispatch sequence, 0.361 s at the 0.80 s nominal flight and
            # 0.417 s at the C-HAND-3 band floor — so an operator following the
            # old text would lower throw_delay toward a number an order of
            # magnitude above the real one, be refused again, and never discover
            # the rungs are legal. Naming the property rather than a literal is
            # what keeps this sentence true through the next re-derivation.
            return 'REJECTED_DWELL'
        if self.num_throws >= 2 and not self.chain_site_reachable:
            # The chain pre-check, caught BEFORE a ball flies. Without this, a
            # session near the box edge throws one ball, catches it, then
            # refuses cycle 2 REJECTED_WORKSPACE with the platform parked
            # outside the planning box and the ball in the cup — actuation for
            # nothing. At the shipped toss_workspace_xy_mm = 160 no valid B
            # trips it (Phase E's former known limitation, dissolved
            # 2026-08-14); it re-binds only if the box is set below
            # cap × ~1.03 (frontier at box = 150: |B| <= 146.5 chained,
            # >= 147.0 did not — probe, 2026-07-29).
            return 'REJECTED_CHAIN_UNREACHABLE'
        return None

    # ── discrete event (from the node) ─────────────────────────────────────────

    def note_cycle_result(self, result: TossResult, t_release: float,
                          landing_perf: float, *,
                          ball_evidence: Optional[str] = None) -> None:
        """Consume the cycle the node just ran to its terminal.

        ``t_release`` and ``landing_perf`` are the cycle's own scheduled instants
        (``seq.t_release`` and ``seq.t_release + seq.flight_time_s``) — the
        session schedules the NEXT cycle off the previous SCHEDULED landing, not
        off an observed one, because the observed landing is exactly the quantity
        the tracker is least trustworthy about and a cadence must not inherit
        that noise.

        ``ball_evidence`` is the node's LIVE tri-state hand-sensor read at the
        cycle's terminal (``SEATED`` / ``EMPTY`` / ``UNKNOWN``, or None when it
        was not taken). It is consulted for exactly ONE decision — whether an
        ``ABORTED_NO_RELEASE`` may be retried — and never as a gate on anything
        that moves.

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
            self._no_release_streak = 0
        elif outcome.startswith(_MISS_PREFIX):
            # A ball flew and was not caught. It counts as a throw.
            self._throws += 1
            self._no_release_streak = 0
        elif outcome == RELOAD_TRIGGER_OUTCOME \
                and self.on_empty_cup == ON_EMPTY_CUP_RELOAD:
            # THE auto-reload trigger (§ 3.9). Nothing flew and — uniquely among
            # the toss terminals — nothing was ARMED either: REJECTED_NO_BALL is
            # minted in CHECKING before `_positioned` / `_prepare_dispatched`, so
            # the cycle's own `_terminal_action` was ACTION_NONE. That is what
            # makes it safe to enter an interlude here and not from, say, a
            # MISSED (whose SAFE_ABORT ladder is still running) or an
            # ABORTED_PREPARE_FAILED (a machine fault, not an empty cup).
            #
            # It does NOT count as a throw, and with the completion test keyed on
            # THROWS (see below) it therefore does not consume one of the
            # num_throws slots the operator asked for — a drop costs a reload,
            # not a data point.
            self._no_release_streak = 0
            stop = self._reload_precheck()
            if stop is not None:
                self._stop_outcome = stop
                return
            self._reload_pending = True
            return
        elif outcome == NO_RELEASE_OUTCOME:
            # Operator decision 6 (2026-08-10), which REOPENED the design's D9
            # deferral. The hand sensor answers the only question that made the
            # retry unsafe: with a VALID HELD read the ball is demonstrably still
            # in the cup, so the airborne-ball hazard a blind retry would carry is
            # structurally absent. UNKNOWN and EMPTY both refuse — blindness is
            # not evidence, and an EMPTY cup after a non-release means the ball
            # went somewhere nobody watched.
            self._no_release_streak += 1
            if (self._no_release_streak < NO_RELEASE_MAX_CONSECUTIVE
                    and str(ball_evidence or '') == EVIDENCE_SEATED_NAME):
                self._retry_next = True
                # The retry gets the SAME cleanup floor the MISS path gets, for
                # the SAME reason: an ABORTED_NO_RELEASE reaches this hook with
                # `_positioned` and `_prepare_dispatched` both true, so its
                # terminal action is ACTION_SAFE_ABORT — the identical ladder,
                # dispatched on the identical service acks. Without the floor
                # `_next_cycle_at` still holds the PREVIOUS cycle's instant,
                # which is already in the past, so the retry starts on the very
                # next FSM tick — while the retract is still descending and the
                # 2.0 s recentre profile is still traversing. That is exactly the
                # REJECTED_HAND_NOT_PARKED / mid-traverse-throw-site pair the
                # MISS floor was added to prevent, and it would ALSO destroy the
                # epidemic gauge this branch exists to feed: the retry would die
                # a machine-fault verdict instead of a second ABORTED_NO_RELEASE,
                # so "two consecutive non-releases stop the session" could never
                # fire and the operator would be routed to the wrong subsystem.
                #
                # `landing_perf` is a SCHEDULED instant (t_release + flight), not
                # an observed one, and for a non-release nothing actually flew —
                # but the constant is still a valid floor here, with slack: this
                # ladder starts at `t_release + release_grace_s` (0.5 s), so the
                # need measured from t_release is 0.5 + GO_HOME_DURATION_S +
                # 2 ticks = 2.60 s, while this grants flight + 2.80 s >= 2.80 s
                # for any flight >= 0. Reusing the constant (rather than deriving
                # a second, shorter one) is also what keeps the two teardown
                # floors from drifting apart — the reload interlude's rung 4
                # reuses it for the third time.
                self._next_cycle_at = (float(landing_perf)
                                       + float(self.miss_cleanup_s))
                return
            self._stop_outcome = 'ABORTED_CYCLE_{}'.format(outcome)
            return
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
        if self._throws >= self.num_throws:
            # Keyed on THROWS, not on cycle_index — and the two are IDENTICAL for
            # every session the pre-2026-08-11 machine could run: the only
            # outcomes that do not increment `_throws` are the REJECTED_*/
            # ABORTED_* family, and every one of those STOPPED the session before
            # reaching here. So this is behaviour-preserving today, and it is what
            # makes an auto-reloaded REJECTED_NO_BALL (and the single
            # ABORTED_NO_RELEASE retry) cost a cycle index without costing one of
            # the num_throws data points the operator asked for. The runaway is
            # fenced by max_reloads and by NO_RELEASE_MAX_CONSECUTIVE, not by this
            # counter.
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

    def _reload_precheck(self) -> Optional[str]:
        """The SESSION's half of the interlude precondition gate — the two rungs
        that are counters rather than observations. Returns a stop code, or None
        when the node may proceed to its own (BB-ready / BB-verified / sensor /
        recentre) rungs.

        The floor tally is incremented FIRST and unconditionally: a ball reached
        the floor whether or not any budget remains, and a count that skips the
        exhausting drop would under-report exactly the sitting that needs the
        pause most. The CODES are then checked budget-first, per § 3.9."""
        self._floor_balls += 1
        if self.reload_budget_remaining <= 0:
            return OUTCOME_STOPPED_RELOAD_BUDGET
        if 0 < int(self.floor_pause_every) <= self._floor_balls:
            return OUTCOME_STOPPED_FLOOR_CLEAR_REQUIRED
        return None

    def note_reload_result(self, ok: bool, *, attempts: int = 1,
                           stop_code: Optional[str] = None) -> None:
        """Consume the interlude the node just ran.

        ``attempts`` is how many reload attempts it actually SPENT — the BB
        not-positioned-in-time retry re-enters the reload FSM inside one
        interlude, and every re-entry is a real BB ball, so it is charged to the
        same budget. ``stop_code`` is the node's named refusal (its half of the
        precondition gate, or a failed reload); the session stops on it.

        A successful interlude flags the NEXT cycle ``reload_settle`` — the cycle
        after a reload is excluded from every fit (guard G10), because a
        just-recentred platform holding a just-delivered ball is not the steady
        state the map is fitted from."""
        if self._finished or not self._reload_pending:
            return
        self._reload_pending = False
        self._phase = SESSION_PHASE_DWELL
        self._reloads_used += max(0, int(attempts))
        if ok:
            self._reload_settle_next = True
            return
        self._stop_outcome = str(stop_code or OUTCOME_STOPPED_RELOAD_FAILED)

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
            cycle_dwell_s=list(self._dwells),
            reloads_used=self._reloads_used)
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
