#!/usr/bin/env python3
"""Cadence-ladder rung checker — does a published rung actually THROW?

``tests/hardware/session_cadence_ladder.md`` publishes a table of
``(throw_height_m, dwell_time_s, throw_delay_s)`` triples and asks an operator to
arm the machine at each one.  Until 2026-08-22 that table was verified only as
far as *"the session accepts it AND the cycle's CHECKING pass reaches
POSITIONING"* — which is two gates short of the one that actually refuses a fast
rung.  ``ABORTED_CANT_MAKE_RELEASE`` is minted in ``_step_preparing``, i.e.
**after** POSITIONING, so a rung can clear both published checks and still abort
every cycle at ``cycle_start + 0.06 s`` with the hand retracting under a seated
ball.

This probe closes that gap.  For each rung it drives BOTH real FSMs, at the real
node tick, and reports the phase the cycle actually reaches:

  1. :class:`toss_session.TossSessionSequencer` — the session accept gate
     (``_checking_reject`` via a real ``step``), which must emit
     ``ACTION_START_CYCLE``.
  2. :class:`toss_sequencer.TossSequencer` — the cycle FSM, ticked at
     ``toss_sequencer.NODE_LOOP_PERIOD_S`` (the MEASURED cost of one
     ``_run_toss_cycle`` iteration, which is what the accept floor is charged in
     — not the ``time.sleep`` at the bottom of it) and fed the **fastest node
     behaviour that is legal**:

       * every live precondition already true on the first tick,
       * POSITIONING answered on the very next tick (``note_position_noop`` when
         the census-B1 skip fires, ``note_position_result`` with the planner's
         ``min_move_duration_s`` floor when it does not),
       * PREPARE succeeding on the next tick after that,
       * the announcement acknowledged instantly.

     Nothing here is optimistic in the machine's favour beyond what the node can
     actually do, so a rung this probe reds cannot be rescued by a faster node —
     and a rung it greens still has to survive the three synchronous service
     round-trips the PREPARE bundle makes, which this probe charges as zero.

**THE SPLIT THAT USED TO BE HERE IS GONE (2026-08-23).**  From 2026-08-22 this
probe reported every rung twice, on a LEVEL chain and an AIMED one 0.38 s apart,
because ``reload_coordinator_node._toss_already_positioned`` returned ``False``
on every TILTED release by construction — ``trajectory/commanded_position``
carried no orientation, so the node could not verify a pre-tilt pose and had to
command a zero-millimetre move on every cycle of every aimed sitting.  That
orientation is now published (``trajectory/commanded_pose``, intent frame), so
the skip works on an aimed chain and the discriminator became **"does POSITIONING
command a move"**, which is a per-CYCLE fact rather than a per-SITTING one:

  * the FIRST cycle of a sitting commands the move (the platform is not yet at
    the pre-tilt pose).  The node grants it the extra lead —
    ``_build_toss_cycle(delay_is_cadence=True)`` — so it flies rather than dying;
  * every CHAINED cycle takes the skip, because a CAUGHT toss ends in
    ``ACTION_STAY`` holding the pose it threw and caught from.

So each rung is checked on BOTH — ``chained`` is the steady state an operator
counts, ``first`` is the one cycle that pays — and twice more for
``ilc_speed_trim_possible``, because a loaded layer-3 artifact makes the session
judge every floor at the slowest release its apply seam could command.

Usage::

    source ~/Desktop/PDJ_venv/venv/bin/activate
    python tools/probes/cadence_rung_check.py             # the published ladder
    python tools/probes/cadence_rung_check.py --solve     # + the per-rung floors
    python tools/probes/cadence_rung_check.py --frontier  # + the fastest legal
                                                          #   cadence per flight
    python tools/probes/cadence_rung_check.py --grid      # + accept-implies-flies
                                                          #   over the whole grid
    python tools/probes/cadence_rung_check.py --pipeline  # + the PIPELINED floors
                                                          #   (plan B, § 2.7/§ 6.2)
    python tools/probes/cadence_rung_check.py --pipeline --loop-period 0.070
                                                          #   … at the MEASURED
                                                          #   loop bound (P1)

Read-only: constructs FSMs in-process, commands nothing, touches no hardware and
no ROS graph.  Outputs to stdout only (see ``tools/probes/README.md``).
Exit code 1 if any rung reds or the grid finds a violation.

Findings on the tree of 2026-08-26, after owner decision D3.  ``--frontier`` is
monotone across the whole C-HAND-3 band, fastest at the FLOOR, and the two
columns MEET there (the throw envelope refuses a negative speed trim at the band
floor, so an armed ILC costs nothing)::

    fastest, aim disarmed  1.1850 s = 50.6 throws/min  (T 0.4949, delay 0.4968, dwell 0.6901)
    fastest, ILC loaded    1.1850 s = 50.6 throws/min  (T 0.4949, delay 0.4968, dwell 0.6901)

The aimed frontier was **40.4 throws/min** on 2026-08-22 and **54.3** on
2026-08-23.  The 34 % gain was the B1 skip becoming reachable on a tilted
release; the accept-floor redesign bought no cadence at all, and was never going
to — what it bought is that a rung the gates accept cannot abort mid-sequence
with the hand committed.

**D3 gave 6.8 % of that back — 54.3 -> 50.6 — and it is the same purchase, made
honestly this time.**  The pre-dispatch sequence is now charged in the FSM loop's
measured PERIOD rather than in its sleep (``toss_sequencer.NODE_LOOP_PERIOD_S``),
because the sleep was never what a tick costs: measured over 28 cycle starts in
bag ``2026-08-26_14-25-16`` an iteration is 0.0267-0.0377 s against a 0.020 s
sleep.  The 54.3 figure was a cadence the gates would accept and the machine
could not make — and on that sitting it did not: two cycles cleared the accept
floor by 26 and 39 ms and then aborted ``ABORTED_CANT_MAKE_RELEASE`` in
PREPARING, with the catch latch raised and the announcement already published.

**The way back to 54.3 is the LOOP, not the floor.**  Every millisecond of
per-tick work is charged four times over in the skip budget, so the levers are
real ones — fewer blocking service calls in the PREPARE bundle, a cheaper
observation build — and each of them is measurable by re-running the grep this
constant was cut from.  Relaxing the floor instead just re-buys the abort.

**``--pipeline`` (2026-08-27, Phase B0 / probe P3 of
``plans/active/toss-pipelined-preamble.md``).**  The other way back is to stop
spending the preamble on the critical path at all: stage cycle ``k+1``'s
CHECKING / POSITIONING / PREPARING inside cycle ``k``'s flight and charge the
cadence only the COMMIT tick.  That mode models the plan's § 2.7 floors and
§ 6.2 rungs, so the milestone can be argued against arithmetic before any of
B1–B4 is written.

✅ **B4 DISCHARGED THE RECONCILIATION (2026-08-27).**  Both
``commit_budget_s`` and the pipelined branch of ``required_dwell_s`` now ship in
``ros_ws/``, and this probe FORWARDS to them rather than modelling them — pinned
as an identity by
``tests/motion/test_cadence_rung_check.py::test_the_probe_imports_the_shipped_commit_budget_rather_than_modelling_it``
and ``::test_the_probes_pipelined_floor_is_the_shipped_required_dwell``.  Every
term (the dispatch budget, the handoff margin, the hand floor, the loop period)
comes from the shipped code and none is restated here.

The one thing that is still the probe's own is :func:`commit_tick`'s tick loop —
a MODEL of ``_step_committing`` + ``_slip``, not an import of them — and it was
reconciled with the FSM's slip semantics on 2026-08-28 (see that function).  It
is the standing obligation: a probe that keeps its own copy of a shipped rule
goes stale silently.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_HERE, '..', '..'))
# FIRST on sys.path, deliberately: the shell profile exports a PYTHONPATH
# pointing at the colcon INSTALL tree, and a probe that silently measured the
# last `colcon build` instead of the working tree would be worse than no probe.
sys.path.insert(0, os.path.join(_REPO_ROOT, 'ros_ws', 'src', 'jugglebot'))

import jugglebot.hardware_config as hw                           # noqa: E402
from jugglebot import toss_sequencer as ts                       # noqa: E402
from jugglebot import toss_session as tsess                      # noqa: E402
from jugglebot.ball_possession import EVIDENCE_SEATED            # noqa: E402
from jugglebot.motion.trajectory import hand_stroke              # noqa: E402
from jugglebot.motion.trajectory.toss_release import (           # noqa: E402
    apex_height_from_flight_time, flight_time_from_height)
from jugglebot.toss_sequencer import (                           # noqa: E402
    CATCH_CONFIRM_WINDOW_S, FLIGHT_TIME_MAX_S, FLIGHT_TIME_MIN_S,
    FLOOR_REPRESENTATION_SLACK_S, NODE_LOOP_PERIOD_S)

# THE ITERATION COST, IMPORTED — never restated here.
#
# Until 2026-08-26 this module carried its own ``NODE_TICK_S = 0.02`` literal and
# advanced the FSM by it, while ``pre_dispatch_budget_s`` charged the SAME ladder
# at ``NODE_LOOP_PERIOD_S`` (0.040 s) per iteration.  A probe whose clock runs at
# half the rate the gate is charged at reports a lead the machine never has: it
# green-lit the pre-D3 arithmetic that aborted two cycles of bag
# ``2026-08-26_14-25-16`` with the hand committed, and reported 0 grid violations
# for it.  The unit is now the one the floor is cut from, by import, so the two
# cannot drift again — move ``NODE_LOOP_PERIOD_S`` and this probe moves with it.

#: The planner floor a real (non-skipped) go_to_pose pays even for a zero-length
#: pre-tilt move.  Sourced from the generated config, never hardcoded.
MIN_MOVE_DURATION_S = float(hw.JB_TRAJ_MIN_MOVE_DURATION_S)

#: The ladder as published in tests/hardware/session_cadence_ladder.md § 2:
#: ``(name, throw_height_m, dwell, delay)`` — ONE pair per rung again.
#:
#: It carried TWO pairs from 2026-08-22 to 2026-08-23, a LEVEL one and an AIMED
#: one 0.38 s apart, because the B1 positioning skip could not verify a tilted
#: pose and every aimed cycle therefore commanded a zero-millimetre move. With
#: the orientation published that split is gone: a CHAINED cycle takes the skip
#: whatever its tilt, so one pair serves both. Each pair below is legal with a
#: layer-3 artifact LOADED as well as without it — the stricter case, because
#: this runbook exists to arm ILC-primary.
#:
#: ⚠ **THE FAST END MOVED ON 2026-08-26 (owner decision D3).** The pre-dispatch
#: sequence is now charged in the FSM loop's measured PERIOD instead of in its
#: sleep (``toss_sequencer.NODE_LOOP_PERIOD_S``), which raises the CHAINED
#: (census-B1 skip) delay floor by **0.080 s** and the FIRST-cycle moving floor by
#: **0.060 s**, and every derived dwell floor by its own delay's change.
#:
#: (The two are different because ``pre_dispatch_budget_s`` is
#: ``(arrival_ticks + 3) * loop`` and only the ``+3`` scales with the loop on the
#: skip path: with a move the 0.400 s arrival is a WALL-CLOCK quantity, so its
#: tick count halves as the period doubles — 20 -> 10 ticks — and the budget goes
#: 0.460 -> 0.520, not 0.460 -> 0.920. "+0.080 s on every floor" was wrong and is
#: corrected here, audit finding W9.)
#:
#: R0-R3 clear their floors by seconds and are untouched; R4 and R5 were both
#: published INSIDE the new floors and are re-cut here, at the ILC-loaded
#: (stricter) column:
#:
#:     R4   0.65 / 0.45  ->  0.69 / 0.50    (46.3 throws/min, was 47.8)
#:     R5   0.70 / 0.47  ->  0.76 / 0.55    (47.5 throws/min, was 49.9)
#:
#: R5's pair is the OWNER'S MARGIN RE-CUT (2026-08-26), not the smallest legal
#: one. The smallest legal pair at that height is 0.72 / 0.51, which clears its
#: delay floor by 2.0 ms; 0.55 restores the 42.0 ms of delay clearance the rung
#: carried before D3, and costs 1.6 throws/min against the razor edge. ⚠ It does
#: NOT restore the DWELL clearance: ``required_dwell_s`` is
#: ``throw_delay + handoff_margin``, so raising the delay raises the dwell floor
#: with it and 0.76 clears by **1.9 ms** — the same razor edge the delay just
#: stepped back from. See the runbook's § 2.1 clearance table.
#:
#: ⚰ **R5-prime is RETIRED** (owner decision, 2026-08-26). It existed to be R5's
#: tighter twin at the same height; under the D3 floors there is no tighter legal
#: pair at that height, so the row duplicated R5 exactly. The runbook keeps a
#: one-line tombstone so inbound references resolve. It stays in
#: ``LADDER_PRE_AUDIT`` below, which is a HISTORICAL record and must not follow.
#: Reviving it as a distinct rung needs the loop period reduced, not the floor
#: relaxed.
LADDER = [
    ('R0',       0.78, 5.60, 5.00),
    ('R1',       0.78, 4.10, 3.50),
    ('R2',       0.78, 3.00, 2.40),
    ('R3',       0.78, 1.50, 0.90),
    ('R4',       0.45, 0.69, 0.50),
    ('R5',       0.31, 0.76, 0.55),
]

#: The ladder as it was published from 78daf4b until the 2026-08-22 audit fix —
#: kept as the REGRESSION case, because three of its rungs abort every cycle and
#: a probe that can no longer show that has lost the finding.
LADDER_PRE_AUDIT = [
    ('R0',       0.78, 5.60, 5.00),
    ('R1',       0.78, 4.10, 3.50),
    ('R2',       0.78, 3.00, 2.40),
    ('R3',       0.78, 1.50, 0.90),
    ('R4',       0.45, 0.75, 0.55),
    ('R5',       0.31, 0.60, 0.40),
    ('R5-prime', 0.31, 0.49, 0.35),
]


#: The § 6.2 pipelined ladder — ``(name, throw_height_m, dwell, delay)``.
#:
#: ``throw_delay_s`` is **0.0** at every pipelined rung and that is not a
#: placeholder: under B2/B4 the release is scheduled ABSOLUTELY
#: (``release_at_perf``) and the delay field stops being the cadence lever
#: (plan § 2.6).  The dwell steps are the owner's: 0.76 -> 0.65 -> 0.55 -> 0.50
#: -> 0.45, at the two milestone heights.  P3 and P4 are the milestone; P5 is its
#: lower edge and is deliberately last.
PIPELINED_LADDER = [
    ('P0', 1.30, 0.76, 0.0),
    ('P1', 1.30, 0.65, 0.0),
    ('P2', 1.30, 0.55, 0.0),
    ('P3', 1.30, 0.50, 0.0),
    ('P4', 1.00, 0.45, 0.0),
    ('P5', 1.00, 0.43, 0.0),
]

#: The § 1.1 physics milestone: a two-balls-in-one-hand dwell ratio of 0.65 of
#: the hand period gives ``D = 0.325·T / 0.675``.  Stated as the ratio rather
#: than as four dwell literals so a different ratio re-cuts the whole table.
MILESTONE_DWELL_RATIO = 0.325 / 0.675

#: The heights § 1.2 / § 2.7 tabulate. The middle two are the milestone.
MILESTONE_HEIGHTS = (0.50, 0.80, 1.00, 1.30)

#: The measured seat-edge bias — the empty->held cup edge, past the SCHEDULED
#: landing: **median +183.9 ms**, band [+87.6, +554.7] ms, n = 33 over four
#: post-FW-14 bags (``logbook/2026-08-24-arrival-band-remeasure.md``).
#:
#: It is NOT part of any floor and must never become one.  A floor is what the
#: gates promise; this is what the PLANT does, and § 1.4's whole point is that
#: the two are different claims scored separately.  It enters this probe in
#: exactly one place — the predicted ``commit_slip_s`` column — and
#: :func:`pipelined_grid_violations` deliberately does not consult it.
MEASURED_SEAT_EDGE_MEDIAN_S = 0.1839


def flight_for_height(h_m: float) -> float:
    """Apex height -> flight time, through the SAME single conversion the node
    uses (``reload_coordinator_node._resolve_toss_flight_s``)."""
    return flight_time_from_height(float(h_m))


def _observations(now: float, **over):
    """Every live precondition TRUE — the fastest legal machine."""
    kw = dict(
        control_mode=ts.TOSS_CONTROL_MODE, streaming=True, mocap_fresh=True,
        platform_levelled=True, hand_fresh=True, hand_parked=True,
        ball_seated=True, ball_evidence=EVIDENCE_SEATED,
        track_active=False, platform_at_target=True,
        # The honest-cache gate (2026-08-28), healthy — this probe measures
        # CADENCE on a machine that is where it says it is; a moved platform is
        # a correctness question, not a floor question.
        staged_site_ok=True,
    )
    kw.update(over)
    return ts.TossObservations(now=now, **kw)


def run_cycle(T: float, delay_s: float, *, aimed: bool,
              max_ticks: int = 2000):
    """Drive one cycle FSM to its first terminal-or-dispatch outcome.

    Returns ``(verdict, t_reached)`` where verdict is ``'FLIES'`` or the FSM's
    own ``REJECTED_*``/``ABORTED_*`` outcome, and ``t_reached`` is the elapsed
    time from ``start()`` at which that happened.

    ``aimed`` is now exactly "does POSITIONING command a move" — the same
    boolean the node caches at cycle build and feeds the FSM as
    ``positioning_move_expected``. Before 2026-08-23 it also meant "is the
    release tilted", because the B1 skip refused every tilted release; the two
    stopped being the same question when the skip learned to verify an
    orientation, so the AIMED column is now the FIRST cycle of a sitting (or any
    cycle after a go_home / reload) and the LEVEL column is every chained cycle,
    aimed or not.
    """
    T = float(T)
    seq = ts.TossSequencer(
        catch_pose_stow_mm=(0.0, 0.0, ts.TOSS_ACTIVE_Z_MM),
        flight_time_s=T,
        throw_delay_s=float(delay_s),
        event_vel_mps=ts.vertical_event_vel_mps(T),
        throw_site_known=True,
        positioning_move_expected=bool(aimed),
    )
    t0 = 0.0
    seq.start(t0)
    now = t0
    prepare_pending = False
    # `now` is recomputed as t0 + k*period from an INTEGER k rather than
    # accumulated, deliberately: 23 additions of 0.04 land above 23*0.04, and
    # the release-window guard at the last tick compares against a budget that a
    # delay sitting exactly on the accept floor matches EXACTLY. Accumulating
    # turns that equality into an abort and makes the probe report a violation
    # the arithmetic does not have.
    for tick in range(int(max_ticks)):
        now = t0 + tick * NODE_LOOP_PERIOD_S
        dec = seq.step(now, _observations(now))
        if dec.action == ts.ACTION_DISPATCH_THROW:
            return 'FLIES', now - t0
        if dec.done and dec.result is not None:
            return dec.result.outcome, now - t0
        # The elif CHAIN below mirrors reload_coordinator_node._step_toss_sequence
        # exactly, including which branches are mutually exclusive. Getting this
        # wrong by one iteration is a 40 ms error in a budget whose whole margin
        # is single-digit ms, so it is modelled rather than approximated.
        if dec.action == ts.ACTION_POSITION_PLATFORM:
            # _position_platform_for_toss calls note_position_* SYNCHRONOUSLY,
            # inside this tick: the go_to_pose ack (or the B1 no-op decision) is
            # a blocking call on the coordinator thread.
            if aimed:
                # A tilted release can never take the B1 skip, so it pays the
                # planner's min_move_duration floor plus the settle pad.
                seq.note_position_result(now, True, MIN_MOVE_DURATION_S)
            else:
                seq.note_position_noop(now)
        elif dec.action == ts.ACTION_PREPARE_CATCH:
            # The verified-arrival tick raises catch/prime_hold ALONE and DEFERS
            # the bundle to the NEXT tick, so the two travel in different
            # catch_coordinator wait-set cycles. note_prepare_result therefore
            # lands one full tick later — and the bundle's three synchronous
            # service round-trips inside it are charged as zero here.
            prepare_pending = True
        elif dec.action == ts.ACTION_ANNOUNCE:
            seq.note_announcement()          # _announce_toss, synchronous
        elif prepare_pending:
            prepare_pending = False
            seq.note_prepare_result(True)
    return 'NO_TERMINAL', now - t0


def session_accepts(T: float, dwell_s: float, delay_s: float,
                    *, ilc_trim: bool = False):
    """The SESSION gate — returns ``None`` on accept, else its reject code.

    ``ilc_trim`` is ``ilc_speed_trim_possible``: with a layer-3 artifact loaded
    the session judges every floor at the SLOWEST release the apply seam could
    command, not the untrimmed one. Defaults False — the shipped machine
    (``JB_OP_TOSS_ILC_ENABLED`` is false) and the ladder's published rungs."""
    sess = tsess.TossSessionSequencer(
        num_throws=5, dwell_time_s=float(dwell_s),
        throw_delay_s=float(delay_s), flight_time_s=float(T),
        ilc_speed_trim_possible=bool(ilc_trim),
    )
    return sess._checking_reject()


def accept_implies_flies(T: float, dwell_s: float, delay_s: float,
                         *, aimed: bool, ilc_trim: bool = False,
                         strict: bool = False):
    """THE contract, evaluated at one grid point.

    Returns ``None`` when the point is consistent, else a string naming the
    inconsistency.  Two strengths, and both are pinned by
    ``tests/motion/test_cadence_rung_check.py``:

    * always — a goal the SESSION accepts must never die
      ``ABORTED_CANT_MAKE_RELEASE``.  That terminal is minted in
      ``_step_preparing``, i.e. with the catch latch UP, the announcement out
      and the hand committed, and its cleanup retracts the hand under a seated
      ball.  Nothing about the goal's static arithmetic may lead there.
    * ``strict`` (the CHAINED cycle, ``aimed=False``) — the session's accept must
      also imply the CYCLE's own CHECKING accept.  The session mirrors the
      steady-state cycle, so any daylight between them at ``positioning_move =
      False`` is the two gates having drifted apart again.

    On a cycle that must COMMAND its positioning move (``aimed``, i.e. the first
    cycle of a sitting) a ``REJECTED_CANT_MAKE_LEAD`` after a session accept is
    the DESIGNED outcome and not a violation: the session's floor is the chained
    steady state, and the node grants that one cycle the extra lead rather than
    refusing (``_build_toss_cycle(delay_is_cadence=True)``).  This probe drives
    the FSM directly, without the node, so it sees the un-granted case."""
    session = session_accepts(T, dwell_s, delay_s, ilc_trim=ilc_trim)
    if session is not None:
        return None
    verdict, when = run_cycle(T, delay_s, aimed=aimed)
    if verdict == 'FLIES':
        return None
    if verdict.startswith('REJECTED_'):
        # A pre-throw refusal at CHECKING: nothing moved, nothing armed.
        if strict:
            return 'session ACCEPTED but the cycle refused {}'.format(verdict)
        if verdict.startswith('REJECTED_CANT_MAKE_LEAD'):
            return None
        return 'unexpected CHECKING refusal {}'.format(verdict)
    return ('session ACCEPTED but the cycle died {} at +{:.3f} s'
            .format(verdict, when))


def min_flyable_delay(T: float, *, aimed: bool,
                      hi: float = 3.0, tol: float = 1e-4) -> float:
    """Smallest ``throw_delay_s`` at which the cycle reaches DISPATCH_THROW.

    Monotone in ``throw_delay`` (a longer delay only ever grants MORE lead to
    the same fixed pre-dispatch sequence), so a bisection is exact."""
    lo = 0.0
    if run_cycle(T, hi, aimed=aimed)[0] != 'FLIES':
        return float('inf')
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        if run_cycle(T, mid, aimed=aimed)[0] == 'FLIES':
            hi = mid
        else:
            lo = mid
    return hi


def fastest_at(T: float, *, ilc_trim: bool = False):
    """The fastest LEGAL cadence at this flight time: ``(delay, dwell, period)``.

    A SESSION question, not a cycle one — an operator counts landing-to-landing,
    and what bounds that is what the session will ACCEPT. ``delay`` is
    ``TossSessionSequencer.min_throw_delay_s`` (the kind-0 dispatch budget plus
    the CHAINED pre-dispatch sequence); ``dwell`` is the smallest
    ``required_dwell_s`` at that delay; ``period`` is ``dwell + T``.

    ``ilc_trim`` is ``ilc_speed_trim_possible``: with a layer-3 artifact loaded
    every floor is judged at the slowest release the apply seam could command.
    """
    delay = session_floor_delay(T, ilc_trim=ilc_trim)
    dwell = required_dwell(T, delay, ilc_trim=ilc_trim)
    return delay, dwell, dwell + float(T)


def grid_violations(*, verbose: bool = False):
    """Sweep the whole ``(T, dwell, delay, aim)`` grid for contract violations.

    Returns a list of ``(T, dwell, delay, aimed, ilc_trim, reason)``. Empty is
    the contract holding. The grid is deliberately coarse in ``T`` and fine
    around the FLOORS, because that is where accept-vs-runtime disagreements
    live: a delay seconds clear of every gate cannot expose one."""
    out = []
    T = float(FLIGHT_TIME_MIN_S)
    while T <= float(FLIGHT_TIME_MAX_S) + 1e-9:
        for ilc_trim in (False, True):
            sess = tsess.TossSessionSequencer(
                num_throws=5, dwell_time_s=99.0, throw_delay_s=1.0,
                flight_time_s=T, ilc_speed_trim_possible=ilc_trim)
            floor = sess.min_throw_delay_s
            # Straddle the floor tightly, then walk out to the operator-scale
            # delays. ±1 ms either side of a floor is where an off-by-a-tick in
            # either gate shows up.
            delays = [floor - 0.001, floor, floor + 0.001,
                      floor + 0.02, floor + 0.10, 0.90, 2.40, 5.00]
            for delay in delays:
                if delay <= 0.0:
                    continue
                probe = tsess.TossSessionSequencer(
                    num_throws=5, dwell_time_s=99.0, throw_delay_s=delay,
                    flight_time_s=T, ilc_speed_trim_possible=ilc_trim)
                req = probe.required_dwell_s
                for dwell in (req - 0.001, req, req + 0.05, req + 0.50):
                    if dwell <= 0.0:
                        continue
                    for aimed in (False, True):
                        why = accept_implies_flies(
                            T, dwell, delay, aimed=aimed, ilc_trim=ilc_trim,
                            strict=not aimed)
                        if why is not None:
                            out.append((T, dwell, delay, aimed, ilc_trim, why))
                            if verbose:
                                print('  VIOLATION T={:.4f} dwell={:.4f} '
                                      'delay={:.4f} {} ilc_trim={}: {}'
                                      .format(T, dwell, delay,
                                              'first' if aimed else 'chained',
                                              ilc_trim, why))
        T += 0.05
    return out


# ── THE PIPELINED MODEL (plan B § 2.2-2.7) ───────────────────────────────────
#
# Everything below modelled a machine that did not exist when it was written.
# B4 landed it (2026-08-27): the budget and the floor are now IMPORTS, and what
# stays a model is `commit_tick`'s tick loop — reconciled with the FSM's slip
# semantics 2026-08-28.


def commit_budget_s(event_vel_mps: float, min_event_delay_s: float = 0.0,
                    loop_period_s: float = NODE_LOOP_PERIOD_S) -> float:
    """Cycle COMMIT -> release, in seconds — the pipelined sibling of
    ``pre_dispatch_budget_s`` + the dispatch budget.

    ✅ **RECONCILED 2026-08-27 (workstream B4).**  This body was a MODEL through
    Phase B0, carrying the arithmetic § 2.7 specified and an explicit obligation
    that B4 replace it with an import.  B4 landed
    ``toss_sequencer.commit_budget_s`` and this is now that function, forwarded —
    exactly as the ``NODE_TICK_S`` literal this module used to carry was deleted
    in favour of an import, and for the same reason: a probe that keeps its own
    copy of a shipped floor is the 2026-08-22 audit's finding wearing a different
    hat (the session's mirror and the cycle's gate were two expressions of one
    floor, and they had drifted).
    ``tests/motion/test_cadence_rung_check.py`` pins the identity, so a future
    edit that re-forks them reds rather than silently diverging.

    **ONE loop period, not four.**  Under the pipeline the announce tick, the
    PREPARE tick and the deferred-bundle tick have already run inside the
    PREVIOUS cycle's flight, and the >=1-tick armed->announce gap is satisfied by
    construction rather than by a tick.  The one period charged is the COMMIT
    tick itself, which is **polled**: the iteration that crosses ``commit_at``
    may be up to one full loop period late, and that lateness comes straight off
    the lead the release-window guard measures.

    Note what is NOT here and is in ``min_throw_delay_for_release_s``: the
    ``max(TOSS_DISPATCH_DEBOUNCE_S, ...)`` clamp.  That constant is a goal-storm
    debounce on ``throw_delay_s``, an operator-facing field; the commit budget is
    an internal schedule offset that no operator types, so clamping it would
    charge a 0.10 s floor for a hazard that is not on this path.  B4 omitted it
    deliberately, and the shipped docstring says so.
    """
    return ts.commit_budget_s(event_vel_mps, min_event_delay_s, loop_period_s)


def _session(T: float, dwell_s: float = 99.0, delay_s: float = 1.0,
             *, ilc_trim: bool = False):
    """A session object built only to read its derived properties."""
    return tsess.TossSessionSequencer(
        num_throws=5, dwell_time_s=float(dwell_s), throw_delay_s=float(delay_s),
        flight_time_s=float(T), ilc_speed_trim_possible=bool(ilc_trim))


def pipelined_terms(T: float, *, ilc_trim: bool = False,
                    loop_period_s: float = NODE_LOOP_PERIOD_S,
                    budget_loop_s=None) -> dict:
    """Every term of the § 2.7 floor table at one flight time.

    Three of the four are read off the SHIPPED session properties
    (``floor_event_vel_mps``, ``handoff_margin_s``, ``hand_floor_dwell_s``) and
    only ``commit`` is the probe's own — see :func:`commit_budget_s`.

    ``budget_loop_s`` separates *the period the loop actually runs at* from
    *the period the budget CHARGES for it*.  They are the same number on a
    correct build and defaulting one to the other says so; splitting them is
    what lets the grid's counter-check ask "what if the commit budget forgot
    the polled tick" without also pretending the machine stopped polling.
    """
    sess = _session(T, ilc_trim=ilc_trim)
    v = sess.floor_event_vel_mps
    dispatch = hand_stroke.min_throw_event_delay_s(v)
    charge = loop_period_s if budget_loop_s is None else budget_loop_s
    commit = commit_budget_s(v, loop_period_s=charge)
    handoff = float(sess.handoff_margin_s)
    hand_floor = float(sess.hand_floor_dwell_s)
    # The SHIPPED pipelined floor, read off a `pipelined=True` session rather
    # than recomputed. It is the number the accept gate will actually enforce,
    # and pinning it EQUAL to the local `floor` at the default loop period (see
    # tests/motion/test_cadence_rung_check.py) is the other half of B4's probe
    # reconciliation: the forwarded `commit_budget_s` closes the budget, this
    # closes the floor built on it. The local expression survives because the
    # sweeps vary the loop period and the shipped property cannot.
    shipped = _session(T, ilc_trim=ilc_trim)
    shipped.pipelined = True
    return {
        'T': float(T), 'v': v, 'dispatch': dispatch, 'commit': commit,
        'handoff': handoff, 'hand_floor': hand_floor,
        'park_reentry': hand_stroke.catch_park_reentry_s(
            v, float(sess.catch_vel_scale)),
        'floor': max(commit + handoff, hand_floor),
        'shipped_floor': float(shipped.required_dwell_s),
        'plumbing_binds': (commit + handoff) >= hand_floor,
    }


def pipelined_required_dwell(T: float, *, ilc_trim: bool = False,
                             loop_period_s: float = NODE_LOOP_PERIOD_S,
                             budget_loop_s=None) -> float:
    """``required_dwell_s``'s pipelined branch (§ 2.7):
    ``max(commit_budget_s(v) + handoff_margin_s, hand_floor_dwell_s)``."""
    return pipelined_terms(T, ilc_trim=ilc_trim, loop_period_s=loop_period_s,
                           budget_loop_s=budget_loop_s)['floor']


def pipelined_session_accepts(T: float, dwell_s: float, *,
                              ilc_trim: bool = False,
                              loop_period_s: float = NODE_LOOP_PERIOD_S,
                              budget_loop_s=None):
    """The pipelined SESSION gate — ``None`` on accept, else its reject code.

    Every gate of the shipped ``_checking_reject`` except the delay one, which
    the pipeline does not measure a staged cycle against: with an absolute
    ``release_at_perf`` (§ 2.6) a staged cycle does not run on ``throw_delay_s``
    at all.

    ✅ **RESOLVED 2026-08-27 (B4).**  This docstring used to record "what
    replaces that gate is a B4 decision this probe does not make", because a
    retired gate that nothing replaces is how the 0.160 s got charged twice in
    the first place.  The decision was taken:
    ``toss_sequencer.min_stage_lead_for_release_s`` — ``stage_budget_s +
    commit_budget_s`` — is charged at a staged cycle's CHECKING against its REAL
    lead, and ``REJECTED_THROW_DELAY`` survives unchanged on both branches
    because the FIRST cycle of every pipelined sitting still runs serially and
    its release really is ``accept + throw_delay``.  This model omits it only
    because it models the STEADY STATE, where every cycle is staged.
    """
    sess = _session(T, dwell_s, 0.0, ilc_trim=ilc_trim)
    if sess.num_throws < 1 or sess.num_throws > sess.max_throws:
        return 'REJECTED_NUM_THROWS'
    if not math.isfinite(float(dwell_s)) or float(dwell_s) < 0.0:
        return 'REJECTED_DWELL'
    if float(dwell_s) < pipelined_required_dwell(
            T, ilc_trim=ilc_trim, loop_period_s=loop_period_s,
            budget_loop_s=budget_loop_s):
        return 'REJECTED_DWELL'
    if sess.num_throws >= 2 and not sess.chain_site_reachable:
        return 'REJECTED_CHAIN_UNREACHABLE'
    return None


def commit_tick(T: float, dwell_s: float, *, ilc_trim: bool = False,
                loop_period_s: float = NODE_LOOP_PERIOD_S,
                seat_edge_s: float = 0.0, poll_lateness_s=None,
                budget_loop_s=None):
    """Model ONE commit, from the previous cycle's landing at ``t = 0``.

    Returns ``(verdict, slip_s, commit_at_s, release_at_s)``, all on a clock
    whose origin is that landing — so ``release_at_s`` **is** the ACHIEVED dwell,
    a different claim from the commanded one (§ 1.4; the two must not be
    collapsed).  ``verdict`` is ``'FLIES'`` or, past the slip bound, **the name
    of the gate that was still holding it** — the shipped FSM's own vocabulary:
    ``'REJECTED_HAND_NOT_PARKED'`` / ``'REJECTED_NO_BALL'`` (and
    ``'REJECTED_BALL_UNKNOWN'``, which this model cannot generate because its cup
    has a definite seat instant rather than an UNKNOWN evidence state) for the
    evidence gates, ``'ABORTED_CANT_MAKE_RELEASE'`` for the release-window guard.

    ⚠ **RECONCILED WITH THE FSM 2026-08-28** (the fix wave of
    ``logbook/2026-08-28-pipeline-first-contact-deadlock.md``).  Until then this
    modelled the RETIRED abort: the release-window inequality was evaluated ONCE
    after the wait loop, an over-bound wait returned a ``'SLIP_UNBOUNDED'``
    verdict that no FSM ever mints, and the bound was counted from the LANDING
    instead of from the scheduled commit.  All three are now the shipped rules —
    the guard is a per-tick SLIP inside the loop (``_step_committing`` rung 6,
    routed through ``_slip(abort_code='CANT_MAKE_RELEASE')``), the bound is
    ``_commit_at_sched + catch_confirm_window_s``, and past it the cycle
    terminalises by the name of whichever gate held it.  A lateness sweep run
    against the old model would have over-reported the abort by ~14x, which is
    the 2026-08-22 audit's finding — a probe keeping its own copy of a shipped
    rule — wearing a different hat.

    **The re-arm ORDERING is load-bearing and is the FSM's**
    (``toss_sequencer._slip``): at the slip instant it sets
    ``_t_release = now + commit_budget`` and only THEN does the loop advance to
    the next tick, so the lead the next tick measures is ``budget − loop`` =
    ``dispatch + slack`` rather than a full fresh budget.  That single tick of
    difference IS D1's mechanism (an iteration longer than one nominal period
    consumes all of the headroom), so advance-then-re-arm would model a machine
    with a period of grace it does not have.

    The § 2.4.2 gate, in its own order, with the two evidence instants modelled:

    * ``hand_parked`` becomes true at ``catch_park_reentry_s`` past the landing
      — the same quantity ``handoff_margin_s`` maxes over, so the floor is
      supposed to cover it and a wait here is a **drift between floor and gate**;
    * ``ball_seated`` becomes true at ``seat_edge_s`` past the landing — a
      measured PLANT property that no floor claims to cover, so a wait here is
      the § 1.4 prediction, not a defect.  Default 0.0 (the fastest legal
      machine, the doctrine ``_observations`` already uses); pass the measured
      median to get the predicted ``commit_slip_s``.

    ``slip_s`` is § 2.4.3's quantity — ``max(0, evidence − commit_at)``, the
    continuous one the plan predicts as 0 ms at ``h = 1.3`` and ~60 ms at
    ``h = 1.0``.  ``release_at_s`` is the POLLED consequence of it and is
    therefore coarser: a slipped commit lands on the next tick boundary, so the
    achieved dwell rounds UP by up to one loop period.  Keeping the two separate
    is deliberate — the plan's § 1.4 achieved-period table is the continuous
    reading and does not carry that quantisation.

    ``poll_lateness_s`` is how late the iteration crossing ``commit_at`` runs.
    It defaults to a full loop period, the WORST case, because that is exactly
    what :func:`commit_budget_s` charges and a bound argument evaluated at the
    best case is not a bound argument.  The truth is uniform on ``[0, loop)`` —
    the loop's phase is inherited from the previous cycle and nothing aligns it
    to ``commit_at``.  It is **not** the knob for § 1.4's continuous reading:
    once the commit waits at all, the wait quantises onto the tick grid whatever
    phase it started from, so ``poll_lateness_s = 0`` still rounds up.  § 1.4's
    number is the ``loop -> 0`` limit, ``max(dwell, evidence + commit_budget)``,
    and :func:`print_pipeline` computes it in closed form for that reason.
    """
    terms = pipelined_terms(T, ilc_trim=ilc_trim, loop_period_s=loop_period_s,
                            budget_loop_s=budget_loop_s)
    loop = float(loop_period_s)
    if loop <= 0.0:
        raise ValueError('the loop period must be positive — a polled machine '
                         'that never advances cannot be modelled')
    late = loop if poll_lateness_s is None else float(poll_lateness_s)
    commit_at = float(dwell_s) - terms['commit']
    # The SAME representation cover FLOOR_REPRESENTATION_SLACK_S buys the delay
    # floor, applied to the same shape of expression. At a dwell sitting exactly
    # on the pipelined floor, `commit_at` is `(commit + handoff) - commit`, which
    # is `handoff` in real arithmetic and up to a few ULPs below it in binary —
    # measured 5.6e-17 s on the grid. A bare `<` turns that into a whole slipped
    # loop period, i.e. the probe would report a 40 ms defect caused by the bit
    # pattern. 1e-6 s is ten orders of magnitude above the error and four below
    # the smallest real quantity here, so it cannot mask a genuine wait.
    evidence_at = (max(terms['park_reentry'], float(seat_edge_s))
                   - FLOOR_REPRESENTATION_SLACK_S)
    slip = max(0.0, evidence_at - commit_at)
    # WHICH evidence gate is the one that would still be holding at the bound.
    # `evidence_at` is the max of the two, and the FSM names them individually
    # (rung 3 hand_parked, rung 4 ball_seated) so a refusal routes the operator
    # to the right subsystem.
    evidence_code = ('REJECTED_HAND_NOT_PARKED'
                     if terms['park_reentry'] >= float(seat_edge_s)
                     else 'REJECTED_NO_BALL')
    now = commit_at + late
    t_release = float(dwell_s)
    while True:
        # ONE tick of `_step_committing`, in the FSM's own rung order: the
        # evidence gates first, then the runtime release-window guard. All three
        # SLIP; none of them aborts on the spot.
        if now < evidence_at:
            held = evidence_code
        elif t_release - now < terms['dispatch']:
            held = 'ABORTED_CANT_MAKE_RELEASE'
        else:
            return 'FLIES', slip, commit_at, t_release
        # `_slip`, in ITS order: the shared bound is tested at `now` FIRST — one
        # bound for every source, counted from the ORIGINAL commit instant, so a
        # cycle cannot launder an unbounded wait by alternating its reasons.
        if now > commit_at + CATCH_CONFIRM_WINDOW_S:
            return held, slip, commit_at, t_release
        # …and only then the re-arm, at the slip instant, BEFORE the tick
        # advances. Reversing these two lines gives the machine a whole extra
        # loop period of lead it does not have (see the docstring).
        t_release = now + terms['commit']
        now += loop


def pipelined_accept_implies_flies(T: float, dwell_s: float, *,
                                   ilc_trim: bool = False,
                                   loop_period_s: float = NODE_LOOP_PERIOD_S,
                                   budget_loop_s=None):
    """THE contract, pipelined, at one grid point. ``None`` when consistent.

    Two strengths, both real:

    * a dwell the session ACCEPTS must never reach ``ABORTED_CANT_MAKE_RELEASE``
      — the same terminal, reached from the same place.

      ⚠ Its COST changed on 2026-08-28 and the old "latch up, announcement out,
      hand committed" reading no longer applies on the pipelined path: the
      commit gate now slips, and nothing is armed and no announcement has gone
      out until the tick that actually commits (§ 2.4.3's staged-failure table).
      What the terminal costs here is the CYCLE — the staged slot is discarded
      and the cadence loses a beat — and what it still means is a machine that
      slipped its whole ``catch_confirm_window_s`` and could not make the
      release. That is a floor-versus-gate disagreement whether or not the hand
      was committed, which is why the contract is unchanged;
    * and it must not force a slip on a HEALTHY machine.  ``handoff_margin_s`` is
      in the pipelined floor exactly so the commit tick lands after the hand is
      back in the park band; if an accepted dwell still slips on that gate, the
      floor and the gate have drifted apart, which is the 2026-08-22 finding's
      shape rather than a cadence the plant simply cannot hold.

    The seat edge is deliberately absent (see :data:`MEASURED_SEAT_EDGE_MEDIAN_S`).
    """
    if pipelined_session_accepts(T, dwell_s, ilc_trim=ilc_trim,
                                 loop_period_s=loop_period_s,
                                 budget_loop_s=budget_loop_s) is not None:
        return None
    verdict, slip, _, _ = commit_tick(T, dwell_s, ilc_trim=ilc_trim,
                                      loop_period_s=loop_period_s,
                                      budget_loop_s=budget_loop_s)
    if verdict != 'FLIES':
        return 'session ACCEPTED but the commit {}'.format(verdict)
    if slip > 0.0:
        return ('session ACCEPTED but the commit slips {:.4f} s on the park '
                'gate the floor is supposed to cover'.format(slip))
    return None


def pipelined_grid_violations(*, loop_period_s: float = NODE_LOOP_PERIOD_S,
                              budget_loop_s=None,
                              verbose: bool = False) -> list:
    """Sweep ``(T, dwell, ilc)`` for pipelined contract violations.

    Same shape as :func:`grid_violations` and same reason for the shape: coarse
    in ``T``, fine around the FLOOR, because a dwell seconds clear of every gate
    cannot expose a disagreement between two gates.
    """
    out = []
    T = float(FLIGHT_TIME_MIN_S)
    while T <= float(FLIGHT_TIME_MAX_S) + 1e-9:
        for ilc_trim in (False, True):
            floor = pipelined_required_dwell(T, ilc_trim=ilc_trim,
                                             loop_period_s=loop_period_s,
                                             budget_loop_s=budget_loop_s)
            for dwell in (floor - 0.001, floor, floor + 0.001, floor + 0.02,
                          floor + 0.10, floor + 0.50, 1.50, 5.60):
                if dwell <= 0.0:
                    continue
                why = pipelined_accept_implies_flies(
                    T, dwell, ilc_trim=ilc_trim, loop_period_s=loop_period_s,
                    budget_loop_s=budget_loop_s)
                if why is not None:
                    out.append((T, dwell, ilc_trim, why))
                    if verbose:
                        print('  VIOLATION T={:.4f} dwell={:.4f} ilc_trim={}: '
                              '{}'.format(T, dwell, ilc_trim, why))
        T += 0.05
    return out


def fastest_pipelined_at(T: float, *, ilc_trim: bool = False,
                         loop_period_s: float = NODE_LOOP_PERIOD_S):
    """``(dwell, period)`` — the fastest legal PIPELINED cadence at this flight."""
    dwell = pipelined_required_dwell(T, ilc_trim=ilc_trim,
                                     loop_period_s=loop_period_s)
    return dwell, dwell + float(T)


def milestone_dwell(T: float) -> float:
    """§ 1.1's two-in-one-hand dwell at this flight time."""
    return MILESTONE_DWELL_RATIO * float(T)


def print_pipeline(loop_period_s: float, seat_edge_s: float,
                   *, frontier: bool = False, grid: bool = False) -> bool:
    """The § 2.7 floor table, the § 6.2 rungs, and optionally the frontier and
    the grid. Returns True when every § 6.2 rung clears its floor."""
    print()
    print('=' * 78)
    print('THE PIPELINED MODEL — plan toss-pipelined-preamble.md § 2.7 / § 6.2')
    print('commit_budget_s = dispatch + {:.3f} s (ONE loop period) + slack '
          '{:.0e}'.format(loop_period_s, FLOOR_REPRESENTATION_SLACK_S))
    print('✅ SHIPPED since B4 (2026-08-27): commit_budget_s is imported from '
          'toss_sequencer,')
    print('   and the floor column below is pinned equal to a pipelined '
          'session\'s required_dwell_s.')

    print()
    print('§ 2.7 THE FLOOR TABLE — ILC artifact NOT loaded (the shipped machine)')
    head = ('{:>5} {:>7} | {:>8} {:>8} {:>8} {:>9} | {:>9} {:>9} {:>10}'
            .format('apex', 'T', 'dispatch', '+1 loop', 'handoff', 'handfloor',
                    'floor', 'milestone', 'clearance'))
    print(head)
    print('-' * len(head))
    for h in MILESTONE_HEIGHTS:
        T = flight_for_height(h)
        t = pipelined_terms(T, loop_period_s=loop_period_s)
        target = milestone_dwell(T)
        clear = target - t['floor']
        print('{:>5.2f} {:>7.4f} | {:>8.4f} {:>8.4f} {:>8.4f} {:>9.4f} | '
              '{:>9.4f} {:>9.4f} {:>+10.4f} {}'
              .format(h, T, t['dispatch'], t['commit'], t['handoff'],
                      t['hand_floor'], t['floor'], target, clear,
                      '✓' if clear >= 0.0 else '✗'))
    print('(serial comparison, same heights: dwell floor '
          + ', '.join('{:.4f}'.format(
              required_dwell(flight_for_height(h),
                             session_floor_delay(flight_for_height(h))))
              for h in MILESTONE_HEIGHTS) + ')')

    print()
    print('§ 6.2 THE RUNGS — modelled commit, seat edge {:.4f} s'
          .format(seat_edge_s))
    head = ('{:<5} {:>5} {:>7} {:>7} | {:>7} {:>9} | {:>10} {:>7} {:>8} {:>8} '
            '{:>8} {:>6}'.format('rung', 'h', 'T', 'dwell', 'floor',
                                 'clearance', 'session', 'slip', 'ach§1.4',
                                 'ach@1T', 'period', 'tpm'))
    print(head)
    print('-' * len(head))
    ok = True
    for name, h, dwell, _delay in PIPELINED_LADDER:
        T = flight_for_height(h)
        floor = pipelined_required_dwell(T, loop_period_s=loop_period_s)
        rej = pipelined_session_accepts(T, dwell, loop_period_s=loop_period_s)
        verdict, slip, _, achieved = commit_tick(
            T, dwell, loop_period_s=loop_period_s, seat_edge_s=seat_edge_s)
        # § 1.4's reading: max(commanded, seat edge + commit budget), the
        # loop-period -> 0 limit. NOT a poll-phase choice — the polled machine
        # quantises to its own tick grid whatever phase it starts on.
        terms = pipelined_terms(T, loop_period_s=loop_period_s)
        continuous = max(float(dwell),
                         max(terms['park_reentry'], seat_edge_s)
                         + terms['commit'])
        period = achieved + T
        ok = ok and rej is None and verdict == 'FLIES'
        print('{:<5} {:>5.2f} {:>7.4f} {:>7.4f} | {:>7.4f} {:>+9.4f} | '
              '{:>10} {:>7.4f} {:>8.4f} {:>8.4f} {:>8.4f} {:>6.1f}  {}'
              .format(name, h, T, dwell, floor, dwell - floor,
                      rej or 'ACCEPT', slip, continuous, achieved, period,
                      60.0 / period, verdict))
    print()
    print('PIPELINED LADDER: {}'.format(
        'all rungs COMMIT' if ok else 'RED'))
    print('  "slip" is § 2.4.3\'s continuous quantity, max(0, seat edge − '
          'commit_at) — the\n  seat edge is a PLANT property, not a floor, and '
          'pipelining does not remove it.')
    print('  "ach§1.4" is the plan\'s continuous achieved dwell; "ach@1T" is '
          'the same\n  machine POLLED at its worst tick phase. "period" uses '
          'ach@1T (the bound).')

    if frontier:
        print()
        print('THE PIPELINED FRONTIER — fastest legal cadence per flight time')
        print('{:>7} {:>6} | {:>8} {:>8} {:>7} | {:>8} {:>8} {:>7}'
              .format('T', 'apex', 'dwell', 'period', 'tpm',
                      'dwell', 'period', 'tpm'))
        print('{:>7} {:>6} | {:^25} | {:^25}'
              .format('', '', 'aim DISARMED', 'ILC artifact LOADED'))
        best = {}
        T = float(FLIGHT_TIME_MIN_S)
        while T <= float(FLIGHT_TIME_MAX_S) + 1e-9:
            row = [T, apex_height_from_flight_time(T)]
            for ilc in (False, True):
                w, p = fastest_pipelined_at(T, ilc_trim=ilc,
                                            loop_period_s=loop_period_s)
                row += [w, p, 60.0 / p]
                key = 'ilc' if ilc else 'disarmed'
                if key not in best or p < best[key][0]:
                    best[key] = (p, T, w)
            print('{:>7.4f} {:>6.3f} | {:>8.4f} {:>8.4f} {:>7.1f} | '
                  '{:>8.4f} {:>8.4f} {:>7.1f}'.format(*row))
            T += 0.05
        print()
        for key in ('disarmed', 'ilc'):
            p, T, w = best[key]
            print('FASTEST PIPELINED {:<8} period {:.4f} s = {:.1f} throws/min '
                  'at T {:.4f}, dwell {:.4f}'.format(key, p, 60.0 / p, T, w))

    if grid:
        print()
        print('PIPELINED ACCEPT-IMPLIES-FLIES over the (T, dwell, ilc) grid')
        bad = pipelined_grid_violations(loop_period_s=loop_period_s,
                                        verbose=True)
        print('{} violation(s) at the MODELLED floors'.format(len(bad)))
        if bad:
            ok = False
        # …and the same sweep against a commit budget that FORGOT the polled
        # tick — the machine still polls at `loop_period_s`, the BUDGET charges
        # zero for it. That is the regression this model's +1 loop period exists
        # to prevent, and non-zero here is the finding staying findable, exactly
        # as LADDER_PRE_AUDIT keeps the 2026-08-22 one findable.
        counter = pipelined_grid_violations(loop_period_s=loop_period_s,
                                            budget_loop_s=0.0)
        print('{} violation(s) with the loop period REMOVED from the commit '
              'budget (must be non-zero)'.format(len(counter)))
        if not counter:
            print('  ⚠ the counter-check no longer reproduces its failure')
            ok = False
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--solve', action='store_true',
                    help='also bisect the minimum flyable throw_delay per rung')
    ap.add_argument('--frontier', action='store_true',
                    help='sweep the C-HAND-3 band for the fastest legal cadence')
    ap.add_argument('--grid', action='store_true',
                    help='sweep (T, dwell, delay, aim) for accept-implies-flies '
                         'violations')
    ap.add_argument('--pipeline', action='store_true',
                    help='also model the PIPELINED floors (plan B § 2.7/§ 6.2) '
                         '— combines with --frontier and --grid')
    ap.add_argument('--loop-period', type=float, default=NODE_LOOP_PERIOD_S,
                    help='loop period the PIPELINED model charges (default: the '
                         'shipped NODE_LOOP_PERIOD_S). The measured chained '
                         'bound is 0.070 s — tools/probes/toss_loop_census.py')
    ap.add_argument('--seat-edge', type=float,
                    default=MEASURED_SEAT_EDGE_MEDIAN_S,
                    help='seat-edge bias used for the PREDICTED slip column '
                         '(default: the measured +183.9 ms median). Never a '
                         'floor — see MEASURED_SEAT_EDGE_MEDIAN_S')
    args = ap.parse_args()

    print('loop period {:.3f} s | min_move_duration {:.3f} s | settle pad '
          '{:.3f} s'.format(NODE_LOOP_PERIOD_S, MIN_MOVE_DURATION_S,
                            ts.TOSS_POSITION_SETTLE_PAD_S))
    print()
    print('THE PUBLISHED LADDER — every rung, ILC artifact loaded AND not')
    head = ('{:<9} {:>5} {:>7} {:>7} {:>7} | {:>6} {:>6} {:>5} {:>10} {:>7} {:>7}'
            .format('rung', 'h', 'T', 'floor', 'park', 'dwell', 'delay',
                    'ilc', 'session', 'chained', 'first'))
    print(head)
    print('-' * len(head))
    ok = True
    for name, h, dwell, delay in LADDER:
        T = flight_for_height(h)
        v = ts.vertical_event_vel_mps(T)
        floor = hand_stroke.min_throw_event_delay_s(v)
        park = hand_stroke.catch_park_reentry_s(v, tsess.DEFAULT_CATCH_VEL_SCALE)
        for ilc in (False, True):
            rej = session_accepts(T, dwell, delay, ilc_trim=ilc)
            # The CHAINED cycle: POSITIONING takes the census-B1 skip, because a
            # CAUGHT toss left the platform holding the pose it threw from.
            chained = run_cycle(T, delay, aimed=False)[0]
            # …and the FIRST cycle of the sitting, which must command the move.
            # The node grants it the extra lead (`_build_toss_cycle` with
            # delay_is_cadence=True), so the check is against the GRANTED delay,
            # exactly as the session path will run it.
            granted = max(float(delay),
                          ts.min_throw_delay_for_release_s(v, True))
            first = run_cycle(T, granted, aimed=True)[0]
            ok = ok and rej is None and chained == 'FLIES' and first == 'FLIES'
            print('{:<9} {:>5.2f} {:>7.4f} {:>7.4f} {:>7.4f} | {:>6.2f} {:>6.2f} '
                  '{:>5} {:>10} {:>7} {:>7}   period {:.4f} = {:.1f}/min'
                  .format(name, h, T, floor, park, dwell, delay,
                          'ON' if ilc else 'off', rej or 'ACCEPT',
                          chained, first, dwell + T, 60.0 / (dwell + T)))
    print()
    print('PUBLISHED LADDER: {}'.format('all rungs FLY' if ok else 'RED'))

    print()
    print('THE PRE-AUDIT LADDER (78daf4b) — the regression this probe found')
    for name, h, dwell, delay in LADDER_PRE_AUDIT:
        T = flight_for_height(h)
        rej = session_accepts(T, dwell, delay)
        lvl = run_cycle(T, delay, aimed=False)[0]
        aim = run_cycle(T, delay, aimed=True)[0]
        print('{:<9} dwell {:>4.2f} delay {:>4.2f} | session {:<14} '
              'chained {:<26} first {}'
              .format(name, dwell, delay, rej or 'ACCEPT', lvl, aim))

    if args.solve:
        print()
        print('PER-RUNG FLOORS — the smallest (delay, dwell) the SESSION admits')
        print('{:<9} {:>7} {:>12} {:>12} {:>12} {:>12}'.format(
            'rung', 'T', 'delay_ilc0', 'delay_ilc1',
            'dwell_ilc0', 'dwell_ilc1'))
        for name, h, _dwell, _delay in LADDER:
            T = flight_for_height(h)
            d0, w0, _ = fastest_at(T, ilc_trim=False)
            d1, w1, _ = fastest_at(T, ilc_trim=True)
            print('{:<9} {:>7.4f} {:>12.4f} {:>12.4f} {:>12.4f} {:>12.4f}'
                  .format(name, T, d0, d1, w0, w1))
        print()
        print('…and the CYCLE FSM\'s own bisected minimum flyable delay, both '
              'sequences:')
        print('{:<9} {:>7} {:>14} {:>14}'.format(
            'rung', 'T', 'chained', 'first(move)'))
        for name, h, _dwell, _delay in LADDER:
            T = flight_for_height(h)
            print('{:<9} {:>7.4f} {:>14.4f} {:>14.4f}'.format(
                name, T, min_flyable_delay(T, aimed=False),
                min_flyable_delay(T, aimed=True)))

    if args.frontier:
        print()
        print('THE REACHABLE FRONTIER — fastest legal cadence per flight time')
        print('{:>7} {:>6} | {:>7} {:>7} {:>7} {:>7} | {:>7} {:>7} {:>7} {:>7}'
              .format('T', 'apex', 'dly', 'dwell', 'period', 'tpm',
                      'dly', 'dwell', 'period', 'tpm'))
        print('{:>7} {:>6} | {:^31} | {:^31}'
              .format('', '', 'aim DISARMED', 'ILC artifact LOADED'))
        best = {}
        T = float(FLIGHT_TIME_MIN_S)
        while T <= float(FLIGHT_TIME_MAX_S) + 1e-9:
            apex = apex_height_from_flight_time(T)
            row = [T, apex]
            for ilc in (False, True):
                d, w, p = fastest_at(T, ilc_trim=ilc)
                row += [d, w, p, 60.0 / p]
                key = 'ilc' if ilc else 'disarmed'
                if key not in best or p < best[key][0]:
                    best[key] = (p, T, apex, d, w)
            print('{:>7.4f} {:>6.3f} | {:>7.4f} {:>7.4f} {:>7.4f} {:>7.1f} | '
                  '{:>7.4f} {:>7.4f} {:>7.4f} {:>7.1f}'.format(*row))
            T += 0.05
        print()
        for key in ('disarmed', 'ilc'):
            p, T, apex, d, w = best[key]
            print('FASTEST {:<8} period {:.4f} s = {:.1f} throws/min at '
                  'T {:.4f} (apex {:.3f} m), throw_delay {:.4f}, dwell {:.4f}'
                  .format(key, p, 60.0 / p, T, apex, d, w))

    if args.grid:
        print()
        print('ACCEPT-IMPLIES-FLIES over the whole (T, dwell, delay, aim) grid')
        bad = grid_violations(verbose=True)
        print('{} violation(s)'.format(len(bad)))
        if bad:
            ok = False

    if args.pipeline:
        ok = print_pipeline(args.loop_period, args.seat_edge,
                            frontier=args.frontier, grid=args.grid) and ok
    return 0 if ok else 1


def required_dwell(T: float, delay_s: float, *, ilc_trim: bool = False) -> float:
    """``TossSessionSequencer.required_dwell_s`` at this (T, delay)."""
    sess = tsess.TossSessionSequencer(
        num_throws=5, dwell_time_s=99.0, throw_delay_s=float(delay_s),
        flight_time_s=float(T), ilc_speed_trim_possible=bool(ilc_trim),
    )
    return float(sess.required_dwell_s)


def session_floor_delay(T: float, *, ilc_trim: bool = False) -> float:
    """``TossSessionSequencer.min_throw_delay_s`` at this flight time."""
    sess = tsess.TossSessionSequencer(
        num_throws=5, dwell_time_s=99.0, throw_delay_s=1.0,
        flight_time_s=float(T), ilc_speed_trim_possible=bool(ilc_trim),
    )
    return float(sess.min_throw_delay_s)


if __name__ == '__main__':
    raise SystemExit(main())
