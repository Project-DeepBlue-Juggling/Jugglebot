"""Unit tests for the pure-Python continuous-session FSM (jugglebot.toss_session).

No ROS: the session FSM takes no observations at all — it reasons about TIME and
CYCLE RESULTS only — so every reject, every scheduling decision and every
stop/continue branch is exercised deterministically by feeding it synthetic
:class:`~jugglebot.toss_sequencer.TossResult`s.

The five session invariants the module docstring states (S1 one live cycle, S2 no
session-level motion, S3 stop_on_miss is a cycle-boundary stop with no new abort
point, S4 cancellation defers to the per-cycle rules, S5 the dwell is a quiescent
wait) are pinned here where they are testable in isolation; S2 and S4 are
node-level and live in test_toss_continuous_node.py.

The dwell floor is DERIVED, not chosen, and this file pins the derivation against
the landed constants it is made of — so a future edit to the cycle FSM's own
delay gates, to the config margin, or to the HAND-GEOMETRY term added on
2026-08-22 cannot silently make a configured session cadence unachievable (or,
worse, make an unachievable one legal).
"""

from __future__ import annotations

import math
import re
from pathlib import Path

import pytest

import hardware_config as hw
from jugglebot.motion.trajectory import hand_stroke
from jugglebot.toss_sequencer import (
    CATCH_CONFIRM_WINDOW_S,
    DEFAULT_TOSS_THROW_DELAY_S,
    FLIGHT_TIME_MAX_S,
    FLIGHT_TIME_MIN_S,
    FLOOR_REPRESENTATION_SLACK_S,
    TOSS_DISPATCH_DEBOUNCE_S,
    TossResult,
    min_throw_delay_for_release_s,
    pre_dispatch_budget_s,
    vertical_event_vel_mps,
)
from jugglebot.toss_session import (
    DEFAULT_SESSION_DWELL_MARGIN_S,
    DEFAULT_SESSION_DWELL_S,
    DEFAULT_SESSION_MAX_RELOADS,
    DEFAULT_SESSION_MAX_THROWS,
    DEFAULT_SESSION_MISS_CLEANUP_S,
    EVIDENCE_SEATED_NAME,
    GO_HOME_DURATION_S,
    NODE_LOOP_PERIOD_S,
    NODE_TICK_S,
    ON_EMPTY_CUP_RELOAD,
    ON_EMPTY_CUP_STOP,
    OUTCOME_STOPPED_FLOOR_CLEAR_REQUIRED,
    OUTCOME_STOPPED_RELOAD_BUDGET,
    OUTCOME_STOPPED_RELOAD_FAILED,
    SESSION_ACTION_NONE,
    SESSION_ACTION_RELOAD,
    SESSION_ACTION_START_CYCLE,
    SESSION_PHASE_CHECKING,
    SESSION_PHASE_DWELL,
    SESSION_PHASE_RELOAD,
    SAFE_ABORT_LADDER_S,
    TossSessionResult,
    TossSessionSequencer,
    resolve_on_empty_cup,
)

TRAJECTORY_NODE = (Path(__file__).resolve().parents[2] / 'ros_ws' / 'src'
                   / 'jugglebot' / 'jugglebot' / 'trajectory_node.py')
COORDINATOR_NODE = (Path(__file__).resolve().parents[2] / 'ros_ws' / 'src'
                    / 'jugglebot' / 'jugglebot' / 'reload_coordinator_node.py')

ACTION_FILE = (Path(__file__).resolve().parents[2] / 'ros_ws' / 'src'
               / 'jugglebot_interfaces' / 'action' / 'TossContinuous.action')

FLIGHT = 0.8
DWELL = 8.0          # comfortably above the 5.6 s floor at the 5.0 s delay
DELAY = 5.0


#: The shipped machine's layer-3 state: ``JB_OP_TOSS_ILC_ENABLED`` is false, so
#: no speed trim is possible and every derived floor is judged at the untrimmed
#: ``vertical_event_vel_mps(T)``. Stated here once, explicitly, because the
#: dataclass default is the OTHER way (fail-closed True — a session nobody told
#: assumes the slowest release layer 3 could command). Tests that mean to
#: exercise the trim charge say so; everything else describes the machine as it
#: ships. See ``test_an_armed_ilc_raises_every_derived_floor``.
NO_ILC_TRIM = dict(ilc_speed_trim_possible=False)


def _session(**kw):
    params = dict(num_throws=3, dwell_time_s=DWELL, throw_delay_s=DELAY,
                  flight_time_s=FLIGHT, **NO_ILC_TRIM)
    params.update(kw)
    s = TossSessionSequencer(**params)
    s.start(0.0)
    return s


def _caught(err=4.0, flight=0.81):
    return TossResult(True, 'CAUGHT', err, flight)


def _missed(outcome='MISSED'):
    return TossResult(False, outcome, float('nan'), float('nan'))


def _run_cycle(session, now, result, *, delay=DELAY, flight=FLIGHT):
    """Drive one cycle end to end: step until START_CYCLE, then report the
    result with the cycle's SCHEDULED release/landing instants. Returns the
    (t_release, landing) pair the node would have passed."""
    d = session.step(now)
    assert d.action == SESSION_ACTION_START_CYCLE, d
    t_release = now + delay
    landing = t_release + flight
    session.note_cycle_result(result, t_release, landing)
    return t_release, landing


def _after_cleanup(landing):
    """The earliest instant a continuation past a SAFE_ABORT ladder may start —
    the shared cleanup floor, measured from the cycle's SCHEDULED landing. Both
    the continued MISS and the single ABORTED_NO_RELEASE retry wear it."""
    return landing + DEFAULT_SESSION_MISS_CLEANUP_S


# ── CHECKING: session-level rejects (nothing built, nothing installed) ─────────

@pytest.mark.parametrize('num_throws', [0, -1, DEFAULT_SESSION_MAX_THROWS + 1])
def test_num_throws_out_of_range_rejected(num_throws):
    s = _session(num_throws=num_throws)
    d = s.step(0.0)
    assert d.done and d.result.outcome == 'REJECTED_NUM_THROWS'
    assert d.action == SESSION_ACTION_NONE          # S2: nothing to undo
    assert d.result.throws_completed == 0


def test_num_throws_one_is_legal():
    """num_throws = 1 is exactly one Toss with the session accounting — the
    degenerate case must not be refused, it is how an operator dips a toe in."""
    s = _session(num_throws=1)
    assert s.step(0.0).action == SESSION_ACTION_START_CYCLE


def test_max_throws_boundary_is_inclusive():
    s = _session(num_throws=DEFAULT_SESSION_MAX_THROWS)
    assert s.step(0.0).action == SESSION_ACTION_START_CYCLE


@pytest.mark.parametrize('dwell', [-1.0, float('nan'), float('inf')])
def test_dwell_non_finite_or_negative_rejected(dwell):
    """A NEGATIVE dwell is a sign typo, not a request for the default: 0.0 is
    the ONLY 'use the default' sentinel (the toss FSM's doctrine, transposed)."""
    s = _session(dwell_time_s=dwell)
    d = s.step(0.0)
    assert d.done and d.result.outcome == 'REJECTED_DWELL'


def test_dwell_below_the_derived_floor_is_refused_not_stretched():
    """The floor is ``throw_delay + handoff_margin``. Below it the session
    REFUSES rather than quietly running slower: a cadence the machine ignores is
    a lie about what it did, and the operator's remedy is only visible if it is
    named.

    Read from ``required_dwell_s`` rather than rebuilt from
    ``DEFAULT_SESSION_DWELL_MARGIN_S``, because ``handoff_margin_s`` takes a
    max() with the hand's park re-entry and the arrival term stopped being the
    winner at the 2026-08-24 band re-measure — a test that re-derives the floor
    from the margin alone tests arithmetic the session does not do."""
    floor = _session().required_dwell_s
    s = _session(dwell_time_s=floor - 0.01)
    d = s.step(0.0)
    assert d.done and d.result.outcome == 'REJECTED_DWELL'
    # …and exactly at the floor it is accepted.
    ok = _session(dwell_time_s=floor)
    assert ok.step(0.0).action == SESSION_ACTION_START_CYCLE


def test_the_floor_is_the_larger_of_the_handoff_and_the_hand_geometry():
    """``required_dwell_s = max(delay + margin, hand_floor_dwell_s)``.

    The max() landed on 2026-08-22 with the retirement of the 3.5 s delay floor
    (census A1/A4). Before it, the plumbing term alone WAS the floor, and it was
    safe only by accident: at 3.5 s it evaluated to 4.10 s, an order of magnitude
    above anything the hand could not make, so the physics never had to be
    consulted. Take the accident away and the plumbing term alone would ACCEPT a
    dwell that dispatches cycle N+1's kind-0 throw inside cycle N's live catch
    stroke — the 2026-07-25 clobbered-stroke defect, which is a hardware event
    and not a refusal.

    Both regimes are pinned, because a one-sided test would pass with the max()
    deleted."""
    # (a) SLOW regime — the handoff binds and the hand term is inert.
    slow = TossSessionSequencer(num_throws=2, throw_delay_s=DELAY,
                                flight_time_s=FLIGHT, **NO_ILC_TRIM,
                                dwell_margin_s=DEFAULT_SESSION_DWELL_MARGIN_S)
    assert slow.required_dwell_s == pytest.approx(
        DELAY + slow.handoff_margin_s)
    # 5.137 until the 2026-08-24 band re-measure, when the arrival term (0.137 ⇒
    # 0.087) fell under the PARK term and the max() changed hands. The number is
    # now hand geometry at this flight, not sensor latency.
    assert slow.required_dwell_s == pytest.approx(5.1200, abs=1e-4)
    assert slow.handoff_margin_s > DEFAULT_SESSION_DWELL_MARGIN_S
    assert slow.hand_floor_dwell_s < slow.required_dwell_s

    # (b) FAST regime — the hand geometry binds. Isolated by zeroing the
    # handoff margin AND dropping the delay under the park term, because
    # `handoff_margin_s` floors the margin at `catch_park_reentry_s` (0.1933 s
    # here) whatever `dwell_margin_s` says, so a zeroed margin alone no longer
    # isolates anything. A test that could not tell the two terms apart would
    # pass with the max() deleted.
    fast = TossSessionSequencer(num_throws=2, throw_delay_s=0.20,
                                flight_time_s=FLIGHT_TIME_MIN_S, **NO_ILC_TRIM,
                                catch_vel_scale=0.9, dwell_margin_s=0.0)
    assert fast.hand_floor_dwell_s == pytest.approx(0.4871, abs=1e-3)
    assert fast.required_dwell_s == pytest.approx(fast.hand_floor_dwell_s)
    assert fast.required_dwell_s > 0.20 + fast.handoff_margin_s


def test_the_hand_floor_is_dominated_by_the_plumbing_term():
    """Which term binds, across the whole band — and since 2026-08-23 the answer
    is the SAME everywhere: the plumbing term.

    This test used to assert a CROSSOVER ("the hand binds at the band floor, the
    plumbing binds at the nominal"), and that was true while the delay floor was
    the kind-0 dispatch budget alone. Adding the pre-dispatch sequence to the
    delay floor raised the plumbing term by 0.080 s at every flight (0.160 s
    since owner decision D3 charged that sequence in the loop's measured PERIOD,
    2026-08-26), which is more than the crossover was ever worth:
    ``throw_delay + handoff_margin`` now exceeds ``hand_floor_dwell_s`` across
    the whole C-HAND-3 band, by at least **0.2030 s**.

    Two things are pinned, and both matter:

    1. **The dominance itself**, with its minimum margin — because
       :attr:`hand_floor_dwell_s` is deliberately left on the UNTRIMMED release
       speed (see its docstring), and that choice is only safe while this margin
       covers the term's worst-case 0.0715 s sensitivity to a maximal negative
       ILC speed trim (measured over the band, 2026-08-23; the margin's own worst
       case is **0.2030 s** at ``T = 0.4949``, so the ratio is **2.8x** — it was
       0.1230 s and 1.7x until D3 raised the plumbing term on 2026-08-26). If a
       future change shrinks the
       plumbing term back under the hand floor, this reds and the argument gets
       re-taken rather than silently relied on.
    2. **That the max() is still live** — a floor that is currently never
       selected is one refactor away from being deleted as dead code. It is not
       dead; it is the guarantee that a cadence the STROKE cannot make is
       refused at goal-accept instead of clobbering a live stroke (the
       2026-07-25 defect). Drop the delay below its own floor and the hand term
       selects, exactly as it must."""
    worst = float('inf')
    for flight in (FLIGHT_TIME_MIN_S, 0.55, 0.6059, 0.7977, 1.00,
                   FLIGHT_TIME_MAX_S):
        s = TossSessionSequencer(num_throws=2, flight_time_s=flight,
                                 catch_vel_scale=0.9, **NO_ILC_TRIM,
                                 dwell_margin_s=DEFAULT_SESSION_DWELL_MARGIN_S)
        plumbing = s.min_throw_delay_s + s.handoff_margin_s
        assert plumbing > s.hand_floor_dwell_s, (flight, plumbing,
                                                 s.hand_floor_dwell_s)
        worst = min(worst, plumbing - s.hand_floor_dwell_s)
    # 0.2030 s at T = 0.4949 (0.1230 s before D3). Asserted with a floor rather
    # than an approx so a change that WIDENS the margin is not a failure — the
    # thing that must not happen is it shrinking back onto the 0.0715 s trim
    # sensitivity this dominance argument spends it on.
    assert worst > 0.20, worst

    # The max() is live: below its own delay floor the hand term selects.
    fast = TossSessionSequencer(num_throws=2, throw_delay_s=0.20,
                                flight_time_s=FLIGHT_TIME_MIN_S, **NO_ILC_TRIM,
                                catch_vel_scale=0.9, dwell_margin_s=0.0)
    assert fast.required_dwell_s == pytest.approx(fast.hand_floor_dwell_s)


def test_an_armed_ilc_raises_every_derived_floor_but_the_hand_geometry():
    """With a layer-3 artifact loaded the session judges itself against the
    SLOWEST release the apply seam could command, not the untrimmed one.

    The finding this closes (2026-08-22 audit, the BLOCKING item's second half):
    the session computed its floors from ``vertical_event_vel_mps(T)`` while the
    cycle FSM received ``v · (1 + ilc_vel_trim)``, and every derived floor RISES
    as the speed FALLS — so a NEGATIVE trim raised the cycle's floor AFTER the
    session had accepted the goal.

    Three properties, and the third is why this costs nothing where it matters:

    * the delay floor and the handoff margin BOTH rise when a trim is possible;
    * the hand-geometry floor does NOT (it is the named C-HAND-1 number the
      census and the runbook quote — see :attr:`hand_floor_dwell_s`);
    * at the C-HAND-3 band FLOOR the rise is ~zero, because the envelope's
      ARM_WINDOW term refuses the negative side outright there — which is
      exactly the flight the cadence rungs are built at."""
    for flight in (0.6059, 0.7977):
        off = TossSessionSequencer(num_throws=2, flight_time_s=flight,
                                   catch_vel_scale=0.9, **NO_ILC_TRIM)
        on = TossSessionSequencer(num_throws=2, flight_time_s=flight,
                                  catch_vel_scale=0.9,
                                  ilc_speed_trim_possible=True)
        assert on.floor_event_vel_mps < off.floor_event_vel_mps
        assert on.min_throw_delay_s > off.min_throw_delay_s
        assert on.handoff_margin_s >= off.handoff_margin_s
        assert on.hand_floor_dwell_s == pytest.approx(off.hand_floor_dwell_s)

    at_floor_off = TossSessionSequencer(num_throws=2,
                                        flight_time_s=FLIGHT_TIME_MIN_S,
                                        catch_vel_scale=0.9, **NO_ILC_TRIM)
    at_floor_on = TossSessionSequencer(num_throws=2,
                                       flight_time_s=FLIGHT_TIME_MIN_S,
                                       catch_vel_scale=0.9,
                                       ilc_speed_trim_possible=True)
    assert at_floor_on.min_throw_delay_s == pytest.approx(
        at_floor_off.min_throw_delay_s, abs=1e-3)

    # Fail-CLOSED by default: a session nobody told charges the trim.
    assert TossSessionSequencer(num_throws=2).ilc_speed_trim_possible is True


def test_the_decided_r5_prime_operating_point_is_REFUSED_and_by_how_much():
    """The operator's decided tuning-phase cadence (2026-08-21 decision 3) —
    dwell 0.49 s at flight 0.4949 s, ~61 throws/min — **is not legal**, and this
    test is the record of why.

    It was pinned as LEGAL here until 2026-08-22, clearing by 2.9 ms against a
    floor of ``max(delay + 0.137, hand_floor)``. The audit fix that landed
    :attr:`handoff_margin_s` moved the floor, because 0.137 s (the earliest
    instant a possession VERDICT can exist) was never the whole handoff: cycle
    N+1's CHECKING also needs ``hand_parked``, and the catch stroke does not
    bring the hand back inside the park band until **0.1933 s** past the landing
    at this flight. The 0.6 s margin this rung was designed under covered that
    by accident; 0.137 s does not.

    So the decided point needed ``0.34 + 0.1933 = 0.5333 s`` of dwell against
    the 0.49 s asked for — 43 ms short. Refusing is the correct outcome:
    accepting it schedules cycle N+1's CHECKING 50 ms inside cycle N's live
    catch stroke, where it reads ~1.5 rev and mints REJECTED_HAND_NOT_PARKED on
    a healthy catch.

    **The verdict moved on 2026-08-23, and it moved to the right field.** The
    delay floor grew the pre-dispatch sequence, so 0.34 s is now under the DELAY
    floor as well as under the dwell floor — and the delay gate runs first,
    deliberately: the dwell floor is DERIVED from the delay, so telling an
    operator to raise a dwell that is only too small because the delay is illegal
    sends them to the wrong knob.

    **And the delay floor moved again on 2026-08-26 (D3), 0.4168 -> 0.4968 s at
    this flight**, when the pre-dispatch sequence started being priced in the
    node loop's measured PERIOD rather than in its sleep. That is 0.080 s of
    additional refusal on this rung and on every rung that takes the census-B1
    skip — the price of two cycles in bag 2026-08-26_14-25-16 clearing this gate
    and then aborting ABORTED_CANT_MAKE_RELEASE with the catch armed. The rung was
    already refused, so nothing published changes here; what changes is the
    number an operator needs in order to re-take the decision.

    Pinned as a REFUSAL, with both shortfalls named, so that (a) nobody
    re-publishes the rung without moving a floor back on purpose, and (b) the
    numbers an operator needs in order to re-take the decision are in the test
    suite rather than only in a runbook."""
    s = TossSessionSequencer(num_throws=5, dwell_time_s=0.49,
                             throw_delay_s=0.34,
                             flight_time_s=FLIGHT_TIME_MIN_S,
                             catch_vel_scale=0.9, **NO_ILC_TRIM,
                             dwell_margin_s=DEFAULT_SESSION_DWELL_MARGIN_S)
    s.start(0.0)
    assert s.step(0.0).action != SESSION_ACTION_START_CYCLE
    assert s._checking_reject() == 'REJECTED_THROW_DELAY'
    assert s.min_throw_delay_s == pytest.approx(0.4968, abs=5e-4)
    assert s.min_throw_delay_s - 0.34 == pytest.approx(0.1568, abs=5e-4)
    # The dwell was short too, and by the amount the 2026-08-22 audit named.
    assert s.required_dwell_s == pytest.approx(0.5333, abs=5e-4)
    assert s.required_dwell_s - 0.49 == pytest.approx(0.0433, abs=5e-4)

    # …and the smallest (delay, dwell) pair that IS accepted at this flight,
    # which is the number the runbook's corrected rung is built from.
    delay = s.min_throw_delay_s
    at_floor = TossSessionSequencer(num_throws=5, dwell_time_s=0.0,
                                    throw_delay_s=delay,
                                    flight_time_s=FLIGHT_TIME_MIN_S,
                                    catch_vel_scale=0.9, **NO_ILC_TRIM,
                                    dwell_margin_s=DEFAULT_SESSION_DWELL_MARGIN_S)
    ok = TossSessionSequencer(num_throws=5,
                              dwell_time_s=at_floor.required_dwell_s,
                              throw_delay_s=delay,
                              flight_time_s=FLIGHT_TIME_MIN_S,
                              catch_vel_scale=0.9, **NO_ILC_TRIM,
                              dwell_margin_s=DEFAULT_SESSION_DWELL_MARGIN_S)
    ok.start(0.0)
    assert ok.step(0.0).action == SESSION_ACTION_START_CYCLE
    # 0.6101 s until 2026-08-26; the D3 delay-floor rise (+0.080 s) carries
    # straight through the dwell floor, which is DERIVED from the delay.
    assert ok.required_dwell_s == pytest.approx(0.6901, abs=5e-4)


def test_no_accepted_session_starts_a_cycle_inside_the_live_catch_stroke():
    """**THE handoff contract.** For every session the gate ACCEPTS,
    ``dwell − throw_delay`` (landing → next cycle's CHECKING, the instant
    ``_next_cycle_at`` schedules) must be at least
    ``hand_stroke.catch_park_reentry_s`` — the time the catch stroke takes to
    bring the hand back inside the park band.

    ``hand_parked`` is a hard CHECKING precondition, so violating this does not
    degrade gracefully: it mints REJECTED_HAND_NOT_PARKED on a healthy catch, a
    machine-fault verdict for a cadence fault, and ends the sitting.

    Swept across the whole C-HAND-3 band and a range of delays rather than
    spot-checked, because both branches of that max() have to stay live. Until
    the 2026-08-24 band re-measure the crossover sat INSIDE the C-HAND-3 band —
    park 0.1204 s at the 0.7977 s flight (UNDER the 0.137 s arrival term) and
    0.1933 s at the band floor (OVER it) — so sweeping the flight alone
    exercised both. **The re-measure took the arrival term to 0.087 s and pushed
    the crossover from a ~0.75 s flight out to ~1.13 s**, past every published
    rung, leaving the ``dwell_margin_s`` branch alive only at the very top of the
    C-HAND-3 band — thin cover for a mutation reducing :attr:`handoff_margin_s`
    to ``catch_park_reentry_s``. The margin is therefore swept too: the shipped
    0.087 s and a 0.30 s margin above every park value in the band (where the
    margin always wins). A single-flight, single-margin test would sit on one
    side and pass with the max() collapsed either way.

    Swept with the ILC speed trim BOTH possible and not (2026-08-23): a slower
    commanded release lengthens the catch stroke and therefore the park
    re-entry, so the contract has to hold against the slowest release the apply
    seam could command, not only against the untrimmed one."""
    #: Above every ``catch_park_reentry_s`` the band can produce (max 0.2081 s at
    #: the band floor with a trim possible), so it keeps the ARRIVAL branch live.
    _MARGIN_OVER_PARK_S = 0.30
    parks = []
    for trim in (False, True):
        for flight in (FLIGHT_TIME_MIN_S, 0.55, 0.6059, 0.70, 0.7977, 1.00,
                       1.14):
            for delay in (0.35, 0.50, 0.90, 2.40, 5.00):
                for margin in (DEFAULT_SESSION_DWELL_MARGIN_S,
                               _MARGIN_OVER_PARK_S):
                    s = TossSessionSequencer(
                        num_throws=5, dwell_time_s=0.0, throw_delay_s=delay,
                        flight_time_s=flight, catch_vel_scale=0.9,
                        ilc_speed_trim_possible=trim,
                        dwell_margin_s=margin)
                    park = hand_stroke.catch_park_reentry_s(
                        s.floor_event_vel_mps, 0.9)
                    parks.append(park)
                    if not trim:
                        # …and with no trim possible that IS the untrimmed speed.
                        assert s.floor_event_vel_mps == pytest.approx(
                            vertical_event_vel_mps(flight))
                    # THE invariant: the smallest dwell the gate admits still
                    # leaves the catch stroke room to finish before the next
                    # CHECKING.
                    smallest = s.required_dwell_s
                    assert smallest - delay >= park - 1e-9, (trim, flight,
                                                             delay, park)
                    # And the margin is exactly the max() it claims to be — not
                    # slack the hand floor happens to provide. A reduction of
                    # handoff_margin_s to EITHER of its two terms reds a row.
                    assert s.handoff_margin_s == pytest.approx(
                        max(margin, park)), (trim, flight, delay, margin)
                    assert smallest == pytest.approx(
                        max(delay + s.handoff_margin_s,
                            s.hand_floor_dwell_s)), (trim, flight, delay,
                                                     margin)
    # Both branches really are exercised, and the state of the crossover is
    # asserted rather than assumed. The 2026-08-24 band re-measure did not delete
    # the crossover, it MOVED it: it now sits near the TOP of the C-HAND-3 band
    # rather than in the middle of it, so it covers the whole published ladder
    # and hands back only above a ~1.13 s flight. The 0.30 s margin clears every
    # park value in the band, which keeps the arrival branch live everywhere.
    assert min(parks) < DEFAULT_SESSION_DWELL_MARGIN_S < max(parks)
    assert max(parks) < _MARGIN_OVER_PARK_S
    # …and at every PUBLISHED rung flight the park term is the winner, which is
    # why the re-measure bought no dwell at any rung.
    for rung_flight in (0.4949, 0.5029, 0.6059, 0.7977):
        assert hand_stroke.catch_park_reentry_s(
            vertical_event_vel_mps(rung_flight), 0.9) > \
            DEFAULT_SESSION_DWELL_MARGIN_S, rung_flight


def test_a_slower_catch_lengthens_the_floor_and_the_session_sees_it():
    """``catch/vel_scale`` is a FLOOR term. The catch is armed at
    ``event_vel x scale`` and the catch tail is inversely proportional to it, so
    the operator setting most likely to be reached for when catches are being
    missed — a SLOWER catch — makes the turnaround LONGER. A session that
    ignored the knob would under-state its own floor in exactly that case."""
    at09 = TossSessionSequencer(num_throws=2, flight_time_s=FLIGHT_TIME_MIN_S,
                                catch_vel_scale=0.9)
    at05 = TossSessionSequencer(num_throws=2, flight_time_s=FLIGHT_TIME_MIN_S,
                                catch_vel_scale=0.5)
    assert at05.hand_floor_dwell_s > at09.hand_floor_dwell_s
    # 0 is the "unset" sentinel and resolves to the config default, not to 1.0.
    unset = TossSessionSequencer(num_throws=2, flight_time_s=FLIGHT_TIME_MIN_S)
    assert unset.catch_vel_scale == pytest.approx(
        hw.JB_OP_CATCH_VEL_SCALE_DEFAULT)
    assert unset.hand_floor_dwell_s == pytest.approx(at09.hand_floor_dwell_s)


def test_an_unresolved_flight_time_is_judged_at_the_strictest_flight():
    """0.0 flight time = "nobody resolved it" (standalone/test construction).
    The hand floor is monotonically DECREASING in flight time, so the fallback
    is the C-HAND-3 band FLOOR and not the 0.80 s nominal — fail-closed, the
    same doctrine as ``throw_site_known``. A nominal fallback would under-state
    the floor by 0.16 s for a session that never said how high it throws."""
    unknown = TossSessionSequencer(num_throws=2, flight_time_s=0.0)
    strictest = TossSessionSequencer(num_throws=2,
                                     flight_time_s=FLIGHT_TIME_MIN_S)
    nominal = TossSessionSequencer(num_throws=2, flight_time_s=0.80)
    assert unknown.hand_floor_dwell_s == pytest.approx(
        strictest.hand_floor_dwell_s)
    assert unknown.hand_floor_dwell_s > nominal.hand_floor_dwell_s


def test_the_025s_dwell_is_unreachable_at_every_admitted_flight_time():
    """The operator's original cadence ask. It is not reachable ANYWHERE in the
    C-HAND-3 band: the hand floor bottoms at 0.2505 s at the very TOP of the
    band (T = 1.1485 s, apex 1.62 m) and rises monotonically from there to
    0.4871 s at the floor. Reaching 0.25 s needs a Platform Teensy flash
    changing calcCatch's geometry — deferred by operator decision 3.

    This test replaces test_the_2s_dwell_from_the_phase_brief_is_unachievable,
    which pinned the same doctrine against the now-retired 3.5 s constant."""
    from jugglebot.toss_sequencer import FLIGHT_TIME_MAX_S
    n = 25
    for i in range(n + 1):
        t = FLIGHT_TIME_MIN_S + (FLIGHT_TIME_MAX_S - FLIGHT_TIME_MIN_S) * i / n
        s = TossSessionSequencer(num_throws=2, flight_time_s=t,
                                 catch_vel_scale=0.9)
        assert s.hand_floor_dwell_s > 0.25, (t, s.hand_floor_dwell_s)
    ceiling = TossSessionSequencer(num_throws=2, flight_time_s=FLIGHT_TIME_MAX_S,
                                   catch_vel_scale=0.9)
    assert ceiling.hand_floor_dwell_s == pytest.approx(0.2505, abs=1e-3)
    # …and a goal that asks for it is REFUSED, not quietly stretched. The delay
    # is set from the LIVE floor rather than typed: the delay gate runs before the
    # dwell gate on purpose, so a typed delay that later falls under its own floor
    # turns this into a REJECTED_THROW_DELAY test by accident — which is exactly
    # what the 2026-08-26 D3 floor rise did to the literal 0.40 that stood here.
    asked = TossSessionSequencer(num_throws=2, dwell_time_s=0.25,
                                 throw_delay_s=ceiling.min_throw_delay_s,
                                 flight_time_s=FLIGHT_TIME_MAX_S,
                                 catch_vel_scale=0.9)
    asked.start(0.0)
    assert asked.step(0.0).result.outcome == 'REJECTED_DWELL'


def test_chain_unreachable_rejected_only_when_chaining():
    """Phase E's KNOWN LIMITATION, caught before a ball flies — but a
    single-cycle session has no chain, so the gate must not fire there."""
    s = _session(num_throws=3, chain_site_reachable=False)
    assert s.step(0.0).result.outcome == 'REJECTED_CHAIN_UNREACHABLE'
    solo = _session(num_throws=1, chain_site_reachable=False)
    assert solo.step(0.0).action == SESSION_ACTION_START_CYCLE


def test_reject_order_num_throws_before_dwell_before_chain():
    """Strictest first: a goal that is wrong in several ways names the most
    fundamental one, so the operator fixes the right field."""
    s = TossSessionSequencer(num_throws=0, dwell_time_s=0.1,
                             chain_site_reachable=False)
    s.start(0.0)
    assert s.step(0.0).result.outcome == 'REJECTED_NUM_THROWS'
    s2 = TossSessionSequencer(num_throws=3, dwell_time_s=0.1,
                              chain_site_reachable=False)
    s2.start(0.0)
    assert s2.step(0.0).result.outcome == 'REJECTED_DWELL'


# ── Defaults ──────────────────────────────────────────────────────────────────

def test_zero_dwell_takes_the_config_default():
    s = TossSessionSequencer(num_throws=2, dwell_time_s=0.0,
                             dwell_default_s=DEFAULT_SESSION_DWELL_S)
    assert s.dwell_time_s == pytest.approx(DEFAULT_SESSION_DWELL_S)


def test_zero_throw_delay_takes_the_toss_default():
    s = TossSessionSequencer(num_throws=2, throw_delay_s=0.0)
    assert s.throw_delay_s == pytest.approx(DEFAULT_TOSS_THROW_DELAY_S)


def test_the_all_defaults_combination_is_legal():
    """dwell default 6.0 must clear the floor the throw-delay default 5.0
    implies (5.137 s since the margin was re-based on 2026-08-22; it was 5.60),
    or every zero-field goal would be REJECTED_DWELL.

    The DEFAULT dwell deliberately did NOT move with the floor: a default must
    never jump cadence. Lowering the floor makes a faster rung LEGAL; the ladder
    runbook selects it explicitly, per goal.

    5.1933, not 5.137: an all-defaults session has no resolved flight time, so
    `handoff_margin_s` is judged at the C-HAND-3 band FLOOR (the strictest case,
    the same fail-closed fallback the other two derived floors use) where the
    hand's park re-entry is 0.1933 s and beats the 0.137 s arrival term."""
    s = TossSessionSequencer(num_throws=3)
    s.start(0.0)
    assert s.required_dwell_s == pytest.approx(5.1933, abs=5e-4)
    assert s.dwell_time_s == pytest.approx(6.0)
    assert s.step(0.0).action == SESSION_ACTION_START_CYCLE


# ── S1: exactly one live cycle ────────────────────────────────────────────────

def test_start_cycle_not_reemitted_while_a_cycle_is_live():
    """Two live TossSequencers would double-own the hand on the Teensy's
    last-writer-wins queue and fight over the single catch/armed latch — the
    cross-action busy hazard, reproduced inside one goal."""
    s = _session()
    assert s.step(0.0).action == SESSION_ACTION_START_CYCLE
    assert s.cycle_live is True
    for t in (0.05, 1.0, 100.0):
        d = s.step(t)
        assert d.action == SESSION_ACTION_NONE
        assert d.cycle_index == 1


def test_a_result_for_a_cycle_nobody_started_is_ignored():
    s = _session()
    s.note_cycle_result(_caught(), 5.0, 5.8)      # no cycle live yet
    assert s.catches_confirmed == 0 and s.throws_completed == 0
    assert s.step(0.0).action == SESSION_ACTION_START_CYCLE


# ── Scheduling: the dwell is a quiescent wait BEFORE the cycle (S5) ───────────

def test_next_cycle_starts_at_landing_plus_dwell_minus_delay():
    s = _session(num_throws=2)
    _t_rel, landing = _run_cycle(s, 0.0, _caught())
    want = landing + DWELL - DELAY
    assert s.step(want - 0.001).action == SESSION_ACTION_NONE
    assert s.step(want).action == SESSION_ACTION_START_CYCLE


def test_the_achieved_dwell_equals_the_request_when_the_handoff_is_prompt():
    s = _session(num_throws=2)
    _t1, landing1 = _run_cycle(s, 0.0, _caught())
    start2 = landing1 + DWELL - DELAY
    assert s.step(start2).action == SESSION_ACTION_START_CYCLE
    t_release2 = start2 + DELAY
    s.note_cycle_result(_caught(), t_release2, t_release2 + FLIGHT)
    res = s.step(start2 + 1.0).result
    assert res.cycle_dwell_s[1] == pytest.approx(DWELL)


def test_lateness_is_absorbed_never_aborted():
    """A cycle whose cleanup ran long (a MISS's telemetry-verified retract
    ladder takes seconds) simply reports a LONGER achieved dwell — the session
    never aborts on being late, because being late is harmless and refusing
    would turn a slow service call into a dead sitting."""
    s = _session(num_throws=2, stop_on_miss=False)
    _t1, landing1 = _run_cycle(s, 0.0, _missed())
    very_late = landing1 + DWELL - DELAY + 30.0
    d = s.step(very_late)
    assert d.action == SESSION_ACTION_START_CYCLE
    t_release2 = very_late + DELAY
    s.note_cycle_result(_caught(), t_release2, t_release2 + FLIGHT)
    res = s.step(very_late + 1.0).result
    assert res.cycle_dwell_s[1] == pytest.approx(DWELL + 30.0)


def test_first_cycle_starts_immediately_and_reports_nan_dwell():
    s = _session(num_throws=1)
    assert s.step(0.0).action == SESSION_ACTION_START_CYCLE
    s.note_cycle_result(_caught(), 5.0, 5.8)
    res = s.step(6.0).result
    assert math.isnan(res.cycle_dwell_s[0])


def test_the_beat_is_one_derivation_and_the_schedule_routes_through_it():
    """``next_release_at`` IS the beat — *previous scheduled landing → next
    RELEASE* — and the session's own next-cycle instant is that number minus one
    ``throw_delay_s``, not a second copy of ``landing + dwell``.

    Pinned by SUBSTITUTION rather than by arithmetic: a session whose beat has
    been replaced schedules off the replacement, which is exactly the operation
    Phase C performs (plan § 2.6 — it replaces this method body and nothing
    else in either FSM learns where the beat came from). Two copies of the
    cadence is how a dwell edit lands in the schedule and not in the node's
    cadence clamp, leaving the hand sensor's retention window closing against a
    release that never comes."""
    s = _session(num_throws=2)
    _t_rel, landing = _run_cycle(s, 0.0, _caught())
    assert s.next_release_at(landing) == pytest.approx(landing + DWELL)
    assert s.next_cycle_at == pytest.approx(s.next_release_at(landing) - DELAY)

    # The beat, moved. The schedule follows it because it is READ, not re-derived.
    moved = _session(num_throws=2)
    moved.next_release_at = lambda land: float(land) + DWELL + 0.25
    _t_rel2, landing2 = _run_cycle(moved, 0.0, _caught())
    assert moved.next_cycle_at == pytest.approx(landing2 + DWELL + 0.25 - DELAY)


# ── stop_on_miss (S3) ─────────────────────────────────────────────────────────

def test_stop_on_miss_true_ends_the_session_at_the_cycle_boundary():
    """'Stopping' is literally not starting cycle N+1 — the cycle's OWN
    SAFE_ABORT has already retracted, lowered the latch and gone home before the
    session sees the result, so the stop is immediate and commands nothing."""
    s = _session(num_throws=5, stop_on_miss=True)
    _run_cycle(s, 0.0, _caught())
    d = s.step(100.0)
    assert d.action == SESSION_ACTION_START_CYCLE and d.cycle_index == 2
    s.note_cycle_result(_missed(), 105.0, 105.8)
    d = s.step(106.0)
    assert d.done
    assert d.result.outcome == 'STOPPED_ON_MISS'
    assert d.result.success is False
    assert d.result.throws_completed == 2 and d.result.catches_confirmed == 1
    assert d.result.cycle_outcomes == ['CAUGHT', 'MISSED']
    # …and no third cycle is ever offered.
    assert s.step(1000.0).action == SESSION_ACTION_NONE


def test_stop_on_miss_false_continues_and_completes_unsuccessfully():
    s = _session(num_throws=2, stop_on_miss=False)
    _t1, landing1 = _run_cycle(s, 0.0, _missed())
    start2 = landing1 + DWELL - DELAY
    assert s.step(start2).action == SESSION_ACTION_START_CYCLE
    s.note_cycle_result(_caught(), start2 + DELAY, start2 + DELAY + FLIGHT)
    res = s.step(start2 + 100.0).result
    assert res.outcome == 'COMPLETED'
    assert res.success is False           # completed != clean
    assert res.throws_completed == 2 and res.catches_confirmed == 1


@pytest.mark.parametrize('outcome', [
    'MISSED', 'MISSED_INFEASIBLE_WORKSPACE', 'MISSED_INFEASIBLE_TOO_FAST'])
def test_every_missed_variant_is_the_miss_class(outcome):
    s = _session(num_throws=3, stop_on_miss=True)
    _run_cycle(s, 0.0, _missed(outcome))
    d = s.step(1.0)
    assert d.result.outcome == 'STOPPED_ON_MISS'
    assert d.result.throws_completed == 1     # a ball DID fly


# ── Cycle faults stop regardless of stop_on_miss ──────────────────────────────

@pytest.mark.parametrize('stop_on_miss', [True, False])
@pytest.mark.parametrize('outcome', [
    'REJECTED_HAND_NOT_PARKED', 'REJECTED_POSE_UNKNOWN', 'REJECTED_WORKSPACE',
    'ABORTED_NO_RELEASE', 'ABORTED_PREPARE_FAILED', 'ABORTED_CANCELLED'])
def test_a_faulted_cycle_always_stops_the_session(outcome, stop_on_miss):
    """A REJECTED/ABORTED cycle is a machine fault, not a missed catch.
    Repeating it num_throws times is how one fault becomes N, so it stops even
    with stop_on_miss false — and the failing cycle's own verdict is carried
    verbatim so the operator routes it exactly as for a single Toss."""
    s = _session(num_throws=4, stop_on_miss=stop_on_miss)
    _run_cycle(s, 0.0, TossResult(False, outcome))
    d = s.step(1.0)
    assert d.done
    assert d.result.outcome == 'ABORTED_CYCLE_{}'.format(outcome)
    assert d.result.throws_completed == 0     # nothing flew
    assert d.result.success is False


def test_a_faulted_cycle_preserves_earlier_accounting():
    s = _session(num_throws=4)
    _t1, landing1 = _run_cycle(s, 0.0, _caught(err=3.0, flight=0.79))
    start2 = landing1 + DWELL - DELAY
    assert s.step(start2).action == SESSION_ACTION_START_CYCLE
    s.note_cycle_result(TossResult(False, 'ABORTED_NO_RELEASE'),
                        start2 + DELAY, start2 + DELAY + FLIGHT)
    res = s.step(start2 + 1.0).result
    assert res.outcome == 'ABORTED_CYCLE_ABORTED_NO_RELEASE'
    assert res.throws_completed == 1 and res.catches_confirmed == 1
    assert res.cycle_outcomes == ['CAUGHT', 'ABORTED_NO_RELEASE']
    assert res.cycle_catch_error_mm[0] == pytest.approx(3.0)
    assert math.isnan(res.cycle_catch_error_mm[1])


# ── Happy path + result shape ─────────────────────────────────────────────────

def test_clean_session_completes_successfully():
    s = _session(num_throws=3)
    now = 0.0
    for _ in range(3):
        _t, landing = _run_cycle(s, now, _caught(err=2.5, flight=0.805))
        now = landing + DWELL - DELAY
    res = s.step(now).result
    assert res.outcome == 'COMPLETED' and res.success is True
    assert res.throws_completed == 3 and res.catches_confirmed == 3
    assert res.cycle_outcomes == ['CAUGHT'] * 3
    assert res.cycle_flight_s == pytest.approx([0.805] * 3)
    assert math.isnan(res.cycle_dwell_s[0])
    assert res.cycle_dwell_s[1] == pytest.approx(DWELL)
    assert res.cycle_dwell_s[2] == pytest.approx(DWELL)


def test_finished_is_terminal_and_replays():
    s = _session(num_throws=1)
    _run_cycle(s, 0.0, _caught())
    first = s.step(1.0)
    assert first.done
    again = s.step(2.0)
    assert again.done and again.result is first.result
    assert again.action == SESSION_ACTION_NONE
    # A late cycle report after the terminal must not mutate the verdict.
    s.note_cycle_result(_missed(), 10.0, 10.8)
    assert s.step(3.0).result.throws_completed == 1


def test_phase_strings_match_the_action_spec():
    text = ACTION_FILE.read_text()
    for phase in (SESSION_PHASE_CHECKING, SESSION_PHASE_DWELL):
        assert phase in text, phase
    s = _session()
    assert s.phase == SESSION_PHASE_CHECKING
    s.step(0.0)
    assert s.phase == SESSION_PHASE_DWELL


# ── force_terminal (the node-level exits) ─────────────────────────────────────

def test_force_terminal_preserves_the_accounting():
    """A cancelled/timed-out session must still report what it actually did —
    returning an empty result would erase the operator's evidence."""
    s = _session(num_throws=5)
    _run_cycle(s, 0.0, _caught(err=1.0, flight=0.80))
    res = s.force_terminal('ABORTED_CANCELLED')
    assert res.outcome == 'ABORTED_CANCELLED'
    assert res.throws_completed == 1 and res.catches_confirmed == 1
    assert res.cycle_outcomes == ['CAUGHT']
    assert res.success is False           # 1 of 5 is not a clean session


def test_force_terminal_is_idempotent_first_terminal_wins():
    s = _session(num_throws=1)
    _run_cycle(s, 0.0, _caught())
    natural = s.step(1.0).result
    forced = s.force_terminal('ABORTED_TIMEOUT')
    assert forced is natural and forced.outcome == 'COMPLETED'


# ── success semantics ─────────────────────────────────────────────────────────

@pytest.mark.parametrize('outcome', [
    'REJECTED_NUM_THROWS', 'REJECTED_DWELL', 'REJECTED_CHAIN_UNREACHABLE',
    'ABORTED_CANCELLED', 'ABORTED_TIMEOUT', 'STOPPED_ON_MISS'])
def test_no_non_completed_terminal_can_report_success(outcome):
    """Regression, and the class rather than the case: ``success`` used to be
    computed from the counts alone, so a ``num_throws = 0`` goal satisfied
    ``0 == 0 and 0 == 0`` VACUOUSLY and a REJECTED session reported success —
    which the node turns into ``goal_handle.succeed()`` on a refused goal.
    Success now also requires the COMPLETED terminal, so no terminal that is not
    a clean completion can ever report it, however the counters land."""
    s = TossSessionSequencer(num_throws=0, dwell_time_s=DWELL,
                             throw_delay_s=DELAY)
    s.start(0.0)
    assert s.force_terminal(outcome).success is False


def test_num_throws_zero_rejects_without_vacuous_success():
    s = _session(num_throws=0)
    d = s.step(0.0)
    assert d.result.outcome == 'REJECTED_NUM_THROWS'
    assert d.result.success is False


@pytest.mark.parametrize('num_throws,caught,expect', [
    (1, 1, True), (3, 3, True), (3, 2, False), (1, 0, False)])
def test_success_requires_every_throw_and_every_catch(num_throws, caught,
                                                      expect):
    s = _session(num_throws=num_throws, stop_on_miss=False)
    now = 0.0
    for i in range(num_throws):
        _t, landing = _run_cycle(s, now, _caught() if i < caught else _missed())
        now = landing + DWELL - DELAY
    assert s.step(now).result.success is expect


# ── Config / wire drift guards ────────────────────────────────────────────────

def test_local_constants_match_generated_config():
    """The module literals are the NO-CONFIG fallback only; the node passes the
    generated values in. Pinned equal so the fallback can never quietly become
    a second, different default (the DEFAULT_TOSS_FLIGHT_TIME_S pattern)."""
    assert DEFAULT_SESSION_DWELL_S == pytest.approx(
        float(hw.JB_OP_TOSS_SESSION_DWELL_DEFAULT_S))
    assert DEFAULT_SESSION_DWELL_MARGIN_S == pytest.approx(
        float(hw.JB_OP_TOSS_SESSION_DWELL_MARGIN_S))
    assert DEFAULT_SESSION_MAX_THROWS == int(hw.JB_OP_TOSS_SESSION_MAX_THROWS)


def test_config_default_dwell_clears_the_config_derived_floor():
    """The two shipped config keys must be mutually consistent: the default
    dwell has to clear the floor the default throw delay implies, or the
    all-defaults goal is REJECTED_DWELL on a healthy machine."""
    floor = DEFAULT_TOSS_THROW_DELAY_S + float(
        hw.JB_OP_TOSS_SESSION_DWELL_MARGIN_S)
    assert float(hw.JB_OP_TOSS_SESSION_DWELL_DEFAULT_S) >= floor


def test_dwell_margin_covers_the_measured_sensor_arrival_edge():
    """The margin must cover the possession handoff, and since 2026-08-10 that
    handoff is the HAND SENSOR's arrival edge, not the mocap tracker's CAUGHT
    verdict (C-POSSESS-1 made possession sensor-PRIMARY).

    This test USED to require >= 0.442 + 0.10 = 0.542 s, the worst measured
    tracker latency plus two node ticks. That bound outlived its channel: the
    tracker is now the FALLBACK, and the sensor's empty->held edge carries zero
    debounce (measured 0/0/0 ms, against 232/241/295 ms on the falling edge)
    with the EARLIEST observed edge at +137 ms past the announced landing
    (n=35, three 2026-08-10 bags).

    The old derivation also added two node ticks; that allowance is gone,
    because it double-counted a guarantee invariant S1 already makes
    structurally (START_CYCLE cannot be emitted until note_cycle_result has
    consumed the previous cycle) and it cost 40 ms of the tightest rung.

    NO LONGER PROVISIONAL: the post-FW14 re-measure this test used to wait on
    landed 2026-08-24 (cadence-ladder § 3.1). The band collapsed +137…+798 ⇒
    +87.6…+554.7 ms (n=33, four bags) and this moved 0.137 ⇒ 0.087 with it. It
    bought no cadence — see ``handoff_margin_s``, where the park term now wins
    the max() at every published rung."""
    from jugglebot.ball_possession import (ARRIVAL_BAND_MAX_S,
                                           ARRIVAL_BAND_MIN_S)
    assert ARRIVAL_BAND_MIN_S == pytest.approx(0.087)
    assert DEFAULT_SESSION_DWELL_MARGIN_S == pytest.approx(ARRIVAL_BAND_MIN_S)
    assert float(hw.JB_OP_TOSS_SESSION_DWELL_MARGIN_S) == pytest.approx(
        DEFAULT_SESSION_DWELL_MARGIN_S)
    # It is a LOWER bound on the verdict, not an upper one — sizing it on the
    # LATEST edge (+554.7 ms) would put the floor 0.47 s higher and forbid every
    # rung. What protects the late-seat case is the C-POSSESS-1 machinery.
    assert DEFAULT_SESSION_DWELL_MARGIN_S < ARRIVAL_BAND_MAX_S


def test_stop_on_miss_wire_default_is_true():
    """Operator decision (c), 2026-07-28. The IDL default is LOAD-BEARING: an
    omitted field must mean STOP, never CONTINUE, because a miss leaves a loose
    ball on the floor under a machine that is about to stroke again. Pinned in
    all three places it exists — the wire, the FSM ctor, and the mock."""
    from jugglebot_interfaces.action import TossContinuous
    text = ACTION_FILE.read_text()
    assert re.search(r'^bool\s+stop_on_miss\s+true\s*(#.*)?$', text,
                     re.MULTILINE), 'the .action IDL default must be true'
    assert TossSessionSequencer(num_throws=1).stop_on_miss is True
    assert TossContinuous.Goal().stop_on_miss is True


# ── The MISS-path cleanup floor ───────────────────────────────────────────────

def test_a_continued_miss_waits_for_its_own_cleanup_ladder():
    """A MISSED cycle the session continues past hands over through a whole
    SAFE_ABORT ladder whose every rung returns on a SERVICE ACK — the retract is
    still descending and the 2.0 s go_home profile is still traversing when
    _run_toss_cycle returns. The plain landing + dwell - delay arithmetic starts
    cycle N+1 inside that teardown; at the SHIPPED DEFAULTS it already does
    (6.0 - 5.0 = 1.0 s vs a 2.8 s ladder). Without the floor the next cycle
    reads a mid-traverse pose as its throw site and meets an unparked hand —
    REJECTED_HAND_NOT_PARKED, a machine-fault verdict for a cadence fault."""
    s = _session(num_throws=2, stop_on_miss=False,
                 dwell_time_s=DEFAULT_SESSION_DWELL_S,
                 throw_delay_s=DEFAULT_TOSS_THROW_DELAY_S)
    _t1, landing1 = _run_cycle(s, 0.0, _missed())
    naive = landing1 + DEFAULT_SESSION_DWELL_S - DEFAULT_TOSS_THROW_DELAY_S
    assert naive < landing1 + DEFAULT_SESSION_MISS_CLEANUP_S   # the defect
    assert s.step(naive).action == SESSION_ACTION_NONE
    assert s.step(landing1 + DEFAULT_SESSION_MISS_CLEANUP_S
                  ).action == SESSION_ACTION_START_CYCLE


def test_the_cleanup_floor_never_shortens_a_longer_cadence():
    """It is a FLOOR, not a schedule: a session already dwelling past the ladder
    is bit-unchanged, so the fix cannot make any cadence faster."""
    s = _session(num_throws=2, stop_on_miss=False)     # DWELL 8.0 - DELAY 5.0
    _t1, landing1 = _run_cycle(s, 0.0, _missed())
    want = landing1 + DWELL - DELAY
    assert want > landing1 + DEFAULT_SESSION_MISS_CLEANUP_S
    assert s.step(want - 0.001).action == SESSION_ACTION_NONE
    assert s.step(want).action == SESSION_ACTION_START_CYCLE


def test_the_cleanup_floor_does_not_apply_after_a_caught_cycle():
    """A CAUGHT cycle's ACTION_STAY teardown commands NOTHING — no retract, no
    go_home — so the CAUGHT handoff is the dwell_margin one and must not be
    slowed by the MISS ladder."""
    s = _session(num_throws=2, dwell_time_s=DELAY + 0.6)
    _t1, landing1 = _run_cycle(s, 0.0, _caught())
    want = landing1 + 0.6
    assert want < landing1 + DEFAULT_SESSION_MISS_CLEANUP_S
    assert s.step(want).action == SESSION_ACTION_START_CYCLE


def test_the_miss_cleanup_floor_is_derived_from_its_sources():
    """Pinned against the three landed numbers it is made of, and each of those
    against the file it comes from — so an edit to the go_home profile duration
    or the settle window cannot leave this floor silently wrong."""
    assert DEFAULT_SESSION_MISS_CLEANUP_S == pytest.approx(
        CATCH_CONFIRM_WINDOW_S + SAFE_ABORT_LADDER_S + GO_HOME_DURATION_S
        + 2.0 * NODE_LOOP_PERIOD_S)
    # SAFE_ABORT_LADDER_S is the term added 2026-08-26 (owner decision D3): the
    # ladder's own dispatch cost between the MISSED verdict and the go_home
    # INSTALL, which was charged at ZERO. Its acceptance is an ABSENCE in the next
    # bag — trajectory_node prints "catch latch armed mid-move — installed a
    # graceful stop (move silenced)" whenever the next cycle's PREPARE arms while
    # the recentre is still traversing, and that line fired on 10 of the 16
    # post-MISS cycles of 2026-08-26_14-25-16.
    assert SAFE_ABORT_LADDER_S == pytest.approx(4.0 * NODE_LOOP_PERIOD_S)
    # 2.80 s since 2026-08-26 (2.60 s from the 2026-08-24 band re-measure, 2.84 s
    # before it, 2.80 s before 2026-08-21). CATCH_CONFIRM_WINDOW_S moved 0.70 -> 0.80
    # when it became DERIVED from ball_possession.ARRIVAL_BAND_MAX_S (census D7):
    # a sensor-primary possession verdict needs a MISSED deadline that outlasts
    # the band a real seat edge lands in (+137..+798 ms then), and 0.70 sat 98 ms
    # under its ceiling. The 0.10 s that added to the MISS-path floor was the
    # wrong direction for cadence and was stated as such rather than hidden — and
    # the 2026-08-24 post-FW14 band re-measure has now given back more than it
    # ever cost: 0.80 -> 0.56 takes this floor to 2.60 s. The census's F1 rung
    # (re-deriving the floor from COMPLETION rather than from service acks) is
    # still the larger win and is still open.
    assert DEFAULT_SESSION_MISS_CLEANUP_S == pytest.approx(2.80)
    # The floor still covers the non-release teardown it is reused for
    # (toss_session ~line 1221): release_grace 0.5 + the ladder + go_home 2.0 +
    # 2 loop periods. The band re-measure narrowed that margin from 300 ms to
    # 60 ms without inverting it, and D3 added the SAME ladder term to both sides,
    # so it is still 60 ms — this is where a further cut would first be caught.
    assert DEFAULT_SESSION_MISS_CLEANUP_S >= (
        0.5 + SAFE_ABORT_LADDER_S + GO_HOME_DURATION_S + 2.0 * NODE_LOOP_PERIOD_S)
    assert re.search(r"declare_parameter\(\s*'go_home_duration_s',\s*2\.0\s*\)",
                     TRAJECTORY_NODE.read_text()), (
        'trajectory_node go_home_duration_s default moved — GO_HOME_DURATION_S '
        'must follow it')
    assert re.search(r'^_TICK_S\s*=\s*0\.02\b', COORDINATOR_NODE.read_text(),
                     re.MULTILINE), (
        'reload_coordinator_node._TICK_S moved — NODE_TICK_S must follow it')


# ── throw_delay floor (the floor the session advertises is only real if the
#    delay itself is gated — and it must be gated at the SAME number the cycle
#    uses, in BOTH directions) ──────────────────────────────────────────────────

@pytest.mark.parametrize('delay', [0.0001, 0.05, 0.20])
def test_throw_delay_below_the_cycle_fsm_gates_is_refused(delay):
    """Refused at SESSION checking, before a cycle is built. Otherwise a goal
    with an illegal delay satisfies dwell >= delay + margin, is ACCEPTED,
    installs a whole cycle's per-goal state, and then dies
    REJECTED_CANT_MAKE_LEAD naming a field the operator did not set wrong.

    The parameters span BOTH cycle gates now that the flat 3.5 s is retired:
    0.0001 and 0.05 are under the goal-storm debounce, 0.20 is over the debounce
    but under the derived :642 dispatch budget (0.337 s at the band floor this
    session is judged at). Mirroring only the debounce would re-open this hole
    one order of magnitude narrower."""
    s = TossSessionSequencer(num_throws=3, dwell_time_s=delay + 1.0,
                             throw_delay_s=delay)
    s.start(0.0)
    assert s.step(0.0).result.outcome == 'REJECTED_THROW_DELAY'


def test_the_session_delay_gate_is_neither_looser_nor_stricter_than_the_cycle():
    """One expression, mirrored — and since 2026-08-23 literally one function.

    Looser and the session accepts a goal the cycle kills mid-sequence; stricter
    and it refuses a cadence the machine can make. Both are verdicts that name
    the wrong field.

    The session mirrors the STEADY-STATE cycle: ``positioning_move=False``, the
    chained cycle that takes the census-B1 skip. That is what a CADENCE is made
    of, and it is why the session does not charge the 0.460 s moving budget that
    only the FIRST cycle of a sitting pays (the node grants that first cycle the
    extra lead — see ``_build_toss_cycle``). The cycle's own gate still charges
    the real per-cycle predicate, which is what keeps
    ABORTED_CANT_MAKE_RELEASE statically unreachable."""
    from jugglebot.toss_sequencer import TossSequencer
    for flight in (FLIGHT_TIME_MIN_S, 0.80):
        s = TossSessionSequencer(num_throws=3, flight_time_s=flight,
                                 **NO_ILC_TRIM)
        cycle = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                              flight_time_s=flight, throw_delay_s=5.0,
                              positioning_move_expected=False)
        assert s.min_throw_delay_s == pytest.approx(
            cycle.min_throw_delay_for_cycle_s)
        # …and it is the dispatch budget PLUS the sequence, not either alone
        # (plus the microsecond of representation slack that makes the floor
        # strictly sufficient rather than exactly-equal — see
        # toss_sequencer.FLOOR_REPRESENTATION_SLACK_S).
        assert s.min_throw_delay_s == pytest.approx(
            cycle.min_event_delay_for_throw_s + pre_dispatch_budget_s(False)
            + FLOOR_REPRESENTATION_SLACK_S, abs=1e-12)
        assert s.min_throw_delay_s == pytest.approx(
            min_throw_delay_for_release_s(s.floor_event_vel_mps, False))
        assert s.min_throw_delay_s > cycle.min_event_delay_for_throw_s
        assert s.min_throw_delay_s > TOSS_DISPATCH_DEBOUNCE_S


def test_throw_delay_exactly_at_the_floor_is_legal():
    """The session must not be stricter than the cycle it repeats."""
    s = TossSessionSequencer(num_throws=3, flight_time_s=FLIGHT)
    floor = s.min_throw_delay_s
    ok = TossSessionSequencer(num_throws=3, flight_time_s=FLIGHT,
                              throw_delay_s=floor,
                              dwell_time_s=floor + DEFAULT_SESSION_DWELL_MARGIN_S
                              + 1.0)
    ok.start(0.0)
    assert ok.step(0.0).action == SESSION_ACTION_START_CYCLE


def test_the_throw_delay_gate_precedes_the_dwell_gate():
    """A goal wrong in both ways names the delay, because the dwell FLOOR is
    derived FROM the delay — telling the operator to raise a dwell that is only
    too small because the delay is illegal sends them to the wrong field."""
    s = TossSessionSequencer(num_throws=3, throw_delay_s=0.05, dwell_time_s=0.1)
    s.start(0.0)
    assert s.step(0.0).result.outcome == 'REJECTED_THROW_DELAY'


def test_action_result_fields_match_the_session_result():
    """Every field the FSM produces must have somewhere to go on the wire —
    otherwise the operator silently loses the per-cycle evidence a sitting is
    scored from."""
    text = ACTION_FILE.read_text()
    for wire_field in ('bool success', 'string outcome',
                       'int32 throws_completed', 'int32 catches_confirmed',
                       'string[] per_cycle_outcomes',
                       'float64[] per_cycle_catch_error_mm',
                       'float64[] per_cycle_flight_s',
                       'float64[] per_cycle_dwell_s'):
        assert wire_field in text, wire_field
    empty = TossSessionResult(success=False, outcome='X')
    assert empty.cycle_outcomes == [] and empty.cycle_dwell_s == []


# ══ 2d — the auto-reload interlude, the NO_RELEASE retry, the two counters ════
#
# Every test below drives the FSM only. The node's half of the interlude (the
# observation-driven gate rungs, the verified recentre, the reload FSM itself and
# the Layer-1.5 dwell reads) lives in test_toss_continuous_node.py, because those
# are seams onto the graph and this module takes no observations at all.


def _no_ball():
    return TossResult(False, 'REJECTED_NO_BALL', float('nan'), float('nan'))


def _no_release():
    return TossResult(False, 'ABORTED_NO_RELEASE', float('nan'), float('nan'))


def _reload_session(**kw):
    params = dict(num_throws=3, stop_on_miss=False,
                  on_empty_cup=ON_EMPTY_CUP_RELOAD)
    params.update(kw)
    return _session(**params)


def _drive_to_reload(session, now=0.0, result=None):
    """Run one cycle that ends in the given terminal, then step once. Returns
    the decision that step produced."""
    d = session.step(now)
    assert d.action == SESSION_ACTION_START_CYCLE, d
    session.note_cycle_result(result if result is not None else _no_ball(),
                              now + DELAY, now + DELAY + FLIGHT)
    return session.step(now + DELAY)


# ── on_empty_cup resolution: the whitelist, not a blacklist ───────────────────

@pytest.mark.parametrize('raw', [
    '', None, 'STOP', 'stop', 'reloadd', 'RELOAD ME', 'NONE', 0, 'Reload!',
])
def test_anything_that_is_not_exactly_reload_resolves_to_stop(raw):
    """The IDL default is STOP and the node re-applies it. Whitelisting the one
    dangerous value is what makes a typo fail in the SAFE direction: the failure
    mode being prevented is an autonomous BB reload — a real ball thrown at a
    real machine — started by a field nobody meant to set."""
    assert resolve_on_empty_cup(raw) == ON_EMPTY_CUP_STOP


@pytest.mark.parametrize('raw', ['RELOAD', 'reload', ' Reload '])
def test_reload_is_accepted_case_and_whitespace_insensitively(raw):
    assert resolve_on_empty_cup(raw) == ON_EMPTY_CUP_RELOAD


def test_on_empty_cup_wire_default_is_stop():
    """Pinned in all three places it exists — the wire, the FSM ctor and the
    resolver — exactly as stop_on_miss's default is."""
    from jugglebot_interfaces.action import TossContinuous
    text = ACTION_FILE.read_text()
    assert re.search(r'^string\s+on_empty_cup\s+"STOP"\s*(#.*)?$', text,
                     re.MULTILINE), 'the .action IDL default must be "STOP"'
    assert TossSessionSequencer(num_throws=1).on_empty_cup == ON_EMPTY_CUP_STOP
    assert resolve_on_empty_cup(TossContinuous.Goal().on_empty_cup) \
        == ON_EMPTY_CUP_STOP


def test_omitted_on_empty_cup_stops_the_session():
    """THE gate the operator asked for: a session that did not ask for reloads
    behaves exactly as it did before 2026-08-11 — REJECTED_NO_BALL stops it,
    verbatim, and no interlude is ever emitted."""
    s = _session(num_throws=3, stop_on_miss=False)      # ctor default STOP
    d = _drive_to_reload(s, 0.0)
    assert d.done and d.result.outcome == 'ABORTED_CYCLE_REJECTED_NO_BALL'
    assert d.action == SESSION_ACTION_NONE
    assert d.result.reloads_used == 0


def test_reload_policy_emits_the_interlude_instead_of_stopping():
    s = _reload_session()
    d = _drive_to_reload(s, 0.0)
    assert not d.done
    assert d.action == SESSION_ACTION_RELOAD
    assert d.phase == SESSION_PHASE_RELOAD


def test_the_interlude_is_re_emitted_until_the_node_answers():
    """The same shape as START_CYCLE-while-a-cycle-is-live: the FSM never starts
    a cycle behind the node's back while an interlude is outstanding."""
    s = _reload_session()
    _drive_to_reload(s, 0.0)
    for t in (DELAY + 1.0, DELAY + 100.0):
        d = s.step(t)
        assert d.action == SESSION_ACTION_RELOAD, t


def test_only_rejected_no_ball_opens_the_interlude(
):
    """The trigger is deliberately ONE terminal — the only toss terminal where
    the FSM provably commanded nothing (minted in CHECKING, terminal action
    ACTION_NONE). Every other REJECTED_*/ABORTED_* still stops the session, so
    an interlude can never start from a machine mid-teardown."""
    for outcome in ('REJECTED_HAND_NOT_PARKED', 'REJECTED_BALL_UNKNOWN',
                    'ABORTED_PREPARE_FAILED', 'REJECTED_NOT_LEVELLED'):
        s = _reload_session()
        d = _drive_to_reload(
            s, 0.0, TossResult(False, outcome, float('nan'), float('nan')))
        assert d.done, outcome
        assert d.result.outcome == 'ABORTED_CYCLE_{}'.format(outcome)


# ── The budget ────────────────────────────────────────────────────────────────

def test_budget_exhaustion_stops_the_session_closed():
    """max_reloads is the ONLY machine-side fence on ball supply (there is no
    ball-count anywhere on ball_butler_node), so its exhaustion must fail the
    session CLOSED rather than keep stroking at an empty magazine."""
    s = _reload_session(num_throws=10, max_reloads=1)
    d = _drive_to_reload(s, 0.0)
    assert d.action == SESSION_ACTION_RELOAD
    s.note_reload_result(True, attempts=1)
    assert s.reloads_used == 1 and s.reload_budget_remaining == 0
    # second drop: no budget left
    d = _drive_to_reload(s, 100.0)
    assert d.done and d.result.outcome == OUTCOME_STOPPED_RELOAD_BUDGET
    assert d.result.reloads_used == 1


def test_zero_budget_refuses_the_first_interlude():
    s = _reload_session(max_reloads=0)
    d = _drive_to_reload(s, 0.0)
    assert d.done and d.result.outcome == OUTCOME_STOPPED_RELOAD_BUDGET


def test_a_negative_budget_behaves_as_zero_not_as_infinite():
    """The node refuses a negative max_reloads by name, so this can only be
    reached by constructing the FSM directly — it must still fail CLOSED."""
    s = _reload_session(max_reloads=-3)
    assert s.reload_budget_remaining == 0
    d = _drive_to_reload(s, 0.0)
    assert d.done and d.result.outcome == OUTCOME_STOPPED_RELOAD_BUDGET


def test_a_retrying_interlude_charges_every_attempt_to_the_budget():
    """The BB not-positioned-in-time retry re-enters the reload FSM, and every
    re-entry is a real BB ball — so it is charged to the same fence."""
    s = _reload_session(num_throws=10, max_reloads=3)
    _drive_to_reload(s, 0.0)
    s.note_reload_result(True, attempts=2)
    assert s.reloads_used == 2 and s.reload_budget_remaining == 1


def test_a_failed_interlude_stops_with_the_nodes_own_code():
    s = _reload_session()
    _drive_to_reload(s, 0.0)
    s.note_reload_result(False, attempts=1, stop_code='STOPPED_BB_NOT_READY')
    d = s.step(1000.0)
    assert d.done and d.result.outcome == 'STOPPED_BB_NOT_READY'
    assert d.result.reloads_used == 1


def test_a_failed_interlude_with_no_code_still_stops():
    """An unnamed failure is still a failure: continuing would stroke over a cup
    nobody proved has a ball."""
    s = _reload_session()
    _drive_to_reload(s, 0.0)
    s.note_reload_result(False)
    d = s.step(1000.0)
    assert d.done and d.result.outcome == OUTCOME_STOPPED_RELOAD_FAILED


def test_note_reload_result_is_ignored_when_no_interlude_is_pending():
    s = _reload_session()
    s.step(0.0)
    s.note_reload_result(False, stop_code='STOPPED_BB_NOT_READY')
    assert s.reloads_used == 0
    assert not s.finished


# ── The floor tally ───────────────────────────────────────────────────────────

def test_the_floor_tally_counts_every_drop_including_the_exhausting_one():
    """Nothing on the robot can see the floor, so this is the session's own tally
    of interludes entered. It counts the drop that exhausted the budget too — a
    count that skipped it would under-report exactly the sitting that most needs
    the operator to clear the floor."""
    s = _reload_session(num_throws=10, max_reloads=1)
    _drive_to_reload(s, 0.0)
    assert s.floor_balls == 1
    s.note_reload_result(True, attempts=1)
    _drive_to_reload(s, 100.0)                     # budget-exhausting drop
    assert s.floor_balls == 2


def test_every_retried_attempt_is_a_ball_on_the_floor_too():
    """**The undercount D2 opened and the 2026-08-26 audit closed (W7).**

    ``_reload_precheck`` charges the floor tally ONE ball when the interlude is
    entered, which was the whole truth before D2: an interlude threw once. D2 lets
    one interlude spend the entire budget on failed throws, and every one of those
    is a real BB ball that did not end up in the cup. The session's floor and
    budget rungs are evaluated ONCE per interlude and are deliberately EXCLUDED
    from the per-attempt ladder (``_reload_interlude_gate`` owns only the
    observation rungs), so nothing else in the system can charge them — which is
    why ``note_reload_result`` charges ``attempts - 1`` itself.

    Before the fix a 3-attempt interlude advanced ``floor_balls`` by 1, and
    ``floor_pause_every`` therefore never fired on the sitting with the most balls
    down. Nothing on the robot can see the floor, so an undercount here is
    invisible until someone trips over it."""
    s = _reload_session(num_throws=20, max_reloads=5, floor_pause_every=0)
    _drive_to_reload(s, 0.0)
    assert s.floor_balls == 1                      # the precheck's charge
    s.note_reload_result(True, attempts=3)         # two failed throws, then a catch
    assert s.floor_balls == 3
    assert s.reloads_used == 3
    # A single-attempt interlude is unchanged — the precheck already charged it,
    # and `attempts - 1` is 0. This is the non-regression half.
    _drive_to_reload(s, 100.0)
    assert s.floor_balls == 4
    s.note_reload_result(True, attempts=1)
    assert s.floor_balls == 4


def test_a_multi_attempt_interlude_trips_the_floor_pause_it_earned():
    """W7's consequence, at the gate the operator actually meets.

    ``floor_pause_every=3`` with one 3-attempt interlude: three balls are down, so
    the NEXT drop must stop the session for a floor clear. Under the undercount
    the tally read 1 and the session sailed past."""
    s = _reload_session(num_throws=20, max_reloads=6, floor_pause_every=3)
    _drive_to_reload(s, 0.0)
    s.note_reload_result(True, attempts=3)
    assert s.floor_balls == 3
    d = _drive_to_reload(s, 100.0)
    assert d.done and d.result.outcome == OUTCOME_STOPPED_FLOOR_CLEAR_REQUIRED


def test_floor_pause_stops_the_session_cleanly_between_cycles():
    s = _reload_session(num_throws=20, max_reloads=10, floor_pause_every=2)
    _drive_to_reload(s, 0.0)
    s.note_reload_result(True, attempts=1)
    d = _drive_to_reload(s, 100.0)
    assert d.done and d.result.outcome == OUTCOME_STOPPED_FLOOR_CLEAR_REQUIRED


def test_floor_pause_of_zero_disables_the_pause():
    s = _reload_session(num_throws=20, max_reloads=10, floor_pause_every=0)
    _drive_to_reload(s, 0.0)
    s.note_reload_result(True, attempts=1)
    d = _drive_to_reload(s, 100.0)
    assert d.action == SESSION_ACTION_RELOAD


def test_the_budget_code_wins_over_the_floor_code():
    """Ordered per § 3.9: an operator whose budget is gone must be told THAT,
    because clearing the floor would not let the session continue."""
    s = _reload_session(num_throws=20, max_reloads=1, floor_pause_every=2)
    _drive_to_reload(s, 0.0)
    s.note_reload_result(True, attempts=1)
    d = _drive_to_reload(s, 100.0)
    assert d.result.outcome == OUTCOME_STOPPED_RELOAD_BUDGET


# ── ABORTED_NO_RELEASE — the retry the operator REOPENED (decision 6) ─────────

def test_no_release_retries_once_on_a_valid_held_sensor():
    """The sensor answers the only question that made a blind retry unsafe: with
    a VALID HELD read the ball is demonstrably still in the cup, so the airborne
    -ball hazard is structurally absent."""
    s = _session(num_throws=5)
    d = s.step(0.0)
    assert d.action == SESSION_ACTION_START_CYCLE
    s.note_cycle_result(_no_release(), DELAY, DELAY + FLIGHT,
                        ball_evidence=EVIDENCE_SEATED_NAME)
    d = s.step(_after_cleanup(DELAY + FLIGHT))
    assert d.action == SESSION_ACTION_START_CYCLE
    assert s.cycle_is_retry is True


@pytest.mark.parametrize('evidence', ['UNKNOWN', 'EMPTY', None, ''])
def test_no_release_does_not_retry_on_unknown_or_empty(evidence):
    """UNKNOWN and EMPTY both refuse. Blindness is not evidence, and an EMPTY cup
    after a non-release means the ball went somewhere nobody watched — which is
    the D9 hazard the deferral existed for."""
    s = _session(num_throws=5)
    s.step(0.0)
    s.note_cycle_result(_no_release(), DELAY, DELAY + FLIGHT,
                        ball_evidence=evidence)
    d = s.step(DELAY)
    assert d.done
    assert d.result.outcome == 'ABORTED_CYCLE_ABORTED_NO_RELEASE'


def test_two_consecutive_no_releases_stop_the_session():
    """The epidemic gauge, preserved: one non-release is a stroke that did not
    release; two in a row is a plant fault repeating, and repeating a fault
    num_throws times is how one fault becomes N."""
    s = _session(num_throws=5)
    s.step(0.0)
    s.note_cycle_result(_no_release(), DELAY, DELAY + FLIGHT,
                        ball_evidence=EVIDENCE_SEATED_NAME)
    d = s.step(_after_cleanup(DELAY + FLIGHT))
    assert d.action == SESSION_ACTION_START_CYCLE          # the retry
    s.note_cycle_result(_no_release(), 2 * DELAY, 2 * DELAY + FLIGHT,
                        ball_evidence=EVIDENCE_SEATED_NAME)
    d = s.step(2 * DELAY)
    assert d.done and d.result.outcome == 'ABORTED_CYCLE_ABORTED_NO_RELEASE'


def test_the_no_release_streak_is_CONSECUTIVE_not_cumulative():
    """A clean cycle between two non-releases resets the gauge: the rule is
    about a fault REPEATING, not about a session's lifetime total."""
    s = _session(num_throws=8, stop_on_miss=False)
    s.step(0.0)
    s.note_cycle_result(_no_release(), DELAY, DELAY + FLIGHT,
                        ball_evidence=EVIDENCE_SEATED_NAME)
    s.step(_after_cleanup(DELAY + FLIGHT))                 # retry starts
    s.note_cycle_result(_caught(), 2 * DELAY, 2 * DELAY + FLIGHT)
    d = s.step(2 * DELAY + 100.0)
    assert d.action == SESSION_ACTION_START_CYCLE
    s.note_cycle_result(_no_release(), 3 * DELAY, 3 * DELAY + FLIGHT,
                        ball_evidence=EVIDENCE_SEATED_NAME)
    d = s.step(_after_cleanup(3 * DELAY + FLIGHT))
    assert d.action == SESSION_ACTION_START_CYCLE          # retried again
    assert s.cycle_is_retry is True


def test_the_no_release_retry_waits_for_its_own_cleanup_ladder():
    """AUDIT FIX 2026-08-11. The retry branch used to `return` without touching
    `_next_cycle_at`, leaving it at the PREVIOUS cycle's already-past instant —
    so the retry started on the very next FSM tick, immediately after a
    SAFE_ABORT ladder that had merely *dispatched* the retract and the go_home.

    An ABORTED_NO_RELEASE terminates in PHASE_THROWING with the platform
    positioned and the latch raised, so `_terminal_action` returns
    ACTION_SAFE_ABORT — the IDENTICAL ladder a continued MISS tears down
    through, and therefore the identical floor. Beyond the two refusals the MISS
    floor prevents (a mid-traverse throw site A, REJECTED_HAND_NOT_PARKED), this
    branch had a third cost: the retry would normally die
    REJECTED_HAND_NOT_PARKED, which is not ABORTED_NO_RELEASE, so the "two
    consecutive non-releases stop the session" epidemic gauge could never fire
    and the operator would be routed to the wrong subsystem."""
    s = _session(num_throws=5)
    assert s.step(0.0).action == SESSION_ACTION_START_CYCLE
    landing = DELAY + FLIGHT
    s.note_cycle_result(_no_release(), DELAY, landing,
                        ball_evidence=EVIDENCE_SEATED_NAME)
    # The pre-fix behaviour: the very next tick after the terminal.
    assert s.step(landing).action == SESSION_ACTION_NONE
    assert s.step(_after_cleanup(landing) - 0.001).action \
        == SESSION_ACTION_NONE
    d = s.step(_after_cleanup(landing))
    assert d.action == SESSION_ACTION_START_CYCLE
    assert s.cycle_is_retry is True


def test_the_retry_floor_is_the_same_constant_the_miss_path_uses():
    """One constant, three continuations past the SAFE_ABORT ladder (continued
    MISS, NO_RELEASE retry, reload interlude rung 4). Pinning the retry against
    `next_cycle_at` rather than against a literal is what stops a future edit to
    the go_home profile from fixing two of the three and leaving the third."""
    s = _session(num_throws=5)
    s.step(0.0)
    landing = DELAY + FLIGHT
    s.note_cycle_result(_no_release(), DELAY, landing,
                        ball_evidence=EVIDENCE_SEATED_NAME)
    assert s.next_cycle_at == pytest.approx(
        landing + DEFAULT_SESSION_MISS_CLEANUP_S)


def test_the_seated_evidence_string_matches_ball_possession():
    """This module is pure and takes the evidence as a caller-supplied string —
    a drift guard, not a runtime import, is what keeps the two equal."""
    from jugglebot.ball_possession import EVIDENCE_SEATED
    assert EVIDENCE_SEATED_NAME == EVIDENCE_SEATED


# ── The inherited flags: exactly one cycle wears each (guards G10 / G11) ──────

def test_reload_settle_is_worn_by_exactly_one_cycle():
    s = _reload_session(num_throws=5)
    _drive_to_reload(s, 0.0)
    s.note_reload_result(True, attempts=1)
    d = s.step(100.0)
    assert d.action == SESSION_ACTION_START_CYCLE
    assert s.cycle_reload_settle is True
    s.note_cycle_result(_caught(), 100.0 + DELAY, 100.0 + DELAY + FLIGHT)
    d = s.step(200.0)
    assert d.action == SESSION_ACTION_START_CYCLE
    assert s.cycle_reload_settle is False


def test_retry_flag_is_worn_by_exactly_one_cycle():
    s = _session(num_throws=5)
    s.step(0.0)
    s.note_cycle_result(_no_release(), DELAY, DELAY + FLIGHT,
                        ball_evidence=EVIDENCE_SEATED_NAME)
    s.step(_after_cleanup(DELAY + FLIGHT))
    assert s.cycle_is_retry is True
    s.note_cycle_result(_caught(), 2 * DELAY, 2 * DELAY + FLIGHT)
    s.step(2 * DELAY + 100.0)
    assert s.cycle_is_retry is False


# ── The completion test: throws, not cycle indices ───────────────────────────

def test_completion_is_keyed_on_throws_so_a_reload_costs_no_data_point():
    """A drop costs a reload, not one of the num_throws data points the operator
    asked for. The change is behaviour-preserving for every pre-2026-08-11
    session (see the comment at the check): the only outcomes that do not
    increment `throws` are the REJECTED_*/ABORTED_* family, and every one of
    those stopped the session before reaching it."""
    s = _reload_session(num_throws=2, max_reloads=2)
    _run_cycle(s, 0.0, _caught())
    d = _drive_to_reload(s, 100.0)                  # cycle 2 finds an empty cup
    assert d.action == SESSION_ACTION_RELOAD
    s.note_reload_result(True, attempts=1)
    d = s.step(200.0)
    assert d.action == SESSION_ACTION_START_CYCLE   # cycle 3 — still owed a toss
    s.note_cycle_result(_caught(), 200.0 + DELAY, 200.0 + DELAY + FLIGHT)
    d = s.step(300.0)
    assert d.done and d.result.outcome == 'COMPLETED'
    assert d.result.success is True
    assert d.result.throws_completed == 2 and d.result.catches_confirmed == 2
    assert d.result.cycle_outcomes == ['CAUGHT', 'REJECTED_NO_BALL', 'CAUGHT']


def test_stop_on_miss_semantics_are_unchanged_by_the_reload_policy():
    """The MISSED class is still governed by stop_on_miss ALONE — on_empty_cup
    touches a different terminal, and a MISSED cycle's SAFE_ABORT ladder is
    exactly the state an interlude must never be entered from."""
    for policy in (ON_EMPTY_CUP_STOP, ON_EMPTY_CUP_RELOAD):
        s = _session(num_throws=3, stop_on_miss=True, on_empty_cup=policy)
        _run_cycle(s, 0.0, _missed())
        d = s.step(DELAY)
        assert d.done and d.result.outcome == 'STOPPED_ON_MISS', policy
        s2 = _session(num_throws=3, stop_on_miss=False, on_empty_cup=policy)
        _run_cycle(s2, 0.0, _missed())
        d2 = s2.step(1000.0)
        assert d2.action == SESSION_ACTION_START_CYCLE, policy


def test_reload_phase_string_is_on_the_wire():
    assert SESSION_PHASE_RELOAD in ACTION_FILE.read_text()


def test_reloads_used_reaches_the_action_result():
    assert 'int32 reloads_used' in ACTION_FILE.read_text()
    assert TossSessionResult(success=False, outcome='X').reloads_used == 0
