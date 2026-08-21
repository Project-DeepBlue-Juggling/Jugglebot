"""Unit tests for the pure-Python reload FSM (jugglebot.reload_sequencer).

No ROS: the FSM is driven with synthesized observations + discrete events, so every
precondition reject, the reload sub-sequence, the throw/announcement/catch path, the
PRIME / PREPARE / RECENTER / SAFE_ABORT actions, and every abort is exercised
deterministically.

ORDERING (reworked 2026-07-23 after the first hardware session): every Jugglebot-side
arming step happens BEFORE the throw is committed — CHECKING emits PRIME_HAND the
moment preconditions pass (the hand's ~0.75 s smooth-move must never race the 0.878 s
flight), PREPARING raises the catch latch and waits for the node-confirmed outcome,
and only then does AIMING send ``bb/throw_at_target``. BB's firmware countdown has no
abort opcode, so an arming failure must abort while there is still nothing to abort
on the BB side. A standing catch-target infeasibility no longer terminates mid-flight
(the primed, parked cup still catches a dead-centre throw — observed twice on
hardware); it resolves the outcome at settle.
"""

from __future__ import annotations

import math

import pytest

from jugglebot.reload_sequencer import (
    ACTION_CALL_RELOAD,
    ACTION_NONE,
    CATCH_CONFIRM_WINDOW_S,
    ACTION_PREPARE_CATCH,
    ACTION_PRIME_HAND,
    ACTION_RECENTER,
    ACTION_SAFE_ABORT,
    ACTION_SEND_THROW,
    BB_STATE_BOOT,
    BB_STATE_ERROR,
    BB_STATE_IDLE,
    BB_STATE_RELOADING,
    BB_STATE_THROWING,
    BB_STATE_TRACKING,
    PHASE_AIMING,
    PHASE_BALL_IN_FLIGHT,
    PHASE_CHECKING,
    PHASE_PREPARING,
    PHASE_THROW_PENDING,
    RELOAD_CONTROL_MODE,
    ReloadObservations,
    ReloadSequencer,
    compute_catch_point_mm,
)

# The two settle instants these tests step to, DERIVED rather than typed. The
# confirm window became `ball_possession.ARRIVAL_BAND_MAX_S` on 2026-08-21
# (census D7) and moved 0.70 -> 0.80; six tests here went red on a hard-coded
# 3.8 / 4.5 for a reason unrelated to what any of them assert.
#: `_fresh()` has throw_delay 3.0 and no announced landing, so the FSM falls back
#: to release + BB's nominal ToF -> landing 3.05.
FALLBACK_SETTLE_T = 3.05 + CATCH_CONFIRM_WINDOW_S
#: With an ANNOUNCED landing of 3.7 (the realistic-flight fixture).
SETTLE_T = 3.7 + CATCH_CONFIRM_WINDOW_S

CATCH_PT = (0.0, 0.0, 809.08)


def _obs(now, **kw):
    base = dict(
        control_mode=RELOAD_CONTROL_MODE, bb_connected=True, bb_state=BB_STATE_IDLE,
        ball_in_hand=True, mocap_fresh=True, streaming=True,
        # Since 2026-07-29 the reload also requires the platform to still be
        # CENTERED (a caught toss now stays at its catch pose, so "parked off
        # centre" is a routine state and the reload's fixed-centre catch cannot
        # reach from there). The healthy-graph default is True; the gate itself
        # is exercised by test_not_centered_* below.
        platform_centered=True,
        ball_caught=False,
        catch_error_mm=float('nan'))
    base.update(kw)
    return ReloadObservations(now=now, **base)


def _fresh(**kw):
    seq = ReloadSequencer(catch_point_mm=CATCH_PT, throw_delay_s=3.0, **kw)
    seq.start(0.0)
    return seq


def _to_throw_sent(seq):
    """The armed-before-throw walk: PRIME (CHECKING) → PREPARE ok (PREPARING) →
    AIMING sends the throw. Times: prime 0.0, prepare 0.02, send 0.05."""
    d = seq.step(0.0, _obs(0.0))
    assert d.action == ACTION_PRIME_HAND and d.phase == PHASE_CHECKING
    seq.note_prime_result(True)
    d = seq.step(0.02, _obs(0.02))
    assert d.phase == PHASE_PREPARING and d.action == ACTION_PREPARE_CATCH
    assert seq.prepared is True                 # armed BEFORE any throw exists
    seq.note_prepare_result(True)
    d = seq.step(0.05, _obs(0.05))
    assert d.phase == PHASE_AIMING and d.action == ACTION_SEND_THROW
    return d


def _to_throw_pending(seq):
    """… then the throw is accepted → THROW_PENDING (no further action: all arming
    already ran). Announcement deadline = 0.1 + 3.0 + 0.5 = 3.6."""
    _to_throw_sent(seq)
    seq.note_throw_result(True, 'thrown')
    d = seq.step(0.1, _obs(0.1))
    assert d.phase == PHASE_THROW_PENDING
    assert d.action == ACTION_NONE
    return d


# ── Preconditions (loud rejects) ───────────────────────────────────────────────

@pytest.mark.parametrize('field,val,code', [
    ('control_mode', 'STANDBY', 'REJECTED_WRONG_MODE'),
    ('control_mode', 'SPACEMOUSE', 'REJECTED_WRONG_MODE'),
    ('bb_connected', False, 'REJECTED_BB_DISCONNECTED'),
    ('bb_state', BB_STATE_THROWING, 'REJECTED_BB_BUSY'),
    ('mocap_fresh', False, 'REJECTED_MOCAP_STALE'),
    ('streaming', False, 'REJECTED_NOT_STREAMING'),
    ('platform_centered', False, 'REJECTED_NOT_CENTERED'),
])
def test_precondition_rejects(field, val, code):
    seq = _fresh()
    d = seq.step(0.0, _obs(0.0, **{field: val}))
    assert d.done is True
    assert d.result.success is False
    assert d.result.outcome == code
    # A precondition reject happens BEFORE the prime — nothing armed, no cleanup.
    assert d.action == ACTION_NONE
    assert seq.prepared is False


def test_not_centered_is_refused_before_bb_is_asked_to_throw():
    """The stay-at-pose seam, closed BEFORE anything is committed.

    A caught toss now leaves the platform at its catch pose, so a reload can be
    commanded with the platform parked off centre. The reload catch is
    hard-fixed at the workspace centre and the reload never pre-positions, so
    arming from there would centre the reach envelope off (0, 0) and reject the
    incoming BB ball WORKSPACE **mid-flight** — ball airborne, nothing
    recoverable. This gate must therefore fire in CHECKING, before
    ACTION_PRIME_HAND and long before the throw: nothing moves and BB is never
    asked."""
    seq = _fresh()
    d = seq.step(0.0, _obs(0.0, platform_centered=False))
    assert d.done and d.result.outcome == 'REJECTED_NOT_CENTERED'
    assert d.action == ACTION_NONE
    assert seq.prepared is False


@pytest.mark.parametrize('also_broken,code', [
    # The two graph-staleness gates win: a dead trajectory link makes the
    # commanded pose UNKNOWABLE rather than off-centre, and a misleading code
    # would send the operator to `go_home` when the real fault is upstream.
    (dict(mocap_fresh=False), 'REJECTED_MOCAP_STALE'),
    (dict(streaming=False), 'REJECTED_NOT_STREAMING'),
    (dict(bb_connected=False), 'REJECTED_BB_DISCONNECTED'),
])
def test_not_centered_yields_to_the_staleness_gates(also_broken, code):
    """Gate order pinned: NOT_CENTERED sits after every observation whose
    failure would make it unknowable."""
    seq = _fresh()
    d = seq.step(0.0, _obs(0.0, platform_centered=False, **also_broken))
    assert d.done and d.result.outcome == code


def test_centered_default_is_fail_closed():
    """ReloadObservations.platform_centered defaults False — an FSM that was
    never told is not entitled to assume. Same doctrine as the toss's
    platform_levelled."""
    assert ReloadObservations(now=0.0).platform_centered is False


def test_active_reload_mode_is_trajectory():
    """RELOAD runs within the active streaming mode (TRAJECTORY), not the retired
    CATCH mode — TRAJECTORY must be accepted, so the first step starts arming
    (the hand prime), not a WRONG_MODE reject."""
    seq = _fresh()
    d = seq.step(0.0, _obs(0.0, control_mode='TRAJECTORY'))
    assert d.action == ACTION_PRIME_HAND and d.phase == PHASE_CHECKING


def test_cant_make_lead_rejected_early():
    seq = ReloadSequencer(catch_point_mm=CATCH_PT, throw_delay_s=1.0)  # < 2.5 floor
    seq.start(0.0)
    d = seq.step(0.0, _obs(0.0))
    assert d.done and d.result.outcome == 'REJECTED_CANT_MAKE_LEAD'
    assert d.action == ACTION_NONE


def test_zero_delay_defaults_to_3s():
    seq = ReloadSequencer(catch_point_mm=CATCH_PT, throw_delay_s=0.0)
    assert seq.throw_delay_s == pytest.approx(3.0)


# ── PRIME at command (the Q1 fix) ──────────────────────────────────────────────

def test_prime_emitted_immediately_when_preconditions_pass():
    """The hand prime is the FIRST action of the sequence — emitted the moment
    preconditions pass, never waiting for a tracked ball. Hardware 2026-07-23:
    the ~0.7 s bottom→top smooth-move vs the 0.878 s flight makes a
    ball-triggered prime a coin flip (lost by 0.06 s, won by 0.09 s)."""
    seq = _fresh()
    d = seq.step(0.0, _obs(0.0))
    assert d.action == ACTION_PRIME_HAND
    assert d.phase == PHASE_CHECKING and not d.done


def test_prime_precedes_reload_call_when_hand_empty():
    """Even an empty-BB-hand sequence primes FIRST: the prime runs during the
    (up to 10 s) reload wait instead of after it."""
    seq = _fresh()
    d = seq.step(0.0, _obs(0.0, ball_in_hand=False))
    assert d.action == ACTION_PRIME_HAND
    seq.note_prime_result(True)
    d = seq.step(0.05, _obs(0.05, ball_in_hand=False))
    assert d.action == ACTION_CALL_RELOAD and d.phase == PHASE_CHECKING


def test_prime_failure_aborts_before_any_throw():
    """A failed prime dispatch aborts the sequence with the throw never sent —
    the arming-before-throw ordering means there is nothing on the BB side to
    abort (BB's firmware countdown has no abort opcode)."""
    seq = _fresh()
    d = seq.step(0.0, _obs(0.0))
    assert d.action == ACTION_PRIME_HAND
    seq.note_prime_result(False)
    d = seq.step(0.05, _obs(0.05))
    assert d.done and d.result.outcome == 'ABORTED_PRIME_FAILED'
    # The prime was dispatched (hand may be mid-move) → SAFE_ABORT retracts it.
    assert d.action == ACTION_SAFE_ABORT
    assert seq._throw_sent is False


# ── PREPARE before the throw (the Q2 fix) ──────────────────────────────────────

def test_prepare_runs_before_throw_and_gates_it():
    """The catch latch is raised and CONFIRMED before bb/throw_at_target is sent:
    prepared is True while the throw is still unsent."""
    seq = _fresh()
    d = seq.step(0.0, _obs(0.0))
    seq.note_prime_result(True)
    d = seq.step(0.02, _obs(0.02))
    assert d.phase == PHASE_PREPARING and d.action == ACTION_PREPARE_CATCH
    assert seq.prepared is True and seq._throw_sent is False
    # No prepare result yet → the FSM WAITS (no throw).
    d = seq.step(0.03, _obs(0.03))
    assert d.phase == PHASE_PREPARING and d.action == ACTION_NONE
    assert seq._throw_sent is False
    seq.note_prepare_result(True)
    d = seq.step(0.05, _obs(0.05))
    assert d.phase == PHASE_AIMING and d.action == ACTION_SEND_THROW


def test_prepare_failure_aborts_before_any_throw():
    """A failed latch raise aborts with the throw never sent — the whole point of
    arming-before-throw: an arming failure never leaves an unabortable BB throw
    heading at an unarmed robot."""
    seq = _fresh()
    seq.step(0.0, _obs(0.0))
    seq.note_prime_result(True)
    seq.step(0.02, _obs(0.02))                    # PREPARING
    seq.note_prepare_result(False)
    d = seq.step(0.05, _obs(0.05))
    assert d.done and d.result.outcome == 'ABORTED_PREPARE_FAILED'
    assert d.action == ACTION_SAFE_ABORT
    assert seq._throw_sent is False


# ── Reload sub-sequence (empty hand) ───────────────────────────────────────────

def test_empty_hand_calls_reload_then_prepares_then_aims():
    seq = _fresh()
    seq.step(0.0, _obs(0.0, ball_in_hand=False))          # PRIME first
    seq.note_prime_result(True)
    # Empty hand → CALL_RELOAD, still CHECKING.
    d = seq.step(0.1, _obs(0.1, ball_in_hand=False))
    assert d.action == ACTION_CALL_RELOAD
    assert d.phase == PHASE_CHECKING and not d.done
    # BB is reloading — keep waiting.
    d = seq.step(1.0, _obs(1.0, ball_in_hand=False, bb_state=BB_STATE_RELOADING))
    assert d.action == ACTION_NONE and not d.done
    # Reload complete (IDLE + ball_in_hand) → PREPARE (latch), then AIMING.
    d = seq.step(2.0, _obs(2.0, ball_in_hand=True, bb_state=BB_STATE_IDLE))
    assert d.action == ACTION_PREPARE_CATCH and d.phase == PHASE_PREPARING
    seq.note_prepare_result(True)
    d = seq.step(2.1, _obs(2.1))
    assert d.action == ACTION_SEND_THROW and d.phase == PHASE_AIMING


def test_reload_timeout_rejects_no_ball_and_retracts():
    seq = _fresh()
    seq.step(0.0, _obs(0.0, ball_in_hand=False))          # PRIME
    seq.note_prime_result(True)
    seq.step(0.1, _obs(0.1, ball_in_hand=False))          # CALL_RELOAD
    d = seq.step(11.2, _obs(11.2, ball_in_hand=False, bb_state=BB_STATE_RELOADING))
    assert d.done and d.result.outcome == 'REJECTED_NO_BALL'
    # The hand was already primed at command → the reject must retract it.
    assert d.action == ACTION_SAFE_ABORT


# ── Throw + announcement + catch (happy path) ──────────────────────────────────

def test_happy_path_caught_emits_recenter():
    seq = _fresh()
    _to_throw_pending(seq)
    # Announcement arrives → BALL_IN_FLIGHT.
    seq.note_announcement(0.5)
    d = seq.step(0.5, _obs(0.5, bb_state=BB_STATE_IDLE))
    assert d.phase == PHASE_BALL_IN_FLIGHT
    # Ball caught (tracker) → CAUGHT, and the FSM asks the node to RE-CENTER.
    d = seq.step(3.1, _obs(3.1, ball_caught=True, catch_error_mm=12.0))
    assert d.done and d.result.success is True
    assert d.result.outcome == 'CAUGHT'
    assert d.result.catch_error_mm == pytest.approx(12.0)
    assert d.action == ACTION_RECENTER


def test_bb_reject_after_arming_safe_aborts():
    """A BB throw reject now lands AFTER arming (prime + latch precede the throw),
    so the reject must SAFE_ABORT — un-arm the robot. Nothing is in the air and no
    throw is pending on the BB side (it was rejected)."""
    seq = _fresh()
    _to_throw_sent(seq)
    seq.note_throw_result(False, 'pitch out of range')
    d = seq.step(0.1, _obs(0.1))
    assert d.done and d.result.success is False
    assert d.result.outcome == 'REJECTED_BB(pitch out of range)'
    assert seq.prepared is True
    assert d.action == ACTION_SAFE_ABORT


def test_no_announcement_aborts_with_safe_abort():
    seq = _fresh()
    _to_throw_pending(seq)
    # No announcement by (accept 0.1) + throw_delay + grace = 3.6.
    d = seq.step(3.6, _obs(3.6))
    assert d.done and d.result.outcome == 'ABORTED_NO_ANNOUNCEMENT'
    assert d.action == ACTION_SAFE_ABORT


def test_caught_then_no_confirm_is_missed_safe_abort():
    seq = _fresh()
    _to_throw_pending(seq)
    seq.note_announcement(0.5)
    seq.step(0.5, _obs(0.5))                              # BALL_IN_FLIGHT
    # Confirm window (release-fallback landing 3.05 + the window) passes, no CAUGHT.
    past = 3.05 + CATCH_CONFIRM_WINDOW_S
    d = seq.step(past, _obs(past, ball_caught=False, catch_error_mm=55.0))
    assert d.done and d.result.outcome == 'MISSED'
    assert d.result.catch_error_mm == pytest.approx(55.0)
    assert d.action == ACTION_SAFE_ABORT                 # armed → safe the robot


# ── Standing infeasibility resolves at settle, not mid-flight ──────────────────

def test_infeasible_does_not_terminate_mid_flight():
    """A standing catch-target rejection must NOT finish the sequence while the
    ball is in the air: the old finish-immediately behaviour ran SAFE_ABORT
    mid-flight — tearing down the hand's armed schedule and retracting the hand
    INTO the incoming ball. The platform holds, the hand keeps its schedule, and
    the outcome resolves at settle."""
    seq = _fresh()
    _to_throw_pending(seq)
    seq.note_announcement(0.5)
    seq.step(0.5, _obs(0.5))                              # BALL_IN_FLIGHT
    seq.note_catch_feasibility(False, 'WORKSPACE')
    d = seq.step(1.0, _obs(1.0))
    assert not d.done                                     # still flying — no teardown
    d = seq.step(FALLBACK_SETTLE_T, _obs(FALLBACK_SETTLE_T, ball_caught=False))
    assert d.done and d.result.outcome == 'MISSED_INFEASIBLE_WORKSPACE'
    assert d.action == ACTION_SAFE_ABORT


def test_infeasible_but_ball_lands_in_cup_is_caught():
    """The primed, parked cup can still catch a dead-centre throw the platform
    never reached for — observed twice on hardware (2026-07-23). An infeasibility
    that stood at catch time must not veto a tracker-confirmed CAUGHT."""
    seq = _fresh()
    _to_throw_pending(seq)
    seq.note_announcement(0.5)
    seq.step(0.5, _obs(0.5))                              # BALL_IN_FLIGHT
    seq.note_catch_feasibility(False, 'WORKSPACE')
    d = seq.step(3.1, _obs(3.1, ball_caught=True, catch_error_mm=20.0))
    assert d.done and d.result.success is True
    assert d.result.outcome == 'CAUGHT'
    assert d.action == ACTION_RECENTER


def test_catch_feasibility_reject_then_accept_clears():
    """A genuine reject followed by a later ACCEPT clears the standing
    infeasibility — only a rejection that still stands at catch time counts, so
    the settle outcome is plain MISSED, not MISSED_INFEASIBLE."""
    seq = _fresh()
    _to_throw_pending(seq)
    seq.note_announcement(0.5, landing_time_perf=3.7)
    seq.step(0.5, _obs(0.5))                              # BALL_IN_FLIGHT
    seq.note_catch_feasibility(False, 'WORKSPACE')       # a real reject …
    seq.note_catch_feasibility(True)                     # … superseded by a later accept
    d = seq.step(1.0, _obs(1.0, ball_caught=False))
    assert not d.done
    d = seq.step(SETTLE_T, _obs(SETTLE_T, ball_caught=False))   # landing 3.7 + window
    assert d.done and d.result.outcome == 'MISSED'


def test_accept_then_reject_stream_resolves_missed_not_infeasible():
    """THE 2026-07-23 third-sitting verdict bug: once ANY target is accepted (the
    pre-tilt), the platform is holding a valid catch pose — a later stream of
    WORKSPACE rejects (the corrupt split-track's drifting landing estimate walking
    out of the 80 mm envelope, 27–46 per flight) says nothing about reachability
    and must NOT flip the verdict to MISSED_INFEASIBLE. All 12 goals of that
    sitting read MISSED_INFEASIBLE_WORKSPACE while 8 balls were caught."""
    seq = _fresh()
    _to_throw_pending(seq)
    seq.note_announcement(0.5)
    seq.step(0.5, _obs(0.5))                              # BALL_IN_FLIGHT
    seq.note_catch_feasibility(True)                      # pre-tilt accepted
    for _ in range(30):                                   # corrupt-track reject stream
        seq.note_catch_feasibility(False, 'WORKSPACE')
    d = seq.step(FALLBACK_SETTLE_T, _obs(FALLBACK_SETTLE_T, ball_caught=False))
    assert d.done and d.result.outcome == 'MISSED'        # honest: reachable, not seated


def test_pretilt_accept_during_aiming_counts():
    """The pre-tilt target's acceptance lands while the FSM is still AIMING (the
    announcement — and therefore the target and its feedback — is published inside
    the throw service call). The feasibility gate keys off _throw_sent, not the
    phase; a phase gate would drop this accept deterministically on every real
    reload and re-open the false-INFEASIBLE hole."""
    seq = _fresh()
    _to_throw_sent(seq)                                   # AIMING, throw commanded
    seq.note_catch_feasibility(True)                      # pre-tilt accept, pre-announce
    seq.note_throw_result(True, 'thrown')
    seq.step(0.1, _obs(0.1))                              # THROW_PENDING
    seq.note_announcement(0.5)
    seq.step(0.5, _obs(0.5))                              # BALL_IN_FLIGHT
    seq.note_catch_feasibility(False, 'WORKSPACE')        # in-flight garbage reject
    d = seq.step(FALLBACK_SETTLE_T, _obs(FALLBACK_SETTLE_T, ball_caught=False))
    assert d.done and d.result.outcome == 'MISSED'


def test_feasibility_before_throw_sent_is_ignored():
    """Target feedback arriving before any throw was commanded (stray targets from
    a previous flight's still-alive tracks) must not seed this goal's verdict."""
    seq = _fresh()
    seq.note_catch_feasibility(False, 'WORKSPACE')        # pre-throw: ignored
    _to_throw_pending(seq)
    seq.note_announcement(0.5)
    seq.step(0.5, _obs(0.5))                              # BALL_IN_FLIGHT
    d = seq.step(FALLBACK_SETTLE_T, _obs(FALLBACK_SETTLE_T, ball_caught=False))
    assert d.done and d.result.outcome == 'MISSED'        # not MISSED_INFEASIBLE


# ── Announcement gating + ToF-aware settle deadline ────────────────────────────

def test_announcement_during_aiming_is_accepted():
    """The announcement is published synchronously inside the throw service call — i.e.
    while the FSM is still AIMING, BEFORE note_throw_result. It must still be honored
    (gated on the throw having been commanded, not on the phase)."""
    seq = _fresh()
    _to_throw_sent(seq)                                  # SEND_THROW issued; still AIMING
    assert seq.phase == PHASE_AIMING
    seq.note_announcement(0.06, landing_time_perf=3.7)   # arrives DURING AIMING
    seq.note_throw_result(True)                          # service returns afterwards
    d = seq.step(0.1, _obs(0.1))                         # AIMING → THROW_PENDING
    assert d.phase == PHASE_THROW_PENDING
    d = seq.step(0.2, _obs(0.2))                         # earlier announcement honored
    assert d.phase == PHASE_BALL_IN_FLIGHT


def test_settle_deadline_includes_time_of_flight():
    """The settle deadline is anchored on the announced landing (release + ToF), so a
    nominal catch confirmed ~0.3 s after landing reads CAUGHT — not MISSED because the
    deadline treated RELEASE as landing."""
    seq = _fresh()                                       # throw_delay 3.0
    _to_throw_pending(seq)
    # Realistic flight: release ≈ 3.0, ToF ≈ 0.7 → land at 3.7 (perf clock).
    seq.note_announcement(0.5, landing_time_perf=3.7)
    d = seq.step(0.5, _obs(0.5))                         # → BALL_IN_FLIGHT
    assert d.phase == PHASE_BALL_IN_FLIGHT
    # Settle deadline = landing + confirm window. DERIVED, not typed: the window
    # became ball_possession.ARRIVAL_BAND_MAX_S on 2026-08-21 (census D7).
    assert seq._settle_deadline == pytest.approx(3.7 + CATCH_CONFIRM_WINDOW_S)
    # At the OLD (release-anchored) deadline of 3.7 the ball has not been declared CAUGHT
    # yet — must NOT report MISSED.
    d = seq.step(3.7, _obs(3.7, ball_caught=False))
    assert not d.done
    # Tracker declares CAUGHT at landing + 0.3 = 4.0, before the ToF-aware deadline.
    d = seq.step(4.0, _obs(4.0, ball_caught=True, catch_error_mm=8.0))
    assert d.done and d.result.outcome == 'CAUGHT'
    assert d.result.catch_error_mm == pytest.approx(8.0)
    assert d.action == ACTION_RECENTER


# ── Aborts ─────────────────────────────────────────────────────────────────────

def test_mode_change_mid_sequence_aborts_safe_abort():
    seq = _fresh()
    _to_throw_pending(seq)                                # armed
    d = seq.step(0.2, _obs(0.2, control_mode='STANDBY'))
    assert d.done and d.result.outcome == 'ABORTED_MODE_CHANGED'
    assert d.action == ACTION_SAFE_ABORT


def test_bb_error_mid_sequence_aborts_safe_abort():
    seq = _fresh()
    _to_throw_pending(seq)                                # armed
    d = seq.step(0.2, _obs(0.2, bb_state=BB_STATE_ERROR))
    assert d.done and d.result.outcome == 'ABORTED_BB_ERROR'
    assert d.action == ACTION_SAFE_ABORT


def test_bb_error_at_checking_rejects_no_cleanup():
    seq = _fresh()
    d = seq.step(0.0, _obs(0.0, bb_state=BB_STATE_ERROR))
    assert d.done and d.result.outcome == 'REJECTED_BB_ERROR'
    # Reject at CHECKING, before any arming → nothing to safe.
    assert seq.prepared is False
    assert d.action == ACTION_NONE


def test_finished_is_terminal():
    seq = _fresh()
    d = seq.step(0.0, _obs(0.0, streaming=False))
    assert d.done
    first = d.result
    assert first is not None
    # Any subsequent step is a no-op terminal that REPLAYS the stored result (not None),
    # so a stray extra step never hands the consumer a None to unpack — and never
    # re-runs the terminal action.
    d2 = seq.step(1.0, _obs(1.0))
    assert d2.done and d2.result is first
    assert d2.action == ACTION_NONE


def test_terminal_action_fires_once_then_replays_none():
    """The terminal action (RECENTER / SAFE_ABORT) is emitted exactly on the step that
    finishes; the finished-replay path returns ACTION_NONE so the node never re-runs it."""
    seq = _fresh()
    _to_throw_pending(seq)
    seq.note_announcement(0.5)
    seq.step(0.5, _obs(0.5))                              # BALL_IN_FLIGHT
    d = seq.step(3.1, _obs(3.1, ball_caught=True))       # CAUGHT → RECENTER
    assert d.action == ACTION_RECENTER
    d2 = seq.step(3.2, _obs(3.2, ball_caught=True))      # replay
    assert d2.done and d2.action == ACTION_NONE


# ── BB heartbeat enum drift guard ──────────────────────────────────────────────

def test_bb_state_constants_match_protocol():
    """The FSM's BB state constants MUST mirror protocol_config.BallButlerStates —
    the heartbeat's `state` field is the firmware enum verbatim. The pre-2026-07-23
    values had RELOADING=2, colliding with the protocol's TRACKING=2 (a latent trap:
    any future gate on RELOADING would have matched a TRACKING heartbeat)."""
    from jugglebot.protocol_config import BallButlerStates
    assert BB_STATE_BOOT == BallButlerStates.BOOT
    assert BB_STATE_IDLE == BallButlerStates.IDLE
    assert BB_STATE_TRACKING == BallButlerStates.TRACKING
    assert BB_STATE_THROWING == BallButlerStates.THROWING
    assert BB_STATE_RELOADING == BallButlerStates.RELOADING
    assert BB_STATE_ERROR == BallButlerStates.ERROR


# ── Catch point helper ─────────────────────────────────────────────────────────

def test_compute_catch_point():
    pt = compute_catch_point_mm(574.3, 170.0)
    assert pt[0] == 0.0 and pt[1] == 0.0
    assert pt[2] == pytest.approx(744.3)
    pt2 = compute_catch_point_mm(574.3, 170.0, landing_z_offset_mm=10.0)
    assert pt2[2] == pytest.approx(754.3)
