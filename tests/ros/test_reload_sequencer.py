"""Unit tests for the pure-Python reload FSM (jugglebot.reload_sequencer).

No ROS: the FSM is driven with synthesized observations + discrete events, so every
precondition reject, the reload sub-sequence, the throw/announcement/catch path, the
PREPARE / RECENTER / SAFE_ABORT terminal actions, and every abort is exercised
deterministically.

The RELOAD action now OWNS the platform + hand for its duration
(reload-action-catch-latch plan): it runs within the active streaming mode
(``TRAJECTORY``, not the retired CATCH mode) and, on throw-accept, PREPAREs the catch
(prime hand + raise the catch-armed latch). Terminals emit RECENTER (caught) or, once
prepared, SAFE_ABORT (retract + recenter).
"""

from __future__ import annotations

import math

import pytest

from jugglebot.reload_sequencer import (
    ACTION_CALL_RELOAD,
    ACTION_NONE,
    ACTION_PREPARE_CATCH,
    ACTION_RECENTER,
    ACTION_SAFE_ABORT,
    ACTION_SEND_THROW,
    BB_STATE_ERROR,
    BB_STATE_IDLE,
    BB_STATE_RELOADING,
    BB_STATE_THROWING,
    PHASE_AIMING,
    PHASE_BALL_IN_FLIGHT,
    PHASE_CHECKING,
    PHASE_THROW_PENDING,
    RELOAD_CONTROL_MODE,
    ReloadObservations,
    ReloadSequencer,
    compute_catch_point_mm,
)

CATCH_PT = (0.0, 0.0, 809.08)


def _obs(now, **kw):
    base = dict(
        control_mode=RELOAD_CONTROL_MODE, bb_connected=True, bb_state=BB_STATE_IDLE,
        ball_in_hand=True, mocap_fresh=True, streaming=True, ball_caught=False,
        catch_error_mm=float('nan'))
    base.update(kw)
    return ReloadObservations(now=now, **base)


def _fresh(**kw):
    seq = ReloadSequencer(catch_point_mm=CATCH_PT, throw_delay_s=3.0, **kw)
    seq.start(0.0)
    return seq


def _to_throw_pending(seq):
    """Ball in hand → send throw → accept → PREPARE (AIMING → THROW_PENDING)."""
    d = seq.step(0.0, _obs(0.0))
    assert d.action == ACTION_SEND_THROW
    seq.note_throw_result(True, 'thrown')
    d = seq.step(0.1, _obs(0.1))
    assert d.phase == PHASE_THROW_PENDING
    assert d.action == ACTION_PREPARE_CATCH
    assert seq.prepared is True
    return d


# ── Preconditions (loud rejects) ───────────────────────────────────────────────

@pytest.mark.parametrize('field,val,code', [
    ('control_mode', 'STANDBY', 'REJECTED_WRONG_MODE'),
    ('control_mode', 'SPACEMOUSE', 'REJECTED_WRONG_MODE'),
    ('bb_connected', False, 'REJECTED_BB_DISCONNECTED'),
    ('bb_state', BB_STATE_THROWING, 'REJECTED_BB_BUSY'),
    ('mocap_fresh', False, 'REJECTED_MOCAP_STALE'),
    ('streaming', False, 'REJECTED_NOT_STREAMING'),
])
def test_precondition_rejects(field, val, code):
    seq = _fresh()
    d = seq.step(0.0, _obs(0.0, **{field: val}))
    assert d.done is True
    assert d.result.success is False
    assert d.result.outcome == code
    # A precondition reject happens BEFORE PREPARE — nothing was armed, so no cleanup.
    assert d.action == ACTION_NONE
    assert seq.prepared is False


def test_active_reload_mode_is_trajectory():
    """RELOAD runs within the active streaming mode (TRAJECTORY), not the retired
    CATCH mode — TRAJECTORY must be accepted, so the first step sends the throw."""
    seq = _fresh()
    d = seq.step(0.0, _obs(0.0, control_mode='TRAJECTORY'))
    assert d.action == ACTION_SEND_THROW and d.phase == PHASE_AIMING


def test_cant_make_lead_rejected_early():
    seq = ReloadSequencer(catch_point_mm=CATCH_PT, throw_delay_s=1.0)  # < 2.5 floor
    seq.start(0.0)
    d = seq.step(0.0, _obs(0.0))
    assert d.done and d.result.outcome == 'REJECTED_CANT_MAKE_LEAD'
    assert d.action == ACTION_NONE


def test_zero_delay_defaults_to_3s():
    seq = ReloadSequencer(catch_point_mm=CATCH_PT, throw_delay_s=0.0)
    assert seq.throw_delay_s == pytest.approx(3.0)


# ── Reload sub-sequence (empty hand) ───────────────────────────────────────────

def test_empty_hand_calls_reload_then_aims():
    seq = _fresh()
    # Empty hand → CALL_RELOAD, still CHECKING.
    d = seq.step(0.0, _obs(0.0, ball_in_hand=False))
    assert d.action == ACTION_CALL_RELOAD
    assert d.phase == PHASE_CHECKING and not d.done
    # BB is reloading — keep waiting.
    d = seq.step(1.0, _obs(1.0, ball_in_hand=False, bb_state=BB_STATE_RELOADING))
    assert d.action == ACTION_NONE and not d.done
    # Reload complete (IDLE + ball_in_hand) → AIMING sends the throw.
    d = seq.step(2.0, _obs(2.0, ball_in_hand=True, bb_state=BB_STATE_IDLE))
    assert d.action == ACTION_SEND_THROW
    assert d.phase == PHASE_AIMING


def test_reload_timeout_rejects_no_ball():
    seq = _fresh()
    seq.step(0.0, _obs(0.0, ball_in_hand=False))          # CALL_RELOAD
    d = seq.step(11.0, _obs(11.0, ball_in_hand=False, bb_state=BB_STATE_RELOADING))
    assert d.done and d.result.outcome == 'REJECTED_NO_BALL'
    # NO_BALL reject is at CHECKING, before PREPARE → no cleanup.
    assert d.action == ACTION_NONE


# ── PREPARE (prime + raise the latch) ──────────────────────────────────────────

def test_throw_accept_emits_prepare_and_sets_prepared():
    seq = _fresh()
    d = seq.step(0.0, _obs(0.0))
    assert d.action == ACTION_SEND_THROW and seq.prepared is False
    seq.note_throw_result(True, 'thrown')
    d = seq.step(0.1, _obs(0.1))
    assert d.phase == PHASE_THROW_PENDING
    assert d.action == ACTION_PREPARE_CATCH           # prime hand + raise latch
    assert seq.prepared is True


# ── Throw + announcement + catch (happy path) ──────────────────────────────────

def _advance_to_throw_sent(seq):
    """Ball already in hand → first step sends the throw."""
    d = seq.step(0.0, _obs(0.0))
    assert d.action == ACTION_SEND_THROW
    return d


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


def test_bb_rejects_throw_before_prepare_emits_nothing():
    seq = _fresh()
    _advance_to_throw_sent(seq)
    seq.note_throw_result(False, 'pitch out of range')
    d = seq.step(0.1, _obs(0.1))
    assert d.done and d.result.success is False
    assert d.result.outcome == 'REJECTED_BB(pitch out of range)'
    # The BB reject lands BEFORE PREPARE (the throw was never accepted) → nothing armed.
    assert seq.prepared is False
    assert d.action == ACTION_NONE


def test_no_announcement_aborts_with_safe_abort():
    seq = _fresh()
    _to_throw_pending(seq)                                # PREPARE ran (prepared)
    # No announcement by throw_delay + grace (3.0 + 0.5).
    d = seq.step(3.6, _obs(3.6))
    assert d.done and d.result.outcome == 'ABORTED_NO_ANNOUNCEMENT'
    # PREPARE already primed the hand + raised the latch → SAFE_ABORT retracts + recenters.
    assert d.action == ACTION_SAFE_ABORT


def test_caught_then_no_confirm_is_missed_safe_abort():
    seq = _fresh()
    _to_throw_pending(seq)
    seq.note_announcement(0.5)
    seq.step(0.5, _obs(0.5))                              # BALL_IN_FLIGHT
    # Confirm window (throw_time 0 + 3.0 + 0.7) passes with no CAUGHT → MISSED.
    d = seq.step(3.8, _obs(3.8, ball_caught=False, catch_error_mm=55.0))
    assert d.done and d.result.outcome == 'MISSED'
    assert d.result.catch_error_mm == pytest.approx(55.0)
    assert d.action == ACTION_SAFE_ABORT                 # prepared → safe the robot


def test_post_release_infeasible_catch_missed_with_code():
    seq = _fresh()
    _to_throw_pending(seq)
    seq.note_announcement(0.5)
    seq.step(0.5, _obs(0.5))                              # BALL_IN_FLIGHT
    seq.note_catch_feasibility(False, 'WORKSPACE')
    d = seq.step(1.0, _obs(1.0))
    assert d.done and d.result.outcome == 'MISSED_INFEASIBLE_WORKSPACE'
    assert d.action == ACTION_SAFE_ABORT


def test_catch_feasibility_reject_then_accept_clears():
    """A genuine reject followed by a later ACCEPT clears MISSED_INFEASIBLE — only a
    rejection that still stands at catch time counts (fix 5)."""
    seq = _fresh()
    _to_throw_pending(seq)
    seq.note_announcement(0.5, landing_time_perf=3.7)
    seq.step(0.5, _obs(0.5))                              # BALL_IN_FLIGHT
    seq.note_catch_feasibility(False, 'WORKSPACE')       # a real reject …
    seq.note_catch_feasibility(True)                     # … superseded by a later accept
    d = seq.step(1.0, _obs(1.0, ball_caught=False))
    assert not d.done                                    # cleared → no MISSED_INFEASIBLE


# ── Announcement gating + ToF-aware settle deadline (fixes 2, 3) ────────────────

def test_announcement_during_aiming_is_accepted():
    """The announcement is published synchronously inside the throw service call — i.e.
    while the FSM is still AIMING, BEFORE note_throw_result. It must still be honored
    (gated on the throw having been commanded, not on the phase) — fix 2."""
    seq = _fresh()
    _advance_to_throw_sent(seq)                          # SEND_THROW issued; still AIMING
    assert seq.phase == PHASE_AIMING
    seq.note_announcement(0.05, landing_time_perf=3.7)   # arrives DURING AIMING
    seq.note_throw_result(True)                          # service returns afterwards
    d = seq.step(0.1, _obs(0.1))                         # AIMING → THROW_PENDING (PREPARE)
    assert d.action == ACTION_PREPARE_CATCH
    d = seq.step(0.2, _obs(0.2))                         # earlier announcement honored
    assert d.phase == PHASE_BALL_IN_FLIGHT


def test_settle_deadline_includes_time_of_flight():
    """The settle deadline is anchored on the announced landing (release + ToF), so a
    nominal catch confirmed ~0.3 s after landing reads CAUGHT — not MISSED because the
    deadline treated RELEASE as landing (fix 3)."""
    seq = _fresh()                                       # throw_delay 3.0
    _advance_to_throw_sent(seq)                          # SEND_THROW at throw_time 0.0
    seq.note_throw_result(True)
    seq.step(0.1, _obs(0.1))                             # → THROW_PENDING (PREPARE)
    # Realistic flight: release ≈ 3.0, ToF ≈ 0.7 → land at 3.7 (perf clock).
    seq.note_announcement(0.5, landing_time_perf=3.7)
    d = seq.step(0.5, _obs(0.5))                         # → BALL_IN_FLIGHT
    assert d.phase == PHASE_BALL_IN_FLIGHT
    # Settle deadline = landing + confirm window = 3.7 + 0.7 = 4.4.
    assert seq._settle_deadline == pytest.approx(4.4)
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
    _to_throw_pending(seq)                                # prepared
    d = seq.step(0.2, _obs(0.2, control_mode='STANDBY'))
    assert d.done and d.result.outcome == 'ABORTED_MODE_CHANGED'
    assert d.action == ACTION_SAFE_ABORT


def test_bb_error_mid_sequence_aborts_safe_abort():
    seq = _fresh()
    _to_throw_pending(seq)                                # prepared
    d = seq.step(0.2, _obs(0.2, bb_state=BB_STATE_ERROR))
    assert d.done and d.result.outcome == 'ABORTED_BB_ERROR'
    assert d.action == ACTION_SAFE_ABORT


def test_bb_error_at_checking_rejects_no_cleanup():
    seq = _fresh()
    d = seq.step(0.0, _obs(0.0, bb_state=BB_STATE_ERROR))
    assert d.done and d.result.outcome == 'REJECTED_BB_ERROR'
    # Reject at CHECKING, before PREPARE → nothing to safe.
    assert seq.prepared is False
    assert d.action == ACTION_NONE


def test_finished_is_terminal():
    seq = _fresh()
    d = seq.step(0.0, _obs(0.0, streaming=False))
    assert d.done
    first = d.result
    assert first is not None
    # Any subsequent step is a no-op terminal that REPLAYS the stored result (not None),
    # so a stray extra step never hands the consumer a None to unpack (fix 12) — and
    # never re-runs the terminal action.
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


# ── Catch point helper ─────────────────────────────────────────────────────────

def test_compute_catch_point():
    pt = compute_catch_point_mm(574.3, 170.0)
    assert pt[0] == 0.0 and pt[1] == 0.0
    assert pt[2] == pytest.approx(744.3)
    pt2 = compute_catch_point_mm(574.3, 170.0, landing_z_offset_mm=10.0)
    assert pt2[2] == pytest.approx(754.3)
