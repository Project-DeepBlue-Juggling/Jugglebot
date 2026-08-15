"""Unit tests for the pure-Python toss FSM (jugglebot.toss_sequencer).

No ROS: the FSM is driven with synthesized observations + discrete events, so every
precondition reject, the POSITIONING arrival/verification path, the PREPARE →
announce → dispatch ordering, the release-evidence path, and every abort is
exercised deterministically.

ORDERING (the reload principle transposed): every Jugglebot-side arming step
happens BEFORE the throw is dispatched — POSITIONING completes and is
mocap-verified before the latch raise (the arm_catch edge captures the reach
envelope from the CURRENT commanded pose), the armed confirm precedes the
self-announcement by an explicit ≥1-tick gap (the announcement and catch/armed
travel on different topics; an announcement arriving unarmed drops its pre-tilt),
and the throw dispatch is the LAST commitment — emitted exactly once, ever
(an ambiguous ack may have armed the stroke; a re-dispatch would replace it or,
post-release, clobber the armed catch stroke on the last-writer-wins queue).
Deliberate deviations from reload_sequencer are pinned by tests whose docstrings
say why: the prepare-dispatch feasibility/announcement gate (reload gates on
_throw_sent — for a toss the announcement precedes the dispatch), the mocap
arrival verification (reload has no platform move to verify), and forced-NaN
catch_error on non-CAUGHT terminals.
"""

from __future__ import annotations

import math
import os

import pytest

from jugglebot.toss_sequencer import (
    ACTION_ANNOUNCE,
    ACTION_DISPATCH_THROW,
    ACTION_NONE,
    ACTION_POSITION_PLATFORM,
    ACTION_PREPARE_CATCH,
    ACTION_REACH_CATCH,
    ACTION_RECENTER,
    ACTION_STAY,
    ACTION_SAFE_ABORT,
    DEFAULT_TOSS_FLIGHT_TIME_S,
    PHASE_BALL_IN_FLIGHT,
    PHASE_CATCHING,
    PHASE_CHECKING,
    PHASE_POSITIONING,
    PHASE_PREPARING,
    PHASE_SETTLING,
    PHASE_THROWING,
    THROW_DISPATCH_AMBIGUOUS,
    THROW_DISPATCH_OK,
    THROW_DISPATCH_REJECTED,
    TIER_8A,
    TIER_8B,
    TOSS_CONTROL_MODE,
    TOSS_MAX_DISPLACEMENT_MM,
    HAND_THROW_RELEASE_OFFSET_MM,
    REACH_VEL_LIMIT_MMPS,
    REACH_ACC_LIMIT_MMPS2,
    REACH_JERK_LIMIT_MMPS3,
    TossObservations,
    TossSequencer,
    reach_displacement_limit_mm,
)

CATCH_POSE = (0.0, 0.0, 170.0)


def _obs(now, **kw):
    base = dict(
        control_mode=TOSS_CONTROL_MODE, streaming=True, mocap_fresh=True,
        platform_levelled=True,
        hand_fresh=True, hand_parked=True, ball_seated=True,
        ball_evidence='SEATED', track_active=False,
        platform_at_target=True, throw_stroke_seen=False,
        ball_track_confirmed=False, ball_caught=False,
        catch_error_mm=float('nan'), catch_event_dt_s=float('nan'),
        ball_time_at_land_perf=float('nan'))
    base.update(kw)
    return TossObservations(now=now, **base)


def _fresh(**kw):
    params = dict(catch_pose_stow_mm=CATCH_POSE, flight_time_s=0.8,
                  throw_delay_s=5.0)
    params.update(kw)
    seq = TossSequencer(**params)
    seq.start(0.0)
    return seq


def _to_positioning(seq):
    """Preconditions pass → the profiled move to the catch pose (one-shot)."""
    d = seq.step(0.0, _obs(0.0))
    assert d.phase == PHASE_POSITIONING and d.action == ACTION_POSITION_PLATFORM
    return d


def _to_prepared(seq):
    """… move accepted (0.05, 0.5 s plan → arrival 0.75) → verified arrival at
    0.8 → PREPARE dispatched. The latch raise strictly follows arrival."""
    _to_positioning(seq)
    seq.note_position_result(0.05, True, 0.5)
    d = seq.step(0.8, _obs(0.8))
    assert d.phase == PHASE_PREPARING and d.action == ACTION_PREPARE_CATCH
    assert seq.prepared is True                 # armed BEFORE any throw exists
    return d


def _to_announced(seq):
    """… armed confirmed → the NEXT tick announces (the ≥1-tick gap), and the
    node confirms the publish."""
    _to_prepared(seq)
    seq.note_prepare_result(True)
    d = seq.step(0.9, _obs(0.9))
    assert d.phase == PHASE_PREPARING and d.action == ACTION_ANNOUNCE
    seq.note_announcement()
    return d


def _to_throw_dispatched(seq):
    """… announcement out → the throw dispatch, the sequence's LAST commitment.
    t_release = 5.0 (delay 5.0 from start(0.0)); release deadline 5.5."""
    _to_announced(seq)
    d = seq.step(1.0, _obs(1.0))
    assert d.phase == PHASE_THROWING and d.action == ACTION_DISPATCH_THROW
    return d


def _to_in_flight(seq):
    """… ack ok + stroke evidence → BALL_IN_FLIGHT. Scheduled landing 5.8;
    settle deadline 5.0 + 0.8 + 0.7 = 6.5."""
    _to_throw_dispatched(seq)
    seq.note_throw_dispatch(THROW_DISPATCH_OK)
    d = seq.step(1.1, _obs(1.1, throw_stroke_seen=True))
    assert d.phase == PHASE_BALL_IN_FLIGHT
    return d


# ── Preconditions (loud rejects) ───────────────────────────────────────────────

@pytest.mark.parametrize('field,val,code', [
    ('control_mode', 'STANDBY', 'REJECTED_WRONG_MODE'),
    ('control_mode', 'SPACEMOUSE', 'REJECTED_WRONG_MODE'),
    ('mocap_fresh', False, 'REJECTED_MOCAP_STALE'),
    ('streaming', False, 'REJECTED_NOT_STREAMING'),
    ('platform_levelled', False, 'REJECTED_NOT_LEVELLED'),
    ('hand_fresh', False, 'REJECTED_HAND_STALE'),
    ('hand_parked', False, 'REJECTED_HAND_NOT_PARKED'),
    ('track_active', True, 'REJECTED_TRACK_ACTIVE'),
])
def test_precondition_rejects(field, val, code):
    """The toss adds four preconditions reload never needed: HAND_STALE (a dead
    hand link blinds release verification), HAND_NOT_PARKED (a kind-0 throw
    stroke commands ABSOLUTE positions from 0 rev — dispatching off the bottom
    park band is a physical hazard), TRACK_ACTIVE (a live
    destination='jugglebot' phantom track would correlate against OUR
    announcement), and NOT_LEVELLED (an un-levelled launch is 0.78° off gravity
    ⇒ 43 mm of drift against a ~35 mm cup: the catch is geometrically
    impossible, so refuse before the ball is airborne). The trace-only
    ball-evidence waiver folds into ball_seated at the node;
    hand_fresh/hand_parked stay hard even under it."""
    seq = _fresh()
    d = seq.step(0.0, _obs(0.0, **{field: val}))
    assert d.done is True
    assert d.result.success is False
    assert d.result.outcome == code
    # A precondition reject happens BEFORE any move or arming — no cleanup.
    assert d.action == ACTION_NONE
    assert seq.prepared is False


@pytest.mark.parametrize('evidence,code', [
    ('EMPTY', 'REJECTED_NO_BALL'),
    ('UNKNOWN', 'REJECTED_BALL_UNKNOWN'),
])
def test_ball_evidence_refusals_name_the_sensor_state(evidence, code):
    """The ball-evidence gate mints TWO codes because they send the operator to
    two different subsystems (C-POSSESS-1 § 3.3).

    `NO_BALL` means the sensor looked and the cup is empty — load a ball.
    `BALL_UNKNOWN` means the sensor could not answer at all (boot before the
    first TxSdo reply, a stale reply, an un-anchored bridge clock, a dead
    poller). Collapsing the second into the first would send the operator hunting
    for a ball while the actual fault is the sensor, and — worse — a future
    reader would have no way to tell that UNKNOWN *refuses*, which is the
    fail-closed choice this project made against BallButler's fail-open boot
    default. Both are pre-motion: nothing has moved and nothing is armed."""
    seq = _fresh()
    d = seq.step(0.0, _obs(0.0, ball_seated=False, ball_evidence=evidence))
    assert d.done and d.result.success is False
    assert d.result.outcome == code
    assert d.action == ACTION_NONE and seq.prepared is False


@pytest.mark.parametrize('also_broken,code', [
    # The two graph-staleness gates win — a stale graph makes the levelling flag
    # UNKNOWABLE, and a misleading reject sends the operator to re-run `level`
    # when the real fault is upstream of the observation entirely.
    (dict(mocap_fresh=False), 'REJECTED_MOCAP_STALE'),
    (dict(streaming=False), 'REJECTED_NOT_STREAMING'),
    # …and NOT_LEVELLED wins over the whole hand-evidence chain and the ball
    # gates below it: those describe how the throw is dispatched, this one says
    # the throw cannot be caught wherever it is dispatched from.
    (dict(hand_fresh=False), 'REJECTED_NOT_LEVELLED'),
    (dict(hand_parked=False), 'REJECTED_NOT_LEVELLED'),
    (dict(ball_seated=False), 'REJECTED_NOT_LEVELLED'),
    (dict(track_active=True), 'REJECTED_NOT_LEVELLED'),
])
def test_not_levelled_sits_between_the_graph_gates_and_the_hand_chain(
        also_broken, code):
    """Gate order pinned. NOT_LEVELLED is a GEOMETRIC verdict — un-levelled the
    launch is 0.78° off gravity, i.e. v·sin(θ)·T = 43 mm of drift over a 0.8 s
    flight at 3.93 m/s against a ~35 mm cup radius — so it outranks every gate
    about *how* the hand throws. It yields only to the two observations whose
    failure would make it unknowable rather than false."""
    seq = _fresh()
    d = seq.step(0.0, _obs(0.0, platform_levelled=False, **also_broken))
    assert d.done and d.result.outcome == code
    assert d.action == ACTION_NONE and seq.prepared is False


def test_levelled_machine_proceeds_to_positioning():
    """The negative half of the gate: platform_levelled True is the ONLY thing
    the FSM asks about the frame — it never inspects the correction's magnitude,
    because a genuinely level machine measures a zero offset and must not be
    refused. (The node builds this flag; test_toss_coordinator pins that half.)"""
    seq = _fresh()
    d = seq.step(0.0, _obs(0.0, platform_levelled=True))
    assert d.phase == PHASE_POSITIONING and d.action == ACTION_POSITION_PLATFORM


def test_toss_mode_is_trajectory():
    """TOSS runs within the active streaming mode (TRAJECTORY) — same doctrine
    as RELOAD: TRAJECTORY must be accepted, so the first step starts the
    positioning move, not a WRONG_MODE reject."""
    seq = _fresh()
    d = seq.step(0.0, _obs(0.0, control_mode='TRAJECTORY'))
    assert d.phase == PHASE_POSITIONING and d.action == ACTION_POSITION_PLATFORM


def test_tier_8b_accepted_when_config_selects_it():
    """Phase 4: the tier gate now ADMITS '8b' (the config JB_OP_TOSS_TIER key
    selects it, the goal cannot). A serviceable co-located 8b goal (throw site
    == B ⇒ displacement 0) starts the positioning move, not REJECTED_TIER."""
    seq = TossSequencer(catch_pose_stow_mm=CATCH_POSE, flight_time_s=0.8,
                        throw_delay_s=5.0, tier=TIER_8B,
                        throw_site_xy_mm=(0.0, 0.0), throw_site_known=True)
    seq.start(0.0)
    d = seq.step(0.0, _obs(0.0))
    assert d.phase == PHASE_POSITIONING and d.action == ACTION_POSITION_PLATFORM


def test_tier_gate_rejects_unknown_tier():
    """Gate order pinned: the tier gate is the FIRST check of the sequence —
    anything outside {8a, 8b} reads REJECTED_TIER even when other preconditions
    also fail. No cleanup — nothing ran."""
    seq = TossSequencer(catch_pose_stow_mm=CATCH_POSE, flight_time_s=0.8,
                        throw_delay_s=5.0, tier='9z')
    seq.start(0.0)
    d = seq.step(0.0, _obs(0.0, control_mode='STANDBY'))
    assert d.done and d.result.outcome == 'REJECTED_TIER'
    assert d.action == ACTION_NONE
    assert seq.prepared is False


def test_throw_delay_floor_rejected():
    """delay < 3.5 s can never fit the positioning + prepare budget plus the
    1.0 s event-delay floor; the runtime ABORTED_CANT_MAKE_RELEASE guard is the
    real enforcement, this is the loud+early copy."""
    seq = TossSequencer(catch_pose_stow_mm=CATCH_POSE, flight_time_s=0.8,
                        throw_delay_s=2.0)
    seq.start(0.0)
    d = seq.step(0.0, _obs(0.0))
    assert d.done and d.result.outcome == 'REJECTED_CANT_MAKE_LEAD'
    assert d.action == ACTION_NONE


def test_zero_delay_defaults_to_5s():
    seq = TossSequencer(catch_pose_stow_mm=CATCH_POSE, throw_delay_s=0.0)
    assert seq.throw_delay_s == pytest.approx(5.0)


def test_zero_flight_time_defaults():
    seq = TossSequencer(catch_pose_stow_mm=CATCH_POSE, flight_time_s=0.0)
    assert seq.flight_time_s == pytest.approx(0.8)


def test_negative_params_never_coerce_to_defaults():
    """Default-substitution fires ONLY on exactly 0.0 (the goal's "unset"
    sentinel): a NEGATIVE flight time / throw delay is a sign typo and must be
    preserved so CHECKING rejects it loudly — silently coercing to the default
    would throw a physically different toss. (The node additionally rejects
    negatives pre-FSM as REJECTED_BAD_GOAL; this pins the FSM's own never-
    coerce belt.)"""
    seq = TossSequencer(catch_pose_stow_mm=CATCH_POSE, flight_time_s=-0.8,
                        throw_delay_s=5.0)
    assert seq.flight_time_s == pytest.approx(-0.8)     # preserved, not defaulted
    seq.start(0.0)
    d = seq.step(0.0, _obs(0.0))
    assert d.done and d.result.outcome == 'REJECTED_FLIGHT_TIME'

    seq2 = TossSequencer(catch_pose_stow_mm=CATCH_POSE, flight_time_s=0.8,
                         throw_delay_s=-5.0)
    assert seq2.throw_delay_s == pytest.approx(-5.0)    # preserved, not defaulted
    seq2.start(0.0)
    d = seq2.step(0.0, _obs(0.0))
    assert d.done and d.result.outcome == 'REJECTED_CANT_MAKE_LEAD'


@pytest.mark.parametrize('flight_t', [0.4, 1.5])
def test_flight_time_bounds_rejected(flight_t):
    seq = TossSequencer(catch_pose_stow_mm=CATCH_POSE, flight_time_s=flight_t,
                        throw_delay_s=5.0)
    seq.start(0.0)
    d = seq.step(0.0, _obs(0.0))
    assert d.done and d.result.outcome == 'REJECTED_FLIGHT_TIME'


def test_event_vel_bound_rejected():
    """The bridge raises outside [0.3, 7.0] m/s at dispatch time — this is the
    loud+early copy, so an infeasible throw never gets as far as a platform
    move."""
    seq = TossSequencer(catch_pose_stow_mm=CATCH_POSE, flight_time_s=0.8,
                        throw_delay_s=5.0, event_vel_mps=7.5)
    seq.start(0.0)
    d = seq.step(0.0, _obs(0.0))
    assert d.done and d.result.outcome == 'REJECTED_EVENT_VEL'


def test_default_event_vel_full_geometry_pin():
    """event_vel_mps = 0 resolves via the full vertical ballistic inverse
    (Δz/T + g·T/2 with Δz = 64.78 − 58.044 = 6.736 mm), matching
    motion/toss_release: T = 0.8 s ⇒ 3930.82 mm/s — NOT the idealised
    g·T/2 = 3922.4."""
    seq = _fresh()
    assert seq.event_vel_mps == pytest.approx(3.93082, abs=1e-4)


@pytest.mark.parametrize('pose', [
    (200.0, 0.0, 170.0),      # |x| beyond the ±150 planning envelope
    (0.0, -200.0, 170.0),     # |y| beyond
    (0.0, 0.0, 225.0),        # |z − 170| beyond the ±50 band
])
def test_workspace_precheck_rejected(pose):
    """Planning-envelope pre-check only — go_to_pose's feasibility gate remains
    the truth (REJECTED_POSITION(<code>) covers what this misses)."""
    seq = TossSequencer(catch_pose_stow_mm=pose, flight_time_s=0.8,
                        throw_delay_s=5.0)
    seq.start(0.0)
    d = seq.step(0.0, _obs(0.0))
    assert d.done and d.result.outcome == 'REJECTED_WORKSPACE'


# ── POSITIONING (deviation: reload has no platform move to make or verify) ─────

def test_position_emitted_once_when_preconditions_pass():
    seq = _fresh()
    d = _to_positioning(seq)
    assert not d.done
    # One-shot: the move is dispatched exactly once; waiting ticks are quiet.
    d = seq.step(0.1, _obs(0.1))
    assert d.phase == PHASE_POSITIONING and d.action == ACTION_NONE


def test_position_reject_terminates_no_cleanup():
    """A go_to_pose reject means the platform never moved and nothing was armed
    — a loud terminal carrying the reject-ladder code, with no cleanup."""
    seq = _fresh()
    _to_positioning(seq)
    seq.note_position_result(0.05, False, 0.0, 'WORKSPACE')
    d = seq.step(0.1, _obs(0.1))
    assert d.done and d.result.outcome == 'REJECTED_POSITION(WORKSPACE)'
    assert d.action == ACTION_NONE
    assert seq.prepared is False


def test_position_no_response_times_out():
    seq = _fresh()
    _to_positioning(seq)
    d = seq.step(5.9, _obs(5.9))
    assert not d.done
    d = seq.step(6.0, _obs(6.0))
    assert d.done and d.result.outcome == 'ABORTED_POSITION_TIMEOUT'
    # No response ⇒ nothing accepted, nothing moved — no cleanup.
    assert d.action == ACTION_NONE


def test_positioning_waits_planned_duration():
    """THE envelope-center invariant: PREPARE is emitted strictly AFTER the
    timed arrival (note-time + planned_duration + 0.2 pad). The arm_catch raise
    captures the reach-envelope center from the CURRENT commanded pose and
    C2-stops any in-flight move — arming mid-move plants the envelope wherever
    the platform happened to stop."""
    seq = _fresh()
    _to_positioning(seq)
    seq.note_position_result(0.1, True, 1.2)          # arrival = 0.1+1.2+0.2 = 1.5
    d = seq.step(1.2, _obs(1.2))
    assert d.phase == PHASE_POSITIONING and d.action == ACTION_NONE
    d = seq.step(1.45, _obs(1.45))
    assert d.phase == PHASE_POSITIONING and d.action == ACTION_NONE
    d = seq.step(1.5, _obs(1.5))
    assert d.phase == PHASE_PREPARING and d.action == ACTION_PREPARE_CATCH


def test_position_arrival_unverified_aborts():
    """The mocap arrival cross-check (a toss addition — reload never moves the
    platform pre-throw): an ACCEPTED move whose arrival mocap never corroborates
    is the silent no-op class — disarmed wire, guard latch — where go_to_pose
    reports a plan but the platform stays put. Arming there would capture the
    catch envelope at the wrong pose, so the sequence aborts at the positioning
    deadline instead of preparing."""
    seq = _fresh()
    _to_positioning(seq)
    seq.note_position_result(0.1, True, 1.2)          # arrival 1.5; deadline 6.0
    d = seq.step(1.6, _obs(1.6, platform_at_target=False))
    assert not d.done                                 # verification window open
    d = seq.step(5.9, _obs(5.9, platform_at_target=False))
    assert not d.done
    d = seq.step(6.0, _obs(6.0, platform_at_target=False))
    assert d.done and d.result.outcome == 'ABORTED_POSITION_FAILED'
    # The move was accepted (platform may have partially moved) → go_home leg.
    assert d.action == ACTION_SAFE_ABORT


def test_wire_disarmed_rejects():
    """The node maps a '[wire DISARMED' go_to_pose response message to the
    WIRE_DISARMED code — the plan-installed-but-not-actuating case."""
    seq = _fresh()
    _to_positioning(seq)
    seq.note_position_result(0.05, False, 0.0, 'WIRE_DISARMED')
    d = seq.step(0.1, _obs(0.1))
    assert d.done and d.result.outcome == 'REJECTED_POSITION(WIRE_DISARMED)'


# ── PREPARING: armed confirm → gap → announce → dispatch ───────────────────────

def test_prepare_failure_safe_aborts():
    """A failed latch raise aborts with nothing announced and no throw armed —
    the arming-before-throw ordering. The platform has moved (positioned), so
    the abort routes through SAFE_ABORT."""
    seq = _fresh()
    _to_prepared(seq)
    seq.note_prepare_result(False)
    d = seq.step(0.9, _obs(0.9))
    assert d.done and d.result.outcome == 'ABORTED_PREPARE_FAILED'
    assert d.action == ACTION_SAFE_ABORT
    assert seq._throw_dispatched is False


def test_prepare_ok_announces_after_gap_then_dispatches():
    """DELIBERATE DEVIATION from reload (which never announces — BB does): the
    self-announcement is emitted on a LATER tick than the PREPARE bundle, never
    inside it. catch/armed and throw_announcements are different topics with no
    cross-topic ordering guarantee, and catch_coordinator drops announcement
    pre-tilts that arrive unarmed — the ≥1-tick gap lets the armed edge land
    first. Only after the node confirms the publish does the throw dispatch —
    the sequence's LAST commitment — go out, each action exactly once."""
    seq = _fresh()
    _to_prepared(seq)
    seq.note_prepare_result(True)
    d = seq.step(0.9, _obs(0.9))
    assert d.phase == PHASE_PREPARING and d.action == ACTION_ANNOUNCE
    # Announce is one-shot; the FSM waits for the node's publish confirm.
    d = seq.step(0.95, _obs(0.95))
    assert d.phase == PHASE_PREPARING and d.action == ACTION_NONE
    seq.note_announcement()
    d = seq.step(1.0, _obs(1.0))
    assert d.phase == PHASE_THROWING and d.action == ACTION_DISPATCH_THROW
    d = seq.step(1.05, _obs(1.05))
    assert d.phase == PHASE_THROWING and d.action == ACTION_NONE


def test_cant_make_release_aborts_before_announcing():
    """Positioning ate the delay budget: t_release − now < the 1.0 s event-delay
    floor at prepare-confirm time. The abort fires BEFORE the announcement goes
    out — an announced-then-aborted toss would leave a phantom tracker
    expectation that REJECTED_TRACK_ACTIVE then refuses on until it expires.
    Ball still seated ⇒ safe to stand down."""
    seq = _fresh(throw_delay_s=3.5)                   # t_release = 3.5
    _to_positioning(seq)
    seq.note_position_result(0.05, True, 2.8)         # arrival = 3.05
    d = seq.step(3.05, _obs(3.05))
    assert d.phase == PHASE_PREPARING and d.action == ACTION_PREPARE_CATCH
    seq.note_prepare_result(True)
    d = seq.step(3.1, _obs(3.1))                      # 3.5 − 3.1 = 0.4 < 1.0
    assert d.done and d.result.outcome == 'ABORTED_CANT_MAKE_RELEASE'
    assert d.action == ACTION_SAFE_ABORT
    assert seq._announce_dispatched is False          # no phantom announcement


def test_announce_lead_short_warns_but_proceeds():
    """announce→landing lead < 2.5 s degrades the pre-tilt toward
    arrive-at-contact — but for a LEVEL Tier-8a toss the pre-tilt target equals
    the already-held pose, so the degeneracy is motion-free: WARN-only (the
    node logs the flag), the sequence proceeds. The Phase-1 "8b hardens this to
    an abort" promise is SUPERSEDED (see test_announce_lead_short_warns_for_8b_too
    and the TOSS_MIN_ANNOUNCE_LEAD_S supersession comment)."""
    seq = _fresh()                                    # t_release 5.0, landing 5.8
    _to_positioning(seq)
    seq.note_position_result(0.05, True, 3.55)        # arrival = 3.8
    d = seq.step(3.8, _obs(3.8))
    assert d.phase == PHASE_PREPARING and d.action == ACTION_PREPARE_CATCH
    seq.note_prepare_result(True)
    d = seq.step(3.85, _obs(3.85))                    # lead 5.8−3.85 = 1.95 < 2.5
    assert d.action == ACTION_ANNOUNCE
    assert seq.announce_lead_short is True
    seq.note_announcement()
    d = seq.step(3.9, _obs(3.9))                      # event_delay 1.1 ≥ 1.0: fine
    assert d.phase == PHASE_THROWING and d.action == ACTION_DISPATCH_THROW


def test_hand_unparked_at_throwing_entry_aborts():
    """The hand-parked RE-VERIFY at THROWING entry (before the dispatch
    commitment): CHECKING's park-band gate is seconds old by the time the
    throw would go out (positioning + prepare), and a kind-0 stroke commands
    ABSOLUTE positions from 0 rev — an off-band hand at dispatch is a physical
    hazard. Prepared ⇒ SAFE_ABORT cleans up; the throw is never dispatched."""
    seq = _fresh()
    _to_announced(seq)
    d = seq.step(1.0, _obs(1.0, hand_parked=False))
    assert d.done and d.result.outcome == 'ABORTED_HAND_NOT_PARKED'
    assert d.action == ACTION_SAFE_ABORT
    assert seq._throw_dispatched is False


# ── THROWING: single-shot tri-state dispatch + release evidence ────────────────

def test_dispatch_rejected_aborts():
    """A definitive reject (service unavailable / bridge-validation raise) means
    no CAN frame exists — nothing is armed, the ball is seated. SAFE_ABORT's
    retract deliberately also clears any half-state on the queue."""
    seq = _fresh()
    _to_throw_dispatched(seq)
    seq.note_throw_dispatch(THROW_DISPATCH_REJECTED, 'Invalid event velocity')
    d = seq.step(1.1, _obs(1.1))
    assert d.done and d.result.outcome == 'ABORTED_THROW_DISPATCH_FAILED'
    assert d.action == ACTION_SAFE_ABORT


def test_dispatch_ambiguous_waits():
    """An ambiguous ack (ERR_TIMEOUT class — the frame may have transmitted) is
    NOT a failure and NOT a license to retry: the FSM waits for release
    evidence, never emitting a second dispatch."""
    seq = _fresh()
    _to_throw_dispatched(seq)
    seq.note_throw_dispatch(THROW_DISPATCH_AMBIGUOUS, 'ERR_TIMEOUT')
    d = seq.step(1.1, _obs(1.1))
    assert not d.done and d.action == ACTION_NONE
    d = seq.step(2.0, _obs(2.0))
    assert not d.done and d.action == ACTION_NONE


def test_never_redispatches_throw():
    """THE LOCKED no-re-dispatch invariant, across the whole goal: an ambiguous
    ack may have armed the stroke, so a re-dispatch would re-pack a new
    wall_time (if the first frame was lost) or REPLACE a live stroke (if the
    ack lied) — and post-release it would clobber the armed CATCH stroke on
    the Teensy's last-writer-wins queue. ACTION_DISPATCH_THROW appears exactly
    once across every decision of an ambiguous-ack goal."""
    seq = _fresh()
    decisions = [seq.step(0.0, _obs(0.0))]
    seq.note_position_result(0.05, True, 0.5)
    decisions.append(seq.step(0.8, _obs(0.8)))
    seq.note_prepare_result(True)
    decisions.append(seq.step(0.9, _obs(0.9)))
    seq.note_announcement()
    decisions.append(seq.step(1.0, _obs(1.0)))
    seq.note_throw_dispatch(THROW_DISPATCH_AMBIGUOUS, 'ERR_TIMEOUT')
    decisions.append(seq.step(1.2, _obs(1.2)))
    decisions.append(seq.step(2.0, _obs(2.0, throw_stroke_seen=True)))
    decisions.append(seq.step(5.9, _obs(5.9, ball_caught=True,
                                        catch_error_mm=10.0)))
    actions = [d.action for d in decisions]
    assert actions.count(ACTION_DISPATCH_THROW) == 1
    assert decisions[-1].done and decisions[-1].result.outcome == 'CAUGHT'


def test_stroke_evidence_advances():
    """Release evidence channel 1: the hand-telemetry ascending-stroke
    signature. Settle deadline = t_release + flight + confirm window (the FSM
    is the announcer — its own schedule IS the landing prediction)."""
    seq = _fresh()
    _to_throw_dispatched(seq)
    seq.note_throw_dispatch(THROW_DISPATCH_OK)
    d = seq.step(1.1, _obs(1.1, throw_stroke_seen=True))
    assert d.phase == PHASE_BALL_IN_FLIGHT
    assert seq._settle_deadline == pytest.approx(6.5)  # 5.0 + 0.8 + 0.7


def test_tracker_evidence_advances():
    """Release evidence channel 2: tracker CONFIRMED for the announced ball —
    the disjunction means either independent channel advances the sequence
    (status IN_FLIGHT alone is time-based and proves nothing)."""
    seq = _fresh()
    _to_throw_dispatched(seq)
    seq.note_throw_dispatch(THROW_DISPATCH_OK)
    d = seq.step(1.1, _obs(1.1, ball_track_confirmed=True))
    assert d.phase == PHASE_BALL_IN_FLIGHT


def test_no_release_evidence_aborts():
    """No evidence on either channel by t_release + 0.5 grace: the stroke is
    presumed never to have fired, the ball presumed seated (possession
    survives the abort, so the next toss is not falsely REJECTED_NO_BALL)."""
    seq = _fresh()
    _to_throw_dispatched(seq)
    seq.note_throw_dispatch(THROW_DISPATCH_OK)
    d = seq.step(5.4, _obs(5.4))
    assert not d.done
    d = seq.step(5.5, _obs(5.5))
    assert d.done and d.result.outcome == 'ABORTED_NO_RELEASE'
    assert d.action == ACTION_SAFE_ABORT


# ── Flight / settle ────────────────────────────────────────────────────────────

def test_happy_path_caught_stays_at_pose():
    """CAUGHT ⇒ ACTION_STAY at the shipped default (Phase E, 2026-07-29): the
    platform holds its catch pose so the NEXT toss reads its throw site A from
    there and a session chains A → B → C. Re-pointed from ACTION_RECENTER —
    which go_home'd, making every chained toss start from the origin again."""
    seq = _fresh()
    _to_in_flight(seq)
    d = seq.step(5.9, _obs(5.9, ball_caught=True, catch_error_mm=12.0))
    assert d.done and d.result.success is True
    assert d.result.outcome == 'CAUGHT'
    assert d.result.catch_error_mm == pytest.approx(12.0)
    assert d.action == ACTION_STAY


def test_settle_deadline_anchored_on_landing_not_release():
    """The settle deadline includes the time-of-flight (t_release + T + 0.7),
    so a catch confirmed after the scheduled landing still reads CAUGHT — not
    MISSED because the deadline treated RELEASE as landing."""
    seq = _fresh()
    _to_in_flight(seq)
    d = seq.step(5.85, _obs(5.85, ball_caught=False))  # past landing, in window
    assert not d.done
    d = seq.step(6.4, _obs(6.4, ball_caught=True, catch_error_mm=8.0))
    assert d.done and d.result.outcome == 'CAUGHT'
    assert d.action == ACTION_STAY


def test_missed_after_settle_window():
    seq = _fresh()
    _to_in_flight(seq)
    d = seq.step(6.5, _obs(6.5, ball_caught=False))
    assert d.done and d.result.outcome == 'MISSED'
    assert d.action == ACTION_SAFE_ABORT               # armed → safe the robot


def test_infeasible_does_not_terminate_mid_flight():
    """A standing catch-target rejection must NOT finish the sequence while the
    ball is in the air (retract-into-incoming-ball hazard — reload hardware
    evidence 2026-07-23, twice). The platform holds, the hand keeps its armed
    schedule, and the outcome resolves at settle."""
    seq = _fresh()
    _to_in_flight(seq)
    seq.note_catch_feasibility(False, 'WORKSPACE')
    d = seq.step(5.2, _obs(5.2))
    assert not d.done
    d = seq.step(6.5, _obs(6.5, ball_caught=False))
    assert d.done and d.result.outcome == 'MISSED_INFEASIBLE_WORKSPACE'
    assert d.action == ACTION_SAFE_ABORT


def test_infeasible_but_caught_is_caught():
    """An infeasibility that stood at catch time must not veto a
    tracker-confirmed CAUGHT — the primed, parked cup can catch a dead-centre
    toss the platform never reached for."""
    seq = _fresh()
    _to_in_flight(seq)
    seq.note_catch_feasibility(False, 'WORKSPACE')
    d = seq.step(5.9, _obs(5.9, ball_caught=True, catch_error_mm=20.0))
    assert d.done and d.result.success is True
    assert d.result.outcome == 'CAUGHT'
    assert d.action == ACTION_STAY


def test_accept_clears_standing_reject():
    """A genuine reject superseded by a later ACCEPT clears the standing
    infeasibility — only a rejection that still stands at catch time counts."""
    seq = _fresh()
    _to_in_flight(seq)
    seq.note_catch_feasibility(False, 'WORKSPACE')
    seq.note_catch_feasibility(True)
    d = seq.step(6.5, _obs(6.5, ball_caught=False))
    assert d.done and d.result.outcome == 'MISSED'


def test_accept_then_reject_stream_still_not_infeasible():
    """The reload third-sitting verdict bug, pinned for the toss: once ANY
    target is accepted the platform is holding a valid catch pose — a later
    stream of WORKSPACE rejects (corrupt split-track class) says nothing about
    reachability and must not flip the verdict to MISSED_INFEASIBLE."""
    seq = _fresh()
    _to_in_flight(seq)
    seq.note_catch_feasibility(True)                   # pre-tilt accepted
    for _ in range(30):                                # corrupt-track reject stream
        seq.note_catch_feasibility(False, 'WORKSPACE')
    d = seq.step(6.5, _obs(6.5, ball_caught=False))
    assert d.done and d.result.outcome == 'MISSED'     # honest: reachable, not seated


def test_feasibility_during_preparing_counts():
    """DELIBERATE DEVIATION from reload's _throw_sent gate: in a toss the
    pre-tilt target (and its acceptance) is triggered by OUR announcement,
    which precedes the throw dispatch — feasibility gates on the PREPARE having
    been DISPATCHED. Reload's gate transposed literally would drop this accept
    deterministically and mint a false MISSED_INFEASIBLE on every real toss."""
    seq = _fresh()
    _to_prepared(seq)                                  # PREPARE dispatched, unconfirmed
    seq.note_catch_feasibility(True)                   # pre-tilt accept counts NOW
    seq.note_prepare_result(True)
    seq.step(0.9, _obs(0.9))                           # ANNOUNCE
    seq.note_announcement()
    seq.step(1.0, _obs(1.0))                           # DISPATCH
    seq.note_throw_dispatch(THROW_DISPATCH_OK)
    seq.step(1.1, _obs(1.1, throw_stroke_seen=True))   # BALL_IN_FLIGHT
    seq.note_catch_feasibility(False, 'WORKSPACE')     # in-flight garbage reject
    d = seq.step(6.5, _obs(6.5, ball_caught=False))
    assert d.done and d.result.outcome == 'MISSED'     # not MISSED_INFEASIBLE


def test_feasibility_before_prepare_dispatch_ignored():
    """Target feedback arriving before the PREPARE was dispatched (stray targets
    from a previous flight's still-alive tracks) must not seed this goal's
    verdict — in either direction: an early reject must not pre-latch
    infeasibility, and an early accept must not pre-clear it."""
    seq = _fresh()
    seq.note_catch_feasibility(False, 'WORKSPACE')     # pre-prepare: ignored
    _to_in_flight(seq)
    d = seq.step(6.5, _obs(6.5, ball_caught=False))
    assert d.done and d.result.outcome == 'MISSED'     # not MISSED_INFEASIBLE

    seq2 = _fresh()
    _to_positioning(seq2)
    seq2.note_catch_feasibility(True)                  # pre-prepare accept: ignored
    seq2.note_position_result(0.05, True, 0.5)
    seq2.step(0.8, _obs(0.8))                          # PREPARE
    seq2.note_prepare_result(True)
    seq2.step(0.9, _obs(0.9))                          # ANNOUNCE
    seq2.note_announcement()
    seq2.step(1.0, _obs(1.0))                          # DISPATCH
    seq2.note_throw_dispatch(THROW_DISPATCH_OK)
    seq2.step(1.1, _obs(1.1, throw_stroke_seen=True))  # BALL_IN_FLIGHT
    seq2.note_catch_feasibility(False, 'WORKSPACE')    # real standing reject
    d = seq2.step(6.5, _obs(6.5, ball_caught=False))
    assert d.done and d.result.outcome == 'MISSED_INFEASIBLE_WORKSPACE'


def test_phase_reports_catching_near_landing():
    """The reported phase flips BALL_IN_FLIGHT → CATCHING at scheduled landing −
    confirm window (5.8 − 0.7 = 5.1)."""
    seq = _fresh()
    _to_in_flight(seq)
    d = seq.step(5.0, _obs(5.0))
    assert d.phase == PHASE_BALL_IN_FLIGHT
    d = seq.step(5.15, _obs(5.15))
    assert d.phase == PHASE_CATCHING


# ── Universal aborts + terminal discipline ─────────────────────────────────────

@pytest.mark.parametrize('stage,expected_action', [
    ('positioning_preaccept', ACTION_NONE),
    ('positioning_accepted', ACTION_SAFE_ABORT),
    ('preparing', ACTION_SAFE_ABORT),
    ('throwing', ACTION_SAFE_ABORT),
    ('in_flight', ACTION_SAFE_ABORT),
])
def test_mode_change_aborts_per_phase(stage, expected_action):
    """The per-phase abort enumeration (plan § Choreography 7): a mode exit
    terminates immediately in EVERY phase — trajectory_node force-disarms the
    latch on the mode change anyway, so mid-flight the catch is already dead
    and SAFE_ABORT is the honest cleanup. Cleanup only once something moved or
    armed: a pre-accept POSITIONING abort has nothing to safe."""
    seq = _fresh()
    if stage == 'positioning_preaccept':
        _to_positioning(seq)
        t = 0.1
    elif stage == 'positioning_accepted':
        _to_positioning(seq)
        seq.note_position_result(0.05, True, 0.5)
        t = 0.2
    elif stage == 'preparing':
        _to_prepared(seq)
        t = 0.9
    elif stage == 'throwing':
        _to_throw_dispatched(seq)
        seq.note_throw_dispatch(THROW_DISPATCH_OK)
        t = 1.1
    else:                                              # in_flight
        _to_in_flight(seq)
        t = 2.0
    d = seq.step(t, _obs(t, control_mode='STANDBY'))
    assert d.done and d.result.outcome == 'ABORTED_MODE_CHANGED'
    assert d.action == expected_action


def test_finished_is_terminal():
    seq = _fresh()
    d = seq.step(0.0, _obs(0.0, streaming=False))
    assert d.done
    first = d.result
    assert first is not None
    # Any subsequent step REPLAYS the stored result (not None) with ACTION_NONE,
    # so a stray extra step never hands the consumer a None — and never re-runs
    # the terminal action.
    d2 = seq.step(1.0, _obs(1.0))
    assert d2.done and d2.result is first
    assert d2.action == ACTION_NONE


def test_terminal_action_fires_once_then_replays_none():
    seq = _fresh()
    _to_in_flight(seq)
    d = seq.step(5.9, _obs(5.9, ball_caught=True))
    assert d.action == ACTION_STAY
    d2 = seq.step(6.0, _obs(6.0, ball_caught=True))
    assert d2.done and d2.action == ACTION_NONE


def test_prepared_property_gates_early_exit():
    """`prepared` keys the node's early-exit safing (cancel/timeout/shutdown):
    False through CHECKING and a merely-DISPATCHED move (a rejected or
    unanswered go_to_pose never moved the platform), True from the position
    ACCEPT (a moved platform needs the go_home leg) and from the PREPARE
    dispatch onward."""
    seq = _fresh()
    assert seq.prepared is False
    _to_positioning(seq)
    assert seq.prepared is False                       # dispatched ≠ accepted
    seq.note_position_result(0.05, True, 0.5)
    assert seq.prepared is True
    seq2 = _fresh()
    _to_prepared(seq2)
    assert seq2.prepared is True


# ── Result fields ──────────────────────────────────────────────────────────────

def test_achieved_flight_from_time_at_land():
    """achieved_flight_s = the tracker's LAST live landing-plane crossing minus
    the COMMANDED release (5.79 − 5.0 = 0.79 s) — a diagnostic field carrying
    the unmeasured command→release latency until T0 measures it."""
    seq = _fresh()
    _to_in_flight(seq)
    seq.step(5.2, _obs(5.2, ball_track_confirmed=True,
                       ball_time_at_land_perf=5.83))
    seq.step(5.5, _obs(5.5, ball_track_confirmed=True,
                       ball_time_at_land_perf=5.79))
    d = seq.step(5.9, _obs(5.9, ball_caught=True, catch_error_mm=10.0))
    assert d.done and d.result.outcome == 'CAUGHT'
    assert d.result.achieved_flight_s == pytest.approx(0.79)


def test_achieved_flight_nan_without_confirmation():
    """No tracking-CONFIRMED evidence ever (stroke-signature-only flight) ⇒ the
    crossing estimate is untrusted ⇒ NaN."""
    seq = _fresh()
    _to_in_flight(seq)                                 # stroke evidence only
    d = seq.step(6.5, _obs(6.5, ball_caught=False))
    assert d.done and d.result.outcome == 'MISSED'
    assert math.isnan(d.result.achieved_flight_s)


@pytest.mark.parametrize('stage', ['reject', 'abort', 'missed'])
def test_catch_error_nan_on_all_non_caught_terminals(stage):
    """catch_error_mm is meaningful ONLY at a plausible-CAUGHT tick.
    DELIBERATE DEVIATION from reload (which passes the observation through on
    MISSED): a finite estimate at a missed settle is the in-flight KF's last
    guess about a ball that did NOT land there — the toss forces NaN on every
    non-CAUGHT terminal."""
    seq = _fresh()
    if stage == 'reject':
        d = seq.step(0.0, _obs(0.0, streaming=False, catch_error_mm=55.0))
    elif stage == 'abort':
        _to_throw_dispatched(seq)
        seq.note_throw_dispatch(THROW_DISPATCH_OK)
        d = seq.step(5.5, _obs(5.5, catch_error_mm=55.0))
    else:                                              # missed
        _to_in_flight(seq)
        d = seq.step(6.5, _obs(6.5, ball_caught=False, catch_error_mm=55.0))
    assert d.done and d.result.success is False
    assert math.isnan(d.result.catch_error_mm)


def test_phase_strings_match_action_spec():
    """The 7 feedback phases are LOCKED by Toss.action — string pins."""
    assert PHASE_CHECKING == 'CHECKING'
    assert PHASE_POSITIONING == 'POSITIONING'
    assert PHASE_PREPARING == 'PREPARING'
    assert PHASE_THROWING == 'THROWING'
    assert PHASE_BALL_IN_FLIGHT == 'BALL_IN_FLIGHT'
    assert PHASE_CATCHING == 'CATCHING'
    assert PHASE_SETTLING == 'SETTLING'


# ── Config drift guards ────────────────────────────────────────────────────────

def test_tier_constant_matches_config():
    """The FSM's serviceable tier MUST mirror the generated config gate
    (jugglebot_operational.toss_tier → JB_OP_TOSS_TIER) — the coordinator
    passes the config value to the ctor, and a drifted default would either
    reject every goal or silently serve an unimplemented tier.

    CHANGE HISTORY (the value is a deliberate operator decision, not a
    convention): shipped '8a' from the plan's Phase 1 through 2026-07-27. Tier
    8b landed behind this same key at Phase 4 (2026-07-25) and was
    hardware-validated at the 2026-07-27 sitting — T4, displaced throw→catch at
    the then-70 mm cap — after which the operator made 8b the DEFAULT
    (2026-07-28). On 2026-08-10 the operator flipped the shipped tier back to
    '8a'; the 8b CAPABILITY stays in the code behind this key, and the tests that
    exercise it now pin `hw.JB_OP_TOSS_TIER` explicitly.

    WHAT THIS GUARD ASSERTS, and why it is no longer a literal. It used to read
    `== TIER_8B`, on the reasoning that a set-membership test would stay green
    through an accidental revert of the shipped default. That reasoning fails
    against the actual threat model: the shipped tier is an OPERATOR decision
    that changes on hardware evidence, so a literal makes every deliberate flip
    look like a regression and trains the next reader to retune the test rather
    than read it. What must never happen silently is DRIFT — the YAML edited
    without `python config/generate_config.py`, leaving the node reading a
    stale generated tier while the file the operator edited says otherwise. So
    the two legs are: the generated constant equals the YAML it is generated
    from, and whatever they agree on is a tier the FSM actually implements
    (an unimplemented value would REJECT_TIER every goal at runtime).
    Sibling of test_hand_ball_detect_config.py::test_generated_pin_matches_the_yaml."""
    import yaml

    from jugglebot.hardware_config import JB_OP_TOSS_TIER

    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    with open(os.path.join(repo_root, 'config', 'hardware_config.yaml')) as f:
        yml = yaml.safe_load(f)

    assert yml['jugglebot_operational']['toss_tier'] == JB_OP_TOSS_TIER, (
        'hardware_config.yaml jugglebot_operational.toss_tier != generated '
        'JB_OP_TOSS_TIER — run: python config/generate_config.py')
    assert JB_OP_TOSS_TIER in (TIER_8A, TIER_8B), (
        f'shipped toss_tier {JB_OP_TOSS_TIER!r} is not a tier the FSM serves '
        f'({TIER_8A} / {TIER_8B}) — every Toss goal would be REJECTED_TIER')


def test_local_constants_match_generated_config():
    """toss_sequencer keeps its physical constants local (importable-standalone,
    the reload BB-enum pattern) — this is the drift guard that pins them to the
    generated inputs they were derived from. DEFAULT_TOSS_FLIGHT_TIME_S is the
    FSM's no-config fallback for the generated JB_OP_TOSS_FLIGHT_TIME_DEFAULT_S
    the coordinator actually resolves — a drift between them would make a
    zero-flight goal fly differently depending on which layer resolved it."""
    import jugglebot.hardware_config as hw
    assert hw.TEENSY_TRAJ_MIN_EVENT_VEL_MPS == pytest.approx(0.3)
    assert hw.TEENSY_TRAJ_MAX_EVENT_VEL_MPS == pytest.approx(7.0)
    assert hw.JB_OP_DEFAULT_ACTIVE_Z_MM == pytest.approx(170.0)
    assert hw.HAND_CATCH_OFFSET_MM == pytest.approx(64.78)
    assert HAND_THROW_RELEASE_OFFSET_MM == pytest.approx(
        hw.GEOM_HAND_AXIS_BOTTOM_OFFSET_MM + hw.HAND_THROW_POS_M * 1000.0)
    assert hw.JB_OP_TOSS_FLIGHT_TIME_DEFAULT_S == pytest.approx(
        DEFAULT_TOSS_FLIGHT_TIME_S)
    # The Phase-E displacement cap and the workspace box are OPERATOR-ADJUSTABLE
    # YAML keys (2026-08-14): no YAML == module-literal equality pin, because
    # that pin is precisely what made a YAML edit turn the suite red. What IS
    # pinned: (a) the module fallbacks keep their documented values, so
    # standalone/bag use is deterministic; (b) the MECHANISM — the ctor value is
    # what gates (test_workspace_box_is_ctor_config and the 8b displacement
    # battery drive explicit ctor values); (c) the RELATIONAL invariant the YAML
    # comment states: box ≥ cap × 1.03, else the centroid-vs-cup chain
    # divergence (2.07 % of displacement, measured 2026-07-29) re-binds at the
    # cap edge and REJECTED_CHAIN_UNREACHABLE returns at the working range.
    assert TOSS_MAX_DISPLACEMENT_MM == pytest.approx(150.0)
    assert hw.JB_OP_TOSS_WORKSPACE_XY_MM >= (
        hw.JB_OP_TOSS_MAX_DISPLACEMENT_MM * 1.03)
    # The closed-form reach bound and the cap are SEPARATE gates; pin the
    # FALLBACK-limits crossover flight explicitly so a change to either shows
    # up as the flight-band shift it really is.
    assert reach_displacement_limit_mm(0.669) == pytest.approx(
        TOSS_MAX_DISPLACEMENT_MM, abs=0.5)
    # The FALLBACK limits the closed-form bound gates against when no live
    # session limits are observed (pre-field publisher, stale status, bag
    # replay). The pin matters because the fallback path can still over-permit
    # exactly the way the pre-2026-08-14 module copies did: lower
    # JB_TRAJ_LEG_JERK_LIMIT_MMPS3 to 15000 in YAML and a stale copy here would
    # pass a 150 mm goal at T = 0.70 (copy bound 171.5 mm) against a real
    # budget of 85.8 mm — a TOO_FAST at t_release with the ball airborne, the
    # mid-flight-verdict class C-REACH-1 exists to remove. The LIVE path closes
    # that hazard for a running system (the gate follows trajectory/status →
    # set_limits, test_reach_bound_prefers_live_session_limits); this pin
    # closes it for the fallback path.
    assert REACH_VEL_LIMIT_MMPS == pytest.approx(hw.JB_TRAJ_LEG_VEL_LIMIT_MMPS)
    assert REACH_ACC_LIMIT_MMPS2 == pytest.approx(hw.JB_TRAJ_LEG_ACC_LIMIT_MMPS2)
    assert REACH_JERK_LIMIT_MMPS3 == pytest.approx(
        hw.JB_TRAJ_LEG_JERK_LIMIT_MMPS3)


def test_stay_at_pose_default_matches_config():
    """The CAUGHT terminal's stay-vs-recenter default is a config key, and the
    ctor default must equal it — otherwise a standalone FSM (tests, the sim
    harness) would model a different terminal than the robot runs.

    CHANGE HISTORY: the CAUGHT terminal was ACTION_RECENTER (go_home) from the
    plan's Phase 1 until 2026-07-29, when the operator ordered session chaining
    (throw A → B → C without repositioning between throws) — which requires the
    platform to still BE at B when the next goal reads its throw site."""
    import jugglebot.hardware_config as hw
    assert hw.JB_OP_TOSS_STAY_AT_POSE_ON_CAUGHT is True
    seq = _fresh()
    assert seq.stay_at_pose_on_caught is True


def test_recenter_still_available_behind_the_config_key():
    """stay_at_pose_on_caught=False restores the pre-2026-07-29 CAUGHT terminal
    verbatim. The escape hatch must actually work: if staying turns out to drop
    balls out of a tilted cup at the bench, the operator's remedy is one config
    flip, not a code change under time pressure."""
    seq = _fresh(stay_at_pose_on_caught=False)
    _to_in_flight(seq)
    d = seq.step(5.9, _obs(5.9, ball_caught=True))
    assert d.done and d.result.outcome == 'CAUGHT'
    assert d.action == ACTION_RECENTER


@pytest.mark.parametrize('stay', [True, False])
@pytest.mark.parametrize('drive,outcome', [
    ('missed', 'MISSED'),
    ('no_release', 'ABORTED_NO_RELEASE'),
])
def test_not_caught_terminals_always_safe_abort(stay, drive, outcome):
    """Every NOT-caught terminal keeps SAFE_ABORT (retract + latch-lower +
    go_home) in BOTH stay settings. A miss leaves a loose ball and possibly a
    hand at the top of its stroke — safing to a known pose is the honest
    cleanup, and chaining (the only thing staying buys) is exactly what the miss
    already ended."""
    seq = _fresh(stay_at_pose_on_caught=stay)
    if drive == 'missed':
        _to_in_flight(seq)
        d = seq.step(6.5, _obs(6.5, ball_caught=False))
    else:
        _to_throw_dispatched(seq)
        seq.note_throw_dispatch(THROW_DISPATCH_OK)
        d = seq.step(5.5, _obs(5.5))
    assert d.done and d.result.outcome == outcome
    assert d.action == ACTION_SAFE_ABORT


# ── Tier 8b (Phase 4): displaced-throw CHECKING gates + deferred A→B reach ─────

def _fresh_8b(pose=(50.0, 0.0, 170.0), throw_site=(0.0, 0.0),
              flight_time_s=0.8, **kw):
    """A fresh Tier-8b sequencer: a displaced catch B (pose) thrown from site A
    (throw_site). event_vel defaults to the 8a vertical closed form — the FSM
    gate under test is the displacement/clamp CHECKING logic, not the aim math
    (the node feeds the real tilted event_vel + tilt_clamp_exceeded flag)."""
    params = dict(catch_pose_stow_mm=pose, flight_time_s=flight_time_s,
                  throw_delay_s=5.0, tier=TIER_8B, throw_site_xy_mm=throw_site,
                  throw_site_known=True)
    params.update(kw)
    seq = TossSequencer(**params)
    seq.start(0.0)
    return seq


@pytest.mark.parametrize('pose,site,flight', [
    ((160.0, 0.0, 170.0), (0.0, 0.0), 0.8),    # 160 mm > 150 cap (within 256 bound)
    ((0.0, -160.0, 170.0), (0.0, 0.0), 0.8),   # −y direction, cap alone
    ((100.0, 0.0, 170.0), (0.0, 0.0), 0.55),   # 100 mm ≤ cap but > 83.2 mm @0.55
    ((150.0, 0.0, 170.0), (0.0, 0.0), 0.60),   # 150 mm ≤ cap but > 108.0 mm @0.60
    ((110.0, 0.0, 170.0), (-90.0, 0.0), 0.8),  # non-origin A: |B−A| = 200 > cap
])
def test_displacement_rejected(pose, site, flight):
    """REJECTED_DISPLACEMENT: |B_xy − A_xy| past the cap OR past the flight's
    closed-form quintic reach bound — a loud PRE-THROW verdict for a reach whose
    trajectory_node verdict would otherwise arrive only after the ball flies.
    Checked BEFORE workspace, so a displaced-but-in-workspace B still rejects.

    Re-pointed 2026-07-29 from the 70 mm cap to 150 mm (Phase E). Two of the
    five legs now exercise the OTHER gate deliberately: at T = 0.55 and T = 0.60
    the closed-form bound (83.2 / 108.0 mm) bites well inside the cap, which is
    the gate doing its job — the production planner is only 3/8 directions at
    150 mm / T = 0.55 s (tools/probes/displaced_reach_frontier.py)."""
    seq = _fresh_8b(pose=pose, throw_site=site, flight_time_s=flight)
    d = seq.step(0.0, _obs(0.0))
    assert d.done and d.result.outcome == 'REJECTED_DISPLACEMENT'
    assert d.action == ACTION_NONE and seq.prepared is False


@pytest.mark.parametrize('pose,site,flight', [
    ((50.0, 0.0, 170.0), (0.0, 0.0), 0.8),       # the pre-Phase-E working point
    ((150.0, 0.0, 170.0), (0.0, 0.0), 0.8),      # AT the new cap, on-axis
    ((150.0, 150.0, 170.0), (44.0, 44.0), 0.8),  # at the cap from an off-centre A
    ((0.0, 0.0, 170.0), (-106.0, -106.0, ), 0.8),  # 150 mm INTO the centre
])
def test_displacement_within_cap_accepted(pose, site, flight):
    """|B−A| within the cap AND the flight's quintic reach bound starts the
    positioning move — the deferred A→B reach is feasible. Includes 150 mm legs
    from off-centre throw sites, which is the Phase-E working range."""
    seq = _fresh_8b(pose=pose, throw_site=site, flight_time_s=flight)
    d = seq.step(0.0, _obs(0.0))
    assert d.phase == PHASE_POSITIONING and d.action == ACTION_POSITION_PLATFORM


@pytest.mark.parametrize('next_b,box_160_outcome', [
    ((0.0, 0.0, 170.0), 'REJECTED_DISPLACEMENT'),
                                # back to centre: |B−A| = 153.10 > the 150 cap —
                                # a REAL requested displacement past the cap, so
                                # the wider box does not (and must not) admit it
    ((-150.0, 0.0, 170.0), 'ACCEPTED'),   # degenerate re-throw at the park
    ((-100.0, 0.0, 170.0), 'ACCEPTED'),   # a short hop back toward centre
])
def test_chaining_at_the_cap_box_dissolves_the_frame_divergence(next_b,
                                                                box_160_outcome):
    """The FORMER known limitation and its 2026-08-14 resolution, both pinned.

    The catch parks the platform CENTROID outside B so the CUP lands ON B
    (swing compensation: centroid = landing - hand_catch_offset*platform_z).
    Measured through the production chain at B = (-150, 0, 170), T = 0.80 s: the
    cup ends at exactly (-150.00, 0) but the centroid ends at (-153.10, 0), and
    trajectory/commanded_position publishes the CENTROID. So the next goal reads
    A = -153.10 — a value 2.07 % of the displacement (hand_catch_offset*sin of
    the receive tilt) outside nominal.

    WITH BOX == CAP == 150 (the module fallback, first arm): the A-box trips at
    the cap edge, chaining worked below ~146 mm and was REFUSED at 150 — the
    original limitation, kept pinned because a standalone/no-config sequencer
    still behaves this way.

    WITH THE SHIPPED BOX (toss_workspace_xy_mm = 160 > cap × 1.03, second arm):
    the parked centroid sits INSIDE the box, so degenerate and short-hop chain
    goals are ADMITTED — the limitation is dissolved by the box/cap separation,
    not by a frame decision. Back-to-centre still refuses on the |B−A| CAP,
    which is correct: 153.10 mm is a genuinely requested displacement past the
    cap, and the remedy (raise toss_max_displacement_mm, or one go_home) is the
    operator's. The centroid-vs-cup frame question itself (A is read as a
    centroid, compute_release_state_tilted consumes it as the CUP xy) remains
    open on single-ball-toss Phase E but no longer gates chaining."""
    parked_centroid_x = -153.10     # measured, not assumed
    seq = _fresh_8b(pose=next_b, throw_site=(parked_centroid_x, 0.0),
                    flight_time_s=0.80)
    d = seq.step(0.0, _obs(0.0))
    assert d.done and d.result is not None
    assert d.result.outcome in ('REJECTED_WORKSPACE', 'REJECTED_DISPLACEMENT')
    # The SAME goal from the nominal (cup) site is accepted at the fallback box
    # — which is what made this a frame divergence rather than a cap that was
    # simply too low.
    ok = _fresh_8b(pose=next_b, throw_site=(-150.0, 0.0), flight_time_s=0.80)
    d_ok = ok.step(0.0, _obs(0.0))
    assert d_ok.phase == PHASE_POSITIONING and d_ok.action == ACTION_POSITION_PLATFORM
    # The resolution arm: the shipped YAML box admits the chain goals the
    # fallback refused, and leaves the cap verdict standing where the request
    # really does exceed the cap.
    import jugglebot.hardware_config as hw
    shipped = _fresh_8b(pose=next_b, throw_site=(parked_centroid_x, 0.0),
                        flight_time_s=0.80,
                        workspace_xy_mm=float(hw.JB_OP_TOSS_WORKSPACE_XY_MM))
    d_s = shipped.step(0.0, _obs(0.0))
    if box_160_outcome == 'ACCEPTED':
        assert (d_s.phase == PHASE_POSITIONING
                and d_s.action == ACTION_POSITION_PLATFORM)
    else:
        assert d_s.done and d_s.result.outcome == box_160_outcome


def test_reach_displacement_limit_closed_form():
    """The closed-form quintic reach bound (module session limits): d_max =
    min(vel·T/1.875, acc·T²/5.7735, jerk·T³/60). Spot values pinned.

    At the OLD 70 mm cap the jerk term (≥ 83.2 mm at T = 0.55 s) always exceeded
    the cap, so this bound never bound. At the Phase-E 150 mm cap it is LIVE and
    is the binding gate below T ≈ 0.669 s — pinned here so a silent change to
    either the factors or the session limits shows up as the flight-band shift
    it would really be."""
    assert reach_displacement_limit_mm(0.55) == pytest.approx(83.19, abs=0.1)
    assert reach_displacement_limit_mm(0.60) == pytest.approx(108.0, abs=0.1)
    assert reach_displacement_limit_mm(0.80) == pytest.approx(256.0, abs=0.5)
    # The cap is NOT unconditionally slack any more: it binds only above the
    # crossover T = (60·cap/jerk)^(1/3) = 0.669 s.
    assert reach_displacement_limit_mm(0.55) < TOSS_MAX_DISPLACEMENT_MM
    assert reach_displacement_limit_mm(0.80) > TOSS_MAX_DISPLACEMENT_MM
    crossover = (60.0 * TOSS_MAX_DISPLACEMENT_MM / 30000.0) ** (1.0 / 3.0)
    assert crossover == pytest.approx(0.669, abs=0.001)
    assert reach_displacement_limit_mm(crossover) == pytest.approx(
        TOSS_MAX_DISPLACEMENT_MM, abs=1e-6)


def test_reach_bound_prefers_live_session_limits():
    """The closed-form reach bound follows the LIVE session limits
    (trajectory/set_limits → trajectory/status → TossObservations) and falls
    back to the YAML-default module copies only when they are absent (0.0).

    This is the 2026-08-14 authority change: the operator's set_limits ramp is
    the primary movement-acceptance authority, so the pre-throw gate must move
    in lockstep with the feasibility gate it fronts for — in BOTH directions.
    RAMP UP: at T = 0.60 the fallback jerk bound is 108.0 mm, refusing a
    150 mm goal the planner would fly at raised limits; with jerk ramped to
    200 000 the binding term becomes acc (5000·0.36/5.7735 = 311.8 mm) and the
    goal passes. RAMP DOWN: at T = 0.70 the fallback bound is 171.5 mm, passing
    a 100 mm goal that a cautious jerk = 15 000 session cannot fly
    (60·d/T³ ≤ 15 000 ⇒ 85.75 mm) — pre-live-limits that goal TOO_FAST-ed at
    t_release with the ball airborne (the C-REACH-1 mid-flight-verdict class);
    now it refuses pre-throw."""
    # Function level: explicit live values move the bound; None reproduces the
    # fallback exactly.
    assert reach_displacement_limit_mm(0.60, jerk_mmps3=200000.0) == (
        pytest.approx(311.77, abs=0.1))
    assert reach_displacement_limit_mm(0.70, jerk_mmps3=15000.0) == (
        pytest.approx(85.75, abs=0.1))
    assert reach_displacement_limit_mm(0.80, None, None, None) == (
        reach_displacement_limit_mm(0.80))
    # FSM level, ramp UP: 150 mm @ T = 0.60 refuses at the fallback limits
    # (the parametrized battery above pins that) and passes on a live-limits
    # observation. The cap does not bind (d == cap, not >).
    up = _fresh_8b(pose=(150.0, 0.0, 170.0), flight_time_s=0.60)
    d = up.step(0.0, _obs(0.0, leg_vel_limit_mmps=1000.0,
                          leg_acc_limit_mmps2=5000.0,
                          leg_jerk_limit_mmps3=200000.0))
    assert d.phase == PHASE_POSITIONING and d.action == ACTION_POSITION_PLATFORM
    # FSM level, ramp DOWN: 100 mm @ T = 0.70 passes at the fallback limits but
    # refuses against a live cautious-session jerk.
    down_ok = _fresh_8b(pose=(100.0, 0.0, 170.0), flight_time_s=0.70)
    d_ok = down_ok.step(0.0, _obs(0.0))
    assert d_ok.phase == PHASE_POSITIONING
    down = _fresh_8b(pose=(100.0, 0.0, 170.0), flight_time_s=0.70)
    d_ref = down.step(0.0, _obs(0.0, leg_vel_limit_mmps=1000.0,
                                leg_acc_limit_mmps2=5000.0,
                                leg_jerk_limit_mmps3=15000.0))
    assert d_ref.done and d_ref.result.outcome == 'REJECTED_DISPLACEMENT'
    # A partial triple (only some fields live) mixes live and fallback
    # per-term — the `or None` mapping is per-field, not all-or-nothing.
    assert reach_displacement_limit_mm(
        0.60, vel_mmps=None, acc_mmps2=None, jerk_mmps3=200000.0) == (
        pytest.approx(311.77, abs=0.1))


def test_workspace_box_is_ctor_config():
    """The ±workspace box is a ctor value the node resolves from YAML
    (toss_workspace_xy_mm → hw.JB_OP_TOSS_WORKSPACE_XY_MM), not a hardcoded
    constant (2026-08-14 — it was the hardcoded ±150, which made raising the
    displacement cap past 150 inert: REJECTED_WORKSPACE caught the same goal
    one line later). The module literal is only the no-config fallback, and
    both the B-box and the 8b A-box read the SAME ctor value."""
    # B-box, Tier 8a: 180 mm is outside the 150 fallback, inside a 200 box.
    ref = _fresh(catch_pose_stow_mm=(180.0, 0.0, 170.0))
    d_ref = ref.step(0.0, _obs(0.0))
    assert d_ref.done and d_ref.result.outcome == 'REJECTED_WORKSPACE'
    wide = _fresh(catch_pose_stow_mm=(180.0, 0.0, 170.0),
                  workspace_xy_mm=200.0)
    d_wide = wide.step(0.0, _obs(0.0))
    assert (d_wide.phase == PHASE_POSITIONING
            and d_wide.action == ACTION_POSITION_PLATFORM)
    # A-box, Tier 8b: A = (155, 0) is outside the fallback box while |B−A| =
    # 105 mm stays inside the cap and the T = 0.8 reach bound (256 mm), so the
    # verdict isolates the A-box; the same goal passes with the wider box.
    ref_a = _fresh_8b(pose=(50.0, 0.0, 170.0), throw_site=(155.0, 0.0))
    d_a = ref_a.step(0.0, _obs(0.0))
    assert d_a.done and d_a.result.outcome == 'REJECTED_WORKSPACE'
    wide_a = _fresh_8b(pose=(50.0, 0.0, 170.0), throw_site=(155.0, 0.0),
                       workspace_xy_mm=200.0)
    d_wa = wide_a.step(0.0, _obs(0.0))
    assert (d_wa.phase == PHASE_POSITIONING
            and d_wa.action == ACTION_POSITION_PLATFORM)


def test_pose_unknown_rejected_before_every_displacement_gate():
    """REJECTED_POSE_UNKNOWN: an 8b goal whose throw site A could not be read
    from the live commanded pose is refused BEFORE the cap, the reach bound and
    the tilt clamp — all three are functions of |B − A| and are meaningless
    without A. The goal below would trip the cap AND the clamp if A were
    assumed to be the origin, and it still reads POSE_UNKNOWN."""
    seq = _fresh_8b(pose=(150.0, 0.0, 170.0), throw_site=(0.0, 0.0),
                    flight_time_s=0.55, throw_site_known=False,
                    tilt_clamp_exceeded=True)
    d = seq.step(0.0, _obs(0.0))
    assert d.done and d.result.outcome == 'REJECTED_POSE_UNKNOWN'
    assert d.action == ACTION_NONE and seq.prepared is False


def test_pose_unknown_never_gates_tier_8a():
    """Tier 8a's throw site IS its catch site by definition, so it never needs a
    live pose read — throw_site_known=False must not refuse it."""
    seq = TossSequencer(catch_pose_stow_mm=(50.0, 0.0, 170.0),
                        flight_time_s=0.8, throw_delay_s=5.0,
                        throw_site_known=False)
    seq.start(0.0)
    d = seq.step(0.0, _obs(0.0))
    assert d.phase == PHASE_POSITIONING and d.action == ACTION_POSITION_PLATFORM


def test_max_displacement_is_ctor_resolved_not_the_module_constant():
    """The cap is a ctor parameter (the node passes the config value); the
    module constant is only the standalone default. A session that lowers it
    must actually bind — otherwise a cautious operator's config edit would be
    silently ignored."""
    seq = _fresh_8b(pose=(100.0, 0.0, 170.0), throw_site=(0.0, 0.0),
                    max_displacement_mm=70.0)
    d = seq.step(0.0, _obs(0.0))
    assert d.done and d.result.outcome == 'REJECTED_DISPLACEMENT'
    # …and the same goal passes at the shipped cap.
    seq2 = _fresh_8b(pose=(100.0, 0.0, 170.0), throw_site=(0.0, 0.0))
    assert seq2.step(0.0, _obs(0.0)).action == ACTION_POSITION_PLATFORM


def test_tilt_clamp_rejected():
    """REJECTED_TILT_CLAMP: the node maps compute_release_state_tilted's
    ThrowTiltInfeasible raise onto tilt_clamp_exceeded; CHECKING mints the
    reject (a silently clamped aim lands short of B — the Rung-2a landing bias —
    so the toss refuses loudly rather than fly mis-aimed). Displacement within
    the cap, so the clamp gate is what fires."""
    seq = _fresh_8b(pose=(50.0, 0.0, 170.0), throw_site=(0.0, 0.0),
                    tilt_clamp_exceeded=True)
    d = seq.step(0.0, _obs(0.0))
    assert d.done and d.result.outcome == 'REJECTED_TILT_CLAMP'
    assert d.action == ACTION_NONE and seq.prepared is False


def test_tilt_clamp_rejects_in_tier_8a_too():
    """AUDIT FIX 2026-08-11. The clamp check used to live INSIDE the 8b block,
    which was right while the only source of a ThrowTiltInfeasible raise was a
    displaced goal near the 12° ceiling. The AIM CORRECTION added a second
    source present in BOTH tiers: the node tilts toward a virtual target for the
    aim whatever the tier, and a raise there leaves `release_cmd` None, so
    `event_vel_mps` falls back to 0.0.

    8b-only, an 8a goal in that state fell through to the EVENT_VEL band and
    reported REJECTED_EVENT_VEL — fail-closed, but naming the Teensy speed limit
    for an aim-ceiling fault, which routes the operator to the wrong subsystem.
    Note the zero event_vel below: it reproduces the node's own fallback, so the
    test fails on the pre-fix code for the right reason."""
    seq = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                        flight_time_s=0.8, throw_delay_s=5.0,
                        event_vel_mps=0.0, tilt_clamp_exceeded=True)
    seq.start(0.0)
    d = seq.step(0.0, _obs(0.0))
    assert d.done and d.result.outcome == 'REJECTED_TILT_CLAMP'
    assert d.action == ACTION_NONE and seq.prepared is False


def test_displacement_precedes_tilt_clamp():
    """Gate order: the displacement cap is checked BEFORE the tilt clamp — a
    goal that trips both reads REJECTED_DISPLACEMENT (the primary contract)."""
    seq = _fresh_8b(pose=(100.0, 0.0, 170.0), throw_site=(0.0, 0.0),
                    flight_time_s=0.55, tilt_clamp_exceeded=True)
    d = seq.step(0.0, _obs(0.0))
    assert d.done and d.result.outcome == 'REJECTED_DISPLACEMENT'


def test_reach_catch_emitted_once_time_triggered_at_release():
    """Tier 8b's deferred A→B reach: ACTION_REACH_CATCH fires exactly once, on
    the FIRST tick with now >= t_release (TIME-triggered, NOT evidence-triggered
    — release evidence can lag the 0.5 s grace and eat the reach lead).
    t_release = 5.0 (delay 5.0 from start(0.0))."""
    seq = _fresh_8b(pose=(50.0, 0.0, 170.0), throw_site=(0.0, 0.0))
    _to_in_flight(seq)                        # in flight well before t_release
    d = seq.step(4.9, _obs(4.9))
    assert d.action == ACTION_NONE            # not yet due
    d = seq.step(5.0, _obs(5.0))
    assert d.action == ACTION_REACH_CATCH     # first tick at/after t_release
    d = seq.step(5.1, _obs(5.1))
    assert d.action == ACTION_NONE            # never re-emitted (commitment flag)


def test_reach_catch_emitted_even_without_release_evidence():
    """TIME-triggered: the reach goes out at t_release even with NO release
    evidence (the stroke may have silently never fired). The platform translates
    A→B carrying the seated ball — the benign-accel class — and
    ABORTED_NO_RELEASE still cleans up at t_release + grace."""
    seq = _fresh_8b(pose=(50.0, 0.0, 170.0), throw_site=(0.0, 0.0))
    _to_throw_dispatched(seq)
    seq.note_throw_dispatch(THROW_DISPATCH_OK)
    d = seq.step(5.0, _obs(5.0))              # t_release, no stroke/track evidence
    assert d.phase == PHASE_THROWING and d.action == ACTION_REACH_CATCH
    d = seq.step(5.5, _obs(5.5))              # t_release + grace: abort
    assert d.done and d.result.outcome == 'ABORTED_NO_RELEASE'
    assert d.action == ACTION_SAFE_ABORT


def test_8a_never_emits_reach_catch():
    """Tier 8a's decision stream is byte-identical to Phase 1 — the Phase-4
    ACTION_REACH_CATCH never appears, on the same tick schedule an 8b goal would
    emit it on. The dispatch is still exactly-once."""
    seq = _fresh()                            # tier 8a (default)
    actions = [seq.step(0.0, _obs(0.0)).action]
    seq.note_position_result(0.05, True, 0.5)
    actions.append(seq.step(0.8, _obs(0.8)).action)
    seq.note_prepare_result(True)
    actions.append(seq.step(0.9, _obs(0.9)).action)
    seq.note_announcement()
    actions.append(seq.step(1.0, _obs(1.0)).action)
    seq.note_throw_dispatch(THROW_DISPATCH_OK)
    actions.append(seq.step(1.1, _obs(1.1, throw_stroke_seen=True)).action)
    for t in (4.9, 5.0, 5.9):                 # across + past t_release
        actions.append(seq.step(t, _obs(t, ball_caught=(t == 5.9),
                                        catch_error_mm=10.0)).action)
    assert ACTION_REACH_CATCH not in actions
    assert actions.count(ACTION_DISPATCH_THROW) == 1


def test_8b_stream_equals_8a_plus_reach_catch():
    """Byte-identity pin: an 8b goal with throw site == B (displacement 0,
    otherwise identical to 8a) produces the 8a decision stream with EXACTLY one
    ACTION_REACH_CATCH inserted at the first tick past t_release — nothing else
    about the sequence changes."""
    schedule = [(0.0, {}), (0.8, {}), (0.9, {}), (1.0, {}),
                (1.1, {'throw_stroke_seen': True}), (4.9, {}), (5.0, {}),
                (5.9, {'ball_caught': True, 'catch_error_mm': 8.0})]

    def _run(tier):
        seq = TossSequencer(catch_pose_stow_mm=CATCH_POSE, flight_time_s=0.8,
                            throw_delay_s=5.0, tier=tier,
                            throw_site_xy_mm=(0.0, 0.0), throw_site_known=True)
        seq.start(0.0)
        acts = []
        for i, (t, kw) in enumerate(schedule):
            acts.append(seq.step(t, _obs(t, **kw)).action)
            if i == 0:
                seq.note_position_result(0.05, True, 0.5)
            elif i == 1:
                seq.note_prepare_result(True)
            elif i == 2:
                seq.note_announcement()
            elif i == 3:
                seq.note_throw_dispatch(THROW_DISPATCH_OK)
        return acts

    acts_8a = _run(TIER_8A)
    acts_8b = _run(TIER_8B)
    assert ACTION_REACH_CATCH not in acts_8a
    assert acts_8b.count(ACTION_REACH_CATCH) == 1
    # Tick-for-tick identical EXCEPT the single tick where 8b emits REACH_CATCH
    # (there 8a idles ACTION_NONE) — the reach REPLACES a quiet tick, it never
    # adds or reorders one.
    assert len(acts_8a) == len(acts_8b)
    diffs = [(i, a, b) for i, (a, b) in enumerate(zip(acts_8a, acts_8b))
             if a != b]
    assert len(diffs) == 1
    _i, a8a, a8b = diffs[0]
    assert a8a == ACTION_NONE and a8b == ACTION_REACH_CATCH


def test_announce_lead_short_warns_for_8b_too():
    """The announce-lead hardening was SUPERSEDED for 8b: the 8b platform reach
    is deferred to t_release with lead = the flight time BY CONSTRUCTION, so the
    2.5 s announce lead sizes no 8b motion. 8b WARNs and proceeds exactly like
    8a — a literal hardening would brick every floor-delay 8b toss."""
    seq = _fresh_8b(pose=(0.0, 0.0, 170.0), throw_site=(0.0, 0.0))  # displacement 0
    _to_positioning(seq)
    seq.note_position_result(0.05, True, 3.55)    # arrival 3.8
    d = seq.step(3.8, _obs(3.8))
    assert d.phase == PHASE_PREPARING and d.action == ACTION_PREPARE_CATCH
    seq.note_prepare_result(True)
    d = seq.step(3.85, _obs(3.85))                # lead 5.8 − 3.85 = 1.95 < 2.5
    assert d.action == ACTION_ANNOUNCE
    assert seq.announce_lead_short is True         # WARN flag, never an abort
    seq.note_announcement()
    d = seq.step(3.9, _obs(3.9))
    assert d.phase == PHASE_THROWING and d.action == ACTION_DISPATCH_THROW
