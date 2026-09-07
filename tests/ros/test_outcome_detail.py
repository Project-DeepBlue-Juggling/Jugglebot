"""THE shared refusal-detail vocabulary (``jugglebot.outcome_detail``).

Three FSMs and the coordinator node mint operator-facing outcome strings, and
since 2026-08-29 every limit-bearing refusal carries its numbers in a
parenthetical: the requested value, the nearest limit, and the knob that moves
it. This module is the single place that punctuation is decided, so a drift here
is a drift in every refusal at once.

It is also the place three PRODUCTION GUARDS now go through — ``toss_session``'s
auto-reload trigger and no-release retry, and the node's zombie-move superseder
— which is why ``base_outcome`` and ``outcome_subcode`` get their own rows
rather than being tested only through their callers. A matcher that stops
matching does nothing VISIBLE: no interlude, no retry, no ``go_home`` under a
plan that may still be executing. These are the rows that would notice.
"""

import math

import pytest

from jugglebot.outcome_detail import (
    base_outcome,
    bound_msg,
    outcome_subcode,
    range_msg,
)


# ── base_outcome ──────────────────────────────────────────────────────────────

@pytest.mark.parametrize('outcome,code', [
    ('REJECTED_NO_BALL', 'REJECTED_NO_BALL'),               # bare, unchanged
    ('REJECTED_DWELL(dwell 0.90 s < floor 1.04 s)', 'REJECTED_DWELL'),
    ('REJECTED_POSITION(BUSY: a move is in flight)', 'REJECTED_POSITION'),
    # COMPOSED forms: the session prefixes the cycle terminal and the reload
    # interlude prefixes the reload one. Both prefix, neither wraps, so
    # splitting at the FIRST paren is right for every form the machine mints.
    ('ABORTED_CYCLE_REJECTED_DWELL(dwell 0.90 s)',
     'ABORTED_CYCLE_REJECTED_DWELL'),
    ('STOPPED_RELOAD_REJECTED_NOT_CENTERED(platform is 150.0 mm)',
     'STOPPED_RELOAD_REJECTED_NOT_CENTERED'),
    # A detail containing its own parentheses (the reach-drift refusal quotes
    # two poses) still yields the code.
    ('REJECTED_REACH_CENTER_DRIFT(B (0.0, 86.5, 170.0) mm is 86.5 mm)',
     'REJECTED_REACH_CENTER_DRIFT'),
    # UH-7a's two. The beat floor is a Layer-B (node) refusal that aborts the
    # goal before any cycle exists, so it reaches the result UNWRAPPED; the chain
    # loss is minted on a cycle and reaches it through the session's prefix.
    # Both have to round-trip, because `_TOSS_POSITION_UNKNOWN_TERMINALS`-style
    # (code, subcode) matching is how a guard tells one from another.
    ('REJECTED_BEAT_TOO_SHORT(dwell 0.750 s < unified chain floor (the next '
     "cycle's lead) 0.800 s; the chain has TWO requirements)",
     'REJECTED_BEAT_TOO_SHORT'),
    ('ABORTED_CYCLE_REJECTED_CHAIN_LOST(NO_WINDOW: STEADY: … | LANDING: … '
     '— held (trajectory/hold: ok))', 'ABORTED_CYCLE_REJECTED_CHAIN_LOST'),
    # The ring that lost its window ENTIRELY: nothing chained, the machine held,
    # and the session stopped. A SESSION terminal, so unwrapped.
    ('STOPPED_CHAIN_LOST(NO_WINDOW: UNACKED after 0.980 s — held '
     '(trajectory/hold: holding at current pose))', 'STOPPED_CHAIN_LOST'),
    ('STOPPED_CHAIN_LOST', 'STOPPED_CHAIN_LOST'),        # bare, no detail
    # …and the ring that was TRUNCATED rather than lost: the fall-back LANDING
    # installed, the ball was caught, and the session stopped by name. It is a
    # SESSION terminal, so it reaches the result unwrapped like COMPLETED and
    # STOPPED_ON_MISS do, and it carries the planner's own composed refusal —
    # which has its own parentheses, so this row is also the nested-detail case.
    ('STOPPED_CHAIN_REFUSED(REJECTED_CYCLE_INFEASIBLE(LIMIT_JERK: 198431 > '
     '150000 mm/s^3))', 'STOPPED_CHAIN_REFUSED'),
    ('', ''),
])
def test_base_outcome_recovers_the_code(outcome, code):
    assert base_outcome(outcome) == code


def test_base_outcome_coerces_rather_than_raises():
    """The callers are guards on a TERMINAL path — an exception there is
    strictly worse than a mismatch, because it takes the cleanup down with it.
    ``None`` from a half-built result must therefore fall through harmlessly."""
    assert base_outcome(None) == 'None'


# ── outcome_subcode ───────────────────────────────────────────────────────────

@pytest.mark.parametrize('outcome,sub', [
    # RELAYED verdicts put the other layer's code first.
    ('REJECTED_POSITION(NO_RESPONSE)', 'NO_RESPONSE'),
    ('REJECTED_POSITION(BUSY: a move is in flight — Phase 2)', 'BUSY'),
    ('REJECTED_POSITION(WIRE_DISARMED: planned [wire DISARMED …])',
     'WIRE_DISARMED'),
    ('REJECTED_THROW_ENVELOPE(END_STOP:modelled peak 10.660 rev)', 'END_STOP'),
    # DESCRIPTIVE details are prose and have no subcode to mistake for one —
    # which is what stops a (code, subcode) guard from matching by accident.
    ('REJECTED_WORKSPACE(B.z 225.0 mm is 55.0 mm > band 50.0 mm '
     '[TOSS_Z_BAND_MM]; measured from the ACTIVE plane 170.0 mm)', ''),
    ('REJECTED_DWELL(dwell 0.90 s < floor 1.04 s; max(throw_delay 0.60))', ''),
    ('REJECTED_NOT_CENTERED(the live commanded pose is UNKNOWN or stale)', ''),
    ('REJECTED_NO_BALL', ''),                    # no parenthetical at all
    ('ABORTED_POSITION_TIMEOUT', ''),
    # UH-7a: the chain-loss refusal leads with a RELAYED-shaped subcode so an
    # operator guard can match (code, subcode) without parsing the two nested
    # refusals it carries…
    ('REJECTED_CHAIN_LOST(NO_WINDOW: STEADY: REJECTED_CYCLE_INFEASIBLE'
     '(LIMIT_JERK: …) | LANDING: NO_CYCLE — held)', 'NO_WINDOW'),
    # …while the beat floor's detail is PROSE, so it has no subcode to be
    # mistaken for one. That asymmetry is the point of the two rows: the first
    # names a mechanism a guard routes on, the second names a number an operator
    # reads.
    ('REJECTED_BEAT_TOO_SHORT(dwell 0.750 s < unified chain floor (the next '
     "cycle's lead) 0.800 s; the chain has TWO requirements)", ''),
    # STOPPED_CHAIN_LOST leads its detail with a flat, matchable NO_WINDOW…
    ('STOPPED_CHAIN_LOST(NO_WINDOW: UNACKED after 0.980 s — held)',
     'NO_WINDOW'),
    # …and carries none when there is no detail at all.
    ('STOPPED_CHAIN_LOST', ''),
    # A truncated ring carries the planner's ALREADY-COMPOSED refusal, so its
    # detail opens with a nested `CODE(` rather than a bare relayed token — and
    # this contract answers "no subcode", correctly. That is worth pinning
    # rather than working around: it is the difference between a guard reading
    # (code, subcode) and one that would have to parse two levels of
    # parentheses. The routable fact is the base outcome; the layer that refused
    # is read out of the detail as prose, exactly as the operator reads it.
    ('STOPPED_CHAIN_REFUSED(REJECTED_CYCLE_INFEASIBLE(LIMIT_JERK: …))', ''),
])
def test_outcome_subcode_tells_a_relayed_code_from_prose(outcome, sub):
    assert outcome_subcode(outcome) == sub


def test_the_position_terminals_guard_matches_both_forms():
    """The shape the node's ``_TOSS_POSITION_UNKNOWN_TERMINALS`` guard relies
    on: enriching a refusal must not move it out of the set.

    That set used to hold the literal ``'REJECTED_POSITION(NO_RESPONSE)'``. Keyed
    on ``(code, subcode)`` instead, the bare and the enriched forms land on the
    same key — which is the whole point, because the enriched form is what a
    trajectory_node with something to say now produces."""
    terminals = frozenset({('REJECTED_POSITION', 'NO_RESPONSE'),
                           ('ABORTED_POSITION_TIMEOUT', '')})
    for outcome in ('REJECTED_POSITION(NO_RESPONSE)',
                    'REJECTED_POSITION(NO_RESPONSE: service unavailable)',
                    'ABORTED_POSITION_TIMEOUT'):
        assert (base_outcome(outcome), outcome_subcode(outcome)) in terminals
    # …and a DIFFERENT refusal of the same code still stays out of it: the
    # superseder exists for the UNKNOWN-motion cases only.
    for outcome in ('REJECTED_POSITION(BUSY: a move is in flight)',
                    'REJECTED_POSITION(WORKSPACE)'):
        assert (base_outcome(outcome),
                outcome_subcode(outcome)) not in terminals


# ── the formatters ────────────────────────────────────────────────────────────

def test_bound_msg_names_value_limit_and_knob():
    """The three things the operator needs, in one clause. The unit is repeated
    on BOTH sides deliberately — ``0.90 s < 1.04 s`` cannot be misread as a
    mixed-unit comparison the way ``0.90 s < 1.04`` can."""
    assert bound_msg('|B-A| =', 178.0, '>', 160.0, 'mm',
                     knob='live limits 1000/5000/30000') == (
        '|B-A| = 178.0 mm > 160.0 mm [live limits 1000/5000/30000]')
    assert bound_msg('dwell', 0.9, '<', 1.0416, 's', digits=3,
                     limit_label='floor',
                     tail='max(throw_delay 0.600 + handoff 0.442)') == (
        'dwell 0.900 s < floor 1.042 s; max(throw_delay 0.600 + handoff 0.442)')


def test_range_msg_names_both_ends_of_a_band():
    assert range_msg('num_throws', 25, 1, 20, digits=0,
                     knob='toss_session_max_throws') == (
        'num_throws 25 outside [1, 20] [toss_session_max_throws]')
    assert range_msg('event_vel', 7.5, 0.3, 7.0, 'm/s', digits=2,
                     knob='TEENSY_MIN/MAX_EVENT_VEL_MPS') == (
        'event_vel 7.50 m/s outside [0.30, 7.00] m/s '
        '[TEENSY_MIN/MAX_EVENT_VEL_MPS]')


@pytest.mark.parametrize('value', [float('nan'), float('inf'), float('-inf')])
def test_a_non_finite_value_is_rendered_not_swallowed(value):
    """A NaN goal field is exactly the case a refusal most needs to name — the
    numerics gate exists for it — so the formatter must print it rather than
    raise or blank it."""
    msg = bound_msg('throw_delay', value, '<', 0.441, 's', digits=3)
    assert ('nan' in msg or 'inf' in msg), msg
    assert math.isnan(value) or math.isinf(value)      # the row is honest


def test_an_omitted_knob_leaves_no_empty_brackets():
    """A derived floor has no single operator knob — it names its terms in the
    tail instead — so the bracket must disappear rather than render empty."""
    assert '[' not in bound_msg('dwell', 0.9, '<', 1.0, 's')
    # range_msg's own brackets are the BAND, so the check is for a trailing
    # empty knob rather than for brackets at all.
    assert range_msg('n', 3, 1, 2, digits=0) == 'n 3 outside [1, 2]'


# ── the wire map's COMPLETENESS ───────────────────────────────────────────────

def test_every_minted_outcome_code_has_an_operator_hint():
    """``REJECT_WIRE_MAP`` is the only enumeration of these codes anywhere, and
    nothing pinned that it was complete.

    That is how ``HAND_BELOW_FLOOR`` and ``UNIFIED_AIM_UNSUPPORTED`` sat unhinted
    from the day they were minted, and how ``STOPPED_CHAIN_LOST`` joined them
    within a session of being written: minting a code is one line, and the map
    lives in another file that nobody is forced to open. An operator reading a
    trace for an unhinted code gets a bare token and no next action — which is
    precisely the thing the table exists to prevent.

    So the check is structural rather than a list: every module constant on the
    two producers whose VALUE looks like an operator-facing terminal must have a
    key. A new code fails this the moment it is written, in the same commit,
    which is the only time the author knows what the hint should say.
    """
    import jugglebot.toss_session as tsess
    from jugglebot import reload_coordinator_node as rcn
    from tests.hardware.toss_trace_recorder import REJECT_WIRE_MAP

    codes = set()
    for mod in (tsess, rcn):
        for name in dir(mod):
            if not (name.startswith('OUTCOME_STOPPED_')
                    or name.startswith('_OUTCOME_')):
                continue
            value = getattr(mod, name)
            if isinstance(value, str) and value:
                codes.add(value)
    # COMPLETED / STOPPED_ON_MISS are not refusals and predate the table; every
    # other terminal an operator can read off a trace needs its next action.
    codes -= {tsess.OUTCOME_COMPLETED, tsess.OUTCOME_STOPPED_ON_MISS}
    missing = sorted(c for c in codes if c not in REJECT_WIRE_MAP)
    assert not missing, (
        'these outcome codes are minted but have no REJECT_WIRE_MAP hint, so a '
        'trace shows the operator a bare token and no next action: %s' % missing)
    # …and non-vacuity: the sweep must actually be finding the UH-7a codes.
    assert {'REJECTED_BEAT_TOO_SHORT', 'REJECTED_CHAIN_LOST',
            'STOPPED_CHAIN_LOST', 'STOPPED_CHAIN_REFUSED'} <= codes
