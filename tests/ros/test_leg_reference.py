"""FW 53 U6b (R3): the "dropped Leg provably kept its reference" predicate
(jugglebot/leg_reference.py), its evidence watch, and the mixed-case verdict.

Each row of the predicate table names the predicate line it pins: deleting or
inverting that line (the mutation) turns exactly that row green-to-red.
"""
from __future__ import annotations

import math

import pytest

from jugglebot.leg_reference import (
    KEEPS_REFERENCE, REF_DRIFT_TOL_REV, LegDrop, LegDropWatch,
    mixed_case_verdict, reference_unproven,
)

IDLE, CL = 1, 8
CURRENT_LIMIT = 4096
MISSING_ESTIMATE = 8
INITIALIZING = 1
SPINOUT = 67108864
UNDER_VOLTAGE = 512
UNDECODED = 1 << 30 << 1          # 2**31: not in odrive.ERROR_CODES

# A clean CURRENT_LIMIT drop at the Active pose, sagged 0.09 rev, sealed at the clear.
GOOD = LegDrop(last_cl_pos=2.19, bits=CURRENT_LIMIT, reason_seen=True,
               reason_lapsed=False, odd_state=False, gap=False, max_drift=0.09,
               sealed=True)

# (id, drop, is_homed, state, fresh, expected substring or None = proven)
TABLE = [
    ('proven', GOOD, True, IDLE, True, None),
    ('every_keeps_bit_proven', GOOD._replace(bits=KEEPS_REFERENCE), True, IDLE, True, None),
    ('drift_at_the_limit_proven', GOOD._replace(max_drift=REF_DRIFT_TOL_REV), True, IDLE, True, None),
    ('not_homed', GOOD, False, IDLE, True, 'homed flag is clear'),
    ('no_record', None, True, IDLE, True, 'never saw it Closed-loop'),
    ('no_baseline', GOOD._replace(last_cl_pos=None), True, IDLE, True, 'never saw it Closed-loop'),
    ('stale_states', GOOD, True, IDLE, False, 'not known from a fresh'),
    ('state_unknown', GOOD, True, None, True, 'not known from a fresh'),
    ('not_idle', GOOD, True, 3, True, 'reads drive state 3'),
    ('gap', GOOD._replace(gap=True), True, IDLE, True, 'paused during the drop'),
    ('odd_state', GOOD._replace(odd_state=True), True, IDLE, True, 'other than Idle or Closed-loop'),
    ('no_reason', GOOD._replace(reason_seen=False), True, IDLE, True, 'no disarm reason'),
    ('reason_lapsed', GOOD._replace(reason_lapsed=True), True, IDLE, True, 'cleared before'),
    ('missing_estimate', GOOD._replace(bits=CURRENT_LIMIT | MISSING_ESTIMATE), True, IDLE, True, 'MISSING_ESTIMATE'),
    ('initializing', GOOD._replace(bits=INITIALIZING), True, IDLE, True, 'INITIALIZING'),
    ('spinout_unknown', GOOD._replace(bits=SPINOUT), True, IDLE, True, 'SPINOUT_DETECTED'),
    ('undervoltage_unknown', GOOD._replace(bits=UNDER_VOLTAGE), True, IDLE, True, 'DC_BUS_UNDER_VOLTAGE'),
    ('undecoded_bit_unknown', GOOD._replace(bits=UNDECODED), True, IDLE, True, 'bit 2147483648'),
    ('drift_over', GOOD._replace(max_drift=REF_DRIFT_TOL_REV + 0.01), True, IDLE, True, 'moved 0.26 rev'),
    ('drift_whole_turn', GOOD._replace(max_drift=1.0), True, IDLE, True, 'moved 1.00 rev'),
    ('drift_nan', GOOD._replace(max_drift=math.nan), True, IDLE, True, 'moved unknown'),
    ('drift_inf', GOOD._replace(max_drift=math.inf), True, IDLE, True, 'moved unknown'),
]


@pytest.mark.parametrize('drop,is_homed,state,fresh,expected',
                         [r[1:] for r in TABLE], ids=[r[0] for r in TABLE])
def test_predicate_table(drop, is_homed, state, fresh, expected):
    got = reference_unproven(drop, is_homed=is_homed, state=state, fresh=fresh)
    if expected is None:
        assert got is None
    else:
        assert got is not None and expected in got, got


def test_keeps_set_excludes_every_loss_and_unknown_bit():
    """The classification itself (logbook table): no loses/unknown bit may be in
    KEEPS_REFERENCE. Mutation caught: adding SPINOUT or MISSING_ESTIMATE to it."""
    for bit in (1, 2, 4, 8, 16, 32, 64, 512, 32768, 65536, 67108864, 1073741824):
        assert not KEEPS_REFERENCE & bit, bit


# ── the watch ─────────────────────────────────────────────────────────────────

def _feed(w, states, active=None, disarm=None, pos=None, gap=False):
    active = active or [0] * 6
    disarm = disarm or [0] * 6
    pos = pos or [2.19] * 6
    w.observe(states, active, disarm, pos, gap)


def _drop_leg2(w, sag=2.10, bits=CURRENT_LIMIT):
    _feed(w, [CL] * 6)
    _feed(w, [CL, CL, IDLE, CL, CL, CL], disarm=[0, 0, bits, 0, 0, 0],
          pos=[2.19, 2.19, sag, 2.19, 2.19, 2.19])


def test_watch_records_a_drop_and_closes_it_on_closed_loop():
    w = LegDropWatch()
    _drop_leg2(w)
    d = w.drop(2)
    assert d.last_cl_pos == pytest.approx(2.19) and d.bits == CURRENT_LIMIT
    assert d.reason_seen and not d.reason_lapsed and not d.gap and not d.odd_state
    assert d.max_drift == pytest.approx(0.09)
    assert w.drop(0) is None
    _feed(w, [CL] * 6)
    assert w.drop(2) is None


def test_watch_reason_lapse_before_the_seal_but_not_after():
    """A disarm reason going back to 0 before the operator's clear is a reboot sign;
    after seal() it is the clear. Mutation caught: the `not d.sealed` term removed
    (every recovery refused) or the lapse term removed (a reboot proven)."""
    w = LegDropWatch()
    _drop_leg2(w)
    _feed(w, [CL, CL, IDLE, CL, CL, CL], pos=[2.19, 2.19, 2.10, 2.19, 2.19, 2.19])
    assert w.drop(2).reason_lapsed

    w = LegDropWatch()
    _drop_leg2(w)
    w.seal()
    _feed(w, [CL, CL, IDLE, CL, CL, CL], pos=[2.19, 2.19, 2.10, 2.19, 2.19, 2.19])
    assert not w.drop(2).reason_lapsed and w.drop(2).sealed


def test_watch_marks_gap_odd_state_drift_and_ors_bits():
    w = LegDropWatch()
    _drop_leg2(w)
    _feed(w, [CL, CL, 0, CL, CL, CL], active=[0, 0, INITIALIZING, 0, 0, 0],
          disarm=[0, 0, CURRENT_LIMIT, 0, 0, 0], pos=[2.19, 2.19, 0.19, 2.19, 2.19, 2.19],
          gap=True)
    d = w.drop(2)
    assert d.gap and d.odd_state and d.bits == CURRENT_LIMIT | INITIALIZING
    assert d.max_drift == pytest.approx(2.0)
    # The drift is the MAX over the drop: coming back does not erase it.
    _feed(w, [CL, CL, IDLE, CL, CL, CL], disarm=[0, 0, CURRENT_LIMIT, 0, 0, 0])
    assert w.drop(2).max_drift == pytest.approx(2.0)


def test_watch_nan_length_and_no_baseline_are_unknown():
    w = LegDropWatch()
    _feed(w, [CL, CL, IDLE, CL, CL, CL], disarm=[0, 0, CURRENT_LIMIT, 0, 0, 0])
    assert w.drop(2).last_cl_pos is None and w.drop(2).max_drift == math.inf
    w = LegDropWatch()
    _feed(w, [CL] * 6)
    _feed(w, [CL, CL, IDLE, CL, CL, CL], disarm=[0, 0, CURRENT_LIMIT, 0, 0, 0],
          pos=[2.19, 2.19, math.nan, 2.19, 2.19, 2.19])
    assert w.drop(2).max_drift == math.inf


def test_closed_loop_with_an_active_error_opens_a_drop():
    """Clean means Closed-loop AND no active error (legs_closed_loop's own rule)."""
    w = LegDropWatch()
    _feed(w, [CL] * 6)
    _feed(w, [CL] * 6, active=[0, 0, CURRENT_LIMIT, 0, 0, 0])
    assert w.drop(2) is not None and w.drop(2).bits == CURRENT_LIMIT


# ── the mixed-case verdict ────────────────────────────────────────────────────

def test_verdict_proven_mixed():
    w = LegDropWatch()
    _drop_leg2(w)
    w.seal()
    assert mixed_case_verdict((CL, CL, IDLE, CL, CL, CL), True, True, w) == (True, [2], None)


def test_verdict_not_mixed_cases():
    w = LegDropWatch()
    assert mixed_case_verdict((CL,) * 6, True, True, w)[0] is False
    assert mixed_case_verdict((IDLE,) * 6, True, True, w)[0] is False
    assert mixed_case_verdict(None, True, True, w)[0] is False
    assert mixed_case_verdict((CL, CL, IDLE, CL, CL, CL), False, True, w)[0] is False
    assert mixed_case_verdict((CL, IDLE), True, True, w)[0] is False


def test_verdict_names_every_unproven_leg():
    w = LegDropWatch()
    _feed(w, [CL] * 6)
    _feed(w, [CL, IDLE, IDLE, CL, CL, CL], disarm=[0, CURRENT_LIMIT, MISSING_ESTIMATE, 0, 0, 0])
    mixed, dropped, refusal = mixed_case_verdict((CL, IDLE, IDLE, CL, CL, CL), True, True, w)
    assert mixed and dropped == [1, 2]
    assert 'Leg 2: ' in refusal and 'MISSING_ESTIMATE' in refusal
    assert 'Leg 1:' not in refusal
