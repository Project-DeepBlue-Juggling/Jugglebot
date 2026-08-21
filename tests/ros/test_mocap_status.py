"""The ``mocap/status`` readiness predicate — jugglebot.mocap_status (F4).

This module is the CANONICAL definition of "QTM is ready for a BB calibration
sweep". Two nodes gate on it and they must answer identically:

* ``teensy_bridge_node._svc_bb_calibrate`` refuses the service call outright;
* ``orchestrator_node._dispatch_request('bb_calibrate')`` SKIPS the HOMING step.

Those two node-level behaviours are pinned in test_teensy_bridge_node_bb.py and
test_orchestrator_node.py respectively. What is pinned HERE is the predicate
itself, once — every refusal branch, every code, and the fail-closed default —
so that a threshold change has exactly one place it can be reviewed.

Pure Python: no ROS objects are constructed, so nothing here depends on the
mocked-ROS layer beyond ``tests/conftest.py``'s path setup.
"""

from __future__ import annotations

import pytest

from jugglebot import mocap_status as ms


def _kv(receiving='1', visible='5', marker3='1', aligned='1', synced='1'):
    """A decoded status dict — healthy by default, spoil one field per test."""
    return {
        ms.KEY_QTM_RECEIVING: receiving,
        ms.KEY_BB_MARKERS_VISIBLE: visible,
        ms.KEY_MARKER3_VISIBLE: marker3,
        ms.KEY_ALIGNED: aligned,
        ms.KEY_QTM_SYNCED: synced,
    }


class _FakeKeyValue:
    def __init__(self, key, value):
        self.key = key
        self.value = value


# ── decode ───────────────────────────────────────────────────────────────────

def test_decode_status_maps_keyvalues_to_dict():
    values = [_FakeKeyValue('a', '1'), _FakeKeyValue('b', 'two')]
    assert ms.decode_status(values) == {'a': '1', 'b': 'two'}


# ── the happy path ───────────────────────────────────────────────────────────

def test_healthy_status_is_ready():
    ready, code, detail = ms.evaluate(_kv(), age_s=0.1)
    assert ready is True
    assert code == ''
    assert detail == ''


def test_exactly_the_minimum_markers_is_ready():
    """The floor is inclusive: 3 visible incl. Marker 3 passes, 2 does not."""
    ready, _, _ = ms.evaluate(_kv(visible=str(ms.MIN_BB_MARKERS_VISIBLE)), age_s=0.1)
    assert ready is True


def test_age_exactly_at_the_limit_is_ready():
    """The staleness test is ``>`` not ``>=`` — a sample landing exactly on the
    limit must not manufacture a refusal out of scheduler jitter."""
    ready, _, _ = ms.evaluate(_kv(), age_s=ms.MOCAP_STATUS_MAX_AGE_S)
    assert ready is True


# ── QTM_STALE ────────────────────────────────────────────────────────────────

def test_no_status_ever_received_is_stale():
    """Fail CLOSED. Before the first message the caller passes None, and the
    only safe answer is 'refuse' — assuming ready would command a physical
    sweep on the strength of having heard nothing at all."""
    ready, code, detail = ms.evaluate(None, age_s=0.0)
    assert ready is False
    assert code == ms.CODE_QTM_STALE
    assert 'mocap_node' in detail


def test_stale_status_is_stale():
    ready, code, detail = ms.evaluate(_kv(), age_s=3.2)
    assert ready is False
    assert code == ms.CODE_QTM_STALE
    assert '3.2' in detail


def test_qtm_not_receiving_is_stale():
    """A FRESH status that says QTM is dark is still QTM_STALE — same code,
    because it sends the operator to the same place (the QTM machine), and the
    detail string is what distinguishes the two."""
    ready, code, detail = ms.evaluate(_kv(receiving='0'), age_s=0.1)
    assert ready is False
    assert code == ms.CODE_QTM_STALE
    assert 'no QTM packets' in detail


def test_missing_receiving_key_is_stale():
    kv = _kv()
    del kv[ms.KEY_QTM_RECEIVING]
    ready, code, _ = ms.evaluate(kv, age_s=0.1)
    assert ready is False
    assert code == ms.CODE_QTM_STALE


# ── BB_MARKERS_NOT_VISIBLE ───────────────────────────────────────────────────

def test_too_few_markers_is_not_visible():
    ready, code, detail = ms.evaluate(_kv(visible='2'), age_s=0.1)
    assert ready is False
    assert code == ms.CODE_BB_MARKERS_NOT_VISIBLE
    assert '2/5' in detail


def test_marker3_missing_is_not_visible_even_with_enough_markers():
    """run_calibration hard-requires Marker 3 (index 2) for the yaw offset, so
    4 visible markers WITHOUT it is still a refusal — a sweep would run to
    completion and then fail in the solver."""
    ready, code, detail = ms.evaluate(_kv(visible='4', marker3='0'), age_s=0.1)
    assert ready is False
    assert code == ms.CODE_BB_MARKERS_NOT_VISIBLE
    assert 'seen=no' in detail


def test_uncountable_marker_value_reads_as_zero():
    """Fail closed on garbage: a non-integer count must refuse, not raise (the
    caller is a service handler; an exception there is a ROS-level failure with
    no named code for the operator)."""
    ready, code, _ = ms.evaluate(_kv(visible='not-a-number'), age_s=0.1)
    assert ready is False
    assert code == ms.CODE_BB_MARKERS_NOT_VISIBLE


# ── the two codes are distinct ───────────────────────────────────────────────

def test_the_two_refusal_codes_differ():
    """Distinct codes per the toss-ladder doctrine: 'QTM is dark' and 'QTM is
    fine but cannot see the BB' are different bench errands, and one merged
    code would send the operator to the wrong one half the time."""
    assert ms.CODE_QTM_STALE != ms.CODE_BB_MARKERS_NOT_VISIBLE


@pytest.mark.parametrize('code', [ms.CODE_QTM_STALE, ms.CODE_BB_MARKERS_NOT_VISIBLE])
def test_codes_are_shouty_identifiers(code):
    """The code is meant to be greppable in a log and in this repo; keep it an
    upper-snake token rather than prose."""
    assert code == code.upper()
    assert ' ' not in code
