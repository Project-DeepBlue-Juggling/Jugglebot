"""Unit tests for controller/teensy_link/encoder_search.py.

The encoder-index-search orchestration state machine, tested in isolation (no
ROS, no UDP, no hardware) by driving :meth:`EncoderSearch.step` with controlled
monotonic times and per-axis status snapshots. End-to-end RPC issue + the
telemetry/diagnostic → AxisStatus mapping are wired (and integration-tested) at
the node level when Jetson-side encoder search lands in teensy_bridge_node.

Empirical recipe these tests encode (ground truth from the 2026-06-19 bench
bring-up + can_node._encoder_search_steps): a leg ODrive with CAN+main power, on
SET_AXIS_STATE(ENCODER_INDEX_SEARCH=6), enters state 6, spins to find the index,
returns to IDLE=1, and its encoder position transitions NaN → finite. Success ==
back to IDLE + finite pos + no active errors, with evidence the search ran.
"""

from __future__ import annotations

import pytest

from controller.teensy_link.encoder_search import (
    EncoderSearch,
    AxisStatus,
    AXIS_STATE_IDLE as IDLE,
    AXIS_STATE_ENCODER_INDEX_SEARCH as SEARCH,
)


def st(state: int, pos_finite: bool, errors: int = 0) -> AxisStatus:
    return AxisStatus(axis_state=state, pos_finite=pos_finite, active_errors=errors)


# ── construction guards ──────────────────────────────────────────────────────

def test_empty_axes_raises():
    with pytest.raises(ValueError):
        EncoderSearch([])


def test_duplicate_axes_raises():
    with pytest.raises(ValueError):
        EncoderSearch([0, 0])


# ── happy paths ──────────────────────────────────────────────────────────────

def test_single_axis_cold_start_success():
    """Standalone-leg rig (axis 0), NaN → search → finite."""
    es = EncoderSearch([0])

    # First step commands the search and nothing else.
    r = es.step(0.0, {})
    assert r.set_search == [0]
    assert r.clear_errors == []
    assert not r.done

    # Command not yet effected: still IDLE, encoder NaN.
    r = es.step(0.1, {0: st(IDLE, pos_finite=False)})
    assert not r.done and r.set_search == []

    # Axis enters the search state.
    r = es.step(0.5, {0: st(SEARCH, pos_finite=False)})
    assert not r.done

    # Search completes: IDLE, encoder now finite, no errors.
    r = es.step(1.8, {0: st(IDLE, pos_finite=True)})
    assert r.done
    assert r.succeeded == [0]
    assert r.failed == {}
    assert es.succeeded == [0]
    assert es.failed == {}


def test_single_axis_targeting_ignores_other_axes():
    """EncoderSearch([0]) only ever commands axis 0, even if status carries more."""
    es = EncoderSearch([0])
    r = es.step(0.0, {})
    assert r.set_search == [0]
    # Extra axes in the status dict are irrelevant.
    es.step(0.3, {0: st(SEARCH, False), 3: st(SEARCH, False)})
    r = es.step(1.0, {0: st(IDLE, True), 3: st(IDLE, True)})
    assert r.done and r.succeeded == [0]


def test_fast_search_missed_state_still_succeeds():
    """If the brief SEARCH state is missed between snapshots, the NaN → finite
    transition is sufficient evidence the search ran."""
    es = EncoderSearch([0])
    es.step(0.0, {})                              # command
    es.step(0.1, {0: st(IDLE, pos_finite=False)})  # observed NaN (saw_nonfinite)
    r = es.step(0.6, {0: st(IDLE, pos_finite=True)})  # already back to IDLE + finite
    assert r.done and r.succeeded == [0]


def test_all_legs_success():
    axes = list(range(6))
    es = EncoderSearch(axes)
    r = es.step(0.0, {})
    assert sorted(r.set_search) == axes
    es.step(0.3, {a: st(SEARCH, False) for a in axes})
    r = es.step(1.5, {a: st(IDLE, True) for a in axes})
    assert r.done and sorted(r.succeeded) == axes and r.failed == {}


def test_rehome_with_observed_search_succeeds():
    """An already-finite axis still succeeds if the SEARCH state is observed."""
    es = EncoderSearch([0])
    es.step(0.0, {})
    es.step(0.3, {0: st(SEARCH, pos_finite=True)})
    r = es.step(1.0, {0: st(IDLE, pos_finite=True)})
    assert r.done and r.succeeded == [0]


# ── safety: no false success ─────────────────────────────────────────────────

def test_prehomed_axis_does_not_false_succeed_without_search_evidence():
    """A finite pos at start with NO evidence the search ran must NOT be reported
    as success — otherwise we'd declare a leg 'homed' that never moved."""
    es = EncoderSearch([0], timeout_s=1.0, max_retries=0)
    es.step(0.0, {})
    r = es.step(0.5, {0: st(IDLE, pos_finite=True)})   # finite but no ran-evidence
    assert not r.done                                   # must keep waiting
    r = es.step(1.5, {0: st(IDLE, pos_finite=True)})   # ages out
    assert r.done and 0 in r.failed


# ── timeout / retry ──────────────────────────────────────────────────────────

def test_timeout_then_retry_succeeds():
    es = EncoderSearch([0], timeout_s=2.0, max_retries=1)
    es.step(0.0, {})                          # command (attempt 0, t0=0)
    es.step(0.5, {0: st(IDLE, False)})        # waiting (NaN, not entered)

    # Exceed the budget -> clear errors and re-command.
    r = es.step(2.5, {0: st(IDLE, False)})
    assert r.clear_errors == [0]
    assert r.set_search == [0]
    assert not r.done

    # Retry succeeds.
    es.step(2.6, {0: st(SEARCH, False)})
    r = es.step(3.5, {0: st(IDLE, True)})
    assert r.done and r.succeeded == [0]


def test_timeout_exhausts_retries_and_fails():
    es = EncoderSearch([0], timeout_s=1.0, max_retries=1)
    es.step(0.0, {})                          # attempt 0, t0=0
    r = es.step(1.5, {0: st(IDLE, False)})    # >1.0 -> retry (attempt 1, t0=1.5)
    assert r.clear_errors == [0] and r.set_search == [0] and not r.done
    r = es.step(3.0, {0: st(IDLE, False)})    # 3.0-1.5=1.5 > 1.0 -> fail
    assert r.done
    assert 0 in r.failed
    assert "timed out" in r.failed[0]
    assert es.succeeded == []


def test_no_telemetry_still_times_out():
    """An axis that never reports any status still ages to a timeout/failure."""
    es = EncoderSearch([0], timeout_s=1.0, max_retries=0)
    es.step(0.0, {})                 # command
    r = es.step(2.0, {})             # never any status -> timeout -> fail
    assert r.done and 0 in r.failed


# ── error handling ───────────────────────────────────────────────────────────

def test_error_during_search_retries_then_fails():
    es = EncoderSearch([0], timeout_s=5.0, max_retries=1)
    es.step(0.0, {})
    es.step(0.2, {0: st(SEARCH, False)})
    # An active error appears -> clear + retry.
    r = es.step(0.5, {0: st(IDLE, False, errors=0x100)})
    assert r.clear_errors == [0] and r.set_search == [0] and not r.done
    # Error persists on the retry -> fail.
    r = es.step(0.7, {0: st(IDLE, False, errors=0x100)})
    assert r.done and 0 in r.failed and "active_errors" in r.failed[0]


def test_idle_without_finite_pos_after_search_fails():
    """Search ran (state observed) but the axis returned to IDLE with a still-NaN
    encoder -> a real failure, not a success."""
    es = EncoderSearch([0], timeout_s=10.0, max_retries=0)
    es.step(0.0, {})
    es.step(0.2, {0: st(SEARCH, False)})              # saw_search
    r = es.step(1.0, {0: st(IDLE, pos_finite=False)})  # IDLE + NaN after search
    assert r.done and 0 in r.failed
    assert "without a finite encoder" in r.failed[0]


# ── mixed / lifecycle ────────────────────────────────────────────────────────

def test_partial_success_and_failure():
    es = EncoderSearch([0, 1], timeout_s=1.0, max_retries=0)
    es.step(0.0, {})
    es.step(0.2, {0: st(SEARCH, False), 1: st(IDLE, False)})   # axis 1 never enters
    r = es.step(1.5, {0: st(IDLE, True), 1: st(IDLE, False)})
    assert r.done
    assert r.succeeded == [0]
    assert 1 in r.failed


def test_idempotent_after_done():
    es = EncoderSearch([0])
    es.step(0.0, {})
    es.step(0.2, {0: st(SEARCH, False)})
    r = es.step(1.0, {0: st(IDLE, True)})
    assert r.done

    # Stepping past completion issues no new commands and stays done.
    r2 = es.step(2.0, {0: st(IDLE, True)})
    assert r2.done
    assert r2.set_search == [] and r2.clear_errors == []
    assert r2.succeeded == [0]


def test_done_property_tracks_progress():
    es = EncoderSearch([0, 1])
    assert not es.done
    es.step(0.0, {})
    assert not es.done
    es.step(0.2, {0: st(SEARCH, False), 1: st(SEARCH, False)})
    assert not es.done
    es.step(1.0, {0: st(IDLE, True), 1: st(IDLE, True)})
    assert es.done
