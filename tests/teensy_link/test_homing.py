"""Unit tests for teensy_link/homing.py (firmware-reported HomingResult; see logbook 2026-07-05-canhub-hardening-18a-homing-result-uplink).

The homing completion-observer state machine, tested in isolation (no ROS, no UDP,
no hardware) by driving :meth:`HomingMonitor.step` with controlled monotonic times
and per-axis status snapshots.

Ground truth = the firmware HOME handler (a port of can_node._home_motor_steps):
the can-bridge drives a leg into its hardstop, detects the current spike, IDLEs,
and snaps the encoder to the home reference — tracking the outcome in a per-axis
HomingResult (NONE/RUNNING/OK/FAILED). That result is now uplinked
in the Diagnostic and the observer TRUSTS it. It previously inferred success from a
CLOSED_LOOP→IDLE state cycle, which a silent firmware abort — which also IDLEs the
leg, but WITHOUT setting the reference — mimicked exactly (a false success that left
the leg's zero uncalibrated while the orchestrator proceeded).
"""

from __future__ import annotations

import pytest

from teensy_link.homing import (
    HomingMonitor,
    AxisStatus,
    AXIS_STATE_IDLE as IDLE,
    AXIS_STATE_CLOSED_LOOP as CL,
    HOMING_NONE, HOMING_RUNNING, HOMING_OK, HOMING_FAILED,
    DEFAULT_HOME_REF_REV,
)

REF = DEFAULT_HOME_REF_REV  # 0.1


def st(state: int, pos_rev: float, errors: int = 0,
       homing_result: int = HOMING_NONE) -> AxisStatus:
    return AxisStatus(axis_state=state, pos_rev=pos_rev, active_errors=errors,
                      homing_result=homing_result)


# ── construction guards ──────────────────────────────────────────────────────

def test_empty_axes_raises():
    with pytest.raises(ValueError):
        HomingMonitor([])


def test_duplicate_axes_raises():
    with pytest.raises(ValueError):
        HomingMonitor([0, 0])


# ── happy path (firmware reports HOMING_OK) ──────────────────────────────────

def test_single_axis_success():
    """RUNNING (the move started) then OK (hardstop found + reference set) → DONE."""
    m = HomingMonitor([0])
    r = m.step(0.0, {})
    assert r.set_home == [0] and not r.done
    r = m.step(0.1, {0: st(CL, pos_rev=1.234, homing_result=HOMING_RUNNING)})
    assert r.set_home == [] and not r.done
    r = m.step(0.5, {0: st(IDLE, pos_rev=-REF, homing_result=HOMING_OK)})
    assert r.done and m.succeeded == [0] and m.failed == {}


def test_success_ignores_telemetry_position():
    """The observer trusts the firmware result — the foam-relaxed post-IDLE position is
    irrelevant. Positions the OLD |pos|≈ref gate rejected still succeed."""
    for pos in (-0.10, -0.05, 0.0, -0.20, 2.49):
        m = HomingMonitor([0])
        m.step(0.0, {})
        m.step(0.1, {0: st(CL, pos_rev=1.0, homing_result=HOMING_RUNNING)})
        r = m.step(0.5, {0: st(IDLE, pos_rev=pos, homing_result=HOMING_OK)})
        assert r.done and m.succeeded == [0], f"pos={pos} should succeed"
        assert m.failed == {}


# ── the false-success class this observer closes ─────────────────────────────────────

def test_silent_abort_is_not_success():
    """THE false-success regression: a firmware abort (bus-down / guard-E-STOP mid-move)
    IDLEs the leg WITHOUT setting the reference → HOMING_FAILED. The state cycle
    (CLOSED_LOOP→IDLE) LOOKS identical to success; the result must override it."""
    m = HomingMonitor([0])
    m.step(0.0, {})
    m.step(0.1, {0: st(CL, pos_rev=1.0, homing_result=HOMING_RUNNING)})
    r = m.step(0.5, {0: st(IDLE, pos_rev=-REF, homing_result=HOMING_FAILED)})
    assert r.done and m.succeeded == [] and 0 in m.failed
    assert "firmware reported homing failed" in m.failed[0]


def test_idle_without_ok_is_not_success():
    """A CLOSED_LOOP→IDLE cycle with the result still RUNNING (not yet OK) must NOT
    be declared success — only HOMING_OK is success (guards the state-cycle mimic)."""
    m = HomingMonitor([0], timeout_s=1.0)
    m.step(0.0, {})
    m.step(0.1, {0: st(CL, pos_rev=1.0, homing_result=HOMING_RUNNING)})
    r = m.step(0.2, {0: st(IDLE, pos_rev=-REF, homing_result=HOMING_RUNNING)})
    assert not r.done   # firmware still RUNNING → keep waiting
    r = m.step(1.5, {0: st(IDLE, pos_rev=-REF, homing_result=HOMING_RUNNING)})
    assert r.done and 0 in m.failed  # timed out, NOT a false success


def test_stale_prior_ok_does_not_terminate_new_home():
    """saw_running gate: a stale HOMING_OK left in the cache from a PRIOR home must
    not immediately succeed the new one before its RUNNING is observed."""
    m = HomingMonitor([0], timeout_s=1.0)
    m.step(0.0, {})
    r = m.step(0.1, {0: st(IDLE, pos_rev=-REF, homing_result=HOMING_OK)})  # stale
    assert not r.done   # never saw RUNNING → ignore the stale OK
    m.step(0.2, {0: st(CL, pos_rev=1.0, homing_result=HOMING_RUNNING)})    # new move runs
    r = m.step(0.3, {0: st(IDLE, pos_rev=-REF, homing_result=HOMING_OK)})
    assert r.done and m.succeeded == [0]


def test_stale_prior_failed_does_not_fail_new_home():
    m = HomingMonitor([0], timeout_s=1.0)
    m.step(0.0, {})
    r = m.step(0.1, {0: st(IDLE, pos_rev=0.0, homing_result=HOMING_FAILED)})  # stale
    assert not r.done   # ignored (no RUNNING yet)
    m.step(0.2, {0: st(CL, pos_rev=1.0, homing_result=HOMING_RUNNING)})
    r = m.step(0.3, {0: st(IDLE, pos_rev=-REF, homing_result=HOMING_OK)})
    assert r.done and m.succeeded == [0]


# ── other failure paths ──────────────────────────────────────────────────────

def test_active_errors_during_homing_fails():
    m = HomingMonitor([0])
    m.step(0.0, {})
    m.step(0.1, {0: st(CL, pos_rev=1.0, homing_result=HOMING_RUNNING)})
    r = m.step(0.2, {0: st(CL, pos_rev=1.0, errors=0x40, homing_result=HOMING_RUNNING)})
    assert r.done and m.succeeded == [] and "active_errors" in m.failed[0]


def test_firmware_failed_while_still_closed_loop_fails():
    """The firmware can report FAILED without an IDLE transition (an abort caught
    mid-drive); the observer fails immediately on the result, not the state."""
    m = HomingMonitor([0])
    m.step(0.0, {})
    m.step(0.1, {0: st(CL, pos_rev=1.0, homing_result=HOMING_RUNNING)})
    r = m.step(0.5, {0: st(CL, pos_rev=1.0, homing_result=HOMING_FAILED)})
    assert r.done and m.succeeded == [] and "firmware reported homing failed" in m.failed[0]


def test_timeout_backstop_fails():
    """A leg that never resolves (no terminal result — e.g. a lost transition)
    fails at the host timeout, set below the firmware's own 30 s."""
    m = HomingMonitor([0], timeout_s=2.0)
    m.step(0.0, {})
    m.step(0.1, {0: st(CL, pos_rev=1.0, homing_result=HOMING_RUNNING)})
    r = m.step(1.9, {0: st(CL, pos_rev=1.0, homing_result=HOMING_RUNNING)})
    assert not r.done
    r = m.step(2.2, {0: st(CL, pos_rev=1.0, homing_result=HOMING_RUNNING)})
    assert r.done and "timed out" in m.failed[0]


# ── multi-axis bookkeeping (the firmware homes one at a time; the monitor still
#    tracks a set independently for the caller that serializes them) ────────────

def test_multi_axis_independent_outcomes():
    m = HomingMonitor([0, 1])
    r = m.step(0.0, {})
    assert sorted(r.set_home) == [0, 1]
    m.step(0.1, {0: st(CL, pos_rev=1.0, homing_result=HOMING_RUNNING),
                 1: st(CL, pos_rev=1.0, homing_result=HOMING_RUNNING)})
    # axis 0 succeeds (OK); axis 1 silently aborts (FAILED — the silent-abort case).
    r = m.step(0.5, {0: st(IDLE, pos_rev=-REF, homing_result=HOMING_OK),
                     1: st(IDLE, pos_rev=-REF, homing_result=HOMING_FAILED)})
    assert r.done
    assert m.succeeded == [0]
    assert 1 in m.failed
