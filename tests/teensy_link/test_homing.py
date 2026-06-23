"""Unit tests for controller/teensy_link/homing.py (Phase 9b).

The homing completion-observer state machine, tested in isolation (no ROS, no
UDP, no hardware) by driving :meth:`HomingMonitor.step` with controlled monotonic
times and per-axis status snapshots.

What the observer encodes (ground truth = the firmware HOME handler, a port of
can_node._home_motor_steps): the can-bridge drives a leg in VELOCITY/VEL_RAMP
into its hardstop, detects the current spike, IDLEs, and snaps the encoder to the
home reference. The Jetson sees the axis cycle IDLE → CLOSED_LOOP → IDLE with the
position landing at |pos| ≈ |home_ref| on success. Telemetry pos_rev is Jugglebot
convention (sign-flipped on RX), so set_absolute_position(+0.1) reads back -0.1 —
the observer compares magnitudes.
"""

from __future__ import annotations

import pytest

from controller.teensy_link.homing import (
    HomingMonitor,
    AxisStatus,
    AXIS_STATE_IDLE as IDLE,
    AXIS_STATE_CLOSED_LOOP as CL,
    DEFAULT_HOME_REF_REV,
)

REF = DEFAULT_HOME_REF_REV  # 0.1


def st(state: int, pos_rev: float, errors: int = 0) -> AxisStatus:
    return AxisStatus(axis_state=state, pos_rev=pos_rev, active_errors=errors)


# ── construction guards ──────────────────────────────────────────────────────

def test_empty_axes_raises():
    with pytest.raises(ValueError):
        HomingMonitor([])


def test_duplicate_axes_raises():
    with pytest.raises(ValueError):
        HomingMonitor([0, 0])


# ── happy path ───────────────────────────────────────────────────────────────

def test_single_axis_success_sign_flipped_pos():
    """Standalone-leg rig (axis 0): IDLE → CLOSED_LOOP → IDLE, pos lands at the
    sign-flipped reference (-0.1 for a +0.1 set_absolute_position)."""
    m = HomingMonitor([0])

    # First step commands HOME and nothing else.
    r = m.step(0.0, {})
    assert r.set_home == [0]
    assert not r.done

    # Move starts: axis enters CLOSED_LOOP, position still arbitrary.
    r = m.step(0.1, {0: st(CL, pos_rev=1.234)})
    assert r.set_home == [] and not r.done

    # Hardstop found, IDLE, reference snapped to -0.1 (Jugglebot convention).
    r = m.step(0.5, {0: st(IDLE, pos_rev=-REF)})
    assert r.done
    assert m.succeeded == [0]
    assert m.failed == {}


def test_success_positive_convention_pos():
    """Magnitude comparison is convention-agnostic: +0.1 also succeeds."""
    m = HomingMonitor([0])
    m.step(0.0, {})
    m.step(0.1, {0: st(CL, pos_rev=2.0)})
    r = m.step(0.5, {0: st(IDLE, pos_rev=REF)})
    assert r.done and m.succeeded == [0]


def test_missed_closed_loop_snapshot_then_idle_still_needs_cl():
    """If CLOSED_LOOP is never observed, a stray IDLE+near-ref must NOT count as
    success (guards against the pre-homing position coincidentally being ~ref)."""
    m = HomingMonitor([0], timeout_s=1.0)
    m.step(0.0, {})
    # Never saw CL; IDLE at ~ref should be ignored until timeout.
    r = m.step(0.2, {0: st(IDLE, pos_rev=-REF)})
    assert not r.done
    r = m.step(1.5, {0: st(IDLE, pos_rev=-REF)})
    assert r.done and m.failed and 0 in m.failed  # timed out, not a false success


# ── failure paths ────────────────────────────────────────────────────────────

def test_active_errors_during_homing_fails():
    m = HomingMonitor([0])
    m.step(0.0, {})
    m.step(0.1, {0: st(CL, pos_rev=1.0)})
    r = m.step(0.2, {0: st(CL, pos_rev=1.0, errors=0x40)})
    assert r.done and m.succeeded == []
    assert "active_errors" in m.failed[0]


def test_idle_without_reference_fails():
    """Returned to IDLE but the position is NOT near the reference — a missed
    hardstop / timeout-abort in firmware (no set_absolute_position happened)."""
    m = HomingMonitor([0])
    m.step(0.0, {})
    m.step(0.1, {0: st(CL, pos_rev=2.5)})
    r = m.step(0.3, {0: st(IDLE, pos_rev=2.49)})   # nowhere near |ref|
    assert r.done and m.succeeded == []
    assert "reference not set" in m.failed[0]


def test_timeout_fails():
    m = HomingMonitor([0], timeout_s=2.0)
    m.step(0.0, {})
    m.step(0.1, {0: st(CL, pos_rev=1.0)})
    r = m.step(1.9, {0: st(CL, pos_rev=1.0)})
    assert not r.done
    r = m.step(2.2, {0: st(CL, pos_rev=1.0)})
    assert r.done and "timed out" in m.failed[0]


def test_pos_tolerance_window():
    """Within the tolerance → success; outside → failure."""
    tol = 0.05
    m_ok = HomingMonitor([0], pos_tol_rev=tol)
    m_ok.step(0.0, {})
    m_ok.step(0.1, {0: st(CL, pos_rev=1.0)})
    r = m_ok.step(0.5, {0: st(IDLE, pos_rev=-(REF + 0.5 * tol))})   # inside
    assert r.done and m_ok.succeeded == [0]

    m_bad = HomingMonitor([0], pos_tol_rev=tol)
    m_bad.step(0.0, {})
    m_bad.step(0.1, {0: st(CL, pos_rev=1.0)})
    r = m_bad.step(0.5, {0: st(IDLE, pos_rev=-(REF + 2 * tol))})    # outside
    assert r.done and m_bad.succeeded == [] and 0 in m_bad.failed


# ── multi-axis bookkeeping (the firmware homes one at a time; the monitor still
#    tracks a set independently for the caller that serializes them) ────────────

def test_multi_axis_independent_outcomes():
    m = HomingMonitor([0, 1])
    r = m.step(0.0, {})
    assert sorted(r.set_home) == [0, 1]
    # axis 0 succeeds, axis 1 errors.
    m.step(0.1, {0: st(CL, pos_rev=1.0), 1: st(CL, pos_rev=1.0)})
    r = m.step(0.5, {0: st(IDLE, pos_rev=-REF),
                     1: st(CL, pos_rev=1.0, errors=0x1)})
    assert r.done
    assert m.succeeded == [0]
    assert 1 in m.failed
