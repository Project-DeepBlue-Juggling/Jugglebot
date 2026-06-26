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


def test_idle_after_closed_loop_is_success_regardless_of_pos():
    """Foam-stop fix (2026-06-26): trust the firmware's current trip. A leg that
    drove (CLOSED_LOOP) and returned to IDLE without errors is homed — whatever
    the foam-relaxed telemetry position. Positions the OLD |pos|≈ref gate rejected
    must now succeed."""
    for pos in (-0.10, -0.05, 0.0, -0.20, 2.49):
        m = HomingMonitor([0])
        m.step(0.0, {})                          # send HOME
        m.step(0.1, {0: st(CL, pos_rev=1.0)})    # drove
        r = m.step(0.5, {0: st(IDLE, pos_rev=pos)})
        assert r.done and m.succeeded == [0], f"pos={pos} should succeed"
        assert m.failed == {}


def test_timeout_fails():
    m = HomingMonitor([0], timeout_s=2.0)
    m.step(0.0, {})
    m.step(0.1, {0: st(CL, pos_rev=1.0)})
    r = m.step(1.9, {0: st(CL, pos_rev=1.0)})
    assert not r.done
    r = m.step(2.2, {0: st(CL, pos_rev=1.0)})
    assert r.done and "timed out" in m.failed[0]


def test_stuck_in_closed_loop_times_out():
    """With the position gate removed, the timeout is what catches a leg that
    never trips — it stays in CLOSED_LOOP and is failed (rather than producing a
    success-looking IDLE). The observer timeout is set below the firmware's own
    homing timeout precisely so this is caught first."""
    m = HomingMonitor([0], timeout_s=2.0)
    m.step(0.0, {})
    r = m.step(1.0, {0: st(CL, pos_rev=0.5)})    # still driving
    assert not r.done
    r = m.step(2.5, {0: st(CL, pos_rev=0.5)})    # still CLOSED_LOOP past timeout
    assert r.done and m.succeeded == [] and "timed out" in m.failed[0]


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
