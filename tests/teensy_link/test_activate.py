"""Unit tests for controller/teensy_link/activate.py (Phase 11 U5).

The activate completion-observer state machine, tested in isolation (no ROS, no
UDP, no hardware) by driving :meth:`ActivateMonitor.step` with controlled
monotonic times and per-axis status snapshots.

Ground truth = the firmware ACTIVATE handler (leg_activate.cpp): the can-bridge
seeds the current pos, sets POSITION/TRAP_TRAJ + CLOSED_LOOP, and commands the
active-pose target; the ODrive runs the trapezoid. The Jetson sees each leg's
``pos_rev`` ramp from the homed hardstop (≈ −0.10) to the active target (≈ 2.19)
and ``vel_rps`` settle to ~0. Both target and reading are Jugglebot convention, so
they compare directly (no magnitude trick, unlike homing).
"""

from __future__ import annotations

import pytest

from controller.teensy_link.activate import (
    ActivateMonitor,
    AxisStatus,
    AXIS_STATE_CLOSED_LOOP as CL,
    DEFAULT_POS_TOL_REV,
    DEFAULT_VEL_TOL_RPS,
)

TGT = 2.19  # representative active-pose target (rev, Jugglebot convention)


def st(pos_rev: float, vel_rps: float = 0.0, state: int = CL,
       errors: int = 0) -> AxisStatus:
    return AxisStatus(axis_state=state, pos_rev=pos_rev, vel_rps=vel_rps,
                      active_errors=errors)


def mon(axes=(0,), target=TGT, **kw) -> ActivateMonitor:
    return ActivateMonitor(list(axes), {a: target for a in axes}, **kw)


# ── construction guards ──────────────────────────────────────────────────────

def test_empty_axes_raises():
    with pytest.raises(ValueError):
        ActivateMonitor([], {})


def test_duplicate_axes_raises():
    with pytest.raises(ValueError):
        ActivateMonitor([0, 0], {0: TGT})


def test_missing_target_raises():
    with pytest.raises(ValueError):
        ActivateMonitor([0, 1], {0: TGT})   # no target for axis 1


# ── happy path ───────────────────────────────────────────────────────────────

def test_single_axis_reaches_target_and_settles():
    m = mon([0])
    # Still at the hardstop, moving up — not done.
    r = m.step(0.0, {0: st(-0.10, vel_rps=0.0)})
    assert not r.done
    r = m.step(0.5, {0: st(1.0, vel_rps=2.0)})   # mid-move, not settled
    assert not r.done
    # Reached target and settled.
    r = m.step(2.0, {0: st(TGT, vel_rps=0.0)})
    assert r.done and m.succeeded == [0] and m.failed == {}


def test_not_settled_at_target_is_not_done():
    """At the target position but still moving (vel above tol) → not done yet."""
    m = mon([0])
    r = m.step(0.0, {0: st(TGT, vel_rps=DEFAULT_VEL_TOL_RPS + 0.5)})
    assert not r.done


def test_settled_but_not_at_target_is_not_done():
    """Settled (vel≈0) but short of the target → not done (e.g. stalled mid-move)."""
    m = mon([0])
    r = m.step(0.0, {0: st(TGT - 0.5, vel_rps=0.0)})
    assert not r.done


def test_already_at_target_immediately_done():
    """Re-activate when already holding the active pose → done immediately."""
    m = mon([0])
    r = m.step(0.0, {0: st(TGT, vel_rps=0.0)})
    assert r.done and m.succeeded == [0]


# ── failure paths ────────────────────────────────────────────────────────────

def test_active_errors_during_activate_fails():
    m = mon([0])
    m.step(0.0, {0: st(0.5, vel_rps=1.0)})
    r = m.step(0.2, {0: st(0.6, vel_rps=1.0, errors=0x40)})
    assert r.done and m.succeeded == [] and "active_errors" in m.failed[0]


def test_timeout_fails():
    m = mon([0], timeout_s=2.0)
    m.step(0.0, {0: st(-0.10, vel_rps=0.0)})
    r = m.step(1.9, {0: st(0.5, vel_rps=2.0)})    # still moving, under timeout
    assert not r.done
    r = m.step(2.2, {0: st(0.5, vel_rps=2.0)})    # never reached target
    assert r.done and "timed out" in m.failed[0]


def test_missing_status_ages_toward_timeout():
    """No fresh status this tick still ages the axis toward its timeout."""
    m = mon([0], timeout_s=1.0)
    m.step(0.0, {})
    r = m.step(1.5, {})
    assert r.done and 0 in m.failed


# ── tolerance windows ────────────────────────────────────────────────────────

def test_pos_tolerance_window():
    tol = DEFAULT_POS_TOL_REV
    m_ok = mon([0])
    r = m_ok.step(0.0, {0: st(TGT - 0.5 * tol, vel_rps=0.0)})   # inside
    assert r.done and m_ok.succeeded == [0]

    m_bad = mon([0], timeout_s=1.0)
    m_bad.step(0.0, {0: st(TGT - 2 * tol, vel_rps=0.0)})        # outside
    r = m_bad.step(1.5, {0: st(TGT - 2 * tol, vel_rps=0.0)})
    assert r.done and m_bad.succeeded == [] and 0 in m_bad.failed


def test_vel_tolerance_window():
    m_ok = mon([0])
    r = m_ok.step(0.0, {0: st(TGT, vel_rps=DEFAULT_VEL_TOL_RPS)})   # exactly at tol → settled
    assert r.done and m_ok.succeeded == [0]


# ── multi-axis (parallel even-rise — all legs fire together) ──────────────────

def test_multi_axis_independent_outcomes():
    m = mon([0, 1, 2])
    # all moving up
    m.step(0.0, {a: st(0.5, vel_rps=2.0) for a in (0, 1, 2)})
    # axis 0 + 2 reach + settle; axis 1 errors out
    r = m.step(2.0, {0: st(TGT, vel_rps=0.0),
                     1: st(1.0, vel_rps=1.0, errors=0x1),
                     2: st(TGT, vel_rps=0.0)})
    assert r.done
    assert sorted(m.succeeded) == [0, 2]
    assert 1 in m.failed


def test_multi_axis_all_succeed_only_when_all_terminal():
    m = mon([0, 1])
    r = m.step(0.0, {0: st(TGT, vel_rps=0.0), 1: st(1.0, vel_rps=2.0)})
    assert not r.done   # axis 1 still moving
    r = m.step(1.0, {0: st(TGT, vel_rps=0.0), 1: st(TGT, vel_rps=0.0)})
    assert r.done and sorted(m.succeeded) == [0, 1]
