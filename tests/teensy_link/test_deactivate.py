"""Unit tests for controller/teensy_link/deactivate.py.

The deactivate completion-observer state machine, tested in isolation (no ROS, no
UDP, no hardware) by driving :meth:`DeactivateMonitor.step` with controlled
monotonic times and per-axis status snapshots.

Ground truth = the firmware DEACTIVATE handler (leg_deactivate.cpp): the
can-bridge seeds the current pos, sets POSITION/TRAP_TRAJ + CLOSED_LOOP, commands
the STOW target, waits for arrival (in CLOSED_LOOP, where position is reliable),
then drops each leg to IDLE. The observer mirrors that: it latches arrival
evidence (``|pos − stow| ≤ tol``) ONLY while CLOSED_LOOP, then accepts the IDLE
as success. This (a) respects the foam-relaxation lesson (never judges from a
post-IDLE position — the 2026-06-26 homing bug) and (b) distinguishes a clean
descent from an abort-to-IDLE (E-STOP / firmware timeout / bus fault all reach
IDLE with no ODrive error, but WITHOUT reaching STOW).
"""

from __future__ import annotations

import pytest

from controller.teensy_link.deactivate import (
    DeactivateMonitor,
    AxisStatus,
    AXIS_STATE_IDLE as IDLE,
    AXIS_STATE_CLOSED_LOOP as CL,
    DEFAULT_ARRIVAL_TOL_REV,
)


def st(state: int = CL, pos_rev: float = 0.0, vel_rps: float = 0.0,
       errors: int = 0) -> AxisStatus:
    return AxisStatus(axis_state=state, pos_rev=pos_rev, vel_rps=vel_rps,
                      active_errors=errors)


def mon(axes=(0,), **kw) -> DeactivateMonitor:
    return DeactivateMonitor(list(axes), **kw)


# ── construction guards ──────────────────────────────────────────────────────

def test_empty_axes_raises():
    with pytest.raises(ValueError):
        DeactivateMonitor([])


def test_duplicate_axes_raises():
    with pytest.raises(ValueError):
        DeactivateMonitor([0, 0])


# ── happy path (descend in CLOSED_LOOP, latch arrival, then IDLE) ─────────────

def test_single_axis_descends_then_idles():
    m = mon([0])
    # Holding the active pose, just started descending — CLOSED_LOOP, not near stow.
    r = m.step(0.0, {0: st(CL, pos_rev=2.19, vel_rps=0.0)})
    assert not r.done
    r = m.step(0.5, {0: st(CL, pos_rev=1.0, vel_rps=-2.0)})   # mid-descent
    assert not r.done
    # Final approach: a CLOSED_LOOP sample lands within the arrival band → latched.
    r = m.step(1.8, {0: st(CL, pos_rev=0.02, vel_rps=-0.3)})
    assert not r.done   # latched, but not IDLE yet
    # Firmware reached STOW + settled + IDLE'd → done.
    r = m.step(2.0, {0: st(IDLE, pos_rev=0.0, vel_rps=0.0)})
    assert r.done and m.succeeded == [0] and m.failed == {}


def test_arrival_latched_in_closed_loop_then_idle_with_foam_drift_is_done():
    """The foam-robust core: arrival is latched WHILE CLOSED_LOOP near stow; the
    subsequent IDLE is accepted even though its resting position has relaxed into
    the foam well away from 0.0 (the post-IDLE drift homing taught us not to gate
    on)."""
    m = mon([0])
    m.step(0.0, {0: st(CL, pos_rev=0.03, vel_rps=-0.2)})   # CLOSED_LOOP, in band → latch
    r = m.step(0.2, {0: st(IDLE, pos_rev=-0.085, vel_rps=0.0)})  # relaxed into foam
    assert r.done and m.succeeded == [0]


def test_at_stow_but_still_closed_loop_is_not_done():
    """In the arrival band but still CLOSED_LOOP (firmware hasn't IDLE'd yet) →
    not done — we wait for the IDLE transition (the firmware's completion)."""
    m = mon([0])
    r = m.step(0.0, {0: st(CL, pos_rev=0.0, vel_rps=0.0)})
    assert not r.done


def test_already_idle_waits_for_reenergize_then_done():
    """Deactivate fired on an already-IDLE leg (precondition violated): the
    firmware re-energises, descends, IDLEs. The observer must NOT instant-succeed
    on the start IDLE (no arrival evidence yet) — it waits for the real cycle."""
    m = mon([0])
    r = m.step(0.0, {0: st(IDLE, pos_rev=-0.08, vel_rps=0.0)})   # IDLE at start
    assert not r.done   # never seen CLOSED_LOOP → keep waiting
    r = m.step(0.3, {0: st(CL, pos_rev=0.01, vel_rps=-0.05)})    # re-energised, near stow
    assert not r.done
    r = m.step(0.5, {0: st(IDLE, pos_rev=-0.06, vel_rps=0.0)})   # IDLE after arrival
    assert r.done and m.succeeded == [0]


# ── failure paths ────────────────────────────────────────────────────────────

def test_active_errors_during_descent_fails():
    m = mon([0])
    m.step(0.0, {0: st(CL, pos_rev=1.0, vel_rps=-2.0)})
    r = m.step(0.2, {0: st(CL, pos_rev=0.9, vel_rps=-2.0, errors=0x40)})
    assert r.done and m.succeeded == [] and "active_errors" in m.failed[0]


def test_idle_without_reaching_stow_is_abort_failure():
    """The WARNING-#1 core: a leg that was DRIVING then dropped to IDLE without
    ever reaching the STOW band (E-STOP / firmware timeout / bus fault — all reach
    IDLE with NO ODrive error) is a FAILURE, not a false success."""
    m = mon([0])
    m.step(0.0, {0: st(CL, pos_rev=2.19, vel_rps=0.0)})
    m.step(0.3, {0: st(CL, pos_rev=1.0, vel_rps=-2.0)})    # driving, mid-descent
    r = m.step(0.4, {0: st(IDLE, pos_rev=-0.10, vel_rps=0.0)})  # abort → IDLE, no errors
    assert r.done and m.succeeded == [] and "before reaching STOW" in m.failed[0]


def test_firmware_timeout_abort_to_idle_is_failure():
    """A stalled descent: the firmware op-timeout aborts the leg to IDLE (no
    error) while it is still far from STOW. Must be FAILED, never success."""
    m = mon([0])
    m.step(0.0, {0: st(CL, pos_rev=1.5, vel_rps=0.0)})     # stuck mid-descent, driving
    m.step(2.0, {0: st(CL, pos_rev=1.5, vel_rps=0.0)})     # still stuck
    r = m.step(5.0, {0: st(IDLE, pos_rev=1.4, vel_rps=0.0)})   # firmware abort → IDLE
    assert r.done and m.succeeded == [] and "before reaching STOW" in m.failed[0]


def test_idle_with_errors_is_failure_not_success():
    """A leg that FAULTED and dropped to IDLE with errors set is a FAILURE — the
    error check precedes the arrival/IDLE checks."""
    m = mon([0])
    r = m.step(0.0, {0: st(IDLE, pos_rev=-0.10, vel_rps=0.0, errors=0x1)})
    assert r.done and m.succeeded == [] and "active_errors" in m.failed[0]


def test_observer_timeout_when_never_idles():
    """Backstop: if the leg never IDLEs (telemetry froze / firmware stuck in
    CLOSED_LOOP past the observer budget), the observer's own timeout fires."""
    m = mon([0], timeout_s=2.0)
    m.step(0.0, {0: st(CL, pos_rev=1.0, vel_rps=-2.0)})
    r = m.step(1.9, {0: st(CL, pos_rev=1.0, vel_rps=-2.0)})   # under timeout
    assert not r.done
    r = m.step(2.2, {0: st(CL, pos_rev=1.0, vel_rps=-2.0)})   # never IDLE'd
    assert r.done and "timed out" in m.failed[0]


def test_missing_status_ages_toward_timeout():
    """No fresh status this tick still ages the axis toward its timeout."""
    m = mon([0], timeout_s=1.0)
    m.step(0.0, {})
    r = m.step(1.5, {})
    assert r.done and 0 in m.failed


# ── arrival-band window ───────────────────────────────────────────────────────

def test_arrival_band_edge():
    tol = DEFAULT_ARRIVAL_TOL_REV
    # A CLOSED_LOOP sample just inside the band latches arrival → IDLE is success.
    m_ok = mon([0])
    m_ok.step(0.0, {0: st(CL, pos_rev=tol - 0.001, vel_rps=-0.1)})
    r = m_ok.step(0.2, {0: st(IDLE, pos_rev=-0.05, vel_rps=0.0)})
    assert r.done and m_ok.succeeded == [0]
    # A CLOSED_LOOP sample just outside the band never latches → IDLE is abort.
    m_bad = mon([0])
    m_bad.step(0.0, {0: st(CL, pos_rev=tol + 0.1, vel_rps=-0.1)})
    r = m_bad.step(0.2, {0: st(IDLE, pos_rev=tol + 0.1, vel_rps=0.0)})
    assert r.done and m_bad.succeeded == [] and 0 in m_bad.failed


# ── multi-axis (parallel even-descent — all legs fire together) ───────────────

def test_multi_axis_independent_outcomes():
    m = mon([0, 1, 2])
    # all descending in CLOSED_LOOP
    m.step(0.0, {a: st(CL, pos_rev=1.0, vel_rps=-2.0) for a in (0, 1, 2)})
    # axis 0 + 2 reach the band in CLOSED_LOOP; axis 1 errors out mid-descent
    m.step(1.5, {0: st(CL, pos_rev=0.02, vel_rps=-0.2),
                 1: st(CL, pos_rev=0.8, vel_rps=-1.0, errors=0x1),
                 2: st(CL, pos_rev=0.01, vel_rps=-0.2)})
    # 0 + 2 IDLE (success); 1 already failed on errors
    r = m.step(2.0, {0: st(IDLE, pos_rev=-0.03, vel_rps=0.0),
                     2: st(IDLE, pos_rev=-0.02, vel_rps=0.0)})
    assert r.done
    assert sorted(m.succeeded) == [0, 2]
    assert 1 in m.failed


def test_multi_axis_all_succeed_only_when_all_terminal():
    m = mon([0, 1])
    m.step(0.0, {0: st(CL, pos_rev=0.0, vel_rps=0.0),
                 1: st(CL, pos_rev=1.0, vel_rps=-2.0)})   # 0 latched, 1 still high
    r = m.step(0.5, {0: st(IDLE, pos_rev=-0.02, vel_rps=0.0),
                     1: st(CL, pos_rev=1.0, vel_rps=-2.0)})
    assert not r.done   # axis 1 still descending
    m.step(1.5, {1: st(CL, pos_rev=0.01, vel_rps=-0.2)})   # 1 latches arrival
    r = m.step(1.7, {1: st(IDLE, pos_rev=-0.02, vel_rps=0.0)})
    assert r.done and sorted(m.succeeded) == [0, 1]
