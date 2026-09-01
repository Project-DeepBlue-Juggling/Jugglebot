"""Unit tests for controller.ballistics — pure-math helpers for the toss
motion path (formerly reached via ``run_mpc.py --toss-motion``, removed
2026-09-01 with the MPC chain)."""

from __future__ import annotations

import numpy as np
import pytest

from controller.ballistics import (
    GRAVITY_MMS2,
    TILT_LIMIT_RAD,
    compute_arrival_velocity,
    compute_launch_velocity,
    compute_orientation,
    flight_time_from_apex,
    rodrigues,
)


# ---------------------------------------------------------------------------
# flight_time_from_apex
# ---------------------------------------------------------------------------

def test_flight_time_symmetric_throw_matches_textbook():
    """Same-height throw with 1.2m apex → t = 2·√(2h/g)."""
    p0 = np.array([0.0, 0.0, 170.0])
    p1 = np.array([100.0, 0.0, 170.0])
    t = flight_time_from_apex(p0, p1, 1200.0)
    expected = 2.0 * np.sqrt(2.0 * 1200.0 / GRAVITY_MMS2)
    assert t == pytest.approx(expected, rel=1e-9)


def test_flight_time_catch_higher_than_throw():
    """Δz > 0: ball lands higher than it was thrown → shorter flight time."""
    p0 = np.array([0.0, 0.0, 170.0])
    p1 = np.array([100.0, 0.0, 270.0])  # +100mm above throw
    t = flight_time_from_apex(p0, p1, 1200.0)
    # Must be shorter than the symmetric case.
    t_sym = 2.0 * np.sqrt(2.0 * 1200.0 / GRAVITY_MMS2)
    assert 0.0 < t < t_sym


def test_flight_time_catch_lower_than_throw():
    """Δz < 0: ball lands below throw → longer flight time."""
    p0 = np.array([0.0, 0.0, 270.0])
    p1 = np.array([100.0, 0.0, 170.0])
    t = flight_time_from_apex(p0, p1, 1200.0)
    t_sym = 2.0 * np.sqrt(2.0 * 1200.0 / GRAVITY_MMS2)
    assert t > t_sym


def test_flight_time_catch_at_apex_is_single_root():
    """Catch exactly at apex height: discriminant is zero, t = v_z/g."""
    p0 = np.array([0.0, 0.0, 0.0])
    p1 = np.array([50.0, 0.0, 1200.0])  # at exactly the apex height above throw
    t = flight_time_from_apex(p0, p1, 1200.0)
    v_z = np.sqrt(2.0 * GRAVITY_MMS2 * 1200.0)
    assert t == pytest.approx(v_z / GRAVITY_MMS2, rel=1e-9)


def test_flight_time_raises_when_catch_above_apex():
    p0 = np.array([0.0, 0.0, 170.0])
    p1 = np.array([0.0, 0.0, 2000.0])  # Δz=1830, apex only 1200
    with pytest.raises(ValueError, match="catch above apex"):
        flight_time_from_apex(p0, p1, 1200.0)


def test_flight_time_raises_on_nonpositive_apex():
    with pytest.raises(ValueError, match="apex_height_mm"):
        flight_time_from_apex(
            np.zeros(3), np.array([0.0, 0.0, 10.0]), 0.0)
    with pytest.raises(ValueError, match="apex_height_mm"):
        flight_time_from_apex(
            np.zeros(3), np.array([0.0, 0.0, 10.0]), -1.0)


# ---------------------------------------------------------------------------
# Consistency: flight_time → launch → arrival round-trip
# ---------------------------------------------------------------------------

def test_launch_arrival_consistency_symmetric():
    p0 = np.array([0.0, 0.0, 170.0])
    p1 = np.array([200.0, 0.0, 170.0])
    apex = 1200.0
    t = flight_time_from_apex(p0, p1, apex)

    v_launch = compute_launch_velocity(p0, p1, t)
    v_arrival = compute_arrival_velocity(v_launch, t)

    # Vertical launch velocity must equal √(2·g·apex) for a symmetric throw
    # that reaches exactly ``apex`` above the throw.
    assert v_launch[2] == pytest.approx(
        np.sqrt(2.0 * GRAVITY_MMS2 * apex), rel=1e-9)
    # Symmetric: arrival z-velocity is the negated launch z-velocity.
    assert v_arrival[2] == pytest.approx(-v_launch[2], rel=1e-9)
    # Horizontal: unchanged throughout flight.
    assert v_arrival[0] == pytest.approx(v_launch[0], rel=1e-9)
    assert v_arrival[1] == pytest.approx(v_launch[1], rel=1e-9)


def test_launch_arrival_consistency_non_planar():
    """Diagonal XY throw with Δz ≠ 0 — verifies the full parabolic solve."""
    p0 = np.array([10.0, -20.0, 180.0])
    p1 = np.array([130.0, 80.0, 220.0])
    apex = 1000.0
    t = flight_time_from_apex(p0, p1, apex)

    v_launch = compute_launch_velocity(p0, p1, t)
    # Integrate the ball forward in time and check it lands at p1.
    landing = p0 + v_launch * t + np.array(
        [0.0, 0.0, -0.5 * GRAVITY_MMS2 * t * t])
    np.testing.assert_allclose(landing, p1, rtol=1e-9, atol=1e-6)

    # Apex height above throw should equal v_z²/(2g).
    apex_above_throw = v_launch[2] ** 2 / (2.0 * GRAVITY_MMS2)
    assert apex_above_throw == pytest.approx(apex, rel=1e-9)


# ---------------------------------------------------------------------------
# compute_orientation
# ---------------------------------------------------------------------------

def test_orientation_pure_up_is_zero_rotvec():
    rv = compute_orientation(np.array([0.0, 0.0, 1.0]))
    np.testing.assert_allclose(rv, np.zeros(3), atol=1e-12)


def test_orientation_forward_tilt_rotates_about_y():
    """A velocity tilted +X from vertical produces a rotation about +Y
    (right-hand rule: +Z × +X = +Y)."""
    angle = np.radians(20.0)  # under the 30° workspace limit
    v = np.array([np.sin(angle), 0.0, np.cos(angle)])
    rv = compute_orientation(v)
    # Only the y-axis component should be non-zero.
    assert abs(rv[0]) < 1e-12
    assert abs(rv[2]) < 1e-12
    assert rv[1] > 0  # positive rotation about +Y tilts platform +X forward
    assert np.linalg.norm(rv) == pytest.approx(angle, rel=1e-9)


def test_orientation_raises_above_tilt_limit():
    """Direction more than 30° from +Z must raise."""
    # 31° tilt in the XZ plane.
    angle = np.radians(31.0)
    v = np.array([np.sin(angle), 0.0, np.cos(angle)])
    with pytest.raises(ValueError, match="Tilt angle"):
        compute_orientation(v)


def test_orientation_at_tilt_limit_does_not_raise():
    """Exactly at the limit is permitted (strict >)."""
    angle = TILT_LIMIT_RAD - 1e-6  # just under
    v = np.array([np.sin(angle), 0.0, np.cos(angle)])
    rv = compute_orientation(v)
    assert np.linalg.norm(rv) == pytest.approx(angle, rel=1e-6)


def test_orientation_rotates_z_axis_to_target():
    """Applying the resulting rotation to +Z should recover target direction."""
    v = np.array([0.3, 0.2, 1.0])
    v_hat = v / np.linalg.norm(v)
    rv = compute_orientation(v)
    R = rodrigues(rv)
    transformed_z = R @ np.array([0.0, 0.0, 1.0])
    np.testing.assert_allclose(transformed_z, v_hat, atol=1e-10)


def test_orientation_rejects_zero_vector():
    with pytest.raises(ValueError, match="Zero-length"):
        compute_orientation(np.zeros(3))


# ---------------------------------------------------------------------------
# rodrigues
# ---------------------------------------------------------------------------

def test_rodrigues_identity_for_zero_rotvec():
    np.testing.assert_allclose(rodrigues(np.zeros(3)), np.eye(3), atol=1e-15)


def test_rodrigues_quarter_turn_about_z():
    rv = np.array([0.0, 0.0, np.pi / 2])
    R = rodrigues(rv)
    expected = np.array([
        [0.0, -1.0, 0.0],
        [1.0,  0.0, 0.0],
        [0.0,  0.0, 1.0],
    ])
    np.testing.assert_allclose(R, expected, atol=1e-14)


# ---------------------------------------------------------------------------
# Re-export compatibility
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# TossMotionSource state machine
# ---------------------------------------------------------------------------

from types import SimpleNamespace
from controller.toss_motion_source import TossMotionSource


def _make_state(pos_mm, rot=None, twist=None, t=0.0):
    """Minimal PlantState stand-in — TossMotionSource only reads a few fields."""
    return SimpleNamespace(
        platform_pos_mm=np.asarray(pos_mm, dtype=float),
        platform_rot=np.asarray(
            rot if rot is not None else [0.0, 0.0, 0.0], dtype=float),
        platform_twist=np.asarray(
            twist if twist is not None else [0.0] * 6, dtype=float),
        time=t,
    )


def test_toss_first_update_captures_initial_pose_and_anchors_ref_events():
    """update() at sim_time=t0 must return ref_events anchored at t0."""
    src = TossMotionSource(
        catch_positions_mm=[np.array([0.0, 100.0, 170.0])],
        apex_height_mm=1200.0,
        hold_s=0.5,
        v_max_mmps=140.0, tau_s=0.05,
        clamp_start_twist_mmps=60.0,
    )
    state = _make_state([0.0, 0.0, 170.0])
    tc = src.update(sim_time=3.25, state=state)
    assert src.phase == 'approach'
    assert tc.ref_events[0].time == pytest.approx(3.25)


def test_toss_none_sentinel_resolves_to_initial_pose():
    src = TossMotionSource(
        catch_positions_mm=[np.array([0.0, 100.0, 170.0]), None],
        apex_height_mm=1200.0, hold_s=0.01,
        v_max_mmps=140.0, tau_s=0.05, clamp_start_twist_mmps=60.0,
    )
    initial = np.array([0.0, 0.0, 170.0])
    src.update(0.0, _make_state(initial))
    # Force-advance to cycle 1 by mutating internals and rebuilding the cycle.
    src._cycle_idx = 1
    src._begin_cycle(10.0, _make_state([0.0, 100.0, 170.0]))
    np.testing.assert_allclose(src._catch_pose_6dof[:3], initial)


def test_toss_approach_does_not_advance_on_pure_angular_twist():
    """Rotation-matched + zero linear vel but live angular vel must NOT
    advance to TRANSIT.  Settled angular vel → advance is still allowed."""
    catch = np.array([100.0, 0.0, 170.0])
    src = TossMotionSource(
        catch_positions_mm=[catch], apex_height_mm=1200.0, hold_s=0.1,
        v_max_mmps=140.0, tau_s=0.05, clamp_start_twist_mmps=60.0,
    )
    src.update(0.0, _make_state([0.0, 0.0, 170.0]))
    # Pose-matched at throw but live angular velocity — must stay in APPROACH.
    at_throw_spinning = _make_state(
        src._throw_pose_6dof[:3],
        rot=src._throw_pose_6dof[3:],
        twist=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        t=0.025,
    )
    src.update(0.025, at_throw_spinning)
    assert src.phase == 'approach'
    # Angular settled — now advance.
    at_throw_settled = _make_state(
        src._throw_pose_6dof[:3],
        rot=src._throw_pose_6dof[3:],
        twist=[0.0] * 6,
        t=0.050,
    )
    src.update(0.050, at_throw_settled)
    assert src.phase == 'transit'


def test_toss_cycle_progresses_approach_transit_hold_done():
    """Full cycle smoke test: APPROACH → TRANSIT → HOLD → DONE."""
    src = TossMotionSource(
        catch_positions_mm=[np.array([100.0, 0.0, 170.0])],
        apex_height_mm=1200.0, hold_s=0.1,
        v_max_mmps=140.0, tau_s=0.05, clamp_start_twist_mmps=60.0,
    )
    src.update(0.0, _make_state([0.0, 0.0, 170.0]))
    assert src.phase == 'approach'
    at_throw = _make_state(
        src._throw_pose_6dof[:3], rot=src._throw_pose_6dof[3:],
        twist=[0.0] * 6, t=0.025)
    src.update(0.025, at_throw)
    assert src.phase == 'transit'
    flight = src._flight_time_s
    catch_pose = src._catch_pose_6dof
    at_catch = _make_state(catch_pose[:3], rot=catch_pose[3:],
                           twist=[0.0] * 6, t=0.025 + flight + 1e-3)
    src.update(0.025 + flight + 1e-3, at_catch)
    assert src.phase == 'hold'
    src.update(0.025 + flight + 0.2, at_catch)
    assert src.done


def test_toss_phase_transition_invalidates_cache():
    """TRANSIT (arrival_time=deadline) → HOLD (arrival_time=None) at the
    same catch pose must refresh the ref_events cache — otherwise the MPC
    keeps tracking a stale deadline after the transit is complete."""
    src = TossMotionSource(
        catch_positions_mm=[np.array([100.0, 0.0, 170.0])],
        apex_height_mm=1200.0, hold_s=0.5,
        v_max_mmps=140.0, tau_s=0.05, clamp_start_twist_mmps=60.0,
    )
    src.update(0.0, _make_state([0.0, 0.0, 170.0]))
    at_throw = _make_state(
        src._throw_pose_6dof[:3], rot=src._throw_pose_6dof[3:],
        twist=[0.0] * 6, t=0.025)
    tc_transit = src.update(0.025, at_throw)
    assert tc_transit.arrival_time is not None
    flight = src._flight_time_s
    catch_pose = src._catch_pose_6dof
    at_catch = _make_state(catch_pose[:3], rot=catch_pose[3:],
                           twist=[0.0] * 6, t=0.025 + flight + 1e-3)
    tc_hold = src.update(0.025 + flight + 1e-3, at_catch)
    assert src.phase == 'hold'
    assert tc_hold.arrival_time is None
    # Cache should have been rebuilt — ref_events is a new list.
    assert tc_hold.ref_events is not tc_transit.ref_events


def test_sim_hand_ballistics_reexports_match_controller():
    """sim/hand/ballistics.py must re-export the same symbols with identical
    numerics — this was the compat contract when we extracted the pure-math
    bits to controller/."""
    from sim.hand import ballistics as sim_ballistics
    from controller import ballistics as ctl_ballistics

    assert sim_ballistics.compute_launch_velocity is (
        ctl_ballistics.compute_launch_velocity)
    assert sim_ballistics.compute_arrival_velocity is (
        ctl_ballistics.compute_arrival_velocity)
    assert sim_ballistics.compute_orientation is (
        ctl_ballistics.compute_orientation)
    assert sim_ballistics.rodrigues is ctl_ballistics.rodrigues
    # Legacy underscored names preserved.
    assert sim_ballistics._GRAVITY_MMS2 == ctl_ballistics.GRAVITY_MMS2
    assert sim_ballistics._TILT_LIMIT_RAD == ctl_ballistics.TILT_LIMIT_RAD
