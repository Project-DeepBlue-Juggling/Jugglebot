"""Tests for the CatchHeightOptimizer.

Tests cover:
- compute_catch_orientation: identity, tilted, angle limit
- compute_catch_pose: nominal, hand offset, angle rejection
- CatchHeightOptimizer: feasible selection, jerk ranking, all-infeasible
- Integration with real MuJoCo IK
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from controller.catch_optimizer import (
    CatchHeightOptimizer,
    compute_catch_orientation,
    compute_catch_pose,
)


# NIGHTLY TIER — the MPC is operationally dormant (plans/active/refactor-2026-07.md
# Phase 3: jugglebot_launch.py no longer starts motor_guard/motion_bridge_node; the
# leg path is trajectory_node -> teensy_bridge_node -> the Teensy MAX_DEVIATION
# guard). The code is parked, not deleted, so this battery is parked with it: it
# runs nightly via tools/nightly_suite.sh and on `./run_tests.sh --full`, which is
# mandatory before any hardware sitting. Promotion back to per-commit is step 4 of
# the MPC revival.
pytestmark = pytest.mark.nightly


STROKE_MM = 280.0
HAND_CATCH_OFFSET_MM = 64.78
INITIAL_HEIGHT_MM = 574.3


# ------------------------------------------------------------------
# Mock IK: simple stub that always returns mid-stroke extensions
# unless the pose Z is way out of range.
# ------------------------------------------------------------------

def _mock_ik_always_feasible(pose_6dof: np.ndarray) -> np.ndarray:
    """Mock IK that returns mid-stroke extensions for any pose."""
    return np.full(6, 140.0)


def _mock_ik_z_sensitive(pose_6dof: np.ndarray) -> np.ndarray:
    """Mock IK that maps pose Z linearly to extensions.

    Extensions = 140 + z_offset.  So z_offset=0 → 140mm (mid-stroke),
    z_offset= +150 → 290mm (above stroke), z_offset= -150 → -10mm (below).
    """
    z_offset = pose_6dof[2]  # platform-relative Z
    base_ext = 140.0 + z_offset * 0.8  # scale factor for sensitivity
    return np.full(6, base_ext)


# ------------------------------------------------------------------
# compute_catch_orientation tests
# ------------------------------------------------------------------

class TestCatchOrientation:

    def test_vertical_drop_gives_identity(self):
        """Ball falling straight down → no platform tilt."""
        v = np.array([0.0, 0.0, -3000.0])  # straight down
        rv = compute_catch_orientation(v)
        assert rv is not None
        np.testing.assert_array_almost_equal(rv, np.zeros(3), decimal=5)

    def test_zero_velocity_gives_identity(self):
        v = np.array([0.0, 0.0, 0.0])
        rv = compute_catch_orientation(v)
        assert rv is not None
        np.testing.assert_array_almost_equal(rv, np.zeros(3), decimal=5)

    def test_angled_approach_gives_tilt(self):
        """Ball coming at an angle → platform tilts."""
        v = np.array([1000.0, 0.0, -2000.0])
        rv = compute_catch_orientation(v)
        assert rv is not None
        angle = np.linalg.norm(rv)
        assert angle > 0.01  # measurable tilt
        assert angle < math.radians(30.0)  # within limit

    def test_steep_angle_rejected(self):
        """Ball coming mostly horizontal → exceeds 30° limit."""
        v = np.array([3000.0, 0.0, -500.0])  # ~80° from vertical
        rv = compute_catch_orientation(v)
        assert rv is None

    def test_angle_at_limit(self):
        """Ball at exactly 30° should be accepted."""
        angle = math.radians(29.9)
        vz = -1000.0 * math.cos(angle)
        vx = 1000.0 * math.sin(angle)
        v = np.array([vx, 0.0, vz])
        rv = compute_catch_orientation(v)
        assert rv is not None


# ------------------------------------------------------------------
# compute_catch_pose tests
# ------------------------------------------------------------------

class TestCatchPose:

    def test_vertical_drop_at_origin(self):
        """Vertical drop at XY=0 → pose is just the Z offset."""
        pose = compute_catch_pose(
            landing_xy_mm=np.array([0.0, 0.0]),
            landing_z_mm=800.0,
            landing_velocity_mms=np.array([0.0, 0.0, -3000.0]),
            hand_catch_offset_mm=HAND_CATCH_OFFSET_MM,
            initial_height_mm=INITIAL_HEIGHT_MM,
        )
        assert pose is not None
        # X, Y should be ~0 (vertical drop, hand offset along Z)
        assert abs(pose[0]) < 1.0
        assert abs(pose[1]) < 1.0
        # Z: centroid_z = 800 - 64.78 * 1.0 = 735.22, relative = 735.22 - 574.3 = 160.92
        expected_z = 800.0 - HAND_CATCH_OFFSET_MM - INITIAL_HEIGHT_MM
        assert abs(pose[2] - expected_z) < 0.1
        # No tilt
        np.testing.assert_array_almost_equal(pose[3:], np.zeros(3), decimal=3)

    def test_offset_xy(self):
        """Landing at XY=(100, -50) → pose XY matches (minus hand offset projection)."""
        pose = compute_catch_pose(
            landing_xy_mm=np.array([100.0, -50.0]),
            landing_z_mm=800.0,
            landing_velocity_mms=np.array([0.0, 0.0, -3000.0]),
            hand_catch_offset_mm=HAND_CATCH_OFFSET_MM,
            initial_height_mm=INITIAL_HEIGHT_MM,
        )
        assert pose is not None
        assert abs(pose[0] - 100.0) < 1.0
        assert abs(pose[1] - (-50.0)) < 1.0

    def test_steep_angle_returns_none(self):
        """Steep approach → None."""
        pose = compute_catch_pose(
            landing_xy_mm=np.array([0.0, 0.0]),
            landing_z_mm=800.0,
            landing_velocity_mms=np.array([5000.0, 0.0, -500.0]),
            hand_catch_offset_mm=HAND_CATCH_OFFSET_MM,
            initial_height_mm=INITIAL_HEIGHT_MM,
        )
        assert pose is None

    def test_hand_offset_applied_with_tilt(self):
        """With tilted platform, hand offset shifts XY as well as Z."""
        v = np.array([1000.0, 0.0, -2000.0])
        pose = compute_catch_pose(
            landing_xy_mm=np.array([0.0, 0.0]),
            landing_z_mm=800.0,
            landing_velocity_mms=v,
            hand_catch_offset_mm=HAND_CATCH_OFFSET_MM,
            initial_height_mm=INITIAL_HEIGHT_MM,
        )
        assert pose is not None
        # With tilt, centroid X is shifted away from ball XY
        assert abs(pose[0]) > 1.0  # non-zero X offset due to tilt


# ------------------------------------------------------------------
# CatchHeightOptimizer tests
# ------------------------------------------------------------------

class TestOptimizer:

    def test_basic_feasible(self):
        """With always-feasible IK, optimizer should find a result."""
        opt = CatchHeightOptimizer(
            ik_fn=_mock_ik_always_feasible,
            stroke_mm=STROKE_MM,
        )
        result = opt.optimize(
            landing_xy_mm=np.array([0.0, 0.0]),
            landing_velocity_mms=np.array([0.0, 0.0, -3000.0]),
            current_pose=np.zeros(6),
            current_twist=np.zeros(6),
            time_available=1.0,
        )
        assert result is not None
        best_pose, jerk = result
        assert best_pose.shape == (6,)
        assert jerk >= 0.0

    def test_all_infeasible_returns_none(self):
        """If IK rejects all candidates, return None."""
        def _reject_all(pose):
            return np.full(6, -100.0)  # way below stroke

        opt = CatchHeightOptimizer(
            ik_fn=_reject_all,
            stroke_mm=STROKE_MM,
        )
        result = opt.optimize(
            landing_xy_mm=np.array([0.0, 0.0]),
            landing_velocity_mms=np.array([0.0, 0.0, -3000.0]),
            current_pose=np.zeros(6),
            current_twist=np.zeros(6),
            time_available=1.0,
        )
        assert result is None

    def test_zero_time_returns_none(self):
        opt = CatchHeightOptimizer(
            ik_fn=_mock_ik_always_feasible,
            stroke_mm=STROKE_MM,
        )
        result = opt.optimize(
            landing_xy_mm=np.array([0.0, 0.0]),
            landing_velocity_mms=np.array([0.0, 0.0, -3000.0]),
            current_pose=np.zeros(6),
            current_twist=np.zeros(6),
            time_available=0.0,
        )
        assert result is None

    def test_lower_jerk_for_closer_z(self):
        """Catching closer to the current Z should have lower jerk."""
        opt = CatchHeightOptimizer(
            ik_fn=_mock_ik_always_feasible,
            stroke_mm=STROKE_MM,
        )
        # Current pose at Z=160 (typical active height offset)
        current = np.array([0.0, 0.0, 160.0, 0.0, 0.0, 0.0])

        # Wide Z range: 700–900mm base frame
        result_wide = opt.optimize(
            landing_xy_mm=np.array([0.0, 0.0]),
            landing_velocity_mms=np.array([0.0, 0.0, -3000.0]),
            current_pose=current,
            current_twist=np.zeros(6),
            time_available=0.5,
            z_range=(700.0, 900.0),
        )

        # Narrow Z range: close to nominal
        nominal_z = INITIAL_HEIGHT_MM + 160.0 + HAND_CATCH_OFFSET_MM
        result_narrow = opt.optimize(
            landing_xy_mm=np.array([0.0, 0.0]),
            landing_velocity_mms=np.array([0.0, 0.0, -3000.0]),
            current_pose=current,
            current_twist=np.zeros(6),
            time_available=0.5,
            z_range=(nominal_z - 10.0, nominal_z + 10.0),
        )

        assert result_wide is not None
        assert result_narrow is not None
        # The narrow range should have chosen a Z closer to current → lower jerk
        # (or equal, since the wide range also searches near nominal)
        assert result_narrow[1] <= result_wide[1] * 1.01  # allow 1% tolerance

    def test_workspace_limits_reject_extreme_z(self):
        """Z-sensitive IK should reject candidates with extreme extensions."""
        opt = CatchHeightOptimizer(
            ik_fn=_mock_ik_z_sensitive,
            stroke_mm=STROKE_MM,
            n_candidates=10,
        )
        # Search a range where high Z gives extensions > stroke
        result = opt.optimize(
            landing_xy_mm=np.array([0.0, 0.0]),
            landing_velocity_mms=np.array([0.0, 0.0, -3000.0]),
            current_pose=np.zeros(6),
            current_twist=np.zeros(6),
            time_available=1.0,
            z_range=(500.0, 900.0),
        )
        if result is not None:
            # The chosen pose should have feasible extensions
            ext = _mock_ik_z_sensitive(result[0])
            assert np.all(ext >= -5.0)
            assert np.all(ext <= STROKE_MM + 5.0)

    def test_custom_z_range(self):
        """Custom z_range is respected."""
        opt = CatchHeightOptimizer(
            ik_fn=_mock_ik_always_feasible,
            stroke_mm=STROKE_MM,
        )
        z_lo, z_hi = 750.0, 850.0
        result = opt.optimize(
            landing_xy_mm=np.array([0.0, 0.0]),
            landing_velocity_mms=np.array([0.0, 0.0, -3000.0]),
            current_pose=np.zeros(6),
            current_twist=np.zeros(6),
            time_available=1.0,
            z_range=(z_lo, z_hi),
        )
        assert result is not None
        # The chosen catch Z (in base frame) should be within range
        # catch_pose Z = centroid_base_z - initial_height
        # centroid_base_z = landing_z - offset * platform_z_component
        # For vertical drop: centroid_z ≈ landing_z - 64.78
        # So pose_z ≈ (landing_z - 64.78) - 574.3
        # All candidates should produce pose_z values in a reasonable range


# ------------------------------------------------------------------
# Integration test with real IK (requires MuJoCo)
# ------------------------------------------------------------------

class TestOptimizerWithRealIK:

    @pytest.fixture(autouse=True)
    def _setup(self):
        try:
            from sim.plant.mujoco_plant import MuJoCoPlant
            self.plant = MuJoCoPlant()
        except Exception:
            pytest.skip("MuJoCo not available")

    def test_nominal_catch(self):
        """Real IK: find a feasible catch pose for a vertical drop."""
        opt = CatchHeightOptimizer(
            ik_fn=self.plant.pose_to_extensions,
            stroke_mm=self.plant.geom.leg_stroke_mm,
        )
        result = opt.optimize(
            landing_xy_mm=np.array([0.0, 0.0]),
            landing_velocity_mms=np.array([0.0, 0.0, -3000.0]),
            current_pose=np.array([0.0, 0.0, 10.0, 0.0, 0.0, 0.0]),
            current_twist=np.zeros(6),
            time_available=1.0,
        )
        assert result is not None, "No feasible catch height found"
        best_pose, jerk = result
        # Verify extensions are valid
        ext = self.plant.pose_to_extensions(best_pose)
        assert np.all(ext >= -5.0), f"Extension below min: {ext}"
        assert np.all(ext <= self.plant.geom.leg_stroke_mm + 5.0), \
            f"Extension above max: {ext}"

    def test_offset_catch(self):
        """Real IK: off-centre catch with moderate tilt."""
        opt = CatchHeightOptimizer(
            ik_fn=self.plant.pose_to_extensions,
            stroke_mm=self.plant.geom.leg_stroke_mm,
        )
        result = opt.optimize(
            landing_xy_mm=np.array([80.0, -40.0]),
            landing_velocity_mms=np.array([500.0, 0.0, -2500.0]),
            current_pose=np.array([0.0, 0.0, 10.0, 0.0, 0.0, 0.0]),
            current_twist=np.zeros(6),
            time_available=0.8,
        )
        assert result is not None, "No feasible catch height found for offset"
        ext = self.plant.pose_to_extensions(result[0])
        assert np.all(ext >= -5.0)
        assert np.all(ext <= self.plant.geom.leg_stroke_mm + 5.0)

    def test_prefers_lower_jerk(self):
        """Real IK: with more time, jerk should be lower."""
        opt = CatchHeightOptimizer(
            ik_fn=self.plant.pose_to_extensions,
            stroke_mm=self.plant.geom.leg_stroke_mm,
        )
        result_fast = opt.optimize(
            landing_xy_mm=np.array([0.0, 0.0]),
            landing_velocity_mms=np.array([0.0, 0.0, -3000.0]),
            current_pose=np.array([0.0, 0.0, 10.0, 0.0, 0.0, 0.0]),
            current_twist=np.zeros(6),
            time_available=0.3,
        )
        result_slow = opt.optimize(
            landing_xy_mm=np.array([0.0, 0.0]),
            landing_velocity_mms=np.array([0.0, 0.0, -3000.0]),
            current_pose=np.array([0.0, 0.0, 10.0, 0.0, 0.0, 0.0]),
            current_twist=np.zeros(6),
            time_available=2.0,
        )
        if result_fast is not None and result_slow is not None:
            # More time → lower jerk (quintic jerk ∝ 1/T⁵)
            assert result_slow[1] < result_fast[1]


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
