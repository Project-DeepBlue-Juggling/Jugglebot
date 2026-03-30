"""Tests for BallButlerSim — ballistic throw source."""

from __future__ import annotations

import math

import numpy as np
import pytest

from ball_butler.sim import (
    BallButlerSim, BallButlerConfig,
    _yaw_solve, _solve_throw, _GRAVITY_MMS2,
)
from hand.coordinator import BallSpawn
from input.scripted import _ball_landing, _compute_catch_target


# ---------------------------------------------------------------------------
# Default BB for tests — above and beside Jugglebot
# ---------------------------------------------------------------------------

# BB is mounted on a shelf above and beside Jugglebot.
# ~500mm horizontal, ~700mm above catch height.  Position is approximate
# until measured on the real setup; user will provide the exact value.
_BB_POS = np.array([300.0, -400.0, 1500.0])
# Yaw offset points BB's local +X toward Jugglebot origin
_BB_YAW_OFFSET = math.atan2(
    0.0 - _BB_POS[1],   # JB_y - BB_y
    0.0 - _BB_POS[0],   # JB_x - BB_x
)

# Jugglebot catch height (platform_height + active_z + hand_catch_offset)
_CATCH_Z = 783.5


def _make_bb(**overrides) -> BallButlerSim:
    """Helper to create a BB with default or overridden position."""
    pos = overrides.pop('position_mm', _BB_POS.copy())
    yaw = overrides.pop('yaw_offset_rad', _BB_YAW_OFFSET)
    return BallButlerSim.from_hardware_config(pos, yaw)


# ===================================================================
# Test: Config loading
# ===================================================================

class TestConfig:
    def test_from_hardware_config(self):
        bb = _make_bb()
        cfg = bb.config
        # Verify key geometry values from hardware_config.yaml
        assert cfg.yaw_s_offset_mm == pytest.approx(-105.65)
        assert cfg.pitch_d_offset_mm == pytest.approx(41.0)
        assert cfg.release_l_position_mm == pytest.approx(150.0)
        assert cfg.pitch_z_offset_mm == pytest.approx(17.5)
        assert cfg.pitch_min_deg == pytest.approx(12.0)
        assert cfg.pitch_max_deg == pytest.approx(85.0)
        assert cfg.max_throw_speed_mps == pytest.approx(5.0)
        np.testing.assert_array_equal(cfg.position_mm, _BB_POS)

    def test_position_stored(self):
        pos = np.array([100.0, 200.0, 1200.0])
        bb = _make_bb(position_mm=pos)
        np.testing.assert_array_almost_equal(bb.config.position_mm, pos)


# ===================================================================
# Test: Yaw solver
# ===================================================================

class TestYawSolver:
    def test_straight_ahead(self):
        """Target on +X axis → yaw ≈ 0."""
        yaw = _yaw_solve(500.0, 0.0, -105.65)
        assert abs(yaw) < math.radians(15)

    def test_no_solution_inside_circle(self):
        """Target inside the s-offset circle → ValueError."""
        with pytest.raises(ValueError, match="No yaw solution"):
            _yaw_solve(10.0, 10.0, 200.0)  # s > hypot(x,y)


# ===================================================================
# Test: Release state
# ===================================================================

class TestReleaseState:
    def test_release_position_above_bb(self):
        """Release point should be above or near BB's Z position."""
        bb = _make_bb()
        target = np.array([0.0, 0.0, _CATCH_Z])
        pos, vel, tof = bb.compute_release_state(target)
        # Release Z should be near BB position Z (within arm reach)
        assert pos[2] > _BB_POS[2] - 200  # not far below BB
        assert pos[2] < _BB_POS[2] + 300  # not unreasonably above

    def test_release_velocity_horizontal_toward_target(self):
        """Horizontal component of release velocity should point toward target.

        The full velocity vector includes an upward (Z) component for the
        parabolic arc, so we check only the XY projection.
        """
        bb = _make_bb()
        target = np.array([0.0, 0.0, _CATCH_Z])
        pos, vel, tof = bb.compute_release_state(target)
        direction_xy = target[:2] - pos[:2]
        dot = np.dot(vel[:2], direction_xy)
        assert dot > 0, "Horizontal velocity should point toward target"

    def test_time_of_flight_positive(self):
        bb = _make_bb()
        target = np.array([0.0, 0.0, _CATCH_Z])
        _, _, tof = bb.compute_release_state(target)
        assert tof > 0
        assert tof < 5.0  # reasonable upper bound


# ===================================================================
# Test: Ball reaches target (end-to-end ballistics)
# ===================================================================

class TestBallReachesTarget:
    def test_centre_target(self):
        """Ball thrown at (0,0,catch_z) lands within 50mm of target."""
        bb = _make_bb()
        target = np.array([0.0, 0.0, _CATCH_Z])
        pos, vel, _ = bb.compute_release_state(target)
        _, landing_pos, _ = _ball_landing(pos, vel, _CATCH_Z)
        error = np.linalg.norm(landing_pos[:2] - target[:2])
        assert error < 50.0, f"Landing XY error {error:.1f} mm > 50 mm"

    def test_offset_target(self):
        """Ball thrown at lateral offset still lands close."""
        bb = _make_bb()
        target = np.array([50.0, -30.0, _CATCH_Z])
        pos, vel, _ = bb.compute_release_state(target)
        _, landing_pos, _ = _ball_landing(pos, vel, _CATCH_Z)
        error = np.linalg.norm(landing_pos[:2] - target[:2])
        assert error < 50.0, f"Landing XY error {error:.1f} mm > 50 mm"


# ===================================================================
# Test: Error cases
# ===================================================================

class TestErrors:
    def test_unreachable_behind_bb(self):
        """Target directly behind BB (negative local X) should fail."""
        # Use a simple BB pointing along world +X so "behind" is -X
        bb = BallButlerSim.from_hardware_config(
            position_mm=[0.0, 0.0, 1500.0],
            yaw_offset_rad=0.0,
        )
        # Target far behind BB (negative local X = negative world X)
        target = np.array([-2000.0, 0.0, 1500.0])
        with pytest.raises(ValueError):
            bb.compute_release_state(target)

    def test_target_too_far(self):
        """Target requiring more than max speed should fail."""
        bb = _make_bb()
        # Very far target at same height — needs huge speed
        target = np.array([5000.0, 5000.0, 1500.0])
        with pytest.raises(ValueError):
            bb.compute_release_state(target)


# ===================================================================
# Test: World-frame transforms
# ===================================================================

class TestWorldFrame:
    def test_different_positions_similar_landing(self):
        """Two BB positions throwing at the same target should hit the same spot."""
        target = np.array([0.0, 0.0, _CATCH_Z])

        # Two BBs at different positions, each pointing at JB
        pos1 = np.array([500.0, 0.0, 1500.0])
        yaw1 = math.atan2(-pos1[1], -pos1[0])
        bb1 = BallButlerSim.from_hardware_config(pos1, yaw1)
        r1_pos, r1_vel, _ = bb1.compute_release_state(target)
        _, land1, _ = _ball_landing(r1_pos, r1_vel, _CATCH_Z)

        pos2 = np.array([0.0, -500.0, 1400.0])
        yaw2 = math.atan2(-pos2[1], -pos2[0])
        bb2 = BallButlerSim.from_hardware_config(pos2, yaw2)
        r2_pos, r2_vel, _ = bb2.compute_release_state(target)
        _, land2, _ = _ball_landing(r2_pos, r2_vel, _CATCH_Z)

        # Both should land near (0, 0)
        err1 = np.linalg.norm(land1[:2])
        err2 = np.linalg.norm(land2[:2])
        assert err1 < 50.0, f"BB1 landing error: {err1:.1f} mm"
        assert err2 < 50.0, f"BB2 landing error: {err2:.1f} mm"

        # Release positions should be different
        assert np.linalg.norm(r1_pos - r2_pos) > 50.0

    def test_yaw_offset_rotates_release(self):
        """Non-zero yaw offset should rotate the release position."""
        # Two BBs at the same spot but with different orientations
        pos = np.array([500.0, 0.0, 1500.0])
        target1 = np.array([0.0, 0.0, _CATCH_Z])

        # BB pointing at origin
        yaw1 = math.atan2(0 - pos[1], 0 - pos[0])
        bb1 = BallButlerSim.from_hardware_config(pos, yaw1)
        rp1, _, _ = bb1.compute_release_state(target1)

        # BB rotated 45° — aim at a different target that's in range
        yaw2 = yaw1 + math.radians(45)
        bb2 = BallButlerSim.from_hardware_config(pos, yaw2)
        # Need a target along bb2's forward axis
        target2 = np.array([
            pos[0] + 500 * math.cos(yaw2),
            pos[1] + 500 * math.sin(yaw2),
            _CATCH_Z,
        ])
        rp2, _, _ = bb2.compute_release_state(target2)

        # Release positions should differ due to different orientations
        assert np.linalg.norm(rp1 - rp2) > 10.0


# ===================================================================
# Test: Convenience methods
# ===================================================================

class TestConvenience:
    def test_throw_at_returns_ball_spawn(self):
        bb = _make_bb()
        spawn = bb.throw_at(np.array([0.0, 0.0, _CATCH_Z]), spawn_time=1.0)
        assert isinstance(spawn, BallSpawn)
        assert spawn.spawn_time == 1.0
        assert spawn.position_mm.shape == (3,)
        assert spawn.velocity_mms.shape == (3,)

    def test_throw_at_jugglebot(self):
        bb = _make_bb()
        spawn = bb.throw_at_jugglebot(spawn_time=0.5)
        assert isinstance(spawn, BallSpawn)
        assert spawn.spawn_time == 0.5
        # Ball should start near BB position (not at Jugglebot)
        assert spawn.position_mm[2] > 1000.0

    def test_scatter_varies_landing(self):
        """Scatter should produce different landing positions."""
        bb = _make_bb()
        landings = []
        for _ in range(50):
            try:
                spawn = bb.throw_at_jugglebot(spawn_time=0.5, scatter_mm=30.0)
            except ValueError:
                continue  # some scattered targets may be out of BB's range
            _, land, _ = _ball_landing(
                spawn.position_mm, spawn.velocity_mms, _CATCH_Z)
            landings.append(land[:2].copy())
        assert len(landings) >= 10, "Too few valid scattered throws"
        landings = np.array(landings)
        # Standard deviation should be non-trivial (> 5mm)
        std = np.std(landings, axis=0)
        assert np.max(std) > 5.0, f"Scatter too small: std = {std}"


# ===================================================================
# Test: End-to-end with catch pipeline
# ===================================================================

class TestEndToEnd:
    def test_bb_spawn_to_catch_target(self):
        """BallSpawn from BB → _compute_catch_target → valid DynamicTarget."""
        bb = _make_bb()
        spawn = bb.throw_at_jugglebot(spawn_time=0.5)
        target, ball = _compute_catch_target(
            spawn.position_mm, spawn.velocity_mms, spawn.spawn_time)
        # Target should have a reasonable pose
        assert target.pose_6dof.shape == (6,)
        # XY should be near zero (we aimed at centre)
        assert abs(target.pose_6dof[0]) < 100.0
        assert abs(target.pose_6dof[1]) < 100.0
        # Arrival time should be positive and reasonable
        assert target.arrival_time > spawn.spawn_time
        assert target.arrival_time < spawn.spawn_time + 3.0
