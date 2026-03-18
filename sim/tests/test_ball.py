"""Tests for ball physics and capture (Phase 5B).

Validates ball spawning, gravity, proximity-based capture via weld constraint,
release, and reset behavior.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import pytest
import mujoco

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from plant.mujoco_plant import MuJoCoPlant


@pytest.fixture(scope='module')
def plant():
    p = MuJoCoPlant()
    return p


@pytest.fixture(autouse=True)
def reset_plant(plant):
    plant.reset()
    yield


class TestBallSpawn:
    """Ball can be spawned at a known position with known velocity."""

    def test_spawn_position(self, plant):
        plant.spawn_ball(np.array([100, 200, 1500]), np.array([0, 0, 0]))
        state = plant.get_ball_state()
        assert state is not None
        assert state.active is True
        assert state.held is False
        np.testing.assert_allclose(state.position_mm, [100, 200, 1500], atol=1.0)

    def test_spawn_velocity(self, plant):
        plant.spawn_ball(np.array([0, 0, 2000]), np.array([500, -300, -1000]))
        state = plant.get_ball_state()
        np.testing.assert_allclose(state.velocity_mms, [500, -300, -1000], atol=50)

    def test_spawn_resets_weld(self, plant):
        """Spawning a ball disables any active weld."""
        plant.spawn_ball(np.array([0, 0, 1000]), np.array([0, 0, 0]))
        state = plant.get_ball_state()
        assert state.held is False


class TestBallGravity:
    """Ball falls under gravity correctly."""

    def test_free_fall(self, plant):
        """Spawn at rest, step 0.5s, verify Z drops by ~½gt²."""
        z0 = 2000.0
        plant.spawn_ball(np.array([0, 0, z0]), np.array([0, 0, 0]))
        mujoco.mj_forward(plant.model, plant.data)

        dt = 0.5
        n_steps = int(dt / plant.timestep)
        for _ in range(n_steps):
            mujoco.mj_step(plant.model, plant.data)

        state = plant.get_ball_state()
        expected_drop = 0.5 * 9.806 * dt**2 * 1000  # ~1225.8 mm
        actual_drop = z0 - state.position_mm[2]
        # Allow 1% tolerance (no drag in MuJoCo for sphere without medium)
        assert abs(actual_drop - expected_drop) < expected_drop * 0.01, \
            f"Drop {actual_drop:.1f} mm, expected ~{expected_drop:.1f} mm"


class TestBallCapture:
    """Ball capture via proximity check and weld constraint."""

    def _prime_hand_and_settle(self, plant):
        """Prime the hand and step until it settles."""
        plant.hand_to_prime()
        for _ in range(30):
            plant.step(0.02)

    def test_capture_from_above(self, plant):
        """Spawn ball just above hand opening with slow downward velocity."""
        self._prime_hand_and_settle(plant)

        # Get hand opening position in world frame
        hand_site_id = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_SITE, 'hand_opening')
        hand_opening_pos_m = plant.data.site_xpos[hand_site_id].copy()
        hand_opening_mm = hand_opening_pos_m * 1000

        # Spawn ball 10mm above hand opening with slow downward velocity
        # This gives the ball time to enter the cone before exceeding the
        # capture velocity threshold (500 mm/s)
        spawn_pos = hand_opening_mm.copy()
        spawn_pos[2] += 10
        plant.spawn_ball(spawn_pos, np.array([0, 0, -100]))  # gentle drop

        # Step at physics rate to ensure we catch the capture frame
        captured = False
        for _ in range(500):  # up to 0.5s
            plant.step(0.001)
            if plant.check_and_capture():
                captured = True
                break

        assert captured, "Ball was not captured"
        state = plant.get_ball_state()
        assert state.held is True

    def test_ball_follows_platform_when_held(self, plant):
        """After capture, ball should move with the platform."""
        self._prime_hand_and_settle(plant)

        # Get hand opening and spawn ball right at it with zero velocity
        hand_site_id = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_SITE, 'hand_opening')
        hand_opening_mm = plant.data.site_xpos[hand_site_id] * 1000

        # Spawn exactly at hand opening with zero velocity (instant capture)
        plant.spawn_ball(hand_opening_mm.copy(), np.array([0, 0, 0]))
        mujoco.mj_forward(plant.model, plant.data)
        plant.check_and_capture()

        state = plant.get_ball_state()
        assert state.held, "Ball should be captured at hand opening"

        # Now command the platform to move and check ball follows
        z_before = state.position_mm[2]
        plant.command(plant.pose_to_extensions(np.array([0, 0, 50, 0, 0, 0])))
        for _ in range(25):
            plant.step(0.02)

        state = plant.get_ball_state()
        # Ball should have moved up with the platform (~50mm)
        z_delta = state.position_mm[2] - z_before
        assert z_delta > 30, f"Ball Z moved only {z_delta:.1f} mm, expected ~50 mm"


class TestBallRelease:
    """Ball can be released from weld and resumes free flight."""

    def test_release_resumes_free_flight(self, plant):
        # Prime hand and capture ball
        plant.hand_to_prime()
        for _ in range(30):
            plant.step(0.02)

        hand_site_id = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_SITE, 'hand_opening')
        hand_opening_mm = plant.data.site_xpos[hand_site_id] * 1000

        plant.spawn_ball(hand_opening_mm.copy(), np.array([0, 0, 0]))
        mujoco.mj_forward(plant.model, plant.data)
        plant.check_and_capture()
        assert plant.get_ball_state().held

        # Release with upward velocity
        z_at_release = plant.get_ball_state().position_mm[2]
        plant.release_ball(np.array([0, 0, 2000]))  # 2000 mm/s upward

        # Step and verify ball goes up
        for _ in range(5):
            plant.step(0.02)

        state = plant.get_ball_state()
        assert state.held is False
        assert state.position_mm[2] > z_at_release, "Ball should be moving upward after release"


class TestBallReset:
    """Ball resets to parked position."""

    def test_reset_parks_ball(self, plant):
        plant.spawn_ball(np.array([100, 200, 500]), np.array([0, 0, -1000]))
        plant.reset()
        state = plant.get_ball_state()
        assert state.active is False
        assert state.held is False
        assert state.position_mm[2] > 4000  # parked high


class TestBallMiss:
    """Ball that misses the hand is not captured."""

    def test_ball_offset_from_hand(self, plant):
        """Spawn ball far from hand axis — capture should never trigger."""
        plant.hand_to_prime()
        for _ in range(30):
            plant.step(0.02)

        hand_site_id = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_SITE, 'hand_opening')
        hand_opening_mm = plant.data.site_xpos[hand_site_id] * 1000

        # Spawn 200 mm to the side (well outside 35mm capture radius)
        spawn_pos = hand_opening_mm.copy()
        spawn_pos[0] += 200
        spawn_pos[2] += 50
        plant.spawn_ball(spawn_pos, np.array([0, 0, -500]))

        for _ in range(100):
            plant.step(0.02)
            captured = plant.check_and_capture()
            assert not captured, "Ball should not be captured when offset from hand"
