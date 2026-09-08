"""Tests for the hand actuator (Phase 5A).

Validates that the hand body, joint, actuator, and sensors are present in the
MJCF model, and that MuJoCoPlant correctly commands and reads hand state.
"""

from __future__ import annotations

import numpy as np
import pytest
import mujoco

from jugglebot import hardware_config as hw
from sim.plant.mujoco_plant import MuJoCoPlant


@pytest.fixture(scope='module')
def plant():
    """Shared MuJoCoPlant instance for hand tests."""
    p = MuJoCoPlant()
    return p


@pytest.fixture(autouse=True)
def reset_plant(plant):
    """Reset plant before each test."""
    plant.reset()
    yield


class TestHandModel:
    """Hand body, joint, actuator, and sensors exist in the model."""

    def test_hand_body_exists(self, plant):
        m = plant.model
        hand_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'hand')
        assert hand_id >= 0, "hand body not found"

    def test_hand_slide_joint(self, plant):
        """Joint range is 0 .. the travel ABOVE ENCODER ZERO, derived from the
        measured hard stop and gain — NOT ``jugglebot_geometry.hand_stroke_mm``
        directly.

        Re-pointed 2026-09-08 (hand-geometry correction, owner decision D2).
        The joint is single-sided from zero, and zero is ENCODER zero, which
        sits 0.107 rev ABOVE the bottom hard stop. ``hand_stroke_mm`` (352.0)
        is the STOP-TO-STOP measurement (10.808 rev): pinning the joint range
        to it directly would give the modelled hand ~3.48 mm of travel the
        real machine does not have above its encoder zero. The correct clip is
        ``hand_motor_hard_stop_revs / gain`` = 10.701 / 30.703768 = 0.348524 m,
        which is what ``sim/model/generate_mjcf.py`` now emits.

        Before 2026-08-18 this assertion hardcoded 0.355 and went stale when
        the operator measured the sensorised hand — one of three places
        (alongside ``mujoco_plant.py``'s clip bound and the MJCF itself) that
        correction had to reach. Deriving it from the hard stop instead of a
        stroke literal is meant to end that recurring drift for good.

        NOT ``teensy_trajectory.hand_stroke_m`` (0.3643707) either — that is
        the throw-profile basis and is a different number on purpose.
        """
        linear_gain_rev_per_m = (
            hw.TEENSY_TRAJ_LINEAR_GAIN_FACTOR
            / (2.0 * np.pi * hw.TEENSY_TRAJ_HAND_SPOOL_RADIUS_M)
        )
        expected_travel_m = hw.GEOM_HAND_MOTOR_HARD_STOP_REVS / linear_gain_rev_per_m
        m = plant.model
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'hand_slide')
        assert jid >= 0, "hand_slide joint not found"
        assert m.jnt_range[jid][0] == pytest.approx(0.0, abs=1e-6)
        assert m.jnt_range[jid][1] == pytest.approx(
            expected_travel_m, abs=1e-6), (
            "hand_slide range disagrees with hand_motor_hard_stop_revs/gain — "
            "regenerate the model: python sim/model/generate_mjcf.py")

    def test_hand_actuator(self, plant):
        m = plant.model
        aid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, 'act_hand')
        assert aid >= 0, "act_hand actuator not found"

    def test_hand_sensors(self, plant):
        m = plant.model
        for name in ['hand_slide_pos', 'hand_slide_vel']:
            sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, name)
            assert sid >= 0, f"sensor '{name}' not found"

    def test_hand_is_child_of_platform(self, plant):
        m = plant.model
        hand_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'hand')
        platform_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'platform')
        assert m.body_parentid[hand_id] == platform_id


class TestHandCommand:
    """Hand responds to position commands correctly."""

    def test_command_to_prime(self, plant):
        """Command hand to prime position, verify it settles within 1 mm / 500 ms.

        The expectation is DERIVED, not a literal.  It read
        ``9.858 * 2π * 5.21`` (~322.7 mm) until 2026-08-21 — the pre-Phase-3
        prime, converted with the wrong gain (no ``LINEAR_GAIN_FACTOR``) — and
        pinned ``MuJoCoPlant`` to the same stale pair, so the sim reproduced the
        76.5 ms prelude Phase 3 removed on hardware.  Deriving it here means a
        future codegen change to ``HAND_STROKE_TOP_REV`` moves the test with the
        plant instead of failing it.
        """
        linear_gain_rev_per_m = (
            hw.TEENSY_TRAJ_LINEAR_GAIN_FACTOR
            / (2.0 * np.pi * hw.TEENSY_TRAJ_HAND_SPOOL_RADIUS_M)
        )
        prime_mm = (hw.TEENSY_TRAJ_STROKE_MARGIN_M * 1000.0
                    + hw.HAND_STROKE_TOP_REV / linear_gain_rev_per_m * 1000.0)
        # 335.0 -> 344.371 on 2026-09-08 (hand-geometry correction): x3 itself
        # is unchanged in rev (the re-based hand_stroke_m holds it fixed), but
        # x3 in mm moved with the gain (315.0 -> 324.371 mm), which this
        # formula adds STROKE_MARGIN_MM (20 mm) on top of.
        assert prime_mm == pytest.approx(344.3707, abs=1e-4)
        plant.hand_to_prime()
        # Step for 500 ms at 20 ms intervals
        for _ in range(25):
            plant.step(0.02)
        state = plant.get_state()
        assert state.hand_pos_mm is not None
        assert abs(state.hand_pos_mm - prime_mm) < 1.0, \
            f"Hand at {state.hand_pos_mm:.1f} mm, expected ~{prime_mm:.1f} mm"

    def test_command_to_home(self, plant):
        """Prime then home, verify return to 0."""
        plant.hand_to_prime()
        for _ in range(25):
            plant.step(0.02)
        plant.hand_to_home()
        for _ in range(25):
            plant.step(0.02)
        state = plant.get_state()
        assert abs(state.hand_pos_mm) < 1.0, \
            f"Hand at {state.hand_pos_mm:.1f} mm, expected ~0 mm"

    def test_command_arbitrary_position(self, plant):
        """Command hand to 150 mm, verify it settles."""
        plant.command_hand(150.0)
        for _ in range(25):
            plant.step(0.02)
        state = plant.get_state()
        assert abs(state.hand_pos_mm - 150.0) < 1.0


class TestHandState:
    """PlantState correctly reports hand position and velocity."""

    def test_hand_state_at_home(self, plant):
        state = plant.get_state()
        assert state.hand_pos_mm is not None
        assert state.hand_vel_mmps is not None
        assert abs(state.hand_pos_mm) < 0.1
        assert abs(state.hand_vel_mmps) < 0.1

    def test_hand_velocity_during_motion(self, plant):
        """Hand velocity is nonzero while moving."""
        plant.hand_to_prime()
        plant.step(0.02)
        state = plant.get_state()
        # After one 20ms step the hand should still be moving (high gains
        # settle fast, so check after just 1 step, not 2)
        assert abs(state.hand_vel_mmps) > 10.0, \
            f"Hand velocity {state.hand_vel_mmps:.1f} mm/s seems too low during motion"


class TestHandBackcompat:
    """Existing PlantState fields still work as before."""

    def test_leg_extensions_unchanged(self, plant):
        """Leg extensions at home are still ~0."""
        state = plant.get_state()
        assert np.allclose(state.leg_extensions_mm, 0, atol=0.5)

    def test_platform_pos_unchanged(self, plant):
        """Platform pos at home is still ~0."""
        state = plant.get_state()
        assert np.allclose(state.platform_pos_mm, 0, atol=0.1)

    def test_has_hand_property(self, plant):
        assert plant.has_hand is True
