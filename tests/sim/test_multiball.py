"""Tests for multi-ball support in the MuJoCo sim (Phase 3a).

The juggling demo needs two balls in the scene at once (one in hand, one in
flight). These tests validate the two-ball model + the BallManager registry:
independent spawn / gravity / capture / reset per ball, and the key
one-held-one-in-flight scenario.
"""
from __future__ import annotations

import numpy as np
import pytest
import mujoco

from plant.mujoco_plant import MuJoCoPlant


@pytest.fixture(scope='module')
def plant():
    return MuJoCoPlant()


@pytest.fixture(autouse=True)
def reset_plant(plant):
    plant.reset()
    yield


def _hand_opening_mm(plant):
    sid = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_SITE, 'hand_opening')
    return plant.data.site_xpos[sid].copy() * 1000.0


def test_model_has_two_balls(plant):
    assert plant.n_balls == 2
    assert plant.ball_manager.count == 2
    assert len(plant.ball_manager) == 2
    assert plant.ball_manager.ball(0).name == 'ball'
    assert plant.ball_manager.ball(1).name == 'ball2'


def test_two_balls_spawn_independently(plant):
    plant.spawn_ball(np.array([100.0, 0.0, 1500.0]), np.zeros(3), ball=0)
    plant.spawn_ball(np.array([-100.0, 200.0, 1800.0]), np.zeros(3), ball=1)
    s0 = plant.get_ball_state(0)
    s1 = plant.get_ball_state(1)
    np.testing.assert_allclose(s0.position_mm, [100, 0, 1500], atol=1.0)
    np.testing.assert_allclose(s1.position_mm, [-100, 200, 1800], atol=1.0)
    assert s0.active and s1.active
    assert not s0.held and not s1.held


def test_two_balls_fall_independently(plant):
    z0_a, z0_b = 2000.0, 2500.0
    plant.spawn_ball(np.array([0.0, 0.0, z0_a]), np.zeros(3), ball=0)
    plant.spawn_ball(np.array([300.0, 0.0, z0_b]), np.zeros(3), ball=1)
    for _ in range(int(0.4 / plant.timestep)):
        mujoco.mj_step(plant.model, plant.data)
    drop_a = z0_a - plant.get_ball_state(0).position_mm[2]
    drop_b = z0_b - plant.get_ball_state(1).position_mm[2]
    expected = 0.5 * 9.806 * 0.4 ** 2 * 1000.0
    assert abs(drop_a - expected) < expected * 0.05
    assert abs(drop_b - expected) < expected * 0.05
    # Distinct bodies — x positions stay separated.
    assert abs(plant.get_ball_state(0).position_mm[0]
               - plant.get_ball_state(1).position_mm[0]) > 250.0


def test_per_ball_capture_is_independent(plant):
    """Ball 0 at the hand opening is caught; ball 1 offset 300 mm is not."""
    plant.hand_to_prime()
    for _ in range(30):
        plant.step(0.02)

    opening = _hand_opening_mm(plant)
    plant.spawn_ball(opening.copy(), np.zeros(3), ball=0)             # on-axis
    plant.spawn_ball(opening + np.array([300.0, 0.0, 50.0]),
                     np.array([0.0, 0.0, -300.0]), ball=1)            # offset

    caught0 = caught1 = False
    for _ in range(120):
        plant.step(0.01)
        caught0 = caught0 or plant.check_and_capture(0)
        caught1 = caught1 or plant.check_and_capture(1)

    assert caught0, "ball 0 (on-axis) should be captured"
    assert not caught1, "ball 1 (300 mm offset) must not be captured"
    assert plant.get_ball_state(0).held
    assert not plant.get_ball_state(1).held


def test_one_held_one_in_flight(plant):
    """The core demo scenario: ball 0 held in hand while ball 1 flies free."""
    plant.hand_to_prime()
    for _ in range(30):
        plant.step(0.02)

    plant.ball_manager.ball(0).spawn_in_hand()                # ball 0 -> hand
    z0 = 2500.0
    plant.spawn_ball(np.array([400.0, 0.0, z0]), np.zeros(3), ball=1)  # in flight
    mujoco.mj_forward(plant.model, plant.data)

    for _ in range(40):
        plant.step(0.02)

    held_ball = plant.get_ball_state(0)
    flight_ball = plant.get_ball_state(1)
    # Ball 0 stays held and tracks the hand opening.
    assert held_ball.held
    drift = np.linalg.norm(held_ball.position_mm - _hand_opening_mm(plant))
    assert drift < 1.0, f"held ball drifted {drift:.2f} mm from the hand"
    # Ball 1 fell freely under gravity, unaffected by the held ball.
    assert not flight_ball.held
    assert flight_ball.position_mm[2] < z0 - 100.0


def test_reset_parks_both_balls(plant):
    plant.spawn_ball(np.array([0.0, 0.0, 500.0]), np.zeros(3), ball=0)
    plant.spawn_ball(np.array([0.0, 0.0, 600.0]), np.zeros(3), ball=1)
    plant.reset()
    s0 = plant.get_ball_state(0)
    s1 = plant.get_ball_state(1)
    assert not s0.active and not s1.active
    assert s0.position_mm[2] > 4000.0 and s1.position_mm[2] > 4000.0
