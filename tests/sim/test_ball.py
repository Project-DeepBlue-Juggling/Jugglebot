"""Tests for ball physics and kinematic hold capture.

Validates ball spawning, gravity, proximity-based capture via kinematic
hold, release, reset, and hold tracking behavior.
"""

from __future__ import annotations

import numpy as np
import pytest
import mujoco

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

    def test_spawn_clears_held(self, plant):
        """Spawning a ball clears any active hold state."""
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
    """Ball capture via proximity check and kinematic hold."""

    def _prime_hand_and_settle(self, plant):
        """Prime the hand and step until it settles."""
        plant.hand_to_prime()
        for _ in range(30):
            plant.step(0.02)

    def test_capture_from_above(self, plant):
        """Spawn ball near hand opening — it should be captured instantly
        when it enters the proximity zone."""
        self._prime_hand_and_settle(plant)

        # Get hand opening position in world frame
        hand_site_id = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_SITE, 'hand_opening')
        hand_opening_pos_m = plant.data.site_xpos[hand_site_id].copy()
        hand_opening_mm = hand_opening_pos_m * 1000

        # Spawn ball 10mm above hand opening with gentle downward velocity.
        # Ball enters the capture zone and is instantly caught (no settle).
        spawn_pos = hand_opening_mm.copy()
        spawn_pos[2] += 10
        plant.spawn_ball(spawn_pos, np.array([0, 0, -200]))  # gentle drop

        # Step until captured — should be near-instant
        captured = False
        for _ in range(500):
            plant.step(0.001)
            state = plant.get_ball_state()
            if state and state.held:
                captured = True
                break

        assert captured, "Ball was not captured"

    def test_ball_stays_locked_to_hand(self, plant):
        """After capture, ball tracks hand opening site exactly (kinematic hold)."""
        self._prime_hand_and_settle(plant)

        # Spawn ball at hand opening with zero velocity — instant capture
        hand_site_id = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_SITE, 'hand_opening')
        hand_opening_mm = plant.data.site_xpos[hand_site_id] * 1000

        plant.spawn_ball(hand_opening_mm.copy(), np.array([0, 0, 0]))

        # Step until captured
        for _ in range(100):
            plant.step(0.001)
            if plant.get_ball_state().held:
                break
        assert plant.get_ball_state().held, "Ball should be captured"

        # Step for 1 second and verify ball tracks hand site
        for _ in range(50):
            plant.step(0.02)

        site_pos_mm = plant.data.site_xpos[hand_site_id] * 1000
        ball_pos_mm = plant.get_ball_state().position_mm
        drift = np.linalg.norm(ball_pos_mm - site_pos_mm)
        assert drift < 0.1, f"Ball drifted {drift:.3f} mm from hand site (kinematic hold should be exact)"


class TestBallRelease:
    """Ball can be released from kinematic hold and resumes free flight."""

    def test_release_resumes_free_flight(self, plant):
        # Prime hand and capture ball
        plant.hand_to_prime()
        for _ in range(30):
            plant.step(0.02)

        hand_site_id = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_SITE, 'hand_opening')
        hand_opening_mm = plant.data.site_xpos[hand_site_id] * 1000

        plant.spawn_ball(hand_opening_mm.copy(), np.array([0, 0, 0]))
        for _ in range(100):
            plant.step(0.001)
            if plant.get_ball_state().held:
                break
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

        # Spawn 200 mm to the side (well outside 30mm capture radius)
        spawn_pos = hand_opening_mm.copy()
        spawn_pos[0] += 200
        spawn_pos[2] += 50
        plant.spawn_ball(spawn_pos, np.array([0, 0, -500]))

        for _ in range(100):
            plant.step(0.02)
            captured = plant.check_and_capture()
            assert not captured, "Ball should not be captured when offset from hand"


class TestBallisticRelease:
    """Ball.ballistic_release — the contact-carry KINEMATIC release (Rung 2b).

    The clean-separation hand-off: cut a contact-carried held ball free at a SET
    velocity, breaking hand contact so no residual push corrupts the launch, then
    re-arm the contact-carry seat capture. See
    ``ball.manager.Ball.ballistic_release``.
    """

    def _carry_plant(self):
        p = MuJoCoPlant(contact_carry=True)
        p.reset()
        return p

    def test_imposes_velocity_and_breaks_contact(self):
        """The contract's core: after ballistic_release the ball is free, its
        linear velocity is EXACTLY the imposed value, and hand contact is broken
        (ball geom contype -> 1, ground-only) so no residual push re-corrupts it."""
        plant = self._carry_plant()
        # settle the platform, then seat a ball in the cup (kinematic spawn_in_hand)
        for _ in range(30):
            plant.step(0.02)
        plant.ball_manager.ball(0).spawn_in_hand()
        assert plant.get_ball_state(0).held

        ball = plant.ball_manager.ball(0)
        v_mms = np.array([300.0, -150.0, 2200.0])       # arbitrary take-off (mm/s)
        plant.ballistic_release(v_mms, ball=0)

        assert not ball.held                            # ball is free
        assert plant.model.geom_contype[ball._ball_geom_id] == 1   # contact broken
        # the imposed velocity landed exactly on the ball's dof (mm/s -> m/s)
        vadr = ball._ball_qvel_adr
        np.testing.assert_allclose(plant.data.qvel[vadr:vadr + 3], v_mms / 1000.0,
                                   rtol=0, atol=1e-9)
        # ...and the reported state velocity matches (a clean, contact-free launch).
        # get_ball_state reads MuJoCo sensordata, refreshed only by mj_step/mj_forward;
        # ballistic_release writes qvel directly, so forward once before reading (in the
        # real loop a plant.step always follows the release, so this never bites there).
        mujoco.mj_forward(plant.model, plant.data)
        st = plant.get_ball_state(0)
        np.testing.assert_allclose(np.asarray(st.velocity_mms), v_mms, rtol=0, atol=1e-6)

    def test_no_residual_push_flies_ballistic(self):
        """With contact broken the ball flies at the imposed velocity (no residual
        contact push): a pure-upward release rises and its horizontal velocity is
        preserved, unlike the emergent contact throw whose take-off depends on the
        contact geometry (the knife-edge the release defeats)."""
        plant = self._carry_plant()
        for _ in range(30):
            plant.step(0.02)
        plant.ball_manager.ball(0).spawn_in_hand()
        vx0 = 250.0
        plant.ballistic_release(np.array([vx0, 0.0, 2500.0]), ball=0)
        # forward once so sensordata reflects the seated qpos + imposed qvel before the
        # baseline z read (ballistic_release writes qpos/qvel directly, no step yet).
        mujoco.mj_forward(plant.model, plant.data)
        z0 = plant.get_ball_state(0).position_mm[2]
        for _ in range(4):
            plant.step(0.02)
        st = plant.get_ball_state(0)
        assert st.position_mm[2] > z0                   # rose (went up)
        # horizontal velocity preserved within a hair (gravity is vertical only)
        assert abs(st.velocity_mms[0] - vx0) < 5.0
