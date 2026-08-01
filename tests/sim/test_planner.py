"""Tests for ThrowCatchPlanner (sim/hand/planner.py).

Tests plan generation, feasibility, and consistency of the throw-catch pipeline.
"""

from __future__ import annotations

import pytest
import numpy as np

from sim.hand.planner import ThrowCatchPlanner, ThrowCatchPlan
from sim.hand.ballistics import compute_launch_velocity, compute_arrival_velocity, rodrigues
from sim.hand.trajectory import max_throw_speed_mps


class TestThrowCatchPlanner:
    """Unit tests for ThrowCatchPlanner.plan()."""

    def test_vertical_throw_catch(self):
        """Symmetric vertical throw/catch — zero tilt, consistent velocities."""
        planner = ThrowCatchPlanner()
        plan = planner.plan(
            throw_pos_mm=np.array([0.0, 0.0, 800.0]),
            catch_pos_mm=np.array([0.0, 0.0, 800.0]),
            throw_time=1.5,
            catch_time=2.5,
        )

        # Throw orientation should be zero (vertical)
        throw_rv = plan.throw_target.pose_6dof[3:6]
        assert np.linalg.norm(throw_rv) < 0.01, \
            f"Expected zero tilt for vertical throw, got {throw_rv}"

        # Catch orientation should also be zero (symmetric)
        catch_rv = plan.catch_target.pose_6dof[3:6]
        assert np.linalg.norm(catch_rv) < 0.01

    def test_angled_throw_catch(self):
        """Angled throw/catch — both poses have tilt."""
        planner = ThrowCatchPlanner()
        plan = planner.plan(
            throw_pos_mm=np.array([0.0, 0.0, 800.0]),
            catch_pos_mm=np.array([30.0, -20.0, 780.0]),
            throw_time=2.0,
            catch_time=3.4,
        )

        throw_rv = plan.throw_target.pose_6dof[3:6]
        catch_rv = plan.catch_target.pose_6dof[3:6]

        # Both should have nonzero tilt (off-axis catch)
        assert np.linalg.norm(throw_rv) > 0.001
        assert np.linalg.norm(catch_rv) > 0.001

    def test_platform_centroids(self):
        """Centroid positions account for hand offset along tilted Z."""
        planner = ThrowCatchPlanner()
        plan = planner.plan(
            throw_pos_mm=np.array([0.0, 0.0, 800.0]),
            catch_pos_mm=np.array([0.0, 0.0, 800.0]),
            throw_time=1.5,
            catch_time=2.5,
        )

        # For vertical throw, centroid Z = ball_z - hand_offset - platform_height
        # Centroid should be below ball position
        throw_z = plan.throw_target.pose_6dof[2]
        assert throw_z < 800.0 - 574.3  # Below ball relative to home

    def test_ball_release_velocity(self):
        """Ball release velocity matches ballistic inverse."""
        planner = ThrowCatchPlanner()
        throw_pos = np.array([0.0, 0.0, 800.0])
        catch_pos = np.array([0.0, 0.0, 800.0])
        plan = planner.plan(
            throw_pos_mm=throw_pos,
            catch_pos_mm=catch_pos,
            throw_time=1.5,
            catch_time=2.5,
        )

        expected_v = compute_launch_velocity(throw_pos, catch_pos, 1.0)
        np.testing.assert_allclose(plan.ball_release_vel_mms, expected_v, atol=1.0)

    def test_catch_arrival_velocity(self):
        """Catch arrival velocity consistent with launch + gravity."""
        planner = ThrowCatchPlanner()
        plan = planner.plan(
            throw_pos_mm=np.array([0.0, 0.0, 800.0]),
            catch_pos_mm=np.array([0.0, 0.0, 800.0]),
            throw_time=1.5,
            catch_time=2.5,
        )

        flight_time = 1.0
        v_arrival = compute_arrival_velocity(plan.ball_release_vel_mms, flight_time)
        # Catch event_vel should match arrival speed
        expected_speed = np.linalg.norm(v_arrival) / 1000.0
        assert plan.catch_target.event_vel_mps == pytest.approx(
            max(0.3, min(7.0, expected_speed)), abs=0.01)

    def test_infeasible_speed(self):
        """Rejects throw requiring > max hand speed."""
        planner = ThrowCatchPlanner()
        with pytest.raises(ValueError, match="throw speed"):
            # Same height, 2s flight → v = 0.5 * g * T = 9806 mm/s = 9.806 m/s > 7.0
            planner.plan(
                throw_pos_mm=np.array([0.0, 0.0, 800.0]),
                catch_pos_mm=np.array([0.0, 0.0, 800.0]),
                throw_time=1.0,
                catch_time=3.0,
            )

    def test_infeasible_tilt(self):
        """Rejects > 30° approach angle."""
        planner = ThrowCatchPlanner()
        with pytest.raises(ValueError, match="[Tt]ilt"):
            # Large lateral offset with short flight → steep angle > 30°
            planner.plan(
                throw_pos_mm=np.array([0.0, 0.0, 800.0]),
                catch_pos_mm=np.array([600.0, 0.0, 800.0]),
                throw_time=1.0,
                catch_time=1.3,
            )

    def test_hand_timing_chain(self):
        """Throw sequence ends before catch sequence starts."""
        planner = ThrowCatchPlanner()
        plan = planner.plan(
            throw_pos_mm=np.array([0.0, 0.0, 800.0]),
            catch_pos_mm=np.array([0.0, 0.0, 800.0]),
            throw_time=1.5,
            catch_time=2.5,
        )

        throw_end = plan.throw_hand_seq.end_time
        catch_start = plan.catch_hand_seq.prelude_start_time
        assert throw_end < catch_start, \
            f"Throw ends at {throw_end:.3f}, catch starts at {catch_start:.3f}"

    def test_modes_set_correctly(self):
        """Throw target has mode='throw', catch has mode='catch'."""
        planner = ThrowCatchPlanner()
        plan = planner.plan(
            throw_pos_mm=np.array([0.0, 0.0, 800.0]),
            catch_pos_mm=np.array([0.0, 0.0, 800.0]),
            throw_time=1.5,
            catch_time=2.5,
        )
        assert plan.throw_target.mode == 'throw'
        assert plan.catch_target.mode == 'catch'

    def test_ball_spawn_matches(self):
        """Ball spawn parameters match throw position and launch velocity."""
        planner = ThrowCatchPlanner()
        throw_pos = np.array([10.0, -5.0, 800.0])
        plan = planner.plan(
            throw_pos_mm=throw_pos,
            catch_pos_mm=np.array([0.0, 0.0, 800.0]),
            throw_time=1.5,
            catch_time=2.5,
        )

        np.testing.assert_allclose(plan.ball_spawn.position_mm, throw_pos)
        assert plan.ball_spawn.spawn_time == 1.5
        np.testing.assert_allclose(
            plan.ball_spawn.velocity_mms, plan.ball_release_vel_mms)

    def test_invalid_catch_before_throw(self):
        """Rejects catch_time before throw_time."""
        planner = ThrowCatchPlanner()
        with pytest.raises(ValueError, match="catch_time"):
            planner.plan(
                throw_pos_mm=np.array([0.0, 0.0, 800.0]),
                catch_pos_mm=np.array([0.0, 0.0, 800.0]),
                throw_time=2.0,
                catch_time=1.0,
            )
