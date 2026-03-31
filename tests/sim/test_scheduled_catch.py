"""Integration tests for the EventScheduler-based catch pipeline.

Tests the full flow: EventScheduler → ScheduledCoordinator → MPC → plant.
Verifies that the scheduler produces smooth trajectories, the hand is
properly managed, and balls can be caught.
"""

from __future__ import annotations

import numpy as np
import pytest

from plant.mujoco_plant import MuJoCoPlant
from controller.mpc import MPCController
from controller.params import MPCParams
from controller.scheduler import EventScheduler, ScheduledEvent, EventType, SchedulerPhase
from hand.scheduled_coordinator import ScheduledCoordinator
from hand.coordinator import BallSpawn
from hand.trajectory import HandCatchSequence
from input.scripted import _compute_catch_target

CONTROL_DT = 0.025  # 40 Hz
IDLE_POSE = np.array([0.0, 0.0, 10.0, 0.0, 0.0, 0.0])


@pytest.fixture
def plant():
    return MuJoCoPlant()


@pytest.fixture
def mpc(plant):
    # Generous time budget for CI; the default 18ms is too tight for the
    # extended parameter vector (accel_ref + weights).
    return MPCController.from_plant(MPCParams(max_cpu_time=0.05), plant)


@pytest.fixture
def scheduler(mpc):
    return EventScheduler(
        active_pose=IDLE_POSE,
        cumulative_times=mpc.params.cumulative_times,
    )


def _run_scheduled_sim(
    plant: MuJoCoPlant,
    mpc: MPCController,
    coord: ScheduledCoordinator,
    duration: float,
) -> dict:
    """Run the simulation using ScheduledCoordinator."""
    max_pos_err = 0.0
    captures = 0
    active_hand_seq = None
    hand_cmds = []

    n_steps = int(duration / CONTROL_DT)
    for _ in range(n_steps):
        state = plant.get_state()
        current_pose = np.concatenate([state.platform_pos_mm, state.platform_rot])
        current_twist = state.platform_twist
        hand_pos = state.hand_pos_mm if state.hand_pos_mm is not None else 0.0

        # Update coordinator (produces target + hand + ball)
        tc, hand_cmd, ball_spawn = coord.update(
            state.time, current_pose, current_twist, hand_pos_mm=hand_pos)

        # Ball spawning
        if ball_spawn is not None:
            plant.spawn_ball(ball_spawn.position_mm, ball_spawn.velocity_mms)

        # Hand commands
        if isinstance(hand_cmd, HandCatchSequence):
            active_hand_seq = hand_cmd
            hand_cmds.append(('catch_seq', state.time))
        elif hand_cmd == 'prime':
            plant.hand_to_prime()
            active_hand_seq = None
            hand_cmds.append(('prime', state.time))
        elif hand_cmd == 'home':
            plant.hand_to_home()
            active_hand_seq = None
            hand_cmds.append(('home', state.time))

        # Sample active hand catch trajectory
        if active_hand_seq is not None:
            pos = active_hand_seq.sample(state.time)
            if pos is not None:
                plant.command_hand(pos)
            else:
                active_hand_seq = None

        # MPC solve
        cmd, _vel, diag = mpc.solve(
            state, tc.target_pose,
            ref_events=tc.ref_events,
            boost_vel_weights=True,
        )
        plant.command(cmd)
        plant.step(CONTROL_DT)

        # Track error
        if tc.target_pose is not None:
            pos_err = np.linalg.norm(current_pose[:3] - tc.target_pose[:3])
            max_pos_err = max(max_pos_err, pos_err)

        # Check capture
        if plant.has_ball and plant.check_and_capture():
            coord.notify_capture(state.time)
            captures += 1

    return {
        'final_pose': np.concatenate([
            plant.get_state().platform_pos_mm,
            plant.get_state().platform_rot,
        ]),
        'max_pos_error_mm': max_pos_err,
        'captures': captures,
        'hand_cmds': hand_cmds,
        'events_log': coord.events_log,
        'final_phase': coord.phase,
    }


class TestSchedulerSingleCatch:
    """Single-ball catch using the event scheduler."""

    def test_approach_and_hold(self, plant, mpc, scheduler):
        """Submit a catch event and verify the platform reaches it."""
        coord = ScheduledCoordinator(scheduler)

        # A simple catch target: above centre, vertical drop
        catch_pose = np.array([0.0, 0.0, 160.0, 0.0, 0.0, 0.0])
        arrival_time = 1.0

        coord.submit_catch(
            catch_pose=catch_pose,
            arrival_time=arrival_time,
            event_vel_mps=2.5,
        )

        result = _run_scheduled_sim(plant, mpc, coord, duration=1.5)

        # Platform should be near the catch pose
        final_pos_err = np.linalg.norm(result['final_pose'][:3] - catch_pose[:3])
        assert final_pos_err < 10.0, f"Position error {final_pos_err:.1f} mm"

        # Hand should have been primed
        prime_cmds = [c for c in result['hand_cmds'] if c[0] == 'prime']
        assert len(prime_cmds) >= 1, "Hand was not primed"

        # Catch sequence should have been sent
        seq_cmds = [c for c in result['hand_cmds'] if c[0] == 'catch_seq']
        assert len(seq_cmds) >= 1, "Catch sequence not sent"

    def test_catch_with_ball(self, plant, mpc, scheduler):
        """Full catch with a real ball — verify capture."""
        coord = ScheduledCoordinator(scheduler)

        # Compute a catch target from a ball drop
        spawn_pos = np.array([0.0, 0.0, 1200.0])
        spawn_vel = np.array([0.0, 0.0, -2000.0])
        spawn_time = 0.1

        target, ball_spawn = _compute_catch_target(
            spawn_pos, spawn_vel, spawn_time,
            active_z_offset_mm=IDLE_POSE[2],
        )

        coord.submit_catch(
            catch_pose=target.pose_6dof,
            arrival_time=target.arrival_time,
            event_vel_mps=target.event_vel_mps,
            ball_spawn=ball_spawn,
        )

        result = _run_scheduled_sim(plant, mpc, coord, duration=2.0)
        assert result['captures'] >= 1, "Ball was not caught"

    def test_return_to_active_after_capture(self, plant, mpc, scheduler):
        """After capture and return, platform should be near active pose."""
        coord = ScheduledCoordinator(scheduler)

        catch_pose = np.array([30.0, 0.0, 160.0, 0.0, 0.0, 0.0])
        arrival_time = 0.8

        coord.submit_catch(
            catch_pose=catch_pose,
            arrival_time=arrival_time,
            event_vel_mps=2.0,
        )

        # Run until holding, then simulate capture
        for _ in range(int(1.0 / CONTROL_DT)):
            state = plant.get_state()
            pose = np.concatenate([state.platform_pos_mm, state.platform_rot])
            twist = state.platform_twist
            hand_pos = state.hand_pos_mm or 0.0
            tc, hand_cmd, _ = coord.update(state.time, pose, twist, hand_pos)
            cmd, _, _ = mpc.solve(
                state, tc.target_pose,
                ref_events=tc.ref_events,
            )
            if isinstance(hand_cmd, HandCatchSequence):
                pass  # skip hand for this test
            elif hand_cmd == 'prime':
                plant.hand_to_prime()
            plant.command(cmd)
            plant.step(CONTROL_DT)

        # Simulate capture
        coord.notify_capture(plant.get_state().time)

        # Run the return phase
        for _ in range(int(3.0 / CONTROL_DT)):
            state = plant.get_state()
            pose = np.concatenate([state.platform_pos_mm, state.platform_rot])
            twist = state.platform_twist
            hand_pos = state.hand_pos_mm or 0.0
            tc, hand_cmd, _ = coord.update(state.time, pose, twist, hand_pos)
            cmd, _, _ = mpc.solve(
                state, tc.target_pose,
                ref_events=tc.ref_events,
            )
            plant.command(cmd)
            plant.step(CONTROL_DT)

        # Should be back near active pose
        final_pose = np.concatenate([
            plant.get_state().platform_pos_mm,
            plant.get_state().platform_rot,
        ])
        pos_err = np.linalg.norm(final_pose[:3] - IDLE_POSE[:3])
        assert pos_err < 10.0, f"Not near active pose: error {pos_err:.1f} mm"
        assert coord.phase == SchedulerPhase.IDLE


class TestSchedulerLiveRefinement:
    """Live target updates during approach."""

    def test_mid_approach_update(self, plant, mpc, scheduler):
        """Update catch target mid-approach — platform should track smoothly."""
        coord = ScheduledCoordinator(scheduler)

        initial_pose = np.array([30.0, 0.0, 160.0, 0.0, 0.0, 0.0])
        coord.submit_catch(
            catch_pose=initial_pose,
            arrival_time=1.0,
            event_vel_mps=2.5,
        )

        # Run for 0.3s
        for _ in range(int(0.3 / CONTROL_DT)):
            state = plant.get_state()
            pose = np.concatenate([state.platform_pos_mm, state.platform_rot])
            twist = state.platform_twist
            hand_pos = state.hand_pos_mm or 0.0
            tc, _, _ = coord.update(state.time, pose, twist, hand_pos)
            cmd, _, _ = mpc.solve(
                state, tc.target_pose,
                ref_events=tc.ref_events,
            )
            plant.command(cmd)
            plant.step(CONTROL_DT)

        # Now update the catch target (ball prediction shifted)
        refined_pose = np.array([50.0, 20.0, 155.0, 0.0, 0.0, 0.0])
        coord.update_catch(
            catch_pose=refined_pose,
            arrival_time=1.05,
            event_vel_mps=2.5,
        )

        # Run to arrival
        for _ in range(int(1.0 / CONTROL_DT)):
            state = plant.get_state()
            pose = np.concatenate([state.platform_pos_mm, state.platform_rot])
            twist = state.platform_twist
            hand_pos = state.hand_pos_mm or 0.0
            tc, _, _ = coord.update(state.time, pose, twist, hand_pos)
            cmd, _, _ = mpc.solve(
                state, tc.target_pose,
                ref_events=tc.ref_events,
            )
            plant.command(cmd)
            plant.step(CONTROL_DT)

        # Platform should be near the refined pose, not the initial one
        final = np.concatenate([
            plant.get_state().platform_pos_mm,
            plant.get_state().platform_rot,
        ])
        err_to_refined = np.linalg.norm(final[:3] - refined_pose[:3])
        err_to_initial = np.linalg.norm(final[:3] - initial_pose[:3])
        assert err_to_refined < err_to_initial, (
            f"Platform closer to initial ({err_to_initial:.1f}) "
            f"than refined ({err_to_refined:.1f})"
        )
        assert err_to_refined < 10.0, f"Position error to refined: {err_to_refined:.1f} mm"


class TestSchedulerTwoEvents:
    """Two-event chaining: catch then return."""

    def test_catch_then_transition(self, plant, mpc, scheduler):
        """Submit two events; verify platform reaches both sequentially."""
        coord = ScheduledCoordinator(scheduler)

        pose1 = np.array([40.0, 0.0, 160.0, 0.0, 0.0, 0.0])
        pose2 = np.array([-40.0, 0.0, 160.0, 0.0, 0.0, 0.0])

        ev1 = coord.submit_catch(
            catch_pose=pose1,
            arrival_time=0.6,
            event_vel_mps=2.0,
        )

        # Queue a second event
        from controller.scheduler import _next_event_id
        ev2 = ScheduledEvent(
            event_id=_next_event_id(),
            event_type=EventType.CATCH,
            pose=pose2,
            time=1.5,
            twist=None,
        )
        scheduler.submit_event(ev2)

        # Run for 2s
        states = []
        for _ in range(int(2.0 / CONTROL_DT)):
            state = plant.get_state()
            pose = np.concatenate([state.platform_pos_mm, state.platform_rot])
            twist = state.platform_twist
            hand_pos = state.hand_pos_mm or 0.0
            tc, _, _ = coord.update(state.time, pose, twist, hand_pos)
            cmd, _, _ = mpc.solve(
                state, tc.target_pose,
                ref_events=tc.ref_events,
            )
            plant.command(cmd)
            plant.step(CONTROL_DT)
            states.append(pose.copy())

        # Platform should be near pose2 at the end
        final = np.concatenate([
            plant.get_state().platform_pos_mm,
            plant.get_state().platform_rot,
        ])
        err = np.linalg.norm(final[:3] - pose2[:3])
        assert err < 15.0, f"Not near second event: error {err:.1f} mm"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
