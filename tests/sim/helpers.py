"""Shared test helpers for MPC simulation tests.

Extracted from test_mpc_dynamic.py so that multiple test files (dynamic targets,
BB sequences, etc.) can reuse the catch simulation runner.
"""

from __future__ import annotations

import numpy as np

from plant.mujoco_plant import MuJoCoPlant
from controller.mpc import MPCController
from hand.coordinator import HandCoordinator
from hand.trajectory import HandCatchSequence

CONTROL_DT = 0.025  # 40 Hz
# Idle pose just above STOW — keeps all leg extensions above the 5mm margin.
IDLE_POSE = np.array([0.0, 0.0, 10.0, 0.0, 0.0, 0.0])


def run_catch_sim(
    plant: MuJoCoPlant,
    mpc: MPCController,
    coordinator: HandCoordinator,
    duration: float,
    *,
    record_states: bool = False,
) -> dict:
    """Run a complete catch simulation and return diagnostic info.

    Parameters
    ----------
    plant, mpc, coordinator : simulation components
    duration : float
        Simulation duration in seconds.
    record_states : bool
        If True, the returned dict includes a 'states' key with a list of
        per-step dicts: {'time', 'pose', 'twist', 'ext', 'target_pose',
        'target_twist'}.

    Returns
    -------
    dict with keys:
        events: list of HandEvent
        final_pose: (6,) final platform pose
        ball_state: final BallState or None
        max_pos_error_mm: max position error during approach
        captures: number of capture events
        states: list of per-step dicts (only if record_states=True)
    """
    max_pos_err = 0.0
    captures = 0
    active_hand_seq = None
    states = [] if record_states else None

    n_steps = int(duration / CONTROL_DT)
    for step_idx in range(n_steps):
        state = plant.get_state()
        current_pose = np.concatenate([state.platform_pos_mm, state.platform_rot])
        hand_pos = state.hand_pos_mm if state.hand_pos_mm is not None else 0.0

        # Check ball spawn
        spawn = coordinator.should_spawn_ball(state.time)
        if spawn is not None:
            plant.spawn_ball(spawn.position_mm, spawn.velocity_mms)

        # Update coordinator
        target, hand_cmd = coordinator.update(
            state.time, current_pose, hand_pos_mm=hand_pos,
            plant_state=state)

        if isinstance(hand_cmd, HandCatchSequence):
            active_hand_seq = hand_cmd
        elif hand_cmd == 'prime':
            plant.hand_to_prime()
            active_hand_seq = None
        elif hand_cmd == 'home':
            plant.hand_to_home()
            active_hand_seq = None

        # Sample active hand catch trajectory
        if active_hand_seq is not None:
            pos = active_hand_seq.sample(state.time)
            if pos is not None:
                plant.command_hand(pos)
            else:
                active_hand_seq = None

        # MPC solve
        if target is not None:
            cmd, _cmd_vel, diag = mpc.solve(
                state, target.pose_6dof)
            # Track error to target
            pos_err = np.linalg.norm(current_pose[:3] - target.pose_6dof[:3])
            max_pos_err = max(max_pos_err, pos_err)
        else:
            cmd, _cmd_vel, diag = mpc.solve(state, IDLE_POSE)

        plant.command(cmd)
        plant.step(CONTROL_DT)

        # Record per-step state
        if states is not None:
            states.append({
                'time': state.time,
                'pose': current_pose.copy(),
                'twist': state.platform_twist.copy(),
                'ext': state.leg_extensions_mm.copy(),
                'target_pose': target.pose_6dof.copy() if target is not None else None,
                'target_twist': target.arrival_twist.copy() if (
                    target is not None and target.arrival_twist is not None
                ) else None,
            })

        # Check capture
        if plant.has_ball and plant.check_and_capture():
            coordinator.notify_capture(state.time)
            captures += 1

    result = {
        'events': coordinator.events,
        'final_pose': np.concatenate([
            plant.get_state().platform_pos_mm,
            plant.get_state().platform_rot,
        ]),
        'ball_state': plant.get_ball_state(),
        'max_pos_error_mm': max_pos_err,
        'captures': captures,
    }
    if states is not None:
        result['states'] = states
    return result
