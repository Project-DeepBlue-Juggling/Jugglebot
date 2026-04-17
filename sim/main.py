"""Simulation entry point — run the MuJoCo Stewart platform simulation.

For real hardware, use ``run_mpc.py`` at the repo root instead.

Usage examples:
    python sim/main.py                              # viewer, hold at active pose
    python sim/main.py --pose 0,0,50,0,0,0          # command z+50mm (direct)
    python sim/main.py --no-viewer --duration 5      # headless 5-second run

    # Sequence: pose@time  pose@time  ...  (space-separated)
    python sim/main.py --sequence "0,0,50,0,0,0@0.5  0,0,0,0,0,0@2.0"

    # MPC mode: use the NMPC controller instead of direct IK commands
    python sim/main.py --mpc --pose 0,0,50,0,0,0
    python sim/main.py --mpc --sequence "0,0,50,0,0,0@0.5 0,0,0,0,0,0@2.0"

    # Interactive input (Phase 4): MPC plans smooth motion to live target
    python sim/main.py --spacemouse                  # SpaceMouse (Linux only)
    python sim/main.py --keyboard                    # Keyboard (cross-platform)

    # Live telemetry dashboard (open http://localhost:8082 in browser)
    python sim/main.py --dashboard --mpc --pose 0,0,50,0,0,0
"""

from __future__ import annotations

import argparse
import os
import sys
import time
import datetime
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np

# Ensure sim/ is importable
_sim_dir = os.path.dirname(os.path.abspath(__file__))
if _sim_dir not in sys.path:
    sys.path.insert(0, _sim_dir)
_repo_root = os.path.dirname(_sim_dir)
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)

from plant.mujoco_plant import MuJoCoPlant
from plant.interface import PlantState
from input.sim_control import SimController
from viz.telemetry import TelemetryLogger, StepRecord, record_from_arrays

if TYPE_CHECKING:
    from hand.coordinator import BallSpawn
    from hand.trajectory import HandCatchSequence


# MPC control rate (Hz).
CONTROL_RATE_HZ = 40
CONTROL_DT = 1.0 / CONTROL_RATE_HZ

# Telemetry data age threshold for MPC deceleration (seconds).
# When motor feedback is older than this, override target to hold-in-place.
# Must be less than motor guard's MPC_CMD_STALENESS_S (0.25 s) so the MPC
# decelerates smoothly before the motor guard triggers E-STOP.
_MPC_STALE_DECEL_S = 0.125


def parse_args():
    p = argparse.ArgumentParser(description='Jugglebot MuJoCo simulation')
    p.add_argument('--no-viewer', action='store_true',
                   help='Run headless (no MuJoCo viewer)')
    p.add_argument('--mpc', action='store_true',
                   help='Use MPC controller instead of direct IK commands')
    p.add_argument('--pose', type=str, default=None,
                   help='Target pose: x,y,z,rx,ry,rz (mm, rad). '
                        'Example: 0,0,50,0,0,0')
    p.add_argument('--sequence', type=str, default=None,
                   help='Pose sequence: "x,y,z,rx,ry,rz@t ..." '
                        'Each pose is commanded at time t (seconds). '
                        'Example: "0,0,50,0,0,0@0.5 0,0,0,0,0,0@2.0"')
    p.add_argument('--duration', type=float, default=None,
                   help='Simulation duration in seconds '
                        '(default: 10, or last sequence time + 2)')
    p.add_argument('--log-dir', type=str, default=os.path.join(_repo_root, 'temp', 'logs'),
                   help='Directory for telemetry CSV output')
    p.add_argument('--trajectory', type=str, default=None,
                   help='Scripted trajectory name (T1, T2, T3, T4). '
                        'Implies --mpc. Overrides --pose/--sequence.')
    p.add_argument('--spacemouse', action='store_true',
                   help='SpaceMouse interactive input (Linux only, implies --mpc)')
    p.add_argument('--keyboard', action='store_true',
                   help='Keyboard interactive input (cross-platform, implies --mpc)')
    p.add_argument('--catch', type=str, default=None,
                   help='Scripted catch sequence (DT1..DT8, BB1..BB4). '
                        'Implies --mpc. Dynamic target + ball physics.')
    p.add_argument('--throw-catch', type=str, default=None,
                   dest='throw_catch',
                   help='Scripted throw-catch sequence (TC1..TC4). '
                        'Implies --mpc. Full throw → catch cycle.')
    p.add_argument('--interactive-catch', action='store_true',
                   dest='interactive_catch',
                   help='Interactive ball catch mode — spawn balls with B key. '
                        'Implies --mpc. Requires viewer.')
    p.add_argument('--juggle', action='store_true',
                   help='Continuous throw-catch mode. Jugglebot throws to '
                        'itself in a loop with real-time parameter adjustment. '
                        'Implies --mpc. Requires viewer.')
    p.add_argument('--bb', action='store_true',
                   help='Enable Ball Butler throws in interactive-catch mode '
                        '(T key). Requires --interactive-catch.')
    p.add_argument('--cycle-time', type=float, nargs='?', const=1.2,
                   default=None, dest='cycle_time',
                   help='Continuous toss loop mode (Phase A/B/C). Optional arg: '
                        'cycle time in seconds (default: 1.2). Implies --mpc.')
    p.add_argument('--hold-ratio', type=float, default=0.4,
                   dest='hold_ratio',
                   help='Hold ratio for --cycle-time (default: 0.4). '
                        'Fraction of cycle spent holding the ball.')
    p.add_argument('--lateral-spacing', type=float, default=0.0,
                   dest='lateral_spacing',
                   help='Lateral spacing in mm for Phase B toss between '
                        'positions (default: 0 = Phase A vertical toss).')
    p.add_argument('--platform-event-speed-ratio', type=float, default=0.0,
                   dest='platform_event_speed_ratio',
                   help='Phase C: platform velocity at throw/catch as a '
                        'fraction of average transit velocity (0..1). '
                        '0 = stop at events (Phase B), '
                        '>0 = continuous motion (Phase C). Default: 0.')
    p.add_argument('--hardware', action='store_true',
                   help='Use real hardware via HardwarePlant. Implies --mpc '
                        '--no-viewer. Without --pose/--sequence, receives '
                        'targets from mpc_bridge_node via ZMQ :5558.')
    p.add_argument('--dashboard', action='store_true',
                   help='Start live telemetry dashboard (web browser)')
    p.add_argument('--dashboard-port', type=int, default=8082,
                   help='Dashboard server port (default: 8082)')
    return p.parse_args()


def _parse_pose(pose_str: str) -> np.ndarray:
    """Parse comma-separated pose string into (6,) array."""
    vals = [float(v) for v in pose_str.split(',')]
    if len(vals) != 6:
        raise ValueError(f"Pose must have 6 values, got {len(vals)}: {pose_str}")
    return np.array(vals)


def _parse_sequence(seq_str: str) -> list[tuple[float, np.ndarray]]:
    """Parse 'pose@time pose@time ...' into sorted [(time, pose), ...]."""
    entries = seq_str.strip().split()
    schedule: list[tuple[float, np.ndarray]] = []

    for entry in entries:
        if '@' not in entry:
            raise ValueError(f"Sequence entry must be 'pose@time', got: {entry}")
        pose_str, time_str = entry.rsplit('@', 1)
        schedule.append((float(time_str), _parse_pose(pose_str)))

    schedule.sort(key=lambda x: x[0])
    return schedule


def _pose_6dof_from_state(state: PlantState) -> np.ndarray:
    """Extract [x,y,z,rx,ry,rz] from a PlantState."""
    return np.concatenate([state.platform_pos_mm, state.platform_rot])


# ---------------------------------------------------------------------------
# Direct-command mode (original Phase 1 path)
# ---------------------------------------------------------------------------

class PoseScheduler:
    """Tracks which pose to command based on simulation time.

    Given a sorted list of (time, pose) pairs, returns the current target
    pose and commanded extensions for any given time.
    """

    def __init__(self, schedule: list[tuple[float, np.ndarray]], plant: MuJoCoPlant):
        self._schedule = schedule
        self._plant = plant
        self._current_idx = -1
        self._target_pose = np.zeros(6)
        self._cmd_extensions = plant.pose_to_extensions(self._target_pose)

    def update(self, sim_time: float) -> bool:
        """Advance to the correct pose for the given time.

        Returns True if the pose changed this call.
        """
        new_idx = self._current_idx
        for i, (t, _) in enumerate(self._schedule):
            if sim_time >= t:
                new_idx = i
            else:
                break

        if new_idx != self._current_idx:
            self._current_idx = new_idx
            self._target_pose = self._schedule[new_idx][1]
            self._cmd_extensions = self._plant.pose_to_extensions(self._target_pose)
            self._plant.command(self._cmd_extensions)
            return True
        return False

    @property
    def target_pose(self) -> np.ndarray:
        return self._target_pose

    @property
    def cmd_extensions(self) -> np.ndarray:
        return self._cmd_extensions


def _log_step(logger: TelemetryLogger, state: PlantState,
              scheduler: PoseScheduler, dashboard=None) -> None:
    """Record one telemetry step (direct-command mode)."""
    record = record_from_arrays(
        time=state.time,
        ref_pose=scheduler.target_pose,
        ref_twist=np.zeros(6),
        actual_pose=_pose_6dof_from_state(state),
        actual_twist=state.platform_twist,
        cmd_extensions=scheduler.cmd_extensions,
        actual_extensions=state.leg_extensions_mm,
        leg_velocities=state.leg_velocities_mmps,
    )
    logger.append(record)
    if dashboard is not None:
        dashboard.broadcast(record)


def _log_mpc_step(logger: TelemetryLogger, state: PlantState,
                  ref_pose: np.ndarray, cmd_ext: np.ndarray,
                  diag: dict, ref_twist: np.ndarray | None = None,
                  dashboard=None,
                  hand_cmd_mm: float = 0.0,
                  overhead_ms: float = 0.0,
                  fk_iterations: int = 0,
                  ff_torque_max_Nm: float = 0.0,
                  ipopt_iter: int = 0,
                  t_ref_s: float = 0.0) -> None:
    """Record one telemetry step (MPC mode)."""
    record = record_from_arrays(
        time=state.time,
        ref_pose=ref_pose,
        ref_twist=ref_twist if ref_twist is not None else np.zeros(6),
        actual_pose=_pose_6dof_from_state(state),
        actual_twist=state.platform_twist,
        cmd_extensions=cmd_ext,
        actual_extensions=state.leg_extensions_mm,
        leg_velocities=state.leg_velocities_mmps,
        hand_cmd_mm=hand_cmd_mm,
        hand_pos_mm=state.hand_pos_mm if state.hand_pos_mm is not None else 0.0,
        hand_vel_mmps=state.hand_vel_mmps if state.hand_vel_mmps is not None else 0.0,
        solve_time_ms=diag.get('solve_time_ms', 0.0),
        solve_status=diag.get('status', 'n/a'),
        cost=diag.get('cost', 0.0),
        constraint_violation=diag.get('constraint_violation', 0.0),
        overhead_ms=overhead_ms,
        fk_iterations=fk_iterations,
        ff_torque_max_Nm=ff_torque_max_Nm,
        ipopt_iter=diag.get('iter_count', 0),
        t_ref_s=t_ref_s,
    )
    logger.append(record)
    if dashboard is not None:
        dashboard.broadcast(record)


# ---------------------------------------------------------------------------
# TargetCommand + TargetSource adapters (Phase 4)
# ---------------------------------------------------------------------------
# Base TargetCommand (MPC-level, no sim deps) and TargetSource protocol live in
# controller/target.py.  The sim extends TargetCommand with hand/ball fields.

from controller.target import TargetCommand as _BaseTargetCommand  # noqa: E402
from controller.target import TargetSource  # noqa: E402  — re-exported for sim consumers
from controller.target import flat_target_to_events  # noqa: E402


@dataclass
class TargetCommand(_BaseTargetCommand):
    """Sim-extended target command with hand and ball-spawn fields.

    Inherits MPC-level fields (target_pose, arrival_time, target_twist) from
    ``controller.target.TargetCommand``.  Adds sim-specific fields that the
    control loop handles outside the MPC solver.
    """
    hand_cmd: str | float | HandCatchSequence | None = None
    ball_spawn: BallSpawn | None = None


from controller.target import StaticTargetSource, WaypointTargetSource  # noqa: E402


class InteractiveTargetSource:
    """Adapts SpaceMouseInput / KeyboardInput to the TargetSource protocol."""

    def __init__(self, input_source, control_dt: float):
        self._src = input_source
        self._dt = control_dt

    def update(self, sim_time: float, state: PlantState) -> TargetCommand:
        if hasattr(self._src, 'apply'):
            self._src.apply(self._dt)
        target = self._src.read()
        return TargetCommand(
            target_pose=target,
            ref_events=flat_target_to_events(
                _pose_6dof_from_state(state), state.platform_twist,
                target, sim_time),
        )

    @property
    def key_callback(self):
        return getattr(self._src, 'key_callback', None)

    def close(self):
        if hasattr(self._src, 'close'):
            self._src.close()


class CatchTargetSource:
    """Adapts a scripted catch sequence to the TargetSource protocol.

    Wraps HandCoordinator and handles ball spawning, hand commands,
    and capture notification.  Supports ``reset()`` for looping in the
    viewer.
    """

    def __init__(self, catch_sequence, feasibility_checker=None,
                 active_pose: np.ndarray | None = None):
        self._sequence = catch_sequence
        self._feasibility = feasibility_checker
        self._active_pose = active_pose
        self._coord = None  # type: ignore[assignment]
        self._build_coordinator()

    def _build_coordinator(self):
        from hand.coordinator import HandCoordinator
        self._coord = HandCoordinator(
            active_pose=self._active_pose,
            feasibility_checker=self._feasibility)
        for target, ball in self._sequence:
            self._coord.submit_target(target, ball)

    def reset(self):
        self._build_coordinator()

    def update(self, sim_time: float, state: PlantState) -> TargetCommand:
        current_pose = _pose_6dof_from_state(state)
        hand_pos = state.hand_pos_mm if state.hand_pos_mm is not None else 0.0

        spawn = self._coord.should_spawn_ball(sim_time)
        target, hand_cmd = self._coord.update(
            sim_time, current_pose, hand_pos_mm=hand_pos, plant_state=state)

        # Hold current pose when coordinator returns None (B-04: avoids
        # abrupt home reference step discontinuity).
        if target is not None:
            pose = target.pose_6dof
            arrival = target.arrival_time
            twist = target.arrival_twist
            ref_events = flat_target_to_events(
                current_pose, state.platform_twist, pose, sim_time,
                target_twist=twist, arrival_time=arrival)
        else:
            pose = current_pose.copy()
            arrival = None
            twist = None
            ref_events = None

        return TargetCommand(
            target_pose=pose,
            arrival_time=arrival,
            target_twist=twist,
            ref_events=ref_events,
            hand_cmd=hand_cmd,
            ball_spawn=spawn,
        )

    def notify_capture(self, sim_time: float):
        self._coord.notify_capture(sim_time)

    def print_summary(self):
        _print_catch_summary(self._coord)


class ThrowCatchTargetSource:
    """Adapts a ThrowCatchPlan to the TargetSource protocol.

    Manages the ball lifecycle: spawns ball in hand at start, releases
    at throw time via BallRelease, catches via standard capture.
    """

    def __init__(self, plan, active_pose: np.ndarray | None = None):
        from hand.planner import ThrowCatchPlan
        self._plan = plan
        self._active_pose = active_pose
        self._coord = None
        self._ball_spawned_in_hand = False
        self._build_coordinator()

    def _build_coordinator(self):
        from hand.coordinator import HandCoordinator
        self._coord = HandCoordinator(active_pose=self._active_pose)
        self._coord.submit_throw_catch(self._plan)
        self._ball_spawned_in_hand = False

    def reset(self):
        self._build_coordinator()

    def update(self, sim_time: float, state: PlantState) -> TargetCommand:
        current_pose = _pose_6dof_from_state(state)
        hand_pos = state.hand_pos_mm if state.hand_pos_mm is not None else 0.0

        target, hand_cmd = self._coord.update(
            sim_time, current_pose, hand_pos_mm=hand_pos, plant_state=state)

        # Spawn ball in hand at simulation start (before throw begins)
        ball_spawn = None
        if not self._ball_spawned_in_hand and sim_time > 0.1:
            self._ball_spawned_in_hand = True
            # Signal to spawn ball in hand (special marker)
            ball_spawn = 'spawn_in_hand'

        # Hold current pose when coordinator returns None (B-04)
        if target is not None:
            pose = target.pose_6dof
            arrival = target.arrival_time
            twist = target.arrival_twist
            ref_events = flat_target_to_events(
                current_pose, state.platform_twist, pose, sim_time,
                target_twist=twist, arrival_time=arrival)
        else:
            pose = current_pose.copy()
            arrival = None
            twist = None
            ref_events = None

        return TargetCommand(
            target_pose=pose,
            arrival_time=arrival,
            target_twist=twist,
            ref_events=ref_events,
            hand_cmd=hand_cmd,
            ball_spawn=ball_spawn,
        )

    def notify_capture(self, sim_time: float):
        self._coord.notify_capture(sim_time)

    def print_summary(self):
        _print_catch_summary(self._coord)


class InteractiveCatchSource:
    """Adapts InteractiveCatchController to the TargetSource protocol.

    Exposes pause/speed/key_callback/render for the unified viewer loop.
    """

    def __init__(self, controller, active_pose: np.ndarray | None = None):
        self._ctrl = controller
        # active_pose stored for future use; controller handles its own ready pose

    def update(self, sim_time: float, state: PlantState) -> TargetCommand:
        current_pose = _pose_6dof_from_state(state)
        hand_pos = state.hand_pos_mm if state.hand_pos_mm is not None else 0.0
        target, hand_cmd, ball_spawn = self._ctrl.update(
            sim_time, current_pose, hand_pos, plant_state=state)

        # Hold current pose when controller returns None (B-04)
        if target is not None:
            pose = target.pose_6dof
            arrival = target.arrival_time
            twist = target.arrival_twist
            ref_events = flat_target_to_events(
                current_pose, state.platform_twist, pose, sim_time,
                target_twist=twist, arrival_time=arrival)
        else:
            pose = current_pose.copy()
            arrival = None
            twist = None
            ref_events = None

        return TargetCommand(
            target_pose=pose,
            arrival_time=arrival,
            target_twist=twist,
            ref_events=ref_events,
            hand_cmd=hand_cmd,
            ball_spawn=ball_spawn,
        )

    def should_step(self) -> bool:
        return self._ctrl.should_step()

    @property
    def sleep_factor(self) -> float:
        return self._ctrl.sleep_factor

    @property
    def key_callback(self):
        return self._ctrl.key_callback

    def render(self, viewer):
        self._ctrl.render_preview(viewer)

    def notify_capture(self, sim_time: float):
        self._ctrl.notify_capture(sim_time)

    def close(self):
        pass  # no cleanup needed


class ContinuousThrowCatchSource:
    """Adapts ContinuousThrowCatchController to the TargetSource protocol.

    Exposes pause/speed/key_callback/render for the unified viewer loop.
    """

    def __init__(self, controller):
        self._ctrl = controller

    def update(self, sim_time: float, state: PlantState) -> TargetCommand:
        current_pose = _pose_6dof_from_state(state)
        hand_pos = state.hand_pos_mm if state.hand_pos_mm is not None else 0.0
        result = self._ctrl.update(
            sim_time, current_pose, hand_pos, plant_state=state)

        # Support both 3-tuple (legacy controllers) and 4-tuple (with
        # ref_events for multi-event MPC lookahead).
        if len(result) >= 4:
            target, hand_cmd, ball_spawn, ref_events = result[:4]
        else:
            target, hand_cmd, ball_spawn = result[:3]
            ref_events = None

        # Hold current pose when controller returns None (B-04)
        if target is not None:
            pose = target.pose_6dof
            arrival = target.arrival_time
            twist = target.arrival_twist
        else:
            pose = current_pose.copy()
            arrival = None
            twist = None
            ref_events = None  # no events when holding

        return TargetCommand(
            target_pose=pose,
            arrival_time=arrival,
            target_twist=twist,
            ref_events=ref_events,
            boost_vel_weights=ref_events is not None,
            hand_cmd=hand_cmd,
            ball_spawn=ball_spawn,
        )

    def should_step(self) -> bool:
        return self._ctrl.should_step()

    @property
    def sleep_factor(self) -> float:
        return self._ctrl.sleep_factor

    @property
    def key_callback(self):
        return self._ctrl.key_callback

    def render(self, viewer):
        self._ctrl.render_preview(viewer)

    def notify_capture(self, sim_time: float):
        self._ctrl.notify_capture(sim_time)

    def print_summary(self):
        self._ctrl.print_summary()

    def close(self):
        pass


class ScheduledCatchSource:
    """Adapts ScheduledCoordinator to the TargetSource protocol.

    Wraps the EventScheduler-based coordinator for use in the unified
    viewer/control loop.
    """

    def __init__(self, coordinator):
        from hand.scheduled_coordinator import ScheduledCoordinator
        self._coord: ScheduledCoordinator = coordinator

    def update(self, sim_time: float, state: PlantState) -> TargetCommand:
        current_pose = _pose_6dof_from_state(state)
        current_twist = state.platform_twist
        hand_pos = state.hand_pos_mm if state.hand_pos_mm is not None else 0.0

        tc, hand_cmd, ball_spawn = self._coord.update(
            sim_time, current_pose, current_twist, hand_pos_mm=hand_pos)

        return TargetCommand(
            target_pose=tc.target_pose,
            arrival_time=tc.arrival_time,
            target_twist=tc.target_twist,
            ref_events=tc.ref_events,
            boost_vel_weights=tc.ref_events is not None,
            hand_cmd=hand_cmd,
            ball_spawn=ball_spawn,
        )

    def notify_capture(self, sim_time: float):
        self._coord.notify_capture(sim_time)

    def close(self):
        pass


def _combine_key_callbacks(*callbacks):
    """Create a single key callback that dispatches to multiple handlers."""
    active = [cb for cb in callbacks if cb is not None]
    if not active:
        return None
    if len(active) == 1:
        return active[0]

    def combined(keycode):
        for cb in active:
            cb(keycode)
    return combined


# ---------------------------------------------------------------------------
# Direct-command loops
# ---------------------------------------------------------------------------

def run_headless(plant: MuJoCoPlant, schedule: list[tuple[float, np.ndarray]],
                 duration: float, logger: TelemetryLogger,
                 dashboard=None) -> None:
    """Run the simulation loop without a viewer (direct-command mode)."""
    scheduler = PoseScheduler(schedule, plant)
    n_steps = int(duration / CONTROL_DT)

    for _ in range(n_steps):
        plant.step(CONTROL_DT)
        state = plant.get_state()
        scheduler.update(state.time)
        _log_step(logger, state, scheduler, dashboard)

    logger.flush()
    final = plant.get_state()
    print(f"Final pose: pos={final.platform_pos_mm} mm, "
          f"rot={np.degrees(final.platform_rot)} deg")
    if logger.records:
        print(f"Tracking error: {logger.records[-1].tracking_error_mm:.3f} mm, "
              f"{logger.records[-1].tracking_error_deg:.4f} deg")


def run_with_viewer(plant: MuJoCoPlant, schedule: list[tuple[float, np.ndarray]],
                    duration: float, logger: TelemetryLogger,
                    dashboard=None) -> None:
    """Run the simulation loop with the MuJoCo passive viewer (direct-command)."""
    import mujoco.viewer

    scheduler = PoseScheduler(schedule, plant)

    with mujoco.viewer.launch_passive(plant.model, plant.data) as viewer:
        start_wall = time.monotonic()
        sim_time_target = 0.0

        while viewer.is_running() and sim_time_target < duration:
            plant.step(CONTROL_DT)
            sim_time_target += CONTROL_DT

            state = plant.get_state()
            scheduler.update(state.time)
            _log_step(logger, state, scheduler, dashboard)

            viewer.sync()

            # Real-time pacing
            elapsed_wall = time.monotonic() - start_wall
            sleep_time = sim_time_target - elapsed_wall
            if sleep_time > 0:
                time.sleep(sleep_time)

    logger.flush()
    if logger.records:
        final_rec = logger.records[-1]
        print(f"Final tracking error: {final_rec.tracking_error_mm:.3f} mm, "
              f"{final_rec.tracking_error_deg:.4f} deg")


# ---------------------------------------------------------------------------
# Unified MPC loops (Phase 4)
# ---------------------------------------------------------------------------

def _execute_hand_cmd(plant, hand_cmd, active_hand_seq, last_hand_cmd_mm,
                      sim_time):
    """Process a hand command from a TargetCommand.

    Returns (active_hand_seq, last_hand_cmd_mm) — updated state.
    """
    from hand.trajectory import HandCatchSequence, HandThrowSequence
    from hand.coordinator import BallRelease

    if isinstance(hand_cmd, BallRelease):
        # Release the ball with specified velocity
        if hasattr(plant, 'ball_manager') and plant.ball_manager is not None:
            plant.ball_manager.release(hand_cmd.velocity_mms)
    elif isinstance(hand_cmd, (HandCatchSequence, HandThrowSequence)):
        active_hand_seq = hand_cmd
    elif isinstance(hand_cmd, (int, float)):
        plant.command_hand(float(hand_cmd))
        last_hand_cmd_mm = float(hand_cmd)
        active_hand_seq = None
    elif hand_cmd == 'prime':
        plant.hand_to_prime()
        active_hand_seq = None
    elif hand_cmd == 'home':
        plant.hand_to_home()
        active_hand_seq = None

    # Sample active hand sequence (catch or throw)
    if active_hand_seq is not None:
        pos = active_hand_seq.sample(sim_time)
        if pos is not None:
            plant.command_hand(pos)
            last_hand_cmd_mm = pos
        else:
            active_hand_seq = None

    return active_hand_seq, last_hand_cmd_mm


def _mpc_solve(mpc, state, tc: TargetCommand, t_now: float | None = None):
    """Call mpc.solve() with the target from a TargetCommand.

    Returns (cmd, cmd_vel, diag, ref_pose, ref_twist).

    ref_pose and ref_twist come from the MPC's own node-0 reference
    (quintic Hermite between events).  This ensures telemetry tracking
    error is measured against the reference the MPC actually optimized
    against.
    """
    cmd, cmd_vel, diag = mpc.solve(
        state, tc.target_pose,
        ref_events=tc.ref_events,
        boost_vel_weights=tc.boost_vel_weights,
        t_now=t_now,
    )

    # Use the MPC's own reference for telemetry.
    mpc_ref = mpc.last_ref_traj
    mpc_twist = mpc.last_twist_traj
    if mpc_ref is not None:
        ref_pose = mpc_ref[0]
        ref_twist = mpc_twist[0] if mpc_twist is not None else np.zeros(6)
    else:
        ref_pose = tc.target_pose
        ref_twist = np.zeros(6)

    return cmd, cmd_vel, diag, ref_pose, ref_twist


def _send_target_feedback(feedback_pub, source, tc, diag, last_arrival):
    """Publish accept/reject feedback for catch targets.  Returns updated last_arrival.

    Only sends when the source is 'catch' and the arrival_time has shifted by
    > 50 ms since the last feedback.  This deduplicates feedback across Kalman
    refinement updates (which produce many targets per ball with slightly
    different arrival_times) while still responding to genuinely new targets.
    """
    source_id = getattr(source, 'source', '')
    if source_id != 'catch' or tc.arrival_time is None:
        return last_arrival

    # Deduplicate: only send when arrival_time shifts meaningfully
    if last_arrival is not None and abs(tc.arrival_time - last_arrival) < 0.05:
        return last_arrival

    from jugglebot.motion.ipc import make_target_feedback

    accepted = diag.get('status', '') in (
        'Solve_Succeeded', 'Solved_To_Acceptable_Level')
    violations = None if accepted else [diag.get('status', 'unknown')]
    feedback_pub.send(make_target_feedback(
        tc.arrival_time, accepted, 'catch', violations))
    return tc.arrival_time


def run_mpc_headless(plant, mpc, source, duration: float,
                     logger: TelemetryLogger, dashboard=None,
                     feedback_pub=None) -> None:
    """Unified headless MPC loop — delegates to controller.runner.

    Paces iterations to wall-clock CONTROL_DT so the MPC's horizon
    predictions match real elapsed time.  If a solve overruns the budget,
    the next iteration runs immediately (no accumulated debt).

    Parameters
    ----------
    source : TargetSource
        Any object with ``update(sim_time, state) -> TargetCommand``.
        May optionally provide ``notify_capture(sim_time)`` and
        ``print_summary()``.
    feedback_pub : TargetFeedbackPub or None
        If provided, publishes accept/reject feedback for catch targets
        on ZMQ :5559.  Only relevant in hardware mode.
    """
    from controller.runner import run_mpc_loop, MpcLoopHooks
    from controller.target import TargetCommand as BaseTargetCommand

    # --- Sim-specific state captured by closures ---
    _sim_state = {'active_hand_seq': None, 'last_hand_cmd_mm': 0.0}
    _fb_last_arrival = [None]  # mutable container for closure

    # --- Hook: target override (stale telemetry → hold-in-place) ---
    def _on_target_override(state, tc):
        if (state.data_age_s is not None
                and state.data_age_s > _MPC_STALE_DECEL_S):
            return BaseTargetCommand(
                target_pose=np.concatenate(
                    [state.platform_pos_mm, state.platform_rot]),
            )

        # Ball spawning (sim-only)
        if getattr(tc, 'ball_spawn', None) is not None:
            if tc.ball_spawn == 'spawn_in_hand':
                if (hasattr(plant, 'ball_manager')
                        and plant.ball_manager is not None):
                    plant.ball_manager.spawn_in_hand()
            else:
                if (hasattr(plant, 'ball_manager')
                        and plant.ball_manager is not None):
                    plant.ball_manager.reset()
                plant.spawn_ball(tc.ball_spawn.position_mm,
                                 tc.ball_spawn.velocity_mms)

        # Hand commands (sim-only)
        hand_cmd = getattr(tc, 'hand_cmd', None)
        _sim_state['active_hand_seq'], _sim_state['last_hand_cmd_mm'] = (
            _execute_hand_cmd(
                plant, hand_cmd,
                _sim_state['active_hand_seq'],
                _sim_state['last_hand_cmd_mm'],
                state.time))

        return tc

    # --- Hook: post-solve (target feedback for catch coordinator) ---
    def _on_post_solve(tc, diag):
        if feedback_pub is not None:
            _fb_last_arrival[0] = _send_target_feedback(
                feedback_pub, source, tc, diag, _fb_last_arrival[0])

    # --- Hook: pre-command (feedforward torques for HardwarePlant) ---
    def _on_pre_command(plant_, mpc_, tc, cmd, cmd_vel, diag):
        if hasattr(plant_, 'set_pose'):
            poses = mpc_.predicted_poses_view
            times = mpc_.predicted_times_view
            if poses is not None:
                dt0 = times[1] - times[0]
                dt1 = times[2] - times[1]
                twist = (poses[1] - poses[0]) / dt0
                twist_next = (poses[2] - poses[1]) / dt1
                accel = (twist_next - twist) / (0.5 * (dt0 + dt1))
                plant_.set_pose(poses[0], twist_6dof=twist, accel_6dof=accel)

    # --- Hook: post-step (ball capture, sim-only) ---
    def _on_post_step(state, sim_time):
        if hasattr(plant, 'has_ball') and plant.has_ball and plant.check_and_capture():
            if hasattr(source, 'notify_capture'):
                source.notify_capture(sim_time)

    # --- Hook: log extras ---
    def _on_log_extras(plant_):
        return {
            'hand_cmd_mm': _sim_state['last_hand_cmd_mm'],
            'fk_iterations': getattr(plant_, 'last_fk_iterations', 0),
            'ff_torque_max_Nm': getattr(plant_, 'last_ff_torque_max_Nm', 0.0),
        }

    hooks = MpcLoopHooks(
        on_target_override=_on_target_override,
        on_pre_command=_on_pre_command,
        on_post_solve=_on_post_solve,
        on_post_step=_on_post_step,
        on_log_extras=_on_log_extras,
    )

    run_mpc_loop(
        plant, mpc, source, duration, logger,
        control_dt=CONTROL_DT, dashboard=dashboard, hooks=hooks,
    )


def run_mpc_with_viewer(plant: MuJoCoPlant, mpc, source, duration: float,
                        logger: TelemetryLogger, dashboard=None) -> None:
    """Unified MPC loop with the MuJoCo passive viewer.

    Supports pause/step/speed control, optional custom rendering, and
    automatic looping when the source has a ``reset()`` method.

    Parameters
    ----------
    source : TargetSource
        Any object with ``update(sim_time, state) -> TargetCommand``.
        Optional attributes checked at runtime:
        - ``should_step() -> bool`` + ``sleep_factor -> float``: pause/speed
        - ``key_callback(keycode)``: viewer keyboard events
        - ``render(viewer)``: custom per-frame rendering (e.g. spawn preview)
        - ``notify_capture(sim_time)``: ball capture notification
        - ``reset()``: enables looping (viewer mode only)
        - ``print_summary()``: called after each run
    """
    import mujoco.viewer
    from viz.horizon import HorizonRenderer

    horizon = HorizonRenderer(plant.geom.init_height_mm)

    # Sim control: use source's if available, else create default
    has_source_ctrl = (hasattr(source, 'should_step')
                       and hasattr(source, 'sleep_factor'))
    sim_ctrl = None if has_source_ctrl else SimController()

    # Combine key callbacks: source's keys + sim control's pause/speed
    source_kc = getattr(source, 'key_callback', None)
    ctrl_kc = sim_ctrl.key_callback if sim_ctrl is not None else None
    combined_kc = _combine_key_callbacks(source_kc, ctrl_kc)

    can_loop = hasattr(source, 'reset')
    has_render = hasattr(source, 'render')

    print("\n  Controls:  Space=pause  Right=step  Up/Down=speed  R=reset speed\n")

    with mujoco.viewer.launch_passive(
        plant.model, plant.data, key_callback=combined_kc
    ) as viewer:
        iteration = 0

        while viewer.is_running():
            iteration += 1
            if iteration > 1:
                if not can_loop:
                    break
                source.reset()
                plant.reset()
                mpc.reset()

            active_hand_seq = None
            last_hand_cmd_mm = 0.0
            wall_budget = 0.0
            # Reference clock — gated by solver success.  See controller/runner.py
            # for the rationale (mirror logic kept in sync here).
            t_ref: float | None = None
            _FALLBACK_KEYWORDS = ('fallback', 'hold', 'cold_hold')
            start_wall = time.monotonic()

            while viewer.is_running():
                # Pause gate
                should_step = (source.should_step() if has_source_ctrl
                               else sim_ctrl.should_step())
                if not should_step:
                    start_wall = time.monotonic()
                    wall_budget = 0.0
                    viewer.sync()
                    time.sleep(0.01)
                    continue

                _t_overhead = time.perf_counter()
                state = plant.get_state()

                # Check sim-time duration
                if state.time >= duration:
                    break

                if t_ref is None:
                    t_ref = state.time
                tc = source.update(t_ref, state)

                # Ball spawning (sim-only — ZmqTargetSource has no ball_spawn)
                ball_spawn = getattr(tc, 'ball_spawn', None)
                if ball_spawn is not None:
                    if ball_spawn == 'spawn_in_hand':
                        if (hasattr(plant, 'ball_manager')
                                and plant.ball_manager is not None):
                            plant.ball_manager.spawn_in_hand()
                    else:
                        if (hasattr(plant, 'ball_manager')
                                and plant.ball_manager is not None):
                            plant.ball_manager.reset()
                        plant.spawn_ball(ball_spawn.position_mm,
                                         ball_spawn.velocity_mms)

                # Hand commands (sim-only — ZmqTargetSource has no hand_cmd)
                hand_cmd = getattr(tc, 'hand_cmd', None)
                active_hand_seq, last_hand_cmd_mm = _execute_hand_cmd(
                    plant, hand_cmd, active_hand_seq, last_hand_cmd_mm,
                    state.time)

                # MPC solve (t_ref = frozen ref clock; state.time = physics clock)
                cmd, cmd_vel, diag, ref_pose, ref_twist = _mpc_solve(
                    mpc, state, tc, t_now=t_ref)
                plant.command(cmd, vel_mm_s=cmd_vel)
                plant.step(CONTROL_DT)

                _overhead_ms = ((time.perf_counter() - _t_overhead) * 1000.0
                                - diag.get('solve_time_ms', 0.0))

                # Ball capture
                if plant.has_ball and plant.check_and_capture():
                    if hasattr(source, 'notify_capture'):
                        source.notify_capture(state.time)

                # Log (record t_ref used for THIS solve, pre-advance)
                _log_mpc_step(logger, state, ref_pose, cmd, diag,
                              ref_twist=ref_twist, dashboard=dashboard,
                              hand_cmd_mm=last_hand_cmd_mm,
                              overhead_ms=_overhead_ms,
                              fk_iterations=getattr(plant, 'last_fk_iterations', 0),
                              ff_torque_max_Nm=getattr(plant, 'last_ff_torque_max_Nm', 0.0),
                              t_ref_s=t_ref)

                # Advance t_ref only on success-class solver status.
                status = diag.get('status', '')
                if not any(kw in status for kw in _FALLBACK_KEYWORDS):
                    t_ref += CONTROL_DT

                # Render
                horizon.update(mpc.predicted_poses_view, mpc.predicted_times_view)
                horizon.render(viewer)
                if has_render:
                    source.render(viewer)
                viewer.sync()

                # Real-time pacing (accumulated budget approach)
                sleep_fac = (source.sleep_factor if has_source_ctrl
                             else sim_ctrl.sleep_factor)
                wall_budget += CONTROL_DT * sleep_fac
                elapsed = time.monotonic() - start_wall
                sleep_time = wall_budget - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif sleep_time < -0.5:
                    # Fallen behind (e.g., speed change) — resync
                    start_wall = time.monotonic()
                    wall_budget = 0.0

            # End-of-iteration summary
            if hasattr(source, 'print_summary'):
                if can_loop:
                    print(f"\n--- Iteration {iteration} ---")
                source.print_summary()

    logger.flush()
    _print_mpc_summary(logger)


def _print_catch_summary(coordinator) -> None:
    """Print summary of catch events."""
    events = coordinator.events
    if not events:
        print("No catch events recorded.")
        return
    for ev in events:
        status = "CAUGHT" if ev.captured else ev.phase.name
        print(f"  Target {ev.target_idx}: {status} at t={ev.time:.3f}s, "
              f"pos_err={ev.arrival_error_mm:.1f} mm, "
              f"timing_err={ev.arrival_timing_error_s*1000:.0f} ms")


def _print_mpc_summary(logger: TelemetryLogger) -> None:
    """Print summary statistics from an MPC run."""
    if not logger.records:
        return
    final = logger.records[-1]
    solve_times = [r.solve_time_ms for r in logger.records if r.solve_time_ms > 0]
    print(f"Final tracking error: {final.tracking_error_mm:.3f} mm, "
          f"{final.tracking_error_deg:.4f} deg")
    if solve_times:
        print(f"Solve time: mean={np.mean(solve_times):.1f} ms, "
              f"max={np.max(solve_times):.1f} ms, "
              f"p95={np.percentile(solve_times, 95):.1f} ms")

    # Overhead verification diagnostics
    overhead = [r.overhead_ms for r in logger.records if r.overhead_ms > 0]
    if overhead:
        print(f"Non-solve overhead: median={np.median(overhead):.1f} ms, "
              f"p95={np.percentile(overhead, 95):.1f} ms")
    fk_iters = [r.fk_iterations for r in logger.records if r.fk_iterations > 0]
    if fk_iters:
        print(f"FK iterations: mean={np.mean(fk_iters):.1f}, "
              f"max={np.max(fk_iters)}")
    ff_torques = [r.ff_torque_max_Nm for r in logger.records
                  if r.ff_torque_max_Nm > 0]
    if ff_torques:
        print(f"FF torque max: mean={np.mean(ff_torques):.3f} Nm, "
              f"max={np.max(ff_torques):.3f} Nm")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    args = parse_args()

    # ---- Mode selection → TargetSource + label + default_duration ----

    source = None            # TargetSource (set below for MPC modes)
    schedule = None          # pose schedule (direct-command only)
    needs_viewer = False     # True if mode requires viewer
    needs_feasibility = False
    needs_high_vel = False   # raise MPC velocity limit
    catch_sequence = None    # raw sequence for CatchTargetSource
    throw_catch_plan = None  # ThrowCatchPlan for ThrowCatchTargetSource

    if args.cycle_time is not None:
        args.mpc = True
        needs_viewer = not args.no_viewer  # allow headless for testing
        needs_high_vel = True
        spacing = args.lateral_spacing
        if args.platform_event_speed_ratio > 0 and spacing > 0:
            phase = "C"
        elif spacing > 0:
            phase = "B"
        else:
            phase = "A"
        label = (f"Toss loop phase {phase} "
                 f"(cycle={args.cycle_time}s, hold={args.hold_ratio})")
        default_duration = 600.0

    elif args.juggle:
        args.mpc = True
        needs_viewer = True
        needs_high_vel = True
        label = "Continuous throw-catch (juggle)"
        default_duration = 600.0

    elif args.interactive_catch:
        args.mpc = True
        needs_viewer = True
        needs_feasibility = True
        needs_high_vel = True
        label = "Interactive catch"
        default_duration = 600.0

    elif args.throw_catch:
        args.mpc = True
        needs_high_vel = True
        from input.scripted import get_throw_catch_sequence
        throw_catch_plan, default_duration = get_throw_catch_sequence(args.throw_catch)
        label = f"Throw-catch: {args.throw_catch}"

    elif args.catch:
        args.mpc = True
        needs_feasibility = True
        needs_high_vel = True
        from input.scripted import get_catch_sequence
        catch_sequence, default_duration = get_catch_sequence(args.catch)
        label = f"Catch: {args.catch}"

    elif args.spacemouse:
        args.mpc = True
        needs_viewer = True
        needs_high_vel = True
        label = "SpaceMouse interactive"
        default_duration = 300.0

    elif args.keyboard:
        args.mpc = True
        needs_viewer = True
        needs_high_vel = True
        label = "Keyboard interactive"
        default_duration = 300.0

    elif args.trajectory:
        args.mpc = True
        needs_high_vel = True
        from input.scripted import get_trajectory
        waypoints, default_duration = get_trajectory(args.trajectory)
        label = f"Trajectory: {args.trajectory}"

    elif args.sequence:
        schedule = _parse_sequence(args.sequence)
        label = f"Sequence: {len(schedule)} poses"
        default_duration = schedule[-1][0] + 2.0

    elif args.pose:
        target = _parse_pose(args.pose)
        schedule = [(0.0, target)]
        label = f"Static pose: {target}"
        default_duration = 10.0

    elif args.hardware:
        # Hardware mode with no explicit target: ROS2 targets via ZMQ.
        # Source is created later in the source-selection chain.
        label = "Hardware MPC (ZMQ targets from mpc_bridge_node)"
        default_duration = 86400.0  # 24h — runs until Ctrl-C

    else:
        schedule = [(0.0, np.zeros(6))]
        label = "Active pose (hold)"
        default_duration = 10.0

    duration = args.duration if args.duration is not None else default_duration

    # Viewer requirement check
    if needs_viewer and args.no_viewer:
        print(f"ERROR: this mode requires the viewer (remove --no-viewer)")
        sys.exit(1)

    # Hardware mode implies MPC + headless
    if args.hardware:
        import warnings
        warnings.warn(
            "sim/main.py --hardware is deprecated. "
            "Use 'python run_mpc.py' instead.",
            DeprecationWarning, stacklevel=1)
        args.mpc = True
        args.no_viewer = True

    mode = "MPC" if args.mpc else "Direct"
    plant_label = "HARDWARE" if args.hardware else "sim"
    print(f"{label} [{mode}, {plant_label}]")
    if schedule is not None:
        for t, pose in schedule:
            print(f"  t={t:.1f}s -> [{', '.join(f'{v:.2f}' for v in pose)}]")
    print(f"Duration: {duration:.1f}s, Control rate: {CONTROL_RATE_HZ} Hz")

    # Create plant
    feedback_pub = None
    if args.hardware:
        from controller.hardware_plant import HardwarePlant
        plant = HardwarePlant(control_dt=CONTROL_DT)
        print("HardwarePlant: connected to motor_guard via IPC")

        # Target feedback for catch coordinator (accept/reject on :5559)
        from jugglebot.motion.ipc import TargetFeedbackPub
        feedback_pub = TargetFeedbackPub()
        print("TargetFeedbackPub: catch feedback on :5559")
    else:
        plant = MuJoCoPlant()

    # Set up telemetry logging
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    prefix = 'mpc' if args.mpc else 'sim'
    log_path = os.path.join(args.log_dir, f'{prefix}_{timestamp}.csv')
    # Tee stdout/stderr to companion .log file (hardware runs only — sim spam
    # is not useful to capture and can bloat logs).
    stdout_log_path = None
    if args.hardware:
        stdout_log_path = os.path.splitext(log_path)[0] + '.log'
        os.makedirs(os.path.dirname(stdout_log_path), exist_ok=True)
        _fh = open(stdout_log_path, 'w', buffering=1)
        class _Tee:
            def __init__(self, *s): self._s = s
            def write(self, d):
                for s in self._s:
                    s.write(d); s.flush()
                return len(d)
            def flush(self):
                for s in self._s: s.flush()
            def isatty(self): return False
        sys.stdout = _Tee(sys.__stdout__, _fh)
        sys.stderr = _Tee(sys.__stderr__, _fh)
    logger = TelemetryLogger(log_path)
    print(f"Logging to: {log_path}")
    if stdout_log_path:
        print(f"Stdout log: {stdout_log_path}")

    # Send session metadata to MPC bridge for ROS2 log correlation
    if args.hardware:
        from jugglebot.motion.ipc import SessionMetadataPush
        session_push = SessionMetadataPush()
        time.sleep(0.1)  # brief pause for ZMQ connection establishment
        session_push.send_session_start(os.path.basename(log_path))
        session_push.close()
        print(f"Session metadata sent to mpc_bridge_node")

    # Optional live dashboard
    dashboard = None
    if args.dashboard:
        from viz.dashboard import DashboardServer
        dashboard = DashboardServer(port=args.dashboard_port)
        dashboard.start()

    # Build feasibility checker for catch modes (needs plant)
    feasibility_checker = None
    if needs_feasibility:
        from hand.feasibility import FeasibilityChecker
        feasibility_checker = FeasibilityChecker(plant)
        print(f"Feasibility checker: coarse MPC (dt={feasibility_checker._coarse_dt}s, "
              f"horizon={feasibility_checker._horizon_s}s)")

    # Build MPC controller
    mpc = None
    if args.mpc:
        from controller import MPCController, MPCParams
        if args.hardware:
            # Hardware: real-time budget — params.py max_cpu_time is authoritative.
            # Cap max_iter for safety (fewer iterations = bounded worst-case).
            param_overrides = dict(max_iter=100)
        else:
            # Simulation: accuracy over speed — no real-time constraint
            param_overrides = dict(max_cpu_time=2.0, max_iter=500)
        if needs_high_vel:
            param_overrides['max_leg_vel_mmps'] = 1000.0
        mpc = MPCController.from_plant(MPCParams(**param_overrides), plant)
        assert abs(CONTROL_DT - mpc.params.dt_fine) < 1e-6, (
            f"CONTROL_DT ({CONTROL_DT}) must match MPC dt_fine "
            f"({mpc.params.dt_fine}) for correct warm-start shifting"
        )
        print("MPC controller initialised "
              f"(N={mpc.params.N}, tau={mpc.params.tau*1000:.0f} ms, "
              f"v_max={mpc.params.max_leg_vel_mmps:.0f} mm/s)")

    # ---- Build TargetSource for MPC modes ----

    # Active pose: the platform's resting/operating position.
    # Z = 170mm is the default operational height (from hardware_config.yaml).
    active_pose = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])

    if args.cycle_time is not None:
        from input.toss_loop import TossLoopController
        controller = TossLoopController(
            cycle_time=args.cycle_time,
            hold_ratio=args.hold_ratio,
            lateral_spacing_mm=args.lateral_spacing,
            platform_event_speed_ratio=args.platform_event_speed_ratio,
            active_pose=active_pose,
        )
        source = ContinuousThrowCatchSource(controller)
        print("  ─── Controls ─────────────────────────────────")
        print("  Space       Pause / unpause")
        print("  Up / Down   Speed ×2 / ×0.5")
        print("  R           Reset speed to 1×")
        print("  B           Reset ball (force restart)")
        print("  PgUp / PgDn Raise / lower ball height")
        print("  ─────────────────────────────────────────────\n")
        plant.model.vis.quality.shadowsize = 0

    elif args.juggle:
        from input.continuous_throw_catch import ContinuousThrowCatchController
        controller = ContinuousThrowCatchController(active_pose=active_pose)
        source = ContinuousThrowCatchSource(controller)
        print("\n  Continuous Throw-Catch Mode (Juggle)")
        print("  ─────────────────────────────────────────────────────")
        print("  T           Toggle editing: throw ↔ catch position")
        print("  Left / Num0 Selected pos X  -/+20mm")
        print("  Num1 / Num3 Selected pos Y  -/+20mm")
        print("  PgUp / PgDn Ball height     ±20mm")
        print("  F / G       Flight time     -/+0.05s")
        print("  B           Reset ball (on drop or force-reset)")
        print("  ─── Playback ─────────────────────────────────────")
        print("  Space       Pause / unpause")
        print("  Up / Down   Speed ×2 / ×0.5")
        print("  R           Reset speed to 1×")
        print("  ─────────────────────────────────────────────────────\n")
        plant.model.vis.quality.shadowsize = 0

    elif args.interactive_catch:
        from input.interactive_catch import InteractiveCatchController

        bb_sim = None
        if args.bb:
            from ball_butler.sim import BallButlerSim
            import math
            bb_pos = np.array([300.0, -400.0, 1500.0])
            bb_yaw = math.atan2(-bb_pos[1], -bb_pos[0])
            bb_sim = BallButlerSim.from_hardware_config(bb_pos, bb_yaw)
            print(f"Ball Butler enabled at pos={bb_pos.tolist()} mm, "
                  f"yaw={math.degrees(bb_yaw):.1f}°")

        controller = InteractiveCatchController(
            feasibility_checker=feasibility_checker,
            active_pose=active_pose,
            ball_butler_sim=bb_sim)
        source = InteractiveCatchSource(controller, active_pose=active_pose)
        # Print interactive catch help
        print("\n  Interactive Catch Mode")
        print("  ─────────────────────────────────────────────────────")
        if bb_sim is not None:
            print("  B           Ball Butler throw")
            print("  G           Spawn ball with current preset")
        else:
            print("  B           Spawn ball with current parameters")
        print("  N / M       Next / previous preset")
        print("  1-5         Select preset directly")
        print("  ─── Adjust spawn position ────────────────────────")
        print("  PgUp/PgDn   Spawn height  ±200mm")
        print("  Left/Num 0  XY offset X   ±20mm")
        print("  Num 1/3     XY offset Y   ±20mm")
        print("  ─── Adjust spawn velocity (numpad) ───────────────")
        print("  Num 8/2     Vz (down/up)    ±200mm/s")
        print("  Num 4/6     Vxy X           ±50mm/s")
        print("  Num 7/9     Vxy Y           ±50mm/s")
        print("  ─── Playback ─────────────────────────────────────")
        print("  Space       Pause / unpause")
        print("  Up/Down     Speed ×2 / ×0.5")
        print("  R           Reset speed to 1×")
        print("  ─────────────────────────────────────────────────────\n")
        plant.model.vis.quality.shadowsize = 0

    elif throw_catch_plan is not None:
        source = ThrowCatchTargetSource(throw_catch_plan, active_pose=active_pose)

    elif catch_sequence is not None:
        source = CatchTargetSource(catch_sequence,
                                   feasibility_checker=feasibility_checker,
                                   active_pose=active_pose)

    elif args.spacemouse:
        from input.spacemouse import SpaceMouseInput
        raw_input = SpaceMouseInput()
        if not raw_input.connected:
            print("ERROR: SpaceMouse not available. Use --keyboard instead.")
            sys.exit(1)
        source = InteractiveTargetSource(raw_input, CONTROL_DT)

    elif args.keyboard:
        from input.keyboard import KeyboardInput
        raw_input = KeyboardInput()
        source = InteractiveTargetSource(raw_input, CONTROL_DT)

    elif args.trajectory:
        source = WaypointTargetSource(waypoints)

    elif args.mpc and schedule is not None:
        source = StaticTargetSource(schedule)

    elif args.hardware:
        # Hardware mode with no explicit target: receive targets from ROS2
        # via mpc_bridge_node → ZMQ :5558.  This is the standard production
        # path where spacemouse/GUI/catch all route through mpc_bridge_node.
        from input.zmq_target import ZmqTargetSource
        default_z = plant.home_extensions_mm[0] if hasattr(plant, 'home_extensions_mm') else 170.0
        source = ZmqTargetSource(default_z_mm=default_z)
        print("ZmqTargetSource: waiting for targets from mpc_bridge_node on :5558")

    # ---- Run ----

    try:
        # Enable HardwarePlant pass-through mode before the MPC loop.
        # Skip when using ZmqTargetSource — the loop manages enable/disable
        # transitions based on mode messages from mpc_bridge_node.
        if args.hardware and not hasattr(source, 'enabled'):
            plant.enable()  # blocks until motor feedback telemetry arrives

        if source is not None:
            # Unified MPC path
            if args.no_viewer:
                run_mpc_headless(plant, mpc, source, duration, logger,
                                dashboard, feedback_pub=feedback_pub)
            else:
                run_mpc_with_viewer(plant, mpc, source, duration, logger,
                                   dashboard)
        else:
            # Direct-command path (no MPC)
            if args.no_viewer:
                run_headless(plant, schedule, duration, logger, dashboard)
            else:
                run_with_viewer(plant, schedule, duration, logger, dashboard)
    except KeyboardInterrupt:
        print("\nInterrupted — flushing telemetry...")
        logger.flush()
    finally:
        # Disable HardwarePlant before closing
        if args.hardware:
            plant.disable()
            time.sleep(0.05)
            plant.close()
        if feedback_pub is not None:
            feedback_pub.close()
        if source is not None and hasattr(source, 'close'):
            source.close()
        if dashboard is not None:
            dashboard.stop()

    print(f"Logged {logger.total_count} records to {log_path}")


if __name__ == '__main__':
    main()
