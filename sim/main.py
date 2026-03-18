"""Simulation entry point — run the MuJoCo Stewart platform simulation.

Usage examples:
    python sim/main.py                              # viewer, hold at home
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

import argparse
import os
import sys
import time
import datetime

import numpy as np

# Ensure sim/ is importable
_sim_dir = os.path.dirname(os.path.abspath(__file__))
if _sim_dir not in sys.path:
    sys.path.insert(0, _sim_dir)

from plant.mujoco_plant import MuJoCoPlant
from plant.interface import PlantState
from viz.telemetry import TelemetryLogger, StepRecord, record_from_arrays


# MPC control rate (Hz).
CONTROL_RATE_HZ = 50
CONTROL_DT = 1.0 / CONTROL_RATE_HZ


class SimController:
    """Pause / step / speed controls for the viewer.

    Keyboard bindings (active in the MuJoCo viewer window):
        Space       pause / unpause
        Right arrow step one frame (while paused)
        Up arrow    speed up (2×)
        Down arrow  slow down (0.5×)
        R           reset speed to 1×

    Pass ``sim_ctrl.key_callback`` to ``launch_passive(key_callback=...)``.
    Call ``sim_ctrl.wait()`` each loop iteration — it blocks while paused
    (except when stepping) and applies the speed multiplier.
    """

    # GLFW key codes
    _KEY_SPACE = 32
    _KEY_RIGHT = 262
    _KEY_UP = 265
    _KEY_DOWN = 264
    _KEY_R = 82

    def __init__(self):
        self.paused = False
        self.speed = 1.0
        self._step_once = False

    def key_callback(self, keycode):
        if keycode == self._KEY_SPACE:
            self.paused = not self.paused
            state = "PAUSED" if self.paused else "RUNNING"
            print(f"  [{state}]  speed={self.speed:.2f}x")
        elif keycode == self._KEY_RIGHT:
            if self.paused:
                self._step_once = True
        elif keycode == self._KEY_UP:
            self.speed = min(self.speed * 2.0, 16.0)
            print(f"  speed={self.speed:.2f}x")
        elif keycode == self._KEY_DOWN:
            self.speed = max(self.speed / 2.0, 0.0625)
            print(f"  speed={self.speed:.2f}x")
        elif keycode == self._KEY_R:
            self.speed = 1.0
            print(f"  speed={self.speed:.2f}x")

    def should_step(self) -> bool:
        """Return True if the sim should advance one step this iteration."""
        if not self.paused:
            return True
        if self._step_once:
            self._step_once = False
            return True
        return False

    @property
    def sleep_factor(self) -> float:
        """Multiply the normal sleep duration by this to control speed."""
        return 1.0 / self.speed


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
    p.add_argument('--log-dir', type=str, default=os.path.join(_sim_dir, 'logs'),
                   help='Directory for telemetry CSV output')
    p.add_argument('--trajectory', type=str, default=None,
                   help='Scripted trajectory name (T1, T2, T3, T4). '
                        'Implies --mpc. Overrides --pose/--sequence.')
    p.add_argument('--spacemouse', action='store_true',
                   help='SpaceMouse interactive input (Linux only, implies --mpc)')
    p.add_argument('--keyboard', action='store_true',
                   help='Keyboard interactive input (cross-platform, implies --mpc)')
    p.add_argument('--catch', type=str, default=None,
                   help='Scripted catch sequence (DT1..DT8). '
                        'Implies --mpc. Dynamic target + ball physics.')
    p.add_argument('--interactive-catch', action='store_true',
                   dest='interactive_catch',
                   help='Interactive ball catch mode — spawn balls with B key. '
                        'Implies --mpc. Requires viewer.')
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
# Reference scheduler (for MPC — does NOT command the plant directly)
# ---------------------------------------------------------------------------

class ReferenceScheduler:
    """Returns the target pose based on a time schedule.

    Unlike PoseScheduler, this does not call plant.command() — the MPC
    controller is responsible for generating and applying commands.
    """

    def __init__(self, schedule: list[tuple[float, np.ndarray]]):
        self._schedule = schedule
        self._target_pose = np.zeros(6)

    def update(self, sim_time: float) -> np.ndarray:
        """Return the target pose for the given simulation time."""
        for t, pose in self._schedule:
            if sim_time >= t:
                self._target_pose = pose
            else:
                break
        return self._target_pose

    @property
    def target_pose(self) -> np.ndarray:
        return self._target_pose


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
                  dashboard=None) -> None:
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
        solve_time_ms=diag.get('solve_time_ms', 0.0),
        solve_status=diag.get('status', 'n/a'),
        cost=diag.get('cost', 0.0),
        constraint_violation=diag.get('constraint_violation', 0.0),
    )
    logger.append(record)
    if dashboard is not None:
        dashboard.broadcast(record)


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
# MPC loops
# ---------------------------------------------------------------------------

def run_mpc_headless(plant: MuJoCoPlant, mpc, schedule, duration: float,
                     logger: TelemetryLogger, dashboard=None) -> None:
    """Run the MPC simulation loop without a viewer."""
    ref = ReferenceScheduler(schedule)
    n_steps = int(duration / CONTROL_DT)

    for step_idx in range(n_steps):
        state = plant.get_state()
        target = ref.update(state.time)
        cmd, diag = mpc.solve(state, target)
        plant.command(cmd)
        plant.step(CONTROL_DT)
        _log_mpc_step(logger, state, target, cmd, diag, dashboard=dashboard)

    logger.flush()
    _print_mpc_summary(logger)


def run_mpc_with_viewer(plant: MuJoCoPlant, mpc, schedule, duration: float,
                        logger: TelemetryLogger, dashboard=None) -> None:
    """Run the MPC simulation loop with the MuJoCo passive viewer."""
    import mujoco.viewer
    from viz.horizon import HorizonRenderer

    ref = ReferenceScheduler(schedule)
    horizon = HorizonRenderer(plant.geom.init_height_mm)

    with mujoco.viewer.launch_passive(plant.model, plant.data) as viewer:
        start_wall = time.monotonic()
        sim_time_target = 0.0

        while viewer.is_running() and sim_time_target < duration:
            state = plant.get_state()
            target = ref.update(state.time)
            cmd, diag = mpc.solve(state, target)
            plant.command(cmd)
            plant.step(CONTROL_DT)
            sim_time_target += CONTROL_DT

            _log_mpc_step(logger, state, target, cmd, diag, dashboard=dashboard)

            # Horizon visualisation
            horizon.update(mpc.predicted_poses)
            horizon.render(viewer)

            viewer.sync()

            # Real-time pacing
            elapsed = time.monotonic() - start_wall
            sleep_time = sim_time_target - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    logger.flush()
    _print_mpc_summary(logger)


# ---------------------------------------------------------------------------
# Trajectory-aware MPC loops (Phase 3)
# ---------------------------------------------------------------------------

def run_trajectory_headless(plant: MuJoCoPlant, mpc, ref_gen, duration: float,
                            logger: TelemetryLogger, dashboard=None) -> None:
    """Run MPC with a ReferenceGenerator (trajectory mode), headless."""
    n_steps = int(duration / CONTROL_DT)

    for step_idx in range(n_steps):
        state = plant.get_state()
        poses, twists = ref_gen.evaluate(state.time, CONTROL_DT, mpc.params.N)
        cmd, diag = mpc.solve(state, poses, ref_twist=twists)
        plant.command(cmd)
        plant.step(CONTROL_DT)
        _log_mpc_step(logger, state, poses[0], cmd, diag,
                      ref_twist=twists[0], dashboard=dashboard)

    logger.flush()
    _print_mpc_summary(logger)


def run_trajectory_with_viewer(plant: MuJoCoPlant, mpc, ref_gen, duration: float,
                               logger: TelemetryLogger, dashboard=None) -> None:
    """Run MPC with a ReferenceGenerator (trajectory mode), with viewer."""
    import mujoco.viewer
    from viz.horizon import HorizonRenderer

    horizon = HorizonRenderer(plant.geom.init_height_mm)

    with mujoco.viewer.launch_passive(plant.model, plant.data) as viewer:
        start_wall = time.monotonic()
        sim_time_target = 0.0

        while viewer.is_running() and sim_time_target < duration:
            state = plant.get_state()
            poses, twists = ref_gen.evaluate(state.time, CONTROL_DT, mpc.params.N)
            cmd, diag = mpc.solve(state, poses, ref_twist=twists)
            plant.command(cmd)
            plant.step(CONTROL_DT)
            sim_time_target += CONTROL_DT

            _log_mpc_step(logger, state, poses[0], cmd, diag,
                          ref_twist=twists[0], dashboard=dashboard)

            horizon.update(mpc.predicted_poses)
            horizon.render(viewer)
            viewer.sync()

            elapsed = time.monotonic() - start_wall
            sleep_time = sim_time_target - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    logger.flush()
    _print_mpc_summary(logger)


# ---------------------------------------------------------------------------
# Interactive input MPC loops (Phase 4)
# ---------------------------------------------------------------------------

def run_interactive_with_viewer(plant: MuJoCoPlant, mpc, input_source,
                                duration: float, logger: TelemetryLogger,
                                dashboard=None) -> None:
    """Run MPC with live interactive input (spacemouse or keyboard), with viewer.

    Parameters
    ----------
    input_source : object
        Must have a ``read() -> np.ndarray`` method returning (6,) target pose,
        and optionally a ``key_callback`` attribute for keyboard input.
    """
    import mujoco.viewer
    from viz.horizon import HorizonRenderer

    horizon = HorizonRenderer(plant.geom.init_height_mm)

    # key_callback must be passed at construction — cannot be set later
    kc = getattr(input_source, 'key_callback', None)
    with mujoco.viewer.launch_passive(
        plant.model, plant.data, key_callback=kc
    ) as viewer:
        start_wall = time.monotonic()
        sim_time_target = 0.0

        while viewer.is_running() and sim_time_target < duration:
            state = plant.get_state()

            # Integrate held-key motion (keyboard); no-op for spacemouse
            if hasattr(input_source, 'apply'):
                input_source.apply(CONTROL_DT)

            # Read the latest target from the input device
            target_pose = input_source.read()

            # Build a static reference for the MPC horizon
            # (MPC plans smooth optimal path to reach the target)
            ref_poses = np.tile(target_pose, (mpc.params.N + 1, 1))
            ref_twists = np.zeros_like(ref_poses)

            cmd, diag = mpc.solve(state, ref_poses, ref_twist=ref_twists)
            plant.command(cmd)
            plant.step(CONTROL_DT)
            sim_time_target += CONTROL_DT

            _log_mpc_step(logger, state, target_pose, cmd, diag,
                          ref_twist=np.zeros(6), dashboard=dashboard)

            horizon.update(mpc.predicted_poses)
            horizon.render(viewer)
            viewer.sync()

            # Real-time pacing
            elapsed = time.monotonic() - start_wall
            sleep_time = sim_time_target - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    logger.flush()
    _print_mpc_summary(logger)


# ---------------------------------------------------------------------------
# Catch loops (Phase 5C)
# ---------------------------------------------------------------------------

def run_catch_headless(plant: MuJoCoPlant, mpc, catch_coordinator,
                       duration: float, logger: TelemetryLogger,
                       dashboard=None) -> None:
    """Run MPC with dynamic targets + ball physics, headless."""
    from catch.coordinator import CatchPhase
    from catch.hand_trajectory import HandCatchSequence
    n_steps = int(duration / CONTROL_DT)
    active_hand_seq = None  # Currently executing HandCatchSequence

    for step_idx in range(n_steps):
        state = plant.get_state()
        current_pose = _pose_6dof_from_state(state)
        hand_pos = state.hand_pos_mm if state.hand_pos_mm is not None else 0.0

        # Check if ball should be spawned
        spawn = catch_coordinator.should_spawn_ball(state.time)
        if spawn is not None:
            plant.spawn_ball(spawn.position_mm, spawn.velocity_mms)

        # Update coordinator → get reference + hand command
        ref_gen, hand_cmd = catch_coordinator.update(
            state.time, current_pose, hand_pos_mm=hand_pos)

        # Execute hand command
        if isinstance(hand_cmd, HandCatchSequence):
            active_hand_seq = hand_cmd
        elif hand_cmd == 'prime':
            plant.hand_to_prime()
            active_hand_seq = None
        elif hand_cmd == 'home':
            plant.hand_to_home()
            active_hand_seq = None

        # Sample active hand catch trajectory every step
        if active_hand_seq is not None:
            pos = active_hand_seq.sample(state.time)
            if pos is not None:
                plant.command_hand(pos)
            else:
                active_hand_seq = None

        # Run MPC with the reference
        if ref_gen is not None:
            poses, twists = ref_gen.evaluate(state.time, CONTROL_DT, mpc.params.N)
            cmd, diag = mpc.solve(state, poses, ref_twist=twists)
        else:
            # No reference — hold home
            home_ref = np.zeros((mpc.params.N + 1, 6))
            cmd, diag = mpc.solve(state, home_ref)

        plant.command(cmd)
        plant.step(CONTROL_DT)

        # Check for ball capture
        if plant.has_ball:
            if plant.check_and_capture():
                catch_coordinator.notify_capture(state.time)

        # Log
        ref_pose = poses[0] if ref_gen is not None else np.zeros(6)
        ref_twist = twists[0] if ref_gen is not None else np.zeros(6)
        _log_mpc_step(logger, state, ref_pose, cmd, diag,
                      ref_twist=ref_twist, dashboard=dashboard)

    logger.flush()
    _print_mpc_summary(logger)
    _print_catch_summary(catch_coordinator)


def run_catch_with_viewer(plant: MuJoCoPlant, mpc, catch_sequence,
                          duration: float, logger: TelemetryLogger,
                          dashboard=None) -> None:
    """Run MPC with dynamic targets + ball physics, with viewer.

    Loops the catch sequence until the viewer is closed.

    Parameters
    ----------
    catch_sequence : list of (DynamicTarget, BallSpawn | None)
        The raw sequence — rebuilt into a fresh CatchCoordinator each loop.
    """
    import mujoco.viewer
    from viz.horizon import HorizonRenderer
    from catch.coordinator import CatchCoordinator

    horizon = HorizonRenderer(plant.geom.init_height_mm)
    sim_ctrl = SimController()

    print("\n  Controls:  Space=pause  Right=step  Up/Down=speed  R=reset speed\n")

    with mujoco.viewer.launch_passive(
        plant.model, plant.data, key_callback=sim_ctrl.key_callback
    ) as viewer:
        iteration = 0

        while viewer.is_running():
            # Fresh coordinator each loop
            catch_coordinator = CatchCoordinator(
                feasibility_checker=feasibility_checker)
            for target, ball in catch_sequence:
                catch_coordinator.submit_target(target, ball)

            # Reset plant + MPC for a clean start
            plant.reset()
            mpc.reset()
            iteration += 1
            active_hand_seq = None

            start_wall = time.monotonic()
            sim_time_target = 0.0

            while viewer.is_running() and sim_time_target < duration:
                # Pause / step gate
                if not sim_ctrl.should_step():
                    viewer.sync()
                    time.sleep(0.01)  # don't spin at 100% CPU while paused
                    continue

                state = plant.get_state()
                current_pose = _pose_6dof_from_state(state)
                hand_pos = state.hand_pos_mm if state.hand_pos_mm is not None else 0.0

                # Check if ball should be spawned
                spawn = catch_coordinator.should_spawn_ball(state.time)
                if spawn is not None:
                    plant.spawn_ball(spawn.position_mm, spawn.velocity_mms)

                # Update coordinator → get reference + hand command
                ref_gen, hand_cmd = catch_coordinator.update(
                    state.time, current_pose, hand_pos_mm=hand_pos)

                from catch.hand_trajectory import HandCatchSequence
                if isinstance(hand_cmd, HandCatchSequence):
                    active_hand_seq = hand_cmd
                elif hand_cmd == 'prime':
                    plant.hand_to_prime()
                    active_hand_seq = None
                elif hand_cmd == 'home':
                    plant.hand_to_home()
                    active_hand_seq = None

                # Sample active hand catch trajectory every step
                if active_hand_seq is not None:
                    pos = active_hand_seq.sample(state.time)
                    if pos is not None:
                        plant.command_hand(pos)
                    else:
                        active_hand_seq = None

                # Run MPC
                if ref_gen is not None:
                    poses, twists = ref_gen.evaluate(
                        state.time, CONTROL_DT, mpc.params.N)
                    cmd, diag = mpc.solve(state, poses, ref_twist=twists)
                else:
                    home_ref = np.zeros((mpc.params.N + 1, 6))
                    cmd, diag = mpc.solve(state, home_ref)

                plant.command(cmd)
                plant.step(CONTROL_DT)
                sim_time_target += CONTROL_DT

                # Check for ball capture
                if plant.has_ball:
                    if plant.check_and_capture():
                        catch_coordinator.notify_capture(state.time)

                # Log + visualise
                ref_pose = poses[0] if ref_gen is not None else np.zeros(6)
                ref_twist = twists[0] if ref_gen is not None else np.zeros(6)
                _log_mpc_step(logger, state, ref_pose, cmd, diag,
                              ref_twist=ref_twist, dashboard=dashboard)

                horizon.update(mpc.predicted_poses)
                horizon.render(viewer)
                viewer.sync()

                # Real-time pacing (adjusted by speed multiplier)
                elapsed = time.monotonic() - start_wall
                sleep_time = (sim_time_target * sim_ctrl.sleep_factor) - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

            # Print summary for this iteration
            print(f"\n--- Iteration {iteration} ---")
            _print_catch_summary(catch_coordinator)

    logger.flush()
    _print_mpc_summary(logger)


def run_interactive_catch_with_viewer(
    plant: MuJoCoPlant,
    mpc,
    feasibility_checker,
    logger: TelemetryLogger,
    dashboard=None,
) -> None:
    """Interactive catch mode — spawn balls on demand with the viewer.

    The user presses B to spawn balls, N/M to cycle presets, and watches
    the full catch pipeline execute.
    """
    import mujoco.viewer
    from viz.horizon import HorizonRenderer
    from input.interactive_catch import InteractiveCatchController
    from catch.hand_trajectory import HandCatchSequence

    controller = InteractiveCatchController(
        feasibility_checker=feasibility_checker)
    horizon = HorizonRenderer(plant.geom.init_height_mm)

    print("\n  Interactive Catch Mode")
    print("  ─────────────────────────────────────────────────────")
    print("  B           Spawn ball with current parameters")
    print("  N / M       Next / previous preset")
    print("  1-5         Select preset directly")
    print("  ─── Adjust spawn position ────────────────────────")
    print("  PgUp/PgDn   Spawn height  ±200mm")
    print("  Left arrow  XY offset X   -20mm")
    print("  ─── Adjust spawn velocity (numpad) ───────────────")
    print("  Num 8/2     Vz (down speed)  ±200mm/s")
    print("  Num 4/6     Vxy X            ±50mm/s")
    print("  Num 7/9     Vxy Y            ±50mm/s")
    print("  ─── Playback ─────────────────────────────────────")
    print("  Space       Pause / unpause")
    print("  Up/Down     Speed ×2 / ×0.5")
    print("  R           Reset speed to 1×")
    print("  ─────────────────────────────────────────────────────\n")

    with mujoco.viewer.launch_passive(
        plant.model, plant.data,
        key_callback=controller.key_callback,
    ) as viewer:
        wall_budget = 0.0  # accumulated wall-time budget (speed-adjusted)
        start_wall = time.monotonic()

        while viewer.is_running():
            if not controller.should_step():
                # While paused, keep wall budget in sync so we don't
                # accumulate a huge deficit
                start_wall = time.monotonic()
                wall_budget = 0.0
                viewer.sync()
                time.sleep(0.01)
                continue

            state = plant.get_state()
            current_pose = _pose_6dof_from_state(state)
            hand_pos = state.hand_pos_mm if state.hand_pos_mm is not None else 0.0

            # Update controller — returns ref, hand command, and optional ball spawn
            ref_gen, hand_cmd, ball_spawn = controller.update(
                state.time, current_pose, hand_pos)

            # Spawn ball if requested
            if ball_spawn is not None:
                if plant.ball_manager is not None:
                    plant.ball_manager.reset()
                plant.spawn_ball(ball_spawn.position_mm, ball_spawn.velocity_mms)

            # Execute hand command
            if isinstance(hand_cmd, (int, float)):
                plant.command_hand(float(hand_cmd))
            elif hand_cmd == 'prime':
                plant.hand_to_prime()
            elif hand_cmd == 'home':
                plant.hand_to_home()

            # MPC solve
            if ref_gen is not None:
                poses, twists = ref_gen.evaluate(
                    state.time, CONTROL_DT, mpc.params.N)
                cmd, diag = mpc.solve(state, poses, ref_twist=twists)
            else:
                home_ref = np.zeros((mpc.params.N + 1, 6))
                cmd, diag = mpc.solve(state, home_ref)

            plant.command(cmd)
            plant.step(CONTROL_DT)

            # Check for ball capture
            if plant.has_ball and plant.check_and_capture():
                from input.interactive_catch import InteractiveCatchState
                if (controller._coordinator is not None
                        and controller._state == InteractiveCatchState.CATCHING):
                    controller._coordinator.notify_capture(state.time)

            # Log + visualise
            ref_pose = poses[0] if ref_gen is not None else np.zeros(6)
            ref_twist = twists[0] if ref_gen is not None else np.zeros(6)
            _log_mpc_step(logger, state, ref_pose, cmd, diag,
                          ref_twist=ref_twist, dashboard=dashboard)

            # Render: MPC horizon first (resets ngeom), then spawn preview on top
            horizon.update(mpc.predicted_poses)
            horizon.render(viewer)
            controller.render_preview(viewer)
            viewer.sync()

            # Real-time pacing: each step adds CONTROL_DT / speed to budget
            wall_budget += CONTROL_DT * controller.sleep_factor
            elapsed = time.monotonic() - start_wall
            sleep_time = wall_budget - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            elif sleep_time < -0.5:
                # Fallen too far behind (e.g., after speed change) — resync
                start_wall = time.monotonic()
                wall_budget = 0.0

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


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    args = parse_args()

    # Interactive input mode (Phase 4) — overrides everything else
    input_source = None
    if args.spacemouse:
        args.mpc = True
        from input.spacemouse import SpaceMouseInput
        input_source = SpaceMouseInput()
        if not input_source.connected:
            print("ERROR: SpaceMouse not available. Use --keyboard instead.")
            sys.exit(1)
        label = "SpaceMouse interactive"
        default_duration = 300.0  # 5 minutes, effectively unlimited
    elif args.keyboard:
        args.mpc = True
        from input.keyboard import KeyboardInput
        input_source = KeyboardInput()
        label = "Keyboard interactive"
        default_duration = 300.0

    # Mode selection: interactive catch > scripted catch > trajectory > pose/sequence
    interactive_catch = False
    catch_coordinator = None
    ref_gen = None

    if args.interactive_catch and input_source is None:
        # Interactive catch mode
        args.mpc = True
        interactive_catch = True
        label = "Interactive catch"
        default_duration = 600.0  # 10 minutes (effectively unlimited)
        schedule = None

    elif args.catch and input_source is None:
        # Scripted catch mode (Phase 5C)
        args.mpc = True
        from input.scripted import get_catch_sequence
        from catch.coordinator import CatchCoordinator
        catch_sequence, default_duration = get_catch_sequence(args.catch)
        catch_coordinator = CatchCoordinator()
        for target, ball in catch_sequence:
            catch_coordinator.submit_target(target, ball)
        label = f"Catch: {args.catch}"
        schedule = None

    elif args.trajectory and input_source is None:
        args.mpc = True  # trajectories always use MPC
        from input.scripted import get_trajectory
        ref_gen, default_duration = get_trajectory(args.trajectory)
        label = f"Trajectory: {args.trajectory}"
        schedule = None  # not used in trajectory mode
    elif input_source is not None:
        schedule = None  # not used in interactive mode
    elif args.sequence:
        schedule = _parse_sequence(args.sequence)
        label = f"Sequence: {len(schedule)} poses"
        default_duration = schedule[-1][0] + 2.0
    elif args.pose:
        target = _parse_pose(args.pose)
        schedule = [(0.0, target)]
        label = f"Static pose: {target}"
        default_duration = 10.0
    else:
        schedule = [(0.0, np.zeros(6))]
        label = "Home"
        default_duration = 10.0

    duration = args.duration if args.duration is not None else default_duration

    mode = "MPC" if args.mpc else "Direct"
    print(f"{label} [{mode}]")
    if schedule is not None:
        for t, pose in schedule:
            print(f"  t={t:.1f}s -> [{', '.join(f'{v:.2f}' for v in pose)}]")
    print(f"Duration: {duration:.1f}s, Control rate: {CONTROL_RATE_HZ} Hz")

    # Create plant
    plant = MuJoCoPlant()

    # Set up telemetry logging
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    prefix = 'mpc' if args.mpc else 'sim'
    log_path = os.path.join(args.log_dir, f'{prefix}_{timestamp}.csv')
    logger = TelemetryLogger(log_path)
    print(f"Logging to: {log_path}")

    # Optional live dashboard
    dashboard = None
    if args.dashboard:
        from viz.dashboard import DashboardServer
        dashboard = DashboardServer(port=args.dashboard_port)
        dashboard.start()

    # Build feasibility checker for catch modes (needs plant)
    feasibility_checker = None
    if catch_coordinator is not None or interactive_catch:
        from catch.feasibility import FeasibilityChecker
        feasibility_checker = FeasibilityChecker(plant)
        if catch_coordinator is not None:
            catch_coordinator._feasibility = feasibility_checker
        print(f"Feasibility checker: coarse MPC (dt={feasibility_checker._coarse_dt}s, "
              f"horizon={feasibility_checker._horizon_s}s)")

    # Optional MPC controller
    mpc = None
    if args.mpc:
        from controller import MPCController, MPCParams
        # Use generous solver budget for interactive use (cold starts need headroom).
        # The tight 18ms budget is for production (Phase 6) where the control
        # period is a hard deadline. In simulation, we want correct solutions.
        param_overrides = dict(max_cpu_time=2.0, max_iter=500)
        if ref_gen is not None or input_source is not None or catch_coordinator is not None or interactive_catch:
            param_overrides['max_leg_vel_mmps'] = 1000.0
        mpc = MPCController.from_plant(MPCParams(**param_overrides), plant)
        print("MPC controller initialised "
              f"(N={mpc.params.N}, tau={mpc.params.tau*1000:.0f} ms, "
              f"v_max={mpc.params.max_leg_vel_mmps:.0f} mm/s)")

    try:
        if input_source is not None:
            # Interactive input mode (Phase 4) — viewer required
            if args.no_viewer:
                print("ERROR: --spacemouse/--keyboard requires the viewer "
                      "(remove --no-viewer)")
                sys.exit(1)
            run_interactive_with_viewer(plant, mpc, input_source, duration,
                                       logger, dashboard)
        elif interactive_catch:
            # Interactive catch mode — viewer required
            if args.no_viewer:
                print("ERROR: --interactive-catch requires the viewer "
                      "(remove --no-viewer)")
                sys.exit(1)
            run_interactive_catch_with_viewer(plant, mpc, feasibility_checker,
                                             logger, dashboard)
        elif catch_coordinator is not None:
            # Catch mode (Phase 5C)
            if args.no_viewer:
                run_catch_headless(plant, mpc, catch_coordinator, duration,
                                  logger, dashboard)
            else:
                # Viewer mode loops until closed — pass raw sequence
                run_catch_with_viewer(plant, mpc, catch_sequence, duration,
                                     logger, dashboard)
        elif ref_gen is not None:
            # Trajectory mode (Phase 3)
            if args.no_viewer:
                run_trajectory_headless(plant, mpc, ref_gen, duration, logger, dashboard)
            else:
                run_trajectory_with_viewer(plant, mpc, ref_gen, duration, logger, dashboard)
        elif args.mpc:
            if args.no_viewer:
                run_mpc_headless(plant, mpc, schedule, duration, logger, dashboard)
            else:
                run_mpc_with_viewer(plant, mpc, schedule, duration, logger, dashboard)
        else:
            if args.no_viewer:
                run_headless(plant, schedule, duration, logger, dashboard)
            else:
                run_with_viewer(plant, schedule, duration, logger, dashboard)
    except KeyboardInterrupt:
        print("\nInterrupted — flushing telemetry...")
        logger.flush()
    finally:
        if input_source is not None and hasattr(input_source, 'close'):
            input_source.close()
        if dashboard is not None:
            dashboard.stop()

    print(f"Logged {len(logger.records)} records to {log_path}")


if __name__ == '__main__':
    main()
