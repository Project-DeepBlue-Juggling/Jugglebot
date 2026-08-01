"""Continuous throw-catch mode — Jugglebot throws to itself in a loop.

Provides ``ContinuousThrowCatchController`` which manages:
  - Continuous throw-catch cycling using ThrowCatchPlanner
  - Real-time parameter adjustment (throw pos, catch pos, height, flight time)
  - Automatic replanning between cycles
  - Ball reset on drops

Usage:
    python sim/main.py --juggle
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from enum import Enum, auto

import numpy as np

from sim.hand.coordinator import HandCoordinator, DynamicTarget, HandPhase
from sim.hand.planner import ThrowCatchPlanner
from sim.input.sim_control import SimController
from sim.plant.interface import PlantState

logger = logging.getLogger(__name__)

import mujoco

# GLFW key codes
_KEY_T = 84           # Toggle throw/catch editing
_KEY_LEFT = 263       # Selected pos X -
_KEY_NUMPAD_0 = 320   # Selected pos X +
_KEY_NUMPAD_1 = 321   # Selected pos Y -
_KEY_NUMPAD_3 = 323   # Selected pos Y +
_KEY_PAGE_UP = 266    # Ball height +
_KEY_PAGE_DOWN = 267  # Ball height -
_KEY_F = 70           # Flight time -
_KEY_G = 71           # Flight time +
_KEY_B = 66           # Reset ball

# Adjustment increments
_XY_STEP = 20.0           # mm
_HEIGHT_STEP = 20.0       # mm
_FLIGHT_TIME_STEP = 0.05  # s

_PLATFORM_HEIGHT_MM = 574.3
_APPROACH_TIME = 2.0       # seconds to approach throw position
_INTER_CYCLE_PAUSE = 0.3   # seconds between catch and next throw


@dataclass
class JuggleParams:
    """Adjustable parameters for continuous throw-catch."""
    throw_x_mm: float = -200.0
    throw_y_mm: float = 0.0
    catch_x_mm: float = 200.0
    catch_y_mm: float = 0.0
    ball_z_offset_mm: float = 255.0   # above platform height
    flight_time_s: float = 1.15

    @property
    def ball_z_mm(self) -> float:
        return _PLATFORM_HEIGHT_MM + self.ball_z_offset_mm

    @property
    def throw_pos_mm(self) -> np.ndarray:
        return np.array([self.throw_x_mm, self.throw_y_mm, self.ball_z_mm])

    @property
    def catch_pos_mm(self) -> np.ndarray:
        return np.array([self.catch_x_mm, self.catch_y_mm, self.ball_z_mm])


class JuggleState(Enum):
    STARTUP = auto()         # Spawning ball, planning first throw
    CYCLE_ACTIVE = auto()    # HandCoordinator running throw-catch
    BETWEEN_CYCLES = auto()  # Pause between successful catch and next throw
    DROPPED = auto()         # Ball missed, waiting for reset


class ContinuousThrowCatchController:
    """Continuous throw-catch controller for the MuJoCo viewer.

    Manages self-throwing cycles: throw -> catch -> throw -> catch ...
    with real-time parameter adjustment.

    Parameters
    ----------
    active_pose : (6,) array or None
        Platform active pose (default: zeros).
    params : JuggleParams or None
        Initial throw-catch parameters.  Defaults to TC2-like settings.
    """

    def __init__(self, active_pose: np.ndarray | None = None,
                 params: JuggleParams | None = None):
        self._active_pose = active_pose if active_pose is not None else np.zeros(6)
        self._params = params if params is not None else JuggleParams()
        self._planner = ThrowCatchPlanner()
        self._sim_ctrl = SimController()

        self._state = JuggleState.STARTUP
        self._state_start_time = 0.0
        self._editing_throw = True  # True = editing throw pos, False = catch pos

        # Coordinator state
        self._coordinator: HandCoordinator | None = None
        self._plan = None
        self._ball_spawned_initial = False

        # Stats
        self._cycle_count = 0
        self._catch_count = 0
        self._drop_count = 0

        # Reset request flag
        self._reset_requested = False

    # ------------------------------------------------------------------
    # Keyboard callback
    # ------------------------------------------------------------------

    def key_callback(self, keycode: int) -> None:
        """Handle keyboard events from the MuJoCo viewer."""
        # Delegate sim control keys (pause/step/speed)
        self._sim_ctrl.key_callback(keycode)

        # Toggle throw/catch editing
        if keycode == _KEY_T:
            self._editing_throw = not self._editing_throw
            which = "THROW" if self._editing_throw else "CATCH"
            print(f"  Editing: {which} position")
            self._print_params()
            return

        # Reset ball
        if keycode == _KEY_B:
            if self._state == JuggleState.DROPPED:
                self._reset_requested = True
                print("  Ball reset requested")
            elif self._state == JuggleState.CYCLE_ACTIVE:
                # Allow force-reset during active cycle (e.g. stuck)
                self._reset_requested = True
                print("  Force reset requested")
            else:
                print(f"  (reset ignored — state is {self._state.name})")
            return

        # Position adjustments (affect selected: throw or catch)
        if keycode == _KEY_LEFT:
            if self._editing_throw:
                self._params.throw_x_mm -= _XY_STEP
            else:
                self._params.catch_x_mm -= _XY_STEP
            self._print_params()
            return
        if keycode == _KEY_NUMPAD_0:
            if self._editing_throw:
                self._params.throw_x_mm += _XY_STEP
            else:
                self._params.catch_x_mm += _XY_STEP
            self._print_params()
            return
        if keycode == _KEY_NUMPAD_1:
            if self._editing_throw:
                self._params.throw_y_mm -= _XY_STEP
            else:
                self._params.catch_y_mm -= _XY_STEP
            self._print_params()
            return
        if keycode == _KEY_NUMPAD_3:
            if self._editing_throw:
                self._params.throw_y_mm += _XY_STEP
            else:
                self._params.catch_y_mm += _XY_STEP
            self._print_params()
            return

        # Ball height
        if keycode == _KEY_PAGE_UP:
            self._params.ball_z_offset_mm += _HEIGHT_STEP
            self._print_params()
            return
        if keycode == _KEY_PAGE_DOWN:
            self._params.ball_z_offset_mm = max(
                100.0, self._params.ball_z_offset_mm - _HEIGHT_STEP)
            self._print_params()
            return

        # Flight time
        if keycode == _KEY_F:
            self._params.flight_time_s = max(
                0.5, self._params.flight_time_s - _FLIGHT_TIME_STEP)
            self._print_params()
            return
        if keycode == _KEY_G:
            self._params.flight_time_s = min(
                2.0, self._params.flight_time_s + _FLIGHT_TIME_STEP)
            self._print_params()
            return

    def should_step(self) -> bool:
        """Return True if the sim should advance one step."""
        return self._sim_ctrl.should_step()

    @property
    def sleep_factor(self) -> float:
        return self._sim_ctrl.sleep_factor

    # ------------------------------------------------------------------
    # Main update
    # ------------------------------------------------------------------

    def update(
        self,
        sim_time: float,
        current_pose: np.ndarray,
        hand_pos_mm: float,
        plant_state: PlantState | None = None,
    ) -> tuple[DynamicTarget | None, object, object]:
        """Advance the state machine.

        Returns
        -------
        target : DynamicTarget or None
            Target for MPC this step.
        hand_cmd : str, float, HandCatchSequence, HandThrowSequence,
                   BallRelease, or None
        ball_spawn : BallSpawn, 'spawn_in_hand', or None
        """
        hand_cmd = None
        ball_spawn = None
        active_target = DynamicTarget(pose_6dof=self._active_pose.copy())

        # -- STARTUP: spawn ball in hand and plan first throw --
        if self._state == JuggleState.STARTUP:
            if not self._ball_spawned_initial:
                self._ball_spawned_initial = True
                self._state_start_time = sim_time
                ball_spawn = 'spawn_in_hand'

            # Wait briefly for ball to settle in hand
            if sim_time - self._state_start_time > 0.3:
                success = self._plan_next_cycle(
                    sim_time, current_pose, hand_pos_mm)
                if success:
                    self._state = JuggleState.CYCLE_ACTIVE
                    self._state_start_time = sim_time
                    self._print_cycle_header()
                # If planning fails, stay in STARTUP and retry next frame

            return active_target, hand_cmd, ball_spawn

        # -- CYCLE_ACTIVE: coordinator running throw-catch --
        if self._state == JuggleState.CYCLE_ACTIVE:
            # Handle force-reset during active cycle
            if self._reset_requested:
                self._reset_requested = False
                ball_spawn = 'spawn_in_hand'
                self._state = JuggleState.BETWEEN_CYCLES
                self._state_start_time = sim_time
                self._coordinator = None
                print("  Force reset — returning to active pose, replanning...")
                return active_target, 'home', ball_spawn

            if self._coordinator is not None:
                target, hcmd = self._coordinator.update(
                    sim_time, current_pose, hand_pos_mm=hand_pos_mm,
                    plant_state=plant_state)
                hand_cmd = hcmd

                # Check if coordinator finished (returned to IDLE)
                elapsed = sim_time - self._state_start_time
                if (self._coordinator.phase == HandPhase.IDLE
                        and elapsed > 1.0):
                    was_caught = any(
                        e.captured for e in self._coordinator.events)
                    if was_caught:
                        self._catch_count += 1
                        print(f"  CAUGHT! "
                              f"({self._catch_count}/{self._cycle_count})")
                        self._state = JuggleState.BETWEEN_CYCLES
                        self._state_start_time = sim_time
                    else:
                        self._drop_count += 1
                        print(f"  DROPPED! ({self._drop_count} drops, "
                              f"{self._catch_count}/{self._cycle_count} "
                              f"caught)")
                        print("  Press B to reset ball")
                        self._state = JuggleState.DROPPED
                        self._state_start_time = sim_time

                return (target or active_target), hand_cmd, ball_spawn

        # -- BETWEEN_CYCLES: brief pause then plan next throw --
        if self._state == JuggleState.BETWEEN_CYCLES:
            elapsed = sim_time - self._state_start_time
            if elapsed >= _INTER_CYCLE_PAUSE:
                success = self._plan_next_cycle(
                    sim_time, current_pose, hand_pos_mm)
                if success:
                    self._state = JuggleState.CYCLE_ACTIVE
                    self._state_start_time = sim_time
                    self._print_cycle_header()
                else:
                    print("  Plan failed — try adjusting parameters")
                    print("  Press B to reset ball and retry")
                    self._state = JuggleState.DROPPED
                    self._state_start_time = sim_time

            return active_target, None, None

        # -- DROPPED: wait for reset --
        if self._state == JuggleState.DROPPED:
            if self._reset_requested:
                self._reset_requested = False
                ball_spawn = 'spawn_in_hand'
                self._state = JuggleState.BETWEEN_CYCLES
                self._state_start_time = sim_time
                print("  Ball reset — planning next cycle...")

            return active_target, None, ball_spawn

        return active_target, None, None

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def _plan_next_cycle(
        self,
        sim_time: float,
        current_pose: np.ndarray,
        hand_pos_mm: float,
    ) -> bool:
        """Plan the next throw-catch cycle using current params."""
        self._cycle_count += 1

        throw_time = sim_time + _APPROACH_TIME
        catch_time = throw_time + self._params.flight_time_s

        try:
            plan = self._planner.plan(
                throw_pos_mm=self._params.throw_pos_mm,
                catch_pos_mm=self._params.catch_pos_mm,
                throw_time=throw_time,
                catch_time=catch_time,
                current_hand_pos_mm=hand_pos_mm,
                current_time=sim_time,
            )
        except ValueError as e:
            print(f"  Plan failed: {e}")
            self._cycle_count -= 1
            return False

        self._plan = plan
        self._coordinator = HandCoordinator(active_pose=self._active_pose)
        self._coordinator.submit_throw_catch(plan)

        print(f"  Throw: {_fmt_vec(self._params.throw_pos_mm)} -> "
              f"Catch: {_fmt_vec(self._params.catch_pos_mm)}")
        print(f"  Flight: {self._params.flight_time_s:.2f}s, "
              f"throw at t={throw_time:.2f}s, "
              f"catch at t={catch_time:.2f}s")

        return True

    # ------------------------------------------------------------------
    # Capture notification
    # ------------------------------------------------------------------

    def notify_capture(self, sim_time: float) -> None:
        """Called when ball capture is detected."""
        if (self._coordinator is not None
                and self._state == JuggleState.CYCLE_ACTIVE):
            self._coordinator.notify_capture(sim_time)

    # ------------------------------------------------------------------
    # 3D preview rendering
    # ------------------------------------------------------------------

    def render_preview(self, viewer) -> None:
        """Draw throw/catch position markers in the viewer."""
        scn = viewer.user_scn

        p = self._params
        throw_pos_m = p.throw_pos_mm / 1000.0
        catch_pos_m = p.catch_pos_mm / 1000.0

        if scn.ngeom >= scn.maxgeom - 2:
            return

        # Throw position marker (green sphere)
        geom = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([0.015, 0.0, 0.0]),
            throw_pos_m,
            np.eye(3).flatten(),
            np.array([0.0, 0.8, 0.0, 0.5], dtype=np.float32),
        )
        label_t = "THROW*" if self._editing_throw else "Throw"
        geom.label = label_t[:99]
        scn.ngeom += 1

        if scn.ngeom >= scn.maxgeom:
            return

        # Catch position marker (blue sphere)
        geom2 = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(
            geom2,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([0.015, 0.0, 0.0]),
            catch_pos_m,
            np.eye(3).flatten(),
            np.array([0.0, 0.4, 1.0, 0.5], dtype=np.float32),
        )
        label_c = "CATCH*" if not self._editing_throw else "Catch"
        geom2.label = label_c[:99]
        scn.ngeom += 1

    # ------------------------------------------------------------------
    # Console output
    # ------------------------------------------------------------------

    def _print_params(self) -> None:
        p = self._params
        t_mark = "*" if self._editing_throw else " "
        c_mark = "*" if not self._editing_throw else " "
        print(f"  {t_mark}throw: ({p.throw_x_mm:.0f}, {p.throw_y_mm:.0f})  "
              f"{c_mark}catch: ({p.catch_x_mm:.0f}, {p.catch_y_mm:.0f})  "
              f"Z={p.ball_z_mm:.0f}mm  flight={p.flight_time_s:.2f}s")

    def _print_cycle_header(self) -> None:
        print(f"\n  === Cycle {self._cycle_count} "
              f"({self._catch_count}/{self._cycle_count - 1} caught) ===")

    def print_summary(self) -> None:
        """Print session summary."""
        total = self._catch_count + self._drop_count
        if total > 0:
            pct = self._catch_count / total * 100
        else:
            pct = 0.0
        print(f"\n  Juggle Summary: {self._catch_count}/{total} caught "
              f"({pct:.0f}%), {self._drop_count} dropped")


def _fmt_vec(v: np.ndarray) -> str:
    """Format a small vector for console output."""
    return '[' + ', '.join(f'{x:.1f}' for x in v) + ']'
