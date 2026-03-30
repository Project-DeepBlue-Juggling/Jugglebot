"""Interactive catch mode — spawn balls on demand and watch the catch pipeline.

Provides ``InteractiveCatchController`` which manages:
  - Spawn presets (configurable ball initial conditions)
  - Keyboard input (spawn, cycle presets, pause/step/speed)
  - State machine (STARTUP → READY → CATCHING → RETURNING → COOLDOWN → READY)
  - Console feedback (spawn info, feasibility result, catch outcome)

All catch pipeline components (HandCoordinator, FeasibilityChecker,
HandCatchSequence) are used as-is — this is purely a frontend.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from enum import Enum, auto

import numpy as np

import os, sys
_sim_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _sim_dir not in sys.path:
    sys.path.insert(0, _sim_dir)

from hand.coordinator import HandCoordinator, DynamicTarget, BallSpawn, HandPhase
from hand.trajectory import HandCatchSequence
from input.scripted import _compute_catch_target, _ball_landing
from input.sim_control import SimController
from plant.interface import PlantState

logger = logging.getLogger(__name__)

import mujoco

# GLFW key codes (catch-specific only; sim control keys live in SimController)
_KEY_LEFT = 263
_KEY_PAGE_UP = 266
_KEY_PAGE_DOWN = 267
_KEY_B = 66           # Primary spawn (BB throw when --bb, else preset)
_KEY_G = 71           # Preset spawn (G key) — not used by MuJoCo
_KEY_N = 78
_KEY_M = 77
_KEY_1 = 49
_KEY_2 = 50
_KEY_3 = 51
_KEY_4 = 52
_KEY_5 = 53
_KEY_NUMPAD_8 = 328   # vz faster (more negative)
_KEY_NUMPAD_2 = 322   # vz slower (less negative)
_KEY_NUMPAD_4 = 324   # vxy X nudge -
_KEY_NUMPAD_6 = 326   # vxy X nudge +
_KEY_NUMPAD_7 = 327   # vxy Y nudge -
_KEY_NUMPAD_9 = 329   # vxy Y nudge +
_KEY_NUMPAD_1 = 321   # xy offset Y -
_KEY_NUMPAD_3 = 323   # xy offset Y +
_KEY_NUMPAD_0 = 320   # xy offset X +

# Adjustment increments
_HEIGHT_STEP = 200.0      # mm
_XY_STEP = 20.0           # mm
_VZ_STEP = 200.0          # mm/s
_VXY_STEP = 50.0          # mm/s


# ---------------------------------------------------------------------------
# Spawn presets
# ---------------------------------------------------------------------------

@dataclass
class SpawnPreset:
    """Named set of ball spawn parameters."""
    name: str
    spawn_height_mm: float       # Z above world origin
    xy_offset_mm: np.ndarray     # (2,) lateral offset from centre
    vz_mms: float                # initial Z velocity (negative = down)
    vxy_mms: np.ndarray          # (2,) horizontal velocity


DEFAULT_PRESETS = [
    SpawnPreset("Centre drop (rest)", 1500.0, np.array([0.0, 0.0]),
                0.0, np.array([0.0, 0.0])),
    SpawnPreset("Offset drop", 1500.0, np.array([40.0, -30.0]),
                -500.0, np.array([0.0, 0.0])),
    SpawnPreset("Fast drop", 1500.0, np.array([0.0, 0.0]),
                -1500.0, np.array([0.0, 0.0])),
    SpawnPreset("Angled throw", 1500.0, np.array([20.0, 0.0]),
                -300.0, np.array([-100.0, 50.0])),
    SpawnPreset("Edge case", 1500.0, np.array([80.0, -60.0]),
                -500.0, np.array([0.0, 0.0])),
]


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

class InteractiveCatchState(Enum):
    STARTUP = auto()     # Ramping to active height + priming hand
    READY = auto()       # Waiting for user to spawn a ball
    CATCHING = auto()    # Catch coordinator running
    RETURNING = auto()   # Returning to ready pose after catch attempt
    COOLDOWN = auto()    # Brief pause before accepting next spawn


# Ready pose: platform at default active Z, hand primed
_READY_POSE = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
_STARTUP_DURATION = 1.0   # seconds to ramp from active pose to ready pose
_RETURN_DURATION = 0.3    # seconds to return to ready pose
_COOLDOWN_DURATION = 0.1  # seconds before accepting next spawn
_CATCH_TIMEOUT = 5.0      # max seconds for a catch attempt


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------

class InteractiveCatchController:
    """Interactive ball catch controller for the MuJoCo viewer.

    Parameters
    ----------
    feasibility_checker
        The ``FeasibilityChecker`` instance for platform feasibility.
    presets : list[SpawnPreset] | None
        Ball spawn presets.  Defaults to ``DEFAULT_PRESETS``.
    """

    def __init__(self, feasibility_checker=None, presets: list[SpawnPreset] | None = None,
                 active_pose: np.ndarray | None = None, ball_butler_sim=None):
        self._feasibility = feasibility_checker
        self._active_pose = active_pose
        self._presets = presets or list(DEFAULT_PRESETS)
        self._bb_sim = ball_butler_sim  # Optional BallButlerSim for T-key throws
        self._preset_idx = 0

        # Sim control — delegate to SimController (pause/step/speed)
        self._sim_ctrl = SimController()

        # State machine
        self._state = InteractiveCatchState.STARTUP
        self._state_start_time = 0.0
        self._spawn_count = 0

        # Startup target (set on first update)
        self._startup_target: DynamicTarget | None = None
        self._startup_built = False

        # Catch state
        self._coordinator: HandCoordinator | None = None
        self._active_hand_seq: HandCatchSequence | None = None
        self._home_pending = False
        self._pending_spawn: BallSpawn | None = None
        self._spawn_requested = False
        self._bb_spawn_requested = False
        self._rejected_spawn: BallSpawn | None = None

        # Return target
        self._return_target: DynamicTarget | None = None

        self._printed_ready = False

    # ------------------------------------------------------------------
    # Keyboard callback
    # ------------------------------------------------------------------

    def key_callback(self, keycode: int) -> None:
        """Handle keyboard events from the MuJoCo viewer.

        Sim-control keys (Space, arrows, R) are delegated to SimController.
        Remaining keys are catch-specific (spawn, presets, adjustments).
        """
        # Delegate sim control keys (pause/step/speed)
        self._sim_ctrl.key_callback(keycode)

        # Spawn: B is the primary action.
        #   --bb mode:  B = Ball Butler throw,  ` = preset spawn
        #   no --bb:    B = preset spawn
        if keycode == _KEY_B:
            if self._state != InteractiveCatchState.READY:
                print(f"  (ignoring spawn — state is {self._state.name})")
                return
            if self._bb_sim is not None:
                self._bb_spawn_requested = True
            else:
                self._spawn_requested = True
            return

        # G key always triggers preset spawn (useful when --bb is active)
        if keycode == _KEY_G:
            if self._state == InteractiveCatchState.READY:
                self._spawn_requested = True
            else:
                print(f"  (ignoring spawn — state is {self._state.name})")
            return

        # Preset cycling
        if keycode == _KEY_N:
            self._preset_idx = (self._preset_idx + 1) % len(self._presets)
            self._print_preset()
            return
        if keycode == _KEY_M:
            self._preset_idx = (self._preset_idx - 1) % len(self._presets)
            self._print_preset()
            return

        # Direct preset selection (1-5)
        if _KEY_1 <= keycode <= _KEY_5:
            idx = keycode - _KEY_1
            if idx < len(self._presets):
                self._preset_idx = idx
                self._print_preset()
            return

        # Adjust spawn position
        p = self._presets[self._preset_idx]
        if keycode == _KEY_PAGE_UP:
            p.spawn_height_mm += _HEIGHT_STEP
            self._print_preset()
            return
        if keycode == _KEY_PAGE_DOWN:
            p.spawn_height_mm = max(500.0, p.spawn_height_mm - _HEIGHT_STEP)
            self._print_preset()
            return
        if keycode == _KEY_LEFT:
            p.xy_offset_mm[0] -= _XY_STEP
            self._print_preset()
            return
        if keycode == _KEY_NUMPAD_0:
            p.xy_offset_mm[0] += _XY_STEP
            self._print_preset()
            return
        if keycode == _KEY_NUMPAD_1:
            p.xy_offset_mm[1] -= _XY_STEP
            self._print_preset()
            return
        if keycode == _KEY_NUMPAD_3:
            p.xy_offset_mm[1] += _XY_STEP
            self._print_preset()
            return

        # Adjust spawn velocity (numpad)
        if keycode == _KEY_NUMPAD_8:
            p.vz_mms -= _VZ_STEP  # more negative = faster downward
            self._print_preset()
            return
        if keycode == _KEY_NUMPAD_2:
            p.vz_mms += _VZ_STEP  # more positive = faster upward
            self._print_preset()
            return
        if keycode == _KEY_NUMPAD_4:
            p.vxy_mms[0] -= _VXY_STEP
            self._print_preset()
            return
        if keycode == _KEY_NUMPAD_6:
            p.vxy_mms[0] += _VXY_STEP
            self._print_preset()
            return
        if keycode == _KEY_NUMPAD_7:
            p.vxy_mms[1] -= _VXY_STEP
            self._print_preset()
            return
        if keycode == _KEY_NUMPAD_9:
            p.vxy_mms[1] += _VXY_STEP
            self._print_preset()
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
    ) -> tuple[DynamicTarget | None, str | HandCatchSequence | None, BallSpawn | None]:
        """Advance the state machine.

        Returns
        -------
        target : DynamicTarget or None
            Target for MPC this step.
        hand_cmd : str, HandCatchSequence, or None
            'prime' / 'home' / HandCatchSequence / None.
        ball_spawn : BallSpawn or None
            If a ball should be spawned this step.
        """
        hand_cmd = None
        ball_spawn = None

        # Ready-pose target (ASAP hold)
        ready_target = DynamicTarget(pose_6dof=_READY_POSE.copy())

        # -- STARTUP: ramp to active height --
        if self._state == InteractiveCatchState.STARTUP:
            if not self._startup_built:
                self._startup_target = DynamicTarget(
                    pose_6dof=_READY_POSE.copy(),
                    arrival_time=sim_time + _STARTUP_DURATION,
                )
                self._state_start_time = sim_time
                self._startup_built = True
                hand_cmd = 'prime'

            elapsed = sim_time - self._state_start_time
            if elapsed >= _STARTUP_DURATION + 0.2:
                self._state = InteractiveCatchState.READY
                self._state_start_time = sim_time
                self._print_ready()
                return ready_target, None, None

            return self._startup_target, hand_cmd, None

        # -- READY: hold at ready pose, wait for spawn --
        if self._state == InteractiveCatchState.READY:
            if not self._printed_ready:
                self._print_ready()
                self._printed_ready = True

            if self._bb_spawn_requested:
                self._bb_spawn_requested = False
                ball_spawn = self._do_bb_spawn(sim_time, current_pose, hand_pos_mm,
                                               plant_state=plant_state)
                if ball_spawn is not None:
                    # Transition to CATCHING (same flow as preset spawn)
                    self._state = InteractiveCatchState.CATCHING
                    self._state_start_time = sim_time
                    self._printed_ready = False

                    target, hcmd = self._coordinator.update(
                        sim_time, current_pose, hand_pos_mm=hand_pos_mm,
                        plant_state=plant_state)
                    if isinstance(hcmd, HandCatchSequence):
                        self._active_hand_seq = hcmd
                    elif hcmd == 'prime':
                        hand_cmd = 'prime'
                    return target or ready_target, hand_cmd, ball_spawn
                elif self._rejected_spawn is not None:
                    rejected = self._rejected_spawn
                    self._rejected_spawn = None
                    return ready_target, None, rejected

            if self._spawn_requested:
                self._spawn_requested = False
                ball_spawn = self._do_spawn(sim_time, current_pose, hand_pos_mm,
                                            plant_state=plant_state)
                if ball_spawn is not None:
                    # Transition to CATCHING
                    self._state = InteractiveCatchState.CATCHING
                    self._state_start_time = sim_time
                    self._printed_ready = False

                    # Get first coordinator update
                    target, hcmd = self._coordinator.update(
                        sim_time, current_pose, hand_pos_mm=hand_pos_mm,
                        plant_state=plant_state)
                    if isinstance(hcmd, HandCatchSequence):
                        self._active_hand_seq = hcmd
                    elif hcmd == 'prime':
                        hand_cmd = 'prime'
                    return target or ready_target, hand_cmd, ball_spawn
                elif self._rejected_spawn is not None:
                    # Ball was rejected but still spawn it visually
                    rejected = self._rejected_spawn
                    self._rejected_spawn = None
                    return ready_target, None, rejected

            return ready_target, None, None

        # -- CATCHING: delegate to coordinator --
        if self._state == InteractiveCatchState.CATCHING:
            # Check ball spawn from coordinator
            if self._coordinator is not None:
                spawn = self._coordinator.should_spawn_ball(sim_time)
                if spawn is not None:
                    ball_spawn = spawn

                target, hcmd = self._coordinator.update(
                    sim_time, current_pose, hand_pos_mm=hand_pos_mm,
                    plant_state=plant_state)

                if isinstance(hcmd, HandCatchSequence):
                    self._active_hand_seq = hcmd
                elif hcmd == 'prime':
                    hand_cmd = 'prime'
                elif hcmd == 'home':
                    # Defer 'home' until the active catch sequence finishes
                    # naturally — sending it now would snap the hand to 0.
                    self._home_pending = True

                # Sample active hand trajectory
                if self._active_hand_seq is not None:
                    pos = self._active_hand_seq.sample(sim_time)
                    if pos is not None:
                        hand_cmd = pos  # float position
                    else:
                        self._active_hand_seq = None
                        # Sequence finished — now send the deferred home
                        if self._home_pending:
                            hand_cmd = 'home'
                            self._home_pending = False

                # Check if coordinator is done — but don't exit while the
                # hand catch sequence is still active (the coordinator may
                # go idle before the hand finishes its catch motion).
                elapsed = sim_time - self._state_start_time
                hand_done = self._active_hand_seq is None
                coordinator_idle = (self._coordinator.phase == HandPhase.IDLE
                                    and elapsed > 0.1
                                    and hand_done)
                timed_out = elapsed > _CATCH_TIMEOUT

                if coordinator_idle or timed_out:
                    self._print_catch_result(timed_out)
                    self._state = InteractiveCatchState.RETURNING
                    self._state_start_time = sim_time
                    self._return_target = DynamicTarget(
                        pose_6dof=_READY_POSE.copy(),
                        arrival_time=sim_time + _RETURN_DURATION,
                    )
                    self._active_hand_seq = None
                    return self._return_target, 'prime', None

                return target or ready_target, hand_cmd, ball_spawn

        # -- RETURNING: ramp back to ready pose --
        if self._state == InteractiveCatchState.RETURNING:
            elapsed = sim_time - self._state_start_time
            if elapsed >= _RETURN_DURATION + 0.2:
                self._state = InteractiveCatchState.COOLDOWN
                self._state_start_time = sim_time
            return self._return_target or ready_target, None, None

        # -- COOLDOWN: brief pause --
        if self._state == InteractiveCatchState.COOLDOWN:
            elapsed = sim_time - self._state_start_time
            if elapsed >= _COOLDOWN_DURATION:
                self._state = InteractiveCatchState.READY
                self._state_start_time = sim_time
                self._printed_ready = False
            return ready_target, None, None

        return ready_target, None, None

    # ------------------------------------------------------------------
    # Ball spawning
    # ------------------------------------------------------------------

    def _do_spawn(
        self,
        sim_time: float,
        current_pose: np.ndarray,
        hand_pos_mm: float,
        plant_state: PlantState | None = None,
    ) -> BallSpawn | None:
        """Compute catch target from current preset and create coordinator.

        Returns the BallSpawn if the catch is feasible, or None if rejected.
        """
        preset = self._presets[self._preset_idx]
        self._spawn_count += 1

        spawn_pos = np.array([
            preset.xy_offset_mm[0],
            preset.xy_offset_mm[1],
            preset.spawn_height_mm,
        ])
        spawn_vel = np.array([
            preset.vxy_mms[0],
            preset.vxy_mms[1],
            preset.vz_mms,
        ])
        spawn_time = sim_time + 0.05  # 50ms delay

        # Compute catch target from ballistics
        try:
            target, ball_spawn = _compute_catch_target(
                spawn_pos, spawn_vel, spawn_time,
                active_z_offset_mm=_READY_POSE[2])
        except ValueError as e:
            print(f"\n=== Ball #{self._spawn_count} ===")
            print(f"  Preset: \"{preset.name}\"")
            print(f"  REJECTED: ballistic prediction failed — {e}")
            return None

        # Print spawn info
        print(f"\n=== Ball #{self._spawn_count} ===")
        print(f"  Preset: \"{preset.name}\"")
        print(f"  Spawn: pos={_fmt_vec(spawn_pos)} mm, "
              f"vel={_fmt_vec(spawn_vel)} mm/s")

        flight_time = target.arrival_time - spawn_time
        print(f"  Flight time: {flight_time:.3f}s, "
              f"arrival at t={target.arrival_time:.3f}s")
        print(f"  Target pose: {_fmt_vec(target.pose_6dof)}")
        if target.event_vel_mps is not None:
            print(f"  Event velocity: {target.event_vel_mps:.2f} m/s")

        # Apply hand catch lead time: in MuJoCo sim the ball contacts the
        # hand's physical geometry ~80ms before the analytical arrival time
        # (collision at hand cup top vs predicted mid-catch position).
        # Shift the hand sequence earlier so ball and hand meet at mid-stroke.
        if target.event_vel_mps is not None and target.arrival_time is not None:
            from hand.coordinator import _hand_catch_lead_time
            lead = _hand_catch_lead_time(target.event_vel_mps)
            target = DynamicTarget(
                pose_6dof=target.pose_6dof,
                arrival_time=target.arrival_time - lead,
                arrival_twist=target.arrival_twist,
                hold_duration=target.hold_duration + lead,  # extend hold to compensate
                event_vel_mps=target.event_vel_mps,
                settle_margin_s=target.settle_margin_s,
                mode=target.mode,
            )

        # Create fresh coordinator with feasibility checker
        self._coordinator = HandCoordinator(
            active_pose=self._active_pose,
            feasibility_checker=self._feasibility)
        self._coordinator.submit_target(target, ball_spawn)

        # Force the coordinator to process the target (advances to next)
        # by calling update once — this triggers feasibility + hand timing
        target_out, hcmd = self._coordinator.update(
            sim_time, current_pose, hand_pos_mm=hand_pos_mm,
            plant_state=plant_state)

        # Check if it was rejected by feasibility
        events = self._coordinator.events
        if events and events[-1].arrival_error_mm == -1.0:
            print(f"  Feasibility: REJECTED (ball still released)")
            self._coordinator = None
            # Return the ball_spawn so the ball is still shown in the viewer,
            # but return None from the outer caller to stay in READY state
            self._rejected_spawn = ball_spawn
            return None

        print(f"  Feasibility: ACCEPTED")

        # Mark ball as already spawned so should_spawn_ball() doesn't re-trigger
        # (main.py spawns the ball immediately from our returned ball_spawn)
        self._coordinator._ball_spawned = True

        return ball_spawn

    def _do_bb_spawn(
        self,
        sim_time: float,
        current_pose: np.ndarray,
        hand_pos_mm: float,
        plant_state: PlantState | None = None,
    ) -> BallSpawn | None:
        """Spawn a ball from Ball Butler using realistic throw kinematics.

        Same flow as _do_spawn but uses BallButlerSim to compute the ball's
        release state instead of using a preset.
        """
        self._spawn_count += 1
        spawn_time = sim_time + 0.05

        try:
            ball_spawn_raw = self._bb_sim.throw_at_jugglebot(
                spawn_time=spawn_time,
                scatter_mm=0.0,
            )
        except ValueError as e:
            print(f"\n=== Ball #{self._spawn_count} (BB throw) ===")
            print(f"  REJECTED: Ball Butler throw failed — {e}")
            return None

        # Compute catch target from ball trajectory
        try:
            target, ball_spawn = _compute_catch_target(
                ball_spawn_raw.position_mm, ball_spawn_raw.velocity_mms,
                ball_spawn_raw.spawn_time,
                active_z_offset_mm=_READY_POSE[2])
        except ValueError as e:
            print(f"\n=== Ball #{self._spawn_count} (BB throw) ===")
            print(f"  REJECTED: catch target computation failed — {e}")
            return None

        # Print throw info
        speed_mps = float(np.linalg.norm(ball_spawn_raw.velocity_mms)) / 1000.0
        flight_time = target.arrival_time - spawn_time
        print(f"\n=== Ball #{self._spawn_count} (BB throw) ===")
        print(f"  Release: pos={_fmt_vec(ball_spawn_raw.position_mm)} mm, "
              f"speed={speed_mps:.2f} m/s")
        print(f"  Flight time: {flight_time:.3f}s, "
              f"arrival at t={target.arrival_time:.3f}s")
        print(f"  Target pose: {_fmt_vec(target.pose_6dof)}")
        if target.event_vel_mps is not None:
            print(f"  Event velocity: {target.event_vel_mps:.2f} m/s")

        # Apply hand catch lead time (same as _do_spawn)
        if target.event_vel_mps is not None and target.arrival_time is not None:
            from hand.coordinator import _hand_catch_lead_time
            lead = _hand_catch_lead_time(target.event_vel_mps)
            target = DynamicTarget(
                pose_6dof=target.pose_6dof,
                arrival_time=target.arrival_time - lead,
                arrival_twist=target.arrival_twist,
                hold_duration=target.hold_duration + lead,
                event_vel_mps=target.event_vel_mps,
                settle_margin_s=target.settle_margin_s,
                mode=target.mode,
            )

        # Create coordinator and submit
        self._coordinator = HandCoordinator(
            active_pose=self._active_pose,
            feasibility_checker=self._feasibility)
        self._coordinator.submit_target(target, ball_spawn)

        target_out, hcmd = self._coordinator.update(
            sim_time, current_pose, hand_pos_mm=hand_pos_mm,
            plant_state=plant_state)

        events = self._coordinator.events
        if events and events[-1].arrival_error_mm == -1.0:
            print(f"  Feasibility: REJECTED (ball still released)")
            self._coordinator = None
            self._rejected_spawn = ball_spawn
            return None

        print(f"  Feasibility: ACCEPTED")

        # Mark ball as already spawned so should_spawn_ball() doesn't re-trigger
        self._coordinator._ball_spawned = True

        return ball_spawn

    # ------------------------------------------------------------------
    # Console output
    # ------------------------------------------------------------------

    def _print_preset(self) -> None:
        p = self._presets[self._preset_idx]
        print(f"  Preset [{self._preset_idx + 1}/{len(self._presets)}]: "
              f"\"{p.name}\"")
        print(f"    pos: xy={_fmt_vec(p.xy_offset_mm)}mm, "
              f"height={p.spawn_height_mm:.0f}mm")
        print(f"    vel: vz={p.vz_mms:.0f}mm/s, "
              f"vxy={_fmt_vec(p.vxy_mms)}mm/s")

    def _print_ready(self) -> None:
        p = self._presets[self._preset_idx]
        if self._bb_sim is not None:
            print(f"\n  READY — Press B for Ball Butler throw, G for preset spawn")
        else:
            print(f"\n  READY — Press B to spawn ball")
        print(f"  Preset [{self._preset_idx + 1}/{len(self._presets)}]: "
              f"\"{p.name}\"")
        print(f"    pos: xy={_fmt_vec(p.xy_offset_mm)}mm, "
              f"height={p.spawn_height_mm:.0f}mm")
        print(f"    vel: vz={p.vz_mms:.0f}mm/s, "
              f"vxy={_fmt_vec(p.vxy_mms)}mm/s")

    def notify_capture(self, sim_time: float) -> None:
        """Notify the controller that a ball was captured."""
        if (self._coordinator is not None
                and self._state == InteractiveCatchState.CATCHING):
            self._coordinator.notify_capture(sim_time)

    def _print_catch_result(self, timed_out: bool) -> None:
        if self._coordinator is None:
            return

        events = self._coordinator.events
        if timed_out:
            print(f"  Result: TIMED OUT ({_CATCH_TIMEOUT:.0f}s)")
        elif not events:
            print(f"  Result: MISSED — no events recorded")
        else:
            for ev in events:
                if ev.captured:
                    print(f"  Result: CAUGHT at t={ev.time:.3f}s "
                          f"(pos_err={ev.arrival_error_mm:.1f}mm, "
                          f"timing_err={ev.arrival_timing_error_s*1000:+.0f}ms)")
                elif ev.arrival_error_mm == -1.0:
                    pass  # already printed as REJECTED
                else:
                    print(f"  Result: MISSED — {ev.phase.name} at t={ev.time:.3f}s "
                          f"(pos_err={ev.arrival_error_mm:.1f}mm)")
        print()

    # ------------------------------------------------------------------
    # 3D preview rendering
    # ------------------------------------------------------------------

    def render_preview(self, viewer) -> None:
        """Draw a ghost ball and velocity arrow at the current spawn position.

        Called each frame by the viewer loop.  Only draws while in READY state.
        Adds geoms to ``viewer.user_scn`` (shared with HorizonRenderer, which
        sets ngeom=0 at the start of its render — so call this BEFORE horizon).
        """
        if self._state != InteractiveCatchState.READY:
            return

        p = self._presets[self._preset_idx]
        # Spawn position in world frame (metres)
        ball_pos_m = np.array([
            p.xy_offset_mm[0] / 1000.0,
            p.xy_offset_mm[1] / 1000.0,
            p.spawn_height_mm / 1000.0,
        ])

        # Velocity vector for label + arrow
        vel_mms = np.array([p.vxy_mms[0], p.vxy_mms[1], p.vz_mms])
        speed_mms = np.linalg.norm(vel_mms)

        scn = viewer.user_scn
        if scn.ngeom >= scn.maxgeom - 2:
            return  # not enough room

        # Ghost ball — translucent orange sphere with label
        geom = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([0.02, 0.0, 0.0]),      # radius 20mm
            ball_pos_m,
            np.eye(3).flatten(),
            np.array([1.0, 0.5, 0.0, 0.4], dtype=np.float32),  # orange, translucent
        )
        label = f"[B] {p.name}  |v|={speed_mms:.0f}mm/s"
        geom.label = label[:99]  # MuJoCo label buffer limit
        scn.ngeom += 1

        # Velocity arrow
        if speed_mms < 1.0:
            return  # no arrow for near-zero velocity

        vel_dir = vel_mms / speed_mms
        # Arrow length: scale speed to a visible length (1000 mm/s → 0.3m arrow)
        arrow_len_m = min(speed_mms / 1000.0 * 0.3, 0.8)

        # Arrow endpoint
        arrow_end_m = ball_pos_m + vel_dir * arrow_len_m

        # MuJoCo ARROW geom: needs 'from' and 'to' positions.
        # mjv_initGeom with ARROW isn't straightforward — use a connector.
        # Instead, use mjv_connector which draws a line/arrow between two points.
        if scn.ngeom >= scn.maxgeom:
            return

        geom2 = scn.geoms[scn.ngeom]
        from_pt = np.array(ball_pos_m, dtype=np.float64)
        to_pt = np.array(arrow_end_m, dtype=np.float64)
        mujoco.mjv_connector(
            geom2,
            mujoco.mjtGeom.mjGEOM_ARROW,
            0.005,  # width (5mm radius)
            from_pt, to_pt,
        )
        geom2.rgba[:] = [1.0, 0.3, 0.0, 0.7]
        scn.ngeom += 1


def _fmt_vec(v: np.ndarray) -> str:
    """Format a small vector for console output."""
    return '[' + ', '.join(f'{x:.1f}' for x in v) + ']'
