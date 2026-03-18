"""Catch coordinator: manages dynamic targets, hand priming, and ball lifecycle.

The coordinator receives DynamicTarget commands and generates time-aware
reference trajectories for the MPC.  It also coordinates hand priming and
ball capture/release, mirroring production's catch_coordinator_node.py.

The MPC controls only the 6 Stewart platform legs.  The hand is commanded
separately (matching production where the Teensy handles hand trajectories).
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from enum import Enum, auto

import numpy as np

import os, sys
_sim_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _sim_dir not in sys.path:
    sys.path.insert(0, _sim_dir)

from controller.reference import ReferenceGenerator
from jugglebot.motion.quintic import create_trajectory, evaluate as quintic_evaluate
from catch.hand_trajectory import HandCatchSequence

logger = logging.getLogger(__name__)


@dataclass
class DynamicTarget:
    """A timed target for the platform to reach.

    Matches the Phase 7 production API: pose + arrival deadline + optional twist.
    """
    pose_6dof: np.ndarray       # [x, y, z, rx, ry, rz] target pose (mm, rad)
    arrival_time: float          # absolute sim time by which platform must be at pose
    arrival_twist: np.ndarray | None = None  # (6,) twist at arrival; None = catch (hold)
    hold_duration: float = 0.5   # seconds to hold after arrival (catch mode only)
    event_vel_mps: float | None = None  # ball landing speed (m/s) for hand trajectory


@dataclass
class BallSpawn:
    """Ball spawn parameters for a scripted test sequence."""
    position_mm: np.ndarray      # (3,) world frame spawn position
    velocity_mms: np.ndarray     # (3,) spawn velocity
    spawn_time: float            # absolute sim time to spawn the ball


class CatchPhase(Enum):
    """State machine for the catch coordinator."""
    IDLE = auto()
    PRIMING = auto()       # Hand moving to prime position
    APPROACHING = auto()   # Platform moving to catch pose
    HOLDING = auto()       # Holding at catch pose, waiting for ball
    CAUGHT = auto()        # Ball captured, retracting hand
    THROWING = auto()      # Platform at throw velocity, releasing ball
    DECELERATING = auto()  # After throw, decelerating to stop
    RETURNING = auto()     # Returning to home


@dataclass
class CatchEvent:
    """Record of a catch/throw event for telemetry."""
    target_idx: int
    phase: CatchPhase
    time: float
    arrival_error_mm: float = 0.0
    arrival_timing_error_s: float = 0.0
    captured: bool = False


class CatchCoordinator:
    """Coordinates platform motion, hand priming, and ball capture.

    Usage in the simulation loop:
        1. Call ``submit_target()`` to queue a DynamicTarget
        2. Each step, call ``update(sim_time, state)`` to get:
           - Reference trajectory for the MPC
           - Hand command (if any)
           - Ball actions (spawn/capture check)
        3. Query ``events`` for telemetry logging
    """

    def __init__(self, home_pose: np.ndarray | None = None, feasibility_checker=None):
        self._home = home_pose if home_pose is not None else np.zeros(6)
        self._feasibility = feasibility_checker  # FeasibilityChecker or None
        self._targets: list[tuple[DynamicTarget, BallSpawn | None]] = []
        self._current_idx = -1
        self._phase = CatchPhase.IDLE
        self._ref_gen: ReferenceGenerator | None = None
        self._phase_start_time = 0.0
        self._events: list[CatchEvent] = []
        self._hand_primed = False
        self._hand_catch_seq: HandCatchSequence | None = None
        self._ball_spawned = False
        self._caught = False
        self._hold_end_time = 0.0
        self._decel_ref: ReferenceGenerator | None = None

    def submit_target(
        self,
        target: DynamicTarget,
        ball_spawn: BallSpawn | None = None,
    ) -> None:
        """Queue a dynamic target with optional ball spawn."""
        self._targets.append((target, ball_spawn))

    @property
    def phase(self) -> CatchPhase:
        return self._phase

    @property
    def events(self) -> list[CatchEvent]:
        return self._events

    @property
    def hand_primed(self) -> bool:
        return self._hand_primed

    @property
    def is_catch_mode(self) -> bool:
        """True when the current target expects zero arrival velocity (catch)."""
        if self._current_idx < 0 or self._current_idx >= len(self._targets):
            return True
        target = self._targets[self._current_idx][0]
        return target.arrival_twist is None or np.allclose(target.arrival_twist, 0)

    def update(
        self,
        sim_time: float,
        current_pose: np.ndarray,
        hand_pos_mm: float = 0.0,
    ) -> tuple[ReferenceGenerator | None, str | HandCatchSequence | None]:
        """Advance the coordinator state machine.

        Parameters
        ----------
        sim_time : float
            Current simulation time.
        current_pose : np.ndarray
            Current platform pose [x, y, z, rx, ry, rz].
        hand_pos_mm : float
            Current hand position in mm (needed for hand trajectory planning).

        Returns
        -------
        ref_gen : ReferenceGenerator or None
            Reference for MPC this step.  None means hold current pose.
        hand_cmd : str, HandCatchSequence, or None
            'prime' / 'home' / HandCatchSequence / None.
        """
        hand_cmd = None

        # Check if it's time to advance to the next target
        if self._phase == CatchPhase.IDLE:
            if self._current_idx + 1 < len(self._targets):
                self._advance_to_next(sim_time, current_pose, hand_pos_mm)
            return self._ref_gen, hand_cmd

        target, ball_spawn = self._targets[self._current_idx]

        # Prime hand on first non-idle step
        if not self._hand_primed and self.is_catch_mode:
            hand_cmd = 'prime'
            self._hand_primed = True

        # Spawn ball at scheduled time
        if ball_spawn is not None and not self._ball_spawned:
            if sim_time >= ball_spawn.spawn_time:
                self._ball_spawned = True
                # Ball spawning is handled externally — we just set the flag

        # State machine transitions
        if self._phase == CatchPhase.PRIMING:
            # Transition to approaching once we have a trajectory
            self._phase = CatchPhase.APPROACHING

        elif self._phase == CatchPhase.APPROACHING:
            # Return the hand catch sequence on the first step it becomes
            # relevant (the main loop samples it every step thereafter).
            if self.is_catch_mode and self._hand_catch_seq is not None:
                if hand_cmd is None:  # don't overwrite 'prime'
                    hand_cmd = self._hand_catch_seq
                    self._hand_catch_seq = None  # only return once

            if sim_time >= target.arrival_time:
                if self.is_catch_mode:
                    self._phase = CatchPhase.HOLDING
                    self._hold_end_time = sim_time + target.hold_duration
                    self._events.append(CatchEvent(
                        target_idx=self._current_idx,
                        phase=CatchPhase.HOLDING,
                        time=sim_time,
                        arrival_error_mm=float(np.linalg.norm(
                            current_pose[:3] - target.pose_6dof[:3])),
                        arrival_timing_error_s=sim_time - target.arrival_time,
                    ))
                else:
                    # Throw mode
                    self._phase = CatchPhase.THROWING
                    self._events.append(CatchEvent(
                        target_idx=self._current_idx,
                        phase=CatchPhase.THROWING,
                        time=sim_time,
                        arrival_error_mm=float(np.linalg.norm(
                            current_pose[:3] - target.pose_6dof[:3])),
                        arrival_timing_error_s=sim_time - target.arrival_time,
                    ))

        elif self._phase == CatchPhase.HOLDING:
            if sim_time >= self._hold_end_time:
                # Done holding — return to home (or next target)
                self._start_return(sim_time, current_pose)

        elif self._phase == CatchPhase.CAUGHT:
            # Ball was captured — retract hand and return home
            hand_cmd = 'home'
            self._start_return(sim_time, current_pose)

        elif self._phase == CatchPhase.THROWING:
            # Build deceleration trajectory from current state
            self._build_deceleration(sim_time, current_pose, target)
            self._phase = CatchPhase.DECELERATING

        elif self._phase == CatchPhase.DECELERATING:
            if self._decel_ref and sim_time >= self._phase_start_time + 1.0:
                self._start_return(sim_time, current_pose)

        elif self._phase == CatchPhase.RETURNING:
            # Check if we've returned close to home
            if self._ref_gen and self._ref_gen.duration is not None:
                if sim_time >= self._phase_start_time + self._ref_gen.duration:
                    self._phase = CatchPhase.IDLE
                    self._hand_primed = False
                    # Try next target
                    if self._current_idx + 1 < len(self._targets):
                        self._advance_to_next(sim_time, current_pose)

        return self._ref_gen, hand_cmd

    def notify_capture(self, sim_time: float) -> None:
        """Called when ball capture is detected."""
        self._caught = True
        if self._phase in (CatchPhase.HOLDING, CatchPhase.APPROACHING):
            target = self._targets[self._current_idx][0]
            self._events.append(CatchEvent(
                target_idx=self._current_idx,
                phase=self._phase,
                time=sim_time,
                arrival_error_mm=0.0,  # ball was physically captured
                arrival_timing_error_s=sim_time - target.arrival_time,
                captured=True,
            ))
            self._phase = CatchPhase.CAUGHT

    def should_spawn_ball(self, sim_time: float) -> BallSpawn | None:
        """Check if a ball should be spawned this step.

        Returns the BallSpawn if it's time, None otherwise.
        """
        if self._current_idx < 0 or self._current_idx >= len(self._targets):
            return None
        _, ball_spawn = self._targets[self._current_idx]
        if ball_spawn is not None and not self._ball_spawned:
            if sim_time >= ball_spawn.spawn_time:
                self._ball_spawned = True
                return ball_spawn
        return None

    def _advance_to_next(
        self,
        sim_time: float,
        current_pose: np.ndarray,
        hand_pos_mm: float = 0.0,
    ) -> None:
        """Start tracking the next queued target."""
        self._current_idx += 1
        target, _ = self._targets[self._current_idx]
        self._ball_spawned = False
        self._caught = False
        self._hand_catch_seq = None
        self._phase_start_time = sim_time

        # Build approach trajectory
        time_to_arrival = max(target.arrival_time - sim_time, 0.1)

        # Feasibility check: can the MPC reach this target in time?
        if self._feasibility is not None:
            from plant.interface import PlantState
            state = PlantState(
                leg_extensions_mm=np.zeros(6),
                leg_velocities_mmps=np.zeros(6),
                platform_pos_mm=current_pose[:3].copy(),
                platform_rot=current_pose[3:].copy(),
                platform_twist=np.zeros(6),
                time=sim_time,
            )
            feasible, reason = self._feasibility.check(
                state, target.pose_6dof, time_to_arrival)
            if not feasible:
                logger.info(
                    "Target %d rejected: %s (time_avail=%.2fs)",
                    self._current_idx, reason, time_to_arrival)
                self._events.append(CatchEvent(
                    target_idx=self._current_idx,
                    phase=CatchPhase.IDLE,
                    time=sim_time,
                    arrival_error_mm=-1.0,  # sentinel for rejection
                    captured=False,
                ))
                self._phase = CatchPhase.IDLE
                return

        if self.is_catch_mode:
            # Build hand catch trajectory if event_vel is available.
            # Check timing feasibility BEFORE committing to the approach —
            # this is the second layer of feasibility (after the platform
            # IK + coarse-MPC check above).
            if target.event_vel_mps is not None:
                result = HandCatchSequence.try_create(
                    event_vel_mps=target.event_vel_mps,
                    arrival_time=target.arrival_time,
                    current_pos_mm=hand_pos_mm,
                    current_time=sim_time,
                )
                if result.feasible:
                    self._hand_catch_seq = result.sequence
                else:
                    logger.info(
                        "Target %d rejected (hand timing): %s",
                        self._current_idx, result.reason)
                    self._events.append(CatchEvent(
                        target_idx=self._current_idx,
                        phase=CatchPhase.IDLE,
                        time=sim_time,
                        arrival_error_mm=-1.0,  # sentinel for rejection
                        captured=False,
                    ))
                    self._phase = CatchPhase.IDLE
                    return

            # Catch: arrive with zero velocity
            self._ref_gen = self._build_catch_ref(
                sim_time, current_pose, target, time_to_arrival)
            self._phase = CatchPhase.APPROACHING
        else:
            # Throw: arrive with specified velocity
            self._ref_gen = self._build_throw_ref(
                sim_time, current_pose, target, time_to_arrival)
            self._phase = CatchPhase.APPROACHING

        if not self._hand_primed and self.is_catch_mode:
            self._phase = CatchPhase.PRIMING

    def _build_catch_ref(
        self,
        now: float,
        current_pose: np.ndarray,
        target: DynamicTarget,
        duration: float,
    ) -> ReferenceGenerator:
        """Build a quintic trajectory to arrive at target with zero velocity."""
        zeros = np.zeros(6)
        traj = create_trajectory(
            start_pose=current_pose.copy(),
            start_twist=zeros,
            start_accel=zeros,
            end_pose=target.pose_6dof.copy(),
            end_twist=zeros,
            end_accel=zeros,
            duration=duration,
            t_start=now,
        )

        def _eval(t):
            if t < now:
                return current_pose.copy(), zeros.copy()
            if t > now + duration + target.hold_duration:
                return target.pose_6dof.copy(), zeros.copy()
            if t > now + duration:
                # Hold phase
                return target.pose_6dof.copy(), zeros.copy()
            pose, twist, _ = quintic_evaluate(traj, t)
            return pose, twist

        total = duration + target.hold_duration + 1.0  # +1s for return
        return ReferenceGenerator.from_function(_eval, duration=total)

    def _build_throw_ref(
        self,
        now: float,
        current_pose: np.ndarray,
        target: DynamicTarget,
        duration: float,
    ) -> ReferenceGenerator:
        """Build a quintic trajectory to arrive at target with specified velocity."""
        zeros = np.zeros(6)
        arrival_twist = target.arrival_twist if target.arrival_twist is not None else zeros

        traj = create_trajectory(
            start_pose=current_pose.copy(),
            start_twist=zeros,
            start_accel=zeros,
            end_pose=target.pose_6dof.copy(),
            end_twist=arrival_twist.copy(),
            end_accel=zeros,
            duration=duration,
            t_start=now,
        )

        def _eval(t):
            if t < now:
                return current_pose.copy(), zeros.copy()
            if t > now + duration:
                # After arrival: extrapolate with arrival twist briefly,
                # then hold (MPC will decelerate)
                dt = t - (now + duration)
                if dt < 0.1:
                    pose = target.pose_6dof + arrival_twist * dt
                    return pose, arrival_twist.copy()
                return target.pose_6dof.copy(), zeros.copy()
            pose, twist, _ = quintic_evaluate(traj, t)
            return pose, twist

        return ReferenceGenerator.from_function(_eval, duration=duration + 1.0)

    def _build_deceleration(
        self,
        now: float,
        current_pose: np.ndarray,
        target: DynamicTarget,
    ) -> None:
        """Build a deceleration trajectory from the current throw velocity."""
        arrival_twist = target.arrival_twist if target.arrival_twist is not None else np.zeros(6)
        zeros = np.zeros(6)

        # Compute endpoint from current velocity (0.5s decel)
        decel_time = 0.5
        endpoint = current_pose.copy()
        endpoint[:3] += arrival_twist[:3] * decel_time * 0.5  # linear displacement
        # Keep endpoint Z reasonable
        endpoint[2] = max(endpoint[2], 0)

        traj = create_trajectory(
            start_pose=current_pose.copy(),
            start_twist=arrival_twist.copy(),
            start_accel=zeros,
            end_pose=endpoint,
            end_twist=zeros,
            end_accel=zeros,
            duration=decel_time,
            t_start=now,
        )
        self._phase_start_time = now

        def _eval(t):
            if t < now:
                return current_pose.copy(), arrival_twist.copy()
            if t > now + decel_time:
                return endpoint.copy(), zeros.copy()
            pose, twist, _ = quintic_evaluate(traj, t)
            return pose, twist

        self._decel_ref = ReferenceGenerator.from_function(_eval, duration=decel_time + 0.5)
        self._ref_gen = self._decel_ref

    def _start_return(self, now: float, current_pose: np.ndarray) -> None:
        """Build a return-to-home trajectory."""
        zeros = np.zeros(6)
        duration = 1.0

        traj = create_trajectory(
            start_pose=current_pose.copy(),
            start_twist=zeros,
            start_accel=zeros,
            end_pose=self._home.copy(),
            end_twist=zeros,
            end_accel=zeros,
            duration=duration,
            t_start=now,
        )
        self._phase_start_time = now

        def _eval(t):
            if t < now:
                return current_pose.copy(), zeros.copy()
            if t > now + duration:
                return self._home.copy(), zeros.copy()
            pose, twist, _ = quintic_evaluate(traj, t)
            return pose, twist

        self._ref_gen = ReferenceGenerator.from_function(_eval, duration=duration)
        self._phase = CatchPhase.RETURNING
