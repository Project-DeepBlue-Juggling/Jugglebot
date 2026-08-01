"""Hand coordinator: manages dynamic targets, hand priming, and ball lifecycle.

The coordinator receives DynamicTarget commands and returns them directly
to the main loop, which passes them to ``mpc.solve()``.  The MPC handles
all trajectory planning internally (variable-resolution horizon).

The MPC controls only the 6 Stewart platform legs.  The hand is commanded
separately (matching production where the Teensy handles hand trajectories).

Supports two flows:
- **Catch-only**: submit_target() with mode='catch' — existing DT1-DT5/DT8 flow.
- **Throw-catch**: submit_throw_catch(plan) — full throw → transit → catch cycle.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from enum import Enum, auto

import numpy as np

from sim.hand.trajectory import HandCatchSequence, HandThrowSequence
from sim.plant.interface import PlantState

logger = logging.getLogger(__name__)


@dataclass
class DynamicTarget:
    """A timed target for the platform to reach.

    Matches the Phase 7 production API: pose + arrival deadline + optional twist.
    """
    pose_6dof: np.ndarray       # [x, y, z, rx, ry, rz] target pose (mm, rad)
    arrival_time: float | None = None  # absolute sim time; None = ASAP
    arrival_twist: np.ndarray | None = None  # (6,) twist at arrival; None = catch (hold)
    hold_duration: float = 0.5   # seconds to hold after arrival (catch mode only)
    event_vel_mps: float | None = None  # ball landing speed (m/s) for hand trajectory
    settle_margin_s: float = 0.1  # platform arrives early by this amount to allow settling
    mode: str = 'catch'           # 'catch' or 'throw'


@dataclass
class BallSpawn:
    """Ball spawn parameters for a scripted test sequence."""
    position_mm: np.ndarray      # (3,) world frame spawn position
    velocity_mms: np.ndarray     # (3,) spawn velocity
    spawn_time: float            # absolute sim time to spawn the ball


@dataclass
class BallRelease:
    """Signal to release the ball with a specific velocity."""
    velocity_mms: np.ndarray   # (3,) world-frame ball velocity


class HandPhase(Enum):
    """State machine for the hand coordinator."""
    IDLE = auto()
    PRIMING = auto()            # Hand moving to prime position (catch-only legacy)
    APPROACHING = auto()        # Platform moving to catch pose (catch-only legacy)
    APPROACHING_THROW = auto()  # Platform moving to throw pose
    THROWING = auto()           # Hand executing throw trajectory
    TRANSITIONING = auto()      # Platform moving from throw to catch pose → HOLDING
    HOLDING = auto()            # At catch pose, waiting for ball
    CAUGHT = auto()             # Ball captured, retracting hand
    DECELERATING = auto()       # After throw, decelerating to stop (legacy)
    RETURNING = auto()          # Returning to active pose


@dataclass
class HandEvent:
    """Record of a catch/throw event for telemetry."""
    target_idx: int
    phase: HandPhase
    time: float
    arrival_error_mm: float = 0.0
    arrival_timing_error_s: float = 0.0
    captured: bool = False


# Proximity tolerances for arrival_time=None ("ASAP") targets.
# When no time deadline is set, transition once the platform is within these bounds.
_ARRIVAL_POS_TOL_MM = 5.0
_ARRIVAL_ROT_TOL_RAD = 0.02  # ~1.1 degrees

# Return-to-active-pose duration estimation.
# Conservative velocity (30% of MPC max 300 mm/s) for gentle motion.
_RETURN_VEL_MMPS = 90.0
_RETURN_ROT_SCALE_MM = 100.0   # mm-equivalent per radian of rotation
_RETURN_MIN_S = 1.0
_RETURN_MAX_S = 3.0
_RETURN_MARGIN_S = 0.3         # additive buffer for accel/decel phases
_RETURN_SAFETY_TIMEOUT_S = 4.0  # hard timeout if proximity never triggers


def _hand_catch_lead_time(event_vel_mps: float) -> float:
    """Seconds earlier the hand sequence should start relative to ball arrival.

    The catch ``arrival_time`` is computed for the ball reaching the hand's
    mid-catch position (hand_catch_offset ~ 65 mm above centroid).  But
    the ball first contacts the physical hand geometry at the top of the
    primed hand cup, which is ~184 mm higher in world frame.

    Calibrated from MuJoCo simulation sweeps: the speed-dependent geometric
    offset (184mm / ball_speed) is augmented by a fixed 30ms margin to
    account for physics substep granularity and collision geometry extent.
    At 3.7 m/s, this gives ~80ms total lead, which positions the hand at
    the velocity-hold zone (~200mm on slider) at ball contact.
    """
    _PHYSICAL_OFFSET_MM = 184.0
    _MARGIN_S = 0.025
    ball_speed_mms = max(event_vel_mps, 0.3) * 1000.0
    return _PHYSICAL_OFFSET_MM / ball_speed_mms + _MARGIN_S


def _arrived_by_proximity(current_pose: np.ndarray, target_pose: np.ndarray) -> bool:
    """Check if the platform is close enough to the target to count as arrived."""
    pos_err = np.linalg.norm(current_pose[:3] - target_pose[:3])
    rot_err = np.linalg.norm(current_pose[3:] - target_pose[3:])
    return pos_err < _ARRIVAL_POS_TOL_MM and rot_err < _ARRIVAL_ROT_TOL_RAD


class HandCoordinator:
    """Coordinates platform motion, hand priming, and ball capture.

    Usage in the simulation loop:
        1. Call ``submit_target()`` or ``submit_throw_catch()``
        2. Each step, call ``update(sim_time, state)`` to get:
           - DynamicTarget for the MPC (passed to ``mpc.solve()``)
           - Hand command (if any)
           - Ball actions (spawn/capture check)
        3. Query ``events`` for telemetry logging
    """

    def __init__(self, active_pose: np.ndarray | None = None, feasibility_checker=None):
        self._active = active_pose if active_pose is not None else np.zeros(6)
        self._feasibility = feasibility_checker  # FeasibilityChecker or None
        self._targets: list[tuple[DynamicTarget, BallSpawn | None]] = []
        self._current_idx = -1
        self._phase = HandPhase.IDLE
        self._current_target: DynamicTarget | None = None
        self._phase_start_time = 0.0
        self._events: list[HandEvent] = []
        self._hand_primed = False
        self._hand_catch_seq: HandCatchSequence | None = None
        self._ball_spawned = False
        self._caught = False
        self._hold_end_time = 0.0
        self._hold_expired = False  # Deferred: set True when hold times out,
                                    # acted on next update (gives notify_capture
                                    # a chance to fire on the same sim step).

        # Throw-catch plan state
        self._throw_catch_plan = None  # ThrowCatchPlan or None
        self._throw_released = False   # Has ball been released in current throw?
        self._throw_hand_sent = False  # Has throw hand sequence been sent?

    def submit_target(
        self,
        target: DynamicTarget,
        ball_spawn: BallSpawn | None = None,
    ) -> None:
        """Queue a dynamic target with optional ball spawn."""
        self._targets.append((target, ball_spawn))

    def submit_throw_catch(self, plan) -> None:
        """Queue a throw-catch plan.

        Internally queues the throw target first, then the catch target
        is queued after the throw completes.
        """
        self._throw_catch_plan = plan
        # Queue throw target (no ball spawn — ball is released via BallRelease)
        self._targets.append((plan.throw_target, None))
        # Queue catch target (no ball spawn — ball is already in flight from throw)
        self._targets.append((plan.catch_target, None))

    @property
    def phase(self) -> HandPhase:
        return self._phase

    @property
    def events(self) -> list[HandEvent]:
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
        # Check mode field first, then fall back to arrival_twist for legacy targets
        if target.mode == 'throw':
            return False
        if target.arrival_twist is not None and not np.allclose(target.arrival_twist, 0):
            return False
        return True

    def update(
        self,
        sim_time: float,
        current_pose: np.ndarray,
        hand_pos_mm: float = 0.0,
        plant_state: PlantState | None = None,
    ) -> tuple[DynamicTarget | None, object]:
        """Advance the coordinator state machine.

        Returns
        -------
        target : DynamicTarget or None
            Target for MPC this step.  None means hold current pose.
        hand_cmd : str, float, HandCatchSequence, HandThrowSequence,
                   BallRelease, or None
        """
        hand_cmd = None

        # Check if it's time to advance to the next target
        if self._phase == HandPhase.IDLE:
            if self._current_idx + 1 < len(self._targets):
                self._advance_to_next(sim_time, current_pose, hand_pos_mm,
                                      plant_state=plant_state)
            return self._current_target, hand_cmd

        target, ball_spawn = self._targets[self._current_idx]

        # ---- Throw-catch flow states ----

        if self._phase == HandPhase.APPROACHING_THROW:
            plan = self._throw_catch_plan
            if plan is not None:
                # Send throw hand sequence on first relevant step
                if not self._throw_hand_sent and hand_cmd is None:
                    hand_cmd = plan.throw_hand_seq
                    self._throw_hand_sent = True

                # Release ball once hand sequence is installed and release time
                # has arrived.  Separate elif ensures these don't fire on the
                # same step (sequence must be active_hand_seq first).
                elif self._throw_hand_sent and sim_time >= plan.ball_release_time:
                    # Release the ball
                    hand_cmd = BallRelease(velocity_mms=plan.ball_release_vel_mms.copy())
                    self._throw_released = True
                    self._phase = HandPhase.THROWING
                    self._phase_start_time = sim_time
                    self._events.append(HandEvent(
                        target_idx=self._current_idx,
                        phase=HandPhase.THROWING,
                        time=sim_time,
                        arrival_error_mm=float(np.linalg.norm(
                            current_pose[:3] - target.pose_6dof[:3])),
                    ))
            return self._current_target, hand_cmd

        if self._phase == HandPhase.THROWING:
            plan = self._throw_catch_plan
            if plan is not None:
                # Wait for the throw hand trajectory to finish its deceleration
                # before transitioning to the catch phase.
                if sim_time < plan.throw_hand_seq.end_time:
                    return self._current_target, hand_cmd
                # Throw trajectory complete — transition to catch
                self._phase = HandPhase.TRANSITIONING
                self._phase_start_time = sim_time

                # Advance to catch target
                if self._current_idx + 1 < len(self._targets):
                    self._current_idx += 1
                    catch_target, _ = self._targets[self._current_idx]
                    self._ball_spawned = False
                    self._caught = False

                    # Recheck catch feasibility — throw may have run long
                    feasible, reason = self._check_target_feasibility(
                        catch_target, sim_time, current_pose,
                        hand_pos_mm, is_catch=True,
                        plant_state=plant_state)
                    if not feasible:
                        logger.warning(
                            "Catch target infeasible after throw: %s — "
                            "returning to active pose", reason)
                        self._events.append(HandEvent(
                            target_idx=self._current_idx,
                            phase=HandPhase.TRANSITIONING,
                            time=sim_time,
                            arrival_error_mm=-1.0,  # sentinel
                        ))
                        self._start_return(sim_time, current_pose)
                        return self._current_target, hand_cmd

                    # Set MPC target to catch pose
                    self._current_target = DynamicTarget(
                        pose_6dof=catch_target.pose_6dof.copy(),
                        arrival_time=catch_target.arrival_time,
                        arrival_twist=None,
                        hold_duration=catch_target.hold_duration,
                        event_vel_mps=catch_target.event_vel_mps,
                    )

                    # Send catch hand sequence
                    hand_cmd = plan.catch_hand_seq
            else:
                # Legacy throw (no plan) — decelerate
                self._phase = HandPhase.DECELERATING
                self._phase_start_time = sim_time
                self._current_target = DynamicTarget(
                    pose_6dof=current_pose.copy(),
                    arrival_time=sim_time + 0.5,
                    arrival_twist=np.zeros(6),
                )
            return self._current_target, hand_cmd

        if self._phase == HandPhase.TRANSITIONING:
            # Wait for platform to reach catch pose, then go straight to HOLDING
            catch_target = self._targets[self._current_idx][0]

            arrived_by_time = catch_target.arrival_time is not None and sim_time >= catch_target.arrival_time
            arrived_by_pos = catch_target.arrival_time is None and _arrived_by_proximity(current_pose, catch_target.pose_6dof)
            if arrived_by_time or arrived_by_pos:
                self._phase = HandPhase.HOLDING
                self._phase_start_time = sim_time
                self._hold_end_time = sim_time + catch_target.hold_duration
                self._hold_expired = False
                self._current_target = DynamicTarget(
                    pose_6dof=catch_target.pose_6dof.copy(),
                    arrival_time=None,  # ASAP = hold
                )
                self._events.append(HandEvent(
                    target_idx=self._current_idx,
                    phase=HandPhase.HOLDING,
                    time=sim_time,
                    arrival_error_mm=float(np.linalg.norm(
                        current_pose[:3] - catch_target.pose_6dof[:3])),
                    arrival_timing_error_s=sim_time - catch_target.arrival_time if catch_target.arrival_time else 0.0,
                ))
            return self._current_target, hand_cmd

        # ---- Catch-only flow states (backward compat) ----

        # Prime hand on first non-idle step
        if not self._hand_primed and self.is_catch_mode:
            hand_cmd = 'prime'
            self._hand_primed = True

        # Spawn ball at scheduled time
        if ball_spawn is not None and not self._ball_spawned:
            if sim_time >= ball_spawn.spawn_time:
                self._ball_spawned = True

        if self._phase == HandPhase.PRIMING:
            self._phase = HandPhase.APPROACHING

        elif self._phase == HandPhase.APPROACHING:
            if self.is_catch_mode and self._hand_catch_seq is not None:
                if hand_cmd is None:
                    hand_cmd = self._hand_catch_seq
                    self._hand_catch_seq = None

            arrived_by_time = target.arrival_time is not None and sim_time >= target.arrival_time
            arrived_by_pos = target.arrival_time is None and _arrived_by_proximity(current_pose, target.pose_6dof)
            if arrived_by_time or arrived_by_pos:
                timing_err = (sim_time - target.arrival_time) if target.arrival_time is not None else 0.0
                if self.is_catch_mode:
                    self._phase = HandPhase.HOLDING
                    self._hold_end_time = sim_time + target.hold_duration
                    self._hold_expired = False
                    self._current_target = DynamicTarget(
                        pose_6dof=target.pose_6dof.copy(),
                        arrival_time=None,
                    )
                    self._events.append(HandEvent(
                        target_idx=self._current_idx,
                        phase=HandPhase.HOLDING,
                        time=sim_time,
                        arrival_error_mm=float(np.linalg.norm(
                            current_pose[:3] - target.pose_6dof[:3])),
                        arrival_timing_error_s=timing_err,
                    ))
                else:
                    self._phase = HandPhase.THROWING
                    self._events.append(HandEvent(
                        target_idx=self._current_idx,
                        phase=HandPhase.THROWING,
                        time=sim_time,
                        arrival_error_mm=float(np.linalg.norm(
                            current_pose[:3] - target.pose_6dof[:3])),
                        arrival_timing_error_s=timing_err,
                    ))

        elif self._phase == HandPhase.HOLDING:
            if self._hold_expired:
                # Deferred timeout: hold expired last step and notify_capture
                # did not fire in between — treat as a miss.
                hand_cmd = 'prime'
                self._start_return(sim_time, current_pose)
            elif sim_time >= self._hold_end_time:
                # Mark expired but don't transition yet — give the main loop
                # one more chance to call notify_capture() for this step.
                self._hold_expired = True

        elif self._phase == HandPhase.CAUGHT:
            # Ball in hand — retract hand to bottom to secure the ball
            hand_cmd = 'home'
            self._start_return(sim_time, current_pose)

        elif self._phase == HandPhase.DECELERATING:
            if sim_time >= self._phase_start_time + 1.0:
                self._start_return(sim_time, current_pose)

        elif self._phase == HandPhase.RETURNING:
            arrived = _arrived_by_proximity(current_pose, self._active)
            timed_out = sim_time >= self._phase_start_time + _RETURN_SAFETY_TIMEOUT_S
            if arrived or timed_out:
                self._phase = HandPhase.IDLE
                self._current_target = None
                self._hand_primed = False
                if self._current_idx + 1 < len(self._targets):
                    self._advance_to_next(sim_time, current_pose,
                                          hand_pos_mm, plant_state=plant_state)

        return self._current_target, hand_cmd

    def notify_capture(self, sim_time: float) -> None:
        """Called when ball capture is detected."""
        self._caught = True
        if self._phase in (HandPhase.HOLDING, HandPhase.APPROACHING,
                           HandPhase.TRANSITIONING):
            target = self._targets[self._current_idx][0]
            self._events.append(HandEvent(
                target_idx=self._current_idx,
                phase=self._phase,
                time=sim_time,
                arrival_error_mm=0.0,
                arrival_timing_error_s=(sim_time - target.arrival_time
                                        if target.arrival_time else 0.0),
                captured=True,
            ))
            self._phase = HandPhase.CAUGHT

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

    def peek_feasibility(
        self,
        target: DynamicTarget,
        sim_time: float,
        current_pose: np.ndarray,
        hand_pos_mm: float = 0.0,
        plant_state: PlantState | None = None,
    ) -> tuple[bool, str]:
        """Check whether a target is feasible without advancing the state machine."""
        if target.mode == 'throw':
            is_catch = False
        elif target.arrival_twist is not None and not np.allclose(target.arrival_twist, 0):
            is_catch = False
        else:
            is_catch = True
        return self._check_target_feasibility(
            target, sim_time, current_pose, hand_pos_mm, is_catch,
            plant_state=plant_state,
        )

    def _check_target_feasibility(
        self,
        target: DynamicTarget,
        sim_time: float,
        current_pose: np.ndarray,
        hand_pos_mm: float,
        is_catch: bool,
        plant_state: PlantState | None = None,
    ) -> tuple[bool, str]:
        """Shared feasibility logic for both peek and advance paths."""
        if target.arrival_time is None:
            return True, ''

        platform_deadline = target.arrival_time - target.settle_margin_s
        time_to_arrival = max(platform_deadline - sim_time, 0.1)

        if self._feasibility is not None:
            if plant_state is not None:
                feas_state = plant_state
            else:
                feas_state = PlantState(
                    leg_extensions_mm=np.zeros(6),
                    leg_velocities_mmps=np.zeros(6),
                    platform_pos_mm=current_pose[:3].copy(),
                    platform_rot=current_pose[3:].copy(),
                    platform_twist=np.zeros(6),
                    time=sim_time,
                )
            feasible, reason = self._feasibility.check(
                feas_state, target.pose_6dof, time_to_arrival,
                target_twist=target.arrival_twist)
            if not feasible:
                return False, reason

        if is_catch and target.event_vel_mps is not None:
            result = HandCatchSequence.try_create(
                event_vel_mps=target.event_vel_mps,
                arrival_time=target.arrival_time,
                current_pos_mm=hand_pos_mm,
                current_time=sim_time,
            )
            if not result.feasible:
                return False, f'hand timing: {result.reason}'

        return True, ''

    def _advance_to_next(
        self,
        sim_time: float,
        current_pose: np.ndarray,
        hand_pos_mm: float = 0.0,
        plant_state: PlantState | None = None,
    ) -> None:
        """Start tracking the next queued target."""
        self._current_idx += 1
        target, _ = self._targets[self._current_idx]
        self._ball_spawned = False
        self._caught = False
        self._hand_catch_seq = None
        self._phase_start_time = sim_time
        self._throw_released = False
        self._throw_hand_sent = False

        # Determine mode: check mode field, then fall back to arrival_twist
        if target.mode == 'throw':
            is_catch = False
        elif target.arrival_twist is not None and not np.allclose(target.arrival_twist, 0):
            is_catch = False
        else:
            is_catch = True

        # Feasibility check
        feasible, reason = self._check_target_feasibility(
            target, sim_time, current_pose, hand_pos_mm, is_catch,
            plant_state=plant_state,
        )
        if not feasible:
            logger.info(
                "Target %d rejected: %s", self._current_idx, reason)
            self._events.append(HandEvent(
                target_idx=self._current_idx,
                phase=HandPhase.IDLE,
                time=sim_time,
                arrival_error_mm=-1.0,
                captured=False,
            ))
            self._phase = HandPhase.IDLE
            return

        platform_deadline = (target.arrival_time - target.settle_margin_s
                             if target.arrival_time is not None else None)

        if target.mode == 'throw' and self._throw_catch_plan is not None:
            # Throw-catch flow: approach throw pose
            # Mark hand as primed to skip legacy priming logic — the throw/catch
            # hand sequences handle all hand positioning internally.
            self._hand_primed = True
            self._current_target = DynamicTarget(
                pose_6dof=target.pose_6dof.copy(),
                arrival_time=platform_deadline,
                arrival_twist=None,
                hold_duration=0.0,
                event_vel_mps=target.event_vel_mps,
            )
            self._phase = HandPhase.APPROACHING_THROW
        elif is_catch:
            # Catch-only flow
            if target.event_vel_mps is not None and target.arrival_time is not None:
                result = HandCatchSequence.try_create(
                    event_vel_mps=target.event_vel_mps,
                    arrival_time=target.arrival_time,
                    current_pos_mm=hand_pos_mm,
                    current_time=sim_time,
                )
                self._hand_catch_seq = result.sequence

            self._current_target = DynamicTarget(
                pose_6dof=target.pose_6dof.copy(),
                arrival_time=platform_deadline,
                arrival_twist=None,
                hold_duration=target.hold_duration,
                event_vel_mps=target.event_vel_mps,
            )
            self._phase = HandPhase.APPROACHING

            if not self._hand_primed:
                self._phase = HandPhase.PRIMING
        else:
            # Legacy throw (arrive with velocity)
            self._current_target = DynamicTarget(
                pose_6dof=target.pose_6dof.copy(),
                arrival_time=platform_deadline,
                arrival_twist=(target.arrival_twist.copy()
                               if target.arrival_twist is not None else None),
                hold_duration=target.hold_duration,
            )
            self._phase = HandPhase.APPROACHING

    def _start_return(self, now: float, current_pose: np.ndarray) -> None:
        """Command a gentle return to active pose via the MPC."""
        dist_mm = np.linalg.norm(current_pose[:3] - self._active[:3])
        rot_rad = np.linalg.norm(current_pose[3:] - self._active[3:])
        effective_dist = max(dist_mm, rot_rad * _RETURN_ROT_SCALE_MM)
        duration = float(np.clip(
            effective_dist / _RETURN_VEL_MMPS + _RETURN_MARGIN_S,
            _RETURN_MIN_S, _RETURN_MAX_S,
        ))

        self._phase_start_time = now
        self._current_target = DynamicTarget(
            pose_6dof=self._active.copy(),
            arrival_time=now + duration,
        )
        self._phase = HandPhase.RETURNING
        self._throw_catch_plan = None  # Clear completed plan
