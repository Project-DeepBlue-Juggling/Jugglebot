"""Toss-loop controller: continuous throw-catch with timing model.

Implements Phases A, B, and C of the continuous-motion juggling plan.

Phase A: vertical toss-to-self (stationary platform, lateral_spacing=0)
Phase B: toss between positions (platform moves during air time)
Phase C: velocity-at-events (platform moving when it throws and catches)

The timing model uses two parameters:
    cycle_time  = air_time + hold_time
    hold_ratio  = hold_time / cycle_time

Everything else — throw speed, hand trajectory durations, dwell time,
platform targets — is derived from these two numbers.

Platform motion uses quintic Hermite interpolation between event poses,
producing a C2-continuous motion stream with no target discontinuities.
The MPC tracks a smoothly moving ASAP reference rather than rushing
toward static endpoint targets.

Usage:
    python sim/main.py --cycle-time 1.2
    python sim/main.py --cycle-time 1.2 --hold-ratio 0.35
    python sim/main.py --cycle-time 1.2 --lateral-spacing 120
    python sim/main.py --cycle-time 1.2 --lateral-spacing 120 --platform-event-speed-ratio 0.8
"""

from __future__ import annotations

import logging
from enum import Enum, auto

import numpy as np

import os, sys
_sim_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _sim_dir not in sys.path:
    sys.path.insert(0, _sim_dir)

from hand.ballistics import compute_hand_offset_mm
from hand.coordinator import DynamicTarget, BallRelease
from hand.planner import ThrowCatchPlanner, ThrowCatchPlan, _HAND_CATCH_X5_MM
from hand.trajectory import (
    HandThrowTrajectory,
    HandCatchTrajectory,
    GRAVITY_MPS2,
)
from input.sim_control import SimController
from plant.interface import PlantState
from controller.target import ReferenceEvent

logger = logging.getLogger(__name__)

_PLATFORM_HEIGHT_MM = 574.3
_INITIAL_APPROACH_S = 0.5   # seconds before first throw for hand prep
_DROP_RESPAWN_DELAY = 0.5   # seconds after drop before respawning
_DROP_TIMEOUT_MARGIN = 0.3  # extra seconds past catch-seq end before declaring drop
_KEY_B = 66                 # GLFW keycode for 'B' (reset ball)
_KEY_PAGE_UP = 266          # GLFW keycode for Page Up (raise ball Z)
_KEY_PAGE_DOWN = 267        # GLFW keycode for Page Down (lower ball Z)
_BALL_Z_STEP_MM = 20.0      # mm per keypress for ball Z adjustment


class _State(Enum):
    STARTUP = auto()
    ACTIVE = auto()
    DROPPED = auto()


class _Phase(Enum):
    APPROACHING_THROW = auto()  # Platform moving to throw pose, hand prelude
    THROWING = auto()           # Ball released, throw trajectory finishing
    IN_FLIGHT = auto()          # Platform transiting to catch, catch traj active
    POST_CATCH = auto()         # Ball captured, catch finishing, planning next



class TossLoopController:
    """Continuous toss loop driven by cycle_time and hold_ratio.

    Platform motion uses quintic Hermite interpolation between event poses,
    producing C2-continuous motion (position, velocity, and acceleration
    are all continuous at segment boundaries).  The MPC tracks a moving
    ASAP reference — no arrival_time deadlines.

    Parameters
    ----------
    cycle_time : float
        Total cycle duration in seconds (throw-to-throw).
    hold_ratio : float
        Fraction of cycle_time the ball is in the hand (0..1).
    lateral_spacing_mm : float
        Phase B/C lateral spacing between toss positions.
        0 = Phase A vertical toss-to-self.
    platform_event_speed_ratio : float
        Platform velocity at throw/catch as a fraction of average transit
        velocity (0..1). 0 = stop at events (Phase B), >0 = continuous
        motion (Phase C).
    active_pose : (6,) array or None
        Platform active pose (default zeros).
    """

    def __init__(
        self,
        cycle_time: float = 1.2,
        hold_ratio: float = 0.4,
        lateral_spacing_mm: float = 0.0,
        platform_event_speed_ratio: float = 0.0,
        active_pose: np.ndarray | None = None,
    ):
        self._cycle_time = cycle_time
        self._hold_ratio = hold_ratio
        self._lateral_spacing = lateral_spacing_mm
        self._event_speed_ratio = platform_event_speed_ratio
        self._continuous = platform_event_speed_ratio > 0
        self._active_pose = active_pose if active_pose is not None else np.zeros(6)

        # Derived timing
        self._air_time = (1.0 - hold_ratio) * cycle_time
        self._hold_time = hold_ratio * cycle_time
        self._v_launch_mps = GRAVITY_MPS2 * self._air_time / 2.0

        # Validate throw speed
        if self._v_launch_mps < 0.3:
            raise ValueError(
                f"Launch speed {self._v_launch_mps:.2f} m/s below minimum "
                f"0.3 m/s — increase cycle_time or decrease hold_ratio")
        if self._v_launch_mps > 7.0:
            raise ValueError(
                f"Launch speed {self._v_launch_mps:.2f} m/s exceeds maximum "
                f"7.0 m/s — decrease cycle_time or increase hold_ratio")

        # Feasibility: dwell must be non-negative
        self._dwell_s = self._compute_dwell(self._v_launch_mps, self._hold_time)
        if self._dwell_s < 0:
            min_ct = self._find_min_cycle_time()
            raise ValueError(
                f"Infeasible: cycle_time={cycle_time}s, hold_ratio={hold_ratio} "
                f"→ dwell={self._dwell_s * 1000:.0f}ms. "
                f"Min cycle_time ≈ {min_ct:.2f}s at hold_ratio={hold_ratio}.")

        # Ball Z: average of throw release and catch arrival hand offsets
        self._ball_z = self._compute_ball_z() + 120.0

        # Components
        self._planner = ThrowCatchPlanner()
        self._sim_ctrl = SimController()

        # State machine
        self._state = _State.STARTUP
        self._phase = _Phase.APPROACHING_THROW
        self._startup_time = 0.0
        self._drop_time = 0.0
        self._ball_spawned = False

        # Current cycle
        self._plan: ThrowCatchPlan | None = None
        self._next_plan: ThrowCatchPlan | None = None
        self._throw_hand_sent = False
        self._ball_released = False
        self._catch_hand_sent = False
        self._ball_captured = False
        self._last_throw_time = 0.0
        self._approach_start_time = 0.0
        self._position_idx = 0

        # Hold-to-throw transition state: saved when _start_cycle replaces
        # Stats
        self._cycle_count = 0
        self._catch_count = 0
        self._drop_count = 0
        self._streak = 0
        self._best_streak = 0
        self._timing_errors: list[float] = []

        # Print timing summary
        height_mm = (self._v_launch_mps ** 2) / (2 * GRAVITY_MPS2) * 1000
        print(f"\n  Toss Loop Configuration")
        print(f"  ---------------------------------------------")
        print(f"  cycle_time = {cycle_time}s  hold_ratio = {hold_ratio}")
        print(f"  air = {self._air_time * 1000:.0f}ms  "
              f"hold = {self._hold_time * 1000:.0f}ms  "
              f"dwell = {self._dwell_s * 1000:.0f}ms")
        print(f"  v_launch = {self._v_launch_mps:.2f} m/s  "
              f"height = {height_mm:.0f}mm")
        if lateral_spacing_mm > 0 and self._continuous:
            avg_vel = lateral_spacing_mm / self._air_time
            event_vel = platform_event_speed_ratio * avg_vel
            print(f"  lateral spacing = {lateral_spacing_mm:.0f}mm  "
                  f"speed ratio = {platform_event_speed_ratio} (Phase C)")
            print(f"  event platform vel = {event_vel:.1f} mm/s  "
                  f"(avg transit = {avg_vel:.1f} mm/s)")
        elif lateral_spacing_mm > 0:
            print(f"  lateral spacing = {lateral_spacing_mm:.0f}mm (Phase B)")
        else:
            print(f"  vertical toss (Phase A)")
        print(f"  ---------------------------------------------\n")

    # ------------------------------------------------------------------
    # Timing helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _compute_dwell(v_mps: float, hold_time: float) -> float:
        """Compute dwell time: hold_time minus hand trajectory durations."""
        throw_traj = HandThrowTrajectory(v_mps)
        catch_traj = HandCatchTrajectory(v_mps)
        throw_pre_release = -throw_traj.start_time   # positive
        catch_post_arrival = catch_traj.end_time      # positive
        return hold_time - catch_post_arrival - throw_pre_release

    def _find_min_cycle_time(self) -> float:
        """Binary search for minimum feasible cycle_time."""
        lo, hi = 0.5, 5.0
        for _ in range(40):
            mid = (lo + hi) / 2.0
            air = (1.0 - self._hold_ratio) * mid
            v = GRAVITY_MPS2 * air / 2.0
            if v < 0.3 or v > 7.0:
                lo = mid
                continue
            hold = self._hold_ratio * mid
            if self._compute_dwell(v, hold) >= 0:
                hi = mid
            else:
                lo = mid
        return hi

    def _compute_ball_z(self) -> float:
        """Ball Z that keeps the platform near active pose for vertical toss."""
        throw_traj = HandThrowTrajectory(self._v_launch_mps)
        throw_offset = compute_hand_offset_mm(throw_traj.release_pos_mm)
        catch_offset = compute_hand_offset_mm(_HAND_CATCH_X5_MM)
        return _PLATFORM_HEIGHT_MM + (throw_offset + catch_offset) / 2.0

    # ------------------------------------------------------------------
    # Position selection (Phase B)
    # ------------------------------------------------------------------

    def _get_positions(self) -> tuple[np.ndarray, np.ndarray]:
        """Return (throw_pos, catch_pos) for the next cycle.

        Phase A (spacing=0): both at centre.
        Phase B (spacing>0): alternate between two positions.
        """
        if self._lateral_spacing == 0:
            pos = np.array([0.0, 0.0, self._ball_z])
            return pos.copy(), pos.copy()
        half = self._lateral_spacing / 2.0
        pos_a = np.array([-half, 0.0, self._ball_z])
        pos_b = np.array([half, 0.0, self._ball_z])
        if self._position_idx % 2 == 0:
            return pos_a.copy(), pos_b.copy()
        return pos_b.copy(), pos_a.copy()

    # ------------------------------------------------------------------
    # Platform velocity (Phase C)
    # ------------------------------------------------------------------

    def _compute_event_twists(
        self,
        throw_pos: np.ndarray,
        catch_pos: np.ndarray,
    ) -> tuple[np.ndarray | None, np.ndarray | None]:
        """Compute platform twist at throw and catch events.

        Returns (throw_twist, catch_twist), each (6,) or None.
        Phase C: the platform has velocity at throw (reducing hand throw
        speed), but always stops at catch.  The catch twist is zero to
        prevent the hold-segment velocity reversal that causes overshoot
        when catch and next_throw are co-located.
        """
        if not self._continuous or self._lateral_spacing == 0:
            return None, None

        # Average transit velocity vector (translational, mm/s)
        displacement = catch_pos - throw_pos
        avg_vel = displacement / self._air_time
        event_vel = self._event_speed_ratio * avg_vel

        # Build 6-DOF twist: [vx, vy, vz, wx, wy, wz]
        # Throw: platform velocity reduces required hand throw speed.
        throw_twist = np.array([
            event_vel[0], event_vel[1], event_vel[2],
            0.0, 0.0, 0.0,
        ])
        # Catch: platform stops.  The hand absorbs the full ball arrival
        # velocity.  This eliminates the hold-segment velocity reversal.
        catch_twist = None
        return throw_twist, catch_twist

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def _make_plan(self, throw_time: float, hand_pos_mm: float,
                   current_time: float) -> ThrowCatchPlan | None:
        """Plan a single throw-catch cycle via ThrowCatchPlanner."""
        throw_pos, catch_pos = self._get_positions()
        catch_time = throw_time + self._air_time
        throw_twist, catch_twist = self._compute_event_twists(
            throw_pos, catch_pos)
        try:
            return self._planner.plan(
                throw_pos_mm=throw_pos,
                catch_pos_mm=catch_pos,
                throw_time=throw_time,
                catch_time=catch_time,
                current_hand_pos_mm=hand_pos_mm,
                current_time=current_time,
                throw_platform_twist=throw_twist,
                catch_platform_twist=catch_twist,
            )
        except ValueError as e:
            logger.warning("Plan failed: %s", e)
            print(f"  Plan failed: {e}")
            return None

    # ------------------------------------------------------------------
    # State transitions
    # ------------------------------------------------------------------

    def _start_cycle(self, plan: ThrowCatchPlan, throw_time: float):
        """Initialise tracking state for a new cycle."""
        self._plan = plan
        self._next_plan = None
        self._last_throw_time = throw_time
        self._phase = _Phase.APPROACHING_THROW
        self._throw_hand_sent = False
        self._ball_released = False
        self._catch_hand_sent = False
        self._ball_captured = False
        self._cycle_count += 1
        self._position_idx += 1
        tz = plan.throw_target.pose_6dof[2]
        cz = plan.catch_target.pose_6dof[2]
        tt = self._twist_or_zero(plan.throw_target)
        print(f"\n  --- Cycle {self._cycle_count} "
              f"(caught: {self._catch_count}, streak: {self._streak}) ---")
        print(f"  throw_z={tz:.1f}mm  catch_z={cz:.1f}mm  "
              f"(dZ={cz - tz:+.1f}mm during flight)")
        if np.any(tt != 0):
            print(f"  throw_twist=[{tt[0]:.0f}, {tt[1]:.0f}, {tt[2]:.0f}] mm/s"
                  f"  catch_twist=[0, 0, 0] mm/s")

    def _handle_drop(self, sim_time: float):
        """Transition to DROPPED state."""
        self._drop_count += 1
        self._streak = 0
        self._state = _State.DROPPED
        self._drop_time = sim_time
        print(f"  DROPPED ({self._drop_count} total, "
              f"{self._catch_count}/{self._cycle_count} caught)")

    # ------------------------------------------------------------------
    # Keyboard / sim control
    # ------------------------------------------------------------------

    def key_callback(self, keycode: int) -> None:
        """Handle keyboard events from the MuJoCo viewer."""
        self._sim_ctrl.key_callback(keycode)
        if keycode == _KEY_B:
            if self._state in (_State.DROPPED, _State.ACTIVE):
                self._state = _State.STARTUP
                self._ball_spawned = False
                print("  Ball reset")
        elif keycode == _KEY_PAGE_UP:
            self._ball_z += _BALL_Z_STEP_MM
            print(f"  Ball Z = {self._ball_z:.0f} mm (raised)")
        elif keycode == _KEY_PAGE_DOWN:
            self._ball_z -= _BALL_Z_STEP_MM
            print(f"  Ball Z = {self._ball_z:.0f} mm (lowered)")

    def should_step(self) -> bool:
        return self._sim_ctrl.should_step()

    @property
    def sleep_factor(self) -> float:
        return self._sim_ctrl.sleep_factor

    # ------------------------------------------------------------------
    # Capture notification
    # ------------------------------------------------------------------

    def notify_capture(self, sim_time: float) -> None:
        """Called by the main loop when ball capture is detected."""
        if self._state == _State.ACTIVE and self._phase == _Phase.IN_FLIGHT:
            self._ball_captured = True

    # ------------------------------------------------------------------
    # Quintic-interpolated platform target
    # ------------------------------------------------------------------

    @staticmethod
    def _twist_or_zero(target: DynamicTarget) -> np.ndarray:
        if target.arrival_twist is not None:
            return target.arrival_twist.copy()
        return np.zeros(6)

    # ------------------------------------------------------------------
    # Multi-event MPC lookahead (single source of truth for platform motion)
    # ------------------------------------------------------------------

    def _build_ref_events(self, sim_time: float) -> list[ReferenceEvent] | None:
        """Construct ReferenceEvent list covering the current MPC horizon.

        All cycle events (throw/catch for current + next cycle) are included
        unconditionally — even past events.  Past events serve as left
        interpolation boundaries so the MPC can quintic-interpolate the
        current segment correctly.  Without the left boundary, the MPC would
        backward-extrapolate from the next event's velocity, pulling the
        platform the wrong way.

        The MPC's ``_build_reference_from_events()`` uses quintic Hermite
        interpolation (C2-continuous) between events that provide
        acceleration, producing smooth, overshoot-free references.

        Returns None when no plan is active (STARTUP, DROPPED).
        """
        if self._plan is None:
            return None

        plan = self._plan
        throw_time = self._last_throw_time
        catch_time = throw_time + self._air_time
        next_throw_time = throw_time + self._cycle_time

        events: list[ReferenceEvent] = []

        # During initial approach, anchor the reference at the start pose
        # so the MPC can interpolate smoothly from rest to the throw event.
        if (self._phase == _Phase.APPROACHING_THROW
                and sim_time < throw_time
                and self._approach_start_time <= sim_time):
            events.append(ReferenceEvent(
                time=self._approach_start_time,
                pose=self._active_pose.copy(),
                twist=np.zeros(6),
                accel=np.zeros(6),
            ))

        # Current cycle's throw event (always included as boundary)
        events.append(ReferenceEvent(
            time=throw_time,
            pose=plan.throw_target.pose_6dof.copy(),
            twist=self._twist_or_zero(plan.throw_target),
            accel=np.zeros(6),
        ))

        # Current cycle's catch event (always included as boundary)
        events.append(ReferenceEvent(
            time=catch_time,
            pose=plan.catch_target.pose_6dof.copy(),
            twist=self._twist_or_zero(plan.catch_target),
            accel=np.zeros(6),
        ))

        # Next cycle's events (if pre-planned)
        if self._next_plan is not None:
            events.append(ReferenceEvent(
                time=next_throw_time,
                pose=self._next_plan.throw_target.pose_6dof.copy(),
                twist=self._twist_or_zero(self._next_plan.throw_target),
                accel=np.zeros(6),
            ))

            next_catch_time = next_throw_time + self._air_time
            events.append(ReferenceEvent(
                time=next_catch_time,
                pose=self._next_plan.catch_target.pose_6dof.copy(),
                twist=self._twist_or_zero(self._next_plan.catch_target),
                accel=np.zeros(6),
            ))

        return events

    # ------------------------------------------------------------------
    # Main update
    # ------------------------------------------------------------------

    def update(
        self,
        sim_time: float,
        current_pose: np.ndarray,
        hand_pos_mm: float,
        plant_state: PlantState | None = None,
    ) -> tuple[DynamicTarget | None, object, object, list[ReferenceEvent] | None]:
        """Advance the state machine.

        Returns
        -------
        target : DynamicTarget or None
            MPC target this step (ASAP, no arrival_time).
        hand_cmd : HandThrowSequence, HandCatchSequence, BallRelease, or None
        ball_spawn : 'spawn_in_hand' or None
        ref_events : list[ReferenceEvent] or None
            Multi-event reference for the MPC.  The MPC quintic-interpolates
            between these events (C2-continuous) to produce the platform
            trajectory.  This is the single source of truth for platform
            motion — there is no separate ASAP target path.
        """
        rest = DynamicTarget(pose_6dof=self._active_pose.copy())

        # -- STARTUP: spawn ball, plan first throw --
        if self._state == _State.STARTUP:
            if not self._ball_spawned:
                self._ball_spawned = True
                self._startup_time = sim_time
                return rest, None, 'spawn_in_hand', None
            if sim_time - self._startup_time < 0.3:
                return rest, None, None, None
            # Plan first throw
            throw_time = sim_time + _INITIAL_APPROACH_S
            plan = self._make_plan(throw_time, hand_pos_mm, sim_time)
            if plan is None:
                return rest, None, None, None
            self._approach_start_time = sim_time
            self._start_cycle(plan, throw_time)
            self._state = _State.ACTIVE

            # Pre-plan next cycle (positions are deterministic)
            next_throw = throw_time + self._cycle_time
            self._next_plan = self._make_plan(
                next_throw, plan.catch_hand_seq.catch_trajectory.end_pos_mm,
                sim_time)

            # Fall through to ACTIVE

        # -- DROPPED: auto-respawn after brief pause --
        if self._state == _State.DROPPED:
            if sim_time - self._drop_time > _DROP_RESPAWN_DELAY:
                self._state = _State.STARTUP
                self._ball_spawned = False
            return rest, None, None, None

        # -- ACTIVE --
        if self._state == _State.ACTIVE:
            return self._update_active(sim_time, current_pose, hand_pos_mm)

        return rest, None, None, None

    def _update_active(self, sim_time, current_pose, hand_pos_mm):
        """Run the cycle phase state machine.

        Platform trajectory is defined entirely by ref_events — the MPC
        quintic-interpolates between them to produce a C2-continuous path.
        The phase machine only controls hand commands and ball lifecycle.
        """
        plan = self._plan
        hand_cmd = None
        throw_time = self._last_throw_time
        catch_time = throw_time + self._air_time
        next_throw_time = throw_time + self._cycle_time

        # -- Event-based reference (single source of truth) --
        ref_events = self._build_ref_events(sim_time)

        # Target pose for diagnostics / fallback: use the first upcoming
        # event's pose, or current pose if no events.
        if ref_events:
            target = DynamicTarget(pose_6dof=ref_events[0].pose.copy())
        else:
            target = DynamicTarget(pose_6dof=current_pose.copy())

        # -- APPROACHING_THROW: send hand sequence, wait for release --
        if self._phase == _Phase.APPROACHING_THROW:
            if not self._throw_hand_sent:
                hand_cmd = plan.throw_hand_seq
                self._throw_hand_sent = True
            elif not self._ball_released and sim_time >= plan.ball_release_time:
                hand_cmd = BallRelease(
                    velocity_mms=plan.ball_release_vel_mms.copy())
                self._ball_released = True
                self._phase = _Phase.THROWING
                error_s = sim_time - plan.ball_release_time
                self._timing_errors.append(error_s)
                logger.debug("Released at %.3f (target %.3f, err %+.1fms)",
                             sim_time, plan.ball_release_time, error_s * 1000)
            return target, hand_cmd, None, ref_events

        # -- THROWING: throw decel finishing, then start catch hand seq --
        if self._phase == _Phase.THROWING:
            if sim_time >= plan.throw_hand_seq.end_time:
                hand_cmd = plan.catch_hand_seq
                self._catch_hand_sent = True
                self._phase = _Phase.IN_FLIGHT
            return target, hand_cmd, None, ref_events

        # -- IN_FLIGHT: wait for capture or timeout --
        if self._phase == _Phase.IN_FLIGHT:
            if self._ball_captured:
                self._catch_count += 1
                self._streak += 1
                self._best_streak = max(self._best_streak, self._streak)
                print(f"  CAUGHT ({self._catch_count}/{self._cycle_count}, "
                      f"streak: {self._streak})")
                self._phase = _Phase.POST_CATCH

            elif sim_time > plan.catch_hand_seq.end_time + _DROP_TIMEOUT_MARGIN:
                self._handle_drop(sim_time)
                return DynamicTarget(
                    pose_6dof=self._active_pose.copy()), None, None, None

            return target, hand_cmd, None, ref_events

        # -- POST_CATCH: wait for hold to end, start next cycle --
        if self._phase == _Phase.POST_CATCH:
            next_plan = self._next_plan

            if next_plan is None:
                # Planning failed — drop
                self._handle_drop(sim_time)
                return DynamicTarget(
                    pose_6dof=self._active_pose.copy()), None, None, None

            # Start next cycle when it's time for the throw hand prelude
            prelude_due = (sim_time >=
                           next_plan.throw_hand_seq.prelude_start_time - 0.02)

            if prelude_due:
                next_throw = self._last_throw_time + self._cycle_time
                self._start_cycle(next_plan, next_throw)
                hand_cmd = next_plan.throw_hand_seq
                self._throw_hand_sent = True

                # Pre-plan the cycle after this one
                future_throw = next_throw + self._cycle_time
                self._next_plan = self._make_plan(
                    future_throw,
                    next_plan.catch_hand_seq.catch_trajectory.end_pos_mm,
                    sim_time)

                return target, hand_cmd, None, ref_events

            return target, hand_cmd, None, ref_events

        # Fallback
        return DynamicTarget(pose_6dof=self._active_pose.copy()), None, None, None

    # ------------------------------------------------------------------
    # Preview rendering
    # ------------------------------------------------------------------

    def render_preview(self, viewer) -> None:
        """Draw throw/catch position markers (Phase B only)."""
        if self._lateral_spacing == 0:
            return
        import mujoco
        scn = viewer.user_scn
        half = self._lateral_spacing / 2.0
        for x, rgba, label in [
            (-half, [0.0, 0.8, 0.0, 0.5], "A"),
            (half, [0.0, 0.4, 1.0, 0.5], "B"),
        ]:
            if scn.ngeom >= scn.maxgeom:
                return
            geom = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(
                geom, mujoco.mjtGeom.mjGEOM_SPHERE,
                np.array([0.015, 0.0, 0.0]),
                np.array([x / 1000.0, 0.0, self._ball_z / 1000.0]),
                np.eye(3).flatten(),
                np.array(rgba, dtype=np.float32),
            )
            geom.label = label
            scn.ngeom += 1

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------

    def print_summary(self) -> None:
        """Print session stats."""
        total = self._catch_count + self._drop_count
        pct = self._catch_count / total * 100 if total > 0 else 0
        print(f"\n  Toss Loop Summary")
        print(f"  -------------------------------------")
        print(f"  Caught: {self._catch_count}/{total} ({pct:.0f}%)")
        print(f"  Drops:  {self._drop_count}")
        print(f"  Best streak: {self._best_streak}")
        if self._timing_errors:
            errs = np.array(self._timing_errors) * 1000
            print(f"  Release timing: mean={np.mean(errs):+.1f}ms, "
                  f"max={np.max(np.abs(errs)):.1f}ms")
