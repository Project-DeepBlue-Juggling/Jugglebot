"""TossMotionSource — platform-only back-and-forth ballistic test source.

Drives the MPC through a sequence of throw→catch platform motions without a
hand or a ball. The source:

  1. Reads the current platform pose at the start of each cycle (so the
     "throw pose" follows the platform's current position as cycles chain).
  2. Computes the 6-DOF throw and catch poses from the current pose, the
     user-supplied catch position, and a desired apex height, using the
     pure ballistics in :mod:`controller.ballistics`.
  3. Emits targets through the K1–K6 reference-feasibility contract so
     IPOPT only ever sees velocity- and acceleration-bounded references.

State machine per cycle:
  APPROACH  — platform rotates in place and/or moves to the throw pose,
              ASAP convergence (K1–K6 enforced).
  TRANSIT   — platform moves throw_pose → catch_pose over ``flight_time``
              seconds (same duration as the notional ball's flight) via the
              K1–K6 min-jerk quintic.  The platform traces a smooth 5th-order
              interpolant, not the ball's parabolic arc — transit is timing-
              matched but not path-matched.  (No intermediate ref_events are
              emitted, so ``flat_target_to_events`` produces a two-event
              quintic with zero endpoint velocities.)
  HOLD      — platform holds at the catch pose for ``hold_s`` seconds
              before advancing to the next cycle.

After the final cycle the source holds at the last catch pose indefinitely
(no automatic return-to-active, per design).
"""

from __future__ import annotations

import logging
from typing import Any, List, Optional, Sequence

import numpy as np

from .ballistics import (
    compute_arrival_velocity,
    compute_launch_velocity,
    compute_orientation,
    flight_time_from_apex,
)
from .target import (
    ReferenceEvent,
    TargetCommand,
    flat_target_to_events,
    is_warm_start_invalidating,
    pose_6dof_from_state,
)

logger = logging.getLogger(__name__)


_DEFAULT_APEX_HEIGHT_MM = 1200.0
_DEFAULT_HOLD_S = 1.0


class TossMotionSource:
    """Platform-only toss-motion sequence source.

    Parameters
    ----------
    catch_positions_mm : sequence of (3,) arrays
        One catch position per cycle. Each cycle's throw position is the
        platform's current 3D position at the *start of that cycle* — the
        platform naturally chains from one catch to the next throw.
    apex_height_mm : float
        Apex of the (notional) ball above each throw position. Determines
        the throw's vertical velocity and hence the flight time.
    hold_s : float
        Dwell time at each catch pose before advancing to the next cycle.
    v_max_mmps, tau_s, clamp_start_twist_mmps
        K1–K6 reference-feasibility parameters — threaded into
        ``flat_target_to_events`` on every emitted reference so the MPC
        only sees velocity-/acceleration-bounded quintics. Pass
        ``mpc.params.max_leg_vel_mmps``, ``mpc.params.tau`` and
        ``mpc.params.max_ref_start_twist_mmps``.

    Notes
    -----
    * No hand, no ball: we ignore the hand centroid offset. The platform
      centroid is placed exactly at the throw / catch positions.
    * No return-to-active: when the sequence finishes, the source holds at
      the final catch pose. Callers wanting an explicit return should add a
      final catch_position equal to the active pose.
    * The transit segment is emitted with ``arrival_time = catch_time`` so
      the MPC honours the ballistic deadline. The approach segment uses
      ``arrival_time = None`` (ASAP) so the K1–K6 layer picks the duration.
    * Transit is timing-matched but not path-matched: the platform slides
      along the quintic between throw and catch in ``flight_time`` seconds,
      while the (notional) ball traces a parabola over the same interval.
      Fine for platform-only tests; a catcher application would need to emit
      intermediate ref_events sampled along the parabola.
    """

    _PHASE_APPROACH = 'approach'
    _PHASE_TRANSIT = 'transit'
    _PHASE_HOLD = 'hold'
    _PHASE_DONE = 'done'

    def __init__(self,
                 catch_positions_mm: Sequence,
                 apex_height_mm: float = _DEFAULT_APEX_HEIGHT_MM,
                 hold_s: float = _DEFAULT_HOLD_S,
                 v_max_mmps: float | None = None,
                 tau_s: float | None = None,
                 clamp_start_twist_mmps: float | None = None):
        if len(catch_positions_mm) == 0:
            raise ValueError("catch_positions_mm is empty — need at least 1")

        # None entries are sentinels for "initial platform pose at source
        # start" — useful for ping-pong: --cycles 3 with --catch-pose X,Y,Z
        # produces [X, None, X] meaning [catch, return-to-initial, catch].
        # The initial pose is captured on the first update() call.
        self._catches: List[np.ndarray | None] = [
            (None if p is None else np.asarray(p, dtype=float).reshape(3))
            for p in catch_positions_mm]
        self._apex_mm = float(apex_height_mm)
        self._hold_s = float(hold_s)
        self._v_max_mmps = v_max_mmps
        self._tau_s = tau_s
        self._clamp_start_twist_mmps = clamp_start_twist_mmps

        # Phase state — initialised lazily on the first update() call so the
        # cycle starts from the platform's actual pose at enable, not from
        # whatever stale default was constructed.
        self._phase: str = self._PHASE_APPROACH
        self._phase_start_time: float | None = None
        self._cycle_idx: int = 0
        self._started: bool = False
        self._initial_pos_mm: np.ndarray | None = None

        # Current cycle's computed poses & timing
        self._throw_pose_6dof: np.ndarray | None = None
        self._catch_pose_6dof: np.ndarray | None = None
        self._flight_time_s: float = 0.0

        # Cache for ref_events — regenerated when target or phase changes
        # so IPOPT sees a stable landscape between solves on the same
        # segment but refreshes when arrival-time semantics change
        # (TRANSIT's deadline → HOLD's ASAP hold at the same catch pose).
        self._cached_events: List[ReferenceEvent] | None = None
        self._cached_target: np.ndarray | None = None
        self._cached_phase: str | None = None
        self._prev_ref_end_pose: np.ndarray | None = None
        self._prev_ref_end_twist: np.ndarray | None = None

    # ------------------------------------------------------------------
    # Public read-only state
    # ------------------------------------------------------------------

    @property
    def done(self) -> bool:
        return self._phase == self._PHASE_DONE

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def cycle_idx(self) -> int:
        return self._cycle_idx

    # ------------------------------------------------------------------
    # TargetSource protocol
    # ------------------------------------------------------------------

    def update(self, sim_time: float, state: Any) -> TargetCommand:
        if not self._started:
            # Capture the initial pose once, on the very first call — used
            # to resolve any None sentinels in the catch list.
            self._initial_pos_mm = np.asarray(
                state.platform_pos_mm, dtype=float).copy()
            self._begin_cycle(sim_time, state)
            self._started = True

        self._maybe_advance_phase(sim_time, state)

        if self._phase == self._PHASE_APPROACH:
            target_pose = self._throw_pose_6dof
            arrival_time: Optional[float] = None
            target_twist: Optional[np.ndarray] = None
        elif self._phase == self._PHASE_TRANSIT:
            target_pose = self._catch_pose_6dof
            # phase_start_time was set at throw moment; catch is flight_time
            # later.  The arrival_time is an absolute deadline, matching the
            # ball's flight.
            arrival_time = (self._phase_start_time + self._flight_time_s)
            target_twist = None  # platform comes to rest at catch
        elif self._phase == self._PHASE_HOLD or self._phase == self._PHASE_DONE:
            target_pose = self._catch_pose_6dof
            arrival_time = None
            target_twist = None
        else:  # pragma: no cover — defensive
            raise RuntimeError(f"Unknown phase: {self._phase}")

        # Regenerate ref_events only when the target changes.  Matches the
        # pattern used by StaticTargetSource / AutoSequenceTargetSource,
        # with phase included in the key so TRANSIT→HOLD at the same
        # catch pose still invalidates (deadline → ASAP semantics change).
        target_changed = (
            self._cached_target is None
            or not np.array_equal(target_pose, self._cached_target)
            or self._cached_phase != self._phase)
        warm_start_valid = True
        if target_changed:
            self._cached_events = flat_target_to_events(
                pose_6dof_from_state(state),
                state.platform_twist,
                target_pose,
                sim_time,
                target_twist=target_twist,
                arrival_time=arrival_time,
                v_max_mmps=self._v_max_mmps,
                tau_s=self._tau_s,
                clamp_start_twist_mmps=self._clamp_start_twist_mmps,
            )
            self._cached_target = target_pose.copy()
            self._cached_phase = self._phase
            new_end_pose = self._cached_events[-1].pose
            new_end_twist = self._cached_events[-1].twist
            if is_warm_start_invalidating(
                self._prev_ref_end_pose, self._prev_ref_end_twist,
                new_end_pose, new_end_twist,
            ):
                warm_start_valid = False
            self._prev_ref_end_pose = new_end_pose.copy()
            self._prev_ref_end_twist = new_end_twist.copy()

        return TargetCommand(
            target_pose=target_pose,
            arrival_time=arrival_time,
            target_twist=target_twist,
            ref_events=self._cached_events,
            boost_vel_weights=True,
            warm_start_valid=warm_start_valid,
        )

    # ------------------------------------------------------------------
    # Phase transitions
    # ------------------------------------------------------------------

    def _maybe_advance_phase(self, sim_time: float, state: Any) -> None:
        if self._phase == self._PHASE_APPROACH:
            # Advance to TRANSIT when the platform has reached the throw
            # pose.  We use pose + twist proximity rather than a hard timer:
            # the K1–K6 approach duration is chosen by the feasibility layer
            # and we should only "fire the throw" once we're actually there.
            pose = pose_6dof_from_state(state)
            pos_err = float(np.linalg.norm(pose[:3] - self._throw_pose_6dof[:3]))
            rot_err = float(np.linalg.norm(pose[3:] - self._throw_pose_6dof[3:]))
            lin_vel = float(np.linalg.norm(state.platform_twist[:3]))
            ang_vel = float(np.linalg.norm(state.platform_twist[3:]))
            if (pos_err < 2.0 and rot_err < 0.02
                    and lin_vel < 10.0 and ang_vel < 0.05):
                self._phase = self._PHASE_TRANSIT
                self._phase_start_time = sim_time
                logger.info(
                    "TossMotionSource: cycle %d APPROACH→TRANSIT at t=%.3fs "
                    "(pos_err=%.2fmm rot_err=%.3frad lin_vel=%.2fmm/s "
                    "ang_vel=%.3frad/s flight_time=%.3fs)",
                    self._cycle_idx, sim_time,
                    pos_err, rot_err, lin_vel, ang_vel,
                    self._flight_time_s)
        elif self._phase == self._PHASE_TRANSIT:
            transit_end = self._phase_start_time + self._flight_time_s
            if sim_time >= transit_end:
                self._phase = self._PHASE_HOLD
                self._phase_start_time = sim_time
                logger.info(
                    "TossMotionSource: cycle %d TRANSIT→HOLD at t=%.3fs",
                    self._cycle_idx, sim_time)
        elif self._phase == self._PHASE_HOLD:
            hold_end = self._phase_start_time + self._hold_s
            if sim_time >= hold_end:
                if self._cycle_idx + 1 < len(self._catches):
                    self._cycle_idx += 1
                    self._begin_cycle(sim_time, state)
                    logger.info(
                        "TossMotionSource: advancing to cycle %d at t=%.3fs",
                        self._cycle_idx, sim_time)
                else:
                    self._phase = self._PHASE_DONE
                    logger.info(
                        "TossMotionSource: DONE after %d cycles at t=%.3fs "
                        "— holding at final catch pose",
                        len(self._catches), sim_time)

    def _begin_cycle(self, sim_time: float, state: Any) -> None:
        """Compute this cycle's throw/catch 6-DOF poses and flight time.

        The throw position is read from the live plant state — this is what
        makes chained cycles natural: cycle N+1's throw is where cycle N's
        catch left us.
        """
        throw_pos = np.asarray(state.platform_pos_mm, dtype=float).copy()
        catch_entry = self._catches[self._cycle_idx]
        if catch_entry is None:
            # None sentinel: return to the pose captured at source start.
            if self._initial_pos_mm is None:  # pragma: no cover — defensive
                raise RuntimeError(
                    "catch_position is None but initial pose has not been "
                    "captured yet — update() has not run")
            catch_pos = self._initial_pos_mm.copy()
        else:
            catch_pos = catch_entry.copy()

        self._flight_time_s = flight_time_from_apex(
            throw_pos, catch_pos, self._apex_mm)

        v_launch = compute_launch_velocity(
            throw_pos, catch_pos, self._flight_time_s)
        v_arrival = compute_arrival_velocity(v_launch, self._flight_time_s)

        # Platform +Z aligned with launch velocity at throw; aligned with
        # negated arrival velocity at catch (platform normal faces into the
        # incoming ball).
        throw_rv = compute_orientation(v_launch)
        catch_rv = compute_orientation(-v_arrival)

        self._throw_pose_6dof = np.concatenate([throw_pos, throw_rv])
        self._catch_pose_6dof = np.concatenate([catch_pos, catch_rv])

        self._phase = self._PHASE_APPROACH
        self._phase_start_time = sim_time
        # Force ref_events regeneration on the next update().
        self._cached_target = None
        self._cached_phase = None

        logger.info(
            "TossMotionSource: begin cycle %d at t=%.3fs — "
            "throw=(%.1f,%.1f,%.1f,rv=%.3f,%.3f,%.3f) "
            "catch=(%.1f,%.1f,%.1f,rv=%.3f,%.3f,%.3f) flight=%.3fs",
            self._cycle_idx, sim_time,
            *self._throw_pose_6dof, *self._catch_pose_6dof[:3],
            *catch_rv, self._flight_time_s)
