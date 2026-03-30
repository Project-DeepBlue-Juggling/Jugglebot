"""Target-setting interface for the MPC controller.

Defines the ``TargetCommand`` data structure and the ``TargetSource`` protocol
that all input modes (spacemouse, GUI, shell, catch coordinator) must satisfy.

The MPC solver consumes ``TargetCommand`` each control step:
    target_pose   → reference pose for the optimization
    arrival_time  → deadline (absolute time) or None for ASAP
    target_twist  → desired velocity at arrival (or None = hold)
    ref_events    → optional time-varying reference trajectory (list of events)

This module has NO sim-specific dependencies (no MuJoCo, no hand/ball types).
Sim-specific fields (hand_cmd, ball_spawn) are added by a subclass in
``sim/main.py``.  When ``controller/`` is extracted to a top-level package
(Phase 3F), this file moves with it unchanged.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, List, Optional, Protocol, Tuple, runtime_checkable

import numpy as np


@dataclass
class ReferenceEvent:
    """A timed waypoint for the MPC reference trajectory.

    Used to build a time-varying reference that the MPC tracks across its
    horizon.  The MPC interpolates between events using quintic Hermite
    splines when acceleration is provided (C2-continuous), falling back to
    cubic Hermite (C1) when acceleration is ``None``.

    Parameters
    ----------
    time : float
        Absolute time (seconds, same clock as ``PlantState.time``).
    pose : (6,) ndarray
        Target platform pose [x, y, z, rx, ry, rz] in mm / rad.
    twist : (6,) ndarray or None
        Desired twist [vx, vy, vz, wx, wy, wz] in mm/s / rad/s at this
        event.  ``None`` is treated as zero (hold at pose).
    accel : (6,) ndarray or None
        Acceleration [ax, ay, az, alphax, alphay, alphaz] in mm/s² / rad/s²
        at this event.  When both bracketing events have ``accel``, the MPC
        uses quintic Hermite interpolation (C2-continuous).  ``None`` falls
        back to cubic Hermite (C1).
    """
    time: float
    pose: np.ndarray
    twist: np.ndarray | None = None
    accel: np.ndarray | None = None


def sample_ref_fn(
    ref_fn: Callable[[float], Tuple[np.ndarray, np.ndarray]],
    t_now: float,
    cumulative_times: np.ndarray,
) -> List[ReferenceEvent]:
    """Sample a reference callback at horizon node times to produce events.

    Convenience adapter: wraps an existing ``ref_fn(t) -> (pose, twist)``
    callable into a list of ``ReferenceEvent`` that the MPC can consume.

    Parameters
    ----------
    ref_fn : callable
        ``ref_fn(t_abs) -> (pose_6, twist_6)`` returning numpy arrays.
    t_now : float
        Current absolute time (same clock as ``PlantState.time``).
    cumulative_times : (N+1,) ndarray
        Relative times at each horizon node (from ``MPCParams.cumulative_times``).

    Returns
    -------
    list[ReferenceEvent]
        One event per horizon node, sorted by time.
    """
    events = []
    for dt in cumulative_times:
        t_abs = t_now + dt
        pose, twist = ref_fn(t_abs)
        events.append(ReferenceEvent(
            time=t_abs,
            pose=np.asarray(pose, dtype=np.float64),
            twist=np.asarray(twist, dtype=np.float64),
        ))
    return events


_ZERO6 = np.zeros(6)


def flat_target_to_events(
    current_pose: np.ndarray,
    current_twist: np.ndarray,
    target_pose: np.ndarray,
    t_now: float,
    response_time: float = 0.3,
    target_twist: np.ndarray | None = None,
    arrival_time: float | None = None,
) -> List[ReferenceEvent]:
    """Convert a flat target into a 2-event quintic reference.

    Produces two ``ReferenceEvent`` objects with zero acceleration at both
    endpoints.  The MPC quintic-interpolates between them, generating a
    smooth S-curve from the current state to the target.

    Parameters
    ----------
    current_pose : (6,) ndarray
        Current platform pose [x, y, z, rx, ry, rz] in mm / rad.
    current_twist : (6,) ndarray
        Current platform twist [vx, vy, vz, wx, wy, wz] in mm/s / rad/s.
    target_pose : (6,) ndarray
        Desired target pose.
    t_now : float
        Current absolute time (seconds).
    response_time : float
        Duration for the quintic transition (seconds).  Shorter = faster
        response.  Used when ``arrival_time`` is None.
    target_twist : (6,) ndarray or None
        Desired twist at the target.  None = zero (hold at target).
    arrival_time : float or None
        Absolute arrival time.  If provided, overrides ``response_time``.

    Returns
    -------
    list[ReferenceEvent]
        Two events: current state at ``t_now`` and target at arrival.
    """
    tw_end = target_twist if target_twist is not None else _ZERO6

    # Single hold-at-target event: the MPC sees the target at the
    # specified time and forward-extrapolates (hold) for later nodes.
    # Nodes before the event get backward-extrapolated.  This gives the
    # MPC full freedom to plan its own optimal approach path rather than
    # constraining it to follow a prescribed ramp.
    t_event = arrival_time if (arrival_time is not None
                               and arrival_time > t_now) else t_now
    return [
        ReferenceEvent(
            time=t_event,
            pose=np.asarray(target_pose, dtype=np.float64),
            twist=np.asarray(tw_end, dtype=np.float64),
            accel=_ZERO6.copy(),
        ),
    ]


@dataclass
class TargetCommand:
    """Command returned by a target source each MPC control step.

    Fields
    ------
    target_pose : (6,) ndarray
        Target platform pose [x, y, z, rx, ry, rz] in mm / rad.
    arrival_time : float or None
        Absolute time to arrive at the target.  ``None`` = converge ASAP.
    target_twist : (6,) ndarray or None
        Desired twist [vx, vy, vz, wx, wy, wz] at arrival.  ``None`` = zero
        (hold at target).
    ref_events : list[ReferenceEvent] or None
        Optional time-varying reference trajectory.  When provided, the MPC
        uses quintic Hermite interpolation (C2) between events that have
        acceleration, falling back to cubic Hermite (C1) otherwise.  Takes
        precedence over the flat-reference path (target_pose repeated at all
        nodes).  ``None`` = legacy flat-reference behavior.
    """
    target_pose: np.ndarray
    arrival_time: float | None = None
    target_twist: np.ndarray | None = None
    ref_events: List[ReferenceEvent] | None = None
    boost_vel_weights: bool = False


@runtime_checkable
class TargetSource(Protocol):
    """Protocol for objects that provide MPC targets each control step.

    Implementations must provide ``update(sim_time, state) -> TargetCommand``.
    Optional lifecycle methods (``reset``, ``close``, ``key_callback``, etc.)
    are duck-typed by the control loop — they are NOT part of this protocol.

    Parameters
    ----------
    sim_time : float
        Current simulation / wall-clock time (seconds).
    state : PlantState (or compatible)
        Current plant state snapshot.  Typed as ``Any`` here to avoid a
        dependency on ``plant.interface`` — concrete implementations should
        type-narrow to ``PlantState``.

    Returns
    -------
    TargetCommand
        The MPC target for this control step.
    """
    def update(self, sim_time: float, state: Any) -> TargetCommand: ...
