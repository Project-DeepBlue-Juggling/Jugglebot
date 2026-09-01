"""Jugglebot MVP trajectory generator (pure Python + numpy).

A deliberately simple, shallowly kinetics-aware Jetson-side trajectory generator
that streams 40 Hz waypoint knots to the can-hub Teensy over the validated
setpoint path — the MVP replacement for the compute-marginal MPC hot loop
(``controller/`` stays dormant and untouched). See
``plans/active/mvp-trajectory-bringup.md``.

This package is **pure**: numpy only, no ROS2 imports, no repo-root /
``controller`` imports. The thin ROS wrapper lives in
``jugglebot/trajectory_node.py``.

Public surface (Phase 1):
  * :class:`TrajectoryLimits` — leg vel/acc/jerk ceilings + knot spacing.
  * :class:`QuinticSegment`, :class:`TrajectoryPlan`, :func:`HoldPlan`.
  * :mod:`planner` constructors (all gate through :func:`feasibility.validate`).
  * :class:`FeasibilityReport`, :class:`TrajectoryInfeasible`.
  * :class:`KnotEmitter` — plan sample → ``make_mpc_command`` frame.
"""

from __future__ import annotations

from jugglebot.motion.trajectory.emitter import KnotEmitter
from jugglebot.motion.trajectory.feasibility import (
    FeasibilityReport,
    TrajectoryInfeasible,
    validate,
    validate_cycle,
    validate_follow,
)
from jugglebot.motion.trajectory.follower import FollowResult, TargetFollower
from jugglebot.motion.trajectory.limits import TrajectoryLimits
from jugglebot.motion.trajectory.plan import HoldPlan, TrajectoryPlan
from jugglebot.motion.trajectory.segment import QuinticSegment
from jugglebot.motion.trajectory.shaping import LeanShaper
from jugglebot.motion.trajectory.tilt_geometry import (
    MAX_TILT_DEG,
    cup_axis,
    tilt_to_receive,
    tilt_to_receive_deg,
    tilt_to_throw,
)

__all__ = [
    'KnotEmitter',
    'FeasibilityReport',
    'TrajectoryInfeasible',
    'validate',
    'validate_cycle',
    'validate_follow',
    'FollowResult',
    'TargetFollower',
    'TrajectoryLimits',
    'HoldPlan',
    'TrajectoryPlan',
    'QuinticSegment',
    'LeanShaper',
    'MAX_TILT_DEG',
    'cup_axis',
    'tilt_to_receive',
    'tilt_to_receive_deg',
    'tilt_to_throw',
]
