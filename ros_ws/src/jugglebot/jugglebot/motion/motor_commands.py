"""Cartesian state → motor command mapping.

Bridges the trajectory evaluation layer (quintic polynomials in Cartesian
space) with the motor execution layer (ODrive position/vel_ff/torque_ff
commands) using IK and dynamics.

Pure Python + numpy.  No ROS2, no ZeroMQ dependency.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from jugglebot.motion.conversions import (
    extensions_mm_to_revs,
    leg_velocities_to_motor_velocities,
)
from jugglebot.motion.dynamics import (
    DynamicsParams,
    compute_full_feedforward_torques,
)
from jugglebot.motion.ik_solver import (
    compute_jacobian,
    pose_to_leg_lengths,
    rotvec_to_rot_matrix,
    twist_to_leg_velocities,
)

if TYPE_CHECKING:
    from jugglebot.motion.geometry import StewartGeometry


def cartesian_to_motor_commands(
    pose: np.ndarray,
    twist: np.ndarray,
    accel: np.ndarray,
    geom: StewartGeometry,
    dynamics_params: DynamicsParams,
    feedforward_enabled: bool = True,
    gravity_correction: np.ndarray | None = None,
    skip_reflected_inertia: bool = False,
    skip_gravity: bool = False,
    skip_platform_inertia: bool = False,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Convert a Cartesian state to motor commands.

    Maps the trajectory evaluator output to the ODrive command triple
    ``(input_pos, vel_ff, torque_ff)`` using IK and dynamics.

    Computes the Jacobian once internally and reuses it across velocity IK
    and feedforward torque computation (avoids 3 redundant evaluations).

    Parameters
    ----------
    pose : (6,) ndarray — [x, y, z, rx, ry, rz] in mm, rad
    twist : (6,) ndarray — [vx, vy, vz, wx, wy, wz] in mm/s, rad/s
    accel : (6,) ndarray — [ax, ay, az, alphax, alphay, alphaz] in mm/s², rad/s²
    geom : StewartGeometry
    dynamics_params : DynamicsParams
    feedforward_enabled : bool
        Whether to include torque_ff (gravity + inertia).
    gravity_correction : (3,3) ndarray or None
        Pre-multiply rotation to align platform with gravity.
    skip_reflected_inertia : bool
    skip_gravity / skip_platform_inertia : bool
        Per-term feedforward gates, mirroring ``dynamics.torque_ff_gravity`` /
        ``dynamics.torque_ff_platform_inertia`` in the config.  Producers that
        must honour the config flags (HardwarePlant) pass these through;
        see compute_full_feedforward_torques for why platform inertia ships
        gated OFF (the firmware's undecayed stale-link torque hold).
        Skip reflected motor inertia (avoids expensive J_dot computation).

    Returns
    -------
    pos_rev : (6,) ndarray — motor positions in revolutions
    vel_ff_rps : (6,) ndarray — velocity feedforward in rev/s
    torque_ff_Nm : (6,) ndarray — torque feedforward in Nm
    J : (6,6) ndarray — Jacobian at this pose (for reuse by caller,
        e.g. condition number check)
    """
    pos = pose[:3]
    rot = rotvec_to_rot_matrix(pose[3:6])
    if gravity_correction is not None:
        rot = gravity_correction @ rot

    # Compute Jacobian once for this pose — reused by velocity IK,
    # feedforward torques, and returned for condition number check.
    J = compute_jacobian(pos, rot, geom)

    # Position IK: pose -> leg extensions (mm) -> motor revolutions
    extensions_mm = pose_to_leg_lengths(pos, rot, geom)
    pos_rev = extensions_mm_to_revs(extensions_mm, geom)

    # Velocity IK: twist -> leg velocities (mm/s) -> motor rev/s
    vel_mm_s = twist_to_leg_velocities(twist, pos, rot, geom, J=J)
    vel_ff_rps = leg_velocities_to_motor_velocities(vel_mm_s, geom)

    # Torque feedforward: gravity + platform inertia + reflected motor inertia
    if feedforward_enabled:
        torque_ff_Nm = compute_full_feedforward_torques(
            pos, rot, twist, accel, geom, dynamics_params, J=J,
            skip_reflected_inertia=skip_reflected_inertia,
            skip_gravity=skip_gravity,
            skip_platform_inertia=skip_platform_inertia)
    else:
        torque_ff_Nm = np.zeros(6)

    return pos_rev, vel_ff_rps, torque_ff_Nm, J
