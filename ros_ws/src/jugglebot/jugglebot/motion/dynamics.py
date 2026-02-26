"""Stewart platform dynamics — gravity compensation and feedforward.

Provides the dynamics computations needed for Phase 3 torque control:
  - Gravity wrench computation (with CoM offset)
  - Leg force decomposition via Jacobian transpose solve
  - Motor torque conversion for gravity feedforward
  - Reflected motor inertia documentation (Phase 5)

Pure Python + numpy.  No ROS2 dependency.

Sign conventions
----------------
- **Gravity wrench**: ``[Fx, Fy, Fz, tau_x, tau_y, tau_z]`` in base frame.
  Force in N, torque in N·mm (consistent with Jacobian units).
  Gravity acts downward: ``Fz = -m·g``.
- **Leg forces**: positive = extension direction (push platform up).
- **Motor torques**: positive = extension direction (Jugglebot convention).
  CAN node handles inversion to ODrive convention.

Force decomposition
-------------------
Our Jacobian maps platform twist → leg velocities: ``q_dot = J · x_dot``.
By virtual work: ``J^T · f = W`` (leg forces → platform wrench).
Therefore leg forces from a required wrench: ``f = J^{-T} · W``,
implemented as ``f = np.linalg.solve(J.T, W)``.
"""

from __future__ import annotations

import numpy as np

import jugglebot.hardware_config as hw
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import compute_jacobian
from jugglebot.motion.conversions import leg_forces_to_motor_torques


class DynamicsParams:
    """Container for platform dynamics parameters.

    Loaded from the generated hardware_config module
    (source: config/hardware_config.yaml, dynamics section).
    """

    def __init__(self, mass_kg: float, com_offset_mm: np.ndarray,
                 gravity_mps2: float):
        self.mass_kg = mass_kg
        self.com_offset_mm = np.asarray(com_offset_mm, dtype=np.float64)
        self.gravity_mps2 = gravity_mps2

    @classmethod
    def from_config(cls) -> 'DynamicsParams':
        """Load dynamics parameters from generated hardware_config."""
        return cls(
            mass_kg=float(hw.DYNAMICS_PLATFORM_MASS_KG),
            com_offset_mm=hw.DYNAMICS_PLATFORM_COM_OFFSET_MM,
            gravity_mps2=float(hw.GRAVITY_MPS2),
        )


# ---------------------------------------------------------------------------
# Gravity wrench
# ---------------------------------------------------------------------------

def compute_gravity_wrench(rot: np.ndarray,
                           params: DynamicsParams) -> np.ndarray:
    """Compute the 6D gravity wrench about the platform centre in world frame.

    Parameters
    ----------
    rot : (3, 3) ndarray
        Rotation matrix from platform frame to base/world frame.
    params : DynamicsParams

    Returns
    -------
    W_gravity : (6,) ndarray
        ``[Fx, Fy, Fz, tau_x, tau_y, tau_z]``.
        Force in N, torque in N·mm.
        Gravity force is ``[0, 0, -m·g]``.  Torque is the moment of gravity
        about the platform geometric centre due to CoM offset.
    """
    m = params.mass_kg
    g = params.gravity_mps2

    # Gravity force in world frame (N)
    F_gravity = np.array([0.0, 0.0, -m * g])

    # CoM position relative to platform centre, in world frame (mm)
    r_com_world = rot @ params.com_offset_mm

    # Moment of gravity about platform centre (N·mm)
    # tau = r_com (mm) x F_gravity (N) → N·mm
    tau_gravity = np.cross(r_com_world, F_gravity)

    W = np.empty(6)
    W[:3] = F_gravity
    W[3:] = tau_gravity
    return W


# ---------------------------------------------------------------------------
# Leg force decomposition
# ---------------------------------------------------------------------------

def gravity_to_leg_forces(pos: np.ndarray, rot: np.ndarray,
                          geom: StewartGeometry,
                          params: DynamicsParams) -> np.ndarray:
    """Compute per-leg forces (N) needed to support the platform against gravity.

    Uses ``f = J^{-T} · W_support`` where ``W_support = -W_gravity``.

    Parameters
    ----------
    pos : (3,) ndarray — platform offset from home (mm)
    rot : (3, 3) ndarray — rotation matrix (platform → world)
    geom : StewartGeometry
    params : DynamicsParams

    Returns
    -------
    f_legs : (6,) ndarray — per-leg forces in N.
        Positive = extension direction (push platform up).
    """
    J = compute_jacobian(pos, rot, geom)
    W_gravity = compute_gravity_wrench(rot, params)
    W_support = -W_gravity  # wrench that the legs must produce

    # Solve J^T · f = W_support for f
    f_legs = np.linalg.solve(J.T, W_support)
    return f_legs


def gravity_to_motor_torques(pos: np.ndarray, rot: np.ndarray,
                             geom: StewartGeometry,
                             params: DynamicsParams) -> np.ndarray:
    """Compute per-motor torques (Nm) for gravity compensation.

    Parameters
    ----------
    pos : (3,) ndarray — platform offset from home (mm)
    rot : (3, 3) ndarray — rotation matrix (platform → world)
    geom : StewartGeometry
    params : DynamicsParams

    Returns
    -------
    torques_Nm : (6,) ndarray — motor torques in Nm (Jugglebot convention).
    """
    f_legs = gravity_to_leg_forces(pos, rot, geom, params)
    return leg_forces_to_motor_torques(f_legs, geom)


# ---------------------------------------------------------------------------
# Reflected motor inertia (documented for Phase 5)
# ---------------------------------------------------------------------------

def compute_reflected_inertia(geom: StewartGeometry,
                              rotor_inertia_kgm2: float) -> np.ndarray:
    """Compute reflected motor inertia per leg at the leg output.

    The reflected inertia at the leg (in kg) is:
        J_reflected = J_rotor / r_spool^2

    where r_spool is the spool radius in metres.  This represents the
    effective mass that the motor inertia adds to each leg's dynamics.

    This value is DOCUMENTED ONLY in Phase 3.  It will be used in Phase 5
    for inertia feedforward: tau_reflected = J_rotor * alpha_motor.

    Parameters
    ----------
    geom : StewartGeometry
    rotor_inertia_kgm2 : float — motor rotor inertia in kg·m^2

    Returns
    -------
    reflected_inertia_kg : (6,) ndarray — per-leg reflected inertia in kg.
    """
    spool_radius_m = geom.spool_radius_mm / 1000.0  # (6,)
    return rotor_inertia_kgm2 / (spool_radius_m ** 2)
