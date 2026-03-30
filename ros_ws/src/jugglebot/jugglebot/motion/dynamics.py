"""Stewart platform dynamics — gravity and inertia feedforward.

Provides the dynamics computations needed for motion control:
  - Gravity wrench computation (with CoM offset) — Phase 3
  - Leg force decomposition via Jacobian transpose solve — Phase 3
  - Motor torque conversion for gravity feedforward — Phase 3
  - Reflected motor inertia per leg — Phase 3 (documented), Phase 5 (used)
  - Platform inertia wrench (Newton-Euler) — Phase 5
  - Full feedforward torques (gravity + platform inertia + reflected motor) — Phase 5

Pure Python + numpy.  No ROS2 dependency.

Sign conventions

----------------
- **Gravity wrench**: ``[Fx, Fy, Fz, tau_x, tau_y, tau_z]`` in base frame.
  Force in N, torque in N·mm (consistent with Jacobian units).
  Gravity acts downward: ``Fz = -m·g``.
- **Inertia wrench**: ``[Fx, Fy, Fz, tau_x, tau_y, tau_z]`` in base frame.
  Force in N, torque in N·mm.  Computed from platform acceleration via
  Newton-Euler: ``F = m·a_com``, ``tau = I·alpha + omega x I·omega``.
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

import logging

import numpy as np

import jugglebot.hardware_config as hw

logger = logging.getLogger(__name__)
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import compute_jacobian
from jugglebot.motion.conversions import (
    leg_forces_to_motor_torques,
    leg_velocities_to_motor_velocities,
)


class DynamicsParams:
    """Container for platform dynamics parameters.

    Loaded from the generated hardware_config module
    (source: config/hardware_config.yaml, dynamics section).
    """

    def __init__(self, mass_kg: float, com_offset_mm: np.ndarray,
                 gravity_mps2: float,
                 inertia_tensor_kgmm2: np.ndarray | None = None,
                 motor_rotor_inertia_kgm2: float = 0.0):
        self.mass_kg = mass_kg
        self.com_offset_mm = np.asarray(com_offset_mm, dtype=np.float64)
        self.gravity_mps2 = gravity_mps2
        # Platform inertia tensor about CoM in body frame (kg·mm²)
        if inertia_tensor_kgmm2 is not None:
            self.inertia_tensor_kgmm2 = np.asarray(
                inertia_tensor_kgmm2, dtype=np.float64)
        else:
            self.inertia_tensor_kgmm2 = np.zeros((3, 3))
        # Motor rotor inertia (kg·m²) — for reflected inertia feedforward
        self.motor_rotor_inertia_kgm2 = motor_rotor_inertia_kgm2

    @classmethod
    def from_config(cls) -> 'DynamicsParams':
        """Load dynamics parameters from generated hardware_config."""
        # Build inertia tensor from flattened config constants (kg·mm²)
        inertia = np.array([
            [hw.DYNAMICS_PLATFORM_INERTIA_TENSOR_KGMM2_IXX,
             hw.DYNAMICS_PLATFORM_INERTIA_TENSOR_KGMM2_IXY,
             hw.DYNAMICS_PLATFORM_INERTIA_TENSOR_KGMM2_IXZ],
            [hw.DYNAMICS_PLATFORM_INERTIA_TENSOR_KGMM2_IXY,
             hw.DYNAMICS_PLATFORM_INERTIA_TENSOR_KGMM2_IYY,
             hw.DYNAMICS_PLATFORM_INERTIA_TENSOR_KGMM2_IYZ],
            [hw.DYNAMICS_PLATFORM_INERTIA_TENSOR_KGMM2_IXZ,
             hw.DYNAMICS_PLATFORM_INERTIA_TENSOR_KGMM2_IYZ,
             hw.DYNAMICS_PLATFORM_INERTIA_TENSOR_KGMM2_IZZ],
        ], dtype=np.float64)

        mass = float(hw.DYNAMICS_PLATFORM_MASS_KG)
        gravity = float(hw.GRAVITY_MPS2)
        motor_inertia = float(hw.DYNAMICS_MOTOR_ROTOR_INERTIA_KGM2)
        assert mass > 0, f"mass must be positive, got {mass}"
        assert gravity > 0, f"gravity must be positive, got {gravity}"
        assert motor_inertia >= 0, f"motor inertia must be non-negative, got {motor_inertia}"
        assert not np.any(np.isnan(inertia)), "inertia tensor contains NaN"

        return cls(
            mass_kg=mass,
            com_offset_mm=hw.DYNAMICS_PLATFORM_COM_OFFSET_MM,
            gravity_mps2=gravity,
            inertia_tensor_kgmm2=inertia,
            motor_rotor_inertia_kgm2=motor_inertia,
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
# Platform inertia wrench (Phase 5)
# ---------------------------------------------------------------------------

def compute_inertia_wrench(rot: np.ndarray,
                           twist: np.ndarray,
                           accel: np.ndarray,
                           params: DynamicsParams) -> np.ndarray:
    """Compute the 6D inertia wrench from platform acceleration.

    Full Newton-Euler dynamics about the platform geometric centre:
      F_inertia = m · a_com  (translational, N)
      tau_inertia = I_world · alpha + omega x (I_world · omega)  (rotational, N·mm)

    The inertia tensor is rotated from body frame to world frame:
      I_world = R · I_body · R^T

    CoM offset correction: the translational acceleration includes the
    contribution from angular acceleration and angular velocity at the
    CoM location: a_com = a_centre + alpha x r_com + omega x (omega x r_com).

    Parameters
    ----------
    rot : (3, 3) ndarray
        Rotation matrix from platform frame to base/world frame.
    twist : (6,) ndarray
        ``[vx, vy, vz, wx, wy, wz]`` in mm/s and rad/s (base frame).
    accel : (6,) ndarray
        ``[ax, ay, az, alphax, alphay, alphaz]`` in mm/s² and rad/s² (base frame).
    params : DynamicsParams

    Returns
    -------
    W_inertia : (6,) ndarray
        ``[Fx, Fy, Fz, tau_x, tau_y, tau_z]``.
        Force in N, torque in N·mm.
    """
    m = params.mass_kg
    omega = twist[3:]     # angular velocity (rad/s)
    a_centre = accel[:3]  # linear acceleration of platform centre (mm/s²)
    alpha = accel[3:]     # angular acceleration (rad/s²)

    # CoM offset in world frame (mm)
    r_com_world = rot @ params.com_offset_mm

    # Linear acceleration of CoM (mm/s²):
    # a_com = a_centre + alpha x r_com + omega x (omega x r_com)
    a_com = (a_centre
             + np.cross(alpha, r_com_world)
             + np.cross(omega, np.cross(omega, r_com_world)))

    # Translational: F = m * a_com  (mm/s² → N requires mm→m: F_N = m * a_mm/s² / 1000)
    # But our wrench convention uses N for force and N·mm for torque.
    # a_com is in mm/s², so F = m * a / 1000 to get N.
    F_inertia = m * a_com / 1000.0  # N

    # Rotational inertia tensor in world frame (kg·mm²)
    I_world = rot @ params.inertia_tensor_kgmm2 @ rot.T

    # Rotational: tau = I * alpha + omega x (I * omega)
    # Units: (kg·mm²) * (rad/s²) = kg·mm²/s² = (kg·m²/s²) * (1e6 mm²/m²)
    #   but we want N·mm = kg·m/s² · mm = kg·mm²/s² / mm * mm = kg·mm²/s²·(1/1000)
    # Actually: kg·mm² * rad/s² = kg·mm²·s⁻² = (N·m)·(mm²/m²)·(s⁻²/s⁻²)
    # Let's be careful:
    #   [I] = kg·mm²,  [alpha] = rad/s²
    #   I·alpha = kg·mm²/s²
    #   To convert to N·mm: 1 N·mm = 1 kg·m/s² · mm = 1 kg·m·mm/s²
    #     = 1 kg · (1000 mm) · mm / s² = 1000 kg·mm²/s²
    #   Therefore: tau(N·mm) = I(kg·mm²) · alpha(rad/s²) / 1000
    I_alpha = I_world @ alpha
    I_omega = I_world @ omega
    tau_inertia = (I_alpha + np.cross(omega, I_omega)) / 1000.0  # N·mm

    W = np.empty(6)
    W[:3] = F_inertia
    W[3:] = tau_inertia
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
    pos : (3,) ndarray — platform offset from stow pose (mm)
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
    try:
        f_legs = np.linalg.solve(J.T, W_support)
    except np.linalg.LinAlgError:
        logger.warning("gravity_to_leg_forces: singular Jacobian — returning zero forces")
        return np.zeros(6)
    return f_legs


def gravity_to_motor_torques(pos: np.ndarray, rot: np.ndarray,
                             geom: StewartGeometry,
                             params: DynamicsParams) -> np.ndarray:
    """Compute per-motor torques (Nm) for gravity compensation.

    Parameters
    ----------
    pos : (3,) ndarray — platform offset from stow pose (mm)
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
# Full feedforward torques (Phase 5)
# ---------------------------------------------------------------------------

def compute_full_feedforward_torques(
    pos: np.ndarray,
    rot: np.ndarray,
    twist: np.ndarray,
    accel: np.ndarray,
    geom: StewartGeometry,
    params: DynamicsParams,
    J: np.ndarray | None = None,
) -> np.ndarray:
    """Compute the full feedforward motor torques (gravity + inertia + reflected motor).

    The total feedforward torque per motor is the sum of three components:

    1. **Gravity + platform inertia** (wrench-based):
       Combined wrench W_total = W_support + W_inertia is decomposed into
       per-leg forces via J^{-T}, then converted to motor torques via
       spool radius.

    2. **Reflected motor inertia** (per-leg):
       Each motor has reflected inertia J_rotor at the motor shaft.
       The feedforward torque is: tau_reflected = J_rotor * alpha_motor,
       where alpha_motor is the motor angular acceleration in rad/s².

    Parameters
    ----------
    pos : (3,) ndarray — platform offset from stow pose (mm)
    rot : (3, 3) ndarray — rotation matrix (platform → world)
    twist : (6,) ndarray — [vx, vy, vz, wx, wy, wz] in mm/s, rad/s
    accel : (6,) ndarray — [ax, ay, az, alphax, alphay, alphaz] in mm/s², rad/s²
    geom : StewartGeometry
    params : DynamicsParams
    J : (6,6) ndarray or None — pre-computed Jacobian (skips recomputation)

    Returns
    -------
    torque_ff_Nm : (6,) ndarray — total feedforward motor torques in Nm
    """
    from jugglebot.motion.ik_solver import (
        accel_to_leg_accels,
        twist_to_leg_velocities,
    )

    if J is None:
        J = compute_jacobian(pos, rot, geom)

    # --- Component 1: Gravity wrench ---
    W_gravity = compute_gravity_wrench(rot, params)
    W_support = -W_gravity

    # --- Component 2: Platform inertia wrench ---
    W_inertia = compute_inertia_wrench(rot, twist, accel, params)

    # --- Combined wrench → per-leg forces → motor torques ---
    W_total = W_support + W_inertia
    try:
        f_legs = np.linalg.solve(J.T, W_total)
    except np.linalg.LinAlgError:
        return np.zeros(6)
    torque_platform = leg_forces_to_motor_torques(f_legs, geom)

    # --- Component 3: Reflected motor inertia ---
    # Motor angular acceleration: q_ddot (mm/s²) → rev/s² → rad/s²
    # q_ddot = J @ accel + J_dot @ twist (mm/s²)
    q_ddot_mm_s2 = accel_to_leg_accels(accel, twist, pos, rot, geom, J=J)
    # Convert mm/s² → rev/s² using mm_to_rev, then → rad/s² (* 2π)
    q_ddot_rps2 = q_ddot_mm_s2 * geom.mm_to_rev          # rev/s²
    q_ddot_rad_s2 = q_ddot_rps2 * (2.0 * np.pi)           # rad/s²
    torque_reflected = params.motor_rotor_inertia_kgm2 * q_ddot_rad_s2  # Nm

    return torque_platform + torque_reflected


# ---------------------------------------------------------------------------
# Reflected motor inertia
# ---------------------------------------------------------------------------

def compute_reflected_inertia(geom: StewartGeometry,
                              rotor_inertia_kgm2: float) -> np.ndarray:
    """Compute reflected motor inertia per leg at the leg output.

    The reflected inertia at the leg (in kg) is:
        J_reflected = J_rotor / r_spool^2

    where r_spool is the spool radius in metres.  This represents the
    effective mass that the motor inertia adds to each leg's dynamics.

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
