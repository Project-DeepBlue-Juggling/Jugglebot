"""Unit conversions for the Stewart platform actuator system.

Provides bidirectional conversion between:
  - Leg forces (N) ↔ motor torques (Nm)
  - Leg extensions (mm) ↔ motor revolutions (rev)

The string-driven actuators have a fixed geometric relationship between
linear leg motion and rotary motor motion, determined by the spool radius.
Per-leg spool radii are derived from the experimentally-measured mm_to_rev
conversion factors in hardware_config.yaml.

Pure Python + numpy, no ROS2 dependency.
"""

from __future__ import annotations

import numpy as np

from jugglebot.motion.geometry import StewartGeometry


# ---------------------------------------------------------------------------
# Force ↔ torque
# ---------------------------------------------------------------------------

def leg_forces_to_motor_torques(forces_N: np.ndarray,
                                geom: StewartGeometry) -> np.ndarray:
    """Convert leg axial forces (N) to motor torques (Nm).

    The string wraps around a spool of effective radius ``r``.
    Power balance:  F·v = τ·ω  ⟹  τ = F·r

    Parameters
    ----------
    forces_N : (6,) ndarray — leg forces in Newtons (positive = tension)
    geom : StewartGeometry

    Returns
    -------
    torques_Nm : (6,) ndarray — motor torques in Newton-metres
    """
    spool_radius_m = geom.spool_radius_mm / 1000.0  # (6,)
    return forces_N * spool_radius_m


def motor_torques_to_leg_forces(torques_Nm: np.ndarray,
                                geom: StewartGeometry) -> np.ndarray:
    """Convert motor torques (Nm) to leg axial forces (N).

    Inverse of ``leg_forces_to_motor_torques``.
    """
    spool_radius_m = geom.spool_radius_mm / 1000.0
    return torques_Nm / spool_radius_m


# ---------------------------------------------------------------------------
# Extension ↔ revolutions
# ---------------------------------------------------------------------------

def extensions_mm_to_revs(extensions_mm: np.ndarray,
                          geom: StewartGeometry) -> np.ndarray:
    """Convert leg extensions (mm) to motor revolutions.

    Parameters
    ----------
    extensions_mm : (6,) ndarray — leg extensions from retracted position
    geom : StewartGeometry

    Returns
    -------
    revs : (6,) ndarray — motor position in revolutions
    """
    return extensions_mm * geom.mm_to_rev


def revs_to_extensions_mm(revs: np.ndarray,
                          geom: StewartGeometry) -> np.ndarray:
    """Convert motor revolutions to leg extensions (mm).

    Inverse of ``extensions_mm_to_revs``.
    """
    return revs / geom.mm_to_rev


# ---------------------------------------------------------------------------
# Velocity conversions
# ---------------------------------------------------------------------------

def leg_velocities_to_motor_velocities(vel_mm_s: np.ndarray,
                                       geom: StewartGeometry) -> np.ndarray:
    """Convert leg extension velocities (mm/s) to motor velocities (rev/s)."""
    return vel_mm_s * geom.mm_to_rev


def motor_velocities_to_leg_velocities(vel_rps: np.ndarray,
                                       geom: StewartGeometry) -> np.ndarray:
    """Convert motor velocities (rev/s) to leg extension velocities (mm/s)."""
    return vel_rps / geom.mm_to_rev
