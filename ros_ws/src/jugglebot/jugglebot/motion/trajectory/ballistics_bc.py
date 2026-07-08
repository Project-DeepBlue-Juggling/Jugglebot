"""Ballistic boundary conditions — touchdown quadratic + launch/arrival velocity.

Pure numpy ballistics (no drag, constant gravity) for the catch/reload path: given
a throw's release state, *when* and *where* does the ball cross the catch height,
and *at what velocity*? Those are the boundary conditions the reach planner
(``planner.build_catch``) and the sim reload gate (``sim/reload_gate.py``) consume:
the touchdown position drives the reach target, and the arrival velocity drives the
receive tilt (``tilt_geometry.tilt_to_receive``) and the hand catch velocity.

**Overlap with ``controller/ballistics.py`` (checked at port time, per the plan).**
``compute_launch_velocity`` / ``compute_arrival_velocity`` there are
*mathematically identical* to :func:`launch_velocity` / :func:`arrival_velocity`
here (same no-drag constant-gravity forms, same ``GRAVITY_MMS2 = 9806`` mm/s²
convention). They are **copied, not imported** — for the exact reason
``quintic.py`` copies ``controller/hermite.py``: ``motion/`` must stay free of any
repo-root / ``controller`` import so it has no path dependence and the dormant
``controller/`` is never reactivated by a live code path. The *new* math this
module adds over ``controller/ballistics.py`` is the **touchdown quadratic**
(``controller/ballistics`` only has the apex-constrained ``flight_time_from_apex``;
the catch path knows the release state, not the apex, so it solves the descending
root of ``z(t) = z_catch`` directly).

Pure Python + numpy. No ROS2 / repo-root / ``controller`` imports.
"""

from __future__ import annotations

import numpy as np

# mm/s² — the mm-native gravity magnitude, matching controller/ballistics.py's
# GRAVITY_MMS2 (motion/ translation units are mm throughout).
GRAVITY_MMS2 = 9806.0

# Downward gravity vector (world), mm/s².
G_VEC_MMS2 = np.array([0.0, 0.0, -GRAVITY_MMS2])


def launch_velocity(release_pos_mm, target_pos_mm, flight_time_s: float) -> np.ndarray:
    """Release velocity (mm/s) for the ball to travel ``release → target`` in
    ``flight_time_s`` under no-drag constant gravity.

    ``v = (p_tgt − p_release)/T − ½·g·T`` (the Ploeger et al. throw boundary
    condition; identical to ``controller.ballistics.compute_launch_velocity``).
    """
    p0 = np.asarray(release_pos_mm, dtype=float).reshape(3)
    p1 = np.asarray(target_pos_mm, dtype=float).reshape(3)
    T = float(flight_time_s)
    if T <= 0.0:
        raise ValueError(f"flight_time_s must be > 0 (got {T})")
    return (p1 - p0) / T - 0.5 * G_VEC_MMS2 * T


def arrival_velocity(launch_vel_mms, flight_time_s: float) -> np.ndarray:
    """Ball velocity (mm/s) after ``flight_time_s`` of free flight from
    ``launch_vel_mms`` (identical to ``controller.ballistics.compute_arrival_velocity``)."""
    v0 = np.asarray(launch_vel_mms, dtype=float).reshape(3)
    return v0 + G_VEC_MMS2 * float(flight_time_s)


def position_at(release_pos_mm, release_vel_mms, t_s: float) -> np.ndarray:
    """Ballistic position (mm) at time ``t_s`` after release: ``p0 + v0·t + ½·g·t²``."""
    p0 = np.asarray(release_pos_mm, dtype=float).reshape(3)
    v0 = np.asarray(release_vel_mms, dtype=float).reshape(3)
    t = float(t_s)
    return p0 + v0 * t + 0.5 * G_VEC_MMS2 * t * t


def velocity_at(release_vel_mms, t_s: float) -> np.ndarray:
    """Ballistic velocity (mm/s) at time ``t_s`` after release: ``v0 + g·t``."""
    return arrival_velocity(release_vel_mms, t_s)


def touchdown_time(release_pos_mm, release_vel_mms, target_z_mm: float,
                   *, descending: bool = True) -> float:
    """Time (s) at which the ball crosses ``target_z_mm`` (world z).

    Solves the vertical quadratic ``z0 + vz·t − ½·g·t² = z_target`` for the
    physically-meaningful crossing:
      * ``descending=True`` (a catch) → the **later** root, when the ball is on the
        way down;
      * ``descending=False`` → the earlier (ascending) crossing.

    Raises ``ValueError`` if the ball never reaches ``target_z_mm`` (the throw
    apexes below the catch height — a genuinely infeasible catch, surfaced loudly
    rather than returning a NaN time).
    """
    p0 = np.asarray(release_pos_mm, dtype=float).reshape(3)
    v0 = np.asarray(release_vel_mms, dtype=float).reshape(3)
    z0 = float(p0[2])
    vz = float(v0[2])
    zt = float(target_z_mm)
    # ½·g·t² − vz·t + (zt − z0) = 0  →  a t² + b t + c = 0.
    a = 0.5 * GRAVITY_MMS2
    b = -vz
    c = zt - z0
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        raise ValueError(
            f"ball never reaches z={zt:.1f} mm from release z={z0:.1f} mm at "
            f"vz={vz:.1f} mm/s (apex below the catch height)")
    sqrt_disc = float(np.sqrt(disc))
    t_early = (-b - sqrt_disc) / (2.0 * a)
    t_late = (-b + sqrt_disc) / (2.0 * a)
    t = t_late if descending else t_early
    if t <= 0.0:
        # Both crossings are in the past for a descending catch only if the ball is
        # already below the target moving down — infeasible as a future catch.
        raise ValueError(
            f"touchdown at z={zt:.1f} mm is not in the future "
            f"(t_early={t_early:.3f}s, t_late={t_late:.3f}s)")
    return float(t)


def arrival_state_at_z(release_pos_mm, release_vel_mms, target_z_mm: float,
                       *, descending: bool = True):
    """Convenience: ``(pos_mm, vel_mms, t_s)`` when the ball crosses ``target_z_mm``.

    The single boundary-condition call the catch path wants: reach target =
    ``pos_mm[:2]`` at ``target_z_mm``, receive tilt from ``vel_mms``, flight time
    ``t_s`` for the arrival-time schedule.
    """
    t = touchdown_time(release_pos_mm, release_vel_mms, target_z_mm,
                       descending=descending)
    return position_at(release_pos_mm, release_vel_mms, t), \
        velocity_at(release_vel_mms, t), t
