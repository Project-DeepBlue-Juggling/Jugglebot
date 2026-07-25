"""Tilt-aimed cup geometry for the online juggle re-architecture (Rung 1+).

The level-platform decoupling (``sim.juggle_online.realize``) forced every
lateral cup motion onto the band-limited platform *translation*. The
re-architecture instead **tilts** the cup: the cup's symmetry axis is rotated so
the ball detaches / seats *along* the tilted axis, with the fast actuator
(slider) supplying speed and a roughly-constant tilt supplying aim. Rung 0
(``tools/probes/juggle_tilt_bandlimit.py``, logbook
``2026-06-29-platform-tilt-tracking-characterisation``) showed the platform
tracks tilt ~6x tighter than lateral translation and that tilt is held
static-exact with a clean, linear lever arm.

This module is the **pure geometry** shared by every tilt-using rung:

* :func:`tilt_to_receive` — the catch tilt (rx, ry) that aligns the cup axis
  *anti-parallel* to the observed arrival velocity (Kai's collinear catch), so
  the ball drops straight down the cup axis with zero lateral velocity in the
  cup frame. Bounded to ``MAX_TILT_DEG``.
* :func:`realize_tilted` — cup Cartesian target (where the ``hand_opening`` site
  must end up) + a tilt -> ``(pose_6dof_mm, slider_mm)`` for the plant, applying
  the **Rung 0 lever-arm compensation**: a tilt swings the cup (≈95 mm above the
  platform centroid) sideways and slightly down, so the centroid is offset to
  cancel that and land the cup opening exactly on the target.

Pure NumPy + the production IK rotation helper; no MuJoCo, no ROS2. SI at the
cup interface, mm at the plant boundary (mirrors the rest of the runner).

NOTE: the realisation constants below mirror ``sim.juggle_online`` (which keeps
its own level-only ``realize`` for the existing 2-ball runner). They are
duplicated rather than imported to keep this module MuJoCo-free; Rung 2a unifies
them when ``juggle_online.realize`` grows orientation.
"""
from __future__ import annotations

import os
import sys

import numpy as np

# Resolve the production motion package for the rotation helper (same path
# dance the plant does), so this module imports standalone in tests/probes.
_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
for _p in (os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(_REPO, 'config', 'generated')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from jugglebot.motion.ik_solver import rotvec_to_rot_matrix
# Single source of truth (single-ball-toss Phase 4): the throw mirror now lives
# in the production trajectory package; re-exported here so every sim
# caller/probe/test keeps importing from sim.juggle_tilt unchanged.
from jugglebot.motion.trajectory.tilt_geometry import tilt_to_throw  # noqa: F401

# ---- Realisation constants (mirror sim.juggle_online; see module docstring) --
Z_ACTIVE_MM = 170.0          # platform centroid height for the juggle pattern
CUP_Z_BASE_MM = 659.6        # cup_z_world = CUP_Z_BASE_MM + slider_mm at level
SLIDER_STROKE_MM = 355.0     # slider travel [0, stroke] mm

# ---- Tilt geometry (from Rung 0 characterisation) ---------------------------
# Usable tilt ceiling — leg headroom is fine to >24 deg, but tilt tracking and
# the small-angle aim model both degrade past ~12 deg (Rung 0).
MAX_TILT_DEG = 12.0
# World z (mm) of the tilt rotation centre — the point the cup (hand_opening
# site) swings ABOUT under a platform tilt. Measured constant at 744.3 mm across
# the whole slider range (a tilt-the-cup probe, this session): the cup is rigidly
# attached above/below this point by ``cup_z_world - 744.3``, so the lever arm is
# NOT a fixed mm/deg — it SCALES with the cup height. At the Rung 0 operating
# slider (180 mm, cup ~840 mm) the arm is +95 mm above centre (the 1.66 mm/deg
# Rung 0 reported); at a low slider (cup < 744 mm) the cup sits BELOW the centre
# and swings the OTHER way. realize_tilted therefore computes the arm from the
# target cup z, which is why this is the load-bearing constant, not a mm/deg.
CUP_TILT_CENTER_Z_MM = 744.3
# Rung 0's reported single-axis lever arm AT THE 180 mm operating slider (mm
# lateral cup shift per degree). Exposed for the probe/tests as the Rung 0
# cross-check; realize_tilted uses the exact height-aware rigid-rotation form
# (CUP_TILT_CENTER_Z_MM) which reproduces the per-slider measurements to <0.1 mm.
LEVER_ARM_MM_PER_DEG = 1.66


def tilt_to_receive(v_arrival, max_tilt_deg: float = MAX_TILT_DEG):
    """Catch tilt (rx, ry) radians aligning the cup axis to receive ``v_arrival``.

    Kai's collinear catch: the cup's up-axis is set **anti-parallel** to the
    ball's arrival velocity, so the ball's velocity in the cup frame is purely
    axial (straight down the cup) with *zero* lateral component — the ball drops
    into the cup instead of skidding across the opening. The required cup axis is
    ``a = -v_arrival / |v_arrival|`` (points up-and-toward the incoming ball,
    since the arrival is descending). The minimal rotation that maps world +z
    onto ``a`` is about the axis ``z x a`` by the angle between them, with **no**
    twist (rz = 0); that rotation vector's x/y components are (rx, ry).

    The from-vertical tilt is clamped to ``max_tilt_deg`` (the lateral aim then
    saturates and a fast lateral arrival is only partially nulled — characterised
    as residual in-cup skid in Rung 1). A straight-down arrival returns level.

    Parameters
    ----------
    v_arrival : (3,) array-like
        Observed ball arrival velocity at the catch (m/s, world). Normally
        descending (vz < 0).

    Returns
    -------
    (rx, ry) : tuple of float
        Platform tilt about world x and y (rad), with rz implicitly 0. Feed
        straight into :func:`realize_tilted` (and the plant pose rot-vector).
    """
    v = np.asarray(v_arrival, dtype=float).reshape(3)
    speed = float(np.linalg.norm(v))
    if speed < 1e-9:
        return 0.0, 0.0
    a = -v / speed                                  # desired cup up-axis (unit)
    lat = a[:2]
    lat_norm = float(np.linalg.norm(lat))
    if lat_norm < 1e-9:
        return 0.0, 0.0                             # straight-down arrival
    angle = float(np.arccos(np.clip(a[2], -1.0, 1.0)))     # from vertical
    angle = min(angle, np.radians(float(max_tilt_deg)))
    # Rotation axis = (z x a) normalised = (-a_y, a_x, 0) / |lat|.
    axis = np.array([-lat[1], lat[0]]) / lat_norm
    rx = float(angle * axis[0])
    ry = float(angle * axis[1])
    return rx, ry


def cup_axis(rx: float, ry: float) -> np.ndarray:
    """World-frame cup up-axis for tilt ``(rx, ry)`` rad (rz = 0). Unit vector."""
    rot = rotvec_to_rot_matrix(np.array([rx, ry, 0.0], dtype=float))
    return rot @ np.array([0.0, 0.0, 1.0])


def cup_lever_arm_mm(cup_z_m: float) -> float:
    """Signed lever arm (mm) of the cup opening above the tilt centre at a given
    target cup height. ``cup_z_world - CUP_TILT_CENTER_Z_MM``: positive when the
    cup rides above the centre (swings WITH +ry toward +x), negative below."""
    return float(cup_z_m) * 1000.0 - CUP_TILT_CENTER_Z_MM


def cup_lateral_shift_mm(rx: float, ry: float, cup_z_m: float) -> np.ndarray:
    """Lateral (xy) shift of the cup opening (mm) under tilt ``(rx, ry)`` rad at
    target height ``cup_z_m``.

    The cup swings about :data:`CUP_TILT_CENTER_Z_MM` on a lever arm of
    ``cup_lever_arm_mm(cup_z_m)``; the rigid rotation moves the opening sideways
    by ``arm * cup_axis_xy``. Rung 0 sign check (arm > 0): +ry swings +x, +rx
    swings -y.
    """
    a = cup_axis(rx, ry)
    return cup_lever_arm_mm(cup_z_m) * a[:2]


def cup_vertical_drop_mm(rx: float, ry: float, cup_z_m: float) -> float:
    """How much lower (mm) the cup opening sits under tilt ``(rx, ry)`` vs level
    at the same slider, for a cup at ``cup_z_m`` — the second-order vertical
    cross-coupling (~2 mm at 12 deg with a +95 mm arm, Rung 0). ``arm * (1 -
    cos(tilt))``; sign follows the arm (a below-centre cup rises slightly)."""
    a = cup_axis(rx, ry)
    return cup_lever_arm_mm(cup_z_m) * (1.0 - float(a[2]))


def realize_tilted(cup_xy_m, cup_z_m: float, rx: float, ry: float):
    """Tilt-aimed cup target -> ``(pose_6dof_mm, slider_mm)`` with lever-arm comp.

    ``cup_xy_m`` / ``cup_z_m`` are where the **cup opening** (``hand_opening``
    site) must end up in the world (m). The platform centroid is offset by the
    height-aware Rung 0 lever arm so the *tilted* cup lands exactly there:

    * centroid_xy = target_cup_xy - lateral_shift(rx, ry, cup_z)   (cancel swing)
    * slider raised by the vertical drop so the cup z still hits the target.

    Returns a ``[x, y, Z_ACTIVE_MM, rx, ry, 0]`` pose (mm/rad) and the slider
    setpoint (mm, clamped to the stroke).
    """
    cup_xy_mm = np.asarray(cup_xy_m, dtype=float).reshape(2) * 1000.0
    shift_mm = cup_lateral_shift_mm(rx, ry, cup_z_m)
    centroid_xy = cup_xy_mm - shift_mm
    pose = np.array([centroid_xy[0], centroid_xy[1], Z_ACTIVE_MM,
                     float(rx), float(ry), 0.0])
    drop_mm = cup_vertical_drop_mm(rx, ry, cup_z_m)
    slider = float(np.clip(float(cup_z_m) * 1000.0 - CUP_Z_BASE_MM + drop_mm,
                           0.0, SLIDER_STROKE_MM))
    return pose, slider
