"""Gravity-levelling frame — the single implementation of the C-LEVEL-1 transform.

Normative document: ``ros_ws/docs/levelling_frame.md`` (contract **C-LEVEL-1**).
Read it before adding a caller: *which* poses may pass through here is the whole
point of the contract, and it is not inferable from these functions.

**What the correction is.** The Platform Teensy's inclinometer measures the
platform's residual tilt against gravity while it is commanded level; the
orchestrator publishes that measurement on ``/gravity_offset`` as
``[tilt_x, tilt_y]`` radians. The correction is the *inverse* of that measured
error, so that commanding "zero tilt" produces a platform that is level **with
respect to gravity** rather than level with respect to its own (imperfectly
mounted) frame.

**The two halves, and why they are separate functions.**

* :func:`correction_from_offset` owns the **sign convention**
  (``rotvec = [-tilt_x, -tilt_y, 0]``). It runs once per ``/gravity_offset``
  message, on the ROS executor thread.
* :func:`apply_gravity_correction` / :func:`correct_pose` own the
  **composition order** (``R_corrected = R_gravity @ R_target``). They run once
  per external pose, at ingest.

Both were duplicated verbatim in ``trajectory_node`` and ``mpc_bridge_node``
before 2026-07-25. Duplication is the drift hazard this module closes: the
composition is **not commutative**, so a re-derived copy that writes
``R_target @ R_gravity`` is a silent frame error — for a ``[0.15, -0.08, 0]``
target on the 2026-07-25 session offset the two orders differ by
``1.268e-3 rad`` (0.0727°), entirely in ``rz``, which is invisible in leg-space
telemetry.

**The pose-dependent extension (C-LEVEL-2).** :func:`correction_for_pose` adds
a residual tilt map on top of the single measured offset — same sign
convention, same single Rodrigues, one more *additive* term in the rotation
vector. It is a strict superset: with no map it is bit-identical to
:func:`correction_from_offset`. The map itself (parse, validate, bilinear
lookup) lives in :mod:`jugglebot.motion.tilt_map`.

Pure Python — no ROS imports (this module is consumed by ROS nodes, but also by
``tests/motion`` with no ROS mocking at all).
"""

from __future__ import annotations

import numpy as np

from jugglebot.motion.ik_solver import (
    rot_matrix_to_rotvec,
    rotvec_to_rot_matrix,
)
from jugglebot.motion.tilt_map import lookup as lookup_tilt_residual

__all__ = [
    'identity_correction',
    'correction_from_offset',
    'correction_for_pose',
    'apply_gravity_correction',
    'correct_pose',
]


def identity_correction() -> np.ndarray:
    """The "no correction loaded" value — a fresh 3×3 identity matrix.

    A function rather than a module constant deliberately: a shared mutable
    ``np.eye(3)`` at module scope would be one in-place write away from
    corrupting the levelling frame of every node in the process.
    """
    return np.eye(3)


def correction_from_offset(tilt_x: float, tilt_y: float) -> np.ndarray:
    """Measured tilt error ``[tilt_x, tilt_y]`` (rad) → the correction matrix ``R``.

    ``tilt_x`` / ``tilt_y`` are the tilt the platform *has* while commanded level
    (the ``/gravity_offset`` wire values). The correction is the inverse rotation
    — the angles are **negated** — so applying it to a level command counter-tilts
    the platform to true level.

    The ``rz`` component is always zero: levelling corrects the two tilt axes, and
    the platform's yaw has no gravity reference to correct against.
    """
    return rotvec_to_rot_matrix(
        np.array([-float(tilt_x), -float(tilt_y), 0.0]))


def correction_for_pose(offset, tilt_map, pose) -> np.ndarray:
    """Level offset + pose-dependent residual → the correction matrix ``R``.

    Contract **C-LEVEL-2** (``ros_ws/docs/levelling_frame.md``). The single
    entry point for the pose-dependent frame, and a strict superset of
    :func:`correction_from_offset`:

    * ``tilt_map is None`` ⇒ ``correction_from_offset(offset[0], offset[1])``,
      bit-identical. An absent or rejected map degrades to exactly C-LEVEL-1
      behaviour — C-LEVEL-2 is a refinement, never a gate.
    * otherwise ⇒ ``correction_from_offset(*(offset + residual))`` where
      ``residual`` is the bilinear map lookup at ``(pose[0], pose[1])``.

    **``pose`` is the UNCORRECTED intent pose**, positions in mm — the pose as
    it entered the node, *before* this correction rewrites its rotation.
    Keying on the corrected pose would make the lookup depend on its own
    output; the intent pose makes it a single evaluation with no fixed-point
    iteration. Only ``pose[0:2]`` is read (the grid is over x/y at one capture
    height); the rotation components are ignored, and the map is applied
    additively at *all* commanded orientations.

    **Composition is additive in the rotation vector, not two rotations.** The
    two tilts are summed and passed through the *one* existing Rodrigues, so the
    number of rotations composed onto a commanded pose stays exactly one. The
    error against the true ``R(map) @ R(level)`` composition is exactly
    ``|offset × residual| / 2`` — **7.19e-5 rad at the measured envelope**
    (0.78185° offset × 0.604° residual, orthogonal), but **1.523e-4 rad at
    1° × 1°**, so it is *not* a blanket sub-degree guarantee. The regime table
    is in ``ros_ws/docs/levelling_frame.md`` § C-LEVEL-2 and the sweep is
    ``tests/motion/test_tilt_map.py::test_additive_composition_second_order_bound_over_the_regime``.

    Raises ``tilt_map.TiltMapError`` if the pose's x/y are non-finite (a
    non-finite intent pose is an upstream defect; the map refuses to launder it
    into the commanded rotation).
    """
    tilt_x = float(offset[0])
    tilt_y = float(offset[1])
    if tilt_map is not None:
        residual_x, residual_y = lookup_tilt_residual(
            tilt_map, float(pose[0]), float(pose[1]))
        tilt_x += residual_x
        tilt_y += residual_y
    return correction_from_offset(tilt_x, tilt_y)


def apply_gravity_correction(rotvec, correction) -> np.ndarray:
    """Compose the correction into a target rotation vector.

    ``R_corrected = R_gravity @ R_target`` — the correction pre-multiplies, i.e.
    it is a **bias applied in the world/base frame** on top of whatever the
    request asked for. Returns the corrected rotation vector as a ``(3,)`` array;
    the input is never mutated.

    **Order matters and the reverse is a plausible typo.** ``R_target @
    R_gravity`` would apply the correction in the *target's* frame, which for a
    non-trivial target is a different rotation (see the module docstring for the
    measured magnitude). A unit test with an identity target cannot tell the two
    apart — pin the order with a non-identity target or the test proves nothing.
    """
    R_target = rotvec_to_rot_matrix(np.asarray(rotvec, dtype=float))
    R_corrected = np.asarray(correction, dtype=float) @ R_target
    return rot_matrix_to_rotvec(R_corrected)


def correct_pose(pose, correction) -> np.ndarray:
    """Apply the correction to a 6-DOF pose's **rotation only**.

    Returns a new ``[x, y, z, rx, ry, rz]`` array: ``[0:3]`` (position, mm) is
    copied through untouched and ``[3:6]`` (rotation vector, rad) is corrected.
    The input is never mutated — several callers pass a stored constant
    (``_neutral_pose``), and correcting it in place would double-apply on the
    next read.

    **Position is deliberately not touched.** The correction is a bias on the
    commanded *rotation*, not a re-expression of the platform frame, so rotating
    a position (or a linear velocity) through it would be a category error. This
    is also what keeps the catch reach-envelope check frame-consistent: both the
    envelope centre and the tested target are position-only 3-vectors, so this
    correction cannot move either of them.
    """
    out = np.array(pose, dtype=float)
    out[3:6] = apply_gravity_correction(out[3:6], correction)
    return out
