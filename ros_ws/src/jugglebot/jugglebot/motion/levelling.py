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

Pure Python — no ROS imports (this module is consumed by ROS nodes, but also by
``tests/motion`` with no ROS mocking at all).
"""

from __future__ import annotations

import numpy as np

from jugglebot.motion.ik_solver import (
    rot_matrix_to_rotvec,
    rotvec_to_rot_matrix,
)

__all__ = [
    'identity_correction',
    'correction_from_offset',
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
