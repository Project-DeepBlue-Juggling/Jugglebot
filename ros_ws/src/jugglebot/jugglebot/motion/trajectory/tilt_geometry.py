"""Cup tilt geometry — the cup-axis-collinear receive/throw tilts + shared lever arm.

Port of ``Jugglebot-bb/sim/juggle_tilt.py`` core tilt math (the geometry that
was **sim-validated** on the bb branch; see the plan's "Prior art" note). Three
concerns live here:

  * :func:`tilt_to_receive` — the catch tilt ``(rx, ry)`` that aligns the cup's
    up-axis **anti-parallel** to the ball's observed arrival velocity ("Kai's
    collinear catch"), so the ball drops straight down the cup axis with *zero*
    lateral velocity in the cup frame (a parked non-collinear rim skids/deflects
    the ball — the failure this prevents). Bounded to :data:`MAX_TILT_DEG` (12°),
    the usable ceiling from the bb Rung-0 tilt-tracking characterisation.
  * :func:`tilt_to_throw` — the throw tilt (single-ball-toss Phase 4, bb
    Rung 2a): the exact mirror (``tilt_to_throw(v) == tilt_to_receive(-v)``),
    aligning the cup axis **along** the ballistic take-off velocity so the fast
    slider stroke ejects the ball at the aim. ``sim/juggle_tilt.py`` re-exports
    it from here (single source of truth since the 2026-07-24 branch merge).
  * :func:`cup_axis` — the world-frame cup up-axis for a given tilt, the inverse
    of :func:`tilt_to_receive` (used by the geometry-regression test to confirm
    the axis lands anti-parallel to the arrival velocity).

**The lever-arm compensation is NOT duplicated here.** Phase 4's ``shaping.py``
already ported ``juggle_tilt.py``'s cup-height-derived lever arm
(``CUP_TILT_CENTER_Z_MM = 744.3``, ``LEVER_ARM_MM_PER_DEG = 1.66``,
:func:`shaping.cup_lever_arm_mm` / :func:`shaping.cup_lateral_shift_mm`) as the
lean shaper's xy compensation. To keep a **single source of truth** for the cup
lever arm (the "protect the contracts" rule — two copies drift), this module
**re-exports** those symbols rather than redefining them; the Phase-6 geometry
regression test pins them here against the bb reference values (1.66 mm/deg at the
Rung-0 operating cup height, ≤12° tilt).

Sign convention (pinned at port time, matching ``shaping.py`` and the bb Rung-0
check): with a positive lever arm (cup above the tilt centre), ``+ry`` swings the
cup toward ``+x`` and ``+rx`` toward ``−y``; the reach planner cancels that swing
by offsetting the platform centroid by ``−cup_lateral_shift_mm`` so the tilted cup
opening lands on the target xy (``centroid = target − shift``).

Pure Python + numpy; the only import is the intra-package IK rotation helper
(``jugglebot.motion.ik_solver``) — no ROS2, no repo-root / ``controller`` imports.
"""

from __future__ import annotations

import numpy as np

from jugglebot.motion.ik_solver import rotvec_to_rot_matrix
# Single source of truth for the cup lever arm — ported in Phase 4 (shaping.py);
# re-exported so tilt_geometry presents the full juggle_tilt catch surface without
# a second, drift-prone copy of the constants.
from jugglebot.motion.trajectory.shaping import (  # noqa: F401  (re-export)
    CUP_TILT_CENTER_Z_MM,
    LEAN_CUP_Z_MM,
    LEVER_ARM_MM_PER_DEG,
    cup_lateral_shift_mm,
    cup_lever_arm_mm,
)

# Usable tilt ceiling. Leg headroom is fine well past this, but tilt tracking and
# the small-angle aim model both degrade past ~12° (bb Rung-0 characterisation),
# so the receive tilt saturates here rather than chasing a fast lateral arrival
# into a regime the platform can't track.
MAX_TILT_DEG = 12.0


def tilt_to_receive(v_arrival, max_tilt_deg: float = MAX_TILT_DEG):
    """Catch tilt ``(rx, ry)`` rad aligning the cup axis to receive ``v_arrival``.

    The cup's up-axis is set **anti-parallel** to the ball's arrival velocity, so
    the ball's velocity in the cup frame is purely axial (straight down the cup)
    with zero lateral component — the ball seats instead of skidding across the
    opening. The desired cup axis is ``a = −v_arrival / |v_arrival|`` (points
    up-and-toward the incoming ball, since a catch arrival is descending). The
    minimal rotation mapping world ``+z`` onto ``a`` is about ``z × a`` by the
    angle between them, with no twist (``rz = 0``); that rotation vector's x/y
    components are ``(rx, ry)``.

    The from-vertical tilt is clamped to ``max_tilt_deg``; past the clamp a fast
    lateral arrival is only partially nulled (residual in-cup skid). A straight-down
    arrival (or a zero-magnitude velocity) returns level ``(0, 0)``.

    ``v_arrival`` is direction-only (internally normalised), so any consistent
    velocity unit works.
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
    # Rotation axis = (z × a) normalised = (−a_y, a_x, 0) / |lat|.
    axis = np.array([-lat[1], lat[0]]) / lat_norm
    rx = float(angle * axis[0])
    ry = float(angle * axis[1])
    return rx, ry


def tilt_to_throw(v_takeoff, max_tilt_deg: float = MAX_TILT_DEG):
    """Throw tilt ``(rx, ry)`` rad aligning the cup axis ALONG ``v_takeoff``.

    The tilt-aimed throw (bb Rung 2a): the ball detaches up the cup's symmetry
    axis, so the cup axis must point PARALLEL to the ballistic take-off velocity
    (ascending, vz > 0); the lateral take-off is delivered by the fast slider
    projected through the tilt — lateral = |v_takeoff|·sin θ — never by
    band-limited platform translation. Exact mirror of :func:`tilt_to_receive`:
    ``tilt_to_throw(v) == tilt_to_receive(-v)``. Direction-only (unit-agnostic).
    Past the clamp the aim SATURATES (landing bias, Rung 2a) — production
    Tier-8b callers must gate the from-vertical angle and reject loudly
    instead of relying on the clamp (see ``toss_release``; the silent clamp is
    the correct semantics only for the catch).
    """
    v = np.asarray(v_takeoff, dtype=float).reshape(3)
    return tilt_to_receive(-v, max_tilt_deg=max_tilt_deg)


def cup_axis(rx: float, ry: float) -> np.ndarray:
    """World-frame cup up-axis (unit 3-vector) for tilt ``(rx, ry)`` rad, ``rz=0``.

    The inverse of :func:`tilt_to_receive`: ``cup_axis(*tilt_to_receive(v))`` is
    anti-parallel to ``v`` (up to the ``max_tilt_deg`` clamp). Uses the production
    IK rotation helper so it matches the pose the emitter/gate actually realise.
    """
    rot = rotvec_to_rot_matrix(np.array([float(rx), float(ry), 0.0], dtype=float))
    return rot @ np.array([0.0, 0.0, 1.0])


def tilt_to_receive_deg(v_arrival, max_tilt_deg: float = MAX_TILT_DEG) -> float:
    """Magnitude of the receive tilt (degrees) for ``v_arrival`` — the scalar the
    reach planner uses to decide feasibility headroom and the harness reports."""
    rx, ry = tilt_to_receive(v_arrival, max_tilt_deg=max_tilt_deg)
    return float(np.degrees(np.hypot(rx, ry)))
