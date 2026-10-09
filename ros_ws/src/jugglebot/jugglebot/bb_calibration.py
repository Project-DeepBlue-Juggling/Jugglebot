"""
Ball Butler calibration: determine BB global position and yaw offset from mocap markers.

During BB's CALIBRATING state, the 7 BB mocap markers (:data:`BB_MARKER_COUNT`)
trace circular arcs as the yaw motor sweeps.  After the sweep completes, this module:

0. Refuses outright if the sweep never happened (``MIN_ARC_DEG``) — point count
   alone cannot tell a truncated sweep from a completed one. The primary test is
   on BB's own reported yaw series (encoder-derived, so noise-immune); the
   per-marker geometry carries a secondary, lower floor. See
   :data:`MIN_ARC_DEG` and :data:`MIN_MARKER_ARC_DEG`.
1. Fits a 3D circle to each marker trajectory (plane fit via SVD + algebraic circle fit).
2. Extracts the common rotation axis (weighted average of circle normals/centers)
   from the largest subset of markers that agree on it — at least
   :data:`MIN_AGREEING_MARKERS`; a marker left out is reported as an outcast
   and the calibration still stands (:data:`MAX_AXIS_DEVIATION_MM`).
3. Intersects the axis with the horizontal plane at the average Z height of the
   CO-PLANAR markers only (:data:`BB_PLANE_MARKER_INDICES`, QTM 3–7), offset by
   the pitch-axis vertical offset, to get the BB global position.
4. Reads yaw_offset_rad — and, since 2026-10-09, the position — from the
   WHOLE SWEEP: every mocap frame is fitted as a rigid body against a stored
   BB body template, the reported yaw's lag behind the frames is fitted per
   sweep, and a fixed yaw-dependent term E(y) is removed
   (:func:`estimate_sweep_yaw_offset`). The arc fit of steps 1–3 stays as the
   tilt, the sweep gates and the position cross-check. The retired estimator
   — one anchor marker's angle about the fitted axis point
   (:data:`BB_YAW_ANCHOR_INDEX`, :func:`calculate_yaw_offset`) — still runs
   when no template is given (sweep-fit fixtures) and as a logged diagnostic.

All functions are pure Python + numpy — no ROS2 dependency.

Lifted from the retired ball_butler_node.py (now attic/ros-jugglebot-archived/)
with minor cleanup.
"""

from __future__ import annotations

import itertools
import math
import numpy as np
from dataclasses import dataclass, field
from typing import Optional


#: Number of BB fiducials, labelled ``Ball Butler - 1`` .. ``Ball Butler - 7`` in
#: QTM. Every array/dict that carries BB markers is indexed ZERO-based, so QTM
#: label ``N`` is index ``N - 1`` throughout. Was 5 until 2026-09-27, when two
#: markers were added (QTM 1 and 2) to keep the constellation tracked through
#: the calibration sweep; the original five were relabelled 3–7.
BB_MARKER_COUNT = 7

#: Zero-based indices of the markers that lie on ONE plane (QTM 3–7). Only these
#: set the Z height of BB's x/y plane. QTM 1 and 2 sit at a different height, so
#: folding them into the Z average would shift the plane — and with it the
#: axis/plane intersection that IS the reported BB position (by the axis tilt ×
#: the Z error laterally, and the full Z error vertically). They still feed the
#: rotation-axis fit, where a marker's height does not matter.
BB_PLANE_MARKER_INDICES = (2, 3, 4, 5, 6)

#: Zero-based index of the marker whose global angle anchors the yaw offset —
#: QTM ``Ball Butler - 4`` since the 2026-09-27 relabelling (owner). It must be
#: the same PHYSICAL marker from one calibration to the next that the yaw-zero
#: convention was set against: anchoring on a different marker shifts
#: yaw_offset_rad by the angle between the two, and every throw is misaimed by
#: that much with no error raised.
BB_YAW_ANCHOR_INDEX = 3

#: Minimum yaw arc (degrees) the sweep must actually cover before a fit is
#: allowed to produce a BB pose.
#:
#: WHY A FLOOR AT ALL. ``find_rotation_axis`` only ever asked for *enough
#: points* (``min_points=50``). At the 200 Hz marker rate that is 0.25 s of
#: data, so a sweep truncated by a mid-sweep QTM dropout could still hand the
#: algebraic circle fit a stubby arc — and a short arc fits a circle with a
#: TINY residual and a wildly wrong centre and radius. The result is not a
#: loud failure but a plausible BB position that ``ball_butler_node`` then aims
#: every throw with. Point count cannot distinguish "swept slowly" from "barely
#: moved"; arc span can.
#:
#: ⚠ WHICH SIGNAL THE FLOOR IS MEASURED ON — the load-bearing detail.
#: It is measured on **BB's reported yaw series**, not on the marker geometry.
#: The original marker-only construction was defeated by ordinary QTM noise,
#: because ``arc_span_deg`` measures the angle subtended at the *fitted* centre
#: and a stubby arc's fitted centre is precisely the ill-conditioned quantity
#: the floor exists to distrust. Probe, ``/tmp/probe_arc_noise.py``, run
#: 2026-08-22 on the project venv's pinned numpy, a true 110 mm-radius arc,
#: 200 samples:
#:
#: ====== ========= =========== ===========
#: sweep  noise mm  fitted r    span read
#: ====== ========= =========== ===========
#: 5°     0.00      110.00 mm   5.0°
#: 5°     0.10        9.59 mm   56.3°
#: 5°     0.50        2.86 mm   327.6°
#: 5°     1.50        3.48 mm   336.6°
#: 180°   0.50      109.98 mm   180.2°
#: ====== ========= =========== ===========
#:
#: QTM marker noise is 0.1–1.5 mm in practice, so on HARDWARE the marker-only
#: floor would have fired on nothing: the radius collapses ~110 mm → ~2 mm and
#: the noise ball is read as a full circle. Worse, the per-marker exclusion
#: INVERTED — the noisiest stubby marker read the widest span, so it was kept
#: and its garbage centre poisoned the weighted average, while the truncated
#: sweep died later in the ``max_dev > 3.0`` check under an occlusion message
#: that points the operator at the wrong subsystem.
#:
#: The yaw series has no such failure: it is encoder-derived, arrives already
#: as a 1-D angle, and needs no centre to be fitted, so a 5° sweep reads 5.0°
#: at any marker noise. It is therefore the PRIMARY gate
#: (:func:`angular_span_deg`, applied in :func:`run_calibration`). The
#: per-marker span check remains as a secondary under its own, lower floor
#: (:data:`MIN_MARKER_ARC_DEG`), alongside :data:`MIN_MARKER_RADIUS_MM` which
#: catches the collapse signature directly.
#:
#: WHY 60° (owner, 2026-08-25; was 20°): a real calibrate swept 118.8°, and
#: the Phase-A probe showed spans ≤25° are defeated by real marker noise, so
#: 60° sits ~2× under real operation and well clear of the noise regime. This
#: is the SWEEP gate only — the per-marker inclusion floor stayed at 20°.
MIN_ARC_DEG = 60.0

#: Per-marker arc floor (degrees) for INCLUSION in the axis average
#: (:func:`find_rotation_axis`). A marker's FITTED arc shrinks under occlusion
#: during a sweep that legally cleared the primary gate, so this floor
#: deliberately did NOT move with the 2026-08-25 raise of :data:`MIN_ARC_DEG`.
MIN_MARKER_ARC_DEG = 20.0

#: Minimum fitted circle radius (mm) for a marker to be folded into the axis
#: average. The BB constellation sits 75–110 mm off the yaw axis, so a fit that
#: reports single-digit millimetres has not measured the constellation — it has
#: measured the noise ball of a marker that barely moved (see the table above:
#: the collapse is 110 mm → ~2 mm, which is not a near-miss but a different
#: order of magnitude). 20 mm is a quarter of the smallest true radius: far
#: below anything a real marker can fit to, far above the collapse.
#:
#: This is the check that survives noise. The per-marker ``arc_span_deg`` test
#: it sits beside is the one noise defeats, so on hardware this is the floor
#: doing the work; both are kept because they name different causes and a
#: marker CAN be genuinely stubby without its radius collapsing (a clean,
#: low-noise partial occlusion — which is what the synthetic fixtures model).
MIN_MARKER_RADIUS_MM = 20.0

#: How far (mm) a marker's fitted circle centre may sit from the rotation axis
#: the consensus markers agree on. Unchanged from the original hard-coded 3.0,
#: and deliberately NOT tightened: on the 14 recorded calibrations of
#: 2026-10-05/06 the healthy markers, with the one outcast removed, still
#: spread 1.29–2.30 mm, because ~1 mm of pose-dependent QTM error is levered up
#: by fitting ~80–120° arcs. A 2.0 mm gate refused 9 of those 14.
MAX_AXIS_DEVIATION_MM = 3.0

#: Fewest markers that may out-vote an outcast (:func:`find_rotation_axis`).
#: Of the 7, up to 2 can be excluded; with 5 or fewer fitted (occlusion) there
#: is no majority to vote against one, and every fitted marker must agree.
#:
#: WHY (2026-10-06 sitting): all 7 rejections that night, and 4 of 5 the night
#: before, were ONE marker — QTM 1, whose readings at the parked pose crept
#: ~3 mm relative to the rest of the constellation over the first minutes of a
#: session — taking the whole calibration down while the other six agreed
#: within 2.3 mm. Fitted without it, BB's position agreed within 0.8 mm across
#: all 9 attempts, so the refusals guarded nothing. The consensus keeps the
#: gate strict for the markers it trusts and names the one it did not.
MIN_AGREEING_MARKERS = 5

#: Minimum yaw readings before the yaw-span gate is meaningful — the same count
#: :func:`calculate_yaw_offset` already demands. Below it, ``run_calibration``
#: always fails at the yaw-offset step anyway, so declining to evaluate the span
#: opens no hole; it just keeps the reported cause the accurate one ("not enough
#: yaw readings", not "the sweep did not complete").
MIN_YAW_READINGS = 5


def wrap_pi(angle: float) -> float:
    """Wrap angle to (-pi, pi]."""
    a = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return a if a != -math.pi else math.pi


def angular_span_deg(angles_deg) -> float:
    """Angular extent (degrees, 0–360) covered by a 1-D series of angles.

    Same ``360° − largest gap`` construction as :func:`arc_span_deg`, and for
    the same branch-cut reason — a sweep through 0°/360° must not read as a
    full circle — but applied to angles that ARE the measurement rather than
    angles derived from a fitted centre.

    That distinction is the whole point. :func:`arc_span_deg` has to fit a
    circle first, and on a truncated sweep that fit is exactly the
    ill-conditioned quantity being distrusted: at 0.5 mm of marker noise a real
    5° arc fits a ~3 mm circle whose subtended angle reads 328°. BB's reported
    yaw is encoder-derived and already 1-D, so nothing is fitted and a 5° sweep
    reads 5.0° regardless of marker noise. See :data:`MIN_ARC_DEG`.

    Fails CLOSED (returns 0.0) on fewer than two samples or any non-finite
    value: a series that cannot be measured must not pass a floor.
    """
    a = np.asarray(angles_deg, dtype=float).ravel()
    if a.size < 2 or not np.all(np.isfinite(a)):
        return 0.0
    # Wrap into [0, 2pi) so the sort below is a genuine circular ordering; raw
    # yaw readings may run negative or past 360 without ever leaving the arc.
    a = np.sort(np.radians(a) % (2.0 * np.pi))
    gaps = np.diff(a)
    wrap_gap = (a[0] + 2.0 * np.pi) - a[-1]
    largest_gap = float(max(gaps.max() if gaps.size else 0.0, wrap_gap))
    return float(np.degrees(2.0 * np.pi - largest_gap))


def canonical_yaw_deg(yaw_deg) -> np.ndarray:
    """BB's reported yaw series on its principal branch, continuous: each
    sample wrapped to [-180, 180), then unwrapped from the first.

    The heartbeat's ``yaw_deg`` is wrapped to [0, 360) on the wire, so BB
    parked at -0.4° arrives as 359.6°. A bare ``np.unwrap`` keeps the branch
    of the FIRST sample, so a sweep whose first sample sat just below zero
    was read on the +360 branch (359.7, 360.1, ... 484, ...), every sample
    fell outside E's valid range (-5...130°), and the sweep failed with
    ``CONSTELLATION_TOO_FEW_MOVING: 0 moving heartbeat yaw samples`` -- 5 of
    the 7 sweeps of bag ``2026-10-09_23-49-07`` (first recorded yaw 357.72 /
    359.98 / 359.96 / 359.61 / 359.63 failed; 0.11 / 0.09 passed; logbook
    2026-10-09-bb-calibration-heartbeat-yaw-wrap). BB's physical yaw range
    (about -5...130°) never approaches ±180°, so the principal branch is the
    right one for every sample. The stamped ``bb_yaw`` (already unwrapped by
    the firmware, may be negative) passes through unchanged.
    """
    y = np.asarray(yaw_deg, dtype=float)
    return np.degrees(np.unwrap(np.radians((y + 180.0) % 360.0 - 180.0)))


# ---------------------------------------------------------------------------
#  3D circle fitting
# ---------------------------------------------------------------------------

def fit_plane(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Fit a plane to 3D points via SVD.  Returns (centroid, unit normal)."""
    centroid = np.mean(points, axis=0)
    centered = points - centroid
    _, _, Vt = np.linalg.svd(centered)
    normal = Vt[-1]
    return centroid, normal


def plane_basis(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Orthonormal (u, v) spanning the plane with the given unit *normal*."""
    ref = np.array([1.0, 0.0, 0.0]) if abs(normal[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(normal, ref)
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)
    v /= np.linalg.norm(v)
    return u, v


def arc_span_deg(points: np.ndarray, center: np.ndarray, normal: np.ndarray) -> float:
    """Angular extent (degrees, 0–360) the *points* cover around *center*.

    Computed as ``360° − largest angular gap``, the standard circular-range
    construction. Plain ``max(angle) − min(angle)`` is wrong here: a sweep that
    crosses the ±π branch cut of ``atan2`` reads as ~360° when it is really a
    few degrees, which would defeat the floor in exactly the case it exists
    for. The gap form has no branch cut, returns ~360° for a full circle, and
    correctly returns ~5° for a 5° arc no matter where that arc sits.
    """
    if points.shape[0] < 2 or not np.all(np.isfinite(normal)) or not np.all(np.isfinite(center)):
        # Fail CLOSED: a degenerate fit (all-identical points give a NaN plane
        # normal) has no meaningful span, and reporting 0° excludes it rather
        # than letting a NaN comparison silently pass the floor.
        return 0.0
    u, v = plane_basis(normal)
    rel = points - center
    angles = np.sort(np.arctan2(rel @ v, rel @ u))
    if not np.all(np.isfinite(angles)):
        return 0.0
    gaps = np.diff(angles)
    # Close the circle: the wrap-around gap from the last sample back to the first.
    wrap_gap = (angles[0] + 2.0 * np.pi) - angles[-1]
    largest_gap = float(max(gaps.max() if gaps.size else 0.0, wrap_gap))
    return float(np.degrees(2.0 * np.pi - largest_gap))


def fit_circle_3d(points: np.ndarray) -> tuple[np.ndarray, float, np.ndarray, float]:
    """Fit a circle to 3D points lying approximately on a plane.

    Returns (center_3d, radius, normal, rms_residual_mm).
    """
    centroid, normal = fit_plane(points)

    # Local 2D basis on the fitted plane
    u, v = plane_basis(normal)

    # Project to 2D
    centered = points - centroid
    x = np.dot(centered, u)
    y = np.dot(centered, v)

    # Algebraic circle fit: solve  [x, y, 1] · [a, b, c]^T = x² + y²
    A = np.column_stack([x, y, np.ones(len(x))])
    b_vec = x ** 2 + y ** 2
    result, _, _, _ = np.linalg.lstsq(A, b_vec, rcond=None)

    cx2d = result[0] / 2
    cy2d = result[1] / 2
    r_sq = result[2] + cx2d ** 2 + cy2d ** 2
    radius = float(np.sqrt(max(r_sq, 0.0)))

    center_3d = centroid + cx2d * u + cy2d * v

    # RMS residual
    dists = np.sqrt((x - cx2d) ** 2 + (y - cy2d) ** 2)
    residual = float(np.sqrt(np.mean((dists - radius) ** 2)))

    return center_3d, radius, normal, residual


# ---------------------------------------------------------------------------
#  Rotation axis from multiple marker trajectories
# ---------------------------------------------------------------------------

@dataclass
class MarkerFitMetrics:
    """Quality metrics for one marker's circle fit."""
    status: str  # 'ok' or 'skipped'
    reason: str = ''
    n_points: int = 0
    radius_mm: float = 0.0
    center: Optional[np.ndarray] = None
    normal: Optional[np.ndarray] = None
    fit_residual_mm: float = 0.0
    distance_from_axis_mm: float = 0.0
    arc_span_deg: float = 0.0


def find_rotation_axis(
    marker_trajectories: dict[int, np.ndarray],
    min_points: int = 50,
    min_marker_arc_deg: float = MIN_MARKER_ARC_DEG,
    min_radius_mm: float = MIN_MARKER_RADIUS_MM,
    max_axis_dev_mm: float = MAX_AXIS_DEVIATION_MM,
    min_agreeing: int = MIN_AGREEING_MARKERS,
) -> tuple[np.ndarray, np.ndarray, dict[int, MarkerFitMetrics]]:
    """Find the rotation axis from multiple marker circular trajectories.

    Args:
        marker_trajectories: marker_index → (N, 3) positions array.
        min_points: minimum samples per marker for inclusion.
        min_marker_arc_deg: minimum arc a marker must trace to be fitted at
            all, and — via the widest marker — for the sweep to count as
            having happened on this (direct-caller) path. This is the
            per-marker floor :data:`MIN_MARKER_ARC_DEG`, NOT the sweep gate
            :data:`MIN_ARC_DEG`: an occluded marker in an otherwise legal
            sweep fits a short arc, and must be excluded, not held to the
            sweep's floor.
        min_radius_mm: minimum fitted circle radius for a marker to be folded
            into the axis average (:data:`MIN_MARKER_RADIUS_MM`) — the
            noise-proof half of the pair, since a collapsed fit is what
            *causes* the span to read wide.
        max_axis_dev_mm: how far a circle centre may sit from the consensus
            axis (:data:`MAX_AXIS_DEVIATION_MM`).
        min_agreeing: fewest markers that may out-vote an outcast
            (:data:`MIN_AGREEING_MARKERS`).

    Returns:
        axis_point:   a point on the axis (mm), from the consensus markers.
        axis_direction: unit vector along the axis, ditto.
        quality_metrics: per-marker fit metrics; an excluded marker carries
            status ``'outcast'`` and its distance from the consensus axis.

    Raises:
        ValueError on insufficient data, too small an arc, or poor geometric
        consistency.
    """
    centers: list[np.ndarray] = []
    normals: list[np.ndarray] = []
    weights: list[float] = []
    quality: dict[int, MarkerFitMetrics] = {}
    widest_arc_deg = 0.0
    n_radius_collapsed = 0

    for idx, pts in marker_trajectories.items():
        if len(pts) < min_points:
            quality[idx] = MarkerFitMetrics(
                status='skipped',
                reason=f'insufficient points ({len(pts)} < {min_points})',
            )
            continue

        center, radius, normal, residual = fit_circle_3d(pts)
        span = arc_span_deg(np.asarray(pts), center, normal)
        widest_arc_deg = max(widest_arc_deg, span)

        # Consistent normal direction
        if normals and np.dot(normal, normals[0]) < 0:
            normal = -normal

        # Individual exclusion as well as the aggregate refusal below: a
        # single marker that was occluded for most of the sweep (the rest of
        # the constellation fine) contributes a garbage centre/normal that the
        # length-weighted average would happily fold in. Dropping it is the
        # same treatment a marker with too few points already got — and if that
        # leaves fewer than two, the existing "insufficient data" refusal fires.
        #
        # TWO independent tests, because they fail on different data. The span
        # test catches a CLEAN partial occlusion. The radius test catches the
        # noisy one — and on hardware that is the common case, because a stubby
        # arc plus 0.1–1.5 mm of QTM noise collapses the fitted radius from
        # ~110 mm to ~2 mm and inflates the span it subtends to near 360°, so
        # the span test alone does not merely miss the bad marker, it PREFERS
        # it (see :data:`MIN_ARC_DEG` for the measured table). The reason names
        # whichever tripped: 'radius 2.9 mm' next to a constellation that is
        # never nearer than 75 mm is an unmistakable signature at the bench.
        skip_reason = ''
        if radius < min_radius_mm:
            skip_reason = (f'fitted radius {radius:.1f} mm < '
                           f'{min_radius_mm:.1f} mm (collapsed fit — the '
                           f'marker barely moved)')
        elif span < min_marker_arc_deg:
            skip_reason = f'arc span {span:.1f}° < {min_marker_arc_deg:.1f}°'
        if skip_reason:
            if radius < min_radius_mm:
                n_radius_collapsed += 1
            quality[idx] = MarkerFitMetrics(
                status='skipped',
                reason=skip_reason,
                n_points=len(pts),
                radius_mm=radius,
                fit_residual_mm=residual,
                arc_span_deg=span,
            )
            continue

        centers.append(center)
        normals.append(normal)
        weights.append(len(pts) / (residual + 0.1))

        quality[idx] = MarkerFitMetrics(
            status='ok',
            n_points=len(pts),
            radius_mm=radius,
            center=center,
            normal=normal,
            fit_residual_mm=residual,
            arc_span_deg=span,
        )

    # Aggregate refusal FIRST, and with its own code: when the sweep never
    # happened (mid-sweep QTM dropout, a wedged yaw motor) every marker is
    # short, they all get excluded above, and the generic "only 0 markers had
    # sufficient data" message would send the operator hunting for occlusion
    # that isn't there. Naming ARC_SPAN_TOO_SMALL points at the sweep instead.
    if widest_arc_deg < min_marker_arc_deg and any(
            len(pts) >= min_points for pts in marker_trajectories.values()):
        raise ValueError(
            f'ARC_SPAN_TOO_SMALL: widest marker arc {widest_arc_deg:.1f}° < '
            f'{min_marker_arc_deg:.1f}° — the yaw sweep did not complete, so '
            'the circle fits are not trustworthy'
        )

    if len(centers) < 2:
        # Name the collapse when it is the cause. Under marker noise a truncated
        # sweep reaches here rather than the span refusal above (every marker
        # skipped on radius, so widest_arc_deg is a large and meaningless
        # number), and a bare "only 0 markers had sufficient data" sends the
        # operator hunting for occlusion again. run_calibration's yaw gate
        # normally catches this case first; this covers the direct callers and
        # the no-yaw-readings path.
        detail = ''
        if n_radius_collapsed:
            detail = (f' ({n_radius_collapsed} excluded for a collapsed circle '
                      f'fit — consistent with a sweep that never happened)')
        raise ValueError(
            f'Calibration failed: only {len(centers)} markers had '
            f'sufficient data{detail}')

    fitted = [idx for idx, m in quality.items() if m.status == 'ok']
    weight_of = dict(zip(fitted, weights))

    def axis_of(subset):
        w = np.array([weight_of[i] for i in subset])
        w /= w.sum()
        n = (np.array([quality[i].normal for i in subset]) * w[:, np.newaxis]).sum(axis=0)
        n /= np.linalg.norm(n)
        p = (np.array([quality[i].center for i in subset]) * w[:, np.newaxis]).sum(axis=0)
        return p, n

    def deviations(point, direction, markers):
        out = {}
        for i in markers:
            v = quality[i].center - point
            out[i] = float(np.linalg.norm(v - np.dot(v, direction) * direction))
        return out

    # Consensus (see MAX_AXIS_DEVIATION_MM / MIN_AGREEING_MARKERS): the largest
    # subset whose centres all lie within tolerance of their OWN axis wins —
    # all markers first, then one fewer, never below MIN_AGREEING_MARKERS. A
    # marker left out is an outcast: excluded from the axis, reported, and
    # the calibration stands. With too few markers to out-vote one, the
    # subset is the full set, i.e. the pre-consensus rule.
    smallest = min_agreeing if len(fitted) > min_agreeing else len(fitted)
    best = None
    for size in range(len(fitted), smallest - 1, -1):
        for subset in itertools.combinations(fitted, size):
            point, direction = axis_of(subset)
            devs = deviations(point, direction, subset)
            worst = max(devs.values())
            if best is None or worst < best[0]:
                best = (worst, subset, point, direction, devs)
        if best[0] <= max_axis_dev_mm:
            break
        if size > smallest:
            best = None

    max_dev, consensus, axis_point, axis_direction, _ = best
    all_devs = deviations(axis_point, axis_direction, fitted)
    for i in fitted:
        quality[i].distance_from_axis_mm = all_devs[i]

    if max_dev > max_axis_dev_mm:
        # Name every marker's deviation from the full-set axis: the failure
        # line is then the per-marker breakdown, which the 2026-10-06 sitting
        # had to dig out of a bag because only successes logged it.
        point, direction = axis_of(fitted)
        full = deviations(point, direction, fitted)
        detail = ', '.join(f'Marker {i + 1} {full[i]:.2f}' for i in sorted(full))
        raise ValueError(
            f'Circle centres deviate up to {max(full.values()):.2f} mm from axis '
            f'({detail} mm; no {smallest} of {len(fitted)} within '
            f'{max_axis_dev_mm:.1f} mm) — non-rigid motion or poor marker visibility'
        )

    n_agree = len(consensus)
    for i in fitted:
        if i not in consensus:
            m = quality[i]
            m.status = 'outcast'
            m.reason = (f'circle centre {m.distance_from_axis_mm:.2f} mm from the '
                        f'axis the other {n_agree} markers agree on '
                        f'(> {max_axis_dev_mm:.1f} mm) — excluded')

    return axis_point, axis_direction, quality


# ---------------------------------------------------------------------------
#  Axis / plane intersection
# ---------------------------------------------------------------------------

def find_axis_plane_intersection(
    axis_point: np.ndarray,
    axis_direction: np.ndarray,
    plane_z: float,
) -> Optional[np.ndarray]:
    """Where the axis intersects a horizontal plane at *plane_z*.  None if parallel."""
    if abs(axis_direction[2]) < 1e-10:
        return None
    t = (plane_z - axis_point[2]) / axis_direction[2]
    return axis_point + t * axis_direction


# ---------------------------------------------------------------------------
#  Yaw offset
# ---------------------------------------------------------------------------

def calculate_yaw_offset(
    anchor_positions: np.ndarray,
    yaw_readings_deg: list[float],
    axis_point: np.ndarray,
) -> tuple[float, float]:
    """Compute the yaw offset between BB's local frame and the global frame.

    Uses the final ~1 s of data (when BB is stationary after calibration sweep).

    Returns (yaw_offset_rad, combined_std_deg).
    Raises ValueError on insufficient data.
    """
    if len(anchor_positions) < 50:
        raise ValueError(f'Insufficient yaw-anchor marker data: {len(anchor_positions)} points')
    if len(yaw_readings_deg) < MIN_YAW_READINGS:
        raise ValueError(f'Insufficient yaw readings: {len(yaw_readings_deg)}')

    # Last ~1 s of marker data (200 Hz → 200 pts)
    recent_m3 = anchor_positions[-min(200, len(anchor_positions)):]
    # Principal branch: a parked hold straddling 0/360 on the wire (359.9, 0.1)
    # would otherwise average to ~180 (see canonical_yaw_deg).
    recent_yaw = [float(v) for v in
                  canonical_yaw_deg(yaw_readings_deg[-min(10, len(yaw_readings_deg)):])]

    # Yaw-anchor marker angle in global frame (circular mean)
    angles = [math.atan2(p[1] - axis_point[1], p[0] - axis_point[0]) for p in recent_m3]
    sin_sum = sum(math.sin(a) for a in angles)
    cos_sum = sum(math.cos(a) for a in angles)
    avg_marker_angle = math.atan2(sin_sum, cos_sum)

    avg_yaw_rad = math.radians(sum(recent_yaw) / len(recent_yaw))

    yaw_offset_rad = wrap_pi(avg_marker_angle - avg_yaw_rad)

    # Uncertainty. The deviations are WRAPPED, because avg_marker_angle above is
    # a CIRCULAR mean and a raw subtraction is not consistent with it: when the
    # hold pose sits near the ±π branch cut of atan2, ordinary marker noise sends
    # individual samples to +π and −π, and the unwrapped differences are ~2π
    # apart. Measured on the real geometry (BB at (-707, -149), Marker 3 pointing
    # directly away from the origin, 0.1 mm noise, probe 2026-08-22): the
    # unwrapped form reported ±242.8° for a spread whose true value is ±0.053°,
    # which the max_yaw_std_deg=5° check then rejected as "uncertainty too high".
    # That is a REAL rejection of a perfectly good calibration, not a synthetic
    # artefact — it depends only on where BB happens to sit in the global frame.
    # Away from the branch cut the two forms are identical, so this strictly
    # removes false rejections and weakens nothing.
    marker_var = sum(wrap_pi(a - avg_marker_angle) ** 2 for a in angles) / len(angles)
    marker_std_deg = math.degrees(math.sqrt(marker_var))

    avg_yaw_deg = sum(recent_yaw) / len(recent_yaw)
    yaw_var = sum((y - avg_yaw_deg) ** 2 for y in recent_yaw) / len(recent_yaw)
    yaw_std_deg = math.sqrt(yaw_var)

    combined_std_deg = math.sqrt(marker_std_deg ** 2 + yaw_std_deg ** 2)

    return yaw_offset_rad, combined_std_deg


# ---------------------------------------------------------------------------
#  Yaw offset from the whole sweep: rigid constellation + fitted latency
#  (2026-10-09, second design)
# ---------------------------------------------------------------------------
#
# WHY NOT THE ANCHOR. ``calculate_yaw_offset`` reads ONE marker's angle about
# the axis point the sweep fitted. That marker sits 117.6 mm from the axis, so
# the offset moves 0.487° per mm of axis-point error, and the sweep's axis
# point is good to ~1–2 mm at best. Two calibrations of an untouched BB stored
# 0.208° and 0.681° on 2026-10-09 (investigation:
# ~/bb_calibration_sessions/yaw_offset_investigation_20261009/REPORT.md).
#
# WHAT THIS DOES. Every mocap frame of the calibration window is fitted, as a
# rigid body, against a stored BB body template (all seven yaw-stage markers,
# matched by geometry, not by QTM label), so each frame gives BB's world pose:
# the yaw θ(t) of the template's x axis and the world position of the
# template origin, which IS the yaw axis point. The reported yaw y(t) (the
# heartbeat, unstamped, or a stamped 100 Hz stream when one exists) lags the
# frames by a per-session constant τ (80–85 ms in the 7-sweep bag, 94–97 ms in
# session A; in session B ~75 ms, with 66–83 % of the samples one 0.1 s
# republish period older — the earlier reading of B as "150→174 ms drifting"
# was that mixture; see :func:`fit_yaw_latency`). Three free-running 10 Hz
# stages sit between the encoder and the heartbeat's receive time. τ is FITTED per
# sweep, never assumed: over the moving samples (|dy/dt| > 5°/s), the τ that
# minimises the scatter of θ(t_y − τ) − y − E(y) about its mean. A lag error
# shows as ±ω·δτ with opposite signs on the outbound and return legs, so the
# minimum is sharp. The offset is the mean at the best τ.
#
# E(y), physical minus reported yaw beyond the reported zero (−0.09° at 10°,
# −0.44° at 70°, −1.57° at 125°; fitted once on the 7-sweep bag and stored in
# the template), is held FIXED inside the estimator so that the offset does
# not depend on which yaw range a window covers. It is an estimator term
# only: moving and stationary relations differ by a few tenths of a degree,
# so E is NOT applied to aim.
#
# The stationary pause is not used for yaw: on the 7-sweep bag the parked
# pose reconstructed non-rigidly (up to 2.5 mm), and one pose cannot beat the
# ~0.1–0.13° pose-to-pose scatter anyway. Per-sweep repeatability of this
# estimator on those 7 sweeps is in the template's ``sigma`` block and the
# logbook entry ``2026-10-09-bb-constellation-yaw-offset``.
#
# GAUGE. The raw offset φ_raw is the angle of the template's x axis minus the
# reported yaw. The published offset is
#     φ_raw − gauge.raw_offset_at_pin_deg + gauge.pinned_yaw_offset_deg
# where ``raw_offset_at_pin_deg`` is what this estimator reads on session A
# (the frame ``throw_affine_correction.json`` was fitted in) and
# ``pinned_yaw_offset_deg`` is A's stored 0.208°. A landing-based correction δ
# is applied by editing ``pinned_yaw_offset_deg`` alone (0.208 → 0.208 + δ).

#: Fewest template markers a frame must match to give a pose. Three is the
#: fewest for which a rigid fit has a residual at all.
CONSTELLATION_MIN_MATCHED = 3

#: A frame point within this distance (mm, 3D) of a posed template marker IS
#: that marker. The closest template markers are ~30 mm apart (the off-plane
#: QTM 1/2 to their neighbours) and frame-to-frame motion at 90°/s and 200 Hz
#: is ~1.2 mm at the outer markers, so a tracked pose from the previous frame
#: assigns correctly with margin.
CONSTELLATION_MATCH_TOL_MM = 3.0

#: Height-difference tolerance (mm) of the label-free initial match only. The
#: proposal stage rotates about world z, but the yaw axis is tilted ~1–1.5°
#: from z, which changes a pair's height difference by up to sep·sin(tilt)
#: (≈ 3 mm at 120 mm); the 3D refit that follows is exact.
CONSTELLATION_INIT_DZ_TOL_MM = 6.0

#: A frame whose rigid-fit RMS exceeds this (mm) gives no pose and is counted
#: as rejected (a reflection, a swapped point). On the 2026-10-09 bags
#: accepted frames have a median RMS of ~0.2–0.4 mm.
CONSTELLATION_FRAME_MAX_RMS_MM = 1.5

#: Fewest posed frames for an estimate (a 0→120→0 sweep at 200 Hz gives
#: ~1700; 200 is ~1 s of motion and far below any real sweep).
CONSTELLATION_MIN_FRAMES = 200

#: Template residual (mm): the largest mean error, over the window, of a
#: marker-pair separation against the template's. A marker that moved on BB
#: changes its separations by up to its displacement; per-frame noise and the
#: pose-dependent reconstruction error (which moves the per-marker residuals
#: by ~0.5–0.8 mm on the 7-sweep bag) largely average out of a distance. Above CONSTELLATION_MAX_RESIDUAL_MM the estimate FAILS (the template no
#: longer describes BB); above GATE_MAX_RESIDUAL_MM a calibration with a
#: reference is refused by the gate.
CONSTELLATION_MAX_RESIDUAL_MM = 1.0

#: Moving samples: |dy/dt| above this (deg/s) on the reported-yaw series.
SWEEP_MIN_SPEED_DEG_S = 5.0
#: Fewest moving yaw samples (a 0→120→0 heartbeat sweep has ~28; a 100 Hz
#: stamped stream ~280).
SWEEP_MIN_MOVING_SAMPLES = 15
#: Latency search grid (s) and step. Reaches 0.30 s because session B's lag
#: drifted to 174 ms; reaches below zero so a stamped source (lag ≈ 0) has its
#: minimum inside the grid. A minimum within LAG_EDGE_STEPS of either end
#: FAILS: the true lag may lie outside the range.
LAG_GRID_S = (-0.05, 0.30)
LAG_STEP_S = 0.001
LAG_EDGE_STEPS = 3
#: A yaw sample is used only if the posed frames around t_y − τ, over the
#: whole lag grid, have no gap longer than this (s): θ is not interpolated
#: across a lost track (occlusion, rejected frames).
LAG_MAX_FRAME_GAP_S = 0.05
#: The heartbeat's republish period (s): a heartbeat sample is either fresh
#: (age τ) or one period stale (τ + this) — see :func:`fit_yaw_latency`.
HEARTBEAT_STALE_PERIOD_S = 0.1
#: Ceiling on the RMS (deg) of θ(t_y − τ) − y − E(y) − φ at the best τ. The
#: 7-sweep bag gives 0.20–0.28°, sessions A/B 0.31–0.45° (0.64° on a sweep
#: with E left out); a scale or unit
#: mismatch in the yaw source, or wrong correspondences, give degrees.
LAG_MAX_RESIDUAL_DEG = 1.0

#: Mocap-clock gate (see :func:`mocap_clock_off_grid`). A frame's stamp is its
#: QTM frame time mapped to ROS through mocap_interface's QTM->ROS offset, an
#: EMA (alpha 0.01 per packet, ~0.3 s) of the packets' RECEIVE latency. Under
#: Jetson load that offset wanders by tens of ms within a sweep (bag
#: 2026-10-10_00-24-06: 4-59 ms per sweep, vs 0.6-2.3 ms on 2026-10-09_23-49-07),
#: so a frame's stamp is late or early by the wander e(t). The lag fit removes
#: only its mean; the rest biases the offset by -mean(omega*e) (~omega*(e_out -
#: e_ret)/2: 0.3 deg for 10 ms at 67 deg/s) - invisible in the lag residual,
#: and identical for the heartbeat and the stamped yaw (logbook
#: 2026-10-10-bb-yaw-offset-spread-stamped-source). The raw QTM frame times sit
#: on a k*P grid, so a stable mapping keeps consecutive stamp differences on it;
#: a wandering one moves them off it. Fraction of consecutive differences more
#: than P/8 off the grid: 0.0-0.4 % per sweep on the clean bag, 11-36 % on the
#: loaded one. Above MOCAP_CLOCK_MAX_OFF_GRID the estimate FAILS.
MOCAP_CLOCK_MAX_OFF_GRID = 0.05
#: Fewest consecutive frame intervals for the mocap-clock metric (below this
#: it is not computed and the gate does not apply).
MOCAP_CLOCK_MIN_INTERVALS = 50
#: Grid tolerance, as a fraction of the frame period.
MOCAP_CLOCK_GRID_TOL = 0.125

#: Consistency gate (see :func:`check_calibration_consistency`).
GATE_N_SIGMA = 3.0
GATE_MIN_DEG = 0.15
GATE_MAX_AXIS_SHIFT_MM = 1.5
GATE_MAX_RESIDUAL_MM = 0.5


@dataclass
class MarkerTemplate:
    """BB body template: marker positions in BB's body frame, plus the
    estimator constants that travel with them.

    Body frame: origin on the yaw axis (at the calibrated BB height), z along
    the yaw axis, x toward QTM 4 (the old anchor marker), mm.
    ``e_coef_deg`` — E(y) = e1·u + e2·u² + e3·u³, u = max(y, 0)/100, deg;
    applied only to yaw within ``e_valid_yaw_deg``. ``raw_offset_at_pin_deg``
    and ``pinned_yaw_offset_deg`` set the gauge (module comment above).
    ``repeatability_deg`` is the out-of-sample per-sweep scatter, combined
    with each sweep's formal error into the published σ.
    """
    points_mm: np.ndarray
    names: tuple = ()
    e_coef_deg: tuple = (0.0, 0.0, 0.0)
    e_valid_yaw_deg: tuple = (-5.0, 130.0)
    raw_offset_at_pin_deg: float = 0.0
    pinned_yaw_offset_deg: float = 0.0
    pin_uncertainty_deg: Optional[float] = None
    repeatability_deg: float = 0.0
    source: str = ''

    def e_deg(self, yaw_deg):
        """E(y) in degrees (array in, array out)."""
        u = np.maximum(np.asarray(yaw_deg, dtype=float), 0.0) / 100.0
        e1, e2, e3 = self.e_coef_deg
        return e1 * u + e2 * u ** 2 + e3 * u ** 3

    @property
    def gauge_shift_deg(self) -> float:
        """Added to the raw offset to put it in the pinned frame."""
        return self.pinned_yaw_offset_deg - self.raw_offset_at_pin_deg


def load_marker_template(path: str) -> MarkerTemplate:
    """Load ``resources/bb_marker_template.json`` (schema 2). Raises ValueError
    if malformed — including a schema-1 (pause-estimator) file."""
    import json
    with open(path, 'r') as f:
        data = json.load(f)
    if data.get('schema') != 2:
        raise ValueError(f'BB marker template {path}: schema {data.get("schema")!r}, '
                         'need 2 (body frame + E(y) + gauge)')
    try:
        markers = data['markers']
        pts = np.array([m['xyz_mm'] for m in markers], dtype=float)
        names = tuple(str(m.get('id', i)) for i, m in enumerate(markers))
        ey = data['E']
        coef = tuple(float(v) for v in ey['coef_deg'])
        valid = tuple(float(v) for v in ey['valid_yaw_deg'])
        gauge = data['gauge']
        raw_pin = float(gauge['raw_offset_at_pin_deg'])
        pinned = float(gauge['pinned_yaw_offset_deg'])
        pin_sd = gauge.get('pin_uncertainty_deg')
        rep = float(data['sigma']['repeatability_deg'])
    except (KeyError, TypeError, ValueError) as e:
        raise ValueError(f'malformed BB marker template {path}: {e!r}')
    if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] < CONSTELLATION_MIN_MATCHED:
        raise ValueError(f'BB marker template {path} needs >= '
                         f'{CONSTELLATION_MIN_MATCHED} markers of xyz, got {pts.shape}')
    if len(coef) != 3 or len(valid) != 2 or valid[0] >= valid[1]:
        raise ValueError(f'BB marker template {path}: E needs 3 coefficients and a '
                         'valid yaw range lo < hi')
    if not (np.all(np.isfinite(pts)) and np.all(np.isfinite(coef))
            and math.isfinite(raw_pin) and math.isfinite(pinned) and math.isfinite(rep)):
        raise ValueError(f'BB marker template {path} has non-finite values')
    return MarkerTemplate(
        points_mm=pts, names=names, e_coef_deg=coef, e_valid_yaw_deg=valid,
        raw_offset_at_pin_deg=raw_pin, pinned_yaw_offset_deg=pinned,
        pin_uncertainty_deg=None if pin_sd is None else float(pin_sd),
        repeatability_deg=rep, source=path)


def _rot2(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def procrustes_rotation_2d(template_xy: np.ndarray, observed_xy: np.ndarray):
    """Rotation θ and translation t minimising Σ|Rθ·templateᵢ + t − observedᵢ|².

    Returns (theta_rad, t_xy, rms_residual_mm).
    """
    a = np.asarray(template_xy, dtype=float)
    b = np.asarray(observed_xy, dtype=float)
    ac, bc = a.mean(axis=0), b.mean(axis=0)
    a0, b0 = a - ac, b - bc
    theta = math.atan2(float((a0[:, 0] * b0[:, 1] - a0[:, 1] * b0[:, 0]).sum()),
                       float((a0 * b0).sum()))
    R = _rot2(theta)
    t = bc - R @ ac
    res = (a @ R.T + t) - b
    return theta, t, float(math.sqrt(np.mean(np.sum(res ** 2, axis=1))))


def kabsch(template: np.ndarray, observed: np.ndarray):
    """Proper rotation R and translation t minimising Σ|R·aᵢ + t − bᵢ|² (rows
    are points). Returns (R, t, rms_mm)."""
    a = np.asarray(template, dtype=float)
    b = np.asarray(observed, dtype=float)
    ac, bc = a.mean(axis=0), b.mean(axis=0)
    H = (a - ac).T @ (b - bc)
    U, _, Vt = np.linalg.svd(H)
    d = 1.0 if np.linalg.det(Vt.T @ U.T) >= 0 else -1.0
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    t = bc - R @ ac
    res = a @ R.T + t - b
    return R, t, float(math.sqrt(np.mean(np.sum(res ** 2, axis=1))))


@dataclass
class TemplateMatch:
    """A label-free match of points to the template (rotation about z)."""
    theta_rad: float            # rotation template → world, about z
    pairs: list                 # [(template_index, point_index), ...]
    rms_mm: float               # x/y RMS residual of the matched pairs
    lever_mm: float             # sqrt(Σ|pᵢ − p̄|²) of the matched template x/y


def _assign(template_w: np.ndarray, points: np.ndarray, tol: float):
    """Greedy one-to-one nearest assignment (3D) within ``tol``; deterministic."""
    d = np.linalg.norm(template_w[:, None, :] - points[None, :, :], axis=2)
    pairs = []
    taken_t, taken_p = set(), set()
    for flat in np.argsort(d, axis=None, kind='stable'):
        ti, pi = divmod(int(flat), d.shape[1])
        if d[ti, pi] > tol:
            break
        if ti in taken_t or pi in taken_p:
            continue
        pairs.append((ti, pi))
        taken_t.add(ti)
        taken_p.add(pi)
    return sorted(pairs)


def match_template(points: np.ndarray, template: np.ndarray,
                   tol_mm: float = CONSTELLATION_MATCH_TOL_MM,
                   dz_tol_mm: Optional[float] = None) -> Optional[TemplateMatch]:
    """Find which points are which template markers, by geometry alone.

    Every (point pair, template pair) whose 3D separations agree within
    ``tol_mm`` and whose height differences agree within ``dz_tol_mm``
    (default ``tol_mm``; the yaw stage turns about a near-vertical axis, so
    Δz is near-invariant) proposes a rotation about z plus a translation; the
    proposal matching the most markers (then the smallest residual) wins and
    is refined by Procrustes on its matches. Labels play no part, so a
    relabelled, missing or spurious marker changes only which points are
    used. Returns None when fewer than :data:`CONSTELLATION_MIN_MATCHED` match.
    """
    dz_tol = tol_mm if dz_tol_mm is None else dz_tol_mm
    P = np.asarray(points, dtype=float).reshape(-1, 3)
    T = np.asarray(template, dtype=float).reshape(-1, 3)
    if len(P) < CONSTELLATION_MIN_MATCHED or len(T) < CONSTELLATION_MIN_MATCHED:
        return None
    dP = np.linalg.norm(P[:, None] - P[None], axis=2)
    dT = np.linalg.norm(T[:, None] - T[None], axis=2)
    best = None
    for i in range(len(P)):
        for j in range(len(P)):
            if i == j:
                continue
            vo = P[j, :2] - P[i, :2]
            for a in range(len(T)):
                for b in range(len(T)):
                    if a == b or abs(dP[i, j] - dT[a, b]) > tol_mm:
                        continue
                    if abs((P[j, 2] - P[i, 2]) - (T[b, 2] - T[a, 2])) > dz_tol:
                        continue
                    vt = T[b, :2] - T[a, :2]
                    th = math.atan2(vt[0] * vo[1] - vt[1] * vo[0], vt[0] * vo[0] + vt[1] * vo[1])
                    R = _rot2(th)
                    txy = P[i, :2] - R @ T[a, :2]
                    tz = P[i, 2] - T[a, 2]
                    Tw = np.c_[T[:, :2] @ R.T + txy, T[:, 2] + tz]
                    pairs = _assign(Tw, P, max(tol_mm, dz_tol))
                    if len(pairs) < CONSTELLATION_MIN_MATCHED:
                        continue
                    ti = [p[0] for p in pairs]
                    pi = [p[1] for p in pairs]
                    _, _, rms = procrustes_rotation_2d(T[ti, :2], P[pi, :2])
                    key = (-len(pairs), rms)
                    if best is None or key < best[0]:
                        best = (key, pairs)
    if best is None:
        return None
    pairs = best[1]
    # Refine: Procrustes on the matches, re-assign once, Procrustes again.
    for _ in range(2):
        ti = [p[0] for p in pairs]
        pi = [p[1] for p in pairs]
        th, txy, rms = procrustes_rotation_2d(T[ti, :2], P[pi, :2])
        tz = float(np.mean(P[pi, 2] - T[ti, 2]))
        Tw = np.c_[T[:, :2] @ _rot2(th).T + txy, T[:, 2] + tz]
        new_pairs = _assign(Tw, P, max(tol_mm, dz_tol))
        if len(new_pairs) < CONSTELLATION_MIN_MATCHED or new_pairs == pairs:
            break
        pairs = new_pairs
    ti = [p[0] for p in pairs]
    pi = [p[1] for p in pairs]
    th, _, rms = procrustes_rotation_2d(T[ti, :2], P[pi, :2])
    a0 = T[ti, :2] - T[ti, :2].mean(axis=0)
    lever = float(math.sqrt(np.sum(a0 ** 2)))
    return TemplateMatch(theta_rad=th, pairs=pairs, rms_mm=rms, lever_mm=lever)


@dataclass
class ConstellationTrack:
    """Per-frame rigid poses of the template over a window (posed frames only)."""
    t: np.ndarray               # (n,) frame stamps, s
    R: np.ndarray               # (n, 3, 3) body → world rotation
    origin: np.ndarray          # (n, 3) world position of the body origin (the axis point)
    rms_mm: np.ndarray          # (n,) per-frame fit RMS
    n_matched: np.ndarray       # (n,) markers matched
    points: np.ndarray          # (n, K, 3) matched points in template order (nan: unmatched)
    template_mm: np.ndarray     # (K, 3) the template the track was fitted with
    n_frames_in: int = 0
    n_rejected: int = 0         # frames with < 3 matched or RMS > the frame ceiling

    @property
    def theta_deg(self) -> np.ndarray:
        """Unwrapped world yaw (deg) of the body x axis."""
        return np.degrees(np.unwrap(np.arctan2(self.R[:, 1, 0], self.R[:, 0, 0])))

    def pair_residual_mm(self, mask=None) -> np.ndarray:
        """(K, K) mean observed − template separation per marker pair over the
        frames in ``mask`` (default all); nan where a pair was never seen."""
        P = self.points if mask is None else self.points[np.asarray(mask, bool)]
        T = self.template_mm
        K = len(T)
        out = np.full((K, K), np.nan)
        dT = np.linalg.norm(T[:, None] - T[None], axis=2)
        for a in range(K):
            for b in range(a + 1, K):
                d = np.linalg.norm(P[:, a] - P[:, b], axis=1)
                d = d[np.isfinite(d)]
                if len(d):
                    out[a, b] = out[b, a] = float(d.mean()) - dT[a, b]
        return out

    def template_residual_mm(self, mask=None) -> float:
        """Largest |mean separation error| over marker pairs: a marker that
        moved on BB changes its separations by up to its displacement, while
        frame noise averages out over the window."""
        r = np.abs(self.pair_residual_mm(mask))
        return float(np.nanmax(r)) if np.any(np.isfinite(r)) else float('nan')


def track_constellation(marker_frames, template,
                        tol_mm: float = CONSTELLATION_MATCH_TOL_MM,
                        max_frame_rms_mm: float = CONSTELLATION_FRAME_MAX_RMS_MM,
                        max_track_gap_s: float = 0.05) -> ConstellationTrack:
    """Fit the template to every frame as a rigid body, label-free.

    ``marker_frames``: sequence of (t_s, (n, 3) points), any order. The first
    frame (and any after a lost track or a gap > ``max_track_gap_s``) is
    matched from scratch (:func:`match_template`); later frames reuse the
    previous pose to assign points to markers within ``tol_mm`` and refit in
    3D (:func:`kabsch`). Deterministic.
    """
    T = template.points_mm if isinstance(template, MarkerTemplate) else np.asarray(template, float)
    K = len(T)
    frames = sorted(((float(t), np.asarray(f, dtype=float).reshape(-1, 3)) for t, f in marker_frames),
                    key=lambda r: r[0])
    ts, Rs, os_, rms_l, nm_l = [], [], [], [], []
    pts_l = []
    prev = None          # (t, R, origin)
    rejected = 0
    for t, P in frames:
        P = P[np.all(np.isfinite(P), axis=1)]
        pairs = None
        if prev is not None and t - prev[0] <= max_track_gap_s and len(P) >= CONSTELLATION_MIN_MATCHED:
            pairs = _assign(T @ prev[1].T + prev[2], P, tol_mm)
            if len(pairs) < CONSTELLATION_MIN_MATCHED:
                pairs = None
        if pairs is None:
            m = match_template(P, T, tol_mm, CONSTELLATION_INIT_DZ_TOL_MM)
            pairs = None if m is None else m.pairs
        fit = None
        if pairs is not None:
            for _ in range(2):   # fit, re-assign with the 3D pose, refit
                ti = [p[0] for p in pairs]
                pi = [p[1] for p in pairs]
                R, o, rms = kabsch(T[ti], P[pi])
                fit = (R, o, rms, pairs)
                new = _assign(T @ R.T + o, P, tol_mm)
                if len(new) < CONSTELLATION_MIN_MATCHED or new == pairs:
                    break
                pairs = new
        if fit is None or fit[2] > max_frame_rms_mm:
            rejected += 1
            prev = None
            continue
        R, o, rms, pairs = fit
        ti = [p[0] for p in pairs]
        pi = [p[1] for p in pairs]
        Pm = np.full((K, 3), np.nan)
        Pm[ti] = P[pi]
        pts_l.append(Pm)
        ts.append(t)
        Rs.append(R)
        os_.append(o)
        rms_l.append(rms)
        nm_l.append(len(pairs))
        prev = (t, R, o)
    return ConstellationTrack(
        t=np.array(ts), R=np.array(Rs).reshape(-1, 3, 3), origin=np.array(os_).reshape(-1, 3),
        rms_mm=np.array(rms_l), n_matched=np.array(nm_l, dtype=int),
        points=np.array(pts_l).reshape(-1, K, 3), template_mm=np.array(T, dtype=float), n_frames_in=len(frames), n_rejected=rejected)


def mocap_clock_off_grid(t_frames) -> Optional[float]:
    """Fraction of consecutive distinct frame stamps whose difference is more
    than ``MOCAP_CLOCK_GRID_TOL`` of a frame period off a whole number of
    periods — a direct measure of how much the QTM->ROS mapping moved between
    frames (see ``MOCAP_CLOCK_MAX_OFF_GRID``). None when fewer than
    ``MOCAP_CLOCK_MIN_INTERVALS`` intervals exist.

    The period P is not assumed (the QTM rate is a QTM setting, and the stamps
    are in ROS seconds): P0 is the median of the differences near their 5th
    percentile; mocap_node publishes snapshots on a timer, so the smallest
    common difference may be a multiple of the true period, and the metric is
    the smallest over P0/n, n = 1..4 (P0/n >= 1 ms) — a finer grid that the
    stamps also fit cannot raise it, so this never refuses a clean clock for
    the wrong P."""
    t = np.unique(np.asarray(t_frames, dtype=float))
    d = np.diff(t)
    d = d[d > 0]
    if len(d) < MOCAP_CLOCK_MIN_INTERVALS:
        return None
    p5 = float(np.percentile(d, 5))
    near = d[(d > 0.75 * p5) & (d < 1.25 * p5)]
    p0 = float(np.median(near)) if len(near) else p5
    best = 1.0
    for n in range(1, 5):
        P = p0 / n
        if P < 1e-3:
            break
        frac = d - np.round(d / P) * P
        best = min(best, float(np.mean(np.abs(frac) > MOCAP_CLOCK_GRID_TOL * P)))
    return best


@dataclass
class LatencyFit:
    """Best lag and offset of θ(t_y − τ) − y − E(y) over the moving samples."""
    lag_s: float
    phi_raw_deg: float          # mean at the best lag (template gauge, before the pin)
    residual_rms_deg: float
    formal_se_deg: float        # SD/√n_eff, n_eff from the lag-1 autocorrelation
    n_moving: int
    at_edge: bool
    lag_grid_s: tuple = LAG_GRID_S
    #: Fraction of samples read one ``stale_period_s`` older than ``lag_s``
    #: (heartbeat only; 0 for a stamped source).
    stale_fraction: float = 0.0
    stale_period_s: float = 0.0


def fit_yaw_latency(t_frames, theta_deg, t_yaw, yaw_deg, e_deg_fn=None,
                    e_valid_yaw_deg=(-1e9, 1e9),
                    lag_grid_s: tuple = LAG_GRID_S, lag_step_s: float = LAG_STEP_S,
                    min_speed_deg_s: float = SWEEP_MIN_SPEED_DEG_S,
                    max_frame_gap_s: float = LAG_MAX_FRAME_GAP_S,
                    stale_period_s: Optional[float] = None) -> Optional[LatencyFit]:
    """Grid-fit the lag τ between the frames' yaw θ(t) and the reported yaw y(t_y).

    Model: θ(t_y − τ) = y + E(y) + φ on samples where BB is turning
    (|dy/dt| > ``min_speed_deg_s``), with y inside ``e_valid_yaw_deg`` and
    t_y − τ inside the frames' span, with no gap between posed frames longer
    than ``max_frame_gap_s``, for every τ of the grid (so the sample set does
    not change with τ, and θ is never interpolated across a lost track). For each τ, φ is the mean and the cost is the
    scatter about it; the minimum is refined by a parabola through its
    neighbours. Returns None when fewer than two samples qualify.

    ``stale_period_s`` (the heartbeat only): each sample may instead be one
    period older than τ, whichever reads closer. The heartbeat's yaw passes a
    0.1 s host timer that republishes the latest value, so when that timer
    and the upstream 10 Hz stage tick nearly together a sample is either fresh
    or exactly one period stale — session B (2026-10-09 03:13) has ages of
    ~75 ms and ~175 ms (17–34 % / 66–83 % per 10-minute window), and a
    single-lag fit there scatters 5° RMS.
    A stamped source has no such stage, so it is fitted with one lag.
    """
    tf = np.asarray(t_frames, dtype=float)
    th = np.asarray(theta_deg, dtype=float)
    ty = np.asarray(t_yaw, dtype=float)
    y = np.asarray(yaw_deg, dtype=float)
    o = np.argsort(ty, kind='stable')
    ty, y = ty[o], canonical_yaw_deg(y[o])
    if len(ty) < 3 or len(tf) < 2:
        return None
    w = np.gradient(y, ty)
    lo, hi = lag_grid_s
    P = float(stale_period_s) if stale_period_s else 0.0
    hi_eff = hi + P                      # oldest age any sample may be read at
    sel = ((np.abs(w) > min_speed_deg_s) & (y >= e_valid_yaw_deg[0]) & (y <= e_valid_yaw_deg[1])
           & (ty - hi_eff >= tf[0]) & (ty - lo <= tf[-1]))
    if len(tf) > 1 and np.any(sel):
        gap = np.diff(tf)
        i0 = np.clip(np.searchsorted(tf, ty - hi_eff) - 1, 0, len(gap) - 1)
        i1 = np.clip(np.searchsorted(tf, ty - lo), 0, len(gap) - 1)
        for k in np.where(sel)[0]:
            if gap[i0[k]:i1[k] + 1].max() > max_frame_gap_s:
                sel[k] = False
    if sel.sum() < 2:
        return None
    ty_m, y_m = ty[sel], y[sel]
    target = y_m + (e_deg_fn(y_m) if e_deg_fn is not None else 0.0)

    def resid(tau):
        """Per-sample d = θ(t_y − age) − target, the age being τ, or τ + P for
        a sample that reads closer one stale period later; and that choice."""
        d0 = np.interp(ty_m - tau, tf, th) - target
        if not P:
            return d0, np.zeros(len(d0), bool)
        d1 = np.interp(ty_m - tau - P, tf, th) - target
        phi = float(np.median(d0))
        for _ in range(3):
            stale = np.abs(d1 - phi) < np.abs(d0 - phi)
            d = np.where(stale, d1, d0)
            phi = float(d.mean())
        return d, stale

    n_steps = int(round((hi - lo) / lag_step_s)) + 1
    taus = lo + lag_step_s * np.arange(n_steps)
    cost = np.empty(n_steps)
    for k, tau in enumerate(taus):
        d, _ = resid(tau)
        cost[k] = float(np.mean((d - d.mean()) ** 2))
    kb = int(np.argmin(cost))
    tau = float(taus[kb])
    if 0 < kb < n_steps - 1:
        c0, c1, c2 = cost[kb - 1], cost[kb], cost[kb + 1]
        den = c0 - 2 * c1 + c2
        if den > 0:
            tau += lag_step_s * float(np.clip(0.5 * (c0 - c2) / den, -0.5, 0.5))
    d, stale = resid(tau)
    if P and stale.mean() > 0.9:
        # (Nearly) every sample one period older than τ is the same fit as
        # all of them fresh at τ + P: report that canonical single-age form
        # when it fits about as well (the few noise picks cost < 1.5× in
        # variance; a genuine two-age mixture costs far more — its fresh
        # samples, read ω·P ≈ degrees off, have no younger age to fall back on).
        d2, stale2 = resid(tau + P)
        if np.var(d2) <= 1.5 * np.var(d):
            tau, d, stale = tau + P, d2, stale2
    # θ and y may differ by whole turns (unwrap branches): wrap about the circular mean.
    m0 = math.degrees(math.atan2(float(np.sin(np.radians(d)).mean()), float(np.cos(np.radians(d)).mean())))
    d = m0 + (d - m0 + 180.0) % 360.0 - 180.0
    phi = float(d.mean())
    r = d - phi
    n = len(r)
    sd = float(np.std(r, ddof=1)) if n > 1 else 0.0
    rho = float(np.corrcoef(r[:-1], r[1:])[0, 1]) if n > 3 and np.std(r) > 0 else 0.0
    rho = min(max(rho, 0.0), 0.95)
    n_eff = max(n * (1.0 - rho) / (1.0 + rho), 1.0)
    edge = LAG_EDGE_STEPS * lag_step_s
    return LatencyFit(
        lag_s=tau, phi_raw_deg=((phi + 180.0) % 360.0) - 180.0,
        residual_rms_deg=float(np.sqrt(np.mean(r ** 2))),
        formal_se_deg=sd / math.sqrt(n_eff), n_moving=n,
        at_edge=bool(tau < lo + edge or tau > hi - edge),
        lag_grid_s=(float(lo), float(hi)),
        stale_fraction=float(stale.mean()) if n else 0.0,
        stale_period_s=P)


#: Joint name of the stamped BB yaw in ``bb/axis_estimates`` (JointState,
#: 100 Hz, stamped with the bridge's synced wall clock at its snapshot). It
#: is the third joint after ``bb_pitch`` / ``bb_hand``, position in DEGREES
#: (BB-local, unwrapped, the heartbeat's zero), appended only when the bridge
#: has a fresh BB yaw frame (BB firmware 6 + can-bridge firmware 28); older
#: firmware publishes two joints and the heartbeat is used. The value is the
#: latest of BB's 150 Hz samples as of the stamp (0–6.7 ms old), so the fitted
#: lag should come out at a few ms; it is still fitted and reported.
STAMPED_YAW_JOINT = 'bb_yaw'


def stamped_yaw_samples_from_joint_state(msg) -> list:
    """[(t_s, yaw_deg)] from one ``bb/axis_estimates`` JointState (empty if it
    carries no stamped yaw). Duck-typed: needs ``header.stamp``, ``name``,
    ``position``."""
    names = list(getattr(msg, 'name', []) or [])
    pos = list(getattr(msg, 'position', []) or [])
    st = msg.header.stamp
    t = st.sec + st.nanosec * 1e-9
    out = []
    if t <= 0:
        return out
    if STAMPED_YAW_JOINT in names:
        i = names.index(STAMPED_YAW_JOINT)
        if i < len(pos) and math.isfinite(pos[i]):
            out.append((t, float(pos[i])))
    return out


@dataclass
class SweepYawEstimate:
    """BB's yaw offset and axis point from one calibration window."""
    yaw_offset_rad: float       # pinned gauge (published)
    yaw_offset_std_deg: float   # formal SE ⊕ out-of-sample repeatability
    formal_se_deg: float
    repeatability_deg: float
    phi_raw_deg: float          # template gauge, before the pin
    lag_s: float
    lag_residual_rms_deg: float
    n_moving: int               # yaw samples in the fit
    axis_point_mm: np.ndarray   # mean world position of the body origin over the posed frames
    axis_point_sd_mm: np.ndarray
    n_frames: int               # posed frames
    n_frames_rejected: int
    matched_min: int
    matched_max: int
    frame_rms_median_mm: float
    template_residual_mm: float
    yaw_source: str = 'heartbeat'
    n_frames_moving: int = 0    # posed frames inside a turning stretch (position + residual)
    stale_fraction: float = 0.0 # heartbeat samples read one republish period older
    #: Fraction of frame intervals off the QTM frame grid (:func:`mocap_clock_off_grid`; None: too few frames to tell).
    frame_clock_off_grid: Optional[float] = None

    def summary(self) -> str:
        """One-line description for the result message."""
        clock = ('n/a' if self.frame_clock_off_grid is None
                 else f'{self.frame_clock_off_grid * 100:.1f} % off grid')
        return (f'sweep estimator: {self.n_frames} frames ({self.n_frames_moving} moving, '
                f'{self.n_frames_rejected} rejected, '
                f'{self.matched_min}-{self.matched_max} markers, median fit '
                f'{self.frame_rms_median_mm:.2f} mm, template residual '
                f'{self.template_residual_mm:.2f} mm, mocap clock {clock}), '
                f'yaw source {self.yaw_source} '
                f'lag {self.lag_s * 1e3:.1f} ms ({self.stale_fraction * 100:.0f} % one period '
                f'stale) over {self.n_moving} moving samples '
                f'(residual {self.lag_residual_rms_deg:.2f}° RMS), '
                f'σ {self.yaw_offset_std_deg:.3f}° (formal {self.formal_se_deg:.3f}° ⊕ '
                f'repeatability {self.repeatability_deg:.3f}°)')


def estimate_sweep_yaw_offset(
    marker_frames,
    yaw_samples,
    template: MarkerTemplate,
    *,
    yaw_source: str = 'heartbeat',
    lag_grid_s: tuple = LAG_GRID_S,
    min_frames: int = CONSTELLATION_MIN_FRAMES,
    max_residual_mm: float = CONSTELLATION_MAX_RESIDUAL_MM,
    max_lag_residual_deg: float = LAG_MAX_RESIDUAL_DEG,
    min_moving: int = SWEEP_MIN_MOVING_SAMPLES,
    track: Optional[ConstellationTrack] = None,
    max_clock_off_grid: float = MOCAP_CLOCK_MAX_OFF_GRID,
) -> SweepYawEstimate:
    """BB's yaw offset (and axis point) from a whole calibration window.

    Args:
        marker_frames: sequence of (t_s, (n, 3) points) — every BB marker
            point of each mocap frame, labels discarded, stamped at the QTM
            frame time mapped to the ROS clock.
        yaw_samples: sequence of (t_s, yaw_deg) on the same clock — the
            heartbeat's receive time, or a stamped stream's sample time.
        template: :class:`MarkerTemplate` (schema 2).
        yaw_source: label for the message ('heartbeat' / 'stamped').
        track: a precomputed :func:`track_constellation` (offline reuse).

    Raises ValueError (loudly, with a code) when: too few frames were posed
    (CONSTELLATION_TOO_FEW_FRAMES / CONSTELLATION_TOO_FEW_MARKERS), the
    template residual is above ``max_residual_mm`` (CONSTELLATION_RESIDUAL),
    the frame stamps' QTM->ROS mapping wandered during the sweep (more than
    ``max_clock_off_grid`` of the frame intervals off the QTM grid;
    CONSTELLATION_MOCAP_CLOCK — the offset would be biased by an amount the
    lag residual cannot show, for either yaw source),
    too few moving yaw samples (CONSTELLATION_TOO_FEW_MOVING), the lag fit
    hits the edge of its grid (CONSTELLATION_LAG_AT_EDGE), or its residual is
    above ``max_lag_residual_deg`` (CONSTELLATION_LAG_RESIDUAL).
    """
    if track is None:
        track = track_constellation(marker_frames, template)
    n = len(track.t)
    if track.n_frames_in < min_frames:
        raise ValueError(
            f'CONSTELLATION_TOO_FEW_FRAMES: {track.n_frames_in} mocap frames with BB '
            f'markers in the calibration window (need {min_frames})')
    if n < min_frames:
        raise ValueError(
            f'CONSTELLATION_TOO_FEW_MARKERS: {n} of {track.n_frames_in} frames matched '
            f'>= {CONSTELLATION_MIN_MATCHED} template markers within '
            f'{CONSTELLATION_FRAME_MAX_RMS_MM:.1f} mm (need {min_frames})')
    off_grid = mocap_clock_off_grid(track.t)
    if off_grid is not None and off_grid > max_clock_off_grid:
        raise ValueError(
            f'CONSTELLATION_MOCAP_CLOCK: {off_grid * 100:.1f} % of the mocap frame intervals are '
            f'off the QTM frame grid (limit {max_clock_off_grid * 100:.1f} %) — the QTM-to-ROS '
            'clock mapping wandered during the sweep (a loaded Jetson), which biases the yaw '
            'offset by up to tenths of a degree whichever yaw source is used; refusing. '
            'Recalibrate with the Jetson unloaded (no test suite or other heavy process)')
    ys = np.array(sorted(((float(t), float(v)) for t, v in yaw_samples),
                         key=lambda r: r[0])).reshape(-1, 2)
    fit = fit_yaw_latency(track.t, track.theta_deg, ys[:, 0], ys[:, 1],
                          template.e_deg, template.e_valid_yaw_deg, lag_grid_s,
                          stale_period_s=HEARTBEAT_STALE_PERIOD_S if yaw_source == 'heartbeat' else None)
    if fit is None or fit.n_moving < min_moving:
        raise ValueError(
            f'CONSTELLATION_TOO_FEW_MOVING: {0 if fit is None else fit.n_moving} moving '
            f'{yaw_source} yaw samples overlap the mocap frames (need {min_moving})')
    if fit.at_edge:
        raise ValueError(
            f'CONSTELLATION_LAG_AT_EDGE: {yaw_source} yaw lag {fit.lag_s * 1e3:.1f} ms '
            f'at the edge of its search range '
            f'[{lag_grid_s[0] * 1e3:.0f}, {lag_grid_s[1] * 1e3:.0f}] ms')
    if fit.residual_rms_deg > max_lag_residual_deg:
        raise ValueError(
            f'CONSTELLATION_LAG_RESIDUAL: mocap vs {yaw_source} yaw {fit.residual_rms_deg:.2f}° '
            f'RMS > {max_lag_residual_deg:.2f}° at lag {fit.lag_s * 1e3:.1f} ms — wrong yaw '
            'units/source or a bad track')
    # Position and template residual come from the MOVING frames too (frame
    # time + lag inside a turning stretch of the reported yaw). The parked
    # pose reconstructs non-rigidly in every 2026-10-09 bag (pair separations
    # off by 1.3–2.4 mm there vs ≤ 0.3 mm (A/B) / 0.6 mm (7-sweep bag) at
    # other yaws), and the pause is ~25 % of a sweep's frames.
    w = np.gradient(canonical_yaw_deg(ys[:, 1]), ys[:, 0])
    moving = np.abs(np.interp(track.t + fit.lag_s, ys[:, 0], w)) > SWEEP_MIN_SPEED_DEG_S
    if moving.sum() < min_frames:
        raise ValueError(
            f'CONSTELLATION_TOO_FEW_MOVING: {int(moving.sum())} posed mocap frames '
            f'while BB was turning (need {min_frames})')
    tres = track.template_residual_mm(moving)
    if not (tres <= max_residual_mm):
        raise ValueError(
            f'CONSTELLATION_RESIDUAL: marker-pair separations off the template by '
            f'{tres:.2f} mm > {max_residual_mm:.2f} mm — a marker moved on BB; rebuild '
            'the template')
    rep = float(template.repeatability_deg)
    offset_deg = fit.phi_raw_deg + template.gauge_shift_deg
    org = track.origin[moving]
    return SweepYawEstimate(
        yaw_offset_rad=wrap_pi(math.radians(offset_deg)),
        yaw_offset_std_deg=math.sqrt(fit.formal_se_deg ** 2 + rep ** 2),
        formal_se_deg=fit.formal_se_deg, repeatability_deg=rep,
        phi_raw_deg=fit.phi_raw_deg, lag_s=fit.lag_s,
        lag_residual_rms_deg=fit.residual_rms_deg, n_moving=fit.n_moving,
        stale_fraction=fit.stale_fraction,
        axis_point_mm=org.mean(axis=0), axis_point_sd_mm=org.std(axis=0),
        n_frames=n, n_frames_rejected=track.n_rejected,
        matched_min=int(track.n_matched.min()), matched_max=int(track.n_matched.max()),
        frame_rms_median_mm=float(np.median(track.rms_mm)),
        template_residual_mm=tres, yaw_source=yaw_source,
        n_frames_moving=int(moving.sum()), frame_clock_off_grid=off_grid)


@dataclass
class GateVerdict:
    accepted: bool
    message: str
    delta_deg: float = 0.0
    threshold_deg: float = 0.0
    axis_shift_mm: float = 0.0
    overridden: bool = False
    no_reference: bool = False


def check_calibration_consistency(
    new_offset_deg: float,
    new_std_deg: float,
    new_position_mm,
    template_residual_mm: float,
    reference: Optional[dict],
    bb_moved: bool = False,
    reference_label: str = 'last accepted',
    n_sigma: float = GATE_N_SIGMA,
    min_deg: float = GATE_MIN_DEG,
    max_axis_shift_mm: float = GATE_MAX_AXIS_SHIFT_MM,
    max_residual_mm: float = GATE_MAX_RESIDUAL_MM,
) -> GateVerdict:
    """Refuse a calibration that disagrees with the last accepted one while BB
    is not declared moved.

    ``reference``: the last accepted calibration ({'yaw_offset_deg',
    'position_mm'}) or None. With no reference the calibration is accepted
    (``no_reference`` set; the caller logs a WARN) — there is nothing to
    compare against, and the template's pin is NOT used as one.

    With a reference, refused when |Δyaw| > max(n_sigma·√(σ_new² + σ_ref²),
    min_deg), or |Δaxis point| > ``max_axis_shift_mm`` (3D), or the template
    residual > ``max_residual_mm``. Both estimates carry their own σ
    (``yaw_offset_std_deg``; the reference's as persisted, 0 if absent): the
    difference of two independent sweeps spreads √2 wider than one, and a
    limit of 3σ_new alone refused a good pair in bag 2026-10-09_23-49-07
    (Δ 0.197° against 0.189°, both sweeps ±0.05–0.06°; logbook
    2026-10-09-bb-calibration-heartbeat-yaw-wrap). ``bb_moved`` — the operator's statement that BB moved
    OR that QTM was recalibrated (either changes the frame; one override
    resets the reference) — accepts a yaw/axis change; it does not excuse a
    template residual (a marker that moved on BB needs a new template, not a
    new reference).
    """
    if reference is None:
        return GateVerdict(True, 'gate: no reference calibration — accepted and '
                                 'persisted as the reference', no_reference=True)
    ref_yaw = float(reference['yaw_offset_deg'])
    delta = math.degrees(wrap_pi(math.radians(new_offset_deg - ref_yaw)))
    ref_std = float(reference.get('yaw_offset_std_deg') or 0.0)
    thr = max(n_sigma * math.hypot(float(new_std_deg), ref_std), min_deg)
    ref_pos = reference.get('position_mm')
    shift = (float(np.linalg.norm(np.asarray(new_position_mm, float) - np.asarray(ref_pos, float)))
             if ref_pos is not None else 0.0)
    head = (f'gate: Δyaw {delta:+.3f}° (limit ±{thr:.3f}°), Δaxis {shift:.2f} mm '
            f'(limit {max_axis_shift_mm:.1f} mm) vs {reference_label} {ref_yaw:.3f}°')
    if template_residual_mm > max_residual_mm:
        return GateVerdict(
            False,
            f'TEMPLATE_RESIDUAL: a BB marker sits {template_residual_mm:.2f} mm off the '
            f'template > {max_residual_mm:.2f} mm — a marker moved on BB; rebuild the '
            'template (bb_moved does not override this)',
            delta, thr, shift)
    problems = []
    if abs(delta) > thr:
        problems.append('yaw')
    if shift > max_axis_shift_mm:
        problems.append('axis point')
    # The refusal's operator line: the code and the decisive numbers only
    # (the estimator detail is the caller's DEBUG). Wording, not logic.
    exceeded = []
    if 'yaw' in problems:
        exceeded.append(f'Δyaw {delta:+.3f}° exceeds ±{thr:.3f}°')
    if 'axis point' in problems:
        exceeded.append(f'axis point moved {shift:.2f} mm > {max_axis_shift_mm:.1f} mm')
    if not problems:
        return GateVerdict(True, f'{head} ok', delta, thr, shift)
    if bb_moved:
        return GateVerdict(True, f'{head} — {" and ".join(problems)} changed, ACCEPTED on '
                                 'the bb_moved override (new reference; refit the aim '
                                 'correction)', delta, thr, shift, overridden=True)
    return GateVerdict(
        False,
        f'CALIBRATION_INCONSISTENT: {", ".join(exceeded)} vs {ref_yaw:.3f}° '
        f'({reference_label}) — set bb_moved:=true if BB or QTM moved',
        delta, thr, shift)

# ---------------------------------------------------------------------------
#  High-level calibration result
# ---------------------------------------------------------------------------

@dataclass
class CalibrationResult:
    """Outputs from a successful BB calibration."""
    bb_position_mm: np.ndarray          # [x, y, z] in global frame
    yaw_offset_rad: float               # add to local yaw → global angle
    yaw_offset_std_deg: float           # uncertainty on yaw offset
    axis_direction: np.ndarray          # unit vector along rotation axis
    axis_tilt_deg: float                # tilt from vertical
    #: Span of BB's reported yaw series (deg) — the PRIMARY sweep-completeness
    #: measurement, and the number MIN_ARC_DEG was set from (the first hardware
    #: calibrate swept 118.8°, 2026-08-25). 0.0 when too few yaw readings
    #: arrived for the span to mean anything (see MIN_YAW_READINGS).
    yaw_span_deg: float = 0.0
    marker_metrics: dict[int, MarkerFitMetrics] = field(default_factory=dict)
    #: Which estimator produced ``yaw_offset_rad``: ``'constellation'`` (the
    #: production path, :func:`estimate_sweep_yaw_offset`) or ``'anchor'``
    #: (the retired single-marker estimator, only when the caller passed no
    #: template — sweep-fit fixtures and diagnostics). ``mocap_node`` refuses
    #: to publish anything but ``'constellation'``.
    yaw_method: str = 'anchor'
    #: The sweep estimate (lag, σ breakdown, residuals); None on the anchor path.
    yaw_estimate: Optional[SweepYawEstimate] = None
    #: The retired anchor estimator's value on the same sweep (rad), kept as a
    #: logged diagnostic on the constellation path; None if it could not be
    #: computed (anchor missing or an outcast — no longer a refusal).
    anchor_yaw_offset_rad: Optional[float] = None
    #: The arc fit's axis point at the plane height (the pre-2026-10-09
    #: position). On the constellation path ``bb_position_mm`` is the body
    #: model's axis point and this is the cross-check; on the anchor path the
    #: two are the same.
    arc_position_mm: Optional[np.ndarray] = None


def run_calibration(
    calibration_data: dict[int, list[np.ndarray]],
    yaw_readings_deg: list[float],
    pitch_z_offset_mm: float,
    min_points: int = 50,
    max_yaw_std_deg: float = 5.0,
    min_arc_deg: float = MIN_ARC_DEG,
    min_radius_mm: float = MIN_MARKER_RADIUS_MM,
    min_marker_arc_deg: float = MIN_MARKER_ARC_DEG,
    plane_marker_indices: tuple = BB_PLANE_MARKER_INDICES,
    yaw_anchor_index: int = BB_YAW_ANCHOR_INDEX,
    max_axis_dev_mm: float = MAX_AXIS_DEVIATION_MM,
    min_agreeing: int = MIN_AGREEING_MARKERS,
    *,
    marker_frames=None,
    yaw_samples=None,
    template=None,
    yaw_source: str = 'heartbeat',
) -> CalibrationResult:
    """Execute the full calibration pipeline.

    Tilt and the sweep gates always come from the sweep's arc fit. The YAW
    OFFSET and the POSITION come from :func:`estimate_sweep_yaw_offset` when
    ``template`` is given (with ``marker_frames`` and ``yaw_samples``, both
    timestamped on one clock; ``yaw_source`` labels the yaw stream) — the
    production path since 2026-10-09; the arc fit's position is then kept as
    ``arc_position_mm``, a cross-check. Without
    a template it falls back to the retired single-anchor estimator
    (:func:`calculate_yaw_offset`), which the sweep-fit tests still exercise;
    ``CalibrationResult.yaw_method`` says which ran, and ``mocap_node``
    refuses to publish an ``'anchor'`` result.

    Args:
        calibration_data:  marker_index → list of [x,y,z] arrays collected
                           during the CALIBRATING state.
        yaw_readings_deg:  yaw readings from BB heartbeat during calibration.
        pitch_z_offset_mm: vertical offset of pitch axis from yaw axis (from config).
        min_points:        minimum marker samples for circle fitting.
        max_yaw_std_deg:   maximum allowed yaw-offset uncertainty.
        min_arc_deg:       minimum yaw arc the sweep must cover
                           (:data:`MIN_ARC_DEG`) — the guard against a
                           truncated sweep fitting a plausible-looking circle.
                           Applied ONLY to BB's reported yaw series, the
                           noise-immune measurement.
        min_radius_mm:     per-marker fitted-radius floor
                           (:data:`MIN_MARKER_RADIUS_MM`), forwarded to
                           :func:`find_rotation_axis`.
        min_marker_arc_deg: per-marker arc floor for INCLUSION in the axis
                           average (:data:`MIN_MARKER_ARC_DEG`), forwarded to
                           :func:`find_rotation_axis`. Separate from
                           ``min_arc_deg`` and deliberately lower: a marker
                           occluded during a legal sweep fits a short arc.
        plane_marker_indices: zero-based indices whose Z sets BB's x/y plane
                           (:data:`BB_PLANE_MARKER_INDICES`). All markers,
                           these or not, feed the axis fit.
        yaw_anchor_index:  zero-based index of the yaw-offset anchor marker
                           (:data:`BB_YAW_ANCHOR_INDEX`).
        max_axis_dev_mm / min_agreeing: the consensus gate, forwarded to
                           :func:`find_rotation_axis`. An outcast is left out
                           of the axis AND the plane height; an outcast yaw
                           anchor refuses the calibration.

    Returns:
        CalibrationResult on success.

    Raises:
        ValueError on any calibration failure.
    """
    # Convert lists → numpy arrays. Every marker feeds the axis fit; only the
    # co-planar ones feed the Z plane (see BB_PLANE_MARKER_INDICES).
    marker_trajectories: dict[int, np.ndarray] = {}
    plane_z: list[float] = []

    for idx, positions in calibration_data.items():
        if positions:
            arr = np.array(positions)
            marker_trajectories[idx] = arr
            if idx in plane_marker_indices:
                plane_z.extend(arr[:, 2].tolist())

    if not marker_trajectories:
        raise ValueError('No valid marker positions recorded')
    if not plane_z:
        # Fail CLOSED rather than fall back to the off-plane markers: their
        # height is not the plane's, so the fallback would hand back a
        # plausible, wrong BB position.
        raise ValueError(
            'No co-planar marker positions recorded (QTM '
            + ', '.join(str(i + 1) for i in sorted(plane_marker_indices))
            + ') — cannot place the BB x/y plane')

    # ── PRIMARY sweep-completeness gate: BB's own yaw series ────────────────
    # Before any geometry. The marker-derived span is measured around a fitted
    # centre, and on a truncated sweep that centre is the ill-conditioned thing
    # the floor exists to distrust: at QTM-realistic noise (0.1–1.5 mm) a real
    # 5° arc fits a ~3 mm circle and subtends ~330°, so a marker-only floor
    # fires on nothing on hardware while the truncated sweep dies later in the
    # axis-deviation check under an occlusion message that blames the wrong
    # subsystem. BB's reported yaw is encoder-derived and already 1-D — no fit,
    # no conditioning, so 5° reads 5.0° at any marker noise. See MIN_ARC_DEG.
    #
    # Guarded on MIN_YAW_READINGS so a too-short yaw series keeps reporting its
    # own accurate cause (calculate_yaw_offset below refuses it either way).
    yaw_span = 0.0
    if len(yaw_readings_deg) >= MIN_YAW_READINGS:
        yaw_span = angular_span_deg(yaw_readings_deg)
        if yaw_span < min_arc_deg:
            raise ValueError(
                f'ARC_SPAN_TOO_SMALL: BB yaw swept only {yaw_span:.1f}° < '
                f'{min_arc_deg:.1f}° — the sweep did not complete'
            )

    # Rotation axis
    axis_point, axis_dir, metrics = find_rotation_axis(
        marker_trajectories, min_points, min_marker_arc_deg, min_radius_mm,
        max_axis_dev_mm, min_agreeing)
    outcasts = {i for i, m in metrics.items() if m.status == 'outcast'}

    # The plane height comes from the co-planar markers the consensus kept: an
    # outcast's readings are the ones that disagreed, so its Z is not trusted
    # either. Fail closed, as above, if that leaves none.
    plane_z = [z for i in plane_marker_indices
               if i in marker_trajectories and i not in outcasts
               for z in marker_trajectories[i][:, 2]]
    if not plane_z:
        raise ValueError(
            'Every co-planar marker with data was an outcast — cannot place '
            'the BB x/y plane')
    avg_z = float(np.mean(plane_z)) + pitch_z_offset_mm

    # Intersection with Z plane
    intersection = find_axis_plane_intersection(axis_point, axis_dir, avg_z)
    if intersection is None:
        raise ValueError('Rotation axis is horizontal — cannot intersect Z plane')

    tilt_deg = float(np.degrees(np.arccos(min(abs(axis_dir[2]), 1.0))))

    if template is not None:
        if marker_frames is None or yaw_samples is None:
            raise ValueError('constellation yaw offset needs timestamped '
                             'marker_frames and yaw_samples with the template')
        est = estimate_sweep_yaw_offset(marker_frames, yaw_samples, template,
                                        yaw_source=yaw_source)
        if est.yaw_offset_std_deg > max_yaw_std_deg:
            raise ValueError(
                f'Yaw offset uncertainty too high: ±{est.yaw_offset_std_deg:.2f}° '
                f'> limit ±{max_yaw_std_deg}°')
        # The retired anchor value on the same sweep, as a diagnostic only:
        # its absence or an outcast anchor no longer refuses anything.
        anchor = None
        if (yaw_anchor_index in marker_trajectories and yaw_anchor_index not in outcasts
                and len(marker_trajectories[yaw_anchor_index]) >= min_points
                and len(yaw_readings_deg) >= MIN_YAW_READINGS):
            anchor, _ = calculate_yaw_offset(
                marker_trajectories[yaw_anchor_index], yaw_readings_deg, intersection)
        # POSITION: the body model's axis point (the template origin, posed
        # in every moving frame and averaged) is primary. On the 7-sweep bag
        # it repeats to 0.04 mm (x) / 0.18 mm (y) per sweep and lands within
        # 0.5 mm of sessions A/B; the arc fit's intersection repeats to
        # 0.04 / 0.38 mm and sits 0.9–1.4 mm south of it in y (and 1.1–2.0 mm
        # of A's stored position) — logbook 2026-10-09-bb-constellation-yaw-offset.
        # The arc fit stays as the cross-check (arc_position_mm) and still
        # supplies the tilt and every sweep-completeness gate above.
        return CalibrationResult(
            bb_position_mm=np.asarray(est.axis_point_mm, dtype=float),
            yaw_offset_rad=est.yaw_offset_rad,
            yaw_offset_std_deg=est.yaw_offset_std_deg,
            axis_direction=axis_dir,
            axis_tilt_deg=tilt_deg,
            yaw_span_deg=yaw_span,
            marker_metrics=metrics,
            yaw_method='constellation',
            yaw_estimate=est,
            anchor_yaw_offset_rad=anchor,
            arc_position_mm=intersection,
        )

    # LEGACY (no template): yaw offset anchored on one physical marker
    # (BB_YAW_ANCHOR_INDEX). Retired for production 2026-10-09 — see the
    # constellation section above for why.
    if (yaw_anchor_index not in marker_trajectories
            or len(marker_trajectories[yaw_anchor_index]) < min_points):
        raise ValueError(
            f'Insufficient yaw-anchor Marker {yaw_anchor_index + 1} data for '
            'yaw offset calculation')
    if yaw_anchor_index in outcasts:
        # Fail CLOSED. The yaw offset is this one marker's parked angle about
        # the axis; a marker whose readings put its circle centre > 3 mm off
        # the axis is biased by that order, ~1.5° at its 117 mm radius, i.e.
        # ~80 mm sideways at a 3 m throw — with no error raised downstream.
        raise ValueError(
            f'Yaw-anchor Marker {yaw_anchor_index + 1} is the outcast, so the '
            f'yaw offset cannot be estimated reliably — it is read from this '
            f'marker alone: {metrics[yaw_anchor_index].reason}')

    yaw_offset_rad, yaw_std_deg = calculate_yaw_offset(
        marker_trajectories[yaw_anchor_index],
        yaw_readings_deg,
        intersection,
    )

    if yaw_std_deg > max_yaw_std_deg:
        raise ValueError(
            f'Yaw offset uncertainty too high: ±{yaw_std_deg:.2f}° > limit ±{max_yaw_std_deg}°'
        )

    return CalibrationResult(
        bb_position_mm=intersection,
        yaw_offset_rad=yaw_offset_rad,
        yaw_offset_std_deg=yaw_std_deg,
        axis_direction=axis_dir,
        axis_tilt_deg=tilt_deg,
        yaw_span_deg=yaw_span,
        marker_metrics=metrics,
        arc_position_mm=intersection,
    )
