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
4. Reads yaw_offset_rad from the ORIENTATION OF THE WHOLE CONSTELLATION
   against a stored BB-local marker template, in the stationary hold(s) of
   the window, minus BB's reported yaw — no axis point involved
   (:func:`estimate_constellation_yaw_offset`, since 2026-10-09). The retired
   estimator — one anchor marker's angle about the fitted axis point
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
            f'({detail} mm; no {smallest} of {len(fitted)} agree within '
            f'{max_axis_dev_mm:.1f} mm). '
            'This may indicate non-rigid motion or poor marker visibility.'
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
    recent_yaw = yaw_readings_deg[-min(10, len(yaw_readings_deg)):]

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
#  Yaw offset from the whole marker constellation (2026-10-09)
# ---------------------------------------------------------------------------
#
# WHY THIS REPLACED THE ANCHOR. ``calculate_yaw_offset`` reads ONE marker's
# angle about the axis point the sweep fitted. That marker sits 117.6 mm from
# the axis, so the offset moves 0.487° per mm of axis-point error, and the
# sweep's axis point is good to ~1–2 mm at best (the consensus gate above
# tolerates 3 mm). Two calibrations on 2026-10-09 with BB untouched stored
# 0.208° and 0.681°, and every throw of the second session was misaimed by
# −8 mm at 1 m (investigation:
# ~/bb_calibration_sessions/yaw_offset_investigation_20261009/REPORT.md).
#
# The estimator below needs no axis point. In each stationary hold it finds
# the yaw-stage markers by GEOMETRY (not by QTM label: labels were reshuffled
# on 2026-09-27 and QTM swaps them under occlusion), fits the rotation about
# world z that carries a stored BB-local template onto them (2D Procrustes on
# x/y, z used only to tell markers apart), subtracts BB's reported yaw and
# averages over holds. Numbers (old vs new per sweep, the gauge pin, the
# cross-session check): logbook ``2026-10-09-bb-constellation-yaw-offset``.
#
# WHICH MARKERS. The template holds the four CO-PLANAR yaw-stage markers only.
# The two off-plane markers (QTM 1 and 2, ~28 mm lower) distort at the parked
# yaw-0 pose — the pose the sweep's pause is at — by up to ~1.5 mm, and pulled
# single-sweep offsets about by ±0.45°; QTM 1 is also the marker whose parked
# readings crept ~3 mm on 2026-10-05/06 (MIN_AGREEING_MARKERS above).
#
# WHY 2D ABOUT WORLD z, NOT A 3D FIT IN THE AXIS FRAME. A 3D fit needs the
# axis direction, i.e. the sweep's tilt estimate, which is itself unstable
# (0.76° vs 1.15° stored for the same mounting; 0.4° from whole-session
# fits). Ignoring a tilt τ biases the 2D rotation by at most
# τ·|Σ(hᵢ−h̄)pᵢ|/Σ|pᵢ|² (markers at different heights shift differently),
# which for the shipped template and τ ≤ 1.2° is small — the template file's
# ``provenance.tilt_coupling_deg_per_deg`` holds the factor.

#: Fewest template markers a hold must match, or the hold is not used. Three
#: is the fewest for which a rigid 2D fit has a residual at all (two points
#: always fit exactly, so a mismatch could not be seen).
CONSTELLATION_MIN_MATCHED = 3

#: A hold point within this distance (mm, 3D) of a transformed template
#: marker is that marker. Hold medians scatter 0.2–1.0 mm about the template
#: (session A, 611 holds) and the closest pair of template markers is ~59 mm
#: apart (the off-plane QTM 1/2, ~30 mm from their neighbours, are not in the
#: template and so are just spurious points), so a stray point cannot steal a
#: match unless it sits on a marker's position.
CONSTELLATION_MATCH_TOL_MM = 3.0

#: Ceiling on a hold's template-fit RMS residual (mm, x/y). A hold above it
#: is not silently dropped: the whole estimate FAILS, because a residual this
#: large means the template no longer describes the markers (a marker moved
#: on BB, or a wrong point set was matched). Set above every hold of the
#: three 2026-10-09 bags with the shipped template — max 0.73 mm over 489
#: holds of session A, 0.80 mm over 204 of B, 0.42 mm over the 7 sweep
#: pauses — and low enough that one marker knocked ~2 mm trips it.
CONSTELLATION_MAX_RESIDUAL_MM = 1.0

#: Stationary-hold detection on BB's reported yaw series (heartbeat, 10 Hz):
#: a run of samples within ±HOLD_YAW_TOL_DEG of its first sample, at least
#: HOLD_MIN_S long, with marker frames taken from HOLD_TRIM_START_S after its
#: first sample to HOLD_TRIM_END_S before its last. The encoder dithers by
#: ±0.05°; the sweep's 0° pause settles by ~0.6° over its first ~0.6 s
#: (2026-10-09 bag), which this excludes. Both trims exceed the heartbeat's
#: lag behind the mocap frames, measured ~80 ms on the same bag (the moving
#: sweep reads ±7° at ~90°/s): the last stationary heartbeat describes BB
#: ~80 ms earlier, so frames up to it could already contain the next move.
HOLD_YAW_TOL_DEG = 0.15
HOLD_MIN_S = 0.8
HOLD_TRIM_START_S = 0.15
HOLD_TRIM_END_S = 0.15
#: Heartbeat gap (s) that breaks a hold: a missing heartbeat is not evidence
#: BB stayed still.
HOLD_MAX_GAP_S = 0.3
#: Consecutive holds closer than this in yaw (deg), with no heartbeat gap
#: between them, are ONE visit to a pose: the yaw crept past
#: HOLD_YAW_TOL_DEG while settling (2026-10-09 bag, sweep 6: 0.4° over 2 s).
#: They are averaged into one draw, because the uncertainty treats holds as
#: independent draws of BB's pointing and two halves of one pause are not.
VISIT_MAX_YAW_STEP_DEG = 1.0
#: A hold point must be present in this fraction of the hold's frames.
HOLD_MIN_PRESENCE = 0.6
#: Clustering radius (mm) when collapsing a hold's frames to marker medians.
HOLD_CLUSTER_RADIUS_MM = 3.0

#: Floor (deg) on the per-hold scatter used for the uncertainty when there are
#: too few holds to measure it (fewer than :data:`HOLD_SCATTER_MIN_N`). Measured
#: 2026-10-09 over 611 (A) and 227 (B) holds: 0.12–0.15° — BB's real pointing
#: relative to its reported yaw varies by this much from one hold to the next,
#: so a single hold cannot be better than this however many frames it has.
HOLD_SCATTER_FLOOR_DEG = 0.13
HOLD_SCATTER_MIN_N = 5

#: Consistency gate (see :func:`check_yaw_offset_consistency`): a new offset
#: more than max(GATE_N_SIGMA·σ, GATE_MIN_DEG) from the last accepted one is
#: refused unless the operator declares that BB moved.
GATE_N_SIGMA = 3.0
GATE_MIN_DEG = 0.15


@dataclass
class MarkerTemplate:
    """BB-local yaw-stage marker positions (mm) and the gauge they carry.

    ``points_mm`` (K, 3): x/y in BB's local frame (x = BB yaw 0, rotation
    about world z), z = world height. The rotation of this template IS the
    yaw-offset gauge: rotating it by δ shifts every estimate by −δ.
    ``reference_yaw_offset_deg`` / ``reference_std_deg`` are what the gauge was
    pinned to (the consistency gate's reference when no calibration has been
    accepted yet).
    """
    points_mm: np.ndarray
    names: tuple = ()
    reference_yaw_offset_deg: Optional[float] = None
    reference_std_deg: Optional[float] = None
    source: str = ''


def load_marker_template(path: str) -> MarkerTemplate:
    """Load ``resources/bb_marker_template.json``. Raises ValueError if malformed."""
    import json
    with open(path, 'r') as f:
        data = json.load(f)
    try:
        markers = data['markers']
        pts = np.array([m['xyz_mm'] for m in markers], dtype=float)
        names = tuple(str(m.get('id', i)) for i, m in enumerate(markers))
    except (KeyError, TypeError) as e:
        raise ValueError(f'malformed BB marker template {path}: {e}')
    if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] < CONSTELLATION_MIN_MATCHED:
        raise ValueError(f'BB marker template {path} needs >= '
                         f'{CONSTELLATION_MIN_MATCHED} markers of xyz, got {pts.shape}')
    if not np.all(np.isfinite(pts)):
        raise ValueError(f'BB marker template {path} has non-finite coordinates')
    gauge = data.get('gauge', {})
    ref = gauge.get('pinned_yaw_offset_deg')
    ref_sd = gauge.get('pinned_std_deg')
    return MarkerTemplate(
        points_mm=pts, names=names,
        reference_yaw_offset_deg=None if ref is None else float(ref),
        reference_std_deg=None if ref_sd is None else float(ref_sd),
        source=path)


def find_stationary_holds(yaw_samples, yaw_tol_deg: float = HOLD_YAW_TOL_DEG,
                          min_hold_s: float = HOLD_MIN_S,
                          max_gap_s: float = HOLD_MAX_GAP_S):
    """Stationary holds in a timestamped yaw series.

    ``yaw_samples``: sequence of (t_s, yaw_deg), time-ordered. Returns a list of
    (t_first, t_last, yaw_deg_mean) for each maximal run of samples within
    ±``yaw_tol_deg`` of the run's first sample (circularly), with no gap above
    ``max_gap_s``, lasting at least ``min_hold_s``. Deterministic, greedy, left
    to right.
    """
    s = np.asarray(yaw_samples, dtype=float).reshape(-1, 2)
    holds = []
    n = len(s)
    i = 0
    while i < n:
        j = i
        while (j + 1 < n
               and abs(math.degrees(wrap_pi(math.radians(s[j + 1, 1] - s[i, 1])))) <= yaw_tol_deg
               and s[j + 1, 0] - s[j, 0] <= max_gap_s):
            j += 1
        if s[j, 0] - s[i, 0] >= min_hold_s:
            d = np.radians(s[i:j + 1, 1] - s[i, 1])
            mean = s[i, 1] + math.degrees(math.atan2(np.sin(d).mean(), np.cos(d).mean()))
            holds.append((float(s[i, 0]), float(s[j, 0]), float(mean)))
        i = j + 1
    return holds


def cluster_hold_points(frames, min_presence: float = HOLD_MIN_PRESENCE,
                        radius_mm: float = HOLD_CLUSTER_RADIUS_MM) -> np.ndarray:
    """Collapse a stationary hold's frames to one median point per marker.

    ``frames``: list of (n_i, 3) arrays (any marker set per frame, no labels).
    A cluster is kept if it has points in at least ``min_presence`` of the
    frames; a marker that flickers in and out, or a reflection, is dropped.
    Returns (M, 3). Deterministic: seeds are taken in frame order.
    """
    frames = [np.asarray(f, dtype=float).reshape(-1, 3) for f in frames]
    frames = [f[np.all(np.isfinite(f), axis=1)] for f in frames]
    nfr = len(frames)
    if nfr == 0:
        return np.empty((0, 3))
    fid = np.concatenate([np.full(len(f), k) for k, f in enumerate(frames)]) if nfr else np.empty(0)
    pts = np.concatenate(frames) if nfr else np.empty((0, 3))
    used = np.zeros(len(pts), bool)
    out = []
    for k in range(len(pts)):
        if used[k]:
            continue
        c = pts[k]
        for _ in range(3):
            m = (~used) & (np.linalg.norm(pts - c, axis=1) < radius_mm)
            c = np.median(pts[m], axis=0)
        m = (~used) & (np.linalg.norm(pts - c, axis=1) < radius_mm)
        used |= m
        if len(np.unique(fid[m])) >= min_presence * nfr:
            out.append(c)
    return np.array(out).reshape(-1, 3)


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


@dataclass
class TemplateMatch:
    """A label-free match of hold points to the template."""
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
                   tol_mm: float = CONSTELLATION_MATCH_TOL_MM) -> Optional[TemplateMatch]:
    """Find which hold points are which template markers, by geometry alone.

    Every (point pair, template pair) whose 3D separations agree within
    ``tol_mm`` (and whose height difference agrees: the yaw stage only turns
    about ~vertical, so Δz is invariant) proposes a rotation about z plus a
    translation; the proposal matching the most markers (then the smallest
    residual) wins and is refined by Procrustes on its matches. Labels play no
    part, so a relabelled, missing or spurious marker changes only which
    points are used. Returns None when fewer than
    :data:`CONSTELLATION_MIN_MATCHED` markers match.
    """
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
                    if abs((P[j, 2] - P[i, 2]) - (T[b, 2] - T[a, 2])) > tol_mm:
                        continue
                    vt = T[b, :2] - T[a, :2]
                    th = math.atan2(vt[0] * vo[1] - vt[1] * vo[0], vt[0] * vo[0] + vt[1] * vo[1])
                    R = _rot2(th)
                    txy = P[i, :2] - R @ T[a, :2]
                    tz = P[i, 2] - T[a, 2]
                    Tw = np.c_[T[:, :2] @ R.T + txy, T[:, 2] + tz]
                    pairs = _assign(Tw, P, tol_mm)
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
        new_pairs = _assign(Tw, P, tol_mm)
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
class HoldEstimate:
    """One stationary hold's contribution to the yaw offset."""
    t_start: float
    t_end: float
    yaw_deg: float              # BB's reported yaw (mean over the hold)
    offset_rad: float           # θ(template → world) − reported yaw
    n_matched: int
    rms_mm: float
    lever_mm: float
    n_frames: int
    visit: int = 0              # holds sharing a visit are one draw (VISIT_MAX_YAW_STEP_DEG)


@dataclass
class ConstellationYawEstimate:
    """Yaw offset from the whole constellation, with a real uncertainty."""
    yaw_offset_rad: float
    yaw_offset_std_deg: float   # total: hold statistics ⊕ template-fit term
    stat_std_deg: float         # hold-to-hold SD/√N (floored for small N)
    template_std_deg: float     # template-fit residual term (does not average down)
    hold_sd_deg: float          # measured SD of the per-visit offsets (0 if N == 1)
    holds: list                 # [HoldEstimate]
    n_holds_rejected: int = 0   # holds with too few matched markers
    visit_offsets_rad: list = field(default_factory=list)  # one per visit: the N averaged

    @property
    def n_holds(self) -> int:
        """N of the σ: independent pose visits (see VISIT_MAX_YAW_STEP_DEG)."""
        return len(self.visit_offsets_rad)


def estimate_constellation_yaw_offset(
    marker_frames,
    yaw_samples,
    template,
    min_matched: int = CONSTELLATION_MIN_MATCHED,
    max_residual_mm: float = CONSTELLATION_MAX_RESIDUAL_MM,
    match_tol_mm: float = CONSTELLATION_MATCH_TOL_MM,
    hold_scatter_floor_deg: float = HOLD_SCATTER_FLOOR_DEG,
    holds=None,
) -> ConstellationYawEstimate:
    """BB's yaw offset from every qualifying stationary hold of a window.

    Args:
        marker_frames: sequence of (t_s, (n, 3) array) — every BB marker point
            of each mocap frame, labels discarded. Same clock as yaw_samples.
        yaw_samples: sequence of (t_s, yaw_deg) — BB's reported yaw.
        template: :class:`MarkerTemplate` or a (K, 3) array.
        holds: optional explicit [(t_first, t_last, yaw_deg)], else
            :func:`find_stationary_holds` on ``yaw_samples``.

    Per hold: frames from HOLD_TRIM_START_S after the first stationary
    heartbeat to HOLD_TRIM_END_S before the last → per-marker medians →
    :func:`match_template` → offset = θ − reported yaw. The holds are then
    circular-averaged with equal weight (each is one draw of BB's pointing vs
    its reported yaw; frames within a hold are not independent draws of that).

    Uncertainty (deg): σ = √(σ_stat² + σ_tmpl²), with
    σ_stat = s/√N over N independent pose visits, s the per-visit SD — but
    never below ``hold_scatter_floor_deg`` while N < HOLD_SCATTER_MIN_N,
    because a handful of visits cannot measure their own scatter and the
    measured value is 0.13°; σ_tmpl = mean over holds of σ_c/lever, the
    template-fit term: σ_c = RMS·√(n/(2n−3)) is the per-coordinate residual
    SD of an n-marker rigid 2D fit, and σ_c/lever is the rotation uncertainty
    it implies. σ_tmpl is systematic at a pose, so it is not divided by √N.
    σ_stat alone is the REPEATABILITY at a pose — what the consistency gate
    compares (:func:`check_yaw_offset_consistency`): σ_tmpl is common to two
    calibrations taken at the same pose and cancels in their difference.

    Raises ValueError (loudly, with the cause) when no hold qualifies, or any
    matched hold's residual exceeds ``max_residual_mm``.
    """
    T = template.points_mm if isinstance(template, MarkerTemplate) else np.asarray(template, float)
    ys = sorted(((float(t), float(y)) for t, y in yaw_samples), key=lambda r: r[0])
    if holds is None:
        holds = find_stationary_holds(ys)
    if not holds:
        raise ValueError(
            'CONSTELLATION_NO_HOLD: BB never held still for '
            f'{HOLD_MIN_S:.1f} s (yaw within ±{HOLD_YAW_TOL_DEG:.2f}°) in the '
            'calibration window — no stationary pose to read the yaw offset from')
    frames = sorted(((float(t), f) for t, f in marker_frames), key=lambda r: r[0])
    ft = np.array([t for t, _ in frames]) if frames else np.empty(0)

    estimates = []
    rejected = []
    visit = -1
    prev = None
    for (t0, t1, yaw) in holds:
        if (prev is None or t0 - prev[1] > HOLD_MAX_GAP_S + 1e-9
                or abs(math.degrees(wrap_pi(math.radians(yaw - prev[2])))) > VISIT_MAX_YAW_STEP_DEG):
            visit += 1
        prev = (t0, t1, yaw)
        a, b = t0 + HOLD_TRIM_START_S, t1 - HOLD_TRIM_END_S
        i0, i1 = np.searchsorted(ft, [a, b], side='left')
        window = [frames[k][1] for k in range(int(i0), int(i1))]
        pts = cluster_hold_points(window)
        m = match_template(pts, T, match_tol_mm) if len(window) else None
        if m is None or len(m.pairs) < min_matched:
            rejected.append((t0, t1, yaw, 0 if m is None else len(m.pairs), len(window)))
            continue
        if m.rms_mm > max_residual_mm:
            raise ValueError(
                f'CONSTELLATION_RESIDUAL: the hold at yaw {yaw:.2f}° fits the BB '
                f'marker template with {m.rms_mm:.2f} mm RMS > {max_residual_mm:.2f} mm '
                f'({len(m.pairs)} markers matched) — the template no longer '
                'describes BB\'s markers (a marker moved, or a wrong point set '
                'was matched); refusing rather than guessing')
        estimates.append(HoldEstimate(
            t_start=t0, t_end=t1, yaw_deg=yaw,
            offset_rad=wrap_pi(m.theta_rad - math.radians(yaw)),
            n_matched=len(m.pairs), rms_mm=m.rms_mm, lever_mm=m.lever_mm,
            n_frames=len(window), visit=visit))
    if not estimates:
        detail = ', '.join(f'yaw {y:.1f}°: {n} matched of {len(T)} ({nf} frames)'
                           for _, _, y, n, nf in rejected)
        raise ValueError(
            f'CONSTELLATION_TOO_FEW_MARKERS: no stationary hold matched '
            f'{min_matched} or more of the {len(T)} template markers ({detail})')

    # One draw per visit (frame-weighted circular mean of its holds), then an
    # equal-weight circular mean over visits.
    phi = []
    for v in sorted({h.visit for h in estimates}):
        hs = [h for h in estimates if h.visit == v]
        w = np.array([h.n_frames for h in hs], dtype=float)
        a = np.array([h.offset_rad for h in hs])
        phi.append(math.atan2(float((w * np.sin(a)).sum()), float((w * np.cos(a)).sum())))
    phi = np.array(phi)
    mean = math.atan2(float(np.sin(phi).mean()), float(np.cos(phi).mean()))
    dev = np.degrees(np.array([wrap_pi(p - mean) for p in phi]))
    n = len(phi)
    hold_sd = float(np.std(dev, ddof=1)) if n > 1 else 0.0
    s = hold_sd if n >= HOLD_SCATTER_MIN_N else max(hold_sd, hold_scatter_floor_deg)
    stat = s / math.sqrt(n)
    tmpl = float(np.mean([
        math.degrees(h.rms_mm * math.sqrt(h.n_matched / (2.0 * h.n_matched - 3.0)) / h.lever_mm)
        for h in estimates]))
    return ConstellationYawEstimate(
        yaw_offset_rad=mean,
        yaw_offset_std_deg=math.sqrt(stat ** 2 + tmpl ** 2),
        stat_std_deg=stat, template_std_deg=tmpl, hold_sd_deg=hold_sd,
        holds=estimates, n_holds_rejected=len(rejected),
        visit_offsets_rad=[float(p) for p in phi])


@dataclass
class GateVerdict:
    accepted: bool
    message: str
    delta_deg: float = 0.0
    threshold_deg: float = 0.0


def check_yaw_offset_consistency(
    new_offset_deg: float,
    new_std_deg: float,
    reference_offset_deg: Optional[float],
    bb_moved: bool = False,
    reference_label: str = 'last accepted',
    n_sigma: float = GATE_N_SIGMA,
    min_deg: float = GATE_MIN_DEG,
) -> GateVerdict:
    """Refuse a yaw offset that disagrees with the reference while BB is unmoved.

    Threshold max(n_sigma·σ_new, min_deg), σ_new the REPEATABILITY of the new
    estimate (``ConstellationYawEstimate.stat_std_deg``: 0.13° for one hold,
    so ±0.39°; it shrinks as √N once there are several holds). The template
    term of the published σ is left out on purpose: it is systematic at a pose
    and common to two calibrations taken at the same pose, so it cancels in
    their difference. The reference is the last accepted
    calibration (or, before any, the template's pinned gauge). The aim
    correction is only valid in the frame it was fitted in, so a calibration
    that moves the frame without BB having moved misaims every throw (the
    0.47° of 2026-10-09); ``bb_moved`` is the operator's explicit statement
    that a change is real, and then the new value is accepted (and the affine
    must be refitted).
    """
    if reference_offset_deg is None:
        return GateVerdict(True, 'gate: no reference (first calibration)')
    delta = math.degrees(wrap_pi(math.radians(new_offset_deg - reference_offset_deg)))
    thr = max(n_sigma * new_std_deg, min_deg)
    if abs(delta) <= thr:
        return GateVerdict(True, f'gate: {delta:+.3f}° vs {reference_label} '
                                 f'{reference_offset_deg:.3f}° (limit ±{thr:.3f}°) ok',
                           delta, thr)
    if bb_moved:
        return GateVerdict(True, f'gate: {delta:+.3f}° vs {reference_label} '
                                 f'{reference_offset_deg:.3f}° exceeds ±{thr:.3f}° — '
                                 'ACCEPTED on bb_moved override (refit the aim correction)',
                           delta, thr)
    return GateVerdict(
        False,
        f'YAW_OFFSET_INCONSISTENT: new yaw offset {new_offset_deg:.3f}° ±{new_std_deg:.3f}° '
        f'differs from the {reference_label} {reference_offset_deg:.3f}° by {delta:+.3f}° '
        f'(limit ±{thr:.3f}°) and BB is not declared moved — refused. If BB really '
        'moved, set the mocap_node parameter bb_moved:=true and recalibrate, then '
        'refit the aim correction',
        delta, thr)


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
    #: production path, :func:`estimate_constellation_yaw_offset`) or
    #: ``'anchor'`` (the retired single-marker estimator, only when the caller
    #: passed no constellation inputs — sweep-fit fixtures and diagnostics).
    #: ``mocap_node`` refuses to publish anything but ``'constellation'``.
    yaw_method: str = 'anchor'
    #: The constellation estimate (holds, σ breakdown); None on the anchor path.
    yaw_estimate: Optional[ConstellationYawEstimate] = None
    #: The retired anchor estimator's value on the same sweep (rad), kept as a
    #: logged diagnostic on the constellation path; None if it could not be
    #: computed (anchor missing or an outcast — no longer a refusal).
    anchor_yaw_offset_rad: Optional[float] = None


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
) -> CalibrationResult:
    """Execute the full calibration pipeline.

    POSITION, tilt and the sweep gates always come from the sweep's arc fit.
    The YAW OFFSET comes from :func:`estimate_constellation_yaw_offset` when
    ``template`` is given (with ``marker_frames`` and ``yaw_samples``, both
    timestamped on one clock) — the production path since 2026-10-09. Without
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
        est = estimate_constellation_yaw_offset(marker_frames, yaw_samples, template)
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
        return CalibrationResult(
            bb_position_mm=intersection,
            yaw_offset_rad=est.yaw_offset_rad,
            yaw_offset_std_deg=est.yaw_offset_std_deg,
            axis_direction=axis_dir,
            axis_tilt_deg=tilt_deg,
            yaw_span_deg=yaw_span,
            marker_metrics=metrics,
            yaw_method='constellation',
            yaw_estimate=est,
            anchor_yaw_offset_rad=anchor,
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
    )
