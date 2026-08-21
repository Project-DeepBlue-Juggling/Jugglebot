"""
Ball Butler calibration: determine BB global position and yaw offset from mocap markers.

During BB's CALIBRATING state, 5 mocap markers trace circular arcs as the yaw motor
sweeps.  After the sweep completes, this module:

0. Refuses outright if the markers never traced a real arc (``MIN_ARC_DEG``) —
   point count alone cannot tell a truncated sweep from a completed one.
1. Fits a 3D circle to each marker trajectory (plane fit via SVD + algebraic circle fit).
2. Extracts the common rotation axis (weighted average of circle normals/centers).
3. Intersects the axis with the horizontal plane at the average marker Z height
   (offset by the pitch-axis vertical offset) to get the BB global position.
4. Compares Marker 3's global angle with the reported yaw to compute yaw_offset_rad.

All functions are pure Python + numpy — no ROS2 dependency.

Lifted from the retired ball_butler_node.py (now attic/ros-jugglebot-archived/)
with minor cleanup.
"""

from __future__ import annotations

import math
import numpy as np
from dataclasses import dataclass, field
from typing import Optional


#: Minimum yaw arc (degrees) the markers must actually trace before a fit is
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
#: WHY 20°. Conservative on purpose: the partial-data failure mode is a span
#: well under 10°, so 20° rejects it with margin while staying far below any
#: plausible real sweep. ⚠ The true commanded sweep span is BB-firmware-owned
#: and lives in no file in this repo, so this is a floor chosen from the
#: failure mode, not from the command. The owner confirms or raises it from the
#: yaw span logged by the first hardware calibrate
#: (``plans/active/operator-observability.md`` § 8).
MIN_ARC_DEG = 20.0


def wrap_pi(angle: float) -> float:
    """Wrap angle to (-pi, pi]."""
    a = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return a if a != -math.pi else math.pi


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
    min_arc_deg: float = MIN_ARC_DEG,
) -> tuple[np.ndarray, np.ndarray, dict[int, MarkerFitMetrics]]:
    """Find the rotation axis from multiple marker circular trajectories.

    Args:
        marker_trajectories: marker_index → (N, 3) positions array.
        min_points: minimum samples per marker for inclusion.
        min_arc_deg: minimum arc a marker must trace to be fitted at all, and
            — via the widest marker — for the sweep to count as having
            happened. See :data:`MIN_ARC_DEG`.

    Returns:
        axis_point:   a point on the axis (mm).
        axis_direction: unit vector along the axis.
        quality_metrics: per-marker fit metrics.

    Raises:
        ValueError on insufficient data, too small an arc, or poor geometric
        consistency.
    """
    centers: list[np.ndarray] = []
    normals: list[np.ndarray] = []
    weights: list[float] = []
    quality: dict[int, MarkerFitMetrics] = {}
    widest_arc_deg = 0.0

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

        if span < min_arc_deg:
            # Individual exclusion as well as the aggregate refusal below: a
            # single marker that was occluded for most of the sweep (the rest
            # of the constellation fine) contributes a garbage centre/normal
            # that the length-weighted average would happily fold in. Dropping
            # it is the same treatment a marker with too few points already
            # got — and if that leaves fewer than two, the existing
            # "insufficient data" refusal fires.
            quality[idx] = MarkerFitMetrics(
                status='skipped',
                reason=f'arc span {span:.1f}° < {min_arc_deg:.1f}°',
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
    if widest_arc_deg < min_arc_deg and any(
            len(pts) >= min_points for pts in marker_trajectories.values()):
        raise ValueError(
            f'ARC_SPAN_TOO_SMALL: widest marker arc {widest_arc_deg:.1f}° < '
            f'{min_arc_deg:.1f}° — the yaw sweep did not complete, so the '
            'circle fits are not trustworthy'
        )

    if len(centers) < 2:
        raise ValueError(f'Calibration failed: only {len(centers)} markers had sufficient data')

    w = np.array(weights)
    w /= w.sum()
    c = np.array(centers)
    n = np.array(normals)

    axis_direction = (n * w[:, np.newaxis]).sum(axis=0)
    axis_direction /= np.linalg.norm(axis_direction)

    axis_point = (c * w[:, np.newaxis]).sum(axis=0)

    # Verify circle centres lie near the axis
    max_dev = 0.0
    for idx, m in quality.items():
        if m.status != 'ok' or m.center is None:
            continue
        v = m.center - axis_point
        perp = v - np.dot(v, axis_direction) * axis_direction
        dist = float(np.linalg.norm(perp))
        m.distance_from_axis_mm = dist
        max_dev = max(max_dev, dist)

    if max_dev > 3.0:
        raise ValueError(
            f'Circle centres deviate up to {max_dev:.2f} mm from axis. '
            'This may indicate non-rigid motion or poor marker visibility.'
        )

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
    marker3_positions: np.ndarray,
    yaw_readings_deg: list[float],
    axis_point: np.ndarray,
) -> tuple[float, float]:
    """Compute the yaw offset between BB's local frame and the global frame.

    Uses the final ~1 s of data (when BB is stationary after calibration sweep).

    Returns (yaw_offset_rad, combined_std_deg).
    Raises ValueError on insufficient data.
    """
    if len(marker3_positions) < 50:
        raise ValueError(f'Insufficient Marker 3 data: {len(marker3_positions)} points')
    if len(yaw_readings_deg) < 5:
        raise ValueError(f'Insufficient yaw readings: {len(yaw_readings_deg)}')

    # Last ~1 s of marker data (200 Hz → 200 pts)
    recent_m3 = marker3_positions[-min(200, len(marker3_positions)):]
    recent_yaw = yaw_readings_deg[-min(10, len(yaw_readings_deg)):]

    # Marker 3 angle in global frame (circular mean)
    angles = [math.atan2(p[1] - axis_point[1], p[0] - axis_point[0]) for p in recent_m3]
    sin_sum = sum(math.sin(a) for a in angles)
    cos_sum = sum(math.cos(a) for a in angles)
    avg_marker_angle = math.atan2(sin_sum, cos_sum)

    avg_yaw_rad = math.radians(sum(recent_yaw) / len(recent_yaw))

    yaw_offset_rad = wrap_pi(avg_marker_angle - avg_yaw_rad)

    # Uncertainty
    marker_var = sum((a - avg_marker_angle) ** 2 for a in angles) / len(angles)
    marker_std_deg = math.degrees(math.sqrt(marker_var))

    avg_yaw_deg = sum(recent_yaw) / len(recent_yaw)
    yaw_var = sum((y - avg_yaw_deg) ** 2 for y in recent_yaw) / len(recent_yaw)
    yaw_std_deg = math.sqrt(yaw_var)

    combined_std_deg = math.sqrt(marker_std_deg ** 2 + yaw_std_deg ** 2)

    return yaw_offset_rad, combined_std_deg


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
    marker_metrics: dict[int, MarkerFitMetrics] = field(default_factory=dict)


def run_calibration(
    calibration_data: dict[int, list[np.ndarray]],
    yaw_readings_deg: list[float],
    pitch_z_offset_mm: float,
    min_points: int = 50,
    max_yaw_std_deg: float = 5.0,
    min_arc_deg: float = MIN_ARC_DEG,
) -> CalibrationResult:
    """Execute the full calibration pipeline.

    Args:
        calibration_data:  marker_index → list of [x,y,z] arrays collected
                           during the CALIBRATING state.
        yaw_readings_deg:  yaw readings from BB heartbeat during calibration.
        pitch_z_offset_mm: vertical offset of pitch axis from yaw axis (from config).
        min_points:        minimum marker samples for circle fitting.
        max_yaw_std_deg:   maximum allowed yaw-offset uncertainty.
        min_arc_deg:       minimum yaw arc the markers must trace
                           (:data:`MIN_ARC_DEG`) — the guard against a
                           truncated sweep fitting a plausible-looking circle.

    Returns:
        CalibrationResult on success.

    Raises:
        ValueError on any calibration failure.
    """
    # Convert lists → numpy arrays, collect Z values
    marker_trajectories: dict[int, np.ndarray] = {}
    all_z: list[float] = []

    for idx, positions in calibration_data.items():
        if positions:
            arr = np.array(positions)
            marker_trajectories[idx] = arr
            all_z.extend(arr[:, 2].tolist())

    if not all_z:
        raise ValueError('No valid marker positions recorded')

    avg_z = float(np.mean(all_z)) + pitch_z_offset_mm

    # Rotation axis
    axis_point, axis_dir, metrics = find_rotation_axis(
        marker_trajectories, min_points, min_arc_deg)

    # Intersection with Z plane
    intersection = find_axis_plane_intersection(axis_point, axis_dir, avg_z)
    if intersection is None:
        raise ValueError('Rotation axis is horizontal — cannot intersect Z plane')

    tilt_deg = float(np.degrees(np.arccos(min(abs(axis_dir[2]), 1.0))))

    # Yaw offset (uses Marker 3 = index 2)
    marker3_idx = 2
    if marker3_idx not in marker_trajectories or len(marker_trajectories[marker3_idx]) < min_points:
        raise ValueError('Insufficient Marker 3 data for yaw offset calculation')

    yaw_offset_rad, yaw_std_deg = calculate_yaw_offset(
        marker_trajectories[marker3_idx],
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
        marker_metrics=metrics,
    )
