"""BB calibration with the 7-marker constellation (2026-09-27).

Two markers (QTM ``Ball Butler - 1`` and ``- 2``) were added to keep BB tracked
through the sweep. They are NOT on the plane the original five (now QTM 3–7)
share, which fixes three rules in ``bb_calibration.run_calibration``:

* every marker feeds the rotation-AXIS fit (a marker's height does not matter
  to where its circle's centre lies on the axis);
* only the co-planar markers (``BB_PLANE_MARKER_INDICES``) set the Z of BB's
  x/y plane — folding the off-plane pair in shifts the plane, and through any
  axis tilt, the reported XY;
* the yaw offset is anchored on QTM 4 (``BB_YAW_ANCHOR_INDEX``, owner) — a
  different physical marker shifts the offset by the angle between the two,
  silently misaiming every throw.

Each test below is built so the OLD rule (all-marker Z average, anchor index 2)
would fail it, not merely so the new one passes.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from jugglebot.bb_calibration import (
    BB_MARKER_COUNT,
    BB_PLANE_MARKER_INDICES,
    BB_YAW_ANCHOR_INDEX,
    find_axis_plane_intersection,
    run_calibration,
    wrap_pi,
)


BB_POS = np.array([-707.0, -149.0, 1724.0])
PITCH_Z_OFFSET_MM = 17.5
YAW_OFFSET_RAD = -0.053
#: Indices 0–1 are the off-plane QTM 1–2; indices 2–6 share one plane.
RADII = (100.0, 105.0, 80.0, 95.0, 110.0, 90.0, 75.0)
Z_OFFSETS = (60.0, 45.0, 0.0, 0.0, 0.0, 0.0, 0.0)
#: Per-marker angular position on BB (rad). Distinct, so the yaw offset read
#: from the wrong marker is visibly wrong.
PHASES = tuple(math.radians(d) for d in (200.0, 250.0, 0.0, 72.0, 144.0, 216.0, 288.0))


def _axis(tilt_deg):
    t = math.radians(tilt_deg)
    return np.array([math.sin(t), 0.0, math.cos(t)])


def _basis(axis):
    ref = np.array([0.0, 1.0, 0.0])
    u = np.cross(ref, axis)
    u /= np.linalg.norm(u)
    return u, np.cross(axis, u)


def _dataset(axis, span_deg=120.0, n_sweep=200, n_hold=250):
    """Every marker sweeps *span_deg* about *axis* through BB_POS, then holds."""
    u, v = _basis(axis)
    data = {}
    for i in range(BB_MARKER_COUNT):
        centre = BB_POS + Z_OFFSETS[i] * axis
        ang = PHASES[i] + np.linspace(0.0, math.radians(span_deg), n_sweep)
        sweep = [centre + RADII[i] * (math.cos(a) * u + math.sin(a) * v) for a in ang]
        data[i] = sweep + [sweep[-1]] * n_hold
    return data


def _plane_z(data):
    return float(np.mean([p[2] for i in BB_PLANE_MARKER_INDICES for p in data[i]]))


def _yaw_readings(data, anchor, origin, span_deg=120.0):
    """BB's yaw series such that anchor-marker angle − yaw = YAW_OFFSET_RAD."""
    end = np.array(data[anchor])[-1]
    end_deg = math.degrees(math.atan2(end[1] - origin[1], end[0] - origin[0])
                           - YAW_OFFSET_RAD)
    return list(np.linspace(end_deg - span_deg, end_deg, 40)) + [end_deg] * 20


def test_constellation_constants():
    """QTM 3–7 are the plane, QTM 4 the anchor — zero-based throughout."""
    assert BB_MARKER_COUNT == 7
    assert tuple(BB_PLANE_MARKER_INDICES) == (2, 3, 4, 5, 6)
    assert BB_YAW_ANCHOR_INDEX == 3
    assert BB_YAW_ANCHOR_INDEX in BB_PLANE_MARKER_INDICES


def test_plane_z_ignores_off_plane_markers():
    """Vertical axis: the reported Z is the co-planar markers' Z + pitch
    offset, exactly. The all-marker average would sit (60+45)/7 = 15 mm high."""
    axis = _axis(0.0)
    data = _dataset(axis)
    res = run_calibration(data, _yaw_readings(data, BB_YAW_ANCHOR_INDEX, BB_POS),
                          pitch_z_offset_mm=PITCH_Z_OFFSET_MM)
    expected_z = BB_POS[2] + PITCH_Z_OFFSET_MM
    assert res.bb_position_mm[2] == pytest.approx(expected_z, abs=0.05)
    all_marker_z = float(np.mean([p[2] for pts in data.values() for p in pts]))
    assert abs(all_marker_z + PITCH_Z_OFFSET_MM - expected_z) > 10.0  # discriminates


def test_tilted_axis_xy_follows_coplanar_plane():
    """With a 5° tilt the plane height moves the XY; the result must match the
    intersection at the CO-PLANAR plane, and must NOT match the all-marker one."""
    axis = _axis(5.0)
    data = _dataset(axis)
    plane = find_axis_plane_intersection(BB_POS, axis, _plane_z(data) + PITCH_Z_OFFSET_MM)
    res = run_calibration(data, _yaw_readings(data, BB_YAW_ANCHOR_INDEX, plane),
                          pitch_z_offset_mm=PITCH_Z_OFFSET_MM)
    assert np.linalg.norm(res.bb_position_mm[:2] - plane[:2]) < 0.1

    all_z = float(np.mean([p[2] for pts in data.values() for p in pts]))
    old = find_axis_plane_intersection(BB_POS, axis, all_z + PITCH_Z_OFFSET_MM)
    assert np.linalg.norm(old[:2] - plane[:2]) > 1.0  # the old rule is distinguishable


def test_off_plane_markers_still_feed_the_axis_fit():
    axis = _axis(0.0)
    data = _dataset(axis)
    res = run_calibration(data, _yaw_readings(data, BB_YAW_ANCHOR_INDEX, BB_POS),
                          pitch_z_offset_mm=PITCH_Z_OFFSET_MM)
    for i in range(BB_MARKER_COUNT):
        assert res.marker_metrics[i].status == 'ok', (i, res.marker_metrics[i].reason)


def test_axis_fit_works_from_off_plane_markers_alone():
    """With only QTM 1–2 plus one co-planar marker visible, the axis still fits
    and the Z plane still comes from the co-planar one."""
    axis = _axis(0.0)
    data = _dataset(axis)
    for i in (2, 4, 5, 6):
        data[i] = []
    res = run_calibration(data, _yaw_readings(data, BB_YAW_ANCHOR_INDEX, BB_POS),
                          pitch_z_offset_mm=PITCH_Z_OFFSET_MM)
    assert res.bb_position_mm[2] == pytest.approx(BB_POS[2] + PITCH_Z_OFFSET_MM, abs=0.05)
    assert np.linalg.norm(res.bb_position_mm[:2] - BB_POS[:2]) < 0.1


def test_yaw_offset_is_anchored_on_qtm_4():
    """Markers sit 72° apart, so reading the old index 2 would be 72° off."""
    axis = _axis(0.0)
    data = _dataset(axis)
    res = run_calibration(data, _yaw_readings(data, BB_YAW_ANCHOR_INDEX, BB_POS),
                          pitch_z_offset_mm=PITCH_Z_OFFSET_MM)
    assert abs(wrap_pi(res.yaw_offset_rad - YAW_OFFSET_RAD)) < 1e-3


def test_missing_anchor_refuses_by_name():
    axis = _axis(0.0)
    data = _dataset(axis)
    yaw = _yaw_readings(data, BB_YAW_ANCHOR_INDEX, BB_POS)
    data[BB_YAW_ANCHOR_INDEX] = []
    with pytest.raises(ValueError, match='yaw-anchor Marker 4'):
        run_calibration(data, yaw, pitch_z_offset_mm=PITCH_Z_OFFSET_MM)


def test_no_coplanar_data_fails_closed():
    """Only the off-plane pair recorded: refuse, never fall back to their Z."""
    axis = _axis(0.0)
    data = _dataset(axis)
    yaw = _yaw_readings(data, BB_YAW_ANCHOR_INDEX, BB_POS)
    for i in BB_PLANE_MARKER_INDICES:
        data[i] = []
    with pytest.raises(ValueError, match='No co-planar marker positions'):
        run_calibration(data, yaw, pitch_z_offset_mm=PITCH_Z_OFFSET_MM)
