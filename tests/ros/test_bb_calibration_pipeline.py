"""Ball Butler calibration pipeline checks (ported from the retired script).

These are checks 1-14 of the former standalone script
``ros_ws/src/jugglebot/jugglebot/tests/test_bb_calibration.py`` (BUG-115, TKT-84
option 1), which no test run collected. Its checks 15-17 (arc-span floor,
marker-radius floor, branch-cut yaw uncertainty) were already pinned in
``test_bb_calibration_arc_span.py``. Synthetic data only: markers on exact
circles about a known axis, then a hold at the final pose; run_calibration must
recover position, tilt and yaw offset, or refuse by name.
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
    fit_circle_3d,
    run_calibration,
    wrap_pi,
)

TRUE_POS = np.array([-707.0, -149.0, 1724.0])
TRUE_AXIS = np.array([0.0, 0.0, 1.0])
TRUE_YAW_OFFSET = -0.053
PITCH_Z_OFFSET = 17.5
# QTM 1-2 (indices 0-1) sit off the plane QTM 3-7 share.
RADII = [100.0, 105.0, 80.0, 95.0, 110.0, 90.0, 75.0]
Z_OFFSETS = [60.0, 45.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def _circle(center, axis, radius, n, angle_range=(0.0, 2 * math.pi),
            noise_std=0.0, rng=None):
    rng = rng if rng is not None else np.random.default_rng(42)
    axis = axis / np.linalg.norm(axis)
    ref = np.array([1.0, 0.0, 0.0]) if abs(axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(axis, ref)
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    ang = np.linspace(angle_range[0], angle_range[1], n, endpoint=False)
    pts = np.array([center + radius * (math.cos(a) * u + math.sin(a) * v) for a in ang])
    if noise_std > 0:
        pts += rng.normal(0, noise_std, pts.shape)
    return pts


def _dataset(pos=TRUE_POS, axis=TRUE_AXIS, n_sweep=200, n_hold=250,
             noise_std=0.0, rng=None):
    rng = rng if rng is not None else np.random.default_rng(42)
    axis = axis / np.linalg.norm(axis)
    data = {}
    for i, r in enumerate(RADII):
        sweep = _circle(pos + Z_OFFSETS[i] * axis, axis, r, n_sweep,
                        angle_range=(0.0, math.pi), noise_std=noise_std, rng=rng)
        hold = np.tile(sweep[-1], (n_hold, 1))
        if noise_std > 0:
            hold = hold + rng.normal(0, noise_std, hold.shape)
        data[i] = [p for p in np.vstack([sweep, hold])]
    return data


def _yaw(end_deg, span_deg=180.0, n_sweep=40, n_hold=20):
    """A series that SWEEPS then holds (a constant list is the failure case)."""
    return list(np.linspace(end_deg - span_deg, end_deg, n_sweep)) + [end_deg] * n_hold


def _yaw_for(data, offset, origin=TRUE_POS):
    m = np.array(data[BB_YAW_ANCHOR_INDEX])[-1]
    return _yaw(math.degrees(math.atan2(m[1] - origin[1], m[0] - origin[0]) - offset))


def test_perfect_circle_recovery():
    data = _dataset()
    r = run_calibration(data, _yaw_for(data, TRUE_YAW_OFFSET),
                        pitch_z_offset_mm=PITCH_Z_OFFSET)
    assert np.linalg.norm(r.bb_position_mm[:2] - TRUE_POS[:2]) < 0.1
    assert r.axis_tilt_deg < 0.1
    assert abs(wrap_pi(r.yaw_offset_rad - TRUE_YAW_OFFSET)) < 0.01
    assert r.yaw_offset_std_deg < 1.0


def test_noisy_data_within_tolerance():
    data = _dataset(n_sweep=300, noise_std=0.5, rng=np.random.default_rng(99))
    clean = _dataset()
    r = run_calibration(data, _yaw_for(clean, TRUE_YAW_OFFSET),
                        pitch_z_offset_mm=PITCH_Z_OFFSET)
    assert np.linalg.norm(r.bb_position_mm[:2] - TRUE_POS[:2]) < 2.0
    assert r.axis_tilt_deg < 2.0
    assert abs(wrap_pi(r.yaw_offset_rad - TRUE_YAW_OFFSET)) < 0.1


@pytest.mark.parametrize('offset', [-math.pi + 0.1, -1.5, 0.0, 1.5, math.pi - 0.1,
                                    -math.pi + 0.01, math.pi - 0.01])
def test_yaw_offset_recovered_including_near_the_wrap(offset):
    data = _dataset()
    r = run_calibration(data, _yaw_for(data, offset), pitch_z_offset_mm=PITCH_Z_OFFSET)
    assert abs(wrap_pi(r.yaw_offset_rad - offset)) < 0.01


def test_all_markers_below_min_points_refused():
    sparse = {i: [np.array([0.0, 0.0, 0.0])] * 10 for i in range(BB_MARKER_COUNT)}
    with pytest.raises(ValueError) as e:
        run_calibration(sparse, _yaw(0.0), pitch_z_offset_mm=PITCH_Z_OFFSET,
                        min_points=50)
    assert 'insufficient' in str(e.value).lower() or 'failed' in str(e.value).lower()


def test_single_marker_with_data_refused():
    data = _dataset()
    for i in [0, 1, 3, 4, 5, 6]:
        data[i] = []
    with pytest.raises(ValueError):
        run_calibration(data, _yaw(0.0), pitch_z_offset_mm=PITCH_Z_OFFSET)


def test_empty_data_refused():
    with pytest.raises(ValueError):
        run_calibration({i: [] for i in range(BB_MARKER_COUNT)}, _yaw(0.0),
                        pitch_z_offset_mm=PITCH_Z_OFFSET)


def test_non_rigid_motion_refused():
    ca, cb = np.array([0.0, 0.0, 100.0]), np.array([50.0, 50.0, 100.0])
    data = {
        0: list(_circle(ca, TRUE_AXIS, 80.0, 200, rng=np.random.default_rng(1))),
        1: list(_circle(cb, TRUE_AXIS, 80.0, 200, rng=np.random.default_rng(2))),
        2: list(_circle(ca, TRUE_AXIS, 90.0, 200, rng=np.random.default_rng(3))),
        3: [], 4: [],
    }
    with pytest.raises(ValueError) as e:
        run_calibration(data, _yaw(0.0), pitch_z_offset_mm=PITCH_Z_OFFSET)
    assert 'deviate' in str(e.value).lower() or 'axis' in str(e.value).lower()


def test_tilted_axis_tilt_and_position_recovered():
    tilt = math.radians(5.0)
    axis = np.array([math.sin(tilt), 0.0, math.cos(tilt)])
    pos = np.array([-500.0, 200.0, 1600.0])
    data = _dataset(pos=pos, axis=axis, rng=np.random.default_rng(77))
    # Only the co-planar markers set the plane height.
    avg_z = np.mean([p[2] for i in BB_PLANE_MARKER_INDICES for p in data[i]]) + PITCH_Z_OFFSET
    inter = find_axis_plane_intersection(pos, axis, avg_z)
    r = run_calibration(data, _yaw_for(data, TRUE_YAW_OFFSET, origin=inter),
                        pitch_z_offset_mm=PITCH_Z_OFFSET)
    assert abs(r.axis_tilt_deg - 5.0) < 1.0
    assert np.linalg.norm(r.bb_position_mm[:2] - inter[:2]) < 0.5


def test_partial_marker_visibility_still_succeeds():
    data = _dataset()
    data[0] = []
    data[4] = []
    r = run_calibration(data, _yaw_for(data, TRUE_YAW_OFFSET),
                        pitch_z_offset_mm=PITCH_Z_OFFSET)
    assert np.linalg.norm(r.bb_position_mm[:2] - TRUE_POS[:2]) < 0.5


def test_fit_circle_3d_accuracy_on_known_circle():
    centre = np.array([100.0, -200.0, 500.0])
    pts = _circle(centre, np.array([0.0, 0.0, 1.0]), 85.0, 100)
    c, r, _n, res = fit_circle_3d(pts)
    assert np.linalg.norm(c - centre) < 1e-6
    assert abs(r - 85.0) < 1e-6
    assert res < 1e-6


def test_too_few_yaw_readings_refused():
    with pytest.raises(ValueError) as e:
        run_calibration(_dataset(), [0.0] * 3, pitch_z_offset_mm=PITCH_Z_OFFSET)
    assert 'yaw' in str(e.value).lower() or 'insufficient' in str(e.value).lower()


def test_high_yaw_uncertainty_refused():
    wild = [float(i * 10) for i in range(20)]
    with pytest.raises(ValueError, match='(?i)uncertainty'):
        run_calibration(_dataset(), wild, pitch_z_offset_mm=PITCH_Z_OFFSET,
                        max_yaw_std_deg=5.0)


def test_horizontal_axis_refused():
    data = _dataset(axis=np.array([1.0, 0.0, 0.0]))
    with pytest.raises(ValueError, match='(?i)horizontal'):
        run_calibration(data, _yaw(0.0), pitch_z_offset_mm=PITCH_Z_OFFSET)
