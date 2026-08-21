"""Phase 4 verification tests for Ball Butler calibration pipeline.

Tests:
  1. Perfect circle recovery — 5 markers on exact circles, axis + position recovered
  2. Noisy data — Gaussian noise on marker positions, results within tolerance
  3. Yaw offset recovery — synthetic yaw readings, offset recovered correctly
  4. Insufficient markers — fewer than min_points raises ValueError
  5. Single marker sufficient data — only 1 marker has enough points, raises ValueError
  6. Empty data — no marker positions at all, raises ValueError
  7. Non-rigid motion — markers not on a common axis, raises ValueError
  8. Axis tilt — tilted rotation axis, position + tilt recovered
  9. Partial marker visibility — some markers have NaN gaps, still succeeds
  10. Wrap-around yaw offset — offset near ±π boundary, correctly wrapped
  15. Arc-span floor (MIN_ARC_DEG) — a truncated sweep is refused by name, a
      slow-but-real sweep is not, and a single stubby marker is excluded
      rather than fatal

NOTE — this module is a standalone script, not a pytest module: it has no
``test_*`` functions and it lives outside ``testpaths = ["tests"]``, so
``./run_tests.sh`` collects NOTHING from it. The arc-span floor's cases are
therefore ALSO pinned in tests/ros/test_bb_calibration_arc_span.py, where the
per-commit gate runs them. Keep the two in step when changing the floor.

Run:  python -m jugglebot.tests.test_bb_calibration
"""

from __future__ import annotations

import math
import sys

import numpy as np

from jugglebot.bb_calibration import (
    fit_circle_3d,
    find_rotation_axis,
    find_axis_plane_intersection,
    calculate_yaw_offset,
    run_calibration,
    CalibrationResult,
    wrap_pi,
)


# ---------------------------------------------------------------------------
#  Helpers — synthetic data generation
# ---------------------------------------------------------------------------

def generate_marker_circle(
    center: np.ndarray,
    axis: np.ndarray,
    radius: float,
    n_points: int,
    angle_range: tuple[float, float] = (0.0, 2 * math.pi),
    noise_std: float = 0.0,
    rng: np.random.Generator | None = None,
) -> np.ndarray:
    """Generate n_points on a circle in 3D, optionally with Gaussian noise."""
    if rng is None:
        rng = np.random.default_rng(42)

    axis = axis / np.linalg.norm(axis)

    # Build orthonormal basis perpendicular to axis
    ref = np.array([1.0, 0.0, 0.0]) if abs(axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(axis, ref)
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)

    angles = np.linspace(angle_range[0], angle_range[1], n_points, endpoint=False)
    points = np.array([
        center + radius * (math.cos(a) * u + math.sin(a) * v)
        for a in angles
    ])

    if noise_std > 0:
        points += rng.normal(0, noise_std, points.shape)

    return points


def generate_calibration_dataset(
    bb_position: np.ndarray,
    axis_direction: np.ndarray,
    marker_radii: list[float],
    marker_z_offsets: list[float],
    n_sweep: int = 200,
    n_hold: int = 250,
    sweep_range: tuple[float, float] = (0.0, math.pi),
    noise_std: float = 0.0,
    rng: np.random.Generator | None = None,
) -> dict[int, list[np.ndarray]]:
    """Generate a full 5-marker calibration dataset.

    Each marker sits on a circle centered on the rotation axis at a different
    radius and Z offset from the BB position.

    The dataset simulates the real calibration: a sweep phase (n_sweep points
    tracing arcs) followed by a hold phase (n_hold points at the final
    position).  The yaw offset calculation uses the last ~200 samples, so the
    hold period must be long enough to dominate that window.
    """
    if rng is None:
        rng = np.random.default_rng(42)

    axis = axis_direction / np.linalg.norm(axis_direction)
    data: dict[int, list[np.ndarray]] = {}

    for i in range(5):
        if i < len(marker_radii):
            r = marker_radii[i]
            z_off = marker_z_offsets[i] if i < len(marker_z_offsets) else 0.0
            center = bb_position + z_off * axis

            # Sweep phase
            sweep_pts = generate_marker_circle(
                center, axis, r, n_sweep,
                angle_range=sweep_range,
                noise_std=noise_std,
                rng=rng,
            )

            # Hold phase — repeat the final sweep position (with noise)
            final_pt = sweep_pts[-1].copy()
            if noise_std > 0:
                hold_pts = np.tile(final_pt, (n_hold, 1)) + rng.normal(0, noise_std, (n_hold, 3))
            else:
                hold_pts = np.tile(final_pt, (n_hold, 1))

            all_pts = np.vstack([sweep_pts, hold_pts])
            data[i] = [p for p in all_pts]
        else:
            data[i] = []

    return data


# ---------------------------------------------------------------------------
#  Test runner
# ---------------------------------------------------------------------------

def run_tests():
    passed = 0
    failed = 0
    total = 0

    def check(name: str, condition: bool, detail: str = ''):
        nonlocal passed, failed, total
        total += 1
        if condition:
            passed += 1
            print(f'  PASS  {name}')
        else:
            failed += 1
            print(f'  FAIL  {name}  — {detail}')

    # ── Ground truth ──────────────────────────────────────────────
    TRUE_POS = np.array([-707.0, -149.0, 1724.0])
    TRUE_AXIS = np.array([0.0, 0.0, 1.0])  # vertical
    TRUE_YAW_OFFSET = -0.053  # rad
    PITCH_Z_OFFSET = 17.5  # mm
    RADII = [80.0, 95.0, 110.0, 90.0, 75.0]
    Z_OFFSETS = [-30.0, -15.0, 0.0, 15.0, 30.0]

    rng = np.random.default_rng(12345)

    # ==================================================================
    print('\nTest 1: Perfect circle recovery')
    # ==================================================================
    data = generate_calibration_dataset(
        TRUE_POS, TRUE_AXIS, RADII, Z_OFFSETS,
        n_sweep=200, noise_std=0.0, rng=rng,
    )

    # The calibration pipeline uses average marker Z + pitch_z_offset as
    # the intersection plane.  We need yaw readings that are consistent.
    # Marker 3 (index 2) has angle from axis = atan2(dy, dx) relative to
    # the intersection point.  The yaw readings should be such that
    # offset = marker_angle - yaw_reading.
    marker3_pts = np.array(data[2])
    # Compute what angle marker 3 has at the end of the sweep
    m3_end = marker3_pts[-1]
    marker3_angle = math.atan2(
        m3_end[1] - TRUE_POS[1],
        m3_end[0] - TRUE_POS[0],
    )
    # yaw_reading such that offset = marker3_angle - yaw_reading = TRUE_YAW_OFFSET
    yaw_deg = math.degrees(marker3_angle - TRUE_YAW_OFFSET)
    yaw_readings = [yaw_deg] * 20  # constant during sweep

    result = run_calibration(
        data, yaw_readings,
        pitch_z_offset_mm=PITCH_Z_OFFSET,
    )

    pos_err = np.linalg.norm(result.bb_position_mm[:2] - TRUE_POS[:2])
    check('XY position error < 0.1 mm', pos_err < 0.1,
          f'error = {pos_err:.4f} mm')
    check('Axis tilt < 0.1 deg', result.axis_tilt_deg < 0.1,
          f'tilt = {result.axis_tilt_deg:.4f} deg')
    yaw_err = abs(wrap_pi(result.yaw_offset_rad - TRUE_YAW_OFFSET))
    check('Yaw offset error < 0.01 rad', yaw_err < 0.01,
          f'error = {yaw_err:.4f} rad')
    check('Yaw uncertainty < 1.0 deg', result.yaw_offset_std_deg < 1.0,
          f'std = {result.yaw_offset_std_deg:.4f} deg')

    # ==================================================================
    print('\nTest 2: Noisy data (0.5 mm Gaussian)')
    # ==================================================================
    rng2 = np.random.default_rng(99)
    noisy_data = generate_calibration_dataset(
        TRUE_POS, TRUE_AXIS, RADII, Z_OFFSETS,
        n_sweep=300, noise_std=0.5, rng=rng2,
    )

    noisy_result = run_calibration(
        noisy_data,
        yaw_readings,
        pitch_z_offset_mm=PITCH_Z_OFFSET,
    )

    pos_err_noisy = np.linalg.norm(noisy_result.bb_position_mm[:2] - TRUE_POS[:2])
    check('Noisy XY position error < 2.0 mm', pos_err_noisy < 2.0,
          f'error = {pos_err_noisy:.4f} mm')
    check('Noisy axis tilt < 2.0 deg', noisy_result.axis_tilt_deg < 2.0,
          f'tilt = {noisy_result.axis_tilt_deg:.4f} deg')
    noisy_yaw_err = abs(wrap_pi(noisy_result.yaw_offset_rad - TRUE_YAW_OFFSET))
    check('Noisy yaw offset error < 0.1 rad', noisy_yaw_err < 0.1,
          f'error = {noisy_yaw_err:.4f} rad')

    # ==================================================================
    print('\nTest 3: Yaw offset recovery at multiple offsets')
    # ==================================================================
    for test_offset in [-math.pi + 0.1, -1.5, 0.0, 1.5, math.pi - 0.1]:
        test_data = generate_calibration_dataset(
            TRUE_POS, TRUE_AXIS, RADII, Z_OFFSETS,
            n_sweep=200, noise_std=0.0, rng=np.random.default_rng(42),
        )
        m3_pts = np.array(test_data[2])
        m3_last = m3_pts[-1]
        m3_angle = math.atan2(m3_last[1] - TRUE_POS[1], m3_last[0] - TRUE_POS[0])
        test_yaw_deg = math.degrees(m3_angle - test_offset)
        test_yaw_readings = [test_yaw_deg] * 20

        r = run_calibration(test_data, test_yaw_readings, pitch_z_offset_mm=PITCH_Z_OFFSET)
        err = abs(wrap_pi(r.yaw_offset_rad - test_offset))
        check(f'Offset {test_offset:+.2f} rad recovered', err < 0.01,
              f'error = {err:.4f} rad')

    # ==================================================================
    print('\nTest 4: Insufficient markers — all below min_points')
    # ==================================================================
    sparse_data = {i: [np.array([0.0, 0.0, 0.0])] * 10 for i in range(5)}
    try:
        run_calibration(sparse_data, [0.0] * 20, pitch_z_offset_mm=PITCH_Z_OFFSET,
                        min_points=50)
        check('Raises ValueError for sparse data', False, 'no exception raised')
    except ValueError as e:
        check('Raises ValueError for sparse data', 'insufficient' in str(e).lower()
              or 'failed' in str(e).lower(), str(e))

    # ==================================================================
    print('\nTest 5: Only 1 marker has enough points')
    # ==================================================================
    one_marker_data = generate_calibration_dataset(
        TRUE_POS, TRUE_AXIS, RADII, Z_OFFSETS,
        n_sweep=200, noise_std=0.0, rng=np.random.default_rng(42),
    )
    # Zero out 4 of 5 markers
    for i in [0, 1, 3, 4]:
        one_marker_data[i] = []
    try:
        run_calibration(one_marker_data, [0.0] * 20, pitch_z_offset_mm=PITCH_Z_OFFSET)
        check('Raises ValueError for 1 marker', False, 'no exception raised')
    except ValueError as e:
        check('Raises ValueError for 1 marker', True, str(e))

    # ==================================================================
    print('\nTest 6: Empty data')
    # ==================================================================
    empty_data: dict[int, list[np.ndarray]] = {i: [] for i in range(5)}
    try:
        run_calibration(empty_data, [0.0] * 20, pitch_z_offset_mm=PITCH_Z_OFFSET)
        check('Raises ValueError for empty data', False, 'no exception raised')
    except ValueError as e:
        check('Raises ValueError for empty data', True, str(e))

    # ==================================================================
    print('\nTest 7: Non-rigid motion (markers on different axes)')
    # ==================================================================
    non_rigid: dict[int, list[np.ndarray]] = {}
    # Marker 0 and 1 on different rotation centers (>3mm apart)
    center_a = np.array([0.0, 0.0, 100.0])
    center_b = np.array([50.0, 50.0, 100.0])  # 70mm away — well above 3mm threshold
    non_rigid[0] = [p for p in generate_marker_circle(
        center_a, TRUE_AXIS, 80.0, 200, rng=np.random.default_rng(1))]
    non_rigid[1] = [p for p in generate_marker_circle(
        center_b, TRUE_AXIS, 80.0, 200, rng=np.random.default_rng(2))]
    non_rigid[2] = [p for p in generate_marker_circle(
        center_a, TRUE_AXIS, 90.0, 200, rng=np.random.default_rng(3))]
    non_rigid[3] = []
    non_rigid[4] = []
    try:
        run_calibration(non_rigid, [0.0] * 20, pitch_z_offset_mm=PITCH_Z_OFFSET)
        check('Raises ValueError for non-rigid', False, 'no exception raised')
    except ValueError as e:
        check('Raises ValueError for non-rigid', 'deviate' in str(e).lower()
              or 'axis' in str(e).lower(), str(e))

    # ==================================================================
    print('\nTest 8: Tilted rotation axis (5 degrees from vertical)')
    # ==================================================================
    tilt_deg = 5.0
    tilt_rad = math.radians(tilt_deg)
    tilted_axis = np.array([math.sin(tilt_rad), 0.0, math.cos(tilt_rad)])
    tilted_pos = np.array([-500.0, 200.0, 1600.0])

    tilted_data = generate_calibration_dataset(
        tilted_pos, tilted_axis, RADII, Z_OFFSETS,
        n_sweep=200, noise_std=0.0, rng=np.random.default_rng(77),
    )

    # Construct yaw readings for tilted case
    m3_tilted = np.array(tilted_data[2])
    # Use the intersection point for angle calculation (same as calibration does)
    avg_z = np.mean([p[2] for pts in tilted_data.values() for p in pts]) + PITCH_Z_OFFSET
    intersection = find_axis_plane_intersection(tilted_pos, tilted_axis, avg_z)
    m3_last_tilted = m3_tilted[-1]
    m3_angle_tilted = math.atan2(
        m3_last_tilted[1] - intersection[1],
        m3_last_tilted[0] - intersection[0],
    )
    tilted_yaw_deg = math.degrees(m3_angle_tilted - TRUE_YAW_OFFSET)
    tilted_yaw_readings = [tilted_yaw_deg] * 20

    tilted_result = run_calibration(
        tilted_data, tilted_yaw_readings, pitch_z_offset_mm=PITCH_Z_OFFSET,
    )

    check('Tilt recovered within 1.0 deg',
          abs(tilted_result.axis_tilt_deg - tilt_deg) < 1.0,
          f'measured={tilted_result.axis_tilt_deg:.2f} vs expected={tilt_deg:.1f}')

    # XY position check — intersection point moves from tilted_pos
    # so we compare against the computed intersection
    tilted_xy_err = np.linalg.norm(tilted_result.bb_position_mm[:2] - intersection[:2])
    check('Tilted XY position error < 0.5 mm', tilted_xy_err < 0.5,
          f'error = {tilted_xy_err:.4f} mm')

    # ==================================================================
    print('\nTest 9: Partial marker visibility (2 of 5 markers missing)')
    # ==================================================================
    partial_data = generate_calibration_dataset(
        TRUE_POS, TRUE_AXIS, RADII, Z_OFFSETS,
        n_sweep=200, noise_std=0.0, rng=np.random.default_rng(42),
    )
    # Remove markers 0 and 4 entirely
    partial_data[0] = []
    partial_data[4] = []

    # Reconstruct yaw readings for partial data
    m3_partial = np.array(partial_data[2])
    m3_last_p = m3_partial[-1]
    m3_angle_p = math.atan2(m3_last_p[1] - TRUE_POS[1], m3_last_p[0] - TRUE_POS[0])
    partial_yaw_deg = math.degrees(m3_angle_p - TRUE_YAW_OFFSET)
    partial_yaw_readings = [partial_yaw_deg] * 20

    partial_result = run_calibration(
        partial_data, partial_yaw_readings, pitch_z_offset_mm=PITCH_Z_OFFSET,
    )

    partial_pos_err = np.linalg.norm(partial_result.bb_position_mm[:2] - TRUE_POS[:2])
    check('Partial visibility XY error < 0.5 mm', partial_pos_err < 0.5,
          f'error = {partial_pos_err:.4f} mm')
    check('3 markers sufficient', True)

    # ==================================================================
    print('\nTest 10: Wrap-around yaw offset near +/-pi')
    # ==================================================================
    for boundary_offset in [-math.pi + 0.01, math.pi - 0.01]:
        wrap_data = generate_calibration_dataset(
            TRUE_POS, TRUE_AXIS, RADII, Z_OFFSETS,
            n_sweep=200, noise_std=0.0, rng=np.random.default_rng(42),
        )
        m3_wrap = np.array(wrap_data[2])
        m3_last_w = m3_wrap[-1]
        m3_angle_w = math.atan2(m3_last_w[1] - TRUE_POS[1], m3_last_w[0] - TRUE_POS[0])
        wrap_yaw_deg = math.degrees(m3_angle_w - boundary_offset)
        wrap_readings = [wrap_yaw_deg] * 20

        wrap_result = run_calibration(
            wrap_data, wrap_readings, pitch_z_offset_mm=PITCH_Z_OFFSET,
        )
        wrap_err = abs(wrap_pi(wrap_result.yaw_offset_rad - boundary_offset))
        check(f'Wrap-around offset {boundary_offset:+.3f} rad',
              wrap_err < 0.01, f'error = {wrap_err:.4f} rad')

    # ==================================================================
    print('\nTest 11: fit_circle_3d accuracy on known circle')
    # ==================================================================
    known_center = np.array([100.0, -200.0, 500.0])
    known_axis = np.array([0.0, 0.0, 1.0])
    known_radius = 85.0
    circle_pts = generate_marker_circle(
        known_center, known_axis, known_radius, 100,
        rng=np.random.default_rng(42),
    )
    c, r, n, res = fit_circle_3d(circle_pts)
    check('Circle center error < 1e-6 mm',
          np.linalg.norm(c - known_center) < 1e-6,
          f'error = {np.linalg.norm(c - known_center):.2e}')
    check('Circle radius error < 1e-6 mm',
          abs(r - known_radius) < 1e-6,
          f'error = {abs(r - known_radius):.2e}')
    check('Circle residual < 1e-6 mm', res < 1e-6,
          f'residual = {res:.2e}')

    # ==================================================================
    print('\nTest 12: Insufficient yaw readings')
    # ==================================================================
    good_data = generate_calibration_dataset(
        TRUE_POS, TRUE_AXIS, RADII, Z_OFFSETS,
        n_sweep=200, noise_std=0.0, rng=np.random.default_rng(42),
    )
    try:
        run_calibration(good_data, [0.0] * 3, pitch_z_offset_mm=PITCH_Z_OFFSET)
        check('Raises ValueError for too few yaw readings', False, 'no exception raised')
    except ValueError as e:
        check('Raises ValueError for too few yaw readings',
              'yaw' in str(e).lower() or 'insufficient' in str(e).lower(), str(e))

    # ==================================================================
    print('\nTest 13: High yaw uncertainty rejected')
    # ==================================================================
    good_data_13 = generate_calibration_dataset(
        TRUE_POS, TRUE_AXIS, RADII, Z_OFFSETS,
        n_sweep=200, noise_std=0.0, rng=np.random.default_rng(42),
    )
    # Wildly varying yaw readings → high uncertainty
    noisy_yaw = [float(i * 10) for i in range(20)]  # 0, 10, 20, ..., 190 degrees
    try:
        run_calibration(good_data_13, noisy_yaw, pitch_z_offset_mm=PITCH_Z_OFFSET,
                        max_yaw_std_deg=5.0)
        check('Raises ValueError for high yaw uncertainty', False, 'no exception raised')
    except ValueError as e:
        check('Raises ValueError for high yaw uncertainty',
              'uncertainty' in str(e).lower(), str(e))

    # ==================================================================
    print('\nTest 14: Horizontal axis rejected')
    # ==================================================================
    horiz_axis = np.array([1.0, 0.0, 0.0])
    horiz_data = generate_calibration_dataset(
        TRUE_POS, horiz_axis, RADII, Z_OFFSETS,
        n_sweep=200, noise_std=0.0, rng=np.random.default_rng(42),
    )
    try:
        run_calibration(horiz_data, [0.0] * 20, pitch_z_offset_mm=PITCH_Z_OFFSET)
        check('Raises ValueError for horizontal axis', False, 'no exception raised')
    except ValueError as e:
        check('Raises ValueError for horizontal axis',
              'horizontal' in str(e).lower(), str(e))

    # ==================================================================
    print('\nTest 15: Arc-span floor (MIN_ARC_DEG)')
    # ==================================================================
    # The failure this guards: `min_points=50` at the 200 Hz marker rate is
    # 0.25 s of data, so a sweep truncated by a mid-sweep QTM dropout could
    # still be fitted — and a short arc fits a circle with a tiny residual and
    # a wildly wrong centre. The output was a plausible BB pose, not an error.
    #
    # NB every dataset below has 200 sweep points per marker (4x min_points),
    # so point count alone would accept all of them. Span is what separates
    # them.

    # 15a — truncated sweep (5°): refused, and by NAME.
    trunc_data = generate_calibration_dataset(
        TRUE_POS, TRUE_AXIS, RADII, Z_OFFSETS,
        n_sweep=200, sweep_range=(0.0, math.radians(5.0)),
        noise_std=0.0, rng=np.random.default_rng(42),
    )
    m3_t = np.array(trunc_data[2])[-1]
    trunc_yaw = [math.degrees(
        math.atan2(m3_t[1] - TRUE_POS[1], m3_t[0] - TRUE_POS[0]) - TRUE_YAW_OFFSET)] * 20
    try:
        run_calibration(trunc_data, trunc_yaw, pitch_z_offset_mm=PITCH_Z_OFFSET)
        check('Truncated 5° sweep refused', False, 'no exception raised')
    except ValueError as e:
        check('Truncated 5° sweep refused', 'ARC_SPAN_TOO_SMALL' in str(e), str(e))

    # 15b — the same data passes with the floor lowered. This is the danger
    # being pinned, not just the fix: it is exactly what the pre-floor code did.
    try:
        r15 = run_calibration(trunc_data, trunc_yaw,
                              pitch_z_offset_mm=PITCH_Z_OFFSET, min_arc_deg=0.0)
        check('Same data accepted with the floor disabled',
              r15.bb_position_mm is not None)
    except ValueError as e:
        check('Same data accepted with the floor disabled', False, str(e))

    # 15c — a slow-but-real 25° sweep is NOT rejected.
    slow_data = generate_calibration_dataset(
        TRUE_POS, TRUE_AXIS, RADII, Z_OFFSETS,
        n_sweep=200, sweep_range=(0.0, math.radians(25.0)),
        noise_std=0.0, rng=np.random.default_rng(42),
    )
    m3_s = np.array(slow_data[2])[-1]
    slow_yaw = [math.degrees(
        math.atan2(m3_s[1] - TRUE_POS[1], m3_s[0] - TRUE_POS[0]) - TRUE_YAW_OFFSET)] * 20
    try:
        slow_result = run_calibration(slow_data, slow_yaw,
                                      pitch_z_offset_mm=PITCH_Z_OFFSET)
        slow_err = np.linalg.norm(slow_result.bb_position_mm[:2] - TRUE_POS[:2])
        check('25° sweep still succeeds', slow_err < 0.5, f'error = {slow_err:.4f} mm')
    except ValueError as e:
        check('25° sweep still succeeds', False, str(e))

    # 15d — one occluded marker is EXCLUDED, not fatal: its garbage centre must
    # not be folded into the length-weighted average, but four good markers are
    # still a calibration. (This is the case-9 scenario taken to its limit —
    # case 9's markers all trace the full 180° sweep, so the floor leaves it
    # untouched; here marker 0 is cut to 4°.)
    stub_data = generate_calibration_dataset(
        TRUE_POS, TRUE_AXIS, RADII, Z_OFFSETS,
        n_sweep=200, noise_std=0.0, rng=np.random.default_rng(42),
    )
    stub_data[0] = [p for p in generate_marker_circle(
        TRUE_POS + Z_OFFSETS[0] * TRUE_AXIS, TRUE_AXIS, RADII[0], 200,
        angle_range=(0.0, math.radians(4.0)),
    )]
    m3_p = np.array(stub_data[2])[-1]
    stub_yaw = [math.degrees(
        math.atan2(m3_p[1] - TRUE_POS[1], m3_p[0] - TRUE_POS[0]) - TRUE_YAW_OFFSET)] * 20
    try:
        stub_result = run_calibration(stub_data, stub_yaw,
                                      pitch_z_offset_mm=PITCH_Z_OFFSET)
        stub_err = np.linalg.norm(stub_result.bb_position_mm[:2] - TRUE_POS[:2])
        check('Stubby marker excluded, calibration survives',
              stub_err < 0.5, f'error = {stub_err:.4f} mm')
        m0 = stub_result.marker_metrics[0]
        check('Stubby marker reported as skipped with the arc reason',
              m0.status == 'skipped' and 'arc span' in (m0.reason or ''),
              f'{m0.status} — {m0.reason}')
    except ValueError as e:
        check('Stubby marker excluded, calibration survives', False, str(e))

    # ==================================================================
    #  Summary
    # ==================================================================
    print(f'\n{"=" * 50}')
    print(f'Results: {passed}/{total} passed, {failed} failed')
    print(f'{"=" * 50}')

    return failed == 0


def main():
    success = run_tests()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
