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
      rather than fatal. Exercised at THREE marker-noise levels (0.0, 0.1,
      0.5 mm), because the floor's first implementation worked only at 0.0
  16. Marker-radius floor (MIN_MARKER_RADIUS_MM) — the collapsed-fit signature
      that noise produces, and the yaw-series span that is immune to it
  17. Branch-cut-safe yaw-offset uncertainty — a hold pose on the ±π cut must
      not read as ±240° of uncertainty

EMPIRICAL NOTE (probe ``/tmp/probe_arc_noise.py``, run 2026-08-22 on the pinned
numpy in the project venv). A true 110 mm-radius arc, 200 samples, measured
through ``fit_circle_3d`` → ``arc_span_deg``:

    sweep   noise mm    fitted r      span read
    5°      0.00        110.00 mm     5.0°
    5°      0.10          9.59 mm     56.3°
    5°      0.50          2.86 mm     327.6°
    5°      1.50          3.48 mm     336.6°
    180°    0.50        109.98 mm     180.2°

QTM delivers 0.1–1.5 mm. So on hardware the marker-only span floor would have
fired on NOTHING: a stubby arc's fitted radius collapses ~110 mm → ~2 mm and its
noise ball subtends most of a circle. The primary gate is therefore BB's
reported yaw series (``angular_span_deg``) — encoder-derived, already 1-D,
nothing fitted, so 5° reads 5.0° at every noise level above.

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
    MIN_ARC_DEG,
    MIN_MARKER_RADIUS_MM,
    angular_span_deg,
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


def sweep_yaw_readings(end_yaw_deg, span_deg=180.0, n_sweep=40, n_hold=20):
    """BB's reported yaw series for a *span_deg* sweep ending at *end_yaw_deg*.

    ⚠ IT MUST SWEEP. mocap_node accumulates these from BB heartbeats across the
    whole CALIBRATING state, so the real series traces the motion and only then
    settles at the hold; ``run_calibration``'s primary sweep-completeness gate
    reads the WHOLE series, while ``calculate_yaw_offset`` reads only the last
    ~10 (the hold). A constant list — what these fixtures used before that gate
    existed — is precisely what a BB whose yaw motor never moved would report,
    so it is not a simplification of the real signal, it is the failure case.
    """
    sweep = list(np.linspace(end_yaw_deg - span_deg, end_yaw_deg, n_sweep))
    return sweep + [end_yaw_deg] * n_hold


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
    yaw_readings = sweep_yaw_readings(yaw_deg)  # sweeps, then holds

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
        test_yaw_readings = sweep_yaw_readings(test_yaw_deg)

        r = run_calibration(test_data, test_yaw_readings, pitch_z_offset_mm=PITCH_Z_OFFSET)
        err = abs(wrap_pi(r.yaw_offset_rad - test_offset))
        check(f'Offset {test_offset:+.2f} rad recovered', err < 0.01,
              f'error = {err:.4f} rad')

    # ==================================================================
    print('\nTest 4: Insufficient markers — all below min_points')
    # ==================================================================
    sparse_data = {i: [np.array([0.0, 0.0, 0.0])] * 10 for i in range(5)}
    try:
        run_calibration(sparse_data, sweep_yaw_readings(0.0),
                        pitch_z_offset_mm=PITCH_Z_OFFSET, min_points=50)
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
        run_calibration(one_marker_data, sweep_yaw_readings(0.0),
                        pitch_z_offset_mm=PITCH_Z_OFFSET)
        check('Raises ValueError for 1 marker', False, 'no exception raised')
    except ValueError as e:
        check('Raises ValueError for 1 marker', True, str(e))

    # ==================================================================
    print('\nTest 6: Empty data')
    # ==================================================================
    empty_data: dict[int, list[np.ndarray]] = {i: [] for i in range(5)}
    try:
        run_calibration(empty_data, sweep_yaw_readings(0.0),
                        pitch_z_offset_mm=PITCH_Z_OFFSET)
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
        run_calibration(non_rigid, sweep_yaw_readings(0.0),
                        pitch_z_offset_mm=PITCH_Z_OFFSET)
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
    tilted_yaw_readings = sweep_yaw_readings(tilted_yaw_deg)

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
    partial_yaw_readings = sweep_yaw_readings(partial_yaw_deg)

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
        wrap_readings = sweep_yaw_readings(wrap_yaw_deg)

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
        run_calibration(horiz_data, sweep_yaw_readings(0.0),
                        pitch_z_offset_mm=PITCH_Z_OFFSET)
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
    #
    # ⚠ AND EVERY SWEEP CASE RUNS AT THREE NOISE LEVELS. The floor's first
    # implementation measured span around the FITTED circle centre, which is
    # exactly the quantity a truncated sweep makes meaningless: see the module
    # docstring's table — at 0.1 mm a 5° arc reads 56°, at 0.5 mm it reads 328°.
    # Noiseless fixtures were the only condition under which it worked, so
    # noiseless fixtures were the reason the defect shipped.

    def _m3_end_yaw(dataset):
        """The yaw reading consistent with Marker 3's final global angle."""
        m3 = np.array(dataset[2])[-1]
        return math.degrees(
            math.atan2(m3[1] - TRUE_POS[1], m3[0] - TRUE_POS[0]) - TRUE_YAW_OFFSET)

    for noise in (0.0, 0.1, 0.5):
        tag = f'{noise:.1f} mm noise'

        # 15a — truncated sweep (5°): refused, and by NAME, at every noise level.
        trunc_data = generate_calibration_dataset(
            TRUE_POS, TRUE_AXIS, RADII, Z_OFFSETS,
            n_sweep=200, sweep_range=(0.0, math.radians(5.0)),
            noise_std=noise, rng=np.random.default_rng(42),
        )
        trunc_yaw = sweep_yaw_readings(_m3_end_yaw(trunc_data), span_deg=5.0)
        try:
            run_calibration(trunc_data, trunc_yaw, pitch_z_offset_mm=PITCH_Z_OFFSET)
            check(f'Truncated 5° sweep refused ({tag})', False, 'no exception raised')
        except ValueError as e:
            check(f'Truncated 5° sweep refused ({tag})',
                  'ARC_SPAN_TOO_SMALL' in str(e), str(e))

        # 15c — a REAL sweep is not rejected, at any of these noise levels. The
        # gate must not cost a calibration that actually happened.
        full_data = generate_calibration_dataset(
            TRUE_POS, TRUE_AXIS, RADII, Z_OFFSETS,
            n_sweep=200, noise_std=noise, rng=np.random.default_rng(42),
        )
        full_yaw = sweep_yaw_readings(_m3_end_yaw(full_data), span_deg=180.0)
        try:
            full_result = run_calibration(full_data, full_yaw,
                                          pitch_z_offset_mm=PITCH_Z_OFFSET)
            full_err = np.linalg.norm(full_result.bb_position_mm[:2] - TRUE_POS[:2])
            check(f'180° sweep still succeeds ({tag})',
                  full_err < 0.5 + 2.0 * noise, f'error = {full_err:.4f} mm')
        except ValueError as e:
            check(f'180° sweep still succeeds ({tag})', False, str(e))

    # 15c-bis — a slow-but-real 25° sweep, just above the floor, is NOT rejected.
    #
    # ⚠ NOISELESS ON PURPOSE, and the reason is worth knowing. Add 0.1 mm and
    # this same 25° sweep fails — not at the arc floor but at the pre-existing
    # `max_dev > 3.0` axis-consistency check (3.12 mm at 0.1 mm, 4.82 mm at
    # 0.5 mm, measured 2026-08-22). A 25° arc simply does not constrain a circle
    # centre well enough to survive real marker noise. So MIN_ARC_DEG = 20°
    # admits sweeps that the downstream check rejects anyway — which is a floor
    # the owner may want to RAISE, not lower, once the first hardware calibrate
    # reports its yaw span (plans/active/operator-observability.md § 8 item 4).
    slow_data = generate_calibration_dataset(
        TRUE_POS, TRUE_AXIS, RADII, Z_OFFSETS,
        n_sweep=200, sweep_range=(0.0, math.radians(25.0)),
        noise_std=0.0, rng=np.random.default_rng(42),
    )
    slow_yaw = sweep_yaw_readings(_m3_end_yaw(slow_data), span_deg=25.0)
    try:
        slow_result = run_calibration(slow_data, slow_yaw,
                                      pitch_z_offset_mm=PITCH_Z_OFFSET)
        slow_err = np.linalg.norm(slow_result.bb_position_mm[:2] - TRUE_POS[:2])
        check('25° sweep still succeeds (noiseless)', slow_err < 0.5,
              f'error = {slow_err:.4f} mm')
    except ValueError as e:
        check('25° sweep still succeeds (noiseless)', False, str(e))

    # 15b — clean 5° data passes with the floors lowered. This is the danger
    # being pinned, not just the fix: it is exactly what the pre-floor code did.
    clean_trunc = generate_calibration_dataset(
        TRUE_POS, TRUE_AXIS, RADII, Z_OFFSETS,
        n_sweep=200, sweep_range=(0.0, math.radians(5.0)),
        noise_std=0.0, rng=np.random.default_rng(42),
    )
    clean_trunc_yaw = sweep_yaw_readings(_m3_end_yaw(clean_trunc), span_deg=5.0)
    try:
        r15 = run_calibration(clean_trunc, clean_trunc_yaw,
                              pitch_z_offset_mm=PITCH_Z_OFFSET,
                              min_arc_deg=0.0, min_radius_mm=0.0)
        check('Same data accepted with the floors disabled',
              r15.bb_position_mm is not None)
    except ValueError as e:
        check('Same data accepted with the floors disabled', False, str(e))

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
    stub_yaw = sweep_yaw_readings(_m3_end_yaw(stub_data), span_deg=180.0)
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
    print('\nTest 16: Marker-radius floor + the noise-immune yaw span')
    # ==================================================================
    # THE finding. Under QTM-realistic noise a stubby arc's circle fit collapses
    # to a few mm, and the span it subtends at that collapsed centre reads WIDE.
    # So the span check alone did not merely miss a bad marker — it preferred
    # it, keeping the noisiest stub while a truncated sweep sailed past the
    # aggregate floor and died later under an occlusion message.

    # 16a — the collapse signature itself.
    noisy_stub = generate_marker_circle(
        TRUE_POS, TRUE_AXIS, RADII[2], 200,
        angle_range=(0.0, math.radians(5.0)),
        noise_std=0.5, rng=np.random.default_rng(42),
    )
    _, stub_radius, _, _ = fit_circle_3d(noisy_stub)
    check('A noisy 5° arc fits a collapsed radius',
          stub_radius < MIN_MARKER_RADIUS_MM,
          f'fitted {stub_radius:.2f} mm from a true {RADII[2]:.0f} mm')

    # 16b — and that collapse is WHY the span check cannot be trusted here.
    from jugglebot.bb_calibration import arc_span_deg as _arc_span
    stub_c, _, stub_n, _ = fit_circle_3d(noisy_stub)
    check('…and its measured span is WIDE, not narrow',
          _arc_span(noisy_stub, stub_c, stub_n) > MIN_ARC_DEG,
          'span read narrow — the inversion this floor exists for is absent')

    # 16c — the yaw series does not move: no fit, no collapse, no inversion.
    check('The yaw span reads the true sweep',
          abs(angular_span_deg(np.linspace(0.0, 5.0, 40)) - 5.0) < 0.01)
    check('The yaw span is branch-cut safe',
          abs(angular_span_deg(np.linspace(-2.0, 3.0, 40)) - 5.0) < 0.01)
    check('The yaw span fails closed on a stationary series',
          angular_span_deg([7.0] * 20) == 0.0)

    # 16d — a single NOISY stub is excluded by radius, and the fit survives.
    noisy_stub_data = generate_calibration_dataset(
        TRUE_POS, TRUE_AXIS, RADII, Z_OFFSETS,
        n_sweep=200, noise_std=0.0, rng=np.random.default_rng(42),
    )
    noisy_stub_data[0] = [p for p in generate_marker_circle(
        TRUE_POS + Z_OFFSETS[0] * TRUE_AXIS, TRUE_AXIS, RADII[0], 200,
        angle_range=(0.0, math.radians(4.0)),
        noise_std=0.5, rng=np.random.default_rng(7),
    )]
    try:
        ns_result = run_calibration(
            noisy_stub_data, sweep_yaw_readings(_m3_end_yaw(noisy_stub_data)),
            pitch_z_offset_mm=PITCH_Z_OFFSET)
        m0n = ns_result.marker_metrics[0]
        check('Noisy stub excluded by the radius floor, not kept',
              m0n.status == 'skipped' and 'radius' in (m0n.reason or ''),
              f'{m0n.status} — {m0n.reason}')
        ns_err = np.linalg.norm(ns_result.bb_position_mm[:2] - TRUE_POS[:2])
        check('…and the four good markers still carry the fit',
              ns_err < 0.5, f'error = {ns_err:.4f} mm')
    except ValueError as e:
        check('Noisy stub excluded by the radius floor, not kept', False, str(e))

    # ==================================================================
    print('\nTest 17: Yaw-offset uncertainty survives the atan2 branch cut')
    # ==================================================================
    # calculate_yaw_offset averages Marker 3's global angle CIRCULARLY and then
    # measured its spread with an UNWRAPPED difference. With BB at (-707, -149)
    # and Marker 3 held pointing directly away from the origin — an ordinary
    # place for a 180° sweep to stop — the hold lands on the ±π branch cut, so
    # 0.1 mm of noise scatters samples between +π and −π and the spread reads
    # ±242.8° against a true ±0.053°. max_yaw_std_deg is 5°, so a perfectly good
    # calibration was refused for 'uncertainty too high'. Real, not synthetic:
    # it depends only on where BB sits in the global frame.
    # A 90° sweep here, not 180°: generate_marker_circle's basis puts angle 0 at
    # +Y, so this fixture's markers END on the −X axis — global angle π, the cut.
    # (The 180° datasets above end at 269°, nowhere near it, which is why they
    # never exposed this.)
    cut_data = generate_calibration_dataset(
        TRUE_POS, TRUE_AXIS, RADII, Z_OFFSETS,
        n_sweep=200, sweep_range=(0.0, math.pi / 2), noise_std=0.1,
        rng=np.random.default_rng(42),
    )
    m3_cut = np.array(cut_data[2])[-1]
    on_cut = abs(abs(math.atan2(m3_cut[1] - TRUE_POS[1],
                                m3_cut[0] - TRUE_POS[0])) - math.pi)
    check('Precondition: the hold pose sits on the ±π branch cut',
          on_cut < 0.05, f'off-cut by {on_cut:.3f} rad')
    try:
        cut_result = run_calibration(
            cut_data, sweep_yaw_readings(_m3_end_yaw(cut_data), span_deg=90.0),
            pitch_z_offset_mm=PITCH_Z_OFFSET)
        check('Branch-cut hold does not inflate the yaw uncertainty',
              cut_result.yaw_offset_std_deg < 1.0,
              f'std = {cut_result.yaw_offset_std_deg:.2f}°')
    except ValueError as e:
        check('Branch-cut hold does not inflate the yaw uncertainty', False, str(e))

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
