"""The BB calibration arc-span floor — bb_calibration.MIN_ARC_DEG (F4/Q5a).

WHY THIS FILE EXISTS SEPARATELY FROM
``ros_ws/src/jugglebot/jugglebot/tests/test_bb_calibration.py``. That module is
a standalone script with a hand-rolled ``run_tests()`` runner and no ``test_*``
functions, and it lives outside ``testpaths = ["tests"]`` (pyproject.toml). So
``./run_tests.sh`` does not collect a single assertion from it — it is run by
hand, or not at all. The arc-span floor is a safety invariant (see below);
"protected by a test nobody runs" is not protection, so the floor's cases are
pinned here too, where the per-commit gate sees them. The pipeline-level cases
still live in that script, next to the other 14.

WHAT THE FLOOR IS FOR. ``find_rotation_axis`` used to ask only for *enough
points* (``min_points=50``). At the 200 Hz marker rate that is 0.25 s of data,
so a sweep truncated by a mid-sweep QTM dropout could still hand the algebraic
circle fit a stubby arc — and a short arc fits a circle with a tiny residual and
a wildly wrong centre and radius. The output is not a loud failure but a
plausible BB position that ``ball_butler_node`` then aims every throw with.
Point count cannot distinguish "swept slowly" from "barely moved"; arc span can.

Empirical note (probe run 2026-08-22, on the pinned numpy in the project venv):
synthetic sweeps of 5°, 25° and 180° measure 4.98°, 24.88° and 179.1°
respectively through ``fit_circle_3d`` → ``arc_span_deg``, so the fixtures below
sit unambiguously on either side of the 20° floor rather than near it.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from jugglebot.bb_calibration import (
    MIN_ARC_DEG,
    arc_span_deg,
    fit_circle_3d,
    find_rotation_axis,
    run_calibration,
)


AXIS = np.array([0.0, 0.0, 1.0])
CENTRE = np.array([0.0, 0.0, 0.0])
BB_POS = np.array([-707.0, -149.0, 1724.0])
RADII = (80.0, 95.0, 110.0, 90.0, 75.0)
Z_OFFSETS = (-30.0, -15.0, 0.0, 15.0, 30.0)
PITCH_Z_OFFSET_MM = 17.5


def _arc(a0, a1, n=200, radius=80.0, centre=CENTRE):
    """*n* points on a circle in the XY plane spanning [a0, a1] radians."""
    ang = np.linspace(a0, a1, n)
    return np.stack([
        centre[0] + radius * np.cos(ang),
        centre[1] + radius * np.sin(ang),
        np.full(n, centre[2]),
    ], axis=1)


def _sweep_dataset(span_deg, n_sweep=200, n_hold=250):
    """A 5-marker dataset whose sweep covers *span_deg*, then holds still.

    Mirrors the real capture: a sweep phase followed by a stationary hold (the
    yaw-offset calculation uses the last ~200 samples). The hold points land on
    the sweep's final position, so they add no arc — which is exactly the
    property the floor depends on.
    """
    data = {}
    for i in range(5):
        centre = BB_POS + Z_OFFSETS[i] * AXIS
        sweep = _arc(0.0, math.radians(span_deg), n_sweep,
                     radius=RADII[i], centre=centre)
        hold = np.tile(sweep[-1], (n_hold, 1))
        data[i] = [p for p in np.vstack([sweep, hold])]
    return data


def _yaw_readings(data, offset_rad=-0.053, origin=BB_POS):
    """Yaw readings consistent with Marker 3's final global angle."""
    m3_end = np.array(data[2])[-1]
    angle = math.atan2(m3_end[1] - origin[1], m3_end[0] - origin[0])
    return [math.degrees(angle - offset_rad)] * 20


# ── arc_span_deg itself ──────────────────────────────────────────────────────

@pytest.mark.parametrize('span_deg', [5.0, 30.0, 90.0, 180.0])
def test_arc_span_recovers_the_swept_angle(span_deg):
    pts = _arc(0.0, math.radians(span_deg))
    assert arc_span_deg(pts, CENTRE, AXIS) == pytest.approx(span_deg, abs=0.5)


def test_arc_span_is_immune_to_the_atan2_branch_cut():
    """The construction is ``360° − largest gap``, not ``max − min``.

    A short arc that happens to straddle ±π reads as ~360° under max−min, which
    would defeat the floor in precisely the case it exists for: the dropout does
    not care where on the circle the sweep was when QTM died.
    """
    straddling = _arc(math.pi - 0.05, math.pi + 0.05)
    assert arc_span_deg(straddling, CENTRE, AXIS) == pytest.approx(5.73, abs=0.2)


def test_arc_span_of_a_full_circle_is_near_360():
    pts = _arc(0.0, 2 * math.pi, n=200)
    assert arc_span_deg(pts, CENTRE, AXIS) > 355.0


def test_arc_span_fails_closed_on_degenerate_input():
    """Fail CLOSED: a marker that never moved has no meaningful span, and 0°
    excludes it rather than letting a NaN comparison slip past the floor."""
    identical = np.tile(np.array([1.0, 2.0, 3.0]), (100, 1))
    assert arc_span_deg(identical, CENTRE, AXIS) == 0.0
    assert arc_span_deg(np.array([[1.0, 2.0, 3.0]]), CENTRE, AXIS) == 0.0
    assert arc_span_deg(np.zeros((10, 3)), CENTRE,
                        np.array([np.nan, np.nan, np.nan])) == 0.0


# ── the floor, through find_rotation_axis / run_calibration ──────────────────

def test_truncated_sweep_is_refused_by_name():
    """The whole point: 200 points per marker — five times ``min_points`` — and
    it is still refused, because the markers only moved 5°."""
    data = _sweep_dataset(5.0)
    with pytest.raises(ValueError) as exc:
        run_calibration(data, _yaw_readings(data),
                        pitch_z_offset_mm=PITCH_Z_OFFSET_MM)
    assert 'ARC_SPAN_TOO_SMALL' in str(exc.value)


def test_truncated_sweep_would_have_been_accepted_without_the_floor():
    """Pins the danger, not just the fix: the same data passes cleanly when the
    floor is lowered, which is what the pre-F4 code did on every truncated
    sweep — a confident BB pose fitted to 5° of arc."""
    data = _sweep_dataset(5.0)
    result = run_calibration(data, _yaw_readings(data),
                             pitch_z_offset_mm=PITCH_Z_OFFSET_MM,
                             min_arc_deg=0.0)
    assert result.bb_position_mm is not None


def test_sweep_just_above_the_floor_succeeds():
    """The floor rejects truncation, not a slow-but-real sweep."""
    data = _sweep_dataset(25.0)
    result = run_calibration(data, _yaw_readings(data),
                             pitch_z_offset_mm=PITCH_Z_OFFSET_MM)
    assert np.linalg.norm(result.bb_position_mm[:2] - BB_POS[:2]) < 0.5


def test_one_stubby_marker_is_excluded_not_fatal():
    """A single occluded marker must not take the calibration down with it.

    Its centre/normal would otherwise be folded into the length-weighted
    average — the same treatment a too-few-points marker already got. The
    remaining four still carry the fit.
    """
    data = _sweep_dataset(180.0)
    stub_centre = BB_POS + Z_OFFSETS[0] * AXIS
    data[0] = [p for p in _arc(0.0, math.radians(4.0), 200,
                               radius=RADII[0], centre=stub_centre)]

    result = run_calibration(data, _yaw_readings(data),
                             pitch_z_offset_mm=PITCH_Z_OFFSET_MM)
    assert np.linalg.norm(result.bb_position_mm[:2] - BB_POS[:2]) < 0.5
    assert result.marker_metrics[0].status == 'skipped'
    assert 'arc span' in result.marker_metrics[0].reason
    assert result.marker_metrics[1].status == 'ok'


def test_metrics_carry_the_measured_span():
    """The span is reported, not just tested against — the owner confirms or
    raises MIN_ARC_DEG from the first hardware calibrate's logged value
    (plans/active/operator-observability.md § 8), and mocap_node logs it."""
    data = _sweep_dataset(180.0)
    result = run_calibration(data, _yaw_readings(data),
                             pitch_z_offset_mm=PITCH_Z_OFFSET_MM)
    assert result.marker_metrics[2].arc_span_deg == pytest.approx(179.0, abs=2.0)


def test_arc_refusal_is_distinct_from_the_insufficient_points_refusal():
    """Different causes, different messages. All-short markers means the SWEEP
    did not happen; the generic 'only 0 markers had sufficient data' would send
    the operator hunting for occlusion that is not there.
    """
    short_sweep = {i: [p for p in _arc(0.0, math.radians(3.0), 200,
                                       radius=RADII[i],
                                       centre=BB_POS + Z_OFFSETS[i] * AXIS)]
                   for i in range(5)}
    with pytest.raises(ValueError) as arc_exc:
        find_rotation_axis({k: np.array(v) for k, v in short_sweep.items()})
    assert 'ARC_SPAN_TOO_SMALL' in str(arc_exc.value)

    too_few = {i: np.array([[0.0, 0.0, 0.0]] * 10) for i in range(5)}
    with pytest.raises(ValueError) as pts_exc:
        find_rotation_axis(too_few)
    assert 'ARC_SPAN_TOO_SMALL' not in str(pts_exc.value)
    assert 'sufficient data' in str(pts_exc.value)


def test_default_floor_is_the_documented_value():
    """20° is a deliberate, conservative choice pending hardware confirmation;
    a silent change to it should show up as a failing test, not as a quietly
    different acceptance threshold on the next calibrate."""
    assert MIN_ARC_DEG == 20.0


def test_fit_circle_3d_is_unchanged_by_the_shared_plane_basis():
    """Q5a refactored ``fit_circle_3d``'s inline basis construction out into
    ``plane_basis`` so ``arc_span_deg`` could reuse it. Same numbers as before:
    an exact circle still recovers to ~1e-12."""
    centre = np.array([100.0, -200.0, 500.0])
    pts = _arc(0.0, 2 * math.pi, n=100, radius=85.0, centre=centre)
    c, r, n, residual = fit_circle_3d(pts)
    assert np.linalg.norm(c - centre) < 1e-6
    assert r == pytest.approx(85.0, abs=1e-6)
    assert residual < 1e-6
