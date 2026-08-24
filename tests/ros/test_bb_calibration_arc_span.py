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

⚠ WHY EVERY SWEEP FIXTURE HERE IS PARAMETRISED OVER MARKER NOISE. The first
version of this file tested only noiseless synthetic arcs, and that is exactly
the condition under which the original marker-only floor worked. On hardware it
would have fired on nothing.

Empirical note (probe ``/tmp/probe_arc_noise.py``, run 2026-08-22 on the pinned
numpy in the project venv; a true 110 mm-radius arc, 200 samples, measured
through ``fit_circle_3d`` → ``arc_span_deg``):

===== ========= ============ ===========
sweep noise mm  fitted r     span read
===== ========= ============ ===========
5°    0.00      110.00 mm    5.0°
5°    0.10        9.59 mm    56.3°
5°    0.50        2.86 mm    327.6°
5°    1.50        3.48 mm    336.6°
180°  0.50      109.98 mm    180.2°
===== ========= ============ ===========

QTM marker noise is 0.1–1.5 mm in practice. So a stubby arc plus ordinary noise
collapses the fitted radius by two orders of magnitude and the noise ball is
then read as most of a circle — the floor never trips, and the per-marker
exclusion actually INVERTS (the noisiest stubby marker reads the widest span, so
it is kept and poisons the axis average). The truncated sweep instead died later
in the ``max_dev > 3.0`` check, under an occlusion message pointing the operator
at the wrong subsystem.

The fix, and what these tests pin: the PRIMARY gate is on BB's reported yaw
series (``angular_span_deg``), which is encoder-derived, already 1-D, and needs
no centre fitted — a 5° sweep reads 5.0° at every noise level in the table. The
marker path keeps its span check for clean partial occlusions and gains
``MIN_MARKER_RADIUS_MM`` for the collapse signature, which is the half that
survives noise.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from jugglebot.bb_calibration import (
    MIN_ARC_DEG,
    MIN_MARKER_RADIUS_MM,
    angular_span_deg,
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


#: Marker-position noise levels every sweep fixture is exercised at (mm, 1σ).
#: 0.0 is the synthetic ideal the original fixtures used; 0.1 and 0.5 bracket
#: what QTM actually delivers. The floor must hold at ALL of them — that it held
#: only at 0.0 is the defect these parameters exist to prevent recurring.
NOISE_LEVELS = (0.0, 0.1, 0.5)


def _arc(a0, a1, n=200, radius=80.0, centre=CENTRE, noise_mm=0.0, rng=None):
    """*n* points on a circle in the XY plane spanning [a0, a1] radians.

    With *noise_mm* > 0, isotropic Gaussian marker noise is added — the QTM
    condition under which a stubby arc's circle fit collapses.
    """
    ang = np.linspace(a0, a1, n)
    pts = np.stack([
        centre[0] + radius * np.cos(ang),
        centre[1] + radius * np.sin(ang),
        np.full(n, centre[2]),
    ], axis=1)
    if noise_mm:
        rng = rng if rng is not None else np.random.default_rng(42)
        pts = pts + rng.normal(0.0, noise_mm, pts.shape)
    return pts


def _sweep_dataset(span_deg, n_sweep=200, n_hold=250, noise_mm=0.0, seed=42):
    """A 5-marker dataset whose sweep covers *span_deg*, then holds still.

    Mirrors the real capture: a sweep phase followed by a stationary hold (the
    yaw-offset calculation uses the last ~200 samples). The hold points land on
    the sweep's final position, so they add no arc — which is exactly the
    property the floor depends on. The hold carries the same *noise_mm* as the
    sweep: a stationary marker is not a noiseless one, and the hold is what the
    yaw-offset uncertainty is measured from.
    """
    rng = np.random.default_rng(seed)
    data = {}
    for i in range(5):
        centre = BB_POS + Z_OFFSETS[i] * AXIS
        sweep = _arc(0.0, math.radians(span_deg), n_sweep,
                     radius=RADII[i], centre=centre, noise_mm=noise_mm, rng=rng)
        hold = np.tile(sweep[-1], (n_hold, 1))
        if noise_mm:
            hold = hold + rng.normal(0.0, noise_mm, hold.shape)
        data[i] = [p for p in np.vstack([sweep, hold])]
    return data


def _yaw_readings(data, span_deg, offset_rad=-0.053, origin=BB_POS,
                  n_sweep=40, n_hold=20):
    """BB's reported yaw series for a *span_deg* sweep, then a hold.

    ⚠ It must SWEEP. mocap_node accumulates these from BB heartbeats across the
    whole CALIBRATING state, so the real series traces the motion and only then
    settles; ``calculate_yaw_offset`` reads the last ~10 (the hold) while the
    primary sweep-completeness gate reads the whole series. A constant list —
    what these fixtures used before the yaw gate existed — is what a BB that
    never moved would report, so it is not a simplification of the real signal
    but an instance of the failure case.
    """
    m3_end = np.array(data[2])[-1]
    angle = math.atan2(m3_end[1] - origin[1], m3_end[0] - origin[0])
    end_deg = math.degrees(angle - offset_rad)
    sweep = list(np.linspace(end_deg - span_deg, end_deg, n_sweep))
    return sweep + [end_deg] * n_hold


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

@pytest.mark.parametrize('noise_mm', NOISE_LEVELS)
def test_truncated_sweep_is_refused_by_name(noise_mm):
    """The whole point: 200 points per marker — five times ``min_points`` — and
    it is still refused, because the sweep only covered 5°.

    ⚠ AT EVERY NOISE LEVEL. This is the case the marker-only floor got wrong:
    at 0.1 mm the same 5° arc's circle fit collapses to a 9.6 mm radius reading
    56° of span, and at 0.5 mm to 2.9 mm reading 328° — both comfortably over
    the 20° floor. The refusal now comes from BB's yaw series, which reads 5.0°
    regardless.
    """
    data = _sweep_dataset(5.0, noise_mm=noise_mm)
    with pytest.raises(ValueError) as exc:
        run_calibration(data, _yaw_readings(data, 5.0),
                        pitch_z_offset_mm=PITCH_Z_OFFSET_MM)
    assert 'ARC_SPAN_TOO_SMALL' in str(exc.value)


@pytest.mark.parametrize('noise_mm', NOISE_LEVELS)
def test_marker_path_alone_refuses_a_noisy_truncated_sweep(noise_mm):
    """The marker path must ALSO refuse, with no yaw series to lean on.

    ``find_rotation_axis`` is called directly here, so the primary yaw gate
    never runs — this is the direct-caller path, and the state the code was in
    before this fix: at 0.1–0.5 mm noise every marker's span read wide, all five
    were accepted, and a garbage axis came out. Now the collapsed radius
    excludes them and the refusal is a refusal, whichever check trips.
    """
    data = _sweep_dataset(5.0, noise_mm=noise_mm)
    trajectories = {k: np.array(v) for k, v in data.items()}
    with pytest.raises(ValueError) as exc:
        find_rotation_axis(trajectories)
    assert ('ARC_SPAN_TOO_SMALL' in str(exc.value)
            or 'sufficient data' in str(exc.value))


@pytest.mark.parametrize('noise_mm', NOISE_LEVELS)
def test_collapsed_marker_fit_is_excluded_by_radius(noise_mm):
    """The collapse signature itself, named. A 110 mm constellation that fits a
    single-digit radius has measured its own noise ball, not the sweep."""
    if noise_mm == 0.0:
        pytest.skip('no collapse without noise — that is the whole finding')
    data = _sweep_dataset(5.0, noise_mm=noise_mm)
    trajectories = {k: np.array(v) for k, v in data.items()}
    with pytest.raises(ValueError) as exc:
        find_rotation_axis(trajectories)
    assert 'collapsed circle fit' in str(exc.value), \
        'the refusal must name the collapse, not generic "poor visibility"'

    # And the fits really are collapsed: RADII are 75-110 mm, two orders of
    # magnitude above what these arcs fit.
    for idx, pts in trajectories.items():
        _, radius, _, _ = fit_circle_3d(pts)
        assert radius < MIN_MARKER_RADIUS_MM, (
            f'marker {idx} fitted {radius:.1f} mm — expected a collapsed fit '
            f'(true radius {RADII[idx]:.0f} mm)')


def test_truncated_sweep_would_have_been_accepted_without_the_floor():
    """Pins the danger, not just the fix: clean 5° data passes cleanly when the
    floors are lowered, which is what the pre-F4 code did on every truncated
    sweep — a confident BB pose fitted to 5° of arc."""
    data = _sweep_dataset(5.0)
    result = run_calibration(data, _yaw_readings(data, 5.0),
                             pitch_z_offset_mm=PITCH_Z_OFFSET_MM,
                             min_arc_deg=0.0, min_radius_mm=0.0)
    assert result.bb_position_mm is not None


@pytest.mark.parametrize('noise_mm', [0.1, 0.5])
def test_without_the_floors_a_noisy_truncated_sweep_blames_the_wrong_subsystem(
        noise_mm):
    """The OTHER half of the danger, and the one that would have been seen on
    hardware. With the floors disarmed, a noisy truncated sweep does not sail
    through — it dies further down, in the pre-existing axis-deviation check,
    whose message reads 'non-rigid motion or poor marker visibility'. That sends
    the operator to inspect markers and lighting for a fault whose actual cause
    is that the sweep never finished. Refusing early, by name, is the fix; this
    test pins what the late refusal says so a regression is recognisable.
    """
    data = _sweep_dataset(5.0, noise_mm=noise_mm)
    with pytest.raises(ValueError) as exc:
        run_calibration(data, _yaw_readings(data, 5.0),
                        pitch_z_offset_mm=PITCH_Z_OFFSET_MM,
                        min_arc_deg=0.0, min_radius_mm=0.0)
    assert 'marker visibility' in str(exc.value)
    assert 'ARC_SPAN_TOO_SMALL' not in str(exc.value)


def test_sweep_just_above_the_floor_succeeds():
    """The floor rejects truncation, not a slow-but-real sweep."""
    data = _sweep_dataset(25.0)
    result = run_calibration(data, _yaw_readings(data, 25.0),
                             pitch_z_offset_mm=PITCH_Z_OFFSET_MM)
    assert np.linalg.norm(result.bb_position_mm[:2] - BB_POS[:2]) < 0.5


@pytest.mark.parametrize('noise_mm', NOISE_LEVELS)
def test_full_sweep_succeeds_at_every_noise_level(noise_mm):
    """The other half of the contract: the gate must not cost a real sweep.

    A 180° sweep is unaffected by marker noise at these levels — the fitted
    radius stays within 0.1 mm of the true 110 mm (see the module docstring's
    table) — so a genuine calibration still lands, and lands accurately.
    """
    data = _sweep_dataset(180.0, noise_mm=noise_mm)
    result = run_calibration(data, _yaw_readings(data, 180.0),
                             pitch_z_offset_mm=PITCH_Z_OFFSET_MM)
    # Tolerance scales with the noise the fit had to average away.
    assert np.linalg.norm(result.bb_position_mm[:2] - BB_POS[:2]) < (
        0.5 + 2.0 * noise_mm)
    assert result.yaw_span_deg == pytest.approx(180.0, abs=0.5)
    assert all(m.status == 'ok' for m in result.marker_metrics.values())


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

    result = run_calibration(data, _yaw_readings(data, 180.0),
                             pitch_z_offset_mm=PITCH_Z_OFFSET_MM)
    assert np.linalg.norm(result.bb_position_mm[:2] - BB_POS[:2]) < 0.5
    assert result.marker_metrics[0].status == 'skipped'
    assert 'arc span' in result.marker_metrics[0].reason
    assert result.marker_metrics[1].status == 'ok'


def test_one_noisy_stubby_marker_is_excluded_by_radius_not_kept():
    """The inversion, pinned. A noisy stubby marker used to read the WIDEST span
    of the five and was therefore the one marker guaranteed to be kept — its
    collapsed centre poisoning the axis average that four good markers had
    otherwise determined. The radius floor drops it instead."""
    data = _sweep_dataset(180.0)
    rng = np.random.default_rng(7)
    stub_centre = BB_POS + Z_OFFSETS[0] * AXIS
    data[0] = [p for p in _arc(0.0, math.radians(4.0), 200,
                               radius=RADII[0], centre=stub_centre,
                               noise_mm=0.5, rng=rng)]

    result = run_calibration(data, _yaw_readings(data, 180.0),
                             pitch_z_offset_mm=PITCH_Z_OFFSET_MM)
    m0 = result.marker_metrics[0]
    assert m0.status == 'skipped', f'noisy stub was KEPT: {m0}'
    assert 'radius' in m0.reason
    assert m0.arc_span_deg > MIN_ARC_DEG, (
        'precondition: the noisy stub reads a WIDE span — that is why the span '
        f'check alone cannot exclude it (read {m0.arc_span_deg:.1f}°)')
    assert np.linalg.norm(result.bb_position_mm[:2] - BB_POS[:2]) < 0.5


def test_metrics_carry_the_measured_span():
    """The span is reported, not just tested against — the owner confirms or
    raises MIN_ARC_DEG from the first hardware calibrate's logged value
    (plans/active/operator-observability.md § 8), and mocap_node logs it."""
    data = _sweep_dataset(180.0)
    result = run_calibration(data, _yaw_readings(data, 180.0),
                             pitch_z_offset_mm=PITCH_Z_OFFSET_MM)
    assert result.marker_metrics[2].arc_span_deg == pytest.approx(179.0, abs=2.0)


def test_result_carries_the_yaw_span_the_owner_reads():
    """§ 8 item 4 is confirmed from the YAW span, so it has to leave the
    pipeline: mocap_node logs ``result.yaw_span_deg``, and the per-marker
    arc_span beside it is the noise-inflated number, not this one."""
    data = _sweep_dataset(180.0, noise_mm=0.5)
    result = run_calibration(data, _yaw_readings(data, 180.0),
                             pitch_z_offset_mm=PITCH_Z_OFFSET_MM)
    assert result.yaw_span_deg == pytest.approx(180.0, abs=0.5)


# ── the yaw series: the primary measurement ─────────────────────────────────

@pytest.mark.parametrize('span_deg', [5.0, 25.0, 90.0, 180.0])
def test_angular_span_recovers_a_yaw_sweep(span_deg):
    assert angular_span_deg(
        np.linspace(0.0, span_deg, 40)) == pytest.approx(span_deg, abs=0.01)


def test_angular_span_is_branch_cut_safe():
    """Same ``360° − largest gap`` construction as the 2-D form, and needed for
    the same reason: BB's yaw origin has no relationship to where a dropout
    happens to stop the sweep."""
    assert angular_span_deg(
        np.linspace(178.0, 183.0, 40)) == pytest.approx(5.0, abs=0.01)
    assert angular_span_deg(
        np.linspace(-2.0, 3.0, 40)) == pytest.approx(5.0, abs=0.01)
    assert angular_span_deg(
        np.linspace(358.0, 363.0, 40)) == pytest.approx(5.0, abs=0.01)


def test_angular_span_fails_closed():
    assert angular_span_deg([]) == 0.0
    assert angular_span_deg([42.0]) == 0.0
    assert angular_span_deg([1.0, float('nan')]) == 0.0
    assert angular_span_deg([7.0] * 20) == 0.0   # never moved


def test_angular_span_is_immune_to_the_noise_that_defeats_the_marker_form():
    """The finding in one assertion. Identical 5° sweeps: the marker-derived
    span blows past the floor once realistic noise is added, the yaw-derived
    span does not move."""
    noisy = _arc(0.0, math.radians(5.0), 200, radius=110.0,
                 centre=BB_POS, noise_mm=0.5, rng=np.random.default_rng(42))
    centre, radius, normal, _ = fit_circle_3d(noisy)
    assert radius < MIN_MARKER_RADIUS_MM      # the fit collapsed…
    assert arc_span_deg(noisy, centre, normal) > MIN_ARC_DEG   # …so the span lies
    assert angular_span_deg(np.linspace(0.0, 5.0, 40)) < MIN_ARC_DEG  # yaw does not


def test_yaw_gate_is_skipped_when_too_few_readings_to_mean_anything():
    """Below MIN_YAW_READINGS the run still fails — at the yaw-offset step, with
    its own accurate message. Reporting 'the sweep did not complete' for what is
    really 'BB barely heartbeated' would send the operator to the wrong place,
    and the gate cannot be load-bearing here because the later refusal is
    unconditional."""
    data = _sweep_dataset(180.0)
    with pytest.raises(ValueError) as exc:
        run_calibration(data, [0.0, 0.0, 0.0],
                        pitch_z_offset_mm=PITCH_Z_OFFSET_MM)
    assert 'ARC_SPAN_TOO_SMALL' not in str(exc.value)
    assert 'yaw readings' in str(exc.value)


def test_yaw_offset_uncertainty_survives_the_atan2_branch_cut():
    """A REAL rejection this used to cause, not a synthetic one.

    ``calculate_yaw_offset`` averages Marker 3's global angle circularly and
    then measured its spread with an UNWRAPPED difference. With BB at
    (-707, -149) and Marker 3 held pointing directly away from the origin — an
    ordinary place for the constellation to stop — the hold sits on the ±π
    branch cut, so 0.1 mm of marker noise scatters samples between +π and −π and
    the unwrapped spread reads ±242.8° (true value ±0.053°). ``max_yaw_std_deg``
    is 5°, so a perfectly good calibration was refused for 'uncertainty too
    high'. The deviations are wrapped now.
    """
    data = _sweep_dataset(180.0, noise_mm=0.1)
    m3_end = np.array(data[2])[-1]
    on_cut = abs(abs(math.atan2(m3_end[1] - BB_POS[1],
                                m3_end[0] - BB_POS[0])) - math.pi)
    assert on_cut < 0.05, 'precondition: the hold pose sits on the branch cut'

    result = run_calibration(data, _yaw_readings(data, 180.0),
                             pitch_z_offset_mm=PITCH_Z_OFFSET_MM)
    assert result.yaw_offset_std_deg < 1.0, (
        f'branch-cut noise inflated the uncertainty to '
        f'{result.yaw_offset_std_deg:.1f}°')


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
