"""Tests for the toss AIM calibration fit — ``tests/hardware/toss_fit_lib.py``
and its thin CLI ``tests/hardware/toss_cal_fit.py`` (build phase 2c of
``plans/active/toss-selftuning.md``).

THE TEST THAT MATTERS
---------------------
:func:`test_closed_loop_the_fitted_map_cancels_the_injected_bias` is the phase's
acceptance gate and everything else here supports it. It injects a **known** aim
bias into a replayed corpus, runs the **real** fit (``fit_nodes`` →
``anchor_estimate`` → ``build_map_document`` → the production
``toss_cal.parse_toss_cal``), installs the resulting map, replays a second corpus
**through the production apply path** (``toss_cal.lookup`` feeding
``aim_target_offset_mm``), and asserts the landing error collapses.

That shape is borrowed from ``tests/motion/test_tilt_cal_grid.py``'s
home-referencing tests, and it is chosen over the obvious alternative — restating
the residual as an assertion — for the reason the plan gives in as many words: a
restated residual drifts in lockstep with a sign flip and would pass on a map
that aims the machine twice as badly as no map at all. Here a sign flip does not
merely fail the assertion, it fails it by DOUBLING the error, and
:func:`test_a_sign_flipped_fit_doubles_the_error` pins that consequence
explicitly so a future reader can see what the gate is protecting.

The corpus generator lives in the library (``toss_fit_lib.synthetic_corpus``) and
builds ``land_err_mm`` from the FORWARD production model
``toss_release.aim_target_offset_mm`` — the same function the node calls to apply
the map. The fit inverts it through a *separately computed* Jacobian. So the
round trip exercises the production sign convention in both directions instead of
restating one of them.

Fixture-backed numbers cite ``tests/ros/toss_record_fixtures.py``, mined from
``2026-08-10_16-30-44`` — the only real corpus that exists — and the re-derived
timing gate is validated against it rather than against a constant the plant does
not honour.
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys

import numpy as np
import pytest
import yaml

_TESTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_HW_DIR = os.path.join(_TESTS, 'hardware')
if _HW_DIR not in sys.path:
    sys.path.insert(0, _HW_DIR)
_ROS_TESTS = os.path.join(_TESTS, 'ros')
if _ROS_TESTS not in sys.path:
    sys.path.insert(0, _ROS_TESTS)

import toss_fit_lib as fit                                       # noqa: E402
import toss_cal_fit as cli                                       # noqa: E402
import toss_record_fixtures as bagfix                            # noqa: E402

import jugglebot.hardware_config as hw                           # noqa: E402
from jugglebot import toss_record                                # noqa: E402
from jugglebot.motion import toss_cal                            # noqa: E402
from jugglebot.motion.trajectory.toss_release import (           # noqa: E402
    aim_target_offset_mm)


GRID_X = [-150.0, 0.0, 150.0]
GRID_Y = [-150.0, 0.0, 150.0]
Z_MM = 170.0
APEX_M = 0.78
T_REF = math.sqrt(8.0 * APEX_M / 9.80665)

_CAPTURED = {'date': '2026-08-11', 'git_sha': 'deadbee', 'tool': fit.TOOL_NAME,
             'args': '--corpus test'}
_REQUIRES = {'tilt_map_version': 'synthetic-tilt/1',
             'level_offset_rad': [0.0, 0.0],
             'estimator_version': toss_cal.ESTIMATOR_VERSION,
             'estimator': {'plane_mm_rule': 'test', 'band_mm': 300.0,
                           'lateral_gate_mm': 300.0, 'min_samples': 5}}
_JACOBIAN = {'S': [[0.0, 1.0], [-1.0, 0.0]], 'gain_mm_per_rad': 3126.53,
             'source': 'test'}


def _document(records, *, x=GRID_X, y=GRID_Y, previous=None, n_min=fit.N_MIN,
              captured=None):
    """The REAL pipeline, end to end: reduce → home-reference → document."""
    fits, _admitted, _excluded = fit.fit_nodes(records, x, y, n_min=n_min,
                                               previous=previous)
    anchor = fit.anchor_estimate(fit.anchor_visits(records, x, y))
    return fit.build_map_document(x, y, Z_MM, fits, anchor,
                                  captured=dict(captured or _CAPTURED),
                                  requires=dict(_REQUIRES),
                                  jacobian=dict(_JACOBIAN)), fits, anchor


# ── the production Jacobian: the one place the sign lives ────────────────────


def test_the_aim_jacobian_is_a_rotation_not_a_scaled_identity():
    """The single most dangerous wrong intuition in this phase.

    ``S`` is ``[[0, 1], [-1, 0]]``: a tilt about **+rx** moves the ball in
    **−y**, and a tilt about **+ry** moves it in **+x**. A naive identity ``S``
    would answer an x-axis landing error with an x-axis tilt and push the ball
    sideways instead of back — which is why the plan makes SC-0 measure ``S``
    rather than assume it, and why this module derives it from the production
    apply path instead of writing it down.
    """
    J = fit.aim_landing_jacobian(T_REF, Z_MM)
    gain = float(np.linalg.norm(J[:, 1]))
    S = J / gain
    assert S[0][0] == pytest.approx(0.0, abs=1e-12)
    assert S[1][1] == pytest.approx(0.0, abs=1e-12)
    assert S[0][1] == pytest.approx(1.0, abs=1e-9)
    assert S[1][0] == pytest.approx(-1.0, abs=1e-9)
    # And the off-diagonal really does dominate — an identity would invert this.
    assert abs(J[0][1]) > 100.0 * (abs(J[0][0]) + 1e-12)


@pytest.mark.parametrize('h', [0.45, 0.60, 0.78, 1.00])
def test_jacobian_magnitude_matches_the_designs_four_h_within_one_percent(h):
    """The design's ``K = 4h`` (mm/rad, h in m) is an idealisation that drops the
    ``Δz`` and drop terms. Keeping the exact production form is worth 0.2 %; this
    pins that the two never diverge further, so a future reader may keep using
    ``4h`` for back-of-envelope work."""
    T = math.sqrt(8.0 * h / 9.80665)
    gain = float(np.linalg.norm(fit.aim_landing_jacobian(T, Z_MM)[:, 1]))
    assert gain == pytest.approx(4.0 * h * 1000.0, rel=0.01)


def test_jacobian_refuses_a_non_positive_flight_time():
    with pytest.raises(fit.TossFitError):
        fit.aim_landing_jacobian(0.0, Z_MM)


# ── the reduction, and why it is a fixed point ───────────────────────────────


def test_reduce_to_aim_recovers_the_negated_plant_bias_with_no_aim_applied():
    psi = (math.radians(0.25), math.radians(-0.10))
    offset = aim_target_offset_mm(psi[0], psi[1], T_REF, Z_MM)
    rec = toss_record.blank_record()
    rec.update({'total_aim_rad': [0.0, 0.0],
                'land_err_mm': [float(offset[0]), float(offset[1])],
                'flight_time_s': T_REF, 'achieved_flight_s_mocap': T_REF,
                'goal_catch_xyz_stow_mm': [0.0, 0.0, Z_MM]})
    b = fit.reduce_to_aim(rec)
    assert b[0] == pytest.approx(-psi[0], abs=1e-6)
    assert b[1] == pytest.approx(-psi[1], abs=1e-6)


def test_reduce_to_aim_is_a_fixed_point_independent_of_the_applied_bias():
    """§ 3.7 item 3's operational claim, made checkable: a capture may run WITH
    the previous map installed, because the applied aim is recorded per toss.

    Two tosses of the same plant, one thrown unaimed and one thrown with a large
    aim already applied, must reduce to the SAME commanded aim. If they did not,
    every capture would have to run with the map uninstalled and the fit would
    be a one-shot measurement rather than a converging fixed point.
    """
    psi = (math.radians(0.30), math.radians(0.18))
    reduced = []
    for applied in [(0.0, 0.0), (math.radians(-0.25), math.radians(0.05))]:
        total = (applied[0] + psi[0], applied[1] + psi[1])
        offset = aim_target_offset_mm(total[0], total[1], T_REF, Z_MM)
        rec = toss_record.blank_record()
        rec.update({'total_aim_rad': list(applied),
                    'land_err_mm': [float(offset[0]), float(offset[1])],
                    'flight_time_s': T_REF, 'achieved_flight_s_mocap': T_REF,
                    'goal_catch_xyz_stow_mm': [0.0, 0.0, Z_MM]})
        reduced.append(fit.reduce_to_aim(rec))
    assert reduced[0][0] == pytest.approx(reduced[1][0], abs=2e-6)
    assert reduced[0][1] == pytest.approx(reduced[1][1], abs=2e-6)
    assert reduced[0][0] == pytest.approx(-psi[0], abs=2e-6)


def test_reduce_refuses_a_record_whose_applied_aim_is_unknown():
    """A mined-only row (no ``/toss/record`` declaration — every row of the three
    2026-08-10 bags) cannot say what aim was applied. Guessing zero would make
    the fit re-learn a correction the machine is already making."""
    rec = toss_record.blank_record()
    rec.update({'land_err_mm': [10.0, 5.0], 'flight_time_s': T_REF,
                'goal_catch_xyz_stow_mm': [0.0, 0.0, Z_MM]})
    assert fit.applied_aim_rad(rec) is None
    with pytest.raises(fit.TossFitError):
        fit.reduce_to_aim(rec)


# ── THE ACCEPTANCE TEST ──────────────────────────────────────────────────────


def test_closed_loop_the_fitted_map_cancels_the_injected_bias():
    """Inject a KNOWN bias → run the REAL fit → apply the map through the REAL
    lookup → the landing error collapses.

    Round 1 records a plant with aim bias ψ and no map. The fit produces a
    document; the document is parsed by the **production** loader. Round 2
    replays the same plant with that map applied exactly as
    ``reload_coordinator_node._toss_aim_for_goal`` applies it — ``toss_cal.lookup``
    at the goal's catch xy, clamped, then fed to ``aim_target_offset_mm``.

    The uncorrected mean landing error is ~15 mm per axis; corrected it must be
    under 1 mm. This is the whole phase in one assertion.

    Note what the map alone cannot do: the grid is home-referenced, so it carries
    only the SPATIAL part. The common mode lives in ``anchor.aim_rad`` and is
    phase 2e's warm-start prior — so the corrected replay below applies
    ``lookup + anchor``, which is exactly what a session with the trim warm-started
    from the map would command.
    """
    psi = (math.radians(0.28), math.radians(-0.15))
    corpus = fit.synthetic_corpus(GRID_X, GRID_Y, plant_bias_rad=psi,
                                  n_per_node=40, sigma_mm=0.0, z_mm=Z_MM,
                                  apex_m=APEX_M)
    doc, fits, anchor = _document(corpus)
    cal = fit.validate_map_document(doc)

    # Sign AND magnitude, at the anchor: the map's absolute correction is -psi.
    assert anchor[0][0] == pytest.approx(-psi[0], abs=1e-6)
    assert anchor[0][1] == pytest.approx(-psi[1], abs=1e-6)
    # A constant field ships a ZERO grid — all of it is common mode.
    assert np.allclose(np.asarray(doc['aim_rad']['rx']), 0.0, atol=1e-9)
    assert np.allclose(np.asarray(doc['aim_rad']['ry']), 0.0, atol=1e-9)

    def _apply(x, y):
        rx, ry = toss_cal.lookup(cal, x, y)
        rx, ry, _hits = toss_cal.clamp_total_aim(
            rx + cal.anchor_aim_rad[0], ry + cal.anchor_aim_rad[1])
        return (rx, ry)

    before = fit.synthetic_corpus(GRID_X, GRID_Y, plant_bias_rad=psi,
                                  n_per_node=4, sigma_mm=0.0, z_mm=Z_MM,
                                  apex_m=APEX_M)
    after = fit.synthetic_corpus(GRID_X, GRID_Y, plant_bias_rad=psi,
                                 applied_aim_rad_fn=_apply,
                                 n_per_node=4, sigma_mm=0.0, z_mm=Z_MM,
                                 apex_m=APEX_M)
    err_before = np.abs(np.asarray([r['land_err_mm'] for r in before])).mean()
    err_after = np.abs(np.asarray([r['land_err_mm'] for r in after])).mean()
    assert err_before > 8.0, 'the injected bias must be worth measuring'
    assert err_after < 1.0, (
        'the fitted map did not cancel the bias it was fitted from: '
        '{:.2f} mm -> {:.2f} mm'.format(err_before, err_after))


def test_a_sign_flipped_fit_doubles_the_error():
    """What the acceptance test is protecting, stated as its own assertion.

    The design's § 3.7 item 3 writes the reduction with a PLUS
    (``b = A + S⁻¹·land_err/4h``); the fixed point is the MINUS. Applying the
    plus-signed answer does not merely fail to help — it commands the bias
    instead of its cancellation and roughly DOUBLES the landing error, which is
    the failure mode rung SC-0 is a blocking gate for.
    """
    psi = (math.radians(0.28), math.radians(-0.15))
    corpus = fit.synthetic_corpus(GRID_X, GRID_Y, plant_bias_rad=psi,
                                  n_per_node=20, sigma_mm=0.0, z_mm=Z_MM,
                                  apex_m=APEX_M)
    correct = fit.anchor_estimate(fit.anchor_visits(corpus, GRID_X, GRID_Y))[0]
    flipped = (-correct[0], -correct[1])

    def _err(applied):
        rows = fit.synthetic_corpus(GRID_X, GRID_Y, plant_bias_rad=psi,
                                    applied_aim_rad_fn=lambda x, y: applied,
                                    n_per_node=2, sigma_mm=0.0, z_mm=Z_MM,
                                    apex_m=APEX_M)
        return float(np.abs(np.asarray([r['land_err_mm'] for r in rows])).mean())

    baseline = _err((0.0, 0.0))
    assert _err(correct) < 0.05 * baseline
    assert _err(flipped) > 1.8 * baseline


def test_closed_loop_recovers_a_SPATIAL_field_node_by_node():
    """A bias that varies over the workspace must come back as the shipped
    HOME-REFERENCED grid, node for node — not merely as a good average.

    This is the test that would catch a transposed ``[iy][ix]`` index, which is
    invisible on a symmetric field, so the injected field is deliberately
    asymmetric in x and y.
    """
    def psi(x, y):
        return (math.radians(0.10 + 0.0010 * x / 150.0 * 150.0 / 150.0 * x / 150.0),
                math.radians(-0.05 + 0.0015 * y))

    corpus = fit.synthetic_corpus(GRID_X, GRID_Y, plant_bias_rad=psi,
                                  n_per_node=30, sigma_mm=0.0, z_mm=Z_MM,
                                  apex_m=APEX_M)
    doc, _fits, anchor = _document(corpus)
    cal = fit.validate_map_document(doc)
    home = psi(0.0, 0.0)
    for iy, y in enumerate(GRID_Y):
        for ix, x in enumerate(GRID_X):
            want_rx = -(psi(x, y)[0] - home[0])
            want_ry = -(psi(x, y)[1] - home[1])
            got = toss_cal.lookup(cal, x, y)
            assert got[0] == pytest.approx(want_rx, abs=2e-6), (x, y)
            assert got[1] == pytest.approx(want_ry, abs=2e-6), (x, y)
    assert anchor[0][0] == pytest.approx(-home[0], abs=2e-6)
    assert anchor[0][1] == pytest.approx(-home[1], abs=2e-6)


def test_closed_loop_under_realistic_noise_lands_inside_the_predicted_se():
    """σ = 20 mm is the design's working assumption (F6 inverts the 2026-08-10
    retest to σ ≈ 26). At the 3126 mm/rad production gain that is 6.4 mrad per
    toss, so a 270-toss pooled estimate has ``se = 6.4/√270`` = 0.39 mrad.

    The bound is **4·se**, not the textbook 3, and the number is measured rather
    than chosen: the per-node estimator is a 10 %-trimmed mean, which at n = 30
    drops 3 samples per tail and inflates the estimator's variance ~1.2× over the
    plain mean. A 20-seed probe (2026-08-11) puts the observed pooled error at a
    max of **3.27 se** with an empirical sd of 1.19 se, so 4 se is ~3.4 empirical
    sigma — tight enough that a sign flip (which sits at 2·bias/se ≈ 20 se) or a
    transposed axis cannot pass, loose enough not to be a flake generator.
    """
    psi = (math.radians(0.22), math.radians(-0.11))
    corpus = fit.synthetic_corpus(GRID_X, GRID_Y, plant_bias_rad=psi,
                                  n_per_node=30, sigma_mm=20.0, z_mm=Z_MM,
                                  apex_m=APEX_M, seed=7)
    fits, _adm, _exc = fit.fit_nodes(corpus, GRID_X, GRID_Y)
    pooled = (float(np.mean([f.rx for f in fits.values()])),
              float(np.mean([f.ry for f in fits.values()])))
    n_total = sum(f.n for f in fits.values())
    se = (20.0 / 3126.53) / math.sqrt(n_total)
    assert abs(pooled[0] + psi[0]) < 4.0 * se
    assert abs(pooled[1] + psi[1]) < 4.0 * se
    # A sign flip would sit here — an order of magnitude outside the bound.
    assert abs(pooled[0] - psi[0]) > 20.0 * se


# ── home-referencing: the level-noise immunity ───────────────────────────────


def test_a_constant_shift_in_the_plant_bias_leaves_the_shipped_grid_alone():
    """THE property home-referencing exists for (§ 3.2). ``level`` is one
    int16-quantised SCL3300 sample with 1.2–1.7 mrad/axis of session-to-session
    scatter, so an absolute map would bake one session's level noise into every
    future session. Add that scatter to the whole field and the SHIPPED grid must
    not move, with the entire difference absorbed by ``anchor.aim_rad``.

    **Not byte-identical, and this is the honest form of the tilt map's claim.**
    ``tilt_cal_grid``'s equivalent is exact because its reduction is a plain
    subtraction of inclinometer readings. This reduction runs through the
    ballistic model, which is linear only to second order in the aim, so a
    2.4 mrad common shift perturbs each node by at most **2.6e-7 rad** on this
    fixture (measured 2026-08-11) — **8e-4 mm** of landing shift at h = 0.78 m,
    three orders below the 0.15° flat-field floor and far below anything the
    machine can command. The bound below is 1e-6 rad, ~4x that; a real failure of
    home-referencing would show up at the size of the shift itself, 2.4e-3, i.e.
    four orders larger, so the bound has no room to hide one.
    """
    def field(x, y):
        return (math.radians(0.05) + 4e-5 * x, math.radians(-0.03) + 4e-5 * y)

    shift = (0.0024, -0.0017)     # ~the worst observed level-to-level jump

    clean, _f, _a = _document(fit.synthetic_corpus(
        GRID_X, GRID_Y, plant_bias_rad=field, n_per_node=12, sigma_mm=0.0,
        z_mm=Z_MM, apex_m=APEX_M))
    stale, _f2, _a2 = _document(fit.synthetic_corpus(
        GRID_X, GRID_Y,
        plant_bias_rad=lambda x, y: (field(x, y)[0] + shift[0],
                                     field(x, y)[1] + shift[1]),
        n_per_node=12, sigma_mm=0.0, z_mm=Z_MM, apex_m=APEX_M))

    for axis in ('rx', 'ry'):
        assert np.allclose(np.asarray(stale['aim_rad'][axis]),
                           np.asarray(clean['aim_rad'][axis]), atol=1e-6), axis
    assert toss_cal.map_version(stale) != toss_cal.map_version(clean), (
        'the anchor DID move, and the version hash covers anchor.aim_rad')
    assert stale['anchor']['aim_rad'][0] == pytest.approx(
        clean['anchor']['aim_rad'][0] - shift[0], abs=1e-6)
    assert stale['anchor']['aim_rad'][1] == pytest.approx(
        clean['anchor']['aim_rad'][1] - shift[1], abs=1e-6)


def test_the_home_node_ships_exactly_zero_when_it_is_the_only_anchor():
    corpus = fit.synthetic_corpus(GRID_X, GRID_Y,
                                  plant_bias_rad=(0.004, -0.002),
                                  n_per_node=10, sigma_mm=0.0)
    doc, _f, _a = _document(corpus)
    hx, hy = fit.home_index(GRID_X, GRID_Y)
    assert doc['aim_rad']['rx'][hy][hx] == 0.0
    assert doc['aim_rad']['ry'][hy][hx] == 0.0


def test_a_grid_without_the_origin_is_refused():
    with pytest.raises(fit.TossFitError):
        fit.home_index([-150.0, -50.0, 150.0], GRID_Y)


def test_the_lookup_clamps_to_the_hull_and_never_extrapolates():
    """C-LEVEL-2's doctrine, applied to the aim map: outside the calibrated hull
    a query returns the nearest hull point, because a wrong-signed edge
    extrapolation aims a throw WORSE than no map at all."""
    corpus = fit.synthetic_corpus(
        GRID_X, GRID_Y,
        plant_bias_rad=lambda x, y: (4e-5 * x, 4e-5 * y),
        n_per_node=10, sigma_mm=0.0)
    cal = fit.validate_map_document(_document(corpus)[0])
    edge = toss_cal.lookup(cal, 150.0, 150.0)
    beyond = toss_cal.lookup(cal, 900.0, 900.0)
    assert beyond == edge


# ── the two guards that refuse a write (§ 3.8) ───────────────────────────────


def test_the_flat_field_guard_refuses_a_map_smaller_than_theta_acc():
    """A map whose largest home-referenced node is under the tilt map's own
    accuracy floor cannot be distinguished from that layer's noise: it buys
    nothing and costs a version bump every consumer must re-verify."""
    corpus = fit.synthetic_corpus(
        GRID_X, GRID_Y,
        # a pure common mode: large, but with NO spatial structure at all
        plant_bias_rad=(math.radians(0.4), math.radians(-0.3)),
        n_per_node=10, sigma_mm=0.0)
    _doc, fits, anchor = _document(corpus)
    is_flat, message = fit.flat_field_verdict(fits, anchor[0])
    assert is_flat
    assert 'FLAT FIELD' in message and '--allow-flat-field' in message


def test_the_flat_field_guard_passes_a_real_field():
    corpus = fit.synthetic_corpus(
        GRID_X, GRID_Y,
        plant_bias_rad=lambda x, y: (math.radians(0.4) * x / 150.0, 0.0),
        n_per_node=10, sigma_mm=0.0)
    _doc, fits, anchor = _document(corpus)
    assert fit.flat_field_verdict(fits, anchor[0])[0] is False


def test_the_ball_flew_guard_refuses_a_corpus_in_which_nothing_flew():
    """The toss analogue of the 2026-07-15 DISARMED tilt capture that wrote a
    plausible all-zeros map because the platform never moved."""
    corpus = fit.synthetic_corpus(GRID_X, GRID_Y, plant_bias_rad=(0.003, 0.0),
                                  n_per_node=10, sigma_mm=0.0)
    for rec in corpus:
        rec['label'] = toss_record.LABEL_UNKNOWN
    fits, _adm, _exc = fit.fit_nodes(corpus, GRID_X, GRID_Y)
    ok, message = fit.ball_flew_verdict(fits, None)
    assert ok is False
    assert 'BALL NEVER FLEW' in message


def test_the_ball_flew_guard_refuses_a_never_measured_node():
    """D15 keeps a thin node's PREVIOUS value; it cannot conjure one. Shipping a
    zero would drag the measured neighbours toward zero through the bilinear
    blend, inventing calibration where the machine has never been."""
    corpus = fit.synthetic_corpus(GRID_X, GRID_Y, plant_bias_rad=(0.003, 0.0),
                                  n_per_node=10, sigma_mm=0.0)
    corpus = [r for r in corpus
              if r['goal_catch_xyz_stow_mm'][:2] != [150.0, 150.0]]
    fits, _adm, _exc = fit.fit_nodes(corpus, GRID_X, GRID_Y)
    ok, message = fit.ball_flew_verdict(fits, None)
    assert ok is False
    assert 'no previous map is installed' in message


def test_a_thin_node_keeps_its_previous_value_and_is_marked_stale():
    """§ 3.7 item 7 / D15, the deliberate deviation from the tilt tool's
    all-or-nothing write: refusing would block 24 good nodes because one had a
    thin week. And the carried value is the PREVIOUS one, never an interpolation
    of the neighbours — that would invent calibration exactly where the machine
    had trouble."""
    full = fit.synthetic_corpus(GRID_X, GRID_Y,
                                plant_bias_rad=lambda x, y: (4e-5 * x, 4e-5 * y),
                                n_per_node=12, sigma_mm=0.0)
    previous = fit.validate_map_document(_document(full)[0])

    thin = [r for r in full
            if r['goal_catch_xyz_stow_mm'][:2] != [150.0, 150.0]]
    thin += [r for r in full
             if r['goal_catch_xyz_stow_mm'][:2] == [150.0, 150.0]][:2]
    fits, _adm, _exc = fit.fit_nodes(thin, GRID_X, GRID_Y, previous=previous)
    corner = fits[(2, 2)]
    assert corner.stale is True and corner.source == 'previous' and corner.n == 2
    assert corner.rx == pytest.approx(float(previous.rx_rad[2][2]), abs=1e-12)
    ok, _msg = fit.ball_flew_verdict(fits, previous)
    assert ok is True

    doc, _f, anchor = _document(thin, previous=previous)
    assert doc['stats']['stale_nodes'] == [
        {'ix': 2, 'iy': 2, 'x_mm': 150.0, 'y_mm': 150.0, 'n': 2,
         'source': 'previous'}]


def test_a_previous_map_on_a_different_grid_is_not_carried_by_index():
    """``[iy][ix]`` on a different grid reads a different physical pose. That is
    the transposed-index class of error dressed up as a refresh, so the previous
    map is used only when its axes match node for node."""
    other = fit.validate_map_document(_document(
        fit.synthetic_corpus([-100.0, 0.0, 100.0], [-100.0, 0.0, 100.0],
                             plant_bias_rad=(0.004, 0.0), n_per_node=10,
                             sigma_mm=0.0),
        x=[-100.0, 0.0, 100.0], y=[-100.0, 0.0, 100.0])[0])
    corpus = fit.synthetic_corpus(GRID_X, GRID_Y, plant_bias_rad=(0.004, 0.0),
                                  n_per_node=10, sigma_mm=0.0)
    corpus = [r for r in corpus
              if r['goal_catch_xyz_stow_mm'][:2] != [150.0, 150.0]]
    fits, _adm, _exc = fit.fit_nodes(corpus, GRID_X, GRID_Y, previous=other)
    assert fits[(2, 2)].source == 'never_measured'
    assert fit.ball_flew_verdict(fits, other)[0] is False


# ── version stability ────────────────────────────────────────────────────────


def test_identical_numbers_produce_an_identical_version():
    corpus = fit.synthetic_corpus(GRID_X, GRID_Y,
                                  plant_bias_rad=lambda x, y: (4e-5 * x, 4e-5 * y),
                                  n_per_node=10, sigma_mm=0.0)
    first = _document(corpus)[0]
    second = _document(corpus, captured={'date': '2026-08-11',
                                         'git_sha': 'OTHER-SHA',
                                         'tool': fit.TOOL_NAME,
                                         'args': 'a completely different call'})[0]
    assert toss_cal.map_version(first) == toss_cal.map_version(second), (
        'the version hashes the numbers the machine ACTS ON, so re-running the '
        'fit must not churn it')


def test_one_node_changing_changes_the_version():
    corpus = fit.synthetic_corpus(GRID_X, GRID_Y,
                                  plant_bias_rad=lambda x, y: (4e-5 * x, 4e-5 * y),
                                  n_per_node=10, sigma_mm=0.0)
    base = _document(corpus)[0]
    bumped = json.loads(json.dumps(base))
    bumped['aim_rad']['rx'][0][0] += 1e-6
    assert toss_cal.map_version(bumped) != toss_cal.map_version(base)


def test_the_written_yaml_reloads_through_the_production_loader(tmp_path):
    corpus = fit.synthetic_corpus(GRID_X, GRID_Y,
                                  plant_bias_rad=lambda x, y: (4e-5 * x, 4e-5 * y),
                                  n_per_node=10, sigma_mm=0.0)
    doc = _document(corpus)[0]
    path = tmp_path / 'toss_calibration.yaml'
    path.write_text(fit.dump_map_yaml(doc))
    loaded = toss_cal.load_toss_cal(str(path))
    assert loaded.version == toss_cal.map_version(doc)
    assert 'do not hand-edit' in path.read_text()
    assert loaded.anchor_aim_rad is not None


# ── partitioning ─────────────────────────────────────────────────────────────


def test_a_cross_partition_fit_is_refused_and_the_census_names_the_partitions():
    """§ 7 R3: a corpus pooling a degraded and a restored plant yields a map that
    is the weighted average of two machines, and it LOOKS fine — the residual
    mean is still a number and the sd only widens a little."""
    a = fit.synthetic_corpus(GRID_X, GRID_Y, plant_bias_rad=(0.004, 0.0),
                             n_per_node=4, sigma_mm=0.0)
    b = fit.synthetic_corpus(GRID_X, GRID_Y, plant_bias_rad=(-0.004, 0.0),
                             n_per_node=4, sigma_mm=0.0)
    for rec in b:
        rec['bridge_fw_version'] = '9 (proto 5)'
    with pytest.raises(fit.TossFitError) as excinfo:
        fit.select_partition(a + b)
    assert 'refusing to fit across 2 partitions' in str(excinfo.value)
    assert 'bridge_fw_version' in str(excinfo.value)

    rows, census, key, warnings = fit.select_partition(a + b, allow_cross=True)
    assert len(rows) == len(a) + len(b) and len(census) == 2 and key is None
    assert any('CROSSING' in w for w in warnings)


def test_an_unknown_partition_key_is_reported_never_assumed_to_match():
    """§ 8: ``hand_odrive_config_sha`` ships nullable and a null is a KNOWN GAP
    in the partition key, not an assumed match."""
    corpus = fit.synthetic_corpus(GRID_X, GRID_Y, plant_bias_rad=(0.004, 0.0),
                                  n_per_node=2, sigma_mm=0.0)
    _rows, _census, _key, warnings = fit.select_partition(corpus)
    assert any('hand_odrive_config_sha' in w for w in warnings)


def test_the_partition_key_reads_z_from_the_goal_pose():
    rec = toss_record.blank_record()
    rec['goal_catch_xyz_stow_mm'] = [10.0, 20.0, 170.0]
    assert fit.partition_key(rec)[fit.PARTITION_KEYS.index('z_mm')] == 170.0


# ── label inclusion (§ 3.7 item 4) ───────────────────────────────────────────


@pytest.mark.parametrize('label,admitted', [
    (toss_record.LABEL_CAUGHT, True),
    (toss_record.LABEL_BOUNCED, True),
    (toss_record.LABEL_MISSED, True),
    (toss_record.LABEL_NO_RELEASE, False),
    (toss_record.LABEL_UNKNOWN, False),
])
def test_label_inclusion_rule(label, admitted):
    rec = fit.synthetic_corpus([-1.0, 0.0, 1.0], [-1.0, 0.0, 1.0],
                               plant_bias_rad=(0.0, 0.0), n_per_node=1,
                               sigma_mm=0.0)[0]
    rec['label'] = label
    assert fit.admit_for_aim(rec)[0] is admitted


def test_a_MISSED_toss_needs_a_non_sparse_mocap_fit():
    """MISSED records are the most informative aim records there are — excluding
    them biases the whole map toward the cup — but a thin track on a missed ball
    is exactly the case where the estimator extrapolates."""
    rec = fit.synthetic_corpus([-1.0, 0.0, 1.0], [-1.0, 0.0, 1.0],
                               plant_bias_rad=(0.0, 0.0), n_per_node=1,
                               sigma_mm=0.0)[0]
    rec['label'] = toss_record.LABEL_MISSED
    rec['n_fit'] = 3
    assert fit.admit_for_aim(rec) == (False, 'missed_with_thin_track')
    rec['label'] = toss_record.LABEL_CAUGHT
    assert fit.admit_for_aim(rec)[0] is True


@pytest.mark.parametrize('field,value,reason', [
    ('fit_sparse', True, 'mocap_fit_sparse'),
    ('fit_rms_mm', 9.0, 'mocap_fit_quality'),
    ('retry_of', 'syn-0-0-0', 'retry_cycle'),
    ('reload_settle', True, 'reload_settle'),
    ('apex_height_m', 0.40, 'apex_out_of_band'),
])
def test_the_per_record_guards_name_their_own_refusal(field, value, reason):
    rec = fit.synthetic_corpus([-1.0, 0.0, 1.0], [-1.0, 0.0, 1.0],
                               plant_bias_rad=(0.0, 0.0), n_per_node=1,
                               sigma_mm=0.0)[0]
    rec[field] = value
    assert fit.admit_for_aim(rec) == (False, reason)


def test_an_off_node_check_pose_is_never_snapped_into_a_node():
    """SC-3's verification poses are chosen OFF-node precisely so they are an
    independent check; snapping one into the nearest node would launder the check
    into the fit it exists to test."""
    rec = toss_record.blank_record()
    rec['goal_catch_xyz_stow_mm'] = [75.0, -40.0, Z_MM]
    assert fit.node_of(rec, GRID_X, GRID_Y) is None
    rec['goal_catch_xyz_stow_mm'] = [150.0, 0.0, Z_MM]
    assert fit.node_of(rec, GRID_X, GRID_Y) == (2, 1)


# ── the RE-DERIVED timing gate (design § 10 hands this to 2c) ────────────────


def test_the_timing_gate_admits_the_cadence_this_plant_actually_runs_at():
    """§ 3.7 item 5 gated on ``sensor_poll_dt_ms_median`` within 10 % of the
    configured ``JB_BD_CHECK_INTERVAL_MS`` (20 ms). MEASURED per-record on
    ``2026-08-10_16-30-44``: 60 / 63 / 70 / 80 / 87 ms
    (min / p5 / median / p95 / max). The shipped gate refuses 100 % of records —
    it is not a gate, it is an outage."""
    assert bagfix.POLL_DT_MS_MEDIAN == pytest.approx(71.0, abs=1.0)
    assert float(hw.JB_BD_CHECK_INTERVAL_MS) * 1.1 < bagfix.POLL_DT_MS_MEDIAN, (
        'the SHIPPED gate would refuse the reference bag entirely')
    rec = _timing_record(bagfix.POLL_DT_MS_MEDIAN)
    assert fit.admit_for_timing(rec) == (True, '')


@pytest.mark.parametrize('dt_ms,reason', [
    (5.0, 'poll_cadence_below_configured_interval'),
    (400.0, 'poll_cadence_unreachable'),
    (None, 'poll_cadence_unmeasured'),
])
def test_the_timing_gate_refuses_both_ends_for_different_reasons(dt_ms, reason):
    """The FLOOR is physics: a cadence below the firmware's own poll interval
    means the stamp is not the poll stamp. The CEILING is reachability: at
    Δ = 200 ms a 5 ms standard error needs n = 133 admitted tosses, more than the
    entire 129-toss first capture."""
    rec = _timing_record(dt_ms)
    assert fit.admit_for_timing(rec) == (False, reason)


def test_the_timing_ceiling_is_the_cadence_at_which_a_fit_stops_being_reachable():
    n_required = (fit.TIMING_POLL_DT_MS_MAX / math.sqrt(12.0)
                  / fit.TAU_SE_MAX_MS) ** 2
    assert n_required == pytest.approx(133.0, abs=1.0)


def test_the_measured_departure_dispersion_IS_the_poll_quantisation():
    """The finding that re-derived the gate, pinned so a future reader does not
    re-litigate it: over the reference bag's 31 self-tosses the departure-shift
    sd is 20.51 ms against a uniform-quantisation prediction of Δ/√12 =
    20.50 ms. The release timing is more repeatable than the instrument can see,
    so the honest gate is a PRECISION gate on Δ, not a match against a constant
    the plant does not honour."""
    measured = float(np.std(np.asarray(bagfix.DEPARTURE_DT_MS), ddof=1))
    predicted = bagfix.POLL_DT_MS_MEDIAN / math.sqrt(12.0)
    assert measured == pytest.approx(predicted, rel=0.05)


def test_timing_fit_reports_a_quantisation_consistent_standard_error():
    rows = [_timing_record(70.0, shift_ms=s) for s in bagfix.DEPARTURE_DT_MS]
    out = fit.timing_fit(rows)
    assert out['n'] == len(bagfix.DEPARTURE_DT_MS)
    assert out['tau_ms'] == pytest.approx(float(np.median(bagfix.DEPARTURE_DT_MS)),
                                          abs=0.5)
    assert out['se_ms'] == pytest.approx(
        out['sd_ms'] / math.sqrt(out['n']), rel=1e-9)
    assert out['applies'] is True


def _timing_record(dt_ms, shift_ms=175.0):
    rec = toss_record.blank_record()
    rec.update({'announce_throw_time_ros': 1786343595.0,
                't_departure_raw_ros': 1786343595.0 + shift_ms / 1e3,
                'ball_held_stamp_wall_anchored': True,
                'sensor_poll_dt_ms_median': dt_ms,
                'uptime_ms_at_release': 60000})
    return rec


# ── speed fit ────────────────────────────────────────────────────────────────


def test_speed_fit_recovers_a_known_height_shortfall():
    """``k̂_v = √(mean(h_cmd/h_ach))`` (§ 3.6.1). A plant throwing 10 % low in
    apex needs ``k_v = 1.049`` on the commanded velocity."""
    rows = []
    for _ in range(20):
        rec = toss_record.blank_record()
        rec.update({'label': toss_record.LABEL_CAUGHT,
                    'flight_time_s': T_REF,
                    'achieved_flight_s_mocap': T_REF * math.sqrt(0.90)})
        rows.append(rec)
    out = fit.speed_fit(rows)
    assert out['n'] == 20
    assert out['k_v'] == pytest.approx(math.sqrt(1.0 / 0.90), rel=1e-9)
    assert out['applies'] is True


# ── population statistics ────────────────────────────────────────────────────


def test_sigma_is_centred_per_axis_so_a_common_bias_does_not_inflate_it():
    """A common bias is exactly what the map removes; folding it into σ would
    make the pre-calibration scatter look larger than the plant's and the
    post-calibration improvement look like variance reduction it is not."""
    unbiased = fit.synthetic_corpus(GRID_X, GRID_Y, plant_bias_rad=(0.0, 0.0),
                                    n_per_node=30, sigma_mm=20.0, seed=3)
    biased = fit.synthetic_corpus(GRID_X, GRID_Y,
                                  plant_bias_rad=(math.radians(0.3), 0.0),
                                  n_per_node=30, sigma_mm=20.0, seed=3)
    assert fit.sigma_land_mm(unbiased) == pytest.approx(
        fit.sigma_land_mm(biased), rel=0.02)
    assert fit.sigma_land_mm(unbiased) == pytest.approx(20.0, rel=0.10)


def test_r_eff_inverts_the_rayleigh_model_of_F6():
    """At zero bias ``P(catch) = 1 − exp(−R²/2σ²)``, so a plant with the
    geometric 35 mm radius and σ = 20 mm must read back R_eff ≈ 35."""
    corpus = fit.synthetic_corpus(GRID_X, GRID_Y, plant_bias_rad=(0.0, 0.0),
                                  n_per_node=200, sigma_mm=20.0, seed=11)
    assert fit.r_eff_mm(corpus) == pytest.approx(hw.GEOM_HAND_RADIUS_MM, rel=0.10)


def test_score_groups_reports_the_continuous_observables_not_just_the_label():
    a = fit.synthetic_corpus(GRID_X, GRID_Y, plant_bias_rad=(0.0, 0.0),
                             n_per_node=6, sigma_mm=10.0, seed=1)
    b = fit.synthetic_corpus(GRID_X, GRID_Y, plant_bias_rad=(0.0, 0.0),
                             n_per_node=6, sigma_mm=30.0, seed=2)
    rows = fit.score_groups(a + b, [('A', a), ('B', b)])
    assert rows[0]['sigma_mm'] < rows[1]['sigma_mm']
    assert rows[0]['mean_err_mm'] is not None
    assert rows[0]['catch_rate'] is not None


# ── the CLI ──────────────────────────────────────────────────────────────────


def _write_corpus(tmp_path, records, name='corpus.jsonl'):
    path = tmp_path / name
    with open(path, 'w') as handle:
        for rec in records:
            handle.write(toss_record.encode(rec) + '\n')
    return str(path)


def test_dry_run_writes_nothing(tmp_path, capsys):
    """``--dry-run`` prints the node-by-node diff and writes nothing."""
    corpus = _write_corpus(tmp_path, fit.synthetic_corpus(
        GRID_X, GRID_Y, plant_bias_rad=lambda x, y: (4e-5 * x, 4e-5 * y),
        n_per_node=10, sigma_mm=0.0))
    target = tmp_path / 'toss_calibration.yaml'
    rc = cli.main(['--corpus', corpus, '--out', str(target), '--dry-run',
                   '--allow-cross-partition'])
    out = capsys.readouterr().out
    assert rc == 0
    assert not target.exists()
    assert 'NOTHING WRITTEN' in out and 'NODES' in out


def test_the_cli_writes_a_map_the_production_loader_accepts(tmp_path, capsys):
    corpus = _write_corpus(tmp_path, fit.synthetic_corpus(
        GRID_X, GRID_Y, plant_bias_rad=lambda x, y: (4e-5 * x, 4e-5 * y),
        n_per_node=10, sigma_mm=0.0))
    target = tmp_path / 'toss_calibration.yaml'
    rc = cli.main(['--corpus', corpus, '--out', str(target)])
    assert rc == 0, capsys.readouterr().out
    loaded = toss_cal.load_toss_cal(str(target))
    out = capsys.readouterr().out
    assert loaded.version in out
    assert 'toss/reload_calibration' in out, (
        'the operator must be told to reload AND read the version back')


def test_the_cli_refuses_a_cross_partition_corpus_without_the_flag(tmp_path,
                                                                   capsys):
    a = fit.synthetic_corpus(GRID_X, GRID_Y,
                             plant_bias_rad=lambda x, y: (4e-5 * x, 0.0),
                             n_per_node=10, sigma_mm=0.0)
    b = fit.synthetic_corpus(GRID_X, GRID_Y,
                             plant_bias_rad=lambda x, y: (4e-5 * x, 0.0),
                             n_per_node=10, sigma_mm=0.0)
    for rec in b:
        rec['platform_fw_version'] = '3'
    corpus = _write_corpus(tmp_path, a + b)
    target = tmp_path / 'toss_calibration.yaml'
    rc = cli.main(['--corpus', corpus, '--out', str(target)])
    assert rc == 1
    assert not target.exists()
    assert 'refusing to fit across' in capsys.readouterr().out


def test_the_cli_refuses_a_flat_field_and_the_override_stamps_the_artefact(
        tmp_path, capsys):
    corpus = _write_corpus(tmp_path, fit.synthetic_corpus(
        GRID_X, GRID_Y, plant_bias_rad=(math.radians(0.4), 0.0),
        n_per_node=10, sigma_mm=0.0))
    target = tmp_path / 'toss_calibration.yaml'
    assert cli.main(['--corpus', corpus, '--out', str(target)]) == 1
    assert not target.exists()
    assert 'FLAT FIELD' in capsys.readouterr().out

    assert cli.main(['--corpus', corpus, '--out', str(target),
                     '--allow-flat-field']) == 0
    doc = yaml.safe_load(target.read_text())
    assert doc['captured']['flat_field_override'] is True


def test_the_cli_stamps_the_partition_census_and_the_source_digests(tmp_path):
    records = fit.synthetic_corpus(
        GRID_X, GRID_Y, plant_bias_rad=lambda x, y: (4e-5 * x, 0.0),
        n_per_node=10, sigma_mm=0.0)
    corpus = _write_corpus(tmp_path, records)
    target = tmp_path / 'toss_calibration.yaml'
    assert cli.main(['--corpus', corpus, '--out', str(target)]) == 0
    doc = yaml.safe_load(target.read_text())
    census = doc['captured']['partition_census']
    assert len(census) == 1 and census[0]['n'] == len(records)
    assert census[0]['hand_odrive_config_sha'] is None, (
        'a null partition key is a KNOWN GAP, recorded as one'
    )
    digests = doc['captured']['source_files']
    assert digests[0]['sha256'] == fit.sha256_of(corpus)
    assert doc['requires']['estimator_version'] == toss_cal.ESTIMATOR_VERSION
    assert doc['jacobian']['S'] == [[0.0, 1.0], [-1.0, 0.0]]


def test_the_cli_refuses_an_even_node_count():
    with pytest.raises(fit.TossFitError):
        cli.build_axis(150.0, 4, 'x')


def test_the_write_target_is_the_first_production_candidate(monkeypatch):
    """Never a walk of the tool's own location: the resolver's order is what
    makes the tool write the file the node then reloads."""
    monkeypatch.setenv(toss_cal.TOSS_CAL_ENV, '/tmp/named-by-the-operator.yaml')
    assert fit.write_target_path() == '/tmp/named-by-the-operator.yaml'


def test_no_ros_import_at_module_scope_in_either_file():
    """The whole fit is desk-side: a tool that needs a sourced graph to tell you
    what it WOULD do is a tool nobody runs, and ``tests/motion/`` must be able to
    import the CLI on a box with no ROS at all.

    Checked by AST over the source rather than by ``'rclpy' not in
    sys.modules`` — the sibling ``tests/ros/conftest.py`` installs MOCK ROS
    modules into ``sys.modules``, so under the parallel gate that assertion
    measures which file the worker ran first, not this file's imports. The
    ``--reload`` path imports rclpy INSIDE its function, which is the point.
    """
    import ast
    for name in ('toss_fit_lib.py', 'toss_cal_fit.py'):
        tree = ast.parse(open(os.path.join(_HW_DIR, name)).read())
        top = [n for n in tree.body if isinstance(n, (ast.Import, ast.ImportFrom))]
        modules = []
        for node in top:
            if isinstance(node, ast.Import):
                modules += [a.name for a in node.names]
            else:
                modules.append(node.module or '')
        offenders = [m for m in modules
                     if m.split('.')[0] in ('rclpy', 'std_msgs', 'std_srvs',
                                            'jugglebot_interfaces',
                                            'geometry_msgs')]
        assert not offenders, (name, offenders)
    assert cli.main.__module__ == 'toss_cal_fit'


def test_the_tool_runs_as_a_script_and_reports_its_refusal(tmp_path):
    """One end-to-end subprocess run, because ``__main__`` wiring and the
    ``sys.path`` bootstrap at the top of the file are exactly what an in-process
    import cannot check."""
    env = dict(os.environ)
    env['JUGGLEBOT_TOSS_CAL'] = str(tmp_path / 'nonexistent.yaml')
    proc = subprocess.run(
        [sys.executable, os.path.join(_HW_DIR, 'toss_cal_fit.py'),
         '--corpus', str(tmp_path / 'missing.jsonl'), '--dry-run'],
        capture_output=True, text=True, env=env, timeout=180)
    assert proc.returncode == 1
    assert 'REFUSED' in proc.stdout
