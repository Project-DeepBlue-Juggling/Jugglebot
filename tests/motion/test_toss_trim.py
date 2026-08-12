"""Tests for the LAYER-2 session trim — ``jugglebot/toss_trim.py``, build phase
**2e** of ``plans/active/toss-selftuning.md``.

THE FIVE GATES THIS PHASE IS ACCEPTED ON
----------------------------------------
1. the trim NEVER exceeds ``TRIM_MAX`` (:func:`test_the_trim_never_exceeds_its_clamp`);
2. the TOTAL is re-clamped **at apply**, over ``map + trim``
   (:func:`test_the_total_is_reclamped_at_apply_over_map_plus_trim`, and at the
   node in ``tests/ros/test_toss_trim_node.py``);
3. every guard path FREEZES rather than zeroes
   (:func:`test_every_guard_path_freezes_and_never_zeroes`);
4. a synthetic constant bias at σ = 20 mm converges inside the clamp and STOPS
   (:func:`test_a_constant_bias_at_sigma_20mm_converges_inside_the_clamp_and_stops`);
5. a zero trim is today's machine bit for bit (the node half of this composes
   with 2b's identity gate and lives in ``tests/ros/test_toss_trim_node.py``;
   the pure half is :func:`test_a_warmup_trim_is_exactly_zero`).

WHY THE SYNTHETIC CORPUS COMES FROM ``toss_fit_lib``
----------------------------------------------------
``toss_fit_lib.synthetic_corpus`` builds ``land_err_mm`` through the FORWARD
production model (``toss_release.aim_target_offset_mm``) and the trim reduces it
through a *separately computed* Jacobian, so the round trip exercises the real
sign convention in both directions rather than restating one of them. Re-writing
a corpus generator here would give the trim its own private physics and the whole
point of phase 2e's move — one reduction, one admission filter, shared by the
online trim and the offline fit — would be lost in its own test file.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest

_TESTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_HW_DIR = os.path.join(_TESTS, 'hardware')
if _HW_DIR not in sys.path:
    sys.path.insert(0, _HW_DIR)

import toss_fit_lib as fit                                       # noqa: E402

from jugglebot import toss_record, toss_trim                     # noqa: E402
from jugglebot.motion import toss_cal                            # noqa: E402
from jugglebot.motion.trajectory.toss_release import (           # noqa: E402
    aim_target_offset_mm)

GRID_X = [-150.0, 0.0, 150.0]
GRID_Y = [-150.0, 0.0, 150.0]
Z_MM = 170.0
APEX_M = 0.78
T_REF = math.sqrt(8.0 * APEX_M / 9.80665)
#: The production aim→landing gain at the reference geometry, mm/rad. Measured,
#: not assumed — the same number ``toss_trim``'s constants table cites.
GAIN = float(np.linalg.norm(
    toss_trim.aim_landing_jacobian(T_REF, Z_MM)[:, 1]))


def _corpus(plant_bias_rad, *, n=90, sigma_mm=20.0, seed=7, nodes=1,
            applied=None):
    """A single-node (or grid) corpus carrying a constant PLANT BIAS ψ.

    **Sign, stated once so every assertion below can be read at a glance.** ψ is
    the equivalent aim the plant is already applying by itself — the corpus
    lands where a machine tilted by ψ would land. The trim's job is to COMMAND
    the aim that cancels it, so the expected trim is ``−ψ`` (:func:`_cancels`).
    That minus is ``reduce_to_aim``'s fixed point (``b = A − J⁻¹·land_err =
    −ψ``, independent of A) and it is the same minus phase 2c had to correct the
    design text on.

    ``nodes=1`` collapses the grid to the home node, which is what a session
    trim actually sees on a fixed-pose sitting; the trim pools across nodes by
    construction so the multi-node case is the same estimator with more spread.
    """
    bias_rad = plant_bias_rad
    xs = [0.0] if nodes == 1 else GRID_X
    ys = [0.0] if nodes == 1 else GRID_Y
    per = max(1, int(round(n / (len(xs) * len(ys)))))
    return fit.synthetic_corpus(
        xs, ys, plant_bias_rad=bias_rad, n_per_node=per, sigma_mm=sigma_mm,
        seed=seed, z_mm=Z_MM, apex_m=APEX_M,
        applied_aim_rad_fn=applied,
        label_fn=lambda err: toss_record.LABEL_CAUGHT)


def _mag(pair):
    return float(math.hypot(float(pair[0]), float(pair[1])))


def _cancels(plant_bias_rad):
    """The commanded trim that cancels a plant bias — its negative. See
    :func:`_corpus`."""
    return (-float(plant_bias_rad[0]), -float(plant_bias_rad[1]))


# ── gate 1: the clamp ─────────────────────────────────────────────────────────

@pytest.mark.parametrize('bias_deg', [0.0, 0.05, 0.12, 0.30, 0.90])
@pytest.mark.parametrize('seed', [1, 2, 3, 4, 5])
def test_the_trim_never_exceeds_its_clamp(bias_deg, seed):
    """GATE 1. Whatever the plant does, ``|δ| ≤ TRIM_MAX``.

    Swept over biases from nothing to 6× the clamp, because the interesting
    failure is not "the estimator is wrong" but "the estimator is right about a
    bias larger than the authority it was given" — which is exactly the case a
    per-axis clamp would let through at ``hypot`` = 1.414× the bound.
    """
    bias = (math.radians(bias_deg), math.radians(bias_deg) * 0.4)
    trim = toss_trim.replay(_corpus(bias, seed=seed))
    assert _mag(trim.aim()) <= toss_trim.TRIM_MAX_RAD + 1e-15


def test_the_clamp_preserves_direction_and_never_rotates_the_aim():
    """A per-axis clamp shortens one component and turns the aim; the magnitude
    clamp shortens both and keeps it pointing where the estimate said."""
    rx, ry, clamped = toss_trim.clamp_trim(0.01, 0.02)
    assert clamped is True
    assert _mag((rx, ry)) == pytest.approx(toss_trim.TRIM_MAX_RAD, rel=1e-12)
    assert (ry / rx) == pytest.approx(2.0, rel=1e-12)


def test_a_warmup_trim_is_exactly_zero():
    """GATE 5, pure half. A fresh trim — and a trim that has seen only
    refusable records — commands exactly ``(0.0, 0.0)``, so the node adds
    nothing and the composed path is bit-identical to 2b."""
    trim = toss_trim.SessionTrim()
    assert trim.aim() == (0.0, 0.0)
    assert trim.state == toss_trim.STATE_WARMUP
    for rec in _corpus((0.005, 0.0), n=20):
        rec['land_err_mm'] = None          # the LIVE record's state today
        trim.observe(rec)
    assert trim.aim() == (0.0, 0.0)
    assert trim.n == 0


def test_the_live_record_shape_is_refused_by_name_not_silently_ignored():
    """The phase's central finding, pinned. ``land_err_mm`` is a MINED field, so
    a record the node can build today carries none — and the trim must say
    ``no_mocap_fit`` rather than quietly learning nothing.

    If a future build adds a live arrival estimator, this test is the one that
    should start failing.
    """
    trim = toss_trim.SessionTrim()
    rec = _corpus((0.005, 0.0), n=1)[0]
    rec['land_err_mm'] = None
    verdict = trim.observe(rec)
    assert verdict.admitted is False
    assert verdict.reason == 'no_mocap_fit'
    assert trim.snapshot()['refusals'] == {'no_mocap_fit': 1}


# ── gate 2: the TOTAL clamp, at apply ─────────────────────────────────────────

def test_the_total_is_reclamped_at_apply_over_map_plus_trim():
    """GATE 2, pure half. Each layer bounds ITSELF — 1.0° for the map, 0.15° for
    the trim — and 1.15° is past the authority every downstream argument (the
    C-LEVEL-2 composition regime, the cup swing, the 55 mm landing shift) is
    sized on. The only place that sum exists is the apply seam, so the clamp has
    to be there and not at either update.
    """
    map_aim = (toss_cal.TOTAL_MAX_RAD, 0.0)          # a saturated map node
    trim_aim = (toss_trim.TRIM_MAX_RAD, 0.0)         # a saturated session trim
    rx, ry, hits = toss_cal.clamp_total_aim(map_aim[0] + trim_aim[0],
                                            map_aim[1] + trim_aim[1])
    assert hits == ['total_aim']
    assert _mag((rx, ry)) == pytest.approx(toss_cal.TOTAL_MAX_RAD, rel=1e-12)
    # And the un-clamped sum really would have been past it — otherwise this
    # test would pass on a build where the clamp had been deleted.
    assert map_aim[0] + trim_aim[0] > toss_cal.TOTAL_MAX_RAD


# ── gate 3: freeze, never zero ────────────────────────────────────────────────

def _frozen_by_outliers():
    trim = toss_trim.SessionTrim()
    corpus = _corpus((math.radians(0.12), 0.0), n=40, sigma_mm=1.0, seed=11)
    for rec in corpus[:20]:
        trim.observe(rec)
    held = trim.aim()
    for rec in corpus[20:23]:
        rec['land_err_mm'] = [4000.0, 4000.0]        # ~1.3 rad of "aim"
        trim.observe(rec)
    return trim, held


def _frozen_by_cusum():
    trim = toss_trim.SessionTrim()
    for rec in _corpus((math.radians(0.05), 0.0), n=30, sigma_mm=20.0, seed=3):
        trim.observe(rec)
    held = trim.aim()
    # A 5σ step: exactly the braking-clamp CLASS of regime change G8 exists for.
    shifted = _corpus((math.radians(0.05), 0.0), n=30, sigma_mm=20.0, seed=4)
    for rec in shifted:
        err = rec['land_err_mm']
        rec['land_err_mm'] = [err[0] + 100.0, err[1]]
        trim.observe(rec)
    return trim, held


def _frozen_by_provenance():
    trim = toss_trim.SessionTrim()
    corpus = _corpus((math.radians(0.10), 0.0), n=30, sigma_mm=2.0, seed=5)
    for rec in corpus[:20]:
        trim.observe(rec)
    held = trim.aim()
    for rec in corpus[20:]:
        rec['tilt_map_version'] = 'a-DIFFERENT-tilt-map/9'
        trim.observe(rec)
    return trim, held


def _frozen_by_stall():
    # A bias 4x the clamp: the demand can never fall, so the loop must STALL
    # rather than integrate into a plant change.
    trim = toss_trim.SessionTrim()
    corpus = _corpus((math.radians(0.60), 0.0), n=60, sigma_mm=2.0, seed=9)
    for rec in corpus:
        trim.observe(rec)
    return trim, None


@pytest.mark.parametrize('maker,expected', [
    (_frozen_by_outliers, 'FROZEN_OUTLIERS'),
    (_frozen_by_cusum, 'FROZEN_CUSUM'),
    (_frozen_by_provenance, 'FROZEN_LEVELLING_CHANGED'),
    (_frozen_by_stall, 'FROZEN_STALLED'),
])
def test_every_guard_path_freezes_and_never_zeroes(maker, expected):
    """GATE 3. Four independent guard paths, one invariant: the state becomes
    ``FROZEN_*`` and ``δ`` KEEPS its value.

    Zeroing on a guard would inject a ``TRIM_MAX``-sized step into the next
    commanded pose — strictly worse than holding a stale but bounded correction,
    and the design says so in as many words (§ 3.6.3 DEGRADED).
    """
    trim, held = maker()
    assert trim.state == expected, trim.snapshot()
    assert trim.frozen is True
    assert _mag(trim.aim()) <= toss_trim.TRIM_MAX_RAD + 1e-15
    if held is not None:
        assert trim.aim() == held, 'a guard ZEROED the trim instead of freezing it'
    assert trim.snapshot()['reason'], 'a freeze must name its cause'


def test_a_LAYER_ONE_map_reload_mid_goal_does_NOT_freeze_the_trim():
    """G5's session check latches the LAYER-0 tilt map only.

    A layer-1 (aim map) reload mid-goal is harmless to this estimator BY
    CONSTRUCTION: the reduction subtracts the aim the toss actually applied, so a
    record thrown under map A and one thrown under map B reduce to the same
    common-mode demand. That is the fixed-point property § 3.7 item 3 is arguing
    for, and freezing on it would punish the exact workflow the capture routine
    uses — fit, reload, verify. Layer 0 is the one that moves the baseline the
    residual is measured against (D3), and it still freezes.
    """
    trim = toss_trim.SessionTrim()
    corpus = _corpus((math.radians(0.10), 0.0), n=30, sigma_mm=2.0, seed=5)
    for rec in corpus[:15]:
        trim.observe(rec)
    for rec in corpus[15:]:
        rec['toss_cal_version'] = '2026-08-11-a-NEWER-map'
        verdict = trim.observe(rec)
        assert verdict.admitted is True, verdict
    assert not trim.learning_stopped, trim.snapshot()
    assert trim.n == 30


def test_a_frozen_trim_keeps_recording_and_refuses_by_name():
    """"Freeze, keep recording" (§ 3.6.3). The corpus of a frozen goal is
    exactly the corpus an operator needs to see WHY it froze, so the records
    keep arriving — they just stop moving the estimate."""
    trim, held = _frozen_by_cusum()
    n_before = trim.snapshot()['n_seen']
    for rec in _corpus((0.0, 0.0), n=5, seed=77):
        verdict = trim.observe(rec)
        assert verdict.admitted is False
        assert verdict.reason == 'frozen'
        assert verdict.y_rad is not None, 'a frozen trim still REDUCES the toss'
    assert trim.snapshot()['n_seen'] == n_before + 5
    assert trim.aim() == held


def test_the_first_freeze_wins():
    """A session that trips two guards is frozen for the one that stopped it.
    Re-labelling would lose the chronology an operator diagnoses from."""
    trim, _ = _frozen_by_outliers()
    assert trim.state == 'FROZEN_OUTLIERS'
    reason = trim.snapshot()['reason']
    for rec in _corpus((0.0, 0.0), n=10, seed=31):
        rec['tilt_map_version'] = 'another-map/2'
        trim.observe(rec)
    assert trim.state == 'FROZEN_OUTLIERS'
    assert trim.snapshot()['reason'] == reason


# ── gate 4: convergence ───────────────────────────────────────────────────────

def test_a_constant_bias_at_sigma_20mm_converges_inside_the_clamp_and_stops():
    """GATE 4. The headline behavioural gate, at the design's own noise level.

    A constant plant bias of 0.12° (6.5 mm) at σ = 20 mm/axis, over 20
    independent sessions of 240 tosses. Every session must (a) command a
    correction of the right SIGN, (b) land inside the clamp and within a few mm
    of the value that cancels the bias, and (c) STOP — leave the commanded trim
    still for the tail of the session rather than churning the pose forever.

    **Why (c) is asserted on the commanded trim and not on the ``CONVERGED``
    label, which is the finding this test carries.** Measured over these same 20
    seeds: at σ = 20 mm the terminal LABEL is not reachable inside a sitting —
    4/20 sessions reach ``CONVERGED``, 8/20 wander past the 0.15° clamp on the
    way and report ``FROZEN_STALLED``, and the rest are still ``ACTIVE`` at
    n = 240. The VALUE converges regardless (mean error 1.6 mm, worst 6.6 mm),
    because ``se`` shrinks as σ/√n while the label needs ``demand + 2.5·se`` to
    fit inside a 0.10° deadband — which at σ = 20 mm needs n ≈ 340. Asserting
    the label here would be asserting that σ is smaller than it is.

    Both halves of that are operator-facing and both are in the logbook: a
    ``FROZEN_STALLED`` on a real sub-clamp bias is expected at this noise level
    and is NOT by itself evidence of a plant change, and the design's promise
    that the loop "converges and stops" is a promise about the number.
    """
    bias = (math.radians(0.12), 0.0)
    want = _cancels(bias)
    errors_mm = []
    acted = 0
    for seed in range(20):
        trim = toss_trim.replay(_corpus(bias, n=240, sigma_mm=20.0,
                                        seed=100 + seed))
        rx, ry = trim.aim()
        snap = trim.snapshot()
        # (a) sign: the trim CANCELS the plant bias, so it must never come back
        # with the SAME sign. A sign flip is the R1 failure and it must not be
        # merely "less accurate", it must be caught. Zero is allowed — at this
        # noise level a session can legitimately never reach significance, and
        # declining to act is the correct answer when it does not.
        assert rx <= 0.0, (seed, math.degrees(rx))
        acted += rx < 0.0
        # (b) inside the clamp.
        assert _mag((rx, ry)) <= toss_trim.TRIM_MAX_RAD + 1e-15
        if rx < 0.0:
            errors_mm.append(abs(rx - want[0]) * GAIN)
        # (c) it STOPPED. A settled loop moves the commanded pose a handful of
        # times and then leaves it alone; a churning one moves on every update.
        assert snap['changes'] <= 4, (seed, snap)
    assert acted >= 16, acted
    assert float(np.mean(errors_mm)) < 3.0, errors_mm


def test_the_converged_label_is_reachable_once_the_noise_is_small_enough():
    """The other half of GATE 4: the label machinery itself, exercised where the
    arithmetic allows it. σ = 4 mm/axis, which is what the arrival scatter would
    have to be for the ``level`` common mode the trim was sized on to be worth
    correcting at all (see ``toss_trim.SE_GATE``'s table)."""
    bias = (math.radians(0.14), 0.0)
    trim = toss_trim.replay(_corpus(bias, n=90, sigma_mm=4.0, seed=61))
    assert trim.state == toss_trim.STATE_CONVERGED, trim.snapshot()
    assert abs(trim.aim()[0] - _cancels(bias)[0]) * GAIN < 2.0


def test_a_bias_inside_the_clamp_is_recovered_and_converges():
    """The same loop where the authority is sufficient: a 0.12° bias at a low
    noise level is recovered to well inside the deadband and the estimator
    reports CONVERGED rather than freezing."""
    bias = (math.radians(0.12), 0.0)
    want = _cancels(bias)
    trim = toss_trim.replay(_corpus(bias, n=60, sigma_mm=4.0, seed=17))
    rx, ry = trim.aim()
    assert abs(rx - want[0]) < toss_trim.DEADBAND_RAD, (rx, want)
    assert abs(ry) < toss_trim.DEADBAND_RAD
    assert trim.state == toss_trim.STATE_CONVERGED, trim.snapshot()


def test_the_estimate_is_a_fixed_point_independent_of_the_applied_map():
    """§ 3.7 item 3's operational property, at the TRIM.

    Two corpora with the SAME plant bias, one recorded with the map commanding
    nothing and one with it commanding a real aim, must produce the same
    *commanded total*. That is what makes "a session does not have to run with
    the map uninstalled" true, and it is the property the design's written ``+``
    (a diverging feedback loop) does not have.
    """
    bias = (math.radians(0.10), math.radians(0.05))
    applied = (math.radians(0.03), math.radians(-0.02))
    plain = toss_trim.replay(_corpus(bias, n=80, sigma_mm=2.0, seed=23))
    with_map = toss_trim.replay(_corpus(
        bias, n=80, sigma_mm=2.0, seed=23,
        applied=lambda x, y: applied))
    # Asserted on the ESTIMATE (`r_rad`), not on the clamped command: the fixed
    # point is a property of the estimator, and a clamp would launder a
    # disagreement into agreement at the bound.
    #
    # The trim owns the COMMON MODE, i.e. the demand left after the map. With a
    # map already commanding `applied`, the trim's own share shrinks by exactly
    # that much — and map + trim lands in the same place.
    r_plain = np.asarray(plain.snapshot()['r_rad'])
    r_mapped = np.asarray(with_map.snapshot()['r_rad']) + np.asarray(applied)
    assert np.allclose(r_plain, np.asarray(_cancels(bias)),
                       atol=math.radians(0.02)), np.degrees(r_plain)
    assert np.allclose(r_plain, r_mapped, atol=math.radians(0.005)), (
        np.degrees(r_plain), np.degrees(r_mapped))


def test_the_anchor_prior_warm_starts_but_does_not_dominate():
    """``n₀ = 4``: the map's ``anchor.aim_rad`` is a PRIOR, not a correction. A
    session that disagrees with it must win inside one node's worth of data —
    which is the whole reason § 3.2 refuses to ship the anchor as a hard
    number."""
    plant_bias = (math.radians(0.12), 0.0)
    want = _cancels(plant_bias)                       # −0.12°
    wrong_prior = (math.radians(+0.20), 0.0)          # the opposite direction
    trim = toss_trim.replay(_corpus(plant_bias, n=60, sigma_mm=2.0, seed=5),
                            anchor_aim_rad=wrong_prior)
    got = trim.aim()[0]
    assert got < 0.0, 'the prior overrode 56 admitted measurements'
    # Much closer to the measurements than to the prior — the shrinkage mean
    # still carries n₀/(n₀+n) = 6.7 % of the prior at n = 56, which is the
    # design's intent (a warm start), not a defect.
    assert abs(got - want[0]) < abs(got - wrong_prior[0]) / 4.0
    assert abs(got - want[0]) < math.radians(0.03), math.degrees(got)


def test_zero_bias_leaves_the_trim_bounded_and_small():
    """Measured, not asserted from theory. With NOTHING to correct the trim is
    not guaranteed silent — the significance gate is a sequential test — but its
    expected cost is ~1 mm and it is always inside the clamp. Both halves matter:
    the first is why ``toss_trim_enabled`` defaults false, the second is why a
    false action cannot cause a miss on its own."""
    mags = []
    for seed in range(12):
        trim = toss_trim.replay(_corpus((0.0, 0.0), n=72, sigma_mm=20.0,
                                        seed=1000 + seed))
        mags.append(_mag(trim.aim()) * GAIN)
    assert max(mags) <= toss_trim.TRIM_MAX_RAD * GAIN + 1e-9
    assert float(np.mean(mags)) < 4.0, mags


# ── the affine model ──────────────────────────────────────────────────────────

def test_the_affine_model_is_refused_when_rank_deficient():
    """GATE (design § 3.6.1). Three collinear cells span a LINE, so the gradient
    across the perpendicular direction is unidentifiable. A ridge term would
    return a plausible number pointing along the axis the session never sampled;
    the refusal is the correct answer."""
    points = [[x, 0.0] for x in (-150.0, 0.0, 150.0)] * 5
    y = [[1e-3, 0.0]] * len(points)
    result = toss_trim.fit_affine(points, y)
    assert result.ok is False
    assert result.reason.startswith('spread') or result.reason == 'rank_deficient'
    assert result.c_rad is None and result.gradient is None


@pytest.mark.parametrize('points,expected', [
    ([[0.0, 0.0]] * 20, 'cells<3'),
    ([[-150.0, -150.0], [150.0, 150.0], [0.0, 0.0]] * 4, 'rank_deficient'),
    ([[-10.0, -10.0], [10.0, -10.0], [0.0, 10.0]] * 4, 'spread'),
])
def test_the_affine_model_names_which_bar_it_failed(points, expected):
    y = [[1e-3, 0.0]] * len(points)
    result = toss_trim.fit_affine(points, y)
    assert result.ok is False
    assert result.reason.startswith(expected), result.reason


def test_the_affine_model_recovers_a_real_gradient_when_the_geometry_supports_it():
    points = []
    y = []
    grad = 2e-6            # rad per mm
    for px in (-150.0, 0.0, 150.0):
        for py in (-150.0, 0.0, 150.0):
            for _ in range(2):
                points.append([px, py])
                y.append([1e-3 + grad * px, -5e-4 + grad * py])
    result = toss_trim.fit_affine(points, y)
    assert result.ok is True, result.reason
    assert result.c_rad[0] == pytest.approx(1e-3, abs=1e-9)
    assert result.gradient[0][0] == pytest.approx(grad, rel=1e-6)
    assert result.gradient[1][1] == pytest.approx(grad, rel=1e-6)


def test_a_session_trim_never_fits_the_affine_model_by_itself():
    """§ 3.6.1: *"Per-node trim is never fitted online."* The default is the
    2-parameter constant and :class:`SessionTrim` must not reach for
    :func:`fit_affine` on its own — 9–25 under-determined cells produce 9–25
    noise chasers, which is the failure the explicit calibration routine exists
    to avoid."""
    import inspect
    source = inspect.getsource(toss_trim.SessionTrim)
    assert 'fit_affine' not in source


# ── admission: the guards, by name ────────────────────────────────────────────

@pytest.mark.parametrize('mutate,reason', [
    (lambda r: r.update({'label': toss_record.LABEL_UNKNOWN}), 'label_unknown'),
    (lambda r: r.update({'label': toss_record.LABEL_NO_RELEASE}),
     'label_no_release'),
    (lambda r: r.update({'throw_stroke_seen': False,
                         'ball_track_confirmed': False,
                         't_departure_raw_ros': None}), 'no_release_evidence'),
    (lambda r: r.update({'fit_sparse': True}), 'mocap_fit_sparse'),
    (lambda r: r.update({'fit_rms_mm': 9.0}), 'mocap_fit_quality'),
    (lambda r: r.update({'retry_of': 'sess-abcd-3'}), 'retry_cycle'),
    (lambda r: r.update({'reload_settle': True}), 'reload_settle'),
    (lambda r: r.update({'apex_height_m': 0.2}), 'apex_out_of_band'),
    (lambda r: r.update({'dip_below_x3_rev': 0.5}), 'plant_dip_below_x3'),
    (lambda r: r.update({'stroke_peak_rev': 10.8}), 'plant_stroke_peak'),
    (lambda r: r.update({'trunc': True}), 'plant_stroke_truncated'),
    (lambda r: r.update({'gravity_correction_loaded': None}),
     'no_gravity_correction'),
    (lambda r: r.update({'tilt_map_applied': False}), 'no_tilt_map'),
    (lambda r: r.update({'total_aim_rad': None, 'map_aim_rad': None}),
     'applied_aim_unknown'),
])
def test_each_admission_guard_refuses_by_its_own_name(mutate, reason):
    """G1–G5 and G9–G11, one row each. The NAME matters as much as the refusal:
    the proposal document and the console both report the refusal census, and a
    census of ``False`` teaches an operator nothing about which fence they hit.
    """
    rec = _corpus((0.0, 0.0), n=1)[0]
    mutate(rec)
    admitted, why = toss_trim.admit_for_aim(rec)
    assert admitted is False
    assert why == reason


def test_g4_is_admitted_when_the_plant_block_is_absent_and_the_gap_is_counted():
    """The PLANT block ships null (§ 10), so G4 cannot be enforced from a record
    yet. Refusing every record for a field the pipeline does not produce would
    make the guard indistinguishable from an outage; admitting silently is how
    the braking clamp hid for a whole session. So: admit, and COUNT."""
    trim = toss_trim.SessionTrim()
    for rec in _corpus((0.0, 0.0), n=6, sigma_mm=1.0):
        assert rec.get('dip_below_x3_rev') is None
        trim.observe(rec)
    snap = trim.snapshot()
    assert snap['n'] == 6
    assert snap['g4_unenforceable'] == 6


def test_a_null_levelling_flag_is_a_refusal_not_a_pass():
    """G5. "I cannot verify which layer 0 was underneath this toss" is not "the
    right one was" — the same fail-closed rule ``TossCal.provenance_mismatch``
    applies to an unknown live tilt-map version (D3)."""
    rec = _corpus((0.0, 0.0), n=1)[0]
    rec['tilt_map_applied'] = None
    assert toss_trim.admit_g5_levelling(rec) == (False, 'no_tilt_map')


# ── the reduction, shared with the offline fit ────────────────────────────────

def test_the_fit_library_imports_the_reduction_rather_than_copying_it():
    """The phase-2e move, pinned. ``toss_fit_lib``'s names must be the SAME
    objects as this module's — not equal implementations, the same ones.

    A copy would let the ONLINE trim and the OFFLINE fit drift about what a toss
    reduces to or which tosses are admissible, and the map an operator promotes
    would be fitted on a different population from the one the machine learnt on.
    Nothing would ever surface that. Same argument D11 makes for the labeller.
    """
    for name in ('aim_landing_jacobian', 'reduce_to_aim', 'admit_for_aim',
                 'admit_for_timing', 'admit_for_speed', 'applied_aim_rad',
                 'achieved_flight_s', 'achieved_apex_m'):
        assert getattr(fit, name) is getattr(toss_trim, name), name
    for name in ('FIT_RMS_MAX_MM', 'APEX_SANITY_FRAC', 'AIM_LABELS',
                 'TIMING_POLL_DT_MS_MIN', 'TIMING_POLL_DT_MS_MAX',
                 'TAU_SE_MAX_MS', 'SPEED_SE_MAX', 'SPEED_TIMING_N_MIN'):
        assert getattr(fit, name) == getattr(toss_trim, name), name
    assert fit.TossFitError is toss_trim.TossTrimError


def test_the_trim_module_does_not_import_the_map_loader():
    """Operator decision 7: ``reload_coordinator_node`` is the map's SINGLE
    owner, and ``tests/motion/test_toss_cal.py`` pins that structurally. So the
    TOTAL authority clamp stays in ``toss_cal`` and is applied by the node at
    the one apply seam; this module owns only ``TRIM_MAX``."""
    import ast
    path = toss_trim.__file__
    with open(path, encoding='utf-8') as handle:
        tree = ast.parse(handle.read())
    names = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names += [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            names.append(node.module or '')
    assert not any('toss_cal' in n for n in names), names


def test_the_trim_module_imports_no_ros():
    import ast
    with open(toss_trim.__file__, encoding='utf-8') as handle:
        tree = ast.parse(handle.read())
    top = []
    for node in tree.body:
        if isinstance(node, ast.Import):
            top += [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            top.append(node.module or '')
    banned = [m for m in top if m.split('.')[0] in (
        'rclpy', 'std_msgs', 'std_srvs', 'geometry_msgs', 'jugglebot_interfaces',
        'ament_index_python')]
    assert banned == []


# ── the scalar sub-estimators ─────────────────────────────────────────────────

def test_the_speed_gain_stays_at_unity_until_its_own_gate_passes():
    trim = toss_trim.SessionTrim()
    corpus = _corpus((0.0, 0.0), n=4, sigma_mm=1.0)
    for rec in corpus:
        trim.observe(rec)
    assert trim.speed_gain() == 1.0
    assert trim.snapshot()['speed_state'] == toss_trim.STATE_WARMUP


def test_the_speed_gain_recovers_a_real_flight_time_error():
    """A plant throwing 5 % short reads ``T_ach = 0.95·T_cmd``, so ``k_v`` must
    rise toward 1/0.95 = 1.0526 — and never past the ±10 % authority."""
    trim = toss_trim.SessionTrim()
    for rec in _corpus((0.0, 0.0), n=30, sigma_mm=1.0):
        rec['achieved_flight_s_mocap'] = rec['flight_time_s'] * 0.95
        trim.observe(rec)
    assert trim.speed_gain() == pytest.approx(1.0 / 0.95, abs=0.01)
    assert 0.9 <= trim.speed_gain() <= 1.1


def test_the_speed_authority_is_hard_clamped_at_ten_percent():
    trim = toss_trim.SessionTrim()
    for rec in _corpus((0.0, 0.0), n=60, sigma_mm=1.0):
        rec['achieved_flight_s_mocap'] = rec['flight_time_s'] * 0.5
        trim.observe(rec)
    assert trim.speed_gain() <= 1.0 + toss_trim.SPEED_AUTHORITY + 1e-12


def test_tau_is_never_persisted_and_says_so_in_the_proposal():
    """§ 3.6.1: τ is uptime-dependent by nature, so a number written into
    ``config/hardware_config.yaml`` would be a claim about a different plant."""
    trim = toss_trim.SessionTrim(session_id='s1', goal_id='g1')
    for rec in _corpus((0.0, 0.0), n=20, sigma_mm=1.0):
        rec['t_departure_raw_ros'] = rec['announce_throw_time_ros'] + 0.130
        trim.observe(rec)
    doc = trim.proposal()
    assert 'release_latency_ms' in doc['session_local']
    assert 'NEVER persisted' in doc['session_local']['release_latency_note']
    # And it is nowhere in a shape a calibration loader would read.
    assert 'grid' not in doc and 'aim_rad' not in doc


def test_the_timing_gate_refuses_an_unreachable_poll_cadence():
    rec = _corpus((0.0, 0.0), n=1)[0]
    rec['t_departure_raw_ros'] = rec['announce_throw_time_ros'] + 0.13
    rec['sensor_poll_dt_ms_median'] = 400.0
    assert toss_trim.admit_for_timing(rec) == (False, 'poll_cadence_unreachable')


# ── the proposal, and premise P1 ──────────────────────────────────────────────

def test_the_proposal_cannot_be_mistaken_for_a_calibration():
    """PREMISE P1, enforced rather than documented. The end-of-goal artefact is
    a proposal; promotion needs the explicit routine and its acceptance gates.
    So the document the trim writes must be one ``parse_toss_cal`` REFUSES —
    a mistaken ``cp`` into ``config/`` then fails loudly instead of silently
    installing an unreviewed online estimate as a calibration."""
    trim = toss_trim.replay(_corpus((math.radians(0.2), 0.0), n=40,
                                    sigma_mm=2.0, seed=3),
                            session_id='sess', goal_id='goal')
    doc = trim.proposal(toss_cal_version='2026-08-11-abcdef12')
    assert doc['schema'] == toss_trim.TRIM_SCHEMA
    assert 'PROPOSAL ONLY' in doc['promotion']
    with pytest.raises(toss_cal.TossCalError):
        toss_cal.parse_toss_cal(doc)


def test_the_proposal_carries_everything_the_design_asks_for():
    """§ 3.6.3: *(c, se, n, node coverage, guards tripped, tilt_map_version,
    estimator_version)*."""
    trim = toss_trim.replay(_corpus((math.radians(0.05), 0.0), n=90,
                                    sigma_mm=2.0, seed=3, nodes=3))
    doc = trim.proposal(tilt_map_version='synthetic-tilt/1',
                        estimator_version='arrival-offset/1')
    assert set(doc['trim']) >= {'c_rad', 'se_rad', 'n', 'node_coverage',
                                'guards_tripped', 'state', 'reason'}
    assert doc['tilt_map_version'] == 'synthetic-tilt/1'
    # The estimator identity is passed IN by the node (this module may not
    # import the map loader); the RECORD schema is what the trim itself knows.
    assert doc['estimator_version'] == 'arrival-offset/1'
    assert doc['record_schema'] == toss_record.SCHEMA
    assert len(doc['trim']['node_coverage']) == 9


# ── the console line ──────────────────────────────────────────────────────────

def test_the_console_line_says_session_only_and_persistent_explicitly():
    """The tilt-cal review's lesson, applied before it costs four documents: a
    number never written to disk and a number that is must not look the same in
    a log."""
    trim = toss_trim.replay(_corpus((math.radians(0.2), 0.0), n=30,
                                    sigma_mm=2.0, seed=3))
    lines = trim.console_lines(node_xy_mm=(150.0, 0.0),
                               map_aim_rad=(1e-3, -2e-3),
                               flight_time_s=T_REF, catch_z_stow_mm=Z_MM,
                               elapsed_s=142.0, applied=True, cycle_index=9)
    blob = '\n'.join(lines)
    assert 'SESSION-ONLY' in blob
    assert 'PERSISTENT' in blob
    assert 'APPLIED' in blob
    assert 'mm' in blob
    for line in lines:
        assert 'uptime' in line, line


def test_the_console_line_says_not_applied_when_the_param_is_off():
    trim = toss_trim.replay(_corpus((math.radians(0.2), 0.0), n=30,
                                    sigma_mm=2.0, seed=3))
    blob = '\n'.join(trim.console_lines(applied=False))
    assert 'NOT APPLIED' in blob
    assert 'uptime UNMEASURED' in blob


# ── constants: the drift guard ────────────────────────────────────────────────

def test_the_constants_table_matches_the_design_where_it_was_not_amended():
    """The § 3.6 table, in degrees, pinned. The three amendments carry their own
    measurement in the module (``SE_GATE``, ``N_MIN_APPLY``, ``CUSUM_H``); the
    rest must not drift silently."""
    assert math.degrees(toss_trim.DEADBAND_RAD) == pytest.approx(0.10)
    assert math.degrees(toss_trim.STEP_MAX_RAD) == pytest.approx(0.10)
    assert math.degrees(toss_trim.TRIM_MAX_RAD) == pytest.approx(0.15)
    assert toss_trim.N0 == 4
    assert toss_trim.OUTLIER_SIGMA == 4.0
    assert toss_trim.SPEED_AUTHORITY == 0.10
    assert toss_trim.TAU_AUTHORITY_MS == 150.0
    # TRIM_MAX must stay strictly inside the map's own authority: the trim is an
    # unreviewed online estimate and the map is an operator-reviewed capture.
    assert toss_trim.TRIM_MAX_RAD < toss_cal.TOTAL_MAX_RAD
    # And the mm equivalents the design's table quotes, at the reference geometry.
    assert toss_trim.TRIM_MAX_RAD * GAIN == pytest.approx(8.19, abs=0.05)
    assert toss_trim.DEADBAND_RAD * GAIN == pytest.approx(5.46, abs=0.05)


def test_the_reference_gain_is_the_production_one_not_four_h():
    """The design's idealised ``4h`` = 3120 mm/rad; the production model carries
    the ``Δz`` and drop terms exactly and reads 3126.6. 0.21 % — small, but the
    trim's whole mm-vs-rad reporting goes through it, so it is measured."""
    assert GAIN == pytest.approx(3126.64, abs=0.5)
    assert abs(GAIN - 4.0 * APEX_M * 1000.0) / GAIN < 0.005