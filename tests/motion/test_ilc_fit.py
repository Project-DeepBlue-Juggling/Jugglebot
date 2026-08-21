"""Tests for the critical-point ILC fit core — ``tests/hardware/ilc_fit_lib.py``
and its thin CLI ``tests/hardware/ilc_fit.py`` (Phase 1 of
``plans/active/critical-point-ilc.md``).

THE FOUR TESTS THAT MATTER
--------------------------
The plan pre-registers four validations and every one of them is a named test
here, so a future reader can run ``-k v1`` / ``-k v2`` / ``-k v3`` / ``-k v4``
and get the phase's own gate rather than a proxy for it:

* **V1** :func:`test_v1_aim_block_reproduces_4h_theta` — ``F``'s aim/landing
  block against the one block with an independent analytic answer, at BOTH the
  production Jacobian (exact, 1e-9 relative) and the idealised ``4h`` (0.22 %).
* **V2** :func:`test_v2a_closed_loop_cancels_an_injected_command_offset` and
  :func:`test_v2b_the_fit_reduces_a_channel_it_never_saw` — the sign test, in
  the ``toss_fit_lib`` 2c shape: inject a KNOWN perturbation through the
  forward model, fit it back, and pin the sign-flipped control to FAIL BY
  DOUBLING rather than merely to miss.
* **V3** :func:`test_v3_held_out_prediction_on_the_mined_corpus`.
* **V4** :func:`test_v4_repeatability_decision_with_preregistered_null_exit`.

WHY THE SIGN-FLIPPED CONTROLS ARE NOT DECORATION
------------------------------------------------
``toss_fit_lib``'s own docstring makes the argument and it transfers verbatim: a
restated residual drifts in lockstep with a sign flip and would pass on a
correction that makes the machine twice as wrong. Both V2 tests therefore assert
the flipped case explicitly, and V2b does it on a channel the fit never consumed
(``release_speed_err_mms``, mined from the OTHER branch of the same arc by a
separate ballistic fit) so the doubling cannot be an artefact of the arithmetic
being restated.

FIXTURE-BACKED NUMBERS
----------------------
Corpus-backed assertions read the three mined JSONL files under ``temp/probes/``
when they exist and SKIP otherwise: ``temp/`` is gitignored, so a clean checkout
has no corpus and a test that hard-failed there would be a test about the
filesystem. The model-side tests (V1, V2a, every screen and gate test) need no
corpus at all and always run — which is the right split, because those are the
ones that pin production behaviour.
"""

from __future__ import annotations

import glob
import json
import math
import os
import subprocess
import sys

import numpy as np
import pytest

_TESTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_REPO = os.path.dirname(_TESTS)
_HW_DIR = os.path.join(_TESTS, 'hardware')
if _HW_DIR not in sys.path:
    sys.path.insert(0, _HW_DIR)

import ilc_fit_lib as lib                                          # noqa: E402
import ilc_fit as cli                                              # noqa: E402

import jugglebot.hardware_config as hw                             # noqa: E402
from jugglebot import toss_trim                                    # noqa: E402
from jugglebot.motion import toss_cal                              # noqa: E402
from jugglebot.motion.trajectory import ballistics_bc              # noqa: E402
from jugglebot.motion.trajectory import throw_envelope             # noqa: E402
from jugglebot.motion.trajectory.toss_release import (             # noqa: E402
    HAND_THROW_OFFSET_MM,
    compute_release_state,
    compute_release_state_tilted,
    validate_event_vel,
)


# ── fixtures ─────────────────────────────────────────────────────────────────


GOAL = lib.TossGoal(catch_pose_stow_mm=(0.0, 150.0, 170.0),
                    flight_time_s=lib.CORPUS_FLIGHT_TIME_S)


def _corpus_paths():
    """The mined corpora under ``temp/probes``, NEWEST MINE PER BAG.

    ``toss_record_miner`` stamps every run, so a bag re-mined after a miner fix
    leaves both files behind; pooling them would double every row and halve
    every standard error without changing a single mean — a corpus bug that
    looks exactly like better data. Basename shape:
    ``toss_records_<YYYY-MM-DD>_<HH-MM-SS>_<minedate>_<minetime>.jsonl``.

    :func:`ilc_fit_lib.load_corpus` now de-duplicates by ``toss_uid`` as well,
    with the same newest-mine-wins rule, so this helper is no longer the only
    defence — it is kept because selecting the files is cheaper than reading
    and discarding them, and
    :func:`test_the_documented_corpus_glob_yields_one_row_per_toss` pins the two
    routes to the same admitted set.
    """
    per_bag = {}
    for path in sorted(glob.glob(os.path.join(_REPO, 'temp', 'probes',
                                              'toss_records_*.jsonl'))):
        parts = os.path.basename(path)[len('toss_records_'):-len('.jsonl')]
        parts = parts.split('_')
        if len(parts) < 4:
            continue
        bag = '_'.join(parts[:-2])
        mine = '_'.join(parts[-2:])
        if bag not in per_bag or mine > per_bag[bag][0]:
            per_bag[bag] = (mine, path)
    return sorted(v[1] for v in per_bag.values())


def _corpus_or_skip_reason(min_admitted=10):
    """Does THIS tree hold a corpus the fit can actually use?

    Returns ``(paths, rows, None)`` when it does and ``(None, None, reason)``
    when it does not, so every caller asks the SAME question one way.

    :func:`_corpus_paths` answers only "are there corpus files?", which is a
    different question and a dangerous stand-in for this one. ``temp/`` is
    gitignored and per-worktree, so a tree can legitimately hold records mined
    by an OLDER miner: a pre-E-1 mine carries no ``usable_for_release_fit``, so
    every row is refused, the fit REFUSES (correctly), and the CLI exits
    non-zero. Guarding on presence alone turns that into a red suite that says
    nothing about the code — and, in the mirror case, lets a test that asserts
    a refusal pass for entirely the wrong reason.
    """
    paths = _corpus_paths()
    if not paths:
        return None, None, 'no mined corpus under temp/probes (temp/ is gitignored)'
    rows = lib.load_corpus(paths)
    admitted = [r for r in rows if lib.admit_record(r)[0]]
    if len(admitted) < min_admitted:
        return None, None, (
            'mined corpus has only {} admitted rows (of {} loaded) — temp/ is '
            'gitignored and per-worktree, so this tree may hold a pre-E-1 mine'
            .format(len(admitted), len(rows)))
    return paths, rows, None


@pytest.fixture(scope='module')
def corpus():
    _paths, rows, reason = _corpus_or_skip_reason()
    if reason:
        pytest.skip(reason)
    return rows


# ── the forward model IS the production chain (design constraint 1) ──────────


def test_the_model_reproduces_the_commanded_launch_velocity_in_the_bags():
    """The chain being differentiated is the chain that produced the corpus.

    ``cmd_launch_vel_mms`` in every mined row is the announcement's
    ``initial_velocity``, which ``build_announcement_fields`` fills from the very
    ``ReleaseState`` the coordinator built. Rebuilding it here from the goal
    alone and getting the bag's number back to a fraction of a mm/s is what makes
    "``F`` is the sensitivity of the real planner" a measurement rather than a
    claim.

    Recipe confirmed by probe (2026-08-12, ``/tmp/probe_ilc_forward.py``): at
    T = 0.9032314457914598 s and z = 170 mm the production chain returns
    ``vz = 4436.0014`` mm/s against the reference bag's 4436.0014.
    """
    rel = compute_release_state((0.0, 150.0, 170.0), lib.CORPUS_FLIGHT_TIME_S)
    assert rel.launch_vel_mms[0] == pytest.approx(0.0, abs=1e-12)
    assert rel.launch_vel_mms[1] == pytest.approx(0.0, abs=1e-12)
    assert rel.launch_vel_mms[2] == pytest.approx(4436.0014, abs=1e-3)


def test_e_model_is_exactly_zero_at_the_nominal_command():
    """``e_model(0) == 0``: the planner solves the boundary-value problem it is
    being scored on, which is why the residual worth learning is the MEASURED
    one and the model is only ever a direction."""
    assert np.allclose(lib.e_model(lib.zero_command(), GOAL), 0.0, atol=1e-9)


def test_the_aim_branch_is_bitwise_degenerate_at_zero_so_the_difference_is_safe():
    """The 8a and 8b paths agree bitwise at zero tilt, so a CENTRAL difference
    straddling ``aim = 0`` is not straddling a discontinuity.

    ``compute_release_state_tilted``'s documented degenerate identity, restated
    here because :func:`ilc_fit_lib.release_state_for_command` branches on it and
    a future change to that branch would silently corrupt two columns of ``F``.
    """
    level = compute_release_state((0.0, 150.0, 170.0), lib.CORPUS_FLIGHT_TIME_S)
    tilted = compute_release_state_tilted(
        (0.0, 150.0, 170.0), lib.CORPUS_FLIGHT_TIME_S,
        throw_site_xy_mm=(0.0, 150.0))
    assert np.array_equal(level.launch_vel_mms, tilted.launch_vel_mms)
    assert np.array_equal(level.release_pos_global_mm,
                          tilted.release_pos_global_mm)
    assert tilted.tilt_rx == 0.0 and tilted.tilt_ry == 0.0


def test_the_model_direction_channel_uses_the_MINERS_lean_definition():
    """The model's ``arrival_dir`` and the mined ``arrival_dir_err_rad`` are the
    same five lines of code, not two conventions that agree today."""
    import toss_record_miner as miner
    assert lib._miner._lean_rad is miner._lean_rad


def test_the_event_vel_trim_scales_the_production_launch_vector_only():
    """The one non-production operation in the module is a scalar multiply on
    the production launch vector — the DIRECTION still comes from
    ``compute_release_state_tilted``'s tilt geometry."""
    u = np.array([0.004, -0.002, 0.03, 0.0])
    _nom, cmd, launch, event_vel = lib.release_state_for_command(u, GOAL)
    assert launch == pytest.approx(np.asarray(cmd.launch_vel_mms) * 1.03)
    assert event_vel == pytest.approx(cmd.event_vel_mps * 1.03)
    # direction preserved exactly
    a = np.asarray(cmd.launch_vel_mms) / np.linalg.norm(cmd.launch_vel_mms)
    b = launch / np.linalg.norm(launch)
    assert a == pytest.approx(b, abs=1e-15)


# ── V1 ───────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize('h_m', [0.60, 0.78, 1.00])
def test_v1_aim_block_reproduces_4h_theta(h_m):
    """**V1**: ``F``'s aim/landing block against the identity ``b = 4.h.theta``.

    Two comparisons, and the tolerances differ on purpose:

    * against the **production** ``toss_trim.aim_landing_jacobian`` — which
      finite-differences ``aim_target_offset_mm``, the function the apply path
      calls — the agreement must be EXACT (1e-9 relative). These are two
      different routes through the same geometry: this module goes the long way
      round (aim → virtual target → tilted release state → ballistic
      propagation → landing point) and the production Jacobian goes straight to
      the target offset. That they agree to 3e-11 is the statement that the
      virtual-target trick is exact, not approximate;
    * against the **idealised** ``4h`` the tolerance is 1 %, and the measured
      difference is +0.22 % at h = 0.78 — the ``Δz`` and tilted-release-drop
      terms the exact form keeps. Matching ``4h`` to better than that would mean
      the model had DROPPED those terms.

    The 54.578 mm/deg quoted in ``aim_target_offset_mm``'s docstring is the
    SECANT gain at a 1° aim, not the derivative at zero; the derivative is
    54.5718 mm/deg and the 0.011 % gap between them is ``tan(1°)/1°`` plus the
    drop term. Both numbers are right; they are different quantities, and this
    test pins the derivative because that is what a Jacobian is.
    """
    T = math.sqrt(8.0 * h_m * 1000.0 / ballistics_bc.GRAVITY_MMS2)
    goal = lib.TossGoal(catch_pose_stow_mm=(0.0, 0.0, 170.0), flight_time_s=T)
    F = lib.sensitivity(goal=goal)
    block = F[0:2, 0:2]

    production = toss_trim.aim_landing_jacobian(T, 170.0)
    assert block == pytest.approx(production, rel=1e-9, abs=1e-6)

    gain = abs(float(block[0, 1]))
    ideal = 4.0 * h_m * 1000.0
    assert gain == pytest.approx(ideal, rel=0.01)
    assert gain > ideal, ('the exact form keeps Δz and the tilted-release drop, '
                          'so it must exceed 4h — a value at or below 4h means '
                          'those terms were dropped')

    # And the gap is not a tolerance, it is a CLOSED FORM. The exact offset is
    # (Δz + g.T^2/2).tan(theta) with the drop term O(theta^2), and T = sqrt(8h/g)
    # makes g.T^2/2 exactly 4h — so the derivative at zero is
    #
    #     dL/dtheta = 4h + Δz,    Δz = HAND_CATCH_OFFSET_MM - HAND_THROW_OFFSET_MM
    #
    # i.e. the "0.22 % above 4h" quoted across this codebase is exactly Δz/4h
    # (6.736/3120 at h = 0.78, 6.736/4000 at h = 1.00). Pinning the identity
    # rather than the percentage means a future change to either generated offset
    # moves the test and the model together, and a change to only one is caught.
    delta_z = float(hw.HAND_CATCH_OFFSET_MM) - float(HAND_THROW_OFFSET_MM)
    assert gain == pytest.approx(ideal + delta_z, rel=1e-6)

    # A 90 degree rotation, NOT a scaled identity: +rx moves the ball in -y.
    assert block[0, 0] == pytest.approx(0.0, abs=1e-6)
    assert block[1, 1] == pytest.approx(0.0, abs=1e-6)
    assert block[0, 1] > 0.0 and block[1, 0] < 0.0


def test_v1_the_secant_gain_at_one_degree_is_the_docstrings_number():
    """The 54.578 mm/deg in ``aim_target_offset_mm``'s docstring, reproduced —
    so the 0.011 % gap against V1's derivative is documented as arithmetic and
    never mistaken for a discrepancy."""
    from jugglebot.motion.trajectory.toss_release import aim_target_offset_mm
    T = math.sqrt(8.0 * 0.78 * 1000.0 / ballistics_bc.GRAVITY_MMS2)
    secant = float(np.linalg.norm(
        aim_target_offset_mm(0.0, math.radians(1.0), T, 170.0)))
    assert secant == pytest.approx(54.578, abs=0.005)
    derivative = abs(float(lib.sensitivity(
        goal=lib.TossGoal(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                          flight_time_s=T))[0, 1])) * math.pi / 180.0
    assert derivative == pytest.approx(54.5718, abs=0.001)
    assert secant > derivative


# ── the sensitivity structure + the v1 screen ────────────────────────────────


def test_the_release_timing_column_is_structurally_zero():
    """``release_timing_offset``'s column is exactly zero — and this test pins
    STRUCTURE, not a measurement through the production seam.

    δt reaches no production call in ``ilc_fit_lib``: it enters :func:`e_model`
    only as a rigid translation of the release instant, every landing quantity is
    a difference of instants that both carry it, and the four spatial channels
    never reference it at all. So the zero below is algebraic — what it pins is
    that the model has acquired no δt dependence, which is worth having (a
    non-zero entry would mean someone had wired one in without a production
    seam behind it), and what it does NOT pin is the physical claim that the
    platform holds its pose through the flight. That claim is assumed by the
    model, not tested by it; routing δt through
    ``reload_coordinator_node._dispatch_toss_throw`` is deferred.
    """
    F = lib.sensitivity(goal=GOAL)
    assert np.array_equal(F[:, 3], np.zeros(lib.N_E))
    for dt in (-0.05, 0.0, 0.05, 0.2):
        u = np.array([0.0, 0.0, 0.0, dt])
        assert np.allclose(lib.e_model(u, GOAL), 0.0, atol=1e-9)


def test_the_full_size_screen_retains_three_channels_and_names_the_fourth():
    F = lib.sensitivity(goal=GOAL)
    screen = lib.screen_channels(F, mask=np.ones(lib.N_E))
    assert screen['retained_labels'] == ('aim_rx', 'aim_ry', 'event_vel_trim')
    verdict = [v for v in screen['verdicts']
               if v['channel'] == 'release_timing_offset'][0]
    assert verdict['verdict'] == 'EXCLUDED'
    assert verdict['reason'] == 'below_noise_floor'
    assert verdict['snr'] == 0.0
    assert screen['report']['rank'] == 3
    # Well-conditioned over the retained directions — the plan's risk 4
    # (event_vel vs release timing near-degenerate) does NOT materialise: the
    # blocks are orthogonal, so the retained condition number is ~1.
    assert screen['report']['condition_retained'] < 2.0


def test_the_v1_subset_under_E1_is_the_event_vel_trim_alone():
    """The v1 answer, and the reason is E-1 rather than conditioning.

    Both aim channels clear the noise floor comfortably on the unmasked matrix
    (2.9 sigma at one trust region) and are excluded only because every error
    channel that observes them is E-1 blocked. The screen must SAY that — the
    difference between "weak channel" and "blocked channel" is the difference
    between a redesign and a mask change when E-1 closes.
    """
    F = lib.sensitivity(goal=GOAL)
    screen = lib.screen_channels(F, mask=lib.E1_MASK)
    assert screen['retained_labels'] == ('event_vel_trim',)
    for name in ('aim_rx', 'aim_ry'):
        v = [x for x in screen['verdicts'] if x['channel'] == name][0]
        assert v['reason'] == 'e1_blocked'
        assert v['snr_unmasked'] > lib.SCREEN_SNR_MIN


def test_a_deliberately_degenerate_pair_is_excluded_not_regularised():
    """The degeneracy rule fires when it should — driven, not assumed.

    Built by handing the screen an ``F`` whose two aim columns are copies. The
    plan's risk 4 says such a pair is EXCLUDED rather than regularised into
    noise; without a driver that fact would be a comment.
    """
    F = lib.sensitivity(goal=GOAL)
    F[:, 1] = F[:, 0] * 1.001
    screen = lib.screen_channels(F, mask=np.ones(lib.N_E))
    v = [x for x in screen['verdicts'] if x['channel'] == 'aim_ry'][0]
    assert v['verdict'] == 'EXCLUDED'
    assert v['reason'] == 'degenerate'
    assert 'aim_rx' in v['detail']


def test_mask_none_means_the_FULL_SIZE_mask_in_every_family():
    """ONE default mask across the module, and since 2026-08-13 it is FULL SIZE.

    Two facts, and they are separate. **One default**: ``conditioning`` and
    ``screen_channels`` once read ``mask=None`` as ``np.ones(N_E)`` while
    ``weight_matrix``, ``solve_step``, ``required_command``, ``iterate`` and
    ``fit_corpus`` read it as :data:`ilc_fit_lib.E1_MASK` — same keyword, same
    sentinel, opposite meaning, so a caller who omitted the argument got a
    full-size conditioning report ("three channels retained") justifying a step
    solved on one channel. **Full size**: E-1 CLOSED 2026-08-13 (owner-adopted;
    the miner's lateral estimators moved to whole-arc fits), so the shared
    default became :data:`ilc_fit_lib.DEFAULT_MASK` = all five channels. The
    historical masked answer stays reproducible by passing ``E1_MASK``, and this
    test pins both readings against the same ``F``.
    """
    F = lib.sensitivity(goal=GOAL)

    assert np.array_equal(lib.DEFAULT_MASK, np.ones(lib.N_E))
    assert np.array_equal(lib.conditioning(F)['mask'], lib.DEFAULT_MASK)
    assert np.array_equal(lib.screen_channels(F)['report']['mask'],
                          lib.DEFAULT_MASK)
    assert np.array_equal(np.diag(lib.weight_matrix()),
                          lib.DEFAULT_MASK / lib.SIGMA_E ** 2)

    # The verdicts follow the mask, not the family.
    assert lib.screen_channels(F)['retained_labels'] \
        == ('aim_rx', 'aim_ry', 'event_vel_trim')
    assert lib.screen_channels(F, mask=np.ones(lib.N_E))['retained_labels'] \
        == ('aim_rx', 'aim_ry', 'event_vel_trim')
    # ... and the HISTORICAL mask still reproduces the v1 answer exactly.
    assert lib.screen_channels(F, mask=lib.E1_MASK)['retained_labels'] \
        == ('event_vel_trim',)

    # The solve family agrees with the report family under the default: a
    # lateral-only residual now asks for a real aim correction, where under E-1
    # it asked for nothing.
    lateral = np.array([50.0, -50.0, 0.0, 0.0, 0.0])
    need = lib.required_command(F, lateral)
    assert np.linalg.norm(need[:2]) > 1e-3
    assert need[2:] == pytest.approx(np.zeros(2), abs=1e-12)
    assert lib.required_command(F, lateral, mask=lib.E1_MASK) \
        == pytest.approx(np.zeros(lib.N_U), abs=1e-12)
    assert lib.solve_step(F, lateral, mask=lib.E1_MASK) \
        == pytest.approx(np.zeros(lib.N_U), abs=1e-12)
    # The blocked-vs-weak distinction is retained, and is now reachable only by
    # asking for the historical mask by name.
    blocked = [v for v in lib.screen_channels(F, mask=lib.E1_MASK)['verdicts']
               if v['channel'] == 'aim_rx'][0]
    assert blocked['reason'] == 'e1_blocked'


def test_the_conditioning_scaling_makes_the_threshold_dimensionless():
    """A scaled singular value IS "sigmas of task error per full trust region",
    so :data:`SCREEN_SNR_MIN` = 1.0 is a definition and not a tuned number."""
    F = lib.sensitivity(goal=GOAL)
    rep = lib.conditioning(F)
    expected = (abs(F[4, 2]) * lib.TAU0[2]) / lib.SIGMA_E[4]
    assert rep['column_norms'][2] == pytest.approx(expected, rel=1e-12)


# ── the update law + the exact-gate re-validation (design constraint 3) ──────


def test_the_step_is_clipped_to_the_box_trust_region():
    F = lib.sensitivity(goal=GOAL)
    huge = np.array([0.0, 0.0, 0.0, 0.0, 10.0])       # 10 s of flight-time error
    du = lib.solve_step(F, huge, tau=lib.TAU0)
    assert du[2] == pytest.approx(-lib.TAU0[2])
    assert np.all(np.abs(du) <= lib.TAU0 + 1e-15)


def test_the_damping_is_dimensionless_across_channels():
    """``R = rho/tau^2`` means one ``rho`` for four channels; building ``R`` in
    raw units would make the same ``rho`` mean a different thing per channel —
    which is how a damping constant silently becomes six."""
    F = lib.sensitivity(goal=GOAL)
    e = np.zeros(lib.N_E)
    e[4] = 0.001                                   # small: damping regime
    du = lib.solve_step(F, e, tau=lib.TAU0, rho=lib.RHO_DAMPING)
    s = (abs(F[4, 2]) * lib.TAU0[2]) / lib.SIGMA_E[4]
    predicted = -(e[4] / lib.SIGMA_E[4]) * s / (s * s + lib.RHO_DAMPING) \
        * lib.TAU0[2]
    assert du[2] == pytest.approx(predicted, rel=1e-9)


def test_a_step_that_would_be_clamped_by_the_D7_gate_is_refused_and_shrunk():
    """**The exact-gate loop, driven.** A current aim near the 1° authority plus
    a residual demanding more must NOT be truncated by
    ``toss_cal.clamp_total_aim`` — the plan's risk 5 (a truncated step
    desynchronises applied-u from recorded-u). The loop shrinks tau instead and
    the accepted step is strictly inside the authority.
    """
    # Each AXIS is well inside the bound; the MAGNITUDE is at 99 % of it. That
    # is the case a per-axis check would wave through and the joint D7 clamp
    # catches — which is why the aim pair is bounded jointly and only there.
    edge = toss_cal.TOTAL_MAX_RAD * 0.7
    u_now = np.array([edge, edge, 0.0, 0.0])
    assert math.hypot(edge, edge) < toss_cal.TOTAL_MAX_RAD
    assert lib.admit_command(u_now, GOAL)[0]
    F = lib.sensitivity(u_now, GOAL)
    e = np.array([-200.0, 200.0, 0.0, 0.0, 0.0])     # demands more of both
    step = lib.propose_step(F, e, GOAL, u_current=u_now, mask=np.ones(lib.N_E))
    assert step['shrinks'] >= 1, 'the D7 clamp should have refused the first step'
    assert any('TRUNCATE' in r for r in step['refusals'])
    assert math.hypot(*step['u_next'][:2]) <= toss_cal.TOTAL_MAX_RAD


def test_when_no_admissible_step_exists_the_loop_refuses_loudly():
    """At the authority bound with a residual pushing further out, EVERY trust
    region refuses — and the answer is a named refusal, never a silent zero."""
    u_now = np.array([0.0, toss_cal.TOTAL_MAX_RAD, 0.0, 0.0])
    F = lib.sensitivity(u_now, GOAL)
    e = np.array([-5000.0, 0.0, 0.0, 0.0, 0.0])
    with pytest.raises(lib.IlcFitError) as exc:
        lib.propose_step(F, e, GOAL, u_current=u_now, mask=np.ones(lib.N_E))
    assert 'NO ADMISSIBLE STEP' in str(exc.value)
    assert 'TRUNCATE' in str(exc.value) or 'authority' in str(exc.value)


def test_the_aim_authority_is_a_magnitude_bound_not_a_per_axis_one():
    """A per-axis aim bound would admit ``sqrt(2)x`` the authority on the
    diagonal. ``clamp_total_aim`` bounds the magnitude, and it is the ONLY place
    the aim pair is bounded — so the diagonal case must refuse."""
    diag = toss_cal.TOTAL_MAX_RAD * 0.99
    ok, why = lib.admit_command(np.array([diag, diag, 0.0, 0.0]), GOAL)
    assert not ok
    assert 'TRUNCATE' in why


def test_the_authority_report_bounds_the_aim_pair_jointly_like_admit_command():
    """The report and the gate must not disagree about the same command.

    A diagonal aim requirement of 1.27x the clamp is 0.90x on EACH AXIS, so a
    per-axis report says ``exceeds=False`` — "ok, inside authority" — for a
    correction :func:`ilc_fit_lib.admit_command` refuses outright. That gap is
    the whole failure: the authority report is the artefact Gate 1 reads to
    decide whether a correction can be commanded at all, so a false "ok" there
    is a decision made on a command the machine will never fly.

    Driven rather than constructed: the residual is built BY the model from a
    known required command (``e = -F.du``), so ``required_command`` returns that
    command back and the report is being asked about a correction the fit could
    genuinely have produced.
    """
    F = lib.sensitivity(goal=GOAL)
    over = 1.27
    per_axis = toss_cal.TOTAL_MAX_RAD * over / math.sqrt(2.0)
    want = np.array([per_axis, per_axis, 0.0, 0.0])
    e = -(F @ want)

    rep = lib.authority_report(F, e, mask=np.ones(lib.N_E))
    assert rep['required'][:2] == pytest.approx(want[:2], rel=1e-9)

    for row in rep['channels'][:2]:
        assert row['fraction'] < 1.0, (
            'per-axis this aim looks fine — that is exactly why the per-axis '
            'test was wrong')
        assert row['joint_required'] == pytest.approx(
            toss_cal.TOTAL_MAX_RAD * over, rel=1e-9)
        assert row['joint_fraction'] == pytest.approx(over, rel=1e-9)
        assert row['exceeds'], 'the joint magnitude is 1.27x the clamp'
    assert rep['any_exceeds']
    # The scalar channels keep their per-axis verdict and gain no joint field.
    for row in rep['channels'][2:]:
        assert row['joint_required'] is None

    # ... and the gate agrees, which is the property the report exists to mirror.
    ok, why = lib.admit_command(rep['required'], GOAL)
    assert not ok and 'TRUNCATE' in why


def test_iterate_reports_aim_saturation_on_the_magnitude_not_the_axis():
    """``iterate``'s saturation flag uses the same joint test — a run that ends
    against the D7 clamp on the diagonal has run out of authority, and calling
    that "unsaturated" because neither axis is at the bound would report a
    converged fit where there is an exhausted one."""
    # Exactly ON the joint bound (the largest aim the D7 clamp still admits), so
    # each AXIS is at 0.707x the authority and a per-axis test sees no
    # saturation at all.
    axis = toss_cal.TOTAL_MAX_RAD * (1.0 - 1e-12) / math.sqrt(2.0)
    u = np.array([axis, axis, 0.0, 0.0])
    assert max(abs(u[0]), abs(u[1])) < 0.8 * toss_cal.TOTAL_MAX_RAD
    assert lib.admit_command(u, GOAL)[0], 'must still be an admissible command'
    steps = lib.iterate(GOAL, np.zeros(lib.N_E), n_iter=1, u0=u)
    assert steps[0]['du'] == pytest.approx(np.zeros(lib.N_U), abs=1e-15)
    assert 'aim_rx' in steps[0]['saturated']
    assert 'aim_ry' in steps[0]['saturated']
    assert 'event_vel_trim' not in steps[0]['saturated']


def test_the_scalar_speed_authority_is_inadmissible_near_both_band_ends():
    """**C2, and this test is the one that failed as written.**

    Its predecessor —
    ``test_the_event_vel_band_is_unreachable_inside_the_speed_authority`` — pinned
    the Gate-1 safety argument: *"over the whole sequencer flight-time band and
    the whole ±0.15 authority, ``event_vel`` stays inside the bridge's
    [0.3, 7.0] m/s band, so the band gate can never be what binds an ILC step."*
    Both halves of that premise have since moved and the conclusion went with
    them:

    * the flight-time band is no longer the hand-picked ``[0.55, 1.10] s``; it is
      DERIVED, ``[0.4949, 1.1485] s`` (contract C-HAND-3, ``throw_envelope``); and
    * the wire band **bounds nothing physical**. The gate that does is
      ``throw_envelope.evaluate``, and it refuses long before [0.3, 7.0] does.

    So the reachability statement is now *true and irrelevant* — the rails stay
    inside the wire band and are refused anyway. What this test pins instead is
    the fact that replaced it: at BOTH ends of the derived band a ±0.15 trim is
    **inadmissible**, and :func:`ilc_fit_lib.admit_command` says so by name.
    Measured 2026-08-21 with ``tools/probes/ilc_speed_band.py`` and re-derived
    here rather than restated.
    """
    lo_T, hi_T = throw_envelope.MIN_FLIGHT_TIME_S, throw_envelope.MAX_FLIGHT_TIME_S

    # LONG-flight end: +0.15 is refused, and the bound that closes it is the
    # decel feedforward's current headroom, not the wire band.
    goal_hi = lib.TossGoal(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                           flight_time_s=1.10)
    u_fast = np.array([0.0, 0.0, +lib.ILC_SPEED_AUTHORITY, 0.0])
    _n, _c, _l, ev_fast = lib.release_state_for_command(u_fast, goal_hi)
    assert validate_event_vel(ev_fast), (
        'the rail is still inside the wire band — that is exactly why the wire '
        'band is not the gate')
    ok, why = lib.admit_command(u_fast, goal_hi)
    assert not ok
    assert 'ADMISSIBLE band' in why and 'DECEL' in why

    # SHORT-flight end: the NEGATIVE rail is refused, bounded by ARM_WINDOW —
    # the counter-intuitive direction (a slow-down trim narrows the catch-arm
    # window because throw_decel_s grows as the release speed falls).
    goal_lo = lib.TossGoal(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                           flight_time_s=lo_T)
    u_slow = np.array([0.0, 0.0, -lib.ILC_SPEED_AUTHORITY, 0.0])
    _n, _c, _l, ev_slow = lib.release_state_for_command(u_slow, goal_lo)
    assert validate_event_vel(ev_slow)
    ok, why = lib.admit_command(u_slow, goal_lo)
    assert not ok
    assert 'ADMISSIBLE band' in why and 'ARM_WINDOW' in why

    # And the corpus's own measured demand, -0.1076, is inadmissible at exactly
    # the R5-prime cadence target. This is fold-in consequence (b): the channel
    # has ZERO authority in the only direction the plant asks for, at the flight
    # time the cadence ladder is aiming at.
    ok, why = lib.admit_command(np.array([0.0, 0.0, -0.1076, 0.0]), goal_lo)
    assert not ok and 'ARM_WINDOW' in why

    # The gates themselves are real, driven directly rather than inferred.
    assert not validate_event_vel(hw.TEENSY_TRAJ_MIN_EVENT_VEL_MPS - 0.01)
    assert not validate_event_vel(hw.TEENSY_TRAJ_MAX_EVENT_VEL_MPS + 0.01)
    assert not throw_envelope.evaluate(hi_T * 1.01,
                                       ev_fast).ok


def test_the_speed_authority_band_is_derived_per_flight_time():
    """The T-sweep the fold-in's build step 1 asks for, pinned by value.

    Ground truth is ``tools/probes/ilc_speed_band.py`` (run 2026-08-21, scan step
    0.001); this function bisects instead of scanning, so the two agree to the
    probe's own resolution and the exact edges land where the scan could only
    say "+0.000".

    The shape is what matters and it is not symmetric: the NEGATIVE side closes
    at the SHORT-flight end (``ARM_WINDOW``) and the POSITIVE side at the
    LONG-flight end (``DECEL_FF_HEADROOM``), so there is no single scalar that is
    admissible everywhere — which is C2 in one sentence.
    """
    v = throw_envelope.vertical_release_speed_mps
    lo_T, hi_T = throw_envelope.MIN_FLIGHT_TIME_S, throw_envelope.MAX_FLIGHT_TIME_S

    # At the two derived edges the binding side is EXACTLY zero: the band edge is
    # the flight time at which that bound reaches equality, so by construction no
    # trim in that direction survives.
    assert lib.speed_authority_band(lo_T, v(lo_T))[0] == pytest.approx(0.0, abs=1e-9)
    assert lib.speed_authority_band(hi_T, v(hi_T))[1] == pytest.approx(0.0, abs=1e-9)

    # Interior points, against the probe's published numbers.
    for T, expect_hi in ((1.00, 0.148), (1.10, 0.043)):
        assert lib.speed_authority_band(T, v(T))[1] == pytest.approx(
            expect_hi, abs=0.001)
    # ... and where the envelope is wide, the ILC ceiling is what binds — 0.15 is
    # kept precisely so an envelope that opened up cannot silently widen a
    # learned trim past the number the owner approved.
    assert lib.speed_authority_band(lib.CORPUS_FLIGHT_TIME_S,
                                    v(lib.CORPUS_FLIGHT_TIME_S)) == (
        -lib.ILC_SPEED_AUTHORITY, +lib.ILC_SPEED_AUTHORITY)

    # Monotone in the right direction, and BOTH edges move the same way: as the
    # flight time grows the positive edge shrinks toward zero (the throw gets
    # faster, so the decel headroom runs out) and the negative edge opens up away
    # from zero (the catch-arm window widens, so a slow-down stops breaking it).
    # Both sequences are therefore non-increasing.
    bands = [lib.speed_authority_band(T, v(T))
             for T in (lo_T, 0.55, 0.7, 0.9032, 1.0, 1.1, hi_T)]
    assert [b[1] for b in bands] == sorted([b[1] for b in bands], reverse=True)
    assert [b[0] for b in bands] == sorted([b[0] for b in bands], reverse=True)

    # Fail-closed: a goal the envelope already refuses UNTRIMMED gets no band at
    # all, rather than a band centred on an inadmissible command.
    assert lib.speed_authority_band(hi_T * 1.05, v(hi_T * 1.05)) == (0.0, 0.0)
    assert lib.speed_authority_band(float('nan'), 4.0) == (0.0, 0.0)


def test_the_throw_envelope_is_wired_into_admit_command():
    """``throw_envelope.evaluate`` is CALLED by
    :func:`ilc_fit_lib.admit_command` — pinned by substitution, in the shape of
    the ``validate_event_vel`` test below it.

    Substitution rather than a real refusal because the band check in step 2b is
    derived from the same function and therefore fires first for any trim big
    enough to break the envelope; the only honest way to show step 6b is wired is
    to make the envelope refuse a command the band admits, and watch the FSM's
    own reject code come out.
    """
    goal = lib.TossGoal(catch_pose_stow_mm=(0.0, 0.0, 170.0), flight_time_s=0.9)
    real = lib.throw_envelope.evaluate
    try:
        lib.throw_envelope.evaluate = lambda t, v: throw_envelope.ThrowEnvelopeVerdict(
            False, 'END_STOP', 'substituted')
        # The band is memoised, and a band computed through the substitute would
        # outlive it — clear on the way IN and on the way OUT.
        lib.speed_authority_band.cache_clear()
        ok, why = lib.admit_command(lib.zero_command(), goal)
    finally:
        lib.throw_envelope.evaluate = real
        lib.speed_authority_band.cache_clear()
    assert not ok
    assert 'REJECTED_THROW_ENVELOPE' in why and 'END_STOP' in why
    assert lib.admit_command(lib.zero_command(), goal)[0], (
        'the substitution leaked — the real gate must admit the nominal command')


def test_the_event_vel_gate_is_wired_into_admit_command():
    """The band gate is CALLED by :func:`ilc_fit_lib.admit_command` — pinned by
    substitution, because no in-authority command can reach it (the test above).

    Without this, "admit_command runs validate_event_vel" would be a comment: the
    real gate is unreachable through the command vector, so the only honest way
    to show it is wired is to make it refuse and watch the refusal come out with
    the FSM's own code on it.
    """
    goal = lib.TossGoal(catch_pose_stow_mm=(0.0, 0.0, 170.0), flight_time_s=0.9)
    real = lib.validate_event_vel
    try:
        lib.validate_event_vel = lambda ev: False
        ok, why = lib.admit_command(lib.zero_command(), goal)
    finally:
        lib.validate_event_vel = real
    assert not ok
    assert 'REJECTED_EVENT_VEL' in why
    assert lib.admit_command(lib.zero_command(), goal)[0], (
        'the substitution leaked — the real gate must admit the nominal command')


def test_the_speed_authority_is_the_ILCs_own_and_toss_trims_is_untouched():
    """Gate 1's decision, pinned from both sides.

    The operator widened the ILC's accumulated speed trim to ±0.15 on
    2026-08-13 (``plans/active/critical-point-ilc.md``, "Gate 1 CLOSED") because
    the measured corpus asks for −0.1076 and two of three goal cells exceed
    ±0.10 on their own. The decision was ILC-SPECIFIC, and the thing that makes
    it safe rather than convenient is that it did NOT touch
    ``toss_trim.SPEED_AUTHORITY`` — a different loop, a different update law, a
    different measurand. Both halves are asserted here, because a future edit
    that "unifies" the two constants would look like tidying and would silently
    widen the session trim.
    """
    assert lib.ILC_SPEED_AUTHORITY == 0.15
    assert toss_trim.SPEED_AUTHORITY == 0.10
    assert lib.AUTHORITY[2] == lib.ILC_SPEED_AUTHORITY

    # Inside the ILC bound: ADMITTED, including the value the corpus asks for.
    for dv in (-0.1076, -lib.ILC_SPEED_AUTHORITY, +lib.ILC_SPEED_AUTHORITY):
        ok, why = lib.admit_command(np.array([0.0, 0.0, dv, 0.0]), GOAL)
        assert ok, why
    # Past it: REFUSED, with the bound's owner named — this module still does
    # not widen a bound to make its own answer fit.
    ok, why = lib.admit_command(
        np.array([0.0, 0.0, lib.ILC_SPEED_AUTHORITY * 1.01, 0.0]), GOAL)
    assert not ok
    assert 'ILC_SPEED_AUTHORITY' in why
    assert 'toss_trim.SPEED_AUTHORITY' in why      # named as NOT changed


def test_an_inadmissible_goal_is_refused_before_any_fitting():
    out_of_band = lib.TossGoal(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                               flight_time_s=2.0)
    ok, why = lib.admit_command(lib.zero_command(), out_of_band)
    assert not ok and 'FLIGHT_TIME' in why
    off_pose = lib.TossGoal(catch_pose_stow_mm=(400.0, 0.0, 170.0),
                            flight_time_s=0.9)
    ok, why = lib.admit_command(lib.zero_command(), off_pose)
    assert not ok and 'WORKSPACE' in why


def test_catch_side_channels_are_declared_and_refuse():
    for name, _unit, _seam in lib.CATCH_CHANNELS:
        with pytest.raises(NotImplementedError) as exc:
            lib.catch_channel(name)
        assert 'G-2' in str(exc.value) and '0b' in str(exc.value)
    with pytest.raises(lib.IlcFitError):
        lib.catch_channel('not_a_channel')


# ── V2 ───────────────────────────────────────────────────────────────────────


def test_v2a_closed_loop_cancels_an_injected_command_offset():
    """**V2a**: inject a KNOWN ``u_plant`` through the forward model, mine the
    synthetic error, fit — the correction must come back as ``-u_plant``.

    Run on all three retained channels at once (mask off, so the aim channels
    participate) because a per-channel test cannot catch a cross-channel sign
    swap, which is the failure mode a 90°-rotation Jacobian invites.
    """
    u_plant = np.array([0.004, -0.003, 0.02, 0.0])
    rows = lib.synthetic_corpus(GOAL, u_plant=u_plant, n=400, seed=7)
    F = lib.sensitivity(goal=GOAL)
    du = lib.required_command(F, lib.pooled_error(rows)[0], mask=np.ones(lib.N_E))
    assert du[0] == pytest.approx(-u_plant[0], rel=0.10)
    assert du[1] == pytest.approx(-u_plant[1], rel=0.10)
    assert du[2] == pytest.approx(-u_plant[2], rel=0.10)
    assert du[3] == 0.0


def test_v2a_a_sign_flipped_F_is_pinned_to_FAIL():
    """The control. A flipped ``F`` does not merely miss — it returns the same
    magnitude the wrong way, DOUBLING the residual, which is the consequence a
    restated-residual assertion would sail straight past."""
    u_plant = np.array([0.0, 0.0, 0.02, 0.0])
    rows = lib.synthetic_corpus(GOAL, u_plant=u_plant, n=400, seed=11)
    e = lib.pooled_error(rows)[0]
    F = lib.sensitivity(goal=GOAL)
    du_ok = lib.required_command(F, e)
    du_bad = lib.required_command(-F, e)
    assert du_ok[2] < 0.0 and du_bad[2] > 0.0
    assert du_bad[2] == pytest.approx(-du_ok[2], rel=1e-12)
    # Residual after each, through the model.
    after_ok = abs(e[4] + float(F[4] @ du_ok))
    after_bad = abs(e[4] + float(F[4] @ du_bad))
    assert after_ok < 0.02 * abs(e[4])
    assert after_bad > 1.9 * abs(e[4])


def test_v2b_the_fit_reduces_a_channel_it_never_saw(corpus):
    """**V2b**, the real-corpus half: fit ``event_vel_trim`` on
    ``flight_time_err_s`` and replay it against ``release_speed_err_mms``.

    The two channels are mined from DIFFERENT branches of the arc by different
    ballistic fits over disjoint mocap samples, and their per-toss scatter
    correlates at only r = 0.23 — so this is genuinely out-of-channel evidence,
    not the same number twice. Measured 2026-08-12: predicted -477.3 mm/s
    against a measured +455.4 mm/s mean, rms 459.0 -> 60.8 (86.8 % reduction);
    the sign-flipped control goes 459.0 -> 934.5 (x2.04).
    """
    fit = lib.fit_corpus(corpus, pool_across_goals=True,
                         allow_cross_partition=True)
    need = lib.required_command(fit['F'], fit['e_meas'])
    assert need[2] < 0.0, ('the plant throws ~11 % FAST, so the trim must be '
                           'NEGATIVE')
    cc = lib.cross_check_replay(need, fit['goal'], corpus)
    assert cc['reduction'] > 0.75
    flip = lib.cross_check_replay(-need, fit['goal'], corpus)
    assert flip['rms_after_mms'] > 1.9 * flip['rms_before_mms']


def test_v2b_the_two_vertical_channels_agree_on_the_bias_through_production(corpus):
    """The model says ``dT/dv = 2/g`` exactly; the corpus agrees to 5 %.

    Two independent measurements of one physical fact — the hand throws fast —
    reached through different branches, different fits and different production
    functions. It is the strongest single piece of evidence that the vertical
    channel is measuring the plant and not the estimator, and it is why V2b's
    cross-check is worth running at all.
    """
    rows = [r for r in corpus if lib.admit_record(r)[0]]
    ft = np.array([r['flight_time_err_s'] for r in rows])
    rs = np.array([r['release_speed_err_mms'] for r in rows])
    predicted = 2.0 / ballistics_bc.GRAVITY_MMS2 * rs.mean()
    assert predicted == pytest.approx(ft.mean(), rel=0.06)
    # ... and yet the per-toss scatter is largely independent: most of the
    # spread in each channel is that channel's own measurement noise.
    assert abs(float(np.corrcoef(ft, rs)[0, 1])) < 0.6


# ── V3 ───────────────────────────────────────────────────────────────────────


def test_v3_held_out_prediction_on_the_mined_corpus(corpus):
    """**V3**: train on a subset, predict the held-out rows.

    Measured 2026-08-12 over the **17** admitted rows that carry a recoverable
    goal (the two whose cup is unrecoverable are excluded — the model prediction
    needs a geometry to evaluate the sensitivity at): flight-time LOO rms
    0.0975 -> 0.0147 s (84.9 %), release-speed 454.9 -> 62.0 mm/s (86.4 %). The
    deterministic 12/5 split predicts the held-out mean at +0.0970 s against an
    actual +0.0954 s, a miss of 0.19 standard errors.

    **The honest reading, and it belongs in the assertion's neighbourhood rather
    than only in a report**: n = 17 across three goal cells at ONE commanded
    flight time. The reduction is large because the residual is a near-constant
    bias, and this corpus can say nothing whatever about generalisation to
    another flight time — a single operating point cannot separate a launch-speed
    GAIN from an OFFSET. That is why the artifact key carries the flight-time
    cell (:data:`ilc_fit_lib.FLIGHT_TIME_CELL_S`).
    """
    for channel, floor in (('flight_time_err_s', 0.7),
                           ('release_speed_err_mms', 0.7)):
        out = lib.held_out_prediction(corpus, channel=channel)
        assert out['loo']['reduction'] > floor
        assert out['split']['reduction'] > floor
        assert out['loo']['rms_after'] < out['loo']['rms_before']
        # PREDICTED vs ACTUAL: the training set predicts the held-out set's
        # mean residual; the held-out set measures it. Agreement inside ~2
        # standard errors of the held-out mean is the claim, and it is a
        # genuine prediction rather than a restatement — the two sets share no
        # rows.
        split = out['split']
        assert abs(split['z']) < 2.0, (
            'predicted {} vs actual {} is {:.1f} se apart'.format(
                split['predicted_test_mean'], split['actual_test_mean'],
                split['z']))


def test_v3_a_pure_noise_corpus_shows_no_held_out_improvement():
    """The control for V3: with no repeatable structure, leave-one-out
    prediction must NOT improve anything — it makes things slightly worse
    (the ``1 + 1/(n-1)`` inflation). Without this, "85 % reduction" is just a
    number with no scale."""
    goal = GOAL
    rows = lib.synthetic_corpus(goal, u_plant=lib.zero_command(), n=40, seed=3)
    out = lib.held_out_prediction(rows, channel='flight_time_err_s')
    assert out['loo']['reduction'] < 0.05


# ── V4 ───────────────────────────────────────────────────────────────────────


def test_v4_repeatability_decision_with_preregistered_null_exit(corpus):
    """**V4**: the pre-registered repeatability statistic and its NULL-exit.

    Statistic: ``R_rep = 1 - MSE_loo/MSE_raw``, where ``MSE_loo`` uses each
    row's own GOAL CELL mean computed without that row. Leave-one-out and not
    in-sample, because an in-sample per-cell mean at n = 3 removes a third of
    the variance by construction.

    Threshold 0.5, derived at :data:`ilc_fit_lib.REPEATABILITY_MIN`; the null
    expectation is NEGATIVE (about -1/(n-1)), so any positive value is already
    evidence.

    Measured 2026-08-12: R_rep = **0.9858** (flight time, rms 0.0975 -> 0.0116 s)
    and **0.9790** (release speed, rms 454.9 -> 65.9 mm/s) over 3 goal cells /
    17 keyed rows. PASS, by a factor of two on the threshold and by an order on
    the null.
    """
    for channel in ('flight_time_err_s', 'release_speed_err_mms'):
        rep = lib.repeatability(corpus, channel=channel)
        assert rep['verdict'] == 'PASS'
        assert rep['R_rep'] > lib.REPEATABILITY_MIN
        assert rep['R_rep'] > 0.9
        assert rep['n_goal_cells'] >= 3
        assert rep['null_expectation'] < 0.0
        assert rep['rms_loo'] < 0.25 * rep['rms_raw']


def test_v4_null_exits_on_a_corpus_with_no_repeatable_structure():
    """The NULL-exit is REACHABLE — pinned by driving it.

    A pre-registered exit nobody has ever seen fire is a comment. Here the
    plant offset is zero and only noise remains, so the per-goal mean carries
    nothing and the statistic must return NULL_EXIT.
    """
    rows = lib.synthetic_corpus(GOAL, u_plant=lib.zero_command(), n=30, seed=5)
    rep = lib.repeatability(rows, channel='flight_time_err_s')
    assert rep['verdict'] == 'NULL_EXIT'
    assert rep['R_rep'] < lib.REPEATABILITY_MIN


def test_the_repeatability_null_is_the_per_cell_LOO_null_not_the_pooled_one():
    """``null_expectation`` must be the null of the statistic actually computed.

    ``R_rep`` is a PER-CELL leave-one-out, so under zero-mean noise
    ``E[r_i^2] = sigma^2.n_k/(n_k-1)`` inside a cell of ``n_k`` rows and the null
    is the row-weighted ``-sum_k n_k/((n_k-1).N)``. The pooled ``-1/(N-1)`` is
    the null of a different estimator (one global mean) and understates this one
    whenever the cells are small: on the 2026-08-12 corpus's 6/8/3 cells it reads
    -0.0625 against a true -0.226, a factor of 3.6 exactly where the reader is
    asked to judge a small positive R_rep against "the null is barely below
    zero".

    Driven on a synthetic corpus built with the REAL corpus's cell shape (6, 8
    and 3 rows at three poses), because a single cell cannot discriminate: at one
    cell of n rows both formulas give -1/(n-1) and a wrong implementation passes.
    """
    poses = (((-150.0, -150.0, 170.0), 6),
             ((0.0, 150.0, 170.0), 8),
             ((150.0, -150.0, 170.0), 3))
    rows = []
    for i, (pose, n) in enumerate(poses):
        goal = lib.TossGoal(catch_pose_stow_mm=pose,
                            flight_time_s=lib.CORPUS_FLIGHT_TIME_S)
        rows.extend(lib.synthetic_corpus(goal, u_plant=lib.zero_command(),
                                         n=n, seed=101 + i))
    rep = lib.repeatability(rows, channel='flight_time_err_s')
    assert rep['n'] == 17 and rep['n_goal_cells'] == 3

    expected = -sum(n / (n - 1.0) for _pose, n in poses) / 17.0
    assert expected == pytest.approx(-0.22605, abs=1e-5)
    assert rep['null_expectation'] == pytest.approx(expected, rel=1e-12)
    # the pooled form is what this replaced, and it is 3.6x too shallow
    assert rep['null_expectation'] < -1.0 / (17.0 - 1.0)

    # A single cell is the case that does NOT discriminate — both forms give
    # -1/(n-1) — which is why the shape above is the one that had to be built.
    one = lib.repeatability(
        lib.synthetic_corpus(GOAL, u_plant=lib.zero_command(), n=30, seed=5),
        channel='flight_time_err_s')
    assert one['n_goal_cells'] == 1
    assert one['null_expectation'] == pytest.approx(-1.0 / 29.0, rel=1e-12)
    # ... and a pure-noise corpus really does sit near its own null.
    assert one['R_rep'] < 0.0


def test_v4_unkeyed_rows_do_not_vouch_for_each_other(corpus):
    """Rows whose goal cannot be recovered are EXCLUDED from the per-goal
    statistic rather than grouped under a shared ``None`` cell — otherwise two
    tosses at two different poses would vouch for each other's repeatability,
    which is the exact claim the statistic makes."""
    rep = lib.repeatability(corpus, channel='flight_time_err_s')
    assert rep['n_unkeyed_rows'] >= 1
    assert all(pg['goal'] is not None for pg in rep['per_goal'])


# ── the corpus side: partitions, keys, admission ─────────────────────────────


def test_the_partition_rule_refuses_to_pool_two_plants(corpus):
    """The 2026-08-12 corpus really does span two bridge firmwares (10 and 12),
    16 rows and 3, so the refusal is exercised by the real data rather than by a
    fixture built to trip it."""
    admitted = [r for r in corpus if lib.admit_record(r)[0]]
    census = lib.partition_census(admitted)
    assert len(census) >= 2
    with pytest.raises(lib.IlcFitError) as exc:
        lib.select_partition(admitted)
    assert 'refusing to fit across' in str(exc.value)
    rows, _census, _key, warnings = lib.select_partition(admitted,
                                                         allow_cross=True)
    assert len(rows) == len(admitted)
    assert any('CROSSING' in w for w in warnings)


def test_load_corpus_de_duplicates_re_mines_and_keeps_the_NEWER_one(tmp_path):
    """A re-mined bag leaves BOTH files in ``temp/probes`` and the documented
    glob matches both. Pooling them doubles every row and halves every standard
    error **without moving a single mean** — a corpus bug that looks exactly like
    better data, which is why it belongs in the library and not in each caller.

    The tie-break is the MINE STAMP in the filename, newest wins, and that is
    load-bearing rather than tidy: on the real corpus the older mine of the
    2026-08-10 bag predates the miner fix that sets ``usable_for_release_fit``,
    so a first-seen-wins rule on a sorted glob would keep 53 rows of which zero
    are admissible.
    """
    def _write(name, uids, usable):
        path = tmp_path / name
        with open(str(path), 'w') as fh:
            for uid in uids:
                fh.write(json.dumps({
                    'schema': 'toss_record/1', 'toss_uid': uid,
                    'usable_for_release_fit': usable,
                    'flight_time_err_s': 0.05}) + '\n')
        return str(path)

    old = _write('toss_records_2026-08-10_16-30-44_20260812_210140.jsonl',
                 ['a', 'b', 'c'], False)
    new = _write('toss_records_2026-08-10_16-30-44_20260812_220718.jsonl',
                 ['a', 'b', 'c'], True)

    for order in ([old, new], [new, old]):
        rows = lib.load_corpus(order)
        assert len(rows) == 3, 'the re-mine must not double the corpus'
        assert {r['toss_uid'] for r in rows} == {'a', 'b', 'c'}
        assert all(r['_source_file'] == new for r in rows), (
            'the NEWER mine must win in either path order — first-seen would '
            'keep the pre-fix rows and admit none of them')
        assert len([r for r in rows if lib.admit_record(r)[0]]) == 3

    # A file whose name carries no mine stamp never supersedes one that does.
    odd = _write('toss_records_oddball.jsonl', ['a'], False)
    rows = lib.load_corpus([new, odd])
    assert [r['_source_file'] for r in rows] == [new, new, new]


def test_the_documented_corpus_glob_yields_one_row_per_toss(corpus):
    """The invocation in ``ilc_fit.py``'s own docstring —
    ``--corpus temp/probes/toss_records_*.jsonl`` — matches every re-mine, so
    the de-duplication above is what keeps that documented command honest."""
    paths = sorted(glob.glob(os.path.join(_REPO, 'temp', 'probes',
                                          'toss_records_*.jsonl')))
    if len(paths) <= len(_corpus_paths()):
        pytest.skip('no re-mined duplicates in this checkout')
    rows = lib.load_corpus(paths)
    uids = [r['toss_uid'] for r in rows]
    assert len(uids) == len(set(uids)), 'the glob must not double any toss'
    admitted = [r for r in rows if lib.admit_record(r)[0]]
    assert len(admitted) == len([r for r in corpus if lib.admit_record(r)[0]]), (
        'the full glob and the newest-per-bag helper must admit the same rows')
    assert len({r['toss_uid'] for r in admitted}) == len(admitted)


def test_the_goal_key_quantises_onto_the_aim_maps_own_grid(corpus):
    """A pose cell and an aim-map node name the same physical pose (150 mm
    pitch), while z and the flight time get their own finer cells."""
    admitted = [r for r in corpus if lib.admit_record(r)[0]]
    keys = {lib.goal_key(r) for r in admitted} - {None}
    assert keys, 'no recoverable goal cell in the corpus'
    for key in keys:
        x, y, z, t = key
        assert abs(x / lib.POSE_CELL_MM - round(x / lib.POSE_CELL_MM)) < 1e-9
        assert abs(y / lib.POSE_CELL_MM - round(y / lib.POSE_CELL_MM)) < 1e-9
        assert z == pytest.approx(170.0)            # the ACTIVE plane, 10 mm cell
        assert t == pytest.approx(0.90)             # 50 ms cell
    import tilt_cal_grid                                            # noqa: E402
    nodes = tilt_cal_grid.build_axis(tilt_cal_grid.DEFAULT_BOX_MM, 3, 'x')
    assert lib.POSE_CELL_MM == pytest.approx(nodes[1] - nodes[0])


def test_the_goal_is_recovered_from_mined_only_rows(corpus):
    """``goal_catch_xyz_stow_mm`` is a declaration field and every mined-only row
    has it null, so the pose comes from ``land_xy_global_mm − land_err_mm`` and
    the plane walked back down the two GENERATED offsets."""
    admitted = [r for r in corpus if lib.admit_record(r)[0]]
    recovered = [lib.goal_of(r) for r in admitted]
    got = [g for g in recovered if g is not None]
    assert len(got) >= 15
    for g in got:
        assert g.catch_pose_stow_mm[2] == pytest.approx(170.0, abs=1e-6)
        assert abs(g.catch_pose_stow_mm[0]) <= 150.0 + 1e-6
        assert abs(g.catch_pose_stow_mm[1]) <= 150.0 + 1e-6


def test_the_admission_gate_is_the_miners_own_flag(corpus):
    """One quality bar for one population — ``usable_for_release_fit``, imported
    as a flag rather than re-derived. Two bars is how a headline becomes
    unreproducible."""
    for rec in corpus:
        ok, why = lib.admit_record(rec)
        if not rec.get('usable_for_release_fit'):
            assert not ok and 'usable_for_release_fit' in why


def test_the_E1_admission_refuses_a_pre_whole_arc_mine_and_says_so():
    """**The guard that makes lifting the E-1 mask safe.**

    ``load_corpus``'s documented glob (``temp/probes/toss_records_*.jsonl``)
    matches every mine of every bag, and de-duplicates by ``toss_uid`` — by mine
    STAMP, not by miner version. So a corpus can perfectly well contain rows
    mined before 2026-08-13, whose lateral velocities came from PER-BRANCH fits
    and therefore ARE the E-1 artefact: 10.9 mrad of phantom arrival direction,
    about 44 mm of phantom aim through the 4007 mm/rad landing gain, and
    repeatable, so averaging does not touch it. With the mask lifted and no
    gate, that row's
    aim error would be fitted as plant.

    ``coverage_asym_s`` is the discriminator, because only the whole-arc
    estimator emits it. Three conditions are pinned here, plus the property that
    makes the refusal cheap rather than corpus-destroying.
    """
    def row(**kw):
        base = {'usable_for_release_fit': True, 'flight_time_err_s': 0.1,
                'land_err_mm': [10.0, 20.0],
                'arrival_dir_err_rad': [0.003, 0.008],
                'coverage_asym_s': 0.01}
        base.update(kw)
        return base

    # 1. A modern row admits on every channel.
    good = row()
    assert lib.lateral_admissible(good) == (True, '')
    assert lib.admit_record(good, need_lateral=True)[0]
    assert not np.isnan(lib.measured_error(good)).any()

    # 2. A PRE-E-1 mine has no coverage_asym_s. Refused, and the reason names
    #    the mechanism rather than saying "missing field".
    stale = row()
    del stale['coverage_asym_s']
    ok, why = lib.lateral_admissible(stale)
    assert not ok and 'whole-arc' in why
    assert not lib.admit_record(stale, need_lateral=True)[0]

    # 3. A half-seen arc: present, out of bound. ABSOLUTE against the miner's
    #    constant, so widening COVERAGE_ASYM_MAX_S fails here.
    import toss_record_miner as miner                                # noqa: E402
    assert lib.lateral_admissible(
        row(coverage_asym_s=miner.COVERAGE_ASYM_MAX_S * 0.99))[0]
    ok, why = lib.lateral_admissible(
        row(coverage_asym_s=miner.COVERAGE_ASYM_MAX_S * 1.01))
    assert not ok and 'coverage_asym_s' in why

    # THE PROPERTY THAT MAKES IT CHEAP: a refused row keeps its VERTICAL
    # channel. `pooled_error` averages per channel, so the stale row still
    # contributes the flight time it measured perfectly well — losing the whole
    # row would be paying for the artefact twice.
    e = lib.measured_error(stale)
    assert np.isnan(e[:4]).all()
    assert e[4] == pytest.approx(0.1)
    assert lib.admit_record(stale)[0], 'the vertical admission must not move'
    pooled, counts = lib.pooled_error([good, stale])
    assert list(counts) == [1, 1, 1, 1, 2]
    assert pooled[1] == pytest.approx(20.0)     # good's value, not an average


def test_a_channel_with_no_measurements_is_not_fitted_as_a_measured_zero():
    """``pooled_error`` returns 0.0 with a count of 0 for an unmeasured channel,
    and a 0.0 residual under a non-zero weight is NOT "no information" — it is
    "this channel is already perfect", which cancels corrections the other
    channels ask for on a coupled column.

    Invisible while the E-1 mask zeroed the lateral channels anyway; reachable
    the moment it was lifted, which is why ``fit_corpus`` intersects the mask
    with the counts and returns both.
    """
    goal = lib.TossGoal(catch_pose_stow_mm=(0.0, 150.0, 170.0),
                        flight_time_s=lib.CORPUS_FLIGHT_TIME_S)
    rows = lib.synthetic_corpus(goal, u_plant=np.array([0.004, 0.0, 0.02, 0.0]),
                                n=40, seed=11)
    for r in rows:                       # a corpus with NO lateral measurement
        del r['coverage_asym_s']
    fit = lib.fit_corpus(rows, goal=goal)
    assert list(fit['channel_counts']) == [0, 0, 0, 0, 40]
    assert list(fit['mask']) == [1.0] * lib.N_E
    assert list(fit['mask_effective']) == [0.0, 0.0, 0.0, 0.0, 1.0]
    # The vertical correction still comes out; the aim channels are silent
    # rather than confidently zero-seeking.
    assert fit['du'][2] < 0.0
    assert fit['du'][0] == pytest.approx(0.0, abs=1e-12)
    assert fit['screen']['retained_labels'] == ('event_vel_trim',)


def test_the_G1_uptime_refusal_is_available_and_its_cost_is_visible(corpus):
    """Defence in depth against risk 1 (learning a moving plant) — and the
    number that makes it a Gate-1 decision rather than a default.

    16 of the 19 admitted rows come from a sitting recorded at 16.7 h of
    can-bridge uptime, so ANY healthy-uptime threshold below that collapses the
    corpus to 3 rows. The refusal therefore ships DISABLED and
    :func:`ilc_fit_lib.uptime_census` prints the cost; choosing the threshold is
    G-1's, not this module's.
    """
    admitted = [r for r in corpus if lib.admit_record(r)[0]]
    census = lib.uptime_census(admitted)
    assert census['max_h'] > 12.0
    assert census['buckets']['>12h'] >= 10
    kept = [r for r in corpus
            if lib.admit_record(r, uptime_max_ms=2 * 3.6e6)[0]]
    assert 0 < len(kept) < len(admitted)


# ── end-to-end: the CLI ──────────────────────────────────────────────────────


def test_cli_self_check_passes():
    assert cli.main(['--self-check']) == 0


def test_cli_validate_runs_all_four_validations_and_exits_zero():
    # Admission-aware guard, not presence-only: V2b/V3/V4 REFUSE on an empty
    # admitted set, so "exit zero" presupposes a corpus this tree can fit.
    paths, _rows, reason = _corpus_or_skip_reason()
    if reason:
        pytest.skip(reason)
    argv = ['--validate', '--allow-cross-partition', '--corpus'] + paths
    assert cli.main(argv) == 0


def test_cli_refuses_a_cross_partition_fit_without_the_flag():
    # Same guard, and here it is what keeps the assertion HONEST: with a
    # stale corpus the CLI also returns 1, for want of admissible rows rather
    # than for the cross-partition pooling this test exists to pin.
    paths, _rows, reason = _corpus_or_skip_reason()
    if reason:
        pytest.skip(reason)
    rc = cli.main(['--corpus'] + paths)
    assert rc == 1


def test_the_module_imports_no_ros():
    """The fit is offline by construction (design constraint 2): importing it
    must not pull ``rclpy`` in, so the whole thing runs in the ordinary suite."""
    out = subprocess.run(
        [sys.executable, '-c',
         'import sys; sys.path.insert(0, {!r}); import ilc_fit_lib; '
         'print("rclpy" in sys.modules)'.format(_HW_DIR)],
        capture_output=True, text=True, cwd=_REPO)
    assert out.returncode == 0, out.stderr
    assert out.stdout.strip() == 'False'
