"""FK convergence-criterion contract tests (``ik_solver.leg_lengths_to_pose``).

Guards the criterion described in the *FK CONVERGENCE CRITERION* block in
``ros_ws/src/jugglebot/jugglebot/motion/ik_solver.py``.  Both halves are
load-bearing and each has its own test here:

  A. a solve that has converged as far as double precision allows MUST return
     (the spurious-failure class this file was written for);
  B. a GENUINE divergence MUST still raise ``RuntimeError`` — dropping that
     would trade a loud spurious failure for a silent wrong answer, which is
     strictly worse.

Plan: ``plans/archived/2026-08-15 fk-convergence-tolerance.md`` (Phases 0-1).

Why the criterion needed changing — measured, not asserted
----------------------------------------------------------
The residual is ``|leg_vector| - init_leg_length``, a difference of two
650-870 mm quantities, so its achievable floor scales with the absolute leg
length and the Jacobian conditioning.  Phase 0 sweep (765 in-envelope poses,
cold start, tilt 0-15 deg, extensions 0-280 mm, run 2026-07-25):

  * floor residual p50 = 1.14e-13 mm, p95 = 4.32e-13 mm, max = 1.53e-10 mm;
  * ``floor <= 2.01 * eps * L * cond(J)`` bounded every single sample;
  * the round-trip POSE error at that floor was <= 4.60e-13 mm / 8.20e-13 rad
    — 5.3e-11 of one leg encoder dead-band (8.607e-3 mm).

A wider 13001-pose sweep at review time (tilt 0-18 deg, z 0-275 mm, |xy| <=
130 mm, 4000 of them random; run 2026-07-25) found 85 poses whose floor exceeds
the Phase 0 max, worst 2.18e-10 mm — still 0 raises at the shipped default.
Where a number below is compared against "the worst floor", it is that 2.18e-10
mm, not the Phase 0 figure: no grid bounds the tail, which is why the stagnation
exit ships as a backstop rather than the relative term shipping alone.

So the shipped ``tol = 1e-10`` mm absolute was below the reachable floor at
part of the workspace, and it rejected *exact* answers there.

Hardware fixtures
-----------------
Both fixtures below are real recorded hardware vectors, not synthesised:

``FIX_SEED`` — ``/robot_state`` ``motor_states[i].pos_estimate``, bag
``~/Desktop/rosbags/2026-07-25_15-22-50``, log_time 1784957069.071425
(2026-07-25 15:24:29).  This is the exact vector behind
``~/.ros/log/python3_31420_1784956973167.log:10``:
``seed FK failed (FK did not converge after 50 iterations (max residual:
1.07e-10 mm)) — not streaming until a valid state``.  That is
``trajectory_node._seed_hold_from`` refusing to stream, which is a
safety-relevant transition (no setpoints are emitted until a seed lands).
The same defect produced a 26-ERROR / 286 ms sustained refusal on
2026-07-24 09:08:55 (``~/.ros/log/python3_198327_1784848076544.log``).

``FIX_ECHO`` — ``/leg_setpoint_echo`` sample 2166 of bag
``~/Desktop/rosbags/2026-07-25_15-17-48``; the worst of the 37 vectors (of
10453, 0.354 %) in that session whose offline FK reconstruction failed at the
shipped default.  Residual 2.05e-10 mm; the whole failing set spans
1.00e-10..2.05e-10 mm.

Both are stored as mm extensions (``pos_estimate / geom.mm_to_rev``) at full
float64 precision so the test needs no bag at run time.
"""

from __future__ import annotations

from unittest.mock import patch

import numpy as np
import pytest
from numpy.linalg import norm

import jugglebot.motion.ik_solver as iks
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import (
    FK_DEFAULT_ATOL_MM,
    FK_STALL_CEILING_MM,
    leg_lengths_to_pose,
    pose_to_leg_lengths,
    rot_matrix_to_rotvec,
    rotvec_to_rot_matrix,
)
from jugglebot.motion.workspace import check_leg_extensions

# --- the two hardware fixtures (leg extensions, mm) ------------------------
FIX_SEED = np.array([155.09416589436165, 155.01513731044534, 154.52316200811464,
                     154.78598783488047, 154.67581630698984, 154.34857784371874])
FIX_ECHO = np.array([150.74095529505567, 154.35368362987822, 153.21159094983648,
                     159.90809011836723, 160.09638426184677, 149.92103849396517])
FIXTURES = {'seed_2026-07-25_15-24-29': FIX_SEED,
            'echo_2026-07-25_15-17-48': FIX_ECHO}

# The historical ACCEPTANCE TEST, expressed through the back-compatible knobs:
# absolute-only, no stagnation exit.
#
# Read this precisely: these knobs reproduce the pre-2026-07-25 *acceptance
# test*, NOT the pre-2026-07-25 control flow.  The post-loop honest-residual
# re-check has no off switch, so a solve whose FINAL step lands under ``tol``
# returns here where the deployed old code raised.  Consequences, both real:
#   * ``test_explicit_loose_tol_is_unchanged`` below therefore proves the
#     criterion is a no-op for the loose-tol caller on every call that
#     CONVERGES, and says nothing about the exhaustion path — which is why
#     ``test_exhaustion_returns_when_the_final_step_converges`` pins that path
#     against an explicit expectation instead;
#   * ``tools/probes/fk_convergence_bag_check.py``'s ``hist_raise`` column is
#     for the same reason a lower bound on what the deployed old code rejected.
HISTORICAL = dict(rtol=0.0, stall_ceiling_mm=0.0)


@pytest.fixture(scope='module')
def geom():
    return StewartGeometry()


def _sweep_poses():
    """Phase 0's structured grid: the reachable envelope INCLUDING the
    low-extension (near-stow) end and the tilt band the catch/reload path uses
    (the 2026-07-25 reload pre-tilt measured 11.08 deg)."""
    out = []
    for z in (2.0, 10.0, 25.0, 50.0, 80.0, 110.0, 140.0, 170.0, 200.0, 230.0):
        for r, th in ((0.0, 0.0), (40.0, 60.0), (40.0, 250.0),
                      (80.0, 0.0), (80.0, 150.0)):
            x, y = r * np.cos(np.deg2rad(th)), r * np.sin(np.deg2rad(th))
            for tilt_deg in (0.0, 3.0, 7.0, 11.08, 15.0):
                for ax in ((1.0, 0.0), (0.7071, 0.7071)):
                    if tilt_deg == 0.0 and ax != (1.0, 0.0):
                        continue
                    rv = np.deg2rad(tilt_deg) * np.array([ax[0], ax[1], 0.0])
                    out.append((np.array([x, y, z]), rotvec_to_rot_matrix(rv)))
    return out


def _in_envelope(geom):
    """Grid poses whose leg extensions are inside the workspace hard limits."""
    keep = []
    for pos, rot in _sweep_poses():
        ext = pose_to_leg_lengths(pos, rot, geom)
        if check_leg_extensions(ext, geom)[0]:
            keep.append((pos, rot, ext))
    return keep


def _residual(ext, pos, rot, geom):
    return float(np.max(np.abs(pose_to_leg_lengths(pos, rot, geom) - ext)))


# ---------------------------------------------------------------------------
# A — converged-to-round-off must RETURN
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('name', sorted(FIXTURES))
def test_hardware_fixture_converges_at_the_default(name, geom):
    """The two recorded hardware vectors return a pose at the shipped default.

    Fails (RuntimeError) without the criterion fix — that is precisely the
    hardware symptom: ``trajectory_node`` refusing to stream.
    """
    ext = FIXTURES[name]
    pos, rot, J = leg_lengths_to_pose(ext, geom)
    assert np.all(np.isfinite(pos)) and np.all(np.isfinite(rot))
    assert J.shape == (6, 6)
    # The accepted pose is exact to well under a picometre — the residual the
    # old criterion rejected was numerical noise, not error.
    assert _residual(ext, pos, rot, geom) <= 1e-9
    ext_back = pose_to_leg_lengths(pos, rot, geom)
    assert np.max(np.abs(ext_back - ext)) <= 1e-9
    # And it gets there in a handful of Newton steps, not the 50-iteration
    # (7.4 ms measured) grind the old criterion forced.
    assert leg_lengths_to_pose.last_iterations <= 8


@pytest.mark.parametrize('name', sorted(FIXTURES))
def test_hardware_fixture_raises_under_the_historical_criterion(name, geom):
    """Guard the guard: with the relative term and the stagnation exit both
    switched off, the fixtures still raise.  If this ever stops raising the
    fixtures have gone stale and tests above no longer prove anything.
    """
    with pytest.raises(RuntimeError, match='did not converge'):
        leg_lengths_to_pose(FIXTURES[name], geom, **HISTORICAL)


@pytest.mark.parametrize('name', sorted(FIXTURES))
def test_relative_term_alone_rescues_the_fixtures(name, geom):
    """Half A of the criterion in isolation: mixed abs/rel test, stagnation
    exit disabled.  Proves the relative term is independently sufficient, so a
    future change that removes the stagnation exit cannot silently reopen the
    failure.
    """
    ext = FIXTURES[name]
    pos, rot, _ = leg_lengths_to_pose(ext, geom, stall_ceiling_mm=0.0)
    assert _residual(ext, pos, rot, geom) <= 1e-9


@pytest.mark.parametrize('name', sorted(FIXTURES))
def test_stagnation_exit_alone_rescues_the_fixtures(name, geom):
    """Half B in isolation: stagnation exit only, relative term disabled.
    Proves the stagnation exit is independently sufficient — it is the backstop
    for any pose whose round-off floor is worse than the Phase 0 sweep saw
    (a future geometry, BLAS or numpy change).
    """
    ext = FIXTURES[name]
    pos, rot, _ = leg_lengths_to_pose(ext, geom, rtol=0.0)
    assert _residual(ext, pos, rot, geom) <= FK_STALL_CEILING_MM


def test_workspace_sweep_never_raises_at_the_default(geom):
    """The contract test: nowhere in the reachable envelope may a cold-start
    FK of its own IK output raise.

    This is the test that fails if anyone re-tightens the criterion back to a
    bare absolute tolerance.  At the shipped ``tol=1e-10`` alone, 3 of these
    299 poses raise (measured 2026-07-25; the figure 5 belongs to the larger
    765-pose Phase 0 grid, which is not what this test iterates).
    """
    poses = _in_envelope(geom)
    assert len(poses) >= 250, 'grid degenerated — the sweep must stay broad'
    worst_res = 0.0
    worst_pos_err = 0.0
    worst_rot_err = 0.0
    max_iters = 0
    for pos, rot, ext in poses:
        recovered_pos, recovered_rot, _ = leg_lengths_to_pose(ext, geom)
        max_iters = max(max_iters, leg_lengths_to_pose.last_iterations)
        worst_res = max(worst_res, _residual(ext, recovered_pos, recovered_rot, geom))
        worst_pos_err = max(worst_pos_err, float(norm(recovered_pos - pos)))
        worst_rot_err = max(
            worst_rot_err,
            float(norm(rot_matrix_to_rotvec(recovered_rot @ rot.T))))
    # Accepted residuals stay physically meaningless: the leg encoder
    # dead-band is 8.6e-3 mm, so 1e-8 mm is ~1e-6 of one count.
    assert worst_res <= 1e-8, f'worst accepted residual {worst_res:.3e} mm'
    # And the POSE the criterion accepts is right, which is the number that
    # actually matters (Phase 0 measured <= 4.60e-13 mm / 8.20e-13 rad).
    assert worst_pos_err <= 1e-8, f'worst pose error {worst_pos_err:.3e} mm'
    assert worst_rot_err <= 1e-8, f'worst rot error {worst_rot_err:.3e} rad'
    # Convergence is complete in a handful of steps; the 50-iteration budget
    # was only ever being spent grinding at the round-off floor.
    assert max_iters <= 10, f'took {max_iters} Newton iterations'


def test_sweep_contains_poses_the_historical_criterion_rejected(geom):
    """Teeth for the test above: the sweep must still contain poses the bare
    absolute tolerance rejects, otherwise it would pass with the fix reverted.

    3 of the 299 in-envelope grid poses raise under ``HISTORICAL`` (measured
    2026-07-25).  If geometry or the grid ever changes such that none do, the
    sweep has stopped covering the round-off-floor region and must be widened
    — the assertion below is what surfaces that.
    """
    rejected = 0
    for _pos, _rot, ext in _in_envelope(geom):
        try:
            leg_lengths_to_pose(ext, geom, **HISTORICAL)
        except RuntimeError:
            rejected += 1
    assert rejected >= 1, (
        'no grid pose reaches the round-off floor any more — widen the sweep, '
        'or test_workspace_sweep_never_raises_at_the_default proves nothing')


def test_low_extension_end_is_covered_and_converges(geom):
    """The relative term must not misbehave near stow, where leg *extensions*
    approach zero.  ``scale`` is the ABSOLUTE leg length (>= ~648 mm), not the
    extension, so it cannot collapse — this pins that choice.
    """
    ext = pose_to_leg_lengths(np.array([0.0, 0.0, 6.0]), np.eye(3), geom)
    assert float(np.max(np.abs(ext))) < 10.0, 'not actually near stow'
    scale = iks._residual_scale_mm(ext, geom)
    assert scale > 600.0, f'scale collapsed near stow: {scale:.3f} mm'
    pos, rot, _ = leg_lengths_to_pose(ext, geom)
    assert _residual(ext, pos, rot, geom) <= 1e-8
    assert norm(pos - np.array([0.0, 0.0, 6.0])) <= 1e-8


def test_final_newton_step_is_no_longer_discarded(geom):
    """The loop evaluates the residual BEFORE stepping, so on exhaustion the
    pose it holds has been advanced once more than the last residual measured.
    A post-loop re-check must accept it if that final step converged.

    Driven with ``max_iter=1`` from a guess 1e-4 mm off: the pre-step residual
    is 3.96e-05 mm (4.5e4 x the acceptance threshold, so the pre-step check
    cannot fire) and the single step lands at 6.14e-12 mm (144 x below it).
    Empirically verified 2026-07-25.
    """
    pos = np.array([3.0, -2.0, 140.0])
    rot = rotvec_to_rot_matrix(np.deg2rad(5.0) * np.array([1.0, 0.0, 0.0]))
    ext = pose_to_leg_lengths(pos, rot, geom)
    guess = (pos + np.array([1e-4, 0.0, 0.0]), rot.copy())
    pre_step_res = _residual(ext, guess[0], guess[1], geom)
    assert pre_step_res > 1e-5, 'guess too close — the pre-step check would fire'

    out_pos, out_rot, _ = leg_lengths_to_pose(
        ext, geom, initial_guess=guess, max_iter=1)
    assert _residual(ext, out_pos, out_rot, geom) <= 1e-9
    assert leg_lengths_to_pose.last_iterations == 1


def test_post_loop_recheck_reaches_the_stagnation_disjunct(geom):
    """Cover the post-loop re-check's *stagnation* half, not just its mixed-test
    half.

    Both acceptance branches live in one predicate (``_fk_accepted``) precisely
    so they cannot drift, but the post-loop call site is the least-travelled
    path in the function, so its stagnation disjunct gets an explicit driver:
    without one, a hardening edit to the stagnation rule (two consecutive
    stalls, a higher ``stall_factor``) could be verified only against the
    in-loop path.

    Recipe, empirically established 2026-07-25: ``FIX_SEED`` with ``rtol=0``
    (so the mixed test cannot fire — the floor residual 1.07e-10 mm exceeds
    ``tol``) accepts through the IN-loop stagnation branch at iteration 6.  Cap
    ``max_iter`` at 5 and the loop exhausts one step earlier, so the identical
    pose is accepted by the POST-loop re-check instead.  The two runs returning
    the same residual at different ``last_iterations`` is what proves the
    max_iter=5 acceptance came from after the loop.
    """
    ext = FIX_SEED
    pos5, rot5, _ = leg_lengths_to_pose(ext, geom, rtol=0.0, max_iter=5)
    iters5 = leg_lengths_to_pose.last_iterations
    pos6, rot6, _ = leg_lengths_to_pose(ext, geom, rtol=0.0, max_iter=6)
    iters6 = leg_lengths_to_pose.last_iterations

    res5 = _residual(ext, pos5, rot5, geom)
    assert iters5 == 5 and iters6 == 6, (
        f'recipe drifted (it={iters5}/{iters6}) — max_iter=5 must EXHAUST '
        f'where max_iter=6 accepts in-loop, or this no longer covers the '
        f'post-loop path')
    assert res5 == pytest.approx(_residual(ext, pos6, rot6, geom), rel=1e-9)
    # It was accepted by the stagnation disjunct, not the mixed test.
    assert res5 > FK_DEFAULT_ATOL_MM, 'mixed test could have fired — rtol leak?'
    assert res5 <= FK_STALL_CEILING_MM


def test_divergence_message_describes_the_abandoned_pose(geom):
    """The residual quoted in the ``RuntimeError`` must be the residual of the
    pose actually being abandoned, not the one from before the final step.

    Measured over the Phase 0 sweep the pre-step figure over-states the
    abandoned pose's residual by up to 3.00x; here the ratio is ~6.3e3, which
    makes the assertion unambiguous.  A safety-relevant error message that
    mis-attributes its own number sends the next reader after the wrong
    magnitude — which is how this defect's evidence table came to quote
    residuals that never described any returned pose.
    """
    pos = np.array([3.0, -2.0, 140.0])
    rot = rotvec_to_rot_matrix(np.deg2rad(5.0) * np.array([1.0, 0.0, 0.0]))
    ext = pose_to_leg_lengths(pos, rot, geom)
    guess = (pos + np.array([1e-1, 0.0, 0.0]), rot.copy())
    pre_step_res = _residual(ext, guess[0], guess[1], geom)
    assert pre_step_res > 1e-2, 'fixture drifted — pre-step residual too small'

    with pytest.raises(RuntimeError) as exc:
        leg_lengths_to_pose(ext, geom, initial_guess=guess, max_iter=1)
    quoted = float(str(exc.value).split('max residual: ')[1].split(' mm')[0])
    assert quoted < 1e-4, (
        f'quoted residual {quoted:.3e} mm looks like the PRE-step value '
        f'({pre_step_res:.3e} mm), not the abandoned pose\'s')


# ---------------------------------------------------------------------------
# B — genuine divergence must still RAISE
# ---------------------------------------------------------------------------

# Extension sets with no pose that satisfies them.  All three are geometrically
# unsatisfiable, not merely out of stroke: base radius 410 mm and platform
# radius 219 mm put a hard lower bound on any leg length well above these.
# (Contrast: extensions past the physical stroke, e.g. 3x stroke, ARE
# mathematically reachable and DO converge — FK is pure kinematics and
# ``workspace.check_leg_extensions`` owns stroke enforcement.  See
# test_out_of_stroke_but_reachable_still_converges.)
UNSATISFIABLE = {
    'alternating +-600 mm': np.array([600., -600., 600., -600., 600., -600.]),
    'all legs -640 mm (past full retract)': np.full(6, -640.0),
    'all legs at zero absolute length': np.full(6, -648.419),
}


@pytest.mark.parametrize('name', sorted(UNSATISFIABLE))
def test_genuine_divergence_still_raises(name, geom):
    """A genuinely unsatisfiable extension set must still raise.

    Also asserts the residual at the raise is orders of magnitude ABOVE
    ``FK_STALL_CEILING_MM`` — i.e. the stagnation exit is not what stops these
    from being silently accepted; the ceiling structurally excludes them.  A
    stagnation exit without that ceiling would return a wrong pose here.
    """
    with pytest.raises(RuntimeError, match='did not converge') as exc:
        leg_lengths_to_pose(UNSATISFIABLE[name], geom)
    quoted = float(str(exc.value).split('max residual: ')[1].split(' mm')[0])
    assert quoted > 1e3 * FK_STALL_CEILING_MM, (
        f'residual at raise {quoted:.3e} mm is close to the stagnation '
        f'ceiling {FK_STALL_CEILING_MM:.0e} mm — the ceiling is doing less '
        f'work than this test assumes')


def test_singular_jacobian_still_raises(geom):
    """The degenerate-Jacobian raise path survives the criterion change.

    A rank-5 Jacobian makes ``np.linalg.solve`` raise ``LinAlgError``, which
    the solver must surface as ``RuntimeError`` rather than return a pose from.
    """
    ext = pose_to_leg_lengths(np.array([0.0, 0.0, 110.0]), np.eye(3), geom)
    singular = np.eye(6)
    singular[0, 0] = 0.0
    with patch.object(iks, 'compute_jacobian', return_value=singular):
        with pytest.raises(RuntimeError, match='singular Jacobian'):
            # A guess far enough out that iteration 0 cannot accept.
            leg_lengths_to_pose(
                ext, geom,
                initial_guess=(np.array([20.0, 0.0, 60.0]), np.eye(3)))


@pytest.mark.parametrize('bad', [np.nan, np.inf, -np.inf])
@pytest.mark.parametrize('tol,max_iter', [(FK_DEFAULT_ATOL_MM, 50), (1e-4, 10)])
def test_non_finite_extensions_raise(bad, tol, max_iter, geom):
    """A non-finite target must be rejected, at BOTH call shapes.

    ``+-inf`` is the case that matters and the reason FK validates finiteness
    explicitly rather than leaning on the comparisons: the acceptance threshold
    is ``tol + rtol * scale``, so an infinite target makes it ``inf``, every
    ``<=`` against it vacuously true, and FK would return its *initial guess*
    as a converged pose with an infinite residual — 1 iteration, no error.
    Measured before the guard landed (2026-07-25): cold start returned
    ``pos=[0,0,0]``; ``hardware_plant``'s shape (warm guess, ``tol=1e-4``)
    returned the warm guess verbatim, which would have cleared
    ``_fk_fail_count`` and made the ``fk_convergence_failure`` e-stop
    unreachable while the MPC closed its loop on a frozen pose.

    A non-finite ``pos_estimate`` is a live concern on this robot, not a
    hypothetical: ``teensy_bridge_node`` documents ``pos_rev is NaN until the
    encoder is ready`` and guards it in two places.

    NaN raised before the guard too (via the all-False comparisons), but only
    after burning the whole iteration budget and with a message reading
    ``max residual: nan mm``.
    """
    ext = pose_to_leg_lengths(np.array([0.0, 0.0, 110.0]), np.eye(3), geom)
    ext = ext.copy()
    ext[3] = bad
    guess = (np.array([0.0, 0.0, 110.0]), np.eye(3))
    with pytest.raises(RuntimeError, match='non-finite'):
        leg_lengths_to_pose(ext, geom, initial_guess=guess,
                            tol=tol, max_iter=max_iter)


def test_out_of_stroke_but_reachable_still_converges(geom):
    """Extensions past the physical stroke are *mathematically* reachable, and
    FK must keep solving them — unchanged from before the fix.

    Pinning this stops a future "tighten FK so it rejects impossible poses"
    change from moving stroke enforcement out of
    ``workspace.check_leg_extensions``, where every planner gate reads it.
    """
    ext = np.full(6, 3.0 * geom.leg_stroke_mm)
    assert not check_leg_extensions(ext, geom)[0], 'fixture is in stroke'
    pos, rot, _ = leg_lengths_to_pose(ext, geom)
    assert _residual(ext, pos, rot, geom) <= 1e-8


# ---------------------------------------------------------------------------
# Back-compatibility for callers that pass an explicit tolerance
# ---------------------------------------------------------------------------

def test_explicit_loose_tol_is_unchanged(geom):
    """``HardwarePlant.get_state`` calls FK with ``tol=_FK_TOL_MM`` (1e-4) and
    ``max_iter=10`` from a warm guess, inside the 40 Hz MPC hot loop.  Every
    call there that CONVERGES must be bit-for-bit what it was before the
    criterion changed.

    Structural argument, then the measurement: the stagnation exit is checked
    strictly AFTER the mixed test and can only fire below
    ``FK_STALL_CEILING_MM`` (1e-6 mm), which is 100x tighter than 1e-4 — so
    for that caller the mixed test always fires first and the stagnation exit
    is unreachable.  The relative term adds ~9e-10 mm to a 1e-4 mm threshold,
    which cannot change any comparison outcome.  Below, the 299-pose
    in-envelope subset of the Phase 0 grid is run both ways and required to
    agree exactly (0 mismatches measured 2026-07-25; the same comparison over
    the full 765-pose Phase 0 grid, which lives in the uncommitted sweep probe,
    also gave 0).

    Scope, stated because it is easy to over-read: ``HISTORICAL`` disables the
    relative term and the stagnation exit, not the post-loop re-check, so this
    test cannot see the EXHAUSTION path — see
    ``test_exhaustion_returns_when_the_final_step_converges``.  The ceiling-vs-
    caller-tol invariant is pinned against the real call site's constant in
    ``tests/sim/test_hardware_plant_failure_paths.py``; the assertion below is
    the ik_solver-side half.
    """
    assert FK_STALL_CEILING_MM * 100.0 <= 1e-4, (
        'the stagnation ceiling has risen to within 100x of hardware_plant\'s '
        'tol=1e-4 — the unreachability argument above no longer holds')
    mismatches = []
    for pos, rot, ext in _in_envelope(geom):
        guess = (pos + np.array([0.5, -0.3, 0.7]), rot.copy())
        new = leg_lengths_to_pose(ext, geom, initial_guess=guess,
                                  max_iter=10, tol=1e-4)
        new_iters = leg_lengths_to_pose.last_iterations
        old = leg_lengths_to_pose(ext, geom, initial_guess=guess,
                                  max_iter=10, tol=1e-4, **HISTORICAL)
        old_iters = leg_lengths_to_pose.last_iterations
        if (new_iters != old_iters
                or not np.array_equal(new[0], old[0])
                or not np.array_equal(new[1], old[1])
                or not np.array_equal(new[2], old[2])):
            mismatches.append(np.round(pos, 2).tolist())
    assert not mismatches, (
        f'{len(mismatches)} poses where the loose-tol caller changed '
        f'behaviour, e.g. {mismatches[:5]}')


def test_exhaustion_returns_when_the_final_step_converges(geom):
    """The one path where the criterion change is NOT a no-op for a caller
    passing an explicit loose ``tol`` — pinned so it stays visible.

    A/B against the real pre-change function (2026-07-25, ``git show HEAD``
    extracted to a scratch module) at ``hardware_plant``'s ``tol=1e-4`` shape:

      | warm-guess offset | max_iter | OLD    | NEW                |
      |-------------------|----------|--------|--------------------|
      | +0.5 mm in x      | 2        | RAISE  | RETURN res 3.1e-12 |
      | +0.1 mm in x      | 1        | RAISE  | RETURN res 6.2e-06 |
      | +20 mm in x       | 2        | RAISE  | RETURN res 8.2e-06 |
      | +0.5 mm in x      | 10       | RETURN | RETURN (identical) |

    For ``HardwarePlant.get_state`` this means a tick whose 10th Newton step is
    the first to land under ``tol`` no longer increments ``_fk_fail_count``
    toward ``_FK_FAIL_ESTOP_THRESHOLD`` and no longer substitutes
    ``_last_measured_pose``.  That is the intended trade — the returned pose
    satisfies the tolerance the caller asked for, so calling it a failure and
    feeding the MPC a staler pose instead was the worse of the two — but it is
    a deliberate reduction in that cascade's sensitivity, so it gets a test of
    its own rather than living only in a comment.

    The two assertions that matter: the returned pose really is within the
    caller's ``tol`` (not merely "returned"), and the iteration budget really
    was exhausted (so this is the exhaustion path and not an early accept).
    """
    pos = np.array([3.0, -2.0, 140.0])
    rot = rotvec_to_rot_matrix(np.deg2rad(5.0) * np.array([1.0, 0.0, 0.0]))
    ext = pose_to_leg_lengths(pos, rot, geom)
    guess = (pos + np.array([0.5, 0.0, 0.0]), rot.copy())
    tol = 1e-4

    # Pre-step residual at the LAST in-loop evaluation must be above tol,
    # otherwise the in-loop check accepts and this is not the exhaustion path.
    assert _residual(ext, guess[0], guess[1], geom) > tol

    out_pos, out_rot, J = leg_lengths_to_pose(
        ext, geom, initial_guess=guess, tol=tol, max_iter=2)
    assert leg_lengths_to_pose.last_iterations == 2, 'not the exhaustion path'
    assert _residual(ext, out_pos, out_rot, geom) <= tol, (
        'the exhaustion path returned a pose OUTSIDE the caller\'s tolerance '
        '— the post-loop re-check must apply the caller\'s own criterion')
    assert J.shape == (6, 6)

    # And a genuine divergence at the same loose tol still raises, so the
    # post-loop re-check has not turned exhaustion into a blanket accept.
    with pytest.raises(RuntimeError, match='did not converge'):
        leg_lengths_to_pose(UNSATISFIABLE['alternating +-600 mm'], geom,
                            tol=tol, max_iter=2)


def test_default_atol_is_the_historical_value():
    """``tol``'s default must stay at the historical 1e-10 mm.  The fix works
    by ADDING a relative term and a stagnation exit, not by loosening the
    absolute one — so a caller that reads ``tol`` as "the absolute residual I
    get" keeps the same guarantee.
    """
    assert FK_DEFAULT_ATOL_MM == 1e-10


def test_scale_is_the_absolute_leg_length(geom):
    """``_residual_scale_mm`` must be the absolute leg length at the target,
    because that is the operand size of the cancellation that sets the
    round-off floor.  A scale derived from *extensions* instead would collapse
    near stow and re-open the failure exactly where the risk register warned.
    """
    for z in (5.0, 110.0, 230.0):
        ext = pose_to_leg_lengths(np.array([0.0, 0.0, z]), np.eye(3), geom)
        expected = float(np.max(np.abs(ext + geom.init_leg_lengths_mm)))
        assert iks._residual_scale_mm(ext, geom) == pytest.approx(expected)
        # sanity: it really is the leg length, i.e. ~648-930 mm
        assert 600.0 < iks._residual_scale_mm(ext, geom) < 1000.0
