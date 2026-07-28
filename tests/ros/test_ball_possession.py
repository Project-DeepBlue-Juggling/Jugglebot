"""ball_possession — the possession-verdict surface, scored on MEASURED fixtures.

Contract: ``ros_ws/docs/ball_possession_contract.md`` (**C-POSSESS-1**).

Every fixture in ``tests/ros/possession_fixtures.py`` is a real CAUGHT estimate
the ball tracker published during the 2026-07-27 validation sitting (bag
``2026-07-27_15-39-38``), extracted by
``tools/probes/possession_verdict_bag_check.py --emit-fixtures``. Nothing here is
synthetic: the whole point is that the pre-2026-07-28 gate scored **0 of 17**
real catches and this file is what makes that unrepeatable.

The two mutations that must break this file (checked 2026-07-28):

  * delete the arrival bound (``ok = True`` in ``TrackerArrivalSource.judge``) —
    the 18 reload fixtures flip to confirmed and
    ``test_every_corrupt_reload_track_is_refused`` goes red;
  * return ``RETENTION_CONFIRMED`` — ``test_tracker_source_never_claims_retention``
    goes red. That is the mutation that matters most: a source claiming an
    observation it does not make is the exact shape of the original defect.
"""

from __future__ import annotations

import importlib.util
import math
import os

import pytest

import jugglebot.hardware_config as hw
from jugglebot.ball_possession import (
    RETENTION_CONFIRMED,
    RETENTION_REJECTED,
    RETENTION_UNKNOWN,
    SOURCE_HAND_BALL_SENSOR,
    SOURCE_TRACKER_ARRIVAL,
    PossessionVerdict,
    TrackerArrivalSource,
    describe,
    lateral_miss_mm,
)
from tests.ros import possession_fixtures as fx

# The catch point the coordinator judges a reload against; the 2026-07-27
# self-tosses were nominated at (0, 0) so the same point serves both populations.
CATCH_POINT = (0.0, 0.0,
               hw.GEOM_INITIAL_HEIGHT_MM + hw.JB_OP_DEFAULT_ACTIVE_Z_MM
               + hw.HAND_CATCH_OFFSET_MM)
TOL = float(hw.GEOM_ARM_RADIUS_MM)


def _src(tol=TOL):
    return TrackerArrivalSource(arrival_tol_mm=tol)


def _judge(xyz, tol=TOL, ref=CATCH_POINT):
    return _src(tol).judge(ball_xyz_mm=xyz, ref_point_mm=ref)


# ── The headline regression: the verdict was structurally always False ────────

@pytest.mark.parametrize('ball_id,x,y,z', fx.SELF_TOSS_CAUGHT)
def test_every_real_self_toss_catch_is_confirmed(ball_id, x, y, z):
    """All 17 self-tosses of 2026-07-27 were catches the operator watched land, and
    all 17 reported MISSED. Each failed on the z bound alone: xy error 0.30-3.88 mm
    against z error 305-1007 mm, because a tracker CAUGHT estimate is a dead-reckoned
    free-fall extrapolation from the moment the marker vanished (C-POSSESS-1 § 1).

    This test is RED against the pre-2026-07-28 gate for every one of the 17."""
    v = _judge((x, y, z))
    assert v.arrival_ok is True
    assert v.confirmed is True
    assert v.arrival_err_mm <= TOL


def test_the_z_bound_that_broke_it_would_still_reject_all_seventeen():
    """Pins WHY the fix is a deletion, not a re-tune: no z bound survives this data.

    The tightest plane drop over the 17 real catches is 305 mm — twice the 150 mm
    bound that shipped, and 2.0x any bound that would also reject the corrupt
    tracks. A future 'let's just loosen z a bit' edit has to face this number."""
    drops = [abs(z - CATCH_POINT[2]) for _i, _x, _y, z in fx.SELF_TOSS_CAUGHT]
    assert min(drops) == pytest.approx(305.03, abs=0.5)
    assert max(drops) == pytest.approx(1007.14, abs=0.5)
    assert min(drops) > 150.0 * 2.0                 # the shipped bound, doubled


def test_plane_drop_is_report_only_and_cannot_veto():
    """The deepest-extrapolated real catch in the session (ball 123, 1007 mm below
    the catch plane) is still CONFIRMED. If a future edit reintroduces a z bound in
    any form, this goes red."""
    ball = next(b for b in fx.SELF_TOSS_CAUGHT if b[0] == 123)
    v = _judge(ball[1:])
    assert v.plane_drop_mm > 1000.0
    assert v.confirmed is True


# ── The negative set stays rejected ───────────────────────────────────────────

@pytest.mark.parametrize('ball_id,x,y,z', fx.RELOAD_CAUGHT)
def test_every_corrupt_reload_track_is_refused(ball_id, x, y, z):
    """Every destination-tagged reload track in the session is a split track whose
    filter is fed by the WRONG marker, so its CAUGHT estimate lands 204.9-752.9 mm
    away while a separate untagged track carries the real ball. Thirteen of these
    were real catches — refusing them is the honest verdict, because the estimate
    carries no evidence either way (C-POSSESS-1 § 4).

    (NOT "the filter received no measurements": all 18 reach tracking=CONFIRMED,
    which matcher.py:344-347 sets only inside kf.update(). Corrected 2026-07-28.)"""
    v = _judge((x, y, z))
    assert v.arrival_ok is False
    assert v.confirmed is False
    assert v.reason == 'ARRIVAL_FAR'


def test_the_2026_07_23_corrupt_track_is_refused():
    """The below-the-floor track the original gate was written against — a second
    capture, so the fixture set is not single-session."""
    assert _judge(fx.CORRUPT_2026_07_23).confirmed is False


def test_the_bound_sits_between_the_two_measured_populations():
    """The margins, so a future re-tune has to argue against numbers.

    The 200 mm bound this replaced cleared the corrupt floor by 4.9 mm — a 1.02x
    margin against minting a FALSE CAUGHT. The arm-aperture bound clears the real
    catches by 18x and sits 2.9x under the corrupt floor."""
    clean = [lateral_miss_mm((x, y, z), CATCH_POINT)
             for _i, x, y, z in fx.SELF_TOSS_CAUGHT]
    corrupt = [lateral_miss_mm((x, y, z), CATCH_POINT)
               for _i, x, y, z in fx.RELOAD_CAUGHT]
    assert max(clean) == pytest.approx(3.88, abs=0.01)
    assert min(corrupt) == pytest.approx(204.89, abs=0.01)
    assert max(clean) < TOL < min(corrupt)
    assert TOL / max(clean) > 18.0                  # headroom above real catches
    assert min(corrupt) / TOL > 2.9                 # headroom under corrupt tracks
    # ... and the bound it replaced had essentially none.
    assert min(corrupt) / 200.0 < 1.03


# ── The bound's KNOWN under-sizing on the reload path ─────────────────────────

# Arrival errors of the 2026-07-27 reload era's *untagged* CAUGHT tracks — the ones
# carrying the REAL marker, i.e. how a destination-tagged track will read once the
# mis-association (C-POSSESS-1 § 4) is fixed. Measured during finalize 2026-07-28
# by scanning /balls in 2026-07-27_15-39-38 for untagged tracks reaching CAUGHT
# within 150 mm of the catch point, ids 57 / 33 / 69 / 15.
MEASURED_REAL_RELOAD_ARRIVAL_MM = (34.4, 34.9, 37.6, 68.4)


def test_the_measured_reload_band_sits_against_the_bound():
    """The bound is knowingly UNDER-SIZED for a healthy reload path — pinned so the
    tracker phase cannot land without meeting this number.

    A genuine reload catch already sits 1.6 mm inside the aperture bound: a 1.02x
    margin, which is the *same* margin (and the same defect shape) that C-POSSESS-1
    § 1 condemns in the 200 mm bound it replaced. It is inert today only because
    every tagged reload track is refused at 204.9-752.9 mm regardless.

    This test does NOT assert the bound is correct. It asserts the margin is thin,
    so that a future re-tune has to argue against a measured number, and so that
    'reload verdicts read MISSED' can never again be assumed to be the tracker's
    fault alone. See ros_ws/docs/ball_possession_contract.md § 4."""
    worst = max(MEASURED_REAL_RELOAD_ARRIVAL_MM)
    # A real reload catch fits — but only just.
    assert worst < TOL, 'a measured real reload catch must still fit the bound'
    assert TOL - worst < 5.0, 'margin is thin: this is the point of the test'
    assert TOL / worst < 1.05, 'a 1.02x margin — the § 1 defect shape'
    # The catch-reach envelope is a SECOND, independent term pushing the same way:
    # the reference point does not move with the reach, so a reached-to catch can
    # read up to the full envelope even with a perfect estimate.
    assert float(hw.JB_TRAJ_CATCH_REACH_ENVELOPE_MM) > TOL, (
        'the reach envelope exceeds the arrival bound — a catch the platform is '
        'designed to reach for can land outside the bound that judges it')


def test_the_bound_is_not_vacuous_in_either_direction():
    """Mutation guard: both populations must be non-empty and land on opposite
    sides, so neither parametrised block can silently become an empty set."""
    assert len(fx.SELF_TOSS_CAUGHT) == 17
    assert len(fx.RELOAD_CAUGHT) == 18
    conf = [_judge(b[1:]).confirmed for b in fx.SELF_TOSS_CAUGHT]
    ref = [_judge(b[1:]).confirmed for b in fx.RELOAD_CAUGHT]
    assert all(conf) and not any(ref)


# ── RETENTION: the claim this source may never make ───────────────────────────

def test_tracker_source_never_claims_retention():
    """C-POSSESS-1 § 2, consequence 2 — and the mutation that matters most.

    The tracker cannot observe retention: measured on the same bag, the track's
    position variance after the first CAUGHT sample is exactly 0.000 mm on all 60+
    CAUGHT tracks (it freezes and is pruned ~2 s later), and a bounce-out's descent
    to the floor raises no successor track — zero new /balls tracks appeared between
    each of the three bounce-outs' CAUGHT verdict and its mocap floor arrival
    0.4-0.6 s later. A source that returns CONFIRMED here is asserting an
    observation it does not make, which is the defect this contract closes."""
    every = list(fx.SELF_TOSS_CAUGHT) + list(fx.RELOAD_CAUGHT)
    for _i, x, y, z in every:
        assert _judge((x, y, z)).retention == RETENTION_UNKNOWN
    # including a perfectly dead-centre estimate — there is no input for which
    # this source may claim the ball stayed put.
    assert _judge((0.0, 0.0, CATCH_POINT[2])).retention == RETENTION_UNKNOWN


def test_the_bounce_out_trap_is_pinned_at_the_true_arrival():
    """THE TRAP, in one test.

    The three 2026-07-27 bounce-outs entered the cup region at **small xy** — ball 6
    at (-7.2, +16.1) mm — before departing to the floor. Today the tracker's own
    estimate for all three is 700+ mm out, so the gate refuses them; but that is an
    accident of the corruption, and a gate whose correctness depends on the tracker
    staying broken is a trap. Scored at the ball's TRUE arrival — what a FIXED
    tracker would publish — an arrival-only verdict calls all three CAUGHT.

    So the contract does not let this source pretend otherwise: arrival is
    affirmative, retention is UNKNOWN, and the verdict carries that distinction to
    the caller and into the log. When the hand sensor supplies RETENTION_REJECTED
    for exactly this case, `confirmed` flips to False with NO change here or at any
    call site."""
    for ball_id, x, y in fx.BOUNCE_OUT_TRUE_ARRIVAL_XY_MM:
        v = _judge((x, y, CATCH_POINT[2]))
        assert v.arrival_ok is True, ball_id
        assert v.retention == RETENTION_UNKNOWN, ball_id
        # the same estimate with a source that CAN see the ball leave:
        rejected = v._replace(retention=RETENTION_REJECTED)
        assert rejected.confirmed is False, ball_id


def test_unknown_retention_does_not_veto_but_rejected_does():
    """C-POSSESS-1 § 2, consequence 3. A source that cannot see retention must not
    be able to refuse a catch it did see arrive — otherwise the contract recreates
    the original defect (a structurally-unreachable True) in the other direction."""
    base = PossessionVerdict(SOURCE_TRACKER_ARRIVAL, True, RETENTION_UNKNOWN,
                             1.0, 400.0, 'ARRIVAL_OK')
    assert base.confirmed is True
    assert base._replace(retention=RETENTION_CONFIRMED).confirmed is True
    assert base._replace(retention=RETENTION_REJECTED).confirmed is False
    # a failed arrival is never rescued by retention
    far = base._replace(arrival_ok=False)
    assert far.confirmed is False
    assert far._replace(retention=RETENTION_CONFIRMED).confirmed is False


# ── Shape, formula and wording ────────────────────────────────────────────────

def test_arrival_error_uses_the_one_shared_formula():
    """The number the gate decides on and the number the operator reads
    (`catch_error_mm`) must be one computation, not two."""
    for _i, x, y, z in fx.SELF_TOSS_CAUGHT + fx.RELOAD_CAUGHT:
        v = _judge((x, y, z))
        assert v.arrival_err_mm == pytest.approx(
            lateral_miss_mm((x, y, z), CATCH_POINT))
        assert v.arrival_err_mm == pytest.approx(
            math.hypot(x - CATCH_POINT[0], y - CATCH_POINT[1]))


def test_plane_drop_sign_is_positive_when_the_estimate_coasted_below():
    """Free fall drives the estimate DOWN, so a positive `plane_drop_mm` is the
    normal reading and its magnitude is the extrapolation depth."""
    v = _judge((0.0, 0.0, CATCH_POINT[2] - 250.0))
    assert v.plane_drop_mm == pytest.approx(250.0)
    assert _judge((0.0, 0.0, CATCH_POINT[2] + 40.0)).plane_drop_mm == \
        pytest.approx(-40.0)


def test_verdict_carries_its_author():
    assert _judge((0.0, 0.0, CATCH_POINT[2])).source == SOURCE_TRACKER_ARRIVAL
    assert SOURCE_HAND_BALL_SENSOR != SOURCE_TRACKER_ARRIVAL


def test_describe_says_what_was_and_was_not_observed():
    """The operator scores the bench row against these lines, so both verdicts must
    name the retention state — a confirmed line that reads like a full catch
    confirmation is how the sensor's absence gets forgotten."""
    sev_ok, ok = describe(_judge((1.0, 1.0, CATCH_POINT[2] - 400.0)), TOL)
    assert sev_ok == 'info' and 'CONFIRMED' in ok and RETENTION_UNKNOWN in ok
    sev_no, no = describe(_judge(fx.CORRUPT_2026_07_23), TOL)
    assert sev_no == 'info' and 'REFUSED' in no
    assert 'dead-reckoned' in no                  # names the error model, not "corrupt"
    for line in (ok, no):
        assert 'ball_possession_contract.md' in line


def test_boundary_is_inclusive():
    """<= not <, so a ball exactly on the aperture is inside it."""
    assert _judge((TOL, 0.0, CATCH_POINT[2])).arrival_ok is True
    assert _judge((TOL + 1e-6, 0.0, CATCH_POINT[2])).arrival_ok is False


def test_reference_point_override_is_honoured():
    """The toss judges against its NOMINATED landing point, not the ACTIVE catch
    point — a displaced 8b catch at (0, -70) must not read as a 70 mm miss."""
    ref = (0.0, -70.0, CATCH_POINT[2])
    ball = (1.0, -71.0, CATCH_POINT[2] - 500.0)
    assert _judge(ball).arrival_ok is False            # against the default point
    assert _judge(ball, ref=ref).arrival_ok is True    # against the nominated one


# ── The bench instrument must keep telling the production story ───────────────
# `tools/probes/possession_verdict_bag_check.py` is what turns the next sitting's
# bag into runbook row POSS-1's verdict, and what regenerates the fixtures above.
# It mirrors two production values (the coordinator's tolerance and its catch
# point); a mirror that drifts scores a healthy capture wrong and nobody finds out
# until the operator has already aborted a good sitting. Guarded in CI, no bag
# needed — the probe imports `mcap_ros2` lazily, inside `read_caught_tracks`.

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_PROBE = os.path.join(_REPO, 'tools', 'probes', 'possession_verdict_bag_check.py')


@pytest.fixture(scope='module')
def probe():
    spec = importlib.util.spec_from_file_location(
        'possession_verdict_bag_check', _PROBE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_probe_self_check_passes(probe):
    """The probe's own eight-case, two-sided acceptance."""
    assert probe.self_check() == 0


def test_probe_self_check_catches_a_drifted_tolerance(probe, monkeypatch):
    """Mutation guard on the mirror. Without this, "the self-check passed" means
    only "the probe agrees with itself", and a re-tune of the node's bound would
    leave the bench instrument silently scoring the old one."""
    assert probe.self_check() == 0, 'baseline must be green before mutating'
    monkeypatch.setattr(probe, 'ARRIVAL_TOL_MM', 200.0)
    assert probe.self_check() == 1, (
        'a probe tolerance that no longer mirrors the coordinator must fail '
        'loudly, not re-score captures against a bound the robot does not use')


def test_probe_scores_the_fixture_populations_the_way_the_node_does(probe):
    """The probe and the node must not be two implementations of one rule."""
    scored = probe.score(
        [(i, 0.0, (x, y, z)) for i, x, y, z in fx.SELF_TOSS_CAUGHT],
        probe.CATCH_POINT_MM, probe.ARRIVAL_TOL_MM)
    assert all(v.confirmed for _i, _p, v in scored)
    scored = probe.score(
        [(i, 0.0, (x, y, z)) for i, x, y, z in fx.RELOAD_CAUGHT],
        probe.CATCH_POINT_MM, probe.ARRIVAL_TOL_MM)
    assert not any(v.confirmed for _i, _p, v in scored)
