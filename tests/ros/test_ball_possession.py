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
    ARRIVAL_CONFIRMED,
    ARRIVAL_REJECTED,
    ARRIVAL_UNKNOWN,
    EVIDENCE_EMPTY,
    EVIDENCE_SEATED,
    EVIDENCE_UNKNOWN,
    RETENTION_CONFIRMED,
    RETENTION_REJECTED,
    RETENTION_UNKNOWN,
    SOURCE_HAND_BALL_SENSOR,
    SOURCE_MERGED,
    SOURCE_TRACKER_ARRIVAL,
    HandBallSensorSource,
    PossessionVerdict,
    TrackerArrivalSource,
    describe,
    lateral_miss_mm,
    merge_possession,
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
    base = PossessionVerdict(SOURCE_TRACKER_ARRIVAL, ARRIVAL_CONFIRMED,
                             RETENTION_UNKNOWN, 1.0, 400.0, 'ARRIVAL_OK')
    assert base.confirmed is True
    assert base._replace(retention=RETENTION_CONFIRMED).confirmed is True
    assert base._replace(retention=RETENTION_REJECTED).confirmed is False
    # a failed arrival is never rescued by retention
    far = base._replace(arrival=ARRIVAL_REJECTED)
    assert far.confirmed is False
    assert far._replace(retention=RETENTION_CONFIRMED).confirmed is False
    # …and neither is an arrival nobody could observe
    blind = base._replace(arrival=ARRIVAL_UNKNOWN)
    assert blind.confirmed is False
    assert blind._replace(retention=RETENTION_CONFIRMED).confirmed is False


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


# ── The hand ball sensor (PRIMARY source, C-POSSESS-1 § 3.2) ──────────────────
#
# Timings here are the MEASURED 2026-08-10 ones, not round numbers: the catch
# band is +137…+798 ms after the predicted landing (n=35, both throwers, three
# bags — 2026-08-10_16-04-26 / _16-13-48 / _16-30-44), the next non-catch edge is
# +3194 ms, and the longest seat-then-leave the sensor resolved is 0.999 s.
# Mined by tools/probes/hand_sensor_verdict_replay.py; recipe and the reconciled
# transition counts in logbook/2026-08-10-sensor-truth-possession.md.

_LEAD_S = float(hw.JB_BD_ARRIVAL_LEAD_S)
_WINDOW_S = float(hw.JB_BD_ARRIVAL_WINDOW_S)
_RETAIN_S = float(hw.JB_BD_RETENTION_WINDOW_S)
_STALE_S = 0.5                       # the node passes _HAND_STATE_STALE_S


def _sensor(**kw):
    cfg = dict(arrival_lead_s=_LEAD_S, arrival_window_s=_WINDOW_S,
               retention_window_s=_RETAIN_S, stale_s=_STALE_S)
    cfg.update(kw)
    return HandBallSensorSource(**cfg)


def _stream(src, t0, t1, held, valid=True, dt=0.01):
    """Feed samples at 100 Hz (the /hand_telemetry rate) over [t0, t1).

    ``held`` may be a bool or a callable of t, so a test can place an edge at a
    measured instant instead of splicing two loops."""
    t = t0
    while t < t1 - 1e-9:
        h = held(t) if callable(held) else held
        src.note_sample(t, held=bool(h), valid=bool(valid))
        t += dt
    return t


def test_boot_is_unknown_not_empty_and_not_held():
    """THE safety property, and the one BallButler gets wrong.

    BallButler boots ``ball_in_hand_ = true`` — fail-OPEN — and this project
    recorded that as one of three of its properties deliberately not copied
    (plans/archived/hand-ball-sensor.md § Context). A source that has never had a
    reply knows nothing, and 'nothing' must project to False everywhere it is
    consumed: the evidence read refuses the throw, and the verdict refuses to
    confirm a catch."""
    src = _sensor()
    assert src.evidence(0.0) == EVIDENCE_UNKNOWN
    v = src.observe(0.0, landing_t=0.0)
    assert v.arrival == ARRIVAL_UNKNOWN
    assert v.arrival_ok is False
    assert v.retention == RETENTION_UNKNOWN
    assert v.confirmed is False


def test_held_is_meaningless_without_valid():
    """``ball_held`` means nothing unless ``ball_held_valid`` — normative in
    plans/archived/hand-ball-sensor.md § Architecture. A TRUE held bit on an
    invalid sample is exactly the stale value a dead ODrive republishes forever,
    which is the second BallButler property not copied."""
    src = _sensor()
    _stream(src, 0.0, 1.0, held=True, valid=False)
    assert src.evidence(1.0) == EVIDENCE_UNKNOWN


def test_evidence_tracks_the_cup_once_the_sensor_is_live():
    src = _sensor()
    _stream(src, 0.0, 1.0, held=False)
    assert src.evidence(1.0) == EVIDENCE_EMPTY
    _stream(src, 1.0, 2.0, held=True)
    assert src.evidence(2.0) == EVIDENCE_SEATED


def test_evidence_goes_unknown_when_the_node_stops_hearing_the_sensor():
    """Staleness is not a value, it is an absence. A cup that read SEATED 5 s ago
    is not evidence that a ball is in it now — that is the ``ball_seated``
    stale-belief failure C-POSSESS-1 § 3.3 replaced."""
    src = _sensor()
    _stream(src, 0.0, 1.0, held=True)
    assert src.evidence(1.0) == EVIDENCE_SEATED
    assert src.evidence(1.0 + _STALE_S + 0.01) == EVIDENCE_UNKNOWN


def test_a_measured_catch_confirms_arrival():
    """The median measured catch: the edge lands +399 ms after the predicted
    landing (2026-08-10 pooled, n=35)."""
    src = _sensor()
    land = 10.0
    _stream(src, 8.0, land + 0.399, held=False)
    _stream(src, land + 0.399, land + 1.0, held=True)
    v = src.observe(land + 1.0, landing_t=land)
    assert v.arrival == ARRIVAL_CONFIRMED
    assert v.confirmed is True
    assert src.arrival_time(land) == pytest.approx(land + 0.399, abs=0.011)


@pytest.mark.parametrize('delta_ms', [137, 269, 399, 798])
def test_the_whole_measured_catch_band_fits_inside_the_window(delta_ms):
    """Both ends of the 2026-08-10 measurement, plus two interior points. The
    +798 ms worst case is the one that sizes the window: it leaves 1.9x margin,
    and the window must ALSO absorb the can-bridge-uptime-dependent dispatch
    shift (+54–63 ms fresh, +118–133 ms at ~16 h), which is why it is not
    trimmed to the band."""
    src = _sensor()
    land = 10.0
    edge = land + delta_ms / 1000.0
    _stream(src, 8.0, edge, held=False)
    _stream(src, edge, edge + 0.5, held=True)
    assert src.observe(edge + 0.5, landing_t=land).arrival == ARRIVAL_CONFIRMED


def test_a_reseat_outside_the_window_is_not_a_catch():
    """The other side of the same measurement: the earliest non-catch edge in the
    three bags is +3194 ms, so a window that swallowed it would score an operator
    reloading the cup by hand as a successful catch and inflate every catch-rate
    number the tuning loop is built on."""
    src = _sensor()
    land = 10.0
    edge = land + 3.194
    _stream(src, 8.0, edge, held=False)
    _stream(src, edge, edge + 0.5, held=True)
    v = src.observe(edge + 0.5, landing_t=land)
    assert v.arrival == ARRIVAL_REJECTED
    assert v.reason == 'SENSOR_NO_ARRIVAL'


def test_the_window_is_unknown_while_it_is_still_open():
    """Before the window closes, 'no edge yet' is not 'it did not arrive' — the
    ball may still be in the air. Reporting REJECTED here would veto the
    tracker's own CAUGHT on a ball that lands 50 ms later."""
    src = _sensor()
    land = 10.0
    _stream(src, 8.0, land + 0.3, held=False)
    v = src.observe(land + 0.3, landing_t=land)
    assert v.arrival == ARRIVAL_UNKNOWN
    assert v.reason == 'SENSOR_WINDOW_OPEN'


def test_a_ball_that_never_left_raises_no_edge_and_so_confirms_nothing():
    """The arrival observable is an EDGE, not a level, and this is why. A throw
    whose stroke never fired leaves the ball sitting in the cup across the whole
    window; a level-based rule would read HELD and mint a catch for a ball that
    never flew. It also makes the negative window lead free: nothing can arrive
    early if nothing transitioned."""
    src = _sensor()
    land = 10.0
    _stream(src, 8.0, land + 2.0, held=True)
    v = src.observe(land + 2.0, landing_t=land)
    assert v.arrival == ARRIVAL_REJECTED
    assert src.evidence(land + 2.0) == EVIDENCE_SEATED      # …and the cup IS full


def test_no_landing_means_unknown_not_rejected():
    """With nothing in the air, "did it arrive?" has no referent. Answering
    REJECTED would let a source with no question in front of it veto the
    tracker."""
    src = _sensor()
    _stream(src, 0.0, 2.0, held=False)
    v = src.observe(2.0, landing_t=None)
    assert v.arrival == ARRIVAL_UNKNOWN
    assert v.reason == 'SENSOR_NO_LANDING'


def test_retention_confirms_only_after_the_window_has_actually_elapsed():
    src = _sensor()
    land = 10.0
    edge = land + 0.4
    _stream(src, 8.0, edge, held=False)
    _stream(src, edge, edge + _RETAIN_S + 0.2, held=True)
    assert src.observe(edge + _RETAIN_S - 0.2,
                       landing_t=land).retention == RETENTION_UNKNOWN
    assert src.observe(edge + _RETAIN_S + 0.1,
                       landing_t=land).retention == RETENTION_CONFIRMED


@pytest.mark.parametrize('hold_s', [0.571, 0.989, 0.999])
def test_the_three_measured_seat_then_leaves_read_as_bounce_outs(hold_s):
    """The bounce-out ground truth, and its honest provenance.

    These three durations are real: bag 2026-08-10_16-30-44, at t ≈ 292/298/299 s,
    with ``pos_meas ≈ 0`` (the hand parked at the bottom) inside a 54 s window
    containing no throw announcement — i.e. the OPERATOR hand-loading the cup, not
    a post-catch bounce-out. They are therefore ground truth for the property the
    retention window actually rests on — that the sensor RESOLVES a sub-second
    seat-then-leave — and are not evidence about how a real bounce-out behaves,
    of which the three bags contain none. Sizing 1.5 s at 1.5x the longest of
    them is the conservative reading of that."""
    src = _sensor()
    land = 10.0
    edge = land + 0.4
    _stream(src, 8.0, edge, held=False)
    _stream(src, edge, edge + hold_s, held=True)
    _stream(src, edge + hold_s, edge + hold_s + 0.3, held=False)
    v = src.observe(edge + hold_s + 0.3, landing_t=land)
    assert v.arrival == ARRIVAL_CONFIRMED       # it DID arrive…
    assert v.retention == RETENTION_REJECTED    # …and then it left
    assert v.confirmed is False


def test_a_legitimate_throw_can_never_read_as_a_bounce_out():
    """The upper constraint on the retention window is the machine's own cadence:
    toss_sequencer.MIN_TOSS_THROW_DELAY_S is 3.5 s, so the earliest a legitimate
    departure can follow a catch is 2.3x the window. A window sized above that
    floor would score every chained Toss → Toss as a bounce-out."""
    from jugglebot.toss_sequencer import MIN_TOSS_THROW_DELAY_S
    assert _RETAIN_S < MIN_TOSS_THROW_DELAY_S / 2.0
    src = _sensor()
    land = 10.0
    edge = land + 0.4
    _stream(src, 8.0, edge, held=False)
    _stream(src, edge, edge + MIN_TOSS_THROW_DELAY_S, held=True)
    _stream(src, edge + MIN_TOSS_THROW_DELAY_S,
            edge + MIN_TOSS_THROW_DELAY_S + 0.3, held=False)
    v = src.observe(edge + MIN_TOSS_THROW_DELAY_S + 0.3, landing_t=land)
    assert v.retention == RETENTION_CONFIRMED


def test_a_blind_window_is_unknown_even_though_the_cup_ends_up_full():
    """The subtle one, and the reason blindness is tracked as SPANS rather than a
    flag. Empty before the gap, held after it: physically a ball arrived, but the
    edge was never seen and therefore cannot be timed or placed inside the
    window. Synthesising an arrival from 'was empty, is now held' is how a blind
    window mints a catch — so the source re-seeds silently and reports UNKNOWN,
    which the merge then answers from the tracker."""
    src = _sensor()
    land = 10.0
    _stream(src, 8.0, land - 0.5, held=False)
    # …link dies across the whole arrival window…
    _stream(src, land + 2.0, land + 2.5, held=True)
    v = src.observe(land + 2.5, landing_t=land)
    assert v.arrival == ARRIVAL_UNKNOWN
    assert v.reason == 'SENSOR_BLIND'
    assert src.evidence(land + 2.5) == EVIDENCE_SEATED    # live read still works


def test_a_recovered_link_does_not_manufacture_an_edge():
    """Same mechanism, stated as the invariant a future edit could break: no
    sample that arrives across a blind boundary may append an edge."""
    src = _sensor()
    _stream(src, 0.0, 1.0, held=False)
    _stream(src, 5.0, 5.5, held=True)          # 4 s gap, then a different level
    assert src.observe(5.5, landing_t=5.0).arrival == ARRIVAL_UNKNOWN


# ── The merge (C-POSSESS-1 § 3.2) ─────────────────────────────────────────────

def _tracker_says(ok):
    xy = 1.0 if ok else 500.0
    return _judge((xy, 0.0, CATCH_POINT[2] - 400.0))


def _sensor_says(arrival, retention=RETENTION_UNKNOWN):
    return PossessionVerdict(SOURCE_HAND_BALL_SENSOR, arrival, retention,
                             float('nan'), float('nan'), 'SENSOR_TEST')


def test_merge_lets_a_valid_sensor_veto_a_tracker_caught():
    """The false-CAUGHT class § 7 sized and ACCEPTED is now closed, and this is
    the assertion that closes it: the tracker's estimate is a dead-reckoned
    free-fall extrapolation, the sensor reads the cup, and when they disagree the
    cup wins."""
    m = merge_possession(sensor=_sensor_says(ARRIVAL_REJECTED),
                         tracker=_tracker_says(True))
    assert m.arrival == ARRIVAL_REJECTED
    assert m.confirmed is False
    assert m.source == SOURCE_MERGED


def test_merge_lets_a_valid_sensor_confirm_what_the_tracker_refuses():
    """The direction that matters most for the RELOAD path: every
    destination-tagged reload track in the reference capture is a split track
    204.9–752.9 mm out, so the tracker refuses a catch the operator watched land.
    The sensor does not care where the tracker thinks the ball was."""
    m = merge_possession(sensor=_sensor_says(ARRIVAL_CONFIRMED),
                         tracker=_tracker_says(False))
    assert m.arrival == ARRIVAL_CONFIRMED
    assert m.confirmed is True


def test_merge_falls_back_to_the_tracker_when_the_sensor_is_blind():
    """The degradation path is exactly the pre-sensor behaviour, in BOTH
    directions. A blind sensor that refused everything would leave the machine
    strictly less capable than it was before the sensor landed."""
    for tracker_ok in (True, False):
        m = merge_possession(sensor=_sensor_says(ARRIVAL_UNKNOWN),
                             tracker=_tracker_says(tracker_ok))
        assert m.arrival_ok is tracker_ok
        assert m.source == SOURCE_TRACKER_ARRIVAL     # names its real author


def test_merge_takes_retention_only_from_the_sensor():
    """The tracker is contract-forbidden from claiming retention (§ 2 consequence
    2) — it has no post-arrival observation to offer — so there is nothing to
    merge and a REJECTED retention must survive intact."""
    m = merge_possession(
        sensor=_sensor_says(ARRIVAL_CONFIRMED, RETENTION_REJECTED),
        tracker=_tracker_says(True))
    assert m.retention == RETENTION_REJECTED
    assert m.confirmed is False


def test_merge_keeps_the_trackers_accuracy_numbers():
    """§ 3's "the one thing a new source must not forget": arrival_err_mm is the
    catch-accuracy number the hardware runbooks score, the sensor cannot produce
    it, and a merged verdict that reported the sensor's NaN would silently empty
    every bench row that reads it."""
    tracker = _tracker_says(True)
    m = merge_possession(sensor=_sensor_says(ARRIVAL_CONFIRMED), tracker=tracker)
    assert m.arrival_err_mm == pytest.approx(tracker.arrival_err_mm)
    assert m.plane_drop_mm == pytest.approx(tracker.plane_drop_mm)


def test_arrival_ok_is_a_projection_not_a_field():
    """C-POSSESS-1.A's enforcement point. If `arrival_ok` were ever re-introduced
    as a settable field, a source could report `arrival=UNKNOWN, arrival_ok=True`
    — a claim minted out of blindness, which is the original defect's exact
    shape. A property makes that unrepresentable."""
    with pytest.raises(TypeError):
        PossessionVerdict(SOURCE_TRACKER_ARRIVAL, ARRIVAL_UNKNOWN,
                          RETENTION_UNKNOWN, 0.0, 0.0, 'R', arrival_ok=True)
    for state, expect in ((ARRIVAL_CONFIRMED, True), (ARRIVAL_REJECTED, False),
                          (ARRIVAL_UNKNOWN, False)):
        v = PossessionVerdict(SOURCE_TRACKER_ARRIVAL, state, RETENTION_UNKNOWN,
                              0.0, 0.0, 'R')
        assert v.arrival_ok is expect


def test_describe_distinguishes_unknown_from_refused():
    """An operator who reads REFUSED goes hunting for a miss; one who reads
    UNKNOWN goes to the sensor. Collapsing them costs a sitting.

    The UNKNOWN branch is DEFENSIVE at today's call site and that is stated
    rather than left to be discovered: `_possession_confirmed` only runs on a
    tracker CAUGHT, the tracker always has an estimate in hand, and the merge
    therefore never yields an UNKNOWN arrival while it is the corroborator. It
    exists because the first tick-driven consumer that calls `observe` without a
    tracker verdict WILL hit it, and a `describe` that rendered blindness as
    "REFUSED … 0 mm from the catch point" would be actively misleading."""
    unknown_verdict = PossessionVerdict(
        SOURCE_HAND_BALL_SENSOR, ARRIVAL_UNKNOWN, RETENTION_UNKNOWN,
        float('nan'), float('nan'), 'SENSOR_BLIND')
    _sev, unknown = describe(unknown_verdict, TOL)
    assert 'UNKNOWN' in unknown and 'REFUSED' not in unknown
    _sev, refused = describe(
        merge_possession(sensor=_sensor_says(ARRIVAL_REJECTED),
                         tracker=_tracker_says(True)), TOL)
    assert 'REFUSED' in refused
    for line in (unknown, refused):
        assert 'Not counted' in line
        assert 'ball_possession_contract.md' in line


def test_the_merged_log_line_names_the_sensor_state_that_produced_it():
    """The operator-facing consequence of the fallback rule. When the sensor is
    blind the merged verdict reads exactly like a tracker-only one — same author,
    same wording — so the ONLY thing distinguishing "the tracker refused" from
    "the sensor could not look and the tracker refused" is the reason string.
    It therefore has to be in the line."""
    blind = merge_possession(
        sensor=PossessionVerdict(SOURCE_HAND_BALL_SENSOR, ARRIVAL_UNKNOWN,
                                 RETENTION_UNKNOWN, float('nan'), float('nan'),
                                 'SENSOR_BLIND'),
        tracker=_tracker_says(False))
    _sev, line = describe(blind, TOL)
    assert 'SENSOR_BLIND' in line
    seen = merge_possession(sensor=_sensor_says(ARRIVAL_CONFIRMED),
                            tracker=_tracker_says(True))
    _sev, ok_line = describe(seen, TOL)
    assert 'SENSOR_TEST' in ok_line and 'CONFIRMED' in ok_line


def test_describe_never_prints_a_contradictory_cross_check():
    """The reporting wart C-POSSESS-1 § 3 warned would "survive for a year because
    nothing fails", now reachable.

    `describe` is handed the TRACKER's tolerance. Once the sensor can mint a
    CONFIRMED the tracker refuses — which is the normal reading on a split track,
    and the whole reason the sensor is primary — the old wording produced
    "possession CONFIRMED … arrival 500 mm <= 70 mm", i.e. a line that reads like
    a broken gate on a working one. The line must name the disagreement instead."""
    disagreeing = merge_possession(sensor=_sensor_says(ARRIVAL_CONFIRMED),
                                   tracker=_tracker_says(False))
    _sev, line = describe(disagreeing, TOL)
    assert 'CONFIRMED' in line and 'DISAGREES' in line
    assert '<=' not in line
    agreeing = merge_possession(sensor=_sensor_says(ARRIVAL_CONFIRMED),
                                tracker=_tracker_says(True))
    _sev, line2 = describe(agreeing, TOL)
    assert 'agrees' in line2
    # A sensor-only verdict has no tracker number at all — say so, do not print NaN.
    _sev, line3 = describe(_sensor_says(ARRIVAL_CONFIRMED), TOL)
    assert 'unavailable' in line3 and 'nan' not in line3.lower()


def test_describe_attributes_a_sensor_veto_to_the_sensor():
    """The MIRROR of the wart above, on the path this phase was BUILT to enable
    (audit 2026-08-10). § 3.2 rule 2 lets a valid sensor REJECTED veto a tracker
    CAUGHT — the headline capability — and that is the commonest REFUSED on a
    self-toss the tracker still likes. The old REFUSED branch printed the
    TRACKER's error against the TRACKER's tolerance unconditionally, so a sensor
    veto rendered as "arrival <small> mm > 70 mm from the catch point" (the audit
    reproduced it live at 3 mm; this test drives 1 mm): a line that is
    arithmetically false and routes the operator to the wrong subsystem, on the
    row (runbook POSS-1) they score the sitting with."""
    veto = merge_possession(sensor=_sensor_says(ARRIVAL_REJECTED),
                            tracker=_tracker_says(True))     # tracker err 1.0 mm
    _sev, line = describe(veto, TOL)
    assert 'REFUSED' in line and 'cup sensor' in line
    assert 'DISAGREES' in line                  # the tracker WOULD have confirmed
    assert '> 70 mm' not in line                # the wart itself
    assert 'Not counted' in line
    # The tracker-authored refusal keeps its own wording — the number IS the
    # reason there, and the plane-drop note is what makes it REPORT-only.
    _sev, tracker_line = describe(_tracker_says(False), TOL)
    assert 'REFUSED' in tracker_line and '> 70 mm' in tracker_line
    assert 'cup sensor' not in tracker_line
    # Sensor and tracker agreeing on a refusal must not read as a disagreement.
    both = merge_possession(sensor=_sensor_says(ARRIVAL_REJECTED),
                            tracker=_tracker_says(False))
    _sev, both_line = describe(both, TOL)
    assert 'agrees' in both_line and 'DISAGREES' not in both_line
