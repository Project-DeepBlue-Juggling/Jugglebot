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

import jugglebot.ball_possession as bp
import jugglebot.hardware_config as hw
from jugglebot.ball_possession import (
    ARRIVAL_BAND_MAX_S,
    ARRIVAL_BAND_MIN_S,
    ARRIVAL_CONFIRMED,
    ARRIVAL_REJECTED,
    ARRIVAL_UNKNOWN,
    EVIDENCE_EMPTY,
    EVIDENCE_SEATED,
    EVIDENCE_UNKNOWN,
    RELEASE_GUARD_S,
    RETENTION_CONFIRMED,
    RETENTION_REJECTED,
    RETENTION_UNKNOWN,
    SOURCE_HAND_BALL_SENSOR,
    SOURCE_MERGED,
    SOURCE_TRACKER_ARRIVAL,
    HandBallSensorSource,
    PossessionVerdict,
    SensorWindows,
    TrackerArrivalSource,
    arrival_blind,
    arrival_boundary_t,
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


_UNSET = object()      # "the caller passed no raw bit at all" — see _stream


def _sensor(**kw):
    cfg = dict(arrival_lead_s=_LEAD_S, arrival_window_s=_WINDOW_S,
               retention_window_s=_RETAIN_S, stale_s=_STALE_S)
    cfg.update(kw)
    return HandBallSensorSource(**cfg)


def _stream(src, t0, t1, held, valid=True, dt=0.01, raw=_UNSET):
    """Feed samples at 100 Hz (the /hand_telemetry rate) over [t0, t1).

    ``held`` may be a bool or a callable of t, so a test can place an edge at a
    measured instant instead of splicing two loops. ``raw`` is the same shape and
    defaults to *omitted entirely* — i.e. the pre-2026-08-21 two-argument feed,
    which the source degrades to the debounced bit. A test that cares about the
    debounce asymmetry passes both."""
    t = t0
    while t < t1 - 1e-9:
        h = held(t) if callable(held) else held
        if raw is _UNSET:
            src.note_sample(t, held=bool(h), valid=bool(valid))
        else:
            r = raw(t) if callable(raw) else raw
            src.note_sample(t, held=bool(h), valid=bool(valid), raw=bool(r))
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


def test_a_legitimate_throw_at_the_SHIPPED_cadence_is_not_a_bounce_out():
    """A slow-cadence departure is inside the unclamped retention window's own
    reach and must read CONFIRMED.

    ⚠ THIS TEST'S PREMISE CHANGED ON 2026-08-22. It used to argue from
    ``MIN_TOSS_THROW_DELAY_S = 3.5 s`` — "the earliest a legitimate departure can
    follow a catch is 2.3x the window" — and that constant is RETIRED (census
    A1). The window is no longer safe by cadence arithmetic; it is safe because
    ``observe`` CLAMPS its horizon to the announced next release
    (C-POSSESS-1 § 3.4, census D1), which is what
    ``test_a_legitimate_throw_is_not_a_bounce_out_at_a_short_dwell`` pins.

    What survives here is the SLOW case, and it is worth keeping separately: at
    the shipped 6.0 s dwell the departure is far outside the window, so the
    clamp is inert and the raw window has to get the answer right on its own. A
    regression that broke the unclamped path would otherwise hide behind the
    clamp at every cadence the ladder actually runs.

    ⚠ R4 (2026-09-24, U6b Cluster C): ``hw.JB_OP_TOSS_SESSION_DWELL_DEFAULT_S``
    (the TossContinuous session dwell default) is deleted with the FSM under
    `fsm-final` — the skill stack schedules on absolute wall-clock, not a
    session dwell. The 6.0 s figure is kept as a plain literal: the claim this
    test pins is "a departure far outside the retention window still reads
    CONFIRMED", and 6.0 s is simply a concrete instance of "far outside" (the
    window itself, ``_RETAIN_S``, is unchanged and asserted below)."""
    departure_after_catch_s = 6.0  # historical: the retired toss_session_dwell_default_s shipped value
    assert _RETAIN_S < departure_after_catch_s
    src = _sensor()
    land = 10.0
    edge = land + 0.4
    _stream(src, 8.0, edge, held=False)
    _stream(src, edge, edge + departure_after_catch_s, held=True)
    _stream(src, edge + departure_after_catch_s,
            edge + departure_after_catch_s + 0.3, held=False)
    v = src.observe(edge + departure_after_catch_s + 0.3, landing_t=land)
    assert v.retention == RETENTION_CONFIRMED


def test_a_blind_window_is_unknown_even_though_the_cup_ends_up_full():
    """The subtle one, and the reason blindness is tracked as SPANS rather than a
    flag. Empty before the gap, held after it: physically a ball arrived, but the
    edge was never seen and therefore cannot be timed or placed inside the
    window. Synthesising an arrival from 'was empty, is now held' is how a blind
    window mints a catch — so the source re-seeds silently and reports UNKNOWN.
    Since D1 (2026-08-26) that UNKNOWN REFUSES; there is no tracker fallback, and
    the cycle terminal is MISSED_SENSOR_BLIND."""
    src = _sensor()
    land = 10.0
    _stream(src, 8.0, land - 0.5, held=False)
    # …link dies across the whole arrival window…
    _stream(src, land + 2.0, land + 2.5, held=True)
    v = src.observe(land + 2.5, landing_t=land)
    assert v.arrival == ARRIVAL_UNKNOWN
    assert v.reason == 'SENSOR_BLIND'
    assert src.evidence(land + 2.5) == EVIDENCE_SEATED    # live read still works


def test_a_feed_that_simply_STOPS_reads_blind_and_not_missed():
    """**The dead-cup mode, and it was the one blindness could not see** (audit
    finding B1, 2026-08-26).

    A blind SPAN is only recorded when the next sample lands and closes it, so a
    feed that stops mid-flight and never comes back left ``_blind_between``
    answering False for ever. The window then closed empty, the source answered
    ``SENSOR_NO_ARRIVAL``, and the FSM minted a plain ``MISSED`` — bit-identical
    to a genuine miss, on a machine whose cup had been dead since before the ball
    was in the air. D1 forbids exactly that outcome: a source that could not look
    must refuse and say so, and ``MISSED_SENSOR_BLIND`` is the saying-so.

    The liveness term closes it: anything past ``_last_t + stale_s`` is unwatched
    whether or not a later sample ever proves it. Note this is the SAME
    arithmetic ``evidence`` has always used (``_live_ok``), so the two halves of
    the source now agree about when it has gone dark.

    PORTED (R4, 2026-09-24, U6b Cluster A follow-up): the deadline used to be
    imported as ``toss_sequencer.CATCH_CONFIRM_WINDOW_S`` (an alias of
    ``ARRIVAL_BAND_MAX_S``, deleted with the FSM); read directly off the
    surviving module instead — this test drives only ``HandBallSensorSource``/
    ``merge_possession``/``arrival_blind``, never the FSM."""
    src = _sensor()
    land = 10.0
    # A healthy 100 Hz feed, empty cup, up to 0.2 s before the landing…
    _stream(src, 8.0, land - 0.2, held=False)
    # …and then nothing. No later sample ever arrives to close the span.
    deadline = land + ARRIVAL_BAND_MAX_S
    v = src.observe(deadline, landing_t=land)
    assert v.arrival == ARRIVAL_UNKNOWN
    assert v.reason == 'SENSOR_BLIND'
    assert arrival_blind(merge_possession(sensor=v)) is True
    # `evidence` already answered UNKNOWN here — the two halves now agree.
    assert src.evidence(deadline) == EVIDENCE_UNKNOWN
    # NON-REGRESSION, and it is the whole reason the term is written against
    # `_last_t + stale_s` rather than against `now`: a healthy 100 Hz feed running
    # right through the same deadline is NOT blind. At the FSM's deadline its
    # window is still open (SENSOR_WINDOW_OPEN — the FSM terminalises on its own
    # deadline, which is what makes "still looking" a real MISS there), and when
    # the source's own window does close it mints a POSITIVE refusal.
    live = _sensor()
    _stream(live, 8.0, deadline + 0.01, held=False)
    lv = live.observe(deadline, landing_t=land)
    assert lv.arrival == ARRIVAL_UNKNOWN
    assert lv.reason == 'SENSOR_WINDOW_OPEN'
    assert arrival_blind(merge_possession(sensor=lv)) is False
    closed = land + _WINDOW_S + 0.01
    _stream(live, deadline + 0.01, closed + 0.01, held=False)
    lc = live.observe(closed, landing_t=land)
    assert lc.arrival == ARRIVAL_REJECTED
    assert lc.reason == 'SENSOR_NO_ARRIVAL'
    assert arrival_blind(merge_possession(sensor=lc)) is False


#: DELETED (R4, 2026-09-24, U6b Cluster A follow-up):
#: ``test_the_fsm_mints_MISSED_SENSOR_BLIND_for_a_feed_that_stopped``. It
#: constructed a real ``TossSequencer`` and drove ``_step_settling`` directly
#: to check the terminal NAME (``MISSED_SENSOR_BLIND`` vs plain ``MISSED``)
#: the deleted FSM minted for a blind vs. a genuine miss — FSM structure, not
#: a claim about ``ball_possession`` (the unit assertion in the test above,
#: which stays, already proves ``arrival_blind`` itself). No live analogue
#: exists to port to: `motion/skills/executor.py`'s outcome capture tracks
#: only a boolean `caught_seen` (INVARIANTS.md `ABORTED_NO_RELEASE` /
#: C-POSSESS-1 rows) and mints no blind-vs-miss distinction at the terminal —
#: flagged in the U6b handoff as a possible coverage gap for a future unit,
#: not invented here.


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
    cup wins.

    Three of these fired for real on bag 2026-08-26_14-25-16 — a tracker CAUGHT
    over an empty cup, one of which drove a phantom reload."""
    m = merge_possession(sensor=_sensor_says(ARRIVAL_REJECTED),
                         tracker=_tracker_says(True))
    assert m.arrival == ARRIVAL_REJECTED
    assert m.confirmed is False
    # The CUP is the author, not a composite: since D1 (2026-08-26) the tracker
    # contributes report fields and nothing else.
    assert m.source == SOURCE_HAND_BALL_SENSOR


def test_merge_lets_a_valid_sensor_confirm_what_the_tracker_refuses():
    """The direction that matters most for the RELOAD path: every
    destination-tagged reload track in the reference capture is a split track
    204.9–752.9 mm out, so the tracker refuses a catch the operator watched land.
    The sensor does not care where the tracker thinks the ball was."""
    m = merge_possession(sensor=_sensor_says(ARRIVAL_CONFIRMED),
                         tracker=_tracker_says(False))
    assert m.arrival == ARRIVAL_CONFIRMED
    assert m.confirmed is True


def test_merge_never_falls_back_to_the_tracker(): # D1, 2026-08-26
    """THE D1 assertion. A blind sensor REFUSES; it does not hand the question to
    the tracker, in either direction.

    This test used to assert the opposite (`..._falls_back_to_the_tracker_when_
    the_sensor_is_blind`), on the argument that a blind sensor which refused
    everything would leave the machine strictly less capable than before the
    sensor landed. Bag 2026-08-26_14-25-16 is the counter-evidence: the fallback
    is a fallback to the LESS reliable observable, keyed on the more reliable one
    being quiet, and on that sitting the tracker scored 11/16 against the cup's
    23/4. Capability is not the metric — being right is.

    The blind case is not silently swallowed either: `arrival_blind` separates it
    from an ordinary still-open window, and both FSMs mint MISSED_SENSOR_BLIND on
    it rather than a plain MISSED."""
    for tracker_ok in (True, False):
        sensor = _sensor_says(ARRIVAL_UNKNOWN)
        m = merge_possession(sensor=sensor, tracker=_tracker_says(tracker_ok))
        assert m.arrival == ARRIVAL_UNKNOWN
        assert m.arrival_ok is False
        assert m.confirmed is False
        assert m.source == SOURCE_HAND_BALL_SENSOR


def test_merge_takes_no_tracker_at_all():
    """The shape change D1 needed: with no CAUGHT estimate in hand there is no
    ball to judge, so the question can be asked on a TICK. Report fields go NaN,
    which is the honest reading and is what a catch the tracker never saw now
    records."""
    m = merge_possession(sensor=_sensor_says(ARRIVAL_CONFIRMED))
    assert m.arrival == ARRIVAL_CONFIRMED
    assert m.confirmed is True
    assert m.source == SOURCE_HAND_BALL_SENSOR
    assert m.arrival_err_mm != m.arrival_err_mm        # NaN
    assert m.plane_drop_mm != m.plane_drop_mm
    assert m.reason == 'SENSOR_TEST'                   # not joined with a '/'


def test_arrival_blind_separates_could_not_look_from_still_looking():
    """The distinction D1 made load-bearing. Both are ARRIVAL_UNKNOWN and both
    refuse, but one is a machine fault to name and the other is the ordinary
    reading of every tick before the ball seats."""
    blind = merge_possession(
        sensor=PossessionVerdict(SOURCE_HAND_BALL_SENSOR, ARRIVAL_UNKNOWN,
                                 RETENTION_UNKNOWN, float('nan'), float('nan'),
                                 'SENSOR_BLIND'))
    waiting = merge_possession(
        sensor=PossessionVerdict(SOURCE_HAND_BALL_SENSOR, ARRIVAL_UNKNOWN,
                                 RETENTION_UNKNOWN, float('nan'), float('nan'),
                                 'SENSOR_WINDOW_OPEN'))
    assert arrival_blind(blind) is True
    assert arrival_blind(waiting) is False
    # It survives the '/'-joined reason a tracker-carrying merge produces, and it
    # is False for every non-UNKNOWN arrival.
    assert arrival_blind(merge_possession(
        sensor=PossessionVerdict(SOURCE_HAND_BALL_SENSOR, ARRIVAL_UNKNOWN,
                                 RETENTION_UNKNOWN, float('nan'), float('nan'),
                                 'SENSOR_BLIND'),
        tracker=_tracker_says(True))) is True
    assert arrival_blind(merge_possession(
        sensor=_sensor_says(ARRIVAL_REJECTED))) is False


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

    The UNKNOWN branch was DEFENSIVE until 2026-08-26 — `_possession_observed`
    only ran on a tracker CAUGHT, the tracker always had an estimate in hand, and
    the merge fell back to it, so an UNKNOWN arrival could not be minted. D1
    deleted the fallback and made the question tick-driven, so this is now the
    COMMON shape: it is what every tick before the ball seats produces. A
    `describe` that rendered blindness as "REFUSED … 0 mm from the catch point"
    would be actively misleading on most lines the operator sees."""
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
    """The reason string has to reach the operator-facing line.

    It mattered under the fallback rule because a blind-sensor verdict read
    exactly like a tracker-only one. It matters MORE since D1 (2026-08-26): the
    verdict is the cup's in every state, so the reason is the only thing in the
    line that says WHICH of the two UNKNOWNs this is — "could not look" (a
    machine fault) or "still looking" (every tick before the ball seats)."""
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


# ── The cadence clamp (C-POSSESS-1 § 3.4 / § 3.5, census D1/D2/D3) ────────────
#
# Every instant below is the machine's own schedule at a named rung of the
# cadence ladder, never a round number chosen to make an assertion pass:
#
#   R3   dwell 1.50 s, flight 0.80 s   — the rung the census orders these fixes
#                                        to land BEFORE
#   R5'  dwell 0.49 s, flight 0.4949 s — the TIGHTEST cadence any published rung
#                                        ever named, deliberately kept after the
#                                        rung itself moved (see below)
#
# and the seat edge is placed at +0.30 s, inside the measured +137…+798 ms band.
#
# ⚠ THESE ARE NO LONGER THE R5-PRIME RUNG'S NUMBERS, and that is deliberate
# (audit fix, 2026-08-22). The ladder republished R5-prime as dwell 0.63 s /
# flight 0.5029 s (level) and 1.01 / 0.5029 (aimed) once the pre-dispatch
# sequence cost was measured, and 0.49 / 0.4949 is not merely a different rung —
# `toss_session` will not ACCEPT it at any throw_delay, because the delay floor
# at that flight (0.3345 s) plus the handoff margin (0.1933 s) already exceeds
# 0.49 s. The pair is retained anyway because these tests pin the D1/D2 CLAMPS,
# and a clamp is period-dependent: holding them at a period 15 % shorter than
# anything the machine will schedule makes every assertion here a strict
# superset of the real case. What was fixed is the LABEL — this block used to
# call 0.49/0.4949 "the operating target", which stopped being true, and a
# future reader comparing it against the ladder would have found two different
# numbers under one name.
_R3_DWELL_S, _R3_FLIGHT_S = 1.50, 0.80
_R5P_DWELL_S, _R5P_FLIGHT_S = 0.49, 0.4949
_SEAT_DT_S = 0.30                      # inside the measured catch band


def _cycle(landing, dwell, flight):
    """-> (next_release_t, next_landing_t) for a cycle landing at ``landing``.

    The session's own arithmetic: ``release(N+1) = landing(N) + dwell`` (the
    dwell is defined previous-SCHEDULED-LANDING -> next RELEASE, see
    ``toss_session``'s module docstring), and the landing follows one flight
    later."""
    rel = landing + dwell
    return rel, rel + flight


def test_a_legitimate_throw_is_not_a_bounce_out_at_a_short_dwell():
    """CENSUS D1, the inversion this clamp exists to kill.

    The retention window is 1.50 s and was justified at the top by
    ``MIN_TOSS_THROW_DELAY_S`` being 3.5 s — "so a legitimate throw can never
    read as a bounce-out". That floor is RETIRED (census A1, landed 2026-08-22:
    the name is gone from the tree). At the R3 dwell the ball leaves
    the cup 1.50 s after the landing for OUR OWN next throw, which is INSIDE the
    unclamped window, so the source returns RETENTION_REJECTED on a perfect
    cycle. With ``on_empty_cup: RELOAD`` that route asks BallButler to throw a
    second ball at a full cup, so this is a safety fault wearing a label's
    clothes.

    Both halves are asserted: the defect reproduces without the clamp, and the
    clamp closes it. A test that only asserted the fixed behaviour would pass
    against an implementation that had simply widened something."""
    land = 10.0
    rise = land + _SEAT_DT_S
    rel, next_land = _cycle(land, _R3_DWELL_S, _R3_FLIGHT_S)
    src = _sensor()
    _stream(src, land - 1.0, rel + 0.5, held=lambda t: rise <= t < rel)
    now = rel + 0.5
    # UNCLAMPED — the shipped behaviour, and the defect.
    assert src.observe(now, landing_t=land).retention == RETENTION_REJECTED
    # CLAMPED — the departure at `rel` is at/after `rel - RELEASE_GUARD_S`, so it
    # is OUR throw and is excluded from the bounce test by construction.
    v = src.observe(now, landing_t=land,
                    next_release_t=rel, next_landing_t=next_land)
    assert v.retention == RETENTION_CONFIRMED
    assert v.confirmed is True


def test_a_real_bounce_out_still_reads_rejected_under_the_clamp():
    """The clamp excludes the announced throw window, NOT everything after the
    arrival. A clamp that swallowed real bounce-outs would trade one mislabel for
    its mirror image, and the bounce-out trap is what C-POSSESS-1 § 7 spent a
    whole section accepting before the sensor closed it."""
    land = 10.0
    rise = land + _SEAT_DT_S
    drop = rise + 0.20                 # well inside the horizon, and not our throw
    rel, next_land = _cycle(land, _R3_DWELL_S, _R3_FLIGHT_S)
    src = _sensor()
    _stream(src, land - 1.0, rel + 0.5, held=lambda t: rise <= t < drop)
    v = src.observe(rel + 0.5, landing_t=land,
                    next_release_t=rel, next_landing_t=next_land)
    assert v.retention == RETENTION_REJECTED
    assert v.confirmed is False


def test_retention_is_unknown_not_confirmed_when_the_cadence_leaves_no_horizon():
    """C-POSSESS-1 § 3.4's third clause, and the honest half of this change.

    At the R5' operating point the seat edge lands +0.30 s after the landing and
    the next release only +0.49 s after it, so `rise` is already PAST
    `next_release - RELEASE_GUARD_S`. There is no interval in which a bounce-out
    could be observed at all — the physics removes it, not the clamp (the
    debounced fall lag alone is ~241 ms).

    The answer must be UNKNOWN. CONFIRMED would claim an observation never made;
    REJECTED is the D1 inversion. UNKNOWN does not veto (§ 2 consequence 3), so
    the catch still confirms — possession is simply ARRIVAL-only again at this
    cadence, which § 3.4 states plainly and routes to the next cycle's live
    evidence read and the catch-outcome penalty loop."""
    land = 10.0
    rise = land + _SEAT_DT_S
    rel, next_land = _cycle(land, _R5P_DWELL_S, _R5P_FLIGHT_S)
    assert rise > rel - 0.30, 'premise: the seat edge is past the horizon'
    src = _sensor()
    _stream(src, land - 1.0, rel + 0.5, held=lambda t: rise <= t < rel)
    v = src.observe(rel + 0.5, landing_t=land,
                    next_release_t=rel, next_landing_t=next_land)
    assert v.arrival == ARRIVAL_CONFIRMED
    assert v.retention == RETENTION_UNKNOWN
    assert v.confirmed is True


def test_adjacent_arrival_windows_abut_and_never_overlap():
    """CENSUS D2. The arrival window is 1.50 s and was justified by a ~4x
    separation between the catch band (+137…+798 ms) and the next non-catch rise
    (+3194 ms). That premise dies once the CYCLE PERIOD drops below the window:
    at R5' the period is ``dwell + T = 0.985 s``, so the NEXT cycle's landing —
    and its seat edge at +0.30 s — fall INSIDE this cycle's unclamped window and
    can be claimed twice. (The rung is R5' and not R3 for a measurable reason:
    R3's period is 2.30 s, still 1.5x clear of the window, so the overlap is not
    yet reachable there.)

    Clamped, this window closes exactly where the next one opens
    (`next_landing - arrival_lead_s`), so the two abut: no edge belongs to both,
    and none falls between them."""
    land = 10.0
    rel, next_land = _cycle(land, _R5P_DWELL_S, _R5P_FLIGHT_S)
    assert next_land - land < _WINDOW_S, 'premise: the periods overlap at R5-prime'
    src = _sensor()
    # ONLY the next cycle's seat edge exists — this cycle's ball never arrived.
    next_rise = next_land + _SEAT_DT_S
    _stream(src, land - 1.0, next_rise + 0.5, held=lambda t: t >= next_rise)
    now = next_rise + 0.5
    # UNCLAMPED the neighbour's edge is stolen and this cycle reads CONFIRMED.
    assert src.observe(now, landing_t=land).arrival == ARRIVAL_CONFIRMED
    # CLAMPED it is out of reach, and a window that closed with no rise in it is
    # a positive observation of non-arrival.
    v = src.observe(now, landing_t=land,
                    next_release_t=rel, next_landing_t=next_land)
    assert v.arrival == ARRIVAL_REJECTED
    # ... and the NEXT cycle owns that same edge, from its own landing.
    assert src.observe(now, landing_t=next_land).arrival == ARRIVAL_CONFIRMED


def test_the_clamps_are_off_by_default_so_a_single_toss_is_unchanged():
    """A single ``Toss`` — and a session's last cycle — have nothing scheduled
    after them, so the honest horizon is the shipped fixed one. None and NaN must
    behave identically: a NaN horizon compares False against everything and would
    silently disable the clamp it was meant to apply."""
    land = 10.0
    rise = land + _SEAT_DT_S
    src = _sensor()
    _stream(src, land - 1.0, rise + _RETAIN_S + 0.2, held=lambda t: t >= rise)
    now = rise + _RETAIN_S + 0.1
    base = src.observe(now, landing_t=land)
    assert base.retention == RETENTION_CONFIRMED
    nan = float('nan')
    for v in (src.observe(now, landing_t=land, next_release_t=nan,
                          next_landing_t=nan),
              src.observe(now, landing_t=land, next_release_t=None,
                          next_landing_t=None)):
        # Field-wise, not tuple-wise: this source reports arrival_err_mm and
        # plane_drop_mm as NaN (it measures the cup, not a position) and NaN is
        # not equal to itself.
        assert (v.arrival, v.retention, v.reason) == (base.arrival,
                                                      base.retention,
                                                      base.reason)


def test_arrival_time_reads_the_same_edge_the_verdict_does():
    """``arrival_time`` feeds the record's ``catch_event_dt_s`` and ``observe``
    feeds the verdict. If only one of them were clamped they would report
    different edges for one catch — the corpus would carry a catch-event time for
    an arrival the verdict says never happened. R5' for the same reason as the
    test above: it is the rung where the windows actually overlap."""
    land = 10.0
    rel, next_land = _cycle(land, _R5P_DWELL_S, _R5P_FLIGHT_S)
    next_rise = next_land + _SEAT_DT_S
    src = _sensor()
    _stream(src, land - 1.0, next_rise + 0.5, held=lambda t: t >= next_rise)
    assert src.arrival_time(land) == pytest.approx(next_rise, abs=0.011)
    assert math.isnan(src.arrival_time(land, next_landing_t=next_land))


# ── The arrival BOUNDARY (C-POSSESS-1 § 3.4 clauses C.1 / C.2, 2026-08-23) ────
#
# C-POSSESS-1.C closed the arrival window at `next_landing_t - arrival_lead_s`.
# That instant pays the NEXT window's guard — a property of the SCHEDULE — out of
# THIS ball's measured band — a property of the BALL.
#
# **The two scenario periods below are DERIVED from the constants, not typed.**
# They used to be the R5' clamp pin (0.9849 s) and the deferred R6 fork
# (0.7529 s), which were the reachable cadences while the band ceiling was
# 0.800 s. The 2026-08-24 post-FW-14 re-measure took the ceiling to **0.560 s**
# and both rungs walked out of both clauses — R6 now watches its whole band out.
# Re-typing a new pair of rung numbers here would only schedule the same rot for
# the next re-measure, so the periods are computed from the arithmetic that
# defines each clause:
#
#   * C.1 bites when `b - lead` closes inside the band, i.e. for a period in
#     [BAND_MAX, BAND_MAX + lead). Anywhere in that half-open interval the FIXED
#     boundary is exactly `land + BAND_MAX`, which is what C.1 restores.
#   * C.2 bites when the next ball lands before the band closes at all, i.e. for
#     a period BELOW BAND_MAX — where no boundary rule can serve both balls.
#
# Both are now SYNTHETIC: at the collapsed ceiling a C.1 period needs a dwell of
# ~0.157 s and a C.2 period a dwell of ~0.001 s, and the C-HAND-1 hand floor is
# 0.487 s. That is the headline of the re-measure, not a weakening of the tests —
# the clauses are invariants over the WINDOW, and `arrival_window_s` configured
# under the band reaches C.2 from a direction the cadence no longer can.

_R6_FLIGHT_S = 0.5029                        # the deferred fork's flight
#: A period that puts the SUPERSEDED `b - lead` clamp inside the band (C.1).
_AMPUTATING_PERIOD_S = ARRIVAL_BAND_MAX_S + _LEAD_S / 2.0
#: A period below the band ceiling itself, where C.2 is the operative half.
_CLAMPED_PERIOD_S = ARRIVAL_BAND_MAX_S * 0.9
_C1_DWELL_S = _AMPUTATING_PERIOD_S - _R6_FLIGHT_S
_R6_DWELL_S = _CLAMPED_PERIOD_S - _R6_FLIGHT_S


def test_the_arrival_window_never_closes_inside_the_measured_band():
    """C-POSSESS-1.C.1 — the defect, and the fix, at one rung.

    The premise is computed from the tree's constants rather than typed in: at
    a period inside [BAND_MAX, BAND_MAX + lead) the superseded clamp lands
    BEFORE the band ceiling, so a catch that seats in that sliver is a REAL catch
    the window never saw. Two consequences, both asserted:

      * ``catch_event_dt_s`` — the ILC catch-timing measurand, the only number
        this machine has for WHEN the ball entered the cup — goes silently NaN;
      * the empty window answers ``ARRIVAL_REJECTED``, a POSITIVE claim of
        non-arrival minted from a cadence number, which then vetoes a tracker
        CAUGHT (§ 3.2).

    The fix takes the guard out of the NEXT window's opening instead."""
    land = 10.0
    rel, next_land = _cycle(land, _C1_DWELL_S, _R6_FLIGHT_S)
    shipped_close = next_land - _LEAD_S
    assert shipped_close < land + ARRIVAL_BAND_MAX_S, (
        'premise: at this period the shipped clamp closes inside the band')
    # In the amputated sliver, and on the 100 Hz sample grid (the stream starts
    # at land - 1.0, so a centisecond offset is a sample instant, not a rounding
    # of one). DERIVED — the sliver moves with ARRIVAL_BAND_MAX_S, and a typed
    # offset would silently stop being inside it at the next band re-measure.
    rise = land + round(0.5 * (shipped_close - land + ARRIVAL_BAND_MAX_S), 2)
    assert shipped_close < rise < land + ARRIVAL_BAND_MAX_S
    src = _sensor()
    _stream(src, land - 1.0, rise + 0.5, held=lambda t: t >= rise)
    now = rise + 0.4

    # THE DEFECT, driven rather than described: a source whose window closes at
    # the shipped instant is reproduced by giving it that instant as its FIXED
    # window, which is arithmetically the same window C-POSSESS-1.C built out of
    # `next_landing_t - arrival_lead_s`. It cannot see the rise at all, so the
    # catch-event time — the ILC measurand — is NaN.
    shipped_src = _sensor(arrival_window_s=shipped_close - land)
    _stream(shipped_src, land - 1.0, rise + 0.5, held=lambda t: t >= rise)
    assert math.isnan(shipped_src.arrival_time(land))
    # And under the SHIPPED code that empty window answered ARRIVAL_REJECTED — a
    # positive claim of non-arrival minted from a cadence number. Clause C.2 now
    # forbids exactly that, so the same geometry reads UNKNOWN.
    #
    # WHAT THAT UNKNOWN BUYS CHANGED WITH D1 (2026-08-26) and the clause is more
    # load-bearing, not less. It used to buy a fallback to the tracker; the
    # fallback is gone, so what it buys now is that the machine does not claim a
    # MISS it never observed.
    #
    # ⚠ AND IT IS A COULD-NOT-LOOK (audit finding W4, 2026-08-26). This test
    # asserted `arrival_blind(refused) is False` on the argument that "the sensor
    # could look perfectly well, its window was simply cut short". That is true
    # about the CAUSE and wrong about the CONSEQUENCE: nothing watched the ball's
    # band through, which is the only question `arrival_blind` asks. Whether a
    # dead poller or a cadence number closed the eye decides who to blame, not
    # whether anything saw the ball — and `_arrival_state`'s own normative comment
    # already said so ("this is the difference between 'the ball missed' and 'we
    # looked away'. Say the second one out loud"). So SENSOR_BAND_CLAMPED is in
    # BLIND_REASONS and this geometry mints MISSED_SENSOR_BLIND, which is the
    # loud, correctly-routed terminal for it.
    refused = shipped_src.observe(now, landing_t=land)
    assert refused.arrival == ARRIVAL_UNKNOWN
    assert refused.reason == 'SENSOR_BAND_CLAMPED'
    assert refused.arrival != ARRIVAL_REJECTED
    assert arrival_blind(refused) is True
    assert merge_possession(sensor=refused,
                            tracker=_tracker_says(True)).confirmed is False

    v = src.observe(now, landing_t=land,
                    next_release_t=rel, next_landing_t=next_land)
    assert v.arrival == ARRIVAL_CONFIRMED
    assert v.reason == 'SENSOR_ARRIVED'
    assert src.arrival_time(land, next_landing_t=next_land) == pytest.approx(
        rise, abs=0.011)
    # ... and the window closed exactly at the band ceiling, not a millisecond
    # sooner and not at the fixed 1.5 s window either.
    assert src._window(land, next_land)[1] == pytest.approx(   # noqa: SLF001
        land + ARRIVAL_BAND_MAX_S)


def test_the_boundary_abuts_from_both_sides_so_no_edge_is_claimed_twice():
    """C-POSSESS-1.C.1's abutment, which is what lets the closing move at all.

    Moving this window's close LATER without moving the next window's open with
    it would hand one edge to two cycles — the census-D2 fault the clamp exists
    to prevent, re-created by its own fix. Both ends are the SAME
    ``arrival_boundary_t`` call on the SAME pair, so the property is structural:
    an edge one microsecond before the boundary belongs to this cycle alone, one
    microsecond after it to the next cycle alone, and none falls between."""
    land = 10.0
    rel, next_land = _cycle(land, _C1_DWELL_S, _R6_FLIGHT_S)
    b = arrival_boundary_t(land, next_land, _LEAD_S)
    assert b == pytest.approx(land + ARRIVAL_BAND_MAX_S)
    # One rise, placed either side of the boundary; the sensor never sees the
    # cup empty again, so there is exactly one edge to fight over.
    for dt, mine, theirs in ((-0.002, True, False), (+0.002, False, True)):
        s = _sensor()
        rise = b + dt
        _stream(s, land - 1.0, rise + 0.5, held=lambda t, r=rise: t >= r,
                dt=0.001)
        now = rise + 0.4
        this_cycle = s.observe(now, landing_t=land, next_release_t=rel,
                               next_landing_t=next_land).arrival
        next_cycle = s.observe(now, landing_t=next_land,
                               prev_landing_t=land).arrival
        assert (this_cycle == ARRIVAL_CONFIRMED) is mine
        assert (next_cycle == ARRIVAL_CONFIRMED) is theirs


def test_the_previous_landing_can_only_narrow_a_window_never_widen_it():
    """The safety direction of the new argument, asserted rather than argued.

    ``prev_landing_t`` exists to move an OPENING later. If it could ever move one
    EARLIER it would let a cycle reach back for the previous ball's seat edge —
    the false-CAUGHT class § 7 sized and § 3.2 closed. ``arrival_boundary_t`` is
    a ``max`` against ``land - lead`` precisely so that cannot happen, at any
    cadence including ones no rung will ever schedule."""
    land = 10.0
    src = _sensor()
    for period in (0.30, 0.7529, 0.9849, 1.1629, 2.2977, 6.3977):
        w0_plain, w1_plain = src._window(land)                 # noqa: SLF001
        w0, w1 = src._window(land, prev_landing_t=land - period)  # noqa: SLF001
        assert w0 >= w0_plain, period
        assert w1 == w1_plain, period
        # ... and never past the band FLOOR either: the earliest edge a real
        # catch has ever produced must still be inside the window.
        assert w0 <= land + ARRIVAL_BAND_MIN_S, period
        # It must also BITE where it is needed. Below a 1.000 s period the
        # previous cycle's window now runs past `land - arrival_lead_s`, so an
        # opening that had not moved would be claiming an edge its neighbour has
        # already claimed — the D2 fault, from the other side.
        assert (w0 > w0_plain) is (period < ARRIVAL_BAND_MAX_S + _LEAD_S), period


def test_a_band_clamped_window_declares_unknown_instead_of_refusing():
    """C-POSSESS-1.C.2 — the half the boundary rule cannot fix.

    Below an ``ARRIVAL_BAND_MAX_S`` cycle period the next ball lands before this
    one's band has closed, and NO boundary rule can give both balls their whole
    band. A window shorter than the evidence it is judging has
    not observed non-arrival, so ``REJECTED`` — which is a positive claim, and
    which VETOES a tracker CAUGHT — is not available to it. It says UNKNOWN and
    names the cause, so the loss is surfaced rather than silent.

    Since D1 (2026-08-26) there is no tracker to veto, and the clause still binds
    for the reason under the veto: a positive claim of non-arrival from a cadence
    number would now be the WHOLE verdict rather than half of it."""
    land = 10.0
    rel, next_land = _cycle(land, _R6_DWELL_S, _R6_FLIGHT_S)
    assert next_land - land < ARRIVAL_BAND_MAX_S, (
        'premise: the next ball lands before this band closes')
    src = _sensor()
    # The cup never fills: this ball genuinely did not arrive INSIDE the window
    # we were allowed to watch — but we were not allowed to watch it out.
    _stream(src, land - 1.0, land + 2.0, held=False)
    v = src.observe(land + 1.5, landing_t=land,
                    next_release_t=rel, next_landing_t=next_land)
    assert v.arrival == ARRIVAL_UNKNOWN
    assert v.reason == 'SENSOR_BAND_CLAMPED'
    # The whole point of UNKNOWN over REJECTED, restated for the sensor-only era:
    # neither state confirms, but only REJECTED is a CLAIM. A schedule number must
    # never manufacture one.
    #
    # ⚠ IT IS ALSO A COULD-NOT-LOOK (audit finding W4, 2026-08-26): this asserted
    # `arrival_blind(merged) is False` until then, on the argument that a clamped
    # window is not a dead sensor. Correct about the cause, wrong about the
    # consequence — `arrival_blind` asks only whether anything watched the band
    # through, and here nothing did. The terminal is MISSED_SENSOR_BLIND, which
    # names a real fault (the cadence closed the eye) rather than pretending the
    # throw missed. The two are still told apart by the REASON STRING, which is
    # what routes the operator: SENSOR_BAND_CLAMPED says "your schedule", where
    # SENSOR_BLIND says "your cup".
    merged = merge_possession(sensor=v, tracker=_tracker_says(True))
    assert merged.confirmed is False
    assert merged.arrival == ARRIVAL_UNKNOWN
    assert arrival_blind(merged) is True
    assert merged.reason.startswith('SENSOR_BAND_CLAMPED')
    # And a window that DID watch the band out still refuses, at the same rung —
    # so this is a clamp test, not a blanket softening of the refusal.
    wide = _sensor()
    _stream(wide, land - 1.0, land + 2.0, held=False)
    far = wide.observe(land + 1.5, landing_t=land)
    assert far.arrival == ARRIVAL_REJECTED
    assert far.reason == 'SENSOR_NO_ARRIVAL'
    assert merge_possession(sensor=far,
                            tracker=_tracker_says(True)).confirmed is False


def test_the_live_evidence_read_falls_with_the_ball_not_with_the_debounce():
    """CENSUS D3 — the fail-OPEN possession gate, and the direction it fails in.

    The debounce is asymmetric: measured ``held->empty`` 232/241/295 ms,
    ``empty->held`` 0/0/0 ms. At the R5' dwell of 0.49 s, cycle N+1's CHECKING
    runs INSIDE that fall lag, so an ``evidence`` read taken from the debounced
    bit would answer SEATED over a cup the previous ball has already left — and
    the gate whose whole posture is "a dead sensor refuses, it does not pass"
    would pass an empty cup.

    The raw bit falls with the ball. Both bits are asserted, at the same instant,
    so the test states the asymmetry rather than assuming it.

    THE REACHABLE INSTANCE, spelled out rather than asserted, because "the gap
    shrinks" is not a schedule. The case is a **bounce-out during a dwell** — the
    one C-POSSESS-1 § 3.3 edit 2 exists for. Rung R4 (dwell 0.75 s, flight 0.8 s)
    with the census's post-B1/B2 ``throw_delay`` floor of 0.38 s:

        seat edge      landing + 0.137   (the measured band FLOOR)
        bounce-out     landing + 0.20
        debounced fall landing + 0.44    (0.20 + the 0.241 lag)
        CHECKING(N+1)  landing + 0.37    (dwell 0.75 - throw_delay 0.38)

    CHECKING lands 170 ms after the cup emptied and 70 ms before the debounced
    bit notices. Read from the debounced bit the gate PASSES, and cycle N+1
    fires a kind-0 stroke into an empty cup."""
    src = _sensor()
    land = 10.0
    rise = land + 0.137                # the measured catch band FLOOR
    bounce = land + 0.20
    lag = 0.241                        # the measured worst-case fall lag
    check_at = land + (0.75 - 0.38)    # R4 CHECKING: dwell - throw_delay
    assert bounce < check_at < bounce + lag, (
        'premise: CHECKING lands after the cup emptied and inside the fall lag')
    held = lambda t: rise <= t < bounce + lag           # debounced: the fall lags
    raw = lambda t: rise <= t < bounce                  # raw: falls with the ball
    # Streamed only up TO the CHECKING tick: `evidence` is a LIVE query, so a
    # stream that ran past it would answer from a later sample than the gate saw.
    _stream(src, land - 1.0, check_at, held=held, raw=raw)
    assert src.evidence(check_at) == EVIDENCE_EMPTY
    # The debounced bit at that same instant still says SEATED — the fail-open
    # answer this rule removes. `evidence_settled` refuses to pick between them.
    assert src.evidence_settled(check_at) == EVIDENCE_UNKNOWN
    # And once the debounce catches up the two agree again, on EMPTY.
    settled_at = bounce + lag + 0.05
    _stream(src, check_at, settled_at, held=held, raw=raw)
    assert src.evidence_settled(settled_at) == EVIDENCE_EMPTY


def test_settled_evidence_refuses_rather_than_licensing_a_throw_on_a_flicker():
    """The cost of reading raw is chatter, and it has exactly ONE dangerous
    consumer: the auto-reload interlude answers an EMPTY cup by asking BallButler
    to throw a ball at it, so a carry-flicker over a seated ball would put a
    second ball into a full cup.

    ``evidence_settled`` requires both bits to agree. A flicker makes them
    disagree, which is "I could not look with confidence" — UNKNOWN — and the
    interlude gate already refuses on UNKNOWN without moving anything."""
    src = _sensor()
    _stream(src, 0.0, 1.0, held=True, raw=True)
    assert src.evidence_settled(1.0) == EVIDENCE_SEATED
    # One raw sample flickers EMPTY under a seated ball (five misses are needed
    # before the debounced verdict would follow).
    src.note_sample(1.0, held=True, valid=True, raw=False)
    assert src.evidence(1.0) == EVIDENCE_EMPTY          # raw, fail-closed
    assert src.evidence_settled(1.0) == EVIDENCE_UNKNOWN  # and it refuses to act
    # Agreement restores a usable answer.
    src.note_sample(1.01, held=True, valid=True, raw=True)
    assert src.evidence_settled(1.01) == EVIDENCE_SEATED


def test_a_missing_raw_bit_degrades_to_the_debounced_one_never_to_no_ball():
    """A node running against an older bag, or a pre-Phase-5 message, has no
    ``ball_held_raw``. The node passes None rather than a defaulted False, and
    None must read as "use the debounced bit" — defaulting to False would make a
    missing FIELD indistinguishable from an empty CUP, which is the same class of
    error as BallButler's fail-open boot, pointed the other way."""
    src = _sensor()
    _stream(src, 0.0, 1.0, held=True)          # raw omitted entirely
    assert src.evidence(1.0) == EVIDENCE_SEATED
    assert src.evidence_settled(1.0) == EVIDENCE_SEATED


def test_the_release_guard_is_one_constant_not_two():
    """The retention horizon closes at ``next_release - RELEASE_GUARD_S`` and the
    NEXT toss's departure search opens at ``throw - SensorWindows.departure_lead_s``.
    Those are the same instant: the two windows abut, so no fall edge belongs to
    both (a good throw read as a bounce-out) and none falls between them (a real
    bounce-out attributed to the throw). Two copies of that number is how that
    property dies quietly, so there is one — ``SensorWindows.departure_lead_s``
    defaults to ``RELEASE_GUARD_S`` directly. (The ``toss_record.DEPARTURE_LEAD_S``
    alias that used to carry this identity across two modules was dropped at the
    2026-09-13 toss_record merge, R3-f1 — one module, one name.)"""
    windows = SensorWindows(arrival_lead_s=0.1, arrival_window_s=0.5,
                            retention_window_s=1.5)
    assert windows.departure_lead_s is RELEASE_GUARD_S


def test_the_catch_confirm_deadline_clears_the_band_it_has_to_outlast():
    """CENSUS D7. Since 2026-08-10 the possession verdict is sensor-PRIMARY, so
    the deadline that mints MISSED must outlast the band in which a real seat
    edge lands (+87.6…+554.7 ms since the 2026-08-24 post-FW-14 re-measure;
    +137…+798 ms, n=35, when D7 landed). It was a hand-written 0.70 s — 98 ms
    UNDER the ceiling of the day, latent only because the tracker's own CAUGHT
    lands earlier (+202…+442 ms) and the merge falls back to it.

    Deriving it put that re-measure in ONE place, and on 2026-08-24 that is what
    happened: the constant moved 0.80 -> 0.56 and both sequencers plus the
    session's MISS-cleanup floor followed with no edit of their own.

    R4 NOTE (2026-09-24, U6b Cluster A follow-up): the two identity checks this
    test used to run (``toss_sequencer.CATCH_CONFIRM_WINDOW_S is
    ARRIVAL_BAND_MAX_S``, same for ``reload_sequencer``) proved those FSM
    modules ALIASED this constant rather than restating it — an FSM-structural
    claim about two modules deleted with the FSM under `fsm-final`. The
    physical claim below (the source's own window ceiling must outlast the
    measured seat-edge band) is what survives; it is a claim about
    ``ball_possession.ARRIVAL_BAND_MAX_S`` itself, which C-POSSESS-1.C
    (INVARIANTS.md § 5) still enforces through this module."""
    # The measured ceiling the deadline must outlast. +0.5547 s over 33 catches,
    # four post-FW-14 bags, 2026-08-24 (it was +0.798, n=35, 2026-08-10 — and the
    # deadline was a hand-written 0.70 under THAT until census D7).
    assert ARRIVAL_BAND_MAX_S >= 0.5547


# ── B4: the window arithmetic at the pipelined milestone (plan § 2.5) ─────────
#
# Probe FIRST, then the test (the house rule, CLAUDE.md "Empirical probe before
# writing tests"). The confirmed recipes, run 2026-08-27 on the pinned stack:
#
#   /tmp/probe_arrival_clamp_pipelined.py  (P4) — where each milestone period's
#       arrival window CLOSES and whether the seat-edge band is watched out. It
#       CALLS `arrival_boundary_t` rather than restating it, which is that
#       function's own instruction and what the 2026-08-24 audit caught a table
#       for violating. Output: closes at +1.0629 / +1.1381 / +1.1978 / +1.3256
#       against a +0.560 ceiling — the clamp is LIVE at every rung, the band is
#       watched out at every rung, SENSOR_BAND_CLAMPED is unreachable.
#
#   /tmp/probe_retention_inverted.py  (P5) — the retention interval at the same
#       rungs against the measured +0.1839 s median seat edge. Output:
#       -49.0 ms (INVERTED) / +10.7 ms / +11.9 ms, answering UNKNOWN /
#       CONFIRMED / CONFIRMED — never REJECTED.
#
# Both are /tmp one-offs and uncommitted (`tools/probes/README.md`: a one-off
# goes to /tmp, a reusable harness gets promoted). Their FINDINGS live here.

#: § 1.4 / B0-P2's measured median seat edge, the number both probes are run at.
_SEAT_EDGE_MEDIAN_S = 0.1839

#: ``(name, flight, dwell)`` — the § 2.5 rungs the milestone is scored on.
_MILESTONE_RUNGS = (
    ('h=1.0 commanded', 0.9032, 0.4349),
    ('h=1.0 achieved', 0.9032, 0.4946),
    ('h=1.3 commanded', 1.0298, 0.4958),
)


def test_the_clamp_is_live_at_the_milestone_but_the_band_is_still_watched_out():
    """§ 2.5's FINDING, and it CORRECTS a premise this plan was commissioned on:
    **``SENSOR_BAND_CLAMPED`` does not become reachable at the milestone.**

    Three distinct thresholds, routinely conflated:

      * the fixed window stops binding below a **1.700 s** period
        (``arrival_window_s`` 1.5 + ``arrival_lead_s`` 0.2) — every rung in
        scope, so the CLAMP is live and is what closes the arrival search;
      * the shipped rule AMPUTATES below a **0.560 s** period
        (``ARRIVAL_BAND_MAX_S``) — and the nearest in-scope period is 1.338 s, a
        factor of 2.4 away.

    So the clamp becoming live is real and the blind-bucket refusal is not
    reachable from it. Evaluated by CALLING ``arrival_boundary_t`` — never by
    restating its formula, which is that function's own instruction (probe P4)."""
    landing = 0.0
    for name, flight, dwell in _MILESTONE_RUNGS:
        period = flight + dwell
        assert period < hw.JB_BD_ARRIVAL_WINDOW_S + hw.JB_BD_ARRIVAL_LEAD_S, (
            '{}: the fixed window would still bind, so the clamp is not '
            'live'.format(name))
        assert period > ARRIVAL_BAND_MAX_S * 2.0, (
            '{}: this rung is close enough to amputation to need a '
            're-argument'.format(name))
        closes = arrival_boundary_t(landing, landing + period,
                                    arrival_lead_s=float(hw.JB_BD_ARRIVAL_LEAD_S))
        assert closes - landing >= ARRIVAL_BAND_MAX_S, (
            '{}: the seat-edge band is amputated'.format(name))
    # the exact instants probe P4 printed, so a constant edit moves this test
    assert arrival_boundary_t(0.0, 0.9032 + 0.4349,
                              arrival_lead_s=float(hw.JB_BD_ARRIVAL_LEAD_S)) \
        == pytest.approx(1.1381, abs=5e-4)


def test_the_arrival_windows_of_two_live_cycles_abut_by_identity():
    """T-U10 — § 2.5(b): with two slots live the abutment holds BY IDENTITY.

    ``arrival_boundary_t(P, L)`` closing ``L``'s window and opening ``N``'s is
    LITERALLY THE SAME CALL ON THE SAME PAIR, so the two ends cannot drift as
    long as the three reads name the right slots (the committed slot's landing,
    the previously-committed slot's, the staged slot's). This asserts the
    identity itself — the node-side slot naming is pinned separately by
    ``test_the_sensor_is_told_the_committed_slots_landing``.

    "Two computations of a boundary is how an abutment stops abutting" is the
    function's own docstring; at the milestone periods the boundary is LIVE, so
    the property stops being theoretical."""
    lead = float(hw.JB_BD_ARRIVAL_LEAD_S)
    for name, flight, dwell in _MILESTONE_RUNGS:
        period = flight + dwell
        prev_landing, landing = 0.0, period
        next_landing = 2.0 * period
        close_of_this = arrival_boundary_t(landing, next_landing,
                                           arrival_lead_s=lead)
        open_of_next = arrival_boundary_t(landing, next_landing,
                                          arrival_lead_s=lead)
        assert close_of_this == open_of_next, name          # the same float
        # …and the previous pair's boundary is the one that opened THIS window,
        # so the chain abuts end to end with no gap and no overlap.
        open_of_this = arrival_boundary_t(prev_landing, landing,
                                          arrival_lead_s=lead)
        assert open_of_this < close_of_this, name
        assert open_of_this <= landing <= close_of_this, name


def test_an_inverted_retention_interval_answers_unknown_not_rejected():
    """T-U9 — § 2.5(c), and at these dwells the inverted case is the NORMAL one.

    Retention closes at ``next_release − RELEASE_GUARD_S``. Against the measured
    +183.9 ms median seat edge that is **−49.0 ms (inverted)** at the h = 1.0
    commanded dwell, +10.7 ms at its achieved dwell and +11.9 ms at h = 1.3
    (probe P5, 2026-08-27).

    C-POSSESS-1.C already governs it — *"where a clamp leaves no interval at
    all, the part it governs is UNKNOWN — never CONFIRMED… and never
    REJECTED"* — but it is now the normal case rather than an edge one, and a
    ``REJECTED`` here would be **a positive claim of a bounce-out on every good
    cycle**. That is what this test forbids."""
    step = 1.0 / 200.0
    seen = {}
    for name, flight, dwell in _MILESTONE_RUNGS:
        landing = 1000.0
        next_release = landing + dwell
        src = HandBallSensorSource(
            arrival_lead_s=float(hw.JB_BD_ARRIVAL_LEAD_S),
            arrival_window_s=float(hw.JB_BD_ARRIVAL_WINDOW_S),
            retention_window_s=float(hw.JB_BD_RETENTION_WINDOW_S),
            stale_s=0.5)
        t = landing - 1.0
        while t <= next_release + 0.5:
            # A perfectly healthy catch: empty through the flight, HELD from the
            # measured seat edge onward, and never bouncing out.
            held = t >= landing + _SEAT_EDGE_MEDIAN_S
            src.note_sample(t, held=held, valid=True, raw=held)
            t += step
        verdict = merge_possession(sensor=src.observe(
            landing + float(hw.JB_BD_ARRIVAL_WINDOW_S), landing,
            next_release_t=next_release,
            next_landing_t=next_release + flight, prev_landing_t=None))
        assert verdict.retention != RETENTION_REJECTED, (
            '{}: a REJECTED here is a positive bounce-out claim on a healthy '
            'cycle'.format(name))
        seen[name] = verdict.retention
        interval = (next_release - RELEASE_GUARD_S) - (landing
                                                       + _SEAT_EDGE_MEDIAN_S)
        if interval < 0.0:
            assert verdict.retention == RETENTION_UNKNOWN, name
    # The three rungs the probe printed, and the h=1.0 COMMANDED one is the
    # inverted one — pinned by name so a dwell edit that un-inverts it is
    # visible rather than silently making this test vacuous.
    assert seen['h=1.0 commanded'] == RETENTION_UNKNOWN
    assert seen['h=1.0 achieved'] == RETENTION_CONFIRMED
    assert seen['h=1.3 commanded'] == RETENTION_CONFIRMED


# ══════════════════════════════════════════════════════════════════════════
# Moved from tests/motion/test_toss_record.py (2026-09-13, R3-f1 — the
# learning-stack deletion; plans/active/two-ball-skill-stack.md § 4 R3).
# Only the tests of the symbols that moved WITH them (schema/FIELDS, the
# announced-ball latch, sensor edges/poll cadence, label_from_sensor, join).
# encode/decode/validate and their tests did NOT move — those functions
# stayed behind with the rest of toss_record.py and were deleted; a schema
# drift guard (test_fields_are_pinned etc.) still applies to FIELDS, which
# join() still needs and which moved for that reason.
# ══════════════════════════════════════════════════════════════════════════
WINDOWS = bp.SensorWindows(
    arrival_lead_s=float(hw.JB_BD_ARRIVAL_LEAD_S),
    arrival_window_s=float(hw.JB_BD_ARRIVAL_WINDOW_S),
    retention_window_s=float(hw.JB_BD_RETENTION_WINDOW_S))

#: Measured on ~/Desktop/rosbags/2026-08-10_16-30-44 (2026-08-10): the debounced
#: held->empty edge lags the raw one by 232/241/295 ms (min/med/max) while the
#: empty->held edge has ZERO lag. Reproduced in every synthetic stream here.
DEBOUNCE_FALL_LAG_S = 0.24

THROW_T = 100.0
FLIGHT_S = 0.8
LANDING_T = THROW_T + FLIGHT_S


def stream(*, departure_dt=0.17, catch_dt=None, drop_after=None,
           blind=None, dt=0.01, fall_lag=DEBOUNCE_FALL_LAG_S):
    """A synthetic /hand_telemetry stream with the REAL debounce asymmetry.

    ``blind`` is a ``(lo, hi)`` span of invalid samples — the tri-state UNKNOWN
    the sensor genuinely produces, not a level.
    """
    t_dep = None if departure_dt is None else THROW_T + departure_dt
    t_catch = None if catch_dt is None else LANDING_T + catch_dt
    t_drop = None if (t_catch is None or drop_after is None) else (
        t_catch + drop_after)
    out = []
    t = THROW_T - 3.0
    while t < LANDING_T + 6.0:
        raw = True
        if t_dep is not None and t >= t_dep:
            raw = False
        if t_catch is not None and t >= t_catch:
            raw = True
        if t_drop is not None and t >= t_drop:
            raw = False
        deb = True
        if t_dep is not None and t >= t_dep + fall_lag:
            deb = False
        if t_catch is not None and t >= t_catch:
            deb = True
        if t_drop is not None and t >= t_drop + fall_lag:
            deb = False
        valid = not (blind and blind[0] <= t <= blind[1])
        out.append(bp.SensorSample(t=t, held=deb, raw=raw, valid=valid,
                                   stamp=t if valid else 0.0))
        t += dt
    return out


def label(**kw):
    return bp.label_from_sensor(stream(**kw), throw_time=THROW_T,
                                landing_time=LANDING_T, windows=WINDOWS)

# ── 1. Schema drift guard ─────────────────────────────────────────────────────

#: Every field name in ``toss_record/1``, pinned. ADDING a field is purely
#: additive and only needs this list extended; REMOVING one or changing what an
#: existing name means must bump ``SCHEMA`` (plan § 3.7 item 1) — this list is
#: the thing that makes that a decision rather than an accident.
EXPECTED_FIELDS = (
    # identity
    'schema', 'toss_uid', 'session_id', 'goal_id', 'action', 'cycle_index',
    'announce_throw_time_ros', 'announce_landing_time_ros', 't_record_ros',
    'perf_minus_ros_s', 'perf_minus_ros_inst_s',
    # provenance
    'git_sha', 'git_dirty', 'toss_cal_version', 'toss_cal_loaded',
    'toss_cal_applied', 'tilt_map_version', 'tilt_map_applied',
    'gravity_correction_loaded', 'level_offset_rad', 'toss_tier',
    'bridge_fw_version', 'platform_fw_version', 'uptime_ms_at_release',
    'hand_odrive_config_sha', 'catch_knobs',
    # goal
    'goal_catch_xyz_stow_mm', 'goal_throw_height_m', 'goal_throw_height_m_raw',
    'goal_throw_delay_s', 'goal_throw_delay_s_raw', 'goal_catch_vel_scale',
    'goal_catch_vel_scale_raw', 'goal_num_throws', 'goal_dwell_time_s',
    'goal_stop_on_miss', 'goal_on_empty_cup', 'goal_max_reloads',
    # release
    'flight_time_s', 'apex_height_m', 'event_vel_mps', 'event_delay_s',
    'release_latency_ms_applied', 'release_pos_global_mm', 'launch_vel_mms',
    'catch_point_global_mm', 'aim_tilt_rx_rad', 'aim_tilt_ry_rad',
    'throw_site_xy_mm',
    # calibration
    'map_aim_rad', 'trim_aim_rad', 'trim_monitor_aim_rad', 'trim_authority',
    'total_aim_rad', 'map_aim_mm_at_h',
    'trim_aim_mm_at_h', 'ilc_aim_rad',
    'ilc_spatial_aim_rad', 'ilc_session_aim_rad', 'ilc_session_applied',
    'ilc_session_reason', 'ilc_session_n',
    'ilc_vel_trim', 'speed_bias_applied',
    'timing_bias_applied_ms',
    'clamp_hits', 'trim_source_n', 'trim_state', 'trim_reset_reason',
    # dwell tilt (Layer 1.5)
    'dwell_tilt_rad', 'dwell_tilt_sd_rad', 'dwell_tilt_n', 'dwell_tilt_span_s',
    'dwell_tilt_last_read_to_release_s', 'dwell_tilt_degraded',
    # fsm
    'outcome', 'success', 'phase_at_terminal', 'throw_dispatch_class',
    'throw_dispatch_message', 'prepare_ok', 'position_accepted',
    'position_planned_s', 'position_code',
    # The POSITIONING BUSY absorb (2026-08-29). ADDITIVE — no SCHEMA bump, per
    # the schema's own rule; 0.0 / 0 (not null) on every cycle that ran an FSM.
    # The pair is HOW LONG + HOW MANY, as commit_slip_s is to commit_slips.
    'position_busy_wait_s',
    'position_busy_polls',
    'catch_target_accepted',
    'announce_lead_short', 'throw_stroke_seen', 'ball_track_confirmed',
    't_accept_perf', 't_release_perf', 't_landing_sched_perf', 'reload_settle',
    'retry_of', 'achieved_flight_s_fsm', 'catch_error_mm_fsm',
    'catch_event_dt_s_fsm',
    # sensor
    'sensor_valid_frac', 'sensor_n_samples', 'sensor_held_at_dispatch',
    't_departure_raw_ros', 't_departure_deb_ros', 't_catch_raw_ros',
    't_catch_deb_ros', 't_dropout_ros', 'held_at_catch_plus_retention',
    'sensor_edge_count', 'sensor_poll_dt_ms_median',
    'ball_held_stamp_wall_anchored',
    # mocap
    'land_xy_global_mm', 'land_err_mm', 'land_err_norm_mm', 'n_fit',
    'fit_rms_mm', 'fit_sparse', 'apex_z_mm', 'achieved_flight_s_mocap',
    't_land_bag', 'qtm_offset_s', 'mocap_gap_ms_max', 'land_plane_mm',
    'floor_arrival',
    # command reference for the mined errors (ILC Phase 0a/0c)
    'cmd_launch_vel_mms', 'cmd_flight_time_s', 'cmd_release_source',
    # the WHOLE-ARC fit (ILC entry condition E-1, resolved 2026-08-13)
    'arc_fit_n', 'arc_fit_rms_mm', 'arc_lateral_vel_se_mms', 'coverage_asym_s',
    # arrival kinematics (ILC Phase 0a)
    'arrival_vel_mms', 'arrival_dir_err_rad', 'arrival_dir_err_norm_rad',
    'arrival_speed_err_mms', 't_arrival_fit_bag', 'arrival_fit_n',
    'arrival_fit_rms_mm', 'arrival_vel_se_mms', 'flight_time_err_s',
    # release-state backcast (ILC Phase 0c)
    'release_pos_track_mm', 'release_vel_track_mms', 't_release_fit_bag',
    'release_time_err_ms', 'release_vel_err_mms', 'release_speed_err_mms',
    'release_dir_err_rad', 'backcast_fit_n', 'backcast_fit_rms_mm',
    'release_vel_se_mms',
    # release-vs-flight split of the landing error
    'land_err_release_mm', 'land_err_flight_mm',
    # plant
    'stroke_peak_rev', 'dip_below_x3_rev', 'pullback_rps', 'trunc', 'seeds',
    'iq_brake_min_a', 'dispatch_shift_ms', 'can_errors',
    'bridge_tx_diag', 'plant_block_source',
    # loop timing (instrument only)
    'loop_n_pre', 'loop_period_max_pre_s', 'loop_period_mean_pre_s',
    'loop_work_max_pre_s', 'loop_obs_max_pre_s', 'loop_body_max_pre_s',
    'loop_sleep_max_pre_s', 'loop_n_over_pre', 'loop_n_post',
    'loop_period_max_post_s',
    # The two-slot pipeline (B4, 2026-08-27). ADDITIVE — no SCHEMA bump, per
    # the schema's own rule; null on every serial cycle.
    'staged_at_s',
    'commit_at_s',
    'commit_slip_s',
    'commit_slips',
    'staged_discarded_reason',
    # quality
    'label', 'label_source', 'label_confidence', 'label_reason', 'rimshot',
    'disagreement', 'record_provenance', 'join_residual_ms',
    'usable_for_aim_fit', 'usable_for_timing_fit', 'usable_for_speed_fit',
    'usable_for_release_fit', 'usable_for_lateral_fit', 'excluded_reason',
)


def test_fields_are_pinned():
    """The drift guard. A removal or a rename goes red here first."""
    assert bp.FIELD_NAMES == EXPECTED_FIELDS


def test_schema_version_is_pinned():
    assert bp.SCHEMA == 'toss_record/1'


def test_field_names_are_unique():
    assert len(set(bp.FIELD_NAMES)) == len(bp.FIELD_NAMES)


def test_names_by_origin_partitions_the_whole_schema():
    """The origin table is how a corpus reader tells a declared number from a
    mined one WITHOUT a per-field provenance marker on every row (§ 3.3 asks the
    declaration to "upgrade" fields; row-level ``record_provenance`` plus this
    partition says the same thing at a fraction of the schema cost). It only
    works if the partition is total and disjoint."""
    seen = []
    for origin in ('D', 'M', 'DM', 'X'):
        seen.extend(bp.names_by_origin(origin))
    assert sorted(seen) == sorted(bp.FIELD_NAMES)
    assert len(seen) == len(set(seen))
    # The join key must be independently recoverable, or a bag with no
    # declaration cannot be joined to anything — including itself.
    assert 'announce_throw_time_ros' in bp.names_by_origin('DM')
    # The label is DERIVED, never declared: the node terminalises 0.80 s before
    # the retention window closes, so it structurally cannot know.
    assert 'label' in bp.names_by_origin('X')


def test_every_field_declares_a_known_origin_and_kind():
    for f in bp.FIELDS:
        assert f.origin in ('D', 'M', 'DM', 'X'), f
        assert f.kind in ('s', 'f', 'i', 'b', 'f2', 'f3', 'l', 'o'), f

# ── 2. The labeller ───────────────────────────────────────────────────────────

def test_a_clean_catch_is_caught():
    got = label(catch_dt=0.4)
    assert got.label == bp.LABEL_CAUGHT
    assert got.fields['held_at_catch_plus_retention'] is True


def test_no_departure_edge_is_no_release():
    """The ball never left the cup — which is a DIFFERENT fact from a miss, and
    a fit that pooled the two would be learning aim from tosses that never
    happened."""
    got = label(departure_dt=None, catch_dt=None)
    assert got.label == bp.LABEL_NO_RELEASE
    assert got.fields['sensor_held_at_dispatch'] is True


def test_departure_without_arrival_is_missed():
    got = label(catch_dt=None)
    assert got.label == bp.LABEL_MISSED
    assert got.fields['t_catch_deb_ros'] is None


def test_an_arrival_that_leaves_inside_retention_is_bounced():
    got = label(catch_dt=0.4, drop_after=0.999)
    assert got.label == bp.LABEL_BOUNCED
    assert got.fields['t_dropout_ros'] is not None


@pytest.mark.parametrize('catch_dt', (0.0876, 0.137, 0.5547, 0.798))
def test_the_measured_catch_band_labels_caught_at_both_ends(catch_dt):
    """Both measured populations' ends, because the search window still has to
    admit both.

    +137 ms / +798 ms are the earliest and latest arrivals in the 2026-08-10
    three-bag population, and the +798 row is the one that SIZED
    ``JB_BD_ARRIVAL_WINDOW_S`` — that knob did NOT move at the 2026-08-24
    re-measure, so that row is still the sizing row and still has to label
    CAUGHT. +87.6 ms / +554.7 ms are the ends of the post-FW-14 population the
    band constants were re-cut from (n=33, four bags). If the search window is
    ever trimmed toward the median, the +798 row goes red first, which is the
    point."""
    assert label(catch_dt=catch_dt).label == bp.LABEL_CAUGHT


def test_the_plans_draft_catch_window_would_have_relabelled_a_measured_catch():
    """§ 3.3's draft used ``CATCH_CONFIRM_WINDOW_S`` (0.70 s) as the arrival
    search window. That constant is the FSM's terminal DEADLINE, not a sensor
    window.

    Measured against the reference bag (2026-08-10, 25 catches): at 0.70 s
    exactly ONE arrival relabels MISSED — the +798 ms row. That is not a small
    consequence, because that row is the population MAXIMUM and is the row
    ``JB_BD_ARRIVAL_WINDOW_S`` was sized on; the runner-up (+675 ms) sits 25 ms
    inside the draft boundary, so the margin is one session's variation wide. The
    shipped window is used instead, which also makes the offline labeller and the
    live ``HandBallSensorSource`` agree by construction (D11).

    **The relabel is now to UNKNOWN, not to MISSED** — changed 2026-08-23 with
    C-POSSESS-1.C.2, and it strengthens rather than softens this argument. A
    window that stops looking before the evidence runs out cannot mint MISSED,
    which is a POSITIVE claim of non-arrival; it can no longer produce a verdict
    at all. That is exactly the enforcement D7 wanted when it derived
    ``CATCH_CONFIRM_WINDOW_S`` from the band in the first place. The row is still
    lost to the fit either way; it is now lost with the cause on the record.

    **The literal 0.70 s draft no longer trips this, and for a reason worth
    recording**: the 2026-08-24 re-measure took ``ARRIVAL_BAND_MAX_S`` to 0.56 s,
    so a 0.70 s window now OUTLASTS the band. The draft would be safe today — by
    accident, on a plant the plan's author never measured, and it would go unsafe
    again the moment the band widened. So the demonstration window is DERIVED to
    sit just under whatever the band currently is, and the historical 0.70 s is
    asserted separately for what it now is: adequate, and adequate by luck.
    """
    draft = bp.SensorWindows(arrival_lead_s=0.30,
                             arrival_window_s=ARRIVAL_BAND_MAX_S - 0.01,
                             retention_window_s=WINDOWS.retention_window_s)

    def under(windows, catch_dt):
        return bp.label_from_sensor(
            stream(catch_dt=catch_dt), throw_time=THROW_T,
            landing_time=LANDING_T, windows=windows)

    assert under(WINDOWS, 0.798).label == bp.LABEL_CAUGHT
    assert draft.arrival_window_s < ARRIVAL_BAND_MAX_S, (
        'premise: the draft window is shorter than the band it must judge')
    # The plan's literal 0.70 s, stated for the record: it outlasts today's band
    # and so labels normally — the historical hazard is gone, not the mechanism.
    plan_draft = bp.SensorWindows(
        arrival_lead_s=0.30, arrival_window_s=0.70,
        retention_window_s=WINDOWS.retention_window_s)
    assert plan_draft.arrival_window_s > ARRIVAL_BAND_MAX_S
    relabelled = under(draft, draft.arrival_window_s + 0.005)
    assert relabelled.label == bp.LABEL_UNKNOWN
    assert 'band clamped' in relabelled.reason
    # …and an arrival INSIDE the short window still labels normally, so this is a
    # clamp test rather than a blanket softening of the label. (In the 2026-08-10
    # argument this row was the +675 ms runner-up, 25 ms inside the plan's 0.70 s
    # boundary — the measure of how thin that margin was. Derived now, because
    # "just inside the window" is the property, not the number.)
    assert under(draft, draft.arrival_window_s - 0.025).label == bp.LABEL_CAUGHT


def test_an_invalid_sample_in_the_window_is_unknown_and_never_collapses():
    """D13 / C-POSSESS-1 § 2. Treating "no valid sample" as "no ball" mints a
    false MISSED on every telemetry hiccup — and those false MISSEDs are exactly
    the records the aim fit wants most, because a miss with a clean mocap fit is
    the most informative aim datum there is."""
    got = label(catch_dt=0.4, blind=(LANDING_T - 0.1, LANDING_T + 0.5))
    assert got.label == bp.LABEL_UNKNOWN
    assert got.confidence == 0.0
    assert got.fields['sensor_valid_frac'] < 1.0


def test_an_empty_window_is_unknown_not_no_release():
    """No samples at all is the sensor not looking, which is UNKNOWN. Reading it
    as NO_RELEASE would invent a mechanical fact out of a missing topic."""
    got = bp.label_from_sensor([], throw_time=THROW_T, landing_time=LANDING_T,
                               windows=WINDOWS)
    assert got.label == bp.LABEL_UNKNOWN
    assert got.fields['sensor_n_samples'] == 0


# ── D12: raw for TIMES, debounced for the VERDICT ─────────────────────────────

def test_the_departure_time_comes_from_the_raw_bit():
    """The measurement that makes D12 a field rather than a footnote: on the
    reference bag the debounced fall lags the raw one by 232-295 ms. A timing fit
    taken off the debounced edge would carry that as a systematic late bias —
    comparable to the +118-133 ms uptime dispatch shift the whole fresh-boot
    discipline exists to control, and indistinguishable from real physics."""
    got = label(departure_dt=0.17, catch_dt=0.4)
    raw = got.fields['t_departure_raw_ros'] - THROW_T
    deb = got.fields['t_departure_deb_ros'] - THROW_T
    assert raw == pytest.approx(0.17, abs=0.011)
    assert deb - raw == pytest.approx(DEBOUNCE_FALL_LAG_S, abs=0.011)


def test_the_catch_time_is_debounce_free_on_both_bits():
    """The other half of the asymmetry, measured at 0/0/0 ms: any single HELD
    reading restores HELD (``plans/archived/hand-ball-sensor.md`` § Debounce
    asymmetry), so raw and debounced agree on the arrival edge. Recording both
    is what proves it stayed true rather than assuming it."""
    got = label(catch_dt=0.4)
    assert got.fields['t_catch_raw_ros'] == pytest.approx(
        got.fields['t_catch_deb_ros'], abs=1e-9)


def test_the_verdict_survives_a_raw_bit_that_flickers():
    """Raw carried 41/42 edges against the debounced 38/39 on the reference bag —
    three spurious pairs. The VERDICT must come off the debounced bit, or a
    contact flicker becomes a bounce-out that never happened."""
    samples = stream(catch_dt=0.4)
    flicked = []
    for s in samples:
        # A single raw dropout mid-hold, invisible to the debounce.
        if LANDING_T + 0.9 <= s.t < LANDING_T + 0.92:
            s = s._replace(raw=False)
        flicked.append(s)
    got = bp.label_from_sensor(flicked, throw_time=THROW_T,
                               landing_time=LANDING_T, windows=WINDOWS)
    assert got.label == bp.LABEL_CAUGHT


def test_the_departure_search_cannot_reach_the_arrival_window():
    """A bounce-out arrives-then-leaves after landing. If the departure window
    ran to ``throw + 1.0`` unclamped it would swallow that fall on a short
    flight and report the bounce as the release."""
    got = label(departure_dt=0.17, catch_dt=0.1, drop_after=0.3)
    assert got.fields['t_departure_raw_ros'] < LANDING_T - \
        WINDOWS.arrival_lead_s


def test_the_measured_departure_band_is_inside_the_window():
    """+148..+212 ms measured across all 32 self-tosses in the reference bag.
    Both ends must label; a window sized to the median would drop the tail."""
    for dt in (0.148, 0.212):
        got = label(departure_dt=dt, catch_dt=0.4)
        assert got.label == bp.LABEL_CAUGHT
        assert got.fields['t_departure_raw_ros'] == pytest.approx(
            THROW_T + dt, abs=0.011)


def test_the_retention_window_is_guarded_on_BOTH_sides_absolutely():
    """Twins straddling the shipped 1.50 s boundary, written ABSOLUTE.

    ``drop_after`` is a RAW instant; the retention rule reads the DEBOUNCED
    dropout, which lags by ~240 ms. So the debounced held-segment is
    ``drop_after + 0.24``:

        raw 1.20  ->  debounced 1.44  <  1.50   =>  BOUNCED
        raw 1.32  ->  debounced 1.56  >  1.50   =>  CAUGHT

    Both twins are absolute, so BOTH directions fail: widen to 1.6 and the 1.56
    case wrongly reads BOUNCED; narrow to 1.4 and the 1.44 case wrongly reads
    CAUGHT. A case phrased as ``retention_window_s + 0.05`` would follow the
    constant wherever it went and could only ever catch a narrowing — the
    one-sidedness Phase 1's audit found next door in
    ``hand_sensor_verdict_replay``.

    That the retention window is a DEBOUNCED-domain quantity is not an accident
    of this test: Phase 1 sized it on debounced seat-then-leave durations
    (0.571 / 0.989 / 0.999 s), so it already carries the lag by construction.
    """
    assert float(hw.JB_BD_RETENTION_WINDOW_S) == 1.5
    assert label(catch_dt=0.4, drop_after=1.20).label == bp.LABEL_BOUNCED
    assert label(catch_dt=0.4, drop_after=1.32).label == bp.LABEL_CAUGHT


def test_the_measured_poll_cadence_is_reported_not_assumed():
    """``sensor_poll_dt_ms_median`` is measured from ``ball_held_stamp``
    advances, per record, and is never inherited from a constant.

    Why it must be measured rather than assumed: the same configured 20 ms
    interval has produced a measured median of 71 ms on
    ``2026-08-10_16-30-44`` and of 20 ms on captures since the FW 14 can-bridge
    fix (2026-08-15). Neither number is a property of the plant — which is the
    whole reason this is a field (``logbook/2026-08-24-hand-sensor-poll-cadence.md``).

    This stream advances the stamp on EVERY sample, so it measures the
    arithmetic only; the republish dedupe that separates a poll from a publish
    is exercised below.
    """
    got = label(catch_dt=0.4)
    assert got.fields['sensor_poll_dt_ms_median'] == pytest.approx(10.0,
                                                                   abs=0.5)


# ── 2b. The poll cadence: what counts as a step ───────────────────────────────
#
# ``/hand_telemetry`` republishes the cached bit at 100 Hz while the bridge polls
# the switch far slower, so a "step" is the advance of ``ball_held_stamp``
# between two consecutive VALID samples — never a message, never a repeat, never
# a jump across a gap or a change of stamp epoch. Each test below drives one of
# those four rules; before them the streams here advanced the stamp on every
# sample, so the dedupe that separates the publish rate from the poll rate was
# never exercised at all.

WALL_T0 = 1786510521.957        # 2026-08-12_14-55-18's first sample, verbatim
BOOT_STAMPS = (17.140004, 17.154007, 17.154007, 17.191004, 17.211004,
               17.251004)       # its 6 leading BOOT-RELATIVE stamps, verbatim


def poll_stream(*, poll_dt=0.020, publish_dt=0.010, n_polls=40, t0=THROW_T):
    """A stream that REPUBLISHES one poll several times, as the bridge does.

    ``publish_dt`` is the ``/hand_telemetry`` period (10 ms shipped) and
    ``poll_dt`` the interval at which ``ball_held_stamp`` actually advances. The
    stamp is built by integer division rather than by quantising ``t``, so the
    repeat count is exact and no float accumulation can turn one poll into two.
    """
    repeat = int(round(poll_dt / publish_dt))
    return [bp.SensorSample(t=t0 + k * publish_dt, held=True, raw=True,
                            valid=True,
                            stamp=t0 + (k // repeat) * poll_dt)
            for k in range(n_polls * repeat)]


@pytest.mark.parametrize('poll_dt', [0.020, 0.050, 0.071])
def test_the_poll_cadence_is_the_poll_rate_not_the_republish_rate(poll_dt):
    """THE measurement this field exists for, and the one the synthetic streams
    above could not make: at 100 Hz publish the stamp repeats, and counting
    messages would report 10 ms — the publish period — for every plant.

    Parametrised across three cadences this robot has actually measured so no
    single one can be special-cased, and so the test cannot be satisfied by a
    constant.
    """
    samples = poll_stream(poll_dt=poll_dt, publish_dt=0.010)
    got = bp.poll_dt_ms_median(samples)
    assert got == pytest.approx(poll_dt * 1e3, abs=0.5)
    assert got != pytest.approx(10.0, abs=0.5), 'reported the PUBLISH rate'
    steps = bp.poll_dt_steps_ms(samples)
    # One step per POLL, not per message: 40 polls -> 39 steps, whatever the
    # republish factor.
    assert len(steps.steps_ms) == 39
    assert steps.n_backwards == 0 and steps.n_domain_breaks == 0


def test_a_backwards_stamp_step_is_dropped_and_counted_not_folded_in():
    """The stamp is bridge-sourced, so a re-anchor can make it go DOWN. Folded
    in as a negative it drags the median toward zero, and any consumer that
    divides by it ("how many polls fit in this window") gets an absurd or
    negative count. Dropped AND counted: a stream that steps backwards at all is
    a finding about the bridge clock, not a cadence.

    Dropping the negative is only half the contract, and the half that is easy
    to get wrong is the other one: if ``prev`` is carried forward onto the LOW
    stamp, the very NEXT sample measures the whole re-anchor distance in the
    POSITIVE direction. A 0.5 s re-anchor then mints a 500 ms "poll" — 25x the
    real cadence, sitting in ``steps_ms`` where every max and p95 reads it — and
    ``min(...) > 0`` says nothing about it. So this pins the MAX, not the sign:
    the re-anchor costs exactly one interval (39 steps -> 38) and mints none.
    """
    samples = poll_stream(poll_dt=0.020)
    k = 20
    hurt = list(samples)
    hurt[k] = samples[k]._replace(stamp=samples[k].stamp - 0.5)
    steps = bp.poll_dt_steps_ms(hurt)
    assert steps.n_backwards == 1
    assert min(steps.steps_ms) > 0.0
    assert max(steps.steps_ms) == pytest.approx(20.0, abs=0.5), (
        'the re-anchor distance came back as a forward step')
    # 40 polls -> 39 steps unbroken; the re-anchor loses the ONE interval that
    # spans it and the tracker reseeds, so 38 — never fewer, never a 39th.
    assert len(steps.steps_ms) == 38
    assert steps.n_domain_breaks == 0
    assert bp.poll_dt_ms_median(hurt) == pytest.approx(20.0, abs=0.5)


def test_an_invalid_span_resets_the_tracker_rather_than_spanning_it():
    """``ball_held_valid`` false is UNKNOWN, and the bridge may have polled many
    times while it was. The stamp difference either side of the gap therefore
    spans an unknown number of polls and is not ONE interval — carrying ``prev``
    across it would mint a single fake 200 ms poll out of a 200 ms blind span.
    """
    samples = poll_stream(poll_dt=0.020, n_polls=40)
    blind = range(30, 50)          # 20 samples = 10 polls of darkness
    hurt = [s._replace(valid=False) if i in blind else s
            for i, s in enumerate(samples)]
    steps = bp.poll_dt_steps_ms(hurt)
    assert max(steps.steps_ms) == pytest.approx(20.0, abs=0.5), (
        'a step spanned the blind gap')
    # 40 polls, 10 of them lost to the gap, and the first poll after it only
    # seeds the tracker: strictly fewer steps than the unbroken stream's 39.
    assert len(steps.steps_ms) < 39
    assert bp.poll_dt_ms_median(hurt) == pytest.approx(20.0, abs=0.5)


def test_the_wall_anchor_discontinuity_is_not_a_poll_interval():
    """``ball_held_stamp`` is wall-epoch only AFTER the bridge's wall anchor
    lands; before it the stamp is boot-relative (module docstring, CLOCK
    DOMAINS). A capture spanning the anchor therefore contains one step of the
    whole wall epoch.

    The stamps here are verbatim from ``2026-08-12_14-55-18``, where that step
    measures **+1.79e12 ms** (17.251 s -> 1786510522.066 s). On that bag it is
    excluded only by luck — its six pre-anchor samples happen to carry
    ``ball_held_valid`` false. This test drives the reachable case, a VALID
    pre-anchor span, which is exactly what a record with
    ``ball_held_stamp_wall_anchored == false`` is: the guard is what makes the
    exclusion a rule instead of an accident.

    A cadence is a DIFFERENCE, so a wholly boot-relative stream still measures
    correctly — only the CHANGE of epoch is refused, and it is counted.
    """
    pre = [bp.SensorSample(t=WALL_T0 + k * 0.010, held=True, raw=True,
                           valid=True, stamp=BOOT_STAMPS[k])
           for k in range(len(BOOT_STAMPS))]
    post = poll_stream(poll_dt=0.020, n_polls=20,
                       t0=WALL_T0 + len(BOOT_STAMPS) * 0.010)
    steps = bp.poll_dt_steps_ms(pre + post)
    assert steps.n_domain_breaks == 1
    assert max(steps.steps_ms) < 1e3, 'the wall epoch entered the statistics'
    assert bp.poll_dt_ms_median(pre + post) == pytest.approx(20.0, abs=1.0)
    # And the same stream read wholly inside ONE epoch still MEASURES: these
    # six boot-relative stamps yield 14 / 20 / 37 / 40 ms, real poll intervals.
    boot_only = bp.poll_dt_steps_ms(pre)
    assert boot_only.n_domain_breaks == 0 and boot_only.n_backwards == 0
    assert all(10.0 <= s <= 50.0 for s in boot_only.steps_ms)
    assert bp.poll_dt_ms_median(pre) is not None


def test_no_stamp_advance_at_all_reports_None_rather_than_a_number():
    """A stream that never advances is not a 0 ms poll. ``None`` is what the
    admission gate reads as ``poll_cadence_unmeasured``; a zero would sail
    through the floor comparison as a very fast sensor."""
    frozen = [bp.SensorSample(t=THROW_T + k * 0.01, held=True, raw=True,
                              valid=True, stamp=THROW_T)
              for k in range(50)]
    assert bp.poll_dt_ms_median(frozen) is None
    assert bp.poll_dt_steps_ms(frozen).steps_ms == ()


def test_edges_skip_invalid_samples_rather_than_reading_them_as_a_level():
    """UNKNOWN is not EMPTY. Letting an invalid sample act as a level would mint
    a departure edge out of a telemetry hiccup."""
    samples = [bp.SensorSample(0.0, True, True, True),
               bp.SensorSample(0.1, False, False, False),   # UNKNOWN
               bp.SensorSample(0.2, True, True, True)]
    assert bp.edges(samples) == ((), ())


# ── 3. The latch ──────────────────────────────────────────────────────────────

class _Ball(object):
    def __init__(self, ball_id, status, destination='', tracking=1):
        self.id = ball_id
        self.status = status
        self.destination = destination
        self.tracking = tracking


IN_FLIGHT = 1
CAUGHT = 2
CONFIRMED = 1
ANNOUNCED = 0


def test_the_latch_prefers_a_destination_tagged_candidate():
    got = bp.latch_announced_ball(
        [_Ball(7, IN_FLIGHT), _Ball(9, IN_FLIGHT, 'jugglebot')],
        robot_name='jugglebot', announced_id=None, preexisting_ids=(),
        untagged_latch=False, in_flight_status=IN_FLIGHT)
    assert got == (9, False)


def test_the_latch_excludes_ids_already_in_flight_at_goal_start():
    """Attempt 5 of the 2026-07-23 re-test latched a phantom untagged track that
    predated the throw and rode it to a wrong MISSED verdict. Our ball cannot be
    airborne before our own announcement's throw_time."""
    got = bp.latch_announced_ball(
        [_Ball(3, IN_FLIGHT)], robot_name='jugglebot', announced_id=None,
        preexisting_ids=(3,), untagged_latch=False,
        in_flight_status=IN_FLIGHT)
    assert got == (None, False)


def test_an_untagged_latch_is_provisional_and_a_tagged_track_displaces_it():
    got = bp.latch_announced_ball(
        [_Ball(7, IN_FLIGHT), _Ball(9, IN_FLIGHT, 'jugglebot')],
        robot_name='jugglebot', announced_id=7, preexisting_ids=(),
        untagged_latch=True, in_flight_status=IN_FLIGHT)
    assert got == (9, False)


def test_a_tagged_latch_is_NOT_displaced():
    got = bp.latch_announced_ball(
        [_Ball(9, IN_FLIGHT, 'jugglebot'), _Ball(11, IN_FLIGHT, 'jugglebot')],
        robot_name='jugglebot', announced_id=9, preexisting_ids=(),
        untagged_latch=False, in_flight_status=IN_FLIGHT)
    assert got == (9, False)


def test_the_latch_ignores_a_ball_bound_for_another_robot():
    got = bp.latch_announced_ball(
        [_Ball(9, IN_FLIGHT, 'ball_butler')], robot_name='jugglebot',
        announced_id=None, preexisting_ids=(), untagged_latch=False,
        in_flight_status=IN_FLIGHT)
    assert got == (None, False)




def test_the_node_calls_the_extracted_latch():
    """Grep-proof for D11: neither the FSM nor the skill node may keep a
    private copy.

    Imported at module scope in both callers, so an accidental re-inlining
    shows up as this import going unused — and this assertion goes red the
    moment the symbol stops being theirs. `reload_coordinator_node` still
    imports it from `jugglebot.toss_record` until R3-f2 re-points it (the
    same rung); `skill_node` was re-pointed to `ball_possession` here.
    """
    import importlib
    skill_node = importlib.import_module('jugglebot.skill_node')
    # The skill node reaches the rule through the per-release flight-latch
    # layer (2026-09-16) rather than calling `latch_announced_ball` directly;
    # the layer itself lives here and IS that call, so re-inlining either half
    # still shows up as this assertion going red.
    assert skill_node.advance_flight_latches is bp.advance_flight_latches
    assert skill_node.flight_in_progress is bp.flight_in_progress
    assert skill_node.FlightLatch is bp.FlightLatch


# ── 3b. The flight in progress (per-release latches, 2026-09-16) ─────────────
#
# Instants below are MEASURED, from bag `2026-09-16_16-22-22` (armB-090 attempt
# 1): announcements at 416.861 / 417.262 / 418.439 for releases at 417.361 /
# 418.511 / 419.661 (add 1789540000 to each), tracker ids 48 / 49 / 50, each
# minted at its announcement and flipped to IN_FLIGHT at its own release
# (measured skew 3 ms).

def test_an_unreleased_latch_is_never_the_flight_in_progress():
    """The regression, at the layer that owns it: id 49 is already reported
    IN_FLIGHT+CONFIRMED (the tracker had it announced-and-seeded), but its
    release is still 0.6 s away, so the flight in progress is 48's."""
    latches = (bp.FlightLatch(417.361, announced_id=48),
               bp.FlightLatch(418.511, announced_id=49))
    balls = [_Ball(48, IN_FLIGHT, 'jugglebot'), _Ball(49, IN_FLIGHT, 'jugglebot')]
    assert bp.flight_in_progress(
        balls, latches=latches, now_s=417.9, in_flight_status=IN_FLIGHT,
        confirmed_tracking=CONFIRMED) == 48
    assert bp.flight_in_progress(
        balls, latches=latches, now_s=418.6, in_flight_status=IN_FLIGHT,
        confirmed_tracking=CONFIRMED) == 49


def test_an_ended_flight_yields_nothing_rather_than_an_earlier_latch():
    """No fallback, deliberately: a release only happens once the previous
    flight of that schedule ball is over, so an earlier latch's landing
    describes a finished flight — the very contamination this closes."""
    latches = (bp.FlightLatch(417.361, announced_id=48),
               bp.FlightLatch(418.511, announced_id=49))
    balls = [_Ball(48, IN_FLIGHT, 'jugglebot'), _Ball(49, CAUGHT, 'jugglebot')]
    assert bp.flight_in_progress(
        balls, latches=latches, now_s=418.9, in_flight_status=IN_FLIGHT,
        confirmed_tracking=CONFIRMED) is None


def test_an_unconfirmed_track_yields_nothing():
    latches = (bp.FlightLatch(417.361, announced_id=48),)
    balls = [_Ball(48, IN_FLIGHT, 'jugglebot', tracking=ANNOUNCED)]
    assert bp.flight_in_progress(
        balls, latches=latches, now_s=417.9, in_flight_status=IN_FLIGHT,
        confirmed_tracking=CONFIRMED) is None


def test_advance_does_not_latch_a_release_that_has_not_happened():
    latches = (bp.FlightLatch(418.511),)
    got = bp.advance_flight_latches(
        [_Ball(48, IN_FLIGHT, 'jugglebot')], robot_name='jugglebot',
        latches=latches, now_s=417.9, in_flight_status=IN_FLIGHT)
    assert got[0].announced_id is None
    # ... and does at its release (the tracker's own flip instant).
    got = bp.advance_flight_latches(
        [_Ball(49, IN_FLIGHT, 'jugglebot')], robot_name='jugglebot',
        latches=latches, now_s=418.514, in_flight_status=IN_FLIGHT)
    assert got[0].announced_id == 49


def test_a_sibling_latch_does_not_lose_its_id_to_the_next_release():
    """Measured overlap at 419.680: ids 49 (caught, marker still visible, so
    still IN_FLIGHT) and 50 (just released) are BOTH destination-tagged and
    IN_FLIGHT. `latch_announced_ball` prefers the first tagged candidate it
    sees, which is 49 — so without the sibling exclusion the third release
    would latch the second flight's id."""
    latches = (bp.FlightLatch(418.511, announced_id=49),
               bp.FlightLatch(419.661, preexisting=(48,)))
    got = bp.advance_flight_latches(
        [_Ball(49, IN_FLIGHT, 'jugglebot'), _Ball(50, IN_FLIGHT, 'jugglebot')],
        robot_name='jugglebot', latches=latches, now_s=419.680,
        in_flight_status=IN_FLIGHT)
    assert [l.announced_id for l in got] == [49, 50]


def test_the_latch_queue_is_bounded_and_ordered_by_release():
    got = bp.advance_flight_latches(
        [], robot_name='jugglebot',
        latches=tuple(bp.FlightLatch(t) for t in (5.0, 1.0, 4.0, 2.0, 3.0)),
        now_s=0.0, in_flight_status=IN_FLIGHT)
    assert [l.t_release_s for l in got] == [2.0, 3.0, 4.0, 5.0]
    assert len(got) == bp.MAX_FLIGHT_LATCHES


# ── 4. The join ───────────────────────────────────────────────────────────────

def test_a_matched_pair_joins_and_reports_its_residual():
    joined = bp.join([{'announce_throw_time_ros': 10.0, 'toss_uid': 'd',
                       'goal_throw_delay_s': 5.0}],
                     [{'announce_throw_time_ros': 10.002,
                       'sensor_valid_frac': 1.0}])
    assert len(joined) == 1
    assert joined[0]['record_provenance'] == bp.PROV_BOTH
    assert joined[0]['goal_throw_delay_s'] == 5.0
    assert joined[0]['sensor_valid_frac'] == 1.0
    assert joined[0]['join_residual_ms'] == pytest.approx(2.0)


def test_outside_the_tolerance_the_halves_do_not_join():
    joined = bp.join([{'announce_throw_time_ros': 10.0}],
                     [{'announce_throw_time_ros': 10.02}])
    assert [r['record_provenance'] for r in joined] == [bp.PROV_MINED,
                                                        bp.PROV_DECLARED]


def test_a_mined_only_row_is_a_full_record():
    """The inversion that makes the whole design work: the miner must produce a
    complete record ALONE, in degraded form. It is what lets the three
    2026-08-10 bags become a corpus before any of this was wired."""
    joined = bp.join([], [{'announce_throw_time_ros': 10.0,
                           'sensor_valid_frac': 1.0, 'label': 'CAUGHT'}])
    assert set(joined[0]) == set(bp.FIELD_NAMES)
    assert joined[0]['record_provenance'] == bp.PROV_MINED
    assert joined[0]['label'] == 'CAUGHT'
    assert joined[0]['toss_uid'] == 'mined-10.000'


def test_an_unmatched_declaration_is_kept_not_dropped():
    """A silently missing row is how a replay overstates its own agreement, and
    an unmatched declaration is itself a finding: either the bag lost the
    announcement or the toss never announced."""
    joined = bp.join([{'announce_throw_time_ros': 10.0, 'toss_uid': 'd'}], [])
    assert [r['record_provenance'] for r in joined] == [bp.PROV_DECLARED]


def test_a_declared_vs_mined_conflict_is_recorded_never_resolved():
    """``D+M`` fields exist precisely so the halves can be cross-checked.
    Resolving a conflict silently throws away the only signal the redundancy
    buys."""
    joined = bp.join([{'announce_throw_time_ros': 10.0,
                       'tilt_map_version': 'A', 'toss_uid': 'd'}],
                     [{'announce_throw_time_ros': 10.0,
                       'tilt_map_version': 'B'}])
    assert joined[0]['disagreement'] == [
        "tilt_map_version: declared='A' mined='B'"]
    assert joined[0]['tilt_map_version'] == 'A'


def test_the_join_key_itself_is_not_reported_as_a_disagreement():
    """The halves are matched ON the key, within tolerance, so any difference is
    by definition the join residual — which has its own field. Reporting it here
    would duplicate ``join_residual_ms`` on every row and bury the conflicts that
    mean something."""
    joined = bp.join([{'announce_throw_time_ros': 10.0, 'toss_uid': 'd'}],
                     [{'announce_throw_time_ros': 10.003}])
    assert joined[0]['disagreement'] == []


def test_the_declaration_wins_on_a_declared_only_field():
    """The node is the only witness of what it commanded."""
    joined = bp.join([{'announce_throw_time_ros': 10.0, 'toss_uid': 'd',
                       'event_vel_mps': 3.91}],
                     [{'announce_throw_time_ros': 10.0,
                       'event_vel_mps': 99.0}])
    assert joined[0]['event_vel_mps'] == 3.91


def test_each_declaration_joins_at_most_once():
    """Two mined rows inside one tolerance would otherwise both claim the same
    declaration and the corpus would double-count a toss."""
    joined = bp.join([{'announce_throw_time_ros': 10.0, 'toss_uid': 'd'}],
                     [{'announce_throw_time_ros': 10.001},
                      {'announce_throw_time_ros': 10.002}])
    assert sorted(r['record_provenance'] for r in joined) == [
        bp.PROV_BOTH, bp.PROV_MINED]



# ── Physical sanity ───────────────────────────────────────────────────────────

def test_the_departure_window_is_far_wider_than_the_measured_shift():
    """A pinned threshold has to be justified by the measurement it covers.
    Measured worst-case release shift is +212 ms; the window is 1.00 s before the
    landing clamp and ~0.60 s after it on the shipped 0.8 s flight."""
    assert bp.DEPARTURE_WINDOW_S >= 4.0 * 0.212
    clamped = FLIGHT_S - float(hw.JB_BD_ARRIVAL_LEAD_S)
    assert clamped >= 2.5 * 0.212
    assert math.isclose(RELEASE_GUARD_S, 0.30)


# ── 4. The cadence clamp (C-POSSESS-1 § 3.4; census D1/D2) ────────────────────
#
# The corpus label and the live verdict are two implementations of one definition
# of "caught". They must clamp the same way from the same constants, or the
# offline miner — which is the R3 rung's gate instrument ("score the miner, not
# the console") — will disagree with the machine it is scoring.
#
# Rung instants, never round numbers:
#   R3   dwell 1.50 s, flight 0.80 s   (the rung these fixes must land before)
#   R5'  dwell 0.49 s, flight 0.4949 s (the tuning-phase operating target)

R3_DWELL_S = 1.50
R5P_DWELL_S, R5P_FLIGHT_S = 0.49, 0.4949
SEAT_DT_S = 0.30                       # inside the measured +137…+798 ms band


def test_a_good_cycle_is_not_labelled_bounced_once_the_dwell_shrinks():
    """CENSUS D1 — the inversion, and the fix, in one test.

    ``retention_window_s`` is 1.50 s and its UPPER justification was written in
    the YAML as *"shorter than MIN_TOSS_THROW_DELAY_S (3.5 s) by 2.3x, so a
    legitimate throw can never read as a bounce-out"*. That floor is retired. At
    the R3 dwell the ball leaves the cup 1.50 s after the landing for OUR OWN
    next throw, which sits inside the unclamped window, so gate 4 mints BOUNCED
    on a perfect cycle — and with ``on_empty_cup: RELOAD`` that same route asks
    BallButler to throw a second ball at a full cup.

    Both halves are asserted: without the clamp the defect reproduces, with it
    the label is CAUGHT. Asserting only the fixed behaviour would also pass
    against an implementation that had merely widened something."""
    next_release = LANDING_T + R3_DWELL_S
    samples = stream(catch_dt=SEAT_DT_S, drop_after=R3_DWELL_S - SEAT_DT_S)

    def under(**kw):
        return bp.label_from_sensor(samples, throw_time=THROW_T,
                                    landing_time=LANDING_T, windows=WINDOWS,
                                    **kw)

    assert under().label == bp.LABEL_BOUNCED                    # the defect
    got = under(next_release_time=next_release,
                next_landing_time=next_release + FLIGHT_S)
    assert got.label == bp.LABEL_CAUGHT
    assert got.confidence == 1.0


def test_a_real_bounce_out_survives_the_clamp():
    """The clamp excludes the announced throw window, NOT everything after the
    arrival. Trading the D1 mislabel for its mirror image — every bounce-out read
    as a catch — would re-open the trap C-POSSESS-1 § 7 spent a whole section
    accepting before the sensor closed it."""
    next_release = LANDING_T + R3_DWELL_S
    got = bp.label_from_sensor(
        stream(catch_dt=SEAT_DT_S, drop_after=0.20), throw_time=THROW_T,
        landing_time=LANDING_T, windows=WINDOWS,
        next_release_time=next_release,
        next_landing_time=next_release + FLIGHT_S)
    assert got.label == bp.LABEL_BOUNCED


def test_an_unobservable_retention_is_declared_not_assumed():
    """C-POSSESS-1 § 3.4's third clause, and the honest half of the change.

    At R5' the seat edge lands +0.30 s after the landing and the next release
    only +0.49 s after it, so the horizon closes BEFORE the ball is even seated.
    Retention was never observable — the physics removes it, not the clamp (the
    debounced fall lag alone is ~241 ms). The label is still CAUGHT (the ball
    demonstrably arrived), but the corpus must be able to tell "held through
    retention" from "we did not look", so the confidence drops and the reason
    says so. A fitter that treats every CAUGHT alike would otherwise inherit an
    unmarked change in what CAUGHT means, at exactly the cadence the fit runs
    at."""
    next_release = LANDING_T + R5P_DWELL_S
    assert LANDING_T + SEAT_DT_S > next_release - RELEASE_GUARD_S, (
        'premise: the seat edge lands PAST the horizon at this cadence')
    samples = stream(catch_dt=SEAT_DT_S,
                     drop_after=R5P_DWELL_S - SEAT_DT_S)
    got = bp.label_from_sensor(samples, throw_time=THROW_T,
                               landing_time=LANDING_T, windows=WINDOWS,
                               next_release_time=next_release,
                               next_landing_time=next_release + R5P_FLIGHT_S)
    assert got.label == bp.LABEL_CAUGHT
    assert got.confidence == 0.5
    assert 'NOT OBSERVABLE' in got.reason


def test_the_arrival_search_stops_where_the_next_cycles_begins():
    """CENSUS D2. At R5' the cycle PERIOD is ``dwell + T = 0.985 s``, under the
    1.50 s arrival window, so the NEXT cycle's seat edge falls inside this
    cycle's unclamped search and both rows can claim it. Clamped, this window
    closes exactly where the next one opens.

    Only the PERIOD to the next landing matters here, so this cycle keeps the
    harness's own throw/landing pair; the R5' numbers set where the next cycle
    lands."""
    period_s = R5P_DWELL_S + R5P_FLIGHT_S              # 0.985 s
    next_release = LANDING_T + R5P_DWELL_S
    next_land = LANDING_T + period_s
    assert period_s < WINDOWS.arrival_window_s, 'premise: they overlap'
    # This cycle MISSED; only the NEXT cycle's ball ever seats.
    samples = stream(catch_dt=period_s + SEAT_DT_S)

    def under(**kw):
        return bp.label_from_sensor(samples, throw_time=THROW_T,
                                    landing_time=LANDING_T, windows=WINDOWS,
                                    **kw)

    assert under().label == bp.LABEL_CAUGHT                     # the edge, stolen
    assert under(next_release_time=next_release,
                 next_landing_time=next_land).label == bp.LABEL_MISSED


# ── The arrival BOUNDARY (C-POSSESS-1 § 3.4 clauses C.1 / C.2, 2026-08-23) ────
#
# `arr_hi` used to be `next_landing_time - arrival_lead_s` outright, which pays
# the NEXT row's pre-landing guard out of THIS row's measured arrival band.
# The two scenario periods are DERIVED from the constants, not typed. They used
# to be the R5' clamp pin (0.9849 s) and the deferred R6 fork (0.7529 s), the
# reachable cadences while the band ceiling was 0.800 s; the 2026-08-24
# post-FW-14 re-measure took the ceiling to 0.560 s and both rungs walked out of
# both clauses. Re-typing a fresh pair would only schedule the same rot for the
# next re-measure, so each clause's period comes from its own arithmetic:
#
#   * C.1 bites for a period in [BAND_MAX, BAND_MAX + arrival_lead) — where
#     `b - lead` closes inside the band and the fixed boundary is `a + BAND_MAX`;
#   * C.2 bites for a period BELOW BAND_MAX, where no rule serves both balls.
#
# Both are SYNTHETIC at the collapsed ceiling (they need dwells of ~0.16 s and
# ~0.001 s against a 0.487 s hand floor). That is the re-measure's headline, not
# a weakening: these are invariants over the WINDOW, and an `arrival_window_s`
# configured under the band reaches C.2 from a direction cadence no longer can.

R5P_PERIOD_S = R5P_DWELL_S + R5P_FLIGHT_S             # 0.9849 s — a real rung
#: A period that puts the SUPERSEDED `b - lead` clamp inside the band (C.1).
C1_PERIOD_S = ARRIVAL_BAND_MAX_S + WINDOWS.arrival_lead_s / 2.0
#: A period below the band ceiling itself, where C.2 is the operative half.
R6_PERIOD_S = ARRIVAL_BAND_MAX_S * 0.9


def test_a_catch_in_the_band_TAIL_is_not_labelled_missed():
    """C-POSSESS-1.C.1, the corpus half — a false MISSED minted off a SCHEDULE.

    The premise comes from the constants, not from a chosen number: at a period
    inside [BAND_MAX, BAND_MAX + lead) the superseded clamp closes before the
    band ceiling. A catch seating in that sliver is a REAL catch the search never
    saw, and the corpus called it a MISS — which is the label the aim fit weights
    most heavily.

    Both halves asserted: the defect reproduces against the shipped instant, the
    fix labels it CAUGHT and keeps the catch-event field."""
    next_land = LANDING_T + C1_PERIOD_S
    shipped_close = C1_PERIOD_S - WINDOWS.arrival_lead_s
    assert shipped_close < ARRIVAL_BAND_MAX_S, (
        'premise: at this period the shipped clamp closes inside the band')
    # In the sliver, and on the 100 Hz sample grid. DERIVED: the sliver moves
    # with ARRIVAL_BAND_MAX_S and a typed offset would silently leave it.
    catch_dt = round(0.5 * (shipped_close + ARRIVAL_BAND_MAX_S), 2)
    assert shipped_close < catch_dt < ARRIVAL_BAND_MAX_S
    # …and the tightest PUBLISHED rung does not reach this state at all, which is
    # what the 2026-08-24 band re-measure bought and is worth pinning rather than
    # asserting in prose: at the R5' clamp pin the boundary sits above the band
    # ceiling, so no rung on the ladder can lose a band tail.
    assert bp.arrival_boundary_t(LANDING_T, LANDING_T + R5P_PERIOD_S,
                                 WINDOWS.arrival_lead_s) \
        >= LANDING_T + ARRIVAL_BAND_MAX_S
    samples = stream(catch_dt=catch_dt)

    # THE DEFECT: the shipped instant as a fixed window is arithmetically the
    # same search C-POSSESS-1.C built, and it cannot see the edge.
    narrow = bp.SensorWindows(arrival_lead_s=WINDOWS.arrival_lead_s,
                              arrival_window_s=shipped_close,
                              retention_window_s=WINDOWS.retention_window_s)
    missed = bp.label_from_sensor(samples, throw_time=THROW_T,
                                  landing_time=LANDING_T, windows=narrow)
    assert missed.fields['t_catch_deb_ros'] is None

    got = bp.label_from_sensor(samples, throw_time=THROW_T,
                               landing_time=LANDING_T, windows=WINDOWS,
                               next_landing_time=next_land)
    assert got.label == bp.LABEL_CAUGHT
    assert got.fields['t_catch_deb_ros'] == pytest.approx(
        LANDING_T + catch_dt, abs=0.011)


def test_a_row_cannot_reach_back_for_the_PREVIOUS_balls_seat_edge():
    """The abutment, from the other end — and why the closing could move at all.

    Moving ``arr_hi`` later without moving the next row's ``arr_lo`` with it hands
    one edge to two rows: the census-D2 fault, re-created by its own fix. Here the
    PREVIOUS ball seats late, which at a C.1 period falls inside this row's
    unclamped pre-landing lead. Unclamped this row reads CAUGHT off its
    neighbour's ball; clamped at the shared boundary it reads the MISS it
    actually was."""
    prev_land = LANDING_T - C1_PERIOD_S
    # The shared boundary, relative to THIS landing, and an edge halfway between
    # it and the unclamped opening — derived, so the scenario follows the band.
    boundary_dt = bp.arrival_boundary_t(
        prev_land, LANDING_T, WINDOWS.arrival_lead_s) - LANDING_T
    stolen_dt = round(0.5 * (boundary_dt - WINDOWS.arrival_lead_s), 3)
    assert -WINDOWS.arrival_lead_s < stolen_dt < boundary_dt < 0.0, (
        'premise: the neighbour edge sits inside the unclamped opening but '
        'outside the clamped one')
    samples = stream(catch_dt=stolen_dt, dt=0.001)

    def under(**kw):
        return bp.label_from_sensor(samples, throw_time=THROW_T,
                                    landing_time=LANDING_T, windows=WINDOWS,
                                    **kw)

    assert under().label == bp.LABEL_CAUGHT                    # the edge, stolen
    assert under(prev_landing_time=prev_land).label == bp.LABEL_MISSED


def test_a_band_clamped_search_declares_unknown_rather_than_missed():
    """C-POSSESS-1.C.2 — the half no boundary rule can fix.

    Below an ``ARRIVAL_BAND_MAX_S`` period the next ball lands before this one's
    band has closed, so both balls cannot have their whole band. MISSED is a
    POSITIVE
    claim, and a search that stopped short of the evidence has not earned it. The
    corpus says UNKNOWN and names the cause, so a fitter can tell "the ball
    missed" from "the schedule looked away" — which is the whole reason gate 1
    exists, one gate up."""
    next_land = LANDING_T + R6_PERIOD_S
    assert R6_PERIOD_S < ARRIVAL_BAND_MAX_S, (
        'premise: the next ball lands before this band closes')
    samples = stream(catch_dt=None)

    def under(**kw):
        return bp.label_from_sensor(samples, throw_time=THROW_T,
                                    landing_time=LANDING_T, windows=WINDOWS,
                                    **kw)

    assert under().label == bp.LABEL_MISSED        # a search that DID watch out
    got = under(next_landing_time=next_land)
    assert got.label == bp.LABEL_UNKNOWN
    assert 'band clamped' in got.reason
    assert got.confidence == 0.0


def test_the_clamp_is_absent_by_default_so_a_single_toss_is_unchanged():
    """A single ``Toss``, and a session's LAST cycle, have nothing scheduled
    after them — the honest horizon is the shipped fixed one. ``None`` and
    ``NaN`` must behave identically: a NaN horizon compares False against
    everything and would silently disable the clamp it was meant to apply."""
    nan = float('nan')
    base = label(catch_dt=0.4)
    for kw in ({}, {'next_release_time': nan, 'next_landing_time': nan},
               {'next_release_time': None, 'next_landing_time': None}):
        got = bp.label_from_sensor(stream(catch_dt=0.4), throw_time=THROW_T,
                                   landing_time=LANDING_T, windows=WINDOWS,
                                   **kw)
        assert (got.label, got.reason, got.confidence) == (
            base.label, base.reason, base.confidence)


def test_the_corpus_and_the_live_verdict_share_ONE_arrival_boundary(monkeypatch):
    """The offline corpus (``label_from_sensor``) and the live source
    (``HandBallSensorSource``) must compute the SAME arrival boundary from the
    same landing pair — a second, independently-written boundary is exactly how
    the corpus and the robot drift onto different notions of "arrived"."""
    calls = []
    real = bp.arrival_boundary_t

    def spy(*a):
        calls.append(a)
        return real(*a)

    monkeypatch.setattr(bp, 'arrival_boundary_t', spy)
    bp.label_from_sensor([], throw_time=100.0, landing_time=100.8,
                         windows=WINDOWS, next_landing_time=101.9,
                         prev_landing_time=99.7)
    n_label = len(calls)
    src = bp.HandBallSensorSource(
        arrival_lead_s=WINDOWS.arrival_lead_s,
        arrival_window_s=WINDOWS.arrival_window_s,
        retention_window_s=WINDOWS.retention_window_s, stale_s=0.5)
    src._window(100.8, next_landing_t=101.9, prev_landing_t=99.7)
    assert n_label == 2 and len(calls) == 4
    assert calls[:2] == calls[2:]


def test_every_joined_row_encodes():
    """Every row ``join()`` emits stays inside the pinned schema — a field
    name outside ``FIELD_NAMES`` or a missing ``schema`` tag is exactly the
    drift ``test_fields_are_pinned`` (module-level) exists to catch, restated
    on the join path specifically."""
    joined = bp.join([{'announce_throw_time_ros': 10.0, 'toss_uid': 'd',
                       'action': 'toss', 'outcome': 'CAUGHT'}],
                     [{'announce_throw_time_ros': 10.0,
                       'land_err_norm_mm': float('nan')}])
    for row in joined:
        assert set(row) <= set(bp.FIELD_NAMES)
        assert row['schema'] == bp.SCHEMA
