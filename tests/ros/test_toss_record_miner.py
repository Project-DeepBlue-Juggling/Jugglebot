"""The toss-record miner, against the MEASURED reference sitting.

Probe: ``tools/probes/toss_record_miner.py``. Fixture:
``tests/ros/toss_record_fixtures.py`` (generated — do not hand-edit).
Plan: ``plans/active/toss-selftuning.md`` §§ 3.3, 3.4.
Logbook: ``logbook/2026-08-10-toss-selftuning-build.md``.

WHY A BAG-BACKED TEST EXISTS ALONGSIDE tests/motion/test_toss_record.py
-----------------------------------------------------------------------
Those tests drive the labeller with SYNTHESISED streams placed at the measured
instants — they pin the RULE. This one asserts that the miner, pointed at a real
sitting, reproduces the census a human counted by hand: **39 departures,
38 catches, 3 seat-then-leaves in 2026-08-10_16-30-44**. The two fail in
different ways. A broken rule breaks the first; a rule that only works on tidy
synthetic data breaks the second.

THE SHAPE OF THE ACCEPTANCE
---------------------------
The bag is machine-local and gitignored, so the bag-backed tests SKIP on any
machine without it — including a fresh clone and CI. The committed fixture is
what carries the evidence across that boundary, which is the whole point of the
``tests/ros/possession_fixtures.py`` pattern this follows: the numbers stay
reviewable, diffable and citable when the 270 MB of mcap is not there.

The reference bag also happens to be the strongest possible test of the § 3.3
inversion — *the miner must produce the whole record ALONE, in degraded form* —
because it predates every line of this phase. It carries no ``/toss/record``, no
``/rosout``, no ``/catch/*`` and no action feedback, and the miner still produces
a full corpus from it, every row ``mined-only``.
"""

from __future__ import annotations

import importlib.util
import os

import pytest

import jugglebot.hardware_config as hw
from jugglebot import toss_record as tr

from tests.ros import toss_record_fixtures as fx

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
_PROBE_PATH = os.path.join(_REPO, 'tools', 'probes', 'toss_record_miner.py')
_BAG_ROOT = os.path.expanduser('~/Desktop/rosbags')
_BAG_DIR = os.path.join(_BAG_ROOT, fx.REFERENCE_BAG)
_HAVE_BAG = os.path.isdir(_BAG_DIR)


@pytest.fixture(scope='module')
def miner():
    spec = importlib.util.spec_from_file_location('toss_record_miner',
                                                  _PROBE_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope='module')
def mined(miner):
    """The reference bag, read SENSOR-ONLY, once per module.

    ``sensor_only`` skips the ``/mocap_data`` + ``/balls`` pass. That is not a
    shortcut around a weak assertion — every claim below is a hand-telemetry
    claim, and the heavy pass costs *minutes* on a 270 MB sitting, which would
    land inside `./run_tests.sh` in exchange for nothing. The mocap block's own
    behaviour on this bag (empty, because the ball is not in the descending band
    over the cup) is recorded in the logbook and cross-checked against the
    shipped ``ball_arrival_offset.py``, not re-derived here every commit.
    """
    if not _HAVE_BAG:
        pytest.skip('reference rosbag is machine-local and gitignored')
    data = miner.read_bag(_BAG_DIR, sensor_only=True)
    rows = tr.join(data.declarations, miner.mine_bag(data))
    return data, rows, miner.ledger(data.hand)


# ── The instrument's own acceptance (no bag) ──────────────────────────────────

def test_the_self_check_is_clean(miner):
    """Two-sided cases at the measured instants: every ACCEPT has a REFUSE twin,
    so a silently widened window fails here rather than at the bench."""
    assert miner.self_check() == 0


def test_the_miner_mirrors_the_nodes_windows(miner):
    """The drift guard that makes the offline label mean the same thing as the
    live verdict. A bench instrument that scores a capture against windows the
    robot never ran agrees with the robot only by coincidence."""
    w = miner.make_windows()
    assert w.arrival_lead_s == pytest.approx(float(hw.JB_BD_ARRIVAL_LEAD_S))
    assert w.arrival_window_s == pytest.approx(float(hw.JB_BD_ARRIVAL_WINDOW_S))
    assert w.retention_window_s == pytest.approx(
        float(hw.JB_BD_RETENTION_WINDOW_S))


def test_the_miner_uses_the_production_label_and_latch(miner):
    """D11, made grep-proof. A miner that grew its own copy of either would drift
    from the node and nobody would notice until a map had been fitted on the
    wrong labels."""
    assert miner.label_from_sensor is tr.label_from_sensor
    assert miner.latch_announced_ball is tr.latch_announced_ball
    assert miner.edges is tr.edges


def test_the_ball_status_constant_matches_the_node(miner):
    """Mirrored rather than imported (importing the node drags rclpy in), so it
    needs a pin. A wrong IN_FLIGHT value silently latches nothing and every row
    reports no tracker evidence."""
    node_src = os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot', 'jugglebot',
                            'reload_coordinator_node.py')
    with open(node_src) as fh:
        text = fh.read()
    assert '_BALL_STATUS_IN_FLIGHT = {}'.format(
        miner.BALL_STATUS_IN_FLIGHT) in text


def test_the_default_fit_plane_is_the_cup_plane_not_the_probe_default(miner):
    """``ball_arrival_offset``'s 1000 mm default is wrong for this corpus, and
    wrong INVISIBLY: for a vertical 8a toss ``v_xy ~ 0``, so a mis-set plane looks
    perfect right up until a displaced throw, at which point every node is biased
    in a direction correlated with displacement (plan § 7 R1)."""
    expected = (float(hw.GEOM_INITIAL_HEIGHT_MM)
                + float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM)
                + float(hw.HAND_CATCH_OFFSET_MM))
    assert miner.DEFAULT_PLANE_MM == pytest.approx(expected)
    assert miner.DEFAULT_PLANE_MM != 1000.0


def _plane_rows():
    """One row past the 5 mm tolerance, one inside it. Both start ADMITTED, so
    the assertions below distinguish 'refused' from 'never admitted'."""
    return [{'toss_uid': 'a', 'catch_point_global_mm': [0.0, 0.0, 500.0],
             'land_plane_mm': 400.0, 'usable_for_aim_fit': True,
             'usable_for_speed_fit': True, 'usable_for_timing_fit': True,
             'usable_for_release_fit': True, 'excluded_reason': None},
            {'toss_uid': 'b', 'catch_point_global_mm': [0.0, 0.0, 500.0],
             'land_plane_mm': 501.0, 'usable_for_aim_fit': True,
             'usable_for_speed_fit': True, 'usable_for_timing_fit': True,
             'usable_for_release_fit': True, 'excluded_reason': None}]


def test_a_declared_plane_mismatch_is_reported_loudly(miner):
    """One line per row, never averaged away. A corpus scoring arrivals at the
    wrong height is the failure that looks like data."""
    problems = miner.check_declared_planes(_plane_rows())
    assert len(problems) == 1
    assert 'a' in problems[0]


def test_a_declared_plane_mismatch_is_REFUSED_not_merely_reported(miner):
    """AUDIT FIX 2026-08-11. Plan § 7 R1 says the miner REFUSES a plane more than
    5 mm from ``catch_point_global_mm[2]``; it was only printing, and the
    mismatched rows kept ``usable_for_aim_fit: true``.

    A printed warning protects nothing downstream — the fit and the ``--jsonl``
    corpus both select on the flag, not on a human having read stdout — and R1's
    whole point is that this error is INVISIBLE in the residuals until a
    displaced throw, i.e. until exactly the moment the map is used in anger."""
    rows = _plane_rows()
    lines = miner.enforce_declared_planes(rows)
    assert len(lines) == 1 and 'a' in lines[0]
    assert rows[0]['usable_for_aim_fit'] is False
    assert rows[0]['usable_for_speed_fit'] is False
    assert 'plane_mismatch' in rows[0]['excluded_reason']
    # Inside the tolerance: untouched, including the reason string.
    assert rows[1]['usable_for_aim_fit'] is True
    assert rows[1]['excluded_reason'] is None


def test_the_plane_refusal_covers_the_TIMING_fit_too(miner):
    """AUDIT FIX 2026-08-12. This used to assert the opposite — that timing was
    SPARED, because "the timing fit reads sensor edges and release instants,
    which the fit plane has no bearing on". That was true when it was written and
    stopped being true the moment Phase 0a landed.

    Release-side timing is still plane-independent (``release_time_err_ms``
    compares the ascending backcast's RELEASE-plane crossing against the
    announced ``throw_time``). Arrival-side timing is not: ``t_arrival_fit_bag``
    IS the ``land_plane_mm`` crossing of the descending fit, and
    ``achieved_flight_s_mocap`` / ``flight_time_err_s`` are derived from it — at
    a 4.9 m/s arrival a 20 mm plane error is ~4 ms of flight time, the same order
    as the dispatch shift the timing fit exists to measure. One flag cannot be
    half-true."""
    rows = _plane_rows()
    miner.enforce_declared_planes(rows)
    assert rows[0]['usable_for_timing_fit'] is False
    assert rows[0]['usable_for_release_fit'] is False
    # And the row INSIDE the tolerance keeps every flag — the refusal has to
    # stay a refusal, not become a blanket.
    assert rows[1]['usable_for_timing_fit'] is True
    assert rows[1]['usable_for_release_fit'] is True


def test_a_mined_only_row_is_plane_checked_against_its_announcement(miner):
    """The check that only ever ran on JOINED rows, run on the corpus that has
    no declarations.

    ``enforce_declared_planes`` compares the fit plane against
    ``catch_point_global_mm``, which only a declaration carries — so on the whole
    reference corpus (every row ``mined-only``) the R1 refusal never fired once.
    The announcement's ``landing_position[2]`` is the same quantity from the same
    ``_toss_release_state``, so it answers the same question on a bag that
    predates ``/toss/record``, against the SAME constant.
    """
    windows = _miner_windows()

    class _Data(object):
        hand = []
        balls = []
        mocap = []
        link = []
        traj = []
        declarations = []
        fixture_cells = {}
        log_minus_stamp_s = 0.0

        def __init__(self, cup_z):
            self.announcements = [{
                'thrower': 'jugglebot', 'target': 'jugglebot',
                'throw_time': 100.0, 'landing_time': 100.8,
                'landing_position': [0.0, 0.0, cup_z],
                'initial_position': [0.0, 0.0, 802.34],
                'initial_velocity': [0.0, 0.0, 4000.0],
                'predicted_tof_sec': 0.8, 't_bag': 100.0}]

    plane = miner.DEFAULT_PLANE_MM
    ok = miner.mine_bag(_Data(plane + miner.PLANE_MISMATCH_TOL_MM - 0.5),
                        windows=windows)[0]
    bad = miner.mine_bag(_Data(plane + miner.PLANE_MISMATCH_TOL_MM + 0.5),
                         windows=windows)[0]
    assert 'plane_mismatch' not in (ok['excluded_reason'] or '')
    assert 'plane_mismatch' in bad['excluded_reason']
    for flag in ('usable_for_aim_fit', 'usable_for_speed_fit',
                 'usable_for_timing_fit', 'usable_for_release_fit'):
        assert bad[flag] is False, flag


def test_the_release_fit_flag_is_not_the_speed_fit_flag(miner):
    """The population behind the headline release-speed statistic, made
    expressible from the corpus.

    ``usable_for_speed_fit`` gates on the LANDING fit — ``land_xy_global_mm`` and
    its RMS — and the release-velocity backcast is a different fit over a
    different branch. The reference bag proves they dissociate in BOTH
    directions: rows with a clean 58/67-sample arc pair and no landing fit at
    all, and rows with a good landing fit whose ascending branch is 6 rows long
    and returns a release speed 1400 mm/s wrong. Before the flag the "clean fits"
    subset existed only as a filter somebody applied by hand in a report.
    """
    def row(**kw):
        r = {'label': 'CAUGHT', 't_departure_raw_ros': 1.0,
             'land_xy_global_mm': [0.0, 0.0], 'fit_rms_mm': 0.5}
        r.update(kw)
        miner._mark_usable(r)
        return r

    good = dict(backcast_fit_n=60, arrival_fit_n=67, release_vel_se_mms=7.0)
    assert row(**good)['usable_for_release_fit'] is True
    # No landing fit -> refused for speed, ADMITTED for release.
    no_land = row(land_xy_global_mm=None, **good)
    assert (no_land['usable_for_speed_fit'], no_land['usable_for_release_fit']) \
        == (False, True)
    # A short branch -> admitted for speed, REFUSED for release.
    short = row(backcast_fit_n=6, arrival_fit_n=None,
                release_vel_se_mms=382.0)
    assert (short['usable_for_speed_fit'], short['usable_for_release_fit']) \
        == (True, False)
    # Each condition alone is load-bearing.
    assert row(backcast_fit_n=miner.RELEASE_FIT_MIN_SAMPLES - 1,
               arrival_fit_n=67,
               release_vel_se_mms=7.0)['usable_for_release_fit'] is False
    assert row(backcast_fit_n=60, arrival_fit_n=67,
               release_vel_se_mms=miner.RELEASE_FIT_MAX_SE_MMS + 0.1
               )['usable_for_release_fit'] is False
    assert row(backcast_fit_n=60, arrival_fit_n=67,
               release_vel_se_mms=miner.RELEASE_FIT_MAX_SE_MMS
               )['usable_for_release_fit'] is True


def test_the_plane_refusal_preserves_an_existing_exclusion_reason(miner):
    """The reasons ACCUMULATE — a row excluded for a bad mocap fit AND a wrong
    plane must say both, or the operator fixes one and re-mines expecting a row
    that is still refused."""
    rows = _plane_rows()
    rows[0]['excluded_reason'] = 'mocap_fit_quality'
    miner.enforce_declared_planes(rows)
    assert rows[0]['excluded_reason'] == 'mocap_fit_quality,plane_mismatch'


def test_the_plane_refusal_runs_before_the_corpus_is_written(miner):
    """The flag the jsonl carries must be the ENFORCED one: a corpus written
    with ``usable_for_aim_fit: true`` on a wrong-plane row is precisely the
    artefact the refusal exists to stop, and it outlives the stdout it was
    reported on. Pinned structurally — the call sits between the join and
    ``write_outputs`` in ``main``."""
    src = open(_PROBE_PATH).read()
    join_at = src.index('rows = toss_record.join(')
    enforce_at = src.index('enforce_declared_planes(rows)', join_at)
    write_at = src.index('write_outputs(name, rows, led)', join_at)
    assert join_at < enforce_at < write_at


def test_the_self_toss_filter_drops_ball_butler_throws(miner):
    """A BB reload throw announces on the SAME topic and lands in the SAME cup.
    Pooling the two would mix Butler aim into a self-toss aim map."""
    anns = [{'thrower': 'jugglebot', 'target': 'jugglebot'},
            {'thrower': 'ball_butler', 'target': 'jugglebot'},
            {'thrower': 'jugglebot', 'target': 'ball_butler'}]
    assert miner.self_tosses(anns, 'jugglebot') == [anns[0]]


# ── The fixture itself ────────────────────────────────────────────────────────

def test_the_fixture_carries_its_own_regeneration_recipe():
    """``tools/probes/README.md``: a committed measurement needs a committed
    recipe, or the next person to change a window cannot re-baseline it."""
    src = os.path.join(os.path.dirname(__file__), 'toss_record_fixtures.py')
    with open(src) as fh:
        text = fh.read()
    assert 'toss_record_miner.py' in text
    assert '--emit-fixture' in text
    assert fx.REFERENCE_BAG in text


def test_the_fixture_pins_the_hand_mined_census():
    """THE acceptance number, stated where a human can check it: 39 departures,
    38 catches, 3 quick-drops. Counted by hand from the sitting."""
    assert fx.LEDGER['n_departures'] == 39
    assert fx.LEDGER['n_catches'] == 38
    assert len(fx.QUICK_DROP_DURATIONS_S) == 3
    assert fx.LEDGER['n_valid'] == fx.LEDGER['n_samples']


def test_the_fixture_records_the_debounce_asymmetry():
    """D12's evidence, carried across the bag boundary. Every departure's
    debounced edge lags its raw edge by 230-300 ms — 2.4x the plan's ~100 ms
    estimate — and that lag is the systematic error a timing fit taken off the
    debounced bit would carry."""
    assert fx.DEBOUNCE_FALL_LAG_MS
    assert min(fx.DEBOUNCE_FALL_LAG_MS) >= 200.0
    assert max(fx.DEBOUNCE_FALL_LAG_MS) <= 320.0


def test_the_fixture_departure_band_justifies_the_pinned_window():
    """A pinned threshold has to be justified by the measurement it covers.
    ``DEPARTURE_WINDOW_S`` must clear the worst measured release shift with real
    margin, or a slow session would silently start losing departures."""
    assert fx.DEPARTURE_DT_MS
    assert min(fx.DEPARTURE_DT_MS) > 0.0
    worst_s = max(fx.DEPARTURE_DT_MS) / 1e3
    assert tr.DEPARTURE_WINDOW_S >= 4.0 * worst_s


def test_the_fixture_records_the_measured_poll_cadence_gap():
    """The plan assumed a 50 Hz (20 ms) sensor poll. The bag says otherwise, and
    the record carries ``sensor_poll_dt_ms_median`` precisely so this is a
    measurement rather than an inherited assumption."""
    assert fx.POLL_DT_MS_MEDIAN > 2.0 * float(hw.JB_BD_CHECK_INTERVAL_MS)


def test_the_fixture_is_a_fully_degraded_capture():
    """The § 3.3 inversion, demonstrated: this bag predates ``/toss/record``, so
    every row is ``mined-only`` and the miner still produced a whole corpus."""
    assert fx.RECORD_PROVENANCE == (tr.PROV_MINED,)


def test_no_self_toss_is_unknown_in_the_reference_sitting():
    """``ball_held_valid`` was 100 % across the capture, so UNKNOWN must not
    appear. An UNKNOWN row here would mean the labeller went blind on a stream
    that never went invalid."""
    assert fx.LABELS.get(tr.LABEL_UNKNOWN, 0) == 0
    assert sum(fx.LABELS.values()) == fx.N_SELF_TOSSES


# ── The same census, against the real bag when it is present ─────────────────

@pytest.mark.skipif(not _HAVE_BAG,
                    reason='reference rosbag is machine-local and gitignored')
def test_the_miner_reproduces_the_hand_mined_census(mined):
    """THE ACCEPTANCE. Guards the fixture against the bag drifting out from under
    it AND the miner against an edge rule that drifts away from the hand count."""
    _data, _rows, led = mined
    assert led['n_departures'] == fx.LEDGER['n_departures']
    assert led['n_catches'] == fx.LEDGER['n_catches']
    assert led['n_samples'] == fx.LEDGER['n_samples']
    assert led['n_valid'] == fx.LEDGER['n_valid']
    assert sorted(round(d, 4) for _t, d in led['quick_drops']) == \
        sorted(fx.QUICK_DROP_DURATIONS_S)


@pytest.mark.skipif(not _HAVE_BAG,
                    reason='reference rosbag is machine-local and gitignored')
def test_every_mined_row_is_a_valid_record(mined):
    """A corpus row that does not round-trip is a row the fitter cannot read.
    Validated in the WIRE form, which is what a consumer sees."""
    _data, rows, _led = mined
    assert rows
    for row in rows:
        assert tr.validate(tr.decode(tr.encode(row))) == (), row['toss_uid']


@pytest.mark.skipif(not _HAVE_BAG,
                    reason='reference rosbag is machine-local and gitignored')
def test_the_reference_bag_lacks_the_topics_the_ONE_list_adds(mined):
    """The § 3.5 recording gap, demonstrated rather than asserted from the plan.

    This is the bag that motivates D18: the launch's record list did not carry
    ``/rosout``, ``/catch/*``, ``/trajectory/commanded_position`` or any action
    feedback, so none of them can be recovered from this sitting. A missing topic
    is unrecoverable after the fact — which is the whole argument for recording
    the superset.
    """
    data, _rows, _led = mined
    for topic in ('/toss/record', '/rosout', '/catch/pretilt_hold',
                  '/catch/armed', '/trajectory/commanded_position'):
        assert topic not in data.topics


@pytest.mark.skipif(not _HAVE_BAG,
                    reason='reference rosbag is machine-local and gitignored')
def test_every_self_toss_released_and_the_butler_throws_did_not(mined):
    """The departure edge is OURS. A ``ball_butler`` announcement lands in the
    same cup but its ball never left our hand, so it must produce no departure —
    the cleanest available check that the departure window is anchored on the
    right event."""
    data, rows, _led = mined
    windows = _miner_windows()
    for row in rows:
        assert row['t_departure_raw_ros'] is not None, row['toss_uid']
    butler = [a for a in data.announcements if a['thrower'] != 'jugglebot']
    assert butler
    for ann in butler:
        block = tr.label_from_sensor(
            data.hand, throw_time=ann['throw_time'],
            landing_time=ann['landing_time'], windows=windows)
        assert block.fields['t_departure_raw_ros'] is None


def _miner_windows():
    return tr.SensorWindows(
        arrival_lead_s=float(hw.JB_BD_ARRIVAL_LEAD_S),
        arrival_window_s=float(hw.JB_BD_ARRIVAL_WINDOW_S),
        retention_window_s=float(hw.JB_BD_RETENTION_WINDOW_S))


@pytest.mark.skipif(not _HAVE_BAG,
                    reason='reference rosbag is machine-local and gitignored')
def test_the_mined_labels_match_the_fixture(mined):
    _data, rows, _led = mined
    counts = {}
    for row in rows:
        counts[row['label']] = counts.get(row['label'], 0) + 1
    assert counts == fx.LABELS
    assert len(rows) == fx.N_SELF_TOSSES


# ── ILC Phase 0a/0c: the arc estimator ────────────────────────────────────────
#
# The miner's --self-check drives these paths with arcs generated BY
# ``ballistics_bc.position_at`` — right for testing the estimator, circular for
# testing the physics. So the fixtures below are written LONGHAND from the
# closed form, with the gravity constant spelled out, and the recovered state is
# compared against the numbers that generated it. If the miner and the planner
# ever end up integrating different physics, this is where it shows.
#
# Confirmed recipe (2026-08-12, the empirical-probe rule): a 4436 mm/s vertical
# release with a 60/-25 mm/s lateral component, sampled every 5 ms — the cadence
# ``/mocap_data`` actually runs at on 2026-08-12_19-02-52 (~235 Hz, i.e. 4.3 ms
# median) — is recovered to well under 1 mm/s. The tolerances below are 1 mm/s
# and 1 ms, i.e. ~200x tighter than the release errors this phase measures
# (+357..+503 mm/s) and ~10x tighter than the measured release-time scatter.

_G_MMS2 = 9806.0            # spelled out, NOT imported: see the note above


def _longhand_arc(p0, v0, plane_mm, t_release=1000.0, dt=0.005):
    """A ballistic arc from the closed form, in the miner's (t, x, y, z) shape."""
    import math as _math
    disc = v0[2] ** 2 - 2.0 * _G_MMS2 * (plane_mm - p0[2])
    t_end = (v0[2] + _math.sqrt(disc)) / _G_MMS2
    rows = []
    k = 0
    while k * dt <= t_end:
        t = k * dt
        rows.append((t_release + t,
                     p0[0] + v0[0] * t,
                     p0[1] + v0[1] * t,
                     p0[2] + v0[2] * t - 0.5 * _G_MMS2 * t * t))
        k += 1
    return rows


def _planes(miner):
    plane = miner.DEFAULT_PLANE_MM
    return plane, plane - float(hw.HAND_CATCH_OFFSET_MM) + \
        miner.HAND_THROW_OFFSET_MM


def test_the_miner_uses_the_production_ballistics_and_gravity(miner):
    """Design constraint 1, made grep-proof: one forward chain. A private copy of
    the ballistics — or of the gravity constant — is exactly the drift the
    plan's ``JB_OP_HAND_CATCH_PRIME_REV`` precedent is about."""
    from jugglebot.motion.trajectory import ballistics_bc
    assert miner.ballistics_bc is ballistics_bc
    assert miner.GRAVITY_MMS2 == ballistics_bc.GRAVITY_MMS2 == _G_MMS2
    # And the tracker agrees, so no comparison in this pipeline crosses a
    # gravity convention. The "tracker-side 9810" warned about in
    # toss_release.py's header is not a live PRODUCTION constant — which is the
    # claim this test can make and the only one the fits depend on. It is NOT
    # absent from the tree, and the miner's own census used to over-claim that:
    # `ball_arrival_offset`'s synthetic-track generator uses 9810, two stale
    # test-local constants still carry it (tests/ros/test_reload_integration.py
    # and tests/ros/test_toss_integration.py, whose "the 9810 the tracking stack
    # uses" description is now false), and attic/ is full of it.
    from jugglebot.tracking.ballistics import GRAVITY_MMPS2
    assert GRAVITY_MMPS2 == _G_MMS2


def test_a_longhand_ballistic_ascent_backcasts_to_its_own_release(miner):
    """THE 0c acceptance. A pure arc, sampled at the mocap rate, must return the
    release velocity, position and instant that generated it."""
    from jugglebot.motion.trajectory import ballistics_bc
    plane, rel_plane = _planes(miner)
    p0 = [150.0, -120.0, rel_plane]
    v0 = [60.0, -25.0, 4436.0]
    asc, _desc = miner.branches(_longhand_arc(p0, v0, plane), rel_plane, plane)
    assert len(asc) > 20
    pos, vel, t_ref, n, rms, se = miner.fit_ballistic(asc)
    lead = asc[0][0] - miner.ASCENT_REF_LEAD_S - t_ref
    p = ballistics_bc.position_at(pos, vel, lead)
    v = ballistics_bc.velocity_at(vel, lead)
    dt = ballistics_bc.touchdown_time(p, v, rel_plane, descending=False)
    got_v = ballistics_bc.velocity_at(v, dt)
    got_p = ballistics_bc.position_at(p, v, dt)
    assert n == len(asc) and rms < 1e-6
    for i in range(3):
        assert got_v[i] == pytest.approx(v0[i], abs=1.0)
        assert got_p[i] == pytest.approx(p0[i], abs=0.1)
    assert t_ref + lead + dt == pytest.approx(1000.0, abs=1e-3)


def test_a_longhand_ballistic_descent_recovers_the_arrival_velocity(miner):
    """THE 0a acceptance, and it is two claims at once: the descending fit
    recovers the truth, AND that truth is what the production
    ``arrival_velocity`` predicts from the commanded release. The second is the
    one that would catch a miner quietly running its own ballistics."""
    from jugglebot.motion.trajectory import ballistics_bc
    plane, rel_plane = _planes(miner)
    p0 = [150.0, -120.0, rel_plane]
    v0 = [60.0, -25.0, 4436.0]
    _asc, desc = miner.branches(_longhand_arc(p0, v0, plane), rel_plane, plane)
    pos, vel, t_ref, _n, _rms, _se = miner.fit_ballistic(desc)
    dt = ballistics_bc.touchdown_time(pos, vel, plane, descending=True)
    got = ballistics_bc.velocity_at(vel, dt)
    flight = ballistics_bc.touchdown_time(p0, v0, plane, descending=True)
    want = ballistics_bc.arrival_velocity(v0, flight)
    for i in range(3):
        assert got[i] == pytest.approx(want[i], abs=1.0)


def test_the_arc_estimator_reports_its_own_noise_floor(miner):
    """A SHORT branch fits a clean RMS and a badly wrong velocity — measured on
    2026-08-12_19-02-52 toss 6 (6 rows, 11.5 mm RMS, release speed 1400 mm/s
    wrong). ``se`` is the only field that says so, so it has to grow."""
    plane, rel_plane = _planes(miner)
    rows = miner.synth_arc([150.0, -120.0, rel_plane], [60.0, -25.0, 4436.0],
                           plane_mm=plane, t_release=1000.0, wobble_mm=8.0)
    asc, _desc = miner.branches(rows, rel_plane, plane)
    short = [r for r in asc if r[0] <= asc[0][0] + 0.06]
    assert len(short) >= miner.MIN_ARC_SAMPLES
    assert miner.fit_ballistic(short)[5][2] > \
        5.0 * miner.fit_ballistic(asc)[5][2]


def test_a_resting_ball_below_the_release_plane_is_not_the_arc(miner):
    """THE bug this phase found in the shipped landing fit. A self toss's window
    opens with our own ball sitting in the cup ~110 mm BELOW the catch plane;
    ``fit_plane_crossing_full`` cuts the descending run at the first sub-plane
    sample, so that one resting row cut the run to zero and every self toss on
    2026-08-12_19-02-52 reported ``land_xy_global_mm: null`` from a bag that
    carries clean arcs."""
    plane, rel_plane = _planes(miner)
    arc = _longhand_arc([150.0, -120.0, rel_plane], [0.0, 0.0, 4436.0], plane)
    rest = [(arc[0][0] - 0.3 + 0.01 * i, 150.0, -120.0, 697.0)
            for i in range(20)]
    asc, desc = miner.branches(rest + arc, rel_plane, plane)
    assert all(r[3] >= rel_plane for r in asc + desc)
    # And the shipped estimator, handed the post-apex rows, now produces a fit.
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'ball_arrival_offset',
        os.path.join(_REPO, 'tools', 'probes', 'ball_arrival_offset.py'))
    bao = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bao)
    assert bao.fit_plane_crossing_full(rest + arc, plane, 300.0) is None
    got = bao.fit_plane_crossing_full(desc, plane, 300.0)
    assert got is not None
    assert got[0] == pytest.approx(150.0, abs=1.0)


def test_a_labelled_marker_cell_is_refused_and_the_ball_is_not(miner):
    """QTM's own labels are the strongest available discriminator, and the
    platform rim markers that lose them for single frames land INSIDE the
    descending fit band. Two-sided: the stray goes, the ball 60 mm away stays."""
    cells = {miner._cell(0.0, 0.0, 1000.0)}
    kept = miner.drop_fixtures(
        [(1.0, 1.0, 1.0, 1001.0), (1.0, 60.0, 0.0, 1000.0)], cells)
    assert [r[1] for r in kept] == [60.0]
    # Time-scoped: the platform MOVES between tosses, so a session-wide fixture
    # set is a swath of space. On 2026-08-12_19-02-52 it held 3374 cells and
    # deleted 301 of toss 8's 341 candidate rows.
    buckets = {10: {miner._cell(0.0, 0.0, 1000.0)}}
    assert miner.fixtures_near(buckets, 10.0, 10.5) == buckets[10]
    assert miner.fixtures_near(buckets, 40.0, 40.5) == set()


def test_the_release_vs_flight_split_is_an_exact_decomposition(miner):
    """It is a DECOMPOSITION or it is two loosely-related numbers. Driven
    end-to-end through ``mine_arc`` on a synthetic bag so the identity is
    asserted on the fields a corpus actually carries."""
    plane, rel_plane = _planes(miner)
    p0 = [150.0, -120.0, rel_plane]
    v0 = [90.0, -40.0, 4436.0]
    data = miner.BagData()
    data.mocap = _longhand_arc(p0, v0, plane, t_release=1000.0)
    # The announced landing is the COMMANDED release's own ballistic landing —
    # that is what `build_announcement_fields` publishes, and it is what makes
    # "release-attributable landing error" equal "measured release vs commanded
    # release" rather than "commanded release vs a cup it was never aimed at".
    from jugglebot.motion.trajectory import ballistics_bc as _bc
    cup, _cv, tof = _bc.arrival_state_at_z(p0, v0, plane, descending=True)
    ann = {'thrower': 'jugglebot', 'target': 'jugglebot',
           'throw_time': 1000.0, 'landing_time': 1000.0 + tof,
           'landing_position': [float(cup[0]), float(cup[1]), plane],
           'initial_position': list(p0), 'initial_velocity': list(v0),
           'predicted_tof_sec': tof, 't_bag': 1000.0}
    land_xy = [155.0, -100.0]
    out = miner.mine_arc(data, ann, plane, land_xy=land_xy)
    assert out['cmd_release_source'] == 'announcement'
    rel, fly = out['land_err_release_mm'], out['land_err_flight_mm']
    for i in (0, 1):
        assert rel[i] + fly[i] == pytest.approx(
            land_xy[i] - ann['landing_position'][i], abs=1e-6)
    # A throw that IS the command has no release-attributable landing error.
    assert abs(rel[0]) < 1.0 and abs(rel[1]) < 1.0


def test_mine_arc_reports_errors_against_the_announcement_alone(miner):
    """The § 3.3 inversion, extended to 0a/0c: a bag that predates
    ``/toss/record`` still carries the commanded release state, because
    ``build_announcement_fields`` fills the announcement from the same
    ``ReleaseState`` the declaration reports. Without this the errors would be
    unavailable on exactly the historical corpus the miner exists to unlock."""
    plane, rel_plane = _planes(miner)
    v0 = [0.0, 0.0, 4436.0]
    fast = [0.0, 0.0, 4700.0]
    data = miner.BagData()
    data.mocap = _longhand_arc([150.0, -120.0, rel_plane], fast, plane,
                               t_release=1000.0)
    ann = {'throw_time': 1000.0, 'landing_time': 1000.9,
           'landing_position': [150.0, -120.0, plane],
           'initial_position': [150.0, -120.0, rel_plane],
           'initial_velocity': list(v0), 'predicted_tof_sec': 0.9}
    out = miner.mine_arc(data, ann, plane)
    assert out['cmd_launch_vel_mms'] == v0
    assert out['release_speed_err_mms'] == pytest.approx(264.0, abs=2.0)
    assert out['release_vel_err_mms'][2] == pytest.approx(264.0, abs=2.0)
    assert out['release_time_err_ms'] == pytest.approx(0.0, abs=2.0)
    # A faster throw arrives faster and LATER — both signs matter.
    assert out['arrival_speed_err_mms'] > 0.0
    assert out['flight_time_err_s'] > 0.0
    assert out['achieved_flight_s_mocap'] == pytest.approx(
        2.0 * 4700.0 / _G_MMS2, abs=5e-3)


def test_the_lean_error_convention_is_signed_the_same_way_at_both_events(miner):
    """A throw leaning +x arrives leaning +x. Without this the release and
    arrival direction channels would need a sign table to be read together, and
    a sign table is where the ILC update law would get its sign wrong."""
    from jugglebot.motion.trajectory import ballistics_bc
    v0 = [60.0, -25.0, 4436.0]
    up = miner._lean_rad(v0)
    down = miner._lean_rad(ballistics_bc.arrival_velocity(v0, 0.9))
    assert up[0] > 0.0 and down[0] > 0.0
    assert up[1] < 0.0 and down[1] < 0.0
    assert miner._lean_rad([0.0, 0.0, -4400.0]) == (0.0, 0.0)


def test_a_bounce_back_up_ends_the_descending_branch(miner):
    """The post-contact excursion guard, restated for a branch whose sub-plane
    rows are already gone: the ball coming back UP is the signal."""
    plane, rel_plane = _planes(miner)
    arc = _longhand_arc([150.0, -120.0, rel_plane], [0.0, 0.0, 4436.0], plane)
    _a, desc = miner.branches(arc, rel_plane, plane)
    bounced = list(arc) + [(arc[-1][0] + 0.02 * i, 400.0, -400.0, plane + 200.0)
                           for i in range(1, 6)]
    _a2, desc2 = miner.branches(bounced, rel_plane, plane)
    assert len(desc2) == len(desc)
