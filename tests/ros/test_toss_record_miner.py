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
             'excluded_reason': None},
            {'toss_uid': 'b', 'catch_point_global_mm': [0.0, 0.0, 500.0],
             'land_plane_mm': 501.0, 'usable_for_aim_fit': True,
             'usable_for_speed_fit': True, 'usable_for_timing_fit': True,
             'excluded_reason': None}]


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


def test_the_plane_refusal_spares_the_TIMING_fit(miner):
    """The timing fit reads sensor edges and release instants, which the mocap
    fit plane has no bearing on. Refusing it too would throw away good data for
    an unrelated fault."""
    rows = _plane_rows()
    miner.enforce_declared_planes(rows)
    assert rows[0]['usable_for_timing_fit'] is True


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
