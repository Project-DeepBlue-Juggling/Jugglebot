"""The hand-ball-sensor verdict, replayed against a MEASURED session.

Contract: ``ros_ws/docs/ball_possession_contract.md`` (C-POSSESS-1 §§ 3.2, 3.3).
Probe: ``tools/probes/hand_sensor_verdict_replay.py``. Logbook:
``logbook/2026-08-10-sensor-truth-possession.md``.

WHY A REPLAY TEST EXISTS ALONGSIDE test_ball_possession.py
----------------------------------------------------------
Those tests drive the source with *synthesised* streams placed at the measured
instants — they pin the RULE. This one drives it with the actual 100 Hz sample
stream the machine produced, including its real jitter, its real debounce
behaviour and its real transition structure, and asserts the LABELS. The two fail
in different ways: a broken rule breaks the first, and a rule that only works on
tidy synthetic data breaks the second.

THE FIXTURE
-----------
``tools/probes/data/hand_sensor_replay_fixture.json`` is a committed cut of
``~/Desktop/rosbags/2026-08-10_16-30-44`` over ``[140, 305) s`` — 16,444
``/hand_telemetry`` samples, 8 announcements — chosen because that one window
carries every piece of evidence the shipped window defaults rest on: the
band-maximum catch (+798 ms, which is what sizes ``arrival_window_s``), five
further catches spanning +269…+468 ms, two genuine misses, and all three of the
session's sub-second seat-then-leave events (0.571 / 0.989 / 0.999 s, which is
what sizes ``retention_window_s``). Regenerate with the recipe stored inside the
fixture itself (``doc['recipe']``) — a fixture nobody can re-baseline is only half
a live reference (``tools/probes/README.md``).

Sample instants are stored as CUMULATIVE integer milliseconds, not deltas:
rounding 16,444 deltas accumulates a ~17 ms random walk, which showed up as a
14 ms error in the replayed catch-event time before it was fixed.

The bag itself is machine-local and gitignored, so the bag-backed test skips when
``~/Desktop/rosbags`` is absent; the fixture-backed tests always run.
"""

from __future__ import annotations

import importlib.util
import os

import pytest

import jugglebot.hardware_config as hw

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
_PROBE_PATH = os.path.join(_REPO, 'tools', 'probes',
                           'hand_sensor_verdict_replay.py')
_BAG_ROOT = os.path.expanduser('~/Desktop/rosbags')
_REFERENCE_BAG = '2026-08-10_16-30-44'


@pytest.fixture(scope='module')
def probe():
    spec = importlib.util.spec_from_file_location(
        'hand_sensor_verdict_replay', _PROBE_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope='module')
def replayed(probe):
    samples, anns, doc = probe.load_fixture()
    rows, led = probe.replay(samples, anns)
    return rows, led, doc


# ── The instrument's own acceptance ───────────────────────────────────────────

def test_probe_self_check_is_clean(probe):
    """Nine two-sided cases at the measured instants. Every ACCEPT case has a
    REFUSE twin, so a window that silently widened fails here."""
    assert probe.self_check() == 0


def test_probe_mirrors_the_nodes_windows(probe):
    """The probe constructs the source from the same generated constants the node
    does. Without this the bench instrument could score a capture against windows
    the robot never ran — the failure mode `possession_verdict_bag_check.py`'s own
    drift guard exists for."""
    src = probe.make_source()
    assert src.arrival_lead_s == pytest.approx(float(hw.JB_BD_ARRIVAL_LEAD_S))
    assert src.arrival_window_s == pytest.approx(float(hw.JB_BD_ARRIVAL_WINDOW_S))
    assert src.retention_window_s == pytest.approx(
        float(hw.JB_BD_RETENTION_WINDOW_S))


def test_probe_self_check_catches_a_widened_arrival_window(probe, monkeypatch):
    """Mutation guard. Without it, "the self-check passed" means only "the probe
    agrees with itself" — and the +3194 ms re-seat row is the one that keeps an
    operator's hand-reload from being counted as a catch."""
    assert probe.self_check() == 0, 'baseline must be green before mutating'
    monkeypatch.setattr(probe, 'ARRIVAL_WINDOW_S', 5.0)
    assert probe.self_check() == 1


# ── The replayed session ──────────────────────────────────────────────────────

def test_the_fixture_ledger_is_fully_valid(replayed):
    """100 % ``ball_held_valid`` across the cut — and stated as a LIMIT, not a
    boast: every UNKNOWN path in this phase is exercised by synthetic tests only,
    because three sessions of real hardware never produced a single invalid
    sample. The first bag that does is the one that validates them."""
    _rows, led, _doc = replayed
    assert led['n_samples'] == 16444
    assert led['n_valid'] == led['n_samples']
    assert led['valid_frac'] == pytest.approx(1.0)


def test_the_replayed_labels_match_the_session(replayed):
    """Six catches and two misses, in order. The +798 ms row is the load-bearing
    one: it is the latest arrival in the whole three-bag population, so if the
    arrival window is ever trimmed toward the median it is the row that goes red
    first — which is the point."""
    rows, _led, _doc = replayed
    assert [r['label'] for r in rows] == [
        'CAUGHT', 'MISSED', 'CAUGHT', 'CAUGHT', 'CAUGHT', 'CAUGHT', 'CAUGHT',
        'MISSED']
    caught = [r['catch_dt_s'] for r in rows if r['label'] == 'CAUGHT']
    assert caught == pytest.approx(
        [0.798, 0.269, 0.468, 0.450, 0.345, 0.298], abs=0.002)
    assert max(caught) < float(hw.JB_BD_ARRIVAL_WINDOW_S)


def test_every_replayed_catch_also_confirms_retention(replayed):
    """No catch in this session bounced out, so RETENTION must read CONFIRMED on
    all six. A retention rule that fired spuriously would show up here as a
    BOUNCE label on a ball the operator watched stay in the cup."""
    rows, _led, _doc = replayed
    for r in rows:
        if r['label'] == 'CAUGHT':
            assert r['retention'] == 'CONFIRMED', r


def test_the_three_measured_seat_then_leaves_are_in_the_ledger(replayed):
    """The retention window's ground truth, carried in the committed cut so the
    sizing argument survives without the bag. These are the operator hand-loading
    the cup (hand parked at pos_meas ≈ 0, no announcement within 54 s), NOT
    post-catch bounce-outs — they prove the sensor RESOLVES a sub-second
    seat-then-leave, which is the property the window rests on."""
    _rows, led, _doc = replayed
    durations = sorted(d for _t, d in led['quick_drops'])
    assert durations == pytest.approx([0.571, 0.989, 1.000], abs=0.002)
    assert max(durations) < float(hw.JB_BD_RETENTION_WINDOW_S)
    # 1.5x margin on the longest — the number the shipped default is sized on.
    assert float(hw.JB_BD_RETENTION_WINDOW_S) / max(durations) >= 1.5


def test_no_replayed_throw_is_unknown(replayed):
    """With a healthy sensor the merge never has to fall back to the tracker. An
    UNKNOWN row on this fixture would mean the source went blind on a stream that
    is 100 % valid — i.e. the staleness bound no longer fits the 100 Hz
    ``/hand_telemetry`` cadence."""
    rows, _led, _doc = replayed
    assert not [r for r in rows if r['label'] == 'UNKNOWN']


def test_a_refusing_tracker_no_longer_suppresses_the_catches(probe, replayed):
    """The capability the merge buys, made countable. Every destination-tagged
    reload track in the 2026-07-27 reference capture was refused by the tracker
    (204.9–752.9 mm out), so ``refuse`` is the measured pessimistic tracker. Under
    it the pre-2026-08-10 machine mints ZERO catches on this session; the sensor
    restores all six."""
    rows, _led, _doc = replayed
    merged = probe.counterfactual(rows, 'refuse')
    assert sum(merged) == 6
    assert [m for m, r in zip(merged, rows) if r['label'] == 'MISSED'] == \
        [False, False]


def test_the_fixture_carries_its_own_regeneration_recipe(replayed):
    """`tools/probes/README.md`: a committed cut needs a committed recipe, or the
    next person to change a window has no way to re-baseline it."""
    _rows, _led, doc = replayed
    assert doc['source_bag'] == _REFERENCE_BAG
    assert 'hand_sensor_verdict_replay.py' in doc['recipe']
    assert '--emit-fixture' in doc['recipe']


# ── The same replay, against the real bag when it is present ──────────────────

@pytest.mark.skipif(not os.path.isdir(os.path.join(_BAG_ROOT, _REFERENCE_BAG)),
                    reason='reference rosbag is machine-local and gitignored')
def test_the_fixture_still_matches_the_bag_it_was_cut_from(probe):
    """Guards the cut against the bag drifting out from under it (a re-record, a
    truncated copy). Skips on any machine without the bag — including CI and a
    fresh clone — which is why the fixture-backed tests above are the ones that
    carry the assertions."""
    samples, anns, _off = probe.read_bag(os.path.join(_BAG_ROOT, _REFERENCE_BAG))
    fx_samples, fx_anns, doc = probe.load_fixture()
    t0, t1 = doc['cut_s']
    cut = [(t, h, v) for t, h, v in samples if t0 <= t < t1]
    assert len(cut) == len(fx_samples)
    assert [n for _t, n, _l in fx_anns] == \
        [n for _t, n, land in anns if t0 <= land < t1]
    # Re-base BOTH streams onto the fixture's origin (its first sample), or the
    # announcements land 140 s away from the samples and everything reads
    # UNKNOWN — which is how this test first failed.
    base = cut[0][0]
    rows_live, _ = probe.replay(
        [(t - base, h, v) for t, h, v in cut],
        [(a - base, n, land - base) for a, n, land in anns if t0 <= land < t1])
    rows_fx, _ = probe.replay(fx_samples, fx_anns)
    assert [r['label'] for r in rows_live] == [r['label'] for r in rows_fx]
    # Catch-event times agree to the fixture's own quantisation (cumulative whole
    # milliseconds on both the sample instant and the landing stamp, so ±1 ms).
    assert ([r['catch_dt_s'] for r in rows_live if r['label'] == 'CAUGHT']
            == pytest.approx(
                [r['catch_dt_s'] for r in rows_fx if r['label'] == 'CAUGHT'],
                abs=0.002))
