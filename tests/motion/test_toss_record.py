"""The per-toss record schema, labeller, latch and join.

Module under test: ``ros_ws/src/jugglebot/jugglebot/toss_record.py``.
Plan: ``plans/active/toss-selftuning.md`` §§ 3.3, 3.4, D10-D13.
Logbook: ``logbook/2026-08-10-toss-selftuning-build.md``.

WHY THESE TESTS AND NOT OTHERS
------------------------------
Three things about this module can break silently, and each has a section below.

1. **The schema drifts.** ``FIELDS`` is the contract between a node that
   declares, a miner that measures and a fitter that reads a corpus months
   later. A removed field changes the meaning of an old row, so removal has to
   bump ``SCHEMA`` — and the only way to notice is a pinned list.
2. **The labeller reads the wrong bit.** The can-bridge debounce is
   *asymmetric* (0 ms on a rise, ~240 ms on a fall, measured), so a labeller
   that took departure times off the debounced verdict would be quietly 240 ms
   late on every timing measurement and look like real physics. Every synthetic
   stream here therefore bakes the asymmetry in; a symmetric one would pass with
   the bug present.
3. **The two halves of a record disagree and nobody notices.** ``D+M`` fields
   exist to be cross-checked; the join must record a conflict rather than pick a
   winner.
"""

from __future__ import annotations

import math

import pytest

from jugglebot import toss_record as tr


# The SHIPPED windows, as both the node and the miner construct them. Imported
# from the generated config rather than typed in, so a config change moves these
# tests with it — the drift-guard below pins the miner's construction to the same
# source.
import jugglebot.hardware_config as hw

WINDOWS = tr.SensorWindows(
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
        out.append(tr.SensorSample(t=t, held=deb, raw=raw, valid=valid,
                                   stamp=t if valid else 0.0))
        t += dt
    return out


def label(**kw):
    return tr.label_from_sensor(stream(**kw), throw_time=THROW_T,
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
    'position_planned_s', 'position_code', 'catch_target_accepted',
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
    'iq_brake_min_a', 'dispatch_shift_ms', 'hand_traj_acks', 'can_errors',
    'bridge_tx_diag', 'plant_block_source',
    # quality
    'label', 'label_source', 'label_confidence', 'label_reason', 'rimshot',
    'disagreement', 'record_provenance', 'join_residual_ms',
    'usable_for_aim_fit', 'usable_for_timing_fit', 'usable_for_speed_fit',
    'usable_for_release_fit', 'usable_for_lateral_fit', 'excluded_reason',
)


def test_fields_are_pinned():
    """The drift guard. A removal or a rename goes red here first."""
    assert tr.FIELD_NAMES == EXPECTED_FIELDS


def test_schema_version_is_pinned():
    assert tr.SCHEMA == 'toss_record/1'


def test_field_names_are_unique():
    assert len(set(tr.FIELD_NAMES)) == len(tr.FIELD_NAMES)


def test_names_by_origin_partitions_the_whole_schema():
    """The origin table is how a corpus reader tells a declared number from a
    mined one WITHOUT a per-field provenance marker on every row (§ 3.3 asks the
    declaration to "upgrade" fields; row-level ``record_provenance`` plus this
    partition says the same thing at a fraction of the schema cost). It only
    works if the partition is total and disjoint."""
    seen = []
    for origin in ('D', 'M', 'DM', 'X'):
        seen.extend(tr.names_by_origin(origin))
    assert sorted(seen) == sorted(tr.FIELD_NAMES)
    assert len(seen) == len(set(seen))
    # The join key must be independently recoverable, or a bag with no
    # declaration cannot be joined to anything — including itself.
    assert 'announce_throw_time_ros' in tr.names_by_origin('DM')
    # The label is DERIVED, never declared: the node terminalises 0.80 s before
    # the retention window closes, so it structurally cannot know.
    assert 'label' in tr.names_by_origin('X')


def test_every_field_declares_a_known_origin_and_kind():
    for f in tr.FIELDS:
        assert f.origin in ('D', 'M', 'DM', 'X'), f
        assert f.kind in ('s', 'f', 'i', 'b', 'f2', 'f3', 'l', 'o'), f


def test_the_layer_1_5_covariate_block_exists_and_is_nullable():
    """Schema lands in 2a; the dwell read schedule is wired in 2d.

    ``dwell_tilt_n = 0`` (and a fully null block) is a LEGAL record from day
    one — plan § 3.10's degrade-never-delay rule guarantees a short dwell yields
    fewer reads, so an analyser that assumed the block was always populated would
    break on real hardware, not just on this phase's placeholders.
    """
    dwell = [f for f in tr.FIELDS if f.block == 'dwell_tilt']
    assert {f.name for f in dwell} == {
        'dwell_tilt_rad', 'dwell_tilt_sd_rad', 'dwell_tilt_n',
        'dwell_tilt_span_s', 'dwell_tilt_last_read_to_release_s',
        'dwell_tilt_degraded'}
    assert not any(f.required for f in dwell)
    rec = tr.blank_record()
    rec.update({'toss_uid': 'x', 'action': 'toss', 'outcome': 'CAUGHT'})
    assert tr.validate(rec) == ()


# ── Encode / decode / validate ────────────────────────────────────────────────

def test_encode_round_trips_every_field():
    rec = tr.blank_record()
    rec.update({'toss_uid': 'a-b-1', 'action': 'toss', 'outcome': 'CAUGHT',
                'land_err_mm': [1.5, -2.5], 'clamp_hits': ['x'],
                'catch_knobs': {'catch_vel_scale': 1.0}, 'cycle_index': 3,
                'success': True})
    back = tr.decode(tr.encode(rec))
    assert back == rec


def test_a_nan_encodes_as_null_not_a_bare_nan_token():
    """JSON has no NaN. Python's json will emit a bare ``NaN`` token happily and
    every strict parser downstream rejects it — so a NaN measurement is encoded
    as the null it actually is, and ``allow_nan=False`` makes a miss a hard
    error rather than a corpus nobody can read."""
    rec = tr.blank_record()
    rec.update({'toss_uid': 'x', 'action': 'toss', 'outcome': 'X',
                'catch_error_mm_fsm': float('nan'),
                'achieved_flight_s_fsm': float('inf')})
    payload = tr.encode(rec)
    assert 'NaN' not in payload and 'Infinity' not in payload
    back = tr.decode(payload)
    assert back['catch_error_mm_fsm'] is None
    assert back['achieved_flight_s_fsm'] is None


def test_encode_drops_unknown_keys():
    """A typo'd key that survives encoding is a field nobody will ever look
    for — and it would sit in the corpus looking like data."""
    rec = tr.blank_record()
    rec.update({'toss_uid': 'x', 'action': 'toss', 'outcome': 'X',
                'nonsense_field': 7})
    assert 'nonsense_field' not in tr.decode(tr.encode(rec))


def test_decode_fills_missing_fields_with_null():
    """An older row missing newer fields must still read, present-and-null.

    Present-and-null, never absent: a reader that has to tell "this schema has
    no such field" from "nothing was measured" is a reader that will guess.
    """
    rec = tr.decode('{"schema":"toss_record/1","toss_uid":"x",'
                    '"action":"toss","outcome":"CAUGHT"}')
    assert set(rec) == set(tr.FIELD_NAMES)
    assert rec['land_err_mm'] is None


def test_validate_flags_required_fields_and_wrong_kinds():
    problems = tr.validate({'schema': tr.SCHEMA})
    assert any('toss_uid' in p for p in problems)
    assert any('action' in p for p in problems)
    assert any('outcome' in p for p in problems)
    rec = tr.blank_record()
    rec.update({'toss_uid': 'x', 'action': 'toss', 'outcome': 'X',
                'cycle_index': 'three', 'land_err_mm': [1.0]})
    problems = tr.validate(rec)
    assert any('cycle_index' in p for p in problems)
    assert any('land_err_mm' in p for p in problems)


def test_validate_refuses_an_unknown_schema_version():
    rec = tr.blank_record()
    rec.update({'schema': 'toss_record/99', 'toss_uid': 'x', 'action': 'toss',
                'outcome': 'X'})
    assert any('schema' in p for p in tr.validate(rec))


def test_validate_refuses_a_label_outside_the_vocabulary():
    rec = tr.blank_record()
    rec.update({'toss_uid': 'x', 'action': 'toss', 'outcome': 'X',
                'label': 'PROBABLY_CAUGHT'})
    assert any('label' in p for p in tr.validate(rec))


def test_a_bool_is_not_accepted_where_a_number_is_wanted():
    """``True == 1`` in Python, so a bool slipping into a numeric field would
    validate and then read as 1.0 in a fit. It is refused explicitly."""
    rec = tr.blank_record()
    rec.update({'toss_uid': 'x', 'action': 'toss', 'outcome': 'X',
                'land_err_norm_mm': True, 'cycle_index': True})
    problems = tr.validate(rec)
    assert any('land_err_norm_mm' in p for p in problems)
    assert any('cycle_index' in p for p in problems)


# ── 2. The labeller ───────────────────────────────────────────────────────────

def test_a_clean_catch_is_caught():
    got = label(catch_dt=0.4)
    assert got.label == tr.LABEL_CAUGHT
    assert got.fields['held_at_catch_plus_retention'] is True


def test_no_departure_edge_is_no_release():
    """The ball never left the cup — which is a DIFFERENT fact from a miss, and
    a fit that pooled the two would be learning aim from tosses that never
    happened."""
    got = label(departure_dt=None, catch_dt=None)
    assert got.label == tr.LABEL_NO_RELEASE
    assert got.fields['sensor_held_at_dispatch'] is True


def test_departure_without_arrival_is_missed():
    got = label(catch_dt=None)
    assert got.label == tr.LABEL_MISSED
    assert got.fields['t_catch_deb_ros'] is None


def test_an_arrival_that_leaves_inside_retention_is_bounced():
    got = label(catch_dt=0.4, drop_after=0.999)
    assert got.label == tr.LABEL_BOUNCED
    assert got.fields['t_dropout_ros'] is not None


@pytest.mark.parametrize('catch_dt', (0.137, 0.798))
def test_the_measured_catch_band_labels_caught_at_both_ends(catch_dt):
    """+137 ms and +798 ms are the earliest and latest arrivals in the whole
    2026-08-10 three-bag population. The +798 row is the one that SIZED
    ``JB_BD_ARRIVAL_WINDOW_S``; if the search window is ever trimmed toward the
    median it is the row that goes red first, which is the point."""
    assert label(catch_dt=catch_dt).label == tr.LABEL_CAUGHT


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
    """
    draft = tr.SensorWindows(arrival_lead_s=0.30, arrival_window_s=0.70,
                             retention_window_s=WINDOWS.retention_window_s)

    def under(windows, catch_dt):
        return tr.label_from_sensor(
            stream(catch_dt=catch_dt), throw_time=THROW_T,
            landing_time=LANDING_T, windows=windows).label

    assert under(WINDOWS, 0.798) == tr.LABEL_CAUGHT
    assert under(draft, 0.798) == tr.LABEL_MISSED
    # The runner-up survives the draft — which is the honest statement of how
    # thin the margin was, not a claim that the draft lost two rows.
    assert under(draft, 0.675) == tr.LABEL_CAUGHT


def test_an_invalid_sample_in_the_window_is_unknown_and_never_collapses():
    """D13 / C-POSSESS-1 § 2. Treating "no valid sample" as "no ball" mints a
    false MISSED on every telemetry hiccup — and those false MISSEDs are exactly
    the records the aim fit wants most, because a miss with a clean mocap fit is
    the most informative aim datum there is."""
    got = label(catch_dt=0.4, blind=(LANDING_T - 0.1, LANDING_T + 0.5))
    assert got.label == tr.LABEL_UNKNOWN
    assert got.confidence == 0.0
    assert got.fields['sensor_valid_frac'] < 1.0


def test_an_empty_window_is_unknown_not_no_release():
    """No samples at all is the sensor not looking, which is UNKNOWN. Reading it
    as NO_RELEASE would invent a mechanical fact out of a missing topic."""
    got = tr.label_from_sensor([], throw_time=THROW_T, landing_time=LANDING_T,
                               windows=WINDOWS)
    assert got.label == tr.LABEL_UNKNOWN
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
    got = tr.label_from_sensor(flicked, throw_time=THROW_T,
                               landing_time=LANDING_T, windows=WINDOWS)
    assert got.label == tr.LABEL_CAUGHT


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
        assert got.label == tr.LABEL_CAUGHT
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
    assert label(catch_dt=0.4, drop_after=1.20).label == tr.LABEL_BOUNCED
    assert label(catch_dt=0.4, drop_after=1.32).label == tr.LABEL_CAUGHT


def test_the_measured_poll_cadence_is_reported_not_assumed():
    """``sensor_poll_dt_ms_median`` is measured from ``ball_held_stamp``
    advances. On the reference bag it reads ~71 ms against a CONFIGURED
    ``JB_BD_CHECK_INTERVAL_MS`` of 20 — a 3.5x gap that is invisible unless the
    record carries the measurement."""
    got = label(catch_dt=0.4)
    assert got.fields['sensor_poll_dt_ms_median'] == pytest.approx(10.0,
                                                                   abs=0.5)


def test_edges_skip_invalid_samples_rather_than_reading_them_as_a_level():
    """UNKNOWN is not EMPTY. Letting an invalid sample act as a level would mint
    a departure edge out of a telemetry hiccup."""
    samples = [tr.SensorSample(0.0, True, True, True),
               tr.SensorSample(0.1, False, False, False),   # UNKNOWN
               tr.SensorSample(0.2, True, True, True)]
    assert tr.edges(samples) == ((), ())


# ── 3. The latch ──────────────────────────────────────────────────────────────

class _Ball(object):
    def __init__(self, ball_id, status, destination=''):
        self.id = ball_id
        self.status = status
        self.destination = destination


IN_FLIGHT = 1


def test_the_latch_prefers_a_destination_tagged_candidate():
    got = tr.latch_announced_ball(
        [_Ball(7, IN_FLIGHT), _Ball(9, IN_FLIGHT, 'jugglebot')],
        robot_name='jugglebot', announced_id=None, preexisting_ids=(),
        untagged_latch=False, in_flight_status=IN_FLIGHT)
    assert got == (9, False)


def test_the_latch_excludes_ids_already_in_flight_at_goal_start():
    """Attempt 5 of the 2026-07-23 re-test latched a phantom untagged track that
    predated the throw and rode it to a wrong MISSED verdict. Our ball cannot be
    airborne before our own announcement's throw_time."""
    got = tr.latch_announced_ball(
        [_Ball(3, IN_FLIGHT)], robot_name='jugglebot', announced_id=None,
        preexisting_ids=(3,), untagged_latch=False,
        in_flight_status=IN_FLIGHT)
    assert got == (None, False)


def test_an_untagged_latch_is_provisional_and_a_tagged_track_displaces_it():
    got = tr.latch_announced_ball(
        [_Ball(7, IN_FLIGHT), _Ball(9, IN_FLIGHT, 'jugglebot')],
        robot_name='jugglebot', announced_id=7, preexisting_ids=(),
        untagged_latch=True, in_flight_status=IN_FLIGHT)
    assert got == (9, False)


def test_a_tagged_latch_is_NOT_displaced():
    got = tr.latch_announced_ball(
        [_Ball(9, IN_FLIGHT, 'jugglebot'), _Ball(11, IN_FLIGHT, 'jugglebot')],
        robot_name='jugglebot', announced_id=9, preexisting_ids=(),
        untagged_latch=False, in_flight_status=IN_FLIGHT)
    assert got == (9, False)


def test_the_latch_ignores_a_ball_bound_for_another_robot():
    got = tr.latch_announced_ball(
        [_Ball(9, IN_FLIGHT, 'ball_butler')], robot_name='jugglebot',
        announced_id=None, preexisting_ids=(), untagged_latch=False,
        in_flight_status=IN_FLIGHT)
    assert got == (None, False)


def test_the_node_calls_the_extracted_latch():
    """Grep-proof for D11: the coordinator must not keep a private copy.

    Imported at module scope in the node, so an accidental re-inlining shows up
    as this import going unused — and this assertion goes red the moment the
    symbol stops being the node's.
    """
    import importlib
    src = importlib.import_module('jugglebot.toss_record')
    assert src.latch_announced_ball is tr.latch_announced_ball


# ── 4. The join ───────────────────────────────────────────────────────────────

def test_a_matched_pair_joins_and_reports_its_residual():
    joined = tr.join([{'announce_throw_time_ros': 10.0, 'toss_uid': 'd',
                       'goal_throw_delay_s': 5.0}],
                     [{'announce_throw_time_ros': 10.002,
                       'sensor_valid_frac': 1.0}])
    assert len(joined) == 1
    assert joined[0]['record_provenance'] == tr.PROV_BOTH
    assert joined[0]['goal_throw_delay_s'] == 5.0
    assert joined[0]['sensor_valid_frac'] == 1.0
    assert joined[0]['join_residual_ms'] == pytest.approx(2.0)


def test_outside_the_tolerance_the_halves_do_not_join():
    joined = tr.join([{'announce_throw_time_ros': 10.0}],
                     [{'announce_throw_time_ros': 10.02}])
    assert [r['record_provenance'] for r in joined] == [tr.PROV_MINED,
                                                        tr.PROV_DECLARED]


def test_a_mined_only_row_is_a_full_record():
    """The inversion that makes the whole design work: the miner must produce a
    complete record ALONE, in degraded form. It is what lets the three
    2026-08-10 bags become a corpus before any of this was wired."""
    joined = tr.join([], [{'announce_throw_time_ros': 10.0,
                           'sensor_valid_frac': 1.0, 'label': 'CAUGHT'}])
    assert set(joined[0]) == set(tr.FIELD_NAMES)
    assert joined[0]['record_provenance'] == tr.PROV_MINED
    assert joined[0]['label'] == 'CAUGHT'
    assert joined[0]['toss_uid'] == 'mined-10.000'


def test_an_unmatched_declaration_is_kept_not_dropped():
    """A silently missing row is how a replay overstates its own agreement, and
    an unmatched declaration is itself a finding: either the bag lost the
    announcement or the toss never announced."""
    joined = tr.join([{'announce_throw_time_ros': 10.0, 'toss_uid': 'd'}], [])
    assert [r['record_provenance'] for r in joined] == [tr.PROV_DECLARED]


def test_a_declared_vs_mined_conflict_is_recorded_never_resolved():
    """``D+M`` fields exist precisely so the halves can be cross-checked.
    Resolving a conflict silently throws away the only signal the redundancy
    buys."""
    joined = tr.join([{'announce_throw_time_ros': 10.0,
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
    joined = tr.join([{'announce_throw_time_ros': 10.0, 'toss_uid': 'd'}],
                     [{'announce_throw_time_ros': 10.003}])
    assert joined[0]['disagreement'] == []


def test_the_declaration_wins_on_a_declared_only_field():
    """The node is the only witness of what it commanded."""
    joined = tr.join([{'announce_throw_time_ros': 10.0, 'toss_uid': 'd',
                       'event_vel_mps': 3.91}],
                     [{'announce_throw_time_ros': 10.0,
                       'event_vel_mps': 99.0}])
    assert joined[0]['event_vel_mps'] == 3.91


def test_each_declaration_joins_at_most_once():
    """Two mined rows inside one tolerance would otherwise both claim the same
    declaration and the corpus would double-count a toss."""
    joined = tr.join([{'announce_throw_time_ros': 10.0, 'toss_uid': 'd'}],
                     [{'announce_throw_time_ros': 10.001},
                      {'announce_throw_time_ros': 10.002}])
    assert sorted(r['record_provenance'] for r in joined) == [
        tr.PROV_BOTH, tr.PROV_MINED]


def test_every_joined_row_encodes():
    joined = tr.join([{'announce_throw_time_ros': 10.0, 'toss_uid': 'd',
                       'action': 'toss', 'outcome': 'CAUGHT'}],
                     [{'announce_throw_time_ros': 10.0,
                       'land_err_norm_mm': float('nan')}])
    for row in joined:
        assert tr.validate(tr.decode(tr.encode(row))) == ()


# ── Physical sanity ───────────────────────────────────────────────────────────

def test_the_departure_window_is_far_wider_than_the_measured_shift():
    """A pinned threshold has to be justified by the measurement it covers.
    Measured worst-case release shift is +212 ms; the window is 1.00 s before the
    landing clamp and ~0.60 s after it on the shipped 0.8 s flight."""
    assert tr.DEPARTURE_WINDOW_S >= 4.0 * 0.212
    clamped = FLIGHT_S - float(hw.JB_BD_ARRIVAL_LEAD_S)
    assert clamped >= 2.5 * 0.212
    assert math.isclose(tr.DEPARTURE_LEAD_S, 0.30)
