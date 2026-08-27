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
from jugglebot.ball_possession import ARRIVAL_BAND_MAX_S


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
    'staged_discarded_reason',
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
    draft = tr.SensorWindows(arrival_lead_s=0.30,
                             arrival_window_s=ARRIVAL_BAND_MAX_S - 0.01,
                             retention_window_s=WINDOWS.retention_window_s)

    def under(windows, catch_dt):
        return tr.label_from_sensor(
            stream(catch_dt=catch_dt), throw_time=THROW_T,
            landing_time=LANDING_T, windows=windows)

    assert under(WINDOWS, 0.798).label == tr.LABEL_CAUGHT
    assert draft.arrival_window_s < ARRIVAL_BAND_MAX_S, (
        'premise: the draft window is shorter than the band it must judge')
    # The plan's literal 0.70 s, stated for the record: it outlasts today's band
    # and so labels normally — the historical hazard is gone, not the mechanism.
    plan_draft = tr.SensorWindows(
        arrival_lead_s=0.30, arrival_window_s=0.70,
        retention_window_s=WINDOWS.retention_window_s)
    assert plan_draft.arrival_window_s > ARRIVAL_BAND_MAX_S
    relabelled = under(draft, draft.arrival_window_s + 0.005)
    assert relabelled.label == tr.LABEL_UNKNOWN
    assert 'band clamped' in relabelled.reason
    # …and an arrival INSIDE the short window still labels normally, so this is a
    # clamp test rather than a blanket softening of the label. (In the 2026-08-10
    # argument this row was the +675 ms runner-up, 25 ms inside the plan's 0.70 s
    # boundary — the measure of how thin that margin was. Derived now, because
    # "just inside the window" is the property, not the number.)
    assert under(draft, draft.arrival_window_s - 0.025).label == tr.LABEL_CAUGHT


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
    return [tr.SensorSample(t=t0 + k * publish_dt, held=True, raw=True,
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
    got = tr.poll_dt_ms_median(samples)
    assert got == pytest.approx(poll_dt * 1e3, abs=0.5)
    assert got != pytest.approx(10.0, abs=0.5), 'reported the PUBLISH rate'
    steps = tr.poll_dt_steps_ms(samples)
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
    steps = tr.poll_dt_steps_ms(hurt)
    assert steps.n_backwards == 1
    assert min(steps.steps_ms) > 0.0
    assert max(steps.steps_ms) == pytest.approx(20.0, abs=0.5), (
        'the re-anchor distance came back as a forward step')
    # 40 polls -> 39 steps unbroken; the re-anchor loses the ONE interval that
    # spans it and the tracker reseeds, so 38 — never fewer, never a 39th.
    assert len(steps.steps_ms) == 38
    assert steps.n_domain_breaks == 0
    assert tr.poll_dt_ms_median(hurt) == pytest.approx(20.0, abs=0.5)


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
    steps = tr.poll_dt_steps_ms(hurt)
    assert max(steps.steps_ms) == pytest.approx(20.0, abs=0.5), (
        'a step spanned the blind gap')
    # 40 polls, 10 of them lost to the gap, and the first poll after it only
    # seeds the tracker: strictly fewer steps than the unbroken stream's 39.
    assert len(steps.steps_ms) < 39
    assert tr.poll_dt_ms_median(hurt) == pytest.approx(20.0, abs=0.5)


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
    pre = [tr.SensorSample(t=WALL_T0 + k * 0.010, held=True, raw=True,
                           valid=True, stamp=BOOT_STAMPS[k])
           for k in range(len(BOOT_STAMPS))]
    post = poll_stream(poll_dt=0.020, n_polls=20,
                       t0=WALL_T0 + len(BOOT_STAMPS) * 0.010)
    steps = tr.poll_dt_steps_ms(pre + post)
    assert steps.n_domain_breaks == 1
    assert max(steps.steps_ms) < 1e3, 'the wall epoch entered the statistics'
    assert tr.poll_dt_ms_median(pre + post) == pytest.approx(20.0, abs=1.0)
    # And the same stream read wholly inside ONE epoch still MEASURES: these
    # six boot-relative stamps yield 14 / 20 / 37 / 40 ms, real poll intervals.
    boot_only = tr.poll_dt_steps_ms(pre)
    assert boot_only.n_domain_breaks == 0 and boot_only.n_backwards == 0
    assert all(10.0 <= s <= 50.0 for s in boot_only.steps_ms)
    assert tr.poll_dt_ms_median(pre) is not None


def test_no_stamp_advance_at_all_reports_None_rather_than_a_number():
    """A stream that never advances is not a 0 ms poll. ``None`` is what the
    admission gate reads as ``poll_cadence_unmeasured``; a zero would sail
    through the floor comparison as a very fast sensor."""
    frozen = [tr.SensorSample(t=THROW_T + k * 0.01, held=True, raw=True,
                              valid=True, stamp=THROW_T)
              for k in range(50)]
    assert tr.poll_dt_ms_median(frozen) is None
    assert tr.poll_dt_steps_ms(frozen).steps_ms == ()


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


def test_departure_lead_is_the_possession_release_guard():
    """The departure search opens at ``throw - DEPARTURE_LEAD_S``; the previous
    toss's retention horizon closes at ``release - RELEASE_GUARD_S``. Same
    instant, so the two windows ABUT — no fall edge belongs to both (a good throw
    read as a bounce-out) and none falls between them (a real bounce-out
    attributed to the throw). Two copies of that number is exactly how the
    property dies quietly, so there is one and this pins the identity."""
    from jugglebot.ball_possession import RELEASE_GUARD_S
    assert tr.DEPARTURE_LEAD_S is RELEASE_GUARD_S


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
        return tr.label_from_sensor(samples, throw_time=THROW_T,
                                    landing_time=LANDING_T, windows=WINDOWS,
                                    **kw)

    assert under().label == tr.LABEL_BOUNCED                    # the defect
    got = under(next_release_time=next_release,
                next_landing_time=next_release + FLIGHT_S)
    assert got.label == tr.LABEL_CAUGHT
    assert got.confidence == 1.0


def test_a_real_bounce_out_survives_the_clamp():
    """The clamp excludes the announced throw window, NOT everything after the
    arrival. Trading the D1 mislabel for its mirror image — every bounce-out read
    as a catch — would re-open the trap C-POSSESS-1 § 7 spent a whole section
    accepting before the sensor closed it."""
    next_release = LANDING_T + R3_DWELL_S
    got = tr.label_from_sensor(
        stream(catch_dt=SEAT_DT_S, drop_after=0.20), throw_time=THROW_T,
        landing_time=LANDING_T, windows=WINDOWS,
        next_release_time=next_release,
        next_landing_time=next_release + FLIGHT_S)
    assert got.label == tr.LABEL_BOUNCED


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
    assert LANDING_T + SEAT_DT_S > next_release - tr.DEPARTURE_LEAD_S, (
        'premise: the seat edge lands PAST the horizon at this cadence')
    samples = stream(catch_dt=SEAT_DT_S,
                     drop_after=R5P_DWELL_S - SEAT_DT_S)
    got = tr.label_from_sensor(samples, throw_time=THROW_T,
                               landing_time=LANDING_T, windows=WINDOWS,
                               next_release_time=next_release,
                               next_landing_time=next_release + R5P_FLIGHT_S)
    assert got.label == tr.LABEL_CAUGHT
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
        return tr.label_from_sensor(samples, throw_time=THROW_T,
                                    landing_time=LANDING_T, windows=WINDOWS,
                                    **kw)

    assert under().label == tr.LABEL_CAUGHT                     # the edge, stolen
    assert under(next_release_time=next_release,
                 next_landing_time=next_land).label == tr.LABEL_MISSED


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


def test_the_corpus_and_the_live_verdict_share_ONE_arrival_boundary():
    """The same discipline ``DEPARTURE_LEAD_S``/``RELEASE_GUARD_S`` carries, one
    clamp along. Two implementations of "caught" that each derive the boundary
    from the constants will agree today and drift the first time one of them is
    edited; the miner is the R3 rung's gate instrument, so a drift there scores
    the machine against a definition the machine is not using."""
    from jugglebot.ball_possession import arrival_boundary_t
    assert tr.arrival_boundary_t is arrival_boundary_t


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
    assert tr.arrival_boundary_t(LANDING_T, LANDING_T + R5P_PERIOD_S,
                                 WINDOWS.arrival_lead_s) \
        >= LANDING_T + ARRIVAL_BAND_MAX_S
    samples = stream(catch_dt=catch_dt)

    # THE DEFECT: the shipped instant as a fixed window is arithmetically the
    # same search C-POSSESS-1.C built, and it cannot see the edge.
    narrow = tr.SensorWindows(arrival_lead_s=WINDOWS.arrival_lead_s,
                              arrival_window_s=shipped_close,
                              retention_window_s=WINDOWS.retention_window_s)
    missed = tr.label_from_sensor(samples, throw_time=THROW_T,
                                  landing_time=LANDING_T, windows=narrow)
    assert missed.fields['t_catch_deb_ros'] is None

    got = tr.label_from_sensor(samples, throw_time=THROW_T,
                               landing_time=LANDING_T, windows=WINDOWS,
                               next_landing_time=next_land)
    assert got.label == tr.LABEL_CAUGHT
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
    boundary_dt = tr.arrival_boundary_t(
        prev_land, LANDING_T, WINDOWS.arrival_lead_s) - LANDING_T
    stolen_dt = round(0.5 * (boundary_dt - WINDOWS.arrival_lead_s), 3)
    assert -WINDOWS.arrival_lead_s < stolen_dt < boundary_dt < 0.0, (
        'premise: the neighbour edge sits inside the unclamped opening but '
        'outside the clamped one')
    samples = stream(catch_dt=stolen_dt, dt=0.001)

    def under(**kw):
        return tr.label_from_sensor(samples, throw_time=THROW_T,
                                    landing_time=LANDING_T, windows=WINDOWS,
                                    **kw)

    assert under().label == tr.LABEL_CAUGHT                    # the edge, stolen
    assert under(prev_landing_time=prev_land).label == tr.LABEL_MISSED


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
        return tr.label_from_sensor(samples, throw_time=THROW_T,
                                    landing_time=LANDING_T, windows=WINDOWS,
                                    **kw)

    assert under().label == tr.LABEL_MISSED        # a search that DID watch out
    got = under(next_landing_time=next_land)
    assert got.label == tr.LABEL_UNKNOWN
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
        got = tr.label_from_sensor(stream(catch_dt=0.4), throw_time=THROW_T,
                                   landing_time=LANDING_T, windows=WINDOWS,
                                   **kw)
        assert (got.label, got.reason, got.confidence) == (
            base.label, base.reason, base.confidence)
