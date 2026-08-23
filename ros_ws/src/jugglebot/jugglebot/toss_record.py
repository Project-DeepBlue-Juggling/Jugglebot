"""The per-toss record — ONE schema, one encoder, one labeller, one latch.

Pure Python: **no ROS2, no file I/O, no config imports** — the
``ball_possession.py`` posture, for the same reason. Every window this module
needs is *injected* by its caller, so the live node and the offline miner cannot
drift onto different constants without a test noticing.

Plan: ``plans/active/toss-selftuning.md`` §§ 3.3, 3.4 (schema and dataflow),
D10-D13 (why the record has this shape). Logbook:
``logbook/2026-08-10-toss-selftuning-build.md``.
Consumers: ``reload_coordinator_node`` (declaration half) and
``tools/probes/toss_record_miner.py`` (measurement half + the join).

WHY THIS MODULE EXISTS (D11)
----------------------------
The record is minted in **two halves that must agree**: the coordinator declares
what it commanded, and an offline miner measures what happened. If each half
carried its own idea of "caught", the two would drift and nobody would find out
until a calibration map had been fitted on the wrong labels. The precedent is
already load-bearing in this repo — ``possession_verdict_bag_check.py`` builds the
arrival source *exactly as the node builds it*, deliberately, and Phase 1's
``hand_sensor_verdict_replay.py`` reconciled to the sample against independent
hand-counted transitions because of it.

So: **one definition of "caught", and it lives in the installed package**, not in
a probe. ``label_from_sensor`` is that definition.

WHERE THE LABEL IS MINTED, AND WHY NOT IN THE NODE
--------------------------------------------------
The node **cannot** label at its own terminal, and this is structural rather than
a choice. A toss terminalises at ``landing + CATCH_CONFIRM_WINDOW_S`` (0.70 s),
while the label needs the *retention* window (1.50 s past the arrival edge) to
tell CAUGHT from BOUNCED. The node therefore publishes a DECLARATION and the
label is derived offline — which is also why ``label`` carries origin ``X``
(derived at join) in :data:`FIELDS` rather than ``D``. The node still imports this
module, for :func:`encode` and the schema, so there is exactly one wire format.

CLOCK DOMAINS (plan § 3.3, verbatim)
------------------------------------
Five clocks are in play and **every field names exactly one**:

``perf``    ``time.perf_counter()`` — machine-global monotonic, never in a bag.
``ros``     wall, what a ``header.stamp`` carries.
``bag``     mcap ``log_time_ns`` — the ONLY clock available for ``/mocap_data``,
            which has no header.
``bridge``  ``uptime_ms``; and ``ball_held_stamp``, which is wall-epoch **only
            while ``ball_held_valid``** and after the bridge's wall anchor lands.
            Before that it is bridge-boot-relative, and joining it to wall time
            mis-times every edge by the whole bridge uptime — the largest silent
            error available anywhere in this pipeline (plan § 7 R2). Hence
            ``ball_held_stamp_wall_anchored`` in the record, and hence this module
            never reads ``ball_held_stamp`` for an edge TIME: edges are taken on
            the ``/hand_telemetry`` sample instants the caller supplies.
``qtm``     related to ``ros`` by ``/qtm_clock_offset_sec``.

RAW EDGES FOR TIMES, DEBOUNCED FOR THE VERDICT (D12), **measured**
------------------------------------------------------------------
The can-bridge debounce is documented as **asymmetric** — five consecutive EMPTY
good replies to drop, any single HELD restores
(``plans/archived/hand-ball-sensor.md`` § Debounce asymmetry). Measured on
``~/Desktop/rosbags/2026-08-10_16-30-44`` (2026-08-10, 70,666 ``/hand_telemetry``
samples, ``ball_held_valid`` 100 %):

    empty->held (the CATCH edge)   all 38 edges   min 0    med 0    max 0    ms
    held->empty (the DEPARTURE)    all 39 edges   min 232  med 241  max 295  ms
    held->empty, self-toss only    31 releases    min 232  med 240  max 251  ms

So the departure edge — the one that feeds every timing fit — is **~241 ms late
on the debounced bit**, 2.4x the ~100 ms the plan estimated from a nominal 50 Hz
poll, and comparable to the +118-133 ms uptime dispatch shift the whole fresh-boot
discipline exists to control. It would have read as real physics. The catch edge,
by contrast, is debounce-free, so ``t_catch_raw_ros`` and ``t_catch_deb_ros``
agree on that bag by construction.

The same capture also measures the poll cadence the plan assumed: consecutive
``ball_held_stamp`` advances run **p5 32 / median 71 / p95 111 ms**, against a
configured ``JB_BD_CHECK_INTERVAL_MS`` of 20. That is why
``sensor_poll_dt_ms_median`` is a MEASURED field and never an assumed one.
"""

from __future__ import annotations

import json
import math
from typing import Any, Dict, Iterable, List, NamedTuple, Optional, Sequence, Tuple

# The ONE import this otherwise self-contained module takes, and it is
# deliberate: the departure guard below and the possession source's retention
# horizon are the SAME instant, so they must be the same constant. See
# DEPARTURE_LEAD_S. `ball_possession` is pure Python with no further jugglebot
# imports, so this cannot cycle.
from jugglebot.ball_possession import (ARRIVAL_BAND_MAX_S, RELEASE_GUARD_S,
                                       arrival_boundary_t)

# ── Schema version ────────────────────────────────────────────────────────────
#: Bumps on field REMOVAL or a semantic change; purely additive fields do not
#: bump (plan § 3.7). ``tests/motion/test_toss_record.py`` pins both the version
#: and the whole field list, so an accidental removal is a red test, not a corpus
#: that silently stops joining.
SCHEMA = 'toss_record/1'

#: Join tolerance on ``announce_throw_time_ros`` (plan § 3.4). Over-determined by
#: ~1000x: the 3.5 s throw-delay floor and the 5.60 s dwell floor put consecutive
#: tosses >= 5 s apart, and the node writes the SAME float into
#: ``ThrowAnnouncement.throw_time`` and into the declaration, so a real match is
#: exact and 5 ms only absorbs float round-tripping through JSON.
JOIN_TOL_S = 0.005

# ── Labels ────────────────────────────────────────────────────────────────────
LABEL_CAUGHT = 'CAUGHT'
LABEL_BOUNCED = 'BOUNCED'
LABEL_MISSED = 'MISSED'
LABEL_NO_RELEASE = 'NO_RELEASE'
LABEL_UNKNOWN = 'UNKNOWN'

LABELS = (LABEL_CAUGHT, LABEL_BOUNCED, LABEL_MISSED, LABEL_NO_RELEASE,
          LABEL_UNKNOWN)

#: Provenance of a joined row (plan § 3.3, ``record_provenance``).
PROV_BOTH = 'declared+mined'
PROV_MINED = 'mined-only'
PROV_DECLARED = 'declared-only'

# ── Record-specific search windows (MEASURED, not inherited) ──────────────────
# The arrival and retention windows are the SHIPPED ``JB_BD_*`` constants and are
# INJECTED (see SensorWindows) so this module and HandBallSensorSource cannot
# disagree. The two below have no shipped equivalent — they exist only to bound
# the DEPARTURE search — so they are pinned here, from measurement.
#
# Measured on 2026-08-10_16-30-44 (2026-08-10): across all 32 ``jugglebot``
# self-tosses the RAW held->empty edge lands at **+148 .. +212 ms** after the
# announced ``throw_time`` (median +172 ms); the debounced edge at +385 .. +455.
# Every ``ball_butler`` announcement in the same bag has NO departure edge, which
# is the correct answer — a Butler ball never leaves our cup.
#
#: How far BEFORE the announced throw_time a departure edge is still ours. The
#: measured departure-edge shifts are all positive, but that is NOT evidence the
#: release runs late: the lag tracks the SENSOR POLL CADENCE rather than dispatch
#: — the mocap backcast of the same tosses puts the release 4.6 ms EARLY
#: (−4.6 ms; see the 2026-08-12 addendum in
#: ``logbook/2026-08-10-toss-selftuning-build.md``). So this lead is headroom for
#: the sensor channel's own lag, and for the fault direction the edges cannot
#: resolve — an early release is a real defect and must be visible as a number
#: rather than invisible as a missing edge.
#:
#: **It is an ALIAS as of 2026-08-21 (census D1), not an independent number.**
#: ``ball_possession.RELEASE_GUARD_S`` is the same instant seen from the other
#: side: ``release - guard`` opens this module's departure search AND closes the
#: previous toss's retention horizon. Two copies of that number is how the two
#: windows start to overlap (a legitimate throw read as a bounce-out) or leave a
#: gap (a real bounce-out attributed to the throw), so there is one. Pinned equal
#: by ``tests/motion/test_toss_record.py::
#: test_departure_lead_is_the_possession_release_guard``.
DEPARTURE_LEAD_S = RELEASE_GUARD_S
#: How far after it. 4.7x the measured worst case (+212 ms), and the window is
#: additionally clamped to end at ``landing - arrival_lead`` so it can never
#: overlap the arrival search and mistake a bounce-out for a departure.
DEPARTURE_WINDOW_S = 1.00


class SensorWindows(NamedTuple):
    """The hand-sensor windows, INJECTED — never imported.

    ``arrival_lead_s`` / ``arrival_window_s`` / ``retention_window_s`` are the
    shipped ``JB_BD_*`` values; both callers build this from the generated config
    (the node from ``hw``, the miner from ``hw``) and a drift-guard test pins the
    miner's construction equal to ``hw``. Same discipline as
    ``ball_possession.HandBallSensorSource``, and for the same reason: a bench
    instrument that scores a capture against windows the robot never ran agrees
    with the robot only by coincidence.

    **Do NOT substitute ``toss_sequencer.CATCH_CONFIRM_WINDOW_S`` for
    ``arrival_window_s``.** That constant is the FSM's terminal deadline, not a
    sensor search window; the plan's § 3.3 draft used it. Measured on the
    reference bag (2026-08-10, 25 catches): at the 0.70 s it then carried,
    exactly ONE arrival relabels MISSED — the +798 ms row, which is the
    population MAXIMUM. The runner-up (+675 ms) sits 25 ms inside that boundary,
    so the margin was one session's variation wide.

    > **Updated 2026-08-21 (census D7).** ``CATCH_CONFIRM_WINDOW_S`` is now
    > derived from ``ball_possession.ARRIVAL_BAND_MAX_S`` (0.80 s — the measured
    > band ceiling, rounded up), so it no longer sits BELOW the band it has to
    > outlast. The two constants remain different quantities and must not be
    > substituted for one another: 0.80 s is the band *ceiling* a deadline must
    > clear, 1.50 s is the search *window* sized with margin above it.
    """
    arrival_lead_s: float
    arrival_window_s: float
    retention_window_s: float
    departure_lead_s: float = DEPARTURE_LEAD_S
    departure_window_s: float = DEPARTURE_WINDOW_S


# ── The schema ────────────────────────────────────────────────────────────────

class Field(NamedTuple):
    """One record field.

    ``origin``  'D' declared by the node | 'M' mined offline | 'DM' both
                (independently recoverable — which is what makes the join
                self-checking) | 'X' derived at join.
    ``kind``    's' str | 'f' float | 'i' int | 'b' bool | 'f2'/'f3' fixed-length
                float lists | 'l' list | 'o' object.
    Every field is nullable unless ``required``; ``None`` is the only null, and
    NaN is never encoded (JSON has no NaN, and a bare ``NaN`` token is not
    portable JSON — :func:`encode` maps non-finite floats to ``None``).
    """
    name: str
    block: str
    origin: str
    kind: str
    doc: str
    required: bool = False


FIELDS: Tuple[Field, ...] = (
    # ── Identity ──────────────────────────────────────────────────────────────
    Field('schema', 'identity', 'D', 's', 'wire schema version', True),
    Field('toss_uid', 'identity', 'D', 's',
          '<session_id>-<goal_id8>-<cycle_index>', True),
    Field('session_id', 'identity', 'D', 's', 'one node launch'),
    Field('goal_id', 'identity', 'D', 's', 'action goal uuid, hex'),
    Field('action', 'identity', 'D', 's', 'toss | toss_continuous', True),
    Field('cycle_index', 'identity', 'DM', 'i',
          '1-based within a session; 1 for a single Toss'),
    Field('announce_throw_time_ros', 'identity', 'DM', 'f',
          'THE JOIN KEY — ThrowAnnouncement.throw_time, ros'),
    Field('announce_landing_time_ros', 'identity', 'DM', 'f',
          'ThrowAnnouncement.landing_time, ros'),
    Field('t_record_ros', 'identity', 'D', 'f', 'when the declaration was minted'),
    Field('perf_minus_ros_s', 'identity', 'D', 'f',
          'FILTERED clock_offset (10 samples, 20-deep history)'),
    Field('perf_minus_ros_inst_s', 'identity', 'D', 'f',
          'single-read offset — the _announcement_landing_perf variant, carried '
          'so the standing reconciliation gap is a measurement not an argument'),

    # ── Provenance ────────────────────────────────────────────────────────────
    Field('git_sha', 'provenance', 'D', 's', 'HEAD at node launch'),
    Field('git_dirty', 'provenance', 'D', 'b', 'working tree dirty at launch'),
    Field('toss_cal_version', 'provenance', 'D', 's', 'phase 2b: motion/toss_cal.map_version'),
    Field('toss_cal_loaded', 'provenance', 'D', 'b', 'a map is present + valid'),
    Field('toss_cal_applied', 'provenance', 'D', 'b',
          'loaded AND the D3 provenance gate agrees — loaded is not applied'),
    Field('tilt_map_version', 'provenance', 'DM', 's', 'TrajectoryStatus'),
    Field('tilt_map_applied', 'provenance', 'DM', 'b',
          'tilt_map_loaded AND gravity_correction_loaded, read together'),
    Field('gravity_correction_loaded', 'provenance', 'DM', 'b', 'TrajectoryStatus'),
    Field('level_offset_rad', 'provenance', 'DM', 'f2', 'the session level'),
    Field('toss_tier', 'provenance', 'D', 's', '8a | 8b'),
    Field('bridge_fw_version', 'provenance', 'M', 's', 'link_status'),
    Field('platform_fw_version', 'provenance', 'M', 's', 'link_status'),
    Field('uptime_ms_at_release', 'provenance', 'M', 'i',
          'can-bridge uptime at the release instant — the R3 partition key'),
    Field('hand_odrive_config_sha', 'provenance', 'D', 's',
          'nullable: a KNOWN GAP in the partition key, never an assumed match'),
    Field('catch_knobs', 'provenance', 'D', 'o', 'the catch-side tunables'),

    # ── Goal as requested ─────────────────────────────────────────────────────
    Field('goal_catch_xyz_stow_mm', 'goal', 'D', 'f3', 'nominated catch pose B'),
    Field('goal_throw_height_m', 'goal', 'D', 'f', 'RESOLVED'),
    Field('goal_throw_height_m_raw', 'goal', 'D', 'f',
          'AS REQUESTED, 0-sentinel preserved'),
    Field('goal_throw_delay_s', 'goal', 'D', 'f', 'RESOLVED'),
    Field('goal_throw_delay_s_raw', 'goal', 'D', 'f', 'AS REQUESTED'),
    Field('goal_catch_vel_scale', 'goal', 'D', 'f', 'RESOLVED'),
    Field('goal_catch_vel_scale_raw', 'goal', 'D', 'f', 'AS REQUESTED'),
    Field('goal_num_throws', 'goal', 'D', 'i', 'session only'),
    Field('goal_dwell_time_s', 'goal', 'D', 'f', 'session only, RESOLVED'),
    Field('goal_stop_on_miss', 'goal', 'D', 'b', 'session only'),
    Field('goal_on_empty_cup', 'goal', 'D', 's', 'session only, null until 2d'),
    Field('goal_max_reloads', 'goal', 'D', 'i', 'session only, null until 2d'),

    # ── Resolved release ──────────────────────────────────────────────────────
    Field('flight_time_s', 'release', 'D', 'f', 'internal flight time'),
    Field('apex_height_m', 'release', 'D', 'f', 'apex above the release plane'),
    Field('event_vel_mps', 'release', 'D', 'f', 'as sent to the hand'),
    Field('event_delay_s', 'release', 'D', 'f', 'as sent, already latency-shifted'),
    Field('release_latency_ms_applied', 'release', 'D', 'f', 'the config shift'),
    Field('release_pos_global_mm', 'release', 'D', 'f3', ''),
    Field('launch_vel_mms', 'release', 'D', 'f3', ''),
    Field('catch_point_global_mm', 'release', 'D', 'f3',
          'via the SINGLE conversion point toss_release.stow_to_global_mm; the '
          'miner recomputes it independently and fails LOUD on a mismatch '
          '(plan § 7 R1)'),
    Field('aim_tilt_rx_rad', 'release', 'D', 'f', 'commanded throw tilt, 0 in 8a'),
    Field('aim_tilt_ry_rad', 'release', 'D', 'f', ''),
    Field('throw_site_xy_mm', 'release', 'D', 'f2', 'tier 8b throw site A'),

    # ── Applied calibration (all null/zero through phase 2a) ──────────────────
    Field('map_aim_rad', 'calibration', 'D', 'f2', 'phase 2b'),
    Field('trim_aim_rad', 'calibration', 'D', 'f2',
          'layer 2 CONTRIBUTED to the commanded aim. A STRUCTURAL zero since '
          '2026-08-21 — the layer-2 aim estimator is monitor-only (C4)'),
    # ADDITIVE, so no schema bump (§ 3.7 item 1). Deliberately a SEPARATE key
    # from trim_aim_rad rather than a repurposing of it: everything that
    # reconstructs the applied aim from a record — toss_trim.applied_aim_rad's
    # map+trim fallback, the miner, ilc_fit_lib — reads trim_aim_rad as "what was
    # applied", and folding a monitor value into that key would re-apply it on
    # paper, which is exactly the C4 double-count wearing a different hat.
    Field('trim_monitor_aim_rad', 'calibration', 'D', 'f2',
          'layer 2 ESTIMATED but did not command, rad. Its divergence from '
          'ilc_aim_rad is the standing validation of the C3 by-decision '
          'resolution (the two reduce different residual channels)'),
    Field('trim_authority', 'calibration', 'D', 's',
          "MONITOR since 2026-08-21 — toss_trim.AIM_AUTHORITY, recorded so a "
          "corpus can prove which build's records carry a commanded trim"),
    Field('total_aim_rad', 'calibration', 'D', 'f2', 'what was actually commanded'),
    Field('map_aim_mm_at_h', 'calibration', 'D', 'f2',
          'REPORT field: mm at THIS toss apex, never the stored unit'),
    Field('trim_aim_mm_at_h', 'calibration', 'D', 'f2', 'REPORT field'),
    # Layer 3 — the critical-point ILC correction (critical-point-ilc.md
    # Phase 2). ADDITIVE fields, so no schema bump (§ 3.7 item 1).
    #
    # POST-GATE, both of them, and that is the whole reason they exist: the plan's
    # risk 5 is that a correction partially truncated by the D7 total-aim clamp
    # (or refused by validate_event_vel) desynchronises applied-u from recorded-u,
    # and the learner then fits against a command the machine never flew. The
    # apply seam refuses rather than truncates and writes the REFUSED value —
    # exactly (0, 0) / 0.0 — here. `total_aim_rad` already carries the sum, so
    # these two say how much of it layer 3 asked for AND GOT.
    Field('ilc_aim_rad', 'calibration', 'D', 'f2',
          'layer 3: the TOTAL ILC aim contribution APPLIED, rad — spatial + '
          'session. Explicit zeros when the feature is off, the artifact is '
          'absent/dormant, the goal cell missed with no session common mode, or '
          'the total-aim clamp REFUSED it'),
    # The C1 split of the field above. ADDITIVE, so no schema bump (s 3.7 item
    # 1), and deliberately a SPLIT rather than a replacement: everything that
    # subtracts what layer 3 applied -- toss_trim.ilc_aim_rad's C4 subtraction
    # first among them -- needs the SUM, and a consumer that had to add two
    # fields to get it would eventually add only one.
    Field('ilc_spatial_aim_rad', 'calibration', 'D', 'f2',
          'layer 3a: the per-cell SPATIAL RESIDUAL applied, rad. Exactly zero '
          'on a cell miss -- toss_ilc.lookup interpolates nothing'),
    Field('ilc_session_aim_rad', 'calibration', 'D', 'f2',
          'layer 3b: the SESSION-LOCAL common mode applied, rad (C1). RAM only, '
          'seeded from the artifact anchor prior, discarded at goal end, and '
          'NOT keyed on a cell hit -- a common mode is not a function of the '
          'cell. This is the quantity a re-level() moves, kept out of every '
          'persisted cell on purpose'),
    Field('ilc_session_applied', 'calibration', 'D', 'b',
          'layer 3b cleared its evidence gate and was commanded'),
    Field('ilc_session_reason', 'calibration', 'D', 's',
          'why layer 3b commanded nothing: no_artifact | no_anchor | '
          'insufficient_evidence | below_se_gate | inside_deadband | '
          'refused_total_aim. Empty when applied -- "commanded nothing" and '
          '"had nothing to command" are different facts about a session. '
          'refused_total_aim is the only one the session component does not '
          'mint itself: the node D7 clamp drops BOTH layer-3 components when '
          'map+ilc exceeds the authority, and before 2026-08-22 this field kept '
          'reading APPLIED on those goals'),
    Field('ilc_session_n', 'calibration', 'D', 'i',
          'independent evidence units (sessions, i.e. level() draws) behind the '
          'anchor prior; 0 when there is none'),
    Field('ilc_vel_trim', 'calibration', 'D', 'f',
          'layer 3: the ILC event_vel trim APPLIED, k_v - 1. Explicit zero on '
          'every path above plus a validate_event_vel refusal'),
    Field('speed_bias_applied', 'calibration', 'D', 'f', 'k_v, phase 2e'),
    Field('timing_bias_applied_ms', 'calibration', 'D', 'f', 'tau, phase 2e'),
    Field('clamp_hits', 'calibration', 'D', 'l', 'per-channel clamp names'),
    Field('trim_source_n', 'calibration', 'D', 'i', ''),
    Field('trim_state', 'calibration', 'D', 's',
          'WARMUP | ACTIVE | CONVERGED | FROZEN_<reason>'),
    Field('trim_reset_reason', 'calibration', 'D', 's', ''),

    # ── Dwell tilt (Layer 1.5) — COVARIATE ONLY, zero control authority ───────
    # Schema lands in 2a (this phase); the dwell read schedule is wired in 2d.
    # Until then every field here is null, which is a LEGAL record: plan § 3.10's
    # degrade-never-delay rule already makes ``dwell_tilt_n = 0`` legal, so the
    # analysers must tolerate absence from day one rather than from 2d.
    Field('dwell_tilt_rad', 'dwell_tilt', 'D', 'f2', 'mean of the N reads'),
    Field('dwell_tilt_sd_rad', 'dwell_tilt', 'D', 'f2', ''),
    Field('dwell_tilt_n', 'dwell_tilt', 'D', 'i', '0 is legal'),
    Field('dwell_tilt_span_s', 'dwell_tilt', 'D', 'f', 'first read -> last read'),
    Field('dwell_tilt_last_read_to_release_s', 'dwell_tilt', 'D', 'f',
          'proves the reads never overlapped PREPARE->THROW'),
    Field('dwell_tilt_degraded', 'dwell_tilt', 'D', 'b',
          'read count reduced to protect the throw'),

    # ── FSM / dispatch ────────────────────────────────────────────────────────
    Field('outcome', 'fsm', 'D', 's', 'verbatim TossResult.outcome', True),
    Field('success', 'fsm', 'D', 'b', 'verbatim TossResult.success'),
    Field('phase_at_terminal', 'fsm', 'D', 's', ''),
    Field('throw_dispatch_class', 'fsm', 'D', 's', 'ok | ambiguous | rejected'),
    Field('throw_dispatch_message', 'fsm', 'D', 's', ''),
    Field('prepare_ok', 'fsm', 'D', 'b', ''),
    Field('position_accepted', 'fsm', 'D', 'b', ''),
    Field('position_planned_s', 'fsm', 'D', 'f', ''),
    Field('position_code', 'fsm', 'D', 's', ''),
    Field('catch_target_accepted', 'fsm', 'D', 'b', ''),
    Field('announce_lead_short', 'fsm', 'D', 'b', ''),
    Field('throw_stroke_seen', 'fsm', 'D', 'b', 'G1 release evidence'),
    Field('ball_track_confirmed', 'fsm', 'D', 'b', 'G1 release evidence'),
    Field('t_accept_perf', 'fsm', 'D', 'f', ''),
    Field('t_release_perf', 'fsm', 'D', 'f', ''),
    Field('t_landing_sched_perf', 'fsm', 'D', 'f', ''),
    Field('reload_settle', 'fsm', 'D', 'b',
          'the cycle after a reload interlude — guard G10. null until 2d'),
    Field('retry_of', 'fsm', 'D', 's',
          'the toss_uid this cycle retried — guard G11. null until 2d'),
    # Diagnostics, NOT truth. catch_error_mm is the tracker Kalman filter's
    # dead-reckoned free-fall extrapolation and is FORBIDDEN as an estimator
    # input (plan F5 / D5); it is recorded so that forbidding it is checkable.
    Field('achieved_flight_s_fsm', 'fsm', 'D', 'f', 'diagnostic, not truth'),
    Field('catch_error_mm_fsm', 'fsm', 'D', 'f',
          'diagnostic ONLY — never an estimator input (D5)'),
    Field('catch_event_dt_s_fsm', 'fsm', 'D', 'f',
          'the live hand-sensor catch_dt the node already logs'),

    # ── Sensor (mined from /hand_telemetry) ───────────────────────────────────
    Field('sensor_valid_frac', 'sensor', 'M', 'f',
          'ball_held_valid fraction over the DECISIVE window'),
    Field('sensor_n_samples', 'sensor', 'M', 'i', 'samples in the decisive window'),
    Field('sensor_held_at_dispatch', 'sensor', 'M', 'b', ''),
    Field('t_departure_raw_ros', 'sensor', 'M', 'f', 'RAW held->empty — a TIME'),
    Field('t_departure_deb_ros', 'sensor', 'M', 'f',
          'DEBOUNCED — carried only to MEASURE the ~241 ms asymmetric lag'),
    Field('t_catch_raw_ros', 'sensor', 'M', 'f', 'RAW empty->held — a TIME'),
    Field('t_catch_deb_ros', 'sensor', 'M', 'f', 'DEBOUNCED — the VERDICT edge'),
    Field('t_dropout_ros', 'sensor', 'M', 'f', 'first DEBOUNCED fall after a catch'),
    Field('held_at_catch_plus_retention', 'sensor', 'M', 'b', ''),
    Field('sensor_edge_count', 'sensor', 'M', 'i',
          'DEBOUNCED edges in the decisive window; > 2 is a rimshot candidate'),
    Field('sensor_poll_dt_ms_median', 'sensor', 'M', 'f',
          'MEASURED from ball_held_stamp advances — never assumed'),
    Field('ball_held_stamp_wall_anchored', 'sensor', 'M', 'b',
          'false => every edge would be mis-timed by the whole bridge uptime'),

    # ── Mocap (mined; estimator IMPORTED from tools/probes/ball_arrival_offset) ─
    Field('land_xy_global_mm', 'mocap', 'M', 'f2', ''),
    Field('land_err_mm', 'mocap', 'M', 'f2',
          'land_xy - catch_point_global_mm[:2] — THE aim observable'),
    Field('land_err_norm_mm', 'mocap', 'M', 'f', ''),
    Field('n_fit', 'mocap', 'M', 'i', ''),
    Field('fit_rms_mm', 'mocap', 'M', 'f', 'G2 track quality'),
    Field('fit_sparse', 'mocap', 'M', 'b', ''),
    Field('apex_z_mm', 'mocap', 'M', 'f', ''),
    Field('achieved_flight_s_mocap', 'mocap', 'M', 'f',
          'MEASURED release -> catch-plane crossing, both instants read off the '
          'SAME mocap arc in the bag clock (so no clock crossing enters it). '
          'Defined by tools/probes/toss_record_miner.mine_arc; null in every '
          'corpus written before 2026-08-12'),
    Field('t_land_bag', 'mocap', 'M', 'f', 'bag clock — /mocap_data has no header'),
    Field('qtm_offset_s', 'mocap', 'M', 'f', ''),
    Field('mocap_gap_ms_max', 'mocap', 'M', 'f', ''),
    Field('land_plane_mm', 'mocap', 'M', 'f',
          'the fit plane USED — so a historical plane mismatch is detectable'),
    Field('floor_arrival', 'mocap', 'M', 'i', 'REPORT-only floor census'),

    # ── Command reference for the mined errors (ILC Phase 0a/0c) ──────────────
    # The commanded release state the arrival/backcast errors are differenced
    # against, RECORDED rather than assumed: on a mined-only row every 'D' field
    # (launch_vel_mms, flight_time_s) is null by construction, so without these
    # the errors below would not be auditable from the corpus alone.
    Field('cmd_launch_vel_mms', 'command_ref', 'M', 'f3',
          'commanded release velocity the mined errors are taken against; '
          'equals the declared launch_vel_mms when a declaration joined'),
    Field('cmd_flight_time_s', 'command_ref', 'M', 'f',
          'commanded release->cup flight time, same source as '
          'cmd_launch_vel_mms'),
    Field('cmd_release_source', 'command_ref', 'M', 's',
          "where cmd_* came from. 'announcement' is the only value the miner "
          'emits: the announcement is filled from the same ReleaseState the '
          'declaration reports, and it is the ONLY source on a bag predating '
          '/toss/record. The slot is named rather than assumed so a corpus can '
          'still say so if that ever changes'),

    # ── The WHOLE-ARC fit (ILC entry condition E-1, resolved 2026-08-13) ──────
    # Definition point: tools/probes/toss_record_miner.mine_arc. A THIRD ballistic
    # fit over the union of the two branches, and it owns every LATERAL velocity
    # component below. WHY, in one paragraph: the tracked point is the centroid of
    # the visible retroreflective cap, so it carries a height-locked position bias
    # b(z) of ~20 mm; height is EVEN about a ballistic apex, so a per-branch line
    # fit reads v_true +- <db/dz.|vz|> and the two branches disagree by 104 mm/s
    # on exactly the aim channels, repeatably. A whole-arc slope cancels that by
    # parity, leaving only the sample coverage's asymmetry about the apex — which
    # is what `coverage_asym_s` measures and what gates `usable_for_lateral_fit`.
    # Evidence and the refuted alternatives: plans/active/critical-point-ilc.md
    # § Phase 1 E-1; probe tools/probes/mocap_parity_bias.py.
    Field('arc_fit_n', 'arc', 'M', 'i',
          'rows in the WHOLE-ARC fit (both branches, apex row de-duplicated)'),
    Field('arc_fit_rms_mm', 'arc', 'M', 'f',
          '3-D RMS residual of the whole-arc ballistic fit'),
    Field('arc_lateral_vel_se_mms', 'arc', 'M', 'f',
          'worst of the two 1-sigma standard errors on the fitted LATERAL '
          'velocities — the noise floor of every direction channel below'),
    Field('coverage_asym_s', 'arc', 'M', 'f',
          'mean sample time MINUS the fitted apex instant, seconds. The one '
          'term the whole-arc estimator does not cancel: cov(tau, b) is exactly '
          'zero for coverage symmetric about the apex, so this is the bias-leak '
          'driver. A GROSS-TRUNCATION guard, not a leak predictor (one 0.0003 s '
          'arc still leaks 6 mm/s). Measured on the 2026-08-13 re-mine, by '
          'population: over the 19 usable_for_release_fit rows, 0.0001-0.0732 s '
          '(median 0.0148), 0 refused; over all 32 mined arcs, median 0.052 s, '
          'worst 0.600 s, 12 refused — every one a half-seen arc the '
          'release-fit gate already refuses. Refused past '
          'COVERAGE_ASYM_MAX_S = 0.1 s'),

    # ── Arrival kinematics (ILC Phase 0a; VERTICAL from the DESCENDING branch) ─
    # Definition point: tools/probes/toss_record_miner.mine_arc. The nominal is
    # ballistics_bc.arrival_velocity(cmd_launch_vel_mms, cmd_flight_time_s) —
    # the PRODUCTION function, never a second copy (plan constraint 1), and it
    # is deliberately NOT stored: one production call reproduces it, a stored
    # copy could drift from it.
    Field('arrival_vel_mms', 'arrival', 'M', 'f3',
          'MEASURED ball velocity at the catch-plane crossing (global mm/s). '
          'xy from the WHOLE-ARC fit (E-1), z from the descending branch'),
    Field('arrival_dir_err_rad', 'arrival', 'M', 'f2',
          'measured - nominal per-axis lean of the arrival velocity off the '
          'vertical it is travelling along: [atan2(vx,|vz|), atan2(vy,|vz|)]. '
          'Bias-immune since E-1: the numerator is whole-arc'),
    Field('arrival_dir_err_norm_rad', 'arrival', 'M', 'f',
          'total angle between the measured and nominal arrival directions'),
    Field('arrival_speed_err_mms', 'arrival', 'M', 'f',
          '|v_measured| - |v_nominal| at the catch plane'),
    Field('t_arrival_fit_bag', 'arrival', 'M', 'f',
          'MEASURED catch-plane crossing instant, bag clock (t_land_bag is the '
          'ANNOUNCED landing crossed into the bag clock — not the same number)'),
    Field('arrival_fit_n', 'arrival', 'M', 'i', 'rows in the descending fit'),
    Field('arrival_fit_rms_mm', 'arrival', 'M', 'f',
          '3-D RMS residual of the descending ballistic fit'),
    Field('arrival_vel_se_mms', 'arrival', 'M', 'f',
          '1-sigma standard error on the fitted VERTICAL arrival velocity. THE '
          'noise floor this phase owes Phase 1 — a short branch fits a clean '
          'RMS and a badly wrong velocity, and only this number says so. The '
          'lateral components are ~10x better determined (their residual is '
          '2-8 mm against 25-34 mm in z, because the bag clock error shows up '
          'multiplied by the axis speed)'),
    Field('flight_time_err_s', 'arrival', 'M', 'f',
          'achieved_flight_s_mocap - cmd_flight_time_s'),

    # ── Release-state backcast (ILC Phase 0c; ASCENDING branch) ──────────────
    # The throw critical point's INPUT-side truth. Everything here is
    # attributable to the RELEASE (hand stroke, dispatch timing, aim);
    # everything in `land_err_flight_mm` below is what the release does NOT
    # explain. That split is the discriminator the whole ILC leans on.
    Field('release_pos_track_mm', 'backcast', 'M', 'f3',
          'MEASURED release point: the ascending fit evaluated where it crosses '
          'the release plane'),
    Field('release_vel_track_mms', 'backcast', 'M', 'f3',
          'MEASURED release velocity at that crossing (global mm/s). xy from '
          'the WHOLE-ARC fit (E-1), z from the ascending branch'),
    Field('t_release_fit_bag', 'backcast', 'M', 'f',
          'MEASURED release instant, bag clock'),
    Field('release_time_err_ms', 'backcast', 'M', 'f',
          't_release_fit crossed to ros MINUS announce_throw_time_ros — an '
          'INDEPENDENT dispatch-shift measurement (the sensor departure edge is '
          'the other one, and they disagree; see the 0a/0c logbook)'),
    Field('release_vel_err_mms', 'backcast', 'M', 'f3',
          'release_vel_track_mms - cmd_launch_vel_mms, per axis'),
    Field('release_speed_err_mms', 'backcast', 'M', 'f',
          '|measured| - |commanded| release speed'),
    Field('release_dir_err_rad', 'backcast', 'M', 'f2',
          'per-axis lean error of the release velocity, same convention as '
          'arrival_dir_err_rad, and whole-arc in the same sense. NOTE the '
          'consequence: release and arrival now share ONE lateral velocity, so '
          'these two channels differ only through their |vz| denominators and '
          'their nominals — they are not independent lateral measurements'),
    Field('backcast_fit_n', 'backcast', 'M', 'i', 'rows in the ascending fit'),
    Field('backcast_fit_rms_mm', 'backcast', 'M', 'f',
          '3-D RMS residual of the ascending ballistic fit'),
    Field('release_vel_se_mms', 'backcast', 'M', 'f',
          '1-sigma standard error on the fitted VERTICAL release velocity — the '
          'arrival_vel_se_mms twin, and the same warning applies'),

    # ── Release-vs-flight split of the landing error (ILC Phase 0c) ──────────
    # By construction land_err_release_mm + land_err_flight_mm == land_err_mm,
    # exactly. The first half is what the MEASURED release state already
    # predicts (propagated to the cup plane by the production ballistics).
    #
    # RE-MARKED 2026-08-13 (E-1's resolution). Both halves are LATERAL (xy)
    # quantities, and under the whole-arc estimator the second half is NOT
    # measurable flight-phase physics:
    #   * the parity decomposition puts an upper bound on any lateral
    #     aerodynamic force well under the artefact it was competing with —
    #     Magnus would need 3.4-8.8 rev/s against an observed ~0.5, and would
    #     land in the ODD channel about the apex, which is empty;
    #   * so what remains in land_err_flight_mm is the disagreement between two
    #     ESTIMATORS of the same crossing — the descending-band position fit
    #     behind land_xy_global_mm, against the release state propagated
    #     forward — plus the unmeasured ABSOLUTE centroid bias, which both
    #     halves carry and neither resolves.
    # Read it as an estimator-agreement diagnostic. **Lateral landing error is
    # RELEASE-side**, and nothing may fit land_err_flight_mm as a flight
    # channel. The VERTICAL split is untouched and stays meaningful:
    # release_speed_err_mms against flight_time_err_s are two disjoint branches
    # measuring one throw, which is exactly what the ILC's V2b cross-check
    # leans on.
    Field('land_err_release_mm', 'split', 'M', 'f2',
          'RELEASE-attributable part of land_err_mm — where the MEASURED '
          'release state lands under production ballistics, minus the cup. It '
          'equals "measured release vs COMMANDED release" only because the '
          'announced landing IS the commanded release own ballistic landing '
          '(build_announcement_fields); an announcement inconsistent with its '
          'own release state would land its inconsistency here. Since E-1 this '
          'is the whole of the lateral landing error, to estimator agreement'),
    Field('land_err_flight_mm', 'split', 'M', 'f2',
          'land_err_mm minus the release-attributable part. NOT a flight-physics '
          'channel since E-1 (2026-08-13) — an estimator-agreement diagnostic; '
          'see the block comment above'),

    # ── Plant (mined; row builder IMPORTED from hand_stroke_timeline) ─────────
    Field('stroke_peak_rev', 'plant', 'M', 'f', ''),
    Field('dip_below_x3_rev', 'plant', 'M', 'f', 'the Phase-0 gate row'),
    Field('pullback_rps', 'plant', 'M', 'f', ''),
    Field('trunc', 'plant', 'M', 'b', ''),
    Field('seeds', 'plant', 'M', 'i', ''),
    Field('iq_brake_min_a', 'plant', 'M', 'f', 'the braking-clamp diagnostic'),
    Field('dispatch_shift_ms', 'plant', 'M', 'f', 'rel_fit - rel_ann'),
    Field('hand_traj_acks', 'plant', 'M', 'i', 'per-cycle delta'),
    Field('can_errors', 'plant', 'M', 'i', 'per-cycle delta'),
    Field('bridge_tx_diag', 'plant', 'M', 's', 'per-cycle delta, verbatim'),
    Field('plant_block_source', 'plant', 'M', 's', 'trace | bag'),

    # ── Label / quality (derived at join) ─────────────────────────────────────
    Field('label', 'quality', 'X', 's', ' | '.join(LABELS)),
    Field('label_source', 'quality', 'X', 's', ''),
    Field('label_confidence', 'quality', 'X', 'f', ''),
    Field('label_reason', 'quality', 'X', 's', 'which gate decided'),
    Field('rimshot', 'quality', 'X', 'b', 'PROVISIONAL, REPORT-only'),
    Field('disagreement', 'quality', 'X', 'l',
          'D-vs-M conflicts, NEVER silently resolved'),
    Field('record_provenance', 'quality', 'X', 's',
          ' | '.join((PROV_BOTH, PROV_MINED, PROV_DECLARED))),
    Field('join_residual_ms', 'quality', 'X', 'f', ''),
    Field('usable_for_aim_fit', 'quality', 'X', 'b', ''),
    Field('usable_for_timing_fit', 'quality', 'X', 'b', ''),
    Field('usable_for_speed_fit', 'quality', 'X', 'b', ''),
    Field('usable_for_release_fit', 'quality', 'X', 'b',
          'the ARC fits are trustworthy: both branches long enough and the '
          'fitted release velocity SE small. NOT implied by _speed_fit, which '
          'gates on the landing fit'),
    Field('usable_for_lateral_fit', 'quality', 'X', 'b',
          'the WHOLE-ARC lateral estimate is admissible (E-1, 2026-08-13): '
          'coverage_asym_s present and within COVERAGE_ASYM_MAX_S. A separate '
          'flag for the same reason _release_fit is one — _aim_fit gates the '
          'aim MAP, which consumes only the landing POSITION and never carried '
          'the branch artefact. ABSENT coverage_asym_s refuses: that is a '
          'pre-E-1 mine, whose lateral velocities ARE the artefact'),
    Field('excluded_reason', 'quality', 'X', 's',
          'every refusal this row accumulated, comma-joined and sorted — across '
          'ALL the usable_* flags, not one of them. A null coverage_asym_s is '
          'deliberately NOT a reason: there is no lateral estimate to exclude'),
)

FIELD_NAMES: Tuple[str, ...] = tuple(f.name for f in FIELDS)
_BY_NAME: Dict[str, Field] = {f.name: f for f in FIELDS}

_FIXED_LEN = {'f2': 2, 'f3': 3}


# ── Blank / encode / decode / validate ────────────────────────────────────────

def blank_record() -> Dict[str, Any]:
    """A record with every field present and null.

    Present-and-null, never absent: a reader that has to distinguish "the field
    does not exist in this schema" from "nothing was measured" is a reader that
    will eventually guess.
    """
    rec = {name: None for name in FIELD_NAMES}
    rec['schema'] = SCHEMA
    return rec


def _clean(value: Any) -> Any:
    """JSON-safe scalar: non-finite floats become ``None``.

    JSON has no NaN. Python's ``json`` will happily emit a bare ``NaN`` token,
    which is not portable JSON and which every strict parser downstream rejects —
    so a NaN measurement is encoded as the null it actually is.
    """
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (list, tuple)):
        return [_clean(v) for v in value]
    if isinstance(value, dict):
        return {str(k): _clean(v) for k, v in value.items()}
    return value


def encode(record: Dict[str, Any]) -> str:
    """Record -> one compact JSON line, in :data:`FIELDS` order.

    Unknown keys are DROPPED rather than passed through: the declaration and the
    mined half must round-trip through the same field list, and a typo'd key that
    survives encoding is a field nobody will ever look for.
    """
    out = {}
    for name in FIELD_NAMES:
        out[name] = _clean(record.get(name))
    return json.dumps(out, separators=(',', ':'), sort_keys=False,
                      allow_nan=False)


def decode(line: str) -> Dict[str, Any]:
    """One JSON line -> a full record (missing fields filled with null)."""
    raw = json.loads(line)
    if not isinstance(raw, dict):
        raise ValueError('toss record must be a JSON object')
    rec = blank_record()
    for name in FIELD_NAMES:
        if name in raw:
            rec[name] = raw[name]
    rec['schema'] = raw.get('schema') or None
    return rec


def validate(record: Dict[str, Any]) -> Tuple[str, ...]:
    """-> a tuple of problem strings; empty means the record is well-formed.

    Validates the **wire form** — what :func:`decode` produces — so a producer
    checks itself with ``validate(decode(encode(rec)))``. That is the shape
    consumers actually see, and it is why a non-finite float is a problem here
    rather than a tolerated alias for null: :func:`encode` has already turned
    every legitimate NaN into ``None``, so a NaN reaching this function means
    something bypassed the encoder.

    Deliberately NOT an exception: a miner that dies on one malformed row loses
    the whole capture, and the record is an instrument, not a control input.
    Callers report and continue.
    """
    problems: List[str] = []
    unknown = sorted(set(record) - set(FIELD_NAMES))
    if unknown:
        problems.append('unknown fields: {}'.format(', '.join(unknown)))
    for f in FIELDS:
        value = record.get(f.name)
        if value is None:
            if f.required:
                problems.append('{}: required, got null'.format(f.name))
            continue
        problems.extend(_kind_problems(f, value))
    schema = record.get('schema')
    if schema is not None and schema != SCHEMA:
        problems.append('schema: got {!r}, this build writes {!r}'.format(
            schema, SCHEMA))
    label = record.get('label')
    if label is not None and label not in LABELS:
        problems.append('label: {!r} not in {}'.format(label, LABELS))
    return tuple(problems)


def _kind_problems(f: Field, value: Any) -> List[str]:
    kind = f.kind
    if kind == 's':
        return [] if isinstance(value, str) else ['{}: want str'.format(f.name)]
    if kind == 'b':
        return [] if isinstance(value, bool) else ['{}: want bool'.format(f.name)]
    if kind == 'i':
        ok = isinstance(value, int) and not isinstance(value, bool)
        return [] if ok else ['{}: want int'.format(f.name)]
    if kind == 'f':
        ok = (isinstance(value, (int, float)) and not isinstance(value, bool)
              and math.isfinite(float(value)))
        return [] if ok else ['{}: want a finite number'.format(f.name)]
    if kind in _FIXED_LEN:
        want = _FIXED_LEN[kind]
        if not isinstance(value, (list, tuple)) or len(value) != want:
            return ['{}: want {} numbers'.format(f.name, want)]
        for v in value:
            if v is None:
                continue
            if not isinstance(v, (int, float)) or isinstance(v, bool):
                return ['{}: want numbers'.format(f.name)]
        return []
    if kind == 'l':
        return [] if isinstance(value, list) else ['{}: want list'.format(f.name)]
    if kind == 'o':
        return [] if isinstance(value, dict) else ['{}: want object'.format(f.name)]
    return ['{}: unknown kind {!r}'.format(f.name, kind)]


# ── The announced-ball latch (extracted from reload_coordinator_node) ─────────

def latch_announced_ball(balls: Iterable[Any], *, robot_name: str,
                         announced_id: Optional[int],
                         preexisting_ids: Iterable[int],
                         untagged_latch: bool,
                         in_flight_status: int) -> Tuple[Optional[int], bool]:
    """Correlate the tracker-assigned id of OUR announced ball. Pure.

    -> ``(announced_id, untagged_latch)``. The caller owns the state; this
    function owns the RULE, so the node and the offline miner apply the same one.
    Extracted verbatim from ``reload_coordinator_node._update_announced_ball_latch``
    (2026-08-10, plan D11) — the node now calls this and keeps only the locking.

    The tracker puts our ball IN_FLIGHT (``destination == robot_name``) after
    correlating the announcement; latch that id, then confirm ONLY that id's
    CAUGHT. This rejects a stray caught ball — a different id, e.g. a leftover
    from a prior throw — that would otherwise falsely confirm the catch.

    Two hardening passes carried over from the 2026-07-23 re-test:

    1. ids already IN_FLIGHT when the throw was accepted are excluded (a phantom
       untagged track that predated the throw was latched as "our" ball);
    2. a destination match is preferred over the empty-destination fallback, and
       an UNTAGGED latch stays PROVISIONAL — the moment a destination-tagged
       candidate appears the latch moves to it, because the tagged track is the
       tracker's own claim about OUR ball.
    """
    preexisting = set(int(i) for i in preexisting_ids)
    candidates = [b for b in balls
                  if int(b.status) == int(in_flight_status)
                  and int(b.id) not in preexisting]
    dest_match = next((int(b.id) for b in candidates
                       if b.destination == robot_name), None)
    if announced_id is None:
        if dest_match is not None:
            return dest_match, False
        for b in candidates:
            if not b.destination:
                return int(b.id), True
        return None, untagged_latch
    if untagged_latch and dest_match is not None:
        return dest_match, False
    return announced_id, untagged_latch


# ── Sensor edges and the label ────────────────────────────────────────────────

class SensorSample(NamedTuple):
    """One ``/hand_telemetry`` sample, in the caller's chosen ``ros``/``bag``
    clock. ``held`` is the DEBOUNCED verdict, ``raw`` the undebounced bit."""
    t: float
    held: bool
    raw: bool
    valid: bool
    stamp: float = 0.0     # ball_held_stamp, for the MEASURED poll cadence


def edges(samples: Sequence[SensorSample], *, debounced: bool = True
          ) -> Tuple[Tuple[float, ...], Tuple[float, ...]]:
    """-> ``(rises, falls)`` transition instants over VALID samples only.

    Invalid samples are SKIPPED, not treated as a level: ``ball_held_valid ==
    false`` is UNKNOWN, and letting UNKNOWN act as EMPTY would mint a departure
    edge out of a telemetry hiccup (C-POSSESS-1 § 2, and D13 here).

    The same rule ``ball_possession.HandBallSensorSource`` applies, so this
    ledger and a live verdict are two views of one stream rather than two
    conventions.
    """
    rises: List[float] = []
    falls: List[float] = []
    prev: Optional[bool] = None
    for s in samples:
        if not s.valid:
            continue
        cur = bool(s.held if debounced else s.raw)
        if prev is None:
            prev = cur
            continue
        if cur != prev:
            prev = cur
            (rises if cur else falls).append(float(s.t))
    return tuple(rises), tuple(falls)


def poll_dt_ms_median(samples: Sequence[SensorSample]) -> Optional[float]:
    """MEASURED sensor poll cadence: the median advance of ``ball_held_stamp``
    over valid samples, in ms. ``None`` when no stamp ever advances.

    Not the ``/hand_telemetry`` publish rate — that is 100 Hz and says nothing
    about how often the bridge actually got an SDO reply. On
    2026-08-10_16-30-44 the two differ by 7x (10 ms publish, 71 ms median poll,
    against a configured ``JB_BD_CHECK_INTERVAL_MS`` of 20).
    """
    steps: List[float] = []
    prev: Optional[float] = None
    for s in samples:
        if not s.valid:
            continue
        stamp = float(s.stamp)
        if prev is not None and stamp != prev:
            steps.append(stamp - prev)
        prev = stamp
    if not steps:
        return None
    steps.sort()
    return steps[len(steps) // 2] * 1e3


def _first_in(instants: Sequence[float], lo: float, hi: float) -> Optional[float]:
    for t in instants:
        if lo <= t <= hi:
            return float(t)
    return None


def _finite_or_none(value: Any) -> Optional[float]:
    """``float(value)`` when it is a real number, else None.

    One helper so None and NaN are treated identically everywhere a SCHEDULE
    instant is optional. A NaN that reached a horizon would compare False against
    everything and silently disable the clamp it was meant to apply — the sibling
    of ``ball_possession.HandBallSensorSource._finite``, and for the same reason."""
    if value is None:
        return None
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


class SensorBlock(NamedTuple):
    """The mined sensor block plus the label it implies."""
    fields: Dict[str, Any]
    label: str
    reason: str
    confidence: float


def label_from_sensor(samples: Sequence[SensorSample], *,
                      throw_time: float, landing_time: float,
                      windows: SensorWindows,
                      next_release_time: Optional[float] = None,
                      next_landing_time: Optional[float] = None,
                      prev_landing_time: Optional[float] = None,
                      stamp_wall_anchored: Optional[bool] = None) -> SensorBlock:
    """THE definition of "caught" for the corpus. Pure; one implementation.

    ``samples`` must cover at least the decisive window; extra samples either
    side are ignored. ``throw_time`` / ``landing_time`` are the announcement's
    own stamps in the SAME clock as ``samples[i].t``.

    ``next_release_time`` / ``next_landing_time`` are the NEXT cycle's announced
    instants in that same clock, or ``None`` when this is the last toss of a
    chain. They are the cadence clamp of C-POSSESS-1 § 3.4 and they are what
    keeps this label honest as the dwell shrinks — see "THE CADENCE CLAMP" below.
    The live verdict takes the identical pair through
    ``ball_possession.HandBallSensorSource.observe``; the two implementations of
    "caught" must agree, so they clamp the same way from the same constants.

    **The decisive window** is ``[throw - departure_lead, landing +
    arrival_window + retention_window]``, both ends subject to the clamp —
    release evidence at one end, the latest instant at which CAUGHT can still
    turn into BOUNCED at the other. ``sensor_valid_frac`` is computed over
    exactly that span, because a sensor that was healthy an hour earlier says
    nothing about this toss.

    **The label, in gate order** (plan § 3.3):

    1. ``sensor_valid_frac < 1.0`` in the decisive window ⇒ **UNKNOWN**, and
       UNKNOWN never collapses to a verdict (D13, C-POSSESS-1 § 2). Treating
       "no valid sample" as "no ball" mints a false MISSED on every telemetry
       hiccup — and those false MISSEDs are exactly the records the aim fit wants
       most, because a MISS with a clean mocap fit is the most informative aim
       datum there is.
    2. no departure edge ⇒ **NO_RELEASE**;
    3. departure, no catch edge ⇒ **MISSED**;
    4. departure, catch edge, dropout inside the retention horizon ⇒ **BOUNCED**;
    5. otherwise ⇒ **CAUGHT**.

    Edge TIMES come from the RAW bit and the VERDICT from the DEBOUNCED one
    (D12) — see the module docstring for the measured ~241 ms asymmetric fall lag
    that makes the distinction worth a field rather than a footnote.

    **THE CADENCE CLAMP — why gate 4 is not a fixed 1.5 s (census D1).**
    ``retention_window_s`` was sized 1.5x the longest seat-then-leave the sensor
    has resolved (0.999 s) and justified at the top by a delay floor of 3.5 s
    that no longer exists. At the tuning-phase dwell of **0.49 s** a legitimate
    throw departs the cup 0.49 s after seating — comfortably inside a fixed
    1.5 s window — so gate 4 would fire on EVERY successful cycle and the corpus
    would read ``BOUNCED`` for a perfect session. Worse, that same route is what
    ``on_empty_cup: RELOAD`` uses to decide a ball is on the floor.

    So the horizon closes at ``next_release_time - departure_lead_s`` — the very
    instant the NEXT toss's departure search opens. A fall at or after it is that
    throw's departure by definition and cannot also be this toss's bounce-out.
    Likewise the arrival search is bounded by ``ball_possession.arrival_boundary_t``
    at BOTH ends — ``arr_hi`` where the next row's search opens, ``arr_lo`` where
    the previous row's closed — so no edge is claimed twice (census D2). When the
    horizon leaves no interval at all the bounce test is SKIPPED rather than
    resolved either way: there is nothing to observe, and inventing a BOUNCED
    from arithmetic is the failure this clamp exists to prevent.

    **C-POSSESS-1.C.1 / C.2, added 2026-08-23.** ``arr_hi`` used to be
    ``next_landing_time - arrival_lead_s`` outright, which pays the NEXT row's
    pre-landing guard out of THIS row's measured arrival band and, once the cycle
    period drops under ``ARRIVAL_BAND_MAX_S + arrival_lead_s`` = 1.000 s, mints a
    **false MISSED** on a catch whose seat edge merely landed in the band's tail.
    ``arrival_boundary_t`` surrenders the guard rather than the band; and where
    the schedule truncates the band anyway (a period under 0.800 s — the deferred
    R6 fork), gate 3 answers **UNKNOWN, not MISSED**, because a window that
    stopped short of the evidence is not a positive observation of non-arrival.
    ``prev_landing_time`` is the other end of the same boundary: pass the
    previous row's ``landing_time`` and adjacent rows abut exactly.
    """
    dep_lo = throw_time - windows.departure_lead_s
    # Clamped so the departure search can NEVER reach into the arrival search and
    # mistake a bounce-out for a departure. On the shipped 0.8 s flight that
    # leaves +0.60 s of departure window against a measured worst case of
    # +0.212 s (2.8x).
    dep_hi = min(throw_time + windows.departure_window_s,
                 landing_time - windows.arrival_lead_s)
    arr_lo = landing_time - windows.arrival_lead_s
    arr_hi = landing_time + windows.arrival_window_s
    next_land = _finite_or_none(next_landing_time)
    if next_land is not None:
        # Census D2 / clause C.1 — close at the boundary this pair of landings
        # shares, which is where the NEXT row's arrival search opens.
        arr_hi = max(arr_lo, min(arr_hi, arrival_boundary_t(
            landing_time, next_land, windows.arrival_lead_s)))
    prev_land = _finite_or_none(prev_landing_time)
    if prev_land is not None:
        # The identical call the PREVIOUS row made to close its own search. Only
        # ever moves this opening LATER, so its absence is the shipped window.
        arr_lo = max(arr_lo, arrival_boundary_t(
            prev_land, landing_time, windows.arrival_lead_s))
        # Degenerate to a single INSTANT rather than inverting, and degenerate to
        # the same instant `HandBallSensorSource._window` does — the low end wins
        # in both. An inverted search would make every `lo <= t <= hi` vacuously
        # false and mint a MISSED out of arithmetic. Only reachable on a schedule
        # whose next landing precedes this one, i.e. never from a real
        # announcement stream; it is written down because the two implementations
        # are required to clamp the SAME way, including where they fail.
        arr_hi = max(arr_lo, arr_hi)
    # Clause C.2: did the search outlast the ball's measured band? A window the
    # SCHEDULE truncated inside the band has not seen the evidence out, so "no
    # rise" is not a positive observation of non-arrival — gate 3 below.
    band_watched_out = arr_hi >= landing_time + ARRIVAL_BAND_MAX_S
    next_rel = _finite_or_none(next_release_time)
    win_lo = dep_lo
    win_hi = arr_hi + windows.retention_window_s
    if next_rel is not None:
        # Census D1 — and never SHORTER than the arrival search itself, or the
        # decisive-sample filter would starve the very gates it feeds.
        win_hi = max(arr_hi, min(win_hi, next_rel - windows.departure_lead_s))

    decisive = [s for s in samples if win_lo <= s.t <= win_hi]
    n = len(decisive)
    n_valid = sum(1 for s in decisive if s.valid)
    valid_frac = (float(n_valid) / n) if n else 0.0

    rises_deb, falls_deb = edges(decisive, debounced=True)
    rises_raw, falls_raw = edges(decisive, debounced=False)

    t_dep_raw = _first_in(falls_raw, dep_lo, dep_hi)
    t_dep_deb = _first_in(falls_deb, dep_lo, dep_hi)
    t_catch_raw = _first_in(rises_raw, arr_lo, arr_hi)
    t_catch_deb = _first_in(rises_deb, arr_lo, arr_hi)

    # The retention HORIZON (C-POSSESS-1 § 3.4), mirroring
    # HandBallSensorSource._retention_horizon exactly.
    ret_hi = None
    if t_catch_deb is not None:
        ret_hi = t_catch_deb + windows.retention_window_s
        if next_rel is not None:
            ret_hi = min(ret_hi, next_rel - windows.departure_lead_s)

    t_dropout = None
    if t_catch_deb is not None and ret_hi > t_catch_deb:
        t_dropout = _first_in(falls_deb, t_catch_deb, ret_hi)

    held_at_dispatch = None
    for s in decisive:
        if s.valid and s.t <= throw_time:
            held_at_dispatch = bool(s.held)
    held_at_retention = None
    if t_catch_deb is not None and ret_hi > t_catch_deb:
        for s in decisive:
            if s.valid and s.t <= ret_hi:
                held_at_retention = bool(s.held)

    edge_count = len(rises_deb) + len(falls_deb)

    fields: Dict[str, Any] = {
        'sensor_valid_frac': valid_frac,
        'sensor_n_samples': n,
        'sensor_held_at_dispatch': held_at_dispatch,
        't_departure_raw_ros': t_dep_raw,
        't_departure_deb_ros': t_dep_deb,
        't_catch_raw_ros': t_catch_raw,
        't_catch_deb_ros': t_catch_deb,
        't_dropout_ros': t_dropout,
        'held_at_catch_plus_retention': held_at_retention,
        'sensor_edge_count': edge_count,
        'sensor_poll_dt_ms_median': poll_dt_ms_median(decisive),
        'ball_held_stamp_wall_anchored': stamp_wall_anchored,
    }

    # Gate 1 — UNKNOWN, and it never collapses.
    if n == 0 or valid_frac < 1.0:
        return SensorBlock(fields, LABEL_UNKNOWN,
                           'sensor_valid_frac={:.3f} over {} samples'.format(
                               valid_frac, n),
                           0.0)
    # Gate 2 — the ball never left the cup.
    if t_dep_deb is None and t_dep_raw is None:
        return SensorBlock(fields, LABEL_NO_RELEASE,
                           'no held->empty edge in [{:+.3f}, {:+.3f}] s of the '
                           'announced throw'.format(dep_lo - throw_time,
                                                    dep_hi - throw_time),
                           1.0)
    # Gate 3 — it left and never came back.
    if t_catch_deb is None:
        dep_at = (t_dep_raw if t_dep_raw is not None else t_dep_deb) - throw_time
        if not band_watched_out:
            # C-POSSESS-1.C.2. The search closed inside the measured band — the
            # next landing truncated it, or `arrival_window_s` is configured
            # under the band — so the corpus does not know whether the ball
            # missed or whether we looked away. MISSED is a POSITIVE claim and is
            # not available; UNKNOWN never collapses to one (gate 1's rule, same
            # reason).
            return SensorBlock(
                fields, LABEL_UNKNOWN,
                'departure at {:+.3f} s, no empty->held edge in [{:+.3f}, '
                '{:+.3f}] s of landing — but the search was band clamped, '
                'closing at {:+.3f} s short of the measured band ceiling '
                '{:+.3f} s, so non-arrival was never observed'.format(
                    dep_at, arr_lo - landing_time, arr_hi - landing_time,
                    (landing_time + ARRIVAL_BAND_MAX_S) - arr_hi,
                    ARRIVAL_BAND_MAX_S),
                0.0)
        return SensorBlock(fields, LABEL_MISSED,
                           'departure at {:+.3f} s, no empty->held edge in '
                           '[{:+.3f}, {:+.3f}] s of landing'.format(
                               dep_at,
                               arr_lo - landing_time, arr_hi - landing_time),
                           1.0)
    # Gate 4 — it arrived and then left again inside the retention HORIZON. A
    # departure at or after `next_release - departure_lead` is the next toss's
    # release and was excluded above, by construction rather than by tolerance.
    if t_dropout is not None:
        return SensorBlock(fields, LABEL_BOUNCED,
                           'arrival at {:+.3f} s, dropout {:+.3f} s later '
                           '(retention horizon {:.2f} s)'.format(
                               t_catch_deb - landing_time,
                               t_dropout - t_catch_deb,
                               ret_hi - t_catch_deb),
                           1.0)
    # Gate 5.
    if ret_hi <= t_catch_deb:
        # The cadence leaves NO interval between the seat edge and the next
        # release, so retention was never observable. CAUGHT is still the right
        # label — the ball demonstrably arrived — but the reason must say the
        # bounce test never ran, and the confidence must not claim it did.
        return SensorBlock(fields, LABEL_CAUGHT,
                           'arrival at {:+.3f} s; retention NOT OBSERVABLE (the '
                           'next release at {:+.3f} s leaves no horizon)'.format(
                               t_catch_deb - landing_time,
                               (next_rel or float('nan')) - landing_time),
                           0.5)
    return SensorBlock(fields, LABEL_CAUGHT,
                       'arrival at {:+.3f} s, held through retention '
                       '({:.2f} s horizon)'.format(
                           t_catch_deb - landing_time, ret_hi - t_catch_deb),
                       1.0)


# ── The join ──────────────────────────────────────────────────────────────────

def names_by_origin(origin: str) -> Tuple[str, ...]:
    """Field names with the given origin — ``'D'`` / ``'M'`` / ``'DM'`` / ``'X'``.

    Public because it is how a corpus reader answers *"which half of the record
    did this number come from?"* without a per-field provenance marker in every
    row. The design (§ 3.3) asks the declaration to upgrade fields from mined to
    declared; the row-level ``record_provenance`` plus this origin table says the
    same thing deterministically and at a fraction of the schema cost — on a
    ``mined-only`` row every ``D`` field is null by construction, and on a
    ``declared+mined`` row the declaration won every ``D`` and ``DM`` field with
    any conflict already listed in ``disagreement``.
    """
    return tuple(f.name for f in FIELDS if f.origin == origin)


def _both_origin_names() -> Tuple[str, ...]:
    return tuple(f.name for f in FIELDS if f.origin == 'DM')


def _declared_names() -> Tuple[str, ...]:
    return tuple(f.name for f in FIELDS if f.origin == 'D')


def _mined_names() -> Tuple[str, ...]:
    return tuple(f.name for f in FIELDS if f.origin == 'M')


def _derived_names() -> Tuple[str, ...]:
    return tuple(f.name for f in FIELDS if f.origin == 'X')


def join(declarations: Sequence[Dict[str, Any]],
         mined: Sequence[Dict[str, Any]],
         *, tol_s: float = JOIN_TOL_S) -> List[Dict[str, Any]]:
    """Join declarations to mined rows on ``announce_throw_time_ros``.

    -> one merged record per mined row, plus one per unmatched declaration.
    Nothing is dropped: a silently missing row is how a replay overstates its own
    agreement, and an unmatched declaration is itself a finding (the bag lost the
    announcement, or the toss never announced).

    Merge rule, by field origin:

    ``D``   the declaration wins outright — the node is the only witness of what
            it commanded;
    ``M``   the mined value wins outright;
    ``DM``  the declaration wins, and a difference is appended to
            ``disagreement`` rather than averaged away. A ``DM`` field exists
            precisely so the two halves can be cross-checked; resolving a
            conflict silently throws away the only signal the redundancy buys.
    """
    out: List[Dict[str, Any]] = []
    remaining = list(range(len(declarations)))
    d_names = _declared_names()
    m_names = _mined_names()
    dm_names = _both_origin_names()
    x_names = _derived_names()

    for m in mined:
        key = m.get('announce_throw_time_ros')
        best = None
        best_dt = None
        if key is not None:
            for idx in remaining:
                k = declarations[idx].get('announce_throw_time_ros')
                if k is None:
                    continue
                dt = abs(float(k) - float(key))
                if dt <= tol_s and (best_dt is None or dt < best_dt):
                    best, best_dt = idx, dt
        rec = blank_record()
        for name in m_names:
            rec[name] = m.get(name)
        # Every X (derived) field the miner already computed carries over.
        # Iterating the ORIGIN rather than a hand-written name list is what stops
        # a field added to FIELDS from being silently dropped at the join — the
        # three the join owns itself are overwritten below.
        for name in x_names:
            if m.get(name) is not None:
                rec[name] = m.get(name)
        disagreement: List[str] = []
        if best is None:
            for name in dm_names:
                rec[name] = m.get(name)
            rec['record_provenance'] = PROV_MINED
            rec['join_residual_ms'] = None
        else:
            d = declarations[best]
            remaining.remove(best)
            for name in d_names:
                rec[name] = d.get(name)
            for name in dm_names:
                dv, mv = d.get(name), m.get(name)
                rec[name] = dv if dv is not None else mv
                # The JOIN KEY is excluded from the disagreement scan: the two
                # halves are matched ON it, within tol_s, so any difference is
                # by definition the join residual — which has its own field.
                # Reporting it here would duplicate join_residual_ms on every
                # single row and bury the disagreements that mean something.
                if name == 'announce_throw_time_ros':
                    continue
                if dv is not None and mv is not None and not _agree(dv, mv):
                    disagreement.append('{}: declared={!r} mined={!r}'.format(
                        name, dv, mv))
            rec['record_provenance'] = PROV_BOTH
            rec['join_residual_ms'] = float(best_dt) * 1e3
        rec['schema'] = SCHEMA
        rec['disagreement'] = disagreement
        if rec.get('toss_uid') is None:
            rec['toss_uid'] = m.get('toss_uid') or _mined_uid(m)
        if rec.get('action') is None:
            rec['action'] = m.get('action') or 'unknown'
        if rec.get('outcome') is None:
            rec['outcome'] = m.get('outcome') or 'UNDECLARED'
        out.append(rec)

    for idx in remaining:
        d = declarations[idx]
        rec = blank_record()
        for name in d_names + dm_names:
            rec[name] = d.get(name)
        rec['schema'] = SCHEMA
        rec['record_provenance'] = PROV_DECLARED
        rec['disagreement'] = []
        out.append(rec)
    return out


def _agree(a: Any, b: Any) -> bool:
    if isinstance(a, (int, float)) and isinstance(b, (int, float)) \
            and not isinstance(a, bool) and not isinstance(b, bool):
        return abs(float(a) - float(b)) <= 1e-6 * max(1.0, abs(float(a)))
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(_agree(x, y) for x, y in zip(a, b))
    return a == b


def _mined_uid(m: Dict[str, Any]) -> str:
    """A stable id for a row nobody declared: the join key itself.

    Millisecond resolution — the join tolerance is 5 ms, so two rows that would
    collide here would also have joined to the same declaration.
    """
    key = m.get('announce_throw_time_ros')
    if key is None:
        return 'mined-unkeyed'
    return 'mined-{:.3f}'.format(float(key))
