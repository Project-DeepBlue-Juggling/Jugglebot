---
title: Toss self-tuning loop — per-toss record, persistent aim map, session trim
created: 2026-08-10
status: active
related_plan: catch-robustness.md
related_logbook:
  - 2026-08-10-sensor-truth-possession.md
  - 2026-08-10-hand-drive-braking-clamp-diagnosis.md
  - 2026-08-10-tilt-cal-c0-blockers-level-noise-and-leg0-spinout.md
related_config:
  - config/toss_calibration.yaml → aim_rad (NEW, machine-written)
  - config/hardware_config.yaml → toss_require_ball_evidence
  - config/hardware_config.yaml → toss_release_latency_ms
related_code:
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py::_build_toss_cycle
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/toss_release.py::compute_release_state_tilted
  - ros_ws/src/jugglebot/jugglebot/ball_possession.py::merge_possession
  - tools/probes/ball_arrival_offset.py
---

# Plan — Toss self-tuning loop

**Parent plan:** [catch-robustness.md](catch-robustness.md) Phase 2.
**Branch:** `mvp-trajectory-bringup`.
**Design authored** 2026-08-10 by a design panel, verified against the working
tree; **operator-reviewed and APPROVED 2026-08-10** (§ 9).

Every file, symbol, constant and reference below was checked against the working
tree. Numbers *derived here* rather than read from the repo are marked
**[derived]**. Numbers that are placeholders awaiting a first corpus are marked
**PROVISIONAL**, in the convention `tests/hardware/tilt_cal_grid.py` already uses
for its timing defaults. Code is cited as `module::symbol` rather than
`file:line` — Phase 1 landed between the design draft and this plan and moved
several hundred lines in `reload_coordinator_node.py`, which is exactly how line
citations rot.

---

## 0. Session flag — read before anything else

`temp/reports/nightly/status` still reads

```
RED 4638/4661 passed, 20 failed, 0 errored, 3 xfailed, 0 skipped 2026-08-10T04:01:41+10:00
```

**That RED is stale and already RESOLVED.** All 20 failures were in
`tests/motion/test_tilt_cal_grid.py` — the 04:00 nightly sampled the tilt-cal
session's half-edited working tree. The test rework landed the same day
(`7cbfd9d` / `5e046cc`) and, as reported by that session, `./run_tests.sh --full`
passed post-landing on **2026-08-10** with **4666 + 9 passed, 3 xfailed**. The
next 04:00 run is the standing confirmation, and until it lands the `status`
file will keep reporting the stale RED.

The P0 gate stands regardless: a **fresh** `./run_tests.sh --full` GREEN before
any sitting. The "Phase 2 prerequisite" the design carried — fix the 20 stale
tilt-cal failures — is therefore **satisfied**, not outstanding.

---

## 1. Goal and premises

**Goal.** Catch success approaching 100 % for single-ball self-tosses across the
±150 mm workspace at z = 170 mm, tier 8a.

**Operator premises (fixed, 2026-08-10).**

| # | Premise |
|---|---|
| P1 | Learning shape **(c)**: a persistent calibration map written **only** by an explicit calibration routine, plus a session-local real-time trim layered on it. Every session starts from the persistent map; the trim never persists. |
| P2 | `TossContinuous` auto-reloads from BallButler on drops. |
| P3 | Ground truth is the hand ball sensor (`HandTelemetryMessage.ball_held` / `_raw` / `_valid`, 100 % valid across 203,922 samples in the three 2026-08-10 bags, ~±20 ms at the 50 Hz poll). Mocap supplies landing offsets and achieved flight. **Sensor-primary possession verdicts LANDED with catch-robustness Phase 1** (`ball_possession::HandBallSensorSource`, `ball_possession::merge_possession`, commits `9caf6bc` / `2716e3b` / `6920e88` / `60f0a86`) — this plan **consumes** that merge and must not re-implement it. |
| P4 | No RL. No catch-knob tuning until the hand-drive braking clamp is bench-restored (catch-robustness Phase 0); catch-knob work is then A/B ladders, never continuous learning. |

**Hard constraints.** `toss_sequencer::MIN_TOSS_THROW_DELAY_S` = 3.5 s untouched
⇒ dwell floor 5.60 s at the 5.0 s default delay ⇒ ~10 cycles/min ceiling. One
ball-op at a time (`reload_coordinator_node._goal_claimed`, taken at goal
ACCEPT). Hand-dispatch ladders and `_MAX_ARM_DISPATCHES` retained. Abort paths
untouched. Trim authority bounded, clamped, and adapted **between tosses only**.
Fresh can-bridge boot before every sitting; `uptime_ms` recorded per toss.
Runtime artifacts under `temp/`, persistent calibration under `config/`
(machine-written, versioned, like `config/tilt_calibration.yaml`), reusable
tools under `tools/`.

---

## 2. Six findings that shape the design

Each was verified in code and each forecloses an otherwise-obvious approach.

### F1 — In tier 8a, offsetting the nominated catch xy corrects nothing. The aim correction must be **angular**.

The 8a platform pre-positions to the nominated catch pose B
(`reload_coordinator_node._toss_positioning_xyz` — 8a returns
`catch_pose_stow_mm` verbatim), throws vertically from there
(`toss_release::compute_release_state(catch_pose, flight)`), and then **holds
that pose open-loop through the whole flight**:
`JB_OP_RELOAD_PLATFORM_OPEN_LOOP = True` (`config/generated/hardware_config.py`),
and `catch_coordinator_node._on_balls` computes

```python
open_loop = ((self._pretilt_hold or hw.JB_OP_RELOAD_PLATFORM_OPEN_LOOP)
             and self._catch_armed and self._announcement_seen)
```

so the tracker-driven reactive reach is suppressed for every armed toss. Two
consequences:

1. A shift Δ in the nominated catch xy moves the **release point and the cup by
   the same Δ**. The residual launch-axis misalignment ψ still lands the ball at
   `cup + 4h·ψ`. The error relative to the cup is unchanged. **A translational
   "nudge the catch point" trim is a structural no-op in 8a.**
2. Nothing reactive absorbs an aim error either. The 80 mm reach envelope is not
   in play during a toss.

The lateral landing offset of a vertical toss tilted θ from vertical is
`b = 4h·θ` **[derived]** — `v = √(2gh)`, `T = √(8h/g)`, `b = v·θ·T = 4h·θ`. This
reproduces both repo anchors exactly: 41.9 mm/° at a 0.6 m toss
([tilt-calibration-grid.md](tilt-calibration-grid.md)) and 43 mm at 0.78° for the
0.78 m default (`Toss.action` `REJECTED_NOT_LEVELLED` comment: "v*sin(theta)*T =
43 mm ... at 3.93 m/s").

| apex h (m) | T (s) | v (m/s) | K = 4h (mm/rad) | mm/° |
|---|---|---|---|---|
| 0.45 | 0.606 | 2.97 | 1800 | 31.4 |
| 0.60 | 0.700 | 3.43 | 2400 | 41.9 |
| **0.78** | **0.798** | **3.91** | **3120** | **54.5** |
| 1.00 | 0.903 | 4.43 | 4000 | 69.8 |

### F2 — An aim-corrected 8a toss is structurally an 8b toss with zero displacement, and the disable path is provably today's machine.

`toss_release::compute_release_state_tilted(B, T, throw_site_xy_mm=B_xy)`
reproduces `compute_release_state` **bitwise** — the documented "Tier-8a
regression net" in `motion/trajectory/toss_release.py`, pinned by
`tests/motion/test_toss_release.py`. So the aim correction rides the existing,
tested tilt path, and **zero bias ⇒ zero tilt ⇒ bit-identical to the shipped 8a
toss**.

What it costs: three branches in `reload_coordinator_node` currently key on
`tier == TIER_8B` and must be re-keyed on *"the release state carries a non-zero
tilt"* — `_position_platform_for_toss` (sends
`_tilt_quaternion(release.tilt_rx, release.tilt_ry)` instead of identity), the
`catch/pretilt_hold` raise at PREPARE, and `_toss_positioning_xyz` (returns the
swing-compensated pre-tilt pose).

### F3 — Without `pretilt_hold`, the stock announcement pre-tilt silently reverts the aim before release.

`catch_coordinator_node._on_throw_announcement` builds a predicted catch target
from the announced landing state and publishes it as a `catch/dynamic_target`.
Its own docstring states the failure exactly: *"the stock pre-tilt's arrival
clamps to ~now+1 s while the toss announces ≥1 s before release, so an
un-suppressed pre-tilt COMPLETES the A→B translate (and the un-tilt to the
receive tilt) BEFORE the ball is released — aim destroyed."* Today 8a never
raises `pretilt_hold`, and it gets away with it only because the 8a pre-tilt
target equals the pose already held. **The moment a toss commands a non-zero aim
tilt, `pretilt_hold` becomes mandatory** — otherwise every log line reports the
trim as applied while the platform is levelled back before release.

### F4 — The label cannot tune anything; every estimator must run on a continuous observable.

Two-proportion test, α = 0.05 two-sided, power 0.8 **[derived]**: distinguishing
0.80 → 0.90 needs **196 tosses per arm** (~39 min of pure throwing at the 10/min
ceiling); 0.90 → 0.95 needs 432. Detecting a bias β through the *arrival offset*
needs `n ≥ (2σ/β)²` **[derived]** — at σ = 20 mm that is 16 tosses for β = 10 mm,
8 for β = 15 mm. **The label is for scoring, never for adaptation.** This is also
what makes the 3.5 s safety fork cost nothing: a 5-toss-per-node estimate is
~1 minute.

### F5 — `catch_error_mm` is the wrong observable and must never feed an estimator.

`ball_possession.py`'s module docstring documents it as the Kalman filter's
**dead-reckoned free-fall extrapolation** from the last real sighting:
`xy error ≈ |v_xy|·dt → 0.30–3.88 mm for a vertical toss`, on 17/17 CAUGHT
self-tosses. It is a seated-position readout (the marker vanishes into the
funnel), it exists **only for caught balls**, and it is therefore both
selection-biased and measuring the wrong thing. The right instrument already
exists: `tools/probes/ball_arrival_offset.py` — a least-squares fit of
`x(z), y(z)` over the descending samples in `[plane, plane+band]`,
outcome-independent by construction, producing a number for CAUGHT, BOUNCED
**and** MISSED alike.

### F6 — Bias removal alone cannot reach the goal; variance reduction is the dominant lever.

For an isotropic 2D arrival Gaussian with per-axis σ and capture radius
`R = GEOM_HAND_RADIUS_MM = 35.0`, at zero bias
`P(catch) = 1 − exp(−R²/2σ²)` **[derived, Rayleigh]**:

| σ (mm) | 10 | 12 | 15 | 20 | 26 |
|---|---|---|---|---|---|
| P(catch) at β=0 | 0.998 | 0.986 | 0.934 | 0.784 | 0.596 |

Inverting the ~60 % catch rate of the 2026-08-10 8a retest at 1.0 m
([catch-robustness.md](catch-robustness.md) § Context) under β ≈ 0 gives
**σ ≈ 26 mm**. On that plant, removing a 20 mm bias buys ~0.50 → 0.60; halving σ
buys 0.60 → 0.99. **The headline consequence, stated plainly because it sets
expectations for the whole programme: the calibration loop's job is to clear the
deterministic part so that σ is all that remains, and it is worth roughly 5–10
catch-rate points, not 40.** The path to ~100 % runs through catch-robustness
Phase 0 (braking-clamp restore), release repeatability and settle discipline.
The loop is still worth building — it is cheap, it is the instrument that
*measures* σ, and it makes every subsequent A/B interpretable — but it is not the
headline lever.

---

## 3. Architecture

### 3.1 Three layers, and why they cannot double-count

```
Layer 0   config/tilt_calibration.yaml       C-LEVEL-2, INCLINOMETER-measured
          "where is gravity at this pose?"    applied at the six C-LEVEL-1 ingest sites
                    │                          via levelling.correction_for_pose
                    ▼  residual, measured WITH layer 0 applied
Layer 1   config/toss_calibration.yaml       NEW: C-TOSS-CAL-1, BALL-measured
          "where does the ball actually go?"  applied ONLY in _build_toss_cycle
                    │                          as a commanded aim tilt
                    ▼  residual, measured WITH layer 1 applied
Layer 1.5 per-toss dwell inclinometer read   COVARIATE ONLY — zero control authority
          "what was the platform doing        recorded in the toss record, never applied
           at this toss?"
                    │
                    ▼
Layer 2   session trim (RAM only, discarded at goal end)
          "what did today's `level` cost?"    common-mode only, frozen per goal
```

**Physical separation.** The inclinometer senses gravity, so Layer 0 nulls the
*gravity-frame* orientation error and nothing else. It is structurally blind to
hand-axis misalignment ψ, to cup/release asymmetry and to platform residual
motion — all of which move the ball identically. Layer 1 is therefore by
construction the residual left after Layer 0, measured by a different sensor
answering a different question. There is no shared parameter to double-count.
Layer 1.5 is a *recorded covariate* in v1 and applies nothing, so it cannot
double-count by construction (§ 3.10).

**Enforcement (D1–D4).**

- **D1 — one application point.** Layers 1 and 2 **never touch
  `motion/levelling.py`**. They enter as a deliberately non-level *commanded
  orientation* computed in `reload_coordinator_node._build_toss_cycle`, the
  single seam that already computes the release state and is shared verbatim by
  `Toss` and every `TossContinuous` cycle. The C-LEVEL-1 "no second
  implementation" clause survives untouched.
- **D2 — composition is already correct.** The levelling ingest pre-multiplies
  `R_corr` onto whatever rotation the toss commands, so the physical orientation
  is `R_corr(level + tiltmap) · R_aim(M + δ)`. At the 1.0° total-authority clamp
  the second-order cross term is **1.523e-4 rad** — the value C-LEVEL-2's own
  regime table states for the 1°×1° entry
  ([tilt-calibration-grid.md](tilt-calibration-grid.md) § regime table) =
  0.0087° = 0.48 mm of landing offset. Negligible, and inside a bound the
  contract already documents.
- **D3 — provenance-gated dormancy.** `config/toss_calibration.yaml` records
  `requires.tilt_map_version` and `requires.estimator_version`. On mismatch the
  map **loads but stays DORMANT** (`toss_cal_loaded = true`,
  `toss_cal_applied = false`, loud WARN) — the exact pattern
  `TrajectoryStatus.tilt_map_loaded` / `tilt_map_version` already ships for
  C-LEVEL-2. Root cause: an aim residual fitted under tilt map A double-counts
  tilt map B's delta. A **re-`level` does NOT invalidate** the map — it perturbs
  only the common mode, which is Layer 2's job (§ 3.2).
- **D4 — structural test.** The aim residual is looked up **exactly once per
  toss goal, in `_build_toss_cycle`**, never in the 40 Hz emitter, never per
  Hermite knot, never in `catch_coordinator`, and **never on the reload path** (a
  BallButler ball's aim belongs to BallButler). Mirrors the existing
  `test_every_apply_has_a_build_in_the_same_scope` shape.

### 3.2 Why the map is home-referenced and the trim owns the common mode

`level` is one int16-quantised SCL3300 sample with measured session-to-session
scatter **1.2–1.7 mrad/axis** ([tilt-calibration-grid.md](tilt-calibration-grid.md)
§ 2026-08-10 C0 entry). Every re-`level` therefore injects a fresh **common-mode**
aim error of 0.069–0.097° = 3.8–5.3 mm at h = 0.78 — identical at every node,
unknowable until a ball flies. Persisting an absolute aim map would bake one
session's `level` noise into every future session.

So, transposing the anchor-mean pattern the tilt map validated on 2026-08-10
(commit `7cbfd9d`):

- The persistent map stores the **home-referenced spatial field**
  `M(P) = ŷ(P) − mean_over_anchors(ŷ(home))`, with the centre node re-visited
  every `--home-revisit-every` nodes and the reference taken as the anchor
  **mean**. That is the stable mechanical part.
- The map also stores `anchor_aim_rad`, the absolute residual at the anchor —
  **not** as a hard correction but as the **warm-start prior** for the session
  trim, with prior strength `n₀`. That is what "every session starts from the
  persistent map" means operationally, without pretending a level-noise-
  contaminated absolute is a constant of the machine.
- The session trim estimates the **common mode** only (2 parameters), pooled
  across every node the session visits. That is the well-conditioned estimate
  (n_total, not n_per_node), and it is exactly the quantity `level` noise
  perturbs.

### 3.3 The per-toss record — `toss_record/1`

**Shape: one JSON object per toss, minted in two halves joined offline.** A
DECLARATION published by the coordinator on `/toss/record` (so it lands in the
bag), and a MEASUREMENT block mined from that same bag by
`tools/probes/toss_record_miner.py`. **The miner must produce the whole record
alone, in degraded form; the declaration only upgrades specific fields from
`inferred` to `declared`.**

That inversion is load-bearing: it makes replayability structural rather than a
feature, and it is what lets the three 2026-08-10 bags be mined into the corpus
today, before any of this is wired.

**Why the coordinator publishes rather than writes.** `std_msgs/String` carrying
JSON needs **no `jugglebot_interfaces` rebuild**, so a schema change cannot
trigger the partial-colcon `ImportError` class the runbook's two-package build
gate exists to prevent. Publishing rather than writing keeps **zero file I/O**
out of the node that owns the hand, the latch and the abort ladder — a full disk
cannot stall a teardown. A best-effort belt
(`temp/logs/toss_records_<launch>.jsonl`, same bytes from the same encoder,
written outside `self._lock`, every exception swallowed, one WARN per goal then
silence) exists only for the `record:=false` bench session.

**Clock domains.** Five are in play and every field names exactly one: `perf`
(`time.perf_counter()`, machine-global, not in a bag), `ros` (wall, in bags as
`header.stamp`), `bag` (mcap `log_time_ns` — the *only* time available for
`/mocap_data`, which has no header), `bridge` (`uptime_ms`; and
`ball_held_stamp`, which is wall-epoch **only while `ball_held_valid`** and after
the wall anchor lands), `qtm` (related to `ros` by `/qtm_clock_offset_sec`). The
join field that lifts perf into wall is `perf_minus_ros_s`, taken from the
**filtered** `clock_offset::measure_offset` (10 samples, 20-deep history, 30 s
refresh), with the single-read variant carried alongside so the standing
reconciliation gap in `_announcement_landing_perf` becomes a measurement rather
than an argument.

**Schema.** `D` = declared by the node, `M` = mined offline, `X` = derived at
join. `D+M` fields are independently recoverable, which is what makes the join
self-checking.

| Block | Fields |
|---|---|
| **Identity** (D) | `schema`, `toss_uid` = `<session>-<goal_id8>-<cycle>`, `session_id`, `goal_id`, `action` (`toss`\|`toss_continuous`), `cycle_index` (D+M via `_action/feedback`), **`announce_throw_time_ros` — THE JOIN KEY** (D+M), `announce_landing_time_ros`, `t_record_ros`, `perf_minus_ros_s`, `perf_minus_ros_inst_s` |
| **Provenance** (D unless noted) | `git_sha`, `git_dirty`, `toss_cal_version`, `toss_cal_loaded`, `toss_cal_applied`, `tilt_map_version` (D+M), `tilt_map_applied` (D+M), `gravity_correction_loaded` (D+M), `level_offset_rad[2]` (D+M), `toss_tier`, `bridge_fw_version`, `platform_fw_version`, **`uptime_ms_at_release`** (D+M), `hand_odrive_config_sha` (nullable until the Phase-0 config-drift fork is chosen), `catch_knobs{catch_vel_scale, catch_vel_ratio, catch_vel_hold_pct, catch_reach_freeze_s, catch_settle_hold_s, catch_reach_envelope_mm, hand pos/vel/vel_int gains}` |
| **Goal as requested** (D) | `goal_catch_xyz_stow_mm[3]`, `goal_throw_height_m`, `goal_throw_delay_s`, `goal_catch_vel_scale`, and for sessions `goal_num_throws`, `goal_dwell_time_s`, `goal_stop_on_miss`, `goal_on_empty_cup`, `goal_max_reloads`. **Raw 0-sentinels kept alongside the resolved values** so a later reader can tell "the operator asked for 0.8 s" from "the default happened to be 0.8 s". |
| **Resolved release** (D) | `flight_time_s`, `apex_height_m`, `event_vel_mps`, `event_delay_s` (as sent, already latency-shifted), `release_latency_ms_applied`, `release_pos_global_mm[3]`, `launch_vel_mms[3]`, **`catch_point_global_mm[3]`** (via the single conversion point `toss_release::stow_to_global_mm`), `aim_tilt_rx_rad`, `aim_tilt_ry_rad` |
| **Applied calibration** (D) — *the most important block* | `map_aim_rad[2]`, `trim_aim_rad[2]`, `total_aim_rad[2]`, `map_aim_mm_at_h[2]` and `trim_aim_mm_at_h[2]` (derived report fields, mm at *this toss's* apex — never the stored unit), `speed_bias_applied`, `timing_bias_applied_ms`, `clamp_hits[]` (per channel), `trim_source_n`, `trim_state` (`WARMUP`\|`ACTIVE`\|`CONVERGED`\|`FROZEN_<reason>`), `trim_reset_reason` |
| **Dwell tilt (Layer 1.5)** (D) | `dwell_tilt_rad[2]` (mean of the N reads), `dwell_tilt_sd_rad[2]`, `dwell_tilt_n`, `dwell_tilt_span_s`, `dwell_tilt_last_read_to_release_s`, `dwell_tilt_degraded` (read count reduced to protect the throw) — **covariate only, never applied** (§ 3.10) |
| **FSM / dispatch** (D) | `outcome` (verbatim `TossResult.outcome`), `phase_at_terminal`, `throw_dispatch_class` (`ok`\|`ambiguous`\|`rejected`) + message, `prepare_ok`, `position_accepted`/`_planned_s`/`_code`, `catch_target_accepted`, `announce_lead_short`, perf instants `t_accept_perf`, `t_release_perf`, `t_landing_sched_perf`; **session-interlude flags** `reload_settle` (the cycle after a reload interlude, guard G10) and `retry_of` (the `toss_uid` this cycle retried, guard G11); plus **diagnostics-not-truth**: `achieved_flight_s_fsm`, `catch_error_mm_fsm` |
| **Sensor** (M, `/hand_telemetry`) | `sensor_valid_frac`, `sensor_held_at_dispatch`, `t_departure_raw_ros`, `t_departure_deb_ros`, `t_catch_raw_ros` (first `empty→held` in `[landing − 0.30, landing + 0.70]`; 0.70 = `toss_sequencer::CATCH_CONFIRM_WINDOW_S`), `t_catch_deb_ros`, `t_dropout_ros`, `held_at_catch_plus_retention` (retention = the shipped `JB_BD_RETENTION_WINDOW_S` = **1.50 s** — Phase 1 *measured* that window, so it is a pinned constant, not a placeholder; it is also the convention already used to mine bag `2026-08-10_16-30-44`), `sensor_edge_count`, `sensor_poll_dt_ms_median` (**measured**, never assumed), `ball_held_stamp_wall_anchored` |
| **Mocap** (M, `ball_arrival_offset` estimator **imported, not re-implemented**) | `land_xy_global_mm[2]`, **`land_err_mm[2]`** = `land_xy − catch_point_global_mm[:2]`, `land_err_norm_mm`, `n_fit`, `fit_rms_mm`, `fit_sparse`, `apex_z_mm`, `achieved_flight_s_mocap`, `t_land_bag`, `qtm_offset_s`, `mocap_gap_ms_max`, `land_plane_mm`, REPORT-only `floor_arrival` |
| **Plant** (M, `hand_stroke_timeline` row builder **imported**) | `stroke_peak_rev`, **`dip_below_x3_rev`** (the Phase-0 gate row), `pullback_rps`, `trunc`, `seeds`, `iq_brake_min_a` (the braking-clamp diagnostic), **`dispatch_shift_ms`** (`rel_fit − rel_ann`), per-cycle deltas `hand_traj_acks`, `can_errors`, `bridge_tx_diag`, and `plant_block_source: trace\|bag` |
| **Label / quality** (X) | `label` ∈ {CAUGHT, BOUNCED, MISSED, NO_RELEASE, UNKNOWN}, `label_source`, `label_confidence`, `rimshot` (**PROVISIONAL**, REPORT-only), `disagreement[]` (never silently resolved), `record_provenance` (`declared+mined`\|`mined-only`\|`declared-only`), `join_residual_ms`, `usable_for_aim_fit` / `_timing_fit` / `_speed_fit`, `excluded_reason` |

**Label rule, in gate order.** (1) `sensor_valid_frac < 1.0` in the decisive
window ⇒ **UNKNOWN**, all `usable_*` false — UNKNOWN never collapses to a verdict
(C-POSSESS-1 § 2). (2) No departure edge ⇒ NO_RELEASE. (3) Departure, no catch
edge ⇒ MISSED. (4) Departure, catch edge, dropout inside retention ⇒ BOUNCED.
(5) Otherwise CAUGHT.

**Raw is for edge TIMES, debounced is for the VERDICT.** The 5-sample debounce at
50 Hz is a 100 ms window; using the debounced edge as a timing measurand bakes
~100 ms of systematic lag into every timing fit — comparable in size to the
uptime effect the whole fresh-boot discipline exists to control, and it would
look like real physics.

### 3.4 Dataflow

```
LIVE, once per cycle at the FSM terminal
  reload_coordinator_node
    ├─> /toss/record (std_msgs/String JSON)  ──> rosbag              [canonical]
    └─> temp/logs/toss_records_<launch>.jsonl                        [belt, best-effort]

BAG, one per sitting (~/Desktop/rosbags/<stamp>/, mcap)

OFFLINE (project venv, read-only, opens .mcap only)
  tools/probes/toss_record_miner.py --bag <BAG> [--trace <jsonl>] [--self-check]
    ├─ /toss/record       -> declarations   (or INFER from announcement+rosout+feedback)
    ├─ hand-sensor ledger -> sensor edges   [toss_record.label_from_sensor  PRODUCTION]
    ├─ mocap fit          -> landing offset [ball_arrival_offset estimator  IMPORTED]
    ├─ hand stroke        -> plant block    [hand_stroke_timeline builder   IMPORTED]
    ├─ /balls             -> tracker block  [toss_record.latch_announced_ball PRODUCTION]
    └─ JOIN on announce_throw_time_ros (±5 ms)
         -> temp/probes/toss_records_<bag>_<ts>.{jsonl,csv,_meta.json}

CALIBRATION (operator-run, explicit only)
  tests/hardware/toss_cal_grid.py   → capture rungs SC-0..SC-3
  tests/hardware/toss_cal_fit.py    → partition, gate, fit, write config/toss_calibration.yaml,
                                       call toss/reload_calibration, READ BACK the version
  tools/toss_cal_analyse.py         → heatmap + quiver, per-node n/sd, anchor series,
                                       residual-vs-uptime scatter, map-vs-map diff, A/B scoring
                                       → temp/reports/
```

**The join.** Primary key `announce_throw_time_ros`, matched within **±5 ms**.
Over-determined by ~1000×: the 3.5 s delay floor and 5.60 s dwell floor put
consecutive tosses ≥ 5 s apart, and the node writes the *same float* into
`ThrowAnnouncement.throw_time` and into the declaration, so the match is exact by
construction. Filter `thrower_name == 'jugglebot' AND target_id == 'jugglebot'`
separates self-tosses from BB reload throws on the same topic.

**Extraction obligation.** The announced-ball latch and the sensor labeller must
be **production functions imported by both the node and the miner**, not miner
re-implementations — the precedent is explicit in
`tools/probes/possession_verdict_bag_check.py`, which replays `/balls` through
the arrival source "constructed exactly as `reload_coordinator_node` constructs
it", and reinforced by Phase 1's `tools/probes/hand_sensor_verdict_replay.py`.
So a new pure module `ros_ws/src/jugglebot/jugglebot/toss_record.py` (no ROS, no
file I/O, `from __future__ import annotations` for the 3.8 target) owns `FIELDS`,
`encode`/`decode`/`validate`, `latch_announced_ball`, `label_from_sensor`,
`join`. That is what stops the live and offline definitions of "caught" from
drifting apart.

### 3.5 The recording gap that blocks all of it

There are **two divergent bag-record lists today and neither is sufficient**
(both verified):

| List | Has | Missing |
|---|---|---|
| `jugglebot_launch.py` (`record:=true`, the `rosbag_record` block) | `/balls`, `/mocap_data`, `/qtm_clock_offset_sec`, `/throw_announcements`, `/hand_telemetry`, `/link_status`, `/trajectory/status`, `/gravity_offset`, `/catch/dynamic_target` | `/rosout`, `/catch/pretilt_hold` |
| `tests/hardware/session_anomaly_fixes.md` § record list | `/rosout`, `/catch/pretilt_hold` | **`/balls`, `/mocap_data`** — the two the mocap landing offset and the tracker join need |

Neither carries `/catch/armed`, `/catch/prime_hold`, `/catch/vel_scale`,
`/catch/reach_center`, `/trajectory/commanded_position`, or any
`_action/feedback`.

**Cheapest and most urgent item in this plan: collapse to ONE list.** Extend the
launch list to the superset (add `/toss/record`, `/rosout`, `/catch/armed`,
`/catch/prime_hold`, `/catch/vel_scale`, `/catch/reach_center`,
`/catch/pretilt_hold`, `/trajectory/commanded_position`, both toss
`_action/feedback` and `_action/status`) and have the runbook say `record:=true`
instead of maintaining a second list. Every addition is low-rate except
`/trajectory/commanded_position` (40 Hz `Point`). A missing topic is
unrecoverable after the fact; a recorded silent topic costs nothing — the launch
file already argues exactly this for `/leg_lengths_topic`.

### 3.6 The trim law

Per axis `a ∈ {x, y}`, in **aim-angle units (rad)**, updated **between cycles
only**.

```
measurement   ŷ_k = S⁻¹ · land_err_mm_k / (4 · h_ach,k)        [rad]
              S = the 2×2 sign/axis map from (rx, ry) to (Lx, Ly),
                  PINNED BY PROBE (rung SC-0), never assumed

admission     w_k ∈ {0,1}  (§ 3.6.2)

estimate      r_n  = (n₀ · anchor_aim + Σ_{w=1} ŷ_k) / (n₀ + n),   n₀ = 4
              se_n = σ̂_y / √(n₀ + n),  σ̂_y = running robust sd of ŷ

gate          APPLY only if  n ≥ 3  AND  |r_n| ≥ 2·se_n  AND  |r_n| ≥ DEADBAND
step clamp    δ ← δ_prev + clamp(−r_n − δ_prev, ±STEP_MAX)
trim clamp    |δ| ≤ TRIM_MAX
TOTAL clamp   |M(P) + δ| ≤ TOTAL_MAX      — re-clamped AT APPLY, not only at update
```

**Constants, each justified by the failure it prevents.** All PROVISIONAL until
the first corpus.

| Constant | Value | mm at h=0.78 | Root cause |
|---|---|---|---|
| `n₀` | 4 | — | The prior must not dominate after a handful of tosses; at 4 its weight falls below 50 % at n = 5, i.e. one node's worth of data. |
| `DEADBAND` | 0.10° = 1.745e-3 rad | 5.5 | A correction worth commanding must be ≥ ~1/6 of the 35 mm capture radius. Below that the loop churns the commanded pose for no measurable catch benefit. |
| `STEP_MAX` | 0.10° | 5.5 | Bounds the commanded-orientation discontinuity between consecutive tosses. At the platform extremity that is `219.075 mm × 1.745e-3 = 0.38 mm` of leg travel, inside a profiled `go_to_pose`. |
| `TRIM_MAX` | 0.15° = 2.618e-3 rad | 8.2 | = θ_acc, the tilt map's own accuracy floor, and ~1.5–2σ of the `level` single-sample scatter (1.2–1.7 mrad/axis) the trim exists to cancel. **A trim demanding more than that is a plant change, and the loop must FREEZE and shout rather than integrate into it.** 8.2 mm is 23 % of the capture radius, so a fully saturated wrong-signed trim cannot by itself cause a miss. |
| `TOTAL_MAX` | 1.0° | 55 | Keeps the additive-rotation composition inside C-LEVEL-2's documented 1°×1° regime entry (cross term 1.523e-4 rad) and caps the cup-swing side effect at `64.78 × sin(1°) = 1.13 mm`. |

> **AMENDED BY THE 2e BUILD (2026-08-11) — read this table with
> `jugglebot/toss_trim.py`'s constants block beside it.** Four rows above are
> superseded by measurement, each with its probe and its root cause in that
> module and in the logbook's § Phase 2e; the rest ship verbatim.
>
> | Written above | Shipped | Why |
> |---|---|---|
> | apply at `n ≥ 3` | `N_MIN_APPLY = 6` | the gate is re-tested every update, so it is a sequential multiple comparison |
> | `\|r_n\| ≥ 2·se` | `SE_GATE = 2.5`, held on `SE_GATE_CONFIRM = 3` consecutive updates | the written gate commanded a trim in **45.7 %** of ZERO-bias sessions; 2.5 halves the zero-bias cost (2.01 → 1.03 mm) for ≤ 0.3 mm at the biases worth correcting |
> | DEADBAND on `\|r_n\|`, applied as written | same — but it is **not** on the step | a deadband on the step opens a 0.10–0.15° dead zone in which no correction can ever be commanded |
> | G8 "a shift > 3·se" | tabular CUSUM, `k = 0.5`, `h = 8.0` sd units | 3·se at n = 16 is 0.75 σ, undetectable inside a goal; re-derived from what G8 protects (2 % false alarm, 99.7 % detection of a 2σ shift within 6 tosses) |
>
> Two further deviations, same place: the two **authority** clamps (`TRIM_MAX`
> and the apply-time total) are **magnitudes**, not the per-axis boxes "per axis
> a ∈ {x, y}" reads as — a box permits `hypot` = 1.414× the bound, 41 % more
> authority than the number that was justified; and a third apply condition,
> `|r_n − δ| ≥ 2.5·se`, was added with **no new constant** because without it
> `δ` tracks `r`'s random walk forever (measured: never 20 quiet updates in 240
> tosses; with it, 0–2 commanded moves per session). § 3.6.3's CONVERGED and
> STALLED are restated against the residual demand `|r_n − δ|`, CONVERGED
> gained an upper-confidence-bound term, it may not fire while saturated, and it
> is **descriptive rather than latching** — the `FROZEN_*` states stay terminal.

**Why a decaying-gain shrinkage mean and not an EWMA** — see the decision table,
D6. In one line: the estimand is *constant within a session*, so a fixed-gain
filter never converges and injects noise forever, while a shrinkage mean's
standard error falls as `σ/√(n₀+n)` (at σ = 20: 8.9 mm at **n₀+n = 5**, 6.7 at
9, 4.0 at 25 — i.e. after 1, 5 and 21 admitted tosses on top of the `n₀ = 4`
prior). The gate `|r| ≥ 2·se` then guarantees a trim is only ever
applied when the estimated bias exceeds twice the noise the action injects.
(Shipped at **2.5·se, held over three consecutive updates** — the amendment box
above has the measurement that moved it. The argument for a decaying gain over a
fixed one is unaffected: it is about the estimand, not the threshold.)

#### 3.6.1 Structure and scalars

- **Default `δ(P) = c`, constant over the workspace** (2 parameters). The
  time-varying part of the error is common-mode by physics (§ 3.2). **Per-node
  trim is never fitted online** — that is what the explicit routine is for;
  letting a session fit 9–25 under-determined cells produces 9–25 noise chasers.
- An **affine** term `δ(P) = c + G·(P − P_anchor)` is opt-in and **structurally
  refused** unless ≥3 non-collinear visited cells spread ≥100 mm in both axes and
  ≥12 admitted tosses. Rank-deficient fits must be refused, not silently
  regularised.
- **Speed gain `k_v`** multiplies `event_vel_mps` so `h_ach → h_cmd`:
  `k̂_v = √(mean(h_cmd/h_ach))`, apply at n ≥ 5 with se ≤ 1 %, deadband 1 %, step
  2 %, authority ±10 %. Safe by construction — `x3`
  (`hand_stroke::STROKE_TOP_REV`) is algebraically velocity-independent, so a
  speed trim cannot move the hand toward the end stop, and `validate_event_vel`
  still gates against `[TEENSY_TRAJ_MIN_EVENT_VEL_MPS, TEENSY_TRAJ_MAX_EVENT_VEL_MPS]`
  = `[0.3, 7.0] m/s`, which a ±10 % trim on 3.91 m/s clears by 1.6×.
- **Release latency `τ` = median(`t_rel_phys` − `t_rel_cmd`)**, written into the
  session-local equivalent of `toss_release_latency_ms`
  (`config/hardware_config.yaml`, ships 0.0; the BB pattern already documented
  in the same file with 44.0 ms). Apply at n ≥ 5 with se ≤ 5 ms, deadband 10 ms,
  step 25 ms, authority ±150 ms. **Never persisted** — it is uptime-dependent by
  nature. This is the direct mitigation for the arm-suppression exposure below.

**The `τ` trim matters more than it looks.** `hand_stroke::stroke_clear_time`
withholds the catch arm until `t_release + t_dec(v) + ARM_SUPPRESS_MARGIN_S`,
with `ARM_SUPPRESS_MARGIN_S = 0.040 s` sized at **1.7× the worst measured
+12.8…+23.4 ms** announcement→physical-release shift. At ~16 h can-bridge uptime
the measured dispatch shift is **+118–133 ms**
([catch-robustness.md](catch-robustness.md) § Context) — 3× that margin. The
suppression window then closes while the throw stroke is still decelerating and
the catch arm lands mid-stroke: exactly the failure the margin exists to
prevent. Fresh-boot discipline is not hygiene here, it is load-bearing, and a
session-local `τ` is the cheapest structural backstop short of the uptime root
cause (owned by [refactor-2026-07.md](refactor-2026-07.md) Phase 7).

#### 3.6.2 Admission filter — anti-noise and anti-degraded-plant

A toss contributes only if **all** hold:

| Guard | Rule | Root cause closed |
|---|---|---|
| G1 release evidence | `throw_stroke_seen` **or** `ball_track_confirmed` | never learn from a stroke that did not fire |
| G2 track quality | ≥5 descending samples, fit RMS ≤ 3 mm, band fully above the rim | the probe's own SPARSE flag |
| G3 apex sanity | `\|h_ach − h_cmd\|/h_cmd ≤ 10 %` | a wrong h mis-normalises ŷ; route to the speed estimator instead |
| G4 plant health | `dip_below_x3 ≤ 0.10 rev`, `peak ≤ 10.060 rev`, braking `iq` tracks commanded | **do not learn against the braking-clamp plant** — the Phase-0 gate metrics reused per toss |
| G5 levelling | `gravity_correction_loaded` ∧ `tilt_map_loaded` ∧ applied ∧ `requires` versions match | no learning on top of a different Layer 0 |
| G6 uptime | always recorded; **spatial** estimates admitted, **`τ` and all label-based scoring refused** when the session's own residual-vs-uptime trend exceeds the between-node signal | the +118–133 ms datum |
| G7 outlier | `\|ŷ_k − r_n\| > 4·σ̂_y` ⇒ drop; **3 consecutive drops ⇒ FREEZE** | regime change, not noise |
| G8 change detector | two-sided CUSUM on ŷ; a shift > 3·se ⇒ **FREEZE and shout**, never re-converge | silently re-converging is how the braking clamp hid for a whole session |
| G9 ball evidence | `ball_held_valid` false ⇒ UNKNOWN, never collapsed | C-POSSESS-1 § 2 |
| G10 post-reload | the cycle immediately after a reload interlude is excluded (`RELOAD_SETTLE`) | the hand has just primed-to-top and retracted, the platform has just traversed home→node; that cycle is not steady-state |
| G11 retry cycle | a cycle retried under the `ABORTED_NO_RELEASE` carve-out (§ 3.9) is recorded and **excluded from every fit** | a retried cycle's dwell, hand state and dispatch history are not the steady-state ones the map describes |

> **IMPLEMENTED BY THE 2e BUILD (2026-08-11), with two notes.** All eleven guards
> now exist in ONE place — `jugglebot.toss_trim` — shared by the online trim and
> by `toss_fit_lib`'s offline fit, so the two can no longer disagree about which
> tosses are admissible. **G4** is enforced *conditionally*: `dip_below_x3_rev` /
> `stroke_peak_rev` ship null until the PLANT block is wired (§ 10), so a record
> that CARRIES them and is out of band is refused by name and one that does not
> is admitted with `g4_unenforceable` counted in the snapshot, the console and
> the proposal — refusing every record for a field the pipeline does not produce
> would make the guard indistinguishable from an outage, and admitting silently
> is how the braking clamp hid for a whole session. **G6** is D16's *measured*
> trend test, made concrete: `(|slope|·sd(uptime)) / sd(per-node means)` over the
> admitted reductions, refusing τ above 1.0 and reporting UNKNOWN — not "benign"
> — when the session has too few paired samples, no uptime span or a single node.

#### 3.6.3 Stop criteria — freeze, never zero

- **CONVERGED** — `|r_n| < DEADBAND` for 3 consecutive updates at n ≥ 6 ⇒ freeze
  `c`, keep recording.
- **STALLED** — `|δ|` at `TRIM_MAX` and `|r_n|` has not fallen ≥20 % over 3
  updates ⇒ freeze, ERROR naming the suspected plant fault. **Do not raise the
  clamp.**
- **DEGRADED** — any of G4/G7/G8 ⇒ **freeze at the current value**. Explicitly
  *not* zeroing: zeroing injects a `TRIM_MAX`-sized step into the next commanded
  pose, which is worse than holding a stale but bounded correction.
- **End of goal** — the trim is **discarded**. It is written only as a *proposal*
  to `temp/logs/<session>_trim_proposal.yaml` with `(c, se, n, node coverage,
  guards tripped, tilt_map_version, estimator_version)`. Promotion into
  `config/toss_calibration.yaml` requires the explicit routine and its acceptance
  gates. This is premise P1, enforced rather than documented.

> **AMENDED BY THE 2e BUILD (2026-08-11).** Read `|r_n|` above as the **residual
> demand** `|r_n − δ|` — this module's `r_n` is the *required total aim* (2c's
> fixed point, independent of what is applied), so read literally CONVERGED can
> never fire, because `r_n` stays at the plant bias no matter how well the trim
> cancels it. Three further corrections, all measured, all in the logbook's
> § Phase 2e: CONVERGED gained an upper-confidence-bound term
> (`demand + 2.5·se < DEADBAND`, because without it the criterion cannot tell "I
> have evidence of no bias" from "I have no evidence of a bias"); it **may not
> fire while `|δ|` is saturated** (a clamped trim with a small remaining demand
> is the STALLED case, not the converged one); and it is **descriptive and
> re-evaluated, not a latch** — the `FROZEN_*` states stay terminal, and a guard
> freeze overrides CONVERGED. P1 is enforced by the proposal being written in a
> schema `parse_toss_cal` REFUSES, so a mistaken `cp` into `config/` fails loudly.

### 3.7 Map format and versioning — `config/toss_calibration.yaml`

Machine-written, committed, versioned. Loader shape copied from C-LEVEL-2
verbatim: env override authoritative, repo source tree searched **before** the
ament share, `resolve_*_path` candidate search (**never** a `__file__` walk — a
tilt-cal Phase-2 finding), all-or-nothing validation, absent ⇒ silent identity
and **exactly today's behaviour**, invalid ⇒ keep previous + `success = False`,
and `toss_cal_loaded` / `toss_cal_applied` / `toss_cal_version` published on
status.

```yaml
version: 1
captured: {date, git_sha, tool, args, uptime_ms_first, uptime_ms_last, base_condition,
           partition_census, source_files: [{path, sha256}]}
requires: {tilt_map_version, level_offset_rad[2], estimator_version,
           estimator: {plane_mm_rule, band_mm, lateral_gate_mm, min_samples}}
units:    {aim: rad, height_scaling_exponent, h_capture_m}      # rad iff SC-1 says p≈1
jacobian: {S[2][2], gain_mm_per_rad, source: SC-0}
grid:     {z_mm, orientation: level, x_mm[], y_mm[]}            # same axes as tilt_calibration.yaml
aim_rad:  {rx[iy][ix], ry[iy][ix]}                              # HOME-REFERENCED field
anchor:   {aim_rad[2], n, se_rad[2]}                            # warm-start prior for the trim
speed:    {k_v, se}
stats:    {n_per_node[][], sd_rad[][], date_range[][], sigma_L_mm, R_eff_mm,
           stale_nodes[], failed_nodes[]}
```

**Three version strings, all in every record.**

1. `schema` = `"toss_record/1"` — bumps on field removal or semantic change;
   purely additive fields do not bump. A `FIELDS` drift-guard test pins it.
2. `toss_cal_version` = `<captured.date>-<sha256(numbers)[:8]>`, hashed over
   **the calibration numbers only** — schema version, both axes, the bias grids —
   **float-normalised**. Both details are copied from the tilt map's own two
   audit findings (`motion/tilt_map.py`, the `map_version` helper): hashing the
   provenance block churns the version on every re-emit (`170` vs `170.0`), and
   hashing ndarrays through `default=str` both collides on large grids and hashes
   identical maps differently.
3. `tilt_map_version` carried through, because an aim bias fitted under tilt map
   A is not valid under tilt map B (D3).

**The pooling rule — the load-bearing part.** `toss_cal_fit.py` partitions the
corpus by
`(tilt_map_version, bridge_fw_version, platform_fw_version, toss_tier, z_mm, hand_odrive_config_sha)`
and **refuses to fit across partitions** without an explicit flag; the census is
stamped into `captured`. This makes *"the 07-16 premises were measured on a
degraded plant"* structural rather than remembered.

**The fit, per grid node.**

1. Select in-partition records with `usable_for_aim_fit`.
2. Require `n ≥ N_MIN` (**PROVISIONAL 8**).
3. Reduce each toss to the commanded aim that would have landed it on B:
   `b_i = map_aim_rad_i + trim_aim_rad_i − J⁻¹·land_err_mm_i`,
   then `bias_new = trimmed_mean(b_i)` (drop top/bottom 10 %), reporting `n`,
   `sd`, `date_range`.
   **CORRECTED BY THE 2c BUILD (2026-08-11), twice over.** This row originally
   read `+ S⁻¹·land_err/(4·h)`. (a) The sign is a **minus**: with applied aim `A`
   and plant bias `ψ` the landing error is `J·(A + ψ)`, so the plus form
   evaluates to `2A + ψ` — at `A = 0` it ships the plant bias *uncancelled*,
   which (since `aim_rad` is the COMMANDED aim) roughly doubles the landing error
   and diverges on the next capture. The minus gives `A − (A + ψ) = −ψ`,
   independent of `A`, which is exactly the fixed-point property the paragraph
   below claims. (b) `S⁻¹/(4h)` is replaced by `J⁻¹`, where
   `J = ∂(aim_target_offset_mm)/∂(rx, ry)` is finite-differenced from the
   **production apply path** — so there is no second implementation of the aim
   geometry for a sign to be wrong in. Measured: `J = [[0, 3126.5], [−3126.5, 0]]`
   mm/rad at h = 0.78 m, i.e. `S` is a **90° rotation, not a scaled identity**,
   and the magnitude is 0.209 % above the idealised `4h` (this is the derivative
   at zero aim; 2b's 54.578 mm/deg is the same quantity as a secant at finite
   aim, and the model's real curvature separates them in the fourth significant
   figure). SC-0 still measures `S`
   on hardware — it now confirms the plant obeys the production model rather than
   supplying a number the fit would otherwise have to guess.
   **Because the applied bias is recorded per toss, captures do not have to run
   with the map uninstalled** — this is the tilt plan's home-referencing
   amendment transposed, and it removes the `--force-uninstall` dance from the
   routine capture path (it stays as an escape hatch). It also makes the fit a
   converging fixed-point rather than a one-shot measurement, so records inside
   one node carrying different applied biases are harmless.
4. **Inclusion by label:** CAUGHT and BOUNCED included (the ball got there; the
   offset is real). **MISSED included iff the mocap fit is non-sparse** — those
   are the most informative aim records and excluding them biases the fit toward
   the cup. NO_RELEASE and UNKNOWN excluded. `rimshot` candidates included for
   aim, excluded from timing.
5. Speed fit consumes `achieved_flight_s_mocap` vs `flight_time_s`; timing fit
   consumes `t_departure_raw_ros − announce_throw_time_ros`, gated on
   `ball_held_stamp_wall_anchored` and on `sensor_poll_dt_ms_median` inside
   **[20 ms, 200 ms]**.
   **RE-DERIVED BY THE 2c BUILD (2026-08-11).** The original gate ("within 10 %
   of 20 ms", the configured `JB_BD_CHECK_INTERVAL_MS`) refuses 100 % of records:
   the measured per-record cadence on `2026-08-10_16-30-44` is 60 / 63 / **70** /
   80 / 87 ms (min/p5/median/p95/max). The replacement is a **precision** gate,
   because the measured departure-shift sd (20.51 ms) equals the poll
   quantisation `Δ/√12` (20.50 ms) to a ratio of 1.001 — the whole observed
   dispersion is the instrument. Floor = the configured interval (a cadence
   *faster* than the poller means the stamp is not the poll stamp). Ceiling =
   200 ms, where `se ≤ 5 ms` needs `n = Δ²/300` = **133** admitted tosses, more
   than the entire 129-toss first capture, i.e. the cadence at which a timing fit
   stops being reachable inside a sitting.
6. Outside the hull: **clamp to the nearest node, never extrapolate** (C-LEVEL-2
   verbatim — a wrong-signed edge extrapolation aims worse than no map).
7. **Deliberate deviation from the tilt tool:** a node with `n < N_MIN` **keeps
   its previous value**, marked `stale: true` with its `n` and `last_updated`,
   rather than failing the whole write. The tilt tool refuses because a tilt
   capture is one complete measurement; this map is an *incremental refinement*,
   and refusing would block progress on 24 good nodes because one had a thin
   week. **Never interpolated from neighbours** — that invents calibration
   exactly where the machine had trouble.
8. `--dry-run` prints the node-by-node diff and writes nothing; `--no-apply`
   captures and validates without writing; `--group NAME=<selector>` mirrors
   `ball_arrival_offset.py`'s A/B CLI, which is how catch-knob ladders get scored
   post-restore.

### 3.8 The explicit calibration routine — `tests/hardware/toss_cal_grid.py`

Modelled on `tests/hardware/tilt_cal_grid.py` deliberately: same location, same
`add_argument_group` shape (`grid` / `timing` / `verification` / `workflow`),
same fail-closed posture, `--dry-run` / `--no-apply` / `--verify-only` /
`--force-uninstall` / `--yes` / `--on-fail`, `BaseException`-guarded
return-to-centre, artifacts to `temp/logs/`, map to `config/`, then
`toss/reload_calibration` (`std_srvs/Trigger`, mirroring
`trajectory/reload_tilt_map`) **plus a version readback** — the readback is the
hard guarantee that the node loaded the file the tool wrote.

**It sends goals and observes. It never arms, never changes mode, never
dispatches a hand move, never publishes on a production topic.** The safing
ladder on every exit belongs to the coordinator (`_safe_toss_on_early_exit`) and
is reached by *cancelling the goal*.

**Preflight refusals — all hoisted before anything moves** (the tilt-cal audit
found a write-target check running *after* a four-minute sweep):

| # | Refusal | Root cause |
|---|---|---|
| R1 | wire DISARMED (`/link_status.mpc_active == 0`) | **the tilt-cal BLOCKING class**: `go_to_pose` is accepted while disarmed, nothing moves, and the tool writes + applies + *verifies* a plausible map that verified because nothing happened. Re-checked between nodes, **outside** the per-node `try`, so `--on-fail continue` cannot demote it |
| R2 | `gravity_correction_loaded == false` | every toss would be `REJECTED_NOT_LEVELLED` |
| R3 | `tilt_map_loaded == false` | this map is **downstream** of the tilt map; capturing without one bakes pose-dependent tilt error into the aim bias |
| R4 | `toss_trim_enabled == true` | session trim contaminates the persistent map |
| R5 | hand sensor not valid, or no live `held→empty` transition observed | no ground truth ⇒ no capture. A static read cannot see a stuck bit |
| R6 | `toss_tier != "8a"` | the map is defined at 8a |
| R7 | `uptime_ms` > 30 min at collection start | **stricter than `tilt_cal_grid.py` on purpose**: static inclinometer reads are uptime-insensitive; a *timing* bias is not |
| R8 | write target not writable / not the resolved first candidate | via `toss_cal_candidates()[0]` — **no `__file__` walk anywhere** |
| R9 | operator confirmations: floor clear, ball supply, E-STOP in reach | `--yes` skips them with a loud live-capture warning |

**Rungs, in blocking order.**

- **SC-0 — sign and gain probe (BLOCKS everything).** Centre node, h = 0.78.
  Commanded aim `(0,0)`, `(±0.5°, 0)`, `(0, ±0.5°)`, n = 5 each ⇒ 25 tosses. Fit
  `J = ∂L/∂(rx, ry)`. **Accept:** diagonal within ±25 % of the predicted
  `4h = 3120 mm/rad`, off-diagonal < 30 % of diagonal, **signs match the
  convention**. *Why blocking:* a sign flip inverts every node and aims the
  machine roughly twice as badly as no map at all — the tilt plan states that
  consequence in exactly those terms for its own reduction formula. The sign must
  be measured, not assumed from the geometry.
- **SC-1 — height ladder (decides the map's UNITS).** Centre node,
  h ∈ {0.45, 0.60, 0.78, 1.00}, n = 8 ⇒ 32 tosses. Regress `log|L|` on `log h`.
  Slope ∈ [0.7, 1.3] ⇒ **store in rad**, one map serves all heights.
  Slope ∈ [0.3, 0.7] ⇒ release-impulse origin, store in Δv units.
  Slope ∈ [−0.2, 0.2] ⇒ fixed-mm, store in mm **and refuse off-capture-height
  application**. CI spanning two branches ⇒ store at the working height only and
  WARN elsewhere (the hull-clamp doctrine applied to the height axis). Also fits
  `k_v` and gives the first honest `σ_L`.
- **SC-2 — the (x, y) grid.** **3×3 over ±150 mm at z = 170, n = 8 ⇒ 72 tosses
  for the first capture.** Centre-out node ordering, home anchors interleaved
  every `--home-revisit-every` nodes, anchor-mean referencing, per-node sd + n.
  5×5 (n = 8 ⇒ 200) only after a 3×3 map verifies. *Why 3×3 first:* the first
  capture's job is to find out whether a per-node **field** exists at all; a 5×5
  is 200 tosses of possibly-inverted, possibly-flat data and does not fit a
  comfortable sitting (§ 6).
- **SC-3 — verification.** ≥6 **off-node** check poses derived from the map's own
  axes, n = 5 each.

| Gate | Threshold | Root cause |
|---|---|---|
| Check-pose accuracy | `\|mean L\|` ≤ **10 mm** at ≥5 of 6 | ~1.2× θ_acc's 8.2 mm floor — asking for better is asking the ball to out-measure the inclinometer |
| Pooled common mode | `\|c\|` ≤ 6 mm | must fit inside one session-trim authority |
| Anchor series | report always; WARN at 2 mrad p-p; abort at 0.5° p-p or a 5 mrad consecutive step | the tilt-cal anchor gate verbatim — it is a *report first* |
| **Flat-field guard** | every node `\|M\| < θ_acc` (0.15°) ⇒ **WARN and refuse to write** | a map indistinguishable from tilt-map noise buys nothing and costs a version bump |
| **Ball-actually-flew guard** | every node needs ≥1 admitted toss satisfying G1+G2 | the direct analogue of the tilt-cal DISARMED capture that wrote a plausible all-zeros map because nothing moved |
| Map-vs-map diff (recapture) | max node delta ≤ max(2·se, 0.05°) | invariance check |
| Provenance | full `captured` + `requires` blocks, per-node `n`/`sd`/`date_range`, uptime span | D3 needs all of it |

**Node exhaustion is not a capture abort.** When a node's reload budget is spent
(`STOPPED_RELOAD_BUDGET`, § 3.9), the tool marks that node **thin/stale**, logs
the census, **skips to the next node and continues the capture**. The write rule
in § 3.7 item 7 already handles a thin node correctly (keeps its previous value,
`stale: true`), so an exhausted node costs one node's refresh, not a sitting.

**Forensics on abort.** `_meta.json` beside the per-toss CSV with `abort_reason`
**always** set (a tilt-cal finding — an abort that writes an empty summary is the
abort most in need of the data). Per-toss summaries appended as they complete,
never built in a trailing loop. Decoded ODrive errors to console **and** file.
Hand-sensor ledger ±3 s around the abort. Anchor series to date.
`except BaseException`, not `except Exception` — the tilt-cal BLOCKING finding
was exactly this, a second reflex Ctrl-C escaping the return-to-centre guard and
leaving the platform parked at a raised displaced pose in silence. Artefact write
and rclpy shutdown each in their own `finally`. First Ctrl-C cancels the live
goal and **prints whether the cancel is honoured now or deferred**:

```
CANCEL requested — cycle is in BALL_IN_FLIGHT: DEFERRED to the catch terminal (~1.2 s).
A second Ctrl-C will NOT skip this wait. Ball is airborne.
```

### 3.9 `TossContinuous` auto-reload-on-drop

**Trigger: cycle N+1's `REJECTED_NO_BALL` — not the MISS verdict.** Root cause,
three parts:

1. It covers the **bounce-out during a dwell**, which `stop_on_miss` structurally
   cannot see (`toss_session.py`'s S3 header says so in as many words).
2. It is the one toss terminal where the FSM provably commanded nothing:
   `REJECTED_NO_BALL` is minted in CHECKING, before `_positioned` or
   `_prepare_dispatched`, so `_terminal_action` returns **`ACTION_NONE`**.
   The interlude is entered from a provably quiescent state.
3. It needs no new "was that a drop?" predicate — the sensor answers the only
   question that matters.

**Hard prerequisite — SATISFIED.** `toss_require_ball_evidence` must be **true**,
otherwise a drop produces a *silent empty dry stroke* and the loop learns from a
toss that never happened. Catch-robustness Phase 1 flipped it: it now ships
`true` in `config/hardware_config.yaml`, CHECKING reads the cup live, and
`REJECTED_BALL_UNKNOWN` refuses on a sensor that cannot answer. The design's
contingency ("if Phase 1 has not landed, auto-reload must refuse to arm at all")
is therefore moot, but the **runtime** guard stays: the interlude reads the live
config value and refuses to arm if it is `false` (the operator's total-bypass
escape hatch must not silently re-open the dry-stroke path).

**Why the interlude must live inside the session.**
`reload_coordinator_node._goal_callback` takes the one-ball-op claim at goal
**ACCEPT** and a session holds it for its whole life, dwells included. An
external `ros2 action send_goal /jugglebot/reload` mid-session is
`REJECTED_BUSY`, and so is a tool-issued one. So auto-reload is a session-level
action driving the existing reload FSM through the node's own methods, under the
same claim. There is no workflow in which the operator reloads manually
mid-session without cancelling it.

**Goal fields, STOP as the IDL default** — the same doctrine `stop_on_miss`
already carries ("an omitted or unreadable field must mean STOP"):

```
on_empty_cup:  STOP (default) | RELOAD      # reachable only with stop_on_miss false
max_reloads:   0 => config default (3)
```

**Ball supply is operator-managed; there is no magazine fence.** No ball-count or
magazine field exists on `ball_butler_node.py`, so supply has **no machine
observability** and none is invented. `max_reloads = 3` is the only machine-side
fence, and on exhaustion the SESSION fails closed with `STOPPED_RELOAD_BUDGET`
(unchanged doctrine). The capture tool's response to that terminal is a **node
skip, not an abort** (§ 3.8).

**Known BB-side defect, retried within budget.** Some BB reload throws ABORT
because BallButler is not positioned in time. Inside the `max_reloads` budget
that abort is **retryable**: the interlude re-enters the reload FSM rather than
terminalising the session. The retry must be **targeted at the identifiable
code** in the BB/reload path and must name it in the log line — a blanket
"retry any BB abort" would swallow the fail-open boot bug below and every real
BB fault. Identifying that code is a build-time task for 2d; if no distinct code
exists, the retry is not shipped and the session stops as today (fail closed,
loudly, naming the ambiguity).

**The interlude ladder.** Every rung is an existing, validated mechanism; the
interlude invents no motion primitive.

1. **Precondition gate** (all must hold, else terminalise with a named code,
   nothing moves): `max_reloads` budget remains (`STOPPED_RELOAD_BUDGET`);
   floor-ball counter under the pause threshold
   (`STOPPED_FLOOR_CLEAR_REQUIRED`); `bb_connected ∧ bb_state == IDLE`
   (`STOPPED_BB_NOT_READY`); **BB `ball_in_hand` observed `false` at least once
   this session** (`STOPPED_BB_UNVERIFIED`, see below); hand sensor
   `ball_held_valid` and reads empty (`STOPPED_SENSOR_UNKNOWN`).
2. **`go_home` + verified arrival.** `reload_sequencer._step_checking` refuses
   off-centre with `REJECTED_NOT_CENTERED` unless the live commanded xy is within
   `reload_coordinator_node::_RELOAD_CENTERED_TOL_MM` =
   `JB_TRAJ_CATCH_REACH_ENVELOPE_MM (80.0) − HAND_CATCH_OFFSET_MM (64.78) · sin(12°)`
   = **66.53 mm**. The gate exists because the reload never pre-positions: from
   an off-centre park it arms a reach envelope centred off (0,0) and rejects the
   incoming BB ball **mid-flight, unsavable**. With
   `toss_stay_at_pose_on_caught: true`, "parked 150 mm off centre" is the
   *routine* state after a CAUGHT cycle. **Do not widen the tolerance and do not
   auto-return inside `reload_sequencer`** (its own comment explains why that was
   rejected). `_go_home()` returns on the service ACK at plan-install, not on
   arrival, so the session waits `toss_session::GO_HOME_DURATION_S` (2.0 s) + pad
   and then confirms `|commanded xy| ≤ 66.53 mm` from a **fresh**
   `trajectory/commanded_position`. Timeout ⇒ `STOPPED_RECENTRE_FAILED`, not a
   reload attempt.
3. **The reload FSM, verbatim** — same build/run shape, same abort ladder, same
   terminal.
4. **Settle**, handing back at
   `landing + toss_session::DEFAULT_SESSION_MISS_CLEANUP_S` (2.80 s =
   `CATCH_CONFIRM_WINDOW_S 0.7 + GO_HOME_DURATION_S 2.0 + 2×NODE_TICK_S`) as a
   floor, exactly as the MISS path already does.
5. **Post-reload discard** — the next cycle is flagged `RELOAD_SETTLE` and
   excluded from the learning record (guard G10).

**The BB fail-open boot bug — consumer-side fence.** BallButler heartbeats
`ball_in_hand = true` from boot **before its first GPIO read**, and the
coordinator gates that bit only on heartbeat freshness
([hand-ball-sensor.md](hand-ball-sensor.md) § the fail-open boot default). A
freshly-rebooted BB therefore makes the reload FSM **skip
`ACTION_CALL_RELOAD`**: it primes the hand, raises the latch, calls
`bb/throw_at_target` on an empty BB and dies `ABORTED_NO_ANNOUNCEMENT` after
`throw_delay + 0.5 s`, having armed everything for nothing. Inside an autonomous
session that is worse than at the bench because nobody is watching the
heartbeat. Two layers: (a) runbook — the operator watches `bb/heartbeat` through
one manual reload and confirms `ball_in_hand` goes false then true; (b) machine
— the session latches `_bb_ball_in_hand_observed_false` on the first `false` it
ever sees and the interlude refuses until it is set. The underlying defect stays
owned by its own investigation; the log line must say this is a consumer-side
fence.

**Abort ladder.**

| Cycle outcome | Session response | Rationale |
|---|---|---|
| `CAUGHT` | continue; feed the record | — |
| `CAUGHT` then sensor empty before next release | **RELOAD interlude**; record labelled `BOUNCED` | the retention case `stop_on_miss` cannot see |
| `MISSED` / `MISSED_INFEASIBLE_*` | `stop_on_miss` ⇒ STOP (unchanged); else next cycle's `REJECTED_NO_BALL` triggers the interlude | S3 preserved: "stopping" is literally not starting cycle N+1 |
| `REJECTED_NO_BALL` | **RELOAD interlude** | nothing moved, nothing armed (`ACTION_NONE`) |
| `ABORTED_NO_RELEASE` | **retry the cycle ONCE iff the hand sensor reads valid-HELD**; UNKNOWN or EMPTY ⇒ STOP as today; **two consecutive ⇒ STOP** | operator decision 6, § 9 — the sensor shows the ball is demonstrably still in the cup, so the airborne-ball hazard is structurally absent |
| every other `REJECTED_*` / `ABORTED_*` | STOP, `ABORTED_CYCLE_<outcome>`, verbatim | unchanged from today (`toss_session::note_cycle_result`) |
| any interlude gate failure | STOP with the named code | fail closed |

The retried cycle is recorded with a `retry_of` back-reference and excluded from
every fit by guard G11.

### 3.10 Layer 1.5 — the dwell inclinometer covariate

**APPROVED as a covariate, with zero control authority** (operator decision 2,
§ 9). During each dwell the coordinator takes **N = 8** SCL3300 reads at a
**0.15 s** gap (~1.2 s of a 6.0 s dwell) and records mean, sd and n in the toss
record. Nothing is applied — it is recorded so that "is arrival repeatability
actually the dominant σ term?" becomes a measurement instead of an argument
(D17). Promotion to a feedforward is gated on `R²` of `land_err` on the
pre-throw tilt ≥ 0.4, and is a separate decision, not part of this build.

**Two hard rules, in priority order.** `get_platform_tilt` **blocks the
Platform-Teensy loop that streams hand moves**, so:

1. **Reads NEVER overlap PREPARE→THROW.** They live entirely in the quiescent
   dwell, and the read schedule is abandoned the moment the cycle leaves dwell.
2. **If the dwell budget is tight, degrade the read count — never delay the
   throw.** A short dwell yields fewer reads and sets `dwell_tilt_degraded`;
   `dwell_tilt_n = 0` is a legal record. The throw's schedule is never a function
   of the covariate.

**Follow-on REGISTERED, not built here.** The structurally right fix is to
restructure the Platform-Teensy SCL3300 read into **timer-driven background
sampling into a cache** — the async-cache pattern the can-bridge hand-sensor
poller already uses — so a tilt read is a cache read and never blocks a control
loop. That matches the standing principle of RTOS-style determinism: no blocking
I/O in control loops. It is firmware work, out of scope for this build, and is
carried in the Open row of [catch-robustness.md](catch-robustness.md) and § 10
below.

---

## 4. Decision table

Each row: what was chosen, what was rejected, and the concrete failure the choice
prevents.

| # | Decision | Rejected alternative(s) | Root-cause rationale |
|---|---|---|---|
| **D1** | **The aim correction is an ANGULAR platform tilt at release**, applied through the existing tilted release path with a virtual aim target; the map stores radians. | (a) Offset the nominated catch xy (the OPERATOR lens's mm trim). (b) "Cup-aim": throw vertically and reach the cup to the measured landing point. | (a) is a **structural no-op** in 8a: `_toss_positioning_xyz` returns the catch pose as the throw site, so a shift moves release and cup together and the error relative to the cup is unchanged (F1, verified). (b) *would* work but chases the symptom: the residual is angular in origin (`b = 4h·ψ`), so a mm-valued map is only valid at the capture height and silently dies the first time the operator changes `throw_height_m` — a routine goal field. (b) also commands a **10–30 mm lateral platform translation during flight**, re-introducing exactly the hazard the toss is open-loop to avoid (2026-07-24: a corrupt track dragged the platform 83.7 mm in the last 0.8 s and cost the catch). The tilt equivalent moves the cup by `64.78·sin(0.4°) = 0.45 mm` — 20–60× less commanded in-flight motion for the same correction. Finally (b) is **not cheaper to build**: it needs the same `pretilt_hold` + deferred-reach plumbing (F3). |
| **D2** | **Ride the existing 8b tilt path**; re-key three branches on "release state carries a non-zero tilt" instead of `tier == TIER_8B`. | A separate 8a aim path. | `compute_release_state_tilted(B, T, throw_site_xy_mm=B_xy)` is documented and test-pinned to reproduce `compute_release_state` **bitwise**, so zero bias ⇒ today's machine bit-for-bit, and the disable path is provable rather than argued. A second aim implementation is a second place for a sign to be wrong. |
| **D3** | **`catch/pretilt_hold` raised for ANY non-zero commanded aim, tier-independent**, with a structural test asserting it. | Leave `pretilt_hold` as an 8b-only concern. | Without it the stock announcement pre-tilt completes an un-tilt to level **≥1 s before release** (`catch_coordinator_node._on_throw_announcement` docstring), reverting the aim on every toss **while every log line reports the trim as applied**. Silent-wrong is the expensive failure class here, not loud-wrong. |
| **D4** | **Announce the UNCORRECTED landing** (B, vertical arrival). | Announce the aim-corrected virtual target. | The announcement is the *prediction of where the ball goes*, and after correction that is B. Keeping it uncorrected leaves the correlation→catch path, the receive-tilt computation and the possession plausibility bound bitwise unchanged. Only the commanded pre-tilt pose and `event_vel` change. |
| **D5** | **Arrival-offset (mocap descending-branch fit) is the ONLY aim observable**; `catch_error_mm` is recorded as a diagnostic and forbidden as an estimator input. | Use the action's `catch_error_mm`. | It is a dead-reckoned free-fall extrapolation reading 0.30–3.88 mm on 17/17 CAUGHT self-tosses, and it exists **only for caught balls** — doubly wrong: selection-biased and measuring seated position, not arrival (F5, `ball_possession.py` module docstring). The correct instrument already exists and is outcome-independent by construction. |
| **D6** | **Shrinkage mean** with prior `n₀ = 4`, effective gain `1/(n₀+n)`, plus a `2·se` significance gate. | Fixed-gain EWMA (`a = 0.5`, `k = 0.35`). | The estimand — this session's `level` offset plus arrival-repeatability common mode — is **constant within a session**, and regime changes are handled by an explicit CUSUM freeze. A fixed-gain EWMA's standard error floors at `≈0.577σ` (≈11.5 mm at σ = 20) and therefore injects ~4 mm of fresh error per update **forever**; a decaying gain converges and its injected noise decays with n. EWMA is the right tool for a *drifting* target; this target does not drift. Removing the gain as a free parameter also removes the stability question entirely. |
| **D7** | **Two clamps, sized by what they protect**: session trim `\|δ\| ≤ 0.15°` (8.2 mm); persistent map `\|M\| ≤ 1.0°` hard with a `--allow-large` requirement above 0.5°; total re-clamped **at apply**. | A single ±12 mm total clamp (OPERATOR) or a single ±0.5°/±1.0° pair (ESTIMATION). | A single tight clamp **binds on the truth**: the inferred plant scatter suggests real residuals near 20 mm, so ±12 mm silently caps the programme's whole benefit and looks like convergence. A single loose clamp gives an unreviewed online estimator ±55 mm of authority — 1.6× the capture radius, so a saturated wrong-signed trim guarantees a miss. The map is written by a gated, sign-tested, operator-reviewed capture; the trim is fitted online from few samples with no review. Different review, different authority. `0.15°` is derived: it equals θ_acc (the tilt map's own accuracy floor) and is 1.5–2σ of the `level` scatter the trim exists to cancel, so a demand for more is a plant change, not a calibration. |
| **D8** | **Auto-reload triggers on cycle N+1's `REJECTED_NO_BALL`**, gated on `toss_require_ball_evidence: true`, behind a mandatory verified `go_home`. | Trigger on the MISS verdict; or reload without recentring. | `REJECTED_NO_BALL` is the one terminal whose `_terminal_action` is `ACTION_NONE` — nothing moved, nothing armed — and it is the only trigger that also catches a **dwell bounce-out**, which `stop_on_miss` structurally cannot see. Reloading from an off-centre park hits `_RELOAD_CENTERED_TOL_MM` (66.53 mm) and rejects a real BB ball **mid-flight, unsavable** — and after a CAUGHT cycle at a corner node, off-centre is the routine state. |
| **D9** | ~~No `ABORTED_NO_RELEASE` retry in v1.~~ **SUPERSEDED by operator decision 6 (2026-08-10, § 9): retry ONCE iff the sensor reads valid-HELD; UNKNOWN or EMPTY ⇒ stop as today; two consecutive ⇒ stop the session.** | The design's v1 position (defer the retry, measure the frequency, decide with data). | The design deferred on the grounds that the sensor made the retry *plausible* but not *measured*, and that `on_empty_cup: RELOAD` was already one relaxation of the absolute-stop rule. The operator reopened it on the strength of Phase 1's shipped evidence: the hand sensor was 100 % valid across 203,922 samples, and a **valid-HELD** reading at that terminal shows the ball is demonstrably in the cup — which makes the D9 airborne-ball hazard (`toss_sequencer` § "ambiguous ack ∧ blind telemetry ⇒ SAFE_ABORT retracts under an airborne ball") **structurally absent for exactly that reading and no other**. The tri-state gate is what makes the reopening safe: UNKNOWN is not HELD, and the two-consecutive stop preserves the epidemic gauge that would otherwise be traded away. |
| **D10** | **Bag-first record**: canonical declaration published on `/toss/record` (`std_msgs/String` JSON) + an offline miner that can produce the whole record alone in degraded form. Best-effort JSONL belt. | (a) Node writes the file. (b) Extend `toss_trace_recorder.py`. (c) Miner only. | (a) puts file I/O in the node that owns the hand, the latch and the abort ladder — a full disk can stall a teardown. (b) cannot see the goal, and it is an operator-remembered separate terminal. (c) cannot recover the requested-vs-resolved distinction, the FSM perf instants, the dispatch tri-state, or — decisively — **the calibration bias evaluated for this toss**. A typed message would need a two-package `colcon` build for every schema tweak, which is exactly the partial-build `ImportError` class the runbook's build gate exists to prevent; the cost of JSON-in-String is paid down by a pure module owning encode/decode/validate plus a `FIELDS` drift-guard test. |
| **D11** | **Production labeller + latch imported by both node and miner** (new pure `jugglebot/toss_record.py`). | Miner re-implements the labeller. | The live and offline definitions of "caught" would drift and nobody would notice until a map was fitted on the wrong labels. The precedent is already load-bearing: `possession_verdict_bag_check.py` constructs the arrival source *exactly as the node does*, deliberately, and Phase 1's `hand_sensor_verdict_replay.py` reconciled EXACTLY with independent transition counts because of it. |
| **D12** | **Raw edges for TIMES, debounced for the VERDICT**, with the *measured* poll cadence recorded. | Use `ball_held` (debounced) for both. | The 5-sample debounce at 50 Hz is a 100 ms window; using it as a timing measurand puts a systematic ~100 ms late bias into every timing fit — the same size as the uptime effect the whole fresh-boot discipline exists to control, and it would read as real physics. |
| **D13** | **UNKNOWN never collapses to a verdict**; `sensor_valid_frac < 1.0` in the decisive window ⇒ label UNKNOWN, all `usable_*` false. | Treat "no valid sample" as "no ball". | Mints a false MISSED on every telemetry hiccup, and those false MISSEDs are exactly the records the aim fit most wants (mocap-visible, non-caught). C-POSSESS-1 § 2 forbids it independently. |
| **D14** | **SC-0 (sign/gain) and SC-1 (height ladder) BLOCK SC-2**; SC-2 is 3×3 × 8 first, 5×5 only after a 3×3 verifies. | Go straight to a 5×5 grid capture. | A sign flip inverts every node and aims the machine roughly twice as badly as no map — 200 tosses of inverted data is a whole sitting wasted, and the failure is invisible until the verification pass. SC-1 decides whether the map's unit is rad, Δv or mm; getting that wrong makes the map silently invalid at any other `throw_height_m`. And 5×5 does not fit a comfortable sitting (§ 6). |
| **D15** | **Thin nodes keep their previous value, marked `stale`**; failed/never-flew nodes refuse the write; **never interpolate a node from neighbours**. | Copy the tilt tool's "any failed node ⇒ no write". | A tilt capture is one complete measurement, so all-or-nothing is right there. This map is an incremental refinement, and refusing the write would block 24 good nodes because one had a thin week. Interpolation is refused for the opposite reason: it invents calibration precisely where the machine had trouble. |
| **D16** | **No invented uptime ceiling.** Hard session-start discipline (fresh boot; abort if `uptime_ms` > 30 min at collection start), `uptime_ms` per toss, and the analyser **refuses a timing fit** whose within-session residual-vs-uptime trend exceeds the between-node signal. | A 2 h or 4 h hard ceiling. | The measured datum is **one point** (+118–133 ms at ~16 h). Neither 2 h nor 4 h is measured, and a sitting is 2–3 h so an above-session ceiling is decorative. Converting the threshold into a *measured within-session trend test* makes the corpus police itself. |
| **D17** | **Layer 1.5 (per-toss inclinometer read in the dwell) ships as a COVARIATE ONLY**, promoted to feedforward only if `R²` of `land_err` on the pre-throw tilt ≥ 0.4. **Operator-APPROVED 2026-08-10** (decision 2, § 9). | Build the feedforward now; or drop it. | It would remove the arrival-repeatability term (1.6–1.8 mrad ≈ 5.0–5.6 mm) and today's `level` common mode **with no loop and therefore no oscillation risk** — genuinely attractive. But it costs ~1.2 s of a 6.0 s dwell (N=8 reads at 0.15 s gap gives σ/√8 ≈ 0.5 mrad ≈ 1.6 mm) and `get_platform_tilt` **blocks the Platform-Teensy loop that streams hand moves**, so reads must live in the quiescent dwell and never inside PREPARE→THROW. Recording it first answers "is arrival repeatability actually the dominant σ term?" for the price of dwell time and zero new authority. |
| **D18** | **One bag-record list**: extend the launch list to the superset; the runbook says `record:=true`. | Keep two lists. | Both current lists are individually insufficient and their union is required. A missing topic is unrecoverable after the fact; a recorded silent topic costs nothing — the argument the launch file already makes for `/leg_lengths_topic`. |
| **D19** | **`max_reloads = 3`, session fails closed on exhaustion (`STOPPED_RELOAD_BUDGET`), and the capture tool treats that terminal as a NODE skip, not a capture abort.** No magazine fence is built. | A machine-side ball-supply model; or aborting the capture on budget exhaustion. | No ball-count or magazine field exists on `ball_butler_node.py`, so a supply model would be a fiction with a number attached. Failing the session closed keeps the doctrine; skipping the node keeps a thin node from costing a sitting, and § 3.7 item 7 already writes a thin node correctly (previous value, `stale: true`). Operator decision 4, § 9. |

---

## 5. Build-phase plan

Sized for this codebase's conventions: pure logic in importable modules, ROS
nodes thin, `from __future__ import annotations` everywhere under `ros_ws/`,
`./run_tests.sh --full` at every phase closure (the `nightly` tier covers
`controller/` and `sim/`), one shared logbook entry for the build with a section
per phase carrying the (date, command, result) triple, `/audit --unstaged` before
any commit touching ≥2 narrative markdown files.

**Everything in phases 2a–2f is desk-side.** No hardware is needed or touched
(operator decision 1, § 9); catch-robustness Phase 0 runs before the first
capture and the operator runs the capture.

**Prerequisites — both satisfied.** The 20 stale `tests/motion/test_tilt_cal_grid.py`
failures are fixed (§ 0). Catch-robustness Phase 1 has landed, so 2d's
`toss_require_ball_evidence` prerequisite holds and the shipped possession merge
is **consumed, not re-implemented**.

| Ph | Scope | Deliverables | Gate |
|---|---|---|---|
| **2a** ✅ **LANDED 2026-08-10** | **Instrument only — zero new control authority.** No map, no trim, nothing applied. | AS SHIPPED: `jugglebot/toss_record.py` (pure: `FIELDS`, encode/decode/validate, `label_from_sensor`, `latch_announced_ball`, `join`, `names_by_origin`); `/toss/record` publisher at the FSM terminal + best-effort JSONL belt; the Layer 1.5 dwell-tilt **schema only**, nullable (the READS move to 2d — operator's build spec); `tools/probes/toss_record_miner.py` with `--self-check` / `--emit-fixture` / `--sensor-only`; the **one** bag-record list; `tests/motion/test_toss_record.py` + `tests/ros/test_toss_record_publisher.py` + `tests/ros/test_toss_record_miner.py` (in `tests/ros/` beside its fixture, not `tests/sim/`). DEFERRED: the PLANT block and `floor_arrival` (§ 10). | MET: full suite green (`./run_tests.sh --full`, 2026-08-10). Miner reproduces the hand-mined ground truth of `2026-08-10_16-30-44` — **39 departures / 38 catches / 3 quick-drops** — as `tests/ros/toss_record_fixtures.py` in the `possession_fixtures.py` pattern, with a graceful skip when the bag is absent. `FIELDS` drift-guard passes. **NOT met, because not built:** the dwell-read schedule test — it belongs with the reads in 2d. |
| **2b** ✅ **LANDED 2026-08-11** | **Map plumbing, applied at zero.** | `config/toss_calibration.yaml` loader (`jugglebot/motion/toss_cal.py`, C-LEVEL-2 loader shape: candidates, env override, all-or-nothing validation, `map_version` over float-normalised numbers only); `toss/reload_calibration` Trigger + status fields; the aim applied in `_build_toss_cycle` through the tilted path; the three `TIER_8B` branches re-keyed on non-zero tilt; **`catch/pretilt_hold` raised for any non-zero aim** | MET: **zero-bias bitwise identity** pinned in `tests/motion/test_toss_release.py` (the aim path at bias 0 equals `compute_release_state` field-for-field, and the offset is exactly `[0.0, 0.0]`), and at the node the disabled path returns the SAME OBJECT (`is`, not `==`). `pretilt_hold` structural test over every axis and sign. D4 single-lookup: an AST manifest pinning `toss_cal.lookup` to one scope and that scope to one caller. Absent map ⇒ no new rejection code, no new topic traffic. `./run_tests.sh --full` (run 2026-08-11) → **5009 + 9 passed, 3 xfailed in 515 s**. **Deviations, both in the logbook**: the status fields ship on a latched `toss/calibration_status` JSON topic, not on `TrajectoryStatus` (a different node publishes that message and cannot know whether this map is applied); and the version hash also covers `units.aim`, `anchor.aim_rad` and `speed.k_v`, because 2e acts on all three. |
| **2c** ✅ **LANDED 2026-08-11** | **Closed-loop sign test, offline.** | AS SHIPPED: `tests/hardware/toss_fit_lib.py` (the pure core — partition rule + census, the fixed-point reduction, admission, the D15 thin-node rule, both write-refusing guards, the document build validated through the production loader, `synthetic_corpus`), `tests/hardware/toss_cal_fit.py` (thin CLI: `--dry-run` / `--no-apply` / `--group` / `--allow-cross-partition` / `--allow-flat-field` / `--reload` + version readback), `tools/toss_cal_analyse.py` (heat map + quiver in LANDING space, per-node n/sd, anchor series, residual-vs-uptime scatter, map-vs-map diff, `--group` A/B, HTML+PNG to `temp/reports/`, `--json`) | MET: the closed loop injects a known bias, runs the REAL fit, installs the map and replays through the **production apply path** — 8+ mm uncorrected → **< 1 mm** corrected; a sign flip is pinned to fail by **>1.8×** the uncorrected error. Spatial-field recovery checked node by node on an asymmetric field. Both guards refuse the write. Version stability: identical numbers ⇒ identical version, one node ⇒ changed. `./run_tests.sh --full` (run 2026-08-11) → **5084 + 9 passed, 3 xfailed in 518 s**. **Deviations, all in the logbook**: § 3.7 item 3's sign corrected (`+` → `−`) and `S⁻¹/(4h)` replaced by a Jacobian differentiated out of the apply path; § 3.7 item 5's timing gate re-derived; home-referencing invariance is 1e-6 rad rather than byte-identical (the ballistic model is only second-order linear); `--allow-flat-field` added as a documented override. **NOT met, because not built:** G4's braking-clamp REFUSE (the PLANT block still ships null) and D16's automatic timing-fit refusal (reported, not enforced). |
| **2d** ✅ **LANDED 2026-08-11** | **`TossContinuous` auto-reload — the first phase that commands motion.** | AS SHIPPED: `on_empty_cup` (IDL default `"STOP"`, re-applied by a **whitelist** resolver — anything not exactly `RELOAD` is STOP) / `max_reloads` (0 ⇒ config 3; negative ⇒ `REJECTED_BAD_GOAL(max_reloads)`) / `reloads_used` on the result; `SESSION_ACTION_RELOAD` + `SESSION_PHASE_RELOAD` with the § 3.9 ladder verbatim; verified-arrival recentre (`GO_HOME_DURATION_S` + a **measured** 1.5 s pad, timeout ⇒ `STOPPED_RECENTRE_FAILED`, never a reload attempt); the BB `observed_false` fence; the targeted `THROW_ABORTED_NOT_SETTLED` retry within budget — **which needed a new wire**, see the deviations; the valid-HELD-gated `ABORTED_NO_RELEASE` single retry with its two-consecutive stop; floor tally + pause; `reload_settle` / `retry_of` / `goal_on_empty_cup` / `goal_max_reloads` and the **Layer-1.5 dwell reads** written to the record; four config keys | MET: `./run_tests.sh` (run 2026-08-11) → **RESULT PASS, 4752 passed of 5187 collected in 221 s**; `colcon build --packages-select jugglebot_interfaces jugglebot` (run 2026-08-11) → **2 packages finished, 0 failed**, and the BUILT IDL is read back (`on_empty_cup` `'STOP'`, `max_reloads` 0, `reloads_used` on the result) so the test mock mirrors the wire rather than the file. Node tests for every named stop code (`STOPPED_BALL_EVIDENCE_DISABLED` / `_BB_NOT_READY` / `_BB_UNVERIFIED` / `_SENSOR_UNKNOWN` / `_CUP_NOT_EMPTY` / `_RECENTRE_FAILED` / `_RELOAD_BUDGET` / `_FLOOR_CLEAR_REQUIRED`); the off-centre park cannot enter the interlude; an omitted `on_empty_cup` STOPS (plus a 9-case whitelist parametrisation); the NO_RELEASE retry is tri-state-gated; a live `false` `toss_require_ball_evidence` refuses to arm; `stop_on_miss` unchanged under both policies; the dwell reads have exactly ONE call site and it is the quiescent-dwell branch (structural test). **Deviations, all in the logbook**: `THROW_ABORTED_NOT_SETTLED` exists but was **unobservable** (`bb/throw_at_target` is fire-and-forget), so `ball_butler_node` now relays the firmware's terminal outcome on `bb/throw_outcome` and the retry keys on that named code; the sensor rung ships **two** codes (UNKNOWN vs the SEATED contradiction), fail-closed; the session completion test moved from `cycle_index` to `throws` (behaviour-identical for every pre-2026-08-11 session) so a drop costs a reload rather than a data point; `_execute_reload` was deliberately NOT refactored. **NOT met as designed:** the Layer-1.5 read budget does not fit the shipped cadence — see § 10. |
| **2e** ✅ **LANDED 2026-08-11** | **The session trim.** | AS SHIPPED: `jugglebot/toss_trim.py` — the shrinkage estimator (`n₀ = 4`), per-axis significance + deadband gates, magnitude `STEP_MAX`/`TRIM_MAX` clamps, admission guards G1–G11, a two-sided CUSUM freeze, CONVERGED/STALLED, freeze-never-zero on every guard path, `fit_affine` structurally refusing a rank-deficient geometry, and the `k_v` / session-local `τ` estimators with their own gates and authority (`τ` never persisted); **plus the reduction, the Jacobian and the three `admit_for_*` filters MOVED here from `tests/hardware/toss_fit_lib.py`** and imported back — one implementation shared by the online trim and the offline fit (D11's argument one layer up), which is also what finally implements G4 (conditionally, gap counted) and G5 (fully). At the node: `toss_trim_enabled` (default **false**), read ONCE at `_build_toss_cycle`, the TOTAL re-clamped **at apply** over `map + trim`, map/trim/total recorded separately, one ingest point fed by the canonical declaration, the end-of-goal proposal to `temp/logs/`, and the `TRIM` console block | MET: `./run_tests.sh` (run 2026-08-11) → **RESULT PASS, 4847 passed in 218.09 s**; `./run_tests.sh --full` (run 2026-08-11) → **RESULT PASS, 5271 passed + 3 xfailed in 475.76 s parallel and 9 passed in 40.41 s serial, total 521 s**. All five property gates pinned (`tests/motion/test_toss_trim.py`, **78** collected) plus the node seams (`tests/ros/test_toss_trim_node.py`, **18**) including a D4-shaped AST manifest for the single trim read and the single ingest point. **Deviations, all in the logbook**: `SE_GATE` is 2.5 not 2 and needs 3 consecutive confirmations at n ≥ 6 (the design's gate commanded a trim in 45.7 % of NOISE-ONLY sessions); G8's `3·se` re-derived as `k = 0.5, h = 8.0`; the deadband is on the ESTIMATE not the step; CONVERGED is descriptive rather than latching and cannot fire while saturated; the two authority clamps are magnitudes, not per-axis boxes. **NOT met, because it does not exist:** the loop cannot close LIVE — `land_err_mm` is a MINED field and both live candidates are D5-forbidden, so a live record is refused by name (`no_mocap_fit`) and the trim commands zero. See § 10. |
| **2f** ✅ **LANDED 2026-08-11** | **Acquisition tool.** | AS SHIPPED: `tests/hardware/toss_cal_grid.py` — rungs SC-0…SC-3 with the § 3.8 gates, all R1–R9 hoisted into ONE pure function over an observation dict, `--dry-run` printing node order + toss count + ETA + **ball budget** + a **gate POWER report**, `BaseException`-guarded probe-map restore *then* return-to-centre, `_meta.json` with `abort_reason` always set, per-toss rows appended **inside** the goal's spin loop, `STOPPED_RELOAD_BUDGET` ⇒ thin/stale + skip + continue, uniform **probe maps** for SC-0's commanded aims (written through `toss_cal_candidates()[0]`, reloaded, version read back, restored on every exit), a **rung ledger** (`temp/logs/toss_cal_rungs.json`) that makes "SC-0 BLOCKS everything" a refusal rather than a sentence, and desk-side `--score` from a mined corpus. Grid geometry + the disarmed-wire verdict are IMPORTED from `tilt_cal_grid.py`, not restated | MET: `./run_tests.sh` (run 2026-08-11) → **RESULT PASS, 4965 passed in 216.76 s**; `tests/motion/test_toss_cal_grid.py` → **117 passed** (run 2026-08-11). `--dry-run` makes zero ROS calls with `rclpy` monkeypatched to explode; the `STOPPED_RELOAD_BUDGET` branch is pinned to `continue` and to carry no `raise`; the wire check is pinned OUTSIDE the per-goal `try` **and** re-run per goal; the safety envelope is a set-equality manifest over every `create_client` / `create_subscription` / `ActionClient` site with **zero publishers**. **Deviations, all in the logbook**: every gate is three-valued and refuses on EVIDENCE (the literal SC-0 gate refuses a perfect plant 67.4 % of the time at the design's own σ, SC-3's 92.0 %); SC-0's accept test is applied in the PREDICTED Jacobian basis (2c measured `S` as a 90° rotation, which inverts the design's literal diagonal/off-diagonal wording); capture and scoring are two invocations because `land_err_mm` is MINED; SC-3's ≥6 off-node poses need edge midpoints on a 3×3; § 5 P5.4's doubled-bias wording corrected. **NOT met as designed:** SC-1 cannot decide the units at any sample size that fits a sitting — see § 10. |

**2a is DONE** (2026-08-10). Its gate passed with two amendments the bags
forced — the catch-search window and the timing-fit poll gate, both in § 10 — and
one deviation from the deliverable list: the Layer-1.5 dwell READS are deferred
to 2d (2a lands the nullable schema only), and the miner's test lives in
`tests/ros/` next to its fixture rather than `tests/sim/`. Full write-up:
`logbook/2026-08-10-toss-selftuning-build.md` § Phase 2a.

**Ordering rationale.** 2a ships value on day one (the three existing bags become
a corpus) and carries zero hardware risk. 2b lands the authority with the disable
path *proved* rather than argued. 2c makes the sign wrong-ness catchable offline,
before any hardware sitting. Only then does hardware time get spent.

---

## 6. Bench / session plan

Operator runs every actuating command; the tools send goals and observe. Cadence
stays at the 6.0 s dwell (operator decision 3, § 9) — the 4.10 s fork is not
built.

### P0 — Desk, no robot (~40 min)

| Step | Command | PASS |
|---|---|---|
| 0.1 | `cat temp/reports/nightly/status` | fresh GREEN |
| 0.2 | `git fetch && git status -sb` | no unexplained divergence or foreign working-tree edits |
| 0.3 | `./run_tests.sh --full` | green (mandatory pre-sitting) |
| 0.4 | `cd ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot` | **both** packages — a partial build takes down every ball-op action |
| 0.5 | `python3 tests/hardware/toss_cal_grid.py --dry-run` | node order, toss count, ETA, **ball budget**, zero ROS calls |

**ABORT** if 0.3 is red for anything outside a known-stale set. A capture on an
unvalidated tree writes a calibration nobody can trust later.

### P1 — Power-up and arm (~10 min)

1. **Power-cycle the can-bridge Teensy.** Non-negotiable for this workstream: the
   map bakes in a *timing* bias and the dispatch shift is +118–133 ms at ~16 h
   uptime. A capture on a warm bridge is a capture of the drift.
2. Launch → home → **`level`** → activate → confirm 40 Hz hold → TRAJECTORY →
   **zero motion at arm**.
3. Record `uptime_ms` from `/link_status`; every downstream number quotes it.

**ABORT** if `uptime_ms` at collection start is already > 30 min — reboot and
re-arm.

### P2 — Read-only preflight (~10 min)

```bash
ros2 topic echo /trajectory/status --once | grep -E 'gravity_correction_loaded|tilt_map_loaded|tilt_map_version'
# The aim map is owned by reload_coordinator_node (decision 7), so its status is
# NOT on /trajectory/status — trajectory_node does not load it and cannot know
# whether it is applied. Corrected by the 2b build; latched, so --once answers.
ros2 topic echo /toss/calibration_status --once   # toss_cal_loaded / _applied / _version / dormant_reason
ros2 topic echo /link_status --once      | grep -E 'uptime_ms|hand_ball_sensor'
ros2 param get /reload_coordinator_node toss_trim_enabled
ros2 topic echo /bb/heartbeat --once     | grep -E 'state|ball_in_hand|connected'
```

| Check | Required | Why a refusal, not a warning |
|---|---|---|
| `gravity_correction_loaded` | true | every toss would be `REJECTED_NOT_LEVELLED` |
| `tilt_map_loaded` + version | true, and the version the new map will record | this map is downstream of the tilt map |
| `hand_ball_sensor` | `held` or `empty`, never unknown/stale | the loop's entire ground truth |
| One live `held → empty` observed | yes | kills the stuck-bit case a static read cannot see |
| `bb/heartbeat ball_in_hand` observed **false** since BB boot | yes | the fail-open boot bug |
| `toss_trim_enabled` | **false** during any capture | trim contaminates the persistent map |
| `toss_tier` | `8a` | the map is defined at 8a |

Physical: floor clear; ball supply staged (operator-managed — no machine fence
exists); E-STOP in reach; nothing under the platform.

### P3 — Baseline (~12 min, 20 tosses at the home node)

Trim off, map loaded (or absent for the first capture). 20 tosses at
`(0, 0, 170)` at the working height.

**Purpose is not tuning — it is a plant-identity check.** It answers *"is this
the same machine the map was captured on?"* before 72–200 tosses are spent on the
assumption that it is.

- **PASS:** catch rate within the map's recorded home-node band; median
  `land_err` within the map's recorded home residual + 2σ; `iq_brake_min_a`
  inside the post-restore band.
- **ABORT:** materially below the recorded baseline ⇒ the plant changed. **Stop
  and diagnose. Do not re-capture on top of it** — a map captured on a degraded
  plant is worse than no map, because it is applied confidently. This is the
  phase that catches a recurrence of the 2026-08-10 braking-clamp class before it
  costs a whole sitting.

### P4 — Collection

**First-ever capture: SC-0 (25) → SC-1 (32) → SC-2 3×3 × 8 (72).** Centre-out
node order; home anchors interleaved. SC-0 and SC-1 are one-node rungs and cost
~12 min of throwing between them. A node whose reload budget is exhausted is
marked thin/stale and skipped; the capture continues at the next node.

### P5 — Review, write, verify (~20 min)

1. `tools/toss_cal_analyse.py` — heatmap + quiver of the aim residual, per-node
   catch/bounce rate, **residual-vs-uptime scatter**, anchor series, map-vs-map
   diff. HTML/PNG to `temp/reports/`, `--json` for machines.
2. `toss_cal_fit.py` writes `config/toss_calibration.yaml`, calls the reload
   service, **reads back `toss_cal_version`**.
3. Verification pass: 6 off-node check poses × 5 tosses, scored against the
   freshly-applied map. Non-zero exit on FAIL.
4. **First hardware application of a new map uses a deliberately DOUBLED bias on
   ONE node.** If doubling the bias doubles the error, the sign is wrong, and one
   sitting says so instead of a whole grid.
5. Logbook entry — this arc hits at least two Discussion triggers.

### Decision points

| # | When | Decision | Criterion |
|---|---|---|---|
| D-a | P0 | run at all | full suite green |
| D-b | P2 | sensor trustworthy | live `held→empty` observed; else no sitting |
| D-c | P3 | plant is the mapped plant | baseline within band; else **STOP and diagnose** |
| D-d | P4 | SC-0 result | signs match + gain within ±25 % ⇒ proceed; else **stop**, do not capture a grid |
| D-e | P4 | SC-1 result | exponent CI selects the map's units; ambiguous ⇒ working height only |
| D-f | P4, per drop | continue or clear the floor | floor counter ≥ `--floor-pause-every` (PROVISIONAL 5) ⇒ clean stop between cycles |
| D-g | P4 anytime | abort | anchor p-p beyond tolerance, leg fault, sensor UNKNOWN, any `ABORTED_*` outside the § 3.9 carve-outs |
| D-h | P5 | write | ≥ `N_MIN` usable tosses at every node, or thin nodes explicitly kept `stale`; no never-flew node |
| D-i | P5 | keep or revert | verification all PASS; else `git checkout config/toss_calibration.yaml` + reload |
| D-j | after | enable session trim | only once a persistent map exists **and verifies**. Trim on top of no map is tuning against an unknown baseline |
| D-k | later | touch catch knobs | **only after catch-robustness Phase 0 restore**, then A/B ladders scored on **continuous seat proxies** (`ball_held_raw` flicker count in the 500 ms after contact, held-dwell-to-stable, `dip_below_x3`, peak `\|iq\|` through the seat, mocap post-contact excursion) at ~30/arm — not on the label at 196/arm (F4) |

**Pre-registered fallback** (CLAUDE.md's checkpoint rule): if per-node residuals
are not stable across the 3×3 capture and its verification pass, **ship the
measured home-node offset as a single global bias and stop.** Do not spiral into
a denser grid hunting a field that is not there.

**Pre-registered discriminators, written down before the first sitting** — these
decide where the *next* month of effort goes:

- **σ_L ≥ 18 mm** ⇒ the throw scatters ⇒ pursue variance reduction
  (catch-robustness Phase 0, settle ladder, release repeatability, Layer 1.5).
  The aim map is worth ~5–8 catch-rate points and no more.
- **σ_L ≤ 12 mm with catches still failing** ⇒ the *seat* is the fault ⇒ the
  catch-knob ladder is the right work, scored on continuous proxies.
- **σ_L > 30 mm** ⇒ **stop.** No calibration converges inside a sitting at that
  scatter; the fault is upstream and the loop would be fitting a broken plant.
- **R² of `land_err` on the dwell tilt read ≥ 0.4** ⇒ build Layer 1.5 as a
  feedforward *before* the trim loop; it is the bigger win and it has no loop to
  destabilise.

### Bench-time budget

Cadence at defaults: dwell 6.0 s ⇒ 10 cycles/min. A drop costs ~25 s all-in
(go_home 2.0 + verify, reload ladder 15–20, cleanup floor 2.80).

| Phase | SC-0+SC-1+3×3 (129 tosses) | 5×5 × 8 (200) |
|---|---|---|
| P0 desk | 40 min | 40 min |
| P1 power-up + arm | 10 | 10 |
| P2 preflight | 10 | 10 |
| P3 baseline (20) | 12 | 12 |
| P4 collection @ 75 % catch | ~22 min motion + ~16 min floor pauses ≈ **38** | ~36 + ~30 ≈ **66** |
| P4 collection @ 90 % catch | ≈ **26** | ≈ **38** |
| P5 verify (30) + analyse | 20 | 20 |
| **Total** | **~2 h 10 m** | **~2 h 40 m** |

(Both totals are the sum of their own column at the 75 % catch rate; the 90 %
row swaps in for the collection line and takes ~12 min off each.)

Two consequences worth stating plainly. **The catch rate is the budget driver,
not the grid size** — 75 % → 90 % nearly halves collection time, which is the
strongest argument for landing catch-robustness Phase 0 *before* the first
capture. And **a 5×5 first capture does not fit a comfortable sitting.**

### What the operator supervises

- **E-STOP in reach for the whole session.** A 129-toss capture is ~40 minutes of
  near-continuous motion. It looks unattended; it is not.
- **The floor.** Every drop leaves a ball under a machine about to stroke again.
  Watch the counter, honour the pause.
- **The ball supply.** There is no machine-side magazine fence; `max_reloads = 3`
  is the only automatic limit, and it fails the session closed.
- **The dwell must LOOK quiescent** except during a declared reload interlude.
  Platform motion during a dwell with no `RELOAD INTERLUDE` banner is S2 violated
  — stop. (The Layer 1.5 inclinometer reads command no motion.)
- **The hand at prime.** A reload primes the hand to the top of its stroke. A
  ball visibly in the cup at that moment means the `catch/prime_hold` ordering
  failed — stop, do not let it finish.
- **Announcement / dispatch lines during the interlude** — a `bb/throw_at_target`
  accept with no announcement is the BB fail-open signature.

### Live console — three deliberately separate streams

```
TOSS 07/72  node(+150,   0)  CAUGHT   | held @ +0.213 s | land (+11.4, -6.2) r=12.9 mm
            | flight 0.812 s (cmd 0.800, +12 ms) | uptime 00:14:22
            | applied: map aim(+0.035,-0.022)deg  trim OFF  k_v 1.000
            | dwell tilt (+0.412,-0.155)mrad sd(0.48,0.51) n=8   [COVARIATE]

NODE (+150,   0) DONE  8 tosses: 6 CAUGHT / 1 BOUNCED / 1 MISSED  (75.0% clean, 87.5% arrived)
  aim residual  mean (+9.8, -4.1) mm  sd (6.2, 5.4)  n=7
SESSION  41/56 clean (73.2%)  drops 15  reloads 3/3 BUDGET EXHAUSTED  floor 5/5 -> PAUSE

TRIM node(+150,0): aim (+0.031,-0.018)deg = (+1.7,-1.0) mm @h0.78  [clamp +-0.150 deg]
     n=6  se 0.041 deg  state ACTIVE  SESSION-ONLY
     (persistent map: (+0.035,-0.022) deg) — applied from cycle 9
```

`SESSION-ONLY` vs `PERSISTENT` in the string from day one — that is the
loaded-vs-applied distinction the tilt-cal review had to go back and fix in four
documents. `[COVARIATE]` on the dwell-tilt line for the same reason: a recorded
number that is never applied must say so on the line that prints it. Every line
carries `uptime_ms`; that is not decoration, it is the only defence against a
session whose late tosses are a different plant from its early ones.

### Rollback and escape hatches

| Hatch | Action | Latency |
|---|---|---|
| Disable trim | `ros2 param set /reload_coordinator_node toss_trim_enabled false` | next cycle (**never** mid-cycle — the trim is frozen at `_build_toss_cycle`) |
| Zero trim, keep learning | `--no-trim` on the tool / goal field | next cycle |
| Revert the map | `git checkout config/toss_calibration.yaml` + reload service | seconds |
| Roll back N versions | `git log config/toss_calibration.yaml`, checkout, reload | seconds |
| Uninstall entirely | `--force-uninstall` (**all** candidates — source tree *and* ament share; the tilt-cal review found a single-file version could not actually uninstall) + reload | seconds |
| Stop the session | cancel the goal | immediate between cycles; deferred if a ball is airborne — the tool prints which |
| Stop everything | E-STOP | immediate, latches until `clear_errors` |

---

## 7. The three riskiest failure modes

### R1 — A frame / plane / sign error produces a map that aims the machine *worse* than no map

Four silent routes: **fit plane ≠ cup plane** (`ball_arrival_offset.py` defaults
`--plane 1000 mm`; the cup plane is
`GEOM_INITIAL_HEIGHT_MM + active_z + HAND_CATCH_OFFSET_MM`) — for a vertical 8a
toss `v_xy ≈ 0`, so this looks perfect right up until a displaced throw, at which
point every node is biased in a direction correlated with displacement; **STOW vs
global** (the 2026-07-23 double-count class — the probe's own docstring warns a
70 mm-displaced catch scored against the default reference reads as a 70 mm
miss); **sign**; and **centroid vs cup** (`/trajectory/commanded_position`
publishes the centroid, not the cup).

Mitigations: `catch_point_global_mm` is computed by the production
`stow_to_global_mm` and *declared*, and the miner independently recomputes it and
**fails loud** on mismatch (one line, never averaged away); the miner **refuses**
a `--plane` differing from `catch_point_global_mm[2]` by more than 5 mm, and
`land_plane_mm` is in every row so a historical mismatch is detectable; SC-0
measures the sign rather than assuming it, as a blocking rung; the offline
closed-loop sign test (2c); and the doubled-bias single-node first application.

### R2 — Label corruption

`ball_held_valid == false` is UNKNOWN and must never collapse to a verdict (D13).
`ball_held_stamp` is wall-epoch **only** while `ball_held_valid` and after the
wall anchor lands — before that it is bridge-boot-relative, and joining it to
wall time mis-times every edge **by the whole bridge uptime**, the largest silent
error available anywhere in this pipeline. The debounce bakes ~100 ms into any
timing fit that uses it (D12). A rimshot produces > 2 edges and a naive
first-edge rule labels it CAUGHT with a wrong catch time.

Mitigations: verdict from debounced, times from raw, both recorded, plus the
*measured* `sensor_poll_dt_ms_median`; `ball_held_stamp_wall_anchored == false` ⇒
the miner falls back to the `/hand_telemetry` ROS receive time and marks
`timing_degraded`, refusing that record for the **timing fit only**;
`sensor_edge_count > 2` ⇒ rimshot candidate, retained for aim, excluded from
timing; and two-sided acceptance against ground truth that **already exists**
(the hand-mined counts from `2026-08-10_16-30-44`), shipped as a committed
fixture plus a bag-free `--self-check`.

### R3 — Corpus poisoning by an unrecorded plant change

Not hypothetical. Dispatch shift +118–133 ms at ~16 h uptime. The hand-ODrive
braking clamp (`iq_meas` −6.5…−11.3 A against ~26 A commanded, one-sided) made
the entire 8a retest unrepresentative, on a machine whose ODrives "occasionally
need config resets". A corpus pooling degraded and restored tosses yields a map
that is a weighted average of two machines — and it will look *fine*, because the
residual mean is still a number and the sd only widens a little.

Mitigations: mandatory partition keys (§ 3.7), crossing one needs an explicit
flag and is stamped into `captured`; `uptime_ms_at_release` first-class with the
within-session trend test (D16); a per-session **plant-drift panel** from the
miner (`uptime_ms` span, `iq_brake_min_a` distribution, `dip_below_x3_rev`
distribution, `dispatch_shift_ms` vs `uptime_ms` scatter); and `toss_cal_fit.py`
printing a REFUSE when the `iq_brake_min_a` median sits outside the post-restore
band — i.e. **the braking clamp is detected from the corpus itself, not
remembered from a logbook entry**. Every map write records the source file list
with per-file sha256, per-node `n`/`sd`/`date_range` and the corpus uptime span,
so a bad map is attributable and rollback-able node by node.

---

## 8. What is deliberately unpinned

Marked PROVISIONAL in code, in the tilt tool's convention, so the commit that
pins each is easy to find: `N_MIN` (8), `DEADBAND`
(0.10°), `TRIM_MAX` (0.15°), `STEP_MAX` (0.10°), `n₀` (4),
`--floor-pause-every` (5), the Layer 1.5 read schedule (N = 8 at 0.15 s), and the
entire `rimshot` discriminator (candidates: `t_catch_raw − t_land_mocap`
threshold, `sensor_edge_count > 2`, post-contact mocap excursion — REPORT-only
until validated against operator eye on one sitting; rim transit is order
30–80 ms = 1.5–4 samples at 50 Hz, i.e. *marginally* resolvable).

`max_reloads` (3) is **operator-set, not provisional** (decision 4, § 9).

`hand_odrive_config_sha` ships **nullable** and depends on the catch-robustness
Phase-0 open fork (launch-time SDO config assertion vs session-runbook probe). A
null is a *known gap* in the partition key, not an assumed match.

Whether the runbook's claim that the bag path cannot reproduce hand-telemetry
resolution still holds: the record carries `plant_block_source: trace|bag`
precisely so this becomes a measurement rather than an inherited assertion.

---

## 9. Operator decisions (2026-08-10)

These decisions were taken in session on 2026-08-10 and **replace the design's
open questions**. They are binding on the build.

| # | Decision | What it settles |
|---|---|---|
| 1 | **Catch-robustness Phase 0 (drive restore) runs before the first capture, and the operator runs the capture.** Everything in this build (2a–2f) is **desk-side** — no hardware needed or touched. | The design's question of whether ~2 h of sitting was worth the ~5–10 catch-rate points before Phase 0 lands (F6). Answer: restore first, then capture, and separate the desk build from the sitting entirely. |
| 2 | **Layer 1.5 APPROVED as a covariate**: ~1.2 s of dwell spent on N = 8 inclinometer reads at 0.15 s gap, recorded per toss. **Covariate ONLY — zero control authority.** Reads NEVER overlap PREPARE→THROW; if the dwell budget is tight, degrade the read count, never delay the throw. A follow-on is **REGISTERED, not built here**: restructure the Platform-Teensy SCL3300 read to timer-driven background sampling into a cache (the async-cache pattern the can-bridge hand-sensor poller uses), per the standing principle of RTOS-style determinism — no blocking I/O in control loops. | The design's question of whether dwell time is cheap enough to spend on a measurement that might return "no correlation" (D17). Answer: yes, as a pure covariate, with the blocking-read hazard fenced by schedule rules and the structural fix registered. § 3.10. |
| 3 | **Cadence stays 6.0 s. The 4.10 s fork is NOT built.** | The throughput-vs-margin fork. The 3.5 s floor stays untouched and no cycle is pushed to the edge its own floor exists to protect. |
| 4 | **BB ball supply is operator-managed — build NO magazine fence.** `max_reloads = 3`. On budget exhaustion the SESSION fails closed (`STOPPED_RELOAD_BUDGET`, unchanged doctrine), and the 2f capture tool treats that terminal as "node exhausted": mark the node thin/stale, **SKIP to the next node, continue the capture**. The known BB-side defect where a reload throw ABORTs because BB is not positioned in time is **retryable within the budget**; the retry must be targeted at that identifiable code and must name it. | The design's question about real magazine capacity and machine observability. Answer: there is none, so no fiction is built; the budget is the only fence and a node's exhaustion costs a node, not a sitting. § 3.8, § 3.9, D19. |
| 5 | **Build everything before any hardware test.** Catch-robustness **Phase 1 HAS LANDED** (`toss_require_ball_evidence: true` live; commits `9caf6bc` / `2716e3b` / `6920e88` / `60f0a86`), so 2d's prerequisite is satisfied — **consume the shipped sensor merge, do not re-implement it**. | The design's contingency about auto-reload refusing to arm if Phase 1 had not landed. Answer: moot as a build ordering question; the runtime refusal on a live `false` config value stays. § 3.9. |
| 6 | **`ABORTED_NO_RELEASE` retry REOPENED.** Rationale, in the operator's words: *"we can trust the hand sensor, especially over 5 reads."* At that terminal, retry the cycle **ONCE iff the sensor reads valid-HELD** — the ball is demonstrably in the cup, so the D9 airborne-ball hazard is structurally absent. **UNKNOWN or EMPTY ⇒ no retry, stop as today. TWO consecutive `ABORTED_NO_RELEASE` ⇒ stop the session** (the epidemic gauge is preserved). | D9, which deferred the retry out of v1. § 3.9 abort ladder, guard G11, phase 2d. |
| 7 | **Map ownership: `reload_coordinator_node` at `_build_toss_cycle`** — the map rewrites a GOAL, not poses (the tilt map lives in `trajectory_node` because it rewrites poses at ingest). **Storage: machine-written versioned YAML at `config/toss_calibration.yaml`** per § 3.7; the corpus stays JSONL under `temp/`. | The design's ownership fork. One map, one owner, one apply point each, at the cost of two reload services and two status field pairs. |
| 8 | **The angular-origin framing is CONFIRMED** (F1/D1), as is reload-from-any-node-once-verifiably-home (D8). **SC-0 and SC-1 stay blocking gates regardless.** | The physical-intuition check the design asked for. The framing is endorsed *and* the measurement that could falsify it stays mandatory — an endorsed hypothesis is still a hypothesis. |

**Standing invitation, restated at the sitting.** If the operator's physical
intuition disagrees with any framing here — particularly the claim that the
residual aim error is *angular* in origin, or that a reload interlude from a
corner node is safe once the platform has verifiably gone home — that is
load-bearing signal, not friction. SC-1 exists precisely because the first claim
might be wrong.

---

## 10. Open

### Raised by the 2a build against real bags (2026-08-10)

Three of these are design assumptions in this document that the reference bag
contradicted. See `logbook/2026-08-10-toss-selftuning-build.md` § Phase 2a for
the measurements.

- **§ 3.3's catch-search window is WRONG and has been superseded in code.** The
  draft `[landing − 0.30, landing + 0.70]` uses `CATCH_CONFIRM_WINDOW_S`, which
  is the FSM's terminal *deadline*, not a sensor window. Scored on
  `2026-08-10_16-30-44` it relabels the **+798 ms** arrival MISSED — the
  population maximum, and the very row `JB_BD_ARRIVAL_WINDOW_S` was sized on.
  `toss_record.label_from_sensor` uses the shipped `JB_BD_ARRIVAL_*` windows,
  injected, so the offline label and the live `HandBallSensorSource` agree by
  construction. **Read § 3.3's window as `[landing − JB_BD_ARRIVAL_LEAD_S,
  landing + JB_BD_ARRIVAL_WINDOW_S]`.**
- ~~**§ 3.7 item 5's timing-fit gate is unsatisfiable on this plant.**~~
  **CLOSED by 2c (2026-08-11).** Re-derived from the measured distribution as a
  *precision* gate, `[20 ms, 200 ms]` plus the `se ≤ 5 ms` apply bar — see § 3.7
  item 5 and the logbook's § Phase 2c. The finding that decided it: the measured
  departure-shift sd (20.51 ms) **is** the poll quantisation `Δ/√12` (20.50 ms),
  ratio 1.001, so the release timing is more repeatable than the instrument can
  see. The underlying 3.5x poll-cadence gap still has no diagnosis and is still a
  can-bridge question, not this plan's.
- **D12's ~100 ms debounce estimate is low by 2.4x, and the lag is asymmetric.**
  Measured: `empty→held` 0/0/0 ms, `held→empty` 232/**241**/295 ms — consistent
  with the documented five-miss-to-drop / one-hit-to-restore firmware rule at the
  measured 71 ms poll. 241 ms is *larger* than the +118–133 ms uptime dispatch
  shift the fresh-boot discipline exists to control, which strengthens D12
  rather than weakening it.
- **Ball visibility in the descending band is an unchecked capture
  precondition, and the reference sitting fails it.** In ±1.5 s of a landing that
  bag carries four unlabelled markers — three static rig markers outside the
  ±300 mm lateral gate and one slow-drifting platform marker below the cup plane.
  The shipped `ball_arrival_offset.py` independently reports **`NO TRACK` on all
  31**, so `usable_for_aim_fit` is 0/31 and the sitting cannot support an aim fit
  at *any* record-list completeness. The ball IS tracked once it reaches the floor
  (the probe's floor census fires after five of the misses), so this is a
  coverage gap over the cup, not a labelling problem. D18 fixes the topic list; it
  does not fix this. Proposed P2 preflight row: one throw, one
  `toss_record_miner.py` run, require a non-null `land_xy_global_mm` before
  spending a sitting.
- **The PLANT block ships null in 2a, with its mapping already worked out.**
  § 3.4 requires the row builder to be IMPORTED, not re-implemented, and wiring
  `hand_stroke_timeline` needs its session-relative clock mapped back onto the
  record's ROS instants — a mapping error there writes plausible WRONG plant
  numbers into the corpus, and 2a has nothing to validate the result against.
  The field mapping, so no work is lost:
  `stroke_peak_rev` ← `ThrowTimeline.peak_pos_rev`;
  `dip_below_x3_rev` ← `.dip_below_x3_rev`;
  `pullback_rps` ← `.pullback_vel_rev_s`;
  `trunc` ← `.trunc is not None`;
  `seeds` ← `.n_seeds`;
  `dispatch_shift_ms` ← `.shift_ms`.
  **`iq_brake_min_a` has NO shipped builder** — `HandSample` does not carry
  `iq_meas` — so it needs both a new computation and a pinned braking window,
  which must be measured before it is written down. Until the block lands, guard
  G4 (plant health) cannot be enforced from a record, which `_mark_usable`
  already states rather than pretending otherwise.
- **`floor_arrival` ships null in 2a.** The floor census lives in
  `ball_arrival_offset.read_bag`, which the miner does not reuse (it imports the
  estimator, not the reader). Small follow-on; the sitting above is the argument
  for it.
- **`uptime_ms_at_release` ships MINED-only in 2a** (the schema marks it `D+M`).
  The coordinator does not subscribe to `/link_status`, and adding a subscription
  to the node that owns the hand, the latch and the abort ladder for a pure
  covariate spends the surface D10 argues to protect; the bag carries the topic
  at 5 Hz, so the miner recovers it to ~200 ms. If a later phase wants it
  declared, argue it on its own merits.

### Raised by the 2c build (2026-08-11)

- **G4 (plant health) is still unenforceable from a record, so § 7 R3's
  braking-clamp REFUSE is NOT implemented.** `iq_brake_min_a` needs the PLANT
  block, which ships null (2a). `toss_fit_lib.admit_for_aim` names the guards it
  does enforce and does not pretend to this one; the R3 mitigation
  *"`toss_cal_fit.py` printing a REFUSE when the `iq_brake_min_a` median sits
  outside the post-restore band"* is therefore still remembered rather than
  structural. Wiring the PLANT block closes it.
- **D16's automatic refusal is reported, not enforced.** The residual-vs-uptime
  trend and the anchor peak-to-peak are printed by both tools and carried in
  `--json`, but *"the analyser refuses a timing fit whose within-session trend
  exceeds the between-node signal"* is currently an operator's eye on two
  numbers. Small follow-on; belongs with the first corpus whose uptime span is
  hours rather than minutes (the analyser withholds the per-hour slope below a
  0.25 h span for exactly that reason).
- **A never-measured node with no previous value refuses the write** — D15 covers
  the *thin* node (keeps its previous value, `stale: true`) but is silent on the
  node that has never flown. Shipping a zero there is not neutral: the bilinear
  blend would drag its measured neighbours toward zero across half a cell. The
  ball-actually-flew guard names the nodes and refuses; if a future capture wants
  a partial grid, this is the rule to revisit deliberately.
- **`--group` A/B scoring is exposed on BOTH tools** over one implementation
  (`toss_fit_lib.score_groups`). § 3.7 item 8 put it on the fit; the analyser is
  where an operator will look.

### Raised by the 2d build (2026-08-11)

- **§ 3.10's read budget does not fit the quiescent dwell, and the arithmetic is
  the finding.** The design asks for N = 8 reads at 0.15 s (~1.2 s) "of a 6.0 s
  dwell". The *quiescent* dwell — the only place § 3.10 permits a read — is
  `dwell_time_s − throw_delay_s − CAUGHT-verdict latency`, because cycle N+1
  STARTS at `landing + dwell − throw_delay` and everything after that instant is
  the next cycle's own countdown, PREPARE and THROW included. At the shipped
  defaults that is **~0.7 s, not 6.0 s**, so 8 reads do not fit and 1–2 do. The
  build ships the schedule with the degrade rule doing exactly what it was
  written for (`dwell_tilt_degraded` true, `dwell_tilt_n` 1–2, a legal record),
  and the lever is the goal's own `dwell_time_s ≈ 7.5 s+`. **Consequence for
  D17**: *"is arrival repeatability the dominant σ term?"* cannot be answered
  from a 6.0 s-cadence corpus. Reading during cycle N+1's CHECKING/POSITIONING
  was rejected — it would convert rule 1 from a STRUCTURAL guarantee (the cycle
  blocks the session loop) into a conditional one, on a service that blocks the
  Platform-Teensy loop streaming hand moves.
- **`THROW_ABORTED_NOT_SETTLED` was identifiable but not OBSERVABLE**, so § 3.9's
  either/or ("ship the targeted retry, or ship nothing") had a third option: make
  it observable. `bb/throw_at_target` is fire-and-forget and the firmware's
  terminal `CMD_RESULT` was log-only, so `ball_butler_node` now relays it on
  `bb/throw_outcome`. Best-effort telemetry — a dropped message costs a retry
  (the session stops by name), never a safety property. If the retry proves
  load-bearing on hardware it wants a typed message and a sequence number.
- **`STOPPED_FLOOR_CLEAR_REQUIRED` is unreachable at the shipped defaults**
  (`max_reloads` 3 binds before `floor_pause_every` 5). It is the fence for a
  deliberately long-budget session; the § 6 D-f decision row should say so.
- **The interlude's reload runs at the reload FSM's OWN throw-delay default**,
  not the session's `throw_delay_s` — deliberate (one is a toss parameter, the
  other a BB countdown floor), but it means a session's cadence and its
  interlude's are set by different numbers. Worth an operator's eye on the first
  sitting, alongside **the interlude's real wall-clock cost**, which is currently
  derived from constants (`_reload_interlude_budget_s`) and never measured.

### Raised by the 2e build (2026-08-11)

- **The session trim has NO live measurement channel, and neither candidate is
  admissible.** § 3.6's measurand is `land_err_mm`, whose schema origin is **M —
  mined**: it comes from the offline mocap descending-branch estimator via
  `toss_record_miner.py`. `reload_coordinator_node` has no producer of it, and
  the two things it could be wired to are both refused on their merits —
  `TossResult.catch_error_mm` is D5's named-forbidden observable (dead-reckoned,
  a scalar *distance* not a signed 2-vector, and defined only for CAUGHT balls,
  i.e. biased toward the cup) and `BallState.landing_position` is the same
  tracker Kalman filter's predicted plane crossing, one message earlier. The
  build therefore ships the seam and refuses the measurement **by name**: a live
  record fails admission at `no_mocap_fit`, the trim stays `WARMUP` and commands
  exactly `(0, 0)`. `toss_trim.replay(records)` closes the loop offline today.
  Closing it live needs either a live arrival estimator in the node or an
  operator-driven replay of a mined corpus into a session — **registered, not
  built**, and it is the decision that governs whether the trim is ever more
  than an instrument.
- **At the common mode the trim was SIZED on, it is a WASH — and σ has never
  been measured on this machine.** Measured expected residual aim error
  `E|δ − b|` over 300 sessions × 72 tosses at σ = 20 mm/axis: at the `level`
  common mode (0.069–0.097° = 3.8–5.3 mm) the trim returns 3.79 mm against
  3.82 mm with no trim, and at **zero** bias it COSTS ~1.03 mm. It only starts
  paying above ~0.12°. The whole value proposition turns on σ, the per-toss
  arrival scatter, and the reference sitting reports `NO TRACK` on all 31
  descending branches (2a) — so 20 mm is the design's working figure, not a
  measurement. **First question for the first corpus.** If σ really is 20 mm the
  honest recommendation is *fit the map, leave the trim off*.
- **`FROZEN_STALLED` on a real sub-clamp bias is EXPECTED at σ = 20 mm** — 1/20
  synthetic sessions at a 0.12° bias, because the estimate wanders past the
  0.15° clamp on its way. An operator meeting the STALLED banner on the first
  sitting needs to know that before they go hunting a braking clamp. The § 6
  console/decision rows should say so.
- **The `TRIM` console line cannot carry can-bridge uptime.** § 6 asks for
  `uptime_ms` on every line; this node does not subscribe to `/link_status` and
  2a's argument against adding that subscription to the node that owns the hand,
  the latch and the abort ladder still stands. The line says `uptime UNMEASURED`
  in as many words and carries goal-elapsed seconds instead; the bag carries the
  topic at 5 Hz for the offline join. Revisit only with a reason of its own.
- **Four of § 3.6's constants were amended against probes, and § 3.6's constants
  table should be read with `toss_trim.py`'s beside it**: `SE_GATE` 2.5 with a
  3-update confirmation at n ≥ 6 (the design's `n ≥ 3 ∧ |r| ≥ 2·se` commanded a
  trim in **45.7 %** of zero-bias sessions — a sequential multiple-comparison
  problem the one-shot arithmetic cannot see); G8's `3·se` re-derived as a
  tabular CUSUM `k = 0.5, h = 8.0` (2 % false alarm, 99.7 % detection of a 2σ
  shift within 6 tosses) because 3·se at n = 16 is 0.75σ and undetectable; the
  DEADBAND on the estimate rather than the step; and both authority clamps as
  magnitudes rather than the design's literal per-axis boxes.

### Raised by the 2f build (2026-08-11)

- **Three of § 3.8's four gates, taken literally, refuse a HEALTHY machine most
  of the time — and the arithmetic is the finding.** They are point comparisons
  against thresholds whose standard error the design never computed. Measured
  (`/tmp/probe_toss_sc_gates.py`, 2026-08-11, 20 000 trials per cell, plant
  obeying the production model exactly, at the design's own working σ = 20
  mm/axis): the literal **SC-0** gate passes a perfect plant **32.6 %** of the
  time (its ±25 % band is 1.08 se wide at n = 5, δ = 0.5°), and the literal
  **SC-3** gate passes a perfect map **8.0 %** of the time (per-pose se 8.94 mm
  against a 10 mm bound). Raising `n` to fix them costs 90 + 96 = 186 tosses —
  more than the whole 129-toss first capture — and raising δ to 1.0° displaces the
  landing 54.6 mm against an 80 mm reach envelope, so most probe tosses would
  miss. **Shipped instead: every gate is three-valued** (FAIL only when the
  `2·se` interval EXCLUDES the bound, PASS only when it is wholly inside,
  INCONCLUSIVE otherwise, with the resolving `n` printed). Nothing the gates
  exist to catch is weakened — a sign flip leaks **0 of 120 000** trials under
  both forms. Read § 3.8's gate table with `toss_cal_grid.py`'s beside it.
- **SC-1 cannot decide the map's UNITS at any sample size that fits a sitting.**
  At n = 8, σ = 20 and the design's own inferred ψ ≈ 0.37°, the exponent's 95 %
  CI has a **median width of 4.47** against a 0.6-wide rad band and lands inside
  a branch **0.1 %** of the time; n = 32 at ψ = 1.0° reaches 13.6 %. The branch
  table's escape hatch (*"CI spanning two branches ⇒ working height only"*) is
  therefore the **expected** result, not the exception, and the tool says so
  before the 32 tosses are spent. The rung keeps its place because its OTHER
  product — the **first honest `σ_L` on this machine** — is the number three
  gates' resolving power and 2e's whole trim-value question depend on. A
  different experiment, not more tosses, is what would settle the exponent.
- **The `working_height_only` branch has NO consumer.** `toss_cal.parse_toss_cal`
  accepts only `units.aim: rad` (deliberately), and `toss_fit_lib` writes
  `height_scaling_exponent: 1.0` with `h_capture_m: null` — so nothing warns when
  a map captured at h = 0.78 m is applied at another `throw_height_m`. Wiring
  `h_capture_m` at write time plus a WARN at apply is a small follow-on and
  should land with the first capture that returns the ambiguous branch, which per
  the row above is nearly all of them.
- **§ 3.8's SC-0 accept test is written for a scaled-identity `S`, and `S` is a
  90° rotation** (2c measured it: `J = [[0, 3126.5], [−3126.5, 0]]`). In the raw
  `(Lx,Ly)×(rx,ry)` basis the design's "diagonal" is the ZERO entry and its
  "off-diagonal" is the gain, so a tool applying the words literally would refuse
  every healthy capture and pass a transposed one. The gate is applied to
  `D = J_pred⁻¹·J_meas`, which means what § 3.8 means and reduces to it exactly
  when `J_pred` is a scaled identity.
- **A capture cannot score itself, and this is the same gap 2e reported.**
  `land_err_mm` is MINED, so § 3.8's rungs are captured in one invocation and
  scored in another (`--score <corpus>`), with a ledger between them carrying the
  blocking preconditions. Closing it live needs the same live arrival estimator
  the session trim needs; if that ever lands, both close together.
- **§ 3.8's "≥6 off-node check poses" and "3×3 first capture" are unsatisfiable
  from cell centres alone** — a 3×3 has 2×2 = 4 interior cells. The tool extends
  the tilt tool's centres with **edge midpoints** (off-node, inside the hull,
  exercising the blend along one axis). Worth a line in § 3.8 if the check-pose
  count is ever revisited.
- **§ 5 P5.4's doubled-bias wording is loose.** With the correct sign the error
  over bias ×0/×1/×2 runs `|b| → ~0 → |b| reversed`; with the wrong sign it runs
  `|b| → 2|b| → 3|b|`. The discriminator is the DIRECTION, not a doubling — which
  is exactly what SC-0 measures, with a gate on it, so P5.4 *is* SC-0 re-run at
  the fitted magnitude. The tool's printed run plan says this.
- **`trajectory/go_home` is in the tool's manifest, and it is a motion command.**
  § 3.8 says safing is reached by cancelling the goal, and that is what happens
  mid-capture — but a CAUGHT cycle ends in `ACTION_STAY`, so after a corner node
  the routine end state is "parked 150 mm off centre, raised", with no goal left
  to cancel. `go_home` is the coordinator's own safing Trigger, arms nothing,
  changes no mode, and the tool verifies ARRIVAL rather than trusting the ack.

### Carried from the design

- **SCL3300 async-read firmware follow-on (REGISTERED, not built here).**
  Restructure the Platform-Teensy `get_platform_tilt` path to timer-driven
  background sampling into a cache, so a tilt read never blocks the loop that
  streams hand moves. Pattern: the can-bridge hand-sensor poller. Driver:
  RTOS-style determinism — no blocking I/O in control loops. Until it lands,
  Layer 1.5's dwell-only schedule and degrade-never-delay rule are the fence
  (§ 3.10). Also carried in [catch-robustness.md](catch-robustness.md)'s Open
  row.
- **Layer 1.5 promotion to feedforward** — gated on `R²` of `land_err` on the
  dwell tilt ≥ 0.4, measured from the first corpus. Separate decision, separate
  build.
- **`hand_odrive_config_sha`** stays nullable until the catch-robustness Phase-0
  config-drift fork is chosen (launch-time SDO assertion vs session-runbook
  probe). A null partition key is a known gap, not an assumed match.
- **BB not-positioned-in-time abort code** — the targeted retry in § 3.9 needs a
  distinct, identifiable code in the BB/reload path. If none exists, the retry is
  not shipped and the session stops as today, naming the ambiguity.
- **Bridge-uptime lag root cause** — owned by
  [refactor-2026-07.md](refactor-2026-07.md) Phase 7. Until it closes, the
  fresh-boot discipline and the session-local `τ` trim are the mitigations.
