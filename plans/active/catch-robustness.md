---
title: Catch robustness — sensor-truth verdicts, hand-drive restoration, toss self-tuning
created: 2026-08-10
status: active
related_logbook:
  - 2026-08-10-hand-drive-braking-clamp-diagnosis.md
  - 2026-08-21-ilc-primary-foldin.md
related_code:
  - ros_ws/src/jugglebot/jugglebot/ball_possession.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - tools/probes/hand_stroke_timeline.py
  - plans/archived/hand-ball-sensor.md
  - plans/active/single-ball-toss.md
  - plans/archived/hand-command-continuity.md
---

# Plan — Catch robustness programme

**Branch:** `mvp-trajectory-bringup`
**Goal:** highly repeatable single-ball tosses across most of the workspace at
z = 170 mm — catch success approaching 100 %, with the hand ball sensor as the
ground truth for whether (and when) a ball was caught.
**Owner decisions (2026-08-10, in-session):** hand sensor becomes the possession
source of truth AND the ball-evidence gate (`toss_require_ball_evidence` → true);
learning shape = (c) persistent calibration map updated only by an explicit
calibration routine, plus a session-local real-time trim layered on it;
`TossContinuous` should auto-reload from BB on drops. The hand-ball-sensor
plan's "flip forbidden until Phase 7 validates" gate is SUPERSEDED by this
decision — the sensor is operator-validated in situ (Phase 7 steps 4–5 remain
open as bench work, not blockers).

## Context — the 2026-08-10 diagnosis

The 8a retest (tilt map live, catch_vel_scale 0.9) caught most throws but showed
a returned post-throw dip on every 1.0 m throw, end-stop bumps, and ~60 %
catch rate at 1.0 m. Offline diagnosis from the three retest bags
(16-04-26 / 16-13-48 / 16-30-44, ~16 h can-bridge uptime):

- **The dip is NOT the 2026-07-25 clobber class returning.** 54/54 throws at
  4.44 m/s show `trunc = -`, `seeds = 0` — the Phase-1 arm gate holds. The
  FW-2 decel FF **is** commanded (`tor_ff_cmd` −0.1500 Nm = `throwDecelToTorque`
  at this tier; legacy would read −0.11).
- **The drive under-delivers braking**: over the 54 decel ramps `iq_meas` reads
  −10.52/−7.60/−3.85 A (min/med/max) against the **−27.2 A** the commanded
  −0.1500 N·m asks for at the hand's Kt 0.0055133, and the whole-bag negative
  floor is −11.8 A while the positive side runs to +49.6 A. One-sided, with a
  hard floor ⇒ drive-side braking limit on the hand ODrive — swapped during the
  2026-07-31 CAN3 elimination, on a machine whose ODrives occasionally need
  config resets. **The leading candidate is named and pre-registered**:
  `axis0.config.torque_soft_min = −0.055133 N·m` = **exactly −10.00 A**
  (asymmetric against `torque_soft_max` +0.5 N·m) — C-HAND-2 § *The negative
  torque clamp*, bench pre-flight **H7.0c**. Its 2026-07-27 counter-evidence
  was measured on the *pre-swap* drive and does not transfer. Regen
  (`dc_max_negative_current`) and a reset `torque_constant` stay in the class
  until the dump rules them out.
- Consequences: coast +0.51…+0.81 rev past x3, position-loop pullback +
  FW-2 braking prelude ⇒ dip below x3 0.27–1.67 rev (median 0.90) on 54/54;
  peaks to 10.766 rev. **The same clamp plausibly degrades the catch seat
  (also regen-direction) — catch knobs must not be tuned against this plant.**
- Dispatch shift +118–133 ms at ~16 h uptime (fresh-boot discipline stands).
- The sensor ledger in the bags is 100 % valid; bag 16-30-44 alone yields
  39 departures / 38 catches / 3 quick-drops (<1.5 s) — automatic bounce-out
  labels, mined offline before any wiring existed.

## 2026-08-18 — the clamp was live, and removing it removed the dip

Operator bench session (ROS2, `torque_soft_min` ladder on the hand axis). **H7.0c's
pre-registered hypothesis is CONFIRMED**: the asymmetric clamp was live on the
flashed drive and was truncating the FW-2 decel feedforward. Flashed and locked in
`config/ODrive config Files/odrive_pro_hand_config.json`:

| | before | after |
|---|---|---|
| `axis0.config.torque_soft_max` | +0.5 N·m (+90.7 A) | **+0.7 N·m (+127 A)** |
| `axis0.config.torque_soft_min` | −0.055133 N·m (**−10.00 A**) | **−0.7 N·m (−127 A)** |

Result: the post-throw dip at the peak is **essentially gone** (operator, by eye) —
the symptom that 54/54 throws showed on 2026-08-10. The clamp is now symmetric and
outside `current_soft_max` = 50.0 A, so **the 50 A current limit is the fence on
both sides** and the torque clamp truncates nothing. Regen does not become the new binding
constraint: at the 50 A `current_soft_max` fence the peak braking mechanical power
is **~250–270 W** at the top tier (0.2757 N·m × 965 rad/s), against the ~400 W that
`dc_max_negative_current` = −8.0 A allows on the ~50 V bus — about 1.5× margin, not
the ~8× an earlier draft of this paragraph implied by computing the power the *old*
−10 A clamp allowed (~50 W), which was circular.

Likely origin of the bad value: `−0.055133` is the *legs'* `torque_constant`
(8.27/Kv), and the hand's Kt is ten times smaller — a paste of the wrong constant
into a clamp field, which is exactly why it landed on a round −10.00 A.

**Three consequences, all now live:**

1. **The Open-row trigger has FIRED.** C-HAND-2's decel-side bound
   `J ≥ 1.0126e-5 kg·m²` was measured *through* the clamp, so it is not a property
   of the plant and `throw_decel_reflected_inertia_kgm2` must be re-derived on the
   restored drive. Discriminator, unchanged: does achieved decel become
   tier-independent?
2. **Every capture dated before 2026-08-18 has clamp-limited braking `iq`.** No
   inertia or authority number mined from those bags transfers. `tools/probes/
   hand_decel_authority.py` now says so in its own output, and its old "very
   probably NOT binding" argument (a_ach grows 2.6× across the band) is recorded
   there as **wrong** so it is not resurrected.
3. **The catch-knob freeze can lift on the clamp count.** The regen-direction
   degradation that made the 60 % baseline untrustworthy is gone.

**Phase 0 is NOT fully closed by this.** Still outstanding: the whole-config dump +
diff (`dc_max_negative_current`, `torque_constant`, `gpio2_mode`), the HAND-7 R0–R5
decel ladder, and the 1.0 m retest against the *quantitative* gate
(`dip_below_x3 ≤ 0.10 rev`, `peak ≤ 10.060 rev`, braking `iq` tracking commanded).
"Dip is gone by eye" is the right qualitative call and is why Phase 1 is unblocked;
it is not the measured gate, and the re-derivation in (1) needs the ladder's numbers.

## 2026-08-21 — the programme's learning half becomes ILC

Owner decision, 2026-08-21 (`logbook/2026-08-21-ilc-primary-foldin.md`):
**[critical-point-ilc.md](critical-point-ilc.md) is THE primary toss learning
architecture.** What that does to this programme:

- **Phase 2's substrate is untouched and is exactly what ILC consumes** — the
  `toss_record/1` schema, the miner, the labeller, the G1–G11 admission core,
  the CUSUM/freeze machinery, `TossContinuous` auto-reload, the Layer-1.5
  covariate, `toss_cal.clamp_total_aim` as the single D7 enforcement point.
  None of it was wasted and none of it is being rewritten.
- **Phase 2's *update laws* retire**: the aim map's per-node fit, SC-0…SC-3 and
  the `toss_cal_grid` acquisition campaign. Root cause, not preference:
  `config/toss_calibration.yaml` **has never existed in either tree**, so the
  map this programme designed was never captured, and the campaign that would
  capture it costs ~2 h 10 m of supervised bench time to produce a 2-DOF special
  case of a correction ILC already extracts from ordinary session bags. The
  session trim's **aim** estimator demotes to **monitor-only** (zero authority,
  still logging so divergence stays visible); its `speed_gain` retires into
  ILC's `event_vel_trim`; its release-latency `τ` stays trim-side and unwired.
  Supersession map: [toss-selftuning.md](toss-selftuning.md) § SUPERSESSION
  NOTICE.
- **Phase 3's baseline changes.** "Catch-rate baseline on the restored plant"
  stands and is now doubly load-bearing: it is *also* the A/B baseline for ILC,
  which is a **no-correction** baseline (no map exists, trim aim is monitor-only,
  `$JUGGLEBOT_TOSS_ILC=0`). Criteria are frozen from that capture's own scatter
  before the first treatment run — `critical-point-ilc.md` § Phase 3.
- **Catch-side learning stays gated on evidence, not enthusiasm.** ILC's
  `catch_timing_offset` lands as the first catch channel **only if** the
  `_arm_hand_catch` `event_delay` seam and the sensor catch-event residual make
  it a clean, gated addition; otherwise it stays declared-unimplemented with the
  residual definition written down. Probe 0b's outcome (ii) — the contact
  transient is **not** resolvable (`vel_meas` 1.03×, iq impulse 1.45× against a
  matched cross-label control) — is why, and that verdict is itself due a
  re-check now that the clamp is gone (every 0b bag was recorded through it).
- **Cadence**: the shipped 6.0 s dwell is replaced by the R0→R5 ladder toward
  **R5-prime** — re-taken and accepted by the operator 2026-08-23 as **dwell
  0.66 s / `throw_delay` 0.44 s at `T = 0.5029 s`, 51.6 throws/min with or
  without an armed aim** (the ~61 throws/min this line asked for is not reachable
  on this build; the frontier is 54.3). See
  [toss-selftuning.md](toss-selftuning.md) § 11 and
  `tests/hardware/session_cadence_ladder.md` § 2.0. ⚠ The dangerous change is not
  lowering `MIN_TOSS_THROW_DELAY_S`; it is lowering the dwell **without** the
  possession-semantics work (§ 11.4), which would leave a fail-open
  `ball_seated` gate, mislabel every good cycle `BOUNCED`, and route good cycles
  into the auto-reload interlude — asking BB to throw a second ball at a full
  cup.

## Phases

| Phase | Scope | Gate | Status |
|---|---|---|---|
| 0 | **Bench (operator): hand-drive restoration.** **H7.0c first** — read `axis0.config.torque_soft_min` off the LIVE hand axis (30 s, `odrivetool`; PASS is `≤ −0.20 N·m`, anything above that is the finding); then dump + diff the whole hand-ODrive config vs `config/ODrive config Files/odrive_pro_hand_config.json` (regen / `dc_max_negative_current`, `torque_constant`, `gpio2_mode`); restore; fresh can-bridge boot; HAND-7 R0–R5 decel ladder; 1.0 m retest. *Opportunistic add-on if the sitting has slack*: the hand-sensor bench-tuning steps re-homed from the archived hand-ball-sensor plan (stroke-jitter with polling on/off; ball soak to size `max_missing_samples`/poll rate — `tests/hardware/session_hand_ball_sensor.md` **runbook steps 5–6** (= the plan’s Phase 7 steps 4–5; runbook step 4 is the SDO cadence measurement), tuning not blockers) | dip_below_x3 ≤ 0.10 rev; peak ≤ 10.060 rev; braking iq tracks commanded | **CLOSED 2026-08-18 (owner decision).** `torque_soft_min/max` now ±0.7 N·m, `save_configuration()` run, dip gone by eye. The owner **declined the remaining characterisation** (whole-config diff, HAND-7 R0–R5 ladder, 1.0 m retest) — the symptom this phase existed to remove is gone and further bench time was judged not worth it. **What that costs, stated so no future reader mistakes it for a scored pass:** the quantitative gate (`dip_below_x3 ≤ 0.10 rev`, `peak ≤ 10.060 rev`, braking `iq` tracking commanded) was never measured on the restored drive, and the config-drift class that produced the clamp is unswept — see the Open row |
| 1 | **CLOSED 2026-08-18 (operator attestation).** **Sensor-truth possession + ball-evidence gate.** `HandBallSensorSource` behind the C-POSSESS-1 seam (tri-state honest; UNKNOWN never collapses to True); sensor-primary verdicts on toss + reload; catch-event time surfaced; `toss_require_ball_evidence` → true (UNKNOWN refuses, loudly); bag-replay validation vs the three 2026-08-10 bags | full gate + bag-replay reproduces the mined labels | **SOFTWARE COMPLETE + AUDITED 2026-08-10** — contract extended (C-POSSESS-1 §§ 2.1/3.2/3.3), `HandBallSensorSource` + `merge_possession` landed, gate flipped, `tools/probes/hand_sensor_verdict_replay.py` reconciles EXACTLY with the independent transition counts on all three bags (35 CAUGHT / 46 MISSED / 0 UNKNOWN over 81 announcements). `logbook/2026-08-10-sensor-truth-possession.md`. **Audit fixes landed same day** (entry § Audit fixes): the operator scoring rows this phase inverted are rewritten (runbook `POSS-1`/`POSS-1.3` had a reload `CAUGHT` — the headline capability — in their ABORT column), `describe()` stops attributing a SENSOR veto to the tracker, and the retention window gains the absolute widen guard it shipped without. **Three items carried to the bench, unfixed on purpose**: (a) the reload's `ACTION_RECENTER` terminal executes on hardware for the first time — ungated REPORT row `POSS-1.7`, gating it is an operator call; (b) `stale_s`'s stated justification vs the real 3.0 s bridge bound (design fork); (c) `_ball_possession` is write-only state, so § 3.3 edit 2 has no consumer. **Bench validation of the UNKNOWN paths is outstanding** — 203,922 real samples were 100 % valid, so every blind/stale path is test-only (row `POSS-1.8`: kill the SDO poller, not the link). **CLOSURE, 2026-08-18:** the operator has run the hand ball sensor through extensive testing across the preceding days and judges it **extremely reliable**; over the same period many reloads and self-tosses were run, with occasional drops but nothing of concern. On that basis the sensor is accepted as the possession source of truth and **§ SECTION POSS / Stage 6 CAP-WORK are hereby declared complete in this plan, without a scored capture** (the runbook rows themselves are left in place, unmarked). The evidence is operator attestation, not the runbook's row-by-row scoring — `POSS-1.7` (the reload `ACTION_RECENTER` terminal) and `POSS-1.8` (the blind-sensor paths) therefore remain **unexercised against their written criteria**, and those runbook rows stay in place for whoever wants them later |
| 2 | **Toss self-tuning loop (design → operator review → build).** Per-toss record schema; session-local trim + persistent calibration map (explicit calibration routine only); TossContinuous auto-reload-on-drop; catch-knob A/B ladders scored by sensor labels | design reviewed by operator before build | **SOFTWARE COMPLETE + INDEPENDENTLY VALIDATED + AUDITED 2026-08-11; the first hardware capture is next (after Phase 0).** Build 2a–2f all landed 2026-08-11 — write-up `logbook/2026-08-10-toss-selftuning-build.md`, per-phase status in [toss-selftuning.md](toss-selftuning.md) § 5. **Audit returned NOT CLEAN: eleven findings (2 HIGH / 6 MEDIUM / 3 LOW), of which SEVEN landed as fixes the same day** with named regression tests (the `ABORTED_NO_RELEASE` retry's missing settle floor, which also silently broke the two-consecutive-non-release stop; a DORMANT map seeding the session trim's prior; the miner printing rather than REFUSING a declared-plane mismatch; the IDL's false "RELOAD needs `stop_on_miss` false"; an 8a tilt-clamp reported as `REJECTED_EVENT_VEL`; two over-claiming docstrings; and a 40 Hz publish-rate claim in the plan that is really 5 Hz). **ONE HIGH IS UNFIXED AND CONSTRAINS THE FIRST SITTING** — a session cancel during the auto-reload interlude is honoured in any reload phase, including with a BB ball airborne, and retracts the hand under it; the reload FSM has no `_toss_cancel_deferred` equivalent, so which phases defer is an operator design call. **Until then: do not cancel a `TossContinuous` session during a reload interlude — cancel in the dwell.** Four operator items before the first capture: clear the test-written `temp/logs` backlog (481 `toss_records_*.jsonl` + 190 `*_trim_proposal.yaml` when written; **re-counted 2026-08-18 and it is 6 + 6** — the backlog cleared itself, so this item is discharged), `git status config/` right after the first SC-0 run (it overwrites the tracked `toss_calibration.yaml` with probe maps), **set `toss_tier: "8a"` for the capture sitting and restore afterwards — the capture tool's R6 rung refuses at the 2026-08-14 8b default because the aim map is DEFINED at 8a**, and expect `dwell_tilt_degraded` at the shipped 6.0 s cadence (~0.7 s of quiescent dwell buys 1–2 of the authorised 8 reads; the full schedule needs `dwell_time_s` ≈ 7.5 s+, a goal field). Design context, unchanged — the design landed as [toss-selftuning.md](toss-selftuning.md), amended with the operator's eight binding decisions (that plan § 9): Phase 0 before the first capture and the operator runs it, so **the whole 2a–2f build is desk-side**; Layer 1.5 dwell inclinometer read approved as a **covariate only** (N=8 at 0.15 s, never overlapping PREPARE→THROW, degrade the read count rather than delay the throw); cadence stays 6.0 s (the 4.10 s fork is not built); ball supply stays operator-managed with **no magazine fence** — `max_reloads = 3`, `STOPPED_RELOAD_BUDGET` fails the session closed and the capture tool skips that node as thin/stale rather than aborting; **`ABORTED_NO_RELEASE` retry REOPENED** (retry ONCE iff the sensor reads valid-HELD, UNKNOWN/EMPTY ⇒ stop, two consecutive ⇒ stop the session) — the design's D9 deferral is superseded; map ownership is `reload_coordinator_node._build_toss_cycle` writing versioned `config/toss_calibration.yaml`; the angular-origin framing is confirmed with SC-0/SC-1 still blocking. Phase 1 having landed satisfies 2d's `toss_require_ball_evidence` prerequisite — the shipped possession merge is **consumed, not re-implemented** |
| 3 | **Bench: re-baseline + tuning sittings.** Catch-rate baseline on the restored plant, then workspace grid tosses with the loop. **RE-SCOPED 2026-08-21**: the loop is ILC, the baseline is a **no-correction** baseline (and is therefore also ILC's Phase-3 baseline arm), and the sitting captures a fresh corpus across **≥ 2 flight-time cells** — the historical 19-row corpus cannot legitimately pool with it (`partition_key` carries `bridge_fw_version`, and every historical row is pre-FW-14 with a clamped hand drive). Cadence for the sitting follows the R0→R5 ladder, not the shipped 6.0 s dwell | ≥ target catch rate across the grid; Phase-3 criteria per `critical-point-ilc.md` § Phase 3 (primary residual `arrival_dir`, possession non-regression, messy-catch non-regression, non-increasing actuation variance, channel-agreement log) | PENDING (after 0–2, and after the ILC build ladder steps 1–5) |
| — | **Open:** recurring ODrive config-drift guard (launch-time SDO config assertion vs session-runbook probe — design fork, not yet chosen); bridge-uptime lag root cause (owned by refactor-2026-07 Phase 7); **FIRED 2026-08-18 — Phase 0 found the clamp WAS live, so C-HAND-2's `J ≥ 1.0126e-5 kg·m²` was measured through it** and `throw_decel_reflected_inertia_kgm2` therefore needs re-deriving on the restored drive. **MEASURED 2026-08-23 and the watch-item premise is REFUTED** (operator flew the R0–R5 ladder, `~/Desktop/rosbags/2026-08-23_19-14-54`, 15 throws × 5 tiers, bridge FW 15 / Platform FW 3). The discriminator answered **NO**: achieved decel did not become tier-independent — η runs **1.018 / 1.044 / 1.053 / 1.074 / 1.076** up the ladder, i.e. above 1 at every tier, meaning the hand **stopped SHORT of `x3` on 15 of 15 throws** and ended 0.113–0.468 rev below it. The clamp removal itself is confirmed in both channels (braking `iq` floor −11.42 → −17.77 A; the shared 4.436 m/s tier's coast peak +0.719 → −0.279 rev vs `x3`). So `J_ff = 9.5e-6` is **not** an under-estimate on the restored drive: the encoder channel bounds `J_true ≤ 9.04e-6` and the ball's ballistic fit (+9.9–15.5 % fast, corroborating the ILC corpus's +11 %) bounds it ≤ ~7.5e-6 — **both below the declared value**, i.e. the one-sided-safety clause is violated in the OVER-braking direction and C-HAND-2's documented response is to LOWER the constant, never raise it. **No value landed**: the sign is settled, the magnitude is not (the two release-speed channels differ by 15 points and `J` scales as `v⁻²`), and the prerequisite is a static rev→mm gain measurement, not another ladder — see `ros_ws/docs/hand_decel_feedforward.md` § *The re-derivation on the UNCLAMPED drive*. Two instrument defects that made the founding numbers unreadable were fixed in the same session (`logbook/2026-08-23-cadence-floor-and-inertia.md`).; **SCL3300 async-read firmware follow-on (REGISTERED 2026-08-10, not built in Phase 2)** — restructure the Platform-Teensy `get_platform_tilt` path to timer-driven background sampling into a cache (the async-cache pattern the can-bridge hand-sensor poller uses) so a tilt read never blocks the loop streaming hand moves; driver is the standing principle of RTOS-style determinism (no blocking I/O in control loops), and until it lands Layer 1.5's dwell-only schedule + degrade-never-delay rule are the fence ([toss-selftuning.md](toss-selftuning.md) § 3.10) | | |

## Constraints

- ~~**No catch-parameter tuning until Phase 0 lands**~~ — **the braking-clamp half
  lifted 2026-08-18** (§ above): the regen-direction degradation behind the
  untrustworthy 60 % baseline is gone, and the uptime half retired 2026-08-15.
  The baseline still has to be **re-measured** on the restored plant before any
  catch knob is tuned against it — that is Phase 3, not a standing freeze.
- Safety forks stay closed: ~~`MIN_TOSS_THROW_DELAY_S = 3.5` floor untouched;~~
  hand ladders + `_MAX_ARM_DISPATCHES` retained; kind-3 abort clobber rights
  untouched (C-HAND-1 "What must NOT change"). **The `MIN_TOSS_THROW_DELAY_S`
  clause is SUPERSEDED 2026-08-21**: it retires *as a floor* down the R0→R5
  ladder ([toss-selftuning.md](toss-selftuning.md) § 11), because it is a policy
  fence around a sequence that measurably costs 0.70 s and it is not what pins
  the turnaround — the hand-stroke geometry is. A ~0.1 s dispatch debounce plus
  the derived state-based interlocks (release-window guard,
  arrived-before-arming, C-HAND-1, the arm window) own the protection instead.
  **The rest of this bullet is unchanged and not on the table.**
- ~~Fresh can-bridge boot before every sitting;~~ **retired 2026-08-15** (the
  uptime lag's root cause was fixed in FW 14 and validated at 5.8 h and 15.2 h —
  `logbook/2026-08-15-fw14-validated-arc-closed.md`). `uptime_ms` is still logged
  with every timing-sensitive number, and the `latency_monitor` row on
  `/link_status` is watched during the sitting.
- All tuning data collection under tier 8a at z = 170 mm, ±150 mm workspace.
  **AMENDED 2026-08-21**: the shipped `toss_tier` default is now **`8b`**, under
  which `aim_site = throw_site` rather than the catch xy. A *self* toss is
  zero-displacement under either tier, so the physical sitting is unchanged —
  but the ILC fit must key its cells on the **aim site** and refuse rows whose
  throw/catch displacement exceeds the key quantisation, because a displaced
  toss's aim residual is not the same measurand (`critical-point-ilc.md` § The 2026-08-21 fold-in, C7).
  Do not "fix" this by forcing the tier back to 8a; that was the old capture
  tool's R6 refusal, and the tool it protected is retired.
