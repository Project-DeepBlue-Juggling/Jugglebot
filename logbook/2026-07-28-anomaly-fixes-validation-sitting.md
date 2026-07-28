---
title: The anomaly-fix run meets hardware — every levelling and hand row passes, and the one genuinely open question came back ABORT
type: investigation
date: 2026-07-28
status: in-progress
phase: "Self-toss anomaly fixes — hardware validation sitting (2026-07-27)"
related_plan: "PROMPT-anomaly-fixes-orchestration.md"
sessions:
  - temp/logs/toss_trace_2026-07-27_15-35-56.jsonl
  - temp/logs/toss_trace_2026-07-27_15-37-55.jsonl
  - temp/logs/toss_trace_2026-07-27_15-39-50.jsonl
files_changed:
  - logbook/INDEX.md
  - plans/active/levelling-frame-contract.md
  - plans/active/hand-command-continuity.md
  - plans/active/catch-reach-degenerate-overshoot.md
  - tests/hardware/session_anomaly_fixes.md
commits:
  - <pending>
subsystem:
  - motion
  - ros
  - can
  - config
tags:
  - testing
  - safety
  - kinematics
  - docs
---

# The anomaly-fix run meets hardware — every levelling and hand row passes, and the one genuinely open question came back ABORT

## Summary

First hardware validation of the 2026-07-25 self-toss anomaly-fix run (four sibling
plans, eleven landed phases, plus `hand-command-continuity` Phase 6). Run from
`tests/hardware/session_anomaly_fixes.md` § THE RUN SHEET on 2026-07-27, then
followed by the `session_phase8_toss_hardware.md` T-rung ladder. Five analysts
scored the captures offline on 2026-07-28.

The levelling contract, the levelling gate, the FK criterion change and the whole
hand-command-continuity set **pass**. `ZSEAT-2` — the only genuinely open question
in the run — came back **ABORT** on its bounce-out arm while passing its rate arm,
with the attribution **INCONCLUSIVE** against a competing measured cause. Two
things nobody was looking for turned up: the hand coasts to within **1.2 mm** of
its declared stroke limit on tosses above the run sheet's prescribed heights, and
the persisted levelling offset is **truncated to int16 milliradians** by the
Platform Teensy.

## Symptoms

Not a fault report — a scheduled validation. What the sitting was asked to settle:

- Did each of the eleven landed fixes do what its plan claims, on hardware?
- Does removing the manufactured catch seat (`_CATCH_TILT_THROUGH_RATE_RADPS`
  `0.07 → 0.0`) cost catches, as the bb-sim geometry finding predicts it might?
- Which of the checks the operator skipped actually matter?

Two operator observations arrived with the captures and are treated as evidence
throughout: the reload sequence went **three consecutive drops, then thirteen
straight catches**, and the phase-8 tosses at ±150 mm x/y showed **repeatable
slight tilts and occasional drops**.

## Diagnosis

### What was run

Can-bridge Teensy rebooted (implied boot 2026-07-27 14:05:32); Platform Teensy
**flashed** and the flash verified. Run-sheet stages 1–3 passed (operator-reported;
independently re-confirmed offline, see `LG-0`/`PF-*` below). Six `ros2 launch`
invocations, each starting its own rosbag:

| bag / launch | window | what it holds |
|---|---|---|
| `~/Desktop/rosbags/2026-07-27_15-23-27` | 15:23:27–15:37:45 | **stage 4** — the `LG-1` refusal *and* the `level`, both **before** the CAP-GATE trace started |
| `2026-07-27_15-37-50` | 15:37:50–15:39:34 | **stage 5** (CAP-RELAUNCH) |
| `2026-07-27_15-39-38` | 15:39:38–15:49:45 | **stage 6 CAP-WORK + stage 7**, 197 MB, all topics |
| `2026-07-27_15-51-24` | 15:51–16:00 | phase-8 T-rungs — the **only extremity bag** (±150 mm) |
| `2026-07-27_16-00-27` | 16:00–16:07 | the successful **T4**: 11 Tier-8b displaced throws, all accepted |
| `2026-07-27_16-07-30` | 16:07–16:10 | four Tier-8b attempts past the envelope, all refused |

Traces: CAP-GATE `temp/logs/toss_trace_2026-07-27_15-35-56.jsonl`, CAP-RELAUNCH
`…_15-37-55.jsonl`, CAP-WORK `…_15-39-50.jsonl` (37.8 MB). The three T-rung
launches had **no trace recorder**.

Capture mandates were met: CAP-WORK carries **16 launched reload balls** (≥ 12) and
**17 executed self-tosses** (≥ 7) at four tiers — commanded flight `0.700 s ×5`,
`0.808 s ×5`, `0.989 s ×5`, `0.557 s ×2` — plus one refused goal. The `0.557 s`
pair is stage 7's two 0.38 m throws; the refused goal is stage 7's attempted 0.1 m
throw. The five `0.989 s` (~1.2 m, `v_cmd` 4.858 m/s) tosses are **off the run
sheet** — stage 6 prescribes 0.6 m and 0.78 m only.

**Can-bridge Teensy uptime**, which every timing number below must be quoted
against (2026-07-18 open finding): `5,427,203 ms` at CAP-GATE start, `5,544,903` at
CAP-RELAUNCH, `5,659,103 → 6,250,503` across CAP-WORK — i.e. **1.51 h → 1.74 h**.
The board was power-cycled this sitting, but 1.5 h *before* the gate stages, not
immediately before.

### Verdict table — levelling gate, firmware, pre-flights

| check | verdict | measured |
|---|---|---|
| **LG-1** toss before level is refused | **PASS** | `Toss REJECTED_NOT_LEVELLED` at ROS `1785130402.553635729`; accept-to-terminate **≤ 0.667 s** including rclpy startup (true figure ~0.32 s, from LG-2's measured 0.353 s CLI-start-to-accept); **zero motion** over `1785130399.554…1785130406.554` — `/leg_setpoint_echo` n=280, all six legs span `0.0000 rev`; `/hand_telemetry` n=697, `pos_cmd` span `0.00000 rev`, `vel_ff_cmd` and `tor_ff_cmd` span `0.00000` (`pos_meas` span `0.00028 rev` = encoder noise at rest); `plan_kind='hold'` throughout |
| **LVL-1** the level itself | **PASS** | `Tilt reading: [0.0249, 0.0056] rad`; orchestrator `Gravity offset published: [0.014455212199034466, 0.00207002178855809]` at `1785130432.868530220`; trajectory_node `gravity correction set: tilt=[0.0145, 0.0021] rad` at `1785130432.869687542` (**+1.157 ms**); `/trajectory/status.gravity_correction_loaded` False→True at `1785130432.954788` (**+86 ms**); `Level state persisted: complete=True, offset=[0.0145, 0.0021] rad` |
| **LG-2** after level the same goal is not refused | **PASS** | all **200** `trajectory/status` rows carry `gravity_correction_loaded=true`, zero flips; goal `1354907d` accepted at trace `t=+18.631 s`; `POSITIONING` `+19.003`, `PREPARING` `+19.773`, `THROWING` `+19.962`, `BALL_IN_FLIGHT` `+22.838`, `CATCHING` `+23.762`, terminal `+25.345`. `CHECKING` cleared in **< 372 ms** |
| **LG-3** the relaunch case | **INCONCLUSIVE** | precondition never established. `/gravity_offset` published once at `1785130677.182675`; trajectory_node `gravity correction set` at `1785130677.184924` (**+0.729 ms**); status flip at `1785130677.218599`. The operator's `activate` did not arrive until `1785130681.807736` — **4.59 s after the flip**. The toss proceeded with `gravity_correction_loaded=TRUE`, i.e. the gate correctly passing a levelled machine, **not** the ABORT (which requires `loaded=false`) |
| **LG-4** no refusal after a level, whole sitting | **PASS** | exactly **1** `Toss REJECTED_NOT_LEVELLED` across the sitting — the deliberate LG-1. Unrelated rejections: `REJECTED_BB`, `REJECTED_BB_BUSY`, `REJECTED_FLIGHT_TIME`, `REJECTED_WORKSPACE`, `3× REJECTED_DISPLACEMENT` |
| **LG-5** runbook reader, literally on CAP-GATE | **NOT-SCORABLE** | `status rows: 200  loaded-flips: [(5400.611621531, True)]` → `INCOMPLETE: need >=2 flips and >=2 outcomes`. Capture-timing artifact, not a machine finding — see § Instrument defects |
| **LG-5** the ordering invariant, reconstructed | **PASS** | all four criteria, across bag + node logs on one clock: exactly one transition, flips list `[(t0, False), (1785130432.954788, True)]`; first outcome `REJECTED_NOT_LEVELLED` at `1785130402.553636`; **1st outcome → flip = +30.401 s** (positive); **flip → 2nd outcome = +149.778 s** (≫ +0.005 s); second outcome `MISSED`. Full wiring observed end to end across **processes** |
| **FW-1** Platform Teensy firmware version | **PASS** | `PLATFORM_FW_CHECK: OK — Platform Teensy reports v1 (expected v1)` on **all six** launches; zero FAIL / UNKNOWN / INTERFACES_STALE. Bag-side, independent: `/robot_state` n=**85270**, single distinct `(platform_fw_version, platform_fw_version_read) = (1, True)`; `/link_status` kv `platform_fw_version = '1'`; same on the 15-37-50 bag (n=**9773**) |
| **FW-2** `jugglebot_interfaces` freshness | **PASS** | zero `INTERFACES_STALE` across all **77** non-empty 2026-07-27 node logs and all three `launch.log`s; positive on-the-wire confirmation — `/robot_state` carries both new fields, `TrajectoryStatus` carries `gravity_correction_loaded` (200/267/2958 rows across the three traces); `/robot_state` published **60,001 msgs over 600 s** at a clean 100 Hz |
| **LG-0 / PF-1…PF-6** stage-3 pre-flights, re-run read-only | **PASS** | `PF1_OK`; `PF2_OK`; `JB_OP_HAND_CATCH_PRIME_REV = 9.9594` and `HAND_STROKE_TOP_REV = 9.95940313273228`; `PF4_OK` then `0`; PF-5 = `1 / 3 / 2`; PF-6 = `1` |
| **Standing rule 1** can-bridge uptime discipline | **PASS** | see the uptime figures above; `link_status` level 0 `OK` on **5915/5915** samples, bus1/bus2 OK throughout |

The **cold-start line at the flash launch** is what made `LG-1` reachable at all:
`cold-start state (boot): is_homed=0 levelling=0 pose=(0.0000,0.0000)` — the
Platform Teensy flash rebooted the board and wiped its persisted offset. That is
*not* standing rule 1's can-bridge power-cycle, and it matters for how `LG-3` can
ever be run (§ Discussion).

### Verdict table — levelling frame, FK, catch reach

| check | verdict | measured |
|---|---|---|
| **FK failure census**, all bags, both streams | **PASS** | **0 failures / 105,318 FK solves** — commanded (`/leg_setpoint_echo`): 0/3326, 0/1468, 0/23283, 0/15737, 0/11536, 0/4564; measured (`/robot_state` `pos_estimate`): 0/45404. Includes the four ±150 mm extremity poses and the two out-of-envelope 8b attempts |
| **FK-1** seed / guard-descent FK failures | **PASS** | zero `seed FK failed` / `guard descent FK failed` in `/rosout` of all three traces |
| **FK-2** deferred / historical RAI, iteration ceiling | **PASS** | `def_rai 0` on both `/leg_setpoint_echo` (23283) and `/robot_state` (60001); `hist_rai` **257/305** > 0 so not vacuous; `max_it 5` ≤ 10 |
| **FK-3** seeded hold pose | **PARTIAL** | two `seeded hold at pose` prints survive: `x=0.5 y=0.5 z=170.3` and `x=0.6 y=0.4 z=170.4`, both well inside ±2.0 mm. The strict "matches the pre-change print to the last digit" arm is **not scorable** — no pre-change reference print was captured, and the T-rung launches' prints are lost to the `output='screen'` gap |
| **LVL-3** park gate, stage 6 | **PASS** | park `rx −0.8021°`, `ry −0.1146°`; expected `−0.8021 / −0.1146`; err **+0.0000 / −0.0000** against ±0.05. 23283 samples, span 582.2 s. Not ≈0.0000 (pre-fix frame), not ≈−1.5576 (double-applied) |
| **LVL-3** park gate, stage 4 | **PASS** (with `--t0 190.0`) | park `rx −0.8282°`, `ry −0.1186°`, err **−0.0000 / −0.0000**. **Without `--t0` it reads FAIL** at `rx +0.0270°` — the 189.0 s pre-`go_home` ACTIVATE hold outvotes the 37.4 s corrected park; the runbook's documented `NOTE:` line fired as designed |
| **LVL-3** park gate, stage 5 | **PASS** | park `rx −0.8022°`, `ry −0.1145°`, err **−0.0000 / +0.0001** — the correction survived the relaunch and was restored automatically on boot |
| **LVL-3-class** commanded `rx` flat across toss goals | **PASS** | **17/17** self-toss goals: `max|rx − target| = 0.0000°` and `max|ry − target| = 0.0000°` on every goal. Whole self-toss block 321.8–565.8 s, **9756 samples**: `rx` ptp `0.0000°`, `ry` ptp `0.0000°`. Target `(−0.802141, −0.114592)°` |
| **LVL-4** mocap cross-check (REPORT-only) | **PASS** | world-frame across the `level` step: commanded `rx` changed **−0.8175°**, measured Platform `rx` changed **+0.8227°** (ratio **1.0064**); commanded `ry` **−0.1301°** vs measured **+0.1330°** (ratio **1.0223**). Correction reached the legs and was applied **once** |
| **CCATCH-2** row 1 — wrong-side excursion | **PASS** | **0.0000°** on every scored self-toss (1, 6, 11, 16, 17) against a model peak of `+1.9028…+2.2386°`. Counterfactual: `UNREQUESTED (wrong-side) excursion 0.3864…0.4344 → 0.0000 deg`, and the echo matches the **fixed** column |
| **CCATCH-2** row 2 — settle | **PASS** | echo settle `rx −0.8021°`, `ry −0.1146°` vs target `−0.802141 / −0.114592` → `|err| ≤ 0.0001°`. The old `×1.3847` aim would have read `−1.099921 / −0.157132`; the echo does not show it |
| **CCATCH-2** row 3 — residual vs gravity at contact | **PASS** | **0.0000°**. Counterfactual quantifies what was removed: settle `rx −1.099921 → −0.802141`, i.e. **+0.297780° of through-seat residual gone** |
| **CCATCH-2** row 4 — plan segments | **INCONCLUSIVE** | the machine's own segment count could not be read off `/trajectory/status` or `/trajectory/diagnostics`. Everything observable is consistent with **2** and inconsistent with a live decay segment (counterfactual rebuilds `['3.4491','0.5000']` vs the pinned-0.07 model's `['3.4491','0.1500','0.5000']`; no decay signature in the echo; `peak_leg_*` published 0.0) |
| **CCATCH-2** row 5 — `peak_leg_acc` / `_jerk` | **PASS** | published `(0.0, 0.0, 0.0)` for vel/acc/jerk — **not** ≈142 / ≈3950. Note the row's stated PASS value (≈1.2 / ≈3) does not describe a healthy machine here: this sitting's self-tosses catch at the park, so the reach is a genuine zero-displacement no-op |
| **CATCH-2** replay `VERDICT: REPRODUCED` | **NOT-SCORABLE** | `NOT-REPRODUCED` on every scored toss, `rx` residual rms **1.54–1.75°**, max **2.71–3.04°**. Instrument limitation — see § Instrument defects. `CCATCH-1`/`ZSEAT-1` self-check was run first and passed: `SELF-CHECK: PASS`, **10/10 OK**, case 7 shows `planner._CATCH_TILT_THROUGH_RATE_RADPS=0.0` **and** recorded-session rate `0.07` |
| **CCATCH-5** `peak_leg_*` clears on a report-less install | **PASS** (partial — the `go_to_pose` half was never run) | of **209** distinct `move_seq` values on `/trajectory/diagnostics`, exactly **18** carry a non-zero `peak_leg_vel_mmps` (**29.8–29.9 mm/s**, the reload pre-tilt reach) and the other **191** read `0.0`; the first non-zero (`move_seq 2`: 29.8 / 38.9 / 174) is followed immediately by `move_seq 3` at `0.0` |

### Verdict table — ZSEAT (the seat-removal experiment)

| check | verdict | measured |
|---|---|---|
| **ZSEAT-1** instrument self-check | **PASS** | `SELF-CHECK: PASS`, 10/10 OK, `planner._CATCH_TILT_THROUGH_RATE_RADPS=0.0`, recorded-session rate `0.07` |
| **ZSEAT-2 rate** | **PASS** | **13 caught / 16 attempted = 0.8125** (≥ 0.63). Denominator: 20 reload goals; goal 1 `REJECTED_BB` and goal 17 `REJECTED_BB_BUSY` (no ball); goals 9 and 18 announced balls 31 and 62 but logged `bb/throw: THROW_ABORTED_NOT_SETTLED (axis=YAW)` so no ball left the BB; the remaining **16** logged `bb/throw: OK`. Numerator confirmed **independently of the operator** by a mocap floor census: balls come to rest on the floor only after attempts 1, 2 and 3; zero floor arrivals after attempts 4–16 |
| **ZSEAT-2 flatness** | **PASS** | `span_deg = 0.0000` on all five measured windows, plus `commanded position span (x,y,z) mm = 0.00, 0.00, 0.00`. Samples 32–33, FK failures 0 on every window. Independently corroborated: the **installed** `planner.py:837` reads `_CATCH_TILT_THROUGH_RATE_RADPS = 0.0`, mtime `2026-07-27 03:29` (before the 15:23 session) |
| **ZSEAT-2 bounce-out** | **ABORT** | **3 bounce-outs, all 3 consecutive** (attempts 1, 2, 3 = balls 6, 9, 11). **Both** ABORT clauses tripped (≥ 3 total, and ≥ 2 consecutive) |
| **ZSEAT-2 overall** | **ABORT** | rate PASS, flatness PASS, bounce-out ABORT ⇒ net ABORT. Routes back to § Section ZSEAT |
| **ZSEAT-2 attribution** vs sitting 4 | **INCONCLUSIVE** | a competing cause is measured and real — see § The three drops |
| **ZSEAT-3** offline counterfactual, reload path | **NOT RUN** | flagged, not scored: the commanded settle measured off the bag is `rx +1.7581° / ry −10.6823°`; ZSEAT-3's reference target is `+1.774062 / −10.636334°` to ±0.02°. `rx` is inside (Δ −0.0160); **`ry` is OUTSIDE** (Δ −0.0460). The target depends on the live gravity offset, which may differ from the 2026-07-25 reference bag — the replay must be run before concluding anything |
| **ZSEAT-4** release stillness, stage 6 | **PASS** (weak) | **35/35** releases (18 ball_butler + 17 jugglebot): worst tilt span `0.0000°`, worst position span `0.0000 mm`, 7–9 samples per window. Weak because all 35 releases fall inside a stationary hold and the Tier-8a catch point equals the park, so the check could not have failed |
| **ZSEAT-4** release stillness, Tier 8b (where it has teeth) | **PASS** | **11/11** on the pre-release half window `[release−0.10 s, release]`: `rx` span `0.0000°`, `ry` span `0.0000°`, position span `0.0000 mm` on every displaced throw — and here the deferred A→B reach genuinely fires (51.5–72.2 mm of travel between release and landing) |
| **ZSEAT-4** Tier-8b dispatch timing | **PASS** | first `catch/dynamic_target` relative to release: `+0.017, +0.024, +0.027, +0.024, +0.030, +0.025, +0.014, +0.037, +0.013, +0.050, +0.045 s` — **never negative**, 11/11. Also 4/4 on the 16-07-30 bag (`+0.049, +0.003, +0.051, +0.003 s`), though all four goals there were refused so no reach ran |
| **ZSEAT-4** Tier 8b, the row **as literally written** (symmetric ±0.10 s) | **NOT-SCORABLE** | **9/11 PASS, 2 FAIL** — toss 1 position span `0.334 mm`, toss 7 `0.260 mm` (tilt spans `0.0093°` / `0.0072°`, both well inside 0.02°). Criterion defect, see § Instrument defects |
| **CAP-WORK attempt-count mandate** | **PASS** | 16 reload attempts with a ball launched (≥ 12); 18 toss goals = 17 executed + 1 `REJECTED_FLIGHT_TIME` (≥ 7). **Not interleaved** as the runbook describes — all 16 reloads ran first (trace `t+18…t+282`), then all 18 tosses (`t+330…t+577`) |
| **one-outcome-line-per-reload** discipline | **PASS** | 20 `Reload requested … goal dispatched` and exactly 20 terminal `Reload not caught:` lines (18 MISSED, 1 REJECTED_BB, 1 REJECTED_BB_BUSY); one per goal, no duplicates, no orphans, verified against the action-status goal UUIDs (20 goals, each 2 → 6) |

### Verdict table — hand command continuity

| check | verdict | measured |
|---|---|---|
| **INST-3** probe self-gate | **PASS** | exit 0; `GATE PASS — 25/25 rows within tolerance` **and** `GATE PASS — fixed-shape branch`, all five cases present |
| **H4.0b** firmware xref suite | **PASS** | **173 passed in 2.05 s, no skips** — `g++` present, so the compile-and-run xref against the shipped `Trajectory.h` actually ran |
| **H4.0d** Platform Teensy FW | **PASS** | `PLATFORM_FW_CHECK: OK — Platform Teensy reports v1 (expected v1)` in the stage-6 bridge node log. **The flash took** |
| **row 1** `trunc` | **PASS** | 3 of 17 print an instant (tosses 19, 20, 34 at `9.8090 / 9.8016 / 9.8704 rev`), **but on all 17** `pos_cmd` reached x3 (`9.9589–9.9594`) and held it for **28.0–61.6 ms** before any new command landed — the queue was never cleared mid-stroke. Criterion defect, see below |
| **row 2** `seeds` / **H4.6** | **PASS** | `seeds=1` on tosses 19, 20, 34; every seed at `|pos_cmd − pos_meas| = 0.0000–0.0009 rev`, targeting exactly x3, landing **28.0–31.9 ms after** `pos_cmd` had already reached x3 |
| **row 3** peak — 0.6 m (×5) and 0.38 m (×2) | **PASS** | 0.6 m: `10.0058, 10.0056, 10.0407, 10.0290, 10.0306`; 0.38 m: `10.0295, 10.0371 rev`. All ≤ 10.041 (bound 10.060), ≥ 1.06 rev headroom to 11.1 |
| **row 3** peak — 0.78 m (×5) | **ABORT** | `10.3203, 10.3045, 10.2851, 10.3258, 10.2940 rev`. **Not a Phase-4 regression**: the pre-fix baseline at the same commanded height was `10.1653–10.3248`, and the **commanded** position during every stroke peaks at exactly x3 (`9.9589`) — the excess is position-loop coast |
| **row 3** peak — ~1.2 m (×5, **off run sheet**) | **ABORT** | `11.0506, 10.8601, 11.0621, 11.0238, 10.9024 rev` = **349.9 mm** at worst; headroom to the 11.1 rev clip bound **0.0379–0.2399 rev = 1.2–7.6 mm**. Peak-over-x3 grows steeply with commanded speed: 2.74 m/s `+0.074`, 3.44 m/s `+0.063`, 3.97 m/s `+0.345`, 4.86 m/s `+1.020 rev` (group means). Max **commanded** position anywhere was `10.2259 rev` |
| **row 4** `dip_below_x3` — **the gated row** | **PASS** | 15 of 17 read **0.0000–0.0257 rev (0.0–0.8 mm)** against a pre-fix range of **0.339–1.748 rev (10.7–55.3 mm)** — a **40–70× reduction** at the same commanded height. Tosses 30 and 32 read `0.1755 / 0.1734 [OVER]` and **both** carry the row-7 annotation (an honoured velocity-continuous brake) — exactly the case the row-4/row-7 qualifier says to REPORT |
| **row 5** pullback — 0.6 m / 0.38 m | **PASS** | `−2.6, −1.5, −1.8, −1.6, −2.3` and `−1.6, −0.9 rev/s` against a pre-fix range of `−17.9 … −42.4 rev/s` |
| **row 5** pullback — 0.78 m / 1.2 m | **NOT-SCORABLE** | row 3 aborted first, so by the runbook's own ordering row 5 is not independently scored. Recorded anyway: the 0.78 m tier still improved **3–4×** against the pre-fix baseline (`−10.7 … −8.3` vs `−29.7 … −42.4 rev/s`) |
| **row 6** `catch_desc` | **PASS** | present on all **35** announcements; no row reads `no-throw-stroke(!)`. The ±20 ms timing half was not independently recomputed — scored on presence only |
| **row 7** `first_neg_cmd` / **H4.3** | **PASS** (REPORT) | annotated on **6 of 17**. Commanded seed velocity identifies the branch: **velocity-continuous on tosses 29, 30, 32, 33** (`v0 = −8.44 / −6.90 / −6.98 / −7.55 rev/s`, all above the 6.0 rev/s dead-band); rest-to-rest on the other 13. Bulge below x3 of `0.267 / 0.254 rev`; max commanded position `10.2259 rev` (0.374 rev under the 10.60 clamp); never below the hand's own live position |
| **H1.2** stroke-busy window latched | **PASS** | **17 latched = 17 jugglebot tosses** — the gate saw every announcement |
| **H1.3** arm withheld | **PASS** | 17 withheld lines, exactly one per toss, durations **81–128 ms** |
| **H1.4** window CLOSED warnings | **PASS** | **0** across the whole sitting, including both 0.38 m throws (the 115 ms window corner), at `vel_scale 0.80` (53/53 arm lines) |
| **H1.5** reported slack | **PASS** | `0.286…0.311` (0.6 m), `0.404…0.408` (0.78 m), `0.598…0.601` (1.2 m), `0.124, 0.124 s` (0.38 m). **Minimum 0.124 s = 2.5× the 0.050 s floor** |
| **H1.6 / H4.7** "Not enough time for smooth-move" | **PASS** (indirect) | 0 hits in the stage-6 `launch.log` and 0 in any node log of that launch; every one of the 35 announcements has a `catch_desc`. **Scored indirectly** — the Teensy serial was not captured, so the primary source does not exist |
| **H1.7** every withheld toss redeemed | **PASS** | each of the 17 withheld lines followed by 1–2 `Arming hand catch` lines before the next latch; **53** arm dispatches total |
| **H2.1** arms per toss | **PASS** | `arms = 1` on 8 tosses, `2` on 9 tosses; **never 3** |
| **H2.2** `seeds` on `arms == 2` | **PASS** | 9 tosses read `arms == 2` (criterion exercised). Tosses 19 and 20 print `seeds=1`; the other 7 print 0. Both seeds land **28.0 / 31.5 ms after** `pos_cmd` reached x3, at `|cmd−meas| = 0.0004 / 0.0009 rev`. Pre-fix signature was **2** seeds per `arms=2` toss at `0.0000 rev` from live pos |
| **H2.3** dip on `arms==2` vs `arms==1` | **PASS** | `arms==2`: `0.0009, 0.0000, 0.0000, 0.0000, 0.0168, 0.0051, 0.0056, 0.0054, 0.0000`; `arms==1`: `0.0004, 0.0257, 0.0176, 0.0000, 0.0066, 0.1755, 0.1734, 0.0000 rev`. No systematic difference attributable to the second dispatch — if anything `arms==2` is tighter. Both OVER rows are `arms==1` (the brake tosses) |
| **H2.4 / H3.6 / H4.8** SAFE_ABORT | **NOT-SCORABLE in stage 6** | 0 occurrences in the stage-6 launch. **But one did occur later**: `SAFE_ABORT: hand retract dispatch failed (attempt 1/4; telemetry stationary)` at `1785132178.810` in the **16:00:27** launch. No trace recorder covered it, but `/hand_telemetry` is in that bag — so these rows moved from *unexercised* to *scorable, not yet scored* |
| **H3.1** `pos_meas` at rest after a prime | **PASS** | 18 full-stroke prime ascents; settled `9.9571–9.9586 rev` at +0.30 s and +0.60 s after arrival (median 9.9575, **total spread 0.0016 rev = 0.05 mm**) |
| **H3.2** `Hand primed to` | **PASS** | **14/14** read `9.959 rev`; zero `9.858` — the installed copy carries the derived prime |
| **H3.3** `ABORTED_PRIME_FAILED` | **PASS** | **0**. Alongside: 11× `Hand priming failed: HAND_TRAJ_CMD: ERR_TIMEOUT` and 5× `hand prime dispatch failed (attempt N/4; telemetry stationary)` — the telemetry-verified ladder absorbed every one, and no re-dispatch landed inside a live ascent |
| **H3.4** `smooth_move_hand` reject / out of range | **PASS** | 0 hits |
| **H3.5** peak `pos_meas` during a prime ascent | **PASS** | `9.9570–9.9586 rev` across 18 ascents — the hand does **not** overshoot the prime target at all |
| **H3.7** peak `vel_meas` during a prime ascent | **PASS** | `24.55–25.60 rev/s` (median ~25.0) against a commanded quintic peak of `24.63` (observed `24.59–24.70`) — **on model to within 4%**, no clobbered-profile signature |
| **H4.1 / HAND-4** the flash did not break the clean path | **PASS** | rows 1–5 on the 0.6 m and 0.38 m tiers are clean; the 0.78 m and 1.2 m peak rows abort on ballistic coast the pre-fix baseline shows at the same magnitudes |
| **H4.2** peak vs a pre-flash capture | **NOT-SCORABLE** | no pre-flash control exists on this run sheet (deliberate). Nearest comparison at 0.78 m: pre-fix mean ~10.24 vs post-flash mean 10.306, Δ ≈ +0.06 rev — but the pre-fix strokes were truncated mid-stroke, which shortens the coast, so it is not like-for-like |
| **H4.9** minimum commanded/measured position | **PASS** | min `pos_cmd` over the whole trace = **−0.0581 rev (−1.84 mm)**, 239 samples across ~44 episodes; **every one is the first sample of a new move, equal to the hand's live `pos_meas`** at that instant, and the profile then rises monotonically. Pre-flash comparison: `−0.1036` and `−0.0274 rev` — the post-flash minimum is **shallower**. Documented endpoint relaxation, pre-existing, not planned travel onto the stop |
| **H4.10** duration of any commanded hand move | **PASS** | 155 commanded-move episodes; longest single move **0.794 s** ≤ 0.8005 s. The one 1.048 s window is two back-to-back moves, split verified in the raw stream where `vel_cmd` flips sign. Cap held including on the four velocity-continuous preludes (0.100–0.325 s) |
| **stage 7 / HAND-1b** two 0.38 m throws | **PASS** | tosses 34 and 35 (`v_cmd 2.742 m/s`, tof 0.557 s): peak `10.0295 / 10.0371`, dip `0.000 / 0.000`, pullback `−1.6 / −0.9 rev/s`, arms 1 / 2, **slack 0.124 s both**, 0 CLOSED warnings. The 115 ms window case did not close |
| **stage 7** the attempted 0.1 m throw | **PASS** | goal `8b68ce0d` EXECUTING → ABORTED in **4.4 ms** with `Toss REJECTED_FLIGHT_TIME`; hand `pos_cmd` identically `0.0000 rev` (span 0.0000) over ±5 s, `|vel_ff_cmd| = 0.00` throughout. Refused in CHECKING before any dispatch — the hand was never touched |
| **reload (ball_butler) rows** — the inert shape | **PASS** | all 18 announcements read `no-throw-stroke` with trunc/seeds/peak/pullback/dip all `-` and a `catch_desc` present on every one. None reads `no-throw-stroke(!)` |
| **ungated** — achieved flight per toss | **PASS** (recorded, not gated) | commanded → achieved (s): `0.700 →` 1.002/1.011/1.091/1.059/0.980; `0.808 →` 1.266/1.093/1.134/1.134/1.259; `0.989 →` 1.276/1.429/1.407/1.469/1.274; `0.557 →` 0.838/0.761. Uniformly **+38…+47 % long**. Caveat: every toss also logged `Toss MISSED` under the tracker corruption below, so the landing-crossing these derive from is suspect |
| **ungated** — dispatch shift (fit − announcement) | **ABORT** | trace path min `+57.3`, median `+63.9`, max `+78.2 ms`; bag path (like-for-like against the pre-fix bag-clock column of `+12.8…+21.9 ms`) min `+54.0`, median `+59.2`, max `+62.7 ms`. **Exceeds the 40 ms margin** the stroke-busy window budgets. Correlates with can-bridge uptime: 0.24 h → `+14.5 ms`, 0.54 h → `+23.4 ms`, 1.57–1.74 h → `+54…+63 ms` |

### The auto-push race is settled, and the push wins

The single most consequential observation of the sitting, and it retires an
"unmeasured" hazard the levelling plan has carried since Phase 0's Table C.

Across all six 2026-07-27 launches there were **7 publishes on `/gravity_offset`,
and `trajectory_node` received every one — 7 of 7**. Orchestrator-line to
trajectory_node-line latency, from the per-node ROS timestamps:

| publish | latency |
|---|---|
| 15:23 post-level re-push | **+1.512 ms** |
| 15:37:50 (stage 5) | **+0.729 ms** |
| 15:39:38 (stage 6) | **+3.502 ms** |
| 15:51:24 | **+3.387 ms** |
| 16:00:27 | **+0.429 ms** |
| 16:07:29 | **+1.716 ms** |
| the `level` publish itself | **+1.157 ms** |

It is not a coin flip, because the race is **structurally rigged in the
subscriber's favour**: the push fires on the orchestrator's *first IDLE entry*,
which cannot happen until `teensy_bridge_node` has completed its boot `RobotState`
read and the orchestrator has left BOOT — while `trajectory_node` was launched
simultaneously and has been up for seconds. Measured head start (trajectory_node
process start → push): **4.34, 2.74, 3.86, 3.48, 3.88 s**.

A second analyst scored the **boot-push subset** independently and got the same
answer — the boot auto-push won **5/5** across every relaunch, with the
`gravity_correction_loaded` flip landing `+3.2, +1.4, +2.8, +3.0, +3.0 s` after
first status in the five relaunch bags.

**Consequence**: on a machine whose Platform Teensy holds a persisted offset, a
relaunch comes back **already levelled**, `LG-1` and `LG-3` as written will not
fire, and the correct operator action is standing rule 2's *check, don't level* —
which the operator did correctly in stage 5. This should be promoted from
"unmeasured" to "measured, push wins 7/7" in standing rule 2 and in the
`LG-1`/`LG-3` warning boxes.

### The three drops

Recorded at exactly the strength the evidence supports, and no stronger.

**What the operator saw** (outranks all trace inference): reload attempts in order
`(0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1)` — three consecutive drops, then thirteen
straight catches. A mocap **floor census** confirms the numerator independently:
balls come to rest on the floor after attempts 1, 2 and 3, and after none of the
other thirteen.

**Classification (trace inference, not operator-confirmed)**: all three were
**bounce-outs**, not missed arrivals. Every one of the 16 balls arrived within
**~40 mm of the cup axis**, so there is no BB-scatter miss anywhere in this sitting.

| ball | evidence | confidence |
|---|---|---|
| **6** (attempt 1) | entered `(−7.2, +16.1) mm` at z=1050 descending 5.2 m/s; vertical rate **collapsed from −5548 mm/s at z=1150 to −1780 mm/s at z=850** (ratio 0.32) with no ballistic explanation; gained +x/+y lateral velocity; struck the floor at ~`(162, 203, 30)` at landing+0.43 s and rolled out of the volume | **HIGH** — contact tracked continuously |
| **9** (attempt 2) | entered `(−18.8, +3.6)` at z=1050; decelerated to `−363 mm/s` by z=750; **dwelt at z=705–735 for ~130 ms** against a seated-rest height of 687 mm, wandering laterally; slid out in +y; came to rest on the floor at `(−33.7, 230.0, −16.4)` at landing+0.64 s | **HIGH** — contact tracked continuously |
| **11** (attempt 3) | entered `(−21.5, +7.4)` at z=1050; mocap lost it at z=1111; reappeared at `(−54.3,−25.3,734.2)` / `(−53.9,−24.2,731.7)` at landing+0.129/+0.133 — **345 mm above the free-fall prediction** and within 2 mm of where ball 9 dwelt; then `(−51.4,+8.2,674.3)` at +0.187 (seated-rest height); rest on the floor at `(−8.9, 338.5, 45.7)` at landing+0.60 s, where unimpeded free fall from its last sighting needs 0.21 s | **MEDIUM-HIGH** — the contact itself is occluded; the position/time budget cannot be reconciled with free fall |

**Attribution is INCONCLUSIVE**, and the competing cause is measured and real. The
three drops arrived **systematically further +x** than the thirteen catches:

- `x` at z=1000 (linear fit): drops `+6.7, −21.6, −10.7` (mean **−8.5**); catches
  `−14.5, −27.0, −36.3, −30.1, −39.8, −41.4, −46.0, −39.2, −35.6, −39.3`
  (mean **−34.9**, sd **8.9**, n=10 usable) — an offset of **~26–39 mm**, about
  **3σ** of the catch spread.
- The cup did **not** move: commanded tilt identical every attempt
  (`rx +1.7581 / ry −10.6823°`), mocap Platform body `(10.5–10.8, 2.1–2.5,
  176.9–177.1) mm` across all 16 (**< 0.5 mm** spread); mocap frame stable.
- The pattern is a **monotonic drift, not a step**:
  `+6.7 → −21.6 → −10.7 → −14.5 → −27.0 → −36.3 → −30.1 → …` plateauing at
  `−35 … −46` around the sixth throw — a **BB warm-up signature**.

The analyst's framing, recorded as written: the warm-up drift *is the proximate
trigger, but it does not exonerate the zero seat and does not convert these into
missed arrivals*. A ~30 mm off-centre arrival on a stationary 10.8° rim is a
description of **the disturbance the 0.07 rad/s seat existed to reject**, not an
alternative to it. *"A seat defect should not heal after three attempts"* is the
wrong null — the seat did not heal, the disturbance shrank. The decisive
marginality: **attempt 3 dropped at `x = −10.7` and attempt 4 caught at
`x = −14.5` — 3.8 mm apart**, so the capture-basin edge sits exactly where the
warm-up excursion was crossing.

This capture contains **no throws at ~+30 mm offset with a non-zero seat**, so the
two hypotheses cannot be separated from it.

Three robot-side alternatives were tested and **refuted** with measured (not
acked) telemetry:

- **Hand not primed / not armed.** Drop 1's log shows 2 `ERR_TIMEOUT`s, no
  `Hand primed to` line and no `armed on Teensy` ack — but the telemetry says the
  hand was at 9.959 rev at landing−1.0 s and executed a full 9.99–10.01 rev stroke
  at peak `|vel| 79.9–98.7 rev/s` on **all 16** attempts.
- **Mistimed stroke.** Stroke start minus the ball's mocap-measured z=1000
  crossing: drops `−12.2, +11.9, +3.7 ms`; catches mean `−21.8 ms`, sd `29.9 ms`
  (n=11). All three drops sit **inside** the catch distribution.
- **Stale install (the seat did not actually ship).** Commanded tilt span
  `0.0000°` over the last 0.8 s before every landing measured, and the installed
  `planner.py` carries `0.0` with a pre-session mtime.

> **Not yet done, and it outranks everything above**: nobody has asked the operator
> whether they *saw* the balls touch the cup and leave. Their by-eye classification
> outranks this trace inference, and it decays by the hour. Ask.

### Extremity tilt at ±150 mm — the seed of the lookup-table decision

From the phase-8 extremity bag `2026-07-27_15-51-24` (20 Tier-8a tosses; the only
extremity bag). The operator's observation was *repeatable slight tilts, occasional
drops*. Quantified:

**It is not commanded.** At every settled hold — centre, `(60,0)`, `(150,0)`,
`(150,150)`, `(−150,150)`, `(−150,−150)`, `(150,−150)` — commanded
`rx = −0.8021°`, `ry = −0.1146°`, `rz = 0.0000°`, `z = 170.00 mm`, with **zero
variation**.

**It is not a leg-tracking error.** FK of the *measured* leg positions departs from
commanded by only `+0.005 … +0.029°` in `rx` and `−0.020 … +0.012°` in `ry`.

**Mocap (Platform in the QTM world frame), centre-referenced:**

| position | n | Δrx | Δry | \|Δ\| | sd_rx | sd_ry | Δz |
|---|---|---|---|---|---|---|---|
| `(60, 0)` | 1 | — | — | **0.041°** | — | — | +0.18 mm |
| `(150, 0)` | 3 | +0.150° | +0.116° | **0.190°** | 0.0013° | 0.0126° | 0.00 mm |
| `(150, 150)` | 5 | −0.073° | +0.133° | **0.152°** | 0.0047° | 0.0095° | −0.96 mm |
| `(−150, 150)` | 3 | −0.224° | +0.001° | **0.224°** | 0.0088° | 0.0051° | −1.36 mm |
| `(−150, −150)` | 4 | −0.127° | −0.196° | **0.234°** | 0.0121° | 0.0144° | −0.92 mm |
| `(150, −150)` | 3 | +0.593° | +0.112° | **0.604°** | 0.0098° | 0.0036° | +0.12 mm |

**Repeatability is excellent**: every sd is `0.001–0.014°`, i.e. **15–40× smaller
than the effect** at that position, and all are inside the ±0.05° levelling gate
band. Visits to a position were separated by returns to centre and by 5–20 s, so
this is genuine revisit repeatability, not within-window noise. **A lookup table is
well founded on repeatability grounds.**

**But it is not linear in (x, y)**: a 2-parameter tilt-vs-position fit calibrated on
`(150,0)` and `(150,150)` predicts `+0.374°` at `(−150,150)` and `+0.074°` at
`(−150,−150)`, against measured `−0.224°` and `−0.127°`. **A real table (or a
higher-order model) is required; two gains will not do.**

**Why it plausibly causes drops.** On a Tier-8a self-toss the platform tilt
mis-aims the *throw* (the hand fires along the platform normal), so the landing
point shifts by `v·θ·T`. At the extremity flight (tof 0.903 s, release
`v = gT/2 = 4.43 m/s`) that is **69.8 mm per degree**; at tof 0.700 s, 41.9 mm/deg.
Applied: `(150,0)` 13 mm, `(150,150)` 11 mm, `(−150,150)` 16 mm, `(−150,−150)`
16 mm, **`(150,−150)` 42 mm**. The worst corner's 42 mm is comparable to or larger
than the cup — which is exactly why the effect presents as *occasional*: three
corners cost 11–16 mm (marginal) and one costs 42 mm (a drop).

> **The tilt→drop link here is a physical argument (`v·θ·T`), not an observed
> correlation.** All 20 tosses in that bag logged `Toss MISSED` under the tracker
> corruption, and the reported flight times (0.859–1.266 s) exceed the announced tof
> (0.700 / 0.903 s) by 20–40 %. The correlation needs the operator's by-eye outcome
> recorded per toss, alongside the position.

**The honest confound — it is NOT proven physical.** A repeatable *mocap bias* at a
given platform position is indistinguishable from a repeatable *physical tilt* at
that position, using mocap alone: both are stable functions of position, and a
small within-window sd shows the solve is **stable**, not that it is **unbiased**.

- *For real*: the operator saw it by eye, which shares nothing with QTM; the
  world-frame tilt instrument reproduces a **known commanded 0.8175° levelling step
  to 0.6 %** (measured +0.8227°); the coherent ~1 mm z droop.
- *For caution*: this QTM setup demonstrably suffers position/motion-dependent
  solution failure (the bolted-down **Base** body's own orientation swings **6.75°
  peak-to-peak** across that session); and mocap reads the platform position
  **2–4 % larger** than commanded at every corner (`|px| 152.3–154.8` for a
  commanded 150), the signature of an imperfect QTM calibration — and a calibration
  that gets scale wrong can get position-dependent orientation wrong too.
- The z droop is **strongly asymmetric** (the two +x corners droop ~0 while the
  other three droop ~1 mm), which does *not* match a simple gravity-sag picture.

**The discriminator**: at each corner, read the **on-board inclinometer** — the same
instrument the `level` routine uses. It is a gravity reference sharing nothing with
QTM or the leg encoders, and it is the natural instrument for building the offset
table anyway. One sitting, six poses, ~10 minutes.

### New defect: the persisted levelling offset is truncated to int16 milliradians

Found independently by two analysts, in agreement, with the source line.

`ros_ws/src/jugglebot/Teensy_code/Teensy_code.ino:430` packs the offset as
`int16_t x = int16_t(s.pose_offset_tiltX * 1000.f)` — a **C cast, which truncates
toward zero**, not a round — in the `0x6E0` RobotState reply, and `:454` restores
`s.pose_offset_tiltX = x / 1000.f`.

Evidence from the bags: the 15:33 `level` measured and published
`[0.014455212199034466, 0.00207002178855809]`, but **every launch from 15:37:50
onward restored** `[0.014000000432133675, 0.0020000000949949026]` — exactly
`float32(14/1000)` and `float32(2/1000)`. The round trip was reproduced in Python
and matches to the last bit.

Consequences:

- Tilt **magnitude** `0.83667° → 0.81028°`, a **0.0264° loss** — half of `LVL-3`'s
  ±0.05° PASS band, consumed before any measurement. Per-axis, this sitting lost
  **0.4552 mrad = 0.0261°** in x and **0.0700 mrad = 0.0040°** in y.
- Truncation is **biased**, not symmetric: worst case **1 mrad = 0.0573° per axis**,
  ~0.081° total — which **exceeds `LVL-3`'s entire ±0.05° band**.
- `LVL-3` still passes because it scores the commanded park against the **same
  quantised offset it was given** — the check is self-consistent and **structurally
  blind** to this error.
- `LVL-2`'s `predicted_mm` moves `2.964 → 2.871 mm` (inside the ±11 % band, benign).
- The **write** path is not the lossy one (`encode_state_write` uses `'<BBff'`
  float32); the loss is entirely in the 8-byte CAN readback.
- It interacts with the extremity table: 0.026° is **~17 % of the smallest corner
  offset** measured above, so it is not negligible at that scale.

**Scoring consequence, immediately**: anyone scoring `LVL-3`/`LVL-2` on any capture
taken *after a relaunch* must pass the **persisted** `[0.0140, 0.0020]`, **not**
`LVL-1`'s printed `[0.014455, 0.00207]`. The run sheet's instruction to use "the
radians LVL-1's log line printed" is wrong for those captures.

Fix direction: `int16_t(lroundf(x * 1000.f))` halves the error and removes the bias;
more bits removes it.

### Instrument and runbook defects surfaced

These cost nothing this sitting only because independent evidence existed. Several
would produce a **false ABORT on a healthy machine**.

1. **`FW-1` / `FW-2`'s `launch.log` grep can never work.** `jugglebot_launch.py`
   declares `teensy_bridge_node` (`:171`) and `trajectory_node` (`:157`) with
   `output='screen'`, so their stdout never reaches `launch.log`. Confirmed: the
   15-37-50 `launch.log` carries lines from eleven other nodes and **not one** from
   either. An operator running `FW-1` literally on a correctly-flashed board gets
   "nothing at all", whose runbook mapping is the ABORT *"you skipped colcon
   build"*. The same gap kills `gravity correction set` (LVL-1), `seeded hold at
   pose` (FK-3) and `returning to neutral` (LVL-2) as launch-log greps.
2. **`FK-1` / `FK-3` / `LG-4` as written self-pass.** `ls -t ~/.ros/log/python3_*.log`
   matches **zero** files on this box, so `xargs grep … | wc -l` prints `0`, which
   reads as a clean PASS regardless of the truth — **three checks that can never
   fail**. The form that works, and which `LG-4` was actually scored with:
   `find ~/.ros/log -maxdepth 1 -name 'python3_*.log' -newermt '<date>' -size +0 -exec grep -H PATTERN {} \;`
   (zsh does not word-split an unquoted `$(find …)`, and 50,943 files match that glob).
3. **The trace recorder cannot carry `platform_fw_version`.**
   `toss_trace_recorder.py::_d_link` (lines 1438–1440) whitelists exactly six keys —
   `uptime_ms`, `mpc_active`, `teensy_mpc_active`, `time_synced`, `bus1_health`,
   `bus2_health` — and drops everything else. `FW-1` is **not scorable from a trace
   file at all**, no matter when the recorder starts. Either add the key, or say in
   the run sheet that `FW-1` is a bag/log check.
4. **Capture sequencing killed `LG-5`'s own instrument.** The CAP-GATE recorder was
   started at 15:35:56.9 — **2 min 34 s after** LG-1's refusal and **3 min 4 s
   after** the loaded-flip. `LG-5` is the one check the unit-test suite structurally
   cannot reach. It cost nothing only because `/trajectory/status`, `/robot_state`
   and `/gravity_offset` were all in the bag from the start of the 15:23 recording.
   **Start the recorder before the `LG-1` goal, not after the level.**
5. **`catch_reach_replay.py:354` pins `THROUGH_SEAT_RATE_RADPS = 0.07`** as a capture
   record, and `build_replay` passes it explicitly — deliberately rebuilding the
   **pre-fix** plan. This sitting ran at the shipped `0.0`, so model and echo can
   never agree; the `rx` residual of 2.7815° on toss 1 is exactly the arrival-twist
   term the model adds and the machine did not. **Do not report that as a machine
   finding.** The module docstring's claim that "every VERDICT below is unchanged"
   is true for pre-2026-07-26 captures and **false** for post-ZSEAT ones. Needs a
   `--session-rate` override (or to infer the rate from the capture) before
   `CATCH-2`, and anything read off it, can be scored on a ZSEAT-era bag.
6. **`ZSEAT-4`'s Tier-8b rows are mutually unsatisfiable.** Its first row (flat over
   `release ± 0.10 s`) and its third (Tier 8b publishes the reach **at** `t_release`)
   cannot both hold: a reach that starts at release necessarily puts motion into the
   second half of a symmetric window. Arithmetic confirms it — a min-jerk 51.54 mm
   reach starting `+0.017 s` after release travels `10s³−15s⁴+6s⁵` at
   `s = (0.100−0.017)/0.903 = 0.0919` → **0.347 mm predicted vs 0.334 mm observed**.
   Narrow the 8b window to `[release−0.10, release]`.
7. **Hand rows 1 / 2 / H2.2 / H4.6 fire on the gated catch arm's own prelude** when
   it lands within 50 ms of the *modelled* stroke end (`_TRUNC_SCAN_MARGIN_S = 0.050`).
   Proof it is an artefact: tosses 21/22/23 have a physically identical seed
   (`9.8036/9.8012/9.8029 rev`, same ~0.156 rev gravity sag, same target,
   `|cmd−meas| ≈ 0`) and print clean, purely because their arm landed 52–62 ms after
   the modelled end. **Fix: require `pos_cmd` to still be short of x3 at the seed
   instant before calling it a truncation.**
8. **The reload coordinator scores a BB-aborted throw as `MISSED`.** Goals 9 and 18
   emitted `Reload MISSED` although `bb/throw: THROW_ABORTED_NOT_SETTLED (axis=YAW)`
   means no ball ever left the BB. Anyone computing the `ZSEAT-2` rate from outcome
   lines alone gets **13/18 = 0.72** instead of **13/16 = 0.81**. Both pass here, but
   at a marginal sitting that inflated denominator pushes a PASS into the
   INCONCLUSIVE band. Recommend a distinct terminal outcome.
9. **`H4.9`'s wording should exclude a seed at the live position**, and **row 3
   should split** into a commanded reading (which passes: max commanded anywhere
   `10.2259 rev`) and a measured end-stop reading with a **velocity-dependent** bound.
10. **The probe's dip window opens at the coasting peak**, so it cannot see the
    post-stroke **sag**: on all five 0.6 m throws the hand sits `0.151–0.159 rev
    (4.8–5.0 mm)` below x3 for ~30–60 ms while `pos_cmd` holds x3, and the gated row
    still reads `0.000`. That is tracking droop, not a commanded yank, and still
    2–11× smaller than the pre-fix dip — but the gated row would not catch it if it
    grew.
11. **`LVL-4`'s Platform-vs-Base reader is unusable on this session's bags beyond
    the first ~90 s** — the bolted-down Base body's own solution swings 6.75° pk-pk.
    Use the **world-frame Platform orientation**; the reader should gain a
    Base-stationarity sanity print.

### Instrumentation the sitting lacked

- **Nothing asks for the ball's measured arrival point.** The mocap `(x, y)` at
  `z = 950 mm` is the single number that made the drop analysis possible, and it
  appears in no log line today. Log it next to every reload outcome.
- **Nothing asks for a BB burn-in before scoring.** ~6 throws with the robot parked,
  confirming from mocap that arrival `x` has plateaued (successive-throw change
  < 10 mm) before counting attempt 1.
- **The instrumented catching cone contributed nothing**: `/cone/catch_event` has
  **0** messages in the bag and the `Catching_Cone` rigid body reads NaN throughout,
  while `/cone/heartbeat` ran at 6022 messages. It would have settled the ball-11
  contact question outright.
- **Mocap cannot always see a seated ball**: 4 of the 13 catches show no marker at
  the cup afterwards, yet no floor arrival either — pose-dependent occlusion. Cup
  occupancy alone would have under-counted catches as **9/16 = 0.5625** and produced
  a spurious rate-row ABORT; the floor census is the reliable discriminator.
- **Tracker verdicts have zero information content this sitting**: all 17 tosses
  logged `Toss MISSED` and all 18 reloads `Reload MISSED`, i.e. **0/13 true
  positives**, with 35 `tracker CAUGHT at (…) is IMPLAUSIBLE` lines. New lead for
  the tracker investigation: the implausible positions cluster tightly at
  approximately `(−630, −348, −480 … −700) mm` — a **fixed spurious locus**, not
  random scatter, and the mocap has no unlabelled marker anywhere near there, so it
  is a **tracker-internal artefact**. The corruption is confined to **Z**: self-toss
  lines read `xy err 0–4 mm` against `z err 305–1007 mm`, so the *lateral* catch
  error is recoverable and already sits inside `CCATCH-2t`'s `< 10 mm` gate — but
  `CCATCH-2t` has not been formally scored, and any figure from it must be quoted
  with the can-bridge uptime beside it.
- The **Teensy serial** was not captured, so `H1.6`/`H4.7` are scored indirectly.
- **ERR_TIMEOUT epidemic gauge** (no runbook threshold; data for the separate
  investigation): over the 592 s stage-6 trace, 11× `HAND_TRAJ_CMD: ERR_TIMEOUT` and
  5× `hand prime dispatch failed` against 14 successes — **11/25 = 44 % of prime
  acks reported failure** — and 53 `Arming hand catch` dispatches vs 24 `armed on
  Teensy` acks. Measured telemetry shows the hand primed and stroking on **all 16**
  reload attempts, so **every one of those 11 ERR_TIMEOUTs was a false failure**.
  Zero functional impact this sitting. Consistent with the known ~59 %-both-ways ack
  behaviour.
- **BB `YAW NOT_SETTLED` occurred twice** (balls 31 and 62), matching the open item
  already in memory.
- **Two `ABORTED_NO_RELEASE`** in the extremity session (goals `(150,150)` and
  `(−150,−150)`, both tof 0.903). **Not consecutive** — successful throws separated
  them — so the phase-8 runbook's "two consecutive ⇒ stop" trip did not fire.

### Build state changed mid-sitting

`JB_OP_TOSS_TIER` went `8a → 8b`: `config/generated` and
`ros_ws/src/jugglebot/jugglebot/hardware_config.py` were regenerated at **15:59:44**
and the install tree rebuilt (`install/…/toss_sequencer.py` mtime **16:07:19**).

**CAP-WORK (15:39:50–15:49:42) and the stage-4/5 gate captures therefore ran on
Tier 8a**; only the 16:00:27 and 16:07:29 T-rung launches ran on 8b. Any
like-for-like comparison between CAP-WORK tosses and the T-rung ladder **crosses a
build boundary**, and any offline replay of CAP-WORK must be scored from a clean 8a
tree (`git worktree add` at `ac05848`) because
`tools/probes/catch_reach_replay.py:322` imports `jugglebot.hardware_config`.

### The phase-8 T-rung session, and the Tier-8b decision

- **15-51-24** — 20 Tier-8a tosses at `(0,0)×1, (60,0)×1, (150,0)×3, (150,150)×5,
  (−150,150)×3, (−150,−150)×4, (150,−150)×3`, tof 0.700 (first 7) then 0.903. This
  is the extremity data above. 20 `MISSED` (tracker corruption), 2
  `ABORTED_NO_RELEASE`, 1 `REJECTED_WORKSPACE` (the operator finding the Tier-8a
  positioning envelope).
- **16-00-27 — the successful T4**: **11 Tier-8b displaced throws** from `(0,0)` to
  `(50,0)`, `(70,0)`, `(−70,0)×4`, `(0,−70)×3`, `(0,70)×2` — **all accepted OK**, at
  the 70 mm Tier-8b cap. The deferred A→B reach fired correctly on every one
  (`ZSEAT-4` Tier-8b rows above: 11/11 exactly still pre-release, dispatch
  `+0.013…+0.050 s` after release, never early).
- **16-07-30** — four Tier-8b attempts at `(100,100)`, `(90,90)`, `(80,80)×2`, **all
  four refused** with `WORKSPACE: catch target 146/131/117/117 mm from the armed hold
  pose exceeds the 80 mm reach envelope` — the operator probing past the diagonal
  envelope (`hypot(80,80) = 113 mm > 80 mm`). No reach ever ran in that bag.

On that evidence the operator enabled **Tier 8b as the default**
(`jugglebot_operational.toss_tier: "8a" → "8b"`). That change is **not yet
committed** — see § Open Questions.

## Discussion

### What this sitting settles

**The levelling frame contract is correct on hardware, at every level it can be
observed.** `LVL-3` passes on all three stages with `err ±0.0000°`; the mirror bug
(a double-applied correction reading ≈−1.5576°) is ruled out on both the goal path
and the activate seed; and the mocap world-frame step validates the whole chain
end-to-end at **0.6 % / 2.2 %** against a *known* commanded rotation. That last
figure matters more than it looks: `LVL-2`, `LVL-3` and `levelling_tilt_bag_check.py`
all share the same FK, so a wrong FK would be a shared-mode failure that every
FK-based row would pass. Mocap is the only witness that would notice, and it agrees.

**The levelling gate works across processes, which is the only place it could have
failed.** Every unit test behind Phase 3 runs on mocked ROS and is *blind to message
choreography by construction* (a lesson this project has already paid for). `LG-5`
reconstructed shows the full chain — orchestrator publish → `/gravity_offset` on the
wire → trajectory_node `gravity correction set` → `/trajectory/status` flip →
`reload_coordinator` CHECKING pass — in the right order, on one clock, with
`+30.401 s` and `+149.778 s` of margin. The refusal itself was **inert**: zero leg
motion, zero hand motion, terminated in ~0.32 s.

**The FK criterion change at `3415617` is clean.** 105,318 solves, zero failures,
across both the commanded and measured streams and including the four extremity
poses and the two out-of-envelope 8b attempts — i.e. no spurious refusals anywhere
near the workspace edge.

**`C-CATCH-1` is in force and the through-seat residual is gone.** The counterfactual
puts a number on what was removed: `+0.297780°` of residual and a wrong-side
excursion of `0.3864–0.4344°` → `0.0000°`, with `peak_leg_*` falling from ≈142/≈3950
to `0.0`.

**The hand-command-continuity set is the cleanest result of the sitting.** The
post-throw dip is gone — **0.000–0.026 rev on 15 of 17 tosses against a pre-fix
0.339–1.748 rev (10.7–55.3 mm)**, a 40–70× reduction — and the mechanism is
*verified rather than inferred*: on all 17 tosses the commanded stroke reached x3
**before** the catch arm's command landed, with 28.0–61.6 ms of margin, and the
commanded velocity never went negative and the commanded position never went below
x3 between release and the arm. There is no commanded yank of any size anywhere in
the capture. The arm gate is textbook (17 latched = 17 tosses, 17 withheld, 0
CLOSED, every withheld redeemed), the derived prime lands with essentially zero
scatter (`9.9571–9.9586 rev`, spread 0.05 mm) and zero overshoot, and Phase 4's
velocity-continuous branch **fired on hardware for the first time** on 4 of 17
tosses, behaving exactly as modelled.

That last point deserves emphasis because the runbook says this branch has "no row
that provokes it". In fact **a fast throw provokes it every time**, because the
deferred arm lands while the hand is still falling back from the coast. The branch
is not an edge case; it is the normal path above ~0.78 m.

### What it leaves open, and the two things that should change behaviour

**`ZSEAT-2` is the headline, and it is genuinely undecided.** The rate arm passes
comfortably (0.8125 vs a 0.63 gate) and the flatness arm proves the zero seat really
shipped, so the experiment measured what it was supposed to measure. But the
bounce-out arm trips **both** of its ABORT clauses, and the runbook's own wording
for that outcome is *"a stationary tilted rim deflects the ball on hardware"* — which
is the bb-sim geometry finding, confirmed.

The reason it must nonetheless be logged as **INCONCLUSIVE on attribution** rather
than as a clean confirmation is that the same capture contains a second measured
cause: a monotonic ~26–39 mm BB warm-up drift in arrival `x` that plateaus around
the sixth throw. The temptation is to treat the drift as an *alternative* to the
seat and close the question — and that would be wrong, for a reason worth writing
down because it is the sort of thing a future session will re-derive badly:

> A ~30 mm off-centre arrival on a stationary 10.8° rim is **a description of the
> disturbance the seat existed to reject**, not a competing explanation for the
> drop. The correct null is not "a seat defect should not heal after three
> attempts". The seat did not heal; the disturbance shrank.

And the marginality is measured: attempt 3 dropped at `x = −10.7` and attempt 4
caught at `x = −14.5`, **3.8 mm apart**. The capture-basin edge sits exactly where
the warm-up excursion was crossing, which is precisely the regime in which a 3 mm
difference decides the outcome and neither hypothesis can be falsified.

This capture cannot separate them because it contains **no throws at ~+30 mm offset
with a non-zero seat**. The experiment that settles it is cheap and specific: burn
in the BB (~6 throws, robot parked, confirm from mocap that arrival `x` has plateaued
to < 10 mm successive change) *before* counting attempt 1; then with aim settled,
deliberately bias the BB **+30 mm in x** for ~6 throws at seat rate `0.0`, and repeat
the same 6 at `0.07 rad/s`. If `0.07` recovers catches that `0.0` loses at the same
offset, the seat's value is demonstrated on hardware **for the first time**; if not,
the lever is BB aim repeatability or cup geometry, not the seat rate.

**The hand's end-stop margin above 0.78 m is the one thing here that should stop a
future sitting.** The five off-run-sheet ~1.2 m tosses drove the **measured** peak to
`10.860–11.062 rev` — **1.2 mm from the declared 11.1 rev limit** — as pure
position-loop coast past a commanded profile that never left x3 (max commanded
anywhere `10.2259 rev`). The growth with commanded speed is steep and superlinear:
`+0.074 → +0.063 → +0.345 → +1.020 rev` at 2.74 / 3.44 / 3.97 / 4.86 m/s. Extrapolating
even conservatively, a **legal in-band** toss at the shipped `FLIGHT_TIME_MAX_S = 1.10 s`
(h ≈ 1.48 m, v ≈ 5.4 m/s) would command a throw whose coast exceeds the 355 mm stroke
top (11.22 rev). **The flight band and the hand's end-stop margin are not consistent
with each other above ~0.8 m.** The reversal at the peak is violent (next telemetry
sample −27 to −47 rev/s at `iq` 25.2 A, the session's maximum current), though 100 Hz
aliased telemetry cannot settle whether the hand touched its travel limit.

Worse, the anchor itself is ambiguous, and that matters when the measured peak is
1.2 mm away from it: `hardware_config.yaml` says *"hand true max ~11.4 rev"* for
`hand_motor_max_position_revs: 11.1`; geometry says `355 mm / 31.617 rev-per-m =
11.224 rev`; the runbook's `H4.5` says the hard stop is 0.76 mm above the guard.
Those give **11.124, 11.224 and 11.4 rev** — the answer moves the remaining margin by
~9 mm. **Pin which is true before the next high tosses.**

**The dispatch shift has outgrown the margin it was sized against.** `+54…+63 ms`
(bag clock) against a pre-fix `+12.8…+21.9 ms`, versus the 40 ms margin Phase 1's
stroke-busy window budgets — so the window, anchored on the announcement, now opens
early. The consequence is already visible: the minimum observed margin between
commanded stroke end and the arm command fell to **28.0 ms**. It tracks can-bridge
Teensy uptime almost perfectly (0.24 h → +14.5 ms, 0.54 h → +23.4 ms, 1.57–1.74 h →
+54…+63 ms), so this is the **known 2026-07-18 uptime-lag finding reaching the arm
gate** — not a new hand defect. It reinforces the standing rule: reboot the
can-bridge Teensy *immediately* before a sitting, not 1.5 h before, and log
`uptime_ms` with every timing number.

### `LG-3` — an unresolved disagreement about whether it can ever be run

`LG-3` is the check that distinguishes the shipped gate (which observes the
**applier**, `trajectory_node`) from the gate the plan originally specified (which
would have observed the **Teensy flag**). It requires `levelling_complete: true`
**and** `gravity_correction_loaded: false` **together**. The auto-push made that
state unreachable this sitting.

The two analysts who examined it **disagree on whether it is reachable at all**, and
the disagreement is recorded rather than adjudicated:

- One holds that the recipe exists: *"TO ACTUALLY RUN LG-3 you must power-cycle the
  PLATFORM Teensy (its cache is 'since last bootup'), not the can-bridge; then
  relaunch, do not level, arm, send the goal."*
- The other holds it is impossible: *"Its precondition is UNREACHABLE by any
  power-cycle… Power-cycling the Platform Teensy clears its levelling cache, which
  drives `levelling_complete` FALSE too — that produces LG-1's state, not LG-3's…
  the honest closure is a unit test that constructs a late-subscribing
  `trajectory_node`, not robot time. Do not spend a sitting trying to force it."*

Bearing on it, from this sitting's own evidence: the 15:23 launch came up with
`cold-start state (boot): is_homed=0 levelling=0 pose=(0.0000,0.0000)` — the flash
cleared the cache and **both** flags read false, which is `LG-1`'s state, not
`LG-3`'s. That observation is consistent with the second reading, but it was not
produced as a test of it and no one has attempted the first analyst's recipe. **Do
not spend robot time on `LG-3` until this is resolved on paper**; if the second
reading holds, the closure is a unit test, and it is cheap.

### Which skipped checks actually matter — the operator's question

Ordered by what a failure would cost, with the analysts' importance calls. Several
"MUST"s were subsequently *answered* by the offline analysis; those are marked so.

| check | status | call |
|---|---|---|
| **Ask the operator: bounce-out or missed arrival?** | **still open, perishable** | **MUST.** Trace inference says bounce-out (HIGH / HIGH / MEDIUM-HIGH). The operator's eye outranks it and decays by the hour. This is the input to the `ZSEAT-2` ABORT and to the one-line default change it routes to |
| **HAND-4 rows 3/4 on the ~1.2 m tosses** | **answered** (ABORT) | **MUST** — adjudicated above. Consequence stands: **no further tosses above 0.78 m** until the top margin is measured and the flight band re-examined |
| **`LVL-2` — first `go_home` after `level`** | **needs new capture** | **MUST.** No `go_home` was issued anywhere in the six launches, so the return-to-neutral install path has **never been observed** post-`C-LEVEL-1`. `LVL-3` already proves the activate seed is uncorrected and the goal-terminal park corrected exactly once, so the residual exposure is narrow — but it is **the first actuating move of every future sitting**, and a doubled correction there silently miscalibrates the neutral every subsequent goal is planned from. `predicted_mm = 203 × hypot(0.014, 0.002) = 2.87 mm`; ABORT above `max(1.8 × 2.87, 2.0) = 5.17 mm`. **~30 s of robot time — bundle it with `CCATCH-4`** |
| **`CCATCH-4` — no motion change outside the catch path** | **needs new capture** | **SHOULD.** Needs the same `go_home` plus a `go_to_pose`; zero marginal cost if run with `LVL-2` (same 60 s, same bag) |
| **`H2.2` seeds on `arms == 2`** | **answered** (PASS) | flagged MUST by one analyst before adjudication; the mechanism (a benign end-of-stroke reseed at `|cmd−meas| ≈ 0`, landing after `pos_cmd` reached x3) resolves it. The **criterion** still needs fixing |
| **`CCATCH-2` — the catch-reach headline** | **answered** (PASS on rows 1/2/3/5; row 4 INCONCLUSIVE) | flagged MUST before adjudication |
| **`ZSEAT-3` — reload-path counterfactual** | **not run** | **SHOULD.** Nothing yet confirms the reload catch ran the 2-segment zero-arrival-rate plan `ZSEAT-2`'s whole experiment assumes, and the `ry` settle is already flagged **outside** its ±0.02° band by −0.0460°. Blocked on the `catch_reach_replay` rate fix (defect 5); score from an 8a worktree |
| **`CATCH-2` — replay reproduces the machine** | **NOT-SCORABLE** | **SHOULD.** Cannot pass on any post-2026-07-26 capture until the probe gains a session-rate override. Until then, `CCATCH-2`, `CCATCH-3` and `ZSEAT-3` are all being read from a model nothing has validated against a post-fix capture |
| **`CCATCH-3` — reload-path blast radius** | **not run** | **SHOULD.** Same probe invocation as `ZSEAT-3`; the "N further catch install(s)" census is the only thing that bounds how many other install sites `C-CATCH-1` touched. Read its `⚠ SUPERSEDED` banner first — five of its rows now ABORT on correct behaviour |
| **`CCATCH-2t` — tracker catch error < 10 mm** | **partially recoverable** | **SHOULD.** The only *outcome* measure that `C-CATCH-1` improved real catch accuracy. The tracker channel is degraded, but the corruption is confined to Z, so the **lateral** half is readable and already 0–4 mm. Score the lateral half; record that the Z half is unmeasurable until the tracker investigation closes — otherwise the improvement claim rests on prediction alone |
| **`H2.4` / `H3.6` / `H4.8` — SAFE_ABORT** | **now scorable** | **SHOULD.** One occurred naturally in the 16:00:27 launch. The kind-3 retract ladder is the **only** un-arm mechanism the Teensy offers — a refused or skipped retract leaves an armed kind-0 catch stroke live on the board with nothing able to cancel it. Confirm the hand reached 0.0 rev |
| **`LG-5` literal single-trace run** | **does not need repeating** | the invariant is confirmed end-to-end from the bag; just start the recorder earlier so the runbook's own instrument works |
| **`FW-1`** | **does not need repeating** | passed on all six launches with three independent lines of evidence |
| **`FK-3`'s "to the last digit" arm** | **not closable from this sitting** | **SHOULD.** No pre-change reference print was captured and the T-rung prints are lost to the `output='screen'` gap |
| **`LG-3`** | **contested** | see above — resolve on paper before spending robot time |
| **`FK-4` — MPC hot loop untouched** | **moot here** | **SKIP.** Requires `run_mpc.py`, deliberately kept off (sole binder on :5557), and the MPC never ran this sitting. Score it on the next sitting that runs the MPC |
| **`CATCH-1` / `CATCH-3` / `LVL-5`** | **retired** | **SKIP.** Explicitly retired by the run sheet, and this sitting's data confirms it — `CATCH-3`'s table is a *pre*-fix reference, so scoring a post-fix capture against it would abort on correct behaviour |

## Verification

This entry and its sibling commits are **markdown + config only**. No `*.py`,
`*.h`, `*.ino` or `*.msg` file under any test path changed, so the full suite is not
the gate — but that justification is traced below, not asserted from the file
extension.

- **Logbook search surface** — `pytest tests/sim/test_logbook_search.py -q`, run
  2026-07-28: **24 passed in 0.19 s**. This suite *does* parse the real `logbook/`
  directory, so a logbook edit is inside the test surface. **Its two blind spots,
  and how they were closed by hand:**
  1. `sim/analysis/logbook_search.py` skips `INDEX.md` outright (`_SKIP_FILES`), so a
     broken INDEX row passes green — the new INDEX row was **read back and confirmed
     to render** as a 5-column table row consistent with its neighbours.
  2. `load_entries` silently `continue`s past any entry whose front matter lacks a
     `title`, so a malformed new entry passes green — this entry's front matter
     **carries a `title:` line**, and the entry was confirmed to be *loaded* (not
     skipped) by the reader rather than merely "not erroring".
- **Config codegen determinism** — `python config/generate_config.py`, run
  2026-07-28: re-running it over the operator's edited YAML produced **no further
  working-tree change** (`git status --porcelain` and `git diff --stat` identical
  before and after: the same 7 files, 7 insertions, 7 deletions). The six generated
  consumer copies are exactly the codegen output of the edited YAML.
- **The config change's own test surface, traced** —
  `grep -rln TOSS_TIER tests/ ros_ws/src/jugglebot/jugglebot/ controller/` names
  `tests/ros/test_toss_sequencer.py` and `tests/ros/test_toss_coordinator.py` as the
  only tests that read this key.
  `pytest tests/ros/test_toss_sequencer.py tests/ros/test_toss_coordinator.py -q`,
  run 2026-07-28: **2 failed, 182 passed in 6.17 s** at `toss_tier: 8b`, and
  **184 passed in 5.78 s** with the same command at `toss_tier: 8a` (the operator's
  change stashed, then restored and verified byte-identical against a saved patch).
  The tier flip is therefore the **sole** cause of both failures. See § Open Questions.
- Plan-table and runbook edits were confined to status/annotation rows; no criterion
  number was altered.

## Outcome

The eleven landed anomaly fixes are **validated on hardware** with the exceptions
recorded above. Concretely:

- `levelling-frame-contract` Phase 4 — **validated** on `LVL-1`, `LVL-3`, `LVL-4`,
  `LG-1`, `LG-2`, `LG-4`, `LG-5`; `LG-3` inconclusive-with-reason; `LVL-2` never run.
- `hand-command-continuity` Phase 5 — **validated** on the gate, prime, repack and
  dip rows; two peak rows ABORT on ballistic coast at and above 0.78 m; the dispatch
  shift ABORTs against Phase 1's margin.
- `catch-reach-degenerate-overshoot` Phase 4 — `CCATCH-2`/`CCATCH-5`/`ZSEAT-4`
  validated; **`ZSEAT-2` ABORT**, attribution inconclusive.
- `fk-convergence-tolerance` — 0 failures in 105,318 solves.

Two defects found that nobody was looking for: the **int16-milliradian truncation**
of the persisted levelling offset, and the **hand end-stop margin** above 0.78 m.
Eleven instrument/runbook defects surfaced, several of which would produce a false
ABORT on a healthy machine.

## Withdrawn claims

- [2026-07-28] Framing carried into the analysis that the CAP-GATE trace would
  contain the `LG-1` refusal and the `level`.
  WITHDRAWN: the recorder was started at 15:35:56.9, **2 min 34 s after** the refusal
  and **3 min 4 s after** the loaded-flip, so the trace contains only `LG-2`.
  Superseded by: `LG-1`, `LVL-1` and `LG-5` scored from rosbag
  `2026-07-27_15-23-27` plus the per-node logs — all of it intact and clean. See
  § Diagnosis and § Instrument defects item 4.
- [2026-07-28] Framing carried into the analysis that no rosbags covered stages 4
  and 5.
  WITHDRAWN: `~/Desktop/rosbags/2026-07-27_15-23-27` (264 MB) and
  `2026-07-27_15-37-50` (33 MB) bracket both gate captures, and the 15-23-27 bag is
  the **only** artefact holding the freshly-measured, unquantised levelling offset —
  which is what made the truncation defect findable.
  Superseded by: § Diagnosis, bag table.
- [2026-07-28] One analyst read the sitting brief's stage-7 description as a
  *total* and reported *"THE PROMPT'S STAGE-7 DESCRIPTION IS WRONG… 18 toss goals at
  three distinct release speeds, not 'two 0.38 m throws and one attempted 0.1 m
  throw'."*
  WITHDRAWN as a contradiction: the brief described **stage 7 only**. Three other
  analysts independently resolved the trace to **17 executed self-tosses at four
  tiers** (`0.700 ×5`, `0.808 ×5`, `0.989 ×5`, `0.557 ×2`) plus one refused goal —
  the `0.557 s` pair *is* the two 0.38 m throws and the refused goal *is* the 0.1 m
  attempt; the other 15 are stage 6's requirement. Both counts agree at 18 goals.
  Superseded by: § Diagnosis, "What was run".
- [2026-07-28] Reload-attempt denominator briefly reported as ambiguous between 16
  and 18 (18 `ball_butler` throw announcements in the bag vs the operator's 16).
  WITHDRAWN: reconciled — the two extra announcements are goals 9 and 18, which
  announced balls 31 and 62 but logged `bb/throw: THROW_ABORTED_NOT_SETTLED
  (axis=YAW)`, so no ball left the BB.
  Superseded by: `ZSEAT-2 rate` row (13/16 = 0.8125). Both denominators clear the
  0.63 gate (13/18 = 0.72), so no verdict changes.

## Open Questions

1. **The Tier-8b config change is NOT committed, and committing it as-is turns the
   branch red.** `jugglebot_operational.toss_tier: "8a" → "8b"` sits uncommitted in
   the working tree with its six regenerated consumer copies. It breaks exactly two
   tests, both test-side, both a direct and intended consequence of the flip:
   - `tests/ros/test_toss_sequencer.py::test_tier_constant_matches_config` asserts
     `JB_OP_TOSS_TIER == TIER_8A`. Its own docstring states the intent is that the
     config must name *a tier the FSM implements*, and the sibling test's docstring
     already says *"the serviceable set is {8a, 8b}"* — so the guard wants widening
     to the serviceable set, not re-pinning to a new single value (re-pinning fires
     again the next time the operator flips it).
   - `tests/ros/test_toss_coordinator.py::test_toss_goal_rejections_via_execute[workspace-REJECTED_WORKSPACE]`
     now returns `REJECTED_DISPLACEMENT`. Mechanism: under Tier 8b the displacement
     gate at `toss_sequencer.py:654-669` runs **before** the WORKSPACE check at
     `:680`, and the case's `x=200 mm` goal exceeds `TOSS_MAX_DISPLACEMENT_MM = 70.0`
     against throw site `A = (0,0)`. The ordering is **intended and documented
     in-code** ("the cap first (the primary contract) … both are loud PRE-THROW
     verdicts"). The case needs to pin the tier it was written for.
   Both are drift guards doing their job. Neither is a production bug. The fix is a
   `*.py` test edit and was out of scope for the session that found it.
2. **Ask the operator, today**: did the three dropped balls touch the cup and leave?
3. **Pin the hand's true stroke limit** (11.124 vs 11.224 vs 11.4 rev) before any
   toss above 0.78 m, and reconcile `FLIGHT_TIME_MAX_S = 1.10 s` with the measured
   end-stop margin.
4. **Fix the int16-milliradian truncation** (`lroundf`, or more bits) — and note that
   `LVL-3` is structurally blind to it, so the fix needs its own check.
5. **Run the seat A/B**: BB burn-in, then ~6 throws biased +30 mm in x at seat rate
   `0.0`, then the same 6 at `0.07 rad/s`.
6. **Resolve `LG-3` on paper** before spending robot time on it.
7. **Give `catch_reach_replay.py` a session-rate override** — `CATCH-2`, `CCATCH-3`
   and `ZSEAT-3` are all blocked behind it.
8. **Read the on-board inclinometer at the six extremity poses** to separate a
   physical tilt from a repeatable mocap bias, before building any offset table.
9. **Log the ball's mocap `(x, y)` at `z = 950 mm`** next to every reload outcome, and
   add a BB burn-in step before scoring.
10. The `toss_tier` YAML comment still reads *"Tier 8b … is REJECTED_TIER until the
    plan's Phase 4 lands behind this same key"*. `single-ball-toss` Phase 4 **is**
    COMPLETE (2026-07-25), so the sentence's condition is satisfied and the `8b`
    value is coherent — but the wording now reads as stale and wants a tidy when the
    change lands.

## Related

- Plans: `plans/active/PROMPT-anomaly-fixes-orchestration.md` (umbrella),
  `plans/active/levelling-frame-contract.md` (Phase 4),
  `plans/active/hand-command-continuity.md` (Phase 5),
  `plans/active/catch-reach-degenerate-overshoot.md` (Phase 4),
  `plans/active/fk-convergence-tolerance.md`,
  `plans/active/single-ball-toss.md` (Phase 5 T-rungs).
- Runbooks: `tests/hardware/session_anomaly_fixes.md` (§ THE RUN SHEET — the executed
  document), `tests/hardware/session_phase8_toss_hardware.md` (the T-rung ladder run
  afterwards).
- Contracts: `controller/REFERENCE_LAYER_CONTRACT.md`,
  `ros_ws/docs/levelling_frame.md` (C-LEVEL-1),
  `ros_ws/docs/catch_arrival_contract.md` (C-CATCH-1),
  `ros_ws/docs/hand_command_continuity.md`,
  `ros_ws/docs/platform_fw_version.md`.
- Prior entries: `logbook/2026-07-27-platform-teensy-fw-version.md`,
  `logbook/2026-07-27-anomaly-run-closeout.md`,
  `logbook/2026-07-27-velocity-continuous-prelude.md`.
- Open siblings: the ERR_TIMEOUT ack epidemic, the tracker split-track corruption
  (2026-07-23 finding), and the 2026-07-18 can-bridge-uptime lag finding — all three
  are visible in this sitting's data and none is owned by this entry.
