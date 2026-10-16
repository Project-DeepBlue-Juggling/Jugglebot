---
title: "BB calibration Phase 1 (gate inversion): the sweep is the output, history is advisory, kappa lives in never-discarded epochs, bb_moved is deleted; replay accepts 23 of 24 logged-refused sweeps"
type: investigation
date: 2026-10-16
status: open
phase: "two-ball-skill-stack - post-R6 (Ball Butler calibration)"
related_plan: bb-calibration-one-attempt.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/bb_base_frame.py
  - ros_ws/src/jugglebot/jugglebot/bb_calibration.py
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
  - ros_ws/src/jugglebot/resources/bb_marker_template.json
  - ros_ws/src/jugglebot/resources/bb_base_frame.json
  - tools/bb_base_frame_build.py
  - tools/probes/bb_calibration_replay.py
  - tools/probes/README.md
  - tests/ros/test_bb_base_frame.py
  - tests/ros/test_bb_calibration_constellation.py
  - tests/ros/test_bb_calibration_replay.py
  - tests/ros/test_mocap_node_base_frame.py
  - tests/ros/test_mocap_node_keep_last_good.py
  - tests/ros/test_mocap_node_yaw_gate.py
  - tests/ros/test_perception_console_lines.py
  - tests/hardware/session_bb_calibration_phase1.md
  - tests/hardware/session_skills_r5_sitting8.md
  - tests/hardware/session_bb_kappa_pin.md
  - plans/active/bb-calibration-one-attempt.md
  - logbook/2026-10-16-bb-calibration-phase1-gate-inversion.md
  - logbook/2026-10-15-bb-calibration-phase0-characterise.md
  - logbook/INDEX.md
subsystem:
  - tracking
  - ros
tags:
  - safety
  - kinematics
  - testing
---

# BB calibration Phase 1: gate inversion

## Summary

Phase 1 of `plans/active/bb-calibration-one-attempt.md`, consuming the Phase 0 band table. The sweep's own yaw offset and axis point are now always the output. Hard refusals remain only for intrinsic sweep quality (solver classes, constellation residual at the 1.0 mm cap). Every history comparison (kappa vs pool, yaw vs last accepted, template residual, lag, BB-in-base shift, QTM frame shift) became one advisory on the single outcome line. The kappa pool is kept in epochs and never discarded; `bb_moved`, `bb_pose_source` and `bb_base_min_sweeps` are deleted. Production-faithful replay of the recorded bags: 23 of the 24 replayable logged-refused sweeps are ACCEPTED; the one still refused (CIRCLE_CENTRES, 10-14 20:54 sweep 3) is refused on its own data. Code is done; no hardware sweep has run under Phase 1 yet (entry stays `open`).

## Symptoms

Phase 0 census (`2026-10-14-bb-calibration-one-attempt-investigation.md`, `2026-10-15-bb-calibration-phase0-characterise.md`): 25 of 42 sweeps refused 2026-10-11..14, 23 of them false positives (every BB_IN_BASE_MOVED, CALIBRATION_INCONSISTENT and TEMPLATE_RESIDUAL refusal). The remedy offered, `bb_moved`, destroyed the pool and dropped the node onto the world-frame gate, which refuses after every QTM re-alignment by construction.

## Diagnosis

The gates, by file (plan § 1, root causes C1-C6): `bb_base_frame.check_base_frame_consistency` (kappa vs pool at a floor-width 0.09 deg limit, BB-in-base shift; C2, C3), `bb_calibration.check_calibration_consistency` (yaw vs last accepted in QTM coordinates, template residual 0.50 mm; C4, C5), and the node's `bb_moved` arming plus pool reset (C3). All of them expressed one premise (C6): the pooled kappa is the output and the sweep only checks it. In the window they caught zero real moves and refused 23 good sweeps. The kappa step itself (C1) is not closed by this phase and does not need to be (plan § 4).

## Discussion

(i) Why the sweep is the output and history only advises (owner decision 1). A pooled estimate is precise about a reference that steps: when kappa stepped +0.25 deg the pool was a tight, confident estimate of a value that no longer held, and the gate enforced it. A fresh sweep measures what BB is now. The tradeoff accepted is pooled precision (within-sitting SD 0.044 deg) in exchange for immunity to steps; the throw learner absorbs 0.25 deg (about 4 mm at 1 m).

(ii) The nearest-epoch rule. A sweep joins the nearest epoch whose median is within 0.20 deg of its kappa (the current epoch first), else opens a new one. Two alternatives were considered. "Open a new epoch on every excursion" flip-flops on a transient: the 10-14 20:54 drift (-0.22 deg and back by 21:27) would open an epoch per sweep. "Require two consecutive excursions" delays the signal by a sweep and still flip-flops on alternating sittings. Nearest-epoch with the current epoch first is flip-proof: a transient that returns to a known median rejoins the old epoch (reported as a rejoin, still an advisory), nothing is discarded, and an epoch count grows only when kappa genuinely visits a new level.

(iii) Bands are fixed numbers, not n-sigma of a pool (root cause C2). The old limit was built from a sigma measured in one sitting and sat at its floor; cross-sitting repeatability was never characterised. The bands are the Phase 0 cross-sitting values (kappa +-0.20 deg per epoch, yaw +-0.20 deg within a world epoch, p_b 1.0 mm, template residual 0.90 mm below the 1.0 mm hard cap, |lag| 13 ms), each covering 100 % of the intrinsic-good population.

(iv) Protection given up. A genuinely bumped BB is now an advisory, not a refusal. The advisory is unmistakable: a 2 mm bump reads as 0.80 deg, four times the kappa band, and 2x the p_b band.

(v) Why the pose-source switch and `bb_moved` were deleted rather than left dormant. An unused mode is an untested mode, and the owner had no desire to keep `bb_moved` (2026-10-15). The pool-size mode switch (base path under 5 records, world path otherwise) was the mechanism that dropped the node onto the world-frame gate; with the sweep as the output there is nothing for it to select.

(vi) BASE_FRAME_RESIDUAL (shelf markers worse than 0.5 mm) stops refusing and instead stops pooling: the sweep is still the output, but its kappa is not trusted to join the history, so the line says "kappa not pooled".

(vii) The pin-correction bug the first Phase 1 replay exposed (run `20261016-110454`). The first epoch code compared raw kappa. Every pool record is stored with the gauge pin it was measured under and is comparable only after being moved to the current pin (kappa + pin_now - pin_rec, what the old `pool_kappa` did). The resource seed (22 records at pin 0.208, raw median 179.503) is 179.324 at today's pin 0.029, the 10-10 level, so that run opened a spurious epoch at 2026-10-10_12-26-22 sweep 6 and the real 10-13 step "rejoined" the seed. Fixed before commit (second run `20261016-112452`). Lesson: a cross-session comparison of kappa is a comparison at a stated pin, and a future re-pin from landings must not read as a BB move.

Orchestrator changes beyond the lib/node/replay handoffs: LAG is judged for the stamped yaw source only (the 13 ms band was characterised on 71 stamped sweeps; a heartbeat-sourced lag, about 80 ms, is shown in the suffix and not judged), and the pooled parenthetical names an opened or rejoined epoch instead of printing a delta.

## Fix

Files (from `git status --short`):

| file | change |
|---|---|
| `bb_base_frame.py` | schema-2 epoch state, `assign_epoch` / `append_record` / `assess_base_frame` (never refuses), `write_base_state` the only writer; gate and `bb_moved`-era symbols deleted |
| `bb_calibration.py` | `assess_sweep_vs_reference`, `format_outcome_body`, advisory bands; `check_calibration_consistency` and GateVerdict deleted |
| `mocap_node.py` | three params and their refusals deleted, `_load_state` / `_write_state` / `_log_startup_state`, one outcome line, persisted `epoch` + `advisory_flags` |
| `resources/bb_marker_template.json` | three prose fields (`gauge.repinned_on`, `how_to_adjust`, `rule`) that told the operator to set or avoid `bb_moved:=true` now say "nothing to arm: the sweep measures BB fresh"; no data field changed |
| `tools/probes/bb_calibration_replay.py` + `README.md` | Phase 1 path in the node's order, Phase 1 verdict columns, `--no-bb-moved` retired |
| `tests/ros/test_bb_base_frame.py`, `test_bb_calibration_constellation.py`, `test_bb_calibration_replay.py`, `test_mocap_node_base_frame.py`, `test_mocap_node_keep_last_good.py`, `test_mocap_node_yaw_gate.py`, `test_perception_console_lines.py` | rewritten for advisory behaviour; absence asserts for the deleted params |
| `tools/bb_base_frame_build.py`, `resources/bb_base_frame.json` | `--from-state` folds only the CURRENT epoch (a stepped history would give a bimodal seed); the resource description no longer names `bb_pose_source` (no data change) |
| `tests/hardware/session_skills_r5_sitting8.md` | row 6a: nothing to arm; repeated KAPPA_EXCURSION / YAW_SHIFT advisories are noted, not a stop |
| `tests/hardware/session_bb_calibration_phase1.md` | new runsheet (dress-rehearsal rows) |
| `tests/hardware/session_bb_kappa_pin.md` | deleted (pin dropped 2026-10-16, Phase 0 entry) |
| plan, INDEX, Phase 0 entry | status and cross-references |

Advisory table (flag, band, what the line says, remedy named):

| flag | band | line | remedy |
|---|---|---|---|
| KAPPA_EXCURSION | +-0.20 deg vs epoch median | "kappa +X deg from epoch Kn: opened epoch Km" / "rejoined epoch Kn" | none |
| YAW_SHIFT | +-0.20 deg vs last accepted, same world epoch only | "yaw +X deg vs the last sweep (advisory > 0.20)"; outside a world epoch the line reads "... is not comparable: the QTM frame moved", with no base seen "(frame unknown)" | none |
| BB_IN_BASE_SHIFT | 1.0 mm (p_b vs the epoch median) | "BB axis point moved X mm in the base frame (advisory > 1.0)" | none |
| TEMPLATE_RESIDUAL | 0.90 mm (hard cap 1.0 mm stays) | "template residual X mm (> 0.90)" | none named (the "re-wand QTM" clause was removed 2026-10-16: a fresh wand calibration left the residual at 0.35-0.46 mm, see Hardware reading) |
| LAG | 13 ms, stamped yaw source only | "lag +X ms (> 13)"; a heartbeat-sourced lag is shown in the sweep suffix and not judged | none |
| BASE_FRAME_RESIDUAL | shelf-marker fit 0.5 mm | "shelf markers fit X mm (> 0.5), base kappa not used - check QTM sees the 4 shelf markers" | check the four shelf markers are visible |
| QTM_FRAME_SHIFT | 2.0 mm or 0.05 deg base-pose jump vs base_ref | "QTM frame moved since the last sweep (X mm, Y deg); world deltas are not comparable" | none |
| BASE_NOT_SEEN / HISTORY_UNAVAILABLE | node-level | "shelf base not seen: kappa not pooled" / "history unavailable: <file>: <ErrClass>" | none |

State schema 2 in five lines: (1) top-level `schema: 2`, `epochs` (id, opened_at, reason seed/migrated/excursion, n, median, sd, median axis point), `records`, `base_ref`, derived `pool`. (2) Each record carries its `epoch` and the pin it was measured under. (3) Migration: a schema-1 file loads as epoch 0 reason `migrated`; no records means no epochs. (4) Backup: every write copies the previous file to `<path>.<stamp>[.<seq>].bak` (newest 20 kept) then writes `.tmp` and `os.replace`. (5) Cap: 200 records per epoch; epochs are never removed.

Outcome line (INFO if no flags, else WARN; example from the node test harness):
`BB calibrated: ... yaw +0.75° ±0.07, axis (-977, -389, 1737) mm, tilt 0.9°, template 0.25 mm, lag +8.0 ms · kappa 179.80° (epoch K1 +0.30°, n 1), p_b Δ0.0 mm · ADVISORY: kappa +0.30 deg from epoch K0: opened epoch K1 · sweep 0.0→0.0° 0.0 s, lag +8.0 ms`. The published form is `Calibration successful (accepted <ts>) · <body>`; the ` · sweep ...` suffix stays last.

Startup line: `BB calibration state: template bb_marker_template.json (7 markers), base model on, pool 10 records in 1 epochs (current K0 median 179.50° n 10), last accepted none, yaw source auto` (test-harness values).

### Replay verdict table

`python tools/probes/bb_calibration_replay.py -v` (2026-10-16, ~882 s), run `temp/probes/bb_calibration_replay/20261016-112452` (second run, after the pin fix; the first, `20261016-110454`, is superseded by the raw-kappa spurious epoch in § Discussion (vii)). Logged class -> Phase 1 verdict, flags, epoch:

| sweeps | logged -> Phase 1 | flags | epoch |
|---|---|---|---|
| 10-11_11-27-53#1, 10-11_22-15-28#1, 10-12_23-24-02#1, 10-13_00-18-46#1 | ACCEPTED -> ACCEPTED | QTM_FRAME_SHIFT | K0 |
| 10-11_15-14-11#1 | CONSTELLATION_TOO_FEW_MOVING -> ACCEPTED (the node-stall sweep) | - | K0 |
| 10-11_15-14-11#2, 10-12_23-24-02#2 | ACCEPTED -> ACCEPTED | - | K0 |
| 10-13_22-00-44#1 | BB_IN_BASE_MOVED -> ACCEPTED | QTM_FRAME_SHIFT; KAPPA_EXCURSION | K1 OPENED |
| 10-13_22-00-44#2-6, 10-13_22-29-08#1-6 | BB_IN_BASE_MOVED -> ACCEPTED | - | K1 |
| 10-13_22-29-08#7-8, 10-13_23-12-50#4 | ACCEPTED -> ACCEPTED | - | K1 |
| 10-13_23-12-50#1-3 | TEMPLATE_RESIDUAL -> ACCEPTED | - | K1 |
| 10-14_20-54-20#1 | CALIBRATION_INCONSISTENT -> ACCEPTED | QTM_FRAME_SHIFT | K1 |
| 10-14_20-54-20#2 | CALIBRATION_INCONSISTENT -> ACCEPTED | - | K1 |
| 10-14_20-54-20#3 | CIRCLE_CENTRES -> CIRCLE_CENTRES | - | - |
| 10-14_20-54-20#4 | TEMPLATE_RESIDUAL -> ACCEPTED | - | K1 |
| 10-14_20-54-20#5-7 | ACCEPTED -> ACCEPTED | - | K1 |
| 10-14_21-27-22#1 | CALIBRATION_INCONSISTENT -> ACCEPTED | QTM_FRAME_SHIFT | K1 |
| 10-14_21-27-22#2-3 | TEMPLATE_RESIDUAL -> ACCEPTED | - | K1 |
| 10-14_21-27-22#4 | TEMPLATE_RESIDUAL -> ACCEPTED | BASE_FRAME_RESIDUAL | not pooled |
| 10-14_21-27-22#5 | ACCEPTED -> ACCEPTED | - | K1 |
| 10-14_21-27-22#6 | ACCEPTED -> ACCEPTED | BASE_FRAME_RESIDUAL | not pooled |
| 10-14_21-56-45#1 | ACCEPTED -> ACCEPTED | - | K1 |

Totals: 79 sweeps from 21 bags; Phase 1 ACCEPTED 71, refused 8 (7 CONSTELLATION_MOCAP_CLOCK in 10-10_00-24-06, 1 CIRCLE_CENTRES). 185/185 measurements match (39/39 sweeps). 23 of 24 logged-refused sweeps ACCEPTED. Flags: QTM_FRAME_SHIFT 8, BASE_FRAME_RESIDUAL 2, KAPPA_EXCURSION 1; no YAW_SHIFT, TEMPLATE_RESIDUAL, LAG or BB_IN_BASE_SHIFT anywhere. Epochs: K1 opened at 2026-10-13_22-00-44#1 (the real step); no rejoin; no epoch opened on 10-10.

## Verification

- 2026-10-16, `pytest tests/ros/test_bb_base_frame.py tests/ros/test_bb_calibration_constellation.py tests/ros/test_bb_calibration_arc_span.py tests/ros/test_bb_calibration_consensus.py tests/ros/test_bb_calibration_coplanar.py -q` (venv): 139 passed, 1 skipped in 25.11 s (library).
- 2026-10-16, `pytest tests/ros/test_mocap_node*.py tests/ros/test_perception_console_lines.py tests/ros/test_gui_bb_calibration_status.py -q` (node unit): 101 passed in 9.66 s.
- 2026-10-16, `pytest tests/ros/test_bb_calibration_constellation.py tests/ros/test_mocap_node*.py tests/ros/test_perception_console_lines.py tests/ros/test_bb_base_frame.py -q` (after the lag-by-source and epoch-wording edits): 181 passed in 61.37 s.
- 2026-10-16, `pytest tests/ros/test_bb_base_frame.py tests/ros/test_bb_calibration_constellation.py tests/ros/test_mocap_node*.py tests/ros/test_bb_calibration_replay.py tests/ros/test_perception_console_lines.py -q`: 189 passed (after the pin fix).
- 2026-10-16, replay run 20261016-112452 (above): 185/185 measurements, 23 of 24 logged refusals accepted.
- 2026-10-16, `./run_tests.sh --full` (venv, log `temp/logs/run_tests_full_phase1.log`): parallel phase 6643 passed, 1 failed, 11 skipped in 290.43 s; serial phase 7 passed in 27.67 s. The failure is `tests/ros/test_replay_overview.py::test_a_pass_still_exiting_for_an_older_version_neither_reports_computing_nor_blocks_the_requeue` (GUI replay, untouched by this phase); scoped rerun 2026-10-16 `pytest tests/ros/test_replay_overview.py -q`: 11 passed in 21.61 s, classified a load flake.
- 2026-10-16, docs: `pytest tests/sim/test_logbook*.py tests/sim/test_plans_index.py -q` 108 passed, 2 skipped in 0.84 s.
- 2026-10-16, `./run_tests.sh` (default gate, after the audit fixes, log temp/logs/run_tests_precommit_phase1.log): 6644 passed, 11 skipped in 291.49 s; serial 4 passed in 16.64 s; exit 0.
- Audit (one pass at phase end, 2026-10-16): no blocking findings; one warning (the seed builder's --from-state folded every epoch into one pool - fixed to the current epoch) and six stale-text notes, all applied before the commit.

## Outcome

True now: every recorded refused sweep except the circle-centres one is accepted in replay; every advisory prints on one line; nothing resets the pool and every state write is backed up; `bb_moved` is gone (no `bb_moved` use in code; remaining mentions are absence assertions in two test files, the fixture's ignored stale keys, and logbook/plan history).

NOT yet true: no hardware sweep has run under Phase 1; the merge of `bb-calibration` into `skill-stack` and the `colcon build --packages-select jugglebot` are owed; plan acceptance 2 (first sweep of ten consecutive sittings accepted) needs ten sittings. The `./run_tests.sh --full` red was one GUI load flake, with a latent race in `ros_ws/gui/replay/api.py` (`_drain` -> `_ov_path`, TypeError on a None rid in the rename, surfaced as a PytestUnhandledThreadExceptionWarning) that is another subsystem and was not fixed here.

## Hardware reading (2026-10-16 sitting)

First hardware sitting under Phase 1 (launch 15:07, source: launch.log of that launch and `~/bb_calibration_sessions/` state files; replay of the two 10-16 bags exit 0, 9 sweeps all ACCEPTED, agrees with the live log to about 0.03 deg).

- **R1-R4 matched.** Startup line once: `BB calibration state: template bb_marker_template.json (7 markers), base model on, pool 8 records in 1 epochs (current K0 median 179.59 n 8), last accepted 2026-10-15T23:51:55 yaw +0.46, yaw source auto`. First sweep ACCEPTED: `BB calibrated: yaw +0.10 +-0.04 ... template 0.44 mm, lag +2.0 ms · kappa 179.23 (epoch K1 opened, n 1), p_b d0.7 mm · ADVISORY: yaw -0.36 deg vs the last sweep (advisory > 0.20); kappa -0.36 deg from epoch K0: opened epoch K1`. Two shelf-covered sweeps ACCEPTED "kappa not pooled"; the re-alignment sweep ACCEPTED with QTM_FRAME_SHIFT (213.4 mm, -11.74 deg) and yaw "not comparable". 7 of 7 accepted, zero operator action. Cosmetic: a 0.01 deg "(frame unknown)" yaw delta prints as an ADVISORY and makes the line WARN.
- **Epoch mechanism behaved as designed.** K1 opened on sweep 1 (KAPPA_EXCURSION, reason excursion); later sweeps joined K1 at +0.08 / +0.00 / +0.01 / +0.00; K1 n 5, median 179.2714, SD 0.028; kappa held through the 213 mm / -11.74 deg re-alignment (frame-invariant). State file schema 2 with 2 epochs, a schema-1 backup (8 records) written before the first write, K0's 8 migrated records intact, last-accepted file epoch 1 / flags [] / pose_source sweep.
- **Mechanism finding: the re-wand was not the cause.** The QTM re-wand was about 10:28 local. The handoff session's sweeps at 10:51 (old code) came after it and still read kappa 179.572 / 179.565 (template 0.353 / 0.350 mm). By 15:08 kappa was 179.25-179.28 with QTM unchanged (shelf pose 0.50 mm / 0.001 deg off the 10:51 base pose). Reconstruction drift is therefore RULED OUT. The two steps are equal and opposite (+0.31 deg on 10-13, -0.33 deg on 10-16), both between sittings: a two-state kappa (179.27 <-> 179.58). Leading candidates: something at BB that changes between sessions (every session boundary is normally a BB power cycle), or a per-session marker-labelling / template-fit artefact (only 5 of 7 markers label at rest and the set varies bag to bag). Phase 0's sweep-dynamics co-variation does not hold (the 10:51 high-kappa sweeps ended at 123.2 deg). The 10:51 and 15:08 bags share one wand calibration and one world frame, a controlled pair that Phase 2 measures first.
- **Template residual reference level.** 0.26 mm was one good 10-10 noon state; under the fresh wand calibration it read 0.35 (10:51) and 0.44-0.46 (15:08). The advisory's reference level is 0.35-0.46 mm, so the "re-wand QTM" remedy was removed from the advisory line (code edit in this commit). The shelf-marker (base) residual against the as-built template, 0.004 on 10-10, was 0.29-0.35 mm on both sides of the reversal and did not recover after the re-wand: a separate finding for Phase 2/3.
- **Acceptance 2 reading.** Process clause passes (accepted first time, no override, no operator action). The band clause fails as written: the sweep measured a real 0.33 deg change, 7.5x the within-sitting SD (0.044), against a +-0.20 deg band. Owner decision pending on the wording (plan § 0); under the reworded acceptance this is 1 of 10, under the literal wording 0 of 10.
- **Frame-queue drops (not Phase 1's).** The mocap node logs `N of M expected QTM frames lost (31-64 %) ... frame queue full` after every sweep and at idle; present since 10-13 and in the 10:51 sitting too. Every sweep passed the mocap-clock check. Open item. The parallel handoff session's tracker work (branch `tracker-fit-latency`, 2026-10-16) found the live ball tracker processing about a third of QTM frames and is reading these `frame queue full` lines against the `/balls` publish share as its first step; the loss is most likely one phenomenon.

2026-10-16 evening: the first hardware sitting passed R1-R4 (7 of 7 sweeps accepted, epoch K1 opened, tally 1 of 10 pending the acceptance rewording); the re-wand was ruled out as the kappa cause, see Hardware reading.

## Open Questions

- The two-state kappa (179.27 <-> 179.58, steps +0.31 on 10-13 and -0.33 on 10-16): BB state versus a per-session labelling/template artefact. Decided by Phase 2's controlled-pair measurement (10:51 vs 15:08 bags: common marker subset, label-free pickup).
- Was BB powered off between 10:58 and 15:05 on 2026-10-16? Owner question; confirms or breaks the session-boundary reading.
- The mocap node's QTM frame-queue drops (31-64 % after every sweep and at idle, since 10-13): cause unknown, not Phase 1's.
- The ~28 s node stall of 10-11 15:14 sweep 1: what blocked the node? Unchanged.
- The 10-14 20:54 transient (physical or measurement; measurement leading). Under Phase 1 it is a rejoin-capable advisory, not a refusal.
- The `_drain` / `_ov_path` race in the GUI replay overview (above): owner decision whether to reopen that closed arc.
