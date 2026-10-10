---
title: BB base-marker frame hardened after its first sittings — identification searches every marker but BB's own (a QTM label grab no longer blinds it), a missing base leads the refusal instead of `bb_moved`, and κ is pooled robustly (per-source MAD clip) behind a tighter gate (Δκ floor 0.15° → 0.09°)
type: bugfix
date: 2026-10-10
status: resolved
phase: "two-ball-skill-stack — R5 (Ball Butler calibration)"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/bb_base_frame.py
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
  - ros_ws/src/jugglebot/resources/bb_base_frame.json
  - tools/bb_base_frame_build.py
  - tests/ros/test_bb_base_frame.py
  - tests/ros/test_mocap_node_base_frame.py
  - tests/ros/test_mocap_node_keep_last_good.py
  - tests/ros/test_mocap_node_yaw_gate.py
  - tests/ros/test_perception_console_lines.py
  - logbook/2026-10-10-bb-base-frame-robustness.md
  - logbook/2026-10-10-bb-base-marker-frame.md
  - logbook/INDEX.md
subsystem:
  - tracking
  - ros
tags:
  - testing
---

# BB base-marker frame robustness

## Problem

Sittings 2026-10-10 14:10 and 14:13 (bags `2026-10-10_14-10-50`, `2026-10-10_14-13-18`; logs `python3_2002263_1791601853452.log`, `python3_2004280_1791602002304.log`):

1. **A label grab blinded the base.** The owner re-enabled QTM's Platform and Base bodies, and QTM labelled two of the four shelf markers (`Base - 2/3`, later `Catching Cone - 4/5`). The identifier only read *unlabelled* markers, so it saw `0 of 805 frames`. `auto` fell back to the sweep without saying so. The owner's routine QTM transform (+0.33°, 31 mm) then tripped the world gate: `CALIBRATION_INCONSISTENT … set bb_moved:=true if BB or QTM moved`. The missing base appeared only at DEBUG.
2. **The base gate was loose.** Its Δκ floor of ±0.15° compares with a per-sweep scatter of 0.030° (stamped) / 0.050° (heartbeat). Two stamped sweeps +0.07° / +0.09° off the median entered the pool (`02:47:06Z` +179.5735°, `03:11:24Z` +179.5911°).

## Analysis (scripts and their `.txt` outputs in `~/bb_calibration_sessions/base_frame_robustness_20261010/`; PREREG.md written before any code change)

- **Identification among all markers but BB's** (`ident_check.py`, `ident_segments.py`; 17 249 + 10 861 distinct frames, up to 27 / 22 candidate markers per frame):
  - **Uniqueness:** an exhaustive check of every 5th frame (3 426 + 2 147 frames) found exactly one 4-subset matching the L in every frame (one 14:10 frame had none: only three markers visible) and **0 decoy triangles**.
  - **Frames posed:** 14:10 from 76 s, 104 → 7 305 of 7 307 frames with markers; 14:13 before 50 s, 0 → 5 794 of 5 794.
  - **Labelled and unlabelled base markers give the same pose.** 14:13: −178.7296° vs −178.7300°, origins within 0.01 mm.
  - **Off-median poses are QTM glitches, not decoys.** Every pose > 0.05° / 1 mm from its time segment's median is one frame of a reconstruction glitch on the same four markers (rms 0.4–0.6 mm). The glitches occur with unlabelled-only sets too, and `estimate_base_pose`'s existing median filter drops them. The one large step in the 14:10 bag (106.5 s: 31.3 mm, +0.33°) is the owner's QTM transform.
- **The 22 pooled κ** (`gate_replay.py`, state as of 03:15:02Z):
  - **Per-source spread.** Heartbeat 1.4826·MAD = 0.066°. Stamped 1.4826·MAD = 0.016°, against SD 0.034° including the two suspects. The stamped SD without them agrees: 0.020° (13:45 sitting less +179.5735) and 0.021° (12:26 offline stamped replay).
  - **Old gate.** Replaying each record in order against the pool before it, it accepted all 22.
  - **Pre-registered new gate.** Limit max(3·√(σ_src² + SE²), 0.09°) with σ_stamped = 0.030°. It **also accepts both suspects**, narrowly: Δ +0.0817° vs a limit of 0.0974° (02:47:06Z), and Δ +0.0937° vs 0.0947° (03:11:24Z). See Discussion.
  - **Robust pool.** It excludes exactly the two suspects and nothing else.

## Discussion

- **The robust pool keeps the two suspects out, not the gate.** PREREG's honesty clause holds: the thresholds were not re-tuned to refuse the two named records. With σ_stamped = 0.020° (the clean value the pool floor uses), the stamped limit would drop to the 0.09° floor. That would refuse 03:11:24Z (Δ 0.0937°) but still not 02:47:06Z (Δ 0.0817°), and would refuse nothing else among the 22. That is the owner's call: a tighter gate means more re-sweeps if the stamped σ really is 0.030°. The pool exclusion does the job either way, because a flagged record's κ no longer enters the published pose.
- **The pool floor is per yaw source.** PREREG first had a single 0.020° floor. A suite test's synthetic heartbeat seed (SD 0.04°, 10 records) lost a legitimate record that way, because a 10-sample MAD reads low and 0.020° is a stamped number. The floor is now 0.020° stamped and 0.050° heartbeat (PREREG amendment). The stated risk: if the stamped σ is really 0.030°, a good stamped record is excluded with p ≈ 4.6 % instead of 0.3 %. That costs only efficiency, since exclusion is symmetric and the record stays in the file.
- **The base-led refusal is narrow.** It fires only under `auto` (or `base_frame`) with the model loaded and the pool at `bb_base_min_sweeps`, i.e. when the base would have decided the sweep, and only for `CALIBRATION_INCONSISTENT`. In every other case `bb_moved` is still the advice: `sweep` mode, no model, a pool below the minimum, or the base seen with BB-in-base changed (`BB_IN_BASE_MOVED`). A missing base under `auto` is now also a WARN line on an accepted sweep.
- **Only BB's own markers are left out.** They turn with yaw, so they can never be static shelf markers, and a BB-labelled copy of the L would be a decoy the identifier would have to break a tie on (pinned by a test). Every other body's labels are ignored.

## Change

- `bb_base_frame.py`:
  - `base_candidate_points(unlabelled, labelled)`: every marker except the `Ball Butler` / `Ball_Butler` bodies.
  - The robust pool: `kappa_outliers`, the yaw source per record (`kappa_record_yaw_source`) and `kappa_sweep_sd_deg` ({stamped 0.030°, heartbeat 0.050°}). `pool_kappa` takes the mean of the kept records, and `PooledKappa` gains `n_total`, `outliers` and `count_note()` (`n 20 of 22, 2 outliers excluded`).
  - Gate floor `KAPPA_GATE_MIN_DEG` 0.15 → 0.09, with σ = the new sweep's per-source SD.
  - `pool_summary`, and the state file gains a derived `pool` block. All records are kept.
- `mocap_node`:
  - feeds `base_candidate_points(unlabelled, labelled)` to the window and to the 5 Hz monitor (`_calib_unlabelled` → `_calib_base_points`);
  - the gate's σ is the per-source SD;
  - the outcome line carries the pool count;
  - adds `_base_expected`, the base-led `CALIBRATION_INCONSISTENT` text and the auto-fallback WARN.
- `tools/bb_base_frame_build.py`:
  - `--bag` reads every marker but BB's;
  - the resource lists `n_records` and `outliers`, and the build prints them.
- `bb_base_frame.json`, rebuilt `--from-state` on the 22-record snapshot:
  - 20 kept of 22;
  - **κ +179.4908° → +179.4974° (+0.0067°)**, SE 0.0083°;
  - p_b (177.537, 148.895, 76.640) mm.

  The live node pools the state file, so for it κ goes from the plain 22-record mean +179.5051° to +179.4974° (**−0.0077°**). Gauge continuity holds over the kept records (pinned by a test).

## Replay through the real `_finalize_calibration` (`node_replay2.py`; ROS mocked, both bags chained, auto/auto, pool = the 18 pre-14:10 records)

| sweep | HEAD 09fbd02f | this change |
|---|---|---|
| 14:10 #1 (pre-transform) | accept, base_frame, κ +179.5913 | accept, base_frame, κ +179.5913 |
| 14:10 #2 (post-transform) | REFUSE `CALIBRATION_INCONSISTENT` Δyaw +0.400°, 31.82 mm, "set bb_moved" | accept, base_frame, QTM frame shift 31.10 mm / +0.331°, κ +179.5754 |
| 14:10 #3 | REFUSE (Δyaw +0.301°) | accept, κ +179.4767 |
| 14:13 #1 | REFUSE (Δyaw +0.328°) | accept, κ +179.5031 |
| 14:13 #2–#4 | accept, κ +179.5122 / +179.4822 / +179.5012 | same κ |

- **The published offsets are stable.** After the transform, all 6 post-transform sweeps of the new path publish +0.767…+0.779° (±0.008–0.010°).
- **What the robust pool excludes.** At the end of the replay it excludes 02:47:06Z, plus the two replayed 14:10 sweeps #1 / #2 (+0.09° / +0.07° off the median). It keeps 22 of 25, with κ +179.4969°.
- **The failure wording on real data.** Run on the new code with the old unlabelled-only feed, the three sweeps the base cannot see are refused as `BASE_FRAME_NOT_SEEN: 0 of 805 frames posed the base markers (need 50) — check QTM sees the 4 shelf markers (occluded? taken into the Ball Butler body?); world gate also refused: Δyaw +0.41°, Δaxis 31.8 mm`. Each ERROR line carries `(kept the calibration from …)`, and there is no `bb_moved` advice.
- **Offline κ is not live κ.** The offline stamped κ differ from the live records by ≤ 0.015° (the bag has receive times, not the node's).

## Verification

- Scoped (`~/Desktop/PDJ_venv/venv/bin/python -m pytest -q tests/ros/test_bb_base_frame.py tests/ros/test_mocap_node_base_frame.py tests/ros/test_mocap_node_keep_last_good.py tests/ros/test_mocap_node_yaw_gate.py tests/ros/test_mocap_node.py tests/ros/test_bb_calibration_*.py tests/ros/test_perception_console_lines.py tests/ros/test_gui_bb_calibration_status.py tests/ros/test_ball_butler_node.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py tests/sim/test_plans_index.py`, run 2026-10-10): **405 passed, 1 skipped**. That includes 10 new tests. The world-gate and console tests (`test_mocap_node_keep_last_good`, `_yaw_gate`, `test_perception_console_lines`) now pin `bb_pose_source:=sweep` and a private base-state file. Under the default `auto` with the seeded pool, their base-less windows rightly lead with `BASE_FRAME_NOT_SEEN` / WARN. Before this change they also read the live `~/bb_calibration_sessions` base state.
- Full gate (`./run_tests.sh --full`, run 2026-10-10 15:34–15:44, alone on the box, log `temp/logs/full_gate_base_frame_robustness.log`): **PASS** — parallel 6500 passed, 9 skipped, 1 xfailed (361 s); serial 6 passed. The scoped run above on the same tree: 288 passed over the base-frame, mocap-node, constellation, console-line, GUI-status and logbook tests.
- Not verified:
  - live on the robot: the 5 Hz monitor with up to ~27 candidate markers on the loaded Jetson, and the WARN/ERROR wording in the GUI;
  - whether the stamped per-sweep σ is 0.020° or 0.030°, which decides whether the gate (not just the pool) should refuse a +0.09° sweep;
  - the live state file: it gained a 23rd record (03:49:36Z, κ +179.4773) during this work, under the old code. It is not in the resource.

external_changes: none (BallButler not edited).
