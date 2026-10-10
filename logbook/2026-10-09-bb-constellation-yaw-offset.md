---
title: Ball Butler's yaw offset and position come from the whole calibration sweep, with the heartbeat lag fitted per sweep and a fixed E(y), against a 7-marker body template; a consistency gate refuses a frame change unless bb_moved is set
type: feature
date: 2026-10-09
status: in-progress
phase: "two-ball-skill-stack — R5 (Ball Butler calibration)"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/bb_calibration.py
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
  - ros_ws/src/jugglebot/resources/bb_marker_template.json
  - ros_ws/docs/choreography.md
  - tests/ros/test_bb_calibration_constellation.py
  - tests/ros/test_mocap_node_yaw_gate.py
  - tests/ros/test_perception_console_lines.py
  - logbook/2026-10-09-bb-constellation-yaw-offset.md
  - logbook/INDEX.md
subsystem:
  - ros
  - tracking
tags:
  - kinematics
  - testing
---

# Ball Butler's yaw offset and position come from the whole calibration sweep, with the heartbeat lag fitted per sweep and a fixed E(y), against a 7-marker body template; a consistency gate refuses a frame change unless bb_moved is set

## Problem

`bb_calibration.calculate_yaw_offset` read BB's yaw offset from ONE marker (QTM 4) at 117.6 mm from the axis point that the sweep fitted, so 1 mm of axis error was 0.49° of yaw. Two calibrations on 2026-10-09 with BB untouched stored 0.208° (session A, `20261009T002142_931068Z`, the frame `throw_affine_correction.json` was fitted in) and 0.681° (session B, `20261009T031319_612936Z`); session B's throws landed −8 mm sideways at 1 m (`~/bb_calibration_sessions/yaw_offset_investigation_20261009/REPORT.md`).

## Superseded first design (same day, commit ceea98c7)

The first fix read the offset from the sweep's stationary yaw-0 pause only: per-marker medians of the hold, a label-free 2D match to a 4-marker co-planar template, σ floored at a 0.13° hold scatter, gauge pinned so the 7-sweep bag's pauses averaged 0.208°. It was no more repeatable per sweep (SD 0.095°) than the anchor (0.090°), it saw only 3 markers at the pause, and a second investigation (`~/bb_calibration_sessions/bb_placement_reinvestigation_20261009/REPORT.md`) showed its pin was taken on frames where the parked pose reconstructs non-rigidly. It also excluded the moving sweep on the strength of an "~80 ms" heartbeat lag treated as a constant, and dropped the off-plane markers QTM 1/2 on a claim ("they distort at yaw 0") that was an instrument effect at one pose. The owner rejected it. This entry replaces it; nothing of the pause estimator, its 0.13° floor, the 80 ms constant or the QTM 1/2 exclusion remains.

## Discussion

**Why the whole sweep.** The moving sweep has ~37 heartbeat samples spread over 0–125° per sweep, where the pause has one pose. What stood in the way was the heartbeat's lag: it is unstamped, and its yaw passes three free-running 10 Hz stages, so it lags the mocap frames by a per-session amount (80–85 ms in the 7-sweep bag, 94–97 ms in A). Fitting that lag per sweep is cheap and well conditioned: a lag error shows as ±ω·δτ with opposite signs on the outbound and return legs.

**Decisions (made before the work; recorded here):**
1. Estimator = whole sweep, lag fitted per sweep, all visible yaw-stage markers as one rigid body, a fixed E(y), one offset per sweep, loud failures.
2. Position from the body model's axis point over the sweep; the arc fit stays as a cross-check; tilt from the arc fit as before.
3. Template rebuilt yaw-balanced from all 7 markers of A, B and the 7-sweep bag, with E(y) and the gauge in it. Gauge pinned to session A's frame: A's own data must read 0.208°. ONE number (`gauge.pinned_yaw_offset_deg`) is the owner's adjustment.
4. σ = per-sweep formal error ⊕ out-of-sample repeatability (leave-sweeps-out on the 7 sweeps). No per-frame SD, no pause floor.
5. Gate: no state file → accept, persist, WARN; afterwards refuse |Δyaw| > max(3σ, 0.15°) (since 2026-10-10: 3·√(σ_new² + σ_ref²), see 2026-10-09-bb-calibration-heartbeat-yaw-wrap), |Δaxis| > 1.5 mm, or template residual > 0.5 mm; the one-shot `bb_moved` resets the reference (also after a QTM recalibration).
6. Drop the 80 ms constant, the pause floor, the QTM 1/2-at-yaw-0 claim.

**Findings made during the work that changed the code (each checked on the bags before acting):**
- **Session B's heartbeat is a two-age mixture, not a drifting lag.** A single-lag fit on B scatters 5° RMS. The per-sample age (where the posed θ crosses the reported yaw) is bimodal: ~75 ms or ~175 ms, exactly one 0.1 s republish period apart, 66–83 % of samples in the older mode per 10-minute window; A is unimodal at 94 ms. The bridge's 0.1 s timer republishes the latest heartbeat, so when it and the upstream 10 Hz stage tick nearly together a sample is fresh or one period stale. The investigation's "150 → 174 ms drifting" was this mixture read through a direction-split median. The lag fit for the heartbeat therefore lets each sample be one period older, whichever reads closer; a stamped source is fitted with one lag. Without this, every heartbeat calibration in a session like B would fail.
- **The parked pose is non-rigid in all three bags**, not only the 7-sweep bag: marker-pair separations at |y| < 1.5° are off the template by up to 1.3–1.4 mm (A, B) and 2.4 mm (7-sweep bag), against ≤ 0.29 mm (A, B) and ≤ 0.57 mm (7-sweep bag) at other yaws. So the position and the template residual are taken over the MOVING frames (frame time + lag inside a turning stretch), as the yaw is. Over all frames the template residual of the 7 sweeps was 0.76–1.28 mm; over the moving frames it is 0.20–0.47 mm.
- **The template residual is a pair-separation residual**, the largest mean (over the sweep) error of a marker-pair distance against the template. A marker that moved on BB changes its separations by up to its displacement; a per-marker mean residual vector instead picks up pose-dependent reconstruction error (0.5–0.8 mm per marker on the 7 sweeps).
- **Yaw samples are not interpolated across a lost track** (gap > 50 ms between posed frames): A's slews toward the reload pose lose the markers, and interpolating θ across those gaps produced 20° residuals.

**What was ruled out.** A 2D fit about world z (the first design): the 3D rigid fit needs no tilt estimate because the body frame carries the axis. An E-free estimator: within one bag it repeats about as well (SD 0.044° vs 0.031°, same yaw range in every sweep), but it reads −0.12° instead of +0.26° raw, so the offset would depend on the yaw range a window covers; A's slews (0–147°) and the sweeps (0–125°) cover different ranges. Applying E(y) to the aim: not done, moving and stationary relations differ by a few tenths.

**Accepted tradeoffs.** The template residual gate (0.5 mm) has thin margin on the 7-sweep bag (worst sweep 0.46 mm). The one-period-stale mixture would also absorb a genuine 100 ms lag jump within a sweep; it reports the stale fraction so that is visible. E(y) comes from one bag (the only one with sweeps).

## What changed

- **Estimator** (`bb_calibration.estimate_sweep_yaw_offset`):
  - `track_constellation` poses every frame against the 7-marker body template, label-free. The first frame, and any after a lost track or a 50 ms gap, is matched by geometry (`match_template`, with a 6 mm height tolerance for the tilted axis). Later frames reuse the previous pose to assign points within 3 mm. Each frame gets a 3D Kabsch fit. Frames with < 3 markers or RMS > 1.5 mm are rejected and counted.
  - `fit_yaw_latency` grid-fits the lag over −50…300 ms in 1 ms steps, with parabolic refinement. Samples are those with |dy/dt| > 5°/s, y inside E's valid range, and an unbroken track. The fit minimises the scatter of θ(t_y − τ) − y − E(y). The offset φ is the mean at the best τ. Its formal SE uses n_eff from the lag-1 autocorrelation.
  - Published offset = φ_raw − `raw_offset_at_pin_deg` + `pinned_yaw_offset_deg`.
  - Loud failures: `CONSTELLATION_TOO_FEW_FRAMES` (< 200), `CONSTELLATION_TOO_FEW_MARKERS`, `CONSTELLATION_RESIDUAL` (pair residual > 1.0 mm), `CONSTELLATION_TOO_FEW_MOVING`, `CONSTELLATION_LAG_AT_EDGE`, `CONSTELLATION_LAG_RESIDUAL` (> 1.0° RMS; a unit or source mismatch gives degrees).
  - The pause holds, visits and the 0.13° floor are deleted. So is the 2D-hold estimator.
- **Position.** `run_calibration` returns the body model's axis point as `bb_position_mm`; the arc fit's intersection is now `arc_position_mm`. It is the primary by the data: per-sweep SD x 0.04 / y 0.18 mm, against x 0.04 / y 0.38 mm for the arc fit. It also lands within 0.5 mm of A's and B's body-model points, while the arc fit sits 0.9–1.4 mm south of it in y. Tilt and all sweep gates are unchanged (arc fit).
- **Template** `resources/bb_marker_template.json`, schema 2 (the loader refuses schema 1):
  - **Markers:** all 7 yaw-stage markers in a body frame: origin on the yaw axis, z along the axis, x toward QTM 4.
  - **Build:** generalised Procrustes over yaw-balanced frames (≤ 40 per 2° bin per bag): A 2941, B 2950, and the 7-sweep bag 2520 with its parked frames excluded.
  - **Contents:** E(y), `gauge` (pin, raw value at the pin, uncertainty, the edit rule) and `sigma.repeatability_deg` 0.0308°, each with provenance.
- **σ** = √(formal SE² + 0.0308²): 0.046–0.070° per sweep.
- **Gate** (`check_calibration_consistency`, in `mocap_node`):
  - **Reference:** the state file (`~/bb_calibration_sessions/bb_calibration_last_accepted.json`), now storing position, lag, source and residual too. With no state file the calibration is accepted, persisted and logged at WARN; the template's pin is not a reference.
  - **Refusals:** `CALIBRATION_INCONSISTENT` (Δyaw > max(3σ, 0.15°) — both sweeps' σ combined since 2026-10-10 — or Δaxis > 1.5 mm in 3D), `TEMPLATE_RESIDUAL` (> 0.5 mm; `bb_moved` does not excuse it: a moved marker needs a new template), and `CALIBRATION_STATE_UNREADABLE` (a corrupt state file is no longer a silent fallback).
  - **Override:** `bb_moved` is one-shot, and is the flag for BB moved OR QTM recalibrated (rotating markers cannot tell the two apart). The accepted calibration becomes the reference.
- **`mocap_node` inputs:**
  - **Frames:** at their QTM stamp (`frame_ros_ns`, ROS clock), one per distinct stamp. The 200 Hz timer snapshots the latest frame, so duplicates are dropped; unstamped frames feed the arc fit only.
  - **Heartbeat yaw:** at ROS receive time; it was `time.monotonic()`, a different clock from the frame stamps.
  - **Stamped yaw:** the node subscribes to `bb/axis_estimates`. A `bb_yaw` joint (degrees; the sibling branch `bb-stamped-yaw-100hz`, BB fw 6 + can-bridge fw 28) is collected. With ≥ 100 samples in the window it is used with a single-age lag fit (expected a few ms); the two-joint message of older firmware falls back to the heartbeat.
- **Result message:** carries the estimator summary (frames, markers, residuals, source, lag, stale fraction, σ split), the arc-fit cross-check and the gate verdict. The INFO line reads `(lag N ms, gate ok)`. `ros_ws/docs/choreography.md` is regenerated for the new subscription.

## Validation (offline, node-shaped inputs, shipped template)

Inputs: labelled BB markers at their QTM stamps, labels dropped; the heartbeat at receive time; the CALIBRATING window; the whole `run_calibration` path. "old" is the anchor value the node published in the bag.

**Rosbag `2026-10-09_18-56-37`, 7 sweeps:**

| sweep | yaw offset ° (pinned) | σ ° | lag ms (stale %) | axis x, y mm (body model) | arc-fit x, y mm | template residual mm | lag-fit RMS ° | old anchor ° |
|---|---|---|---|---|---|---|---|---|
| 1 | 0.435 | 0.070 | 84.7 (0) | −975.86, −389.85 | −975.82, −391.22 | 0.46 | 0.28 | 1.020 |
| 2 | 0.493 | 0.050 | 83.8 (0) | −975.79, −389.87 | −975.79, −391.13 | 0.38 | 0.24 | 1.058 |
| 3 | 0.459 | 0.050 | 80.5 (3) | −975.78, −389.74 | −975.75, −391.03 | 0.37 | 0.24 | 1.018 |
| 4 | 0.443 | 0.046 | 80.5 (0) | −975.74, −389.51 | −975.72, −390.55 | 0.26 | 0.20 | 0.916 |
| 5 | 0.428 | 0.050 | 82.3 (0) | −975.75, −389.50 | −975.81, −390.40 | 0.33 | 0.22 | 0.820 |
| 6 | 0.447 | 0.046 | 82.6 (3) | −975.75, −389.48 | −975.75, −390.40 | 0.23 | 0.21 | 0.887 |
| 7 | 0.431 | 0.046 | 84.1 (0) | −975.75, −389.48 | −975.76, −390.41 | 0.20 | 0.20 | 0.878 |
| **SD** | **0.023** (mean 0.448, range 0.065) | | 1.7 | x 0.043, y 0.178 (z 1735.00 ± 0.04) | x 0.037, y 0.376 | | | 0.090 |

- 1087–1273 frames per sweep are posed (0–25 rejected) with 3–7 markers matched, and 32–38 heartbeat samples enter each fit.
- **Leave-sweeps-out (the check the investigation left undone).** Each sweep was estimated with E(y) refitted on the other six. The out-of-sample per-sweep SD is **0.031°** (range 0.090°), against 0.023° in-sample. The largest change of any sweep's value is 0.016°. The E coefficients move (e₁ 0.26–0.60), but the offsets barely do: E is constrained where the sweeps put their samples. The template's `repeatability_deg` is this 0.031°.
- E(y) as fitted (moving relation, at fixed offset): +0.03° at 10°, −0.11° at 50°, −0.31° at 70°, −0.57° at 90°, −1.10° at 125°.
  - The shape is like the investigation's (−0.44° at 70°, −1.57° at 125°). Its level differs because the template, the anchor direction and the lag model differ, and only differences of φ matter.

**Sessions A and B through the same estimator** (their bags start after their calibration, so these are the throw slews in 10-minute windows; per-window lag fitted):

| | windows: yaw offset ° (pinned) | lag ms | stale % | axis x, y mm | template residual mm |
|---|---|---|---|---|---|
| A (00:21) | 0.256, 0.191, 0.204, 0.194, 0.194 → **0.208** by construction (window SD 0.028, SE 0.012) | 94.2–97.1 | 3–4 | −976.22, −389.49 | 0.13–0.20 |
| A, last partial window | 0.050, **excluded**: template residual 0.64 mm > the gate's 0.5 mm, axis z +1 mm (unexplained) | 94.3 | 4 | | 0.64 |
| B (03:13) | 0.212, 0.197, 0.195 → **0.201** | 74.0–76.0 | 66–83 | −976.44, −389.89 | 0.16–0.18 |

- **B − A = −0.007°**, well within 0.05°. A and B are the same state, as both investigations found.
- **The 7-sweep bag reads 0.448°: +0.24° from A's frame.**
  - This is the shift the reinvestigation reported as +0.4 ± 0.2° (its methods spread +0.24…+0.6°, body model +0.43°). This estimator sits at the low end.
  - Marker geometry and the axis point are unchanged: body-model axis N vs A 0.47 mm.

**The pin and its uncertainty.** `gauge.raw_offset_at_pin_deg` = 0.0244° is A's mean raw value over its 5 accepted windows, so A reads exactly 0.208°. The transfer is good to the window SE (0.012°). Whether today's BB should aim with ~0.45° (this estimator's reading of the 18:56 state) or something nearer 0.6° is not decided by mocap: the cause of the A → 18:56 shift is unidentified. Take the pin as ± 0.2° until a validation sitting measures it. B's validation showed bearing error responds 1:1 to offset error.

## How to adjust the pin after the validation sitting

If ~40 validation throws give a mean bearing error implying an offset error δ (positive = the offset should be larger):
1. Edit ONLY `gauge.pinned_yaw_offset_deg` in `ros_ws/src/jugglebot/resources/bb_marker_template.json`, from `0.208` to `0.208 + δ`. Nothing else in the file changes; the template is not rebuilt.
2. Rebuild/deploy the package.
3. Set `bb_moved:=true` on `mocap_node` and calibrate once, so the stored reference moves by the same δ. Otherwise the gate compares a δ-shifted calibration with the old reference and refuses it when |δ| > the limit.

## Rule: moving BB or recalibrating QTM means re-pin + refit

The affine and the gauge are valid together for this mounting and QTM calibration only. After a physical move of BB or a QTM recalibration:
- set `bb_moved:=true`;
- calibrate (the gate lets one changed yaw/axis through and makes it the reference);
- refit `throw_affine_correction.json`, or re-pin from landings.

Recorded in the template's `gauge.rule`.

## Deferred

- **Merge into `skill-stack`, build, deploy, the first live sweep, and the validation sitting** that sets the pin. The node check is the INFO line `… (lag N ms, gate ok)`, the WARN for the first (reference-less) calibration, and the persisted state file.
  - *2026-10-10:* merged, built and deployed; live sweeps run since 2026-10-10 00:24, on the stamped `bb_yaw` since 13:45 (lag −0.2…+2.5 ms per sweep); the published offset is +0.776° after the owner's QTM world transform (base-frame path, [2026-10-10-bb-base-marker-frame](2026-10-10-bb-base-marker-frame.md)). **The pin sitting did not settle the pin:** `run_item2_sitting.sh` (2026-10-10 15:00 local, session `20261010T035151_835895Z`, 35 of 40 throws accepted) landed 117 ± 5 mm SHORT (radial, SD 31 mm) and +39.5 ± 2.5 mm lateral, and the rotation-vs-translation fit reads the lateral term as a translation (slope −0.3 ± 0.6°, intercept +46 ± 11 mm), not a frame rotation — so `settle_yaw_gauge.py`'s RE_PIN +2.406° is an artefact of its bearing rule and is NOT applied; the pin stays 0.208°. The same runner, plan, affine and signed s landed 21 mm RMS on 2026-10-09 (BB FW 5); since then BB FW 6 and bridge FW 28 were flashed, and in this sitting the hand jolted −3.3 → 0 mm before every throw (owner's observation; investigation under `~/bb_calibration_sessions/hand_jolt_20261010/`). Status stays in-progress until a sitting settles the pin.
- **The stamped `bb_yaw` path end to end** (sibling branch `bb-stamped-yaw-100hz`): the first live sweep with fw 6/28 should report lag a few ms with 0 % stale.
- The unexplained items: the A → 18:56 shift (+0.24° here), and A's last window (residual 0.64 mm, axis z +1 mm). Fixed base or room markers would separate a BB move from an encoder or frame change (REPORT Q3).
- Offline scripts in the worktree's `temp/bbyaw/sweep/` (`build1_template.py` … `build5_validate.py`, `probe_*.py`, not committed). They reuse the reinvestigation's `common.py` loader, GPA and axis helpers, and run the production functions.

## Verification

- Scoped, during the work (2026-10-09, `pytest tests/ros/test_bb_calibration_constellation.py tests/ros/test_mocap_node_yaw_gate.py tests/ros/test_perception_console_lines.py tests/ros/test_mocap_node.py tests/ros/test_bb_calibration_arc_span.py tests/ros/test_bb_calibration_coplanar.py tests/ros/test_bb_calibration_consensus.py ros_ws/src/jugglebot/jugglebot/tests/test_bb_calibration.py tests/ros/test_mocap_status.py tests/ros/test_choreography_map.py -q -p no:cacheprovider`): all green after the choreography regeneration.
- What the new and rewritten tests cover:
  - label-free matching and the tracker;
  - the lag fit on synthetic sweeps with known lags (20–250 ms);
  - the one-period-stale heartbeat (60 % stale: φ within 0.03°; a single-lag fit > 1° RMS);
  - the stamped 100 Hz path (4 ms lag, 0 % stale);
  - E held fixed (0–60° and 0–120° windows agree; without E they differ by > 0.1°), and E's valid range;
  - the gauge (an edit of the pin by δ shifts by δ) and σ;
  - every loud failure;
  - the gate: no reference, yaw, axis, residual even with `bb_moved`, the override, wrap;
  - `run_calibration` (position = body model, arc cross-check kept, missing anchor);
  - the `bb_yaw` JointState parsing (3-joint and 2-joint);
  - the shipped template and the loader's refusals;
  - in the node: first-calibration WARN, persistence, refusal, the one-shot reset of the reference, the corrupt state file, QTM-stamped frame de-duplication, and the stamped-source preference and fallback.
- The anchor-path tests (`test_bb_calibration_arc_span.py`, `_coplanar.py`, `_consensus.py`, `jugglebot/tests/test_bb_calibration.py`) are unchanged and run the legacy path.
- Full gate `./run_tests.sh --full`: the (date, command, result) triple is in the commit message (`git log --grep "Logbook-Entry: 2026-10-09-bb-constellation-yaw-offset"`).
