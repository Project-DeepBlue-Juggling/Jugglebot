---
title: BB base-marker frame — four fixed, unlabelled markers on BB's shelf are identified by geometry and posed every sitting; BB's pose in that frame (κ, p_b) is pooled over accepted sweeps, and `bb_pose_source` (sweep default | base_frame | auto) publishes the base pose composed with it, gated in the base frame so a QTM frame shift is reported and accepted while a BB move on the shelf is refused
type: feature
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
  - logbook/2026-10-10-bb-base-marker-frame.md
  - logbook/INDEX.md
subsystem:
  - tracking
  - ros
tags:
  - testing
---

# BB base-marker frame

## Problem

The sweep estimator measures BB in QTM's world frame. That frame moves 0.3–1.5 mm (non-rigidly) between sittings and is sometimes recalibrated, so a QTM change reads as a BB move: the gate refuses it, or `bb_moved` accepts it, and neither can tell "QTM moved" from "BB moved". BB is bolted to its shelf. The owner fastened four markers to the shelf (an L: C–S 115 mm; C–M–E collinear, 80 / 240 mm), with no QTM label. The first sitting that recorded them is bag `2026-10-10_12-26-22`: 10 sweeps, all accepted live.

## Analysis (bag 2026-10-10_12-26-22; scripts and their `.txt` outputs in `~/bb_calibration_sessions/base_frame_20261010/`; the full report was delivered in the session hand-off (the harness refused a REPORT.md))

- Identification: the four are the only 4-subset of the 13 unlabelled markers that matches the nominal distances within 4 mm. There were 0 ambiguous frames and 0 spurious triangles in 650 sampled frames. All four were visible together in 99.5 % of the distinct frames; the rest have no unlabelled markers at all.
- As-built vs nominal: CM +0.15, CE −0.18, CS +0.33, ME −0.33, MS +0.03, ES −0.13 mm. M sits 0.31 mm off the C–E line, and S is at 89.94° from it.
- The plane is tilted 1.04° from world z, toward the same azimuth as BB's yaw axis (0.8–1.0°). So the shelf, or QTM's z, is tilted, not BB on its mount.
- Base pose over the sitting: heading −179.051° with a per-frame SD of 0.004–0.005°. The SE of the static mean is 0.0003° (10 s blocks) and 0.0004–0.0009° per sweep window. The heading drifts −0.0014°/min (0.0036° over the 2 min); the origin's SD is 0.01–0.03 mm.
- BB-in-base over the 10 sweeps, using the live published values: κ = +179.491°, SD 0.050°, SE 0.016°; p_b = (177.54, 148.86, 76.71) mm, SD 0.03–0.04 mm. Offline replays give: stamped yaw κ +179.507° (SD 0.021°), heartbeat yaw at bag time +179.512° (SD 0.038°). The world-frame SD equals the BB-in-base SD to 0.0001°, because the base is static within a sitting.

## Discussion

- **κ comes from the live published offsets, not a replay.** Its gauge has to be the published one, the frame the deployed aim correction lives in. Offline, the heartbeat path can't reproduce the node's values: the bag has the recorder's receive times, not the node's. Per-sweep live−offline differences reach ±0.11° (SD ~0.07°), and the lag reads 72–76 ms offline vs 73–82 ms live. The stamped replay is reproducible and 2.4× tighter (SD 0.021°), but it is not what was published. The three seeds agree within 1.3 SE. Going forward the node pools whatever it accepts. Using `bb_yaw_source:=stamped` would shrink the pool's SD about 2.4×, but that is the owner's call (it is not changed here).
- **Sweep mode keeps the world gate unchanged.** The BB-in-base gate is the principled one, but making it the default would change the default behaviour. So under `sweep` the base frame is only a diagnostic and feeds the pool. A refused world-gate sweep whose BB-in-base is unchanged gets the suffix `· base frame: BB unmoved, QTM frame shifted`.
- **S is required for a pose.** C, M and E are collinear, so a match without S fixes no roll about their line. It also has no handedness: a mirrored L matches C–M–E with the normal up. The task's "≥ 3 of 4" therefore means S plus any two of C, M, E. S was visible whenever C, M and E were.
- **The pin is carried, not frozen.** Each κ record stores the marker template's `gauge.pinned_yaw_offset_deg` it was measured under. Pooling moves each record to the current pin. So the landing-derived delta from BallButler `settle_yaw_gauge.py` (which edits that pin) moves κ by the same delta, with no re-sweep and no edit to this resource. The pin's ±0.2° is common to every sitting and is reported, not folded into σ.
- **A separate module.** The base frame lives in `bb_base_frame.py`, beside `bb_calibration.py`, not inside it. It imports `kabsch` / `_assign` from there, and `bb_calibration.py` is unchanged.

## Change

- `bb_base_frame.py` (pure):
  - label-free identification (`identify_base_markers`): triangle proposals, Kabsch, normal up, non-collinear;
  - a tracker (`BaseFrameTracker` / `track_base_frame`), the GPA template (`learn_as_built`), and the static pose (`estimate_base_pose`: median-outlier rejection, block SE, visibility, template residual);
  - `bb_in_base` / `compose_bb_pose`, `pool_kappa` (wrap-safe, pin-normalised, SE floor), and `check_base_frame_consistency`;
  - `BaseFrameMonitor` (slow running estimate), the resource loader and the state file I/O.
- `mocap_node`:
  - collects every distinct frame's unlabelled markers in the window, and feeds the monitor at 5 Hz;
  - logs one INFO line when the base is first seen, with its shift vs the stored base pose;
  - new params `bb_pose_source` (default `sweep`), `bb_base_frame_file`, `bb_base_frame_state_file` (`~/bb_calibration_sessions/bb_base_frame_state.json`) and `bb_base_min_sweeps` (5);
  - under `base_frame`, publishes heading + κ_pool and o + R·p_b_pool with σ = √(SE_κ² + SE_h²), still reports the sweep's own value and the Δ, and pools each accepted sweep;
  - `auto` uses the base frame once the pool has ≥ 5 sweeps and the base is seen with residual ≤ 0.5 mm, else the sweep;
  - the world state file gains `pose_source`, `sweep_yaw_offset_deg`, `sweep_position_mm`, `base_frame` and `kappa_deg`.
- New refusal codes: `BASE_FRAME_NOT_SEEN`, `BASE_FRAME_MODEL_MISSING`, `BASE_FRAME_STATE_UNREADABLE`, `BB_IN_BASE_MOVED` (BB or the frame moved on the shelf; `bb_moved` starts a new pool), `BASE_FRAME_RESIDUAL` (frame knocked; `bb_moved` does not override), `BB_POSE_SOURCE_INVALID`. The existing codes are unchanged.
- `resources/bb_base_frame.json`: as-built template, the 10 seed records, `base_pose_at_build` (the first frame-shift reference) and the gauge note. It was built by `tools/bb_base_frame_build.py --bag …`; `--from-state` folds the node's pool back into the resource.

## Replay (10 sweeps)

**Leave-one-out.** The base path uses κ pooled over the other 9 sweeps. Live published values: Δ(base − sweep) RMS 0.053°, 9/10 within the sweep's 1σ, 10/10 within 2σ (sweep 6: Δ −0.093° against σ 0.069°). The base-path SD is 0.005° against 0.051° for the sweeps. The offline replays give the same 9/10 and 10/10, with RMS 0.040° (heartbeat) and 0.023° (stamped).

**Through the real `_finalize_calibration`** (ROS mocked, resource seed):
- `base_frame` publishes SD 0.002–0.004° against the sweeps' 0.021–0.038°;
- `auto` with an empty seed switches to the base frame after 5 pooled sweeps;
- `sweep` publishes the sweep exactly.

The literal ship criterion was "the base path within each sweep's σ for all 10". It is not met (9/10, as pre-registered), so **the default stays `sweep`**. The one miss is that sweep's own scatter.

## Verification

- Scoped (`~/Desktop/PDJ_venv/venv/bin/python -m pytest -q tests/ros/test_mocap_node.py tests/ros/test_mocap_node_yaw_gate.py tests/ros/test_mocap_node_keep_last_good.py tests/ros/test_perception_console_lines.py tests/ros/test_ball_butler_node.py tests/ros/test_choreography_map.py tests/ros/test_bb_calibration_*.py tests/ros/test_bb_base_frame.py tests/ros/test_mocap_node_base_frame.py tests/ros/test_qtm_clock_sync.py tests/ros/test_mocap_interface.py`, run 2026-10-10): **337 passed, 1 skipped**. That includes the 19 + 17 new tests.
- Full gate (`./run_tests.sh --full`, run 2026-10-10 13:01–13:12 local, this tree minus this line and the resource's provenance note): **PASS — 6485 passed, 9 skipped, 1 xfailed (parallel, 405 s) + 6 passed (serial, 26 s); total 431 s**. After those two edits: see the commit message.
- Not verified on hardware:
  - the node path live (the 5 Hz monitor's cost on the loaded Jetson; the first-sighting INFO line);
  - a second sitting. Whether κ holds across a QTM recalibration is THE open question: a non-rigid QTM warp that differs between BB and the base, ~180 mm apart, is not removed.

external_changes: none (BallButler not edited). `settle_yaw_gauge.py`'s ACTION line ("add δ to gauge.pinned_yaw_offset_deg … rebuild, re-run the sweep") stays correct. Under `base_frame` / `auto`, κ follows the pin after the rebuild, and the sweep is only needed to refresh the world reference.

**Merge follow-up (skill-stack, 2026-10-10):** `bb_yaw_source` default flipped `heartbeat` → `auto`. This sitting is the first clean-clock bag carrying both sources; the replay above reads 0.021° SD stamped against 0.038° heartbeat, meeting the rule pre-registered in 2026-10-10-bb-yaw-offset-spread-stamped-source (stamped SD ≤ 0.08° on a clean sitting). `test_auto_is_the_default_yaw_source_and_prefers_the_stamped_stream` pins it.

**Second sitting with the frame (2026-10-10 13:45 local, 8 sweeps, log `python3_1977193_1791600348251.log`):** all accepted on the stamped yaw (lag −0.2…+2.5 ms, clock gate 0.0 %, base markers 100 % visible); launch line `BB base frame seen … vs the stored base pose (build): 0.69 mm, +0.003° (QTM frame shift)`; per-sweep κ 179.467 179.514 179.505 179.574 179.518 179.502 179.531 179.513 → mean +179.515°, SD 0.030°, SE 0.011°; Δ vs the build's +179.491° ± 0.016° = +0.024° (1.3 combined SE, inside the 0.05° warp criterion); p_b (177.50, 148.99, 76.48) mm vs (177.54, 148.86, 76.71). Within the sitting the base pose crept +0.004° / 0.3 mm and the base residual 0.20 → 0.30 mm (watch). **`bb_pose_source` default flipped `sweep` → `auto`** on this evidence; `test_default_pose_source_is_auto_and_an_invalid_value_is_refused` pins it.

Operational notes: after the first sittings (14:10 / 14:13) the identifier searches every marker but BB's own (a QTM rigid body that labels a base marker blinded it there), a base expected but unseen leads the refusal instead of `bb_moved`, and κ is pooled robustly behind a 0.09° gate floor — see [2026-10-10-bb-base-frame-robustness](2026-10-10-bb-base-frame-robustness.md).
