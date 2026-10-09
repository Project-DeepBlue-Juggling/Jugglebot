---
title: Ball Butler's yaw offset comes from the whole marker constellation, not one anchor marker about the sweep's axis point; a consistency gate refuses a frame change unless bb_moved is set
type: feature
date: 2026-10-09
status: in-progress
phase: "two-ball-skill-stack — R5 (Ball Butler calibration)"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/bb_calibration.py
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
  - ros_ws/src/jugglebot/resources/bb_marker_template.json
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

# Ball Butler's yaw offset comes from the whole marker constellation, not one anchor marker about the sweep's axis point; a consistency gate refuses a frame change unless bb_moved is set

## Problem

`bb_calibration.calculate_yaw_offset` read BB's yaw offset from ONE marker (QTM 4) at 117.6 mm from the axis point that the sweep fitted, so 1 mm of axis error was 0.49° of yaw. Two calibrations on 2026-10-09 with BB untouched stored 0.208° (session A, `20261009T002142_931068Z`, in whose frame `throw_affine_correction.json` was fitted) and 0.681° (session B), and session B's throws landed −8 mm sideways at 1 m. The investigation (`~/bb_calibration_sessions/yaw_offset_investigation_20261009/REPORT.md`) showed that the axis point caused it, and that an axis-free constellation orientation reproduced A↔B to 0.05°. This entry implements that estimator. The design decisions (estimator, position from the sweep, template resource, gauge pin, σ, gate, sweep unchanged) were made before the work started and are not re-opened here.

## What changed

- **Estimator** (`estimate_constellation_yaw_offset`). Stationary holds come from BB's heartbeat yaw (within ±0.15° for ≥ 0.8 s; frames trimmed 0.15 s at both ends). Each hold collapses to per-marker medians (points present in ≥ 60 % of frames). The **template is matched by geometry only**: every (point pair, template pair) whose 3D separation and height difference agree within 3 mm proposes a rotation about world z, and the proposal that matches the most markers wins and is then refined by Procrustes. QTM labels are ignored, so a relabelled, missing or spurious point only changes which points are used. Per hold, offset = θ − reported yaw. Consecutive holds less than 1° apart with no heartbeat gap count as one visit (sweep 6 crept 0.4° during its pause), and the visits are circular-averaged. The estimator fails loudly with `CONSTELLATION_NO_HOLD`, `CONSTELLATION_TOO_FEW_MARKERS` (< 3 matched) or `CONSTELLATION_RESIDUAL` (a hold's x/y RMS > 1.0 mm). It runs in 2D about world z, not 3D in the axis frame, because a 3D fit would need the sweep's tilt, which is unstable (0.76° vs 1.15° stored for one mounting). The 2D bias from ignoring a tilt τ is τ × 0.0094 for the shipped template: 0.011° at 1.2°.
- **Position, tilt, the sweep, the arc-span floor and the consensus are unchanged.** `run_calibration(..., marker_frames=, yaw_samples=, template=)` takes the constellation path. Without a template it still runs the retired anchor estimator, so the sweep-fit fixtures and their anchor pins (`test_yaw_offset_is_anchored_on_qtm_4`, `test_outcast_yaw_anchor_refuses`, …) are kept unchanged as tests of that legacy path. `CalibrationResult.yaw_method` names the path, and `mocap_node` refuses to publish anything but `'constellation'`. On the constellation path a missing or outcast QTM 4 no longer refuses; the anchor value is logged at DEBUG as a diagnostic.
- **Template** `resources/bb_marker_template.json`: 4 markers, BB-local mm. It was built by generalised Procrustes over the 611 stationary holds of session A, with provenance, cross-checks and excluded markers recorded in the file. **Deviation from the plan, made on the data:** it holds only the four co-planar yaw-stage markers. The two off-plane markers (QTM 1/2, about 28 mm lower) distort at the parked yaw-0 pose, which is where the sweep pauses. On the 7 sweeps below they raise the pinned per-sweep SD from 0.095° to 0.297° and the pause RMS from 0.13–0.42 mm to 0.6–1.3 mm. QTM's own `Ball_Butler` 6DOF follows the same ±0.3° (SD 0.298°). QTM 1 is also the marker whose parked readings crept about 3 mm on 2026-10-05/06. At the throwing yaws of A/B the six-marker set is the better one (per-hold SD 0.15° vs 0.20–0.21°), so the multi-hold follow-up must revisit marker selection per pose. The instrument check is in Verification.
- **σ** (`yaw_offset_std_deg`, published) = √(σ_stat² + σ_tmpl²):
  - σ_stat = s/√N over N visits, where s is the per-visit SD, floored at the measured 0.13° hold scatter while N < 5.
  - σ_tmpl = mean of σ_c/lever, where σ_c = RMS·√(n/(2n−3)) is the per-coordinate residual SD of an n-marker fit.
  - For one sweep pause this gives 0.16–0.33°. It is not the per-frame SD, which was the old 0.03°.
- **Gate** (`check_yaw_offset_consistency`, in `mocap_node`). A new offset more than max(3σ_stat, 0.15°) from the reference is published as a failure, `YAW_OFFSET_INCONSISTENT …`, which names the delta, the limit and the override. σ_stat is the repeatability; the template term is common to two calibrations at the same pose. With one pause the limit is ±0.39°, which would have refused session B's 0.47°.
  - **Reference:** the last accepted calibration (`~/bb_calibration_sessions/bb_calibration_last_accepted.json`, parameter `bb_calibration_state_file`, written atomically on accept). If no calibration has been accepted yet, or the state file is corrupt, the reference is the template's pinned 0.208°.
  - **Override:** the parameter `bb_moved` (default false) is one-shot. It arms the next calibration only and re-arms when set true again.
  - **Interface:** `BallButlerCalibrationResult` is unchanged; `message` carries the method, the visit count, the σ split and the gate verdict.

## Validation (offline, three bags, BB untouched since session A)

**Headline: the rosbag `2026-10-09_18-56-37`** (7 calibration sweeps, 0 → 125.5° → 0 with a pause of about 2 s at 0). The 125° turnaround is not stationary: 2 heartbeats, about 0.2 s, so it contributes no hold. "Old" is the anchor value the node published in the bag. "New" is an end-to-end replay of `run_calibration` + the gate with the node's exact inputs: labelled BB markers only, the window's heartbeats, and the shipped template. The replay reproduces the published positions to ≤ 0.03 mm and the published anchor values to ≤ 0.02°, which checks the replay against the node.

| sweep | old (anchor) ° | old axis y mm | new ° | new σ ° (stat ⊕ tmpl) | markers / pause RMS mm | gate vs 0.208 |
|---|---|---|---|---|---|---|
| 1 | 1.020 | −391.57 | 0.288 | 0.164 | 3 / 0.14 | ok (+0.08) |
| 2 | 1.058 | −391.48 | 0.056 | 0.204 | 3 / 0.21 | ok (−0.15) |
| 3 | 1.018 | −391.36 | 0.180 | 0.331 | 3 / 0.42 | ok (−0.03) |
| 4 | 0.916 | −390.86 | 0.235 | 0.315 | 3 / 0.39 | ok (+0.03) |
| 5 | 0.820 | −390.72 | 0.123 | 0.161 | 3 / 0.13 | ok (−0.09) |
| 6 | 0.887 | −390.72 | 0.325 | 0.207 | 3 / 0.16 | ok (+0.12) |
| 7 | 0.878 | −390.73 | 0.250 | 0.285 | 3 / 0.35 | ok (+0.04) |
| **spread** | mean 0.942, **SD 0.090, range 0.24** | | mean 0.208 (pinned), **SD 0.095, range 0.27** | | | |

- **Read honestly.** Within one bag the new estimator's scatter (0.095°) equals the old one's (0.090°); a single pause cannot beat the ~0.1–0.13° hold-to-hold scatter. The gain is in the cross-session numbers. The old estimator put this untouched BB at 0.94°, i.e. 0.73° from session A's 0.208°, through its axis point (y −391.6 … −390.7 vs A's −389.26; 1.5–2.3 mm × 0.49°/mm). Had this calibration been accepted, the deployed affine would have been misapplied by about 0.7° (about 12 mm at 1 m). Only 3 markers are seen at the pause (QTM 4, 6, 7), the minimum.
- **Gauge pin** (decision 4): the template is rotated so that the mean over these 7 sweeps is 0.208° (raw mean 0.502°; pinned SE 0.036°). Every number above and below uses this gauge.
- **Cross-check on the session bags** (in-session stationary holds, shipped template):

  | | A (00:21) | B (03:13) | B − A |
  |---|---|---|---|
  | all holds (n 489 / 204) | −0.093° (SD 0.21) | −0.051° (SD 0.20) | **+0.042°** (investigation: +0.054°) |
  | yaw-0 holds right after the session's calibration (n 2 / 1) | −0.171° | −0.025° | |
  | yaw 1–20 / 20–40 / 40–70 | −0.21 / 0.00 / +0.18° | −0.16 / +0.07 / +0.13° | |

  A and B agree with each other. Both read **0.25–0.38° below** this bag's pauses, including the like-for-like post-calibration yaw-0 holds (A −0.17°, B −0.03° vs 0.21 ± 0.04°). This bag's own pre-sweep idle hold read −0.08°.

  The difference is **not explained**. The candidates are all untested:
  - BB parked at reported yaw +0.07° (A) and −0.32° (B) but −0.26 … −0.59° after these sweeps: a backlash band near the yaw zero;
  - a yaw re-home or a QTM recalibration between 03:13 and 18:56;
  - pose-dependent mocap error at 3 markers.

  The anchor estimator's own A→N difference (+0.73°) is fully accounted for by its axis error, so it says nothing about a real change. **Consequence:** if A's frame differs from this bag's by about 0.3°, the pin carries that into every throw (≲ 5 mm at 1 m), and the gate's ±0.39° would have just passed A's post-cal hold (Δ −0.38°). The first live sweep plus a short validation set settles it.
- **Pose dependence** (the investigation's tentative 0.3° at yaw 0). With the six-marker set A/B read about +0.3° at yaw 0 relative to the throwing yaws, as in the investigation. With the four co-planar markers the yaw-0 holds sit within 0.04° (A) and 0.14° (B) of the 1–20° level instead, so most of that 0.3° was the off-plane pair distorting at yaw 0. The four-marker set has its own trend across the throwing yaws (−0.2° → +0.15° from 1–20° to 40–70°; the six-marker set is flatter there). The moving sweep cannot measure it: the heartbeat lags the mocap frames by about 80 ms (±7° at 90°/s), and the outbound/return average still leaves about 1°. This is why both hold trims are 0.15 s.

## Rule: moving BB means re-pin + refit

The pinned gauge and the affine are only valid together and only for this mounting. If BB is physically moved: set `bb_moved:=true` on `mocap_node`, calibrate (the gate lets one changed offset through and persists it), re-pin the template's `gauge` to the new frame, and refit `throw_affine_correction.json`. Recorded in the template's `gauge.rule`.

## Deferred

- **Multi-hold sequence** across the throwing yaws, approached from one direction (firmware sub-state or host-driven aims). The estimator already adds the extra holds to N, so σ_stat drops as √N. It should revisit the six- vs four-marker choice per pose.
- **The BallButler runner's post-hoc per-session offset and affine rotation** (REPORT § 4). Its "Cross-repo note" belongs in the BallButler logbook when that lands.
- **Merge into `skill-stack`, build, deploy, and the first live sweep** with the node check: the INFO line `… ±σ (1 hold(s), gate ok)` and the persisted state file. Also: settle the 0.25–0.38° A/B-vs-pause difference above, and ask the owner whether BB was power-cycled or re-homed, or QTM recalibrated, between 03:13 and 18:56.
- Offline scripts (`extract.py`, `build_template.py`, `pin_gauge.py`, `validate_new_bag.py`, `session_phi.py`, `replay_node.py`, `sweep_posedep.py`) live in the worktree's `temp/bbyaw/` and are not committed. Their logic is that of the investigation directory.

## Verification

- New and touched calibration tests (2026-10-09, `pytest tests/ros/test_bb_calibration_constellation.py tests/ros/test_mocap_node_yaw_gate.py tests/ros/test_perception_console_lines.py tests/ros/test_mocap_node.py tests/ros/test_bb_calibration_arc_span.py tests/ros/test_bb_calibration_coplanar.py tests/ros/test_bb_calibration_consensus.py ros_ws/src/jugglebot/jugglebot/tests/test_bb_calibration.py tests/ros/test_mocap_status.py -q -p no:cacheprovider`): **149 passed, 1 skipped in 3.52 s**.
  - The new tests cover: label-free matching (permuted, missing, spurious, < 3); a known rotation under noise; the gauge sign; holds, creep and gaps; visits; σ (N = 1 floor, N = 16 SD/√N); loud failures; the gate's accept, refuse, 3σ, override and wrap cases; the node's one-shot override, persistence, template-gauge fallback, corrupt state file, missing template and anchor refusal; and the shipped file (gauge 0.208°, separations > 2 × tolerance, self-match at any yaw minus any marker).
  - `test_perception_console_lines.py` changed deliberately: the INFO line now carries `(N hold(s), gate ok)`, and its node writes gate state under `tmp_path`, never `~/bb_calibration_sessions`.
- Full gate `./run_tests.sh --full`: the (date, command, result) triple is in the commit message (`git log --grep "Logbook-Entry: 2026-10-09-bb-constellation-yaw-offset"`).
