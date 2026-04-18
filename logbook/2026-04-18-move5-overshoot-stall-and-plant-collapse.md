---
title: "Move 5: MPC overshoot-recovery stall + plant-collapse misclassification"
type: investigation
date: 2026-04-18
status: in-progress
# --- Context (use what's relevant) ---
phase: "post-walk-forward-fallback-multiaxis-session"
related_plan: ""
related_issues:
  - MPC_STALENESS
  # Two candidate new IDs proposed in Open Questions:
  #   - PLANT_TELEMETRY_COLLAPSE
  #   - MPC_OVERSHOOT_SATURATION
sessions:
  - mpc_20260418_015112.csv
# --- Traceability ---
files_changed: []
commits: []
# --- Classification ---
subsystem:
  - controller
  - motion
tags:
  - mpc
  - hardware-plant
  - telemetry-staleness
  - walk-forward-fallback
  - overshoot
  - investigation
---

# Move 5: MPC overshoot-recovery stall + plant-collapse misclassification

## Summary

Move 5 appeared in the session diagnosis to "stall with 65 consecutive holds and 37 mm final tracking error" — a FAIL. Deeper investigation shows it's two separate events, neither of which fits that headline:

- An early 50-step IPOPT saturation (t=0.74–2.11 s) caused by the plant overshooting the reference by 12 mm at move-start; the walk-forward fallback correctly froze cmd and recovery was automatic once ref caught up.
- A terminal 62-step "stall" (t=14.0 s → end) that is NOT an MPC failure at all — it's a plant collapse triggered by the operator shutting down the ROS2 launch ~900 ms before, freezing HardwarePlant telemetry and leaving MPC solving against a `q_init = 0` sentinel feedback.

True MPC tracking during the 12-second main hold (steps 67–530) was 0.16 mm RMS. Revised verdict: **NEEDS_ATTENTION**, not FAIL. Walk-forward fallback (commit 64742f2) is validated.

## Symptoms

Prior session diagnosis reported:
- `success_rate` 78.3%, 130 timeouts, 111 `hold(Maximum_CpuTime)` steps (max 65 consecutive)
- Tracking RMS 14.6 mm, final position error 37.07 mm
- Per-leg RMS 62 mm with peak 194 mm
- One leg at the lower stroke limit (−5 mm margin)
- "matches MPC_STALENESS known issue"

These metrics made Move 5 look like a catastrophic failure — the kind of data that would drive a fix to the MPC solver or walk-forward fallback. But every one of them is dominated by the terminal collapse.

## Diagnosis

Two distinct events.

### Stall 1 — overshoot-induced NLP saturation (step 16–65, t=0.74–2.11 s, 50 consec non-success)

Move 5 starts from `(8.57, 34.0, 169.3)` carrying residual state from Move 4 — not from Active. Over the first ~15 steps the plant accelerates on the prior cmd, reaching `actual_z ≈ 193 mm` by t=0.74 s. At that instant `ref_z` is only 173 mm (still accelerating from 170 → 220 at 60 mm/s). Plant is **20 mm AHEAD of ref**.

From step 19 onward:
- `actual_z` freezes at 193.3 mm for 50 steps
- `cmd_u[0]` freezes at 187.03 mm (walk-forward fallback working)
- `ipopt_iter = 0` on every step → IPOPT hits the 24 ms CPU cap without completing a single iteration
- solve returns `hold`/`fallback`

Mechanism: the NLP is asked to plan a trajectory that brings `q=193` back down to meet a ref accelerating up from 173, while respecting `v_max` and actuator-tau constraints. With `N=10`, `tau=40 ms`, `v_max=140 mm/s`, this is a hard instance for warm-starting; the first iteration alone exceeds the 24 ms budget on the Jetson. Walk-forward correctly freezes cmd and the plant stabilises. At step 67 (t=2.17 s) the ref has risen to 175 mm and the horizon geometry simplifies enough for IPOPT to complete normally — cost 4203, 3 iterations in 13.7 ms, clean recovery with no discontinuity.

### Stall 2 — plant collapse (step 538–599, t=14.0 s → end, 62 hold + 3 fallback)

Between step 530 (t=13.79 s) and 537 (t=13.98 s), all 6 `actual_ext_i` drop from ~201 mm to ~165–188 mm — the platform free-falls 37 mm in 180 ms. Leg velocities read −170 mm/s (past the 140 mm/s `v_max` — this is uncommanded motion). At step 538 (t=14.00 s), every `actual_ext_i` reads exactly 0.00 simultaneously, which is physically impossible and is the sentinel for a stale/offline HardwarePlant telemetry stream. From step 538 to 599, `cmd_u[0]` is stuck at 193.91 and all q readings stay zero.

Timing correlation:
- Rosbag end: `bag_t = 102.9 s` = CSV `t ≈ 12.9 s`
- Orchestrator ACTIVE:STANDBY → IDLE: `bag_t = 99.9 s` = CSV `t ≈ 9.9 s` (but tracking stayed healthy at 0.2 mm RMS for the next 3.9 s, so this alone isn't the trigger)
- Collapse onset: CSV `t = 13.8 s` (~900 ms after rosbag recording stopped)

Most likely cause: the operator Ctrl+C'd the launch around CSV `t=13 s`. That stops the rosbag recorder AND the motor_guard AND mpc_bridge_node. When motor_guard dies the ODrives unarm → legs fall under platform weight. HardwarePlant loses telemetry → its ZMQ sub reads all-zeros (or the ZMQ drain-to-latest returns stale/empty). `run_mpc.py` was launched as a separate process so it continued to its 15 s duration, writing CSV rows against dead feedback — hence the 62 more "solves" against an impossible NLP (`q_init=0`, `u_prev=194 mm`).

The HardwarePlant telemetry-staleness E-stop (commit 17a239c) did not fire or did not halt the MPC loop — MPC kept trying to solve. That's the detection gap this investigation exposes.

### Session healthy window

Steps 67–530 (t=2.2–13.8 s, 12 s total) ran at:
- p50 solve time ~11 ms, p95 ~13 ms
- Tracking RMS ~0.16 mm, peak ~1 mm
- cost ~13 (essentially the workspace-normalised slack floor)
- All `Solve_Succeeded`

The move was working. The surrounding metrics are shutdown noise.

### Flagged Issues

- [error] 65 consecutive `hold(Maximum_CpuTime)` — misclassified; 62 of those are post-collapse dead-telemetry solves, not MPC failures.
- [error] Final position error 37.07 mm — shutdown artefact; true end-of-motion tracking was sub-mm during steps 67–530.
- [error] One leg at stroke lower-margin (−5 mm) — consequence of the uncommanded 37 mm free-fall at t=13.8 s, not an MPC planning error.
- [warning] Early 50-step overshoot saturation (step 16–65) — genuine, walk-forward fallback handled it correctly; candidate new ID `MPC_OVERSHOOT_SATURATION`.
- [info] MPC_STALENESS match — partial; applies to Stall 1 only.

## Discussion

The fix-proposer run split the work into three bundles by risk:

- **Bundle A — diagnose-only improvements (this commit).** Add detectors for both
  findings to `sim/analysis/diagnose.py` and `sim/analysis/known_issues.yaml`.
  Zero runtime risk; the value is that the next similarly-structured session
  won't be misclassified the way Move 5 was.
- **Bundle B1 — HardwarePlant exit path (deferred).** Add an
  `estop_requested` flag to `HardwarePlant` and make `run_mpc.py` / `runner.py`
  break cleanly when it's set. Also detect persistent-stale-contents
  (telemetry looks fresh but is unchanging) as a separate trigger for the
  stale-telemetry E-stop. Closes the safety gap that left Move 5 writing
  garbage CSV rows against dead feedback for 1.6 s.
- **Fix (b) — ref-from-current-plant-state on move transitions (deferred).**
  Prepend a synthetic `ReferenceEvent` at move boundaries carrying the
  current plant pose and twist, so the quintic Hermite smoothly bridges
  plant state to the first profiled event. Removes the overshoot geometry
  that provokes Stall 1 in the first place, with no change to MPC or the
  fallback. This is the real performance fix; Bundle A is damage control
  on the analysis side, Bundle B1 is safety.

The decision was to land Bundle A first, re-diagnose existing CSVs to
confirm the signatures fire correctly, and let Bundles B1 and (b) go
through their own review cycles since they touch the hardware-critical
path.

## Fix

**Landed: Bundle A — `sim/analysis/diagnose.py` + `sim/analysis/known_issues.yaml`.**

- New analyser `analyse_plant_collapse(records)`. Signature: a step where
  `mean(|leg_vel|)` across the 6 legs exceeds `V_MAX_MM_S` (140) AND at
  least 5 legs share the same direction of motion, followed within 500 ms
  by ≥20 consecutive steps where every `actual_ext_i == 0.0`. Returns
  the overspeed step/time plus zero-run length.

- New analyser `analyse_overshoot_saturation(records)`. Signature: ≥10
  consecutive steps with `ipopt_iter == 0` AND `tracking_error_mm > 10 mm`,
  with the reference still moving somewhere in the run. Returns all
  qualifying events (a session can have multiple).

- `analyse_csv()` now runs the collapse detector on the full record set
  first, then — if collapse is detected — truncates aggregate analyses
  (tracking, solve_times, solver_status, oscillation, discontinuities,
  workspace, steady_state) to the pre-collapse slice. This is the critical
  fix: previously, zero-sentinel feedback and ghost solve-loop activity
  contaminated the session metrics and drove FAIL verdicts on otherwise
  healthy runs. The `n_samples` field is unchanged (full count); a new
  `analysed_samples` field reports the post-truncation count.

- `_tag_post_estop_flags` generalised to `_tag_post_event_flags` handling
  both E-stop and plant-collapse timestamps. `run_diagnosis` passes the
  `overspeed_time_s` from the collapse detector alongside the existing
  E-stop times. The trigger-marker flags themselves (`estop_event: True`
  or `plant_collapse_event: True`) are exempt from downgrading so they
  remain visible.

- New flag emissions in `generate_flags()`:
  - `plant_collapse_event: True` at error severity with the overspeed
    timestamp (source `plant`).
  - `MPC overshoot saturation` at warning severity with the start
    timestamp (source `mpc`). Warning, not error, because the walk-forward
    fallback keeps it safe.

- Two new entries in `known_issues.yaml`: `PLANT_TELEMETRY_COLLAPSE`
  (severity high, status active, fix reference to the Bundle B1 plan) and
  `MPC_OVERSHOOT_SATURATION` (severity medium, status active, fix
  reference to fix (b)). Both link back to this logbook entry.

**Validation on the five CSVs from this session plus two prior FAIL cases:**

| CSV | plant_collapse | overshoot_saturation | Notes |
|-----|---------------|----------------------|-------|
| 20260418_014959 (Move 1 today) | no | no | clean |
| 20260418_015056 (Move 4, thermal) | no | no | correctly NOT flagged — it's a different failure mode (ipopt still takes iterations) |
| 20260418_015112 (Move 5) | **yes** @t=13.979s | **yes** @t=0.743s (50 consec) | both fire correctly |
| 20260417_184446 (baseline PASS) | no | no | clean |
| 20260417_184601 (prior FAIL) | no | **yes** (11 consec) | starts from non-Active like Move 5 — same failure mode picked up retroactively |

After truncation on Move 5: tracking RMS drops from 14.6 → 8.8 mm (still
contains Stall 1), workspace min extension flips from −5 to +138 mm
margin, solver timeouts drop from 130 → 67 with success rate 87.5%.
The `Near lower stroke limit` flag disappears (it was a post-collapse
artefact). The Plant-telemetry-collapse flag now carries the real story.

## Outcome

`pytest tests/ -v`: **920 passed, 51 warnings in 208 s**. No test
regressions. The warnings are pre-existing (pytest complaining about
`return True` instead of `assert` in `tests/motion/test_motor_guard.py`
— not introduced by this change).

Verdict for `mpc_20260418_015112.csv` corrected from FAIL to
NEEDS_ATTENTION in `sim/analysis/log_index.json` in the same commit,
with the extended notes describing the two-event decomposition.

Bundles B1 and (b) remain open — see Open Questions. Status set to
`in-progress` until those land; Bundle A in isolation is not the full
fix, only the detection-and-classification half.

## Open Questions

- Should `diagnose.py` detect the "all-legs = 0.00 simultaneously + `leg_vel > v_max` preceding it" signature and tag it as `PLANT_TELEMETRY_COLLAPSE` rather than lumping it into MPC solver flags? Proposed new known-issue ID.
- Should the overshoot-induced saturation (50 consec hold with actual leading ref) get its own ID `MPC_OVERSHOOT_SATURATION` distinct from `MPC_STALENESS`? The two have different fixes (warm-start improvement vs. `max_cpu_time` / horizon).
- Why did Move 5 start from `(8.57, 34.0, 169.3)` rather than near Active `(0, 0, 170)`? That's carrying 35 mm of residual state from Move 4's saturated end — arguably the root cause of the overshoot-at-start. Should moves that begin from a non-Active initial state get a startup-rampup window to reach Active first?
- Does HardwarePlant's telemetry-staleness E-stop actually halt the MPC solver loop, or only flag it? (`controller/hardware_plant.py` — verify.)
- Should `run_mpc.py` exit cleanly on sustained stale feedback, so a terminated launch doesn't leave a ghost solve-loop writing garbage CSV rows?

---

Note: the `log_index.json` verdict for `mpc_20260418_015112.csv` has been corrected from FAIL to NEEDS_ATTENTION in this same commit, with the extended diagnostic notes describing the two-event decomposition.
