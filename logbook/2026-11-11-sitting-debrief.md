---
title: "Bench Hand Sittings 2026-11-11 debrief: BUG-55 passed on the bench Hand; FEAT-45 rung A flown, the scope tool's angle analysis was wrong in units and frame, rung B never ran, and the SPINOUT false-positive reading of BUG-57 was raised and refuted the same night"
type: investigation
date: 2026-11-11
status: complete
phase: "Sitting debrief"
related_plan: ""
related_issues: []
files_changed:
  - logbook/2026-11-11-sitting-debrief.md
  - logbook/2026-11-11-feat45-bench-hand-frame-and-scope-trigger.md
  - logbook/INDEX.md
  - tests/hardware/hand_stroke_cut.py
  - tests/hardware/hand_stream_bench.py
  - tests/motion/test_hand_stream_bench_stroke_cut.py
  - tools/probes/hand_foc_scope.py
  - tests/motion/test_hand_foc_scope.py
  - map/bug-hand-stall-stop-near-a-release-cannot-fit-the-stroke-the-20.md
  - map/feature-measure-the-hand-s-commutation-angle-error-through-a-hard.md
  - map/bug-hand-overshoot-at-apex-1-35-saturated-the-stepped-brake-at.md
  - map/bug-the-triangle-and-moving-gap-bench-stages-still-energise-the.md
  - map/feature-arm-a-power-capture-with-the-fault-trigger-on-the-robot-hand.md
  - docs/project-map.md
  - docs/runbook.md
subsystem: [hand, tooling]
tags: [sitting, hardware, diagnostics, bench]
---

# Bench Hand Sittings 2026-11-11 debrief

## Context

Two bench Sittings on the bench Hand (production Hand, shorter stroke, same current cap; ODrive Pro FW 0.6.11, CAN node 6; can-bridge FW 43 / PROTOCOL 14). The launch was down, so there is no rosbag and no launch log. The evidence is the stage CSVs in `temp/logs/`, the scope captures in `temp/hand_foc_scope/`, and the owner's typed notes on the two Items. The stage prints its verdict lines to the console only, so S2, S3 and `sched_refused` counter deltas and peak Iq are known only as the owner's "all PASS".

- 19:56 to 19:58: BUG-55, `--stage stroke_cut`.
- 20:08 to 20:15: FEAT-45, probe, rung A, and seven refused attempts at rung B.
- 21:17: the owner ran the BUG-55 `--cut-mode slow` control that had never run.

Nothing physically notable at either Sitting. No branch was merged (no held branch for either Item) and `ros_ws` did not change, so there is no rebuild.

## Results

| Row | Verdict | Evidence |
|---|---|---|
| BUG-55 upstroke at defaults (19:56:04, 3 runs) | pass | Fault 0, CLOSED_LOOP throughout. Firmware hold 3.086 rev equals the 14 ms C1 model to 5 digits (the FW 36 alternative is 4.433). Encoder overshoot +0.13 to +0.16 rev, settled in about 66 to 91 ms. |
| BUG-55 upstroke at `--cut-at-vel 64` (3 runs, plus one single run) | pass | Hold 4.960 rev equals the model. Overshoot +0.21 to +0.25 rev, settled in about 63 ms. The single run has no pasted verdict; its data match the repeats. |
| BUG-55 downstroke control at cut 64 (3 runs) | pass, with a deviation | C2 hold 2.518 rev equals the model. Flown at cut 64, not the row's default 55 (model hold 4.394). A faster entry into the same C2 path. |
| BUG-55 slow control at 21:17 (3 runs, owner: "all PASS, max belt residual 0.6552 rev") | pass | Read from `hand_stream_stroke_cut_20261111_211712.csv`: fault 0, axis_state 8 and lead_mask 0 in all 273 rows. Echo hold 2.821 rev in every run, equal to the model (full 14 ms grace path, FW 36 and model agree). Overshoot +0.09 to +0.11 rev. Cut vel taken as the default 0.9 x 40 = 36 rev/s. |
| FEAT-45 probe (20:08) | pass | Re-judged GO from the saved JSON: 8003 Hz, 255.9 ms window, no counter gaps. |
| FEAT-45 rung A (4 captures, three stage runs of three strokes) | pass for the clauses the files show | Each capture fired on a brake with the whole brake inside the window. Stage CSVs show fault 0, axis_state 8, echo hold equal to the model in 9 of 9 runs. Counter deltas unverified. |
| FEAT-45 rung B | did not run | Seven refusals from 20:14:28 to 20:15:37: at `--stroke-accel 1500` a 100 rev/s stroke needs 8.667 rev, which collides with the 1.0 rev margin under the 10.501 clip from a 0.49 rev park. |
| FEAT-45 `bench_100_0` capture | not a capture | A 60 s TIMED OUT window of the Hand at rest, saved by design. |

BUG-55: slow control and the other clauses above close it. Belt residual grew with entry speed (0.49, 0.72, 0.93 rev at 63, 73, 76 rev/s), against a 1.9 rev cut-window bound. At rung B or the robot's 130 to 150 rev/s a linear extrapolation gives 1.3 to 1.9 rev, which would be tight (inference).

Gaps carried: the downstroke control was not flown at the row's default entry; counter deltas and peak Iq were not saved (the new `_report.json` fixes that).

## Discussion

**The scope tool's analysis was wrong, and how it was caught.** The tool's printed lines for rung A (angle about -7 deg, flux linkage 8 times off, 22 to 24 phase jumps, "torque loss: none", "SPINOUT condition 37 ms") were taken at first as results. Recomputing offline caught four independent errors:

- FW 0.6.11 reports `foc.phase` in electrical turns and `foc.phase_vel` in electrical turns per second. The tool treated both as radians. `phase_vel / vel(pos)` is -6.85 in all four captures (7 pole pairs), and phase advances by `phase_vel/fs` per sample.
- That made the flux-linkage fit 8 times off (the tool's own docstring names that as the symptom of a units error) and the flat -7 deg angle mostly artefact.
- The 22 to 24 "phase jumps" are the wraps of a turns signal: 7 pole pairs times 3.13 rev of travel. In the right units the largest residual between phase step and `phase_vel/fs` is 0.006 turns (2 deg).
- `pos_rel` counts up while `phase_vel` is negative. The median acceleration per amp therefore came out negative and the tool's `k > 0` guard skipped the torque-loss detector, so "torque loss: none" printed without the check running, on every capture.

The recomputed brake is clean: about 154 rev/s^2 per A on the run-up and the brake, with about 450 rev/s^2 of gravity and friction; no jumps; angle about -3 deg plus 0.22 deg/A. This setup resolves only gross angle errors, because the voltage model (effective resistance about 45 % above configured, plus a dead-time-like step) dominates at 8 to 80 rev/s. The tool earlier passed its own tests because they fed it synthetic data in the tool's own assumed units; nothing checked the units against a real capture until this Sitting.

**Why the row's flags were wrong.** The row's rung flags (`--stroke-accel 2000 --stroke-top 6.0`, `7.9`) were written without a dry run against the stage's own planner, a dress-rehearsal miss. The stage's cap at 1500 rev/s^2 was refused at 2000 on the owner's first command, and the row's "brake from about 55 rev/s" premise was false: the stage models the stop from the cut frame's decision velocity, 79.4 rev/s at rung A. The cap's reason ("a bench Hand at 10 A follows 900 to 1050") was stale: the run-up measured 1600 to 1865 rev/s^2 at 11 to 15 A. A planner sweep offline now gives verified flags for each rung with the margin to the top stop.

**The false-positive hypothesis, raised and withdrawn the same night.** Through the clean brake the drive's own electrical power was positive (median +29 W, above +10 W for 13 of 18 ms). BUG-57's Reading section had inferred an angle error from "P_elec above +10 W", so that inference is wrong, and the first reading of the data was the opposite hypothesis: the robot's SPINOUTs might be ODrive false positives on correctly commutated hard brakes (filtered mechanical power reached only -2 W against the -10 W threshold on the bench). It was tested by replay before it went anywhere:

- A power model fitted on the bench (1.5 to 2 W rms against the drive's own power) was run through the drive's spinout filters on the robot's `hand_capture` data.
- Result at nominal parameters: 9 of the 12 faults trip, but so do 118 of 1122 clean launches. No setting of an 81-point grid trips most faults and spares the clean launches. Its fault-versus-clean AUC (0.87 to 0.91) is below that of peak brake current alone (0.99).
- L1019 and L2588 cannot be false positives (modelled filtered mechanical power +5 W and +3 W at the collapse). L10110 shows a loss while armed at the 60 A clamp, which a false positive cannot produce.

So the reading is refuted as the explanation. What survives is the correction: the power sign is not evidence of an angle error either way. Torque per amp was unchanged after every fault (factor 0.81 to 1.33, both signs), which makes a permanent slip of 30 deg or more unlikely, and a transient angle error the leading reading. Confidence in the refutation: moderately high, not certain; the firmware's exact power formula is inferred. The deciding measurement is still a robot power capture at a real fault (FEAT-49).

**The TIME_SYNCED abort.** After the 19:58 stage, every later command aborted with "bridge not TIME_SYNCED". Mechanism, traced in code: the can-bridge is the client of the TimeOfDay exchange. The only answerer with the launch down is the TimeOfDayServer the bench script creates for its own lifetime. When the last script exits the anchor goes stale after 90 s and the flag clears, with the next query up to 30 s away. A script that starts in that window read the flag once and aborted, and its own short-lived server rarely caught a query. Confidence: about 85 % for the mechanism and 50 % that it explains every abort that evening; an owner retry after a minute or any command in the bridge's 500 ms retry mode should have passed. A bridge re-flash cleared it, which masks the defect rather than proving anything. Confirming test not run: two stages 100 to 120 s apart with the bridge not rebooted.

**Why a 3000 cap and refusal before energising.** Owner decisions: raise the cap if the data supports it (it does, with the measured accelerations; the figures at 3000 are a one-point extrapolation, about 22 A mean and 31 A transient) and refuse before the Hand is energised. The owner would rather push the bench Hand hard to find its limits than repeat these tests on the robot's Hand. The rung C "margin" row is the one with 0.5 rev to a 10.4 top stop; the "max" variants with 0.1 to 0.2 rev were left out of the row.

## Fix

Stage (`tests/hardware/hand_stroke_cut.py`, `hand_stream_bench.py`, `tests/motion/test_hand_stream_bench_stroke_cut.py`):

- `STROKE_ACCEL_MAX` 3000 rev/s^2, with the measured-facts comment and a pure `stroke_accel_refusal`; the stale 10 A / 900 to 1050 text is gone and a test pins that.
- `--top-stop` (default the production clip, clamped to it): the planner's limit tightens and never loosens.
- The plan is built and every refusal printed before bring-up, so a refused plan leaves the Hand untouched. A refusal that only appears after bring-up (the position moved) still ends energised and the text now says so.
- A bounded wait of 35 s for TIME_SYNCED while the TimeOfDay server stays alive, with a failure text that no longer says "re-run".
- A report file `<csv stem>_report.json` with the plan, per-run criteria, the counter deltas and the console lines, written in a `finally` so aborts keep it.

Scope tool (`tools/probes/hand_foc_scope.py`, `tests/motion/test_hand_foc_scope.py`):

- Units: `foc.phase` is read as electrical turns and `phase_vel` as electrical turns per second. Phase jumps are the wrapped residual of the phase step against `phase_vel/fs`, in turns (threshold 0.08 turn). The largest residual is printed.
- Frame: the sign comes from the data (median `phase_vel` over d(pos)/dt where speed is at least 10 rev/s, at least 50 samples), not from the config. The robot's committed config has scale +7 and the bench drive reads -7. With too little motion the tool prints "frame: not evaluated".
- Torque check: acceleration per amp is a regression plus the median, and the loss detector normalises each sample by the capture's median, so it runs on either sign. When it cannot run it prints "torque loss: not evaluated" and the reason, never "none".
- SPINOUT model: the filtered electrical and mechanical power with the margin in W past each threshold and the best joint margin. Assumptions are printed: the filter is first order (`y += min(1, bw/fs)*(x-y)`), P_mech is Kt times Iq times omega, and the trip is filtered P_elec above the threshold with filtered P_mech below its threshold. Parameters come from the drive's identity when it was read, else the committed hand config, labelled "NOT read from this drive". The model is a model of the drive's check, not the check.
- Angle: reported only where the expected back-EMF is at least 0.5 V and at least 3 times the R, wL and step terms. No bench sample passes, so no angle is printed on these brakes. On the robot at 137 rev/s and 36 to 60 A wL times I is at least E, so none will qualify there either. The effective R and voltage step are fitted and reported, not applied.
- Capture bookkeeping: `analyze` prints kind, fired, polls and clean exits, re-judges a probe GO or NO-GO, and returns 3 on a TIMED OUT window, which is never passed off as a capture. Files are named `<label>_<n>_<kind>`.
- Fault trigger: fires only on an edge to non-zero of `disarm_reason` or `active_errors` against the previous poll. The first poll is the baseline, so a latched value never fires. A clean CLOSED_LOOP exit re-arms and saves only with `--save-clean`. Fault mode has no timeout by default and saves nothing on one. The default trigger point of 0.9 still goes through the scope grid.
- Probe labels: field strength and working offset are still recorded and now carry notes that they are not health checks (the offset tracks the electrical position).
- Brake mode no longer fires on a clean disarm (it used to fire on any CLOSED_LOOP exit), and each poll reads one more USB field (about 1.9 ms per poll, unmeasured).
- Unflown: the 8-channel `spinout` scope preset. Unresolved: `phase_vel` reads 2.4 % low against the phase step, so K may be about 5 % high.
- Before and after on the six real captures: jumps 22 to 24 became 0, acceleration per amp -130 became +154 to 155 (intercept about 450), the torque check ran and found no loss, and the angle is withheld. The model gave a joint margin of about -8 W with filtered P_elec +25.7 W, so no trip.

## Verification

- 2026-11-11, `python -m pytest tests/motion/test_hand_stream_bench_stroke_cut.py tests/teensy_link/test_hand_stream_bench_gap.py -q` in the worktree: 86 passed in 1.24 s (80 before, 6 new). `python -m py_compile tests/hardware/hand_stream_bench.py` OK.
- 2026-11-11, `python -m pytest tests/motion/test_hand_foc_scope.py -q`: 78 passed in 2.68 s.
- 2026-11-11, `python -m pytest tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_sitting_sections.py tests/sim/test_map_items.py tests/sim/test_plans_index.py tests/sim/test_sitting_debrief_protocol.py -q`: 102 passed in 3.38 s.
- 2026-11-11, `python -m pytest tests/sim/test_mapkit*.py -q`: 247 passed in 18.78 s.
- Not run: the new stage code and the fixed scope tool have never touched hardware. The first dry run of the stage with the launch down is the first real exercise.
- Gate: 2026-11-11, `./run_tests.sh -q` in the scratch worktree: one failure, `tests/sim/test_map_items_rated.py::test_every_item_is_rated` for FEAT-45 (moving it out of the inbox left it without an `owner` rating); every other test passed, serial phase included (612 s). FEAT-45 was then rated `owner: hands`. Re-run after that edit and after the later Item, logbook and Runbook edits, 2026-11-11: `pytest tests/sim/test_map_items_rated.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_sitting_sections.py tests/sim/test_map_items.py tests/sim/test_plans_index.py tests/sim/test_sitting_debrief_protocol.py tests/sim/test_mapkit*.py tests/motion/test_hand_foc_scope.py tests/motion/test_hand_stream_bench_stroke_cut.py tests/teensy_link/test_hand_stream_bench_gap.py -q`: 709 passed, 38 skipped in 28.25 s. The full gate was not re-run after the frontmatter edit.

## Withdrawn claims

- "P_elec above +10 W implies an angle error" (BUG-57 Reading, 2026-11-03): withdrawn. A clean hard brake on the bench already shows +29 W median.
- "The SPINOUTs are false positives of the drive's spinout test" (raised from the same bench data): withdrawn after the replay above.
- The scope tool's angle, flux-linkage, phase-jump and "torque loss: none" lines for rung A: not results (units and frame sign).
- The row's "brake from about 55 rev/s" premise and the stage's 10 A / 900 to 1050 rev/s^2 cap reason: false and stale.

## Follow-ups filed

- BUG-88: the `triangle` and `moving_gap` stages still energise the Hand before a refusal can abort (low).
- FEAT-49: a power capture with the fault trigger on the robot Hand to catch a BUG-57 fault (medium, robot); blocked on FEAT-45 rungs B and C passing with the fixed tool.
- FEAT-45 stays open with the next bench Sitting: re-measure the top stop first, then rung B and rung C (peak 108, cut 103, accel 2300, top 8.3: modelled entry 122.1 rev/s). That set was chosen over the 123.8 rev/s one because the planner accepts it for any park from 0.30 to 0.70 rev, where the faster set is refused at 0.35 and at 0.60.
- BUG-57 gets the 2026-11-11 section; its older Reading bullet is left as written and pointed at by Progress.
- BUG-55 is closed (Progress note plus `complete`).
