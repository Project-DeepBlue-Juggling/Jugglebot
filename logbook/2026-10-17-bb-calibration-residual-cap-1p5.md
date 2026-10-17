---
title: "BB calibration template-residual cap raised 1.0 -> 1.5 mm and advisory 0.90 -> 1.0 mm on measured evidence: kappa deviation does not grow with the residual to 1.25 mm on 89 real sweeps, the worst injected yaw bias is 0.155 deg per mm of residual (so <= 0.23 deg at the cap), and vertical modes are invisible to any cap"
type: investigation
date: 2026-10-17
status: resolved
phase: "two-ball-skill-stack - post-R6 (Ball Butler calibration)"
related_plan: bb-calibration-one-attempt.md
related_entries:
  - 2026-10-16-bb-calibration-power-cycle-ab
  - 2026-10-16-bb-calibration-phase2-template-residual
  - 2026-10-10-bb-template-residual-coverage
files_changed:
  - ros_ws/src/jugglebot/jugglebot/bb_calibration.py
  - tests/ros/test_bb_calibration_constellation.py
  - tests/ros/test_bb_calibration_replay.py
  - tests/ros/test_mocap_node_keep_last_good.py
  - tests/ros/test_mocap_node_yaw_gate.py
  - tools/probes/bb_residual_cap_evidence.py
  - tools/probes/README.md
  - logbook/2026-10-17-bb-calibration-residual-cap-1p5.md
  - logbook/2026-10-16-bb-calibration-power-cycle-ab.md
  - logbook/INDEX.md
  - plans/active/bb-calibration-one-attempt.md
  - ros_ws/docs/bb_calibration_readings.md
subsystem:
  - tracking
  - ros
  - tools
tags:
  - kinematics
  - testing
  - docs
---

# BB calibration residual cap 1.0 -> 1.5 mm

## Summary

Owner question, 2026-10-17 morning: is the CONSTELLATION_RESIDUAL refusal worth fussing over, and is there a workaround that does not compromise accuracy. After a measured evidence run the owner decided (2026-10-17): `CONSTELLATION_MAX_RESIDUAL_MM` 1.0 -> 1.5 mm and `TEMPLATE_RESIDUAL_ADVISORY_MM` 0.90 -> 1.0 mm. The worst single-sweep yaw bias the new cap can admit is 0.155 deg/mm x 1.5 mm = 0.23 deg (0.219 deg was the largest any injection produced under the cap). It does not protect against vertical marker modes, which no residual cap sees (Diagnosis). The contract landed together: the constants' doc comments (document), the constants (one enforcement point), and the band test (one test).

## Symptoms

The 2026-10-16 22:00 state (entry `2026-10-16-bb-calibration-power-cycle-ab`): pair Q3-Q6 1.14-1.25 mm off the template after the 86 min BB-only off, 7 of 8 sweeps refused CONSTELLATION_RESIDUAL at the 1.0 mm cap, while the same sweeps solved with the cap lifted gave kappa 179.22-179.37, inside the sitting's scatter. Three workarounds were on the table: (1) a leave-worst-marker-out subset-refit fallback with a measured offset (the Phase 2 leave-one-out bounded it at <= 0.09 deg per dropped marker); (2) raise the cap with evidence; (3) operator power-cycle and retry. The owner chose (2) and asked for the evidence run.

## Diagnosis

Evidence run (Opus agent, 2026-10-17): record `temp/probes/bb_calibration_replay/20261017-cap-evidence/` (`REPORT.md`, `partA.txt`, `partC.txt`), promoted harness `tools/probes/bb_residual_cap_evidence.py` (stages decode/prep/A/B/C).

**Part A, the empirical question.** 89 sweeps: 75 from the daytime replay plus the 14 sweeps of the 2026-10-16 evening re-solved with the cap lifted so they share one footing (7 of those 14 the old cap had refused). Deviation of kappa from the sitting reference vs the sweep's residual:

| residual band (mm) | n | SD (deg) | max (deg) |
|---|---|---|---|
| 0.0-0.3 | 24 | 0.032 | 0.110 |
| 0.3-0.5 | 42 | 0.044 | 0.160 |
| 0.5-0.7 | 9 | 0.043 | 0.083 |
| 0.7-0.9 | 7 | 0.027 | 0.078 |
| 1.0-1.3 | 7 | 0.050 | 0.102 |

Signed slope -0.022 +/- 0.016 deg/mm, |dev| slope +0.024 +/- 0.013; worst |dev| 0.160 at 0.50 mm (10-14 20:54 #7). Reference rule: median kappa of the same sitting's sweeps with residual <= 0.5 mm; the deformed 22:00 group has no clean sweep and uses the 20:31-20:34 median 179.269 (same sitting, same warm state, BB-only off); three daytime bags with no clean sweep use the nearest bag of the same date (the agent's fallback, flagged `ref_src`); 11 references rest on one sweep. Direction split: the deformation moves the two sweep legs about +/-0.04 deg in opposite senses (E1 out-ret +0.055, E2 -0.022) with the mean unchanged.

**Part B, injection into the real solver** (1589 solves on 7 real sweeps, 0 refused for another reason). Controls: rigid rotation +/-0.1 deg gives exactly -/+0.1000 with the residual unchanged; +/-0.75 % uniform xy scale gives residual 1.17-1.96 mm, yaw <= 0.013 deg. Yaw bias is linear in displacement. Slope (deg of yaw per mm of residual): Q3/Q6 tangential in opposite senses (the 10-16 mode) 0.020-0.041; single marker tangential/radial 0.0035-0.12; Q3 vertical 0.155 (worst), Q5 vertical 0.134, Q3+Q6 same sense 0.13-0.15; Q5 -z is invisible (residual flat 0.33 mm out to 3 mm displacement, bias 0.077 deg). Detection gap: the vertical modes reach 0.1 deg of bias at 0.64-0.75 mm of residual (the 10-10 entry's gap, reproduced: Q3 vertical reaches 0.1 deg at 2.9 mm of displacement reading 0.64 mm). Directly measured (no extrapolation) largest bias under a cap C: 0.117 (1.0), 0.172 (1.25), 0.219 (1.5), 0.233 (2.0).

**Part C, cap table.**

| cap C (mm) | worst-case bias (deg) | refused of 89 |
|---|---|---|
| 1.0 | 0.155 | 7 |
| 1.25 | 0.194 | 0 |
| 1.5 | 0.233 | 0 |
| 2.0 | 0.311 | 0 |

Worst case <= 0.20 deg up to 1.29 mm; <= 0.10 deg only up to 0.64 mm, so the old 1.0 cap already admitted 0.155.

## Discussion

1. **Why 1.5 and not the agent's 1.25 with a 0.65 advisory.** 1.25 clears the largest residual on record (1.247 mm) by 0.003 mm, so it does not stop the recurrence of this very refusal on the next deformation of the same size. An advisory at 0.65 would have flagged 16 of 89 sweeps, which is noise on the operator line. 1.5 costs 0.08 deg over the old cap by the slope bound (0.233 - 0.155) and 0.10 deg by the direct injections (0.219 - 0.117); 0.04-0.05 deg over 1.25. The evidence report itself called 1.5 "not defensible against the 0.20 deg band" (the worst case stays <= 0.20 deg only for a cap <= 1.29 mm, and it recommended 1.25 mm with a 0.65 advisory); the owner accepted that exposure on 2026-10-17 because the only observed deformation mode (Q3/Q6 opposite tangential) costs <= 0.06 deg at 1.5 mm. The kappa-epoch advisory is the second line, but it flags only a kappa shift at or above 0.20 deg between sweeps and nothing smaller.
2. **Why not the subset-refit fallback.** More code on the solver path for a bias the data say is inside scatter. It stays available if the deformation grows past 1.5.
3. **Why not a template rebuild.** The deformation is re-rolled at each BB power-on (A/B entry), so a rebuild chases it; the builder scripts are gone.
4. **What the cap is and is not.** A bound on the visible modes. The vertical modes, and anything beyond the 3 mm match tolerance, are outside it (Open questions). No cap value is a 0.1 deg guarantee.
5. **Contract framing.** Document, one enforcement point and one test landed together. The A/B entry's Discussion point 4 ("do not relax the cap") is superseded by this measurement, not by convenience; it carries an addendum line.

## Fix

- `ros_ws/src/jugglebot/jugglebot/bb_calibration.py`: the two constants and their doc comments (the comments are the contract text).
- `tests/ros/test_bb_calibration_constellation.py`: the old refusal recipe (1.6 mm displacement, read 1.58 mm, only 0.08 above the new cap) raised to 2.5 mm reading 2.48 mm (4.0 mm reads 0.005 mm because the pose fit absorbs it); constellation 0.95 residual literal -> the advisory constant + 0.05, `startswith` computed; `outcome_body` res 1.0 -> advisory + 0.2 (not in the brief, found by the run); new band test `test_a_residual_inside_the_advisory_to_cap_band_is_accepted_flagged_and_bounded` (recipe 1.2 mm displacement reads 1.184 mm, yaw -0.094 deg, bound 0.25 deg, `TEMPLATE_RESIDUAL` flag asserted).
- `tests/ros/test_mocap_node_keep_last_good.py`: same 2.5 mm recipe. `tests/ros/test_mocap_node_yaw_gate.py`: `tres` 0.95 -> 1.2 and the mocked refusal text 1.05 -> 1.55 mm. `tests/ros/test_bb_calibration_replay.py`: docstring band 0.90 -> 1.0.
- `tools/probes/bb_residual_cap_evidence.py` promoted (stages decode/prep/A/B/C) with a row in `tools/probes/README.md`.

## Verification

All 2026-10-17.
- Evidence run: stage B 1589 solves in 348 s (record above).
- Implementer scoped run, `pytest <six files> -q`: 122 passed, 1 failed (the `outcome_body` res=1.0 literal); then `pytest tests/ros/test_bb_calibration_constellation.py -q`: 49 passed.
- `pytest tests/ros/ -q -x`: 3111 passed, 2 skipped in 646.60 s.
- Probe smoke: stage A byte-identical to the record (89 rows, slope -0.022 +/- 0.016, worst 0.160); stage B 1589 solves in 433 s; stage C byte-identical (0.1554 deg/mm Q3 z+); decode of 2026-10-16_16-10-37 in 21 s; pyflakes clean. Orchestrator re-ran stage A after the decode edit: headline identical.
- Gate, final tree (`./run_tests.sh`, 2026-10-17 10:31-10:37, log `temp/logs/run_tests_bbcap_final.log`): parallel phase **6710 passed, 11 skipped in 300.78 s**, serial phase **4 passed in 16.58 s**, exit 0. An earlier run on the code changes alone (before the narrative edits and audit fixes; `run_tests_bbcap_pre.log`) gave the same counts.
- `/audit --unstaged` (Sonnet audit-reporter, 2026-10-17): 2 warnings (the entry and the cap comment omitted that the evidence report called 1.5 not defensible against the 0.20 deg band; `tools/probes/bb_calibration_bands.py` still hard-coded the 1.0 cap and clamped its proposed advisory at 0.9) and 3 notes (plan parentheticals, the advisory comment's sitting-range wording, the entry's 7-of-89 wording); all five applied before the final gate.

## Outcome

The 22:00-state sweeps would now be accepted with the TEMPLATE_RESIDUAL advisory; the operator sees "template residual 1.2 mm (> 1.00)" on an accepted sweep. Nothing else in the node changes. The next sitting's first sweep reads whether the deformation persists; the ledger records it.

## Open Questions

- (a) Verified 2026-10-17 by reading the solver: matching is label-free and geometric, so a marker displaced beyond `CONSTELLATION_MATCH_TOL_MM` = 3 mm is never assigned and the fit proceeds on the remaining markers (floor `CONSTELLATION_MIN_MATCHED` = 3); the only trace is the unflagged `matched_min`-`matched_max` range. Accuracy cost is bounded by the Phase 2 leave-one-out at <= 0.09 deg of kappa for any single marker, so this is a reporting gap; parked: `plans/parked/bb-marker-never-matched-advisory.md` (triggers in its INDEX row).
- (b) The vertical-mode detection gap: an axis-tilt or per-marker z check would close it; not scheduled.
- (c) Whether the deformation persists is read by the next sitting's first sweep.
- (d) The hardware cause (ledger hypotheses H4-H6) is still open.
