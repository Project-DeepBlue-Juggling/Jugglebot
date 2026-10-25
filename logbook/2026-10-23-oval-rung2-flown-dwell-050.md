---
title: "Oval rung 2 flown: one-ball oval at dwell 0.50 caught 38/38, MET by owner GO"
type: feature
date: 2026-10-23
status: resolved
phase: "two-ball-oval - rung 2"
related_plan: two-ball-oval.md
related:
  - 2026-10-23-deactivate-rejected-state-machine-divergence
  - 2026-10-23-rest-rebase-stale-seed-hand-bump
  - 2026-10-23-oval-p2-banking-leak-not-reproduced
files_changed:
  - tests/hardware/session_oval_rung1.md
  - plans/active/two-ball-oval.md
  - plans/active/INDEX.md
  - logbook/INDEX.md
subsystem:
  - skills
  - testing
tags:
  - oval
  - hardware
  - rung-2
---

# Oval rung 2 flown, dwell 0.50 is the new one-ball baseline

## Summary
Rung 2 (one-ball oval at dwell 0.55 then 0.50 on the as-is `level` planner, today's limits) flew in two launches on 2026-10-23. Dwell 0.50 caught 38/38 throws (a 6-run and a 32-run), 0.55 4/4, 0.65 75/76 (the one miss was the cold first goal after launch). The seat bit (`ball_held`, debounced) stayed unbroken on 36/36 steady carries at 0.50. Leg 2 peaked at 10.05 A (15 A clamp), up 46 % on the rung-1 baseline. The three owed fixes verified in the logs. No planner code changed; this is a sitting read-back, docs only. The runbook's § 7.1 holds the full result tables.

## What flew
- **Launch A**, 15:53:59 (bag `2026-10-23_15-54-00`): before the box-cell commits, so 0.65 only. One-ball `oval_1ball` 4/4, three `columns_1ball_fed`/`columns_1ball` goals 2/2, then four two-ball `columns` goals that ended MISSED_CATCH (7/7, 3/3, 3/3, 29/29; see Residue). Satisfied R2-1/2/3.
- **Launch B**, 17:30:31 (bag `2026-10-23_17-30-31`, 22-cell box loaded): 17 dispatches, 14 ran, 3 refused (two `REJECTED_BALL_IN_HAND`, operator error, re-sent; one box refusal at dwell 0.40, by name, as designed). `oval_1ball_fed` at 0.65 (seven goals incl. a 32-run), `columns_1ball_fed` x2, one `oval_1ball_fed` at 0.55 (4/4), `oval_1ball` at 0.65 x2, at 0.50 x2 (6/6, 32/32). Every 0.55 / 0.50 goal COMPLETED.

## Owed fixes verified
- **R2-1 I-PLAN-11 rest_slew seed (daa495e0): MET.** About 16 `reload`/hop goals opened on a cross-site REST; zero `LIMIT_JERK` at t~0.025 (219k / 262k before). All seven sitting `LIMIT_JERK` hits are late re-aim or bump plans at t >= 0.175 s (204k-537k), ball caught each time.
- **R2-2 columns parity rule (21e02cfa): partly measurable.** Four `columns_1ball_fed` goals read 2/2 with no MISSED and no slam line. Cup SEATED at the end was not checked (no log line).
- **R2-3 feed-line latch (3fe2cdee): MET.** 8 `feed ball 1 CAUGHT` lines, all numeric, zero `--`.

## Rung 2 metrics (steady carries, bag B)
| metric | 0.65 baseline | 0.65 today | 0.55 | 0.50 |
|---|---|---|---|---|
| carry s | 0.68-0.70 | 0.68-0.71 | 0.58 | 0.53-0.55 |
| leg jerk median / max | 35.3k / 83.1k | 35.2k / 496k* | 55.3k / 58.9k | 66.3k / 77.1k |
| leg 1 / leg 2 iq max A | 4.53 / 6.90 | 5.15 / 7.44 | 6.07 / 7.76 | 7.49 / 10.05 |
| seat bit unbroken | 16/16 | 69/69 | 3/3 | 36/36 |
| raw dropout carries | 9/16 (56 %) | 30/69 (43 %) | 3/3 (n tiny) | 25/36 (69 %) |
| catches | 28/28 | 75/76 | 4/4 | 38/38 |

*Three 0.65 carries read echo jerk 417k / 476k / 496k (node tick-diff max 52-55k for the same goals); not at 0.50 / 0.55, not at rung 1. Undiagnosed, flagged as probe artefacts (the probe labels that column seam-prone). FET temperature was not measured in this read-back.

## Owner decisions (2026-10-23)
1. Rung 2 is GO at dwell 0.50 as the new one-ball oval baseline. Leg 2 peak 10.1 A of the 15 A clamp is accepted as margin. The 0.55 column (one 4-throw goal) is noted thin but moot since 0.50 passed.
2. Hand bump after feed catches: cause is a stale in-motion seed REBASE of the post-catch REST (2/2 reproduced vs 0/6, confirmed offline). Fix in flight this day with its own entry: `2026-10-23-rest-rebase-stale-seed-hand-bump`.
3. The four two-ball `columns` MISSED_CATCH ends in the 15:53 launch were real misses (see Residue); not a regression from the three unflown fixes.
4. Heartbeat dropouts: watch (see Residue).
5. The deactivate `ERR_REJECTED` at 17:40:21 was diagnosed and fixed, unflown: `2026-10-23-deactivate-rejected-state-machine-divergence`.

## Residue (open, unowned unless stated)
- **Two-ball `columns` lateral outliers.** Ends at throw 7, 3, 3, 29 of 50 (all compiled for 50; no Stop, no exhaustion). Ball 2 left with lateral launch error of 77 / 39 / 29 mm/s (G4's Ball 1: 191 mm/s) against a typical 5-30, giving 50-130 mm landing errors beyond the clamped aim authority (10 mm out, 35 mm in, 20 mm y); the cup read EMPTY for the whole window (55-59 samples). Raw markers confirmed G1 physically (ball ~110 x 126 mm off the cup). The streak lengths sit inside the 2026-10-13 sitting-8 curve (5, 5, 5, 0, 10, 4, 4, 11, 0, 10, 32, 9), and the three fixes do not touch the columns catch/throw path (21e02cfa reporting only, daa495e0 on the `reload` REST, 3fe2cdee report plumbing). The launch outlier's cause is undetermined; a refused re-aim before the release (`SPLICE_TOO_LATE` in G1/G2, a declined BUMP-LAUNCH in G4) is a candidate for some goals, none pins G3. **Confirming replay (not run):** `sim/skills_gate.py` two-ball columns, apex 0.95, separation 125-130, with a refused re-aim injected before a Ball 2 release, on a worktree at the parent of 21e02cfa vs HEAD, plus the lateral release velocity of the four failing throws vs the 29 good ones from `/leg_setpoint_echo`.
- **Heartbeat dropouts: WATCH.** 21 episodes in launch B (3 in A, 5 at rung 1). These are the per-axis ODrive CAN heartbeat on the leg bus (500 ms threshold, 10 Hz), not the UDP link; the known class of the archived `leg-bus-frame-drops.md`. Rate per minute of trajectory: 2.4 (B), 1.25 (A), 2.3 (rung 1), versus 1.7 on 2026-09-17; no more dropouts in-goal than between goals (1.7 vs 2.8 per min), dwell 0.50 not implicated, setpoint stream never gapped, fault state NONE, longest 1.2 s. **Escalate** on an episode approaching 2.0 s, a `MAX_DEVIATION` trip or catch miss coincident with a lead-clamp mask on the same axis, or a rate past about 4/min. Not ruled out: a long-uptime contribution (bridge ~247 h up; no freshly booted bridge to compare).
- **SPLICE_TOO_LATE:** 175 mentions in launch B (88 wire-deadline); not tallied per re-aim in this read-back.
- **0.55 thin:** one 4-throw goal, three steady carries; plain `oval_1ball` at 0.55 never flown.
- **0.65 echo-jerk spikes** (above): undiagnosed.
- **Dropout rise at 0.50:** `ball_held_raw` dropouts 56 % to 69 % of carries; the debounced bit held. P3's physical watch item.

## Verification
- 2026-10-23, `python tools/probes/oval_carry_bag_metrics.py /home/jetson/Desktop/rosbags/2026-10-23_17-30-31/2026-10-23_17-30-31_0.mcap --launch-log /home/jetson/.ros/log/2026-10-23-17-30-31-087871-jetson-428635/launch.log --out temp/probes/oval_rung2_1730`: ran (~3 min), goal parser worked, no `--windows` fallback; outputs `temp/probes/oval_rung2_1730/{row27.md,bag.pkl}`. Per-dwell split by goal order with a scratch script (windowing +-30 ms).
- 2026-10-23, `pytest tests/sim/test_plans_index.py tests/sim/test_docs_links.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py -q`: 123 passed in 0.98 s (docs-only change; these are the tests that read the plan, INDEX and logbook paths).
