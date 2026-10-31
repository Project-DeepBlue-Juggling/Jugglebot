---
title: "Pattern progression P4: per-ball Apex in the Columns compile (JP-9, goal columns_highlow) - one Beat from the high ball, the low ball dwells longer, its own box label; the sweep finds 1.00/0.90 admissible at separation 100 and nothing at 125"
type: feature
date: 2026-10-31
status: complete
phase: "pattern-progression P4"
related_plan:
  - pattern-progression.md
related_entries:
  - 2026-10-30-pattern-progression-plan.md
  - 2026-10-31-pattern-progression-p2-turned-columns.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/admissible.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/records.py
  - tools/admissible_sweep.py
  - sim/skills_gate.py
  - config/generated/admissible_box.yaml
  - docs/patterns.md
  - tests/motion/test_skills_schedule.py
  - tests/motion/test_skills_executor.py
  - tests/motion/test_skills_admissible.py
  - tests/motion/test_records.py
  - tests/ros/test_skill_node.py
subsystem:
  - motion
tags:
  - patterns
  - columns
  - admissible-box
  - schedule
---

# Pattern progression P4: high-low Columns (JP-9)

## What

- `schedule.Pattern.apex_m_ball_b` (None = same as `apex_m`; Ball 1 = schedule id 0 keeps `apex_m`, Ball 2 = id 1). `columns_ball_timing(apex_a, apex_b, dwell)` is the one place the arithmetic lives, used by the compile and the sweep: one shared Beat from the HIGH ball, `beta = (t_high + d) / 2`; the high ball keeps the commanded Dwell, the low ball's dwell is `d + (t_high - t_low)`; the empty transit before catching ball i is `t_i - beta`, about 45 ms shorter for the low ball at 1.05/0.95. The Schedule's scalars are the high ball's; `dwell_by_ball` / `flight_by_ball` are set only when the Apexes differ, read through `dwell_for(id)` / `flight_for(id)`. Compile refuses by name when a ball would have no transit.
- Executor: every `adm.select` / `describe_miss` reads `_box_dwell_s(idx)` = the planned Skill's ball's dwell, so the low ball's cell (stamped at its longer dwell) is the one selected. The phantom first-catch landing and `skills_gate._columns_final_removal` read `flight_for`.
- Box label `columns_highlow` (`admissible.PATTERNS`, `schedule.COLUMNS_HIGHLOW`), not `columns` (see Discussion).
- New goal `columns_highlow` (JP-9, map JB-107): node parameter `apex_m_ball2` (refused while a goal is active), one accept-time message naming every refusal at once (parameter unset, equal to `apex_m`, planner not `pose`), the box check per Site at that ball's Apex and dwell. In a fed start with the default feed Site P1, Ball 1 cycles at P2 and Ball 2 at P1; the sweep covers both assignments. Trailing `apex2` field on the goal start line and event, this goal only. Not done: the GUI Juggle panel has no entry yet, so the goal is sent with `ros2 action`.
- Sweep tool `--apex-pair HIGH LOW` (`highlow_sweep`, the existing `sweep()` untouched): four pose cells per pair, both Sites x both assignments, each stamped with its ball's dwell; `sim/skills_gate.py --apex-ball2`.
- `records.PATTERNS` gains `columns_highlow` ("Columns - high-low", 2); `docs/patterns.md` JP-9 label filled.

## Result of the sweep (the viability question the owner asked)

| pair | separation 125 | separation 100 |
|---|---|---|
| 1.05 / 0.95 | all four cells EMPTY | high cells admissible (x [0, 10] / [-10, 0], y +-20 mm; Apex clip tops out at 1.05 = the Hand limit); low cells EMPTY |
| 1.00 / 0.90 | all four cells EMPTY | all four admissible (x windows 6 to 20 mm, y +-20 mm) |

At 125 mm the low ball's transit (0.283 s at 1.05/0.95, 0.270 s at 1.00/0.90) falls under the ~0.30 s empty-transit floor the catalogue already names; the refusals are HAND_LIMIT_ACC on the low catch after a 1.05 launch, CHAIN_CATCH:MARGIN on the high cell, LIMIT_VEL for the lower pair. So JP-9 flies at separation 100 with the 1.00/0.90 pair; 1.05/0.95 waits for the Hand thread (JB-114). Only the 8 separation-100 cells are in the box: the 125 cells carry the same Site names, dwell stamps and bands and differ only in xy, which the loader's file-level uniqueness key ignores (the same key that made P2 name its turned Sites `P1@45`), so merging both would be refused; a goal at 125 is refused by a box miss, which is the same fail-closed outcome. The finding is pinned by `test_pattern_progression_cells_are_the_tail_after_the_62`.

## Discussion

Its own box label. The brief said label `columns`; the implementation chose `columns_highlow` and the reviewer agreed: each high-low cell is planned from the OTHER ball's release at a different Apex, so a plain Columns goal at 1.00 or 1.05 (runbook § 11 flies those Apexes at dwell 0.27) would otherwise select a high cell planned from the wrong starting state, and plain cells later swept at those bands would overlap under one key. With the separate label the collision with the 54 committed cells is impossible and the loader's uniqueness check passes for both pairs.

Bit-identity. The equal-Apex path runs today's exact expressions with no float reordering: 672 configurations (separation 100/125, Apex 0.85 to 1.00, dwell 0.27/0.30, n 1 to 20, phantom balls none/0/1, free and fed starts) compiled with the old and the new `schedule.py` differ in no field of any Skill or Schedule. No gated file changed in P4, so the re-sweep's second round, which ran on the merged P2+P4 tree while the first ran on P2 alone, is the box-level proof: byte-identical.

Not settled: the Learner's memory rows are keyed by Site and target, so with both assignments flown one Site mixes 1.05 and 0.95 targets; whether the neighbourhood separates them by Apex is unverified. The runbook § 11 flies Columns at "Sep. 100" while the committed straight Columns cells are at 125 and `select` matches xy to 1e-6 mm; that is the oval Pathway's runbook (JB-40) and is flagged to the owner, not changed here.

## Verification

- Whole-box re-sweep (2026-10-31 00:01 to 02:00, `temp/sweeps/p2p4/sweep_p2p4.sh` + `new_cells.sh`: the 14 pose-default invocations + the 4 apex-extension oval invocations + 2 turned-Columns invocations per round, BLAS one thread, two rounds, round a on the P2 tree and round b on the merged P2+P4 tree): all 20 files byte-identical by `cmp`; the merge asserted the 54 committed cells value-identical; gate_hash c6c4d7adccf9 = `admissible.gate_hash()` on the final tree, swept_at 2026-10-31. Scripts, logs and per-family files kept under `temp/sweeps/p2p4/` (gitignored).
- High-low sweep (2026-10-31 00:45 to 01:20, `temp/sweeps/p2p4/sweep_hl.sh`, 4 invocations per round, two rounds): all 4 files byte-identical (`cmp`); the 8 separation-100 cells appended by `merge_final.py` (the 125 cells excluded, above); the loader's uniqueness check passes on the 74-cell box.
- Scoped, by the implementing agents before the merge: P2 (2026-10-30, `pytest -q tests/motion/test_skills_sites.py tests/motion/test_turned_columns_box.py tests/motion/test_records.py tests/ros/test_skill_node.py tests/sim/test_docs_links.py tests/sim/test_skills_gate.py tests/sim/test_plans_index.py`): 468 passed, 21 failed, every failure the stale box's gate hash (sites.py is gated) or the records fixed point, both cleared below. P4 (2026-10-31, `pytest -q -n 4 --dist loadfile -m "not nightly" tests/motion/test_skills_schedule.py tests/motion/test_skills_executor.py tests/motion/test_skills_admissible.py tests/motion/test_records.py tests/ros/test_skill_node.py tests/sim/test_skills_gate.py tests/sim/test_docs_links.py tests/motion/test_hold_rung_bias_probe.py`): 1029 passed, 1 failed (the records fixed point, cleared by re-rendering `docs/records.md`). Merged tree before the box landed (2026-10-31 00:30): 1028 passed, 32 failed, all 32 the stale box.
- Full suite (2026-10-31 02:30, `./run_tests.sh --full` on the final tree, log `temp/logs/p2p4_full2.log`): **parallel 7812 passed / 63 skipped in 367.76 s + serial 8 passed in 30.16 s, exit 0**. A first run at 02:03 had 3 failures, all tests whose premise the new cells changed (the sweep's own no-collision check now meeting its own committed cells; the node-default-dwell pin meeting the low ball's longer dwell stamp); both tests were taught the exception and the suite re-run.
