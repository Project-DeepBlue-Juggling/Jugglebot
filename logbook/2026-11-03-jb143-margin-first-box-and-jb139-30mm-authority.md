---
title: "Box re-swept with the landing rectangle chosen room-around-the-target first and a +/-30 mm grid; Learner lateral authority 20 -> 30 mm for Throw and catch re-aims"
type: feature
date: 2026-11-03
status: complete
phase: "pattern progression"
related_plan: pattern-progression.md
related_issues: []
files_changed:
  - tools/admissible_sweep.py
  - tools/admissible_sweep_run.py
  - tests/motion/test_admissible_sweep_run.py
  - config/generated/admissible_box.yaml
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/bump.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/learner.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - sim/skills_gate.py
  - tools/probes/capture_move_replay_extract.py
  - tests/motion/test_skills_admissible.py
  - tests/motion/test_cup_capture.py
  - tests/motion/test_skills_capture_replay.py
  - tests/ros/test_skill_node.py
  - tests/sim/test_skills_gate.py
  - map/bug-box-at-hand-acc-4500-traded-hop-landing-width-for-apex-band.md
  - map/bug-ball-2-learner-command-pinned-at-the-20-20-mm-clamp-corner.md
  - docs/project-map.md
  - docs/runbook.md
  - logbook/2026-11-03-jb143-margin-first-box-and-jb139-30mm-authority.md
  - logbook/INDEX.md
subsystem: [motion, tools]
tags: [admissible-box, learner, testing]
---

# Box re-swept margin-first on a +/-30 mm grid; Learner authority 30 mm (JB-143, JB-139)

Items: JB-143 (closed here), JB-139 (progressed). Owner decisions 2026-11-02:
- the landing rectangle is chosen with the room around the target first (the smallest distance from (0, 0) to an edge), then area;
- the Apex band is only grown where it costs the rectangle nothing;
- the Learner gets +/-30 mm for both Throw and catch re-aims;
- the Learner should have more authority, not less, at higher Apex.

## What changed

- **`tools/admissible_sweep.py`**:
  - `_max_rectangle` ranks rectangles by `_rect_key` = (margin about (0, 0), area, grid points), computed from a prefix-sum table.
  - `_flight_band_and_rect` fixes that rectangle at the centre flight first. Among contiguous bands that keep it, it takes the widest.
  - `OFFSETS_MM` gains +/-25 and +/-30. `HOP_OFFSETS_MM` stops at +/-25, because `check_site_workspace` has a 150 mm bound.
  - The docstring now states the one-round-plus-spot-check sweep norm (owner, 2026-11-03).
- **Learner authority**: `skill_node` sets `learner_lateral_authority_mm` 20 -> 30. That clamp also bounds catch re-aims (`_clamp_lateral_to_schedule`), so the gated `cup_cycle.CAPTURE_MAX_OFFSET_M` follows it: `hypot(0.035, 0.020)` = 40.31 mm becomes `hypot(0.035, 0.030)` = 46.10 mm. INVARIANTS I-CATCH-6 is updated first.
- **The box** `config/generated/admissible_box.yaml`:
  - 104 cells, gate_hash 12ec30249323, limits unchanged (350 / 5000 / 200000, Hand 4500).
  - Same 30-family set and cell order as the box it replaces (gate_hash 242c56b683b5).
- **`tools/admissible_sweep_run.py`** (the JB-150 runner, merged earlier the same day):
  - Its default family set `committed` is now the 30 families of this box; it was the 26 of the box before C-HAND-5, and `--set h4500` keeps those 26.
  - The leg and Hand limits default to the committed box's `limits` stamp, with `--leg-vel`, `--leg-acc` and `--leg-jerk` flags for a working-point change.
  - Its family arguments and merge order match the sweep that built this box, checked argument by argument.
  - Its round b is already the owner's 2026-11-03 norm: a 3-family sample compared byte for byte.
- **`test_release_step_resweep_kept_the_pose_cells...`** pinned the 38 untilted pose cells bit-identical to the pre-release-step box. It now pins that each keeps at least its old room, which is what the new rule guarantees on an unchanged pass set.

## Result (sweep 2026-11-03)

- **Determinism.** Round a ran 14:05-16:04 and round b 16:04-17:46: `CMP_STATUS 0`, all 30 family files byte-identical. The loader accepts 104 cells at the live gate_hash.
- **Cells changed.** 81 of 104 cells changed: room grew in 65, stayed equal in 16, and **fell in none**.
- **Room histogram, old -> new.**
  - 0 mm: 4 -> 0 (Columns pose 1.25 at both Sites, P1 at 1.40, Turned Columns 45 P2 at 0.95).
  - 2 mm or less: 11 -> 8.
  - 25 mm or more: 7 -> 57.
- **Columns pose by Apex** (the prediction `columns_apex_h70.md`, run on the h70 logs, held wherever its grid reached):
  - 0.95: room 2 (feasibility MARGIN, unchanged).
  - 1.00: 6.
  - 1.05-1.10: 10.
  - 1.15: 10.
  - 1.20: 20.
  - 1.25-1.50: 25-30.
  - The owner's "more authority at higher Apex" holds.
- **Accepted cost.** 46 Apex bands narrowed, mostly on the low side, where no operating flight goes. Bands that now END at the cell's centre Apex:
  - Columns level 0.95 (top) and 0.90 (bottom).
  - Turned Columns 45 P2 at 0.95 (top).
  - Columns pose P2 at 1.15 (top).
  - Oval 1-ball level, dwell 0.40 (top) and 0.55 (bottom).
  - Columns pose at 1.40-1.50 is at the centre as before, at the grid top.

  In these cells the Learner's Apex command clips to one side.

## Discussion

- **Why room-first, not area-first.** The night-shift JB-143 branch (f462835b, merged by a peer the same day) also fixes the rectangle before the band, but picks it by area. A long thin rectangle wins on area while leaving the target 2 mm from an edge. The Learner's corrections are centred on (0, 0) and roughly symmetric, so the number that limits it is the nearest edge. This entry's `_max_rectangle` / `_flight_band_and_rect` supersede f462835b's.
- **Area before band, a non-obvious consequence.** With the +/-30 rows added, a rectangle that gains y to -30 at equal margin outranks a wider Apex band. That is why Columns pose 1.00 went from band 0.640-1.210 to 0.902-1.210 while keeping room 6. The owner ranked landing room over Apex range. Most of the Apex range lost is far below anything flown.
- **What is still narrow.** The +/-2 mm at Columns 0.95 is feasibility MARGIN (the 90 % leg-limit margin) at the centre flight itself. No selection rule can widen it. Any change there is an owner limit decision. P4's leg working point (600 / 8000) is the next re-sweep and should move it.

## Verification

- 2026-11-03, `WT=<wt143> CAPTURE_CAP_MM=46.10 bash sweep_jb143.sh` (30 families, two rounds, 6 workers, BLAS 1): CMP_STATUS 0, 104 cells, gate_hash 12ec30249323.
- 2026-11-03, `pytest -q tests/motion/test_skills_admissible.py tests/ros/test_skill_node.py tests/motion/test_cup_capture.py tests/motion/test_skills_capture_replay.py tests/sim/test_skills_gate.py`: 563 passed (with the history test fixed).
- 2026-11-03, `./run_tests.sh --full` (this tree, rebased on 341950b5): **PASS**, parallel 8119 passed / 52 skipped in 422 s, serial 8 passed, total 459 s.
- One `/audit` pass (Sonnet audit-reporter): no correctness findings. Five stale comments and a test gap were fixed:
  - 20 mm references in `executor.py`, `bump.py`, `learner.py` and `test_skill_node.py`.
  - The 38-cell history test also pins each new Apex band inside the old one.
- After those fixes, 2026-11-03, `./run_tests.sh`: **PASS**, 8119 passed / 52 skipped parallel, 5 serial, 425 s. The gate_hash is still 12ec30249323.
