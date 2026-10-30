---
title: "Pattern progression P2: turned Columns on the compiled path (JP-10, goal columns_turned) - the Site pair rotates about the origin, turned Sites carry their own names so the box cannot collide, 12 pose cells at 45 and 90 deg swept"
type: feature
date: 2026-10-31
status: complete
phase: "pattern-progression P2"
related_plan:
  - pattern-progression.md
related_entries:
  - 2026-10-30-pattern-progression-plan.md
  - 2026-10-31-pattern-progression-p4-high-low-columns.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/sites.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/records.py
  - tools/admissible_sweep.py
  - sim/skills_gate.py
  - config/generated/admissible_box.yaml
  - docs/patterns.md
  - docs/records.md
  - tests/motion/test_skills_sites.py
  - tests/motion/test_turned_columns_box.py
  - tests/motion/test_skills_admissible.py
  - tests/motion/test_records.py
  - tests/ros/test_skill_node.py
subsystem:
  - motion
tags:
  - patterns
  - columns
  - admissible-box
---

# Pattern progression P2: turned Columns (JP-10)

## What

- `sites.columns_sites(separation_mm, angle_deg=0.0)`: the pair rotated about the Platform origin, P1 at (-half cos, -half sin), P2 at (+half cos, +half sin); 90 deg puts P1 at -y. Angle 0 returns the original arrays byte for byte (a test checks the bytes: a plain rotation would give -0.0).
- New goal `columns_turned` (JP-10, map JB-108): node parameter `columns_angle_deg` (refused while a goal is active, the `pre_release_hold_s` precedent), dispatched to the same Ball-Butler-fed two-ball `_run_columns` on the rotated pair. Refused by name under the `level` planner (direction-sensitive) and at angle 0 (that is plain Columns). The fed Site is the one nearest Ball Butler's Base marker along its bearing (`_columns_turned_feed_index`, tie within 1 mm picks P1; refused by name if the Ball Butler pose or the robot frame is missing). The goal start line and the ATTEMPT_START event gain a trailing angle field for this goal only; the parsers search the line, so nothing else changes.
- `records.PATTERNS` gains `columns_turned` ("Columns - turned", 2 balls); `docs/records.md` re-rendered (an empty table). `docs/patterns.md` JP-10 machine label filled.
- Sweep tool `--angle-deg` (pose Columns only) and `sim/skills_gate.py --angle-deg`.
- Box: `sites.py` is a gated file, so the whole box was re-swept (gate_hash 81a2244c1297 -> c6c4d7adccf9); the 54 committed cells are value-identical; 12 turned cells appended (both Sites x bands 0.825 / 0.875 / 0.925, dwell 0.27, separation 125, pose), then P4's 8 high-low cells (74 cells in all). Result: at 45 deg only the 0.925-0.975 band is admissible (landing windows a few mm wide, like the straight pose Columns cells); at 90 deg the 0.875 and 0.925 bands are. The 0.825 band is empty at every angle, as it is straight.

## Discussion

Names, not stamps. `admissible.select` already compares the Site xy stamps to 1e-6 mm, so a straight cell could never be selected for a turned pair. But the file-level uniqueness check keys on `(pattern, site_pair names, dwell_s, planner_mode)` plus band and ignores xy, so turned cells named P1/P2 would make the loader refuse the merged box. The turned Sites are therefore named `P1@45` / `P2@45` / `P1@90` / `P2@90`: the collision is impossible in the file, not only on select, and the fed-Site logic that reads names still works. The alternative, teaching the uniqueness check about xy, would have widened a contract for one caller.

Not decided here: whether the late catch re-aim (`bump.BUMP_ENABLED_PATTERNS`) should cover `columns_turned`; whether Ball Butler's landing bias (`columns_feed_bb_bias_mm`, a robot-frame offset) should rotate with the pair. Both wait for the first turned Attempt.

## Verification

- Re-sweep: see P4's entry (`2026-10-31-pattern-progression-p4-high-low-columns.md` § Verification): one script, two rounds, every family, byte-identical.
- Scoped tests and the full suite: the same section.
