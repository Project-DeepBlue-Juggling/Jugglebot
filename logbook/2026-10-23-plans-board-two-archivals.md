---
title: "Plans board: late-catch-reaim-bump and columns-empty-cup-two-feed archived as completed"
type: refactor
date: 2026-10-23
status: resolved
phase: "plans board housekeeping"
related_plan: late-catch-reaim-bump.md
files_changed:
  - tests/hardware/session_skill_stack_rehearsals.md
  - plans/archived/late-catch-reaim-bump.md
  - plans/archived/columns-empty-cup-two-feed.md
  - plans/active/INDEX.md
  - plans/archived/INDEX.md
  - tests/motion/test_skills_bump.py
  - tools/probes/late_reaim/parse_resends.py
  - tools/probes/late_reaim/bump_replay.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/bump.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - tests/ros/test_skill_node.py
  - ros_ws/gui/js/juggle-panel.js
  - tools/probes/capture_move_sim.py
  - logbook/2026-10-16-late-catch-reaim-bump-design.md
  - logbook/2026-10-17-columns-cup-keyed-accept-lob-retired.md
  - logbook/2026-10-19-held-axis-reload-catch-refuses-tracker-reaim.md
  - logbook/2026-10-23-plans-board-two-archivals.md
  - logbook/INDEX.md
subsystem:
  - skills
tags:
  - documentation
---

## Summary

Owner decision 2026-10-23: archive both plans as completed. Each was `git mv`'d unchanged in name to `plans/archived/` with `archived: 2026-10-23`, `status: completed` and an Archival-note section; both INDEX rows moved; path-qualified references (`plans/active/<name>.md`) in code comments, INVARIANTS.md (I-CATCH-5 row), the GUI panel comments and three logbook entries were re-pointed. Three logbook entries were resolved with an Outcome sentence.

## Close-out numbers

- **late-catch-reaim-bump** (criterion MET): 2026-10-21 74.1 % installed (20/27; refusals 3 BUMP-LAUNCH, 4 LIMIT_ACC); 2026-10-22 79.5 % (58/73; 6 LIMIT_JERK, 5 SPLICE_TOO_LATE, 3 BUMP-LAUNCH, 1 SERVICE_TIMEOUT); zero bump-caused LIMIT/guard events, 0 CHAIN_DISCONTINUITY; throw landing median 24.6 vs 29.7 mm (10-21) and 22.1 vs 21.9 mm (10-22), bumped vs unbumped goals.
- **columns-empty-cup-two-feed** (P3 flown): 6 empty-cup chains fed B, release-to-gate-open 6.4-6.8 s (n=6, upper bound from the log), 15 s refill constant kept; row 32 satisfied at separation 125; rows 33 (Stop between feeds) and 34 (gate timeout) unexercised on hardware; C-CUP-4 cone class did not recur in about 30 columns goals. Unowned residue (sim `--reload` LIMIT_JERK at 08ec3fe6, latent 'firing'-phase hang, feed y bias about 20 mm, rows 33/34) is listed in the plan's Archival note, unscheduled.

## Verification

- 2026-10-23, `pytest tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py tests/sim/test_docs_links.py -q` -> 123 passed in 0.97 s.
- 2026-10-23, `./run_tests.sh` (default gate; the .py/.js edits are comment-only) -> 7058 passed, 4 skipped, 1 failed in 343.70 s; serial tail 4 passed; the one failure is `tests/ros/test_replay_overview.py::test_a_pass_still_exiting_for_an_older_version_neither_reports_computing_nor_blocks_the_requeue` (a queue-vs-worker-thread race, untouched by this change), which passed 3/3 when re-run scoped (`pytest tests/ros/test_replay_overview.py -q`, 11 passed each time) - classed as a load flake, first occurrence, noted here so a second one gets a fix (`temp/logs/gate_archivals_20261023.log`).
