---
title: "learner-generalisation merged to main, the live checkout rebuilt and every plant's memory migrated; the Next Sitting Runbook gains a `first:` key (JB-183)"
type: feature
date: 2026-11-06
status: complete
phase: "learner-generalisation merge"
related_plan: learner-generalisation.md
files_changed:
  - mapkit/sitting.py
  - tests/sim/test_sitting_sections.py
  - docs/agents/map-writes.md
  - map/feature-first-sitting-on-the-merged-learner-generalisation-stack.md
  - map/pathway-learner-generalisation.md
  - plans/active/learner-generalisation.md
  - docs/runbook.md
  - docs/project-map.md
  - logbook/2026-11-06-learner-generalisation-merge-and-runbook-first-key.md
  - logbook/INDEX.md
subsystem:
  - skill-stack
  - learner
  - tests-tooling
tags:
  - merge
  - memory-migration
  - runbook
---

# The merge of the five no-motion phases, and a way to say "fly this row first"

**What (owner's ask, 2026-11-06).** Branch `p3-online-admissibility` (P3a, P3b, P3c, P1, P4 of plan `learner-generalisation.md`) is merged to `main` as `e0edc2ea`. One conflict, `logbook/INDEX.md` (both sides added rows; both kept). Main's JB-176 change to `executor.py` merged without conflict; it touches the landing-row bookkeeping only.

**Done on the live box the same day.**
- The live checkout was 22 commits behind `origin/main`: the auto-pull skips a tree with a modified tracked file, and `temp/learn/jugglebot/memory.csv` carries 89 uncommitted rows. It was fast-forwarded by hand (`git merge --ff-only origin/main`; the merge does not touch that file, md5 identical before and after). The rows are still uncommitted, so the auto-pull stays blocked until they are committed.
- `colcon build --packages-select jugglebot_interfaces jugglebot`: 2 packages finished; `ros2 interface show jugglebot_interfaces/srv/InstallSegment` lists `check_only` and `margin`.
- `python tools/migrate_memory_schema_v2.py --memory temp/learn/<plant>/memory.csv --out temp/learn/<plant>/memory_v2.csv` for seven plants (`jugglebot`, `jb-139-test`, `p4-oval-2026-11-03`, `zvd-oval-2026-11-05`, `r4-20260928`, `r4-20260929`, `r5-20260930`): every row carried, 0 dropped (3365, 162, 75, 36, 18, 106, 58). Every `memory.csv` has the same md5 before and after. A copy of the migrated `jugglebot` file replayed through `PlantModel` + `Learner`: 3334 accepted, 31 rejected, 0 unusable, K 1.0773, b +5.4/+0.9 mm, s -0.0764/m.
- The three bags and three `hand_capture` CSVs of the unpowered measurement launches (2026-11-06 00:19, 00:37, 02:04) are deleted.

**The `first:` key.** The Runbook orders rows by priority weight per minute, so a 25-minute row can never be listed first, yet every other row now flies on the merged stack and its result is unreadable if that stack misbehaves. `mapkit/sitting.py` takes an optional `first: <reason>` in a `## Sitting` section: such a row sorts ahead of every scored row and its setup set opens the rig's first block; an empty `first:` is a parse error. `docs/agents/map-writes.md` documents it. The map app reads `collect()` and `render_runbook()` only, so it follows the new order with no change of its own.

**The row.** JB-183 (`first:`), child of JB-38: the unpowered `skills/check` / `skills/nominate` rehearsal, Columns 1.00 / Sep. 100 on the new stack (T-H3), then Columns at Sep. 170, a pair of Sites the memory has no rows at (T-H2). Offline nomination at 600 / 8000 / 200000, Hand 4500 put Sep. 170 at margin 0.21; Sep. 200 passes at 0.08 only, which is the thin-margin case the P4 entry found failing in the sim, so the row stays at 170.

**Verification.** Merge: 2026-11-06 12:46, `./run_tests.sh --full` on the merged tree before the push: 8754 passed, 51 skipped in the parallel phase, 10 passed in the serial phase, RESULT PASS. The `first:` key (tier 2, tooling): `grep -rn "mapkit.sitting\|import sitting" --include="*.py" .` finds `mapkit/gen_docs.py` and three test files, and `grep -rn "mapkit.sitting" tools` adds `tools/night_shift_candidate.sh` (it imports only `has_sitting_section`, unchanged); the two new tests in `tests/sim/test_sitting_sections.py` fail with `mapkit/sitting.py` stashed (2 failed, 21 passed) and pass with it restored. 2026-11-06 13:02, `pytest -q tests/sim/test_sitting_sections.py tests/sim/test_sitting_debrief_protocol.py tests/sim/test_night_shift_candidate.py tests/sim/test_night_shift_gate.py tests/sim/test_plans_index.py tests/sim/test_map_items.py tests/sim/test_map_items_rated.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py tests/sim/test_mapkit*.py`: 468 passed, 43 skipped.

**Not done.** No launch was started, so P3a and P3b stay in progress in the plan until the rehearsal in JB-183 is on record.
