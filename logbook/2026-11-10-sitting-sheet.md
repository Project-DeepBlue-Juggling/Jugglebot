---
title: "Sitting sheets: /sitting-sheet merges a rig's Items into one ordered page the app steps through (IDEA-19)"
type: feature
date: 2026-11-10
status: complete
phase: "IDEA-19"
related_plan: []
files_changed:
  - mapkit/sheet.py
  - mapkit/__main__.py
  - tests/sim/test_mapkit_sheet.py
  - tests/sim/test_mapkit_no_ros.py
  - .claude/commands/sitting-sheet.md
  - .claude/commands/sitting-debrief.md
  - docs/agents/map-writes.md
subsystem:
  - map-improvements
tags:
  - sitting
  - runbook
  - map-app
---

**What.** `/sitting-sheet <rig>` has a Session merge the open `## Sitting` sections into `map/sheets/<date>-<rig>[-N].yaml`, one ordered list of steps. The map app's Next Sitting page now opens the newest sheet: per step a start state, the Terminal or GUI action, what to see, what stops you, what to record, the Items served, and boxes. App: `jugglebot-map` commits on main (`/sheets` routes, `SheetView`), the old per-Item page kept behind `All Items`.

**Why.** IDEA-19: the Runbook groups by Item, so an operator had to merge the steps in their head. Each step must be doable without reading ahead.

**How it stays honest.** A `do:` step is free text, so a model has to order them. `mapkit sheet check` holds the result to the Items: every `do:` placed or omitted with a reason, every quoted command verbatim, each Item's order kept, an unbroken state chain, one verdict step per Item, whose start state includes the Item's `setup:`. A cold read by a fresh agent covers what no tool can see.

## Discussion

**The order changed.** TKT-53 chose one hand-built sheet flown first, then the command, because a page built on an unflown format risks rework. The owner chose on 2026-11-10 to build the command and the page together, with a Sitting the same day. The accepted cost: any format change after the first flight touches both repos.

**One verdict per Item on a flown step, not one per step.** `/sitting-debrief` merges a held branch on a `pass`, and two Items on one flight can come out differently, so one box could pass an Item whose own criterion failed. Setup and restore steps take done or problem. Verdicts still land as `## Sitting result`, so the debrief's input is unchanged.

**Re-saves.** The first build appended a result on every save. A comment edit on a verdict step re-posted the verdict, and a corrected `pass` left a stale `pass` above it. An unchanged verdict is now a no-op on the Item, and the debrief takes the last result in file order as the verdict.

**What the check caught on the first real sheet.** It refused a step that Levelled and Activated inside BUG-82's verdict step. The cold read found what the check can't: FEAT-32's canary flying on the oval plant, a venv leaking into the shell that then built ROS, and a skip that jumped over FEAT-39's verdict step. Two rules were loosened by real cases: `reorder: <why>` for an Item whose `do:` states a precondition after the steps it governs, and a bare `ros2 param get` is not a command to copy.

**The first sheet went stale before it flew, and would not load.** A peer Session flashed can-bridge FW 43 and added a BUG-7 Sitting section after the sheet was written, so its v42 stop would have stopped the first launch. Separately, the page refused the whole sheet because a ride-along serve cited no `do` step. The check now compares any `can-bridge v<N>` on a sheet with the tree's expected firmware, and the app runs the check on every load and shows a banner, so drift after writing is visible at the robot. `enrich` always sends `do`. `setup_waived: <why>` lets BUG-7 read back what a relaunch restored before that launch levels again.

**The page after its first Sitting (owner feedback, same day).** The sheet page became a dense protocol view: launch groups with Item sub-blocks, one-click done on steps that decide no verdict, comments behind an icon, an Item picker that folds the steps of deselected Items, and a tooltip on each start-state word from `STATE_HELP` in `mapkit/sheet.py`. BUG-7 was closed during the Sitting and the page then called the sheet stale. The app's check now runs with `live=True`, which accepts a step serving an Item closed since the sheet was written; the author's check still refuses one.

A simple step can also be saved `skipped` (not flown this Sitting; owner, 2026-11-10), which folds it like done and tells the debrief the step is no evidence either way. The `tilt` state word, coined in Night Shift P4 and used by no Item yet, now reads as the owner defines Tilt: a held angle about x and/or y.

**The rehearsals on the first sheet were real goals (found at the Sitting, 2026-11-10).** Three Items' rows said to rehearse by sending a Juggle goal with the Cup empty and Ball Butler off, "nothing moves", and the sheet put those goals before Level with a note that a WRONG_MODE line was expected. Read from `skill_node.py`: an empty Cup is not a refusal for a fed Pattern, WRONG_MODE is the pre-level move's hard refusal outside TRAJECTORY mode, and once the Platform is Active the goal is accepted, the Platform makes its pre-level move and the Hand homes before the Ball Butler refusal ends the Attempt. The owner flew steps 7 and 8 as real Attempts. The class is closed three ways: the rows of FEAT-39, IDEA-10 and IDEA-11 now rehearse with `/skills/nominate`; `/sitting-sheet` states the chain Boot > Home > Level > Activate > goal and one nominate rehearsal step per launch after Level; and the sheet check fails any step that sends a Juggle goal without starting from `launched, level, armed`. The 2026-11-10-robot-2 sheet has Level at step 6 and one four-call rehearsal at step 7 (42 steps). Not flown yet: `/skills/nominate` for these Patterns on the live robot.

`tests/sim/test_mapkit_no_ros.py` now exempts `sheet.py` from the word `ros2` only: the module names that CLI in strings. It still may not name a ROS Python package, and the import-graph test still holds.

**Flown.** The 2026-11-10 Sitting flew steps 1 to 8 of the first sheet from the app, results saved per step; the rehearsal-as-goal defect above was its one failure. Closed with IDEA-19 by `2026-11-10-sitting-debrief.md`.

## Verification

- Scoped run (2026-11-10, `python -m pytest tests/sim/test_mapkit*.py tests/sim/test_sitting_sections.py tests/sim/test_sitting_debrief_protocol.py tests/sim/test_map_items.py tests/sim/test_night_shift_gate.py tests/sim/test_docs_links.py tests/sim/test_logbook_front_matter.py tests/sim/test_plans_index.py -q`): the count and result are in the commit message.
- Full gate (2026-11-10, `./run_tests.sh --full` on 92c404e2): 9301 passed + 10 serial, **1 failed**: `test_no_legacy_item_ids`, because the sheet quoted BUG-63's plant name, which began with an old-form Item ID. The scoped run had missed it: that guard scans the whole tree and imports nothing from `mapkit`, so a grep for importers can't find it. Fixed at the source (BUG-63 and today's sheet now say `bug63-fresh-`; the name is arbitrary), the pins regenerated.
- App server (2026-11-10, `MAPKIT_SRC=<worktree> python3 -m pytest -q` in the jugglebot-map worktree): 132 passed.
- App frontend (2026-11-10, `npx tsc -b --noEmit`, `npx eslint . --max-warnings 0`, `npx vitest run`): clean, 346 passed and 1 skipped.
- `python -m mapkit sheet check map/sheets/2026-11-10-robot.yaml` (2026-11-10): `sheet check: OK`, 44 steps.
