---
title: "GLOSSARY gains Item ID (JB-<n>) after a grilling round; map-item-ids P6 complete"
type: refactor
date: 2026-10-27
status: resolved
phase: "map-improvements"
files_changed:
  - GLOSSARY.md
  - DOCUMENTATION_GUIDE.md
  - map/idea-glossary-add-item-id-jb-n-via-grilling-domain-modeling.md
  - docs/project-map.md
  - tests/sim/avoid_words_checked.txt
  - CLAUDE.md
  - docs/glossary.md
  - plans/active/map-item-ids.md
  - tests/sim/test_glossary_avoid_words.py
subsystem:
  - docs
tags:
  - project-map
  - glossary
---

## Summary

`GLOSSARY.md` gains **Item ID** in the Project map group, right after **Item**. This closes Idea JB-63 and the last step of `plans/active/map-item-ids.md` P6. The term went through `grilling` + `domain-modeling` as `CLAUDE.md` requires, and the owner agreed every recommendation on 2026-10-27:

- **Name:** Item ID. "Short ID" stays an allowed description, not an _Avoid_ word: it is the app's `short_id` field and a plain description of the same thing, not a competing concept.
- **Definition:** it states the domain facts only.
  - One sequence across every kind of Item, never reused.
  - It changes only before publication or to repair a duplicate. That covers the edge case where two sessions file the same number and the repair renumbers one of them after both were pushed.
  - Plans, Steps and logbook entries have none.

  Storage details (the `id:` field, references in files staying stems) stay in `DOCUMENTATION_GUIDE.md` § 2.10.
- **Avoid:**
  - `ticket number`: Ticket is one kind of Item, and every kind has an Item ID.
  - `issue number`, as a name for an Item ID only: a GitHub Issue's own number is a different, legitimate thing.
- **Enforcement:** `ticket number` joins `tests/sim/avoid_words_checked.txt`, since its sweep is clean. `issue number` does not, because its GitHub sense would need an exemption on every use.
- **Never reused, made true:** the audit noticed that allocation is one past the highest ID present, so deleting the top Item and pushing would hand its number out again. Items were never deleted in practice, since `dropped` exists for that. `DOCUMENTATION_GUIDE.md` § 2.10 now says so (owner, 2026-10-27), and the glossary keeps "never reused".
- **No ADR:** `docs/adr/` holds hardware-architecture decisions, and the plan's § 2 records the ID decisions.

`python -m mapkit gen` regenerated the `CLAUDE.md` term block (972 characters, under the 1200 ceiling) and `docs/glossary.md`.

## Verification

- 2026-10-27, `pytest -q tests/sim/test_glossary_avoid_words.py tests/sim/test_mapkit_glossary.py tests/sim/test_plans_index.py tests/sim/test_mapkit_docs.py`: 51 passed (the Avoid scan, its pinned phrase list, the term block and the generated boards).
- 2026-10-27, `./run_tests.sh`, after the audit fixes: PASS, 7362 passed + 5 serial, 357 s. The run before the fixes failed one test, `tests/ros/test_replay_overview.py::test_a_pass_still_exiting_for_an_older_version_neither_reports_computing_nor_blocks_the_requeue`, which passed 5/5 alone and is filed as a load flake.
