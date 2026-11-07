---
title: "Per-kind Item IDs P0: GLOSSARY gains Kind, Item ID and Ticket are rewritten after a grilling round, ADR-0015 records the reversal"
type: refactor
date: 2026-11-07
status: resolved
phase: "map-improvements"
files_changed:
  - GLOSSARY.md
  - CLAUDE.md
  - docs/glossary.md
  - docs/adr/0015-per-kind-item-ids.md
  - docs/adr/index.md
  - mkdocs.yml
  - plans/active/item-id-per-kind.md
  - logbook/INDEX.md
  - map/feature-update-item-naming-convention-based-on-type-eg-pathways-can.md
  - docs/project-map.md
subsystem:
  - docs
tags:
  - project-map
  - glossary
---

## Summary

P0 of `plans/active/item-id-per-kind.md` is complete. No Item changes ID here: every Item still carries `JB-<n>`, and `mapkit` still allocates it.

The owner settled five points in one `grilling` + `domain-modeling` round:

- **Item ID** is now a kind's prefix and a number counted within that kind, and a number alone names nothing. A dropped ID is never reused, and a promoted Idea keeps its ID while the new Feature gets its own.
- **The glossary runs ahead with one temporary sentence**, saying Items carry the retired `JB-<n>` until the cutover. `JB-<n>` stays off `_Avoid_` until P5, because the Night Shift reads the glossary as normative and still has to write those IDs. `ticket number` stays on `_Avoid_`.
- **Kind** is a new term. It exists because the request said "type", and a Ticket's `type` is a different thing. "Skill kind" and the Avoid-word test's list are unchanged.
- **Ticket** is corrected, and all Tickets share `TKT`. The old entry said every Ticket sits under a Pathway, but 29 of the 47 are question Tickets under a Bug, Feature or Idea.
- **ADR-0015** records the reversal of `plans/archived/map-item-ids.md` § 2 decision 1, since the plan holding that reasoning will be archived.

## Verification

Tier 3, named tests (2026-11-07, `pytest -q tests/sim/test_mapkit_glossary.py tests/sim/test_glossary_avoid_words.py tests/sim/test_mapkit_docs.py tests/sim/test_docs_links.py tests/sim/test_logbook_front_matter.py tests/sim/test_plans_index.py tests/sim/test_map_items.py tests/sim/test_map_items_rated.py tests/sim/test_night_shift_gate.py`): **343 passed, 48 skipped**. Read by trace: `test_mapkit_glossary.py` and `test_mapkit_docs.py` pin the generated term block and `docs/glossary.md` to `GLOSSARY.md`, `test_plans_index.py` pins `steps:` to the step headings and the boards, and `test_docs_links.py` follows the new ADR's links.

One `/audit --unstaged` pass ran at the end of the phase: 2 warnings and 2 notes, all narrative, all applied. They moved ADR-0015 out of the CAN group in `docs/adr/index.md`, corrected its Context to state all four reasons the archived decision gave, dated its census figures, and fixed this entry's `files_changed`.

Plan correction the same day: P1 as first written changed `ids.parse` to return a string, which would have broken the live map app within two minutes of the push, since the app imports mapkit from a clone on a pull timer and treats that value as an int. P1 now adds new names (`parse_ref`, `canonical_ref`, `format_ref`), keeps the old ones until P5, and gains check T-I0.
