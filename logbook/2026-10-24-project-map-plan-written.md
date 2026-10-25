---
title: "Project map implementation plan written from wayfinder map #21: six phases (mapkit, glossary rename, map/ seed and session wiring, fork skeleton, the two pages, Pi deployment)"
type: feature
date: 2026-10-24
status: resolved
phase: "project-map - design closed, implementation P0 next"
related_plan: project-map.md
files_changed:
  - plans/active/project-map.md
  - plans/active/INDEX.md
  - logbook/INDEX.md
subsystem:
  - docs
tags:
  - docs
  - planning
---

## Summary

`plans/active/project-map.md` lands: the phased implementation plan for the project map, the viewer app and the shared dictionary, written from wayfinder map #21 (github.com/Project-DeepBlue-Juggling/Jugglebot/issues/21) after its 16 tickets were resolved on 2026-10-23/24. The plan designs nothing; every decision is in the ticket answers, indexed in the plan's § 1. It is the first plan to carry the frontmatter it introduces (`pathway`, `branch`, `scope`, `steps`), ahead of the parser (P0) that will read them.

## Motivation

The map's Destination was "nothing left to decide before implementation starts and the plan is in `plans/active/`". The last two sessions closed the frontier: #24 (prototype: the thin file-backed server holds, so the pre-registered fallback of forking TaskTree whole was not taken), #31 (fork shape, Pi hosting), #30/#38 (intake, per-kind rendering), #25/#27 (item schema, plan frontmatter and generated INDEX files), then #28/#29/#32/#33/#34/#36/#37 by posted defaults with one owner confirmation.

## Discussion

Two owner decisions went against the recommendation and shape the plan: wayfinder maps move onto the new system (a `ticket` item kind, `mapkit frontier | claim | resolve`, #21 imported on day one) rather than staying on GitHub issues; and Claude moves map items through every transition, including triage. Two glossary calls also reversed drafts: "Wave" is the Freefly Wave camera and never a unit of work, and UH-n is vestigial with no entry. The plan's `steps:` list is named `steps`, not `phases`, because CONTEXT.md separates Phase from Rung and the oval plan has a Rung row.

## Verification

- 2026-10-24, `python -m pytest tests/sim/test_plans_index.py -q --noconftest` (the Windows box lacks `hypothesis`, so the repo conftest is skipped; the file is pure text): **84 passed in 0.13 s**. The first run failed `test_index_mentions_no_plan_that_left_its_directory[active]` because the INDEX row named `CLAUDE.md`, `CONTEXT.md` and `GLOSSARY.md` bare; the guard treats any bare `.md` name as a plan reference, so the row was reworded.
- No `*.py` or `*.yaml` changed; the only tests that read these paths are `tests/sim/test_plans_index.py` (run above) and `tests/sim/test_logbook_front_matter.py` (this entry's frontmatter has title, type, date, status).

## Outcome

Plan written and indexed; #35 and #21 closed with pointers. Next: a fresh session starts P0 (`mapkit/`). The two prototype servers from #24 were stopped.
