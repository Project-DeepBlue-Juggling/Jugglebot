---
title: "Pattern progression plan: Pathway JB-116 written up as plans/active/pattern-progression.md (ten steps, no new design)"
type: process
date: 2026-10-30
status: complete
phase: "pattern exploration"
related_plan:
  - pattern-progression.md
related_entries:
  - 2026-10-30-pattern-catalogue-jp-ids.md
files_changed:
  - plans/active/pattern-progression.md
  - plans/active/INDEX.md
  - docs/project-map.md
subsystem:
  - docs
tags:
  - patterns
  - plan
  - map
---

# Pattern progression plan

## What

`plans/active/pattern-progression.md` is the schedulable form of Pathway JB-116: its `steps:` mirror the Pathway's ten steps (P1 to P10) so the map app shows the step table. Sections: Context, Decisions (ordering principle, owner decisions D-a to D-d with when each is needed), Where we are, the steps with their gates, Runs in parallel, Not in this plan.

## Why

The Pathway body carries the steps as prose; a plan gives each a status, effort and hardware tag and a gate. Written from the Pathway item and the Opus review of the first proposal, which found the Apex lift is not on the critical path (it is Hand work, JB-114) and that per-Beat Sites are already owned by JB-38. No design is added; two choices were mine: step ids use the P prefix for Rungs too (heading `P3`, steps `kind: rung`) so the ids stay P1 to P10, and effort for the shared Ideas (JB-107, JB-109, JB-111) is split across their Phase and Rung.

## Verification

`OPENBLAS_NUM_THREADS=1 pytest -q tests/sim/test_plans_index.py tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py tests/sim/test_map_items.py tests/sim/test_mapkit_docs.py tests/sim/test_logbook_search.py`, run 2026-10-30 in the scratchpad worktree at origin/main a7728227: **66 passed in 3.30 s**.
