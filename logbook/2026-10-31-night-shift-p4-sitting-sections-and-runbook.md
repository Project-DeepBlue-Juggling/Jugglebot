---
title: "Night Shift P4: the ## Sitting section on hardware Items, the generated Next Sitting Runbook docs/runbook.md (rows scored by priority per minute, grouped by setup state), the candidate-branch stacker, and the result write-back"
type: feature
date: 2026-10-31
status: complete
phase: "night-shift P4"
related_plan:
  - night-shift.md
related_entries:
  - 2026-10-31-night-shift-p2-work-loop-gate-and-questions.md
files_changed:
  - mapkit/sitting.py
  - mapkit/gen_docs.py
  - mapkit/__main__.py
  - docs/runbook.md
  - tools/night_shift_candidate.sh
  - docs/agents/map-writes.md
  - CLAUDE.md
  - tests/sim/test_sitting_sections.py
  - tests/sim/test_night_shift_candidate.py
  - tests/sim/test_mapkit_docs.py
subsystem:
  - tests-tooling
  - docs
tags:
  - night-shift
  - runbook
  - map
---

# Night Shift P4: Next Sitting Runbook

## What

A hardware Item (`hardware: bench-leg|robot`) carries a `## Sitting` section with fixed keys
(`setup:` from `level|tilt|armed|bb-fed|bridge-cycled`, `why:`, `do:` hardware steps only,
`pass:`, `record:`, `minutes:`, optional `branch:`); `mapkit/sitting.py` parses it, scores each
row `priority weight x (2 if a held branch) / minutes`, groups rows into blocks sharing a setup
set, and `python -m mapkit gen` (or `runbook`) writes `docs/runbook.md`: a before-power block
from the standing facts, the blocks, `## Needs a Sitting section` for every hardware Item
without a valid section (the generator never invents a `do:` step), `## Questions a Sitting can
answer` from open question tickets under hardware Items, and a `## Close-out` ending in
`/sitting-wrapup <date>`. The page carries no wall-clock date so the currency test is
meaningful. `tools/night_shift_candidate.sh --date D [--push]` stacks every held
`night-shift/*-JB-n` branch whose Item has a section into `night-shift/sitting-D` in a
temporary worktree, reports conflicts as HOLD rows, and never touches `main`.
`append_sitting_result` is the write the map app's result box uses
(`## Sitting result (<date>)` with verdict and notes, placed before `## Progress`). On the
real board today all 23 open hardware Items are listed as needing a section; the first rows
appear as Sessions write them (CLAUDE.md now asks for that on filing or progressing one).

## Decisions taken (owner asleep)

- Setup sets compare as sets (`armed, level` and `level, armed` share a block); blocks are
  ordered by their best row.
- Items without `priority` read "unrated" and take weight 1.
- The candidate branch is a build product: rebuilt from base on every run, plain push only,
  `PUSH_REJECTED` rather than force.
- Candidate rows name the remote-qualified branch (`origin/night-shift/...`).

## Verification

- Scoped, 2026-10-31: `pytest -q tests/sim/test_sitting_sections.py tests/sim/test_mapkit_docs.py tests/sim/test_mapkit_gen.py tests/sim/test_map_items.py tests/sim/test_glossary_avoid_words.py tests/sim/test_docs_links.py` 47 passed; `pytest -q tests/sim/test_night_shift_candidate.py` 5 passed in 22.72s.
- Full gate before the commit, 2026-10-31 (`./run_tests.sh --full`): run 2026-10-31 00:56-01:04 on the final tree with no other load: **parallel 7764 passed, 65 skipped in 482.66 s; serial 8 passed in 32.55 s; exit 0** (an earlier run at 00:37, under five concurrent agents, had 5 parallel failures that passed scoped and 2 serial wall-clock failures, all load).
- The map app half (Next Sitting page, `GET /runbook`, result POST) is in the `jugglebot-map`
  repository: fork commit `1ca8001` (pushed to the fork's `main`), server suite 96 passed in 142.83 s with `MAPKIT_SRC` at this tree, frontend lint, typecheck, 322 vitest passed, build green; deployment and the HTTP check are recorded in the follow-up docs commit.
