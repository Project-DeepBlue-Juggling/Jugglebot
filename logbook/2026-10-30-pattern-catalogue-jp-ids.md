---
title: "Pattern catalogue: every Pattern gets a JP-n ID in docs/patterns.md (owner decision), the two-ball exploration's eighteen Patterns registered"
type: process
date: 2026-10-30
status: complete
phase: "pattern exploration"
related_plan:
  - two-ball-oval.md
related_entries:
  - 2026-10-30-two-ball-oval-p3-regate-go-and-dwell-headroom.md
files_changed:
  - docs/patterns.md
  - mkdocs.yml
  - map/feature-pattern-catalogue-give-every-pattern-a-jp-n-id-a-name-and.md
subsystem:
  - docs
tags:
  - patterns
  - catalogue
  - map
---

# Pattern catalogue with JP-n IDs

## What

A new published page `docs/patterns.md` (nav entry "Patterns") registers every Pattern with a catalogue ID JP-n, its name, machine label(s), Site count, what moves, status (flown / destination / idea / unfiled) and where it lives. Flown: JP-1 Self-throw, JP-2 Hop, JP-3 Columns one ball, JP-4 Oval one ball, JP-6 Magazine throw, JP-7 Columns. Destination: JP-8 Oval. Ideas from the 2026-10-30 two-ball pattern exploration: JP-5, JP-9, JP-10, JP-11, JP-13, JP-14, JP-15, JP-17 (map JB-106 to JB-113). Unfiled: JP-12, JP-16, JP-18. Closes JB-105.

## Why

Owner, 2026-10-30: "We need to number/codify these patterns so that we can talk about them more easily", then "Yes to JP-n". A flat series parallel to JB-n, allocated once and never reused, keeps the conversation handle apart from the machine label the code, the admissible box and the records ledger key on (JB-33 settled that identifiers stay). The GLOSSARY Pattern entry does not yet point at the page; that edit goes through the grilling and domain-modeling skills.

## Verification

`OPENBLAS_NUM_THREADS=1 python -m pytest -q tests/sim/test_docs_links.py tests/sim/test_mapkit_docs.py tests/sim/test_map_items.py tests/sim/test_glossary_avoid_words.py tests/sim/test_logbook_search.py`, run 2026-10-30 in a scratchpad worktree at origin/main ec6bdbdc: **52 passed in 2.54 s**. These are the tests that read `docs/` (absolute-link rule and path existence), the map item and its generated page, the avoid-word scope and the logbook directory; `mkdocs build --strict` is not installed on this box and is left to the docs CI.
