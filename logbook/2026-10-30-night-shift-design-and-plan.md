---
title: "Night Shift designed and planned: an unattended Claude Code run after the nightly works the board through a deterministic merge gate, holds the rest on branches, asks the owner through question tickets; the Next Sitting Runbook is generated from the Items (Pathway JB-119, plans/active/night-shift.md)"
type: process
date: 2026-10-30
status: complete
phase: "night-shift design"
related_plan:
  - night-shift.md
files_changed:
  - map/pathway-night-shift.md
  - plans/active/night-shift.md
  - plans/active/INDEX.md
  - docs/project-map.md
  - logbook/INDEX.md
subsystem:
  - docs
  - tests-tooling
tags:
  - night-shift
  - map
  - plan
  - automation
---

# Night Shift: design and plan

## What

Pathway JB-119 and `plans/active/night-shift.md` were written from a two-round design discussion with the owner on 2026-10-30. Night Shift is an unattended Claude Code run on this Jetson after the 04:00 nightly suite, in its own clone and billed to the owner's otherwise idle API credits, that walks every open Item, works what Claude can finish alone, merges only what passes a deterministic gate, holds everything else on `night-shift/<date>-JB-n` branches and writes one committed report whose operator questions are answerable `ticket` Items. The second half is the Next Sitting Runbook: one ordered page of every hardware check waiting across all Pathways, generated from `## Sitting` sections on the Items, with results typed in the map app and a wrap-up Session doing the merging.

## Why

The board held 57 open Items on 2026-10-30, 10 of them already break-sized by the owner's own metrics, and Items are filed faster than interactive Sessions clear them. The owner's share of a Sitting today includes steps a Session can run (the suite, git), and hardware checks waiting across Pathways have no single page.

## Discussion

Three runner shapes were weighed. A cloud routine or a Managed Agents scheduled deployment would have no access to the gate that makes a change trustworthy here (the venv, MuJoCo, the serial timing tests, the teensy_link wire tests all live on this box), so it was ruled out as the primary runner. A hand-rolled orchestrator over the Messages API would reimplement the harness and every repo rule already encoded in CLAUDE.md, the skills and mapkit, which is exactly the tech debt the owner named as the failure condition. Headless Claude Code under a user timer reuses all of it; its new surface is one timer, a runner, a protocol file and a gate script.

The path gate first proposed (forbid control and firmware paths outright) was replaced by a merge gate after the owner asked for a working `main` every morning with doubtful work held aside, and for hardware-touching Items to be worked and parked for a Sitting rather than skipped. The gate is code with a test, not judgment: suite green, audit clean, re-rated hardware none, radius not normative, no open question, no normative or firmware path.

Owner decisions recorded in the Item body: no cap on items or spend (a budget fuse only); Fable orchestrates and picks Sonnet or Opus per unit, never a Fable subagent; one audit per Item; unrated Items are rated and considered; decision-gated Items are researched then asked; ideas get a feasibility note and easy ones may be built; Night Shift claims the nightly ticker; reports are committed; questions are tickets answered in the app; the Runbook is one ordered list grouped by setup state with Pathway filters, not separate panels; a Sitting ends with a wrap-up Session and the owner's steps are the hardware ones only. "Runsheet" is a glossary Avoid word, so the page is the Next Sitting Runbook; the name Night Shift was adopted without a grilling round by owner decision.

## Verification

- 2026-10-30, `pytest -q tests/sim/test_map_items.py tests/sim/test_glossary_avoid_words.py tests/sim/test_mapkit_docs.py tests/sim/test_plans_index.py tests/sim/test_logbook*.py` in the scratchpad worktree: **70 passed in 3.60s** (docs-only change; these are the tests that read map/, plans/active/, logbook/ and the generated pages).
- Design only; no code behaviour changed. The plan's P1 verifies API-key billing and the headless permission model before anything runs unattended.
