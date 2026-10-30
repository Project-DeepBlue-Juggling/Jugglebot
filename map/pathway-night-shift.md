---
id: JB-119
kind: pathway
title: 'Night Shift: unattended overnight work on the board, and the Next Sitting Runbook'
status: active
created: '2026-10-30'
updated: '2026-10-30T12:36:21Z'
owners: []
priority: high
effort: xl
hardware: robot
owner: hands
radius: cross-cutting
area: tests-tooling
blocked_by: []
links: []
destination: Every morning main is green and carries the overnight work that needed no hands; everything else waits on a held branch, an answerable question in the app or a row of the Next Sitting Runbook, and the owner's share of a Sitting is the hardware steps only.
---

The board gains small Items faster than Sessions clear them. Night Shift is an unattended Claude Code run on this Jetson after the 04:00 nightly suite, in its own clone, billed to the owner's API credits. It walks every open Item, works the ones Claude can finish alone, holds the rest on branches, and writes one report.

The Next Sitting Runbook is the second half: one page listing every hardware check waiting across all Pathways, Items and plans, with the owner's steps cut to the hardware-relevant ones.

## Decisions (2026-10-30 design discussion)

- **Runner**: headless Claude Code (`claude -p`) under a user systemd timer, in its own clone under `~/map-live/` style isolation, never in `~/Desktop/Jugglebot`. Full auto-approval inside that clone. API key in a mode-600 environment file. No per-night cap on items or spend.
- **Orchestrator**: Fable, context kept small, deciding per unit whether Sonnet or Opus does the work. No Fable subagents. One audit per Item, led by a model the orchestrator picks.
- **Merge gate**: a branch merges to `main` only when the full suite gate is green, its audit found nothing behaviour-affecting, the walker's own re-rating says hardware none, radius is not normative, and no operator question is open. Everything else is held on `night-shift/<date>-JB-n` and listed in the report.
- **Every open Item is considered**, unrated ones included; the walker re-rates before acting. Decision-gated Items get the research done and options drafted into the body, then a question. Ideas get a feasibility note; easy ones may be built.
- **Operator questions** are `ticket` children of the Item they block, answered under `## Answer`; the report tab in the map app offers an inline answer box.
- **Night Shift claims the nightly ticker** and folds the verdict into its report. Reports are committed.
- Robot is never powered overnight; the bridge is. The clone must not reach the UDP link.

## Next Sitting Runbook

- One ordered list, grouped by setup state (Level, Armed, Ball Butler fed, bridge power-cycled), highest value per minute of robot time first; Pathway chips filter the same list rather than separate lists.
- Rows come from a `## Sitting` section any Session writes on a hardware Item (why, do, pass criterion, record, time), plus held Night Shift branches stacked into one `night-shift/sitting-<date>` candidate branch.
- Results are typed per row in the app and saved into the Item. Each Sitting ends with a wrap-up Session that merges, commits records and files the follow-ups; the owner's steps are the hardware ones only.
- Existing `tests/hardware/` Runbooks stay as reference documents linked from rows.

## Plan

One plan under this Pathway, `plans/active/night-shift.md`, in six steps: P1 runner skeleton and dry run; P2 work loop, merge gate, report and ticker; P3 Night Shift page with answerable tickets; P4 `## Sitting` sections, Runbook generator and page; P5 Sitting wrap-up command; R6 the first Runbook-driven Sitting. The glossary gains Night Shift and Next Sitting Runbook entries in P1.

## Open questions

- Confirm in P1 that Claude Code with an API key bills the Console credits. The headless flags (`--max-budget-usd`, `--model`, `--effort`, `--output-format json`, `--dangerously-skip-permissions`) were checked on the installed 2.1.314.
