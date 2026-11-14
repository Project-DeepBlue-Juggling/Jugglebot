---
id: PWAY-9
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

The board gains small Items faster than Sessions clear them. Night Shift is an unattended Claude Code run on this Jetson after the 04:00 nightly suite, in its own clone, `~/night-shift/Jugglebot`. It examines what changed on the board, works the Items Claude can finish alone, holds the rest on branches, and writes one committed report. Its protocol is `.claude/commands/night-shift.md`; its runner is `tools/night_shift.sh`.

The Next Sitting Runbook is the second half: one page listing every hardware check waiting across all Pathways, Items and plans, with the owner's steps cut to the hardware-relevant ones.

## How the night runs

- **Runner**: headless `claude -p` under a user systemd timer (04:45), in the clone on `main`, never in `~/Desktop/Jugglebot`. A deny-list settings file is installed into the clone on every run; the runner refuses to start on a live robot session, an owner hold file or an unreviewed ungated landing.
- **Models and billing**: the orchestrator is Opus (`NIGHT_SHIFT_MODEL`); the runner refuses a Fable model (`FORBIDDEN_MODEL`) and subagents are Sonnet, or Opus with a stated reason. With an API key the night runs on it; if the credit balance runs out and a subscription token is set, the same session resumes on the subscription. With no key it runs on the subscription from the start.
- **The delta, not the board**: the preflight (`tools/night_shift_preflight.py`) lists the Items added, changed, carried over, unblocked or never classified since the last completed night. An empty delta ends the night `NOTHING_NEW` with no claude call and no ticker claim. Sundays, and a `full-walk.request` file, walk every open Item.
- **Skip reasons live on the Item**: an Item the night leaves alone carries `night_skip` (a fixed vocabulary, `mapkit/items.py`) so it is not re-examined until it changes; an `input-*` reason is listed in the report's section of Items the owner could unblock with a small input.
- **Offline only**: no hardware. Every test, probe and rosbag read runs under `tools/night_shift_nonet.sh`, which has no network and reads the bags and launch logs read-only. The robot is never powered overnight; the bridge is.
- **Operator questions** are `ticket` children of the Item they block, answered under `## Answer` in the app.
- **Night Shift claims the nightly ticker** and folds the verdict into its report; a failed night re-raises it and leaves a stub report where it can.

## The merge gate

A held branch merges to `main` only through `tools/night_shift_gate.py`, which fails closed and exits 0 only on MERGE. Eight conditions, all evaluated:

- (a) the full suite passed on the branch tip;
- (b) the audit names that tip and has no open behaviour-affecting finding;
- (c) the Item is rated `hardware: none`, the branch did not lower a rating the base had, and the Item has no `## Sitting` section;
- (d) the Item is rated and its radius is not normative;
- (e) no open question ticket names the Item;
- (f) the branch touches no frozen path;
- (g) the branch did not change an `owner` rating the base had;
- (h) a branch touching `mapkit/` also passed the map app's own suite.

Only (c) is waivable, by `/sitting-debrief` after a hardware pass. A merge with no gate record is `UNGATED_MERGE` and holds every later night until the owner clears it. A branch that edits the Night Shift's own machinery (the never-list in `tools/night_shift_landed.py`) is flagged `SELF_EDIT_BRANCH`.

## Next Sitting Runbook

- One ordered list, grouped by setup state (Level, Armed, Ball Butler fed, bridge power-cycled), highest value per minute of robot time first; Pathway chips filter the same list rather than separate lists.
- Rows come from a `## Sitting` section any Session writes on a hardware Item (why, do, pass criterion, record, time), plus held Night Shift branches stacked into one `night-shift/sitting-<date>` candidate branch.
- A Sitting is flown from a Sitting sheet (`/sitting-sheet`); results are typed in the app and saved into the Item.
- `/sitting-debrief` ends each Sitting: it reads the results and merges a passed Item's held branch through `tools/sitting_merge.sh prepare` and `land`, which run the gate with (c) waived. The owner's steps are the hardware ones only.
- Existing `tests/hardware/` Runbooks stay as reference documents linked from rows.

## Plan

One plan under this Pathway, `plans/active/night-shift.md`, in six steps: P1 runner skeleton and dry run; P2 work loop, merge gate, report and ticker; P3 Night Shift page with answerable tickets; P4 `## Sitting` sections, Runbook generator and page; P5 Sitting debrief command; R6 the first Runbook-driven Sitting. Its hardening is `plans/active/night-shift-closeout.md`.
