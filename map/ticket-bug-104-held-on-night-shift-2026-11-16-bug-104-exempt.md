---
id: TKT-78
kind: ticket
title: 'BUG-104 held on night-shift/2026-11-16-BUG-104: exempt Deactivate and Clear Errors from the latch and merge, merge as is, or drop?'
status: open
created: '2026-11-16'
updated: '2026-11-15T19:02:09Z'
owners: []
priority: low
effort: xs
hardware: none
owner: decision
radius: local
area: gui
blocked_by: []
links: []
parent: bug-gui-command-buttons-have-no-in-flight-guard
type: question
---

BUG-104 is built on night-shift/2026-11-16-BUG-104: one click on an orchestrator command button latches the buttons until the orchestrator state changes or 1.5 s passes, so a double click publishes once (the orchestrator enqueues every command, so a duplicate is not guaranteed harmless). The audit found that the latch also covers Deactivate and Clear Errors: after an Activate that is refused or slow, those two stay disabled for up to 1.5 s.

## Options

1. **Exempt Deactivate and Clear Errors:** they are never latched by another button; each button still guards against its own double click. The next night builds it, re-gates and merges on MERGE.
2. **Merge as is:** all five buttons share the latch; the worst case is Deactivate dark for 1.5 s after a refused Activate.
3. **Drop the branch.**

Separately, the Item asked whether Activate deserves a hold-to-confirm like Clear Errors. Nothing was added; say yes here if you want it built too.

## What each answer unblocks

- 1 or 2: the next night re-gates and merges.
- 3: BUG-104 goes back to open.

Recommendation: 1. Deactivate is the operator's way out of Active, and it should never wait on another button's latch.
