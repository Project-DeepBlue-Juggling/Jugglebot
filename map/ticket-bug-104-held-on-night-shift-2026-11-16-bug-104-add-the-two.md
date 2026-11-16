---
id: TKT-81
kind: ticket
title: 'BUG-104 held on night-shift/2026-11-16-BUG-104: add the two missing tests and merge, merge as is, or drop?'
status: open
created: '2026-11-17'
updated: '2026-11-16T19:00:10Z'
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

BUG-104 is rebuilt on night-shift/2026-11-16-BUG-104 as you chose on TKT-78: Deactivate and Clear Errors are exempt from the shared latch, and each keeps a private latch against its own double click. Clear Errors is the only hold-to-confirm button, and a completed hold still publishes once. The audit passed the code: no latch can stay set, and no other button can disable Deactivate. Two test gaps are open.

## Options

1. **Add the two tests and merge:** a case where Activate is clicked in IDLE and the state then moves to ACTIVE, with Deactivate enabled and publishing once (the current case clicks Level in ACTIVE, which the real GUI cannot do); and a second Clear Errors hold that reaches the private latch rather than being refused by the disabled button. The next night builds them, re-gates and merges on MERGE.
2. **Merge as is:** the behaviour is right; only the tests are weaker than they look.
3. **Drop the branch.**

## What each answer unblocks

- 1 or 2: the next night re-gates and merges once BUG-109 (the legacy ID pin, TKT-79) is green.
- 3: BUG-104 goes back to open.

Recommendation: 2. You noted the buttons are mostly hold-to-confirm, so the risk is small; the code is right and both gaps are test-only.
