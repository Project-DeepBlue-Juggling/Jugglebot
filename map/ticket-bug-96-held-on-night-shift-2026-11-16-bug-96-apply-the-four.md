---
id: TKT-80
kind: ticket
title: 'BUG-96 held on night-shift/2026-11-16-BUG-96: apply the four findings (a race-free bind check) and merge, merge as is, or drop?'
status: complete
created: '2026-11-17'
updated: '2026-11-17T22:50:53Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-trajectory-node-stays-up-and-reports-healthy-when-its
type: question
---

BUG-96 is rebuilt on night-shift/2026-11-16-BUG-96 as you chose on TKT-77: `main()` checks the bind-failure flag right after building the node and, if set, runs `on_shutdown()` and `destroy_node()` and exits 1 without building the executor. The audit found the check is not race-free: the bind runs on the emitter thread, so a failure that lands after the check still reaches the executor constructor and exits by traceback.

## Options

1. **Apply the four open findings and merge:** wait on a bind-done event before the check (or build the executor inside the `try`, so the cleanup always runs); wrap the early exit in `try/finally`; add a test for a failure during `spin()`; and make the new test assert that the segment-solver worker's `close()` ran rather than stubbing `on_shutdown`. The next night builds them, re-gates and merges on MERGE.
2. **Merge as is:** the common ordering exits 1 cleanly; the rare late failure still exits non-zero, by traceback.
3. **Drop the branch.**

## What each answer unblocks

- 1 or 2: the next night re-gates and merges once BUG-109 (the legacy ID pin, TKT-79) is green.
- 3: BUG-96 goes back to open.

Recommendation: 1. The bind-done event is a few lines, and it is what makes the exit deterministic, which is what you asked for.

## Answer

I agree with your recommendation
