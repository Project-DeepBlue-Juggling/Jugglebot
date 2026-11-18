---
id: TKT-86
kind: ticket
title: 'BUG-96 held on night-shift/2026-11-16-BUG-96: apply the two new findings (log the swallowed exception, test the bind wait) and merge, merge as is, or drop?'
status: complete
created: '2026-11-19'
updated: '2026-11-18T21:24:25Z'
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

BUG-96 is rebuilt on night-shift/2026-11-16-BUG-96 with your TKT-80 choice, all four findings applied. The emitter thread sets a bind-done Event either way, and `main()` waits up to 3 s for it, then proceeds. The executor is built inside the `try/finally`. The early exit's cleanup is in a `try/finally`. The tests assert that the segment-solver worker's `close()` ran. Tonight's audit verified the exit is deterministic and no path exits 0 after a bind failure. It left two findings open.

## Options

1. **Apply the two findings and merge.** First, log the exception that `main()` now swallows when the bind has failed; today the exit code is still 1 but the cause is hidden. Second, add two tests for the Event wait itself: one with a real emitter thread and a failing bind, one with a 0.01 s wait and a bind that hangs. Every current test runs without the emitter thread, so the wait is never executed. The next night builds them, re-gates and merges on MERGE.
2. **Merge as is:** the behaviour is right and verified by reading. Only the log line and the tests of the wait are missing.
3. **Drop the branch.**

## What each answer unblocks

- 1 or 2: the next night re-gates and merges on MERGE.
- 3: BUG-96 goes back to open.

Recommendation: 1. Both are a few lines. Without the log line, a late failure would show only exit code 1.

## Answer

Agreed. GO with Option 1
