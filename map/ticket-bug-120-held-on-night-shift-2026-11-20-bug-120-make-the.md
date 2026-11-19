---
id: TKT-87
kind: ticket
title: 'BUG-120 held on night-shift/2026-11-20-BUG-120: make the starvation skip load-conditional and cap restarts at one, then merge; merge as is; or drop?'
status: open
created: '2026-11-20'
updated: '2026-11-19T17:40:09Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: tests-tooling
blocked_by: []
links: []
parent: bug-test-a-fresh-throw-installs-through-the-real-worker-process
type: question
---

BUG-120 is built on night-shift/2026-11-20-BUG-120, test only. The real-worker Throw install test reserved the fresh Throw's 250 ms solve budget. A loaded box exceeds it: under 8 busy loops the old test passed 0 of 10 runs and the fixed test passed 7. The fix widens the lead to 1.5 s for this one test and retries a worker restart. If six solves are all too late, the test skips.

The audit left two findings open:

1. **The skip can hide a regression.** A planner slowdown past about 1.4 s per solve, or a deadline or clock bug that makes every reply read late, would skip instead of failing. No test now measures a real worker against the 250 ms budget. The fix is to skip only when the box is oversubscribed (load average above the core count) and fail otherwise.
2. **One worker death is retried away.** A single `SOLVER_UNAVAILABLE` followed by a good solve passes. The fix is to allow at most one restart, or report that one happened.

## Options

1. **Apply both findings and merge.** The next night makes the skip load-conditional, caps restarts at one, re-gates, and merges on MERGE.
2. **Merge as is.** The test is far less flaky than today, and a worker that crashes on every attempt still fails.
3. **Drop the branch.** BUG-120 goes back to open, and the test stays flaky under load.

## What each answer unblocks

- 1 or 2: the next night re-gates and merges on MERGE.
- 3: BUG-120 reopens.

Recommendation: 1. Both changes are a few lines. Without the first, the gate can turn a real slowdown into a skip without anyone noticing.
