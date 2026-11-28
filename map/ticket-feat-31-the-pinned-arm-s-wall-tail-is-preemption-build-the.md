---
id: TKT-104
kind: ticket
title: 'FEAT-31: the pinned arm''s wall tail is preemption; build the SCHED_FIFO worker now, wait for BUG-149, or drop it?'
status: complete
created: '2026-11-28'
updated: '2026-11-28T00:37:52Z'
owners: []
priority: high
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: feature-solver-worker-on-sched-fifo-take-the-catch-solve-out-of-the
type: question
---

FEAT-31 was off by your decision until BUG-32's cpu_partition arm showed whether the solve's wall tail still bites. The 2026-11-27 Sitting answered: it does, and pinned was worse (wall p95 202.9 ms pinned against 143.9 unpinned; 14 % of pinned solves waited 80 ms or more on only 73 to 89 ms of CPU, not queued, with many involuntary switches). That is preemption, which SCHED_FIFO addresses and CPU pinning does not. One 30-solve sample per arm.

## Options

1. Build it now: the solver worker on SCHED_FIFO (with a fallback and a log line when the priority is refused), on a branch with an offline lateness test, held for a Sitting A/B against today's worker.
2. Wait for BUG-149 (install CPU growing through an Attempt, being profiled tonight) and re-measure first, so the Sitting can attribute one change at a time.
3. Drop it.

## What each answer unblocks

1: the next night builds and holds it for a Sitting; whether the box grants real-time priority to the launch user is checked offline first, and if it needs a system change that is yours. 2: the Item waits on BUG-149's Sitting. 3: the Item closes.

Recommendation: 1, because the measured tail is involuntary preemption, a different lever from BUG-149's install cost, and a held branch costs nothing until you fly it.

## Answer

Agreed. Go with Option 1
