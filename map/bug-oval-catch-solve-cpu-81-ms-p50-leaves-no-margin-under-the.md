---
id: JB-96
kind: bug
title: Oval CATCH solve CPU (81 ms p50) leaves no margin under the 150 ms unpinned re-aim cliff
status: inbox
created: '2026-10-28'
updated: '2026-10-28T10:51:28Z'
owners: []
priority: medium
effort: l
area: skill-stack
blocked_by: []
links:
- 2026-10-28-jb76-solve-tail-is-environment-starvation-instrumented
parent: two-ball-oval
filed_by: claude (JB-76 21:02 Sitting read-out)
---

21:02 Sitting 2026-10-28, starvation fixed (CATCH wall/CPU 1.27): 3 of 109 oval throws still had a re-aim refused SPLICE_TOO_LATE.

## Finding

- All CATCH solves at 136-148 ms wall on 83-88 ms CPU (oval CATCH CPU p50 81 ms vs columns 65 ms).
- The SOLVE_BUDGET_KNOTS=6 (~150 ms) unpinned cliff was sized 2026-09-18 on CATCH p50 76 / p95 113 ms.
- Residue of JB-76, not starvation.

## Levers

- Worker SCHED_FIFO (closes the remaining wall-CPU gap).
- Profile the oval CATCH solve itself (why 81 ms).
- Revisit the budget.

## Scan

temp/probes/solve-starvation-2026-10-28-2102.txt
