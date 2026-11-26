---
id: BUG-32
kind: bug
title: Oval CATCH solve CPU (81 ms p50) leaves no margin under the 150 ms unpinned re-aim cliff
status: inbox
created: '2026-10-28'
updated: '2026-11-26T03:18:28Z'
owners: []
priority: medium
effort: l
hardware: robot
owner: hands
radius: subsystem
area: skill-stack
blocked_by: []
links:
- 2026-10-28-jb76-solve-tail-is-environment-starvation-instrumented
parent: two-ball-oval
filed_by: claude (BUG-21 21:02 Sitting read-out)
night_skip: sitting
night_skip_on: '2026-11-22'
night_skip_note: A flown CPU measurement on the robot, owner hands.
---

21:02 Sitting 2026-10-28, starvation fixed (CATCH wall/CPU 1.27): 3 of 109 oval throws still had a re-aim refused SPLICE_TOO_LATE.

## Finding

- All CATCH solves at 136-148 ms wall on 83-88 ms CPU (oval CATCH CPU p50 81 ms vs columns 65 ms).
- The SOLVE_BUDGET_KNOTS=6 (~150 ms) unpinned cliff was sized 2026-09-18 on CATCH p50 76 / p95 113 ms.
- Residue of BUG-21, not starvation.

## Levers

- Worker SCHED_FIFO (closes the remaining wall-CPU gap).
- Profile the oval CATCH solve itself (why 81 ms).
- Revisit the budget.

## Scan

temp/probes/solve-starvation-2026-10-28-2102.txt

## Sitting

- setup: level, armed, bb-fed
- why: offline, the warm start cut the oval CATCH solve CPU by about 10 ms and pinning the worker cut loaded wall time; only a flown oval Attempt shows either on the robot.
- do:
  - Launch with `cpu_partition:=false` and fly one `oval_1ball_fed` Attempt at the usual working point for at least 20 Catches.
  - Relaunch with `cpu_partition:=true` and fly the same Attempt again.
- pass: the launch log's CATCH solve CPU p50 (CATCH solves after the first) is at most 72 ms in both arms, against 78 to 81 ms on the 11-20 to 11-23 launches, with no new refusal codes; the partition arm's wall p95 is reported against the other arm's.
- record: per arm the CATCH solve CPU p50 and p95, wall p50 and p95, the count of SPLICE_TOO_LATE and refused re-aims, and the launch log path.
- minutes: 20
- branch: offline/bug32-warm

## Progress

- 2026-11-26: Profile done offline (scripts in temp/probes/bug32-profile/): oval_1ball_fed CATCH CPU p50 is still 78 to 81 ms (launch logs 11-20 to 11-23); Columns CATCH is now 45 ms, not 65. Oval wall p95 is 160 to 168 ms, past the 6-knot cliff. No two-ball oval log exists.
- 2026-11-26: Cause of the gap: the steady-window QP (168 variables, 1034 inequalities, 46 active-set iterations, against 69, 449 and 25 for Columns). It always runs cold: the Segment carries the settle QP's warm start, which plan_window drops on a key mismatch. Feeding the previous steady active set offline: 46 to 12 iterations, CPU 60.5 to 51.4 ms, solution differs by 4e-9.
- 2026-11-26: Candidate, not built: carry a steady-window solver state per window kind so the steady QP warm-starts (plan unchanged to solver tolerance; needs its own design and gate). The wall tail is scheduling: wall/CPU is about 1.4 and launches since 11-20 log the worker as CFS with cpu_partition off. Do not raise SOLVE_BUDGET_KNOTS.
- 2026-11-26: Built on held branch offline/bug32-warm: a Segment carries one QP warm start per window kind, so the oval steady QP starts from the previous beat's (46 to 12 iterations); a warm solve that refuses is re-solved cold. Plans agree with cold to 2e-10. Offline CPU p50: oval 61.9 to 52.0 ms, Columns 36.5 to 34.3 ms. Not flown.
- 2026-11-26: Scheduling: nothing is partitioned at runtime. cpu_partition defaults off; its one A/B (2026-11-18) measured bridge drops, not solve wall time, and showed no effect. Offline, pinning the worker cut loaded wall p50 100.6 to 81.0 ms. The Sitting section adds a partition arm. Worker FIFO stays off (owner, 2026-10-28).
