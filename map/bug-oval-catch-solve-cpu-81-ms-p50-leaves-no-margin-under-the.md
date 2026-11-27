---
id: BUG-32
kind: bug
title: Oval CATCH solve CPU (81 ms p50) leaves no margin under the 150 ms unpinned re-aim cliff
status: in_progress
created: '2026-10-28'
updated: '2026-11-27T04:11:45Z'
owners: []
priority: medium
effort: l
hardware: robot
owner: hands
radius: subsystem
diagnosis: confirmed
area: skill-stack
blocked_by: []
links:
- 2026-10-28-jb76-solve-tail-is-environment-starvation-instrumented
parent: two-ball-oval
filed_by: claude (BUG-21 21:02 Sitting read-out)
night_skip: sitting
night_skip_on: '2026-11-27'
night_skip_note: Warm start held on offline/bug32-warm; its Sitting section flies it and A/Bs cpu_partition on the Catch solve wall time.
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

## Sitting result (2026-11-27 15:08)

- verdict: fail
- notes: No issues. Not really visibly distinguishable from the Launch 3 attempt (sheet 2026-11-27-robot, step 30) [derived: CATCH solve CPU p50 78.2 ms (launch 3, cpu_partition off) and 77.8 ms (launch 4, on), n 30 each, against the 72 ms bar; the same day without the warm start read 77.3 and 76.7 ms. No new refusal code. Wall p95 143.9 ms off against 202.9 ms on; solves at or past 150 ms: 1 against 4. All 64 Throws caught.]

## Progress

- 2026-11-26: Profile done offline (scripts in temp/probes/bug32-profile/): oval_1ball_fed CATCH CPU p50 is still 78 to 81 ms (launch logs 11-20 to 11-23); Columns CATCH is now 45 ms, not 65. Oval wall p95 is 160 to 168 ms, past the 6-knot cliff. No two-ball oval log exists.
- 2026-11-26: Cause of the gap: the steady-window QP (168 variables, 1034 inequalities, 46 active-set iterations, against 69, 449 and 25 for Columns). It always runs cold: the Segment carries the settle QP's warm start, which plan_window drops on a key mismatch. Feeding the previous steady active set offline: 46 to 12 iterations, CPU 60.5 to 51.4 ms, solution differs by 4e-9.
- 2026-11-26: Candidate, not built: carry a steady-window solver state per window kind so the steady QP warm-starts (plan unchanged to solver tolerance; needs its own design and gate). The wall tail is scheduling: wall/CPU is about 1.4 and launches since 11-20 log the worker as CFS with cpu_partition off. Do not raise SOLVE_BUDGET_KNOTS.
- 2026-11-26: Built on held branch offline/bug32-warm: a Segment carries one QP warm start per window kind, so the oval steady QP starts from the previous beat's (46 to 12 iterations); a warm solve that refuses is re-solved cold. Plans agree with cold to 2e-10. Offline CPU p50: oval 61.9 to 52.0 ms, Columns 36.5 to 34.3 ms. Not flown.
- 2026-11-26: Scheduling: nothing is partitioned at runtime. cpu_partition defaults off; its one A/B (2026-11-18) measured bridge drops, not solve wall time, and showed no effect. Offline, pinning the worker cut loaded wall p50 100.6 to 81.0 ms. The Sitting section adds a partition arm. Worker FIFO stays off (owner, 2026-10-28).
- 2026-11-26: Owner decisions: the warm start covers every window kind, and the Sitting keeps its cpu_partition:=true arm as an A/B on the Catch solve wall time.
- 2026-11-27: Failed at the Sitting of 2026-11-27: CATCH solve CPU p50 78.2 and 77.8 ms against 72; main the same day 77.3 and 76.7. The warm start ran but its key (the STEADY window's knot count) matched only 9 of about 240 solves, because the window starts at the moving splice point; where it matched it saved 5 to 10 ms. The offline bench used one fixed seed. Do not merge as is.
- 2026-11-27: New lead from 2026-11-27: install CPU grows about 1 ms per 100 record Knots (about +15 ms by Throw 32) on main and branch alike while check CPU stays flat; it tracks SPLICE_TOO_LATE rising with Throw index (0, 4, 6, 5 refusals per 16 Throws). Filed as its own Item. Owner decision on offline/bug32-warm: drop it, or rebuild as a shifted warm start.
