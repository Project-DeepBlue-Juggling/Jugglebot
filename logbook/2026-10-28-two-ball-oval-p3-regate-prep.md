---
title: "Two-ball oval P3 re-gate prep: the re-gate flies the default (CFS solver worker) launch, `level` then `pose`; JB-95 FIFO stalled the setpoint stream and is off by default"
type: process
date: 2026-10-28
status: in-progress
phase: "two-ball-oval P3 - re-gate Sitting"
related_plan:
  - two-ball-oval.md
related_entries:
  - 2026-10-28-jb95-solver-worker-sched-fifo.md
  - 2026-10-27-leg-lead-clamp-age-aware-p3-robot.md
  - 2026-10-24-two-ball-oval-p3-ab-sittings-skid-bound.md
files_changed:
  - tests/hardware/session_oval_rung1.md
  - plans/active/two-ball-oval.md
subsystem:
  - motion
tags:
  - runbook
  - oval
  - p3
---

# Two-ball oval P3 re-gate prep (docs only)

## What

Owner decisions of 2026-10-28 recorded in `tests/hardware/session_oval_rung1.md` § 9 and `plans/active/two-ball-oval.md` § 4 P3 (nothing flown, no code):

- **Re-gate shape:** the `level` column (P3R-2) then the `pose` column (P3R-3/4) in ONE launch of the DEFAULT configuration (CFS solver worker). P3R-1 checks `ros2 param get /trajectory_node solver_rt_priority` reads 0. History: the JB-95 SCHED_FIFO worker was flown once (22:26): wall/CPU 1.01 as predicted, but it stalled the setpoint stream 60-96 ms per CATCH solve (GIL-coupled preemption, inferred), past the firmware 70 ms STOP cover, so the lane was refused and the guard E-stopped on 3 of 5 goals; off by default, JB-95 back to backlog (peer session).
- `num_cycles` 30 -> 50 in P3R-2/P3R-4 (matches the 21:09 oval 50/50); the gate stays at least 30 attempts in one launch.
- P3R-4: a `ball_held` seat-bit flicker with the ball still thrown is logged, not a fail. P3R-6: a bridge `CLAMP_DUTY` warning (0.16-0.27 seen at 21:02 and 22:26) is recorded, not a stop.
- Prerequisites updated: lead-clamp plan MET 2026-10-27, JB-76 closed 2026-10-28. Leg-current references (7.49 / 10.05 A) unchanged; newer `level` 0.50 references noted (35/35, 35/35, 50/50).
- **JB-98** is a WATCH item, not a blocker: the 22:26 trips are explained by the FIFO worker; the 21:02 trip stays unexplained and a recurrence on the default launch ends the attempt (P3R-6). Power-cycle the bridge before the launch.

Why: the 10-24..10-28 Sittings all ran `level`, so the re-gate flies the configuration that has actually flown.

## Verification

- **Verification:** `./run_tests.sh --full`, run 2026-10-28 ~22:50 on the final tree (`temp/logs/p3-regate-prep-full.log`): **parallel 7549 passed / 41 skipped in 336.62 s + serial 8 passed in 30.21 s, PASS** (exit 0). Skip count up from 4 on 2026-10-27; reasons recorded below once read.
