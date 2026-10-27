---
title: "JB-76: the CATCH solve tail tracks box load, not a bigger problem; starvation is the leading hypothesis; per-solve CPU time and context switches now logged"
type: investigation
date: 2026-10-28
status: in-progress
phase: "JB-76 - SPLICE_TOO_LATE solve tail"
related_plan:
  - two-ball-oval.md
related_entries:
  - 2026-10-27-leg-lead-clamp-age-aware-p3-robot.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/solver_worker.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - tools/probes/profile_session.py
  - tools/probes/README.md
  - tests/motion/test_solver_worker.py
  - tests/ros/test_install_segment.py
subsystem:
  - motion
tags:
  - solver-worker
  - splice-too-late
  - jetson-load
---

# JB-76: the solve tail tracks box load

## What

JB-76: on 2026-10-27, SPLICE_TOO_LATE ended 5 of 10 goals. This entry is the offline deep dive, plus the instrumentation the next Sitting needs to name the cause.

## Findings

- **The problem is not bigger.** Sim probes on this Jetson (scratchpad, not committed):
  - The oval `level` CATCH solves in 64 ms at dwell 0.50 and 70 ms at dwell 0.65, max under 85 ms. Hop CATCH 54 ms, REST 21 ms, THROW 35 ms.
  - Busy loops on 3 or 5 cores moved these by less than 4%.
- **The live problem is the same size.** The first CATCH of each live goal (no ball in flight, more than 1.5 s idle) solves in 57–81 ms. Mid-juggle CATCHes run at a 150–210 ms median on good launches and 200–350 ms on bad ones.
- **The worker path is not the cause.** A worker replay (scratchpad, not committed) put 97 captured CATCH requests through the production `WorkerSegmentSolver`:

  | Condition | CATCH median |
  |---|---|
  | In-process | 66.7 ms |
  | Worker | 68.4 ms |
  | Worker, 0.6 s idle gaps | 69.3 ms |
  | Busy loops on 3 cores | 69.8 ms |
  | Memory-bandwidth hog on 3 cores | 78.9 ms |

- **The slowdown varies with box load.** Within the 2026-10-27 launch, in 20 s bins:
  - Bins where the `mocap_node` queue sat near 6 of 150 had CATCH solves of 81–152 ms.
  - Bins where the queue was full had 270–346 ms.
  - The REST solve, a near-constant problem, moved from 40 to 149 ms with it.
  - Across launches, the share of seconds with the queue at or above 100 of 150 tracks the CATCH median:

  | Launches | Seconds with queue >= 100 | CATCH median |
  |---|---|---|
  | Most of 2026-10-19 to 2026-10-24 10:44 | about 0% | 90–150 ms |
  | 2026-10-24 11:34 | 6% | 205 ms |
  | 2026-10-27 | 16% | 194 ms |

  No motion code changed between 10-24 10:44 and 11:34: `git log --since='2026-10-24 10:40' --until='2026-10-24 11:35' -- ros_ws/src controller` is empty.
- **Two budgets are exposed.**
  - The unpinned CATCH (re-aim RESEND, hop feed catch) has about 150 ms. About 15 re-aims were refused on 2026-10-27, and two hop goals ended on it.
  - The pinned handoff CATCH has about 350–385 ms. It failed only in starved windows: 350 ms and 380 ms solves, and two "no reply" cases.

## Discussion

Ruled out for the 2026-10-27 window:
- uptime: all Sittings since mid-September ran on one 46-day boot;
- CPU frequency scaling (min = max = 1510400 kHz) and thermals (53 °C);
- Claude sessions, test gates and sims running during the window;
- extra rosbridge clients (one, as on fast days);
- the worker transport and idle-gap cache coldness.

Two hypotheses were tested and weakened:
- **OpenBLAS busy-spin from un-capped nodes.** Only `trajectory_node` and `skill_node` are capped to 1 thread. But small `lstsq` calls at 300 Hz under the default 6-thread pool cost 0.02–0.03 cores, so there was no spin at the tracker's call size.
- **Memory bandwidth.** About 10 ms.

Plain CPU oversubscription is weak too. Nine busy loops on 6 cores moved the worker's median from 68 to 74 ms, because the scheduler shares time fairly. A 3–5× slowdown would need far more runnable work than one launch should have.

The leading hypothesis is preemption and cache churn from the many processes that wake per mocap frame. The mocap queue overflow is read as a fellow victim of that starvation, not as its cause: it overflowed before any goal ran, and fixing its pipeline alone would not free the solver. The data cannot yet separate the two readings, which is what the instrumentation is for.

## Instrumentation (this commit)

- `SegmentSolveReply` gains:
  - `cpu_s`: the worker's `process_time` over the `solve_s` window;
  - `vcsw` and `ivcsw`: voluntary and involuntary context switches from `getrusage`.

  Refusals and the in-process double carry `nan` and `-1`.
- `trajectory_node` appends ` (cpu X ms, ivcsw N, vcsw M)` after `solve X ms`, on both the accept debug line and the refusal detail. Existing greps for `solve X ms` still match.
- The reading: if CPU time is close to the wall time, the solve itself is slow and live requests need capturing. If CPU time is well under the wall time and `ivcsw` is high, the box preempted the worker.
- Probe check, 2026-10-28: the captured replay was run through the worker, idle and then with nine busy loops (`worker_replay/cpuprobe.py`, scratchpad).

  | Condition | Wall median | CPU median | `ivcsw` median |
  |---|---|---|---|
  | Idle | 68.2 ms | 67.9 ms | 1 |
  | Nine busy loops | 74.0 ms (p90 112 ms) | 68.6 ms | 9 (max 21) |

  The instrument separates compute from preemption.
- `tools/probes/profile_session.py` gains a 1 s `vmstat` recorder: run queue, context switches and swap in/out. It is recorded raw; `report.md` does not analyse it yet. The existing `pidstat` (with `-w` for each process's context switches) and `mpstat` recorders cover the rest. The whole recorder set (`pidstat`, `mpstat`, `tegrastats`, `vmstat`) costs about 2–3% of one core, almost all of it `pidstat`, in an 8 s smoke run.
- For the next Sitting: `python tools/probes/profile_session.py start` before the launch, and `... stop` after it.

## Next

1. Fly one Sitting with the recorder running.
2. Read `cpu` and `ivcsw` for the slow solves, and `pidstat` for the processes that wake most during those seconds.
3. If the environment is confirmed, give the worker its own core: `sched_setaffinity` on the worker, and keep the other launch processes off that core (no root needed).
4. Restart the Jetson only after that Sitting's data is in.

## Verification

- Scoped, 2026-10-28: `pytest -q tests/motion/test_solver_worker.py tests/ros/test_install_segment.py tests/motion/test_solver_worker_lateness.py` gave 93 passed before the test was narrowed to a BLAS-capped worker; afterwards `tests/motion/test_solver_worker.py` gave 16 passed.
- The gate's triple is in the commit message.
