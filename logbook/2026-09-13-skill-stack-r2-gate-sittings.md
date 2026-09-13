---
title: "R2 hardware-gate sittings (x2) — a schedule that is only exact near t = 0, a driver that coupled its attempts, and where the Jetson's CPU actually goes"
type: investigation
date: 2026-09-13
status: in-progress
phase: "two-ball-skill-stack — R2 (hardware gate)"
related_plan: two-ball-skill-stack.md
---

# R2 hardware-gate sittings

## Outcome

`tests/hardware/session_skills_r2_plan_gate.md` was flown twice on 2026-09-13
(robot activated, wire disarmed, nothing moved; G5 passed every row). **The gate
is not met yet**, and both sittings failed for the same two defects, neither of
which is plan time:

- **A production bug in `motion/skills/schedule.py`, now fixed.** The schedule
  compared absolute instants to 1e-9 s. On the ROS wall clock (~1.79e9 s) a
  double resolves ~2.4e-7 s, so at the robot's start time a 20-throw schedule
  held 24 skills instead of 22 (two catch/throw pairs unfolded, each THROW then
  refused `LIMIT_JERK` one knot after a rest-terminal catch) and every other
  handoff got the 150 ms general lead instead of the 200 ms handoff lead (every
  `SPLICE_TOO_LATE` of sitting 1). `skill_node` schedules on the same clock, so
  R3's first powered session would have failed the same way.
- **A driver defect, now fixed.** After an attempt ended early the next
  attempt's REST pre-position spliced onto the still-moving plan and refused;
  in sitting 2 that ended attempts 1 and 2 of every row immediately.
- **The rehearsal was not the robot's solver, now fixed.** It ran without the
  planner nodes' one-thread BLAS cap and without the carried QP warm start, so
  offline it produced idle solves to 159.5 ms that the node cannot. Capped, the
  rehearsal runs three 20-throw attempts back to back with a worst solve of
  31.16 ms and every install accepted.

Plan time itself: at the re-run's load (load1 1.4–2.5) G1 passed on arms A and
B (worst 42.1 / 43.7 ms) and failed only on the stressed row C (54.4 ms), with
G2 margins of 67–79 ms. Sitting 1 ran at load1 7.4–8.8 and its solves were
1.6–2.4× slower; that extra load is unattributed. A re-fly after these fixes is
the gate.

## Discussion

### What the two sittings measured

| | Sitting 1 (11:35–11:38) | Sitting 2 (11:58–12:00) |
|---|---|---|
| load1 during the rows | 7.4–8.8 (row C 8.0–8.8) | 1.4–2.5 |
| CATCH+throw p50 / max, arm A (ms) | 76.4 / 119.1 | 40.2 / 42.1 |
| CATCH+throw p50 / max, arm B (ms) | 52.8 / 80.3 | 40.8 / 43.7 |
| CATCH+throw p50 / max, row C (ms) | 70.3 / 97.9 | 48.3 / 54.4 |
| `SPLICE_TOO_LATE` | 4 (all on the short lead) | 0 |
| Emitter gap max (G3) | 29–34 ms | 26–28 ms |
| G4 | FAIL (bug + coupling) | FAIL (bug + coupling) |

Same code, same launch file, same limits. The idle rehearsal the same morning
(load1 0.9–1.2) read CATCH+throw p50 28.7 / max 30.0 ms.

### The schedule bug — why the rehearsal, the sim gate and the tests all missed it

Diagnosed from the console alone and confirmed offline: compiling the same
20-throw pattern at `t0 = 10.0` and `t0 = 12345.678` gave 22 skills with every
catch on `HANDOFF_LEAD_S`; at `t0 = 1789263419.5` and at sitting 1's own start
it gave 24 skills (3 THROW, 17 CATCH+throw, 3 CATCH) and leads alternating
0.200 / 0.150 s — the live sequence skill for skill, including the THROW after
the ninth catch. The dispatch instants in sitting 1's CSV were 0.628 s and
0.528 s apart, alternately (β ± 50 ms). The four `SPLICE_TOO_LATE` refusals all
reported budgets of 42–65 ms from dispatch: 125 ms less the 50 ms lead that was
missing, less knot rounding and dispatch lateness.

The class is every comparison of absolute instants at a tolerance below the
clock's resolution; there were three, all in `schedule.py` (the pair fold, the
lead assignment, the dispatch-order check). The executor's comparisons are on
windows, i.e. differences of instants, and are sound. Every schedule test and
the sim gate (plant time) ran within seconds of t = 0, and the rehearsal on
`perf_counter` (~1.8e5 s after two days of uptime), where the same arithmetic is
exact to ~1e-15 s and ~3e-11 s respectively. The fix removes the class
rather than widening a tolerance: `compile_columns` builds the whole schedule on
its own clock (t = 0 at the first throw), makes every comparison there, and adds
`t0_abs_s` once, at the end. The tests now compile at five start times up to the
sitting's own and require the identical schedule; the driver's attempt-loop
smoke runs at t = 0 and at a ROS-epoch start; and the rehearsal runs on a
monotonic clock shifted to the epoch, so a rehearsal can see this class again.

### The rehearsal's idle outliers — four hypotheses, one survivor

The first epoch-clock rehearsal after the schedule fix completed attempts 0
and 1, then ended attempt 2 on a 159.5 ms solve with load1 at 1.08 and nothing
else running. A 160 ms stall on an idle box would fail G2 on the robot at any
load, so it was run down before anything else (scratchpad probes, uncommitted,
2026-09-13, each a full three-attempt rehearsal):

| Hypothesis | Measurement | Verdict |
|---|---|---|
| Python garbage collection | `gc.callbacks` over 170 installs: largest pause 0.2 ms, zero generation-2 collections | dead |
| Cost grows with the plan record (each splice keeps the whole head) | median solve 29.9–30.8 ms and remainder stage 4.4 ms, flat from 0 to 499 head knots | dead |
| QP iteration count | every solver window took 3–9 iterations; a 51 ms window ran 3 of them, the same work as a typical 2.5 ms window | dead |
| The OpenBLAS thread pool | same rehearsal, `OPENBLAS_NUM_THREADS=1`: worst install 94.9 → 31.8 ms, worst solver window 68.1 → 6.4 ms, installs over 45 ms 6 → 0 | **confirmed** |

It is the UH-3 mechanism (`jugglebot.motion.blas_threads`): six busy-spinning
workers stall a solve made of thousands of small numpy calls. The launch has
capped the planner nodes since 2026-09-06 and both sittings' launch logs read
`blas threads: 1` for `trajectory_node`, so this explains the rehearsal and
not sitting 1. The same probe found the rehearsal passed no QP warm start,
where `trajectory_node` carries one from install to install; carrying it moved
the uncapped worst install from 133.5 to 94.9 ms without changing the median
iteration count (3), so it is fidelity rather than speed.

### Hypotheses withdrawn

- **"The solves are 2–4× slower under sitting load."** Withdrawn as a statement
  about the sitting's load: sitting 2, same launch, read ~1.4× idle at load1
  ~2. Sitting 1 carried roughly five extra cores of load that was already
  building five minutes before its rows (load15 2.96, load5 5.70, load1 7.86 at
  11:35:42). **Not identified.**
- **"The 11:30 launch was never shut down, so two stacks ran."** Withdrawn: the
  11:30 bag's metadata closes at 11:34:15, before sitting 1's rows began, and
  neither later launch log shows a bind or duplicate-node error.

### Where the CPU goes (sitting 2, `pidstat -u 1`, 156 s)

The stack with nothing moving averaged 295 % of 600 % (max 461 %):

| Process | mean %CPU |
|---|---|
| `teensy_bridge_node` | 57.6 |
| python3 launched first (by launch order, rosbridge for the GUI) | 51.5 |
| `trajectory_node` (40 Hz emitter + every solve) | 28.8 |
| `reload_coordinator_node` (idle) | 22.9 |
| `orchestrator_node` | 16.0 |
| `mocap_node` | 11.3 |
| `ros2` (bag record) | 10.4 |
| `skill_node` (idle) | 10.3 |
| `spacemouse_handler` (no device connected) | 7.3 |

An idle catch-with-throw segment solve (36 knots, venv, uncapped BLAS,
2026-09-13) takes p50 22.1 ms: QP 5.1, gate 6.8, remainder 4.2, tilt 3.5,
decompose 2.2 — about 8,750 Python function calls per solve, dominated by small
numpy calls (SVD, inverse, einsum, norms). Capped, as the node runs, a whole
rehearsed session's solver windows read p50 2.2 / p95 4.0 / max 6.4 ms and its
installs p50 10.4 / p95 29.9 / max 31.8 ms. Whether to move any of this out of Python is an open owner
question; the facts above are what it would be decided on.

## Fix

- `motion/skills/schedule.py`: `compile_columns` builds on its own clock and
  shifts once (`_shifted`); `tests/motion/test_skills_schedule.py` pins the
  schedule at five wall-clock magnitudes and the handoff lead at a ROS start.
- `tests/hardware/skills_plan_bench.py`: each attempt waits for the previous plan
  to finish (live: `trajectory/status.plan_time_remaining_s`; rehearse: one record
  carried across attempts); G3 samples the whole attempt, pre-position included
  (sitting 1 had a 40.2 ms gap there that G3 never saw); a refused install keeps
  the budget class it would have spent and G2 reads only installs that spent one;
  the rehearsal runs on an epoch-sized monotonic clock, carries the QP warm start
  as `trajectory_node` does, and caps the BLAS pool to one thread before numpy
  is imported (printing the read-back). Tests in
  `tests/ros/test_skills_plan_bench.py`, including the cap's import order.
- The runsheet's status banner records the two sittings. The remembered-level
  first attempt (the Platform Teensy keeps `levelling=1` in RAM across a relaunch
  and the orchestrator re-pushes the offset; a power-cycle of Jugglebot cleared
  it) is recorded here only; the owner declined a bring-up check for it.

## Verification

- Sitting logs: `temp/logs/skills_plan_bench_{A,B}_loaded_20260913_1{13554,13709,15840,15923}.csv`,
  `..._B_stress2_20260913_{113800,120000}.csv` with their `_meta.json`; launch logs
  `temp/logs/launch_r2gate_20260913_{1130,1134,1157}.log`; bags
  `~/Desktop/rosbags/2026-09-13_{11-30-06,11-34-47,11-57-45}`; load capture
  `~/Desktop/Jugglebot/temp/logs/loadavg_r2gate_20260913.txt`; pidstat
  `~/Desktop/Jugglebot/temp/logs/pidstat_r2gate_20260913.txt`.
- `python3 tests/hardware/skills_plan_bench.py --rehearse --arm B` (2026-09-13,
  idle Jetson, after every fix): `blas threads: 1`; attempts 0, 1, 2 each 22/22
  skills, none ended early; G1 PASS worst 31.16 ms over 176 solves; G2 PASS, 0
  `SPLICE_TOO_LATE`, handoff max 31.16 of 125 ms, unpinned max 11.36 of 75 ms;
  G4 PASS, 107 re-sends refused (expected at ±3 mm); G3/G5 SKIP offline.
- `pytest tests/motion/test_skills_schedule.py tests/motion/test_skills_executor.py
  tests/motion/test_skills_segments.py tests/ros/test_skills_plan_bench.py
  tests/ros/test_install_segment.py tests/ros/test_skill_node.py -q` (2026-09-13):
  135 passed; after the BLAS-order test, `pytest tests/ros/test_skills_plan_bench.py -q`:
  44 passed.
- `./run_tests.sh` (2026-09-13, default gate: the changes are `motion/skills/schedule.py` under `ros_ws/` plus the bench driver and tests, nothing under `controller/` or `sim/`): **6514 passed, 9 skipped** in 244.76 s (parallel) + **3 passed** in 7.99 s (serial); `RESULT: PASS`. That is 11 more than the same gate at `ed8feb9` (6503), the 11 tests added here.

## Handoff

Re-fly rows 15–17 with `pidstat` running. Note load1 before starting a row; a
sitting-1-sized load will fail G1 whatever the code does.
