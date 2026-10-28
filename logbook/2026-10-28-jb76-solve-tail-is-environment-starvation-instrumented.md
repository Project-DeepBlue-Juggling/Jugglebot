---
title: "JB-76: the CATCH solve tail is the launch starving its own solver worker (65 ms of CPU takes 150 ms of wall on a 5.5-of-6-core box); instrumented, measured in one Sitting, low-priority prefixes as the first fix; second Sitting: refusals 42 % -> 1.3 %, wall/CPU 2.18 -> 1.97; fix 2: the tracker's OpenBLAS pool capped (40-45 % of its CPU for nothing)"
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
  - tools/probes/solve_starvation_scan.py
  - tools/probes/README.md
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - tests/ros/test_launch_priority.py
  - tests/motion/test_solver_worker.py
  - tests/ros/test_install_segment.py
  - ros_ws/src/jugglebot/jugglebot/ball_tracker_node.py
  - tests/ros/test_launch_nodes.py
subsystem:
  - motion
tags:
  - solver-worker
  - splice-too-late
  - jetson-load
---

# JB-76: the solve tail is the launch starving its own solver

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

## The Sitting of 2026-10-28 09:49 (same day, second half of this entry)

Flown with the recorder running (`temp/probes/profile_20261028-094925/`, 482 s; launch log `~/.ros/log/2026-10-28-09-49-31-556395-jetson-3246295/launch.log`; bag under `~/Desktop/rosbags/2026-10-28_09-49*`). Nine goals; none ended SPLICE_TOO_LATE, 56 re-aims were refused on it. Read with the new `tools/probes/solve_starvation_scan.py` (output `temp/probes/solve-starvation-2026-10-28.txt`).

**Starvation confirmed.** CPU time is the intrinsic cost; the rest is waiting for a core.

| Solve | n | wall p50 / p90 | CPU p50 / p90 | wall/CPU p50 | ivcsw p50 |
|---|---|---|---|---|---|
| CATCH, accepted | 206 | 153 / 226 ms | 67 / 98 ms | 2.18 | 59 |
| refused (30 SPLICE_TOO_LATE, 26 transit, 8 planner codes; a refusal line carries no kind, assumed CATCH) | 64 | 174 / 210 ms | 63 / 80 ms | 2.71 | 74 |
| THROW | 8 | 63 / 90 ms | 40 / 48 ms | 1.66 | 36 |
| REST | 19 | 54 / 100 ms | 41 / 49 ms | 1.41 | 35 |

- 162 of 270 CATCH-class solves took over 150 ms of wall time; **zero** took over 150 ms of CPU. The 150 ms re-aim cliff sits below the median wall time.
- 9 involuntary switches per 10 ms of CPU: the worker loses its core about once a millisecond.
- The handoff CATCHes survived only because their 350 ms budget cleared the 297 ms worst case.

**The load is the launch itself.** Before launch the box was 1–8 % busy (`vmstat.log`, the six rows before 09:49:31). Launch up and idle: 435 % of 600. During goals: **555 % of 600**, run queue p50 13 on 6 cores, 27 k context switches/s. Per process, mean % of one core over the Sitting (during goals / idle in brackets):

| Process | mean % | note |
|---|---|---|
| `rosbridge_websocket_lean` | 86 (77 / 86) | one GUI client |
| `ball_tracker_node` | 83 (129 / 36) | above one core while juggling: threads busy outside the GIL |
| `teensy_bridge_node` | 79 (78 / 72) | |
| `skill_node` | 73 (78 / 61) | |
| `mocap_node` | 71 (66 / 68), peaks 238 (`report.md`) | |
| `ball_butler_node` | 37 | |
| `trajectory_node` | 30 | |
| `ros2 bag record` | 19 | |
| `orchestrator_node` | 18 | |
| solver worker | 12 | 65 ms bursts, ~45 % of a core each |

Not on the list: the two Claude CLIs (3 % together), VS Code, the map app. One Claude Code auto-update (`npm install`) ran 15 s at 81 % with no solve inside its window. Swap did not page (`si`/`so` 0). The mocap queue peaked at 9 of 150 with zero overflows (the launch log's 1 Hz `queue max` lines), so the overflow seen on 10-24 and 10-27 was a fellow victim of crowding on worse days, not the cause; with it absent the starvation was still 2.2×.

**Why the budgets stopped fitting.** `SOLVE_BUDGET_KNOTS` was sized on 2026-09-18 from CATCH p50 76 / p95 113 ms. The intrinsic solve has not grown; the launch has (Ball Butler feed chain, GUI traffic, the tracker's matcher), past the point where fair-share scheduling still hands the worker a whole core.

## Fix 1 (this commit): low priority for the processes that are not on the control chain

`jugglebot_launch.py` adds `prefix='nice -n 10'` to `rosbridge_websocket`, `rosbag_record`, `spacemouse_handler` and `ball_butler_node`. Under CFS nice 10 weighs about 1/9 of nice 0, so under contention they yield to mocap → tracker → skill → trajectory + worker → teensy_bridge, and to `orchestrator_node` (owner: it dispatches the skills and handles the ENDs, so it stays at nice 0). They keep every idle cycle; only who waits changes. No root needed. Expected: CATCH wall time back near 80–100 ms. Costs: GUI stutter during solve bursts, and GUI-relayed commands (the Soft E-stop button included) reach ROS tens of ms later under contention; the bag may drop messages; `ball_butler_node`'s own scheduling latency grows, which the Feed tolerates because the BB release instant is an absolute time the BB itself holds (protocol 10), not a Jetson-timed send. Verified on this box by resolving every process's prefix from the real launch objects (exactly the four carry `nice -n 10`; `--print` does not show prefixes) and pinned by `tests/ros/test_launch_priority.py`, which also asserts that no control-chain node carries any prefix; the gate is the next Sitting's scan.

## The Sitting of 2026-10-28 14:29: fix 1 and the GUI copies live

Flown with the recorder (`temp/probes/profile_20261028-142931/`, 317 s; launch log `~/.ros/log/2026-10-28-14-29-37-005424-jetson-3516886/launch.log`), the first Sitting with `nice -n 10` on the four non-control processes and the GUI on the 20 Hz `gui/` mocap copies (`2026-10-28-gui-mocap-copies-cut-rosbridge-load.md`). Columns only (five goals plus one 5-throw `columns_1ball_fed`), 79 counted throws, every one caught; two goals completed (5/5 and a 49/49 columns record), four ended `MISSED_CATCH` on a CATCH that produced no ball. The 09:49 Sitting mixed oval and columns, so the per-goal numbers are not comparable; the solve statistics are. Scan output `temp/probes/solve-starvation-2026-10-28-1429.txt`; the comparison was done by a Sonnet agent against both launch logs and both profile directories.

| CATCH solves | 09:49 (before) | 14:29 (after) |
|---|---|---|
| wall p50 / p90 / max | 153 / 226 / 297 ms | 117 / 160 / 223 ms |
| CPU p50 | 67 ms | 60 ms |
| wall/CPU p50 / p90 | 2.18 / 2.81 | 1.97 / 2.41 |
| ivcsw p50 | 59 | 47 |
| over 150 ms wall (CATCH + refused) | 162 of 270 (60 %) | 23 of 176 (13 %) |
| re-aims refused, per caught throw | 64 of 154 (42 %; 58 SPLICE_TOO_LATE) | 1 of 79 (1.3 %; one SPLICE_TOO_LATE at throw 48/50; the scan shows a second, LIMIT_ACC, worker refusal that reached no throw line) |
| rosbridge, mean % of a core | 86 | 56 |
| box during goals / idle, % of 600 | 555 / 435 | 537 / 423 |

**The symptom is mostly gone; the cause is not.** The refusal rate fell thirty-fold because the wall tail moved under the ~150 ms re-aim cliff, but the worker is still descheduled for about half its wall time: the pass mark ("wall/CPU well under 2.18") was missed at 1.97. The box still runs at 537 of 600 % during goals with all six cores saturated for seconds at a time. The roughly 0.4 core released during goals by rosbridge (77 → 46 %) and `ball_butler_node` (36 → 25 %) shows up as only 18 points less box load (555 → 537): the nice-0 processes absorbed part of it (tracker +19, `skill_node` +6) and the rest went unused, while the worker's own share of the run queue barely moved. The largest of those is `ball_tracker_node`: 36 % idle, **148 % during goals**, which a single-threaded Python node cannot reach, so it is the uncapped OpenBLAS pool (the launch's `_planner_blas_env` covers only the three planner callers). `skill_node` went 61 → 68 % idle and 78 → 84 % during goals; `mocap_node` and `teensy_bridge_node` are unchanged. The nice-10 set now yields: `ball_butler_node` 36 → 25 % during goals, rosbridge 77 → 46 %.

Two things this Sitting could not show. The nice values themselves: `profile_session.py`'s ps snapshot had no `ni` column (fixed in this commit; the only direct trace is the bag's recorded command line `nice -n 10 ros2 bag record ...`), so the four prefixes are verified by `tests/ros/test_launch_priority.py` and by the yielding pattern above, not by a read of the running processes. And a pattern-controlled comparison: columns CATCH solves may be lighter than oval ones, so a second scanned Sitting with an oval goal is needed before JB-76 closes. Secondary indicators were null: zero mocap overflows in either Sitting, no SETPOINT_STALE, no E-stop, no hand fault.

**Working read-out for the operator.** Fix 1 plus the copies bought roughly 35 ms of CATCH wall p50 and removed the refusals at today's load; a heavier Sitting (oval, two balls, more GUI panels) will push the tail back over the cliff because the box is still over 5.3 cores busy. The next lever is the tracker's thread cap.

## Fix 2 (this commit): the tracker's OpenBLAS pool capped to one thread

The launch comment of 2026-09-06 left `ball_tracker_node` and `mocap_node` out of the BLAS cap because they do "genuinely large-matrix work where a real thread pool earns its keep". For the tracker that was never true: `tracking/kalman.py` runs 6×6 predict/update steps (`F @ P @ F.T`, a 3×3 `inv`), `tracking/flight_fit.py` one `lstsq` on an (n, 2) matrix per fit pass, and `tracking/matcher.py` 3-vector norms in Python loops; no scipy. Every one of those calls fanned out to OpenBLAS's six-worker pool, which is how a single-threaded Python node reached 148 % of a core while juggling.

Measured offline by a Sonnet agent, box idle (load 0.37), replaying the 14:29 bag (88 666 frames, 297.8 s of bag) through the real tracker with `tools/probes/tracker_bag_replay.py`, three interleaved uncapped/capped pairs (two under the system Python 3.8 that runs the node, one under the venv; both numpy 1.24.4 on OpenBLAS 0.3.21 pthreads). Outputs under `temp/probes/tracker_blas_cap_2026-10-28/` (`report.md`, per-run `time.txt`, replay CSVs, `lat_sys_*`).

| | uncapped | capped |
|---|---|---|
| OpenBLAS threads (`threadpoolctl`) | 6 | 1 |
| whole-replay CPU, three pairs | 386.7 / 412.0 / 352.8 s | 232.4 / 225.2 / 211.9 s (−40 / −45 / −40 %) |
| whole-replay wall | 225.9 / 244.4 / 209.0 s | 233.5 / 226.8 / 212.8 s (within spread) |
| CPU / wall | 1.69–1.71 | 0.99–1.00 |
| per-frame `process_frame`, in-flight, p50 / p95 / p99 / max | 1583 / 2149 / 2486 / 11465 µs | 1536 / 2065 / 2382 / 3302 µs |
| per-frame, no ball in flight, p50 / p95 / p99 / max | 25 / 38 / 57 / 1661 µs | 24 / 34 / 49 / 1661 µs |
| per-ball result CSV | md5 `030b9438…` | identical, all six runs |

The sanity bag (2026-10-27 22:59, 150 468 frames) repeats it: in-flight p50 885 → 876 µs, max 3998 → 2125 µs, CPU/wall 1.55 → 1.00. The cap costs nothing measurable at any percentile, removes the pool's spin (CPU/wall 1.7 → 1.0), trims the worst single frame by 3×, and leaves the tracker's output byte-identical. Scaled to live rates the tracker drops from about 1.3–1.4 to 0.7–0.8 cores while a ball flies: roughly half a core returned to the box, where the solver worker waits. Under contention the benefit is inferred from the idle-box measurement, not measured; the next scanned Sitting measures it.

Not applied to `mocap_node`: its visible linear algebra is 3-vector and quaternion norms, but it owns a `CalibrationSolveWorker` thread whose matrix sizes were not surveyed, so it stays uncapped and `tests/ros/test_launch_nodes.py` keeps that boundary explicit (the test's parameter list shrank from two nodes to one). One open question from the probe: an in-flight frame costs ~1.5 ms of single-thread CPU, a lot for 6×6 maths; not profiled here, and a candidate for the py-spy step.

`jugglebot_launch.py`: `ball_tracker_node` gets `additional_env=dict(_planner_blas_env)` (its block moved below the dict), the comment rewritten with the measurement. `ball_tracker_node.py` reads the cap back at start-up through `blas_threads.check_blas_threads` and folds `BLAS 1 thread` into its "Ball tracker up" line, WARNing if the cap did not land. `tests/ros/test_launch_nodes.py`: `CAPPED_NODES` = planners + tracker, the not-launch-wide test keeps only `mocap_node`.

## Next, in order (owner-agreed 2026-10-28, renumbered after the 14:29 Sitting)

1. ~~Fly a Sitting with the recorder, run the scan, compare wall/CPU against 2.18.~~ Done 14:29: 1.97, improved but not met; see above.
2. Trim the load where the table points: ~~the tracker's 148 % during goals~~ done as fix 2 above (capped; measured offline, unverified live); then `py-spy` on `skill_node` (84 % during goals), `teensy_bridge_node` (77 %) and the tracker's 1.5 ms in-flight frame. rosbridge's share was the GUI copies (done, 86 → 56 %); the lean server's deserialise-after-throttle (option 4) stays optional.
3. Core isolation for the worker only after step 2 frees headroom: today 5.3 cores of demand on the 5 remaining cores would squeeze the setpoint stream.
4. The 150 ms unpinned re-aim cliff stays structurally thin and is a separate decision.
5. A Jetson restart is not a fix: the load is the running launch.
6. Close JB-76 only after a scanned Sitting that includes an oval goal reads wall/CPU well under 2 with zero re-aim refusals.

## Verification

- Instrumentation commit, 2026-10-28: `pytest -q tests/motion/test_solver_worker.py tests/ros/test_install_segment.py tests/motion/test_solver_worker_lateness.py` gave 93 passed before the test was narrowed to a BLAS-capped worker; afterwards `tests/motion/test_solver_worker.py` gave 16 passed. Gate `./run_tests.sh`, 2026-10-28: 7369 passed / 4 skipped + serial 5 passed, PASS.
- Fix 1 commit, 2026-10-28: `pytest -q tests/ros/test_launch_priority.py` 1 passed; `./run_tests.sh` 7370 passed / 4 skipped + serial 5 passed, PASS (`temp/logs/jb76-nice-gate.log`); the scan reproduces every table above from the saved Sitting data (`python tools/probes/solve_starvation_scan.py ~/.ros/log/2026-10-28-09-49-31-556395-jetson-3246295/launch.log temp/probes/profile_20261028-094925`).
- Fix 2 commit, 2026-10-28: offline measurement as tabled above (`temp/probes/tracker_blas_cap_2026-10-28/report.md`); `pytest -q tests/ros/test_launch_nodes.py tests/ros/test_launch_priority.py` 18 passed (the cap test now parametrised over `CAPPED_NODES`), the seven `tests/ros/test_ball_tracker_*.py` files 55 passed with the node's new `check_blas_threads` receipt; the `./run_tests.sh` triple is in the commit message.
- 14:29 Sitting read-out, 2026-10-28: `python tools/probes/solve_starvation_scan.py ~/.ros/log/2026-10-28-14-29-37-005424-jetson-3516886/launch.log temp/probes/profile_20261028-142931` (output `temp/probes/solve-starvation-2026-10-28-1429.txt`) gives the right-hand column of the table; the goal/re-aim counts come from the launch log's per-throw lines (`grep -c "then refused" launch.log`: 64 vs 1; denominators are CAUGHT throw lines, 154 vs 79). `profile_session.py`'s snapshot parser was checked on the old-format 14:29 log (491 PIDs resolved, empty nice map) and on a synthetic new-format block (nice 10 / 0 / `-` read back).
