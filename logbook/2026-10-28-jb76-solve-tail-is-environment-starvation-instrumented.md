---
title: "JB-76: the CATCH solve tail is the launch starving its own solver worker (65 ms of CPU takes 150 ms of wall on a 5.5-of-6-core box); instrumented, measured in one Sitting, low-priority prefixes as the first fix; second Sitting: refusals 42 % -> 1.3 %, wall/CPU 2.18 -> 1.97; fix 2: the tracker's OpenBLAS pool capped; third Sitting: wall/CPU 1.27, CATCH wall p50 95 ms, oval 50/50 record; JB-76 CLOSED, residue filed"
type: investigation
date: 2026-10-28
status: resolved
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

## The Sitting of 2026-10-28 21:02: all three fixes live, py-spy on the nice-0 chain

Recorder `temp/probes/profile_20261028-210241/` (439 s; launch log `~/.ros/log/2026-10-28-21-02-48-831501-jetson-3765883/launch.log`; scan `temp/probes/solve-starvation-2026-10-28-2102.txt`; flame graphs `pyspy_{ball_tracker_node,skill_node,mocap_node}.svg`, py-spy 100 Hz, ptrace_scope set to 0 for the Sitting). Six dispatches: two columns (dwell 0.27) and four `oval_1ball` (dwell 0.5), 188 throws caught, 4 missed. Two records: columns 57 of 59 and **oval 50/50 completed** (previous best 35/35). The tracker's up-line read `BLAS 1 thread`; the report's new nice column read 10 on rosbridge, ball_butler and the bag and 0 on every control-chain process. Comparison by a Sonnet agent against the two earlier Sittings (notes in the scratchpad; the numbers below are the scan's).

| CATCH solves | 09:49 (none) | 14:29 (nice + copies) | 21:02 (+ tracker cap) |
|---|---|---|---|
| n | 206 | 174 | 397 (161 columns, 236 oval) |
| wall p50 / p90 / max | 153 / 226 / 297 ms | 117 / 160 / 223 ms | 95 / 125 / 172 ms |
| CPU p50 / p90 | 67 / 98 ms | 60 / 72 ms | 74 / 92 ms (columns 65, oval 81) |
| wall/CPU p50 / p90 | 2.18 / 2.81 | 1.97 / 2.41 | **1.27 / 1.55** (oval alone 1.28 / 1.52) |
| over 150 ms wall (CATCH + refused) | 162 of 270 | 23 of 176 | 4 of 418 |
| SPLICE_TOO_LATE re-aim refusals | 58 of 154 throws | 1 of 79 | 3 of 188 (all oval, 109 oval throws) |
| other refusals | 6 | 1 | 18 (15 LIMIT_JERK, 2 INFEASIBLE, 1 LIMIT_ACC; none cost a catch) |
| ball_tracker, goals / idle | 129 / 36 % | 148 / 36 % | **66 / 33 %** |
| skill_node, goals / idle | 78 / 61 % | 84 / 68 % | 67 / 56 % |
| box during goals / idle (% of 600) | 555 / 435 | 537 / 423 | 518 / 394 raw; ≈ 479 / 364 net of the three py-spy samplers (≈ 39 / 30 %) |

The CPU p50 rise to 74 ms is goal mix, not a regression: split by the goal in force at each solve, columns CATCHes cost 65 ms (60 at 14:29, a columns-only Sitting) and oval ones 81 ms (92 at 09:49, the same per-pattern split on that log: `oval_1ball` CATCH CPU p50 91.8 ms, `oval_1ball_fed` 94.8 ms). The tracker cap is the largest single saving, about 0.8 core during goals, and `skill_node` fell too (its 300 Hz subscriptions now compete with one fewer busy pool). The three residual SPLICE_TOO_LATE refusals were oval CATCHes at 136–148 ms wall on 83–88 ms of CPU (wall/CPU 1.6–1.7): CPU above the oval median against the ~150 ms unpinned re-aim cliff, with starvation now a minority of the gap. Three further non-terminal `BUMP-DECLINED SPLICE_TOO_LATE` lines were 12–16 mm landing nudges declined because the wire advanced two knots between dispatch and node (a short-plan wire race), and the 12 raw `SPLICE_TOO_LATE` grep hits are these six events logged twice each. No goal ended on it.

**JB-76 closed (owner, 2026-10-28 evening).** The root cause, the launch starving its own solver worker, is removed: 2.18 → 1.27 with oval goals in the scan. The close criterion's second clause (zero SPLICE_TOO_LATE refusals) was missed by three events whose cause is now solve CPU against the budget, which is a different and smaller item, filed as the first residue bug below.

**Flame graphs (py-spy, 100 Hz, 439 s; shares are of each node's samples, cores are estimates from pidstat × non-idle share).** Blocked threads (`thread.py:78` queue.get, `executors.py:593` wait set) are counted as idle.

- `skill_node` (80 k samples, 27 % non-idle; pidstat 0.67 core during goals, so ≈ 0.5 core of it is the executor machinery below): about 79 % of the work is rclpy `MultiThreadedExecutor` machinery, not callbacks: the per-spin wait-set rebuild (`executors.py:493–661`, `handle.py:116–127`), task/future construction (`task.py:201/211`, `thread.py:160–195`) and `_take_subscription` (`executors.py:347`), driven by the 200–300 Hz `rigid_body_poses` and `balls` subscriptions (`skill_node.py:1334–1406`). Callback bodies are about 12 %: `_on_tick` 2.7 % (mostly `_wait_future`), `_on_balls` 1.4 %, logger `_find_caller` 0.2 %.
- `mocap_node` (44 k samples, 82 % non-idle): the 100 Hz `_publish_mocap_data` timer is 54 %: the three full-rate publishes (`mocap_node.py:1113`, `:1170`, `:1185`) 24 % incl. C-level serialise (of which `bb/markers` is the 3.5 % below), `RigidBodyPose()` nested construction (`:1152`) 6.7 %, `_feed_robot_frame` + `settled()` per frame (`:1131`, `robot_frame.py:572/630`) 4 %, `bb/markers` built and published even when empty (`:1179–1185`) 3.5 %, `now().to_msg()` 2.2 %. QTM parse (`mocap_interface.py:224`, `qtm_rt/protocol.py:148`) 6.4 %; calibration solve thread 4.7 %. The 20 Hz GUI copies cost about 1 % (`:1115`, `:1172`): working as designed.
- `ball_tracker_node` (31 k samples, 94 % non-idle): message take and Python object construction for every marker of every 300 Hz frame (`executors.py:347`, `_mocap_data_single.py:78–136`, `_point.py:74–79`) 27 %; `process_frame` 17 %, of which the per-frame landing refit (`matcher.py:1051` → `flight_fit.py:215`, `lstsq` at `:172`) is 10.6 %, the gate norm 2.4 %, Kalman 4.1 %; `_ball_to_msg` (`BallState()` + `now()`, `ball_tracker_node.py:284–285`) 12 %; publish at 300 Hz 8 %; per-marker `np.array` (`:262`) 1.9 %. That is where the 1.5 ms in-flight frame goes: about half of it (27 + 12 + 8 + 2 of 94 % non-idle) is message plumbing rather than maths, and the maths share that remains is dominated by the per-frame refit.

**Also in this log, filed separately, none a solver symptom:** three goals ended `ABORTED_NO_RELEASE` ("throw N never left the hand", new tonight); a Teensy guard `MAX_DEVIATION` latch on the hand (dev +8.4 rev) during oval goal 4 → ACTIVE → FAULT → LANE_REFUSED, `recover_lane` reporting nothing latched, then a `WRONG_MODE` rejection of the re-dispatch from STANDBY; mocap `queue full` loss of 544 frames in two bursts between t+44 and t+85 s before the first dispatch (490 at 09:49, 373 at 14:29, zero overflow during goals); ten leg heartbeat-dropout episodes (497–1599 ms) and two `LATENCY MONITOR CLAMP_DUTY` warnings (0.27 vs 0.15) on a bridge at 6.7 h uptime. Zero SETPOINT_STALE, E-stop or NO_HEARTBEAT; CPU 59.6 °C max, clocks pinned.

## What remains, and where it lives (2026-10-28 evening)

1. ~~Fly a scanned Sitting~~ done three times today; JB-76 closed on the 21:02 read-out.
2. ~~Tracker thread cap~~ fix 2, verified live (148 → 66 %).
3. **Worker real-time priority** (owner's pick for the next lever): make CATCH wall ≈ CPU regardless of box load by taking the solver worker out of the fair-share queue (`SCHED_FIFO`, bounded at ~100 ms CPU per solve, fail-open to CFS when the rtprio limit is 0). Needs a one-time `sudo` edit of the rtprio limit. Filed as its own feature item.
4. The nice-0 CPU trims from the flame graphs (skill_node executor ≈ 0.5 core, mocap_node publish/construction ≈ 0.28, tracker message plumbing ≈ 0.28), filed as one feature item with the file:line anchors above. `teensy_bridge_node` (79 % in every Sitting) and rosbridge (70 % during goals) were not sampled and are the next py-spy targets.
5. The oval re-aim residue (CATCH CPU 81 ms vs the ~150 ms unpinned cliff; 3 refusals in 109 oval throws) filed as a bug; it is the reason the close criterion was missed by three events.
6. A Jetson restart was never a fix: the load is the running launch.

## Verification

- Instrumentation commit, 2026-10-28: `pytest -q tests/motion/test_solver_worker.py tests/ros/test_install_segment.py tests/motion/test_solver_worker_lateness.py` gave 93 passed before the test was narrowed to a BLAS-capped worker; afterwards `tests/motion/test_solver_worker.py` gave 16 passed. Gate `./run_tests.sh`, 2026-10-28: 7369 passed / 4 skipped + serial 5 passed, PASS.
- Fix 1 commit, 2026-10-28: `pytest -q tests/ros/test_launch_priority.py` 1 passed; `./run_tests.sh` 7370 passed / 4 skipped + serial 5 passed, PASS (`temp/logs/jb76-nice-gate.log`); the scan reproduces every table above from the saved Sitting data (`python tools/probes/solve_starvation_scan.py ~/.ros/log/2026-10-28-09-49-31-556395-jetson-3246295/launch.log temp/probes/profile_20261028-094925`).
- Fix 2 commit, 2026-10-28: offline measurement as tabled above (`temp/probes/tracker_blas_cap_2026-10-28/report.md`); `pytest -q tests/ros/test_launch_nodes.py tests/ros/test_launch_priority.py` 18 passed (the cap test now parametrised over `CAPPED_NODES`), the seven `tests/ros/test_ball_tracker_*.py` files 55 passed with the node's new `check_blas_threads` receipt; the `./run_tests.sh` triple is in the commit message.
- 21:02 Sitting read-out, 2026-10-28: `python tools/probes/solve_starvation_scan.py ~/.ros/log/2026-10-28-21-02-48-831501-jetson-3765883/launch.log temp/probes/profile_20261028-210241` (output `temp/probes/solve-starvation-2026-10-28-2102.txt`) gives the right-hand column; goal and refusal counts from the launch log's `Juggle requested via` / `ENDED` / `done:` / `throw N/M` / `then refused` lines (20 refusals = 15 LIMIT_JERK + 3 SPLICE_TOO_LATE + 2 INFEASIBLE; the scan counts 21 because it also sees one worker LIMIT_ACC that reached no throw line, and its 3 `transit` rows are the same three RESEND-REFUSED SPLICE_TOO_LATE events); the per-pattern CPU split from the scan's parsed solves bucketed by the goal in force; flame-graph shares parsed from the SVGs' `<title>` sample counts. `grep "Ball tracker up" launch.log` ends `BLAS 1 thread`.
- 14:29 Sitting read-out, 2026-10-28: `python tools/probes/solve_starvation_scan.py ~/.ros/log/2026-10-28-14-29-37-005424-jetson-3516886/launch.log temp/probes/profile_20261028-142931` (output `temp/probes/solve-starvation-2026-10-28-1429.txt`) gives the right-hand column of the table; the goal/re-aim counts come from the launch log's per-throw lines (`grep -c "then refused" launch.log`: 64 vs 1; denominators are CAUGHT throw lines, 154 vs 79). `profile_session.py`'s snapshot parser was checked on the old-format 14:29 log (491 PIDs resolved, empty nice map) and on a synthetic new-format block (nice 10 / 0 / `-` read back).
