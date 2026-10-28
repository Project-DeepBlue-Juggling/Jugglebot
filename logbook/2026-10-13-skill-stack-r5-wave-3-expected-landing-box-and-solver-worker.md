---
title: R5 wave 3 - the admissible box bounds the learner's EXPECTED landing, the segment solve moves into a worker process, the lane-latch rehearsal becomes a script, BB FW 9 flashed
type: investigation
date: 2026-10-13
status: resolved
phase: "two-ball-skill-stack — R5 (two-ball columns)"
related_plan: two-ball-skill-stack.md
related:
  - 2026-10-13-skill-stack-r5-wave-2-after-sitting-7.md
  - 2026-10-13-skill-stack-r5-sitting-7-columns-pb.md
files_changed:
  - CLAUDE.md
  - logbook/2026-10-13-skill-stack-r5-wave-2-after-sitting-7.md
  - logbook/2026-10-13-skill-stack-r5-wave-3-expected-landing-box-and-solver-worker.md
  - logbook/INDEX.md
  - plans/active/two-ball-skill-stack.md
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/motion/skills/learner.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/memory.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/solver_worker.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - run_tests.sh
  - sim/skills_gate.py
  - tests/hardware/session_skills_r5_sitting8.md
  - tests/motion/test_lane_latch_rehearsal.py
  - tests/motion/test_learner.py
  - tests/motion/test_skills_executor.py
  - tests/motion/test_solver_worker_lateness.py
  - tests/motion/test_solver_worker.py
  - tests/ros/conftest.py
  - tests/ros/test_install_segment.py
  - tools/probes/lane_latch_rehearsal.py
  - tools/probes/README.md
subsystem:
  - skill-stack
  - trajectory-node
  - ball-butler
tags:
  - admissible-box
  - learner
  - solver-worker
  - emitter-stall
  - lane-latch
  - hardware-sitting
---

# R5 wave 3: answering the wave-2 owner decisions

## What happened

Wave 2 ended with a list of owner decisions. The owner answered on the evening of 2026-10-13 and wave 3 built the answers. Nothing is flown.

| Owner answer (2026-10-13 evening) | Unit | What landed |
|---|---|---|
| Box: "Instead of clipping the learner's aim, run the aim through the learner's own prediction of where the ball will land and clip that. The box then bounds where the ball is expected to go, not the number the learner asks for." | H | `learner.local_model`, `Memory.local_model`, `SkillExecutor._forward_clip_xy`; box values and gate hash `5072c89d4ddc` unchanged |
| Stall: "move the planner into its own process so the stream is never starved." | I, I1, I2 | `motion/skills/solver_worker.py` (new), `trajectory_node` seam, new refusal `SOLVER_UNAVAILABLE` |
| The hand stays where it stopped after an automatic lane recovery (no park) | none | Already the behaviour; runbook row 18 states it as the owner ruling |
| The hop entry counts as "BB initiates" for the 30-consecutive gate | orchestrator | Plan assumption confirmed; runbook row 25 |
| Lane-recovery dress rehearsal approved; write a command the operator runs | J | `tools/probes/lane_latch_rehearsal.py` (+ 10 tests, README row); runbook rows 17a-17d |

Ball Butler FW 9 was FLASHED 2026-10-13 20:09 (dry-run first; receipt "Ball Butler FW version: 8 -> 9", log `temp/logs/bb_fw9_flash_20261013.log`). The wave-2 cancel rung (row 19) is now runnable.

## Discussion

### 1. Why the box on the EXPECTED landing is the physically right bound

The box was swept on the CATCH transit: the leg motion from the previous release to the catch point. Unit H read the S10b columns sweep log (`temp/logs/admissible_sweep_S10b_columns_20261011.log`) at the nominal 0.8804 s flight. Every refusal at -20/-10/-8/-6 and +6/+8/+10/+20 mm is `CHAIN_CATCH:*`, `MARGIN` or `LIMIT_ACC`; none is `LAUNCH:*`, and the launch at a -20 mm command passed. So the swept quantity is where the ball LANDS. Clipping the COMMAND bounds a number the plant then turns into a landing through its own bias: sitting 7's commanded -24 mm was clipped to -4 mm while the plant added about +27 mm. The failure class is "the bound is applied to a proxy of the quantity that was swept".

What stays a command clip: the apex. The apex limit is the hand's envelope on the commanded release speed, and the box's apex range (0.608-1.047 at the 0.95 band) is the band of commanded flights that passed (`SINGLE_APEX_FLIGHT_FRAC`, rungs -20..+10 %), so it is already in command units.

How the lateral clip works (`executor._forward_clip_xy`, executor.py:2116):
1. Thin support (`local_model` returns None) gives today's clip, silently.
2. Expected landing inside the box: the command passes unchanged (`BOX-EXPECTED-LANDING ... inside`).
3. Outside: the EXACT inverse `Dxy^-1` pull-back puts the expected landing on the box edge (`... on the box edge`). The alternative was the eta-damped learner step, which would need several throws to reach the edge and would leave the first throws outside it; the inverse is one step and uses the same local fit that predicted the landing.
4. The lateral authority (20 mm) is re-applied. Unit H widened its band to include the box's own range, because without that a desired landing outside the box by more than the authority would command further out than today's clip (a discontinuity between a thin fit and an identity fit). With `y_d = 0`, every columns throw, the band is exactly +-a.
5. A fit whose gain is below `BOX_FORWARD_GAIN_FLOOR = 0.5` (smallest eigenvalue of the symmetric part of the 2x2 xy block of D, executor.py:641), a sign-flipped or rotated block, or a failed fit gets today's clip (`BOX-FORWARD-CLIP-FALLBACK`). The floor caps the pull-back step at twice the landing excess. D is the identity prior to about 1 % (the ridge in `learner.py`'s docstring), and every bias measured on this plant is an offset `d`, not a gain, so the floor is a guard against a corrupted fit, not a tuned value.

The case ACCEPTED, not refused: when the authority binds, the box cannot be met. The command is accepted and logged `BOX-EXPECTED-LANDING-UNMET`. Reasoning: owner decision 3 (wave 2) made the 20 mm authority the bound on the command; the catch authority (35 mm inward) and the live `validate_cycle` QP gate still bound the catch, so an unmet box never reaches the wire unvalidated. Refusing would end the attempt on a plant bias the learner is trying to correct, which is the failure that stopped sitting 7.

Tradeoff and what it does. With a +27 mm lean the expected landing at -20 mm is about +7 mm, still outside [-4,+2]. Sitting-7 case before: command -4, lands about +23. After: command -20, lands about +7, inside the 35 mm inward catch authority. The command's lateral launch velocity grows by about 23 mm/s over a 0.88 s flight. The command is still computed once per throw and cached, so nothing changes mid-transit.

Honest gap: the sim gate is BLIND to this rule. Columns runs with `boxes=None` (`sim/skills_gate.py`) and no lateral authority, and the self_toss box (+-30-40 mm) never binds. Unit H's before/after sim runs are identical (self_toss 25/0/3 x3 PASS, columns 25/0/25 x2 PASS; 69 `inside` lines, no pull-back). The sitting-7 case is pinned only by unit tests (`test_box_forward_clip_sitting_7_lean_keeps_the_authority_command`, `..._pulls_the_expected_landing_onto_the_edge`).

### 2. The solver worker

Wave-2 unit E showed the 45-57 ms stall is trajectory_node's OWN 135 ms CATCH `install_segment` solve starving its 40 Hz emitter thread in the same interpreter (emitter gaps p50 25.7 / p99 34.3 / max 45.7 ms; at least 45 ms about every 16 s). Unit I measured the alternative:

| Measurement (2026-10-13 20:17-20:25, idle box, launch down) | Value |
|---|---|
| Round-trip overhead, real CATCH (48-77 kB request, 62-86 kB reply) | median 1.86-2.27 ms, max 2.84 ms; about 1.1 ms of it holds the node's GIL |
| 600 real CATCH solves through the worker, emitter lateness | max 1.69 ms, no gap > 30 ms |
| Synthetic 150 ms solve, emitter lateness | 149-277 ms in-process; at most 0.37 ms via the worker |
| Spawn cost at 433 MB RSS | `posix_spawn` 0.61 ms; `subprocess.Popen` 17.7 ms (holds the GIL in 3.8) |
| Ready time | multiprocessing-spawn 0.68-0.74 s (re-runs the colcon wrapper's `pkg_resources`); `-m` 0.33 s |

Why `posix_spawn` of `-m jugglebot.motion.skills.solver_worker --fd 3`, not fork and not multiprocessing: fork copies a multithreaded process (rclpy/DDS threads, the emitter, the ZMQ io thread) with their locks; multiprocessing-spawn pays the wrapper re-import and a resource tracker; `Popen` forks under the GIL. `posix_spawn` execs a fresh image at 0.6 ms.

Why everything ships per call (request fields: record, seed rest, kind, terminal, `t_now_s`, lead, `cfg`, `limits`, `geom`, warm start, deadline): `cfg` and `limits` are live-mutable (`pre_release_hold_s`, `set_limits`), and everything except the record totals under 3 kB. Per-call shipping removes the staleness class (a changed limit reaches the next install by construction) at the price of the record's 12-78 kB, which grows with the chain (a splice keeps the head). The worker keeps no state; the node still carries the warm start.

Failure policy: `SOLVER_UNAVAILABLE` for a dead, hung (2 s), not-ready, wrong-tree or twice-failed worker; one restart per episode. THROW/CATCH wait until `t_wall + lead + dt` and then refuse with the existing `ORIGIN_TOO_LATE`/`SPLICE_TOO_LATE`; REST waits 1.0 s. **No in-process fallback, ever.** The worker exists so a 60-206 ms solve never shares an interpreter with the 40 Hz emitter. A fallback would put the solve back exactly when the system is degraded, and silently: installs would keep being accepted and the only symptom would be the 45 ms stalls that latched a lane, invisible until the firmware latches. A refused install fails loud and safe: the attempt ends rest-terminal, the wire keeps streaming the record it has, and the operator sees `SOLVER_UNAVAILABLE` by name. `InProcessSegmentSolver` exists only as a test double (an autouse fixture in `tests/ros/conftest.py`), and a test pins that the production default is `WorkerSegmentSolver`.

**A design hypothesis the measurement overturned: the 3 ms return-transit allowance.** The design had the worker evaluate the executor's lateness check at `now + 3 ms` and the node refuse a reply that arrives after that (`reply.t_inst_s`). Unit I2's real-worker smoke refused EVERY real splice (3 of 3): the reply reached the node 6.1-6.3 ms after `t_inst_s`, because the executor reads its clock BEFORE `splice_at` and the result build, and then the reply is pickled, piped and unpickled. As built, the node re-evaluates the executor's own splice fence at ITS clock immediately before `_install` (`trajectory_node._reply_outran_wire`, trajectory_node.py:4679): refuse iff `k_s <= floor((now - t0)/dt) + WIRE_READ_KNOTS`, with `k_s = 0` for a reserved fresh origin and a fresh REST exempt because the executor rebases it. Same timing codes, still fail-closed (a SIGSTOPped node wakes to a seconds-late reply and refuses), no heuristic allowance for a splice. A fresh REST is the one case that rule cannot judge (its un-rebased origin is legitimately behind the clock), so the audit added a stall test off the worker's own clock read: refuse `ORIGIN_TOO_LATE` when more than 0.5 x WIRE_READ_KNOTS x dt (37.5 ms) has elapsed since `t_inst_s - REPLY_TRANSIT_ALLOW_S` (normal transit 6-9 ms; the in-process double's `inf` never trips it). After the change the same smoke accepted 3 of 3. `reply.t_inst_s` and `REPLY_TRANSIT_ALLOW_S` remain in the worker (its own check runs 3 ms early) but the node does not use them. Orchestrator accepted the deviation 2026-10-13 ~21:30.

**Control-system walk-through, one 40 Hz cycle (dt = 25 ms).** (1) The callback runs the guards, ROS-to-perf crossing, terminal and the record snapshot under `_plan_lock` in under 1 ms. (2) The request is pickled: 0.5-0.6 ms holding the GIL, so the emitter's next wake is at most one pickle late. (3) `send_bytes` then `poll`/`recv_bytes` sleep in the kernel with the GIL released. (4) During the 40-206 ms solve the emitter keeps its absolute 25 ms grid and samples the old record under `_plan_lock`; the worker runs on another core, so the wire and the hand feedforward `J.alpha` see the same stream as with no install in flight. (5) The reply is unpickled (0.5 ms), then epoch check, `_install_continuity_ok`, `_reply_outran_wire`, and `_install` under `_plan_lock`. The plan the emitter receives is bit-identical to the in-process result: same function, same pickled inputs, float64 round-trips are exact. No new discontinuity source. A hold during the solve is refused `SUPERSEDED_BY_HOLD` as today; the worker's result is wasted compute and nothing is cancelled across the pipe.

Two implementation finds (unit I1): (a) run as `__main__`, the reply dataclasses pickled as `__main__.SegmentSolveReply` and the parent could not unpickle them; `_main` now serves via the canonical module import, do not "simplify" it. (b) R6, the two unattributed 17-19 ms emitter wakes in the design probes, are attributed: GC is ruled out (`gc.callbacks`, zero collections in 6 runs); spikes of 4-7 ms appear ONLY when the worker runs without the launch's BLAS pin (its multithreaded BLAS competes for the emitter's core). The worker inherits the pin through `os.environ` (`_planner_blas_env`); keep it. READY logs `blas_threads=` and WARNs if it is not 1.

Not chosen, the two options unit E left on the table: `SCHED_GRACE_S` 20 -> 45 ms (a firmware safety-review change that widens how stale a streamed lane may be before the Teensy refuses it, for a stall the Jetson can instead stop causing) and `EMIT_LEAD_KNOTS` 0 -> 1 (adds 25 ms to every tracking delay to hide an occasional gap). Both treat the symptom; moving the solve removes the cause. The design's verification step 1 (`gilprobe --rehearse` through the worker) was NOT re-run as such; the bar is covered by I1's serial lateness test (40 real CATCH solves via the worker beside a 25 ms grid, below) and the 600-solve probe. Step 2 (live box, sitting 8) is the open verification.

### 3. The rehearsal as a script

The wave-2 rehearsal was two kill commands: `kill -STOP <pid>; sleep 0.10; kill -CONT <pid>`. A script instead because an operator reading five log lines can miss an expectation, and the freeze is only valid if the preconditions hold. `lane_latch_rehearsal.py` checks every precondition and reports every refusal at once (the UH-3 lesson), verifies the pid is exactly one `trajectory_node` (scanning `/proc` argv, not `pgrep -f`, which matches shells), sends CONT in a `finally`, and refuses a freeze of 200 ms or more (the 250 ms SETPOINT_STALE margin). Its move is +40 mm in x over 0.5 s; peak per-leg speed through the Jacobian at z = 170 is [0.49 0.77 0.77 0.49 0.29 0.28] rps against the 0.3 rps resume tolerance, so the default is fine and the pre-flight refuses any move whose maximum is 0.3 or less. It cannot see `/leg_setpoint_echo` flatness (row 17d, from the bag) or the hand-stroke variant (row 18, manual). `tools/probes/README.md` line 71 says probes observe only; the new row documents the one commanding exception and the README rule line is left for the owner to reword.

## Fix

- **H** `motion/skills/learner.py:408` `local_model` (same `_neighbourhood`/`_local_fit` as `command` and `predict`; None on thin support); `motion/skills/memory.py:312` `Memory.local_model`; `motion/skills/executor.py:641` `BOX_FORWARD_GAIN_FLOOR`, `_command_u` (~L2088), `_box_note_once`, `_forward_clip_xy` (L2116); tests `test_box_forward_clip_*` (9) and `test_local_model_*` (2). `skill_node.py` five `local_model=` namespace lines (L3530, L3940, L4510, L4653, L4818) and `sim/skills_gate.py:777` `_MemoryLearner.local_model` added by the orchestrator. No gated file touched.
- **I1** new `motion/skills/solver_worker.py` (`SegmentSolveRequest`/`Reply`, `WorkerSegmentSolver`, `InProcessSegmentSolver`, `worker_main`, `SOLVER_UNAVAILABLE`, `SKIPPED_LATE`), new `tests/motion/test_solver_worker.py` (14) and `test_solver_worker_lateness.py` (1, `serial`).
- **I2** `trajectory_node.py`: `_default_segment_solver_factory` (L262), constructor seam before `_start_emitter`, `_solver_log` (L1151), `_warm_segment_solver` (L1263; also solves one throwaway REST through the seam so the worker is warm), `_svc_install_segment` request/deadline/refusal mapping, `_reply_outran_wire` (L4679), `solve %.1f ms` on the existing DEBUG line (L4709), `poll_health` in `_publish_status`, `close` in `on_shutdown`; `tests/ros/conftest.py` autouse `in_process_segment_solver`; 13 tests in `tests/ros/test_install_segment.py` (12 with a fake solver + 1 through the real worker process).
- **J** new `tools/probes/lane_latch_rehearsal.py`, `tests/motion/test_lane_latch_rehearsal.py` (10), one row in `tools/probes/README.md`.
- **Orchestrator** `CLAUDE.md` serial-test sentence and `run_tests.sh` rationale (third default-gate serial test), plan § 0/§ 2.5 rule text and the hop ruling, runbook rows 4/17a-18/25, memory.
- **K (docs)** this entry, `logbook/INDEX.md` row, one clause on each of C-REACH-1, I-CATCH-1 and I-CATCH-3 in `motion/skills/INVARIANTS.md`, runbook rows 10a/16a/22/closeout, the wave-2 follow-up paragraph, one plan status line.

## Verification

All 2026-10-13; scoped runs by the units in the shared main tree:
- H: `pytest -q -p no:cacheprovider tests/motion/test_skills_executor.py tests/motion/test_learner.py` (20:31) -> **306 passed in 5.62 s**; `pytest -q -p no:cacheprovider -n 2 tests/ros/test_skill_node.py tests/ros/test_possession_replay.py tests/ros/test_install_segment.py tests/ros/test_skills_plan_bench.py` (20:31) -> **364 passed**. Sim gate, `OPENBLAS_NUM_THREADS=1`, same seeds, before and after: `--learn --pattern self_toss --seeds 0 1 2` 25/0/3 x3 PASS both; `--learn --pattern columns --seeds 0 1` 25/0/25 x2 PASS both.
- Orchestrator, after the skill_node and sim-stub integration edits (~21:10): `test_skill_node.py` + `test_learner.py` + `test_skills_gate.py` -> **352 passed**.
- I1: `pytest -q -p no:cacheprovider tests/motion/test_solver_worker.py tests/motion/test_solver_worker_lateness.py` -> **15 passed in 33.9 s**. Lateness, end-of-work metric, 40 real CATCH solves, worker BLAS pinned:

| Condition | p99 (ms) | max (ms) |
|---|---|---|
| Idle (load 1-2), 4 runs | 0.31-0.75 | 0.32-0.92 |
| Loaded (load 2.5-8.7), 3 runs | 0.24-0.73 | 0.27-1.26 |
| In-process control, idle / loaded | n/a | 128-149 / 174-196 |
| Unpinned worker (not production), idle | 0.5-3.7 | 0.6-7.3 |

- I2: `pytest -q -p no:cacheprovider tests/ros/test_install_segment.py tests/ros/test_trajectory_node.py tests/ros/test_trajectory_tilt_map.py tests/ros/test_levelling_frame.py tests/ros/test_unified_cycle_levelling.py tests/ros/test_trajectory_node_console.py tests/ros/test_orchestrator_node.py tests/motion/test_solver_worker.py` (21:01, after the real-worker node test was added) -> **452 passed in 33.2 s**. The real-worker path through the node is a PERMANENT test, `test_a_fresh_throw_installs_through_the_real_worker_process` (tests/ros/test_install_segment.py, ~L1616: drops the autouse fixture, waits for READY, installs one fresh THROW through `_svc_install_segment`, 0.5 s); the earlier throwaway smoke also saw 3 real CATCH splices accepted, handler 37-40 ms for a 34 ms solve.
- J: `pytest -q -p no:cacheprovider tests/motion/test_lane_latch_rehearsal.py` -> **10 passed in 0.20 s**. The ROS/OS layer is untested offline (no rclpy in the venv).
- K (docs only): see the handoff for the plans-index and logbook test runs.
- Audit (2026-10-13 ~21:10, audit-reporter over the staged set, verdict "issues found, none blocking"): applied the fresh-REST stall fence above (tests/ros/test_install_segment.py ~L1493/1514), a READY timeout for a worker that never reports ready (`READY_TIMEOUT_S` 10 s -> the one-restart-then-DOWN budget; test at the end of tests/motion/test_solver_worker.py), a 5 ms lock acquire in `_solve` so a status-timer `poll_health` cannot refuse a real install as "busy", the stale `t_inst_s` docstrings, the `SEGMENT_OWNED` comment placement, and in the probe a SIGTERM handler around the freeze (a SIGTERM no longer leaks the STOP) with `FREEZE_MAX_MS` 200 -> 150. Skipped: the blocking warm-up on the startup timer (startup only). Re-runs after the fixes: I1 `pytest -q -p no:cacheprovider tests/motion/test_solver_worker.py tests/motion/test_solver_worker_lateness.py` -> 16 passed in 74.0 s (loaded box); I2 the 8-file scoped command (21:17) -> 455 passed in 34.0 s (the real-worker node test now retries only an `ORIGIN_TOO_LATE` refusal, up to 4 times, because a fresh THROW's 175 ms budget is real wall clock on a loaded box); J `pytest -q -p no:cacheprovider tests/motion/test_lane_latch_rehearsal.py` -> 10 passed in 0.22 s.
- Full gate: see the commit message's triple.

## Open

1. The gain floor 0.5 is unmeasured; no gain data exists on this plant.
2. A sim columns run with boxes at apex 0.95 and a biased plant would make the gate see the forward-clip rule. Not built.
3. The 3 ms allowance is gone; the open number is the loaded-box transit and the `solve`/`plan` split. Sitting 8 records it from the `solve %.1f ms` DEBUG line.
4. The worker's RSS (estimated about 80 MB) and the live node's RSS were never measured.
5. Verification step 2 on the live box: `max_emit_gap_ms` p99 < 30 ms and max < 35 ms over the run, against L2's 34.3 / 45.7.
6. The rehearsal's ROS/OS layer first runs at row 17a; the hand-stroke variant (row 18) stays manual.
7. skill_node's 43 ms solve and `go_home`/`go_to_pose`/hold solves stay in their own interpreters (design R8); the same seam can take them later.
8. `BOX-FORWARD-CLIP-UNAVAILABLE` must never appear on the live path; skill_node logs executor lines at DEBUG, so the "WARN" is a text tag only.

**Follow-up (2026-10-13 night).** Sitting 8 met the R5 gate (`2026-10-13-skill-stack-r5-sitting-8-gate-met.md`). Verification step 2 measured over 2741 windows: `max_emit_gap_ms` p99 29.94 ms (target < 30: MET overall, 32.5 inside the gate attempt) and max 47.04 ms (target < 35: NOT MET; L2 baseline 34.3 / 45.7). Solve/plan split recorded (solve p50 103.7 / p99 192.2 / max 357.1 ms); worker RSS never logged. The lane-latch rehearsal (rows 17a-20) is pending; the remaining items are rows in `plans/active/skill-stack-dress-rehearsals-and-residue.md` § 2.

> Closed 2026-10-28: wave 3 flown; forward-clip sim run tracked as JB-32, other solves as JB-31 (JB-34 survey).
