# Motion Planner Optimization Analysis

**Date:** 2026-03-02
**Status:** Analysis complete, implementation pending

---

## Context

The motion planner plan (Phases 5 and 7) identifies "loop timing optimization" as a suggested next step. Analysis reveals this label covers two fundamentally different performance problems with different severity levels and different production implications.

---

## Problem 1: Per-Cycle Loop Rate (~290 Hz vs 500 Hz target)

### Root cause

**6 redundant `compute_jacobian()` calls per cycle at the same pose.**

During one trajectory-mode control cycle, the call graph traces:

| Call site | Jacobians | Location |
|---|---|---|
| `twist_to_leg_velocities()` | 1 | `ik_solver.py:227` |
| `compute_full_feedforward_torques()` → direct J | 1 | `dynamics.py:311` |
| └→ `accel_to_leg_accels()` → direct J | 1 | `ik_solver.py:304` |
| └→ `compute_jacobian_dot()` → J at pose | 1 | `ik_solver.py:274` |
| └→ `compute_jacobian_dot()` → J at perturbed pose | 1 | `ik_solver.py:276` |
| `compute_condition_number()` → J + SVD | 1 | `workspace.py:192` |
| **Total** | **6** | |

5 of the 6 are at the identical (pos, rot). The 6th (the perturbed pose in `compute_jacobian_dot`) is a genuine second evaluation. A simple cache or pass-through of J would reduce this to 2 Jacobian evaluations + 1 SVD.

### Production impact: LOW-TO-MODERATE

The position-control architecture provides a large buffer. The ODrive holds position at 8 kHz between Python updates. At 290 Hz (3.5 ms/cycle), a trajectory with peak velocity of 20 rev/s advances only ~0.07 rev between updates — comfortably within the ODrive's correction bandwidth. All Phase 4–7 hardware tests passed at this rate with tracking errors well within thresholds.

This becomes relevant at ball-catching speeds if peak velocities exceed ~30 rev/s, but even then the ODrive's PID absorbs most of the feedforward staleness.

### Recommendation

Address opportunistically (low-hanging fruit: cache J per pose, pass J into `compute_condition_number`). Not a production blocker.

---

## Problem 2: Trajectory-Planning Stalls (250 ms – 2 s blocking)

**This is the critical production issue.**

### Root cause

`submit_dynamic_target()` and `_plan_return_to_home()` run expensive feasibility checks **inline on the control loop thread**, blocking all motor command output for their duration.

### Where the stall occurs

The control loop structure (`control_loop.py:245–271`):
```
while running:
    _process_ipc()       ← dynamic target IPC arrives here
        └→ _on_dynamic_target()
            └→ TrajectoryManager.submit_dynamic_target()
                └→ check_feasibility(n=50)     ← 250ms STALL
                └→ realtime_restamp             ← band-aid
    _check_heartbeat()
    _compute()           ← NO motor commands during stall
    _publish_telemetry()
    sleep(remainder)
```

For return-to-home, the stall happens inside `_compute()` → `TrajectoryManager.evaluate()` → `_plan_return_to_home()` → `find_min_feasible_duration()` (~2 seconds on Jetson).

### What makes feasibility checking expensive

Each `check_feasibility` sample evaluates the full kinematics + dynamics pipeline. Per sample:

| Operation | Jacobian evals | Location |
|---|---|---|
| `pose_to_leg_lengths()` | 0 | `ik_solver.py:152` |
| `twist_to_leg_velocities()` | 1 | `ik_solver.py:227` |
| `accel_to_leg_accels()` | 3 (1 direct + 2 in J_dot) | `ik_solver.py:304–306` |
| `compute_condition_number()` | 1 + SVD | `workspace.py:192` |
| `compute_full_feedforward_torques()` | 4 (1 direct + 3 via accel_to_leg_accels) + solve | `dynamics.py:311,328` |
| `evaluate_jerk()` | 0 (polynomial only) | — |
| **Total per sample** | **~9** | |

Several of these calls redundantly compute J at the same pose within a single sample — the same cache problem as Problem 1, but amplified 50× across the sample loop.

With 50 samples: **~450 Jacobian evaluations per feasibility check**.

For `find_min_feasible_duration()`: 9 feasibility checks (1 initial + 8 bisections) → **~4,050 Jacobian evaluations**, taking ~2 seconds on Jetson.

### Production impact: CRITICAL

In ball-catching mode, the ball predictor will continuously refine its target as the ball approaches. Each refinement calls `submit_dynamic_target()` → 250ms stall. At 2–10 Hz prediction updates (as in the DT4 hardware test at 2 Hz), the control loop is stalled **50–100% of the time**. During each stall:

- No motor position/velocity/torque commands are sent
- The ODrive holds its last commanded position (safe, but no trajectory tracking)
- The trajectory clock advances, creating a time gap that `realtime_restamp` patches but cannot eliminate
- No telemetry is published
- IPC heartbeat timer accumulates (risk of false heartbeat-loss E-stop if stall + prior latency approaches the timeout)

The `realtime_restamp` workaround shifts `t_start` forward to account for the blocked time, but this means:
1. The first command after the stall jumps to a different point on the polynomial than the physical robot state
2. Any nonzero velocity/acceleration at the splice creates a feedforward transient
3. The platform was physically stationary during the gap, so the "restamp" pretends the gap didn't happen — but the ODrive's encoder position didn't change, creating a step in tracking error on the first post-stall cycle

For `_plan_return_to_home()` (~2s stall): the platform holds at the outbound trajectory's end pose for 2 seconds with no position updates. Safe (ODrive holds position), but produces a visible 2-second pause before the return motion begins.

### Quick win: skip unnecessary torque computation

`check_feasibility()` calls `compute_full_feedforward_torques()` per sample — the most expensive per-sample operation (~4 Jacobians + linear solve). But when `torque_limit_Nm` is `None` (the default), the result is never checked against any limit. The call in `submit_dynamic_target()` passes no `torque_limit_Nm`, so these computations are entirely wasted.

**Skipping the torque computation when no limit is set would cut feasibility check time by ~40% with a one-line change.**

---

## Resolution Strategy: Async Feasibility Pipeline

### Architecture

Decouple the expensive feasibility computation from the control loop thread. The control loop should never block on a feasibility check.

```
1. Dynamic target arrives via IPC
2. Control loop immediately queues the target for async feasibility checking
3. Control loop continues its normal cycle (sending motor commands for current trajectory)
4. A background thread runs the feasibility check
5. When the check completes, the result is atomically made available
6. The control loop picks up the new trajectory on its next cycle
```

### A. Background feasibility thread in `TrajectoryManager`

Add a single background thread managed by `TrajectoryManager` that runs feasibility checks. The thread communicates with the main loop via a thread-safe result slot.

**Key data structures:**

```python
# In TrajectoryManager.__init__:
self._pending_check: threading.Event       # signals background thread
self._pending_target: dict | None          # target params (protected by lock)
self._check_result: tuple | None           # (traj, accepted, pending_return)
self._check_lock: threading.Lock()         # guards _pending_target and _check_result
self._bg_thread: threading.Thread          # daemon, long-running
```

**Background thread lifecycle:**
- Starts as a daemon thread when `TrajectoryManager` is created
- Blocks on `_pending_check.wait()` when idle (zero CPU cost)
- When woken: reads `_pending_target`, runs `check_feasibility()`, writes `_check_result`
- Returns to waiting

**Main loop integration (`_on_dynamic_target`):**
```python
def _on_dynamic_target(self, msg):
    target_pos = np.array(msg['target_pos'])
    target_quat = np.array(msg['target_quat'])
    target_vel = np.array(msg['target_vel'])
    arrival_time = msg['arrival_time']
    t_now = time.perf_counter()

    # Queue for background check (non-blocking)
    self._traj_manager.request_dynamic_target(
        target_pos, target_quat, target_vel, arrival_time, t_now)
```

**Main loop integration (`_compute` or top of cycle):**
```python
# Check if background feasibility result is ready
result = self._traj_manager.poll_pending_result()
if result is not None:
    traj, accepted, pending_return = result
    if accepted:
        self._traj_manager.commit_trajectory(traj, pending_return)
```

### B. Handling the timing gap — re-sample on commit

With async checking, the start state is sampled when the target arrives (`t_request`) but the trajectory is committed ~250ms later (`t_commit`). During that 250ms, the robot has continued moving along its current trajectory, making the sampled start state stale.

**Solution:** When the background check completes with a "feasible" result, the commit step should:

1. Re-sample the current state at `t_commit = perf_counter()`
2. Re-create the trajectory from `(current_state_at_t_commit → target)` with adjusted duration `arrival_time - t_commit`
3. **Skip the full feasibility re-check** — the dynamics haven't changed dramatically in 250ms, so the initial check's result is a valid proxy
4. Submit the new trajectory with `t_start = t_commit`

This eliminates both the timing gap and the stale-start-state problem. The only risk is that the re-created trajectory is slightly different from the one that was checked — but since the start state only moved slightly along the previous trajectory (250ms of motion), the new trajectory will be very similar and the feasibility result is conservative.

If this "trust the previous check" approach is uncomfortable, a lightweight fast-check could be added: evaluate only the start and end samples (2 samples instead of 50) as a sanity gate.

### C. Return-to-home pre-computation

`_plan_return_to_home()` currently blocks for ~2s inside `evaluate()` when the outbound trajectory completes. This can be eliminated:

**Key insight:** The outbound trajectory's end state is known from the moment the trajectory is submitted. The return-to-home planning can begin immediately in the background, well before the outbound trajectory completes.

**Implementation:**
- When `submit()` accepts a trajectory with `_pending_return = True`, immediately queue a background `find_min_feasible_duration()` for the return path
- The background thread computes the return trajectory and stores it as `_precomputed_return`
- When `evaluate()` detects trajectory completion and `_pending_return is True`, it checks for `_precomputed_return`:
  - If available: submit immediately (zero stall)
  - If not yet ready (background still computing): hold at end pose and poll on subsequent cycles
  - If failed: fall through to COMPLETE as currently done

**Timing alignment:**
- The precomputed return trajectory's `t_start` must be set when it's actually committed (not when it was pre-computed). Use the existing `_dc_replace(traj, t_start=t_now)` pattern.

### D. Thread safety considerations

The control loop is single-threaded today. Adding a background thread introduces shared-state concerns:

**What the background thread reads:**
- `self.geom` — immutable after construction (safe)
- `self.dynamics_params` — immutable after construction (safe)
- `self._gravity_correction` — set by main loop, read by background. Changes rarely; accept staleness or protect with lock
- Current trajectory state (for `_get_current_state()`) — **must be sampled on the main thread** before queuing, not read by the background thread

**What the background thread writes:**
- `_check_result` — written once after check completes, read once by main thread. Protected by lock.

**Proposed synchronization:**
- One `threading.Lock` guarding `_pending_target` and `_check_result`
- One `threading.Event` to wake the background thread
- A monotonic generation counter: each new target request increments a counter. The background thread records the counter when it starts a check. On commit, the main thread only accepts results whose generation matches the current counter. If a newer target was requested (or a cancel/E-stop occurred), the stale result is silently discarded.

### E. Complementary feasibility check speedups

These reduce the background thread's wall-clock time, which reduces the staleness of the start-state re-sample:

1. **Skip torque computation when no torque limit is set** — eliminates ~40% of per-sample cost (one-line guard)
2. **Early exit on first violation** — add `if violations: break` to the sample loop. Rejection becomes nearly instant
3. **Reduce redundant Jacobians within a sample** — compute J once per sample and pass it through (~60% per-sample cost reduction)
4. **Reduce `n_feas_samples` for `find_min_feasible_duration`** — 20 instead of 50 per bisection (~60% binary search speedup)

---

## Summary of Recommendations

### Must-do for production (Problem 2):
1. **Async feasibility pipeline** — move `check_feasibility()` to a background thread, commit results atomically on the main loop
2. **Pre-compute return-to-home** — start planning the return as soon as the outbound trajectory is submitted, not when it completes
3. **Skip unnecessary torque computation in feasibility** — the default call doesn't use a torque limit, so `compute_full_feedforward_torques` is called 50× for nothing

### Should-do (Problem 1, low effort):
4. **Cache Jacobian per (pos, rot)** within a single cycle — reduces 6 Jacobian calls to 2
5. **Pass J into `compute_condition_number`** — trivial signature change

### Nice-to-have:
6. **Early exit in feasibility checker** — instant rejection of infeasible trajectories
7. **Reduce `n_feas_samples` in binary search** — 20 instead of 50
8. **Analytical J_dot** — eliminates the perturbed-pose Jacobian entirely

---

## Verification

### Offline tests (completed)

All 64 offline tests pass (up from 56, added 8 async pipeline tests):

| Suite | Count | Status |
|---|---|---|
| test_kinematics | 6 | PASS |
| test_control_loop | 3 | PASS |
| test_dynamics | 14 | PASS |
| test_trajectory | 7 | PASS |
| test_dynamic_target | 14 | PASS |
| test_hardening | 12 | PASS |
| **test_async_pipeline** | **8** | **PASS** |

Key results from `test_async_pipeline`:
- `request_dynamic_target()` completes in **0.12ms** (was 250ms blocking)
- Early exit gives **23x** speedup on infeasible rejection (2.3ms vs 52.6ms)
- Torque skip gives **1.9x** speedup when no torque limit set
- Pre-computed return trajectory available before outbound trajectory completes
- Generation-based supersession correctly discards stale results
- Cancel atomically clears all pending async state

---

### Hardware test protocol (Problem 2 validation)

**Goal:** Confirm that the async feasibility pipeline eliminates control loop stalls without regressing tracking accuracy.

**Prerequisites:**
- All Phase 4-7 tests previously PASS at 100% speed
- Robot fully assembled, platform free-standing, legs homed
- Run from `tools/dynamic_target_test.py` on Jetson

#### AP1: Regression — Re-run DT1–DT5

**Purpose:** Verify no regression from the async pipeline changes.

**Procedure:**
```
python3 tools/dynamic_target_test.py --home --test all --speed-scale 1.0
```

**Pass criteria (unchanged from Phase 7):**
- DT1 (static target): tracking error < 3.0 mm
- DT2 (nonzero velocity + return): tracking error < 3.0 mm, auto-return completes
- DT3 (mid-motion replan): tracking error < 3.0 mm, splice smooth
- DT4 (rapid targets at 2 Hz): ≥ 8/10 targets accepted, tracking error < 3.0 mm
- DT5 (infeasible target): current trajectory undisturbed

**What to watch:** If a test FAILs that previously PASSed, the async pipeline introduced a regression. The most likely failure mode is a timing issue in `commit_async_trajectory()` where the re-sampled start state diverges from the actual robot position.

#### AP2: Loop stall elimination — DT4 timing instrumentation

**Purpose:** Prove that the control loop no longer stalls during `submit_dynamic_target()`.

**Procedure:**
1. Run DT4 (rapid target updates at 2 Hz) with `--speed-scale 1.0`
2. Observe the periodic timing log output during the test
3. Record the `p99` and `max` loop cycle times from the log

**Pass criteria:**
- **p99 loop cycle time < 5 ms** (at 500 Hz, target is 2 ms)
- **max loop cycle time < 20 ms** (no single cycle exceeds 10x target)
- No startup spike > 50 ms after the first target (the old code showed ~250 ms spikes on every target)

**What to watch:** Compare against the pre-optimization baseline. Before this fix, DT4 showed ~250ms spikes on every target submission. After, the control loop should maintain near-constant 2ms cycle time because feasibility checking runs on the background thread.

**Manual verification:** Add temporary `logger.info` timestamps around `_poll_async_result()` in `control_loop.py` to measure the time from target IPC arrival to trajectory commit. Expected: < 10 ms commit latency (async poll + re-create trajectory), with the background feasibility check running in parallel.

#### AP3: Return-to-home stall elimination

**Purpose:** Confirm that the ~2-second stall at the EXECUTING→RETURNING transition is eliminated.

**Procedure:**
1. Run DT2 (nonzero velocity + auto-return) with `--speed-scale 1.0`
2. Visually observe the platform as the outbound trajectory completes
3. Record loop timing logs at the transition point

**Pass criteria:**
- **No visible pause** between outbound trajectory completion and return motion start
- The return trajectory begins within **1 control cycle** of the outbound completion (i.e., within 2-4 ms), because the return was pre-computed while the outbound trajectory was still executing
- Loop timing max cycle time at the transition < 10 ms

**What to watch:** Before this fix, the platform visibly paused for ~2 seconds at the target pose before beginning the return-to-home motion. After, the return should begin immediately because `_bg_compute_return()` pre-computes the return trajectory in the background as soon as the outbound trajectory is committed.

**Fallback behavior:** If the return precompute hasn't finished by the time the outbound trajectory completes (extremely unlikely — the outbound trajectory typically takes 1-3 seconds, and the return precompute takes ~0.5 seconds on Jetson), the platform will hold at the end pose for a few cycles until the precompute completes, then smoothly transition. This is safe (ODrive holds position) but suboptimal.

#### AP4: Stress test — rapid 5 Hz target stream

**Purpose:** Push the async pipeline harder than DT4's 2 Hz to verify it handles target supersession correctly under load.

**Procedure:**
1. Modify DT4 temporarily to use a **5 Hz** target stream (200 ms between targets) instead of 2 Hz
2. Run with `--speed-scale 0.5` (moderate speed for safety)
3. Record loop timing and count of accepted/rejected targets

**Pass criteria:**
- Loop p99 cycle time < 5 ms (same as AP2)
- Most targets are superseded (background check takes ~50 ms, new target arrives every 200 ms) — the async pipeline should accept the *latest* target and discard stale results
- Platform motion remains smooth with no visible jerks or pauses
- At least the final target in each burst is accepted and tracked

**What to watch:** This test validates the generation-based supersession mechanism. With 200 ms between targets and ~50 ms feasibility check time, most checks should complete in time. But if a target arrives while the previous check is still running, the stale result should be silently discarded.

#### AP5: E-stop during async check

**Purpose:** Verify that an E-stop during a pending async feasibility check is handled correctly.

**Procedure:**
1. Start a DT2 or DT3 trajectory
2. While the platform is moving, send a rapid sequence of dynamic targets (to keep the background thread busy)
3. Immediately trigger an E-stop (Ctrl-C or external signal)
4. Verify the platform stops safely and no stale trajectory is committed after the E-stop

**Pass criteria:**
- Platform immediately stops (legs go to IDLE)
- No motor commands sent after E-stop
- After E-stop, `TrajectoryManager.state` is IDLE
- No stale async result is committed after the E-stop (cancel() clears pending state)

**What to watch:** The `cancel()` method increments the generation counter, so any in-flight background result will be discarded when `commit_async_trajectory()` checks the generation. Verify this by inspecting the log output after the E-stop — there should be no "Async trajectory committed" message after the E-stop.

---

### Summary of pass criteria

| Test | Critical metric | Threshold |
|---|---|---|
| AP1 | DT1-DT5 tracking error | < 3.0 mm (same as Phase 7) |
| AP2 | p99 loop cycle time during DT4 | < 5 ms |
| AP2 | Max loop cycle time during DT4 | < 20 ms |
| AP3 | Return-to-home transition delay | < 10 ms (1-2 cycles) |
| AP4 | p99 loop cycle time at 5 Hz | < 5 ms |
| AP5 | Post-E-stop stale commits | 0 |

---

## Problem 3: GIL Contention in Async Pipeline (2026-03-12)

**The async pipeline does not provide true parallelism.** Python's GIL means CPU-bound numpy work in the background thread blocks the main control loop.

### Root cause

`threading.Thread` shares the GIL with the main thread. When the main loop calls `time.sleep()` for rate control, it releases the GIL. The background thread immediately acquires it and begins numpy-heavy computation (`check_feasibility`, `find_min_feasible_duration`). When the sleep expires, the main thread attempts to reacquire the GIL but **cannot** — the background thread holds it through numpy operations, only releasing at Python bytecode boundaries between numpy calls.

The result: the "non-blocking" async pipeline blocks the main loop for the duration of each background computation.

### Evidence: DT2 hardware test log (2026-03-11)

The DT2 test runs a 500 Hz control loop sending motor commands every 2ms. With the async pipeline, we expect uninterrupted command output. Instead:

| Phase | Samples | Duration | Loop dt | What happened |
|---|---|---|---|---|
| Sample 0→1 | 1 | 209 ms | 209 ms | `_bg_check_feasibility()` held GIL — initial feasibility check |
| Sample 1→2 | 1 | 1193 ms | 1193 ms | `_bg_compute_return()` → `find_min_feasible_duration()` held GIL — entire outbound trajectory missed |
| Samples 2–94 | 93 | 376 ms | ~4 ms | Normal operation (no background work) |

**Total: 95 samples in 1.778s** (expected ~889 at 500 Hz). The loop was blocked for 1.4 of 1.8 seconds.

### Why 18.6mm tracking error

Timeline:

1. **t=0.000s** — Target queued. `request_dynamic_target()` creates trajectory, queues background feasibility check. Motor command: hold at home (Z=170mm).
2. **t=0.000–0.209s** — GIL blocked by `_bg_check_feasibility()`. Zero motor commands sent.
3. **t=0.209s** — Feasibility result polled. `commit_async_trajectory()` commits outbound trajectory (1.0s duration from t=0.209). `_start_return_precompute()` queues `find_min_feasible_duration()` on background thread.
4. **t=0.209–1.402s** — GIL blocked by `_bg_compute_return()` → `find_min_feasible_duration()` (8 bisections × 50 samples). **Zero motor commands sent for entire 1.2 seconds. The outbound trajectory executes in software only — motors never move.**
5. **t=1.402s** — GIL released. `evaluate(t=1.402)` finds outbound complete (1.402 > 0.209 + 1.0). Precomputed return is ready. Code creates return trajectory starting at Z=190mm. But **motors are still at Z=170mm** (home). Result: **18.6mm commanded position step**.
6. **t=1.402s+** — Loop runs normally at ~4ms. Return trajectory slowly moves from Z=190→170, but motors chase from Z=170 upward first.

### Why the original analysis missed this

The Problem 2 analysis (above) correctly identified that moving computation to a background thread prevents the main thread from *synchronously blocking* on a function call. However, it assumed `threading.Thread` provides concurrent execution. In CPython, the GIL serializes all Python bytecode execution. The background thread's numpy work (which mixes Python-level loops with numpy C extensions) holds the GIL for extended periods, preventing the main thread from running even though the main thread isn't blocked on any explicit synchronization primitive.

The distinction: the async pipeline eliminates **synchronous blocking** (the main thread no longer calls `check_feasibility()` directly), but it does not eliminate **GIL contention** (the background thread's CPU-bound work still prevents the main thread from executing).

---

## Bug: C2 Discontinuity in Return Trajectory Splice

### Location

`trajectory.py` lines 1189–1198 (inside `TrajectoryManager.evaluate()`)

### Description

When the outbound trajectory completes and the precomputed return trajectory is ready, the code **discards** the precomputed trajectory and creates a new one from rest:

```python
ret_traj = create_trajectory(
    start_pose=end_pose,
    start_twist=np.zeros(6),      # ← BUG: should be outbound's end velocity
    start_accel=np.zeros(6),
    end_pose=self._home_pose,
    ...
    duration=precomputed.duration,  # ← only this value is reused
    t_start=t,
)
```

The comment at lines 1183–1188 explains: *"if we held at end_pose for several cycles waiting for precomputation, the platform is now stationary — not at the outbound's end velocity."*

This comment is **wrong for the normal case**. When the precomputed return is ready on the first cycle after outbound ends (which is the expected outcome — precomputation starts as soon as the outbound is committed, well before it finishes), the platform was just commanded with the outbound's end velocity on the previous cycle. Starting the return from rest creates a velocity discontinuity.

The comment is only correct for the **fallback path** (lines 1207–1214) where the precomputed return wasn't ready and the platform held at end_pose with zero velocity for multiple cycles. In that case, starting from rest is appropriate because the platform has physically come to rest.

### Correct behavior

Two cases need distinct handling:

1. **Precomputed ready immediately** (first cycle after `t >= t_end`): Use the precomputed trajectory directly — it was planned with the outbound's actual end velocity for C2 continuity. Only restamp `t_start`:
   ```python
   ret_traj = _dc_replace(precomputed, t_start=t)
   ```

2. **Precomputed ready after holding** (platform held at end_pose for multiple cycles): The platform is now at rest. A new rest-to-rest trajectory at the precomputed duration is needed (the current behavior).

These cases can be distinguished with a flag (e.g., `_held_at_end`) set in the fallback hold path.

---

## Dead Code: `_plan_return_to_home()`

`_plan_return_to_home()` (lines 1072–1135) is the original synchronous return planner. It is **never called** anywhere in the codebase — fully replaced by the async path (`_start_return_precompute()` → `_bg_compute_return()`). Should be deleted.

---

## Proposed Solutions

### A. Targeted fixes (smallest change)

1. **Replace `find_min_feasible_duration()` in `_bg_compute_return`** with a fixed generous duration (2.0s) + single `check_feasibility()`. Reduces GIL hold from ~1200ms to ~30ms. Rest-to-rest over 80mm max displacement at 2.0s gives peak velocity ~40mm/s and peak accel ~53mm/s² — well within limits. Single feasibility check catches edge cases. Fallback to 3.0s if 2.0s infeasible.

2. **Add `yield_interval` to `check_feasibility()`** — call `time.sleep(0)` every N samples to periodically release the GIL. Breaks the ~200ms initial feasibility check into ~40ms chunks, giving the main thread opportunities to run between chunks. Backward-compatible (default `yield_interval=0` means no yielding).

3. **Fix return trajectory splice** — use the precomputed trajectory directly (restamp `t_start`) when ready on the first cycle. Track `_held_at_end` flag for the fallback case where holding occurred.

4. **Delete `_plan_return_to_home()`** — dead code.

### B. Trajectory queue (architectural improvement)

Refactor `TrajectoryManager` to maintain a queue of trajectories whose boundary conditions chain together:

- Each trajectory's start state must match the previous trajectory's end state (enforced at append time).
- When an outbound trajectory with nonzero end velocity is committed, immediately append the precomputed return to the queue.
- `evaluate()` walks the queue: when the current trajectory ends, advance to the next. No special EXECUTING/RETURNING state distinction needed.
- C2 continuity is guaranteed by construction — no per-case splice logic.
- Mid-motion replanning replaces the queue from the current point forward.
- Bigger refactor but eliminates entire classes of splice bugs permanently.

### C. Move to multiprocessing (GIL bypass)

Use `multiprocessing.Process` instead of `threading.Thread` for the background feasibility worker:

- True parallelism — numpy computation runs on a separate CPU core with its own GIL.
- Zero GIL contention regardless of computation duration.
- Requires serializing trajectory data between processes (pickle or shared memory).
- More complex IPC (pipes/queues instead of shared objects + lock).
- Fundamentally solves the GIL problem for all future background computation.
- Jetson has 6 ARM cores — ample capacity for a dedicated feasibility process.

### Recommendation

Start with **A** (targeted fixes) to get DT2/DT3 passing. Then evaluate **B** (trajectory queue) as a follow-up architectural improvement — it's the cleanest long-term solution. **C** (multiprocessing) is the nuclear option if GIL contention reappears with heavier future workloads (e.g., ball predictor integration).
