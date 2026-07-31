---
title: The full suite was 28 minutes and quietly ignored — xdist with --dist loadfile cuts it to ~8, and the ROS plugin suppression had never worked
type: optimization
date: 2026-07-31
status: resolved
phase: "Developer workflow — test-suite runtime"
files_changed:
  - pyproject.toml
  - run_tests.sh
  - sim/requirements.txt
  - tests/sim/test_hot_loop_allocation_contract.py
  - tests/sim/test_mpc_time_pathologies.py
  - tests/sim/test_mpc_static.py
  - tests/sim/test_mpc_trajectory.py
  - tests/motion/test_motor_guard_friction_ff.py
  - controller/HOT_LOOP_CONTRACT.md
  - CLAUDE.md
  - sim/JUGGLE_DEMO.md
  - tests/firmware/native/README.md
# backfilled after the commit lands
commits:
  - PENDING
subsystem:
  - sim
  - motion
tags:
  - testing
  - performance
  - docs
---

# The full suite was 28 minutes and quietly ignored — `--dist loadfile` cuts it to ~8, and the ROS plugin suppression had never worked

## Summary

The pre-commit gate (`pytest tests/ -q`) had grown to **4297 tests / 27 min 40 s**
on the Jetson. It now runs as a parallel phase (4 xdist workers, `--dist loadfile`)
plus a 7-test serial phase, via `./run_tests.sh`, in **~8 minutes** — a 3.5×
cut with no test rewritten and no coverage dropped.

A second, unrelated defect fell out of the survey: the `-p no:launch_testing` /
`-p no:launch_testing_ros` entries in `pyproject.toml` **never disabled anything**,
because those are module names and pytest's `-p no:` takes *entry-point* names
(`launch`, `launch_ros`). Both ROS launch-testing plugins had been active on every
run since the addopts block was written.

## Motivation

The suite has roughly quadrupled since May (the "full suite target < 5 minutes"
note in `tests/conftest_hypothesis.py` dates from ~1200 tests). At 28 minutes the
CLAUDE.md rule "run the full suite before every commit" stops being something
anyone actually does — the rule silently degrades into scoped subsets, and the
regressions the gate exists to catch start landing.

Timing is not evenly spread, which matters for what fixes are available:

| Directory | Time | Tests |
|---|---:|---:|
| sim | 1054 s (64 %) | 1304 |
| motion | 331 s | 871 |
| firmware | 176 s | 368 |
| ros | 84 s | 1563 |
| teensy_link | 3 s | 191 |

Five tests account for 8.2 minutes (30 %): the native-firmware fault-machine binary
(172 s), the retime corpus-optimality corpus (134 s), and three juggle
optimizer/sim end-to-end runs (88 s, 58 s, 39 s). Sum-of-per-test-time is 1648.6 s
against 1660.65 s wall-clock, so pytest overhead is ~12 s — there is no framework
waste to reclaim. The time is real work, and the only lever that does not delete
coverage is running it concurrently.

## Approach

Three changes, plus the plugin fix:

1. **`pytest-xdist` with `--dist loadfile`**, 4 workers, pinned (not `-n auto`).
2. **A `serial` marker** on the 7 tests that measure process-global resources —
   five `tracemalloc`/GC-event tests and two CasADi wall-clock solve-time budgets —
   registered in `pyproject.toml` with a normative description of what does and
   does not qualify.
3. **`./run_tests.sh`** — the blessed gate: parallel phase `-m "not serial"`,
   then serial phase `-m serial`. Both phases always run, exit codes combine, and
   extra args forward to both (so `--hypothesis-profile=ci-deep` still works).
4. **`--strict-markers`**, so a typo'd marker fails loudly instead of silently
   selecting nothing. Verified safe: the only custom marker in use is `slow`
   (2 uses), and hypothesis registers its own.

## Benchmarks

All runs 2026-07-31 on the Jetson (6 cores, 7.2 GB), in a detached git worktree at
`a7da5c5` with `temp/logs` copied in so the replay tests ran rather than skipped.

| Run | Wall-clock | Result |
|---|---:|---|
| Serial baseline, `pytest tests/ -q` | **1660.65 s (27:40)** | 4294 passed, 3 xfailed |
| Parallel, `-n 4 --dist loadfile` | **444.37 s (7:24)** | 1 failed, 4293 passed, 3 xfailed |

The single parallel failure was
`test_mpc_time_pathologies.py::TestT3bH4PostSolveAllocation` — a `tracemalloc`
allocation-budget test, i.e. exactly the class the `serial` marker now covers. The
serial tail costs **28.2 s** of the 1660 s baseline (24.0 s of allocation tests plus 4.2 s of solve-time budgets).

### Worker count × BLAS thread pinning (2×2)

`-n auto` would select 6 workers here. RAM, not cores, is the binding constraint,
so the choice was measured rather than assumed:

| BLAS threads | workers | wall | avail-mem floor | peak load | result |
|---|---:|---:|---:|---:|---|
| default | 4 | 444.37 s | (not sampled) | — | 1 failed (alloc), 4293 passed |
| default | 5 | 429.14 s | 237 MB | 8.13 | 1 failed (alloc), 4293 passed |
| pinned to 1 | 4 | **444.69 s** | **417 MB** | **6.32** | 4294 passed, 3 xfailed |
| pinned to 1 | 5 | 403.55 s | 337 MB | 6.41 | 4294 passed, 3 xfailed |

Shipped config is the bolded row. Two things came out of this that were not
obvious in advance, and both inverted an assumption I had going in:

**Thread pinning is not a speed knob here — it is a *contention* knob.** At 4
workers it is worth nothing in wall-clock (444.69 s vs 444.37 s, 0.07 %). What it
does is stop numpy/CasADi sizing their thread pools from the core count *inside
every worker*, so 4 workers stop asking for ~24 threads on 6 cores. Peak load
drops 8.13 → 6.41 and the memory floor rises ~100 MB at 5 workers. It is pinned in
`run_tests.sh` for the headroom, not the speed.

**Pinning is what makes more workers pay.** Unpinned, going 4 → 5 workers bought
only 3.4 % (444.37 → 429.14 s); its memory cost is unknown, because the unpinned
4-worker floor was never sampled. Pinned, the same step buys 9.2 % and costs 80 MB
of floor (417 → 337 MB), because the workers are no longer fighting each other's
BLAS pools.

Even so the shipped default stays at 4. The 5-worker row is a genuinely
defensible choice and is one env var away (`JB_TEST_WORKERS=5`), but this Jetson
also runs the robot — the GUI daemon, sometimes ROS2 nodes, sometimes a second
agent session (~930 MB resident across three at the time of measurement). Swap
here is **zram**, i.e. compressed RAM rather than disk, and already ~50 % used, so
over-commit degrades hard rather than gracefully. An OOM-killed xdist worker
surfaces as "node down", which is exactly the kind of confusing, non-reproducible
failure the blessed gate must not produce. 41 seconds does not buy that risk. The
measurement is recorded here so the next person can re-decide without re-running
the experiment.

Caveat on the memory figures: sampled at 10 s intervals, so the true floors sit
somewhat below the numbers above. They are comparable to each other, not exact.

## Verification

- Marker selection: `pytest -q -m serial --collect-only` (2026-07-31) selects
  exactly the intended **7 of 4297** tests, 4290 deselected.
- `--strict-markers` is safe: the full 4297 collect cleanly with it active.
- Plugin fix confirmed against ground truth with `pytest --trace-config`: both
  launch plugins register under the old config and are absent under
  `-p no:launch -p no:launch_ros`. Collected count is identical (4297) either way,
  so the fix changes failure modes, not selection.
- The two BLAS-pinned cells both reproduced the serial baseline result **exactly**
  (4294 passed, 3 xfailed), which is the evidence that pinning does not perturb
  numerical results through changed reduction orders.
- Full gate end-to-end on the shipped two-phase config (`./run_tests.sh`, run
  2026-07-31 against working tree at `971d12c`): **parallel 440 s (1 failed, 4286
  passed, 3 xfailed) | serial 32 s (7 passed) | total 472 s (7:52)**. Against the
  1660.65 s serial baseline that is **3.52×**. Test accounting reconciles exactly:
  4286 + 7 = 4293 passed, +1 failed, +3 xfailed = 4297 collected.
- The one failure, `tests/teensy_link/test_protocol_codec.py::test_constants_match_firmware_spec`
  (`assert 5 == 4` on `PROTOCOL_VERSION`), is **pre-existing and unrelated to this
  change**: a parallel session bumped the protocol to v5 in `bf1e9a5`
  ("cone PROFILE slot (UDP v5, FW8)") at 15:51 that day without updating this
  assertion. It is not touched here — it belongs to that change, and none of the
  files in this commit go near the protocol surface. Reproduced identically under
  plain serial `pytest`, so it is not a parallelism artefact.

## Outcome

The pre-commit gate goes from **27:40 to ~8 minutes** (a 3.5× cut) with no test rewritten,
no coverage dropped, and `pytest tests/ -q` still available and still equivalent.
The plugin-suppression fix removes a failure mode where one unimportable file
took down all 4297 tests.

One thing this deliberately does *not* do is make the suite cheaper — 30 % of the
runtime is still five tests, and parallelism only hides them. If the suite keeps
growing at its current rate, the next lever is splitting the largest files (which
also raises the ceiling on useful worker counts, since `loadfile` cannot balance
within a file), not adding workers.

The allocation-budget failure seen under parallel load is **non-deterministic**,
not a hard incompatibility: it failed in both unpinned parallel cells and passed
in both pinned ones. That is consistent with it being load-sensitive rather than
parallel-hostile, and it is the empirical justification for the `serial` marker
rather than an attempt to fix the test.

## Discussion

### `--dist loadfile`, not the default `--dist load` — this is the load-bearing choice

`loadfile` keeps every test in a file on a single worker. That one property is what
made this safe to adopt without auditing 4300 tests individually, because it
collapses two whole hazard classes at once:

* **Module/session-scoped fixtures** (49 of them — shared `MuJoCoPlant`s, built
  solvers) are constructed once per file on one worker, exactly as they are
  serially. Under `--dist load` the same file's tests scatter across workers and
  each worker rebuilds the fixture, which is both slower and semantically
  different for any fixture carrying state between tests.
* **Fixed-port binds.** Two files bind hard-coded localhost ports —
  `tests/motion/test_motor_guard.py` (:15555/:15556) and
  `tests/ros/test_teensy_bridge_node_setpoint.py` (:5599). Under `loadfile` they
  cannot collide, because a file's tests never straddle workers and no two files
  share a port. Under `load` they would collide nondeterministically.

Everything else was already parallel-safe by construction, which is a credit to
how the tests were written: `tests/teensy_link/conftest.py` binds ephemeral ports
with the comment "Both bind to OS-assigned ports so multiple tests can run in
parallel", `tests/sim/_zmq_test_harness.py` binds `tcp://127.0.0.1:0` and reads
back `LAST_ENDPOINT`, the ros tests are pure in-process mocks, and the file-writing
tests use `tmp_path`. The `temp/logs` references are read-only replays.

The tradeoff accepted: `loadfile` cannot balance *within* a file, so the critical
path is bounded below by the largest single file (`test_juggle_selfcatch`, 230.8 s).
At 4 workers the per-worker share is 412 s, so that bound is not currently binding —
but it becomes binding somewhere past 7 workers, and it is the reason the measured
speedup is 3.7× rather than 4×. If the suite ever needs to scale past ~6 workers,
the fix is to split the big files, not to switch to `--dist load`.

### A marker plus a wrapper script, not `-n 4` in `addopts`

Putting the parallel flags in `addopts` would have been a smaller diff and would
have made *every* invocation fast, including `pytest tests/motion/`. It was
rejected because it makes the fast path the *only* path: every scoped iteration run
would pay ~3 s of worker startup, `-v` output would interleave across workers, and
`pdb` breakpoints stop working under xdist. Iterating on one failing test is the
single most common thing anyone does with this suite, and that workflow should stay
boring. Keeping bare `pytest` serial and unmodified also means the parallel setup is
never load-bearing for correctness — if xdist ever misbehaves, `pytest tests/ -q`
is still there, still equivalent, just slower.

The marker + script split also makes the design expandable in the intended
direction: new tests are parallel by default with zero ceremony, and the only thing
a future author must learn is a single narrow rule about when to reach for
`serial`. That rule is now stated normatively in `pyproject.toml`'s marker
description and in CLAUDE.md's Critical Conventions, so the "why" travels with the
mechanism rather than living only here.

### The serial phase should make the allocation tests *less* flaky, not more

`project_hot_loop_alloc_test_flaky` records these tests as order/load-flaky under
the full suite and fine in isolation. It would be easy to read the serial phase as a
grudging carve-out for known-bad tests. It is closer to the opposite: their
measurement is only meaningful against a clean interpreter baseline, and today they
run at whatever point in a 4297-test serial ordering they happen to fall. Running
them together in a dedicated process, after the parallel phase, gives them a
*fixed, small* prefix of prior work — a more controlled environment than they have
ever had. Whether that actually reduces the observed flake rate is worth watching,
and it is the reason the marker's description is written in terms of "measures a
process-global resource" rather than "is known to be flaky": the former is a
mechanism a future author can apply to a *new* test, the latter is a label you can
only apply after being burned.

### `-p no:` takes entry-point names — a silently-ineffective-config failure

The two wrong plugin names had been in `pyproject.toml` long enough to look load
bearing, and nothing ever surfaced them: a `-p no:<name>` that matches no plugin is
not an error, it is a no-op. The five `ament_*` names in the same block are correct
(their entry points genuinely are `ament_copyright` etc.), which is precisely why
the two wrong ones blended in.

The consequence is not cosmetic. The launch plugin imports every collected module
at collection time, so **any** unimportable test file aborts the entire run as a
collection error rather than failing one file. That is not hypothetical — it is how
this investigation's first benchmark attempt died: a parallel session had a
half-applied `Teensy_code/` → `Teensy_code_platform/` rename on disk, one firmware
xref test could not find its header, and all 4297 tests refused to run.

The general class here is *configuration that fails open and silently*: a mistyped
`-p no:`, a mistyped marker (now caught by `--strict-markers`), a `--ignore` path
that no longer exists. The first two are closed. Worth remembering that pytest's
default posture for all of these is to shrug and continue.

### Enumerating the marked set from the contract, not from what happened to fail

The first pass at the marked set was selected the lazy way: the one test that
failed in the `-n 4` probe, plus the files that obviously shared its mechanism.
That is backwards, and an audit of this change caught it. The marker's own
description names **wall-clock** as a qualifying process-global measurement, so
the honest procedure is to grep for wall-clock assertions and judge each against
the contract — not to wait for a flake to nominate them.

Doing that surfaced two more: `test_mpc_static.py::TestMPCPerformance::test_solve_time`
(asserts mean CasADi solve time < 15 ms) and
`test_mpc_trajectory.py::TestTrajectoryPerformance::test_solve_time` (< 20 ms).
Both read `diag['solve_time_ms']`, i.e. real elapsed time, and both passed in all
four parallel cells — but at roughly 2.7× and 3.4× headroom against 4-way
contention, on a box that also runs the robot. They are now `serial` (+4.2 s to
the tail). `test_settle_time` in the same class looks similar and is **not**
marked: it computes `settled_idx * CONTROL_DT`, which is simulated time and is
entirely deterministic under load.

The general lesson is the one the marker description was written to encode: a
contract that says "apply this when X" is only worth having if the set is derived
*from X*. Deriving it from observed failures instead produces a set that is
correct about the past and silent about everything that has not flaked yet.

### What was deliberately left alone

* **`tests/motion/test_motor_guard.py::test_loop_timing` `return`s its pass/fail
  instead of asserting it** — its 500 Hz p99-jitter gate can therefore never fail a
  run, and has not been able to for as long as it has been written that way. It is
  a real defect and it is *not* fixed here, because fixing it means making a live
  gate out of a 10 s wall-clock jitter measurement, and under a 4-worker parallel
  phase that measurement is close to meaningless. Deciding what that test should
  assert (and whether it belongs in `serial`, or in the hardware harnesses instead)
  is its own question and should not ride along on a tooling change. Flagged here so
  it is not lost; `test_ipc_latency` in the same file deserves the same look.
* **The five expensive tests.** Parallelism hides 30 % of the runtime rather than
  removing it. Shrinking `test_native_firmware`'s 172 s binary run or the retime
  corpus is a separate, optional exercise with real coverage risk; it should not be
  bundled with a change whose whole appeal is that it deletes no coverage.

### The "mocked ROS" tests are not as hermetic as they look

The first parallel probe reported 215 failures and 77 errors, which briefly looked
like a serious parallelism problem. It was my harness bug: I injected xdist by
*replacing* `PYTHONPATH` instead of prepending to it, which dropped `/opt/ros/foxy`
off the path. The failures were all `ModuleNotFoundError` on `diagnostic_msgs`
(260) and `ament_index_python` (32).

The lesson worth keeping is the one about the tests, not the harness:
`tests/ros/conftest.py` mocks `rclpy` and the `jugglebot_interfaces` messages, and
the suite is described as running "on Windows without a ROS2 installation" — but
`diagnostic_msgs` and `ament_index_python` are *not* mocked, and are imported by
`teensy_bridge_node`, `trajectory_node`, `test_motion_bridge_node` and
`test_orchestrator_node`. Those tests depend on the real ROS install on this box.
That is fine on the Jetson and invisible until something perturbs the environment,
but it means the "mocked ROS2, runs anywhere" claim in the conftest docstring is
now only partly true.

## Related

- `project_hot_loop_alloc_test_flaky` (memory) — the prior record of the
  allocation-budget tests' order/load sensitivity.
- `tests/conftest_hypothesis.py` — the "< 5 minutes" target in its docstring is
  stale as of this entry; the ci-fast full-suite figure is now ~8 min parallel.
