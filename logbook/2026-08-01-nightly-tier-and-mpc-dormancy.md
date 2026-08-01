---
title: "Refactor Phases 2+3 — nightly runner armed, test tiering, MPC dormancy"
type: refactor
date: 2026-08-01
status: resolved
phase: "refactor-2026-07 Phases 2 & 3"
related_plan: refactor-2026-07.md
files_changed:
  - run_tests.sh
  - tools/nightly_suite.sh
  - tools/systemd/jugglebot-nightly.service
  - tools/systemd/jugglebot-nightly.timer
  - tools/systemd/README.md
  - pyproject.toml
  - CLAUDE.md
  - plans/active/refactor-2026-07.md
  - controller/HOT_LOOP_CONTRACT.md
  - controller/DIAG_SCHEMA_CONTRACT.md
  - controller/REFERENCE_LAYER_CONTRACT.md
  - sim/JUGGLE_DEMO.md
  - docs/motion_planner/safety.md
  - docs/motion_planner/control_loop.md
  - ros_ws/docs/safety.md
  - ros_ws/docs/control_modes.md
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/sim/test_juggle_selfcatch.py
  - tests/sim/test_juggle_selfcatch_nightly.py
  - tests/sim/test_demo_juggle_optimizer.py
  - tests/sim/test_demo_juggle_planner.py
  - tests/sim/test_demo_juggle_sim.py
  - tests/sim/test_mpc_static.py
  - tests/sim/test_mpc_dynamic.py
  - tests/sim/test_mpc_trajectory.py
  - tests/sim/test_mpc_runner.py
  - tests/sim/test_mpc_input_fuzz.py
  - tests/sim/test_mpc_adversarial_sequences.py
  - tests/sim/test_mpc_time_pathologies.py
  - tests/sim/test_solver_failures.py
  - tests/sim/test_catch_optimizer.py
  - tests/sim/test_diag_schema_fuzz.py
  - tests/sim/test_zmq_target.py
  - tests/sim/test_zmq_corruption.py
  - tests/sim/test_hardware_plant_failure_paths.py
  - tests/sim/test_hardware_plant_deadband.py
  - tests/sim/test_hot_loop_allocation_contract.py
  - tests/motion/test_retime.py
  - logbook/INDEX.md
subsystem:
  - testing
  - ros
  - docs
tags:
  - process
  - testing
  - mpc
---

# Refactor Phases 2+3 — nightly runner, tiering, MPC dormancy

## What changed and why

Phases 2 and 3 of `plans/active/refactor-2026-07.md`, landed together because
the plan's hard rule is that **the runner and the first demotion ship in the
same commit** — without a live runner, `nightly` is a delete button.

1. **Nightly runner armed.** `tools/nightly_suite.sh` runs
   `./run_tests.sh --full --hypothesis-profile=ci-deep` and writes
   `temp/reports/nightly/{YYYY-MM-DD.md, -junit/, .log}` + a `latest.md` symlink
   + a one-line `status` file (`GREEN|RED <counts> <iso-date>`), pruning
   date-named artifacts older than 30 days. Armed as a **user systemd timer**
   (`jugglebot-nightly.timer`, `OnCalendar=04:00`, `Persistent=true`,
   `RandomizedDelaySec=300`, `Linger=yes`, units checked in at `tools/systemd/`
   so a Jetson rebuild can re-arm). `Persistent` matters: a night the box is off
   must fire on next boot, or a demotion on a powered-down Jetson is a silent
   delete. Delivery is channel 2 (owner's choice): a CLAUDE.md session-start
   rule reads `status` and surfaces a RED. Counts come from the junit XML, not
   the pytest summary line — xdist interleaves output and a crashed worker can
   eat the summary entirely. **A live-session guard** rides with it: `Persistent`
   also means a missed run fires at the *next boot*, which on this box is often
   the minute an operator powers up for a sitting, and the unit's `Nice=10` /
   `IOSchedulingClass=idle` bound CPU and IO but not memory (run_tests.sh's own
   header records a ~417 MB available-memory floor at 4 workers on 7.3 GB with
   zram). An OOM-killed pytest worker is a confusing test failure; an OOM-killed
   ROS node mid-sitting is a robot event. So the runner *waits* for the box
   (2 h budget, 2 min poll) rather than competing with it, and on expiry writes
   `DEFERRED` — never GREEN, never silence, and never a skip, which would be the
   delete button `nightly` exists to avoid.
2. **`run_tests.sh` gained tiers.** Default gate is
   `-m "not serial and not nightly"` / `-m "serial and not nightly"`; `--full`
   is `-m "not serial"` / `-m serial`. The AND-composition is load-bearing: a
   bare `not nightly` would drop serial+nightly tests out of both phases while
   the gate still printed PASS. Also: the native firmware g++ build now runs
   *before* phase 1 (hash-cached; otherwise a cold ~170 s compile serializes
   into one xdist worker and becomes the critical path), `JB_JUNIT_DIR` gives
   each phase its own junit file, and the zero-serial-tests guard was reworked.
   That guard was widened from `$# -eq 0` to "no deselecting flag was passed" —
   the old form meant every nightly run (which passes `--hypothesis-profile`)
   silently opted out of the guard that exists to catch exactly this commit's
   kind of change — and then *split*, because the widening created a new trap:
   the deferred `motor_guard` demotion would empty the default serial phase and
   hard-fail every commit on the branch. It now re-checks `-m serial` and
   distinguishes "no serial tests ANYWHERE" (marker renamed/dropped — hard fail,
   the allocation contracts would never run again) from "some, all nightly" (a
   legitimate state — print a note and continue).
3. **Demotions (content boundary, never runtime).** `test_juggle_selfcatch.py`
   split: the MAKE acceptance capability
   (`test_oscillation_kinematic_release_sustains`, seed 0), the composition
   smoke and the motion-quality test stay per-commit; the two BREAK
   characterizations, their loop-gain signatures and the three 6-seed sweeps
   moved to `test_juggle_selfcatch_nightly.py`. `test_demo_juggle_*` (research
   demo) and the 15-file MPC battery got `pytestmark = pytest.mark.nightly`.
   `tests/motion/test_motor_guard*.py` were **deferred** — another session had
   them uncommitted in the shared tree.
4. **MPC dormancy (Phase 3).** `jugglebot_launch.py` no longer starts the
   `motor_guard` ExecuteProcess or the `motion_bridge_node` Node. Both were
   already driving nothing: the MVP leg path is
   `trajectory_node -> :5557 -> teensy_bridge_node -> can-bridge Teensy` (which
   does the 500 Hz interpolation), `motor_guard`'s :5556 output goes unconsumed,
   and `leg_lengths_topic`'s only consumer (`can_node`) was deleted in the
   2026-07-06 SocketCAN decommission. Node source, entry points and tests all
   stay — revival is re-adding two launch entries (**both**: `HardwarePlant.enable()`
   blocks on motor-feedback telemetry from the guard's :5556 and the guard is fed
   by `motion_bridge_node`, so `run_mpc.py` cannot come back on one alone — it
   fails closed with a `RuntimeError` before publishing anything). **One
   operator-visible consequence, accepted:** `motion_bridge_node` was also the
   sole publisher of `motion/diagnostics` and `motion/tracking_error`, and the
   GUI subscribes to the former (`gui/js/main.js:299`). Its 3 s timeout now fires
   permanently, so the Motion panel badge sits at `DISABLED` with a blank
   trajectory label — honest (the MPC motion chain *is* disabled) and it degrades
   to a badge, never a false ERR. Both topics stay in the rosbag record list and
   record empty, like `leg_lengths_topic`. This is precisely the dependency the
   2026-07-11 GUI work flagged as the last one on that node
   (`logbook/2026-07-11-gui-leg-setpoint-echo-poscmd.md`) going dark; a GUI-side
   rewording for the MVP topology is owed and out of scope here. Worth stating
   accurately because `tests/ros/test_launch_nodes.py` is a tripwire for exactly
   this class ("a node deleted from the production launch while its downstream
   consumers stay wired", the 2026-07-06 `catch_correlation_node` incident) — it
   still passes (`pytest tests/ros/test_launch_nodes.py -q`, run 2026-08-01:
   **2 passed in 0.03 s**) only because its `CONSUMER_WIRED_NODES` list contains
   just `catch_correlation_node`. It did not catch this because nobody added
   `motion_bridge_node` to that list, not because nothing was affected.
   Docs re-framed: CLAUDE.md now
   names the **Teensy-side `MAX_DEVIATION` guard** as the leg-path safety
   authority and calls the motor_guard/MPC chain a parked fallback; `run_mpc.py`
   and the AOT-compile section are marked dormant; the stale
   `_start_setpoint_output` docstring that claimed a ZMQ SUB on motor_guard's
   :5556 now says :5557/`mpccmd` (the correct story was already 1600 lines
   above it at `_MpcCommandSetpointSource`).
5. **Marker vocabulary.** `slow` deleted — one use, on a ~9 s module, and
   nothing ever selected on it. Left unregistered so `--strict-markers` turns a
   new `@pytest.mark.slow` into a hard error pointing at `nightly`.
   `hypothesis_deep` kept but annotated as reserved/unused (the nightly raises
   the profile suite-wide, so nothing needs to opt in).
6. **Contract documents updated, not silently drifted.** Three of the demoted
   modules are the named enforcement points of normative contracts, and
   `controller/HOT_LOOP_CONTRACT.md` § "CI expectations" *asserted* it ran in
   phase 2 of the gate on every commit. Demoting the test without amending the
   document would have been exactly the drift CLAUDE.md's "protect the
   contracts" rule forbids, so all three now state the new cadence
   (`HOT_LOOP_CONTRACT.md`, `DIAG_SCHEMA_CONTRACT.md`,
   `REFERENCE_LAYER_CONTRACT.md`). **No threshold or invariant was relaxed** —
   only the cadence moved, and only while the code they guard runs nowhere. The
   obligation is met by the `controller/`-and-`sim/` path trigger: any change
   that can break these contracts gates with `--full`, which runs them.
   `ros_ws/docs/{safety,control_modes}.md` got DORMANT banners for the same
   reason (a safety doc naming `motor_guard` as the authority now contradicts
   CLAUDE.md; both were already stale from the `can_node` deletion), as did
   `docs/motion_planner/{safety,control_loop}.md` — the *published* mkdocs pages
   (`mkdocs.yml` nav: "Motor Command Safety", "Motor Guard"), which is where a
   reader most plausibly lands on the contradiction. `sim/JUGGLE_DEMO.md` learned
   its test command skips the default gate.
7. **`test_retime.py` `_optimum_T` 60 -> 25 iterations.** Called 54 times, each
   iteration costing 54 build+validate pairs. 25 leaves a 2.3e-7 s bracket, 4x
   tighter than the tightest assert that reads it (`Tm >= opt - 1e-6`); the
   residual error is one-directional (returns the feasible `hi`, so early
   stopping can only OVERestimate).

## Verification

**Collect-only equality gate** (`pytest tests/ -q --collect-only -m <expr>`,
run 2026-08-01):

| selection | count |
|---|---|
| default p1 `not serial and not nightly` | 3890 |
| default p2 `serial and not nightly` | 3 |
| `nightly` | 432 |
| **sum** | **4325** |
| `--full` p1 `not serial` | 4316 |
| `--full` p2 `serial` | 9 |
| **sum** | **4325** |
| whole tree, unfiltered | 4325 |

Equality holds. The 9 `serial` tests partition cleanly: 3 stay in the default
serial phase (`test_motor_guard::{test_loop_timing,test_ipc_latency}`,
`test_motor_guard_friction_ff::test_friction_ff_no_steady_state_alloc`) and 6
are serial+nightly (`test_hot_loop_allocation_contract` ×3,
`test_mpc_static::test_solve_time`, `test_mpc_trajectory::test_solve_time`,
`test_mpc_time_pathologies::…on_post_solve_allocates_within_budget`). **None
vanished from both default phases.** Total rose 4324 -> 4325: the seed-0
kinematic MAKE now exists in both the gate file and the nightly sweep.

**Retime corpus diff** (`/tmp/probe_retime_iters.py`, run 2026-08-01): all 54
corpus cases evaluated at `iters=60` and `iters=25`; **0 per-case verdict
differences**, both aggregate asserts unchanged (mean model overshoot 0.019896,
mean legacy 0.082823 at both), `max(opt_25 - opt_60) = 2.279e-7 s` against the
2.325e-7 s bracket bound. `pytest tests/motion/test_retime.py -q`, run
2026-08-01: **10 passed in 91.28 s**.

**Split file, per-commit cost** (`pytest tests/sim/test_juggle_selfcatch*.py -q`,
run 2026-08-01): gate file **8 passed in 60.97 s** (was 230.83 s for the whole
file — ~170 s off the xdist critical path); nightly file **27 passed, 1 xfailed
in 190.46 s**.

**Timer armed** (`systemctl --user list-timers jugglebot-nightly.timer`, run
2026-08-01): `NEXT Sun 2026-08-02 04:03:23 AEST`, `is-enabled = enabled`,
`loginctl show-user jetson -p Linger = Linger=yes`.

**PRE-COMMIT GATE — `--full`, because this touches `controller/` and `sim/`**
(`./run_tests.sh --full`, run 2026-08-01 on the Jetson under
`~/Desktop/PDJ_venv/venv`): **parallel 4313 passed, 3 xfailed in 438.02 s;
serial 9 passed in 39.66 s; total 483 s; RESULT: PASS** (exit 0). Phase counts
match the collect-only table exactly (4313 + 3 = 4316 = full-p1; 9 = full-p2).
This is the run the commit cites. `tests/ros/test_launch_nodes.py`, the launch
tripwire, is inside it and green.

**Default gate — the point of the exercise** (`./run_tests.sh`, run 2026-08-01):
**parallel 3890 passed in 179.99 s; serial 3 passed in 15.35 s; total 200 s;
RESULT: PASS** (exit 0). Counts match the collect-only table exactly. Against
the pre-tiering gate on the same box (`./run_tests.sh`, run 2026-07-31: parallel
446 s / 4287 passed + 3 xfailed, serial 32 s / 7 passed, total 478 s) that is
**478 s -> 200 s, a 58% cut** — the plan projected ~8 -> 5.5–6 min, so the
native pre-build and the retime trim beat the estimate. Caveat: measured on a
warm tree (page cache hot, native binaries cached) right after two full runs; a
cold first run of the day will be slower. Second caveat: this 200 s figure was
measured before the review-fix pass; those fixes changed only comments in test
modules, prose in normative docs, and `tools/nightly_suite.sh` (which the gate
does not run), so the figure stands — but the `--full` run above is the one that
actually gates this commit.

**Live-session guard, both directions** (run 2026-08-01, with a stand-in process
named `trajectory_node`): budget expiry → `tools/nightly_suite.sh` exited 0
without running the suite and wrote
`DEFERRED live robot session held the box for 4s; suite NOT run <iso>`; session
clearing mid-wait → `session cleared after 6s — proceeding`, then the suite;
no live session → proceeds immediately. The pre-existing `status` file was
backed up and restored around the test.

**Full tier — the runner exercised end to end** (`tools/nightly_suite.sh`, i.e.
`./run_tests.sh --full --hypothesis-profile=ci-deep`, run 2026-08-01): **parallel
4313 passed + 3 xfailed in 1213.70 s; serial 9 passed in 39.97 s; total 1259 s;
RESULT: PASS** (exit 0). `temp/reports/nightly/status` reads
`GREEN 4322/4325 passed, 0 failed, 0 errored, 3 xfailed, 0 skipped
2026-08-01T12:00:32+10:00`; the report, per-phase junit XML and the `latest.md`
symlink all landed. Phase counts match the collect-only table exactly
(4313 + 3 = 4316 = full-p1; 9 = full-p2).

`colcon build --packages-select jugglebot`, re-run 2026-08-01 after the
review-fix launch-comment edits: 1 package finished [2.47 s]; the installed copy
was re-checked for the dormancy edits and parses. **The launch runs the INSTALLED
copy — relaunch
`ros2 launch jugglebot jugglebot_launch.py` to pick up the dormancy.** Expect
two fewer processes in the launch output; nothing on the leg or hand path
changes.
