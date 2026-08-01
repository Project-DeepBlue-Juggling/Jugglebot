---
title: "Refactor programme Phase 6 (slice 2) — one sim import root, controller/demo → sim/juggle_planner, clock-offset dedup, bridge test harness"
type: refactor
date: 2026-08-01
status: resolved
phase: "refactor-2026-07 Phase 6 (slice 2)"
related_plan: refactor-2026-07.md
files_changed:
  - sim/_paths.py
  - tests/sim/test_sim_import_style.py
  - tests/sim/conftest.py
  - sim/juggle_planner/
  - sim/viz/reference_plot.py
  - controller/generate_solver.py
  - ros_ws/src/jugglebot/jugglebot/clock_offset.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - tests/ros/_bridge_harness.py
  - tests/ros/test_clock_offset.py
  - tools/motion_onset_cogging_study.py
  - tools/probes/
  - run_mpc.py
  - plans/active/refactor-2026-07.md
subsystem:
  - sim
  - ros
  - testing
---

# Phase 6 slice 2 — four structural cleanups (sequenced, each gated)

Four mechanical-but-wide changes, landed in the plan's mandated order (import
root before the move, so the move cannot add dual-identity surface). No
control-path behaviour changes: the ROS dedup is byte-equivalence-gated and
everything else is imports, file locations and test scaffolding.

**Unit 1 — one import root for sim/.** 179 bare-style imports across 71 files
(`from plant.mujoco_plant import …`) became `sim.*`, and the ~30 hand-rolled
`sys.path` blocks became one `sim/_paths.bootstrap_paths()` called only by
runnable entry scripts; library modules under `sim/` now mutate `sys.path`
never. The bug this closes: with `sim/` *and* the repo root both on the path,
the same file loads as two module objects, so a `patch()` against one identity
silently patches nothing. `tests/sim/test_sim_import_style.py` (4 tests) is the
enforcement point — an AST grep that fails if a bare-style import or a
hand-rolled path block reappears anywhere in `sim/ tests/ tools/ controller/`.
**Deliberate deviation from the plan text**: the bootstrap installs *three*
roots, not four — `sim/` is retired, because keeping it would keep the
dual-identity mechanism live in every bootstrapped process (including pytest,
where it is exactly the bug) while providing nothing after the conversion.
Direct script runs still see `sim/` because CPython always prepends the
script's own directory, which is why the durable guarantee is the import-style
contract, not path hygiene.

**Unit 2 — `controller/demo/` → `sim/juggle_planner/`** (`git mv`, 49 refs
across 21 files). Every consumer is a sim entry script, a sim test or a
`tools/probes/juggle_*` probe; nothing on the hardware control path imported
it, while `controller/` *is* imported by the hardware MPC loop. The move also
dissolved the boundary that forced seven hand-copied hand-stroke constants into
`juggle_optimizer.py`; they are now real imports from `sim.hand.ballistics` and
`sim.hand.trajectory`, verified bit-identical at the swap (20.0 / 315.0 /
−129.0 / 44.4 / 0.747 / 0.05 / 0.10, printed before and after).

**Unit 3 — clock-offset dedup.** `trajectory_node` and
`catch_coordinator_node` carried character-identical copies of the
ROS-clock→`perf_counter` estimator (10-sample median, 20-deep history, 30 s
refresh). Both now call `jugglebot/clock_offset.py` — pure, clock injected as a
callable, each node keeping its own `nanoseconds / 1e9` expression so the last
ulp cannot move. `tests/ros/test_clock_offset.py` (8 tests) drives the new code
and a verbatim transcription of the old implementation through the same
scripted clock and asserts **exact float equality**, including 30 refreshes
past the 20-deep trim. `reload_coordinator_node`'s single-instantaneous-read
variant is left alone with a comment saying so: reconciling it is an open
decision, not an oversight.

**Unit 4 — bridge test harness.** 21 files imported `_build_paired_node` /
`_wait_until` *from a test module*, making a read-side test file the de-facto
public API of the bridge suite, and `_teardown` had been copy-pasted 19 times.
All of it moved to non-collected `tests/ros/_bridge_harness.py`, together with
the byte-identical `_poll` (×6), `_platform_frame` (×3), `_link_kv` (×2) and
`_messages` (×2). The one divergent `_teardown` (shutdown_stow's) is absorbed:
its extra line is a no-op for the other 18, which is what makes one canonical
teardown safe.

## Verification

- Collect-only counts (`pytest tests/ -q --collect-only`, run 2026-08-01):
  **4398** before Unit 1 → **4402** after Unit 1 (+4 = the new import-style
  guard file) → **4402** after Unit 2 (move adds no tests) → **4410** after
  Unit 3 (+8 = `test_clock_offset.py`) → **4410** after Unit 4 (harness
  extraction adds and removes none — the Unit 4 gate).
- Dual-identity probe (`/tmp/probe_dual_identity*.py`, run 2026-08-01): under
  the bootstrap the legacy spelling is unimportable (`ModuleNotFoundError`);
  under the direct-script condition (`sim/` force-added, as CPython does) a
  full `sim.main` import loads 11 sim modules and **zero** under a legacy
  identity.
- `python sim/main.py --no-viewer --duration 2`, run 2026-08-01: exit 0,
  tracking error 0.035 mm / 0.0028°, 80 records logged.
- Entry-script coverage, restated honestly after review (see below): an
  import smoke executes the **module body only** and never enters `main()`,
  which is where four of the five review-found breaks lived. Post-fix, from
  cwd `/tmp` with `PYTHONPATH` cleared: **54 / 55** changed `sim/`, `tools/`,
  `run_mpc.py` and `controller/generate_solver.py` modules import clean (the
  55th, `sim/plant/mujoco_plant.py`, uses package-relative imports and is not
  loadable standalone by construction); `--help` exits 0 for all four `tools/`
  CLIs, which *does* reach the argparse defaults; and an AST
  undefined-global-name scan over all **138** changed `.py` files returns
  **0 hits** — the check that closes the dangling-rename class.
- `controller/generate_solver.py`'s import chain (`controller.params`,
  `controller.mpc`, `sim.plant.mujoco_plant`) resolves under an emulation of
  the real invocation (`sys.path[0] = controller/`, clean env), run
  2026-08-01. Verified **by import, never by running the script** — it
  unlinks `mpc_gen.so` and `mpc_gen.hash` before rebuilding, so a failed run
  would delete the AOT artifact the hardware MPC loads.
- `sim.viz.reference_plot._load_geometry()` returns real geometry
  (init height 574.3 mm) from cwd `/tmp`, run 2026-08-01.
- `colcon build --packages-select jugglebot`, run 2026-08-01: 1 package
  finished, 2.51 s. Installed-copy import smoke under **system python3.8.10**
  (installed tree only on `PYTHONPATH`): `jugglebot.clock_offset` imports,
  constants 10 / 20 / 30.0, both functions evaluate.
- Full gate, **implementer's run** (`./run_tests.sh --full`, run 2026-08-01 on
  the Jetson under `~/Desktop/PDJ_venv/venv`): parallel 4398 passed, 3 xfailed
  in 440.66 s; serial 9 passed in 40.10 s; total 486 s; RESULT: PASS (exit 0).
- Full gate, **re-run after the review fixes and the only one that gates this
  commit** (`./run_tests.sh --full`, run 2026-08-01 on the Jetson under
  `~/Desktop/PDJ_venv/venv`): **parallel 4398 passed, 3 xfailed in 444.94 s;
  serial 9 passed, 4401 deselected in 40.23 s; total 491 s; RESULT: PASS**
  (exit 0). Collect total unchanged at 4410 — the fixes changed test *bodies*
  (`test_shared_constants_match_both_node_call_sites`,
  `test_no_dual_module_identity_in_the_test_session`), not the test count.
- Logbook artefacts, written after that gate (`pytest
  tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py
  tests/sim/test_plans_index.py -q`, run 2026-08-01):
  **54 passed in 0.47 s**. These are the tests that parse `logbook/`
  and `plans/active/INDEX.md`; note the suite reads neither this entry's body
  nor the plan bullets, so their correctness rests on review, not on green.

## Found at review (all fixed before the commit)

Two adversarial reviews found **five real breaks the full suite passed over**,
all of one shape: *code the test suite never executes*.

| Break | Why the suite missed it |
|---|---|
| `controller/generate_solver.py` — `from sim.plant.mujoco_plant import …` lost its roots when the plant's self-bootstrap was deleted; `ModuleNotFoundError: jugglebot` | The only mention of `generate_solver` in `tests/` is a *string* match (`pytest.raises(match='generate_solver.py')`). The script is never executed. Its imports are late (after `parse_args`), so even `--help` passes. |
| `sim/viz/reference_plot.py:59` — `from motion.geometry import …` lost the **inner** `ros_ws/src/jugglebot/jugglebot` root, which the bootstrap deliberately does not install | The import is lazy (inside `_load_geometry`), and no test imports `reference_plot` at all. |
| `tools/motion_onset_cogging_study.py:273,280` — `_REPO_ROOT` renamed to `_repo_root`, two uses left behind; `NameError` on **any** invocation | In argparse defaults inside `main()`. No test covers the tool. |
| `tools/probes/juggle_catch_offset.py:84` and `juggle_throw_accuracy.py:93` — `_REPO` undefined; `juggle_motion_quality.py:129` — `_root` undefined | In the **result-writing** path: each probe runs its whole MuJoCo sweep, then dies while writing the output. Import-smoking cannot reach it. |

The class is *not* "the reviewers were thorough" — it is that this codebase's
runnable surface (build scripts, probes, one-off CLIs) is deliberately outside
the test suite, so a tree-wide mechanical rewrite gets **no gate at all** on
roughly 20 files. The generalisable check is the one now recorded above: an
AST undefined-global-name scan over every changed file, plus `--help` on
anything with an argparse block, plus an explicit import of any module a build
script imports. A green `./run_tests.sh --full` says nothing about any of them.

The lower-severity findings applied at the same time:
`REFRESH_PERIOD_S` was documented as the nodes' timer period while both nodes
still hardcoded `30.0` — the shared constant had **no reader**, which is the
duplicated-timing-constant shape the unit set out to kill, merely relocated.
Both `create_timer` call sites now pass it, and
`test_shared_constants_match_both_node_call_sites` AST-reads those two call
sites and fails if either reverts to a literal (mutation-checked: reverting
one makes the test fail). Nineteen entry scripts carried a bootstrap comment
naming `sim/` as an installed root — the exact opposite of the contract, in
the 19 places a reader looks first. `juggle_planner` joined `_SIM_TOP_LEVEL`,
without which the newest sim package was the one hole the dual-identity class
could re-open through (the stale plan documents still teach
`from juggle_planner.timeline import …`). And
`test_no_dual_module_identity_in_the_test_session` now pins `sim.__file__`:
`tests/sim/__init__.py` makes tests/sim a *regular package also named `sim`*,
and `tests/` is on `sys.path`, so `import sim` resolves correctly today only
by pytest's insertion order.

**Relaunch guidance**: `ros_ws/src` changed, so the running system needs
`cd ros_ws && colcon build --packages-select jugglebot && source install/setup.bash`
and a relaunch — `jugglebot_launch.py` runs the *installed* copy, not the
live tree.

## Discussion

The plan text said the bootstrap must keep all four path roots; keeping `sim/`
turned out to be incompatible with the unit's own acceptance probe, and the
probe is testing the thing that matters. With `sim/` installed, an explicit
bare import still mints a twin — verified live, not assumed — so "four roots
survive" and "one module object" cannot both hold. Every capability the fourth
root provided is served by the repo root after the conversion, and its only
remaining function was to keep the bug reachable, so it went. The residual
exposure (a direct `python sim/main.py` always has `sim/` on the path, courtesy
of CPython) is unfixable by path hygiene and is why the enforcement is a
contract with a test rather than a clever bootstrap: a `sys.modules`-aliasing
import hook would have closed the last gap, but it would have put an import
hook in the process that runs `run_mpc.py --sim`, to buy only out-of-tree
scratch scripts that are broken by definition. Contract over patch.

The `controller/demo` grep is at zero for code but not for prose: 9 logbook
entries and 4 plan documents still name the old path. Logbook entries are
historical record and must not be rewritten; the plan-document refresh is left
as a separate narrative change (it trips the `/audit` gate, and this commit is
code-only). `sim/juggle_planner/__init__.py` carries the forwarding note so a
reader arriving from a stale reference lands correctly.

Two latent indirection deltas in Unit 3 are recorded rather than fixed, because
neither has a caller today and both would be paid for with worse code. The old
`_refresh_clock_offset` called `self._measure_clock_offset()` — a virtual
dispatch — while the new one calls the module function directly, so overriding
that method on an instance no longer affects the refresh path; and
`perf_clock: Callable = time.perf_counter` binds the function object at import
time, so a `patch('time.perf_counter')` that used to reach the nodes now
reaches nothing. Grepped: no test or subclass does either. The second is
ironic (it is a small instance of the same "patches nothing" class, in the
module written to close it) but injecting the clock is what makes the
byte-equivalence gate possible at all, and the fix — a late `time.perf_counter`
lookup per call — would add an attribute lookup between the two clock reads,
which is the one place this module cares about. Also: the dedup puts two extra
Python frames between the perf read and the ROS read, a consistently-signed
sub-microsecond bias the scripted-clock tests structurally cannot see. Against
millisecond-scale catch deadlines it is immaterial; the docstring's "a few
microseconds" claim is what the byte-equivalence gate does *not* prove.

One pre-existing breakage was found and deliberately not fixed: `sim/Dockerfile`
+ `compose.yaml` (untouched since the first sim commit, `c543748`) put `/app`
and `/repo/ros_ws/...` on `PYTHONPATH` but never `/repo`, while `sim/main.py`
imports `controller.target` at module level — so `docker compose up` could not
have worked before this change either. Fixing it properly needs a
writable-volume decision (the repo is mounted read-only, and telemetry writes to
`temp/logs/`), which is an owner call, not a refactor side-effect.
