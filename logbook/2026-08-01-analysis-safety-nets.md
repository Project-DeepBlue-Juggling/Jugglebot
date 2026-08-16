---
title: "Analysis-layer safety nets — logbook front-matter contract, topic-choreography map, GUI FK golden vectors, stale-doc sweep"
type: feature
date: 2026-08-01
status: resolved
phase: "refactor-2026-07 Phase 6 (slice 1)"
related_plan: refactor-2026-07.md
files_changed:
  - sim/analysis/logbook_search.py
  - tests/sim/test_logbook_front_matter.py
  - logbook/README.md
  - .claude/agents/fix-proposer.md
  - tools/gen_choreography_map.py
  - ros_ws/docs/choreography.md
  - tests/ros/test_choreography_map.py
  - tools/gen_gui_fk_golden.py
  - tools/README.md
  - tests/ros/gui_fk_golden.json
  - tests/ros/js/fk_harness.js
  - tests/ros/test_gui_fk_golden.py
  - docs/sim_mpc/velocity_tracking.md
  - docs/sim_mpc/usage.md
  - docs/sim_mpc/nlp_formulation.md
  - docs/sim_mpc/tuning.md
  - docs/sim_mpc/variable_horizon.md
  - docs/sim_mpc/index.md
  - docs/motion_planner/operations.md
  - docs/motion_planner/results.md
  - docs/motion_planner/trajectory.md
  - docs/adr/0001-offload-can-and-interpolator-from-jetson.md
  - docs/adr/0008-time-sync-master-on-can-bridge.md
  - docs/adr/0009-freertos-tsandmann-port.md
  - docs/adr/0012-hermite-interpolator-port.md
  - docs/adr/index.md
  - ros_ws/docs/can-node-teensy-parity.md
  - ros_ws/docs/safety.md
  - logbook/INDEX.md
subsystem:
  - testing
  - docs
  - ros
  - gui
tags:
  - observability
  - drift-gate
  - kinematics
---

# Analysis-layer safety nets (Phase 6, slice 1)

## What changed and why

Four nets over things that could rot without anything going red. No
control-path code was touched.

1. **`logbook_search` warns instead of dropping silently.** `load_entries`
   skipped any entry whose front matter lacked a `title`, with no output — so
   "this investigation was never done" and "this entry is unreadable" looked
   identical to the fix-proposer that consults it. New `scan_entries()` returns
   `(entries, warnings)`; `load_entries` keeps its signature and surfaces to
   stderr unless a caller passes a list. A *titled* entry missing another
   required key still loads (it is real prior art) but warns.
   `tests/sim/test_logbook_front_matter.py` asserts every committed
   `logbook/*.md` carries title/type/date/status with an ISO date, that the
   loader returns exactly one record per file, and that it emits **zero**
   warnings. All 180 entry files (183 `logbook/*.md` less INDEX/README/TEMPLATE,
   this entry included) already passed — no front matter needed fixing.
   The contract's third part landed too: `logbook/README.md` now *states* the
   four required keys, the ISO-date rule and the test that gates them (with the
   two innocent-looking ways to redden it — a trailing `# comment` on a required
   value, which the ~30-line no-dependency parser keeps verbatim, and an
   unfilled `TEMPLATE.md` placeholder). `.claude/agents/fix-proposer.md` gained
   the stdout-is-JSON / warnings-on-stderr note, since Bash merges the streams.
2. **Generated topic-choreography map** (`ros_ws/docs/choreography.md`): **75
   distinct wires across 140 endpoints** (41 publishers, 43 subscribers, 30
   service servers, 18 service clients, 5 action servers, 3 action clients).
   Pure `ast`, never imports rclpy. Two deliberate honesty properties: names are
   resolved only through exact paths (literal, module/class constant,
   all-literal f-string, ROS-parameter default — tagged `param:<key>`) and
   anything else emits `UNRESOLVED(<expr>)`; and call-site **line numbers are
   omitted**, because they rot silently and would make the drift test fire on
   unrelated edits. The scan covers *every* module under the package, not
   `*_node.py`: `spacemouse_handler.py` is a launched node without the suffix
   and the sole publisher of `platform_pose_topic`, which a filename-convention
   scan would have printed as publisher-less. `catch/dynamic_target`,
   `throw_announcements` and the five `catch/*` hand-ownership latches carry
   one-line contract notes sourced from `ros_ws/docs/`. Parked nodes are tagged
   **`(not launched)`** — see Discussion.
3. **GUI FK golden vectors.** `ros_ws/gui/js/stewart-fk.js` is the repo's only
   duplicate kinematics implementation and was pinned only by a manual browser
   page. `tools/gen_gui_fk_golden.py` writes 25 poses spanning 11.6–246 mm of
   the 280 mm stroke from the **Python** IK; the pytest replays them through the
   real JS under node. No shim was needed — `stewart-fk.js` imports only
   `geometry-config.js` (no THREE.js), and the fixture copies both verbatim
   beside a `{"type":"module"}` package.json rather than rewriting sources.
   Measured agreement: IK 1.1e-13 mm, FK pose 4.6e-7 mm, FK rotation 3.4e-9 rad.
   Skips (never fails) when node is unresolvable.
4. **Stale-doc sweep.** `docs/sim_mpc/velocity_tracking.md`'s
   `sim/tests/test_mpc_dynamic.py` → `tests/sim/`, plus the rest of the dead
   paths found by scanning `docs/`, `ros_ws/docs/` and `tools/README.md` — every
   one enumerated under Verification rather than counted, so a reader can check
   them individually.

## Discussion

**Three silent-lie holes in the choreography resolver, closed before the map
shipped.** The map's whole premise is "a map that quietly guesses is worse than
no map", and review found the generator violating it in three places — all
latent (the committed map was true), all fixed here with the two probes turned
into tests:

- `self.X` was answered from a same-named *module* constant, with provenance
  `constant`. `TOPIC = 'module/level'` at module scope plus
  `self.TOPIC = 'instance/level'` in `__init__` rendered the module value,
  confidently and unmarked. The `ast.Attribute` resolution path is **deleted**
  rather than narrowed: keying constants by bare name cannot distinguish an
  instance attribute from a module constant from another class's class-body
  constant, and honest instance-attribute tracking is real work nobody needs
  yet — no endpoint in the package uses the shape. It now falls through to
  `UNRESOLVED(self.TOPIC)`, which the "Unresolved names" section makes visible.
  Cost: the synthetic `self.CLASS_CONST` case stops resolving. Accepted — a
  visible hole beats a plausible lie, and the fix is additive when someone needs
  it.
- The per-function local index was built with `ast.walk`, which descends into
  nested `def`s, so a helper's binding leaked upward and answered an *enclosing*
  call site with provenance `literal`. Now bounded by `_own_nodes()`, which
  stops at `FunctionDef`/`AsyncFunctionDef`/`Lambda`/`ClassDef`; nested
  functions still get their own scope, so nothing is lost.
- The doc's closing line read *"every endpoint name resolved to a literal"*
  while the body two hundred lines earlier tagged `control_mode_topic` as
  `param:control_mode_topic`. Fixed wording.

**Why the drift test could not have caught any of these.**
`test_committed_doc_matches_regeneration` pins doc == generator, not generator
== source truth: a wrong resolution lands on both sides and stays green. That is
an argument for the hand-written truth anchors (spacemouse_handler, the
multi-publisher wires, the five latch names) and now for the resolver-refusal
tests, not against the drift gate — but it is worth stating plainly, because a
future reader will otherwise over-trust a green suite.

**The map presented two dormant nodes as live graph participants.** Phase 3
(`02ffe0a`, 2026-08-01) made `motion_bridge_node` dormant and dropped
`mpc_bridge_node` from `jugglebot_launch.py`. The map carried both across eight
wires with no marker, so "who consumes `platform_pose_topic`?" answered with a
node that never starts — lying by omission about precisely what Phase 3 had just
changed. Fixed with a rendered `(not launched)` tag plus a preamble paragraph,
and — because a hand-maintained tag list is the next thing to rot — a test that
cross-checks `NOT_LAUNCHED_NODES` against the launch file **in both directions**:
re-enabling a node must remove its tag, and dropping one must add it. The scanned
set is today exactly the launched set plus those two, so the assertion is an
equality, not a subset.

**`FK_POS_TOL_MM` is now derived, not observed.** It was the smallest margin in
the file (4.6e-7 mm observed against a 5e-6 mm gate) and the first assertion a
different node build would move. Raising `stewart-fk.js`'s `const tol = 1e-6` in
a sandbox copy shows the pose error is *bounded* by ~0.53x the solver's residual
tolerance (1e-5 → 5.29e-6 mm; 1e-6 → 4.64e-7 mm; 1e-7 → 7.96e-9 mm — it falls
faster than linearly below, because Newton's last step overshoots the stopping
test). So the algorithmic ceiling at the shipped tolerance is ~5.3e-7 mm and the
gate is ~10x *that*, not 10x one lucky sample. Recorded in the test docstring
with a "re-derive the ceiling before loosening this" instruction.

**Two test-shape changes made for gate robustness, not style.**
`test_golden_file_matches_python_ground_truth` compared the committed JSON
**byte-for-byte** and runs unconditionally (it is not behind `requires_node`), so
on the Win10 clone a different numpy/BLAS float repr would redden the gate for a
non-defect, with a 25-vector JSON blob as the diff. It now compares parsed
structure at 1e-12 relative — four orders tighter than the loosest kinematics
gate here, so a real IK change still fails. And the front-matter test's
`parametrize(os.listdir(logbook/))` made xdist *collection identity* depend on
the logbook directory being stable across worker startup; CLAUDE.md documents
parallel sessions on this shared tree as normal, and a file appearing mid-
collection aborts the whole run with "Different tests were collected between gw0
and gw1" instead of producing a clean red. Now one test that loops and names
every offender — the per-file test id was the only thing lost.

## Verification

- **Full gate**: `./run_tests.sh --full`, run 2026-08-01 on the Jetson under
  `~/Desktop/PDJ_venv/venv`: **parallel 4386 passed, 3 xfailed in 446.49 s;
  serial 9 passed in 40.18 s; total 493 s; RESULT: PASS** (exit 0). `--full`
  because this slice changes `sim/analysis/logbook_search.py`.
- Scoped, run 2026-08-01: `pytest tests/ros/test_choreography_map.py
  tests/ros/test_gui_fk_golden.py tests/sim/test_logbook_front_matter.py
  tests/sim/test_logbook_search.py -q` → **71 passed in 5.54 s** after the
  review fixes (the per-file parametrization that made it 246 is gone).
- Mutation-checked the JS half rather than trusting a green run (2026-08-01):
  flipping `motorRevs[i] / MM_TO_REV[i]` to `*` in a sandbox copy of
  `stewart-fk.js` moves `fk_from_revs` by **261.9 mm** against a 5e-6 mm gate.
  A rev/mm inversion is the exact class this test exists for.
- Resolver-refusal probes reproduced against the real generator before and after
  the fix (2026-08-01): `self.TOPIC` with a shadowing module constant went
  `('module/level', 'constant')` → `('UNRESOLVED(self.TOPIC)', 'unresolved')`;
  a nested-`def` binding went `('inner/only', 'literal')` →
  `('UNRESOLVED(topic)', 'unresolved')`. The real package is unchanged by both
  fixes: 140 endpoints, 139 `literal` + 1 `param:`, zero `constant`.
- Choreography `--check` returns 0 on the committed doc and 1 on a mutated one;
  `generate()` is byte-stable across runs.
- Dead paths fixed mechanically: `sim/controller/{mpc,params}.py` →
  `controller/` (3 files + the `sim_mpc/index.md` table), `tools/*_test.py` →
  `tests/hardware/` (2 files), `tests/test_mpc_{static,dynamic}.py` →
  `tests/sim/` (usage.md), `plans/archived/HANDOFF-teensy-can-offload-firmware-wip.md`
  → `plans/archived/2026-07-05 …` (3 ADRs), `archived/level_platform_node.py`
  → `attic/ros-jugglebot-archived/` (parity matrix). Two dead ADR *links* to
  files deleted in the 2026-07-06 SocketCAN decommission (`can_node.py`,
  `can/bus.py`) were de-linked to code spans naming the deletion, since ADRs are
  historical records and the targets are not coming back.
- The `tools/archived/` rot pocket is closed, not deferred: `git log
  --diff-filter=D` shows all nine "missing" harnesses (`trajectory_test.py`,
  `inertia_test.py`, `hardening_test.py`, `dynamic_target_test.py`,
  `juggling_test.py`, `smoother_test.py`, `trajectory_viewer.py`,
  `catch_sim_test.py`, `throw_catch_test.py`) — plus `submit_dynamic_target_sync`
  in `tests/archived/test_helpers.py` — deleted **deliberately** in 67889a6
  (2026-04-17, "remove tests/archived/ — pre-MPC era tests"). That was the
  were-these-deleted-deliberately answer the sweep was missing, so
  `tools/README.md`'s table, `docs/motion_planner/operations.md`'s "archived test
  harnesses" note (which pointed at a `tools/archived/` directory that has never
  existed) and `docs/motion_planner/trajectory.md`'s synchronous-path note are
  all re-pointed at `git show 67889a6^:tests/archived/<name>`.

## Known-stale, left for the owner

- **`ros_ws/docs/safety.md` §3** documents a position-step limit in
  `jugglebot/can_node.py`, deleted 2026-07-06. The clamp survives — ported into
  `controller/teensy_link/setpoint_pump.py` (`max_step_rev`, gating against the
  prior *accepted* setpoint rather than encoder feedback, complementary to the
  Teensy `MAX_DEVIATION` guard). Phase 3's existing STALE banner was **extended**
  to name §3 and point at the live module, rather than spot-patching the section:
  §§1–2 describe the same parked chain, and patching one section would imply the
  others are current. The file is still owed the one re-framing pass Phase 3
  flagged.
- `docs/knot-rate-analysis.md` references `scratchpad/knotrate/` and `out/*.json`
  from an external working directory; never repo paths, left as-is.
- **`mkdocs build --strict` was not exercised** — mkdocs is not installed in the
  venv, and `.github/workflows/docs.yml` only runs it on pushes to `main`/
  `refactor`. All 15 markdown links touched by the sweep were resolved
  programmatically instead (0 broken, including the `%20`-escaped
  `plans/archived/2026-07-05 HANDOFF-…md`).
- **Warm-start FK and the mocap path are unpinned.** The harness calls
  `resetFKState()` before every vector, so only the cold-start FK is covered;
  `quatToRotMatrix()` / `poseToPlatNodes()` are exported but unexercised. Stated
  in `tests/ros/js/fk_harness.js` so a green run is not over-read.
