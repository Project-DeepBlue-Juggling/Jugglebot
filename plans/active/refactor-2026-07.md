# Refactor programme — 2026-07

**Status:** active (Phases 0–4 landed; 5–7 partial — see the row in `plans/active/INDEX.md`)
**Owner decisions recorded:** 2026-07-31 (all items below marked APPROVED were
explicitly approved by the operator in the 2026-07-31 review session)
**Source:** 13-agent codebase review + adversarial red-team pass, 2026-07-31.
Full findings were synthesized in-session; everything load-bearing is baked
into this plan. Companion memory: `refactor-programme-2026-07`.

## Verdict the plan rests on

The load-bearing architecture (K1–K6 reference feasibility, hot-loop
allocation contract, PlantInterface, protocol codegen + byte-diff xlang
tests) is healthy. The drag is: package boundaries that no longer match
reality, dead weight, three god-files, and process ceremony. **Prime
directive: the robot keeps working exactly as it does.** Every phase below
is staged so no control-path behaviour changes without its own gate.

Explicitly REJECTED (do not resurrect without new evidence): big-bang
rewrites of teensy_bridge_node/mpc.py/sequencers; extracting
trajectory_node's mode/streaming/escalation state machine (lock discipline
is load-bearing — `trajectory_node.py` ~832–954, ~1436–1541); deleting the
MPC code (vs. parking it); re-opening a CAN3 software source-hunt (hardware
verdict at ~85–90%: clean-era binaries reproduced the fault, fault follows
the transmitter, role-swap left the fault on the CAN3 column); trimming
hypothesis ci-fast depth before the nightly runner has weeks of history.

## Phase 0 — zero-risk slice (LANDED 2026-07-31)

- [x] Telemetry CSV loader: single typed `load_records()` in
  `controller/telemetry.py`; `analysis.compare.load_csv` delegates; fixes
  the confirmed `float('')`-on-`throw_phase` crash AND the int-fields-as-str
  defect in `TelemetryLogger.load`. Pinned by
  `tests/sim/test_telemetry_roundtrip.py` (types, not just values).
- [x] Dead-weight sweep: `jugglebot/archived/` (30 files, 16.3k lines) →
  `attic/ros-jugglebot-archived/`; deleted sim ghosts (8 unreferenced
  `sim/viz/reference_*.png`, `sweep_speed_ratio.py`+pngs,
  `motor_command_verification.png`, empty `sim/tests/` + its stale
  gitignore rule), untracked `sim/controller/` + `tests/archived/` pycache
  remnants and dead `ros_ws/src/yasmin`; `archived/` doc pointers
  re-pointed at the attic; probes README gained the missing
  `canbridge_reboot_latch_probe.py` row.

## Phase 1 — process changes (LANDED 2026-08-01)

All approved 2026-07-31; all six items implemented 2026-08-01 — logbook
`2026-08-01-process-rules-phase1`. Gate: `./run_tests.sh`, run 2026-08-01 on
the Jetson under `~/Desktop/PDJ_venv/venv`: **parallel 4312 passed, 3 xfailed
in 435.18 s; serial 9 passed in 39.75 s; RESULT: PASS** (exit 0). Two deliberate deviations, both recorded
there: (a) `leg-gain-tuning-methodology.md` was **left active** — the gain
*hunt* closed, but the document is the live normative methodology cited by
`sim/analysis/diagnose.py`, `config/hardware_config.yaml`, four hardware
harnesses and the active accel-FF plan, so archiving it would have rotted
production references for no gain; (b) CLAUDE.md landed at **22.1 KB** (25.4 → 22.1, −13%), not the
~15 KB target — the three named worked examples are fully relocated and every
rule was tightened, but reaching 15 KB would require deleting rules, which the
"without losing a single rule" constraint forbids. Review restored ~0.6 KB of
obligation text the diet had over-trimmed (the open-ended normative-doc list;
the probe README-row rule moved into `tools/probes/README.md`).

1. **Retire the SHA-backfill second commit** (was 103/342 = 30% of July
   commits). The `Logbook-Entry:` trailer is the canonical link; entries no
   longer carry their own commit SHA. Verify every 2026 entry stays
   reachable via `git log --grep` before deleting the convention text.
2. **Short-form logbook entries by default** (10–30 lines: what/why +
   (date, command, result) verification triple). The full investigation
   form stays MANDATORY under the three existing Discussion triggers
   (hypothesis withdrawn; non-obvious tradeoff; approach beat an
   alternative for non-inferable reasons).
3. **Scope the /audit gate** to genuinely multi-document narrative changes
   (≥2 narrative files or any normative doc), not any-logbook-touch.
4. **Clarify the double-suite wording** in CLAUDE.md: one full-suite run
   satisfies both "after the change" and "before the commit" provided no
   edits intervene between suite and commit.
5. **Archive the ≥6 self-declared-done plans** via `/archive-plan`
   (follower-cadence RESOLVED 07-10; leg-gain CLOSED 07-13;
   reload-action-catch-latch landed 07-21; hardware-bringup,
   dashboard-3d-mesh, telemetry-daemon stale since May–June). Add a
   ~20-line `plans/active/INDEX.md` with a status column.
6. **Compact the memory layer**: merge closed-arc files into tombstones,
   target ≤25 files / ≤6 KB index; live safety-critical entries stay
   verbatim. Then a CLAUDE.md diet: move the inline worked examples into
   linked entries, keep one-line rules (CLAUDE.md is ~24 KB, the largest
   fixed-context artifact — bigger than the memory index).

## Phase 2 — nightly runner + test tiering (LANDED 2026-08-01, with Phase 3)

Landed together with Phase 3 per the hard rule below — logbook
`2026-08-01-nightly-tier-and-mpc-dormancy`. Gate: `./run_tests.sh --full`, run
2026-08-01 on the Jetson under `~/Desktop/PDJ_venv/venv`: **parallel 4313 passed,
3 xfailed in 438.02 s; serial 9 passed in 39.66 s; total 483 s; RESULT: PASS**
(exit 0). Collect-only equality gate (`pytest tests/ -q --collect-only -m <expr>`,
run 2026-08-01): 3890 + 3 + 432 = 4325 and 4316 + 9 = 4325, against 4325
unfiltered; the nightly tier is 432 tests across exactly 19 files. Default gate
478 s → 200 s. Two deviations: (a) the `test_motor_guard*.py` demotion (Phase 3
item 2) was **deferred** — both files were another session's uncommitted work in
the shared tree; `run_tests.sh`'s zero-serial guard was pre-adjusted so that the
demotion *emptying* the default serial phase does not fail the gate. **Landed
2026-08-01** by the parallelisation session that owned those files (logbook
`2026-08-01-vacuous-tests-return-not-assert`); the phase is now empty as
anticipated and the guard fired live for the first time; (b) a **live-session guard**
was added to `tools/nightly_suite.sh` beyond the plan text: `Persistent=true`
fires a missed run at the next boot, which can land inside a powered sitting, and
the unit's Nice/IOSchedulingClass bound CPU and IO but not memory — the runner
now waits for the box and writes `DEFERRED` on expiry rather than competing or
skipping.

**Hard rule: the runner and the first demotion land in the SAME commit.**
Without a real runner, demoted tests never run (verified: no crontab, no
timer, `.github/workflows` = docs.yml only).

- systemd timer, 04:00, `Persistent=true` (a night the Jetson is off →
  run fires on next boot). Runs `./run_tests.sh` full tier +
  `--hypothesis-profile=ci-deep`.
- Report: `temp/reports/nightly/YYYY-MM-DD.md` (+junit XML), `latest.md`
  symlink, one-line `status` file (GREEN/RED + counts). **Delivery =
  channel 2 (owner choice):** a session-start rule — any Claude session
  checks the status file first and surfaces a RED nightly with the failure
  list. No email/GUI channel for now.
- Demotions (content-drawn boundary — research/demo characterization only;
  the hardware-safety surface stays per-commit): split
  `test_juggle_selfcatch.py`'s three 6-seed sweeps (230.8 s, the xdist
  critical path) into gate-smoke (≥1 seed of the MAKE capability
  `test_oscillation_kinematic_release_sustains` STAYS per-commit) +
  nightly; demote `test_demo_juggle_*` e2e. Expected: gate ~8 → ~5.5–6 min
  wall-clock (~450 s serial removed).
- **MPC battery demotion rides here too** (Phase 3): ~370 s serial across
  15 files.
- Mechanics: `-m "not nightly"` composition with the existing serial phase
  MUST be gated on per-phase `--collect-only` count comparison (a wrong
  composition silently deselects the serial-marked hot-loop tests from
  BOTH phases). Fix the marker vocabulary (`slow` has one lying use;
  `nightly`/`hypothesis_deep` zero uses).
- Zero-coverage-loss mechanics: `test_retime.py` bisection 60 → ~25 iters
  (still 4× tighter than the tightest assert; verified safe in both
  directions — fewer iters only OVERestimates, no marginal flip possible);
  pre-build `tests/firmware/native` before the parallel phase so a cold
  ~170 s g++ compile never serializes into one worker.
- CLAUDE.md companion rules: `--full` mandatory before any hardware
  sitting and at plan-phase closure; path-trigger — changes under
  `controller/` or `sim/` run `--full` pre-commit.

## Phase 3 — MPC dormancy (LANDED 2026-08-01, with Phase 2)

Same commit and same gate as Phase 2 above. Item 2 landed in full; the
`test_motor_guard*.py` half followed on 2026-08-01 (logbook
`2026-08-01-vacuous-tests-return-not-assert`), which empties the default serial
phase as anticipated. Two things the plan text
did not anticipate, both recorded in the logbook entry: `motion_bridge_node` was
also the **sole publisher of `motion/diagnostics` and `motion/tracking_error`**,
so the GUI Motion panel now sits at `DISABLED` and those two bag topics record
empty (accepted — the MPC motion chain genuinely is disabled, and it degrades to
a badge, never a false ERR); and revival is **both** launch entries, not one,
because `HardwarePlant.enable()` blocks on motor-feedback telemetry from the
guard and the guard is fed by `motion_bridge_node`. Doc re-framing (item 3) also
reached the *published* mkdocs pages `docs/motion_planner/{safety,control_loop}.md`,
which the plan did not name; the rest of that nav section still describes the
parked chain and is owed a rewrite.

Owner intent (2026-07-31): remove MPC for now, bring it back later.
Dependency mapping shows `controller/` is NOT MPC-only — live sim paths
import `controller.{ballistics,target,telemetry,scheduler,plant}` — so
deletion is rejected in favour of:

1. Delete the launch entries for `motion_bridge_node` and the `motor_guard`
   ExecuteProcess (confirmed unconsumed:
   `teensy_bridge_node.py:352-360` — "motor_guard leaves the leg path; its
   :5556 output simply goes unconsumed"; GUI migrated off per
   `main.js:417`). Node files stay (git-recoverable either way).
2. Demote the MPC test battery (15 files, ~370 s serial:
   test_mpc_trajectory 88.6 s, test_solver_failures 65.1 s,
   test_mpc_time_pathologies 47.0 s, …) + motor_guard tests to nightly.
   Lands WITH the Phase 2 runner.
3. Re-frame docs: CLAUDE.md stops calling motor_guard "the safety-critical
   500 Hz control loop" — the Teensy-side MAX_DEVIATION guard is the
   leg-path safety authority in the MVP topology (owner confirmed). Mark
   `run_mpc.py` + AOT-compile sections dormant; fix the stale bridge
   docstring (~:1989) claiming a ZMQ SUB on motor_guard's :5556.
4. Revival = re-add two launch entries + promote the tests. The mpc.py
   internal cleanup (ref-builder extraction, single warm-start-invalidation
   enforcement point — duplication verified at ~1013–1055) is DEFERRED to
   the revival's foundation commit.

## Phase 4 — teensy_link → repo root (LANDED 2026-08-01; hardware-confirmed 2026-08-01)

> **Hardware smoke CONFIRMED, 2026-08-01 23:17.** The operator ran
> `colcon build --packages-select jugglebot`, relaunched, and exercised a
> LEVELLING sequence end-to-end — beyond the disarmed link-up smoke the banner
> asked for. Log evidence (`~/.ros/log/2026-08-01-23-17-27-*/launch.log`):
> `teensy_bridge_node` started and finished cleanly through the repo-root
> `teensy_link` import path; the `[config] freshness check OK` banner fired at
> launch; no motor_guard/motion_bridge entries (dormant chain gone); the only
> anomalies (rclpy `__del__` shutdown tracebacks, `ros2 bag` exit code 2 at
> teardown) are the same pre-existing class seen in the 2026-07-31 pre-change
> log — Foxy shutdown noise, none referencing the bridge or `teensy_link`. Logbook entry flipped to `resolved` accordingly.

`git mv controller/teensy_link teensy_link` — logbook
`2026-08-01-teensy-link-repo-root`. Owner picked **repo root** over
install-into-the-ROS-package: the injection makes the bridge run the LIVE tree,
so a wire-format edit is live at the next relaunch, where installing it would
put every `protocol.py` edit behind a `colcon build` whose omission is silent.
327 references across 134 files; 89 files rewritten. Both PYTHONPATH injections
(`jugglebot_launch.py` ~389-397, `teensy_bridge_launch.py` ~57) keep their
hard-coded repo path per the plan — re-pointed only, and
`teensy_bridge_launch.py`'s superseded "long-term fix: install into the ROS
package" note now records the decision instead of proposing its opposite.

Two things the mechanical rewrite would have missed, both fixed: `protocol.py`
derived the repo root as `../..` from its own file (now `..`, and it has a
`sys.path` fallback, so the break would have surfaced late rather than loudly);
and the bridge **stopped importing CasADi** — the old path executed
`controller/__init__.py` → `mpc`, and the ROS package now has zero
`controller.*` imports. The compat shim `controller/teensy_link.py` is
`sys.modules` **aliasing** and aliases every submodule explicitly (a re-export
would give `RpcError`/`RpcTimeout` two class identities and let a `raise` sail
through the other path's `except`); TEMPORARY, delete after 2026-09.

Gate: `./run_tests.sh --full`, run 2026-08-01 on the Jetson under
`~/Desktop/PDJ_venv/venv`: **parallel 4406 passed, 3 xfailed in 448.43 s;
serial 9 passed, 4409 deselected in 40.52 s; total 494 s; RESULT: PASS**
(exit 0). Wire bytes unmoved — xlang digest
`1383b3fc18dc085c51eba58979ef60a3898b2ff74dc99931926d04c0bc7ceccb` identical
before and after, pin file untouched; `fault_golden.json`'s only diff is a
`_note` path string and a freshly compiled emission still equals the committed
golden.

Deferred out of this phase, deliberately:

- **7 active plan documents** still name `controller/teensy_link`
  (accel-ff-inertia, hand-ball-sensor, learned-ff-residuals,
  leg-gain-tuning-methodology, levelling-frame-contract, mvp-trajectory-bringup,
  teensy-can-offload) — narrative change, per the Phase 6 `controller/demo`
  precedent. The shim keeps every instruction in them working until 2026-09;
  they must be re-pointed before the shim is deleted.
- **4 firmware comments** under `Teensy_code_canbridge/` (`rpc.h:55`,
  `leg_activate.h:34`, `leg_deactivate.h:30`, `leg_interp.cpp:156`) — firmware
  source is out of scope on this branch; comments only, no build or wire impact.
- **The launch PYTHONPATH injection stays hard-coded**, per the bullet above,
  even though `jugglebot_launch.py` now has a worktree-aware `_repo_root()`
  helper next to it. Switching the injection to it is a behaviour change on the
  production launch path that this off-robot stage cannot validate, and
  `teensy_bridge_launch.py` would need the same change to stay consistent.
  Worth doing in a stage that ends at a powered session.

## Phase 5 — config freshness contract (PARTIALLY LANDED 2026-08-01)

Landed: the codegen `--check` drift gate (item 3's parenthetical), item 2 in
full, and item 1 for **one** node — logbook `2026-08-01-config-drift-gate`.
Still owed: item 1 for the remaining nodes and for *effective tuning values*
(delivered is identity only — path + sha256 + mtime — on `teensy_bridge_node`),
and item 3's loader-unification decision. Three deviations from the plan text,
all recorded in that entry: the `--check` **exit code covers the 14 in-repo
destinations only** (`../BallButler` is a separate checkout — it must not be
able to redden this repo's gate or raise its bring-up banner; its drift prints
as `EXTERNAL DRIFT:`); the comparison is **text, not "byte-diff"** (the write
path's `newline=None` emits CRLF on Windows, where a byte compare reports
permanent drift on a clean checkout); and everything **not** checked is
announced, because the first version reported a green `CONFIG FRESH: 7` when
the hardware YAML was absent and 9 of 16 artifacts had silently dropped out.

Production stays **BUILD-FROZEN** (staler-than-expected fails safe;
fresher-than-expected on an actuator path is the dangerous direction —
owner concurs). Confusion is killed by observability, not freshness:

1. Every node logs its effective tuning values + a hash of the config it
   loaded, at boot. **[PARTIAL — `teensy_bridge_node` only, identity not
   values]**
2. Launch-time drift check: compare source `hardware_config.yaml` (+
   generated artifacts) against the installed copies; on mismatch warn
   loudly — "source config differs from installed — run generate_config +
   colcon build". Converts the silent staleness trap into a named prompt.
   **[LANDED — two links reported separately: YAML→repo via `--check`, and
   repo→INSTALLED via byte compare, which is the link `--check` cannot see]**
3. Any future loader unification must preserve per-consumer resolution
   order — `friction_ff_params.py`'s env → ament-share → source-tree order
   is a landed crash fix (2026-06-24) and sits in motor_guard's import
   chain; a source-tree-first unified loader would silently flip
   production freshness. The codegen `--check` drift gate (render in
   memory, byte-diff all delivered copies, one parametrized test) lands
   here too (honest scope: it would NOT have caught the 24608bb stale
   binary — that fix is the pio-clean forcing in Phase 7).
   **[drift gate LANDED; `friction_ff_params.py` UNTOUCHED — owner decision
   still pending]**

## Phase 6 — structural cleanups (sequenced; each independently gated)

**Order constraints (red-team):** (a) sim import-root unification BEFORE
moving controller/demo into sim (else the move adds dual-module-identity
surface); (b) Phase 4 (teensy_link) BEFORE any teensy_bridge_node split.

- [x] One import root for sim/: bare-vs-`sim.*` imports created two module
  objects per file in one interpreter (verified live pre-fix in
  `sim/reload_gate.py` vs `tests/sim/test_reload_gate.py`) — the
  "test patches nothing" class. ~50 mechanical rewrites; the single
  bootstrap must keep all four path roots (repo, sim, ros_ws pkg,
  config/generated). **DONE 2026-08-01** (slice 2) — 179 bare imports across
  71 files converted; one `sim/_paths.bootstrap_paths()`;
  `tests/sim/test_sim_import_style.py` is the enforcement point.
  **Deviation, deliberate**: three roots ship, not four — `sim/` is retired,
  because with it installed a bare import still mints a twin, so "four roots
  survive" and "one module object" cannot both hold. Rationale in
  `logbook/2026-08-01-structure-cleanups`.
- [x] Move `controller/demo/` (2,075 lines) → `sim/juggle_planner/`; six test
  files also import it (not just sim scenarios); replace the hand-copied
  constants at `juggle_optimizer.py:98-102` with real imports. **Gate this
  move with `./run_tests.sh --full`**: three of those six importers
  (`test_demo_juggle_{sim,optimizer,planner}.py`) went `nightly` on
  2026-08-01, so the default gate now sees only
  `test_demo_{sim_playback,timeline,trajectory}.py`. CLAUDE.md's
  `sim/`-path trigger already requires `--full` here; this is the reason.
  **DONE 2026-08-01** (slice 2) — `git mv`, 49 refs across 21 files; the
  seven constants are now imports from `sim.hand.*`, printed bit-identical
  before and after. Still owed: 4 active plan documents name the old path
  (narrative change, deferred); the 9 logbook entries that name it are
  historical record and must NOT be rewritten.
- [x] Dedupe the ROS-clock→perf_counter offset estimator (three copies in the
  catch-timing path; reload variant already drifting). Byte-equivalence
  gate + recorded-sample test; clock injected as a callable. **DONE
  2026-08-01** (slice 2) — `jugglebot/clock_offset.py` (pure);
  `tests/ros/test_clock_offset.py` asserts exact float equality against a
  verbatim transcription of the old code. Two of three copies unified;
  `reload_coordinator_node`'s single-read variant is left with a pointer
  comment, its reconciliation an **open decision**, not an oversight.
- Thin trajectory_node's request-validation handlers into
  `motion/trajectory/requests.py` (snapshot-in/decision-out; the
  mode/streaming state machine is fenced OUT per the REJECT list).
- Split teensy_bridge_node.py along its per-domain test seams (arming
  contract ~1954–2245 explicitly out of scope; one domain per commit; each
  stage needs setup.py packages check + colcon build + installed-copy
  import smoke; NOT while any sitting series is active).
- [x] Bridge test harness extraction (`tests/ros/_bridge_harness.py`; 20 files
  currently import from test_teensy_bridge_node_read); collect-only count
  identical before/after. **DONE 2026-08-01** (slice 2) — 21 importers
  re-pointed, 19 duplicated `_teardown` copies deleted, plus the
  byte-identical `_poll` / `_platform_frame` / `_link_kv` / `_messages`
  helpers; collect-only **4410 → 4410**, the gate this item asked for.
- [x] Generated topic-choreography map + drift-diff test (banner: "Python-node
  graph only — GUI/rosbridge consumers not included"; topic names are not
  all literals, budget introspection under the mocked-ROS conftest).
  **DONE 2026-08-01** (slice 1). `ros_ws/docs/choreography.md`: 75 distinct
  wires / 140 endpoints, pure `ast`. Parked nodes (`motion_bridge_node`,
  `mpc_bridge_node` — Phase 3) render `(not launched)`, cross-checked against
  the launch file in both directions. Only one name in the package is
  non-literal (`param:control_mode_topic`).
- [x] GUI kinematics golden vectors: `stewart-fk.js` is the repo's only true
  duplicate kinematics implementation (hand-ported IK + Newton-Raphson FK),
  currently pinned only by a manual browser page — add JS-vs-Python FK
  golden-vector check runnable in CI. **DONE 2026-08-01** (slice 1). 25 poses
  from the Python IK replayed through the real JS under node, no shim; skips
  (never fails) with no node. `FK_POS_TOL_MM` headroom is derived from a
  residual-tolerance sweep, not one observation.
- [x] Stale-doc sweep remainder + logbook_search hardening (warn-on-skip +
  front-matter validation test — it silently drops malformed entries).
  **DONE 2026-08-01** (slice 1). Contract landed with all three parts:
  `logbook/README.md` (normative), `scan_entries()` (enforcement),
  `tests/sim/test_logbook_front_matter.py` (gate). Still owed:
  `ros_ws/docs/safety.md`'s one re-framing pass (banner-flagged, not
  spot-patched) — it belongs with the Phase 3 dormancy write-up.

**Slice 1 gate** (`./run_tests.sh --full`, run 2026-08-01 on the Jetson under
`~/Desktop/PDJ_venv/venv`): **parallel 4386 passed, 3 xfailed in 446.49 s;
serial 9 passed in 40.18 s; total 493 s; RESULT: PASS** (exit 0). Logbook
`2026-08-01-analysis-safety-nets`.

**Slice 2 gate** (`./run_tests.sh --full`, run 2026-08-01 on the Jetson under
`~/Desktop/PDJ_venv/venv`): **parallel 4398 passed, 3 xfailed in 444.94 s;
serial 9 passed, 4401 deselected in 40.23 s; total 491 s; RESULT: PASS**
(exit 0). Logbook `2026-08-01-structure-cleanups`. Slice 2 covers the four
items marked `[x]` above (sim import root, `controller/demo` move,
clock-offset dedup, bridge harness). **Still open in Phase 6**: the
trajectory_node request-validation thinning and the teensy_bridge_node split.
Note for whoever takes them: this slice's five review-found breaks were all in
runnable surface the suite never executes (a build script, three probes, one
lazily-imported plotter), so a green `--full` is not evidence a tree-wide
mechanical rewrite is safe — see the entry's "Found at review" table for the
checks that are.
- Tracked-media policy (OPEN, owner call deferred): `experimenting/`
  carries ~180 MB git-tracked (1.3 GB on disk, not gitignored — crawled by
  every grep/indexer), `simulations/` ~48 MB. Recommendation:
  `git rm --cached` + gitignore for scratch dirs (files stay on disk but
  leave git backup); keep deliberate artifacts (Circuit Diagrams, CAD)
  tracked. Needs an explicit owner yes because untracked = no git backup.

## Phase 7 — CAN3 residue (software-shaped, hardware verdict untouched)

- [x] ERR_TIMEOUT epidemic recount pre/post bus-role swap from existing bags
  (read-only; `link_status_health_scan.py` pattern; count ERR_BUS_DOWN and
  ERR_TIMEOUT separately — distinct firmware paths). **DONE 2026-08-01** —
  logbook `2026-08-01-err-timeout-recount`. The epidemic **survived** the swap
  (pre 47–52 %, post 4/8 + 4/8) on a bus reading zero wire errors and zero
  TX-gate refusals on all 10513 samples, so it is NOT the CAN3 drive-path
  fault; every failure is `ERR_TIMEOUT`, never `ERR_BUS_DOWN`, and never a
  host-side `RpcTimeout`. Mechanism narrowed to `bus.write() <= 0` — which is
  in open tension with the "lying ack" premise behind
  `_MAX_ARM_DISPATCHES`; the `tx_write_fail` counter below settles it and the
  latch behaviour stays as-is until then. Follow-ups this opened: split
  `tx_write_fail` from `tx_gated` and uplink it (a wire change — needs the new
  MsgType treatment, the payload is exact-size-unpacked), per-send attribution,
  a TX-queue high-water mark, a `link_status` hand-ack-failure counter, and one
  ordinary post-swap reload sitting to tighten the 4/8 interval.
- [ ] The four counter/identity follow-ups above + the fw-identity item below
  — **SOFTWARE DONE 2026-08-02** (unchecked per the Phase 5 PARTIAL
  convention: the reload sitting and the flash are still open) (logbook `2026-08-02-err-timeout-attribution-instrumentation`,
  four commits). Two additive MsgTypes, FW_VERSION 8→9, no PROTOCOL_VERSION
  bump: per-bus `tx_deferred` + `tx_q_hwm` + per-stage hand counters
  (`BRIDGE_TX_DIAG` 0x8D), wire-visible bridge identity warn-never-refuse
  (`BRIDGE_IDENTITY` 0x8E), host-side `hand_traj_acks` row (works against the
  CURRENT bridge), pio staleness guard. **The counter is `tx_deferred`, not the
  `tx_write_fail` named above**: FlexCAN_T4 source (verified) shows
  `write() <= 0` queues the frame into the 64-slot software ring drained by the
  TX-complete ISR — deferral, not drop — which withdraws the recount's
  "never enqueued ⇒ not armed" inference and makes the lying-ack premise
  compatible with the narrowing (latch fence still holds until counters confirm
  on hardware). Step 1 sequence analysis (399 outcomes): random, state bug
  excluded, congestion-consistent. NOT yet done: FW 9 flash + colcon build
  (operator), the bench discriminator sitting
  (`tests/hardware/session_err_timeout_bench.md` +
  `tools/probes/hand_dispatch_ladder.py`),
  and the ordinary reload sitting. CAN-mute pio platform image (971d12c)
  root-cause remains open, untouched.
- Bridge-uptime tracking-lag reboot-isolation experiment (pre-registered;
  calendar cost is real — the degraded cell needs a multi-hour soak).
- Post-repair flash window: wire-visible firmware identity as a **NEW
  message type** (never append fields to an existing frame — old-bridge/
  new-Jetson skew makes exact-size prefix unpacks raise per-frame and the
  message goes dark; new msg_type is ignored cleanly by an old Jetson),
  warn-never-refuse like PLATFORM_FW_VERSION_EXPECTED; pio-clean forcing
  when `udp_protocol.h` is newer than the build dir (the ACTUAL 24608bb
  fix); root-cause the CAN-mute pio platform image (971d12c).
  **[2026-08-02: identity + pio-clean CODE landed, FW 9 NOT yet flashed — see
  the SOFTWARE DONE row above; the flash itself and the 971d12c root-cause
  both remain from this bullet.]**
- The re-plug discriminating probe: DEFERRED by owner (2026-07-31). If ever
  run: it is a flash→probe→flash-back cycle with a three-part revert (both
  bus declarations + ESR1 addresses), not a "60-second" job.
- Bounded FlexCAN retransmit (T3: firmware TX path, real safety surface) —
  only after the CAN3 repair, native-harness first, bench soak.

## Standing coordination rules

- Never run a test suite while another session's suite is running. (Full
  gates now enforce this themselves: `run_tests.sh` takes an flock on
  `/tmp/jugglebot-run_tests.lock` and queues, landed 2026-08-01 after two
  live gate-vs-gate collisions. Scoped runs stay lock-free.)
- Before touching CLAUDE.md / pyproject.toml / run_tests.sh, check the
  working tree for another session's in-flight edits. (2026-07-31: the
  parallelisation session upgraded the ROOT `run_tests.sh` in place —
  `./run_tests.sh` is the blessed gate; there is no `scripts/` copy.)
