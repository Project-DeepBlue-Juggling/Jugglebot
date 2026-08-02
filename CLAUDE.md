# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Jugglebot is a real-time robotics control codebase (MPC planner, MuJoCo simulation, ODrive PID). Incorrect velocity, feedforward, or timing changes can cause dangerous jerky hardware movement. Always verify physics/control implications of changes, not just test passage.

Jugglebot is a Stewart platform robot that catches and throws balls. The codebase has three main subsystems that share a common config layer:

1. **ROS2 hardware stack** (`ros_ws/`) — runs on Jetson Orin Nano (Ubuntu 20.04, ROS2 Foxy, Python 3.8)
2. **MuJoCo simulation** (`sim/`) — standalone Python 3.11+, no ROS2 dependency
3. **MPC controller** (`controller/`) — pure Python with CasADi, imported by both sim and hardware

## Architecture

```
config/hardware_config.yaml  ← single source of truth for all physical parameters
config/generate_config.py    ← generates .py/.h/.js constants → config/generated/ + consumer dirs
controller/                  ← MPC runtime: solver, plant abstractions, telemetry, MPC loop, hardware plant
teensy_link/                 ← can-bridge UDP transport (protocol/client/rpc + the leg-op observers)
run_mpc.py                   ← hardware MPC entry point (uses controller/) — DORMANT, not launched
sim/                         ← MuJoCo simulation (plant/, hand/, ball/, input/, viz/, analysis/)
ros_ws/src/jugglebot/        ← ROS2 package (can/, motion/, tracking/, nodes)
tests/                       ← all tests (ros/, sim/, motion/, hardware/, archived/)
tools/                       ← standalone utilities (tracking_analyzer, nightly_suite.sh)
run_tests.sh                 ← the gate (parallel phase + serial tail; --full adds the nightly tier)
temp/reports/nightly/        ← 04:00 full-tier run: status, latest.md, per-day reports
logbook/                     ← engineering logbook (investigation entries, INDEX.md)
plans/active/                ← in-progress plans and implementation reports
plans/archived/              ← completed or superseded plans
```

**Documentation:**
- `DOCUMENTATION_GUIDE.md` is the single reference for how documentation is organised across the codebase (logbook, plans, `docs/`, subsystem docs, `.claude/`, inline). Read it before creating or editing any markdown artifact, or when you're unsure which layer a piece of information belongs in.

**Engineering logbook & planning:**
- All code changes are logged in `logbook/` — see `logbook/README.md` for the full guide
- **Short form is the default entry** (10–30 lines: what/why + the (date, command, result) verification triple). Escalate to the full investigation form only under the three Discussion triggers below.
- `/investigate` — hardware diagnosis-to-fix pipeline; `/log` — log non-hardware changes; `/logbook` — browse/search entries
- `/archive-plan` — move completed plans from `plans/active/` to `plans/archived/` (with critical review)
- `plans/active/INDEX.md` lists every active plan with a status column; adding or archiving a plan updates it in the same commit (pinned by `tests/sim/test_plans_index.py`)
- Commits carry a `Logbook-Entry: <slug>` trailer. **That trailer is the canonical bidirectional link** between code and logbook: `git blame` → commit → trailer → entry, and `git log --grep "Logbook-Entry: <slug>"` → every commit for an entry. Entries therefore do **not** carry commit SHAs — no `commits:` frontmatter on new entries and no SHA-backfill follow-up commit (convention retired 2026-08-01; historical entries keep theirs).

**Key architectural boundaries:**
- `ros_ws/.../motion/`, `controller/` and `teensy_link/` are pure Python — no ROS2 imports allowed
- ROS2 nodes (`*_node.py`) are thin wrappers; business logic lives in pure-Python modules
- `controller/plant.py` defines `PlantInterface` — implemented by `MuJoCoPlant` (sim) and `HardwarePlant` (real robot via ZMQ IPC)
- IPC between processes uses ZeroMQ PUB/SUB on tcp://localhost:5556 (telemetry) and :5557 (commands), msgpack serialization
- **`teensy_link/` is a top-level package at the repo root, deliberately NOT installed into the ROS package** (moved out of `controller/` 2026-08-01, `plans/active/refactor-2026-07.md` Phase 4). Both launch files inject the repo root on `PYTHONPATH` so `teensy_bridge_node` runs the LIVE tree: a wire-format edit is live at the next relaunch instead of sitting behind a `colcon build` whose omission is silent. Import it as `teensy_link…`; `controller/teensy_link.py` is a `sys.modules`-aliasing shim for the old path, to be **deleted after 2026-09**.
- **Leg-path safety authority is the Teensy-side `MAX_DEVIATION` guard**, in can-bridge firmware. The MVP leg path is `trajectory_node` → :5557 → `teensy_bridge_node` → can-bridge Teensy (which does the 500 Hz interpolation). Nothing on the Jetson is in that safety loop.
- **The MPC chain is parked DORMANT** (2026-08-01, `plans/active/refactor-2026-07.md` Phase 3): `jugglebot_launch.py` no longer starts `motor_guard` or `motion_bridge_node`, and `run_mpc.py` is not launched. `motor_guard.py` was a 500 Hz interpolator + monitor on the *pre-cutover* leg path; it is a parked fallback, not the safety layer — do not describe it as one. All the code, entry points and tests stay; revival = re-add the two launch entries + promote the `nightly`-marked MPC battery. `controller/` is NOT MPC-only, so it is not deleted: live sim paths import `controller.{ballistics,target,telemetry,scheduler,plant}`.

## Environment

On the Jetson, always use the project virtualenv for all Python commands (tests, sim, scripts):
```bash
source ~/Desktop/PDJ_venv/venv/bin/activate
```
The system `python3` (3.8.10) lacks MuJoCo and other project dependencies. The venv at `~/Desktop/PDJ_venv/venv/` has everything installed.

## Commands

### Config generation (run from repo root)
```bash
python config/generate_config.py
```

### MPC solver AOT compile (run on Jetson) — DORMANT
```bash
# DORMANT with the rest of the MPC chain (2026-08-01). Nothing launches the MPC,
# so this is only needed when reviving it. Kept because the revival needs it.
# Required after any change to controller/mpc.py or controller/params.py.
# Produces controller/generated/mpc_gen.so (~15-30s), removing the 27-100ms
# first-solve cold start. Missing → soft fall-back to in-process build;
# stale (hash mismatch) → hard-fail with a clear message.
python controller/generate_solver.py
```

### Simulation
```bash
python sim/main.py [--mpc] [--keyboard] [--pose 0,0,50,0,0,0] [--no-viewer --duration 5]
python sim/main.py --dashboard --mpc --pose 0,0,50,0,0,0   # dashboard on :8082
```

### Hardware MPC (on Jetson) — DORMANT
```bash
# NOT part of bring-up (2026-08-01), and NOT runnable as written until revived:
# HardwarePlant.enable() blocks up to 2 s on motor-feedback telemetry from
# motor_guard's :5556 and then raises, and neither motor_guard nor its feeder
# motion_bridge_node is launched any more. Reviving = re-add BOTH launch entries
# (or start both by hand), THEN stop trajectory_node so run_mpc.py owns the :5557
# funnel (the single-binder interlock makes a conflict loud). It fails closed —
# RuntimeError before any command is published — so a mistaken attempt is safe.
python run_mpc.py --pose 0,0,170,0,0,0 --duration 10   # hold at an active pose
python run_mpc.py [--dashboard]                        # targets from mpc_bridge_node
```

### Tests
```bash
# THE GATE. Parallel phase (4 xdist workers, --dist loadfile) then a serial
# phase for `serial`-marked allocation/timing tests. Deselects `-m nightly`.
./run_tests.sh
./run_tests.sh --full                         # EVERY tier, `nightly` included
./run_tests.sh --hypothesis-profile=ci-deep   # nightly depth (1000; ci-fast=50 per PR)
cat temp/reports/nightly/status               # GREEN/RED from the 04:00 run
tools/nightly_suite.sh                        # run the nightly by hand

# Scoped (bare pytest is still right for iterating on one file)
pytest tests/ros/ -v      # ROS2 node tests (conftest mocks ROS2)
pytest tests/sim/ -v      # MPC + simulation tests
pytest tests/motion/ -v   # kinematics, dynamics, motor_guard

# Hardware test harnesses (standalone scripts, require real robot)
python tests/hardware/free_platform_test.py --test all
python tests/hardware/single_leg_test.py --test all
```

### ROS2 (on Jetson)
```bash
cd ros_ws && colcon build --packages-select jugglebot
source install/setup.bash
ros2 launch jugglebot jugglebot_launch.py
```

### Documentation
```bash
mkdocs serve   # local preview at http://localhost:8000
```

## Workflow Rules

- **Grep before refactoring**: before renaming or refactoring any symbol, grep the entire codebase for all references first. List every file and line number, count total occurrences. After making changes, verify the count drops to zero. A partial find-and-replace is not acceptable.
- **Analyze control-system implications before changes**: before implementing changes to MPC, feedforward, or timing code, analyze the control-system implications first. What happens to the feedforward path? Could this cause discontinuities, oscillation, or timing issues at 40 Hz? Walk through one MPC cycle step-by-step with the proposed change.
- **TodoWrite checklist for multi-file tasks**: for tasks involving changes to multiple files, create a TodoWrite checklist before starting. List every file that needs changes, every test that needs updating, and a final verification step. Check off each item as you complete it. Do not declare the task done until all items are checked.
- **Run the full suite after code changes AND before `git commit`, automatically**: run `./run_tests.sh` after any change to `*.py` or `*.yaml` and again immediately before the commit is written, without being asked. **One run satisfies both obligations when no edit intervenes between the run and the commit**; any later edit invalidates it. (Parallel phase + a serial phase for `serial`-marked tests — **200 s measured 2026-08-01**, down from 478 s pre-tiering; `--full` is 1259 s at ci-deep. Bare `pytest tests/ -q` is ~28 min and runs `nightly` too. `-q` for the gate, `-v` when debugging a failure.) **Use `./run_tests.sh --full` — every tier, `nightly` included — in three cases: (a) before ANY hardware sitting, (b) at plan-phase closure, and (c) pre-commit for any change under `controller/` or `sim/`.** Those are the paths the `nightly` tier covers, so the default gate cannot see a regression you just introduced there. Scoped subsets are fine for iteration; the full suite is the final pre-commit gate. `config/*.yaml` order: edit YAML → `python config/generate_config.py` → stage the regenerated artifacts → run tests → commit. Never commit known-failing code without the user's explicit acknowledgement; report count and result in the same response as the commit. **A "docs-only, so no tests needed" exemption is NOT automatically safe: name the tests that read the path you changed and trace what they actually assert** — never a bare appeal to the file extension, never a mechanism you haven't traced, never coverage inferred from a passing count. Worked example (logbook edits *are* inside the test surface, yet the obvious test misses the two failures you'd most expect it to catch): `logbook/README.md` § "What the logbook tests actually check".
- **Audit multi-document narrative changes**: run `/audit --unstaged` before any commit touching **≥2 narrative `*.md` files** (logbook entries, plans, READMEs) **or any normative doc** (`CLAUDE.md`, `DOCUMENTATION_GUIDE.md`, `controller/REFERENCE_LAYER_CONTRACT.md`, per-plan tuning-methodology docs, **and any document a rule here delegates an obligation to** — currently `logbook/README.md` and `tools/probes/README.md`; the list is open-ended, so if you move an obligation out of this file the receiving doc joins it). Cross-document inconsistencies (swapped values, stale line refs, contradictory headlines) are common in narrative work and the audit catches them reliably. A single logbook entry landing alongside its code — the common case — does not trigger the gate; code-only commits never did.
- **Capture user-corrections as memory, proactively**: when the user manually does a step Claude could have done — pushing after a commit, fixing a typo Claude introduced, running a command Claude should have suggested — ask whether the behaviour should be saved to memory so future sessions don't repeat the omission. Memory lives in `~/.claude/projects/-home-jetson-Desktop-Jugglebot/memory/`, indexed in `MEMORY.md`; follow the format of the existing topic files (`feedback_workflow_discipline.md` is a representative one) and prefer a new section in an existing topic file over a new file — 53 single-topic files is how the layer got compacted on 2026-08-01. A typical ask: *"I noticed you pushed manually after my commit — should I remember to push automatically after future commits on this branch?"* Don't wait for `/remember`.
- **Invite physical-intuition pushback on hardware investigations**: at the start of any hardware-investigation session, and at every framing pivot during one, explicitly tell the user *"if your physical intuition disagrees with my framing, that's load-bearing signal — say so."* User intuition has repeatedly caught framing errors Claude alone missed at conceptual pivots (canonical example: the friction-FF bench arc, `logbook/2026-04-27-friction-feedforward-bench-validation.md` § Discussion). Make it cheap for them to push back; you'll save days.
- **Session start: read the nightly status**: `cat temp/reports/nightly/status` — one line, `GREEN|RED|DEFERRED <counts> <iso-date>`. This is the ONLY delivery channel for the 04:00 full-tier run (owner's choice: no email, no GUI), so an unread RED is an invisible RED. **Anything that is not a fresh GREEN gets surfaced in your first substantive reply**, before other work: RED → list the failures from `temp/reports/nightly/latest.md`; DEFERRED → a live robot session held the box, so no run happened; a date more than ~2 days old → the runner itself stopped; **file missing entirely → the runner was never armed on this box, or `temp/` (gitignored) was cleaned** — do not read absence as "fine". For the last three, check `systemctl --user list-timers jugglebot-nightly.timer` and re-arm from `tools/systemd/README.md`.
- **Check for parallel-session work on the branch**: parallel Claude sessions on one branch are a real workflow here. Before starting any non-trivial task AND immediately before every `git push`, run `git fetch && git status -sb`. If `origin/<branch>` is ahead of local OR the working tree contains modifications you didn't make this session, **pause** and surface what you see before continuing — don't `git pull --rebase` silently, don't push over divergence, don't overwrite an unfamiliar working-tree change. (Treat "file modified by external process" system reminders as the same signal.)
- **Justify design choices by root-cause, not appeal-to-authority**: when surfacing a design alternative to the user (via `AskUserQuestion` or in prose), lead with the **concrete failure modes the choice prevents**, not "the plan says so" / "the contract requires it" / "convention is X". Plans and contracts are shorthand for root causes — restate the root cause at the moment of justification so the user can judge the choice on its merits and push back on the plan if the root cause is wrong. Canonical case (what a root-cause justification looks like, written out as three named regression classes): `logbook/2026-05-11-tier1a-real-solver-failures.md` § "Why T-U-T1a-6 uses `stats()` injection, not real exit codes".
- **Empirical probe before writing tests for specific failure modes / thresholds**: when a test drives a specific failure code (solver exit, watchdog firing, contract violation) OR asserts a threshold (20 mm shift, 500 ms staleness, N consecutive failures), prototype the driver in a probe first, confirm it behaves deterministically on the pinned dependency stack, then document the confirmed recipe in the test docstring AND the phase's logbook. Resist plan-author hedges like "(or whichever sentinel the code surfaces)" — verify against ground truth; two saves from doing so: `logbook/2026-05-11-tier1a-real-solver-failures.md` §§ "Real-driver strategy per exit code" and "Why `Infeasible_Problem_Detected` needs pinned decision bounds". One-off probes go to `/tmp/probe_*.py`, uncommitted; reusable replay/scenario harnesses go to `tools/probes/<name>.py`, committed, outputs under `temp/probes/` — conventions and rationale (`/tmp` references rot at the next reboot) in `tools/probes/README.md`. When in doubt default to `/tmp/` and promote later.
- **Cite test-count claims with the (date, command, result) triple**: any Outcome paragraph, logbook Verification section, or commit message asserting a passing count MUST cite the date of the run, the exact invocation, and the result. Good: *"ci-deep (`pytest tests/ -q --hypothesis-profile=ci-deep`, run 2026-05-11): **1193/1193 pass in 560.98 s**."* Bad: *"Full suite passes at ci-deep (1193/1193, 560.98 s)."* Without the triple the claim is unverifiable from the artefact alone and the audit-reporter flags it as BLOCKING; with it, a future reader can re-run the command or check the date against `git log`.
- **Fix surfaced bugs in the same session when diagnosis is clear**: when a test commit or audit surfaces a real bug AND the diagnosis is clear AND the fix is small/well-scoped, fix it in the same session — next commit, not a deferred end-of-plan obligation. A bug captured as `xfail(strict=True)` is a known unfixed defect; treat it as one. The "production-code changes triggered by tests" rule (separate commit, separate logbook) protects rollback granularity; it does NOT license deferring the fix to a later session. "Diagnosis is clear" means the failure path is fully traceable *in writing*, not "I think I know what's wrong". Defer ONLY when (a) that test genuinely leaves the diagnosis uncertain, (b) the fix needs substantial design work or a stakeholder decision, or (c) the user says to defer — and surface the *intent to fix this session* when you mark the xfail, so a redirect is explicit. Why: latest-acceptable-moment scheduling accumulates xfails and loses the original session's cognitive context. Canonical case: `logbook/2026-05-11-tier1c-input-fuzz-bugfix.md` § "The 'fix in same session' lesson".
- **Checkpoint before sinking effort, not after**: domain and physical context lives with the user. Before spending real effort, do a *brief inline sync* — one or two lines stating the approach and the key assumption it rests on — and wait for a quick confirm. Triggers: (a) before committing to a non-trivial approach, (b) when leaning on a physical/system assumption the user could correct, (c) when deviating from the agreed plan. The pushback rule above makes it cheap for the user to *object*; this one makes you *proactively surface* the decision before it is expensive to unwind. For root-cause hunts, **pre-register the fallback and the decision criterion that flips you to it** ("if the kinematic scales aren't stable across two runs, ship the measured offset") so the pivot is planned, not a sunk-cost spiral. Canonical cost of skipping: the 2026-06-10→18 BallButler temporal-accuracy chapter burned multiple sessions on effort sunk before a sync — including a "ball-inertia" feedforward coded and flashed that made the lag *worse* (44→56 ms). Story: `logbook/2026-06-18-temporal-accuracy-resolved-fractured-solution.md` in the **BallButler** repo, not this one.
- **Proactively suggest session boundaries**: end each working session as a unit — logbook entry closed, suite green, the logical unit committed — and don't let edits or red tests accumulate across sessions. *Offer* to close out and start fresh at natural boundaries (unit done + committed + logged, context getting long, topic about to switch subsystems); keep it a one-line offer the user can decline. Canonical cost of skipping: the BallButler chapter accumulated 43 uncommitted files and 68 failing tests across four sessions, all surfacing at commit time — each one-file-cheap to fix when introduced.

## Engineering Philosophy

These principles are normative. They reflect how successful work on this
codebase has actually been done — deviate from them only with clear reason.

- **Climb one level of abstraction before you fix.** Ask whether the bug is the
  *symptom of a class of failures*, and enumerate the class explicitly. One-off
  fixes solve today's problem; contracts close the whole class forever.
  Reference pattern: the K1–K6 reference-feasibility contract
  (`controller/REFERENCE_LAYER_CONTRACT.md`) — one root cause drove 14 failure
  modes, and the fix was a single invariant with a single enforcement point.
- **Favour contracts over patches.** A contract has three parts: a normative
  document stating the invariant, one canonical enforcement point in code,
  and a test that fails if the invariant is violated. On a recurring class of
  failures, land all three together. Resist "just relax this one invariant
  for this one case" — that is how contracts die, and every future failure of
  that class is on you.
- **Rigor is an investment, not an overhead.** When the user asks for "the
  most robust fix" or insists on a full enumeration before patching, honour
  that even if you see a faster path. Don't settle for the first working fix
  if a systematic one is in reach.
- **Push back when evidence contradicts a hypothesis.** If you commit to an
  explanation and the next data point doesn't fit, abandon the hypothesis —
  don't rescue it. Confidence in an explanation is not evidence for its
  correctness. When the user pushes back on a hypothesis, that is
  load-bearing signal, not friction: say "you're right, that hypothesis
  doesn't survive the new data" and reset.
- **The logbook is the most valuable artefact in this repo.** Future sessions
  (human or AI) reconstruct the arc from these entries, so every change gets
  one. **Short form is the default** — 10–30 lines: what changed, why, and the
  (date, command, result) verification triple. **Escalate to the full
  investigation form, with a real Discussion section, when any of these hold**:
  (a) a hypothesis was withdrawn or reframed mid-investigation, (b) a
  non-obvious tradeoff was accepted, (c) the chosen approach beat another
  reasonable approach for reasons future-readers wouldn't infer from the code
  alone. **A hardware investigation almost always hits at least one trigger; a
  docs or plumbing change almost never does** — when in doubt, ask whether a
  future session could reconstruct *why* from the code and the commit alone,
  and if not, write the Discussion. Under those triggers it is non-negotiable — *why this
  approach over others*, *what was ruled out*, *what tradeoffs were accepted* —
  and it is written *before* the Fix section, so the Fix's rationale is already
  in front of you. A Discussion that feels tedious is usually the one that
  saves the most time later; skipping it under a trigger is the single most
  common way this project's investigations lose institutional memory.
- **Protect the contracts you've landed.** The normative documents
  (`controller/REFERENCE_LAYER_CONTRACT.md`, `DOCUMENTATION_GUIDE.md`,
  per-plan tuning methodologies) encode expensive lessons. Under pressure to
  ship, resist both the "just this once" carve-out and the "this case is
  special" exemption. If a contract genuinely needs to change, change the
  document first and ripple it through enforcement — don't silently drift.

## Critical Conventions

- **Python 3.8 compatibility** in `ros_ws/`: always use `from __future__ import annotations` for modern type hints
- **Config codegen**: after editing `hardware_config.yaml` or `protocol_config.yaml`, run `python config/generate_config.py` — never hand-edit generated files
- `tests/conftest.py` sets up shared paths; `tests/ros/conftest.py` injects mock ROS2 modules for Windows
- **New tests are parallel by default**: `./run_tests.sh` runs xdist with `--dist loadfile`, so a file's tests share one worker. Write tests that hold under that: bind **ephemeral** ports (`:0`, then read back the assigned port — see `tests/teensy_link/conftest.py`, `tests/sim/_zmq_test_harness.py`) and write files via `tmp_path`, never a fixed shared path. Reach for `@pytest.mark.serial` **only** when the test measures a process-global resource (`tracemalloc` heap growth, GC events, wall-clock timing) whose baseline is corrupted by concurrent load; needing a port or a temp file does not qualify. `serial` costs the whole suite serial time — keep the marked set small (9 marked, **0** of them in the default gate after the 2026-08-01 carve-out below, i.e. the gate's serial phase is currently empty and `run_tests.sh` says so explicitly). **Marker vocabulary is `serial` + `nightly` and nothing else** (`slow` was deleted 2026-08-01; `--strict-markers` makes a new one a hard error). Demote to `nightly` on CONTENT, never on runtime: research/demo characterization and operationally dormant code only — the hardware-safety surface (teensy_link wire bytes, firmware natives, ROS nodes, `motion/`) is never demoted. One carve-out (2026-08-01): three `tests/motion/` tests that *measure the machine* rather than *assert a contract* — motor_guard's loop-jitter and IPC-latency characterizations and the friction-FF allocation budget. motor_guard's safety logic (E-stops, NaN rejection, workspace limits, staleness) is unmarked and per-commit. See the `nightly` marker's definition in `pyproject.toml` for the full contract.
- Stewart platform uses mixed mm/rad units; Jacobian is normalized by `plat_radius_mm` before numeric work (condition number ~3-8 at home, not the raw ~450)
- Jacobian convention: J maps `[vx,vy,vz,wx,wy,wz]` to `[q_dot_1..q_dot_6]`
- Force decomposition: `f = J^{-T} * W` (use `np.linalg.solve(J.T, W)`), NOT `J^T * W`
- All movements must use profiled trajectories — never command step position changes
- CAN encoding must match the can-bridge firmware (`Teensy_code_canbridge/`) and the generated `protocol_config` constants: negate, scale by appropriate value, int16, clamp (`can_node.py` is deleted; the firmware is the encoding authority)
- All runtime artifacts live under `temp/` (not `/tmp/`, not under `sim/`). Both `sim/main.py` and `run_mpc.py` write telemetry CSVs and companion `.log`/`.png`/`_report.html` files to `temp/logs/`. Cross-session comparison HTML reports go to `temp/reports/`. Nothing under `sim/` should accumulate runtime output.
