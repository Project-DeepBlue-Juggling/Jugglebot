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
run_mpc.py                   ← hardware MPC entry point (uses controller/)
sim/                         ← MuJoCo simulation (plant/, hand/, ball/, input/, viz/, analysis/)
ros_ws/src/jugglebot/        ← ROS2 package (can/, motion/, tracking/, nodes)
tests/                       ← all tests (ros/, sim/, motion/, hardware/, archived/)
tools/                       ← standalone utilities (tracking_analyzer)
logbook/                     ← engineering logbook (investigation entries, INDEX.md)
plans/active/                ← in-progress plans and implementation reports
plans/archived/              ← completed or superseded plans
```

**Documentation:**
- `DOCUMENTATION_GUIDE.md` is the single reference for how documentation is organised across the codebase (logbook, plans, `docs/`, subsystem docs, `.claude/`, inline). Read it before creating or editing any markdown artifact, or when you're unsure which layer a piece of information belongs in.

**Engineering logbook & planning:**
- All code changes are logged in `logbook/` — see `logbook/README.md` for the full guide
- `/investigate` — hardware diagnosis-to-fix pipeline; `/log` — log non-hardware changes; `/logbook` — browse/search entries
- `/archive-plan` — move completed plans from `plans/active/` to `plans/archived/` (with critical review)
- Commits include `Logbook-Entry: <slug>` trailers for traceability from `git blame` to logbook entries

**Key architectural boundaries:**
- `ros_ws/.../motion/` and `controller/` are pure Python — no ROS2 imports allowed
- ROS2 nodes (`*_node.py`) are thin wrappers; business logic lives in pure-Python modules
- `controller/plant.py` defines `PlantInterface` — implemented by `MuJoCoPlant` (sim) and `HardwarePlant` (real robot via ZMQ IPC)
- IPC between processes uses ZeroMQ PUB/SUB on tcp://localhost:5556 (telemetry) and :5557 (commands), msgpack serialization
- `motor_guard.py` is the safety-critical 500 Hz control loop; MPC runs at 40 Hz in a separate process

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

### MPC solver AOT compile (run on Jetson)
```bash
# Required after any change to controller/mpc.py or controller/params.py.
# Produces controller/generated/mpc_gen.so (~15-30s). Eliminates the 27-100ms
# cold-start penalty on the first real-time solve. Missing → soft fall-back to
# in-process build; stale (hash mismatch) → hard-fail with a clear message.
python controller/generate_solver.py
```

### Simulation
```bash
# Basic
python sim/main.py
python sim/main.py --mpc --pose 0,0,50,0,0,0
python sim/main.py --keyboard --mpc
python sim/main.py --no-viewer --duration 5

# With dashboard (http://localhost:8082)
python sim/main.py --dashboard --mpc --pose 0,0,50,0,0,0

# Docker (GPU required)
cd sim && docker compose up
```

### Hardware MPC (on Jetson)
```bash
# Hold at active pose
python run_mpc.py --pose 0,0,170,0,0,0 --duration 10

# Production: receive targets from ROS2 via mpc_bridge_node
python run_mpc.py

# With live dashboard
python run_mpc.py --dashboard
```

### Tests
```bash
# All automated tests (from repo root)
pytest tests/ -v

# By category
pytest tests/ros/ -v                          # ROS2 node tests (conftest mocks ROS2)
pytest tests/sim/ -v                          # MPC + simulation tests
pytest tests/motion/ -v                       # motion module tests (kinematics, dynamics, motor_guard)
pytest tests/ros/test_can_node.py -v          # single file
pytest tests/sim/test_mpc_static.py -v        # single file

# Nightly hypothesis run (max_examples=1000, ~10 min wall-clock; ci-fast=50 is the per-PR default)
pytest tests/ -q --hypothesis-profile=ci-deep

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
- **Run pytest after code changes AND before `git commit`, automatically**: run `pytest tests/ -q` after any change to `*.py` or `*.yaml`, and run it again immediately before the commit gets written — both as part of the same response, without waiting for the user to ask. Scoped subsets (e.g. `pytest tests/motion/`) are fine for iteration but the full suite is the final pre-commit gate. For `*.yaml` changes under `config/` the order is: edit YAML → `python config/generate_config.py` → stage the regenerated artifacts → run tests → commit. Never commit known-failing code without the user's explicit acknowledgement; report the count and result in the same response as the commit. (Use `-q` consistently for the pre-commit gate; `-v` is fine for debugging when a test fails.)
- **Audit multi-document narrative changes**: any commit that touches a logbook entry, a plan document, the README, CLAUDE.md, or a normative `*.md` (DOCUMENTATION_GUIDE, REFERENCE_LAYER_CONTRACT, per-plan tuning methodologies, etc) must be preceded by `/audit --unstaged`. Cross-document inconsistencies (swapped values, stale line refs, contradictory headlines) are common in narrative work and the audit catches them reliably. Code-only commits don't need this gate.
- **Capture user-corrections as memory, proactively**: when the user manually does a step Claude could have done — pushing after a commit, fixing a typo Claude introduced, running a command Claude should have suggested — that is signal. Ask whether the behaviour should be saved to memory so future sessions don't repeat the omission. Memory lives in `~/.claude/projects/-home-jetson-Desktop-Jugglebot/memory/` as one short file per topic, indexed in `MEMORY.md` — follow the existing pattern (see `feedback_investigate_gating.md` and `feedback_auto_push_after_commits.md` for the format). A typical ask: *"I noticed you pushed manually after my commit — should I remember to push automatically after future commits on this branch?"* Don't wait for the user to invoke `/remember`.
- **Invite physical-intuition pushback on hardware investigations**: at the start of any hardware-investigation session, and at every framing pivot during one, explicitly tell the user *"if your physical intuition disagrees with my framing, that's load-bearing signal — say so."* User intuition has repeatedly caught framing errors that Claude alone missed at conceptual pivots in this project's hardware investigations (see the friction-FF bench arc in `logbook/2026-04-27-friction-feedforward-bench-validation.md`'s Discussion section for the canonical example). Make it cheap for them to push back; you'll save days.
- **Check for parallel-session work on the branch**: parallel Claude sessions on the same branch (especially `refactor`) are a real workflow now that `/next-phase-prompt` makes them cheap to spin up. Before starting any non-trivial task AND immediately before every `git push`, run `git fetch && git status -sb` (or equivalent). If `origin/<branch>` is ahead of local OR if the working tree contains modifications you didn't make this session, **pause** and surface what you see to the user before continuing — don't `git pull --rebase` silently, don't push over divergence, and don't overwrite an unfamiliar working-tree change. The cost of a 2-second `fetch` is negligible; the cost of clobbering another session's commits or landing a merge conflict on a shared branch is high. (System reminders may also surface "file modified by external process" notices — treat those as the same signal.)
- **Justify design choices by root-cause, not appeal-to-authority**: when surfacing a design alternative to the user (via `AskUserQuestion` or in prose), lead with the **concrete failure modes the choice prevents**, not with "the plan says so" / "the contract requires it" / "convention is X". Plans and contracts are shorthand for root causes — at the moment of justification, restate the root cause so the user can judge the choice on its actual merits and push back on the plan/contract if the root cause is wrong. (Canonical example: when justifying `T-U-T1a-6`'s monkey-patch approach during the 2026-05-11 sad-path session, Claude's first answer was "the plan said so"; the user demanded concrete failure modes the approach prevents — the resulting framing was substantially better and surfaced tradeoffs the appeal-to-plan would have hidden. See `logbook/2026-05-11-tier1a-real-solver-failures.md`'s Discussion section.)
- **Empirical probe before writing tests for specific failure modes / thresholds**: when adding a test that drives a specific failure code (real solver exit, hardware watchdog firing, contract violation) OR asserts a threshold (e.g., 20 mm walk-forward shift, 500 ms staleness, N consecutive failures), prototype the driver empirically in a probe script before writing the test. **Where the probe lives depends on its scope.** One-off exploratory probes (testing a single recipe, then discarded) go to `/tmp/probe_*.py` and are NOT committed — the original rule. Reusable replay or scenario harnesses (e.g., production-faithful replay of a recorded hardware CSV, generic fail-MPC drivers that future investigations will re-use) live at `tools/probes/<name>.py`, committed, with a header docstring linking the logbook entry and test that use them, an entry in `tools/probes/README.md`, and outputs written to `temp/probes/` (gitignored — same pattern as `temp/logs/`). When in doubt, default to `/tmp/` and promote to `tools/probes/` later if the probe gets reused — the cost of one extra commit later is lower than the cost of cluttering `tools/` with abandoned scripts. **Why this distinction matters**: the 2026-05-20 warm-start-deadlock investigation's offline probe lives at `/tmp/mpc_dvfs_probe/` and that entry's "Related" section references its findings files; those paths sit on volatile `/tmp` storage that any reboot will wipe (or `tmpwatch` will eventually expire), so the references are one power-cycle away from rotting. Reusable harnesses get committed so the references stay live and future collaborators (especially AI sessions) can run the recipe, not just read prose about it. Confirm the recipe produces the expected behaviour deterministically on the pinned dependency stack. Document the empirically-confirmed recipe in the test docstring AND in the phase's logbook (mirror Phase 1 Tier-1a's "Real-driver strategy per exit code" table in `logbook/2026-05-11-tier1a-real-solver-failures.md`). Resist the trap of trusting plan-author hedges like "(or whichever sentinel the code surfaces)" — verify against ground truth. Two concrete saves from this discipline so far: (a) Working Note #1's claim that `max_cpu_time=1e-6` causes a CasADi init error was empirically false on our pinned stack — the entire 1e-6 to 1e-2 range returned clean `Maximum_CpuTime_Exceeded`; (b) the pre-existing "flip `ubg<lbg`" infeasibility trick in `tests/sim/test_mpc_static.py` routes through the CasADi pre-validation exception path, not the `Infeasible_Problem_Detected` IPOPT classifier — opposite of what the test name implied.
- **Cite test-count claims with the (date, command, result) triple**: any Outcome paragraph, logbook Verification section, or commit message that asserts a passing test count MUST cite the date the run happened, the exact pytest invocation used, and the result — not just the count. Example good: *"ci-deep (`pytest tests/ -q --hypothesis-profile=ci-deep`, run 2026-05-11): **1193/1193 pass in 560.98 s**."* Example bad: *"Full suite passes at ci-deep (1193/1193, 560.98 s)."* Without the triple, the claim is unverifiable from the artefact alone — the audit-reporter will flag it as a BLOCKING finding (as it did during the 2026-05-11 Plan 2 Phase 0 closure). The triple lets a future reader re-run the same command and compare, or check the date against git log to confirm the run happened in the session that wrote the artefact.
- **Fix surfaced bugs in the same session when diagnosis is clear**: when a test commit (or audit) surfaces a real bug AND the diagnosis is clear AND the fix is small/well-scoped, address it in the same session — not as a deferred end-of-plan obligation. The "production-code changes triggered by tests" rule (separate commit, separate logbook) protects rollback granularity; it does NOT license deferring the fix to a later session. A real bug captured as `xfail(strict=True)` is a known unfixed defect — treat it that way. "Diagnosis is clear" operationally means: the corruption/failure path is fully traceable in writing (as in Phase 3's logbook Discussion → "Bug surfaced" trace), not "I think I know what's wrong." Defer ONLY when (a) the diagnosis is genuinely uncertain by that test, (b) the fix requires substantial design work or stakeholder discussion, or (c) the user explicitly says to defer. The default is fix-now: same session, next commit. Latest-acceptable-moment scheduling (e.g. "before plan archival") turns known bugs into scheduled work, accumulates xfails, and loses the cognitive context the original session had — the next session has to reload the trace, re-verify the analysis, and re-design the fix. Surface the *intent to fix in the same session* at the moment you mark the xfail — give the user the chance to redirect ("defer this one because…") explicitly rather than implicitly. (Canonical example: 2026-05-11 Plan 2 Phase 3's `T-U-T1c-7-bug` was originally xfail-deferred to "before archival"; the user flagged this and asked for the same-session fix. Result: `logbook/2026-05-11-tier1c-input-fuzz-bugfix.md` landed minutes after the test commit, before Phase 4 started, with full cognitive context still loaded.)

## Engineering Philosophy

These principles are normative. They reflect how successful work on this
codebase has actually been done — deviate from them only with clear reason.

- **Climb one level of abstraction before you fix.** When you find a bug, pause
  and ask whether it is the *symptom of a class of failures*. Enumerate the
  class explicitly. Cheap one-off fixes solve today's problem; contracts that
  close the whole class solve every related problem forever. The K1–K6
  reference-feasibility contract (`controller/REFERENCE_LAYER_CONTRACT.md`)
  is the reference pattern — one root cause drove 14 distinct failure modes,
  and the fix was a single normative invariant with a single enforcement
  point, not 14 patches.
- **Favour contracts over patches.** A contract has three parts: a normative
  document stating the invariant, one canonical enforcement point in code,
  and a test that fails if the invariant is violated. When you identify a
  recurring class of failures, land all three together. Resist suggestions
  to "just relax this one invariant for this one case" — that is how
  contracts die, and every future failure of that class is on you.
- **Rigor is an investment, not an overhead.** When the user asks for "the
  most robust fix" or insists on doing a full enumeration before patching,
  honour that request even if you see a faster path. The compounding returns
  of structural thinking are enormous; the short-term cost is real but
  bounded. Don't settle for the first working fix if a systematic one is
  in reach.
- **Push back when evidence contradicts a hypothesis.** If you commit to an
  explanation and the next data point doesn't fit, abandon the hypothesis —
  don't rescue it. The confidence of an explanation is not evidence for its
  correctness. When the user pushes back on one of your hypotheses, treat
  that as load-bearing signal, not friction. Say "you're right, that
  hypothesis doesn't survive the new data" and reset.
- **The logbook is the most valuable artefact in this repo.** Every
  investigation entry must have a real **Discussion** section — not just
  what changed, but *why this approach over others*, *what was ruled out*,
  *what tradeoffs were accepted*. Future sessions (human or AI) reconstruct
  the full arc from these entries. When a Discussion section feels tedious
  to write, that's usually the one that'll save the most time later.
  **The Discussion section is non-negotiable when any of the following
  hold**: (a) a hypothesis was withdrawn or reframed mid-investigation,
  (b) a non-obvious tradeoff was accepted, (c) the chosen approach beat
  another reasonable approach for reasons future-readers wouldn't infer
  from the code alone. Write the Discussion *before* the Fix section, so
  the reasoning is fresh and the Fix's rationale is already in front of
  you. Skipping Discussion in these cases is the single most common way
  this project's investigations lose institutional memory.
- **Protect the contracts you've landed.** The normative documents
  (`controller/REFERENCE_LAYER_CONTRACT.md`, `docs/DOCUMENTATION_GUIDE.md`,
  per-plan tuning methodologies) encode expensive lessons. Under pressure
  to ship, resist both the "just this once" carve-out and the "this case
  is special" exemption. If a contract genuinely needs to change, change
  it in the document first and ripple the change through enforcement —
  don't silently drift.

## Critical Conventions

- **Python 3.8 compatibility** in `ros_ws/`: always use `from __future__ import annotations` for modern type hints
- **Config codegen**: after editing `hardware_config.yaml` or `protocol_config.yaml`, run `python config/generate_config.py` — never hand-edit generated files
- `tests/conftest.py` sets up shared paths; `tests/ros/conftest.py` injects mock ROS2 modules for Windows
- Stewart platform uses mixed mm/rad units; Jacobian is normalized by `plat_radius_mm` before numeric work (condition number ~3-8 at home, not the raw ~450)
- Jacobian convention: J maps `[vx,vy,vz,wx,wy,wz]` to `[q_dot_1..q_dot_6]`
- Force decomposition: `f = J^{-T} * W` (use `np.linalg.solve(J.T, W)`), NOT `J^T * W`
- All movements must use profiled trajectories — never command step position changes
- CAN encoding must match `can_node.py`: negate, scale by appropriate value (check protocol_config), int16, clamp
- All runtime artifacts live under `temp/` (not `/tmp/`, not under `sim/`). Both `sim/main.py` and `run_mpc.py` write telemetry CSVs and companion `.log`/`.png`/`_report.html` files to `temp/logs/`. Cross-session comparison HTML reports go to `temp/reports/`. Nothing under `sim/` should accumulate runtime output.
