# PROMPT — 2026-07-25 self-toss anomaly fixes (workflow-orchestrated build run)

Kickoff prompt for a fresh Opus 5 session on the Jetson
(`/home/jetson/Desktop/Jugglebot`, branch `mvp-trajectory-bringup`). This run
implements, adversarially reviews, and software-validates the four fix plans
produced by the 2026-07-25 self-toss anomaly investigation, and hands the
operator a single ordered hardware checklist at the end.

**Operator authorization (2026-07-25): orchestrate this run with the Workflow
tool — multi-agent orchestration is explicitly opted in.** Build each phase with
fresh implementer agents and put every phase through an adversarial review panel
before its commit. Use Opus 5 agents throughout.

## Mission

Implement, in the execution order below, every **software** phase of:

1. `plans/active/levelling-frame-contract.md` — Phases 0–3 (items 1–2)
2. `plans/active/hand-command-continuity.md` — Phases 0–4 (items 3–6)
3. `plans/active/fk-convergence-tolerance.md` — Phases 0–1 (item 7)
4. `plans/active/catch-reach-degenerate-overshoot.md` — Phases 0–2 (item 8)

Each phase: implemented, independently reviewed, full-suite tested, logged,
committed, pushed. Update each plan's **Implementation Phase Summary** status
column as its phases land.

**Do not run the hardware phases** (`levelling-frame-contract` Phase 4,
`hand-command-continuity` Phase 5). Instead accumulate a single operator
runbook at `tests/hardware/session_anomaly_fixes.md` — see § Hardware handoff.

## Read first (in order)

1. `CLAUDE.md` — workflow rules and engineering philosophy are **normative**.
2. The four plans above. Their Context sections contain the entire measurement
   evidence base (no logbook entry exists yet — that is a deliberate operator
   decision, do not create one for the *diagnosis*; you still write one per
   implementation phase).
3. `DOCUMENTATION_GUIDE.md` — before creating or editing any markdown.
4. `controller/REFERENCE_LAYER_CONTRACT.md` — the reference pattern for the
   contract-shaped fixes in plans 1 and 2.
5. The code spine: `ros_ws/src/jugglebot/jugglebot/trajectory_node.py`,
   `catch_coordinator_node.py`, `motion/trajectory/planner.py`,
   `motion/ik_solver.py`, `Teensy_code/Trajectory.h`, `Teensy_code/Teensy_code.ino`,
   `sim/hand/trajectory.py`.
6. `plans/active/PROMPT-single-ball-toss-software-run.md` and
   `.claude/workflows/mvp-phase-runner.js` — the established phase-runner shape.
   Reuse it; do not invent a new one.

## Do not re-litigate

The diagnosis is settled and operator-reviewed. The measurements quoted in the
plans (commanded-pose FK reconstructions, the 43 mm hand dip, the ±0.05 %
un-levelled control session, the clean 11.08° reload pre-tilt, the axis-ratio
match to four significant figures) are the evidence base. Do not re-derive them
to "check" — spend the effort on the fixes.

**One exception, explicitly required:** `catch-reach-degenerate-overshoot.md`
Phase 0 step 1 *must* re-verify the "single plan install" inference from raw bag
data. That plan flags it as the weakest link in the original reasoning. Treat it
as an open question, not a finding.

## Execution order

Dependencies and file contention drive this, not plan numbering.

| # | Phase | Why here | Status |
|---|---|---|---|
| 1 | `fk-convergence-tolerance` P0 → P1 | Smallest, fully independent. Use it to shake down the implement→review→finalize pipeline before anything load-bearing. | **DONE** |
| 2 | `levelling-frame-contract` P0 | Enumeration. **Hard STOP at its gate** — the plan requires operator review of the enumeration before P1. | **DONE** (gate cleared, `3365ac8`) |
| 3 | `levelling-frame-contract` P1 → P2 | The headline fix. P2 is the one that closes the operator-visible tilt. | **DONE** — `aea7b49` |
| 4 | `hand-command-continuity` P0 | Probe + mirror verification. **Possible STOP** (see forks). | **DONE** |
| 5 | `hand-command-continuity` P1 → P2 | Closes the operator-visible dip and the throw truncation. | **DONE** — `6179a88` |
| 6 | `catch-reach-degenerate-overshoot` P0 | Analysis only; writes `tools/probes/` + a findings doc, touches no production code. Safe to interleave. | **DONE** — `a680298` |
| 7 | `hand-command-continuity` P3, then `levelling-frame-contract` P3 | **Both touch files a parallel session was editing** — gate on § Contention. | **DONE** — `94fe817`, `e36d60d` |
| 8 | `hand-command-continuity` P4 | Firmware. Implement + test; **never flash.** | **DONE in source, NOT FLASHED** — `5369fc2` |
| 9 | `catch-reach-degenerate-overshoot` P1 → P2 | P1's gate may escalate this plan's priority; if it does, stop and re-plan with the operator. | **DONE** — `407154f`, `30e9723`; P3 (seat rate → `0.0`, operator decision) added in-run, `e58ed89` |
| 10 | Emit `tests/hardware/session_anomaly_fixes.md` | Consolidated operator checklist. | **DONE** — accumulated across phases |
| 11 | **Run close-out** — 9b review repairs + whole-file runbook coherence pass | Added in-run. The runbook accumulated append-only across eleven phases, so rows written early contradicted code that landed later; a top-to-bottom operator would have aborted a healthy sitting on at least four of them. | **DONE 2026-07-27** — see § Run close-out — Outcome |

## Workflow shape — one Workflow invocation per phase

Do **not** try to pipeline all fourteen phases in one script. One workflow per
phase, three or four agents each: this stays inside the session's ≤15-agent
guideline, and — more importantly — it puts you back in the loop between phases
so you read each result before committing to the next. Chain across turns.

Per phase:

1. **Implement** (1 agent). Given: the plan file, the specific phase, the read-first
   list, and the hard rules below. Writes code + tests. Does **not** commit.
2. **Review** (2–3 agents in parallel, read-only, distinct lenses). Use
   `agentType: 'audit-reporter'` where it fits. Lenses that matter here:
   - *correctness/physics* — does the change do what the plan's root cause requires?
     For plan 1: is the correction applied exactly once, to requests only? For
     plan 2: does the timing window actually hold at the flight-band extremes?
   - *contract integrity* — is there one enforcement point, or did N call sites
     appear? Does the bypass/drift-guard test genuinely fail without the fix?
   - *regression surface* — what else consumes the thing being changed? (Plan 2 P3's
     park-band consumers are the canonical trap.)
   Prompt reviewers to **refute**, and default to "not proven" when uncertain.
3. **Finalize** (1 agent, or do it yourself). Applies confirmed findings, then in
   this order:
   - `git fetch && git status -sb`; pause and surface on any divergence or
     unfamiliar working-tree change.
   - For YAML changes: `python config/generate_config.py`, stage the regenerated
     artefacts.
   - **Full `pytest tests/ -q` in the foreground.** Never overlap two suites.
   - `/audit --unstaged` if the phase touches any plan / logbook / normative `.md`.
   - Write the phase's logbook entry (`/log` for non-hardware changes). A real
     **Discussion** section is mandatory wherever a hypothesis was withdrawn, a
     non-obvious tradeoff accepted, or the chosen approach beat another reasonable
     one.
   - `git diff --cached --stat` between `add` and `commit`.
   - Commit with a `-F` message file (never `-m` with backticks) carrying
     `Logbook-Entry: <slug>` and the `Co-Authored-By: Claude Opus 5 (1M context)`
     trailer; then backfill the SHA into the logbook entry as a small follow-up
     commit.
   - Fetch-guarded `git push`.
   - Update the plan's phase-table status row.

**Commit every phase before starting the next.** A workflow that dies mid-phase
leaves uncommitted code in the shared tree, and salvaging that has cost this
project a session before.

## Hard rules (non-negotiable)

- **Never run robot-actuating commands.** The operator runs anything that moves
  the machine, flashes firmware, or launches the ROS graph. You may run read-only
  inspection (`git`, `pytest`, bag reads, `ros2 topic list`) freely.
- **Serialize the code-writing agents on the shared tree.** Only read-only agents
  (reviewers, analysts) run in parallel. Do not use `isolation: 'worktree'` for
  phases that touch `ros_ws/` — the colcon install tree and the shared Jetson make
  worktree isolation more hazardous than serialization.
- **Full suite is the pre-commit gate**, run after any `*.py`/`*.yaml` change *and*
  again immediately before the commit, in the same response. Report the count and
  result with the commit. Cite passing-test claims as the (date, command, result)
  triple, or the audit will flag it.
- Hot-loop allocation-budget tests are order/load-flaky under the full suite. If
  one fails, rerun it isolated; a pass isolated does not block the commit — say so
  explicitly rather than silently ignoring it.
- `ros_ws/` is Python 3.8: `from __future__ import annotations` for modern hints.
  No ROS2 imports in `motion/` or `controller/`.
- Grep before refactoring: list every reference with file:line, count them, and
  verify the count reaches zero afterwards. Plan 1 P0 and plan 2 P3 both depend on
  this being done properly.
- Empirical probe before any test that asserts a threshold or drives a specific
  failure mode. One-off probes → `/tmp/probe_*.py`, uncommitted. Reusable replay
  harnesses → `tools/probes/<name>.py`, committed, with a header docstring linking
  the plan and consuming test, a `tools/probes/README.md` entry, and outputs to
  `temp/probes/`. Plan 2 P0 and plan 4 P0 both produce the reusable kind.
- Never weaken or `xfail` a test to reach green. If a test must change, that is a
  STOP.
- If a phase surfaces a real bug with a clear, small diagnosis, fix it in the same
  phase (separate commit, separate logbook entry) rather than deferring it.
- State the deployment requirement in every phase report: `ros_ws/` or config
  changes ⇒ `colcon build --packages-select jugglebot` **+ relaunch** (the launch
  runs the installed copy); firmware ⇒ **flash**, operator-run.

## STOP conditions — ask the operator, do not guess

- **`levelling-frame-contract` P0 gate**: the plan requires operator review of the
  ingest-surface enumeration before P1. Stop and present it.
- **Contention** (see below) still live when reaching item 7.
- **`hand-command-continuity` P0**: `sim/hand/trajectory.py` is not a faithful
  mirror of `Trajectory.h`. This invalidates P4's test gate — stop and re-plan.
- **`hand-command-continuity` P3**: any park-band or parked-top window is tight
  against the new prime rev. A wrong park band means a kind-0 stroke dispatched
  off-band, which `toss_sequencer` documents as a physical hazard. Safety fork.
- **`hand-command-continuity` P4**: always stop before flashing. Implement, test
  in the sim mirror, transcribe, xref-test, then hand off.
- **`catch-reach-degenerate-overshoot` P1**: if the mechanism is general to the
  C2-replan-to-fixed-arrival pattern rather than specific to the near-degenerate
  case, it is a live risk on the shipping reload path. Stop and re-prioritise.
- Any change that would alter commanded motion magnitudes, the park-band
  thresholds, or the kind-3 clobber path (the only un-arm mechanism the Teensy
  offers — a pre-release SAFE_ABORT depends on it).
- Any test that would have to be weakened.

## Decide and document — reversible forks, do not block on these

Record the decision and its reason in the phase's logbook entry.

- **Plan 1 P1**: normative home for the C-LEVEL-1 contract —
  `controller/REFERENCE_LAYER_CONTRACT.md` vs a new `ros_ws/docs/levelling_frame.md`.
- **Plan 1 P2**: what happens to an in-flight plan when a new `gravity_offset`
  arrives. The plan states the default (live plan keeps its frame; next install
  picks up the new correction) — adopt it unless you find a reason not to.
- **Plan 2 P1**: where the stroke-end instant comes from. The plan recommends
  deriving `t_dec` from the announcement's `initial_velocity` through **one shared
  `motion/` helper**, because the stroke model already exists twice
  (`Trajectory.h`, `sim/hand/trajectory.py`) and a third copy will drift. Reject
  the fixed-conservative-delay option — it is a magic number that mis-sizes at the
  flight-band extremes.
- Probe placement when genuinely ambiguous: default `/tmp`, promote later.

When you justify any of these to the operator, lead with **the concrete failure
mode the choice prevents**, not "the plan says so".

## Contention — check this before you start

A parallel session was editing these files as of 2026-07-25, and two phases need
them:

| File | Needed by |
|---|---|
| `reload_coordinator_node.py` | plan 1 P3 |
| `ros_ws/src/jugglebot_interfaces/action/Toss.action` | plan 1 P3 (comment only) |
| `config/hardware_config.yaml` | plan 2 P3 |
| `motion/trajectory/toss_release.py` | not needed, but in the same tree |

First action of this run: `git fetch && git status -sb` plus `git log --oneline -5`.
If those files still show as modified by work you did not do, or `origin/` is
ahead, **do not touch them and do not rebase**. Run items 1–6 (which need none of
them), then stop and report before item 7. Note that the plans were written
against a tree where `Toss.action` had `flight_time_s` while
`tests/hardware/session_phase8_toss_hardware.md` documents `throw_height_m` — so
that interface may already have changed underneath. Verify before editing.

## Hardware handoff

Build `tests/hardware/session_anomaly_fixes.md` as you go — append each phase's
validation needs as that phase lands, rather than writing it at the end. Model it
on `tests/hardware/session_phase8_toss_hardware.md`. It must contain, in the order
the operator will execute them:

- prerequisites (`colcon build` + relaunch; any firmware flash, called out
  separately and explicitly);
- the exact copy-pasteable commands, with the trace recorder and rosbag enabled;
- per-check PASS / ABORT criteria as **numbers**, not adjectives. The two headline
  ones from the plans:
  - plan 1: commanded `rx` (FK of `/leg_setpoint_echo`) flat to **±0.05°** across
    the whole goal, matching the un-levelled 15:04:35 baseline; tracker catch error
    **< 10 mm** against today's measured 16 mm; a pre-`level` toss returns
    `REJECTED_NOT_LEVELLED`;
  - plan 2: **no dip** — hand position monotonic to `x3` then flat until the catch
    descent, peak overshoot past 9.9594 rev under 0.1 rev, no negative velocity
    between release and the catch descent; no
    `Not enough time for smooth-move` on the Teensy serial;
- which analysis command turns each capture into a verdict (the plan-2 P0 probe is
  the instrument for the dip; FK of `/leg_setpoint_echo` for the tilt);
- a reminder that tracker verdicts still read MISSED on real catches, so catches
  are judged by eye as well as by `outcome`;
- a reminder to reboot the can-bridge Teensy before the session, and that
  `levelling_complete` is per-boot so a manual `level` is always required;
- for each check, which plan and phase it validates, so a failure routes back to
  the right place.

## Reporting

After each phase, report in ≤10 lines: what landed, the test triple, the commit
SHA, any decide-and-document fork you resolved and why, and what is next. At the
end of the run, one summary: phases landed, phases blocked and why, the operator
checklist path, and any residual risk you would not sign off on.

If a background agent reports completion, verify the work is genuinely finished
(files present, tests actually run) before treating it as done — background tasks
have fired "completed" mid-work in this project before.

---

## Run close-out — Outcome

**Landed 2026-07-27**, commits `2dacc3a` (close-out) and `2fa13ed`
(`CLAUDE.md`, split out so `git blame` on the pytest rule points at a process
decision rather than at a runbook coherence pass). Logbook:
`logbook/2026-07-27-anomaly-run-closeout.md`.

`pytest tests/ -q`, run 2026-07-27 on the Jetson in the project venv:
**3947 passed, 3 xfailed in 1383.73 s (23:03)**, exit 0.

The delta against the `1e78f3f` baseline (`pytest tests/ -q`, run 2026-07-27:
**3943 passed, 3 xfailed in 1399.35 s**) is **+4 passed**, and the prediction that
it would be zero was wrong — the baseline was taken *before* the implementer's
test repairs. The `+4` is accounted for exactly and is entirely parametrisation:
four `@pytest.mark.parametrize('seat_rate', (0.0, _SEAT_RATE_RADPS))` decorators,
each turning one test into two legs. Collected items go `3946 → 3950`, which
`pytest tests/ -q --collect-only` confirms. The xfail count stayed at **3**; no
test was weakened, skipped or deleted (zero xfail/skip markers added, assert
counts 71→75 / 83→88 / 455→458, no test function removed or renamed).

**What the close-out was for.** Item 10's runbook was written append-only, one
phase at a time, and was never read end-to-end until now. Doing so surfaced one
defect class repeated four times — *a criterion that fires on correct behaviour* —
plus a fifth found only by building an instrument the runbook had been citing
without owning:

- `§ CHECK CCATCH-3`'s `peak off the park` moves `0.292407°` on a healthy
  post-fix capture, 2.9× its own `> 0.10°` ABORT (F1).
- `§ PASS / ABORT per throw` row 3 said `hard abort at > 10.5 rev` where
  H4.4/H4.5 say `10.60` — a `10.55` reading had two incompatible responses, one
  E-STOP-grade.
- Standing rule 2 told the operator to `level` after the very relaunch CHECK LG-3
  needs un-levelled, converting LG-3's ABORT into a false positive.
- Run-sheet `HAND-1` said "any row ABORTs" where `HAND-4` makes row 4 REPORT on a
  braking-prelude toss — same capture, same row, opposite verdicts.
- **`§ CHECK LVL-4`** gated parked mocap tilt at `±0.10°` with no probe in the
  repo reading `/rigid_body_poses`. The reader written to close that gap measured
  the parked *uncorrected* Platform at `0.087°` off the `Base` body while the
  inclinometer read `0.7787°` off gravity at the same instant — so `Base` is not a
  plumb line and a correctly-levelled platform reads LVL-4's old ABORT verbatim.
  Demoted to REPORT; LVL-3 keeps the gate.

`ZSEAT-2` — the run's only genuinely open experiment — also had an unscored
verdict band (`7/12 = 0.583` satisfies neither `≥ 0.63` nor `≤ 0.58`), found
independently by two review lenses. It is now scored as a **rate** with an
explicit INCONCLUSIVE branch, in the runbook *and* in
`catch-reach-degenerate-overshoot.md`'s Phase-4 gate cell so the two cannot drift.

### Deferred operator handoff

**No software work remains in this run.** Everything outstanding is the operator's
powered sitting, and `tests/hardware/session_anomaly_fixes.md` § THE RUN SHEET is
the single authority for it — seven stages, every row with numeric PASS/ABORT, the
analysis command that turns each capture into a verdict, and the plan+phase a
failure routes to. Three deployments are required and **one of them is silent**:
`colcon build --packages-select jugglebot_interfaces jugglebot` (both packages —
`TrajectoryStatus.msg` gained a field), **a Platform Teensy flash** of
`Teensy_code.ino` (`5369fc2`), and no config regeneration.
See § DEPLOYMENT MATRIX.

> **Amended 2026-07-27.** The parenthetical above originally read *"the board
> carries no `FW_VERSION`, so a skipped flash is indistinguishable from the
> expected PASS"*. That is no longer true. At the operator's request, a phase added
> after this run closed — `hand-command-continuity` **Phase 6** — gave the Platform
> Teensy a `FW_VERSION` and a report path (bytes 5-6 of the 0x6E0 RobotState reply
> it already sends), so a skipped flash now reads `0 (PRE-VERSIONING)` on
> `link_status/platform_fw_version` and logs `PLATFORM_FW_CHECK: FAIL`. Run-sheet
> row **FW-1**. It WARNS and never refuses (`ros_ws/docs/platform_fw_version.md`),
> so the operator must still run the row — but the flash is no longer silent, and
> `RobotState.msg` gaining two fields is a second reason the two-package build is
> mandatory.

Two items are deferred out of this run with reasons recorded in the runbook's
§ RESIDUAL RISK: `sim/plant/mujoco_plant.py`'s stale `9.858` rev hand prime (item
11 — a `sim/` production change plus a test change, its own logical unit), and
whether `makeSmoothMove` should get a second, higher arrest acceleration limit
(item 7 — an envelope decision the coming sitting does not need answered).
