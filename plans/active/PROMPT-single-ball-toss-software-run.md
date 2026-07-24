# PROMPT — single-ball-toss software build run (unattended, workflow-orchestrated)

Kickoff prompt for a fresh Fable 5 session on the Jetson
(`/home/jetson/Desktop/Jugglebot`, branch `mvp-trajectory-bringup`). The
operator is away for several hours; this run builds and critically reviews the
software portion of the single-ball toss system unattended.

**Operator authorization (2026-07-24): orchestrate this run with the Workflow
tool — multi-agent orchestration is explicitly opted in.** Build each phase
with fresh implementer agents and put every phase through an adversarial
review panel before its commit.

## Mission

Implement `plans/active/single-ball-toss.md` Phases **0, 1, 2, 4**
SOFTWARE-complete, in that order, each phase reviewed, tested, committed, and
pushed. Phase **3** is *prep-only*: build the trace-capture harness and write
its runbook, but do **not** execute the live-launch trace (operator-run).
Phase **5** (hardware) is untouched. Update the plan's phase table as phases
land.

## Read first (in order)

1. `plans/active/single-ball-toss.md` — authoritative scope, architecture,
   `Toss.action` spec, tiering, gate design.
2. `plans/active/reload-action-catch-latch.md` — the action pattern being
   mirrored; its latch/`trajectory_node` safety notes are normative.
3. `logbook/2026-07-24-phase8-kickoff-bb-branch-merge.md` — what the
   2026-07-24 merge landed and the design rationale (merge-not-port,
   action-not-service, tiering, kinematic-release gate).
4. The spine: `ros_ws/src/jugglebot_interfaces/action/Reload.action`,
   `reload_sequencer.py`, `reload_coordinator_node.py`,
   `catch_coordinator_node.py`, `trajectory_node.py` (arm_catch latch).
5. `sim/reload_gate.py` — the production-in-the-loop gate pattern to clone.

## Locked design decisions (operator-approved 2026-07-24 — do not re-litigate)

- `Toss.action` goal/result/feedback fields exactly as specced in the plan.
- `catch_position` is STOW-relative platform frame; ONE tested conversion
  point to global for the self-`ThrowAnnouncement` (the z double-add lesson).
- Throw = `traj_type=0` + tracker-driven catch via the latch-gated
  `catch_coordinator` path. NOT `traj_type=2`.
- Platform open-loop through flight; hand dispatches use telemetry-verified
  ladders (the ack lies both ways — never blind re-dispatch).
- Sim gate throw = kinematic release + seeded release noise; the
  contact-physics variant is a non-gating diagnostic column.
- Tier 8b returns `REJECTED_TIER` until Phase 4 lands.
- Coordinator hosting: measure the shared-client surface first; >70%
  duplication → extend `reload_coordinator_node`, else a new node. Document
  the measurement either way.

## Hard rules

- venv for ALL python: `source ~/Desktop/PDJ_venv/venv/bin/activate`.
- Full `pytest tests/ -q` green before EVERY commit; never overlap two suites;
  allocation-budget tests are order-flaky — re-run isolated before judging a
  failure real.
- `ci-deep` (`pytest tests/ -q --hypothesis-profile=ci-deep`) at Phase 2 and
  Phase 4 exits; cite every suite claim with the (date, command, result)
  triple.
- NO hardware actuation, NO `jugglebot_launch.py`, NO binding UDP 5005/5006,
  no powered-ODrive assumptions anywhere.
- `ros_ws/src` changes: every relevant doc/logbook mention carries
  "`colcon build --packages-select jugglebot` + relaunch" guidance (launch
  runs the installed copy).
- New interfaces (`Toss.action`): register exactly as `Reload.action` is
  (interfaces package build files) and extend the `tests/ros` mock conftest
  the same way Reload's tests consume it.
- Written control-system analysis BEFORE touching `trajectory_node` /
  `arm_catch` control flow: walk one full toss cycle (arm → throw dispatch →
  flight hold → catch reach → seat → disarm → recenter) and check for command
  discontinuities at every seam, per CLAUDE.md.
- Empirical-probe rule: any test asserting a specific failure code or
  threshold gets a probe first (`/tmp` one-offs; `tools/probes/` if reusable).
- Commits: message via `-F` file, no backticks, `Logbook-Entry:` trailer,
  `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`; push after each
  commit; `git fetch && git status -sb` before every push — stop on
  divergence.
- Per phase: `/audit` the phase diff before its commit; fix-and-land clear
  BLOCKING findings (and WARNINGs with an obvious safe fix) and document them
  in the phase logbook entry (autonomous-runner policy); logbook entry +
  INDEX row per phase.
- **Do not touch or stage** `logbook/2026-07-24-phase7-fourth-sitting-openloop-telemetry-ladders.md`
  or `tests/hardware/session_phase7_reload.md` if they still carry
  uncommitted changes — that is the operator's in-flight addendum. Known-benign
  at kickoff: the untracked `.claude/workflows/*.js` runner scripts — leave
  them alone. Any other unfamiliar working-tree change: leave it alone, note
  it in the report.

## Phase notes

- **Phase 0** includes the 4 INDEX row backfills listed in the plan
  (read each entry before summarizing it; match the INDEX row format
  exactly), the `sim/JUGGLE_DEMO.md` banner, and the bb-led plan note.
- **Phase 1**: verify (with a test) whether `catch_coordinator`'s correlation
  path is thrower-agnostic for `thrower_name='jugglebot'`.
- **Phase 2**: the gate sweep runs headless; JSON report to `temp/reports/`;
  quote the pass table in the phase logbook entry.
- **Phase 3 (prep-only)**: harness + runbook under `tests/hardware/`
  (mirroring `session_phase7_reload.md` conventions); the harness implements
  the plan's trace-only ball-evidence waiver (bench affordance, unset in every
  powered-session runbook) plus the un-waived `REJECTED_NO_BALL` short trace;
  mark the plan's Phase 3 as PREPARED (operator execution pending).
- **Phase 4**: port `tilt_to_throw` per the plan's grep-before-refactor note;
  map the ±100 mm directional asymmetry in the gate sweep and record the
  envelope in the logbook entry.

## Stop conditions (stop, write the report, do NOT guess)

- Any change that would alter `Toss.action`'s goal/result fields.
- Anything requiring hardware, a live launch, or powered ODrives.
- A safety-relevant ambiguity in the latch / `trajectory_node` interplay.
- Origin ahead of local at push time, or unfamiliar modifications to tracked
  files beyond the two named operator files.

## Deliverables

- Phases 0, 1, 2, 4 committed + pushed individually; Phase 3 harness+runbook
  committed; suite green per commit; ci-deep triples at Phase 2/4 exits.
- One logbook entry + INDEX row per phase.
- `plans/active/single-ball-toss.md` phase table updated as phases land.
- A closing run summary in the final logbook entry: what landed, what was
  deliberately not done (Phase 3 execution, Phase 5), every audit
  BLOCKING/WARNING that was fixed-and-landed, and anything a fresh reader
  needs before the next hardware sitting.
