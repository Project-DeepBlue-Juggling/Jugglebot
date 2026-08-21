export const meta = {
  name: 'mvp-phase-runner',
  description: 'Autonomously implement one mvp-trajectory-bringup phase SOFTWARE-complete (hardware sessions deferred to the operator) — fresh Opus 4.8 agent per phase, full pytest gated, auto-commit+push, decide+document at reversible forks, stop at safety forks',
  whenToUse: 'Advance plans/active/mvp-trajectory-bringup.md one or more software phases unattended; pass {phases:[N,...]} (falls back to PHASES_TO_RUN).',
  phases: [{ title: 'Phase runner' }],
}

const PLAN = 'plans/active/mvp-trajectory-bringup.md'
const BRANCH = 'mvp-trajectory-bringup'
const FORMAT_PRECEDENT = 'logbook/2026-06-29-canbridge-phase0-native-harness.md'
const BASELINE = 'pytest tests/ -q, 2026-07-08 (post Phase-6 + audit fixes, bf5b46e): 2223 passed, 1 xfailed in 561.11s'

// Per-phase hard gate. For this plan, "gate" = the operator hardware session the
// autonomous run CANNOT do (deferred to the bench), or — for Phase 6 — the sim
// gate the agent MUST run itself.
const GATES = {
  1: 'HARDWARE SESSION (DEFERRED) — arm-at-ACTIVE + 120 s hold via the new path. Implement ALL Phase 1 software + tests, AND write the operator protocol file tests/hardware/session_phase1_hold.md + the read-only probe tools/probes/traj_stream_probe.py (with a tools/probes/README.md row). Hardware validation happens when the operator returns to the bench.',
  2: 'HARDWARE SESSION (DEFERRED) — waypoint move battery + one loud-rejection demo at the default low limits. Implement all Phase 2 software/tests + the operator protocol (extend the Phase 1 protocol pattern).',
  3: 'HARDWARE SESSION (DEFERRED) — spacemouse flight (gentle fly, saturation shove, mid-flight unplug). Implement all Phase 3 software/tests + protocol. TWO ORCHESTRATION PREREQUISITES from the Phase 2 audit (read the plan Phase 3 prerequisite line + the Phase 2 logbook Audit-fixes section): (1) full validate() costs ~377 ms on this Jetson — the per-tick follower replan CANNOT run it; design the follower gate to low-single-digit ms (vectorise the sampling chain, decimate/skip the per-sample condition-number SVD, and/or a reduced follower-specific check) while keeping the full gate for service-path plans — this is a design decision you own, document it; (2) build a duration-stretched always-valid graceful-stop primitive (a stop that lengthens its horizon until it passes the gate) and use it for STANDBY-exit mid-move (replacing the Phase-2 catch-and-complete fallback) and follower input-loss.',
  4: 'MOSTLY BENCH (DEFERRED) — implement ONLY the small Phase 4 code (shaping.py lean heuristic + per-call override, diagnostics peak tracking, the /diagnose per-move-peaks extension, tests/hardware/traj_ramp_battery.py) + the per-step ramp protocol doc. The ramp itself is operator work; do NOT invent limit values. INHERITED CONSTRAINTS: shaping runs BEFORE validate (the gate must always see the shaped plan — this ordering is the canonical invariant); the lean per-call override on GoToPose.srv is an interface change (colcon gate applies); the tilt lever arm is POSITIVE 1.66 mm/deg of cup travel per degree and cup-height-derived — follow the plan tilt_geometry port note, define the compensation sign convention explicitly, and pin it with a geometry test (the sign was stated backwards in early exploration docs; do not trust prose, verify against Jugglebot-bb/sim/juggle_tilt.py:63-75).',
  5: 'HARDWARE SESSION (DEFERRED) — timed-move accuracy (±25 ms, mocap-measured) + rejection demos. Implement all Phase 5 software/tests + protocol, including the catch_coordinator feedback-topic swap (preserve blacklist semantics). INHERITED CONTEXT (read the Phase 2-4 logbook Audit-fixes sections): (a) TimedTarget.srv response mirrors GoToPose\'s STRING code convention (documented Phase-2 deviation from the plan\'s int32); (b) the Phase-2 BUSY restriction on mid-plan supersede was explicitly temporary, "lifted by Phase 3/5" — Phase 5\'s hardware test REQUIRES a mid-plan superseding timed target with a C2 replan, so you must OWN the supersede design: the full analytic gate costs ~0.4-1 s on the service path while the emitter streams on (the exact TOCTOU class the Phase-2 BLOCKING finding closed via the install-continuity guard) — options include seeding from a predicted install-time state, reusing the fast validate_follow-style gate for the supersede path, guard+retry, or a hybrid; decide, implement, document the fork with its failure modes; (c) _move_seq now bumps on EVERY non-follower install (Phase-4 audit fix) — respect that in any diagnostics wiring; (d) the perf_counter clock-domain conversion must live at ONE point in trajectory_node (plan Architecture); (e) ci-deep is DUE at the end of this phase (deferred from Phase 2): run `pytest tests/ -q --hypothesis-profile=ci-deep` as an ADDITIONAL gate after the normal full suite, cite its own triple.',
  6: 'SIM GATE — RUN IT YOURSELF (no hardware needed; MuJoCo 3.2.3 is in the venv; run headless, seeded; commit the JSON gate report). The full 20-run reload gate per the plan MUST be executed. LIGHT-SCOPE RULE (operator-approved): if the hand-contact criteria cannot be met after TWO distinct, documented diagnosis attempts, land the port + harness + geometry (everything except the contact/hold criteria) with the open questions precisely documented, and return landed with open_questions populated. Do not spiral. INHERITED CONTEXT (read the Phase 2-5 logbook Audit-fixes sections first): (a) PREMISE — real hardware catches are ALREADY smooth with the current Platform-Teensy arm-and-forget hand; the old Jugglebot-bb sims (sub-tick velocity-matched hand commands) are the janky side — your job is to make the sim REPRODUCE the hardware-proven arm-and-forget behaviour (use the sim/hand/trajectory.py mirror with realistic arm latency), NOT to tune hardware configs; (b) tilt-to-receive must be encoded IN the pose trajectory (plain QuinticSegments from build_catch), NEVER via the LeanShaper wrapper — validate_follow raises TypeError on shaped plans by design; own the gate-choice fork for build_catch (per-announcement planning can afford the analytic gate; pre-freeze supersedes arrive at announcement rate, not 40 Hz) and document it; (c) the catch z-convention is the Phase-5 +JB_OP_DEFAULT_ACTIVE_Z_MM lift (MPC-offset → STOW-relative), hardware-UNVERIFIED — use one convention consistently in harness + build_catch and keep errors loud; (d) the reach-freeze is now settle-bounded with FROZEN feedback (Phase-5 audit fix) — repeated catches in one session must work (Phases 8/9 depend on it; exercise ≥2 sequential catches in the harness); (e) diff-audit the six diverged sim support files (sim/plant/mujoco_plant.py, sim/model/jugglebot.xml, sim/ball/manager.py + __init__.py, sim/ball_butler/sim.py, sim/hand/trajectory.py, sim/main.py) — port only juggle-required changes; (f) check ballistics_bc overlap with controller/ballistics.py before writing new math; (g) the gate report MUST publish the required leg vel/acc/jerk limits back into the plan Phase 4 section (the operator ramp targets depend on them).',
  7: 'HARDWARE SESSIONS 7a/7b/7c (DEFERRED) — implement Reload.action, reload_sequencer.py + reload_coordinator_node.py, the BallButlerThrow.srv point-target extension, and the announcement→correlation→coordinator integration test against recorded bags (rosbags exist under ~/Desktop/rosbags; if none is suitable, build the test from synthesized messages and say so). Write the staged 7a/7b/7c operator protocols. INHERITED CONTEXT (read the Phase 5+6 logbook entries incl. Audit-fixes sections first): (a) INTEGRATION GAP you must close — trajectory_node\'s catch/dynamic_target path still routes through Phase-5 build_timed (reach only); swap it to Phase-6 planner.build_catch (tilt-to-receive from the announced arrival VELOCITY in DynamicTargetCommand.target_vel, tilt-through-seat, quiescent hold) while preserving the settle-bounded freeze + FROZEN feedback semantics — without this swap a hardware reload catches WITHOUT tilt; (b) the catch z-convention (+JB_OP_DEFAULT_ACTIVE_Z_MM lift) is hardware-UNVERIFIED — the 7a aim-only protocol must verify the QTM-world frame AND this z-convention before any ball flies; (c) preconditions per the plan reload design (BB heartbeat IDLE ∧ ball_in_hand via bb/reload with RELOADING→IDLE wait, mocap fresh, streaming on, CATCH mode set by operator); REJECTED_BB on CANT_MAKE_LEAD; throw_delay_s default 3.0 ≥ BB\'s 2.5 s lead floor; (d) contact quality has NO sim criterion (capture-model limitation, Phase-6 audit) — 7b\'s two-consecutive-bounce-out abort is the operative guard, keep it in the protocol; (e) SECONDARY, SEPARABLE item (own commit; stop-and-document if it balloons past ~a dozen test updates): port sim/hand/trajectory.py CATCH_VEL_RATIO 0.9→0.6 (config/firmware truth, HIGH-flagged Phase-6 open question), reconcile the existing hand tests, and re-run the seeded nominal reload gate refreshing the committed evidence in logbook/artifacts/ (expect vel-match numbers to shift; note the delta).',
}

const PHASE_RESULT = {
  type: 'object',
  additionalProperties: false,
  required: ['status', 'summary'],
  properties: {
    status: { type: 'string', enum: ['landed', 'blocked-decision', 'blocked-red', 'error'],
      description: 'landed = software complete + full suite green + committed + pushed (hardware gate deferred). blocked-decision = stopped at a safety-relevant/expensive-to-unwind fork. blocked-red = could not get the suite green. error = unsafe to proceed (dirty tree, branch divergence, etc.).' },
    summary: { type: 'string', description: '3-6 sentences: what was implemented + the verification result with the (date, command, result) pytest triple.' },
    commits: { type: 'array', items: { type: 'string' }, description: 'short SHAs landed + pushed.' },
    gate_handoff: { type: 'string', description: 'The EXACT deferred operator procedure (protocol file path + one-line summary), or for Phase 6 the sim-gate result.' },
    decisions_made: { type: 'array', items: { type: 'string' }, description: 'Every reversible design fork you decided under the decide+document policy: "fork — choice — why" (also recorded in the logbook Discussion).' },
    open_questions: { type: 'array', items: { type: 'string' }, description: 'Questions needing the operator (physical intuition / bench data). Empty if none.' },
    blocked_reason: { type: 'string', description: 'If blocked-*: the precise fork/failure. For blocked-decision, state the options and why it must not be auto-picked.' },
  },
}

function phasePrompt(n) {
  return [
    `You are implementing **Phase ${n}** of ${PLAN}, FULLY AUTONOMOUSLY (the operator is away from the bench and cannot answer questions). Repo root: /home/jetson/Desktop/Jugglebot, branch ${BRANCH}. This is a real-time robotics motor-control codebase — incorrect velocity/feedforward/timing changes can cause dangerous jerky hardware movement when the operator later runs your code on the robot. Rigor over speed.`,
    ``,
    `## Read first (confirm every cited file:line against ground truth before coding)`,
    `1. ${PLAN} — the WHOLE plan: the "### Phase ${n}" detail section, EVERY "## Architecture" subsection it references, the sim-gate table, and "## Notes for Collaborators" (every bullet is binding).`,
    `2. The most recent mvp-trajectory-bringup logbook entry (find via logbook/INDEX.md; for Phase 1 there is none yet — use ${FORMAT_PRECEDENT} as the format precedent: mirror its frontmatter shape, Discussion depth, and (date, command, result) test-count discipline).`,
    `3. CLAUDE.md — "Workflow Rules" and "Engineering Philosophy" in full; they are normative and override defaults.`,
    `4. The seam files your phase touches (the plan cites exact file:line references — verify them; they were audited 2026-07-07 but code may have moved).`,
    ``,
    `## Environment (gotchas that have burned prior sessions)`,
    `- ALWAYS \`source ~/Desktop/PDJ_venv/venv/bin/activate\` before python/pytest commands (system python3.8 lacks MuJoCo/deps).`,
    `- BUT run colcon in a shell WITHOUT the venv (ROS2 Foxy builds against system python 3.8). If you touch ros_ws/src/jugglebot_interfaces/** (new .srv/.msg/.action + CMakeLists.txt) or setup.py/launch files, \`cd ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot\` MUST succeed. pytest does NOT need colcon (tests/ros/conftest.py mocks rclpy), but the build gate is still mandatory when interfaces change.`,
    `- ros_ws python is 3.8: \`from __future__ import annotations\` in every new module; motion/** must stay pure Python (no ROS imports, no repo-root imports).`,
    `- After editing config/hardware_config.yaml: new sections ALSO need an HW_SECTIONS row in config/generate_config.py (a bare regenerate silently emits nothing) — then \`python config/generate_config.py\` and stage ALL regenerated artifacts; regenerate again to prove the tree stays clean (determinism check).`,
    `- Do NOT launch ROS, run run_mpc.py, or execute anything robot-actuating. Read-only probes and sim runs are fine.`,
    `- controller/ is DORMANT — read it, COPY from it where the plan says (hermite/feasibility -> motion/trajectory/quintic.py), never import it from ros_ws code and never modify it.`,
    ``,
    `## Scope & fork policy (operator-approved for this run)`,
    `Implement ONLY the SOFTWARE portion of Phase ${n}, up to its gate:`,
    `    GATE: ${GATES[n] || '(read the plan)'}`,
    `- DECIDE + DOCUMENT: for reversible software forks (naming, equivalent implementations, test structure), make the call yourself, record "fork — choice — why (concrete failure modes, not appeal-to-plan)" in the logbook Discussion, and list it in decisions_made. The pipeline must not stall on reversible forks.`,
    `- HARD STOP (blocked-decision): safety-relevant forks (anything that changes what the robot could physically do at the bench: envelope/limit semantics, arming order, fault handling, staleness behaviour) or forks expensive to unwind (wire/protocol changes, cross-subsystem contracts). State the fork + options; do NOT auto-pick.`,
    `- The emitter/pump contract is load-bearing: every frame the emitter can produce MUST be accepted by a real SetpointPump instance (test this, don't assume it). All platform motion flows through planner.py -> feasibility.validate(); never add a side-channel motion path.`,
    `- Before justifying any design choice, restate the concrete failure mode it prevents. For anything touching command timing or feedforward, walk through one 40 Hz emitter tick step-by-step first.`,
    ``,
    `## Mandatory gates BEFORE committing (non-negotiable)`,
    `- \`pytest tests/ -q\` — FULL suite, 0 failed. Baseline before Phase ${n}: the figure in the latest mvp logbook entry (currently: ${BASELINE}). Cite YOUR run's (date, command, result) triple; explain any net count change in the logbook. Known order/load-flaky under full suite: test_hot_loop_allocation_contract and test_t3b_h4_on_post_solve_allocates_within_budget — if one fails, re-run it ISOLATED; isolated-pass = not a regression (note it, don't block). NEVER run two pytest suites concurrently.`,
    `- colcon build gate when interfaces/setup/launch changed (see Environment).`,
    `- If you cannot get the suite green, return blocked-red with the failing test + output. Do NOT commit known-failing code.`,
    ``,
    `## Documentation (mirror the format precedent exactly)`,
    `- New logbook entry logbook/<today's date>-mvp-phase${n}-<slug>.md: frontmatter (type: feature, date, status: resolved, phase: "${n}", related_plan, files_changed, subsystem, tags) + Summary / Motivation / Design / Implementation / Verification (a (date,command,result) triple for EVERY count) / Discussion (REAL — why this approach over alternatives, what was ruled out, tradeoffs, every decide+document fork; non-negotiable) / Open Questions / Related. Add the topmost row to logbook/INDEX.md.`,
    `- Update ${PLAN}: the Phase ${n} summary-table row to "CODE COMPLETE (hardware deferred)" (Phase 6: "SIM GATE PASSED" or "LANDED LIGHT — open questions"), and add an **Outcome** paragraph to the "### Phase ${n}" detail with commit SHAs, the test-count triple, and the exact deferred operator handoff.`,
    ``,
    `## Self-audit before committing`,
    `Read your OWN full diff. Check: cross-document consistency (logbook vs plan vs INDEX — no contradictory numbers); every test-count claim carries its triple; codegen deterministic (re-run generators -> clean tree); no stale file:line citations; no backticks anywhere in commit-message text.`,
    ``,
    `## Commit + push (auto, to ${BRANCH})`,
    `- Logical units; write each message to a file and \`git commit -F\` (NEVER -m with backticks). Code commits end with a \`Logbook-Entry: <slug>\` trailer + \`Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>\`.`,
    `- Between add and commit: \`git diff --cached --stat\` to verify exactly what is staged.`,
    `- Do NOT put commit SHAs in logbook frontmatter (convention RETIRED 2026-08-01; the Logbook-Entry: <slug> trailer on every commit is the canonical bidirectional link). Plan Outcome paragraphs may still cite SHAs.`,
    `- Before pushing: \`git fetch && git status -sb\`. If origin/${BRANCH} is ahead of local, do NOT push — return error with the divergence. Otherwise \`git push\`.`,
    ``,
    `## Return (structured)`,
    `status, summary (with the pytest triple), commits, gate_handoff, decisions_made, open_questions, blocked_reason if blocked. Use TaskCreate to track your multi-file work. The full-suite gate is the hard correctness floor; the logbook Discussion is the artifact future sessions depend on.`,
  ].join('\n')
}

// EDIT THIS before each launch (args.phases overrides IF it propagates; the
// constant is the reliable control — see the canbridge runner's 2026-06-29 note).
const PHASES_TO_RUN = [7]

const phases = (args && Array.isArray(args.phases) && args.phases.length) ? args.phases : PHASES_TO_RUN
const phaseSource = (args && Array.isArray(args.phases) && args.phases.length) ? 'args.phases' : 'PHASES_TO_RUN constant'

phase('Phase runner')
log(`mvp phase-runner: attempting phase(s) ${phases.join(', ')} (from ${phaseSource}) — fresh Opus 4.8 agent each, full-gated, auto-commit+push to ${BRANCH}, decide+document at reversible forks, stop at safety forks.`)

const out = []
for (const n of phases) {
  log(`▶ Phase ${n} — gate: ${GATES[n] || 'see plan'}`)
  const r = await agent(phasePrompt(n), { schema: PHASE_RESULT, label: `phase-${n}`, phase: `Phase ${n}`, model: 'opus', effort: 'high' })
  out.push({ phase: n, result: r })
  if (!r) { log(`Phase ${n}: agent returned null — stopping the chain.`); break }
  if (r.status !== 'landed') {
    log(`Phase ${n}: status=${r.status} — stopping the chain. ${r.blocked_reason || ''}`)
    break
  }
  log(`✓ Phase ${n} landed (${(r.commits || []).join(', ')}). Deferred gate: ${r.gate_handoff || GATES[n]}`)
}
return { phasesAttempted: phases, results: out }
