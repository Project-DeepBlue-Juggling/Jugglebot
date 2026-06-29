export const meta = {
  name: 'bb-tilt-phases',
  description: 'Run a segment of the online-juggle tilt re-architecture (plans/active/bb-online-juggle-tilt-rearchitecture.md). Each phase = fresh agents: implement -> independent audit -> finalize (apply fixes, full pytest gate, commit, fetch-guarded push, write next-phase prompt). Stops at the segment end for the human go/no-go gate. args: {from, to, handoff?}.',
  phases: [
    { title: 'Implement', detail: 'fresh agent implements the phase per the plan' },
    { title: 'Audit', detail: 'independent audit-reporter on the working-tree diff' },
    { title: 'Finalize', detail: 'apply fixes, full pytest gate, commit, fetch-guarded push, write next-phase prompt' },
  ],
}

// ---------------------------------------------------------------------------
const PLAN = 'plans/active/bb-online-juggle-tilt-rearchitecture.md'
const BRANCH = 'demo/bb-led-two-ball-juggle'
const VENV = 'source ~/Desktop/PDJ_venv/venv/bin/activate'
const COAUTHOR = 'Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>'

const from = (args && args.from != null) ? args.from : 0
const to = (args && args.to != null) ? args.to : from
let priorHandoff = (args && args.handoff) ? String(args.handoff) : null

log(`tilt re-architecture: running phases ${from}..${to} from ${PLAN} on ${BRANCH}`)

const IMPL_SCHEMA = {
  type: 'object',
  additionalProperties: false,
  required: ['summary', 'filesChanged', 'acceptanceStatus', 'testResult'],
  properties: {
    summary: { type: 'string', description: 'What you changed and why (1-2 paragraphs)' },
    filesChanged: { type: 'array', items: { type: 'object', additionalProperties: false,
      required: ['path', 'why'], properties: { path: { type: 'string' }, why: { type: 'string' } } } },
    acceptanceStatus: { type: 'string', description: 'Per the plan Phase acceptance criteria: met / partial (explain)' },
    testResult: { type: 'string', description: 'The (date, exact command, result) triple for the tests you ran' },
    notesForReviewer: { type: 'string', description: 'Anything the auditor/finalizer must know (risks, deferred items, design choices)' },
  },
}
const AUDIT_SCHEMA = {
  type: 'object',
  additionalProperties: false,
  required: ['verdict', 'findings'],
  properties: {
    verdict: { type: 'string', enum: ['CLEAN', 'ISSUES'] },
    findings: { type: 'array', items: { type: 'object', additionalProperties: false,
      required: ['severity', 'title', 'detail'],
      properties: {
        severity: { type: 'string', enum: ['BLOCKING', 'WARNING', 'NOTE'] },
        title: { type: 'string' }, detail: { type: 'string' },
        file: { type: 'string' }, suggestedFix: { type: 'string' },
      } } },
  },
}
const FINAL_SCHEMA = {
  type: 'object',
  additionalProperties: false,
  required: ['committed', 'fullSuiteResult', 'pushStatus', 'acceptanceMet', 'gateAssessment', 'nextPhasePrompt'],
  properties: {
    committed: { type: 'boolean', description: 'true only if the full suite passed and a commit was made' },
    commitShas: { type: 'array', items: { type: 'string' } },
    fixesApplied: { type: 'string', description: 'Which audit findings you fixed (and which you skipped + why)' },
    fullSuiteResult: { type: 'string', description: 'The (date, command, result) triple for pytest tests/ -q; note the flaky alloc test if it appeared' },
    pushStatus: { type: 'string', enum: ['pushed', 'aborted-diverged', 'aborted-other', 'not-attempted'] },
    parallelSessionWarning: { type: 'string', description: 'If push aborted on divergence, what you saw (origin ahead by N, the foreign commit) — for the human' },
    acceptanceMet: { type: 'boolean' },
    gateAssessment: { type: 'string', description: 'Plain verdict on the phase acceptance criteria, for the human gate' },
    nextPhasePrompt: { type: 'string', description: 'Self-contained handoff prompt for the NEXT phase (the /next-phase-prompt style: where you are, what is done, read-first, the phase goal/approach/acceptance, process gates). Empty if this was the last phase.' },
  },
}

const results = []
let aborted = null

for (let ph = from; ph <= to && !aborted; ph++) {
  // ---- Implement ----
  const impl = await agent(
    `You are a FRESH Claude implementing **Phase ${ph}** of the plan at ${PLAN}.\n\n` +
    `READ FIRST (in order): (1) ${PLAN} — esp. its "Phase ${ph}" section (goal / approach / ` +
    `files / acceptance) AND §3 cross-cutting constraints; (2) CLAUDE.md — Workflow Rules + ` +
    `Engineering Philosophy, follow them exactly (analyse control/physics implications before ` +
    `touching the planner/realisation; TodoWrite for multi-file work; run pytest after *.py/*.yaml ` +
    `changes); (3) the prior-phase handoff below if present.\n\n` +
    `PRIOR-PHASE HANDOFF:\n${priorHandoff || '(none — first phase of this segment; start from the plan + the current repo state)'}\n\n` +
    `IMPLEMENT Phase ${ph} fully (code + tests). Use the venv for ALL python/pytest: \`${VENV}\`. ` +
    `Honour the project's physical-intuition culture — if the plan's framing looks wrong as you ` +
    `implement, say so loudly in notesForReviewer rather than forcing it. Do NOT git-commit (the ` +
    `finalize stage commits). Run the relevant tests and report the (date, command, result) triple.`,
    { label: `impl-p${ph}`, phase: 'Implement', schema: IMPL_SCHEMA }
  )
  if (!impl) { aborted = `Phase ${ph}: implementer agent died (terminal error). Segment stopped.`; break }

  // ---- Audit (independent) ----
  const audit = await agent(
    `Audit the UNSTAGED working-tree changes that implement **Phase ${ph}** of ${PLAN}.\n` +
    `Run \`git diff\` + \`git status\`; read every changed file IN FULL. This is robotics control ` +
    `code — weigh correctness, SAFETY / physics implications (velocity, feedforward, timing, jerk, ` +
    `tilt kinematics), broken contracts, missing edge cases, and narrative/number consistency in any ` +
    `docs. Classify findings BLOCKING / WARNING / NOTE.\n\n` +
    `The implementer's own summary (for context, not to be trusted blindly):\n${JSON.stringify(impl)}`,
    { label: `audit-p${ph}`, phase: 'Audit', agentType: 'audit-reporter', schema: AUDIT_SCHEMA }
  )
  // Audit is advisory, so a dead auditor doesn't stop the segment — but its death
  // must be VISIBLE, never coerced to the literal "undefined" in the finalize prompt.
  const auditBlock = audit
    ? JSON.stringify(audit)
    : '(AUDIT AGENT DIED — no findings produced. Do NOT silently skip the audit: '
      + 'say so in fixesApplied AND gateAssessment so the human reviews this phase manually.)'

  // ---- Finalize: fix, test-gate, commit, fetch-guarded push, next-phase prompt ----
  const fin = await agent(
    `You are FINALIZING **Phase ${ph}** of ${PLAN} on branch ${BRANCH}. Use the venv (\`${VENV}\`) ` +
    `for python/pytest. Steps, in order:\n` +
    `1. Apply the audit's BLOCKING findings and reasonable WARNINGs; skip NOTE / out-of-scope (record ` +
    `which in fixesApplied). Audit findings:\n${auditBlock}\n` +
    `   Implementer summary:\n${JSON.stringify(impl)}\n` +
    `2. FULL TEST GATE: \`pytest tests/ -q\`. It MUST pass. If the ONLY failure is the known order/load- ` +
    `flaky \`test_hot_loop_allocation_contract\`, re-run it isolated to confirm it passes and note that; ` +
    `ANY other failure -> set committed=false, do NOT commit, explain in gateAssessment. Cite the ` +
    `(date, command, result) triple in fullSuiteResult.\n` +
    `3. If your fixes touched a logbook/plan/normative .md, run \`/audit --unstaged\` on the doc changes ` +
    `(CLAUDE.md rule).\n` +
    `4. COMMIT: stage the phase's files (verify with \`git diff --cached --stat\`); write the message to ` +
    `a file and use \`git commit -F\` (NEVER backticks in -m). Include a \`Logbook-Entry:\` trailer if a ` +
    `logbook entry was created, and end with:\n${COAUTHOR}\n` +
    `   If a logbook entry was created, backfill its commit SHA in a small follow-up commit.\n` +
    `5. PUSH, FETCH-GUARDED: \`git fetch\`; \`git status -sb\`. If \`origin/${BRANCH}\` is AHEAD of local ` +
    `(has commits you don't have) OR the working tree has foreign changes you didn't make, DO NOT push ` +
    `and DO NOT rebase/pull — set pushStatus='aborted-diverged', describe exactly what you saw in ` +
    `parallelSessionWarning, and stop (the human resolves it). Otherwise \`git push\` and set ` +
    `pushStatus='pushed'.\n` +
    `6. WRITE THE NEXT-PHASE PROMPT (Phase ${ph + 1}) as a fully self-contained handoff in the ` +
    `/next-phase-prompt style: worktree+branch+venv, the baseline commit chain incl. your new SHA, ` +
    `read-first list, Phase ${ph + 1}'s goal/approach/files/acceptance (from ${PLAN}), the process gates, ` +
    `and the success criterion. Put it in nextPhasePrompt. If Phase ${ph} is the last (Phase 4), set ` +
    `nextPhasePrompt to '' and say so in gateAssessment.\n` +
    `Return the structured result. gateAssessment must give the human a plain verdict on whether ` +
    `Phase ${ph}'s acceptance criteria were met.`,
    { label: `finalize-p${ph}`, phase: 'Finalize', schema: FINAL_SCHEMA }
  )
  if (!fin) { aborted = `Phase ${ph}: finalize agent died (terminal error) — changes may be uncommitted in the working tree. Segment stopped; inspect git status.`; break }

  results.push({
    phase: ph,
    acceptanceMet: fin.acceptanceMet,
    committed: fin.committed,
    commitShas: fin.commitShas || [],
    fullSuiteResult: fin.fullSuiteResult,
    pushStatus: fin.pushStatus,
    parallelSessionWarning: fin.parallelSessionWarning || '',
    auditVerdict: (audit && audit.verdict) || 'NO-AUDIT',
    auditBlocking: ((audit && audit.findings) || []).filter(f => f.severity === 'BLOCKING').length,
    gateAssessment: fin.gateAssessment,
  })
  priorHandoff = fin.nextPhasePrompt || null

  // Stop the segment early if a phase failed its test gate or hit divergence.
  if (fin.committed === false) { aborted = `Phase ${ph}: NOT committed (test gate failed or blocked). Segment stopped — see gateAssessment.`; break }
  if (fin.pushStatus === 'aborted-diverged') { aborted = `Phase ${ph}: push aborted — origin/${BRANCH} diverged (parallel session). Segment stopped; human must resolve.`; break }
}

// Gate guidance for the human at the segment boundary.
const GATES = {
  0: 'GATE after Phase 0: is tilt-tracking good enough (modest tilt held accurately, lever-arm understood) to build on? Also: did the workflow mechanics work end-to-end? If yes -> run {from:1,to:2}.',
  2: 'GATE after Phase 2 (MAKE-OR-BREAK): did the tilt-aimed throw kill the divergence? If yes -> run {from:3,to:4}. If no -> STOP and re-plan with the operator.',
  4: 'Phase 4 complete — sustained 2-ball is the goal; review the catch count + the headline tests.',
}

return {
  ranPhases: `${from}..${to}`,
  aborted,
  results,
  nextGate: aborted
    ? '(segment aborted before its gate — resolve `aborted` / `results` first; no go/no-go yet)'
    : (GATES[to] || `Segment ${from}..${to} done; next segment per ${PLAN} §4.`),
  nextPhaseHandoff: priorHandoff,
}
