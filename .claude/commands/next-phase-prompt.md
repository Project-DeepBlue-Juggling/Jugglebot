---
description: Generate a self-contained prompt for a fresh Claude session to implement the next phase of a phased plan. Invoke with /next-phase-prompt <plan-name> [phase-number].
---

# Next Phase Prompt Command

Generate a self-contained prompt that a fresh Claude session can use
to implement the next (or specified) phase of a phased plan in
`plans/active/`. The prompt embeds the four core values
(**consistency**, **robustness**, **determinacy**, **rigour**) with
concrete actionable interpretations, the reading order, the process
gates, and the clarifying-question prompts.

This command **does not implement** the phase. It only renders the
prompt for a fresh Claude session to take on.

## Arguments

- `<plan-name>`: plan filename, with or without `.md`, with or
  without the `plans/active/` prefix. Examples:
  `mpc-sadpath-coverage-tiers-1-3`, `mvp-trajectory-bringup`,
  `plans/active/can-process-refactor.md`.
- `[phase-number]` (optional): explicit phase to prompt for. If
  omitted, auto-detect the first **NOT STARTED** or **IN PROGRESS**
  row in the plan's summary table.

## Protocol

### Step 1: Locate and parse the plan

1. Find the plan file in `plans/active/`. If not found there, check
   `plans/archived/` — if archived, report "plan is already
   archived; no further phases" and stop. If not found anywhere,
   list available active plans and stop.
2. Verify the plan has a phased structure. Try in order:

   **Primary — Implementation Phase Summary table.** Search for a
   markdown table whose header row includes `Phase` and `Status`
   (typically directly under an `## Implementation Phase Summary`
   heading). If present, parse rows into
   `(phase_id, scope, status, date)`.

   **Fallback — direct phase headings.** If no summary table,
   search for headings matching the pattern
   `^## Phase <N>:` or `^### Phase <N>:`. For each:
   - `phase_id` = the integer after `Phase`
   - `scope` = the heading text after the integer, with any
     trailing ` — COMPLETE (date)` / ` — NOT STARTED` /
     ` — IN PROGRESS` suffix stripped
   - `status` = the suffix marker if present (`COMPLETE`,
     `NOT STARTED`, `IN PROGRESS`). For bare headings (no suffix):
     default the FIRST bare phase (smallest `phase_id` without a
     `COMPLETE` suffix) to `IN PROGRESS` and every subsequent bare
     phase to `NOT STARTED`. This matches the natural reading of a
     sequence like `## Phase 4: ...` followed by `## Phase 5: ...`
     where Phase 4 is mid-flight and Phase 5 hasn't started.
   - `date` = parsed from `COMPLETE (YYYY-MM-DD)` suffix if present

   Both `mvp-trajectory-bringup.md` (heading-only convention) and
   `mpc-sadpath-coverage-tiers-1-3.md` / `can-process-refactor.md`
   (summary-table convention) parse correctly via the appropriate
   path.

   If NEITHER convention yields phases, report:

   > Plan `<name>` does not appear to be phased (no
   > "Phase / Status" summary table and no `## Phase N:` /
   > `### Phase N:` headings found). This command is scoped to
   > phased plans only. Suggest writing a tailored prompt manually.

   and stop.

3. (After parsing, regardless of source) sanity-check the result:
   if zero rows parsed but the plan file is non-trivial, the
   convention may be third-party — fall back to reporting "could
   not auto-parse phases" rather than guessing.

### Step 2: Resolve the target phase

- If `[phase-number]` was provided:
  - Verify the phase exists in the parsed table; if not, list
    available phase numbers and stop.
  - If the requested phase is already `COMPLETE`, warn the user:

    > Phase `{N}` of `{plan-name}` is already COMPLETE (date `{D}`).
    > Generating the prompt anyway — confirm intent before pasting
    > into a fresh session (retrospective regeneration is supported
    > but rarely what you want; re-shipping a completed phase
    > requires careful coordination with prior commits).

    and proceed with the requested phase.
- Otherwise, find the **first row** with status `NOT STARTED` or
  `IN PROGRESS`. If none exist (all phases COMPLETE), report:

  > Plan `<name>` has no pending phases. Consider running
  > `/archive-plan <name>` to move it to `plans/archived/`.

  and stop.

### Step 3: Auto-pull prior-phase facts

Find the **immediately-previous COMPLETE phase** (largest phase_id
< target with status `COMPLETE`). If one exists, extract three facts
by reading the plan's detailed Phase section and the latest git log:

1. **`{PREV_SHA}`** — Parse the previous phase's `### Phase N: ... — COMPLETE`
   detail section for the commit SHA. Constrain the search to the
   **Outcome paragraph** (the text between `**Outcome.**` and the
   next bold marker `**...**` or the next `### Phase` heading); do
   NOT search elsewhere in the file (other plan sections may cite
   cross-plan SHAs that would be wrong to pick up). Within that
   region, find 7-character hex codes in backticks. If multiple
   appear (e.g., a phase with two bug-fix commits), use the LAST
   one in file-position order (the chronologically-later commit on
   a fast-forward branch). Fallback chain:
   1. `git log --oneline --grep="Plan {PLAN_NUMBER} Phase {prev_phase_id}" -1`
      (skip this step if `{PLAN_NUMBER}` is None)
   2. `git log --oneline -1 -- plans/active/<plan>.md` for the
      latest plan-touching commit
   3. `git log --oneline -1` for the latest branch commit
   4. Literal placeholder `<run "git log -1" to confirm baseline SHA>`

2. **`{PREV_BASELINE}`** — Parse the previous phase's Outcome
   paragraph for a passing test count (e.g., `1210/1210 + 1
   xfailed`, `1193 passing`). Fallback: instruct the receiving
   agent to confirm via `./run_tests.sh` themselves; use the
   literal `<run "./run_tests.sh" to confirm baseline pass count>`.

3. **`{PREV_LOGBOOK_SLUG}`** — Parse the previous phase's Outcome
   paragraph for a logbook link (e.g., `logbook/2026-05-11-tier1a-real-solver-failures.md`).
   Fallback chain:
   1. Search `logbook/INDEX.md` for the most recent row whose
      `Phase` cell contains `<plan-name> — Phase {prev_phase_id}`
   2. Literal placeholder `<no prior phase logbook found — match Phase 0/Phase 1 conventions>`

If **no previous COMPLETE phase exists** (target is Phase 0 / first
phase), set all three to `None` and omit the prior-phase reference
sentences in the rendered prompt.

### Step 4: Render the prompt

Emit a single ```` ``` ````-fenced markdown block in the conversation
containing the assembled prompt. Use the template in the
`<prompt-template>` section below; substitute:

| Placeholder                | Source                                                          |
|----------------------------|-----------------------------------------------------------------|
| `{PLAN_NAME}`              | Plan filename without `.md` (e.g., `mpc-sadpath-coverage-tiers-1-3`). Used in the archival closer for final-phase renders. |
| `{PLAN_PATH}`              | Full path including `plans/active/` prefix                       |
| `{PLAN_NUMBER}`            | The plan's `Plan N` identity (e.g., `2` for the MPC sad-path plan). Source: `grep -oE "Plan [0-9]+ Phase" plans/active/<plan>.md \| sort -u \| head -1` against the target plan; fall back to `git log --oneline -- plans/active/<plan>.md \| grep -oE "Plan [0-9]+" \| head -1`. If no `Plan N` identity is found (e.g., a new plan that hasn't been numbered yet), set to None — the template's commit-format example wraps the `Plan {PLAN_NUMBER}` reference in `{IF PLAN_NUMBER}…{END}`. |
| `{TARGET_PHASE}`           | The target phase integer                                         |
| `{TARGET_PHASE_PLUS_ONE}`  | `TARGET_PHASE + 1` as integer. If `TARGET_PHASE` is the FINAL phase in the parsed table, set to None and the template's "Phase N+1 cleared to start" closer is replaced with an `/archive-plan` suggestion. |
| `{PHASE_TITLE}`            | The Scope cell of the target row, trimmed                        |
| `{PREV_PHASE}`             | Previous COMPLETE phase integer (omit related sentences if None) |
| `{PREV_SHA}`               | From Step 3, with fallback if needed                             |
| `{PREV_BASELINE}`          | From Step 3, with fallback if needed                             |
| `{PREV_LOGBOOK_SLUG}`      | From Step 3, with fallback if needed                             |
| `{PLAN_HAS_HYPOTHESIS}`    | True if any phase scope mentions "hypothesis", "fuzz", or "property" — include the Working Note #5 sentence in core values |
| `{PLAN_HAS_XFAIL}`         | True if the plan has a "Production-code changes triggered by tests" subsection — include the xfail-discipline reminder |
| `{COMMIT_PREFIX}`          | Infer from the previous phase's commit message: `feat(...)`, `fix(...)`, `test(...)`, `docs(...)`. If absent, the `{IF NOT COMMIT_PREFIX}` alternate in the template renders the generic "follow the prefix convention the phase's prior commits established" sentence. |

**Conditional template sections.** The template uses
`{IF FLAG}...{END}` and `{IF NOT FLAG}...{END}` blocks for content
that should appear only when a flag (`PREV_PHASE`,
`PREV_LOGBOOK_SLUG`, `COMMIT_PREFIX`, `PLAN_HAS_HYPOTHESIS`,
`PLAN_HAS_XFAIL`, `PLAN_NUMBER`, `TARGET_PHASE_PLUS_ONE`) is
truthy / falsy respectively. When rendering:

- `{IF FLAG}…{END}` — emit the enclosed text WITH placeholders
  substituted iff `FLAG` is True or the named placeholder has a
  real (non-None, non-empty) value.
- `{IF NOT FLAG}…{END}` — the exact complement: emit the enclosed
  text iff `FLAG` is False or the named placeholder is None / empty.
  Paired `{IF FLAG}…{END}` / `{IF NOT FLAG}…{END}` blocks render
  alternatives without leaving orphan whitespace.
- When omitting: drop the entire enclosed text INCLUDING the
  leading bullet / list marker / newline if the omitted content
  was a standalone bullet. Don't leave dangling punctuation,
  orphan list markers, or runs of blank lines.

Pre-prompt note (above the code block): one short sentence stating
which plan + phase the prompt targets, and the resolved
prior-phase facts.

- With a prior phase (e.g., Phase 2 of an in-flight plan):
  > "Targeting Phase 2 of mpc-sadpath-coverage-tiers-1-3 (prior
  > phase: Phase 1, SHA `f466829`, baseline 1210 passing + 1
  > xfailed, logbook
  > `2026-05-11-tier1a-real-solver-failures`)."
- Without a prior phase (Phase 0 / inaugural):
  > "Targeting Phase 0 of `<plan>` — inaugural phase, no prior
  > baseline. Receiving agent will record the inaugural
  > `./run_tests.sh` count and `git log -1` SHA in this phase's
  > logbook."

### Step 5: Closing note

After the code block, emit one of:

- With a prior phase:
  > Copy the block above into a fresh Claude session (or `/clear`
  > and paste into this one). The receiving agent will work the
  > phase end-to-end. If the agent reports the baseline doesn't
  > match `{PREV_SHA}` / `{PREV_BASELINE}`, investigate before
  > continuing — stale prior-phase facts are the most common cause.
- Without a prior phase:
  > Copy the block above into a fresh Claude session (or `/clear`
  > and paste into this one). This is the plan's inaugural phase;
  > the receiving agent will record the baseline `git log -1` SHA
  > and `./run_tests.sh` count in the logbook so future phases
  > have a reference point.

## Important notes

- **The command is for phased plans only.** Methodology docs
  (Level 1/2/3), milestone docs, and free-form refactor logs require
  manual prompting.
- **Auto-pulled facts gracefully degrade to placeholders.** If the
  previous phase's Outcome paragraph doesn't match the expected
  pattern (e.g., a plan whose phases don't follow the
  `Outcome → commit SHA + baseline + logbook link` convention), the
  prompt's baseline-confirmation step instructs the receiving agent
  to confirm the facts themselves rather than trusting a guess.
- **The rendered prompt is intentionally verbose.** It is
  load-bearing for setting the fresh agent up for rigour. Do not
  trim it for brevity.
- **Don't modify the template's core-values section.** Those four
  values (consistency, robustness, determinacy, rigour) are the
  project's stated foundation and should appear verbatim in every
  rendered prompt, with phase-specific actionable interpretations
  substituted in.
- **The command itself never invokes pytest, git push, or any
  external state-changing operation.** It is read-only against the
  repo (reads the plan, git log, logbook).

## Prompt template

The rendered prompt fills the placeholders in this template and emits
it as a single ```` ``` ````-fenced block.

<prompt-template>
Implement Phase {TARGET_PHASE} of {PLAN_PATH}:
"{PHASE_TITLE}".

Read first, in this order:
  1. {PLAN_PATH} — the full plan, with particular attention to:
     • The Phase {TARGET_PHASE} detail section (the work spec —
       Scope, New/modified files, Test cases, Critical details,
       Dependencies, Exit criteria)
     • The "Working notes" or "Notes for Collaborators" section if
       present (read every numbered point; these are the lessons
       the plan author distilled from prior phases — they apply to
       every phase)
     • The "Production-code changes triggered by tests" subsection
       if present — confirms whether this phase is test-additions-
       only or production-code is in scope. If a test surfaces a
       real bug AND this phase is test-additions-only, file per the
       plan's xfail discipline (test ID + tracking reference +
       target close phase or date); do NOT auto-fix.
     {IF PREV_PHASE}• The Phase {PREV_PHASE} Outcome paragraph and
       its detailed section — establishes the format precedent for
       this commit, the logbook entry shape, and any helpers landed
       in prior phases you should reuse rather than reinvent.{END}
  2. Every file the Phase {TARGET_PHASE} section's "Critical
     details" cites (line numbers + symbol names). Confirm the
     citations match ground truth BEFORE writing tests — prior phase
     audits in this plan caught multiple line-citation drift errors.
     If a citation has drifted (function moved, lines shifted),
     refresh it as part of this phase's work.
  3. {IF PREV_LOGBOOK_SLUG}{PREV_LOGBOOK_SLUG} — the prior phase's
     logbook entry. Mirror its frontmatter shape, Discussion-section
     depth, xfail-accounting format, and cross-reference style. The
     "Logbook-Entry: <slug>" trailer in this phase's commit is
     required.{END}
     {IF NOT PREV_LOGBOOK_SLUG}If no prior phase logbook exists for
     this plan, model the new entry on
     logbook/2026-05-10-plant-interface-contract-phase-6-p3-p4-enforcement.md
     (canonical Discussion-section depth) and
     logbook/2026-05-11-tier1a-real-solver-failures.md (canonical
     per-recipe table + xfail accounting format).{END}
  4. CLAUDE.md "Workflow Rules" section — re-read especially:
     • "Analyze control-system implications before changes" (walk
       through one MPC cycle / one hardware update / one control
       loop with the proposed change before writing any code)
     • "TodoWrite checklist for multi-file tasks"
     • "Run pytest after code changes AND before git commit"
     • "Audit multi-document narrative changes" (/audit --unstaged
       before each commit; this work touches plan + logbook + test
       file at minimum)
     • "Grep before refactoring" (if any symbol rename or signature
       change is in scope)
     • "Capture user-corrections as memory, proactively" (if the
       user pushes back on framing, save the lesson)

Core values, non-negotiable throughout:

  • **Consistency** — match the prior phases' commit-message format
    ({IF COMMIT_PREFIX}`{COMMIT_PREFIX}: … — {IF PLAN_NUMBER}Plan {PLAN_NUMBER} {END}Phase {TARGET_PHASE}` with
    `Logbook-Entry: <slug>` trailer{END}{IF NOT COMMIT_PREFIX}follow the prefix
    convention the phase's prior commits established; include a
    `Logbook-Entry: <slug>` trailer{END}), logbook frontmatter shape,
    Discussion-section structure (Why this approach, What was ruled
    out, Tradeoffs, Xfail accounting if applicable, Open Questions).
    Reuse helpers / fixtures / contract patterns established by
    prior phases — do NOT reinvent them. If you find yourself
    writing a helper that "feels similar" to an existing one, grep
    first.

  • **Robustness** — each scenario test MUST be confirmed to drive
    the real failure path BEFORE the assertion logic is correct.
    For production-code changes (e.g., contract enforcement),
    confirm the regression test fails BEFORE the fix and passes
    AFTER (stash the production change, run the new test alone,
    verify failure; restore and re-run). Run `./run_tests.sh`
    after each commit; the hot-loop allocation contract test
    (tests/sim/test_hot_loop_allocation_contract.py) MUST stay green
    throughout.

  • **Determinacy** — hypothesis tests run with
    `--hypothesis-seed=0` for reproducibility in the verification
    gate. Property tests must pass at ci-deep
    (`--hypothesis-profile=ci-deep`, max_examples=1000) before
    declaring the phase complete. No flaky tests introduced — if a
    test flakes once in the verification gate, investigate the root
    cause before commit.
    {IF PLAN_HAS_HYPOTHESIS}Per the plan's Working Notes on hot-loop
    allocation flakes: when adding the first hypothesis test in a
    phase, decide whether to pre-empt the
    test_hot_loop_allocation_contract flake (gc.collect() at the
    start of the test OR `slow` marker + separate pytest invocation)
    NOW or defer to a later phase; document the choice.{END}

  • **Rigour** — pre-implementation, prototype each driver
    empirically in a throwaway script (`/tmp/probe_*.py`, NOT
    committed) before writing tests. Confirm the recipe for each
    test case works deterministically on the pinned dependency
    stack. Document the recipe as a comment block in the test file
    AND as a table in the logbook (mirror prior-phase logbooks).
    For each bug or test case, decide the design EXPLICITLY (raise
    vs atomic transition; real-driver vs synthetic; xfail vs add
    production-code support) and surface decision points to the
    user via AskUserQuestion BEFORE writing code. Don't paper over
    "the test passes" with mocked returns when the real path is
    drivable.

Process gates, in order:

  1. {IF PREV_PHASE}./run_tests.sh baseline — must show
     **{PREV_BASELINE}**. Baseline SHA: **{PREV_SHA}** or later
     (`git log -1` to confirm). If the baseline doesn't match,
     investigate — stale prior-phase facts are the most common cause
     and indicate prior commits weren't pushed or this branch isn't
     current.{END}{IF NOT PREV_PHASE}./run_tests.sh — this is the
     plan's first phase, so there is no prior baseline. Run
     `./run_tests.sh` and record the count (e.g., "1184 passing")
     as the inaugural baseline in this phase's logbook Verification
     section.  Also record the current `git log -1` SHA so future
     phases can confirm they started from this point.{END}

  2. Pre-implementation analysis:
     a. Read the production-code files cited in the Phase
        {TARGET_PHASE} Critical details section. Confirm line
        citations match ground truth.
     b. For each test case, run an empirical probe
        (`/tmp/probe_*.py`, not committed) confirming the driver
        produces the expected behaviour deterministically. Mirror
        the methodology in
        logbook/2026-05-11-tier1a-real-solver-failures.md
        ("Real-driver strategy per exit code" table).
     c. Audit any pre-existing tests in the same area that the
        plan says you should "extend" or "verify" — they often
        have naming-vs-behaviour mismatches worth surfacing in the
        logbook Discussion.
     d. Surface design decisions to the user via AskUserQuestion
        BEFORE writing tests. Common decision points to consider:
        - Real-driver vs synthetic (stats injection / mock /
          parameter pinning)
        - Test file structure (extend existing vs new file)
        - xfail policy when a real driver isn't available
        - Hypothesis stateful vs @composite + @given for property
          tests
        - Whether to pre-empt known flakes documented in the
          plan's Working Notes

  3. Implementation, test-by-test (or fix-by-fix for contract
     phases):
     a. Implement each test case / production change.
     b. Confirm regression-style tests fail before the production
        change and pass after (stash + re-run pattern).
     c. After each test class or production-fix unit, run
        `pytest tests/<scoped subset> -q` at ci-fast.
     d. Surface mid-implementation pivots to the user — don't drift
        from the agreed design silently.

  4. Verification gate:
     a. `./run_tests.sh` at ci-fast (the blessed full gate: parallel
        phase + a serial phase for `serial`-marked tests) — must
        equal `{PREV_BASELINE}` + new tests, with no regressions.
     b. If any hypothesis tests added or modified:
        `pytest tests/sim/<changed file> --hypothesis-profile=ci-deep --hypothesis-seed=0`
        to validate the property at 1000 examples.
     c. Verify hot_loop_allocation_contract still green:
        `pytest tests/sim/test_hot_loop_allocation_contract.py -q`
        at ci-fast (and ci-deep if hypothesis tests were added in
        this phase).

  5. Logbook entry (one per phase; mirror the prior-phase format):
     a. Frontmatter (type, date, status: resolved, phase reference,
        related_plan, related_entries, files_changed, subsystem,
        tags).  **No `commits:` field** — the commit's
        `Logbook-Entry:` trailer is the canonical bidirectional
        link (convention retired 2026-08-01).
     b. Short form is the default (10-30 lines: what/why + the
        `(date, command, result)` verification triple).  Escalate
        to the full section set — Summary, Motivation, Design,
        Implementation, Verification, Discussion, Open Questions,
        Related — when a Discussion trigger fires.
     c. Discussion is NON-NEGOTIABLE under any of the three
        triggers: (a) a hypothesis was withdrawn or reframed
        mid-phase, (b) a non-obvious tradeoff was accepted, (c) the
        chosen approach beat another reasonable approach for
        reasons future-readers wouldn't infer from the code alone.
        A phase that hits a trigger almost always hits (c).  Under
        a trigger, include "Why this approach over alternatives",
        "What was ruled out" and "Tradeoffs accepted" — skipping it
        there is the single most common way this project loses
        institutional memory.  See `logbook/README.md` § "Entry
        Length — short form is the default".
     d. Xfail accounting if applicable (Plan 2's three-field rule:
        test ID + tracking reference + target close phase or date).
     e. Update logbook/INDEX.md with the new entry as the topmost
        row.

  6. Update the plan: Phase {TARGET_PHASE} → COMPLETE in BOTH the
     summary table (status + date columns) AND the detailed phase
     heading (add an **Outcome** paragraph mirroring prior phases —
     commit SHA, logbook link, test-count delta, {IF TARGET_PHASE_PLUS_ONE}"Phase
     {TARGET_PHASE_PLUS_ONE} cleared to start" closer{END}{IF NOT TARGET_PHASE_PLUS_ONE}"This is the
     plan's final phase — consider running `/archive-plan {PLAN_NAME}`
     once any outstanding hardware-bringup / xfail items are resolved" closer{END}).
     **Any test-count claim** (in the Outcome paragraph here OR in
     the logbook Verification section) MUST cite the
     `(date, command, result)` triple — not just the count.  Example
     good: *"ci-deep (`pytest tests/ -q --hypothesis-profile=ci-deep`,
     run 2026-05-11): **1193/1193 pass in 560.98 s**."*  Example bad:
     *"Full suite passes at ci-deep (1193/1193, 560.98 s)."*  Without
     the triple, the claim is unverifiable from the artefact alone
     and the audit will flag it BLOCKING.  See the CLAUDE.md
     workflow rule "Cite test-count claims with the
     (date, command, result) triple".

  7. /audit --unstaged on the combined test + logbook + plan
     changes. Apply LOW/MEDIUM fixes auto; pause for HIGH-risk.
     The audit will catch line-citation drift, statement-count
     contradictions, stale comment references, and cross-document
     inconsistencies. Per prior-phase precedent, expect 2–5
     NOTE-level findings even on clean work.

  8. Commit (one logical unit per concrete behaviour change — match
     prior-phase granularity), then push in the same response. Do
     NOT write a SHA-backfill follow-up commit — the commit's
     `Logbook-Entry:` trailer already carries the link, and
     `git log --grep "Logbook-Entry: <slug>"` is the reverse query.

  9. If the plan update wasn't included in the commit above, do it
     as a separate commit (mirrors the prior-phase precedent):
     /audit --unstaged on the plan diff, commit, push.

Ask clarifying questions before assuming any of:

  • Real-driver vs synthetic strategy for any test case where the
    real path is "hard" — surface the alternatives (parameter
    tuning, monkey-patch, stats injection, xfail) with their honest
    tradeoffs. Don't paper over the difficulty by choosing
    synthetic silently.
  • Test file structure when the plan says "extend
    <file>" — confirm whether a single file with multiple classes
    or a split is preferred.
  • {IF PLAN_HAS_HYPOTHESIS}Hypothesis stateful structure
    (RuleBasedStateMachine vs @composite + @given) — match the
    in-repo exemplar (SchedulerStateMachine in
    tests/sim/test_scheduler_contract.py is the canonical stateful
    pattern).{END}
  • {IF PLAN_HAS_XFAIL}xfail policy if a real driver isn't
    available for a test case — confirm whether to xfail
    permanently with documented justification, monkey-patch as
    fallback, or extend production-code surface (the latter
    crosses into a separate commit per the plan's "test additions
    only" discipline).{END}
  • Commit granularity when the phase has multiple distinct
    behaviour changes — separate commits per change is the
    default; one combined commit is acceptable only when the
    changes are tightly coupled and the plan text doesn't
    explicitly mandate separate commits.
  • If a NEW bug surfaces during this phase's work that is
    orthogonal to the phase's stated scope — DO NOT auto-fix as
    part of this phase. File per the plan's xfail discipline
    (test ID + tracking reference + target close phase) and
    continue, OR raise the discovery to the user before
    proceeding if the bug is safety-critical.

Begin by reading the Phase {TARGET_PHASE} section of the plan in
full {IF PREV_PHASE}and confirming the baseline (`git log -1` should show
{PREV_SHA} or later; `./run_tests.sh` should show {PREV_BASELINE}).{END}{IF NOT PREV_PHASE}and recording the current `git log -1` SHA + `./run_tests.sh` count as the inaugural baseline in this phase's logbook Verification section.{END}
</prompt-template>
