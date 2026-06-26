---
description: Generate a self-contained prompt for a fresh Claude session to implement the next phase of work — an explicit plan phase OR a conversationally-understood "next phase". Invoke with /next-phase-prompt <plan-name [phase-number] | free-form phase description>.
---

# Next Phase Prompt Command

Generate a self-contained prompt that a fresh Claude session can use to
implement the next (or specified) **phase of work**. A "phase" may be:

- an explicit phase of a phased plan in `plans/active/` (the original,
  most-structured case), **or**
- a conversationally-understood next unit of work — e.g. "implement the
  contact-mechanics integration", "land the Phase-4 hardware bring-up",
  "do the next migration batch" — where the current session knows what
  the next phase means even though no numbered plan row spells it out.

The rendered prompt embeds the four core values (**consistency**,
**robustness**, **determinacy**, **rigour**) with concrete actionable
interpretations *tailored to the kind of work*, the reading order, the
process gates, and the clarifying-question prompts.

This command **does not implement** the phase. It only renders the
prompt for a fresh Claude session to take on.

The skill's robustness does **not** come from the plan table — it comes
from two things it always does regardless of source: (a) hand the fresh
session **verified ground-truth prior-state facts** (baseline SHA, test
count, prior logbook), and (b) impose the **read-order / core-values /
process-gates / clarifying-questions skeleton**. Both survive when the
phase is described rather than tabulated — change the *sources*, never
drop the *requirement*.

## Arguments

Accepts **either** form:

- **Explicit-plan mode** — `<plan-name> [phase-number]`: plan filename
  (with or without `.md` / the `plans/active/` prefix) and an optional
  phase integer. Examples: `mpc-sadpath-coverage-tiers-1-3`,
  `hardware-bringup 4`, `plans/active/can-process-refactor.md`.
- **Described mode** — a free-form description of the next phase, with
  no plan file required. Examples:
  `bb-led-two-ball-juggle — contact mechanics integration`,
  `next batch of the CAN handler migration`,
  `implement the QTM correction fallback we just discussed`.

If the argument is ambiguous, prefer explicit-plan mode when the first
token resolves to a file in `plans/active/`; otherwise treat the whole
argument as a described-mode phase. State which mode you used.

## Protocol

### Step 1: Resolve the phase definition (resolution ladder)

Determine *what the phase is* from the most-specific available source.
Try the rungs in order; **state in the pre-prompt note which rung you
used.** Do NOT stop just because there is no phase table — fall through.

1. **Explicit plan phase.** If the argument names a plan file in
   `plans/active/` (or `plans/archived/` — if archived, report "plan is
   already archived; no further phases" and stop):

   **Primary — Implementation Phase Summary table.** Search for a
   markdown table whose header row includes `Phase` and `Status`
   (typically under an `## Implementation Phase Summary` heading). Parse
   rows into `(phase_id, scope, status, date)`.

   **Fallback — direct phase headings.** If no summary table, search for
   headings matching `^## Phase <N>:` or `^### Phase <N>:`. For each:
   - `phase_id` = the integer after `Phase`
   - `scope` = the heading text after the integer, trailing
     ` — COMPLETE (date)` / ` — NOT STARTED` / ` — IN PROGRESS` stripped
   - `status` = the suffix marker if present (`COMPLETE`, `NOT STARTED`,
     `IN PROGRESS`). For bare headings: default the FIRST bare phase
     (smallest `phase_id` without a `COMPLETE` suffix) to `IN PROGRESS`
     and every subsequent bare phase to `NOT STARTED`.
   - `date` = parsed from a `COMPLETE (YYYY-MM-DD)` suffix if present

   Both `hardware-bringup.md` (heading-only) and
   `mpc-sadpath-coverage-tiers-1-3.md` / `can-process-refactor.md`
   (summary-table) parse via the appropriate path. After parsing,
   sanity-check: if zero rows parsed but the file is non-trivial, the
   convention may be third-party — drop to rung 2/3 rather than guessing.

2. **Semi-structured next-steps.** If there is no parseable phase table
   but a clear next-unit specification exists elsewhere, use it:
   - the **latest logbook entry**'s `## Integration plan` /
     `## Next steps` / `## Open Questions` (follow-up) section — this is
     the canonical handoff when a prior session deferred work to a fresh
     one (it lists the concrete steps, the deferred decisions, and the
     probe recipes);
   - a plan's prose "next" / "remaining work" section that isn't a
     numbered table.
   The phase scope is that section's content, attributed to its source.

3. **Described / conversational phase.** Otherwise synthesize the phase
   spec from: the free-form argument, the **current conversation** (what
   "next phase" was just agreed), the latest logbook entry, and git/test
   state. Write an explicit one-paragraph scope so the fresh session
   isn't reconstructing it from nothing.

Only stop and ask the user if NONE of the rungs yield a determinable
phase (e.g. an empty argument with no conversational context and no
logbook breadcrumb).

### Step 2: Resolve the target phase / scope

- **Explicit-plan mode:**
  - If `[phase-number]` was provided, verify it exists; if not, list
    available phase numbers and stop. If it is already `COMPLETE`, warn:
    > Phase `{N}` of `{plan-name}` is already COMPLETE (date `{D}`).
    > Generating anyway — confirm intent before pasting (retrospective
    > regeneration is rarely what you want).
    and proceed.
  - Otherwise find the **first** `NOT STARTED` / `IN PROGRESS` row. If
    all are COMPLETE, report:
    > Plan `<name>` has no pending phases. Consider
    > `/archive-plan <name>` to move it to `plans/archived/`.
    and stop.
- **Described / next-steps mode:** the target scope is the resolved
  description from Step 1 (rung 2 or 3). There is no phase integer; use a
  short slug label (e.g. `contact-mechanics-integration`) wherever the
  template references a phase identifier, and set the numeric
  placeholders (`{TARGET_PHASE}`, `{TARGET_PHASE_PLUS_ONE}`) to None so
  their conditional blocks render the label/closer variants.

### Step 3: Gather prior-state facts (always — the robustness core)

Hand the fresh session three verified facts. Each has a resolution
ladder; use the most-specific available source and note any fallback.

1. **`{PREV_SHA}`** — the baseline commit the phase builds on.
   1. *Plan mode:* the previous phase's `### Phase N: … — COMPLETE`
      **Outcome paragraph** (text between `**Outcome.**` and the next
      bold marker / `### Phase` heading) — 7-char hex in backticks; if
      several, the LAST in file order.
   2. `git log --oneline --grep="Plan {PLAN_NUMBER} Phase {prev}" -1`
      (skip if `{PLAN_NUMBER}` is None).
   3. The latest logbook entry's `commits:` frontmatter SHA.
   4. `git log -1` (the current branch tip — always available).
2. **`{PREV_BASELINE}`** — the passing test count to start from.
   1. *Plan mode:* the previous phase's Outcome paragraph
      (e.g. `1210/1210 + 1 xfailed`).
   2. The latest logbook entry's **Verification** `(date, command,
      result)` triple.
   3. Literal `<run "pytest tests/ -q" to confirm baseline pass count>`.
3. **`{PREV_LOGBOOK_SLUG}`** — the prior entry to mirror.
   1. *Plan mode:* the previous phase Outcome's logbook link.
   2. The topmost row of `logbook/INDEX.md` (the most recent entry —
      this is the right anchor in described mode, where the prior work
      was just logged).
   3. Literal `<no prior logbook found — match Phase 0/Phase 1
      conventions>`.

If there is genuinely no prior state (an inaugural unit), set all three
to None and omit the prior-state reference sentences in the render.

### Step 4: Classify the work type

Infer `{WORK_TYPE}` from the phase scope + source — it selects which
domain-specific gates and clarifying questions the template renders (the
generic skeleton and the four core values are always present):

- **`test-coverage`** — adding/strengthening tests (scope mentions
  tests, fuzz, property, coverage, sad-path, contract-enforcement
  tests). Renders: real-driver-vs-synthetic, ci-deep hypothesis,
  allocation-flake handling, xfail discipline.
- **`implementation`** — building a feature / integration / migration
  (scope mentions implement, integrate, port, build, wire, migrate).
  Renders: control-system-implications walk, probe-before-integrate,
  checkpoint-before-sinking-effort, physical-intuition pushback,
  end-to-end validation (not just unit tests).
- **`investigation`** — diagnosing a hardware/runtime issue (scope
  mentions diagnose, root-cause, why, failure, regression hunt).
  Renders: reproduce-deterministically-first, diagnose→hypothesis→fix→
  verify, abandon-hypotheses-that-don't-survive-data.
- **`refactor`** — behaviour-preserving restructuring (scope mentions
  refactor, rename, extract, restructure, dedupe). Renders:
  grep-all-references + count + verify-zero-after, behaviour-preservation
  (tests green before AND after, no semantic change).
- **`docs`** — documentation / planning artefacts. Renders: the audit
  gate for multi-document narrative and cross-reference consistency;
  lighter test gates.

If the scope spans types, pick the dominant one and note the secondary
in the pre-prompt note (e.g. "implementation, with a test-coverage
sub-gate"). Also retain the legacy flags as facets of this:
`{PLAN_HAS_HYPOTHESIS}` (scope mentions hypothesis/fuzz/property),
`{PLAN_HAS_XFAIL}` (the plan has a "Production-code changes triggered by
tests" discipline).

### Step 5: Render the prompt

Emit a single ```` ``` ````-fenced markdown block containing the
assembled prompt, filling the template in `<prompt-template>`. Substitute:

| Placeholder                | Source                                                          |
|----------------------------|-----------------------------------------------------------------|
| `{PHASE_REF}`              | Plan mode: `Phase {N}`. Described mode: the short slug label (e.g. `the contact-mechanics integration`). Used wherever the prompt names the unit. |
| `{PHASE_TITLE}`            | The Scope cell (plan mode) or the resolved one-paragraph scope (described mode), trimmed. |
| `{WORK_TYPE}`              | From Step 4 — selects the domain-specific template blocks.       |
| `{SOURCE_PATHS}`           | The concrete artefacts the fresh session must read first: the plan path (plan mode) and/or the latest logbook entry path (the handoff doc) and/or the named code files. |
| `{PLAN_PATH}`              | Full path incl. `plans/active/` (plan mode; else None).         |
| `{PLAN_NAME}`              | Plan filename without `.md` (plan mode; used in the archival closer). |
| `{PLAN_NUMBER}`            | The plan's `Plan N` identity if any (`grep -oE "Plan [0-9]+ Phase" …`); else None — the template wraps `Plan {PLAN_NUMBER}` refs in `{IF PLAN_NUMBER}…{END}`. |
| `{TARGET_PHASE}`           | The target phase integer (plan mode; None in described mode). Used to derive `{TARGET_PHASE_PLUS_ONE}`; not substituted into output directly. |
| `{TARGET_PHASE_PLUS_ONE}`  | `TARGET_PHASE + 1`, or None if final phase / described mode — selects the closer variant. |
| `{PREV_SHA}` / `{PREV_BASELINE}` / `{PREV_LOGBOOK_SLUG}` | From Step 3 (omit the `{IF PREV_SHA}` prior-state sentences if all None). |
| `{COMMIT_PREFIX}`          | From the prior commit's message (`feat(...)` / `fix(...)` / `test(...)` / `docs(...)`); else the generic alternate renders. |

**Conditional template sections.** The template uses `{IF FLAG}…{END}`
and `{IF NOT FLAG}…{END}` blocks, plus `{IF WORK_TYPE == X}…{END}`
blocks. When rendering:

- `{IF FLAG}…{END}` — emit iff `FLAG` is True or the named placeholder
  is non-None / non-empty. `{IF NOT FLAG}…{END}` is the exact complement.
- `{IF WORK_TYPE == X}…{END}` — emit iff the classified work type is `X`.
  Several may be authored; render only the matching block. When the same
  core value (e.g. Robustness) has per-type variants, render the one
  matching `{WORK_TYPE}`; if none matches, render the generic fallback.
- When omitting, drop the entire enclosed text INCLUDING the leading
  bullet/marker/newline. No dangling punctuation, orphan list markers,
  or runs of blank lines.

**Pre-prompt note** (one or two short sentences above the code block),
stating: the resolved mode + rung, the target phase/scope, the work
type, and the prior-state facts. Examples:

- Plan mode, with prior phase:
  > "Targeting Phase 2 of mpc-sadpath-coverage-tiers-1-3 (explicit-plan
  > table; work type: test-coverage; prior phase 1, SHA `f466829`,
  > baseline 1210 passing + 1 xfailed, logbook
  > `2026-05-11-tier1a-real-solver-failures`)."
- Described mode, from a logbook handoff:
  > "Targeting the contact-mechanics integration of bb-led-two-ball-
  > juggle (described mode, from the 2026-06-26 logbook 'Integration
  > plan'; work type: implementation; baseline SHA `519f606`, 1502
  > passing / 4 skipped / 1 xfailed, logbook
  > `2026-06-26-velocity-matched-catch-and-contact-mechanics-feasibility`)."
- Inaugural unit, no prior baseline:
  > "Targeting `<scope>` — inaugural unit, no prior baseline. The
  > receiving agent will record the `git log -1` SHA and `pytest tests/
  > -q` count in this unit's logbook."

### Step 6: Closing note

After the code block, emit one of:

- With prior state:
  > Copy the block above into a fresh Claude session (or `/clear` and
  > paste here). The receiving agent will work the phase end-to-end. If
  > the baseline doesn't match `{PREV_SHA}` / `{PREV_BASELINE}`,
  > investigate before continuing — stale prior-state facts are the most
  > common cause (unpushed work / wrong branch).
- Without prior state:
  > Copy the block above into a fresh Claude session (or `/clear` and
  > paste here). This is the inaugural unit; the receiving agent will
  > record the baseline `git log -1` SHA and `pytest tests/ -q` count in
  > the logbook so future phases have a reference point.

## Important notes

- **Works for phased plans AND described phases.** The resolution ladder
  (Step 1) means a missing phase table is no longer a stop condition —
  only a genuinely undeterminable phase is. Methodology docs and
  milestone docs with no actionable "next" still warrant manual prompting.
- **Prior-state facts always gathered, gracefully degrading.** When the
  most-specific source isn't present, the fact falls to git/logbook/INDEX
  and finally to a "confirm it yourself" instruction for the receiving
  agent — never a silent guess.
- **The rendered prompt is intentionally verbose.** It is load-bearing
  for rigour. Do not trim it for brevity.
- **Don't modify the template's core-values section.** The four values
  (consistency, robustness, determinacy, rigour) are the project's stated
  foundation and appear verbatim in every render, with phase- AND
  work-type-specific actionable interpretations substituted in.
- **The command is read-only.** It never invokes pytest, git push, or any
  state-changing operation — it reads the plan, conversation, git log, and
  logbook only.

## Prompt template

The rendered prompt fills the placeholders and emits a single
```` ``` ````-fenced block.

<prompt-template>
Implement {PHASE_REF}{IF PLAN_PATH} of {PLAN_PATH}{END}:
"{PHASE_TITLE}".

Read first, in this order:
  1. {IF SOURCE_PATHS}{SOURCE_PATHS} — the work spec / handoff. Read it in
     full, with particular attention to: the concrete scope (what to
     build/test/fix), the deferred decisions and design forks left for
     this session, any probe recipes or feasibility findings, and the
     "what was ruled out / why" reasoning so you don't re-litigate settled
     choices.{END}{IF NOT SOURCE_PATHS}The scope described above — there is
     no separate handoff doc; reconstruct the full spec from it and the
     latest logbook entry before starting.{END}
  2. Every file the scope cites (line numbers + symbol names). Confirm
     the citations match ground truth BEFORE writing code — prior audits
     in this project repeatedly caught line-citation drift. Refresh any
     citation that has drifted (symbol moved, lines shifted).
  3. {IF PREV_LOGBOOK_SLUG}{PREV_LOGBOOK_SLUG} — the prior logbook entry.
     Mirror its frontmatter shape, Discussion depth, and cross-reference
     style. A `Logbook-Entry: <slug>` trailer in this unit's commit is
     required.{END}{IF NOT PREV_LOGBOOK_SLUG}If no prior logbook exists,
     model the new entry on a recent resolved entry in logbook/ for
     Discussion depth and frontmatter shape.{END}
  4. CLAUDE.md "Workflow Rules" + "Engineering Philosophy" — re-read
     especially: "Analyze control-system implications before changes",
     "TodoWrite checklist for multi-file tasks", "Run pytest after code
     changes AND before git commit", "Audit multi-document narrative
     changes (/audit --unstaged)", "Grep before refactoring", "Checkpoint
     before sinking effort", "Invite physical-intuition pushback on
     hardware investigations", "Capture user-corrections as memory",
     "Check for parallel-session work before every push".

Core values, non-negotiable throughout (interpretations tuned to this
unit's work type — {WORK_TYPE}):

  • Consistency — match the prior commits' message format
    ({IF COMMIT_PREFIX}`{COMMIT_PREFIX}: … — {IF PLAN_NUMBER}Plan {PLAN_NUMBER} {END}{PHASE_REF}` with a
    `Logbook-Entry: <slug>` trailer{END}{IF NOT COMMIT_PREFIX}follow the
    prefix convention the prior commits established; include a
    `Logbook-Entry: <slug>` trailer{END}; NO backticks inside
    `git commit -m` — use a heredoc / `-F`), logbook frontmatter shape,
    and Discussion structure (Why this approach, What was ruled out,
    Tradeoffs). Reuse helpers / fixtures / patterns established by prior
    work — grep before adding anything that "feels similar".

  • Robustness —
    {IF WORK_TYPE == test-coverage}each scenario test MUST be confirmed to
    drive the real failure path before its assertions are trusted; for
    production-code changes, confirm the regression test fails BEFORE the
    fix and passes AFTER (stash + re-run). {END}{IF WORK_TYPE == implementation}the
    change must survive the REAL system, not a toy probe — validate the
    behaviour end-to-end (run the app / sim / full cycle), not just a unit
    test. If a feasibility probe used a simplified proxy, re-validate
    under realistic conditions before declaring done. {END}{IF WORK_TYPE == investigation}reproduce
    the failure DETERMINISTICALLY before attempting a fix; confirm the fix
    addresses the root cause (the failure no longer reproduces), not just
    a symptom. {END}{IF WORK_TYPE == refactor}the change is
    behaviour-preserving — the full suite is green BEFORE and AFTER with
    no semantic diff; grep ALL references to any renamed/moved symbol,
    count them, and verify the count drops to zero post-change. {END}Run
    `pytest tests/ -q` after each commit; safety-critical tests
    (e.g. tests/sim/test_hot_loop_allocation_contract.py — a KNOWN
    order-dependent tracemalloc flake; confirm it passes in ISOLATION if
    it trips in a full run) must stay green throughout.

  • Determinacy — runs must be reproducible: use seeded RNG where the code
    provides it; a result that varies run-to-run at a fixed seed is a bug
    to root-cause before proceeding, not noise to tolerate.
    {IF PLAN_HAS_HYPOTHESIS}Hypothesis tests run with `--hypothesis-seed=0`
    and must pass at ci-deep (`--hypothesis-profile=ci-deep`,
    max_examples=1000) before the unit is complete; decide whether to
    pre-empt the allocation-contract flake (gc.collect() / `slow` marker)
    now or later and document the choice.{END}

  • Rigour —
    {IF WORK_TYPE == test-coverage}prototype each driver empirically in a
    throwaway `/tmp/probe_*.py` (not committed) before writing the test;
    confirm the recipe is deterministic on the pinned stack; document it
    in the test + the logbook. Decide each design EXPLICITLY (raise vs
    atomic transition; real-driver vs synthetic; xfail vs production
    support) and surface forks to the user via AskUserQuestion BEFORE
    coding.{END}{IF WORK_TYPE == implementation}probe before you integrate
    — prototype the risky mechanism in `/tmp/probe_*.py` (not committed)
    and confirm it works deterministically before wiring it into the
    system. Walk the control-system implications of the change on paper
    (one full cycle) before writing code. Checkpoint before sinking real
    effort: do a brief inline sync on the approach + key assumption and
    pre-register the fallback + the criterion that flips you to it.
    Surface every real design fork to the user via AskUserQuestion before
    building on it.{END}{IF WORK_TYPE == investigation}follow
    diagnose→hypothesis→fix→verify; when a data point contradicts the
    current hypothesis, ABANDON it — don't rescue it. Treat user
    physical-intuition pushback as load-bearing. Write the Discussion
    (what was ruled out, why) before the Fix.{END}{IF WORK_TYPE == refactor}grep
    the entire codebase for every reference before changing a symbol; list
    files + line numbers + total count up front; verify zero after. Don't
    settle for a partial find-and-replace.{END}{IF WORK_TYPE == docs}run
    /audit --unstaged for cross-document consistency; verify every cited
    line/symbol/value against ground truth; cite any test-count claim with
    the (date, command, result) triple.{END}

Process gates, in order:

  1. {IF PREV_SHA}Baseline — `git fetch && git status -sb` (confirm origin
     not ahead, no foreign working-tree changes from a parallel session).
     `git log -1` should show **{PREV_SHA}** or later; `pytest tests/ -q`
     should show **{PREV_BASELINE}**. If the baseline doesn't match,
     investigate before continuing.{END}{IF NOT PREV_SHA}Baseline — run
     `pytest tests/ -q` and `git log -1`; record both as the inaugural
     baseline in this unit's logbook Verification section.{END}

  2. Pre-implementation analysis: confirm code citations against ground
     truth; {IF WORK_TYPE == implementation}walk one full cycle of the
     change (control-system implications); re-run any feasibility probe
     from the handoff doc; {END}{IF WORK_TYPE == test-coverage}empirically
     probe each test driver (`/tmp/probe_*.py`); {END}surface the design
     forks (below) to the user via AskUserQuestion BEFORE writing code.

  3. Implementation, in small units: implement → validate the unit
     ({IF WORK_TYPE == implementation}run the app/sim end-to-end;
     {END}{IF WORK_TYPE == test-coverage}confirm fail-before / pass-after;
     {END}scoped `pytest tests/<subset> -q`) → surface mid-implementation
     pivots; don't drift from the agreed design silently.

  4. Verification gate: full `pytest tests/ -q` — must equal
     {IF PREV_BASELINE}{PREV_BASELINE}{END}{IF NOT PREV_BASELINE}the
     inaugural baseline{END} plus any new tests, no regressions.
     {IF WORK_TYPE == implementation}Validate the unit's behaviour
     end-to-end (the real success oracle, not a proxy metric). {END}{IF PLAN_HAS_HYPOTHESIS}Run
     changed hypothesis tests at ci-deep (`--hypothesis-profile=ci-deep
     --hypothesis-seed=0`). {END}Cite every test-count claim with the
     (date, command, result) triple.

  5. Logbook entry (mirror the prior entry): Summary, Motivation,
     Design/Implementation, Verification (with the triple), a real
     Discussion (Why this approach over alternatives, What was ruled out,
     Tradeoffs accepted{IF PLAN_HAS_XFAIL}, Xfail accounting (test ID +
     tracking reference + target close phase/date){END}), Open Questions.
     Update logbook/INDEX.md (topmost row).

  6. {IF PLAN_PATH}Update {PLAN_PATH}: mark {PHASE_REF} COMPLETE/RESOLVED
     in the summary table AND the detailed section (add an **Outcome**
     paragraph — commit SHA, logbook link, test-count delta with the
     triple, {IF TARGET_PHASE_PLUS_ONE}"Phase {TARGET_PHASE_PLUS_ONE}
     cleared to start" closer{END}{IF NOT TARGET_PHASE_PLUS_ONE}"final
     phase — consider `/archive-plan {PLAN_NAME}` once outstanding items
     resolve" closer{END}).{END}{IF NOT PLAN_PATH}If this unit was tracked
     in a plan or a prior logbook's "next steps", update that artefact to
     mark it done and point to the new logbook entry.{END}

  7. /audit --unstaged on the combined diff (code + tests + logbook +
     plan). Apply LOW/MEDIUM fixes; pause for HIGH-risk. Expect a few
     NOTE-level findings even on clean work.

  8. Commit (one logical unit per behaviour change). Immediately backfill
     the SHA into the logbook frontmatter (same response). `git fetch &&
     git status -sb`, then push.

Ask clarifying questions before assuming any of:
  • {IF WORK_TYPE == implementation}The approach for any mechanism whose
    feasibility is uncertain — surface the alternatives with honest
    tradeoffs (fidelity vs robustness vs effort) and pre-register the
    fallback. The success oracle / acceptance metric for the unit.{END}{IF WORK_TYPE == test-coverage}Real-driver
    vs synthetic for any test where the real path is "hard" — surface the
    tradeoffs (parameter tuning / monkey-patch / stats injection / xfail);
    don't silently choose synthetic.{END}{IF WORK_TYPE == investigation}The
    leading hypothesis and the cheapest experiment that would falsify it,
    before committing effort to a fix.{END}{IF WORK_TYPE == refactor}The
    blast radius (every reference site) and whether any behaviour change
    sneaks in — confirm behaviour-preservation is the contract.{END}
  • Test/file structure when extending an existing area (single file with
    classes vs a split).
  • Commit granularity when the unit has multiple distinct changes —
    separate commits per change is the default.
  • If a NEW issue surfaces orthogonal to this unit's scope — do NOT
    auto-fix; surface it (file it / raise to the user), fixing in-session
    only when the diagnosis is clear, well-scoped, and the user agrees.

Begin by reading {IF SOURCE_PATHS}{SOURCE_PATHS}{END}{IF NOT SOURCE_PATHS}the
scope above{END} in full {IF PREV_SHA}and confirming the baseline
(`git log -1` shows {PREV_SHA} or later; `pytest tests/ -q` shows
{PREV_BASELINE}).{END}{IF NOT PREV_SHA}and recording the current
`git log -1` SHA + `pytest tests/ -q` count as the inaugural baseline in
this unit's logbook.{END}
</prompt-template>
