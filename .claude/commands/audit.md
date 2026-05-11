---
description: Audit a just-completed code change, then propose and apply fixes. Invoke with /audit [commit-range | --staged | --unstaged | --commits N].
---

# Code Audit & Fix Pipeline

Orchestrates a three-stage pipeline:

1. **Audit** (audit-reporter agent) — read-only analysis, produces findings
2. **User review** — approve/reject/modify findings before proceeding
3. **Fix proposals** (audit-fixer agent) — concrete fixes with risk assessment
4. **Apply** — auto-apply LOW/MEDIUM risk fixes; pause for HIGH risk approval
5. **Wrap-up** — run tests, summarize, prompt for `/commit`

## Arguments

- **No arguments**: audit all uncommitted changes (staged + unstaged) against HEAD
- **`<commit-range>`**: audit a specific range (e.g. `HEAD~3..HEAD`, `abc123..def456`)
- **`--staged`**: audit only staged changes
- **`--unstaged`**: audit only unstaged changes
- **`--commits N`**: audit the last N commits on the current branch

$ARGUMENTS

---

## Stage 1: Audit

Spawn the **audit-reporter** agent with the following prompt, constructed from the
arguments above:

```
Audit the code changes in this repository.

Diff source: <one of the following based on arguments>
- (no args): all uncommitted changes — run `git diff` + `git diff --cached` + `git status`
- <range>: `git diff <range>` + `git log --oneline <range>`
- --staged: `git diff --cached`
- --unstaged: `git diff`
- --commits N: `git diff HEAD~N..HEAD` + `git log --oneline HEAD~N..HEAD`

Follow your full audit process: gather diff, read all changed files in full,
evaluate each audit category, and produce the structured findings report.
```

When the agent returns, present the full audit report to the user.

### Review Gate 1

After presenting the audit report, present **your own recommendation for each
finding** before asking the user to choose. The user wants Claude's judgment on
the table, not just a passive list. Do not skip this — passive presentation
forces the user to do the synthesis Claude should be doing.

For each finding, give a one-sentence recommendation: **Approve / Skip** plus
the specific reason that drives the call. Examples of good recommendations:

- *"**Approve** — typo fix, ~5 chars, no behaviour change."*
- *"**Approve** — the misleading docstring will trip the next reader who
  cross-checks against `_cold_start`. Doc-only."*
- *"**Skip** — the audit flagged a missing comment but the code is
  self-explanatory at the call site. Comment would be noise."*
- *"**Approve with caveat** — fix is correct but the audit missed a ripple
  in `foo.py:142`; I'll include it when I propose the fix."*

Then close with:

> **Recommendation: <approve all / approve N,N / skip / mixed — see per-finding>.**
>
> You can:
> - **approve** — proceed to fix proposals for all findings
> - **approve N,N,...** — proceed with only the listed finding numbers
> - **reject** — stop here, no fixes needed
> - **comment** — add context or corrections before proceeding
>
> Which findings should I propose fixes for?

**Do not proceed past this gate without explicit user approval.**

If the verdict is **CLEAN**, report it and stop — do not ask about fixes.

---

## Stage 2: Fix Proposals

Spawn the **audit-fixer** agent with the approved findings:

```
Propose fixes for the following approved audit findings:

<paste only the approved findings here, preserving the exact format>

For each finding, read the full source file, search for ripple effects, and
propose a concrete minimal fix. Follow your full fix-proposal process and
produce the structured output with before/after snippets, risk assessment,
and test guidance.
```

When the agent returns, present the full fix proposals to the user, then proceed
based on risk level.

### Auto-Apply vs Review Gate 2

The user has already acknowledged the findings in Review Gate 1. Fix proposals
inherit that acknowledgment — **a second approval is only needed for HIGH risk
fixes**.

**Categorize each fix by its risk level** (as assessed by the audit-fixer agent):

- **LOW risk** — typo, missing import, stale reference, simple off-by-one.
  Auto-apply immediately after presenting.
- **MEDIUM risk** — logic change, signature update, ripple effects confined to
  tests or a single caller. Auto-apply immediately after presenting.
- **HIGH risk** — safety-critical code (motor_guard, workspace, fault detection),
  IPC/CAN protocol changes, changes with broad ripple effects, or anything the
  audit-fixer agent flagged as HIGH. **Pause and ask for explicit approval.**

After presenting the fix proposals, proceed as follows:

1. If **all fixes are LOW/MEDIUM**: announce "All fixes are low/medium risk —
   applying now." and proceed directly to Stage 3.
2. If **some fixes are HIGH**: announce which fixes will be auto-applied and which
   need approval. Apply the LOW/MEDIUM fixes immediately, then ask:

> **The following HIGH-risk fixes need your approval:**
> <list the HIGH-risk fixes with their risk rationale>
>
> - **approve all** — apply all remaining fixes
> - **approve N,N,...** — apply only the listed fix numbers
> - **reject** — skip these, keep only the already-applied fixes
> - **modify** — describe what you'd change before I apply
>
> Which HIGH-risk fixes should I apply?

3. If **all fixes are HIGH**: present them and ask for approval before applying any.

**Do not auto-apply HIGH-risk fixes without explicit user approval.**

---

## Stage 3: Apply Fixes

For each fix (auto-approved or explicitly approved), in the recommended order from
the fix proposals:

1. **Read the target file** to get the current state
2. **Apply the change** using the Edit tool
3. **Verify** — if the fix proposal mentioned ripple effects, apply those too
4. **Report** what was changed: file, line, before → after (one-line summary)

After all fixes are applied:

1. Run `pytest tests/ -v` to verify nothing is broken
2. Report the test results
3. Summarize all changes made

If any test fails after applying fixes:
- Report which test failed and why
- Ask the user whether to revert the change or investigate further
- **Do not silently revert or modify** — the user decides

---

## Stage 4: Wrap-up

After all fixes are applied and tests pass (or the user has acknowledged failures):

1. Present a final summary:

> ### Audit Complete
>
> **Applied N fixes** (N auto-applied, N user-approved)
> **Tests**: all passing / N failures acknowledged
> **Files changed**: `file1.py`, `file2.py`, ...
>
> Ready to commit? Run `/commit` to review and commit these changes.

2. If there are no remaining issues and the working tree has changes, suggest
   `/commit` as the natural next step. Do not run it automatically — just prompt.

---

## Constraints

- **Review Gate 1 is mandatory.** The user must acknowledge findings before any fixes
  are proposed or applied.
- **Review Gate 2 is risk-gated.** LOW/MEDIUM fixes auto-apply after the user has
  acknowledged the finding in Gate 1. HIGH-risk fixes always require explicit
  approval at Gate 2.
- **Agents are read-only.** Only the main conversation (Stage 3) modifies files.
- **Respect the user's selections.** If they approved only findings 1 and 3, do not
  sneak finding 2 into the fix proposals. If they approved only fix 1, do not apply
  fix 2.
- **No bonus improvements.** Do not refactor, clean up, add comments, or "improve"
  code beyond the scope of the approved fixes.
- **Be transparent about test failures.** If tests fail after fixes, present the
  information clearly and let the user decide the path forward.
- **Always suggest `/commit` when done.** If the pipeline produced changes and tests
  pass, prompt the user to run `/commit` — but never run it automatically.
