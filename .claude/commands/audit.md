---
description: Audit a just-completed code change, then propose and apply fixes. Invoke with /audit [commit-range | --staged | --unstaged | --commits N].
---

# Code Audit & Fix Pipeline

Orchestrates a three-stage pipeline with user review gates between each stage:

1. **Audit** (audit-reporter agent) — read-only analysis, produces findings
2. **User review** — approve/reject/modify findings before proceeding
3. **Fix proposals** (audit-fixer agent) — concrete fixes with risk assessment
4. **User review** — approve which fixes to apply
5. **Apply** — execute approved fixes in the main conversation

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

After presenting the audit report, ask:

> **Review the findings above.** You can:
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

When the agent returns, present the full fix proposals to the user.

### Review Gate 2

After presenting the fix proposals, ask:

> **Review the proposed fixes above.** You can:
> - **apply all** — I'll make all the proposed changes
> - **apply N,N,...** — I'll apply only the listed fix numbers
> - **reject** — stop here, no changes made
> - **modify** — describe what you'd change about a proposal before I apply it
>
> Which fixes should I apply?

**Do not proceed past this gate without explicit user approval.**

---

## Stage 3: Apply Fixes

For each approved fix, in the recommended order from the fix proposals:

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

## Constraints

- **Never skip a review gate.** The user must explicitly approve before the pipeline
  advances to the next stage.
- **Agents are read-only.** Only the main conversation (Stage 3) modifies files.
- **Respect the user's selections.** If they approved only findings 1 and 3, do not
  sneak finding 2 into the fix proposals. If they approved only fix 1, do not apply
  fix 2.
- **No bonus improvements.** Do not refactor, clean up, add comments, or "improve"
  code beyond the scope of the approved fixes.
- **Be transparent about test failures.** If tests fail after fixes, present the
  information clearly and let the user decide the path forward.
