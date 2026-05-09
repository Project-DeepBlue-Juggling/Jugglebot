---
description: Survey uncommitted changes, propose commit groupings, and execute commits after approval. Invoke with /commit.
disable-model-invocation: true
---

# Git Commit Manager

Survey all uncommitted changes in the repository, propose sensible commit groupings
with clear messages, and execute them. Low/moderate complexity commits auto-execute
after presenting the plan; high complexity commits require explicit user approval.

## Phase 1: Survey

Run the following in parallel to understand the full state of the working tree:
- `git status` — overview of staged, unstaged, and untracked files
- `git diff` — detailed changes to tracked files
- `git diff --cached` — any already-staged changes
- `git log --oneline -5` — recent commit history, to match tone/style of existing messages

If the working tree is completely clean (no changes, no untracked files), report back clearly:
- The current branch name
- The last commit hash and message
- That there is nothing to commit

Do not proceed further if the tree is clean.

## Phase 2: Propose Groupings

Analyse the changes and group them into logical commits. Good groupings:
- Separate concerns (e.g. a bug fix should not share a commit with a refactor)
- Keep related files together (e.g. a new module and its tests)
- Respect conventional commit prefixes where appropriate: `feat:`, `fix:`, `refactor:`, `chore:`, `docs:`, `test:`
- Are ordered sensibly (e.g. foundational changes before things that depend on them)

Present your proposed groupings in a clear, structured format. For each proposed commit, show:
1. The commit message
2. The files included and why they're grouped together
3. A one-line rationale for the grouping

Example format:
---
**Commit 1:** `feat: add quintic trajectory interpolation for hand motion`
- `controller/hermite.py` — new quintic_interp implementation
- `tests/test_hermite.py` — unit tests for the above
- *Rationale: new self-contained feature with its tests*

**Commit 2:** `fix: correct rotation matrix sign in global_to_bb_frame()`
- `sim/hand/ballistics.py` — sign bug fix
- *Rationale: isolated bug fix, separate from feature work*
---

Then assess the **complexity** of the overall commit plan:

**LOW complexity** — all of:
- Single commit (or multiple commits that are all trivially separable)
- Changes are small, self-contained, and obvious (config, docs, tooling, small fixes)
- No safety-critical files (motor_guard, workspace, fault detection, CAN encoding)
- No ambiguity in grouping — there's clearly only one sensible way to commit these

**MODERATE complexity** — any of:
- 2-3 commits with clear, unambiguous groupings
- Moderate-sized changes where the intent is obvious from the diff
- No safety-critical files

**HIGH complexity** — any of:
- Ambiguous grouping (multiple reasonable ways to split the commits)
- Safety-critical files touched (motor_guard, workspace, fault detection, CAN, IPC)
- Large or cross-cutting changes where the commit message needs careful framing
- Mixed concerns that could reasonably be grouped differently

### Approval Gate

**LOW/MODERATE complexity**: present the plan, then announce "Straightforward
changes — committing now." and proceed directly to Phase 3. Do not wait for
approval.

**HIGH complexity**: present the plan and ask:
> "Does this look right? You can approve as-is, ask me to re-group, rename any
> messages, or exclude any files before I proceed."

**Do not proceed past Phase 2 for HIGH complexity commits without explicit user
approval.**

## Phase 3: Execute

For LOW/MODERATE, execute immediately after presenting the plan. For HIGH, execute
once the user approves (or after incorporating requested changes).

For each commit in order:
1. `git add` only the specific files for that commit (never `git add .` or `git add -A`)
2. **Verify the staged set matches expectations.** Run `git diff --cached --stat`
   and read the output.  If it lists ONLY the files just added, proceed.  If
   it lists ANY extra file, stop and surface to the user before committing —
   the user may have staged work in parallel, or a hook may have auto-staged
   something.  Skipping this check has caused real attribution incidents
   (see `feedback_verify_staged_before_commit` in auto-memory).
3. Create the commit. Always pass the message via HEREDOC:
   ```bash
   git commit -m "$(cat <<'EOF'
   commit message here

   Co-Authored-By: Claude Opus 4.6 (1M context) <noreply@anthropic.com>
   EOF
   )"
   ```
4. Confirm success before moving to the next commit

### Logbook-Entry trailer

If the current conversation has an active logbook entry (created by `/investigate`,
`/log`, or `/logbook --new`), add a `Logbook-Entry:` trailer to commits that
contain the code changes described by that entry:

```bash
git commit -m "$(cat <<'EOF'
fix: cold-hold fallback holds at current position instead of stroke minimum

Logbook-Entry: 2026-04-01-cold-hold-fallback-stroke-minimum
Co-Authored-By: Claude Opus 4.6 (1M context) <noreply@anthropic.com>
EOF
)"
```

This enables one-hop traceability: `git blame` → commit → `Logbook-Entry` trailer
→ full logbook entry with context, discussion, and outcome.

Only add the trailer to commits containing the actual code fix/change — not to
commits that only update the logbook entry itself.

After all commits are done, report the final state:
- Branch name
- Number of commits created
- Commit hashes and messages

### Phase 4: Suggest logbook entry

After committing, check whether the committed files are covered by an existing
logbook entry. To check: grep `logbook/` for the committed file paths in
`files_changed:` frontmatter.

If **no logbook entry** covers these files, suggest:
> "These changes don't have a logbook entry. Consider running:
> `/log <suggested-type> <suggested-title-from-commit-message>`
> or `/log --from-commits <hash>` to document this change."

Only suggest — never create a logbook entry automatically. Skip the suggestion
if the commit only touches documentation, test files, or logbook files themselves.

Do NOT push unless the user explicitly asks. If the user asks to push and it is rejected (e.g. remote has diverged), report the error clearly and ask how to proceed.

## Constraints
- Never use `git add .` or `git add -A` — always stage files explicitly
- **Always run `git diff --cached --stat` between `git add` and `git commit`** —
  catches parallel adds, auto-stage hooks, and leftover staged state.  Stop
  if the staged set has anything other than what was just added.
- Never amend, rebase, or force push without explicit user instruction
- Never proceed past Phase 2 for HIGH complexity commits without clear user approval
- Never push without explicit user instruction
- If anything looks ambiguous or risky, ask rather than assume
