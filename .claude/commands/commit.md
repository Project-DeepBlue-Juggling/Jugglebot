---
description: Survey uncommitted changes, propose commit groupings, and execute commits after approval. Invoke with /commit.
disable-model-invocation: true
---

# Git Commit Manager

Survey all uncommitted changes in the repository, propose sensible commit groupings with clear messages, get explicit user approval, then execute the commits.

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

Then explicitly ask:
> "Does this look right? You can approve as-is, ask me to re-group, rename any messages, or exclude any files before I proceed."

**Do not run any git commands that modify state until the user has explicitly approved.**

## Phase 3: Execute (only after explicit approval)

Once the user approves (or after incorporating any requested changes):

For each commit in order:
1. `git add` only the specific files for that commit (never `git add .` or `git add -A`)
2. Create the commit. Always pass the message via HEREDOC:
   ```bash
   git commit -m "$(cat <<'EOF'
   commit message here

   Co-Authored-By: Claude Opus 4.6 (1M context) <noreply@anthropic.com>
   EOF
   )"
   ```
3. Confirm success before moving to the next commit

After all commits are done, report the final state:
- Branch name
- Number of commits created
- Commit hashes and messages

Do NOT push unless the user explicitly asks. If the user asks to push and it is rejected (e.g. remote has diverged), report the error clearly and ask how to proceed.

## Constraints
- Never use `git add .` or `git add -A` — always stage files explicitly
- Never amend, rebase, or force push without explicit user instruction
- Never proceed past Phase 2 without clear user approval
- Never push without explicit user instruction
- If anything looks ambiguous or risky, ask rather than assume
