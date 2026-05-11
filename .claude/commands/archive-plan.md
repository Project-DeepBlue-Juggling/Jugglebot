---
description: Archive a completed plan — critically review it first, then move from plans/active/ to plans/archived/. Invoke with /archive-plan <plan-name>.
---

# Archive Plan Command

Move a plan from `plans/active/` to `plans/archived/` after verifying it is truly complete.

## Arguments

- **`<plan-name>`**: the plan filename (with or without .md, with or without `plans/active/` prefix)

## Protocol

### Step 1: Locate the plan

1. Find the plan file in `plans/active/`
2. If not found, check `plans/archived/` — if already archived, inform the user
3. If not found anywhere, list available active plans

### Step 2: Critical review

Spawn the **plan-reviewer** agent with the full plan content. The plan-reviewer will:
- Check every phase/milestone against the actual codebase
- Verify claimed completions (code exists, tests pass, no skipped items)
- Check for loose ends, stale references, and open logbook entries
- Produce a structured review with a READY TO ARCHIVE / NOT READY verdict

Present the review to the user.

### Step 3: Decision

Before deciding READY/NOT READY, **always surface the "Residual
xfail markers" section** from the plan-reviewer's output as a
distinct, user-visible block — even when it's empty (empty is
meaningful evidence that the plan added no permanent xfails).  For
each marker classified:

- **DOCUMENTED PERMANENT** — list it in a "Permanent xfails to
  carry forward" sub-block.  Even if the marker's `reason=` is
  well-documented in code, the user must explicitly acknowledge
  that they accept the xfail as permanent at archival time.  The
  acknowledgement creates a checkpoint for periodic re-audit; a
  marker that nobody has acknowledged in 6 months may have lost
  its load-bearing justification.
- **UNDOCUMENTED** — block archival pending either (a) the user
  documenting the marker's justification (target close phase, issue
  reference, or "accept as permanent" decision) and committing that
  update, or (b) the user explicitly waiving the requirement.

**If READY TO ARCHIVE:**
- Surface the xfail block as above.
- If any DOCUMENTED PERMANENT xfails exist, ask the user to confirm
  each one is still load-bearing ("Accept `<test_id>` as permanent
  / Re-audit later / Block archival to fix now").
- If all xfails are accepted, ask the user to confirm final
  archiving.

**If NOT READY:**
- Present the list of incomplete items.
- Surface any UNDOCUMENTED xfails as a separate "must resolve"
  block (they block archival independently of the completion
  check).
- Ask the user whether to:
  - **Archive anyway** — with the incomplete items AND any
    accepted-permanent xfails noted in the frontmatter (under a
    new `residual_xfails:` key listing test IDs).  Example
    frontmatter shape:
    ```yaml
    residual_xfails:
      - "tests/sim/test_solver_failures.py::TestRestorationFailedNotDrivable"
    ```
    Each entry is a pytest node-ID (the same form `pytest <id>`
    accepts).  Future re-audits can `grep residual_xfails:`
    across `plans/archived/` to enumerate the project's
    permanently-acceptable xfail surface.
  - **Address items first** — stop here, fix the issues / document
    the xfails, then re-run `/archive-plan`.
  - **Split** — archive completed portions, create a new plan for
    remaining work (the new plan inherits any unresolved xfails).

### Step 4: Execute archival

1. Move the file with completion date prefix:
   `git mv plans/active/<name>.md "plans/archived/YYYY-MM-DD <name>.md"`
   where YYYY-MM-DD is today's date (the completion date)
2. Update the frontmatter:
   - Set `status: completed` (or `superseded` if replaced by a new plan)
   - Add `completed: YYYY-MM-DD`
3. Check if any logbook entries reference this plan via `related_plan:`
   - If so, update the path in those entries
4. Check if anything else in the codebase references the old path
   - Update any references found (e.g., in other plans, CLAUDE.md, command files)

### Step 5: Report

Summarize what was done:
- Plan archived to `plans/archived/<name>.md`
- N logbook entries updated
- N cross-references updated
- Any remaining action items

## Important Notes

- The plan-reviewer agent is the critical gate — it prevents premature archiving
- The reviewer checks the actual codebase, not just the plan's claims
- Plans with `status: active` in `plans/active/` are the only candidates
- If a plan is superseded (not completed), use `status: superseded` instead of `completed`
