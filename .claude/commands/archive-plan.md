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

**If READY TO ARCHIVE:**
- Ask the user to confirm archiving

**If NOT READY:**
- Present the list of incomplete items
- Ask the user whether to:
  - **Archive anyway** — with the incomplete items noted in the frontmatter
  - **Address items first** — stop here, fix the issues, then re-run `/archive-plan`
  - **Split** — archive completed portions, create a new plan for remaining work

### Step 4: Execute archival

1. Move the file: `git mv plans/active/<name>.md plans/archived/<name>.md`
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
