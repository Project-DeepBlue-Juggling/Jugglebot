---
description: Investigate a hardware issue end-to-end — diagnose, log, discuss, fix, test, commit. Invoke with /investigate [csv_filename | --latest | --resume <entry>].
---

# Hardware Investigation Agent

Orchestrate the full symptom-to-fix cycle for a hardware test session. Each step
is gated by user approval. The investigation creates or updates an engineering
logbook entry that captures the full arc: symptom, diagnosis, discussion, fix, outcome.

## Arguments

- **No args** or **`--latest`**: investigate the most recent unanalysed session
- **`<csv_filename>`**: investigate a specific session
- **`--resume <entry>`**: resume an existing logbook entry (filename with or without .md)
- **`--sessions csv1,csv2,...`**: investigate multiple related sessions together
- **`--dry-run`**: diagnose and create logbook entry, but skip fix/implement/commit steps

## Protocol

### Step 1: Determine scope

**If resuming an existing entry:**
1. Read the entry file from `logbook/`
2. Determine which step to resume from based on which sections have content:
   - Empty Summary/Symptoms/Diagnosis → resume at Step 2
   - Empty Discussion → resume at Step 4
   - Empty Fix → resume at Step 5
   - Empty Outcome → resume at Step 9
   - All sections filled but status != resolved → ask user what to do
3. Skip to the appropriate step

**If starting fresh:**
1. Determine target CSV(s) using the same logic as `/diagnose`:
   read `sim/analysis/log_index.json`, find latest unanalysed or specified file
2. **Auto-group related sessions:** Check `log_index.json` for other CSVs that share
   the same `rosbag` path or have timestamps within 1 hour of the target. If found,
   offer to include them all in a single investigation:
   > "Found 4 related sessions from the same rosbag session (2026-04-01_13-18-01).
   > Include all in this investigation, or just the specified one?"
3. Check if any existing logbook entry already references these sessions
   (grep `logbook/` for the CSV filename)
4. If found, ask the user whether to update the existing entry or create a new one

### Step 2: Diagnose

Run the `/diagnose` protocol on the target session(s):
1. Run `python3 sim/analysis/diagnose.py <csv_path> [--rosbag <path>] --json`
2. Cross-reference against `sim/analysis/known_issues.yaml`
3. Read `plans/active/hardware-bringup.md` for phase context

Present the diagnosis report.

---

### Gate: DIAGNOSE

Present the diagnosis findings to the user.

**Options:**
- **proceed** — create/update logbook entry with symptoms + diagnosis, continue to fix discussion
- **skip-to-fix** — user already knows the fix, skip discussion
- **stop** — save diagnosis to logbook entry and stop here (can resume later with `--resume`)

---

### Step 3: Create/update logbook entry

Spawn the **logbook-updater** agent to:
1. Generate a slug from the most prominent issue
2. Create `logbook/YYYY-MM-DD-<slug>.md` with `type: investigation`
3. Fill in frontmatter:
   - title, type, date, phase, sessions, related_issues
   - `subsystem:` — auto-detect from files referenced in the diagnosis flags
   - `tags:` — use controlled vocabulary (see TEMPLATE.md)
   - `files_changed:` — leave empty (populated after fix is implemented)
   - `commits:` — leave empty (populated after committing)
4. Fill in: Summary (from verdict), Symptoms, Diagnosis sections
5. Set status to `in-progress`
6. Update `logbook/INDEX.md`
7. Update `sim/analysis/log_index.json` with `logbook_entry` pointers

**If `--dry-run` was specified:** Stop here. The logbook entry has been created with
the diagnosis. Report the entry path and exit. The entry can be resumed later with
`/investigate --resume <entry>` when the user is ready to work on fixes.

---

### Step 4: Discuss potential fixes

Spawn the **fix-proposer** agent with:
- The diagnosis report (verdict, flags, key metrics)
- Flagged issue details and known_issues.yaml references
- The phase context from the bringup plan

The fix-proposer will read relevant source code and propose 1-3 fixes with
risk assessment and hardware re-test requirements.

---

### Gate: FIX_PLAN

Present the proposed fixes to the user.

**Options:**
- **proceed <N>** — implement fix option N (or the recommended one)
- **discuss** — ask questions or propose alternatives before deciding
- **stop** — save discussion to logbook entry and stop here

---

### Step 5: Implement the fix

Make the agreed code changes. This is normal code editing — no special agent needed.

Update the logbook entry's Fix section (via logbook-updater) with:
- Files modified and description of each change
- The rationale linking back to the diagnosis

---

### Gate: IMPLEMENTATION

Present the code changes (show `git diff`).

**Options:**
- **proceed** — run tests
- **revise** — modify the implementation
- **revert** — undo changes and return to discussion
- **stop** — save implementation details to logbook entry and stop here

---

### Step 6: Test

1. Run `pytest tests/ -v`
2. Report results (count, pass/fail)
3. If hardware-specific changes, suggest which hardware test to re-run
   (referencing the relevant phase from `plans/active/hardware-bringup.md`)

---

### Gate: TEST

Present test results.

**Options:**
- **proceed** — commit the changes
- **fix-tests** — tests failed, iterate on the fix
- **stop** — save test results to logbook entry and stop here

---

### Step 7: Commit

Follow the project's commit convention:
1. Stage the relevant files (code changes + logbook entry + index updates)
2. Propose a commit message with conventional prefix (fix:, feat:, refactor:, etc.)
3. Include `Logbook-Entry: <entry-slug>` trailer on the code-change commit
4. Wait for user approval before executing

---

### Gate: COMMIT

Present the proposed commit (files to stage + message).

**Options:**
- **proceed** — create the commit
- **modify** — change the commit message or file grouping
- **stop** — stop before committing

---

### Step 8: Push (optional)

---

### Gate: PUSH

**Options:**
- **push** — push to remote
- **stop** — leave as local commit only

---

### Step 9: Update logbook outcome

Spawn the **logbook-updater** agent to:
1. Populate `files_changed:` in frontmatter from `git diff --name-only` of the commit(s)
2. Populate `commits:` in frontmatter with the commit hash(es)
3. Fill in the Outcome section:
   - Test results (pass/fail, key metrics)
   - Commit hash and message
   - Whether pushed
4. If all symptoms are addressed: set status to `resolved`
5. If partially addressed: keep status `in-progress`, note what remains in Open Questions
6. Update `logbook/INDEX.md` with new status

## Gating Design

Each gate is a **named decision point** with explicit options. The current design
gates every step. To relax gating in the future:
- Add `--auto <gate1,gate2,...>` to auto-proceed on specific gates
- Or add `--auto-all` to auto-proceed on everything except PUSH
- The named-gate pattern means individual gates can be selectively relaxed

## Important Notes

- This command **does modify code** (Steps 5-7). Exercise the same caution as any code change.
- Always present `git diff` before committing.
- The logbook entry is the primary artifact — even if the user stops early, the entry captures progress so far and can be resumed.
- Use the same verdict criteria as `/diagnose` (PASS / NEEDS_ATTENTION / FAIL).
- Each agent (fix-proposer, logbook-updater) runs in its own context to keep scope focused.
