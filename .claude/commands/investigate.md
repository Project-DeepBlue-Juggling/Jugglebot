---
description: Investigate a hardware issue end-to-end — diagnose, log, discuss, fix, test, commit. Invoke with /investigate [csv_filename | --latest | --resume <entry>].
---

# Hardware Investigation Agent

Orchestrate the full symptom-to-fix cycle for a hardware test session. Each step
is gated by user approval. The investigation creates or updates an engineering
logbook entry that captures the full arc: symptom, diagnosis, discussion, fix, outcome.

## Arguments

- **No args** or **`--latest`**: investigate the most recent session group. A session group is a set of CSVs that share a rosbag — all moves in one operational run. `--latest` finds every CSV belonging to the most-recent rosbag's `session_group` (see `/diagnose` → `result['session_group']` in `log_index.json`), not just the single most recent CSV. Ask the user whether to include all moves in the group or just a specific one.
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
   the same `session_group` (populated by `/diagnose` from the rosbag directory name)
   or — for older entries that predate `session_group` — the same `rosbag` path, or
   timestamps within 1 hour of the target. If found, offer to include them all in a
   single investigation:
   > "Found 4 related sessions from session-group 2026-04-01_13-18-01.
   > Include all in this investigation, or just the specified one?"
3. Check if any existing logbook entry already references these sessions
   (grep `logbook/` for the CSV filename)
4. If found, ask the user whether to update the existing entry or create a new one

### Step 2: Diagnose

Run the `/diagnose` protocol on the target session(s):
1. Run `python3 sim/analysis/diagnose.py <csv_path> [--rosbag <path>] --json`
2. Cross-reference against `sim/analysis/known_issues.yaml`
3. Read `plans/active/hardware-bringup.md` for phase context

**Flush `sim/analysis/log_index.json` immediately** after each diagnose run, not
just at the very end of `/investigate`. The user may drop out of the investigation
mid-flow (hitting "stop" on any gate, or just context-switching); if the index
isn't updated here, the session is still marked unanalysed on the next `/diagnose`
and the work is duplicated. Minimum fields to flush per session:
`analyzed`, `last_analyzed`, `verdict`, `flags_count`, `rosbag`, `session_group`,
plus `logbook_entry` if Step 3 has already run for this investigation.

Present the diagnosis report.

---

### Gate: DIAGNOSE

Present the diagnosis findings to the user.

**Options:**
- **proceed** — create/update logbook entry with symptoms + diagnosis, continue directly to `fix-proposer` (Step 4)
- **discuss** — create/update the logbook entry, then pause for open-ended back-and-forth with the user **without** spawning the fix-proposer agent. Use this when the diagnosis raises design-level questions or the user wants to explore the data before committing to a fix direction. The fix-proposer is opinionated and expensive; it should only run when the user says "propose fixes" explicitly. When the user is ready, continue to Step 4.
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

Extract flag types and subsystem tags from the diagnosis to enable prior art search:
- **Flag types:** classify each flag message into: `oscillation`, `discontinuity`,
  `solve_time`, `tracking`, `workspace`, `torque` (based on the flag content)
- **Subsystems:** extract from the flagged file paths (e.g., `controller/` → `controller`,
  `motion/` → `motion`) and from the logbook entry's `subsystem:` frontmatter

Spawn the **fix-proposer** agent with:
- The diagnosis report (verdict, flags, key metrics)
- Flagged issue details and known_issues.yaml references
- The phase context from the bringup plan
- The flag types and subsystem tags as JSON arrays (for logbook prior art search)

The fix-proposer will search the logbook for similar past investigations, then
read relevant source code and propose 1-3 fixes with risk assessment and
hardware re-test requirements.

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
- **proceed** — continue to Step 5b (verify-applied) before running tests. For source-resident changes (Python-only on the Jetson, no CAN/IPC push), the subsequent VERIFY_APPLIED gate accepts `skip` and goes straight to TEST.
- **revise** — modify the implementation
- **revert** — undo changes and return to discussion
- **stop** — save implementation details to logbook entry and stop here

---

### Step 5b: Verify the fix actually landed on the target device

**This step is mandatory when the change modifies a value that lives on a
second device.** Examples: ODrive gains, ODrive control / input modes,
Teensy parameters, remote config pushed via IPC, any constant that is
loaded at boot rather than read live. Skip this step only when the change
is purely source-resident (Python-only logic on the Jetson, no CAN/IPC
push involved).

Why this gate exists: on 2026-04-19 an A/B leg-gain test produced misleading
metrics because the gain setter was only called during homing, not on
activation; the user had activated without re-homing, so the YAML edit and
rebuild had no runtime effect. The bug was only caught because the user
eyeballed ODriveGUI. A protocol-level check would have saved 30 minutes.

Explicitly ask the user to verify the new value is present on the target
hardware BEFORE running the performance test:

> "Before we measure, please confirm the new value is live on the target
> device. Options: (a) check ODriveGUI for gain values with an explicit
> config re-read, (b) issue an odrivetool query that reads the live
> register, (c) grep the startup log for the value the code claims to
> have applied. Report the observed value here."

Record the observation in the logbook Fix section as "Verified live
value: X". If the observed value doesn't match the source code's
intended value, something is wrong — do NOT run the performance test
yet; go back to the fix, because the test will otherwise produce
nonsense data that wastes a hardware session.

---

### Gate: VERIFY_APPLIED

**Options:**
- **confirmed** — the live value on the target matches the fix's intended value; proceed to TEST
- **mismatch** — the live value is different from the intended value. Return to Step 5 (fix the application path). Do NOT run the performance test on a mismatched config.
- **skip** — this change is source-resident only, no device-level push; verification N/A. Proceed to TEST.

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

**If a re-test was performed after the fix**, suggest comparing the before/after sessions:
> "Run `/diagnose --compare <pre_fix_csv> <post_fix_csv>` to see a side-by-side
> comparison of metrics before and after the fix."
The pre-fix CSV is available from the logbook entry's `sessions:` frontmatter field.

Spawn the **logbook-updater** agent to:
1. Populate `files_changed:` in frontmatter from `git diff --name-only` of the commit(s)
2. Populate `commits:` in frontmatter with the commit hash(es)
3. Fill in the Outcome section:
   - Test results (pass/fail, key metrics)
   - Commit hash and message
   - Whether pushed
4. Set status according to this ladder:
   - `resolved` — every symptom in the entry's scope is addressed and verified; no open follow-ups inside this entry's scope
   - `tuned` — the specific thing this entry was investigating is addressed (e.g. hold-phase fighting brought into spec) but the entry intentionally leaves a sibling investigation open elsewhere. Use this when a committed change ships a real improvement but the entry's Open Questions or a sibling logbook entry still have work. This is better than stranding entries at `in-progress` forever.
   - `in-progress` — neither of the above; the investigation is genuinely still unfinished
5. Update `logbook/INDEX.md` with new status
6. If the entry's Discussion section references a methodology plan in its
   frontmatter (`related_plan:`), ensure that plan is linked inline from the
   body text of the Discussion section (not just the frontmatter). A reader
   landing in the entry from `git blame` should see the plan link in prose,
   not have to read ahead to find it.

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
