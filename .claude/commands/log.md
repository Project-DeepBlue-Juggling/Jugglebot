---
description: Log a code change, refactor, feature, or optimization to the engineering logbook. Invoke with /log <type> <title> (types — bugfix, refactor, feature, optimization).
---

# Log Entry Command

Create an engineering logbook entry for a code change that isn't a hardware
investigation. Use this for refactors, features, bugfixes (non-hardware), and
optimizations. For hardware diagnosis workflows, use `/investigate` instead.

## Arguments

- **`<type> <title>`**: entry type and descriptive title
  - Types: `bugfix`, `refactor`, `feature`, `optimization`
  - Example: `/log refactor Extract MPC orchestration from sim/ into controller/`
- **`--from-commits <hash1,hash2,...>`**: create an entry from existing commits (retroactive logging)
- **`--wip`**: mark as in-progress (default is to walk through the full flow)

## Protocol

### Step 1: Gather context

**If `--from-commits` is provided:**
1. Run `git show --stat <hash>` for each commit to get files changed and commit messages
2. Run `git diff <hash>~1..<hash>` to understand the actual changes
3. Auto-populate: title (from commit message), files_changed, date. Do **not**
   add a `commits:` frontmatter field — the commit's `Logbook-Entry:` trailer is
   the canonical link (convention retired 2026-08-01)

**If starting fresh (no commits yet):**
1. Ask the user what they're about to do (or have just done)
2. If there are uncommitted changes, run `git diff --stat` to identify files involved
3. If recent commits exist on the branch, offer to include them

### Step 2: Classify and tag

Based on the type and files involved:
1. Auto-suggest `subsystem:` tags from the file paths:
   - `controller/` → `controller`
   - `ros_ws/.../motion/` → `motion`
   - `ros_ws/.../can/` or `can_node.py` → `can`
   - `ros_ws/.../tracking/` → `tracking`
   - `sim/` → `sim`
   - `ros_ws/gui/` → `gui`
   - `config/` → `config`
   - (`mpc` is historical — its files were removed 2026-09-01, tag `mpc-final`)
   - `ros_ws/.../orchestrator_node.py` or `state_machine.py` → `ros`
2. Auto-suggest additional `tags:` based on content:
   - Changes to `*_test.py` or `test_*.py` → `testing`
   - Changes to `*.md` or `docs/` → `docs`
   - Changes to `motor_guard.py`, workspace checks, fault detection → `safety`
   - Changes to IPC, ZMQ → `IPC`
   - Changes to timing, rates, solve times → `performance`
3. Present suggestions and let the user confirm or modify

### Step 3: Create the logbook entry

Spawn the **logbook-updater** agent to create the entry:
1. **Short form is the default** (10–30 lines): what changed, why, and the
   `(date, command, result)` verification triple. That is the whole obligation
   for a routine bugfix, refactor, feature or optimization — which is most of
   what `/log` handles. Escalate to the full type-specific section set below
   only when a Discussion trigger fires (see "Writing a logbook entry well").
2. Full form, when escalated — use the appropriate type template sections:
   - **bugfix**: Summary, Problem, Root Cause, Fix, Verification, Outcome
   - **refactor**: Summary, Motivation, Changes, Verification, Outcome
   - **feature**: Summary, Motivation, Design, Implementation, Verification, Outcome
   - **optimization**: Summary, Motivation, Approach, Benchmarks, Verification, Outcome
3. Fill in frontmatter: title, type, date, status, phase, files_changed,
   subsystem, tags. **No `commits:` field** — the commit's `Logbook-Entry:`
   trailer is the canonical link.
4. Fill in Summary from commit messages or user description
5. Fill in the remaining sections with available information
6. Update `logbook/INDEX.md`

### Step 4: Review

Present the draft entry to the user. If they want changes, iterate.

### Step 5: Finalize

If there are uncommitted changes to the logbook entry itself:
- Stage and commit the logbook entry with the convention:
  `docs: logbook — <entry title>`
  Include trailer: `Logbook-Entry: <entry-slug>`

## Subsystem Path Map

This mapping is used for auto-tagging. Entries are matched in order (first match wins
for ambiguous paths):

```
controller/                → controller
ros_ws/.../motion/         → motion
ros_ws/.../can/            → can
ros_ws/.../tracking/       → tracking
ros_ws/.../orchestrator*   → ros
ros_ws/.../state_machine*  → ros
ros_ws/gui/                → gui
sim/                       → sim
config/                    → config
tools/                     → tools
tests/                     → testing
docs/                      → docs
```

## Important Notes

- This command creates a logbook entry — it does NOT make code changes
- For code changes that need the full diagnose → fix pipeline, use `/investigate`
- If the entry references a plan, set `related_plan:` in the frontmatter
- The `files_changed:` field is critical for reverse lookups — be thorough
- Use `--from-commits` for retroactive logging of changes already committed

## Writing a logbook entry well

- **A Discussion section is non-negotiable under any of the three triggers**
  (verbatim from CLAUDE.md's Engineering Philosophy): (a) a hypothesis was
  withdrawn or reframed mid-investigation, (b) a non-obvious tradeoff was
  accepted, (c) the chosen approach beat another reasonable approach for
  reasons future-readers wouldn't infer from the code alone. Under a trigger
  it is the most valuable part of the entry — write *why this approach over
  others*, *what was ruled out*, *what tradeoffs were accepted*, and write it
  *before* the Fix section. If it feels tedious, that's signal it's exactly
  the one that'll save the most time later. Outside the triggers a short-form
  entry with no Discussion is correct; see `logbook/README.md` § "Entry
  Length — short form is the default".
- **Don't describe the diff — the diff is in git.** Describe the decision
  tree that produced the diff. The next person to touch this code needs
  to know why it looks the way it does, not what characters changed.
- **If the change is a point fix for a pattern that could recur, say so
  explicitly in the entry.** Flag the class of failures, not just this
  instance. That primes the next investigator to climb one level of
  abstraction before patching.
