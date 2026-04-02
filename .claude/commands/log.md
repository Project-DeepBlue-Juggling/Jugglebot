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
3. Auto-populate: title (from commit message), files_changed, commits, date

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
   - `controller/mpc.py` or `controller/params.py` → `mpc`
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
1. Use the appropriate type template sections:
   - **bugfix**: Summary, Problem, Root Cause, Fix, Verification, Outcome
   - **refactor**: Summary, Motivation, Changes, Verification, Outcome
   - **feature**: Summary, Motivation, Design, Implementation, Verification, Outcome
   - **optimization**: Summary, Motivation, Approach, Benchmarks, Verification, Outcome
2. Fill in frontmatter: title, type, date, status, files_changed, commits, subsystem, tags
3. Fill in Summary from commit messages or user description
4. Fill in the type-specific sections with available information
5. Update `logbook/INDEX.md`

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
controller/mpc.py          → mpc, controller
controller/params.py       → mpc, controller
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
