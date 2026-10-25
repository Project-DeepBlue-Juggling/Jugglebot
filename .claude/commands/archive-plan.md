---
description: Archive a completed plan — critically review it first, then move from plans/active/ or plans/parked/ to plans/archived/. Invoke with /archive-plan <plan-name>.
---

# Archive Plan Command

Move a plan into `plans/archived/` after verifying it is truly complete.

## The plans board is three-way

`plans/active/` = schedulable now. `plans/parked/` = deliberately not now,
each row in `plans/parked/INDEX.md` naming **what would unpark it**.
`plans/archived/` = done or superseded. See `DOCUMENTATION_GUIDE.md` § 2.6.

**A plan's filename NEVER changes for its life.** Archival is a `git mv` of the
file *unchanged*, plus an `archived:` frontmatter field. Do not add a date
prefix — that convention was retired 2026-08-16 because the rename broke every
inbound cross-reference (`related_plan:` is filename-only and resolves by
searching the three plan directories, so a renamed file resolves nowhere).
`tests/sim/test_plans_index.py` fails if a date-prefixed filename reappears.

**Parking is a different move from archiving.** Park when the work is real but
blocked or deprioritised; archive when there is nothing left to schedule. There
is no `/park-plan` command — do it by hand (see § Parking a plan below), or
archive `superseded` if nothing would ever unpark it.

## Arguments

- **`<plan-name>`**: the plan filename (with or without .md, with or without a
  `plans/active/` or `plans/parked/` prefix)

## Protocol

### Step 1: Locate the plan

1. Find the plan file in `plans/active/`, then `plans/parked/` — a parked plan
   is an ordinary archival candidate, not a special case
2. If not found, check `plans/archived/` — if already archived, inform the user
3. If not found anywhere, list available active and parked plans

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

1. Move the file **without renaming it**:
   `git mv plans/active/<name>.md plans/archived/<name>.md`
   (or from `plans/parked/`). **No date prefix.** The filename is the plan's
   stable identity — renaming it is what used to break every inbound reference.
2. Update the frontmatter:
   - Set `status: completed` (or `superseded` if replaced by a new plan)
   - Add `archived: YYYY-MM-DD` — today's date. This is now the only in-file
     record of when the plan closed, since the filename no longer carries it.
   - Add `completed: YYYY-MM-DD` too when there is a distinct completion date
     worth recording (work finished earlier than the archival housekeeping)
3. Add the Archival-note section to the plan body: what shipped, why it closed,
   and where any residue was re-homed.
4. **Board bookkeeping, same commit.** The three `plans/*/INDEX.md` and
   `docs/project-map.md` are generated from plan frontmatter. Set the
   frontmatter (step 2: `status`, `archived`, optional `completed`), then run
   `python -m mapkit gen` and stage the regenerated `plans/*/INDEX.md` and
   `docs/project-map.md`. `tests/sim/test_plans_index.py` (the currency test)
   fails if you forget. Do not hand-edit an INDEX row.
   - Prose that cross-references another plan must still be **path-qualified**
     (`plans/active/<other>.md`) wherever a bare `*.md` name would claim the
     file lives in the wrong directory.
5. Sweep inbound references. `related_plan:` entries need **no change** — that
   is the point of the stable filename. What does need re-pointing is any
   *path*-qualified prose reference (`plans/active/<name>.md` →
   `plans/archived/<name>.md`) in other plans, logbook entries, CLAUDE.md,
   command files, docs, tests and code comments.
   - **`rg` is NOT installed on this box.** A ripgrep-based survey returns a
     silent zero. Use `grep -rn` (or `git grep -n`).
   - **A path can be split across two source lines** by string concatenation,
     so neither the full path nor the bare filename matches. Grep for
     distinctive fragments too, and for lines *ending* in a partial path.
   - **Always check whether a hit lives in a generator.** Fixing a generated
     artifact without fixing its generator restores the stale path at the next
     codegen. `config/generate_udp_protocol.py` emits five artifacts and has
     hidden exactly this, twice.
   - If a generated artifact changes, regenerate from the generator and verify
     the firmware image is unaffected:
     `cd ros_ws/src/jugglebot/Teensy_code_canbridge && pio run -e teensy41`,
     then compare `text/data/bss` and the `firmware.hex` md5 against the
     pre-change build. A comment-only change must be byte-identical — say so in
     the report so nobody infers a reflash is needed.
   - Leave *historical narrative* alone: a logbook sentence describing what the
     naming convention was at the time is a record, not a live path.

### Step 4b: Gated offer for residue

List the plan's residue rows (its residue section, deferred items, open
follow-ups) and offer, at ONE gate (default yes, nothing silent), to file each
as a Bug or Feature item with `links: [plans/archived/<plan>.md]`,
`status: inbox`, `parent:` = the plan's `pathway:`. Filing follows the
`/inbox` steps (write, commit `map: file <kind> JB-<n> <title>`, push).

### Step 4c: Map items

Read the plan's `pathway:` (`python -m mapkit show <pathway>`; silent). List open children
that name this plan, and any ticket whose `blocked_by` is this plan (archival unblocks it:
say so). Find items promoted to this plan with `grep -l "promoted_to: .*<plan filename>" map/*.md`
(`rg` is not installed; `mapkit show` lists only open items). Do NOT change a `promoted`
item's status; append a note with `set-status <slug> promoted --note "plan archived <date>"`
(re-setting the same status is allowed without `--force`), without `--commit`, and stage the
printed paths into the archive commit (a plan-board/`map` housekeeping change, no
`Logbook-Entry` trailer, consistent with `DOCUMENTATION_GUIDE.md` § 2.10). If this is the Pathway's last plan, ASK whether the Pathway is `complete` (never
auto-set). Recipe: `docs/agents/map-writes.md`.

### Step 5: Report

Summarize what was done:
- Plan archived to `plans/archived/<name>.md` (filename unchanged), `archived: <date>`
- Boards regenerated (`python -m mapkit gen`): which INDEX lost the row, which gained it
- N path-qualified cross-references re-pointed, and where the sweep was
  deliberately not applied
- Firmware byte-identical result, if any generated artifact was touched
- Any remaining action items

## Parking a plan

No slash command exists for this yet — a `/park-plan` counterpart is worth
adding. Until it does, the manual flow is:

1. `git mv plans/active/<name>.md plans/parked/<name>.md` — filename unchanged.
2. Set the frontmatter (a reader who opens the file directly must learn it
   is parked without finding the board; the board is generated from these):
   `status: parked   # YYYY-MM-DD — <why, in one line>` AND the three fields
   `parked: YYYY-MM-DD`, `parked_why:`, `unpark:`. Derive `unpark:` (the
   concrete gate, prerequisite or decision) from the plan's own text; do not
   invent a gate — if the plan states none (parked on priority), say exactly
   that. For a pre-frontmatter plan a `**Status:**` banner may stand in.
3. Run `python -m mapkit gen` and stage the regenerated `plans/*/INDEX.md` and
   `docs/project-map.md`.
4. Re-point path-qualified inbound references `plans/active/<name>.md` →
   `plans/parked/<name>.md`, with the same grep cautions as Step 4 above.
5. Unparking is the same move in reverse: remove the three fields, set
   `status: active`, `git mv` back, then `python -m mapkit gen`.

## Important Notes

- The plan-reviewer agent is the critical gate — it prevents premature archiving
- The reviewer checks the actual codebase, not just the plan's claims
- Candidates are plans in `plans/active/` (`status: active`) and in
  `plans/parked/` (`status: parked`) — a parked plan that will never be
  unparked belongs in `plans/archived/` as `superseded`, not parked forever
- If a plan is superseded (not completed), use `status: superseded` instead of `completed`
- **Never rename the file.** No date prefix, no slug change, ever — not at
  archival, not at parking. The bare filename is the plan's identity and the
  only thing a `related_plan:` reference has to go on
