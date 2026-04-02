---
description: Browse, search, or create engineering logbook entries. Invoke with /logbook [--list | --summary | --timeline | --search <query> | --file <path> | --subsystem <name> | --status <status> | --new <title> | <entry-name>].
---

# Engineering Logbook Browser

Browse, search, and manage engineering logbook entries in `logbook/`.

## Arguments

- **No args** or **`--list`**: show the full index
- **`--summary [N]`**: digest of entries from the last N days (default 7)
- **`--timeline`**: chronological view with plan milestones
- **`--search <query>`**: search entries by keyword (greps entry files)
- **`--file <path>`**: find entries that modified a specific file (searches `files_changed:` frontmatter)
- **`--subsystem <name>`**: filter by subsystem tag (mpc, can, tracking, motion, sim, gui, etc.)
- **`--status <status>`**: filter by status (open, in-progress, resolved)
- **`--phase <phase>`**: filter by bringup phase
- **`--new <title>`**: create a new manual entry (not tied to /investigate)
- **`<entry-name>`**: read and display a specific entry (filename with or without .md)

## Protocol

### List mode (default, or `--list`)

1. Read `logbook/INDEX.md`
2. Present the table
3. Report counts: N entries (X open, Y in-progress, Z resolved)
4. Highlight any `open` or `in-progress` entries as needing attention

### Summary mode (`--summary [N]`)

Generate a digest of recent logbook activity. N is the number of days to look back (default: 7).

1. Read all entry files in `logbook/` (excluding INDEX.md, TEMPLATE.md, README.md)
2. Parse frontmatter from each, filter to entries with `date:` within the last N days
3. Present a structured digest:

```
## Logbook Summary (last 7 days)

**Activity:** 3 entries (1 resolved, 1 in-progress, 1 open)

### Resolved
- 2026-04-01 [investigation] Cold-hold fallback commanded stroke minimum
  Files: controller/mpc.py, sim/analysis/diagnose.py (+3 more)
  Commit: 3a3381f

### In Progress
- 2026-04-02 [refactor] Extract CAN process from ROS2 node
  Files: ros_ws/.../can_node.py (+5 more)

### Open (needs attention)
- 2026-04-02 [investigation] MPC solve budget violations on off-centre targets
  Phase: 4.2 | Subsystem: mpc
```

4. If no entries in the time window, report that and suggest a wider range

### Timeline mode (`--timeline`)

Show a chronological view interleaving logbook entries and plan milestones.

1. Read all entry files in `logbook/`, extract date + title + status + type
2. Read all plans in `plans/active/` and `plans/archived/`, extract created + completed dates
3. Merge into a single chronological list, newest first:

```
## Project Timeline

2026-04-02  [logbook/refactor]     Extract MPC orchestration (resolved)
2026-04-01  [logbook/investigation] Cold-hold fallback stroke minimum (resolved)
2026-03-30  [logbook/investigation] Velocity feedforward oscillation (resolved)
2026-03-30  [plan/completed]       MPC Oscillation Analysis → archived
2026-03-28  [plan/started]         MPC Hardware Bringup → active
2026-03-18  [plan/completed]       Ball Tracking & Catch → archived
2026-03-18  [plan/completed]       Codebase Overhaul → archived
...
```

4. Limit to last 30 entries by default. Accept an optional count argument: `--timeline 50`

### Search mode (`--search <query>`)

1. Grep all `.md` files in `logbook/` (excluding INDEX.md and TEMPLATE.md) for the query string
2. For each match, extract the front matter (title, date, status, phase)
3. Present results as a table with the matching context line

### File mode (`--file <path>`)

1. Grep all `.md` files in `logbook/` (excluding INDEX.md and TEMPLATE.md) for the
   file path in the `files_changed:` frontmatter section
2. For each match, extract the front matter (title, date, status, type, subsystem)
3. Present results as a table
4. This is the key reverse-lookup: "what logbook entries touched this file?"

Example: `/logbook --file motor_guard.py` finds every investigation, bugfix, or
refactor that modified motor_guard.py.

### Subsystem mode (`--subsystem <name>`)

1. Grep all `.md` files in `logbook/` for the subsystem name in the `subsystem:` frontmatter
2. For each match, extract front matter and present as a table
3. Valid subsystems: mpc, controller, motion, can, tracking, ros, gui, sim, config, tools

### Filter mode (`--status <status>` or `--phase <phase>`)

1. Read `logbook/INDEX.md`
2. Filter rows matching the specified status or phase
3. Present the filtered table

### Read mode (`<entry-name>`)

1. Read the specified entry file from `logbook/`
2. Present its full content
3. If the entry has status `open` or `in-progress`, suggest:
   - Resume investigation with `/investigate --resume <entry>`
   - Or update the entry manually

### New entry mode (`--new <title>`)

1. Spawn the `logbook-updater` agent to create a new entry:
   - Generate a slug from the title
   - Create `logbook/YYYY-MM-DD-<slug>.md` from the template
   - Fill in title and date; leave other fields for the user to complete
   - Set status to `open`
   - Update INDEX.md
2. Present the new entry path for the user to review

## Important Notes

- This is primarily a **read** command (list, search, filter, read modes)
- The `--new` mode creates a logbook file but does not modify any code
- For the full investigation workflow, use `/investigate` instead
- Entries with status `template` (i.e., TEMPLATE.md) are always excluded from listings
