---
description: Browse, search, or create engineering logbook entries. Invoke with /logbook [--list | --search <query> | --status <status> | --new <title> | <entry-name>].
---

# Engineering Logbook Browser

Browse, search, and manage engineering logbook entries in `logbook/`.

## Arguments

- **No args** or **`--list`**: show the full index
- **`--search <query>`**: search entries by keyword (greps entry files)
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

### Search mode (`--search <query>`)

1. Grep all `.md` files in `logbook/` (excluding INDEX.md and TEMPLATE.md) for the query string
2. For each match, extract the front matter (title, date, status, phase)
3. Present results as a table with the matching context line

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
