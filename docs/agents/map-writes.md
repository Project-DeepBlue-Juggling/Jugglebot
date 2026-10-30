# Map writes: the one recipe

Every command that moves a `map/` item follows this page instead of copying steps. Item model: `DOCUMENTATION_GUIDE.md` § 2.10. Tool: `mapkit/itemcli.py`. Run with the project venv from the repo root.

## Naming an Item

Every Item has a short ID `JB-<n>` (frontmatter `id:`, one global sequence; plan `plans/archived/map-item-ids.md`). Every command below takes the ID (`JB-12`, `jb12`, `jb-12`, any case), the stem (`bug-x`) or the bare slug (`x`); resolution is stem, then slug, then ID. **Bare digits (`12`) are accepted by read commands only** (`items`, `show`): on a write they are an error, because GitHub issue numbers live alongside and a one-digit typo in one sequence lands on another real Item. Every write prints `JB-n <stem> ... <title>`; check it names the Item you meant. Stored references (`parent`, `blocked_by`, `links`) stay stems: an ID typed into `item new --parent` / `--link` is stored as the stem.

## Reads (silent, short)

- `python -m mapkit show <pathway slug or ID>`: Destination, active plans with step table, open children by kind. Quote only the Destination and the open-bug lines into context.
- `python -m mapkit items [--pathway X] [--kind K] [--open | --status S] [--stale] [--json]`: one line per item, ID first; `--stale` keeps only items left too long (clock = newest dated Progress line, else `created`; inbox 30 d, backlog 60 d, in_progress 14 d) and appends the age and clock date.
- The pathway slug is the `pathway:` of the plan (or the item's `parent:`). No pathway found: skip the read, do not ask.

## Writes

1. **Set status** (with `--commit` the CLI pulls `--ff-only` first; before a close without `--commit`, run `git fetch && git status -sb` and `git pull --ff-only` yourself. The CLI also validates the status for the kind, refuses to re-open a `complete` item or start one another owner holds, appends a dated `## Progress` line, de-duplicates `--link`, regenerates `docs/project-map.md`):
   `python -m mapkit item set-status <id|stem|slug> <status> [--note TEXT] [--link PATH]... [--owner NAME] [--force] [--commit [--no-push]]`
2. **Which commit carries what** (hybrid rule, owner decision 2026-10-25):
   - **Start and progress notes** (`in_progress`, evidence lines): `--commit`. Its own `map: <status> JB-n <title>` commit, no `Logbook-Entry` trailer, pushed at once so the app shows it.
   - **Closing** (`complete`): omit `--commit`. The CLI prints the two paths (`map/<file>`, `docs/project-map.md`); stage them into the SAME commit as the fix and its logbook entry, which carries the `Logbook-Entry:` trailer. Reason: a `complete` item whose fix was reverted or never pushed is the worst map lie; bundled, a revert reopens the item textually.
   - **`promoted`, `dropped`, filing**: own `map:` commit, no trailer.
   - If the push races the app: `git pull --rebase`, re-run `python -m mapkit gen`, stage again.
3. **Announce** a status change in one line (`JB-12 bug-x -> in_progress`). No approval step. The one exception is a **Pathway** going `complete`: always ask the owner.
4. **Scoped tests** after a write: `pytest -q tests/sim/test_map_items.py tests/sim/test_glossary_avoid_words.py tests/sim/test_mapkit_docs.py` (item bodies are inside the Avoid-word scope).

### Question tickets

When an Item needs a decision only the owner can make, file a question ticket that blocks it, instead of guessing:

`python -m mapkit item new ticket "<question>?" --type question --parent JB-n --body-file F --commit`

- **Who files:** Night Shift (`.claude/commands/night-shift.md`), or any Session that hits an owner-only decision. `--parent` is the bug, feature or idea it blocks (ID, stem or slug; stored as the stem); a pathway or ticket parent is refused.
- **How it is answered:** the owner types in the app's answer box, which appends `## Answer` to the body and completes the ticket in one save. By hand: add `## Answer` with the reply, then `python -m mapkit item set-status <id> complete`. Completing without an answer succeeds but warns on stderr.
- **Gate:** while a question is `open`, the Night Shift merge gate holds its parent's branch; completing it releases the hold.

## Writing an Item body

The owner reads Items in the app's drawer; a body that is one long paragraph is illegible there (owner, 2026-10-30, on JB-114). Write the body to be scanned:

- A short opening paragraph (the context or the ask), then `## ` sections named for what they hold (`## Wanted`, `## Findings`, `## Risks`, `## Decision`, `## Open questions`).
- Bullets for separate points, a numbered list where order matters, a table where the text pairs several values (readings, parameters, options). Bold only the one claim a section turns on.
- Backticks for code identifiers, config keys, paths and commands, never for a `JB-n` ID (see below).
- A structured body goes in through `item new --body-file F`, not `--body`, which is for one line.
- A Progress note (`--note`) is flattened to one line: keep it to its headline. Detail goes in a body section placed just before `## Progress`, headed `## <topic> (YYYY-MM-DD)`, or in the logbook entry the note links; end the note with `(detail below)`.

The caps are enforced for every committed Item by `tests/sim/test_map_items.py::test_item_bodies_are_readable` (rule and numbers: `mapkit.items.readability_violations`): a paragraph or list item at most 500 characters, a Progress line at most 400, and a `##` heading on any body over 1,200 characters (Progress excluded). Tables and fenced code are exempt. Fix a failure by restructuring, never by cutting facts.

## The `## Sitting` section

A hardware Item (`hardware: bench-leg | robot`) says what it needs from the next Sitting in a body section headed exactly `## Sitting`, placed before `## Progress`. A Session that files or progresses a hardware Item writes it. It is a bullet list with fixed keys (`- key: value`; `do:` may be one line or a nested `  - ` list):

| key | value |
| --- | --- |
| `setup:` | one or more of `level`, `tilt`, `armed`, `bb-fed`, `bridge-cycled`, comma separated (required) |
| `why:` | one sentence (required) |
| `do:` | the hardware steps only, imperative (required) |
| `pass:` | the criterion (required) |
| `record:` | what to write down (required) |
| `minutes:` | positive integer (required) |
| `branch:` | a held Night Shift branch name (optional) |

`python -m mapkit runbook` (and `gen`) turns every valid section into `docs/runbook.md`, the Next Sitting Runbook, ordered by priority weight (high 3, medium 2, low 1; double with a `branch:`) per minute and grouped by setup set. The generator never invents a `do:` step: an Item with a missing or malformed section is listed under "Needs a Sitting section" with the parse errors. Results typed in the app land on the Item as `## Sitting result (<date>)` (verdict pass or fail, notes). Parser and generator: `mapkit/sitting.py`; pinned by `tests/sim/test_sitting_sections.py`.

## Rules

- Subagents never write map files; the calling command does. They return "item to close: <slug>" in their final message.
- Never hand-edit YAML; `set-status` and `mapkit.items.write_item` keep the canonical form.
- Re-setting the same status without `--note` is a no-op ("unchanged", nothing written or committed). With `--note` it is not idempotent: re-running appends another Progress line, so check the status first (`mapkit items` or the file) before writing. `--owner` takes the session's name on `in_progress`.
- **Filing** goes through `python -m mapkit item new <kind> "<title>" [--parent ID|slug] [--priority P] [--effort E] [--hardware H] [--owner O] [--radius R] [--diagnosis D] [--area A] [--filed-by NAME] [--body TEXT | --body-file F] [--link L]... [--type T] [--destination D] [--commit [--no-push]]` (used by `/inbox`): it allocates the next ID, writes the canonical form and regenerates the docs page. **Every filing path screens the title first** (`mapkit/similar.py`; the app's create answers 409 with the same rule): a title equal to an open Item's (case, punctuation and spacing ignored) is refused outright as `duplicate of JB-n`, and a near title (or the same title as a closed Item) is refused as `similar Items exist` with the list until `--allow-similar` (the app: `allow_similar: true`, sent by its "File anyway" button) says it is a different thing; `tests/sim/test_map_items.py::test_no_two_open_items_share_a_title` catches a duplicate that skipped the screen. `item new` screens once, after its first pull and before the write: a same-titled Item a peer pushes in the seconds before the push-loop rebase is caught only by that gate test. With `--commit` it commits `map: file <kind> JB-n <title>` (no trailer) and runs up to 3 rounds of `pull --rebase` -> duplicate check -> push; if a rebase brought in an Item with the same ID it renumbers its own Item, amending its not-yet-pushed `map: file` commit so no subject on the branch cites a wrong ID (a separate `map: renumber JB-a -> JB-b <title>` commit only when that commit is no longer HEAD; it rebases onto the branch's upstream and repairs against that same ref, and an Item it has not pushed never keeps a contested ID). If the rebase conflicts (two writers chose the same slug, say), it aborts the rebase, pushes nothing and exits 2 naming the unmerged paths, its local commit and any pending autostash: give the Item a new slug by hand (`git mv`, amend), then push and run `python -m mapkit ids --check`. `--commit --no-push` stops after the local commit; `python -m mapkit item push <stem>` then runs the same bounded rebase / renumber / push loop (this is how `/inbox` runs its Avoid-word check before anything reaches `main`). Any failure after the commit exists says that a local commit is waiting and how to push (`item push`) or undo (`git reset --soft HEAD~1`, then delete the Item file and revert the docs page if unwanted; never `--hard`, which destroys autostashed edits) it. Allocation, `ids --repair` and `--assign-missing` skip a file that fails to parse (named on stderr) but still count its `id:` line, so a malformed Item's number is never reused; `ids --check` stays strict. A hand-written Item, `item new` without `--commit`, or a plain `git push` is not checked: the gate (`tests/sim/test_map_items.py::test_item_ids_unique`) catches a duplicate and `python -m mapkit ids --repair` fixes it; `python -m mapkit ids --assign-missing` numbers an ID-less Item.
- **Subjects** written by Claude cite the ID after the verb (`map: file bug JB-63 <title>`, `map: in_progress JB-12 <title>`, `map: claim JB-9 <slug>`). The app's own commits stay `map: <title>`: it commits before it knows whether its ID survives the rebase.
- **Metrics** (`effort`, `hardware`, `owner`, `radius`, `diagnosis` (bugs only), `area`; enums in `DOCUMENTATION_GUIDE.md` § 2.10) are optional on `item new` (the flags above) and validated when present. Set or change them on an existing Item with `python -m mapkit item rate JB-n --effort s --area gui [--hardware ... --owner ... --radius ... --diagnosis ...] [--commit [--no-push]]` (at least one flag; rewrites the file in canonical key order after `priority`, bumps `updated`, regenerates the docs page, commits `map: rate JB-n <title>` with `--commit`). Rate an Item you file when you know the answer; leave a field off rather than guess.
- **Prose names other Items by ID.** In text the drawer renders (an Item's body, a `--note` Progress line, an `item new --body`, or a plan or phase description), refer to another Item as plain `JB-n` (`blocked on JB-14`, `split from JB-58`), not by stem or title alone; a title may follow (`JB-14 (Cup Banking clamp)`). The app's drawer turns every known `JB-n` in that text into a link that opens the Item, so a plain ID is a working cross-link there. Write it bare: an ID in backticks or inside a Markdown link stays text, and a range links only its first ID (write `JB-68, JB-69, JB-70`, not `JB-68..70`). Stored references (`parent`, `blocked_by`, `links`) still stay stems (above).
