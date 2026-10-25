# Map writes: the one recipe

Every command that moves a `map/` item follows this page instead of copying steps. Item model: `DOCUMENTATION_GUIDE.md` § 2.10. Tool: `mapkit/itemcli.py`. Run with the project venv from the repo root.

## Reads (silent, short)

- `python -m mapkit show <pathway>`: Destination, active plans with step table, open children by kind. Quote only the Destination and the open-bug lines into context.
- `python -m mapkit items [--pathway X] [--kind K] [--open | --status S] [--json]`: one line per item.
- The pathway slug is the `pathway:` of the plan (or the item's `parent:`). No pathway found: skip the read, do not ask.

## Writes

1. **Set status** (with `--commit` the CLI pulls `--ff-only` first; before a close without `--commit`, run `git fetch && git status -sb` and `git pull --ff-only` yourself. The CLI also validates the status for the kind, refuses to re-open a `complete` item or start one another owner holds, appends a dated `## Progress` line, de-duplicates `--link`, regenerates `docs/project-map.md`):
   `python -m mapkit item set-status <slug> <status> [--note TEXT] [--link PATH]... [--owner NAME] [--force] [--commit [--no-push]]`
2. **Which commit carries what** (hybrid rule, owner decision 2026-10-25):
   - **Start and progress notes** (`in_progress`, evidence lines): `--commit`. Its own `map: <status> <title>` commit, no `Logbook-Entry` trailer, pushed at once so the app shows it.
   - **Closing** (`complete`): omit `--commit`. The CLI prints the two paths (`map/<file>`, `docs/project-map.md`); stage them into the SAME commit as the fix and its logbook entry, which carries the `Logbook-Entry:` trailer. Reason: a `complete` item whose fix was reverted or never pushed is the worst map lie; bundled, a revert reopens the item textually.
   - **`promoted`, `dropped`, filing**: own `map:` commit, no trailer.
   - If the push races the app: `git pull --rebase`, re-run `python -m mapkit gen`, stage again.
3. **Announce** a status change in one line (`bug-x -> in_progress`). No approval step. The one exception is a **Pathway** going `complete`: always ask the owner.
4. **Scoped tests** after a write: `pytest -q tests/sim/test_map_items.py tests/sim/test_glossary_avoid_words.py tests/sim/test_mapkit_docs.py` (item bodies are inside the Avoid-word scope).

## Rules

- Subagents never write map files; the calling command does. They return "item to close: <slug>" in their final message.
- Never hand-edit YAML; `set-status` and `mapkit.items.write_item` keep the canonical form.
- Not idempotent: re-running appends another Progress line, so check the status first (`mapkit items` or the file) before writing. `--owner` takes the session's name on `in_progress`.
- Filing new items stays in `/inbox` (`map: file <kind> <title>`, no trailer).
