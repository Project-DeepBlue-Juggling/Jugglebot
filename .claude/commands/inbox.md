---
description: File a Bug, Feature or Idea item into the project map's inbox (map/<kind>-<slug>.md), commit and push it. Invoke with /inbox <bug|feature|idea> "<title>" [one-line description] [--pathway <slug>] [--priority low|medium|high] [--link <path-or-url>]... [--from-issue <n>].
---

# Inbox Command

File one map item. The item is a file in `map/`; it is invisible to the map
until it is committed and pushed.

## Arguments

`/inbox <bug|feature|idea> "<title>" [one-line description] [--pathway <slug>]
[--priority low|medium|high] [--link <repo-path-or-url>]... [--from-issue <n>]`

Minimum from the filer: kind, title, one line. If the line is missing, ask for it.

## Protocol

1. **Fill the frontmatter.** `kind` as given; `status: inbox`; `created` = today;
   `updated` = now (UTC, ISO seconds); `owners: []`; `priority` default `medium`;
   `links` a plain list (`--link` values); `parent` = `--pathway` slug when given;
   `filed_by` = `harrison` when the owner typed the command, else the session's
   name (e.g. `claude`). The one-line description is the body.
2. **`--from-issue <n>`**: import title, body and `links: [<issue URL>]` via
   `~/bin/gh issue view <n> --json title,body,url`, then close the issue with a
   comment pointing at the item file.
3. **Pull first** (`git pull --ff-only`: the app writes `map/` concurrently), then
   **slug** = kebab-case of the title (ASCII, <= 60 chars). The file is
   `map/<kind>-<slug>.md`; if it exists, append `-2`. The filename is the item's
   identity and is never renamed.
4. **Write** with `mapkit.items.write_item` (the canonical form), e.g. a
   `python -c` one-liner or heredoc run from the repo root with the project venv.
5. **Check** (the item is inside the Avoid-word scope):
   `pytest tests/sim/test_map_items.py tests/sim/test_glossary_avoid_words.py -q`.
   Then `python -m mapkit gen`: items do not change the plan INDEX boards, but they do feed `docs/project-map.md` (its Inbox table and the attached-items lists), and `tests/sim/test_mapkit_docs.py` fails on a stale page.
6. **Commit and push**: `git add map/<file> docs/project-map.md && git commit -m "map: file <kind> <title>"`
   then `git push`. No `Logbook-Entry` trailer: this is data, not code.

Shared recipe, commit rule and reads: `docs/agents/map-writes.md`.

State plainly in the reply: an unpushed item is invisible to the map; the push
also carries any other unpushed local commits, which is the normal flow; and the
item does not appear in the map app until it reaches `main` (merge or ff as the
branch rule says).

## Stress cases

- A bug found mid-Sitting on the Jetson: file it with a one-line description and
  keep flying; do not start diagnosing here. `/work-inbox` picks it up later.
- An idea from a phone goes through the app, not this command.
