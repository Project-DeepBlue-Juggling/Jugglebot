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

1. **Fields.** `kind` as given; `priority` default `medium`; `--link` values as given
   (an ID such as `JB-12` is stored as its stem); parent = `--pathway` (slug or `jb` ID);
   `filed_by` = `harrison` when the owner typed the command, else the session's
   name (e.g. `claude`). The one-line description is the body.
2. **`--from-issue <n>`**: import title, body and the issue URL (as a `--link`) via
   `~/bin/gh issue view <n> --json title,body,url`, then close the issue with a
   comment pointing at the item file. The issue number is never the Item's ID.
3. **File it with the one creation command**, from the repo root with the project venv:
   `python -m mapkit item new <kind> "<title>" --body "<line>" [--parent <pathway>]
   [--priority P] [--filed-by NAME] [--link L]... --commit`.
   It pulls `--ff-only`, picks the slug (kebab-case, <= 60 chars, `-2` if taken by
   any kind; the filename is the Item's identity and is never renamed), allocates the
   next ID `JB-<n>`, writes the canonical form, regenerates `docs/project-map.md`,
   commits `map: file <kind> JB-<n> <title>` (no `Logbook-Entry` trailer: data, not
   code) and pushes, renumbering its own Item if a rebase brought in the same ID.
   It prints `JB-<n> <stem> filed (inbox) <title>`: quote that line in the reply.
4. **Check** (the item is inside the Avoid-word scope):
   `pytest tests/sim/test_map_items.py tests/sim/test_glossary_avoid_words.py tests/sim/test_mapkit_docs.py -q`.

Shared recipe, commit rule and reads: `docs/agents/map-writes.md`.

State plainly in the reply: an unpushed item is invisible to the map; the push
also carries any other unpushed local commits, which is the normal flow; and the
item does not appear in the map app until it reaches `main` (merge or ff as the
branch rule says).

## Stress cases

- A bug found mid-Sitting on the Jetson: file it with a one-line description and
  keep flying; do not start diagnosing here. `/work-inbox` picks it up later.
- An idea from a phone goes through the app, not this command.
