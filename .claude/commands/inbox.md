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
   `~/bin/gh issue view <n> --json title,body,url`. Do NOT close the issue yet: it is
   closed in step 6, only after the push has landed. The issue number is never the Item's ID.
3. **File it locally with the one creation command**, from the repo root with the project venv
   (run `git pull --ff-only` first: the app writes `map/` concurrently, and `--commit --no-push` does not pull, so that preceding pull is the only one before the file commit):
   `python -m mapkit item new <kind> "<title>" --body "<line>" [--parent <pathway>]
   [--priority P] [--effort E] [--hardware H] [--owner O] [--radius R] [--diagnosis D] [--area A]
   [--filed-by NAME] [--link L]... --commit --no-push`. The six metric flags are optional (a
   bug adds `--diagnosis`; no other kind may); pass only the ones you know from the report, never
   a guess (enums: `DOCUMENTATION_GUIDE.md` § 2.10; `item rate JB-n` sets them later).
   It picks the slug (kebab-case, <= 60 chars, `-2` if taken by
   any kind; the filename is the Item's identity and is never renamed), allocates the
   next ID `JB-<n>`, writes the canonical form, regenerates `docs/project-map.md` and
   commits `map: file <kind> JB-<n> <title>` (no `Logbook-Entry` trailer: data, not
   code). Nothing is pushed yet. It prints `JB-<n> <stem> filed (inbox) <title>`.
4. **Check BEFORE the push** (the item is inside the Avoid-word scope; a title with an
   Avoid word that reaches `main` paints the gate RED):
   `pytest tests/sim/test_map_items.py tests/sim/test_glossary_avoid_words.py tests/sim/test_mapkit_docs.py -q`.
   On a failure, fix the offending word in `map/<stem>.md` (frontmatter title or body; the
   filename stays), run `python -m mapkit gen`, then
   `git add map/<stem>.md docs/project-map.md && git commit --amend -m "map: file <kind> JB-<n> <fixed title>"`
   and re-run the check. Never push on a red check.
5. **Push** with the same bounded loop `item new` uses:
   `python -m mapkit item push <stem>` (up to 3 rounds of `pull --rebase` -> duplicate-ID
   check -> push; if a rebase brought in the same ID it renumbers its own Item by amending
   its unpushed commit). Quote the `JB-<n> pushed` line in the reply. A failure says a local
   commit is waiting and how to push or undo it.
6. **`--from-issue` only, after `item push` reports pushed**: close the issue with a comment
   pointing at the item file (`~/bin/gh issue close <n> --comment "Filed as JB-<n> (map/<stem>.md)"`).
   If the Avoid-word check or the push failed, leave the issue open.

Shared recipe, commit rule and reads: `docs/agents/map-writes.md`.

State plainly in the reply: an unpushed item is invisible to the map; the push
also carries any other unpushed local commits, which is the normal flow; and the
item does not appear in the map app until it reaches `main` (merge or ff as the
branch rule says).

## Stress cases

- A bug found mid-Sitting on the Jetson: file it with a one-line description and
  keep flying; do not start diagnosing here. `/work-inbox` picks it up later.
- An idea from a phone goes through the app, not this command.
