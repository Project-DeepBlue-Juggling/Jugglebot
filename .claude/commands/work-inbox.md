---
description: Work map items from the inbox/backlog - classify each, dispatch to the right skill or command, write the result back and commit. Invoke with /work-inbox [slug ...] [--pathway <slug>] [--kind bug|feature|idea] [--dry-run].
---

# Work Inbox Command

Take map items (`map/<kind>-<slug>.md`) and move them toward `complete` or
`promoted`.

## Arguments

`/work-inbox [slug ...] [--pathway <slug>] [--kind bug|feature|idea] [--dry-run]`

With no slugs: list `backlog` items first, then `inbox`, by priority then age
(filtered by `--pathway` / `--kind`), and ask which to take.

## Protocol

1. **Read before working**: the item file, every `links:` target, and the parent
   pathway item (its Destination and body; `python -m mapkit show <pathway>` also
   gives the active plan's step table). `git pull --ff-only` first.
2. **Classify** the item and pick the dispatch below. `--dry-run` prints the
   classification and plan of action only, and writes nothing.
3. **Dispatch**:

| The item asks for | Dispatch | Writes back |
|---|---|---|
| facts, options, a comparison | `research` skill (findings file beside the item, `links:` updated) | `## Findings` summary in the body |
| a design decision | `grilling` with the owner, in this session | `## Decision` in the body |
| a plan-sized implementation | `/implementation-report` -> new plan in `plans/active/` with `pathway:` = the item's parent | item `promoted`, `promoted_to: <plan filename>` |
| a new line of work with a Destination | a pathway file (after a grilling round for the Destination) | item `promoted`, `promoted_to: <pathway slug>` |
| a small, clearly diagnosed fix | the fix in this session, logged with `/log` or `/investigate` | item `complete`, `links:` gains the logbook entry |
| a bug needing hardware diagnosis | hands off to `/investigate` with the item's links as starting evidence | `in_progress`, `links:` gains the entry |

`promoted` / `promoted_to` apply to all three inbox kinds (bug, feature, idea).

3b. **Start**: before dispatching, `python -m mapkit item set-status <slug> in_progress
   --owner <session> --commit` (own `map:` commit, pushed). A "fix" row that names a
   plan step ticks that step too.
4. **Done** = status `complete` or `promoted`. `complete`: run
   `python -m mapkit item set-status <slug> complete --link logbook/<entry>.md`
   WITHOUT `--commit` and stage the printed paths into the fix's commit, which
   carries `Logbook-Entry:`. `promoted`: `python -m mapkit item set-status <slug>
   promoted` WITH `--commit` (its own `map: promoted <title>` commit, no trailer).
5. **Partial** = `in_progress` plus a dated `## Progress` note (done / next /
   evidence), committed and pushed before the session ends; the next run
   resumes from it.

Recipe and commit rule: `docs/agents/map-writes.md` (`complete` rides in the fix's
commit with its `Logbook-Entry:` trailer; `promoted` is its own `map:` commit, no trailer).
Claude moves items through every status transition without asking. Ask only
when the classification is ambiguous or the action is a hardware Sitting.
Bump `updated` on every write (use `mapkit.items.write_item`).
