---
id: JB-18
kind: ticket
title: Wayfinder tracker operations on the new system
type: grilling
parent: project-map
blocked_by:
- ticket-map-item-schema-and-the-map-directory-layout
- ticket-intake-the-slash-command-the-ui-form-inbox-triage-states-and
owners: []
rank: 13
status: complete
links:
- https://github.com/Project-DeepBlue-Juggling/Jugglebot/issues/34
created: '2026-10-23'
updated: '2026-10-23T14:37:58Z'
priority: medium
---

Part of #21

## Question

CORRECTION (2026-10-23): docs/agents/issue-tracker.md (GitHub issues, with a Wayfinding operations section) and docs/agents/triage-labels.md already exist on main. This ticket rewrites those two files for the new system rather than creating them.

How do future wayfinder efforts live on the map? Write the "Wayfinding operations" section the wayfinder skill expects, for this tracker: map = a Pathway item (or a dedicated type?) carrying the Destination / Notes / Decisions / Fog body; child tickets = items with a wayfinder:<type> tag; blocking = blocked_by; claim = an assignee or status field; frontier = a query; resolve = answer section + status + decision pointer. Decide where the setup-matt-pocock-skills tracker doc lives in Jugglebot so /setup-matt-pocock-skills finds it, and the triage-label mapping. Include migrating THIS map out of the gitignored .scratch/ once the system exists.

## Answer

## Answer (owner decision, 2026-10-23)

**Owner (overriding the recommendation to stay on GitHub issues): wayfinder maps and their tickets move onto the new system. "I want everything on the new system."** This confirms the charting decision "this system becomes the issue tracker that future wayfinder maps live on" and extends it to the current map.

### The model

- **A wayfinder map is a Pathway item.** Its `destination` is the map's Destination; its body holds the Notes, Decisions so far, Not yet specified and Out of scope sections. "Resolved tickets" is **computed** from its complete ticket children (title, gist from the first line of the ticket's answer, link), never hand-maintained.
- **A ticket is a new item kind, `ticket`** (adds to #25's four): `kind: ticket`, `type: research | prototype | grilling | task`, `parent: <pathway slug>`, `blocked_by:` (ticket or item slugs), `owners:` (non-empty = claimed), `rank:` (map order), `status: open | complete | dropped`, `links:`; body = the question, and the answer appended under `## Answer` at resolution. Tickets do not use the inbox status set; they are never untriaged.
- **Operations** (replace the "Wayfinding operations" section of `docs/agents/issue-tracker.md`; #33 writes the text, the `/wayfinder` skill reads it): create map = write `map/pathway-<slug>.md`; child ticket = write `map/ticket-<slug>.md` with `parent`; blocking = `blocked_by`; **frontier** = open tickets under the pathway with no open blocker and no owner, lowest `rank` first; **claim** = add yourself to `owners` and commit-and-push the file (the #30 commit rule, message `map: claim <slug>`); **resolve** = append `## Answer`, set `complete`, commit-and-push; sharpened tickets = new ticket files with `blocked_by`. `mapkit` exposes `frontier`, `claim`, `resolve` as subcommands so the skill does not reimplement them.
- **In the app**: tickets render as inbox-style cards (#38's design, type label instead of kind hue) in their pathway's Now (claimed) / Later (open) / Done columns; the Front page tile shows "n tickets open" beside the progress bar. The drawer edits them like inbox items.

### Migration

- **This map (#21) finishes on GitHub**: it is 11 of 17 done and moving it mid-flight is churn. On day one `python -m mapkit import-issues <map-number>` imports it as the `project-map` pathway's tickets (title, body, the answer comment as `## Answer`, `wayfinder:<type>` label → `type`, native dependencies → `blocked_by`, closed → `complete`), and the GitHub issues stay as read-only history. Every later map is born on the new system.
- GitHub issues remain the public door for outsiders (#30); nothing of the owner's planning lives there afterwards.
- **ADRs and logbook entries are not items**: they stay linked by reference through `links:` (closes the map's fog item on that question).

### Sharpened

- #25 amended: fifth kind `ticket` with the fields above; `rank` is a ticket-only field.
- #33 receives the issue-tracker.md rewrite and the `mapkit frontier | claim | resolve | import-issues` subcommands.
- #35 receives the import of #21 as a day-one step and the ticket card rendering.
