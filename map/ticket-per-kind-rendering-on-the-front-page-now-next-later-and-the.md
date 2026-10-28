---
id: JB-9
kind: ticket
title: 'Per-kind rendering on the Front page, Now/Next/Later and the drawer: what a pathway, plan, phase, idea, feature and bug each show'
type: grilling
parent: project-map
blocked_by:
- ticket-the-fork-repo-name-runtime-shape-what-is-kept-stubbed-or
owners: []
rank: 17
status: complete
links:
- https://github.com/Project-DeepBlue-Juggling/Jugglebot/issues/38
created: '2026-10-23'
updated: '2026-10-23T14:37:58Z'
priority: medium
effort: s
hardware: none
owner: decision
radius: local
area: map-app
---

Part of #21

## Question

What does each item kind show on the canvas? Stock TaskTree draws every node as the same small card (title, avatar, status stripe). Seeing the #24 prototype, the owner asked for per-kind rendering: a **plan** as a rich card like a pop-out editor (title, description, phase-completion summary such as "P1 ✔ P2 retired P3-P5 open", branch), a **pathway** carrying its Destination, while **bug / idea / feature** items stay brief (title plus a one-line description).

Grill the design: which fields per kind, does a rich card expand in place or open the detail panel, how a plan's phase summary is derived (from the `phases:` frontmatter list of #23) and whether the List view's rows differ by kind too. The answer is a per-kind field table that #35's implementation plan can build from.

Context: #24 answer (keep / stub / drop list), #26 (the five pathways and their plans), #22 (the Task shape the canvas node receives).


## Re-scoped by #31 (owner, 2026-10-23)

The canvas is gone: the fork ships two pages only, **Front page** (one tile per Pathway: Destination, progress bar from phase statuses, computed Next, owners, attached inbox items; Keep-style inbox wall below) and **Now / Next / Later** (Pathway swimlanes against a commitment horizon, inbox rail split into Ideas / Bugs / Features). Editing is the shared detail drawer. The question is now: what does each kind (pathway, plan, phase, idea, feature, bug) show in a Front page tile, in a Now/Next/Later card, in the inbox wall and rail, and in the drawer; and what does hover-highlight (an item lit up in every place it appears) look like per kind. Starting material: the formats prototype on branch `prototype/24-thin-server`, `.scratch/project-map/prototype/formats/index.html` (`?variant=c` and `?variant=d`), its `HANDOFF.md`, and the #31 answer. Owner rules carried in: minimal scrolling, dark mode default.

## Answer

## Answer (grilling, owner, 2026-10-23, one round)

Per-kind field table for the two pages fixed by #31 (Front page, Now / Next / Later), the shared drawer and hover-highlight. Starting material was the formats prototype (branch `prototype/24-thin-server`, `.scratch/project-map/prototype/formats/index.html`, `?variant=c|d`); every rule below is the prototype's unless marked **new**. #35 builds from this table.

### Page rules

- **Front page:** equal pathway tiles in a grid, **no lead tile, fixed tile height** (**new**), so five or six pathways fit one screen at 1500×900. The what-remains list inside a tile is capped at four rows plus "+n more" (**new**); the rest is in the drawer. The inbox wall below is a **fixed-height band that scrolls internally; the page itself does not scroll** (**new**). The wall shows every inbox-kind item, attached or not, with a pathway pin.
- **Now / Next / Later:** pathway swimlanes × Now (in progress, QA, blocked) / Next (computed) / Later (queued, parked, waiting on a blocker, attached inbox items) / Done; inbox rail on the right split into Ideas, Bugs, Features, **unattached items only**.
- **Computed, never stored:** pathway order (priority, status, title); progress units (a plan's phases, else the plan; dropped excluded); Next (first unfinished phase of the first plan with no open blocker; if every open plan waits on a blocker, that blocker).

### Per-kind table

| Kind | Front page | Now / Next / Later | Drawer |
|---|---|---|---|
| **Pathway** | Tile: status pill, priority, title, Destination in full, segmented progress bar (one segment per unit), "x of y stops done", Next box, capped what-remains list with dep tags, owners, attached inbox items as tokens, **branch chip only when not `main`** (new) | Lane header: title, Destination, status, priority, owners | Editable Destination, computed Next, Contains (plans), owners, status, priority, branch; blocked-by / blocks editable |
| **Plan** | Only as its segments in the bar and, when it is the next stop, in the Next box | Card: kind label with parked marker, short title, **one row of phase chips P1..Pn coloured by status** (new; this is the "P1 ✔ P2 retired P3–P5 open" summary in one line), owners, priority, dep tags, branch chip when not `main` | **Read-only** (new): phases list with statuses, branch, parked reason, link to the plan file; blocked-by / blocks editable. The plan file is the source; Claude sessions edit it |
| **Phase** | A segment in its pathway's bar; in the Next box when it is next | Card: status glyph, "Phase · <plan short name>", heading title, **two-line summary clipped from the phase's first paragraph** (new; the parser's real summary, not the seed's "Heading: …" text), dep tags. No owners or priority: inherits the plan's | **Read-only**: parent plan, summary, status; blocked-by / blocks editable |
| **Idea / Feature / Bug** | Wall card, one design for all three, kind hue: title, two-line description, priority, **age since filing** (new), pathway pin or "unattached", **a link chip per `links:` entry** (new), dep tags, **for a promoted idea an arrow to what it became** (new) | Rail card (unattached) or Later-column card (attached): same fields; token on the tile when attached | **Fully editable**: title, description, status (the #30 set), priority, pathway selector, links, owners; **Promote** on an idea hands off to #37; blocked-by / blocks editable |

Bugs carry nothing a feature does not; the kind hue, the description line, the age and the link chips do the separating.

### Hover-highlight

- Every rendered element carries the item id. Hovering any of them **glows every other element with that id** (tile, lane header, card, segment, token, pin, dep tag) and outlines the item's **open blockers in red**. No dimming of the rest.

### Palette

- Dark default, light via the existing toggle. Kind hues: **bug = rose, feature = sky, idea = amber**, muted for dark. One hue per pathway from a fixed cycling set, used by its tile edge, lane edge, pins and segments.

### Sharpened

- #36 (inbox page prototype) is now the wall band + rail of this table against #30's states.
- #35 receives the table as the per-kind acceptance criteria and the no-page-scroll rule.
- #27 receives the phase summary requirement (first paragraph under the phase heading) and the plan's parked reason.
