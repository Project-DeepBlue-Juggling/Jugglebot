---
id: JB-8
kind: ticket
title: Map item schema and the map/ directory layout
type: grilling
parent: project-map
blocked_by: []
owners: []
rank: 4
status: complete
links:
- https://github.com/Project-DeepBlue-Juggling/Jugglebot/issues/25
created: '2026-10-23'
updated: '2026-10-23T14:37:58Z'
priority: medium
---

Part of #21

## Question

What is the frontmatter schema for map items (Pathway, Feature, Bug, Idea, decision ticket) and how is map/ laid out? Decide: the field set per type (title, type, status, pathway, blocked_by, created, owner, branch, destination for pathways, promoted_to for ideas, tags, related_plan, related_logbook); the status vocabulary and which statuses are terminal; whether blocking is a blocked_by list on the item (matching TaskTree's dependency DAG); the filename rule (filename never changes; slug vs numbered); one directory with the type in frontmatter vs a directory per type; where the inbox lives (a status, not a directory?); how a pathway references its plans (plan frontmatter points up via pathway:, so pathway files carry no list).

Use domain-modeling: write each resolved term (Pathway, Idea, Inbox, Item, Destination) into GLOSSARY.md as it crystallises. Stress-test with: an idea promoted to a pathway; a bug filed on the robot that belongs to no pathway; a pathway completed while one of its plans is parked.

## Answer

## Answer (grilling, owner, 2026-10-23, one round)

Incorporates #30's inputs (fields `created`, `filed_by`, `links`, `promoted_to`; the inbox-kind status set) and #26's seed content.

### Layout and identity

- **`map/` is flat:** one markdown file per item, `kind:` in frontmatter, no per-kind directories. `map/config.yaml` (#31) sits beside the items; the parser ignores non-markdown. The inbox is a status, not a directory.
- **Identity is the filename:** `map/<kind>-<slug>.md`, never renamed, no numeric id, no stored UUID. The server derives the UUID that TaskTree's schemas need as uuid5 of the slug. Two sessions filing the same slug meet as a git add/add conflict at push; the loser re-slugs before its first push (unpublished, so the never-rename rule holds). **Closes the map's "item identity across branches" fog.**

### Fields

| Field | Kinds | Notes |
|---|---|---|
| `kind` | all | `pathway`, `idea`, `feature`, `bug` (no decision-ticket kind; wayfinder tickets are #34's) |
| `title` | all | |
| `status` | all | per-kind sets below |
| `created` | all | date |
| `updated` | all | ISO timestamp, written by the server or `/inbox` on every write; the conflict token (`if_unchanged_since`) |
| `owners` | all | list of user slugs (`harrison`, `claude`) |
| `priority` | all | `low`, `medium`, `high` |
| `parent` | idea, feature, bug | slug of the pathway the item is attached to; absent = unattached |
| `blocked_by` | all | list; each entry a map-item slug or a plan's bare filename; no step-level targets in v1 |
| `links` | all | list of repo paths or URLs (logbook entry, bag path, GitHub issue) |
| `filed_by` | idea, feature, bug | owner or the session |
| `destination` | pathway | required, one line |
| `branch` | pathway | default `main` |
| `promoted_to` | idea | slug of the plan or pathway it became |
| body | all | description, markdown |

### Status per kind

- **Idea / Feature / Bug (#30):** `inbox` → `backlog` → `in_progress` → `complete`; `dropped` from anywhere; `promoted` (idea only). Terminal: `complete`, `dropped`, `promoted`. Triage = leaving `inbox`; attachment (`parent`) is a separate axis.
- **Pathway:** `active`, `complete`, `dropped`; terminal: the last two. "Blocked" is computed from `blocked_by`, never stored.
- **Plans** keep `active | parked | completed | superseded`; steps carry their own status inside the plan (#27).

### Tree edges

- A plan points up with `pathway: <slug>` (#27); an inbox item points up with `parent: <slug>`; a pathway file carries no list of either. The parser maps both to one tree edge. (Two words because `parent_plan:` already means something else on the handoff plans.)

### Closed pathways and the archive

- A closed pathway is an item with status `complete`, kept in `map/`. Day one creates exactly one: `pathway-two-ball-skill-stack` (complete; Destination = its R5 gate, 30 consecutive Ball Butler-fed Catches in one Attempt, met 2026-10-13) as the worked example. The ~62 other archived plans get no retroactive `pathway:` and no pathway items. **Closes the map's "migration of archived plans" fog.**

### Stress tests

- **Idea promoted to a pathway:** the idea becomes `promoted` with `promoted_to: <pathway-slug>`; a new pathway file is created with a Destination and the idea's body as its description.
- **Bug on the robot with no pathway:** `status: inbox`, no `parent`. Done.
- **Pathway completed while one of its plans is parked:** the server **refuses** (409, TaskTree's delete-conflict shape) while any active or parked plan still points at the pathway; the plan is reassigned or archived first. Same rule for `dropped`.

### Glossary entries (CONTEXT.md format; #28/#29 paste them when the file is restructured, not edited now)

```
**Pathway**:
A named line of work with a stated Destination, grouping plans and usually owning a branch. Every Pathway has a Destination; a Pathway whose Destination is met is closed.
_Avoid_: arc, track, epic, theme

**Destination**:
The one-line statement of where a Pathway leads; met or not met, never partly.
_Avoid_: goal, objective, vision

**Item**:
One file in `map/`: a Pathway, Idea, Feature or Bug. Plans are not Items; they are read in place from `plans/`.
_Avoid_: ticket, task, card

**Inbox**:
The Items whose status is `inbox`: filed and not yet triaged. Triage means leaving the inbox, independently of whether the Item is attached to a Pathway.
_Avoid_: backlog (for untriaged work), queue

**Idea**:
An intake Item with no Pathway and no Destination required; promotable to a plan under a Pathway, or to a new Pathway once it has a Destination.

**Feature**:
An intake Item asking for behaviour the robot or its tooling does not have.
_Avoid_: enhancement, story

**Bug**:
An intake Item recording behaviour that is wrong, with where it was seen.
_Avoid_: issue (for a defect), defect (in Item titles)

**Step**:
One entry of a plan's `steps:` list: a Phase or a Rung. Where a Step closes decides which: a Rung closes on a hardware criterion at a Sitting, a Phase closes without one.
_Avoid_: phase (as the generic word), stage, milestone

**Reference document**:
A normative procedure kept beside the plans but never scheduled (e.g. the leg-gain tuning methodology). `kind: reference` in its frontmatter.
```

### Sharpened

- #27 receives: `pathway:` on plans, the tree-edge rule, `blocked_by` targets as bare plan filenames.
- #35 receives: the day-one `map/` seed (five pathways + one closed pathway from #26, the residue bugs, the three ideas, the #17/#18/#19 migrations from #30), the 409 on closing a pathway with live plans.
- #34 receives: whether wayfinder tickets become a kind.
