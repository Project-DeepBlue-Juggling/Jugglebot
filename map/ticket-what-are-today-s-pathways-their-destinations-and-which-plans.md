---
id: JB-19
kind: ticket
title: What are today's pathways, their destinations, and which plans and branches belong to each?
type: grilling
parent: project-map
blocked_by: []
owners: []
rank: 5
status: complete
links:
- https://github.com/Project-DeepBlue-Juggling/Jugglebot/issues/26
created: '2026-10-23'
updated: '2026-10-23T14:37:58Z'
priority: medium
---

Part of #21

## Question

Enumerate the live pathways as the owner sees them, each with a one-line destination, and assign every active and parked plan and every live branch to one. Known material: plans/active on main = two-ball-oval, learner-generalisation, box-sweep-same-site-fidelity, leg-gain-tuning-methodology (INDEX calls it "reference": a pathway, a reference doc, or neither?); the wave-camera plan on the uncommitted 2026-10_wave-camera-can3 checkout; the parked bb-calibration gate; the open residue sections (dress-rehearsals section 2, columns-empty-cup rows 33/34, the LIMIT_JERK sim regression); origin branches main and two-ball-oval.

Also ask which ideas the owner already carries that have no home today, to seed the inbox. The answer is the content the implementation session loads into map/ on day one.

## Answer

## Resolution (grilling, owner, 2026-10-23, two rounds)

This is the content the implementation session loads into `map/` on day one. Terms follow `CONTEXT.md` (Sitting, Rung, Phase, Throw, Feed, Catch); "Pathway" uses the charting definition: an arc with a stated destination that groups plans and usually owns a branch.

### Today's pathways (five)

| Pathway | Destination (one line) | Plans | Branch |
|---|---|---|---|
| **Two-ball oval** | Two balls in one Hand on an oval, every Throw caught and carried back in the Cup, closed by 30 consecutive Ball Butler-fed Catches in one Attempt (P5 Rung gate). | `plans/active/two-ball-oval.md` (P1 complete, P2 retired 2026-10-23, next = the Sitting flying the three owed fixes then Rung 2, P3-P5 not started) | `main` (trunk) |
| **Learner generalisation** | A global plant model with a physics-keyed residual replaces the local-linear Memory and the offline Admissible box, so any Site, Apex, Dwell or yaw can be commanded and changed mid-Pattern. | `plans/active/learner-generalisation.md` (P1-P5 not started); `plans/active/box-sweep-same-site-fidelity.md` (assigned here because P3 decides the box's fate, so the two sequence together) | `main` (trunk) |
| **Wave camera on CAN3** | The Wave camera is visible over CAN3 and automated recording start/stop is working. | `wave-camera-can3-integration.md` (uncommitted, on the other checkout) | `2026-10_wave-camera-can3` |
| **Ball Butler** | Ball Butler feeds reliably enough that no Sitting is lost to the feeder. | `plans/parked/bb-marker-never-matched-advisory.md` (parked; unpark gate in its INDEX row) | `main` (trunk) |
| **Project map and shared dictionary** | This map's own Destination (#21). | none yet; the implementation plan from #35 will be its first | `project-map` |

Two-ball oval is its **own** pathway, not a continuation of the archived two-ball-skill-stack arc: a pathway whose destination is met is closed, and the oval has a new destination. (How archived pathways appear on the map stays in "Not yet specified".)

### Plans that are not plans

- `plans/active/leg-gain-tuning-methodology.md` is a **reference document**, not a plan: gains frozen 2026-07-13, nothing schedulable. It is exempt from pathway assignment and carries a `kind: reference` marker; whether it stays in `plans/active/` or moves under `docs/` is the implementation session's call (ticket #27 territory).

### Branch disposition

- `main` is the shared trunk, owned by no pathway; Two-ball oval, Learner generalisation and Ball Butler all work on it.
- `origin/two-ball-oval` is a fast-forwarded mirror of `main` with no work of its own: **deleted by the implementation session**, not assigned.
- `2026-10_wave-camera-can3` -> Wave camera on CAN3. `project-map` -> Project map.
- The six stale local branches on the Windows box are **not live** and are not on the map: `refactor`, `skill-stack`, `mvp-trajectory-bringup` already have `archive/<branch>` tags and are deleted locally; `demo/bb-led-two-ball-juggle`, `phase-a-plus-accuracy-cal`, `claude/unruffled-beaver` each get an `archive/<branch>` tag, then are deleted.

### Open residue: inbox items, no residue pathway

Residue has no destination, so there is no "skill-stack residue" pathway. On day one these become **inbox items** (Bug or Idea as fits), attached to the pathway whose Sitting would trigger them where one exists, unattached otherwise:

- From `plans/archived/skill-stack-dress-rehearsals-and-residue.md` § 2, the nine rows tagged **(b)**: the 10 % jitter-margin class (`LIMIT_*` refusals) [Two-ball oval]; cup Banking saturating at its 12 deg clamp on small lateral offsets [Two-ball oval]; sim columns with a biased plant for the forward-clip rule; other solves in their own interpreters; NOT_SETTLED retry dead in the field [Ball Butler]; `bb/throw_outcome` carries no throw identity [Ball Butler]; mocap `_finalize_calibration` in a worker process; the emitter 47 ms gap; the magazine feed-tuning loop.
- Columns-empty-cup rehearsal rows 33 (Stop between feeds) and 34 (gate timeout, `ABORTED_BB_NOT_READY`), never exercised on hardware [Ball Butler].
- The levelling contract's int16 offset-truncation defect (OPEN, no owner; read in its archived plan).
- The LIMIT_JERK sim regression named in this ticket.

The **(c)** rows (trivial/cosmetic/watch) stay in the archived plan, read by reference; they are not imported.

### Ideas with no home (day-one inbox seed, type Idea, unattached)

1. A mechanism to search through rosbags from recordings to find specific information.
2. A window showing Jugglebot's all-time personal best (Throws and Catches) for each Pattern.
3. Rename the columns/oval one-ball Patterns to use the Ball1/Ball2 terminology. (Glossary note for #29: "Ball1"/"Ball2" is not a `CONTEXT.md` term yet; define it there before the rename, and the rename itself is its own effort per the map's Out of scope rule on mass renames.)

### Two decisions that sharpened into tickets (created as children of #21)

- **Inbox page** (wayfinder:prototype, blocked by #24 and #30): a Keep-style wall of every Idea, Feature and Bug item: scroll, click-to-expand, edit, delete, attach to a pathway.
- **`/work-inbox` slash command** (wayfinder:grilling, blocked by #30): loads one or more inbox items and performs the research, analysis or implementation they call for, asking clarifying questions as it goes. New beside the charted intake command, which only writes items; this one consumes them, so what it does with an item when done (close, promote to a plan, log) is the open question.

### Glossary (deferred to #28/#29, candidate wording carried here)

- **Pathway**: a named line of work with a stated destination, grouping plans and usually owning a branch. Every pathway has a destination. _Avoid_: arc, track, epic.
- **Idea**: an intake item with no pathway and no destination required; promotable to a plan under a pathway, or to a new pathway once it has a destination.
- **Inbox**: where Idea, Feature and Bug items land before triage attaches them to a pathway.
- **Reference document**: a normative procedure kept beside the plans but never scheduled (e.g. the leg-gain tuning methodology).

Not written into `CONTEXT.md` now: #28 re-confirms the file's name and per-term fields, and entries written into a file about to be restructured would be a second edit for no gain.
