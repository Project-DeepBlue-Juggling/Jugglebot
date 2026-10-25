---
kind: ticket
title: 'The /work-inbox slash command: consuming inbox items (research, analysis, implementation) and what happens to an item when done'
type: grilling
parent: project-map
blocked_by:
- ticket-intake-the-slash-command-the-ui-form-inbox-triage-states-and
owners: []
rank: 16
status: complete
links:
- https://github.com/Project-DeepBlue-Juggling/Jugglebot/issues/37
created: '2026-10-23'
updated: '2026-10-23T14:37:58Z'
priority: medium
---

Part of #21

## Question

The charted intake command only WRITES inbox items. The owner (#26, 2026-10-23) also wants a slash command (working name /work-inbox) that loads one or more inbox items and performs the research, analysis or implementation they call for, asking clarifying questions as it goes. Decide: how items are selected (by id, by pathway, by type); which existing skills it dispatches to (research, grilling, implementation-report, investigate); what it does with an item when done (close, promote to a plan under a pathway, promote to a pathway, log); how it records partial progress; and the item fields it needs that #30 must provide.

Blocked by: #30

## Answer

## Answer (owner, 2026-10-23, by standing agreement with the recommendations)

### Invocation and selection

- `/work-inbox [slug ...] [--pathway <slug>] [--kind bug|feature|idea] [--dry-run]`. With no arguments it lists `backlog` items first, then `inbox` items, by priority then age, and asks which to take. Several slugs run in sequence, each to its own close.
- Before working an item it reads the item file, every `links:` target, and the parent pathway's item (its Destination and body) so the work is framed by where it leads. `--dry-run` only prints the classification and the plan of action.

### Dispatch

The item's text decides the skill; the command states its classification and asks only when the text supports two readings:

| The item asks for | Dispatch | Writes back |
|---|---|---|
| facts, options, a comparison | `research` skill (findings file beside the item, `links:` updated) | `## Findings` summary in the body |
| a design decision | `grilling` with the owner, in this session | `## Decision` in the body |
| a plan-sized implementation | `implementation-report` → new plan in `plans/active/` with `pathway:` = the item's parent | item `promoted`, `promoted_to: <plan filename>` |
| a new line of work with a Destination | a pathway file (after a grilling round for the Destination) | item `promoted`, `promoted_to: <pathway slug>` |
| a small, clearly diagnosed fix | the fix in this session, logged with `/log` or `/investigate` as fits | item `complete`, `links:` gains the logbook entry |
| a bug needing hardware diagnosis | hands off to `/investigate` with the item's links as the starting evidence | `in_progress`, `links:` gains the entry |

- **Amendment to #25/#30:** `promoted` and `promoted_to:` apply to all three inbox kinds, not ideas only; a feature or bug that becomes a plan is promoted, not completed.

### When done, and partial progress

- Done = the item's status moves to `complete` or `promoted` and the file is committed and pushed with the #30 rule (`map: <slug> complete`), in the same commit as the work's logbook entry when there is one.
- Partial progress = status `in_progress`, a dated `## Progress` note in the body (what was done, what is next, where the evidence is), committed and pushed before the session ends. The next `/work-inbox` run on that slug resumes from the note.
- Claude moves items through every transition without asking (#30, owner override), and only asks when the classification is ambiguous or the action is a hardware sitting.

### Fields it needs (all already in #25/#30)

- `parent`, `links`, `priority`, `created`, `status`, `promoted_to`, body sections `## Findings`, `## Decision`, `## Progress`.

### Sharpened

- #33 receives the command file.
- #35 receives the `promoted` amendment and `/work-inbox` as part of the session-wiring phase.
