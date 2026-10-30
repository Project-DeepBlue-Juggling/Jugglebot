---
id: JB-20
kind: ticket
title: Write the phased implementation plan and hand off
type: task
parent: project-map
blocked_by:
- ticket-prototype-tasktree-frontend-running-against-a-thin-file
- ticket-what-are-today-s-pathways-their-destinations-and-which-plans
- ticket-plan-frontmatter-extension-the-phases-list-and-generating
- ticket-intake-the-slash-command-the-ui-form-inbox-triage-states-and
- ticket-the-fork-repo-name-runtime-shape-what-is-kept-stubbed-or
- ticket-the-generated-mkdocs-view-of-the-map-and-glossary
- ticket-claude-session-wiring-claude-md-documentation-guide-slash
- ticket-wayfinder-tracker-operations-on-the-new-system
owners: []
rank: 14
status: complete
links:
- https://github.com/Project-DeepBlue-Juggling/Jugglebot/issues/35
created: '2026-10-23'
updated: '2026-10-23T14:37:58Z'
priority: medium
effort: m
hardware: none
owner: decision
radius: local
area: docs
---

Part of #21

## Question

Everything is decided; produce the deliverable. Run /implementation-report to write `plans/active/project-map-and-shared-dictionary.md` (filename never changes).

**Phases, in dependency order:**

1. Shared parser package and tests.
2. `GLOSSARY.md` landing with the CLAUDE.md term list and the Avoid test.
3. `map/` seeded from ticket 05, plus plan-frontmatter migration and generated INDEXes.
4. The fork repo and thin server.
5. The mkdocs view.
6. Slash-command and doc wiring.
7. The tracker doc.

Each phase gets checkable acceptance criteria, the (date, command, result) verification convention, and risk lines.

Add the INDEX row and the logbook entry for the charting work, and run the gate before commit. The owner reviews the plan; a fresh session then implements phase 1 via /next-phase-prompt.

## Answer

## Resolved (2026-10-24)

The phased implementation plan is written and committed on branch `project-map`: [`plans/active/project-map.md`](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/project-map/plans/active/project-map.md), with its `plans/active/INDEX.md` row and logbook entry `logbook/2026-10-24-project-map-plan-written.md`.

- Six phases, each with files, tests, gate and the ticket numbers it implements: **P0** `mapkit/` package and generated plan boards; **P1** glossary rename and term block; **P2** `map/` seed and session wiring; **P3** fork skeleton and thin server; **P4** the two pages; **P5** Pi deployment and publishing. Dependencies: P1←P0, P2←P0+P1, P3←P0 (may run in parallel with P1/P2), P4←P3, P5←P2+P4.
- The plan designs nothing; its § 1 indexes the decision each ticket fixed, and § 4 lists the residue (parse-error behaviour on a malformed hand-edited file, the logbook backfill Idea, branch housekeeping, the deferred smoke e2e).
- It is the first plan carrying the frontmatter it introduces (`pathway`, `branch`, `scope`, `steps`), ahead of the P0 parser.
- Verification: 2026-10-24, `python -m pytest tests/sim/test_logbook_front_matter.py tests/sim/test_plans_index.py -q --noconftest`: 94 passed in 0.47 s. `/audit --unstaged` run once at phase end.

Next: a fresh session opens the plan and starts P0. The map's Destination is met; #21 closes with this ticket.
