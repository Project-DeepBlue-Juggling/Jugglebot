---
id: JB-4
kind: ticket
title: 'Claude-session wiring: CLAUDE.md, DOCUMENTATION_GUIDE, slash commands and agents that read or write the map'
type: grilling
parent: project-map
blocked_by:
- ticket-map-item-schema-and-the-map-directory-layout
- ticket-plan-frontmatter-extension-the-phases-list-and-generating
- ticket-glossary-md-entry-format-the-claude-md-term-name-list-and
owners: []
rank: 12
status: complete
links:
- https://github.com/Project-DeepBlue-Juggling/Jugglebot/issues/33
created: '2026-10-23'
updated: '2026-10-23T14:37:58Z'
priority: medium
---

Part of #21

## Question

How do Claude sessions use the system without being told each time? Decide: the CLAUDE.md additions (the generated term-name list and its lookup rule; a one-paragraph map pointer: where items live, how to file, how to update a phase status); which existing slash commands change (/archive-plan, /implementation-report, /log, /investigate, /next-phase-prompt, /logbook) and what each writes to map/ or plan frontmatter; whether a new filing command exists; the DOCUMENTATION_GUIDE layer the map and glossary join; a session-start rule (read the pathway for the branch you are on?); and which of today's CLAUDE.md narrative (for example the skill-stack arc paragraph) moves into pathway items so CLAUDE.md shrinks. Respect the token rule: nothing large loads by default.

## Answer

## Answer (owner, 2026-10-23, by standing agreement with the recommendations)

Everything a Claude session reads or writes about the map, enumerated from the upstream answers (#25, #27, #28, #30, #31, #34, #37). #35 turns this into one phase.

### Normative docs

- **`CLAUDE.md`**: the generated term block between `<!-- glossary-terms:start/end -->` with the hand-written rule sentence (#28); `CONTEXT.md` → `GLOSSARY.md` in the three existing lines; a new "Project map" paragraph under Agent skills: where `map/` is, that `/inbox` files items, that a session working on a Pathway reads its item first, that plans carry `pathway:`/`steps:`, that INDEX files and the term block are generated (`python -m mapkit gen`, never hand-edited), and the Pi app URL. Fix `runsheet` at CLAUDE.md:128 (it is on the glossary's Avoid line).
- **`DOCUMENTATION_GUIDE.md`**: § 2.6 plans: the new fields, `steps:`, generated INDEX files, parked fields replacing the status comment; § 2.9 → GLOSSARY.md; a new § 2.10 "Project map (`map/`)" layer with the item kinds and the commit rule; § 2.5 logbook: `links:` from items point at entries, entries keep `related_issues:` for known_issues ids.
- **`docs/agents/issue-tracker.md`**: the Wayfinding operations section rewritten to the file-based operations of #34 (map = pathway item, ticket kind, frontier/claim/resolve via `mapkit`); GitHub issues kept as the public door only. **`docs/agents/domain.md`**: GLOSSARY.md, and the skill's lazy-creation note updated. **`docs/agents/triage-labels.md`**: unchanged (outsider issues).

### Slash commands

- **New**: `/inbox` (#30), `/work-inbox` (#37).
- **`/investigate`**, **`/log`**: the gated offer to file open follow-ups as items at entry close (#30). **`/archive-plan`**: the same offer for residue; sets fields and runs `python -m mapkit gen` instead of hand-editing rows; parking sets `parked`, `parked_why`, `unpark` (#27). **`/implementation-report`**: writes `pathway`, `branch`, `scope`, `last_updated`, `steps` and runs the generator. **`/next-phase-prompt`**: reads `steps:` for the phase it is asked for. **`/commit`**: no change (the generator's output is ordinary staged content). **`/logbook`**, **`/diagnose`**, **`/audit`**: no change.

### Agents

- **logbook-updater**: status ladder unchanged; when the status it sets is not `resolved`, it returns the open follow-ups so the calling command can make the offer. **plan-writer**: emits the new frontmatter fields and a `steps:` list mirroring its headings. **plan-reviewer**: checks `steps:` against headings and that every step is `complete` or `retired` before archival. **fix-proposer**, **audit-***: no change.

### Tests the session relies on

- Currency tests (INDEX ×3, CLAUDE.md term block, `docs/project-map.md`, `docs/glossary.md`), the steps mirror test, the plan field test, the Avoid-word test, the `mapkit` parser suite. All in `tests/sim/`, unmarked, per-commit.

### Sharpened

- #35 receives this as the "session wiring" phase, after `mapkit` and before the fork is pointed at `main`.
