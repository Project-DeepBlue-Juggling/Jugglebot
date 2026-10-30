---
id: JB-105
kind: feature
title: 'Pattern catalogue: give every Pattern a JP-n ID, a name and its machine label'
status: complete
created: '2026-10-30'
updated: '2026-10-30T09:44:00Z'
owners: []
priority: medium
effort: s
hardware: none
owner: decision
radius: normative
area: docs
blocked_by: []
links:
- docs/patterns.md
- logbook/2026-10-30-pattern-catalogue-jp-ids.md
filed_by: claude (jugglebot-f5, pattern exploration 2026-10-30)
---

Owner ask 2026-10-30: number and codify the Patterns so they can be talked about.

## Proposal

A flat sequence JP-n (parallel to JB-n) kept in docs/patterns.md, one row per Pattern with these columns:

- ID
- name
- machine label (the skill_node `_PATTERNS` key and the records.md label)
- balls
- Sites
- what moves
- status (flown / planned / idea)
- first flown
- record

## Flown today

| ID | Pattern | Label |
|---|---|---|
| JP-1 | Self-throw | `self_toss` |
| JP-2 | Hop | `hop` |
| JP-3 | Columns one ball | `columns_1ball`, `columns_1ball_fed` |
| JP-4 | Oval one ball | `oval_1ball`, `oval_1ball_fed` |
| JP-6 | Magazine throw | `magazine_throw` |
| JP-7 | Columns | `columns` |

Destination: JP-8 Oval.

## Alongside

- Ideas filed alongside this item carry their JP code in the title.
- The GLOSSARY Pattern entry would then point at the catalogue; that edit goes through the grilling and domain-modeling skills.

## Progress

- 2026-10-30: docs/patterns.md published (nav Patterns), JP-1 to JP-18 registered; logbook 2026-10-30-pattern-catalogue-jp-ids
