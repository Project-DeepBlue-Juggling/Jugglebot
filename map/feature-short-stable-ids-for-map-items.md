---
id: JB-62
kind: feature
title: Short, stable IDs for map items
status: promoted
created: '2026-10-25'
updated: '2026-10-27T09:16:56Z'
owners:
- harrison
priority: medium
blocked_by: []
links:
- plans/archived/map-item-ids.md
parent: map-improvements
filed_by: claude
promoted_to: map-item-ids.md
---

## Problem

An Item's only identity is its filename stem, often 40-60 characters long, so nobody says it aloud, types it on a phone or cites it in a commit subject; prose cites titles, which drift.

## Proposal

Give every Item a short, unique, stable ID (`JB-<n>`, one global sequence) in its frontmatter, allocated at creation and shown on the tile, in the drawer and in the `mapkit` listings; every command that takes a stem also takes the ID.

## Progress

- 2026-10-25: promoted to `plans/active/map-item-ids.md`.
- 2026-10-27: plan archived 2026-10-27
