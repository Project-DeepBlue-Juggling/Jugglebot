---
id: JB-67
kind: bug
title: item_uuid(slug) collides across kinds that share a slug
status: inbox
created: '2026-10-27'
updated: '2026-10-28T02:10:50Z'
owners: []
priority: low
effort: s
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: map-app
blocked_by: []
links:
- plans/archived/map-item-ids.md
parent: map-improvements
filed_by: claude
---

The app server derives each Item's uuid from its slug alone, so two Items of different kinds with the same slug (bug-x and feature-x) get one uuid and the store silently skips the second file. Only mapkit-side writers can create such a pair (the app refuses a duplicate slug). Fix: derive the uuid from the stem (kind + slug), with a one-time redirect for existing deep links. Residue of plans/archived/map-item-ids.md section 7.

## Survey (2026-10-28)

Agent survey, not started.

- Change ONLY Item.uuid (`mapkit/items.py:76`) to item_uuid(stem) and keep item_uuid(key) generic: the fork server uid() (`store.py:54`) also derives user, project, dep, plan and phase ids from it.
- Add legacy_item_uuid(slug) for a one-time redirect.
- The fork records legacy uuid5(slug) -> new uuid at index build (first item wins, matching today survivor) and resolves it in store.task()/resolve and the ?item= route.
- `store.py:582` looks up by slug-uuid after create: must become the stem key.
- Test pinning the old formula: `tests/sim/test_mapkit_items.py:94-100`.
- Land mapkit (Jugglebot main) first, then the fork (its tests copy mapkit from MAPKIT_SRC).
- No item uuid is persisted anywhere in `map/` or `plans/` (grepped 2026-10-28).

## Progress

- 2026-10-28: 2026-10-28 survey (agent, not started): change ONLY Item.uuid (mapkit/items.py:76) to item_uuid(stem) and keep item_uuid(key) generic (detail below)
