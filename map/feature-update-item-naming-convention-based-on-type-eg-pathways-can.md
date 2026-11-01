---
id: JB-118
kind: feature
title: Update item naming convention based on type (eg. Pathways can be "PWAY-<N>" and Plans can be "PLAN-<N>" etc.
status: complete
created: '2026-10-30'
updated: '2026-11-01T01:04:55Z'
owners: []
priority: medium
effort: m
hardware: none
owner: decision
radius: cross-cutting
area: map-app
blocked_by: []
links: []
filed_by: harrison
---

Having everything share a JB- prefix is a little confusing

Filed from the Robot tab at 2026-10-30T23:35:51+11:00.

## Options (2026-11-01)

1. Keep one sequence, add a kind letter in the DISPLAY only (the app and `mapkit items` print `JB-12 (bug)` or a coloured kind pill); ids, links and commit subjects unchanged. No migration.
2. Per-kind prefixes with one shared counter (PWAY-12, BUG-13, IDEA-14 never collide); every existing JB-n keeps resolving as an alias. Migration touches ids in map/, plans/, logbook/ and commit subjects stay historical.
3. Per-kind prefixes with per-kind counters (BUG-1, BUG-2, PWAY-1): the cleanest read and the largest migration (every cross-reference, every test fixture, the app's parser).
4. Plans get their own prefix only (PLAN-n), since plans are the one artefact that is not an Item today; Items stay JB-n.

Recommendation: 1 now (zero migration, answers the confusion in the app), 4 if plans need addressing by number.

A question ticket carries this to the owner; the answer under its `## Answer` re-opens this Item.

## Progress

- 2026-11-01: Answered by JB-131 (display only, one JB-n sequence kept); shipped in the map app (fork cc2f3d4, deployed 2026-11-01): a kind pill beside every Item ID, JB-n in reports links to the drawer.
