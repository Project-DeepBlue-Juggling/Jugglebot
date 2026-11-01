---
id: JB-131
kind: ticket
title: 'Item prefixes: a kind label in the display only, per-kind prefixes with one or per-kind counters, or a PLAN-n prefix for plans alone?'
status: complete
created: '2026-11-01'
updated: '2026-11-01T00:21:22Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: map-app
blocked_by: []
links: []
parent: feature-update-item-naming-convention-based-on-type-eg-pathways-can
type: question
---

JB-118 asks for per-kind Item prefixes (PWAY-n for Pathways, PLAN-n for plans, and so on) because everything sharing the JB- prefix reads as confusing. The ID scheme is the owner's call: the allocator and the resolver live in `mapkit/` (a path the Night Shift gate freezes), the app accepts JB-n everywhere, and `plans/archived/map-item-ids.md` fixed one global sequence on purpose.

Night Shift 2026-11-01 drafted the options into the Item's `## Options (2026-11-01)` section; a unit follows the answer in the owner's Session.

## Options

1. Keep one sequence, add a kind letter in the DISPLAY only (the app and `mapkit items` print `JB-12 (bug)` or a coloured kind pill); ids, links and commit subjects unchanged. No migration.
2. Per-kind prefixes with one shared counter (PWAY-12, BUG-13, IDEA-14 never collide); every existing JB-n keeps resolving as an alias. Migration touches ids in map/, plans/, logbook/ and commit subjects stay historical.
3. Per-kind prefixes with per-kind counters (BUG-1, BUG-2, PWAY-1): the cleanest read and the largest migration (every cross-reference, every test fixture, the app's parser).
4. Plans get their own prefix only (PLAN-n), since plans are the one artefact that is not an Item today; Items stay JB-n.

Recommendation: 1 now (zero migration, answers the confusion in the app), 4 if plans need addressing by number.

## What each answer unblocks

- 1: a map-app unit (display only) plus a small `mapkit items` print change in the owner's Session.
- 2, 3 or 4: a migration plan with its own tests, scheduled by the owner; not Night Shift work.

## Answer

Good thinking! I'm happy with a 'kind' coloured pill being displayed alongside the Item ID, as long as it's visible everywhere the ID is - at least on the Map (if this is impractical, raise this concern and we can discuss our options)
