---
title: "Map search lists ongoing Items first when a bare number matches several kinds (FEAT-63)"
type: feature
date: 2026-11-23
status: complete
phase: night-shift
subsystem: map-app
related_items: [FEAT-63]
files_changed:
  - jugglebot-map:frontend/src/lib/search.ts
  - jugglebot-map:frontend/src/lib/search.test.ts
---

# Map search lists ongoing Items first when a bare number matches several kinds (FEAT-63)

## What and why

Searching `20` finds BUG-20, FEAT-20, IDEA-20 and the ticket numbered 20. `searchMap` walked the groups in
display order and spliced every group whose best hit was an exact ID match to the FRONT, so with several such
groups their order came out reversed (Tickets, Ideas, Features, Bugs) and a complete Item's group could lead an
ongoing one's.

Groups are now placed in three bands, each in `SEARCH_GROUPS` display order: (1) best hit is an exact ID match
on an ongoing Item (not `isDone`), (2) best hit is an exact ID match on a closed Item, (3) every other group.
Groups stay grouped by type and nothing changes inside a group. A single closed exact match (`IDEA-7`) still
leads, being the only band-2 group. The module stays pure; the doc comment on `searchMap` states the order.

## Verification

- 2026-11-23, `cd frontend && night_shift_nonet.sh npx vitest run src/lib/search.test.ts`: 16/16 pass
  (new case: open BUG, complete IDEA, open FEAT, open ticket and a plan with no ID match, searched as `20`,
  lists bug, feature, ticket, idea, plan).
- The new case FAILS on the unchanged `search.ts` (source restored from the index, same command, 1 failed |
  15 passed; got ticket, idea, feature, ... instead of bug, feature, ticket, ...).
- 2026-11-23, `npm run typecheck` (`tsc -b --noEmit`) and `npm run lint` (`eslint . --max-warnings 0`), both
  under `night_shift_nonet.sh`: clean. `prettier --check` on the two files: clean.
