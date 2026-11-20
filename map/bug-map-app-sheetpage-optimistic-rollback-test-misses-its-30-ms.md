---
id: BUG-128
kind: bug
title: Map app SheetPage optimistic-rollback test misses its 30 ms optimistic window on a loaded box and fails the app suite
status: inbox
created: '2026-11-21'
updated: '2026-11-20T16:14:08Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: none
radius: local
diagnosis: hypothesis
area: map-app
blocked_by: []
links: []
filed_by: night-shift
---

Found by the Night Shift of 2026-11-21 while gating FEAT-61. Related: FEAT-61, BUG-121, BUG-127.

The map app test `frontend/src/pages/NextSitting/SheetPage.test.tsx` "an optimistic status rolls back when the server refuses (422)" failed in the app suite on `night-shift/2026-11-21-FEAT-61` (log `temp/reports/night-shift/app-FEAT-61.log`, at line 509 `findByTestId('sheet-step-done')`: element not found). The same tip passes it in isolation, and the night's unit saw it fail on the unchanged source too, while a full Jugglebot gate suite was loading the box.

Hypothesis: the mocked handler holds the 422 reply for only 30 ms (`delay(30)`), so the optimistic Done state exists for about 30 ms, and on a loaded box `findByTestId`'s polling can miss that window entirely.

Fix: hold the mocked reply until the test has seen the optimistic state, for example with a promise the test resolves after `findByTestId`, instead of a fixed 30 ms delay. Test-only, in the app repo.

Why it matters: an app suite run during a full gate suite turns a clean app branch into a HOLD at (a).
