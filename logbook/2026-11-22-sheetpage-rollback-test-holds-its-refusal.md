---
title: Map app SheetPage rollback test holds its 422 until the optimistic state is seen (BUG-128)
type: bugfix
date: 2026-11-22
status: complete
phase: night-shift
subsystem: map-app
related_items: [BUG-128]
files_changed:
  - jugglebot-map:frontend/src/pages/NextSitting/SheetPage.test.tsx
---

# Map app SheetPage rollback test holds its 422 until the optimistic state is seen

## What and why
The test "an optimistic status rolls back when the server refuses (422)" let its mocked refusal land after a fixed `delay(30)`. The optimistic Done state therefore lived about 30 ms, and on a box loaded by a full gate suite `findByTestId`'s polling could miss it, which failed the app suite on the 2026-11-21 FEAT-61 branch with no change of its own (BUG-128).

The mocked handler now waits on a promise the test resolves only after it has seen `sheet-step-done`, the same pattern the neighbouring "a status click shows at once" test already uses. The test still asserts both halves: the optimistic Done state appears, then rolls back with the progress counter at 0/3. Test-only; no app code changed.

## Verification
- 2026-11-22, `night_shift_nonet.sh npm --prefix .../jugglebot-map/frontend run test -- --run src/pages/NextSitting/SheetPage`: Test Files 1 passed (1), Tests 19 passed (19).
- 2026-11-22, same wrapper with `run typecheck` and `run lint`: clean.
- The failure was load-dependent, so no deterministic red run exists for the old form; the fix removes the timing window instead of widening it.
