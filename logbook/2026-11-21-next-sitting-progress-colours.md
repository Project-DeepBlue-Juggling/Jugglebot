---
title: "Next Sitting progress rail: one strip per Step, coloured by recorded state (BUG-127 slice)"
type: bugfix
date: 2026-11-21
status: complete
phase: night-shift
subsystem: map-app
related_items: [BUG-127]
files_changed:
  - jugglebot-map:frontend/src/pages/NextSitting/SheetView.tsx
  - jugglebot-map:frontend/src/pages/NextSitting/SheetPage.test.tsx
  - jugglebot-map:frontend/src/theme/palette.ts
---

# Next Sitting progress rail: one strip per Step, coloured by recorded state (BUG-127 slice)

## What and why

BUG-127 slice 1 and 2. The Next Sitting rail drew one grey bar per launch group with a green fill
proportional to Steps done, so a Problem Step was indistinguishable from an untouched one. It now draws
one strip per Step, coloured by the Step's recorded state only (never by whether its card is folded or
open): done green, Problem red (status `problem`, or a schema 1 saved `fail` verdict), Skipped amber,
untouched slate. The palette lives in `theme/palette.ts` (`STEP_RAIL_STATES`), with light and dark
variants. Each strip carries a `title` and `aria-label` ("Step 3: More, Problem") and the rail is no
longer aria-hidden, so colour is not the only carrier.

- Owner's counter rule (TKT-90, 2026-11-22): a Problem Step counts as done. The per-launch-group
  `data-done` counter in `SheetRail` now counts every recorded Step (done, Problem, skipped: every state
  but untouched). Strip colours are unchanged (Problem stays red).
- The same rule applies to the one other operator-facing progress count: the `launch-progress` header
  text ("n/total") used `isDone`, which excludes a `problem` Step with no saved notes. It now counts
  every recorded Step via `railState !== 'untouched'`, so header and rail agree. `isDone` itself is
  unchanged (it still drives the folded-card behaviour); only these two counts changed.
- Tests added 2026-11-22: the rail test asserts `data-done` per group; a group of done, Problem,
  skipped and untouched Steps reads `data-done="3"` and header `3/4`; a schema 1 saved `fail` verdict
  gives a red (`problem`) strip; saved notes on a status-less Step give a green (`done`) strip.

Slice only. Not built: marking the Steps that depend on a Problem Step. The sheet format has no
dependency field (a Step has `from_`, `to`, `serves`, `stop_if`, none naming another Step); the owner
wants the sitting-sheet builder to derive dependencies, filed as its own Item, FEAT-62.

## Verification

- 2026-11-21, `npm --prefix jugglebot-map/frontend run test -- --run src/pages/NextSitting`: 3 files,
  36/36 tests pass. The two new/rewritten rail tests fail (2 failed, 18 passed in SheetPage) against
  the pre-change SheetView.
- 2026-11-21, `... run typecheck`: clean (tsc -b --noEmit, no output).
- 2026-11-21, `... run lint`: clean (eslint --max-warnings 0, no output).
- 2026-11-22, `night_shift_nonet.sh npm --prefix jugglebot-map/frontend run test -- --run src/pages/NextSitting`:
  3 files, 46/46 tests pass (SheetPage 30).
- 2026-11-22, same wrapper with `run typecheck` and `run lint`: both clean.
- 2026-11-22, fail-without-fix: with the rail counter reverted to done+skipped, `... run test -- --run
  src/pages/NextSitting/SheetPage` gives 2 failed, 28 passed (the rail test and the counter test).
- 2026-11-22, after the audit's fixes (a test that a saved `fail` verdict wins over a Done status, and the fold test now checks the card unfolded): same wrapper, `run test -- --run src/pages/NextSitting`: 3 files, 47/47 pass; typecheck and lint clean.
