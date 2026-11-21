---
title: Next Sitting Step view - labelled, tinted field groups, list lines and numbered sub-steps (FEAT-61)
type: feature
date: 2026-11-21
status: complete
phase: night-shift
subsystem: map-app
related_items: [FEAT-61]
files_changed:
  - jugglebot-map:frontend/src/pages/NextSitting/SheetView.tsx
  - jugglebot-map:frontend/src/pages/NextSitting/SheetPage.test.tsx
---

# Next Sitting Step view - labelled, tinted field groups, list lines and numbered sub-steps

## What and why
FEAT-61 asked for the expanded Step on the Next Sitting page to be easier to leave and return to, without more height or scrolling. Presentation only; no data, API or server change.
- `do`, `see`, `stop if`, `record` and each Item's pass row are now `role="group"` blocks with an `aria-label` and a 2 px left border tint; `do`, `see`, `stop if` and `record` also get a fixed-width (w-12) label column (sky for see, slate for do/record, emerald for pass). `stop if` carries an amber border, amber tint background and bold amber label, and `data-warning="true"`.
- `see` / `stop if` / `record` text that contains newlines renders one line per entry (a `ul`; blank lines are dropped); single-line text sits in the column beside its label, so it wraps at a narrower width than before.
- A Step with more than one action numbers them as sub-steps (`3a`, `3b`, ...) in front of each action (`data-testid="sub-step"`).
- Same font sizes; no added vertical padding. The label moves from inline to a column, so on a wide screen a Step is not taller (on a phone the label column narrows the content by about 52 px, so long `do` commands can wrap sooner); a multi-line `see` / `stop if` is as tall as its line count, where the run-on text wrapped to about the same. Save buttons, comment boxes and the progress bar are untouched.

## Owner decision (2026-11-22)
The 2026-11-21 audit left two findings: the label column narrows the content by about 52 px on a phone, and sub-step letters repeat after the 26th action. The owner answered TKT-91 "merge as is": the page is only used on a computer with a full-size monitor, and they will judge the look on the live app. The Night Shift of 2026-11-22 merged `origin/main` into the branch (clean) and re-gated it unchanged.

## Verification
- 2026-11-21, `night_shift_nonet.sh nice -n 19 npm --prefix .../jugglebot-map/frontend run test -- --run src/pages/NextSitting`: Test Files 3 passed (3), Tests 36 passed (36) (35 existing + 1 new test).
- 2026-11-21, same wrapper with `run typecheck` (`tsc -b --noEmit`): clean; with `run lint` (`eslint . --max-warnings 0`): clean.
- The new test ("marks each field group, warns on stop if, lists multi-line fields and numbers sub-steps") was shown to FAIL against the original `SheetView.tsx` (copied aside under /tmp, reverted, run, restored). No existing test needed editing.
- 2026-11-22, after merging `origin/main` (f88694a), `night_shift_nonet.sh tools/night_shift_app_suite.sh --app .../jugglebot-map --mapkit .`: EXIT=0 (server 143 passed, 3 skipped; frontend Tests 378 passed, 1 skipped; lint, typecheck and the upstream-name check clean).
