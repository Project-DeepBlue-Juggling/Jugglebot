---
title: "Next Sitting runbook row saves on Ctrl+Enter, keeps input on a failed save, and is read-only while saving (BUG-60)"
type: bugfix
date: 2026-11-11
status: complete
phase: night-shift
files_changed:
  - frontend/src/pages/NextSitting/RunbookRow.tsx
  - frontend/src/pages/NextSitting/NextSitting.tsx
  - frontend/src/pages/NextSitting/NextSittingView.tsx
  - frontend/src/pages/NextSitting/NextSitting.test.tsx
subsystem:
  - map-app
tags:
  - night-shift
  - jugglebot-map
  - next-sitting
---

# Next Sitting Ctrl+Enter save

**What.** The notes textarea of a Next Sitting runbook row (`RunbookRow.tsx`) saves the result on Ctrl+Enter (Cmd+Enter too), the same path as the Save result button. A failed save now keeps the row's verdict and notes (the save callback returns a boolean; only success clears), so a result is not lost to a transient error. An IME guard (`isComposing` / keyCode 229) stops a composition-confirming Enter from saving, a key-repeat guard stops a held key from posting twice, and an in-flight ref makes a second trigger a no-op. Tonight's fix: the row is read-only while its save is in flight (textarea `readOnly`, verdict buttons `disabled`), because Ctrl+Enter leaves focus in the textarea and anything typed or picked during a slow save was wiped when the success cleared the row.

**Why this way.** `readOnly` (not `disabled`) on the textarea keeps focus where the operator left it. Closing the window removes the race; merging typed text into the cleared state would have been a guess about intent.

**Verification.** 2026-11-11, `npx vitest --run src/pages/NextSitting` (under `night_shift_nonet.sh`): 28/28 pass (2 files). The new test (row read-only and verdict buttons disabled during a held save, editable and cleared after) fails with the `readOnly`/`disabled` lines removed (`expected element to have attribute readonly`) and passes restored. `npm run typecheck` and `npm run lint` clean. jsdom has no real typing or focus feel; only a real browser confirms it on a slow save.

**History.** Built on the 2026-11-08 night and held twice at the audit (TKT-49, TKT-57). The owner chose option 2 on TKT-57: apply only the read-only row, and add no Cmd+Enter or key-repeat test. App suite on the merged tip 4406950 (2026-11-11, `tools/night_shift_nonet.sh tools/night_shift_app_suite.sh --app <app-clone> --mapkit . --log temp/reports/night-shift/app-BUG-60.log`): server 131 passed, 3 skipped; frontend vitest 362 passed, 1 skipped; live contract 1 passed; `EXIT=0`.
