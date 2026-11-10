---
title: "Arrow-keying through the Map search results scrolls the active result into view (BUG-83)"
type: bugfix
date: 2026-11-11
status: complete
phase: night-shift
files_changed:
  - frontend/src/features/search/SearchBar.tsx
  - frontend/src/features/search/SearchBar.test.tsx
subsystem:
  - map-app
tags:
  - night-shift
  - jugglebot-map
  - search
---

# Map search: arrow keys scroll the active result

**What.** In the app's `SearchBar.tsx`, the arrow keys moved the active result but the results list never scrolled, so the highlight left the panel. An arrow key that changes the active result now sets a keyboard flag. A scroll effect consumes that flag and calls `scrollIntoView({ block: 'nearest' })` on the active option. A hover also changes the active result but does not set the flag, so it never scrolls the list under the pointer. With a single result the active result does not change, so no flag is set and none can go stale.

**History.** Built on the 2026-11-08 night and held twice at the audit. The owner answered TKT-58 (merge as is), and that accepts the one open cosmetic finding: the first hit of a group leaves its group label scrolled out. The 2026-11-11 re-audit found nothing new.

**Verification.** 2026-11-11, `tools/night_shift_nonet.sh tools/night_shift_app_suite.sh --app <app-clone> --mapkit . --log temp/reports/night-shift/app-BUG-83.log` on the merged tip 1f230199: server 131 passed, 3 skipped; frontend vitest 358 passed, 1 skipped; live contract 1 passed; `EXIT=0`. jsdom has no layout, so the tests stub `scrollIntoView`. Only a real browser can confirm that the list scrolls as it should.
