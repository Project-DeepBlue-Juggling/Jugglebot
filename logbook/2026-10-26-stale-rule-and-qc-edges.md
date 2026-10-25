---
title: "One Progress rule for the stale badge and items --stale; quick-capture edge cases"
type: bugfix
date: 2026-10-26
status: resolved
phase: "map-improvements - audit findings"
files_changed:
  - mapkit/items.py
  - tests/sim/test_map_stale.py
  - ros_ws/gui/js/quick-capture.js
  - tests/ros/test_gui_quick_capture.py
subsystem:
  - docs
tags:
  - tooling
  - project-map
---

## Summary

Two audit findings on the map-improvements Features.

**A. Progress parse divergence.** The map's stale badge (fork, `frontend/src/lib/done.ts` `progressDates`, used by `stale.ts`) and `python -m mapkit items --stale` (`mapkit/items.py` `_progress_dates`) shared thresholds and the `created` fallback but parsed Progress lines differently (separator after the date, which `## Progress` sections, heading case, impossible dates), so the two could disagree about the same Item. One rule now, on both sides: a line is `YYYY-MM-DD` after an optional `-`/`*` bullet followed by `:`, `-`, en or em dash; EVERY `## Progress` section counts (an Item can gain a second by hand; first-only silently ignored attention); the heading matches case-insensitively; an impossible date (`2026-02-30`) is skipped. A grep over `map/*.md` before the change found 14 live Progress lines, all identical under old and new parse. The shared fixture gained rows 11-14 (no separator: does not count; lower-case heading: counts; two sections: the second counts; invalid date newer than a valid one: skipped), pinned in `tests/sim/test_map_stale.py` (`row11`..`row14`) and in the fork's `frontend/src/lib/stale.test.ts` (`row 11`..`row 14`). The fork change (not committed here; the orchestrator commits it in `~/Desktop/jugglebot-map-nnl`): `frontend/src/lib/done.ts` (`progressDates`: all sections, case-insensitive heading, calendar-date check), `done.test.ts` (one added case; existing assertions unchanged and still true), `stale.test.ts`.

**B. Quick-capture edges** (`ros_ws/gui/js/quick-capture.js`). (1) Esc, Cancel and the `+` toggle go through `dismiss()`, which does nothing while a request is in flight, so a late failure is still on screen. (2) Errors go through `showError()`: into the form, or into the fading confirmation element if the form is hidden. (3) The `+` button stops `keydown`/`keyup`/`keypress` propagation, so Space on it no longer reaches the chart pause (`telemetry-charts.js`) or the replay trackbar handler. (4) An abort timeout after the POST was sent now says "No answer from the map server - check the inbox before filing again" (the server has no idempotency key, so the Item may exist); a timeout on the project lookup, or a plain network error, keeps "Map server unreachable - text kept". Static tests pin (1), (3), (4) and the toast route.

No control or hardware path touched.

## Verification

2026-10-26, `pytest tests/sim/test_map_stale.py tests/sim/test_mapkit_items.py tests/sim/test_map_items.py tests/ros/test_gui_quick_capture.py tests/sim/test_glossary_avoid_words.py -q`: 49 passed. Fork, 2026-10-26, `npm test` / `npm run typecheck` / `npm run lint` in `frontend/`: 254 passed (1 skipped), typecheck and lint clean. Gate 2026-10-26, `./run_tests.sh -q`: PASS (parallel 368 s rc 0, serial 22 s rc 0, total 390 s; the TypeError thread warning in the replay overview drain is a pre-existing warning, not a failure).
