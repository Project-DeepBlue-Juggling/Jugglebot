---
title: "Map improvements: Done view, search, item history, stale flags, nightly chip and quick capture, verified in a real browser"
type: feature
date: 2026-10-26
status: resolved
phase: "map-improvements"
files_changed:
  - map/feature-item-provenance-logbook-entries-and-commits-in-the-drawer.md
  - map/feature-done-view-for-complete-pathways-and-closed-items.md
  - map/feature-search-across-items-plans-and-glossary.md
  - map/feature-stale-items-are-flagged-on-the-map.md
  - map/feature-nightly-status-chip-in-the-map-header.md
  - map/feature-quick-capture-files-a-map-item-from-the-robot-tab.md
subsystem:
  - project-map
tags:
  - project-map
  - jugglebot-map
---

## Summary

The six Features under the `map-improvements` Pathway, filed 2026-10-25, are built. Most of the code is in the `jugglebot-map` fork, deployed on the Jetson (:8765); the Jugglebot halves have their own entries: `2026-10-25-mapkit-stale-items-and-noop-writes`, `2026-10-25-gui-quick-capture` and `2026-10-26-stale-rule-and-qc-edges`.

| Feature | Fork commits | Closed here? |
|---|---|---|
| Item provenance (drawer History) | 533daa2 (endpoint), a494145 (drawer), b576a6c (undated `Logbook-Entry:` trailers resolve to the dated file) | yes |
| Done view | 41b24e4 | yes |
| Search | 517df62, b576a6c (debounce 100 -> 50 ms) | yes |
| Stale items | a494145 (badges), 74826ea (one Progress-line rule shared with mapkit), b576a6c (Front page inbox cards) | yes |
| Nightly chip | 533daa2, a494145 | no: its Done-when asks the owner to confirm the chip does not reverse the "no GUI" choice for the 04:00 run, else it stays off. It is on, pending that word. |
| Quick capture | 533daa2 (CORS for :8081) + the GUI "+" in Jugglebot | no: verified against a throwaway server; the live GUI has not loaded it yet (see Residue) |

## Verification

All of these checks were run on the Jetson on 2026-10-26.

**Unit tests and lint, fork at b576a6c:**
- `npm test`: 269 passed, 1 skipped (the env-gated live contract).
- `npm run typecheck` and `npm run lint` (`--max-warnings 0`): clean.
- `MAPKIT_SRC=~/Desktop/Jugglebot-map pytest tests/ -q`: 65 passed.

**Real browser:** Playwright 1.64.1 with headless Chromium 149, at 1500x900 and 390x844, in light and dark. The live app was only read: it had 166 tasks before the pass and 166 after.
- **Done page:** 3 complete Pathways, newest first, each with its dates, and 26 closed items grouped by Pathway. The kind filter works, and a click opens the drawer.
- **Search:** "toss" returns Throw, labelled "avoid: toss". Keystroke to results takes 55-58 ms after b576a6c; it was about 107 ms before, almost all of it the 100 ms debounce.
- **History:** the Wave camera Pathway lists its logbook document and 4 commits, and the GitHub links return 200. Before b576a6c, 14 of 152 document rows were "missing"; now 0 are.
- **Stale:** Now / Next / Later rings exactly the set that `python -m mapkit items --stale` prints (JB-1, JB-2, JB-3), and the Key lists "stale".
- **Nightly chip:** it reads GREEN, and its tooltip matches `/api/v1/nightly`.
- **Console:** zero errors on every page.
- **Quick capture:** the GUI was served from this tree and routed to a throwaway server whose git origin was a scratch bare repo.
  - Zero key events reached the document while typing, Enter, Tab and Esc were pressed in the form.
  - The filed item carried `id`, `status: inbox`, `filed_by: harrison` and the "Filed from the Robot tab at ..." line.
  - It took 0.63 s from clicking "+" to the confirmation.
  - With the server stopped, the form stayed open with its text.

**Jugglebot gate:** on 2026-10-26, `./run_tests.sh` at e23883cd returned RESULT PASS (7349 passed parallel, 5 serial, 356 s). This is the passing run that `2026-10-25-mapkit-stale-items-and-noop-writes` lacked: its own run had one load flake in `tests/ros/test_replay_overview.py`, which passed alone.

## Residue

- The live checkout `~/Desktop/Jugglebot` holds another Sitting's uncommitted work, so `jugglebot-pull.sh` fetches without merging. The GUI on :8081 therefore lacks the quick-capture "+" and the Ball 1 / Ball 2 labels until that work is committed or set aside.
- At 390 px the app has no horizontal scroll, but its fixed 224 px sidebar clips Now / Next / Later, the drawer and the search panel.
- The archived plan `two-ball-skill-stack.md` has no `pathway:`, so the Done page shows its Pathway with no plans.
- Quick capture records the wall-clock time but not the bag name, because the GUI is never told the rosbag name.
