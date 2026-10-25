---
title: "GUI Map tab (P8) and map-aware slash commands (P9b)"
type: feature
date: 2026-10-25
status: resolved
phase: "project-map - P8, P9b"
related_plan: project-map.md
files_changed:
  - ros_ws/gui/index.html
  - ros_ws/gui/css/app-tabs.css
  - ros_ws/gui/js/app-tabs.js
  - tests/ros/test_gui_app_tabs.py
  - .claude/commands/
  - .claude/agents/
  - docs/agents/map-writes.md
  - DOCUMENTATION_GUIDE.md
  - CLAUDE.md
subsystem:
  - gui
  - docs
tags:
  - tooling
  - project-map
---

## Summary

**P8.** The GUI has a 32 px Robot | Map tab bar; owner ask: "I'd prefer to only need to have one tab open at a time". The map app now runs on the same Jetson (P6), so the Map tab frames `<host>:8765`.
- The frame is created on the first switch to Map, after a health probe that shows a one-line note if the app does not answer, and is then kept alive.
- The robot view is never unmounted or paused; ROS, telemetry, replay and the 3D loop keep running underneath. While Map shows, the robot container is `inert`, so its document-level keyboard handlers cannot act on the hidden view.
- A load always opens on Robot unless the URL says `#map`. The last tab is deliberately not remembered: an operator who reloads must see E-stop, Fault and ROS state first.

**P9b.** The commands and agents call the P9a CLI through one recipe, `docs/agents/map-writes.md`, under the owner's choices of 2026-10-25:
- The Pathway Destination and its open bugs are read silently on start in `/investigate`, `/diagnose`, `/implementation-report` and `/next-phase-prompt`.
- Starting work and progress notes are their own `map:` commit with no trailer.
- Closing an item rides in the fix's commit with its `Logbook-Entry` trailer, so a reverted fix reopens the item. `DOCUMENTATION_GUIDE.md` § 2.10 and `CLAUDE.md` state this exception.
- Subagents never write map files.
- `/archive-plan` leaves `promoted` items promoted, adding a Progress note only, and asks the owner before a Pathway is marked complete.

One `/audit --unstaged` over both units found no blocking issues. Its four warnings and nine notes were applied before the commit:
- `work-inbox` step 4 still stated the pre-hybrid rule;
- a close without `--commit` does not pull, so the recipe now pulls first;
- promoted items are found by grep, because `show` lists only open items;
- the Map tab's stacked "unreachable" notes;
- the persisted Map tab, which hid robot state;
- a URL that broke on an IPv6 host.

## Verification

- 2026-10-25, `pytest -q tests/ros/test_gui_app_tabs.py tests/sim/test_mapkit_*.py tests/sim/test_map_items.py tests/sim/test_plans_index.py tests/sim/test_glossary_avoid_words.py tests/sim/test_docs_links.py`: exit 0 after the audit fixes. The 7 GUI tab tests pin the tablist, the lazy and never-destroyed frame, the bar height taken from `#app`, the pane cleared before each probe, no `localStorage`, `inert` toggled, and the initial tab being `robot` unless the URL says `#map`.
- 2026-10-25, gate `./run_tests.sh -q` (log `temp/logs/gate_p8_p9b_2026-10-25.log`): RESULT PASS (parallel 327 s rc=0, serial 22 s rc=0, total 349 s).
- 2026-10-25, headless Firefox against a scratch GUI server on :8091 at `http://192.168.20.34:8091/#map` and `#robot`, read by eye: the Map tab shows the live project map from the Jetson's app; the Robot tab shows the GUI, disconnected because ROS was not running.

## Follow-up (2026-10-25): theme and font size in the bar, carried into the Map

The owner asked for the GUI's `Light`, `A-` and `A+` controls to sit in the Robot / Map bar, on the right, and to work in the Map on every page.
- **Move:** they moved into `#app-tabs` (ids unchanged, so `theme.js` and `main.js` are untouched; `#sidebar-toolbar` deleted). Their own text is px-sized, so the bar stays 32 px at every font size.
- **Carry into the Map:** `app-tabs.js` watches `data-theme` and `style` on `<html>` with a MutationObserver. It posts `{type: 'jugglebot-gui-prefs', theme, fontSize}` to the frame on its `load` and on every change, with the map origin as targetOrigin.
- **Map side (fork `13f1ad4`):** the app accepts the message only from the same hostname. The theme drives the `dark` class and wins over the stored user preference. The font size scales the whole app by `fontSize / 14` with CSS zoom, sized so the viewport stays filled.
- **Light palette:** the Now / Next / Later and Dictionary colours gained light values, with text at 4.5:1 or better and marks at 3:1 or better.
- **Owner's same-turn ask:** "Signed in as" is removed.

A live round trip turned up one bug, fixed in fork `8620e6f` with a regression test: the Dictionary page forced the `dark` class on itself.

Verification:
- 2026-10-25, `pytest -q tests/ros/test_gui_*.py tests/ros/test_replay_*.py tests/firmware/test_config_drift.py`: 596 passed, 1 skipped, exit 0.
- 2026-10-25, gate `./run_tests.sh -q` (log `temp/logs/gate_gui_prefs_2026-10-25.log`): PASS (parallel 327 s, serial 23 s, total 350 s).
- 2026-10-25: fork vitest 197 passed and 1 skipped; fork CI green (runs 40170580502 and 40170906406).
- 2026-10-25, live round trip through the real message: a test page on :8092 framed the deployed map and posted the prefs. In light at 14 px, the Dictionary and Now / Next / Later rendered light; in dark at 11 px, the whole app scaled down.
