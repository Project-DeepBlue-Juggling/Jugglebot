---
title: "Map: filing refuses a duplicate title and warns on near titles (mapkit.similar), after the Robot tab filed one bug four times"
type: feature
date: 2026-10-30
status: resolved
phase: "map-improvements"
files_changed:
  - mapkit/similar.py
  - mapkit/itemcli.py
  - ros_ws/gui/js/quick-capture.js
  - ros_ws/gui/css/app-tabs.css
  - tests/sim/test_mapkit_similar.py
  - tests/sim/test_map_items.py
  - tests/sim/test_mapkit_item_ids_cli.py
  - tests/ros/test_gui_quick_capture.py
  - docs/agents/map-writes.md
  - .claude/commands/inbox.md
subsystem:
  - project-map
  - gui
tags:
  - map-improvements
  - duplicates
---

## What happened

At 19:33-19:43 the owner filed "With STATE MACHINE window minimized, the Records window doesn't
snap to the left; it remains floating" from the Robot tab four times (JB-103..106, one truncated,
one as an idea). The map server's journal shows each POST answering 201 after 8-12 s: the Jetson
was at load 23 on 6 cores (a peer session's test gate) and the map server runs at Nice 10.
`quick-capture.js` aborts at 8 s with "No answer from the map server - check the inbox before
filing again", the server finished the write anyway, and nothing stopped the retry. JB-104..106
were deleted through the app's API and JB-103 got the full title (map-bot commit `a7485979`).
Allocation is 1 + the highest ID present, so the three freed IDs are reused: JB-104 is now a
different bug (`47e0f5cb`); any older mention of JB-104..106 as this duplicate means the deleted copies.

## What changed

`mapkit/similar.py` is the one title screen, run by every filing path before anything is written:
`mapkit item new` (exit 2) and the map server's create (409, fork change below).

- **Duplicate** = same title as an open Item, ignoring case, punctuation and spacing. Always
  refused, with no override: a retry after a client timeout now gets "Already filed as JB-n".
- **Similar** = a near title, or the same title as a closed Item. Refused until the filer
  confirms (`--allow-similar`; the app and Robot tab show the list and a "File anyway" button
  that resends with `allow_similar: true`). Near is any of: character ratio >= 0.80, >= 3
  shared content words at Jaccard >= 0.40, or >= 4 shared words that make up >= 75 % of the
  shorter title.
- `test_no_two_open_items_share_a_title` (map items gate) catches a duplicate that skipped the
  screen (hand-written file, plain push); a planted copy of JB-103 fails it.

Thresholds were set on the live 103-Item map: no existing pair trips any bar (closest: JB-40 /
JB-41 ratio 0.61; JB-66 / JB-64 Jaccard 0.38; JB-43 / JB-68 containment 0.75 on 3 words), while
three rewordings of JB-103 and a short rewording of the sentence-length JB-101 are all flagged.
It is lexical only: a rewording that shares no words is not caught.

Fork (`jugglebot-map`): `Store.create` calls the screen and answers 409 `code: duplicate | similar`
with the matches; `NewItemModal` shows them inline (Create becomes "File anyway" for similar,
is disabled for a duplicate); `tests/test_similar.py` + two vitest cases.

## Verification

- 2026-10-30, `pytest -q tests/sim/test_mapkit_similar.py tests/sim/test_mapkit_item_ids_cli.py
  tests/sim/test_mapkit_itemcli.py tests/sim/test_mapkit_metrics.py`: 65 passed;
  `pytest -q tests/sim/test_map_items.py tests/sim/test_mapkit_docs.py`: 19 passed (rebased on
  `bf12d79c`, 113 Items, none flagged by the screen); `pytest -q tests/ros/test_gui_quick_capture.py`: 18 passed.
- 2026-10-30, fork `MAPKIT_SRC=<this tree> pytest -q tests/`: 81 passed;
  `npx vitest run`: 313 passed, 1 skipped; `npm run lint` and `tsc -b` clean.
