---
title: "Night Shift skip reasons live on the Item: four validated keys, `item skip`, close clears them"
type: feature
date: 2026-11-14
status: complete
phase: "night-shift close-out C3"
subsystem: mapkit
files_changed:
  - mapkit/items.py
  - mapkit/itemcli.py
  - tests/sim/test_item_night_skip.py
  - DOCUMENTATION_GUIDE.md
  - docs/agents/map-writes.md
  - logbook/INDEX.md
---

## What

The Night Shift report copied ~110 "Skipped" rows by hand every night and the reasons went
stale (BUG-7, BUG-72). A skip reason is now stored once, on the Item: `night_skip` (ten-value
enum), `night_skip_on` (YYYY-MM-DD), `night_skip_note` (one sentence, <= 160 chars) and
`night_skip_input` (<= 200 chars, required iff the reason starts with `input-`). Validation in
`mapkit/items.py` is the single enforcement point (refused on pathways and question tickets;
the keys come as a set). `python -m mapkit item skip <ID> --reason R --note N [--input I]`,
`--clear`, and `--batch FILE.json [--commit]` (all-or-nothing, one commit `map: night skip
reasons <date> (<n> Items)`, no trailer) write them. `set-status complete|dropped` removes all four.

## Why this design

Skip writes do NOT bump `updated`. The C4 delta walk treats an Item whose `updated` or `status`
moved as changed and walks it; a skip written by the night itself would otherwise make every
skipped Item look changed the next night and defeat the delta. Pinned by
`test_skip_sets_without_bumping_updated_then_clear_restores` (clear restores the file byte for
byte). Consequence: closing through the map app (which does not call `set-status`) leaves the
keys on a closed Item; they are harmless, and the next CLI close or `--clear` removes them.
No Item in `map/` is seeded here; that is a later reviewed step. The map app's server
(`~/map-live/jugglebot-map/server`) loads Items through `mapkit.items.load_item`, has no
strict frontmatter-key list and preserves unknown keys on save, so it does not choke.

## Verification

2026-11-14, `pytest tests/sim/test_item_night_skip.py -q`: 33 passed. Scoped set recorded in
the commit message.
