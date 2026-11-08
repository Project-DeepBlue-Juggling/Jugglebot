---
title: "GUI: the Records panel sits hard left and dodges the collapsed State Machine card only when they would overlap (BUG-38)"
type: bugfix
date: 2026-11-01
status: complete
phase: gui
files_changed:
  - ros_ws/gui/js/records-model.js
  - ros_ws/gui/js/records-panel.js
  - ros_ws/gui/css/records-panel.css
  - tests/ros/js/records_hover_place_harness.js
  - tests/ros/test_gui_records_hover_place.py
  - map/bug-with-state-machine-window-minimized-the-records-window-doesn.md
  - docs/project-map.md
  - logbook/INDEX.md
subsystem:
  - gui
tags:
  - ux
---

# BUG-38: Records panel snaps left

**What.** With the State Machine card collapsed (bottom-left, 210 px wide, growing up from the
command strip), the Records panel was fixed 210 px + two gaps in from the left, so it floated in
the middle of the scene top. It now sits at `left: var(--gap)` under the connection indicator.
`records-panel.js` `placePanel` adds `.dodge-minimap` (the old offset) only while the collapsed
card reaches the panel's vertical band (a short window). A `ResizeObserver` on the panel, the
card and `#viewer-pane` re-runs it, since a pane resize moves the bottom-anchored card without
resizing it. Owner's choice (2026-11-01) over "always hard left", where the card (z 11) would
cover the list's bottom.

**Why it cannot oscillate.** The decision (`dodgesMinimap` in `records-model.js`, pure) compares
only vertical extents (`records.bottom + gap > card.top`), and dodging moves the panel only sideways.
With the card expanded, the existing `#viewer-pane.minimap-expanded #records-panel` rule wins on
specificity, and `dodgesMinimap` returns false anyway.

## Verification

- 2026-11-01, `pytest tests/ros/test_gui_records_hover_place.py tests/ros/test_gui_records_panel.py -q`:
  16 passed in 0.54 s. The dodge cases cover a tall pane, a reaching card, a card within one gap,
  expanded, and either element hidden, plus source checks of the wiring and the CSS default.
- Not run: a live browser pass (owner to eyeball at the next GUI load).
