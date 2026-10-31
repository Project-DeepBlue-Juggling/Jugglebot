---
id: JB-89
kind: bug
title: 'GUI: Persistent ''View'' setting selections'
status: in_progress
created: '2026-10-28'
updated: '2026-10-31T20:06:37Z'
owners:
- night-shift
priority: medium
effort: s
hardware: none
blocked_by: []
links: []
filed_by: harrison
---

Choosing to (eg) hide the Jugglebot model should persist a GUI refresh

Filed from the Robot tab at 2026-10-28T21:21:39+11:00.

## Night Shift hold (2026-11-01)

The branch makes every View-menu scene toggle survive a page refresh (`ros_ws/gui/js/view-toggles.js`, one localStorage key, wired in `main.js`; node harness and five tests; entry logbook/2026-11-01-jb89-view-menu-toggles-persist.md). Gate green on the tip.

- Open audit finding: the save rewrites the whole map from a snapshot read when the menu was built, so a second tab's later choice for another group is overwritten by this tab's next toggle (last writer wins, display only). Fix shape: re-read the store in the change handler before saving.
- Unblocked by the owner's call on that finding, then a re-gate.

## Progress

- 2026-11-01: Night Shift 2026-11-01: held on night-shift/2026-11-01-JB-89 (fix complete, Item closed on the branch, gate green): one audit finding open, the cross-tab stale snapshot on save; unblocked by the owner's call on it, then a re-gate (detail below)
