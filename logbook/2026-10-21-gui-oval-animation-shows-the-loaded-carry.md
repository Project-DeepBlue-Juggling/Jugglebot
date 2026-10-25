---
title: "GUI Juggle panel: the oval pattern's diagram and animation show the loaded carry (it drew the hop)"
type: bugfix
date: 2026-10-21
status: resolved
phase: "two-ball-oval - P1 flown-in (GUI)"
related_plan: two-ball-oval.md
files_changed:
  - ros_ws/gui/js/juggle-panel.js
subsystem:
  - gui
tags:
  - gui
  - oval
---

# GUI Juggle panel: the oval diagram and animation show the loaded carry

## What changed

Both oval entries in `JUGGLE_PATTERNS` (`oval_1ball`, `oval_1ball_fed`) were
`kind: 'one'` with two sites, so `drawFrame` took the hop branch: the ball
flew P1 -> P2, then P2 -> P1, with no carry. The owner noticed at the
2026-10-21 22:53 sitting that the oval icon "is basically the same as the
hop pattern".

`ros_ws/gui/js/juggle-panel.js` (+25 lines): an `isOval(p)` helper; the
static diagram gains a second dashed arc under the rim from P2 back to P1 so
the two arcs form the oval; `drawFrame` gains an oval branch ahead of the hop
branch: the ball flies P1 (throw site) -> P2 (catch site) on the same parabola
as before, the cup then carries the ball back P2 -> P1 over the dwell (eased
x, the usual scoop dip), and the next throw leaves from P1. Colours, sizes
and the cycle length (flight + dwell) are unchanged. The carry is spread
evenly over the dwell; it does not separate the catch-settle time from the
hand-motion time.

The GUI unit serves this worktree's `ros_ws/gui` statically on :8081, so a
hard refresh (Ctrl+Shift+R) shows it; no service restart.

## Verification

(2026-10-21, `pytest tests/ros/test_gui_juggle_defaults.py -q`, the only test
that reads `juggle-panel.js`) -> 6 passed in 0.43 s. (2026-10-21,
`node --check ros_ws/gui/js/juggle-panel.js`) -> exit 0. Not yet viewed in a
browser by the owner at commit time.
