---
id: JB-135
kind: feature
title: Hovering on a CONSOLE message in a recording replay should invoke a line that appears on the playback bar at the time of that message, to indicate when that message occurred relative to the session
status: complete
created: '2026-11-01'
updated: '2026-11-02T09:00:00Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: local
area: gui
blocked_by: []
links:
- logbook/2026-11-02-jb135-console-hover-marks-the-trackbar.md
filed_by: harrison
---

Filed from the Robot tab at 2026-11-01T18:05:06+11:00.

## Progress

- 2026-11-02: Night Shift 2026-11-02: held on night-shift/2026-11-02-JB-135: (b) one audit note open, a hover marker can go stale if the CONSOLE view is hidden by keyboard with the mouse resting on a row; unblocked by setMarker(null) in setConsoleVisible(false) and a re-gate; the feature itself is complete on the branch (gate green, 7926 passed)
- 2026-11-02: owner approved the audit one-liner; applied (setMarker(null) when the CONSOLE view hides), re-gated and merged
