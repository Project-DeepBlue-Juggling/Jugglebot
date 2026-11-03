---
id: JB-163
kind: bug
title: CONSOLE search missing some entries?
status: inbox
created: '2026-11-03'
updated: '2026-11-03T17:50:25Z'
owners: []
priority: medium
effort: s
hardware: none
owner: decision
radius: local
diagnosis: hypothesis
area: gui
blocked_by: []
links: []
filed_by: harrison
---

At least the BRIDGE and PLATFORM fw check lines don't appear for a search for "v38" or "v7"

Filed from the Robot tab at 2026-11-03T22:31:17+11:00.

## Findings (2026-11-04)

- The search is not the cause. `matchesQuery` in `ros_ws/gui/js/console-logic.js` is a case-insensitive substring over node name and message; run under node on the exact line texts (`BRIDGE_FW_CHECK: OK — can-bridge v38`, `PLATFORM_FW_CHECK: OK — Platform Teensy v7`), "v38" and "v7" both match.
- The lines are not in the CONSOLE at all. Each is logged once, at bridge start (the Platform line only when the version changes), and the CONSOLE holds only `/rosout` lines that arrived after the page subscribed.
- Two ways they go missing. The ring keeps `RING_CAP` = 5000 lines, about 8-17 min at the measured 5-11 msg/s, so on a longer Sitting the startup lines are evicted. And `/rosout` is volatile on Foxy (`rcl` has no rosout QoS profile there), so a page that connects or reconnects (2 s retry) after the check never sees them.
- Not confirmed which one fired on 2026-11-03: that needs the launch time against the 22:31 search, from the launch log the night cannot read. The versions themselves are shown in the hardware-versions sidebar either way.

## Options (2026-11-04)

1. Keep one-shot startup lines (`*_FW_CHECK`, identity) in a small pinned set the ring cap never evicts. GUI-only, s. It does not cover a late subscription.
2. Show the ring's window at the top of the CONSOLE ("lines since HH:MM:SS, N dropped"), so a missing line reads as outside the window, not as a search miss. GUI-only, xs.
3. Backfill the CONSOLE from the current run's `launch.log` through a `gui_server.py` endpoint. Covers both mechanisms, m.
4. Leave it: the sidebar already shows the versions.

Recommendation: 2 then 1. Both are small, and 2 makes any future miss legible. 3 only if a startup line other than the version checks turns out to matter.
