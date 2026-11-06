---
id: JB-159
kind: bug
title: GUI Chart hover-over callouts overlap
status: complete
created: '2026-11-03'
updated: '2026-11-05T18:30:27Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: gui
blocked_by: []
links:
- logbook/2026-11-04-gui-callout-pills-no-overlap.md
- logbook/2026-11-06-jb159-closed-owner-confirmed.md
filed_by: harrison
---

When data lines are near each other (eg. position and temp), the hover-over callout 'pills' overlap each other and any behind the front one are invisible

Filed from the Robot tab at 2026-11-03T12:42:47+11:00.

After the 2026-11-04 Night Shift work, this doesn't seem to be fixed? I hard-refreshed the GUI and opened a recording, and the hover-over callouts still overlap

## Progress

- 2026-11-06: Owner confirmed in JB-170: the 2026-11-04 fix works; the reopen was an incorrect checkout
