---
id: JB-80
kind: bug
title: Clicking off this menu ("+ File new change request") should close the dialog box without saving
status: complete
created: '2026-10-28'
updated: '2026-10-28T10:00:02Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: gui
blocked_by: []
links: []
filed_by: harrison
---

Filed from the Robot tab at 2026-10-28T14:06:53+11:00.

## Progress

- 2026-10-28: Click outside the open quick-capture form closes it via dismiss(); draft kept in localStorage
- 2026-10-28: 2026-10-28 reopened: headless probe on live :8081 shows Robot-pane clicks close the form; only the Map iframe click did not. Blur listener added; Robot-pane report not reproduced.
- 2026-10-28: Owner confirmed 2026-10-28 (Chrome): click-outside closes the form on both tabs after the Map-iframe blur fallback (5d86afa6)
