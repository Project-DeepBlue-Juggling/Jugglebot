---
kind: bug
title: 'Front page leaves a third of the screen blank while the inbox wall is one card tall'
status: inbox
created: '2026-10-24'
updated: '2026-10-24T08:10:27Z'
owners: []
priority: high
blocked_by: []
links:
- logbook/2026-10-24-project-map-p4-the-two-pages.md
- https://github.com/Project-DeepBlue-Juggling/jugglebot-map
parent: project-map
filed_by: harrison
---

Owner reaction round, 2026-10-24: on the owner's screen the Front page shows about a third of the page blank below the tiles while the Inbox wall band is barely taller than one card. Cause: the page is laid out to a fixed 860 px budget (tiles 276 px, wall 168 px) tuned to a 900 px viewport instead of filling the viewport; a taller window leaves the rest empty. Fix: the tile grid keeps its fixed height and the wall takes the remaining height (`flex-1 min-h-0`), so on any screen the wall is as tall as the window allows; keep the no-document-scroll rule.
