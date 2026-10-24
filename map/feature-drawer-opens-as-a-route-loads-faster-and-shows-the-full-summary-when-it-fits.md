---
kind: feature
title: 'Drawer: open as a route so Back works, load faster, show the full summary when it fits the window'
status: inbox
created: '2026-10-24'
updated: '2026-10-24T08:10:27Z'
owners: []
priority: medium
blocked_by: []
links:
- logbook/2026-10-24-project-map-p4-the-two-pages.md
- https://github.com/Project-DeepBlue-Juggling/jugglebot-map
parent: project-map
filed_by: harrison
---

Owner reaction round, 2026-10-24: the edit panel is nice but takes a while to load; the owner wants to go back to the previous page (browser Back) especially with the links as clean as they are; the Summary section should be longer - as long as the whole panel fits the window height, nothing should be truncated. Three parts: (1) the open item becomes URL state (`?item=<id>` or `/projects/<p>/item/<id>`) so Back closes it and a link opens it; (2) the drawer renders from the already-loaded task list first and only refetches the detail (dependencies) in the background, instead of waiting on `GET /tasks/{id}`; (3) the summary/description is not clipped in the drawer - the panel scrolls only when its content is taller than the window.
