---
id: JB-69
kind: feature
title: Map pages update without a manual refresh
status: complete
created: '2026-10-27'
updated: '2026-10-27T09:45:32Z'
owners: []
priority: high
blocked_by: []
links:
- logbook/2026-10-27-map-live-updates-scroll-plan-stack-pills.md
parent: map-improvements
filed_by: claude
---

Owner ask 2026-10-27: an open page did not show the changes pushed that morning until a manual refresh. Diagnose why the server-sent invalidate did not reach the page, then: pull the data clone every 30 s (was 2 min); refetch everything when the event stream reconnects; and have an open page pick up a newly deployed app build by itself, waiting while a field is being typed in or a form is open (the drawer survives, it lives in the URL).

## Progress

- 2026-10-27: built in jugglebot-map bb1fc29 and deployed on the Jetson
