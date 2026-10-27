---
id: JB-69
kind: feature
title: Map pages update without a manual refresh
status: inbox
created: '2026-10-27'
updated: '2026-10-27T09:18:22Z'
owners: []
priority: high
blocked_by: []
links: []
parent: map-improvements
filed_by: claude
---

Owner ask 2026-10-27: an open page did not show the changes pushed that morning until a manual refresh. Diagnose why the server-sent invalidate did not reach the page, then: pull the data clone every 30 s (was 2 min); refetch everything when the event stream reconnects; and have an open page pick up a newly deployed app build by itself, waiting while a field is being typed in or a form is open (the drawer survives, it lives in the URL).
