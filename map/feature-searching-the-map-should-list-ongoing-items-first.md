---
id: FEAT-63
kind: feature
title: Searching the Map should list ongoing Items first
status: complete
created: '2026-11-22'
updated: '2026-11-22T15:55:42Z'
owners: []
priority: low
effort: xs
hardware: none
owner: none
radius: local
area: map-app
blocked_by: []
links:
- logbook/2026-11-23-map-search-lists-ongoing-items-first.md
filed_by: harrison
parent: map-improvements
---

Current behaviour seems to list Tickets > Features > Bugs (or similar) when searching for a shared Item number (eg. "20" for ITEM-20 and BUG-20).

I'd like for the default behaviour to sort the Item types so that any types with ongoing Items with that number (/search result) are listed at the top. The Items should still be grouped by type, and all results for types whose closest match is complete should be listed lower down, in a similar fashion to how they are now.
