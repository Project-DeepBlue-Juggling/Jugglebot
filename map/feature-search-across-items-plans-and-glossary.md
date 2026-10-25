---
id: JB-59
kind: feature
title: 'Search across items, plans and glossary terms'
status: inbox
created: '2026-10-25'
updated: '2026-10-25T10:00:00Z'
owners:
- harrison
priority: medium
blocked_by: []
links:
- https://github.com/Project-DeepBlue-Juggling/jugglebot-map
parent: map-improvements
filed_by: claude
---

## Problem

Finding a past bug or a plan means remembering its Pathway or its file name; the Dictionary page lists glossary terms but nothing searches across kinds.

## Proposal

One search box in the app header (shortcut `/`) matching titles and bodies of map items, plan titles and scopes, and glossary terms and definitions. Results are grouped by kind, closed items included but ranked below open ones. A click opens the drawer, the plan page or the Dictionary entry.

Client-side over data the app already loads, plus plan scopes and the glossary from their existing endpoints; no index on the server unless measured slow.

## Done when

- A query returns results within 100 ms on the current map size, measured on the Jetson.
- Glossary `_Avoid_` words find their preferred term ("toss" finds Throw).
- A frontend test pins grouping and ranking.
