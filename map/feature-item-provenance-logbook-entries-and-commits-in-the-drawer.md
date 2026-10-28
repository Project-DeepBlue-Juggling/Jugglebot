---
id: JB-56
kind: feature
title: 'Item provenance: the drawer lists the logbook entries and commits that touched an item'
status: complete
created: '2026-10-25'
updated: '2026-10-25T23:31:22Z'
owners:
- harrison
priority: high
effort: s
hardware: none
owner: none
radius: subsystem
area: map-app
blocked_by: []
links:
- https://github.com/Project-DeepBlue-Juggling/jugglebot-map
- logbook/2026-10-26-map-improvements-features-verified.md
parent: map-improvements
filed_by: claude
---

## Problem

Why an item is complete lives in the logbook and git history, not on the item. Closing an item rides in the fix's commit with its `Logbook-Entry:` trailer (`docs/agents/map-writes.md`), so the link exists, but the drawer does not show it and finding it means leaving the map.

## Proposal

A "History" section in the item drawer listing, newest first:
- commits that touched `map/<kind>-<slug>.md` (`git log --follow`), each with its subject, date and `Logbook-Entry:` trailer;
- the logbook entries those trailers name, plus any entry or plan named in the item's `links:`, each as a link to its rendered page.

The server computes it from its own clone (`~/map-live/Jugglebot`) behind a new read-only endpoint, cached per HEAD.

## Done when

- Opening a closed bug shows the commit that closed it and its logbook entry, one click from the text.
- An item with no history says so instead of showing an empty section.
- A server test pins the endpoint on a fixture repo with a trailer.

## Progress

- 2026-10-26: built and verified in a real browser on the Jetson (logbook/2026-10-26-map-improvements-features-verified.md)
