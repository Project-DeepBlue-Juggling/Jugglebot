---
id: JB-61
kind: pathway
title: 'Map improvements'
status: active
created: '2026-10-25'
updated: '2026-10-25T10:00:00Z'
owners:
- harrison
priority: medium
effort: xl
hardware: none
owner: decision
radius: subsystem
area: map-app
blocked_by: []
links:
- https://github.com/Project-DeepBlue-Juggling/jugglebot-map
- plans/archived/project-map.md
destination: The map shows each item's history, takes new items from the Robot tab mid-sitting, finds anything by text, and surfaces stale items and a red nightly without leaving the GUI.
branch: main
---

Follow-on work for the map app after `plans/archived/project-map.md` closed. The app is the fork at https://github.com/Project-DeepBlue-Juggling/jugglebot-map, served on the Jetson (user unit `jugglebot-map`, redeploy `~/map-live/jugglebot-map/deploy-jetson.sh`); this repo holds the data and `mapkit`.

## Notes

Six Features proposed by Claude on 2026-10-25 and filed at the owner's request. None is scheduled; each waits in the inbox until a plan or `/work-inbox` takes it.

## Decisions so far

- Every Feature here keeps the existing write path: the app and `mapkit` write `map/` files and commit them; nothing adds a database.
- Anything read from the Robot side (bags, the nightly status) is read-only from the map.

## Not yet specified

- Order. Claude's suggestion is the provenance view and quick capture first, since they join the map to the logbook and to sittings.

## Out of scope

- Multi-user features; the owner is the only user.
