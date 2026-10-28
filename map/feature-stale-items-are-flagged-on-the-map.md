---
id: JB-60
kind: feature
title: 'Stale items: flag items left in the inbox or in progress too long'
status: complete
created: '2026-10-25'
updated: '2026-10-25T23:31:23Z'
owners:
- harrison
priority: medium
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

An item can sit in `inbox` or `in_progress` indefinitely with nothing on the map showing it has been left behind; `/work-inbox` only sees what it is pointed at.

## Proposal

- A badge or dimming on items whose last `updated` is older than a threshold per status (proposed: inbox 30 days, backlog 60, in_progress 14), on the Front page, Now / Next / Later and the inbox rail.
- The thresholds live in one place in the app's config.
- a new flag, `python -m mapkit items --stale`, lists the same set, so a command can offer them.

## Done when

- The badge and the CLI agree on the same fixture set (one test each side, same dates).
- A Progress note resets the clock and a save that changes nothing does not: `updated` is bumped by every write (DOCUMENTATION_GUIDE § 2.10), so either the app skips no-op writes or staleness is dated from the last `## Progress` line, decided when this is planned.

## Progress

- 2026-10-26: 2026-10-25 - mapkit half: is_stale, items --stale, no-op writes (Jugglebot side); app badge pending (fork)
- 2026-10-26: built and verified in a real browser on the Jetson (logbook/2026-10-26-map-improvements-features-verified.md)
