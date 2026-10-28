---
id: JB-57
kind: feature
title: Nightly status chip in the map header
status: complete
created: '2026-10-25'
updated: '2026-10-26T02:15:37Z'
owners:
- harrison
priority: low
effort: s
hardware: none
owner: none
radius: subsystem
area: map-app
blocked_by: []
links:
- https://github.com/Project-DeepBlue-Juggling/jugglebot-map
- logbook/2026-10-26-map-improvements-features-verified.md
- logbook/2026-10-26-owner-decisions-chip-oval-label-rename-scope.md
parent: map-improvements
filed_by: claude
---

## Problem

`temp/reports/nightly/status` is the only delivery channel for the 04:00 full-tier run (CLAUDE.md, owner's choice: no email, no GUI), so a RED is seen only when a Claude session claims the ticker.

## Proposal

A chip in the map header showing GREEN, RED, DEFERRED or STALE (date older than about two days) with the status line as its tooltip, read by the map server from the live checkout's `temp/reports/nightly/status` (read-only, and the map server never writes to that checkout; this adds a configured path to `~/Desktop/Jugglebot`, since the server's own clone `~/map-live/Jugglebot` has no `temp/`). A click opens `latest.md` rendered.

## Done when

- The chip matches the ticker's reading for each of the five cases, including a missing file (NEVER).
- Reading the file never blocks a request: the server reads it on a timer and serves the cached value.
- The owner confirms that showing it in the GUI does not reverse the "no GUI" choice, or the chip stays off by default.

## Progress

- 2026-10-26: built and deployed, ON; waits only for the owner's word that the chip does not reverse the no-GUI choice (logbook/2026-10-26-map-improvements-features-verified.md)
- 2026-10-26: owner 2026-10-26: keep the chip on; it does not reverse the no-GUI choice (logbook/2026-10-26-owner-decisions-chip-oval-label-rename-scope.md)
