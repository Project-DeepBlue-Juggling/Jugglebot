---
id: JB-58
kind: feature
title: 'Quick capture: file a bug or idea from the GUI''s Robot tab mid-sitting'
status: in_progress
created: '2026-10-25'
updated: '2026-10-25T13:43:24Z'
owners:
- harrison
priority: high
blocked_by: []
links:
- https://github.com/Project-DeepBlue-Juggling/jugglebot-map
parent: map-improvements
filed_by: claude
---

## Problem

Most bugs and ideas are noticed during a sitting, with the Robot tab in front of the owner. Filing one today means switching to the Map tab or asking Claude, so items are lost or arrive later without their context.

## Proposal

A small "+" control in the GUI's Robot / Map bar opening a two-field form (kind: bug | idea | feature, title, optional one line), which posts to the map server's existing create endpoint. The item gets, automatically:
- `filed_by: harrison`, `status: inbox`;
- the wall-clock time, and the active bag name if a recording is running, in its body, so `/diagnose` can find the moment.

## Done when

- Filing takes under ten seconds and never takes focus away from motion controls for longer than the form is open.
- The item reaches `main` the same way an app edit does, and shows in the inbox rail.
- With the map server down, the form says so and keeps the text instead of losing it.

## Progress

- 2026-10-26: GUI + control and popover landed (cc608ace); needs the map server CORS (fork 533daa2, deployed 2026-10-26) and a GUI page refresh
