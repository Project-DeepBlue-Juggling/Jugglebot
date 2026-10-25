---
kind: feature
title: Drawer must not cover the inbox rail on Now / Next / Later
status: complete
created: '2026-10-24'
updated: '2026-10-24T08:43:37Z'
owners: []
priority: medium
blocked_by: []
links:
- logbook/2026-10-24-project-map-p4-the-two-pages.md
- https://github.com/Project-DeepBlue-Juggling/jugglebot-map/commit/5ca6452
parent: project-map
filed_by: harrison
---

Owner reaction round, 2026-10-24: expanding the edit panel hides the Ideas / Bugs / Features rail on Now / Next / Later, which the owner dislikes. Fix: the drawer takes space from the lanes (the grid shrinks, the rail stays), or opens on the left, or the rail collapses to a narrow strip that stays readable; decide together with the Now / Next / Later re-layout ticket.

## Progress

2026-10-24: fixed in the fork, commit `5ca6452` (vitest-covered, screenshot-checked at 1500x900); live on the Pi after its next `deploy.sh`.
