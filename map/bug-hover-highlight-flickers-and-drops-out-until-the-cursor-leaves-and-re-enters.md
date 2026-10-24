---
kind: bug
title: Hover highlight flickers and drops out until the cursor leaves and re-enters
status: complete
created: '2026-10-24'
updated: '2026-10-24T08:43:37Z'
owners: []
priority: high
blocked_by: []
links:
- logbook/2026-10-24-project-map-p4-the-two-pages.md
- https://github.com/Project-DeepBlue-Juggling/jugglebot-map/commit/7585cac
parent: project-map
filed_by: harrison
---

Owner reaction round, 2026-10-24: hovering most panels is glitchy - the glow flashes and often disappears completely until the cursor moves off and back onto the panel. Likely cause: `features/hover/` uses `mouseover`/`mouseout` with `stopPropagation`, so moving between a card's child elements fires `mouseout` on the card and clears the hovered id. Fix: use `mouseenter`/`mouseleave` (which do not bubble) per `Openable`, or `pointerenter`/`pointerleave`, and never clear on a child transition; a vitest that moves the pointer from a card onto its dep tag keeps the card's twins glowing.

## Progress

2026-10-24: fixed in the fork, commit `7585cac` (vitest-covered, screenshot-checked at 1500x900); live on the Pi after its next `deploy.sh`.
