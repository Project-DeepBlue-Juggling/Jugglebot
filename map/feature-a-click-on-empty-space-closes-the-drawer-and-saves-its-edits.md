---
kind: feature
title: A click on empty space closes the drawer and saves its edits
status: complete
created: '2026-10-25'
updated: '2026-10-25T00:55:00Z'
owners: []
priority: medium
blocked_by: []
links:
- https://github.com/Project-DeepBlue-Juggling/jugglebot-map/commit/c1e98d9
parent: project-map
filed_by: harrison
---

Owner, 2026-10-25, during the Now / Next / Later prototype round: "clicking off the edit panel into empty space should close it, saving any changes that have been made."

## Progress

2026-10-25: fixed in the fork, commit `c1e98d9`. A pointer-down on empty space outside the drawer closes it; every close (button, Esc, click-away) and every switch to another item saves unsaved edits first, and a failed or conflicting save keeps the drawer open. Four vitest cases cover click-away, the close button, controls not dismissing, and the conflict. Live on the Pi after its next `deploy.sh`.
