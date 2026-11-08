---
id: BUG-59
kind: bug
title: Editing long Descriptions is buggy
status: complete
created: '2026-11-02'
updated: '2026-11-07T18:11:28Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: local
diagnosis: unknown
area: map-app
blocked_by: []
links:
- logbook/2026-11-08-drawer-description-editor-scroll-reset.md
filed_by: harrison
---

The whole edit panel scrolls to the top as soon as a character is entered into the editor, making long Descriptions impossible to edit while seeing what you're typing

Filed from the Robot tab at 2026-11-02T20:47:05+11:00.

## Review (2026-11-07)

Open to the Night Shift through the app's night clone. A headless test may not reproduce a scroll bug, so a fix here needs the owner's eyes after the deploy; the report row should say so.
