---
kind: bug
title: Keyboard shortcuts dialog lists shortcuts that no longer exist
status: inbox
created: '2026-10-24'
updated: '2026-10-24T08:10:27Z'
owners: []
priority: medium
blocked_by: []
links:
- logbook/2026-10-24-project-map-p4-the-two-pages.md
- https://github.com/Project-DeepBlue-Juggling/jugglebot-map
parent: project-map
filed_by: harrison
---

Owner reaction round, 2026-10-24: the Keyboard shortcuts dialog (sidebar, `?`) is wrong - it still describes the origin app's keys. Only `n` (new item), `Esc` (close the drawer) and whatever the pages actually bind exist now. Fix: rewrite `ui/KeyboardShortcutsDialog.tsx` from the bindings that exist, with a test that every listed key has a handler.
