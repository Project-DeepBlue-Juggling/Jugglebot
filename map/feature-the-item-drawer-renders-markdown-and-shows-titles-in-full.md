---
id: JB-51
kind: feature
title: The item drawer renders Markdown and shows titles in full
status: complete
created: '2026-10-25'
updated: '2026-10-25T06:00:00Z'
owners: []
priority: medium
blocked_by: []
links:
- https://github.com/Project-DeepBlue-Juggling/jugglebot-map/commit/7ad1872
- https://github.com/Project-DeepBlue-Juggling/jugglebot-map/commit/3ffeb2e
parent: project-map
filed_by: harrison
---

Owner, 2026-10-25: "The edit pop-out window should support markdown formatting to make them more readable. Titles should always display in full. Tbh I think the edit/detail window could benefit from a review of a Sonnet/Opus design agent"

## Progress

2026-10-25: an Opus design review wrote the spec; the owner chose raw Markdown editing with Write | Preview tabs over a rich-text editor, and a Pathway's body shown and editable under Notes. Built in the fork, commits `7ad1872` and `3ffeb2e`:

- A reading view by default. The full title wraps as a heading (click to edit), there is a compact property grid, and every text block renders as Markdown. Raw HTML shows as text and only http(s) links are live. Soft line breaks are joined, which fixes phase summaries that broke mid-sentence.
- Body editing has an Edit button with Write | Preview tabs. Ctrl/Cmd+Enter saves, and Esc leaves the editor without closing the drawer.
- A sticky bar shows Unsaved / Saving… / Saved.
- The width is `clamp(360px, 28vw, 560px)`, with page-matched colours.
- The save path (diff, conflict check, save before close or switch) is unchanged.
- Verification: vitest 181 passed, 1 skipped; fork CI green (run 40152871552).

Left over: a long Pathway name still clips in the closed Pathway dropdown, because a native select cannot wrap. The optional widen toggle was skipped.
