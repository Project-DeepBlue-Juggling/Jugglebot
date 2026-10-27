---
title: "map-writes: prose names other Items by plain JB-n, so the drawer links them"
type: refactor
date: 2026-10-27
status: resolved
phase: "map-improvements"
files_changed:
  - docs/agents/map-writes.md
subsystem:
  - project-map
tags:
  - project-map
---

## Summary

The owner asked whether `JB-n` cross-links work on the map site and why none were visible. They do: the map app's item drawer turns every known `JB-n` in an Item's text into a link that opens that Item (`ui/MarkdownImpl.tsx`, `remarkItemIds`, in the `jugglebot-map` fork). On 2026-10-27 the live map held only one real cross-mention, the JB-65 in JB-58's last Progress line. IDs date from 2026-10-26, and structured references (`parent`, `blocked_by`, `links`) are stems shown in their own drawer sections. The owner chose the map site only; GitHub autolinks and mkdocs linking were not wanted.

`docs/agents/map-writes.md` § Rules now makes the habit explicit. It widens the old one-clause Progress-line convention into its own bullet.
- Text the drawer renders (an Item's body and Progress notes, `item new --body`, plan and phase descriptions) names other Items by plain `JB-n`, optionally followed by a title.
- The ID is written bare. The linkifier skips code spans and Markdown links, and a range such as `JB-68..70` links only its first ID.

## Verification

Docs only. No test reads `docs/agents/map-writes.md`: `tests/sim/test_docs_links.py` excludes `docs/agents/` (`_EXCLUDED_PREFIXES`), and the Avoid-word scan covers plans, logbook and `map/` only. On 2026-10-27, with this entry and its INDEX row present, `pytest -q tests/sim/test_logbook_front_matter.py tests/sim/test_glossary_avoid_words.py tests/sim/test_logbook_search.py`: 37 passed. The linkifier's skip rules were read from `remarkItemIds` in the fork at bb1fc29.
