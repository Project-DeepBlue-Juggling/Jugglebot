---
id: FEAT-52
kind: feature
title: Generate logbook/INDEX.md from entry frontmatter with a capped summary
status: complete
created: '2026-11-14'
updated: '2026-11-14T02:53:54Z'
owners: []
priority: medium
effort: m
hardware: none
owner: none
radius: cross-cutting
area: docs
blocked_by: []
links:
- logbook/2026-11-14-logbook-index-generated.md
filed_by: claude
---

Generate `logbook/INDEX.md` from entry frontmatter, as the three plan indexes already are, in place of an agent appending a row by hand. From the 2026-11-13 maintainer review.

## Findings

- The file is 927 KB in 691 lines. Rows reach 22,617 characters; the median title cell is 465 characters and the 90th percentile 2,632.
- It was touched by 474 commits in three months, so it is the file most likely to conflict between parallel Sessions, the Night Shift and a debrief.
- Recent rows are shorter (about 370 characters on average in November against 3,040 in August), so the habit improved while the old rows remained.
- No test checks the index against the entries; `tests/sim/test_logbook_front_matter.py` skips it.

## Wanted

- A capped one-line `summary:` in each entry's frontmatter, the index built by `python -m mapkit gen`, and a currency test like `tests/sim/test_plans_index.py`.
- Backfill: old rows are cut to their bold title; the long text already lives in the entry.

## Open questions

- `/logbook --search` and `sim/analysis/logbook_search.py` read entries, not the index; confirm nothing else depends on the long rows.
- Whether the index should be committed at all, or built with the docs site.

## Progress

- 2026-11-14: Generator, currency test and every consumer instruction landed; index 929 KB to 213 KB
