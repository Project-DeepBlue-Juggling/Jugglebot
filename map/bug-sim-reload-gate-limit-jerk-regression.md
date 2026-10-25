---
kind: bug
title: Sim `--reload` gate LIMIT_JERK regression bisected to 08ec3fe6 (2026-09-30)
status: inbox
created: '2026-10-24'
updated: '2026-10-24T13:05:00Z'
owners: []
priority: medium
blocked_by: []
links:
- plans/archived/columns-empty-cup-two-feed.md
filed_by: claude
---

The sim `--reload` gate was good at 8364808c (5/5 makes, dwell 0.30 s); after 08ec3fe6 it gets 1 make and `LIMIT_JERK` in the continuation throws. Candidate fix: set lateral authorities in the sim gate (plan section 6). Owner decision pending.
