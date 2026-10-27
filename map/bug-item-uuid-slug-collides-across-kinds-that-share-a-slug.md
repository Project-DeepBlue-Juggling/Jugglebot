---
id: JB-67
kind: bug
title: item_uuid(slug) collides across kinds that share a slug
status: inbox
created: '2026-10-27'
updated: '2026-10-27T09:18:16Z'
owners: []
priority: low
blocked_by: []
links:
- plans/archived/map-item-ids.md
parent: map-improvements
filed_by: claude
---

The app server derives each Item's uuid from its slug alone, so two Items of different kinds with the same slug (bug-x and feature-x) get one uuid and the store silently skips the second file. Only mapkit-side writers can create such a pair (the app refuses a duplicate slug). Fix: derive the uuid from the stem (kind + slug), with a one-time redirect for existing deep links. Residue of plans/archived/map-item-ids.md section 7.
