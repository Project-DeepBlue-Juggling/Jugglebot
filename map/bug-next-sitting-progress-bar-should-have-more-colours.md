---
id: BUG-127
kind: bug
title: '''Next Sitting'' Progress Bar should have more colours'
status: inbox
created: '2026-11-20'
updated: '2026-11-20T15:50:11Z'
owners: []
priority: low
effort: m
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: map-app
blocked_by: []
links: []
filed_by: harrison
parent: map-improvements
---

Currently it turns green for complete Steps, but remains grey for anything open (non-collapsed, even if complete) or problematic. Problematic Steps should get a red strip, and the collapsed state of a complete Step shouldn't affect its colour.

Recall that Problematic Steps aren't necessarily blockers; there are often later Steps that don't depend on any others. In fact, it would be awesome to have these auto-keyed together, so that one failed/problematic Step would (by default, but in an overrideable manner) mark dependent Steps Problematic as well (perhaps suggesting to collapse them?)
