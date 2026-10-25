---
id: JB-55
kind: feature
title: 'A Done view: complete Pathways and closed items, with their dates'
status: inbox
created: '2026-10-25'
updated: '2026-10-25T10:00:00Z'
owners:
- harrison
priority: medium
blocked_by: []
links:
- https://github.com/Project-DeepBlue-Juggling/jugglebot-map
parent: map-improvements
filed_by: claude
---

## Problem

Since 2026-10-25 neither the Front page nor Now / Next / Later draws a complete Pathway, and closed items never showed. Finished work is now only findable by file name.

## Proposal

A "Done" page in the sidebar:
- complete Pathways, newest first, each with its completion date (the "met <date>" in the Destination, else the last Progress line, else `updated`) and its archived plans;
- closed bugs, features, ideas and tickets, grouped by Pathway, filterable by kind and month;
- a click opens the usual drawer.

## Done when

- Every complete Pathway and closed item can be reached in two clicks from the sidebar.
- Dropped items appear only behind an explicit "show dropped" toggle.
- A frontend test pins the date rule and the grouping.
