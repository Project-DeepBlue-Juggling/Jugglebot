---
id: TKT-53
kind: ticket
title: 'IDEA-19 one ordered Sitting page: a slash command, a richer generated Runbook, or one hand-built page first, and what goes on each step?'
status: complete
created: '2026-11-09'
updated: '2026-11-08T21:35:40Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: tests-tooling
blocked_by: []
links: []
parent: idea-improve-next-sitting-runsheet
type: question
---

You asked to agree the contents of each step before anything is built. The Item's `## Proposed contents of each step` lists a first draft; edit it there or in your answer.

## Options

1. A slash command that writes one ordered page for a rig, and a check that every Item's step is on it.
2. New `terminal:` and `needs:` keys on `## Sitting`, sorted by `mapkit gen`. No model at Sitting time, but every open section is rewritten.
3. Hand-build one sheet for the next Robot Sitting, fly it, then build the command from what worked.

Recommendation: 3, then 1.

## Also wanted in your answer

- Where you read it: a page in the map app with result boxes per step, or a plain page?
- Does `optional` mean priority low, or anything you did not name for that Sitting?
- Should a step shared by two Items merge into one, with both Items recording from it?
- Do you give a time budget, with lower-priority steps dropped to fit?

## What each answer unblocks

- 1: the next night can draft the command as a held branch.
- 2: the next night can add the keys and the sort.
- 3: the night before the next Sitting builds the first sheet.

## Answer

I agree: (3), then (1). Answering your other questions:
- This should replace the 'Next Sitting' page. 
- `optional` should be anything priority 'low'.
- Yes, but each step should name the Items that it's in the runsheet for
- No, let's leave out a time budget for now.

Also, each step should have a simple 'verdict' / 'comments' section, similar to today's Next Sitting runsheet.
