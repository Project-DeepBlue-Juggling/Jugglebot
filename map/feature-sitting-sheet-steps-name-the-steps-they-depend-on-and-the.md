---
id: FEAT-62
kind: feature
title: Sitting sheet Steps name the Steps they depend on, and the Next Sitting page marks the dependants of a Problem Step
status: inbox
created: '2026-11-22'
updated: '2026-11-21T15:50:12Z'
owners: []
priority: low
effort: m
hardware: none
owner: none
radius: subsystem
area: map-app
blocked_by: []
links:
- bug-next-sitting-progress-bar-should-have-more-colours
filed_by: night-shift
night_skip: too-large
night_skip_on: '2026-11-23'
night_skip_note: Spans the sheet builder, the mapkit sheet loader and the Next Sitting page (effort m); a day Session or a night slice once the field is named.
---

Split from BUG-127 by the Night Shift of 2026-11-22, on the owner's TKT-90 answer.

BUG-127 asked that the Next Sitting progress rail also mark the Steps that depend on a Step recorded as a Problem. The rail part is built (one strip per Step, coloured by its recorded state). The dependency part is not, because a Sitting sheet has no field that names another Step.

The owner's answer (TKT-90): "I was hoping this could be something that the Claude that builds the sitting-sheet could figure out. I'd like to start trialling this, and I can give feedback and refine this figuring-out as we go."

## Wanted

- The `/sitting-sheet` builder (`.claude/commands/sitting-sheet.md`) works out, for each Step, which earlier Steps it needs (same rig state, a calibration it relies on, a build or flash it runs on), and writes them into the sheet as a per-Step list of Step numbers.
- The sheet loader in `mapkit` accepts and validates that field (earlier Steps only, numbers that exist), and the map app serves it.
- The Next Sitting page marks every Step that depends, directly or through another Step, on a Step recorded as a Problem.
- The owner trials it on the next sheets and refines the builder's rules from feedback.

## Open

- The field's name and whether it lists direct dependencies only (the app follows the chain).
- How the page marks a dependent Step (a hatched strip, an icon on the card, or both).
