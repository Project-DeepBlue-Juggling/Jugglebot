---
id: IDEA-19
kind: idea
title: Improve the Next Sitting Runbook
status: backlog
created: '2026-11-08'
updated: '2026-11-09T18:16:35Z'
owners: []
priority: medium
effort: m
hardware: none
owner: decision
radius: subsystem
area: tests-tooling
blocked_by: []
links: []
filed_by: harrison
---

## Request

Currently, having all Items and their associated steps is nice, but not nearly as convenient as it could be. 

I would much prefer to have a single cohesive Runbook that integrates all the Items' steps, in logical sequence.

This will likely reward the construction of a custom Claude slash-command that I invoke just before beginning a Sitting. I should be able to say what kind of Sitting it will be (robot [which includes BB by default], bench leg, bench hand) and the command will have Claude order all the steps for the Items that need Sitting time.

It would be nice to have lower-priority Items' steps indicated as 'optional', with a link to the Item they're for, so I can evaluate whether I want to run the step or not.

To make my goal clear: when I am ready for a Sitting, I want a single Runbook that I can step through, instructing me what commands to run, when a new ROS2 session should be initiated, etc (please come back to me with your suggestions for what to include on each step and we can agree on this before building anything).

Each step must be clear, and anything that must happen before a given step must be placed before it in the sheet. It should NOT be assumed that I will read the whole Runbook before beginning the Sitting.

The buildout of this idea will reward our discussion of its intricacies. Please come back to me with any questions you have before we build this.

Filed from the Robot tab at 2026-11-08T21:48:40+11:00.

## Feasibility (2026-11-09)

- What it takes: a page that merges every Item's `## Sitting` steps for one rig into one order. Today `docs/runbook.md` groups rows by setup state and keeps each Item's steps together.
- Effort m, hardware none. Nothing on the robot path changes.
- Risk: a merged order that puts a step before something it needs. Every section's `do:` is free text, so no tool can read those dependencies today.

## Options (2026-11-09)

1. A slash command, `/sitting-sheet <robot|bench-leg|bench-hand> [minutes]`: Claude reads the `## Sitting` sections and held branches and writes one ordered page. A check confirms every Item's `do:` step appears on it.
2. Deterministic: add `terminal:` and `needs:` keys to `## Sitting`, and `mapkit gen` sorts on them. No model at Sitting time, but every open section is rewritten and shared steps never merge.
3. Hand-build one sheet for the next Robot Sitting, fly it once, then build the command from what worked.

Recommendation: 3, then 1. It settles the step format on a real Sitting before anything is built, as you asked.

## Proposed contents of each step (for your edit)

- Step number, and the state it starts from (Idle, Armed, Level, BB-fed).
- Which terminal, and the exact command or GUI action. A new ROS2 launch is a step of its own.
- What you should see, and what makes you stop.
- What to record, and the Item it serves (linked). Lower-priority Items read `optional`.
- Anything a step needs comes earlier: builds, flashes, parameter sets, and the restore afterwards.

## Decisions (TKT-53, 2026-11-08)

- Order: Option 3 first (one hand-built sheet for a Robot Sitting, flown once), then Option 1 (the `/sitting-sheet` command) built from what worked.
- The sheet replaces the app's Next Sitting page in the end.
- `optional` = an Item of priority low.
- A step shared by two Items is one step, and it names every Item it serves.
- No time budget for now.
- Every step gets a verdict box and a comments box, as today's Next Sitting page has per Item.

## Next

- The first sheet is built the night before the next Robot Sitting, from the Runbook as it stands then. The Legs are being re-jointed (TKT-52), so a sheet built now would be stale.
- To get it: put the planned Sitting date in a Progress note here (or tell a Session), and the Night Shift before that date builds the sheet on a branch.

## Progress

- 2026-11-09: Night Shift 2026-11-09: options and a draft step format written; awaiting the owner's answer
- 2026-11-10: Night Shift 2026-11-10: TKT-53 decisions recorded; the first sheet is built the night before the next Robot Sitting, once its date is noted here
