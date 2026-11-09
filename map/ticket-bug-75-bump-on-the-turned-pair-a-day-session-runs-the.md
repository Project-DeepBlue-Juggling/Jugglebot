---
id: TKT-59
kind: ticket
title: 'BUG-75 BUMP on the turned pair: a day Session runs the capture and replay first, enable it at 90 deg and let a Sitting judge, or close?'
status: open
created: '2026-11-10'
updated: '2026-11-09T17:50:05Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-catch-re-aim-refused-infeasible-on-8-4-of-throws-at-the-p2
type: question
---

You leaned to Option 2 on TKT-56 and asked what the matching-record capture involves. The answer is in BUG-75 under Findings (2026-11-10): a sim capture of the turned pair at 45 and 90 deg, the 2026-11-06 and 2026-11-07 re-aim rows, and a replay through the production BUMP. All offline, about an hour, then a one-line enable and one Sitting.

The Night Shift cannot run the capture: its permission list refuses `tools/probes/`.

## Options

1. A day Session runs the capture and replay. If the turned refusals pass at roughly the Columns rate (87 to 95 %), the next night enables the BUMP for `columns_turned` on a branch held for a Sitting.
2. Enable it now at 90 deg only, where the planner is not direction-sensitive, and let the Sitting be the replay. Skips the condition you set on 2026-10-16.
3. Close BUG-75 after all: a refused second ask costs nothing measured.

Recommendation: 1. It keeps your 2026-10-16 condition, and the replay is cheap.

## What each answer unblocks

- 1: your Session runs steps 1 to 3; the night builds step 4 once the replay result is on BUG-75.
- 2: the next night builds the enable on a branch, held for a Sitting.
- 3: the next night closes BUG-75.
