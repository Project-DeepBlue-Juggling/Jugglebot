---
id: TKT-56
kind: ticket
title: 'BUG-75: the turned pair''s refusals come from the BUMP being off for it: close it, enable the BUMP on columns_turned after a capture, or fence the asks?'
status: open
created: '2026-11-09'
updated: '2026-11-08T17:58:26Z'
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

Your reading (TKT-51, Option 2) is done and reverses the question. The turned pair does not ask twice more often: both Patterns have 27 Catches with two or more re-aims. Columns' second asks go to the BUMP (29 installed, none refused), and the turned pair has the BUMP off, so its second asks take the raw QP re-solve, and 25 of 27 are refused.

## Options

1. Close BUG-75. A refused second ask costs nothing measurable: the Hand stays on the accepted first re-aim, and the 26 turned Catches after a refused second ask were all caught at 25.1 mm median radius, against 25.0 mm with no second ask.
2. Enable the BUMP on `columns_turned`. It needs the matching-record capture you required on 2026-10-16, then a replay, then a Sitting. It changes flown motion.
3. Keep the refusals, and stop counting them as a problem by moving the check to the pre-solve fence so they do not spend a solve.

Recommendation: 1. Nothing is lost today. Choose 2 only if the turned pair ever needs the second re-aim, and BUG-79 or BUG-74 may decide that.

## What each answer unblocks

- 1: the next night closes BUG-75.
- 2: a Sitting row for the capture, then a branch.
- 3: a held branch, with the control walk the fence needs.
