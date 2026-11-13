---
id: TKT-70
kind: ticket
title: 'BUG-91 negative re-aim margin at nomination: refuse, flag in capitals, refuse below a floor, or leave it?'
status: open
created: '2026-11-14'
updated: '2026-11-13T17:50:58Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-nominate-accepts-a-setting-whose-own-reply-shows-a-negative
type: question
---

At the Sitting of 2026-11-13, `/skills/nominate` accepted columns_turned 90 deg, Apex 0.90, tilt-back 3 deg with `success: True` while its reply read -0.080 with the Catch re-aim at its limit. All five Attempts ended at the first Catch after the Feed (LIMIT_JERK x3, NO_ADMISSIBLE_COMMAND x2).

## Options

1. **Refuse:** `success: False` when the re-aim-limit figure is below zero or a Segment there is refused.
2. **Flag in capitals:** `success` stays True, the reply ends `WARNING: RE-AIM MARGIN NEGATIVE`, shown red in the GUI.
3. **Refuse below a floor, flag above it:** e.g. refuse under -0.05.
4. **Leave it:** the reply already warns.

## What each answer unblocks

- 1, 2 or 3: the next night builds it (rated xs, no hardware) and the gate can merge it.
- 4: BUG-91 closes.

Recommendation: 2. The re-aim figure is walked on the identity command (BUG-87: it read -0.049 when the live Segment refused at -0.208), so it is neither conservative nor exact; refusing on it would block rows you fly to learn. A refusal mid-Attempt ends cleanly, so nothing physical rides on this.
