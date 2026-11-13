---
id: BUG-91
kind: bug
title: nominate accepts a setting whose own reply shows a negative margin with the Catch re-aim at its limit
status: inbox
created: '2026-11-13'
updated: '2026-11-13T17:50:21Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
diagnosis: confirmed
area: skill-stack
blocked_by: []
links:
- map/idea-turned-columns-at-90-deg-tilt-the-platform-back-x-at-the.md
- map/bug-nomination-s-re-aim-figure-read-0-049-while-the-live-feed.md
filed_by: claude
---

`/skills/nominate` accepted `columns_turned` at 90 deg, Apex 0.90, tilt-back 3 deg with `success: True` although its own reply printed a negative margin at the Catch re-aim limit. All five Attempts at the Sitting of 2026-11-13 then ended at the first Catch after the Feed.

## Numbers

- Reply: worst margin +0.027 at the nominal Sites (CATCH Ball 1 at P2@90) and -0.080 with the Catch re-aim at its limit (CATCH Ball 2 at P1@90), with the warning "An Attempt may end mid-Pattern on an admissibility refusal".
- Flown: LIMIT_JERK x3 at 204 to 217k mm/s^3 (200k limit, leg 2, +0.050 s into the re-gated range) and NO_ADMISSIBLE_COMMAND x2 at margins -0.301 and -0.288; best 2 Throws.
- One 25 ms knot of Ball 2 arrival shifted the Ball 1 Catch margin from +0.031 to -0.301 at the same command, so +0.027 sits inside the splice-state spread.
- The nominate sees the identity command (0, 0), not the learner command (-35.6, +0.7) mm at Apex 0.827 that a warmed plant gives (the BUG-87 class).

## Wanted

- Decide whether `/skills/nominate` refuses, or flags in capitals, a setting whose re-aim-limit margin is negative. A policy choice for the owner; no limit change is proposed.

## Options (2026-11-14)

1. **Refuse:** `success: False` when the re-aim-limit figure is below zero or a Segment there is refused; the operator cannot arm it.
2. **Flag in capitals:** `success` stays True, the reply ends `WARNING: RE-AIM MARGIN NEGATIVE` and the GUI shows it in red.
3. **Refuse below a floor, flag above it:** e.g. refuse under -0.05, flag between -0.05 and 0.
4. **Leave it:** the reply already says an Attempt may end mid-Pattern.

Recommendation: 2. The re-aim figure is walked on the identity command (the BUG-87 class: it read -0.049 when the live Segment refused at -0.208), so it is neither conservative nor exact, and a refusal on it would block rows the operator flies to learn. An admissibility refusal ends an Attempt cleanly, so nothing physical rides on the choice. Rated xs, no hardware: the night can build 1 or 2.
