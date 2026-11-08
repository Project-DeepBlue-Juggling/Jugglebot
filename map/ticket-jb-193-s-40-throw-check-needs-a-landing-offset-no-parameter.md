---
id: JB-205
kind: ticket
title: 'JB-193''s 40-Throw check needs a landing offset no parameter commands: build one, fly the ride-along without it, or close JB-193?'
status: complete
created: '2026-11-08'
updated: '2026-11-08T00:15:47Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: learner
blocked_by: []
links: []
parent: bug-the-flyback-over-corrects-a-catch-offset-by-1-2-to-1-4-times
type: question
---

You chose the 40-Throw check in JB-200. Writing its Sitting row showed it cannot be flown today: it needs commanded landing offsets of +20 and -20 mm from rest, and `skill_node` has no parameter for one, so a one-ball Throw from rest flies at about zero Tilt.

## Options

1. Build a landing-offset parameter for one-ball goals, default 0, on a branch held for the Sitting. Effort s. It touches the Throw's target on the robot path, so it needs a control walk and a rehearsal.
2. Fly without new code: compare x and y on the next one-ball goals with their ordinary Learner offsets. No new information beyond the 15:53 bag is likely, since `u` sd is only 3 mm.
3. Close JB-193 with no change: the effect is about 0.4 mm of 18 mm.

Recommendation: 3 unless JB-192 or JB-186 need the same lateral picture, then 1. A new knob on the Throw path is a lot to carry for 0.4 mm.

## What each answer unblocks

- 1: the next night builds it and writes the Sitting row.
- 2: the next night writes the Sitting row for the ride-along.
- 3: the next night closes JB-193.

## Answer

I prefer Option 1, especially as this is likely to help with other Items.
