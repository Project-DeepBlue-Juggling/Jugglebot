---
id: TKT-67
kind: ticket
title: 'IDEA-22 held on night-shift/2026-11-12-IDEA-22: 3 deg about +x is refused at P4 limits; keep 3 deg and fly at Apex 0.90, change the default to 1.9 deg, step down at the Sitting, or raise the Leg jerk limit?'
status: complete
created: '2026-11-13'
updated: '2026-11-12T22:12:18Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: idea-turned-columns-at-90-deg-tilt-the-platform-back-x-at-the
type: question
---

Your TKT-65 answer is built on night-shift/2026-11-12-IDEA-22: the Catch tilt-back is now a rotation about +x, so the Cup leans toward -y, and the default is 3 deg. The Feed Catch stays Level; 45 deg and every other Pattern are unchanged.

About +x, 3 deg is refused offline at P4 limits (Leg 1 jerk 227k against 200k, margin -0.262). 2 deg is refused too (-0.013); 1.9 deg passes (0.011). 3 deg passes at Apex 0.90 (0.027). The +y figures you chose from were 0.024 at 3 deg.

Why it costs more: the Catch the jerk binds on arrives moving +y, so its own bank is already about 1.9 deg about +x. The new offset adds in line with it (4.9 deg in total); the old +y offset was perpendicular (3.5 deg).

Side effect: at 3 deg the pinned Catch asks for about 227 mm/s of sideways Cup motion toward +y at touch-down, as a soft target.

## Options

1. **Keep 3 deg, fly at Apex 0.90:** the default stays; the Sitting nominates at Apex 0.90 instead of 0.95.
2. **Default 1.9 deg, fly at Apex 0.95:** one line and its test; the largest tilt-back that passes at the flown Apex.
3. **Keep 3 deg and step down at the Sitting:** the operator sets 1.9 deg only if the nomination refuses (it will at Apex 0.95).
4. **Raise the Leg jerk limit for this Pattern:** a day Session's call; not built at night.

## What each answer unblocks

- 1 or 3: the Sitting row flies as written; a pass merges the branch.
- 2: the next night changes the default, re-gates and holds for the Sitting.
- 4: a day Session.

Recommendation: 1. It keeps your 3 deg, the tilt you judged the ball needs, and a 50 mm lower Apex sits inside the admissible box at 90 deg (0.875 to 0.975).

## Answer

Alright, let's go with Option 1.
