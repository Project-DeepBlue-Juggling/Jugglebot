---
id: BUG-92
kind: bug
title: static Ball Butler frame markers are tracked as human_throw
status: inbox
created: '2026-11-13'
updated: '2026-11-13T17:50:22Z'
owners: []
priority: medium
effort: s
hardware: robot
owner: decision
radius: subsystem
diagnosis: confirmed
area: tracking
blocked_by: []
links:
- map/idea-jp-17-three-ball-cascade-with-the-owner-as-the-second-hand.md
filed_by: claude
---

Static Ball Butler frame markers are tracked as `human_throw` tracks. The operator saw it at the Sitting of 2026-11-13 (step 29, "a few predicted throws were the Ball Butler frame markers").

## Numbers (launch 3 bag, 13:10:40 to 13:12:50)

- 12 of 124 live `/balls` human_throw tracks start within 40 mm of a static marker (about 6.4 per minute).
- 26 of the 30 offline bench phantoms start on the same static cells (x -730 to -988 mm, z 1665 to 1861).
- Ball Butler markers `Ball Butler - 1..7` sit at fixed positions (sd 0.1 to 0.4 mm, present in 99 % of frames), plus four unlabelled static markers. `excluded_label_prefixes` cannot name `Ball Butler`, because QTM labels the flying ball `Ball Butler - 1`.
- A tolerance on the parabola would not help: a static marker never rises.

## Proposed rule (owner decision)

Exclude a human_throw candidate whose first sample lies within about 40 mm of a static cell: a 10 mm cell that an eligible marker occupied in more than 90 % of the last 30 to 120 s of frames. It needs no label and no zone list.

- On the launch 3 slice it removes 26 of 30 bench phantoms (13.8 to 1.8 per minute), 12 of 124 live tracks, and 0 of 114 true human Throws.
- Not checked: Ball Butler own announced Throws (they use `/throw_announcements`), a ball held still in a hand for over a second, the 4 remaining non-static phantoms, other bags.

## Options (2026-11-14)

1. **Build the proposed rule, on by default:** a human_throw candidate whose first sample is within 40 mm of a static 10 mm cell (occupied over 90 % of the last 30 to 120 s) is dropped; a parameter turns it off. Tests on synthetic markers; a Sitting checks the phantom rate and that no real Throw is lost.
2. **The same rule, off by default:** a Sitting turns it on for an A/B with `detect_human_throws`.
3. **A zone in config:** drop starts inside the Ball Butler frame box (x -730 to -988, z 1665 to 1861 mm). Simpler; breaks when Ball Butler moves.
4. **Leave it:** `detect_human_throws` is off by default, so phantoms only appear in human-Throw windows.

Recommendation: 1. On the launch 3 slice it removed 26 of 30 bench phantoms and 0 of 114 true Throws, and it needs no label or zone. FEAT-50 builds a rolling static-cell filter tonight that this could share.
