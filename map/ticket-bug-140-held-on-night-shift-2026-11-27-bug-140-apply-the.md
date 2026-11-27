---
id: TKT-99
kind: ticket
title: 'BUG-140 held on night-shift/2026-11-27-BUG-140: apply the audit''s 2 fixes before the Sitting, fly it as is, or drop it?'
status: complete
created: '2026-11-27'
updated: '2026-11-27T00:16:15Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-fresh-rest-rebases-outside-the-band-are-uncapped-up-to-1-0-s
type: question
---

BUG-140 is built on night-shift/2026-11-27-BUG-140 as you asked in TKT-96 (option 2): a REST rebased from rest outside the band now starts on the Knot grid, 0 to 25 ms later than before, with no cap, and the install line reports the slide in whole Knots. It is held for a Sitting either way, but the audit left two behaviour-affecting findings open.

## Open findings

1. The grid rounding guard is absolute (1e-9 Knot). After about 3 days of box uptime the clock's rounding error can exceed it, and a band REST would then start one Knot later than computed, possibly past its 8-Knot cap. The fix passes the band REST's whole-Knot slide straight through.
2. A ROS test patches the slide helper to return a fractional Knot count; it is safe today only because every trial seeds at rest. The fix makes the patch integer-safe and adds a sibling test without the patch.

## Options

1. Apply both fixes: the next night makes them on the branch, re-audits, and leaves it held for the Sitting.
2. Fly as is: finding 1 cannot bite on a box rebooted within 3 days (the Jetson had been up 27 days on 2026-11-27, so it would need a reboot first); a day Session fixes it before any merge.
3. Drop the branch.

## What each answer unblocks

1: the next night applies the fixes, then the Sitting section stands as written. 2: the Sitting may fly the branch now. 3: the branch is left for you to delete and BUG-140 goes back to open.

Recommendation: 1, because finding 1 is a silent one-Knot shift on the band REST that only appears on a long-running box.

## Answer

Agreed. Go for Option 1
