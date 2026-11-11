---
id: TKT-65
kind: ticket
title: 'IDEA-22 held on night-shift/2026-11-12-IDEA-22: fly it with the 4 deg default, change the default to 3 deg, keep the velocity match on the arrival first, or build the fixed attitude per Site instead?'
status: open
created: '2026-11-12'
updated: '2026-11-11T19:00:15Z'
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

The tilt-back is built on night-shift/2026-11-12-IDEA-22 and held for a Sitting. Every auto-banked Catch of `columns_turned` at 90 deg gets a rotation about +y added, so the Cup leans toward +x. The amount is the `columns_turned_catch_tilt_back_deg` parameter, settable between Attempts. The Feed Catch stays Level, and 45 deg and every other Pattern compile exactly as today.

Two things came out offline. First, your 4 deg default is refused by the planning check at P4 limits (Leg jerk 212k against 200k); 3 deg passes thinly (margin 0.024) and 2 deg comfortably (0.228). Second, your answer named Option 2 (a fixed attitude per Site) but described an offset from the computed Catch tilt, which is Option 1's mechanism; the offset is what was built.

A side effect to know: the offset rides the existing Catch pin, and a pinned Catch aims its soft velocity match along its own tilted axis. At 4 deg that asks for about 300 mm/s of sideways Cup motion toward -x at touch-down, as a soft target.

## Options

1. **Fly it as built:** default 4 deg; at the Sitting, step down to 3 then 2 if the nomination refuses. The Sitting section says so.
2. **Change the default to 3 deg and fly:** one line and its test; the nomination passes at P4 out of the box.
3. **Keep the velocity match on the arrival first:** a day Session changes the pin so only the attitude tilts, not the soft match, then fly. Removes the sideways ask.
4. **Build Option 2 as labelled instead:** a fixed Catch attitude per Site, no auto-bank.

## What each answer unblocks

- 1 or 2: the Sitting row flies; a pass merges the branch.
- 3 or 4: a further night or day Session changes the branch before it flies.

Recommendation: 2, then 3 if the Sitting shows the Cup sliding sideways at the Catch.
