---
id: JB-124
kind: ticket
title: Suppress-edge flush with vel_ff 0 on every lane, legs only, report only, or folded into the next can-bridge firmware revision?
status: complete
created: '2026-11-01'
updated: '2026-11-01T00:37:10Z'
owners: []
priority: low
effort: xs
hardware: none
owner: decision
radius: local
area: firmware
blocked_by: []
links: []
parent: bug-motor-fb-stale-suppression-leaves-the-odrive-holding-its
type: question
---

JB-73 is a can-bridge firmware class: during a MOTOR_FB_STALE suppression the bridge stops transmitting setpoints and the ODrive holds its last setpoint INCLUDING its vel_ff, so the leg settles up to about 6 mm past the last transmitted command (bench leg, 2026-10-27); three related ways the Hand's ODrive state can diverge from what the bridge believes were folded in by the 2026-10-28 survey.

The body's candidate fix (one final setpoint with vel_ff = 0 at the suppress edge) needs a design look against the hand lane and the stow path, which share the gate. Night Shift 2026-11-01 drafted the options below into the Item's `## Options (2026-11-01)` section; the firmware is a gate-forbidden path and `pio` is denied at night, so nothing can be built or flashed unattended.

## Options

1. Suppress-edge flush: on the suppress edge send one final setpoint at the clamped position with vel_ff = 0, for every lane the gate covers (legs, hand, stow). One edge, one frame; needs the design look the body names.
2. Legs only: the flush on the leg lanes, hand and stow untouched until each is looked at. Smaller, leaves the Hand instance open.
3. Report only: leave the hold as it is (bounded, no wind-up, re-baselined on recovery) and fix the LEG_CMD reporting quirk so bench analysers see which commands reached the ODrive. No behaviour change.
4. Fold into the next can-bridge firmware revision with the other three instances (input-mode check, Guard against the post-slew command, RETARGET's 0x011 / 0x00C arbitration order) as one firmware unit with a bench-leg Sitting.

Recommendation: 1 if the owner wants the suppressed state to be a true hold, flown first on the bench leg with the gap driver (`encoder_msg_rate_ms=0`) and the pass criterion "leg within 0.01 rev of the last transmitted command after suppression"; otherwise 3 now and 4 when the firmware is next touched.

## What each answer unblocks

- 1 or 2: a firmware unit (owner's Session, not Night Shift: forbidden path) plus a bench-leg Sitting section on this Item.
- 3: a Night Shift unit on the analyser side only.
- 4: this Item is re-scoped as the firmware revision's checklist.

## Answer

Option 1
