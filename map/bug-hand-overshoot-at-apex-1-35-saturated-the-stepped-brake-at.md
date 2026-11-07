---
id: JB-147
kind: bug
title: Hand overshoot at Apex 1.35 saturated the stepped brake at 60 A and drove the Hand into the top stop (SPINOUT)
status: in_progress
created: '2026-11-02'
updated: '2026-11-07T06:22:26Z'
owners:
- jugglebot-3d
priority: high
effort: m
hardware: robot
owner: decision
radius: subsystem
diagnosis: hypothesis
area: hand
blocked_by: []
links:
- logbook/2026-11-04-sitting-debrief.md
filed_by: jugglebot-3d
---

At the first Apex 1.35 Columns goal (2026-11-02 17:00 Sitting, throw 10), one launch overshot. The Hand reached 168 rev/s against a planned ~151, and the release step's brake saturated at the 60 A limit (-60.4 A). The Hand overran the planned top (10.16 rev) and the 10.501 clip to 10.83 rev and hit the top stop; the encoder slipped (SPINOUT_DETECTED). The Ball went to 1.65 m.

## Evidence

Across the ~190 launches from Apex 1.05 to 1.35 that Sitting, the Hand topped out at 10.16-10.23 rev with brake current -10 to -36 A; only this one exceeded them. The launch came straight off a Catch that dived to the stroke floor (0.03 rev). There was no stream gap, the Platform z was fixed at 170 mm, and the cause of the overshoot is not resolved by the 100 Hz `/hand_telemetry`.

## Why it matters

The stepped brake runs near the constant-deceleration ideal, so an overspeed has only the current headroom and about 0.34 rev of stroke above the planned top. The same margin shrinks as the Apex rises.

## Options

Raise `hand_curr_limit_a` to the owner's 70 A (configure pushes it over the GUI value); scale a top margin with release speed; capture the Hand at a higher rate to find the overshoot's cause; a firmware overspeed guard. Related: JB-144, JB-145, the JB-114 bumper.

## Diagnosis (2026-11-03)

The title is the 2026-11-02 hypothesis and is withdrawn (no mapkit command changes a title; read this section as the current framing). Flown 2026-11-03 at 70/80 A configured (see the 60 A fact below) with `hand_capture` on (`temp/hand_capture/hand_capture_20261103_123004.csv`, 100 Hz per kind, 0 dropped rows). Launch log `~/.ros/log/2026-11-03-12-30-00-967281-jetson-3807496/launch.log`. 10 SPINOUT events plus 1 MAX_DEVIATION (filed separately) were collapsed from 98 SPINOUT lines.

**The hypothesis "launch overspeed saturates the brake, the Hand overshoots into the top stop, SPINOUT" is WITHDRAWN.** In 9 of 10 SPINOUTs the braking torque vanished 10-20 ms into the brake, with the Hand 1.9-2.5 rev under the clip (8.0-8.6 rev, 45-95 rev/s), 0.3-0.6 rev behind its command, with last fresh iq 13-54 A against a 60 A clamp. Deceleration fell to about 500 rev/s^2, the unpowered gravity-plus-friction value. The stop hits are the coast after the disarm.

| # | log line | goal | TL pos / vel (rev, rev/s) | coast peak (rev) | last fresh iq (A) | owner heard |
|---|---|---|---|---|---|---|
| 1 | 1019 | 0.95 | 8.11 / 51 | 10.716, no contact | -23.0 | none |
| 2 | 2277 | 0.95 | 8.53 / 57 | 10.783, light touch | -38.9 | none |
| 4 | 5380 | 1.35 a1 | 8.39 / 78 | 10.855, ~50 rev/s hit | -25.9 | not reported |
| 5 | 6358 | 1.35 a3 | 8.54 / 53 | 10.754, soft touch | -13.4 | silent |
| 6 | 6906 | 1.45 a2 | 8.1-9.8 / 95-74 | 10.816, +74 -> -44 hard hit | -54.0 | audible |
| 7 | 7374 | 1.45 a3 | 8.46 / 47 | 10.462, under clip | -16.8 | silent |
| 8 | 7616 | 1.50 a1 | 8.50 / 45 | 10.389, under clip | -18.4 | silent |
| 9 | 7853 | 1.50 a2 | 8.03 / 76 | 10.749, collision | -27.4 | audible |
| 10 | 8106 | 1.50 a3 | 8.59 / 58 | 10.780, ~15-20 rev/s contact | -17.7 | audible |
| 11 | 10110 | high-low 1.00 | brake at clamp: 141 -> 60 rev/s in 30 ms | 10.64 at 60 rev/s, stop ~10.74 | -47 to -61.3 (sp -59.94) | audible |

Event 3 (MAX_DEVIATION, L2528) is not a Hand excursion; it has its own Item. Events 2, 6 and 11 show fresh iq of -39 to -61 A with only 1050-1700 rev/s^2 (25-45 % of nominal) before the disarm.

## Reading: commutation-type torque loss

- At the last fresh iq before each disarm a correctly commutated motor has electrical power below zero in 10 of 10 events (c = 1 copper-loss scaling; 8 of 10 at c = 1.5).
- The ODrive trip needs it above +10 W while commanding braking, so the stator current was not opposing the back-EMF: an electrical-angle error. The copper-loss false positive is withdrawn for 8 of the 10; the SPINOUTs look genuine.
- Candidates, none tested: encoder or magnet slip under the release step's angular deceleration, or an offset error.
- Event 11 is the exception in form: armed at the clamp with 25-60 % of nominal torque, it struck the stop while still armed.

## Fault rate against brake hardness

| block | launches | SPINOUTs |
|---|---|---|
| Columns 0.95 | 170 | 2 |
| Columns 1.35 | 48 | 2 |
| Columns 1.45 | 28 | 2 |
| Columns 1.50 | 11 | 3 |
| high-low 1.00/0.90 | 66 | 1 |
| high-low 1.05/0.95 | 51 | 0 |
| turned 0.95 | 120 | 0 |

In every block the faulting launches sit at or above the p90 of that block's measured peak brake deceleration (at 0.95: faults 4178 and 5629 rev/s^2 against a normal median 2882, p90 3321). The rate rises with the brake.

## The 60 A fact and the limit decision

- The Hand iq setpoint never exceeded 59.94 A all Sitting (iq_measured min -62.2, max +55.3).
- The owner's correction: the ODrive soft limit was 60 A (hard 80 A), so the config's 70 A never reached the drive; it is not a plumbing bug.
- The owner has since set 70/80 A in the GUI, so the next Sitting brakes with up to 70 A, and the backup JSON (70/80) is now right.
- Concern recorded, not re-litigated: the torque loss happened at 13-54 A, below the clamp, so the limit does not explain it, and a 70 A brake is harder.
- Decision (owner, 2026-11-03): keep C-HAND-6 K 4.75e-5, no Apex cap, SPINOUT thresholds unchanged.

## The "3-4 cm extra at the top" and what never fired

- The C-HAND-6 margin IS the owner's 3-4 cm: margin(v) = 4.75e-5 v^2 gives 1.186 rev = 38.6 mm at 1.35 (158.0 rev/s), 41.5 mm at 1.45, 42.9 mm at 1.50; commanded tops 9.10, 9.02, 9.01 rev; measured armed tops at most 9.10 rev, leaving 47-50 mm of stroke below the 10.501 clip.
- Every armed launch topped at or under its command; every clip crossing (10.64-10.86 rev) came after a disarm.
- FW 36 stall guard (JB-146): never fired (`diag_flags` 2 in all 31,389 rows).
- Torque feedforward clamp: not reached (`cmd_tor_ff` -0.184..+0.138 N m, 0 rows at the 0.234 clamp).

## Sitting result (2026-11-03)

- verdict: fail
- notes (the owner's box, split into bullets by the debrief for the 500-character readability cap; wording unchanged):
  - At 1.35, the hand isn't using the full stroke. It looks like there's at least 3-4cm extra at the top of the stroke. One attempt ended with LANE_REFUSED. The last attempt ended with SPINOUT_DETECTED, despite no obvious cause for this to my eye (no visible collision with the end stop, no sudden sounds).
  - At 1.45, the first attempt ended with a messy throw being uncatchable. Second ended with SPINOUT_DETECTED, for which it sounded like the hand hit the end-stop (or something?). Third attempt ended the same way, but with no audible collision between the hand and the end-stop.
  - At 1.50, the first attempt had a non-audible SPINOUT_DETECTED, while the second attempt ended with the same error but I could hear the collision between the hand and the end-stop. Third attempt ended the same as the second (audbily).

## Sitting result (2026-11-07 16:19)

- verdict: pass
- notes: Several attempts all either succeeded or ended due to messy throws or mid-air collisions being unrecoverable.

## SPINOUT at the 2026-11-07 Sitting

**Do not close JB-147 on the 2026-11-07 pass (owner, 2026-11-07): the root cause comes first.** The pass covers only the four Apex 1.10 Attempts, and the same fault fired later in that Sitting. Launch log `~/.ros/log/2026-11-07-16-15-21-645789-jetson-2503132/launch.log`, capture `temp/hand_capture/hand_capture_20261107_161525.csv`, 100 Hz.

| goal (log line) | catches | peak brake iq (A) | peak brake decel (rev/s^2) | SPINOUT |
|---|---|---|---|---|
| L600 | 9/9 | -24.7 | 4045 | 0 |
| L963 | 12/12 | -23.6 | 3853 | 0 |
| L1360 | 20/20 | -28.2 | 3891 | 0 |
| L1970 | 19/20 | -29.0 | 4147 | 0 |

**L8157, 16:31:20, `columns_turned` Apex 0.95, the same torque-loss signature:**

- At -36 A, far under the 70 A limit, deceleration fell from about 2000 to 83 rev/s^2 in 20 ms, 30 to 50 ms after peak speed.
- The Hand then hit the top stop: +59 to -49 rev/s in 10 ms at 10.74 rev, against a commanded top of 9.50 rev.
- It was the fastest launch (137 rev/s, commanded 134, block median 123) and the hardest brake (5416 rev/s^2) of all 397. No surviving launch reached 4300.

**What the row did not settle:**

- **Does the fault rate move at 70/80 A?** Not shown. No brake in the four Attempts passed 29 A. Zero SPINOUTs in 69 launches was 43 % likely at the Apex 0.95 rate of 2026-11-03 and 5 % at its Apex 1.35 rate.
- **Sitting-wide rate:** 1 in 397 against 3 in 407 for the comparable blocks of 2026-11-03 at 60 A, too few to separate (Fisher p = 0.62).
- **Was the 70 A limit live?** Yes, setpoint -69.93 A at L963, but that was the JB-202 stall at zero speed, not a brake.

**Next:** the encoder or magnet slip and offset candidates in the Diagnosis above are still untested. The flown row is removed from this Item so the Runbook does not ask to fly it again; the next row waits until a root-cause hypothesis says what to measure.

## Progress

- 2026-11-02: Landed 8ffeba66: 70 A code limit, stream STOP 4500, C-HAND-6 speed-scaled top margin (K 4.75e-5), plus the opt-in hand_capture:=true for diagnosing the overshoot. Cause of the overshoot still open; fly with the capture on.
- 2026-11-03: Failed at the Sitting of 2026-11-03: 7 of 9 Apex 1.35-1.50 attempts ended SPINOUT_DETECTED (L5380, 6358, 6906, 7374, 7616, 7853, 8106), tops to 10.855 rev vs clip 10.501. Capture shows torque lost 10-20 ms into the brake at 13-54 A, not saturation: overshoot hypothesis withdrawn (detail below).
- 2026-11-04: 2026-11-03 Sitting (22:29, P4 ramp): REFUTED for the three firmware-STOP ends (goals 3, 9, 15): the Hand braked on its profile at -23..-24 A, no SPINOUT_DETECTED all Sitting, peak |iq_set| 54.1 A vs the 70 A clamp. Withdrawn for that class; the Apex 1.35 overshoot it was filed for was not exercised.
- 2026-11-05: 2026-11-05 L2596: SPINOUT after a C1-stop brake: decel 4826 -> ~500 rev/s^2 within 10 ms of the brake peak, 25 ms into the brake, at 4.4 rev / 20 rev/s, iq -41 A (limit 70, not saturated). Same torque-loss signature as 11-03, far from the clip; Hand rested at 5.0 rev. 1 of 3 C1 brakes at -40 A went SPINOUT.
- 2026-11-07: 2026-11-07 Sitting: Apex 1.10 row passed (4 Attempts, 69 launches, 0 SPINOUT, brakes at most 29 A), but L8157 at Apex 0.95 is the same torque loss at -36 A into the top stop. Stays open until the root cause is known (owner). Flown row removed (detail below).
