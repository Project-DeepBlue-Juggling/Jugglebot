---
id: IDEA-10
kind: idea
title: 'JP-9 High-low columns: Ball 1 at apex 1.05 m, Ball 2 at 0.95 m'
status: complete
created: '2026-10-30'
updated: '2026-11-13T05:58:04Z'
owners:
- jugglebot-f5
priority: medium
effort: m
hardware: robot
owner: hands
radius: subsystem
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-10-sitting-debrief.md
- logbook/2026-11-13-sitting-debrief.md
filed_by: claude (jugglebot-f5, pattern exploration 2026-10-30)
parent: pattern-progression-columns-to-centre-and-orbit
---

Owner wants to see it (2026-10-30). Two-ball columns with unequal apexes.

## Timing

- The 0.045 s flight difference is absorbed by the Dwells (the low ball holds longer) and shortens one of the two empty transits.
- The offline probe planned 1.05 over 0.95 at today's limits under pose.

## Binding constraint

Hand acceleration at the high ball (3843 of 3900 rev/s2 at 1.05).

## Software

- Pattern carries one apex_m for both Sites and the box keys on one apex band, so the compiler needs a per-Site apex and the admissible box a two-band cell.
- The learner's apex outcome is already per throw.

## Appearance

Visually subtle: the two apexes differ by 100 mm.

## Sitting

- setup: level, armed, bb-fed
- why: P5 of plan pattern-progression.md: two Apexes in one Pattern (JP-9). The Rung flies 1.00 over 0.90 at separation 100; since the 2026-10-31 re-sweep (Hand prime 10.45) 1.05 over 0.95 is admissible at both Sites too, but a 1.05 launch waits for the faster hand current capture (plan D-c).
- do:
  - Rehearse before arming: `ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'columns_highlow,apex_m=1.00,separation_mm=100,dwell_s=0.27,apex_m_ball2=0.90'}"`. The reply must read `success: True` with no refusal line; nothing moves, and launch up and homed is enough. Never rehearse with a Juggle goal: with the Cup empty it is accepted and moves the Platform and the Hand (found 2026-11-10).
  - GUI: Juggle panel `Columns — high-low`, Apex 1.00, Apex 2 0.90, Sep. 100, Dwell 0.27, Throws 21 (Start writes apex_m_ball2 to the node first). Terminal fallback: `ros2 param set /skill_node apex_m_ball2 0.90` (refused while a goal runs), then `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: columns_highlow, apex_m: 1.00, separation_mm: 100.0, dwell_s: 0.27, num_cycles: 21}" --feedback`.
  - Watch the hand current at every launch; cancel the goal if any hand iq sample reaches 55.0 A (the hand current limit is 60 A since 2026-11-01, was 50 A with a 50.0 A watch).
  - Afterwards `ros2 param set /skill_node apex_m_ball2 0.0`.
- pass: 20 consecutive Catches in one Attempt, flown with Throws 21 (the 21st throw is the Stop throw, not counted, so a full Attempt reads `COMPLETED (20/20 caught)`; with Throws 20 it reads 19/19 and the bar is unreachable); Tracker mean Apex difference between the two balls within 20 mm of the commanded 100 mm; hand iq under 54 A at every launch, judged on launches only (the brake is excluded; it reached the 60 A clamp twice on 2026-11-03).
- record: caught of thrown, each ball's mean Tracker Apex, peak hand iq per launch, Ball 2's first-throw landing (x, y) mm per Attempt from the install line (BUG-63: 4 of 11 landed about 180 mm outward and a fifth 37 mm on 2026-11-03), the bag path; commit docs/records.md.
- minutes: 12

## Sitting result (2026-11-03)

- verdict: pending
- notes: With Ball 1 at apex 1.0 m, Ball 2 at 0.9 m: First, third, fourth attempts ended with a messy throw being uncatchable. Second ended with leg ACCEL_LIM error. One attempt ended with SPINOUT_DETECTED, for which I heard what sounded like the hand colliding with the end-stop. Trying with Ball 1 at apex 1.05 m, Ball 2 at 0.95 m, there were a lot of messy throws, leading to collisions and missed catches. Every attempt ended this way.

## Sitting result (2026-11-10)

- verdict: fail
- notes: Flown as real Attempts from sheet step 7 on the default plant: 0/0, 0/0, 2/3 (HAND_JAM). Ball 2's first Throw after the Feed landed outward (BUG-63). Verdict from the logs at the debrief, confirmed by the owner 2026-11-11.

## Sitting result (2026-11-13 12:05)

- verdict: pass
- notes: Took a few attempts, with many attempts ending due to messy throws, but after some tries the pattern held for the full 21 throws! (sheet 2026-11-13-robot, step 14)

## Progress

- 2026-10-31: P4 of plan pattern-progression.md done offline: goal columns_highlow, parameter apex_m_ball2, per-ball Apex and dwell in the compile, 16 pose cells swept. Nothing fits at separation 125; at 100 the pair 1.00/0.90 fits at both Sites, 1.05/0.95 only the high ball. Next: the JP-9 Rung (P5) at 100 with 1.00/0.90. Logbook 2026-10-31-pattern-progression-p4-high-low-columns.md.
- 2026-11-01: 2026-11-01 Sitting: gate NOT met. 8/8 (GUARD_LATCHED) and 12/13 (leg LIMIT_ACC 5142.8 > 5000, a late re-send seed); apex difference 87 mm vs 100 commanded (Ball 1 0.997, Ball 2 0.910 m); hand iq peaks 32.1 / 41.5 A. First high-low records are in docs/records.md.
- 2026-11-03: Failed at the Sitting of 2026-11-03: best 18 consecutive at 1.00/0.90, 15 at 1.05/0.95; apex gap 119.2 and 89.7 mm; launch iq max 36.5 A; brake hit the 60 A clamp twice; 5 of 11 attempts ended on Ball 2's first throw landing ~180 mm outward. Plan D-c answered: 1.05/0.95 flown, Ball 1 reached only 1.019 m.
- 2026-11-10: Failed at the Sitting of 2026-11-10 (step 7, real Attempts, default plant): 0/0, 0/0, 2/3 (HAND_JAM). Ball 2's first throw after the Feed outward 157, 23, 37 mm (BUG-63); Ball 1's hop-entry Catch missed twice, fit 26 mm off. No end on an admissibility refusal, so the probe's -0.245 re-aim margin is not the killer. Logbook 2026-11-10-sitting-debrief.md.
- 2026-11-13: Passed at the Sitting of 2026-11-13: columns_highlow COMPLETED 20/20 at 12:03:51 (one of 9 Attempts on that plant); Tracker mean Apex difference 100.9 mm (1.0099 vs 0.9090 m) against the commanded 100 mm; Hand iq 35.9 A over that Attempt. Not verifiable from the log: Ball 2 first-Throw landing from the install line.
