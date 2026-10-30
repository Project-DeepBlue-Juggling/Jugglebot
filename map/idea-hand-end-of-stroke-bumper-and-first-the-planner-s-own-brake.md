---
id: JB-114
kind: idea
title: 'Hand end-of-stroke bumper, and first the planner''s own brake shape: let the ceiling, not the Hand, cap the Apex'
status: inbox
created: '2026-10-30'
updated: '2026-10-30T09:16:45Z'
owners: []
priority: medium
effort: l
hardware: robot
owner: decision
radius: cross-cutting
area: hand
blocked_by: []
links: []
filed_by: claude (jugglebot-f5, Opus design study 2026-10-30)
---

Owner's recurring thought 2026-10-30: the ceiling is about 1.5 m above the catch plane (Apex about 1.45 m, release 5.3 m/s); a shock-absorber bumper at the top of the Hand stroke would replace the post-release motor brake so the motor could be run hard. Opus design study 2026-10-30 (scratchpad bumper_study.md, to be promoted). Finding one, before any hardware: the planner's brake SHAPE caps the Apex today, not the hardware. The logged post-release demands 3489 / 3657 / 3843 / 3902 rev/s2 at Apex 0.95 / 1.00 / 1.05 / 1.06 are a constant 1.73 times the ideal constant-deceleration value over the 138 mm brake stroke; the retired firmware throw braked at 1.00 times it and tracked at 0.95. A flat brake at today's 3900 rev/s2 over 138 mm stops a 5.9 m/s throw (Apex about 1.8 m) with no bumper. Rung R0 is offline: find why the cup-cycle QP brakes at 1.73 (throw_coast_weight ships 0, so it is the objective or the rest-terminal rows) and whether a flatter brake is admissible. Finding two, with a bumper: the THROW acceleration with the ball aboard binds (50 A gives 3262 rev/s2 on the ascent); the ceiling Apex needs 48-64 A at release 860 but only 34-48 A if the release (the bumper face) rises to 920-940 mm, which the freed brake stroke allows; the Catch is not a motor limit (relative impact 2.9 to 3.7 m/s, the risk is a bounce out of the Cup). Recommended class: a soft-entry spring plus damper, stroke 40-60 mm, k 1-3 N/mm, c 20-40 N s/m (damping ratio 0.6-0.8), 3.8 J per throw (5.2 J if the rotor's energy goes through the cable), peak force 120-170 N, restitution at most 0.25; prototype as printed TPU or cast PU with a parallel damping element; off-the-shelf miniature hydraulic absorbers (6-13 mm stroke) would stop the carriage at 300-450 N and sit at the edge of their mass and speed ratings. Running the motor harder: current_soft_max can go to 60-65 A under the 70 A hard max; dropping the brake pulse halves copper loss per Beat, buying about 20 ms per throw plus 20-40 ms from the faster stop. Risks: (1) the position loop pushes 53 N into the bumper if the commanded hold sits above its face (a 1 N/mm bumper bottoms out), so the hold must sit below the face, deviation then at most 1.5 rev inside the 2.5 rev hand guard; a stop shorter than 30-50 mm decelerates the rotor (0.110 of the 0.391 kg moving mass) faster than 50 A can follow and loads the cable, slack or derailment on the far side; the cable layout and pretension are not documented in the repo. (2) Platform reaction: same impulse as today's brake (1.45 N s) but 2-3 times the peak force over 20-40 ms into the six position-controlled Legs; measure against the leg guard. Measurement: an off-robot drop tower (a guided 0.28 and 0.39 kg slug from 1.4 m reaches 5.2 m/s), high-speed video plus an accelerometer, fit k, c and restitution, 1000 drops at three temperatures; at plus or minus 20 percent, damping moves the stop depth +5/-4 mm, stiffness about 2 mm. Config discrepancy found: the top hard stop is 10.701 rev (hardware_config.yaml line 524, re-measured 2026-09-06) while the end_stop_margin_rev comment still says 10.8.
