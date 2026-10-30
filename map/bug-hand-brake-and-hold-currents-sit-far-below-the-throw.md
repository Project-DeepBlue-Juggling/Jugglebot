---
id: JB-117
kind: bug
title: 'Hand brake and hold currents sit far below the throw envelope''s model (brake -14 to -20 A measured vs 35 to 40 A predicted; hold 1.5 vs 2.6 A): Kt, moving mass or friction is mis-modelled'
status: inbox
created: '2026-10-30'
updated: '2026-10-30T11:50:57Z'
owners: []
priority: medium
effort: s
hardware: bench-leg
owner: hands
radius: subsystem
diagnosis: hypothesis
area: hand
blocked_by: []
links:
- bug-launch-throw-with-the-ball-aboard-may-demand-more-than-the
- idea-hand-end-of-stroke-bumper-and-first-the-planner-s-own-brake
filed_by: claude (jugglebot-f5, JB-115 bag check 2026-10-30)
---

Side finding of the JB-115 bag check (logbook `2026-10-30-jb115-launch-current-bag-check.md`), 2026-10-28 Columns bags 14:29 and 21:02.

## Finding

- The post-release brake draws −14 to −20 A at a measured −2900 to −3400 rev/s2. The throw envelope's B2 model (`ros_ws/docs/hand_throw_envelope.md`, J 1.05e-5 kg m2 ball-free, Kt 0.0055133) predicts 35 to 40 A there. The 0.281 kg Hand alone needs about 22 A at the configured Kt.
- The ball-free gravity hold reads 1.5 A against 2.6 A computed for 0.281 kg.
- The ascent fits iq = 4.0 + 8.56e-3 a (A per rev/s2), an effective inertia of about 7.5e-6 with the ball aboard, under the model's 1.29e-5; the slope is diluted by 10 ms timing jitter, so it is a lower bound.

## Candidates

Friction helping the brake and the hold; a real Kt 1.4 to 1.7 times the configured value (the configured Kt was pasted from the legs' clamp once before, see the 2026-08-18 braking-clamp fix); a moving mass below 0.281 kg. Not separable from these bags.

## Why it matters

Every current prediction in JB-114 (the Apex climb to the ceiling: 37 A brake, 40 A launch) and in JB-115 uses these constants. If the drive is 1.4 times stronger than modelled, the ceiling has more margin than R0 says; if friction is doing the work, the brake authority on a cold or worn Hand is smaller than the model and the bumper question returns. Settle it with a bench hold-and-step on the Hand (measured iq against a known mass and acceleration) before the Apex Ladder.

Also: the `hand_overspeed_bag_probe.py` docstring says "zero torque feedforward on the streamed hand lane"; in these bags `tor_ff_cmd` reaches +0.126 / −0.146 N m, so that note is stale.
