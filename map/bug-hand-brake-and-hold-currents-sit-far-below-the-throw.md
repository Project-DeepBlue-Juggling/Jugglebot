---
id: BUG-41
kind: bug
title: 'Hand brake and hold currents sit far below the throw envelope''s model (brake -14 to -20 A measured vs 35 to 40 A predicted; hold 1.5 vs 2.6 A): Kt, moving mass or friction is mis-modelled'
status: backlog
created: '2026-10-30'
updated: '2026-11-24T13:19:43Z'
owners: []
priority: medium
effort: s
hardware: bench-hand
owner: hands
radius: subsystem
diagnosis: hypothesis
area: hand
blocked_by: []
links:
- bug-launch-throw-with-the-ball-aboard-may-demand-more-than-the
- idea-hand-end-of-stroke-bumper-and-first-the-planner-s-own-brake
filed_by: claude (jugglebot-f5, BUG-40 bag check 2026-10-30)
night_skip: sitting
night_skip_on: '2026-11-22'
night_skip_note: A bench-hand measurement; no pass criterion yet for a Sitting section.
---

Side finding of the BUG-40 bag check (logbook `2026-10-30-jb115-launch-current-bag-check.md`), 2026-10-28 Columns bags 14:29 and 21:02.

## Finding

- The post-release brake draws −14 to −20 A at a measured −2900 to −3400 rev/s2. The throw envelope's B2 model (`ros_ws/docs/hand_throw_envelope.md`, J 1.05e-5 kg m2 ball-free, Kt 0.0055133) predicts 35 to 40 A there. The 0.281 kg Hand alone needs about 22 A at the configured Kt.
- The ball-free gravity hold reads 1.5 A against 2.6 A computed for 0.281 kg.
- The ascent fits iq = 4.0 + 8.56e-3 a (A per rev/s2), an effective inertia of about 7.5e-6 with the ball aboard, under the model's 1.29e-5; the slope is diluted by 10 ms timing jitter, so it is a lower bound.

## Candidates

Friction helping the brake and the hold; a real Kt 1.4 to 1.7 times the configured value (the configured Kt was pasted from the legs' clamp once before, see the 2026-08-18 braking-clamp fix); a moving mass below 0.281 kg. Not separable from these bags.

## Why it matters

Every current prediction in IDEA-17 (the Apex climb to the ceiling: 37 A brake, 40 A launch) and in BUG-40 uses these constants. If the drive is 1.4 times stronger than modelled, the ceiling has more margin than R0 says; if friction is doing the work, the brake authority on a cold or worn Hand is smaller than the model and the bumper question returns. Settle it with a bench hold-and-step on the Hand (measured iq against a known mass and acceleration) before the Apex Ladder.

Also: the `hand_overspeed_bag_probe.py` docstring says "zero torque feedforward on the streamed hand lane"; in these bags `tor_ff_cmd` reaches +0.126 / −0.146 N m, so that note is stale.

## Capture fit (2026-11-25)

Fit over 4117 clean strokes in 54 Hand captures (2026-11-03 to 2026-11-23), probe `tools/probes/hand_launch_iq_census.py`. Model: iq = c1 x acceleration + c0.

| Quantity | Measured | Model |
| --- | --- | --- |
| c1, ascent with the ball (A per rev/s2) | 0.0073 to 0.0097 | 0.0147 |
| c1, brake after release | 0.0037 to 0.0063 | 0.0120 |
| Friction | 1.2 to 1.5 A | none |
| Static hold, no ball | 1.85 A | 2.59 A |
| Static hold, ball aboard | 2.60 A | - |

- **The ball adds 0.5 to 0.75 A of hold against 0.655 A predicted, so the torque constant is close; the model's inertia or moving mass is about twice too high.**
- The envelope therefore overstates ascent current by 1.5 to 2 times and brake current by about 2.5 times: conservative for peak current.
- Captures cannot separate the torque constant from inertia; the bench ramp series below does.
- Ranges, not standard errors: the spread is dominated by the iq-to-velocity lag assumed (0 to 20 ms).

## Sitting

- setup: bridge-cycled
- why: the Hand's iq per unit acceleration and its friction and hold offsets are mis-modelled; a bench ramp series at known accelerations separates inertia from friction and gravity without the robot Hand's damaged motor.
- do:
  - Launch down, bridge up, Cup empty, motor cool, E-stop button in hand. Home the bench Hand with `python tests/hardware/bench_hand_frame_bench.py home` and write down the bench top stop it reports: every stroke below is sized from it, never from the production stroke.
  - Friction and hold: `python tests/hardware/hand_stream_bench.py --stage triangle --tri-span 2 --tri-speed 2 --close-loop`, then again with `--tri-speed 6`.
  - Inertia: for 500, 1000 and 2000 rev/s2 run `python tests/hardware/hand_stream_bench.py --stage stroke_cut --close-loop --top-stop <bench top stop> --stroke-accel <A> --stroke-peak-vel <V> --repeat 5`, with V chosen so V squared over A is at most half the travel from the start to the top stop (A 1000 with V 30 is 0.9 rev). Accept every refusal the stage gives; never lengthen the stroke toward the top stop.
  - Repeat the 1000 rev/s2 row once with a 71 g ball taped in the Cup.
- pass: per run, up-stroke peak iq minus the up-going friction offset is linear in acceleration: the three c1 values agree within 15 % and the fit residual stays under 1.5 A. Verdict `pass` if ball-free c1 falls in 0.0035 to 0.0070 A per rev/s2 and the up-going offset in 1.0 to 2.5 A; outside that the Item stays open with the model to re-identify.
- record: the bench top stop, each run's CSV path, per acceleration the peak iq (up and brake), c1 and offsets with and without the ball, motor temperature before and after.
- minutes: 30

## Progress

- 2026-11-25: capture fit over 4117 strokes: iq per unit acceleration is about half the model's (inertia or moving mass too high, friction 1.2 to 1.5 A); the ball's hold increment says the torque constant is close. Sitting section written (bench ramp series). Body section Capture fit (2026-11-25).
