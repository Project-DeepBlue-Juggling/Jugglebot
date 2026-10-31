---
id: JB-102
kind: feature
title: 'Pre-release hold rung: release from a moving Platform (hold 0.100 → 0)'
status: inbox
created: '2026-10-30'
updated: '2026-10-31T04:52:13Z'
owners: []
priority: high
effort: m
hardware: robot
owner: hands
blocked_by: []
links:
- plans/active/two-ball-oval.md
- logbook/2026-10-31-hold-ladder-sitting-the-hold-model-was-wrong.md
parent: two-ball-oval
filed_by: claude (P3 re-gate close-out 2026-10-30)
---

Owner decision 2026-10-30 (P3 re-gate GO): a pre-release-hold rung is scheduled ahead of the P4 working-point commit (plans/active/two-ball-oval.md P4.0).

## Why

The dwell limit under the pose planner is the 0.100 s PRE-RELEASE HOLD, not the legs. At dwell 0.40 the refusal is HAND_LIMIT_ACC 4308 vs 3900, flat across apex, and it vanishes when the hold shrinks: hold 0.050 makes 0.40 admissible at today's limits (hand -10 %, leg velocity -12 %); hold 0.025 makes 0.35 admissible.

The P4 working point 600/8000/200k produces identical pose plans (limits only gate), so it is margin, not reach. Two balls in one hand need dwell 0.30-0.35 at apex 1.0-1.05, which only a hold of 0 plus P4 permits (offline arithmetic, scratch probe, 2026-10-30).

## Why the hold exists

The R4 finding (2026-09-27/28), the Platform still moving +70..+100 mm/s at separation added overshoot; 0.100 = 45 ms tracking delay + 30 ms lag + 20 ms.

## Design item

The ideal result is NO hold, the Platform moving continuously through the pattern. Release from a moving Platform: compose the Platform's velocity into the release state and aim, and account for the 45 + 30 ms tracking delay using measured, not commanded, state.

## Ladder

Hold 0.100 -> 0.075 -> 0.050 -> 0, at dwell 0.45, judged on landing scatter and catch rate.

## Gate

Landing scatter not worse than the 0.100 baseline (sd about 14-16 mm in the 2026-10-30 re-gate) at dwell 0.45, and catch rate at least 30/30.

## Reframe (2026-10-31)

The Sitting flew the ladder and **withdrew the model** this Item was written on. Do not build delay-aware composition as specified under `## Design item`: it would predict short and add forward velocity, making throws longer.

- Result: a shorter hold lands the ball LONG (hold 0.050 is 39.5 mm long of the control, hold 0 a further 28.4 mm), monotone in the hold, nothing dropped.
- Cause: the Platform tracks the executed command within about 15 ms, not 45 + 30 ms. The carry deceleration excites a roughly 6 Hz ring; the hold only moves the ring phase at release.
- Decision (owner): `pre_release_hold_s` default 0.000. Hold 0 flew 30/30 with the tightest spread (9.8 mm) and leg acceleration down 44 percent.

## Design options that remain

1. Identify the roughly 6 Hz mode from the 2026-10-31 bag and compose through it.
2. Soften the carry deceleration (jerk-limit it).
3. End the deceleration a whole number of ring periods before release.
4. Damp the mode in the leg loop; this reopens the frozen gain hunt and is the owner's call.

## Progress note 2026-10-30 evening

- Reframe: the planner already composes the whole cup velocity (Platform carrier + stroke) into the ball's launch velocity. The hold protects against PLANT TRACKING DELAY (45 ms + 30 ms lag), so the open design item is delay-aware composition, not "compose the velocity".
- Hold 0 at 0.45/150/0.95 predicts a −81..−103 mm short landing, beyond the learner + catch authority (~55 mm); the 10-17 "0.30 at apex 1.0" number is a hold-0 number.
- Gate addition: a mean-bias criterion (plant residual y − u per rung, read by `tools/probes/hold_rung_bias.py --latest` from the launch log and the one shared memory; it attributes goals and holds itself and prints the next hold), since scatter + catch rate cannot see a shift the learner absorbs.
- Sitting Runbook section: `tests/hardware/session_oval_rung1.md` § 11. Logbook: `logbook/2026-10-30-sitting-prep-columns-check-hold-ladder-and-apex-cells.md`.

## Progress

- 2026-10-31: 2026-10-31 ladder flown: shorter hold lands LONG, model withdrawn, hold default now 0; do not build delay-aware composition as specified (detail in Reframe section; logbook 2026-10-31-hold-ladder-sitting-the-hold-model-was-wrong)
