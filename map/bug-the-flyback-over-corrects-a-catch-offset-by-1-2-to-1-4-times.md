---
id: BUG-80
kind: bug
title: 'The flyback over-corrects a Catch offset by 1.2 to 1.4 times: each landing error returns at -0.3 to -0.45 on the next Throw'
status: in_progress
created: '2026-11-06'
updated: '2026-11-13T02:36:51Z'
owners:
- night-shift
priority: medium
effort: s
hardware: robot
owner: none
radius: local
diagnosis: hypothesis
area: learner
blocked_by: []
links:
- logbook/2026-11-06-sitting-debrief-2.md
parent: learner-generalisation
filed_by: claude
---

In the one-ball goals of the P2 Sitting (2026-11-06) each landing error comes back on the next Throw at -0.3 to -0.45 of itself. It damps, so it is not an instability, but the cause is a flyback gain that is too strong, a planner matter and not a plant-model term.

## Findings
- The Cup releases from where it caught: commanded release position against the previous landing has slope 0.87 (r +0.93).
- The planner Banks to fly the offset back. Tilt against the previous landing: r -0.91, -0.21 mrad per mm. Ball lateral velocity against commanded tilt: 6.7 mm/s per mrad, against about 4.2 expected from the Hand speed.
- The flyback returns 1.19 mm per mm of offset against 0.87 mm of release offset, so the net is -0.32, which matches the measured lag-1 slope (-0.30; corpus one-ball x -0.45, y -0.24). The gain is about 1.2 to 1.4 times too strong.
- In Columns it acts at lag 2 (same Site): -0.12 and -0.18, including about -0.1 of demeaning bias.
- The lateral scatter is set by the Platform tilt at release (ball lateral velocity against tilt r 0.78 x, 0.62 y). Release-time error (+9 ms mean, sd 7.5 ms) has no effect on Apex.

## Wanted
- Find where the flyback gain comes from in the Banking and flyback planner, and whether it should be 0.87 instead of 1.19. A change touches the feedforward and timing path: analyse it as a control change and replay it offline first.

## Links
Evidence: `temp/probes/sitting_debrief_2026-11-06b/unit_transit.md` section 3.3, `unit_A_C.md`. Related: `logbook/2026-11-06-sitting-debrief-2.md`.

## Findings (2026-11-07)

**Verdict.** The Item's cause is withdrawn. The planner has no flyback gain that is too strong: it returns 1.00 mm per mm of release offset on paper and in replay.

The over-correction is real on Robot x only, in one-ball Throws only. The excess is in the plant, its mechanism is not settled, and it is worth under 0.5 mm of 18 mm scatter. Do not change the gain.

Evidence base: bag and launch log of 2026-11-06 15:53, 222 one-ball Throws in 12 goals and 115 Columns Throws. Commanded pose is FK of `/leg_cmd_executed`. Measured pose is the mocap Platform body. All slopes are within-goal.

### 1. Where the flyback comes from

- `executor.py::_catch_terminal` sets the release at the caught xy (`_fold_release_mm`), and `target_mm` is the commanded target plus `_offset_flyback_mm`.
- `_flyback_for_offset_mm` returns `r * (1 - 1/g)`, with `g = apex_d / u_apex`, clamped to 0.5..2.0. There is no tuning constant.

- At g = 1.08 that shift is only 0.074 r. Most of the return is plain geometry: the target stays at the Site while the release moved.
- `cup_cycle.takeoff_velocity` turns target minus release into the launch velocity, and `_throw_tilt_from_velocity` points the Cup along it.

Paper gain: planned displacement `u - r/g`, stretched by the plant's g, returns exactly r. At the mean Apex that is 0.271 mrad per mm of r, 4.24 mm/s per mrad and 0.872 s, so 1.00 mm per mm.

Measured against the code: Tilt on r is -0.260 +- 0.002 mrad/mm on both axes, Tilt on u +0.294 on both. Implied return 0.96. Replay at r = 20 mm gives 23.3 mm/s over 0.857 s = 20.0 mm. The code does what it says, isotropically.

### 2. The Item's numbers against this bag

| Claim | Item | x | y |
|---|---|---|---|
| Cup release on previous landing | 0.87 | 0.83 | 0.77 |
| Tilt on previous landing, mrad/mm | -0.21 | -0.207 | -0.183 |
| Ball v on Tilt, mm/s per mrad | 6.7 | 5.67 +- 0.30 | 4.11 +- 0.35 |

| Claim | Item | x | y |
|---|---|---|---|
| Return per mm of landing | 1.19 | 1.02 | 0.66 |
| Net (release minus return) | -0.32 | -0.19 | +0.11 |
| Lag-1 slope, raw | -0.30 | -0.26 +- 0.07 | +0.04 +- 0.07 |
| Lag-1, less -0.05 demeaning bias | | -0.21 | +0.09 |

- The arithmetic holds and the budget closes on both axes (T = 0.872 s). So does the Item's mechanism, as a chain.
- But it is one axis: goal-cluster bootstrap x minus y is -0.30 [-0.49, -0.11] on the lag slope and +1.55 [0.78, 2.25] on the velocity slope.

- 6.7 did not reproduce. Raw ball markers (+30 to +100 ms) give 5.49 +- 0.24 and 4.50 +- 0.29, so this is not a Tracker artefact.
- The corpus -0.45 and -0.24 did not reproduce either: launch logs since 2026-09-28 hold only 18 other `self_toss` lag pairs (x +0.13 +- 0.24). The Item's probe output was out of reach.

### 3. What the 1.6 times is not

Budget on x, mm/s per mrad: launch speed 4.24, Tilt tracking +0.27, Platform velocity +0.25, lever times Tilt rate -0.05. Sum 4.71 against 5.67 measured. About 0.9 +- 0.3 is left.

The same budget on y sums to 4.16 against 4.11, so y has no gap.

| Candidate | Measured | Result |
|---|---|---|
| Over-Tilt (mocap on command) | x 1.064, y 0.973 | small |
| Platform v, -15..+15 ms | x +0.25 +- 0.18, y +0.02 +- 0.13 | small |
| 116 mm lever times Tilt rate | x -0.05 +- 0.03, y +0.01 +- 0.05 | none |

| Candidate | Measured | Result |
|---|---|---|
| Release position | ball on command x 0.99, y 0.97 | none |
| Catch miss (landing minus r) | x +0.10 +- 0.17, y -0.21 +- 0.17 | none |
| Deadband near level (x) | \|Tilt\| >= 2.5 mrad still 5.71 +- 0.32 | none |

- **Learner.** `u` on the previous landing is +0.021 and +0.055, the wrong sign for feedback, and `u` sd is only 3 mm. Not the cause.
- **Regression to the mean.** Bias is -0.05 at 20 Throws. The no-flyback control is positive: `oval_1ball` (release pinned at the Site), x +0.138 +- 0.040, y +0.076 +- 0.040, n 644.

### 4. Why it is not a plain Tilt gain

- Hop entries along x fly about 30 mrad with r near 0: y_x is +0.3 +- 1.4 mm, n 229, Columns corpus. A 1.32 gain would give about +38 mm on 120 mm.
- In this bag two of three x hops give 4.1 and 4.2 mm/s per mrad; the third (+39 mm) gives 6.0. That reads commanded Tilt less a constant -10 mrad offset seen on the y hops (inference).

- It does not carry to Columns: same-ball lag since 2026-09-28 is x +0.008, y -0.054, n 1484, bias -0.075. On 2026-11-06 alone, x -0.033 and y -0.147 +- 0.045, n 555.

- The Columns velocity slope here does not keep the x-only shape: x 4.97 +- 0.75, y 5.98 +- 0.84; along-x goals 4.14 and 6.96, turned goals 5.40 and 5.09.
- Columns Tilt per mm of r is about 0.19 mrad, not 0.26, which is why its budget nets near zero. Why it is lower is not traced.

- Separating the two regressors: ball v on measured Tilt plus r gives Tilt 3.64 +- 0.85, r -0.47 +- 0.25 on x, and Tilt 4.02 +- 0.86, r -0.02 +- 0.22 on y. Tilt sits at Hand speed. The x excess leans to r, at 1.9 sigma (corr 0.97 with commanded Tilt).

- It starts before detach: ball marker -40..-12 ms gives x 3.89 +- 0.20 at a mean axial 3.06 m/s, y 2.57 +- 0.59. Expected on x is about 3.4, so it is not a detach impulse (2.6 sigma).

**Hypothesis, not measured:** the Cup's lateral path during a folded stroke differs from the Platform body along x (Hand rail or tower flex, or the ball in its seat). No marker on the Cup exists to show it. Hints at 2 sigma: x slope 4.17 +- 0.80 at Site x = 0 against 5.89 +- 0.31 at |x| >= 50, and +0.58 +- 0.29 per 100 mm Apex.

### 5. Does a change need to be a gain?

No. Isotropic scaling trades x for y. Predicted lag-1 at scale s: x = 0.83 - 1.02 s, y = 0.77 - 0.66 s.

| s | x | y |
|---|---|---|
| 1.00 (today) | -0.19 | +0.11 |
| 0.92 (least squares) | -0.11 | +0.17 |
| 0.76 (the Item's 0.87 / 1.19) | +0.05 | +0.27 |

- **Worth:** removing -0.21 takes x sd from 18.17 to 17.76 mm (2.2 %). y gains 0.4 %.
- An x-only term would be an unexplained per-axis fudge, on a contract (I-CARRY-1, the docstring's identity prior) that Hop and Columns data do not support.

Offline replay, r = 20 mm, Apex 0.9, cold Memory, `_carried_chain` through the real install, scale patched in the probe only:

| s | Release Tilt, mrad | Peak Tilt rate, mrad/s | Largest knot step |
|---|---|---|---|
| 1.00 | 5.60 | 40.9 | 11.8 |
| 0.87 | 4.87 | 35.6 | 10.2 |
| 0.76 | 4.26 | 31.1 | 8.9 |

Control walk at the 40 Hz tick, if one were ever wanted:

- The scale enters only `target_mm`, at dispatch and accepted re-send, never per tick. The QP seeds from the splice knot, so C1 holds by construction; no refusals in replay.
- Tilt rate stays about 1 % of the 3000 mrad/s cap; lowering s lowers it.

- A re-send moves the release Tilt by a re-aim delta times 0.26 mrad/mm times s, so a smaller s is a smaller retarget.
- Identity prior: `u -> y` keeps its slope, but rows flown at the old scale carry the old r term. At mean r near 0 and k = 16 the residual turns over within one goal (inference).

### 6. Other things found

- At 15:53 the Memory row's `seat` was the dispatch value, not the flown release: `seat` on the previous landing is 0.027 against 0.83 for the commanded Cup. This is BUG-81, fixed 19:50 that evening. Rows before that fix cannot size r.
- `r` on the previous landing is 0.77 to 0.83, not 1: the Cup does not reach the fit (re-send tolerance and refusals). In Columns it is 0.58 to 0.62.

### 7. `## Sitting` candidate (only if the owner wants the mechanism)

- do: from rest at Site (-50, 0), Apex 1.00, fly 40 Throws with no carried fold, commanded landing offsets alternating +20 and -20 mm, 20 on x then 20 on y. Keep the bag.
- pass: slope of ball lateral velocity on commanded Tilt on x is within 4.4 +- 0.5 mm/s per mrad, n 20 per axis. That puts the excess on the fold (r or the Catch).

If x instead shows about 5.7, it is a small-Tilt plant gain, and the Hop result needs explaining next. Either way a Cup marker would then show the path directly. Priority low on the effect size above.

### Verification

(2026-11-07, `/tmp/probe_JB193_*.py` against bag `2026-11-06_15-53-05` and launch logs since 2026-09-28: 222 and 115 Throws matched; Fit landing minus row y sd 1.1 and 1.7 mm.) Probes deleted after the run; no repo change, gate not run.

## The 40-Throw check has no command (2026-11-08)

TKT-47 chose the check. Its section 7 candidate asks for commanded landing offsets of +20 and -20 mm from rest, and no parameter on `skill_node` sets one.

- `self_toss` releases and lands on one Site (`site_x_mm`, `site_y_mm`), so a Throw from rest flies at about zero Tilt.
- The Learner's `u` moves the commanded landing, but `u` is not an operator input.

So there is no `## Sitting` section yet: a `do:` step nobody can fly would be invented. The question is in the ticket filed tonight.

## Sitting
- setup: level, armed
- why: separates a Tilt gain from an effect of the catch-and-rethrow fold on Robot x; only a goal's first Throw flies a commanded offset's Tilt (a fold releases at the caught xy, about 0.4 mrad against 5.0), so the check is 40 single-Throw Attempts from rest.
- do:
  - Before power, launch up from the branch with the bag recording: `ros2 param set /skill_node plant_id bug80-<date>` (a name no Memory file has) and read it back, so these rows stay out of the live Memory. Then `ros2 param set /skill_node landing_offset_x_mm 20.0` and `ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'self_toss,apex_m=1.00'}"`. Nothing moves. The reply must read viable and name `landing offset (+20.0, +0.0) mm`.
  - Leave the Site at its default (-50, 0). Place the ball in the Cup by hand. The Cup must read seated at accept: an empty Cup makes the first skill a Ball Butler Feed, whose Throw is a fold and does not count.
  - Block x, 20 Attempts of one Throw each, signs alternating, 10 of each. Between Attempts, `ros2 param set /skill_node landing_offset_x_mm 20.0` (next time `-20.0`), then `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: self_toss, apex_m: 1.00, num_cycles: 1}" --feedback`. The parameter is refused while a goal is active. After a drop, place the ball back by hand and repeat that sign.
  - `ros2 param set /skill_node landing_offset_x_mm 0.0`, then block y: the same 20 Attempts on `landing_offset_y_mm`.
  - Restore and read back all three: `ros2 param set /skill_node landing_offset_x_mm 0.0`, `ros2 param set /skill_node landing_offset_y_mm 0.0`, `ros2 param set /skill_node plant_id jugglebot`.
- pass: from the bag, the slope of ball lateral velocity on commanded Tilt on x is within 4.4 +/- 0.5 mm/s per mrad, n 20 per axis, which puts the excess on the fold; about 5.7 on x instead is a small-Tilt plant gain and the Hop result needs explaining next. Type the result as pending: the slope is computed at the debrief.
- record: the bag and launch log paths, the number of Attempts flown per axis and sign, each repeat and why, any refusal text, and any Attempt whose first skill was not the Throw of the held ball.
- minutes: 30
- branch: night-shift/2026-11-09-BUG-80

## Sitting result (2026-11-13 13:36)

- verdict: pass
- notes: (sheet 2026-11-13-robot, step 37)

## Progress

- 2026-11-08: Night Shift 2026-11-08: re-rated robot on TKT-47; no Sitting row yet because no parameter commands a landing offset (detail below)
- 2026-11-09: Night Shift 2026-11-09: held on night-shift/2026-11-09-BUG-80: landing offset built, gate passed; unblocked by the 40-Throw Sitting
