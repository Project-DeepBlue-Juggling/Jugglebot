# Self-toss landing error: bias or scatter, and which channel?

Type: task
Mode: AFK (Opus: hardware diagnosis)
Status: open
Blocked by: none

## Question

Why do the self-toss balls land far enough from their scheduled landing that almost every catch uses the full 20 mm re-aim? Is the error a repeatable bias (fixable open-loop) or scatter (a noise floor)? Which channel does it enter through?

Evidence to start from (sitting 3, `temp/logs/skills_r4_20260928_2355.log`):
- 8 of 11 re-aims were exactly 20.0 mm, so the correction was clamped.
- OUTCOME landings were within about ±30 mm.
- The apex was 0.98 m in run 1 and 0.91 m in run 2 (commanded apex still to be confirmed).

Decompose, per throw, across sittings 1 to 3 (bags `2026-09-27_22-37-26`, `2026-09-28_19-53-37`, `2026-09-28_21-41-10`, `2026-09-28_23-55-19`):
1. The commanded landing and apex: the schedule prior plus the learner command (`memory rows`), in the schedule frame.
2. The planned release state: cup position, velocity and tilt at the release knot of the installed plan.
3. The realised release state from the leg and hand encoders (forward kinematics) at the same instant, and from mocap if the platform is marked. Is the plant following the plan at release?
4. The ball's initial velocity from the converged flight fit. Does it match the realised cup velocity (a clean release)?
5. The landing error split into: plan error + tracking error + release error + frame offset.

Report whether the error is repeatable across throws and sittings, and give the effective 1σ with re-aim contributions removed. Name the dominant term and the fix it implies. Probes go to `tools/probes/` if reusable, with outputs to `temp/probes/`.
