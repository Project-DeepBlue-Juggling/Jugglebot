# Self-toss landing error: bias or scatter, and which channel?

Type: task
Mode: AFK (Opus: hardware diagnosis)
Status: resolved
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

## Answer

(2026-09-29; evidence: [result_t01](../evidence/result_t01.md), reconciliation in [result_t02](../evidence/result_t02.md); probe `tools/probes/selftoss_landing_decomposition.py`.)

The error is **scatter, not bias.** On rest throws, 43 fitted self-tosses across sittings 1–3 give a 1σ of (15.8, 23.0) mm; the mean is (+6, +2) mm with no direction that repeats. The error enters as the ball's **lateral launch velocity** (σ 18/26 mm/s), not as a platform pose error:
- platform tilt varies by only 0.02–0.09°, and release position by 2.6 mm;
- after subtracting the platform-velocity transfer law at +15 ms, rest throws still show 19/25 mm/s of launch velocity, about 17/22 mm of landing;
- the y error is uncorrelated with the platform's y velocity (r = 0.00).

**The owner's premise does not survive:** a still, well-placed platform does not reach ±10 mm by itself. The residual comes from the ball–cup release (how the ball sits in the cup, sideways play or rolling, lip contact, or the hand axis not being parallel to the platform normal). It needs an instrument at the release to separate these.

Secondary findings:
- Carried throws add error because the platform is still sliding at release (up to 17 mm/s).
- The 20 mm re-aims chase REAL misses (22 of 26), but the early fit that triggers them reads about 9 mm short in x.
- The learner fixes apex (the hand is 6–8 % fast, and the factor drifts between sittings), but its lateral command wanders ±10 mm on noise and made things worse. Implied action: freeze lateral learning and keep apex learning.
