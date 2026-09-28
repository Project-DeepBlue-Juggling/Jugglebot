# Reload catch refused: the resting pose is off the catch line

Type: task
Mode: AFK
Status: resolved
Blocked by: none

## Question

Every reload CATCH was refused at install with `CATCH_AXIS` (seed 5.681 mm and 1.065 mm off the line, bound 0.100 mm). No plan was installed, so the hand stayed parked at the bottom of its stroke and took the ball hard.

Why is the REST pose that precedes a reload catch not on the held-axis line (`site_xy + kappa·(z − site_z)`)? Suspects: the levelling correction, the post-release hold axis change in 977cbb9, a prior skill's end pose, or the BB aim point.

Reproduce the refusal offline from the sitting-3 seed, fix it at the source (the REST that precedes the catch lands on the line, or the catch plans from the seed), and add a test that fails before and passes after.

Also decide the fallback: when a CATCH is refused, should the hand still rise to a safe receive height rather than sit at the bottom?

## Answer

(2026-09-29; evidence: [result_t03](../evidence/result_t03.md).)

The CATCH dispatched at `pretilt_end − LEAD_S`. The fresh-install test already passes there, but the PRE-TILT REST still has 225 ms of its tilt-and-translate slew left. `trajectory_node` seeds a fresh install from the active plan's COMMANDED state at `t_now`, which is mid-slew and so off the held-axis line. The two residuals differ with tick and service-call jitter. The pure-executor path seeds from the record's terminal, which is why the unit tests and the sim never saw it. The old schedule test checked only the arithmetic identity.

**Fixed** in `schedule.compile_reload`: the pre-tilt REST now ends `LEAD_S` earlier, so the CATCH dispatches exactly at the REST's end. The catch motion keeps its realised 0.725 s and its window stays 0.5 s.
- New test `test_compile_reload_catch_dispatches_only_once_the_pretilt_rest_has_ended` failed before and passes after.
- Reload sim gate: 5/5 pass (2026-09-29, `python sim/skills_gate.py --reload`, log `temp/logs/skills_gate_reload_20260929.log`).
- Cost: BB's announcement must lead the landing by about 225 ms more (the sitting-3 delay of 2.96 s is ample).

**Fallback:** none exists. A refused install ends the attempt, and the machine holds the last terminal, which for a reload is the parked, bottom-of-stroke hand. This remains a design gap; see the fog.
