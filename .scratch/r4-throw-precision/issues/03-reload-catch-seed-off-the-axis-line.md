# Reload catch refused: the resting pose is off the catch line

Type: task
Mode: AFK
Status: open
Blocked by: none

## Question

Every reload CATCH was refused at install with `CATCH_AXIS` (seed 5.681 mm and 1.065 mm off the line, bound 0.100 mm). No plan was installed, so the hand stayed parked at the bottom of its stroke and took the ball hard.

Why is the REST pose that precedes a reload catch not on the held-axis line (`site_xy + kappa·(z − site_z)`)? Suspects: the levelling correction, the post-release hold axis change in 977cbb9, a prior skill's end pose, or the BB aim point.

Reproduce the refusal offline from the sitting-3 seed, fix it at the source (the REST that precedes the catch lands on the line, or the catch plans from the seed), and add a test that fails before and passes after.

Also decide the fallback: when a CATCH is refused, should the hand still rise to a safe receive height rather than sit at the bottom?
