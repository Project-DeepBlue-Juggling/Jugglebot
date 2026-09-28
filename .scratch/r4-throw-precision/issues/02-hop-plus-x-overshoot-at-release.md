# Hop +x overshoot: what is the cup doing at the moment of the throw?

Type: task
Mode: AFK (Opus: hardware diagnosis)
Status: open
Blocked by: none

## Question

Why do the hop throws land +88 to +110 mm long in x, with an apex of 0.99–1.02 m against a band of 0.85–0.95 m? Test the owner's hypothesis that this is a tilt/orientation error at release, and find the actual mechanism.

Reconstruct each hop throw across sittings 2 and 3 (bags `2026-09-28_21-41-10` and `2026-09-28_23-55-19`):
- the planned release: cup velocity vector, cup tilt, and where the post-release hold starts;
- the realised release from the leg and hand encoders (FK), and mocap if available: tilt, velocity and timing of the actual release;
- the ball's launch velocity from the flight fit.

Discriminate between these candidate mechanisms:
- (a) realised tilt ≠ planned tilt, from leg tracking lag on a fast lateral move;
- (b) the ball follows the cup velocity, not the cup axis, and the plan has the wrong velocity;
- (c) the hand stroke is overspeeding (the 0.9 m overspeed history; K=0.7 torque FF);
- (d) the levelling correction or frame transform is applied wrongly for the lateral component (the 977cbb9 hold-axis change);
- (e) release timing: the ball leaves before or after the planned knot while the platform is still accelerating in x.

Also note whether the overshoot scales with apex or learner rows (memory rows 10 → 11 at sitting 3). Name the mechanism, the evidence, and the test that would confirm it on the robot.
