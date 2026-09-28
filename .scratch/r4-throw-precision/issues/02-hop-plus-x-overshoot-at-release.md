# Hop +x overshoot: what is the cup doing at the moment of the throw?

Type: task
Mode: AFK (Opus: hardware diagnosis)
Status: resolved
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

## Answer

(2026-09-29; evidence: [result_t02](../evidence/result_t02.md).)

**Mechanism (a), as TRANSLATION lag, not tilt.**
- Tilt at release matches the plan within 0.03–0.14° on all six hops. The ball does leave at 4–5° against a 3.7–4.1° axis, but the platform's position is doing that, not its orientation.
- The plan swings the platform +x (about +106 mm/s at −75 ms) and asks it to stop at the release knot. The legs lag about 25 ms on the encoders and 40–50 ms in mocap, so at ball separation (+10..+20 ms) the platform still moves at about +100 mm/s.
- Transfer law: the ball's lateral velocity equals the platform x velocity at +15 ms (slope 0.83–1.17, R² 0.70, 38 throws). That gives the +87..+110 mm landings.
- The post-release hold was present in the commands but could not act, because the residual velocity comes from BEFORE release. The sitting-1 attribution (the return after release) is corrected.

Minor or ruled out:
- (c) hand overspeed accounts for only about 15–20 mm of the error.
- (d) is ruled out.
- The learner pushed the right way but hit its 10 mm cap.

**Implied fix:** a PRE-release platform hold, so the platform is stationary for the last N knots and the hand does the final push. Being probed offline for feasibility.

**Robot test:** 5 hops as now against 5 with the hold. The deciding number is mocap platform x velocity at +10 ms: about 84–103 mm/s now, and ≤ 20 mm/s predicted with the hold. The landing x error should fall to within ±25 mm.
