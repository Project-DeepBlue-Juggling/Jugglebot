# Learner characterisation Sitting (P2 of `plans/active/learner-generalisation.md`)

Written 2026-11-06, after the first powered run of the merged stack
(`logbook/2026-11-06-sitting-debrief.md`). Runbook row: FEAT-44. About 40 minutes, robot armed and level,
Ball Butler needed only for block D.

**If your physical intuition disagrees with the framing below (what the lateral offset is, why the Throws
drift, which Sites are worth flying), that is load-bearing signal: say so before the Sitting.**

## What this Sitting decides

The Learner's plant model has four terms: an Apex gain K (Throws fly about 8 % high), a lateral offset b
(a few mm), a centre pull s (the landing error grows with the Site's x), and a per-ball offset that is built
but switched off. Every row so far was flown on the x axis with two balls, so three things cannot be told
from the data. Each block answers one, with the rule fixed in advance:

| Block | Question | Rule (plan § 4, Phase 2) |
|---|---|---|
| A | Is the lateral offset a fixed distance, or an aim angle that grows with Apex? | Fit the lateral offset against Apex over five levels. Slope under 2 standard errors from zero: fixed-distance model. Otherwise: aim model. If the data cannot separate them, keep the fixed-distance model (fewer parameters). |
| B, D | Does the centre pull act along y as well as x (radial)? Does the lateral offset turn with the Pattern or stay fixed in the Robot frame? | Fit both forms and keep the one with the lower out-of-sample error. This also fixes the frame a per-beat rotation of the Pattern (the `yaw_rad` field, refused today) would use. |
| C | Do different balls land differently at the same command? | Three balls, 20 Throws each, same Site and Apex. A per-ball offset is switched on only if the balls differ by more than their own scatter. |

Release telemetry (the plan's item d) needs nothing extra at the Sitting: the launch log prints the Tracker's
release time against the command on every `throw n/N CAUGHT ... release +N ms` line, and the bag's
`/robot_state` holds the Hand's velocity at each release. Both are read at the debrief.

## How far this is from what has flown

- Blocks A, B and C are one ball thrown straight up and caught where it was thrown (`Self-toss`). The
  Platform holds the Site; it moves only for the small Catch re-aim. The nomination plans every Segment with
  no Leg motion (2026-11-06, `nominate.nominate` at 600 / 8000 / 200000, Hand 4500: margin 1.000 at Apex 0.60
  to 1.20 and at every Site below). There are no fast transits and no second ball to hit.
- Apex stays at or under 1.05. The standing rule allows 1.10, but a Hand stall stop at 1.10 fits the stroke
  by only 0.08 rev (0.47 rev at 1.00; 2026-11-05 probe on the Hand stall-stop Item), and five levels from 0.70
  to 1.05 answer block A's question almost as well. Higher Apexes are a Hand question (BUG-57).
- Block D is the only two-ball block: the Columns pair at Sep. 120 turned to 90 degrees. Its plan asks the
  Legs for 264 mm/s and 3011 mm/s^2 over 120 mm transits (same offline run). The Sitting of 2026-11-06 flew
  clean up to 361 mm/s, 4672 mm/s^2 and 156 mm transits; the transits that separated Legs 1 and 2 were
  187 mm at 486 mm/s (BUG-72). Block D is inside the clean set. Sep. 120 leaves 46 mm between the balls, so
  some Attempts will end on a collision (BUG-74); repeat until the row count is met.
- The plant stays `jugglebot`. The rows are told apart afterwards by their time (and by Site, `ball_id`
  and `release_kind`), so no `plant_id` is set. (A fresh plant would start from the identity prior and throw about 8 % high.)

## Steps

Mark three balls A, B and C. Ball A is the usual ball. Write down the clock time of each Start; that is how
the rows are found afterwards.

**Before power** (launch up, bag recording, GUI open; nothing moves):

1. `ros2 service call /skills/check std_srvs/srv/Trigger`. It answers with a refusal list; before power
   that is the empty cup and the missing Platform stream (`REJECTED_NO_BALL`, `REJECTED_FRAME_OFFSET`), as on
   2026-11-06.
2. `ros2 param set /skill_node site_x_mm 0.0` and `ros2 param set /skill_node site_y_mm 0.0`, then
   `ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'self_toss,apex_m=1.05,dwell_s=0.27,num_cycles=20'}"`.
   The reply must read `success=True` with a message starting `nomination OK`; a refusal lists every reason.

**Block A, Apex sweep at the centre** (armed, level, ball A in the cup by hand, Site still 0, 0):

3. GUI Juggle panel `Self-toss`, Apex 0.90, Dwell 0.27, Throws 20. Start.
4. The same at Apex 1.00.
5. The same at Apex 0.80.
6. The same at Apex 1.05. Watch the Hand current; cancel if any sample reaches 55.0 A.
7. The same at Apex 0.70.

The order is mixed on purpose, so a slow drift over the Sitting does not look like an Apex effect. If an
Attempt ends before 20 Throws, repeat that Apex once. An Attempt ended by a Hand stall stop is not an
`ADMISSIBILITY` refusal: note it and repeat once.

**Block B, Sites off the centre** (ball A, Apex 1.00, Dwell 0.27, Throws 20; set the Site, then Start
`Self-toss`):

8. `ros2 param set /skill_node site_y_mm 60.0` (x stays 0.0). Start.
9. `ros2 param set /skill_node site_y_mm -60.0`. Start.
10. `ros2 param set /skill_node site_y_mm 0.0` and `ros2 param set /skill_node site_x_mm 60.0`. Start.
11. `ros2 param set /skill_node site_x_mm -60.0`. Start.

The first goal after a Site change moves the Platform to the new Site before the first Throw. Set the Site
only between goals (the node does not refuse a change while a goal runs). Read it back with
`ros2 param get /skill_node site_x_mm` (and `site_y_mm`) if in doubt.

**Block C, ball swap** (centre Site, Apex 1.00; ball A's set is step 4):

12. `ros2 param set /skill_node site_x_mm 0.0` (y is already 0.0). With no goal running, swap ball A for
    ball B in the cup. `Self-toss`, Apex 1.00, Dwell 0.27, Throws 20. Start.
13. Swap in ball C. The same goal. Start. Then put ball A back.

**Block D, the pair along y** (Ball Butler up and loaded, one ball in the cup, as for any two-ball Columns):

14. `ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'columns_turned,apex_m=1.00,separation_mm=120,dwell_s=0.27,num_cycles=21,columns_angle_deg=90'}"`.
    Nothing moves. The reply must read `success=True` with `nomination OK`. This call needs Ball Butler's
    position, so it cannot be made before power: a reply that Ball Butler's position is unknown means Ball
    Butler is not up, not that the Pattern is refused.
15. GUI `Columns — turned`, Angle 90, Apex 1.00, Sep. 120, Dwell 0.27, Throws 21. Start. Repeat, at most
    three Attempts, until about 20 Throws per Site have been caught in total.
16. GUI `Columns`, Apex 1.00, Sep. 120, Dwell 0.27, Throws 21. Same rule. This is the same pair along x, flown
    the same day, for the comparison.

**Afterwards:**

17. `ros2 param set /skill_node columns_angle_deg 0`, `ros2 param set /skill_node site_x_mm -50.0`,
    `ros2 param set /skill_node site_y_mm 0.0` (the launch defaults).

## Stop conditions

- An Attempt ends on an `ADMISSIBILITY` refusal, or a goal is refused by the nomination: stop that block and
  note the line. Do not change limits to get past it.
- Hand current at 55.0 A (step 6), any guard latch, or anything that looks or sounds wrong at the base joints:
  stop and note it.

## Pass, and what to write down

- Pass at the Sitting: every step 3 to 13 has an Attempt of 20 Throws (or two shorter ones), steps 15 and 16
  have about 20 caught Throws per Site, and no Attempt ended on an `ADMISSIBILITY` refusal. The three
  decisions in the table are computed at the debrief, so type the result as `pending`.
- Write down: the launch log and bag paths; the clock time of each Start with its step number; which ball
  (A, B or C) flew in each Attempt; any Attempt repeated and why; anything seen at the base joints.

## At the debrief

- Rows: the Sitting's rows in `temp/learn/jugglebot/memory_v2.csv` (read a copy), split by the step times.
- Block A: lateral offset (landing minus command, x and y) against the commanded Apex; slope and its standard
  error. Also the Apex gain per level, which says whether K is one number over 0.70 to 1.05.
- Blocks B and D: landing error against Site x and Site y; the fixed-frame and the turned fits compared on
  held-out Attempts. One-ball rows (B) and two-ball rows (D) are compared at the same Sites, which also says
  how much of the offset comes from the transit.
- Block C: mean landing and Apex per ball against the scatter within a ball.
- Telemetry: the `release +N ms` values and the Hand velocity at release, per Throw, against the landing
  error. This is also the first data for what sets the lateral drift behind the collisions (BUG-74).
- Outputs: the plant model's structure pinned in the plan's § 2.2, and the frame for a per-beat rotation.
