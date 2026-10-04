---
title: "R5 sitting 4 (2026-10-04 evening): the hand-jam detector was blind to a 1 Hz diagnostic, a missed feed never ended the pattern because the cup sensor confirmed the other ball's seat, the catches are on time but Ball Butler lands 41 mm off the cup, and the live refusals are landing-time jitter against a 10 % margin"
type: investigation
date: 2026-10-04
status: in-progress
phase: "two-ball-skill-stack — R5"
related_plan: two-ball-skill-stack.md
related_entries:
  - 2026-10-04-skill-stack-r5-sitting-3.md
  - 2026-10-02-skill-stack-r5-sitting-2.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/hand_jam.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp
  - sim/skills_gate.py
  - tools/probes/hand_jam_replay.py
  - tools/probes/missed_catch_fixture.py
  - tools/probes/feed_lateral_miss.py
  - tests/motion/test_hand_jam.py
  - tests/motion/test_skills_executor.py
  - tests/motion/fixtures/hand_jam_20261002.csv
  - tests/motion/fixtures/missed_catch_20261004.csv
  - tests/hardware/session_skills_r5_sitting5.md
tags: [skill-stack, R5, columns, ball-butler, hand-jam, missed-catch, can-bridge, firmware]
---

## Summary

Four launches on the evening of 2026-10-04 flew the sitting-4 runsheet: the jam bench check
(§ 2), the level measurement (§ 3, 31 self-tosses → `level_trim_deg [+0.0945, +0.3678]`),
one-ball columns (§ 4, "passed fairly well, very few catches clean"), twenty Ball-Butler-fed
columns attempts (§ 5), and two more after the owner re-aligned the QTM frame to the Base
frame. Fed columns never got past four throws. The owner's notes name three failure shapes —
the fed ball not caught yet Jugglebot's own ball "thrown every time", the fed ball "caught but
not seated before the first throw", and `LIMIT_JERK/ACC/VEL` at the third or fourth throw —
plus two hand pinches on the funnel ring that the new jam detector never acted on, one of
which the owner had to E-STOP.

Six analyses (five agents plus the orchestrator's own replay; reports under
`temp/reports/r5_sitting4/`) answered the owner's questions:

1. **The jam detector could never fire on the robot.** Its P6 predicate demanded the hand
   axis's DIAGNOSTIC frame to be younger than 50 ms on every sample through the 100 ms
   sustain, and the can-bridge sends that frame on change or at 1 Hz: in a steady stall at
   the clamp the next frame came 1.018 s after the hand hit 50 A. P6 was true for ≤ 50 ms per
   second. The offline replay and the fixture computed the age from the 100 Hz `/robot_state`
   republish instead, so they fired where the robot could not — the production-faithful
   replay lesson, third time. A second gap: `HAND_MOVE_TO` never writes the command cache,
   so the bench check (§ 2) compared against a stale 0.0 and could not have tested P3.
2. **A missed feed catch does not end the pattern because the cup sensor confirms the other
   ball's seat.** The possession observer ignores `ball_id`, and the release-evidence latch
   accepts a SEATED reading at any time after release: ball A arriving at P2 and its
   20–80 ms settle flicker fell inside ball B's 0.5 s grace window and "confirmed" B's empty
   release — on a 40 Hz tick lottery, which is exactly the split between the attempts that
   ran all six throws and those that stopped at throw 2. Even a correct drop comes one beat
   late. The cup sensor alone separates the populations: zero raw SEATED samples on 14/14
   missed feeds (22 feeds; attempt 2's was a miss too), a seat by +0.27 s on 8/8 caught ones.
3. **The catches are on time; the bounce is lateral.** Contact lands within ±10 ms of the
   planned touchdown in every population, the hand is 3–14 ms late (not early), and bounce
   does not correlate with timing (r = 0.00) but with lateral miss (r = +0.51; seat time
   +0.69). Ball Butler's balls land (+29, +25) mm from the point it was asked to hit on every
   feed (41 mm radial median, y sd 10 mm), unchanged by the QTM re-alignment, and arrive
   1.3 m/s faster than Jugglebot's own throws. No feed (0/15) seated before the hand reversed
   for ball B's first throw at +0.19 s — the "caught, not seated" chain, which then throws
   88 mm wide at P1.
4. **The live `LIMIT_*` refusals are landing-TIME jitter against a 10 % margin**, not
   landing offsets. All nine fed refusals and both one-ball refusals were regular catch folds
   at or near the nominal site. In the virtual loop at the flown point, a catch reported
   20 ms early compresses the site-to-site transit (`LIMIT_ACC/VEL`), 20 ms late compresses
   the catch-to-release dwell (`LIMIT_JERK`), reproducing the live magnitudes (+2..+22 %) and
   the code mix (6 jerk, 2 acc, 1 vel). Neither the box nor the plan-bench rehearsal ever
   varied landing time. More dwell buys NO margin here (the two-ball transit is
   (flight − dwell)/2), and the session acc/jerk limits already sit on the YAML ceilings.
5. **Attempt 2's guard latch was a false trip on stale hand feedback.** The hand encoder
   estimate froze for ~95 ms during a normal throw stroke (the current showed the normal
   accelerate-then-brake signature); the hand guard extrapolates a frozen velocity with no
   age gate and latches on one 2 ms tick, so at an anchor age of 99.9 ms it saw 3.27 rev of
   deviation where the true residual was ~1 rev. The underlying dropouts are the stream-gated,
   uptime-independent ODrive-side frame-drop class `plans/active/leg-bus-frame-drops.md`
   already owns (2 episodes per streaming minute at 3 h and at 94 h alike; the log's
   "0/N encoder frames" is a 1 Hz census artefact — real encoder gaps max ~100 ms).

Also found: `hold_tilt_max_deg` only reaches the reload compiles, so the owner's change to
4.0 before attempt 9 was a no-op for fed columns; the level trim's effect on the landings
could not be confirmed (the learner moved the self-toss command 15 mm in the same session);
the ball naming convention is settled (**Ball 1** = the ball in Jugglebot's hand at the
start, schedule id 0; **Ball 2** = Ball Butler's, id 1; note Ball 1 holds at P2 and Ball 2
lands at P1).

Fixes landed the same evening (§ Fix): the detector's age bound derived from the firmware
cadence plus a measured-descent P1 and `HAND_MOVE_TO` writing its target as the command (the
detector now fires on both of today's pinches in a pessimistic-age replay); a `MISSED_CATCH`
end on a sample-complete seat window plus a 0.10 s seat gate on the release evidence; can-
bridge FW 27 (the hand guard counts only fresh feedback and the 150 ms staleness suppression
covers axis 6); the `columns_1ball_fed` practice mode the owner asked for; a
`columns_feed_bb_bias_mm` parameter with the probe that measures it; Ball 1/Ball 2 in every
operator-facing line. The refusal margin is NOT fixed — it needs a geometry or ceiling
decision (§ Open Questions).

## What the owner reported

(Verbatim intent, 2026-10-04 evening.) Step 2 of the runsheet was mistaken: "the detector
takes the command from `_last_hand_cmd`. Only the streamed lane's echo frame writes that
value, plus ACTIVATE … HAND_MOVE_TO reuses ACTIVATE's TRAP_TRAJ move: the ODrive plans the
trajectory internally and nothing echoes it back. So pos_cmd stayed at its startup value of
0.000 … P3 was only true by coincidence." The level trim came from an Opus agent's analysis
of the 31 throws. Step 4 "passed fairly well (though very few catches were what I'd call
'clean')". Twenty fed attempts, in order: BB ball not caught / JB ball thrown every time /
should have stopped (1); TEENSY GUARD LATCH (2); caught but not seated before the first
throw, first throw messy (3, 6); jerk limit, balls collided after ~3 throws (4, 5); messy,
collided, both dropped (7); fed ball messy, jerk limit (8); `hold_tilt_max_deg 4.0` set out
of curiosity; BB throw dropped / bounced out / not seated (9–11, 13, 15, 17, 19, 20); "almost
a full success, just a little too messy to be sustainable" (12); bounced, LIMIT_JERK (14);
"hand jammed on BB's ball without the detector firing … hand temp rose by far more than
normal" (16); LIMIT_VEL (18). After re-aligning the QTM frame: BB not caught, pattern
continued through all requested throws (R1); BB ball jammed under the hand after the pattern
tried to stop, E-STOP before the hand got too hot, "DETECTOR FAILED" (R2). Asks: improve
general catches ("hand slightly too early or late"); practise the other half of columns with
one ball fed straight from Ball Butler; make the jam detector active at all times, at least
in the ring band; and a way to refer to each ball ("Ball 1"/"Ball 2" by Jugglebot's first-
throw order).

## Measured

Launches and bags: L1 19:26 (bench check; `/hand_move_to 4.0` ARRIVED in 3739 ms, the 0.0
return refused ERR_BUS_DOWN because the owner had already E-STOPped), L2 19:39 (31 self-
tosses, `level_vs_ballfit` lean x +0.0945 y +0.3678 deg), L3 19:52 (trim applied at
1791103956.66; four one-ball runs; 20 fed attempts 1791104229–933), L4 20:10 (QTM re-aligned;
two fed attempts; pinch at 1791105152.80, power cut ≈ 155.0). Bags `~/Desktop/rosbags/
2026-10-04_{19-26-00,19-39-06,19-52-08,20-10-26}`.

**Fed attempts (20 + 2).** End codes: ABORTED_NO_RELEASE 9 (throw 4–6 "never left the hand"),
LIMIT_JERK 6 (203–244k vs 200k), LIMIT_ACC 2 (5100–5104 vs 5000), LIMIT_VEL 1 (352.6 vs 350),
DROPPED_SURVIVOR_STOPPED 3, GUARD_LATCHED 1. Throw 1 (ball A) was caught cleanly on every
attempt; no attempt reached a fifth throw with both balls.

**The jam (orchestrator replay + bag).** Replaying the bags through the detector as shipped
(`tools/probes/hand_jam_replay.py`, age from `/robot_state`) fires HAND_JAM at 1791104785.346
(attempt 16: hand +2.546 rev, cmd +0.545, iq −52.6 A, 0.12 s at the clamp) and at
1791105152.939 (R2: +2.533 rev, cmd +0.534, −48.3 A, 2.17 s at the clamp until the power cut;
motor thermistor 31 → 46 °C). The live node was silent both times. In the R2 bag the hand
reached the clamp at 1791105152.80; the hand axis's diagnostic cache next changed at
1791105153.82 (+1.018 s), then at +0.40, +0.30, +0.20, +0.20 s as the motor temperature
crossed 1 °C steps. The firmware's `diag_changed` (`telemetry.cpp`) sends a per-axis
DIAGNOSTIC on an iq_setpoint change > 0.5 A, a temperature change > 1 °C, a bus-voltage
change, or state/error changes, else forced once per `DIAG_FORCE_PERIOD_US` (1 s) per axis,
staggered. The detector's `max_age_s` was 0.05.

**Missed-catch evidence (A1, `temp/probes/a1_*_s4.log`).** Attempt 1: the raw cup bit was
EMPTY on 50/50 valid samples from −0.257 s (ball A's throw 1 leaving) to +0.702 s (ball A
arriving at P2) around the feed landing L; ball B never touched the sensor. B's carried throw 2
(release +0.261, deadline +0.761) was "confirmed" by A's SEATED at +0.702 and its 24 ms EMPTY
flicker at +0.726. Throw 4's deadline missed A's next flicker by 53 ms → DROP at +1.935; skill 5
had already dispatched at +1.687; throw 6 ran empty; END ABORTED_NO_RELEASE at +3.083
overwrote the D3 code. Over 25 tick phases, throw 2 "confirms" on 17–25 phases for attempts
1, 9, 10, 16, R1 and on 0 phases for 11, 13, 17, 19, R2 (the ones that stopped at throw 2);
attempt 15 sat on a 1 ms boundary. Caught feeds seated at +0.163..+0.272 s (n = 8); over all
Ball Butler flights the 11 that seated did so by +0.272 s. The feed catch registers no
outcome row, and the tracker labels a missed ball CAUGHT at landing with its position
extrapolated below the floor — neither is miss evidence.

**Catch timing (A2, `temp/probes/a2_*_s4.*`).** Raw `/mocap_data` markers tracked per catch;
contact = the first departure from a fixed-g free-fall fit (≤ 7 ms); the cup plane from the
hand position and the mocap Platform body. Contact − planned touchdown (ms, median
[p10, p90]): one-ball P2 −3.9 [−11.0, +10.6] (n 25); fed P2 ball A −4.4 [−10.1, +3.7] (20);
fed P1 ball B −5.8 (4); feeds +3.3 [−9.6, +11.4] (7). The ball's free-fall arrival at 830 mm
vs the plan: +7.5 [+0.7, +21.9]; +5.3; −0.7; 0 ± 12. The commanded cup reaches 830 mm ~5 ms
after t_land and the measured cup trails it 5–8 ms; the ball meets the seat ~17 mm above the
modelled plane (seated ball at +17.5 mm, n 42), so contact comes 9–12 ms before the free-fall
crossing and nearly cancels the lateness. Impact (vertical, mm/s): JB's own catches ball
−4205, cup −1590 (0.39×, as the τ 0.125 s profile was designed — the QP's 0.7 ratio is capped
~0.39 by the C-CUP-2 window), relative 2554; feeds ball −5551, cup −1570, relative 3752 plus
~1.1 m/s lateral. Rebound after contact: JB 9–17 mm, feeds 143 mm (n 6). First SEATED: JB
+107..111 ms median [53, 209]; feeds 7 at +162..+270 ms (during B's throw stroke) and 7
never (their +605..+712 ms "seats" are ball A arriving). Lateral miss at arrival, feeds:
ball − Platform-body xy (+29, +25) mm median, radial 41 [32, 63], min 23; after the QTM
re-alignment (+33, +23) (n 2). The cup sat exactly at the point sent to Ball Butler
(CATCH-AIM skill 1 (−52.5, 0, 830) = the feed site walked 10 mm toward A). JB's own P2
catches read 4–6 mm by the same method.

**Refusals (A3, A4).** Each of the nine fed refusals is a regular CATCH(+then_throw) fold,
none the D2 Stop fold; the committed aim was at the nominal site or inside the 20 mm
authority; the two one-ball refusals (jerk 213k, vel 381) came with the preceding aim exactly
(−62.5, 0, 830). Virtual loop (`temp/probes/a4_dt_sweep_s4.log`, the production executor
against a boxed clock at the bench operating point, tracker t_land = nominal + dt), worst
peak/cap over 8 installs, `*` = refused:

| dwell, limits | −30 ms | −20 | −10 | 0 | +10 | +20 | +30 | alt ±20 |
|---|---|---|---|---|---|---|---|---|
| 0.27, 350/5000/200000 | 0.82* | 1.00 (acc 4991.5) | 0.90 | 0.89 | 0.89 | 0.82* | 0.82* | 1.00* |
| 0.30 | 0.82* | 0.82* | 0.82* | 0.91* | 0.93 | 0.89 | 0.89 | 0.82* |
| 0.33 | 0.82* | 0.82* | 0.82* | 0.82* | 0.96* | 0.96 | 0.89 | 0.82* |

Session acc/jerk (5000 / 200000) equal the YAML ceilings (`leg_acc_ceiling_mmps2`,
`leg_jerk_ceiling_mmps3`), so `with_session_limits` clamps any "raised" request back to them
(raw peaks bit-identical). Landings: Ball Butler feeds offset from the committed site (A3's
probe, frame unreconciled with A2 — B4 settles it) x −57 (24.9) y +19.6 (39.6) mm (n 20);
JB self-tosses L2 (trim 0, n 31) x −0.8 (19.0) y +2.3 (18.2); L3 trimmed (n 19) x −9.3
(18.7) y +6.1 (23.1). The learner appended 144 rows today (120 caught / 24 not); the self-
toss command moved from (−16.8, −3.2) mm to (−1.5, −19.5) mm within the session. 38
TRACKER-IDENTITY-REFUSED lines, 24 at the feed catch; every one "the catch keeps the
schedule"; no wrong-ball row reached memory.

**Feedback dropouts (A5, `temp/probes/a5_*_s4*`).** 156 "heartbeat gap" INFO lines across
eight launches since 2026-10-02: ~2 episodes per streaming minute and 6.5–10.8 % of streaming
windows at 3 h and at 94 h of bridge uptime alike; zero in 514 idle windows; random victim
axis; bus utilisation 62 % streaming. Every bridge-side counter flat (seq_gaps, crc,
decode, rx_cap_hits, fifo_overflows, leak = 0): the ODrive does not transmit the frames.
Heartbeats vanish for 1–2 s while encoder frames only thin (10–90/s), hard ceiling ~100 ms
between them (`/cache_diag age_max` ≤ 99 ms in every window of every bag). Attempt 2: hand
heartbeat silence ~245.0–246.2; `/robot_state` hand frozen at 1.213 rev / 51.30 rev/s from
246.006 to 246.101; latch snapshot u0 9.6116, fb_ex 6.3405 → anchor age 99.9 ms; true residual
~0.4–1.4 rev. `leg_interp.cpp` extrapolates `fb + fbv·age` (age capped at 150 ms) and
`fault_machine.cpp` latches on one exceed tick; the leg-only `MOTOR_FB_STALE` suppression
excludes axis 6. Sitting 4's three CLAMP_DUTY warnings are honest leg-4 lag in the one-ball
runs, the mask frozen during the 68 s guard latch, and the R2 jam — none stale feedback.

## Discussion

**Why the jam detector's fixture passed.** The detector was designed against one bag with a
replay whose P6 age came from `/robot_state`'s 100 Hz republish of the cached diagnostic —
0–10 ms always — and the fixture CSV carried that number as `diag_age_s`. The live node
measures the age of the DIAGNOSTIC frame itself. The two agree only while the hand's iq is
changing by more than 0.5 A every tick, which is every moment EXCEPT a steady stall at the
clamp — the one state the detector exists for. This is the production-faithful-replay
lesson for the third time (`feedback_testing_discipline.md`): the replay must reconstruct
the live node's inputs, not a convenient proxy, and the fixture is only as honest as the
replay. The fix keeps P6's purpose (reject a frozen cache: state/error transitions are
on-change and prompt) and derives the age bound from the firmware cadence — one forced
period plus the stagger and jitter, 1.5 s — while the bridge's own telemetry-age and gap
checks keep the link-silence case. The replay now reconstructs the age from the last CHANGE
of the cached diagnostic tuple, which over-estimates it (a forced frame with identical
content is invisible), so the replay can only under-fire relative to the robot. A faster
hand diagnostic in firmware (axis 6 at 20 Hz) would let the bound tighten; it is not needed
for the detector to work and was not done.

**"Active at all times" meant: for every command source.** The streamed lane's echo and the
ACTIVATE park already wrote the command cache; `HAND_MOVE_TO` did not, so a pinch during a
bench descent (or during the jam recovery's own lower) had no command to compare against.
Writing the TRAP_TRAJ target as the command makes P3 true for the whole descent (the hand is
above its target); P1 — "the lane was descending" — then needs an alternative the command
cannot supply mid-move, and the encoder supplies it: the hand itself moved down at more than
`descend_vel_rps` inside the last 0.3 s. Enumerated against the false-positive classes: a
catch is moving at low current (P2, P3, P4), the bottom stop is outside the band and its
command is never 0.5 rev below it (P5, P3), the throw's pre-stroke dip is moving (P2), a
ball landing on the cup is shorter than the sustain.

**Why the feed miss could not end the pattern, and the contract that fixes it.** The cup
sensor is one bit for one cup; the schedule has two balls. The executor's release evidence
was written for one ball in the cup at a time (R3), where "SEATED then EMPTY after release"
can only be this ball leaving. In columns, the other ball lands in the same cup 0.34 s after
this ball's release and chatters for 20–80 ms as it settles — inside the 0.5 s grace window.
The invariant that survives two balls: possession confirms a release only through a
SEATED-then-EMPTY pair whose SEATED side lies within this ball's own carry (at or before
t_release + 0.10 s — the raw bit falls 0.045–0.057 s after a real release, and the other
ball cannot arrive before +0.34 s). On its own that turns every missed feed into the
"drop at throw 2's deadline" case, which still costs one empty beat because the dropped
ball's next catch-and-throw is dispatched at release + 0.27 s, before the +0.5 s deadline.
Ending earlier needs the miss itself as evidence, and the sensor gives it: no SEATED sample
anywhere in [the previous release + 0.10 s, this ball's release + 0.12 s] — a SAMPLE-complete
count over a ring buffer, because the tick-sampled read is exactly the lottery above — with
a liveness floor (≥ 80 % of the expected 100 Hz samples, no gap over 50 ms) and a veto on
any single SEATED reading (a bounce that settles, a late real catch: feeds seat by +0.272 s,
JB catches by +0.358 s max, 92–142 ms inside the window). Replayed on the sitting the rule
fires on 14/14 misses and 0/8 caught feeds (A1's analysis counted 13 because attempt 2's
feed, ended by the guard latch, was outside its table; the bag-decoded fixture includes it). It decides at about +0.39 s after the feed
landing, 0.145 s before ball B's next catch would dispatch, so a missed feed costs three
throws (one empty) instead of six (three empty plus two empty catch dives at P1). It does not
remove the jam exposure: the survivor's own throw and catch still stroke over the loose
ball. Stripping ball A's already-dispatched throw contradicts the 2026-09-29 "make the first
throw after a catch" rule and was left to the owner.

**Why the catches bounce.** The owner's reading — "the hand slightly too early or late" —
does not survive the mocap: contact is within ±10 ms of the plan everywhere, and the bounce
does not move with timing at all. It moves with where the ball lands on the cup. Jugglebot's
own throws land 4–6 mm off axis and rebound 9–17 mm (the designed 2.5 m/s relative impact of
the τ 0.125 s profile — the cup meets the ball at 0.39× its speed); Ball Butler's land 41 mm
off axis on the rim side toward ball A and rebound 143 mm, and none of them is seated when the
hand reverses for the throw stroke 0.19 s later. Two mechanisms stack for the feeds — the
lateral miss and the 47 % higher impact speed (the ball falls from ~1.6 m) — and this data
cannot separate them because every feed had both. The bias is Ball Butler's, not the
cup's: the cup sat exactly on the request point, the number survived the QTM re-alignment,
and the one-ball reload feeds at P2 show a different (smaller, +y) error, so it is an
aim-point-dependent calibration error in Ball Butler. The cheapest lever is to cancel it at
the request: ask Ball Butler for aim − bias and plan the catch at announced + bias, keep the
cup site where it is (no collision margin spent), and re-measure every sitting with the
promoted probe. The dwell is the second lever (0.1 s more would cover the 229–270 ms late
seats), but at this geometry more dwell is less transit (§ next paragraph), so it waits for
the geometry decision. τ stays: 0.100 s barely changes the impact and the catch is already
the limit hotspot.

**Why the planner refuses what the box certified.** The admissible box certifies a fold at
the nominal landing time with a 10 % margin; the live catch is planned for the tracker's
fitted landing time, and the schedule's next release is a fixed instant. Every millisecond
the fit is early comes out of the site-to-site transit; every millisecond late comes out of
the catch-to-release dwell. At the flown point the margin is gone at −20 ms and a refusal
appears at ±20–30 ms — and the live fits scatter that far at dispatch (the catch is planned
~0.5 s before landing, near the ball's apex). The box's landing-xy band was never the binding
dimension here: the x-band is ±2–4 mm because the SAME fold is already at the edge, and the
20 mm lateral authority (which the catch needs — see the bounce paragraph) sits outside it.
More dwell does not help: in two-ball columns the transit is (flight − dwell)/2, so 0.30 and
0.33 s refuse `LIMIT_ACC` even at dt = 0. The levers that remain are geometry (apex up for
more flight, separation down for less transit — against the collision clearance) and the
leg acc/jerk ceilings — which are YAML ceilings today, and the legs were already on the 10 A
current clamp in transits (sitting 3), so the clamp would have to rise with them: an owner
hardware decision, not a parameter. Whatever is chosen, the box sweep needs a landing-time
jitter dimension (±25 ms) so the next geometry is certified against the thing that actually
refuses.

**Why the hand guard tripped on a normal stroke, and what to trade.** The guard compares the
500 Hz command against an extrapolated encoder estimate and latches on a single tick with no
regard for how old the estimate is. A 100 ms frame gap during a 1700 rev/s² stroke puts the
extrapolation 3 rev from the truth. Counting the residual only when the anchor is fresh
(≤ 30 ms; healthy is 1–10 ms) removes the false trip; extending the 150 ms staleness
suppression to axis 6 adds the guard the hand never had (a hand streaming blind past 150 ms
stops instead of extrapolating). The trade: a genuine runaway that coincides with a dropout
is unguarded for the gap's length — measured ≤ ~100 ms across six bags, hard-bounded at
150 ms. The owner's 2026-07-16 ceiling note already names MAX_DEVIATION as the command-space
backstop behind the kinematic gate and the clamps; this keeps it honest rather than weaker.
A bridge reboot before the next sitting is not warranted: the rate is the same at 3 h and
94 h and the leak counters are zero.

**Hypotheses withdrawn.** (a) "The hand is early or late": no — ±10 ms, uncorrelated with
bounce. (b) "0/N encoder frames = a complete 300–800 ms encoder dropout" (the orchestrator's
first reading of the log): no — a 1 Hz census artefact; real gaps ≤ 100 ms. (c) "Cumulative
per-cycle drift across a fold chain" (A3): not needed — a single catch's landing-time offset
reproduces every refusal. (d) "The level trim shifted the landings": unconfirmed, confounded
by the learner moving the command 15 mm in the same session; a clean A/B needs the learner
frozen. (e) "Tighten the 20 mm lateral authority to the box band" (A3): rejected, because
the lateral miss is what makes the catches bounce.

## Fix (landed 2026-10-04 evening)

Nine units (five analyses, then J1/J2/B1/B1b/B1c/B2/B3/B4), briefs and the saved reports under
`temp/reports/r5_sitting4/`.

1. **Hand-jam detector reaches the robot** (`motion/hand_jam.py`, `teensy_bridge_node.py`,
   `tools/probes/hand_jam_replay.py`, `tests/motion/fixtures/hand_jam_20261002.csv`).
   `JamConfig.max_age_s` 0.05 → 1.5 s, derived from the firmware cadence (one forced period plus
   the stagger and jitter; `validate()` bounds it at 5 s); P1 gains the hand's own measured descent
   (`vel_meas < −stall_vel_rps` inside `descend_window_s` — 1.5 rev/s, not the commanded
   alternatives' 1.0, because a −1.42 rev/s encoder noise sample on the 2026-10-02 bag fires at
   1.0, the same creep band `stall_vel_rps` was raised for); `teensy_hand_move_to` and its
   `_nowait` twin write the TRAP_TRAJ target into `_last_hand_cmd` before the RPC (the target IS
   the command; the recovery's own moves pass through here too, correctly). The replay
   reconstructs the diagnostic age pessimistically from the last change of the cached motor-6
   diagnostic tuple, carries a per-bag `EXPECTED` table (the 2026-10-02 latch and its three
   un-latched sub-fires, the bench pinch, attempt 16, R2), defaults to the six bags, and
   regenerates the fixture (`--fixture-window`, `--fixture-out`). Tests: the 1 Hz-cadence
   regression (`test_fires_through_the_one_hertz_diagnostic_cadence`), the age boundary moved
   to 2.0 s, three measured-descent cases, the bridge's command-cache write.
2. **`MISSED_CATCH` and the release-evidence seat gate** (`motion/skills/executor.py`,
   `skill_node.py`, `tools/probes/missed_catch_fixture.py`, `tests/motion/fixtures/
   missed_catch_20261004.csv`). `RELEASE_SEAT_EPS_S = 0.10`: a SEATED reading arms a release
   only at or before `t_release + 0.10 s` (the one-line gate in `_advance_release_evidence`).
   `_PendingCatch` registered when a CATCH is accepted (never for a phantom);
   `_advance_catch_evidence` runs each tick before `_advance_release_evidence`; the window is
   [the latest release of any ball before L + 0.10, min(own carried release + 0.12, L + 0.50,
   the other ball's next landing − 0.10)], decided at `t_close + 0.03`; the verdict is a
   SAMPLE-complete count over `skill_node`'s 2 s `/hand_telemetry` ring buffer
   (`_seat_window`), RAW bit only (the debounced bit lingers ~0.14 s after the previous
   release and vetoed 14/14 real misses until B1c), with a liveness floor (≥ 80 % of the
   expected samples, no gap > 50 ms); any raw SEATED sample vetoes; a fire drops the ball's
   own pending rows, keeps the D3 survivor (`_install_survivor_tail` now also removes the
   dropped ball's undispatched skills — this fixes the pre-existing
   `DROPPED_SURVIVOR_STOPPED` path too), or stops; `done` waits for open catch windows.
   The fixture is a bag replay (3229 rows, 22 feeds, 14 miss / 8 caught; attempt 2's feed
   was a miss — its +0.68 s "seat" is Ball 1 arriving) carrying each attempt's real window
   instants (`t_open_rel_s` ≈ −0.20, `t_close_rel_s` 0.39). The sim's possession model is
   per-ball and instantaneous, so `seat_window` stays `None` there (rule off in sim —
   an open item). The phase audit found that `done` waits on every open catch window
   while the window was only visited while the attempt was live, so a later skill's
   refusal would have hung the goal forever; the window is now visited after the end
   too (resolving the row without dispatching), pinned by
   `test_an_attempt_ended_by_a_later_skill_still_resolves_an_open_catch_window`, and
   the ring buffer is locked on both threads.
3. **`columns_1ball_fed`** (`schedule.py` docstrings, `executor.py`, `skill_node.py`,
   `sim/skills_gate.py`, `Juggle.action`, `state-minimap.js`). `Pattern.phantom_balls=(0,)`
   through the fed path (`_run_columns(..., reload=True, phantom_a=True)`, `reload=False`
   refused: Ball Butler is the only feed); the dispatch ladder asks for ball evidence only
   for a real ball's THROW; a phantom is never a D3 survivor; the opening bridge holds no
   ball. Sim `--one-ball-fed` (the seed loop's attempt floor is 3, not 2: Ball 2 sits at the
   odd index and an even `n_throws` makes its only throw the row-less Stop).
4. **`columns_feed_bb_bias_mm`** (`skill_node.py`, `sim/skills_gate.py`,
   `tools/probes/feed_lateral_miss.py`). `bias = landing − request`, the same vector in the
   mocap and schedule frames (the two differ by a pure translation), +x toward Ball 1's site;
   on the fed path only (`ctx.kind == 'columns'`, which `columns_1ball_fed` shares): the point
   sent to Ball Butler is `aim − bias`, the announced landing becomes the prior as
   `announced + bias`, one INFO line names all three; clamp ±60 mm with a WARN; the reload
   and hop paths are untouched (tests). The probe measures ball − Platform-body xy at the
   free-fall 830 mm crossing and prints `RECOMMENDED columns_feed_bb_bias_mm = [...]` for the
   bias in force; L3 gives `[+25.8, +27.3]` (n 14), L4 `[+33.3, +23.2]` (n 1). The sim's
   `--bb-bias-mm` displaces the physical landing from the announced one; it is wired and
   inert at 0, but the clean sim re-aims the catch from its own fit and so does not reproduce
   the hardware's 41 mm miss (see Open Questions).
5. **Can-bridge FW 27** (`canbridge_config.h`, `leg_interp.cpp/.h`, `fault_machine.cpp`,
   `teensy_link/rpc_args.py`, `tests/firmware/`). `HAND_DEV_FRESH_US = 30000`: a hand residual
   over 2.5 rev counts toward the MAX_DEVIATION latch only when the encoder anchor's raw age is
   ≤ 30 ms (else `s_hand_dev_stale_skips`, on the `[hand7]` console line only — no wire change,
   PROTOCOL_VERSION 9); the recoverable `MOTOR_FB_STALE` suppression now covers axis 6 (lane
   active, stream active, axis seen, feedback older than 150 ms → the shared output gate
   closes, the hand ramps back from the live encoder on recovery). Native tests: attempt 2's
   numbers at 30.001 and 99.9 ms do not latch and count a stale skip; the same residual at 5
   and 30 ms latches on one tick; a 200 ms-stale hand suppresses output and recovers; two
   negative controls; mutation check (gate forced true → the attempt-2 test latches). Flashed
   21:44 (`pio run -e teensy41 -t upload`: SUCCESS, 284672 bytes, booted). The post-flash
   identity sniff was refused by the session's permission classifier, so the receipt is the
   bridge's `BRIDGE_FW_CHECK: OK — can-bridge v27` at the next launch (runsheet row 8).
   Python twin `hermite_xref/teensy_interp.py` not updated (follow-up).
6. **Ball names** (`schedule.ball_label`, every operator-facing line in `skill_node.py`,
   the executor's evidence lines, the sim gate's verdict lines; parsers accept both spellings
   for pre-2026-10-05 logs). Plan § 0 carries the glossary.
7. **Runsheet** `tests/hardware/session_skills_r5_sitting5.md`: bring-up (FW 27 receipt, the
   trim, the honest bench check), the fed half alone with the bias measured then applied,
   Ball 1's half, fed columns with `MISSED_CATCH` live, the optional trim A/B with fresh
   `plant_id`s, the dropout watch items.

## Verification

- Jam replay (`python tools/probes/hand_jam_replay.py`, six bags, 2026-10-04 22:0x,
  `temp/probes/hand_jam_replay_20261004_final.log`): **PASS across 6 bags** — fires at
  1791102420.349 (bench, 70 ms after the stall), 1791104785.346 (attempt 16), 1791105152.939
  (R2, 140 ms after the clamp), the 2026-10-02 latch within 0.3 s before its report and its
  three sub-fires; no fire on the level bag or the 2026-10-02 18:47 bag.
- Missed-catch fixture replay (`pytest tests/motion/test_skills_executor.py -q -k
  missed_catch`, 2026-10-04): 22 attempts, **14/14 misses fire, 0/8 caught feeds fire**.
- Fed columns sim (`python sim/skills_gate.py --learn --no-viewer --pattern columns --apex-m
  0.95 --separation-mm 125 --target-throws 30 --feed-angle-deg 11.9 --feed-speed-mmps 5600`,
  2026-10-04, `temp/logs/skills_gate_columns_feed_b1_20261004.log`): **PASS 5/5** (32/32
  makes, 0 drops, association OK). One-ball fed (`--one-ball-fed`, same geometry, seeds 0–4,
  `temp/logs/skills_gate_columns_1ball_fed_20261004.log`): **PASS 5/5**, 0 drops, wrong-ball
  rows 0. Bias knob (`--bb-bias-mm 29 25` and `0 0`, 3 seeds each): PASS 3/3 both.
- Firmware natives (2026-10-04): `test_fault_machine` 27/27 cases (540 assertions),
  `test_leg_interp` 47/47 (7698); `pio run -e teensy41` SUCCESS 11.5 s (text 249152, data
  35520, bss 111200).
- Scoped gates after each unit (2026-10-04): `test_hand_jam.py` 54/54, the bridge jam test
  13/13, `test_skills_executor.py` + `test_skill_node.py` 366 passed, `test_skill_node.py`
  202 passed, `test_skills_gate.py` 30 passed; intermediate full gates
  (`./run_tests.sh`, `temp/logs/gate_{b1,j1_jam_fix,j2}_20261004.log`): PASS 5946 passed,
  9 skipped + serial 3 passed.
- Full gate before the audit (`./run_tests.sh --full`, run 2026-10-04 23:25-23:31,
  `temp/logs/gate_full_r5_sitting4_fixes_20261004.log`): PASS — 6001 passed, 9 skipped,
  1 xfailed in 298.10 s; serial 6 passed. Final gate after the audit fixes
  (`./run_tests.sh --full`, run 2026-10-04 23:44-23:50,
  `temp/logs/gate_full_r5_sitting4_fixes_20261004b.log`): **PASS — 6002 passed, 9 skipped,
  1 xfailed in 308.65 s; serial 6 passed in 21.96 s.**

## Open Questions

1. **The refusal margin (the one thing not fixed).** Two-ball columns at apex 0.95 / separation
   125 / dwell 0.27 / 350/5000/200000 refuses a catch fold whenever the fitted landing time is
   20–30 ms off the schedule, which it is on about a third of the chains. More dwell is less
   transit. The levers: (a) a `t_land` jitter dimension (±25 ms) in `tools/admissible_sweep.py`
   so the box certifies what actually refuses, then (b) a geometry with margin (apex up, or
   separation down against the collision clearance) or (c) higher leg acc/jerk ceilings in
   `hardware_config.yaml` — which only help if the leg current clamp (10 A, saturated in
   sitting 3's transits; leg 4 trailed its command honestly today) rises with them. (c) is the
   owner's hardware call; (a) is the next software unit either way.
2. **The feed's impact speed vs its lateral miss** cannot be separated on this data (every
   feed had both). After the bias correction, ~10 centred feeds decide it: rebound still above
   ~40 mm means the 3.75 m/s impact, and the lever is a lower Ball Butler apex or a
   feed-specific cup velocity match. The seat-before-reversal criterion (≥ 70 %) failed on
   every feed regardless of bias (1/14, 0/1); 0.1 s more dwell for the feed catch would cover
   the 229–270 ms seats but costs transit (question 1).
3. **`MISSED_CATCH` is off in the sim** (per-ball instantaneous possession, no cup-level
   sample series) and the sim does not reproduce the bounce failure from a lateral-bias
   injection (its executor re-aims from a clean fit). A tracker-correction-disabled mode, or a
   cup-level possession model, would make both rehearsable.
4. **The level trim's effect on the landings is unconfirmed** (the learner moved the command
   15 mm in the same session); runsheet § 6 is the clean A/B with fresh `plant_id`s.
5. **The hand dropouts' source** stays with `plans/active/leg-bus-frame-drops.md` workstream B;
   FW 27 tolerates them. `MOTOR_FB_STALE` carries no axis index on the wire (which axis went
   stale is `/cache_diag age_max_us_*`). The Python twin of the hand guard lacks the gate.
6. **Ball Butler's bias is aim-point dependent** (P2 reload feeds show a different, smaller
   error): one parameter per feed site is a patch; the durable fix is Ball Butler's own
   calibration, or a learner for its aim like the throws'.
7. The jam recovery itself has still never run on the robot (sitting 4's bench check never
   reached it); runsheet § 2 is the first honest test. Stripping Ball 1's already-dispatched
   throw after a missed feed (one fewer stroke over a loose ball) contradicts the 2026-09-29
   "first throw after a catch" rule and waits for the owner.
