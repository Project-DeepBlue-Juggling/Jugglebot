---
title: "R5 sitting 3 (2026-10-02 evening): every fed columns start dies at the fourth skill because the tracker correlation hands one ball the other ball's flight, the throws are wide from the ball in the cup and a frozen learner intercept rather than a tilting platform, and 26 mm of column clearance cannot hold 30-85 mm of scatter"
type: investigation
date: 2026-10-04
status: in-progress
phase: "two-ball-skill-stack — R5"
related_plan: two-ball-skill-stack.md
related_entries:
  - 2026-10-02-skill-stack-r5-sitting-2.md
  - 2026-09-30-skill-stack-r5-sitting-1.md
files_changed:
  - ros_ws/src/jugglebot_interfaces/msg/BallState.msg
  - ros_ws/src/jugglebot_interfaces/action/Juggle.action
  - ros_ws/src/jugglebot/jugglebot/ball_possession.py
  - ros_ws/src/jugglebot/jugglebot/ball_tracker_node.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/hand_jam.py
  - ros_ws/src/jugglebot/jugglebot/motion/levelling.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/learner.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_activate.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_activate.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - ros_ws/gui/js/state-minimap.js
  - ros_ws/docs/choreography.md
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - config/generated/admissible_box.yaml
  - docs/teensy-udp-protocol.md
  - teensy_link/protocol.py
  - teensy_link/rpc.py
  - teensy_link/rpc_args.py
  - sim/skills_gate.py
  - tools/probes/hand_jam_replay.py
  - tools/probes/level_vs_ballfit.py
  - tools/probes/README.md
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - tests/hardware/skills_plan_bench.py
  - tests/hardware/session_skills_r5.md
  - tests/hardware/session_skills_r5_sitting4.md
  - tests/firmware/native/test_leg_activate.cpp
  - tests/firmware/native/test_rpc_dispatch.cpp
  - tests/firmware/test_bridge_fw_version_xref.py
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/motion/fixtures/hand_jam_20261002.csv
  - tests/motion/test_hand_jam.py
  - tests/motion/test_learner.py
  - tests/motion/test_levelling.py
  - tests/motion/test_skills_executor.py
  - tests/motion/test_skills_schedule.py
  - tests/ros/conftest.py
  - tests/ros/test_ball_possession.py
  - tests/ros/test_ball_tracker_identity_wire.py
  - tests/ros/test_levelling_frame.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_skill_node_resend_param.py
  - tests/ros/test_skills_plan_bench.py
  - tests/ros/test_teensy_bridge_node_hand_jam.py
  - tests/ros/test_trajectory_node.py
  - tests/ros/test_two_ball_association.py
  - tests/sim/test_skills_gate.py
  - tests/teensy_link/test_hand_move_to.py
  - plans/active/two-ball-skill-stack.md
  - .gitignore
subsystem:
  - motion
  - ros
  - tracking
tags:
  - skill-stack
  - columns
  - ball-butler
  - tracker
  - learner
  - hardware-sitting
---

## Summary

The owner flew two sittings of `tests/hardware/session_skills_r5_sitting3.md` on the
evening of 2026-10-02 (launches 18:11 and 18:47; bags `2026-10-02_18-11-14` and
`2026-10-02_18-47-03`). The swapped columns layout from sitting 2 worked as designed:
Ball Butler's feed no longer crosses ball A's column, the feed catch is no longer refused,
and fed columns reached its third throw on most attempts. It never reached a fourth.
Four questions came back: why do throws look less vertical than at R3, why does every
columns attempt end `WINDOW_TOO_SHORT` at about the third throw, why do the second and
third throws collide, and what would keep the Platform horizontal during the stroke.

Five parallel analyses of the two bags and launch logs (reports preserved under
`temp/reports/r5_sitting3/`) answered them:

1. **`WINDOW_TOO_SHORT` is a software defect, on 21 of 21 fed columns attempts.**
   `skill_node` correlates tracker flights to schedule balls one latch per ball, and a
   latch resolves about 50 ms before its own release. At that instant its own ball is
   still `TO_BE_THROWN` while the other ball, thrown one beat earlier, is `IN_FLIGHT`,
   and nothing excludes an id the other ball's latch already claimed. Skill 3 (ball B's
   catch at P1) therefore aimed at ball A's fitted landing time, which was already in the
   past at the splice: `-0.23 s` is A's landing (`0.400 + 0.857 s` on the plan clock)
   against the splice at knot 60 (`1.500 s`), `-0.243 s` plus A's lateness. The same mis-latch credits throw N
   with throw N-1's flight (`release -580 ms`, landing shifted by the 100 mm site
   separation) and appended 33 bogus rows to the learner memory. Replaying the real
   `ball_possession` functions over the bag's `/balls` reproduces the wrong latch on
   every attempt examined. The sim rehearsal (`sim/skills_gate.py`, PASS 5/5 on
   2026-10-02) cannot see it: its tracker stand-in is keyed by the true schedule ball.
2. **The throws are not wide because the Platform tilts.** Mocap and leg-encoder FK agree
   on the Platform attitude to about 0.05°, it holds within about 0.2° through the
   stroke, and tilt at release explains none of the landing scatter (R² 0.00, n = 63
   self-tosses). What moved since the R4 gate: the per-axis landing spread grew from
   about 17 mm to about 23 mm; the best single predictor is the ball's offset from the
   cup axis at release (2 to 4 mm of landing per mm, the same sign in all sittings); the
   learner's y command flipped sign (u_y about +4 mm today against about −5 mm at R4,
   with the plant landing about +11 mm in y beyond the command); and the `level`
   reference leans about 0.3° toward +y against gravity measured from free-flight ball
   fits, which was already true at R4. The first throw after a Ball Butler feed is not
   materially worse than later throws (about +12 mm further in y).
3. **The collisions are geometric.** Two 74 mm balls in columns 100 mm apart leave 26 mm
   of surface clearance. The balls pass 490 mm up, 0.139 s before the falling ball
   lands, where it still carries 84 % of its landing error. The 20 mm feed aim toward A
   chosen in sitting 2 uses 17 of those 26 mm on the pass right after the feed. The feed
   catch itself enters the cup 30 to 40 mm off-centre, open-loop, because the tracker
   fit never converges before the resend deadline. Ground truth from raw mocap: 2 of the
   6 attempts examined had the balls come within 74 mm (42 and 58 mm).
4. **The actuators have velocity headroom but no acceleration headroom.** Legs 1, 2 and 4
   sit on their 10 A current clamp during every fast transit (48 episodes, median 50 ms),
   because the velocity loop is chasing about 25 ms of lag, not because of inertia. Peak
   commanded leg speed was 247 mm/s against the 300 mm/s limit. The one E-STOP was the
   hand, not the legs: a ball pinched between the funnel's rigid ring and the underside
   of the descending hand (owner's explanation), the hand stalled at 2.83 rev on its
   current clamp and tripped the 2.5 rev deviation guard.

Fixes approved by the owner on 2026-10-04 and landed in this entry: the two-ball
association contract (identity key on the wire, one claimed set, a time gate in the
executor, a faithful sim rehearsal), the learner's local fit regularised toward the
identity prior with the 33 bogus rows quarantined, a `level` trim parameter with the
probe that measures it, a single-ball columns mode for testing vertical throws under
transit, a jam-recovery behaviour for the hand, and the roomier geometry (apex 0.95 m,
dwell 0.27 s, separation 125 mm, leg velocity 350 mm/s, feed aim 10) with the admissible
box re-swept.

## What the owner reported

Verbatim intent, condensed: the swapped layout performs much better but columns never
passes 2 to 3 catches; throws are not reliably straight up, even in self_toss, and look
worse after a Ball Butler feed despite the hold; `WINDOW_TOO_SHORT` at about the third
throw; the second and third throws collide, which "screams inaccurate throws"; a proposal
to refine each step separately (BB feeding, JB self-tosses, JB columns with one ball);
disappointment that self-tosses look no more reliable than at R3; openness to raising
hardware limits. On 2026-10-04 the owner added: the ball has a unique seat in the hand
and finds it quickly; the E-STOP was a ball trapped between the funnel's rigid ring
(coplanar with the top of the hand at the bottom of its stroke) and the hand, and a
check that raises the hand slightly to let the ball fall through would be welcome; the
robot may have been bumped between the two sittings.

## Measured

Ledger (`temp/reports/r5_sitting3/u3_ledger.md`, both launches): 61 Juggle goals, of
which 27 self_toss, 26 columns, 3 hop, 5 refused before starting. Columns: 21 of 26
ended `WINDOW_TOO_SHORT` at skill 3, every one with a −0.20 to −0.25 s window from the
knot 60/61 splice; the other 5 ended at the start (`ABORTED_BB_THROW_TIMEOUT` ×2,
`REJECTED_BB(THROW_ABORTED_NOT_SETTLED)`, `REJECTED_NO_BALL`, `GUARD_LATCHED`).
Self_toss: `ABORTED_NO_RELEASE` after a miss (the empty-hand throw) in most attempts,
`REJECTED_NO_BALL` at the first throw after 6 of 33 feeds in the first launch (the feed
catch failed or the ball was not sensed; the launch log cannot tell which), one
`LIMIT_JERK` at the closing REST (219k > 200k after a re-aim run).

| Self-toss accuracy | R4 gate (2026-09-29) | Sitting 3 |
|---|---|---|
| Mean landing offset, clean rows | 23.0 mm (sd 15.2, p95 45.1, n 106) | 49.2 mm (sd 67.1, p95 136.8, n 144) |
| Per-axis landing sd (U2) | ~17 mm | ~23 mm |
| y bias at the site | +8.7 mm | +13.4 (A), +14.4 (B) mm |
| Learner u_y | −5.4 mm | +4.2 (A), +2.1 (B) mm |
| Catch lateral authority | 20 mm | 20 mm |
| Implied capture radius | | 35 to 40 mm (caught 91 % at 20-40 mm, 44 % at 40-60 mm) |

The catch reach was already pinned to 20 mm at the R4 gate (history: authority 0 on
09-16, 40 mm on 09-21, 20 mm on 09-23 after Block B dropped balls at 40 mm), so the
regression is in the throws, not the catch. Of 36 real misses with a clamp event, 14
were within 40 mm of the site; a 40 or 60 mm authority reaches the same 14.

Throw decomposition (`u2_throw_verticality.md`, sitting A, n = 63 self-tosses):
platform tilt at release R² 0.00 in x and y; ball-in-cup offset R² 0.22 (x) / 0.31 (y)
with slope −2.2 to −4 mm per mm; leg-tracking lean at release (−0.12, +0.03)° against
(−0.04, −0.02)° at R4, worth about +5 mm in x; platform lateral velocity at release
(+4.6, +2.7) mm/s; the QTM z axis 0.6° off gravity, unchanged since R4 (biases short-arc
tracker fits by 10 to 17 mm, does not move a landing); hand stroke peak about 2000 to
2200 rev/s², the same as R4 and well under either cap, so the 3900 cap changed nothing
in self-toss. The tilt calibration map is not loaded on this path (deliberate,
1b187e8c); only the `level` gravity offset applies.

Feed (`u4_feed_and_columns_truth.md`): landing +94.2 ms late (sd 143.2, n 31) in A and
+183.6 ms (sd 222.7, n 10) in B against +55.3 ms (sd 24.2) at midday; several +600 ms
outliers are a known ball/arm mistracking artefact. Cup entry offset: self_toss-site
feeds mean 28.9 mm (sd 21.0, n 10), columns-site feeds mean 41.4 mm (sd 34.8, max 140,
n 16). Every feed catch logged `AIM-LATERAL-CLAMPED` then `RESEND-SKIPPED
NO-CONVERGED-FIT`, so it ran on the schedule prior alone. Feed offset and timing do not
correlate with the first throw's landing error (r −0.27, −0.21, +0.09, n 10).

Clearance (`u5_limits_geometry.md`): at apex 0.9 / dwell 0.3 the balls pass 0.139 s
before the falling ball lands, 490 mm up; 52.6 mm of pass error per degree on the falling
ball, 10.2 mm per degree on the rising one. With the 20 mm catch authority, collision-free
passes at separation 100 are 92 / 85 / 80 % for launch sigma 0.3 / 0.5 / 0.8°; 95 % per
pass needs 104 / 119 / 138 mm; a 95 %-clean 30-throw run needs 128 / 156 / 198 mm.
Planner demand for the steady catch-with-throw (vel / acc / jerk): 256/4164/163k at
sep 100, 321/5461/161k at 125, 392/6812/171k at 150; a 20 mm re-aim away from the
other site already needs 318/5201 at sep 100. Legs: peak commanded 247 mm/s and about
4083 mm/s², tracking error peak 10.4 mm (15 % of the 70.5 mm deviation guard), legs
1/2/4 at the 10 A clamp on all 24 fastest transits, saturation ending 250 to 395 ms
before release and worth about 0.05 to 0.1° of tilt at release.

Nightly: the 04:00 run of 2026-10-04 is RED in the MAIN checkout
(`tests/firmware/test_bb_fw_update_xref.py`: BallButler `FW_VERSION` 5 against that
branch's `BB_FW_VERSION_EXPECTED` 4). That checkout is `mvp-trajectory-bringup`, behind
this branch, and the mismatch is the BB FW 5 flash of 2026-09-30 seen from the old
expectation. Not a regression of this branch.

## Discussion

**The platform-tilt framing was withdrawn.** The owner's question and my first framing
were "keep the Platform horizontal during the stroke". The bag says the Platform is
already horizontal to 0.2° through the stroke and that its tilt at release carries no
information about where the ball lands. A tilt feedforward during the stroke would be
worth about 6 mm and would not touch the scatter. The error is set inside the cup and by
the aim: the ball's position relative to the cup axis at release is the strongest
measured predictor, the learner's y command has the wrong sign, and the level reference
leans against gravity. The owner confirmed the ball has a unique seat it finds quickly,
which makes the in-cup term more likely to be the ball leaving the seat during the 6 to
7 g stroke than a seating problem before it; that could not be measured with the
markers available and stays open.

**Why the association bug waited for columns.** Every earlier pattern had at most one
Jugglebot-thrown ball in flight. The per-ball latch with a `preexisting` snapshot was
correct for one ball and for a feed; two Jugglebot balls one beat apart defeat it
because the snapshot is taken when the other ball's flight already exists and the only
timing rule ("lands after this ball's previous release") passes the wrong flight. The
fix is a contract rather than a patch: an estimate is used only for the release it was
announced for (identity carried on the wire as the announcement's `throw_time`), one
claimed set across both balls, and a single physical gate in the executor
(|t_land − scheduled| ≤ beat/3, else the schedule) that aim, re-aim,
release evidence and OUTCOME all pass through. The sim rehearsal is made faithful by
feeding it one-id-per-announcement records through the same correlator, so a pattern
that mis-latches fails the gate before it reaches the robot.

**Why the learner flipped.** The local affine fit has u_y almost constant across the
self-toss neighbourhood, so the fitted slope in y is noise and its sign decides the
command. Dropping the 33 bogus columns rows does not change the command (the 10 mm
kernel isolates the self-toss context), so the purge is hygiene, not the fix. The fix
regularises the local slope toward the identity prior with a strength tied to the
neighbours' spread in u: where the data cannot identify a slope, the slope is the prior.

**Why geometry before limits.** Clearance at separation 100 is smaller than routine
scatter, and the feed aim consumed most of it, so no amount of catch authority rescues
columns at 100 mm (a wider x authority also needs more leg acceleration than the box
allows and lowers the collision-free odds). Separation 125 at apex 0.95 and dwell 0.27
raises the clean-pass odds to about 96 % at 0.5° sigma for a 350 mm/s velocity limit
with acceleration and jerk unchanged. Acceleration is left alone because the legs are
already on their current clamp at about 4000 mm/s²; that headroom belongs to the
acceleration-feedforward plan (`plans/active/accel-ff-inertia.md`). Apex 1.0 is
hand-bound (94 % of the cap nominal, and the learner pushes higher).

**Feed catch accuracy is the next limiter and is not fixed here.** Once the association
is right, columns will meet the 30 to 40 mm feed entry offset directly: B's re-throw
from an off-centre catch lands toward A's site. The feed catch runs open-loop because the
fit does not converge before the resend deadline; letting the feed catch follow a later
fit, or widening its authority, is a separate unit for the owner's "refine BB feeding"
step.

## Fix (landed 2026-10-04)

Seven units, built by parallel agents from the five analysis reports and reconciled here.
Each names its invariant, its single enforcement point and the test that fails without it.

**U1 — the two-ball association contract** (`ball_possession.py`, `ball_tracker_node.py`,
`BallState.msg`, `skill_node.py`, `motion/skills/executor.py`). Invariant: *a tracker estimate
is used only for the release it was announced for; when the wanted ball has no valid estimate
the catch keeps the schedule.* `BallState` carries the announcement's `throw_time`; the tracker
copies it from the announcement it latched; `FlightLatch` gets a `thrower` and resolves only to
the track with that source and a `throw_time` within 2 ms of its own release
(`match_announced_track`), and `advance_correlation` resolves every ball's latches against ONE
claimed set, with the old `preexisting` snapshot kept as a backstop. In the executor one rule,
`tracked_landing_refusal`, gates catch aim, re-aim, release evidence and the OUTCOME row on both
time (|t_land − scheduled| ≤ beat/3; the brief's 0.2 s ceiling was dropped because a real row
arriving +0.204 s late on a one-ball pattern is exactly the bias the learner must absorb, and on
columns beat/3 is already 0.193 s) and position (200 mm, not 2× the 20 mm authority, because real
fits land 50 to 120 mm off and the timing would have been discarded on nearly every throw); a
refusal logs `TRACKER-IDENTITY-REFUSED` once per skill. Both nodes refuse to start on a stale
build (`require_identity_fields` names the two-package colcon command). The −830.8 mm feed-ball
reading (u1 § 6) is now refused twice, by source and by the gate. Tests:
`tests/ros/test_two_ball_association.py` replays attempts 263 and 275 from the bag with real
timestamps (HEAD's rule reproduces the wrong latch; the fix and the claimed-set-only variant latch
correctly), `tests/ros/test_ball_tracker_identity_wire.py` (round trip + stale-build refusal),
plus a two-ball correlation test in `test_skill_node.py` and three executor tests including a
real `compile_columns` run where each ball is fed the other's track.

**U1b — the sim rehearsal sees it** (`sim/skills_gate.py`, `tests/sim/test_skills_gate.py`).
`_IdentityTracker` mints one id per ANNOUNCEMENT, carries source and `throw_time`, flips
TO_BE_THROWN → IN_FLIGHT at the announced release and CAUGHT only after the physical ball was
seen airborne and landed, and resolves `tracker(ball_id)` through the imported
`ball_possession.advance_correlation` (never a copy); the Ball Butler feed ball is announced as a
`ball_butler` track. `_association_verdict` adds three pass criteria (no `WINDOW_TOO_SHORT`, no
row refused as another release's flight, every row's |release error| < beat/2).
`--correlator head` routes the same records through HEAD's rule as a negative control: the fed
columns run then refuses every throw past the first with a one-beat landing gap, and the fixed
rule gets every row with a 17.5 ms worst release error. Both are pinned as tests.

**U2 — the learner** (`motion/skills/learner.py`, plan § 2.5 revised). The analysis agent's guess
(a noisy slope sign) was wrong and the build agent said so: forcing the slope to identity still
gave u_y = +4 mm. The real mechanism is survivorship in the neighbour selection — the joint
(state, outcome) metric picks, among the 159 rows sharing the P1 state, the sixteen that happened
to land nearest the target, so the fitted intercept is the target by construction and the command
freezes wherever history left ū. Fix: the outcome bandwidth only GATES which rows count
(`support_d2 = 9.0`), the neighbourhood is the k most RECENT rows inside that gate (row order is
now part of the law; memory is appended oldest-first), ū keeps the paper's weights, the forward
fit uses state-only weights, and a query with no supported row returns the identity prior. On the
real memory the self-toss command moves from (−15.9, +4.1) to (−17.5, −4.5) mm. Tests in
`tests/motion/test_learner.py`: a 48-row real P1 fixture, a synthetic frozen-learner case,
identical-u gives D = I exactly, a well-spread neighbourhood reaches 88/79/76 % of the true slope
(55/58/63 % before); swapping the old law back fails four of them. One sim-gate assertion changed
with it: the self-toss apex-band test judges the MEAN signed apex error from band entry rather than
the last throw alone, because seed 0's throws 3 to 5 scatter (0, +26, −45) mm on one command and
the old law passed only because its +57 mm bias cancelled the −45 mm draw.

**U2b — the memory.** 41 wrong-ball rows quarantined into
`temp/learn/jugglebot/memory_quarantine_20261002.csv` (33 keyed by u1's purge list, 8 more from the
00:22 launch of 2026-10-03 with the same −560 ms signature), backup `memory_backup_20261004.csv`
(275 rows, SHA-256 checked), `memory.csv` now 234 rows; multiset and order verified.

**U3 — the `level` trim** (`motion/levelling.py`, `trajectory_node.py`,
`tools/probes/level_vs_ballfit.py`). `level_trim_deg [x, y]` (|trim| ≤ 1°, else WARN and 0) is
ADDED to the raw `/gravity_offset` before `correction_from_offset` negates it, so the trim that
cancels a measured lean L is +L in the wire's own sign — written down at length in both files
because the analysis report's "(−0.07, −0.30)" phrasing meant the applied counter-rotation. The
probe measures the lean from free-flight ball fits (gravity's UP is the negation of the fitted
free-fall acceleration; the probe's own first draft got that backwards and its docstring says so)
and prints the value to paste. On sitting A it gives (+0.090, +0.234)° over 80 throws against
u2's (+0.07, +0.30)° over 63 self-tosses. A trim of 0 is bit-identical (tested).

**U4 — `columns_1ball`** (`motion/skills/schedule.py`, `executor.py`, `skill_node.py`,
`Juggle.action`, the GUI dropdown). Invariant: *a phantom ball produces motion and nothing else.*
`Pattern.phantom_balls` is threaded onto the `Schedule` (motion byte-identical to columns, pinned),
`compile_reload_columns` reuses `compile_reload` for the lead-in; the executor's three enforcement
points are `_register_outcome` (no pending row → no evidence wait, no OUTCOME, no learner row),
`_tracked_landing` (never consults the tracker) and `_predicted_landing` (the phantom's first catch
gets a synthesised vertical self-toss landing instead of hanging on `NO_LANDING`); `_maybe_announce`
never announces it. Same box as columns (`'columns'` box kind, both site pairs).

**U5 — the hand-jam detector and recovery** (`motion/hand_jam.py`, `teensy_bridge_node.py`,
can-bridge FW 26 `HAND_MOVE_TO`, `teensy_link`). The bag showed the E-STOP never relieved the
pinch: the latch stops the stream but the hand ODrive keeps its last command, so it pushed about
57 N at the 50 A clamp (about 150 W) for 2.05 s until the ball gave way, and the 2.5 rev deviation
guard caught it by 0.027 rev. Two more pinches that sitting never latched at all (5.5 s at the
clamp, thermistor 27 → 49 °C, ended by the hand ODrive going IDLE on `DC_BUS_UNDER_VOLTAGE`).
Invariant: *a stalled hand at the clamp under a descending command is relieved before anything
else and never pushed through.* `JamDetector` (100 Hz on the bridge's RX thread, latch or no
latch): descending, |v| < 1.5 rev/s (1.0 in the design; a −1.24 rev/s creep sample in the fixture
reset it), command > 0.5 rev below the hand, |iq| ≥ 0.9 × limit, position in a provisional
[1.0, 3.6] rev band, axis healthy, all for 100 ms. `JamRecovery`: cut the hand current limit to
10 A FIRST (legal under a latch, about 14 N), the existing converge-first clear and disarm, raise
1.0 rev with `HAND_MOVE_TO`, dwell 0.6 s, lower at 10 A watching for a re-stall, attempt 2 raises
2.5 rev, else `HAND_JAM_UNRECOVERED` leaves the hand raised and never restores 50 A;
`HAND_JAM_RECOVERED` restores 50 A last, at rest. `/recover` resumes at the lower step; `/park_hand`
and arming are refused while a jam is held; `/hand_jam_dry_run` prints the live predicates and
plan; `/hand_move_to` (SetFloat) is the operator's bench handle. FW 26 adds `HAND_MOVE_TO` (0x61,
additive, PROTOCOL_VERSION 9 unchanged) through ACTIVATE's own ladder and gates with a deferred
terminal reply carrying the outcome; a second call RETARGETS the move (ACTIVATE's refuse-while-busy
would have IDLEd the hand onto the ball at the 10 s timeout), so the bridge checks `outcome`, uses
the no-wait client and preempts with a new move. Built (`pio run -e teensy41`, 800,824 B, md5
`8d5ab3d10d220fd20aec43e584fb91c8`), flashed 2026-10-04 16:34 after the commit (receipt: the board's
`BridgeIdentity` frame reads `fw_version=26, protocol_version=9`). Tests: 49 pure (fixture of the
event's `/hand_telemetry`, 96 rows: fires 103 ms before the latch; silent stall fires; every
false-positive row of the design does not; sequence order), 12 bridge (relief first, disarm before
raise, SUPERSEDED/retarget, `RpcTimeout` before `RpcError`, UNRECOVERED keeps 10 A), 7 native
firmware cases + 3 dispatch cases, `tests/teensy_link/test_hand_move_to.py`. The replay probe
`tools/probes/hand_jam_replay.py` passes on the three known stalls and nothing else.

**U6 — the geometry.** `skill_node._DEFAULT_DWELL_S` 0.30 → 0.27, `columns_feed_aim_toward_a_mm`
20 → 10 (the sim gate's and the plan bench's defaults follow), and the admissible box re-swept at
dwell 0.27, leg velocity 350, acceleration 5000, jerk 200000, hand 3900, columns separation 125
(apex rungs 0.85/0.90/0.95), hop at 250, self-toss at (−50, 0): see Verification. Apex 0.95 and
separation 125 are goal fields, set from the runsheet. The feed aim is 10 rather than 0 because
the undisplaced feed catch at 125 mm demands 98 % of the acceleration limit (u5), and 10 mm
spends 8 of the 51 mm of clearance. **The dwell went to 0.27, not the 0.25 the limits report
proposed**, and the box was swept three times for it: at 0.25 the one-ball trial refused
`LIMIT_JERK` on every sim seed and the jitter-free executor loop of
`tests/hardware/skills_plan_bench.py` (8 throws through the real planner, including the D2
Stop fold) refused 205 to 226 k mm/s³ against 200 k — the catch-with-throw has 50 ms less to
re-level for its throw — while at 0.30 the same loop is acceleration-bound at 125 mm
(4507 to 4813 mm/s² against 4500 at the sweep's 90 % margin). A (separation, dwell) scan at 90 %
of all three limits passes 115/0.27-0.29, 120/0.27-0.28 and 125/0.27 only; 0.27 keeps 10 % on
every limit at 125. The one-ball start also needed its first catch moved to the FED start's aim
point and level pin (`schedule.phantom_feed_prior`), and the one-ball reload's columns anchored
one launch window after the decay REST (`compile_reload_columns`): both measured in the sim,
both recorded at the code. Two sim-gate constants that restated the session limits as
literals (`SelfTossGateConfig.leg_vel_mmps = 300.0`, and the plan bench's own operating point)
now follow the session constants, which is the derived-literal lesson of 2026-10-02 biting a
third time.

## Verification

All runs 2026-10-04 unless stated. Console output by path under `temp/logs/`.

- **Admissible box**: swept three times today. At dwell 0.25 (runs S4a/S4b, bit-identical,
  discarded) and at dwell 0.27 (runs S4c 14:33 and S4d 15:00, `scratchpad/s3/sweep_run_s4.sh`,
  BLAS 1 thread, `--dwell-s 0.27 --leg-vel 350 --leg-acc 5000 --leg-jerk 200000 --hand-acc
  3900`, columns `--separation-mm 125 --single-apex 0.85 0.90 0.95`, hop `--single-apex 0.80
  0.85 0.90 0.95`, single `0.5..0.9`): the three S4c/S4d files `cmp` IDENTICAL (md5 columns
  `2661d0e3…`, hop `f52c3202…`, single `8477ea61…`), merged into
  `config/generated/admissible_box.yaml` (19 boxes, gate `458bb544b345` unchanged, md5
  `3c2087eff6e65bab0761c9f9d0912301`). Columns at 125 mm is admissible ONLY in the 0.925-0.975
  apex band (P2 x [−2, +4] y [−8, +20] mm; P1 x [−4, +2] y [−10, +20]); the 0.85 and 0.90 bands are
  empty, so `skills/check` refuses apex 0.9 at 125 mm by name. Hop at 250: x ±1 to ±8 by band;
  self-toss at (−50, 0): x [−40, +30] y [−40, +30].
- **Sim gates** (`sim/skills_gate.py --learn --no-viewer`, 5 seeds, dwell 0.27, session legs
  350/5000/200000, hand 3900; `scratchpad/s3/run_gates_s4.sh`, 15:03):
  - fed columns `--pattern columns --apex-m 0.95 --separation-mm 125 --target-throws 30
    --feed-angle-deg 11.9 --feed-speed-mmps 5600`: **PASS 5/5**, 30/30 makes, 0 drops, assoc OK,
    worst release error 16 ms (`skills_gate_columns_feed_d27_20261004.log`);
  - one-ball columns `--one-ball`: **PASS 5/5** (`skills_gate_columns_1ball_d27_20261004.log`);
    `--one-ball --one-ball-reload`: **PASS 5/5** (`…_1ball_reload_d27_…`);
  - unfed two-ball columns: **PASS 5/5** (`skills_gate_columns_unfed_d27_20261004.log`);
  - the negative control (`--correlator head`, pinned as a test) refuses every throw past the
    first with a one-beat landing gap;
  - hop `--pattern hop --apex-m 0.9 --target-throws 20`: **FAIL 0/5**, 20/20 makes and 0 drops on
    every seed but `band_xy` never entered (the pre-existing re-aim refusals, unchanged);
  - self_toss `--apex-m 0.9 --target-throws 20`: 20/20 makes, 0 drops, band entry by throw 3 on
    every seed, but seeds 1 and 2 fail the xy-monotone criterion at dwell 0.27 (they pass at
    `--dwell-s 0.30`, same run) — a dwell effect, recorded, not fixed;
  - self_toss `--reload`: seed 1 PASS, seeds 0/2/3/4 `LIMIT_JERK` at the closing REST (204 920
    mm/s³) — the same at `--dwell-s 0.30`; sitting 2 ran only seeds 0-1 (0 FAIL, 1 PASS), so
    this is the known closing-REST issue characterised over five seeds, not a regression.
- **`./run_tests.sh --full`** (15:23): **5970 passed, 9 skipped, 1 xfailed in 286.36 s; serial
  6 passed** (`temp/logs/gate_full_r5_sitting3_fixes_20261004b.log`); repeated after the audit's
  narrative fixes (16:22): **5970 passed, 9 skipped, 1 xfailed in 287.50 s; serial 6 passed**
  (`…_20261004c.log`). The first full run (15:0x,
  `…_20261004.log`) had 31 failures, all four classes fixture-side: the node tests' status and
  box fixtures pinning leg velocity 300 (now one constant), real-box columns tests at the old
  100 mm / 0.9 m point (now `_flown_columns_goal`), the levelling call-site census (the trim's
  four store-class ingests enumerated), and B2's own decay-anchor pin.
- **Per-unit scoped runs** (each agent's report, all 2026-10-04): association
  `pytest tests/ros/test_skill_node.py tests/ros/test_skill_node_resend_param.py -q -n 4 --dist
  loadfile` 193 passed; learner `pytest tests/motion/test_learner.py -q` 49 passed; levelling
  `pytest tests/motion/test_levelling.py tests/ros/test_trajectory_node.py
  tests/ros/test_trajectory_node_console.py -q` 178 passed; hand jam `pytest
  tests/motion/test_hand_jam*.py tests/ros/test_teensy_bridge*.py -q -p no:xdist` 522 passed;
  firmware `pytest tests/teensy_link tests/firmware -q` 713 passed, 1 skipped in 246.95 s, native
  `test_leg_activate` 27 cases / 199 assertions and `test_rpc_dispatch` 30 / 189, 0 failed;
  `pio run -e teensy41` SUCCESS 12.5 s, hex 800,824 B, md5 `8d5ab3d10d220fd20aec43e584fb91c8`,
  flashed 16:34, receipt `BridgeIdentity(fw_version=26, protocol_version=9)` on the stream port.
- **Probes**: `tools/probes/hand_jam_replay.py` over the four 2026-10-02 bags
  (`temp/probes/hand_jam_replay_20261002.log`): the latched event fires 103 ms before the latch;
  two more fires are the un-latched pinches at 1790929174.17 and 1790929358.65/360.83 (now the
  probe's known list), none elsewhere in 2 752 s of hand telemetry.
  `tools/probes/level_vs_ballfit.py` on sitting A: FK lean (+0.090, +0.234)° over 80 throws
  (`temp/probes/level_vs_ballfit_20261002A.log`).
- **Build**: `colcon build --packages-select jugglebot_interfaces jugglebot` twice (the message
  change, then the final edits; `temp/logs/colcon_build_r5_sitting3_fixes_20261004{,b}.log`).


## Open Questions

1. **FW 26 is flashed** (16:34, receipt taken), so the sitting-4 sheet's § 2 bench check applies in
   full. Standing rule from the owner (2026-10-04): a finished firmware build that is the only relevant
   one going forward is flashed as part of finishing the unit, not held for a decision.
2. **The D2 Stop fold at dwell 0.27.** The jitter-free virtual loop passes it at 125 mm with 10 %
   margin; the live splice has less. If every attempt's final cross-site throw refuses
   `LIMIT_JERK` on the robot, the fix is a same-site Stop (the cup transits to the last ball,
   which is the steady transit the box certifies) — a change to owner decision D2.
3. **Feed-catch accuracy** (30-40 mm entry offset, open-loop on the schedule prior): the next
   limiter for fed columns once the association is right; not addressed here.
4. **The ball in the cup.** The owner confirms a unique seat found quickly, so the −2 to −4 mm of
   landing per mm of in-cup offset at release is the ball leaving the seat during the stroke, or
   a seat off the cup axis; neither was measurable with the markers available.
5. **skill_node refusing a start while a jam is held** needs a `fault_state` key on
   `/link_status` and an `Observations` field (`executor.py` precondition ladder) — a follow-up;
   the bridge refuses to ARM while a jam is held, which is the hard stop today.
6. **Hop**: the box at 250 mm gives x ±1 mm in the 0.875-0.925 band; the sim's re-aim refusals
   are unchanged; hop is not on the sitting-4 sheet.
7. **The nightly runs against the main checkout**, where the BB firmware expectation is stale
   (4 against the flashed 5): RED every night until that branch is bumped or the runner moves to
   this worktree.

