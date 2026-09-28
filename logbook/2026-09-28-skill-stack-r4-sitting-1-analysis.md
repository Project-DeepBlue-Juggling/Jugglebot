---
title: "R4 sitting 1 under the calibrated geometry: a stale learner aimed every catch 20 mm off, the throw after a moving catch missed by 30–110 mm, the hop overshot 100 mm on a late release, and the reload never threw — four mechanisms from one bag, four fixes"
type: investigation
date: 2026-09-28
status: in-progress
phase: "two-ball-skill-stack — R4"
related_plan: two-ball-skill-stack.md
sessions:
  - temp/logs/skills_r4_20260927_2237.log
  - ~/Desktop/rosbags/2026-09-27_22-37-26 (mcap, 332 MB — not in the repo)
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/segments.py
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - tools/probes/release_motion_bag_probe.py
  - tools/probes/planned_release_motion.py
  - tools/probes/README.md
  - tests/hardware/session_skills_r4.md
  - plans/active/two-ball-skill-stack.md
  - logbook/INDEX.md
subsystem:
  - motion
  - ros
  - hardware
tags:
  - skill-stack
  - sitting
  - learner
  - planner
  - reload
---

# R4 sitting 1: what the bag said, and what changed because of it (2026-09-28)

## Symptom (owner, 2026-09-27 22:37 session, one launch)

The first `session_skills_r4.md` sitting after the kinematic calibration (`3059cc1`, FW 24).
Every self-toss throw "had the platform slightly tilted, so the platform had to move rather far
to catch every throw"; no `num_cycles: 5` chain completed (the 3rd or 4th throw dropped); every
Ball Butler reload attempt ended in an error or abort; the three 250 mm hops "looked quite good"
but the platform "was tilted too far and the ball consistently over-shot the target". The
owner's question: retrain the learner now that the geometry has changed?

## Diagnosis

Everything below is read off the launch log and the bag with two probes written for it:
`tools/probes/release_motion_bag_probe.py` (per announced release: mocap Platform tilt/xy −0.6..
+0.2 s, a g-fixed fit of the tracked free flight, the hand telemetry over a window) and
`tools/probes/planned_release_motion.py` (the same releases re-planned offline through the real
install chain, centroid xy / tilt / velocity per knot). Retraining alone would not have fixed
the sitting: only the first mechanism is the learner's.

### 1. The learner memory was learned against the old geometry (retrain: yes)

The 79 rows of 2026-09-23 converged on `u_y = +0.020` (the box edge) to land the ball at `y ≈ 0`
— a 20 mm plant bias in y. On 09-27 the same command landed the ball at `y = +0.020`: the
announced landing was (−51, +20) mm on all four single throws and the fitted landings were
within 10 mm of it. The calibration removed the bias the memory still compensates for; the apex
gain moved too (0.885 m for the command that used to give 0.900). Every catch therefore re-aimed
by the full 20 mm lateral authority, and *that* is the "platform tilted for every catch" the
owner saw: the planned bank for a 20 mm lateral catch is 1.2°, measured 1.1–1.5° at −0.4 s. The
memory is quarantined (`temp/learn/_quarantine_20260928/`, README alongside); the next sitting
starts cold.

### 2. The throw after a laterally re-aimed catch (the chain drops)

| chain | throw 1 (from rest) | throws after a catch |
|---|---|---|
| 5-cycle #1 | y +14 mm, caught | +114 mm, dropped |
| 3-cycle | +8, +2 | +64 (caught at the rim, seat +0.278 s) |
| 5-cycle #2 | −7 | +32 (x +48) dropped, +50 dropped |
| 5-cycle #3 | +35 | +54, +42 caught; +48 (x +44) dropped |

The first throw of every chain landed within 35 mm; every throw that followed a catch missed
toward +y by 30–110 mm. The bag shows the platform level (0.3°) and stationary at those releases,
so it is not a tilt at release. The offline replan (self-toss, every landing +20 mm in y) shows
what the plan does between the catch and the carried release: the centroid returns from y = +23.8
mm (−0.45 s) to 0 at the release knot, peaking at −104 mm/s at −0.15 s, with a 1.2° bank at
−0.5 s — while the hand carries the ball down from 9.6 rev and into the stroke. The measured
platform matched that plan (y +16..22 at −0.4 s → −3 at release). The ball then left with +85..
+110 mm/s of lateral velocity the plan never gave it (fitted launch v_y = 110 mm/s against an
announced 25). The mechanism is the ball's, not the platform's: a ball riding a hand that is
dropping at 9 rev in 0.3 s while its cup translates 20 mm sideways does not arrive centred and
settled for a 3 g stroke 0.35 s later; the seat times of the sitting (+0.17..+0.28 s) say the
same. R3 flew 26/26 with lateral authority pinned to 0, so no catch ever moved and no throw ever
started from a moving cup; 09-27 was the first flight with it unpinned (20 mm, owner 2026-09-23).

### 3. The hop: right tilt, late release, the return starts too soon

| hop | planned angle | fitted angle | landing error x | free flight vs release point at t_rel |
|---|---|---|---|---|
| 1 | 4.07° | 4.77° | +87 mm | −172 mm |
| 2 | 3.81° | 5.09° | +102 mm | −82 mm |
| 3 | 3.91° | 3.91° → 5.21° | +104 mm | −48 mm |

The mocap platform tilt at release was 3.9–4.0° against a planned 4.01°: the tilt is right. The
offline replan shows the planned centroid stationary at the release knot (the release equality
realises the launch velocity through the hand along the cup axis: planned cup velocity (292, 0,
4166) mm/s with platform (0, 0, 0)), then re-accelerating toward the far site immediately after
it: +33 mm/s at +0.05 s, +115 at +0.10, +196 at +0.15, +276 at +0.20. The ball's free flight,
extrapolated back to the planned release time, sits 48–172 mm BELOW the release point on every
throw of the sitting (self-toss too, 54–102 mm): the ball separates from the hand 20–40 ms after
the plan's release knot. A late separation on a stationary self-toss cup costs nothing; on the hop
it hands the ball the platform's first 20–40 ms of return (plus the hand's 5–13 % overspeed along
the 4° axis) — the +95 mm/s of surplus x velocity that overshot the far site by 100 mm three
times. The learner cannot absorb it: the hop box (carried item (1) of the R4 Outcome) admits
x ∈ [−20, +2] mm toward the far site and the error is 100.

### 4. The reload never threw, and the one thing that did move was the E-stop

Five attempts, five `THROW_REJECTED_BAD_STATE`. Ball Butler's `RELOAD` CAN command runs the
firmware's `requestCheckBall()` → CHECKING_BALL (pitch dips to the disrupt position, the sensor
is sampled, back to IDLE 1.1 s later: `/bb/heartbeat` state 1 → 6 at 22:47:28.84 → 1 at 29.94),
and skill_node sent `bb/throw_at_target` 5 ms after `bb/reload` — `requestThrow` accepts only
IDLE/TRACKING. Three more defects stacked on it: `ball_butler_node` publishes the
ThrowAnnouncement inside the service handler, before the Teensy answers; skill_node armed
`_reload_ctx` only after `_wait_future` returned, so every announcement (5/5) was logged
"arrived with no reload awaiting one — ignored"; and nobody consumed `bb/throw_outcome`
("THROW_REJECTED_BAD_STATE (axis=n/a, detail1=0)", published 20 ms after each request), so each
attempt waited out the 4.6 s `ABORTED_NO_ANNOUNCEMENT` timeout with the platform at rest.

The guard latch at 22:46:04.9 is a fifth item. The aborted 5-cycle run (`ABORTED_NO_RELEASE`,
its hold installed) left the hand at 8.87 rev. The next reload attempt was refused by BB
(calibration not yet received) but its opening REST — 6.6 s, hand 8.87 → 0.31 rev — was already
dispatched (the bridge executor goes live before BB is asked). `executor.install_segment` gives a
fresh origin `t0 = t_now` with no check that the wire can still meet it; the solve took 381 ms,
so the scheduled block reached the firmware ~0.4 s into its own profile. The firmware's scheduled
lane compares a resuming block, evaluated at arrival, against the held hand
(`SCHED_RESUME_TOL_POS_HAND_REV` 0.05 rev): 0.1 rev off, refused on all 80 frames
(`/link_status sched_refused` 0 → 80 over 22:46:02.9–04.9), the ODrive received NO new command
(`pos_cmd` flat at 8.858, `vel_ff` 0), while the deviation guard integrated the refused block's
base (`s_base_pos`, the firmware's "keep holding, stay loud") against the unmoving hand and
tripped `MAX_DEVIATION` at −2.508 rev. The self-toss opening RESTs (solve ~100 ms, a 0.3 rev
walk) passed the same check all evening. skill_node's per-tick `hand_lane_refused` end did not
fire in the field (its unit test passes) — investigated in the Fix.

A sixth, small one: one reload REST was refused `HAND_STROKE` at t = 0.006 s because the hand
seed velocity, −0.0435 rev/s of telemetry noise on a hand at rest, made the plan dive below the
parked start.

## Discussion

**The tilt the owner saw was not a defect; the motion it belonged to was.** My first reading of
"every throw had the platform slightly tilted" was a levelling or geometry error — the sitting
was the first under the new geometry, `level` had been run three times, and a wrong tilt centre
would look exactly like that. The bag refuted it in one line: the hop's platform tilt at release
matched the plan to 0.1°, and the self-toss releases were level to 0.3°. The tilt was the planned
1.2° bank of a 20 mm lateral catch, seen on every catch because the learner aimed every throw 20
mm off. The hypothesis that survived is one level up: **the platform must not move while it holds
a ball between a catch and the release that follows it**, and the plan moved it — back to the
nominal site during the dwell — because R4's carried throw releases at the SITE, not where the
ball was caught. The owner chose the redesign over re-pinning the authority to 0: throw from
where the ball was caught, and re-centre during the flight, which the SETTLE tail after the release
already does. The alternative (pin lateral authority to 0, R3's flawless configuration) was
rejected because the calibrated plant is near-identity in y only until the next drift, and a catch
that cannot follow the ball is what the tracker aim was built to replace.

**The hop error is a timing error wearing a geometry costume.** A 100 mm overshoot at 250 mm reads
like a 30 % range error or a 1.2° aim error, and the owner read it as "tilted too far". Neither:
the tilt was right and the vertical was within 3 %. The instrument that settled it is the
free-flight parabola extrapolated back to the planned release instant — below the release point
on every throw, i.e. the ball leaves late. The same lag is invisible on a stationary cup and fatal
on one that starts moving the instant the plan says "released". The chosen fix (a 75 ms platform
hold after every release, owner 2026-09-28) treats the plan's side; the ball's late separation
itself (hand overspeed under K = 0.7, cup compliance) is left as a characterisation item because
the learner already absorbs its vertical effect and the hold removes its lateral one.

**Two fail-closed mechanisms existed and neither acted.** The firmware's scheduled lane refused
the block loudly — by letting the deviation guard E-stop the machine two seconds later — and the
host's `hand_lane_refused` end, built for exactly this (2026-09-18), stayed silent. A refusal the
host learns of only through an E-stop is not a refusal surface; the host-side fix is to never
send a fresh-origin block the wire cannot meet (rebase a REST, refuse an event-bearing kind), and
to find why the per-tick end did not fire.

**What was ruled out.** A geometry or levelling error (tilt at release matched the plan); the
tracker (fits converged on every caught throw, landing errors ≤ 20 mm on throws from rest); a
Ball Butler hardware fault (its state machine did exactly what its RELOAD handler says; it had a
ball throughout); the hop box collapse as the cause of the overshoot (100 mm is five times the
authority in any box).

## Fix

Four units, each with fail-before / pass-after tests; owner decisions 2026-09-28: redesign the
carried throw before the next sitting (not re-pin the authority), a post-release hold in the
planner, fix every reload defect now, retrain cold.

**A — the carried throw releases from where the ball was caught** (`executor._catch_terminal`):
`ThrowAfterCatch.site_mm` is the CLAMPED landing's xy at the site's throw z; the target, flight and
release instant are the schedule's; the rest site stays the nominal site's, so the return to the
site happens in the SETTLE tail after the ball has left. The announcement / landing prior of the
next catch is computed from the same release position (`_release_pos_mm`, one answer to "where
does this release happen"). Offline (self-toss 0.9 m, landing +20 mm in y): cup wander over the
dwell 20.00 → 1.27 mm, peak centroid lateral speed 104.6 → 7.09 mm/s (the residual is banking
compensation of a 0.32° release tilt, not translation). A landing ON the site clamps to the site's
xy bit for bit, so every existing pinned plan is unchanged. The admissible boxes were swept for
releases at the site; the release offset is bounded by the same 20 mm authority and the online
`validate_cycle` gate remains the authority (docstring caveat).

**A′ — the release offset is learner state, and its fly-back is pre-compensated in closed form.**
Unit A's first sim gate walked the landing 50 mm off the site on 2 of 5 seeds (0 drops). Two
things were behind it. (i) `memory.Experience.x[2:4]` — "the seat-offset xy of the ball just
caught", reserved since R3 and pinned at zero — is exactly this offset; `_command_u` now queries
with it and the Experience row carries the final re-sent value. Honest state, bit-identical
commands in a 25-throw cold run (the local affine fit's δx slope is not identifiable from one
site's rows). (ii) The plant scales the whole launch vector by a speed ratio s (the launch lies
along the cup axis with the platform stationary at release), which multiplies apex AND lateral
reach by s²; s² is the ratio the learner already commands to land the apex, g = apex_d / u_apex,
so `_offset_flyback_mm` aims r·(1 − 1/g) toward the release side and the landing is site + g·u_dy,
free of r. Measured on the fixed gate (below): late-half median error 0.65 → 0.22 mm, worst 2.7 →
0.7 mm on seeds 0 and 3. No knob: g is clamped to [0.5, 2].

**Sim gate — a re-send supersedes its pending release** (`sim/skills_gate._make_installer`). The
walk itself was the gate's: `pending_releases` appended every accepted install of the same
(ball, release instant), and the stream loop released the ball with the FIRST dispatch's takeoff
(planned for a release at the site) from the re-aimed cup, the re-sent takeoffs carrying the
fly-back sitting behind it unused (`held` already False). Traced with the plant's `release_ball`
instrumented: v_y at release −2.6 mm/s = (−34.3 · 1.11 + 3764.5 · 1.11 · sin 8.5 mrad), the
first dispatch's plan through the gate's bias model, on a ball released 13.9 mm off the site.
Harmless before unit A (identical takeoffs); now the last terminal wins. Seeds 0 and 3 pass with
the fix, with or without the pre-compensation.

**C-node — the reload chain** (`skill_node`, tests `test_c1_*`…`test_c5b_*`): (C1) after
`bb/reload` the node waits for Ball Butler's own heartbeat to read IDLE + ball in hand + connected
before `bb/throw_at_target` (phase `await_bb_idle`, `RELOAD_BB_READY_TIMEOUT_S` 3.0 s, else
`ABORTED_BB_NOT_READY(<state>, ball_in_hand=<bool>)`); the throw delay is derived from the bridge
REST's own settle instant at fire time. (C2) `await_announcement` is armed before the throw call.
(C3) `bb/throw_outcome` is consumed: a non-`OK` token ends the attempt the same tick with
`REJECTED_BB(<token>)`, both while the throw is awaited and after an announcement has already
compiled the schedule (`_reload_throw_inflight`). (C4) every refusal after the bridge executor
exists ends the attempt through one point (`_end_attempt`) instead of rejecting the goal over a
live executor. (C5b, root cause of the silent guard latch) the bridge is a ONE-skill schedule, so
its executor was `done` — and nulled — the tick it dispatched, up to 6.4 s before the REST settled,
and `hand_lane_refused` had nothing left to be read by; `_check_reload_lane_refused` now watches
the reload context directly. A double `trajectory/hold` call found on the way was fixed. One
change the unit proposed was REVERTED: announcing only on the Teensy's OK result — that result is
terminal (`OK` once the ball has left, `ABORTED_NOT_SETTLED` if it aborts at fire), so the
announcement would have arrived after the flight began and starved `compile_reload` of its
pre-tilt lead. Ball Butler keeps announcing at dispatch; the outcome consumer covers the
announced-then-refused case instead. The phase-end audit (2026-09-28, `/audit --unstaged`) found one behaviour defect in this unit and five narrative slips (a "13 mm/s" residual quoted for the shipped 2-knot hold that belonged to the rejected 3-knot cut, two wrong test-count triples, a cited test-name pattern that did not exist, a `files_changed` entry with no diff), all applied: `_fire_reload_throw` blocks on Ball Butler without `_reload_lock` while the tick keeps running, and when another path (a `sched_refused` bump, a Stop, the timeout) had already ended the attempt, its late failure branch called `_end_attempt` again and — with the bridge executor retired — overwrote `_goal_end_code`, losing `HAND_LANE_REFUSED` to a later `REJECTED_BB`. `_still_current(ctx)` now guards every failure branch; the hold the earlier path armed stays.

**B — the platform holds its translation for 50 ms after every release** (`unified_cycle.
POST_RELEASE_HOLD_S`, `CycleGoals.hold_platform_knots`, the hold rows in `cup_cycle.plan_window`,
set by `segments._hold_knots` on every window built off a post-release seed; sized at 2 knots — see below). The hold pins the cup
VELOCITY onto the line a frozen platform's stroke can reach, `v_xy = axis_xy · v_z` (the same slope
`cup_realize.decompose` uses for the centroid), which makes the realised pose velocity zero rather
than small; velocity, not position, because four position+velocity rows per knot against three
jerk variables are rank-deficient, and what the ball leaves with is the cup's velocity. The rows
REPLACE the detach cone over those knots, and that names the root of fact 3: the cone's
`acc = g + λ·axis` buys its axial specific force by translating the platform — `pose_acc_xy =
−acc_z·sin 4.006° = 686 mm/s²`, exactly the +16.9 mm/s at +1 knot and +32.8 at +2 the plan
showed. Two things the brief did not foresee. (i) The hold must sit on the LANDING/STEADY
windows too, not just the THROW tails: a hop's CATCH at the far site is installed 0.27 s before
the release and spliced AT the release knot, so the post-release knots come from the catch's own
window (with only the tails held the hop verdict did not move at all, 68.10 → 68.10 mm/s); it is
keyed on `seed.post_release`, and `protected_release_knots()` widens the splice guard so a splice
cannot land inside the held block. (ii) The ATTITUDE is not held: freezing the tilt over any span
of the hold REFUSES on the 250 mm hop at the operating point (`LIMIT_JERK` over knots 0–1,
`LIMIT_ACC` 5538 > 5000 over 0–2, 12 199 over 0–3, 21 711 over 0–4) because the tail then has
75 ms less to level and the pin is a kink, not a rescaled slew; the banking slew therefore leaves a
residual through the 224–261 mm cup lever. Measured through the real install chain (worst centroid
speed over the 3 post-release knots): hop 68.10 → 13.36 mm/s (2.02 → 0.34 mm), self-toss re-aimed
releases 8.5–11.6 → 5.9–12.5 mm/s, level self-toss 0 → 0. The hop's CATCH at P2 with 75 ms less to
return: leg vel 191 → 213 mm/s (bound 300), acc 1421 → 1469 (5000), jerk 92 454 → 92 374
(150 000), hand acc 3265 → 2978 (3500); nothing refused, no limit widened. The attitude half is
carried to the plan as an owner item (a flat-then-slew tilt schedule or a longer tail). QP-level tests (`test_cup_cycle.py`, `test_unified_cycle.py`: the held-line relation at the residual floor, bit-identity with the field absent at the assembled-program and solved-plan levels, the four refusal paths, `protected_release_knots` and the splice guard) found one latent defect on the way — `protected_release_knots` passed its config positionally into `post_release_hold_knots`'s `hold_s` slot, a `TypeError` for any non-None config that no production caller yet passes — fixed with the keyword. **Sizing:** the first cut was 3 knots (75 ms, 2× the worst 40 ms lag); with it the R2 two-ball columns schedule at 100 mm refused `LIMIT_VEL` 303.6 > 300 mm/s and the columns sim smoke run dropped a ball (3/4 makes) — every knot held is a knot the next window loses for its return. Measured in-process with `segments._hold_knots` clamped to 0, 2 and 3: 2 knots plans everything and the smoke run is 4/4 again. `POST_RELEASE_HOLD_S = 0.050` (1.25× the worst lag); the hop's platform speed at +50 ms is what sitting 2 watches. B's residual numbers above were measured at 3 knots; the 2-knot values are in the Verification section.

**C-exec — a fresh origin the solve outran, and the hand seed at rest.** `executor.install_segment`'s
fresh branch re-reads the install clock after the solve, as the splice branch always did: when the
solve exceeded the wire lead (`WIRE_READ_KNOTS`·dt = 75 ms), a REST is REBASED to `t_inst + 75 ms`
(the same motion, later; its message says so and the record's `t0_s` is the rebased one) and an
event-bearing THROW/CATCH is REFUSED `ORIGIN_TOO_LATE` — one sentence naming the solve time and how
old the first knot would be at the wire. Inside the lead nothing changes (a pin holds `t0 == t_now`
bit-identical). The live path already passed `t_install_s=time.perf_counter` for both branches, so
the fix is live without a node change. Typical segment solves are 26–34 ms (2026-09-12); the
6.6 s hand-homing REST's 381 ms is the long-window case the rebase exists for. In
`trajectory_node._commanded_hand_state`, a hand state taken from the last emitted rev or from the
encoder (no hand-bearing plan) now seeds velocity 0.0: the streamed lane is the only hand master,
so a hand nothing is moving is at rest and the ±0.05 rev/s of telemetry is noise. One existing
test that simulated "the hand is moving" through that telemetry channel was reworked to drive the
motion through an active plan, which its own docstring said was the intended source. A real
load-flake surfaced: the one install test that read the real clock could now measure a genuinely
late origin on this shared box (1.43 s once under a load average of 4.5) and was frozen like its
neighbours; the 75 ms production margin was not touched.

## Verification

* 2026-09-28, `python tools/probes/planned_release_motion.py` at `POST_RELEASE_HOLD_S = 0.050`
  (2 knots), limits 300/5000/150000, hand 3500, HOLD verdict = worst centroid speed /
  displacement / tilt change over the held knots after each release, through the real install
  chain: **hop 250 mm 2.75 mm/s / 0.076 mm / 0.013°** (68.10 mm/s unheld, 13.36 at 3 knots);
  self-toss ×3 on the site 0 / 0 / 0; self-toss ×3 with every landing +20 mm in y 6.53, 3.92,
  4.30 mm/s (8.5–11.6 unheld). Nothing refused.
* 2026-09-28, in-process hold clamp (`segments._hold_knots` → min(N)) on
  `tests/motion/test_skills_executor.py::test_a_six_throw_columns_schedule_installs_end_to_end`,
  `::test_a_columns_schedule_attributes_every_catch_to_its_own_ball`,
  `tests/sim/test_skills_gate.py::test_the_smoke_run_is_not_vacuous_and_never_drops`: **N = 0 →
  3 passed; N = 2 → 3 passed; N = 3 → 3 failed** (`LIMIT_VEL` 303.6 > 300.0; smoke run 3/4
  makes). This is the sizing measurement behind 2 knots.
* 2026-09-28, `python sim/skills_gate.py --learn --pattern self_toss --seeds 0 3 --no-viewer`
  after the gate's supersede fix: **PASS 2/2** with the fly-back pre-compensation (late-half
  median err 0.22 / 0.20 mm, worst 0.7 / 0.6) and **PASS 2/2** with it disabled in place (0.65 /
  0.59 mm, worst 2.7 / 1.0); before the supersede fix, with or without the pre-compensation,
  **FAIL** on both (late-half median 42.8 / 43.6 mm, `mono_xy False`, 0 drops, 25 makes).
* 2026-09-28, `pytest tests/motion/test_skills_segments.py tests/motion/test_skills_install_origin.py
  tests/motion/test_skills_executor.py tests/sim/test_skills_gate.py -q -p no:cacheprovider
  -p no:randomly` at 2 knots → **178 passed in 58.13 s**.
* 2026-09-28, `pytest tests/ros/test_skill_node.py tests/ros/test_ball_butler_node.py -q
  -p no:cacheprovider` → **184 passed in 3.81 s** (C-node, with the C2b revert and the widened C3).
* 2026-09-28, `pytest tests/motion/test_skills_install_origin.py tests/ros/test_trajectory_node.py
  tests/ros/test_install_segment.py -q -p no:cacheprovider` → **156 passed in 52.78 s** (C-exec).
* Fail-before / pass-after, every new test (2026-09-28, each production change reverted in place
  and restored, no git state): A's five real-chain tests (cup at the site / "cup wandered 20.00 mm"
  / prior vel y 0 vs −23.34 / hop cup (125, 0) not (140, 0)) FAIL → PASS; the learner-state pair
  (`0.0 == 0.02`) FAIL → PASS; the fly-back pair 1 FAIL 1 PASS (the bit-identity pin passes both
  ways) → 2 PASS; the gate supersede test FAIL (2 pending entries) → PASS; B's two behaviour tests
  (50.17 / 74.44 mm/s off the stroke line) FAIL → PASS with the pin passing both ways; C-node's
  new tests 9 FAIL 1 PASS → all PASS; C-exec's C5 pair FAIL → PASS, C6 pair (`−0.0435 == 0.0`)
  FAIL → PASS.
* 2026-09-28 12:25–12:37 (`temp/logs/r4b_gates_hold50_20260928_1225.log`, box under a
  concurrent sweep), `python sim/skills_gate.py --learn --pattern self_toss --seeds 0 1 2 3 4
  --no-viewer` twice → **PASS 5/5 both runs, 25 makes / 0 drops / band entry 3 throws per
  seed, reports bit-identical with wall times stripped**; `--pattern hop` twice → **25 makes /
  0 drops on every seed, both runs bit-identical**; the hop's band verdict is `None` on every
  seed, as it was at R4 (`tests/sim/test_skills_gate.py::test_a_small_hop_learner_run_is_apex_
  clipped_not_converged`: the hop learner is box-clipped, not converged — carried item (1)), so
  the R4 criterion (drops, bit-identity) is what these runs assert.
* 2026-09-28 12:39–13:45 (`temp/logs/admissible_sweep_r4b_20260928_1239.log`), `python
  tools/admissible_sweep.py --site-pairs all --single-apex 0.5 0.6 0.7 0.8 0.9 --out
  temp/probes/admissible_box_r4b_run<n>.yaml` twice (33.2 and 32.2 min, `OMP/OPENBLAS_NUM_THREADS=1`,
  a `./run_tests.sh` running alongside the first) → **the two boxes are bit-identical** (`swept_at`
  excluded); installed as `config/generated/admissible_box.yaml`, **gate_hash `3fda47b2ad5b`**
  (was `7f76f68d4943`; `segments.py`, `unified_cycle.py` and `cup_cycle.py` changed). Bounds:
  every self-toss and columns box unchanged; the hop's x bound away from the far site halved —
  P1→P2 x [−10, +2] mm (was [−20, +2]), P2→P1 x [−1, +10] (was [−0.5, +20]); y ±20 and apex
  0.85–0.95 unchanged. The hold costs the hop's return 50 ms and the learner 10 mm of the
  authority it never had toward the far site (carried item (1) of the R4 Outcome).
* 2026-09-28 14:24, `./run_tests.sh` on the final tree with the new box installed
  (`temp/logs/run_tests_r4b_final_20260928_1424.log`) → **PASS: 5477 passed, 8 skipped in 218.28 s
  (parallel); serial 3 passed in 8.72 s**. (The run before the box landed, 12:39, read 22 failed:
  19 `admissible box refused … gate_hash` in `test_skill_node.py`, the regenerated choreography map,
  and one load-flaked UDP-diag test that passes alone.)
* 2026-09-28, phase-end audit `/audit --unstaged` (one Sonnet reporter, file by file): 1 BEHAVIOUR + 5 NARRATIVE
  findings, all applied (Fix, C-node). The race's test
  `tests/ros/test_skill_node.py::test_c1_a_throw_call_overtaken_by_a_lane_refusal_does_not_overwrite_the_end_code`
  FAILS with `_still_current` forced True (`_goal_end_code == 'REJECTED_BB(BAD_STATE)'`) and PASSES with the
  guard; `pytest tests/ros/test_skill_node.py tests/ros/test_ball_butler_node.py -q` → **185 passed in 3.52 s**.
* 2026-09-28 14:43, `./run_tests.sh` on the post-audit tree (`temp/logs/run_tests_r4b_postaudit_20260928_1443.log`)
  → **PASS: 5478 passed, 8 skipped in 219.26 s; serial 3 passed in 8.71 s**.
* 2026-09-28 14:28, `./run_tests.sh --full` (`temp/logs/run_tests_r4b_full_20260928_1428.log`) →
  **PASS: 5515 passed, 8 skipped, 1 xfailed in 278.17 s (parallel); serial 6 passed in 19.95 s**
  (the R4 closure run of 2026-09-24 read 5416 passed, 8 skipped, 1 xfailed).
* 2026-09-28 14:48, `./run_tests.sh --full` on the post-audit tree (`temp/logs/run_tests_r4b_full2_20260928_1447.log`)
  → **PASS: 5516 passed, 8 skipped, 1 xfailed in 277.30 s; serial 6 passed in 19.69 s** — the phase's closing gate.
