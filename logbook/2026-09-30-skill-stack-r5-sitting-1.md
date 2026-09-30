---
title: "R5 sitting 1 (2026-09-30 evening): the 4 deg / 0 deg cup test and the fused reload flew, the YAW settle refusal traces to a sigma-delta dither on a 150 Hz finite-difference rate, the feed lands 52 ms LATE not early (a withdrawn claim), the lateral bias traces to BB's stale aim affine, and Block C (human lob) is retired for a BB-fed feed"
type: investigation
date: 2026-09-30
status: in-progress
phase: "two-ball-skill-stack — R5"
related_plan: two-ball-skill-stack.md
related_entries:
  - 2026-09-30-skill-stack-r5-columns-bb-start.md
files_changed:
  - tests/hardware/session_skills_r5.md
  - tests/hardware/session_skills_r5_sitting2.md
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/segments.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/admissible.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/ball_butler_node.py
  - sim/skills_gate.py
  - tools/probes/feed_catch_bag_probe.py
  - teensy_link/rpc_args.py
  - config/generated/admissible_box.yaml
subsystem:
  - motion
  - ros
  - tracking
tags:
  - performance
  - kinematics
---

# R5 sitting 1: the cup test, the fused reload, and why the human lob is retired

## Summary

The first R5 hardware sitting (2026-09-30, launch 16:20-16:57, bag `2026-09-30_16-20-05`)
flew the runsheet day 1's planner findings produced (`tests/hardware/session_skills_r5.md`):
a leg-jerk ramp to 200 000 mm/s³, Block A (the reload's receive-tilt cap swept 12°→4°→0°,
skipping 8°), Block B (the fused catch→re-level→throw reload window), and Block C (a
human-lobbed columns feed). The ramp held, the cup took 4° and 0° cleanly enough that the
owner wants 0° made the default, and the fused reload flew with no rebound. Block C never
registered a single lob and is retired. Offline analysis after the sitting found: the BB
settle refusal is a firmware rate-gate false-positive, not a real slew; the feed lands **52 ms
late**, not the ~0.1 s early an earlier reading of the log claimed; the +27/+11 mm lateral
bias traces to a BB aim calibration that drifted out from under a six-month-old affine; and
the BB-fed columns start still does not fit inside one flight from BB's current perch — a
held-level catch that carries the throw is the lever the owner picked instead.

## Context

Day 1 (`logbook/2026-09-30-skill-stack-r5-columns-bb-start.md`) measured the BB-fed columns
start on the real planner before any hardware ran: the six-step re-attitude sequence the plan
opened R5 with does not close against one 0.9 m flight, a fused held-catch→re-level→throw
window shortens every reload by 0.20-0.25 s, and the admissible box was re-swept at 200 000
mm/s³ jerk after a segment-modelling defect was found and fixed. That re-scoped the runsheet to
what this sitting flew: a jerk-ramp measurement, a reload receive-tilt sweep (the "4° cup
test"), the fused-reload control flights, and an interim columns start fed by a human lob
(since BB cannot yet aim at two live sites in one round trip).

## Observations

Owner's report, in their own words (2026-09-30 evening): everything before Block A ran fine
(the dress rehearsal was **not run** — the owner never runs it); "4.0 deg worked perfectly
every time" so 8.0 was skipped; tried down to 0 deg, "still almost all of the feed throws were
caught"; "the timing seems a little off"; a hunch that 0° can be made seamless. Ball Butler
kept refusing feeds with "YAW axis not settled" although it visibly does not move. Block B
passed; some cycles looked jerky; jerk would be lowered from 200 000 if affordable. Block C
never registered anything, even near-vertical lobs beside the robot — no more human-initiated
multi-ball routines; BB-led patterns are the goal.

**Feed catches by hold cap** (`launch.log`, bag `2026-09-30_16-20-05`):

| Cap | Caught / fed | Refusals (THROW_ABORTED_NOT_SETTLED) | Notes |
|---|---|---|---|
| 12° | 5/5 | 1 (detail1 = 30 centideg) | includes Block B's fused-reload flights |
| 4° | 2/2 | 2 (detail1 = -18, -40) | 8° skipped, owner call |
| 0° | 6/7 | 2 (detail1 = -47, -32) | the 1 miss was `REJECTED_NO_BALL` (`launch.log:1114`), normal timing |

5 settle refusals total, all `axis=YAW`, all detail1 values inside the 1.0° position
tolerance (30, -18, -40, -47, -32 centidegrees) — the refusal is on the rate term, not
position.

**Feed landing timing** (raw mocap, 14 feeds, `feed_timing_report.md`):

| Measurement | Value (mean, stdev) |
|---|---|
| Ball crosses 830 mm plane vs. announced/committed landing | **+51.9 ms LATE** (15.4) |
| Release vs. announced `throw_time` | +37.2 ms (matches `flight_fit.py`'s documented 5-55 ms push phase) |
| Flight duration vs. nominal | +14.7 ms long |
| Platform dive start vs. crossing | ~206 ms before |
| Platform bottoms out vs. crossing | +139.6 ms (15.7) |
| Seat (HELD) | 108-603 ms |

All tilt-independent. The log's `RESEND-SKIPPED / NO-CONVERGED-FIT` lines (-0.09..-0.15 s) are
an unconverged-fit artefact, not a real early landing.

**Lateral bias** (all 14 feeds): **+26.9 mm (4.7) in +y, +11.2 mm (9.0) in +x** of the
commanded site (`y_bias_report.md`, `lateral_ratio_report.md`).

**Jerk** (`jerk_report.md`, bag + analytic ladder): self_toss uses ~0 of the leg-jerk budget;
hop peaks 0.28-0.32× the 200 000 cap in the bag (0.55-0.79× on the analytic ladder,
140 000-200 000); columns is hand-bound (0.950 flat) above 160 000 and only jointly
jerk+hand-bound at 150 000. No columns segment flew this sitting.

**Block C**: three columns-feed attempts (`reload: false`, human lob), each
`ABORTED_NO_COLUMNS_FEED` after the 30 s deadline (goals accepted `launch.log:2017, 2035,
2053`; two of the three ended by `launch.log:2029, 2047` — the third's end line falls after
the excerpted window). `detect_human_throws` was confirmed live on `/ball_tracker_node` at
`launch.log:2003`. Not diagnosed further this sitting — retired (owner decision, below).

Also observed, not this sitting's focus: leg heartbeat dropouts (0.4-1.4 s, several legs,
load-gated, `plans/active/leg-bus-frame-drops.md`); two 12°-hop reloads dropped throw 3
(`launch.log:1731, 1813`, `ABORTED_NO_RELEASE`, x +31/+32 mm).

## Diagnosis

**Settle refusals.** `not_settled_report.md` traces the mechanism: `YawAxis.vel_rps` is a
150 Hz finite difference of a 16384-count encoder, smoothed by an EMA at α=0.3. One encoder
count per sample already reads 3.296 deg/s — above the firmware's 3.0 deg/s `YAW_RATE_TOL_DPS`
gate — so a genuinely settled, pure-P-controlled axis with sigma-delta dither on its deadzone
crosses the instantaneous rate threshold on noise alone. 4 of the 5 refusals sat on a flat,
sub-0.4° peak-to-peak plateau indistinguishable from the accepted throws (report §§ Q1-Q3): a
false-positive rate, not a real unsettled axis.

**Feed timing.** The ball crosses the 830 mm catch plane 51.9 ms after the committed landing —
not early. Release already runs 37.2 ms after the announced `throw_time` (matches
`flight_fit.py`'s documented 5-55 ms push-phase delay), and the flight is 14.7 ms long on top
of that; the two roughly account for the crossing offset. The platform's own dive starts
~206 ms ahead of the crossing and bottoms out 139.6 ms after it — the catch is early into the
dive and still closing when the ball arrives, consistent with the "timing seems a little off"
the owner reported by eye.

**Lateral bias.** `throw_affine_correction.json` (`n_pairs=41`) was fitted 2026-06-09 at BB
pose (-892.5, -191.9, 1741.2) mm, yaw offset -8.35°. Today's live BB self-calibration reads
(-1018, -433, 1740) mm, yaw offset +1.95° — a 10.3° yaw shift traced to the 2026-09-27
coplanar-marker recalibration (`logbook/2026-09-27-bb-calibration-coplanar-markers.md`). The
affine now corrects for a BB that no longer stands where it was fitted; fix is a fresh BB
accuracy volley, not a code change.

**The BB-fed columns start's planner refusal.** With no `hold_tilt`, the window builder pins
the catch attitude with `tilt_to_receive` (`unified_cycle.py` ~1927-1929): a "level" catch of
a 12° feed banks 12° during the transit and dive, refusing leg jerk at 1.72×, velocity at
1.2-1.3×, acceleration at 1.2×, and the cup-contact floor. Sweeping `catch_vel_ratio` 0.7→0.0
changes nothing; sweeping leg-velocity limits 300/400/500 changes nothing. From BB's current
perch, 39/39 sampled launch arcs refuse; a placement ~0.5 m from the cup with an 84°-lob
arrival (6.2 m/s at 4.3°) admits.

**The held-level program that flew.** `hold_tilt=(0,0)` — attitude pinned, zero platform
translation, a pure vertical hand stroke — is what Block A actually flew. Carrying the throw
through it fits at dwell 0.30 s, hand cap 3500 (87.5% of cap, 0% of every leg channel,
`held_level_feed_report.md` §1); at 4° it is 7.6% over the hand cap (§2, cross-checked against
`bbfed_columns_probe.md`'s 3765.1 rev/s² figure). Blocker for using it inside the actual
columns schedule: `held_from_k > 0` is refused by name at `unified_cycle.py:1979-1994`,
`cup_realize.py:1201-1205`, `cup_cycle.py:958-967` — a held span can only start at knot 0, and
the columns feed catch is seeded mid-window, after the transit from P1 (§3).

## Discussion

**The three claims withdrawn this evening, and why each fell.**

1. *"Every feed landed ~0.1 s EARLY."* Withdrawn: this read the `RESEND-SKIPPED /
   NO-CONVERGED-FIT` log lines (-0.09..-0.15 s) as landing timing. Those lines are an
   unconverged ballistic fit, not a landing measurement — the raw mocap crossing says +52 ms
   **late**. The direction of the error matters for any timing fix: a "pull the catch earlier"
   change built on the withdrawn claim would have moved the platform the wrong way.
2. *"The lateral velocity match (`catch_vel_ratio`) is what hits the leg walls."* Withdrawn:
   sweeping it 0.7→0.0 on the BB-fed columns probe changes nothing. The real binder is the
   attitude pin `tilt_to_receive` installs whenever no `hold_tilt` is given — a level catch of
   an off-vertical arrival banks the platform to meet the ball, and that banking, not the
   lateral velocity match, is what costs the jerk/velocity/acceleration margin.
3. *Day 1's premise, "a level cup cannot take an 11.9° arrival"* (the F-a framing that a ~2°
   tolerance bounded the level catch). Refuted directly by Block A: a 0° hold caught 6/7 fed
   balls whose natural arrival at P2 is nowhere near vertical. The cup takes a real off-axis
   arrival; what it cannot take is the *attitude change* a mid-flight re-bank costs when there
   is a second ball and a throw to fit around it.

**The `tilt_to_receive` finding.** Because it fires whenever `hold_tilt` is unset, every
"level catch of an angled arrival" the planner has ever offered was secretly banking to meet
the ball. That reframes what R4's 12° hold and this sitting's 4°/0° sweep were comparing: not
"how far off-axis can the cup take a ball" (Block A answered that: quite far, cleanly) but "how
much attitude change can the window afford before the flight runs out" — the real columns-start
constraint.

**Why held-level beats placement as the primary lever, and what flips it.** A held-level catch
carrying the throw costs 0% of every leg channel and 87.5% of the hand cap at the current
operating point (dwell 0.30, hand 3500) — cheap, and no change to where BB stands. BB placement
(~0.5 m closer, an 84° lob) is the pre-registered fallback specifically because it requires
re-siting hardware the owner would rather not move if the planner-side fix works. Flip
condition (day 1's D1 criterion, re-confirmed here): if the held-level catch cannot be seeded
where the columns schedule actually seeds it — mid-window, off the previous throw's release
state, not an idle rest — the decision reopens on BB's placement (the open blocker above,
`held_from_k > 0` refused by name).

**Why the jerk ceiling stays at 200 000.** The owner's bar was "not terribly worrying if not"
lowered. The numbers support leaving it: self_toss uses essentially none of the budget, hop
sits at 0.28-0.32× the cap in the bag, and lowering the ceiling changes nothing about the jerky
look the owner watched for — the candidates for that (rest-to-motion splices, the
pre-/post-release holds, the re-level slew) are elsewhere in the trajectory, not at the jerk
wall. Only columns is jerk-relevant at all, and it never flew this sitting.

**Why not the held-axis capture span.** A design was drafted (`design_held_level_feed_catch.md`,
read-only, no repo edits) that would let a held attitude span start mid-window, off a segment's
own `release_state`, at knot `k_h = k_td - L` with `L = 0` at zero bank — the more general fix,
since it would also cover an oblique arrival. Its own arithmetic rules it out at the current
operating point: the minimum rest-to-rest lateral transit under the session's 300 mm/s³ jerk box
is `T = (32·D/J)^(1/3)`, which for the nominal 100 mm separation is 0.220 s against 0.225 s
available — a 5 ms margin — and the executor's own ±40 mm landing clamp already permits a
transit as long as D=140 mm, whose 0.246 s bound the window cannot meet. The level-pinned
`receive_tilt` build needs no held span, no arrival triple and no lateral slaving, so it carries
no such transit floor and wins on scope for R5; the held-axis design stays a design note for if
the feed routinely lands displaced beyond what `receive_tilt` alone covers.

**Why the +20 mm cliff became +30 mm.** `level_pinned_feed_report.md` §3's probe hand-built
`ThrowAfterCatch.site_mm` pinned to the nominal site, so its +20 mm case still paid for a
return-to-centre move inside the dwell. `plan_columns_first_cycle` instead goes through the real
`SkillExecutor._catch_terminal`, whose 2026-09-28 rule ("the release rides the caught xy, not
the site") removes that return-to-centre move for a displaced feed, which measurably widens the
margin: reprobed 2026-09-30, +20 mm now plans clean and +30 mm is the smallest displacement that
reliably refuses (`LIMIT_VEL`, 323.8 mm/s, 108% of cap) — same mechanism, same direction, a
larger displacement, not a contradiction of the report.

## Fix

Six units landed in parallel this evening; none are committed yet, so a later
`git log --grep "Logbook-Entry: 2026-09-30-skill-stack-r5-sitting-1"` is how a future session
finds them.

**(a) `receive_tilt` — level-pinned feed catch.** New optional touch-down override, threaded
`CycleGoals.receive_tilt` (`unified_cycle.py`, field beside `hold_tilt`; `_realize`'s window
builder, ~1960-1973) → `CatchTerminal.receive_tilt` (`motion/skills/segments.py`) →
`Skill.receive_tilt` (`motion/skills/schedule.py`; `compile_columns` sets `(0.0, 0.0)` on the
feed catch only) → `_catch_terminal` (`motion/skills/executor.py`); three docstring/error-string
text fixes in `motion/skills/admissible.py`. *Why*: whenever `hold_tilt` is unset the window
builder calls `tg.tilt_to_receive(...)` and banks the platform to meet the ball's raw arrival
direction — that banking, not the lateral velocity match, costs the ~1.7× jerk margin (Diagnosis,
above); `receive_tilt` pins the touch-down attitude level without touching the lateral channel.
Acceptance (`probe_receive_tilt_acceptance.py`, real field, no monkeypatch): S2
`receive_tilt=None` refuses `LIMIT_VEL` (366.5 mm/s > 300.0); S2 `receive_tilt=(0,0)` plans at
hand frac **0.951**; S2v (vertical arrival) unchanged at v0.852 a0.794 j0.802 h0.950. Two of the
seven `_GATED_FILES` touched (`unified_cycle.py`, `segments.py`) → box re-sweep needed, gate
hash `f96fc30012d7` (was `581109806b4e`; re-swept and installed, see Verification). Test (2026-09-30): `pytest
tests/motion/test_unified_cycle.py tests/motion/test_cup_cycle.py
tests/motion/test_skills_segments.py tests/motion/test_skills_schedule.py
tests/motion/test_skills_executor.py tests/motion/test_skills_admissible.py -q` →
**550 passed, 2 skipped, 15.69 s** (re-run after unit (b)'s three executor tests landed; the unit's own snapshot read 547).

**(b) pre-throw feed check — `plan_columns_first_cycle` + `REJECTED_COLUMNS_FEED_UNCATCHABLE`.**
New helper `executor.plan_columns_first_cycle(schedule, limits, ...)` (`motion/skills/
executor.py`) plans ball A's THROW-from-rest and the feed CATCH-with-throw through the real
`_throw_terminal`/`_catch_terminal` machinery before any executor swap; `skill_node.py.
_install_columns_schedule` calls it right after `compile_columns` succeeds and refuses with
`REJECTED_COLUMNS_FEED_UNCATCHABLE` (limit code + the landing's lateral offset from site1) on
failure. *Why*: without it an uncatchable feed silently swaps the executor and installs the
columns schedule anyway (the fail-before bug); the cliff is directional — a feed displaced
further from P1 stretches the required transit beyond what the dwell affords. Cliff
(`probe_feed_check.py`): **+30 mm in x beyond P2** is the smallest displacement that reliably
refuses — `LIMIT_VEL`, 323.8 mm/s (108% of the 300 mm/s cap); 0..+20 mm plans clean (Discussion,
"Why the +20 mm cliff became +30 mm"). Wall time: undisplaced 55.9 ms mean, +30 mm 42.4 ms mean —
both inside the ~130 ms budget, ~2.7 s before t0. Test (2026-09-30): `pytest
tests/motion/test_skills_executor.py -q` → **150 passed, 4.05-4.09 s**; `pytest
tests/ros/test_skill_node.py -k "refuses_an_uncatchable_feed or at_the_nominal_site_still_installs
or bb_announcement_installs_the_fed_schedule" -q` → **3 passed**.

**(c) sim gate's feed option + rehearsal command.** `sim/skills_gate.py` gains a FEED start for
the columns gate — `SelfTossGateConfig.feed_angle_deg`/`feed_speed_mmps`, a
`run_columns_attempt` branch that backward-integrates a synthetic arrival along BB's real bearing
and compiles via `sk.compile_columns(pattern, feed=sk.LandingPrior(...))` (the same call shape
`SkillNode._install_columns_schedule` uses), plus `--feed-angle-deg`/`--feed-speed-mmps` CLI
flags. *Why*: the day-1 "columns sim gate MET" run drove the R2 vertical self-toss spawn, not the
feed path BB now dispatches through. Test (2026-09-30): `pytest tests/sim/test_skills_gate.py
-q` → **24 passed, 77.76 s**. Rehearsal command (result in Verification: PASS 5/5 seeds at
200 k; the first run at the learner gate's stale 150 k default restarted 4–30 times per seed on
`LIMIT_JERK` at 100–103 % of that cap, so the default now follows `_SESSION_LEG_JERK_MMPS3`):
```
python sim/skills_gate.py --learn --pattern columns --seeds 0 1 2 3 4 \
  --no-viewer --apex-m 0.90 --target-throws 30 --feed-angle-deg 11.9 --feed-speed-mmps 5600
```

**(d) announced-landing bias constants + promoted probe.** `BB_RELEASE_PUSH_LAG_S = 0.037` /
`BB_FLIGHT_BIAS_S = 0.015` (`ball_butler_node.py`, constants after `_G_MMPS2`), applied in
`_publish_throw_announcement` (~809-819) to `throw_time`/`landing_time` only — `ann.
predicted_tof_sec` (raw solver prediction) untouched. Promoted probe:
`tools/probes/feed_catch_bag_probe.py` (new, 574 lines) — parses the launch log and bag
directly, not the sitting's hand-curated table, so it replays unedited on a future log. *Why*:
the ball crosses the catch plane 51.9 ms after the committed landing, not early — release lands
37.2 ms after the announced `throw_time` and the flight runs 14.7 ms long; folding both into the
announcement moves the catch dive onto the true crossing time. Test (2026-09-30): `pytest
tests/ros/test_ball_butler*.py -q` → **89 passed, 0.92 s**. Probe re-run: `python
tools/probes/feed_catch_bag_probe.py --bag ~/Desktop/rosbags/2026-09-30_16-20-05 --log
~/.ros/log/2026-09-30-16-20-04-988735-jetson-2494716/launch.log` → 14 feeds, decomposition
**mean +51.9 ms (15.4 stdev)** — reproduces the report exactly.

**(e) Jetson retry-once.** Bounded, once-per-attempt retry in `_on_bb_throw_outcome`
(`skill_node.py` ~1297) — on `THROW_ABORTED_NOT_SETTLED` with an exact enum-name match,
`ctx.phase == 'await_announcement'` and `not ctx.not_settled_retried`, re-fires via the existing
`_fire_reload_throw` (re-derives its own delay floor and deadline unmodified); a second
`NOT_SETTLED` on the same ctx, or any other code, falls through to the original unconditional
`_end_attempt` path. New `not_settled_retried=False` field on `_reload_ctx` (`_start_reload`
~3071). *Why*: the firmware's abort contract guarantees zero frames sent before a `NOT_SETTLED`
abort (`HandTrajectoryStreamer.h:104-110`), so one retry is safe — it absorbs the false-positive
rate-gate trips (4 of this sitting's 5 refusals sat on a flat, settled plateau) without masking a
genuine second refusal. Test (2026-09-30): `pytest tests/ros/test_skill_node.py
tests/ros/test_skill_node_resend_param.py -q` → **181 passed, 8.26 s** (re-run after unit (b)'s two node tests landed; the unit's own snapshot read 179).

**(f) Ball Butler FW 5.** BallButler repo, `~/Desktop/BallButler`, uncommitted — settled now
requires the old instantaneous `|err| <= 1.0 deg` AND `|filtered rate| <= 12.0 deg/s` (was 3.0)
AND a new history term, `settled_samples >= 15` (100 ms at 150 Hz) inside the error band
(`BallButlerConfig.h:343-345`; counter `YawAxis.h:65,603`/`YawAxis.cpp:822-831`; gate
`HandTrajectoryStreamer.h:99-102`; wired `StateMachine.cpp:1013-1015`; `FW_VERSION = 5`,
`FwUpdate.h:76,80`). Jugglebot lockstep: `BB_FW_VERSION_EXPECTED = 5`
(`teensy_link/rpc_args.py:486-494`). *Why*: `YawAxis.vel_rps` is a 150 Hz finite difference of a
16384-count encoder (one count = 3.296 deg/s raw); a genuinely settled, pure-P axis with
sigma-delta dither on its deadzone crosses an instantaneous 3.0 deg/s gate on noise alone, so the
rate limit alone can't tell dither from a real traverse — the sustained-sample history term can.
**Two corrections found while building it**: a single-count flicker never tripped the old gate
(filters to 0.99 deg/s via the EMA, below 3.0); what tripped it was 4+ counts in one sample, or
2+2 adjacent. Throw #1 — presumed "genuinely still converging" — replays (probe C,
`hb_outcomes.pkl` at 150 Hz) as the same dither on a plateau settled ~0.7 s earlier: FW 5 passes
it, and so would FW 4 on the same trace; refusing it as recorded needs N ≈ 118 samples (790 ms),
exceeding Layer A's 0.6 s wind-up plus 0.1 s schedule margin. Rate ceiling: by the wind-up
convention (12 deg/s held for the full 0.6 s `WINDUP_DURATION_S`) the aim-budget ceiling is 4×
FW 4's (139.0 mm vs. 34.6 mm at 1.1 m), but the history term only passes a *moving* yaw at
<= 8 deg/s while `|err| <= 0.23 deg`. Nothing on the Jetson reads BB's firmware version at
runtime (`teensy_bridge_node.py` compares only `PLATFORM_FW_VERSION_EXPECTED`), so a BB still on
FW 4 against this tree is silent, not darkened. BB-repo logbook entry:
`logbook/2026-09-30-yaw-settle-history-term.md` (BallButler repo). Test (2026-09-30, 18:31):
build `pio run -e teensy40_can` → **SUCCESS, 5.04 s**, 0 warnings/errors, hex md5
`4798642934b60aaa43edee5b985d3aa1` — **built, NOT flashed**. Jugglebot cross-repo: `pytest
tests/firmware -q` → **263 passed, 1 skipped, 12.42 s**.

**Retired: Block C (human-lob columns start)** — code path kept for reference; no further
diagnosis of the tracker silence planned, since BB-led feeds are the target.

## Owner decisions

Verbatim from the 2026-09-30 evening decision record:

- Next sitting shape: feed fixes first, a 0° reload gate, then BB-fed columns in the same
  sitting.
- 0° gate: 10 BB feeds at 0°, ≥9 caught, ≥8 smooth seats (+0.05..+0.15 s band), bag-measured
  landing-vs-committed within ±30 ms, three catches filmed on the owner's high-speed camera.
- Feed catch lever: held-level catch inside columns (planner unit); BB placement (~0.5 m,
  84° lob) is the pre-registered fallback if the 0° gate fails.
- Hand cap may rise toward 3900 if needed — **not needed** (87.5% at 3500).
- Settle refusals: firmware N-sample confirm (FW 5), Claude flashes BB over CAN; plus a
  Jetson retry-once.
- Hold default for plain reloads stays 12° until 0° timing is fixed.
- Jerk ceiling stays 200 000 ("not terribly worrying if not").
- Block C (human lob) retired; human-initiated multi-ball is off the table until BB-led
  patterns work.

## Verification

**The sitting's own gate**: `tests/hardware/session_skills_r5.md` § 11 records the ramp verdict
(HELD at 200 000, no clamp masks, no guard latch) and the block verdicts — Block A caught
13/14 fed balls across 12°/4°/0° with 5 settle refusals; Block B passed with no rebound and no
`HAND_LANE_REFUSED`; Block C 0/3, retired.

**The six Fix units, scoped** (full commands are in Fix, above; all 2026-09-30):

| Unit | Result |
|---|---|
| (a) receive_tilt | 550 passed, 2 skipped, 15.69 s |
| (b) feed check | executor 150 passed, 4.05-4.09 s; node (3 targeted) 3 passed |
| (c) sim feed gate | 24 passed, 77.76 s |
| (d) bias correction | 89 passed, 0.92 s; bag probe reproduces +51.9 ms (15.4) |
| (e) retry-once | 181 passed, 8.26 s |
| (f) BB FW 5 | build SUCCESS 5.04 s; `tests/firmware` 263 passed, 1 skipped, 12.42 s |

A wider `pytest tests/ros/test_skill_node.py -q` shows 19 pre-existing failures reachable from
units (a)/(b), all the same `gate_hash` mismatch (not a regression) — goes to 0 once re-swept.

**Phase-closing triples (main session, 2026-09-30 evening)**:

- **Box re-sweep** — 2026-09-30 19:11–19:56, `bash scratchpad/sweep_run.sh E 200000` then
  `... F 200000` (each run = the columns / hop / single install sweeps of
  `tools/admissible_sweep.py --dwell-s 0.30 --leg-jerk 200000`, BLAS at one thread; logs
  `temp/logs/admissible_sweep_r5_{E,F}_{columns,hop,single}_20260930.log`), merged per run:
  **runs E and F bit-identical** (md5 `0b883c42b4a4fe0fb849d61bebafe13b`), gate `f96fc30012d7`,
  19 boxes, content identical to the previous box apart from `gate_hash`/`swept_at` (the sweep
  never plans a feed catch); installed as `config/generated/admissible_box.yaml`. Node tests
  after the install (2026-09-30): `pytest tests/ros/test_skill_node.py
  tests/ros/test_skill_node_resend_param.py -q` → **181 passed, 8.95 s** (the 19 hash-mismatch
  failures cleared).
- **Sim rehearsal** — 2026-09-30, `python sim/skills_gate.py --learn --pattern columns --seeds 0
  1 2 3 4 --no-viewer --apex-m 0.90 --target-throws 30 --feed-angle-deg 11.9 --feed-speed-mmps
  5600` (`temp/logs/skills_gate_columns_feed_0.90_200k_20260930_2000.log`): **PASS 5/5 seeds,
  1 attempt each, 0 drops, 30/30 consecutive, wall 106.3 s**. The vertical regression run
  (same command without the feed flags, `..._learn_0.90_200k_...log`): **PASS 5/5, 1 attempt
  each, 0 drops, wall 99.6 s**. The first fed run at the stale 150 k default
  (`..._feed_0.90_20260930_1937.log`): FAIL 1/5 (seed 0 longest 29, an attempt-boundary
  artefact), attempts 4/10/6/12/30, every restart `LIMIT_JERK` at 100.2–103.2 % of 150 k on the
  feed catch-and-throw with landing offsets under 2 mm (`sim_feed_attempts_report.md`).
- **`./run_tests.sh --full`** — 2026-09-30 20:04
  (`temp/logs/gate_full_r5_sitting1_fixes_20260930_2004.log`): **PASS — 5784 passed, 9
  skipped, 1 xfailed in 281.32 s; serial phase 6 passed in 19.93 s**. Re-run pre-commit after
  the audit's comment-only fix in `ball_butler_node.py`, 2026-09-30 20:22
  (`temp/logs/gate_full_r5_sitting1_precommit_20260930_2022.log`): **PASS — 5784 passed, 9
  skipped, 1 xfailed in 284.13 s; serial 6 passed in 19.81 s** — the definitive gate for the
  commits that follow.
- **`colcon build --packages-select jugglebot`** — 2026-09-30, `ros_ws`: **1 package finished,
  3.47 s** (`temp/logs/colcon_build_r5_20260930_2004.log`).

The logbook- and plan-index-reading tests (`test_logbook_front_matter.py`,
`test_logbook_search.py`, `test_plans_index.py`) were run separately against this entry's own
frontmatter and index edits — see the session handoff.

## Open Questions

- The 0.35/0.40 s dwell non-monotonic ridge in `held_level_feed_report.md` §1 (peak hand
  acceleration rises 3341→4010.8 rev/s² then drops back to 3064 at 0.50 s) — flagged, not
  diagnosed; candidate mechanism is a knot-grid discretisation effect, unverified.
- The leg heartbeat dropout episodes seen throughout the sitting are the known load-gated
  frame-drop class (`plans/active/leg-bus-frame-drops.md`) — carried, not this entry's.
- The two hop-reload throw-3 drops at 12° (`ABORTED_NO_RELEASE`, `launch.log:1731, 1813`) —
  noted, not traced this sitting.
- Block C's tracker silence: `detect_human_throws` was confirmed live and no lob — including
  near-vertical ones beside the robot — was ever claimed. Not diagnosed; the path is retired
  rather than debugged, per the owner's decision to pursue BB-led feeds instead.
- **BB FW 5 flash**: built (2026-09-30, `pio run -e teensy40_can`, md5
  `4798642934b60aaa43edee5b985d3aa1`), NOT flashed — pending the boards being powered. Flash:
  launch DOWN, BB in IDLE/ERROR, `cd ~/Desktop/BallButler/ball_butler_main && pio run -e
  teensy40_can -t upload`; the receipt is the tool's `FW version: 4 -> 5` line, not a matching
  hex md5. Next sitting: count `THROW_ABORTED_NOT_SETTLED` against this sitting's baseline
  (5/19, 4 on plateaus) — expect ~0 on plateaus.
- **Layer A follow-ups** (carried from the FW 5 build, not this sitting's to fix): the 100 ms
  settle dwell eats into Layer A's 0.1 s `SCHEDULE_MARGIN_S`, which does not reserve it
  explicitly; and Layer A's `YAW_TRAVERSE_DEG_PER_S = 60` assumption predicts 0.29 s for throw
  #1's 17.4° move, against the ~1.2 s the P + tapered-FF approach actually took (~4× slower near
  the target).
- **BB accuracy volley re-fit**: `throw_affine_correction.json` was fitted 2026-06-09 at a BB
  pose/yaw the 2026-09-27 coplanar-marker recalibration has since moved by 10.3°. Needs a fresh
  volley, scheduled for the next sitting, before the 0° reload gate (owner decision, above).
