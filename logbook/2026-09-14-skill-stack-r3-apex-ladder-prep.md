---
title: "Streamed hand lane sends zero torque feedforward where the retired Platform engine sent one — traced R3's +25% overspeed, prepped an apex ladder, and fixed two admissible-box contract defects along the way"
type: investigation
date: 2026-09-14
status: in-progress
phase: "two-ball-skill-stack — R3"
related_plan: two-ball-skill-stack.md
files_changed:
  - plans/active/two-ball-skill-stack.md
  - ros_ws/src/jugglebot/jugglebot/motion/skills/admissible.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - sim/skills_gate.py
  - tests/hardware/session_skills_r3.md
  - tests/hardware/session_skills_r3_apex_ladder.md
  - tests/hardware/skills_plan_bench.py
  - tests/motion/test_skills_admissible.py
  - tests/motion/test_skills_executor.py
  - tests/motion/test_skills_schedule.py
  - tests/ros/test_skill_node.py
  - tools/admissible_sweep.py
  - tools/probes/README.md
  - tools/probes/hand_overspeed_bag_probe.py
subsystem:
  - motion
  - ros
  - sim
  - tools
tags:
  - safety
  - testing
  - performance
---

# R3 apex ladder prep

## Summary

R3's first sitting threw ~25% fast at 0.9 m
(`logbook/2026-09-14-skill-stack-r3-first-powered-sitting.md`). Traced to the
streamed hand lane sending the hand ODrive zero torque feedforward on every
frame, where the retired Platform-Teensy stroke engine sent one on every
frame — an initial "tracking-lag" hypothesis was withdrawn once the old
engine's own bags were checked and showed the opposite signature. Prepared an
apex ladder sitting (0.5–0.9 m) to measure overspeed against commanded
acceleration and set R3's gate apex. On the way, found and fixed two latent
defects in the admissible-box contract (one of which would have aimed a cold
learner's throws off the cup), and promoted a bag-analysis probe to a
committed tool.

## Context

Bags: `2026-09-13_22-57-18` (streamed, R3's first sitting), `2026-08-23_19-14-54`
and `2026-08-21_10-11-42` (Platform Teensy era, pre-R1). Scratch notes with the
full per-stroke tables: `/tmp/claude-1000/-home-jetson-Desktop-Jugglebot-skills/8d633aac-0413-43dc-bd8b-203814044eea/scratchpad/agent_hand_overspeed.md`.
Runsheet for the ladder itself: `tests/hardware/session_skills_r3_apex_ladder.md`.

## Investigation

### Thread 1 — overspeed across eras, and the withdrawn lag hypothesis

Streamed (2026-09-13): meas/cmd hand peak ratio 1.02–1.27 across three
strokes (e.g. 127.6→134.7 rev/s, 127.7→162.1 rev/s), peak `iq` 23–54 A,
torque feedforward **0 on every stroke**; ball landing ≈ encoder ×
32.567 mm/rev within 0.95–1.05 of prediction, so the R1-measured hand gain
itself is not in question.

Platform Teensy (2026-08-23, tiers 85.6–153.6 rev/s commanded): meas/cmd
0.95–1.02, `iq` 10–28 A, torque feedforward 0.03–0.10 N·m (~68% of `J·α`).
Platform Teensy (2026-08-21, 140–177 rev/s, mean accel 1.8–2.8k rev/s² —
*higher* than tonight's 1.7–2.1k): meas/cmd 0.96–1.00, `iq` 25–38 A. Same
drive both eras — hand gains 35 / 0.007 / 0.07 and the 50 A current limit
are identical (`git show 1e2c0c9^:config/hardware_config.yaml` lines
597-600 vs `config/hardware_config.yaml:624-627`); PASSTHROUGH input mode
per `config/ODrive config Files/odrive_pro_hand_config.json:182` (not
re-read off the live drive this session).

The difference is the feedforward path, not the gains or the drive: the
Platform engine packed a torque feedforward (`accelToTorque`,
`Trajectory.h` buildThrow) into every 500 Hz frame
(`git show 1e2c0c9^:Teensy_code_platform/Teensy_code_platform.ino` lines
382-407); the can-bridge sends the hand setpoint with torque `0.0`
(`ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp:1021`), even
though the wire carries and validates a hand `torque_ff` at ingest
(`leg_interp.cpp:299`) — the field exists end to end and is simply zeroed
at the sender. Mechanism: `J·α` at 3000 rev/s² ≈ 1.02e-5 × 2π × 3000 ≈
0.19 N·m needs ≈27 rev/s of velocity-loop error at `vel_gain` 0.007 to
generate on the P-loop alone (≈9 rev/s with the old feedforward carrying
most of the torque); that error, integrated, is paid back as overspeed
once the ramp ends, and current saturation near 50 A compounds it.

**Withdrawn hypothesis** (stated to the owner before this thread closed):
overspeed = `vel_ff` lag + `pos_gain` 35 × tracking error. It does not
survive the Platform-Teensy data — that era ran 1.1–1.4 rev of raw
tracking lag at peak (which would predict +40–50 rev/s of catch-up
overspeed by the same mechanism) and measured ~0 overspeed instead; the raw
lag there is mostly a 6–16 ms pure transport delay, not accumulated
velocity-loop error, so it does not convert into overspeed the way the
streamed lane's error does. The feedforward-gap explanation replaces it and
is consistent with both eras' `iq` and torque-ff columns.

Also checked: the old engine's own "~10% fast" note
(`logbook/2026-08-23-*` era entry) is largely explained by the rev→mm gain
correction (31.628 → 32.567 mm/rev, ~3 points) with the remainder inside
ball-fit method sensitivity (±5 points against that entry's fit) — it is
not the same defect as tonight's feedforward gap.

R1's bag (`2026-09-11_19-36-52`) has no hand motion in `/hand_telemetry`,
so there is no low-apex streamed reference to interpolate from; the ladder
is the only way to get one.

### Thread 2 — the apparent "command surge" is a telemetry artefact

An early read of the streamed bag showed a 57.5 → 57.5 → 118.1 rev/s
command jump in ~21 ms. This is not real: the bridge coalesces its 500 Hz
hand command echo down to one frame per 100 Hz telemetry tick
(`Teensy_code_canbridge/telemetry.cpp:230-280`);
`teensy_bridge_node._on_hand_cmd_echo` discards the echo's own
`t_bridge_us` (`teensy_bridge_node.py:2157-2172`) and
`_publish_hand_telemetry` stamps the message with the Jetson's 100 Hz poll
clock instead (`teensy_bridge_node.py:3486`). Peak values, current, and
mocap positions are robust to this; fitted inter-sample delays and
sub-10 ms accelerations computed from `/hand_telemetry` timestamps are not
and should not be trusted for the streamed lane until `t_bridge_us` is
carried through (filed below).

### Thread 3 — two admissible-box contract defects, found while preparing the ladder

**Defect 1 (executor didn't scope by apex).** `SkillExecutor._command_u`
looked the admissible box up by site pair alone. A 0.5 m self-toss run
against a box swept for 0.9 m silently installed a 0.75 s flight (clipped
up from a nominal 0.639 s) — confirmed by running the pre-fix executor
directly. Fixed: `schedule.apex_m` (the exact inverse of `flight_s`),
`admissible.select(boxes, site_pair, apex_m)`, `load`/`dump` now refuse
overlapping apex bands for one site pair, the executor selects a box by the
*nominal* flight's apex before the learner runs, `skill_node` refuses an
uncovered apex before any motion, and `skills/check` reports per-pair
bands. `start_columns` is unchanged — R2's columns run unclipped by
design.

**Defect 2 (rectangles didn't have to contain the identity offset).** Found
by the first `--single-apex` sweep (`temp/logs/admissible_sweep_apex_ladder_20260914.log`):
`_max_rectangle` returned the largest all-passing rectangle with no
requirement that it contain (0, 0). Every new single-site box excluded the
origin (0.5 m: y pinned at −30 mm; 0.6–0.8 m: x in [−40, −30] mm) — had this
shipped, a cold learner's very first throws at those apexes would have been
clipped off-centre by construction, aimed off the cup before it learned
anything. R3's existing box only contained the origin by accident, because
its three-flight grid happened to miss the bad flight times. Fixed:
`must_contain=(0, 0)` threaded into `_flight_band_and_rect`; two tests
added, `test_max_rectangle_must_contain_restricts_to_rectangles_through_the_point`
and `test_flight_band_and_rect_always_admits_the_identity_offset` (the
latter fails without the fix).

Why the rectangles went off-centre in the first place is filed, not fixed:
at flight times near 0.64 s a ring of small landing offsets (±10–20 mm, 19
cells) fails the chained catch's 90% margin check (`CHAIN_CATCH:MARGIN`)
while (0, 0) and larger offsets pass — seen at 0.6387 s (0.5 m nominal),
0.6423 s (0.7 m at −15%), 0.6463 s (0.8 m at −20%), with smaller three-cell
crosses at 0.77 s and 0.81 s inside the 0.9 m band. The failure tracks
flight time, not apex — it looks like a planner artefact in the pin-blend
family that produced R3's earlier `_TILT_BLEND_MIN_KNOTS` floor bug. The
tradeoff accepted here: the identity-containing rectangle at some of these
apexes can be small (little xy authority left for the learner), but the
ladder measures speed, not xy learning, and flies single throws rather than
the chained catch this margin ring is actually about — so shipping the
smaller-but-correct rectangle now, rather than chasing the margin ring
first, does not block the ladder.

### Thread 4 — re-swept boxes and their reach against the overspeed already measured

Re-swept with `tools/admissible_sweep.py --single-apex` after both fixes
(`temp/logs/admissible_sweep_apex_ladder_20260914b.log`, wall time 253.4 s):

| site pair | apex band (m) | landing xy box (mm) | flight band (s) | reach = nominal ÷ lowest flight |
|---|---|---|---|---|
| (P1, P2) | 0.85–0.95 | [−20, 10] × [−20, 20] | 0.8327–0.8569 | — (chained) |
| (P2, P1) | 0.85–0.95 | [−20, 20] × [−10, 20] | 0.8327–0.8569 | — (chained) |
| (P1, P1) | 0.45–0.55 | [0, 0] × [0, 0] | 0.5109–0.7025 | 1.250 |
| (P1, P1) | 0.55–0.65 | [0, 10] × [0, 0] | 0.5597–0.7696 | 1.250 |
| (P1, P1) | 0.65–0.75 | [0, 0] × [0, 0] | 0.6046–0.8313 | 1.250 |
| (P1, P1) | 0.75–0.85 | [0, 0] × [0, 0] | 0.6463–0.8483 | 1.250 |
| (P1, P1) | 0.85–0.95 | [0, 40] × [0, 0] | 0.6855–0.8569 | 1.250 |

All limits 300/5000/150k mm (vel/acc/jerk), hand acc 3500 rev/s². Every
single-site rectangle contains (0, 0), confirming Defect 2's fix held on
the re-sweep. Reach = nominal flight `2·√(2A/9.80665)` ÷ the box's lowest
admitted flight, using each band's centre apex (0.5…0.9 m); it comes out to
exactly 1.250 on every single-site row because the sweep's fastest tested
cell (`f = −0.20`, i.e. 20% shorter flight than nominal) is what defines
the lower edge of every one of these bands — none of the single-site boxes
were limited by anything tighter than the grid's own floor. That is a
useful ceiling to know going into the ladder: Thread 1's streamed data
already measured a meas/cmd ratio of 1.27 at 0.9 m, which corresponds to a
flight-time ratio *steeper* than this 1.25× floor — i.e. the worst
already-observed sample sits right at, or just past, the edge of what this
sweep even tested for. If the ladder reproduces anything close to that at
0.9 m, the box at that apex is already near its tested limit and will need
re-sweeping wider, not just re-centring.

A feasibility probe confirmed every ladder rung is achievable pre-flight
(2026-09-14, `python tools/probes/skills_single_site_sweep.py --study grid
--apex 0.5 0.6 0.7 0.8 0.9 --dwell 0.30 --jerk 150000 --vel 300 --acc 5000
--hand-acc 3500 --site-xy=-50,0`): all cells OK; commanded peak hand
acceleration single-throw 1706 / 1871 / 2309 / 2715 / 3097 rev/s², chained
1706 / 2044 / 2503 / 2973 / 3341 rev/s² (`temp/logs/apex_ladder_feasibility_20260914.log`).

### Thread 5 — installing the honest box re-opens R3's sim criterion in xy

The dense re-sweep replaced `config/generated/admissible_box.yaml` (columns
boxes, limits and `gate_hash` identical to the committed file; only the
single-site boxes changed). R3's own 0.9 m box went from ±40 × ±30 mm over
flights 0.750–0.857 s to x 0…+40 mm, y 0 over 0.686–0.857 s. The old
rectangle came from a three-flight grid that missed the flights where the
chained catch's small-offset cells fail margin (0.77 and 0.81 s at this apex).

Re-running R3's sim criterion on the new file
(`python sim/skills_gate.py --learn --policy A|B --seeds <s>`, seeds 0–4,
2026-09-14, logs `temp/logs/skills_gate_learn_{A,B}_newbox_20260914.log`):

| Policy | Flight in band by throw | Monotone | Drops | Makes | xy band |
|---|---|---|---|---|---|
| A | 3 (all seeds) | yes | 0 | 25 | never (all seeds) |
| B | 5 (all seeds) | yes | 0 | 24–25 | never (all seeds) |

The sim injects its aim error toward +y (`sim/skills_gate.py:356-381`), and
the new box admits no y correction, so the learner cannot enter the xy band.
Flight learning is unaffected. The 25-throw criterion is run by hand, but a
5-throw version runs in the nightly tier: the first `--full` gate on this
change failed `tests/sim/test_skills_gate.py::test_a_small_self_toss_learner_run_enters_the_band`
on exactly this xy assertion. It is split: `test_a_small_self_toss_learner_run_enters_the_flight_band` keeps flight,
drops and catches (passes), and `test_a_small_self_toss_learner_run_enters_the_xy_band`
is `xfail(strict=True)` citing plan R3 item (k), so it fails loudly the day
xy authority returns. An earlier draft of this entry said no test covered
the criterion; that was wrong.

## Discussion

**(a) Why the ladder before the firmware flash, not after.** The obvious
fix for Thread 1 is to have the can-bridge send a real torque feedforward
the way the Platform engine did. That is a firmware change and the owner's
call, and it needs its own before-curve to evaluate against — flashing
first would mean flying it blind, with no baseline overspeed-vs-acceleration
curve at the *current* operating point to compare it to, and no evidence
for what acceleration ceiling is actually safe to fly meanwhile. The ladder
measures the plant as it stands today; the flash, when it happens, gets
measured against this baseline rather than against nothing.

**(b) Why the runsheet predicts verdict B (rising overspeed) over verdict A
(flat), leaning on the 2026-08-21 numbers specifically.** The 08-21 bag is
the cleanest read against the mechanism in Thread 1 because it ran *higher*
mean acceleration (1.8–2.8k rev/s²) than tonight's sitting (1.7–2.1k) and
still tracked meas/cmd to 0.96–1.00 — i.e. acceleration alone, with the old
feedforward in place, does not explain overspeed. What changed between that
bag and tonight is specifically the feedforward term going to zero, and the
velocity-loop-error mechanism in Thread 1 predicts that the *size* of the
resulting overspeed scales with the torque demand, which scales with
commanded acceleration — rising with apex, since higher apex needs a
faster stroke in the same dwell. That is verdict B's shape. Verdict A
(flat across apex) would only hold if the streamed lane's overspeed were
apex-independent, which the Thread 1 mechanism gives no reason to expect
and the withdrawn lag hypothesis (which *was* roughly apex-independent, a
transport delay) already failed to explain on the 08-21 data.

**(c) The withdrawn lag hypothesis.** Recorded in full in Thread 1: it was
stated to the owner as the working explanation, then withdrawn once the
Platform-Teensy bags were pulled and showed large tracking lag (1.1–1.4
rev) coexisting with ~0 measured overspeed — the opposite of what the lag
model predicts. The data point that killed it was decisive rather than
ambiguous, which is why it was dropped rather than patched with a
secondary correction term.

**(d) The identity-offset tradeoff (Defect 2).** `must_contain=(0, 0)` can
shrink a box to a single point at some apexes (as it did at 0.5, 0.65, and
0.75 m here) rather than the largest all-passing rectangle available.
That is accepted deliberately: a box that maximises area but excludes the
origin is a box that steers every cold-learner throw away from the cup by
construction, and the ladder's purpose is to characterise speed at a fixed
aim point, not to give the learner room to explore xy this run. Widening
these boxes again is downstream of understanding the margin-ring artefact
in Thread 3, which is filed rather than chased now because it is a
separate, apex-independent planner question and does not block flying the
ladder.

**(e) Why the margin ring is filed, not chased.** The ring's failures
track flight *time*, not apex, and recur at three different apex bands'
edges — it reads as one planner-level defect (likely in the same pin-blend
family that produced the earlier `_TILT_BLEND_MIN_KNOTS` bug) rather than
anything specific to the overspeed investigation. Chasing it now would
delay the ladder, which is the actual R3 gate blocker, for a fix that
applies to xy authority the ladder does not exercise.

## Fix

1. `ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py` — added
   `apex_m` (exact inverse of `flight_s`).
2. `ros_ws/src/jugglebot/jugglebot/motion/skills/admissible.py` —
   `select(boxes, site_pair, apex_m)`; `load`/`dump` refuse overlapping
   apex bands for one site pair; `_flight_band_and_rect` takes
   `must_contain=(0, 0)`.
3. `ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py` —
   `_command_u` selects the box by the nominal flight's apex before the
   learner runs, instead of by site pair alone.
4. `ros_ws/src/jugglebot/jugglebot/skill_node.py` — refuses an uncovered
   apex before any motion; `skills/check` reports per-pair bands.
5. `tools/admissible_sweep.py` — `--single-apex` mode (per-apex flight
   grid, `sc.flight_s(A)·(1+f)` for `f` in −0.20…+0.10, apex band A ± 0.05 m).
6. `tools/probes/hand_overspeed_bag_probe.py` — new; per-stroke hand/ball
   overspeed extraction from a bag, primary timing-robust columns plus
   "(adv)" timing-sensitive ones flagged per Thread 2; validated against
   the hand analysis on both the streamed bag and `2026-08-23_19-14-54`
   (`--cmd-source announcement --mm-per-rev 31.628`: 15 rows, meas/cmd
   0.947–1.023, `iq` 10.5–27.6 A). Promoted into `tools/probes/README.md`.
7. `tests/hardware/session_skills_r3_apex_ladder.md` — new runsheet:
   pre-registered predictions (verdict B: meas/cmd ≈1.03–1.06 at 0.5 m
   rising to ≈1.13–1.27 at 0.9 m, knee near 0.8 m; verdict A: flat
   1.00 ± 0.05) and a decision rule (fly up to the highest rung where
   ball/cmd ≤ 0.9× the box's reach and peak `iq` < 45 A; a flat result
   re-opens the analysis; failure even at 0.5 m means flying the
   torque-feedforward flash before any further ladder rungs).
8. `start_columns` left unchanged — it runs unclipped by design (R2), so
   Defect 1's fix does not touch it.
9. Tests: `test_max_rectangle_must_contain_restricts_to_rectangles_through_the_point`
   and `test_flight_band_and_rect_always_admits_the_identity_offset` added
   to `tests/motion/test_skills_admissible.py`; apex-scoped selection and
   refusal-path tests added across `tests/motion/test_skills_executor.py`,
   `tests/motion/test_skills_schedule.py`, `tests/ros/test_skill_node.py`.

- `tests/hardware/skills_plan_bench.py`: `--apex-m` for the self-toss dry
  run and rehearsal (the bench was pinned to 0.9 m, so no ladder rung below it
  could be rehearsed offline); test
  `test_dry_run_self_toss_honours_the_apex_option`.
- `tests/sim/test_skills_gate.py`: the small self-toss learner test split into a
  flight test and a strict-xfail xy test (Thread 5).
- `config/generated/admissible_box.yaml` replaced by the dense apex-ladder
  sweep (Thread 5); `plans/active/two-ball-skill-stack.md` marks R3's sim
  criterion re-opened in xy.

## Verification

- (2026-09-14, `./run_tests.sh`, PASS — 5960 passed, 9 skipped + 3 serial;
  run before the identity-offset fix, by the box-plumbing unit)
- (2026-09-14, `python sim/skills_gate.py --learn --policy A --seeds 0`,
  identical before and after the apex-scoped lookup change: PASS
  `band_xy=3 band_flt=3 attempts=3 drops=0 makes=25`; logs
  `temp/logs/skills_gate_before_apex_boxes.log` and
  `temp/logs/skills_gate_after_apex_boxes.log`)
- (2026-09-14, `pytest tests/motion/test_skills_admissible.py -q`,
  40 passed)
- (2026-09-14, `tools/admissible_sweep.py --single-apex`, first run —
  `temp/logs/admissible_sweep_apex_ladder_20260914.log` — surfaced Defect 2)
- (2026-09-14, `tools/admissible_sweep.py --single-apex`, re-run after the
  `must_contain=(0, 0)` fix — `temp/logs/admissible_sweep_apex_ladder_20260914b.log`,
  wall time 253.4 s — table in Thread 4, every single-site box contains
  (0, 0))
- (2026-09-14, `python sim/skills_gate.py --learn --policy A --seeds <s>` and `--policy B`, seeds 0–4, new box file): every seed FAIL on the xy band only — Thread 5's table.
- (2026-09-14, `pytest tests/motion/test_skills_admissible.py -q -k "max_rectangle or identity_offset"`): 2 passed; `test_flight_band_and_rect_always_admits_the_identity_offset` fails with the `must_contain` argument removed.
- (2026-09-14, `pytest tests/ros/test_skills_plan_bench.py -q -k dry_run`): 3 passed.
- Offline rehearsals (2026-09-14, quiet machine, `python3 tests/hardware/skills_plan_bench.py --rehearse --pattern self-toss --arm A --attempts 3 --n-throws 1 --apex-m A`, log `temp/logs/apex_ladder_rehearse_quiet_20260914.log`): G1 and G2 PASS on all six runs (0.9 m twice, 0.5, 0.6, 0.7, 0.8 m). G4 PASS at 0.5, 0.7, 0.8 m and on the first 0.9 m run; G4 FAIL on the second 0.9 m run and at 0.6 m, one attempt each, both the closing REST refused `LIMIT_JERK` after an accepted catch. Both refused dispatches were under 1 ms late on the 40 Hz tick (0.75 and 0.83 ms); accepted ones were 2.8–23 ms late, except one at 0.60 ms. An earlier run the same day, taken while two sims loaded the machine, showed the same refusal; the load explanation offered then was withdrawn when quiet runs repeated it. **It predates today's changes**: an A/B of five 0.9 m rehearsals each (2026-09-14, same command without `--apex-m`, log `temp/logs/rehearse_ab_9149819_vs_head_20260914.log`) refused once on a detached worktree at 9149819 (R3's docs commit: closing REST after a catch at knot 22, re-gated from knot 67, dispatch 3.75 ms late) and not at all on today's tree. The 3.75 ms case means the refusal window is not only the first millisecond of the tick; the threshold is pinned by the deterministic sweep in Filed item 6.
- Full gate (2026-09-14, `./run_tests.sh --full`): **PASS** — parallel 6001 passed, 9 skipped, 2 xfailed in 279.69 s; serial 6 passed. The first run on this change failed one test, the 5-throw sim learner test on its xy assertion (Thread 5); it was split into a passing flight test and a strict-xfail xy test before this run.

## Filed / Follow-ups

1. **Hand torque-feedforward flash** — owner decision; when it lands, fly
   it against the same ladder for a before/after comparison (Discussion a).
2. **Carry `t_bridge_us` into `/hand_telemetry`** so fitted inter-sample
   timing on the streamed lane stops riding the Jetson's 100 Hz poll clock
   (Thread 2).
3. **The chained-catch margin ring near 0.64 s flight** (`CHAIN_CATCH:MARGIN`
   failures at fixed flight times across three apex bands) — likely a
   pin-blend planner artefact, filed rather than chased (Discussion e).
4. **`start_columns` has no apex-scoped box check** — runs unclipped by
   design (R2); not in scope here, noted for completeness.
5. **Touching apex bands resolve to the first listed box at an exact
   edge** — `admissible.select` does not yet tie-break a query apex that
   lands exactly on a shared band boundary.

6. **Closing REST intermittently refused `LIMIT_JERK` after an accepted catch** — pre-existing (present at 9149819). Fires when the REST is dispatched early in its 40 Hz tick, splicing a knot or two closer to the catch. Ends the attempt after the throw and catch; the catch's own rest tail keeps streaming. Deterministic sweep (2026-09-14, scratch probe driving the same install chain on a virtual clock with the closing REST's dispatch lateness swept 0–24 ms in 1 ms steps; bit-identical on today's tree and on a detached worktree at 9149819):

   | Apex | Refused lateness window | Share of a 25 ms tick | Peak leg jerk (limit 150 000 mm/s³) |
   |---|---|---|---|
   | 0.9 m | 0.12–5.12 ms | 24 % | 303 720 |
   | 0.6 m | 0.36–12.36 ms | 52 % | 181 483 |

   Mechanism: the install quantises the REST's splice to the 25 ms knot grid (`uc.splice_knot(record.meta, tau, LEAD_S)`), and an early dispatch picks a knot still inside the catch's deceleration, leaving too short a settle. The bench's own lateness clusters at 18–23 ms, which is why live rehearsals see it rarely; the robot's tick phase is not guaranteed to. **Fix is the next unit, not folded in here**: the choice is between a minimum splice distance after the catch knot and moving the closing REST's scheduled time a knot later (a longer dispatch lead would move the splice EARLIER with the same splice rule, the wrong direction). Either touches executor timing the columns and chained cycles share and needs the sim gates and rehearsals re-run. The ladder is unaffected: the refusal lands after the throw and catch are recorded.

## Status

in-progress — the hardware ladder sitting is outstanding; this session
prepared the runsheet, the re-swept boxes, and the feasibility probe but
flew nothing.
