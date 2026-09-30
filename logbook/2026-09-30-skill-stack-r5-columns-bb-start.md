---
title: "R5 two-ball columns: the Ball Butler start does not fit one flight from BB's current perch (measured on the planner, the D1 criterion fired); the fused held-catch→re-level→throw window lands for the reload; the columns box cell was the wrong segment; the Stop is a cross-site last throw; the survivor is caught after a drop"
type: investigation
date: 2026-09-30
status: in-progress
phase: "two-ball-skill-stack — R5"
related_plan: two-ball-skill-stack.md
related_entries:
  - 2026-09-29-skill-stack-r4-gate-met.md
  - 2026-09-29-skill-stack-stop-after-a-drop.md
  - 2026-09-29-skill-stack-r4-sitting-3-analysis.md
files_changed:
  - plans/active/two-ball-skill-stack.md
  - config/generated/admissible_box.yaml
  - tools/admissible_sweep.py
  - tools/probes/throw_outcome_bag_probe.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/admissible.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_realize.py
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/segments.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/report.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/ball_tracker_node.py
  - sim/skills_gate.py
  - tests/hardware/session_skills_r5.md
  - tests/hardware/session_skills_r4.md
  - tests/hardware/skills_plan_bench.py
  - tests/motion/test_skills_admissible.py
  - tests/motion/test_cup_cycle.py
  - tests/motion/test_unified_cycle.py
  - tests/motion/test_skills_segments.py
  - tests/motion/test_skills_schedule.py
  - tests/motion/test_skills_executor.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_skill_node_resend_param.py
  - tests/ros/test_ball_tracker_frame_stamp.py
  - tests/sim/test_skills_gate.py
subsystem:
  - motion
  - ros
  - sim
  - tools
tags:
  - kinematics
  - testing
  - performance
---

# R5 two-ball columns, day 1: the start, the box, the stop

## Symptom

R5 opened with a plan text written at kickoff (2026-09-09): "Start phase (hand holds A; B
arrives from the Ball Butler as a CATCH at P2, or is placed)". Four facts from R4 made that
text untestable as written, and the owner's decisions (asked before code, in order) re-scoped
the rung. This entry records the measurements the decisions were made on, what landed, and
what is left.

1. The box admitted no columns throw under the 100 ms pre-release hold (R4 sitting 3).
2. `compile_columns` had no opening REST, so `skill_node._hand_home_error` refused any columns
   start with a displaced hand (R3 carry (c)).
3. Nothing on the robot feeds ball B: the sim spawns it in flight; the Ball Butler ball needs
   the R4 reload's pre-tilt.
4. The two-ball drop policy and the Stop shape were open.

## Diagnosis

### 1. The columns box was empty because the sweep's cell is a segment columns never flies

`tools/admissible_sweep.py::sweep()` gated a cross-site pair with `_throw_cell(P1, P2, T,
dwell)`: a THROW seeded at P1's rest that releases at P2 inside one 0.3 s window. The pattern
(`schedule.compile_columns`, `_fold_catch_throw_pairs`) never plans that segment: its first
skill is a launch from rest at its OWN site over 0.4 s, every later throw rides a
CATCH-with-throw whose window is the transit τ from the OTHER site's release, and the last
catch is a standalone transit LANDING. Probe A (scratchpad `probe_columns_cells.py`, limits
300/5000/150 000/3500, apex 0.9 m, 100 mm, dwell 0.30):

| cell | hold 0 | 0.05 | 0.10 |
|---|---|---|---|
| sweep's rest → transit → release cell | OK | MARGIN | INFEASIBLE |
| launch from rest at own site (0.4 s) | OK | OK | OK |
| transit catch-with-throw at (0, 0) | MARGIN, leg jerk 136 k | MARGIN 138 k | MARGIN 144 k |
| same at (0, +10…+30) mm | OK 125–133 k | OK/MARGIN | OK |
| same at (±10, 0), (0, −10…−20) | MARGIN 137–141 k | MARGIN | MARGIN |
| same at (+20…+30, 0) | LIMIT_VEL | LIMIT_VEL | LIMIT_VEL |

The old cell passed by letting the cup slide through the release, which the hold forbids. The
real segments pass every hold; what binds them is the 100 mm transit in 0.278 s, at 87–98 % of
the leg jerk limit against the sweep's 90 % margin. R3 carry (a), "the columns jerk creep", is
this. A longer dwell shortens the transit (τ = (t_f − d)/2) and makes it worse. Unit B's
origin-cell table on the re-modelled cell: only apex 0.85 m / dwell 0.30 s admits the identity
command at the 90 % margin; 0.90 m is MARGIN; 0.95 m refuses `HAND_LIMIT_ACC`.

### 2. A Ball Butler ball cannot be caught inside a columns transit

From its perch (mocap (−1019, −435, 1738) mm; R4 gate log 2026-09-29: pitch 69.7°, 3.34 m/s,
ToF 0.881 s) the ball arrives at P2 at (1073, 436, −5507) mm/s: 5.6 m/s, 11.9° off vertical.
Probe B, a banking CATCH in a 0.278 s transit from a post-release seed:

| arrival | verdict |
|---|---|
| 11.9° (BB) | REFUSED `LIMIT_VEL` 351.5 > 300, cup contact −7039 < −6864 mm/s² |
| 5° | REFUSED `LIMIT_VEL` 302.2 > 300 |
| 2° | MARGIN (jerk 148 k) |
| vertical | MARGIN (jerk 139 k) |
| held-axis catch from the level post-release seed | REFUSED `CATCH_AXIS` |

So B needs the 12° receive attitude, and the held-axis form needs a seed already on the axis.

### 3. The owner's six-step start, step by step

Owner (2026-09-30): platform at P2 holding a ball; BB throws to P1; Jugglebot throws its ball
vertically; the platform moves to P1 while orienting; catches BB's ball, re-orients, throws it
vertically; columns runs. Probes C/D (`probe_reattitude.py`, `probe_owner_sequence.py`), floors
per step on the real planner:

| step | floor today | binds | at the YAML ceilings (vel 1000, jerk 200 k, hand 3900) |
|---|---|---|---|
| vertical launch from rest | 0.857 s flight at 0.9 m; 1.0 m refused `HAND_LIMIT_ACC` at 3500 | hand | 0.903 s at 1.0 m |
| transit P2→P1 + 12° tilt (REST, `rest_tilt`) | 0.50 s | `TILT_PIN` (the tilt-acceleration budget), then `LIMIT_VEL` / `HAND_LIMIT_C2` | 0.40 s |
| held-axis catch from the tilted rest | 0.20 s (0.15 infeasible; 0.30 a solver artefact) | hand 2942 rev/s² | 0.20 s |
| re-orient + vertical throw from the tilted seated rest | 0.40 s (`LIMIT_JERK` 243 k at 0.30) | leg jerk; the re-level rides inside the launch window | 0.40 s |
| transit + catch of ball 1 | 0.278 s | legs | 0.278 s |
| sum vs the flight | 1.38 s vs 0.857 | | 1.28 s vs 0.903 |

`TILT_PIN`'s message: "a 11.877° attitude change over 0.400 s needs 7.480 rad/s² of tilt
acceleration, past the 5.222 rad/s² limit — give the REST at least 0.479 s." That cap is
`cup_realize.TILT_ACCEL_BUDGET_FRACTION × leg_acc / TILT_ACCEL_LEVER_MM` (469 mm), a static
lever model rebuilt per call by `unified_cycle.build_realize_config`; the legs ran at 1135 of
5000 mm/s² and 58 k of jerk through the 0.5 s re-tilt. Lifting the budget ×2 buys 0.05 s on
the re-tilt and breaks the re-orient-and-throw window (`LIMIT_JERK` at every length). The cup
sits ~0.75 m above the tilt centre, so 12° is a ~150 mm cup excursion, and the start needs two.

### 4. Unit F-a: the fused window, and the D1 criterion

F-a (Opus, no repo edits, ten scratchpad probes monkeypatched onto the real QP) designed the
fused held-catch → re-level → throw as rows in the one QP (the held line slaved over
`[k_a, k_td]` only; the xy jerk box dropped on that span; the lateral release rows written;
the tilt schedule constant at the hold to the touch-down then a quintic to the take-off tilt)
and measured it:

| limits | ball 1 | hold | fused approach + dwell | + τ | total | margin |
|---|---|---|---|---|---|---|
| session | 0.9 m (0.857) | 11.9° | 0.625 + 0.40 | 0.278 | 1.303 | −0.446 |
| ceilings | 1.0 m (0.903) | 11.9° | 0.575 + 0.35 | 0.278 | 1.203 | −0.300 |
| ceilings | 1.0 m | 6° | 0.355 + 0.30 | 0.278 | 0.933 | −0.030 |
| ceilings | 1.0 m | 4° | 0.365 + 0.25 | 0.278 | 0.893 | +0.010 (10 ms islands; `LIMIT_JERK` 202 k > 200 k between) |

The floors depend on the HOLD angle, not on how steep the arrival is; catching BB's ball at P2
(no transit) passes no cell at all. The pre-registered D1 criterion fired.

### 5. Unit F-b: the seam F-a's probe could not see

F-b built the reload-only fused window (k_a ≡ 0). F-a's slew landed on the release knot;
`cup_realize._knot_derivative`'s one-sided 3-knot stencil does not differentiate a quintic's
(1−u)³ approach to zero, so the release knot carried ry −0.036 rad/s and x +3.99 mm/s, and
`extend`'s seam re-gate read 315 107 mm/s³ against 133 k for the STEADY alone — a velocity step
the Teensy would have streamed. Fix: `HELD_SLEW_LANDS_BEFORE_RELEASE_KNOTS = 2` (the slew
reaches the take-off tilt two knots early; the stencil's three knots are equal; the release
velocity is exactly 0). Re-probed floors through the tail and `extend` (`probe_fb_floors2.py`):

| dwell | session 300/5000/150 k/3500 | ceilings 1000/5000/200 k/3900 |
|---|---|---|
| 0.40 | `LIMIT_JERK` 182–219 k | OK at w 0.30 (T 0.70), j 182 k |
| 0.45 | `LIMIT_JERK` 152 k | OK at w 0.20 (T 0.65), j 152 k |
| 0.50 | OK at w 0.20 (T 0.70), j 110 k | OK |

Floor 0.70 s (session) / 0.65 s (ceilings) against 0.90 s for today's R4 reload (catch window
0.20 + its 0.30 s rest tail + a 0.40 s throw), i.e. −0.20 / −0.25 s per reload. The 4° start
cell therefore misses by 40–90 ms at every ceiling.

### 6. Unit E: the first Stop could not cross the sites

E built D2 as a REST at the last ball's site timed to its landing, spliced after the previous
catch's tail; the window left is 0.228 s for 100 mm from rest: `LIMIT_VEL` 315 > 300 in
isolation, and `test_a_six_throw_columns_schedule_installs_end_to_end` failed its rest-terminal
assertion (0.88 mm/s residual). A carry with the seated ball over the whole beat would need
~10° of banking. TODO(main): E-2's redesign (the cross-site last throw) and its probe verdict.

### 7. The sweep's second solves were not "the same window"

Run 1 of the re-sweep crashed the hop ladder after 911 rows: `_hop_catch_cell` re-solved the
catch-with-throw raw, for the next release seed, with goals that omitted the hold-knot fields
its production solve carried, and the raw solve refused `HAND_LIMIT_ACC` 3552.8 > 3500 on a
seed the production solve had just accepted. Unit B had met and patched the same gap in the
new columns cell, and the main session in `_chained_catch_cell`; the hop copy was the third.
The fourth copy is worse than a crash: `_throw_cell`'s raw `plan_launch` for the release seed
omitted the pre-release hold and the settle site, so every chained cell swept since the hold
landed (the 2026-09-28 and 2026-09-29 boxes, gate hashes `3fda47b2ad5b`, `bf095653d422`,
`ad36fa53aab2`) was seeded from a launch the machine does not fly. The class fix: the segment
records its own post-release state (`segments.Segment.release_state`, set by `_plan_throw` and
`_plan_catch_throw` before the SETTLE tail is joined), and the tool's four raw re-solves are
deleted — one enforcement point, no twin solve to drift. The columns box from the aborted run
(invalid, but indicative): at 0.85 m the admitted landing offsets were x ±1 mm and y −4…+8 mm;
0.90 and 0.95 m empty.

### 8. The corrected box at 150 k is a needle; the sim says 0.90 m; the owner ramps to 200 k

Run 1 on the corrected tool (gate hash `581109806b4e`, limits 300/5000/150 000/3500, dwell
0.30, per-apex bands of ±0.025 m; columns 7.4 min, hop 13.0 min):

| box | 0.85 m | 0.90 m | 0.95 m |
|---|---|---|---|
| columns (P1,P1) | x −0.5…+1.0, y ±4 mm, apex pinned 0.85 | EMPTY | EMPTY |
| columns (P2,P2) | x −1…0, y −2…+8 mm, apex pinned 0.85 | EMPTY | EMPTY |
| hop P1→P2 / P2→P1 | x 0 (a line), y ±20 | x −8…+6 / −4…+8, y −1…+20 / −8…+20 | x ±10, y ±20 |

The hop's 0.80 m row admits x −10…+4 / −2…+10, y 0…+20 / −8…+20. So at 100 mm the columns
learner would have no authority on any axis, and a plant that throws ~10 % high could not be
pulled down from a pinned 0.85 m command. Unit G's sim run says the opposite of the box: at
0.90 m the columns learner run is clean (30/30 × 5 seeds, 0 drops) with `boxes=None`, i.e. on
the 100 % runtime gate alone, while 0.85 m is the flaky point (mid-schedule `LIMIT_VEL` /
`LIMIT_ACC` refusals). The pattern lives between 90 and 100 % of a 150 000 mm/s³ session limit
that was chosen on 2026-09-16 as "safe enough", not measured; the R2 sizing point and the YAML
ceiling are 200 000. Owner (2026-09-30): ramp the session leg jerk to 200 000, re-sweep the
whole box at it, keep the launch default at 150 000 until the ramp is logged, and make the
sitting's first block the ramp measurement on self-toss and hop.

### 9. At 200 k the 0.90 m band opens laterally, and the ladder's timing model pins the apex

Run A at 300/5000/200 000 (columns 7.8 min, hop 15.1 min): columns (P1,P1) at 0.90 m admits
x −2…+4 / y −20…+10 mm, (P2,P2) x −4…+1 / y ±20; 0.85 m x ±0.5 / y −10…+4; 0.95 m empty; the
hop's bands widen to x ±8…10 / y ±20 (0.85 stays a y-only line). Every `apex_m` bound is a
point at its band's centre, and the reason is the ladder, not the legs: `_single_apex_boxes`
varies the FLIGHT for every cell, and the chained cells derive their timing from it (`tau =
transit_s(T', dwell)`, `t_land = T'`), so a commanded apex below the pattern's shortens the
transit and refuses on jerk. That was the R3 semantics, when the command was the flight time.
Since 2026-09-18 the command is an apex: the schedule's catch instants and transits come from
the PATTERN's apex (`compile_columns`, `compile_one_ball`), the throw's launch follows the
command (`executor._throw_terminal` / `_catch_terminal`, `flight_s = flight_s(u_apex)`), and
the next catch is aimed at the schedule's instant (`_predicted_landing` reads `y_d[1]`). The
sweep therefore refuses commands the pattern would fly, and it is why the R4 reload hops were
floored at 0.85 for a 0.9 m target while the plant threw ~10 % high. Unit L separates the two
flights (the pattern's for timing and arrival, the command's for the carried throw) before the
installed sweep.

## Discussion

**Why the sweep cell was re-modelled rather than the hold made per-pattern.** Carry item 1
framed the choice as "a shorter hold for columns, or a longer dwell". Both answers keep a cell
that measures nothing the pattern does; the wrong segment shape was the defect, and a
per-pattern hold is a second path (plan § 0). The re-modelled cell is the launch from rest,
the transit catch-with-throw and its reverse leg, intersected — the same three-gate shape the
single-site branch already used. Cost: the box is honest about a pattern at the jerk margin,
so the owner's 0.9 m operating point is MARGIN at 100 mm and the box will say so.

**Why the BB start was measured on the planner before any design was proposed, and what the
measurements changed.** My first framing (a human lob; BB out by physics) rested on a
1/T³ scaling of the reload's decay and a quintic estimate of a 12° slew at a 200 mm radius —
both withdrawn below. The owner's pushback ("the platform can reorient extremely quickly")
was right about the legs (23 % of their acceleration budget in the 0.5 s re-tilt) and the
probes located the real binder in the planner's tilt budget and, past it, in the cycle
itself: two ~150 mm cup excursions plus two transits, a catch and a throw inside one 0.9 s
flight. The number that decides is the arrival angle. From 1.1 m away every BB lob is ≥ 12°
unless it arrives faster than the hand can match; within ~0.25–0.35 m of the site it is ≤ 5°
and the stock columns catch nearly takes it level (5° refused by 2 mm/s at 300 mm/s).

**Why the fused window was built for the reload even though the start does not close.** It
is the subset every BB-fed pattern needs, it removes a full stop from every reload (0.90 →
0.70 s), and its one contract change — the re-level slew inside a held STEADY is shaped by the
tilt budget and refused by `validate_cycle` alone, the rule a launch from a tilted seed already
follows — is stated in `cup_realize._smooth_slew`'s docstring, the only place the check was
ever normative; SETTLE and LANDING slews stay checked. The 2-knot early landing was kept over
an analytic tilt-rate channel or a higher-order end stencil because either changes every
plan's end velocity and breaks bit-for-bit parity planner-wide; the early landing costs
0.05–0.10 s of dwell and buys an exactly-zero release velocity.

**Why the Stop is a cross-site last throw.** The owner's D2 wording ("the platform moves
under it as though it were being caught, hand low") does not require the platform to carry a
ball; aiming the last throw at the site the held ball is already at puts the platform under
the landing by construction and leaves the closing REST unchanged. The carrying REST was ruled
out by measurement (§ 6); a hand-pinned CATCH kind was ruled out as a planner change for a
non-resumable end state.

**Tradeoffs accepted.** The owner chose a programme my numbers said was 0.05–0.1 s short at
every ceiling, with the sim as arbiter and BB placement as the pre-registered fallback; the
measurement came back worse (0.30 s short at 12°, 40–90 ms at 4°), and the owner held F-c
pending a cup test. R5's hardware gate therefore waits on either the cup test plus a
re-attitude the numbers do not support, or on where BB can stand.

## Fix

- Unit B (Sonnet): `_columns_catch_throw_cell`; the cross-site branch of `sweep()` gates the
  launch from rest at the target site, the transit catch-with-throw and the reverse leg; every
  box carries `dwell_s` (`AdmissibleBox`, dump/load, `check_limits(dwell_s=…)`);
  `_single_apex_boxes(pattern=…)` gives columns and the hop per-apex bands; `HOP_APEXES_M`
  gains 0.80 m; the raw second solve in the new cell builds the same goals as the production
  solve (hold knots included). Main session: the same one-line fix in `_chained_catch_cell`.
- Unit F-b (Opus): `CatchEvent.held_from_k`; the "can't also throw" `CATCH_AXIS` refusal is a
  `HOLD_WINDOW` rank refusal (≥ 3 + N free lateral jerks after the span); slaving and the xy
  jerk box over the held span; `tilt_schedule(held_from_k=…)`; `_smooth_slew(checked=…)`;
  `HELD_SLEW_LANDS_BEFORE_RELEASE_KNOTS = 2`; `hold_tilt` allowed on a STEADY;
  `CatchTerminal` accepts `then_throw` + `hold_tilt`; a `held_from_k > 0` start is refused by
  name until F-c. LANDING programs and segments bit-identical before and after (fingerprints).
- Unit E-2 (Sonnet): `compile_columns` aims the LAST throw at the other site (`Skill.
  shadow_landing` / `ThenThrow.shadow_landing`; E's shadow REST and its inference deleted);
  the executor bypasses the learner and the box for that throw, finalises it `caught=False`
  with no memory row and "landed on the held ball"; the survivor policy: on a
  no-release-evidence END for ball X with another ball still due a CATCH, the executor
  rewrites its schedule to that catch (throw stripped) plus a closing REST and ends
  `DROPPED_SURVIVOR_STOPPED`; `skill_node`'s two `check_limits` calls pass the live dwell.
- Unit S / S-2 (Sonnet): `compile_reload(max_tilt_deg=…)` + the `hold_tilt_max_deg` node
  parameter (4° → the pre-tilt REST and the held CATCH both at 4.000° for the R4-log arrival);
  `detect_human_throws` a live tracker parameter; the D2 end line; `_log_attempt_end` /
  `_goal_end_code` read `end_code` (they gated on `attempt_ended`, which D3 keeps False, so
  the survivor code was dropped from the Juggle result); the feed-triggered start:
  `compile_reload_wait(holds_ball=…)` as the bridge, `compile_columns(feed=LandingPrior)`
  with t0 = t_land − τ, `_reload_ctx.kind='columns'` resolved by a BB announcement
  (`reload=True`, the "columns until R5" refusal lifted) or an un-announced tracker fit
  within ±10 x / ±20 y mm of P2 with lead ≥ τ + launch + LEAD_S, `ABORTED_NO_COLUMNS_FEED`
  at 30 s; `_hand_home_error` deleted (the bridge homes the hand, R3 carry (c)).
- Main session: `_stop_attempt` clears the feed/reload context in its executor branch too
  (a Stop while the bridge REST's executor was still live left the wait armed and the Juggle
  goal hung until the wait's deadline; unit T found it).
- Unit G (Sonnet): the sim gate's Stop assertion (`FINAL_CUP_XY_TOL_MM` 25) and
  `--learn --pattern columns --apex-m --target-throws`; a missing `observer=` fixed on the way.
- Unit R (Sonnet): `tests/hardware/session_skills_r5.md` (50 rows; a 200 k ramp block with a
  pre-registered stop rule; Block A the 4° cup test; B the fused reload; C columns from a
  lob); the bag probe's plane pin derived from config (`FSM_ERA_CATCH_Z_MM`), no literal.
- Unit L (Sonnet): the sweep ladder's `pattern_flight_s` (Diagnosis § 9).
- Sweep install: TODO(main).

## Verification

- 2026-09-30, baseline `./run_tests.sh -q` at a94b6ae4: **PASS, parallel 5669 passed /
  13 skipped in 216 s, serial 3 passed in 12 s** (scratchpad `baseline_gate.log`; the count
  tallied from the progress marks because `-q` doubled pytest's quiet flag).
- 2026-09-30, unit B, `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python -m pytest
  tests/motion/test_skills_admissible.py -q`: **59 passed before, 68 passed after**; with the
  old tool restored under the new tests exactly the 12 new tests fail. Main session's
  `_chained_catch_cell` fix: **68 passed**.
- 2026-09-30, unit F-b, `… python -m pytest tests/motion/test_cup_cycle.py
  tests/motion/test_unified_cycle.py tests/motion/test_skills_segments.py -q`: **205 passed /
  2 skipped before, 211 passed / 2 skipped after**; against a shadow copy of the four
  pre-change planner files the six new tests fail (6 failed, 205 passed); with the early-landing
  constant forced to 0 the STEADY and segment tests fail (seam 224 749 > 200 000).
- 2026-09-30, unit G, `python sim/skills_gate.py --learn --pattern columns --seeds 0 1 2 3 4
  --no-viewer --apex-m 0.90 --target-throws 30` (`temp/logs/skills_gate_columns_learn_0.90_20260930.log`):
  **PASS 5/5 seeds, 0 drops, 30/30 consecutive catches on every seed, wall 103.0 s** — with
  `boxes=None`, because the committed columns box is empty: the 100 % runtime gate alone
  carried the learner's commands. The same at `--apex-m 0.85`: 0 drops on every seed but 2–3
  cold resets per seed on mid-schedule `LIMIT_VEL` / `LIMIT_ACC` refusals and isolated
  `caught=False` rows with no physical drop (longest run 12–23; flagged, not root-caused).
  `python -m pytest tests/sim/test_skills_gate.py -q`: **15 passed before, 20 passed after**
  (64.56 s). The Stop assertion reads 0.04 mm of cup error at the last landing and 100.04 mm
  with the pre-E1' own-site target monkeypatched back.
- 2026-09-30, unit E-2, scoped: `tests/motion/test_skills_schedule.py` **98 passed**,
  `tests/motion/test_skills_executor.py` **146 passed** (fail-before by temporary reverts);
  the cross-site last throw probed on the real planner first (accepted at 0.9 m / 0.30 /
  100 mm, MARGIN — irrelevant, the shadow throw bypasses the box).
- 2026-09-30, units S / S-2 / T / T-2, `python -m pytest tests/ros/test_skill_node.py -q`:
  S's 4° cap and end-code tests fail-before/pass-after; S-2's seven feed-wait tests; unit T
  re-fixtured 27 pre-existing columns tests (122/46 → 149/19); T-2's stop-during-feed-wait
  test fails against the pre-fix `_stop_attempt` (monkeypatched) and passes after: **150
  passed, 19 failed**, the 19 all on the committed box's stale `gate_hash` against the
  working-tree planner (cleared by the re-swept install below). `tests/motion/
  test_skills_schedule.py` 104 → **110 passed**. Tracker: 33/33 in its three files.
- 2026-09-30, unit L, `tests/motion/test_skills_admissible.py`: 68 → **71 passed** (a real-solve
  ladder at 200 k admits a commanded apex below 0.90 m).
- Sweeps, all `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python tools/admissible_sweep.py
  --site-pairs {columns|hop|single} --single-apex … --dwell-s 0.30 --leg-jerk <J>` (columns
  0.85/0.90/0.95 and hop 0.80–0.95 with `--single-apex-halfwidth 0.025`; single 0.5–0.9), three
  in parallel per run, logs `temp/logs/admissible_sweep_r5_<run>_<pattern>_20260930.log`:
  - run 1 at 150 k on the first corrected tool (Diagnosis § 8): columns 7.4 min, hop 13.0 min;
  - run A at 200 k, before the ladder fix (Diagnosis § 9): columns 7.8 min, hop 15.1 min;
  - **runs C and D at 200 k on the fixed ladder, 2026-09-30: bit-identical on all three files
    (`cmp`), 12 180 + 15 470 + 16 240 = 43 890 verdict rows each; columns 12.0 / 11.7 min, hop
    17.7 / 17.0 min, single 22.2 / 21.9 min;** run C merged (19 boxes) into
    `config/generated/admissible_box.yaml`, gate hash `581109806b4e` = the live hash at install.
- 2026-09-30, after the install: `python -m pytest tests/ros/test_skill_node.py -q` **169
  passed** (the test file's live status and its box builders now stamp 200 k, its node factory
  publishes the status, six columns starts pass the 100 mm separation, and two tests were
  rewritten to the two-phase start); `tests/ros/test_skill_node_resend_param.py` +
  `tests/ros/test_skills_plan_bench.py` + `tests/sim/test_skills_gate.py`: **93 passed** after
  the bench's self-toss jerk followed the box to 200 k and the hop learner test's 0.85 m floor
  (a ladder artefact) was rewritten.
- Full gate, `./run_tests.sh --full`, run 2026-09-30 15:10–15:16
  (`temp/logs/gate_full_r5_day1_final2_20260930_1510.log`): **PASS — 5760 passed, 9 skipped,
  1 xfailed in 282.42 s (parallel), serial 6 passed in 19.73 s, 308 s total.** The two runs
  before it on interim trees (`…final_20260930_*.log`, `…day1_20260930_1458.log`) failed only on
  the node-fixture population described above, and are kept as the record of that.

## Withdrawn claims

- [2026-09-30 ~11:30] "The columns box is empty because the 100 ms pre-release hold does not
  fit the 0.3 s dwell." WITHDRAWN: the sweep's cell modelled a rest → transit → release throw the
  pattern never flies; the real segments pass every hold and are jerk-bound by the transit.
  Superseded by Diagnosis § 1.
- [2026-09-30 ~12:15] "A BB-fed start needs weeks: a ~4× jerk ramp and a new primitive; no
  leg ramp this machine has run reaches it." PARTLY WITHDRAWN: the legs are not what binds the
  re-attitude; the planner's tilt budget is, and a re-level rides inside a launch window at no
  cost. What stands is the cycle arithmetic (§ 3, § 4). Superseded by Diagnosis §§ 3–5.
- [2026-09-30 ~12:30] "A 12° slew alone in 0.28 s takes ~280 of the 300 mm/s and ~115 k of
  the 150 k mm/s³." WITHDRAWN as a mechanism (a quintic estimate at a 200 mm radius; the
  measured 0.5 s re-tilt runs at 156 mm/s and 58 k). Superseded by Diagnosis § 3.
- [2026-09-30 ~15:00] "The fused catch-throw floor is 0.60 s" (F-a). WITHDRAWN by F-b: the
  probe planned the STEADY alone and never saw the seam; through the tail and `extend` the
  floor is 0.70 / 0.65 s. Superseded by Diagnosis § 5.
- [2026-09-28/29, carried into today] "The hop box widened under the hold (x upper +2 → +10
  mm) and the self-toss boxes narrowed (±40 → ±30 mm)" — those widths were measured on chained
  cells seeded from an un-held raw launch (Diagnosis § 7). The re-sweep on
  `Segment.release_state` is the first box whose chained cells start from the segment the
  machine flies; the earlier widths are provenance for the tool defect, not for the plant.
- A probe bug, recorded so it is not repeated: the first owner-sequence run passed a
  hand-built realize config into held-tilt windows, forcing banking on, and read `LIMIT_VEL`
  on a catch whose platform barely moves; fixed by scaling `TILT_ACCEL_BUDGET_FRACTION` and
  letting `plan_cycle` build its own config.

## Open Questions

- The 4° cup test (owner): a `hold_tilt_max_deg` cap on the R4 reload, watched for the seat
  with the ball entering ~8° off the cup axis.
- Where Ball Butler can stand: the start closes with the stock catch when the ball arrives
  within ~5° of vertical at ≤ 4.5 m/s (a release point within ~0.25–0.35 m of the catch site
  at BB's current height).
- F-a's untraced `HAND_LIMIT_C2` on the longer slewing windows; the same one-sided stencil at
  slew joins is the candidate (F-b § 0).
- The columns operating point: 0.90 m is MARGIN at 100 mm on the re-modelled cell; the sweep
  tables decide between 0.85 m and a margin decision.
