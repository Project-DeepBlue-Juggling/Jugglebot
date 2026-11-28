---
title: "JP-11 rotating Columns U3: a rotating Throw's Learner command is checked on its ball's next Catch turned back onto the dispatched Catch (stand-in rule (iv), I-ADMIT-1 amended); before it every rotating Throw was envelope-only. The exact next Segment is not computable at command time; the turn costs up to about 0.03 margin at 10 deg per Beat and 0.15 at 20"
type: feature
date: 2026-11-27
status: in-progress
phase: "JP-11 rotating Columns U3"
related_plan: "pattern-progression.md"
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - tests/motion/test_skills_executor_online.py
subsystem: [skill-stack]
tags: [pattern, rotating-columns, jp-11, admissibility, learner]
---

# JP-11 rotating Columns U3: the stand-in rule for a rotating Throw

## What

The command-time admissibility check (I-ADMIT-1) judges a Learner command `u` for a carried Throw by planning a STAND-IN Segment from the seed the install will use. A rotating Columns Throw leaves S(k) for S(k+2), a Site turned 2 x step about the Platform origin, so `_standin_rule` gave it rule (iii): no stand-in, the throw envelope alone. Every Throw of a rotating Attempt had no step-back: a Learner command out of margin was sent as commanded, and the first sign of it was the 100 % install gate refusing the ball's next Catch, which ends the Attempt. New rule (iv), for the `columns_rotating` label only: the ball's next Catch is built exactly as rule (i) builds it at the TARGET Site (landing = target + predicted miss, clamped there about the target's outward unit, its Throw aimed by `u`), and that whole Segment is turned back by the Site-to-target angle onto the dispatched Catch. One plan and gate per evaluation, at most three, as before.

## One control cycle with the change

Learner proposes `u` for Throw k (Catch k at S(k), Throw to S(k+2), rotating stream at step D; Catch k is being dispatched, its seed is the streaming record's splice state on Segment k-1, at the release of Throw k-1 from S(k-1)).

1. `_resolve_catch_command` (dispatch, or `_early_command` up to `EARLY_COMMAND_MAX_S` before it for a handoff Catch) computes the candidate and the model command, unchanged.
2. `_standin_rule`: target name differs from the Site, release Site is the Catch Site (same-site fold), not point-symmetric, label `columns_rotating`, both Sites on one circle about the origin within 1 mm, turn `theta` = 2D (10 / 20 / 40 deg at 5 / 10 / 20 deg per Beat) within `STANDIN_ROTATE_MAX_DEG` = 40 deg: rule (iv).
3. `verdict_fn(u)`: (a) the envelope on the REAL Throw k (release at its real release point, aimed at S(k+2) + `u`), unchanged; a refusal costs no solve. (b) the stand-in: predicted landing `y_hat(u)` at S(k+2), clamped by the lateral authority about S(k+2) + `y_d` along S(k+2)'s outward unit (Robot frame, exactly the clamp Catch k+2 will apply), then mapped onto this Catch: landing = S(k) + R(-theta)(clamped - S(k+2)), release riding it, Throw aimed at S(k+2) + R(-theta) `u` (the image of S(k+4) + `u`), this Catch's clock, arrival velocity, rest site and tilts. (c) `self.checker(CATCH, standin, t)` sends it to the solver worker (`check_only`), which plans it from the seed the install of Catch k would use and returns `admissible.margin` at `MARGIN_FRAC` 0.9.
4. `admissible.resolve_command`: a pass sends `u`; a refusal steps toward the model command to the margin's zero, backed off 1 mm, and confirms with one more solve (at most three); a refused model command, or an unevaluated check, ends the Attempt `NO_ADMISSIBLE_COMMAND` before any install. All unchanged.
5. One `ADMISSIBILITY skill k: rotating stand-in (R4@30 -> R6@50, next Catch turned back 20.0 deg): u=... passed m=...` line (or `refused ... -> stepped to ...`).
6. The command is cached for the skill; Catch k's REAL terminal (its real landing, the Throw aimed at S(k+2) + the resolved `u`, unrotated) is installed through `install_segment`, gated at 100 % by `validate_cycle`, and the Teensy `MAX_DEVIATION` guard still watches every Setpoint. Nothing on the install or wire path changes; Catch k+2 is gated at 100 % at its own install as before.

Timing is unchanged: the stand-in has the same knot count and window as Columns' rule (i) stand-in. In-process CPU per stand-in check at the P4 point (600 / 8000 / 200000 / 4500, model command, Catch 4, 12 repeats, BLAS pinned to one thread, 2026-11-27): 30.9 ms for Columns rule (i), 33.1 / 33.2 ms for rule (iv) at 10 / 20 deg per Beat, against the 45 ms worker CPU bound of `tests/motion/test_check_segment_budget.py`.

## Discussion

**The exact check the brief asked for first is not computable at command time.** The true Segment that `u` binds is Catch k+2's: the transit from Throw k+1's release at S(k+1) into S(k+2) + miss. Its seed is the release state of Segment k+1, which depends on (1) Ball k-1's landing at S(k+1) (a same-site fold releases from where the ball was caught), still in flight and unfitted at Catch k's dispatch; (2) the Learner's command for Throw k+1, resolved one Beat later after its own outcome updates the Memory; (3) Segment k+1's plan, which does not exist yet and is itself seeded from Segment k, the one carrying `u`. The Schedule holds S(k+2) and the transit's timing, not the state it starts from. A nominal-chain substitute (plan Segment k+1 at nominal inputs, then Segment k+2) is itself a stand-in seed, needs three solves per evaluation (nine per command against the three-evaluation bound; at the worker's 46 ms idle p50 about 0.4 s per command and up to about 2.4 s at the measured 265 ms max, longer than a Beat), and a new worker request, since `check_only` plans only from the live record.

**Options weighed.**

| Option | What it checks | Admits (failure modes not caught) | Prevents |
|---|---|---|---|
| (iii) today: envelope only | the Throw's release envelope | every leg-limit or margin refusal of the next Catch; first seen as an install refusal ending the Attempt | envelope violations only |
| (iv-a) turned back (CHOSEN) | ball's next Catch, exact in the platform frame, turned by 2D onto the dispatched Catch | errors from evaluating the leg layout 2D away from where the Segment flies (measured below); the one-period-early seed instance (shared with rule (i)) | out-of-margin commands, with a step-back, in the same three solves |
| (iv-b) unturned | this Catch with the landing moved by the unturned miss | the miss's direction relative to the transit is 2D wrong and the 124 mm transit itself is 2D off the true one: inexact in BOTH frames | less than (iv-a) |
| (iv-c) nominal chain at the true Sites | Segments k+1 and k+2 planned in turn | nominal seed (inputs (1)-(3) above); 3x solves, new worker request | the orientation error |
| (iv-d) true terminal, SEED turned forward by 2D | the next Catch at its real Sites from this Segment's seed rotated about z | the one-period-early seed instance only (exact leg orientation) | the orientation error, at one solve |

(iv-a) beats (iv-b) on the merits: (iv-a) is a rigid turn of the whole Segment k+2 (seed Site S(k+1) -> S(k-1), which IS where this seed is; arrival chord S(k) -> S(k+2) -> S(k-2) -> S(k), this Catch's own; S(k+4) -> S(k+2)), so the transit, the miss and the Throw keep their true geometry relative to each other and only the azimuth over the base is off. The Platform's yaw is 0 throughout (rotating Columns is explicit per-Beat Sites, yaw 0), so neither the true Segment nor the stand-in yaws the body: the turned stand-in is a Segment this Attempt's own geometry flies two Beats earlier, not an abstraction. Rule (ii) for the hop already accepts the same kind of inexactness at 180 deg; (iv-a) has it at <= 40 deg.

(iv-d) would be exact in orientation at the same one-solve cost and is the natural successor, but it needs the check request to carry a seed turn from the executor through `skill_node._checker`, `trajectory_node._check_segment_reply` and the solver worker into `check_segment`, a correct rotation of a `CycleState` (positions, velocities, accelerations, tilt as a horizontal axis), and a new carve-out in I-ADMIT-1's "planned from the seed the install will use" clause. That is outside this unit's files (`skill_node.py` is not mine) and is design-bearing on the planner seed, so it is reported, not built.

**What the turn costs, measured** (real planner, OLD point 350 / 5000 / 200000 / 4500, pose planner, Apex 0.95, Dwell 0.27, separation 125, seed = a Throw from rest at S(k-1) aimed at S(k+1), probe `$S/out/jp11-standin/probe_rot.py`, uncommitted):

- Model command around a full turn, stand-in margin vs Site angle: 0.028 to 0.074 at 10 deg per Beat, 0.042 to 0.114 at 20 deg per Beat, period 120 deg (the leg layout's three-fold symmetry). Two Catches 2D apart differ by up to about 0.03 (10 deg per Beat) and 0.07 (20 deg per Beat).
- Displaced commands at Catch 4, stand-in (turned) vs the TRUE Catch k+2 (same seeding convention, the ball landing where the command sends it):

| step | command | stand-in m | true next Catch m |
|---|---|---|---|
| 5 | +30 mm radial | -0.280 LIMIT_VEL | -0.245 LIMIT_VEL |
| 5 | +15 mm radial | -0.121 LIMIT_VEL | -0.092 |
| 5 | -30 mm tangential | -0.013 | -0.024 |
| 5 | +30 mm tangential | 0.040 | 0.033 |
| 10 | +30 mm radial | -0.225 LIMIT_VEL | -0.269 LIMIT_VEL |
| 10 | +15 mm radial | -0.076 | -0.112 LIMIT_VEL |
| 10 | -30 mm tangential | -0.029 | 0.027 |
| 10 | +30 mm tangential | 0.038 | 0.025 |
| 20 | +30 mm radial | -0.253 LIMIT_VEL | -0.099 |
| 20 | +15 mm radial | -0.099 | 0.002 |
| 20 | -30 mm tangential | -0.005 | 0.001 |
| 20 | +30 mm tangential | 0.073 | 0.075 |

The stand-in's error is in both directions, up to about 0.06 at 10 deg per Beat and 0.15 at 20. The slack between the 90 % margin and the 100 % install gate is 0.11 (margin -0.111 is the limit itself). So at 5 and 10 deg per Beat a stand-in pass leaves the true next Catch inside the install gate in every case measured; at 20 deg per Beat a stand-in pass could still be an install refusal (no worse than today, where nothing is checked) and a stand-in refusal can step back a command the true Catch would accept (costs Learner authority, not safety). Accepted: the Rung gate is 10 deg per Beat.

**Clamp before the turn.** The lateral clamp's across-axis part is a per-axis box (`_lateral_clip_mm`), which does not commute with a turn. Rule (ii)'s docstring claims its clamp commutes with the reflection; for a point reflection that holds (a box is symmetric under negation), for a turn it does not, so rule (iv) clamps at the target in the Robot frame, as Catch k+2 itself will, and turns afterwards. The test pins the order (the two differ by more than 0.5 mm in its case).

**Scope.** Label-gated on purpose: Centre and orbit and polygon also have same-site folds aimed at other Sites; whether a turn fits them is a separate question (see the report), and their rule stays (iii) line for line. The Stop of a rotating Attempt (S(n-1) -> S(n), 180 + D deg) is past `STANDIN_ROTATE_MAX_DEG` and stays (iii); in plain Columns the Stop is rule (ii).

## Fix

- `executor.py`: `_STANDIN_ROTATE`, `STANDIN_ROTATE_MAX_DEG` = 40, `_standin_rotation_deg` (signed Site-to-target angle on one circle, else None), `_rot2`; `_standin_rule` returns (iv) after (ii) fails, only under `sch.COLUMNS_ROTATING`; `_catch_standin` builds rule (i) at the target then maps the landing and the aim through R(-theta) after the clamp; `_resolve_catch_command` writes the `rotating stand-in (...)` line. Every other rule's code path and log line is untouched (the new branches are reached only when the rule is (iv)).
- `INVARIANTS.md` I-ADMIT-1: one clause naming rule (iv), three tests, AMENDED note in the Fate column.
- `tests/motion/test_skills_executor_online.py` section (5): 16 tests.

## Verification

- 2026-11-27, `flock /tmp/jugglebot-tests.lock nice -n 10 python -m pytest tests/motion/test_skills_executor_beats.py tests/motion/test_skills_executor_human_catch.py tests/motion/test_skills_executor_learner.py tests/motion/test_skills_executor_online.py tests/motion/test_skills_executor.py tests/motion/test_skills_admissible_online.py tests/motion/test_skills_columns_rotating.py tests/motion/test_skills_columns_rotating_wiring.py tests/ros/test_skill_node_columns_rotating.py tests/sim/test_skills_gate_online.py -q -p no:cacheprovider -n 2`: **552 passed, 1 failed in 354.68 s**. The failure, `test_skills_executor_human_catch.py::test_no_other_compile_sets_the_window_start`, is U2's and not this change: it raises `ROTATING_STEP_OUT_OF_RANGE` for `step_deg = 0.0` inside `nominate.build_pattern` -> `rotating_span`, and fails identically with this unit's `executor.py` replaced by the branch's (HEAD) copy.
- Every existing stand-in, rule (i), (ii) and (iii), and its log line is covered by the pre-existing tests in that run (`test_the_standin_is_planned_at_the_predicted_landing`, `test_a_hop_catch_is_checked_on_its_own_segment_reflected`, `test_an_asymmetric_cross_site_catch_is_admitted_on_the_envelope_alone`, the real-planner Columns pins): all pass unchanged. `test_every_compiled_rotating_throw_but_the_stop_gets_rule_iv[None]` pins compiled plain Columns at (i) x 6 + (ii).
- New tests shown to fail without the change, 2026-11-27, `$S/out/jp11-standin/mutate.sh` (each mutation of `executor.py`, then `python -m pytest tests/motion/test_skills_executor_online.py -q -k "rotat or turned or rule_iv or outward_unit"`, file restored and compared after each): M0 the branch's `executor.py` (no rule (iv)): 6 failed (the turned stand-in, the clamp order, the real-planner step-back, the compiled rule map at 5 / 10 / 20); M1 label gate removed: the 6 other-label cases fail; M2 turn before the clamp: the clamp-order test fails; M3 `_outward_unit_at` from one other Site instead of the mean: the 3 outward-unit pins fail. With the change: 16 passed in 1.04 s.
- Real-planner numbers in the Discussion: 2026-11-27, `$S/out/jp11-standin/probe_rot.py` modes `dirs`, `true`, `orient`, `cost` (outputs `orient_old_10.txt`, `orient_old_20.txt`, `cost.txt` beside it), not committed.

## Findings

- `_outward_unit_at` on rotating streams is not exactly radial: it points away from the MEAN of all other Sites in the Schedule, and a partial turn's Sites do not average to the origin. Worst over n_throws 4 .. a full turn: 2.48 / 4.85 / 8.88 deg at 5 / 10 / 20 deg per Beat, all at n = 4; a full turn is within 1.6 deg (0.26 deg at 10 deg x 38). Pinned by `test_the_catch_clamp_outward_unit_is_near_radial_on_rotating_streams`. Effect: with a separate outward authority, a radial miss is clamped along a unit up to about 9 deg off, so the outward limit lets through up to 1/cos(9 deg) = +1.2 % more radial reach on a 20 deg per Beat short Attempt. Radial is the right target: the transit into S(k) runs along S(k) - S(k-1), at -D/2 from radial, and the one out of it at +D/2.
