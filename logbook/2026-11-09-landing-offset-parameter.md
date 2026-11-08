---
title: "BUG-80: skill_node commanded landing offset (landing_offset_x_mm, landing_offset_y_mm) for self_toss goals, default 0, held on its branch for the 40-Throw from-rest check"
type: feature
date: 2026-11-09
status: complete
phase: "BUG-80 flyback"
subsystem: skills
files_changed:
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/nominate.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - tests/motion/test_skills_landing_offset.py
  - tests/ros/test_skill_node_landing_offset.py
---

# Commanded landing offset for self_toss goals (BUG-80)

## What changed

TKT-47 chose a 40-Throw check that needs commanded landing offsets of +20 and -20 mm from rest, and TKT-48's answer was to build the parameter on a branch held for a Sitting. Before this, a one-ball Throw from rest flew at about zero Tilt because nothing could command it off its Site.

`skill_node` now has `landing_offset_x_mm` and `landing_offset_y_mm` (mm, Robot frame, default 0). `OneBallPattern.landing_offset_m` adds them to each Throw's `y_d`.

- **Applies to `self_toss` only.** Every other Pattern, the two-ball ones included, refuses a non-zero offset by name at goal accept and at `skills/nominate` (`nominate.landing_offset_refusal`). A goal that ignored the parameter would look like the experiment while being an ordinary Attempt.
- **Refused while a goal runs** and past `learner_lateral_authority_mm` per axis, re-checked at accept in case the authority moved.
- **Default 0 is today, bit for bit**: same digest of every Segment's pose, pose velocity, Hand and takeoff before and after the change (Verification).

Held on `night-shift/2026-11-09-BUG-80` for a Sitting by TKT-48. Not on `main`.

## Discussion

### Control walk, one cycle at 40 Hz (Knot dt 0.025 s)

The case: `self_toss`, `num_cycles: 1`, Site (-50, 0), Apex 1.0, offset (+20, 0) mm, cold Memory, nominal rung.

**Release and Tilt.** `_throw_terminal` is the one place the offset becomes a target: `target_mm` is the catch point plus (20, 0), the release stays at the Site. `cup_cycle.takeoff_velocity` gives (22.14, 0, 4395.33) mm/s and `tilt_to_throw` gives 5.04 mrad at release (knot 16 of 29, t = 0.40 s). Platform xy velocity at release is 0.1 mm/s, so the lateral speed is all Tilt: 5.04 mrad times 4395 mm/s is 22.15 mm/s against 22.14 commanded.

**Feedforward and timing.** The offset enters once, at dispatch, as a target. Nothing per-tick reads it. The Hand's axial speed is unchanged to the printed digit (4395.33 at both offsets; 1 - cos 5 mrad is 1.3e-5), so its stroke and feedforward are not moved. The Legs' feedforward comes from the plan's pose and pose velocity as for any Segment. The QP seeds from the REST splice knot (Level, at rest), so C1 holds by construction and the Tilt ramps from 0:

| Rung (commanded-flight) | Tilt at release, mrad | Peak Tilt rate, mrad/s | Largest knot-to-knot rate change, mrad/s |
|---|---|---|---|
| -0.20 | 7.90 | 72.0 | 24.4 |
| 0.0 (nominal) | 5.04 | 45.8 | 18.7 |
| +0.10 | 4.16 | 44.6 | 20.2 |

The worst rung is 2.4 % of the 3000 mrad/s cap. Both planner modes give the same numbers. After release the Tilt returns to 0 in the settle tail (0.0 at knot 28). The Catch is aimed at Site + d and the ending REST is at the Site, profiled like any other.

**Do Memory rows or the identity prior read a commanded offset as a landing error? No.** A row stores `u` (the command sent, offset included) and `y` (landing minus the Site's catch point, `pend.target_xy_mm`). The plant model's innovation is `y - u`, so a commanded +20 mm that lands +20 mm has zero innovation and no residual. The identity prior takes the offset straight through `y_d`; `test_rows_flown_at_an_offset_do_not_shift_a_zero_offset_command` pins that a zero-offset command stays at 0 after 20 such rows, and that a real miss does move it.

Two caveats, neither unsafe:

- A plant gain error at the offset is honest plant evidence and is stored as such. A flight-time gain of 1.08 gives +1.6 mm of residual at +20 mm, and the x excess this check looks for would give about +7 mm. The kernel does not separate +20 from -20 rows (dv 44 mm/s against h_v 0.15 m/s), so alternating signs average out inside the check. A block of one sign would leave a residual in the next goals' k = 16 window, so the Sitting uses a fresh `plant_id`, restored afterwards.
- The operator's OUTCOME and landing-error lines measure from the Site, so they read about 20 mm. That is a report, not a Learner input, and goal accept logs a WARN saying so.

**Fly-back.** `_flyback_for_offset_mm` is `r * (1 - 1/g)` for the release offset `r`. A Throw from rest has `r = 0` and returns exact zeros, so the first Throw of a goal does not interact with it. A same-Site fold does: its `r` is about d.

**Finding 1 (review), confirmed through the real executor: a constant offset only tests the first Throw of a goal.** Probe `/tmp/probe_bug80_fold.py` (uncommitted), `nominate.compile_schedule` with `_throw_terminal`, `_fold_release_mm` and `_aim_target_mm`, for +20 mm x at Apex 1.0:

| Throw | Release x, mm | Aim x, mm | Net displacement, mm | Tilt, mrad |
|---|---|---|---|---|
| First, from rest | -50.0 | -30.0 | 20.0 | 5.04 |
| Fold, cold (g = 1.00) | -30.0 | -30.0 | 0.0 | 0.0 |
| Fold, g = 1.08 | -30.0 | -28.519 | 1.481 | 0.403 |

So Throw 2 onward releases at about Site + d and aims at about Site + d, and the Tilt the check needs exists only on Throw 1. The TKT-47 check is therefore single-Throw goals (`num_cycles: 1`) with the parameter set between them. Alternating signs inside one goal is not an option: it would reintroduce the fold the check exists to exclude. `n_throws = 1` compiles to REST, THROW, CATCH, REST with nothing carried, pinned through the real goal path by `test_a_single_throw_goal_flies_one_throw_with_no_fold`.

**Finding 2 (review), confirmed: the bound limits the offset, not the reach.** The original docstring said an offset cannot ask for a landing further off its Site than the stack may already move one. That is false. The Catch clamp (`_clamp_lateral_inner`) and the context Learner's cap (`c_max_xy_m`, about the model command) are centred on `y_d`, so both ride with the offset and reach is offset plus authority per axis, up to 60 mm at the default 30 mm authority. A 20 mm offset reaches about 50 mm, about 14 mrad, inside the 30 mrad Hops already fly. No bound or limit changed; the docstring and the `skill_node` comment now state this, and `test_the_reach_from_the_site_is_offset_plus_authority` pins it (a landing 200 mm out is clipped at Site + 50 mm for a 20 mm offset at 30 mm authority).

**Verdict: safe to fly.** Target-only change at dispatch, no per-tick path, C1 by construction, Tilt rate 2.4 % of the cap, Memory reads a commanded offset as a command. The fold interaction is a limit on what the parameter can test, not a hazard.

### Why not per-Throw alternation in the schedule

It looks like the direct way to fly the +20/-20 sequence in one goal. But a same-Site fold is the mechanism the check exists to exclude, so the alternation would be measuring the Throw it was meant to separate from. Single-Throw goals cost a goal start each and leave the schedule unchanged.

## Verification

- (2026-11-09, `OMP_NUM_THREADS=1 python -m pytest -q tests/motion/test_skills_landing_offset.py tests/ros/test_skill_node_landing_offset.py`, worktree on the branch): 45 passed.
- (2026-11-09, mutation: `y_d` forced to zeros in `_append_one_ball_pattern`, same two files, `-k` the four tests added today): 4 failed, restored after; the earlier 2026-11-09 mutation on the unit's first pass recorded 40 failed, 1 passed.
- (2026-11-09, `/tmp/probe_bug80_digest.py` on `3f03e2b3` and on the branch, `OMP_NUM_THREADS=1`, 12 Segments each): SHA-256 prefixes pose `6d50f425ec1d8e59` and level `68312e1319f82337` on both, so zero offset is bit-identical to the pre-change code (every Segment's pose, pose velocity, Hand, time and takeoff, plus each `y_d`).
- (2026-11-09, `./run_tests.sh` in the branch worktree, on the code and tests committed here): parallel phase 9295 passed, 52 skipped; serial phase 7 passed; RESULT: PASS, 648 s. The only edit after it is this line.
