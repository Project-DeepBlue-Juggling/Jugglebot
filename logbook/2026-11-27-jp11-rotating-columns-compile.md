---
title: "JP-11 rotating Columns compiled (U1): Pattern.step_deg turns the Site pair step_deg per Beat, label columns_rotating, five refusals by name; every existing Schedule byte-identical (337 configs plus 60 Centre and orbit), Centre and orbit's stream generalised into one per-ball Site-cycle stream"
type: feature
date: 2026-11-27
status: in-progress
phase: "JP-11 rotating Columns U1"
related_plan: "pattern-progression.md"
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/sites.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - tests/motion/test_skills_columns_rotating.py
subsystem: [skill-stack]
tags: [pattern, rotating-columns, jp-11]
---

# JP-11 rotating Columns: the compile (U1)

## What

Branch `offline/jp11-rotating` = origin/main + the held Centre and orbit branch `offline/pway8-p9` (merged without conflicts). U1 only: the compile. Nominate, node parameter, records, `skills_gate` and the stand-in rule are U2/U3 and untouched; `executor.py` untouched.

- `Pattern.step_deg` (None or 0 = Columns, bit for bit). `sites.rotating_sites(pair, step_deg, n_throws)` returns `S(0)..S(n+1)`; `S(k)` at polar angle `phi1 + (k-1)(step+180)` deg, radius `r` = half the separation, `S(1)` = the Feed Site P1 exactly, `S(0)` = the held Site exactly. Names `R<k>@<angle>` (unique per Site).
- `schedule._cycle_events` replaces P9's `_orbit_events`: each ball walks its own Site list, Centre and orbit is `((C,), orbit)`, rotating Columns is `(S[0::2], S[1::2])`. `COLUMNS_ROTATING = 'columns_rotating'` passed through `compile_beats(label=)`; one `label` design.
- Refusals by name (all at build time, before any Segment): `ROTATING_STEP_OUT_OF_RANGE` (outside (0, 20] deg, nan, bool), `ROTATING_APEX_PAIR_UNEQUAL`, `ROTATING_PHANTOM_REFUSED`, `ROTATING_NEEDS_HOP_ENTRY` (a t0 start would leave S(0), a Site no ball is caught at), `ROTATING_NOT_A_PAIR`, `ROTATING_WITH_ORBIT`. The non-pose-planner refusal (as `columns_turned`) lives in `skill_node` and is U2's.

## One Beat walked (10 deg per Beat, r 62.5 mm, first 5 Throws)

Throw k leaves `S(k)` and is aimed at `S(k+2)`; its ball is caught there and thrown again there after the Dwell. S(1) = (-62.5, 0) P1; S(2) = (61.55, 10.85); S(3) = (-58.73, -21.38); S(4) = (54.13, 31.25); S(5) = (-47.88, -40.18); S(6) = (40.17, 47.88).

- Throw 0 (Ball 1, hop entry): leaves P1, aimed S(2) (chord 124.5 mm, Columns' hop: the Feed, the lane and the cup parking are exactly flown Columns').
- Throw 1 (Ball 2, the fed ball caught at P1): leaves P1 after the inflight Catch, aimed S(3), chord 2r sin 10 = 21.7 mm.
- Throw 2: S(2) -> S(4); Throw 3: S(3) -> S(5); Throw 4: S(4) -> S(6); each 21.7 mm.
- One Beat: Ball 1 is caught at S(2) at its Catch instant, the cup Dwells 0.27 s there, releases Throw 2 from S(2), then transits 2r cos 5 = 124.52 mm to S(3) for Ball 2's Catch, Dwells, releases Throw 3. Apex, Dwell and flight are unchanged, so every `t_abs_s`, window and lead equals Columns' at the same Apex and Dwell (tested field by field).
- The Stop (last Throw n-1) lands where Throw n-2's ball is held (`S(n)`), as Columns' does; the stream validator checks it.

Control-cycle walk (40 Hz): the Schedule is compiled before any command; each THROW/CATCH reads its own `site`/`target` (no Pattern-level Site cache), so a moving Site is a different value of the same field, the splice and leads see nothing new. The only physical difference from Columns is the lateral release speed (chord / flight, 21.7 mm / 0.88 s = 25 mm/s at 10 deg), which the nomination (U2) and the command-time check (U3) judge.

## Verification

- (2026-11-27, `dump_sched.py` of 337 existing Pattern configs (Columns straight and turned, phantoms, high-low, fed hop entry, one-ball, polygon) with PYTHONPATH on the merged tree before the change and on this tree): `cmp` identical, sha256 f09c9519c88e (same as P9's). Centre and orbit, 60 Schedules (`dump_orbit.py`, 3 radii x 2 Apex x 5 counts, hop and t0): identical, sha256 602ba28fa917 before and after.
- (2026-11-27, `pytest tests/motion/test_skills_columns_rotating.py -q`): 27 passed. Mutation: the Site list stride changed from `(j+1)` to `(j+2)` in `_cycle_events`: 13 of 27 fail (stream, chord, Stop, hop, timing, uniqueness); restored.
- (2026-11-27, `pytest tests/motion/test_skills_schedule.py test_skills_sites.py test_skills_beats.py test_skills_centre_orbit.py test_skills_columns_rotating.py test_skills_polygon_walk.py test_skills_oval.py test_skills_landing_offset.py test_skills_executor_beats.py tests/ros/test_skill_node_centre_orbit.py -q -m 'not nightly' -n 2 --dist loadfile`, under the flock): 718 passed in 286.95 s.
- (2026-11-27, `pytest tests/ros/test_skill_node.py -q -m 'not nightly' -k "opens_on_a_rest or columns or orbit" -n 2 --dist loadfile`): 112 passed in 27.26 s. The whole-file run was cut by a 560 s timeout and is owed to the integration gate.
- `./run_tests.sh --full` is owed at integration (sim/ and motion/ readers).
