---
title: "JP-14 Centre and orbit compiled on the beat stream (label centre_orbit): Ball 2 walks a Site list, Ball 1 keeps the centre, no yaw and no Learner change; sim gate PASS at r 100 N 8 and r 95 N 6 (5 seeds each); steady ball-to-ball clearance 95.7 mm at the working point, the lap-closing Stop only 78.6 mm, so a Rung flies an even Throw count"
type: feature
date: 2026-11-25
status: in-progress
phase: "pattern-progression P9"
related_plan: "pattern-progression.md"
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/sites.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/nominate.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/juggle_request.py
  - ros_ws/src/jugglebot/jugglebot/records.py
  - docs/records.md
  - sim/skills_gate.py
  - tools/probes/centre_orbit_clearance.py
  - tests/motion/test_skills_centre_orbit.py
  - tests/ros/test_skill_node_centre_orbit.py
  - tests/ros/test_skill_node.py
subsystem: [skill-stack]
tags: [pattern, centre-and-orbit, jp-14]
---

# JP-14 Centre and orbit: the compile (P9)

## What

A new two-ball Pattern, machine label `centre_orbit` (provisional until the owner names it, plan decision D-b): Ball 1 is thrown and caught at the centre Site C (Platform origin); Ball 2 is fed at O1 (-r, 0), where Columns' P1 is, and each of its Throws is aimed one step on around a regular N-gon of radius r (O1 .. ON, counter-clockwise). Wired end to end: `sites.centre_orbit_sites`, `Pattern.orbit_sites`, the stream in `columns_events`, the Schedule label, `skills/nominate` (key `orbit_n`), the `orbit_n` node parameter and the Juggle goal through `_run_columns` (Columns' own entry: hop entry from O1, two-feed chain on an empty cup), the records row, and `sim/skills_gate.py --orbit-n N` (with `--separation-mm` as the radius). The GUI Juggle panel is not wired.

## Discussion (the design, settled before code)

1. **What exists.** `compile_beats` already compiles any stream of `BeatEvent`s whose balls alternate and whose every Throw leaves the Site its ball was caught at; it never needed "two Sites" (only `Pattern.sites` is a pair, read by `columns_events`). Cross-site Throws already fly in it (the hop entry, the Stop), and the fold pairs a Catch with the next Throw of the same ball at the same Site, which holds for Ball 2 (caught at Ok, thrown from Ok). So the Site list is a stream: `Pattern.orbit_sites` (None = Columns), `_orbit_events` (Ball 1 C -> C; Ball 2's j-th Throw O(j mod N) -> O(j+1 mod N); the Stop lands where Throw n-2's ball is held, as in Columns), and a `label` argument on `compile_beats`, carried in `BeatSource` so `with_events` keeps it. JP-5's `polygon_walk` is the one-ball compiler (`OneBallPattern(roles='polygon')`) and is not reused: the stream is two-ball by construction. Every other Pattern compiles byte for byte as before (Verification).
2. **Geometry and timing** at the working point (r 100 mm, N 8, Apex 0.95, Dwell 0.27): flight 0.880 s, Beat 0.575 s, transit 0.305 s, one Ball 2 lap 16 Beats = 9.2 s. Every cup transit is C <-> Ok = r, Columns' at separation r. Ball 2's chord is 2 r sin(pi/N) = 76.5 mm, a lateral release speed of 87 mm/s (the 100 mm Hop asks 114). The Sitting limits are judged by the nomination check, not by hand: OK at r 100 N 8, worst margin 0.189 (jerk, the Catch of Ball 2 at O7, rung -20 %) on the default limits. Platform tilt for the aim is inside what the Hop flies (unchecked by hand; the online check judges each command).
3. **P8 is ordering, not a prerequisite.** JP-14 only translates Sites: every event has `yaw_rad = 0`, so the dormant `BEAT_YAW_NOT_SUPPORTED` refusal is never touched. Proceeded without P8.
4. **Moving Sites downstream.** The executor reads each Skill's own `site` / `target` (no Pattern-level Site cache: the only `.sites` readers are the node's run path, which picks the Feed Site O1 and Ball 1's Site C, and the nomination builder). The Learner keys rows by release xy and target (I-LEARN-1), so Ball 2's eight release Sites are eight contexts, as the polygon walk's four were (flown 25/25 from an empty Memory); no Learner change. The online admissibility check judges each command at its own release (I-ADMIT-1). Unchecked: a mid-Attempt beat change (`columns_change` on this stream) was not exercised.
5. **Ball-to-ball clearance** (`tools/probes/centre_orbit_clearance.py`, nominal paths, ball diameter 74 mm from `juggling_ball_radius_mm`): in the steady lap the closest approach is where Ball 2's chord passes the centre column, r cos(pi/N) in xy, both balls at 544 mm above the Catch plane: 95.7 mm at r 100 N 8 (margin 21.7 mm), 87.9 mm at the worst corner r 95 N 6. The Stop is the binding case: an ODD Throw count ends on Ball 1's Throw from C onto Ball 2 at O1 while Ball 2's last chord converges on O1: 78.6 mm at r 100 N 8, 71.8 mm (overlap) at r 95 N 6. An EVEN count ends on Ball 2's Throw onto Ball 1 at C, Columns' own Stop: 82.7 mm at r 100 (Columns at separation 100: 82.7). **Tradeoff accepted:** the compile allows either parity; the nomination's short pattern is 2 N + 2 (every chord once, then the even Stop) and the Rung flies an even count.
6. **Range refusal.** r outside 95..105 mm and N outside 6..12 are refused by name (`ORBIT_RADIUS_OUT_OF_RANGE`, `ORBIT_N_OUT_OF_RANGE`, from `sites.centre_orbit_sites`, so the compile, the nomination and the node all refuse the same way), never clamped. The range is IDEA-14's 2026-10-30 probe, confirmed by the sim gate at r 95 N 6 (the other corners did not finish); what binds outside it is not yet measured.

One control cycle (40 Hz, a carried Ball 2 Throw): the CATCH at Ok installs on the previous release (its window is the transit), the solver plans touch-down at Ok, the Dwell and a release whose velocity reaches O(k+1) in one flight; the stream is unchanged in kind from the hop entry's cross-site Throw, so the emitter, the splice and the leads see nothing new. Nothing changes for any existing Pattern.

## Verification

- (2026-11-25, `tools/probes`-style dump of every existing Pattern's Schedule repr, 337 configurations: Columns straight/turned 45/90, both phantoms, high-low, fed hop entry, one-ball oval/hop, polygon walk; `$S/out/pway8-p9/dump_sched.py` run with PYTHONPATH on `git archive HEAD` and on this tree): byte-identical, sha256 f09c9519c88e.
- (2026-11-25, `pytest tests/motion/test_skills_centre_orbit.py tests/ros/test_skill_node_centre_orbit.py -q`): 27 passed. Mutation (Ball 2's last chord not stepping on): the three lap-closure tests fail; restored.
- (2026-11-25, `pytest tests/ros/test_skill_node.py -k opens_on_a_rest -q`): 20 passed, including both `centre_orbit` cup cells.
- (2026-11-25, scoped regression: `pytest` over the 22 skills schedule/executor/nominate/beats/polygon/records, sim gate and skill_node files plus the two new ones, `-m 'not nightly' -n 3 --dist loadfile`, under the flock): INCOMPLETE. One F at 55 % before the run was stopped at 13:10 for the unit's budget; the failing test is unidentified. To be rerun at integration.
- (2026-11-25, `python sim/skills_gate.py --orbit-n 8 --separation-mm 100 --apex-m 0.95 --n-throws 17 --seeds 0 1 2 3 4`): PASS, 17/17 Catches every seed, 0 drops, installs 19/19.
- (2026-11-25, same with `--orbit-n 6 --separation-mm 95 --n-throws 13`): PASS, 13/13 every seed, 0 drops.
- Not finished (stopped for budget): r 95 N 12, r 105 N 6, r 105 N 12.
- (2026-11-25, `nominate.nominate(NominationConfig('centre_orbit', 0.95, 100, 0.27, orbit_n=8, entry='hop'), default_limits(), search=False)`, short pattern then 2 N + 1): nomination OK, worst margin 0.189 (jerk, CATCH Ball 2 at O7, rung -20 %), 3 min 47 s. The grid outside the range did not run.
- `./run_tests.sh --full` is owed at integration (sim/ and motion/ changed).
