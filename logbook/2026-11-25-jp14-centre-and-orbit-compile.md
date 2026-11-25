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
  - ros_ws/src/jugglebot/jugglebot/motion/skills/orbit_clearance.py
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
6. **Odd Throw counts are REFUSED (owner decision 2026-11-25, see Verification (2)).** `ORBIT_ODD_THROW_COUNT`, raised by `schedule.Pattern` itself, so the compile, the nomination (a named COMPILE line) and the node goal all refuse it; never rounded up. The "compile allows either parity" tradeoff in item 5 is superseded.
7. **Range refusal.** r outside 95..105 mm and N outside 6..12 are refused by name (`ORBIT_RADIUS_OUT_OF_RANGE`, `ORBIT_N_OUT_OF_RANGE`, from `sites.centre_orbit_sites`, so the compile, the nomination and the node all refuse the same way), never clamped. The range is IDEA-14's 2026-10-30 probe, confirmed by the sim gate at r 95 N 6 (the other corners did not finish); what binds outside it is not yet measured.

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

## Verification (2), 2026-11-25 (continuation unit pway8-p9-verify)

**Owner decisions folded in (2026-11-25):** odd Throw counts refused by name (`ORBIT_ODD_THROW_COUNT`, `sites.py`, raised in `Pattern.__post_init__`; the WARNING variant that was built first was removed), a Ball Butler Feed at x = -100 mm is acceptable, the label stays `centre_orbit`. Control-cycle walk: the refusal is at build time, before any Segment is planned or any command emitted, so the 40 Hz path, the emitter and the splice see only even-count Schedules, which compile exactly as before; no existing Pattern is touched (below).

1. **The F at 55 % was two failures** (the scoped rerun, 2026-11-25, `-v`, same file order). (a) `tests/motion/test_skills_nominate.py::test_columns_at_p4_plans_every_segment_and_matches_the_p3a_probe_cell`: the worst-margin rung is +0.10, the test pins -0.20 (margins themselves pass). NOT this branch: it fails identically on a detached `origin/main` worktree (same test, `-x`, 1 failed in 40.7 s) on a plain `columns` nomination: a pre-existing drift for the orchestrator to triage; left unfixed. (b) `tests/ros/test_skill_node_nominate.py::test_the_node_and_the_nomination_build_the_identical_pattern[centre_orbit-EMPTY|SEATED]`: a REAL defect of this branch, not a Pattern-list pin: the goal accept nominated `centre_orbit` with `feed_site` `P1` (the `columns_feed_site` default) while the run path builds Ball 2's feed at `O1`, so the nominated config was not the flown config (the contract that test pins; harmless to the Schedule because `build_pattern` ignores the name for this Pattern, but the cache key and any future reader of the name would differ). Fixed in `skill_node._nominate_at_accept` (the orbit's own first Site; an out-of-range radius still falls through to the nomination's by-name refusal). After the fix (2026-11-25, `pytest tests/ros/test_skill_node_nominate.py -k "identical_pattern and centre_orbit"`): 2 passed, 309 s on a box at load 20.
2. **Pattern-label enumerations** (grep `columns_highlow` / `polygon_walk`): `sim/skills_gate.py` default table: present. `skill_node.py` `_PATTERNS`, `_CUP_RULE`: present. `records.py` / `docs/records.md`: present. `juggle_request.py`: `orbit_n` key present. `nominate.py`: present. GUI Juggle panel (`ros_ws/gui/js/juggle-panel.js`, `test_gui_juggle_defaults.py`, `juggle_nomination_harness.js`): ABSENT deliberately, as `polygon_walk` is (neither is in the panel; a product decision for the owner, not a test to append to). `docs/patterns.md`: generated from map items, label column `-` (not hand-edited). `tests/sim/test_skills_gate_nominate.py` / `test_skills_schedule.py` / `test_records.py`: Pattern-specific cases, no enumeration of all labels.
3. **Sim gate grid** (`python sim/skills_gate.py --orbit-n N --separation-mm R --apex-m 0.95 --n-throws T --seeds 0 1 2 3 4`, 2026-11-25, T = 2N + 2, every Catch made, 0 drops, no refusal, installs all matched): (95, 12, T 26; an earlier T 25 odd run also passed 25/25 before the refusal landed) 26/26 on every seed; (105, 6, T 14) 14/14; (105, 12, T 26) 26/26; (100, 8, T 18) 18/18; (95, 6, T 14) 14/14. All five seeds each, verdict PASS.
4. **Outside the range:** radius 90 and 110, N 5 and 13 are each refused by name through the node's `skills/nominate` (`ORBIT_RADIUS_OUT_OF_RANGE` / `ORBIT_N_OUT_OF_RANGE`), never clamped, no nearest-viable suggestion: `tests/ros/test_skill_node_centre_orbit.py::test_just_outside_the_range_is_refused_by_name_never_clamped` (4 cells, new; passes as written because the range refusal pre-existed).
5. **Clearance table** (`python tools/probes/centre_orbit_clearance.py`, minimum ball-to-ball centre distance, mm, ball diameter 74.0; lap = steady lap before the Stop; EVEN = Stop at 2N + 2; ODD = the refused 2N + 1):

| r | N | lap | even Stop | odd Stop (refused) |
|---|---|-----|-----------|--------------------|
| 95 | 6 / 8 / 10 / 12 | 87.9 / 90.9 / 92.4 / 93.2 | 78.5 / 78.6 / 78.6 / 78.5 | 71.8 / 74.7 / 76.0 / 76.8 |
| 100 | 6 / 8 / 10 / 12 | 92.5 / 95.7 / 97.2 / 98.1 | 82.7 (all N) | 75.5 / 78.6 / 80.0 / 80.8 |
| 105 | 6 / 8 / 10 / 12 | 97.2 / 100.5 / 102.1 / 103.0 | 86.8 (all N) | 79.3 / 82.5 / 84.0 / 84.9 |

   Every odd Stop at r 95 and 100 and at r 105 N 6 / 8 is at or below diameter + 10 mm (84 mm); r 95 N 6 odd overlaps (71.8). The even Stop at r 95 (78.5) is within diameter + 10 mm as well, but it is Columns' own Stop at separation 95 (78.5), flown before.
6. **Byte-identity** (2026-11-25, `dump_sched.py` on `git archive origin/main` vs this tree, 337 Schedule configurations): `cmp` identical, sha256 f09c9519c88e.
7. **Tests:** `pytest tests/motion/test_skills_centre_orbit.py tests/ros/test_skill_node_centre_orbit.py -q` -> 31 + 8 passed after the refusal (odd-count refusal tests fail with the check removed: the new refusal tests raise nothing, mutated once to `if False:` in the WARNING form earlier and the refusal form is the same shape). Scoped suite rerun (22 files + 2, `-m 'not nightly' -n 3`, under the flock): 1019 passed, 1 failed (item 1), run unfinished at 56 % when the unit ended: see the report.
