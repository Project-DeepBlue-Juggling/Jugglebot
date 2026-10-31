---
title: "JB-120 replay: the Columns Ball 2 HAND_LIMIT_ACC refusal does not reproduce offline from the box's inputs (lateral u and catch aim move the peak <= 0.5 pt); the learned-prior fallback missed it because the classifier did not know the HAND_LIMIT_ prefix (fixed)"
type: bugfix
date: 2026-11-01
status: complete
phase: "JB-120 (two-ball-oval)"
related_plan:
  - two-ball-oval.md
related_entries:
  - 2026-10-31-hold-ladder-sitting-the-hold-model-was-wrong.md
  - 2026-10-30-sitting-prep-columns-check-hold-ladder-and-apex-cells.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - tests/motion/test_skills_executor.py
  - logbook/INDEX.md
subsystem:
  - skills
tags:
  - admissible-box
  - hand-limit
  - columns
---

# JB-120 replay: box gate vs HAND_LIMIT_ACC

**What.** `temp/probes/jb120_replay.py` (uncommitted) plans the 2026-10-31 goal-4 skill 13 (Columns, sep 125, apex 0.95, dwell 0.27, pose; CATCH of Ball 2 at P1 carrying its self-throw) through `segments.plan_segment` in two shapes: the box sweep's (`_columns_catch_throw_cell`: catch landing = throw target = site + u, release at the site) and the executor's (`_catch_terminal`: catch at the aim, release at the caught xy, target site + u + fly-back). It ran on the flown tree (`48bf4fcb^`, a `/tmp` worktree) and on HEAD, with two seeds: the sweep's (rest launch at P2) and a live-like one (Ball 1's P2 catch-with-throw caught at (+10, -20) mm, u (-8.8, -15), apex 0.882).

**Finding (reframes the Item).** The lateral inputs do not drive the peak: on the flown tree at hold 0.100 the throw u (0 to (-16.6, -20)) and the catch aim (0, +5.2 or -20 mm in y) move it by at most 0.5 pt (89.8-90.5 % on the sweep seed). The seed moves it by 12 pt (90.5 -> 78.1 %). A learned apex of 0.93-1.00 (catch time -9 to +23 ms, faster arrival) gives 77.8-91.1 %. **No offline case reaches 3900**: the live 4112.5 (105.4 %) is 14 pt above the highest replay. The Item's "gate and solver disagree ~5 % at the y = -20 edge" is withdrawn as a cause, and with it fix candidate 1 (shrink the y edge). The input left is the live seed (the BUMPED, re-aimed installed plan at the release snap, k_s 293), which is not logged and was not rebuilt. Two sweep gaps are real but not shown to be causal: `_within_margin` applies `MARGIN_FRAC` 0.9 to the legs only, so the Hand is certified at 100 % of 3900 (the cell sits at 90 % in the sweep's own shape). And the box bounds the expected landing y_hat (2026-10-13 rule) while the sweep certified u = landing: the live u x = -16.6 mm is outside the box x [-2, +1], and the sweep's shape at that u refuses LIMIT_VEL (354 mm/s). Today's defaults (hold 0, stroke top +16 mm) put every replayed case at 66-79 %, 21+ pt of margin.

**Bug fixed.** Goal 4's catch dispatches all printed `source=schedule:learned`, so a refusal should have run the unit-A2 `CATCH-AIM-FALLBACK`. It did not, because `install_segment` returns the validate gate's bare check code (`InstallResult(False, exc.code, ...)`). `_aim_dependent_refusal('HAND_LIMIT_ACC')` was False: no `LIMIT_` prefix and no `INFEASIBLE`. The brief's premise (a tracker-fit aim, deliberately not retried) does not hold for this dispatch. The fix adds the `HAND_LIMIT_` prefix (ACC and VEL). Control path, one dispatch: the learned-prior CATCH is refused `HAND_LIMIT_ACC`. The active plan is untouched (`_reject_segment`) and the 40 Hz emitter keeps streaming. The executor builds the y_d terminal through the same `_catch_terminal` (same clamp, same cached carried-throw u) and makes one more worker solve (25-200 ms; goal 4's refusal came back at +66.6 ms against a +355 ms deadline). `installer` re-measures the window against its own t_now. Accepted means a normal splice; refused means END with the second code. Nothing changes on an accepted dispatch. The replay says a y_d aim (catch offset 0) is unlikely to rescue this case by itself. The fix restores the designed fallback; it is not shown to cure JB-120. `HAND_STROKE` is also aim-dependent in the replay (it flips with catch offset and time on HEAD at hold 0.100) and is left out on purpose (owner call).

## Verification

- Replay, 2026-11-01: `python temp/probes/jb120_replay.py /tmp/jb120_pre` (flown tree) and `python temp/probes/jb120_replay.py` (HEAD); peak Hand acc in rev/s^2 (% of 3900), u (-16.6, -20) unless marked:

| tree, hold | sweep shape, sweep seed, u (0,-20) | live, sweep seed, catch 0 | live, sweep seed, catch -20 y | live, live-like seed, catch 0 | live, live-like seed, catch -20 y |
|---|---|---|---|---|---|
| flown, 0.100 | 3504.0 (89.8) | 3528.4 (90.5) | 3511.0 (90.0) | 3046.5 (78.1) | 3045.8 (78.1) |
| flown, 0.000 | 3510.6 (90.0) | 3528.4 (90.5) | 3512.1 (90.1) | 3047.5 (78.1) | 3047.6 (78.1) |
| HEAD, 0.100 | 3053.6 (78.3) | 3077.8 (78.9) | HAND_STROKE (-0.005 rev) | 2596.0 (66.6) | 2595.4 (66.5) |
| HEAD, 0.000 | 3060.1 (78.5) | 3077.8 (78.9) | 3061.6 (78.5) | 2597.0 (66.6) | 2597.2 (66.6) |

  Live 2026-10-31: 4112.5 (105.4). The sweep shape at u (-16.6, -20) refuses LIMIT_VEL on both trees. Apex scan (`JB120_MODE=apex python temp/probes/jb120_where.py [/tmp/jb120_pre]`): flown 77.8-91.1 %, HEAD hold 0 66.3-79.4 %, no HAND_LIMIT_ACC anywhere.
- Scoped tests, 2026-11-01: `pytest tests/motion/test_skills_executor.py tests/motion/test_skills_admissible.py tests/ros/test_install_segment.py -q -p no:randomly`, **462 passed**. The new test `test_a_hand_limit_refusal_of_the_learned_prior_falls_back_to_y_d` failed before the executor change (`-k hand_limit_refusal_of_the_learned_prior`: 1 failed). `./run_tests.sh` was not run (Night Shift unit).
