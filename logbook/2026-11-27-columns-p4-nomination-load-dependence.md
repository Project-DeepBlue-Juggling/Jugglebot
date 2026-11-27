---
title: "BUG-148: the nominated worst rung was named by float noise, not by load; ties within 1e-8 now resolve to the first rung in walk order"
type: bugfix
date: 2026-11-27
status: complete
phase: "Night Shift 2026-11-27"
related_issues: [BUG-148]
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/nominate.py
  - tests/motion/test_skills_nominate.py
---

# Columns P4 nomination: worst rung named by float noise (BUG-148)

**Symptom.** `test_columns_at_p4_plans_every_segment_and_matches_the_p3a_probe_cell` returned worst rung (5, 0.1, 'jerk') on a loaded box, the test pins (5, -0.2, 'jerk').

**Hypothesis withdrawn: a wall-clock budget.** With `search=False` the only clock reads (`perf_counter` in `nominate.py`, `unified_cycle.py`) fill timing fields (`aim_wall_s`, `plan_wall_s`, `search_wall_s`); none gates a solve. There is no time budget in the planned path.

**Cause.** `NominationReport.worst()` was a bare `min` over margins. Skill 5's margin is flat across the three rungs (the same leg-jerk peak binds): 0.33469821121515 / ...25289 / ...25306 at one BLAS thread, and 0.33469821125378 / ...25477 / ...23312 with BLAS threads unset (probe `/tmp/probe_bug148.py`, 2026-11-27, quiet box, load 0.3). The rungs differ at the 1e-10 level, so the argmin is chosen by summation-order noise: the test's recipe pins `OPENBLAS_NUM_THREADS=1`, and a differently threaded or scheduled BLAS flips it. The unthreaded run on a quiet box already returned +10 %, so load only changes which noise appears.

**Fix.** `_worst_of` (used by `worst()` and `aim_worst()`): among verdicts within `WORST_TIE_TOL = 1e-8` of the minimum, take the first in walk order (rung -20 %). Display and the label of the worst rung only: the margins, the admissibility gate, `ok` and every planned Segment are untouched, so nothing flown changes. `worst()` also chooses the single rung the `full=False` re-aim walk uses (`add_aim_figure`), so on a tie that walk is now deterministically at rung -20 % as well (a display figure; it decides nothing). The reported margin can differ from the true minimum by under 1e-8; the summary prints 3 decimals.

**Test.** `test_worst_rung_is_independent_of_float_noise_between_tied_margins` permutes the noise over skill 5's three verdicts and requires (5, -0.20) each time, and that a real 1e-3 gap still wins. It fails on the old `min` (the second permutation puts the minimum on the +10 % rung).

## Verification

2026-11-27, `python -m pytest tests/motion/test_skills_nominate.py tests/motion/test_skills_polygon_walk.py -q -p no:cacheprovider` under the Night Shift wrapper: 61 passed in 615.42 s (0:10:15), including the new test.
