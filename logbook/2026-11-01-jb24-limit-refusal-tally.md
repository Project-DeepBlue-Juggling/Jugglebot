---
title: "BUG-5: LIMIT_* refusals are not the dominant miss class over the 2026-10-28..31 Sittings (4 of 58 goals, 3 of them the BUG-42 Hand cell; 31 of 37 leg refusals were harmless re-sends); margin stays 10 percent; tally committed as a probe"
type: feature
date: 2026-11-01
status: complete
phase: "night-shift follow-up"
related_plan:
  - two-ball-oval.md
related_entries:
  - 2026-10-31-hold-ladder-sitting-the-hold-model-was-wrong.md
files_changed:
  - tools/probes/limit_refusal_tally.py
  - tools/probes/README.md
  - map/bug-limit-refusal-jitter-margin.md
  - logbook/INDEX.md
---

# BUG-5: the LIMIT_* refusal tally

**Question.** BUG-5 holds that landing-time jitter (plus or minus 20-30 ms) against the
10 percent planning margin makes `LIMIT_*` refusals the dominant miss class. The owner chose
TKT-23 option 4: tally the refusals per goal from the last three Sittings' launch logs and decide
on the number, with no code until the class is shown dominant at hold 0.

**Method.** `tools/probes/limit_refusal_tally.py` (new) reads `~/.ros/log/<launch>/launch.log`,
splits it into goals (`New goal accepted` .. `finished with state`), and counts each goal's end,
the planner's `install_segment refused` lines by row, and what skill_node did with each:
`RESEND-REFUSED` (re-aim after a tracker fit refused; the old plan stays installed),
`CATCH-AIM-FALLBACK` (learned aim refused; dispatched at the nominal aim) or a fatal `END`.
Launches: 2026-10-28 09:49, 14:29, 21:02, 22:26; 2026-10-30 16:17; 2026-10-31 13:06, 13:14
(the 10-28 0-byte launches and the goal-less 10-31 13:13 and 17:07 launches skipped).

**Result** (`python3 tools/probes/limit_refusal_tally.py <the 7 dirs>`, 2026-11-01):

| | Count |
|---|---|
| Goals | 58 (835/847 balls caught where a summary line exists) |
| Ends | done 17, `MISSED_CATCH` 18, `LANE_REFUSED` 10, no summary 4, `ABORTED_NO_RELEASE` 3, `HAND_LIMIT_ACC` 3, `LIMIT_VEL` 1, `TILT_PIN` 1, `SPLICE_TOO_LATE` 1 |
| Leg `LIMIT_*` refusals | jerk 28, acc 6, vel 3 = 37: 31 re-sends, 5 aim fallbacks, 1 fatal |
| `HAND_LIMIT_ACC` refusals | 5 (2 re-sends, 3 fatal; all 4049-4112 vs the 3900 ceiling, the BUG-42 cell) |
| Other planner refusals | transit 37, `SPLICE_TOO_LATE` 43, `INFEASIBLE` 15, node-wait 4 |

**Verdict.** Not dominant: 4 of 58 goals ended on a `LIMIT`-class code, and three of those are
one known Hand cell (BUG-42's `HAND_LIMIT_C2` question), not jitter on the time rows. The leg
refusals land almost entirely on the re-send path, where a refusal is harmless by design. The
misses that matter are `MISSED_CATCH` and the firmware `LANE_REFUSED` (BUG-34).

**Limits of the tally.** All goals but one flew at the old hold 0.100; the one hold-0 goal
(10-31 13:14 G16, oval) caught 30/30 with zero refusals, so hold 0 is neither confirmed nor
refuted. The launch log carries no per-goal hold (taken from the hold-ladder entry). Four goals
have no summary line (cancelled or cut, one by the 10-31 cable `SPINOUT_DETECTED`).

**Decision.** The margin stays at 10 percent (option 1 not taken). BUG-5 stays open with its
trigger; re-run the probe after a Sitting with several hold-0 goals.

**Verification.** The probe reproduces the analysis agent's independent per-Sitting totals
(58 goals; leg jerk 28, acc 6, vel 3; `HAND_LIMIT_ACC` 5; fatal 3 + 1). No reader imports it
(`grep -rn limit_refusal_tally` hits only the README row, this entry and the Item).
