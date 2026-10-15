---
title: "Handoff-catch SPLICE_TOO_LATE: offline replay of 299 recorded handoff catches sizes the lead - Fix 1's 250 ms leaves 13-28 ms on the recorded failures, 8 extra knots leaves 113 ms, but any longer lead moves nearly every catch onto the prior so the pinned re-aim (and its mis-keyed fences) carries the correction"
type: investigation
date: 2026-10-15
status: open
phase: "handoff-solve-chaining — Phase 0 (replay baseline)"
related_plan: handoff-solve-chaining.md
related:
  - 2026-10-14-two-ball-columns-sessions-why-attempts-end.md
files_changed:
  - tools/probes/handoff_replay.py
  - tools/probes/README.md
  - plans/active/handoff-solve-chaining.md
  - plans/active/INDEX.md
  - logbook/2026-10-15-handoff-replay-baseline.md
  - logbook/INDEX.md
subsystem:
  - skill-stack
  - motion
tags:
  - timing
  - catching
  - replay
---

## Summary

Phase 0 of `handoff-solve-chaining.md`. A committed probe, `tools/probes/handoff_replay.py`, replays every recorded handoff CATCH (first-dispatch, non-RESEND, scheduled release minus dispatch > 0.29 s) from the launch logs and bags of sessions S1 (2026-10-14 20:54), S2 (21:27) and sitting 8 (2026-10-13 23:12) through the serial-queue timing model at handoff leads of 4, 6 and 8 extra knots. 299 catches (S1 65 / S2 97 / S8 137). The predecessor queue never bound (predecessor installs are ~1 s earlier). Outputs: `temp/probes/handoff_replay/handoff_catches.csv` and `summary.md`.

Cliff: derived as t0 + (K - 3)*dt from the logged splice knot K, about dispatch + 0.238 s. The nominal dispatch + 0.200 was withdrawn because accepted installs landed 0.223-0.231 s after dispatch. The 38 ms bias between derived and nominal is unexplained (open; resolve from Fix 1's `_refusal_timing` lines at the next sitting).

Sanity at the recorded lead (0.275 s): 0 negative accepted margins (one -0.4 ms rounding in S8); 11 of the 12 attempt-ending refusals negative (the exception S1 L720: +4 ms). The 7 SPLICE_TOO_LATE RESEND refusals do not move with the lead.

First-install margin on the 12 END refusals (min / median, ms):

| Lead | Extra knots | Margin | Accepted-install min |
|------|-------------|--------|----------------------|
| 0.325 s (Fix 1) | 4 | 13 / 28 (upper bounds for the two "solver did not answer" rows) | 50 |
| 0.375 s | 6 | 63 / 78 | 100 |
| 0.425 s | 8 | 113 / 128 | 150 |

Tracker fit converged at the dispatch instant (267 catches matched to a bag track): recorded 61 %, 45 % at 4, 10 % at 6, 5 % at 8. Live fits converge mostly 0.2-0.3 s after release; the earlier 0.11 s figure came from the offline tracker replay. Bag availability agrees with the logged `source=tracker` on 90 % of catches.

Re-aim budget for prior-aimed catches (cliff minus max(fit time, first-install end)), share with >= 190 ms (tracker-aim p90): 16 % at 4, 68 % at 6, 75 % at 8; about 16 % of catches have their fit after the cliff at every lead.

Code fact verified 2026-10-15 (`executor.py` `_resend_live_catch`, the MIN_WINDOW fence and the CONTACT-WINDOW decline): both use `t + skill.lead_s`, i.e. HANDOFF_LEAD_S for a handoff catch, while a snapped re-send really splices at the predecessor's release knot and the node's raw knot uses LEAD_S. The re-aim window therefore closes earlier as the lead grows (at 8 knots ~40 ms after a p50 first install, never after a slow one). The replay's budget table assumes these fences are fixed.

## Discussion

Withdrawn hypotheses:
- "The handoff catch's first aim is prior-based": 74 % (S1) / 61 % (S2) are tracker-aimed.
- "Worker chaining is needed for 8 knots": the skill_node tick blocks on each install, so a due catch queues; chaining would add ~25 ms.
- "Fits converge at 0.11 s live": that was the offline tracker replay; live fits converge 0.2-0.3 s after release.

Decision (owner, 2026-10-15): the pre-registered rule (a) passes at 6 and 8 extra knots, rule (b) fails at every lead. The owner chose 8 extra knots AND the fence fix in Phase 1.

Tradeoff accepted: first aims move onto the prior in exchange for a 350 ms first-install budget and a re-aim that finally has budget. About 20 % of catches end on the prior with the fence fix, versus ~39 % today (and ~95 % at 8 knots without it).

## Verification

- 2026-10-15, `python tools/probes/handoff_replay.py` with the S1/S2/S8 log and bag paths as in its docstring: 299 catches; the sanity check at the recorded lead reproduced the recorded outcome.
- 2026-10-15, `pytest tests/sim/test_plans_index.py -q -p no:cacheprovider`: 75 passed, 2 skipped.

## Outcome

Open: Phase 0 complete; Phase 1 (8 extra knots plus the fence fix) is next under the plan. The 38 ms cliff bias remains to be resolved from `_refusal_timing` lines.
