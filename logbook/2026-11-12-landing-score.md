---
title: "Landing score: per-Throw distance to the target Site, median and a no-Fit count on the end line, every score in the records ledger"
type: feature
date: 2026-11-12
status: complete
phase: night-shift
subsystem: skill-stack
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/report.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/records.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/motion/test_skills_report.py
  - tests/motion/test_records.py
  - tests/motion/test_skills_executor.py
  - tests/ros/test_skill_node.py
  - logbook/2026-11-12-landing-score.md
  - logbook/INDEX.md
  - map/idea-create-and-display-an-accuracy-tidyness-score.md
  - docs/project-map.md
tags:
  - night-shift
  - records
---

# Landing score (IDEA-21, owner's decision in TKT-63, option 1)

**What.** A Throw's score is `hypot(dx, dy)` in mm of the `landing_err_mm` its report already carries (Fit's landing minus the target Site): `report.landing_score_mm`, `landing_scores` (Throw order, plus the count of Throws with no Fit; same base as `summarise`, so the columns Stop and an unverified final Catch are out), `median_mm`. Report-only: nothing aims, dispatches or learns from it, no flown motion changes.

**End line.** After `worst landing N mm` it adds ` · landing median 24 mm over 18 (2 without a Fit)`; the parenthesis only when some Throw had no Fit, the whole phrase only when at least one Throw was scored.

**Feeds.** `FeedCatchReport.landing_err_mm` (new, default `None`) is the Fit's landing latched in flight (within 0.3 s of the scheduled landing) minus the announced landing point (`_PendingCatch.target_xy_mm`, from `landing_prior`), under the same Fit-within-0.3 s guard the Feed's crossing already used. `skill_node._drain_reports` keeps the scores in `_goal_feed_scores`. A Feed that missed has no report, so no score; a Feed without a Fit has none either.

**Ledger** (`docs/records.md`, written live, committed by the owner at the Sitting close-out). A `## Landing scores` section, omitted while empty (an old ledger renders and parses unchanged): per Pattern `| When | Scored | Median mm | Without a Fit | Scores mm |` (one row per Attempt, integer scores in Throw order, `-` when none) and one `### Feeds` table `| When | Pattern | Score mm |`. Written whether or not a record is beaten; a `magazine` run books no Throw score (never a catch target, TKT-64); a Feed goal writes Feed rows only. `parse` checks Scored equals the number of scores and Median equals their median, so `parse(render(x)) == x` holds and hand damage is a format error. `skill_node._offer_record` passes the scores inside its never-raises guard.

**Decisions.** "Miss count" = Throws without a Fit (TKT-63 Findings). Scores rounded to whole mm in the ledger; the end line's median is the unrounded one, so the two medians can differ by 1 mm. Rows grow one per Attempt, unbounded like the record tables; a pruning rule is left to the owner.

## Verification
- 2026-11-12, `tools/night_shift_nonet.sh python -m pytest -q tests/motion/test_skills_report.py tests/motion/test_records.py tests/ros/test_skill_node_magazine.py tests/ros/test_gui_records_api.py tests/ros/test_replay_stdlib.py`: 152 passed in 4.92 s.
- 2026-11-12, same wrapper, `tests/ros/test_skill_node.py tests/motion/test_skills_executor.py` (full files): 731 passed in 186.21 s.
- New tests fail without the change (unit's report, output not kept): with the four source files stashed, the same wrapper over the four changed test files with `-k "landing or score or median or feed_report_carries or drained_feed"` gave 17 failures, and passed again restored.
- Open at the audit (2026-11-12), the owner's call: a plain `reload` Feed is scored against Ball Butler's raw announcement while a Columns Feed is scored against the bias-corrected point; Throws lost to an empty Cup (FEAT-48, no Throw report) are in neither Scored nor Without a Fit.
