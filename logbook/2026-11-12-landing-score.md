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

## Resume 2026-11-13 (TKT-66)

Owner's answer: a Throw's score, for Ball Butler and Jugglebot alike, is measured against the target that Throw was aimed at; what the other robot or the Catch does must not matter; and the owner agreed with (b).

- **(a) Feed target.** A Columns Feed's `LandingPrior` is walked forward by `columns_feed_bb_bias_mm` for the Catch; that walk is not Ball Butler's aim. The node now passes the raw announcement (before `+ bias`) to `_install_columns_schedule(score_target_xy_mm=)`, which sets the report-only `SkillExecutor.feed_score_target_xy_mm`; `_PendingCatch.target_xy_mm` uses it when set and falls back to `landing_prior.pos_mm` (a plain `reload` Feed, already the aimed point). It is an executor attribute rather than a `LandingPrior` field because every `LandingPrior` field is serialised by `tools/probes/columns_golden.py` into the committed columns schedule golden, so a new field would have changed that fixture. `LandingPrior.pos_mm`, the planner, the Catch, the learner and the wire are untouched.
- **(b) Lost Throws.** The records ledger's `no_fit` now includes the Throws lost to an empty Cup (not for `magazine`), so Scored plus Without a Fit equals the Attempt's Throws. The end line adds the same count to its "(N without a Fit)" because `end_line` already receives `missed_catches`. The ledger's pair comes from the new `report.record_landing_scores`, on `record_counts`' own base: it also scores the `unverified` final Catch that `record_counts` adds to the ledger's Throws on a `COMPLETED` goal (that row has a landing), so the identity holds there too.
- Consequence of (a): Ball Butler is SENT `aim_site - columns_feed_bb_bias_mm` and announces exactly that point, so the Feed's target is that request point; with a non-zero bias configured the Feed score shows Ball Butler's own landing bias (about |bias| when the compensation works), by the owner's answer. The INFO line's "request" figure is the aim Site before the bias walk, not the point sent.
- Tests: `test_a_feed_is_scored_against_the_point_its_thrower_was_aimed_at` (executor), `test_the_end_line_counts_a_throw_lost_to_an_empty_cup_as_without_a_fit` (report), `test_throws_lost_to_an_empty_cup_are_without_a_fit_in_the_ledger` (node), `test_the_ledger_scores_and_no_fit_add_up_to_the_ledger_throws` (report), `test_columns_feed_score_target_is_the_announced_point_not_the_biased_prior` (node, the Columns announcement hand-off with a non-zero bias).

## Verification
- 2026-11-12, `tools/night_shift_nonet.sh python -m pytest -q tests/motion/test_skills_report.py tests/motion/test_records.py tests/ros/test_skill_node_magazine.py tests/ros/test_gui_records_api.py tests/ros/test_replay_stdlib.py`: 152 passed in 4.92 s.
- 2026-11-12, same wrapper, `tests/ros/test_skill_node.py tests/motion/test_skills_executor.py` (full files): 731 passed in 186.21 s.
- New tests fail without the change (unit's report, output not kept): with the four source files stashed, the same wrapper over the four changed test files with `-k "landing or score or median or feed_report_carries or drained_feed"` gave 17 failures, and passed again restored.
- 2026-11-13, `tools/night_shift_nonet.sh python -m pytest -q tests/motion/test_records.py tests/motion/test_skills_executor.py tests/motion/test_skills_report.py tests/ros/test_skill_node.py`: 864 passed in 184.66 s. Of the four new Resume tests, the first three (executor Feed, report end line, node ledger) fail with the three source files stashed (report.py, executor.py, skill_node.py); the fourth (the ledger-base test) is covered by the next line.
- 2026-11-13, after the orchestrator added `record_landing_scores`: `tools/night_shift_nonet.sh python -m pytest -q tests/motion/test_skills_report.py tests/motion/test_records.py tests/ros/test_skill_node.py -k "score or no_fit or fit or ledger or empty_cup or feed"`: 78 passed in 50.68 s; the new report test fails with `report.py` stashed.
- 2026-11-13, after the audit-fix docs pass and the new Columns hand-off node test: `tools/night_shift_nonet.sh python -m pytest -q tests/ros/test_skill_node.py tests/motion/test_skills_executor.py tests/motion/test_skills_report.py`: 778 passed in 187.16 s. The hand-off test fails with `skill_node.py` restored to its pre-resume copy (1 failed) and passes restored.
- Resolved by TKT-66 (see Resume above); was open at the audit (2026-11-12), the owner's call: a plain `reload` Feed is scored against Ball Butler's raw announcement while a Columns Feed is scored against the bias-corrected point; Throws lost to an empty Cup (FEAT-48, no Throw report) are in neither Scored nor Without a Fit.
