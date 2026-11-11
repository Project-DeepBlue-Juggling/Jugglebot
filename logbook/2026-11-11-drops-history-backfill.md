---
title: "Drops to date: the history backfill, 819 drops from 2026-07-23 to 2026-11-10 (FEAT-47)"
type: feature
date: 2026-11-11
status: resolved
phase: ""
subsystem: skill-stack/gui
files_changed:
  - docs/records.md
  - logbook/artifacts/2026-11-11-drops-history.csv
  - logbook/INDEX.md
  - logbook/2026-11-11-drops-history-entry.md
  - map/feature-add-total-drops-to-date-counter.md
tags: [FEAT-47, backfill, records]
related_items: [FEAT-47]
---

# Drops to date: the history backfill

## Summary

FEAT-47 asked for a "Drops to date" counter atop the Records panel. The live counter landed in slices 1 to 3 (`2026-11-10-drops-to-date-live-counter`, `2026-11-11-drops-count-feed-misses`, `2026-11-11-drops-count-lost-throws`) and the entry point for a history figure in slice 4 (`2026-11-11-drops-history-entry`). This entry records the history figure itself: **819 drops, 2026-07-23 through 2026-11-10** (low 753, high 884), written into `docs/records.md` with `tools/set_drops_history.py`. The live count starts at 0 on 2026-11-11. The evidence, one row per Attempt or per reconciled launch, is `logbook/artifacts/2026-11-11-drops-history.csv` (994 rows).

If anything in this entry differs from the backfill's own `FINAL_summary.md`, that file was the source and wins; the figures below were checked against it.

## Owner decisions (TKT-55, TKT-61, TKT-64 and the Session's answers, 2026-11-11)

- Full history, not a base figure from end lines only.
- Zero before 2026-07-09. Throws at the Cone are not drops.
- July to September are counted from launch logs, the logbook and every bag together.
- A Throw that cannot be decided gets a best estimate, and the doubt is listed (see the ten largest uncertainties).
- The counting scripts are one-off and uncommitted. They lived in a Session scratchpad that is gone, so this entry and the evidence CSV are the record. Nothing here can be re-run from the repo; the CSV is what to audit.

## Method

**Era 2026-09-29 to 2026-11-10 (script).** 137 hardware launches, 825 Attempts, counted by a script over the per-Throw and Attempt end lines in five classes: the end-line tally; a missed Catch with no Throw report; a lost ball while the other was caught; a Feed miss; and the old end-line format. Two independent samples checked it: 20 of 20 and 18 of 20 agreed, and the second sample found one script bug (the "no further throws" ending booked 0 for 31 lost balls). With that fixed the script total moved from 607 to 633. Cross-checks: 49 of the 59 matching rows of `docs/records.md` agree with the script (9 are outside the window, 1 differs by two Throws), and the learner Memory is consistent.

**Era 2026-07-09 to 2026-09-28 (bags).** 298 launches and 261 bags (14.8 GB). Every bag was read by a purpose-built mocap and Hand-sensor tool, validated on four bags (table below). The 65 launches with Catch activity were reconciled per Sitting against the launch log and the logbook, in eight chunks. The other 196 bags were swept for any Throw or Feed at the Hand: one false positive, nothing added. 2026-07-09 to 2026-07-22 has no Throw or Feed at the Hand at all, which is why the figure starts on 2026-07-23.

## Discussion: what a future reader must not rediscover

- **July and August launch-log Catch verdicts are unreliable in both directions.** The tracker split tracks, so it reported CAUGHT on balls that fell. Equally it reported "Reload MISSED" on balls the Hand sensor proved seated, and "Toss MISSED" for 17 bag-proven catches on 2026-07-27. So the bag and the Hand sensor outrank the log verdict lines. The log is still the only source for the Feed count and the Attempt boundaries.
- **Evidence order used:** Hand sensor or a ball seen on the floor, then the operator's eyewitness count in the logbook, then mocap-only inference, then the log verdict line. A new track near the Cup is never Catch evidence by itself.
- **The bag tool splits one flight into several rows.** In self-Throw chains it over-counts rows by 15 to 30 percent, so its raw totals are never a count. Rows were reconciled per Sitting against the log and logbook; the tool was used as evidence, not as a counter.
- **Eight Throws on 2026-07-25 to 2026-07-27 have no evidence at all** (bag undecidable, no operator count). They were estimated at the measured miss rate, 4 drops in 47 evidence-backed self-Throws (8.5 percent), giving 0.68 expected, booked as 1 drop. The log line said 8. The 07-25 rate may be worse because it predates the 07-26 catch-reach overshoot fix; that is carried in the high case.
- **The two 2026-09-13 calls follow the operator's count.** An operator eyewitness count beats a mocap-only inference but not a Hand-sensor reading or a floor sighting, and neither exists there. The bag alone would add 2.
- **The backfill exposed a live-counter gap.** The live counter omitted Throws lost to an empty Cup, about a third of all drops (216 of 633 in the script era). It was fixed the same day (`2026-11-11-drops-count-lost-throws`).
- **A Throw that never left the Hand is not a drop.** A `magazine` Throw is never a drop.
- **Why a best estimate and not a refusal:** a counter that says "unknown" for 30 rows of 994 is less useful than one with a stated range, and the range is published here and in the CSV (`drops_low`, `drops_high`, `confidence`).

## Result

819 drops, 2026-07-23 through 2026-11-10, of 7304 Throws and Feeds at the Hand, 6485 caught, 127 of the drops Feed misses. Low 753 and high 884.

- **Low (753):** only proven or strong evidence counts. Uncertain rows book 0; bag-versus-log doubts resolve to fewer drops; 10 percent of the script era's lost-ball +1 rows and of its Feed misses are taken as false; the 7 uncertain Feed misses on THROW-kind faults are not drops.
- **High (884):** the log verdicts the bag overturned were right where the bag was blind (chunks 3 and 4); the step-2 rows revert to their log or bag drops; the 6 `ABORTED_BB_THROW_TIMEOUT` Feeds each reached the Cup and missed.

| era | rows | Throws and Feeds | caught | drops | Feed misses |
|---|---:|---:|---:|---:|---:|
| reload_coordinator | 73 | 665 | 506 | 159 | 53 |
| ring_fsm | 16 | 17 | 16 | 1 | 0 |
| skill_stack_pre_ledger | 80 | 490 | 464 | 26 | 0 |
| script_era (2026-09-29 on) | 825 | 6132 | 5499 | 633 | 74 |

| confidence | rows | Throws and Feeds | caught | drops | Feed misses |
|---|---:|---:|---:|---:|---:|
| proven | 643 | 4697 | 4253 | 444 | 48 |
| likely | 321 | 2555 | 2204 | 351 | 70 |
| uncertain | 30 | 52 | 28 | 24 | 9 |

| month | rows | Throws and Feeds | caught | drops | Feed misses |
|---|---:|---:|---:|---:|---:|
| 2026-07 | 19 | 141 | 103 | 38 | 22 |
| 2026-08 | 54 | 524 | 403 | 121 | 31 |
| 2026-09 | 140 | 675 | 631 | 44 | 1 |
| 2026-10 | 520 | 2928 | 2517 | 411 | 63 |
| 2026-11 | 261 | 3036 | 2831 | 205 | 10 |

Script-era rows are Attempts; earlier rows are per-Sitting or per-launch reconciliations and are not one-to-one with Attempts. Per chunk (low/best/high): 14/17/29, 21/24/30, 58/59/79, 25/42/52, 17/18/18, 5/8/12, 12/14/19, 3/4/6; script era 598/633/639.

### The ten largest uncertainties

| # | what | low delta | high delta |
|---|---|---:|---:|
| 1 | 210 script-era Attempts booked +1 for a lost ball the end line did not report | -21 | 0 |
| 2 | Chunk 3 (08-10 to 08-18): sensor-blind rows booked caught against a log MISSED | 0 | +20 |
| 3 | Chunk 4 (08-20 to 08-26): log MISSED verdicts the bag overturned | -17 | +10 |
| 4 | 74 Feed misses booked from the log with no seat evidence | -14 | 0 |
| 5 | Chunk 1 (07-23 to 07-27): 8 rate-estimated self-Throws; chain-inferred catches | -3 | +12 |
| 6 | Chunk 2 (07-27 to 08-09): chain-inferred outcomes on mocap-blind Throws | -3 | +6 |
| 7 | 6 `ABORTED_BB_THROW_TIMEOUT` Feeds booked 0 | 0 | +6 |
| 8 | Chunk 7 (09-17 to 09-27): chain and bounce row ambiguity | -2 | +5 |
| 9 | Chunk 6 (09-09 to 09-17): 09-13 kept on the operator count; undetected flights on 09-16 | -3 | +4 |
| 10 | Chunk 8 (09-28): the n/a 283 mm throw and two empty-Cup strokes | -1 | +2 |

## Fix: the figure in the ledger

`python tools/set_drops_history.py --history 819 --from 2026-07-23 --through 2026-11-10 --since 2026-11-11 --path docs/records.md` (the committed tool; the marker was not hand-edited). The section reads "Drops to date: 819 (0 counted live since 2026-11-11; 819 from history, 2026-07-23 to 2026-11-10)". No robot launch has happened on 2026-11-11, so the live window opens today. Evidence CSV (994 rows; columns include `confidence`, `source`, `note`, `drops_low`, `drops_high`): [2026-11-11-drops-history.csv](artifacts/2026-11-11-drops-history.csv), kept under `logbook/artifacts/` like the records seed CSV of 2026-10-28. No motion, timing or control path is touched.

## Open follow-ups

- The Attempt end line's "N/M caught" tally and the Juggle result still omit Throws lost to an empty Cup. The drops counter already counts them (`SkillExecutor.missed_catch_drops`); filed as a map Item.
- Two test-coverage gaps: the executor-to-node join for Feed misses and lost Throws is pinned by seeded values, not by a real executor run through the node.

## Verification

- Bag tool validation, 2026-11-11, four bags against their references (`B2_RECIPE.md` of the backfill): 2026-07-23 third Sitting 8 caught of 10 Feeds (exact); 2026-08-10 26 caught / 16 missed against the Hand sensor's 25/15 (off by one each way); 2026-09-06 6 of 7 (exact); 2026-09-16 7/0 (agrees with the caught-window probe). 0 wrong verdicts. Two Catching_Cone bags correctly gave 0 attempts.
- Script reproduction, 2026-11-11, `count_script_era_v2.py` with `post_v2.py` over the whole `~/.ros/log` tree (run by the backfill, not re-run for this entry): reproduced the 825 script-era rows exactly, 0 differing.
- Tool write, 2026-11-11, `python tools/set_drops_history.py --history 819 ...`: "total 819 = 819 history + 0 live (since 2026-11-11)", exit 0.
- Tier 3 named tests, 2026-11-11, `pytest tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_plans_index.py tests/sim/test_mapkit*.py tests/motion/test_records.py tests/ros/test_gui_records_panel.py tests/ros/test_gui_records_api.py tests/ros/test_skill_node.py tests/ros/test_skill_node_magazine.py tests/sim/test_map_items.py tests/sim/test_map_items_rated.py tests/sim/test_glossary_avoid_words.py tests/sim/test_sitting_debrief_protocol.py -q`, RESULT: **1038 passed, 38 skipped in 178.90 s**, exit 0. The readers of `docs/records.md` came from `grep -rn "records.md" tests/` (the last five files before the map ones); `test_records.py` parses the real ledger section.
