---
title: "The Hand C1 grace stays at 14 ms (a proposed FW 41 at 6 ms was withdrawn before flashing); FEAT-32 and BUG-57 rows refreshed; ZVD rows stay off on Columns"
type: investigation
date: 2026-11-06
status: complete
phase: "post-debrief state review"
related_plan: carry-ring-zvd.md
related_issues: []
files_changed:
  - logbook/2026-11-06-hand-c1-grace-stays-14-ms.md
  - logbook/INDEX.md
  - plans/active/carry-ring-zvd.md
  - plans/active/learner-generalisation.md
  - map/feature-pre-release-hold-rung-release-from-a-moving-platform-hold-0.md
  - map/bug-hand-overshoot-at-apex-1-35-saturated-the-stepped-brake-at.md
  - map/bug-hand-stall-stop-near-a-release-cannot-fit-the-stroke-the-20.md
  - map/feature-p4-ramp-sitting-columns-canary-at-600-8000-and-20-a-then-the.md
  - map/bug-firmware-scheduler-stop-fires-on-a-clean-host-stream-and.md
  - map/bug-two-ball-throws-land-about-9-mm-toward-the-partner-site-an.md
  - docs/runbook.md
  - plans/active/INDEX.md
subsystem: [firmware, motion]
tags: [safety, firmware]
---

# The Hand C1 grace stays at 14 ms

**What.** No firmware change: the board stays on FW 40 with `HAND_C1_STOP_GRACE_US` 14000. Owner decision, 2026-11-06: keep the number of variables down while the learner-generalisation work flies, and revisit if a problem shows at high Apex. The Runbook rows that had gone stale are corrected, and plan `carry-ring-zvd.md` section 8 is answered.

## Discussion

**A hypothesis withdrawn before it reached the robot.** A state review proposed going back to 6 ms on the reasoning that FW 40's queue fix had removed the missing frames that FW 39's 14 ms was sized for, and that the one in-stream C1 stop under FW 40 was a consequence of a SPINOUT. The owner approved that. Reading the Hand trace before flashing refuted the second half. At 14:55 (launch log L7017, `1793937984.37`) streaming was live, the forward gap was 60 ms, the Hand command fell from 78 rev/s to rest mid-upstroke, and torque loss and SPINOUT followed, with the Hand coasting to 8.75 rev: the same chain as 2026-11-05. The downlink ledger read lost 0. The queue fix removed lost frames, not host-side gaps, so a shorter grace costs more stops than the review implied.

**The rule, and why a first measure misled.** The firmware fires when the playing frame's age plus the gap to the next frame passes 50 ms plus the grace. Frame age alone misses a host-side gap, because the next frame's stamp jumps with it and it arrives young: four frames over 31 ms across the 2026-11-06 bags looked like a small cost, and was not the whole of it.

| Grace | C1 stops per 1175 Throw upstrokes | Release-knot stop, margin to the 10.501 rev clip at Apex 1.25 |
|---|---|---|
| 14 ms | about 1 (measured) | -0.50 |
| 10 ms | about 2 to 3 (estimate) | +0.36 |
| 6 ms | about 6 (estimate) | +1.21 |

The estimates scale from intervals between accepted frames in `/leg_setpoint_echo` inside upstrokes, which has a 100 Hz timer's resolution, so they are good to about a factor of two. Attempts end at about 53 per 1175 today. Margins are from `tools/probes/hand_stall_stop.py --option 2`.

**The tradeoff accepted.** Above Apex 1.10 a release-knot C1 stop overruns the clip at 14 ms, and nomination accepts up to Apex 1.25, so the limit is a written rule and not an enforced one. It needs a C1 stop on one knot of a Throw, so it is rare, and nothing above 1.10 has flown since FW 39. If Apex above 1.10 is wanted, 10 ms is the value to test first: it clears every Apex the planner accepts, while the Hand limit stays 4500 rev/s^2. A proposed FW 41 was built and tested in a worktree, never flashed and never committed.

**BUG-57's row.** It sent the operator to Apex 1.35 and said to wait for FEAT-41, which closed on 2026-11-06, so it read as cleared while the overrun stood. Nomination also refuses 1.35 at Dwell 0.27, 0.30 and 0.35 (Hand 4605 rev/s^2 at the +10 % rung, limit 4500). The row now flies Apex 1.10, the highest the grace clears. That is a softer brake than the 2026-11-03 SPINOUTs, so a clean run there says less about the 70/80 A question than 1.35 would have.

**ZVD rows on Columns (section 8, option (a)).** The 2026-11-05 headline "Columns rows-on empty" was too broad: nomination accepts Sep. 125, Dwell 0.27 with the rows at Apex 1.00 and 1.05 and refuses 0.95 and 0.97. The rows stay off for Leg acceleration and a small gain: planned 5655 mm/s^2 at Sep. 125 and 6276 at Sep. 135, against 6296 at the BUG-72 separation. This is a decision and not a guard: `carry_zvd` defaults off, and nothing refuses a Columns goal with it on.

## Fix

FEAT-32's row names FW 40, launches with `gc_freeze:=true` for both arms, drops the identification step and states the planned Leg peaks; it is not the next Sitting (owner). BUG-57's row flies Apex 1.10. BUG-55 and FEAT-39 rows name FW 40 (proto 12). BUG-55 and BUG-57 are held by other owners, so their progress is this entry and not a note on the Item.

## Verification

Rehearsed offline (2026-11-06, `nominate.nominate` at the Sitting limits): `oval_1ball_fed` Apex 1.00, Sep. 150 at Dwell 0.40 with the flag off and on and 0.35 with it on, and Columns Sep. 125, Dwell 0.27 at Apex 0.95 and 1.10 flag off, all nomination OK.

Tests read by path, tier 3 (2026-11-06, `python -m pytest tests/sim/test_map_items.py tests/sim/test_map_items_rated.py tests/sim/test_sitting_sections.py tests/sim/test_plans_index.py tests/sim/test_mapkit_docs.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py -q -p no:cacheprovider`): 222 passed, 48 skipped. One read-only narrative audit pass; its findings were applied.

## Outcome

Nothing on the robot changed. The next launches keep the conditions every Learner result so far was flown under.
