---
title: "Emit lead 1 price measured offline (P2): zero drops and no re-aim lost to the earlier fences in 440 sim Catches, handoffs 95 -> 100 % prior-aimed, 40 % of fresh RESTs newly REBASED (proxy); measurement only, no code change"
type: investigation
date: 2026-11-25
status: complete
phase: "emit-lead P2"
related_plan: ""
related_issues: []
files_changed:
  - tools/probes/emit_lead_admissibility.py
  - tools/probes/fresh_rest_census.py
  - tools/probes/skill_constants_dump.py
  - tools/probes/README.md
  - logbook/2026-11-25-emit-lead-p2-measurements.md
subsystem: [motion, tooling]
tags: [measurement, emit-lead]
---

## Summary

Design `emit_lead_design.md` section 5 asked what lead 1 costs: every Catch re-aim fence closes 25 ms earlier and every handoff is planned from a Fit 25 ms younger. Three committed probes measure it; no code under `ros_ws/`, `sim/` or `config/` changed and the configured lead stays 0. Verdict against the design's pass bars: no new dropped Catch (0 of 440, both leads); re-aims lost to the earlier fences 0 (bar: 1 per 20); unconverged-at-dispatch share 95 % -> 100 % (bar: at most twice). The one number that is not small is the fresh-REST census: about 40 % of recorded fresh RESTs solve in 50-75 ms and would be REBASED at lead 1.

## Measurements

**(a) Sim admissibility** (`tools/probes/emit_lead_admissibility.py`, seeds 0-4, one subprocess per run, `skills_gate` tracker-sourced aims). The sim solve is instantaneous, so this measures fences and windows, NOT solve lateness, real tracker noise or the Hand.

| config (seeds 0-4) | lead | Catches made / dropped | re-aims accepted (per seed) | last re-aim before touch-down, ms p10 / p50 / p90 (min) | silent fence closes (MIN-WINDOW) | declines |
|---|---|---|---|---|---|---|
| Columns | 0 | 100 / 0 | 1 (0,0,1,0,0) | 716 (one) | 5 | WITHIN-TOLERANCE 100 |
| Columns | 1 | 100 / 0 | 40 (6,9,7,10,8) | 688 / 740 / 769 (567) | 5 | WITHIN-TOLERANCE 100 |
| Columns 45 deg | 0 | 100 / 0 | 3 (1,0,1,0,1) | 708 / 716 / 737 (708) | 5 | WITHIN-TOLERANCE 100 |
| Columns 45 deg | 1 | 100 / 0 | 43 (7,12,7,9,8) | 638 / 733 / 770 (609) | 5 | WITHIN-TOLERANCE 100 |
| Columns 90 deg | 0 | 100 / 0 | 1 (0,0,0,0,1) | 708 (one) | 5 | WITHIN-TOLERANCE 100 |
| Columns 90 deg | 1 | 100 / 0 | 35 (5,9,7,8,6) | 646 / 736 / 768 (567) | 5 | WITHIN-TOLERANCE 100 |
| one-ball oval (Ball Butler reload entry) | 0 and 1 | 60 / 0 | 52 (7,10,11,10,14), identical | 777 / 801 / 833 (633), identical | 60 | CAP-SPENT 2, WITHIN-TOLERANCE 60 |
| one-ball fed Columns | 0 | 80 / 0 | 0 (the gate sets `resend_max_per_catch=0`) | - | 20 | CAP-SPENT 2, WITHIN-TOLERANCE 80 |
| one-ball fed Columns | 1 | 80 / 0 | 0 (same) | - | 20 | CAP-SPENT 31, WITHIN-TOLERANCE 79 |

Reading: no refusal of any kind (no RESEND-REFUSED, no refused install, no compile error or exception, 0 drops of 440 Catches per lead). The silent FREEZE / MIN-WINDOW closes are equal at both leads (5 per config, i.e. the last Catch of each stream), so the earlier fences cost no re-aim. Re-aims land 567-833 ms before touch-down against a fence 250 / 275 ms before it, so the 25 ms is nowhere near binding in the sim. What DOES move is demand: at lead 1 the plain Columns handoff is dispatched from a 25 ms younger Fit, aimed from the prior more often, and a re-aim follows (1 -> 40 accepted per 100 Catches; the fed Columns run, which has re-aims disabled, shows the same demand as CAP-SPENT 2 -> 31 of 80 Catches whose Fit disagrees with the commit). The oval's entry-fed Catches are unchanged. Band-REST installs: none in any run was REBASED (no `band REST ... slide N` message), so the slide against the 8-knot cap was not exercised in the sim (every slide is one knot larger at lead 1 by construction; unmeasured). Bias variants (`--plant-bias-xy 10 0`, `--bb-bias-mm 29 25`) gave byte-identical results to the unbiased runs, so they were dropped from the probe. The bumper's own tally (`BUMP_STATS`) read 1 accepted on Columns lead 1 and 0 elsewhere; BUMP-DECLINED was WITHIN-TOLERANCE only.

**(b) Handoff dispatch** (`tools/probes/handoff_replay.py --wire-read-knots`, the S1/S2/S8 recipe: 299 accepted handoff Catches, 267 matched to a Fit). Recorded outcome is lead 0.275 s; "today" is main's 0.425 s at wire read 3; lead 1 is 0.450 s at wire read 4.

| | Fit available at dispatch | unconverged at dispatch | min / p5 / p50 margin of accepted handoffs, ms | END refusals negative | re-aim budget p5 / p50, ms | pinned re-aim firable (of unconverged) | PINNED-BUDGET declined |
|---|---|---|---|---|---|---|---|
| recorded 0.275 s | 61 % | 39 % | -0 / 27 / 91 | 11 of 12 | - | - | - |
| lead 0, 0.425 s, wire 3 | 5 % | 95 % | 150 / 177 / 241 | 0 | 189 / 276 | 165 of 216 (76 %) | 2 |
| lead 1, 0.450 s, wire 4 | 0 % | 100 % | 150 / 177 / 241 | 0 | 185 / 263 | 176 of 229 (77 %) | 4 |

Unconverged share ratio 1.05 against the bar of 2. Per session the lead-1 availability is 2 / 0 / 0 % (S1 / S2 / S8) against 10 / 6 / 2 % at lead 0. The margin is unchanged because the model keeps the recorded solve P; the cost shows as the re-aim budget p50 losing 13 ms (276 -> 263) and two more PINNED-BUDGET declines; the raw-path count is identical (49 both). The section-0 "sanity" rows at wire 4 are not a reproduction of the recording (the cliff moved one knot).

**(c) Fresh-REST census** (`tools/probes/fresh_rest_census.py`). The executor prints a solve time only for a REBASE, so the proxy is the trajectory_node line `install_segment REST ... splice_k=0, plan P ms` (service entry to accept; overstates the solve). Only the 52 `temp/logs/launch_2026-11-13_*` / `-18_1` logs carry those lines (the `~/.ros/log` launch logs from 2026-11-15 on have none; 59 of them carry 33 REBASED skill lines and no plan times). Result: 243 fresh REST installs, plan ms `<25: 30, 25-50: 79, 50-75: 97, 75-100: 32, >=100: 5`, p50 52 ms, p90 80 ms. At lead 1, 97 (40 %) would be newly REBASED; 37 (15 %) already are at 75 ms. No band RESTs in those logs (BUG-95 is newer), so the band path is outside the census. A REBASE moves the REST's start up to 125 ms later (design section 3); whether that matters per REST (from rest versus seed in motion, I-PLAN-12 refusals) is unmeasured.

**(d) Constants** (`tools/probes/skill_constants_dump.py --emit-lead N`, 167 constants): lead 0 -> 1 changes 22 values, exactly the design table: WIRE_READ_KNOTS 3 -> 4, LEAD 9 -> 10 knots (0.225 -> 0.250 s), CATCH_FREEZE_S 0.250 -> 0.275, HANDOFF_LEAD 17 -> 18 (0.425 -> 0.450), FRESH_THROW_LEAD 13 -> 14 (0.325 -> 0.350), BUMP_LEAD_KNOTS 5 -> 6, BUMP_FREEZE_LEAD_S 0.225 -> 0.250, FRESH_REST_REBASE_S 0.075 -> 0.050; RESEND_PINNED_BUDGET_FLOOR_S, the stall fence, REST_RETURN_ONSET_S and the G2 budgets unchanged.

## What the sim cannot say

Real tracker noise (re-aim demand rises with it and the sim fit is 0.5 mm noise and converges sharply just before the lead-0 dispatch, which is why the sim jumps 1 -> 40 re-aims where the recorded logs go 95 -> 100 % prior-aimed); real solve times (the sim solve is instantaneous: nothing here tests the re-aim solve against its budget, p5 185 ms vs the 150 ms floor); the Hand (SPINOUT, stamp-gap STOPs, resume tolerance); band-REST slides against the cap; and the per-REST consequence of a later start.

## Verification

- 2026-11-25, `flock /tmp/jugglebot-tests.lock python tools/probes/emit_lead_admissibility.py --out $S/out/emit-lead-p2/matrix_v1_resend_only` (first probe version, 7 configs including the two bias variants, seeds 0-4, 70 child runs, 13-57 s each, nice 10): 0 drops, 0 errors; table above. The probe was then extended (bump re-aims counted and timed, zero-slide band REST counted, bias variants removed); that revision passed `python -m py_compile` and its full-matrix re-run was queued behind other sessions' test-lock runs for over an hour and was not completed, so the BUMP counts in the table come from `grep " BUMP skill "` over the first version's logs.
- 2026-11-25, `python tools/probes/handoff_replay.py --log S1=.. --log S2=.. --log S8=.. --bag S1=.. --bag S2=.. --bag S8=.. --wire-read-knots 3 --leads 0.425` and `--wire-read-knots 4 --leads 0.450`: both completed, summaries in `$S/out/emit-lead-p2/handoff_w3/summary.md` and `handoff_w4/summary.md`.
- 2026-11-25, `python tools/probes/fresh_rest_census.py temp/logs/launch_*.log` (and with the 2026-11 `~/.ros/log` launch logs): counts above, `$S/out/emit-lead-p2/fresh_rest_census_*.txt`.
- 2026-11-25, `python tools/probes/skill_constants_dump.py OUT.json --emit-lead 0` and `1`, then `--diff`: 22 changed, 0 removed, 0 new.
