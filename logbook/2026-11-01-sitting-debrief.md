---
title: "Sitting 2026-11-01 debrief: 23 goals (straight Columns, first high-low and first turned Attempts); four open hypotheses reframed by the data"
type: investigation
date: 2026-11-01
status: in-progress
phase: "pattern progression"
related_plan: pattern-progression.md
related_issues: []
files_changed:
  - logbook/2026-11-01-sitting-debrief.md
  - logbook/INDEX.md
subsystem: [motion, ros, can, config, tools]
tags: [safety, performance, kinematics, dynamics, docs]
---

# Sitting 2026-11-01 debrief

This is the first run of `/sitting-debrief` (formerly `/sitting-wrapup`, see `2026-11-01-sitting-debrief-command.md`): analysis first, then paperwork. **No code changed in this entry.** The code changes it decides (HAND_STROKE fix, instrumentation, observability, limit raise, re-swept box) land in a separate commit with their own entry, `logbook/2026-11-01-hand-limit-4500-and-stroke-floor.md` (to be written).

## Summary

Launch 22:08, bag `~/Desktop/rosbags/2026-11-01_22-08-45`, launch log `~/.ros/log/2026-11-01-22-08-45-206249-jetson-2419376/launch.log` (5313 lines). 23 goals: 10 straight Columns (apex 0.95, sep 125, dwell 0.27, planner pose, hold 0), the first high-low (JP-9) and first turned (JP-10) Attempts ever. Owner saw nothing at either GUARD_LATCHED, catches were clean to the end. Best Attempt: straight 29 Throws / 28 caught. Ends (`tools/probes/limit_refusal_tally.py`, 23 goals): HAND_STROKE 7, HAND_LIMIT_ACC 6, MISSED_CATCH 5, GUARD_LATCHED 2, LIMIT_ACC 1, LIMIT_VEL 1. The three largest end classes were each attributed to a different cause than the Item that tracked them.

## Verdicts

| Item | flown | verdict | evidence |
|---|---|---|---|
| BUG-42 | 10 straight goals, none a 50-Throw goal | FAIL | 6 of 10 ended HAND_LIMIT_ACC 3910.6-4096.8 vs 3900 |
| BUG-43 | 10 straight + 5 turned-90 starts | FAIL (7/10 straight pass first 2 Catches) | HAND_STROKE at skill 2 in 3 straight and 4 turned starts; no inboard bias |
| BUG-4 | yes | PASS on the goal, FAIL launch-wide | 28/29 goal max 32.8 ms; launch max 47.8 ms (goal 16/17, L1264) |
| IDEA-10 | 2 Attempts | FAIL on 20 consecutive; apex gap and iq PASS | 8/8 GUARD_LATCHED, 12/13 LIMIT_ACC; apex diff 87 mm vs 100 (inside 20 mm); iq max 41.5 A |
| IDEA-11 | 45 and 90 deg | FAIL (no 20-run at either angle) | 45: best 2/2; 90: best 16/16 (GUARD_LATCHED); no HAND_LIMIT line |
| BUG-19 (bench leg) | not in the artefacts | not flown / unknown | no new run dir in `temp/probes/lead_clamp_gap_bench` (newest 2026-10-27) |

## Symptoms and Diagnosis

**Straight Columns ends (10 goals):** HAND_STROKE (-0.004, -0.002, -0.001 rev; 3), HAND_LIMIT_ACC (6), MISSED_CATCH (1, cup empty, 3/3 caught). Tracker Apex on the refused balls 0.938-0.956, inside the 0.93-0.97 spread of the passing ones.

**HAND_LIMIT_ACC (BUG-42).** All six ends are the first install of a Catch seeded from a BUMPED predecessor (7 of 7 refusals counting one re-send; 3 of 5 on 10-31). 25 BUMPs were accepted in straight Columns; 6 were followed by a refusal; about 59 first installs on unbumped seeds gave 0.

| refusal | Catch / Ball | peak (% of 3900) | previous bump delta (mm) |
|---|---|---|---|
| L568 | skill 4, Ball 1 | 4002.8 (102.6) | (10.3, 3.4) |
| L690 | skill 4, Ball 1 | 4013.0 (102.9) | (14.7, 7.3) |
| L979 | skill 6, Ball 1 | 4084.0 (104.7) | (15.3, -3.6) |
| L1354 | skill 17, Ball 2 | 3910.6 (100.3) | (11.1, 0.2) |
| L1966 | skill 29, Ball 2 | 3918.4 (100.5) | (10.1, 0.0) |
| L2668 | skill 14, Ball 1 | 4096.8 (105.0) | (20.4, 4.4) |

Offline (production `plan_segment` + `apply_catch_bump`, HEAD, hold 0, pose) reproduces the live peaks within -1.9 to +3.0 %: unbumped successor 2688-2811 (69-72 %); the peak moves to span 1 of the 0.050 s post-release hold and climbs about 240 rev/s^2 per mm of bump x-delta above ~4 mm (x scan: 9 mm 3823, 9.5 mm 3930 refused). y-only bumps (3, all Ball 2) never refused. The learner command and the aim move the peak by at most 0.7 pt; the solve is not late (26-57 ms wall, deadline +250..+367 ms). The probe misses one live case (Ball 2 bump (10.3, -0.7) at L1252 passed; offline gives 102.3 %). Why lateral platform velocity at the release loads the Hand inside the post-release hold is NOT traced.

**HAND_STROKE (BUG-43).** 9 install refusals (L455, 784, 1457, 2139, 2456, 4853, 4933, 5024, 5169), 7 goals ended (L472, 800, 2156, 4870, 4953, 5043, 5190: 3 straight, 4 turned), every one the skill-2 CATCH (hop start) at t = 0.488 s, hand -0.000..-0.004 rev. Failure path: `HAND_STROKE_MIN_REV = 0.0` (`feasibility.py:1123`) is strict for in-band plans; `validate_cycle` samples 4 points per 25 ms span and refuses with no tolerance (`feasibility.py:1849`), and t = 0.4875 s is the mid-span sample in all 9; the cup QP's z box rows exist for knots only (`cup_cycle.py:1916-1924`, class already noted at `cup_cycle.py:846-858`), floor = band floor + 2 mm inset (0.0614 rev). The hop-start catch reverses into the next throw at the stroke bottom (~730-830 rev/s^2); a cubic reversing mid-span between two floor knots dips a x dt^2 / 8 below them, so the 2 mm inset is used up at ~790 rev/s^2 (0.305 rev = 9.9 mm at the 3900 cap; the old 10 mm inset covered that, which is why it never fired before 2026-10-31). Physically harmless: the firmware clips the Hand at 0.0 rev and the end stop is 0.1 rev below; -0.004 rev is 0.13 mm. Open: the 10-31 note says insets 0/0.5/1 mm refused at only -0.001..-0.002, which a constant mid-span dip would not predict (replay one to settle).

**GUARD_LATCHED, high-low (L2906, dev +8.967 rev, 8/8).** BUG-34's mechanism (STOP, refused resume, plan walks away from the held Hand: sched_stops +2, sched_refused 0 -> 2, hold latched; held at 1.157 rev 25 ms into the throw stroke, plan top ~10.12 rev). The stall was `teensy_bridge_node`: setpoint echo gap 109 ms, hand telemetry 94 ms, robot_state 89 ms, the only >60 ms gap in 69,856 telemetry intervals; the emitter was clean (max_emit_gap 25.7-28.8 ms in every window) and the preceding CATCH solve ordinary (58.8 ms).

**GUARD_LATCHED, turned-90 (L4695, dev +3.707 rev, 16/16).** A different mechanism. No STOP (sched_stops constant 12). Hand ODrive heartbeat dropout 701 ms (hb_stale_mask 64), axis-6 encoder age max 100.1 ms (2.2 ms normally); the bridge's anchor froze in 75-95 ms chunks, the 2.0 rev lead clamp (`leg_interp.cpp:1402-1406`) pinned the throw command to stale-anchor + 2.0 (echo 3.413...5.036 matches 0.332 + 27.05 x age + 2.0), the Hand genuinely fell behind, and the guard fired correctly on a fresh sample. Why the Hand's frames starve is open (Hand is node 6, lowest CAN arbitration priority on the bus: hypothesis, not measured).

**"ENDED: N ms stale" lines (21 episodes: legs 0-5 and 4 Hand episodes of 601-798 ms).** Emitted by `teensy_bridge_node.py:7440-7495`, diagnostic only, an edge on the firmware heartbeat-staleness mask; unrelated to the setpoint STOPs. The census counts frames, not freshness ("0 co-dropped" looked healthy while the encoder was 100 ms stale): `cache_diag age_max_us_6` is the freshness number. 11 of 13 non-trip STOP increments had no Jetson-side gap and are unattributable until the firmware counters leave serial.

**Leg LIMIT_* (BUG-5 class).** Harmless re-send refusals 1.0-1.9 % over (4 of 4 caught). Two ENDs: high-low LIMIT_ACC 5142.8 (+2.9 %, seed skill 14 re-sent +0.011 s, clamped outward) and turned LIMIT_VEL 361.0 (+3.1 %, seed re-sent +0.023 s; the fallback re-solved the same aim since the learned prior equalled y_d). Both are first installs seeded from a late re-sent predecessor: the same structural class as the BUMP (a late edit to record N is gated only on itself, never on record N+1's splice). Leg ends are 2 of 23; the margin is not the problem.

**Other findings.** Ball 2 learner command pinned at the (-20, -20) mm corner (u_y = -20 in 37/40 straight rows; the box prints BOX-EXPECTED-LANDING-UNMET every Ball 2 Catch in the long goal). Hand iq peaked 53.3 A at L3521 (45 deg hop launch, apex 1.027), over the 50 A watch; high-low launches max 41.5 A. BUG-4: goal 28/29 p50 26.2 / p99 30.9 / max 32.8 ms; launch-wide max 47.8 ms right after a 74-83 ms CATCH solve. IDEA-10: Tracker Apex Ball 1 0.9972 (n=9), Ball 2 0.9102 (n=10), difference 87.0 mm (8/8 and 12/13 Catches). IDEA-11: 45 deg best 2/2, 90 deg 16/16; MISSED_CATCH at turned angles read "cup EMPTY ... no ball left" (ball not arriving, tracker identity refusals), not a rotated bias.

## Discussion

Four hypotheses were reframed; each stood on the Item text until the bag disagreed.

| Item hypothesis | What the data says | Why it matters |
|---|---|---|
| BUG-42: the cell sits at a box edge, the aim fallback (held night-shift branch, `HAND_LIMIT_` prefix) would help | Not flown: installed `executor.py` and HEAD have 0 `HAND_LIMIT_` hits, `origin/night-shift/2026-11-01-BUG-42` unmerged. And it could not have helped: the fallback changes only the aim, the cause is the seed (aim moves the peak <= 0.3 pt). `tools/admissible_sweep.py` has no bump model, so "inside box" is true and blind | The Runbook pass test could not have passed either way. Merge the branch only with exact codes `HAND_LIMIT_ACC`/`HAND_LIMIT_VEL` (closes the HAND_LIMIT_C2 audit finding); it restores the designed fallback and does not cure BUG-42. Item retitled to the bump-seed mechanism |
| BUG-43: hop-start throw lands 25-56 mm inboard at hold 0 | Not reproduced: hop landing x mean +1.1 mm, sd ~19 mm, worst 38 mm. The start failures are HAND_STROKE floor dips: the planner bounds height at knots, the gate samples the continuum with a strict floor | Largest end class (7 of 23 goals); a planner-vs-gate contract mismatch, not a lateral-aim problem |
| BUG-34: the emitter wake gap stalls the setpoint stream | For the high-low latch the emitter was clean (25.7-28.8 ms) and `teensy_bridge_node` itself was starved 109 ms; the STOP cliff is still the mechanism. The launch-log STOP text blames the Jetson-side stream only by luck: nothing measures the forwarder's own gap | Instrument the forwarder; the emitter's gap monitor cannot see this |
| The two GUARD_LATCHED are one phenomenon | Turned-90 is a different mechanism: Hand ODrive feedback starvation (701 ms), stale-anchor lead clamp, real 3.7 rev deviation, guard correct | Needs the bench CAN arbitration test, not the STOP-cliff work |

Ruled out: the learner (u pinned at -20 on passing and refused Catches alike), the apex (0.938-0.956 within the passing spread), solve lateness (26-57 ms), the 10 % leg margin (leg ends are successors of late re-sends). Option trade-offs recorded for the owner: a bump x-cap at ~9 mm (offline edge 98 %) would decline or shrink 19 of 25 bumps to stop 6, a poor trade versus a successor-feasibility check that declines only the infeasible ones; limit raise 3920 keeps 2 of 6, 4100 keeps 6 of 6 but offline bumps of 20-22 mm reach 4177-4209 (107-108 %), 4250 covers today's range.

## Owner decisions (this debrief)

| Topic | Decision |
|---|---|
| HAND_STROKE | Fix with mid-span cup-QP rows (planner stops planning a continuum the gate refuses), plus a 0.01 rev (0.33 mm) gate tolerance as stopgap; inset unchanged |
| HAND_LIMIT_ACC | 3900 -> 4500 rev/s^2 with `hand_curr_limit_a` 50 -> 60 A, then a box re-sweep (owner open to 70-80 A later). The structural fix (successor check in the bump decision; trace the hold-row coupling first, option (c)) is FILED on BUG-42, not built |
| Guard latches | Instrument: bridge forward-gap monitor in `_setpoint_loop`, hand encoder age (`cache_diag age_max`) in the dropout census, STOP warning reworded to name both gaps. No scheduling change (CPU isolation, priority, firmware cover) yet |
| Leg margin | Stays 10 % (BUG-5 option 1) |

## Items touched

BUG-42 (retitled: "A BUMP of catch N leaves a release seed catch N+1 cannot plan from: HAND_LIMIT_ACC 3910-4097 vs 3900 at the next first solve"), BUG-43, BUG-4, IDEA-10, IDEA-11, BUG-34, BUG-5: progress notes. New: BUG-49 HAND_STROKE at the hop-start CATCH (knot-only box vs continuum gate); BUG-50 Hand ODrive feedback starvation (701 ms dropout, lead-clamped throw); BUG-51 Hand iq 53.3 A on a 45 deg hop launch; BUG-52 Ball 2 learner command pinned at the (-20, -20) corner. Plans noted: `pattern-progression.md` (P3, P5), `two-ball-oval.md` (note under 2.6: the box certifies unbumped seeds), `learner-generalisation.md` (Phase 2). `plant_id` finding: the node param (`skill_node.py:1094`) is global, so straight, 45 and 90 share one id; rows are keyed by release-Site xy (I-LEARN-1) and turned Sites have distinct xy, so exact rows are not shared, but nothing separates by angle and the Runbook never set it. PWAY-2 P2(b) (does the bias rotate with the pair?) is NOT answered: no 20-run, and the early 45 deg landings are single samples; at 90 deg no rotated bias is visible in the 16/16 run (Ball 1 (3.2, 5.4), Ball 2 (0.0, 5.1) mm). No bug filed for it; recorded in P3.

## Open Questions

- Why lateral platform velocity at the release loads the Hand inside the 0.050 s post-release hold (hold-row or detach-cone defect?). Reproducer: the bump-seed probe (to be promoted to `tools/probes/`; it imports an uncommitted night-shift file).
- Late RESEND of record N gated only on itself: record the invariant "an edit to record N keeps N+1's splice feasible" in `motion/skills/INVARIANTS.md` when the bump fix lands.
- Peak Hand acceleration of accepted installs is not logged (`trajectory_node.py:5269` prints only on refusal).
- Why the Hand's CAN frames starve (arbitration vs bus load, bench test).
- Replay a 10-31 inset 0/0.5/1 mm refusal against the mid-span dip model.
- Whether a stale-anchor lead clamp mid-throw should end the throw rather than let the guard latch.
- CATCH-AIM-FALLBACK re-solves the same aim when the learned prior equals y_d (L3957; wasted 32 ms solve).

## Verification

Analysis only, no code. Paperwork: 2026-11-01, `pytest tests/sim/test_plans_index.py tests/sim/test_mapkit*.py tests/sim/test_docs_links.py tests/sim/test_map_items.py tests/sim/test_map_items_rated.py tests/sim/test_glossary_avoid_words.py tests/sim/test_sitting_sections.py -q` after `python -m mapkit gen`: 303 passed, 40 skipped. Command rename: 2026-11-01, `pytest tests/sim/test_sitting_debrief_protocol.py tests/sim/test_night_shift_gate.py tests/sim/test_mapkit*.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_plans_index.py -q`: 285 passed in 56.63 s. Logbook front matter and links for this entry: see the commit message.

## Outcome

Open. The analysis is done and the decisions are recorded; the HAND_STROKE fix, instrumentation, limit raise and re-swept box follow in their own commit and entry, after which BUG-42/BUG-43 are re-flown.
