---
title: Pattern progression - Columns to Centre and orbit
created: 2026-10-30
status: active
owner: Harrison
last_updated: 2026-10-31
pathway: pattern-progression-columns-to-centre-and-orbit
branch: main
scope: >-
  Take two-ball Columns to the first moving two-ball show Pattern, one new capability per step:
  turned Columns (JP-10), high-low Columns (JP-9), the polygon walk (JP-5) and rotating Columns (JP-11)
  on per-Beat Sites, then Centre and orbit (JP-14) as the Destination. Ten steps: launch current check,
  turned Columns, per-Site Apex, the PWAY-2 dependency, and the Centre and orbit compile.
steps:
  - {id: P1, title: Launch current check, kind: phase, status: complete, effort: xs, hardware: none}
  - {id: P2, title: Turned Columns on the compiled path, kind: phase, status: complete, effort: s, hardware: none}
  - {id: P3, title: JP-10 turned Columns, kind: rung, status: not_started, effort: s, hardware: robot}
  - {id: P4, title: Per-Site Apex, kind: phase, status: complete, effort: m, hardware: none}
  - {id: P5, title: JP-9 high-low Columns, kind: rung, status: not_started, effort: s, hardware: robot}
  - {id: P6, title: Per-Beat Sites dependency owned by PWAY-2, kind: phase, status: not_started, effort: m, hardware: none}
  - {id: P7, title: JP-5 polygon walk, kind: rung, status: not_started, effort: s, hardware: robot}
  - {id: P8, title: JP-11 rotating Columns, kind: rung, status: not_started, effort: l, hardware: robot}
  - {id: P9, title: Centre and orbit compile, kind: phase, status: not_started, effort: m, hardware: none}
  - {id: P10, title: JP-14 Centre and orbit, kind: rung, status: not_started, effort: m, hardware: robot}
related_logbook:
  - 2026-10-30-pattern-progression-plan.md
  - 2026-10-30-jb115-launch-current-bag-check.md
  - 2026-10-31-pattern-progression-p2-turned-columns.md
  - 2026-10-31-pattern-progression-p4-high-low-columns.md
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py::compile_columns
  - ros_ws/src/jugglebot/jugglebot/motion/skills/admissible.py
  - ros_ws/src/jugglebot/jugglebot/records.py::PATTERNS
  - ros_ws/src/jugglebot/jugglebot/skill_node.py::_PATTERNS
  - tools/admissible_sweep.py, sim/skills_gate.py, tools/probes/hand_overspeed_bag_probe.py (the hand bag decode the P1 probes copied)
---

# Pattern progression: Columns to Centre and orbit

> 2026-11-06 (learner-generalisation P3c, branch p3-online-admissibility, pending the owner's merge): the Admissible box, its sweep (`tools/admissible_sweep.py`, `admissible_sweep_run.py`) and `admissible.select` are retired. Wherever this plan says to sweep or re-sweep the box, check a `gate_hash`, or lists the box among a phase's files, that is history; goal acceptance is now the nomination (`skills/check`, `skills/nominate`, the GUI Juggle panel's report, `python sim/skills_gate.py --nominate ...`, I-ADMIT-2) and each commanded Throw is checked online (I-ADMIT-1).

Branch `main`. The schedulable form of Pathway item PWAY-8 (its body carries the ten steps); this plan adds no design. Catalogue: `docs/patterns.md` (JP-1 to JP-18). Design record for the ordering: `logbook/2026-10-30-pattern-progression-plan.md` and the Opus review it cites.

## 1. Context

- Columns is flown two-ball (49 Catches in one Attempt, 2026-10-28, Apex band 0.95). Every Pattern here flies at today's Apex (0.93 to 1.05) under the pose planner, so the Apex lift is not on the critical path.
- The catalogue gives each Pattern a JP-n ID; the machine label in code, the Admissible box and the records ledger stays separate and is owner-named.
- Review finding 1: the first draft ordered the Apex lift first. Wrong: no Pattern here needs a higher Apex, and the inside-contract reach is only Apex about 1.10; the lift is Hand work (IDEA-17) that mainly serves the two-ball oval's Dwell.
- Review finding 2: the shared need is Sites that change Beat by Beat, and Pathway PWAY-2 already owns it (decisions D4/D5: the `BeatEvent` stream, online admissibility, the cold-start Learner). Fixed-Site Patterns fly first on the compiled path; moving-Site Patterns follow PWAY-2.
- Precondition, owned by PWAY-4 and not a step here: the Columns check Sitting under pose (runbook section 11 of `tests/hardware/session_oval_rung1.md`; `plans/active/two-ball-oval.md` section 3).

## 2. Decisions

**Ordering principle:** build per-Beat Sites once, in the place the owner already chose (PWAY-2), and fly the Patterns that exercise it smallest increment first. Fixed-Site work (P1 to P5) does not wait for PWAY-2.

**Owner decisions pending, each with when it is needed:**

- D-a, DECIDED 2026-10-30 (owner): PWAY-2 D4/D5 stand; P2 built plain Admissible box cells.
- D-b, DECIDED 2026-10-30 (owner) for the first two: labels `columns_turned` (JP-10) and `columns_highlow` (JP-9), JP-10 angles 45 and 90 deg. Still open for the polygon, rotating and Centre-and-orbit labels (before P7).
- D-c, ANSWERED BY THE SWEEP 2026-10-31 (owner asked for both pairs): at separation 125 every high-low cell is empty (the low ball's transit falls under the ~0.30 s floor); at separation 100 the 1.00/0.90 pair is admissible at both Sites and 1.05/0.95 only for the high ball. So P5 flies 1.00/0.90 at separation 100. Update 2026-10-31 evening: jugglebot-cd's re-sweep after the Sitting (Hand prime 10.45, floor 0.0614 rev, box 82 cells, gate_hash 961808ee2d20) makes all eight high-low cells admissible, 1.05/0.95 included; that pair now waits only on the faster current capture below, not on IDEA-17. The faster current capture (hand `iq_setpoint` on `/hand_telemetry`) is still wanted before any 1.05 launch.
- D-d, at PWAY-2's P2: whether the JP-10 rows stand in for its rotated-pair flight.

The release-step exception to `HAND_LIMIT_C2` is on the Hand thread (IDEA-17), not decided here.

**Standing preconditions of every Rung:** its runbook section dress-rehearsed on the loaded Jetson against the live `/robot_state`, with every refusal reported at once; `./run_tests.sh --full` green before the Sitting; its own machine label in `records.py::PATTERNS` (and `skill_node._PATTERNS`) before the first Attempt, so a turned or high-low Columns never overwrites the 49-Catch Columns record. The records row is evidence, not the gate: each Rung gate states its own criteria.

## 3. Where we are

| Step | Status | Gate | Blocked on |
|------|--------|------|------------|
| P1 Launch current check | complete 2026-10-30 | bag check: no clipping below release-relative Apex 0.935; the 1.0 to 1.05 regime was never flown | D-c now open |
| P2 Turned Columns on the compiled path | complete 2026-10-31 | 12 pose cells swept twice identical; 45 deg admits the 0.925 band, 90 deg 0.875 and 0.925 | - |
| P3 JP-10 turned Columns | flown 2026-11-01, gate NOT met | 20 Catches at 45 and 90 deg | 45 deg best 2/2, 90 deg best 16/16 (GUARD_LATCHED, BUG-50); see P3 section |
| P4 Per-Site Apex | complete 2026-10-31 | 16 pose cells swept twice identical (8 at separation 100 in the box); 1.00/0.90 admissible at 100 only | - |
| P5 JP-9 high-low Columns | flown 2026-11-01, gate NOT met | 20 Catches, Apex gap, hand iq | 8/8 (GUARD_LATCHED) and 12/13 (leg LIMIT_ACC); apex difference 87 mm; see P5 section |
| P6 PWAY-2 dependency | not started | owned by PWAY-2 | PWAY-2 P1, P3, P4 |
| P7 JP-5 polygon walk | not started | 24 Catches, no first-visit miss | P6 |
| P8 JP-11 rotating Columns | not started | one full turn at 10 deg per Beat | P7 |
| P9 Centre and orbit compile | not started | suite, skills gate, clearance script | P8 |
| P10 JP-14 Centre and orbit | not started | the Destination | P9, BUG-34 |

## 4. Steps

### P1: Launch current check

- Added: whether the launch Throw with the ball aboard already exceeds 50 A. Result (logbook `2026-10-30-jb115-launch-current-bag-check.md`): the corollary "the drive already clips today" is refuted; the claim at release-relative Apex 0.99 to 1.05 is inconclusive because neither bag flew it.
- Check: (2026-10-30, one-off probes `decode.py`, `analyse.py`, `fit.py`, `split.py`, `summ.py` on `~/Desktop/rosbags/2026-10-28_14-29-37` and `2026-10-28_21-02-49`, result: peak measured hand iq 39.90 / 39.83 A, 0 samples at or above 49 A, the record Attempt's launch 26.3 A, all 11 launches 24.8 to 39.9 A; planned release speeds 115.7 to 131.6 rev/s = release-relative Apex 0.86 to 0.935; measured peak acceleration tracks the plan, median 2595 vs 2542 rev/s2).
- What it did not settle: the sampled iq (about 100 Hz, forwarded on change) can read up to 20 % low, and extrapolating the flown strokes to the 3509 rev/s2 launch peak brackets 50 A (35 A by regression, 53 to 56 A by the worst sampled stroke; the envelope's B3 model says 53.6 A and is not refuted). The filing's error was the Apex mapping: the R0 numbers are release-relative, the goal `apex_m` is above the Catch plane, and the Learner's per-Throw commands sat about 0.05 m below the 0.95 goal.
- Items consumed: BUG-40 (back to backlog with this result). Side finding filed as its own bug: brake and hold currents sit far below the envelope's model.
- Owner decision: D-c.

### P2: Turned Columns on the compiled path

- Adds: a rotation parameter on the Columns Site pair, two Admissible box cells (45 and 90 deg), a new machine label.
- Done 2026-10-31 (logbook `2026-10-31-pattern-progression-p2-turned-columns.md`): rotation on `columns_sites`, goal `columns_turned` with the `columns_angle_deg` parameter, label in records, sweep and sim-gate flags; the whole box re-swept twice byte-identical (sites.py is gated), 12 turned cells appended. Finding: 45 deg admits only the 0.925 band, 90 deg the 0.875 and 0.925 bands, so P3 flies at Apex 0.95 (45 deg) and 0.90 or 0.95 (90 deg).
- Depends on: D-a, D-b (both decided). Items consumed: IDEA-11.

### P3: JP-10 turned Columns

- Adds: two-ball Columns off the x axis.
- Rung gate: at 45 and at 90 deg, 20 consecutive Catches in one Attempt, landing scatter within 3 mm of a straight Columns control flown the same Sitting. Each angle has its own `plant_id`, so the rows answer PWAY-2 P2(b) (does the lateral bias rotate with the Pattern). May share a Sitting with P5.
- Flown 2026-11-01 (first Sitting at hold 0), gate NOT met: 45 deg best 2/2 (three MISSED_CATCH on an empty cup, two after TRACKER-IDENTITY-REFUSED, one after a half-speed launch 2341 vs 4268 mm/s; one leg LIMIT_VEL 361 > 350 at 0/1); 90 deg best 16/16 ending GUARD_LATCHED (hand feedback starvation, BUG-50). No HAND_LIMIT line. Hand iq 53.3 A on a 45 deg hop launch (over the 50 A watch, BUG-51). Tracker apex 1.01-1.04 m turned vs 0.93-0.96 straight.
- The plan's "each angle has its own `plant_id`" was NOT done: one `plant_id` (`jugglebot`) served straight, 45 and 90, all rows in one `memory.csv`. Rows are keyed by the release-site xy (I-LEARN-1), and the turned Sites carry distinct xy, so the angles do not share exact rows, but nothing separates them by angle. PWAY-2 P2(b) (does the lateral bias rotate with the Pattern) is NOT answered: no 20-run, no tracker landing on any miss. A re-fly sets `plant_id` per angle.
- Flown again 2026-11-03 (second flight), gate NOT met: 45 deg COMPLETED 19/19 twice of 5 Attempts (19 Catches plus the Stop throw, a new record; the other three MISSED_CATCH 7/7, 3/3, 5/5), 90 deg 0 of 7 (best 12/12, one LANE_REFUSED 0/0). At 90 deg Ball 2 landed 37-50 mm outward of its aim in 5 of 7 against a 10 mm outward authority (`AIM-LATERAL-CLAMPED`). `plant_id` was NOT set per angle again, so PWAY-2 P2(b) is still unanswered; a 14th goal was rejected at angle 135 (left at the default). No HAND_LIMIT line. Landing scatter control sd x 25.5 / y 19.4 mm; 45 deg 21.8 / 22.9; 90 deg 17.8 / 24.1 with the mean x shifted -13 mm. Re-fly with Throws 21, `plant_id` per angle, and the scatter clause defined (proposed: per ball per axis, mean within 15 mm and sd within 10 mm of the control; owner to confirm).
- Depends on: P2. Items consumed: IDEA-11. Plus the three standing Rung preconditions (section 2).

### P4: Per-Site Apex

- Adds: one Apex per Site in the Columns compile and a two-band box cell, new label.
- Done 2026-10-31 (logbook `2026-10-31-pattern-progression-p4-high-low-columns.md`): `Pattern.apex_m_ball_b`, `columns_ball_timing`, per-ball dwell in the executor's box select, goal `columns_highlow` with the `apex_m_ball2` parameter, box label `columns_highlow`, `--apex-pair` sweep; 16 cells swept for both pairs at separations 125 and 100 (the 8 at 100 are in the box; the 125 set is all empty and recorded in the logbook). Finding: nothing fits at 125; at 100 the 1.00/0.90 pair fits at both Sites, 1.05/0.95 only the high ball. Equal Apexes are bit-identical to before (672 configurations).
- Depends on: D-c (answered by the sweep).
- Items consumed: IDEA-10. Owner decision: D-c.

### P5: JP-9 high-low Columns

- Adds: two Apexes in one Pattern.
- Flies at separation 100 with the 1.00/0.90 pair (1.05/0.95 is admissible since the 2026-10-31 re-sweep but waits for the faster current capture); the GUI Juggle panel's `Columns — high-low` entry (2026-10-31) writes `apex_m_ball2` to the node before each Start; the terminal path (`ros2 param set /skill_node apex_m_ball2 0.90`, then `ros2 action send_goal`) stays as the fallback.
- Rung gate: 20 consecutive Catches in one Attempt; Tracker mean Apex difference within 20 mm of the commanded difference; hand iq under 45 A at every launch. Stop rule: any hand `iq_setpoint` sample at 50.0 A ends the Attempt (needs the faster iq capture named under D-c, or the Sitting flies the lower pair).
- Flown 2026-11-01, gate NOT met: 8/8 ended GUARD_LATCHED (teensy_bridge_node stall, BUG-34) and 12/13 ended leg LIMIT_ACC 5142.8 > 5000 (a late re-send seed, BUG-5 class). Tracker apex difference 87 mm vs 100 commanded (Ball 1 0.997, Ball 2 0.910 m). Hand iq peaks 32.1 / 41.5 A (under the 45 A bar). First high-low records are in `docs/records.md`.
- Flown again 2026-11-03 (second flight), gate NOT met: 11 Attempts, best 18 consecutive Catches at 1.00/0.90 (6 Attempts) and 15 at 1.05/0.95 (5 Attempts). Tracker apex difference 119.2 mm (1.00/0.90) and 89.7 mm (1.05/0.95) against 100; Ball 1 reached only 1.019 m when commanded 1.05. Launch iq at most 36.5 A; the Hand brake reached the 60 A clamp twice (-62.2 A measured). Plan D-c answered: 1.05/0.95 was flown and is not worth another flight at the current limits. Five of 11 Attempts ended on Ball 2's first Throw landing about 180 mm outward (BUG-63); one Attempt ended SPINOUT_DETECTED (BUG-57 class). Throws 20 cannot reach the gate (the 20th is the Stop Throw, 19/19): fly Throws 21.
- Depends on: P4. Items consumed: IDEA-10. Plus the three standing Rung preconditions.

### P6: Per-Beat Sites dependency owned by PWAY-2

- Not built here; owned by PWAY-2 (`plans/active/learner-generalisation.md`): Learner core P1, online admissibility P3, `BeatEvent` stream P4. This Pathway waits on it and adds no box key of its own.
- Gate (Phase check): PWAY-2 P1, P3 and P4 complete; (date, command, result) is recorded in that plan - to be linked.
- Depends on: PWAY-2. Items consumed: PWAY-2. Owner decision: D-d at PWAY-2's P2.

### P7: JP-5 polygon walk

- Adds: the first Pattern whose Site changes every Throw, on `BeatEvent`, one ball.
- Rung gate: a square whose edge the Hop already flies, 24 consecutive Catches in one Attempt (six laps), no miss on a first visit to a Site.
- Depends on: P6. Items consumed: IDEA-9. Plus the three standing Rung preconditions.

### P8: JP-11 rotating Columns

- Adds: two-ball per-Beat Sites, the smallest two-ball move (11 mm per Beat at 5 deg, inside the Learner's 20 mm lateral clamp). A Ladder of 5, 10 and 20 deg per Beat in one Sitting.
- Rung gate: one full turn of the pair at 10 deg per Beat without a drop (about 36 consecutive Catches).
- Depends on: P7. Items consumed: IDEA-12. Plus the three standing Rung preconditions.

### P9: Centre and orbit compile

- Adds: a Site list for Ball 2, aimed Ball 2 Throws across a 77 mm chord, a fixed centre Site for Ball 1.
- Gate (Phase check): (date, `./run_tests.sh --full` plus `sim/skills_gate.py` on the new label plus the ball-to-ball clearance script at radius 100, result) - to be filled.
- Depends on: P8. Items consumed: IDEA-14.

### P10: JP-14 Centre and orbit

- Adds: asymmetric Site roles with a moving second Site. This is the Destination.
- Rung gate: 30 consecutive Catches in one Attempt at radius 100 mm with 8 Sites per lap, under pose.
- Depends on: P9 and BUG-34 (the 70 ms stream STOP cliff ended a 0.45 oval Attempt at Throw 41; a 30-Catch Attempt is exposed to it). Items consumed: IDEA-14, BUG-34. Plus the three standing Rung preconditions.

## 5. Runs in parallel

- The two-ball oval (PWAY-4) keeps priority and its Sittings; Rungs here ride as the tail of a PWAY-4 Sitting where they can.
- The Hand thread (IDEA-17, BUG-40): in-contract `max_acc_z` first (Apex about 1.10), then the release step, which needs an owner decision on a named exception to `HAND_LIMIT_C2`, then a one-ball self-throw Apex Ladder under the R0 stop criteria plus a Catch-bounce check. It mainly serves PWAY-4's Dwell and only optionally JP-9 at 1.05/0.95.
- The human-partner Pathway (IDEA-16), still to be filed: first a Phase replaying the 2026-09-30 bag through the throw detector, and an owner decision on safety rules for a person inside the Platform's reach before any Rung.

## 6. Not in this plan

- JP-13 (IDEA-13): moves the Catch plane with the Platform, and the glossary defines the Catch plane as fixed; needs a glossary change first. Promote after the Destination.
- JP-15 (IDEA-15), JP-12, JP-16: need PWAY-4's P4 working point.
- JP-17 (IDEA-16): separate Pathway (a person in the workspace, perception of a human Throw).
- JP-18: not yet probed offline.
- The Hand bumper (IDEA-17): a safety stop on the Hand thread.
