---
title: Pattern progression - Columns to Centre and orbit
created: 2026-10-30
status: active
owner: Harrison
last_updated: 2026-10-30
pathway: pattern-progression-columns-to-centre-and-orbit
branch: main
scope: >-
  Take two-ball Columns to the first moving two-ball show Pattern, one new capability per step:
  turned Columns (JP-10), high-low Columns (JP-9), the polygon walk (JP-5) and rotating Columns (JP-11)
  on per-Beat Sites, then Centre and orbit (JP-14) as the Destination. Ten steps: launch current check,
  turned Columns, per-Site Apex, the JB-38 dependency, and the Centre and orbit compile.
steps:
  - {id: P1, title: Launch current check, kind: phase, status: complete, effort: xs, hardware: none}
  - {id: P2, title: Turned Columns on the compiled path, kind: phase, status: not_started, effort: s, hardware: none}
  - {id: P3, title: JP-10 turned Columns, kind: rung, status: not_started, effort: s, hardware: robot}
  - {id: P4, title: Per-Site Apex, kind: phase, status: not_started, effort: m, hardware: none}
  - {id: P5, title: JP-9 high-low Columns, kind: rung, status: not_started, effort: s, hardware: robot}
  - {id: P6, title: Per-Beat Sites dependency owned by JB-38, kind: phase, status: not_started, effort: m, hardware: none}
  - {id: P7, title: JP-5 polygon walk, kind: rung, status: not_started, effort: s, hardware: robot}
  - {id: P8, title: JP-11 rotating Columns, kind: rung, status: not_started, effort: l, hardware: robot}
  - {id: P9, title: Centre and orbit compile, kind: phase, status: not_started, effort: m, hardware: none}
  - {id: P10, title: JP-14 Centre and orbit, kind: rung, status: not_started, effort: m, hardware: robot}
related_logbook:
  - 2026-10-30-pattern-progression-plan.md
  - 2026-10-30-jb115-launch-current-bag-check.md
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py::compile_columns
  - ros_ws/src/jugglebot/jugglebot/motion/skills/admissible.py
  - ros_ws/src/jugglebot/jugglebot/records.py::PATTERNS
  - ros_ws/src/jugglebot/jugglebot/skill_node.py::_PATTERNS
  - tools/admissible_sweep.py, sim/skills_gate.py, tools/probes/hand_overspeed_bag_probe.py (the hand bag decode the P1 probes copied)
---

# Pattern progression: Columns to Centre and orbit

Branch `main`. The schedulable form of Pathway item JB-116 (its body carries the ten steps); this plan adds no design. Catalogue: `docs/patterns.md` (JP-1 to JP-18). Design record for the ordering: `logbook/2026-10-30-pattern-progression-plan.md` and the Opus review it cites.

## 1. Context

- Columns is flown two-ball (49 Catches in one Attempt, 2026-10-28, Apex band 0.95). Every Pattern here flies at today's Apex (0.93 to 1.05) under the pose planner, so the Apex lift is not on the critical path.
- The catalogue gives each Pattern a JP-n ID; the machine label in code, the Admissible box and the records ledger stays separate and is owner-named.
- Review finding 1: the first draft ordered the Apex lift first. Wrong: no Pattern here needs a higher Apex, and the inside-contract reach is only Apex about 1.10; the lift is Hand work (JB-114) that mainly serves the two-ball oval's Dwell.
- Review finding 2: the shared need is Sites that change Beat by Beat, and Pathway JB-38 already owns it (decisions D4/D5: the `BeatEvent` stream, online admissibility, the cold-start Learner). Fixed-Site Patterns fly first on the compiled path; moving-Site Patterns follow JB-38.
- Precondition, owned by JB-40 and not a step here: the Columns check Sitting under pose (runbook section 11 of `tests/hardware/session_oval_rung1.md`; `plans/active/two-ball-oval.md` section 3).

## 2. Decisions

**Ordering principle:** build per-Beat Sites once, in the place the owner already chose (JB-38), and fly the Patterns that exercise it smallest increment first. Fixed-Site work (P1 to P5) does not wait for JB-38.

**Owner decisions pending, each with when it is needed:**

- D-a, before P2 writes code: reaffirm or reopen JB-38 D4/D5 for these Patterns. It decides whether P2 builds plain Admissible box cells or a Pattern-relative key. Default if unanswered: D4/D5 stand, plain cells.
- D-b, before P2: the machine labels for the new Patterns (turned, high-low, polygon, rotating, Centre and orbit) and the JP-10 angles (proposed 45 and 90 deg).
- D-c, before P4: which JP-9 pair flies first. P1 could not decide it from the bags (they never flew above release-relative Apex 0.935; the 50 A crossing is no lower than release-relative 0.99, about 1.02 above the Catch plane). Options: fly 1.05/0.95 with a hand-current stop rule and a faster iq capture (hand `iq_setpoint` on `/hand_telemetry`, or a 250 Hz Diagnostic on axis 6), or fly 1.00/0.90 first.
- D-d, at JB-38's P2: whether the JP-10 rows stand in for its rotated-pair flight.

The release-step exception to `HAND_LIMIT_C2` is on the Hand thread (JB-114), not decided here.

**Standing preconditions of every Rung:** its runbook section dress-rehearsed on the loaded Jetson against the live `/robot_state`, with every refusal reported at once; `./run_tests.sh --full` green before the Sitting; its own machine label in `records.py::PATTERNS` (and `skill_node._PATTERNS`) before the first Attempt, so a turned or high-low Columns never overwrites the 49-Catch Columns record. The records row is evidence, not the gate: each Rung gate states its own criteria.

## 3. Where we are

| Step | Status | Gate | Blocked on |
|------|--------|------|------------|
| P1 Launch current check | complete 2026-10-30 | bag check: no clipping below release-relative Apex 0.935; the 1.0 to 1.05 regime was never flown | D-c now open |
| P2 Turned Columns on the compiled path | not started | suite, box twice identical, skills gate | D-a, D-b |
| P3 JP-10 turned Columns | not started | 20 Catches at 45 and 90 deg | P2 |
| P4 Per-Site Apex | not started | suite, box sweep, skills gate | D-c |
| P5 JP-9 high-low Columns | not started | 20 Catches, Apex gap, hand iq | P4 |
| P6 JB-38 dependency | not started | owned by JB-38 | JB-38 P1, P3, P4 |
| P7 JP-5 polygon walk | not started | 24 Catches, no first-visit miss | P6 |
| P8 JP-11 rotating Columns | not started | one full turn at 10 deg per Beat | P7 |
| P9 Centre and orbit compile | not started | suite, skills gate, clearance script | P8 |
| P10 JP-14 Centre and orbit | not started | the Destination | P9, JB-98 |

## 4. Steps

### P1: Launch current check

- Added: whether the launch Throw with the ball aboard already exceeds 50 A. Result (logbook `2026-10-30-jb115-launch-current-bag-check.md`): the corollary "the drive already clips today" is refuted; the claim at release-relative Apex 0.99 to 1.05 is inconclusive because neither bag flew it.
- Check: (2026-10-30, one-off probes `decode.py`, `analyse.py`, `fit.py`, `split.py`, `summ.py` on `~/Desktop/rosbags/2026-10-28_14-29-37` and `2026-10-28_21-02-49`, result: peak measured hand iq 39.90 / 39.83 A, 0 samples at or above 49 A, the record Attempt's launch 26.3 A, all 11 launches 24.8 to 39.9 A; planned release speeds 115.7 to 131.6 rev/s = release-relative Apex 0.86 to 0.935; measured peak acceleration tracks the plan, median 2595 vs 2542 rev/s2).
- What it did not settle: the sampled iq (about 100 Hz, forwarded on change) can read up to 20 % low, and extrapolating the flown strokes to the 3509 rev/s2 launch peak brackets 50 A (35 A by regression, 53 to 56 A by the worst sampled stroke; the envelope's B3 model says 53.6 A and is not refuted). The filing's error was the Apex mapping: the R0 numbers are release-relative, the goal `apex_m` is above the Catch plane, and the Learner's per-Throw commands sat about 0.05 m below the 0.95 goal.
- Items consumed: JB-115 (back to backlog with this result). Side finding filed as its own bug: brake and hold currents sit far below the envelope's model.
- Owner decision: D-c.

### P2: Turned Columns on the compiled path

- Adds: a rotation parameter on the Columns Site pair, two Admissible box cells (45 and 90 deg), a new machine label.
- Gate (Phase check): (date, `./run_tests.sh --full` plus the box re-swept twice bit-identical plus `sim/skills_gate.py` on the new label, result) - to be filled. BLAS one thread; no gated-file edit from sweep start to commit.
- Depends on: D-a, D-b. Items consumed: JB-108.

### P3: JP-10 turned Columns

- Adds: two-ball Columns off the x axis.
- Rung gate: at 45 and at 90 deg, 20 consecutive Catches in one Attempt, landing scatter within 3 mm of a straight Columns control flown the same Sitting. Each angle has its own `plant_id`, so the rows answer JB-38 P2(b) (does the lateral bias rotate with the Pattern). May share a Sitting with P5.
- Depends on: P2. Items consumed: JB-108. Plus the three standing Rung preconditions (section 2).

### P4: Per-Site Apex

- Adds: one Apex per Site in the Columns compile and a two-band box cell, new label.
- Gate (Phase check): (date, `./run_tests.sh --full` plus the box sweep plus `sim/skills_gate.py`, result) - to be filled.
- Depends on: D-c. P1 found no clipping up to release-relative Apex 0.935 and could not see the 1.0 to 1.05 regime; the cells are swept at the pair D-c names.
- Items consumed: JB-107. Owner decision: D-c.

### P5: JP-9 high-low Columns

- Adds: two Apexes in one Pattern.
- Rung gate: 20 consecutive Catches in one Attempt; Tracker mean Apex difference within 20 mm of the commanded difference; hand iq under 45 A at every launch. Stop rule: any hand `iq_setpoint` sample at 50.0 A ends the Attempt (needs the faster iq capture named under D-c, or the Sitting flies the lower pair).
- Depends on: P4. Items consumed: JB-107. Plus the three standing Rung preconditions.

### P6: Per-Beat Sites dependency owned by JB-38

- Not built here; owned by JB-38 (`plans/active/learner-generalisation.md`): Learner core P1, online admissibility P3, `BeatEvent` stream P4. This Pathway waits on it and adds no box key of its own.
- Gate (Phase check): JB-38 P1, P3 and P4 complete; (date, command, result) is recorded in that plan - to be linked.
- Depends on: JB-38. Items consumed: JB-38. Owner decision: D-d at JB-38's P2.

### P7: JP-5 polygon walk

- Adds: the first Pattern whose Site changes every Throw, on `BeatEvent`, one ball.
- Rung gate: a square whose edge the Hop already flies, 24 consecutive Catches in one Attempt (six laps), no miss on a first visit to a Site.
- Depends on: P6. Items consumed: JB-106. Plus the three standing Rung preconditions.

### P8: JP-11 rotating Columns

- Adds: two-ball per-Beat Sites, the smallest two-ball move (11 mm per Beat at 5 deg, inside the Learner's 20 mm lateral clamp). A Ladder of 5, 10 and 20 deg per Beat in one Sitting.
- Rung gate: one full turn of the pair at 10 deg per Beat without a drop (about 36 consecutive Catches).
- Depends on: P7. Items consumed: JB-109. Plus the three standing Rung preconditions.

### P9: Centre and orbit compile

- Adds: a Site list for Ball 2, aimed Ball 2 Throws across a 77 mm chord, a fixed centre Site for Ball 1.
- Gate (Phase check): (date, `./run_tests.sh --full` plus `sim/skills_gate.py` on the new label plus the ball-to-ball clearance script at radius 100, result) - to be filled.
- Depends on: P8. Items consumed: JB-111.

### P10: JP-14 Centre and orbit

- Adds: asymmetric Site roles with a moving second Site. This is the Destination.
- Rung gate: 30 consecutive Catches in one Attempt at radius 100 mm with 8 Sites per lap, under pose.
- Depends on: P9 and JB-98 (the 70 ms stream STOP cliff ended a 0.45 oval Attempt at Throw 41; a 30-Catch Attempt is exposed to it). Items consumed: JB-111, JB-98. Plus the three standing Rung preconditions.

## 5. Runs in parallel

- The two-ball oval (JB-40) keeps priority and its Sittings; Rungs here ride as the tail of a JB-40 Sitting where they can.
- The Hand thread (JB-114, JB-115): in-contract `max_acc_z` first (Apex about 1.10), then the release step, which needs an owner decision on a named exception to `HAND_LIMIT_C2`, then a one-ball self-throw Apex Ladder under the R0 stop criteria plus a Catch-bounce check. It mainly serves JB-40's Dwell and only optionally JP-9 at 1.05/0.95.
- The human-partner Pathway (JB-113), still to be filed: first a Phase replaying the 2026-09-30 bag through the throw detector, and an owner decision on safety rules for a person inside the Platform's reach before any Rung.

## 6. Not in this plan

- JP-13 (JB-110): moves the Catch plane with the Platform, and the glossary defines the Catch plane as fixed; needs a glossary change first. Promote after the Destination.
- JP-15 (JB-112), JP-12, JP-16: need JB-40's P4 working point.
- JP-17 (JB-113): separate Pathway (a person in the workspace, perception of a human Throw).
- JP-18: not yet probed offline.
- The Hand bumper (JB-114): a safety stop on the Hand thread.
