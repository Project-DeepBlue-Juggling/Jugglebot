---
id: JB-116
kind: pathway
title: 'Pattern progression: Columns to Centre and orbit'
status: active
created: '2026-10-30'
updated: '2026-10-30T11:31:18Z'
owners: []
priority: medium
effort: xl
hardware: robot
owner: hands
radius: cross-cutting
area: skill-stack
blocked_by: []
links:
- idea-jp-10-turned-columns-the-columns-pair-at-fixed-angles-about
- idea-jp-9-high-low-columns-ball-1-at-apex-1-05-m-ball-2-at-0-95-m
- bug-launch-throw-with-the-ball-aboard-may-demand-more-than-the
- idea-jp-5-polygon-walk-one-ball-n-sites-the-cheapest-way-to-land
- idea-jp-11-rotating-columns-the-pair-turns-a-few-degrees-every
- idea-jp-14-centre-and-orbit-ball-1-in-a-column-ball-2-on-a-circle
- pathway-learner-generalisation
- pathway-two-ball-oval
- bug-teensy-guard-max-deviation-latched-on-the-hand-8-4-rev-mid
- idea-hand-end-of-stroke-bumper-and-first-the-planner-s-own-brake
destination: 'JP-14 Centre and orbit flown on the robot: 30 consecutive Catches in one Attempt at radius 100 mm with 8 Sites per lap, under pose.'
branch: main
---

Catalogue: `docs/patterns.md` (JP-1 to JP-18). This Pathway takes two-ball Columns to the first moving two-ball show Pattern, one new capability per step.

## Ordering principle

Every Pattern here flies at today's Apex (0.93 to 1.05) under the pose planner, so the Apex lift is not on the critical path: it runs in parallel as Hand work (JB-114). The shared need is Sites that change Beat by Beat, and the owner already placed that machinery in JB-38 (decisions D4 and D5: the `BeatEvent` stream, online admissibility, the cold-start Learner). So fixed-Site Patterns fly first on today's compiled path, and moving-Site Patterns follow JB-38, smallest increment first.

## Steps

Precondition (owned by JB-40, not a step here): the Columns check Sitting under pose, runbook § 11 of `tests/hardware/session_oval_rung1.md`.

Every Rung below needs its runbook section dress-rehearsed on the loaded Jetson and `./run_tests.sh --full` green before its Sitting, and its Pattern needs its own machine label in `records.py` before its first Attempt.

1. **Phase: launch current check (JB-115).** Adds: knowing whether the launch Throw with the ball aboard already exceeds 50 A from Apex about 0.99. Check: (date, `tools/probes/hand_stroke_timeline.py` on the 2026-10-28 14:33 Columns bag, result: peak hand iq at the launch knot against commanded Apex per Throw). Depends on nothing. Owner decides after it which JP-9 pair flies first.

2. **Phase: turned Columns on the compiled path (JB-108).** Adds: a rotation parameter on the Columns Site pair, two Admissible box cells (45 and 90 deg), a new machine label. Check: (date, `./run_tests.sh --full` plus the box re-swept twice bit-identical plus `sim/skills_gate.py` on the new label, result). Owner decides first: reaffirm or reopen JB-38 D4/D5 for these Patterns (it decides whether this Phase builds plain cells or a relative key), then the label and angles.

3. **Rung: JP-10 turned Columns (JB-108).** Adds: two-ball Columns off the x axis. Rung gate: at 45 and at 90 deg, 20 consecutive Catches in one Attempt, landing scatter within 3 mm of a straight Columns control flown the same Sitting. Each angle gets its own `plant_id`, so the rows answer JB-38 P2(b) (does the lateral bias rotate with the Pattern). May share a Sitting with step 5.

4. **Phase: per-Site Apex (JB-107).** Adds: one Apex per Site in the Columns compile and a two-band box cell, new label. Check: (date, `./run_tests.sh --full` plus the box sweep plus `sim/skills_gate.py`, result). Depends on step 1. If step 1 shows the launch clipping at 1.05, the cells are swept at the highest pair step 1 shows clean, and 1.05/0.95 waits for JB-114.

5. **Rung: JP-9 high-low Columns (JB-107).** Adds: two Apexes in one Pattern. Rung gate: 20 consecutive Catches in one Attempt; Tracker mean Apex difference within 20 mm of the commanded difference; hand iq under 45 A at every launch. Depends on step 4.

6. **Dependency, owned by JB-38: Learner core P1, online admissibility P3, `BeatEvent` stream P4.** Not built here. This Pathway waits on it and adds no box key of its own. Owner decides at JB-38's P2 whether the JP-10 rows stand in for its rotated-pair flight.

7. **Rung: JP-5 polygon walk (JB-106).** Adds: the first Pattern whose Site changes every Throw, on `BeatEvent`, one ball. Rung gate: a square whose edge the Hop already flies, 24 consecutive Catches in one Attempt (six laps), no miss on a first visit to a Site. Depends on step 6.

8. **Rung: JP-11 rotating Columns (JB-109).** Adds: two-ball per-Beat Sites, the smallest two-ball move (11 mm per Beat at 5 deg, inside the Learner's 20 mm lateral clamp). A Ladder of 5, 10 and 20 deg per Beat in one Sitting. Rung gate: one full turn of the pair at 10 deg per Beat without a drop (about 36 consecutive Catches). Depends on step 7.

9. **Phase: Centre and orbit compile (JB-111).** Adds: a Site list for Ball 2, aimed Ball 2 Throws across a 77 mm chord, a fixed centre Site for Ball 1. Check: (date, `./run_tests.sh --full` plus `sim/skills_gate.py` on the new label plus the ball-to-ball clearance script at radius 100, result). Depends on step 8.

10. **Rung: JP-14 Centre and orbit (JB-111), the Destination.** Adds: asymmetric Site roles with a moving second Site. Rung gate: the Destination line. Depends on step 9 and on JB-98 (the 70 ms stream STOP cliff ended a 0.45 oval Attempt at Throw 41; a 30-Catch Attempt is exposed to it).

## Runs in parallel

- The two-ball oval (JB-40) keeps priority and its Sittings; Rungs here ride as the tail of a JB-40 Sitting where they can.
- The Apex lift (JB-114, JB-115) is Hand work: in-contract `max_acc_z` first (Apex about 1.10), then the release step, which needs an owner decision on a named exception to `HAND_LIMIT_C2`, then a one-ball self-throw Apex Ladder under the R0 stop criteria plus a Catch-bounce check. It mainly serves JB-40's Dwell and only optionally this Pathway (JP-9 at 1.05/0.95).
- The human-partner thread (JB-113) is its own Pathway: first a Phase replaying the 2026-09-30 bag through the throw detector, and an owner decision on safety rules for a person inside the Platform's reach before any Rung.

## Not on this Pathway

JP-13 (JB-110): it moves the Catch plane with the Platform, and the glossary defines the Catch plane as fixed, so it needs a glossary change first; promote after the Destination. JP-15 (JB-112), JP-12 and JP-16: need JB-40's P4 working point. JP-17 (JB-113): separate Pathway. JP-18: not yet probed offline. The bumper (JB-114): a safety stop on the Hand thread.
