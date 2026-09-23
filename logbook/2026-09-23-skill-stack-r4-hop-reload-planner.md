---
title: "R4 software — the one-ball hop across two sites, the Ball Butler reload re-cut as a held-axis catch, and two seam defects the probes found on the way"
type: feature
date: 2026-09-23
status: in-progress
phase: "two-ball-skill-stack — R4"
related_plan: two-ball-skill-stack.md
sessions:
  - temp/logs/admissible_sweep_r4_run1_20260923.log
  - temp/logs/skills_gate_reload_20260923.log
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/admissible.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/segments.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_realize.py
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot_interfaces/srv/InstallSegment.srv
  - tools/admissible_sweep.py
  - config/generated/admissible_box.yaml
  - sim/skills_gate.py
  - tests/hardware/skills_plan_bench.py
  - tests/hardware/session_skills_r3.md
  - tests/motion/test_skills_admissible.py
  - tests/motion/test_skills_schedule.py
  - tests/motion/test_skills_executor.py
  - tests/motion/test_skills_segments.py
  - tests/motion/test_cup_cycle.py
  - tests/motion/test_unified_cycle.py
  - tests/motion/test_unified_cycle_splice.py
  - tests/ros/conftest.py
  - tests/ros/test_install_segment.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_skills_plan_bench.py
  - tests/sim/test_skills_gate.py
  - logbook/INDEX.md
subsystem:
  - motion
  - ros
  - sim
tags:
  - skill-stack
  - planner
  - learner
  - reload
---

# R4 software: the hop, the reload, and two seam defects (2026-09-23)

## Summary

R4's build half landed in three layers on one day, each probed before it was written
and each with its owner decision recorded here. **The pattern** (owner decision D1): one
ball hopping between two sites — released at P1 landing at P2, caught at P2 carrying the
throw back toward P1 — which is `compile_self_toss` with the site cycling, so
`OneBallPattern`/`compile_one_ball` replaces it and one site is bit-identical to the retired
compiler. **The box** (D2): `admissible_box.yaml` boxes are keyed by pattern, by (release
site, target site) — the executor's own lookup, which the sweep's columns key had never
matched — and stamped with both sites' xy so a 250 mm box cannot start a 100 mm schedule;
the gate hash now covers six planner files, not two. **The reload** (D4): the FSM's proven
choreography, expressed as skills anchored on the Ball Butler's `ThrowAnnouncement` — one
opening REST that homes the hand AND ends at the receive attitude on the axis through the
announced landing, a held-axis CATCH whose cup opening may only move along that axis, a
DECAY REST back to level from rest, then the ordinary pattern. Two seam defects fell out of
the probes and were fixed at their root rather than in the schedule: the splice join kept a
truncated head's stale commanded velocity at the seam knot (59 k mm/s³ of leg jerk at a
250 mm hop seam), and a REST dispatched exactly two tails after a touch-down still spliced
into the tail's last knot instead of starting fresh from rest.

The reload's physics is the entry's Discussion trigger: my first framing (match the ball's
lateral arrival by translating the platform) was wrong, the owner's pushback was right, and
three probes were needed to find that the planner could not express what the FSM did.

## What the probes established (all 2026-09-23, venv, production `plan_segment` path, limits 300/5000/150 000/3500, apex 0.9 m, dwell 0.30 s)

| probe (scratchpad) | question | result |
|---|---|---|
| `probe_r4_hop.py` | can a cross-site THROW (P1 → P2) and the CATCH-with-throw back plan? | 100 mm: every cell OK (throw jerk 102 743, catch 31 198, dx ±40 OK). 250 mm: throw 125 996, catch 87 530, dx +40 REFUSED, −40/±20 OK. Release cup velocity (117, 0, 4201) mm/s == the ballistic launch: the QP already pins it, no separate aim compensation exists. A tail resting under the ball at P2 is SINGULAR at 250 mm → the abort tail stays at the release site. |
| `probe_r4_hop_schedule.py` | the hop schedule through the REAL install chain (`SkillExecutor` + `install_segment`, perfect tracker) | 100 mm: all 9 skills OK (peak jerk 101 394). 250 mm: every throw and catch OK (125 702), the closing REST refused `LIMIT_JERK 156 495` at the seam (knot 362). |
| `probe_r4_bb_catch.py` | a BB-like catch (18–40° arrival) with the velocity projected onto the clamped 12° receive axis | `LIMIT_VEL` at every angle, standalone and with a throw. |
| `probe_r4_bb_catch2.py` | the same from a PRE-TILTED rest | raw / projected `LIMIT_VEL`, lateral-zeroed `LIMIT_JERK`, a plain THROW from the tilted rest `LIMIT_JERK`. |
| `probe_r4_bb_catch3.py` | banking OFF, tilt HELD | `LIMIT_VEL` 624 mm/s with the lateral zeroed = v_z·sin 12° exactly: the decomposition's `arm·axis_xy` shift. |
| `probe_r4_bb_catch4.py` (U2) | the held-AXIS catch (cup xy slaved to z) from a pre-tilt REST | pre-tilt REST 1.0 s: leg vel 70.8, jerk 10 450 (shortest passing period); held catch at 18/25/30/40°: leg vel **3.1 mm/s**, hand acc 1029–1076; cup at the touch-down knot exactly the landing point; decay REST mirrors; the 0.9 m THROW from the level rest passes (hand 3152). |
| `probe_r4_tilt_accel_jump.py` (U4) | the seam tilt-RATE step (plan § 0 watch item 5) | 0 of 7 seams over the schedule's own bound, max 0.048× — the pin is NOT needed. But the seam knot's COMMANDED velocity was stale: 2.916 rad/s² of tilt acceleration at CATCH k_s = 85 (5th of 142 knots), fixed → 1.422 (20th), the gate's seam window 123 096 → 86 003 mm/s³. |
| `probe_r4_resend_verdicts.py` (U4) | why every 250 mm tracker re-send refuses (watch item 4) | the TAIL, not the seam: `LIMIT_JERK` 338 180 / 503 138 at a 0.45 s lead (+15 mm), `LIMIT_ACC 5 091.7` at +40 mm; within 25 % of the cap at a 0.70 s lead. The lever is an EARLIER re-send. |

## Discussion

### Why this approach — the reload as a held-axis catch

The owner's pushback on my first reload proposal was load-bearing: *"if the Platform is
tilted to 'face' the oncoming ball, the hand only needs to move along its usual linear axis
to match the ball's vertical and lateral velocities. The RELOAD sequence has been working
flawlessly."* The code confirmed it: `planner.build_catch` forced the platform's
translational arrival velocity to zero ("velocity matching is the hand's job"),
`catch_coordinator` pre-tilted during BB's countdown, and real BB arrivals are 18–40° off
vertical against a 12° usable clamp — the residual is in-cup skid the FSM accepted.

The unified planner could express none of the three parts. (1) Its banking schedule returns
a resting cup to level, so a held tilt slews inside the dive — and a 12° slew at the platform
radius is 620–800 mm/s of leg velocity. (2) The cup QP plans the opening's xy independently
of z (jerk boxes 300 vs 6000 m/s³), so on a held tilt the dive's lateral projection
v_z·sin 12° ≈ 620 mm/s must be cancelled by the legs. (3) A REST could not end at a chosen
attitude. The fix states the FSM's one-axis stroke inside the QP: `CatchEvent.axis` slaves
the cup opening's xy to z along the receive axis (on the jerk variables — the per-knot
position form is rank-deficient against the terminal and touch-down rows), one axial
velocity term replaces the three per-axis rows, `CycleGoals.hold_tilt` holds the attitude
with banking off, `CycleGoals.rest_tilt` gives a SETTLE a terminal attitude through a
smoothstep slew with both tilt caps checked, and `segments.hold_axis_site` is the one
derivation of the cup rest ON the axis (29.8 mm from the landing xy at the 12° ceiling, not
under it — the seed and the settle site must lie on the line or the QP refuses `CATCH_AXIS`
naming the xy).

### What was ruled out

* *Translate the platform to match the lateral arrival* — my first option; the legs cannot,
  and the FSM never did.
* *Project the arrival onto the clamped axis* alone — the xy channel cannot ramp the
  stroke's lateral projection during the dive.
* *Pre-tilt the seed* alone — banking returns it to level in-window.
* *Defer the reload to R5* — offered; the owner chose the faithful port.
* *The seam tilt-RATE pin* (watch item 5) — measured unnecessary (seams are C1 at 0.05× the
  bound, structurally: a splice opens inside the unseated dive where C-CUP-1 holds the
  attitude on both sides); the real defect was one layer down, in the join.
* *Fix the stale seam velocity by pinning the tilt rate to the seed's `pose_vel`* — that IS
  the stale quantity; re-deriving it from the joined series is the repair, scoped to a
  truncated head because at a pure `extend` seam the head's velocity is a boundary condition
  (zero at a REST, the throw at a release) and re-deriving there silently repaired a
  caller-injected bad seam the gate exists to refuse (three tests said so).

### Tradeoffs accepted

* The hop's admissible box at 250 mm is narrow (x −20…+6 mm toward P2, apex 0.850–0.900; the
  0.95 apex cell refuses) and late re-aims refuse: at a 0.45 s lead a +15 mm re-send reads
  2.3–3.6× the jerk limit, so catches at 250 mm are effectively open-loop past ~0.5 s before
  touch-down. The owner chose to sweep and fly 250 mm first (D2); the first sitting measures
  whether the plant's scatter fits the box.
* `Schedule.pattern` is REQUIRED (the U1 agent defaulted it): thirteen hand-built test
  schedules now name their pattern, so a hand-built schedule can never borrow another
  pattern's box silently — the same defect class as a box swept for one apex reused at
  another.
* Every REST that follows a catch (closing, decay) is a fresh origin two knots past the
  catch segment's end (`REST_FRESH_MARGIN_S`): the attempt ends 50 ms later, which nothing
  measures, and the seam that refused `LIMIT_JERK` at 250 mm and `CUP_CONTACT_ACC` in the sim
  reload trial no longer exists.
* The held-axis QP costs ~5× a level LANDING per iteration (~0.09 s unloaded); the reload
  CATCH is a fresh origin, so no splice budget is at risk.
* `InstallSegment.srv` gained `hold_tilt_rad` / `rest_tilt_rad` with a NaN sentinel, not
  zeros: a DECAY REST's real (0, 0) target is a distinct fact from "no tilt" (it selects the
  `rest_slew` path).
* The axial-only hand-ratio scaling for a lateral target (`_hand_corrected_landing`) is a
  no-op below the 12° throw-tilt clamp — `tilt_to_throw` aligns the cup axis with the launch
  velocity, so the stroke IS the lateral velocity — and only diverges past saturation, which
  R4 never reaches. Kept as the more general form, proved equal.

### Open questions

* The sim reload trial with the live default `catch_aim_source=tracker` produced a garbage
  early fit on the gate's synthetic 4 s flight (x = 306 078 mm); the trial runs open-loop from
  the announcement. Before the first reload sitting the executor must bound a tracker fit
  for an externally announced ball against the announced landing in TIME as well as
  laterally (the lateral clamp already applies; the time does not).
* The held-axis catch's axial velocity match is ~51 % satisfied at the reference weight —
  a harder pull is a weight decision for the first sitting.

## Fix

Owner decisions D1–D6 and the unit-by-unit build are in the commit messages; the code is
the record. Unit briefs, handoffs and probe scripts are in the session scratchpad and are
summarised above; the probe recipes are repeated in the tests' docstrings.

## Verification

* 2026-09-23, `pytest tests/motion/test_skills_admissible.py -q` → **54 passed in 774.24 s**.
* 2026-09-23, `pytest tests/motion/test_skills_schedule.py tests/motion/test_skills_executor.py tests/motion/test_skills_sites.py -q -x -p no:cacheprovider` → **205 passed in 359.13 s** (before the reload units; schedule alone after them: **96 passed in 1.11 s**).
* 2026-09-23 (U2), `pytest tests/motion/test_cup_cycle.py tests/motion/test_cup_realize.py tests/motion/test_unified_cycle.py tests/motion/test_skills_segments.py tests/motion/test_cup_contact_contract.py tests/motion/test_validate_cycle.py -q` → **322 passed in 7.69 s**.
* 2026-09-23 (U4), `pytest tests/motion/test_unified_cycle_splice.py tests/motion/test_unified_cycle.py -q -p no:randomly -k "not test_the_qp_and_the_gate_stay_within_an_order_of_magnitude"` → **111 passed, 1 deselected in 388.37 s**.
* 2026-09-23 (U3), `pytest tests/ros/test_skill_node.py -q` → **108 passed**; `pytest tests/ros/test_install_segment.py -q` → **22 passed in 4.10 s**.
* 2026-09-23, `python tools/admissible_sweep.py --site-pairs all --single-apex 0.5 0.6 0.7 0.8 0.9 --out config/generated/admissible_box.yaml` → **2159.9 s (36.00 min)**, 15 702 admitted / 769 refused rows, every columns and self-toss box bit-identical to HEAD's bounds, two new hop boxes; `gate_hash` 2fcdec288853 → 7966cb6fadc9.
* Sim reload trial and the per-commit gate: see the Outcome below.

## Outcome

* 2026-09-24 00:24, `python sim/skills_gate.py --reload --seeds 0 1 2 3 4 --no-viewer` under the
  live `catch_aim_source=tracker` default → **PASS 5/5 seeds**, 2 makes / 0 drops / 0 pump rejects
  each, 7–8 installs per seed (tracker re-aims accepted), wall 20.0 s. The first run of the trial
  (2026-09-23 23:57) read `REJECTED_NO_BALL` on all five seeds with 3/3 installs: the synthetic ball
  had been spawned 69.5 m below the MuJoCo floor (one constant for the announcement lead and the
  ball's flight), fixed in the trial only. The `AIM_TRACKER` garbage fit reported earlier was
  measured against that broken flight; the executor's new time band stands on its own.
* 2026-09-24, `./run_tests.sh` (log `temp/logs/r4_gate_wave1b_20260924_*.log`) → **PASS: 6491 passed,
  9 skipped in 244.87 s (parallel phase); serial phase 3 passed in 8.64 s** — the baseline 6361 plus
  this phase's new tests and the peer session's kinematic-calibration tests on the same tree.
* Committed as three commits from this one gated tree state — planner (U2 + U4), admissible box
  (U0 + the regenerated yaml), schedule/executor/node/wire/sim (U1 + U3) — each carrying the
  `Logbook-Entry` trailer; the two intermediate trees carry a box `gate_hash` one commit apart from
  the planner they were swept against, which `admissible.check_limits` would refuse at a start and
  which no test on those trees asserts.
* Still to come in R4: `Juggle.action` + the GUI relay (U5), the FSM deletion under `fsm-final`
  (U6, census in hand), the hop sim gate at 20 cycles × 5 seeds, the determinism re-sweep,
  `./run_tests.sh --full`, the runsheet and the plan's R4 row.
