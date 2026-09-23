---
title: Cup-contact contract — banking defined only under seating force, the cup never falls away from a ball, lateral aim unpinned
created: 2026-09-18
status: completed
completed: 2026-09-23
archived: 2026-09-23
owner: harrison
last_updated: 2026-09-23
related_plan: two-ball-skill-stack.md
related_logbook:
  - 2026-09-16-banking-saturates-on-small-lateral-offsets.md   # the root cause: tilt_to_receive has no solution in the dive, clamps to 12°, azimuth is scale-free
  - 2026-09-17-late-catches-are-a-late-tracker.md              # the contact-phase cliff: cup run-up > g, HELD/EMPTY/HELD
  - 2026-09-18-learn-the-apex-aim-from-the-tracker.md          # the paper's a_throw = g release constraint; the learner's lateral channel
  - 2026-09-18-tracker-aim-carried-the-lateral-bias-park-race-splice-budget.md   # 52 lateral re-sends refused LIMIT_VEL/ACC/JERK; the lateral clamp
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_realize.py::tilt_schedule (the banking prescription, `_banking_raw`; the widen loop, `_accel_bounded_schedule`)
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/tilt_geometry.py::tilt_to_receive (the 12° clamp and the normalised azimuth)
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py::plan_window (the QP; catch velocity match z_ratio 0.7, catch runway)
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py::validate_cycle (the K1-K6 gates; the one enforcement point for the new gate)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py::_clamp_lateral_to_schedule, lateral_authority_m (what this contract unpins)
---

# Cup-contact contract

> **Status 2026-09-23 (evening): CLOSED OUT — both § 5 criteria met on hardware.** Block A
> 2026-09-22: τ = 0.125 s holds (87/87 caught, seat median +0.089/+0.079 s, hop and late-seat
> rates no worse than the 09-18 control). Block B 2026-09-23: the lateral learner pulled the
> median y miss through zero at 0.6 and 0.9 m; 40 mm authority dropped balls on the 0.9 m
> 25-throw chains, 20 mm was stable → launch default 20 mm (§ 6 amended). The § 1 frame check
> refused the first Block B on a (−1.56, −8.53) mm offset that the § 9 z-sweep proved to be a
> 0.77° lever arm between the QTM frame and the base plane; the measured offset is now
> subtracted from every tracker landing (§ 1 amended) and the base should be shimmed level.
> Carried out of this plan into R4's § 0: authority 20 mm, τ = 0.125 s, the frame subtraction,
> and three watch items — the hand recovery park's unresolved upper-stop push (2026-09-23), the
> stale-hand-encoder latches at throw onset (leg-bus frame drops), and re-sends seeded inside
> the contact window (`CUP_CONTACT_ACC`). Entries: `2026-09-20-cup-contact-contract-implemented`,
> `2026-09-21-two-sided-knot-sampling`, `2026-09-23-cup-contact-first-sitting`,
> `2026-09-23-block-b-lateral-learner-and-hand-endstop-push`. § 6 is decided, τ moved 0.060 →
> 0.125 s on a measurement (§ 7), `contact_knots` is a tuple of ranges. Design owner-approved
> 2026-09-18.


## Archival note (2026-09-23)

Shipped: the three normative clauses (C-CUP-1 banking only under seating force, C-CUP-2 the
`CUP_CONTACT_ACC` dive floor over a τ = 0.125 s contact window, C-CUP-3 jerk-aware widening),
their gates and tests, the two-sided knot sampling, the session-start frame check with the
mocap-to-schedule subtraction, and lateral authority unpinned to a 20 mm launch default.
Flown: Block A 2026-09-22 (τ holds, 87/87), the § 9 z-sweep and Block B 2026-09-23 (the lateral
learner works). Closed because both § 5 criteria were met on hardware (owner, 2026-09-23).
Residue re-homed to `plans/active/two-ball-skill-stack.md` § 0 ("Carried in from"): the
frame-offset lever arm and the base-shim remedy, the hand recovery park's unresolved upper-stop
push, the stale-encoder latches, contact-window re-sends, the `LIMIT_JERK` re-send rate, the
seam tilt-rate pin and the QP/gate wall-clock bar. Entries: 2026-09-20, 2026-09-21,
2026-09-23 ×2 (see the status block). Runsheet `tests/hardware/session_cup_contact.md` kept
as the flown record. **Two path references were deliberately NOT swept:** the comments
in `motion/trajectory/feasibility.py` and `motion/skills/segments.py` still say
`plans/active/cup-contact-contract.md`, because `admissible.gate_hash()` hashes those two
files' full text and a comment edit would demand the 29-minute box re-sweep (the first
archival attempt did exactly that and failed 18 start-path tests on a stale-box refusal).
Re-point them in the next commit that re-sweeps the box for a real reason.

## 0. Why — the one root cause behind four symptoms

The cup's banking schedule prescribes, at every knot, the attitude that seats a ball
under the *apparent* gravity in the cup, `f = g − a_cup`
(`cup_realize.tilt_schedule`, `tilt_geometry.tilt_to_receive`). In the 125 ms before a
catch the cup dives at 1.2–2.8 g, so `f_z > 0`: apparent gravity points UP, no attitude
can seat the ball, and the prescription has no solution. `tilt_to_receive` then clamps
the angle to `max_tilt_deg` = 12° and takes the azimuth from the **normalised**
lateral residual, which is scale-free — a 0.4 milli-g lateral residual and a 60 milli-g
one demand the same full-scale tilt. Measured 2026-09-16: `raw_max = 12.000°` for every
lateral offset from 0.5 mm to 40 mm, `0.000°` at exactly 0. The tilt smoother's widen
loop then exits on tilt acceleration only and picks a discrete branch by 0.001 rad/s²
(137 k vs 50 k mm/s³ of leg jerk), so the jerk is flat in the offset and above the
150 k limit from 4 mm upward.

That single mechanism is behind:

1. **The 0.9 m "wobble"** (2026-09-16): a 3.9 mm learner aim correction commanded 2.24°
   of tilt and 13.5 mm of platform excursion in the catch window.
2. **The learner's lateral channel is pinned** (`learner_lateral_authority_mm = 0`,
   2026-09-16) and the tracker aim is clamped to the schedule's site
   (`_clamp_lateral_to_schedule`, 2026-09-18): the platform cannot be allowed to
   move laterally in the dive, so nothing may ask it to.
3. **Timing-only re-aims refuse `LIMIT_JERK`** (18 of 21 on 2026-09-18 16:16): the dive
   is already at the jerk ceiling with zero lateral offset, so a 50 ms re-phase tips it.
4. **The contact cliff** (2026-09-17): after touch-down the cup accelerates DOWNWARD at
   ~15 m/s² (> g) in its run-up, a ball that arrived early loses contact
   (HELD/EMPTY/HELD), and the catch tolerance is ±20 ms instead of ±100. The paper
   constrains the mirror case at release (`a_throw = g`, "reducing additional contact
   forces"); we constrain neither side.

And it is on R4's critical path: two sites 250 mm apart means every throw is a lateral
aim, and every catch a lateral arrival.

## 1. Prerequisite — measure in the frame you command in (owner, in progress)

The learner's lateral outcome is the fitted landing minus the target site, both in the
mocap frame. On 2026-09-18 16:16 the mocap `Platform` body sat **+30 mm in y** of the
commanded platform position with the `Base` body aligned to 1 mm; the owner reproduced
the cause the same evening — re-aligning QTM to the base a few times moved the reported
platform between +30 and −50 mm, i.e. a small base-alignment error amplified over the
base-to-platform lever. The owner is adding a base marker. **Until the platform-vs-command
offset is below 5 mm at session start, lateral authority stays pinned** — otherwise the
learner spends its first throws correcting an offset that is not a miss. The
implementing session adds a session-start check (runsheet § 1 and a `skill_node`
precondition reading `/rigid_body_poses` `Platform` against `/trajectory/commanded_position`
over 1 s: refuse to unpin above 5 mm, with the number). Real flight drift, after removing
the offset: median +31 mm in y, scatter −34…+91 mm (60 throws, 2026-09-18) — the learner
can remove the median; the scatter is per-throw.

**Amended 2026-09-23 (first sitting, `logbook/2026-09-23-cup-contact-first-sitting.md`):**
the residual offset after a clean base alignment is (−1.56, −8.53) mm, stable to 0.03 mm and
invariant under relocating the base — 574.3 mm × the levelling pose offset (0.015, 0.002) rad,
a lever arm between the QTM Base-body frame and the machine's base plane, which no alignment
precision removes. Measuring in the frame you command in therefore means SUBTRACTING the
measured offset: `skill_node._on_balls` now takes it off every tracker landing's xy, so the
learner outcome and the catch aim are relative to the cup's real position at any authority.
The check keeps a 25 mm sanity bound (a wrong alignment, the +30/−50 mm cases above) and a
2 mm Platform-body stability requirement; an offset under the bound is adopted, not refused.
The runsheet's § 9 z-sweep discriminates the mechanism (a lever arm scales ≈ 1.4 mm per
100 mm of commanded height) before Block B flies. At zero lateral command the 2026-09-22 real
misses were about (+4, −5) mm at 0.6 m and (+5.5, −9) mm at 0.9 m, scatter sd 10 / 17 mm.

## 2. The contract (normative)

**C-CUP-1 — banking is defined only under seating force.** The banking prescription is
evaluated only where the apparent gravity in the cup has a seating component,
`s = g + a_cup,z ≥ ε·g` with `ε = 0.2` (world z up; `s` is the magnitude of
`(g − a_cup)_z` pressing a ball INTO the cup — `g` at rest, 0 in free fall, negative in
the > g dive of § 0, where that section's "`f_z > 0`" is the same fact in the signed
vector convention. Wording made sign-explicit 2026-09-18 at implementation; the meaning
is unchanged). Elsewhere the schedule **carries the last
valid attitude** (a hold, later smoothed by the existing blend), never a saturated one.
Consequence, which is the test: the tilt demand is amplitude-aware — it tends to zero as
the lateral residual tends to zero at any fixed dive (with `f_z ≥ εg` the angle is
`atan(|f_lat| / f_z) ≤ atan(|f_lat| / εg)`, so a milli-g residual is a sub-degree tilt).
The 12° clamp stays as a hard cap that a defined prescription never reaches.

**C-CUP-2 — the cup never falls away from a ball.** Whenever a ball is or may be in the
cup, the cup's vertical acceleration satisfies `a_cup,z ≥ −κ·g`, `κ = 0.7`. "May be" is
the contact window: from `τ = 0.125 s` before the planned touch-down (owner decision
2026-09-20, § 6 — the design's 0.060 s was the measured arrival scatter of 2026-09-17/18;
see § 7 for why it grew) to the knot BEFORE the release knot for a catch-and-throw, or to rest for
a standalone catch and for the opening/closing REST while a ball is held. Upward
acceleration (the cup decelerating a falling ball) is unbounded by this contract — it is
the hand runway's business (`catch_runway_requirement`). At the release instant itself
the paper's `a_throw = g` is the limiting case and is already how the release is planned.

**C-CUP-3 — jerk-aware widening.** The tilt smoother's widen loop exits only when the
third difference of the schedule (the quantity `LIMIT_JERK` refuses on) is inside the
budget as well as the second, so the achieved leg jerk is continuous in the commanded
offset instead of jumping between discrete branches.

## 3. Enforcement — one point per clause

| Clause | Where | Shape |
|---|---|---|
| C-CUP-1 | `cup_realize.tilt_schedule`, the `banking_enabled` branch (`_banking_raw`) | one `if f_z >= eps*g: prescribe else: carry` per knot; `tilt_geometry.tilt_to_receive` unchanged in its geometry but never called with an unseatable field |
| C-CUP-2 (feasible by construction) | `cup_cycle.plan_window` | acceleration-row inequality `Aa[k]·x + ca[k] ≥ −κg` for k in the contact window, alongside the existing box rows (the QP already carries `Aa`); `contact_knots` is a tuple of `(k0, k1)` knot-index ranges (`cycle_plan._checked_contact_knots`), not a single pair — a chained catch-and-throw plan carries two windows |
| C-CUP-2 (the gate) | `feasibility.validate_cycle` | new named refusal `CUP_CONTACT_ACC` next to `LIMIT_ACC`/`LIMIT_JERK`, computed from the plan's cup acceleration over the window; reported with every other refusal (the "report every refusal at once" rule) |
| C-CUP-3 | `cup_realize._accel_bounded_schedule`'s widen loop, `_tilt_leg_jerk_mmps3` | the exit test is the MEASURED per-knot composite leg jerk (`_tilt_leg_jerk_mmps3`, the discrete-Leibniz expansion of the tilt channel's contribution — arm term + platform-rotation term + the cup's own v_z/a_z/j_z cross terms), not a static lever: owner decision 2026-09-20 (§ 6) after the fitted 2.1× lever constant measured 1.15–2.4× loose on R3 fixtures and 6.3× too SMALL on the `test_unified_cycle` STEADY ring. The loop searches floor-first then bisects with real-valued (non-`ceil`) widths onto the cap so the schedule sits AT the budget, not under it by a whole search step. `_tilt_jerk_lever_mm`'s analytic estimate survives only to size how many widen attempts the bisection needs, and as the fallback for a duck-typed plan with no `pos`/`vel`/`jerk`. |

Constants `ε`, `κ`, `τ` live in `config/hardware_config.yaml` (trajectory section) →
generated; one definition each. `gate_hash` (sha256 of `feasibility.py` + `segments.py`)
CHANGES: every admissible box is re-swept with `tools/admissible_sweep.py` at the launch
limits (300/5000/150000) and written in apex units — the sweep now samples 0–10 mm
lateral offsets explicitly (the 2026-09-16 hole); at the densified grid the sweep takes
~29 min (1745.0 s measured 2026-09-21 — see § 7's box table), up from "well under 5 min"
at the old grid.

## 4. Tests that fail today (write first)

1. **Amplitude invariance**: at the R3 operating point, plan the catch-and-throw for
   lateral offsets 0, 0.5, 1, 2, 4, 8, 16, 31 mm; assert the peak commanded tilt is
   monotone in the offset and below 1° at 4 mm (today: 12° at every nonzero offset).
2. **The 2026-09-16 reproduction**: dx = 3.9 mm at 0.9 m → leg jerk < 150 k and
   monotone in dx across the grid (today 137 k at 3.9, 86 k at 31, non-monotone).
3. **Contact bound**: over the contact window of every segment kind, `min(a_cup,z) ≥
   −κg`; a synthetic plan that dips below is refused `CUP_CONTACT_ACC` by
   `validate_cycle` and the refusal names the knot and the value.
4. **Sim gate with authority**: `sim/skills_gate.py --learn --policy A --seeds 0-4` with
   `learner_lateral_authority_mm = 40` enters the xy band (the R3 item (k) xfail flips
   to pass) with 0 drops.
5. **Executor**: the tracker aim's lateral clamp and the learner's lateral clamp both
   follow the one `lateral_authority_m`; with 40 mm authority a fitted landing 85 mm off
   moves the catch 40 mm, not 85.

## 5. Unpinning, in order

1. Tests 1–3 green; boxes re-swept; full gate.
2. Test 4 green in the sim.
3. Session-start frame check (§ 1) green on the loaded Jetson.
4. `learner_lateral_authority_mm` launch default 0 → 40 (the box's lateral range);
   the executor's tracker-aim clamp inherits it. One sitting at 0.6 m and 0.9 m: the
   contact phase `seat=` should stay in +0.05…+0.15 s and the median lateral miss go from
   +31 mm toward 0 within five throws.

## 6. Owner decisions

**Decided 2026-09-20 (owner, at implementation):**

- `κ = 0.7` (a ball keeps 0.3 g of seating force through the window), `ε = 0.2`, authority
  40 mm — CONFIRMED as proposed. **Amended 2026-09-23 (owner, after Block B): launch default
  20 mm.** At 40 mm the 0.9 m 25-throw chains dropped balls (the learner's y command reached
  +29 mm against a 17.6 mm plant scatter; 2 drops in 51); 20 mm was stable at 0.6 and 0.9 m.
  The box still admits ±40 mm; the parameter is the executor's clamp.
- The contact window of a catch-and-throw closes on the knot BEFORE the release knot —
  CONFIRMED. (The release equality pins `a = −g` at the release knot itself, so the bound
  cannot include it; measured, the run-up only reaches −0.32 g, so the alternative — ending
  at the hand's release-stroke start — would pass too and would leave the stroke ungated.)
- `τ`: **0.060 → 0.125 s, "lengthen τ only"** — chosen over a two-tier approach floor, a
  lower velocity-match weight, and accepting the design as drawn. See § 7 for the
  measurement that forced the question.
- C-CUP-3's jerk cap: **derive the lever analytically** rather than keep a fitted factor
  (§ 3's "the same map the accel cap uses" is wrong by ~2.1× for a third difference: the
  cup's own vertical speed adds product-rule terms the static lever has no counterpart for).
  **Shipped form (2026-09-20, at implementation): the analytic derivation alone is not a
  bound** — measured 1.15–2.4× loose on R3 fixtures but 4.45× UNDER on the
  `test_unified_cycle` STEADY ring (its composite is carried by `|j_z|·|θ|`, a term the
  knot-scale closure under-reads ~70× there) — so the widen loop's exit test uses the
  MEASURED per-knot composite (`_tilt_leg_jerk_mmps3`) directly; the analytic lever
  survives only as the estimate that sizes the first widen attempt. See § 3.

**Decided 2026-09-21 (owner, at the unpinning):**

- `learner_lateral_authority_mm` launch default 0 → 40 — CONFIRMED as the plan wrote it.
  The owner was shown the re-swept boxes' lateral ranges (§ 7 table) before deciding: the
  0.6 m single-site box admits only ±30 mm in y (gate-limited, `LIMIT_JERK`/`MARGIN`), not
  the full ±40 mm the other apex rungs clear — the admissible box clips any out-of-band
  throw command regardless of the executor's lateral-authority clamp, so a uniform 40 mm
  authority does not let a command past what the box (a separate, orthogonal check) would
  refuse at 0.6 m.
- `τ = 0.125 s` — CONFIRMED after the box re-sweep found 0 `CUP_CONTACT_ACC` refusals in
  14 731 rows (§ 7 table): the value chosen on the 2026-09-20 measurement holds at the
  launch limits with the densified grid, so no further change to τ was needed.

## 7. Cost and risk

**Measured 2026-09-20 — the prediction below ("re-phase earlier and coast") was WRONG, and
the cost of C-CUP-2 is the touch-down velocity match.** The cup QP is stroke-limited: coasting
at the match speed through the window costs travel the hand does not have above the
touch-down height, so with the floor on, the QP sits AT the floor through the whole window and
buys its dive in the ONE knot before the window opens. At 0.9 m (pre-contract: peak dive
−34.8 k mm/s², touch-down `v_z` −2.74 m/s against a 4.20 m/s ball):

| τ (s) | pre-window knot `a_z` (mm/s²) | touch-down `v_z` (m/s) |
|---|---:|---:|
| 0.060 | −61 900 (−6.3 g) | −1.90 |
| 0.100 | −43 400 | −1.67 |
| **0.125** | **−29 600** | **−1.54** |
| 0.150 | −21 700 | −1.47 |
| 0.200 | −11 300 | −1.44 |

`τ = 0.125 s` is the smallest knot multiple whose pre-window dive is no harsher than the dive
flown before the contract. The ball now meets the cup ~1.2 m/s faster in relative speed
(2.66 vs 1.46 m/s). The evidence that this is the acceptable side of the trade: the operator's
"smoothest" catches of 2026-09-17/18 (`seat=` +0.10 s) met a cup moving only 0.3 m/s — what
made a catch bad was the cup falling AWAY, not the closing speed. That is a sitting-level
hypothesis, not a result: read `seat=` and the bounce rate at the first sitting and revisit τ
(or the two-tier floor) if catches bounce.

- Re-phasing the dive (the cup at its matching velocity `τ` earlier and coasting) shifts
  the contact phase the 2026-09-17 metric was calibrated on; expect `seat=` to move
  toward +0.05 s. Re-read the smooth/bounce bands after the first sitting.
- Every box is re-swept; the 0.9 m box may shrink laterally if C-CUP-2 binds at the
  session leg limits — report the new boxes' lateral ranges before unpinning.
- `gate_hash` changes invalidate the three box YAMLs on disk until the sweep runs.

**Re-swept 2026-09-21** (`tools/admissible_sweep.py --site-pairs both --single-apex 0.5
0.6 0.7 0.8 0.9`, 300/5000/150000, hand 3500; deterministic across two runs; 0
`CUP_CONTACT_ACC` refusals in 14 731 rows — see the logbook entry for the run
provenance):

| box | old (2026-09-16, `gate_hash 37d68192b0e1`) xy mm | new xy mm | apex m |
|---|---|---|---|
| single 0.9 m | x[0,40] y[0,0] | ±40 × ±40 (grid edge) | 0.576–0.900 (unchanged) |
| single 0.8 m | (0,0) | ±40 × ±40 | 0.512–0.882 |
| single 0.7 m | (0,0) | ±40 × ±40 | 0.448–0.847 |
| single 0.6 m | x[0,10] y[0,0] | ±40 × ±30 (y gate-limited: `LIMIT_JERK`/`MARGIN`) | 0.384–0.726 |
| single 0.5 m | (0,0) | ±20 × ±10 | 0.320–0.605 |
| columns P1→P2 | x[−20,10] y[−20,20] | ±20 × ±20 | 0.850–0.900 → **0.900 only** |
| columns P2→P1 | x[−20,20] y[−10,20] | ±20 × ±20 | 0.850–0.900 → **0.900 only** |

**Carried to R4**: the columns apex-band collapse to a single 0.900 m point (both edge
apexes, 0.850/0.950 m, now fail `THROW:MARGIN` at zero offset) — a real narrowing, not a
grid artefact.

**CLOSED 2026-09-21**: two-sided knot sampling in `validate_cycle` (Diagnosis #5 Red 1
above) — see `logbook/2026-09-21-two-sided-knot-sampling.md`.
