---
title: Cup-contact contract — banking defined only under seating force, the cup never falls away from a ball, lateral aim unpinned
created: 2026-09-18
status: active
owner: harrison
last_updated: 2026-09-18
related_plan: two-ball-skill-stack.md
related_logbook:
  - 2026-09-16-banking-saturates-on-small-lateral-offsets.md   # the root cause: tilt_to_receive has no solution in the dive, clamps to 12°, azimuth is scale-free
  - 2026-09-17-late-catches-are-a-late-tracker.md              # the contact-phase cliff: cup run-up > g, HELD/EMPTY/HELD
  - 2026-09-18-learn-the-apex-aim-from-the-tracker.md          # the paper's a_throw = g release constraint; the learner's lateral channel
  - 2026-09-18-tracker-aim-carried-the-lateral-bias-park-race-splice-budget.md   # 52 lateral re-sends refused LIMIT_VEL/ACC/JERK; the lateral clamp
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_realize.py::realize_tilt (the banking prescription, ~L611; the widen loop, ~L455-500)
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/tilt_geometry.py::tilt_to_receive (the 12° clamp and the normalised azimuth)
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py::plan_window (the QP; catch velocity match z_ratio 0.7, catch runway)
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py::validate_cycle (the K1-K6 gates; the one enforcement point for the new gate)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py::_clamp_lateral_to_schedule, lateral_authority_m (what this contract unpins)
---

# Cup-contact contract

> **Status 2026-09-18: DESIGN, owner-approved in principle ("agreed" on the ordering
> contract → R4 → frame drops). Implementation not started. This document is the
> brief for the implementing session; read it whole before touching code.**

## 0. Why — the one root cause behind four symptoms

The cup's banking schedule prescribes, at every knot, the attitude that seats a ball
under the *apparent* gravity in the cup, `f = g − a_cup`
(`cup_realize.py` ~L611, `tilt_geometry.tilt_to_receive`). In the 125 ms before a
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

## 2. The contract (normative)

**C-CUP-1 — banking is defined only under seating force.** The banking prescription is
evaluated only where the apparent gravity in the cup has a seating component,
`f_z = (g − a_cup)_z ≥ ε·g` with `ε = 0.2`. Elsewhere the schedule **carries the last
valid attitude** (a hold, later smoothed by the existing blend), never a saturated one.
Consequence, which is the test: the tilt demand is amplitude-aware — it tends to zero as
the lateral residual tends to zero at any fixed dive (with `f_z ≥ εg` the angle is
`atan(|f_lat| / f_z) ≤ atan(|f_lat| / εg)`, so a milli-g residual is a sub-degree tilt).
The 12° clamp stays as a hard cap that a defined prescription never reaches.

**C-CUP-2 — the cup never falls away from a ball.** Whenever a ball is or may be in the
cup, the cup's vertical acceleration satisfies `a_cup,z ≥ −κ·g`, `κ = 0.7`. "May be" is
the contact window: from `τ = 0.060 s` before the planned touch-down (the measured
arrival scatter, 2026-09-17/18) to the release knot for a catch-and-throw, or to rest for
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
| C-CUP-1 | `cup_realize.realize_tilt`, the `banking_enabled` branch (~L611) | one `if f_z >= eps*g: prescribe else: carry` per knot; `tilt_geometry.tilt_to_receive` unchanged in its geometry but never called with an unseatable field |
| C-CUP-2 (feasible by construction) | `cup_cycle.plan_window` | acceleration-row inequality `Aa[k]·x + ca[k] ≥ −κg` for k in the contact window, alongside the existing box rows (the QP already carries `Aa`) |
| C-CUP-2 (the gate) | `feasibility.validate_cycle` | new named refusal `CUP_CONTACT_ACC` next to `LIMIT_ACC`/`LIMIT_JERK`, computed from the plan's cup acceleration over the window; reported with every other refusal (the "report every refusal at once" rule) |
| C-CUP-3 | `cup_realize` widen loop (~L455-500) | `_max_tilt_jerk(out, dt) <= jerk_cap` added to the exit test; `jerk_cap` derived from the session leg-jerk limit through the same map the accel cap uses |

Constants `ε`, `κ`, `τ` live in `config/hardware_config.yaml` (trajectory section) →
generated; one definition each. `gate_hash` (sha256 of `feasibility.py` + `segments.py`)
CHANGES: every admissible box is re-swept with `tools/admissible_sweep.py` at the launch
limits (300/5000/150000) and written in apex units — the sweep now samples 0–10 mm
lateral offsets explicitly (the 2026-09-16 hole).

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

- `κ = 0.7` (a ball keeps 0.3 g of seating force through the window), `τ = 0.060 s`,
  `ε = 0.2`, authority 40 mm — proposed, confirm or move.
- Whether the contact window of a catch-and-throw ends at the release knot (proposed) or
  at the hand's release-stroke start.

## 7. Cost and risk

- Re-phasing the dive (the cup at its matching velocity `τ` earlier and coasting) shifts
  the contact phase the 2026-09-17 metric was calibrated on; expect `seat=` to move
  toward +0.05 s. Re-read the smooth/bounce bands after the first sitting.
- Every box is re-swept; the 0.9 m box may shrink laterally if C-CUP-2 binds at the
  session leg limits — report the new boxes' lateral ranges before unpinning.
- `gate_hash` changes invalidate the three box YAMLs on disk until the sweep runs.
