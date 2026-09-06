---
title: "First UH-3 attempt — refused three deep on one physical fact, then E-STOPped by its own solve; four defects fixed, the rung still not flown"
type: investigation
date: 2026-09-06
status: in-progress
phase: "unified-7dof-planner — Phase 5 (first UH-3 attempt)"
related_plan: unified-7dof-planner.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/hand_stroke.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/blas_threads.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator_node.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - tests/hardware/unified_cycle_bench.py
  - tests/hardware/session_unified7_cycle_ladder.md
  - tests/motion/test_blas_threads.py
  - tests/motion/test_unified_cycle.py
  - tests/ros/test_launch_nodes.py
  - tests/ros/test_unified_cycle_integration.py
  - tools/probes/emitter_gap_under_solve.py
  - plans/active/unified-7dof-planner.md
  - plans/active/INDEX.md
  - logbook/2026-09-06-uh3-first-attempt-refusals-and-estop.md
  - logbook/INDEX.md
subsystem:
  - motion
  - ros
  - can
tags:
  - safety
  - testing
  - performance
---

# First UH-3 attempt — three refusals stacked on one fact, and an E-STOP the watchdog was right to fire

## Summary

The first attempt at UH-3 (carry a seated ball) ran 12:05–12:09 on 2026-09-06. **The ball never
moved.** Six invocations of `unified_cycle_bench.py --rung carry`, six aborts: a precondition
refusal, two `SETTLE_SITE`, one `HAND_STROKE`, one `GUARD_LATCHED`, one more `HAND_STROKE`. The
can-bridge guard E-STOPped **twice**, both times while the planner was solving.

Three of those refusals are the **same physical fact** seen through three layers: *a homed hand
parks BELOW its own encoder zero, by design* (`HOMING_HAND_ABS_POS_REV = -0.1`; measured on the
day, −0.038 rev). The planner's cup box, the stroke gate and the driver's site arithmetic each
assumed a non-negative hand, so each refused in turn as the one above it was cleared. A fourth
defect waited behind them and would have moved the robot: the same parked seed, 11.2 mm under the
QP's cup z box, makes a *flat* 1.4 s carry fire the cup to the box ceiling — **984.6 mm, ~295 mm of
slider at 78 rev/s, with a ball in the cup** — and every gate in the stack accepts it.

A **fifth** defect fell out of the follow-up measurement itself: a window planned from a *moving*
seed was handed free fall as its boundary condition — the same lie fixed for the rest branch on
2026-09-05, on the other branch of the same function.

Seven things landed: one canonical constant for the rest band, a stroke floor that follows knot 0
when the hand is parked, a knot-0 tilt pin, a seed-relaxed cup z box, an honest acceleration and
`post_release` for a moving seed, a driver belt that holds the machine on a carry that is not a
carry, and a start-up planner warm-up.

**The E-STOP's cause was then pinned by measurement the same evening: the default 6-thread OpenBLAS
pool.** The solve is thousands of small numpy calls fanning out to six busy-spinning workers; idle
that is free (194–223 ms capped or not), but at **three busy cores of six** the uncapped solve takes
**1350–2314 ms** and gaps the 40 Hz emitter **225–942 ms**, past the can-bridge's 250 ms watchdog,
while the capped one takes **214–217 ms** at any load. The earlier `--threads 1` null that withdrew
this hypothesis was **sampled below the knee**, at 0–2 busy cores, where the two arms are identical
by construction. Three more things landed for it: the cap in the launch file, a start-up read-back
that WARNs when it did not land, and a per-stage solve split — which immediately showed that
`validate_cycle`, not the QP, is ~89 % of a solve. **UH-3 has still not been flown.**

---

## Symptoms

Six runs, all `tests/hardware/unified_cycle_bench.py --rung carry`. Timestamps are the shell
prompt's. The driver transcript is preserved at
`temp/logs/cycle_ladder_20260906_driver_runs{1-3,4-6}.txt`, the Teensy serial capture at
`temp/logs/cycle_ladder_20260906_120224.log`.

| # | prompt | invocation | outcome |
|---|---|---|---|
| 1 | 12:05:13 | `--dx 60` | **P1 REFUSED** — `age=0.06 s mode='STANDBY' streaming=True` |
| 2 | 12:06:05 | `--dx 60` | **`SETTLE_SITE`** (pre-solve, 9.3 ms) |
| 3 | 12:06:23 | `--dx 10` | **`SETTLE_SITE`** (pre-solve, 2.5 ms) |
| 4 | 12:07:29 | `--dx 10 --z-mm 690` | **`HAND_STROKE`** (post-solve, **2158.89 ms**) |
| 5 | 12:08:19 | `--dx 10 --z-mm 720` | **`GUARD_LATCHED`** |
| 6 | 12:08:35 | `--dx 10 --z-mm 720` | **`HAND_STROKE`** (post-solve, **2021.16 ms**) |

Run 1 — the robot was not activated. The driver said so, but listed all three of P1's conditions
without naming which had failed.

Runs 2 and 3 — the site derived from the live cup opening was below the planner's floor. Dropping
`--dx` from 60 to 10 changed nothing; the deficit is in z:

```
cup opening now: [-0.09, 0.94, 678.68] mm (hand -0.0380 rev)
  REJECTED_CYCLE_INFEASIBLE(SETTLE_SITE: settle site z 0.6787 m is outside the
  cup box [0.6896, 0.9846] m the same window's knots are held inside)
```

Run 4 — `--z-mm 690` cleared `SETTLE_SITE` and exposed the next layer: `HAND_STROKE: hand position
-0.031 rev outside [0.000, 9.959] rev; at t=0.000s; BELOW the homed zero, i.e. past the physical
bottom of travel`. Runs 5 and 6 — `--z-mm 720` moved the site further up and changed nothing about
the hand. Run 5 hit a guard that had latched **0.5 s before its plan call**; run 6, after the
operator recovered it, returned the same `HAND_STROKE` and latched the guard again on its own solve.

Every run also printed, on a machine the operator knew to be settled: `pose live (tilt 0.652 deg) —
NOT level`. Solve times are `/trajectory/status.cycle_plan_wall_ms`: the bench prints
`plan_wall_ms` only on acceptance and nothing was accepted, so the five carry CSVs are header-only.

---

## Diagnosis

### Defect 1 — the parked hand is below zero, and three layers did not know it

`HOMING_HAND_ABS_POS_REV = -0.1` rev: a freshly-homed hand rests **at** the homing reference,
0.1 rev below retract, by construction. The firmware knows it — `hand_source.cpp:42-47` calls the
hand parked at retract over `[Homing::HAND_ABS_POS_REV − HAND_SETTLE_BAND_REV,
JBOp::HAND_RETRACT_REV + HAND_SETTLE_BAND_REV]` = **[−0.20, +0.10] rev**. On the day the hand read
**−0.0380 rev**, squarely inside that band. Three planner-side layers assumed otherwise:

* **`unified_cycle.SETTLE_CUP_Z_MM` = 689.6 mm**, the cup box floor (the slider band's bottom,
  679.6 mm, plus a 10 mm inset). A hand at −0.038 rev puts the cup opening at 678.68 mm. **Even a
  hand at exactly 0.0 rev gives 679.6 mm — still 10 mm under.** The rung was unplannable from *any*
  non-negative hand position, which is why runs 2 and 3 could not be argued out of.
* **`feasibility.HAND_STROKE_MIN_REV` = 0.0**, with zero tolerance. Knot 0 carried **−0.031 rev**
  and was refused at `t=0.000s`.
* **The driver**, which built its settle site from the live cup z with no notion of the floor, so
  it handed the planner a site the planner would always refuse.

The **−0.038 → −0.031 rev** shift between encoder and knot 0 is not rounding: it is **+0.0089 rev**
of platform z re-pinned into the slider when the cup site is expressed in the planner's frame, and
**−0.0029 rev** from the knot-0 banking tilt (defect 2).

### Defect 2 — knot 0 came out tilted on a level machine

Found offline; the operator never saw it, because it sits **behind** `HAND_STROKE` in
`validate_cycle` and the install guard was never reached.

`_start_tilt_for` returned `None` for any state that did not follow a release. Knot 0 is not an
anchor in the tilt schedule, so the accel-bounded smoother blends it toward the terminal pin and it
comes out **tilted**, even though the raw banking value at `a_cup = 0` is exactly level. Replaying
the sitting's own seed: **2.53°** of knot-0 tilt, which through the 744.3 mm `CUP_TILT_CENTER_Z_MM`
lever is **0.1248 rev** of leg drift against `_install_continuity_ok`'s **0.06 rev** bound —
`STALE_STATE: leg position drift 0.1248 rev > 0.0600`. The probe's synthetic seed gave **3.0727° /
0.1519 rev**. The two agree at **0.0494 rev per degree**, which is what identifies them as one
mechanism rather than two coincidences. Nothing downstream refuses a tilted knot 0 on its own
merits — `validate_cycle` passes both — so the only guard is an install gate, and an install gate
can only say no.

### Defect 3 — the slam: a box that binds knots 1…n, and a seed outside it

The one that would have moved the robot. `_assemble` writes the cup z box rows for knots **1…n**
only; knot 0 is the caller's `pos0`, a constant, so bounding it can only be vacuous or
unsatisfiable. Sound for a seed inside the box — **but for a seed below the floor it silently
becomes a one-`dt` re-entry constraint**: the box demands the cup be back inside by knot 1, 25 ms
later. The QP cannot refuse (the jerk box is wide), so it satisfies it the only way it can, by
launching the cup out of the seed. The 11.2 mm climb inside one `dt` leaves ~0.9 m/s at knot 1, and
under a mean-square-acceleration objective **coasting is free** — so it coasts, to the ceiling.

Measured on the shipped UH-3 request (parked hand → cup z 678.398 mm, 11.20 mm under the 689.6 mm
floor; 60 mm lateral; 1.4 s):

| | before | after |
|---|---|---|
| cup z peak | **984.5999 mm** (the ceiling) | 689.6000 mm (the settle) |
| hand peak | 0.3162 → **9.6482 rev** | **0.3161715 rev** |
| hand peak speed | **78.37 rev/s** | 0.3864 rev/s |
| slider excursion | **~295 mm** | ~0 |

**`validate_cycle` accepts it** — peak 9.6482 rev against `HAND_STROKE_MAX_REV` 9.9594, **0.311 rev
of headroom** — as do the jerk boxes and the workspace box. A gate that cannot fire is the worst
kind of silence: the operator's only warning would have been the machine doing it, with a ball in
the cup.

### Defect 4 — the driver's "NOT level" warning was inverted

`trajectory/commanded_pose` publishes the **intent** frame: `_intent_orientation` runs
`levelling.uncorrect_pose`, *removing* the gravity/tilt correction. So a platform actively tilted
to compensate for base tilt — i.e. genuinely level in the world — publishes the correction, negated.
The 0.652° flagged as "NOT level" was the levelling correction reported back correctly (gravity
offset **[0.0126, 0.0011] rad = 0.725°**, less the loaded map's residual at that pose). The driver
was telling the operator to check the one thing that was right.

### Defect 5 — a moving seed was handed free fall too

Found by the 14:08 measurement, not by the sitting. Rep 2 of the recorded arm refused
`HAND_STROKE: hand position −0.002 rev outside [0.000, 9.959] rev; at t=0.019s` — the **unrelaxed**
bound, on a plan whose knot 0 was *above* zero, so the parked-hand relaxation correctly did not
apply.

The trigger is a probe-sequencing artefact sitting on top of a real defect. The probe never waits
for a window to finish, so rep 2 seeded from rep 1's **still-running** plan at τ = 0.3033 s — hand
at **+0.010434 rev, ascending at +0.2617 rev/s**. `_cycle_start_state`'s MOVING branch sets
`post_release=True` with `cup_accel_mm_s2=None`, so `to_cup_state` supplied **free-fall g** as the
boundary condition of a window planned over a live carry with no ball in the air. That is exactly
the lie fixed for the *rest* branch on 2026-09-05, on the other branch of the same function.

A/B on the identical seed: as shipped, `knot1 − knot0 = −4.4e-9 rev` with velocities **+0.2608 →
+3.3553 rev/s**, so the cubic Hermite between them dips **−0.011953 rev** below knot 0 (−0.002158
absolute — **0.38 mm** of slider, **120× the 1e-4 dive tolerance**) and is refused. With an honest
acceleration — zero, here — the head is `+6.527e-3 rev = v0·dt` and the dip is **0.000000**;
accepted. **The dive tolerance is deliberately not widened**: 0.38 mm is a real command, and the
tolerance was sized for 79 nm of interpolation curvature. The fix is the boundary condition.

### The E-STOP

Both latches are in the Teensy capture, and both fall inside a `plan_cycle` solve. Neither attempt
that refused *before* solving gapped anything (`rx` flat at 48–50/s).

| | latch #1 | latch #2 |
|---|---|---|
| wall clock | 12:08:18.767 | 12:09:05.766 |
| `sp_age_ms` | **804** | **324** |
| Teensy `rx` that second | **+18** (nominal 50) | **+38**, then +24 |
| `guard_mode` / `output` | 1 → **2** / 1 → **0** | 1 → **2** / 1 → **0** |
| solve it fell inside | run 4, **2158.89 ms** | run 6, **2021.16 ms** |

What the capture rules out, in the latch second itself: `[hand7]` shows `sent=566440` frozen,
`lane=idle`, `guard=ARMED`, with `lead`, `dev_over`, `dev_max=10.9794`, `dev_cmd` and `dev_fb`
byte-identical to their pre-ladder values — **stale-lane history, not a deviation event**, and no
deviation fired. `[axes] fresh=7/7`, all seven `s8` (CLOSED_LOOP). `[canhealth] jugglebot` reads
`err=0 tec=0 rec=0 txq=0 gated=0 defer=0` and `[cantx] defer_by_class` reads `legs=0 hand=0`:
**the CAN bus is innocent** — the hole is upstream of the Teensy, on the Jetson/UDP side. Recovery
at **12:08:52.766** (`guard_mode` 2 → 1, `output` 0 → 1) preceded run 6's plan call, which is why
run 6 returned `HAND_STROKE` rather than `GUARD_LATCHED`.

**The watchdog is right.** `MPC_CMD_STALENESS_US = 250000` (`canbridge_config.h:284`) latches
`MPC_STALE` when no accepted Setpoint frame arrives for 250 ms while `s_mpc_active`
(`fault_machine.cpp:371-375`), and it holds until an explicit `CLEAR_ERRORS`. An 804 ms hole in a
40 Hz stream is what it exists to catch.

**The offline reproduction is real.** `tools/probes/emitter_gap_under_solve.py` drives a production
`TrajectoryNode` with its real emitter thread and reads the emitter's own `_max_emit_gap_s`. Under
CPU contention: **2209 ms solve → 198 ms gap** (just under the threshold); **4900 ms → ~2000 ms**.
The solve is ~16 ms on an idle box and is thousands of *small* numpy calls — no BLAS hotspot,
nothing that releases the GIL for long. `sys.setswitchinterval` at 0.005 / 0.001 / 0.0002 was
measured **ineffective** (gap 2.4–3.4 s throughout).

**But the operator's measurement did not reproduce it** (14:08–14:11, launch up, robot activated,
nothing moving, `--reps 3`):

| arm | solves | max emitter gap | verdict |
|---|---|---|---|
| as-is, inside a **recorded** session | 245 / 21 / 214 ms | **0 ms** | survives |
| `--threads 1`, same session | 223 / 20 / 186 ms | **0 ms** | survives |
| as-is, **fresh unrecorded** session | 247 / 76 / 215 ms | **0 ms** | survives |

Under real session load the solve is **190–250 ms, not 2 s**, and at that length the emitter misses
nothing at all. **BLAS thread count and rosbag recording are both ruled out** — neither moved the
number.

The leading candidate was then a **cold first solve** — `reload_coordinator_node._unified_warm_planner`
exists precisely because its own first joined solve measured **3267 ms** against a **424 ms** warm
median, and the bench driver calls `trajectory_node`'s service *directly*, so that path had no
warm-up at all. **Measured, and withdrawn.** In a fresh process the first solve costs 191.0 / 189.8
/ 185.4 ms against a second solve at 193.2 / 183.5 / 178.9 ms; with a warm-up ahead of it, 183.2 /
181.2 / 183.3 ms. **The cold penalty is 5–7 ms, about 3 %** — nothing like 2 s, and it never
explained the *second* slow solve anyway.

So the 2.1 s solves stay **open**, and **CPU oversubscription is the only class that reproduces
them at all** (`--load 2`: 2209 ms solve → 198 ms gap).

The threading claim the Phase 4 design rests on is *structurally* correct: `uc.plan_cycle` runs
with `_plan_lock` **not held** (`trajectory_node.py:3650`, `:3676`, `:3680`), and the emitter takes
that lock only for a four-field snapshot at 40 Hz (`:845-849`). The solve does not block the
emitter through a lock. It starves it through the **CPU**, which is a different mechanism.

### Cause pinned (2026-09-06 evening) — the BLAS thread pool, and the null that hid it

**The actor is the default OpenBLAS thread pool.** Measured this evening, 60+ reps, dose-response.

First, the two interpreters are the same underneath, which removes the leading candidate from
Open Question 8 (*"the launched node's solve is ~7× slower than the probe's"*): the launched
`trajectory_node` (`/usr/bin/python3` 3.8.10, numpy 1.24.4 from `~/.local`) and the probe (venv)
load the **byte-identical** OpenBLAS 0.3.21 wheel — `libopenblas64_p-r0-cecebdce.3.21.so`,
pthreads/armv8 — with a **default 6-thread pool**, and no `OPENBLAS_NUM_THREADS` /
`OMP_NUM_THREADS` / `MKL_NUM_THREADS` / `NUMEXPR_NUM_THREADS` is set anywhere on this box. It was
never an interpreter difference. It was what the two processes were competing with.

| busy cores (of 6) | **default pool** | **`--threads 1`** |
|---|---|---|
| 0 — idle | 194–223 ms | 195–207 ms |
| 1 | 520–674 ms | 200–203 ms |
| 2 | 506–546 ms | 207–210 ms |
| **3** | **1350–2314 ms**, emitter gap **225–942 ms** — **LATCHES** | 214–217 ms, gap 27–31 ms |
| 2 × 6-thread burners | **876–4028 ms**, gap **1799 ms** — **LATCHES** | 204–210 ms, gap 126 ms |

**Idle, the two arms are indistinguishable — 194–223 against 195–207 ms. At three busy cores of six
they are a factor of ten apart, and only one of them E-STOPs the machine.**

**The mechanism.** The solve is thousands of *small* numpy calls (that part of the earlier reading
was right). Each one fans out to a 6-way pool whose workers **busy-spin** waiting for the next call.
On an idle box the spinning is free. The moment anything else wants a core, the spinners are
descheduled and every one of those thousands of calls pays a scheduler round trip — **6115–9461 ms
of CPU burned for 2081–3381 ms of wall, i.e. ~2.9 cores of a 6-core box, for a solve that needs
one** — and the same spinners **evict the 40 Hz emitter thread**, which is a different thread in the
same process asking for a core every 25 ms. The hardware's five slow solves (**1655.1 / 2021.2 /
2158.9 / 1461.5 / 1444.7 ms**) sit inside the 3-busy-cores default band.

**Why the 14:08 measurement said the opposite.** It was sampled at **0–2 busy cores — below the
knee**, exactly where the table says the two arms are identical by construction. The `--threads 1`
arm was never paired with a load arm, so the experiment could not have separated them whatever the
answer was. A second confound compounds it: the probe's process is *quieter* than the launched
node's — mocked rclpy, so no DDS threads and no 98 Hz `/robot_state` ingest — so even the "same
request, same box" comparison was not comparing the same thing.

**The bag confirms the shape end to end** (`~/Desktop/rosbags/2026-09-06_15-24-53`, the 15:25
re-attempt):

| instant | event |
|---|---|
| 15:24:58 | warm-up solve **219 ms** — it ran, it was fast, and it did not help |
| 15:25:20.612 | solve starts |
| 15:25:21.339 | last setpoint before the hole |
| 15:25:21.699 | `fault_state` NONE → **MPC_STALE** — **+1.088 s into a 1.655 s solve** |
| 15:25:21.838 | stream self-resumes |
| 15:25:22.267 | install, `max_emit_gap_ms=` **494.6** |

`/trajectory/status` gapped **1667.7 ms** while `/robot_state` **never gapped** (35.8 ms worst) —
so the blockage is **`trajectory_node`-local**, not a box-wide freeze and not the transport.
Nothing moved (legs ≤ 0.00036 rev). The CAN side is clean again (`setpoints_rejected=0`,
`seq_gaps=0`, `defer jb=0`). The operator's `clear_errors` came at 15:26:05 — **43.8 s latched**.
The decisive field was `max_emit_gap_ms` on `/trajectory/status`; there is **no `sp_age_ms` on
`/link_status`** (the nearest is `heartbeat_age_ms`), which is worth knowing before the next hunt.

**The warm-up is now definitively insurance and not medicine**: it ran, at 219 ms, twenty-two
seconds before the latch, and the next solve still took 1655 ms.

---

## Discussion

### (a) Why the refusals stacked — one fact, unspelled in three layers

Nothing here was three bugs. It was one physical fact — *the hand rests below its own zero* — that
the firmware spells and three planner layers did not. Each layer had a local reason to assume a
non-negative hand (`HAND_STROKE_MIN_REV` is the operating band's bottom; the cup box is inset above
the slider band's bottom; the driver reads a live cup z), and each was individually defensible. The
failure is that they were **serial**: clearing the top one only revealed the next, so the operator
discovered the same fact three times in three vocabularies, with the last discovery costing a 2 s
solve and an E-STOP.

That is why "one canonical constant" is the shape of the fix rather than a tidiness preference.
`HAND_HOMED_REST_FLOOR_REV` now lives once, in `hand_stroke.py` — the lowest layer of
`motion/trajectory` (`math` plus the generated config, no intra-package imports), so everything
above it imports it without a cycle — derived from the same two constants
`hand_source.cpp::hand_settled_at_rest` uses. Before today it was an expression in `feasibility`
and arithmetic inside a `cup_cycle` comment; the third spelling, in `unified_cycle`'s settle site,
did not exist at all, which is exactly why that layer refused.

The contract has all three parts: the constant is the invariant, `hand_stroke` is the single
enforcement point, and `test_hand_rest_floor_matches_the_firmware_settle_window` fails if firmware
and planner disagree — including if the firmware keeps both numbers but changes the *expression*
(the lower edge re-anchored to `HAND_RETRACT_REV`, say), which a value-only check would pass
straight through.

### (b) "Knot 0 must equal the machine in every channel" — the invariant, not a physics choice

The tilt pin is easy to misread as a modelling decision: *should a carry start level, or banked?*
It is not that question. `_start_tilt_for` now returns the **seed's own** `(rx, ry)` — whatever
they are. A level machine gets a level knot 0; the ~0.65° the levelling map leaves standing is
carried exactly; a seed past the 12° ceiling raises `TILT_PIN` rather than being silently clamped.
**The pin carries what the machine is at; it does not assume level.**

That is the general form of the week's guards. Four separate fixes converge on it: **position and
velocity** (`_install_continuity_ok`, which until 2026-09-05 compared positions only and would
install a fictional knot-0 velocity unremarked; it now carries leg and hand velocity terms),
**hand** (`_cycle_start_state` / `_cycle_stroke_floor`: knot 0's slider is where the machine's
slider *is*, including below zero, and the gate's floor follows it rather than refusing it),
**tilt** (`_start_tilt_for`, today), and **acceleration** (defect 5, today: a moving seed was told
it was in free fall). Stated once: **a plan's knot 0 must equal the machine's commanded state in
every channel — position, velocity, hand, tilt, acceleration.** Every one of these defects was a
channel where knot 0 was allowed to be somewhere the machine was not, and in every case the only
downstream guard was an install refusal — a gate that can say no and nothing else. Pinning knot 0
at the source is the only place the answer can be *yes*.

Defect 5 is worth dwelling on, because it is the *same defect as 2026-09-05's*, on the other branch
of the same function, found a day later by a probe that happened to seed mid-window. The rest
branch was fixed with a predicate; the moving branch kept `cup_accel_mm_s2=None` and
`post_release=True` as a shrug. A fix that repairs one branch of a two-branch lie has not closed the
class — and the tell was there in the code all along, in the shape of the two branches.

The measured cost is small: across the unified gate's eight displaced-launch cases, jerk rose
**+0.8–8.8 %** and acceleration **+1.6–14.5 %** (worst 88 107 → 95 884 mm/s³; 709.5 → 812.7 mm/s²),
verdicts unchanged, `cycle_gate` byte-identical. One number is worth reading twice:
`preposition_err` **collapses to a single value, 0.0283157 mm, in all eight directions**. Before
the pin it differed by direction, because knot 0's uncommanded tilt gave some directions a pre-tilt
head start. That was free performance the plan had no right to, taken from a state the machine was
not in.

### (c) The slam — a box cannot be asked to re-enter in one `dt`

Defect 3 is not "the QP misbehaved". The QP did what it was told, and what it was told was
impossible to satisfy gently. Bounding knots 1…n while leaving knot 0 free is a sound structure
*for a seed inside the box*; outside it, there is no jerk limit low enough to make an 11 mm step in
25 ms anything but violent.

The fix is to stop asking: the effective bound becomes `min(z_min, seed_z)` / `max(z_max, seed_z)`.
Crucially, **nothing that pins where the window ENDS moves** — `_gate_settle_site` still refuses a
rest site outside the *true* box, a throw's release site is a hard equality — so a relaxed window
can only linger near where the machine already is; it cannot travel somewhere new. And for a seed
inside the box, `min`/`max` return the configured bounds *as the same floats*: the assembled
program is bit-identical and the T-U2 parity fixtures stay exact, asserted matrix-for-matrix rather
than inferred. The 20 mm allowance is derived, not chosen: 10 mm of inset plus the 6.326 mm rest
band gives a **16.33 mm** deepest legitimate seed below, and 10.00 mm above. Further out is a
boundary-condition problem — a stale state, a hand outside its operating band — and it now refuses
`START_BELOW_BOX` / `START_ABOVE_BOX` **with the numbers**, where before it reached the solver and
came back *"unbounded dual step admitting inequality 112"*: true, and useless.

**The LAUNCH trade is the uncomfortable part, and is deliberately unresolved.** The same 11.20 mm
deficit distorts the shipped 0.6 s LAUNCH: peak **886.166 mm**, 26 mm past its own 860 mm release
site, at **107.5 m/s²** of cup acceleration = 3400 rev/s² of hand against the owner-signed 3500 cap.
The relaxed solve is plainly better — **860.000 mm** at **45.3 m/s²**, half the acceleration for
the same take-off velocity. It is not shipped because of a **cross-layer collision**: with the
floor at the seed, a release-terminal window winds up at the bottom of the stroke before the throw,
and two knots sitting *on* the floor with opposite velocities put the interpolated cubic **0.007 rev
(0.22 mm)** under it — which `validate_cycle` refuses as `HAND_STROKE`, *"the plan DIVES past the
bottom of travel"*, on a rung that plans today. **Turning a working rung into a refusal to buy a
distortion every gate already accepts is the worse trade**, so LAUNCH keeps the box it has. The
close-out is a per-knot floor vector — relaxed for the escape, the true floor re-applied after the
climb — which the row structure supports but which needs the re-entry knot, i.e. a second solve.

One behaviour change is worth stating plainly: a **LANDING** planned from the park now **refuses**
`HAND_STROKE` instead of slamming 295 mm (its unrelaxed solve is the carry's failure exactly — cup
to 984.6 mm, hand to 9.6437 rev at 78.3 rev/s), so the choice is a loud refusal versus a silent slam
and the refusal wins. Nothing shipped plans one; the coordinator's chained LANDING is seeded at a
release, inside the box.

### (d) The E-STOP — the watchdog is right; the 135× inflation is the finding

It would be easy to write this up as "the staleness threshold is too tight for a planner in the
loop". It is not. 250 ms is ten missed emitter periods on a machine whose leg-path safety authority
is a Teensy-side deviation guard fed by that stream; a watchdog that tolerated a 2 s hole would not
be a watchdog. It fired twice, latched both times, gated the output, and cost a `CLEAR_ERRORS`.
**That is the system working.**

The finding is the **135× inflation** — ~16 ms idle to 2158.89 ms on the loaded robot — and the
mechanism the probe established is CPU starvation, not lock contention. That matters because *the
Phase 4 design's answer to "does planning disturb the wire?" is thread separation*, and thread
separation is exactly what does not help here. `uc.plan_cycle` genuinely runs unlocked; the emitter
genuinely holds `_plan_lock` for microseconds. Both are true and neither prevents the gap, because
a GIL-holding, CPU-bound solve on an oversubscribed box starves a 40 Hz thread regardless of what
locks either takes. **The separation the design assumes is necessary and not sufficient.**

`sys.setswitchinterval` is **withdrawn** on measurement: three values spanning 25× left the gap at
2.4–3.4 s. It was the cheapest hypothesis and it is simply wrong — the starvation is OS scheduling,
not GIL hand-off latency. BLAS thread pools and rosbag recording are withdrawn on the operator's
measurement.

Which leaves an uncomfortable but honest position: **the offline reproduction is real and the
on-robot reproduction failed.** The gap class exists — 2209 ms of solve buys 198 ms of gap, and the
hardware's 804 ms at 2158 ms sits inside the same 9–46 % band — but the contention that produced 2 s
solves at 12:08 is unidentified, and the same box at 14:08 solved the same request in 214 ms with
zero missed ticks. Shipping a fix against an unreproduced cause is the "rescue the hypothesis"
failure this project has written down before. So the warm-up ships (cheap, medicine the coordinator
already takes at the other door, commands nothing) and the E-STOP stays open with the recipe in the
runbook.

For scale: `go_to_pose` is the precedent this path was justified against, and it blocks the same
executor for **90–377 ms** today. At the observed 9–46 % gap ratio that is a 34–173 ms gap — under
250 ms by about 1.4×. **The precedent has no margin to spare**, which is why a 2 s solve here is a
safety question rather than a latency annoyance.

### (e) The driver's belt and hold, and why `validate_only` is the follow-up

`PlanCycle`'s contract is that `accepted` means *gated **and installed***. By the time the driver
reads `hand_peak_rev` the plan is already streaming. So the belt — refuse if the planned hand peak
is more than **0.5 rev (16 mm)** above `hand_rev_for_cup_z(settle z)` — is the last chance anything
on this side has to notice that a 60 mm sideways carry has become a 295 mm vertical stroke. It is
one-sided (a hand below settle is not an excursion) and reads the **requested** settle height, so
`--z-mm` moves the bar with it. Against the real number it fires:
`carry_excursion_refusal(9.6482, …)` prints *"lift the hand 295 mm"*.

A belt that only prints is worse than none, which is why `trajectory/hold` is ordered **first**:
hold, explain, stop. `hold` builds a profiled decel-to-rest from the live commanded state and
installs *that*, superseding the cycle — and because a legacy hold carries **no hand channel**,
`HAS_HAND` falls and FW 17's falling-edge decay leaves the slider where it is rather than
continuing the stroke. The driver never assumes it worked: it prints one of five lines, four of
which contain *"IS STILL RUNNING"*, so a missing or refused service is loud.

But the belt is a workaround for a service that installs before anyone can look at the result. The
structural answer is a **`validate_only` field on `PlanCycle.srv`**: solve, gate, return the peaks,
install nothing. That is a wire-format change and is a follow-up rather than done here. The three
smaller driver fixes are the same shape — say the true thing, in the operator's units: the settle
site is lifted to the floor with a plain sentence, P1 names which of its three conditions failed,
and the tilt line explains the levelling correction instead of telling the operator to distrust a
correct reading.

### (f) A null is only evidence where the two arms are known to differ

This entry withdrew the BLAS hypothesis in the morning and reinstated it as *the cause* in the
evening, on the same box, with the same probe. That is worth writing down carefully, because the
error was not carelessness — it was a measurement that looked clean and conclusive and could not
possibly have found the effect.

The 14:08 arms were **`--reps 3` on a robot sitting idle**: launch up, activated, nothing moving,
which is 0–2 busy cores of six. The dose-response table says the capped and uncapped arms at 0–2
cores are **194–223 vs 195–207 ms** — indistinguishable, and *necessarily* so, because with cores to
spare a spinning pool costs nothing. The experiment had **no power to reject anything**, and it was
read as a rejection. The tell was available at the time and was not noticed: the null arm reported
*the same solve time as the null hypothesis predicted for a healthy box*, i.e. the measurement
reproduced the healthy case and not the failing one. **A probe that does not reproduce the symptom
cannot exonerate a candidate for causing it.**

The correction is procedural and cheap: the probe's `--load` knob already existed, and one arm —
`--load 3`, default vs `--threads 1` — separates the two by a factor of ten. It was not run because
the morning's framing was *"does the emitter survive a normal session?"* rather than *"what makes a
solve take 2 s?"*, and the first question is answerable on an idle box while the second is not. The
probe docstring now leads with the causal arm and marks the 14:08 arms as below-the-knee provenance,
so the next reader cannot repeat the reading.

Two further confounds are recorded because they were also invisible at the time. The probe's process
is **quieter than the launched node's** — mocked rclpy, so no DDS threads and no 98 Hz
`/robot_state` ingest — so "the same request in the probe takes 200 ms" was never the same
experiment. And the `--threads 1` arm was never paired with a load arm, so even a genuine difference
would have had nowhere to show up. `plans/active/unified-7dof-planner.md`'s own pre-registration
rule ("pre-register the fallback and the decision criterion that flips you to it") has a sibling this
episode names: **pre-register the condition under which the arms are expected to differ, and refuse
to read a null taken outside it.**

The second reframing in the same session is smaller but the same shape. The per-stage split was
added on the assumption that the QP dominates a solve, and its first output said `validate_cycle` is
~89 % and the QP ~6 %. Nothing depended on the assumption yet, which is the only reason it was
cheap — had a fix been built on "optimise the QP" first, the split would have arrived after the
work. Both corrections came from *instrumenting before concluding*, which is the same lesson from
opposite ends.

---

## Fix

1. **One canonical rest-band constant.** `HAND_HOMED_REST_FLOOR_REV` moves to
   `motion/trajectory/hand_stroke.py`, derived as `hw.HOMING_HAND_ABS_POS_REV −
   HAND_SETTLE_BAND_REV` = **−0.20 rev = −6.326 mm**. `feasibility.py` imports and re-exports it;
   `cup_cycle.py` imports it and derives `REST_FLOOR_BELOW_HAND_ZERO_M` (**6.326 mm**) for the
   `SEED_OUTSIDE_BOX_MAX_M` derivation that previously spelled the arithmetic out in a comment.
   `unified_cycle_bench.py` never spelled the band and is unchanged on this point. New test
   `test_hand_rest_floor_matches_the_firmware_settle_window` parses `canbridge_config.h`, the
   canbridge `hardware_config.h` **and** `hand_source.cpp` — values *and* the `pos >=` expression.

2. **The stroke floor follows knot 0 when parked** (`feasibility.py`). `_cycle_stroke_floor`
   returns `max(HAND_HOMED_REST_FLOOR_REV, hand_at(0.0) − 1e-4)` for a plan whose first knot is
   below zero, `HAND_STROKE_MIN_REV` otherwise. Read through `hand_at(0.0)`, not `hand_rev[0]`, so
   the floor comes from the curve the gate samples. **Dive tolerance 1e-4 rev (3.2 µm)**: the
   measured interior Hermite extremum on the 2026-09-06 carry was 2.5e-06 rev = **79 nanometres**
   below knot 0. The **catch runway floor is deliberately unchanged** at `HAND_STROKE_MIN_REV` —
   the runway asks how much travel is left to *stop* in, bounded by the physical bottom, not by
   where the hand parked. Three refusal texts now separate *re-home it* / *the plan DIVES* /
   *below the homed zero*.

3. **Knot 0's tilt is the seed's** (`unified_cycle._start_tilt_for`). Returns `state.pose[3:5]` for
   a non-post-release state; the `detach_axis` round trip is kept for the chained case, where the
   cup is in free fall and the banking objective is degenerate. Knot-0 tilt **3.0727° → 0.0000°**,
   drift **0.1519 → 0.0000 rev**, leg jerk **123 128 → 26 301 mm/s³**; a 0.65° seed is carried
   exactly. Gate cost as in Discussion (b); `cycle_gate` byte-identical.

4. **`_seed_relaxed_z_box`** (`cup_cycle.py`). `(min(z_min, seed_z), max(z_max, seed_z))`, applied
   to **rest-terminal windows only** (`throw is None`), refusing `START_BELOW_BOX` /
   `START_ABOVE_BOX` past `SEED_OUTSIDE_BOX_MAX_M` = **0.020 m**. The carry numbers are in
   Diagnosis; a seed inside the box assembles the **bit-identical** program (all six matrices
   compared with `np.array_equal`), and a seed 25 mm below — previously `INFEASIBLE("unbounded dual
   step …")` — now refuses `START_BELOW_BOX` with the numbers.

5. **An honest boundary condition for a MOVING seed** (`trajectory_node._cycle_start_state`). The
   moving branch now samples the **active cycle's own cup acceleration at τ** and takes
   `post_release` from the cycle's release marks, instead of passing `cup_accel_mm_s2=None` with
   `post_release=True` and letting `to_cup_state` substitute free fall. A moving seed with **no**
   active cycle is **refused rather than guessed**. The dive tolerance is deliberately not widened.

6. **The driver** (`unified_cycle_bench.py`). Settle site lifted to `SETTLE_CUP_Z_MM` with a plain
   sentence (`--z-mm` still honoured exactly, with a warning below the floor);
   `carry_excursion_refusal` at 0.5 rev above `hand_rev_for_cup_z(settle z)`; `trajectory/hold`
   abort ordered **first**, with four loud failure lines; the levelling-map tilt explanation; the
   P1 STANDBY hint; `REV_PER_MM` 31.65 → **31.6172**, pinned. One existing test re-anchored —
   `test_without_the_seed_pin_the_carry_starts_where_the_machine_is_not` was written against
   numbers measured on the *slamming* carry (3.0727° / 3.0 mm); with the carry flat the same
   unpinned schedule leaves **0.3971° / 0.4568 mm ≈ 0.020 rev**, inside the 0.06 rev install bound,
   so the bars are halved and a flatness assertion added.

7. **A start-up planner warm-up** (`trajectory_node._warm_planner_once`). A one-shot timer 0.5 s
   after construction solves one throwaway **5 mm SETTLE carry** at `SETTLE_CUP_Z_MM` under the
   node's own limits — 5 mm and not more because at the config defaults 1000/5000/30000 a 7 mm
   carry refuses `LIMIT_JERK` at 30 833 — on the executor thread, result dropped on the floor
   (`_install` is not called and nothing reads live state, so it cannot move a knot; a failure is
   WARN-logged). It costs **~185 ms at start-up**. It ships as cheap insurance and as the same
   medicine `reload_coordinator_node._unified_warm_planner` already takes at the other door — but
   say it plainly: **it does not explain the 2 s solves.** The measured cold penalty is 5–7 ms.

8. **Test-clock freeze and load-sensitivity documentation**, in the same commit:
   `test_the_velocity_term_passes_the_same_origin_re_installs` had its clock frozen in this commit, but it remains load-sensitive (under an overlapping pytest run at ~83 % CPU it failed at τ = 0.929 s, in the wall-clock τ-overrun class the mocked-node tests share) — it passes on a quiet box; the gate runs alone by rule, and hardening the class (freezing every τ source in those tests) is a follow-up (see Verification and Open Questions, item 4); the probe's mid-window reseeding is what surfaced defect 5.

### The E-STOP fix (2026-09-06 evening) — the measured one

9. **The BLAS thread cap** (`ros_ws/src/jugglebot/launch/jugglebot_launch.py`).
   `OPENBLAS_NUM_THREADS=1` and `OMP_NUM_THREADS=1` on the `additional_env` of the three nodes that
   call the planner — `trajectory_node`, `reload_coordinator_node`, `catch_coordinator_node` —
   through one shared `_planner_blas_env` dict carrying the dose-response table as its comment.
   **The launch file is the only place this can go**: the variables have to be set before numpy is
   imported, so by the time any node module executes it is far too late. **Deliberately not
   launch-wide** — `mocap_node` and `ball_tracker_node` do genuinely large-matrix work where a real
   pool earns its keep, and neither is on the setpoint stream's thread; a second test
   (`test_the_blas_cap_is_not_applied_launch_wide`) makes a future globalisation re-argue that
   rather than absorb it. Both variables, because numpy 1.24.4 here links OpenBLAS but an
   OpenMP-threaded transitive dependency reads the other name.

10. **The self-check** (`motion/blas_threads.py`, called by all three nodes at start-up). Reads the
    effective pool — `threadpoolctl.threadpool_info()` when importable (**3.5.0 is present in BOTH
    interpreters on this box, the venv and `/usr/bin/python3`, which is the one the launch actually
    runs**), else the `*_NUM_THREADS` environment — logs one INFO line `blas threads: N (source)`,
    and **WARNs loudly** when N > 1 *or unknown*, naming the two variables, the launch file and this
    entry. A **WARN and not a refusal**: a developer running the node by hand has no launch file and
    must still be able to work, and an uncapped pool is a slow solve rather than an unsafe command.
    Unknown warns too — fail-closed means a node that cannot *prove* it is capped must not read as
    capped. threadpoolctl is the authoritative reader because it inspects the loaded shared objects,
    so it catches the case the env read cannot: a variable set too late to take effect.

11. **Solve attribution** (`unified_cycle.CycleMeta.stage_wall_s`, additive; the split on
    `trajectory_node._accept_cycle`'s INFO line). Five stages — `qp` (`cup_cycle.plan_window`),
    `tilt` (`tilt_schedule`), `dec` (`decompose`), `val` (`validate_cycle`) and `cont`, the
    **residual**, so the five sum to `plan_wall_s` exactly and nothing can hide between them. Plus
    `os.getloadavg()[0]` as `load1`, because **the bag carries no host-CPU channel at all** and
    after the fact there is no way to ask whether the box was busy. `max_emit_gap_ms` — the other
    half — already rides `/trajectory/status`.

    **The split's first act was to overturn the assumption it was written under.** It was added
    expecting the QP to dominate. Measured (2026-09-06, venv, idle box, ms):

    | window | qp | tilt | dec | val | cont | total |
    |---|---|---|---|---|---|---|
    | 0.6 s LAUNCH | 3.3 | 1.2 | 1.1 | **63.0** | 0.5 | 69.1 |
    | 1.0 s LANDING | 6.7 | 3.9 | 2.6 | **117.7** | 0.6 | 131.5 |
    | 1.4 s STEADY | 11.2 | 4.2 | 3.7 | **163.9** | 0.7 | 183.6 |

    **`validate_cycle` is ~89 % of the solve and the QP ~6 %** — a factor of ~15. "The planner is
    slow" has always meant "the *gate* is slow". That is not a complaint (the gate is why nothing
    unexecutable reaches the wire), and it is pinned by
    `test_the_gate_and_not_the_qp_dominates_a_healthy_solve` so a future change that moves the bulk
    has to come and re-argue the reading.

    **And under starvation the shape inverts — which is what turns the split from a curiosity into
    a discriminator.** Measured on the shipped code the same evening (`--reps 2 --load 3`, three
    busy cores of six, ms):

    | arm | qp | tilt | dec | val | total |
    |---|---|---|---|---|---|
    | uncapped, rep 1 (cold QP) | **2256.4** | 21.7 | 3.7 | 636.7 | **2920.3** |
    | uncapped, rep 2 (warm start) | 6.7 | 18.6 | 3.7 | 609.1 | **641.1** |
    | capped, rep 1 | 10.4 | 4.8 | 4.9 | 186.4 | **208.3** |
    | capped, rep 2 | 7.8 | 6.3 | 5.0 | 195.6 | **216.7** |

    The gate inflates **~3×** under starvation (186 → 609–637 ms) — bad but proportionate. The
    **cold QP inflates ~200×** (10 → 2256 ms), because that is the densest run of tiny numpy calls
    and every one of them pays a scheduler round trip. So the operational read of a slow line is:
    **`val` large with `qp` small is just a big window; `qp` comparable to or larger than `val` is
    thread-pool starvation** — go and look at the `blas threads:` line and `load1`. (Rep 2's `qp` is
    small even uncapped because it re-uses rep 1's factorisation; the *cold* QP is the exposed one,
    which is also why the start-up warm-up is worth its 185 ms even though it explains nothing.)

---

## Verification

Scoped gate (2026-09-06, `python -m pytest tests/motion/test_cup_cycle.py
tests/motion/test_unified_cycle.py tests/motion/test_validate_cycle.py
tests/ros/test_unified_cycle_bench.py tests/ros/test_unified_cycle_integration.py
tests/firmware/test_hermite_xref.py tests/sim/test_plans_index.py tests/sim/test_logbook_search.py
tests/sim/test_logbook_front_matter.py -q -p no:randomly`) — **434 passed in 39.22 s**.

**One wall-clock flake, characterised and then fixed.** An earlier run of the same command, taken
while four background agents were saturating the box, failed
`tests/ros/test_unified_cycle_integration.py::test_the_velocity_term_passes_the_same_origin_re_installs`
with *"extended plan is not continuous at the live plan time (tau=0.785 s, hand position drift
3.9125 rev > 1.0000)"*. It is **wall-clock sensitive by construction**: it samples the plan at a
`perf_counter` τ taken one line before `_current_state()` and bounds the difference at 1e-6 rev, so
on a loaded box τ can overrun the *live* 0.6 s LAUNCH it extends — the live plan then reads its
terminal HOLD while the joined plan reads the extension, and the continuity check correctly reports
the difference. **Neither 2026-09-06 planner fix causes it**: a probe that neutered
`_seed_relaxed_z_box` and restored the old `_start_tilt_for`, separately and together, PASSED in all
four arms, and on a quiet box the test passes three times out of three in isolation. its clock was frozen in this commit, but it remains load-sensitive (under an overlapping pytest run at ~83 % CPU it failed at τ = 0.929 s, in the wall-clock τ-overrun class the mocked-node tests share) — it passes on a quiet box; the gate runs alone by rule, and hardening the class (freezing every τ source in those tests) is a follow-up.

Firmware pin (2026-09-06, `python -m pytest tests/firmware/test_hermite_xref.py -q -p
no:randomly`) — **12 passed in 15.25 s**.

Constant-move parity (2026-09-06, a scratch probe re-running the accept/refuse sweep on the
post-edit tree) — `HAND_HOMED_REST_FLOOR_REV` is **bit-identical** (`-0.2`) to the expression
`feasibility` previously carried, `SEED_OUTSIDE_BOX_MAX_M` is unmoved at `0.02`, and the verdict
across parked starts `{-0.30, -0.2001, -0.20, -0.10, -0.038, -0.031, -0.002, 0.0, 0.5, 9.9594,
10.5}` is unchanged: **PARITY OK**.

Cycle gate (2026-09-06, `python sim/cycle_gate.py`) — **PASS, binding-band PASS, parity EXACT
(worst 0), banking-beats-level 11/11, slam-free YES**; 11 trials, 11 accepted, 10 core-clean.

Hardware artefacts, from the operator, all 2026-09-06: `temp/logs/cycle_ladder_20260906_120224.log`
(the Teensy serial capture, 12:02:24 → 12:09:51 — both latches, the `rx` deltas, the `[hand7]` /
`[axes]` / `[canhealth]` evidence and the recovery all come from it);
`temp/logs/cycle_ladder_20260906_driver_runs{1-3,4-6}.txt` (the driver transcript for all six runs,
copied into `temp/logs/` because it existed only in an ephemeral paste cache);
`temp/logs/unified_cycle_bench_carry_20260906_*.csv` (five files, all **header-only** — nothing was
accepted, so nothing streamed); and the 14:08–14:11 emitter-gap measurement
(`tools/probes/emitter_gap_under_solve.py --reps 3`, three arms), tabulated in § Diagnosis.

- Full gate (2026-09-06, `./run_tests.sh --full`, parallel **6788 passed / 4 skipped / 2 xfailed in 440.01 s**, serial **4 passed in 26.08 s**, total 472 s, exit 0) — **GREEN**, on the tree carrying every fix in this entry.
- A per-file run of `test_unified_cycle_integration.py` overlapping a background suite showed 4 load-artefact failures; the same file passed 239/239 three times alone (2026-09-06).

---

## Outcome

**UH-3 has NOT been flown.** No cycle was installed, no motion was commanded, the ball never moved.
Six runs, six aborts.

**Re-attempt 15:25 (operator, after a `colcon build` of this tree, bag `~/Desktop/rosbags/2026-09-06_15-24-53`).** The refusal chain is closed on hardware: the driver printed the levelling-correction line (map `2026-08-10-3bf7964f`), the carry request `settle_site_mm [59.808, 0.749, 695.270]` from a hand at +0.4865 rev was **ACCEPTED**, and the planned hand was flat (peak 0.4954 rev = the settle, "-0.0 mm above"). The E-STOP recurred: `plan 1655.1 ms` inside the launched node, the Teensy guard latched `MPC_STALE` during that solve, the carry streamed into a frozen hold (no motion), and the return move was refused `GUARD_LATCHED`. Three of three full solves in the launched `trajectory_node` have now taken 1.6–2.2 s while the same solve in the probe's own process measured 190–250 ms nine of nine times under the same load — the difference is inside the launched process or its environment, under investigation (both interpreters carry numpy 1.24.4).

Five defects are closed at the root — one canonical constant for the rest band, one invariant for
knot 0 (now spanning position, velocity, hand, tilt and acceleration), one rule for a seed outside
the box, and a driver that says true things — with the class closed rather than the instance in
each case.

**The E-STOP's cause is pinned and the fix is in** (2026-09-06 evening): the default 6-thread
OpenBLAS pool, capped to one thread in the launch file for the three planner-calling nodes, with a
start-up read-back that WARNs if the cap did not land and a per-stage split so the next slow solve
attributes itself. The watchdog behaved correctly throughout; nothing about it changes. Two earlier
readings are corrected in Withdrawn claims — the `--threads 1` null was measured **below the knee**
where no effect could exist, and "CPU oversubscription is the only class" was necessary but not
sufficient: it takes oversubscription *plus* the spinning pool, which is why a half-idle box was
enough and why 12:08 never needed an exotic explanation.

**What remains is verification on the robot, and it has not happened.** The fix is measured offline
and on the bench, not in a launched session with a ball in the cup. The first thing the next sitting
does after the `colcon build` is confirm `blas threads: 1` for `trajectory_node` in the launch
terminal; the first thing it must *capture* is `/proc/loadavg` and `vmstat 1` alongside the rung,
because the bag has no host-CPU channel and a recurrence would otherwise be unattributable again.
The probe stays as the diagnostic if a latch recurs — `--load 3`, default vs `--threads 1`, is the
arm that shows it.

---

## Withdrawn claims

* **"The driver's NOT-level warning identified a real tilt."** It did not. `commanded_pose`
  publishes the intent frame with the levelling correction removed, so the 0.652° *was* the
  correction, reported back correctly.
* **"Fixes 1 and 2 are sufficient to fly UH-3."** They are not. With the stroke floor and the tilt
  pin in place, the carry still planned a 295 mm slider slam that every gate accepted; only
  `_seed_relaxed_z_box` closed it. The two are also entangled in the other direction: fix 3 removed
  most of fix 2's *measured* magnitude (3.0727° / 0.1519 rev was an amplified reading taken against
  the slamming carry; flat, the same unpinned schedule leaves 0.3971° ≈ 0.020 rev, inside the
  install bound). Fix 2 remains right — as an invariant, not as the guard against this refusal.
* **"`sys.setswitchinterval` will recover the emitter."** Measured ineffective at 0.005, 0.001 and
  0.0002 — the gap stayed at 2.4–3.4 s.
* ~~**"BLAS thread pools are the contention."**~~ **WITHDRAWAL REVERSED (2026-09-06 evening) —
  the BLAS thread pool IS the actor.** The withdrawal rested on a `--threads 1` arm (223 / 20 /
  186 ms, gap 0 ms) taken at **0–2 busy cores**, which the dose-response table shows is **below the
  knee**: at 0–2 cores the capped and uncapped arms are *identical by construction* (194–223 vs
  195–207 ms idle), so that measurement could not have separated them whatever the truth was. At
  three busy cores of six the same pair is **1350–2314 ms vs 214–217 ms**, and only the uncapped
  arm gaps the emitter past 250 ms. The claim was withdrawn on a null measured where no effect
  could exist; it is reinstated as **the** cause. See Diagnosis § "Cause pinned".
* **"CPU oversubscription is the only class that reproduces a multi-second solve."** **Incomplete,
  not wrong.** Oversubscription is *necessary* — the solve is flat at 194–223 ms on an idle box in
  both arms. But it is not *sufficient on its own*, and stating it that way is what made the
  question look unanswerable: with the pool capped, **three busy cores buys 214–217 ms** and no gap
  at all. It takes oversubscription **plus** a 6-way spinning pool. That is why "what was competing
  for cores at 12:08?" felt like it needed an exotic answer — the box only had to be **half idle**,
  which any session is.
* **"The solve is ~16 ms on an idle box."** **Does not reproduce.** Every idle measurement this
  evening, both arms, both interpreters, is **194–223 ms** — an order of magnitude off. The 16 ms
  figure is not traceable to a run recorded anywhere in this entry and the "135× inflation" framing
  built on it (Discussion (d)) is correspondingly wrong: the real inflation is **~10×**, from ~200 ms
  to ~2 s. The conclusion that paragraph draws — that thread separation is necessary and not
  sufficient — survives unchanged; only the multiplier was inflated.
* **"Rosbag recording is the contention."** A fresh unrecorded session solved in 247 / 76 / 215 ms
  with a 0 ms gap — indistinguishable from the recorded one. **Still withdrawn** (this one was an
  arm the load did not confound: the two recorded/unrecorded arms sat at the same box load, and the
  bag's own 15:25 timeline shows `/robot_state` never gapping while `/trajectory/status` did).
* **"A cold first solve explains the 2.1 s."** It does not. Measured in a fresh process: first
  solve 191.0 / 189.8 / 185.4 ms, second solve 193.2 / 183.5 / 178.9 ms, and with a warm-up ahead
  of it 183.2 / 181.2 / 183.3 ms — a cold penalty of **5–7 ms, about 3 %**. **Confirmed on hardware
  the same day**: the 15:24:58 warm-up ran at **219 ms** and the solve 22 s later still took
  **1655 ms**. The warm-up still ships as insurance; it was never the explanation. (The trailing
  clause this bullet used to carry — *"CPU oversubscription remains the only class that reproduces
  a multi-second solve at all"* — is itself amended above: oversubscription is necessary, the
  6-thread pool is what makes a half-idle box sufficient.)

---

## Open Questions

1. ~~**What made the sitting's two solves take 2.1 s?**~~ **CLOSED 2026-09-06 (evening): the
   default 6-thread OpenBLAS pool under partial CPU load.** Dose-response in Diagnosis § "Cause
   pinned"; the fix is the launch-file cap. The sub-question — *"what was competing for cores at
   12:08 that was not at 14:08?"* — is **answered for the 15:25 event by the bag's own rates**
   (`/robot_state` at 98 Hz never gapping while `/trajectory/status` gapped 1667.7 ms puts the
   contention inside `trajectory_node`'s own process, and the pool is what turns a half-idle box
   into a 10× solve) and is **moot for 12:08**: it no longer takes an exotic answer, because with a
   6-way spinning pool the box only has to be **half idle**, which every session is. Still open only
   as a *verification* item, item 8 below.
2. **The three candidate E-STOP fixes, and what each prevents. RE-RANKED 2026-09-06 evening.**
   The cap (fix 9) is now the primary and it addresses the mechanism directly — measured 214–217 ms
   at three busy cores against 1350–2314 ms uncapped. The others become **defence-in-depth,
   sequenced AFTER it**, and one of them needs a correction:
   (a) **The start-up warm-up**, shipped — insurance against a cold first solve, nothing more; the
   15:24:58 warm-up ran at 219 ms and the next solve still took 1655 ms.
   (b) **SCHED_FIFO on the emitter thread** — prevents starvation of the wire by *any* CPU-bound
   work in the process, including work the cap does not cover. Cheap, and worth doing, but no longer
   the only candidate that addresses the reproducing mechanism.
   (c) **The solve in a worker process** — **note the correction: a worker process does NOT solve
   this on its own.** A forked or spawned worker **inherits the same 6-thread pool**, so it would
   move the spinning to another process and still oversubscribe the same six cores; it separates the
   *GIL* and the emitter's scheduling fate from the solve, which is real value, but it must carry
   the cap too. Defence-in-depth after the cap, not instead of it.
   The long-term shape is neither: **whole-plan upload to the Teensy**, so the Jetson's scheduling
   is not on the wire's critical path at all and no amount of box load can gap a stream the firmware
   is generating from a plan it already holds.
3. **Are there other one-branch fixes like defect 5?** `_cycle_start_state`'s rest branch was
   repaired on 2026-09-05 and its moving branch carried the identical lie for another day. Worth a
   deliberate sweep of every place a boundary condition is *defaulted* rather than measured — the
   `None` that becomes free fall is the pattern, and it has now cost two entries.
4. **The per-knot floor vector for LAUNCH.** The relaxed launch is measurably better (860.000 vs
   886.166 mm peak; 45.3 vs 107.5 m/s², i.e. 3400 rev/s² against the 3500 cap) and is not shipped
   only because a floor-riding release-terminal window trips `HAND_STROKE` on a 0.22 mm continuum
   ripple between two floor knots. Owner decision; it needs a second solve to find the re-entry
   knot.
5. **`validate_only` on `PlanCycle.srv`.** `accepted` currently means *gated and installed*, so the
   driver's belt inspects a plan that is already streaming and has to `hold` to undo it. A
   `validate_only` request would solve, gate, return the peaks and install nothing.
6. **The FW 18 rename bundle.** `MPC_STALE` is a historical name — the MPC chain was deleted
   2026-09-01 and the watchdog is the *setpoint stream's*. The owner has bundled the rename with
   FW 18 because the mechanism is load-bearing and the prefix only historical. One generator entry
   drives it (`config/generate_udp_protocol.py:262`), delivering four files. **Correction to the
   estimate: the site count is not ~15.** `grep -rnw "MPC_STALE"` returns **158 sites across 60
   files** (190 without the word boundary, which also catches `MPC_STALENESS`).
   `sim/analysis/known_issues.yaml`'s `MPC_STALENESS` is unrelated — a `/diagnose` signature id for
   the deleted solver's *solve-time* budget, whose `reference: controller/params.py` no longer
   exists. `mpc_active` is a separate job at **633 sites** across Python, C++ and JavaScript,
   parsed as a literal `KeyValue` key on `/link_status` in Python and JS.
7. **Does a 2 s solve also break the coordinator's `_UNIFIED_PLAN_BUDGET_S`?** **Yes.** The budget
   is 1.20 s (`reload_coordinator_node.py:575`) and there is **no runtime check** — it is used
   once, to size `_UNIFIED_LAUNCH_LEAD_S = window + B`. The plan anchors at the *install* instant,
   so the release lands at `trigger + cost + window` against an FSM expecting `trigger + lead`:
   skew is `cost − B`. A 2.1 s solve is therefore **~0.9 s late**, straight through
   `toss_sequencer.TOSS_RELEASE_GRACE_S` (0.5 s), and `_step_throwing` mints
   **`ABORTED_NO_RELEASE`** — with the ball already in the air. Not hypothetical: a 1.06 s joined
   solve under concurrent load has already put a release 0.708 s late. **UH-6 must not be attempted
   while a 2 s solve is possible.**
8. ~~**The launched node's solve is ~7× slower than the probe's (1655 / 2021 / 2159 ms vs
   190–250 ms).**~~ **CLOSED 2026-09-06 (evening) — it was never an interpreter difference.** Both
   processes load the byte-identical OpenBLAS 0.3.21 wheel with the same default 6-thread pool; the
   difference was **what each was competing with**, and the probe's own process is the quieter of
   the two (mocked rclpy — no DDS threads, no 98 Hz `/robot_state` ingest). The remaining item is
   **verification on the robot, and it is a measurement the next sitting must take**: the bag has
   **no host-CPU channel at all**, so `/proc/loadavg` and `vmstat 1` have to be captured alongside
   the rung — otherwise a recurrence is again unattributable after the fact. `load1` on the accept
   line (fix 11) is the in-band half of that; it is not a substitute for a sampled series. First
   thing to confirm after the `colcon build`: `blas threads: 1` in the launch terminal for
   `trajectory_node`.
9. Four `tests/ros/test_unified_cycle_integration.py` tests are wall-clock load-sensitive (the τ-overrun class); they pass when the suite runs alone, as the gate does. Follow-up: freeze every τ source in them. Do not widen tolerances.
