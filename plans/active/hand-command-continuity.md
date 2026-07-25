---
title: Hand-command continuity — stop clobbering a live stroke (post-throw dip + throw truncation)
created: 2026-07-25
status: active
related_logbook:
  - 2026-07-24-phase7-fourth-sitting-openloop-telemetry-ladders.md
  - 2026-07-25-toss-phase3-trace-validated.md
  - 2026-07-25-toss-phase1-action-sequencer-coordinator.md
related_code:
  - ros_ws/src/jugglebot/Teensy_code/Trajectory.h::makeSmoothMove
  - ros_ws/src/jugglebot/Teensy_code/Teensy_code.ino
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator_node.py::_arm_hand_catch
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator_node.py::_on_hand_traj_done
  - sim/hand/trajectory.py
  - config/hardware_config.yaml::jugglebot_operational.hand_catch_prime_rev
---

# Plan — Hand-command continuity

**Branch:** `mvp-trajectory-bringup`
**Covers:** fix items 3 (arm-after-stroke), 4 (velocity-continuous prelude),
5 (repack safety), 6 (prime == catch-trajectory start) from the 2026-07-25
self-toss anomaly investigation.
**Sibling plans:** `plans/active/levelling-frame-contract.md` (items 1–2),
`plans/active/fk-convergence-tolerance.md` (7),
`plans/active/catch-reach-degenerate-overshoot.md` (8).

## Context

### The observation

After every self-toss the operator saw the hand "quickly dip partially down its
stroke, then back up for the next catch", where it should simply have paused at
the top.

### The hand is designed to pause at the top

From `Teensy_code/Trajectory.h` with the shipped constants
(`LINEAR_GAIN = 1.035 / (2π · 0.00521) = 31.6172 rev/m`,
`totalStroke = 0.355 − 2·0.02 = 0.315 m`):

| point | value | meaning |
|---|---|---|
| `x2` | 0.187044 m = 5.914 rev | release — end of vel-hold, decel begins (`HAND_THROW_POS_M`) |
| `x3` | 0.315 m = **9.9594 rev** | throw stroke end, at rest |
| `makeCatch` first sample | `x3` = **9.9594 rev** | catch stroke start |
| `x5` | 0.19378 m = 6.127 rev | catch contact (`HAND_CATCH_POS_M`) |

`x3 = accelSt + velHold = totalStroke` algebraically, **independent of the
commanded velocity** — so the throw's end position and the catch's start position
are the same point by construction (`buildCommand`'s `xA[3] == xA[4] == x3` relies
on it). The intended behaviour is: stroke up, stop at the top, wait, descend.

### What actually happens

`Teensy_code.ino:538-547`, for **any** kind-0/1/2 command:

```cpp
Trajectory smooth = makeSmoothMove(activeTraj.x.front());   // prelude from LIVE position
...
packedMsgs.clear();  sendUs.clear();                        // discards whatever is playing
if (!smooth.t.empty()) packTrajectory(smooth, now_us);      // starts immediately
packTrajectory(activeTraj, event_wall_us);
```

and `makeSmoothMove` (`Trajectory.h:242-301`) seeds a quintic from
`current_hand_position` with **v = 0, a = 0** boundary conditions.
`current_hand_velocity` is declared `extern` two lines above (`Trajectory.h:47`)
and never read.

The catch arm arrives **8–18 ms after the scheduled release** (15:17:48 session:
release ≈ 1784956866.88, `Arming hand catch` at 866.888 and 866.898) — inside the
throw stroke's 65 ms deceleration ramp. So the queue is cleared mid-stroke and
replaced by a rest-to-rest ramp computed from a position the hand is travelling
through at ~100 rev/s.

Measured, `temp/logs/toss_trace_2026-07-25_15-24-25.jsonl` (hand telemetry ~100 Hz):

| t (s) | event |
|---|---|
| 20.836 | stroke starts, `pos_cmd` leaves 0 |
| 20.927 | passes the release point (5.914 rev) at 119.6 rev/s = 3.78 m/s |
| **20.942** | `pos_cmd` truncates to **7.700 rev** — the live encoder value — and a 0.36 s quintic to 10.05 rev begins |
| 20.992 | hand coasts to **10.174 rev (321.8 mm)**, past the planned 315 mm end, 0.93 rev from the 11.1 rev limit |
| 21.025 | position loop pulls back at **−31.3 rev/s (−0.99 m/s)** |
| 21.077 | **dip bottom 8.814 rev** — 1.36 rev = **43 mm**, 14 % of the stroke |
| 21.294 | back at 10.05 rev, at rest |
| 21.605 | catch descent begins (contact ~21.71) |

Down in 85 ms, back up in 217 ms, ~300 ms total. That is the operator's dip.

### The stroke timing budget (why there is plenty of room)

With `t_acc = 0.342578/v` and `t_dec = 0.747·t_acc`:

- at `v = 3.93 m/s`: stroke spans **release − 91 ms → release + 65 ms**
- the catch's first main sample is at `event − t_acc_catch`, with
  `t_acc_catch = 0.404133 / v_catch`; at the armed 3.13 m/s that is **129 ms**, so
  for an 0.8 s flight it lands at **release + 671 ms**

So a safe arming window is roughly **[release + 70 ms, release + 650 ms]** —
*superseded by Phase 0's measurement: the physical release lands +12.8…+21.9 ms
after the announcement's `throw_time` read from the bags, and up to +23.4 ms read
from a jsonl trace of the same toss, so the lower bound is release + 105 ms at a
40 ms margin. See § Phase 0 — Outcome.* — and
arming anywhere in it makes the prelude **exactly empty**, because the hand is
already at rest at `x3` = the catch trajectory's own start. `makeSmoothMove`
returns an empty trajectory and the hand simply waits at the top. The correct
behaviour costs nothing; it only needs the arm to be late.

### Collateral damage: the throw itself

The same `packedMsgs.clear()` discards the throw's own deceleration ramp. The
ball's departure conditions are then set by the position loop's reaction to a
frozen setpoint, at whatever instant the arm frame happens to land (8–18 ms of
observed jitter), rather than by the planned profile. Consistent with this:
achieved flights of **0.887 s** and **1.091 s** against a commanded 0.8 s. At the
top of the shipped flight band (1.10 s ⇒ ~5.4 m/s) the momentum is ~1.9× and the
overshoot past 315 mm grows with it, against a 351 mm hard limit.

So item 3 is not cosmetic: it is a throw-repeatability fix as much as a
cosmetic one.

### Why the re-arm exists, and why its premise fails during a toss

The double dispatch is **by design**, not a latch bug. Only ball id 11 is present
across the arm window (checked in the bag), so the per-ball one-shot latch was not
bypassed by a split track. `_on_hand_traj_done` re-opens the latch on a failed ack,
and its own comment states the rationale:

> the kind-1 stroke's catch instant is an ABSOLUTE wall_time invariant across
> retries, so a lying-ack arm still physically armed and **further repacks just
> churn** the Teensy's last-writer-wins queue

That premise is true for a **reload**, where the hand is parked at the top at rest
— a repack there is genuinely harmless. It is false during a **toss**, where the
repack lands inside the throw stroke and each one re-clears the queue and
re-preludes from a new live position. The `HAND_TRAJ_CMD` ack fails 40–60 % of the
time (the 2026-07-23 ERR_TIMEOUT epidemic), so this fires often —
`_MAX_ARM_DISPATCHES = 2` (`catch_coordinator_node.py:144`), which accounts
exactly for the two observed dispatches per toss.

Note the interaction: **Phase 1 restores the premise.** Once the arm is gated to
after the stroke, a repack happens with the hand at rest at `x3`, prelude empty,
and the retry policy is harmless again. Phase 2 is therefore a cheap guard that
makes the premise explicit, not a redesign.

### Item 6

`JB_OP_HAND_CATCH_PRIME_REV = 9.858` but the catch trajectory starts at
**9.9594 rev** — 0.101 rev = **3.2 mm** low. So even a clean catch-from-rest
generates a non-empty prelude. Deriving the prime from the stroke geometry makes
the no-op case a true no-op. `hardware_config.yaml:497` already *documents* the
value as "top of stroke", which is the intent — it is simply 3.2 mm off it, which
is exactly the kind of hand-maintained duplicate the codegen layer exists to
eliminate.

### Latent failure this also removes

`Teensy_code.ino:533` refuses the whole command if
`now + smoothDur + SAFETY_GAP > firstMainAbs`, printing
`Not enough time for smooth-move; command ignored` **to serial only** — invisible
to ROS, and the catch silently never fires. Today the margin is 0.37 s prelude
against 0.66 s available. With the prelude empty the check becomes trivially
satisfied.

## Architecture

### The contract

> **C-HAND-1.** No hand command may create a discontinuity — in position **or in
> velocity** — between the live hand state and the newly commanded trajectory.
> Two obligations follow: a scheduled stroke is not dispatched while another
> stroke is physically executing (host-side), and a smooth-move prelude is
> continuous with the live hand velocity, not merely with its position
> (firmware-side).

The host-side half (Phases 1–2) is cheap and closes the observed defect. The
firmware half (Phase 4) closes the *class*: any hand command landing while the hand
moves — a prime, a retract ladder rung, a SAFE_ABORT — has the same failure shape.
Both halves are wanted; the host half first because it is testable without a flash.

### What must NOT change

The last-writer-wins queue is deliberately load-bearing: a pre-release
`SAFE_ABORT`'s kind-3 retract **replaces** an armed kind-0 throw stroke, and that
is the only un-arm mechanism the Teensy offers (`toss_sequencer.py` ORDERING
PRINCIPLE). Nothing in this plan may make a kind-3 refuse to clobber. Phase 2's
guard applies to *scheduled* kind-0/1/2 arms, never to the abort path.

### Build and test path for the firmware half

`Teensy_code/` is an Arduino sketch with **no `platformio.ini`** (unlike
`CatchingCone_code/` and `Teensy_code_canbridge/`), so there is no native
doctest target for `Trajectory.h` today. But the generator is **mirrored in
Python** at `sim/hand/trajectory.py`, already covered by
`tests/sim/test_hand_trajectory.py`. So Phase 4 is test-driven in the sim mirror
first, then transcribed to `Trajectory.h`, with an xref test pinning the two — the
same pattern as `tests/firmware/test_hand_traj_xref.py` uses for the 0x6D0 payload.
Phase 0 must confirm that mirror is faithful before it is trusted as the gate.

**Phase 0 found it faithful only outside the catch timeline, and found the
absolute frames unrelated by any cancelling convention.** Both constraints bind
Phase 4 — see § Phase 0 — Outcome, Confirmation 2, before writing the xref test or
converting the `[0, 11.1]` rev end-stop bound into sim mm.

## Implementation Phase Summary

| Phase | Scope | Gate | Status |
|---|---|---|---|
| 0 | Reusable stroke-timeline probe; confirm the timing window and the sim mirror | probe reproduces the measured dip from the recorded trace | **DONE** (25/25 rows ≤0.4 ms, plus a four-case synthetic post-fix branch that pins the verdict half; see Phase 0 — Outcome) |
| 1 | Gate the hand-catch arm until the throw stroke completes (item 3) | full pytest | TODO |
| 2 | Repack safety guard while a stroke is in flight (item 5) | full pytest | TODO |
| 3 | Prime rev derived from stroke geometry (item 6) | full pytest + codegen | TODO |
| 4 | Velocity-continuous prelude — sim mirror then firmware (item 4) | sim tests + xref; flash | TODO |
| 5 | Hardware validation (operator-run) | `trunc=-`, `seeds=0`, `peak <= 10.060` rev, `dip_below_x3 <= 0.10` rev; throw scatter recorded | TODO |

## Implementation Phases

### Phase 0 — Probe and characterisation

Phases 1 and 2 assert a **threshold** (`release + t_dec + margin`), so per the
repo's empirical-probe rule the recipe gets prototyped and confirmed before any
test asserts it. This one is a reusable replay harness, not a one-off, so it lives
in the repo: `tools/probes/hand_stroke_timeline.py`, with a header docstring
linking this plan and the tests that consume it, an entry in
`tools/probes/README.md`, and outputs to `temp/probes/`.

The probe takes a `toss_trace_*.jsonl` (and/or a session rosbag) and emits, per
throw: stroke start, the modelled release instant, the observed `pos_cmd`
truncation instant, the coasting peak, the dip bottom and depth in rev and mm, the
recovery instant, and the catch descent start. It must reproduce the numbers in
the Context table from `temp/logs/toss_trace_2026-07-25_15-24-25.jsonl` — that is
the phase gate.

Also in this phase, no code changes to production:

1. Confirm `t_acc`/`t_dec`/`t_acc_catch` closed forms against the shipped header
   constants (`Teensy_code/hardware_config.h` — note it contains **two**
   `HAND_SPOOL_RADIUS_M` blocks, `TeensyTraj::` at :211 and another at :286;
   `Trajectory.h` uses `TeensyTraj::`, so pin which one and say so).
2. Confirm `sim/hand/trajectory.py` reproduces `Trajectory.h`'s `x1/x2/x3/x5`,
   `t_acc/t_vel/t_dec` and `makeSmoothMove` for at least three velocities across
   the shipped flight band. If it does not, fixing the mirror is the first job of
   Phase 4 and the plan's gate assumption must be revised.
3. Confirm which firmware image owns `Trajectory.h` on the shipped path. The hand
   is driven by the Platform Teensy via the can-bridge conduit; state the chain
   explicitly (`SetHandTrajCmd` → `teensy_bridge_node` → can-bridge → 0x6D0 →
   Platform Teensy) so Phase 4 flashes the right board.
4. Confirm the dispatch accounting: `_MAX_ARM_DISPATCHES = 2` should exactly bound
   the observed two `Arming hand catch` lines per toss. If any session shows more,
   the cap is not being honoured and that is a separate defect.

**Gate:** probe reproduces the recorded dip within the telemetry's own resolution;
findings written up (they feed Phases 1–4's docstrings).

### Phase 0 — Outcome (2026-07-25, hardened 2026-07-26)

**Gate: PASS.** `tools/probes/hand_stroke_timeline.py --gate` reproduces all
**25/25** rows of the Context table from
`temp/logs/toss_trace_2026-07-25_15-24-25.jsonl`; every instant matches to
**≤ 0.4 ms** against a 20 ms tolerance (two hand-telemetry new-value periods —
measured median **10.0–10.2 ms** across all six 2026-07-25 sources). Twenty of
the rows are the Context table's own instants and positions; five more
(`x2_rev`, `x3_rev`, `shift_ms`, `n_seeds`, `dip_below_x3_rev`) pin quantities the
instant rows cannot, because a 5 % error in `x2` moves `x2_cross` by only ~2.4 ms
and a broken release fit or seed detector moves no instant at all — and all are
load-bearing downstream (`shift_ms` sizes Phase 1's margin, `n_seeds` and
`dip_below_x3_rev` are runbook ABORT rows). `--gate` then runs a **second branch**
against **four** synthetic post-fix captures (`clean`, `overshoot`,
`short-flight`, `braking-prelude`); see § second-pass re-verification and § third-pass
hardening for why that branch exists and what it caught.
Rosbag input is supported as well — it was cheap (the same two topics,
`mcap_ros2`) and it unlocks the only multi-throw session in the evidence base,
`2026-07-25_15-17-48`, for which no jsonl trace exists.

#### The baseline is wider — and worse — than the Context table

Seven distinct self-tosses across three sessions, plus two BallButler reloads
and one `ABORTED_NO_RELEASE` toss the probe correctly reports as strokeless:

| session | ball | arms | trunc (rev) | peak (rev / mm) | headroom to 11.1 rev | dip (rev / mm / % stroke) | shift (ms) |
|---|---|---|---|---|---|---|---|
| 15-04-35 | 34 | 2 | 7.1245 | 10.2611 / 324.5 | 0.839 rev | 0.641 / 20.3 / 6.4 | +12.8 |
| 15-17-48 | 10 | 1 | 6.7562 | 10.2513 / 324.2 | 0.849 rev | **2.040 / 64.5 / 20.5** | +19.0 |
| 15-17-48 | 11 | 2 | 6.1965 | 10.2684 / 324.8 | 0.832 rev | 1.656 / 52.4 / 16.6 | +20.7 |
| 15-17-48 | 13 | 1 | 7.1897 | 10.2813 / 325.2 | 0.819 rev | 1.790 / 56.6 / 18.0 | +15.4 |
| 15-17-48 | 17 | 2 | 6.8525 | **10.3248 / 326.6** | **0.775 rev** | 0.726 / 23.0 / 7.3 | +20.2 |
| 15-22-50 | 2 | 1 | 7.7825 | 10.1653 / 321.5 | 0.935 rev | 1.271 / 40.2 / 12.8 | +17.2 |
| 15-22-50 | 3 | 1 | 7.7004 | 10.1743 / 321.8 | 0.926 rev | 1.361 / 43.0 / 13.7 | +21.9 |

Three corrections to the Context section follow from that table:

1. **43 mm is the median dip, not the worst.** The worst is **64.5 mm — 20.5 %
   of the stroke**, on a toss that received a **single** arm dispatch. Dip
   magnitude does not track the dispatch count: the two smallest dips
   (20.3 mm, 23.0 mm) are both double-dispatch tosses. Phase 1 (gate the arm)
   therefore owns the operator-visible defect outright; Phase 2 is the guard
   the plan already says it is, not a second half of the same fix.
2. **0.93 rev of end-stop headroom is the best case, not the typical one.** The
   worst measured is **0.775 rev = 24.5 mm** — and that is at 3.93 m/s, in the
   middle of the flight band. The plan's note that momentum is ~1.9× at the top
   of the band starts from 24.5 mm of margin, not 29 mm.
3. **The truncation position spans 6.1965–7.7825 rev (1.586 rev = 50 mm).** Peak
   overshoot tracks it inversely — the two latest freezes (7.70, 7.78 rev)
   produce the two lowest peaks (10.174, 10.165 rev) — which is the expected
   physics: freezing earlier in the decel ramp leaves more live velocity to
   coast on. Dip depth is noisier and is not monotone in the truncation
   position, so it is the peak, not the dip, that Phase 5 should gate the
   end-stop margin on.

#### A dispatch latency the window arithmetic omits

`teensy_bridge_node.py:3652` stamps the absolute event as
`int(time.time() * 1000) + int(event_delay * 1000)` — the bridge reads its OWN
clock at handler-execution time and adds a delay the caller computed from a
clock read of its own (`reload_coordinator_node.py:1687`:
`seq.t_release - time.perf_counter() - release_latency`, and
`catch_coordinator_node.py:665`: `cmd.landing_time - current_time`). Every
microsecond of ROS service transit between those two clock reads therefore
lands the physical event **late**, and nothing compensates it
(`JB_OP_TOSS_RELEASE_LATENCY_MS` ships `0.0`).

Measured over the seven tosses, read from the **bags**: **+12.8 to +21.9 ms,
median +19.0 ms** later than the announcement's `throw_time`. The catch arm
shows the same order of lag (`catch_desc` lands ~+20 ms after
`arm_log_time + event_delay - t_acc_catch`, ±10 ms given the log's 2-decimal
`delay` print and the telemetry's sampling lag), consistent with one mechanism.

The two tosses that were *also* captured to a jsonl trace read **1.5–1.7 ms
later** through that path than through the bag — ball 34 `+14.5` vs `+12.8`,
ball 3 `+23.4` vs `+21.9` — because the two sources timestamp differently (the
bag stamps the recorder's receipt of each message; the jsonl carries the
recorder's `perf_counter` mapped onto the ROS clock by the announcement's own
row). Neither is calibrated to better than the ~5 ms sampling bias below, so
the honest statement of the worst case is **+23.4 ms**, not +21.9.

Consequence for Phase 1: a stroke-busy window computed as
`t_release_announced + t_dec(v)` **expires while the stroke is still
decelerating**, because the announcement's `throw_time` is up to 23 ms early.
The margin must cover the dispatch latency, not merely the model's `t_dec`.
`margin = 40 ms` (**1.7× the worst measured shift**, either clock path) leaves
the slack budget intact:

| flight | `t_dec` | earliest safe arm | latest usable arm | window |
|---|---|---|---|---|
| 0.80 s (`v` 3.93, armed 3.13) | 65.1 ms | release + 105 ms | release + 651 ms | **546 ms** |
| 0.55 s (`FLIGHT_TIME_MIN_S`, `v` 2.70, armed 2.16) | 94.9 ms | release + 135 ms | release + 343 ms | **208 ms** |

(latest usable arm = `event − t_acc_catch − prelude − SAFETY_GAP`, prelude empty,
`SAFETY_GAP` 20 ms.) The window narrows with flight time exactly as the plan
predicts but stays positive at the shortest shipped flight.

**Both rows are at the NOMINAL armed velocity** (`catch_coordinator_node.py:667`
scales the event velocity by `JB_OP_CATCH_VEL_SCALE_DEFAULT = 0.8`, clamped to
`[0.3, 7.0]`; the 2026-07-25 launch log confirms `vel=3.13 m/s (scale 0.80)`
against a 3.9224 m/s throw). Since `t_acc_catch = 0.404073 / v_armed`, a *low*
tracker landing-speed estimate lengthens the lead and moves the right-hand edge
earlier: at the 0.55 s flight the window closes once `v_armed < 1.02` m/s (a
tracker landing speed under ~1.28 m/s for a 2.70 m/s throw). Phase 1 step 3
therefore has to check the fit against the runtime `event_vel`, not against this
table.

One caveat on `shift`, recorded in the probe docstring so it does not rot:
`hand_telemetry` publishes a ~100 Hz snapshot of a 500 Hz frame stream, so each
sample's timestamp trails the frame it reports by 0–10 ms. During the ascent the
command moves at up to 120 rev/s, which biases `rel_fit` late by roughly half a
telemetry period (~5 ms). The bias inflates the window, i.e. it has the safe
sign for sizing a margin — but `shift` must not be quoted as a calibrated
latency without subtracting it.

#### Confirmation 1 — the closed forms (derived, not copied)

**The live block is `TeensyTraj::`** (`Teensy_code/hardware_config.h:210-226`,
`HAND_SPOOL_RADIUS_M` at `:211`).
The second block at `:285` is `BBTraj::` — a *different namespace* for the
BallButler's own hand (spool 0.0052493 m, gain 1.0, stroke 0.28 m, catch ratio
0.8), not a redefinition of the same symbol. `Trajectory.h:29-45` qualifies
every constant explicitly (`TeensyTraj::HAND_SPOOL_RADIUS_M`) and
`Teensy_code/` contains no `using namespace`, so there is no shadowing risk;
`BBTraj::` is referenced by no `.ino`/`.h`/`.cpp` in `ros_ws/src` other than its
own definition.

Derived from the shipped constants (float64):

| quantity | derived | plan | verdict |
|---|---|---|---|
| `LINEAR_GAIN` | 31.617152802 rev/m | 31.6172 | agrees |
| `totalStroke` | 0.315 m | 0.315 | agrees |
| `x1` | 0.171293646 m = 5.415817 rev | (not quoted) | — |
| `x2` | 0.187043646 m = 5.913788 rev | 0.187044 / 5.914 | agrees |
| `x3` | 0.315 m = 9.959403 rev | 9.9594 | agrees |
| `x5` | 0.193778191 m = 6.126715 rev | 0.19378 / 6.127 | agrees |
| `t_acc` coefficient | **0.342587293** / v | 0.342578 / v | **plan low by 9.3e-6 (2.7e-5 rel)** |
| `t_dec` | `t_acc · INERTIA_RATIO` = 0.747·`t_acc` | 0.747·`t_acc` | agrees exactly |
| `t_acc_catch` coefficient | **0.404072696** / v_cmd | 0.404133 / v_cmd | **plan high by 6.0e-5 (1.5e-4 rel)** |

Both discrepancies are plan-side rounding/transposition (0.342587 → 0.342578),
not code defects — neither changes any conclusion (`t_acc` shifts by 2.4 µs at
3.93 m/s). Phases 1–4 must derive from the header constants rather than copying
the plan's literals. Note `t_acc_catch`'s denominator: the coefficient above is
per **commanded** velocity, i.e. the `event_vel` on the wire; per the hand's
actual catch speed `|vC| = 0.6·v` it is 0.242443.

`x3 = accelSt + velHold = totalStroke` holds *algebraically*, independent of
velocity — confirmed symbolically, not just numerically: the decel segment
contributes `0.5·INERTIA_RATIO·v·t_acc`, so
`x3 = 0.5·v·t_acc·(1 + INERTIA_RATIO) = accelSt`, plus `velHold`. So the throw's
end position and the catch's start position coincide by construction, as the
plan states.

#### Confirmation 2 — the sim mirror: faithful where Phase 4 needs it, two catch-timeline divergences, and an absolute frame that does NOT cancel

Method: `Trajectory.h` transcribed 1:1 in numpy **float32** (matching the
firmware's `float`) and compared against `sim/hand/trajectory.py` at three
velocities spanning the shipped flight band (`T` 0.55 / 0.80 / 1.10 s ⇒ `v`
2.697 / 3.931 / 5.393 m/s), plus four `makeSmoothMove` cases covering the moves
that actually occur (prime-to-bottom, the observed 7.7004 → 10.0543 repack, the
catch descent, and a short brake from the coasting peak).

**Faithful, to float32 round-off:** `x1`/`x2`/`x3`/`x5`/`x6`, throw
`t_acc`/`t_vel`/`t_dec` and `start_time`, catch `t_acc`/`t_vel`/`t_dec`, and
`makeSmoothMove`'s duration `T` and mid-move position. **The normative bounds are
the second pass's** — see § Second-pass re-verification: ≤4.4e-8 m on positions,
≤3.5e-8 s on segment durations, ≤3.2e-8 s on `T`, ≤6.3e-7 rev on the mid-move
position. (The first pass quoted 8.5e-8 m / 1.3e-8 s / 1.0e-7 s / 1.0e-5 rev;
those are superseded and must **not** be used to size Phase 4's xref tolerance —
they are up to 16× looser, so a genuine few-1e-6-rev mirror drift introduced when
`makeSmoothMove` is rewritten for `v0 ≠ 0` would pass a test written to them.)

**The absolute position frames are NOT related by a cancelling convention, and
this matters for Phase 4's end-stop assertion.** An earlier draft of this Outcome
said the +20 mm gap was "documented and intentional — the sim measures from the
physical bottom, the firmware from the encoder zero at the bottom of the
effective stroke". The second clause is **wrong**, and three shipped facts refute
it:

- `Teensy_code/hardware_config.h:99/102` — `Homing::HAND_SPEED_RPS = -3.0f`
  (the hand homes **downward**, into the bottom hard stop) and
  `Homing::HAND_ABS_POS_REV = -0.1f`. So encoder zero sits **0.1 rev = 3.16 mm
  above the physical bottom**, not 20 mm.
- `hand_motor_max_position_revs: 11.1` (`config/hardware_config.yaml:384`) =
  351.08 mm above encoder zero = **354.24 mm above the physical bottom**, i.e.
  0.76 mm below the top of the 355 mm stroke (`:332`, "used to detect
  overextension"). That guard is only coherent if zero is the physical bottom.
  Were zero 20 mm up, the shipped guard would sit **16 mm past the hard stop**.
- `generate_config.py:585` — `HAND_CATCH_OFFSET_MM = hand_axis_bottom_offset +
  x5·1000` with **no margin term**, so the host codegen also treats the firmware's
  `x` as measured from the physical bottom.

So: the firmware's `x = 0` is the homed physical bottom (±3.16 mm) and its stroke
occupies 0…9.9594 rev = 0…315 mm with 40 mm of unused travel **above**. The sim
instead *centres* the 315 mm stroke in the 355 mm travel — `sim/hand/trajectory.py`
defaults the throw start to `STROKE_MARGIN_MM` and the catch start to
`STROKE_MARGIN_M + TOTAL_STROKE_M`, i.e. 20…335 mm, against
`sim/plant/mujoco_plant.py:377`'s `command_hand(pos_mm)` = "mm from bottom of
travel", clipped `[0, 355]`. That 20 mm inset is a **sim-side placement choice**,
not a mirror of the firmware.

**Constraint on Phase 4, therefore:** the only valid mapping is
`rev = sim_mm/1000 × LINEAR_GAIN` with **no margin term**, and the end-stop bound
`[0, GEOM_HAND_MOTOR_MAX_POSITION_REVS = 11.1]` rev is `[0, 351.08]` mm measured
from the firmware's zero — *not* `[20, 371.08]` mm. Carrying the +20 mm across
makes the ceiling **20 mm = 0.63 rev too permissive**: the physical point it then
names sits 16.8 mm (**0.532 rev**) past the 11.1 rev overextension guard — which
itself is only 0.76 mm below the top of the 355 mm stroke — and 16.1 mm above the
top of the sim's own modelled travel. On a system that already reached 10.325 rev
with 0.775 rev of headroom, that is the difference between a sweep test that
bounds the excursion and one that waves it through before the profile is
transcribed to `Trajectory.h` and flashed. Phase 4 must carry that conversion at
**one** enforcement point, not inline per assertion.

Recorded as a **second** candidate contributor to the known sim-catch fidelity gap
(`project_hand_catch_hardware_smooth_sim_janky`), alongside the 15.75 mm timeline
offset below: `sim/input/scripted.py:315` computes
`_HAND_CATCH_X5_PHYSICAL_MM = STROKE_MARGIN_M·1000 + x5·1000` = 213.78 mm, while
the host's `HAND_CATCH_OFFSET_MM` derivation uses `x5·1000` = 193.78 mm from the
same physical bottom. The sim therefore catches **20 mm higher** than the host
stack believes it does. Displacements still agree exactly; it is the absolute
placement that differs.

**Two divergences, both in the catch timeline.** The first is the TIME ORIGIN.
`makeCatch` shifts by
`-(t5 - t4) = -t_acc` (`Trajectory.h:78`), and since `t6 = t5 + t_vel` the
velocity hold spans `[t5, t6]`; so the firmware's kind-1 `t = 0` is the
**start** of the catch velocity hold, at position `x5`. `calcCatch`'s comment at
`:131` calls `t5` the "centre of vel hold" and `makeCatch`'s at `:78` says "t=0
at mid velocity-hold"; both are wrong about the code — `t5 = t2 + airT − 0.5·t_vel`
means the *arrival* `t2 + airT` is the hold's centre, and `t5` is its start.
`sim/hand/trajectory.py:127` sets `_t_offset = t_acc + 0.5·t_vel`, putting its
`t = 0` at the hold **centre**. Measured divergence:

| `v` | firmware `start_time` | sim `start_time` | Δt | Δpos at own t=0 |
|---|---|---|---|---|
| 2.697 | −0.1498424 s | −0.1595768 s | −9.73 ms | 0.498 rev = 15.75 mm |
| 3.931 | −0.1027960 s | −0.1094740 s | −6.68 ms | 0.498 rev = 15.75 mm |
| 5.393 | −0.0749213 s | −0.0797884 s | −4.87 ms | 0.498 rev = 15.75 mm |

(the position offset is `0.5·velH` and so is velocity-independent).

The **second** divergence is the catch `end_time`, and it is benign: the sim's
`_t_end` adds `END_PROFILE_HOLD_S` (`sim/hand/trajectory.py:143-144`) whereas
`buildCatch`'s `buildSegment(t4, tA, …)` has `tA[3] = t7` and so emits nothing
past `t7` (`Trajectory.h:176-184`); `t8 = t7 + END_PROFILE_HOLD` exists only in
`buildCommand`/kind-2, which *does* honour it. Measured gap **+90.3 / +93.3 /
+95.1 ms** at flights 0.55 / 0.80 / 1.10 s — exactly
`END_PROFILE_HOLD_S − 0.5·t_vel_catch`. Both models hold the same final position,
so nothing physical differs; it is recorded here only so a Phase-4 xref test does
not assert catch `end_time` parity and lose a debug cycle to a 90 ms mismatch
that is not a defect.

**This does not make Phase 4's gate illusory, and the plan's gate assumption
stands** — with one scope constraint. Phase 4 changes `makeSmoothMove` and its
tests assert the prelude's peak-acceleration bound across a `v0` sweep, the
bounded overshoot inside `[0, 11.1]` rev, and the at-target-but-moving braking
case. Every one of those rests on `HandSmoothMove`, `x3` (the throw end / catch
start) and the stroke limit — all confirmed faithful above. The catch time
origin enters none of them. So the constraint is on step 4: the xref test must
pin the **prelude and the stroke geometry**, and must *not* assert catch
timeline parity until the origin question below is settled, or it will fail for
a reason unrelated to Phase 4.

The host stack is self-consistent with the firmware's convention —
`generate_config.py:581` derives `HAND_CATCH_POS_M = x5`, `:585` turns it into
`HAND_CATCH_OFFSET_MM = hand_axis_bottom_offset + x5·1000 = 64.78 mm`, and that
is what the reload and toss coordinators use as the ball's catch height
(`sim/input/scripted.py:316` mirrors the same x5 in the sim's own frame) — so
the sim mirror's *timeline* is the outlier, not the firmware.
(`motion/trajectory/toss_release.py:45` only *mentions* `HAND_CATCH_POS_M` in a
comment; what it consumes is the throw sibling `HAND_THROW_POS_M` = `x2`.) **Whether the hold should be
centred on the predicted arrival (which `calcCatch`'s own construction implies,
and which `makeFull`/kind-2 does honour) is an open question for the catch path,
out of scope for this plan.** It is worth recording against the known
sim-fidelity gap on catches (`project_hand_catch_hardware_smooth_sim_janky`):
a 15.75 mm / 5–10 ms sim-vs-firmware offset in where the hand is at ball
arrival is a candidate contributor.

One cosmetic divergence, recorded so it is not mistaken for a bug later:
`makeSmoothMove`'s already-there dead-band is `|Δ| < 1e-6` rev = 3.16e-5 mm;
the sim's is `|Δ| < 1e-3` mm = 3.16e-5 rev, i.e. **31.6× larger in rev**. Both
are orders of magnitude below the encoder resolution, so nothing reachable
distinguishes them.

#### Confirmation 3 — firmware ownership

`Trajectory.h` is `#include`d by exactly one translation unit:
`ros_ws/src/jugglebot/Teensy_code/Teensy_code.ino:24` — the **Platform Teensy**
sketch. (`CatchingCone_code/` and `Teensy_code_canbridge/` are the other two
sketches; neither includes it. The three archived/`toss_release` mentions of
"Trajectory.h" are prose references, not includes.) Verified from the code, the
shipped chain is:

`catch_coordinator_node._arm_hand_catch` → `SetHandTrajCmd` service →
`teensy_bridge_node._svc_set_hand_traj` (`:3633`, validates then stamps
`wall_time_ms`) → `rpc_args.encode_hand_traj_cmd` (8-byte payload) →
`teensy_hand_traj_cmd` RPC → **can-bridge Teensy** (sends the CLOSED_LOOP +
POSITION/PASSTHROUGH preamble, then forwards the payload verbatim on the
firmware-owned **0x6D0** id) → **Platform Teensy** `Teensy_code.ino:458`
(`CMD_TRAJ_ID`, len 8) → `HandTrajGenerator` / `makeSmoothMove` in
`Trajectory.h`.

**Phase 4 flashes the Platform Teensy, not the can-bridge.** The can-bridge is
a byte-transparent forwarder for 0x6D0 and needs no change (its payload parity
is already pinned by `tests/firmware/test_hand_traj_xref.py`).

#### Confirmation 4 — dispatch accounting: the cap is honoured

`_MAX_ARM_DISPATCHES = 2` (`catch_coordinator_node.py:144`) bounds the observed
dispatches **exactly** — no session shows a third. Counted from
`~/.ros/log/<stamp>/launch.log`, because the three 2026-07-25 bags were recorded
without `/rosout` (verified from their mcap channel summaries) — not because bags
cannot carry it: the runbook's § Recording command does record it, and both of the
probe's input paths read the channel when it is present:

| session | ball | thrower | arms | telemetry seeds | note |
|---|---|---|---|---|---|
| 15-04-35 | 34 | jugglebot | 2 | **2** | |
| 15-17-48 | 4 | ball_butler | 2 | n/a | reload: hand parked, no stroke to clobber |
| 15-17-48 | 5 | ball_butler | 2 | n/a | reload |
| 15-17-48 | 10 | jugglebot | 1 | **1** | |
| 15-17-48 | 11 | jugglebot | 2 | **2** | the toss the Context section cites |
| 15-17-48 | 13 | jugglebot | 1 | **1** | |
| 15-17-48 | 14 | jugglebot | 1 | 0 | `ABORTED_NO_RELEASE` — dispatch eaten, no stroke |
| 15-17-48 | 17 | jugglebot | 2 | **2** | |
| 15-22-50 | 2 | jugglebot | 1 | **1** | |
| 15-22-50 | 3 | jugglebot | 1 | **1** | the Context table's toss |

Ten tosses, fifteen dispatches, maximum two per ball. **No separate defect.**

The `telemetry seeds` column is an independent instrument, not a restatement of
the log: the probe counts from-rest quintic seeds in `hand_telemetry`, and it
agrees with the log's arm count on **7/7** tosses that had a stroke (the seven
`jugglebot` rows with a numeric seed count). The eighth numeric row — ball 14 —
reads log 1 / seeds 0 and is the `ABORTED_NO_RELEASE` case: the dispatch was
logged, no stroke followed, so a 0 there is the *expected* disagreement, not a
counter-example. On the three double-dispatch tosses the seeds are
**20 / 9 / 17 ms** apart (balls 34 / 11 / 17) against log spacings of
**17.9 / 10.3 / 17.7 ms** — matching to within one telemetry period. Every seed
sits **0.0000 rev** (worst 0.0001) from the live `pos_meas` of the preceding
samples, which is the direct fingerprint of `makeSmoothMove` reading
`current_hand_position`.

Two of those seven are corroborated a third time, from a source with its own
`/rosout`: the jsonl traces `15-06-38` (ball 34, `arms=2`, 2 seeds) and
`15-24-25` (ball 3, `arms=1`, 1 seed) agree with both the launch log and the
bag reading of the same physical toss.

Two refinements to the plan's narrative:

- The Context section's "the two observed dispatches per toss" holds for the
  session it was measured in, but **five of ten tosses dispatched only once** —
  including the toss the Context table itself measures. The dip is caused by
  *any* kind-0/1/2 command landing mid-stroke, not by the retry.
- Phase 2's stated limitation ("an armed stroke produces no observable until its
  event time") is true of **arming**, but a repack that clobbers a **live**
  stroke is directly observable — the seed jump above. So Phase 2's guard *can*
  be validated against a capture, even though the arm itself cannot. The
  protocol-level "armed stroke" telemetry field remains the right follow-up for
  verifying arms; it is not needed to verify the guard.

#### Second-pass re-verification (same day, independent re-derivation)

Phase 0 was re-run by a second implementer before any of Phases 1–4 consumed
its numbers. The re-run did **not** re-execute the probe's own model and call
that agreement: `Trajectory.h` and `hardware_config.h` were transcribed afresh
(float64 for the closed forms, numpy `float32` for the mirror comparison, to
match the firmware's `float`) and compared against both the probe and
`sim/hand/trajectory.py`.

Everything above reproduced. `LINEAR_GAIN` 31.617152802, `t_acc` coefficient
0.342587293, `t_acc_catch` 0.404072696 per commanded velocity / 0.242443618 per
hand velocity, `x1` 5.415817 / `x2` 5.913788 / `x3` 9.959403 / `x5` 6.126715
rev, `x3 == totalStroke` to 1.1e-16 m across `v` ∈ [0.5, 7.0]; the mirror
faithful to ≤4.4e-8 m on positions, ≤3.5e-8 s on segment durations, ≤3.2e-8 s
on `makeSmoothMove`'s duration and ≤6.3e-7 rev on its mid-move position; the
catch time origin diverging by −9.73 / −6.69 / −4.87 ms with a
velocity-independent 0.498 rev = 15.75 mm position offset. Every `file:line`
cited in this Outcome was re-checked against the tree, and every row of the
baseline and dispatch tables was re-measured from the bags and launch logs.

The re-run found three defects **in the instrument itself**, all now fixed:

1. **`--all` aborted the whole sweep on one corrupt bag.** A session killed
   mid-write leaves a truncated `.mcap`; the reader raises
   `mcap.exceptions.RecordLengthLimitExceeded`, which is not an `OSError`, so
   the per-input guard in `main()` did not catch it. Because discovery is
   date-sorted, the traceback landed on `2026-06-08_22-25-46` and the sweep died
   **before reaching any 2026-07-25 session** — the instrument analysed none of
   the sessions this plan is about while exiting with a traceback rather than a
   verdict. The guard now catches per input (with `SystemExit` still named,
   since `load_bag` raises it for a missing `mcap_ros2`), reports the bad input,
   and continues.
2. **A jsonl trace with no `/rosout` reported `arms=0` rather than `?`.** The
   bag path already refuses to collapse "channel absent" into "genuinely none";
   the trace path did not, so the committed gate fixture read `arms=0` on a
   toss that really got one dispatch. `load_trace` now returns `arms=None` when
   the trace carries no `/rosout` row at all, and keeps a true 0 when it does.
3. **The gate reference had no committed recipe.** The fixture is committed
   because the recording it is cut from lives under gitignored `temp/`, but a
   fixture nobody can regenerate is only half a live reference. The cut is now
   executable — `--emit-gate-fixture <out>` with `--trace <full>` — and its
   output reproduces the previous fixture's row set exactly (1 anchor +
   1 announcement + 278 `hand_telemetry`), plus the 5 `/rosout` rows that make
   the fixture reproduce the `arms` column too. Gate on the regenerated
   fixture at the time: 24/24, deltas identical to the full trace. (Re-checked
   after the third pass added the `dip_below_x3_rev` row: `--emit-gate-fixture`
   still regenerates the committed file **byte-identically**, 285 of 5764 rows,
   and the gate forced onto the fixture alone — the fresh-clone path — returns
   25/25 with `arms=1` and the same deltas as the full trace.)

One line-reference note, recorded rather than edited into the operator-reviewed
Context section: the code block quoted there as `Teensy_code.ino:538-547` is
really two places — the prelude is built at `:522`
(`makeSmoothMove(activeTraj.x.front())`) and the queue clear plus both
`packTrajectory` calls are at `:539-546`. The `:533` time-budget refusal, the
`:467-479` kind-3 path, `Trajectory.h:47/78/101/131/242/257`, and
`catch_coordinator_node.py:144` are all exact.

#### Third-pass hardening — the instrument's POST-fix half

An adversarial review panel then read the tree with three independent lenses. The
finding that survived verification from **all three** was the same one, and it is
the defect class that matters most for a phase whose deliverable is an
*instrument*: **every gated criterion had been validated against the broken shape
only.** An instrument validated on one side scores a working fix as FAILED at the
bench, routes correct work back for rework, and burns a powered sitting — worse
than having no instrument.

Verified by running the probe's own model of the fixed shape (2026-07-26): the
`clean` synthetic — the probe's shipped model of a *perfect* post-fix capture —
printed `dip_bottom 9.9594 rev depth 0.020 rev = 0.6 mm` and `pullback −0.3 rev/s`,
while the runbook said PASS was `dip_bottom = -` / ABORT on "any dip printed", and
gave `pullback` two contradictory criteria in the same row. Root cause: `dip` is
**peak-minus-bottom**, so it is non-zero on any capture that overshoots and
settles — including the bounded overshoot Phase 4 step 2 makes the *expected*
behaviour (20.2 mm at a 10.60 rev coast, which is larger than the smallest pre-fix
defect at 20.3 mm). The criterion was an artefact of the instrument's control flow,
not a measured threshold.

Fixed at the enforcement point rather than in four places of prose. The probe now
measures **`dip_below_x3`** — how far under the stroke end the hand ended up — which
is the physically-meaningful discriminator: pre-fix the position loop yanks the
hand **0.339–1.748 rev = 10.7–55.3 mm below x3** (re-measured from all three bags),
whereas a healthy stroke settles *onto* x3 from above and never goes under (the
four post-fix synthetics read 0.000–0.001 rev). Band `<= 0.10` rev = 3.2 mm, the
same allowance the plan already grants for overshoot *above* x3, so the criterion
is symmetric about the stroke end rather than a free parameter. Margin: ~100× on
the healthy side, **3.4× on the tightest pre-fix defect** (ball 34, 0.339 rev) —
stated in the runbook so a post-fix reading near 0.3 rev is debriefed, not waved
through. The per-toss `dip_below_x3` and `pullback` columns live in
`tests/hardware/session_anomaly_fixes.md` § Section HAND's baseline table, next to
the `trunc`/`peak`/`dip` columns the § baseline table above already carries. `pullback` is now bounded (ABORT below −5.0 rev/s; pre-fix −17.9 to
−42.4) and explicitly *conditional on `peak` having passed*, because a healthy
settle from the coasting peak is genuinely negative and its magnitude is set by
the peak: −0.31 rev/s at 0.02 rev of overshoot, −1.58 at the runbook's 10.060 rev
ceiling, −10.03 at 10.60 rev.

Two more instrument defects were verified and fixed in the same pass:

1. **`catch_desc` was any downward command, so a braking prelude spoofed it.**
   `catch_desc` bounds *both* the truncation scan and the peak/pullback/dip
   window, and the old predicate was "first sample with commanded velocity below
   −1 rev/s". A `makeSmoothMove` **brake** from the coasting peak back to x3 is
   also downward — and plan **Phase 4 step 3 specifically charters that brake**
   for the at-target-but-moving case, so Phase 4's own deliverable guarantees the
   spoofing frame exists on every capture the probe must score. `catch_desc` now
   requires the commanded position to actually travel `0.5` rev below x3 (the
   armed catch runs x3→x5 = 3.83 rev; the largest observed coast is 0.37 rev, so
   0.5 separates them 7× on one side), then walks back to the onset of that
   descent's negative-velocity run — anchored on measured geometry, not tuned:
   `x3 → x5` is 3.833 rev while the largest coast above `x3` across the seven
   observed tosses is 0.365 rev, so 0.5 rev sits **7.7×** clear of the descent it
   must accept and **1.4×** clear of the largest brake it must reject. The brake is
   reported separately as `first_neg_cmd`, so it is *visible* rather than shadowing
   the descent it is not. Mutation-verified: with the old predicate the new
   `braking-prelude` gate case fails with `catch_desc` **567 ms early** (at the
   brake, release+130 ms), the peak/pullback/dip window collapsed from ~600 ms to
   ~130 ms, and the end-stop `peak` row reading **10.1298 rev against a real
   10.1588**.
2. **The fixed-shape gate ran only at `tof = 0.80`**, where `_DIP_WINDOW_S = 0.6`
   binds before `catch_desc` — so neither `catch_desc` cap was individually
   load-bearing. A `short-flight` case at `FLIGHT_TIME_MIN_S = 0.55` was added,
   where the descent begins inside the window; mutation-verified that deleting
   the dip-window cap now fails (the clean short-flight capture reports 3.83 rev
   of dip instead of 0.001). The **truncation-scan** cap remains unpinned and is
   recorded as such honestly: the 50 ms margin binds ahead of it in every
   physically reachable case, so it is belt-and-braces rather than load-bearing,
   and no synthetic was contorted to pretend otherwise.

Plus two operator-facing corrections and one hygiene fix:

- **`/rosout` is now detected, not assumed.** `load_bag` hardcoded `arms=None`
  with the note "bags carry no /rosout", and the docstring, the README and
  Confirmation 4 all generalised it. But that is a property of the *recording*:
  the three 2026-07-25 evidence bags genuinely lack the channel (verified from
  their mcap summaries), while the runbook's own § Recording command records it.
  So an operator following the runbook would have had the arm count printed as
  `?` with a note telling them the channel does not exist. Both input paths now
  read `/rosout` when present and report unavailable only when it is absent.
- **The recorder env in the runbook was wrong.** Section HAND told the operator to
  `source` the venv before `toss_trace_recorder.py record`, while the recorder's
  own docstring and `session_phase8_toss_trace.md:100` both say system `python3`
  with the ROS env and **NOT** the venv. Two runbooks in the same directory
  contradicting each other on the one mandatory capture step costs a sitting
  eventually; Section HAND now matches the recorder (and notes that the *probe* is
  the opposite — it needs the venv).
- **`--json` wrote a fixed filename**, so running the verdict command after
  `CHECK HAND-1` and again after `CHECK HAND-4` in one sitting destroyed the first
  analysis. Now timestamped, like both sibling bag probes.

Findings verified and deliberately **not** acted on: `smooth_move_duration_s` and
`StrokeModel` are a third host-side copy of the stroke model, which Phase 1 step 2
already mandates collapsing into a shared `motion/` helper — the probe now carries
a note at the formula saying it is the pre-Phase-4 rest-to-rest form and must move
with it, rather than pre-empting Phase 1's design. And the conventions this probe
deviates from (`tools/README.md`'s matplotlib `--preview` rule, `tools/probes/`'s
"never write inside `tools/`") were scoped **in the convention documents first**,
per the repo's own contract rule, rather than left as a carve-out asserted inside
the deviating file.

#### Findings handed forward (not fixed here)

- **`makeSmoothMove`'s early return can defeat a kind-3 clobber.**
  `Teensy_code.ino:472-475` returns *before* `packedMsgs.clear()` when the
  smooth move is empty, so a kind-3 retract whose target is within 1e-6 rev of
  the live position does **not** clear an armed stroke — and kind-3 is the only
  un-arm mechanism the Teensy offers. The window is 3.16e-5 mm wide against a
  noisy float encoder estimate, so this is latent rather than observed. Phase 4
  step 3 closes it as a side effect (at-target-but-moving must brake, not return
  empty); the at-target-**and**-at-rest case needs the clear moved ahead of the
  early return, which is a change to the clobber path and therefore an operator
  decision, not an implementer's.
- The Context section's `HAND_CATCH_POS_M`/`x5` row is correct, but the
  catch-timeline origin question above should be resolved before anyone treats
  `sim/hand/trajectory.py`'s catch timing as ground truth.

#### Phase 0 close-out

**Landed as** `cabf3c6` (probe + docs) on `mvp-trajectory-bringup`, with the SHA
backfilled by the immediate follow-up commit. **Logbook:**
`logbook/2026-07-26-hand-stroke-timeline-probe.md`.

**Test gate.** `pytest tests/ -q` (run 2026-07-26, on the Jetson): **3429 passed,
3 xfailed, 198 warnings in 1323.44 s (0:22:03)** — identical pass/xfail counts to
the pre-change baseline at HEAD `cf58157` (3429 passed, 3 xfailed in ~1331–1341 s),
so **no net count change** and the xfail count holds at 3. No test file
was added, modified, weakened, xfailed or skipped by this phase — the deliverable
is an offline probe that no test imports, plus markdown. (Mechanically:
`pytest.ini` sets `testpaths = ["tests"]`, so `tools/` is never collected, and
`grep -rIl hand_stroke_timeline tests/` returns exactly one file —
`session_anomaly_fixes.md`, this runbook — with **zero** `.py` hits.) Instrument
self-checks, same date:
`--gate` → `GATE PASS — 25/25 rows within tolerance` + `GATE PASS — fixed-shape
branch`, exit 0; the gate forced onto the committed fixture alone (the fresh-clone
path) → identical deltas with `arms=1`; `--emit-gate-fixture` regenerates that
fixture byte-identically (285 of 5764 rows); the three fix-relevant mutations are
each caught by the fixed-shape branch (see the logbook's Outcome table).

**Deployment: none.** Nothing under `ros_ws/src/` or `config/` changed, so **no
colcon build, no relaunch, and no codegen run**; the probe is offline and read-only,
so **no flash**. For later phases: Phases 1–3 are `ros_ws` + config ⇒ colcon build
**+ relaunch** (the launch runs the installed copy); Phase 4 is firmware ⇒ a
**Platform Teensy flash** (not the can-bridge).

**Operator handoff — deferred, nothing actuating in this phase.**
`tests/hardware/session_anomaly_fixes.md` § Section HAND carries the capture
requirement (trace recorder mandatory, system `python3` **not** the venv), the
offline instrument self-check, the verdict command, the **seven ordered numeric
PASS/ABORT rows**, and the pre-fix baseline every post-fix row is scored against.
Phases 1, 2 and 4 append their own `CHECK HAND-n` bodies under the same header.
Two decisions are the operator's, not an implementer's: whether kind-1 should
centre the catch velocity hold on the predicted arrival (§ Confirmation 2), and
whether to move `packedMsgs.clear()` ahead of the kind-3 early return
(§ Findings handed forward) — that one changes the only un-arm mechanism the
Teensy offers.

### Phase 1 — Gate the hand-catch arm until the throw stroke completes

The single highest-value change in this plan: it removes the dip **and** the
throw-truncation jitter, and it makes Phase 2's premise true again.

1. `catch_coordinator_node` learns a *stroke-busy* window. While our own toss is
   live (it already tracks `_pretilt_hold` / `_announcement_seen`; the toss also
   knows `t_release`), suppress the hand-catch arm until
   `t_release + t_dec(v_throw) + margin`.
2. Where the window comes from matters. The coordinator does not currently know
   the throw's `event_vel`, and `t_dec` depends on it. Options, to be decided in
   this phase and recorded:
   - the toss coordinator publishes the release instant and stroke-end instant
     alongside the announcement (most explicit, adds a field/topic);
   - the catch coordinator derives `t_dec` from the announcement's
     `initial_velocity` (no new wire, but duplicates the stroke model host-side);
   - a fixed conservative delay (simplest, but a magic number that silently
     mis-sizes at the ends of the flight band — prefer not to).
   Recommendation: derive from the announcement, using **one shared helper** in
   `motion/` so the stroke model is not copied a third time (it already exists in
   `Trajectory.h` and `sim/hand/trajectory.py`).
3. Assert the window fits: the arm must still land before
   `event − t_acc_catch − prelude − SAFETY_GAP`. With the prelude empty this is
   ~600 ms of slack at an 0.8 s flight, but it **shrinks with flight time** — check
   it at `FLIGHT_TIME_MIN_S = 0.55` and log loudly if the window would close.
   Phase 0 measured both ends: 546 ms of window at 0.80 s and 208 ms at 0.55 s,
   with a 40 ms margin sized from the measured dispatch latency. The margin is
   **not** optional slack — the announcement's `throw_time` is up to **23.4 ms**
   earlier than the physical release, so `t_release + t_dec` alone expires
   mid-ramp. See § Phase 0 — Outcome.
   **Evaluate the fit against the RUNTIME `event_vel` at dispatch, not a nominal.**
   Phase 0's window table is computed at the nominal armed velocity
   (`JB_OP_CATCH_VEL_SCALE_DEFAULT = 0.8` × the throw speed), but the catch arm's
   own `event_vel` comes from the tracker and `t_acc_catch = 0.404073 / v_armed`,
   so a low landing-speed estimate *lengthens* the required lead and moves the
   window's right-hand edge earlier. At the 0.55 s flight the window closes once
   `v_armed < 1.02` m/s. Log loudly and dispatch immediately when it would close —
   an arm that lands after `event − t_acc_catch − SAFETY_GAP` is refused wholesale
   by `Teensy_code.ino:533-535` with `Not enough time for smooth-move` printed to
   **serial only** (`:534`), so the catch silently never fires with no ROS-visible
   signal.
   Surfacing that refusal over the wire is a worthwhile follow-up.
4. Do not touch the reload path's timing: there is no throw stroke there, so the
   window must be inert when the throw is not ours.

**Files:** `catch_coordinator_node.py`, `motion/` stroke-model helper (new),
`tests/ros/test_catch_coordinator_node.py`, `tests/ros/test_catch_coordinator.py`,
a new `tests/motion/` case for the helper.
**Gate:** `pytest tests/ -q` green. Tests: the arm is withheld until the stroke-end
instant; it is dispatched immediately after; the window is inert for a reload; the
0.55 s-flight case still leaves positive slack.
**Deployment:** `colcon build --packages-select jugglebot` + relaunch.

### Phase 2 — Repack safety guard

Make C-HAND-1's host-side half explicit rather than relying on Phase 1's timing.

1. In `_on_hand_traj_done`'s retry path, refuse the re-arm while a stroke is
   physically in flight (same stroke-busy predicate as Phase 1). Defer the retry to
   the first tick after the stroke ends rather than dropping it.
2. Preserve `_MAX_ARM_DISPATCHES` and the keep-the-latch-after-the-cap behaviour —
   they are correct and hard-won.
3. Update the `_on_hand_traj_done` docstring: its "further repacks just churn"
   claim is only true with the hand at rest, and that is now an enforced
   precondition rather than an assumption. Leaving a false premise in a comment is
   how the next person re-introduces this.
4. Explicitly exempt the abort path: a kind-3 retract must still clobber.

An honest limitation to state in the phase notes: an armed stroke produces **no
observable** until its event time, so the arm cannot be telemetry-verified the way
the hand ladders were (`4e33b53`). A Teensy-side "armed stroke" field in
`hand_telemetry` or `link_status` would make it verifiable; that is a protocol
change and is **out of scope** here — record it as a follow-up rather than
pretending the ack problem is solved.

**Files:** `catch_coordinator_node.py`, tests as above.
**Gate:** `pytest tests/ -q` green; a test that a failed ack during the stroke
defers rather than repacks, and that the abort path is unaffected.

### Phase 3 — Prime rev derived from stroke geometry

`hand_catch_prime_rev` should not be an independently-tunable literal: it must
equal the catch trajectory's first sample or the prelude is never empty.

1. Derive it in `config/generate_config.py` from the same inputs the firmware uses:
   `(HAND_STROKE_M − 2·STROKE_MARGIN_M) · LINEAR_GAIN_FACTOR / (2π ·
   HAND_SPOOL_RADIUS_M)` = 9.9594 rev for the shipped values.
2. Keep the YAML key as an explicit override with a comment saying what it must
   equal and why; add a drift-guard test asserting the config value equals the
   derived value (the pattern `toss_sequencer`'s constants already use).
3. Order of operations: edit YAML → `python config/generate_config.py` → stage the
   regenerated artefacts → `pytest tests/ -q` → commit.

**Risk to check before changing the number:** the prime position is also the
reference for the reload path's parked-top detection and park-band qualifiers
(`_hand_dispatch_confirmed`, the park-band constants, and `toss_sequencer`'s
`hand_parked` bottom band). Moving the prime 3.2 mm up must not fall outside any
of those windows. Grep every consumer of `JB_OP_HAND_CATCH_PRIME_REV` and each
park/parked-top threshold, list them, and check each — a silent
`REJECTED_HAND_NOT_PARKED` or a missed parked-top would be a nasty regression for
a 3.2 mm cosmetic win. If any window is tight, widen it deliberately in the same
commit rather than shaving the derivation.

**Files:** `config/hardware_config.yaml`, regenerated `config/generated/*` and the
consumer copies, a drift-guard test.
**Gate:** `pytest tests/ -q` green; regenerated artefacts staged.

### Phase 4 — Velocity-continuous prelude

The class fix. Firmware, so it is last and gated hardest.

1. **Sim mirror first.** Implement in `sim/hand/trajectory.py` and drive it from
   tests before touching `Trajectory.h`.
2. Replace the rest-to-rest quintic with one that starts at the live
   `(position, velocity)`: `v0 = current_hand_velocity`, `a0 = 0`, `v1 = a1 = 0`.
   Two things the current duration formula cannot survive:
   - `T = sqrt(|Δ| · QUINTIC_S2_MAX / A_max)` assumes `v0 = 0`. With `v0 ≠ 0` the
     peak acceleration is no longer `|Δ|·s''_max/T²`; derive a correct bound (or
     iterate on the analytic peak) and pin it with a test that sweeps `v0`.
   - the trajectory will legitimately **overshoot** the target and come back when
     `v0` points away from it. That is correct and must be allowed — but it must
     be clamped against the stroke limits: assert the planned excursion stays
     inside `[0, GEOM_HAND_MOTOR_MAX_POSITION_REVS = 11.1]` with margin, and
     decide what to do when it cannot (refuse loudly, or brake to the limit).
     The un-fixed system already reached 10.174 rev, so this is a live concern.
     **Convert the bound with `rev = sim_mm/1000 × LINEAR_GAIN` and no margin
     term** — the sim's 20 mm stroke inset is a sim-side placement, not a frame
     offset the firmware shares, so adding it makes the ceiling 20 mm = 0.63 rev
     too permissive (0.532 rev past the overextension guard, which is itself only
     0.76 mm below the hard stop). See § Phase 0 — Outcome, Confirmation 2.
3. Keep `makeSmoothMove(target)` returning empty when already at the target **and
   at rest**; when at the target but moving, it must produce a braking profile,
   not an empty trajectory. That distinction is the whole point. Note that this
   makes a legitimate **negative commanded velocity** appear on post-fix captures
   well before the catch — the Phase-0 probe distinguishes it (`first_neg_cmd`)
   from the armed catch descent (`catch_desc`) so the dip/peak measurement window
   is not collapsed by it.
4. Transcribe to `Trajectory.h`. Add an xref test pinning the sim mirror against
   the firmware source constants (extend or mirror
   `tests/firmware/test_hand_traj_xref.py`'s approach). **Scope it to the prelude
   and the stroke geometry.** Do *not* assert catch-timeline parity: the mirror
   diverges from the firmware in the catch time origin (−4.9…−9.7 ms, a
   velocity-independent 0.498 rev) and in the catch `end_time` (+90…+95 ms, the
   `END_PROFILE_HOLD_S` term), both benign and both out of scope for this plan, so
   such an assertion fails for a reason unrelated to Phase 4 — and the obvious
   "repair" (changing `sim/hand/trajectory.py:127`'s `_t_offset`) silently moves
   the sim's hand position at ball arrival by 15.75 mm and pre-empts an open
   question this plan reserves for the operator. Sized against the second pass's
   bounds (≤4.4e-8 m / ≤3.2e-8 s / ≤6.3e-7 rev), not the first pass's. See
   § Phase 0 — Outcome, Confirmation 2.
5. Bump the firmware version if the Platform Teensy carries one, and note in the
   phase logbook that bridges/firmware older than this cannot be mixed (the
   can-hub v3 precedent).

**Files:** `sim/hand/trajectory.py`, `tests/sim/test_hand_trajectory.py`,
`Teensy_code/Trajectory.h`, an xref test.
**Gate:** `pytest tests/ -q` green; the sweep tests pass; a bench flash on the
Platform Teensy. **This phase requires a firmware flash, not a relaunch.**

### Phase 5 — Hardware validation (operator-run)

Operator runs the actuating commands; implementer prepares them and verifies
read-only.

Run with `tests/hardware/toss_trace_recorder.py record` active, then analyse with
the Phase-0 probe.

**PASS** (the numeric per-row criteria live in
`tests/hardware/session_anomaly_fixes.md` § Section HAND; this is the summary):
- **No dip.** The gated measure is the probe's `dip_below_x3` row — how far under
  the 9.9594 rev stroke end the hand ended up — and it must be `<= 0.10` rev
  (3.2 mm). Peak overshoot past 9.9594 rev also under 0.1 rev.
  **Not** "no negative velocity between release and the catch descent": a healthy
  stroke that overshoots and settles onto `x3` from above has a genuinely negative
  measured velocity on the way down (−0.3 rev/s at a 0.02 rev overshoot, ~−10 rev/s
  at 0.64 rev), and Phase 4 step 2 makes a bounded overshoot the *expected*
  behaviour. `pullback` is recorded, and aborts only below −5.0 rev/s — pre-fix it
  read −17.9 to −42.4 rev/s.
- Commanded position never truncates mid-stroke (`pos_cmd` follows the planned
  decel ramp to `x3`), and no from-rest quintic seed appears inside the stroke.
- Throw repeatability: ≥5 tosses at a fixed height; record achieved flight and
  release-speed scatter. Expect achieved flight to move toward the commanded 0.8 s
  from the current 0.887–1.091 s. Record the numbers; do not gate on a specific
  value this round, since the release model is still unmeasured (Phase 5 T0 of
  `plans/active/single-ball-toss.md`).
- No `Not enough time for smooth-move` on the Teensy serial.

**ABORT:** any second stroke or mid-ascent yank; hand position exceeding 10.5 rev;
E-STOP.

Validate alongside `plans/active/levelling-frame-contract.md` Phase 4 in one
sitting if both have landed, but score them separately.

## Testing Plan

| Level | What |
|---|---|
| probe | `tools/probes/hand_stroke_timeline.py` reproduces the recorded dip |
| unit | stroke-model helper: `t_acc`/`t_dec`/`t_acc_catch`/`x1..x5` against pinned values |
| unit | prelude with `v0 ≠ 0`: peak-accel bound respected across a `v0` sweep; overshoot bounded inside the stroke |
| unit | `makeSmoothMove` at-target-and-moving produces a braking profile, not empty |
| integration (mocked ROS) | arm withheld until stroke end; dispatched right after; inert for reload; positive slack at 0.55 s flight |
| integration (mocked ROS) | failed ack during the stroke defers instead of repacking; abort path still clobbers |
| xref | sim mirror == firmware constants |
| drift-guard | `hand_catch_prime_rev` == derived stroke-top |
| hardware | Phase 5 |

Mocked-ROS tests cannot see the Teensy's queue semantics, which is where the
actual failure lives — so Phase 5 is not optional, and the Phase-0 probe is the
instrument that makes it readable.

## Risk register

| Risk | Mitigation |
|---|---|
| Delaying the arm eats the catch's own lead | Phase 1 step 3 checks the window at the shortest shipped flight and logs loudly if it closes |
| A kind-3 abort retract stops clobbering an armed stroke | explicit exemption + a test; called out in both Phase 2 and the contract |
| Velocity-continuous prelude overshoots into the stroke end-stop | bound the excursion against 11.1 rev; decide the cannot-fit behaviour deliberately |
| Sim mirror is not faithful, so Phase 4's gate is illusory | Phase 0 verifies the mirror *before* it is trusted |
| Moving the prime rev trips park-band / parked-top windows | Phase 3 greps and checks every consumer before changing the number |
| Firmware/host version skew after Phase 4 | version bump + a phase-logbook note on mixing |
| Parallel session editing the same files | `git fetch && git status -sb` before starting and before every push |

## Notes for collaborators

- Items 3 and 4 are independent fixes for the same root cause at two levels. If
  time is short, Phase 1 alone removes the operator-visible defect and the throw
  jitter; Phase 4 is what stops the next variant appearing somewhere else.
- The `hand_telemetry` stream at ~100 Hz is genuinely enough to see all of this —
  every number in the Context section came from it plus the modelled stroke. Keep
  recording traces during hardware sittings.
- Deployment reminder: Phases 1–3 are `ros_ws` + config ⇒ `colcon build` +
  **relaunch** (the launch runs the installed copy). Phase 4 is firmware ⇒
  **flash**.
