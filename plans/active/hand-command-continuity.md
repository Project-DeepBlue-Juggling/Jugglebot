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
| 1 | Gate the hand-catch arm until the throw stroke completes (item 3) | full pytest | **DONE** (shared `motion/trajectory/hand_stroke.py`; one enforcement point in `_arm_hand_catch`; window 395 ms at 0.80 s / 115 ms at 0.55 s; contract written down at `ros_ws/docs/hand_command_continuity.md` — see Phases 1-2 — Outcome) |
| 2 | Repack safety guard while a stroke is in flight (item 5) | full pytest | **DONE** (same enforcement point — the retry defers rather than repacks; cap and keep-the-latch preserved) |
| 3 | Prime rev derived from stroke geometry (item 6) | full pytest + codegen | **DONE** (prime 9.858 → the derived 9.9594 rev; `HAND_STROKE_TOP_REV` emitted by codegen; three-route drift guard; no threshold widened. Review also corrected the bang-bang-vs-quintic profile model three neighbours were sized against — see Phase 3 — Outcome) |
| 4 | Velocity-continuous prelude — sim mirror then firmware (item 4) | sim tests + xref; flash | **DONE in source, NOT FLASHED** (`makeSmoothMove` seeds `(x0, v0, 0) → (target, 0, 0)`; exact `pos = x0 + δ·s + (v0·T)·h` decomposition; closed-form duration bound; TWO cannot-fit tests — excursion against the end stops and duration against the longest rest-to-rest move the stroke admits — both falling back to today's profile. Scope narrowed against the plan and recorded: continuity is affordable only to ~9.1 rev/s at the stroke top / ~20.3 rev/s mid-stroke, so the measured ~120 rev/s case still falls back and stays owned by Phase 1. Finalize moved the commanded-position floor off the bottom hard stop and added the duration cap — see Phase 4 — Outcome) |
| 5 | Hardware validation (operator-run) | `trunc=-`, `seeds=0`, `peak <= 10.060` rev, `dip_below_x3 <= 0.10` rev; throw scatter recorded | **PARTIALLY VALIDATED 2026-07-27** — sitting run from § THE RUN SHEET stages 6–7; verdicts in `logbook/2026-07-28-anomaly-fixes-validation-sitting.md`. Flash confirmed on all six launches (`H4.0d`/`FW-1`: `PLATFORM_FW_CHECK: OK — v1`), so **the Phase-4 rows mean something**. **THE HEADLINE PASSES**: `dip_below_x3` reads **0.000–0.026 rev on 15 of 17 tosses** against a pre-fix **0.339–1.748 rev (10.7–55.3 mm)** — a **40–70× reduction** — and the mechanism is *verified, not inferred*: on **all 17** tosses `pos_cmd` reached x3 and held it **28.0–61.6 ms** before any new command landed, with commanded velocity never negative and commanded position never below x3 between release and the arm. The two `[OVER]` rows (`0.1755 / 0.1734`) both carry the row-7 brake annotation — the REPORT case the row-4/row-7 qualifier exists for. **Phase 4's velocity-continuous branch fired on hardware for the first time** on 4 of 17 tosses (`v0 = −8.44 / −6.90 / −6.98 / −7.55 rev/s`, max commanded `10.2259 rev`, 0.374 rev under the clamp) — and the runbook's claim that "no row provokes it" is wrong: **a fast throw provokes it every time**. Also PASSED: `H1.2`–`H1.7` (17 latched = 17 tosses, 17 withheld, **0** CLOSED, min slack **0.124 s** = 2.5× the floor), `H2.1`–`H2.3`, `H3.1`–`H3.7` (prime settles `9.9571–9.9586 rev`, spread **0.05 mm**, **zero** overshoot; peak vel on model to 4 %), `H4.9`, `H4.10`, `H4.0b` (173 passed, no skips), and **stage 7 `HAND-1b`** (both 0.38 m throws clean, the 115 ms window did **not** close) plus the 0.1 m refusal (`REJECTED_FLIGHT_TIME` in 4.4 ms, `pos_cmd` identically 0.0000 rev). **ABORTS, neither a Phase-4 regression**: (a) `peak` exceeds 10.060 on the **0.78 m** tier (`10.2851–10.3258`, pre-fix `10.1653–10.3248` at the same height) and on five **off-run-sheet ~1.2 m** tosses (`10.8601–11.0621 rev` = **1.2–7.6 mm from the declared 11.1 rev limit**) — pure position-loop coast past a commanded profile that never left x3, growing steeply with speed (`+0.074/+0.063/+0.345/+1.020 rev` at 2.74/3.44/3.97/4.86 m/s), such that **a legal in-band toss at the shipped `FLIGHT_TIME_MAX_S = 1.10 s` would exceed the 355 mm stroke top**; (b) the **dispatch shift** has grown to `+54…+63 ms` (bag clock) from the pre-fix `+12.8…+21.9 ms`, **exceeding the 40 ms margin Phase 1's stroke-busy window budgets** — it tracks can-bridge Teensy uptime, so this is the 2026-07-18 lag finding reaching the arm gate, not a hand defect. **NOT SCORED**: `H4.2` (no pre-flash control, by design); `H2.4`/`H3.6`/`H4.8` — 0 SAFE_ABORT in stage 6, **but one occurred naturally in the 16:00:27 launch**, so these moved from *unexercised* to *scorable, not yet scored*; `H1.6`/`H4.7` scored **indirectly** (no Teensy serial capture). **ACTION: no further tosses above 0.78 m** until the true stroke limit is pinned (11.124 vs 11.224 vs 11.4 rev — the three sources disagree by ~9 mm of margin) and the flight band re-examined. Rows 1/2/`H2.2`/`H4.6` carry a **criterion defect** (they fire on the gated arm's own prelude landing within `_TRUNC_SCAN_MARGIN_S = 0.050` of the *modelled* stroke end) — adjudicated PASS; the criterion still needs fixing. Run it from `tests/hardware/session_anomaly_fixes.md` § THE RUN SHEET (stage 6 rows HAND-1…HAND-4, stage 7 HAND-1b), which is the authority for the order, the shared capture and the numbers. **Phase 4's half needs a PLATFORM TEENSY FLASH, not a colcon build** — see the runbook's § DEPLOYMENT MATRIX row C. Whether it took is now READABLE (row **FW-1** / **H4.0d**, Phase 6); it is still not *enforced*, so the row must actually be run. Row 4 (`dip_below_x3`) is qualified by row 7 (`first_neg_cmd`) once Phase 4 is flashed: on a toss where a braking prelude fires, the two score the same event in opposite directions and row 4 becomes REPORT |
| 6 | Platform Teensy `FW_VERSION` — make host/firmware skew detectable (operator-requested, added after the run closed) | scoped pytest + a whole-sketch compile | **DONE in source, NOT FLASHED.** `FW_VERSION = 1` in `Teensy_code.ino`, reported in bytes 5-6 of the 0x6E0 RobotState reply (previously hard-zeroed reserved bytes, so a pre-versioning board ANSWERS with 0 rather than going silent); surfaced as `robot_state.platform_fw_version` / `link_status/platform_fw_version` / a `PLATFORM_FW_CHECK` log line. WARNS, never refuses. Contract `ros_ws/docs/platform_fw_version.md`. Also adds the first compile gate on this sketch (`Teensy_code/platformio.ini`, build-only). **LANDED** `bb15d9b` (+ SHA backfill), full suite green (3966 passed, 3 xfailed, 2026-07-27), logbook `2026-07-27-platform-teensy-fw-version.md` — see § Phase 6 — Outcome |
| 7 | **Post-release deceleration feedforward** — stop flirting with the end stop (operator decision (b), 2026-07-28) | scoped pytest + the `pio` compile gate; **flash** | **DONE in source, NOT FLASHED** (2026-07-29, `f920087`). Root cause: `accelToTorque` models the hand axis as a pure translating mass on a spool, implying a reflected inertia of `7.3695e-6` kg·m² against a measured `>= 1.0126e-5` — so the braking feedforward delivered **~70 %** of the torque the commanded decel needs, and the shortfall fell to a loop whose integrator unwind constant (0.100 s) is 1.1–2.1× the whole 47.4–93.3 ms ramp. Fixed at ONE enforcement point (`throwDecelToTorque`, one caller: `buildThrow`'s `torA[2]`) with the inertia declared in config at a **deliberate 6–10 % under-estimate** of a **decel-side** lower bound (1.0126e-5 kg·m², re-anchored during review — the accel-phase 1.015e-5 figure was ball-inclusive), which makes the feedforward one-sided-safe. Gravity brakes in the same direction on an upward decel, so the open-loop total does exceed `J_true` below `a_cmd ≈ 1900 rev/s²`; the enforcement is bench row H7.4 and the new band-floor rung **R0**, not the inequality. **Commanded position and velocity are bit-identical on every kind**, so C-HAND-1's window, `_PRIME_INFLIGHT_S` and the timeline probe's whole verdict model stay valid without moving. Contract `ros_ws/docs/hand_decel_feedforward.md` (**C-HAND-2**); `FW_VERSION` **1 → 2**; bench `tests/hardware/session_anomaly_fixes.md` § CHECK HAND-7 (stage 8), ladder **R0→R5** with three desk pre-flights (H7.0a xref zero-skips, H7.0b probe `--self-check`, **H7.0c read `torque_soft_min` off the live drive**) — see § Phase 7 — Outcome |

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

> **⚠ SUPERSEDED by Phase 1 — the `546 ms` / `208 ms` in the table below are NOT
> the shipped windows.** Phase 0 sized them with neither the smooth-move duration
> FLOOR (`fmaxf(T, 0.05f)`, so the prelude is 50–76 ms and never zero) nor
> `_MIN_EVENT_DELAY_S = 0.3`. The real windows are **395 ms at 0.80 s** (not 546)
> and **115 ms at 0.55 s** (not 208) — carried in `catch_coordinator_node.py:1033`
> and `motion/trajectory/hand_stroke.py:97`. Size the runbook's H1.5 slack row
> against 395/115; see § Phases 1–2 — Outcome below. Kept as the correct account of
> the model Phase 0 could see.

| flight | `t_dec` | earliest safe arm | latest usable arm | window |
|---|---|---|---|---|
| 0.80 s (`v` 3.93, armed 3.13) | 65.1 ms | release + 105 ms | release + 651 ms | **546 ms** |
| 0.55 s (`FLIGHT_TIME_MIN_S`, `v` 2.70, armed 2.16) | 94.9 ms | release + 135 ms | release + 343 ms | **208 ms** |

(latest usable arm = `event − t_acc_catch − prelude − SAFETY_GAP`, prelude empty,
`SAFETY_GAP` 20 ms.) The window narrows with flight time exactly as the plan
predicts but stays positive at the shortest shipped flight.

*The `t_dec` column rounds `v`: at the production `v` = 2.7089 (not the 2.70 shown)
`t_dec` is **94.5 ms**, which is the figure § Phases 1-2 — Outcome and the shipped
docstrings carry. Not a contradiction — a rounding artefact of this table's `v`
column.*

*Both `latest usable arm` figures are optimistic and were revised during Phase 1.*
`prelude empty` is unreachable — `makeSmoothMove`'s dead-band is 1e-6 rev =
3.16e-5 mm and every non-empty duration is floored at 0.05 s
(`Trajectory.h:260`), so the real prelude is 50–76 ms — and the caller drops the
arm outright below `_MIN_EVENT_DELAY_S = 0.3` s, which binds ahead of the Teensy
budget at every nominal velocity. Carrying both: **release + 500 ms / 395 ms** at
0.80 s and **release + 250 ms / 115 ms** at 0.55 s. Still positive at both ends;
see § Phases 1-2 — Outcome.

**Both rows are at the NOMINAL armed velocity** (`catch_coordinator_node.py:667`
scales the event velocity by `JB_OP_CATCH_VEL_SCALE_DEFAULT = 0.8`, clamped to
`[0.3, 7.0]`; the 2026-07-25 launch log confirms `vel=3.13 m/s (scale 0.80)`
against a 3.9224 m/s throw). Since `t_acc_catch = 0.404073 / v_armed`, a *low*
tracker landing-speed estimate lengthens the lead and moves the right-hand edge
earlier: at the 0.55 s flight the window closes once `v_armed < 1.02` m/s (a
tracker landing speed under ~1.28 m/s for a 2.70 m/s throw) — *revised to 1.26 /
~1.58 m/s once the prelude FLOOR and the caller's `_MIN_EVENT_DELAY_S` drop are
carried; see § Phases 1-2 — Outcome*. Phase 1 step 3
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
   must accept and **1.4×** clear of the largest brake it must reject.
   > **SUPERSEDED by Phase 4 (2026-07-27).** The "largest brake it must reject"
   > premise held only for the settle-from-above case, which is all seven observed
   > tosses carried. A brake seeded from a *downward* velocity dives
   > `0.00778·v0²` rev below x3 — past 0.5 rev at 8.05 rev/s and honoured out to
   > 3.213 rev — and with the 0.5 rev separator the probe put `catch_desc` on the
   > brake and reported `dip_below_x3 = 0.0` on a capture that dove 3.21 rev.
   > The separator is now `_CATCH_DESC_ABOVE_X5_REV`: the descent must reach the
   > catch region (`x5 + 0.33` rev), which no honoured brake can. See § Phase 4 —
   > Outcome.

   The brake is
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
   with a 40 ms margin sized from the measured dispatch latency —
   *superseded by Phase 1's implementation: those figures omit the smooth-move
   duration FLOOR (`fmaxf(T, 0.05f)`, so the prelude is 50–76 ms, never zero) and
   the caller's own `_MIN_EVENT_DELAY_S = 0.3` drop. The real windows are 395 ms
   and 115 ms, and the 0.55 s closure velocity is 1.26 m/s rather than 1.02. All
   three still pass this step. See § Phases 1-2 — Outcome.* The margin is
   **not** optional slack — the announcement's `throw_time` is up to **23.4 ms**
   earlier than the physical release, so `t_release + t_dec` alone expires
   mid-ramp. See § Phase 0 — Outcome.
   **Evaluate the fit against the RUNTIME `event_vel` at dispatch, not a nominal.**
   Phase 0's window table is computed at the nominal armed velocity
   (`JB_OP_CATCH_VEL_SCALE_DEFAULT = 0.8` × the throw speed), but the catch arm's
   own `event_vel` comes from the tracker and `t_acc_catch = 0.404073 / v_armed`,
   so a low landing-speed estimate *lengthens* the required lead and moves the
   window's right-hand edge earlier. At the 0.55 s flight the window closes once
   `v_armed < 1.02` m/s (*revised to 1.26 m/s — § Phases 1-2 — Outcome*). Log
   loudly and dispatch immediately when it would close —
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

### Phases 1-2 — Outcome (2026-07-26)

**Both landed together.** They share one enforcement point, so splitting them
into two commits would have meant landing a predicate with no consumer or a
consumer with no predicate.

**Commit:** `6179a88` (code + tests + contract + runbook + logbook + this plan),
followed by a small SHA-backfill commit.
**Suite:** `pytest tests/ -q`, run 2026-07-26 on the Jetson under
`~/Desktop/PDJ_venv/venv`: **3517 passed, 3 xfailed in 1351.82 s (22:31)**.
Baseline at HEAD `2395244` was 3484 passed, 3 xfailed in 1371.51 s — **+33
passed**, exactly the 33 cases this phase adds (15 + 18); the xfail count is
unchanged at 3.
**Logbook:** `logbook/2026-07-26-hand-command-continuity-arm-gating.md`.

**DEFERRED TO THE OPERATOR — this phase is NOT validated until it runs.** Both
halves are host-side, so deployment is `colcon build --packages-select jugglebot`
**+ relaunch** (the launch runs the *installed* copy — a relaunch alone keeps the
old code). **No codegen run** (no YAML changed) and **no firmware flash**: the
firmware half of C-HAND-1 is Phase 4. The bench checks are
`tests/hardware/session_anomaly_fixes.md` § Section HAND, rows **H1.1-H1.7**
(+ optional HAND-1b) and **H2.1-H2.4**, both scored off a single capture. Note
what the bench is actually for: mocked-ROS tests prove the arm is not
*dispatched* during the stroke, but they cannot see the Teensy's queue semantics,
which is where the failure lives — H1.1's `trunc`/`seeds` rows are the only
evidence that no queue was cleared.

#### What shipped

* **`ros_ws/src/jugglebot/jugglebot/motion/trajectory/hand_stroke.py`** (new) —
  THE host-side model of `Trajectory.h`. `HandStrokeModel` (calcThrow +
  calcCatch), `throw_decel_s`, `catch_lead_s`, `smooth_move_duration_s`,
  `stroke_clear_time`, `required_arm_lead_s`, and the two policy constants
  `ARM_SUPPRESS_MARGIN_S = 0.040` / `HAND_SETTLE_BAND_REV = 0.10`. The module
  itself imports only `math` + `jugglebot.hardware_config` and no ROS; note that
  importing it *through* the `jugglebot.motion.trajectory` package still pulls
  numpy, via that package's own `__init__`.
* **`catch_coordinator_node`** — `_latch_throw_stroke_window` (off the
  announcement) and `_throw_stroke_gate_ok` (the single enforcement point,
  consulted from `_arm_hand_catch`).
* **`tools/probes/hand_stroke_timeline.py`** — its local `StrokeModel` and
  `smooth_move_duration_s` were **deleted** and are now imported from the shared
  helper.

#### Fork — where the stroke-end instant comes from → derive from the announcement

Both inputs already exist on the wire: `throw_time` is the kind-0 event instant,
which IS ball release (`makeThrow`'s `shiftTime(-t2)` puts t = 0 at the end of
the velocity hold, and `_dispatch_toss_throw` schedules the event at the same
`t_release` the announcement stamps), and `|initial_velocity|/1000` IS the
commanded `event_vel` (`compute_release_state` returns
`event_vel_mps = |launch_vel|/1000` and the sequencer sends exactly that). So no
new field and no new topic — a wire change would have been a stop, not a fork.

The fixed-conservative-delay option was rejected on its failure modes, not on the
plan's say-so: `t_dec` spans **94.5 ms at the 0.55 s flight to 47.4 ms at
1.10 s**, a 2x range. A delay sized for the short end wastes half the window at
the long end; one sized for the long end lands the repack back inside the decel
ramp at the short end — and at the top of the band the momentum is ~1.9x with the
smallest measured end-stop headroom (0.775 rev), which is where a mis-sized
window costs the most.

#### Fork — shared helper vs a pinned duplicate → shared, and the probe moved into it

Phase 0 noted the probe had already made a **third** host-side copy of the stroke
algebra. A pin between the probe and a fourth copy in the node was rejected: the
probe is the **Phase-5 verdict instrument**, so a `t_dec` divergence would make
the bench score the fix against a different model than the one that shipped, and
a pin only catches the quantities it happens to assert. There is now nothing to
pin. `tools/probes/hand_stroke_timeline.py --gate` still returns **GATE PASS —
25/25 rows** plus **GATE PASS — fixed-shape branch**, exit 0, and
`--emit-gate-fixture` still regenerates the committed fixture **byte-identically**
(285 of 5764 rows).

#### Fork — where the stroke-busy predicate lives → the node, at the dispatch

The pure time arithmetic is in the helper; the *state* (is a throw of ours live,
when does it clear) is node state, and the refusal sits inside `_arm_hand_catch`
rather than at its call site. Concrete failure mode that placement prevents: a
second caller added later — a re-arm timer, a recovery path — would bypass a
call-site check silently, and the defect it re-introduces is invisible in ROS
(the Teensy prints nothing about a queue clear). It also makes Phase 2 free:
`_on_hand_traj_done`'s retry re-opens the latch, the next balls tick re-enters
`_arm_hand_catch`, and the same gate defers it. **Two predicates would have been
two things to keep in sync; there is one.**

#### The dispatch latency, and why 40 ms

Phase 0's measurement is the reason the margin exists at all: the announcement's
`throw_time` is the *intended* release, and the physical release lands **+12.8 to
+23.4 ms** later because nothing compensates the ROS-service transit between the
caller's clock read and `teensy_bridge_node`'s re-stamp from its own clock
(`JB_OP_TOSS_RELEASE_LATENCY_MS` ships 0.0). A window of
`throw_time + t_dec` with zero margin expires mid-ramp and reproduces the defect
exactly. **40 ms = 1.71x the worst measured shift**, and it costs 40 ms out of a
window that is 395 ms wide at 0.80 s and still 115 ms wide at the band floor.

#### Two Phase-0 statements that did not survive re-derivation

1. **"Arming inside the window makes the prelude EXACTLY empty" is an
   idealisation.** `makeSmoothMove`'s dead-band is `|Δ| < 1e-6` rev =
   **3.16e-5 mm**, unreachable against a live float encoder reading, and every
   non-empty duration is floored at **0.05 s** (`fmaxf(T, 0.05f)`,
   `Trajectory.h:260`). So a catch armed with the hand "at rest at x3" still
   costs **50-76 ms** of prelude. That is harmless *motion* — 0.63 mm at 24 mm/s
   for a 0.02 rev residual — but it is not free *time*, and the fit check now
   budgets `smooth_move_duration_s(0.10 rev) = 76.0 ms` for it. The allowance is
   deliberately the same 0.10 rev band the runbook gates the post-fix settle on,
   so any capture that PASSES the bench is inside the excursion the arithmetic
   assumed.
2. **The window's right-hand edge is set by `_MIN_EVENT_DELAY_S`, not by the
   Teensy budget, at every nominal velocity.** The caller drops the arm outright
   below a 0.3 s `event_delay`, while the Teensy needs only
   `t_acc_catch + prelude + SAFETY_GAP` = 225 ms at the nominal armed 3.13 m/s.
   The deadline therefore takes the **max** of the two — deferring past the
   caller's own floor would lose the catch just as surely as the Teensy refusing
   it. Consequences for Phase 0's table, which had neither term: the real windows
   are **395 ms at 0.80 s** (not 546) and **115 ms at 0.55 s** (not 208), and the
   window closes at 0.55 s once `v_armed < 1.26` m/s (not 1.02) — a tracker
   landing speed under ~1.58 m/s for a 2.71 m/s throw. All three still pass Phase
   1 step 3 comfortably; the numbers are simply honest now.

#### What happens when the window would close

Log **loudly** and dispatch immediately, accepting today's degraded behaviour.
Root cause: `Teensy_code.ino:533` refuses the WHOLE command when
`now + smoothDur + SAFETY_GAP > firstMainAbs` and prints the refusal to serial
only (`:534`). An arm deferred past that point does not arrive late — the catch
silently never fires, with no ROS-visible signal and the ball on the floor. A dip
is ugly and recoverable; a silently-refused catch is neither. The branch is
evaluated against the RUNTIME `event_vel`, not a nominal, because
`t_acc_catch = 0.404 / v_armed` and a low tracker landing-speed estimate is
exactly what lengthens the lead.

#### The abort path is untouched, and that is deliberate

The gate sits inside `_arm_hand_catch`, which only ever emits `traj_type = 1`.
The kind-3 smooth-move path (`_prime_hand`) is not gated, and the SAFE_ABORT
retract is dispatched by `reload_coordinator_node` through its own
`smooth_move_hand` client and never passes through this node at all. A kind-3
replacing whatever is queued is the ONLY un-arm mechanism the Teensy offers and a
pre-release SAFE_ABORT depends on it clobbering an armed kind-0
(`toss_sequencer`'s ORDERING PRINCIPLE). The toss's own prime-during-stroke
hazard is owned by `catch/prime_hold`, raised for the whole PREPARE→terminal
span — a separate, already-enforced gate. Pinned by
`test_kind3_smooth_move_is_not_gated_by_the_stroke_window`.

#### The reload path is inert, by construction

The window is latched only when `thrower_name == robot_name`. `target_id` alone
cannot discriminate — a BB throw aimed at us carries `target_id == robot_name`
too. During a reload there is no Jugglebot throw stroke, the hand is parked at the
top at rest, and delaying the arm would eat lead the catch needs. Pinned by
`test_reload_announcement_leaves_the_stroke_window_inert`.

#### Limitation, recorded rather than papered over

An **armed** stroke produces no observable until its event time, so the arm cannot
be telemetry-verified the way the hand ladders were (`4e33b53`) — which is why
the retry path exists at all, against a 40-60 % `ERR_TIMEOUT` ack failure rate. A
Teensy-side "armed stroke" field in `hand_telemetry` or `link_status` would make
it verifiable; that is a **protocol change and is out of scope here**. Follow-up,
not solved. What a capture CAN verify is the harm the guard prevents: a repack
that clobbers a **live** stroke re-seeds the queue from the live encoder, which
the Phase-0 probe counts as a from-rest quintic `seed` — that is CHECK HAND-2's
criterion.

Mocked-ROS tests cannot see the Teensy's queue semantics, which is where the
actual failure lives: they prove the arm is not *dispatched* during the stroke,
not that no queue was cleared. The bench closes that gap —
`tests/hardware/session_anomaly_fixes.md` § CHECK HAND-1 (rows H1.1-H1.7, with
H1.2/H1.3 reading the mechanism out of `/rosout` so a PASS by luck is
distinguishable from a PASS by design) and § CHECK HAND-2 (H2.1-H2.4).

#### Review adjudication — five claims that changed the artefact

The three-lens review produced twelve findings; nine were verified and fixed,
three were verified-but-deferred. Five changed what ships, and each is a limit on
the fix that a bench session would otherwise have mis-scored:

1. **The closed-window branch promises an attempt, not a catch** (converged, two
   lenses). Its fit check budgets the *at-rest* prelude, but on that branch the
   hand is mid-stroke, so the firmware's live-encoder prelude is 0.37-0.76 s and
   `Teensy_code.ino:533` may refuse the dispatch outright. Not a regression — it
   is the pre-fix arithmetic exactly — and settled by reading the firmware:
   `:533`'s `return` sits **before** `packedMsgs.clear()` at `:539`, so a refusal
   leaves the live throw stroke **intact**. The cost is a lost catch, never a
   clobbered stroke. The suggested code fix (drop the arm when the worst-case
   prelude will not fit) was **rejected**: a drop guarantees no catch, whereas a
   dispatch is refused only if the Teensy's own clock agrees it will not fit. The
   docstrings, the warning text and runbook H1.4/H1.6 now say so.
2. **`required_arm_lead_s` excludes the downstream transit.** Only the *upstream*
   caller→bridge leg cancels (the bridge re-stamps the event from its own clock);
   `:533`'s `now_us` is read at the Platform Teensy after the bridge→can-bridge→
   CAN3 hop, and that leg is pure budget loss, bounded by the ~23 ms measured
   shift. Sized rather than fixed: `max(_MIN_EVENT_DELAY_S, budget)` means the
   0.3 s floor binds at every nominal armed velocity (the budget only overtakes
   it below `v_armed` 1.98 m/s), and the tightest case — the 0.55 s flight at the
   default knob — leaves 15.9 ms of floor headroom against the ~23 ms bound. A
   7 ms shortfall, against the **115 ms** of slack the gate actually dispatches
   with, because it fires at the FIRST tick after the window opens, not the last.
   Runbook H1.5 is the bench guard.
3. **`catch/vel_scale` can close the window on its own** — the docstrings blamed
   only a 40 % tracker under-read. Swept against the production velocities: at the
   0.55-0.56 s flight, scale **0.45 closes it (−15 ms)** and 0.50 barely opens it
   (+18 ms), inside the shipped `[0.3, 1.5]` range, with a healthy tracker. That
   is exactly the corner HAND-1b runs in, so H1.4 now tells the operator to read
   the knob before routing a CLOSED warning to a tracker fault, and HAND-1b is
   pinned to the default scale.
4. **The flight-time error is probably not this phase's to fix.** The pre-fix
   0.887/1.091 s against a commanded 0.800 s was attributed to the truncated
   decel ramp "setting the release conditions". But the ball separates at the
   decel ONSET (`x2`), and all seven measured truncations sit past the commanded
   `x2` crossing (6.1965-7.7825 rev against `x2` = 5.9138 rev) — so the ball had
   most likely already left the cup. A null result on flight time is expected and
   is **not** a Phase-1 failure; the release model is `single-ball-toss.md` Phase
   5 T0's measurand. Softened in the node header and the runbook so the next
   session does not re-open the queue-clobber question.
5. **The deferral is tick-dependent** (converged, two lenses, both NOT-PROVEN on
   reachability). The gate is reached only from `_on_balls` and nothing re-enters
   it on a timer, so a track dropout spanning the whole remaining window — or a
   landing revision pushing `event_delay` under the 0.3 s floor — bypasses the
   gate entirely, and even the closure branch cannot fire. Argued-against by the
   node's own probed note (announced-vs-tracked landing agreeing to 0.000 s at
   the arm moment, n = 6105, in early life = exactly the suppression window), so
   **instrumented rather than fixed**: new runbook row **H1.7** counts withheld
   lines with no matching dispatch. A non-zero count is the signal to make the
   deferral self-driving with a one-shot timer.

Deferred with reason: instrumenting the measured hand velocity at the window's
opening (the probe's gate fixture is byte-compared, and this phase's evidence
that moving the model did not perturb the instrument rests on that byte-identity
— H1.1's `dip_below_x3` already measures the harm the premise protects against);
and the two liveness fixes above, which are behaviour changes on the strength of
an unproven trigger.

#### Tests

`tests/motion/test_hand_stroke.py` (15 cases) pins the model against the
**shipped firmware header**, parsed rather than copied — so a firmware algebra
change codegen cannot see still fails — plus the velocity-independence of `x3`,
the smooth-move dead-band and floor, and the window at both flight-band ends.
`tests/ros/test_catch_coordinator_node.py` adds 18 cases for the latch, the gate,
the deferral, the cap, the inert reload path, the closure branch and the kind-3
exemption.

`test_margin_covers_the_measured_dispatch_latency` asserts the **residual**
property (`margin >= worst_shift − compensated_latency`) rather than pinning
`JB_OP_TOSS_RELEASE_LATENCY_MS == 0.0`. When `single-ball-toss.md` Phase 5 T0
fills that slot the kind-0 event moves earlier while `throw_time` does not, so
the shift this margin covers *shrinks* — pinning the equality here would turn
that improvement into a red suite in a module with nothing to say about it. The
deliberate tripwire on the slot already lives in `tests/ros/test_toss_coordinator.py`.

`test_stroke_window_cleared_on_both_latch_edges` drives both edges
independently. As first written it exercised only the disarm edge — nothing is
latched by the time it re-arms, so the closing assertion passed on the disarm
clear alone and deleting the arm-edge block left the file green. Verified: the
same mutation now fails it.

Mutation-verified rather than assumed: deleting the one-line gate call in
`_arm_hand_catch` fails **8** of the new node tests; setting
`ARM_SUPPRESS_MARGIN_S = 0` fails **9** across both files; removing the
`thrower_name` discriminator fails **14** (including
`test_reload_announcement_leaves_the_stroke_window_inert`, the one that carries
the semantics).

#### Deployment

`colcon build --packages-select jugglebot` **+ relaunch** — the launch runs the
installed copy. `setup.py` already lists `jugglebot.motion.trajectory` as a
package, so the new module installs with no packaging change. No codegen run (no
YAML changed) and **no firmware flash**: Phases 1-2 are host-side only. Phase 4
is the flash.

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

#### Phase 3 — Outcome

**Landed 2026-07-26** in `94fe817` (code + docs) and `410b1ea`
(SHA backfill). Logbook: `logbook/2026-07-26-hand-prime-rev-derived.md`.

`hand_catch_prime_rev` moves `9.858 → 9.9594` rev — it had drifted `0.101403`
rev = **3.207 mm** below x3 for the life of the constant, so every "catch from
rest at the top" opened with a real 76.5 ms / 3.2 mm prelude. `generate_config.py`
now emits `HAND_STROKE_TOP_REV` (x3, in rev) alongside the x2/x5 landmarks; the
YAML key stays an explicit override with the derivation written beside it, and
`tests/motion/test_hand_stroke.py::test_catch_prime_equals_the_stroke_top` guards
it by three independent routes (generated constant, host model, and x3 re-derived
from the **parsed shipped firmware header** — the only route that survives a
hand-edit bypassing codegen).

**The Phase-3 risk paragraph above did not bind.** Every park/parked-top window
was enumerated and measured at both primes and none is tight, so nothing was
widened: near-band margin `0.2156 rev = 6.8 mm` in the pessimistic reading,
bridge headroom `1.1406 rev = 36.1 mm`, and `toss_sequencer`'s `hand_parked` gate
is keyed to `JB_OP_HAND_RETRACT_REV = 0.0` (the **bottom** band), so the kind-0
off-band hazard is structurally independent of the prime. Peak commanded
acceleration is **unchanged at 100 rev/s²** — the duration formula solves for it,
so a taller stroke buys duration, never a harder command.

**What the review added, and it was not in this phase's charter.** The
`sqrt(a·Δx)` / `2·sqrt(Δ/a)` closed forms three of the prime's neighbours are
sized against are **bang-bang** forms; `makeSmoothMove` is a quintic. Both a
duration (`MIN_THROW_EVENT_DELAY_S`'s "~0.63 s" against the real **0.758 s**) and
a velocity (`_THROW_STROKE_VEL_RPS`'s "peak ascent 31.56 rev/s" against the real
**24.63 rev/s**) were wrong, in opposite directions, and this phase had
propagated both while updating them for the new prime. Fixed at the root:
`hand_stroke.smooth_move_peak_vel_rps` / `_bound_rps` are now the named model all
four sites quote. The code guard was **not** re-sized — clearing a conservative
bound is the right thing for a guard to do — but bench row H3.7 was recalibrated
from `PASS <= 35` (which would have scored a 26 % overspeed as "on model") to
`PASS <= 30` / DEBRIEF 30-40 / ABORT `>= 40`. `_PRIME_INFLIGHT_S = 1.2` was also
found unpinned: sized from the ascent duration but never naming the constant, so
a grep sweep misses it structurally.

**Tests**: `pytest tests/ -q`, run 2026-07-26 on the Jetson — **3531 passed,
3 xfailed in 1356.73 s (0:22:36)**, exit 0 (baseline at `fbd8d13`: 3527 passed,
3 xfailed in 1367.68 s; the +4 is exactly the four new test functions, xfail
unchanged). No test weakened, skipped, deleted or xfailed; the one restated literal
(`prelude_peak == approx(31.4 → 31.56)`) was adjudicated ACCEPT with the
tolerance and both inequalities unchanged.

**Deferred to the operator**: `colcon build --packages-select jugglebot` +
**relaunch**, then `tests/hardware/session_anomaly_fixes.md` § **CHECK HAND-3**
(7 rows, no extra robot motion — read off the HAND-1 capture plus `launch.log`).
**No firmware flash**: `JBOp::HAND_CATCH_PRIME_REV` is a dead `constexpr` no
sketch reads.

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

#### Phase 4 — Outcome

**Landed in source 2026-07-27, NOT FLASHED.** Commit `5369fc2` (code; this paragraph backfilled in the
follow-up commit). Full suite: `pytest tests/ -q`, run 2026-07-27 on the Jetson at
HEAD `854df28` with the phase uncommitted — **3943 passed, 3 xfailed in 1399.35 s**
(baseline 3574 passed, 3 xfailed; the +369 is accounted for per changed test file
in the logbook entry). **xfail unchanged at 3.**

`makeSmoothMove` now reads `current_hand_velocity` — the `extern volatile` that
sat two lines above it and was never read, which *was* the defect — and seeds the
quintic `(x0, v0, a = 0) → (target, 0, 0)`. The profile decomposes exactly into
`pos = x0 + δ·s(τ) + (v0·T)·h(τ)` with `h(τ) = τ(1−τ)³(3τ+1)`; because
`h(0) = h(1) = 0` and `h ≥ 0`, the live velocity never moves the endpoints and its
whole effect is a one-sided bulge of `|v0|·T·16/81` — which *is* the legitimate
overshoot, and is what gets clamped. The duration is the positive root of
`a_max·T² − |v0|·H2·T − |δ|·S2 = 0`, which reduces bit-identically to the
historical form at `v0 = 0`, so every existing rest-to-rest hand move is unchanged
(every commanded position bit-identical; the one difference is a `−0.0f → +0.0f`
first velocity sample for `δ < 0`, which encodes to the same `int16` 0).

**Three of the plan's own statements did not survive, and are superseded here.**

1. *Step 2's cannot-fit menu ("refuse loudly, or brake to the limit") is
   unshippable in both branches.* Refusing means a kind-3 does not clobber, and a
   kind-3 retract clobbering an armed kind-0 is the only un-arm mechanism the
   Teensy offers; refusing via an empty trajectory is worse still, since
   `Teensy_code.ino:472-475` returns *before* `packedMsgs.clear()`. Braking to the
   limit is unbounded by anything the firmware declares (~28 000 rev/s² for a
   near-target abort at the measured −60 rev/s, 280× the declared limit). Shipped
   instead: **fall back to the rest-to-rest profile** — today's exact behaviour,
   adding no commanded magnitude the firmware could not already produce, never
   empty, and observable as the probe's `seeds` row.
2. *Step 2's excursion bound `[0, 11.1]` rev was implemented as `[−0.1, 10.6]`,
   and finalize put the floor back.* `Homing::HAND_ABS_POS_REV = −0.1` rev **is**
   the bottom hard stop (the axis homes downward into it), and it is below the
   floor the host itself declares (`teensy_bridge_node` rejects a smooth-move
   target < 0; `can/odrive.py` clips a hand setpoint < 0 and warns). Brute-forced
   over the honoured branch, that floor admitted commanded troughs to −0.0999 rev
   on the retract, prime *and* catch-descent paths, and a prime dispatched at the
   mid-point of a live retract (5.0 rev, −24.6 rev/s) dived to −0.057 rev — 5 rev
   *below its start*, for a move whose target is the top of the stroke. The floor
   is now `JBOp::HAND_RETRACT_REV = 0.0`. The recorded justification for the
   zero-margin floor ("a margin would make every abort retract infeasible by
   construction") was **refuted**: the bound is relaxed to
   `min(FLOOR, start, target)`, so the target is admitted at *any* floor value and
   the profiles that would have dived under simply take the documented fallback.
3. *There is no `FW_VERSION` on the Platform Teensy to bump* (step 5). It carries
   none, so host/firmware skew on the hand path is undetectable from the Jetson.
   Runbook row **H4.0** is the only guard and it is procedural.
   > **SUPERSEDED by Phase 6 (2026-07-27, operator-requested).** The board now
   > declares `FW_VERSION = 1` — this Phase-4 change is exactly what that first
   > numbered release marks — and reports it in the 0x6E0 RobotState reply.
   > Runbook rows **FW-1** / **H4.0d** read it directly, superseding H4.0's
   > four-link inference chain. Step 5's other half (a note that firmware older
   > than this cannot be mixed) is answered too, and the answer is *it can*: the
   > skew is reported and never enforced, because the same 0x6D0 path carries the
   > kind-3 retract. See § Phase 6 and `ros_ws/docs/platform_fw_version.md`.

**A second cannot-fit test was added at finalize.** Arresting `v0` costs *time* as
well as travel, and the duration grows linearly in `|v0|` while the excursion
grows as `v0²` — so mid-stroke, where there is room for a big bulge, the clamp
alone let a prelude run arbitrarily long. A prime dispatched into a live retract at
the retract's own peak descent speed (start ≈ 5.04 rev, −24.6 rev/s) solved to
**1.206 s**, outgrowing `catch_coordinator_node._PRIME_INFLIGHT_S = 1.2 s` — the
anti-stutter window whose pinning test names "a Phase-4 duration change" as the
thing it exists to catch, and which evaluated only the `v0 = 0` branch. An honoured
prelude is now bounded by `smoothMoveMaxDuration()` = the longest **rest-to-rest**
move the stroke admits (0.8005 s), so it can never outlast a profile this firmware
could already emit and every host window sized on one stays valid without moving.
The pinning test now asserts both bounds.

**Instrument correction, same finalize.** The Phase-0 probe's brake/descent
separator (`_CATCH_DESC_BELOW_X3_REV = 0.5` rev) was sized on the pre-fix brake
sample (0.206–0.365 rev, all settle-from-above). An honoured brake seeded from a
*downward* velocity dives `0.00778·v0²` rev below `x3`, crossing 0.5 rev at just
8.05 rev/s and honoured out to 3.213 rev. Demonstrated on a synthetic capture at
the clamp's reach: with the old separator the probe put `catch_desc` at the brake,
under-reported `peak` by 0.019 rev, and reported `dip_below_x3` as **0.0 rev on a
capture that dove 3.21 rev = 102 mm below the stroke end** — a false PASS on the
row guarding the 11.1 rev end stop. The separator is now anchored on the catch
region (`x5 + 0.33` rev), and a `deep-brake` case at the clamp's reach is a
permanent gate case.

**Deferred operator handoff.** The flash itself (Platform Teensy only — not the
can-bridge, not the CatchingCone), and the envelope question the fallback stands
in for: whether `makeSmoothMove` should get a second, higher acceleration limit so
it can brake harder instead of falling back. `MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2 =
100` rev/s² is a *comfort* limit 19–36× below what the throw profile itself
commands (1902 rev/s² at a 0.80 s flight, **3597** at `FLIGHT_TIME_MAX_S = 1.10 s`
— corrected 2026-07-29 from "6055 at the band top", which is the decel at the
Teensy's `MAX_EVENT_VEL_MPS = 7.0` builder clamp, not at the flight-band top, and
which asserted a commanded decel *above* the axis's own 4178–4333 rev/s² ceiling;
see C-HAND-2); raising it would
widen the continuity band from ~9 to ~40 rev/s and changes what the machine can
physically do. Nothing this sitting needs the answer — the fallback is today's
behaviour, so declining costs nothing. Bench: `tests/hardware/session_anomaly_fixes.md`
§ CHECK HAND-4, rows H4.0a–c (pre-flight) and H4.1–H4.10.

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

### Phase 6 — Platform Teensy `FW_VERSION` (added 2026-07-27, operator-requested)

Added **after** this plan's run closed, at the operator's explicit request. The
run's final independent review named one operational fact as its main residual
risk, and this phase closes it: the Platform Teensy carried no version, so an
un-flashed board was indistinguishable from a flashed one from the Jetson. Phase 4
shipped a firmware change the operator must flash, and there was no positive
confirmation that they had.

Phase 4 step 5 said "bump the firmware version **if** the Platform Teensy carries
one" and correctly declined to invent one — that is a protocol change an
implementer should not make unasked. It has now been asked for.

#### Phase 6 — Outcome

**Landed in source 2026-07-27, NOT FLASHED.** Contract:
`ros_ws/docs/platform_fw_version.md` (**C-PLATFW-1**). Bench:
`tests/hardware/session_anomaly_fixes.md` § Section FW, rows INST-4, INST-5,
FW-1 and H4.0d.

##### The report path — the one real design decision

The can-bridge reports its identity over its own USB serial console and over UDP.
The Platform Teensy has neither: it is reached only through the can-bridge over
CAN3, and there is no serial monitor attached during a launch. So the version had
to come back over that conduit. Candidates, judged on failure modes:

| candidate | why not / why yes |
|---|---|
| **bytes 5-6 of the 0x6E0 RobotState reply** (chosen) | Already a host-driven request/response the boot path runs. Decisively: **every firmware ever built zero-filled those bytes at the same dlc 8, so a pre-versioning board ANSWERS with 0** rather than going silent. The un-flashed case produces a definite wire value instead of a timeout |
| a dedicated version query (new CAN id / new RPC) | An old board would not answer, so the un-flashed signature would be *absence* — indistinguishable from a CAN3 hiccup, an unpowered board or a bridge not forwarding a new id. Also needs a new `PlatformCanId`, a new relay trigger, a new reply-id predicate entry, a `udp_protocol` regeneration and **a can-bridge flash**: a second silent deployment introduced to detect a silent deployment |
| the 0x7DF traffic report | Periodic and already on the bus, but no host consumer and not in the bridge's relay-reply set ⇒ can-bridge change + flash again |
| the 0x7DE tilt frame | Two float32s; full |
| serial banner only | Shipped as well (`[boot] jugglebot-platform v1`) and genuinely useful at flash time, but "invisible from the Jetson" is the whole defect |

Properties of the chosen path, all deliberate: **no new CAN frame** (nothing added
to the duty cycle of the bus the 0x6D0 hand conduit shares, so nothing can preempt,
delay, reorder or drop a hand command), **dlc unchanged at 8** (the bridge's
`(can_id, dlc)` correlator and the host's `expected_dlc=8` await are untouched),
**no can-bridge change and no can-bridge flash**, and **direction-safe** (the
bridge's `state_write` zero-fills 5-7 and `decodeStateCANMessage` never reads
them, so a host write cannot poison the reported version).

**One-shot per read, not polled.** The capture rides the existing RobotState reads
(boot, UDP reconnect, CAN3 recovery). A version changes only by a flash, and a
flash reboots the board; a dedicated poll would add CAN3 traffic for a constant.

##### Warn, never refuse

A skew produces an ERROR log line, a `link_status` field and two typed
`robot_state` fields. **Nothing is refused.** The reasons are concrete, not
cautious:

1. **A refusal on the hand dispatch path destroys the abort.** `SetHandTrajCmd`
   carries the kind-3 retract, and a kind-3 clobbering an armed kind-0 is the only
   un-arm mechanism the Teensy offers.
2. **Refusing kind-1 after kind-0 has flown drops a ball** the pre-fix stack was
   catching 15/19 of.
3. **The input can be legitimately unknown.** The version rides a cached relay read
   whose failure is a *documented benign transient* on this robot. `is_homed`'s safe
   fallback is "force a re-home"; there is no harmless fallback for "refuse all hand
   commands".
4. **A stale board is today's behaviour, not a new hazard.** `smoothMoveDuration`'s
   `v0 == 0` branch is bit-identical to the historical form, so a stale board
   differs only on a command landing mid-motion — which Phase 1 already prevents on
   the toss path. The cost is a mis-scored bench row: a diagnostic cost, fully
   addressed by a loud warning.
5. **A detector is monotonic; a refusal is not.** Enforcement can be added later on
   evidence; a blind refusal cannot be un-shipped from a board mid-sitting.

This also matches the pattern being mirrored: the can-bridge's own `FW_VERSION` is
documented as "a human-facing identity marker only — it has no runtime/handshake
effect", with wire compatibility enforced separately by `JbUdp::PROTOCOL_VERSION`.

**Open for the operator**: whether a *toss start* should refuse on a definite
`version == 0` — the shape `REJECTED_NOT_LEVELLED` already has. It is the only
refusal shape without one of the failure modes above, and it still needs an answer
for the UNKNOWN case.

##### Numbering

`0` is reserved for "pre-versioning" and is never a release — not a convention the
host imposes but what an un-flashed board physically sends. `1` (2026-07-27) is the
first numbered release and marks Phase 4's velocity-continuous `makeSmoothMove`
plus the identity block itself. The host's expected value is a **second,
independently-authored constant** (`rpc_args.PLATFORM_FW_VERSION_EXPECTED`): the
skew being detected is *board vs tree*, and a single codegen'd value would move in
the tree without the board being flashed — agreeing with itself in exactly the
situation the check exists to catch. `tests/firmware/test_platform_fw_version_xref.py`
pins the pair in both drift directions.

##### The compile gate, and why it does not flash

`Teensy_code/` had **no `platformio.ini`**, so nothing in the repository compiled
this sketch — `test_hand_smooth_move_xref.py` host-compiles `Trajectory.h` with
`g++`, which is real coverage of that header but not of the `.ino` against
FlexCAN_T4, SCL3300 and the generated config. Phase 4 rewrote `makeSmoothMove` and
could not build-check it at all. There is one now, and the shipped sketch compiles
and links for the Teensy 4.0 (102 720 B text).

It deliberately has **no `upload_command`**. The pio image is not the image this
board has been running: linking the sketch's `std::vector` needs
`-fno-exceptions` and a patched linker script (libgcc's `pr-support.o` emits a bare
`.ARM.extab` that falls outside the stock script's ITCM pattern and blows its
`R_ARM_PREL31` relocation — the can-bridge's diagnosis, retargeted from
`imxrt1062_t41.ld` to `imxrt1062.ld`). Switching the flash toolchain in the same
sitting that validates Phase 4 would confound the two: a misbehaving § CHECK HAND-4
could then be Phase 4 *or* the toolchain. Keep flashing from the Arduino IDE;
switch deliberately, on its own, with its own powered re-validation. As a bonus,
without `upload_command` a stray `pio run -t upload` fails on PlatformIO's
glibc-2.34 loader helpers rather than flashing the live board.

**Deployment:** `colcon build --packages-select jugglebot_interfaces jugglebot`
(`RobotState.msg` gained two fields) **+ relaunch**, and a **Platform Teensy
flash** — the same flash Phase 4 already required, now confirmable. No codegen run.

##### Phase 6 — Outcome (finalize, 2026-07-27)

**LANDED in source, NOT FLASHED.** Commit `bb15d9b` (the phase), plus a
follow-up SHA backfill. Logbook:
`logbook/2026-07-27-platform-teensy-fw-version.md`.

**Test triple**: `pytest tests/ -q`, run 2026-07-27 on the Jetson —
**3966 passed, 3 xfailed in 1411.32 s (23:31)**, exit 0. The pre-phase baseline
was `2b0f3f0` at **3947 passed, 3 xfailed**; the `+19` is exactly this phase's
new tests (10 xref + 8 coldstart from the implementer, 1 from finalize).
**xfail count unchanged at 3.** Neither allocation-budget flake failed.

**Three convergent reviewer findings were verified and fixed at finalize**, and
they are worth carrying forward because two of them are about *this phase's own
instruments* rather than its feature:

1. **A false claim in the code, repaired by making it true.** The comment
   justifying plain-attribute assignment of the new `RobotState` fields said a
   stale `jugglebot_interfaces` build makes the node "die loudly". It does not —
   `_publish_robot_state` catches its own exceptions, so the real signature is one
   throttled ERROR per 5 s plus a silently-dead `/robot_state`, with the node
   still listed and the orchestrator stalling in BOOT blaming power/CAN. Added
   `_warn_if_robot_state_msg_is_stale()` (construction-time, logs `INTERFACES_STALE`
   naming the missing fields and the rebuild command, never raises), corrected the
   comment and runbook row B, and added run-sheet row **FW-2**.
2. **The `UNKNOWN` verdict was routed as a hard fault on a healthy board.** It is
   the signature of this robot's documented benign boot-read transient. All
   operator-facing carriers now say *relaunch once and re-read, escalate only if
   it repeats*, with the co-signature to confirm. The code retry all three
   reviewers suggested was **declined**: it would also refresh the cold-start
   cache and flip `is_homed` mid-session. Recorded as an open question.
3. **The "warn, never refuse" test did not cover `teensy_hand_traj_cmd`** — the
   single funnel both hand services route through, and the natural site for a
   future gate. The test now drives both real service handlers and extends the
   tripwire to the funnel; validated two-sided (passes clean, fails on a gate at
   the funnel).

**Deferred operator handoff.** Nothing here has run on hardware. The operator
owns, in order: (a) at the desk, **INST-4** (`pytest tests/firmware/... -q`, zero
skips) and **INST-5** (`pio run`, `[SUCCESS]`); (b) `colcon build
--packages-select jugglebot_interfaces jugglebot` + source + **flash
`Teensy_code/Teensy_code.ino` to the PLATFORM Teensy from the Arduino IDE** +
relaunch; (c) at stage 3, **FW-1** (`grep PLATFORM_FW_CHECK`) and **FW-2**
(`grep INTERFACES_STALE`), and **H4.0d** before § CHECK HAND-4. All rows, with
numeric PASS/ABORT criteria, are in `tests/hardware/session_anomaly_fixes.md`
§ Section FW. **The check reports; it does not protect** — a
`0 (PRE-VERSIONING)` board will run the whole HAND-4 section and produce
plausible-looking rows.

**Open for the operator**: whether a toss start should refuse on a definite
`version == 0` (see § Warn, never refuse), and whether the `UNKNOWN` verdict
should become recoverable within a launch.

### Phase 7 — Post-release deceleration feedforward (added 2026-07-28, operator decision (b))

Added after the 2026-07-27 validation sitting, in which the hand made **light
physical contact with its mechanical end stop** on ~1.2 m throws. The operator
explicitly rejected the obvious remedy (capping throw height / `FLIGHT_TIME_MAX_S`)
in favour of a more aggressive post-release deceleration.

The physical insight that opened the design space: **after release the ball is
gone**, so the decel segment's only remaining constraints are motor authority and
the end stop — and the axis's reflected inertia there is unambiguous (rotor +
hand, no ball).

#### Phase 7 — Outcome

**Landed in source 2026-07-29, NOT FLASHED.** Commit `f920087`. Contract: `ros_ws/docs/hand_decel_feedforward.md`
(**C-HAND-2**). Probe: `tools/probes/hand_decel_authority.py`. Bench:
`tests/hardware/session_anomaly_fixes.md` § CHECK HAND-7 (stage 8, CAP-DECEL).
Logbook: `logbook/2026-07-29-hand-post-release-decel.md`.

**Tests.** Full suite `pytest tests/ -q`, run 2026-07-29 on the Jetson in the
project venv: **4096 passed, 3 xfailed in 1428.61 s (0:23:48)**, exit 0. That is
`+28` against the 4068-pass baseline at `ac74c1a` — this phase's two new files
(`tests/sim/test_hand_throw_decel_ff.py` 21, `tests/firmware/test_hand_throw_decel_xref.py`
7) — with the xfail count unchanged at 3 and no existing test modified.

**Deferred operator handoff.** Two deployments, and skipping either makes
§ CHECK HAND-7 meaningless: (1) a **Platform Teensy flash** of
`Teensy_code/Teensy_code.ino` (`FW_VERSION` 1 → 2) — not the can-bridge, not the
CatchingCone; (2) `colcon build --packages-select jugglebot` + source + relaunch,
for the regenerated `hardware_config.py` and `PLATFORM_FW_VERSION_EXPECTED`. Then
the **R0 → R5 ladder**, gated. Three desk pre-flights run *before* the flash:
H7.0a (the firmware xref with ZERO skips), H7.0b/INST-6 (`--self-check` on the
probe), and **H7.0c — read `axis0.config.torque_soft_min` off the live hand
ODrive**, which is the one that can invalidate the whole phase (see the review
findings below).

##### What the adversarial review changed before this landed

Three reviewers ran independently against the implementer's work; the findings
that survived verification and their dispositions are in the logbook entry's
*Findings adjudication* section. Four are worth naming here because they changed
what the bench will do:

1. **The safety clause was anchored on the wrong measurement.** Two lenses
   independently found that the "accel-phase torque balance → 1.015e-5 kg·m²"
   identification is **ball-inclusive** (release is at `x2`, so the ball is in the
   cup for the whole ascent; `INERTIA_RATIO = 0.747 = m_hand/(m_hand+m_ball)`
   makes the ball worth 2.412e-6 kg·m², 24 % of it). Ball-corrected it gives
   7.74e-6 — *below* the declared 9.5e-6 — so the claim that 9.5e-6 sat "7–10 %
   below BOTH identifications" was false. Re-anchored on a genuine **decel-side
   lower bound**, `J ≥ (τ_FF + τ_grav)/(2π·a_achieved) = 1.0126e-5`, which uses no
   `iq` measurement and so is immune to the telemetry aliasing. The test constant
   moved 1.015e-5 → **1.0126e-5** (tighter).
2. **The one-sided-safety proof omitted gravity.** On an upward decel gravity
   brakes in the *same* direction, worth `τ_grav/(2π·a_cmd)` — +1.46e-6 at the
   band floor. Open-loop the total therefore *exceeds* `J_true` below
   `a_cmd ≈ 1900 rev/s²`, so "it can never over-brake" is false at the bottom of
   the band. No declared value fixes it (8.69e-6 would satisfy the inequality but
   puts the pessimistic peak at **10.80 rev**, past the abort line), so the
   enforcement moved to where it belongs — bench row H7.4 — and the ladder gained
   a **band-floor rung R0**, the only rung that tests the over-braking direction.
3. **The verdict instrument was reading the wrong phase of the stroke.** Its
   `iq_ramp` window was anchored on the announcement's `throw_time`, which lands
   120.5–166.9 ms before the commanded stroke end while `t_dec` is 52.7–93.3 ms —
   **zero overlap with the real deceleration on 13 of 17 tosses**; it was
   reporting the ascent current. Window re-anchored on the commanded profile, and
   the probe gained a two-sided **`--self-check`** (INST-6) that scores a
   synthetic pre-fix capture FLAG and a post-fix one ACCEPT through the same
   `analyse()` the bench uses. Confirmed by mutation: restoring the old window
   makes the self-check FAIL.
4. **An unexamined drive-config clamp could make the whole phase a no-op.**
   `odrive_pro_hand_config.json` declares `axis0.config.torque_soft_min =
   −0.0551 N·m` = **exactly −10.00 A**, against a `torque_soft_max` of +0.5 N·m.
   If live it truncates the decel feedforward — legacy and corrected alike —
   above ~0.49 m. Counter-evidence says it is probably not binding (a hard clamp
   would force one achieved decel at every tier, and the measured value grows
   2.6× across the band), but it is unresolved, so it became pre-flight **H7.0c**
   plus a named failure-table row, and H7.3 is documented as its in-band
   discriminator.

Also corrected: the stale **"6055 rev/s² at the band top"** figure at its three
surviving sites (it is the decel at the `MAX_EVENT_VEL_MPS = 7.0` builder clamp,
not at `FLIGHT_TIME_MAX_S`, where the value is 3597 — the old figure asserted a
routinely-commanded decel *above* the axis's own 4178–4333 rev/s² ceiling); the
H7.3 flatness gate 0.25 → **0.35 rev** (0.25 sat at 95 % of the model's own
prediction of 0.237, a coin flip that would have blocked R5 on a working fix);
the `settle_offset` baseline restated **per tier** (the single span
"+0.019…+0.067" silently excluded the 4.858 m/s tier, the one this phase exists
for, where the offset is ~0); and two operator-facing sites still calling
`platform_fw_version = 1` the correct flashed reading.

##### The physics decided the fork, and it decided it against the obvious answer

The plan's brief offered three levers: a steeper commanded ramp, a computed
undershoot with a gentle approach back to `x3`, or a braking-torque feedforward
boost. Measured on the sitting's own capture
(`tools/probes/hand_decel_authority.py --trace temp/logs/toss_trace_2026-07-27_15-39-50.jsonl`):

| commanded release | commanded decel | measured peak | over `x3` | implied `eta` |
|---|---|---|---|---|
| 2.742 m/s | 929 rev/s² | 10.033 | +0.074 | 0.982 |
| 3.440 m/s | 1462 rev/s² | 10.022 | +0.063 | 0.985 |
| 3.969 m/s | 1947 rev/s² | 10.306 | +0.347 | 0.921 |
| 4.858 m/s | 2916 rev/s² | 10.980 | **+1.020** | **0.799** |

**Not authority-limited in the current sense**: the whole-session max `|iq|` is
**25.67 A against a 50 A limit**, with a smooth tail and no clipping, and the
achieved deceleration keeps *growing* (911 → 2330 rev/s²) rather than saturating.

**But steepening is still unavailable**, and that is what killed the first two
options. With the reflected inertia identified at 1.0126e-5–1.050e-5 kg·m², the
axis's decel ceiling at `hand_curr_limit_a = 50` is 4178–4333 rev/s², and the
commanded decel at the shipped `FLIGHT_TIME_MAX_S = 1.10 s` is already
**3597 rev/s² — 83–86 % of it**. The steepest commandable ramp shortens the decel
distance from 4.046 to ~3.48 rev, i.e. buys ~14 % of the overshoot: 1.02 → 0.88 rev
on the tier that already touched. It also moves the release point `x2` up the
stroke, changing the ball's release height.

The **computed undershoot** is mathematically the same lever (it shortens the
commanded decel distance) and inherits the same ceiling, while additionally
requiring the overshoot to be *predicted*. The measured overshoot at the tier that
touched scatters over 0.901–1.103 rev (0.202 rev on five throws), so no chosen undershoot both helps at the top of the band
and avoids driving the hand **below** `x3` at the bottom, where the plant already
tracks to `eta = 0.98`. That would re-create the operator-visible dip Phases 1–4
removed, on the *commanded* profile, where `dip_below_x3 <= 0.10 rev` cannot tell it
from the defect.

##### What was actually wrong

`accelToTorque(a) = a · INERTIA_HAND_ONLY_KG · HAND_SPOOL_RADIUS_M` models the axis
as a pure translating mass on a spool. It omits the motor's rotor entirely and uses
the raw spool radius rather than the effective one `LINEAR_GAIN_FACTOR = 1.035`
implies, so its implied reflected inertia is `m·r/(2π·LINEAR_GAIN)` = **7.3695e-6
kg·m²** against a measured **1.02e-5–1.05e-5**. The feedforward — the only term
that commands braking open-loop — delivered ~70 % of the required torque, and the
rest had to come from a loop with `pos_gain 35`, `vel_gain 0.007` Nm/(rev/s) and an
integrator unwind constant of **0.100 s**, against a decel ramp of 47.4–93.3 ms.

The geometry gives that shortfall a large lever arm: `calcThrow` allocates the decel
`IR·accelSt/(1+IR)` = **4.046 rev, velocity-independent**, ending exactly on `x3` =
the top of the usable stroke. So it budgets the ideal stopping distance with zero
allowance for tracking error, and `peak = x3 + 4.046·(1/eta − 1)`.

##### Why the declared inertia is deliberately too low

`9.5e-6`, 7–10 % below both identifications. The feedforward alone produces
`a_cmd · J_ff/J_true`, so while `J_ff <= J_true` it can **never** over-brake — the
hand cannot be commanded to stop short of `x3` and be dragged back up. That is
C-HAND-2's second obligation and it is what makes the change one-sided: it can only
reduce the overshoot.

##### The prediction is a bracket, not a number — deliberately

The plant is **not identifiable** from this capture to the accuracy a point
prediction needs: `hand_telemetry` is a ~100 Hz snapshot of a 500 Hz stream,
`iq_meas` has a median repeat-run of 4 samples (effective refresh 13–25 Hz), and a
cascade simulation on the shipped ODrive gains under-predicts the measured 4.86 m/s
peak by 0.87 rev while over-predicting the current by 40 %. So:

* **pessimistic** (feedforward is the only braking): `over = 4.046·(J_true/J_ff − 1)`
  — **velocity-independent**, 0.267–0.426 rev, peak ≤ **10.39 rev** at every speed;
* **optimistic** (loop keeps its measured ~348 rev/s²): ≈ 10.15 rev at the ceiling.

Against the runbook's 10.60 hard-abort line that is **0.21 rev = 6.7 mm** of margin
in the pessimistic bracket. The velocity-independence is the property that matters:
it is what removes the superlinear growth (+0.074 → +1.020 rev over 2.74 → 4.86 m/s)
that made the band ceiling unflyable. The bracket is honest about being pessimistic —
evaluated on the PRE-change feedforward it predicts +1.72 rev where +1.02 was
measured, so the loop closes ~40 % of it in practice. **The bench ladder is what
turns the bracket into a measurement**, and § CHECK HAND-7's flatness row **H7.3** is
its falsifiable form.

##### Scope, and why it is this narrow

Commanded position and velocity are **bit-identical on every kind**; only the throw's
decel-segment torque moved. Consequences that would otherwise have had to move in the
same commit and did not: C-HAND-1's stroke-busy window, `MIN_THROW_EVENT_DELAY_S`,
`_PRIME_INFLIGHT_S`, `hand_catch_prime_rev`, and `tools/probes/hand_stroke_timeline.py`'s
entire verdict model — including its byte-identical gate fixture. **The plan's
"how does the timeline probe model two profile generations" fork therefore does not
arise**: there is only one commanded profile generation.

The **ascent** feedforward is untouched on purpose: correcting it would make the hand
track the ascent better and so *raise the achieved release velocity*, re-calibrating
every throw height the machine has flown — on a machine whose hand already reaches
its end stop. Operator decision. `makeCatch` and `makeSmoothMove` are likewise
untouched, the latter because the kind-3 clobber is the only un-arm mechanism the
Teensy offers.

##### Two things found on the way, neither fixed here

1. **`buildCommand` (kind 2, `makeFull`) has a unit defect**: `accelToTorque(acc *
   LINEAR_GAIN)` feeds a rev/s² quantity into an m/s² conversion, so kind-2's torque
   feedforward is **31.6× too large**. No live host dispatches kind 2 (only
   `archived/`). Recorded in the code and in the contract; fix with a bench
   validation if kind 2 is ever revived.
2. **Every throw ends with a residual velocity feedforward.** `buildSegment` emits
   `while (t < end)`, so the last frame carries `|a_dec|·(t3 − t_last)` — up to
   **7.19 rev/s at the band ceiling** — which the drive latches until the next hand
   command, biasing the hand upward by up to `7.19/35 = 0.205` rev. Pinned with its
   exact bound rather than fixed: the residual is a property of `buildSegment`,
   shared with `buildCatch`, so removing it changes the commanded terminal frame of
   the catch descent that meets the ball. Bench row **H7.6**.
   *A hypothesis withdrawn in the process*: the settled `pos_meas − pos_cmd` offset
   (per tier, all 17 tosses: +0.0669…+0.0671 at 2.742 m/s, +0.0429…+0.0663 at
   3.440, +0.0190…+0.0412 at 3.969, **−0.0043…+0.0069 at 4.858**) was first
   attributed to this residual. The trace refutes it — the median `vel_ff_cmd`
   through those windows is 0.00–0.02 rev/s, predicting 0.0006 rev. The settled
   offset has another cause and is an open question. *(Quoted per tier since
   2026-07-29: the earlier single span "+0.019…+0.067" silently excluded the
   4.858 m/s tier — the one this phase exists for, and the one where the offset is
   ~0 — so a post-flash reading there had no valid baseline to compare against.)*

##### Deployment

**A Platform Teensy flash** (`FW_VERSION` **1 → 2**), plus `colcon build
--packages-select jugglebot` **+ relaunch** for the regenerated `hardware_config.py`.
No `jugglebot_interfaces` change.


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
| xref | `Teensy_code.ino`'s `FW_VERSION` == `rpc_args.PLATFORM_FW_VERSION_EXPECTED`; the shipped 0x6E0 codec compiled and run, packing the version where the host reads it |
| unit (mocked ROS) | a pre-versioning board (bytes 5-6 zero) is detected and surfaced on all three surfaces; UNKNOWN is not collapsed into it; a reboot/failed re-read does not erase a known version; a skew does NOT gate the hand dispatch path |
| build | `pio run` in `Teensy_code/` compiles and links the whole sketch |
| unit | C-HAND-2: declared decel inertia <= the measured reflected inertia (one-sided safety), > the legacy implied one, and reaching the mirror AND the shipped firmware header by three routes |
| xref | the SHIPPED `Trajectory.h` throw, compiled and run: decel torque on the corrected conversion, accel/hold on the legacy one, position and velocity streams unchanged, `buildCatch` untouched |
| unit | authority: peak commanded decel current inside `hand_curr_limit_a` with headroom, at both flight-band ends |
| drift-guard | `hand_catch_prime_rev` == derived stroke-top |
| hardware | Phase 5 |

Mocked-ROS tests cannot see the Teensy's queue semantics, which is where the
actual failure lives — so Phase 5 is not optional, and the Phase-0 probe is the
instrument that makes it readable.

## Risk register

| Risk | Mitigation |
|---|---|
| Delaying the arm eats the catch's own lead | Phase 1 step 3 checks the window at the shortest shipped flight and logs loudly if it closes |
| The deferral needs a later balls tick that never arrives (track dropout, or a landing revision pushing `event_delay` under the 0.3 s floor) — both bypass the gate, so even the closure branch cannot fire and the arm is silently never dispatched | Argued-against by the node's n = 6105 probed note (announced-vs-tracked landing agreeing to 0.000 s at the arm moment in early life), so instrumented rather than fixed: runbook row **H1.7** counts withheld lines with no matching dispatch. A non-zero count ⇒ replace the tick-driven retry with a one-shot timer |
| The forced (window-closed) dispatch is itself refused by `Teensy_code.ino:533`, because the fit check budgets the at-rest prelude and the hand is mid-stroke | Bounded, not eliminated: it is the pre-fix arithmetic exactly, and `:533` returns *before* `packedMsgs.clear()` so the live stroke survives — the cost is a lost catch, never a clobbered stroke. Runbook H1.6 reads the serial refusal; H1.4 warns that a clean dip row is not evidence the catch fired |
| `catch/vel_scale` closes the window on its own at the short-flight end (0.45 ⇒ −15 ms at T 0.557 s, inside the shipped `[0.3, 1.5]` range) | H1.4 tells the operator to read the knob before routing a CLOSED warning to a tracker fault; HAND-1b is pinned to the default 0.8 |
| A kind-3 abort retract stops clobbering an armed stroke | explicit exemption + a test; called out in both Phase 2 and the contract |
| Velocity-continuous prelude overshoots into the stroke end-stop | bound the excursion against 11.1 rev; decide the cannot-fit behaviour deliberately |
| Sim mirror is not faithful, so Phase 4's gate is illusory | Phase 0 verifies the mirror *before* it is trusted |
| Moving the prime rev trips park-band / parked-top windows | **CLOSED, did not bind** (Phase 3, 2026-07-26): every consumer enumerated and every window measured at both primes — tightest is the near-band's `0.2156 rev = 6.8 mm` pessimistic margin, bridge headroom `1.1406 rev`, and `hand_parked` is keyed to the *bottom* band so the kind-0 hazard never involved the prime. Nothing widened; margins pinned by `test_prime_move_leaves_the_park_band_windows_open`. The window the constant-grep sweep DID miss was `_PRIME_INFLIGHT_S` — sized from the ascent duration without naming the constant — now pinned too |
| Firmware/host version skew after Phase 4 | **CLOSED by Phase 6** (2026-07-27): `FW_VERSION = 1` declared in `Teensy_code.ino` and reported in bytes 5-6 of the 0x6E0 RobotState reply, so a pre-versioning board (which zero-fills those bytes) is a POSITIVE verdict rather than silence. Surfaced on `robot_state`, `link_status` and a `PLATFORM_FW_CHECK` log line; runbook rows FW-1 / H4.0d. Deliberately WARNS and never refuses — the same path carries the kind-3 retract (`ros_ws/docs/platform_fw_version.md`) |
| The corrected decel feedforward OVER-brakes, so the hand stops short of `x3` and is dragged back up — a commanded dip the C-HAND-1 gate cannot distinguish from the defect it exists to catch | **Structural, not tuned**: the feedforward alone produces `a_cmd·J_ff/J_true`, so declaring `J_ff` BELOW the measured reflected inertia makes over-braking impossible from the feedforward. Shipped 7–10 % low; pinned by `test_declared_inertia_cannot_over_brake`, and the int16 wire quantisation's worst case is bounded separately (`test_wire_quantisation_cannot_produce_a_visible_undershoot`). Bench row **H7.4**, whose instruction is *lower it, never raise it* |
| The peak is not predictable from the shipped loop model, so a bench ladder scored against a point prediction mis-scores a working fix | Stated as a BRACKET (pessimistic 10.39 rev, optimistic 10.15) with the reason it cannot be tighter recorded — 100 Hz aliased telemetry, `iq_meas` refreshing at 13–25 Hz. The ladder climbs per tier and stops at the first failure; **H7.3** turns the bracket's one real prediction (velocity-INDEPENDENT overshoot) into a falsifiable row |
| The corrected feedforward saturates the drive, leaving the loop no authority | Designed to 31.6 A at the tier that touched and 38.9 A at the band ceiling against a 50 A limit (22 % headroom); pinned at both band ends by `test_peak_decel_feedforward_current_stays_inside_the_shipped_limit`. Bench row **H7.5** aborts at 45 A, and raising `hand_curr_limit_a` is explicitly NOT the response |
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
  **flash**. Phase 6 is **both** — `RobotState.msg` gained two fields, so the build
  is `colcon build --packages-select jugglebot_interfaces jugglebot`, and the
  firmware half rides Phase 4's flash.
- Phase 6 is the reason a skipped Phase-4 flash is now visible. If you are
  debugging a HAND-4 row that "looks like the pre-fix baseline", read
  `link_status/platform_fw_version` **before** anything else: `0 (PRE-VERSIONING)`
  means the board was never flashed and every row in that section is meaningless.
  Nothing refuses on it, so the check has to be run, not relied on.
