---
title: "Unified 7-DoF planner Phase 0 — four probe decisions recorded: in-process QP, v1-exact Hermite, a prepared 7-frame bench sitting, owner-signed hand guards"
type: investigation
date: 2026-08-30
status: resolved
phase: "unified-7dof-planner — Phase 0"
related_plan: unified-7dof-planner.md
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/platformio.ini
  - ros_ws/src/jugglebot/Teensy_code_canbridge/README.md
  - tests/hardware/session_unified7_bus_headroom.md
  - plans/active/unified-7dof-planner.md
  - plans/active/leg-bus-frame-drops.md
  - plans/active/INDEX.md
  - logbook/2026-08-30-unified-7dof-planner-phase0-probes.md
  - logbook/INDEX.md
subsystem:
  - motion
  - can
tags:
  - trajectory
  - probes
  - firmware-bench
  - safety
  - performance
---

# Unified 7-DoF planner Phase 0 — four probe decisions recorded

## Summary

Phase 0 of [`plans/active/unified-7dof-planner.md`](../plans/active/unified-7dof-planner.md)
is the plan's desk phase: four probes whose recorded results are the promotion
gate (`proposed` → `active`) and the dependency of Phases 1 and 3. Three of the
four are **closed at the desk**; the fourth is **prepared and awaits one operator
bench sitting**.

1. **QP solver — IN-PROCESS numpy QP. The escape hatch (CasADi in a separate
   venv, bridged over ZMQ) is NOT taken.** Parity against the CasADi/IPOPT
   reference is 1.91e-4 mm / 2.47e-3 mm/s worst case over 6 cycles against
   bars of 1 mm / 10 mm/s; runtime under the **production** interpreter
   (`/usr/bin/python3`, 3.8.10) is p50 5.577 ms / p90 6.367 ms on n=40, against
   a < 10 ms bar. Two findings bind Phase 1: the naive KKT-plus-violated-boxes
   loop **cycles and returns silently wrong answers**, so the port must be a
   Goldfarb–Idnani dual active set; and IPOPT was caught **falsely refusing a
   feasible cycle**, a runtime-refusal class the escape hatch would have
   inherited.
2. **Hermite stroke fidelity — v1-exact reconstruction verified, with an honest
   scope split.** On knot-aligned piecewise cubics (what the unified QP emits
   *by construction*) v1-exact is float-exact: ≤ 3.6e-15 rev. On the **legacy**
   closed-form stroke it is ≤ 3.25 mm vs ≤ 11.84 mm for the forward-difference
   fallback — a 3.2–3.6× improvement, **not** exactness. The plan's "exact by
   design" claim applies to the planner's own output only.
3. **Bus headroom + frame-drop A/B — PREPARED, verdict PENDING the sitting.** A
   runtime-toggle BENCH firmware variant (`UNIFIED7_BENCH_BUILD`, default 0,
   console `bench7 on|off`) is built and byte-verified against the stock image,
   and the operator runbook is
   [`tests/hardware/session_unified7_bus_headroom.md`](../tests/hardware/session_unified7_bus_headroom.md)
   (28 rows). The FlexCAN P3/P4 desk verdict against the widened burst is
   **clean** and is recorded verbatim below.
4. **Hand guard constants — derived and OWNER-SIGNED-OFF 2026-08-30** (all four
   recommendations accepted): `MAX_DEVIATION_HAND_REV` 2.5 rev
   **velocity-compensated**, `MAX_LEAD_HAND_REV` 2.0 rev **freshness-aware**, a
   coupled fifth constant the plan did not list (`HAND_VELFF_LIMIT_RPS` 300),
   and **two-tier** planner caps (200/300 rev/s, 3500/3900 rev/s²).

Probe scripts are `/tmp/probe_*.py` throwaways, deliberately uncommitted per
[`tools/probes/README.md`](../tools/probes/README.md) — nothing here is a
reusable replay or scenario harness. **Interpreters are named on every triple**
because the whole point of probe 1 was that the production interpreter is not
the one CasADi lives in.

## Motivation

Phase 0 exists because three of the plan's architectural commitments rest on
numbers nobody had measured:

- Phase 1 ports `sim/juggle_planner/juggle_planner.py::plan_cup_cycle` — a
  CasADi/IPOPT program — into `ros_ws/.../motion/trajectory/`, which is
  **pure-Python, numpy-only, Python 3.8** by architectural rule. CasADi is not
  importable there. Either the problem's convex-QP structure survives a
  dependency-free re-implementation at real-time speed, or the phase ships a
  second process and a ZMQ hop.
- Phase 3 widens the bridge's 2 ms interp burst from 6 frames to 7. The
  jugglebot bus already carries a measured, unexplained per-axis encoder-frame
  drop (`plans/active/leg-bus-frame-drops.md`), and adding TX pressure to a bus
  whose drop mechanism is unconvicted is exactly the sequencing question that
  must be answered *before* the firmware work, not after it.
- Phase 3 also puts the hand on a streamed 500 Hz lane behind firmware guards
  that do not exist yet. The legs' constants (`MAX_DEVIATION`, the 0.10 rev
  lead clamp) are meaningless at 221 rev/s, and a wrongly-sized guard is either
  dead weight or a stall at 13× leg speed.

## Decision 1 — QP solver: in-process numpy QP

**Decision: implement the QP in-process, numpy-only, under the production
Python 3.8. The ZMQ-bridged-CasADi escape hatch is not taken.**

### Reference capture

(2026-08-30, `/home/jetson/Desktop/PDJ_venv/venv/bin/python /tmp/probe_qp_ref.py`,
wrote `/tmp/qp_ref_cases.npz`; **IPOPT wall times 0.176–0.558 s across 6 cases**)
— venv interpreter, **CasADi 3.7.2, numpy 1.24.4**. This is the authority the
port is measured against, frozen to a file so the parity run never needs CasADi.

### Parity

(2026-08-30, `/usr/bin/python3 /tmp/probe_qp_solve.py`, **worst-case max
deviation 1.91e-4 mm / 2.47e-3 mm/s across 6 cases**) — the **production ROS
interpreter**, Python 3.8.10, numpy 1.24.4, no CasADi on the path.

| Property | Result |
|---|---|
| Case set | both test-fixture families from `tests/sim/test_demo_juggle_planner.py` + 4 grid points |
| Bars (from the plan) | 1 mm position, 10 mm/s velocity |
| Worst deviation | **1.91e-4 mm**, **2.47e-3 mm/s** — 3–4 orders inside the bars |
| Box constraints | **ACTIVE on every case** (this is not a parity test of the unconstrained solve) |
| Active set | QP's matched IPOPT's saturated-constraint count on every case |
| Determinism | run twice, **bit-identical** |

That the boxes are active on all six cases matters: an equality-only KKT solve
would also have hit parity here and told us nothing about the part of the
problem that is hard.

### Timing (quiet machine, post-gate)

(2026-08-30, `/usr/bin/python3 /tmp/probe_qp_solve.py --time 800 --case
grid_p100_tall`, **n=40: cached-maps p50 5.577 / p90 6.367 / p99 16.100 /
max 28.642 ms**) and (2026-08-30, same script `--case fixture_level`,
**n=25: p50 4.233 / p90 4.623 / p99 12.318 ms**).

The plan's **< 10 ms steady-state** bar is judged **MET on p50 and p90**. The
p99/max tail is OS/interpreter jitter, not solver work: a gc-on/off experiment
moved p99 18.6 → 14.9 ms and max 124.8 → 25.1 ms **without touching p50**. A
tail that a garbage-collector toggle halves is not algorithmic. It also sits
inside Phase 4's ≤ 50 ms per-cycle budget, so it is a budget question already
answered, not a new risk.

Against IPOPT's 0.176–0.558 s on the same cycles, the in-process solver is
**~45–100× faster**.

### Two findings that BIND Phase 1

**(i) The naive active-set loop cycles.** The obvious implementation — solve
the equality-constrained KKT system, add every violated box to the working set,
repeat — **does not converge**. Working sets grew to rank 37 of 53, `cond(K)`
reached ~1e23, and the loop returned answers **124 m** from the reference
without raising anything. A silently wrong trajectory is the worst possible
failure mode for this code path, and it is reachable from a reading of the plan
text as written.

The working solver is a **Goldfarb–Idnani dual active set** (~60 lines): 1–13
iterations across the case set, monotone dual objective, finite termination by
construction. `np.linalg.solve` throughout — the `lstsq` fallback branch was
never reached on any case. **Phase 1's `cup_cycle.py` is bound to this
algorithm**; the plan's Phase 1 bullet has been amended to say so.

**(ii) IPOPT falsely refuses a feasible cycle.** On a tilted 0.55 s cycle IPOPT
returned `Infeasible_Problem_Detected`. The QP solves the same problem in **1
iteration** and verifies the solution feasible to **1e-14**. Reproducer:
`/tmp/probe_qp_false_infeasible.py`, reproduced under **both** interpreters.

This is not a curiosity — it is a runtime *refusal* class. Had the escape hatch
been taken, the planner would inherit a solver that intermittently refuses
feasible cycles at run time, on the ball's clock, with no local means of
distinguishing that refusal from a genuinely infeasible request. The finding
**strengthens** the in-process decision rather than merely being compatible
with it.

### Implementation notes worth carrying into Phase 1

- **The tilted detach cross-product is rank-2.** Written literally as three
  cross-product rows it is a rank-deficient equality block. It is enforced as
  **2 orthonormal rows perpendicular to the detach axis**, verified equivalent
  to the literal cross-product form to **≤ 3.3e-16**.
- **Equality rows are 2-norm-normalised.** Without it `cond(K)` runs 1e5–1e7,
  driven by the `dt³/6` coefficients in the interpolated-catch rows. This is
  conditioning hygiene, not cosmetics — it is what keeps `np.linalg.solve`
  sufficient and `lstsq` unreached.

## Decision 2 — Hermite stroke fidelity: v1-exact, with an honest scope split

(2026-08-30, `/usr/bin/python3 /tmp/probe_hermite_hand.py`, **two process
invocations byte-identical**) — production interpreter, Python 3.8.10,
numpy 1.24.4.

| Reconstruction target | v1-exact (`HAS_V1` set) | forward-difference fallback |
|---|---|---|
| **Knot-aligned piecewise cubics** (≤ one cubic per knot — the unified QP's output **by construction**) | **≤ 3.6e-15 rev (≤ 1.1e-13 mm)** — float-exact | n/a (not the shipping path) |
| **Legacy closed-form stroke** (phase boundaries at arbitrary times), worst over 250 grid phases × `event_vel` 3–7 m/s | **≤ 3.25 mm** | **≤ 11.84 mm** |

So: **v1-exact is exact where the plan says it is, and a 3.2–3.6× improvement
where it is not.** The plan's "exact by design" phrasing is correct only for
the planner's own output; on the legacy stroke, whose phase boundaries do not
land on knots, v1 carriage improves the error but does not remove it. The
entry states the distinction because T-H2's replay tolerance has to be set from
the second row, not the first.

**Side finding, and it is an independent corroboration.** The forward-difference
fallback rule **alone** saturates the 0.15 rev lead clamp modelled by the xref
reference (the live firmware's 0.10 rev saturates earlier still) at `v ≥ 4 m/s`
— against *perfect* feedback, i.e. with no telemetry latency in the model at
all. That is owner resolution 5 (v1 carriage in the v6 wire format) arriving
from a second direction, and it is a direct input to the lead-clamp derivation
in Decision 4.

**Velocity source: the analytic piecewise derivative.** Central differencing is
wrong at phase boundaries — it produced a 7.7e-4 rev/s excursion at `t = 0`,
where the true derivative is continuous and known in closed form.

## Decision 3 — bus headroom / frame-drop A/B: PREPARED, verdict PENDING

This probe cannot be answered at a desk. What Phase 0 delivers is the
instrument, the runbook, and the desk verdict on whether the widened burst
re-opens anything the FlexCAN provenance work closed.

### Deliverable A — the BENCH firmware variant

A **runtime-toggle** bench image, `UNIFIED7_BENCH_BUILD` (default **0**),
console command `bench7 on|off`, modelled on the existing `BENCH_SYSID_BUILD`
pattern. The 7th frame is
`encode_leg_setpoint(HAND_AXIS, axes[6].pos_rev, 0, 0)` emitted inside the
**same `s_output_enabled` gate** as the leg burst, classified `TxCls::LEGS`.

Stale-cache policy, and it is load-bearing for the experiment's validity:
**skip + count `unseen_skip`** before the first encoder frame from axis 6;
**hold-last-known + count `stale_hold`** afterwards. Skipping *later* would
silently self-cancel the A/B — every window in which the cache went stale would
quietly revert arm B to a 6-frame burst while still being recorded as 7.

Modified: `canbridge_config.h`, `leg_interp.h`/`.cpp`,
`Teensy_code_canbridge.ino`, `platformio.ini`, and the Teensy `README.md`.

**Why a console command and not an RPC.** The sitting already holds the
`pio device monitor` open — the `[cantx]` / `[canhealth]` / `[bench7]` lines
*are* the measurement channel, and they exist nowhere else. An additive
`MsgType` would buy nothing the console does not already give and would leave a
permanent bench entry in the generated protocol headers on **both** ends of the
link, for an experiment that runs once.

### Deliverable B — build proof

(2026-08-30, clean `pio` builds of both environments) —

| Image | md5 | Note |
|---|---|---|
| stock, with these edits in the tree | `b4caafd66944d563415839222c63402d` | **IDENTICAL** to the pristine-tree stock build |
| BENCH | `295b528a63b379ed903e8b90c51ae330` | +4096 text bytes; boot banner + 1 Hz `[bench7]` line |

The byte-identical stock image is the claim that matters: the edits are inert
in the shipping build.

**`FW_VERSION` is deliberately NOT offset for the bench image** —
`tests/firmware/test_bridge_fw_version_xref.py` pins it to
`teensy_link.rpc_args.EXPECTED_BRIDGE_FW_VERSION`. The consequence is stated
in the runbook rather than discovered mid-sitting: with the toggle off the
bench image has **no wire tell**, `BRIDGE_FW_CHECK` reads OK either way, the
console banner is the only authority, and **the end-of-sitting stock reflash is
the real control**.

### Deliverable C — the runbook

[`tests/hardware/session_unified7_bus_headroom.md`](../tests/hardware/session_unified7_bus_headroom.md),
28 rows, modelled on `session_err_timeout_bench.md`.

**Arms.** 6 vs 7 frames **interleaved on one boot** (A → B → A′ → B′, so the
comparison carries no uptime/temperature/seating confound and `tx_q_hwm`'s
boot-monotonicity is read as an increment), plus the `leg-bus-frame-drops`
§ 4.1 ODrive-cyclic-disable arm. That arm **mandates `odrivetool`**: the
`SDO_WRITE` RPC encodes `value` as float32, so zeroing an integer
`*_msg_rate_ms` works *by accident* (0.0f and 0u share a bit pattern) while the
**restore silently corrupts it** (10.0f is `0x41200000`, not 10).

The § 4.1 **halved-rate arm was NOT built.** That plan calls it "no firmware";
on the current architecture it is not — it means `INTERP_RATE_HZ` 500 → 250,
which halves the cadence of the lead clamp, the stroke clamp and the whole
500 Hz safety ladder. That is a control-path timing change and an owner
decision, not something to fold into a bus-load probe. Runbook row 16 leaves
the owner the explicit fly-without vs authorise-a-companion-build choice.

**Observables.** `/profile` `can1_util_pct` — noting that
`tools/probes/teensy_link_profiling/jetson/profile_monitor.py` **cannot run
during the sitting** (single-owner UDP link; it is the launch-less alternative
only); `BridgeTxDiag` 0x8D `tx_deferred` / `tx_q_hwm` under boot-monotone
reading discipline; the console `[cantx]` per-class census; and the § 2.4
per-window encoder-frame deficit — whose recipe is **written out in full in row
21 because no committed tool computes it** (the 2026-08-15 numbers came from an
ad-hoc reduction).

**Pinned predictions** (recorded before looking, per the plan): ~3500 fps leg
burst, ~65 % `can1_util_pct`, `tx_deferred` and `tx_q_hwm` increments **0**,
design-bound pending frames 8 → 9 against 16 mailboxes.

**Decision rule, verbatim from the plan:** *if the drop rate scales with TX
rate, the `leg-bus-frame-drops` source fix sequences before Phase 3.*

**Known hazard, stated up front:** an unpowered hand ODrive broadcasts no
encoder frames ⇒ `pos_timestamp_us` stays 0 ⇒ the 7th frame is never
transmitted ⇒ **arm B silently measures 6 frames while claiming 7**. The
`[bench7] unseen_skip` counter is the check, and it is the single most likely
way this sitting produces a confidently wrong number.

**The sitting is also FW 16's first flash** — the board has self-reported
`bridge_fw_version` 15 since ~2026-08-20 while the tree has carried 16
committed-unflashed, so the sitting inherits FW 16's own first-flash acceptance
(poller cadence, per-class deferral counters).

### Deliverable D — the P3/P4 desk verdict (verbatim)

> FW 14's P4 disarm still holds in the current tree — the `break;` sits at
> `lib/FlexCAN_T4/FlexCAN_T4.tpp:1206` inside the `frame.mb == -1` refill loop
> under its P4 marker at `:1178`, recorded in `PROVENANCE.md` § P4 and
> source-scan-pinned by `tests/firmware/test_flexcan_tx_defer_guard.py`. P3 is
> untouched by a widened burst: it is an RX-path guard whose IRQ-off cost is
> bounded by `CAN_RX_DRAIN_BUDGET = 32` sub-0.1 µs pops per 1 kHz tick against
> the FlexCAN FIFO's ~690 µs tolerance, and a 7th `set_input_pos` draws no
> reply, so the 7-frame arm adds zero RX traffic. A 7th frame per tick does not
> re-open TX deferral: the design bound on simultaneously-pending
> jugglebot-bus frames goes 8 → 9 (12 with a worst-case 3-frame hand dispatch)
> against FW 10's 16 TX mailboxes — `defer jb` stays 0 by construction and P4's
> branch stays unreachable, with four mailboxes of margin. That is an
> arithmetic claim, not a measured one, which is exactly why the sitting must
> read `defer jb` / `txq jb` / `[cantx] legs=` as increments per arm; if they
> ever move, P4's `break` means the blast radius is one late frame rather than
> one frame duplicated into sixteen mailboxes while `pop_front()` silently
> discards its successors.

## Decision 4 — hand guard constants (owner-signed-off 2026-08-30)

(2026-08-30, `python3 /tmp/probe_hand_guards.py`, **byte-identical double
run**). Presented to the owner via `AskUserQuestion`; **all four
recommendations accepted**.

| Constant | Value | Basis |
|---|---|---|
| `MAX_DEVIATION_HAND_REV` | **2.5 rev** (79.1 mm), **velocity-compensated both sides** | residual budget 1.996 rev at 221.3 rev/s with no torque FF |
| `MAX_LEAD_HAND_REV` | **2.0 rev** (63.3 mm), clamped against the **age-extrapolated** encoder (`fb + vel·age`) | requirement 1.553 rev, speed-independent |
| `HAND_VELFF_LIMIT_RPS` | **300** | the coupled 5th constant the plan did not list |
| `hand_vel_limit_rps` / `hand_vel_ceiling_rps` | **200 / 300** | +12 % over the C-HAND-3 certified 178.23 rev/s peak |
| `hand_acc_limit_rps2` / `hand_acc_ceiling_rps2` | **3500 / 3900** | under the C-HAND-2 authority bound 3925.5 rev/s² |

### `MAX_DEVIATION_HAND_REV` = 2.5 rev, velocity-compensated

The residual is computed **in the 500 Hz interp tick**, with the 10 Hz fault
task latching the E-stop off the tick's verdict. It ships **observe-first**:
report the max residual for a sitting, arm the trip at the second.

**The static worst-lag alternative was ruled out by arithmetic, not taste.**
Honest apparent deviation at peak speed sums to **8.6–11.3 rev**: knot age up
to 25 ms, feedback latency 10–20 ms, and 1.06 rev of P-path tracking error. A
static bound must sit above that with margin — **9.6–17.7 rev against a
10.8 rev stroke**. A guard whose trip threshold exceeds the entire mechanical
stroke is dead weight: it can never fire before the hard stop does.

**Architecture fact found while deriving it, and it changes the shape of the
answer:** `fault_machine.cpp:371-373` reads `u0` and the encoder as a
**same-iteration snapshot pair**. The 10 Hz task rate is therefore *detection
latency*, not additional command-vs-feedback skew. Sizing the bound for 100 ms
of extra skew — the natural reading of "a 10 Hz guard" — would have been wrong
by construction.

### `MAX_LEAD_HAND_REV` = 2.0 rev, freshness-aware

Two alternatives were priced and rejected:

- **Freshness-blind** (clamp against the raw last-known encoder value) needs
  **5.0 rev — 46 % of the stroke** — to avoid clipping legitimate commands. A
  clamp that loose is not a guard.
- **The naive leg-time-equivalent, 1.3 rev**, sits **below the 3.5 rev
  staleness term** and would clip **every peak-speed throw**. That is the S1/S2
  silent-stall failure mode — a freshness-blind clamp amplifying a stale
  anchor into a commanded stop — reproduced at **13× the speed**, on the axis
  that throws the ball.

A **hand lead-duty counter is required** alongside it. Non-zero duty during a
throw is a **hard-abort-the-sitting** signal, on the FW 14 arc's precedent
where clamp duty 0 was the acceptance criterion.

### `HAND_VELFF_LIMIT_RPS` = 300 — the constant the plan did not list

The legs' `LEAD_CLAMP_VELFF_LIMIT_RPS` is **3.5**. Applied to axis 6 unchanged
— which is what a mechanical port of the leg lane would do — it is a **51×
feedforward cut** on the hand, i.e. the velocity feedforward silently ceases to
exist on the fastest axis in the machine. 300 rev/s is above any planned hand
speed and below the **327.67 rev/s int16 wire saturation**.

### Planner caps, two-tier

The **limit/ceiling** pattern already used for the legs. `hand_vel_limit_rps`
**200** is +12 % over the C-HAND-3 certified peak of 178.23 rev/s; the
provisional 250 in the plan left a **40 % invisible band** between certified
and permitted. This matters more than it looks: **the planner cap is the only
practical overspeed guard on the hand today** — the firmware leg loop correctly
excludes axis 6, and the only other backstop is the ODrive's own 1200 rev/s
trip.

`hand_acc_limit_rps2` stays **3500** with a **3900** ceiling, under the
**C-HAND-2 authority bound of 3925.5 rev/s²**. The binding number is the
authority bound, **not** the measured 4178–4333 rev/s² axis ceiling — which
makes C-HAND-2 a *structural* constraint on the planner rather than a
characterisation note.

**Also required (does not exist today):** a `fault_machine` hand overspeed
guard at ~1.15× the ceiling.

## Discussion

### Why in-process QP beat the escape hatch

Three independent reasons, in ascending order of how much they would have cost
to learn later:

1. **Dependency-free.** `ros_ws/.../motion/` is pure Python by architectural
   rule; CasADi cannot be imported there. The escape hatch does not "add a
   dependency", it adds a *second process, a serialization format and a ZMQ
   hop* on the ball's clock — the `run_mpc.py` pattern, whose operational
   history in this repo is the reason the MPC chain is currently dormant.
2. **45–100× faster.** 0.176–0.558 s of IPOPT wall time versus 4–6 ms p50, and
   that is before the IPC round trip the hatch would add. A cycle planner that
   costs half a second cannot replan on the catch side, which Phase 4 requires.
3. **Immune to a demonstrated refusal class.** IPOPT returned
   `Infeasible_Problem_Detected` on a cycle the QP solves in one iteration and
   verifies feasible to 1e-14. The hatch would have carried that behaviour into
   the runtime. This third reason was not available when the plan was written —
   it is the probe paying for itself.

Note the honest shape of the argument: reason 1 alone was already decisive
under the architecture rules, but reasons 2 and 3 are what make the decision
robust to someone later proposing to relax rule 1.

### What was ruled out, and on what evidence

| Ruled out | Evidence |
|---|---|
| Naive KKT + add-violated-boxes active-set loop | Cycles; working sets to rank 37/53, `cond(K)` ~1e23, **silently wrong** answers 124 m off |
| Static worst-lag `MAX_DEVIATION_HAND_REV` | Would need 9.6–17.7 rev against a 10.8 rev stroke — cannot fire before the hard stop |
| Freshness-blind hand lead clamp | Needs 5.0 rev = 46 % of stroke to avoid clipping legitimate commands |
| Naive leg-time-equivalent lead clamp (1.3 rev) | Below the 3.5 rev staleness term ⇒ clips **every** peak-speed throw; the S1/S2 silent-stall mode at 13× speed |
| Central-difference hand velocities | Wrong at phase boundaries — 7.7e-4 rev/s excursion at `t = 0` |
| An RPC `MsgType` for the bench toggle | The console is already the sitting's measurement channel; an RPC leaves a permanent bench entry in generated headers at both ends |
| Offsetting `FW_VERSION` for the bench image | `tests/firmware/test_bridge_fw_version_xref.py` pins it; the console banner and the stock reflash are the controls instead |

### Tradeoffs accepted

- **The p99 timing tail.** p99 12–16 ms and a 28.6 ms max are accepted against
  a p50 of 4–6 ms. Justification: the tail responds to a **gc toggle**
  (p99 18.6 → 14.9, max 124.8 → 25.1, p50 unmoved), so it is scheduler and
  allocator jitter rather than solver work, and it sits inside Phase 4's
  ≤ 50 ms per-cycle budget. If Phase 4 later needs the tail, the lever is
  allocation discipline in the hot loop — a known technique in this repo — not
  a different solver.
- **v1-exact is not exact on the legacy stroke.** Accepted because the legacy
  closed-form stroke stays on the **Platform Teensy** engine until Phase 6
  retires it; the unified path never reconstructs it. The concrete benefit of
  measuring it anyway is that T-H2's replay tolerance now has an **honest
  prediction (≤ 3.25 mm)** instead of an inherited "exact by design" that would
  have been set as a tolerance and then mysteriously failed.
- **The halved-rate arm is deferred, not skipped.** Flying rows 11–15 gives a
  bridge-TX-rate A/B in the same direction as § 4.1's arm 2, just weaker
  (+17 % rather than −50 %). If the deficit moves on that weaker lever the
  hypothesis is convicted without a safety-ladder cadence change; if it does
  not, the owner can price the companion build against real evidence.
- **The guards ship observe-first.** A guard that has never seen the real
  residual distribution is as likely to nuisance-trip as to catch anything.
  Observe at sitting one, arm at sitting two.

## Deliverables

**In the tree (this commit):**

- `ros_ws/src/jugglebot/Teensy_code_canbridge/` — `canbridge_config.h`,
  `leg_interp.h`, `leg_interp.cpp`, `Teensy_code_canbridge.ino`,
  `platformio.ini`, `README.md`: the `UNIFIED7_BENCH_BUILD` variant, its
  `bench7` console command, the 7th-frame emit inside the `s_output_enabled`
  gate, the `unseen_skip` / `stale_hold` accounting and the 1 Hz `[bench7]`
  line. **Default off; the stock image is byte-identical to the pristine
  build.**
- `tests/hardware/session_unified7_bus_headroom.md` — the 28-row operator
  runbook (new).
- `plans/active/unified-7dof-planner.md` — Phase 0 marked
  `DESK-COMPLETE (bench arm pending)` with an Outcome paragraph; the probe-4
  spec's unsourced latency figure corrected; the signed-off two-tier caps
  noted; Phase 1's `cup_cycle.py` bullet bound to Goldfarb–Idnani.
- `plans/active/leg-bus-frame-drops.md` § 4.1 — points at the prepared runbook
  and records that the halved-rate arm needs an unauthorised companion build.
- `plans/active/INDEX.md`, `logbook/INDEX.md`.

**Not in the tree, by design:** `/tmp/probe_qp_ref.py`,
`/tmp/probe_qp_solve.py`, `/tmp/probe_qp_false_infeasible.py`,
`/tmp/probe_hermite_hand.py`, `/tmp/probe_hand_guards.py`, and
`/tmp/qp_ref_cases.npz`. Per `tools/probes/README.md` these are one-off
drivers, not reusable harnesses; the recipes that matter are written out here
and in the runbook.

## Verification

- **Inaugural baseline** — this is the plan's first phase, so the suite result
  is recorded as its baseline: (2026-08-30, `./run_tests.sh`, **6253 passed /
  4 skipped, 269.50 s parallel phase + empty serial phase, total 283 s —
  PASS**) at HEAD `a6877ebf468778270f026247b112c17ce6598b6b`. The nightly that
  morning: **GREEN 6685/6692 (2026-08-30T04:01)**.
- **Firmware natives after the BENCH edits**: (2026-08-30,
  `pytest tests/firmware/ -q`, **406 passed in 207.59 s**).
- **The four test locations that regex-parse `canbridge_config.h` /
  `leg_interp.*`**: (2026-08-30, `pytest tests/motion/test_leg_torque_ff.py
  tests/motion/test_kt_lib.py tests/ros/test_gui_geometry.py
  tests/teensy_link/ -q`, **561 passed in 13.06 s**). These are named
  explicitly rather than covered by a "firmware-only, so no Python tests" claim
  — they read the constants this change touches.
- Pre-commit gate (2026-08-30, `./run_tests.sh`, after the audit fixes): **6253
  passed, 4 skipped in 266.69 s** (parallel phase; serial phase empty), total
  279 s, PASS — identical count to the inaugural baseline.

## Withdrawn claims

- **"At 221 rev/s peak stroke speed, telemetry latency alone produces ~1 rev of
  apparent deviation"** (plan § 4, Phase 0 probe 4, as written 2026-08-29) —
  **WITHDRAWN, unsourced and too small.** 1 rev at 221.3 rev/s implies **4.5 ms**
  of latency. The measured end-to-end figure is **10–15.9 ms**
  (`logbook/2026-08-15-fw14-validated-arc-closed.md`, the FW 14 validation),
  i.e. **2.2–3.5 rev** from latency alone. The premise was load-bearing: it is
  the term that makes a static deviation bound look plausible, and at the
  corrected value the static bound is arithmetically dead. The plan text has
  been corrected in place with a pointer here.

## Citation drifts found and corrected this phase

- The frame-drop A/B **protocol** is `plans/active/leg-bus-frame-drops.md`
  **§ 4.1** (workstream B, "the cheap A/B"), not § 2.5. § 2.5 states the
  *hypothesis* the A/B discriminates; citing it as the protocol sends a reader
  to a paragraph with no arms in it.
- The PROFILE-counter reader is
  `tools/probes/teensy_link_profiling/jetson/profile_monitor.py`.
  `profile_session.py` is a **system-load** profiler and reads none of these
  counters.
- The "~1 rev from latency" figure was **unsourced** — see Withdrawn claims.
- The repo idiom for an operator sheet is `tests/hardware/session_*.md`
  (`session_err_timeout_bench.md` is the model the new runbook follows).

## Outcome

Phase 0 is **DESK-COMPLETE**, with one arm pending an operator sitting.

- **Phase 1 is cleared to start.** Its stated dependencies are Phase 0
  decisions 1 and 2, and both are closed. The port is bound to a
  Goldfarb–Idnani dual active set and to analytic piecewise-derivative hand
  velocities.
- **The promotion gate (`proposed` → `active`) and Phase 3 await probe 3's
  bench numbers.** The runbook, the image and the desk verdict are ready; the
  decision rule is pre-registered so the sitting cannot be read
  post-hoc.
- **The hand guard constants are owner-signed and no longer open forks** —
  Phase 3 implements numbers, not decisions.

## Open Questions

1. **A hand torque feedforward would drop tracking error 1.057 → 0.101 rev**
   and could tighten *both* firmware constants by roughly 40 %. Everything
   above was sized assuming **no** torque FF — deliberately, the safe side. If
   torque FF lands, the guards are worth re-deriving rather than inheriting.
2. **The § 2.4 per-window deficit reduction has no committed tool.** Row 21 of
   the runbook states the recipe in full, but promoting it to `tools/probes/`
   is what would make the sitting's answer reproducible rather than re-derived
   — the same gap that made the 2026-08-15 numbers an ad-hoc reduction.
3. **The `INTERP_RATE_HZ` 250 companion build is unauthorised.** It is an
   owner decision (a 500 Hz safety-ladder cadence change), and runbook row 16
   frames it as fly-without-first.
4. **Observe-then-arm sequencing for the hand guards** needs a named second
   sitting. Shipping observe-first is only safe if the arming actually happens;
   an observe-only guard left in place indefinitely is a guard that does not
   exist.
