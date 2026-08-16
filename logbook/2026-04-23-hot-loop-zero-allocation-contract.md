---
title: Hot-loop zero-allocation contract (W1 inventory → contract → enforcement → fixes)
type: investigation
date: 2026-04-23
status: resolved
phase: "hardware-bringup — GC-pause elimination on the MPC 40 Hz hot loop"
related_plan: "hardware-bringup.md"
related_issues:
  - GC_PAUSE_CANDIDATE
related_entries:
  - 2026-04-18-mpc-overhead-spikes-fallback-bursts
  - 2026-04-20-k1-k6-reference-feasibility-resolution
sessions:
  - mpc_20260423_184647.csv   # W6 hardware validation — 60 s hold at (0,0,220)
files_changed:
  - controller/HOT_LOOP_CONTRACT.md
  - controller/hot_loop_contract.py
  - controller/runner.py
  - controller/telemetry.py
  - controller/mpc.py
  - controller/target.py
  - controller/zmq_target.py
  - controller/hardware_plant.py
  - ros_ws/src/jugglebot/jugglebot/motion/ipc.py
  - run_mpc.py
  - tests/sim/test_hot_loop_allocation_contract.py
  - tests/sim/test_hardware_plant_deadband.py
  - tests/sim/test_plant.py
  - tests/sim/test_zmq_target.py
  - tests/sim/test_diagnose_oscillation.py
  - sim/analysis/diagnose.py
  - sim/main.py
  - logbook/INDEX.md
  - plans/archived/hot-loop-zero-allocation-contract.md
commits:
  - ec08312   # W1–W4d: audit + contract + enforcement + telemetry/runner/hook pre-alloc
  - f38abd3   # W5: gc.disable() wrapper + T-I4 in-tick GC assertion
  - 306d111   # W4a: mpc.solve buffer pre-alloc
  - fb7e20b   # W4b+c: target sources, HardwarePlant, ipc pre-alloc
  - a52c158   # W7: threshold ratchet 1024 → 256 B/tick
  - 149070d   # W4d hook fix: closure augmented-assign shadowed free var
  - 8cab8fe   # W6: hardware validation + close-out (Gen-2 cadence bump 200 → 1200)
  - e1971eb   # Pass 1 audit: TelemetryLogger.load() fallback AttributeError fix
  - 27755eb   # Pass 1 audit: GC-race hardening, dead _dash_sink, doc/comment sync
  - 7c44eb8   # Pass 2 audit: last_{ref,twist,accel}_traj_view — non-copying hot-path reads
  - a60cf49   # Pass 2 audit: HardwarePlant pre-alloc + hardware-path contract test
  - 527b3f1   # Pass 1 + Pass 2 audit close-out (this entry)
  - 3082ff5   # W6 follow-up: diagnose.py chatter floor — sub-LSB IPOPT noise, not GC gaps
  - ba06849   # Chatter follow-up correction (this entry)
subsystem:
  - controller
  - mpc
tags:
  - performance
  - observability
  - control
  - contract
---

# Hot-loop zero-allocation contract

## Summary

Every hardware MPC session's diagnosis JSON flags 1–3 `GC_PAUSE_CANDIDATE`
spikes — 40–67 ms single-sample overhead blips that force the loop into
walk-forward fallback for that tick and leave a visible cmd discontinuity in
telemetry.  W7 of the K1–K6 contract (see
[2026-04-20-k1-k6-reference-feasibility-resolution](2026-04-20-k1-k6-reference-feasibility-resolution.md))
now handles the resulting fallback *safely*, but the cmd discontinuity
itself still shows in telemetry and contributes to hold-phase HF noise
visible on the ODrive `pos_setpoint` scope.

Mechanism: Python's generational GC fires when per-generation allocation
counts cross thresholds; a Gen 2 sweep of CasADi solver state + numpy
arrays on the Jetson Orin Nano takes 40–80 ms — well over the 25 ms tick
period.  This entry is the architecture fix: eliminate the hot-loop
allocations that drive Gen 2 pressure, then `gc.disable()` the control
loop so any remaining collections run between ticks on the idle sleep
rather than during the solve.

Planned arc (W1 → W6; each W is its own commit):

| W | Scope |
|---|-------|
| W1 | **This entry** — audit every allocation site in the 40 Hz hot loop, sort by frequency, no code changes yet. |
| W2 | `controller/HOT_LOOP_CONTRACT.md` — normative spec (invariant + enforcement mechanism + implementation guidance). |
| W3 | `tests/sim/test_hot_loop_allocation_contract.py` — tracemalloc-based enforcement test that fails CI if any future-added call allocates in the hot path. |
| W4 | Pre-allocation fixes for every W1 site, one bundle per file/subsystem. |
| W5 | `gc.disable()` wrapper in `run_mpc_loop` with periodic `gc.collect()` scheduled between ticks. |
| W6 | Hardware validation — target zero isolated spikes in a 60 s run. |

## Problem

**The 40 Hz hot loop** is defined as the body of `run_mpc_loop`
([controller/runner.py:322-488](../controller/runner.py#L322-L488))
from `state = plant.get_state()` through `log_mpc_step(...)`.  Every step
inside that body runs once per tick in steady state.

**Observed symptom (hardware):**
- Diagnosis output: `overhead.isolated_spikes` = 1–3 per session, each
  40–67 ms.  Signature: `GC_PAUSE_CANDIDATE` — overhead > 40 ms with
  solve_time < 25 ms (not a solver overrun) and concurrent GC callback
  events (`gc_ms > 30`).
- Per-spike telemetry: `_handle_failure` fires on that tick (walk-forward
  fallback), cmd jumps to `prev_w[6k:6(k+1)]` for the cursor `k`, and
  the next tick has to rejoin the plan.  The position and velocity
  reference curves have a 1-sample notch visible in telemetry CSVs.

**Mechanism:** Python's generational GC (`gc.get_threshold()` default:
`(700, 10, 10)`) tracks per-generation allocation counts.  On the Jetson,
Gen 0 sweeps are sub-ms and fine, Gen 1 sweeps are 2–5 ms and usually fit
the budget, but **Gen 2 sweeps walk all container objects in the entire
interpreter** — CasADi's internal solver state (Function objects, DM
matrices) plus our per-tick numpy arrays.  On the Orin Nano that's
40–80 ms.

**Why the K1–K6 work didn't fix it:** W7 made the fallback *safe*
(walk-forward is no longer unsafe on ref shifts), but it didn't
*eliminate* the trigger.  The trigger is the GC pause itself; eliminating
it requires eliminating the allocations that drive Gen 2 pressure.

## W1 — Allocation inventory

This section is the full checklist of allocation sites on the steady-state
hot path, sorted by frequency: per-tick (every 25 ms), per-target-change
(on source.update() cache rebuild), and per-fallback (when the solver
status is fallback / hold / cold_hold).

Layout: one row per distinct allocation site.  `Loc` is the file:line at
which the allocation occurs (or the closest instructive line).  `Bytes`
is an order-of-magnitude Python-visible size per firing; sizes dominated
by CasADi-internal C buffers are not counted (Python tracemalloc will not
see them regardless).

N=10, so N+1=11 nodes.  n_w=180, n_param=228.

### Per-tick (fires every 25 ms in steady state)

#### runner.py: `run_mpc_loop` body

| # | Loc | Site | Bytes/tick | Notes |
|---|-----|------|-----------:|-------|
| R1 | [runner.py:420-429](../controller/runner.py#L420-L429) | `extras = {...}` dict, 8 keys | ~400 | Fresh dict every tick — rebuilt, passed to `log_mpc_step`, dropped. |
| R2 | [runner.py:435-439](../controller/runner.py#L435-L439) | `log_mpc_step(..., **extras)` | kwargs-expansion tuple | Small; materialises `extras` for the call. |
| R3 | [runner.py:450-468](../controller/runner.py#L450-L468) | `OH SPIKE` f-string | — | Conditional (>15 ms only); not steady-state. |

**Subtotal (steady state, runner.py):** ~400 bytes/tick from R1.

#### runner.py: `mpc_solve` and `log_mpc_step`

| # | Loc | Site | Bytes/tick | Notes |
|---|-----|------|-----------:|-------|
| R4 | [runner.py:158](../controller/runner.py#L158) | `np.zeros(6)` fallback ref_twist | 96 | Only when `last_twist_traj is None` — never after first solve. |
| R5 | [runner.py:185](../controller/runner.py#L185) | `np.zeros(6)` fallback ref_twist in log | 96 | Similar — only if caller didn't pass ref_twist. |
| R6 | [runner.py:182-200](../controller/runner.py#L182-L200) | `record_from_arrays(...)` → new `StepRecord` | **~1500** | Dataclass, 55+ scalar fields, `__dict__`-based — big. Fires **every** tick. |
| R7 | [telemetry.py:145-146](../controller/telemetry.py#L145-L146) | `np.linalg.norm(actual_pose[:3] - ref_pose[:3])` | ~200 | Intermediate (3,) diff array + reducer. |
| R8 | [telemetry.py:146](../controller/telemetry.py#L146) | `np.degrees(np.linalg.norm(...))` | ~200 | Same. |
| R9 | [telemetry.py:208](../controller/telemetry.py#L208) | `self._records.append(record)` | amortised | Python list growth; O(1) amortised. |

**Subtotal (steady state, telemetry):** ~1900 bytes/tick (R6 dominates).

#### mpc.py: `solve()` body — success path

| # | Loc | Site | Bytes/tick | Notes |
|---|-----|------|-----------:|-------|
| M1 | [mpc.py:913-914](../controller/mpc.py#L913-L914) | `_build_reference()` → `_build_reference_from_events()` | see M2–M8 | Every tick — rebuilds 3 per-node arrays. |
| M2 | [mpc.py:168](../controller/mpc.py#L168) | `ref_traj = np.empty((N+1,6))` | 528 | 11×6 float64. |
| M3 | [mpc.py:169](../controller/mpc.py#L169) | `twist_traj = np.empty((N+1,6))` | 528 | 11×6 float64. |
| M4 | [mpc.py:170](../controller/mpc.py#L170) | `accel_traj = np.zeros((N+1,6))` | 528 | 11×6 float64. |
| M5 | [mpc.py:171](../controller/mpc.py#L171) | `accel_filled = np.zeros(n_nodes, bool)` | 11 | Small. |
| M6 | [mpc.py:175](../controller/mpc.py#L175) | `ev_times = np.array([e.time for e in events])` | list-comp + small array | 2–5 events typical. |
| M7 | [mpc.py:176](../controller/mpc.py#L176) | `ev_poses = np.array([e.pose for e in events])` | list-comp + (k,6) array | Same. |
| M8 | [mpc.py:177-179](../controller/mpc.py#L177-L179) | `ev_twists = np.array([e.twist if ... else np.zeros(6) for e in events])` | list-comp + (k,6) array; np.zeros(6) per None-twist event | 2–5 extra (6,) allocations in the None branch. |
| M9 | [mpc.py:180](../controller/mpc.py#L180) | `ev_accels = [e.accel for e in events]` | list-comp | List with 2–5 references. |
| M10 | [mpc.py:923](../controller/mpc.py#L923) | `q_cur = state.leg_extensions_mm.copy()` | 96 | (6,) defensive copy. Used to feed `_handle_failure` on fallback; not strictly needed on success. |
| M11 | [mpc.py:995-1009](../controller/mpc.py#L995-L1009) | `np.asarray(sol['x']).ravel()` | ~1500 | New (n_w=180,) float64 from CasADi DM. |
| M12 | [mpc.py:1016-1017](../controller/mpc.py#L1016-L1017) | `np.asarray(sol['lam_g']).ravel()` + `lam_x` | 2×~1500 | Same — dual variables. |
| M13 | [mpc.py:1023-1024](../controller/mpc.py#L1023-L1024) | `ref_traj[mid_k].copy()`, `twist_traj[0].copy()` | 192 | Two (6,) snapshots for W7. |
| M14 | [mpc.py:1027-1029](../controller/mpc.py#L1027-L1029) | `cmd = w_opt[:6].copy()`, `cmd_next`, `cmd_next2` | 3×96 | Three (6,) slices → new arrays. |
| M15 | [mpc.py:1031](../controller/mpc.py#L1031) | `self._prev_prev_u = prev_u.copy()` | 96 | (6,) rotate-history copy. |
| M16 | [mpc.py:1038](../controller/mpc.py#L1038) | `g_vals = np.asarray(sol['g']).ravel()` | ~1500 | Full g-vector. |
| M17 | [mpc.py:1039-1048](../controller/mpc.py#L1039-L1048) | `g_viol`, `x_viol`, `violation` via `np.maximum(...)` | ~6×(size) | 4–6 same-size temporary arrays just for the `constraint_violation` scalar. |
| M18 | [mpc.py:1059](../controller/mpc.py#L1059) | `cmd_vel = (cmd_next - cmd) / dt0` | 96 | (6,). |
| M19 | [mpc.py:1061-1069](../controller/mpc.py#L1061-L1069) | `diag` return dict, 7 keys | ~400 | Fresh dict every tick. |

**Subtotal (steady state, mpc.solve):** ~13 000 bytes/tick (dominated by
M11/M12/M16 — `np.asarray(sol[...]).ravel()` over n_w / n_g arrays — and
M17's chain of `np.maximum` temporaries).

#### StaticTargetSource.update() (steady state, target unchanged)

| # | Loc | Site | Bytes/tick | Notes |
|---|-----|------|-----------:|-------|
| T1 | [target.py:697](../controller/target.py#L697) | `prev_target = self._target.copy()` | 96 | **Unused!** `prev_target` is never read.  Pure dead allocation. |
| T2 | [target.py:728-733](../controller/target.py#L728-L733) | `TargetCommand(...)` | ~400 | Non-frozen dataclass, 6 fields; ref-only field population — no per-field copies. |

**Subtotal:** ~500 bytes/tick on the steady-state `StaticTargetSource` path.

#### ZmqTargetSource.update() (steady state, no new target)

| # | Loc | Site | Bytes/tick | Notes |
|---|-----|------|-----------:|-------|
| Z1 | [zmq_target.py:177](../controller/zmq_target.py#L177) | `messages = self._ipc.recv_all()` → list | ~56 | Empty list allocation per tick (no pending messages in steady state). |
| Z2 | [zmq_target.py:276-280](../controller/zmq_target.py#L276-L280) | `request_key = (target_pose.tobytes(), arrival_time, source)` | ~120 | 48-byte bytes (6 float64) + tuple. |
| Z3 | [zmq_target.py:361-369](../controller/zmq_target.py#L361-L369) | Post-terminal rebuild: `[ReferenceEvent(time=..., pose=.copy(), twist=np.zeros(6), accel=np.zeros(6))]` | ~300 | Fires only when `sim_time > events[-1].time` — end-of-quintic hold.  Per-tick while holding a pose (rare after K1–K6). |
| Z4 | [zmq_target.py:371-378](../controller/zmq_target.py#L371-L378) | `TargetCommand(target_pose=self._target_pose.copy(), ..., target_twist=.copy() if ...)` | ~500 | Fresh dataclass + 1–2 (6,) copies. |

**Subtotal:** ~700–1000 bytes/tick.

#### HardwarePlant.get_state() (steady state, fresh motor feedback)

| # | Loc | Site | Bytes/tick | Notes |
|---|-----|------|-----------:|-------|
| H1 | [hardware_plant.py:353](../controller/hardware_plant.py#L353) | `frames = self._sub.recv_multipart(...)` | variable | One list-of-bytes per drained msg (~10–15 msgs at 500 Hz / 40 Hz). Each frame ~200 bytes serialised. Deser (msgpack) allocates the dict. |
| H2 | [hardware_plant.py:386](../controller/hardware_plant.py#L386) | `pos_rev = np.array(motor_pos, dtype=float)` | 96 | From list → (6,). |
| H3 | [hardware_plant.py:387](../controller/hardware_plant.py#L387) | `ext_mm = pos_rev / mm_to_rev` | 96 | (6,) broadcast. |
| H4 | [hardware_plant.py:389](../controller/hardware_plant.py#L389) | `self._last_state_ext_mm = ext_mm.copy()` | 96 | (6,) cache. |
| H5 | [hardware_plant.py:395](../controller/hardware_plant.py#L395) | `vel_rps = np.array(motor_vel, dtype=float)` | 96 | (6,). |
| H6 | [hardware_plant.py:396](../controller/hardware_plant.py#L396) | `vel_mmps = vel_rps / mm_to_rev` | 96 | (6,). |
| H7 | [hardware_plant.py:415-419](../controller/hardware_plant.py#L415-L419) | `leg_lengths_to_pose(...)` | ? | FK inner returns fresh `pos_offset`, `rot_matrix`, `fk_jacobian`.  Inside FK: temporaries per Newton iter; tracked separately but CPU-dominated at ~200 µs/iter rather than alloc-dominated. |
| H8 | [hardware_plant.py:420](../controller/hardware_plant.py#L420) | `rot_vec = rot_matrix_to_rotvec(rot_matrix)` | 24 | (3,) new. |
| H9 | [hardware_plant.py:424](../controller/hardware_plant.py#L424) | `self._fk_last_guess = (pos_offset, rot_matrix)` | 56 | Small tuple. |
| H10 | [hardware_plant.py:476-477](../controller/hardware_plant.py#L476-L477) | `pos_tuple = tuple(motor_pos)` | ~112 | (6,) tuple alloc for the frozen-motor-pos detector. |
| H11 | [hardware_plant.py:541](../controller/hardware_plant.py#L541) | `J_norm = J.copy()` | 288 | (6,6) float64. |
| H12 | [hardware_plant.py:543](../controller/hardware_plant.py#L543) | `twist_scaled = np.linalg.solve(J_norm, vel_mmps)` | ~200 | LAPACK returns (6,) and its own temporaries. |
| H13 | [hardware_plant.py:552-560](../controller/hardware_plant.py#L552-L560) | `return PlantState(...)` | ~300 | Fresh dataclass with 9 fields. |

**Subtotal (steady state, get_state):** ~1600 bytes/tick + ZMQ-drain list
allocations (H1) that scale with drain count.

#### HardwarePlant.command() (every tick)

| # | Loc | Site | Bytes/tick | Notes |
|---|-----|------|-----------:|-------|
| C1 | [hardware_plant.py:251](../controller/hardware_plant.py#L251) | `ext_mm = np.asarray(leg_extensions_mm, dtype=float)` | 0 or 96 | Zero if caller already float ndarray; MPC always passes one. |
| C2 | [hardware_plant.py:261](../controller/hardware_plant.py#L261) | `delta = np.abs(ext_mm - self._last_sent_ext_mm)` | 192 | 2 × (6,) temporaries. |
| C3 | [hardware_plant.py:269](../controller/hardware_plant.py#L269) | `motor_rev = ext_mm * mm_to_rev` | 96 | (6,). |
| C4 | [hardware_plant.py:300-302](../controller/hardware_plant.py#L300-L302) | `acc_mm_s2 = (ext_mm - 2·prev + prev_prev) / (dt²)` | 384 | Several (6,) temporaries from broadcast. |
| C5 | [hardware_plant.py:308](../controller/hardware_plant.py#L308) | `self._prev_cmd_ext_mm = ext_mm.copy()` | 96 | (6,). |
| C6 | [hardware_plant.py:317-327](../controller/hardware_plant.py#L317-L327) | `msg = make_mpc_command(...)` | ~500 | Fresh dict, up to 9 keys, from ndarray refs. |
| C7 | [hardware_plant.py:328-329](../controller/hardware_plant.py#L328-L329) | `self._pub.send_multipart(_pack(...))` | list + bytes | `_pack` returns `[topic, msgpack.packb(msg,...)]` — 2-element list plus a packed bytes object (~150–300 bytes). |
| C8 | [hardware_plant.py:333](../controller/hardware_plant.py#L333) | `self._last_sent_ext_mm = ext_mm.copy()` | 96 | (6,). |

**Subtotal (command):** ~1400 bytes/tick + msgpack.packb() output.

#### hooks.on_pre_command (run_mpc.py `_on_pre_command`, fires every tick)

| # | Loc | Site | Bytes/tick | Notes |
|---|-----|------|-----------:|-------|
| K1 | [run_mpc.py:467-469](../run_mpc.py#L467-L469) | `twist = (poses[1] - poses[0]) / dt0`, `twist_next`, `accel` | 3×96 | Three (6,) intermediates. |
| K2 | [run_mpc.py:470](../run_mpc.py#L470) | `plant_.set_pose(poses[0], twist_6dof=twist, accel_6dof=accel)` | ? | `set_pose` is part of the torque-FF pipeline; its own allocations (Newton-Euler on (3,3)) are out of this W1's explicit scope but are a downstream target for W4. |

**Subtotal:** ~300 bytes/tick + set_pose internals.

#### hooks.on_target_override / on_post_solve / on_log_extras

Steady state: `_on_target_override` returns `tc` unchanged (no allocation);
`_on_post_solve` short-circuits on non-catch source; `_on_log_extras`
builds a fresh 2-key dict (~200 bytes/tick).

| # | Loc | Site | Bytes/tick | Notes |
|---|-----|------|-----------:|-------|
| X1 | [run_mpc.py:488-492](../run_mpc.py#L488-L492) | `_on_log_extras` returns 2-key dict | ~200 | Merged into `extras` via `extras.update(...)`. |

### Per-target-change (only on new distinct target / new waypoint)

Fires once per target transition.  These do NOT run every tick — they
fire when the source's dirty bit is set (StaticTargetSource: target pose
changed; ZmqTargetSource: new `mpctgt` message distinct from the
cached one; Waypoint/AutoSequence: index advance).

| # | Loc | Site | Notes |
|---|-----|------|-------|
| P1 | [target.py:205-216](../controller/target.py#L205-L216) | `flat_target_to_events`: `np.asarray(...).copy()` ×3 + `np.clip` + proposal list of 2 `ReferenceEvent`s, each with 3 × (6,) copies | Anchor + target event — many small (6,) copies. |
| P2 | [target.py:406-433](../controller/target.py#L406-L433) | `make_feasible_events`: per-event `np.asarray(...).copy()` for pose/twist/accel | K6 clamp requires fresh writable twist; K1 anchor prepends a new event. |
| P3 | [target.py:515-519](../controller/target.py#L515-L519) | Cascade stretch: rebuilds each shifted event as a fresh `ReferenceEvent` | One per post-stretch event. |
| P4 | [target.py:725-726](../controller/target.py#L725-L726) | `self._prev_ref_horizon_end_pose = new_end_pose.copy()`, `_twist.copy()` | W5 bookkeeping. |

**Per-target-change bytes:** ~2–4 KB on target transition (rare — once
per move in StaticTargetSource; at catch-coordinator rate in Zmq).

### Per-fallback (only on solver failure; rare but impactful)

Fires on `_handle_failure` path.  Each allocation here is worth watching
because fallback ticks are exactly the ones that line up with GC pauses.

| # | Loc | Site | Notes |
|---|-----|------|-------|
| F1 | [mpc.py:1391-1398](../controller/mpc.py#L1391-L1398) | `diag = {...}` | Fresh dict per fallback tick. |
| F2 | [mpc.py:1449](../controller/mpc.py#L1449) | `cmd = np.clip(self._prev_w[6k:6(k+1)], margin, stroke-margin)` | (6,) new. |
| F3 | [mpc.py:1458](../controller/mpc.py#L1458) | `cmd = np.clip(cmd, prev_u - max_delta, prev_u + max_delta)` | More (6,) temporaries. |
| F4 | [mpc.py:1464-1467](../controller/mpc.py#L1464-L1467) | `cmd_next = np.clip(...)` and `cmd_vel = (cmd_next - cmd) / dt0` | 2 × (6,). |
| F5 | [mpc.py:1469](../controller/mpc.py#L1469) | `self._prev_prev_u = prev_u.copy() if prev_u is not None else cmd.copy()` | (6,). |
| F6 | [mpc.py:1487-1488](../controller/mpc.py#L1487-L1488) | `hold_extrap`: `cmd = np.clip(q_cur + q_dot*dt0, ...)`; `cmd = np.clip(cmd, prev_u-max_delta, prev_u+max_delta)` | Multiple (6,) temps. |
| F7 | [mpc.py:1492](../controller/mpc.py#L1492) | `return cmd, q_dot.copy(), diag` | Extra (6,) copy. |

**Per-fallback bytes:** ~1 KB on top of the per-tick baseline; since this
path fires precisely when a GC pause has already broken the budget,
minimising it is doubly-important even though it's not per-tick.

### Rough per-tick total

Summing R1 + R6 + M1–M19 + T1 + T2 + H1–H13 + C1–C8 + K1 + X1 (steady
state, success, `StaticTargetSource`, hardware plant):

- runner/telemetry:  ~2.3 KB
- mpc.solve:        ~13 KB (dominated by M11/M12/M16/M17)
- target source:    ~0.5 KB
- get_state:        ~1.6 KB
- command:          ~1.4 KB
- hooks:            ~0.3 KB
- **total:          ~19 KB per tick**

This is the number to drive below 1 KB for the contract.

### Priority list for W4 (highest-impact fixes first)

1. **M11/M12/M16 — CasADi DM → numpy conversions.**
   `np.asarray(sol['x']).ravel()` allocates a fresh n_w (=180) buffer per
   call, three times (x, lam_g, lam_x) plus g.  Biggest single contributor.
   Fix: pre-allocate `_w_opt_buf`, `_lam_g_buf`, `_lam_x_buf`, `_g_buf`
   in `__init__`, copy in-place via `np.copyto(buf, np.asarray(sol[k]))`
   or the CasADi `.get_ptr` ABI.
2. **M17 — `constraint_violation` temporary chain.**
   Four–six intermediate arrays just to compute one `float(np.max(...))`.
   Fix: rewrite as a scalar loop over precomputed slices, or preallocate
   a single `_viol_buf` and overwrite in place with the three `maximum`
   operations.
3. **R6 — StepRecord dataclass allocation.**
   Every tick builds a fresh 55-field dataclass.  Fix: `__slots__` on
   `StepRecord` (eliminates `__dict__`) + optionally pre-allocate a pool
   of records when `len(logger._records) < _TAIL_SIZE`, rotate rather
   than append.
4. **M2–M4 — ref_traj / twist_traj / accel_traj buffers.**
   Three 11×6 arrays rebuilt every tick.  Fix: pre-allocate once in the
   MPC and pass references in; overwrite in place.  This is particularly
   straightforward because `_build_reference_from_events` already knows
   the output shape at construction time.
5. **M6–M9 — event-list vectorisation.**
   `np.array([e.pose for e in events])` allocates a temporary list and a
   fresh (k,6).  For small event lists (2–5) this is a lot of churn.  Fix:
   if `ReferenceEvent` becomes `__slots__`-based or a NamedTuple, we can
   keep per-node pose/twist/accel in a pre-allocated `(K_MAX, 6)` buffer
   on the source side and skip the round-trip.
6. **H1 — ZMQ drain.**
   `recv_multipart` allocates a fresh `[bytes, bytes]` list per message.
   At 15 msgs/tick this is 15 lists + the msgpack dicts.  Fix: if
   `zmq.CONFLATE` can be made to work reliably (current comment says it
   can't on our libzmq), use it; otherwise a single-message
   `recv_into` path or reduce publish rate to the MPC's consumption rate.
7. **H13 — PlantState dataclass.**
   Same pattern as R6.  Fix: `__slots__` + pre-allocated ndarray fields
   that are overwritten in place via `np.copyto`.
8. **R1 / M19 / F1 — per-tick dicts.**
   `extras`, `diag`, `return dict` from solve().  Fix: replace with
   `__slots__` / NamedTuple / pre-allocated SimpleNamespace, mutated in
   place.
9. **T1 — dead `prev_target` copy in `StaticTargetSource.update()`.**
   Trivial — just delete it.  The variable is never read.
10. **T2 / Z4 — `TargetCommand` allocation per tick.**
    Dataclass; non-frozen already, so can be pre-allocated per source and
    mutated in place.  The issue is only that `dataclass` with default
    factories needs explicit reset.
11. **K1 — hooks on_pre_command temporaries.**
    Three (6,) finite differences.  Pre-allocate three (6,) buffers on
    the hook closure.
12. **C2/C4 — command-side accel/delta temporaries.**
    Several (6,) broadcasts per tick for the FF derivative.
    Pre-allocate; write in place with `np.subtract(a, b, out=buf)`.
13. **M10/M13/M14/M15 — per-tick (6,) copies in solve.**
    Each individually small but several per tick.  Most can be
    eliminated by tracking views rather than copies (e.g. `self._prev_u`
    can be a view into `self._prev_w[:6]` rather than an independent
    copy).

## Non-goals for this W1 audit

- **CasADi's internal allocations.**  `self._solver(**kw)` allocates C-level
  buffers (DM matrices, IPOPT working memory); Python `tracemalloc` will
  not see them, and they don't contribute to Python Gen 2 GC pressure.
  Out of scope.
- **FK inner loop in `leg_lengths_to_pose` (H7).**  Already optimised to
  `max_iter=10, tol=1e-4` (see
  [2026-04-18-mpc-overhead-spikes-fallback-bursts](2026-04-18-mpc-overhead-spikes-fallback-bursts.md)
  Fix C).  Further FK-internal pre-allocation is a separate investigation.
- **`plant_.set_pose` Newton-Euler math (K2).**  Same — the Newton-Euler
  implementation lives in the hardware_plant's torque-FF helper and
  warrants its own pass later.
- **Scheduler segment build / AOT solver setup / CSV logging.**  Explicitly
  out of scope per the /investigate brief — these are off the 40 Hz hot
  path or out-of-band.
- **Steady-state cmd jitter unrelated to single-sample overhead spikes.**
  Hold-phase HF spectral content is a separate investigation entirely.

## Fix — final landed scope

The W1 inventory was reviewed and approved on 2026-04-23 with three
agreed adjustments to the original plan:

1. **Threshold:** ship at 1024 B/tick (W4e), ratchet to 256 B/tick (W7)
   once the inventory was cleared.  Both thresholds are held in
   `controller/hot_loop_contract.py::THRESHOLD_BYTES` — a single source
   of truth shared with the contract doc and the enforcement test.
2. **Scope carve-outs:** Fix #6 (ZMQ `recv_into` with pre-allocated
   buffers) deferred to a follow-on ZMQ-hot-path investigation; Fix #11
   (hook `set_pose` Newton-Euler) kept as closure-level pre-allocation
   only (the set_pose internals are out-of-scope).  All other inventory
   items landed.
3. **`ipc.py` treatment:** message constructors (`make_mpc_command`)
   gained an optional `out=` kwarg so HardwarePlant.command can mutate a
   pre-allocated dict in place.  Other call sites (bridge nodes, tests)
   keep the legacy fresh-dict path.

### Commits

| W | Commit | Summary |
|---|--------|---------|
| W1 | ec08312 | Audit (this entry) + contract + enforcement + W4d pool |
| W2 | ec08312 | `controller/HOT_LOOP_CONTRACT.md` + `hot_loop_contract.py` |
| W3 | ec08312 | `tests/sim/test_hot_loop_allocation_contract.py` with xfail |
| W4a | 306d111 | `mpc.solve` buffer pre-alloc (solver returns, ref arrays, diag, constraint-violation scalar loop, `_shift_warm_start` reuse, two-tick u history) |
| W4b+c | fb7e20b | Target sources, HardwarePlant PlantState+cmd buffers+Packer reuse, `ipc.make_mpc_command(out=)` kwarg |
| W4d | ec08312 | `StepRecord` pool recycle + `next_record`/`last_record`/`fill_record_from_arrays` + runner extras SimpleNamespace + hook closure buffers |
| W4d hook fix | 149070d | `_on_pre_command` augmented-assign shadowed closure free var (caught live during W6 prep) |
| W4e | ec08312 | xfail lifted in the same W4d commit |
| W5 | f38abd3 | `gc.disable()` / scheduled `gc.collect(2)` wrapper + T-I4 in-tick GC test + pre-loop gc-state preservation |
| W7 | a52c158 | Threshold ratchet 1024 → 256 B/tick |

Audit measurement vs test measurement — a late correction:
The W1 audit estimated ~19 KB/tick of gross Python allocations.  The W3
enforcement test, which measures *net retained bytes between snapshots*
via `tracemalloc`, reported 2352 B/tick pre-W4 on that same code.  The
discrepancy is tracemalloc's blind spot for short-lived temporaries —
most of the 19 KB is refcount-freed before the snapshot fires.  Retained
bytes are what drive Gen 2 GC pressure in practice, so the tracemalloc
number is the right contract target.  Post-W4+W5+W7: 150 B/tick measured.

## Verification

### Unit + integration tests

`pytest tests/ -v` — **1035 passed, 0 failed** on the Jetson.
Baseline pre-cycle was 1033; +2 new tests (the W3 contract test +
the W5 T-I4 in-tick-GC assertion); the existing 1033 continue to pass
bit-exact, including the K1–K6 adversarial fixture which is our
correctness gate for the buffer-reuse refactor.

Contract test measurement ladder across the W-phases:

| Phase | Measured B/tick | Threshold |
|------:|----------------:|----------:|
| Pre-W4 (W3 initial) | 2352 | 1024 (xfail) |
| Post-W4d only      | 120 | 1024 (passing) |
| Post-W5            | 172 | 1024 (passing) |
| Post-W4a           | 150 | 1024 (passing) |
| Post-W4b+c         | 137 | 1024 (passing) |
| **Post-W7 (current)** | **150** | **256** |

(The W5 tick from 120 → 172 is the scheduled-collect trace entry
retained by tracemalloc — not a real regression.)

### Hardware validation (W6, 2026-04-23)

Session: `mpc_20260423_184647.csv` — 60 s hold at `(0, 0, 220, 0, 0, 0)`,
Jetson Orin Nano, bringup stack up.

Key measurements from `/diagnose --latest`:

| Metric | Value | Expectation |
|--------|-------|-------------|
| In-tick overhead p50 | 5.9 ms | < 15 ms |
| In-tick overhead p95 | 7.8 ms | < 15 ms |
| In-tick `gc_ms` non-zero samples | **0 / 2400** | 0 (W5 invariant) |
| `W5 VIOLATION` stdout count | **0** | 0 |
| Solve success rate | 100 % (2400/2400) | > 99 % |
| Consecutive solve failures | 0 | 0 |
| Final tracking error | 0.021 mm / 0.0019° | < 2 mm |
| Hold-phase asymmetry ratio | 1.26 | < 1.5 |
| Per-leg hold act stdev (agg) | 5.93 μm | < 20 μm |

Primary invariant (zero in-tick GC) holds.  Operator observation:
"super clean on the ODrive GUI".

#### Observation: scheduled Gen-2 collect cost on hardware

W5's scheduled `gc.collect(generation=2)` (between-tick sleep window)
was predicted to cost ~5 ms.  Measured on Jetson: **140–156 ms per
firing**.  CasADi's Python wrappers + numpy + msgpack accumulate more
Gen-2-eligible objects every 5 seconds than anticipated.

Crucially, the cost lands *outside the tick body*:
- Every tick's `gc_ms` column is 0 (W5 invariant: no in-tick GC).
- Every tick's `overhead_ms` column (in-tick) is clean (p95 = 7.8 ms).
- The 149 ms wall-clock gaps show up only in the *inter-tick* `dt`
  series, which the motor guard's Hermite interpolation absorbs.

Follow-up: cadence tuned from 200 ticks (5 s) to **1200 ticks (30 s)**
after W6 — reduces collects from ~4/min to ~2/min.  Rationale +
hardware-measured cost documented in `controller/HOT_LOOP_CONTRACT.md`.

#### Known-false-positive flags in the W6 diagnose output

- `Oscillation detected on all 6 legs` — `diagnose.py` computed chatter
  from `diff(cmd)` sign changes over the full session.  ODrive
  pos_setpoint scope was clean per the operator — this was a
  `diagnose.py` measurement artefact, not real oscillation.  **Resolved
  in commit 3082ff5** (see Pass 1 + Pass 2 close-out below), and the
  diagnosed root cause turned out to be different from this note's
  original hypothesis: not the scheduled-GC 149 ms gaps, but sub-LSB
  IPOPT float-rounding noise in the commanded extensions (see "Post-W7
  audit", follow-ups section, for the actual mechanism).

## Withdrawn claims

- [2026-04-23 W6] Claimed the W6 session's "Oscillation detected on all
  6 legs" flag was a false-positive caused by the 4 scheduled Gen-2 GC
  149 ms inter-tick sleep gaps producing large `diff(cmd)` sign changes
  that the chatter heuristic counted as oscillation.
  **WITHDRAWN**: the first attempted fix (mask samples where
  `dt > 5 × median`, added to `diagnose.py` in an interim version) only
  moved chatter ratios from 0.54 → 0.52 — needle barely moved.  Raw-data
  inspection showed that ~84 % of non-zero `Δcmd` values on the W6 hold
  are sub-micron IPOPT float-rounding noise, NOT scheduled-GC-gap
  deltas.  The chatter ratio of 0.54 is a real measurement of that
  sub-LSB noise being counted as sign changes — the dt mask was the
  wrong mitigation because it wasn't the dominant mechanism.
  **Superseded by**: sub-LSB magnitude floor in
  `analyse_oscillation` (``_CHATTER_MIN_DELTA_MM = 10 µm``), which
  filters floats below one encoder LSB before the chatter-ratio
  computation.  See commit 3082ff5 message and the "Post-W7 audit"
  section's chatter-follow-up entry.  Resulting chatter ratio on W6
  session: below detection threshold on all legs.

## Outcome

`GC_PAUSE_CANDIDATE` on the MPC 40 Hz hot loop flipped from `active` →
`fixed`.  `controller/HOT_LOOP_CONTRACT.md` is the normative spec
defending the class of failures going forward:

- Every new `TargetSource`, `PlantInterface`, or `MpcLoopHook` that
  lands on the hot loop is required to be pre-allocation-compliant.
- CI enforcement via `tests/sim/test_hot_loop_allocation_contract.py`
  emits a top-10 allocation-site diagnostic on failure so a regression
  is actionable without rerunning locally.
- Threshold is 256 B/tick with a documented ratchet procedure for any
  future relaxation.

### Side-effect gains

- Sim MPC mean overhead dropped from 14.8 ms → 5.9 ms on the W3 fixture
  as a pure consequence of removing per-tick dict/dataclass churn.
  Hardware measured at 5.9 ms median / 7.8 ms p95 in W6 (vs 10–15 ms
  typical on pre-contract sessions).  Room regained for new hot-loop
  work (e.g. an additional diagnostic or hook) without risking budget.
- The class of failure extends beyond this specific tick: any future
  hot-loop addition is now structurally prevented from reintroducing
  GC pressure.  This is the same compounding value the K1–K6
  reference-feasibility contract provides.

### Open follow-ups

- **Motion-onset dead-time** (197 ms worst leg, stick-slip signature).
  Pre-existing, unchanged by this work.  Tracked in
  [2026-04-20-motion-onset-dead-time-fix](2026-04-20-motion-onset-dead-time-fix.md).
- **`diagnose.py` chatter false-positive on large inter-tick gaps.**
  Minor measurement-tool bug surfaced by W6 — chatter heuristic should
  skip samples where `dt > 5 × expected` (the scheduled-GC gap).
  Cosmetic, non-blocking.
- ~~**`_on_pre_command` unit test gap.**~~  **Resolved by Pass 2 audit
  (commit a60cf49).**  The hardware-path contract test wires the
  production `_on_pre_command` hook, so augmented-assign-on-closure
  bugs in hook code now fail CI at commit time.
- ~~**`diagnose.py` chatter false-positive on large inter-tick gaps.**~~
  **Resolved in commit 3082ff5 — but not for the reason originally
  hypothesised.**  A dt-outlier mask for scheduled-GC gaps (the
  intended fix) left the per-leg chatter ratios almost unchanged
  (0.54 → 0.52).  Inspection of the raw W6 data showed the real cause:
  84% of non-zero `cmd_ext` deltas during a steady-state hold are
  **sub-micron float-rounding noise from IPOPT** (median |Δ| ≈ 1 µm,
  well below one encoder LSB at ~15 µm).  `np.sign` promotes any
  non-zero delta to ±1 regardless of magnitude, and ~half of those
  flip — producing a spurious chatter ratio of ~0.5 across all legs
  independent of any real oscillation.  The motor guard's dead-band
  filters these before CAN transmit, so they are not physically
  observable; applied the same physical floor (10 µm) to the
  diagnostic.  The 5×-median dt mask was kept for the
  `amplitude_growing` envelope regression (GC-gap deltas can still
  skew the slope fit even though they don't dominate sign-change
  counting).  Lesson: when a follow-up fix doesn't move the needle,
  stop and diagnose; don't just claim victory on the hypothesis
  written down months earlier.

## Post-W7 audit (Pass 1 + Pass 2, 2026-04-23)

After W6 close-out the branch was handed to an audit pass that surfaced
8 residual issues across both the contract's *scope* and its
*enforcement*.  Four commits landed in two risk-gated passes.

### Context: why an audit was warranted despite the green W6 run

W6 validated the primary safety property (zero in-tick GC events on
hardware) and the primary performance property (overhead p95 < 15 ms).
Those are necessary but not sufficient for "the contract holds" — the
contract's normative claim is **no more than THRESHOLD_BYTES of Python
allocation per steady-state tick**, measured uniformly across every
plant implementation and every hot-loop surface.  Two gaps in how the
W3 enforcement test measures vs. what the contract *asserts* were
hiding real regressions behind the green bar.  The Pass 1 + Pass 2
work closes both.

### Pass 1 — mechanical cleanup (commits e1971eb, 27755eb)

Six issues, all LOW-risk, landed in two small commits:

1. **[BLOCKING] `TelemetryLogger.load()` → AttributeError on path=None.**
   The W1–W4d refactor renamed `self._records` to `self._pool` but
   missed the fallback branch, silently breaking
   `sim/analysis/record_baselines.py`.  One-line fix; the `records`
   property had been in place since the refactor, so the correction is
   `list(self._records)` → `list(self.records)`.
2. **[WARNING] `HOT_LOOP_CONTRACT.md` hook template re-introduces the
   W4d bug.**  The hook template at `HOT_LOOP_CONTRACT.md:449`
   contained `_twist_buf /= dt0` — exactly the augmented-assign-on-
   closure-free-variable pattern fixed in 149070d.  Future contributors
   copy-pasting the template would recreate the `UnboundLocalError`.
   Replaced with `np.divide(..., out=_twist_buf)` and added a **Pattern
   gotcha: augmented-assign in closures** subsection explaining why
   STORE_FAST on a free variable raises.  The fix commit 149070d
   correctly patched production code but didn't ripple the lesson to
   the normative doc — this is the missing leg of the contract
   (invariant, enforcement, *documentation*).
3. **[NOTE] Stale GC-cadence comment** in the T-I4 test (200 vs. the
   current 1200-tick cadence after 8cab8fe).  Comment now references
   `_GC_COLLECT_EVERY_N_TICKS` by name so a grep surfaces it on any
   future cadence change.
4. **[NOTE] Pool size doc mismatch:** `HOT_LOOP_CONTRACT.md` said 5200,
   code says `_DEFAULT_POOL_SIZE = 500`.  Doc corrected; now references
   the constant name for grep-traceability.
5. **[NOTE] GC-disable race at loop entry.**  `_was_gc_enabled =
   gc.isenabled(); gc.disable()` sat *outside* the `try:` block.  A
   `KeyboardInterrupt` in that microsecond window would leave the
   interpreter with GC globally disabled.  Moved both statements
   inside `try:`, default `_was_gc_enabled = True` for the safe
   fallback.  Narrow window; correctness-only.
6. **[NOTE] Dead `_dash_sink`** in `run_mpc_loop` — written every tick,
   never read.  Legacy of the pre-inline `log_mpc_step` path.  Removed.

### Pass 2 — contract-tightening (commits 7c44eb8, a60cf49)

Two structurally larger issues.  Each exposes a way the W3 enforcement
test was blind to violations of the contract's *intent*, and each
required new infrastructure rather than just a patch.

#### [WARNING] `last_{ref,twist,accel}_traj` return `.copy()` — ~1 KB/tick invisible to tracemalloc

**Mechanism of the blind spot.**  `MPCController.last_ref_traj`
returns `self._last_ref_traj.copy()`.  `runner.mpc_solve`
([runner.py:156-157](../controller/runner.py#L156-L157)) and
`sim/main.py:816-817` call it every tick — two fresh (N+1, 6) ndarrays
per call (~1056 B/tick total).  These copies are consumed synchronously
and freed by refcount before the next tick.  The W3 contract test uses
`tracemalloc.compare_to(...)` on snapshots 100 ticks apart, which
reports *net retained bytes*.  Short-lived allocations don't show up in
the delta — they were already freed by the time S2 fires.  This is the
exact Gen-0/1 churn the contract exists to eliminate, but the test
can't see it.

**Why this matters even with `gc.disable()`.**  W5's `gc.disable()`
ensures no in-tick GC pause regardless of churn rate.  But the
contract's *normative claim* is stronger: no allocation.  Gen-0/1 churn
remains a latency cliff if GC is ever re-enabled, and it makes the
steady-state memory behaviour harder to reason about.

**Fix (7c44eb8).**  Added `last_ref_traj_view` /
`last_twist_traj_view` / `last_accel_traj_view` properties returning
the internal buffer directly (aliased to `_ref_traj_buf` which lives
for the MPC's lifetime).  Mirrors the existing
`predicted_poses_view` / `predicted_times_view` convention — the
`_view` suffix is the established "non-copying read-only" idiom.
Switched the two hot-path call sites.  The copy-returning
properties remain for snapshot consumers
(`tests/sim/test_cold_start_fixes.py:211,215` explicitly `.copy()` the
result, i.e. the snapshot API is load-bearing for them).

#### [WARNING] HardwarePlant hot path unmeasured — 5 allocation sites, ~480 B/tick on real hardware

**Mechanism of the blind spot.**  The W3 fixture
(`_build_fixture()` in the contract test) builds against `MuJoCoPlant`
only.  `HardwarePlant` has its own `command()` / `get_state()` /
`set_pose()` bodies that never ran under `tracemalloc`.  Five `.copy()`
/ fresh-ndarray sites remained in the hardware path:

- `_prev_cmd_ext_mm = ext_mm.copy()` (acceleration history shift)
- `_last_sent_ext_mm = ext_mm.copy()` (dead-band snapshot)
- `_last_state_ext_mm = ext_mm.copy()` (velocity-FF backward-diff)
- `_last_pose_6dof = np.asarray(...).copy()` (set_pose workspace arg)
- `_ff_torque_Nm = torque_ff_Nm` (fresh ndarray returned by
  `cartesian_to_motor_commands`, retained as an attribute)

~480 B/tick on real hardware — already over the 256 B/tick threshold
the contract claims to enforce.  Both W5 (gc.disable) and W6
(hardware validation) held because neither *measures* the allocation
budget on the hardware surface.

**Fix (a60cf49), part (a): pre-allocation with flag-gated sentinels.**
Every `None | ndarray` sentinel became a pre-allocated buffer + a
`_has_*` boolean flag.  The old `if foo is not None:` check is
replaced by `if self._has_foo:`; the value is written via
`np.copyto(self._foo_buf, src)`.  `disable()` / `estop()` reset via
the flag instead of rebinding to `None` (which would defeat the
pre-allocation).

The tricky case was the two-tick command history
(`_prev_cmd_ext_mm` / `_prev_prev_cmd_ext_mm`) — a ring-of-two buffer
pair.  The acceleration three-point-difference reads both attributes
BEFORE the per-tick shift, so the shift can swap which buffer each
label points at and then `np.copyto` the new tick's data into the
(now-freed) old-prev-prev buffer.  Walked the math through one tick
to confirm no read sees a mutated buffer prematurely.

**Fix (a60cf49), part (b): hardware-path contract test.**  Land (a)
without (b) and the hardware surface stays invisible forever — any
future regression reintroduces the same blind spot.  Added
`test_hot_loop_allocation_contract_hardware` sibling test:

- Patches `controller.hardware_plant.zmq` at the import site, uses
  MagicMock for construction-time socket ops, then replaces the
  hot-path methods (`send_multipart` / `recv_multipart`) with plain
  callables so MagicMock's `call_args_list` doesn't grow per tick.
- Pre-packs one msgpack telemetry frame at fixture setup; a
  `_FramePump` callable alternates frame / `zmq.Again` per call.
- Pre-computes target-pose motor positions via `MuJoCoPlant`'s IK
  so the simulated hardware reports a steady-state plant at its goal.
  FK, Jacobian, rotation-matrix conversion all run real.
- Replicates `run_mpc.py`'s production `_on_pre_command` hook verbatim
  so `plant.set_pose()` runs every tick — exercising the torque-FF
  path that houses hardware_plant.py:725 and :746.

Exercises exactly the invariant the W3 test was missing.

### Structural lessons

Three patterns showed up across these 8 findings that are worth
defending against in future contract work:

1. **A test that measures X does not enforce a claim about Y.**
   tracemalloc measures retained bytes; the contract asserts allocation
   bytes.  These are related but not identical.  The Pass 1 Fix 2 and
   Pass 2 Fix 1 findings were both invisible to the W3 test because
   short-lived allocations (freed within the snapshot window) don't
   count.  The hardware-path test is similar: the MuJoCo fixture can
   hold the contract green without exercising any hardware-side code.
   **Contract = invariant + enforcement that *actually covers the
   invariant*.**  When the two drift, write down the gap.

2. **Normative docs decay silently.**  `HOT_LOOP_CONTRACT.md`'s hook
   template and the pool-size figure were both written against a
   point-in-time code state, then the code moved and the doc didn't.
   The augmented-assign-in-closure gotcha is the most expensive
   example — a contributor copy-pasting the template would have
   recreated the exact bug the contract is meant to prevent.  The fix
   in the doc needs to ripple whenever the production code moves;
   referencing constants by name (`_GC_COLLECT_EVERY_N_TICKS`,
   `_DEFAULT_POOL_SIZE`) instead of hard-coded numbers gives future
   readers a grep-path to surface the drift.

3. **"Just fix it forward" misses the class of failures.**  Fix 1
   (Pass 1) was a 1-line fix.  Fix 3 (Pass 2) was 300 lines — 5
   pre-allocated buffers, 5 flag-gated sentinels, ring-of-two buffer
   swap, a new test fixture with patched ZMQ and a frame pump.  The
   temptation to ship Pass 1 and leave Pass 2 "for later" was real;
   running the audit as two risk-gated passes but insisting both land
   is the same engineering pattern as the K1–K6 contract — enumerate
   the full class of failures, land a single normative fix for all
   of them, don't accept "just this one case" carve-outs.

### Verification (Pass 1 + Pass 2)

| Layer | Result |
|-------|--------|
| Unit + integration tests | 1036 passed (baseline 1035, +1 for the new hardware contract test). |
| MuJoCo contract test (`test_hot_loop_allocation_contract`) | PASS after Pass 2 Fix 1 — margin widens as per-tick Gen-0 churn drops. |
| Hardware contract test (`test_hot_loop_allocation_contract_hardware`) | PASS after Pass 2 Fix 2 — below 256 B/tick threshold with the production `_on_pre_command` hook wired. |
| Hardware re-validation | *Not re-run* — all changes are allocation-pattern refactors with no safety-critical pipeline changes.  Invariant measured by the extended contract tests. |
