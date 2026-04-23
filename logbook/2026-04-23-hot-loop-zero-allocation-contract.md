---
title: Hot-loop zero-allocation contract (W1 inventory → contract → enforcement → fixes)
type: investigation
date: 2026-04-23
status: in-progress
phase: "hardware-bringup — GC-pause elimination on the MPC 40 Hz hot loop"
related_plan: "hardware-bringup.md"
related_issues:
  - GC_PAUSE_CANDIDATE
related_entries:
  - 2026-04-18-mpc-overhead-spikes-fallback-bursts
  - 2026-04-20-k1-k6-reference-feasibility-resolution
sessions: []
files_changed: []
commits: []
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

## Next steps (gate)

This is the W1 deliverable.  No code has been changed yet.  Awaiting
operator approval on the inventory before W2 (contract document).
Specific items to review:

1. **Threshold choice.**  The W3 test proposes `< 1024 bytes/tick` as the
   contract.  Current measured per-tick allocation is ~19 KB, so the
   threshold represents a >95 % reduction.  Is 1 KB the right target,
   or should it be tighter (~256 B, essentially "nothing should
   allocate") once the heavy hitters are fixed?
2. **Scope — is anything in the priority list too aggressive?**  Fix 1
   (CasADi DM → numpy) in particular touches the solver return path and
   needs care around numeric correctness.  Fix 3 (StepRecord `__slots__`)
   may interact with the CSV flush path (`asdict()` in `_flush_batch`).
3. **Non-goals — anything missing?**  In particular, the
   `ros_ws/src/jugglebot/jugglebot/motion/ipc.py` message constructors
   (`make_mpc_command`, `make_motor_feedback`) were listed as audit
   targets but end up in "per-tick, inside C6 of command()" — the dicts
   they build are the hot thing, not the modules themselves.  Fine, or
   do we want to separate-call them out?
