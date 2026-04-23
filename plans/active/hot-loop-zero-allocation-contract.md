---
title: Hot-Loop Zero-Allocation Contract (MPC 40 Hz)
created: 2026-04-23
status: active
---

# Hot-Loop Zero-Allocation Contract (MPC 40 Hz)

## Context

### Why this contract exists

Every hardware MPC session's diagnosis JSON flags 1–3 `GC_PAUSE_CANDIDATE`
spikes: 40–67 ms single-sample overhead blips on the 40 Hz hot loop.
Each spike forces the loop into walk-forward fallback for that tick and
leaves a visible cmd discontinuity in telemetry.  W7 of the K1–K6
contract (see
[logbook/2026-04-20-k1-k6-reference-feasibility-resolution.md](../../logbook/2026-04-20-k1-k6-reference-feasibility-resolution.md))
made the fallback *safe*, but did not *eliminate* the trigger.

The trigger is Python's generational GC.  On the Jetson Orin Nano a Gen 2
sweep of CasADi solver state + per-tick numpy arrays takes 40–80 ms —
well over the 25 ms tick period.  Gen 2 fires when per-generation
allocation counts cross thresholds, so eliminating the pauses means
eliminating the hot-loop allocations that drive Gen 2 pressure.

W1 audit (see
[logbook/2026-04-23-hot-loop-zero-allocation-contract.md](../../logbook/2026-04-23-hot-loop-zero-allocation-contract.md))
measured **~19 KB of Python allocations per steady-state tick**.  The top
contributors are:

- `np.asarray(sol['x']/lam_g/lam_x/g).ravel()` in `mpc.solve` (~6 KB)
- `constraint_violation` `np.maximum` temporary chain (~3 KB)
- `StepRecord` 55-field dataclass with `__dict__` every tick (~1.5 KB)
- `ref_traj` / `twist_traj` / `accel_traj` rebuild (~1.6 KB)
- ZMQ drain lists + msgpack dicts, `PlantState` dataclass, `extras` dict,
  `diag` dict, `TargetCommand` dataclass, assorted (6,) `.copy()` calls

### What this achieves

1. **Structural elimination of GC-pause-induced cmd discontinuities.**
   Once the hot loop allocates <1 KB/tick, Gen 2 collections fire at a
   dramatically lower rate (empirically: never, once combined with
   `gc.disable()` on the hot loop).
2. **Enforcement via CI, not discipline.**  A tracemalloc-based pytest
   fails on any future addition that re-introduces per-tick allocations.
   This is a *contract*, the same pattern as the K1–K6 reference-layer
   contract.
3. **Latency floor reduction.**  Even without GC, pre-allocated buffers
   run 5–15 % faster than freshly-allocated ones on the Jetson due to
   cache locality and malloc-free overhead.
4. **Clearer hot-path boundaries.**  The contract doc forces every new
   `TargetSource` / hook / plant implementation to think explicitly about
   per-tick behavior.

### When to do this

**After** the K1–K6 hardware validation (2026-04-20) and before the next
batch of hardware-bringup dynamic-motion work (catch coordination, toss
loop re-integration).  The contract's enforcement test prevents the class
of regressions that would otherwise compound as new sources and hooks are
added.

**Prerequisites:** none — the W1 audit has already been completed.

### Related work

- [logbook/2026-04-18-mpc-overhead-spikes-fallback-bursts.md](../../logbook/2026-04-18-mpc-overhead-spikes-fallback-bursts.md) —
  original GC-pause investigation and Fix A instrumentation (`_GCTracker`).
- [logbook/2026-04-20-k1-k6-reference-feasibility-resolution.md](../../logbook/2026-04-20-k1-k6-reference-feasibility-resolution.md) —
  K1–K6 contract; "Open follow-ups" section names GC pauses explicitly.
- [logbook/2026-04-23-hot-loop-zero-allocation-contract.md](../../logbook/2026-04-23-hot-loop-zero-allocation-contract.md) —
  W1 audit output; the per-site inventory referenced throughout this plan.
- [controller/REFERENCE_LAYER_CONTRACT.md](../../controller/REFERENCE_LAYER_CONTRACT.md) —
  structural template for the new HOT_LOOP_CONTRACT.md.

---

## Architecture

### Current

```
run_mpc_loop (40 Hz, ~19 KB alloc/tick)
 │
 ├── plant.get_state() ────── ~1.6 KB (msgpack dicts, np.array from list,
 │                                     J.copy(), PlantState dataclass)
 │
 ├── source.update() ──────── ~0.5–1 KB (TargetCommand dataclass,
 │                                       target_pose.tobytes() + tuple)
 │
 ├── hooks.on_target_override  0 B (steady state)
 │
 ├── mpc.solve() ─────────── ~13 KB  (np.asarray(sol[k]).ravel() ×4,
 │                                    np.maximum temp chain, ref/twist/
 │                                    accel_traj rebuild, (6,) copies,
 │                                    diag dict)
 │
 ├── hooks.on_post_solve        0 B (non-catch)
 │
 ├── hooks.on_pre_command ──── ~300 B ((6,) finite-diff temps)
 │
 ├── plant.command() ─────── ~1.4 KB (msg dict, msgpack bytes, (6,) copies)
 │
 ├── plant.step() ──────────── 0 B (no-op on hardware)
 │
 ├── hooks.on_post_step        0 B (non-sim)
 │
 ├── extras dict build ────── ~400 B (runner.py:420-429)
 │
 └── log_mpc_step() ─────── ~2.3 KB  (StepRecord dataclass,
                                      np.linalg.norm temps)

GC behaviour: generational GC enabled by default.
Gen 0 fires ~every 700 allocations (sub-ms).
Gen 2 fires on allocation-count threshold crossings,
  irregularly, 40–80 ms per sweep on the Orin Nano.
```

### Proposed

```
run_mpc_loop (40 Hz, <1 KB alloc/tick after W4; <256 B after W7)
 │    # gc.disable() active throughout the body
 │    # periodic gc.collect() every 200 ticks on the idle sleep
 │
 ├── plant.get_state() ──── pre-allocated PlantState (shared, mutated in
 │                          place).  ZMQ drain into pre-sized buffers.
 │                          J_norm buffer reused.
 │
 ├── source.update() ────── pre-allocated TargetCommand per source,
 │                          mutated in place.  request_key tuple reuse
 │                          (pre-bytes of target_pose cached by dirty bit).
 │
 ├── mpc.solve() ────────── pre-allocated _w_opt_buf, _lam_g_buf,
 │                          _lam_x_buf, _g_buf (np.copyto from CasADi).
 │                          Pre-allocated ref_traj/twist_traj/accel_traj.
 │                          _viol_buf single scalar-reduction pass.
 │                          diag as pre-allocated SimpleNamespace.
 │
 ├── hooks.on_pre_command ─ closure-level (6,) twist/accel buffers.
 │
 ├── plant.command() ───── msg dict pre-allocated per plant, mutated;
 │                          msgpack.packb() into a pre-allocated bytes
 │                          buffer (msgpack supports this via Packer).
 │
 ├── extras ──────────────── SimpleNamespace pre-allocated on the runner,
 │                          fields written in place.
 │
 └── log_mpc_step() ────── StepRecord with __slots__; pool of 200 records
                            rotated rather than appended-and-popped.
                            CSV flush unchanged (asdict still works on
                            slotted dataclasses).

Enforcement: tests/sim/test_hot_loop_allocation_contract.py.
  - Runs 200 ticks against MuJoCoPlant + StaticTargetSource.
  - Skips first 50 ticks (warmup).
  - tracemalloc snapshot at tick 50 and tick 150.
  - Asserts (s150.total - s50.total) / 100 < threshold.
  - Threshold: 1024 B/tick after W4 (initial ship).
  - Threshold: 256 B/tick after W7 (ratchet follow-up).
  - On failure: emits top-10 allocation sites with bytes and tracebacks.
```

### What changes vs what stays the same

**Changes:**
- `controller/mpc.py`: pre-allocated solver-return buffers; pre-allocated
  reference arrays (M2–M4); pre-allocated `SimpleNamespace` diag; elimination
  of M13/M14/M15 defensive copies via view-tracking; in-place `constraint_violation`
  scalar reduction.
- `controller/runner.py`: `extras` becomes a `SimpleNamespace`; `gc.disable()`
  wrapper; periodic `gc.collect()` scheduled between ticks.
- `controller/target.py`: `ReferenceEvent` becomes `__slots__`-based;
  `TargetCommand` becomes `__slots__`-based.  `StaticTargetSource` deletes
  the dead `prev_target.copy()` and pre-allocates its `TargetCommand`.
- `controller/zmq_target.py`: pre-allocated `TargetCommand`; cached
  `target_pose.tobytes()` invalidated on the dirty bit.
- `controller/hardware_plant.py`: pre-allocated `PlantState` mutated in
  place; pre-allocated (6,) command-side buffers; pre-allocated
  MPC-command dict; `msgpack.Packer` with `use_bin_type=True` reused.
- `controller/telemetry.py`: `StepRecord` gains `__slots__`; record pool
  instead of `list.append`.
- `ros_ws/src/jugglebot/jugglebot/motion/ipc.py`: `make_mpc_command` gains
  an `out` kwarg path for in-place dict mutation (called only from
  `HardwarePlant.command`).  Other call sites unchanged.
- `run_mpc.py`: hook closures gain pre-allocated buffers for on_pre_command
  finite differences and on_log_extras.
- **New** `controller/HOT_LOOP_CONTRACT.md` — normative spec.
- **New** `tests/sim/test_hot_loop_allocation_contract.py` — enforcement.

**Stays the same:**
- Public APIs: `TargetCommand` field set, `TargetSource.update()` signature,
  `PlantInterface.get_state()` return type, `mpc.solve()` return tuple.
- On-the-wire: every ZMQ message format, `StepRecord` CSV schema,
  protocol frames.  Mutating these dataclasses in place is invisible to
  consumers because they serialise the field values, not the object
  identity.
- K1–K6 semantics.  Pre-allocation does not touch reference-layer
  feasibility — `make_feasible_events` still constructs new event objects
  on target change (that's the per-target-change path, not per-tick).
- Solver numerics.  Fix 1 (CasADi DM → pre-allocated numpy) copies bytes
  out of CasADi DM into the same memory layout — bit-exact with the
  current `np.asarray(...).ravel()` path.

---

## Implementation Phase Summary

| Phase | Scope | Status | Date | Risk | Validates |
|-------|-------|--------|------|------|-----------|
| W2 | `controller/HOT_LOOP_CONTRACT.md` — normative spec | NOT STARTED | | Low | Contract language + operator approval before enforcement lands |
| W3 | `tests/sim/test_hot_loop_allocation_contract.py` — tracemalloc enforcement (threshold=1024 B/tick, initially `xfail` while W4 is in flight) | NOT STARTED | | Low | Harness works; top-10 allocation diagnostic emits correct sites |
| W4a | `mpc.solve` pre-allocation (sol buffers M11/M12/M16; violation chain M17; ref/twist/accel_traj M2–M4; diag dict M19) | NOT STARTED | | **Med-High** | Bit-exact solve output on every pytest scenario + MuJoCo adversarial fixture |
| W4b | `controller/target.py` + `zmq_target.py` + `StaticTargetSource` — `TargetCommand` pre-alloc, dead-copy removal, request_key cache | NOT STARTED | | Low | Existing target tests (K1–K6 property/scenario, zmq_target unit) stay green |
| W4c | `HardwarePlant` — `PlantState` mutation, (6,) command-side buffers, MPC-command dict pre-alloc, `msgpack.Packer` reuse | NOT STARTED | | Med | Hardware integration test against live motor guard |
| W4d | `StepRecord` `__slots__` + record pool; runner `extras` SimpleNamespace; hook closure buffers (on_pre_command, on_log_extras) | NOT STARTED | | Low | CSV load-back round-trip test; 200-tick `StaticTargetSource` sim run |
| W4e | Flip W3 test from `xfail` to active; verify on CI at 1024 B/tick threshold | NOT STARTED | | Low | Threshold gate |
| W5 | `gc.disable()` / periodic `gc.collect()` wrapper in `run_mpc_loop`; `_GCTracker` asserts zero in-tick GC events | NOT STARTED | | Low | No GC events during tick window in sim or hardware |
| W6 | Hardware validation — 60 s run of `--pose 0,0,220,0,0,0`; target zero isolated spikes in `/diagnose` output | NOT STARTED | | **Stop gate** | End-to-end: `overhead.isolated_spikes == 0` |
| W7 | Ratchet threshold from 1024 B/tick → 256 B/tick; second tracemalloc pass catches any remaining per-tick allocations | NOT STARTED | | Low | Tighter invariant permanent; future contributors held to the real floor |

Each W is a single commit with a `Logbook-Entry:` trailer.  W4 is split
into five sub-phases because the total diff is too large to review in one
shot and each sub-phase is independently testable.

---

## Implementation Phases (detailed)

### Phase W2: Contract document — NOT STARTED

**New files:**
- `controller/HOT_LOOP_CONTRACT.md`

**Scope:**
- Define the 40 Hz hot loop explicitly (the body of `run_mpc_loop` from
  `state = plant.get_state()` through `log_mpc_step(...)`, inclusive).
- State the invariant: the loop body MUST allocate < `THRESHOLD_BYTES`
  of Python objects per steady-state tick, measured by `tracemalloc`
  over a 100-tick window after warmup.  `THRESHOLD_BYTES = 1024` on
  initial ship (after W4e), ratcheting to `256` in W7.
- Define what counts: every Python object whose constructor runs inside
  the loop body.  Excluded: CasADi-internal C buffers (invisible to
  tracemalloc anyway), msgpack-internal C buffers from `Packer.pack`,
  OS file-descriptor operations.
- Implementation guidance for new contributors:
  - "Don't write a dict literal in the hot path.  Use
    `SimpleNamespace` or a `__slots__` dataclass pre-allocated in
    `__init__`, mutate fields in place."
  - "Don't write `arr.copy()` in the hot path.  Either the callee
    mutates it and you need a *view* (`arr[:]` or `arr.view()`), or
    you pre-allocate a twin buffer and `np.copyto(dst, src)` into it."
  - "Don't build `np.array([...])` from a list comprehension.  Preallocate
    and index-assign."
  - "Don't slice-then-copy (`w[:6].copy()`).  Track views + a
    separate-buffer copy made once per transition, not every tick."
- Reference to the W1 audit as the exhaustive inventory.
- Reference to the W3 test as the enforcement mechanism.
- Note that `ipc.py`'s message constructors are hot-loop-constrained only
  *when called from `HardwarePlant.command`* (the `out=` path); other call
  sites (bridge nodes, tests) are unconstrained.
- `@hot_loop_body` marker convention: functions that run on the hot path
  get a short docstring note so greppable.

**Critical details:**
- Contract doc follows the same section structure as
  [REFERENCE_LAYER_CONTRACT.md](../../controller/REFERENCE_LAYER_CONTRACT.md):
  Background, The Invariant, Enforcement, Implementing a new X (here: a
  new TargetSource / PlantInterface / MpcLoopHook), Diagnosis, Related.
- Language must be normative.  MUST / MUST NOT where appropriate (RFC 2119
  style).
- The doc names the test file explicitly so grep navigation works:
  "Enforcement is in `tests/sim/test_hot_loop_allocation_contract.py`."

**Dependencies:**
- W1 audit logbook entry must exist.  (Done.)

---

### Phase W3: Enforcement test — NOT STARTED

**New files:**
- `tests/sim/test_hot_loop_allocation_contract.py`

**Scope:**
- Fixture: construct `MPCController` + `MuJoCoPlant` + `StaticTargetSource`
  exactly as `sim/main.py --mpc` does.
- Schedule: single static target at `(0,0,170,0,0,0)`.
- Run loop for 200 ticks at simulated 40 Hz (wall-clock-paced off).
- Warmup: skip first 50 ticks.  Measures Python JIT warmup, MPC prime
  solve, first-target quintic build, CSV header write.  tracemalloc
  snapshot `S1` at end of tick 50.
- Measure: run ticks 51–150.  tracemalloc snapshot `S2` at end of tick 150.
- Assert: `(S2.total_traced_bytes - S1.total_traced_bytes) / 100 <
  THRESHOLD_BYTES`.
- On failure: compute `S2.compare_to(S1, 'lineno')`, print the top-10
  growth entries with `size_diff`, `count_diff`, traceback (one frame).
  This is the key debugging affordance — future contributors see exactly
  which `file.py:lineno` is over-allocating.
- Mark: `@pytest.mark.xfail(reason="W4 in progress")` until W4e lifts the
  marker.  Without the xfail, W4a–d cannot land because the test would
  fail at each intermediate commit.

**Test pseudocode:**
```python
@pytest.mark.xfail(reason="W4 pre-allocation in progress; lifted at W4e")
def test_hot_loop_allocation_contract():
    plant, mpc, source, logger, hooks = _build_sim_fixture()
    # warmup
    for _ in range(50):
        _run_one_tick(plant, mpc, source, logger, hooks)
    tracemalloc.start(25)  # 25-frame traceback for useful diagnostic
    s1 = tracemalloc.take_snapshot()
    for _ in range(100):
        _run_one_tick(plant, mpc, source, logger, hooks)
    s2 = tracemalloc.take_snapshot()
    tracemalloc.stop()
    bytes_per_tick = (s2.statistics('lineno')[0].size
                      - s1.statistics('lineno')[0].size) / 100
    if bytes_per_tick >= THRESHOLD_BYTES:
        _emit_top10_diagnostic(s1, s2)
        pytest.fail(
            f"Hot loop allocated {bytes_per_tick:.0f} B/tick "
            f">= {THRESHOLD_BYTES} B/tick threshold"
        )
```

**Critical details:**
- `tracemalloc.start(25)` (25 frames) captures enough traceback to
  attribute allocations to our code even through helper functions.
  `tracemalloc.start()` alone gives 1 frame and is useless for
  diagnosis.
- The threshold constant `THRESHOLD_BYTES` is imported from the contract
  module (`from controller.hot_loop_contract import THRESHOLD_BYTES`) so
  W7's ratchet is a one-line change.
- The test runs `MuJoCoPlant`, NOT `HardwarePlant`, because the hardware
  plant requires ZMQ infrastructure and a running motor guard.  Sim
  plant tests the `controller/` + `target/` + `runner/` allocation
  sources.  Hardware-specific allocations (get_state, command) are
  tested separately in T-H1.
- The W3 test explicitly does NOT run with `gc.disable()`.  W3 verifies
  the *allocation* invariant (the thing GC reacts to); W5 is what
  disables GC.  Separating these keeps the failure modes distinct.

**Dependencies:**
- W2 contract doc exists and defines `THRESHOLD_BYTES`.

---

### Phase W4a: `mpc.solve` pre-allocation — NOT STARTED

**Modified files:**
- `controller/mpc.py`

**Scope:**
- Pre-allocate in `__init__`:
  - `self._w_opt_buf = np.empty(self._n_w)`
  - `self._lam_g_buf = np.empty(self._n_g)`
  - `self._lam_x_buf = np.empty(self._n_w)`
  - `self._g_buf = np.empty(self._n_g)`
  - `self._ref_traj = np.empty((N+1, 6))`
  - `self._twist_traj = np.empty((N+1, 6))`
  - `self._accel_traj = np.empty((N+1, 6))`
  - `self._accel_filled = np.zeros(N+1, dtype=bool)`
  - `self._viol_buf = np.empty(self._n_w + self._n_g)`
  - `self._ev_times_buf = np.empty(K_EVENTS_MAX)` (K_EVENTS_MAX=16)
  - `self._ev_poses_buf = np.empty((K_EVENTS_MAX, 6))`
  - `self._ev_twists_buf = np.empty((K_EVENTS_MAX, 6))`
  - `self._ev_accels_present = np.zeros(K_EVENTS_MAX, dtype=bool)`
  - `self._ev_accels_buf = np.zeros((K_EVENTS_MAX, 6))`
- Pre-allocate a `types.SimpleNamespace` (or `__slots__` class)
  `self._diag` with fields:
  `solve_time_ms, status, iter_count, cost, constraint_violation,
   cmd_next_mm, cmd_next2_mm, fallback_step`.  Mutate in place.
- Replace `np.asarray(sol['x']).ravel()` with
  `np.copyto(self._w_opt_buf, np.asarray(sol['x']).ravel())` and return
  `self._w_opt_buf` (as a view, via `self._w_opt_buf[:]`).  Same for
  lam_g/lam_x/g.
  - **Alternative (preferred once confirmed):** CasADi DM supports
    `cs.DM.T` buffer protocol; if `np.frombuffer(sol['x'], dtype=np.float64)`
    works on the DM wrapper, no intermediate array is needed.  Verify in a
    unit test before committing to this path.
- Rewrite M17 `constraint_violation` as a single-pass scalar loop:
  ```python
  max_viol = 0.0
  for i in range(self._n_g):
      v = g_vals[i]
      d = max(v - self._ubg[i], self._lbg[i] - v, 0.0)
      if d > max_viol: max_viol = d
  # same for x_vals vs (_ubw, _lbw)
  ```
  No intermediate arrays.
- M13/M14/M15: track `self._prev_w` as a view into `self._w_opt_buf`;
  `self._prev_u` becomes `self._w_opt_buf[:6]`.  On the "success" branch,
  no per-tick (6,) copies — the view IS the previous cmd.  The one caveat:
  `_handle_failure` currently reads `self._prev_w` expecting it to be
  stable across ticks; pre-allocation means it IS stable, which is what
  we want.  But the per-tick `cmd = w_opt[:6].copy()` was historically
  creating a snapshot; with views, we need to be sure the plant sees a
  stable array between command issuance and the next solve.  Since
  `plant.command` is called immediately after `mpc.solve` and before the
  next solve, this is fine.
- M10 `q_cur = state.leg_extensions_mm.copy()`: view is fine; `state` is
  also pre-allocated after W4c.
- Rewrite `_build_reference_from_events` to take destination arrays:
  ```python
  def _build_reference_from_events(events, t_now, cumulative_times,
                                   ref_out, twist_out, accel_out):
  ```
  All three outputs written in place.
- Replace list comprehensions (M6–M9) with explicit index assignment:
  ```python
  for j, e in enumerate(events):
      self._ev_times_buf[j] = e.time
      self._ev_poses_buf[j] = e.pose
      ...
  ```
- Build a view `self._ev_times_buf[:len(events)]` for the rest of the
  function.  Raise if `len(events) > K_EVENTS_MAX`.

**Critical details:**
- **Correctness regression risk.**  The change from per-solve
  `np.asarray(sol[k]).ravel()` (always a fresh array) to a pre-allocated
  buffer (shared across ticks) means callers that *hold a reference to
  the previous solve's return value* will see it change.  Grep for every
  caller of `mpc.solve`:
  - `controller/runner.py:mpc_solve` reads `cmd, cmd_vel, diag` and
    passes them on — fine, they don't persist.
  - `_handle_failure` reads `self._prev_w` — handled above.
  - Tests: `tests/sim/test_mpc_*.py` store `cmd` arrays across ticks for
    comparison.  These tests need `.copy()` added at the call site.
    Grep first; fix caller-by-caller.
- **Numerical bit-exactness.**  After the refactor, run the adversarial
  MuJoCo fixture
  ([tests/sim/test_mpc_adversarial_sequences.py](../../tests/sim/test_mpc_adversarial_sequences.py))
  and compare `cmd_ext_*` values sample-by-sample against a pre-W4a
  baseline.  Pass criterion: identical to 1e-10 mm on every tick of
  every scenario.  Any deviation is a real correctness change and must
  be investigated before committing.
- `_cold_start` and `_shift_warm_start` also allocate (M's adjacent
  sites).  Include their buffers in the same pre-allocation pass.

**Dependencies:**
- W3 test exists (xfail marker) — W4a will make it go from xfail-pass
  (unexpected pass) toward xfail-fail (expected fail, progressing).

---

### Phase W4b: target source pre-allocation — NOT STARTED

**Modified files:**
- `controller/target.py`
- `controller/zmq_target.py`

**Scope:**
- `ReferenceEvent`: convert to `@dataclass(slots=True)` (Python 3.8
  compat: `__slots__ = ('time', 'pose', 'twist', 'accel')` explicitly).
- `TargetCommand`: same.  Ensure it's already non-frozen (yes — audit
  confirms).
- `StaticTargetSource`:
  - Delete the dead `prev_target = self._target.copy()` line.
  - Pre-allocate `self._tc = TargetCommand(...)` in `__init__`.
  - `update()` mutates `self._tc.target_pose = ...`,
    `self._tc.ref_events = ...`, `self._tc.boost_vel_weights = True`,
    `self._tc.warm_start_valid = warm_start_valid` and returns `self._tc`.
  - The `_cached_events` list is already rebuilt only on target change —
    unchanged.
- `ZmqTargetSource`:
  - Pre-allocate `self._tc`.
  - Cache `self._target_pose_bytes = self._target_pose.tobytes()` and
    invalidate it on the dirty bit (not recomputed every tick).  The
    `request_key` tuple becomes the only per-tick allocation on the
    steady path — acceptable if small (tuple of 3 refs ~72 B).  If
    necessary, keep `request_key` in a pre-allocated 3-tuple attribute
    and just mutate the `_last_rejected_request` comparison path.
  - Guard the Z3 branch (terminal-event hold): only rebuild the
    single-event `ref_events_out` when `sim_time` first crosses
    `events[-1].time`.  Cache a pre-allocated `_terminal_hold_event` and
    reuse.
- `WaypointTargetSource` and `AutoSequenceTargetSource`: same `TargetCommand`
  pre-alloc pattern.  Their `_cached_events` rebuild path stays
  per-target-change.

**Critical details:**
- `@dataclass(slots=True)` was added in Python 3.10.  Since `controller/`
  targets Python 3.11+ (see CLAUDE.md: "MuJoCo simulation — standalone
  Python 3.11+"), that's fine.  The ROS2 stack (`ros_ws/`) is Python 3.8
  but does NOT import `controller/target.py` directly; it imports via
  the ROS2 IPC surface.  Verify by grepping `from controller.target` in
  `ros_ws/` — should be zero hits.
- The pre-allocated `TargetCommand` shared across ticks means consumers
  CANNOT hold references to it across calls.  The MPC reads its fields
  immediately in `solve()` and doesn't retain a reference; confirm by
  grep.
- K1–K6 feasibility construction (`make_feasible_events`) still allocates
  fresh `ReferenceEvent` objects on target change.  Not on the per-tick
  path, so out of scope.  Noted explicitly in the contract.

**Dependencies:**
- W4a's buffer ownership decisions (pre-allocated vs view) inform
  whether `TargetCommand.ref_events` should be a list-of-refs or a
  pool-recycled list.  Small: keep as list-of-refs for now.

---

### Phase W4c: HardwarePlant + ipc pre-allocation — NOT STARTED

**Modified files:**
- `controller/hardware_plant.py`
- `ros_ws/src/jugglebot/jugglebot/motion/ipc.py`

**Scope:**
- `PlantState`: convert to `__slots__` (explicit, Python 3.8 compat).
  Ndarray fields become mutable references to pre-allocated buffers.
- `HardwarePlant.__init__` pre-allocates:
  - `self._state = PlantState(ext=np.empty(6), vel=np.empty(6),
     pos=np.empty(3), rot=np.empty(3), twist=np.empty(6), time=0.0,
     hand_pos_mm=None, hand_vel_mmps=None, data_age_s=None)`
  - `self._J_norm_buf = np.empty((6, 6))`
  - `self._cmd_delta_buf = np.empty(6)`
  - `self._cmd_motor_rev_buf = np.empty(6)`
  - `self._cmd_acc_buf = np.empty(6)`
  - `self._cmd_msg` as pre-allocated dict with all keys present (value =
    ref to buffer); `send_multipart` takes the refs.
  - `self._packer = msgpack.Packer(use_bin_type=True,
     default=_ndarray_default)`.  `Packer.pack(msg)` returns bytes but
    reuses internal buffers across calls.
- `get_state()`:
  - Replace `np.array(motor_pos, dtype=float)` with
    `np.copyto(self._state.leg_extensions_mm,
              np.asarray(motor_pos, dtype=np.float64) / mm_to_rev)`.
    (msgpack returns a list; the intermediate `np.asarray` is unavoidable
    unless we switch to NumPy-aware msgpack with `ext_hook`.  Log this
    as a follow-up for W7 if still dominant.)
  - Write `platform_pos_mm`, `platform_rot`, `platform_twist`,
    `leg_velocities_mmps` into the pre-allocated buffers in place.
  - Replace `self._fk_last_guess = (pos_offset, rot_matrix)` with a
    pre-allocated tuple or just two attributes.
  - Replace `pos_tuple = tuple(motor_pos)` with a stable-byte-compare
    using `bytes(np.asarray(motor_pos).tobytes())` cached across calls,
    OR — simpler — memoryview the motor_pos bytes.  Grep shows this is
    only used by the frozen-motor-pos detector, which is safety-critical
    but doesn't need the tuple form; a byte-equality compare works.
  - Return `self._state` directly (mutated in place).  Document clearly
    that callers MUST NOT retain the reference across `get_state` calls.
- `command()`:
  - Pre-allocated `ext_mm` buffer.  Callers pass an ndarray; we
    `np.copyto` into our buffer instead of `ext_mm = np.asarray(...)`.
  - Replace `delta = np.abs(ext_mm - self._last_sent_ext_mm)` with
    `np.subtract(ext_mm, self._last_sent_ext_mm, out=self._cmd_delta_buf);
     np.abs(self._cmd_delta_buf, out=self._cmd_delta_buf)`.
  - Same pattern for `motor_rev`, `acc_mm_s2`.
- `ipc.make_mpc_command`:
  - Add optional `out` kwarg: `def make_mpc_command(..., out=None)`.
  - When `out` is provided, mutate `out` in place and return `out`.
    When `out` is None, retain legacy behavior (fresh dict).  Tests
    unchanged; only `HardwarePlant.command` passes `out=self._cmd_msg`.
- `_pack()` call: use `self._packer.pack(msg)` directly; avoid the
  `[topic, packb(msg)]` helper function by inlining the 2-element list
  at the call site (or pre-allocating it).

**Critical details:**
- **Safety invariant — frozen-motor-pos detector.**  The existing code
  uses `tuple(motor_pos) == prev` for equality.  The byte-compare
  replacement MUST be equivalent: if any single encoder bit differs, the
  counter resets.  Add a test T-U3 that injects identical-bit-pattern
  followed by distinct-bit-pattern sequences and verifies the detector
  fires at the same threshold as before.
- **msgpack Packer reuse.**  `msgpack.Packer` docs: "The buffer is
  reset between calls to `pack`."  Verify via a unit test that
  consecutive `packer.pack(msg1); packer.pack(msg2)` produce the same
  bytes as `packb(msg1); packb(msg2)`.  This is T-R1.
- **PlantState mutation ordering.**  `plant.command()` reads
  `self._last_sent_ext_mm` which is a buffer.  Between `command()` and
  the next `get_state()`, nothing else writes it.  Explicit about this
  in a comment; test T-U4 verifies by running 10 command→get_state
  cycles and asserting history correctness.
- ZMQ drain (H1) `recv_multipart` will still allocate the frames list
  and msgpack dict.  True fix is out of W4c scope — flagged as a W7
  follow-up if it's the remaining bottleneck.  Partial mitigation:
  `zmq.Again` fast-path skips the alloc entirely on zero-pending; the
  existing code already does this.

**Dependencies:**
- W4b landed so `TargetCommand` no longer allocates.  Otherwise mixed
  allocation noise makes the W3 test harder to interpret.

---

### Phase W4d: telemetry + runner + hooks — NOT STARTED

**Modified files:**
- `controller/telemetry.py`
- `controller/runner.py`
- `run_mpc.py`

**Scope:**
- `StepRecord`: add `__slots__` explicitly (Python 3.8-safe).  Confirm
  `asdict()` still works on `_flush_batch` path (it does — `dataclasses.asdict`
  reads fields via `fields()` then `getattr`, both of which work on
  slotted dataclasses).
- `TelemetryLogger`: replace `self._records: list = []; append(...)` with
  a **ring buffer pool**:
  - `self._record_pool = [StepRecord() for _ in range(_POOL_SIZE)]`
    where `_POOL_SIZE = _FLUSH_EVERY + _TAIL_SIZE` (= 5200).
  - `self._head_idx = 0` cursor.
  - `append(record_args)` is gone.  New API: `logger.next_record()`
    returns the next pool slot for in-place population.
  - `record_from_arrays` becomes `fill_record_from_arrays(rec, ...)` —
    writes into the supplied `rec` instead of allocating.
  - On flush, iterate `pool[flushed_idx : self._head_idx]` and write to
    CSV.  `_TAIL_SIZE` records remain valid in the tail for summary
    stats.
- `run_mpc_loop`:
  - `extras = SimpleNamespace(overhead_ms=0.0, t_getstate_ms=0.0, ...)`
    pre-allocated outside the loop.  Per-tick: mutate fields.
  - `log_mpc_step` call passes `extras` by reference; `fill_record_from_arrays`
    reads `extras.overhead_ms` etc.
  - `_t_seg`, `_t_overhead` are already scalar floats — no change.
  - `_dash_sink` already pre-allocated.
- Hook closures in `run_mpc.py`:
  - `_on_pre_command`: pre-allocate `_twist_buf`, `_twist_next_buf`,
    `_accel_buf` as (6,) arrays in the outer scope.  Use
    `np.subtract(..., out=buf)` and `np.divide(..., out=buf)`.
  - `_on_log_extras`: pre-allocate `_log_extras_ns = SimpleNamespace(
       fk_iterations=0, ff_torque_max_Nm=0.0)`.  Return the namespace;
    update runner to read from it.  (This means `extras.update(hook_extras)`
    becomes `extras.fk_iterations = hook_extras.fk_iterations;
    extras.ff_torque_max_Nm = hook_extras.ff_torque_max_Nm`.  Two writes,
    no dict.)

**Critical details:**
- The ring buffer re-uses StepRecord slots.  When `_flush_batch` is called
  the flushed records become re-usable.  This means reading
  `logger.records` after a flush shows the NEW tail, not the old flushed
  records.  Existing callers only read `logger.records[-1]` (the most
  recent record) or `logger.records[-N:]` (last N for summary stats) —
  both still work.  Verify by grepping.
- `_TAIL_SIZE` unchanged at 200.  The 5000 records-before-flush pattern
  is unchanged; pool just sizes the buffer.
- `_log_extras_ns`'s field set must match exactly what `record_from_arrays`
  accepts.  Introduce the namespace schema in a shared module so runner
  and hooks share definitions (e.g. `controller/runner_schemas.py`).
- CSV load-back (`TelemetryLogger.load`) reads from disk — unaffected by
  pool changes.

**Dependencies:**
- W4c landed so PlantState allocation isn't still polluting the W3 measurement.

---

### Phase W4e: lift xfail + merge — NOT STARTED

**Modified files:**
- `tests/sim/test_hot_loop_allocation_contract.py`

**Scope:**
- Remove the `@pytest.mark.xfail` decorator.
- Run `pytest tests/ -v`.  Expected: `1034 passed` (1033 baseline + 1 W3).
- If it fails, the top-10 diagnostic names the remaining site(s); fix
  individually and commit.  `THRESHOLD_BYTES` stays at 1024.

**Critical details:**
- If a site not in the W1 inventory shows up, append it to the W1 logbook
  entry (the inventory is part of the audit trail, not a throwaway
  artifact).

**Dependencies:**
- W4a–d all merged and the W3 test observably passes on a local run.

---

### Phase W5: GC disable wrapper — NOT STARTED

**Modified files:**
- `controller/runner.py`

**Scope:**
- Wrap the `for _step_idx in range(n_steps)` body with:
  ```python
  _gc.disable()
  try:
      for _step_idx in range(n_steps):
          ...  # existing body
          # periodic between-tick collect, scheduled on idle sleep
          if _step_idx % 200 == 0 and sleep_time > 0:
              _t_gc_start = _time.perf_counter()
              _gc.collect(generation=2)
              _gc_manual_ms = (_time.perf_counter() - _t_gc_start) * 1000.0
              # subtract from the sleep budget
              sleep_time -= _gc_manual_ms / 1000.0
  finally:
      _gc.enable()
  ```
- The manual `gc.collect(generation=2)` runs once per 200 ticks = once
  every 5 seconds.  Gen 2 sweep on the Orin Nano at this allocation rate
  (<1 KB/tick) is expected to be <5 ms, since there's almost nothing to
  trace.
- Tighten existing `_GCTracker` into an assertion: if `_gc_count > 0`
  on any tick whose `sleep_time` was > 0 at entry, log a WARNING
  (indicates an in-tick collection slipped through).
- Keep `_gc.enable()` in the `finally` so an exception doesn't leave
  the interpreter with GC disabled.

**Critical details:**
- **Never disable GC globally.**  The wrapper is scoped to the loop.
  Other threads / processes / tests using GC are unaffected.
- **Memory leak risk.**  With GC disabled, any *reference cycle* created
  by a tick is not collected.  Refcount-based collection still runs —
  99 % of Python cycles in scientific code are refcount-closed.  Verify
  with a 10-minute sim run and `sys.getallocatedblocks()` — growth
  should be linear in CSV size, not super-linear.  This is T-I3.
- `gc.collect(generation=2)` with `gc.disable()` active: the call is
  still legal and runs the full sweep.  Python docs confirm.
- The `200 ticks` cadence is empirical; if T-I3 shows meaningful
  allocation accumulation, drop to 100.  If it shows zero accumulation,
  raise to 400 to reduce the 5 ms periodic cost further.

**Dependencies:**
- W4e merged (contract passing).

---

### Phase W6: Hardware validation — NOT STARTED (STOP GATE)

**New files:**
- New logbook entry (amend
  [logbook/2026-04-23-hot-loop-zero-allocation-contract.md](../../logbook/2026-04-23-hot-loop-zero-allocation-contract.md)
  or new dated entry, whichever fits).

**Scope:**
- On the Jetson, with bridge + CAN + motor guard running:
  ```bash
  source ~/Desktop/PDJ_venv/venv/bin/activate
  python run_mpc.py --pose 0,0,220,0,0,0 --duration 60
  ```
- Diagnose the resulting session:
  ```bash
  /diagnose --latest
  ```
- Inspect `overhead.isolated_spikes` field.  Pass criterion: **== 0**
  for a 60 s run with no target changes, and **<= 1** for a 60 s run
  with 4 target changes (`(0,0,220) → (0,0,170) → (0,0,220) → (0,0,170)`).
- Inspect per-tick `gc_ms` column in the CSV.  Pass criterion: zero
  non-zero rows except at the scheduled `_step_idx % 200 == 0` ticks
  (and even those should be < 5 ms).
- Compare baseline: run the same two scenarios on the pre-W2 commit,
  record `isolated_spikes` count, compute delta.

**Critical details:**
- This is the stop gate.  If hardware still shows isolated spikes, the
  W4 inventory was incomplete; go back to W1 and find the missed site.
  **Do not** continue to W7 or declare the contract resolved until
  hardware validation passes.
- Safety: operator on E-stop throughout, per Phase 4 bringup convention.
- Record both CSVs (before and after) in the logbook entry's `sessions:`
  frontmatter so future /diagnose calls can re-read them.

**Dependencies:**
- W5 merged (GC disabled on the hot loop).

---

### Phase W7: Threshold ratchet — NOT STARTED

**Modified files:**
- `controller/HOT_LOOP_CONTRACT.md` (threshold update)
- `controller/hot_loop_contract.py` (or wherever `THRESHOLD_BYTES` lives)

**Scope:**
- Change `THRESHOLD_BYTES = 1024` → `THRESHOLD_BYTES = 256`.
- Run W3 test.  If it fails, top-10 names the remaining site(s).  Fix
  one by one.  Typical candidates at this level: ZMQ drain list alloc
  (H1), list copies from `.recv_multipart()`, last residual `np.array(...)`
  calls that survived W4 because they allocated < 200 B individually.
- Update the contract doc's "threshold" section to document the new
  floor and the rationale: "After W4+W5 shipped, the remaining ~800 B
  slack above 256 B was unnecessary and caused instrumentation drift on
  extensions of `on_log_extras` / new hooks; tightening forces every
  new hot-path addition to be zero-alloc by construction."

**Critical details:**
- Order matters: ratchet only after W6 is verifiably green.  A tighter
  threshold catches regressions but also risks a false-positive CI
  failure from interpreter-internal allocations.  If such a false positive
  happens, the fix is to raise the threshold a small amount (say, to
  320 B) with a contract-doc comment explaining the irreducible
  interpreter floor.

**Dependencies:**
- W6 passed.

---

## Testing Plan

### Unit tests (offline, no hardware)

| ID | Test | Pass criterion |
|----|------|----------------|
| T-U1 | `test_step_record_slots_compatible` | `dataclasses.asdict(StepRecord(time=1.0))` returns the correct dict; `fields(StepRecord)` lists all declared fields |
| T-U2 | `test_target_command_preallocated_mutation` | `StaticTargetSource` returns the SAME `TargetCommand` object id across consecutive `update()` calls; fields update in place |
| T-U3 | `test_frozen_motor_pos_byte_compare_equivalent` | Byte-compare detector fires on ≥N identical encoder packets exactly when the previous `tuple(...)` detector did; bit-level perturbation on the N-th packet resets the counter identically |
| T-U4 | `test_plant_state_mutation_safe_between_calls` | 10 `command()→get_state()` cycles: each `get_state()` returns the *same* PlantState object reference; leg_extensions_mm contents reflect the freshest motor_pos, never a stale copy |
| T-U5 | `test_build_reference_from_events_in_place` | `_build_reference_from_events(events, t, ct, ref_out, twist_out, accel_out)` writes correct values; identical to pre-refactor output on 50 K1–K6-compliant event lists |
| T-U6 | `test_mpc_solve_buffer_reuse_bit_exact` | Run `mpc.solve(...)` twice with identical inputs.  Second call's `cmd`, `cmd_vel`, `cost`, `constraint_violation` bit-identical to first (bit-identical, not `np.allclose`).  Confirms the buffer path didn't introduce stale-data bugs |
| T-U7 | `test_mpc_solve_view_aliasing_safety` | After `cmd, cmd_vel, diag = mpc.solve(...)`, call `plant.command(cmd, vel_mm_s=cmd_vel)`.  Then call `mpc.solve(...)` a second time.  Assert that the `cmd` from the first call now holds the SECOND call's values (i.e. it IS a view).  This is the flip side of T-U6 — documents the aliasing contract so future contributors don't rely on the old copy semantics |
| T-U8 | `test_telemetry_pool_rotation` | Run 6000 `logger.next_record()` + fill + append sequence (exceeds `_FLUSH_EVERY=5000`).  Verify: CSV on disk has 6000 rows, pool grew to no more than `_POOL_SIZE`, `logger.records` returns last 200 |
| T-U9 | `test_ipc_make_mpc_command_out_kwarg` | `make_mpc_command(..., out=msg)` mutates `msg` in place and returns it; `make_mpc_command(..., out=None)` returns a fresh dict (legacy path).  Both produce identical msgpack-serialised bytes |
| T-U10 | `test_msgpack_packer_reuse_bit_exact` | `packer.pack(msg)` for 100 consecutive calls produces identical bytes to `packb(msg)` on each |
| T-U11 | `test_k_events_max_raises_on_overflow` | `mpc.solve(ref_events=[... K_EVENTS_MAX+1 events ...])` raises `ValueError` with a clear message.  Keeps the buffer bounded; doesn't silently truncate |
| T-U12 | `test_dead_prev_target_copy_removed` | Grep-equivalent: assert `StaticTargetSource.update` source does NOT contain `prev_target = self._target.copy()` (sentinel test; rots fast but catches accidental revert) |
| T-U13 | `test_violation_scalar_loop_bit_exact` | On 1000 random `(g_vals, lbg, ubg)` triples, the new scalar loop and the old `np.maximum` chain produce `float(np.max(violation))` identical to the last bit |

### Integration tests (sim, no hardware)

| ID | Test | Pass criterion |
|----|------|----------------|
| T-I1 | `test_hot_loop_allocation_contract` | The W3 test itself.  (a) With `@pytest.mark.xfail` during W4 in-flight — xfail as expected.  (b) Without xfail after W4e — passes at 1024 B/tick.  (c) After W7 — passes at 256 B/tick |
| T-I2 | `test_mpc_adversarial_sequences_unchanged` | Run [tests/sim/test_mpc_adversarial_sequences.py](../../tests/sim/test_mpc_adversarial_sequences.py) on post-W4 code.  Every scenario `cmd_ext_*` telemetry identical to pre-W4 baseline sample-by-sample to 1e-10 mm.  No numerical regressions |
| T-I3 | `test_gc_disabled_loop_no_unbounded_growth` | Run `run_mpc_loop` for 24 000 ticks (= 10 min at 40 Hz) with W5's `gc.disable()` wrapper active.  Record `sys.getallocatedblocks()` every 1000 ticks.  Assert growth after the first 5000 ticks is **linear** with slope < 2 blocks/tick (this is the CSV record append, not a leak).  Super-linear growth = reference cycle accumulation, fail |
| T-I4 | `test_gc_tracker_zero_in_tick_collections` | With W5 active, run 1000 ticks.  Assert `_GCTracker` records zero GC events inside the tick window (between `_t_overhead` start and log end).  All GC events must fall in the sleep region |
| T-I5 | `test_target_command_alias_safety` | Run 100 ticks with `ZmqTargetSource` fed a scripted stream.  Verify the MPC sees correct field values on every tick; verify no cross-tick data corruption from the shared `TargetCommand` object |
| T-I6 | `test_full_pytest_suite_green` | `pytest tests/ -v` exits 0.  Reports 1034 passed (1033 + W3) |

### Hardware tests (real actuators, E-stop ready)

| ID | Test | Pass criterion |
|----|------|----------------|
| T-H1 | `run_mpc.py --pose 0,0,220 --duration 60` baseline | Pre-W2 commit: record `isolated_spikes` and `gc_ms` max from `/diagnose` |
| T-H2 | `run_mpc.py --pose 0,0,220 --duration 60` after W6 | Post-W5: `isolated_spikes == 0`, `gc_ms` max < 5 ms |
| T-H3 | 4-target rapid-succession hardware workout (same sequence as the K1–K6 `mpc_20260420_160242` session) | post-W5: `isolated_spikes == 0`; success rate unchanged (>= 99 %); no E-stop |
| T-H4 | 10-minute endurance run (sustained hold at `(0,0,220)`) | post-W5: zero cmd discontinuities in telemetry; linear memory growth (mirror T-I3 on hardware); no E-stop |

### Regression tests (bit-exact comparisons)

| ID | Test | Pass criterion |
|----|------|----------------|
| T-R1 | `test_msgpack_bytes_bit_exact` | For 100 synthetic `make_mpc_command` calls, `self._packer.pack(msg)` output bit-identical to `msgpack.packb(msg, use_bin_type=True, default=...)` output |
| T-R2 | `test_csv_schema_unchanged` | CSV column order and types identical to pre-W4d output (diff the header row of two runs) |
| T-R3 | `test_reference_layer_contract_still_satisfied` | All K1–K6 property tests in [tests/sim/test_make_feasible_events.py](../../tests/sim/test_make_feasible_events.py) pass.  Confirms W4b's `ReferenceEvent` slots change didn't break feasibility-layer semantics |
| T-R4 | `test_zmq_target_protocol_unchanged` | [tests/sim/test_zmq_target.py](../../tests/sim/test_zmq_target.py) passes.  Confirms W4b's `TargetCommand` pre-alloc didn't break ZMQ target decode |
| T-R5 | `test_k1_k6_adversarial_unchanged` | [tests/sim/test_mpc_adversarial_sequences.py](../../tests/sim/test_mpc_adversarial_sequences.py) — all 15 scenarios pass with bit-exact cmd_ext output |

---

## Notes for Collaborators

### Safety-critical invariants that must be preserved

| Invariant | Location | Consequence if violated |
|-----------|----------|-------------------------|
| **K1–K6 reference-feasibility contract** | [controller/REFERENCE_LAYER_CONTRACT.md](../../controller/REFERENCE_LAYER_CONTRACT.md); enforcement in `make_feasible_events` | Re-introduction of `MPC_OVERSHOOT_SATURATION` class of failures |
| **`ReferenceEvent` field set** (time, pose, twist, accel) | [controller/target.py:36-64](../../controller/target.py#L36-L64) | `make_feasible_events` breaks; K1–K6 property tests fail |
| **`PlantState` numerical contents, not object identity** | [controller/plant.py:16-34](../../controller/plant.py#L16-L34) | Mutation is safe IF and ONLY IF consumers read field values synchronously within one tick.  Any consumer that retains a PlantState reference across ticks (tests, loggers, dashboard snapshots) will see field-level corruption.  Grep before adding any such retention: `# RETAINS PlantState:` |
| **ZMQ wire format** (msgpack dict keys and types) | [ros_ws/src/jugglebot/jugglebot/motion/ipc.py](../../ros_ws/src/jugglebot/jugglebot/motion/ipc.py) | Motor guard / bridge node decode failures; hardware unresponsive |
| **CSV column schema** (`StepRecord` field order) | [controller/telemetry.py:19-111](../../controller/telemetry.py#L19-L111) | Downstream `/diagnose` / analysis scripts break.  `__slots__` preserves field order via `fields()`; regression T-R2 guards this |
| **Frozen-motor-pos detector threshold** (_FROZEN_MOTOR_POS_ESTOP) | [controller/hardware_plant.py](../../controller/hardware_plant.py) near line 490 | Missed telemetry-dead E-stop; runaway commands.  T-U3 guards the byte-compare equivalence |
| **CasADi solver return numerical semantics** | [controller/mpc.py:839-1091](../../controller/mpc.py#L839-L1091) | Numerically incorrect MPC commands — the solver contract is that `w_opt[:6]` is the optimal u[0].  T-I2 guards bit-exactness over the adversarial fixture |
| **GC still available globally** | [controller/runner.py](../../controller/runner.py) — W5 change | `gc.disable()` is scoped via try/finally.  An exception path that skips the `finally` would leave the interpreter with GC off.  Validate by running the pytest suite with `-x` (exit on first fail) and asserting `gc.isenabled() is True` at the end of every test |

### Architecture decisions

- **Why `SimpleNamespace` over `__slots__` dataclass for `extras` and `diag`?**
  They're used as ad-hoc collections of timing fields.  `SimpleNamespace`
  is already slotted internally, allows trivial `ns.field = value`
  mutation, and doesn't require maintaining a class declaration alongside
  the field set.  `__slots__` dataclasses make more sense for
  `StepRecord` (many fields, named types, CSV schema is load-bearing) and
  `PlantState` (abstract interface, publicly documented).
- **Why a record pool in `TelemetryLogger` rather than pre-allocate 5000
  records and index them?**  The pool IS pre-allocated at 5200 slots.
  The pool just wraps the `list[StepRecord]` API.  Chose this over a
  ring buffer because ring buffers complicate the "flush the oldest
  5000, keep the last 200" semantics that `_flush_batch` already
  implements.
- **Why not remove `ReferenceEvent` entirely and use structured ndarrays?**
  Considered and rejected.  Structured ndarrays are clunky to build and
  read (named axes, no method dispatch), and the per-target-change path
  where `ReferenceEvent` is allocated is not on the hot tick path.
  `__slots__` on the existing dataclass gets 80 % of the benefit with
  zero API churn.
- **Why `THRESHOLD_BYTES = 1024` initially rather than `256` directly?**
  The 1024 B ceiling lets W4a–d land incrementally — each sub-phase
  reduces allocations by some amount, and the test can stay green
  throughout rather than go red/green/red/green.  Tightening to 256 B
  in W7 comes *after* stabilisation and with explicit diagnostic output
  if any single call site is over-budget.
- **Why `msgpack.Packer` reuse rather than a raw-bytes fast path?**
  `Packer` has documented safe reuse semantics; rewriting the msgpack
  encode path is out of scope and has its own correctness risks (e.g.
  `default=_ndarray_default` callback).  Reuse gets ~80 % of the benefit
  at near-zero risk.

### Startup/shutdown ordering

No change to process startup/shutdown ordering.  The W5 `gc.disable()`
scope is strictly inside `run_mpc_loop`.  Pre-allocation happens in
each component's `__init__`, which is already in the startup path.

Tear-down path: `logger.flush()` writes any records the ring buffer
holds but hasn't yet flushed — unchanged from current behavior.

### Files affected

| File | Action | Phase |
|------|--------|-------|
| `controller/HOT_LOOP_CONTRACT.md` | CREATE | W2 |
| `controller/hot_loop_contract.py` | CREATE (holds `THRESHOLD_BYTES`) | W2 |
| `tests/sim/test_hot_loop_allocation_contract.py` | CREATE | W3 |
| `controller/mpc.py` | MODIFY (pre-allocate solver-return, ref arrays, event buffers, diag namespace) | W4a |
| `controller/target.py` | MODIFY (`ReferenceEvent` / `TargetCommand` slots, `StaticTargetSource` / `WaypointTargetSource` / `AutoSequenceTargetSource` pre-alloc, delete dead `prev_target.copy()`) | W4b |
| `controller/zmq_target.py` | MODIFY (pre-alloc `TargetCommand`, cache `target_pose.tobytes()`) | W4b |
| `controller/hardware_plant.py` | MODIFY (pre-alloc `PlantState`, command buffers, MPC-command dict, `msgpack.Packer` reuse) | W4c |
| `ros_ws/src/jugglebot/jugglebot/motion/ipc.py` | MODIFY (`make_mpc_command(..., out=None)` kwarg) | W4c |
| `controller/telemetry.py` | MODIFY (`StepRecord` slots, record pool) | W4d |
| `controller/runner.py` | MODIFY (`extras` SimpleNamespace, `gc.disable()` wrapper) | W4d + W5 |
| `run_mpc.py` | MODIFY (hook closures pre-allocate buffers) | W4d |
| `logbook/2026-04-23-hot-loop-zero-allocation-contract.md` | UPDATE (add per-phase outcomes, W6 hardware results) | W1–W7 ongoing |

### Rollback plan

Each W is its own commit.  Rollback is `git revert <commit>`.  The key
safety property: W4a–d are all *internal-mutation-without-API-changes*,
so reverting a later phase does not break an earlier one.  The only
non-revertable commit is W5's `gc.disable()` — but that's a 20-line
wrapper around the existing `for` loop; reverting to `gc.enable()`-always
restores the pre-W5 behaviour exactly.

If a hardware regression is discovered after W6 and the bisect narrows
to W4 sub-phases, the rollback path is: `git revert` the offending Wn,
`THRESHOLD_BYTES` reverts to the larger value from the contract doc,
CI stays green at the relaxed threshold while the underlying bug is
investigated in a new commit.

**Irreversible risks:** none.  No schema changes, no IPC format changes,
no hardware-side persistence touched.  Every change is observable on a
single Jetson without coordinating with external systems.
