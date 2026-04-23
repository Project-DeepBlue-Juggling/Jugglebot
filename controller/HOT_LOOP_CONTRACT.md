# Hot-Loop Zero-Allocation Contract

This document is the **normative specification** of the zero-allocation
invariant that every implementation touching the MPC 40 Hz hot loop must
satisfy.  It exists so that new ``TargetSource`` implementations, new
``PlantInterface`` implementations, new ``MpcLoopHooks`` callbacks, and
new telemetry fields can be added without reintroducing the class of
single-sample GC-pause-induced cmd discontinuities that W1–W7 of the
2026-04-23 work structurally eliminated.

It is the direct companion to
[REFERENCE_LAYER_CONTRACT.md](REFERENCE_LAYER_CONTRACT.md) and uses the
same normative language (MUST, MUST NOT, SHOULD, MAY per RFC 2119).

## Background

The Jugglebot MPC runs at 40 Hz (25 ms tick period).  Pre-contract, every
hardware session's diagnosis JSON flagged 1–3
``GC_PAUSE_CANDIDATE`` spikes per minute — 40–67 ms single-sample
overhead blips.  Each spike forced the loop into walk-forward fallback
for that tick and left a visible cmd discontinuity in telemetry.  W7 of
the K1–K6 contract (see
[REFERENCE_LAYER_CONTRACT.md](REFERENCE_LAYER_CONTRACT.md) and
[logbook/2026-04-20-k1-k6-reference-feasibility-resolution.md](../logbook/2026-04-20-k1-k6-reference-feasibility-resolution.md))
handled the resulting fallback *safely*, but did not *eliminate* the
trigger.

The trigger is Python's generational GC.  On the Jetson Orin Nano a
Gen 2 sweep of CasADi solver state plus per-tick numpy arrays takes
40–80 ms — well over the 25 ms tick period.  Gen 2 fires when
per-generation allocation counts cross thresholds, so eliminating the
pauses means eliminating the hot-loop allocations that drive Gen 2
pressure.

The W1 audit
([logbook/2026-04-23-hot-loop-zero-allocation-contract.md](../logbook/2026-04-23-hot-loop-zero-allocation-contract.md))
measured ~19 KB of Python allocations per steady-state tick pre-contract
— primarily ``np.asarray(sol[k]).ravel()`` solver-return conversions,
the ``constraint_violation`` ``np.maximum`` temporary chain, per-tick
``StepRecord`` / ``PlantState`` / ``TargetCommand`` dataclasses, and
per-tick ``extras`` / ``diag`` / ``msg`` dicts.  This contract closes the
whole class of failures at their source: every component on the hot
loop now pre-allocates its buffers and mutates them in place, and the
enforcement test fails CI on any future-added allocation.

## Scope

### The hot-loop body

**Normative definition.**  The hot-loop body is the Python code
executed between the following two points of
[controller/runner.py::run_mpc_loop](runner.py), inclusive:

1. **Start:** ``state = plant.get_state()`` (the first per-tick call at
   the top of the ``for _step_idx in range(n_steps)`` block after all
   lifecycle / estop guards).
2. **End:** the final statement of ``log_mpc_step(...)`` (the last
   per-tick telemetry write before the wall-clock pacing sleep).

Everything executed between those two points, transitively, is on the
hot path.  This includes:

- ``plant.get_state()`` and everything it calls (FK, J⁻¹ twist solve,
  ZMQ drain).
- ``source.update()`` (and ``source.poll()`` when invoked).
- ``hooks.on_target_override``, ``hooks.on_pre_command``,
  ``hooks.on_post_solve``, ``hooks.on_post_step``,
  ``hooks.on_log_extras`` — any that are wired.
- ``mpc.solve()`` — including reference construction, parameter pack,
  warm-start shift, solver invocation, and post-solve extraction.
- ``plant.command()`` and ``plant.step()``.
- ``log_mpc_step`` → ``record_from_arrays`` → ``logger.append``.
- Any callable reached transitively from the above.

### The invariant

**The hot-loop body MUST allocate no more than ``THRESHOLD_BYTES`` of
Python objects per steady-state tick**, where ``THRESHOLD_BYTES`` is
imported from [controller/hot_loop_contract.py](hot_loop_contract.py).

"Steady state" means: the source is returning a stable ``TargetCommand``
(no target transition), the MPC solver is converging (no fallback), the
plant is delivering fresh telemetry (no stale-age E-stop), no dashboard
is attached.

"Python objects" means: anything that ``tracemalloc`` attributes to a
hot-loop call site.  Explicitly excluded:

- **CasADi-internal buffers** (DM matrices, IPOPT working memory).  These
  are C-level allocations invisible to ``tracemalloc`` and do not drive
  Python's generational GC, which is the failure mode this contract
  defends against.
- **msgpack Packer internal buffers** (when used with
  ``msgpack.Packer`` reuse).  Same reason — C-level.
- **OS kernel buffers** from ZMQ socket operations.  Same reason.

### Threshold values

| Value | Effective (date) | Rationale |
|------:|------------------|-----------|
| `1024 B/tick` | W4e (2026-04-23) | Relaxed ceiling that let the W4 sub-phases land incrementally while the enforcement test stayed green at each commit.  Superseded by the W7 ratchet below. |
| `256 B/tick`  | **W7 (2026-04-23, current)** | Tightened to ~2x the measured 137 B/tick floor once the full W1 inventory shipped.  Enough headroom for minor interpreter-internal variation (tracemalloc's own filter machinery, f-string intern churn) but strict enough that any new per-tick dict, ndarray, or similar-size retained object trips CI.  Values tighter than ~200 B/tick trip on interpreter-internal allocations — 256 B is the empirical floor. |

The threshold MUST be raised (a "relaxation") only via a commit that:

1. Explains the cause of the new floor in the commit message and in
   [hot_loop_contract.py](hot_loop_contract.py)'s comment block.
2. Updates this document's threshold table with the new value, new
   effective-after-phase name, and new rationale.
3. Keeps the ratchet monotone: relaxations are documented; they do not
   silently raise the floor.

The threshold MAY be tightened (further ratchet) at any point.

### Per-target-change and per-fallback allocations are out of scope

The contract applies to **steady-state ticks only**.

- **Per-target-change allocations** (K1–K6 quintic rebuild inside
  ``make_feasible_events``, fresh ``ReferenceEvent`` list construction,
  stretch binary-search) run once per target transition.  These are
  out of scope — the allocation count on a target-change tick will
  legitimately be thousands of bytes larger than a steady-state tick.
- **Per-fallback allocations** (``_handle_failure`` body) run when the
  solver status is a fallback / hold / cold_hold class.  Steady state
  is solver success; fallback ticks are excluded from the contract
  measurement.

Both paths SHOULD still avoid gratuitous allocation where easy — but
neither is CI-enforced, because both are rare and their cost is already
dwarfed by the event that triggered them (a new target, a solver
failure).

## Enforcement

### The contract test

All contract compliance is enforced in one place:
[tests/sim/test_hot_loop_allocation_contract.py](../tests/sim/test_hot_loop_allocation_contract.py).

The test:

1. Builds a sim fixture — ``MuJoCoPlant`` + pre-built ``MPCController`` +
   ``StaticTargetSource`` holding at ``(0, 0, 170, 0, 0, 0)``.
2. Runs ``run_mpc_loop`` for ``HOT_LOOP_CONTRACT_WARMUP_TICKS`` ticks
   (default 50) — warmup for CasADi JIT, MPC prime, first-target quintic
   build.
3. Starts ``tracemalloc`` with
   ``HOT_LOOP_CONTRACT_TRACEBACK_FRAMES`` frames of traceback, takes
   snapshot S1.
4. Runs ``run_mpc_loop`` for ``HOT_LOOP_CONTRACT_WINDOW_TICKS`` more
   ticks (default 100).  Takes snapshot S2.
5. Asserts ``(S2.total - S1.total) / WINDOW_TICKS < THRESHOLD_BYTES``.
6. On failure, emits the top-10 entries from
   ``S2.compare_to(S1, 'lineno')`` — each entry is a
   ``(file.py:lineno, bytes_added, count_added, traceback)`` tuple.
   This is the diagnostic affordance: a future contributor whose commit
   failed CI sees exactly which call site grew, in bytes and count,
   without having to rerun locally.

### CI expectations

The test runs as part of the default ``pytest tests/ -v`` invocation.
It MUST pass on every commit to ``main``.  A commit that trips the test
and cannot be resolved by pre-allocation MUST either:

1. Relocate the offending code off the hot path (e.g. compute in
   ``__init__`` or in a per-target-change callback); or
2. Accept a threshold relaxation via the procedure above and land both
   the code change and the relaxation in one atomic commit.

Silent threshold bumps are contract violations even if the test passes.

### Local verification during development

```bash
# Just run the contract test
pytest tests/sim/test_hot_loop_allocation_contract.py -v

# See the full top-10 diagnostic on a failing run
pytest tests/sim/test_hot_loop_allocation_contract.py -v -s
```

The ``-s`` flag is useful: on failure, the top-10 diagnostic is printed
via ``print()`` rather than as a pytest assertion message, and pytest
only shows printed output when ``-s`` is passed.

## Implementation guidance

This section is non-normative but strongly recommended patterns for new
contributors.

### Patterns to use

**Pre-allocated buffers with ``np.copyto`` / ``out=`` kwargs.**  Instead
of ``arr = np.array([...])`` or ``arr = a - b``, pre-allocate a buffer
once and write in place:

```python
# In __init__:
self._delta_buf = np.empty(6)

# In the hot path:
np.subtract(ext_mm, self._last_ext_mm, out=self._delta_buf)
np.abs(self._delta_buf, out=self._delta_buf)
# use self._delta_buf
```

**``__slots__`` dataclasses for named per-tick records.**  ``StepRecord``
and ``PlantState`` are the canonical examples.  Prefer an explicit
``__slots__ = (...)`` tuple for Python 3.8 compatibility (``ros_ws/``
code path); ``@dataclass(slots=True)`` is fine for ``controller/`` which
targets 3.11+.

**``types.SimpleNamespace`` for ad-hoc field bundles.**  ``extras`` and
``diag`` use this.  Mutate with ``ns.field = value``; no class
declaration needed.

**Views instead of ``.copy()``.**  When a callee reads but does not
outlive the call, pass a view (``arr[:]``) or a slice (``arr[:6]``) —
slicing an ndarray does NOT copy, it creates a view.  If the callee
does outlive the call but only needs an immutable snapshot, consider
whether the snapshot is really needed or whether a pre-allocated twin
buffer + ``np.copyto`` suffices.

**Pool-recycle instead of append.**  ``TelemetryLogger``'s record pool
pre-allocates 5200 ``StepRecord``s; ``next_record()`` hands out the
next slot for in-place population; ``_flush_batch`` iterates the
filled prefix and writes to CSV.

**CasADi DM → pre-allocated numpy.**  Instead of
``np.asarray(sol['x']).ravel()`` (fresh array every tick), copy into a
pre-allocated buffer:

```python
# In __init__:
self._w_opt_buf = np.empty(self._n_w)

# In solve():
np.copyto(self._w_opt_buf, np.asarray(sol['x']).ravel())
# or (once verified safe) exploit the DM buffer protocol directly
```

### Patterns to avoid

**Dict literals in the hot path.**  ``{'key': value, ...}`` allocates a
fresh dict every call.  Use a pre-allocated ``SimpleNamespace`` or
``__slots__`` class, or mutate a pre-allocated dict (with an ``out=``
kwarg pattern — see ``make_mpc_command``).

**List comprehensions into ``np.array``.**  ``np.array([e.pose for e in
events])`` allocates both a temporary list and a fresh ndarray.  Pre-
allocate ``(K_MAX, 6)`` and index-assign; take a view of the filled
prefix for downstream consumers.

**``arr.copy()`` in the hot path.**  Either the callee outlives the
call (rare — usually the callee just reads and returns), in which case
a pre-allocated twin buffer + ``np.copyto`` is the pattern; or the
callee doesn't outlive (common), in which case a view works.

**``np.concatenate`` or ``np.stack`` in the hot path.**  Allocates a
fresh output every call.  Pre-allocate and index-assign into slices.

**Broadcasting into fresh arrays.**  ``out = a - 2 * b + c`` allocates
``(2 * b)``, ``(a - 2 * b)``, and finally the output.  Use
``np.subtract(a, ..., out=buf)`` or explicit scalar arithmetic.

**Multi-element ``np.maximum`` temporary chains.**  ``np.maximum(0.0, a -
b)`` allocates.  For scalar reductions (``float(np.max(...))``), write
an explicit scalar loop instead — on 6–200 element arrays this is
faster than numpy anyway.

### Marker convention

Functions whose body is reached from the hot-loop body SHOULD include
the marker string ``hot-loop body`` somewhere in their docstring.
Grep for this string to enumerate the hot-path surface:

```bash
grep -r "hot-loop body" controller/ ros_ws/src/jugglebot/
```

The marker is non-normative — failing to include it does not fail CI —
but makes the contract's surface navigable for a new contributor.

## Implementing a new ``TargetSource``

Template for a contract-compliant source:

```python
class MyTargetSource:
    def __init__(self, ..., v_max_mmps, tau_s):
        ...
        # K1–K6 bookkeeping (see REFERENCE_LAYER_CONTRACT.md)
        self._v_max_mmps = v_max_mmps
        self._tau_s = tau_s
        self._cached_events: list[ReferenceEvent] | None = None
        self._prev_ref_end_pose: np.ndarray | None = None
        self._prev_ref_end_twist: np.ndarray | None = None

        # Hot-loop contract: pre-allocate the return TargetCommand.
        # Field values are mutated in place by update().  Callers MUST
        # NOT retain references across calls — field values change on
        # the next tick.
        self._tc = TargetCommand(
            target_pose=np.empty(6),
            ref_events=None,
            boost_vel_weights=False,
            warm_start_valid=True,
        )

    def update(self, sim_time, state):
        """Return the MPC target for this tick (hot-loop body)."""
        target_pose, target_twist, arrival_time = self._pick_target(sim_time, state)

        # Per-target-change path: OK to allocate here (out of contract scope).
        if self._target_changed(target_pose, target_twist, arrival_time):
            events, reason = flat_target_to_events(
                pose_6dof_from_state(state), state.platform_twist,
                target_pose, sim_time,
                target_twist=target_twist, arrival_time=arrival_time,
                v_max_mmps=self._v_max_mmps, tau_s=self._tau_s,
                return_reason=True,
            )
            # ...handle reason...
            self._cached_events = events
            # W5 warm-start hint, per REFERENCE_LAYER_CONTRACT.md
            self._warm_start_valid_hint = not is_warm_start_invalidating(
                self._prev_ref_end_pose, self._prev_ref_end_twist,
                events[-1].pose, events[-1].twist,
            )

        # Per-tick path: MUTATE the pre-allocated TargetCommand in place.
        # DO NOT return a fresh TargetCommand(...).
        np.copyto(self._tc.target_pose, target_pose)
        self._tc.ref_events = self._cached_events
        self._tc.boost_vel_weights = True
        self._tc.warm_start_valid = self._warm_start_valid_hint
        self._warm_start_valid_hint = True  # one-tick latch
        return self._tc
```

### Caveats

- The returned ``TargetCommand`` is the SAME object on every call.
  Consumers (``run_mpc_loop``, ``mpc.solve``) read its fields
  synchronously within a single tick and do NOT retain references.  If
  you add a consumer that needs a cross-tick snapshot, copy the fields
  explicitly — do not retain the object.
- ``self._cached_events`` may be a list of shared ``ReferenceEvent``
  objects.  MPC ``_build_reference_from_events`` reads the fields and
  copies into its pre-allocated ref arrays, so the shared-reference
  pattern is safe.
- Never go around ``make_feasible_events``.  That's a K1–K6 contract
  requirement; this hot-loop contract is orthogonal to it.

## Implementing a new ``PlantInterface``

Template:

```python
class MyPlant(PlantInterface):
    def __init__(self, ...):
        ...
        # Hot-loop contract: pre-allocate the PlantState and all its
        # ndarray fields once.  get_state() mutates them in place.
        self._state = PlantState(
            leg_extensions_mm=np.zeros(6),
            leg_velocities_mmps=np.zeros(6),
            platform_pos_mm=np.zeros(3),
            platform_rot=np.zeros(3),
            platform_twist=np.zeros(6),
            time=0.0,
            data_age_s=None,
        )
        # Per-tick working buffers
        self._cmd_delta_buf = np.empty(6)
        self._cmd_motor_rev_buf = np.empty(6)
        # ...

    def get_state(self):
        """hot-loop body — returns pre-allocated state, mutated in place."""
        # Fill self._state fields via np.copyto / direct slice assignment
        np.copyto(self._state.leg_extensions_mm, ...)
        # ...
        self._state.time = ...
        return self._state

    def command(self, leg_extensions_mm, vel_mm_s=None, ...):
        """hot-loop body — pre-allocated buffers, in-place msg build."""
        ...
```

### Caveats

- ``get_state()`` returns the SAME ``PlantState`` object every tick.
  Consumers that retain a reference across ticks will see field-level
  corruption.  The convention is: consumers read fields synchronously
  within one tick, or explicitly copy fields if they need a snapshot.
  Grep for ``# RETAINS PlantState:`` comments to find any exceptions.
- The same applies to any pre-allocated return values from ``command()``
  (though ``command`` currently has no return value) and to the
  ``predicted_poses_view`` / ``predicted_times_view`` properties on
  ``MPCController``, which are already documented as non-copying views.

## Implementing a new ``MpcLoopHook``

Hooks run per-tick inside the hot-loop body.  Contract applies.

```python
# Hook closures pre-allocate buffers in the outer scope.
_twist_buf = np.empty(6)
_accel_buf = np.empty(6)
_extras_ns = SimpleNamespace(fk_iterations=0, ff_torque_max_Nm=0.0)

def _on_pre_command(plant_, mpc_, tc, cmd, cmd_vel, diag):
    """hot-loop body — uses pre-allocated twist/accel buffers."""
    poses = mpc_.predicted_poses_view  # already non-copying
    times = mpc_.predicted_times_view
    if poses is not None:
        dt0 = times[1] - times[0]
        # Fill pre-allocated buffer, do not allocate new arrays
        np.subtract(poses[1], poses[0], out=_twist_buf)
        _twist_buf /= dt0
        # ...
        plant_.set_pose(poses[0], twist_6dof=_twist_buf, accel_6dof=_accel_buf)

def _on_log_extras(plant_):
    """hot-loop body — mutates pre-allocated namespace, returns it."""
    _extras_ns.fk_iterations = getattr(plant_, 'last_fk_iterations', 0)
    _extras_ns.ff_torque_max_Nm = getattr(plant_, 'last_ff_torque_max_Nm', 0.0)
    return _extras_ns
```

### Caveats

- Hooks that wrap existing per-tick allocations (e.g. a debug hook that
  snapshots state for post-hoc analysis) MUST pre-allocate their
  snapshot buffers and mutate in place.  The CI test runs with the
  production hook set; a debug-only hook that allocates will trip the
  contract only when it's enabled, which is still a contract violation
  but a scoped one.
- ``on_target_override`` that replaces ``tc`` SHOULD return the existing
  ``tc`` (unchanged) or a pre-allocated override ``TargetCommand``.  A
  fresh ``TargetCommand(...)`` per tick fails the contract.

## Diagnosis

If the contract test starts failing on a commit:

1. **Look at the top-10 diagnostic output**.  Each entry names
   ``file.py:lineno`` and the growth in bytes / count since the warmup
   snapshot.  The offender is usually at the top of the list.
2. **Check whether the failing site is new in this commit**.  `git log -p
   -- <file>` shows what changed.
3. **Apply the appropriate pattern** from the "Implementation guidance"
   section above.  Most commonly: pre-allocate the buffer in
   ``__init__`` and mutate in place.
4. **If the offender is transitively reached but not in this commit's
   diff**, the new hot-path surface you added is calling into previously
   cold code.  Either inline the cold code's alloc-free equivalent or
   add the cold code to the contract surface and pre-allocate its
   state.
5. **If none of the above work and the allocation is fundamentally
   necessary**, follow the threshold-relaxation procedure in the
   "Threshold values" section.

If the contract test starts failing **on an unmodified commit** (i.e.
you merged main and CI now fails): this is usually a platform drift
(numpy/CasADi minor-version change, Python micro-version change affecting
f-string interning, etc.).  Reproduce locally, capture the diagnostic,
and raise as an issue — relaxations due to platform drift SHOULD come
with upstream ticket references.

## Related

- [REFERENCE_LAYER_CONTRACT.md](REFERENCE_LAYER_CONTRACT.md) — the K1–K6
  reference-feasibility contract.  Orthogonal to this one (that one is
  about *what* references the MPC tracks, this one is about *how* the
  hot loop is implemented), but follows the same structural template.
- [hot_loop_contract.py](hot_loop_contract.py) — the ``THRESHOLD_BYTES``
  constant and enforcement-test timing parameters.
- [tests/sim/test_hot_loop_allocation_contract.py](../tests/sim/test_hot_loop_allocation_contract.py)
  — enforcement test; top-10 diagnostic on failure.
- [logbook/2026-04-23-hot-loop-zero-allocation-contract.md](../logbook/2026-04-23-hot-loop-zero-allocation-contract.md)
  — W1 audit inventory; running log of contract evolution.
- [plans/active/hot-loop-zero-allocation-contract.md](../plans/active/hot-loop-zero-allocation-contract.md)
  — phased implementation plan (W2–W7).
- [logbook/2026-04-18-mpc-overhead-spikes-fallback-bursts.md](../logbook/2026-04-18-mpc-overhead-spikes-fallback-bursts.md)
  — original GC-pause investigation; introduced the ``_GCTracker``
  instrumentation this contract builds on.
