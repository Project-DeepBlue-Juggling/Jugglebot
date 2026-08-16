# Reference-Layer Contract (K1–K6)

This document is the **normative specification** of the reference-feasibility
contract every ``TargetSource`` in the MPC pipeline must satisfy.  It exists
so that new sources (future scheduler variants, juggling choreography, GUI
input paths, etc.) can be implemented without reintroducing the failure
modes that the W1–W11 cycle structurally eliminated.

## Background

The MPC consumes a *reference trajectory* — a time-indexed list of
``ReferenceEvent`` objects (see ``controller/target.py``).  The reference is
not a free parameter: it is the MPC's demand signal, and the MPC is only as
well-behaved as the reference permits.

Pre-W1, references were built by each source independently with no shared
feasibility contract.  This produced a family of failures:

- [MPC_OVERSHOOT_SATURATION](../sim/analysis/known_issues.yaml) — the
  Move-5 and 223902 failures, where quintic peak-velocity exceeded actuator
  v_max and IPOPT couldn't reconcile cost vs rate-constraint within the
  CPU budget.
- Silent twist-clamp gaps leading to FK-noise-spikes becoming reference
  start-velocities.
- Quintic rebuilds with boundary conditions in the past (arrival_time
  before t_now).
- Scheduler/catch-coordinator output occasionally infeasible under hard
  arrival deadlines.

The K1–K6 invariants below close that entire class of failures.

## The Invariants

Every ``ReferenceEvent`` list emitted by a ``TargetSource`` must satisfy:

### K1 — Kinematic continuity

The first event MUST be at ``t = t_now`` with ``pose == plant.live_pose``
and ``twist == plant.live_twist`` (after K6 clamp).  The reference may not
step-discontinuity away from the plant's current state.

**Why:** A reference that starts at a point the plant is not at, or with a
velocity the plant is not carrying, immediately creates tracking error at
t=0.  IPOPT's first iteration must then plan trajectories that bridge that
gap within the actuator's rate limits — which on 223902 saturated the
solver.

### K2 — Velocity bound

For every segment in the reference, the *interpolating polynomial's* peak
velocity magnitude on any linear axis MUST satisfy

    max |v_i(t)|  ≤  β · v_max_mmps     for i ∈ {0, 1, 2}, for all t in segment

where ``β`` defaults to **0.85** (see `controller/target.py:_make_feasible_events`).
The 15% margin gives IPOPT tracking headroom — a reference tight at v_max
saturates rate constraints and stalls the solver.

**Why:** This is the deep root-cause invariant for
[MPC_OVERSHOOT_SATURATION](../sim/analysis/known_issues.yaml).  Without K2,
boundary conditions like `(p0, v0=+60, p1=p0-40, v1=0)` produce quintics
with peak velocity 177 mm/s even though both endpoints are feasible — the
MPC then cannot track.

### K3 — Acceleration bound

Peak acceleration magnitude MUST satisfy

    max |a_i(t)|  ≤  β · (v_max_mmps / tau_s)     for i ∈ {0, 1, 2}

This approximates the actuator's bandwidth (first-order lag with time
constant ``tau``).  A reference whose peak accel exceeds the actuator
bandwidth cannot be tracked regardless of velocity feasibility.

**Why:** Short response-time quintics (large pose change, zero-vel
boundaries, tiny T) produce accel profiles with peaks on the order of
`p_delta/T²`.  K3 catches these and drives the stretch.

### K4 — Monotonic time

Event times MUST be strictly increasing.  Minimum inter-event span is
**50 ms**.  Near-duplicate events are merged (the second is dropped) with
a debug-level log.

**Why:** IPOPT's internal node spacing assumes strictly positive segment
durations.  Duplicate-time events are a caller bug; `make_feasible_events`
enforces K4 by merging, not raising, since the user intent is usually
"here are two near-identical waypoints" rather than "I want an impulse."

### K5 — Twist consistency at event boundaries

Two events at the same time MUST have identical twists.  Attempting to
emit two coincident events with different twists is a contract violation
and raises ``ValueError``.

**Why:** A twist discontinuity at an event boundary would mean the
reference is asking for an impulse in velocity — infinite acceleration for
an instant.  No actuator can track that; no MPC can plan for it.

### K6 — Bounded event twist values

Each event's linear twist components (indices 0..2) MUST be clipped to
``±(β · v_max_mmps)`` element-wise.  Angular components (indices 3..5)
are not constrained by this invariant (tighter angular limits belong to
the application layer — jugglebot's angular twists are typically much
smaller than the linear clamp anyway).

**Why:** FK-derived plant twists can spike (encoder glitches, numerical
noise at near-zero motion).  Without K6 those spikes seed the quintic
start-velocity with physically-impossible boundary conditions, which K2
cannot recover from via stretching alone (the boundary itself is the
problem).

## Enforcement: ``make_feasible_events``

All six invariants are enforced in one place:
[controller/target.py:make_feasible_events](target.py).
Every ``TargetSource`` calls this function (directly or via the
``flat_target_to_events`` wrapper) at the boundary where it emits events.

Signature:

```python
events, rejection_reason = make_feasible_events(
    current_pose,       # live plant pose (6,)
    current_twist,      # live plant twist (6,)
    events_proposal,    # caller's desired events (list[ReferenceEvent])
    t_now,              # current absolute time (s)
    v_max_mmps,         # from MPCParams.max_leg_vel_mmps
    tau_s,              # from MPCParams.tau
    no_stretch=False,   # True for catch targets (hard arrival deadline)
    beta=0.85,          # K2/K3 safety margin
    max_stretch_ratio=4.0,
)
```

Returns:
- ``events``: a K1–K6-compliant event list.
- ``rejection_reason``: ``None`` on clean success; a short machine-readable
  string on rejection (``no_stretch=True`` path) or when stretch failed to
  converge within ``max_stretch_ratio``.

## Stretch policy

When K2 or K3 would be violated with the caller's requested ``arrival_time``,
``make_feasible_events`` **stretches** the segment duration until the
interior peak velocity and acceleration fall inside their bounds.  A binary
search finds the smallest stretch ratio that satisfies both bounds.

- Stretches > 25% of requested duration log a ``WARNING`` for operator
  visibility.
- Every stretch publishes a `stretch_warning` TargetFeedback message (when
  a `feedback_pub` is wired) so downstream consumers can re-plan around the
  delay.  See `ZmqTargetSource._maybe_publish_stretch_warning`.

## Catch-path exception (``no_stretch=True``)

Catch targets have hard arrival deadlines — the ball arrives when physics
says.  Silently stretching a catch's arrival_time would make the platform
miss the ball.

When a catch source's target would violate K2/K3:

1. ``make_feasible_events`` returns ``(events_proposal, rejection_reason)``
   — events are **un-stretched** (caller is expected to discard).
2. The source (``ZmqTargetSource``) **keeps** the last feasible cached
   events rather than overwriting them.
3. The source publishes a ``rejected_infeasible`` TargetFeedback so the
   catch coordinator can retarget or abort.
4. Once the cached quintic's terminal time has passed, the source emits a
   single hold event at the quintic's terminal pose — the platform holds
   position rather than entering an undefined state.

This is the policy agreed in the [2026-04-19 Day-2 discussion](../logbook/2026-04-19-bundle-a-quintic-ref-settling-and-live-twist-trap.md):
"continuance of the last feasible move; once that's done, hold position."

## Warm-start invalidation (W5)

When consecutive distinct targets produce a **structurally different**
reference, the MPC's warm-start from the previous solve is unlikely to be
useful.  ``TargetCommand.warm_start_valid`` is a one-tick hint from the
source to the solver:

- ``True`` (default): the MPC is free to shift-and-reuse ``_prev_w``.
- ``False``: the MPC invalidates ``_prev_w`` / ``_timeout_hint`` and
  cold-starts with W6's per-node-IK initial guess.

A source should set ``warm_start_valid=False`` when either:

- ``‖new_ref_horizon_end − prev_ref_horizon_end‖_∞`` > **20 mm** on any
  linear axis, OR
- any linear axis's reference velocity reverses direction (where the prior
  magnitude exceeded **10 mm/s** — noise floor).

The ``is_warm_start_invalidating`` helper in `controller/target.py`
implements this predicate; built-in sources (``StaticTargetSource``,
``WaypointTargetSource``, ``AutoSequenceTargetSource``, ``ZmqTargetSource``)
all use it.  Custom sources SHOULD.

## Implementing a new ``TargetSource``

Template:

```python
class MyTargetSource:
    def __init__(self, ..., v_max_mmps, tau_s, *, clamp_start_twist_mmps=None):
        ...
        self._v_max_mmps = v_max_mmps
        self._tau_s = tau_s
        self._cached_events = None
        self._prev_ref_end_pose = None
        self._prev_ref_end_twist = None

    def update(self, sim_time, state):
        # 1. Determine target from whatever your source is tracking.
        target_pose, target_twist, arrival_time = self._pick_target(sim_time, state)

        # 2. Delegate to the feasibility layer.
        events, reason = flat_target_to_events(
            pose_6dof_from_state(state), state.platform_twist,
            target_pose, sim_time,
            target_twist=target_twist, arrival_time=arrival_time,
            v_max_mmps=self._v_max_mmps, tau_s=self._tau_s,
            clamp_start_twist_mmps=None,  # subsumed by K6
            return_reason=True,
        )
        # ... handle reason (see catch-path section) ...

        # 3. Detect structural shift for W5 warm-start hint.
        warm_start_valid = True
        if self._prev_ref_end_pose is not None:
            if is_warm_start_invalidating(
                self._prev_ref_end_pose, self._prev_ref_end_twist,
                events[-1].pose, events[-1].twist,
            ):
                warm_start_valid = False
        self._prev_ref_end_pose = events[-1].pose.copy()
        self._prev_ref_end_twist = events[-1].twist.copy()

        return TargetCommand(
            target_pose=target_pose,
            target_twist=target_twist,
            arrival_time=arrival_time,
            ref_events=events,
            boost_vel_weights=True,
            warm_start_valid=warm_start_valid,
        )
```

Caveats:

- Never build ``ReferenceEvent`` lists directly without going through
  ``make_feasible_events`` or ``flat_target_to_events``.  If you do, you
  are opting out of K1–K6 and likely reintroducing a failure mode.
- The scheduler (``controller/scheduler.py``) is a special case: it builds
  its own quintics via ``_QuinticSegment`` but each one is verified against
  K2/K3 at construction time via ``_verify_segment_feasibility``.  New
  segment-building code in the scheduler MUST call that method too.
- ``sample_ref_fn`` in ``controller/target.py`` (callback-based sampling)
  is NOT routed through ``make_feasible_events``.  Callers of it are
  responsible for feasibility; it exists for tests and prototype code
  only.

## Diagnosis

> Note: the Tier-1 fallback rewrite (logbook
> [2026-05-20-hold-extrap-positive-feedback-chaotic-motion.md](../logbook/2026-05-20-hold-extrap-positive-feedback-chaotic-motion.md))
> restructured how the fallback chain protects against ref shifts:
> walk-forward and `hold_extrap` were replaced by a single linear
> cmd-stream extrapolation arm with a 500 ms `cold_hold` time bound.
> Step 4 of the playbook below reflects the post-rewrite structure.

If an MPC session saturates again with the
[MPC_OVERSHOOT_SATURATION signature](../sim/analysis/known_issues.yaml):

1. Check ``TargetCommand.ref_events`` for the session's log — is it None?
   If so, the source isn't routing through the feasibility layer.
2. Check ``make_feasible_events`` return values — any ``reason`` other than
   None?  Those are rejections or stretch-failures worth surfacing.
3. Inspect `is_warm_start_invalidating` firing — did W5 correctly flag
   the transition?
4. Inspect `_handle_failure`'s `cascade_too_long` check — did the 500 ms
   staleness escalate to `cold_hold(q_cur)` when it should have?

## Related

- [MPC_OVERSHOOT_SATURATION](../sim/analysis/known_issues.yaml) — primary
  failure class this contract defends against.
- [2026-04-18 Move 5 investigation](../logbook/2026-04-18-move5-overshoot-stall-and-plant-collapse.md)
  — original diagnosis of the pathology.
- [2026-04-19 Bundle A fix](../logbook/2026-04-19-bundle-a-mpc-overshoot-saturation-fix.md)
  — the first ref-source-side fix (subsumed by K6).
- [controller/feasibility.py](feasibility.py) — closed-form
  quintic peak-velocity / peak-acceleration math.
- [tests/sim/test_make_feasible_events.py](../tests/sim/test_make_feasible_events.py)
  — K1–K6 property tests and scenario regression tests.
- [tests/sim/test_mpc_adversarial_sequences.py](../tests/sim/test_mpc_adversarial_sequences.py)
  — end-to-end MuJoCo regression fixture, one test per failure-mode scenario.
  `nightly`-marked since 2026-08-01 (MPC dormancy,
  `plans/parked/refactor-2026-07.md` Phase 3): run by `./run_tests.sh --full`
  (mandatory pre-commit for `controller/` changes) and by the 04:00 nightly, not
  by the default gate. The K1–K6 property tests in `test_make_feasible_events.py`
  above are NOT demoted — the contract's primary enforcement stays per-commit.
