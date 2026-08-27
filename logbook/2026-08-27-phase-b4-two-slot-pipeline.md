---
title: "The two-slot pipeline — the preamble leaves the critical path"
type: feature
date: 2026-08-27
status: resolved
phase: "toss-pipelined-preamble — Phase B4"
related_plan: toss-pipelined-preamble.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/toss_session.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/toss_record.py
  - ros_ws/src/jugglebot_interfaces/action/TossContinuous.action
  - config/hardware_config.yaml
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - tools/probes/cadence_rung_check.py
  - tools/probes/possession_replay.py
  - tests/ros/test_toss_sequencer.py
  - tests/ros/test_toss_session.py
  - tests/ros/test_toss_continuous_node.py
  - tests/ros/test_ball_possession.py
  - tests/ros/test_possession_replay.py
  - tests/ros/test_toss_coordinator.py
  - tests/ros/test_toss_integration.py
  - tests/ros/test_toss_pipeline_properties.py
  - tests/motion/test_cadence_rung_check.py
  - tests/motion/test_toss_record.py
subsystem:
  - ros
tags:
  - feature
  - toss
  - cadence
  - safety
  - pipeline
---

# The two-slot pipeline — the preamble leaves the critical path

## Summary

Phase B4 is the core of the pipelined-preamble plan: cycle `k+1`'s whole preamble
(the STATIC `CHECKING` gates, the skip-only `POSITIONING`, the entire `PREPARE`
ladder) now runs **inside cycle `k`'s flight**, and what is left on the critical
path is exactly one tick — the **COMMIT**, which re-reads the evidence and then,
in that same tick and in that order, publishes the announcement and dispatches
the throw.

It ships **dormant**: `jugglebot_operational.toss_pipeline_enabled` defaults
`false`, and with the flag false the decision stream is identical to the pre-B4
tree over the whole `cadence_rung_check` grid.

What landed:

* **`toss_sequencer`** — `PHASE_STAGED` / `PHASE_COMMITTING`, `_step_staged`,
  `_step_committing`, the `SLIP` decision, `commit_budget_s`, `stage_budget_s`,
  `min_stage_lead_for_release_s`, `commit_at`, a populated `slip_s`, and the
  STATIC/EVIDENCE gate split in `_step_checking`.
* **`toss_session`** — the `pipelined` flag, S1′ (two slot flags:
  `cycle_live` spans START→COMMIT, `committed_live` spans COMMIT→terminal),
  `note_cycle_committed`, `note_stage_abandoned`, and `required_dwell_s`'s
  pipelined branch.
* **`reload_coordinator_node`** — `_tick_toss_pipeline` (committed slot ALWAYS
  first), `_start_pipelined_cycle`, `_promote_toss_staged`,
  `_discard_toss_staged`, `_commit_toss_catch`, `_staged_observations`,
  slot-aware clamp reads, and the drain's discard half.
* **`toss_record`** — four additive fields (`staged_at_s`, `commit_at_s`,
  `commit_slip_s`, `staged_discarded_reason`); no `SCHEMA` bump, per the
  schema's own additive rule.

The floors, from the SHIPPED `required_dwell_s` (§ 2.7's table, reproduced):

| apex | pipelined floor | milestone dwell | clearance |
|---|---|---|---|
| 0.50 | 0.4941 | 0.3075 | −0.1866 ✗ |
| 0.80 | 0.4390 | 0.3890 | −0.0501 ✗ |
| **1.00** | **0.4170** | 0.4349 | **+0.0179** ✓ |
| **1.30** | **0.3941** | 0.4958 | **+0.1018** ✓ |

## Discussion

### One read, two decisions — and why it had to be one read

§ 2.4.4's claim is that at the commit tick a SEATED cup *is* the CAUGHT evidence
for the upstream cycle, so the read that admits the throw and the read that
terminalises cycle `k` are the same read. The obvious implementation — call
`_build_toss_observations` once per slot — would have sampled the cup at two
instants for one decision, which is the split-observation class C-POSSESS-1 § 3.3
edit 1 closed for `ball_evidence`.

So `_tick_toss_pipeline` builds **one** snapshot per tick, from the committed
slot's state, and both slots reason from it. The staged slot gets a derived view
(`_staged_observations`) that zeroes only the per-cycle EVIDENCE belonging to the
committed cycle — its release evidence, its possession verdict and the two
diagnostics derived from them. Everything the commit gate actually reads
(`ball_seated`, `hand_parked`, `track_active`, the four link gates) is the
committed cycle's own instant, unmodified. `_step_toss_sequence` grew an optional
`obs` parameter for exactly this, and builds its own when not given one, so the
serial path is untouched.

The ordering that makes the identity work is the committed-slot-first rule: cycle
`k`'s terminal is resolved, `note_upstream_terminalised()` is called on the staged
slot, and only then is the staged slot stepped — all within one tick, on one
snapshot. That is also what makes S1′ true *within* a tick rather than merely
between ticks, and it is pinned structurally on `_tick_toss_pipeline`'s AST.

### Slip rather than refuse, and the bound that is not a new constant

The cup's seat edge is late by a measured, systematic **+183.9 ms median**. A
commit that refused on a not-yet-seated cup would turn a healthy machine's
ordinary timing into `REJECTED_NO_BALL` — a machine-fault verdict for a cadence
fact, the same mis-routing `REJECTED_HAND_NOT_PARKED` was called out for at R5.
So `hand_parked` and `ball_seated` both SLIP.

The slip moves `_t_release` with `now`, which buys the release bound for free:
`t_release − now` is pinned at the commit budget and can never fall under the
dispatch budget it contains. That is what makes `ABORTED_CANT_MAKE_RELEASE`
structurally unreachable on the pipelined path, which § 6.4 requires (one would be
a *design* finding, not a tuning finding). The upstream bound is
`catch_confirm_window_s` — the instant by which cycle `k` terminalises at the very
latest — so no new constant is introduced, and `test_the_slip_is_bounded_by_the_
confirm_window_and_nothing_else` drives it at three different window values so a
mutation of the constant moves the bound.

`track_active` does **not** slip: unlike the other two it is not a thing that
resolves by waiting.

### What the plan left open, and what I decided

**1. `catch/armed` must not drop between chained cycles — the plan contradicts
itself here, and I had to pick a side.** § 2.3's S6 amendment keeps `catch/armed`
per-cycle on two grounds: it installs no graceful stop, and the bench trace
recorder's `cycle_spans` segments every CS check off its edges. § 2.4.2 then
asserts that the ≥1-tick armed→announce gap is "satisfied by S6, not by a tick"
because the latch was raised seconds earlier.

Both cannot be true. Reading `catch_coordinator_node` settles which: the gap
concerns `catch/armed` (the **topic**) — `_on_throw_announcement` returns early on
`not self._catch_armed` — not the `trajectory/arm_catch` latch that S6 hoists. And
under the pipeline the previous cycle's `_toss_stay` (which publishes
`catch/armed` False) and the next cycle's announcement land **in the same tick**.
Keeping the per-cycle edge would put a False, a True and an announcement into one
`catch_coordinator` wait-set with no cross-topic ordering guarantee: if it drains
the False before the announcement, the announcement is dropped and with it the
`_latch_throw_stroke_window` call that is the C-HAND-1 stroke-busy protection.

**Decision: `_toss_stay` holds `catch/armed` HIGH while a staged slot is live.**
The root cause is that STAY is the *chaining* terminal — it installs no `go_home`,
so a standing latch has no move to land inside, and the hazard the per-cycle edge
was defended for does not exist on that path. The latch comes down at
`_disarm_session` like every other session-scoped resource. The diagnostics cost is
real and is B6's: CS spans collapse across a pipelined chain, and B6 already has to
re-cut CS-4 for pipelined sessions.

**2. `TRACK_ACTIVE` at commit: REJECTED, not ABORTED.** § 2.4.2's pseudocode says
`abort('TRACK_ACTIVE')`; § 2.4.3's staged-failure table says `REJECTED_TRACK_ACTIVE`
with `ACTION_NONE`. I took § 2.4.3, because it is the half that argues: nothing is
armed at the hand and no announcement was published, so an ABORTED (which reads as
a plant fault and drags a SAFE_ABORT ladder behind it) would be a false statement
about the machine.

**3. A refused staged cycle emits `ACTION_NONE`, and this is a hazard fix rather
than tidiness.** `_terminal_action`'s serial test would emit SAFE_ABORT for one:
`_positioned` is True (POSITIONING is skip-only, so the noop declared an arrival
that traversed zero millimetres) and `_prepare_dispatched` is True (the staged
PREPARE is the drift guard and nothing else). But a staged cycle can refuse **while
the upstream ball is still airborne**, and SAFE_ABORT's ladder retracts the hand —
under the incoming ball, with the catch torn down. The narrowing is guarded on
`self.staged and not self._committed`, so the serial path cannot see it.

**4. The retired delay gate needed a replacement, and the probe said so.**
`cadence_rung_check.pipelined_session_accepts` recorded, in writing, that dropping
`REJECTED_THROW_DELAY` for a staged cycle "is a B4 decision this probe does not
make", because *a retired gate that nothing replaces is how the 0.160 s got charged
twice in the first place*. The replacement is `min_stage_lead_for_release_s` =
`stage_budget_s + commit_budget_s`, charged at a staged cycle's CHECKING against
its REAL lead (`_t_release − now`). `REJECTED_THROW_DELAY` itself SURVIVES on both
branches — the first cycle of every pipelined sitting runs serially and its release
really is `accept + throw_delay`.

A pleasing check falls out: `stage_budget_s + commit_budget_s` is **bit-for-bit**
`min_throw_delay_for_release_s(v, False)`. The pipeline charges the same four loop
periods and the same dispatch budget; three of the four simply moved off the DWELL
and onto the previous cycle's FLIGHT. That is § 9.2's "the floors are re-derived,
never relaxed" made literal, and it is now a test.

**5. `note_cycle_committed` is keyed on the DISPATCH, not on the promotion.** My
first cut called it from `_promote_toss_staged`, which is wrong in a way the tests
caught immediately: the pipeline's FIRST cycle runs serially and never promotes, so
it held the staging slot until its terminal and **nothing ever pipelined behind
it** — the flag was on and the machine was serial. One notification, from
`_tick_toss_pipeline`, fired when either slot emits `ACTION_DISPATCH_THROW`.

**6. The per-cycle PREPARE publishes and the D6 latch roll move to the COMMIT
tick.** B1's report flagged the `_announced_ball_id` / `_preexisting_flight_ids`
latching as B4's problem. Making them per-slot was one option; deferring them was
the other, and deferring is strictly better:

* the phantom snapshot's whole job is "any id already IN_FLIGHT before OUR throw is
  a phantom", and it is now taken microseconds before the announcement instead of
  four ticks. Taken at stage time it would have recorded the upstream cycle's
  airborne ball as a pre-existing phantom;
* the D6 roll at stage time would have dropped the id cycle `k`'s release evidence
  and `achieved_flight_s` are latched on;
* `catch/armed` at stage time would be lowered again by cycle `k`'s own terminal
  before this cycle throws.

So `_build_toss_cycle(staged=True)` does two assignments and nothing else, and
`_promote_toss_staged` + `_commit_toss_catch` do the rest at the commit.

**7. `track_active`'s exclusion set widens to both announced-ball latches.** At a
staged cycle's commit the upstream ball has only just landed: its id is still in
`_announced_ball_id` (the roll happens at this very commit) and its track stays
IN_FLIGHT until the tracker mints CAUGHT, +0.202…+0.442 s later. Without this the
gate hard-rejects `REJECTED_TRACK_ACTIVE` on every healthy pipelined cycle, for the
machine's own ball — census D6 exactly, one cycle earlier. It is a **no-op on the
serial path**: `_build_toss_cycle` clears the latch before CHECKING's first tick,
and `track_active` is computed above `_update_announced_ball_latch`.

**8. A cycle that cannot stage is dropped, not degraded.** Its positioning decision
is only knowable after the release state is solved, so the build must run first; a
staged build is side-effect-free apart from the slot install, so dropping it costs
one solve and no state. `note_stage_abandoned` gives the index and the inherited
one-cycle flags back (guards G10/G11 depend on exactly one cycle wearing each) and
holds the slot shut until `note_cycle_result` reschedules, so the cycle is rebuilt
SERIALLY exactly once rather than re-attempted every tick. A persistent fault
therefore ends the session with a named terminal instead of minting a record per
tick.

**9. `TossDecision.action_then`, not a tuple.** The commit tick emits two actions.
A general `actions` container would have invited a second producer; a second named
field says there is exactly one, and the FSM has exactly one — `_step_committing`.

**10. Layer 1.5 goes inert under the pipeline, and that is honest rather than
broken.** The dwell-tilt reader's guard was `not session.cycle_live`, which is
False for the whole flight under the pipeline — precisely the window § 3.10 rule 1
forbids a blocking read in. The guard gains `and not session.committed_live`, so a
pipelined session reads no dwell tilt at all. There is no quiescent dwell to read
one in; the covariate simply does not exist at these cadences.

**11. The commit tick REPORTS `COMMITTING` while the FSM's phase advances to
`THROWING`.** The two are different questions. The FSM's phase must advance —
`_toss_cancel_deferred` reads it and the dispatch is committed from that instant.
The DECISION's phase is what happened *in that tick*, and two consumers need it:
the session feedback (or `COMMITTING` becomes a wire string that only ever appears
on a SLIP), and `LoopPeriodCensus`, which classifies iterations by the reported
phase — reporting `THROWING` would file the pipeline's ONE pre-dispatch tick under
the post-dispatch idle majority, which is the dilution the pre/post split exists to
prevent. `PHASE_COMMITTING` therefore joins `PRE_DISPATCH_PHASES`; `PHASE_STAGED`
deliberately does not (it is a WAIT, the pipelined analogue of flight-waiting).

**12. `STAGED` reaches the wire as an EDGE, `COMMITTING` as a level.** The
committed slot's phase is the default — it is the cycle with a ball in the air, and
the runbook watches `BALL_IN_FLIGHT`/`CATCHING`/`SETTLING` through it. The staged
slot takes the feedback slot on the one-tick edge into `STAGED` ("the next cycle is
prepared and waiting", information the serial path never had), for every
`COMMITTING` tick including the slips, and whenever there is no committed slot to
report. A level would have shadowed the flight.

**13. The reload interlude is excluded while a cycle owns the hand — explicitly,
though it is unreachable.** The interlude MOVES the platform, and S2 admits it only
because `REJECTED_NO_BALL` is minted in CHECKING, from a quiescent machine. Under
the pipeline "no cycle is live" stopped being the same statement as "no cycle owns
the hand", so the guard now names the second one. `_reload_pending` cannot be set
while `committed_live` today; "unreachable today" is not a guard, and an interlude
under an airborne ball would recentre the platform out from under the catch.

### One deliberate deviation from the plan's text

§ 2.4.3's staged-failure table says a cycle that fails a STATIC gate while staging
"re-stages on a later tick if there is still time". I implemented **rebuilt
SERIALLY, once**, via `note_stage_abandoned`. Re-staging every tick mints a toss
record per attempt and never terminates on a persistent fault — a
`REJECTED_NOT_LEVELLED` would loop until the session ceiling. The serial rebuild
retries exactly once, and if the fault is real the same gate mints the same
refusal on the serial path, where `note_cycle_result` stops the session by name
(`ABORTED_CYCLE_REJECTED_NOT_LEVELLED`). A transient hiccup costs one stage attempt
and the cadence absorbs it.

The same routing has a second, useful consequence worth stating: a staged cycle
that slips out on an empty cup (the ball bounced out during the dwell) does **not**
trigger the reload interlude directly — it abandons the stage, the cycle is rebuilt
serially, and its CHECKING mints `REJECTED_NO_BALL` through `note_cycle_result`,
which is where the interlude lives. The session's fate stays decided by a cycle
that actually ran.

### The plan's T-G4 number is wrong, and the correct one is thinner

§ 5.6 predicts that the plumbing term's dominance over `hand_floor_dwell_s`
narrows from 0.2030 s to **0.0947 s** (ratio 2.8× → 1.3×) and calls T-G4 "the one
existing test this plan genuinely stresses". Measured over the whole C-HAND-3 band
at `catch_vel_scale` 0.9 with no ILC trim, the worst-case pipelined margin is
**0.0830 s at T = 0.4949**, i.e. **1.16×**.

The correct value is not a matter of measurement noise — it falls out of the
arithmetic. Both plumbing terms are `dispatch + n × loop + slack + handoff` and
they differ by exactly `3 × NODE_LOOP_PERIOD_S`, so the margin is
`0.2030 − 0.120 = 0.0830` and can be nothing else. The plan's number would require
the gap to be 0.1083 s, which is not three of anything.

1.16× is still a cover rather than a coincidence — the 0.0715 s sensitivity is a
WORST CASE at a maximal negative ILC trim, and the throw envelope refuses almost
the whole negative side at exactly the flights where this margin is narrowest — but
it is thinner than the plan believed, and the re-taken test now asserts the
DIFFERENCE between the two branches (`3 × NODE_LOOP_PERIOD_S`) as well as the
absolute floor, so the next constant edit re-derives instead of needing a
re-measure.

### Five defects found while writing this — two of them mine, and silent

Worth recording because two of them are the shape that stays invisible:

1. **`note_cycle_committed` on the promotion only** (above) — the flag was on, the
   session completed with three catches, and every cycle was serial. Only
   `test_the_pipelined_builder_passes_the_beat_and_the_stage_flag`, which asserts
   the kwarg set per build, saw it.
2. **`note_cycle_result` cleared the wrong slot.** With a serial committed cycle
   and a staged successor, clearing `_cycle_live` on the committed cycle's terminal
   freed a slot that was full — a third cycle could stage behind two. Fixed by
   clearing `_committed_live` when set and `_cycle_live` otherwise, which is a
   no-op on the serial path where `_committed_live` is never set.
3. **The property model passed while doing nothing.** Two model defects (a dispatch
   stub that never set `throw_dispatched`, and a missing `note_announcement`) left
   every cycle dying at a terminal, and all five properties stayed green — because
   a universal over a stream is satisfied by a model that never gets anywhere.
   `test_the_model_actually_reaches_the_states_the_properties_are_about` is the
   non-vacuity guard that now stands in front of them.
4. **I put the loop census on the slot and then fed a different one.** Moving
   `census` onto `TossCycleState` (so a promotion carries it) left `_run_toss_cycle`
   still minting its own local — the loop fed one object and
   `_toss_record_fields` read the other, so **every serial cycle would have
   declared all-null timing fields while the loop was being measured perfectly**.
   Nothing would have gone red: a null is "not measured", which is a legal value,
   so a whole sitting's timing census would have vanished silently. That is the
   failure the census itself exists to prevent one level up. `_run_toss_cycle` now
   reuses the slot's census, `TossCycleState.clear()` nulls it (else a
   REJECTED_BAD_GOAL record would inherit the previous cycle's timings), and
   `test_the_loop_census_on_the_record_is_the_one_the_cycle_actually_fed` pins the
   identity.
5. **And then I gated the census-end on the committed slot's terminal**, which
   skipped exactly the COMMIT tick — the one iteration `commit_budget_s` charges
   its single loop period for — because the commit and the upstream terminal land
   on the same tick BY CONSTRUCTION (§ 2.4.4). The pipeline's whole pre-dispatch
   census would have read as the empty set, and again nothing would have gone red.
   Each slot now closes its own iteration with its own reported phase, and
   `test_a_pipelined_cycle_censuses_its_own_commit_tick` asserts a non-null
   `loop_n_pre ≥ 4` on every staged cycle.

   Both of these are the same shape and it is worth naming: **an instrument that
   silently reports "not measured" cannot fail a test that does not look for the
   measurement.** Every field B4 added to the record now has a test that asserts
   it is populated, not merely that it exists.

### Probes P4 and P5, and what they confirmed

Both `/tmp` one-offs, per `tools/probes/README.md`; their recipes and outputs are
recorded in the test docstrings that consume them
(`tests/ros/test_ball_possession.py`).

* **P4** (`/tmp/probe_arrival_clamp_pipelined.py`) — the § 2.5 table, by CALLING
  `arrival_boundary_t` rather than restating it. Confirms the plan: the clamp is
  LIVE at every milestone rung (period < 1.700 s) and the seat-edge band is
  watched out at every one of them, so **`SENSOR_BAND_CLAMPED` does not become
  reachable at the milestone** — it needs a period under 0.560 s and the nearest
  in scope is 1.338 s.
* **P5** (`/tmp/probe_retention_inverted.py`) — the retention interval against the
  measured seat edge. Reproduces § 2.5c exactly: **−49.0 ms (inverted)** at the
  h = 1.0 commanded dwell answering `RETENTION_UNKNOWN`, +10.7 ms and +11.9 ms at
  the other two answering `RETENTION_CONFIRMED`. Never `REJECTED`, which at these
  dwells would be a positive bounce-out claim on every good cycle.

### Probe reconciliation

`tools/probes/cadence_rung_check.py` modelled `commit_budget_s` through Phase B0
under an explicit obligation that B4 replace it with an import. It now forwards to
`toss_sequencer.commit_budget_s`, and the identity is pinned three ways
(`test_the_probe_imports_the_shipped_commit_budget_rather_than_modelling_it`). Its
`pipelined_terms` also now carries `shipped_floor`, read off a `pipelined=True`
session, pinned equal to the probe's own `floor` column — the forwarded budget
closes the BUDGET, this closes the FLOOR built on it.

## Verification

* Pipelined FSM + session + node, scoped
  (`python -m pytest tests/ros/test_toss_sequencer.py
  tests/ros/test_toss_session.py tests/ros/test_toss_continuous_node.py
  tests/ros/test_toss_coordinator.py tests/ros/test_toss_integration.py -q`,
  run 2026-08-27): **all pass**.
* Hypothesis properties T-P1…T-P5 at nightly depth
  (`python -m pytest tests/ros/test_toss_pipeline_properties.py -q
  --hypothesis-profile=ci-deep`, run 2026-08-27): **9/9 pass in 58.81 s** (the
  commit message's 57.23 s is an earlier run of the same nine).
* T-R1, the recorded sitting through the pipelined verdict path
  (`python -m pytest tests/ros/test_possession_replay.py -q`, run 2026-08-27):
  **11/11 pass** — bag `2026-08-26_14-25-16` still comes out **23 CAUGHT /
  4 MISSED, row for row**, including the blind flag and the arrival edge, while
  the clamps demonstrably moved.
* **T-U13 / T-G1, the flag-false grid, as a literal byte diff against HEAD.**
  The strongest form of the acceptance, and worth recording because "identical
  verdicts" is weaker than "identical stream": a throwaway dumper
  (`/tmp/b4_stream_dump.py`, uncommitted) drove the real `TossSequencer` at the
  real loop period over the whole `(T, delay, aim, ilc)` grid, emitting every
  tick's `(t, phase, action, done, outcome)` plus each grid point's four session
  floors, and hashed the lot. Run 2026-08-27 against a `git worktree` of HEAD
  (`5382aee`) and against the B4 tree:

      HEAD:    grid decision lines: 17349   sha256: c5fbef58d79137cf…aa2ce51a
      B4 tree: grid decision lines: 17349   sha256: c5fbef58d79137cf…aa2ce51a

  Identical. The in-tree standing gates for the same property are
  `test_the_flag_false_decision_stream_is_the_pre_b4_one_over_the_whole_grid`
  and `test_no_serial_decision_moves_when_a_cycle_is_merely_capable_of_staging`.
* The probe's own flag-false run
  (`python tools/probes/cadence_rung_check.py --grid`, run 2026-08-27):
  **0 violations**, published ladder all-fly, pre-audit ladder still reds.
* The pipelined grid and the § 2.7 floor table
  (`python tools/probes/cadence_rung_check.py --pipeline --grid`,
  run 2026-08-27): **0 violations**, floor table reproduces 0.4941 / 0.4390 /
  0.4170 / 0.3941 from the SHIPPED `required_dwell_s`, all six § 6.2 rungs COMMIT.
* Scoped suites (`python -m pytest tests/ros/ tests/motion/ -q -p no:randomly
  -n 4 --dist loadfile`, run 2026-08-27): **4464 passed, 4 skipped in 226.16 s**.
* T-I1, the announcement through the REAL `BallTracker` and `CatchCoordinator`
  with a cycle staged (`python -m pytest tests/ros/test_toss_integration.py -q`,
  run 2026-08-27): **9/9 pass** — a staged cycle publishes nothing until its
  commit, the one announcement it then publishes correlates to exactly ONE
  `destination='jugglebot'` expectation, and the coordinator emits a catch
  command for it.

Full gate, run by the orchestrator after this entry was drafted and before the
commit (`./run_tests.sh --full`, **2026-08-27**): parallel **6547 selected,
rc=0, in 527 s** + serial **9 passed (rc=0) in 45 s**, total 572 s,
**RESULT: PASS** — the (date, command, result) triple § 5.7 requires. The
commit (`e00a974`) carries the same triple.

## Outcome

The two-slot pipeline exists, fully tested and DORMANT: `toss_pipeline_enabled`
defaults false and the flag-false decision stream is sha256-identical to the
pre-B4 tree over the whole grid, so the flag flip is the only thing the bench
will be validating. With the flag true the shipped `required_dwell_s`
reproduces the § 2.7 floors (0.4170 at h = 1.0, 0.3941 at h = 1.3 — both
admitting the milestone), the probe's `--pipeline --grid` reports zero
accept-implies-flies violations, and the recorded sitting's 23/4 census
survives the pipelined clamps row-for-row. What stands between this and a
flown milestone: B5 (landed next, same day), the B6 sitting ladder, and
prerequisite P-4 for aimed-8b staging.

## Open questions / follow-ups

* **The flag is untested on hardware.** It ships false and the first pipelined
  sitting is § 6.2's rung P0 (h = 1.30, dwell 0.76 — a third of a second of
  margin), not a cadence.
* **PIPE-2 will fire on the first pipelined sitting.** The plan's own caveat: on
  the pre-B5 tree `loop_n_over_pre` is non-zero on 48 of 66 successful cycles, so
  that stop condition presumes B5's tick pacing has landed. Flying a pipelined rung
  before then makes PIPE-2 an instant stop — which is the census doing its job.
* **B6 must re-cut the bench trace recorder's CS spans for pipelined sessions.**
  It already had to for CS-4 (the reach-centre declaration under a standing latch);
  holding `catch/armed` high across a chained boundary adds CS-1…CS-5 to that list.
* **`SESSION_NOT_ARMED` is a structurally-unreachable refusal.** A staged cycle
  whose session was never armed refuses rather than raising `arm_catch` mid-flight.
  Cycle 1 is always serial and is what arms the session, so reaching it would be a
  pipeline sequencing defect; it exists so that defect cannot become an
  `arm_catch` raise that C2-stops a move under an airborne ball.
* **Prerequisite P-4 still gates engagement on the shipped tier.** With the aimed
  8b re-command open, no chained cycle takes the census-B1 skip and nothing stages.
  The pipeline is then safely inert — and `test_a_cycle_that_must_move_does_not_
  stage` pins that it is inert rather than degraded.
