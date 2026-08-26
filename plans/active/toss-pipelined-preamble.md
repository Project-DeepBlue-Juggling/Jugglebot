---
title: Phase B — pipelining the toss preamble out of the critical path
created: 2026-08-26
status: active
owner: Harrison
last_updated: 2026-08-27
related_logbook:
  - 2026-08-26-possession-verdicts-become-sensor-only.md
  - 2026-08-26-toss-loop-period-census.md
  - 2026-08-23-cadence-floor-and-inertia.md
  - 2026-08-24-arrival-band-remeasure.md
  - 2026-08-24-poller-cadence-and-tristate-tx.md
related_config:
  - config/hardware_config.yaml → jugglebot_operational.toss_session_dwell_margin_s
  - config/hardware_config.yaml → jugglebot_operational.toss_tier
  - config/hardware_config.yaml → ball_detection.arrival_lead_s / arrival_window_s / retention_window_s
related_code:
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py::TossSequencer
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py::pre_dispatch_budget_s
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py::min_throw_delay_for_release_s
  - ros_ws/src/jugglebot/jugglebot/toss_session.py::TossSessionSequencer.required_dwell_s
  - ros_ws/src/jugglebot/jugglebot/toss_session.py::TossSessionSequencer.handoff_margin_s
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py::_run_toss_cycle
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py::_execute_toss_continuous
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py::_step_toss_sequence
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py::_set_toss_next_cycle_perf
  - ros_ws/src/jugglebot/jugglebot/ball_possession.py::arrival_boundary_t
---

# Phase B — pipelining the toss preamble out of the critical path

## 1. Context

### 1.1 The goal, in the owner's terms

Reduce a single-ball self-toss session's dwell to **two-balls-in-one-hand
equivalent timing** at throw heights **0.5–1.3 m**. In a two-in-one-hand pattern
each ball's cycle is `T + dwell` and the hand throws twice per ball cycle, so the
hand period is `(T + dwell)/2`. At a dwell ratio of 0.65 of the hand period:

```
    D = 0.65 · (T + D)/2   ⇒   D = 0.325·T / 0.675 = 0.48148·T
```

**The physics table.** `T_ideal = 2·√(2h/g)` at `g = 9.806` (the ballistics-side
constant, `toss_sequencer.GRAVITY_MMS2`); `T_machine` is what
`motion/trajectory/toss_release.flight_time_from_height` returns for the same
apex, which is the number a `throw_height_m` goal actually converts to. The two
agree to 0.1 ms across the band, so the owner's arithmetic and the machine's are
the same arithmetic.

| apex `h` | `T_ideal` | `T_machine` | required dwell (ratio 0.65) | rehearsal period `T + D` | throws/min |
|---|---|---|---|---|---|
| 0.50 m | 0.6387 s | 0.6387 s | **0.3075 s** | 0.9462 s | 63.4 |
| 0.80 m | 0.8079 s | 0.8079 s | 0.3890 s | 1.1969 s | 50.1 |
| 1.00 m | 0.9032 s | 0.9032 s | **0.4349 s** | 1.3381 s | 44.8 |
| 1.30 m | 1.0298 s | 1.0298 s | **0.4958 s** | 1.5257 s | 39.3 |

(Re-derivable: `python - <<'EOF'` importing `toss_release.flight_time_from_height`
— the exact recipe is in § 5.1's probe row. Every number in this document that is
not quoted from a cited artefact comes from that recipe against the tree at
`f997470`.)

**The milestone for THIS plan is the middle two rows: a commanded dwell of
0.43–0.50 s at `h = 1.0–1.3 m`, admitted by the accept gates and flown on
hardware.** `h = 0.5` (dwell 0.31 s) is explicitly **out of scope** — § 7 names
why and names its two successors.

### 1.2 What stands in the way, stated as arithmetic

The session refuses a dwell below

```
    required_dwell_s = max(throw_delay_s + handoff_margin_s,   # the plumbing term
                           hand_floor_dwell_s)                 # the physics term
```

(`toss_session.py:961`), and `throw_delay_s` is itself floored at

```
    min_throw_delay_for_release_s(v) = hand_stroke.min_throw_event_delay_s(v)   # the :642 dispatch budget
                                     + pre_dispatch_budget_s(positioning_move)  # the PREAMBLE
```

(`toss_sequencer.py:975`). On a chained cycle `pre_dispatch_budget_s(False)` is
`(1 + 3) × NODE_LOOP_PERIOD_S = 0.160 s` (`toss_sequencer.py:897`, owner decision
D3 of 2026-08-26). Evaluated at the milestone heights:

| apex | dispatch budget | preamble | ⇒ delay floor | handoff margin | hand floor | **dwell floor today** | target | **short by** |
|---|---|---|---|---|---|---|---|---|
| 0.50 | 0.3040 | 0.1600 | 0.4640 | 0.1501 | 0.3939 | **0.6141** | 0.3075 | 0.3066 |
| 0.80 | 0.2802 | 0.1600 | 0.4402 | 0.1188 | 0.3264 | **0.5590** | 0.3890 | 0.1701 |
| 1.00 | 0.2707 | 0.1600 | 0.4307 | 0.1063 | 0.2994 | **0.5370** | 0.4349 | 0.1021 |
| 1.30 | 0.2608 | 0.1600 | 0.4208 | 0.0933 | 0.2713 | **0.5141** | 0.4958 | 0.0182 |

Two facts fall straight out of that table and they set the whole plan:

1. **The preamble is the deficit.** At `h = 1.3` the shortfall (18.2 ms) is a
   tenth of the preamble; at `h = 1.0` it is two-thirds of it. Deleting the
   0.160 s from the *critical path* — not from the *floor*, which would just
   re-buy `ABORTED_CANT_MAKE_RELEASE` — reaches the milestone at both heights
   with margin.
2. **The hand floor is not the binder anywhere in scope.** `hand_floor_dwell_s`
   is 0.2713–0.2994 s at the milestone heights against a plumbing term of
   0.5141–0.5370 s. The physics has 0.20–0.24 s of headroom that the plumbing is
   currently spending. That is the whole opportunity, and it is why this is a
   software phase and not a firmware one.

### 1.3 Why now, and what already says so

This work is already named as a prerequisite in two places, by other people's
analysis:

* `tests/hardware/session_cadence_ladder.md`, the **R5** card, `must have landed`
  row: *"census **F1** (the miss-cleanup floor re-derived from COMPLETION rather
  than service acks), **F6** (pipelining, OR a demonstrated verdict path with
  ≤ 0.10 s latency), **F7** (invariant S5 re-argued **in writing** — at this dwell
  the 'quiescent wait' is under 0.2 s and the reactive catch path is effectively
  always live). **None of these has landed. R5 is NOT reachable today.**"*
* `plans/active/toss-selftuning.md` § 11.2, **Layer F**: *"session invariant S1,
  `_run_toss_cycle` being a **blocking** call, which is the structural reason the
  verdict handoff is a floor and not just a number — any dwell below ~0.5 s
  requires **pipelining** cycle N+1's CHECKING/PREPARE into cycle N's flight."*

Phase A (`f997470`, `logbook/2026-08-26-possession-verdicts-become-sensor-only.md`)
is the prerequisite for all of it and it has landed: the possession verdict is the
cup sensor's alone, so the machine now has a verdict channel it can schedule
against. That entry's own § Open questions names this plan: *"Phase B–E roadmap
(owner will commission the plan separately): pipelined preamble → beat clock with
bounded slip → displaced pose → ILC unpark."* It also names the lever: *"The
frontier is a LOOP-cost problem now… The candidates are the PREPARE bundle's
blocking service calls and the per-tick observation build."*

### 1.4 What this plan does NOT get, and the number that says so

Removing the preamble from the critical path does not make the machine reach the
commanded dwell *as an achieved period*, because a second quantity binds
underneath it: **the cup's seat edge**.

The possession verdict for cycle `k` is minted at the empty→held edge, measured
at **+87.6 … +554.7 ms past the scheduled landing, median +183.9 ms**
(n = 33 over four post-FW-14 bags, `logbook/2026-08-24-arrival-band-remeasure.md`;
the constants are `ball_possession.ARRIVAL_BAND_MIN_S` = 0.087 and
`ARRIVAL_BAND_MAX_S` = 0.56). Cycle `k+1` may not release a ball until the cup
says it holds one, so the earliest honest release is

```
    release(k+1) ≥ seat_edge(k) + commit_budget_s
```

and `seat_edge` has a **+183.9 ms median bias** that no amount of pipelining
removes. ~~About 52.3 ms of it is the release itself running late~~ **[CORRECTED
by B0/P2, 2026-08-27]**: the ballistic back-cast puts the release at **−1.6 ms**
(on time) — the "+52.3 ms late departure" of Phase A § Summary was the ball
occluding the cup beam while accelerating out, and it belongs to the sensor term.
The measured three-way split (n=25): release **−1.6 ms**, flight-time model
**+102.1 ms**, seating/detection **+85.9 ms** (`tools/probes/seat_edge_decomposition.py`,
2026-08-27). The flight-model term is the correctable half (Q-2); the seating
residual is the hard floor on any evidence-gated commit.

Carried through, the prediction is:

| apex | achieved period **today** | achieved period **after Phase B** (loop 0.040) | after Phase B **+ loop 0.025** | target |
|---|---|---|---|---|
| 0.50 | 1.287 s (46.6/min) | 1.167 s (51.4/min) | 1.152 s (52.1/min) | 0.946 s (63.4/min) |
| 0.80 | 1.432 s (41.9/min) | 1.312 s (45.7/min) | 1.297 s (46.3/min) | 1.197 s (50.1/min) |
| **1.00** | 1.518 s (39.5/min) | **1.398 s (42.9/min)** | 1.383 s (43.4/min) | 1.338 s (44.8/min) |
| **1.30** | 1.635 s (36.7/min) | **1.526 s (39.3/min)** | 1.526 s (39.3/min) | 1.526 s (39.3/min) |

So: **`h = 1.3` reaches the target exactly and with no slip; `h = 1.0` reaches
0.4946 s of achieved dwell against a commanded 0.4349 s, i.e. it lands at the top
of the milestone band rather than the bottom, and its residual is the seat-edge
bias, not the preamble.** Both are inside the milestone's 0.43–0.50 s. That
distinction — commanded floor versus achieved period — is scored separately in
§ 6 and must not be collapsed; collapsing it is how a cadence claim becomes a
cadence the machine cannot make, which is the defect D3 closed one gate over.

---

## 2. Architecture

### 2.1 Current architecture

```
_execute_toss_continuous  (reload_coordinator_node.py:5446)
  │  outer loop, time.sleep(_TICK_S) at the bottom
  │
  ├─ session.step(now) ──► SESSION_ACTION_START_CYCLE
  │     when now >= _next_cycle_at,  _next_cycle_at = landing(k-1) + dwell − throw_delay
  │
  ├─ _build_toss_cycle(...)  ─► one TossSequencer + one TossCycleState [B1]
  │                             (.release_state, .aim, .positioning_move, …),
  │                             installed as self._toss_committed; the
  │                             _announced_ball_id latches stayed node-global
  │
  ├─ _set_toss_next_cycle_perf(seq, session)   ← the C-POSSESS-1 § 3.4 clamp,
  │                                              latched node-global
  │
  └─ _run_toss_cycle(seq, …)   ◄────── **BLOCKS until the cycle terminalises**
        while rclpy.ok():
          now = perf_counter()
          decision = _step_toss_sequence(seq, now)      obs build + step + action
          …
          time.sleep(_TICK_S)                            0.020 s sleep, 0.040 s period

        CHECKING ─► POSITIONING ─► PREPARING ─────────────► THROWING ─► BALL_IN_FLIGHT
        └──────────── 4 loop periods = 0.160 s ───────────┘   └── the flight, ~0.9 s ──►
                     charged to throw_delay_s                      … ─► SETTLING ─► terminal
```

Every cycle's preamble is serial with the previous cycle's verdict, because
`_run_toss_cycle` is a blocking call and session invariant **S1** forbids a second
live `TossSequencer`. The 0.160 s is therefore spent *after* the ball is in the
cup and *before* the next one leaves it.

### 2.2 Proposed architecture — a two-slot cycle pipeline

```
_execute_toss_continuous  (non-blocking; one loop, two slots)
  │
  │   ┌──────────────── committed slot (owns the hand + the airborne ball) ─────────────┐
  │   │  COMMITTING ─► THROWING ─► BALL_IN_FLIGHT ─► CATCHING ─► SETTLING ─► terminal   │
  │   └────────────────────────────────────────────────────────────────────────────────┘
  │            │                                                              │
  │            │  commit(k)                                 seat edge / verdict(k)
  │   ┌────────┴──────────── staged slot (owns nothing that moves) ───────────┴────────┐
  │   │            CHECKING(static) ─► POSITIONING(skip) ─► PREPARING ─► STAGED        │
  │   └────────────────────────────────────────────────────────────────────────────────┘
  │                                    ▲                                        │
  │                                    │ staged during the PREVIOUS cycle's     │ promotes to
  │                                    │ flight/dwell — off the critical path   │ committed at
  │                                    │                                        │ commit(k+1)
  │
  └─ per iteration: step the COMMITTED slot first, then the STAGED slot.
     `_run_toss_cycle` survives unchanged for the single `Toss` action (one slot).

  commit(k+1) = release_at(k+1) − commit_budget_s
  commit_budget_s = hand_stroke.min_throw_event_delay_s(v) + 1 × NODE_LOOP_PERIOD_S
```

**What moves:** `CHECKING`'s *static* gates, `POSITIONING`, and the whole
`PREPARING` ladder (prime_hold, reach-centre declaration, pretilt_hold, the
PREPARE bundle) run in the previous cycle's flight and dwell.

**What stays on the critical path:** exactly one tick — the **COMMIT** — which
re-reads the evidence, publishes the announcement, and dispatches the throw.

**What does not change at all:** every action string and every action handler
(`_position_platform_for_toss`, `_prepare_toss_catch`, `_announce_toss`,
`_dispatch_toss_throw`, `_publish_toss_reach`, `_toss_stay`, `_toss_recenter`,
`_toss_safe_abort`); the single-shot never-retried dispatch and its tri-state
classification; the hand ladders and `_MAX_ARM_DISPATCHES`; the Teensy-side
`MAX_DEVIATION` guard, which remains the leg-path safety authority and is not on
any path this plan touches; every commanded motion staying a profiled move
through `trajectory_node`/`planner`/`feasibility.validate`.

### 2.3 The invariants — one amended, three new

**S1′ replaces S1, and it preserves S1's hazard rather than its wording.** S1 said
"at most ONE cycle is live", justified by *"Two live `TossSequencer`s would
double-own the hand on the Teensy's last-writer-wins queue and fight over the
single `catch/armed` latch"* (`toss_session.py:30`). Both halves of that hazard are
about **ownership of a shared actuator**, not about the number of FSM objects. So:

> **S1′** — at most one cycle may be **past its COMMIT point** at any instant, and
> only that cycle may emit a hand-bearing action. A staged cycle's decision set is
> restricted to `{ACTION_NONE, ACTION_POSITION_PLATFORM, ACTION_PREPARE_CATCH}`,
> and `ACTION_POSITION_PLATFORM` is admissible only as the census-B1 **no-op skip**
> (§ 2.4.1). A structural test pins the emittable action set per stage.

> **S6 (new)** — the catch latch and the catch-coordinator holds are
> **session-scoped**, with exactly one raise and one lower per contiguous run of
> chained cycles. No cycle raises or lowers `trajectory/arm_catch`,
> `catch/prime_hold` or `catch/pretilt_hold`. **[AMENDED 2026-08-27]** The
> `catch/armed` topic publish **stays per-cycle** (as § 4 B3's scope always said —
> the original wording here listed it and contradicted B3): it installs no
> graceful stop (the arm-mid-move hazard lives solely in `trajectory_node`'s
> `_svc_arm_catch` raise path), and the bench trace recorder's `cycle_spans`
> segments every CS check off its edges — session-scoping it would collapse
> CS-1…CS-5 to one span per sitting.

> **S7 (new)** — **the pipeline is DRAINED before any `go_home`.** Every path that
> dispatches `trajectory/go_home` (SAFE_ABORT, RECENTER, the reload interlude, the
> session terminal) first discards the staged slot and lowers the latch, in that
> order.

S6 and S7 together close the **arm-mid-move seam by construction rather than by
timing**, which is the property the current build buys with a floor and does not
get. Today `trajectory_node` prints *"catch latch armed mid-move — installed a
graceful stop (move silenced)"* whenever the next cycle's PREPARE arms the latch
while the previous cycle's SAFE_ABORT `go_home` is still traversing; it fired on
**10 of the 16 post-MISS toss cycles** of bag `2026-08-26_14-25-16`
(`toss_session.py:499`), and the remedy shipped was to lengthen
`DEFAULT_SESSION_MISS_CLEANUP_S` to 2.80 s so the arm lands after the profile.
That is a timing fence over a race. With S6 there is no re-raise to race, and with
S7 there is no staged cycle alive when the profile is installed. The cleanup floor
stays (it protects other things — the retract's descent, the throw site), but it
stops being the only thing between an interrupted `go_home` and a throw from a
site the aim was not solved for.

> **S5′ — the "quiescent wait" is re-argued, in writing** (this is census F7, and
> it is a prerequisite the runbook already names).
>
> S5 chose "dwell as a quiescent wait" over "dwell as a stretched `throw_delay`"
> for three named failure modes (`toss_session.py:71`). Under the pipeline the
> dwell is no longer quiescent, so each is re-taken on its merits:
>
> 1. *"`catch/armed` stays RAISED for the whole dwell with a ball resting in the
>    cup, so `catch_coordinator`'s reactive catch path is live for that entire
>    window."* **This is now accepted deliberately, and it is the one real cost of
>    S6.** At the milestone dwells the armed window is already ~97 % of wall time
>    (dwell 0.435 s inside a 1.34 s period, against a catch that is armed from
>    PREPARE to terminal), so S6 converts a 97 % duty cycle into 100 % rather than
>    creating a new state. The mitigations are already shipped and unconditional:
>    `catch/pretilt_hold` is raised for the whole goal (census E5,
>    `reload_coordinator_node.py:3970`) so no announcement pre-tilt can command
>    motion, `catch/prime_hold` suppresses the armed-edge auto-prime, and contract
>    C-REACH-1 centres the reach envelope on the nominated catch B so any
>    commanded reach is bounded at 80 mm. What is genuinely new is that a foreign
>    tracked ball entering the volume between cycles now meets an armed machine
>    where before it met one for 97 % of the interval. Accepted; the runbook keeps
>    the by-eye watch (§ 6, row PIPE-5).
> 2. *"An armed dwell looks identical to an about-to-throw machine, so the
>    operator's intervention window is one in which the robot is armed."*
>    **Already true at these cadences and already documented**:
>    `session_cadence_ladder.md` § 5 records that from R5 down *"a cancel is
>    always deferred… your stop button gains one full cycle of latency"*
>    (`TOSS_CANCEL_CUTOFF_S` = 0.25 s). The pipeline adds at most one further
>    cycle of latency, because a deferred cancel must also drain the staged slot.
>    The runbook says so explicitly and repeats that the cancel button is not the
>    E-STOP.
> 3. *"A stretched delay moves every cycle's internal timing off the profile the
>    hardware measured."* **Unaffected, and this is the load-bearing half.** From
>    COMMIT onward the cycle is byte-identical to a validated single toss: same
>    guard, same announcement, same single-shot dispatch, same flight, same settle.
>    Only the preamble moved, and the preamble commands nothing on a chained
>    cycle.

### 2.4 The dispatch/arm choreography — the state machine

#### 2.4.1 Stage boundaries, and what may run where

`CHECKING` splits along a line that already exists in the code but has never been
named: whether a gate is a function of the **previous cycle's outcome**.

| gate | source | stage | why |
|---|---|---|---|
| tier, `throw_delay` floor, flight-time validity, event-vel band, throw envelope, workspace, displacement, tilt clamp | goal + config, pure | **STATIC** (staged) | cycle-invariant in a steady session; no observation |
| `control_mode`, `streaming`, `mocap_fresh`, `platform_levelled`, `hand_fresh` | live but slowly-varying | **STATIC** (staged) **and re-read at COMMIT** | cheap to re-read; a hiccup between stage and commit must refuse |
| `hand_parked` | the previous catch stroke | **EVIDENCE** (commit only) | the hand is inside the park band from `catch_park_reentry_s` past the landing; it is false for part of the staging window by construction |
| `ball_seated` / `ball_evidence` | the live cup read | **EVIDENCE** (commit only) | the cup is EMPTY for the whole flight; this is the hard gate |
| `track_active` | the tracker | **EVIDENCE** (commit only), with the previous cycle's own id excluded | the machine's own airborne ball is a live track destined for it during the whole staging window |
| `positioning_move_expected` | live commanded pose | **STATIC**, and it **gates staging itself** | see below |

**A cycle stages only if its positioning decision is SKIP.** The staged slot may
not command a `go_to_pose`: it would move the platform during the previous cycle's
flight, under a ball the catch is armed for. So `_toss_already_positioned` is
evaluated at stage time and a `False` answer means **the cycle does not stage** —
it falls back to the serial path, pays the 0.520 s moving budget, and the session
absorbs it exactly as it does today (`_build_toss_cycle(delay_is_cadence=True)`
grants the lead with one WARN line). The first cycle of every sitting therefore
runs serially, which is correct: there is nothing to pipeline it behind.

> ⚠ **This makes `session_cadence_ladder.md`'s carried finding 2 a hard
> prerequisite for the pipeline to engage at all on the shipped tier.** That
> finding records that on `JB_OP_TOSS_TIER = '8b'` with an aim armed, the deferred
> A→B reach re-commands the platform orientation at `t_release`, so
> `_toss_already_positioned` fails its 2.71 mrad test on the next cycle and every
> chained cycle re-commands the move. With that unfixed, a shipped-tier aimed
> session never stages and the pipeline is inert — safely inert, but inert.
> § 8 carries it as prerequisite **P-4**.

#### 2.4.2 The COMMIT gate — the arm point, defined precisely

> **The arm point is the single tick at which the FSM evaluates the evidence set
> and, in the same tick and in this order, publishes the announcement and
> dispatches the throw. No dispatch is ever issued on evidence read at an earlier
> tick.**

Pseudocode of `_step_committing(now, obs)`, which replaces the guard half of
`_step_preparing`:

```
if now < commit_at:                       return NONE            # wait
if not upstream_terminalised:             return SLIP            # § 2.4.3
if not (obs.streaming and obs.mocap_fresh and obs.platform_levelled
        and obs.hand_fresh):              return abort('MODE'|'…')
if not obs.hand_parked:                   return SLIP            # the hand is still landing
if not obs.ball_seated:                   return SLIP or MISSED  # § 2.4.3
if obs.track_active:                      return abort('TRACK_ACTIVE')
if self._t_release - now < self.min_event_delay_for_throw_s:
                                          return abort('CANT_MAKE_RELEASE')
ANNOUNCE ; DISPATCH_THROW                                        # one tick, in order
```

Three properties of that ordering are load-bearing:

* **`ball_seated` remains a hard gate on release, evaluated at the last tick
  before the CAN frame exists.** The evidence-to-dispatch distance shrinks from
  the current ~0.16–0.52 s (CHECKING to DISPATCH) to zero ticks. The pipeline
  therefore makes the empty-stroke gate *stricter*, not weaker.
* **The announcement follows the slip, not the schedule.** `ThrowAnnouncement`
  carries `throw_time` and `landing_time`; a slipped release invalidates a
  previously-published announcement, and there is no withdrawal message on the
  wire. Publishing at commit is what keeps the announcement true.
* **The ≥1-tick armed→announce gap is satisfied by S6, not by a tick.** The gap
  exists because `catch_coordinator` drops announcement pre-tilts that arrive
  unarmed (`toss_sequencer.py:82`). Under S6 the latch was raised at session
  start, seconds earlier, so the gap is satisfied by construction and the tick it
  used to cost is returned to the dwell floor. This is where one of the four
  preamble loop periods actually goes.

**Rejected alternative — commit early, veto late.** A pre-release SAFE_ABORT's
kind-3 retract does replace an armed kind-0 stroke on the last-writer-wins queue
(`toss_sequencer.py:145`), so a design could dispatch optimistically at
`release − dispatch_budget` and veto at `release − windup` if the cup never
filled. It buys ~0.19 s of dwell floor at `h = 1.0` and it is refused: it converts
the empty-stroke gate from a precondition into a **race against a service round
trip on the queue the 2026-07-25 clobber defect lives on**, and the failure mode
of a lost veto is a full-speed throw of nothing with the hand ascending from an
unverified position. A safety gate that is a race is not a gate.

#### 2.4.3 The unwind — a false start on `k+1` when `k` fails

The staged slot has commanded **nothing that moves** (S1′ + the skip-only rule),
so discarding it is a pure state drop: no publish, no service call, no retraction
needed. The full transition table at the instant the committed cycle resolves:

| committed cycle `k` resolves as | staged `k+1` | latch (S6) | go_home | session |
|---|---|---|---|---|
| `CAUGHT` (cup edge inside the window) | **promotes**; its commit gate runs | held | none (`ACTION_STAY`) | continues |
| `MISSED` / `MISSED_SENSOR_BLIND` / `MISSED_INFEASIBLE_*` | **discarded** before the ladder | lowered by the ladder | SAFE_ABORT's, after the drain (S7) | `stop_on_miss` adjudicates |
| any `ABORTED_*` | **discarded** before the ladder | lowered | SAFE_ABORT's, after the drain | stops |
| `REJECTED_NO_BALL` (reload trigger) | **discarded** | lowered | interlude's recentre, after the drain | interlude, then a cold restart of the pipeline |
| node-level exit (cancel honoured / timeout / shutdown) | **discarded** | lowered | `_safe_toss_on_early_exit`, after the drain | terminalises |

And the two failures that belong to the staged cycle itself:

| staged `k+1` fails at | outcome | machine state |
|---|---|---|
| a STATIC gate, while staging | `REJECTED_<code>`, `ACTION_NONE` | nothing armed, nothing moved; the slot simply never fills, and the cycle re-stages on a later tick if there is still time |
| the COMMIT evidence gate, past the slip bound | `REJECTED_NO_BALL` / `REJECTED_HAND_NOT_PARKED` / `REJECTED_TRACK_ACTIVE`, `ACTION_NONE` | nothing armed at the hand (the dispatch never ran) and **no announcement was published** — so, unlike the two `ABORTED_CANT_MAKE_RELEASE` cycles of bag `2026-08-26_14-25-16`, no phantom tracker expectation is left behind |

**The slip, and its bound.** `SLIP` re-arms the commit for the next loop
iteration and moves `_t_release` with it, so the released ball's own schedule
stays self-consistent (the announcement has not gone out yet). Two bounds, both
**derived rather than chosen**:

* **the upstream bound** — a commit may not run until cycle `k` has terminalised
  (S1′). Cycle `k` terminalises at the cup edge, or at
  `landing(k) + CATCH_CONFIRM_WINDOW_S` (= `ARRIVAL_BAND_MAX_S` = 0.560 s) at the
  latest, so the slip is bounded by that constant and no new one is introduced;
* **the release bound** — the slipped release must still satisfy the runtime
  guard `t_release − now ≥ min_event_delay_for_throw_s`. It does by construction:
  the slip moves `t_release` and `now` together.

A slip past cycle `k`'s settle deadline means `k` is MISSED, which routes to the
discard row above. Expected slip at the milestone: `max(0, seat_edge −
(dwell − commit_budget))` — **0 ms at `h = 1.3`, ~60 ms median at `h = 1.0`**
(§ 1.4). `commit_slip_s` is recorded per cycle (§ 4, B4) so the operator scores
it rather than inferring it.

#### 2.4.4 Where the possession verdict comes from at commit

Since owner decision D1 the cup is the **sole** possession source, which yields a
simplification worth stating explicitly: **a SEATED cup at the commit tick is
itself the CAUGHT evidence for cycle `k`.** The commit gate therefore does not
need a separate "has `k` terminalised" query in the common case — the same
`_possession_observed(now)` read that admits the throw is the read that
terminalises the upstream cycle, on the same tick, from one instant. Deriving them
from two `observe` calls would read the cup at two instants, which is the
split-observation class C-POSSESS-1 § 3.3 edit 1 closed
(`reload_coordinator_node.py:1885`).

### 2.5 Window arithmetic at short periods — a first-class section

Every window in C-POSSESS-1 § 3.2 is clamped by the machine's own next scheduled
event (§ 3.4, clause **C-POSSESS-1.C**). At the milestone periods the clamp is
**live** — it, and not the configured 1.5 s window, is what closes the arrival
search. The three thresholds are distinct and are routinely conflated:

| threshold | value | what happens below it |
|---|---|---|
| the fixed window stops binding | period **1.700 s** = `arrival_window_s (1.5) + arrival_lead_s (0.2)` | `arrival_boundary_t` becomes the closing edge; the clamp is LIVE |
| the superseded `b − lead` rule began amputating | period 0.760 s = `ARRIVAL_BAND_MAX_S + arrival_lead_s` | (historical; that rule is superseded by C.1 — do not restore the subtraction) |
| the shipped rule amputates ⇒ `SENSOR_BAND_CLAMPED` | period **0.560 s** = `ARRIVAL_BAND_MAX_S` | the band cannot be watched out; the verdict must answer UNKNOWN, and `SENSOR_BAND_CLAMPED` is in `ball_possession.BLIND_REASONS`, so both FSMs mint `MISSED_SENSOR_BLIND` |

Evaluated by calling the shipped `ball_possession.arrival_boundary_t` (never by
restating its formula — that is C.1's own instruction, and the 2026-08-24 audit
caught a table computed from the superseded rule):

| rung | period | arrival window closes | band ceiling | band watched out? | `SENSOR_BAND_CLAMPED`? |
|---|---|---|---|---|---|
| R5 as published | 1.2629 s | +1.0629 | +0.560 | yes | no |
| `h = 1.0`, commanded 0.4349 | 1.3381 s | +1.1381 | +0.560 | yes | no |
| `h = 1.0`, achieved ~0.4946 | 1.3978 s | +1.1980 | +0.560 | yes | no |
| `h = 1.3`, commanded 0.4958 | 1.5257 s | +1.3257 | +0.560 | yes | no |

**Finding, and it corrects a premise this plan was commissioned on:
`SENSOR_BAND_CLAMPED` does NOT become reachable at the milestone.** The clamp
becomes the binding edge below a 1.700 s period — which is every rung in scope —
but amputation, and with it the blind-bucket refusal, needs a period under
0.560 s. The nearest in-scope period is 1.338 s, a factor of 2.4 away. The
`SENSOR_BAND_CLAMPED` path stays a blind-bucket refusal that this milestone
cannot reach; it becomes live only at the deferred R6 firmware fork. What the
milestone *does* make live is the clamp itself, and that carries three real
consequences:

**(a) The clamp stops being a prediction and becomes the staged schedule.** Today
`_set_toss_next_cycle_perf` computes the next release as
`landing + session.dwell_time_s` — a prediction made a cycle early, and C.1's own
text names that as the weakness of the superseded rule (*"the clamp used the
SCHEDULED next landing while the next window opened at the ACTUAL one, so a
release that ran late pulled the two ends apart"*). Under the pipeline the staged
cycle **exists** when the committed cycle's windows are evaluated, so the clamp is
fed `slot_staged.t_release` and `slot_staged.landing_perf` — the actual numbers,
slip included. This is a strict improvement and it is one of the reasons to
pipeline rather than merely to shorten.

**(b) The abutment must survive two live cycles, and it survives by identity.**
`_expected_landing_perf()` currently reads `self._active_seq` — with two slots
that is ambiguous, and getting it wrong evaluates cycle `k`'s arrival verdict
against cycle `k+1`'s landing. The rule is explicit and testable: the sensor is
**always** told the **committed** slot's landing; `prev_landing_t` is the
previously-committed slot's landing; `next_landing_t` is the **staged** slot's.
Because `arrival_boundary_t(P, L)` closing `L`'s window and opening `N`'s is
*literally the same call on the same pair*, the abutment holds by identity as long
as those three reads name the right slots. A structural test pins each read to its
slot.

**(c) Retention is not merely dark — it CLOSES BEFORE THE MEDIAN SEAT EDGE, and
that must be said in the runbook.** Retention closes at
`next_release_t − RELEASE_GUARD_S` (= 0.30 s):

| case | retention closes | median seat edge | observable interval |
|---|---|---|---|
| `h = 1.0`, commanded dwell 0.4349 | +0.1349 | +0.1839 | **−49.0 ms (inverted)** |
| `h = 1.0`, achieved dwell 0.4946 | +0.1947 | +0.1839 | +10.8 ms |
| `h = 1.3`, dwell 0.4958 | +0.1958 | +0.1839 | +11.9 ms |

against a debounced fall lag of ~241 ms. C-POSSESS-1.C already governs the
inverted case — *"Where a clamp leaves no interval at all, the part it governs is
`UNKNOWN` — never `CONFIRMED`… and never `REJECTED`"* — so the behaviour is
specified and safe, but it is now the **normal** case rather than an edge one.
Consequences, all of which are already the documented posture at R5′ and are
inherited rather than invented: possession is ARRIVAL-only; a CAUGHT whose bounce
test never ran carries `confidence 0.5` and `retention NOT OBSERVABLE` in the
corpus; the actuation half stays closed by the next cycle's live raw-bit evidence
read (C-POSSESS-1 § 3.5) — which under the pipeline is the COMMIT gate itself, the
strictest placement it has ever had; and the by-eye cup watch becomes *more*
load-bearing (§ 6, row PIPE-6). **T-U9 pre-registers the check that an inverted
retention interval answers UNKNOWN and not REJECTED at these exact numbers**, because
a `REJECTED` there would be a positive claim of a bounce-out on every good cycle.

### 2.6 The Phase C seam — absolute-time releases with bounded slip

Phase C replaces "next release = previous landing + dwell" with a **beat clock**:
a free-running metronome that issues absolute release instants and tolerates
bounded slip. Phase B must make that a change of *who computes a number*, not a
change of the FSM. The seam is one constructor field:

```python
TossSequencer(
    ...,
    release_at_perf: float = 0.0,   # ABSOLUTE scheduled release on the perf clock.
                                    # 0.0 ⇒ derive as start(now) + throw_delay_s,
                                    # which reproduces today's behaviour bit-for-bit.
)
```

with three rules that Phase C relies on and Phase B must land:

1. **`_t_release` is an input, never a re-derivation.** `start(now)` sets
   `_t_release = release_at_perf or (now + throw_delay_s)`. Nothing downstream
   recomputes it; `_landing_perf()`, the runtime guard, the cancel cutoff, the
   deferred 8b reach and the announcement all already read `_t_release` and keep
   doing so.
2. **The commit instant is derived from the release, not the reverse.**
   `commit_at = _t_release − commit_budget_s`. A scheduler that moves the release
   moves the commit for free.
3. **Slip is reported, not hidden.** The FSM exposes `slip_s` (commit-time minus
   scheduled commit) and the record carries `commit_slip_s`. Phase C's "bounded
   slip" policy is then a *consumer* of a measured quantity rather than a new
   mechanism, and the bound it enforces can be tuned against a corpus that already
   exists.

`toss_session.TossSessionSequencer` gains one property,
`next_release_at(landing_perf)`, that returns `landing + dwell_time_s`. Phase C
replaces exactly that method body. Nothing else in either FSM knows where the
beat came from.

### 2.7 The floor re-derivation — one function, both gates

`pre_dispatch_budget_s` does not change; the serial path still charges it, and the
single `Toss` action still runs serially. What is added is its pipelined sibling,
in the same module, next to it, so an edit to either sees the other:

```python
def commit_budget_s(event_vel_mps, min_event_delay_s=0.0,
                    loop_period_s=NODE_LOOP_PERIOD_S) -> float:
    """Cycle COMMIT -> release, in seconds. The pipelined sibling of
    `pre_dispatch_budget_s` + the dispatch budget.

    ONE loop period, not four: under the pipeline the announce tick, the PREPARE
    tick and the deferred-bundle tick have already run inside the PREVIOUS
    cycle's flight, and the armed->announce gap is satisfied by the
    session-scoped latch (S6) rather than by a tick.  The one period charged is
    the commit tick itself, which is polled: the iteration that crosses
    `commit_at` may be up to one loop period late.
    """
    dispatch_s = (min_event_delay_s if min_event_delay_s > 0.0
                  else hand_stroke.min_throw_event_delay_s(event_vel_mps))
    return dispatch_s + float(loop_period_s) + FLOOR_REPRESENTATION_SLACK_S
```

and `toss_session.required_dwell_s` becomes

```python
    max(commit_budget_s(v) + handoff_margin_s,   # pipelined
        hand_floor_dwell_s)
```

when the session is pipelined, and keeps `throw_delay_s + handoff_margin_s`
otherwise. Both branches route through **one derivation each**, which is the
property the 2026-08-22 audit was written after (the session's mirror and the
cycle's gate had drifted). Resulting floors:

| apex | dispatch | +1 loop | handoff | hand floor | **pipelined dwell floor** | milestone | clearance |
|---|---|---|---|---|---|---|---|
| 0.50 | 0.3040 | 0.3440 | 0.1501 | 0.3939 | 0.4941 | 0.3075 | **−0.1866** ✗ |
| 0.80 | 0.2802 | 0.3202 | 0.1188 | 0.3264 | 0.4390 | 0.3890 | **−0.0500** ✗ |
| **1.00** | 0.2707 | 0.3107 | 0.1063 | 0.2994 | **0.4170** | 0.4349 | **+0.0179** ✓ |
| **1.30** | 0.2608 | 0.3008 | 0.0933 | 0.2713 | **0.3941** | 0.4958 | **+0.1017** ✓ |

`h = 1.0` clears by **17.9 ms**, which is a real clearance but a thin one — the
same order as R5's published 1.9 ms dwell clearance, which the runbook calls a
razor edge. **That is what workstream B5 is for**: at a trimmed
`NODE_LOOP_PERIOD_S` of 0.025 s the `h = 1.0` floor falls to 0.4020 s and the
clearance triples to 32.9 ms. B5 is subordinate to B4 in sequence, and — at the
measured 0.070 s loop (B0/P1) — load-bearing for the `h = 1.0` half of the
milestone, not only for the margin (see § 3's B5 row).

---

## 3. Implementation Phase Summary

| Phase | Scope | Status | Date | Risk | Validates |
|-------|-------|--------|------|------|-----------|
| **B0** | Measure first: the seat-edge decomposition probe, the first census read, `cadence_rung_check --pipeline` model. No production code. | **COMPLETE** | 2026-08-27 | none | that the § 1.4 predictions are measurements, not models |
| **B1** | Extract per-cycle node state into `TossCycleState`; every reader takes it explicitly. Pure refactor, zero behaviour change. | **COMPLETE** | 2026-08-27 | low | that two cycles can coexist without sharing state |
| **B2** | `release_at_perf` as a `TossSequencer` input; `TossSessionSequencer.next_release_at`. Bit-identical default. **The Phase C seam.** | NOT STARTED | | low | that the schedule is an input |
| **B3** | Session-scoped arming (S6) + drain-before-`go_home` (S7). PREPARE bundle shrinks to the per-cycle remainder. | NOT STARTED | | **medium — changes the armed window** | the arm-mid-move seam closed by construction |
| **B4** | The two-slot pipeline: `STAGED`/`COMMITTING`, the commit gate, the slip, the unwind, the clamp re-homing, `commit_budget_s`, the re-derived `required_dwell_s`. Ships behind `toss_pipeline_enabled`, default **false**. | NOT STARTED | | **high — the core** | the milestone floors |
| **B5** | Loop-cost trim: absolute-schedule tick pacing, incremental observation build, blocking calls off the tick; then a reviewed re-cut of `NODE_LOOP_PERIOD_S`. | NOT STARTED | | medium | **[re-scoped 2026-08-27, B0/P1] the h=1.0 half of the milestone, and the margin.** The census read found `NODE_LOOP_PERIOD_S = 0.040` is not a bound (chained p50 0.0447, max 0.0626; honest ceil-to-10 ms bound **0.070**), and at 0.070 the h=1.0 pipelined floor is 0.4470 — **12.1 ms short of the 0.4349 milestone** — while h=1.3 still clears by +71.8 ms. B5 must land (and a post-B3/B4 census must be read) before the h=1.0 rungs P4–P5 are flown; the h=1.3 rungs P0–P3 do not wait for it. The dominant term is `body` (blocking calls, argmax 40/73 cycles), not the observation build (~6 %); the sleep overshoot is ~1.5 ms. |
| **B6** | Hardware validation ladder (§ 6), close-out, runbook + contract updates. | NOT STARTED | | **hardware** | the milestone, on the machine |

Phases are strictly incremental: B1 and B2 are behaviour-preserving and land
independently; B3 is a behaviour change that is valuable on its own (it closes the
arm-mid-move seam whether or not B4 ever lands); B4 ships dormant so the flag flip
is the only thing the bench is validating; B5 is a pure cost reduction whose only
gate-visible effect is a reviewed constant.

---

## 4. Implementation Phases (detailed)

### Phase B0: Measure before designing against a number — COMPLETE 2026-08-27

**New files**
* `tools/probes/seat_edge_decomposition.py` — committed, outputs to
  `temp/probes/` per `tools/probes/README.md`.
* `tools/probes/toss_loop_census.py` — committed, the P1 reader for the shipped
  `LoopPeriodCensus` fields in `temp/logs/toss_records_*.jsonl`.

**Modified files**
* `tools/probes/cadence_rung_check.py` — add `commit_budget_s` modelling and a
  `--pipeline` column; add `PIPELINED_LADDER`.
* `tests/motion/test_cadence_rung_check.py` — extend for the new column.

**Scope.** Three measurements, all offline against the existing corpus, none
requiring a sitting:

1. **P1 — the loop census, first read.** The census landed in `f997470` and has
   never run against a real executor. Read `loop_period_max_pre_s`,
   `loop_obs_max_pre_s`, `loop_body_max_pre_s`, `loop_sleep_max_pre_s`,
   `loop_n_over_pre` from the next sitting's `temp/logs/toss_records_*.jsonl`. It
   answers the three questions
   `logbook/2026-08-26-toss-loop-period-census.md` § Outcome poses, and it is what
   sizes B5's TARGET. **Until it has been read, B5 chooses no number.**
2. **P2 — the seat-edge decomposition.** `catch_event_dt_s` has a +183.9 ms
   median. Decompose it into (a) release execution lateness (measured −1.6 ms —
   on time; the "+52.3 ms" was beam occlusion, see § 1.4),
   (b) flight-time model error (`achieved_flight_s` vs
   `flight_time_s`), (c) sensor detection lag (the residual). Only (b) and part of
   (a) are correctable; the residual is a hard floor on any evidence-gated commit,
   and § 1.4's whole prediction rests on which is which.
3. **P3 — the pipelined-floor model.** Teach `cadence_rung_check.py` the pipelined
   commit ladder and re-run `--frontier` and `--grid`. The probe must red the
   pipelined ladder for the same reason it reds the pre-audit one: a probe that
   cannot show the failure has lost the finding.

**Acceptance.** P1 produces a `max(loop_period_max_pre_s)` across a session and a
named dominant term. P2 produces a three-way split summing to the measured median
within 10 ms. P3's `--pipeline --grid` reports zero violations against the *new*
floors and non-zero against the old ones.

**Dependencies.** P1 needs one sitting's corpus (any rung; R0–R3 are bookable
today). P2 and P3 need nothing.

### Phase B1: Per-cycle node state becomes an object — COMPLETE 2026-08-27

**Modified files**
* `ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py`
* `tests/ros/test_toss_coordinator.py`, `tests/ros/test_toss_continuous_node.py`

**Scope (the layout as it stood BEFORE B1 — none of these bare names is a node
attribute any more; see "As built" below).** `_build_toss_cycle` installed ~20
fields of **node-global**
per-cycle state under `self._lock` (`_toss_release_state`, `_toss_release_cmd`,
`_toss_aim`, `_toss_landing_global_mm`, `_toss_platform_target_mm`,
`_toss_positioning_move`, `_toss_prepare_pending`, `_toss_throw_dispatched`,
`_toss_stroke_seen`, `_toss_track_confirmed`, `_toss_pretilt_hold_raised`,
`_toss_announced_reach`, `_announced_ball_id`, `_prev_announced_ball_id`,
`_preexisting_flight_ids`, `_toss_next_release_perf`, `_toss_next_landing_perf`,
`_toss_prev_landing_perf`, `_toss_cycle_landing_perf`, `_toss_record_announce`),
and `_clear_toss_cycle_state` tears them down. Two coexisting cycles would
silently share every one of them.

This phase extracts a `TossCycleState` dataclass, returns it from
`_build_toss_cycle` alongside the sequencer, and threads it explicitly through
`_step_toss_sequence`, `_build_toss_observations`, `_position_platform_for_toss`,
`_prepare_toss_catch`, `_announce_toss`, `_dispatch_toss_throw`,
`_publish_toss_reach`, `_toss_stay` / `_toss_recenter` / `_toss_safe_abort`, and
the record builders. **No behaviour changes**: the single-slot callers pass the
one state object and the code path is otherwise identical.

**As built (2026-08-27).** Every moved name above is now a `TossCycleState`
field reached through the committed slot — `self._toss_committed.release_state`,
`.release_cmd`, `.aim`, `.landing_global_mm`, `.platform_target_mm`,
`.positioning_move`, `.prepare_pending`, `.throw_dispatched`, `.stroke_seen`,
`.track_confirmed`, `.pretilt_hold_raised`, `.announced_reach`,
`.next_release_perf`, `.next_landing_perf`, `.record_announce` — and B4's scope
should be read in that spelling, not the bare-attribute one.

Five of the fields listed above did **not** move into the per-cycle state, and
each has a reason: the `_announced_ball_id` pair (`_announced_ball_id` /
`_prev_announced_ball_id` — census D6, and shared verbatim with the RELOAD path)
plus `_preexisting_flight_ids` (written by the reload paths), and
`_toss_prev_landing_perf` / `_toss_cycle_landing_perf` (the arrival boundary's
cross-cycle latch, reset per SESSION by `_reset_toss_arrival_boundary`).
`_ball_possession` (the latch survives across cycles) stays too, and was never
in the twenty-field list.

**Critical details.** The lock discipline is unchanged — the state object is
written once at build under `self._lock` and read under it. The subscriber
callbacks (`_on_balls`, `_on_hand_telemetry`, `_on_target_feedback`,
`_on_announcement`) were expected to write into node-global per-cycle fields and
to need re-routing to the **committed** slot's state, which is the one field this
phase adds to the node (`self._toss_committed`) and B4 extends with
`self._toss_staged`. **As built that rule was a no-op**: after D1 (2026-08-26)
moved the possession latch out of `_on_balls`, no subscriber callback writes any
moved field.

**Acceptance.** `./run_tests.sh --full` green with **zero test edits that change
an assertion** — only the ones that reach into the moved fields. A structural test
asserts `_build_toss_cycle` returns a state object and that no per-cycle field
remains a bare node attribute.

**Dependencies.** none.

### Phase B2: The release instant becomes an input — NOT STARTED

**Modified files**
* `toss_sequencer.py` (`release_at_perf` field, `start()`, `slip_s` property)
* `toss_session.py` (`next_release_at(landing_perf)`)
* `reload_coordinator_node.py` (`_build_toss_cycle` passes it)
* `tests/ros/test_toss_sequencer.py`, `tests/ros/test_toss_session.py`

**Scope.** § 2.6's three rules. `release_at_perf = 0.0` (the default) reproduces
today's `now + throw_delay_s` **bit-for-bit** — the same "0.0 is the only unset
sentinel, a negative is a preserved sign typo" doctrine `__post_init__` already
uses.

**Critical details.** `_toss_deadline_s` reads `seq.throw_delay_s` to size the
per-goal ceiling (`reload_coordinator_node.py:671`). With an absolute release the
relevant quantity is `release_at − start`, and a ceiling that drifts inside a
legitimate window SAFE_ABORTs; the ceiling must be re-derived from the same
absolute number, and its docstring's doctrine ("never inside a legitimate sequence
window") re-stated at the new expression.

**Acceptance.** A test constructs the same goal both ways (derived and absolute)
and asserts the two FSMs produce **identical decision sequences instant for
instant**. `./run_tests.sh --full` green.

**Dependencies.** B1 (not strictly, but landing it after keeps the diffs
separable).

### Phase B3: Session-scoped arming, and drain-before-`go_home` — NOT STARTED

**Modified files**
* `reload_coordinator_node.py` (`_execute_toss_continuous`, `_prepare_toss_catch`,
  the three teardowns, a new `_arm_session` / `_disarm_session`)
* `toss_sequencer.py` (PREPARE's contract narrows; the ≥1-tick gap's rationale is
  re-stated at the constant, not deleted)
* `toss_session.py` (S5′ / S6 / S7 in the module docstring — census F7's
  "re-argued **in writing**")
* `tests/ros/test_toss_coordinator.py`, `tests/ros/test_toss_continuous_node.py`,
  `tests/ros/test_toss_session.py`

**Scope.** Move `trajectory/arm_catch` raise + confirm, `set_hand_gains`,
`catch/vel_scale`, `catch/prime_hold`, `catch/pretilt_hold` **and the
`catch/reach_center` declaration** (Q-3 resolution, 2026-08-27 — declared once,
immediately before the session raise that consumes it) from the per-cycle
PREPARE bundle to a **session-scoped** raise, executed once before cycle 1 and
lowered once at the session terminal. What remains per-cycle in
`_prepare_toss_catch`: `catch/prime_dispatched`, `catch/armed` (see the S6
amendment in § 2.3), and the phantom-flight snapshot refresh — all publishes, no
service round trips — plus the new **reach-centre drift guard** (refuse a cycle
whose B leaves the session envelope).

Implement S7 as one method, `_drain_pipeline_and_disarm()`, called at the top of
every path that reaches `_go_home()`: `_toss_safe_abort`, `_toss_recenter`,
`_recentre_for_reload`, `_safe_toss_on_early_exit`, and the session terminal.

**Critical details, in the order the publishes must happen.**
* The in-bundle order that survives is load-bearing and is documented at
  `_prepare_toss_catch`: gains → arm raise → vel_scale → prime_dispatched → armed
  → snapshot. Hoisting the first three to session scope preserves the *relative*
  order (they now all precede every cycle's armed edge by seconds), which is
  strictly stronger than the tick-based guarantee it replaces.
* The teardown order does **not** change: `catch/armed` False must precede the
  `prime_hold` release, because a released hold meeting a still-armed
  `catch_coordinator` re-opens the auto-prime with a ball in the cup
  (`_toss_stay` / `_toss_recenter`). Session scope moves *when* the teardown runs,
  never the order inside it.
* `_arm_catch(True)` C2-stops any in-flight move. Under S7 no move is ever in
  flight when it runs (the session arms before cycle 1's positioning is
  dispatched — or, if cycle 1 must move, after that move's verified arrival,
  exactly as `_enter_preparing` requires today).
* **[Q-3 RESOLVED 2026-08-27 — the capture is edge-triggered, and worse than
  this plan assumed.]** `trajectory_node._on_reach_center` only stores a pending
  declaration; `_svc_arm_catch` read-and-clears it **before** its idempotent
  early return, so under a standing latch every per-cycle declaration is
  *consumed and discarded* and the envelope centre stays **frozen at whatever the
  session raise captured** (not "falls back per cycle" — a session that arms
  while parked away from B fails from cycle 1, demonstrated empirically through
  the real node harness). **The adopted design is to scope the declaration the
  way S6 scopes the raise: declare `catch/reach_center` ONCE, at session scope,
  immediately before the session `arm_catch` raise, and drop the per-cycle
  declaration.** Zero `trajectory_node` change, zero contract change; it also
  closes the leaked-pending hazard (`catch_reach_envelope.md` § 5 residual 5 —
  with no per-cycle raise, a pending declaration would otherwise sit live for a
  later interlude raise to consume). Root cause: *the declaration's lifetime is
  scoped to the raise it feeds*. Two obligations ride with it: (a) a **drift
  guard** — a cycle whose nominated B differs from the session centre by more
  than the envelope margin must refuse (or re-arm), so the foreclosed
  per-cycle-varying-B case fails loudly; the documented forward path for
  displaced chaining is the redundant-raise capture (move the idempotent early
  return after the centre capture for `want=True`), taken only when a session
  genuinely needs a different B per cycle. (b) **T-I3 must assert the captured
  value of `_catch_envelope_center`, not publish ordering** — the bench trace
  recorder's CS-4 (one declaration per cycle, ≥1 tick before the arm) stays
  green under B3 while the declaration goes unapplied, i.e. CS-4 alone is a
  false green; B6 must re-cut CS-4 for pipelined sessions.

**Acceptance.** `./run_tests.sh --full` green. A structural test pins that
`_go_home` has no call site that is not preceded by `_drain_pipeline_and_disarm`
inside the same function (the `test_dwell_tilt_reads_have_exactly_one_call_site`
idiom, `tests/ros/test_toss_continuous_node.py:1492`). A bench grep for *"catch
latch armed mid-move"* across the B6 sitting returns **zero lines** — an ABSENCE
acceptance, exactly as `SAFE_ABORT_LADDER_S`'s comment already prescribes.

**Dependencies.** B1.

### Phase B4: The two-slot pipeline — NOT STARTED

**New/modified files**
* `toss_sequencer.py` — `PHASE_STAGED`, `PHASE_COMMITTING`, `_step_staged`,
  `_step_committing`, `commit_budget_s`, `commit_at`, `slip_s`,
  `SLIP` decision, the static/evidence gate split
* `toss_session.py` — the two slots, S1′, `required_dwell_s`'s pipelined branch,
  the unwind table of § 2.4.3
* `reload_coordinator_node.py` — `_tick_toss_pipeline`, slot-aware clamp reads,
  the discard path
* `ros_ws/src/jugglebot_interfaces/action/TossContinuous.action` — two additive
  feedback phase strings (comment-only otherwise)
* `config/hardware_config.yaml` → `jugglebot_operational.toss_pipeline_enabled`
  (**default `false`**) + regenerate
* `toss_record.py` — additive fields `staged_at_s`, `commit_at_s`,
  `commit_slip_s`, `staged_discarded_reason` (no `SCHEMA` bump: the schema's own
  rule is that purely additive fields do not bump)

**Scope.** Everything in § 2.2–2.4. `_run_toss_cycle` is retained verbatim for the
single `Toss` action and for a non-staging session cycle; the session gains
`_tick_toss_pipeline`, which per iteration steps the committed slot, then the
staged slot, then sleeps. Ordering is not cosmetic: the committed slot owns the
hand, so it must always get the tick first, and a structural test pins the order.

**IPC / message formats.** No new topics, no new services, no wire changes. The
two additive feedback strings:

```python
SESSION_PHASE_STAGED     = 'STAGED'       # a cycle's preamble is complete and it is
                                          #   waiting for its commit instant
SESSION_PHASE_COMMITTING = 'COMMITTING'   # the commit tick (evidence -> announce -> dispatch)
```

Additive in the same sense `SESSION_PHASE_RELOAD` was: an existing consumer sees a
phase it does not recognise, never a phase that changed meaning.

**Critical details.**
* **The commit gate is the arm point** (§ 2.4.2). `ball_seated` is read on the
  dispatch tick and nowhere earlier; a structural test asserts `ball_seated` and
  `hand_parked` are referenced only from `_step_committing` (and `_step_checking`
  for the serial path), reusing the `inspect.getsource`-per-handler idiom already
  in `tests/ros/test_possession_replay.py:192`.
* **Staging is skip-only** (§ 2.4.1). A structural test asserts `_step_staged`
  cannot emit `ACTION_POSITION_PLATFORM` with `positioning_move_expected` true.
* **The clamp reads name their slots** (§ 2.5b). `_expected_landing_perf` →
  committed; `_expected_prev_landing_perf` → previously committed;
  `_expected_next_cycle_perf` → staged. Structural + behavioural tests.
* **The unwind runs before the ladder** (§ 2.4.3, S7). The staged slot is
  discarded, its record is closed with `staged_discarded_reason`, and only then
  does `_toss_safe_abort` run.
* **`intends_another_cycle`** becomes "a staged slot exists or will be created",
  and keeps its deliberately-unclamped-on-the-boundary approximation.
* **The single `Toss` action is untouched.** It has no previous cycle to pipeline
  behind; it takes the serial path and its arithmetic is unchanged.

**Acceptance.** The flag defaults false and a test asserts the shipped default
produces **decision sequences identical to the pre-B4 tree** on the whole
`cadence_rung_check` grid. With the flag true, the probe's `--pipeline --grid`
reports zero accept-implies-flies violations, and the § 2.7 floor table
reproduces from the shipped `required_dwell_s`. `./run_tests.sh --full` green.

**Dependencies.** B1, B2, B3.

### Phase B5: Loop-cost trim — NOT STARTED

**Modified files**
* `reload_coordinator_node.py` (`_run_toss_cycle`, `_tick_toss_pipeline`,
  `_build_toss_observations`)
* `toss_sequencer.py` (`NODE_LOOP_PERIOD_S`, and only after B5's measurement)

**Scope.** Three levers, in the order their cost is known:

1. **Absolute-schedule tick pacing.** Replace `time.sleep(_TICK_S)` with
   `next_due += _TICK_S; sleep(max(0, next_due − now))`, plus a **half-tick
   early-fire band** so a due instant landing marginally after the wake does not
   cost a whole extra period. This is not a new idea in this repo: it is exactly
   the fix the hand-sensor poller took — *"Feature 1 — absolute schedule **plus** a
   half-tick early-fire band… `s_next_due_us += POLL_INTERVAL_US` from the previous
   DUE instant"* (`logbook/2026-08-24-poller-cadence-and-tristate-tx.md` § Feature
   1), and the reason it needed the band there applies here for the same reason:
   the absolute due instants land exactly *on* nominal tick instants. It converts
   the period from an output into an input, which is what every budget in this
   stack is denominated in.
2. **Incremental observation build.** `_build_toss_observations` rebuilds ~25
   fields per tick under the lock, including a numpy `hypot` and a sensor query.
   Cache the slowly-varying half behind the subscriber callbacks' own dirty flags
   and rebuild only the live half per tick. `loop_obs_max_pre_s` from B0/P1 says
   whether this is the dominant term before any of it is written.
3. **Blocking calls off the tick.** B3 already removes the three synchronous
   service round trips from the per-cycle path. What remains is
   `_dispatch_toss_throw`'s own call, which is *inside* the commit tick by
   necessity, and `_go_home` / `_arm_catch` on the teardown paths, which are off
   the cadence path entirely.

**Only then** may `NODE_LOOP_PERIOD_S` be re-cut, and it is re-cut **by a human
reading the census**, never by the census. `test_the_census_never_feeds_a_budget`
(`tests/ros/test_toss_sequencer.py:1838`) inspects the compiled identifier set of
the budget functions and must keep passing: *"a bound that tracks its own
degradation hides the degradation"*.

**Acceptance.** The census's `loop_period_max_pre_s` over a full session falls
below the re-cut constant with the sizing discipline `ARRIVAL_BAND_MAX_S` uses
(ceil to the next 10 ms, so the constant is a bound rather than a datum).
`loop_n_over_pre` is 0 on every successful cycle. The probe's `--frontier` moves
by the predicted amount and the runbook's clearance table is re-cut.

**Dependencies.** B0/P1 (the measurement), B3 and B4 (which remove work).

### Phase B6: Hardware validation and close-out — NOT STARTED

**Scope.** Fly § 6's ladder. Then: update
`tests/hardware/session_cadence_ladder.md` with the pipelined rungs and the
re-cut clearance table; add C-POSSESS-1 § 3.4's slot-naming rule to
`ros_ws/docs/ball_possession_contract.md`; write the logbook entry (full
investigation form — this plan clears at least two of the three Discussion
triggers before it starts); archive this plan.

**Acceptance.** § 6's PASS rows at rungs P1–P4 minimum, with the ladder's stop
conditions unbreached.

**Dependencies.** B4 (B5 optional but strongly recommended — see the 17.9 ms
clearance at `h = 1.0`).

---

## 5. Testing Plan

The rule for this plan: **every threshold gets a probe before it gets a test.**
No number below is typed into an assertion until a probe has produced it
deterministically against the pinned stack, and the confirmed recipe goes into the
test docstring and the phase's logbook (`tools/probes/README.md`; the two saves
that made this a rule are in
`logbook/2026-05-11-tier1a-real-solver-failures.md`).

### 5.1 Probes — first, and named

| ID | probe | question it answers | output |
|---|---|---|---|
| **P1** | the shipped `LoopPeriodCensus`, read from a sitting's `temp/logs/toss_records_*.jsonl` by `tools/probes/toss_loop_census.py` (new, committed) | is `NODE_LOOP_PERIOD_S = 0.040` still a bound, and which of obs / body / sleep dominates? | three maxima + `loop_n_over_pre` |
| **P2** | `tools/probes/seat_edge_decomposition.py` (new, committed) | how does the +183.9 ms median `t_catch_raw_ros − announce_landing_time_ros` (the mined ROS-clock RAW-bit measurand; its live perf-clock twin `catch_event_dt_s_fsm` measures +3.7 ms apart) split into release lateness / flight-model error / detection lag? | a three-way split summing to the median within 10 ms |
| **P3** | `tools/probes/cadence_rung_check.py --pipeline` (extended) | do the pipelined floors admit the milestone, and does accept still imply flies? | the § 2.7 table + zero grid violations |
| **P4** | `/tmp/probe_arrival_clamp_pipelined.py` (one-off, uncommitted) | at the milestone periods, where does the arrival window close and is the band watched out? | the § 2.5 table, **by calling `arrival_boundary_t`, never by restating it** |
| **P5** | `/tmp/probe_retention_inverted.py` (one-off) | does an inverted retention interval answer UNKNOWN rather than REJECTED at the milestone numbers? | the state and reason string per rung |
| **P6** | `tools/probes/possession_replay.py --emit-fixture` (existing) | does the pipelined verdict path reproduce a recorded sitting row-for-row? | a new fixture module, § 5.4 |

The re-derivation recipe for every arithmetic number in this document:

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate
python tools/probes/cadence_rung_check.py --solve --frontier --pipeline
```

### 5.2 Unit tests (offline, no hardware)

Against the house idiom: sentence-style names that state the assertion, module-level
helpers rather than fixtures, and constants imported rather than typed.

| ID | test | pass criterion |
|---|---|---|
| **T-U1** | `test_the_commit_budget_is_one_loop_period_not_four` | `commit_budget_s(v)` == `min_throw_event_delay_s(v) + NODE_LOOP_PERIOD_S + FLOOR_REPRESENTATION_SLACK_S`, and is **strictly less** than `min_throw_delay_for_release_s(v, False)` by exactly `3 × NODE_LOOP_PERIOD_S` |
| **T-U2** | `test_a_staged_cycle_can_never_emit_a_hand_bearing_action` | over every reachable `(phase, observation)` pair in the staged stage, the emitted action is in `{NONE, POSITION_PLATFORM(skip), PREPARE_CATCH}` — **property test**, § 5.3 |
| **T-U3** | `test_the_dispatch_is_evidence_armed_on_its_own_tick` | flipping `ball_seated` False on the commit tick yields SLIP-or-REJECT and **never** `ACTION_DISPATCH_THROW`; flipping it False on the tick *before* commit and True at commit still dispatches (the gate is the commit read, not a latch) |
| **T-U4** | `test_ball_seated_is_read_only_from_the_commit_handler` | structural: `inspect.getsource` per handler; `'ball_seated'` absent from `_step_staged`, `_step_positioning`, `_step_preparing` |
| **T-U5** | `test_a_staged_cycle_never_stages_behind_a_commanded_move` | `positioning_move_expected=True` ⇒ the session takes the serial path; asserted on the decision stream, not on a flag |
| **T-U6** | `test_the_slip_is_bounded_by_the_confirm_window_and_nothing_else` | the slip's maximum is `CATCH_CONFIRM_WINDOW_S` by **derivation** — the test reads the constant, and a mutation of it moves the bound |
| **T-U7** | `test_the_unwind_discards_the_staged_slot_before_any_go_home` | for each of the five upstream terminals in § 2.4.3, the ordered call log has the discard strictly before `_go_home` |
| **T-U8** | `test_the_sensor_is_told_the_committed_slots_landing` | with both slots live and *different* landings, `_expected_landing_perf` returns the committed one; `_expected_next_cycle_perf` returns the staged one; `_expected_prev_landing_perf` returns the previously-committed one |
| **T-U9** | `test_an_inverted_retention_interval_answers_unknown_not_rejected` | at `(dwell 0.4349, T 0.9032)` the retention part is `UNKNOWN`; **probe P5 first** |
| **T-U10** | `test_the_arrival_windows_of_two_live_cycles_abut_by_identity` | `close(L)` and `open(N)` are the same float, driven through the shipped `arrival_boundary_t` |
| **T-U11** | `test_the_pipelined_dwell_floor_admits_the_milestone` | `required_dwell_s` at `(h=1.0, 0.4349)` and `(h=1.3, 0.4958)` is below the commanded dwell, with the clearances of § 2.7; **probe P3 first** |
| **T-U12** | `test_the_release_instant_is_an_input_not_a_rederivation` | derived and absolute construction produce identical decision streams instant for instant (B2) |
| **T-U13** | `test_the_shipped_default_is_the_serial_pipeline` | with `toss_pipeline_enabled` false, the decision stream over the whole grid is identical to the pre-B4 tree |
| **T-U14** | `test_the_census_never_feeds_a_budget` (**existing, must keep passing**) | `commit_budget_s` joins the inspected set |
| **T-U15** | `test_the_committed_slot_is_ticked_before_the_staged_slot` | structural, on `_tick_toss_pipeline`'s AST — call order, the `test_the_worker_is_drained_before…` idiom |

### 5.3 Property tests on the pipeline FSM

**These would be the first Hypothesis tests in `tests/ros`** (the library is used
only in `tests/sim/` today; profiles `ci-fast` = 50 / `ci-deep` = 1000 are
registered suite-wide in `tests/conftest_hypothesis.py` and **no per-test
`@settings` override exists anywhere in the repo** — do not add one, and do not
reach for the unused `hypothesis_deep` marker).

| ID | property | strategy |
|---|---|---|
| **T-P1** | **At most one cycle is past COMMIT at any instant** (S1′) | a random interleaving of tick instants, observation flips (`ball_seated`, `hand_parked`, `track_active`, freshness), upstream outcomes and cancel requests; assert the invariant after every step |
| **T-P2** | **No `ACTION_DISPATCH_THROW` is ever emitted on a tick whose observation had `ball_seated` False** | same strategy; this is the safety property stated as a temporal one |
| **T-P3** | **Every terminal is reached exactly once and every action fires at most once** | the existing single-cycle guarantee, lifted to two slots |
| **T-P4** | **Discard is total**: after any unwind, no staged state survives into the next cycle | assert the slot object is unreachable and no per-cycle field is non-default |
| **T-P5** | **The schedule is monotone**: `t_release(k+1) > t_release(k)` under every slip sequence | catches a slip that runs backwards |

Strategies generate *observation streams*, not internals, so the properties hold
against a refactor rather than against an implementation.

### 5.4 Replay-driven acceptance on recorded bags

The house pattern is three instances deep (`possession_verdict_bag_check.py →
possession_fixtures.py`; `toss_record_miner.py → toss_record_fixtures.py`;
`possession_replay.py → toss_verdict_replay_fixtures.py`) and this plan adds a
fourth consumer rather than a fourth pattern.

| ID | test | pass criterion |
|---|---|---|
| **T-R1** | `test_the_pipelined_verdict_census_is_the_cup_census_exactly` | replay bag `2026-08-26_14-25-16` through the **pipelined** verdict path; the 27 adjudicated cycles must still come out **23 CAUGHT / 4 MISSED**, row for row. The pipeline must not move a single verdict on a recorded sitting — if it does, it changed the possession semantics, which it has no business doing |
| **T-R2** | `test_the_pipelined_schedule_reproduces_the_recorded_release_spacing` | driving the real `TossSessionSequencer` over the fixture's cycles, the serial and pipelined schedules agree wherever the dwell is above the serial floor |
| **T-R3** | `test_a_pipelined_replay_of_the_B6_sitting_matches_its_own_log` | after B6, emit a new fixture from the pipelined sitting and pin the census, the per-cycle `commit_slip_s`, and the achieved dwells |
| **T-R4** | `test_the_pre_audit_ladder_still_reproduces_its_failure` (**existing**) | unchanged — the regression stays findable |

### 5.5 Integration tests (real system, safe conditions)

| ID | test | pass criterion |
|---|---|---|
| **T-I1** | the toss's self-announcement through the **real** `BallTracker` correlation and **real** `CatchCoordinator`, with a staged cycle live during the previous flight (extends `tests/ros/test_toss_integration.py`) | exactly one announcement is correlated per ball; a staged cycle contributes none |
| **T-I2** | `catch/pretilt_hold` + `catch/prime_hold` replayed into a real `CatchCoordinatorNode` under S6's standing latch | no auto-prime fires across the whole session; no pre-tilt is installed |
| **T-I3** | the reach-envelope-centre declaration under a standing latch (Q-3) | `trajectory_node` captures the declared centre per cycle, or the test names the degradation explicitly |

### 5.6 Regression tests

| ID | test | pass criterion |
|---|---|---|
| **T-G1** | `cadence_rung_check --grid` with the flag **false** | byte-identical decisions to the pre-B4 tree over ~1500 grid points |
| **T-G2** | the published R0–R5 ladder, four ways, flag false | `PUBLISHED LADDER: all rungs FLY`, unchanged |
| **T-G3** | the single `Toss` action | no decision changes at all; it never stages |
| **T-G4** | `test_the_hand_floor_is_dominated_by_the_plumbing_term` (**existing**) | must be **re-taken**, not deleted: under the pipelined floor the plumbing term's dominance narrows from 0.2030 s to 0.0947 s at the band floor. It is still dominant, but the margin over the 0.0715 s worst-case ILC trim sensitivity falls from 2.8× to **1.3×**. The test's docstring must carry the new number and the re-taken argument |

**T-G4 is the one existing test this plan genuinely stresses**, and it is worth
surfacing rather than quietly re-baselining: shortening the plumbing term brings
the hand's own geometry back toward binding, which is the intended direction and
also the direction in which a bad number stops being caught by a comfortable
margin.

### 5.7 The gate

`./run_tests.sh --full` (every tier, `nightly` included) before **every** commit
in this plan and before the B6 sitting — the rule's cases (a) before any hardware
sitting and (b) at plan-phase closure both apply. Report the (date, command,
result) triple in each commit message and each logbook entry.

---

## 6. Hardware validation ladder — the runbook

This section is a **runbook sketch**; B6 lands it as rungs in
`tests/hardware/session_cadence_ladder.md` alongside R0–R5 rather than as a second
file, so one board carries all bookable rungs.

### 6.1 Standing settings — unchanged from the cadence ladder

```
stop_on_miss  : true      one miss ends the sitting
on_empty_cup  : STOP      never RELOAD
num_throws    : 5         first run of any rung
catch_vel_scale: 0.0      (⇒ the 0.9 config default — changing it invalidates every dwell below)
```

REBOOT the can-bridge Teensy before the sitting. Record `uptime_ms`,
`iq_brake_min_a`, the per-cycle `dwell_s`, and the toss-record JSONL. **Score the
miner, not the console.** Do not book any rung until
`session_cadence_ladder.md`'s carried finding 2 is closed (§ 8, P-4) — with it
open, an aimed chain never stages and the sitting measures the serial path.

### 6.2 The rungs

Two heights, five dwells, stepping down. `throw_delay_s` is **0.0** at every
pipelined rung: under B4 the release is scheduled absolutely and the delay field
is no longer the cadence lever (§ 2.6). The dwell steps are the owner's:
0.76 → 0.65 → 0.55 → 0.50 → 0.45.

| rung | `throw_height_m` | ⇒ `T` | `dwell_time_s` | floor (loop 0.040) | clearance | predicted achieved dwell | predicted period | throws/min |
|---|---|---|---|---|---|---|---|---|
| **P0** | 1.30 | 1.0298 | 0.76 | 0.3941 | 366 ms | 0.760 | 1.790 | 33.5 |
| **P1** | 1.30 | 1.0298 | 0.65 | 0.3941 | 256 ms | 0.650 | 1.680 | 35.7 |
| **P2** | 1.30 | 1.0298 | 0.55 | 0.3941 | 156 ms | 0.550 | 1.580 | 38.0 |
| **P3** | 1.30 | 1.0298 | **0.50** | 0.3941 | 106 ms | 0.4958 → 0.500 | 1.530 | **39.2** ⭐ |
| **P4** | 1.00 | 0.9032 | **0.45** | 0.4170 | 33 ms | ~0.495 (slip ~45 ms) | ~1.398 | **42.9** ⭐ |
| **P5** | 1.00 | 0.9032 | 0.43 | 0.4170 | **13 ms** | ~0.495 (slip ~65 ms) | ~1.398 | 42.9 |

**P3 and P4 are the milestone.** P5 is the milestone's lower edge and is
deliberately last: its 13 ms of accept clearance is the same razor-edge class the
runbook flags at R5's 1.9 ms, and it is the rung B5 exists to widen. **P0 is not
optional** — it is the first sitting under a standing catch latch (S6) and under a
staged preamble, at a dwell with a third of a second of margin, and its job is to
find out whether either of those is a surprise before the cadence is anywhere near
the edge.

### 6.3 What each rung measures

| row | measurand | source | why |
|---|---|---|---|
| **PIPE-1** | `commit_slip_s`, per cycle | toss record | the § 1.4 prediction, tested. Median slip at P3 should be ~0; at P4/P5 ~45–65 ms. A slip **rising** across a session is a loop-cost regression |
| **PIPE-2** | `loop_period_max_pre_s`, `loop_n_over_pre` | toss record | `loop_n_over_pre` must be **0 on every successful cycle**. Non-zero on a success is the early warning the census exists for |
| **PIPE-3** | achieved `landing → release`, per cycle | toss record | scored against the commanded dwell **and** against `seat_edge + commit_budget_s`. These are two different claims and both are published |
| **PIPE-4** | `dispatch → catch-stroke-end` gap, per cycle | hand telemetry | the C-HAND-1 no-overlap margin. **A negative gap is an abort-the-sitting event, not a data point** (inherited verbatim from R4/R5) |
| **PIPE-5** | any commanded platform motion between the verdict and the next release | `/trajectory/commanded_position` | S6's accepted cost (§ 2.3, S5′ point 1). Any motion here is a stop |
| **PIPE-6** | the cup, by eye, between cycles | the operator | retention is CLOSED BEFORE THE MEDIAN SEAT EDGE at these dwells (§ 2.5c), so a bounce-out has no machine witness. This row is more load-bearing at these rungs than at any before them |
| **PIPE-7** | grep the log for *"catch latch armed mid-move — installed a graceful stop"* | `/rosout` | **B3's acceptance is an ABSENCE.** Zero lines |
| **PIPE-8** | `landing → hand back inside the park band` | hand telemetry | the model says 0.106 s at `h = 1.0`, 0.093 s at `h = 1.3`; `handoff_margin_s` is sized on it |

### 6.4 Stop conditions

Stop the sitting, do not step down, and debrief on any of:

* any **negative** `dispatch → catch-stroke-end` gap (PIPE-4);
* any `REJECTED_HAND_NOT_PARKED` on a cycle whose previous catch was good — that
  is the handoff margin being too small, not a machine fault;
* any `REJECTED_DWELL` at the accept gate — a floor moved; re-run
  `cadence_rung_check --pipeline` before continuing;
* any `ABORTED_CANT_MAKE_RELEASE` — under the pipeline this should be
  **structurally unreachable** (the commit gate slips rather than aborting), so
  one is a design finding, not a tuning finding;
* any commanded platform motion between a verdict and the next release (PIPE-5);
* any *"catch latch armed mid-move"* line (PIPE-7);
* `loop_n_over_pre` non-zero on a successful cycle (PIPE-2) — **[caveat
  2026-08-27]** on the pre-B5 tree this fires on 48 of 66 successful cycles
  (B0/P1), so this stop condition presumes B5's tick pacing has landed and
  `NODE_LOOP_PERIOD_S` has been honestly re-cut from a post-B3/B4 census; flying
  any pipelined rung before that makes PIPE-2 an instant stop, which is the
  census doing its job;
* any `MISSED_SENSOR_BLIND` — the cup could not look, which at these periods
  should not be schedule-caused (§ 2.5) and therefore points at the sensor;
* any HAND row outside `tests/hardware/session_anomaly_fixes.md` § PASS/ABORT.

**A cancel is always deferred at these dwells, and under the pipeline the stop
button gains up to two cycles of latency** (the deferred cycle plus the staged
drain). The cancel button is not the E-STOP. Say this out loud before arming.

---

## 7. Non-goals — stated so they are not drifted into

* **True two-ball overlap (`P < T`).** This plan rehearses two-ball *timing* with
  one ball. Nothing here puts a second ball in the air, and the pipeline's whole
  safety argument (§ 2.3, S1′) rests on exactly one ball existing at a time.
* **Dwell 0.31 s at `h = 0.5 m`.** The pipelined floor there is 0.4941 s against a
  0.3075 s target — **61 % over**, and the deficit is not plumbing: the dispatch
  budget alone is 0.3040 s and the park re-entry 0.1501 s. Two successors own it,
  and **neither is planned here**: the accel-FF / inertia arc
  (`accel-ff-inertia.md`, currently parked — its own hard prerequisite is a
  firmware stale-hold torque decay), which is what shortens the hand's
  windup and tail; and the R6-class Platform-Teensy fork
  (`session_cadence_ladder.md` § R6 — a `calcCatch` geometry change, operator
  decision 3 of 2026-08-21, **not being built**). `h = 0.8` (target 0.3890 s
  against a 0.4390 s floor, 50 ms over) is the nearest miss and is the first rung
  either successor would unlock.
* **Metronome adaptation policy.** Whether and how the beat clock re-anchors on
  the observed seat edge, and what "bounded" means for its slip, is Phase C. This
  plan exposes `slip_s` and `commit_slip_s` and stops. Deriving a policy from a
  corpus that does not exist yet is the failure this plan's own B0 exists to
  avoid.
* **Displaced (non-zero `|B−A|`) pipelined chains.** The deferred A→B reach moves
  the platform during flight, so a staged cycle cannot verify its throw site. The
  skip-only rule (§ 2.4.1) makes such a cycle fall back to the serial path, which
  is correct and is not a regression — it simply does not get faster.
* **Relaxing any floor.** `min_throw_delay_for_release_s`,
  `hand_floor_dwell_s`, `handoff_margin_s`, `ARRIVAL_BAND_MAX_S`,
  `CATCH_CONFIRM_WINDOW_S` and `RELEASE_GUARD_S` are unchanged. The gain comes
  from removing work from the critical path, never from charging less for it.

---

## 8. Prerequisites and open questions

### 8.1 Prerequisites

| # | prerequisite | hard? | why |
|---|---|---|---|
| **P-1** | **FW 16 flashed on the can-bridge Teensy** | ✅ **SATISFIED 2026-08-26** (operator flashed from the Win10 box; first live `/link_status` read pending) | FW 16 is the poller + tri-state image (`587b363`). Post-flash the panel's `15 (SKEW — expected v16)` advisory disappears |
| **P-2** | **Phase A is deployed** — `cd ros_ws && colcon build --packages-select jugglebot` | **hard** | the install space must carry `f997470` + `6036476`; a sitting that measures the tracker-primary verdict measures nothing this plan cares about |
| **P-3** | **B0/P1 has read one sitting's census** | ✅ **SATISFIED 2026-08-27** (`tools/probes/toss_loop_census.py` over the 21:29 sitting, n=74) | the read re-scoped B5 (see § 3): the honest chained bound is **0.070 s**, `body` dominates, and the over-period counter already fires on 48/66 successful cycles |
| **P-4** | **`session_cadence_ladder.md` carried finding 2 is closed** | **hard for the pipeline to engage on tier 8b with an aim armed** — fix approved into this arc (own commit, between B2 and B4). Traced 2026-08-27: the re-command is **real, physically-required motion** (the catch policy levels the platform to receive; the throw aim re-tilts it; nothing reconciles them — they agree only at zero aim). Adopted fix: in `_publish_toss_reach`, reach at the commanded pre-tilt quaternion **iff** the receive tilt is within the ±1° aim authority of it (zero-aim byte-identical; displaced never triggers, its delta is ~2θ). Pending one owner physical-intuition check: the fix accepts the cup seating with ≤1° of tilt on aimed 8b — which aimed 8a already does on every validated cycle | with it open, every chained aimed cycle re-commands a ≤1 mm/≤1° move charged at a fixed 0.360 s ⇒ no cycle ever stages. Safely inert, but inert |
| **P-5** | the drive-restoration state of 2026-08-18 (`b084f98`) holds | **hard** | pre-2026-08-18 braking-clamp behaviour invalidates every catch-tail number this plan's floors are built on |

### 8.2 Open questions — decisions required before the phase named

**Status 2026-08-27: Q-1, Q-4, Q-5 DECIDED by the owner (2026-08-26 session);
Q-2 and Q-3 ANSWERED by measurement/diagnosis.** The original texts are kept
below for the record; the resolutions:

* **Q-1 → S6 as written** (session-scoped latch), owner-approved. Amended only
  in that `catch/armed` stays per-cycle (§ 2.3).
* **Q-2 → the release-latency hook must NOT be populated.** B0/P2 measured the
  ballistic release at **−1.6 ms** (on time); the "+52.3 ms" was beam occlusion
  on exit. The correctable term is the **flight-time model (+102.1 ms, 53 % of
  the bias)** — correcting it is a future arc's own change (it moves every
  scheduled landing), not this plan's. The hard seating/detection residual is
  +85.9 ms.
* **Q-3 → resolved: session-scoped declaration (option C)** — see § 4 B3. The
  redundant-raise capture is the documented forward path for per-cycle-varying B.
* **Q-4 → band on both measurands**: the milestone reads "the machine holds a
  0.43–0.50 s dwell", commanded and achieved both inside the band.
* **Q-5 → the owner books P5 in the first sitting** (declining the hold-for-B5
  recommendation). Carried with it, from B0/P1: at the measured loop bound the
  h=1.0 rungs P4–P5 are not honestly bookable until B5 lands and a post-B3/B4
  census is read (§ 3, B5 row) — so "first sitting" means the first sitting at
  which h=1.0 is bookable at all, and the P0–P3 (h=1.3) sitting does not wait.

* **Q-1 — the session-scoped latch (S6) is an owner decision, not a technical
  one.** It converts the armed duty cycle from ~97 % to 100 % and it is what
  closes the arm-mid-move seam by construction. The alternative — keep the
  per-cycle raise and transfer *ownership* between cycles without lowering —
  achieves the same seam closure with a more complex lifecycle and one more state
  to get wrong. **Decision required before B3.** The recommendation is S6 as
  written, because the failure it prevents (a `go_home` silenced mid-traverse,
  followed by a throw from a site the aim was not solved for) is a *mis-aimed ball
  at 4.4 m/s*, and the cost it accepts (a foreign ball meeting an armed machine in
  a 40 ms window that used to be disarmed) is bounded by `pretilt_hold`,
  `prime_hold` and an 80 mm reach envelope that are all already standing.
* **Q-2 — how much of the +183.9 ms seat-edge bias is correctable.**
  `_dispatch_toss_throw` already carries the hook (*"shifted EARLY by the
  T0-measured release latency (ships 0.0)"*) and Phase A measured the release
  running +52.3 ms late. Populating that shift would return ~52 ms of achieved
  period directly. **It is deliberately NOT in this plan** — it moves a
  motion instant on every cycle and belongs in its own change with its own
  logbook. Probe P2 sizes it; the decision is the owner's.
* **Q-3 — the reach-envelope centre under a standing latch.** C-REACH-1's capture
  happens at the `arm_catch` raise. With the latch standing, is the per-cycle
  `catch/reach_center` declaration still captured, or does the envelope fall back
  to the commanded pose? **Must be answered by reading `trajectory_node` before
  B3 lands**, and T-I3 is the test either way. Benign for the zero-displacement
  chain in scope; a hard blocker for displaced 8b.
* **Q-4 — what the achieved-dwell claim is scored against.** § 1.4 predicts
  `h = 1.0` reaching 0.4946 s achieved against a 0.4349 s commanded. **The
  milestone is met on both measurands** (both sit inside 0.43–0.50), but they are
  different claims and the runbook publishes both. Confirmation required that the
  owner reads the milestone as *"the machine holds a 0.43–0.50 s dwell"* and not
  as *"the commanded dwell is achieved to the millisecond"*, because the second
  reading is not reachable at `h = 1.0` without Q-2.
* **Q-5 — should P5 (dwell 0.43 at `h = 1.0`) be booked at all before B5?** Its
  13 ms of accept clearance is the razor-edge class the runbook declined at R5.
  The recommendation is to fly P0–P4, land B5, re-cut the clearance table, and
  then book P5.

---

## 9. Notes for Collaborators

### 9.1 Safety-critical invariants that must be preserved exactly

| invariant | where | consequence of getting it wrong |
|---|---|---|
| **`ball_seated` is a hard gate on any release, read on the dispatch tick** | `toss_sequencer._step_committing` | a full-speed empty stroke with the hand ascending from an unverified position |
| **The throw dispatch is single-shot and never retried** | `_step_throwing`, `_dispatch_toss_throw` | a re-dispatch re-packs a new `wall_time` if the first frame was lost, REPLACES a live stroke if the ack lied, and post-release clobbers the armed catch stroke on the last-writer-wins queue |
| **The hand ladders and `_MAX_ARM_DISPATCHES` are retained defence in depth** | `_prime_hand_with_retries`, `_retract_hand_with_retries` | see `project_reload_action_catch_latch` — never blind-re-dispatch a hand move, and never re-open the latch-cap question without re-reading the entry |
| **All motion is profiled; never a step position change** | `trajectory_node` / `planner` / `feasibility.validate` | jerky hardware movement; this plan commands no new motion primitive |
| **The Teensy-side `MAX_DEVIATION` guard is the leg-path safety authority** | can-bridge firmware | nothing on the Jetson is in that loop, and nothing in this plan enters it |
| **C-HAND-1: the catch stroke and the next throw stroke do not overlap** | `hand_stroke.min_turnaround_dwell_s`, kept in `required_dwell_s`'s `max()` | any kind-0/1/2 command clears the whole packed queue and reseeds the prelude from a live encoder reading taken at 41–96 rev/s — the 2026-07-25 clobber |
| **The teardown publish order: `catch/armed` False before the `prime_hold` release** | `_toss_stay` / `_toss_recenter` / `_toss_safe_abort` | a released hold meeting a still-armed `catch_coordinator` re-opens the auto-prime with the ball in the cup; the ascent would launch it |
| **C-POSSESS-1.C: a window may never outlast the machine's next scheduled event of the kind it is looking for; where a clamp leaves no interval, the answer is UNKNOWN** | `ball_possession.HandBallSensorSource._window` / `._retention_horizon`, `toss_record.label_from_sensor` | at these dwells a `REJECTED` from an inverted retention window is a positive bounce-out claim on every good cycle |
| **The arrival boundary has ONE home** | `ball_possession.arrival_boundary_t` | two computations of a boundary is how an abutment stops abutting |
| **The census has no control authority** | `test_the_census_never_feeds_a_budget` | a bound that tracks its own degradation hides the degradation |

### 9.2 Architecture decisions and their root causes

* **Two slots, not two FSMs sharing a hand.** S1's hazard is actuator ownership,
  not object count. S1′ preserves the hazard and relaxes the implementation, and a
  structural test makes the ownership rule mechanical rather than a matter of
  discipline.
* **The latch is session-scoped (S6) because a re-raise is a race.** The current
  build pays 0.160 s of `SAFE_ABORT_LADDER_S` and 2.80 s of
  `DEFAULT_SESSION_MISS_CLEANUP_S` to keep an arm from landing inside a `go_home`.
  A resource that is never lowered cannot be re-raised at the wrong moment.
* **Commit is one tick, and the announcement lives in it.** A slipped release
  invalidates an already-published announcement and there is no withdrawal message
  on the wire. Publishing at commit is what keeps the announcement true, and the
  tick it costs is returned by S6 making the armed→announce gap free.
* **Slip rather than refuse.** The cup's seat edge is late by a measured,
  systematic +183.9 ms median. A commit that refuses on a not-yet-seated cup would
  turn a healthy machine's ordinary timing into `REJECTED_NO_BALL`, which is a
  machine-fault verdict for a cadence fact — the same mis-routing
  `REJECTED_HAND_NOT_PARKED` was called out for at R5.
* **The floors are re-derived, never relaxed.** `commit_budget_s` charges strictly
  less than `pre_dispatch_budget_s` because strictly less work happens between the
  guard and the CAN frame. If the pipeline is disabled, the old budget is charged
  unchanged. Two branches, one derivation each, both imported by both gates.

### 9.3 Startup / shutdown ordering

`_arm_session` runs after cycle 1's verified arrival and before its PREPARE;
`_disarm_session` runs from `_drain_pipeline_and_disarm`, which every `go_home`
path calls first. On a node-level exit (`cancel` / `timeout` / `shutdown` /
exception) `_safe_toss_on_early_exit` routes through the same drain, so the staged
slot cannot outlive the executor. `rclpy` shutdown terminalises nothing on the
goal handle — a status transition on a dying executor can itself raise, which
would replace a clean shutdown with a spurious `ABORTED_EXCEPTION`.

### 9.4 Files affected

| file | action | phase |
|---|---|---|
| `ros_ws/src/jugglebot/jugglebot/toss_sequencer.py` | modified | B2, B3, B4 |
| `ros_ws/src/jugglebot/jugglebot/toss_session.py` | modified | B2, B3, B4 |
| `ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py` | modified | B1, B2, B3, B4, B5 |
| `ros_ws/src/jugglebot/jugglebot/toss_record.py` | modified (additive fields) | B4 |
| `ros_ws/src/jugglebot_interfaces/action/TossContinuous.action` | modified (comment + phase strings) | B4 |
| `config/hardware_config.yaml` + `config/generated/*` | modified | B4 |
| `ros_ws/docs/ball_possession_contract.md` | modified (§ 3.4 slot-naming rule) | B6 |
| `tests/hardware/session_cadence_ladder.md` | modified (P0–P5 rungs, clearance table) | B6 |
| `tools/probes/cadence_rung_check.py` | modified (`--pipeline`) | B0 |
| `tools/probes/seat_edge_decomposition.py` | **created** | B0 |
| `tools/probes/possession_replay.py` | modified (pipelined path) | B4 |
| `tests/ros/test_toss_sequencer.py`, `test_toss_session.py`, `test_toss_coordinator.py`, `test_toss_continuous_node.py`, `test_possession_replay.py`, `test_ball_possession.py` | modified | B1–B4 |
| `tests/ros/test_toss_pipeline_properties.py` | **created** (Hypothesis) | B4 |
| `tests/motion/test_cadence_rung_check.py`, `tests/motion/test_toss_record.py` | modified | B0, B4 |

### 9.5 Rollback plan

Three levels, in increasing cost:

1. **`toss_pipeline_enabled: false`** — one YAML key plus
   `python config/generate_config.py` plus a `colcon build`. The session reverts to
   the serial path and T-U13 pins that the decision stream is identical to the
   pre-B4 tree. This is the rollback the bench uses.
2. **Revert B4 alone.** B1, B2, B3 and B5 are independently valuable and
   independently green; B4 is the only phase whose revert loses the milestone.
3. **Revert B3** if S6 surprises on hardware. The per-cycle raise returns, and
   with it the arm-mid-move seam and the 2.80 s cleanup floor that fences it.
   `DEFAULT_SESSION_MISS_CLEANUP_S` must not be lowered while B3 is out.

### 9.6 Deploy

`cd ros_ws && colcon build --packages-select jugglebot && source install/setup.bash`,
then relaunch. B4's `TossContinuous.action` change is comment-plus-string-constant
and does not require a `jugglebot_interfaces` rebuild; B4's config change does
require `python config/generate_config.py` **before** the build, with the
regenerated artifacts staged in the same commit.
