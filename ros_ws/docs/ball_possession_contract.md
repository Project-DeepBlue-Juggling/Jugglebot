# The Ball-Possession Contract — C-POSSESS-1

**Normative.** This document specifies how a "we have the ball" claim is minted:
what a *possession source* must declare, what it may never assert, and where the
one enforcement point is. It is the written third of the repo's contract pattern
(normative statement + one enforcement point + a test that fails without it); the
other two are `jugglebot/ball_possession.py` and `tests/ros/test_ball_possession.py`.

Scope: every consumer of a tracker `CAUGHT`, of the ball-possession latch, and —
when it lands — of the ball-in-cup hand sensor. Sibling contracts:
`ros_ws/docs/catch_arrival_contract.md` (C-CATCH-1),
`ros_ws/docs/levelling_frame.md` (C-LEVEL-1).

## 1. Why this exists — the failure it closes

Every ball op the machine has ever run reported `MISSED`.

On the 2026-07-27 validation sitting all **17** self-tosses and all **18** scored
reloads returned `success = False`, while the operator watched 17 catches land and
a mocap floor census independently confirmed 13 of 16 reload catches. The verdict
was not merely wrong; it was **structurally unreachable**. The gate was

```python
_CAUGHT_MAX_XY_ERROR_MM = 200.0
_CAUGHT_MAX_Z_ERROR_MM  = 150.0
ok = xy_err <= _CAUGHT_MAX_XY_ERROR_MM and z_err <= _CAUGHT_MAX_Z_ERROR_MM
```

and the z half cannot be satisfied by a real catch, because of what the tracker
means by CAUGHT. From `jugglebot/tracking/matcher.py::_check_lifecycle`:

> a ball is declared CAUGHT **because its mocap marker disappeared** around the
> predicted landing time.

So the published `BallState.position` at that instant is not an observation. It is
the Kalman filter's **dead-reckoned free-fall extrapolation** from the last real
sighting, frozen for the ~2 s the terminal track is retained. Measured on bag
`~/Desktop/rosbags/2026-07-27_15-39-38`: positional variance after the first
CAUGHT sample is **exactly 0.000 mm**, on all 60+ CAUGHT tracks in the session.

Free fall splits that extrapolation error by axis:

| axis | error term | measured, 17 self-toss catches |
|---|---|---|
| z | `\|v_z\|·dt + g·dt²/2` | **305 – 1007 mm** |
| xy | `\|v_xy\|·dt` | **0.30 – 3.88 mm** |

It could never have passed: the *tightest* real catch in the session drops
**305 mm**, twice the bound. **z carries the artefact; xy carries the
information.**

> *(An earlier draft added "the 150 mm bound corresponds to `dt ~ 0.037 s` — under
> a third of the measured dead-reckoning window (0.029 – 0.163 s)". **Removed
> 2026-07-28: it does not reconcile with the measured drops** and would invite a
> future reader to re-derive a "generous" z bound and reintroduce the defect. At
> the quoted 0.029 s floor and a ~3.4 m/s plane crossing the drop is only ~104 mm,
> i.e. the bound would have *passed* — contradicting the measured 0/17. The
> 0.029 – 0.163 s figure is the constant-KF-velocity window, which is **not** the
> plane-crossing-to-CAUGHT coast the error model needs. The measured drops carry
> the argument on their own; no dt estimate is required.)*

### The class of failure, not the one bound

The defect was not "150 was too small". It was **a plausibility bound applied to
an observable whose error model was never written down**. Had the model above been
stated when the bound was written (`682ddc5`, 2026-07-23), no z bound would exist.

A **second instance of the same class was live in the same expression**, and it
is the one that shows this is a class and not an accident: the `200 mm` xy bound
sat **4.9 mm** below the corrupt-track floor the next sitting measured (204.9 mm)
— a **1.02x** margin against minting a *false* CAUGHT. Both halves were written
the same day, by the same reasoning, from the same absence of an error model; one
was unreachably tight and the other was one dead-reckoned track away from being
unreachably loose. Re-tuning two numbers would have left the third instance to
find on hardware.

C-POSSESS-1 closes the class rather than the two bounds.

## 2. The contract

> **C-POSSESS-1.** A possession claim has **two independent parts**:
>
> - **ARRIVAL** — the ball reached the cup;
> - **RETENTION** — it was still there afterwards (it did not bounce out).
>
> A **possession source** shall report both. For any part it does not positively
> observe it shall report `UNKNOWN`; it shall never report `True`/`CONFIRMED` for
> a part it cannot observe. Every bound a source applies shall be accompanied, in
> the source, by the **error model of the observable it is applied to** — what the
> quantity measures and what error it carries at the instant it is read. An
> observable whose error model makes it non-discriminating is **REPORT-ONLY**: it
> may be logged and reported, and no code may branch on it.

Three consequences, all deliberate:

1. **`z` is REPORT-ONLY, forever, for the tracker source.** It is retained in the
   verdict as `plane_drop_mm` because it is a genuinely useful diagnostic — it
   states how much of a verdict is extrapolation — but branching on it is the
   defect above.
2. **The tracker source reports `RETENTION_UNKNOWN`, always.** Not a hedge: the
   track freezes at CAUGHT (`_associate_confirmed_balls` skips non-`IN_FLIGHT`
   balls, `matcher.py:261`) and is pruned ~2 s later, so **nothing observable
   reaches this source from the terminal track.**

   > **⚠ NARROWED 2026-07-28 (finalize).** This point first claimed, flatly,
   > *"a bounce-out raises no successor track … There is nothing for it to
   > observe"*, on the strength of zero new `/balls` tracks appearing between each
   > 2026-07-27 bounce-out's CAUGHT and its floor arrival. **That measurement is
   > confounded and does not support the general claim.** Those three tracks were
   > mis-associated (§ 4): the real marker was *already* carried by a separate
   > untagged track, so no *new* track could appear — the probe asked a question
   > the corruption had already answered. In a healthy tracker the CAUGHT track
   > goes terminal, stops consuming its markers, and they fall through to
   > `_detect_parabolic` (`matcher.py:220`, `parabolic_min_frames = 3`), which is a
   > real successor channel that was **not** evaluated. Two further retention
   > signals exist and are likewise unevaluated: `/mocap_data` (200 Hz unlabelled
   > markers — the very signal the § 6 floor census uses offline; the coordinator
   > does not subscribe to it today) and the matcher's own CAUGHT precondition,
   > which is *itself* weak retention evidence — `matcher.py:575-582` mints CAUGHT
   > only after ≥ 10 consecutive missed frames, and explicitly lets a still-visible
   > ball continue (*"it might be bouncing"*), so a bounce-out that stays in view
   > never mints CAUGHT at all.
   >
   > So the honest statement is **"unobservable from the terminal track"**, not
   > "unobservable". See § 7 for why a fate term was nonetheless not built here.
3. **`UNKNOWN` retention does not veto a confirmed arrival.** A source that cannot
   see retention must not be able to refuse a catch it *did* see arrive —
   otherwise the contract would re-create the original defect in the other
   direction (a structurally-unreachable `True`). `RETENTION_REJECTED` — a source
   that positively saw the ball leave — does veto.

### 2.1 ARRIVAL is tri-state too — corrected 2026-08-10

The clause above says **both** parts are tri-state, and until 2026-08-10 the code
only made RETENTION so: `PossessionVerdict.arrival_ok` was a bare `bool`, which
forces *"I could not look"* and *"I looked and it did not arrive"* onto the same
value. **That is this contract's own § 1 defect one level down** — a boolean
applied to an observable whose failure modes were never enumerated — and it was
inert only for as long as the sole source was one that always has an estimate in
hand. The hand sensor is the first source that can be genuinely blind (boot before
the first TxSdo reply, a stale reply, an un-anchored bridge clock, a
`Get_Version` gate that has not passed), so the hole became reachable the moment
it landed.

> **C-POSSESS-1.A.** `arrival` is `CONFIRMED` / `REJECTED` / `UNKNOWN`, exactly as
> `retention` is. `arrival_ok` is a **derived projection** (`arrival ==
> CONFIRMED`), never an independently-settable field — so no source can report the
> two inconsistently. `ARRIVAL_UNKNOWN` projects to `False`: blindness is the
> conservative direction and must never mint a catch.

Enforcement point: `ball_possession.PossessionVerdict` (the property, not a
field). Pinned by
`tests/ros/test_ball_possession.py::test_arrival_ok_is_a_projection_not_a_field`.

## 3. Adding a source

A source is any object with a `name` from `ball_possession.SOURCE_*` and a
`judge(ball_xyz_mm, ref_point_mm) -> PossessionVerdict`. Sources are swapped at
one place, `ReloadCoordinatorNode._possession_source`; all three consumers
(`_on_balls`, `_build_observations`, `_build_toss_observations`) go through
`_possession_confirmed`. Pinned by
`tests/ros/test_reload_coordinator_node.py::test_possession_source_is_pluggable_at_one_seam`
(which substitutes a source that does **not** inherit `PossessionSource` — the
seam is duck-typed on purpose, so a sensor source need not import this module's
base class).

**What the seam does and does not buy you — read this before the sensor phase.**
The swap is real, but it is a **filter on a tracker `CAUGHT`, not an origination
point.** All three call sites sit inside an `int(b.status) == _BALL_STATUS_CAUGHT`
guard (`reload_coordinator_node.py:709`, `:793`, `:1029`), and `judge()` takes only
a position and a reference point — no time, no tick. Three consequences the
sensor phase inherits, stated so it does not discover them at the bench:

- **A source cannot originate a possession claim.** If the tagged track never
  reaches `CAUGHT` — 55 of the 139 tracks in the reference capture never do;
  `UNKNOWN` is the other terminal state — the seam is never called, `obs.ball_caught`
  stays False, and the goal runs to the settle deadline and `SAFE_ABORT`s **with the
  ball in the cup**. A sensor holding a ball cannot say so.
- **A source cannot answer late.** `toss_sequencer._step_in_flight` finishes the
  goal on the first confirmed tick, so a retention answer that needs settle time can
  never bind.
- **A source cannot clear the latch.** `_ball_possession` is cleared only in
  `__init__` and the release-evidence branch; a `RETENTION_REJECTED` verdict does not
  reach it.

So the earlier flat claim that "a new source changes no call site" holds only for
the **arrival-filter** role. Making the hand sensor genuinely PRIMARY needs a
tick-driven query with the ball optional, plus a latch-clear path — that is design
work belonging to the sensor phase, not a drop-in.

**The one thing a new source must not forget.** `_possession_confirmed` passes
`_CAUGHT_MAX_XY_ERROR_MM` — the *tracker* source's bound — into `describe()` for
the log line. That is correct only while the installed source is the tracker one.
A source with a different (or no) arrival bound must either supply its own value
there or bring its own `describe`, otherwise every log line quotes a threshold the
verdict was not judged against, and the bench row that compares gate-to-eye is
being read off a number that means nothing. It is a *reporting* wart, not a
verdict one — `judge` is unaffected — but it is the kind that survives for a year
because nothing fails.

### The ball-in-cup hand sensor (`SOURCE_HAND_BALL_SENSOR`) — LANDED 2026-08-10

Installed on the machine 2026-07-28. **It is now the PRIMARY source**
(`ball_possession.HandBallSensorSource`), because it is the only one that can
observe RETENTION: it reads the cup directly and keeps reading it after the catch.
`TrackerArrivalSource` is demoted to the arrival **corroborator** — it still
supplies `arrival_err_mm`, which is the catch-accuracy number the hardware
runbooks score and which the sensor cannot supply.

Whether an affirmative sensor read should gate the possession latch (and therefore
`toss_require_ball_evidence` / `REJECTED_NO_BALL`) was left open here for whoever
validated the sensor. **Decided 2026-08-10 by the operator, who validated the
sensor in situ: yes.** `toss_require_ball_evidence` now defaults `true` and the
precondition is a LIVE sensor read (§ 3.3). The gate the hand-ball-sensor plan
carried — *"the flip is forbidden until Phase 7 validates"* — is **SUPERSEDED** by
that authorisation; Phase 7 steps 4–5 remain open as bench work, not as blockers.
Recorded at the plan's own statement of the gate
(`plans/active/hand-ball-sensor.md` § Notes for collaborators → Out of scope) and
in `plans/active/catch-robustness.md`.

### 3.2 Merging two sources — the tick-driven kind, and the merge rules

The § 3 seam above describes one shape of source: `judge(ball_xyz_mm,
ref_point_mm)`, a **filter on a tracker `CAUGHT`**. That section already recorded
why the sensor cannot be that shape ("a source cannot originate a possession
claim", "cannot answer late", "cannot clear the latch"). The contract therefore
recognises a second kind:

> **C-POSSESS-1.B.** A **tick-driven source** has a `name` and
> `observe(now, landing_t=None) -> PossessionVerdict`, where `landing_t` is the
> caller's *predicted landing instant* in the same clock as `now` (the FSMs'
> `landing_perf`), `None`/NaN meaning "nothing is in the air". The window is
> passed **per query and never latched**: a latched arm/disarm pair outlives its
> goal, and a window that outlives its goal vetoes the *next* ball's arrival with
> a stale "nothing came in" — a lifecycle bug that would otherwise have to be
> guarded in three teardown paths. A source with no `landing_t` answers
> `ARRIVAL_UNKNOWN`, which § 2 consequence 3 forbids from vetoing anything.
> It takes no ball and no reference point: it
> observes the *cup*, so it can answer on a tick when no track exists, before a
> throw, and during a dwell. A tick-driven source MAY additionally expose
> `evidence(now) -> SEATED | EMPTY | UNKNOWN`, which is a **different question**
> from ARRIVAL — "is a ball in the cup right now", not "did one come in around the
> predicted landing".

When both kinds are installed, they are combined at **one** place —
`ball_possession.merge_possession`, reached only through
`ReloadCoordinatorNode._possession_confirmed` — by three rules:

1. **RETENTION is the sensor's, always.** The tracker is forbidden from claiming
   retention (§ 2 consequence 2); there is nothing to merge. Taking anything else
   from the tracker here would re-open § 7's accepted bounce-out trap.
2. **ARRIVAL is the sensor's whenever the sensor observed it (`CONFIRMED` or
   `REJECTED`); the tracker's only when the sensor is `UNKNOWN`.** The sensor
   reads the cup, so it is the one source whose error model does not run through a
   dead-reckoned free-fall extrapolation — that is *why* it is primary, and why a
   valid sensor `ARRIVAL_REJECTED` **vetoes** a tracker `CAUGHT`. That veto is the
   § 7 false-CAUGHT class finally being closed rather than accepted. The fallback
   is not politeness: a blind sensor that refused everything would leave the
   machine strictly **less** capable than before the sensor landed, so the
   degradation path is exactly today's tracker-only behaviour.
3. **`arrival_err_mm` / `plane_drop_mm` stay the TRACKER's, always** — the sensor
   cannot supply them, and § 3's "the one thing a new source must not forget" is
   about exactly this reporting wart.

**The direction of the new failure mode, sized honestly.** A sensor that reads
valid-EMPTY on a genuinely seated ball (a stuck-open switch, a ball resting off the
contact) turns every catch into `MISSED` → `SAFE_ABORT`, i.e. a retract under a
seated ball. That is not a new hazard: it is the behaviour EVERY toss had before
2026-07-28, and § 5 already argues it safe (the cup carries the ball down at
~3.16 m/s² ≪ g). False-MISSED remains the conservative direction.

**Retention deliberately does NOT gate the catch verdict.** § 3 records that a
source "cannot answer late" — `toss_sequencer._step_in_flight` finishes the goal
on the first confirmed tick. Requiring `RETENTION_CONFIRMED` before minting the
catch would hold the terminal open for the whole retention window (1.5 s), an
actuation-timing change on the one path (`RECENTER`/`STAY`) whose safety argument
in § 5 was written against today's timing. So a fresh arrival mints
`RETENTION_UNKNOWN`, which consequence 3 forbids from vetoing, and retention binds
where it can still act: § 3.3.

### 3.3 The ball-evidence precondition is a LIVE read — § 7.1's two edits, landed

§ 7.1 named two coordinator edits the sensor phase must not inherit as "already
done". Both are now done, and this is where they are specified:

1. **The cycle-start precondition is a live source query, not a latch read.**
   `_build_toss_observations` sets `ball_seated` from
   `HandBallSensorSource.evidence(now)`, not from `self._ball_possession`. With the
   gate required:
   `SEATED ⇒ pass`, `EMPTY ⇒ REJECTED_NO_BALL`, `UNKNOWN ⇒ REJECTED_BALL_UNKNOWN`.
   **`UNKNOWN` refuses.** A dead sensor must not silently allow throws — that is
   the fail-open boot default this project explicitly refused to copy from
   BallButler — and it gets its OWN outcome code because an operator who reads
   `NO_BALL` goes hunting for a ball, where the fault is the sensor.
   `toss_require_ball_evidence: false` restores the pre-2026-08-10 unconditional
   pass and is the operator's escape hatch.
2. **The latch can be cleared without release evidence.** A valid `EMPTY`
   clears `_ball_possession`; a valid `SEATED` sets it. The latch is no longer a
   belief that can outlive the ball.

## 4. What this contract does NOT claim

- **It does not fix the tracker.** The split-track corruption is an open
  investigation. Its signature, measured 2026-07-27: every
  `destination='jugglebot'` reload track's Kalman filter is fed by the **wrong
  marker**, so its CAUGHT estimate lands **204.9 – 752.9 mm** from the catch point
  while a *separate*, untagged track carries the real ball. Of those eighteen,
  **13** were real catches, **3** were eye-confirmed bounce-outs and **2** (balls 31
  and 62) are goals whose Butler throw aborted `YAW NOT_SETTLED` so no ball ever
  flew — and the estimates are **indistinguishable** across all three groups
  (bounce-outs 702.6 / 721.1 / 726.4 mm, catches 204.9 – 752.9 mm).

  > **⚠ CORRECTED 2026-07-28 (finalize).** This paragraph first read *"a track
  > whose Kalman filter received **no** measurements at all (its lateral velocity
  > is constant across the whole descent), so its CAUGHT estimate is a pure
  > open-loop projection from the Ball Butler launch state"*. **That is measurably
  > false and it is recorded here because it would have sent the tracker
  > investigation after the wrong defect.** All 18 tagged tracks reach
  > `tracking=CONFIRMED`, which `matcher.py:344-347` sets *only* inside
  > `kf.update(marker)`; and their in-flight `velocity.x` spread is **31.2 – 515.9
  > mm/s**, not constant (an open-loop track would hold the announced launch
  > `vx = 1032.2 mm/s` exactly). The filters are **not starved — they are
  > mis-fed.** The mechanism to start from is the adaptive announced-ball gate at
  > `matcher.py:325-327`, `threshold = base + speed*0.05 + t_since_throw*50`,
  > capped at **400 mm**: it widens with time-since-throw until it can capture a
  > near-stationary marker beside the Butler, and once `_match_announced_balls`
  > promotes on that marker the track is locked to it. Re-verified against the bag
  > during finalize.

- **The 70 mm arrival bound is knowingly under-sized for the RELOAD path, and the
  tracker phase must re-derive it.** Two independent terms push a *legitimate*
  reload catch past it, and the first one is measured, not derived:
  - **dead-reckoning drift.** The reload era's untagged (real-marker) tracks reach
    CAUGHT at **34.4 / 34.9 / 37.6 / 68.4 mm** (ids 57 / 33 / 69 / 15) — so a
    genuine reload catch already sits **1.6 mm inside** the bound, a **1.02x**
    margin. That is the same margin, and the same defect shape, that § 1 condemns
    in the old 200 mm bound.
  - **catch reach.** `JB_TRAJ_CATCH_REACH_ENVELOPE_MM` is **80 mm** and the
    reference point does not move with the reach, so a reached-to catch can read up
    to the full envelope even with a perfect estimate. The node's own levelling
    gate meanwhile treats the **35 mm** `GEOM_HAND_RADIUS_MM` as the cup — three
    numbers (35 / 70 / 80) for one geometry, which is worth resolving with the
    operator rather than picking a fourth.

  It is **not re-tuned here**, deliberately: re-tuning needs data from a *fixed*
  tracker, and the only number available today is the 204.9 mm corrupt floor — an
  artefact of the very bug being fixed. Sizing a bound against a bug is this
  contract's own § 1 defect, one level down. The risk is inert until the tracker
  lands (every tagged reload track is refused at 204.9–752.9 mm whether the bound
  is 70 or 200), and `landing_position.xy` was measured as an alternative and is
  **worse** (ids 33/69 read 140.0/120.3 mm there). Pinned by
  `tests/ros/test_ball_possession.py::test_the_measured_reload_band_sits_against_the_bound`.
- **It therefore does not restore the reload verdict.** With no real evidence in
  the tagged track, the only honest verdict is refusal, and the reload path
  continues to read `MISSED`. That is the contract working, not failing: it
  refuses to mint a claim from a signal that carries none. What the contract *does*
  restore is the **self-toss** verdict, where the tracks are measurement-driven.
- **It does not change actuation.** `toss_require_ball_evidence` stays `false`, so
  `ball_seated` remains unconditionally True and no goal is refused that was not
  refused before. It does change *when a successful toss terminates* — see § 5.

  > **⚠ SUPERSEDED 2026-08-10.** This bullet described the state at the 2026-07-28
  > landing and is kept because § 5's safety argument is written against it. The
  > flip is now made (§ 3.2 / § 3.3): the default is `true`, and CHECKING refuses
  > `REJECTED_NO_BALL` on a valid-empty cup and `REJECTED_BALL_UNKNOWN` on a
  > sensor that cannot answer. Both are pre-motion rejects — nothing has moved and
  > nothing is armed at CHECKING — so the refusals add no actuation, they only
  > subtract goals. § 5's timing analysis is untouched.

## 5. The one behavioural consequence, and why it is safe

The toss and reload FSMs already branch on `obs.ball_caught`
(`toss_sequencer._step_in_flight`, `reload_sequencer._step_in_flight`): a
confirmed catch finishes the goal immediately with `ACTION_RECENTER`, where an
unconfirmed one runs to the settle deadline and finishes with `ACTION_SAFE_ABORT`.
That branch has been dead on hardware for the machine's whole life. Un-breaking
the verdict makes it live, and it is a genuine actuation change on every
successful self-toss:

| | before (verdict always False) | after |
|---|---|---|
| terminal instant | landing + `catch_confirm_window_s` (0.70 s) | the CAUGHT tick — measured landing + **0.202 – 0.442 s**, median **0.209 s**, over the 17 self-tosses |
| terminal action | `SAFE_ABORT` — retract the hand, lower latch, `go_home` | `RECENTER` — lower latch, `go_home`, **no retract** |

Both deltas were checked against the same bag before the change landed:

- **The catch stroke has already finished.** Hand `pos_meas` at the CAUGHT instant
  is within **±0.045 rev** of the retract target on all 17 self-tosses (range
  `−0.045 … +0.026`), where 0.3 s earlier 7 of the 17 were still descending
  through **0.30 – 3.10 rev**. So the earlier terminal cannot interrupt a moving
  hand, and skipping the retract leaves the hand where the catch trajectory
  already put it — inside the `±0.5 rev` bottom park band the *next* goal's
  `hand_parked` precondition needs. Over the whole 3 s after the CAUGHT tick the
  worst excursion is **0.069 rev**, a **7.2x** margin. Chained Toss → Toss is
  unaffected.
- **The `catch/prime_hold` release moves 0.26 – 0.50 s earlier**, re-opening
  `catch_coordinator`'s auto-prime while a ball rests in the cup. This is not a new
  hazard: today's `SAFE_ABORT` path releases the same hold at the settle deadline
  with the ball equally seated, and it releases it *after* retracting the hand to
  the same ~0.0 rev the catch trajectory already reached. Same state, up to half a
  second earlier.

`go_home` also fires up to 0.5 s earlier, with the ball seated in a cup rigidly
mounted on the platform, along a profiled trajectory. Same class as today.

**Two residuals on this path, both un-measured and both watched at the bench
rather than argued away here.**

1. **`RECENTER` is the one terminal with no telemetry-verified hand step.**
   `_safe_abort` retracts and *verifies* the retract from telemetry ("every step's
   failure is LOUD here: this is the safing path"); `_recenter` publishes
   `armed False` and goes home. Its safety therefore rests entirely on the verdict
   being true. If a catch stroke's dispatch were silently lost — this project's hand
   acks are known to lie ~59% **in both directions**, which is why every hand ladder
   is telemetry-verified — the hand would stay at the ~9.96 rev prime while the gate
   minted CAUGHT from tracker xy alone, and the next Toss would be rejected
   `HAND_NOT_PARKED`. A loud reject, not a hazard, which is why it is watched
   (runbook **POSS-2.1** / **POSS-2.2**) rather than coded around. Note the
   observation that would corroborate it is already assembled one function away:
   `_build_toss_observations` has `hand_pos`/`hand_fresh` in scope at the point it
   calls `_possession_confirmed`.
2. **The ball's settle time before the platform moves is not measured.** The
   evidence offered above is hand `pos_meas`, which says where the *hand* is, not
   whether the *ball* has come to rest. A caught toss now begins `go_home`
   0.26–0.50 s earlier, i.e. essentially at the instant the catch stroke arrests the
   ball, where before it had the full 0.70 s settle window plus a retract. "Same
   class as today" is an assertion about a path that has **never executed on
   hardware**. Runbook row **POSS-2.4** scores ball retention through the `RECENTER`
   `go_home` from mocap and is the row that retires this sentence. If the operator
   prefers zero new risk on the first powered run, the decoupling is cheap: resolve
   the verdict at the CAUGHT tick but hold `RECENTER`'s `go_home` until the old
   settle deadline — that lands the reporting fix with no timing change at all.

## 6. Enforcement and tests

| part | where |
|---|---|
| normative statement | this document |
| enforcement point | `jugglebot/ball_possession.py` — `TrackerArrivalSource.judge`, `HandBallSensorSource.observe` and `merge_possession`, all reached only through `ReloadCoordinatorNode._possession_confirmed` |
| enforcement point (§ 3.3, the precondition) | `ReloadCoordinatorNode._build_toss_observations` — the ONE live `evidence(now)` read, consumed by `toss_sequencer._step_checking` |
| tests | `tests/ros/test_ball_possession.py` (the verdict surface, on 2026-07-27 fixtures + the 2026-08-10 sensor timings), `tests/ros/test_reload_coordinator_node.py` (the node seam) and `tests/ros/test_toss_coordinator.py` (the CHECKING refusals) |
| bag replay | `tools/probes/hand_sensor_verdict_replay.py` — replays `/hand_telemetry` + `/throw_announcements` through the PRODUCTION verdict logic and prints per-throw labels; `tests/ros/test_hand_sensor_replay.py` runs it on a committed fixture cut and skips when `~/Desktop/rosbags` is absent |

The tests are built from the sitting's **measured** values, not synthetic ones:
`tests/ros/possession_fixtures.py` carries all 35 tagged-track CAUGHT estimates
(17 self-toss, 18 reload) with their ids, so a future change is scored against the
same ground truth. Regenerate them with
`tools/probes/possession_verdict_bag_check.py --emit-fixtures`.

**Why the 17 self-toss fixtures may be asserted as TRUE catches**, which is the
load-bearing premise of the whole positive set and so was checked independently
rather than inherited. Two arguments, both from the capture:

1. The 17 tosses ran **consecutively with zero interleaved reload goals** (all 18
   Butler-thrown tracks precede all 17 self-toss tracks in the bag). A toss throws
   the ball the previous toss caught, so a miss would have stranded the sequence.
2. A **mocap floor census** over the whole self-toss era (315–600 s of
   `2026-07-27_15-39-38`, unlabelled `/mocap_data` markers below z = 250 mm) finds
   **zero** positions that are not already static in the first 5 s — the only three
   are structural and never move. No ball rested on the floor at any point. This is
   the same method that produced the sitting's reload numerator, and it is the
   argument that does not depend on assuming the operator never hand-reseated a
   ball.

Two mutations must go red (checked 2026-07-28, see the logbook entry):

- delete the arrival bound (`ok = True`) — the 18 reload fixtures flip to
  confirmed;
- make `TrackerArrivalSource` return `RETENTION_CONFIRMED` — the retention pin
  fires. This is the mutation that matters most, because a source that claims
  retention it cannot see is the exact shape of the original defect.

## 7. The fate term that was specified and NOT built — an explicit deviation

**This is a deviation from the phase's design brief and it is recorded here rather
than left to be inferred from the code.** The brief specified a two-part verdict —
arrival **AND** a post-arrival persistence/fate term — precisely so the gate would
not carry the false positive the 2026-07-27 sitting had already demonstrated (a
ball that enters the cup, is minted CAUGHT, and then bounces out). **What shipped
is arrival-only.** `RETENTION_UNKNOWN` never vetoes (§ 2 consequence 3), no
production code branches on `retention`, and
`test_the_bounce_out_trap_is_pinned_at_the_true_arrival` pins `arrival_ok = True`
for all three measured bounce-outs at their true mocap arrival. So the trap is
**accepted and documented, not closed.**

> **✔ CLOSED 2026-08-10 — but read HOW, because it is not how this section
> anticipated.** `HandBallSensorSource` observes retention directly, and it is
> `RETENTION_REJECTED` on a bounce-out (§ 3.2 rule 1). It does **not** close the
> trap by delaying the catch verdict — § 3 records that a source cannot answer
> late, so the retention answer is not available at the tick that mints the catch,
> and the fresh arrival still carries `RETENTION_UNKNOWN`. What it closes is the
> *consequence*: the possession latch a bounce-out used to leave standing is now
> cleared by the sensor's live `EMPTY` (§ 3.3 edit 2), and the next cycle's
> precondition is that same live read rather than the stale latch (edit 1). So the
> ONE tick that reports CAUGHT over a just-bounced ball survives as a **reporting**
> error; the actuation consequence — cycle N+1 firing an empty stroke — is gone.
> `stay_at_pose_on_caught` keeps the hand where it is either way. The rejected
> candidates below stay rejected on their own merits.

Why it was not built, with the failure mode each rejected option carries:

| candidate | why not |
|---|---|
| post-CAUGHT track evolution | Genuinely unobservable — the track is frozen and pruned (§ 2). This one is settled. |
| successor track via `_detect_parabolic` | **Not evaluated.** The measurement that appeared to rule it out is confounded (§ 2 note). It needs a healthy tracker to test against, which is the open investigation. |
| `/mocap_data` floor census, online | Real, and the strongest candidate. Costs a new 200 Hz subscription **and** a verdict delay of ~0.5 s spent inside `catch_confirm_window_s` (0.70 s) and the `_toss_deadline_s` budget — a budget whose timeout path is `SAFE_ABORT`, i.e. **it retracts the hand under a seated ball.** Buying retention evidence by moving the toss closer to a retract-under-ball timeout is a bad trade on the one path where the cost is a hazard. It also cannot see a ball that stays on the platform. |
| lateral velocity at CAUGHT | **Measured and rejected**, and this one nearly shipped. It separates the 2026-07-27 data 75x (self-toss 0.8–7.0 mm/s vs reload 530–1124 mm/s) — but that is *geometry*, not retention: a self-toss arrives vertically so its lateral velocity is ~0 by construction, while a **legitimate** reload catch arrives at ~1 m/s and the same threshold would refuse it. It would have looked validated on 17 fixtures and silently false-negatived every reload catch the moment the tracker was fixed. |

**The residual exposure, sized honestly.** A false CAUGHT needs a bounce-out **and**
≥ 10 consecutive missed frames spanning the CAUGHT instant — the matcher lets a
still-visible bouncing ball continue (§ 2 note), so the tracker's own CAUGHT
precondition already excludes the visible half of the failure mode. The 2026-07-27
capture does **not** demonstrate the reachable case: all three bounce-outs reached
CAUGHT only because their tagged tracks were mis-associated, and their own
estimates read 702.6–726.4 mm out, so an arrival gate refuses them today for the
wrong reason. Consequence if it does fire: an inflated catch-rate numerator, a
possession latch set over an empty cup, and `RECENTER` instead of `SAFE_ABORT` —
reporting harm plus a skipped retract, on a hand the catch stroke has already
parked (§ 5). No commanded magnitude changes.

### 7.1 A continuous SESSION multiplies this exposure — added 2026-07-29

> **✔ BOTH EDITS LANDED 2026-08-10** — see § 3.3, which is now the normative
> statement of what they do. This section is kept verbatim below because it is the
> *specification* the edits were built against, and because its trace of the
> shipped code is what a future reader needs in order to check that the edits
> actually went where the gap was. Read § 3.3 for what the code does today.

`TossContinuous` (`plans/active/single-ball-toss.md` Phase F) runs `num_throws`
toss-catch cycles from one goal, with a dwell between them. **Between a catch and
the next throw the ball sits in the cup with the platform holding, and NOTHING
re-verifies possession there.** The verdict this contract mints is ARRIVAL-only
(§ 7 above), so a post-CAUGHT bounce-out during a dwell leaves `_ball_possession`
set and cycle N+1 fires an empty stroke. That stroke is benign — it is exactly the
no-ball dry-trace case, a normal kind-0 stroke into an empty cup — but the verdict
is wrong, and a session repeats the exposure `num_throws` times instead of once.

**`stop_on_miss` does not close it**, and it is worth being explicit about why: the
flag stops the session on a NOT-caught verdict, and a bounce-out happens *after* a
CAUGHT verdict. The two failures are disjoint.

**The seam for the sensor's ARRIVAL verdict is already in place and needs no wire
change** — `_possession_confirmed` → `_possession_source` is where a
`BallInCupSource` drops in, and the session FSM and the action are untouched by
it. **RETENTION needs two coordinator edits on top of that, and they are named
here so the sensor phase does not inherit the belief that the work is already
done.** Traced 2026-07-29 against the shipped code:

1. **The cycle-start precondition is a LATCH READ, not a source query.**
   `_build_toss_observations` sets
   `ball_seated = bool(waiver or possession or not JB_OP_TOSS_REQUIRE_BALL_EVIDENCE)`
   where `possession` is a plain read of `self._ball_possession`. The one
   in-observation call to `_possession_confirmed` is inside `if announced_id is
   not None`, and `_build_toss_cycle` resets `_announced_ball_id = None` for every
   cycle — so at cycle N+1's CHECKING tick **no code path consults
   `_possession_source` at all.** Closing retention needs a LIVE sensor read
   there.
2. **Nothing can clear the latch during a dwell.** `_on_balls` only ever SETS
   `_ball_possession` True; the single clear path is gated on
   `throw_dispatched`, which `_build_toss_cycle` resets False each cycle. And
   `PossessionSource.judge(ball_xyz_mm, ref_point_mm)` is only reachable from a
   `/balls` CAUGHT message, while the tracker prunes the terminal track ~2 s
   after CAUGHT and the dwell floor is 4.10 s (default 6.0 s) — so for most of
   every dwell nothing calls the source even in principle. Closing retention
   needs a path that can clear `_ball_possession` **without release evidence**.

Today the gap is additionally masked by
`jugglebot_operational.toss_require_ball_evidence` defaulting `false` (there is no
sensor, so a tracker-derived belief must not be able to refuse a physically-loaded
ball) — flipping that default is part of the same sensor phase, not this one.

Operator-facing statement: runbook § SECTION CONT's *"What this section does NOT
cover"* tells the operator to **watch the cup between cycles by eye and score a
lost ball as a finding even when the outcome line says CAUGHT.**

**Who closes it.** The ball-in-cup hand sensor (§ 3), which observes retention
directly and continuously. Until then the operator-facing statement of this gap is
runbook row **POSS-1.2** in `tests/hardware/session_anomaly_fixes.md`, which is
deliberately scoped so that an arrive-then-leave CONFIRMED is a **REPORT** row that
sizes the sensor work, and only a CONFIRMED on a ball that never arrived is an
ABORT. Do not tighten that row into an ABORT before the sensor lands: it would
abort a healthy sitting on behaviour this contract specifies.

**Resolution, 2026-08-10.** Edit 1 landed as the live `evidence(now)` read
(§ 3.3.1); edit 2 landed as the sensor's `EMPTY` clearing `_ball_possession`
(§ 3.3.2); the default flipped to `true` under the operator's in-situ validation
(§ 3). The residual is the ONE tick that still reports CAUGHT over a ball that
bounced out immediately afterwards — a reporting error the retention term records
as `RETENTION_REJECTED` in the very next verdict but cannot retract, because the
goal has already finished. POSS-1.2 therefore stays a REPORT row and stays
un-tightened: it is now scoring a known, bounded reporting residual rather than an
un-sized actuation gap.
