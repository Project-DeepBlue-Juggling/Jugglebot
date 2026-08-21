---
title: z=30 solve-failure root cause — motor_pos=None after CAN drop evades both telemetry watchdogs (Finding 3 resolved)
type: investigation
date: 2026-05-18
status: tuned
#
# Status ladder (for investigation entries):
#   open         — nothing done yet
#   in-progress  — diagnosis done, fix/verification still ongoing
#   tuned        — symptom scoped to this entry addressed + verified, open sibling
#   resolved     — every symptom in scope addressed and verified
#
phase: "hardware-bringup"
related_plan: "hardware-bringup.md"
related_entries:
  - 2026-05-18-hardware-bringup-t2b1-t2a1-cascade-validation
  - 2026-05-12-tier3a-fuzz-bugfix
  - 2026-05-11-tier2b-hardware-plant-telemetry-ff
sessions:
  - mpc_20260518_200137.csv
files_changed:
  - controller/hardware_plant.py
  - controller/PLANT_INTERFACE_CONTRACT.md
  - tests/sim/test_hardware_plant_failure_paths.py
  - tests/sim/_hardware_plant_stub.py
  - logbook/2026-05-18-hardware-bringup-t2b1-t2a1-cascade-validation.md
commits:
  - c8a5dd4
subsystem:
  - controller
  - hardware-plant
  - safety
  - ipc
tags:
  - hardware-bringup
  - safety
  - watchdog
  - telemetry-stale
  - telemetry-frozen
  - investigation
  - contract-gap
  - open-question
  - hypothesis-withdrawn
---

# z=30 solve-failure root cause — motor_pos=None after CAN drop evades both telemetry watchdogs (Finding 3 resolved)

## Summary

The cascade-validation entry's
[Finding 3](2026-05-18-hardware-bringup-t2b1-t2a1-cascade-validation.md)
("MPC cannot solve at z=30 … 1769+ consecutive failures from session
start … never produced a real optimized solution … z=30 may be
numerically pathological") is **factually refuted**. z=30 is a
healthy, solvable pose — the same session held it for 16 s at 100%
solve success. The real root cause is a CAN/encoder telemetry dropout
during the T-H-T2a-1 fault-injection that evades **both**
telemetry-staleness watchdogs, leaving the platform blind for 68 s
while the MPC is fed a self-inconsistent state IPOPT cannot solve.

This entry **resolves Finding 3** (root cause found) but leaves a
**safety contract gap OPEN** pending fix-direction discussion with
the user — the Fix section is intentionally empty. This is **not a
Plan 2 blocker**: the cascade PASSes stand. This entry **extends**,
and does not contradict, the cascade entry — that entry measured only
the *final* ESTOP at t≈99 s and never inspected the 68 s window before
it.

## Symptoms

The cascade-validation entry's Finding 3 claimed, verbatim:

> `mpc_20260518_200137.log` shows `MPC solve failed (N consecutive):
> Maximum_CpuTime_Exceeded` from **line 11 (session start, right
> after "MPC controller initialised")** through to the cascade —
> **1769+ consecutive failures**, every tick, during a *static z=30
> hold*. `Solve time: mean=29.0 ms, max=101.6 ms` vs a ~25 ms budget.
> The platform was held **entirely by the fallback path** the whole
> session; it never produced a real optimized solution. … it means
> **`z=30` may be a numerically pathological / near-infeasible
> operating pose** for this MPC.

What was actually observed in `temp/logs/mpc_20260518_200137.{log,csv}`
(the T-H-T2a-1 20:01 session — 2979 records, 99.0 s wall-clock,
`dt_median` 30.65 ms; all numbers below verified this session
2026-05-18):

- The session **did** solve z=30 cleanly — for 16 continuous
  seconds, at 100% success, before any failure run began.
- The "from session start" / "1769+ consecutive" claim is an
  artefact of grep-counting log lines whose counter resets on every
  success (see Diagnosis).
- The all-tick-failing window is bounded and *follows* a clean hold;
  it begins at a single hard latch, not at session start.
- The "z=30 numerically pathological" hypothesis does not survive
  the in-data control (Discussion §1).

## Diagnosis

> **Line-number note:** `hardware_plant.py` references in this section
> point at the **pre-P5-fix** file. The Fix (below) inserted the P5
> arm into `get_state()` and shifted everything after it down ~57
> lines (e.g. the `telemetry_stale` ESTOP moved `:637`→`:656`, the
> frozen-content gate `:658`→`:715`). Post-fix locations are in the
> Fix section; the refs here are a deliberate retrospective record of
> the code as it was when the failure occurred.

Evidence chain from the CSV/log of `mpc_20260518_200137` (all numbers
verified this session, 2026-05-18):

### Phase table — z=30 solves cleanly until a hard latch

| Phase | Window | Solve success | Solve mean | IPOPT iters | actual_ext range |
|-------|--------|---------------|------------|-------------|------------------|
| Move 54→30 | t = 0–9.5 s | 97% | 21.1 ms | 2.5 | 27–155 mm |
| **z=30 static hold** | **t = 10–25 s** | **100%** | **13.8 ms** (p95 25.0) | **1.58** | **26.75–27.15 mm** |
| z=30 "failure" | t = 32–99 s | 5% | 30.2 ms | 0.17 | **exactly 0.00** |

The decisive row is the middle one: **16 s of 100%-success solves at
exactly the pose Finding 3 called pathological**, 1.58 IPOPT
iterations per solve (a trivially-easy problem), 27 mm of stroke
margin. A pose cannot be numerically pathological *and* solve at
100% / 1.58 iters / 13.8 ms for 16 s while the commanded pose never
changes.

### z=30 solve time is healthy, not pathological

Reference run `mpc_20260518_195035.log` (z=170, same Jetson, same
launch load): solve `mean=9.4 ms`, `p95=12.2 ms`. The z=30 healthy
hold ≈ **13.8 ms** is the same order of magnitude — ~1.5× the z=170
reference, **not** the 3× "near-infeasible" inflation Finding 3
implied. (The 30.2 ms "failure"-window mean is the *latched failure
state*, not z=30's intrinsic cost; see the latch below.)

### "1769 consecutive" is one bounded run, not a from-start run

The `MPC solve failed (N consecutive)` counter **resets to 0 on every
successful solve**. The 1769-figure is one *unbroken* run that starts
at **t ≈ 30.7 s**, preceded by ~1200 mostly-successful ticks. The
early `(1 consecutive)` log lines were **interspersed** single-tick
CPU-budget clips under full `jugglebot_launch.py` launch load — the
counter kept resetting between them. Grep-counting all 1801
`MPC solve failed` lines and reading "from line 11" conflated *total
failures over the session* with *consecutive failures from start*.
The diagnose flag `Near lower stroke limit: -5.0 mm (min extension
0.0 mm)` is likewise an **artefact**: min-extension is polluted by
the all-zero frozen tail; recomputed over the healthy hold window the
stroke margin is **~27 mm**.

### The latch — telemetry loss, not mechanical bottoming-out

At CSV idx **1034, t ≈ 30.70 s**, all six `actual_ext_*` columns
snap from ≈26.7 mm to **exactly 0.000000 in a single 25 ms tick** and
stay frozen for the remaining 68 s. `actual_pose_z` simultaneously
freezes at **exactly 30.036541** and never changes again.
Bit-identical zeros across all 6 legs in one tick is **physically
impossible** for real mechanical motion (no leg-to-leg variance, no
ramp, no settling) ⇒ this is **telemetry loss**, not the platform
bottoming out against its lower stroke limit.

### Code trace — motor_pos=None drives exactly this signature

`motor_guard.py:351,1078,1128` publish `msg['motor_pos'] = None` on
the fault / no-feedback paths (the telemetry **publisher stays alive
at full rate** after the CAN drop — only the payload goes invalid).
In `HardwarePlant.get_state()`:

- `motor_pos is None` →
  [`state.leg_extensions_mm.fill(0.0)`
  (hardware_plant.py:564)](../controller/hardware_plant.py) ⇒
  `actual_ext` becomes exactly `0.0` on every leg.
- `ik_ext_mm = None` → the FK branch is skipped → pose ← frozen
  `_last_measured_pose`
  ([hardware_plant.py:630](../controller/hardware_plant.py)) ⇒
  `actual_pose_z` frozen at its last good value (30.036541).

This is the exact CSV signature observed at the latch.

### Watchdog A — `telemetry_stale` (recv-time based) correctly silent

[hardware_plant.py:637](../controller/hardware_plant.py) escalates on
`telem_age > _telem_stale_estop_s` (0.5 s). `telem_age` is keyed on
`_last_telem_recv_time`, which refreshes whenever `drain_count > 0`
([:522–523](../controller/hardware_plant.py)). Through the entire
68 s window `zmq_drain_count` stayed **> 0** (mean 14–18 per tick,
**0% zero-drain ticks**) — frames kept arriving, the payload was just
invalid. So `telem_age` stayed fresh and Watchdog A was **correctly
silent**. It fired only at **t ≈ 99 s** (log line 1950,
`telem_age=0.505 s`) when frames *truly* stopped — which is precisely
the ESTOP the cascade entry recorded as the T-H-T2a-1 PASS.

### Watchdog B — `telemetry_frozen` (content based) gated out by its own None-check

[hardware_plant.py:643–693](../controller/hardware_plant.py) is the
persistent-stale-contents detector, **explicitly designed for the
"publisher-alive-but-dead after CAN drops"** scenario (its own
comment: *"the publisher-alive-but-dead signature (e.g. motor_guard
holding its last feedback frame after CAN drops)"*). But its body is
gated behind:

```python
if motor_pos is not None and drain_count > 0:   # hardware_plant.py:658
```

With `motor_pos = None` the detector body **never executes** — the
frozen-content counter never advances, the warn never logs, the
`telemetry_frozen` ESTOP never escalates. Confirmed by grep of
`mpc_20260518_200137.log`: **no** `motor_pos frozen` warning and
**no** `telemetry_frozen` ESTOP line appears anywhere in the session.

### Net effect — 68 s blind, zero safety escalation

For 68 s the MPC was fed `leg_ext = [0,0,0,0,0,0]` (from the None
branch) against a **frozen FK pose z ≈ 30** (from
`_last_measured_pose`). That state is **self-inconsistent**: zero leg
extensions do not correspond to a z≈30 platform pose under FK. IPOPT
cannot reconcile the contradiction within the 25 ms budget, so every
tick latches `Maximum_CpuTime_Exceeded`. The "z=30 can't solve"
symptom is entirely downstream of the telemetry loss — and the
platform ran **blind for 68 s with zero safety escalation** until
frames truly stopped at t≈99 s.

## Discussion

### 1. Hypothesis withdrawn — "z=30 numerically pathological"

The cascade entry's lead candidate — *"ill-conditioned Jacobian near
the lower workspace boundary → IPOPT iteration blow-up → CPU-time
cap"* — is **dead**. The decisive evidence is an **in-data control
stronger than any sim repro**: the *same session*, on the *same
Jetson*, under the *same full-launch load*, held *exactly z=30* for
**16 continuous seconds at 100% solve success, 13.8 ms mean, 1.58
IPOPT iterations, 27 mm stroke margin**. A pose that is numerically
pathological cannot solve trivially for 16 s and then "become"
pathological while the commanded pose never changes. The pose did
not change; the *telemetry* did.

The originally-planned sim repro and offline Jacobian-conditioning
probe were judged **redundant and not run**. A 16 s clean *hardware*
hold at the exact pose, exact load, exact solver build is strictly
more authoritative than a sim reproduction: it removes every
sim-vs-hardware modelling gap (real solver build, real Jetson
contention, real msgpack telemetry path) as a confound. Running the
sim probe would have answered a question the hardware data already
answered, at the cost of implying the hardware evidence was
insufficient. The control *is* the experiment.

### 2. Why the original framing misled

Two compounding instrument errors produced the "pathological z=30"
narrative:

- **Counter-reset blindness.** `MPC solve failed (N consecutive)`
  resets on every success. Grep-counting the 1801 log lines and
  reading "from line 11" silently substituted *total failures over
  99 s* for *consecutive failures from session start*. The two are
  wildly different here: ~1200 mostly-successful early ticks
  (interspersed single-tick clips) followed by one 68 s latched run.
- **Frozen-tail pollution.** The diagnose engine's `Near lower
  stroke limit: -5.0 mm (min extension 0.0 mm)` flag derived its
  min-extension from a column whose last 68 s are all-zero
  *telemetry-loss* values, not real strokes. Over the healthy hold
  the margin is ~27 mm — comfortably inside the envelope. The flag
  is an artefact of the very bug under investigation, not
  independent evidence for it.

This is worth recording because the next reader who greps the same
log will see the same 1801 lines and the same stroke flag, and will
re-derive the same wrong conclusion unless the counter-reset and
tail-pollution traps are written down.

### 3. The failure class, not the instance

Per CLAUDE.md *"climb one level of abstraction before you fix"*: the
instance is "z=30 didn't solve in this one session". The **class**
is:

> *Any fault that keeps the telemetry publisher emitting frames while
> `motor_pos` is None (or otherwise sentinel-invalid) evades **both**
> staleness watchdogs for the entire lifetime of the publisher —
> `telemetry_stale` because the frames are recv-fresh, and
> `telemetry_frozen` because its detector body is gated on
> `motor_pos is not None`.*

The `telemetry_frozen` detector was *built for* the
publisher-alive-but-dead-after-CAN-drop scenario. Its own
`motor_pos is not None` gate (added to suppress false positives from
zero-drain re-reads and de-energised start-up frames) carves out a
blind spot for the **None variant of exactly the scenario it was
designed to catch**. The detector that should have fired is silenced
by a guard intended to make it *more* reliable. That is the precise
shape of the gap.

This is a **plant-state-validity** gap. It is *distinct from*:

- **K1–K6** (`REFERENCE_LAYER_CONTRACT.md`) — that contract governs
  *what trajectory the MPC tracks* (reference feasibility), not
  whether the *measured* plant state is valid. Orthogonal layer.
- **P1–P4** (`PLANT_INTERFACE_CONTRACT.md`) — P4 derives staleness
  *thresholds* from `control_dt`; it says nothing about telemetry
  *validity*. P3's trusted-callee boundary governs `command()`
  inputs, not `get_state()` outputs. No existing P-invariant covers
  "the payload arrived fresh but is explicitly invalid".

The gap is a missing invariant on the *output* side of
`get_state()`, adjacent to but not covered by any landed contract.

### 4. Why this extends, not contradicts, the cascade entry

Both of the following are simultaneously true:

- The cascade entry's *"T-H-T2a-1 PASS, ESTOP at telem_age=0.505 s
  vs 0.5 s threshold (5 ms late), MPC clean-exit"* is **correct**.
- The platform ran blind for 68 s with zero safety escalation
  **before** that ESTOP.

There is no contradiction: the cascade entry measured the **final
ESTOP at t≈99 s**, when frames *truly* stopped (the genuine total
frame-loss the `telemetry_stale` watchdog is designed for, and which
it handled correctly to spec). It never inspected the 68 s
present-but-invalid window that *preceded* total frame loss, because
that window produced no watchdog line to measure. The watchdog the
cascade entry validated works correctly **for total frame loss**; it
is blind **to present-but-invalid frames**. The cascade PASS and
Plan 2's hardware gate are **not invalidated** — they answered the
question they posed (does `telemetry_stale` fire on real CAN loss);
this entry answers a question they did not pose (is the platform
safe in the window before total frame loss).

### 5. z=30 verdict for the hardware-bringup plan

z=30 is a **safe, solvable operating pose**. The 16 s clean hold at
100% / 1.58 iters / 27 mm margin is the proof. The bringup plan's
open "what is the lowest safe pose / is z=30 numerically OK" question
is **moot** — z=30 itself is fine; nothing about the pose needs to
change.

One secondary caveat is worth recording for the plan: under the
*full* `jugglebot_launch.py` + rosbag-record + test-script +
diagnostic-shell load, the solve-time margin **thins at low z**. The
move-down 54→30 clipped the 25 ms budget on ~3% of ticks (mean
21.1 ms), plus a ~57 ms first-sample cold solve. This is real but
**secondary** — it explains the operator's "jittery, not smooth"
move-down observation and is *not* a blocker. The bringup plan
should simply **expect ~20 ms move-phase solves at low z under full
launch load** (vs ~9 ms unloaded at z=170) and budget accordingly.

## Fix

Scope **B-additive** (operator decision, 2026-05-18 — see Discussion
§6 / Open Questions): add the new `telemetry_invalid` enforcement arm
**only**; leave the landed Tier-2a `telemetry_frozen` content detector
and its T-U-T2a-6/7 tests untouched. The blast-radius grep
(`telemetry_frozen` / `_FROZEN_MOTOR_POS*` / `_frozen_motor_pos*`)
showed the frozen-content detector is a test-pinned, logbook-documented
contract from [2026-05-11-tier2a-hardware-plant-fk-degradation.md](2026-05-11-tier2a-hardware-plant-fk-degradation.md);
the single-enforcement-point variant (B-full) would have rewritten it
for marginal benefit. P5 unifies the *class* at the contract-document
layer instead.

Implemented as the canonical contract triple (doc + one enforcement
point + regression test):

**1. Normative — `controller/PLANT_INTERFACE_CONTRACT.md`.** Title
`P1–P4`→`P1–P5`; new **P5 — Telemetry validity** section naming the
invariant *"the MPC MUST NOT run blind for longer than the budget
after arming"* with its three cooperating arms (no-frames →
`telemetry_stale`; frames-unchanging → `telemetry_frozen`;
frames-no-usable-payload → `telemetry_invalid` ← new). Enforcement-
table P5 row; P5 added to the "Implementations MUST" list; the
Phase-landing paragraph records P5's 2026-05-18 origin and that the
Tier-2a arms were deliberately not refactored.

**2. Enforcement — `controller/hardware_plant.py`.**
- `__init__`: `_last_valid_motor_pos_time: float | None = None` +
  `_telem_invalid_warned = False` (edge-triggered), beside
  `_last_telem_recv_time` / `_telem_stale_warned`.
- `get_state()` `motor_pos is not None` branch: refresh
  `_last_valid_motor_pos_time = now` and clear
  `_telem_invalid_warned` (so a None→valid recovery before budget
  does not escalate).
- `get_state()` immediately after the existing `telemetry_stale`
  ESTOP block: if `_fk_ever_succeeded` and a usable `motor_pos` is
  older than `_telem_stale_estop_s`, `estop('telemetry_invalid')`;
  else once-only WARN past half-budget. The `not _estop_requested`
  guard keeps `telemetry_stale` authoritative on a true total
  frame loss (it runs first and latches). No per-tick allocation
  (preserves P1/W4c + the HOT_LOOP zero-alloc contract).

**3. Regression test —
`tests/sim/test_hardware_plant_failure_paths.py::TestP5TelemetryValidity`**
(T-U-P5-1..4) + new `install_motor_pos_none_pump` helper in
`tests/sim/_hardware_plant_stub.py`. Cases: (1) None-after-valid past
budget ⇒ exactly one `telemetry_invalid` ESTOP **and** asserts
`telemetry_stale` did *not* fire (frames fresh, `data_age_s≈0`);
(2) half-budget ⇒ one once-only WARN, no ESTOP; (3) cold-start
`motor_pos=None` from tick 0 ⇒ no ESTOP (the `_fk_ever_succeeded`
gate); (4) recovery resets the clock then re-trips. Recipe was
empirically confirmed deterministic on the pinned stack
(`/tmp/probe_p5_telemetry_invalid.py`, 4/4, not committed) **before**
the test was written, per CLAUDE.md.

ESTOP reason string: `telemetry_invalid` (sibling of
`telemetry_stale` / `telemetry_frozen`).

## Verification

- Isolated (`pytest tests/sim/test_hardware_plant_failure_paths.py::TestP5TelemetryValidity
  ::TestFrozenMotorDetector::TestTelemetryStalenessThresholds
  tests/sim/test_hardware_plant_deadband.py -q`, run 2026-05-19):
  **27 passed in 2.43 s** — the 4 new T-U-P5 cases pass AND the
  pre-existing Tier-2a frozen-content / staleness / deadband tests
  still pass unchanged (B-additive's no-regression promise verified).
- Authoritative full pre-commit gate, post-`/audit` fixes
  (`pytest tests/ -q`, run 2026-05-19): **1411 passed, 1 xfailed
  in 412.06 s, exit 0 — fully clean.** The 1 xfailed is the
  pre-existing inherited T-U-T1a-4 `Restoration_Failed` permanent
  xfail (unrelated to this work).
- Pre-`/audit` gate (`pytest tests/ -q`, run 2026-05-19):
  **1410 passed, 1 failed, 1 xfailed in 428.52 s**. The single
  failure was `tests/sim/test_hot_loop_allocation_contract.py::
  test_hot_loop_allocation_contract` — the **documented
  load-induced tracemalloc-baseline flake** (the test's own
  docstring lines 254–263 + `logbook/2026-05-11-tier1c-input-fuzz.md`
  describe this exact class). It is *structurally impossible* for
  the P5 change to have caused it: that test builds a
  **`MuJoCoPlant`** (`_build_fixture` → `MuJoCoPlant()`), whose
  `get_state()` never executes a line of the `HardwarePlant`-only
  P5 code. Confirmed by isolation re-run
  (`pytest tests/sim/test_hot_loop_allocation_contract.py -q`,
  run 2026-05-19): **3 passed in 16.34 s**; and the flake did not
  recur in the authoritative post-audit run above.

## Outcome

Finding A (the telemetry-validity watchdog gap) is **resolved**:
`motor_pos=None`-while-fresh now escalates to
`estop('telemetry_invalid')` within the same 0.5 s blind budget as
total frame loss, gated so a never-armed cold start cannot false-trip.
The 68 s blind window this entry diagnosed cannot recur. Finding 3 of
the cascade-validation entry is laid to rest (root cause found; the
"numerically pathological z=30" hypothesis refuted — see Withdrawn
claims). Finding B (why `motor_pos` stayed None 68 s after a ~1.5 s
unplug) remains **open and spun out** — it does not block this safety
fix. Commit hash + final test triple recorded after the gate + COMMIT.

## Withdrawn claims

- [2026-05-18] (Inherited from the cascade-validation entry's
  Finding 3, not from this entry's own analysis.) Claimed
  *"`z=30` may be a numerically pathological / near-infeasible
  operating pose; ill-conditioned Jacobian near the lower workspace
  boundary → IPOPT iteration blow-up"*.
  WITHDRAWN: refuted by an in-data control — the *same session* held
  exactly z=30 for 16 continuous seconds at 100% solve success,
  13.8 ms mean, 1.58 IPOPT iterations, 27 mm stroke margin. A
  pathological pose cannot solve trivially for 16 s while the
  commanded pose is unchanged. The "1769 consecutive from line 11"
  figure was a counter-reset artefact (the counter resets on every
  success); the `Near lower stroke limit` flag was frozen-tail
  pollution from the all-zero telemetry-loss values.
  Superseded by: this entry's Diagnosis (the latch is a
  `motor_pos=None` telemetry dropout, not pose pathology) and
  Discussion §1. The cascade entry's Finding 3 needs a forward
  pointer added (see Open Questions — pending narrative edit, not
  yet applied).

- [2026-05-18] (Claude's own inference during the fix-direction
  discussion, *not* written into the Diagnosis.) Claimed
  *"replugging the CAN bus does not auto-restore ODrive encoder
  streaming — the ODrives fault on CAN loss and stay
  error/disarmed until re-armed, so motor_guard correctly emits
  `motor_pos=None` for the full 68 s."*
  WITHDRAWN on operator pushback: the operator's physical model is
  that the ODrives **do** auto-resume CAN telemetry broadcast once
  the bus returns (and no ODrive-side CAN watchdog is configured).
  Per the Investigator's Discipline, the hypothesis was dropped, not
  rescued with a secondary mechanism. Consequence: the 68 s of
  `motor_pos=None` is a genuine, *unexplained* recovery failure —
  re-scoped as **Finding B** (spun out; see Open Questions). This
  withdrawal does not affect Finding A: the consumer-side watchdog
  gap is real regardless of *why* `motor_pos` was None, and the
  operator approved its fix.

## Open Questions

### The telemetry-validity contract gap (PRIMARY — design RESOLVED, implementation pending)

A `motor_pos=None` (or otherwise sentinel-invalid) payload sustained
while the MPC is active and frames stay fresh is as dangerous as
frozen content — arguably **more** dangerous: it is an explicit "I am
blind" signal from motor_guard being **silently swallowed** by
`get_state()`'s `else: fill(0.0)` branch, with no escalation. The
platform ran 68 s on this swallowed signal.

**Fix direction agreed with the operator (2026-05-18 discussion):**
land a **new normative invariant P5 — Telemetry validity — in
`controller/PLANT_INTERFACE_CONTRACT.md`** (contract = doc + one
canonical enforcement point in `get_state()` + a regression test),
*not* a one-off patch, per CLAUDE.md *"favour contracts over
patches"*. The class is broader than this one None instance (any
sentinel-invalid / wrong-shape / NaN payload).

Resolved design parameters:

1. **One unified path (Option B), not two parallel detectors.**
   `motor_pos=None`/invalid and byte-frozen content are the same
   class — *"publisher alive, feedback unusable."* P5 collapses them
   into a single "must obtain a fresh, valid, **changing**
   `motor_pos` within the budget after arming, or ESTOP" invariant
   with one enforcement point. The existing `_frozen_motor_pos_count`
   tick-count detector is folded into P5 (subsumed, not kept
   alongside) — one canonical enforcement point is the K1–K6 / DIAG
   pattern and is harder to leave a third gap in.
2. **Threshold = `_telem_stale_estop_s` (0.5 s, 20× control_dt,
   P4-scaled).** Reuse the *existing* total-frame-loss budget — one
   source of truth for "MPC must never run blind for >0.5 s after
   arming," whether blind via no-frames (P4/`telemetry_stale`) or
   frames-without-valid-feedback (P5). Operationally accepted
   consequence, recorded deliberately: a CAN bounce longer than
   0.5 s now ESTOPs and requires re-arm — it no longer silently
   rides through (which it never should have).
3. **`_fk_ever_succeeded` gate retained.** Escalate only
   None-after-valid (feedback was good, then regressed). A
   never-armed / cold-start robot with no CAN has `motor_pos=None`
   from tick 0 and MUST NOT ESTOP — same guard the existing
   FK-fail and frozen-content escalations use.
4. **Consumer-side only.** motor_guard emitting `motor_pos=None`
   when it genuinely has no feedback is *correct, honest* behaviour
   (the alternative — fabricating/holding stale values — is the
   hazard the P-contracts exist to prevent). The bug is the
   *consumer* swallowing the signal; the fix belongs in
   `get_state()`, not motor_guard.

ESTOP reason string: `telemetry_invalid` (sibling of
`telemetry_stale` / `telemetry_frozen`). The implementation,
test, and `/audit` gate are tracked as this entry's Fix work
(separate commit, `Logbook-Entry:` trailer, full pytest gate).

### Finding B — why did `motor_pos` stay None for 68 s? (SPUN OUT — separate follow-up)

Distinct from the consumer-side gap above: *why was the feedback
absent for 68 s after a ~1.5 s CAN unplug?* The operator's physical
model is that the **ODrives auto-resume CAN telemetry broadcast once
the bus is restored**, and that **no ODrive-side CAN watchdog is
currently configured**. If the ODrives do resume broadcasting, the
68 s of `motor_pos=None` means the break is *downstream of the
ODrives* (can_node not re-forwarding, or motor_guard latching a
fault that doesn't clear on bus recovery) — or the ODrives drop to
an error/disarmed state on CAN loss (no watchdog) and broadcast
telemetry without valid closed-loop feedback.

Forensic discriminator available **without re-running hardware**:
`temp/reports/th_t2a1_candump_20260518_200207.log` (32 828 frames;
the 20:02:07 capture window straddles the t≈30.7 s dropout). Parsing
the ODrive CAN traffic across the unplug/replug shows directly
whether ODrive telemetry resumed on the bus — discriminating
"ODrives didn't recover (config/watchdog fix)" from
"can_node/motor_guard latched (software fix)".

Decision (operator, 2026-05-18): **spin this out** as its own
`/investigate` so the Finding A safety fix ships unblocked. This
section is the pointer; Finding B is not addressed in this entry's
Fix scope.

### Back-reference the cascade entry's Finding 3 (pending narrative edit)

The cascade entry
([2026-05-18-hardware-bringup-t2b1-t2a1-cascade-validation.md](2026-05-18-hardware-bringup-t2b1-t2a1-cascade-validation.md))
Finding 3 should gain a one-line forward pointer:
*"Superseded — root cause found, see
[2026-05-18-z30-solve-failure-motor-pos-none-watchdog-gap]; the
'numerically pathological' hypothesis was refuted by an in-data 16 s
clean z=30 hold."* This is a **pending narrative edit** — per
CLAUDE.md it requires `/audit --unstaged` before any commit, and the
cascade entry has **not** been edited as part of this entry's
creation. Flagged here so the edit is not forgotten and is gated
correctly.

## Related

- [logbook/2026-05-18-hardware-bringup-t2b1-t2a1-cascade-validation.md](2026-05-18-hardware-bringup-t2b1-t2a1-cascade-validation.md)
  — the cascade-validation entry. This entry **supersedes its
  Finding 3** (root cause found; the "numerically pathological"
  hypothesis refuted). The cascade PASSes and Plan 2's hardware gate
  are *not* invalidated — see Discussion §4.
- [controller/hardware_plant.py](../controller/hardware_plant.py)
  (`get_state()`, lines 498–712) — the None-branch
  (`:564`, `:630`), the `telemetry_stale` escalation (`:637`), and
  the `telemetry_frozen` detector with its `motor_pos is not None`
  gate (`:643–693`, gate at `:658`).
- [controller/PLANT_INTERFACE_CONTRACT.md](../controller/PLANT_INTERFACE_CONTRACT.md)
  — P1–P4. The candidate telemetry-validity invariant would live
  here. No existing P-invariant covers `get_state()`-output
  validity (Discussion §3).
- [controller/REFERENCE_LAYER_CONTRACT.md](../controller/REFERENCE_LAYER_CONTRACT.md)
  — K1–K6. **Contrast only**: this is *not* a K1–K6 gap; K1–K6
  governs reference feasibility, not measured-state validity
  (Discussion §3).
- `ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py:351,1078,1128`
  — the fault / no-feedback publish paths that set
  `msg['motor_pos'] = None` while the publisher stays alive.
- [plans/archived/hardware-bringup.md](../plans/archived/hardware-bringup.md)
  — z=30 verdict for the bringup plan (Discussion §5): z=30 is a
  safe, solvable pose; expect ~20 ms move-phase solves at low z
  under full launch load.
- Session: `temp/logs/mpc_20260518_200137.{log,csv}` (T-H-T2a-1
  20:01 run; 2979 records, 99.0 s). Companion candump
  `temp/reports/th_t2a1_candump_20260518_200207.log` (gitignored
  runtime artefacts).
