---
title: Finding B — the 68 s motor_pos=None was a motor_guard GuardMode.ESTOP sticky latch, NOT a CAN dropout (shared root cause with Finding A)
type: investigation
date: 2026-05-19
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
  - 2026-05-18-z30-solve-failure-motor-pos-none-watchdog-gap
  - 2026-05-18-hardware-bringup-t2b1-t2a1-cascade-validation
sessions:
  - mpc_20260518_200137.csv
subsystem:
  - motion
  - safety
  - ipc
  - controller
  - can
tags:
  - hardware-bringup
  - safety
  - motor-guard
  - estop
  - observability
  - can
  - hypothesis-withdrawn
  - investigation
  - open-question
  - contract-gap
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion_bridge_node.py
  - tests/ros/test_motion_bridge_node.py
commits:
  - 9159aa9
---

# Finding B — the 68 s motor_pos=None was a motor_guard GuardMode.ESTOP sticky latch, NOT a CAN dropout (shared root cause with Finding A)

## Summary

Finding B — spun out of the
[z=30 entry](2026-05-18-z30-solve-failure-motor-pos-none-watchdog-gap.md)'s
Open Questions — asked: *does the CAN bus auto-restore after a dropout,
and how should dropouts be handled?* The forensic answer overturns the
question's premise: **the CAN/ODrive layer auto-restores completely and
was never the problem.** The 68 s of `motor_pos=None` was a
**`motor_guard` `GuardMode.ESTOP` sticky latch that fired ~12 s BEFORE
the CAN cable was unplugged**, triggered by one of motor_guard's own
internal *non-CAN* safety checks (almost certainly `mpc_cmd_stale` /
`ipc_heartbeat_lost`) under the *same* low-z full-`jugglebot_launch.py`
CPU saturation that Finding A diagnosed. The CAN unplug is a
causally-unrelated red herring **for this symptom**.

Consequence: **Findings A and B share one root cause** (low-z + full
launch → MPC blows its solve budget → command/IPC stream starves
motor_guard's own thresholds → sticky ESTOP). Finding A's
already-shipped P5 (`c8a5dd4`) closes the consumer-side blind-window
safety hole, so the 68 s blind window *cannot recur* regardless of
cause. Finding B's residual issues are (1) an **orchestrator-
observability gap** — a safety-critical motor_guard ESTOP was invisible
to the system-level state for 68 s — which is being fixed in this same
session, and (2) a **latent `fatal_can_error` recovery path** that this
incident never exercised (untested, deferred).

This entry's diagnosis is **complete**. The Fix / Verification /
Outcome sections are intentionally empty placeholders — the
observability fix is being scoped and landed immediately after this
entry, in the same session, and those sections will be filled then.

## Motivation

The z=30 entry spun this out verbatim
([Open Questions → "Finding B — why did `motor_pos` stay None for
68 s?"](2026-05-18-z30-solve-failure-motor-pos-none-watchdog-gap.md)):

> Distinct from the consumer-side gap above: *why was the feedback
> absent for 68 s after a ~1.5 s CAN unplug?* The operator's physical
> model is that the **ODrives auto-resume CAN telemetry broadcast once
> the bus is restored**, and that **no ODrive-side CAN watchdog is
> currently configured**. If the ODrives do resume broadcasting, the
> 68 s of `motor_pos=None` means the break is *downstream of the
> ODrives* (can_node not re-forwarding, or motor_guard latching a
> fault that doesn't clear on bus recovery) — or the ODrives drop to
> an error/disarmed state on CAN loss (no watchdog) and broadcast
> telemetry without valid closed-loop feedback.
>
> Forensic discriminator available **without re-running hardware**:
> `temp/reports/th_t2a1_candump_20260518_200207.log` (32 828 frames;
> the 20:02:07 capture window straddles the t≈30.7 s dropout). Parsing
> the ODrive CAN traffic across the unplug/replug shows directly
> whether ODrive telemetry resumed on the bus — discriminating
> "ODrives didn't recover (config/watchdog fix)" from
> "can_node/motor_guard latched (software fix)".

The two operator questions were therefore:

1. Does the CAN bus / ODrive layer **auto-restore** telemetry
   broadcast after a transient unplug (the operator's physical
   model: yes, no ODrive-side CAN watchdog configured)?
2. **How should dropouts be handled** so a transient bus loss does
   not strand the platform blind?

**Scope decided with the operator (2026-05-19):** forensic-first —
decode the captured artefacts against *this repo's* CAN map, trace the
software path in code + rosbag, and produce a *diagnostic + prioritised
recommendation*. No hardware re-run for the forensic phase; the
discriminator is available offline.

## Investigation

Three phases. All `file:line` references are against the repo state at
this session; all frame/time figures decoded faithfully from the
captured artefacts.

### Phase 1 — forensic candump decode

Decoded `temp/reports/th_t2a1_candump_20260518_200207.log`
(**32 828 frames, 18.08 s span**) against **this repo's** CAN map:
`protocol_config.ODRIVE_COMMANDS`, `can/odrive.py`
`decode_heartbeat` / `decode_error` / `decode_encoder_estimate`,
`CMD_ID_MASK=0x1F`, arbitration ID = `(node << 5) | cmd`, legs =
node `0–5`. Throwaway decoder per CLAUDE.md empirical-probe discipline
(`/tmp/probe_findingB_*.py`, not committed).

Results:

- **Exactly one** clean, simultaneous all-node bus gap:
  **20:02:19.70 → 20:02:21.34 (~1.64–1.65 s)**. This is the physical
  unplug — it matches the ~1.5 s test design, and it is the *only*
  bus-wide silence in the entire 18 s capture.
- Every leg ODrive **resumed broadcasting within ~ms** of the bus
  returning. `Encoder_Estimates` resumed (~465 frames/leg over the
  capture) and flowed **continuously** after the gap — no second
  silence, no stutter.
- Every leg Heartbeat `current_state` is a constant
  **`CLOSED_LOOP(8)` across AND after the gap** — **zero transitions**
  on any of the 6 nodes. `get_error` decoded **all-zero on all 6 legs
  throughout**. Encoder position ≈ **−0.379 rev** on every leg (a
  sane, static, continuous hold value) before, during the broadcast
  resume, and after.
- **Conclusion:** the ODrives never faulted, never disarmed, never
  left closed-loop. The CAN/ODrive layer **fully auto-recovers with
  zero software intervention** — exactly the operator's physical
  model. Operator Question 1: **answered, yes**.

### Phase 2 — code trace + rosbag

Rosbag `/home/jetson/Desktop/rosbags/2026-05-18_20-00-43/…mcap`
(**147.4 s**, hand-decoded CDR). The feedback pipeline:

```
ODrive → can_node._poll_can_bus (1 kHz)
       → _handle_message / _handle_encoder
       → motors.update
       → _publish_robot_state (100 Hz /robot_state)
       → motion_bridge_node._on_robot_state  (pure pass-through, NO fault gate)
       → ZMQ TOPIC_MOTOR_FB
       → motor_guard._on_motor_feedback
```

Decisive code facts:

- `motor_guard._has_motor_fb` is set `False` **only** at
  `motor_guard.py:254` (init) and `True` **only** at `:678`; it is
  **never reset to `False` anywhere in the repo** (exhaustively
  grepped). Therefore the 68 s `motor_pos=None` **cannot** be
  `_publish_telemetry` / `_publish_feedback_telemetry` running with
  `_has_motor_fb=False`. By elimination the only remaining producer
  of a `motor_pos`-absent payload is the **ESTOP path
  `_publish_fault_telemetry` (`motor_guard.py:1084-1100`), which omits
  the `motor_pos` key entirely** → on the consumer side
  `HardwarePlant.get_state()` does `telem.get('motor_pos')` → `None` →
  `actual_ext` filled to exactly `0.0` (the z=30 entry's documented
  None-branch).
- Run-loop selector (`motor_guard.py:392-402`):
  `_publish_fault_telemetry` runs **iff** `self.mode ==
  GuardMode.ESTOP`. `GuardMode.ESTOP` has **NO automatic exit**
  (`_on_mode_command` `:605-652`: from `ESTOP`, an `enable` command is
  a **no-op** — the only way out is `disable` → `DISABLED`, then
  `enable` to re-arm). The latch is sticky by design.
- **Rosbag timeline** (rel = seconds relative to the candump unplug at
  epoch `1779098539.703`):
  - `/robot_state` healthy with `has_fatal_can_error == 0` for **all
    14 153 messages** across the whole 147 s run.
  - can_node reports `current_state = 8` (CLOSED_LOOP); encoder data
    fresh and *changing* over the window rel −41 … +55 s.
  - motor_guard's *valid* telemetry (`/leg_lengths_topic` at 500 Hz,
    `/motion/diagnostics`) **ceased cleanly at rel −12.10 s** — the
    last good sample is `level=0, msg='OK', fault_state=None` — and
    **never resumed** for the rest of the run.
  - `/control_mode_topic` stayed `'STANDBY'` and
    `/orchestrator_state` stayed `ACTIVE:STANDBY` the **entire run** —
    the system-level state never reflected the ESTOP.
  - Cross-check against the MPC CSV: `actual_ext` → 0.0 at idx 1034 =
    MPC-t 30.701 s; blind-window length 99 − 30.7 ≈ **68 s** — agrees
    with the z=30 entry's measurement.
- This **refutes the Phase-2 working hypothesis** (that a
  CAN-recovery software latch was *triggered by the unplug*): the
  latch begins at rel **−12.10 s**, i.e. ~12 s *before* the unplug;
  `has_fatal_can_error` is **never** set; can_node's watchdog /
  `attempt_restore_steps` path is **never entered** (the bus gap of
  1.65 s is shorter than `_HEARTBEAT_TIMEOUT_S = 2.0 s`, so can_node
  never even noticed a dropout).

### Phase 2b — trigger attribution

The exact `_trigger_estop` reason string is **not recoverable from
this session**: `jugglebot_launch.py` launches nodes with
`output='screen'`, so motor_guard's stdout was never written to disk.
`~/.ros/log/2026-05-18-20-00-43-…/launch.log` contains only process
start/finish epochs (motor_guard pid 12223 ran
`1779098443.83 → 1779098595.50`) — no diagnostic lines.

By elimination + timing, the trigger is one of motor_guard's
**non-CAN** safety checks:

- `mpc_cmd_stale` — MPC command older than
  `MPC_CMD_STALENESS_S = 0.25 s` (`motor_guard.py:844-852`), or
- `ipc_heartbeat_lost` — IPC heartbeat older than
  `IPC_HEARTBEAT_TIMEOUT_S = 0.5 s` (`motor_guard.py:836-841`).

The motion-anomaly triggers are **ruled out** by the candump + CSV:
`max_deviation` (encoders ≈ constant at −0.379 rev — no deviation),
`workspace_hard_limit` (z≈30 is in-workspace), `motor_overspeed`
(no velocity per candump/CSV). The platform was in a static hold.

**Strong independent corroboration:** Finding A's own `/diagnose`
flagged *"120 isolated MPC overhead spike(s): max 257.6 ms"*. A
257.6 ms MPC stall **directly exceeds** motor_guard's 250 ms
`MPC_CMD_STALENESS_S` threshold — i.e. the exact mechanism that would
fire `mpc_cmd_stale` is independently measured in the same session's
telemetry. This is a **high-confidence inference, not a proof**: the
literal reason string is unrecoverable, and `ipc_heartbeat_lost`
(0.5 s) is an equally-plausible sibling under the same CPU starvation.
Residual uncertainty: *which* of the two non-CAN thresholds tripped
first. It does not affect the root-cause attribution — both are
downstream of the same low-z full-launch CPU saturation, and both
latch the same sticky `GuardMode.ESTOP`.

## Discussion

This investigation hit *every* condition CLAUDE.md names as making the
Discussion non-negotiable: **three hypotheses were withdrawn**, there
was a **major mid-investigation reframe**, and the chosen synthesis
(**one shared root cause across Findings A and B**) is not inferable
from the code alone. Written below in full.

### 1. Three hypotheses withdrawn, in sequence

This arc is a textbook application of the Investigator's Discipline —
*"don't rescue a dying hypothesis."* Each was dropped on the single
data point that contradicted it, not patched with a secondary
mechanism:

- **(a) "z=30 is numerically pathological"** — *inherited* from the
  cascade entry's Finding 3. Already refuted in the z=30 entry by an
  in-data control (the same session held z=30 cleanly for 16 s at
  100% solve success). Carried here only as the chain's first link.
- **(b) "the ODrives stay disarmed until re-armed"** — Claude's own
  inference during the z=30 fix-direction discussion (the ODrives
  fault on CAN loss and emit `motor_pos=None` for the full 68 s).
  Refuted by **Phase 1**: every Heartbeat is `CLOSED_LOOP(8)` with
  zero transitions and all-zero `get_error` across *and after* the
  gap. The ODrives never faulted. This withdrawal was first recorded
  in the z=30 entry on operator pushback; Phase 1 now supplies the
  *forensic proof* the pushback predicted.
- **(c) "a CAN-recovery software latch triggered by the unplug"** —
  Phase 2's *own* working hypothesis (some software component latches
  a fault when the bus drops and fails to clear it on recovery).
  Refuted by **the rosbag**: the latch begins at rel −12.10 s, ~12 s
  *before* the unplug; `has_fatal_can_error` is never set; can_node's
  watchdog never even fires (1.65 s gap < 2.0 s timeout). The unplug
  is causally downstream of nothing here.

The discipline that mattered: hypothesis (c) was *attractive* — it
made the unplug the cause and gave a tidy story. The rosbag's −12.10 s
timestamp killed it cleanly, and it was abandoned rather than rescued
with "well, maybe a *different* latch."

### 2. The time-alignment flag was load-bearing

In Phase 1, before the rosbag was decoded, Claude explicitly flagged
**two** possible MPC-t→wall-clock alignments and declined to
hand-wave the choice:

- a **"parsimonious"** alignment in which the `motor_pos=None` onset
  coincides with the unplug (dropout == unplug); and
- a **"scarier"** alignment in which `motor_pos=None` starts ~12 s
  *before* the unplug.

Resolution was *deferred to the rosbag* rather than assumed. The
rosbag confirmed the **scarier** alignment (latch at rel −12.10 s).
Had the parsimonious reading been assumed — the natural, lower-effort
default — the entire root cause would have been mis-attributed to the
CAN unplug, and the real mechanism (CPU-starvation → motor_guard's own
non-CAN thresholds → sticky ESTOP) would have been missed entirely.
**Methodological lesson, recorded deliberately:** when a time
alignment is ambiguous and the two readings imply *different root
causes*, do not pick the parsimonious one to move faster — defer to
the artefact that disambiguates. The 12 s offset was the whole ball
game.

### 3. "Does the CAN bus auto-restore?" was the wrong question

The operator's framing question is *true* (yes, the bus auto-restores
— Phase 1 proves it) but **not load-bearing** for this symptom. The
decidable, load-bearing questions turned out to be:

1. *Did the **axes** stay armed?* — Phase 1: **yes**, constant
   `CLOSED_LOOP(8)`, zero faults.
2. *Where does the **software** latch?* — Phase 2: **motor_guard
   `GuardMode.ESTOP`**, and it latched ~12 s *before* the bus event.

Distinguishing these three layers — *bus-broadcast-resume* vs
*axis-armed-state* vs *software-latch* — mattered because they imply
**completely different fixes**: a bus/ODrive-config/watchdog fix, a
re-arm-sequencing fix, or a consumer/observability fix, respectively.
Conflating "the bus came back" with "the platform recovered" is
exactly the error that the parsimonious time-alignment (§2) would have
baked in. Phase 1 isolates the bus layer as healthy; Phase 2 localises
the defect to the software-latch layer — and, crucially, to a latch
that is *upstream in time* of the bus event entirely.

### 4. Findings A and B share ONE root cause

This is the non-obvious synthesis. Stated as a single causal chain:

> **Low z + full `jugglebot_launch.py` + rosbag-record + test-script +
> diagnostic-shell load → MPC blows its ~25 ms solve budget**
> (Finding A measured: mean 29 ms, max 101 ms, fallback-path the whole
> static hold; `/diagnose` flagged 257.6 ms overhead spikes) **→ the
> MPC→motor_guard command/IPC stream starves past motor_guard's *own*
> 0.25 s `mpc_cmd_stale` / 0.5 s `ipc_heartbeat_lost` thresholds →
> motor_guard latches `GuardMode.ESTOP` (sticky, no auto-exit,
> orchestrator-invisible) → `_publish_fault_telemetry` omits
> `motor_pos` → `HardwarePlant.get_state()` fills `actual_ext = 0.0` →
> 68 s of `motor_pos=None`.**

"z=30 can't solve" (Finding A) and "68 s blind" (Finding B) are
**two symptoms of one root cause**, not two bugs. The CAN unplug is
**incidental to both** — it is a real test event that happens 12 s
*after* the latch and is the genuine target of the T-H-T2a-1 cascade
(which fired correctly at t≈99 s, per the cascade entry), but it has
**no causal role** in the `motor_pos=None` symptom. The durable fix
for *both* findings is the same: **relieve MPC solve-budget pressure
at low z under full launch** — and that is hardware-bringup-plan
scope, not a one-off patch in either entry.

### 5. The real Finding-B issue is observability, not CAN handling

The operator's framing ("how to handle CAN dropouts?") presumed the
defect was in CAN handling. It is not. The defect this entry surfaces
is that **a safety-critical `motor_guard` ESTOP was invisible to the
orchestrator for 68 s**:

- `/control_mode_topic` never left `'STANDBY'`;
  `/orchestrator_state` never left `ACTIVE:STANDBY`.
- The motion bridge only forwards a fault *upward* via the
  `cond`-bearing diagnostic path — and that path goes silent
  **precisely when** `_publish_fault_telemetry` omits `cond_number`
  (the same omission that drops `motor_pos`). The one channel that
  could have surfaced the ESTOP is muted by the very fault state it
  would report.

A safety mechanism that **no other component can observe** is a latent
hazard class in its own right: the platform can be in ESTOP while the
system-level state machine believes everything is nominal, with no
operator alert and no escalation. This — not CAN handling — is the gap
the in-session follow-up fix targets.

### 6. What is covered vs what is latent

- **Covered (Finding A, already shipped).** P5 (`c8a5dd4`,
  `controller/PLANT_INTERFACE_CONTRACT.md` + `hardware_plant.py`
  enforcement + regression test) makes the MPC ESTOP within 0.5 s of
  `motor_pos=None` **regardless of cause**. The 68 s blind window
  **cannot recur** — whatever upstream produces a None payload, the
  consumer now escalates inside the budget. Finding B's root-cause
  attribution does **not** reopen Finding A; it explains *why* the
  None payload appeared, which P5 deliberately does not depend on.
- **Latent (dormant here, NOT this incident's mechanism).**
  `MotorStateTracker.fatal_can_error` has **no clear recovery path**
  without a full re-arm: `can/motor_state.py` `clear_error_flags`
  omits it, and it is cleared **only** by direct assignment in
  `can_node._watchdog_check` (around `:1294`). A CAN dropout **longer
  than 2 s** (vs the **1.65 s** here, which never tripped can_node's
  `_HEARTBEAT_TIMEOUT_S = 2.0 s`) would exercise can_node's
  **untested** watchdog / `attempt_restore_steps` / `_emergency_idle`
  path and could set a `fatal_can_error` that never clears without a
  re-arm. *That* is the genuine, still-unvalidated "CAN dropout
  handling" question the operator originally asked — but it is **not**
  what happened in this session, so it is deferred (Recommendation 4),
  not conflated with this incident.

## Recommendations

Prioritised; diagnostic-only — **no code fixes are prescribed in this
entry** (the observability fix is scoped+landed in the same session
immediately after, see Fix below):

1. **(Being fixed this session — highest leverage.)** Make a
   `motor_guard` `GuardMode.ESTOP` **observable to the orchestrator**
   so the system-level state (`/control_mode_topic` /
   `/orchestrator_state`) reflects it, the cascade escalates promptly,
   and the operator is alerted. The latent hazard is "safety mechanism
   no other component can observe" (Discussion §5).
2. **Relieve MPC solve-budget pressure at low z under full launch**
   (the shared A↔B root cause, Discussion §4). Hardware-bringup-plan
   item — candidates: AOT-warm solver on the real launch path, a
   lighter bringup launch profile, or a solve-headroom review at low
   z. This is the *durable* fix for both findings.
3. **Configure `motor_guard` stdout → file logging for bringup
   sessions** so the next `_trigger_estop` reason string is
   diagnosable in one shot. This session's reason was unrecoverable
   (`output='screen'` in `jugglebot_launch.py`); had it been on disk,
   Phase 2b would have been a one-line grep instead of an
   elimination argument.
4. **Deferred:** a controlled **>2 s** CAN-dropout hardware test to
   validate can_node's untested watchdog / `attempt_restore_steps` /
   `fatal_can_error` recovery path (the latent item in Discussion §6).
   This is the only remaining piece of the operator's original
   "how to handle dropouts" question that this forensic analysis
   could not answer offline.

## Fix

Scoped to **Layer 1** (operator/GUI/rosbag observability) per the
FIX_PLAN gate (operator-approved 2026-05-19). Layer 2 (orchestrator
auto-escalation to ERROR on a motor_guard self-ESTOP) is explicitly
**not** landed here — it is a safety-control *behaviour* change with
cascade implications and a genuine policy question (alert-only vs
auto-escalate for a freewheeling platform); recorded as Recommendation
1b / Open Question, a separate gated decision.

Root cause (code-confirmed): `motion_bridge_node._poll_telemetry`
gated the entire `motion/motor_guard` `DiagnosticStatus` publish
behind `if cond is not None:`. `motor_guard._publish_fault_telemetry`
(the `GuardMode.ESTOP` telemetry path) carries `fault_state` +
`workspace_status` but **omits `cond_number`**, so the ERROR-level
fault diagnostic was structurally suppressed *exactly* when motor_guard
was in ESTOP — the fault alarm was wired through the value that
disappears on fault.

| File | Change |
|------|--------|
| `ros_ws/src/jugglebot/jugglebot/motion_bridge_node.py` | Re-gate the diagnostic publish on `cond is not None or fault` (was `cond is not None`). `fault_state` ⇒ `level=ERROR` / `message='FAULT: …'` regardless of `cond`. `cond_number` is included in `values` only when the telemetry path carries it (inserted at index 0 to preserve historical ordering); `workspace_speed_scale` keeps its existing default. Happy path byte-for-byte unchanged. Added a comment stating the invariant + linking this entry. |
| `tests/ros/test_motion_bridge_node.py` (new) | Regression suite (7 tests, 3 classes): ESTOP-shaped telem with **no** `cond_number` ⇒ exactly one ERROR `DiagnosticStatus` naming the fault (pre-fix: 0 — silently swallowed); fault precedence over workspace; happy-path OK/WARN/ERROR unchanged with `cond_number` present; no-cond-no-fault and `None` telem ⇒ no diagnostic spam. No bridge test existed before (the `diagnostic_msgs`/IPC import surface had no precedent); fixture patches `BridgeIPC` so construction opens no ZMQ socket (mirrors `test_orchestrator_node`'s all-externals-mocked pattern). |

Fix locus is the **consumer (bridge)**, not the producer
(`motor_guard._publish_fault_telemetry` padding `cond_number`): the
bug is the bridge conflating "monitoring data present" with "is there
anything worth reporting." Same root-cause logic as Finding A's P5 —
fix the consumer that swallows the signal, not the honest producer.
Recipe was empirically validated red→green in a throwaway
`/tmp/probe_bridge_fault.py` (not committed) **before** the committed
test, per CLAUDE.md: pre-fix the ESTOP case asserted "publish NOT
called" (bug reproduced); post-fix all cases green.

Proportionality note: this is a single-site gating bug, not a
recurring multi-site class — so the deliverable is patch + regression
test + an in-code invariant comment + this Discussion principle, **not**
a new normative `*.md` (a contract document at K1–K6 / P5 scale would
be over-engineering for one call site). The principle —
*"a fault / safety signal MUST NOT be gated behind an optional
monitoring field"* — is recorded in Discussion §5 and at the fix site.

## Verification

Each claim carries the (date, exact command, result) triple per the
CLAUDE.md workflow rule.

- Isolated (`pytest tests/ros/ -q`, run 2026-05-19): **446 passed
  in 3.61 s** — the full `tests/ros/` suite *including* the 7 new
  bridge tests (no regression in the shared conftest ROS2-mock
  surface). (An earlier draft cited 453 from
  `pytest tests/ros/test_motion_bridge_node.py tests/ros/ -q`, which
  double-counts the new file — named explicitly *and* via
  `tests/ros/`; the honest suite total is 446.)
- Full pre-commit gate (`pytest tests/ -q`, run 2026-05-19):
  **1418 passed, 1 xfailed in 422.03 s, exit 0 — fully clean.**
  1418 = 1411 (prior clean baseline, Finding A) + 7 new bridge tests.
  The 1 xfailed is the pre-existing inherited T-U-T1a-4
  `Restoration_Failed` permanent xfail (unrelated). No load-flake this
  run.

## Outcome

Finding B's actionable Layer-1 gap is **closed and verified**: a
motor_guard `GuardMode.ESTOP` now publishes an ERROR
`DiagnosticStatus` on `motion/diagnostics` regardless of whether
`cond_number` is present, so the fault is visible to the operator,
GUI, and rosbag in real time — the 68 s silent blind window
diagnosed here cannot recur silently (and Finding A's shipped P5
independently ESTOPs the MPC within 0.5 s of `motor_pos=None`).

Status set **`in-progress` → `tuned`**: the symptom this entry
investigated (the silent 68 s latch) is addressed and verified, but
the entry intentionally leaves siblings open — Layer 2 (orchestrator
auto-escalation policy, Open Question), the shared A↔B root cause
(MPC solve-budget headroom at low z under full launch, a
hardware-bringup-plan item), and the deferred controlled >2 s
CAN-dropout test (Recommendation 4, the still-unvalidated
`fatal_can_error` path). Commit hash + final triple recorded after
the COMMIT gate.

## Withdrawn claims

- [2026-05-18] (Inherited from the cascade-validation entry's
  Finding 3, via the z=30 entry — not this entry's own analysis.)
  Claimed *"`z=30` may be a numerically pathological / near-infeasible
  operating pose."*
  WITHDRAWN: refuted in the z=30 entry by an in-data control (the same
  session held exactly z=30 for 16 continuous seconds at 100% solve
  success, 13.8 ms mean, 1.58 IPOPT iterations, 27 mm stroke margin).
  Carried here only as the first link of Finding B's causal chain.
  Superseded by:
  [2026-05-18-z30-solve-failure-motor-pos-none-watchdog-gap.md](2026-05-18-z30-solve-failure-motor-pos-none-watchdog-gap.md)
  Diagnosis + Discussion §1, and this entry's Discussion §4 (the real
  root cause is solve-budget pressure, not pose pathology).

- [2026-05-18] (Claude's own inference during the z=30 fix-direction
  discussion — already recorded in the z=30 entry's Withdrawn-claims;
  re-stated here because it is central to Finding B's arc.) Claimed
  *"replugging the CAN bus does not auto-restore ODrive encoder
  streaming — the ODrives fault on CAN loss and stay error/disarmed
  until re-armed, so motor_guard correctly emits `motor_pos=None` for
  the full 68 s."*
  WITHDRAWN: refuted by **Phase 1 forensic decode** — every leg
  Heartbeat is a constant `CLOSED_LOOP(8)` with zero transitions and
  all-zero `get_error` across *and after* the 1.65 s bus gap; every
  ODrive resumes broadcasting within ~ms. The ODrives never faulted
  and never disarmed. (First withdrawn in the z=30 entry on operator
  pushback; this entry supplies the forensic proof.)
  Superseded by: this entry's Phase 1 and Discussion §1(b). Cross-link:
  [z=30 entry → Withdrawn claims](2026-05-18-z30-solve-failure-motor-pos-none-watchdog-gap.md).

- [2026-05-19] (This entry's own Phase-2 *working* hypothesis,
  recorded as a withdrawal because it was load-bearing during the
  investigation.) Claimed *"the 68 s `motor_pos=None` is a
  CAN-recovery software latch **triggered by the unplug** — some
  component latches a fault when the bus drops and fails to clear it
  on bus recovery."*
  WITHDRAWN: refuted by the rosbag time alignment — motor_guard's
  valid telemetry ceases at rel **−12.10 s**, ~12 s *before* the
  unplug; `has_fatal_can_error` is never set; can_node's watchdog
  never fires (1.65 s gap < `_HEARTBEAT_TIMEOUT_S = 2.0 s`). The latch
  precedes and is causally independent of the unplug.
  Superseded by: this entry's Phase 2 / Phase 2b and Discussion §1(c)
  / §4 — the latch is a motor_guard `GuardMode.ESTOP` fired by a
  non-CAN threshold (`mpc_cmd_stale` / `ipc_heartbeat_lost`) under
  shared-root-cause CPU starvation.

## Open Questions

- **Which non-CAN threshold tripped first** — `mpc_cmd_stale`
  (0.25 s) or `ipc_heartbeat_lost` (0.5 s)? Unrecoverable from this
  session (`motor_guard` stdout not on disk). Recommendation 3 makes
  the next occurrence a one-shot diagnosis. Does not affect the
  root-cause attribution (both are downstream of the same CPU
  saturation; both latch the same sticky ESTOP).
- **The latent `fatal_can_error` / >2 s-dropout recovery path**
  (Discussion §6, Recommendation 4) — genuinely unvalidated; deferred
  to a controlled hardware test. This is the residue of the operator's
  original "how to handle CAN dropouts" question that the forensic
  analysis could not settle offline.
- **The observability fix itself** — scoped and landed in this same
  session immediately after this entry (Recommendation 1); its Fix /
  Verification / Outcome will be filled in this entry. Final status
  will move `in-progress → tuned` (not `resolved`) because the
  >2 s-dropout sibling above is intentionally left open.
- **Latent edge in the bridge gate — `fault_state == ''`** (surfaced
  by `/audit` of the Layer-1 fix; accepted, not patched). The new gate
  `if cond is not None or fault:` and selector `if fault:` use
  truthiness, so an *empty-string* `fault_state` would degrade to
  `level=OK` / no publish. Verified **unreachable with today's
  producer**: every `motor_guard._trigger_estop(...)` caller passes a
  non-empty literal/f-string and `_fault_state` is only ever `None` or
  such a string (`motor_guard.py:334,619,637,1142`). Deliberately not
  changed: switching to `is not None` would diverge from the pre-fix
  `if fault:` semantics the FIX_PLAN promised to preserve and is
  scope-expansion on a safety path for a non-bug. Documented here so a
  future producer change that could emit `''` is caught by this note
  rather than silently degrading; revisit if `_fault_state`'s domain
  ever widens.

## Related

- [2026-05-18-z30-solve-failure-motor-pos-none-watchdog-gap.md](2026-05-18-z30-solve-failure-motor-pos-none-watchdog-gap.md)
  — the z=30 entry. **This entry resolves its Finding B** (the
  spun-out "why was `motor_pos` None for 68 s?" question). Findings A
  and B share one root cause (Discussion §4); Finding A's shipped P5
  (`c8a5dd4`) already closes the consumer-side blind window.
- [2026-05-18-hardware-bringup-t2b1-t2a1-cascade-validation.md](2026-05-18-hardware-bringup-t2b1-t2a1-cascade-validation.md)
  — the cascade-validation entry. Its T-H-T2a-1 PASS (ESTOP at
  t≈99 s, `telem_age=0.505 s` vs 0.5 s) stands and is unaffected; the
  CAN unplug it validated is the same event Finding B shows is
  *incidental* to the 68 s `motor_pos=None` symptom.
- [plans/archived/hardware-bringup.md](../plans/archived/hardware-bringup.md)
  — the shared A↔B root cause (relieve MPC solve-budget pressure at
  low z under full launch) is a bringup-plan item (Recommendation 2);
  the deferred >2 s-dropout test (Recommendation 4) belongs here too.
- `ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py` — the sticky
  ESTOP latch: `:254` (`_has_motor_fb` init, only `False` site),
  `:392-402` (run-loop selector → `_publish_fault_telemetry` iff
  `mode == GuardMode.ESTOP`), `:605-652` (`_on_mode_command`: no
  auto-exit from ESTOP), `:836-852` (`ipc_heartbeat_lost` /
  `mpc_cmd_stale` non-CAN triggers), `:1084-1100`
  (`_publish_fault_telemetry` omits `motor_pos` / `cond_number`).
- `ros_ws/src/jugglebot/jugglebot/can_node.py` (`:1275-1323`,
  `_watchdog_check`) — the untested watchdog / restore /
  `fatal_can_error`-clear path; never entered this session (1.65 s
  gap < 2.0 s `_HEARTBEAT_TIMEOUT_S`). Latent (Discussion §6).
- `ros_ws/src/jugglebot/jugglebot/can/motor_state.py` —
  `clear_error_flags` omits `fatal_can_error`; only cleared by direct
  assignment in `can_node._watchdog_check`. Latent recovery-path gap.
- Artefacts (gitignored runtime):
  `temp/reports/th_t2a1_candump_20260518_200207.log` (32 828 frames,
  18.08 s — Phase 1 decode); rosbag
  `/home/jetson/Desktop/rosbags/2026-05-18_20-00-43/…mcap` (147.4 s —
  Phase 2 trace); `temp/logs/mpc_20260518_200137.{log,csv}`
  (T-H-T2a-1 session — the original `motor_pos=None` symptom).
- Forensic decoders were throwaway `/tmp/probe_findingB_*.py`
  (candump + CDR), **not committed**, per CLAUDE.md empirical-probe
  discipline.
