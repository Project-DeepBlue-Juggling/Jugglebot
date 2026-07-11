---
title: GUI orchestrator state-machine minimap — safe click-to-state navigation with a runbook-enforcing sequencer and hold-to-confirm
type: feature
date: 2026-07-11
status: resolved
phase: GUI + orchestrator control surface
related_plan: mvp-trajectory-bringup.md
related_entries:
  - 2026-07-11-gui-can-traffic-per-bus-panel
  - 2026-07-11-gui-leg-setpoint-echo-poscmd
  - 2026-07-02-canbridge-phase4-orchestrator-wiring
files_changed:
  - ros_ws/gui/js/state-minimap.js
  - ros_ws/gui/css/state-minimap.css
  - ros_ws/gui/index.html
  - ros_ws/gui/js/main.js
  - tests/ros/test_gui_geometry.py
  - tools/probes/gui_dom_probe.py
  - tools/probes/gui_synthetic_stack.py
  - tools/probes/README.md
commits:
  - pending-backfill
subsystem:
  - gui
  - ros
tags:
  - safety
  - observability
  - testing
---

## Summary

New user-specified GUI feature: a **translucent minimap of the orchestrator
state machine** snapped bottom-left of the 3D scene, above the command strip.
Click the header → it expands to full scene height, goes opaque, and pushes the
3D scene AND command strip right (the scene trims left/right symmetrically via
the fixed-vertical-FOV camera + the existing ResizeObserver — **zero viewer.js
changes**). The current state is highlighted (FAULT pulses); unreachable states
are greyed with per-node reasons; **clicking a node drives the system there in
the fewest SAFE steps** via an 800 ms hold-to-confirm; a contextual button
surfaces the "currently necessary command" (Arm Setpoints / Disarm / Recover /
Clear Errors / Retry Boot) with service reject reasons verbatim. The
safety-critical transition table is implemented as reviewable DATA, and the
sequencer enforces the bench runbook's Sharp Edges — teardown always routes
disarm through an anchored echo-quiescence wait and a HARD verify-gate before
`deactivate` is ever published. A four-lens adversarial review plus adversarial
verification of the HIGH findings: **2 HIGH roots CONFIRMED (both fixed
in-session), 1 HIGH refuted, 18 medium/low triaged (most fixed)**.
Post-fix probe re-runs green (minimap **33/33 ×2** + scenario1 **17/17**);
the full-suite triple is recorded in Verification.

## Motivation

The orchestrator state machine has been operator-invisible: state is inferred
from scattered badges, and driving the system between states requires holding
the bench runbook's safe command ordering (Sharp Edges) in your head — the
worst silent failure being a `deactivate` published while MPC_STALE with
un-stowed legs. The minimap makes the machine visible (current state, what is
reachable and *why not* when it isn't) and makes navigation safe-by-
construction: the GUI computes and executes the runbook-correct sequence, with
every hazard gate in code rather than in operator memory. It is deliberately
**stricter than the command strip**, which still carries the known BOOT-Home
silent no-op.

## Design

**Layout/interaction (user-specified):** translucent minimap bottom-left of
the 3D scene, above the command strip; header click expands to full scene
height, opaque, pushing scene + command strip right. The scene reflow rides
the fixed-vertical-FOV camera + existing ResizeObserver, so viewer.js is
untouched. Current state highlighted (FAULT pulses); unreachable states greyed
with per-node reasons; unknown state string ⇒ **all grey, fail-safe**.

**Architecture:** new ES module `ros_ws/gui/js/state-minimap.js` (~1500 lines,
sectioned: constants + SVG layout tables / snapshot store / reachability /
plan table / sequencer / contextual button / rendering / init) +
`css/state-minimap.css`. An architect-designed plan (Plan agent) was reviewed
before implementation; the safety-critical transition table is DATA, not code.

**Inputs:** `orchestrator_state`, `control_mode_topic`, `robot_state`,
`link_status` (via the router the CAN panel reserved), and `leg_setpoint_echo`
(the sibling entry's new topic — used as the `go_home` quiescence proxy).
**Commands:** the shared cached `orchestrator_command` publisher + std_srvs
services via `ros.callService`.

**Sequencer safety (enforces the bench runbook Sharp Edges):**

- Teardown = standby-if-needed → trajectory/`go_home` (best-effort) →
  **anchored echo-quiescence wait** → `set_setpoint_output(false)` ALWAYS →
  **HARD verify-gate** (a fresh `link_status` tick with `mpc_active` exactly 0;
  ABORT otherwise — `deactivate` is never published unverified; prevents the
  MPC_STALE + un-stowed-legs worst silent failure) → `deactivate`.
- **Recovery entry point** (plan refinement over the original design): ALWAYS
  the bridge `/clear_errors` Trigger — it self-routes to converge-first
  `/recover` when armed using the bridge's authoritative in-process
  `_mpc_active`, immune to stale GUI-side reads. The orchestrator
  `clear_errors` command is used ONLY for the boot-timeout FAULT flavour
  (F-C Retry Boot).
- Plans are computed **at confirm time from a fresh snapshot**; per-step pre
  re-checks before every publish; a single-flight lock; global abort on
  FAULT-mid-sequence / disconnect / timeout, surfaced in the status line +
  Event Log.

**Reachability** = client-side guards replicating orchestrator ground truth
(heartbeats = 7 axes `current_state != 0`, `firmware_validated`, `is_homed`,
`error[]`, guard latch via `link_status.fault_state`) + freshness guards (see
review fixes). BOOT/HOMING/LEVELLING source states grey everything
("commands discarded").

## Discussion

### Why click-to-state is a table, not BFS

The obvious implementation is a graph search over transitions. Chosen instead:
an explicitly enumerated **plan table** — every multi-step route is a row a
human wrote down and reviewed against the runbook. Concrete failure mode this
prevents: a search can *emerge* a route that is graph-legal but
runbook-illegal (a fresh edge added for one purpose becomes a shortcut through
an unconsidered intermediate state), and reviewing a search means proving
properties of an algorithm rather than auditing rows of data. The machine is
tiny and stable, so the table costs nothing and turns the safety-critical
artifact into something the adversarial review could read line-by-line — which
is also why the transition table shipped as DATA in its own module section.

### Why recovery routes through the bridge Trigger (stale-read immunity)

The original design chose recovery commands GUI-side based on the GUI's view
of armed state. Refinement: the entry point is ALWAYS the bridge
`/clear_errors` Trigger, because the bridge self-routes to converge-first
`/recover` using its authoritative **in-process** `_mpc_active`. A GUI-side
choice would act on a snapshot that can be stale in exactly the situations
recovery runs in (link flap, FAULT churn) — the bridge's own read cannot be.
The orchestrator `clear_errors` command survives only for the boot-timeout
FAULT flavour (F-C Retry Boot), which the bridge Trigger does not address.

### The quiescence-anchor bug class: freshness ≠ relevance (review HIGH #1)

The teardown's echo-quiescence wait **false-passed on pre-`go_home` hold
samples**: the window had no anchor to the wait's start, so the 40 Hz hold
stream trivially satisfied it — disarm would land ~100 ms into the 2 s neutral
descent, freezing moving legs (the hard verify-gate stayed intact; the
neutral-return step was silently gutted). The general lesson: a **fresh flat
stream can still be the WRONG evidence** — freshness checks answer "is this
data live?", not "is this data *about* the thing I'm waiting for?". Fixed:
the quiescence window is anchored to wait-step activation and requires
≥400 ms of NEW samples; validated by a synthetic-stream probe (7/7: the
descent holds it open, it passes 500 ms after landing). Notably, the
interactive probe could never have caught this — its fake `go_home` has no
echo dynamics (see the complementarity note below).

### Vacuous hold-identity guard (review HIGH #2)

The mid-hold action-identity guard was vacuous: `renderedActionKey` was
rewritten by renders during the 800 ms hold, so a hold begun as Disarm could
fire Arm if the contextual action flipped mid-hold. Fixed: identity pinned at
pointerdown (action key; node plan label) + an `element.isConnected` check at
confirm.

### Refuted HIGH: armed submode change vs runbook S4.3

The claim — "an armed submode change contradicts runbook S4.3 wording" — was
**mechanism-refuted**: all six submodes are in trajectory_node's streaming
set; streaming→streaming while armed is safe; Sharp Edge #1's mechanism is the
emitter *stopping*, which doesn't happen here. The runbook's S4.3 blanket
wording is stricter than its own mechanism — flagged as a doc note, not a code
change.

### Worst-case defaults for safety indicators (review fix cluster)

Freshness + worst-case-defaults fixes, all in-session: `bridge_link` consumed
as an **actuation guard** (a frozen `fault_state=NONE` is a lie while the
uplink is down — the same deception class as the CAN panel's HIGH);
`orchestrator_state`/`robot_state` staleness guards (>2 s ⇒ all grey); FAULT
recovery branches gated on freshness; `mpc_active` parse is tri-state
(unparseable ⇒ UNVERIFIED, never 'disarmed'); the worst-case armed rule
(stale link + last-known armed ⇒ an 'ARMED?' badge + an enabled best-effort
Disarm — never a hidden badge or a disabled Arm); Arm requires `fault_state`
exactly NONE (UNKNOWN blocked). The direction is uniform: when evidence is
missing, indicators degrade toward "assume the dangerous state and keep the
mitigating action available".

### Why the minimap is stricter than the command strip

BOOT/HOMING/LEVELLING grey everything with the reason "commands discarded" —
the orchestrator discards commands in those source states, so offering them is
offering a silent no-op. The command strip still has the known BOOT-Home
silent no-op; the minimap deliberately refuses to reproduce it. An unknown
state string greys everything (fail-safe) rather than guessing.

### Probe-vs-review complementarity

The two verification instruments caught disjoint bug classes, and neither
subsumes the other: the interactive probe's fake `go_home` has **no echo
dynamics**, so only code review could catch the quiescence-anchor bug
(HIGH #1); conversely, only the probe could pin the **live teardown order**
end-to-end — the recorded timelines show the negative case (mpc_active never
clears) aborting while naming Sharp Edge #6 with `deactivate` NEVER published,
and the positive case publishing `deactivate` only 62 ms after the armed flag
clears.

### Other review fixes (accepted and fixed in-session)

- **Pane-relative width clamp** — `min(280, paneWidth*0.55)` via
  ResizeObserver; the previous 40vw viewport clamp could zero the viewer with
  wide sidebars.
- **Greyed-label contrast** ≥4.3:1 in both themes (light-theme amber −700).
- **Graph faithfulness** — the BOOT→IDLE skip-if-homed edge added;
  commanded-direction arrowheads.
- **T_MODE/T_DEACT 5→15 s** — the ActiveHandler queues commands until the
  ACTIVATE move completes; the queued-command-after-abort residual is
  documented.
- Tripwire test derives consumed keys from source; init idempotence; the
  citation anchor corrected to the real arming-precondition lines.

### Deliberately not fixed (documented)

- **Keyboard/AT path** — pointer-only holds are the GUI-wide idiom; not
  introducing a divergent interaction model here.
- The sticky status line takes precedence over the disconnected note —
  per-node reasons carry the disconnect information.

## Implementation

- **`ros_ws/gui/js/state-minimap.js`** (NEW, ~1500 lines): constants + SVG
  layout tables, snapshot store, reachability, the plan table, the sequencer,
  the contextual button, rendering, init.
- **`ros_ws/gui/css/state-minimap.css`** (NEW): translucent/expanded states,
  FAULT pulse, greyed-node styling (contrast-checked both themes).
- **`ros_ws/gui/index.html` + `js/main.js`**: panel markup + module hookup;
  inputs ride the existing topic router (the slot the CAN panel reserved).
- **`tests/ros/test_gui_geometry.py`**: +2 EXPECTED_FILES cases + 4 tripwire
  tests (incl. disarm-before-deactivate ordering and go_home-before-disarm),
  consumed keys derived from source.
- **`tools/probes/gui_dom_probe.py` / `gui_synthetic_stack.py` /
  `README.md`**: new `--scenario minimap` (CDP pointer holds against real
  rosbridge + the scripted fake stack), with recorded safety timelines.

## Verification

- **GUI geometry / tripwire suite** (`pytest tests/ros/test_gui_geometry.py -q`,
  run 2026-07-11): **54 passed** (48 prior + 2 EXPECTED_FILES + 4 tripwires
  incl. disarm-before-deactivate ordering and go_home-before-disarm).
- **GUI JS**: node module syntax checks on all new/modified JS — clean.
- **Quiescence-predicate probe** (scratchpad, one-off): **7/7** — the synthetic
  descent stream holds the anchored window open; it passes 500 ms after
  landing.
- **Interactive synthetic-stack probe** (`python3 tools/probes/gui_dom_probe.py
  --scenario minimap`, CDP pointer holds against real rosbridge + the scripted
  fake stack): first green **27/27 twice pre-fixes**, with recorded safety
  timelines — negative case (mpc_active never clears) aborts naming Sharp
  Edge #6 with `deactivate` NEVER published; positive case publishes
  `deactivate` only 62 ms after the armed flag clears.
  **Post-fix re-run** (2026-07-11): minimap scenario **33/33 PASS, twice
  consecutively** (the 6 new assertions pin G_BRIDGE uplink-lost greying, the
  `ARMED?`+enabled-Disarm worst-case rule, and error-ingestion-before-FAULT
  determinism) and scenario1 regression **17/17 PASS**; the recorded teardown
  timelines now show the anchored quiescence gap (~0.5–0.6 s between `go_home`
  and disarm, vs ~0.04 s pre-fix) and `deactivate` ≤65 ms **after** the probe
  clears `mpc_active` (30/65 ms across the two green runs); teardown flags clean on every run.
- **Full suite** (`pytest tests/ -q`, run 2026-07-11, immediately pre-commit):
  **2478 passed, 1 xfailed in 597.42 s** (+6 vs the CAN-panel entry's 2472:
  two parametrized EXPECTED_FILES cases + four tripwire tests).

## Outcome

The orchestrator state machine is operator-visible and safely drivable from
the GUI: current state, per-node unreachability reasons, fewest-safe-steps
click-to-state with hold-to-confirm, and a contextual next-command button —
with the runbook's Sharp Edges enforced in the sequencer (anchored quiescence
wait + hard verify-gate; `deactivate` never published unverified) and
worst-case-default safety indicators throughout. Two review-confirmed HIGH
roots fixed in-session; one refuted with a mechanism argument (S4.3 doc note
flagged). Post-fix probe re-runs green (minimap 33/33 ×2 + scenario1 17/17)
and the pre-commit full suite is recorded in Verification. The MVP plan's
deferred 'orchestrator-automated arming' item is unobstructed: the minimap
ships manual Arm only, so nothing double-arms when that lands — adjust
contextual-button rule 6 then.

## Related

- [[2026-07-11-gui-can-traffic-per-bus-panel]] — sibling entry from the same
  arc; reserved the topic-router slot the minimap's inputs ride, and its
  bridge-republish HIGH is the same deception class as the minimap's
  `bridge_link` actuation-guard fix.
- [[2026-07-11-gui-leg-setpoint-echo-poscmd]] — sibling entry; source of
  `leg_setpoint_echo`, the minimap's `go_home` quiescence proxy.
- [`tests/hardware/mvp_bench_runbook.md`](../tests/hardware/mvp_bench_runbook.md)
  — the Sharp Edges the sequencer enforces; note the S4.3 wording-vs-mechanism
  doc note from the refuted HIGH.
- [[2026-07-02-canbridge-phase4-orchestrator-wiring]] — the orchestrator wiring
  whose state machine this minimap renders and drives.
- Plan: [`mvp-trajectory-bringup.md`](../plans/active/mvp-trajectory-bringup.md)
  — the deferred 'orchestrator-automated arming' item: the minimap ships
  manual Arm only (nothing to double-arm when that lands; adjust
  contextual-button rule 6 then).
