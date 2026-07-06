---
title: Parity-matrix reconcile vs the canhub-hardening arc + Phase-13 decommission pre-check (PARTIAL safety scan clean)
type: investigation
date: 2026-07-06
status: resolved
phase: "teensy-can-offload Phase 13 (pre-check)"
related_plan: teensy-can-offload.md
related_entries:
  - 2026-06-27-can-node-teensy-parity-audit
  - 2026-07-02-canhub-hardening-tier1
  - 2026-07-02-canhub-hardening-tier2
  - 2026-07-04-canbridge-stow-on-shutdown
  - 2026-07-05-deactivate-at-stow-noop-guard
  - 2026-07-05-canhub-hardening-18a-homing-result-uplink
  - 2026-07-06-canhub-health-of-busoff-wiring
files_changed:
  - ros_ws/docs/can-node-teensy-parity.md
commits:
  - 30f49b8
subsystem:
  - can
  - ros
tags:
  - safety
  - docs
  - decommission
---

# Parity-matrix reconcile + Phase-13 decommission pre-check

## Summary

One-last-pass reconciliation of `ros_ws/docs/can-node-teensy-parity.md` (the
definition-of-done for "the legacy `can_node` foundation is fully and robustly
ported") against everything landed since the last reconcile (2026-07-02): the
whole **canhub-hardening arc** (Tier 1 + Tier 2, archived
`plans/archived/2026-07-05 canhub-hardening.md`, flashed 2026-07-03,
powered-validated 2026-07-04), the **stow-on-shutdown port** (`25557e2`), the
**deactivate-at-STOW no-op guard** (`559c6dc`), the **[18A] HomingResult
uplink** (`2b749e3`, `PROTOCOL_VERSION` 1→2), the **marginal-CAN3 resolution +
presence gate** (`b8b6faf`, `9f8c3fc`), and the **health_of() bus-off wiring**
(`10beb10`). This is the Phase-A gate for the Phase-13 decommission of the
Jetson's legacy SocketCAN stack (`can_node.py`, `can/bus.py`, `python-can`).

**Counts: 79/12/23/3 → 85/9/20/3** (ported+validated / ported+unvalidated /
PARTIAL / GAP, of 117 capabilities).

**Decommission verdict: NO BLOCKER.** The scan of every PARTIAL row found no
silently-dropped `can_node` safety behaviour still open; the one genuine
dropped safety semantic discovered in this family since the audit (automatic
stow on clean shutdown) was surfaced by the operator on 2026-07-04 and closed
the same day (`25557e2`). The 3 GAPs are confirmed non-losses (details below).

## Row flips (each grep-verified against the current code, file:line in the matrix)

| Row | Capability | Was → Now | Closed by |
|-----|-----------|-----------|-----------|
| 53 | deactivate 'already stowed' short-circuit | PARTIAL → ported+validated | `559c6dc` `_legs_already_stowed` guard; hardware-validated 2026-07-05 both directions |
| 57 | `clear_disarm_reasons()` optimistic zeroing on clear | PARTIAL → ported+validated | Tier-2 [17] §4 (`83ac938`, flashed): `fault_machine.cpp:147-169`, NUM_AXES incl. hand; native test `test_fault_machine.cpp:706` |
| 19 | clean-shutdown automatic profiled stow | PARTIAL → ported+unvalidated | `25557e2` `_shutdown_stow()` (host-side by design — the 2026-05-19 inversion is preserved); 5 node tests; hardware Ctrl-C confirm pending |
| 1 | re-arm deferred-stow on mid-descent re-drop | ported+unvalidated → ported+validated | fault_golden `rearm_on_redrop` executes the REAL `fault_machine.cpp` natively (`test_fault_machine.cpp:320`, harness `5f07ad5`) |
| 17 | CAN-loss detection during blocking cold-start ops | ported+unvalidated → ported+validated | native mid-op CAN3-loss abort tests on the real TUs (`431ea89`) + 2026-07-05 hardware smoke (CAN3 dropped mid-HOMING → clean abort + re-home) |
| 31 | homing/activate/deactivate/stow mutual exclusion | ported+unvalidated → ported+validated | native interlock tests on the real TUs (`431ea89`); [16] MPC↔cold-start interlock hardware-observed 2026-07-04 (CHECK 4) |
| 55 | homing abort on fatal / bus-down / E-STOP | ported+unvalidated → ported+validated | native reject/abort tests (`test_leg_homing.cpp:84-145,:248` — ERR_BUS_DOWN/ERR_REJECTED rejects + mid-homing-loss→IDLE); [18A] host now trusts the firmware FAILED |

Also updated in place (no status change): row 42 (hand vel/curr-limit topic
parity restored + cached, `c7425e9`; leg half unchanged and dormant), row 22
(item-20 REBOOT-during-stow reject noted), row 20 + the CAN-loss domain
bullets (health_of() now classifies live FLTCONF — `10beb10`), the stale
"no compiled-C++ firmware test" caveats (native harness compiles the real
fault machine + cold-start TUs since `5f07ad5`/`431ea89` — the audit's claim
went stale two days after it was written), and the frozen-robot_state /
clean-shutdown-stow / six-leg-stow / CAN-loss-mid-cold-start domain-gap
bullets (each struck with its closing commit). The pre-commit `/audit
--unstaged` caught two more stale bullets of the same class (the
Open-uncertainties register-read-TODO note; the hand vel/curr-limit
adversarial note) plus a flash-date imprecision and a citation-range nit —
all fixed in the same pass.

## PARTIAL safety scan (the decommission gate)

Every remaining PARTIAL row was scanned for a silently-dropped hard-won
`can_node` safety semantic (vs a mere interface re-architecture). Verdict per
row — the 20 remaining PARTIALs: **10** (step gate re-expressed as pump gate +
firmware MAX_DEVIATION E-STOP + 0.15 rev lead clamp — equivalent-or-stronger,
two layers), **11** (garbled-mode-string trip gone, but the hazard it guarded
— mode-string parsing — no longer exists; ERROR-mode response replaced by the
firmware fault machine + the stream-stop chain), **16** (socketcan
restore/emergency-idle dropped because the failure mode ceased to exist —
FlexCAN self-recovers; terminal safe state equivalent; BUS_OFF now visible
end-to-end via `10beb10`), **22** (settle guarantee — convenience, and the
dangerous half was closed by the Phase-6 reboot latch + item-20 reject),
**23** (SDO read-back — diagnostic capability, sole safety consumer replaced),
**25 + 42** (runtime limit persistence — a real semantic drop but a **dormant
surface**: no live producer publishes `set_motor_vel_curr_limits` in the beta
launch; noted, not a blocker), **26** (per-leg heartbeat wait before configure
— partially substituted by the Phase-3 all-axes version handshake gating the
automated path), **30** (encoder-search slow-fail on mid-search fault — slow,
not unsafe: the search commands no leg motion), **35/46/48/61/64**
(diagnostic-fidelity/convenience losses, raw signals preserved), **41**
(torque_ff drop is decision D9 with a measured-null hardware penalty), **43**
(mode-driven axis-state management deliberately replaced by explicit services;
the automated caller was re-wired in Phase 4 and validated), **50** (mid-motion
/configure window — configure is motion-free and orchestrator-sequenced;
residual operator-error window accepted at low), **52** (redundant during-move
safety-net clamp dropped — defence-in-depth reduction, primary TRAP_TRAJ cap
hardware-proven), **56** (position seed relocated INTO activate/deactivate,
improved non-clipped form), **59** (positions-only ingress now rejected —
stricter, safety-positive).

**No decommission blocker.** None of these get worse by deleting `can_node` —
the fallback is already non-functional (the legs are on CAN3 behind the
can-bridge; `can0` is up-but-idle), so the deletion loses reference material
only.

## GAP dispositions (3)

- **Row 18 (UDP-link deferred-stow EXECUTION on reconnect)** — stays OPEN
  deliberately. A NEW-surface safety item for armed dynamic moves
  (teensy-can-offload Phase 12), not a `can_node` CAN-functional loss: the
  Jetson↔Teensy link did not exist when the Jetson owned CAN. Since Tier-1 [2]
  (`c7425e9`) the loss itself is orchestrator-visible on
  `robot_state.has_fatal_can_error`; only the post-reconnect auto-self-park is
  unported.
- **Row 47 (`platform_target_reached`)** — confirmed no live consumer (grep of
  ros_ws/controller/sim/tools/tests, non-archived: only the producer
  `can_node.py`, the conftest mock, and the `jugglebot_launch.py` rosbag
  record list — a dead-topic residue to clean with Phase 13). Cold-start
  completion is met via the activate/deactivate/home RPC returns.
- **Row 63 (`can_traffic`)** — accepted. No live consumer (grep: only
  `can_node.py`, the conftest mock, `test_can_node.py` — all deleted with
  Phase 13). Bus-load/health intent covered by the `profile` topic
  (`can1_rx`/`can1_util_pct`) + the 1 Hz `[canhealth]` line whose `err=` now
  counts real wire errors (`9f8c3fc`) with `[canerrs]`/`[canesr1]` attribution.

## Discussion

*(The judgment calls a future reader won't infer from the diff.)*

- **Why rows 1/17/31/55 flip on test evidence.** The matrix's own status
  definition says ported+validated = "concrete hardware/**test** evidence".
  These four rows were held at unvalidated because, at audit time
  (2026-06-27), their firmware gates rested on line-by-line review and
  hand-kept Python transcriptions that a diverging C++ edit would not trip.
  That evidentiary state changed materially: the native harness now compiles
  the REAL TUs (`fault_machine.cpp` since `5f07ad5`, the cold-start movers
  since `431ea89`) and asserts the exact semantics each row names (the golden
  `rearm_on_redrop`, the mid-op CAN3-loss aborts, the mutual-exclusion
  rejects). Rows whose evidence state did NOT change since their last
  deliberate assessment (e.g. 34 — the version-mismatch branch, host-unit-only
  then and now; 12/15/33 — relay persistence, awaiting a restore-path
  hardware run) were deliberately NOT flipped, to avoid re-litigating the
  matrix's standards mid-reconcile.
- **Why row 19 lands at unvalidated, not validated.** The stow-on-shutdown
  port has 5 node tests but its logbook is explicitly
  `fix-landed-pending-hardware-confirm`; a real Ctrl-C-while-elevated has not
  been operator-witnessed. The row's hazard is physical (platform left
  holding), so hardware is the right bar for "validated" here — matching how
  the hand rows were staged in Phase 5.
- **The scan's one near-miss family.** The only PARTIALs with any genuine
  safety texture are the limit-persistence pair (25/42) and the mid-motion
  /configure window (50). Both are dormant surfaces today (no live producer;
  no caller sequences a mid-move configure) and neither is affected by
  deleting `can_node` — recorded so the eventual dynamic-moves work re-audits
  them when those surfaces go live.
- **What made this reconcile bigger than the brief.** The task brief listed
  five hardening deliverables; enumerating `git log --since=2026-07-02`
  surfaced two more row-closing changes the brief didn't name
  (stow-on-shutdown `25557e2` → row 19; the Tier-1/Tier-2 native-harness and
  parity items → rows 1/17/31/55 + the stale global caveats). The lesson
  stands: reconcile against the commit log, not against the prompt's list.

## Verification

- Matrix arithmetic machine-checked (2026-07-06): gap-list rows 32/9/20/3 = 64;
  domain-table rows 85/9/20/3 = 117 (script over the status cells; both tables
  agree).
- Full suite (`pytest tests/ -q`, run 2026-07-06, doc-only change —
  pre-commit gate): **2054 passed, 1 xfailed in 473.15 s** (the 1 xfail is the
  pre-existing DOCUMENTED-PERMANENT marker from the archived MPC sad-path
  plan).
- Cited hardware/test evidence quoted from source entries: Tier-2 powered
  validation 2026-07-04 (7/7 PASS), deactivate-at-STOW hardware validation
  2026-07-05, [18A] flash + CAN3-drop smoke 2026-07-05, health_of() bus-off
  bench validation 2026-07-05/06.

## Related

- The matrix: `ros_ws/docs/can-node-teensy-parity.md` (this entry's only file).
- Phase-13 decommission (Phase B of this session) executes against this
  pre-check: delete `can_node.py` + `can/bus.py` + `python-can` (+ test/
  entry-point ripples), KEEP `can/odrive.py` + `can/catching_cone.py` (bridge +
  orchestrator import them), operator-run `can0` teardown.
