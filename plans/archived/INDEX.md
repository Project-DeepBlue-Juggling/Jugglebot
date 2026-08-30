# Archived plans

Completed or superseded plans. **The filename never changes for the life of a
plan** — archival is a `git mv` of the file unchanged, plus an `archived:
YYYY-MM-DD` frontmatter field and an Archival-note section. That is what keeps
every inbound `related_plan:` and every prose cross-reference resolving after
archival; the date prefix this directory used to carry broke all of them at
exactly the moment the plan stopped being editable. See
`../../DOCUMENTATION_GUIDE.md` § 2.6.

The chronology the old filename prefix used to give you lives here instead: this
table is sorted by `archived:`, and `git log --diff-filter=R --follow` has the
move. **Archiving a plan adds its row in the same commit** —
`tests/sim/test_plans_index.py` fails otherwise, in both directions.

The **Status** column is verbatim from each plan's own frontmatter and is *not*
normalised, so the drift is visible rather than hidden. Four 2026-07-05 entries
still read `active`, one reads `archived`, one has no status at all (`—`), and
`complete` / `completed` both appear — nothing updated them at archival back
when the date prefix carried the signal. They are left as-found rather than
silently rewritten: the correct verdict for each is a judgement call for whoever
next touches that plan, not a mechanical edit. The vocabulary going forward is
`completed | superseded` (`../../DOCUMENTATION_GUIDE.md` § 3).

| Archived | Plan | Status | Title |
|----------|------|--------|-------|
| 2026-03-18 | [ball-implementation.md](ball-implementation.md) | completed | Ball Tracking & Catch Coordination |
| 2026-03-18 | [codebase-rewrite.md](codebase-rewrite.md) | completed | Codebase Overhaul — ROS2 Layer Rewrite |
| 2026-03-28 | [simulation-development.md](simulation-development.md) | completed | MPC Simulation Development |
| 2026-03-30 | [mpc-oscillation-analysis.md](mpc-oscillation-analysis.md) | completed | Phase 3.1 Oscillation — Root Cause Analysis |
| 2026-04-23 | [hot-loop-zero-allocation-contract.md](hot-loop-zero-allocation-contract.md) | completed | Hot-Loop Zero-Allocation Contract (MPC 40 Hz) |
| 2026-05-08 | [friction-ff-motor-guard-integration.md](friction-ff-motor-guard-integration.md) | completed | Friction-FF Integration into motor_guard |
| 2026-05-08 | [motion-onset-deadtime-investigation.md](motion-onset-deadtime-investigation.md) | superseded | Motion-Onset Dead-Time — Investigation & Fix Plan |
| 2026-05-10 | [mpc-tier0-contracts.md](mpc-tier0-contracts.md) | completed | MPC Tier 0 — Scheduler & PlantInterface Contracts |
| 2026-05-18 | [mpc-sadpath-coverage-tiers-1-3.md](mpc-sadpath-coverage-tiers-1-3.md) | completed | MPC Sad-Path Coverage Rollup — Tiers 1–3 |
| 2026-07-05 | [can-process-refactor.md](can-process-refactor.md) | active | CAN Process Extraction Refactor |
| 2026-07-05 | [canbridge-foundation-coldstart-parity.md](canbridge-foundation-coldstart-parity.md) | archived | Can-bridge foundation seams — restore orchestrator-driven automatic cold-start (parity) |
| 2026-07-05 | [canhub-hardening.md](canhub-hardening.md) | completed | Can-hub Teensy firmware/UDP pipeline — hardening pass (Fable-5 fresh-eyes review) |
| 2026-07-05 | [HANDOFF-firmware-three-bus-WIP.md](HANDOFF-firmware-three-bus-WIP.md) | active | Teensy CAN Offload — Three-Bus Firmware Refactor Handoff |
| 2026-07-05 | [HANDOFF-teensy-can-offload-bridge-wip.md](HANDOFF-teensy-can-offload-bridge-wip.md) | active | Teensy CAN Offload — Jetson Bridge WIP Handoff (Phase 10b) |
| 2026-07-05 | [HANDOFF-teensy-can-offload-firmware-wip.md](HANDOFF-teensy-can-offload-firmware-wip.md) | active | Teensy CAN Offload — Firmware WIP Handoff |
| 2026-07-05 | [PROMPT-canbridge-rx-drain-throughput.md](PROMPT-canbridge-rx-drain-throughput.md) | — | Prompt: can-bridge CAN3 RX-drain throughput investigation |
| 2026-07-17 | [shaped-planning-efficiency.md](shaped-planning-efficiency.md) | completed | Plan: Shaped-planning efficiency — batched dense gate + retiming-invariant duration search |
| 2026-08-01 | [always-on-telemetry-daemon.md](always-on-telemetry-daemon.md) | deferred | Always-On Telemetry Daemon (ROS2-independent GUI feed via firmware tap) |
| 2026-08-01 | [dashboard-3d-mesh-and-sim-port.md](dashboard-3d-mesh-and-sim-port.md) | superseded | Dashboard 3D Mesh Upgrade + Sim Port |
| 2026-08-01 | [follower-cadence-and-divergence.md](follower-cadence-and-divergence.md) | superseded | Design proposal — SpaceMouse follower: knot-cadence protection and command-divergence bounding |
| 2026-08-01 | [hardware-bringup.md](hardware-bringup.md) | superseded | MPC Hardware Bringup |
| 2026-08-01 | [reload-action-catch-latch.md](reload-action-catch-latch.md) | completed | Plan — RELOAD as a self-contained action; retire CATCH & SHELL modes |
| 2026-08-15 | [bb-led-two-ball-juggle-demo.md](bb-led-two-ball-juggle-demo.md) | superseded | Ball-Butler-initiated two-ball oval juggling demo |
| 2026-08-15 | [bridge-temporal-trustworthiness.md](bridge-temporal-trustworthiness.md) | completed | Bridge Temporal Trustworthiness — closing the uptime command-latency drift and the clock-precision half in one arc |
| 2026-08-15 | [fk-convergence-tolerance.md](fk-convergence-tolerance.md) | complete | FK convergence tolerance — spurious "did not converge" at round-off-level residuals |
| 2026-08-15 | [hand-ball-sensor.md](hand-ball-sensor.md) | complete | Hand ball-present sensor — G02 wiring through firmware, protocol, ROS, GUI |
| 2026-08-15 | [lead-clamp-content-freshness.md](lead-clamp-content-freshness.md) | superseded | Content-freshness contract on the can-bridge lead clamp (DRAFT PROPOSAL) |
| 2026-08-15 | [teensy-can-offload.md](teensy-can-offload.md) | superseded | Teensy CAN Offload Architecture |
| 2026-08-15 | [tilt-calibration-grid.md](tilt-calibration-grid.md) | complete | Tilt calibration grid — pose-dependent gravity-level reference (residual map over the workspace) |
| 2026-08-21 | [hand-command-continuity.md](hand-command-continuity.md) | completed | Hand-command continuity — stop clobbering a live stroke (post-throw dip + throw truncation) |
| 2026-08-25 | [operator-observability.md](operator-observability.md) | completed | Operator observability quartet — chart units, UDP message rates, ODrive error propagation, QTM calibrate gate |
| 2026-08-25 | [udp-channel-health.md](udp-channel-health.md) | completed | UDP channel-health metrics — four independently-landable phases |
| 2026-08-30 | [toss-multi-catch-pose.md](toss-multi-catch-pose.md) | superseded | Multi-pose catch cycling — halted at the pre-M2 boundary, superseded outright by `plans/active/unified-7dof-planner.md`; M0–M6 never ran |

## Where the other plans are

- `plans/active/INDEX.md` — the active board: what is actually schedulable now.
- `plans/parked/INDEX.md` — deliberately not now, each with what would unpark it.
