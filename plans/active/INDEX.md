# Active plans

One row per plan in `plans/active/`. **Adding or archiving a plan updates this
table in the same commit** — `tests/sim/test_plans_index.py` fails otherwise.
Only *active* plan filenames may appear in this file; name an archived plan here
and the test fails (that is the point — it catches stale rows).

`Last touched` is the date of the last commit that changed the file.

| Plan | Status | Last touched | Scope |
|------|--------|--------------|-------|
| [accel-ff-inertia.md](accel-ff-inertia.md) | active | 2026-07-25 | Inertia-aware acceleration feedforward for lateral platform moves (wired behind a config flag; firmware prereq open) |
| [bb-led-two-ball-juggle-demo.md](bb-led-two-ball-juggle-demo.md) | active | 2026-07-31 | Ball-Butler-initiated two-ball oval juggling demo |
| [bb-online-juggle-tilt-rearchitecture.md](bb-online-juggle-tilt-rearchitecture.md) | active | 2026-07-04 | Online-juggle bring-up: clean catch → single-ball toss → two-ball, tilt-aimed |
| [bridge-clock-frequency-discipline.md](bridge-clock-frequency-discipline.md) | active | 2026-07-25 | Bridge clock rate discipline for µs-stable time across a multi-hour session |
| [catch-reach-degenerate-overshoot.md](catch-reach-degenerate-overshoot.md) | active | 2026-07-28 | Catch-reach near-degenerate overshoot (0.78° target → 2.32° excursion) |
| [fk-convergence-tolerance.md](fk-convergence-tolerance.md) | active | 2026-07-27 | FK convergence tolerance — spurious "did not converge" at round-off residuals |
| [hand-ball-sensor.md](hand-ball-sensor.md) | active | 2026-07-30 | Hand ball-present sensor: Phases 0–6 landed, Phase 7 gated on the hand-ODrive identity check |
| [hand-command-continuity.md](hand-command-continuity.md) | active | 2026-07-31 | Hand command continuity across the Jetson→Platform-Teensy conduit |
| [hand-trajectory-generator-overhaul.md](hand-trajectory-generator-overhaul.md) | active | 2026-07-31 | Jerk-limited, time-budget-parameterised hand trajectory generator |
| [learned-ff-residuals.md](learned-ff-residuals.md) | active | 2026-07-25 | Batch ILC on the leg torque channel (learned feedforward residuals) |
| [leg-gain-tuning-methodology.md](leg-gain-tuning-methodology.md) | reference | 2026-07-16 | Leg PID tuning methodology. The **gain hunt closed 2026-07-13** (40/0.20/0.32 ships, gains FROZEN); the document stays active as the normative procedure for any future tuning round |
| [levelling-frame-contract.md](levelling-frame-contract.md) | active | 2026-07-31 | One gravity/levelling correction applied to every pose request |
| [mvp-trajectory-bringup.md](mvp-trajectory-bringup.md) | active | 2026-07-31 | The live bringup plan: streaming trajectory path, S1–S8 hardware sittings |
| [refactor-2026-07.md](refactor-2026-07.md) | active | 2026-08-09 | 2026-07 refactor programme. Landed 2026-07-31/2026-08-01: Phases 0–3 (dead-weight sweep, process rules, nightly tier, MPC dormancy), 5 (partial — config drift gate), 6 slices 1–2 (analysis safety nets, structure cleanups), 7a (ERR_TIMEOUT recount). Phase 4 (teensy_link → repo root) landed, hardware smoke **CONFIRMED 2026-08-01 23:17** (LEVELLING end-to-end). 2026-08-02: Phase 7 attribution instrumentation SOFTWARE DONE (tx_deferred/BRIDGE_TX_DIAG/BRIDGE_IDENTITY, FW9 unflashed; sequence analysis = random, state bug excluded); **bench discriminator sitting DONE 2026-08-09** on FW 9 — idle 0/40 vs 500 Hz-stream 15/40 (Fisher p=8.5e-09), **congestion CONFIRMED** and the mechanism quantified (per-stage split = mailbox-occupancy readout; rate anomaly resolved by phase-locked dispatch quantisation — one link, the PERCLK clock source, was flagged INFERENCE, not read from source (RETIRED by the phase probe), and the phase-stamp probe is its falsifiable test), **Step 4 fix DECIDED, IMPLEMENTED, FLASHED and VALIDATED 2026-08-09 as FW 10 — the ERR_TIMEOUT epidemic is CLOSED**: `setMaxMB(16→24)` (8→16 TX mailboxes, jugglebot bus only) + a console-only `[handphase]` phase-stamp diagnostic, chosen as the only candidate with no 0x6D0 duplicate/invert hazard and the one that also makes the `events()` residual unreachable; wire-invisible (no MsgType, no payload, PROTOCOL_VERSION stays 5), so an FW 9 board decodes identically and a healthy link is not evidence the fix is aboard. **Validation 2026-08-09**: post-fix three-arm ladder **0/40 every arm = 120/120** (armB was 15/40; Fisher p=1.21e-05), `defer jb`/`txq` all 0, `can3_errors` zero and `bridge_fw_version` `10 (proto 5)` in all 120 rows; `[handphase]` gave **two tight clusters at ~60/~1060 µs**, confirming the two-phase quantisation model and **retiring the PERCLK INFERENCE flag**; `can1_tx` **~3150 fps unchanged**, **refuting the arbitration-scan hazard**; `interp_max_jitter_us ≤ 2`, `deadline_misses 0`. Plus an ordinary reload sitting with the hand responsive on every attempt. Latch fence DOWN (comment + docstring fix in `catch_coordinator_node.py`, mechanically verified ZERO EXECUTABLE CHANGE; the cap + telemetry-verified ladders retained deliberately as defense-in-depth). One residual: the vendored FlexCAN_T4 `events()` missing-`break`, dormant at `defer jb=0` but blast-radius DOUBLED by the fix — reachable only if a future TX producer re-opens deferral. Remaining: Phase 5 items 1/3, Phase 6 later slices, Phase 7 = 971d12c root-cause + the bridge-uptime lag experiment (plus two blocked bullets: the owner-DEFERRED re-plug probe and bounded FlexCAN retransmit, gated on the CAN3 repair) |
| [single-ball-toss.md](single-ball-toss.md) | active | 2026-07-29 | Single-ball self-toss capability and its follow-on programme |
| [teensy-can-offload.md](teensy-can-offload.md) | active | 2026-07-31 | can_node → Teensy CAN offload; re-scoped to a can_node↔Teensy parity audit |
| [tilt-calibration-grid.md](tilt-calibration-grid.md) | active | 2026-08-02 | Pose-dependent gravity-level reference: inclinometer grid capture → residual tilt map, layered on C-LEVEL-1 |

## Orchestration prompts

`PROMPT-*.md` files are self-contained session prompts for phase-runner
workflows rather than plans, so they are exempt from the table above:
`PROMPT-anomaly-fixes-orchestration.md`, `PROMPT-single-ball-toss-software-run.md`.
Completed prompts are DELETED (owner convention, 2026-08-09) — their arc lives in
the logbook, so the file adds nothing once its Done-means list is satisfied.
(First deletion: the ERR_TIMEOUT attribution prompt, authored 2026-08-01, every
Done-means item satisfied 2026-08-09 — see the ERR_TIMEOUT closure entry in the
logbook. Filenames deliberately not written here: the plans-index guard treats
any md name in this file as a live plans/active reference, which is exactly the
staleness protection we want to keep.)

## Recently archived

Five plans moved to `plans/archived/` on 2026-08-01 (follower-cadence,
reload-action-catch-latch, hardware-bringup, dashboard-3d-mesh, telemetry
daemon) — each carries an "Archival note" section explaining what shipped and
why it closed. Filenames are deliberately not listed here; see
`plans/archived/` and `plans/active/refactor-2026-07.md` § Phase 1.
