# Engineering Logbook

Structured record of investigations: symptom → diagnosis → fix → outcome.

Browse with `/logbook`, create entries with `/investigate` or `/logbook --new <title>`.

| Date | Status | Phase | Title | Entry |
|------|--------|-------|-------|-------|
| 2026-05-10 | resolved | mpc-tier0-contracts — Phase 8 (final) | MPC Tier-0 contracts — Phase 8 CI hypothesis profiles wired (Plan 1 closes) | [2026-05-10-mpc-tier0-phase-8-ci-hypothesis-profiles](2026-05-10-mpc-tier0-phase-8-ci-hypothesis-profiles.md) |
| 2026-05-10 | resolved | mpc-tier0-contracts — Phase 7 | K1–K6 reference contract — Phase 7 hypothesis retroactive expansion | [2026-05-10-k1-k6-hypothesis-phase-7-retroactive-expansion](2026-05-10-k1-k6-hypothesis-phase-7-retroactive-expansion.md) |
| 2026-05-10 | resolved | mpc-tier0-contracts — Phase 6 | Plant-interface contract — Phase 6 enforcement of P3, P4 | [2026-05-10-plant-interface-contract-phase-6-p3-p4-enforcement](2026-05-10-plant-interface-contract-phase-6-p3-p4-enforcement.md) |
| 2026-05-09 | resolved | mpc-tier0-contracts — Phase 5 | Plant-interface contract — Phase 5 enforcement of P1, P2 | [2026-05-09-plant-interface-contract-phase-5-p1-p2-enforcement](2026-05-09-plant-interface-contract-phase-5-p1-p2-enforcement.md) |
| 2026-05-09 | resolved | mpc-tier0-contracts — Phase 4 | Plant-interface contract — Phase 4 audit and P1–P4 draft | [2026-05-09-plant-interface-contract-phase-4-audit](2026-05-09-plant-interface-contract-phase-4-audit.md) |
| 2026-05-09 | resolved | mpc-tier0-contracts — Phase 3 | Scheduler contract — Phase 3 enforcement of S4, S5, S6 + RuleBasedStateMachine | [2026-05-09-scheduler-contract-phase-3-s4-s6-enforcement](2026-05-09-scheduler-contract-phase-3-s4-s6-enforcement.md) |
| 2026-05-09 | resolved | mpc-tier0-contracts — Phase 2 | Scheduler contract — Phase 2 enforcement of S1, S2, S3 | [2026-05-09-scheduler-contract-phase-2-s1-s3-enforcement](2026-05-09-scheduler-contract-phase-2-s1-s3-enforcement.md) |
| 2026-05-09 | resolved | mpc-tier0-contracts — Phase 1 | Scheduler contract — Phase 1 audit and S1–S6 draft | [2026-05-09-scheduler-contract-phase-1-audit](2026-05-09-scheduler-contract-phase-1-audit.md) |
| 2026-05-08 | resolved | pr-3b-persistent-enable | Friction-FF platform limit cycle — diagnosis and smooth-gate fix | [2026-05-08-friction-ff-platform-limit-cycle](2026-05-08-friction-ff-platform-limit-cycle.md) |
| 2026-04-27 | tuned | post-per-leg-gains-deadband-session | Friction feedforward — bench characterisation, model fit, and FF validation | [2026-04-27-friction-feedforward-bench-validation](2026-04-27-friction-feedforward-bench-validation.md) |
| 2026-04-23 | resolved | hardware-bringup — GC-pause elimination on the MPC 40 Hz hot loop | Hot-loop zero-allocation contract (W1 inventory → contract → enforcement → fixes) | [2026-04-23-hot-loop-zero-allocation-contract](2026-04-23-hot-loop-zero-allocation-contract.md) |
| 2026-04-20 | tuned | post-per-leg-gains-deadband-session | Motion-onset dead-time — cogging-torque-first investigation (continued in 2026-04-27 entry) | [2026-04-20-motion-onset-dead-time-fix](2026-04-20-motion-onset-dead-time-fix.md) |
| 2026-04-20 | resolved | hardware-bringup — platform-only dynamic-motion entry point | Add `--toss-motion` hardware test and `controller/ballistics` helpers | [2026-04-20-toss-motion-hardware-test](2026-04-20-toss-motion-hardware-test.md) |
| 2026-04-20 | resolved | | Add DOCUMENTATION_GUIDE.md as single reference for documentation architecture | [2026-04-20-documentation-guide-single-reference](2026-04-20-documentation-guide-single-reference.md) |
| 2026-04-20 | resolved | hardware-bringup — Phase 4 (moderate motions) | K1–K6 reference-feasibility contract resolves MPC_OVERSHOOT_SATURATION | [2026-04-20-k1-k6-reference-feasibility-resolution](2026-04-20-k1-k6-reference-feasibility-resolution.md) |
| 2026-04-20 | resolved | hardware-bringup — sim-side unlock for Phase 6/7 | Migrate sim catch sources to K1–K6 reference-feasibility contract and lower sim v_max to 500 mm/s | [2026-04-20-sim-catch-source-k1-k6-migration](2026-04-20-sim-catch-source-k1-k6-migration.md) |
| 2026-04-19 | resolved | hardware-bringup — Phase 4 (moderate motions) | Bundle A quintic-ref settling lag + ZMQ live-twist feedback trap | [2026-04-19-bundle-a-quintic-ref-settling-and-live-twist-trap](2026-04-19-bundle-a-quintic-ref-settling-and-live-twist-trap.md) |
| 2026-04-19 | in-progress | hardware-bringup — Phase 4 (moderate motions) | Bundle A — ref-from-plant-state on move transitions (MPC overshoot-saturation fix) | [2026-04-19-bundle-a-mpc-overshoot-saturation-fix](2026-04-19-bundle-a-mpc-overshoot-saturation-fix.md) |
| 2026-04-19 | resolved | post-per-leg-gains-deadband-session | Leg 1 pose-dependent hold-phase twitch at (0,-100,200) | [2026-04-19-leg1-pose-dependent-hold-twitch](2026-04-19-leg1-pose-dependent-hold-twitch.md) |
| 2026-04-18 | in-progress | STANDBY-mode MPC (post-64742f2 fallback walk-forward) — tail-latency attribution | MPC-loop overhead spikes trigger fallback bursts; audible "fighting" on every move | [2026-04-18-mpc-overhead-spikes-fallback-bursts](2026-04-18-mpc-overhead-spikes-fallback-bursts.md) |
| 2026-04-18 | in-progress | post-walk-forward-fallback-multiaxis-session-2 | Hold fighting + motion-onset jitter — residual hardware jitter downstream of clean MPC | [2026-04-18-hold-fighting-motion-onset-jitter](2026-04-18-hold-fighting-motion-onset-jitter.md) |
| 2026-04-18 | in-progress | post-walk-forward-fallback-multiaxis-session | Move 5: MPC overshoot-recovery stall + plant-collapse misclassification | [2026-04-18-move5-overshoot-stall-and-plant-collapse](2026-04-18-move5-overshoot-stall-and-plant-collapse.md) |
| 2026-04-17 | in-progress | STANDBY-mode multi-axis MPC | MPC fallback-command sawtooth produces visible stutter on off-Active multi-axis moves | [2026-04-17-mpc-fallback-cmd-sawtooth-stutter](2026-04-17-mpc-fallback-cmd-sawtooth-stutter.md) |
| 2026-04-15 | resolved | | ZMQ telemetry stale on :5556 — RECONNECT_IVL fix and staleness E-STOP | [2026-04-15-zmq-telemetry-stale-reconnect-ivl](2026-04-15-zmq-telemetry-stale-reconnect-ivl.md) |
| 2026-04-01 | resolved | 4.1 | Cold-hold fallback commanded stroke minimum | [2026-04-01-cold-hold-fallback-stroke-minimum](2026-04-01-cold-hold-fallback-stroke-minimum.md) |
| 2026-03-30 | resolved | 3.1 | Velocity feedforward oscillation | [2026-03-30-velocity-feedforward-oscillation](2026-03-30-velocity-feedforward-oscillation.md) |
