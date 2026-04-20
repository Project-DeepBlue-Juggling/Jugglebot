# Engineering Logbook

Structured record of investigations: symptom → diagnosis → fix → outcome.

Browse with `/logbook`, create entries with `/investigate` or `/logbook --new <title>`.

| Date | Status | Phase | Title | Entry |
|------|--------|-------|-------|-------|
| 2026-04-20 | resolved | | Add DOCUMENTATION_GUIDE.md as single reference for documentation architecture | [2026-04-20-documentation-guide-single-reference](2026-04-20-documentation-guide-single-reference.md) |
| 2026-04-20 | resolved | hardware-bringup — Phase 4 (moderate motions) | K1–K6 reference-feasibility contract resolves MPC_OVERSHOOT_SATURATION | [2026-04-20-k1-k6-reference-feasibility-resolution](2026-04-20-k1-k6-reference-feasibility-resolution.md) |
| 2026-04-19 | in-progress | hardware-bringup — Phase 4 (moderate motions) | Bundle A quintic-ref settling lag + ZMQ live-twist feedback trap | [2026-04-19-bundle-a-quintic-ref-settling-and-live-twist-trap](2026-04-19-bundle-a-quintic-ref-settling-and-live-twist-trap.md) |
| 2026-04-19 | in-progress | hardware-bringup — Phase 4 (moderate motions) | Bundle A — ref-from-plant-state on move transitions (MPC overshoot-saturation fix) | [2026-04-19-bundle-a-mpc-overshoot-saturation-fix](2026-04-19-bundle-a-mpc-overshoot-saturation-fix.md) |
| 2026-04-19 | resolved | post-per-leg-gains-deadband-session | Leg 1 pose-dependent hold-phase twitch at (0,-100,200) | [2026-04-19-leg1-pose-dependent-hold-twitch](2026-04-19-leg1-pose-dependent-hold-twitch.md) |
| 2026-04-18 | in-progress | STANDBY-mode MPC (post-64742f2 fallback walk-forward) — tail-latency attribution | MPC-loop overhead spikes trigger fallback bursts; audible "fighting" on every move | [2026-04-18-mpc-overhead-spikes-fallback-bursts](2026-04-18-mpc-overhead-spikes-fallback-bursts.md) |
| 2026-04-18 | in-progress | post-walk-forward-fallback-multiaxis-session-2 | Hold fighting + motion-onset jitter — residual hardware jitter downstream of clean MPC | [2026-04-18-hold-fighting-motion-onset-jitter](2026-04-18-hold-fighting-motion-onset-jitter.md) |
| 2026-04-18 | in-progress | post-walk-forward-fallback-multiaxis-session | Move 5: MPC overshoot-recovery stall + plant-collapse misclassification | [2026-04-18-move5-overshoot-stall-and-plant-collapse](2026-04-18-move5-overshoot-stall-and-plant-collapse.md) |
| 2026-04-17 | in-progress | STANDBY-mode multi-axis MPC | MPC fallback-command sawtooth produces visible stutter on off-Active multi-axis moves | [2026-04-17-mpc-fallback-cmd-sawtooth-stutter](2026-04-17-mpc-fallback-cmd-sawtooth-stutter.md) |
| 2026-04-15 | resolved | | ZMQ telemetry stale on :5556 — RECONNECT_IVL fix and staleness E-STOP | [2026-04-15-zmq-telemetry-stale-reconnect-ivl](2026-04-15-zmq-telemetry-stale-reconnect-ivl.md) |
| 2026-04-01 | resolved | 4.1 | Cold-hold fallback commanded stroke minimum | [2026-04-01-cold-hold-fallback-stroke-minimum](2026-04-01-cold-hold-fallback-stroke-minimum.md) |
| 2026-03-30 | resolved | 3.1 | Velocity feedforward oscillation | [2026-03-30-velocity-feedforward-oscillation](2026-03-30-velocity-feedforward-oscillation.md) |
