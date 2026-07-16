---
title: The gravity feedforward is armed, sign-verified on hardware, and ships enabled — the leg feedforward arc closes
type: investigation
date: 2026-07-16
status: resolved
phase: "Leg feedforward — ARC COMPLETE: gravity FF armed on hardware, A/B verified, shipped ON"
related_plan: leg-gain-tuning-methodology.md
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code/hardware_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - tests/motion/test_leg_torque_ff.py
  - tests/hardware/session_torque_ff.md
commits:
  - ff81a4d
subsystem:
  - motion
  - can
tags:
  - dynamics
  - safety
  - feedforward
---

# The gravity feedforward is armed, sign-verified on hardware, and ships enabled

## Summary

The operator ran the `session_torque_ff.md` A/B pair on 2026-07-16 morning:
session A (`torque_ff_enabled: false`, rosbag `2026-07-16_10-12-33`, 223 s)
and session B (`true`, rosbag `2026-07-16_10-17-45`, 140 s), identical
launch/activate/trajectory topology with a 13-move battery (z ±20 mm, then
tilts). **Every arming criterion passed**; the operator chose to ship the
feature enabled — `dynamics.torque_ff_enabled: true` is now the shipped
config, and the S2 tripwire test is inverted to pin it
(`test_shipped_config_has_the_feature_on`). This closes the leg feedforward
arc that began with the 2026-07-12 "not convinced we've found the right
gains" conversation: gains stayed at production 40/0.20/0.32, Kt was measured
(0.0570 ± 0.0008), the tff channel was verified (+18.35 A/Nm), and the
gravity term is now live on the wire.

## Evidence

Extraction harness: `tools/probes/gravity_ff_ab_extract.py` (committed —
reusable for the pre-registered follow-ups below; outputs to `temp/probes/`,
this run's metrics at `temp/probes/gravity_ff_ab_metrics_20260716.json`).
Ground truth: the two rosbags. Every headline number below was independently
re-derived by a second, separately-written script before being believed —
all matched to ≤0.002 A.

**The chain, end-to-end (session B):** arming banner with the expected
`wire scale 0.967251`; `torque_ff_ramp` 0 → 1.000 starting at the arm instant
and completing in exactly 2.00 s (80 frames @ 40 Hz, clean linear);
`setpoints_without_ff = 0` across all 5,058 post-arm setpoints;
`torque_clamp_mask = 0` (the firmware ingest backstop never bound);
`setpoints_rejected = 0`; `fault_state = NONE` throughout.

**S3 sign check — NO INVERSION.** Post-activate holds at the same pose
(matched to 0.0008 rev) with identical approach history: per-leg
`iq_measured` delta (B−A) = **[−0.025, −0.296, −0.137, −0.163, −0.150,
+0.083] A, leg-mean −0.115 A** (combined std 0.10–0.29 A). The inverted-sign
abort signature (+0.2–0.7 A rise on every leg) is absent; the small drop is
the session doc's "ideal signature". Corroborated at two other hold pairs
(leg-mean −0.005 A and −0.040 A).

**Zero motion at the arm edge**, through the 2 s ramp: max per-leg
peak-to-peak 0.0014 rev (encoder noise); max |vel| at quantization noise.

**The dynamic A/B: statistically identical**, as the session doc's
honest-expectation paragraph pre-registered. Across 11 pos-delta-matched move
pairs: peak |deviation| delta −0.0015 rev, post-move 2 s |deviation| integral
delta −0.0001 rev·s, paired settle-time delta ≈ 0. Physically consistent: the
pose-dependent gravity change over ±20 mm z is ~0.01 Nm (~0.2 A), buried
under 1–1.5 A friction/approach-history scatter, and move-time deviations sit
at the MAX_LEAD 0.10 rev clamp ceiling in both bags (symmetric censoring).
The FF's value — the integrator no longer carrying gravity — is a margin for
*loaded/dynamic* operation, not an unloaded-bench accuracy win.

**Bonus: the 2026-07-15 arming contract's first hardware validation.**
Session A launched with last night's stale MPC_STALE latch still on the
Teensy; the BOOT pre-flight cleared it silently (BOOT→IDLE in 2.6 s, no
HOMING wedge, zero seed-rejection spam). Auto-arm fired ~230 ms after
activate in both sessions; both shutdown stows completed cleanly from the
armed state.

## Anomalies / follow-ups (none blocking)

1. **`/leg_torques_diagnostic` recorded zero messages in both bags** — the
   topic is in the launch's rosbag record list but the emitter never
   publishes it. The Nm-level FF evidence therefore rests on the link-status
   flags + physics cross-checks (which suffice). Small observability gap —
   wire the emitter's computed torques to the topic in a future pass.
   **→ CLOSED same day**: `trajectory_node` now stashes each emitted frame's
   `torque_Nm` (one reference assignment on the 40 Hz hot path) and publishes
   it at 5 Hz on `leg_torques_diagnostic` (same Float64MultiArray
   type/semantic as `motion_bridge_node`'s MPC-path publisher), gated on
   streaming+seeded. Bonus empirical fact from its test: the legs do NOT
   share gravity equally at the active pose — per-leg FF spans
   0.0125–0.0394 Nm (leg 3 lowest), genuine geometry.
2. **Session B's first move drew the session's peak current** (leg 5: 7.35 A
   vs 3.51 A for the matched move in A) and engaged the lead clamp on all
   six legs (mask 63) for ~0.2 s. Isolated — not repeated across the other
   12 moves, below the 10 A soft max. Watch on the next session; if it
   recurs on first-move-after-armed-hold, suspect a stiction-breakaway +
   integrator-composition interaction.
3. **Approach-history hysteresis in hold currents is large** (same-pose holds
   within one bag differ by up to ±1.5 A per leg depending on the preceding
   move) — it dwarfs the 0.2–0.7 A sign-inversion signature. Future
   S3-style checks MUST compare identical-approach-history holds (this
   analysis did: post-activate vs post-activate).
4. The two batteries were not identical: A's move_seq 5 was a null move
   (commanded peak vel 0.0), making A.seq6 half-amplitude vs B.seq6 —
   pos-delta matching excluded the pair. Seq-index-only matching would have
   silently compared different moves.

## Verification

- Full suite (`pytest tests/ -q`, run 2026-07-16): **2803 passed, 1 xfailed
  in 592.97 s** — with the flag flipped, the tripwire inverted, the
  emitter test now pinning live gravity torques in the 0.013–0.041 Nm band,
  and `test_config_master_flag_gates_the_mpc_producer` decoupled from the
  shipped value (it now patches the master flag off explicitly, preserving
  the 2026-07-14 AND-gate regression regardless of what ships).
- Notably: the hot-loop allocation contract passed with FF-on — the first
  time it has measured the production FF-enabled `set_pose` path.
- A `/audit --unstaged` round before this commit caught: the shipped-off
  pin in the hardware-plant test (would have been the suite's one red),
  **nine** stale "ships false / default-OFF" claims across the YAML header,
  `docs/motion_planner/dynamics.md`, the pump/emitter/torque_ff docstrings,
  `run_mpc.py`, and the bridge (all swept — the docs now say the wire is
  live), the volatile scratchpad reference this Evidence section originally
  carried (fixed by promoting the harness to `tools/probes/`), and the
  session doc's re-arming dead-end (mirrored-tripwire note added).

## Discussion

**Why ship it on despite a null dynamic A/B.** The pre-registered expectation
(session doc S5) was exactly this: on an unloaded platform the gravity FF is
smaller than the friction floor and cannot improve hold or arrival numbers.
The criteria that mattered — sign, ramp, zero-motion arm, chain integrity,
no regressions — all passed with margin. Shipping now (a) banks the verified
state before anything drifts, (b) puts the integrator-relief margin in place
before the loaded/BB-reload work that motivated the arc, and (c) converts the
tripwire test into a guard in the opposite direction: disarming is now the
deliberate, logged act.

**Why the operator's "current draw looked roughly equivalent" was the right
observation.** At a static hold the velocity integrator re-pins total current
at the load regardless of the FF — equal totals are the *correct* outcome;
only the composition shifts (invisible in `iq_measured`). The 2026-07-15
edge-capture chapter established this ("static-hold steady-state FF response
is exactly zero"); this session confirmed its corollary on the full platform.

**Methodology note.** The A/B extraction was run by one script over both bags
(identical treatment), then every headline number was re-derived by an
independent second script before being believed — the 2026-07-13
sanity-check-magnitudes lesson applied. The pos-delta move matching (rather
than seq-index matching) caught the null-move battery divergence that would
otherwise have corrupted the comparison.

## Related

- `tests/hardware/session_torque_ff.md` — the procedure of record, now
  marked COMPLETED with results.
- `logbook/2026-07-15-kt-first-measurement-and-tff-channel.md` — Kt 0.0570
  shipped + tff channel verified (the measurement chapter).
- `logbook/2026-07-14-kt-reconciliation-and-gravity-ff.md` — the FF
  implementation + adversarial review chapter.
- `logbook/2026-07-15-arming-contract.md` — the arming choreography this
  session ran on (and validated).
- Rosbags: `~/Desktop/rosbags/2026-07-16_10-12-33` (off),
  `~/Desktop/rosbags/2026-07-16_10-17-45` (on).
