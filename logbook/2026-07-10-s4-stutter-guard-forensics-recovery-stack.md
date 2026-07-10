---
title: S4 stutter forensics (~6 Hz servo limit cycle), MAX_DEVIATION latch + failed recovery, and the recovery stack that came out of it
type: investigation
date: 2026-07-10
status: resolved (software; bench validation of the recovery stack + the gain retune pending)
related_plan: mvp-trajectory-bringup.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py (guard-freeze + profiled descent + reseed service)
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py (P0 hardening; guard repeat-ERROR; shutdown stow; /recover; v3 parsing)
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py + state_machine.py (guard_latched → FAULT, mode-preserving)
  - ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py + mocap_interface.py (log ergonomics)
  - ros_ws/src/jugglebot/Teensy_code_canbridge/ (v3 firmware — FLASHED 2026-07-10)
  - config/generate_udp_protocol.py + generated copies (protocol v2→v3)
  - ros_ws/requirements.txt (tornado pin — rosbridge/GUI fix)
  - tests/hardware/session_gain_retune.md (new) + mvp_bench_runbook.md (S4b)
  - plans/active/leg-gain-tuning-methodology.md (fast-motion tier registered)
commits:
  - 8692506   # P0 bridge hardening (parallel session's work, committed this session)
  - cf8728b   # docs: S1-S3 results + S3 resolution + S4 ladder
  - 5daf53e   # the main change set (recovery stack + firmware v3 + shutdown + ergonomics + rosbridge)
  - cd64d54   # gain-retune session protocol (S4b) + fast-motion gain tier
---

## Symptom

Evening bench session 2026-07-10 (rosbag `~/Desktop/rosbags/2026-07-10_18-06-03/`):
operator ran enlarged vertical strokes (z 170→250→50→170, his uncommitted battery
edit) at the top of the recommended limit ladder (156 / 660 / 10 500). Large,
particularly vertical, movements were "quite stuttery"; the session ended when the
Teensy fault guard latched during an otherwise normal-looking vertical move.
`ros2 service call /clear_errors` did not recover it; subsequent spacemouse commands
had no physical effect. Operator judgment (correct, it turned out): the commanded
motion was well within the robot's physical capability.

## Investigation

Method: one extraction agent (bag → CSVs + per-node logs + FACTS timeline), then
three parallel Opus analysts — empirical stutter characterization, end-to-end
control-architecture review, guard/recovery trace — against a pre-registered
six-hypothesis plan (H1 ODrive vel-limit saturation, H2 40 Hz C1 join
discontinuities, H3 CAN3 delivery jitter, H4 Jetson emit gaps, H5 firmware lead
clamp, H6 structural resonance). Key findings, all evidence-cited in the agents'
reports (scratchpad artifacts; headline numbers reproduced here):

- **The stutter is a ~6 Hz leg-servo limit cycle.** Lomb-Scargle on the big-stroke
  position residuals: fundamental 5.9–6.1 Hz on all six legs, strong 2nd harmonic
  ~12.3 Hz, and — decisive — the frequency does NOT scale with commanded speed
  (corr 0.06 across 37 moves). 6.1 Hz ≈ the ODrive position-loop bandwidth
  `pos_gain/2π = 40/6.28 = 6.37 Hz`. The ODrives actively drive it: `iqset` slams
  −3.8→+5.9 A cycle-by-cycle with braking current at each collapse; leg 0
  momentarily REVERSES against a commanded up-stroke; 58 % of the latch move is
  spent near-stationary while commanded at 2.1 rev/s. The config's own comment
  block records that `pos_gain 40 / vel_gain 0.20 / vel_int 0.32` were tuned for
  quiet HOLD, never fast tracking — and the TRAP_TRAJ activation move runs the same
  motors smoothly at 2.75 rev/s, exonerating the mechanics.
- **The operator's 40 Hz question was answered quantitatively, and 40 Hz is
  exonerated**: the Teensy cubic-Hermite reconstruction tracks the true quintic to
  22 µm; the per-join velocity-feedforward step is 6.1 mm/s (4 % of peak, matching
  the analytic `0.5·T·a_peak`); the stutter spectrum has essentially zero power at
  40 Hz; and the CAN3 setpoint load (500 Hz × 6 legs ≈ 39 % of 1 Mbps) is
  knot-rate-independent because the interp ISR transmits every 500 Hz tick
  regardless. Raising the knot rate buys nothing for this failure; a C2/quintic
  firmware interpolant (join step → exactly 0) is worthwhile polish only.
- **H4/H3/H1a/H6-as-primary all killed by data** (emit gaps flat at 25 ms,
  follow_block 0 during TRAJECTORY; setpoints_rejected 0; peak measured leg vel
  1.5–2.1 rev/s ≪ the 4.0 vel_limit, no ceiling sawtooth; behaviour scales with the
  command path, not fixed-amplitude). H5 (lead clamp) implicated as an aggravator:
  `leg_interp.cpp` ZEROED vel_ff exactly when the clamp engaged, and
  `pos_gain × MAX_LEAD = 40 × 0.15 = 6 rev/s` exceeds the 4.0 vel_limit — a
  designed-in bang-bang kick. Loop-vs-structure at 6 Hz is the one question this
  bag cannot settle (single gain point); the pos_gain sweep in the S4b session is
  the discriminant.
- **The MAX_DEVIATION latch (18:13:23.9) was legitimate.** The guard compares the
  RAW commanded knot u0 against the encoder (`fault_machine.cpp`), while the ODrive
  only ever sees the lead-CLAMPED command — so the pipeline looked healthy
  (`setpoints_rejected 0`, ODrives CLOSED_LOOP, zero errors) while u0 ran ~0.49 rev
  ahead of a stuttering, stalling leg on the fastest vertical up-stroke. The
  stutter's position ripple itself (~1 mm) is two orders below the 0.5 rev
  threshold; the trip is the *integral* of the stutter's velocity deficit.
- **The failed recovery was a designed-in catch-22, code-confirmed.**
  `trajectory_node` had no guard-state subscription: after the latch it kept
  advancing (move_seq 37→39, then spacemouse targets to z→30) against legs frozen
  at z≈238 — divergence reached 2.4–2.8 rev. `/clear_errors` WAS forwarded and DID
  clear, but the still-diverged u0 re-latched within one 10 Hz fault tick
  (invisible at the 10 Hz `/link_status` rate). The TRAJECTORY→SPACEMOUSE switch
  never re-seeds (both in the streaming set); `/trajectory/hold` seeds from the
  COMMANDED state; the orchestrator's `ctx.errors` reads only ODrive-level errors,
  so it stayed blind and kept accepting mode commands for a frozen robot.

## Discussion

**Why the analysts' two mechanism stories were allowed to stand side by side.** The
stutter lens says "under-damped position loop ringing at its own bandwidth"; the
architecture lens says "lead-clamp bang-bang (vel_ff zeroing + P-term > vel_limit)".
Both fit the data; they overlap in remedy (retune vel/pos gains; fix the clamp) and
differ only in which experiment kills which — so we shipped the clamp fix (cheap,
firmware, strictly removes a discontinuity) and pre-registered the pos_gain sweep as
the discriminant rather than picking a winner by narrative. The forensics plan's
MEASURED/INFERRED/HYPOTHESIZED discipline is what kept this honest.

**Why the guard threshold was NOT raised and the 40 Hz rate NOT changed.** Both were
tempting "fixes" for the symptom as reported. The guard fired on a real 0.5 rev
command-plant divergence — raising it deletes a safety net to mask a servo defect.
And the knot rate was measured to be irrelevant to this failure (22 µm fidelity,
zero 40 Hz spectral power) — changing it would have been the classic
plausible-but-wrong fix. The operator's framing ("tailor the scheme to the robot,
not an arbitrary choice") was exactly right — but the robot-mismatched parameter
turned out to be the servo gains, not the rate.

**The recovery design principle: never stop the stream, never step the command.**
Every piece of the recovery stack follows from two constraints discovered in this
(and the S3) forensics: stopping the 40 Hz stream while armed trips MPC_STALE
(250 ms), and stepping u0 is forbidden by three nested gates (pump 0.3 rev,
firmware 0.5 rev, physical). Hence: on a latch the trajectory_node freezes target
acceptance but the emitter KEEPS streaming; the command collapses onto the frozen
encoder via a gate-validated PROFILED DESCENT (the plant cannot move — output is
suppressed — so this is a command-space collapse only); `/recover` waits for
convergence before firing CLEAR_ERRORS. The instant-reseed the original fix spec
called for was discovered by the implementing agent itself to be blocked by the
pump's step gate (0.3 < 0.5: a genuinely diverged reseed can never pass) — it
shipped the safe fail-closed version and flagged the gap instead of rushing a
motion-adjacent design change unsupervised; the descent landed as a follow-up with
production-in-the-loop tests (every descent knot through a real SetpointPump).

**The BLOCKING catch: observability-FAULT must not change the control mode.** The
first orchestrator integration (guard_latched → FAULT) reused FaultHandler's
existing `control_mode='ERROR'` publish. The adversarial diff review traced the
choreography end-to-end and showed it silences the emitter ~100–200 ms after the
latch (ERROR is outside the streaming set), abandoning the in-flight descent,
re-latching MPC_STALE on top, and deadlocking `/recover` — strictly worse than no
fix. The landed design makes a guard-only FAULT preserve the streaming mode
(visibility + clear_errors routing only), promotes to the real-fault path if an
actual ODrive error joins, and exits straight back to ACTIVE with a one-shot
resume-armed flag (re-running `activate` would fire TRAP_TRAJ against the live
stream — two setpoint sources fighting). This is the second time in three days a
cross-process choreography bug survived every mocked per-node test and was caught
only by an agent tracing the real message ordering (cf. the Phase-7 audit arc);
the two-node hand-delivered-messages test pattern added here is the partial
mitigation.

**Honesty about what the firmware change does and does not fix.** MAX_LEAD
0.15→0.10 bounds the P-term at exactly vel_limit; with vel_ff now passed through
(capped 3.5 rev/s) the TOTAL setpoint can still saturate vel_limit while
clamped-and-advancing. The immediate win is boundary continuity (no vel_ff
discontinuity at clamp engage). Sprint elimination — and the stutter itself —
depends on the bench gain retune (S4b). Expect large fast vertical strokes to still
stutter until then; they will now recover instead of killing the session.

**One commit for six changes** (5daf53e): the changes share `teensy_bridge_node.py`
across four of six concerns, and the firmware was already flashed — an "independent
rollback" of entangled hunks would be fictional. The commit message carries the
per-change sections; this entry carries the reasoning.

**Attribution note**: commit 8692506 (P0 arming-refusal + `/link_status` recording)
is the parallel S3-followup session's work, committed this session to unblock the
bridge changes — it had already flown two hardware sessions uncommitted. The
`/link_status` recording it added is the single reason the guard forensics had
ground truth at all.

## Fix

Landed across 8692506 / 5daf53e / cd64d54 (details in the commit messages):
guard-latch recovery stack (freeze + profiled descent + orchestrator
mode-preserving FAULT + one-call `/recover`); firmware v3 (vel_ff kept at clamp,
MAX_LEAD 0.10, per-leg deviation + lead-clamp mask + latch snapshot telemetry) —
FLASHED; bridge-owned Ctrl-C disarm→settle→stow; log ergonomics (guard 5 s
repeat-ERROR, motor_guard 10 s summaries, QTM transitions-only); rosbridge/GUI
fixed (venv-invisible tornado → pinned + installed); S4b gain-retune protocol
(3-point pos_gain discrimination, then vel_gain damping A/B; each point costs a
YAML→codegen→build→relaunch cycle — there is no runtime leg-gain service).

## Verification

- Full suite in the worktree (`pytest tests/ -q`, run 2026-07-10): **1 failed
  (stale PROTOCOL_VERSION pin — the intended v3 bump) + 2366 passed, 5 skipped,
  1 xfailed in 755.46 s**; after the pin update the scoped `tests/teensy_link`
  suite passes **188/188** (run 2026-07-10). `tests/ros` standalone: **835
  passed** (run 2026-07-10).
- Firmware: pio native tests pass (test_leg_interp 12 cases, test_fault_machine
  13 cases, run 2026-07-10); built + flashed; post-flash
  passive listen decoded 100 HeartbeatT2J frames in 10 s at the v3 73-byte layout
  with all new fields present. xlang wire-hash re-pinned and independently
  recomputed by the reviewer.
- Adversarial diff review: 2 findings confirmed (1 BLOCKING choreography deadlock,
  1 MEDIUM comment overclaim), both fixed same-session; 16 areas checked sound
  (protocol coherence across all four copies, fail-safe mixed-version window,
  pump-passable descent knots, emitter-never-stops, executor safety, thresholds
  intact, hot-loop allocation discipline).
- NOT yet verified (operator sittings): the Ctrl-C stow scenarios A/B/C, the
  live `/recover` recovery, GUI-over-rosbridge end-to-end, and the S4b gain
  retune — runbook + `session_gain_retune.md` carry the exact recipes.

## Related

- The SAME-DAY morning arc — S3 chase-clamp follower rework (commit 73dba2b: 0
  rejects/0 parking on the recorded S3 stream replay; S3 re-fly PASS) — is
  documented in `plans/active/follower-cadence-and-divergence.md` § RESOLUTION
  (kept there because that document IS the proposal it resolves; this entry stays
  scoped to the evening arc).
- `tests/hardware/session_gain_retune.md` (S4b), `tests/hardware/mvp_bench_runbook.md`
  (S3/S4/S4b, sharp edges, recovery basics), `plans/active/leg-gain-tuning-methodology.md`
  (fast-motion tier).
- Forensic artifacts: session scratchpad `s4_extract/` + analyst reports (volatile,
  /tmp — headline numbers preserved here and in the commit messages by design).
