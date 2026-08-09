# Bench session — ERR_TIMEOUT hand-path discriminator (Step 3)

## Purpose

Attribute the ~50% HAND_TRAJ_CMD ERR_TIMEOUT rate with the new counters, and run
the WITH/WITHOUT-leg-stream discriminator. Established offline (do not re-derive):
every failure is a Teensy-returned ERR_TIMEOUT from one of hand_ops' three
can_jugglebot_send calls; write()==-1 means DEFERRED to the 64-slot software TX
ring (not dropped) — verified in FlexCAN_T4 source; per-dispatch outcomes are
statistically independent (no state bug; Step 1 sequence analysis, 399 outcomes);
per-frame deferral implied ~21% under independence.

## Predictions this session tests

| Hypothesis | Prediction |
|---|---|
| TX mailbox/queue congestion from the 500 Hz leg stream | Arm B (stream ON) fail rate >> Arm A (idle); tx_deferred_jb delta tracks hand per-stage fails; tx_q_hwm_jb rises in Arm B |
| Deferral-not-drop (the lying-ack resolution) | fail_teensy acks WITH the hand still physically arming/moving (kind-3 null moves complete despite failed acks) |
| Something else (both arms fail equally at ~50% with tx_deferred ≈ 0) | The mechanism is NOT write()-deferral — escalate before Step 4; do NOT fit a fix |

## Safety (operator-owned)

- REBOOT the can-bridge Teensy before the session (standing rule). Log uptime_ms
  with EVERY measurement block (standing rule — the uptime-lag degradation).
- Hand dispatches energise the hand ODrive (preamble = CLOSED_LOOP). Hand strokes
  here are ZERO-DISTANCE smooth moves (target = current position) — mechanically
  null but treat the hand as live at all times. Safe-state between blocks.
- NEVER dispatch while a catch sequence could be live (kind-3 erases the packed
  queue — last-writer-wins). No reload/catch actions this session.
- 48 V per operator judgement: the legs need not move in Arm B if a null/hold
  trajectory is used, but the CAN partners must be alive (ODrive heartbeats feed
  partner_recent; a dark bus measures tx_gated, not tx_deferred — that would be a
  DIFFERENT experiment and invalidates the arm).

## Procedure

Counters snapshot = one line each of /link_status rows: hand_traj_acks,
bridge_tx_diag, can3_errors, uptime_ms (ros2 topic echo is flaky on this box —
use the GUI or `ros2 topic echo --once /link_status` retried, or the tally the
ladder script prints).

0. Deploy preconditions (once, before the session):
   - Firmware: flash the bridge to FW 9 (pio run -e teensy41 -t upload from
     Teensy_code_canbridge — operator runs the flash; close any pio device
     monitor first, it holds /dev/ttyACM0). The a00d984 staleness guard makes
     a stale-header flash structurally unlikely; still wire-verify.
   - Jetson: cd ros_ws && colcon build --packages-select jugglebot — the node
     runs from the INSTALL copy (unlike teensy_link/, which the launch runs
     live), so BOTH the hand_traj_acks row and the new bridge rows need this.
   Then: reboot bridge Teensy, launch jugglebot_launch, confirm
   bridge_fw_version reads '9 (proto 5)' (BRIDGE_FW_CHECK OK) and
   bridge_tx_diag is present (not 'unknown (never seen)').
1. Snapshot counters (block start) + uptime_ms.
2. ARM A (no leg stream — idle launch, mpc_active=0, setpoints_sent static):
   run the dispatch ladder: N=40 smooth_move_hand calls to the current hand
   position, >=2 s apart (outside any plausible retry window). Record the
   script's tally + counter snapshot after.
3. ARM B (leg stream ON): operator starts a benign platform trajectory
   (slow hold/breathing motion — operator's choice; setpoints_sent must be
   climbing at ~40 Hz and the 500 Hz CAN ladder active). Repeat the identical
   N=40 ladder. Snapshot after. Stop trajectory.
4. ARM A' (repeat Arm A): N=40 idle again — controls for within-session drift
   (the unexplained prime-path wall-clock drift from Step 1; A vs A' separation
   with B between them is the cheap test of it).
5. Optional if time: N=40 with the stream ON at a HIGHER-load trajectory.

Per-arm capture: uptime_ms at start/end; hand_traj_acks delta; per-stage
(hand_pre1_fail/hand_pre2_fail/hand_traj_fail) deltas; tx_deferred_jb delta;
tx_q_hwm_jb; can3_errors (must stay all-zero — a nonzero wire-error count
invalidates the congestion read and reopens the bus-health question).

tx_q_hwm is BOOT-MONOTONE with no reset RPC: whichever arm drives it highest
pins it for the rest of the boot. The arm order above (idle A before loaded B)
is deliberate — read the per-arm INCREMENT, and never read a pinned value from
an earlier arm as evidence about a later one. Rebooting between arms is the
clean alternative if the increments are ambiguous.

## Interpretation (pre-registered)

| Observation | Conclusion |
|---|---|
| B fail-rate >> A, A ≈ A' ≈ low; tx_deferred delta ≈ 3×(per-stage fail sum) | Congestion CONFIRMED; proceed to Step 4 candidates (mailbox allocation / burst scheduling / bounded retry) |
| A ≈ B ≈ ~50% with tx_deferred climbing in both | Deferral confirmed but NOT stream-driven — the background TX census (0x7DD + polls) suffices; re-examine mailbox config before any fix |
| A ≈ B ≈ ~50% with tx_deferred ≈ 0 | Mechanism REFUTED on hardware — the ERR_TIMEOUT source is not write() deferral; STOP, re-open the attribution with the per-stage counters' evidence |
| fails concentrated in hand_pre1_fail vs hand_traj_fail | Different Step 4 shapes: preamble-fail = hand never in CLOSED_LOOP (operational story: stroke never armed); traj-fail = armed-state ambiguity (the latch premise cell) |
| A' >> A (drift within one bench session, no catches) | The Step 1 prime-drift signal is bench-reproducible — becomes its own investigation thread |

PASS for the sitting-validation gate (Step 4, later): dispatch failure rate far
below the 47-52% baseline over >= 40 dispatches. ABORT the bench session if: any
guard E-STOP, any can3_errors wire-error tick, any unexpected hand motion
(dispatches are supposed to be null moves), or the launch loses the bridge link.

## Artifacts

Ladder script: tools/probes/hand_dispatch_ladder.py (rclpy; prints per-dispatch
outcome + counter snapshots; logs to temp/probes/hand_dispatch_ladder_<ts>.csv).
Session log: this file's Results section, filled by the operator/next session.

## Results — 2026-08-09

Run on FW 9 (`bridge_fw_version` = `9 (proto 5)` in all 127 logged rows), bridge uptime
369 s at armA start → 660 s at armA2 end (a deliberately FRESH plant, 6.2–11.0 min
post-reboot). N = 40 dispatches per arm, ≥ 2 s apart, zero-distance smooth moves.

| arm | leg stream | fails | rate (95 % CP) | pre1 | pre2 | traj |
|---|---|---|---|---|---|---|
| A (idle) | off | 0/40 | 0.0 % [0.0, 8.8] | 0 | 0 | 0 |
| B (500 Hz stream, platform holding) | on | **15/40** | **37.5 % [22.7, 54.2]** | **0** | **7** | **8** |
| A2 (idle) | off | 0/40 | 0.0 % [0.0, 8.8] | 0 | 0 | 0 |

Cross-arm counters (post-armB firmware snapshot, boot-cumulative):
`defer jb = 16` (15 hand deferrals + 1 non-hand jb deferral, localised to
dispatch 25's interval — 0-based CSV `i`, 1-based #26), `defer bb = 0`,
`defer cone = 0`; `txq jb high-water = 2` (bb 0, cone 0 — watermark, not a count);
**`can3_errors` all 15 fields zero in all 127 logged rows** — the validity
condition held. (127 = the three arms' 3 × 40 **plus** the 7 rows of the aborted
run below. Those 7 are pooled for the wire-error check and for boot invariants
like `bridge_fw_version` **only** — a wire-error count is a property of the bus
during the logging window whether or not a dispatch reached the Teensy. They are
NEVER pooled into a rate; every rate above is out of 40.)
Host `hand_traj_acks`: `calls = 80, ok = 65, fail_teensy = 15, fail_host = 0`,
agreeing exactly with the firmware's own counters (26/26 cross-layer identities
pass). `rej = 0`, `busdown = 0` throughout.

Fisher exact, armB vs pooled idle (0/80): **p = 8.5e-09**. The A-B-A design also
kills the warm-up/uptime confound: armA2 returns to 0/40 at a *higher* uptime
(582 s) than armA (369 s).

**Interpretation row that fired — row 1: "B fail-rate >> A, A ≈ A' ≈ low" ⇒
congestion CONFIRMED.** The `tx_deferred ≈ 3 × (per-stage fail sum)` clause in
that row's wording is wrong as written and should be read as satisfied: a failing
dispatch aborts at its failing stage, so it contributes exactly **one** deferral,
not three — and the arithmetic closes exactly at 15 hand deferrals for 15 fails.

The per-stage split also fires row 4, but not in either of the two shapes that
row anticipated. Row 4's `pre1` branch — "hand never in CLOSED_LOOP, stroke never
armed" — did NOT occur at all (`pre1 = 0`). The 7 `pre2` failures are a **third**
case the row did not name: `pre1` succeeded, so the hand **IS** in CLOSED_LOOP and
energised; what failed is the `Set_Controller_Mode` send, so `hand_ops` returns
before the 0x6D0 frame and **no stroke is commanded**. The correct operational
phrasing is *"no stroke"*, not *"hand never armed"* and not *"hand de-energised"* —
the ack is truthful about the stroke, and the hand is live. The 8 `traj` failures
are row 4's armed-state-ambiguity cell (frame queued and usually transmitted, ack
lying). So both operational stories run at roughly 50/50 and the ladder can now
tell them apart per dispatch. Row 5 (A' >> A, the prime-drift signal) did **not**
fire — both idle arms are clean, so the bench has no purchase on that signal.

CSVs: `temp/probes/hand_dispatch_ladder_armA_2026-08-09_13-33-28.csv`,
`..._armB_2026-08-09_13-35-19.csv`, `..._armA2_2026-08-09_13-37-01.csv`.
⚠️ A fourth file, `..._armA_2026-08-09_13-33-08.csv`, is an ABORTED 7-dispatch
run (`Invalid target: -0.140 rev` — a HOST-side reject; nothing reached the
Teensy, counters stayed 0). **Do not pool it with armA.**

Full mechanism — the mailbox-occupancy readout behind the 0/7/8 split, why the
rate is 37.5 % and not the naive ~6 %, the statistics-honesty caveats, and two
side-findings in the vendored FlexCAN_T4 and a firmware comment — is in
`logbook/2026-08-02-err-timeout-attribution-instrumentation.md` § "Addendum —
2026-08-09".

**Step 4 fix — DECIDED 2026-08-09 (owner), implementation pending:**
`setMaxMB(16 → 24)` on `can_jugglebot` (8 → 16 TX mailboxes) **plus** a
console-only phase-stamp diagnostic (`micros64() - s_last_tick_us` at hand
dispatch — the addendum's own falsifiable test, shipped with the fix); chosen as
the only candidate carrying no 0x6D0 duplicate-or-invert hazard, and the one that
also makes the `events()` residual unreachable at the observed occupancy. The
latch fence stays up until that fix lands AND an ordinary reload sitting
validates it.
