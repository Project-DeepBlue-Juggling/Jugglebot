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
