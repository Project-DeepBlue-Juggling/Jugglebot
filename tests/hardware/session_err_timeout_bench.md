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

Ladder script: tools/probes/archived/hand_dispatch_ladder.py (archived 2026-08-09 with the arc CLOSED; still runnable for regression checks) (rclpy; prints per-dispatch
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

**Step 4 fix — DECIDED 2026-08-09 (owner), IMPLEMENTED 2026-08-09 as FW 10;
FLASHED AND VALIDATED the same day — see § Results below:** `setMaxMB(16 → 24)` on `can_jugglebot` (8 → 16 TX
mailboxes) **plus** a console-only phase-stamp diagnostic (`micros64() -
interp_last_tick_us()` at hand dispatch — the addendum's own falsifiable test,
shipped with the fix); chosen as the only candidate carrying no 0x6D0
duplicate-or-invert hazard, and the one that also makes the `events()` residual
unreachable at the observed occupancy. The latch fence stays up until that fix
lands AND an ordinary reload sitting validates it. Post-fix procedure below.

## Post-fix validation — FW 10 (procedure; RAN 2026-08-09 — results below)

Same experiment, same script, one new readout. Re-running the identical three-arm
ladder is the point: it is the only design that already killed the warm-up/uptime
confound, and a single loaded arm on its own could not.

0. Preconditions:
   - **BEFORE flashing**, capture the pre-fix baseline on the FW 9 board: one
     `/profile` sample with the 500 Hz leg stream running (`can1_tx`,
     `interp_max_jitter_us`, `interp_deadline_misses`). Two of the rows below are
     before/after comparisons and the "before" is unobtainable once FW 10 is on
     the board.
   - Flash the bridge to FW 10 (`pio run -e teensy41 -t upload` from
     `Teensy_code_canbridge` — **operator runs the flash**). FW 10 is
     WIRE-INVISIBLE (no MsgType, no payload, PROTOCOL_VERSION still 5), so a
     healthy link proves nothing about which build is aboard: confirm
     `bridge_fw_version` reads `10 (proto 5)` and `BRIDGE_FW_CHECK` is OK before
     believing any number from this session.
   - No `colcon build` is needed for the firmware half; `teensy_link/` runs live
     from the tree, so the bumped `EXPECTED_BRIDGE_FW_VERSION` is already in
     effect. (A `colcon build` is still needed if `teensy_bridge_node.py` moved.)
   - Reboot the bridge Teensy (standing rule), log `uptime_ms` per block.
1. **Open a pio device monitor for arm B**: `pio device monitor -e teensy41` from
   `Teensy_code_canbridge`, to read the 1 Hz `[handphase]` lines. **Open it only
   AFTER the flash** — the monitor holds `/dev/ttyACM0` and a flash with it open
   fails. Close it before any re-flash.
2. Run arms A / B / A2 exactly as above (N = 40, ≥ 2 s apart, zero-distance
   smooth moves; arm B with the 500 Hz leg stream and the platform holding).

Pre-registered reads — decide before looking:

| Observable | PASS | What a miss means |
|---|---|---|
| arm B failure rate | **0/40** (was 15/40) | any failure at all ⇒ 16 mailboxes was not enough, or the mechanism is not purely occupancy |
| `defer jb` increment across arm B | **0** (was +16) | non-zero with 0 hand failures ⇒ a non-hand producer is still deferring; read `tx_q_hwm_jb` too |
| arms A / A2 | 0/40, unchanged | a regression introduced by the mailbox change would show here first |
| `can3_errors` | all 15 fields zero | any wire-error tick invalidates the run, exactly as in the pre-fix session |
| `[handphase]` phase_us | **two tight clusters** ⇒ § A3's phase-quantisation verdict CONFIRMED | a ~uniform spread over 0–2000 ⇒ verdict **REFUTED**; the fix may still work, but the *explanation* is wrong and the occupancy story re-opens. Record this honestly even if arm B is 0/40 — a green fix is not evidence for the model that predicted it |
| **jugglebot bus TX fps** — `/profile` key `can1_tx` (wire slot 1 = jugglebot role; `teensy_bridge_node.py:3114`) | **~3150 fps, unchanged** vs a pre-fix boot | a DROP means the MAXMB arbitration-scan hazard is real: FlexCAN scans all enabled mailboxes each arbitration round and the RM notes the scan duration scales with MAXMB, which can cost a back-to-back transmission slot. This is the **one RM-unverified inference** in the whole fix. It degrades bus THROUGHPUT, not ISR timing, so **interp jitter would not show it** — capture `can1_tx` on a pre-fix boot before flashing, or the comparison is unavailable afterwards |
| `interp_max_jitter_us` / `interp_deadline_misses` | within their pre-fix budget | the one place the longer `write()` mailbox scan could cost the 500 Hz loop (it runs inside the existing PRIMASK region). The arbitration-scan hazard is the row above, and this row cannot see it. Capture both before and after on the same boot |

Note `[handphase]` prints on-change and reports `+N` since the last line, so an
N > 8 burst is visible as an overrun rather than silently truncated to the 8-deep
ring. At the ladder's ≥ 2 s spacing against a 1 Hz diag tick, expect `+1` lines.

ABORT criteria are unchanged from the pre-fix session. A PASS here clears the
bench half only — the latch fence comes down after an ordinary reload sitting.

## Results — post-fix, FW 10, 2026-08-09 (afternoon)

Run on FW 10 (`bridge_fw_version` = `10 (proto 5)` in all 120 logged rows — the
check that matters, since FW 10 is wire-invisible and a healthy link proves
nothing about which build is aboard). Bridge uptime 123 s at armA start → 393 s
at armA2 end, one boot across all three arms. N = 40 dispatches per arm, ≥ 2 s
apart, zero-distance smooth moves — identical to the pre-fix session.

| arm | leg stream | fails | rate (95 % CP) | pre1 | pre2 | traj | pre-fix |
|---|---|---|---|---|---|---|---|
| A (idle) | off | **0/40** | 0.0 % [0.0, 8.8] | 0 | 0 | 0 | 0/40 |
| B (500 Hz stream, platform holding) | on | **0/40** | 0.0 % [0.0, 8.8] | 0 | 0 | 0 | **15/40** |
| A2 (idle) | off | **0/40** | 0.0 % [0.0, 8.8] | 0 | 0 | 0 | 0/40 |
| pooled | — | **0/120** | 0.0 % [0.0, **3.0**] | 0 | 0 | 0 | 15/120 |

armB, the load-bearing arm: **15/40 → 0/40**, Fisher exact **p = 1.21e-05**
(0/120 pooled vs the pre-fix 15/40: p = 9.0e-11). Under the pre-fix rate of
0.375, a clean 40 has probability **6.8e-09**. The arm really was loaded:
per-dispatch `setpoints_sent` climbs **199 → 3327** across armB, so the 500 Hz
leg ladder was live throughout; armA and armA2 sit static (0 and 3646).

Counters as sampled on the final logged row of armA2: hand **`calls = 119,
ok = 119`**, `rej = 0, busdown = 0, pre1 = pre2 = traj = 0`;
**`defer jb = 0, bb = 0, cone = 0`**; `txq jb = bb = cone = 0` (high-water, so the
software ring was never entered on any bus, all boot). Host `hand_traj_acks`, same
final-row sampling: `calls = 119, ok = 119, fail_teensy = 0, fail_host = 0`. All 15
`can3_errors` fields zero in all 120 rows.

**Why 119 and not 120**: each row's counter snapshot precedes that row's own
dispatch, so the 120th call is not in any file — 119 is the largest readable value.
The tally of 120/120 comes from the per-row `ok` column, which is 1 on all 120 rows.

CSVs: `temp/probes/hand_dispatch_ladder_armA_2026-08-09_16-33-06.csv`,
`..._armB_2026-08-09_16-34-42.csv`, `..._armA2_2026-08-09_16-36-19.csv`.

### The pre-registered observable table — every row PASSED

| Observable | Pre-registered PASS | Measured | Verdict |
|---|---|---|---|
| arm B failure rate | **0/40** (was 15/40) | **0/40** | **PASS** — the fix's primary claim; 16 mailboxes were enough |
| `defer jb` increment across arm B | **0** (was +16) | **0** | **PASS** — and mechanistically the strongest row: no producer deferred at all, so the occupancy model's own prediction (peak pending 11 ≤ 16) holds |
| arms A / A2 | 0/40, unchanged | **0/40, 0/40** | **PASS** — no regression introduced by the mailbox change; armA2 clean at 393 s vs armA's 123 s kills the freshness confound again |
| `can3_errors` | all 15 fields zero | all zero, 120/120 rows | **PASS** — validity condition held, as in the pre-fix session |
| `[handphase]` phase_us | **two tight clusters** ⇒ § A3 CONFIRMED; uniform 0–2000 ⇒ REFUTED | **two tight clusters, ≈ 60 µs and ≈ 1060 µs, each ± 3 µs** (operator console reading during armB) | **PASS — the CONFIRM branch fired.** 1000 µs apart = the two 1 kHz FreeRTOS tick parities within the 2 ms interp cycle. This is a falsification test that could have killed the verdict with armB still at 0/40, and did not; it also **retires the PERCLK inference flag**, because non-commensurate clocks could not hold a fixed ±3 µs separation over a 78 s arm |
| jugglebot bus TX fps (`can1_tx`) | **~3150 fps, unchanged**; a DROP means the arbitration-scan hazard is real | **~3150 fps, unchanged** vs the FW 9 pre-flash baseline | **PASS — the "unchanged" branch fired**, so the one RM-unverified inference in the whole fix is **REFUTED at MAXMB = 24**. The pre-flash baseline capture is what makes this a comparison rather than a plausible-looking number |
| `interp_max_jitter_us` / `interp_deadline_misses` | within their pre-fix budget | **≤ 2 µs / 0** | **PASS** — the longer `write()` mailbox scan inside the PRIMASK region cost the 500 Hz loop nothing measurable; the ISR is untouched, as designed |

### Sitting half, and what this closes

A separate ordinary reload session followed (operator-reported, not ladder-
instrumented): **the hand was responsive on every reload attempt.** The single
abort was BB-side yaw `NOT_SETTLED` — the ball never flew, so it is not a
hand-path outcome, and it is a known separate open item. That satisfies the
"ordinary reload sitting" requirement, which existed precisely because a
null-move ladder is not a reload.

**Both halves are now green, so the ERR_TIMEOUT epidemic is CLOSED and the latch
fence comes DOWN.** Full close-out reasoning — including what is *not* claimed
(wire latency was never expected to change and was not measured to) and the one
surviving residual (the vendored FlexCAN_T4 `events()` missing-`break`, dormant
at `defer jb = 0` but with a blast radius the fix doubled) — is in
`logbook/2026-08-02-err-timeout-attribution-instrumentation.md` § "Addendum —
2026-08-09 (3)".
