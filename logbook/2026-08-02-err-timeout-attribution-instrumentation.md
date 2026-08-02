---
title: "ERR_TIMEOUT hand-path: sequences are random (state bug excluded), write() rejection is deferral not drop, and the attribution counters land"
type: investigation
date: 2026-08-02
status: in-progress
phase: "refactor-2026-07 Phase 7 / PROMPT-err-timeout-hand-path Steps 1-2"
related_plan: refactor-2026-07.md
files_changed:
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/extra_script.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hand_ops.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hand_ops.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - teensy_link/__init__.py
  - teensy_link/protocol.py
  - teensy_link/rpc_args.py
  - tests/firmware/native/test_hand_ops.cpp
  - tests/firmware/test_bridge_fw_version_xref.py
  - tests/firmware/test_firmware_build_pins.py
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/hardware/toss_trace_recorder.py
  - tests/ros/test_teensy_bridge_node_bridge_diag.py
  - tests/ros/test_teensy_bridge_node_hand_acks.py
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - logbook/2026-08-01-err-timeout-recount.md
  - logbook/2026-08-02-err-timeout-attribution-instrumentation.md
  - logbook/INDEX.md
subsystem:
  - canbridge
  - ros
tags:
  - err-timeout
  - can3
  - observability
  - forensics
---

# ERR_TIMEOUT hand-path: Step 1 sequence verdict + mechanism reframe + attribution instrumentation

Continues `2026-08-01-err-timeout-recount` (read it first). Three results this
session, in evidential order:

## 1. Step 1 — the 399 recorded outcomes are RANDOM, not structured

Ordered per-dispatch outcome sequences were reconstructed for all 16 sessions
with hand dispatches (2026-07-23 → 07-31) from `~/.ros/log/*/launch.log`,
validated cell-for-cell against the recount table (pooled prime 133/63 fail,
arm 266/139 fail — 202/399 = 50.6 %). Arm-path ordering uses a
single-outstanding-channel reconstruction justified from
`catch_coordinator_node.py:1171-1172` (a same-ball re-arm can only follow the
previous arm's failure callback) and validated empirically: in the codes-visible
era, 97/97 arm dispatches have visible responses with 0 interleavings, and 167
measured response→next-dispatch margins are all positive (min +3.5 ms).

Verdict (independently re-derived, adversarially attacked, pairing-invariant):

- **No alternation** (arm z = +0.54, prime z = +0.19), **no parity effect**,
  **no lag-1 state signature** (P(F|F) vs P(F|O): OR 0.82 arm / 0.76 prime,
  both ≈ p 0.45; retry-vs-first OR ≈ 0.9) — the stale-pending-slot / state-bug
  class is EXCLUDED at the effect sizes that matter (caveat: n=84 short-lag
  attempts cannot exclude modest effects, OR ≤ 1.7).
- **No genuine time-clustering** once within-retry-chain pairs are fully
  removed (0 excess FF pairs at W=1 s), and **session rates are homogeneous**
  (X² 11.03/15 df, dispersion 0.74) across fresh-to-30 h bridge uptimes.
- Prime (47.4 %) and arm (52.3 %) rates are statistically compatible
  (p = 0.40); under the code-verified 3-send structure (`hand_ops.cpp:55/59/73`)
  independence implies ~21 %/frame. (The exponent check k=3 is circular from
  rates alone — k=3 is verified in source, not fitted.)
- Random ⇒ **congestion-consistent**: exactly what per-dispatch-independent
  TX-mailbox pressure would produce, and what a state machine would not.

Per-session ordered sequences (dispatch order, O=ok F=fail) — durable copy so
re-analysis never needs the extraction again:

| session | prime seq | arm seq |
|---|---|---|
| 07-23 09:13 | OOOFFOFFFOOOFF | FFOFOOOOFO |
| 07-23 11:37 | OFOFFOOOFFOOOFOOFOOF | FFFOOOFOOFFFO |
| 07-23 17:51 | FOOOFFOOOFOFOOOOFFFO | FOOFFOOFFOFOOOOFFFOOFFO |
| 07-24 09:07 | FOOFOFOOFOOOOFOFFOFFFFFFOFOFOFOFF | FOFOFOFOFOFFOOFFOFOFFFFOFOFOOFFOFOOOFOOFFFOFFFFFFFO |
| 07-25 15:04 | - | FF |
| 07-25 15:17 | OFO | FOFOOFOOOFO |
| 07-25 15:22 | - | OO |
| 07-27 15:23 | - | O |
| 07-27 15:37 | - | FO |
| 07-27 15:39 | FFOOOOOFOOFFOOFFOOFOFOOFF | FFOFOOOOFFOOFFFOOFOOFFFFOFOFOFOFFFOOOOFFOOFFOFFOFFOFF |
| 07-27 15:51 | - | FOOOOFFFOFFFFOFFOOOOFOOOOOFF |
| 07-27 16:00 | - | OOFFOFFOOFFFOFFFO |
| 07-27 16:07 | - | OOOFF |
| 07-29 11:10 | OOOFFFF | FOOFFFFFFOFOOFFOFOOOFOFFOOFOOFOFFFFFFO |
| 07-31 13:56 | FFO | OO |
| 07-31 14:51 (post-swap) | FOFOOFFO | FOFFOFOO |

Two secondary signals, recorded honestly:
- **A real, unexplained within-session drift on the PRIME path only**
  (+0.62–0.77 log-odds/100 s, p 0.006–0.015, wall-clock-shaped not
  call-count-shaped, robust to retry removal and leave-one-session-out) while
  the arm path is flat and session MEANS stay homogeneous. Unresolved — the
  bench A/A' arms below are the cheap test.
- A boundary-significant same-reload-cycle prime↔arm association (MH OR 2.79,
  Fisher p 0.05–0.09 depending on cycle definition) — consistent with shared
  time-local pressure, under-determined at this n.

## 2. Mechanism reframe — `bus.write() <= 0` is DEFERRAL, not drop [VERIFIED IN SOURCE]

The recount narrowed the mechanism to `bus.write() <= 0` and inferred
"write rejected ⇒ frame never enqueued ⇒ hand NOT armed", placing it in open
tension with the catch latch's lying-ack premise. Reading the vendored
FlexCAN_T4 (`~/.platformio/.../FlexCAN_T4.tpp`, orchestrator-verified line by
line):

- `write(msg)` returns ONLY 1 (mailbox written) or **-1 = pushed into the
  64-slot software txBuffer** (`struct2queueTx`, :1024-1048). Never a drop at
  the call site. `send_on()`'s `> 0` maps -1 to false.
- The TX-complete ISR drains the ring correctly, one frame per freed mailbox,
  FIFO order (:1243-1256) — TX mailbox interrupts are enabled (8 TX mailboxes,
  MB8-15, `setMaxMB(16)` + FIFO). A deferred frame goes out ~0.1–1 ms later.
- Two genuine loss paths exist and are narrower: ring overflow silently
  overwrites the oldest entry (`circular_buffer.h:340-349`), and the `events()`
  TX drain has a real defect (peeks one frame, writes it to EVERY free mailbox,
  pops one entry per write → duplicates the head, discards the rest;
  :1084-1102) — it only runs when the 1 kHz tick beats the ISR to a non-empty
  ring.

**Consequence: the recount's "hand NOT armed" inference is withdrawn** (addendum
added there). A failed ack with the frame transmitted ~1 ms later is exactly
the lying-ack field observation — the two premises are COMPATIBLE. This is
source-reading, not measurement: **the latch fence stays up** until the
counters confirm on hardware. It also shifts the epidemic's likely operational
cost from "lost stroke" to "the FSM's reaction to a false-failure ack"
(aborted reloads, ladder churn) — a Step 4 question, explicitly not acted on.

Open gap the counters must close: naive phase-uniform burst arithmetic with 8
TX mailboxes predicts well under 50 % per-call failure — either arrivals are
phase-correlated with the leg tick, effective TX capacity is lower than
modeled, or something else adds pressure. Do not paper over this.

## 3. What landed

- Host-side (commit 1 of this entry's arc) — `hand_traj_acks` row (calls/ok/fail_teensy/fail_host)
  at the `_call_rpc` choke point; splits Teensy-returned from host-synthesised
  ERR_TIMEOUT (the pooling the recount unpicked by hand); closes the 20d01e9
  forensics blind spot against the CURRENT bridge. `/link_status` 37→38 keys.
- Build guard (commit 2) — pio staleness guard: sha256 of `udp_protocol.h` vs a BUILD_DIR
  marker; wipes project objects + link products on mismatch (measured: full
  BUILD_DIR rmtree breaks the run — library Dir nodes already materialised).
  Closes the 24608bb class for ADDITIVE changes, where it is silent.
- Wire + firmware (commit 3, lands with this entry) — BRIDGE_TX_DIAG (0x8D) + BRIDGE_IDENTITY (0x8E), FW_VERSION
  8→9, no PROTOCOL_VERSION bump: per-bus `tx_deferred` + `tx_q_hwm` (sampled at
  send instants inside the PRIMASK section), per-stage hand counters
  (calls/rej_homing/bus_down/pre1/pre2/traj), wire-visible bridge firmware
  identity with warn-never-refuse BRIDGE_FW_CHECK. `/link_status` 38→40.
  Independently audited pre-commit: COMMIT-READY, zero blocking findings;
  the six polish notes were applied before the commit was written.

## Discussion

**Why the counter is `tx_deferred`, not the planned `tx_write_fail`.** The name
in the recount/plan encoded the "rejected ⇒ lost" model this session refuted.
A wire-format field name outlives its investigation; naming it for the verified
mechanism (deferral into the software ring) prevents the next reader from
re-deriving the wrong operational story. The two REAL loss paths (ring
overflow; the events() drain defect) are named in the field's comment instead.

**Why per-stage cumulative counters, not new RpcStatus codes.** Distinct codes
(ERR_HAND_PRE1/...) would give per-call attribution in the launch log — but
they mutate the semantics of an existing wire field consumed by live logic
(catch ladders key on failure acks; old-host `== ERR_TIMEOUT` comparisons stop
matching under skew; the recount's own regexes break). The additive contract
exists precisely to keep cross-version semantics frozen. Rates per stage are
what Steps 3-4 decide on; per-call joins are deferred (0x8F remains free).

**Why two MsgTypes, not one.** Identity and diagnostics have different
lifecycles: the diag payload will plausibly evolve (each evolution = a new
MsgType under the never-append rule), while identity is a stable contract with
its own consumer (the xref-pinned expected-version check). Bundling would
couple the stable contract to the evolving one's id churn.

**Why the discriminator drives the REAL stack** (Step 3 runbook): a synthetic
UDP client would need hand-crafted ArgHandTraj payloads (deadline semantics =
stroke risk), fights the single-owner UDP constraint, and validates a path
production never takes. `smooth_move_hand` to the current position is a
mechanically-null dispatch through the identical 3-send firmware path, with
the counters read off `/link_status`.

## Open questions

1. The prime-path within-session drift (above) — bench A/A' arms test it.
2. The ~50 % vs naive-burst-math gap — tx_q_hwm + Step 3 arms discriminate.
3. tx_q_hwm is boot-monotone with no reset RPC — bench arms must run
   low-load-first and read per-arm increments, or reboot between arms; a
   pinned 64 from an earlier phase is NOT evidence about the current one.
4. txBuffer loss paths are un-instrumentable without touching the vendored
   library — tx_q_hwm==64 is the overflow proxy; the events() drain defect is
   a candidate Step 4 item ONLY if attribution shows real frame loss.
5. One ordinary post-swap reload sitting (n up from 4/8) — unchanged from the
   recount; now doubles as the counters' first field data.

## Verification

- Step 1: extraction validated 16/16 sessions cell-for-cell vs the recount
  table; independent re-derivation 0/16 mismatch; raw-log spot-check on
  07-23 17:51 + 07-31 14:51 exact incl. µs timestamps. Scripts (one-off, probe
  policy): session scratchpad `seq_extract.py`, `seq_analysis_*.py`,
  `seq_verify.py`; durable numbers are in this entry.
- Commit 1: ./run_tests.sh, run 2026-08-02: 3988 passed, 195.40 s parallel,
  serial phase empty, RESULT PASS. New 5-test file drives the real RpcClient
  against the FakeTeensy; mutation-checked.
- Commit 2: pio run -e teensy41 and -e teensy41_bench_sysid each ×2, run
  2026-08-02: guard fires once then stays quiet, both SUCCESS; ./run_tests.sh,
  run 2026-08-02: 3989 passed, 189.34 s parallel, RESULT PASS.
- Commit 3: `pio run -e teensy41` and `pio run -e teensy41_bench_sysid`, run
  2026-08-02: both **SUCCESS** (7.31 s / 7.40 s); the pio staleness guard fired
  once per env on each post-regen build (`udp_protocol.h CHANGED
  2198631f6813 -> ... -> 81db5186babd`, wiping `$BUILD_DIR/src` + link
  products) and stayed quiet on rebuilds with an unchanged header — commit 2
  exercised on its first real trigger, twice, since a late wording fix to a
  payload summary regenerated the header again.
  `pytest tests/firmware -q`, run 2026-08-02: **399 passed in 192.91 s** (389
  before this arc). `./run_tests.sh`, run 2026-08-02: **4009 passed, parallel
  phase; serial phase empty (4444 deselected); RESULT: PASS**.
  Independent pre-commit audit: COMMIT-READY, zero blocking
  findings; six polish items applied before the commit: (1) `getTXQueueCount`
  is a single member load, not two; (2) the HWM sample means different things on
  the deferral vs mailbox paths; (3) the torn-read note on a transient `ok=-1`
  (left unclamped — a PERSISTENT negative is real signal); (4) the RX-thread
  logging deviation recorded explicitly; (5) `proto` given a read site in the
  row; (6) the announce-on-change test re-synchronised on `rx_count_by_type`
  (its old wait predicate was already satisfied by frame 1, so the repeats were
  never witnessed as arrived and the assertion proved nothing).
  A SECOND, narrative-only audit of this entry then caught three more, all
  mine: a suite count of 4020 that had been written from expectation rather
  than measurement (the real figure is 4009 — the run had not happened yet when
  the sentence was drafted), a wrong pre-arc firmware-tier baseline, and an
  INDEX row with six columns against a five-column header, which silently drops
  the entry link under GFM. Recorded because the first two are the failure mode
  the (date, command, result) rule exists to prevent, and it still got past a
  first pass.
- Session closure: [close-out commit adds the ./run_tests.sh --full triple]
